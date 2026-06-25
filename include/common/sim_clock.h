/*
 * Mercury: A configurable open-source software-defined modem.
 *
 * sim_clock.h — virtual ("sim") time source for the -x sim device-free
 * channel backend (see fact-documents/sim-arq-channel.md).
 *
 * MOTIVATION
 * ----------
 * The -x sim backend drives the full ARQ control loop through a software
 * channel relay with NO audio hardware, so a gearshift/ARQ fix can be
 * validated in minutes instead of a ~3h hardware re-measure. But the modem's
 * control-loop clocks are wall-clock gated:
 *   - cl_timer (timer.cc) reads CLOCK_MONOTONIC_RAW: gates PTT delays,
 *     ACK/HAIL/BREAK timeouts, retransmit timers.
 *   - opt_now_ms() (arq.h) reads std::chrono::steady_clock: the Q-table
 *     effective-rate optimizer's bytes/time measurement.
 * Because audio is relayed over localhost TCP at the modem's own pace, those
 * wall-clock reads still tick at ~1x real time -> the sim runs no faster than
 * a radio. WORSE: if the relay is faster than real time but the clocks are
 * not, the optimizer measures bytes/wall-time and reports a rate ~50x too high
 * -> garbage Q-table configs (the §5 landmine in the design doc).
 *
 * THE FIX
 * -------
 * A single virtual clock whose time is derived from the number of audio
 * samples that have flowed through the RX boundary (rx_transfer). At 48 kHz,
 * one sample == 1/48000 s of *channel* time. The RX bridge advances the sample
 * counter as fast as the relay can deliver chunks, so virtual time runs as
 * fast as the host can compute the channel — decoupled from wall-clock. Every
 * decision-path clock reads this virtual time when g_sim_time_enabled is set.
 *
 * PRODUCTION SAFETY
 * -----------------
 * g_sim_time_enabled defaults to FALSE and is set true ONLY in the "-x sim"
 * branch of main.cc. When false, sim_clock_now_ns() returns
 * clock_gettime(CLOCK_MONOTONIC_RAW) — byte-for-byte the same source the
 * stock cl_timer used. Every non-sim mode is unchanged.
 *
 * DUAL-LANGUAGE
 * -------------
 * This header is included from C++ (timer.cc, arq.h, main.cc) AND from C
 * (audioio.c, gcc -std=c17). The shared atomic counter is defined in
 * sim_clock.cc (C++); the C side touches it ONLY through the C-linkage
 * accessor functions declared below.
 */
#ifndef INC_SIM_CLOCK_H_
#define INC_SIM_CLOCK_H_

#include <stdint.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Master sample rate of the passband audio boundary (Hz). One -x sim sample
 * == 1/SIM_CLOCK_SAMPLE_RATE_HZ seconds of virtual channel time. */
#define SIM_CLOCK_SAMPLE_RATE_HZ 48000

/* C-linkage view, shared by the C audioio.c producer and any C consumer.
 * Implemented in sim_clock.cc over a std::atomic<uint64_t>.
 *
 * sim_clock_enabled()      : non-zero when -x sim virtual time is active.
 * sim_clock_set_enabled(e) : flip the flag (called once from main.cc).
 * sim_clock_wire_stamp()   : non-zero when --wire-stamp relay-stamped clock is
 *                            active (2-process -x sim only). Selects the stamped
 *                            RX wire format + the SET clock. SIM_INPROC / --test
 *                            leave it 0 (bare wire + ADD clock).
 * sim_clock_set_wire_stamp(on) : flip the wire-stamp gate (once, from main.cc).
 * sim_clock_add_samples(n) : ADDITIVE producer hook — advance virtual time by
 *                            n samples (rx_transfer per RX chunk on the
 *                            non-wire-stamp / SIM_INPROC / --test paths).
 * sim_clock_set_samples(n) : RELAY-STAMPED producer hook (--wire-stamp) —
 *                            adopt the relay's authoritative per-direction
 *                            END-sample stamp. CAS-max: virtual time only ever
 *                            moves FORWARD (never rewinds). See sim_clock.cc.
 * sim_clock_now_samples()  : current virtual sample count.
 * sim_clock_now_ns()       : current time in nanoseconds. Virtual when
 *                            enabled, else CLOCK_MONOTONIC_RAW. The single
 *                            entry point every decision-path clock funnels
 *                            through.
 * sim_clock_fill_timespec(ts) : write sim_clock_now_ns() into *ts as a
 *                            monotonic timespec (the cl_timer call shape). */
int      sim_clock_enabled(void);
void     sim_clock_set_enabled(int enabled);
int      sim_clock_wire_stamp(void);
void     sim_clock_set_wire_stamp(int on);
void     sim_clock_add_samples(uint64_t n);
void     sim_clock_set_samples(uint64_t n);
uint64_t sim_clock_now_samples(void);
uint64_t sim_clock_now_ns(void);
void     sim_clock_fill_timespec(struct timespec *ts);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* INC_SIM_CLOCK_H_ */
