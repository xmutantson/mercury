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
 * samples that have flowed through the channel. At 48 kHz, one sample ==
 * 1/48000 s of *channel* time. Virtual time runs as fast as the host can
 * compute the channel — decoupled from wall-clock. Every decision-path clock
 * reads this virtual time when g_sim_time_enabled is set.
 *
 * THE FIX (Q3 update, sim-arq-channel.md §10.5b — relay-stamped shared clock)
 * --------------------------------------------------------------------------
 * ~~Originally virtual time was sourced LOCALLY: each peer's rx_transfer
 * (prep-thread demod cadence) called sim_clock_add_samples per consumed chunk,
 * so the two peers each accumulated their OWN local RX cadence into
 * independent g_sim_samples counters. That left the commander's (short) MFSK
 * turnaround coupled but the (longer) OFDM data-ACK turnaround UNcoupled — the
 * commander's virtual ACK timeout elapsed before the responder's ACK travelled
 * back through the real-time relay, so over-climb->collapse never reproduced
 * (§10.4). It also let a listening peer's 5 ms idle-silence flood multiply
 * virtual time ~8.5x on the RX peer (§9.6 warp).~~
 * NOW virtual time is sourced from the RELAY: the relay maintains a monotonic
 * per-direction virtual-sample counter and prepends it as an 8-byte LE header on
 * every forwarded chunk. Each peer's RX bridge (sim_rx_bridge_thread) calls
 * sim_clock_set_samples(stamp) on chunk ARRIVAL, adopting the relay's
 * authoritative count via max(current, stamp). A peer's clock is thus driven by
 * what the channel ACTUALLY carried TO it in its RX direction — not a local
 * silence flood — so the commander's ACK-timeout window lives in the same b2a
 * timeline that carries the responder's reply (the §10.4 wall) and the §9.6
 * idle-silence warp is gone (the consumer SETs to the relay count, not ADD).
 * (sim-arq-channel.md §10.7 records why a shared SUM/MAX counter was falsified.)
 * sim_clock_add_samples is retained for the unit tests / disabled path but is
 * NO LONGER called on the live -x sim path.
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
 * sim_clock_add_samples(n) : ADDITIVE producer hook — advance virtual time by n
 *                            samples. NO LONGER on the live -x sim path (the
 *                            relay-stamped clock uses set_samples instead);
 *                            retained for the --test-sim-clock additive case
 *                            and as a documented no-op on the disabled path.
 * sim_clock_set_samples(n) : AUTHORITATIVE producer hook for the relay-stamped
 *                            shared clock. The RX bridge calls this with the
 *                            relay's monotonic per-direction sample index (the
 *                            END index of the arriving chunk). MONOTONIC: a
 *                            stale/reordered stamp can never rewind virtual time
 *                            (keeps max(current, n)). This is the live advance
 *                            path under -x sim so both peers adopt the SAME
 *                            channel timeline instead of each accumulating its
 *                            own local RX cadence (sim-arq-channel.md §10.4/§10.5b).
 * sim_clock_now_samples()  : current virtual sample count.
 * sim_clock_now_ns()       : current time in nanoseconds. Virtual when
 *                            enabled, else CLOCK_MONOTONIC_RAW. The single
 *                            entry point every decision-path clock funnels
 *                            through.
 * sim_clock_fill_timespec(ts) : write sim_clock_now_ns() into *ts as a
 *                            monotonic timespec (the cl_timer call shape). */
int      sim_clock_enabled(void);
void     sim_clock_set_enabled(int enabled);
void     sim_clock_add_samples(uint64_t n);
void     sim_clock_set_samples(uint64_t n);
uint64_t sim_clock_now_samples(void);
uint64_t sim_clock_now_ns(void);
void     sim_clock_fill_timespec(struct timespec *ts);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* INC_SIM_CLOCK_H_ */
