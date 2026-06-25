/*
 * Mercury: A configurable open-source software-defined modem.
 *
 * sim_clock.cc — definitions for the -x sim virtual time source.
 * See include/common/sim_clock.h for the design rationale.
 */
#include "common/sim_clock.h"

#include <atomic>
#include <chrono>

// Defaults: virtual time DISABLED. Production (-x wasapi/alsa/...) never flips
// this, so sim_clock_now_ns() always falls through to CLOCK_MONOTONIC_RAW and
// behavior is byte-identical to the pre-sim-clock cl_timer.
static std::atomic<int>      g_sim_time_enabled{0};
static std::atomic<uint64_t> g_sim_samples{0};

// --wire-stamp gate. Defaults to FALSE. Set true ONLY by the modem's
// --wire-stamp CLI flag (main.cc), which ONLY the 2-process -x sim harness
// passes. It selects (1) the stamped RX-bridge wire format (read 8-byte LE
// stamp ahead of each chunk) and (2) the relay-stamped SET clock instead of
// the demod-consumption ADD clock. Keyed SEPARATELY from sim_clock_enabled()
// because SIM_INPROC and --test-sim-clock ALSO enable the sim clock but have
// NO relay / NO stamps — they must keep the bare-chunk wire + ADD clock.
static std::atomic<int>      g_sim_wire_stamp{0};

extern "C" int sim_clock_enabled(void)
{
    // Relaxed: the flag is set ONCE at startup (main.cc, before the bridge
    // threads exist) and only read thereafter — no cross-thread ordering to
    // establish against other state.
    return g_sim_time_enabled.load(std::memory_order_relaxed);
}

extern "C" void sim_clock_set_enabled(int enabled)
{
    g_sim_time_enabled.store(enabled ? 1 : 0, std::memory_order_relaxed);
}

extern "C" int sim_clock_wire_stamp(void)
{
    return g_sim_wire_stamp.load(std::memory_order_relaxed);
}

extern "C" void sim_clock_set_wire_stamp(int on)
{
    // Set ONCE at startup from main.cc's --wire-stamp arg, BEFORE the bridge
    // threads exist. Read by the RX bridge (wire format) and rx_transfer (add
    // gate). Relaxed: same single-set-then-read discipline as g_sim_time_enabled.
    g_sim_wire_stamp.store(on ? 1 : 0, std::memory_order_relaxed);
}

extern "C" void sim_clock_add_samples(uint64_t n)
{
    // ADDITIVE producer. On the LIVE two-process -x sim path WITHOUT
    // --wire-stamp this is the clock (rx_transfer demod-consumption count).
    // It is ALSO the clock for SIM_INPROC (-m SIM_INPROC pump) and the
    // --test-sim-clock additive unit cases, both of which set
    // sim_clock_enabled() but NEVER set g_sim_wire_stamp / attach a relay.
    // Under --wire-stamp the live path swaps to sim_clock_set_samples below;
    // this fetch_add then no longer runs on the 2-process path (audioio.c
    // guards the rx_transfer add on !g_sim_wire_stamp), so the two clocks
    // never double-count. fetch_add keeps it correct under any producer count.
    g_sim_samples.fetch_add(n, std::memory_order_relaxed);
}

extern "C" void sim_clock_set_samples(uint64_t n)
{
    // Relay-stamped shared clock (sim-arq-channel.md §10.5b / §11.2): adopt the
    // relay's authoritative monotonic per-direction END-sample index, delivered
    // as an 8-byte wire stamp ahead of each RX chunk (audioio.c sim_rx_bridge,
    // --wire-stamp mode). Both peers SET their virtual clock to this ONE relay
    // timeline, so the CMD ACK-timeout window and the RSP reply share a single
    // clock regardless of host (wall) speed — the Linux FTRT drift that
    // desynced CONNECT is eliminated.
    //
    // CAS-max so virtual time can ONLY move FORWARD: a stamp that races in
    // slightly out of order, duplicates, or arrives late can never rewind the
    // clock. cl_timer deltas (timer.cc) MUST stay non-negative — a rewind would
    // make get_elapsed_time_ms() return a negative/huge value and a window would
    // never expire (a hang replacing the connect-timeout). In the 2-process
    // path there is a single live producer per process (this peer's RX bridge),
    // but max() is correct under any ordering and any future second producer.
    uint64_t cur = g_sim_samples.load(std::memory_order_relaxed);
    while (n > cur &&
           !g_sim_samples.compare_exchange_weak(cur, n,
               std::memory_order_relaxed, std::memory_order_relaxed))
    {
        // cur is reloaded by compare_exchange_weak on failure; loop until we
        // either win the CAS or observe n <= cur (a newer/equal stamp landed).
    }
}

extern "C" uint64_t sim_clock_now_samples(void)
{
    return g_sim_samples.load(std::memory_order_relaxed);
}

extern "C" uint64_t sim_clock_now_ns(void)
{
    if (sim_clock_enabled())
    {
        // Virtual time: samples * (1e9 / 48000) ns. Integer math avoids drift;
        // 1e9 % 48000 != 0 so we keep full precision by multiplying first.
        // samples * 1000000000 overflows uint64 only past ~5.8e11 samples
        // (~140 days of channel time) — far beyond any sim run.
        uint64_t s = g_sim_samples.load(std::memory_order_relaxed);
        return (s * 1000000000ULL) / (uint64_t)SIM_CLOCK_SAMPLE_RATE_HZ;
    }
    // Production / non-sim fallback. Portable monotonic source (no Win32
    // clock_gettime shim dependency — that shim is file-local to timer.cc).
    // This path is reached ONLY when sim is disabled, and ONLY by callers that
    // route through sim_clock (opt_now_ms). cl_timer keeps its own
    // CLOCK_MONOTONIC_RAW read on the disabled path so the timer behavior is
    // byte-identical to pre-change; the two monotonic clocks are independent
    // and only deltas matter, so mixing them is safe.
    return (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

extern "C" void sim_clock_fill_timespec(struct timespec *ts)
{
    uint64_t ns = sim_clock_now_ns();
    ts->tv_sec  = (time_t)(ns / 1000000000ULL);
    ts->tv_nsec = (long)(ns % 1000000000ULL);
}
