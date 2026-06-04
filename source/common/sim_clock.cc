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

extern "C" void sim_clock_add_samples(uint64_t n)
{
    // ADDITIVE producer. NO LONGER on the live -x sim path (the relay-stamped
    // shared clock advances via sim_clock_set_samples on RX-bridge arrival).
    // Retained for the --test-sim-clock additive case and the disabled path.
    // fetch_add keeps it correct if a future caller adds a second producer.
    g_sim_samples.fetch_add(n, std::memory_order_relaxed);
}

extern "C" void sim_clock_set_samples(uint64_t n)
{
    // Relay-stamped shared clock (sim-arq-channel.md §10.5b): adopt the relay's
    // authoritative monotonic per-direction sample index. CAS-max so a chunk
    // that races in slightly out of order (two RX bridges, two directions) can
    // never rewind virtual time — cl_timer deltas must stay non-negative. In
    // practice there is a single live producer per process (this peer's RX
    // bridge), but max() is correct under any ordering.
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
