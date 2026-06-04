/*
 * Mercury: A configurable open-source software-defined modem.
 *
 * sim_clock_tests.cc — unit tests for the -x sim virtual clock (Inc 1+2 of
 * the sim-arq-channel design). Wired via `mercury.exe --test`.
 *
 * Test (a) of the design's §3 acceptance criteria: PROVE cl_timer's elapsed
 * time tracks g_sim_samples (virtual channel time) and NOT wall-clock when
 * the sim clock is enabled, and that it falls back to real monotonic time
 * (production-identical) when disabled.
 *
 * Each test prints "  [OK] name" on pass or "  [FAIL] name: reason" on
 * failure. The function returns the total number of failed tests.
 */
#include "common/sim_clock_tests.h"
#include "common/sim_clock.h"
#include "datalink_layer/timer.h"

#include <cstdio>
#include <cstdint>
#include <thread>
#include <chrono>

namespace {

int g_failed = 0;
#define SC_CHECK(cond, name, ...) do {                         \
    if (cond) { printf("  [OK] %s\n", name); }                 \
    else { printf("  [FAIL] %s: ", name); printf(__VA_ARGS__); \
           printf("\n"); g_failed++; }                         \
} while (0)

// RAII: guarantee the global flag is restored to FALSE no matter how a test
// exits, so a test crash can never leak sim mode into a later test or into a
// process that later runs production code.
struct SimEnableGuard {
    SimEnableGuard()  { sim_clock_set_enabled(1); }
    ~SimEnableGuard() { sim_clock_set_enabled(0); }
};

// ---------------------------------------------------------------------------
// (a.1) Enabled: cl_timer elapsed tracks g_sim_samples, not wall-clock.
//
// Advance virtual time by exactly N samples and assert the timer reports
// N/48 ms (samples/48000*1000), independent of how much real time passed.
void test_timer_tracks_sim_samples()
{
    SimEnableGuard g;
    // Reset virtual clock to a known baseline. (We cannot zero g_sim_samples
    // directly — by design only sim_clock_add_samples mutates it — so we
    // measure DELTAS via two timer reads around a known advance.)
    cl_timer t;
    t.start();                       // captures startTime = current sim ns
    // Advance virtual time by 96000 samples == exactly 2000 ms of channel
    // time at 48 kHz. This is the ONLY thing that should move the clock.
    sim_clock_add_samples(96000);
    int elapsed_ms = t.get_elapsed_time_ms();
    SC_CHECK(elapsed_ms == 2000, "a1_timer_tracks_sim_samples",
             "expected 2000 ms (96000 samples / 48), got %d ms", elapsed_ms);
}

// (a.2) Enabled: wall-clock sleep does NOT advance the timer. Sleep 50 ms of
// REAL time while adding ZERO samples; the timer must read ~0 ms (it counts
// virtual time only). This is the load-bearing distinction: a wall-clock
// timer would report ~50 ms here.
void test_timer_ignores_wall_clock()
{
    SimEnableGuard g;
    cl_timer t;
    t.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));  // real time
    // no sim_clock_add_samples() — virtual time is frozen
    int elapsed_ms = t.get_elapsed_time_ms();
    SC_CHECK(elapsed_ms == 0, "a2_timer_ignores_wall_clock",
             "virtual timer advanced %d ms over a 50 ms real sleep "
             "(should be 0 — it must NOT track wall-clock)", elapsed_ms);
}

// (a.3) Enabled: monotonic + additive. Two successive 48000-sample advances
// (1000 ms each) accumulate to 2000 ms on one running timer.
void test_timer_additive_monotonic()
{
    SimEnableGuard g;
    cl_timer t;
    t.start();
    sim_clock_add_samples(48000);
    int after1 = t.get_elapsed_time_ms();
    sim_clock_add_samples(48000);
    int after2 = t.get_elapsed_time_ms();
    SC_CHECK(after1 == 1000 && after2 == 2000, "a3_timer_additive_monotonic",
             "expected 1000 then 2000 ms, got %d then %d ms", after1, after2);
}

// (a.4) ns->sample mapping is exact at the boundary. sim_clock_now_ns() must
// equal samples * 1e9 / 48000 with integer math (no float drift).
void test_now_ns_mapping_exact()
{
    SimEnableGuard g;
    uint64_t s0 = sim_clock_now_samples();
    uint64_t ns0 = sim_clock_now_ns();
    SC_CHECK(ns0 == (s0 * 1000000000ULL) / 48000ULL, "a4_now_ns_mapping_exact",
             "ns %llu != samples %llu * 1e9 / 48000",
             (unsigned long long)ns0, (unsigned long long)s0);
    sim_clock_add_samples(12345);
    uint64_t s1 = sim_clock_now_samples();
    uint64_t ns1 = sim_clock_now_ns();
    SC_CHECK(s1 == s0 + 12345 &&
             ns1 == (s1 * 1000000000ULL) / 48000ULL, "a4b_now_ns_after_advance",
             "after +12345: samples %llu (want %llu), ns %llu",
             (unsigned long long)s1, (unsigned long long)(s0 + 12345),
             (unsigned long long)ns1);
}

// (a.5) DISABLED (production) path: cl_timer falls back to wall-clock, so a
// real sleep IS measured and sim_clock_add_samples has NO effect on it. This
// proves the production timer is unchanged.
void test_disabled_uses_wall_clock()
{
    sim_clock_set_enabled(0);   // explicit: production mode
    cl_timer t;
    t.start();
    // Adding samples must NOT affect a wall-clock timer.
    sim_clock_add_samples(48000 * 100);  // 100 s of virtual time — ignored
    std::this_thread::sleep_for(std::chrono::milliseconds(30));  // real
    int elapsed_ms = t.get_elapsed_time_ms();
    // Real 30 ms sleep: allow generous slack for scheduler jitter, but it must
    // be clearly real-time-bounded (NOT the 100000 ms of virtual time we added,
    // and NOT 0).
    SC_CHECK(elapsed_ms >= 20 && elapsed_ms < 5000,
             "a5_disabled_uses_wall_clock",
             "production timer read %d ms for a 30 ms real sleep (added 100 s "
             "of virtual time that must be ignored)", elapsed_ms);
}

// (a.7) AUTHORITATIVE + MONOTONIC setter (the Q3 relay-stamped live path).
//   sim_clock_set_samples adopts the relay's absolute per-direction sample
//   count. After set(96000) the clock reads 2000 ms; a STALE set(48000) must
//   NOT rewind it (still 2000 ms); set(144000) advances to 3000 ms. This is the
//   invariant cl_timer consumers rely on: virtual time is monotonic
//   non-decreasing, so a slightly-out-of-order or duplicated relay chunk can
//   never produce a negative elapsed delta.
//
//   NOTE: the cross-peer system property — "the CMD ACK-timeout window and the
//   RSP reply now share ONE time base, so the commander's virtual ACK timeout
//   cannot expire ahead of the channel that carries the reply" — is a SYSTEM
//   property validated by the harness `repro_overclimb_collapse` flag
//   (sim-arq-channel.md §10 ladder), not by this in-process unit test.
void test_set_samples_authoritative_monotonic()
{
    SimEnableGuard g;
    // g_sim_samples is only ever ADDED to (no zeroing API by design), so prior
    // tests leave it at some baseline B. set_samples is ABSOLUTE, so we phrase
    // the test as DELTAS relative to a baseline captured AFTER the timer start,
    // using absolute targets above B. (A non-monotonic implementation would let
    // the stale set rewind, producing a NEGATIVE/zero elapsed delta.)
    uint64_t base = sim_clock_now_samples();
    cl_timer t;
    t.start();                                    // startTime captured at base
    sim_clock_set_samples(base + 96000);          // +2000 ms absolute -> elapsed 2000
    int after_set = t.get_elapsed_time_ms();
    sim_clock_set_samples(base + 48000);          // STALE (< current) -> ignored
    int after_stale = t.get_elapsed_time_ms();
    sim_clock_set_samples(base + 144000);         // +3000 ms absolute -> elapsed 3000
    int after_adv = t.get_elapsed_time_ms();
    // Also assert the absolute sample count never rewound on the stale set.
    SC_CHECK(after_set == 2000 && after_stale == 2000 && after_adv == 3000,
             "a7_set_samples_authoritative_monotonic",
             "expected 2000/2000/3000 ms (set/stale-ignored/advance), got "
             "%d/%d/%d ms", after_set, after_stale, after_adv);
}

// (a.6) enabled()/set_enabled() flag round-trips and leaves FALSE at the end.
void test_flag_roundtrip()
{
    sim_clock_set_enabled(0);
    bool off = (sim_clock_enabled() == 0);
    sim_clock_set_enabled(1);
    bool on = (sim_clock_enabled() != 0);
    sim_clock_set_enabled(0);
    bool off2 = (sim_clock_enabled() == 0);
    SC_CHECK(off && on && off2, "a6_flag_roundtrip",
             "flag did not round-trip: off=%d on=%d off2=%d", off, on, off2);
}

} // namespace

int run_sim_clock_tests()
{
    printf("=== sim_clock tests (Inc 1+2: virtual control-loop clock) ===\n");
    g_failed = 0;
    test_timer_tracks_sim_samples();
    test_timer_ignores_wall_clock();
    test_timer_additive_monotonic();
    test_now_ns_mapping_exact();
    test_disabled_uses_wall_clock();
    test_flag_roundtrip();
    test_set_samples_authoritative_monotonic();
    // Safety: leave production default no matter what.
    sim_clock_set_enabled(0);
    printf("=== sim_clock tests: %d failed ===\n", g_failed);
    return g_failed;
}
