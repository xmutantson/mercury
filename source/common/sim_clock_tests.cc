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
#include "datalink_layer/arq.h"   // pumped_settle_wait

#include <cstdio>
#include <cstdint>
#include <thread>
#include <chrono>
#include <atomic>

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

// ---------------------------------------------------------------------------
// (a.7) CONNECT-UNDER-LOAD ROOT-CAUSE TEST (fix/sim-connect-virtual-clock).
//
// In the two-process `-x sim` paced sim, the handshake DEADLINES are virtual
// (cl_timer reads of the sample-driven clock), but the paired SETTLE/POLL waits
// (pumped_settle_wait: the RSP turnaround arq_responder.cc:174, the CMD 50ms
// HAIL poll arq_commander.cc:585) FELL TO A VERBATIM WALL msleep. Under host CPU
// load the relay's RT pacer re-anchors and the shared virtual clock advances
// SLOWER than wall, so a wall msleep(N) overshoots its virtual budget — the
// settle desyncs from the deadline it pairs with and the RSP reply lands outside
// the CMD window (connect misses). The FIX routes pumped_settle_wait's
// two-process path through the SAME virtual clock the deadlines use.
//
// This test reproduces the desync DETERMINISTICALLY (no host load needed): a
// background thread advances the virtual clock at HALF wall-speed (48 samples
// per real ms = 1 virtual ms per 2 real ms), mimicking the relay falling behind
// wall under load. We then call pumped_settle_wait(WAIT_MS) and measure how much
// VIRTUAL time elapsed across the call (delta of cl_timer virtual reads taken
// before/after).
//
//   FAIL-BEFORE (pre-fix wall msleep): the call sleeps WAIT_MS of WALL time, and
//     at half-speed only ~WAIT_MS/2 virtual ms accrue -> virtual delta ~= 50.
//   PASS-AFTER (virtual-clock spin): the call waits for WAIT_MS of VIRTUAL time
//     -> virtual delta ~= WAIT_MS (100), measured on the SAME clock the deadline
//     uses, immune to the wall-vs-virtual rate mismatch.
//
// The assertion (virtual delta >= 90) PASSES only when the wait is governed by
// the virtual clock (the fix); the pre-fix wall msleep yields ~50 and FAILS it.
// Pump is null here (no SIM_INPROC stepper) so this exercises EXACTLY the
// two-process `-x sim` path the fix changes.
void test_pumped_settle_wait_is_virtual_clock_faithful()
{
    SimEnableGuard g;                 // sim_clock_enabled() == 1; pump stays null

    const int WAIT_MS = 100;          // virtual ms requested of pumped_settle_wait
    // Half-speed virtual clock: 48 samples == 1 virtual ms; emit every ~2 real
    // ms => virtual advances at HALF wall rate (the relay-behind-wall condition).
    std::atomic<bool> stop_pacer{false};
    std::thread pacer([&]() {
        while (!stop_pacer.load())
        {
            sim_clock_add_samples(48);   // +1 virtual ms
            std::this_thread::sleep_for(std::chrono::milliseconds(2));  // 2 real ms
        }
    });

    // Measure VIRTUAL time consumed across the call (same clock the deadlines use).
    cl_timer vt;
    vt.start();
    pumped_settle_wait(WAIT_MS);
    int virtual_delta_ms = vt.get_elapsed_time_ms();

    stop_pacer.store(true);
    pacer.join();

    // Virtual-clock-faithful => the wait spans ~WAIT_MS of VIRTUAL time. The
    // pre-fix wall msleep spans WAIT_MS of WALL time, during which only
    // ~WAIT_MS/2 virtual ms accrue (half-speed pacer) -> ~50, which FAILS this.
    SC_CHECK(virtual_delta_ms >= WAIT_MS - 10,
             "a7_pumped_settle_wait_virtual_clock_faithful",
             "pumped_settle_wait(%d) advanced only %d VIRTUAL ms (want >= %d). "
             "A wall msleep (pre-fix) returns after %d WALL ms, accruing only "
             "~%d virtual ms at half-speed -> desyncs from the virtual deadline",
             WAIT_MS, virtual_delta_ms, WAIT_MS - 10, WAIT_MS, WAIT_MS / 2);
}

// (a.8) PRODUCTION GUARD: with sim DISABLED, pumped_settle_wait is the verbatim
// wall msleep (byte-identical to stock / HW). A real WAIT_MS sleep must elapse
// in WALL time, and the virtual clock (which we spin fast in the background)
// must have NO bearing on when it returns. This proves the fix did not touch the
// production/HW path.
void test_pumped_settle_wait_production_is_wall()
{
    sim_clock_set_enabled(0);         // production / HW mode

    const int WAIT_MS = 60;
    // Spin the virtual clock FAST in the background; production must ignore it.
    std::atomic<bool> stop_pacer{false};
    std::thread pacer([&]() {
        while (!stop_pacer.load())
        {
            sim_clock_add_samples(48000);  // +1000 virtual ms per tick — ignored
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    auto t0 = std::chrono::steady_clock::now();
    pumped_settle_wait(WAIT_MS);
    auto t1 = std::chrono::steady_clock::now();
    int wall_ms = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
        t1 - t0).count();

    stop_pacer.store(true);
    pacer.join();

    // Production msleep(60): real-time-bounded (>= ~50, < 5000), NOT short-
    // circuited to ~0 by the fast virtual clock and NOT inflated to the seconds
    // of virtual time we injected.
    SC_CHECK(wall_ms >= WAIT_MS - 15 && wall_ms < 5000,
             "a8_pumped_settle_wait_production_is_wall",
             "production pumped_settle_wait(%d) took %d ms wall (want ~%d, "
             "wall-bounded). The fix must not touch the sim-disabled path.",
             WAIT_MS, wall_ms, WAIT_MS);
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
    test_pumped_settle_wait_is_virtual_clock_faithful();
    test_pumped_settle_wait_production_is_wall();
    test_flag_roundtrip();
    // Safety: leave production default no matter what.
    sim_clock_set_enabled(0);
    printf("=== sim_clock tests: %d failed ===\n", g_failed);
    return g_failed;
}
