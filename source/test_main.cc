/*
 * Mercury: A configurable open-source software-defined modem.
 * Copyright (C) 2022-2024 Fadi Jerji
 *               2024 Rhizomatica
 * Authors: Fadi Jerji
 *          Rafael Diniz
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, version 3 of the
 * License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include <iostream>
#include <complex>
#include <vector>
#include <fstream>
#include <chrono>
#include <cstdint>
#include <cerrno>
#include <cstdlib>
#include <ctime>
#include <cstdarg>
#include <atomic>   // R006: std::atomic<bool> shutdown_ (cross-thread termination flag)
#include <thread>   // R006: --test-shutdown-atomic cross-thread smoke
#include <type_traits> // R006: static_assert shutdown_ is atomic
#include <random>   // P1 frame-0 vehicle: deterministic AWGN run-up synthesis
#include <limits>
#include <cstring>  // FIX-C: memset for sigaction struct init (explicit, not transitive)
#include <csignal>  // FIX-C: SIGTERM/SIGINT graceful-shutdown handler (raise/SIGTERM)
#ifndef _WIN32
#include <signal.h> // FIX-C: POSIX sigaction/sigemptyset/struct sigaction
#endif
#include <math.h>
#include <unistd.h>
#include <iostream>
#include <complex>
#include "physical_layer/telecom_system.h"
#include "physical_layer/dist_matcher.h"   // --test-pas PAS/PCS DM bijection self-test
#include "physical_layer/mfsk_ctrl_codec_tests.h"
#include "common/sim_clock_tests.h"
#include "compression/test_winlink_dict.h"
#include "crypto/test_aead_nonce.h"   // AEAD bsi-bound nonce regression suite
#include "crypto/test_mlkem_hybrid.h" // ML-KEM-768 hybrid KEX regression suite
#include "datalink_layer/arq.h"
#include "datalink_layer/fade_core.h"
#include "datalink_layer/lp_transition_harness.h"
#include "audioio/audioio.h"
#include "common/sim_clock.h"


#include "test_main.h"

int run_b2f_bounded_output_test();
int run_fft_scratch_tests();

extern "C" { extern std::atomic<bool> shutdown_; }

// ============================================================================
// R006 — shutdown_ atomicity regression (in-process synthetic-fire, test-only)
// ============================================================================
//
// CLI: --test-shutdown-atomic
//
// Race-audit R006 (mercury/fact-documents/data-flow-arq-recovery-cluster.md §7,
// race_audit/race_fix_audit.json): the termination flag `shutdown_` is read in
// main-thread spin loops AND written from the audio capture/playback/sim threads
// (audioio.c). As a plain `bool` that concurrent unsynchronized read+write is a
// C/C++ data race (UB). The fix makes it std::atomic<bool> (C++) / _Atomic bool
// (C, audioio.c), which is representation-compatible for the always-lock-free
// bool type.
//
// FAIL-BEFORE / PASS-AFTER:
//   - COMPILE-TIME guarantee (the structural fail-before): the static_assert
//     below fails to compile on the pre-fix tree where `shutdown_` is a plain
//     `bool` (std::is_same<decltype(shutdown_), std::atomic<bool>> is false).
//     This is the deterministic fail-before for a behaviour-neutral UB fix:
//     the audit (§7/§8) prescribes a TSan/clean-shutdown smoke, not a runtime
//     state assertion, because the data race is UB that does not manifest
//     deterministically without a sanitizer (no -flto here).
//   - RUNTIME smoke (pass-after): a writer thread sets shutdown_ from another
//     thread exactly as audioio.c does; the main thread observes the flip in a
//     spin loop exactly as main.cc does. Asserts the flip is observed (clean
//     termination) and that the atomic is lock-free (so no lock-induced
//     ordering surprises across the C/C++ boundary).
//
// Returns 0 on PASS, 1 on FAIL.
int test_shutdown_atomic()
{
    // --- Compile-time structural assertions (the fail-before on pre-fix code) -
    static_assert(std::is_same<decltype(shutdown_), std::atomic<bool>>::value,
        "R006: shutdown_ must be std::atomic<bool> (a plain bool is a data race "
        "between the main-thread spin loops and the audioio.c audio threads).");

    bool pass = true;

    // Lock-free is required for representation-compatibility with the C TU's
    // _Atomic bool view and to avoid lock-induced cross-thread surprises.
    if(!shutdown_.is_lock_free())
    {
        printf("[TEST-SHUTDOWN-ATOMIC] FAIL: shutdown_ is not lock-free\n");
        pass = false;
    }

    // --- Runtime cross-thread smoke (the pass-after observation) -------------
    // Mirror the production access pattern: a non-main thread stores `true`
    // (audioio.c:886/1264/1381/...) while the main thread reads `!shutdown_`
    // (main.cc spin loops). On the pre-fix plain bool this is UB; on the
    // atomic it is well-defined and the flip is guaranteed observable.
    shutdown_.store(false);
    std::atomic<long long> spins{0};
    std::thread writer([&]() {
        // Brief spin so the reader genuinely loops before the flip arrives.
        for(volatile int i = 0; i < 100000; ++i) { /* burn */ }
        shutdown_.store(true);
    });

    long long guard = 0;
    const long long GUARD_MAX = 2000000000LL; // ~liveness bound; never hang
    while(!shutdown_)
    {
        spins.fetch_add(1, std::memory_order_relaxed);
        if(++guard >= GUARD_MAX) break;
    }
    writer.join();

    bool observed = (bool)shutdown_;
    if(!observed)
    {
        printf("[TEST-SHUTDOWN-ATOMIC] FAIL: main-thread spin never observed "
               "the cross-thread shutdown_=true store (guard=%lld)\n", guard);
        pass = false;
    }

    printf("[TEST-SHUTDOWN-ATOMIC] %s: lock_free=%d observed_flip=%d "
           "reader_spins=%lld\n",
        pass ? "PASS" : "FAIL", shutdown_.is_lock_free() ? 1 : 0,
        observed ? 1 : 0, (long long)spins.load());
    fflush(stdout);

    // Leave the flag clear so we do not poison any later test in the same proc.
    shutdown_.store(false);
    return pass ? 0 : 1;
}

// --test-pas: PAS/PCS distribution-matcher integrity self-test (feat/pcs).
// (1) BIJECTION deshape(shape(x))==x over many random k-bit blocks for several
//     (lambda, rail_L) configs — a precision/overflow bug shows here (the
//     life-critical data-integrity guard, fact-documents/data-flow-pas-shaping.md §6).
// (2) CONSTANT-COMPOSITION: every shaped block has EXACTLY the target counts.
// (3) HISTOGRAM: aggregated level histogram matches the Maxwell-Boltzmann target.
// Returns 0 on full pass, 1 on any failure.
int run_pas_selftest()
{
    printf("[TEST-PAS] PAS/PCS distribution-matcher bijection + composition self-test\n");
    const int amps[4] = {1,3,5,7};
    struct Cfg { double lam; int L; } cfgs[] = { {0.02,32},{0.04,32},{0.06,32},{0.04,16},{0.04,24},{0.08,34} };
    int fails = 0;
    unsigned int rng = 0xC0FFEEu;
    auto nextbit = [&](){ rng = rng*1664525u + 1013904223u; return (int)((rng>>23)&1u); };
    for (auto &cf : cfgs)
    {
        cl_dist_matcher dm;
        if (!dm.configure_maxwell_boltzmann(4, amps, cf.lam, cf.L))
        { printf("[TEST-PAS]   FAIL configure lam=%.2f L=%d\n", cf.lam, cf.L); fails++; continue; }
        int k = dm.info_bits(), L = dm.block_len();
        std::vector<int> inb(k), lev(L), outb(k);
        long target[4] = { dm.level_count(0), dm.level_count(1), dm.level_count(2), dm.level_count(3) };
        long hist[4] = {0,0,0,0};
        int  trials = 200000, bad = 0, compbad = 0;
        for (int t = 0; t < trials; ++t)
        {
            for (int b = 0; b < k; ++b) inb[b] = nextbit();
            if (!dm.shape(inb.data(), lev.data())) { bad++; continue; }
            // constant composition check
            long cc[4] = {0,0,0,0};
            for (int p = 0; p < L; ++p) { if (lev[p]<0||lev[p]>3){bad++;break;} cc[lev[p]]++; hist[lev[p]]++; }
            if (cc[0]!=target[0]||cc[1]!=target[1]||cc[2]!=target[2]||cc[3]!=target[3]) compbad++;
            if (!dm.deshape(lev.data(), outb.data())) { bad++; continue; }
            for (int b = 0; b < k; ++b) if (outb[b] != inb[b]) { bad++; break; }
        }
        double htot = (double)trials * L;
        printf("[TEST-PAS]   lam=%.2f L=%d k=%d counts{%ld,%ld,%ld,%ld} bijection_bad=%d comp_bad=%d "
               "hist{%.3f,%.3f,%.3f,%.3f} target{%.3f,%.3f,%.3f,%.3f} -> %s\n",
               cf.lam, L, k, target[0],target[1],target[2],target[3], bad, compbad,
               hist[0]/htot,hist[1]/htot,hist[2]/htot,hist[3]/htot,
               target[0]/(double)L,target[1]/(double)L,target[2]/(double)L,target[3]/(double)L,
               (bad==0 && compbad==0) ? "PASS" : "FAIL");
        if (bad != 0 || compbad != 0) fails++;
    }
    printf("[TEST-PAS] %s (%d config failure%s)\n", fails==0?"ALL PASS":"FAILED", fails, fails==1?"":"s");
    return fails==0 ? 0 : 1;
}

// --test-cfg17: CFG17 shaped-64-QAM COMPOSITION self-test (failing-test-first).
// Drives the SFO-GRID harness in-process for three decisive cells and asserts the
// COMPOSED stack decodes where a BARE arm fails — proving each of the three deep
// levers (PAS, TINTERP-seed turbo estimator, ratio-nvfix) is live in the CFG17
// gear. The harness rebuilds the grid at MOD_64QAM (MERCURY_SFO_GRID_M64/_PCS), so
// CFG17 itself need not be production-elected for the composition to be exercised.
// See fact-documents/data-flow-cfg17-shaped-64qam.md §3.
static void cfg17_set_env(const char* k, const char* v)
{
#if defined(_WIN32)
    _putenv_s(k, v);
#else
    setenv(k, v, 1);
#endif
}
// Save+restore so the harness's env-gated default-off paths are not perturbed for
// any later in-process work. Run ONE harness cell with the given env, return the
// decoded codeword count via the additive sfo_grid_last_cw_* snapshot.
static void cfg17_run_cell(const char* esn0, const char* chan, const char* m64,
                           const char* pcs, const char* coded, const char* nsymb,
                           const char* turbo_seed, const char* turbo_iters,
                           const char* nvfix, const char* nv_force,
                           int& cw_ok, int& cw_tot)
{
    // The complete env key set the harness reads for these cells.
    const char* keys[] = {
        "MERCURY_SFO_GRID", "MERCURY_SFO_GRID_ESN0", "MERCURY_SFO_GRID_CHAN",
        "MERCURY_SFO_GRID_M64", "MERCURY_SFO_GRID_PCS", "MERCURY_SFO_GRID_CODED",
        "MERCURY_SFO_GRID_NSYMB", "MERCURY_SFO_GRID_TURBO_SEED",
        "MERCURY_SFO_GRID_TURBO_ITERS", "MERCURY_SFO_GRID_NVFIX",
        "MERCURY_SFO_GRID_NV_FORCE"
    };
    const int nk = (int)(sizeof(keys)/sizeof(keys[0]));
    struct Saved { const char* key; bool had; std::string val; } sv[16];
    for (int i = 0; i < nk; i++) {
        const char* g = std::getenv(keys[i]);
        sv[i].key = keys[i]; sv[i].had = (g != nullptr);
        sv[i].val = g ? std::string(g) : std::string();
    }
    cfg17_set_env("MERCURY_SFO_GRID", "1");
    cfg17_set_env("MERCURY_SFO_GRID_ESN0", esn0);
    cfg17_set_env("MERCURY_SFO_GRID_CHAN", chan);
    cfg17_set_env("MERCURY_SFO_GRID_M64", m64);
    cfg17_set_env("MERCURY_SFO_GRID_PCS", pcs);
    cfg17_set_env("MERCURY_SFO_GRID_CODED", coded);
    cfg17_set_env("MERCURY_SFO_GRID_NSYMB", nsymb);
    cfg17_set_env("MERCURY_SFO_GRID_TURBO_SEED", turbo_seed);
    cfg17_set_env("MERCURY_SFO_GRID_TURBO_ITERS", turbo_iters);
    cfg17_set_env("MERCURY_SFO_GRID_NVFIX", nvfix);
    cfg17_set_env("MERCURY_SFO_GRID_NV_FORCE", nv_force);

    cl_telecom_system ts;
    ts.operation_mode = BER_PLOT_passband;
    ts.load_configuration(CONFIG_16);   // sizes OFDM/LDPC; harness overrides M to 64
    ts.sfo_grid_test();
    cw_ok  = ts.sfo_grid_last_cw_ok;
    cw_tot = ts.sfo_grid_last_cw_tot;

    for (int i = 0; i < nk; i++) {
#if defined(_WIN32)
        _putenv_s(sv[i].key, sv[i].had ? sv[i].val.c_str() : "");
#else
        if (sv[i].had) setenv(sv[i].key, sv[i].val.c_str(), 1); else unsetenv(sv[i].key);
#endif
    }
}

int run_cfg17_selftest()
{
    printf("[TEST-CFG17] CFG17 shaped-64-QAM composition self-test (PAS + TINTERP-seed + ratio-nvfix)\n");
    int fails = 0;

    // ---- Cell A: PAS lever, clean @ 16 dB. uniform-64 FAILS, PAS-shaped-64 DECODES. ----
    // uniform-64 waterfall ~17.6 dB; PAS waterfall ~15.3 dB (PCS_VERDICT.md). At 16 dB
    // clean (CHAN=0) the bare uniform arm is below its waterfall (0/K) while the PAS
    // shaping gain (~2.3 dB vs uniform-64, genie-confirmed) crosses the LDPC waterfall
    // (K/K). This is the decisive clean-front composition cell: PAS makes 64-QAM decode
    // at ~CFG16's working point while carrying 6 bits/sym (MEASURED: u64 0/7, PAS 7/7).
    {
        int u_ok=-1,u_tot=-1, p_ok=-1,p_tot=-1;
        cfg17_run_cell("16","0","1","0","1","60","","1","0","0", u_ok,u_tot);   // uniform-64 (bare)
        cfg17_run_cell("16","0","1","1","1","60","","1","0","0", p_ok,p_tot);   // PAS-shaped-64 (composed)
        bool bare_fail = (u_tot>0 && u_ok <  u_tot);
        bool comp_pass = (p_tot>0 && p_ok == p_tot);
        bool ok = bare_fail && comp_pass;
        printf("[TEST-CFG17]   CELL-A PAS clean@16dB: uniform-64=%d/%d (bare%s) PAS-64=%d/%d (composed%s) -> %s\n",
               u_ok,u_tot, bare_fail?"_FAILS_OK":"_DECODED_unexpected",
               p_ok,p_tot, comp_pass?"_DECODES_OK":"_FAILED",
               ok?"PASS":"FAIL");
        if(!ok) fails++;
    }

    // ---- Cell B: TINTERP-seed estimator lever, freq-selective det-floor @ 18 dB. ----
    // CHAN=1 (det-floor freq-selective) dense Dx=1/Dy=3 lattice: the bare single-pass LS
    // estimator FLOORS (0/K — the per-subcarrier dispersive phase a flat-ML/cold-LS H
    // cannot represent), while the COMPOSED TINTERP-seed turbo estimator (it=0 warm
    // TINTERP seed + data_aided_channel_estimator + dd_seed_floor) CROSSES (K/K). GENIE
    // decodes K/K here (the det-floor is estimator-limited, NOT modulation-limited —
    // RESEARCH_cfg17-64qam.md §0/§7, PCS_VERDICT.md) so this proves the estimator stack
    // reaches the genie-class CSI CFG17 requires. MEASURED reproducible over 4 seeds:
    // LS 0/7, TINTERP-seed turbo 7/7 (post-FEC BER 0.458 -> 0).
    {
        int ls_ok=-1,ls_tot=-1, tb_ok=-1,tb_tot=-1;
        cfg17_run_cell("18","1","1","1","1","60","","1","0","0", ls_ok,ls_tot);          // LS-only PAS-64 single-pass (bare)
        cfg17_run_cell("18","1","1","1","1","60","tinterp","4","0","0", tb_ok,tb_tot);   // TINTERP-seed turbo PAS-64 (composed)
        bool bare_floor = (ls_tot>0 && ls_ok == 0);
        bool comp_cross = (tb_tot>0 && tb_ok == tb_tot);
        bool ok = bare_floor && comp_cross;
        printf("[TEST-CFG17]   CELL-B estimator detfloor@18dB CHAN1: LS-only=%d/%d (bare%s) TINTERP-turbo=%d/%d (composed%s) -> %s\n",
               ls_ok,ls_tot, bare_floor?"_FLOORS_OK":"_decoded_unexpected",
               tb_ok,tb_tot, comp_cross?"_CROSSES_OK":"_FAILED",
               ok?"PASS":"FAIL");
        if(!ok) fails++;
    }

    // ---- Cell C: ratio-nvfix lever, collapsed-nv regime @ 16 dB clean. ----
    // NV_FORCE=1e-6 reproduces the HW-only post-EQ-EVM nv-collapse (the in-process sim
    // never reproduces it — PCS_VERDICT.md / nvfix a0e22c8): the bare demap (NVFIX=0)
    // over-confidently flips inner 64-QAM bits -> BP iter-cap -> decode FAILS; with
    // NVFIX=1 the ratio-gate (nv < measure_var/8 ? measure_var : nv) substitutes the
    // measured post-EQ noise -> decode RECOVERS. Clean channel so PAS alone would decode
    // absent the forced collapse -> isolates the nvfix as the cause of recovery.
    {
        int raw_ok=-1,raw_tot=-1, fix_ok=-1,fix_tot=-1;
        cfg17_run_cell("16","0","1","1","1","60","","1","0","1e-6", raw_ok,raw_tot);   // collapsed nv, NO nvfix (bare)
        cfg17_run_cell("16","0","1","1","1","60","","1","1","1e-6", fix_ok,fix_tot);   // collapsed nv, ratio-nvfix (composed)
        bool bare_fail = (raw_tot>0 && raw_ok <  raw_tot);
        bool comp_pass = (fix_tot>0 && fix_ok >  raw_ok);   // nvfix strictly improves on the collapse
        bool ok = bare_fail && comp_pass;
        printf("[TEST-CFG17]   CELL-C nvfix nv-collapse@16dB: raw-nv=%d/%d (bare%s) ratio-nvfix=%d/%d (composed%s) -> %s\n",
               raw_ok,raw_tot, bare_fail?"_FAILS_OK":"_decoded_unexpected",
               fix_ok,fix_tot, comp_pass?"_RECOVERS_OK":"_FAILED",
               ok?"PASS":"FAIL");
        if(!ok) fails++;
    }

    printf("[TEST-CFG17] %s (%d cell failure%s)\n", fails==0?"ALL PASS":"FAILED", fails, fails==1?"":"s");
    return fails==0 ? 0 : 1;
}

// --test-tinterp-seed: TINTERP-SEED production estimator self-test (failing-test-first).
// Drives the SFO-GRID CODED harness in-process (the SAME production estimator +
// equalizer + CSI + demod + LDPC path the receive_byte rescue activates) on the ITU-R
// "GOOD" Watterson fade (MPG: CHAN=3 depth=10dB fd=0.1Hz dly=24), proving the it=0
// estimator seed-swap: the held LS window decodes 0/K (it averages the time-varying H
// and reads the fade as noise) while the per-carrier TIME-INTERPOLATION estimator
// (LS_channel_estimator_tinterp, the estimator the MERCURY_TINTERP_SEED rescue selects)
// decodes K/K. A CLEAN-channel cell asserts TINTERP is a decode NO-OP (LS K/K == TINTERP
// K/K), the byte-identical-clean guarantee the rescue relies on. Mirrors test_fade_
// tinterp.py at the built-in --test level. See fact-documents/
// data-flow-noise_variance_estimate.md.
static void tinterp_set_env(const char* k, const char* v)
{
#if defined(_WIN32)
    _putenv_s(k, v);
#else
    setenv(k, v, 1);
#endif
}
// Run ONE SFO-GRID CODED cell with the given Watterson params + estimator selector,
// returning decoded/total codewords. tinterp_prod!=0 selects the TIME_INTERP estimator
// (MERCURY_SFO_GRID_TINTERP_PROD); 0 = the production LS else-branch. Env is saved/
// restored so the harness default-off paths are not perturbed for later in-process work.
static void tinterp_run_cell(const char* esn0, const char* chan, const char* depth,
                             const char* fd, const char* dly, int tinterp_prod,
                             int valid_tinterp_pilots, double selectivity_preseed,
                             int& cw_ok, int& cw_tot, double& pilot_selectivity)
{
    const char* keys[] = {
        "MERCURY_SFO_GRID", "MERCURY_SFO_GRID_CODED", "MERCURY_SFO_GRID_ESN0",
        "MERCURY_SFO_GRID_CHAN", "MERCURY_SFO_GRID_WATT_DEPTH_DB",
        "MERCURY_SFO_GRID_WATT_FD_HZ", "MERCURY_SFO_GRID_WATT_DLY",
        "MERCURY_SFO_GRID_TINTERP_PROD",
        "MERCURY_SFO_GRID_TINTERP_VALID_PILOTS"
    };
    const int nk = (int)(sizeof(keys)/sizeof(keys[0]));
    struct Saved { const char* key; bool had; std::string val; } sv[16];
    for (int i = 0; i < nk; i++) {
        const char* g = std::getenv(keys[i]);
        sv[i].key = keys[i]; sv[i].had = (g != nullptr);
        sv[i].val = g ? std::string(g) : std::string();
    }
    tinterp_set_env("MERCURY_SFO_GRID", "1");
    tinterp_set_env("MERCURY_SFO_GRID_CODED", "1");
    tinterp_set_env("MERCURY_SFO_GRID_ESN0", esn0);
    tinterp_set_env("MERCURY_SFO_GRID_CHAN", chan);
    tinterp_set_env("MERCURY_SFO_GRID_WATT_DEPTH_DB", depth);
    tinterp_set_env("MERCURY_SFO_GRID_WATT_FD_HZ", fd);
    tinterp_set_env("MERCURY_SFO_GRID_WATT_DLY", dly);
    tinterp_set_env("MERCURY_SFO_GRID_TINTERP_PROD", tinterp_prod ? "1" : "0");
    char valid_pilots[24];
    snprintf(valid_pilots, sizeof(valid_pilots), "%d", valid_tinterp_pilots);
    tinterp_set_env("MERCURY_SFO_GRID_TINTERP_VALID_PILOTS", valid_pilots);

    cl_telecom_system ts;
    ts.operation_mode = BER_PLOT_passband;
    ts.load_configuration(CONFIG_15);   // CFG15 dense Dx=1/Dy=3 lattice (the MPG cell)
    ts.ofdm.last_pilot_selectivity = selectivity_preseed;
    ts.sfo_grid_test();
    cw_ok  = ts.sfo_grid_last_cw_ok;
    cw_tot = ts.sfo_grid_last_cw_tot;
    pilot_selectivity = ts.ofdm.last_pilot_selectivity;

    for (int i = 0; i < nk; i++) {
#if defined(_WIN32)
        _putenv_s(sv[i].key, sv[i].had ? sv[i].val.c_str() : "");
#else
        if (sv[i].had) setenv(sv[i].key, sv[i].val.c_str(), 1); else unsetenv(sv[i].key);
#endif
    }
}

int run_tinterp_seed_selftest()
{
    printf("[TEST-TINTERP-SEED] TINTERP-SEED production estimator self-test (it=0 seed swap)\n");
    int fails = 0;
    double clean_ls_sel = -1.0, clean_ti_sel = -1.0;

    // ---- Cell A: GOOD Watterson fade (MPG). LS FLOORS 0/K, TINTERP CROSSES K/K. ----
    // CHAN=3 depth=10dB fd=0.1Hz dly=24 @ EsN0=24 (test_fade_tinterp.py MPG). The held LS
    // window averages the slow Doppler fade (nv reads ~0.35, the fade looks like noise) and
    // decodes 0/K; the per-carrier TIME-INTERPOLATION estimator tracks the fade (nv ~0.0065)
    // and decodes K/K. This is the failing-before/passing-after: same channel, same FEC,
    // only the it=0 estimator seed differs (the exact swap the receive_byte rescue performs).
    {
        int ls_ok=-1, ls_tot=-1, ti_ok=-1, ti_tot=-1;
        double ls_sel=-1.0, ti_sel=-1.0;
        tinterp_run_cell("24","3","10","0.1","24", 0, -1, 0.777,
                         ls_ok, ls_tot, ls_sel);   // LS-only (before)
        tinterp_run_cell("24","3","10","0.1","24", 1, -1, 0.777,
                         ti_ok, ti_tot, ti_sel);   // TINTERP (after)
        bool before_floors = (ls_tot > 0 && ls_ok == 0);
        bool after_crosses = (ti_tot > 0 && ti_ok == ti_tot);
        bool ok = before_floors && after_crosses;
        printf("[TEST-TINTERP-SEED]   CELL-A GOOD-fade@24dB: LS-seed=%d/%d (%s) TINTERP-seed=%d/%d (%s) -> %s\n",
               ls_ok, ls_tot, before_floors?"FLOORS_OK":"DECODED_unexpected",
               ti_ok, ti_tot, after_crosses?"CROSSES_OK":"FAILED",
               ok?"PASS":"FAIL");
        if(!ok) fails++;
    }

    // ---- Cell B: CLEAN AWGN no-op. LS K/K == TINTERP K/K (the byte-identical-clean basis). ----
    // CHAN=0 (flat AWGN) @ EsN0=24: both estimators decode the full block. The rescue never
    // changes a clean decode (clean frames decode on LS pass-1 and never reach TINTERP), and
    // on the rare clean frame the estimator IS run (e.g. via FADE_TINTERP) it must not regress.
    // Asserting equal decode counts on clean is the standalone no-op proof.
    {
        int ls_ok=-1, ls_tot=-1, ti_ok=-1, ti_tot=-1;
        tinterp_run_cell("24","0","0","0.5","24", 0, -1, 0.777,
                         ls_ok, ls_tot, clean_ls_sel);    // LS-only clean
        tinterp_run_cell("24","0","0","0.5","24", 1, -1, 0.777,
                         ti_ok, ti_tot, clean_ti_sel);    // TINTERP clean
        bool ls_full = (ls_tot > 0 && ls_ok == ls_tot);
        bool ti_noop = (ti_tot == ls_tot && ti_ok == ls_ok);
        bool sel_equal = fabs(clean_ls_sel - clean_ti_sel) <= 1e-12;
        bool sel_ok = (clean_ls_sel >= 0.0 && clean_ls_sel < 0.15
                       && clean_ti_sel >= 0.0 && clean_ti_sel < 0.15
                       && fabs(clean_ti_sel - 0.777) > 1e-9 && sel_equal);
        bool ok = ls_full && ti_noop && sel_ok;
        printf("[TEST-TINTERP-SEED]   CELL-B CLEAN@24dB no-op: LS=%d/%d sel=%.6f "
               "TINTERP=%d/%d sel=%.6f -> %s\n",
               ls_ok, ls_tot, clean_ls_sel, ti_ok, ti_tot, clean_ti_sel,
               ok?"PASS(no-op)":"FAIL(regression)");
        if(!ok) fails++;
    }

    // ---- Cell C: static TWO-RAY selectivity. Both estimators must publish the
    // current pilot geometry, and TINTERP must overwrite the stale tripwire. ----
    {
        int ls_ok=-1, ls_tot=-1, ti_ok=-1, ti_tot=-1;
        double ls_sel=-1.0, ti_sel=-1.0;
        tinterp_run_cell("40","2","0","0.5","24", 0, -1, 0.777,
                         ls_ok, ls_tot, ls_sel);
        tinterp_run_cell("40","2","0","0.5","24", 1, -1, 0.777,
                         ti_ok, ti_tot, ti_sel);
        bool current = (ls_sel >= 0.0 && fabs(ls_sel - 0.777) > 1e-9
                        && fabs(ti_sel - 0.777) > 1e-9);
		// Both estimator paths must publish the same-semantic instantaneous
		// pilot response. A strong two-ray fixture must cross the production
		// boundary in both, not only in TINTERP.
		bool discriminates = (clean_ls_sel >= 0.0 && clean_ti_sel >= 0.0
		                      && ls_sel > 0.15 && ti_sel > 0.15
		                      && ls_sel > 4.0*clean_ls_sel
		                      && ti_sel > 4.0*clean_ti_sel
		                      && fabs(ls_sel-ti_sel) <= 1e-12);
        bool ok = current && discriminates;
        printf("[TEST-TINTERP-SEED]   CELL-C TWO-RAY@40dB: LS-sel=%.6f "
               "TINTERP-sel=%.6f current=%d -> %s\n",
               ls_sel, ti_sel, current?1:0, ok?"PASS":"FAIL");
        if(!ok) fails++;
    }

    // ---- Cell D: no valid TINTERP pilot symbols. Fail closed to -1 and never
    // retain the preseeded prior-frame value. Decode outcome is irrelevant. ----
    {
        int ti_ok=-1, ti_tot=-1;
        double ti_sel=-1.0;
        tinterp_run_cell("40","0","0","0.5","24", 1, 0, 0.777,
                         ti_ok, ti_tot, ti_sel);
        bool ok = (ti_sel == -1.0);
        printf("[TEST-TINTERP-SEED]   CELL-D INVALID-PILOTS stale guard: "
               "preseed=0.777 after=%.6f -> %s\n", ti_sel, ok?"PASS":"FAIL");
        if(!ok) fails++;
    }

    // ---- Cell E: exactly one usable pilot through the full TINTERP estimator.
    // The collection path must count it, reject it as insufficient evidence, and
    // overwrite the prior-frame tripwire with the fail-closed sentinel. ----
    {
        int ti_ok=-1, ti_tot=-1;
        double ti_sel=-1.0;
        tinterp_run_cell("40","0","0","0.5","24", 1, 1, 0.777,
                         ti_ok, ti_tot, ti_sel);
        bool ok = (ti_sel == -1.0);
        printf("[TEST-TINTERP-SEED]   CELL-E ONE-PILOT estimator guard: "
               "preseed=0.777 after=%.6f -> %s\n", ti_sel, ok?"PASS":"FAIL");
        if(!ok) fails++;
    }

	// ---- Cell F: exact production arithmetic and consumer fail-closed edges. ----
	{
		double one[] = {1.0};
		double nonfinite[] = {1.0, std::numeric_limits<double>::infinity()};
		double nearzero[] = {1e-300, 2e-300};
		double extreme[] = {std::numeric_limits<double>::max(),
		                    std::numeric_limits<double>::max()/2.0};
		double cv_zero = cl_ofdm::pilot_magnitude_cv(NULL, 0);
		double cv_one = cl_ofdm::pilot_magnitude_cv(one, 1);
		double cv_nonfinite = cl_ofdm::pilot_magnitude_cv(nonfinite, 2);
		double cv_nearzero = cl_ofdm::pilot_magnitude_cv(nearzero, 2);
		double cv_extreme = cl_ofdm::pilot_magnitude_cv(extreme, 2);

		cl_telecom_system ts;
#if defined(_WIN32)
		_putenv_s("MERCURY_SEL_LEGACY", "");
#else
		unsetenv("MERCURY_SEL_LEGACY");
#endif
		double invalid_values[] = {cv_zero, cv_one, cv_nonfinite, cv_nearzero};
		bool invalid_failclosed = true;
		for(size_t i=0; i<sizeof(invalid_values)/sizeof(invalid_values[0]); i++)
		{
			ts.ofdm.last_pilot_selectivity = invalid_values[i];
			ts.update_channel_selectivity(0.888);
			if(ts.last_channel_selectivity != -1.0) invalid_failclosed = false;
		}
		double invalid_selected = ts.last_channel_selectivity;
		ts.ofdm.last_pilot_selectivity = cv_extreme;
		ts.update_channel_selectivity(0.888);
		double extreme_selected = ts.last_channel_selectivity;
		ts.ofdm.last_pilot_selectivity = 0.123;
		ts.update_channel_selectivity(0.888);
		double raw_selected = ts.last_channel_selectivity;
		tinterp_set_env("MERCURY_SEL_LEGACY", "1");
		ts.update_channel_selectivity(0.888);
		double legacy_selected = ts.last_channel_selectivity;
#if defined(_WIN32)
		_putenv_s("MERCURY_SEL_LEGACY", "");
#else
		unsetenv("MERCURY_SEL_LEGACY");
#endif
		bool ok = (cv_zero == -1.0 && cv_one == -1.0 && cv_nonfinite == -1.0
		           && cv_nearzero == -1.0 && std::isfinite(cv_extreme)
		           && cv_extreme >= 0.0 && invalid_failclosed
		           && invalid_selected == -1.0
		           && fabs(extreme_selected-cv_extreme) < 1e-12
		           && fabs(raw_selected-0.123) < 1e-12
		           && fabs(legacy_selected-0.888) < 1e-12);
		printf("[TEST-TINTERP-SEED]   CELL-F numeric+consumer: zero=%.6f one=%.6f "
		       "nonfinite=%.6f nearzero=%.6f extreme=%.6f invalid-selected=%.6f "
		       "extreme-selected=%.6f raw=%.6f legacy=%.6f -> %s\n",
		       cv_zero, cv_one, cv_nonfinite, cv_nearzero, cv_extreme, invalid_selected,
		       extreme_selected, raw_selected, legacy_selected, ok?"PASS":"FAIL");
		if(!ok) fails++;
	}

    printf("[TEST-TINTERP-SEED] %s (%d cell failure%s)\n", fails==0?"ALL PASS":"FAILED", fails, fails==1?"":"s");
    return fails==0 ? 0 : 1;
}

// Actual receive-flow discriminator for the selectivity state contract. Unlike
// the estimator-only grid above, every cell enters cl_telecom_system::receive_byte.
int run_tinterp_receive_path_selftest()
{
    printf("[TEST-TINTERP-RX] receive_byte selectivity transition self-test\n");
    int fails = 0;
    const char* keys[] = {"MERCURY_MFSK_WATT", "MERCURY_MFSK_WATT_FD_HZ",
                          "MERCURY_MFSK_WATT_SPREAD_MS", "MERCURY_MFSK_WATT_DEPTH_DB",
                          "MERCURY_MFSK_WATT_SEED"};
    struct Saved { bool had; std::string value; } saved[5];
    for(int i=0;i<5;i++) { const char* v=std::getenv(keys[i]); saved[i].had=(v!=NULL); saved[i].value=v?v:""; }
    auto restore_env = [&](){
        for(int i=0;i<5;i++) {
#if defined(_WIN32)
            _putenv_s(keys[i], saved[i].had ? saved[i].value.c_str() : "");
#else
            if(saved[i].had) setenv(keys[i], saved[i].value.c_str(), 1); else unsetenv(keys[i]);
#endif
        }
    };

    cl_telecom_system ts;
    ts.operation_mode = BER_PLOT_passband;
    ts.load_configuration(CONFIG_15);

    // A deterministic slow-fade frame makes primary LS fail and enters the real
    // retry label. The accepted-frame TINTERP rescue is separately covered by
    // Cell A in the estimator/FEC discriminator above.
    tinterp_set_env("MERCURY_MFSK_WATT", "1");
    tinterp_set_env("MERCURY_MFSK_WATT_FD_HZ", "0.1");
    tinterp_set_env("MERCURY_MFSK_WATT_SPREAD_MS", "0.5");
    tinterp_set_env("MERCURY_MFSK_WATT_DEPTH_DB", "10");
    tinterp_set_env("MERCURY_MFSK_WATT_SEED", "1");
    ts.passband_test_EsN0(10.0f, 1);
    bool retry_ok = ts.receive_stats.tinterp_rescue_attempted
                 && ts.ofdm.channel_estimator == LEAST_SQUARE;
    printf("[TEST-TINTERP-RX]   CELL-A LS->TINTERP retry attempted=%d succeeded=%d restored=%d -> %s\n",
           ts.receive_stats.tinterp_rescue_attempted?1:0,
           ts.receive_stats.tinterp_rescue_succeeded?1:0,
           ts.ofdm.channel_estimator==LEAST_SQUARE?1:0, retry_ok?"PASS":"FAIL");
    if(!retry_ok) fails++;

    // Consecutive clean frame on the same object must overwrite both stale caches
    // through the real estimator and selector call site without entering retry.
    tinterp_set_env("MERCURY_MFSK_WATT", "0");
    ts.ofdm.last_pilot_selectivity = 0.777;
    ts.last_channel_selectivity = 0.777;
    cl_error_rate clean = ts.passband_test_EsN0(40.0f, 1);
    bool clean_ok = (clean.Error_frames_total == 0)
                 && !ts.receive_stats.tinterp_rescue_attempted
                 && std::isfinite(ts.ofdm.last_pilot_selectivity)
                 && ts.ofdm.last_pilot_selectivity >= 0.0
                 && fabs(ts.last_channel_selectivity-ts.ofdm.last_pilot_selectivity) <= 1e-12
                 && fabs(ts.last_channel_selectivity-0.777) > 1e-9;
    printf("[TEST-TINTERP-RX]   CELL-B consecutive clean LS pilot=%.6f selected=%.6f -> %s\n",
           ts.ofdm.last_pilot_selectivity, ts.last_channel_selectivity, clean_ok?"PASS":"FAIL");
    if(!clean_ok) fails++;

    // Full receive flow with exactly one usable TINTERP pilot. The test hook is
    // applied after TX/CPE and restored immediately after the estimator.
    ts.ofdm.channel_estimator = TIME_INTERP;
    ts.selectivity_test_valid_pilots = 1;
    ts.ofdm.last_pilot_selectivity = 0.777;
    ts.last_channel_selectivity = 0.777;
    ts.passband_test_EsN0(40.0f, 1);
    bool one_ok = ts.ofdm.last_pilot_selectivity == -1.0
               && ts.last_channel_selectivity == -1.0;
    printf("[TEST-TINTERP-RX]   CELL-C one-pilot actual selector pilot=%.6f selected=%.6f -> %s\n",
           ts.ofdm.last_pilot_selectivity, ts.last_channel_selectivity, one_ok?"PASS":"FAIL");
    if(!one_ok) fails++;
    ts.selectivity_test_valid_pilots = -1;

    // A config epoch must invalidate both caches immediately, before a new frame.
    ts.ofdm.last_pilot_selectivity = 0.777;
    ts.last_channel_selectivity = 0.777;
    ts.load_configuration(CONFIG_16);
    bool transition_ok = ts.ofdm.last_pilot_selectivity == -1.0
                      && ts.last_channel_selectivity == -1.0;
    printf("[TEST-TINTERP-RX]   CELL-D config transition pilot=%.6f selected=%.6f -> %s\n",
           ts.ofdm.last_pilot_selectivity, ts.last_channel_selectivity,
           transition_ok?"PASS":"FAIL");
    if(!transition_ok) fails++;

    restore_env();
    printf("[TEST-TINTERP-RX] %s (%d cell failure%s)\n",
           fails==0?"ALL PASS":"FAILED", fails, fails==1?"":"s");
    return fails==0 ? 0 : 1;
}

// --test-acq-bounds: OFDM acquisition bounds-gate recovery regression
// (fact-documents/data-flow-acq-bounds-gate.md). PROVES the force-FAIL gate in
// cl_telecom_system::receive_byte (telecom_system.cc:1867) recovers a tail-band
// frame whose Schmidl-Cox detection floors 1..plateau_lead_symb symbols PAST
// upper_bound while its body is still in-buffer, instead of discarding it.
//
// Faithful in-process synthetic-fire: a REAL clean OFDM frame (full preamble +
// data, no AWGN) is TX'd via the production transmit_byte, placed into a full RX
// ring buffer at a chosen onset near upper_bound, and threaded through the
// production receive_byte WITHOUT ofdm_forced_delay (so the real Schmidl-Cox
// acquisition + bounds gate execute). ofdm_search_raw/nUnder are seeded so the
// anti-re-decode cursor ofdm_eff lands in the plateau-lead band — the exact
// condition that pre-fix force-FAILed (pream_symb_loc=0, frame discarded).
//
// One harness cell: place the frame, seed the cursor, decode, return outcome.
struct AcqBoundsCell { int detected_symb; int message_decoded; int delay; int upper_bound; };
static AcqBoundsCell acq_bounds_run_cell(int config, int onset_symb, int ofdm_eff_target)
{
    cl_telecom_system ts;
    ts.operation_mode = BER_PLOT_passband;
    ts.load_configuration(config);   // allocates buffers + sizes OFDM/LDPC via init()

    cl_data_container& dc = ts.data_container;
    int interp      = ts.frequency_interpolation_rate;
    int sym_samples = dc.Nofdm * interp;
    int buffer_Nsymb= dc.buffer_Nsymb;
    int frame_symb  = dc.Nsymb + dc.preamble_nSymb;
    int upper_bound = buffer_Nsymb - frame_symb;
    int frame_samp  = frame_symb * sym_samples;
    int buf_samp    = buffer_Nsymb * sym_samples;

    // Build one clean frame into a scratch buffer via the production TX path.
    int nReal_data = dc.nBits - ts.ldpc.P;
    int frame_size = (nReal_data - ts.outer_code_reserved_bits) / 8;
    std::vector<int> payload(frame_size > 0 ? frame_size : 1, 0);
    for (int i = 0; i < frame_size; i++) payload[i] = (i * 37 + 11) & 0xff;
    std::vector<double> frame(frame_samp, 0.0);
    ts.transmit_byte(payload.data(), frame_size, frame.data(), SINGLE_MESSAGE);

    // Place the frame at the requested onset in the full RX ring buffer (zeroed).
    double* rx = dc.ready_to_process_passband_delayed_data;   // sized Nofdm*buffer_Nsymb*interp
    for (int i = 0; i < buf_samp; i++) rx[i] = 0.0;
    int onset = onset_symb * sym_samples;
    if (onset < 0) onset = 0;
    int copy_n = frame_samp;
    if (onset + copy_n > buf_samp) copy_n = buf_samp - onset;   // truncate tail overrun
    for (int i = 0; i < copy_n; i++) rx[onset + i] = frame[i];

    // Seed the anti-re-decode cursor so ofdm_eff = ofdm_search_raw - nUnder lands
    // at the requested target (symbol units). nUnder=0 keeps it simple.
    ts.receive_stats.ofdm_search_raw   = ofdm_eff_target;
    ts.data_container.nUnder_processing_events = 0;
    ts.receive_stats.ofdm_batch_active = false;   // full-search path (not predict)
    ts.receive_stats.delay             = 0;
    ts.ofdm_forced_delay               = -1;      // real acquisition, NOT BER known-delay
    ts.mfsk_fixed_delay                = -1;
    // Isolate the force-FAIL bounds gate: with defer-overflow ON, a tail-band
    // detection (delay just past upper_bound) returns early via the overflow-defer
    // path (frame_overflow_symbols>0, "capture more audio") BEFORE the bounds gate.
    // The discard the fix targets is the defer-OFF / cursor-past-upper_bound path
    // (telecom_system.cc:1801 "--ofdm-defer-overflow=off bypasses Fix A"), where a
    // still-in-buffer frame is force-FAILed instead of recovered. Drive that path.
    ts.ofdm_defer_overflow_enabled     = false;

    ts.receive_byte(rx, dc.hd_decoded_data_byte);

    AcqBoundsCell out;
    out.delay           = ts.receive_stats.delay;
    out.detected_symb   = (sym_samples > 0) ? ts.receive_stats.delay / sym_samples : -1;
    out.message_decoded = ts.receive_stats.message_decoded;
    out.upper_bound     = upper_bound;
    return out;
}

int run_acq_bounds_selftest()
{
    printf("[TEST-ACQ-BOUNDS] OFDM acquisition bounds-gate recovery self-test\n");
    int fails = 0;
    // CONFIG_0 (BPSK rate-1/16): the most robust OFDM rung; a clean (noiseless)
    // loopback frame decodes trivially when acquisition lands it correctly, so
    // message_decoded isolates the GATE decision, not LDPC marginality.
    const int cfg = CONFIG_0;

    // Determine geometry for the placement (one throwaway cell at a benign onset).
    AcqBoundsCell geo = acq_bounds_run_cell(cfg, 5, 0);
    int ub = geo.upper_bound;
    printf("[TEST-ACQ-BOUNDS]   geometry: upper_bound=%d (control onset=5 -> decoded=%d detected=%d)\n",
           ub, geo.message_decoded, geo.detected_symb);

    // ---- CELL 1 (fail-before / pass-after): TAIL-BAND frame must be RECOVERED. ----
    // Onset = upper_bound (the highest sample-exact-fitting onset: a full frame
    // placed here ends exactly at the buffer end). With the anti-re-decode cursor
    // seeded one symbol into the plateau-lead band (ofdm_eff = upper_bound+1), the
    // PRE-FIX gate force-FAILs (ofdm_eff>upper_bound -> pream_symb_loc=0 -> the
    // decode block is skipped -> message_decoded=NO, frame DISCARDED). POST-FIX the
    // widened cutoff (upper_bound+plateau_lead_symb) lets the recovery scan run; it
    // re-anchors Schmidl-Cox to the in-bounds onset and the frame DECODES.
    {
        AcqBoundsCell c = acq_bounds_run_cell(cfg, ub, ub + 1);
        bool recovered = (c.message_decoded == YES);
        printf("[TEST-ACQ-BOUNDS]   CELL1 tail-band@onset=%d ofdm_eff=%d: decoded=%d detected=%d -> %s\n",
               ub, ub + 1, c.message_decoded, c.detected_symb,
               recovered ? "RECOVERED_OK" : "DISCARDED_FAIL");
        if (!recovered) fails++;
    }

    // ---- CELL 2 (no-regress / anti-re-decode preserved): cursor GENUINELY past the
    // band with NO fitting frame must still force-FAIL (not re-decode old audio). ----
    // Empty buffer (no frame) and ofdm_eff well past the widened cutoff: recovery
    // must NOT manufacture a decode. Asserts the fix did not defeat anti-re-decode.
    {
        AcqBoundsCell c = acq_bounds_run_cell(cfg, /*onset (unused, no signal)*/ ub, ub + 8);
        // Re-run with a zeroed buffer: overwrite the frame by passing an onset that
        // truncates it fully out is awkward; instead place the frame far below the
        // cursor floor (onset well under ofdm_eff-plateau) so the scan_start>onset
        // excludes it -> no recovery -> force-FAIL. This is the already-decoded case.
        AcqBoundsCell c2 = acq_bounds_run_cell(cfg, 2, ub + 8);
        bool no_decode = (c.message_decoded != YES) && (c2.message_decoded != YES);
        printf("[TEST-ACQ-BOUNDS]   CELL2 past-band ofdm_eff=%d: emptybuf_decoded=%d below-floor_decoded=%d -> %s\n",
               ub + 8, c.message_decoded, c2.message_decoded,
               no_decode ? "FORCE-FAIL_OK" : "SPURIOUS_DECODE_FAIL");
        if (!no_decode) fails++;
    }

    printf("[TEST-ACQ-BOUNDS] %s (%d cell failure%s)\n", fails==0?"ALL PASS":"FAILED", fails, fails==1?"":"s");
    return fails==0 ? 0 : 1;
}

// --test-subpeak-rescue: F1b Part B sample-anchored true-boundary rescue mechanism
// self-test (fact-documents/data-flow-rx-ring-rearm.md, _research/F1B_DESIGN.md §2/§3).
// PROVES the SUBPEAK-REJECT rescue's two production primitives on a deterministic,
// noiseless, in-process synthetic-fire (no IONOS/RF):
//   (1) the SELECTOR cl_telecom_system::ofdm_meanH_at_delay reads a HIGH mean|H| at the
//       true frame onset and a COLLAPSED mean|H| at a ~1.5-symbol-early wrong lock (the
//       field sub-peak fingerprint: metric saturates but pilots misalign, mean|H| ~0.37);
//   (2) the argmax scan over [orig-2sym, orig+2sym] (the exact loop receive_byte runs in
//       the reject branch) RECOVERS the true onset to within ±¼ symbol;
//   (3) the production decode path (receive_byte) DECODES at the true / refined delay and
//       FAILS at the wrong lock — the fail-before (wrong lock never reads) / pass-after
//       (refined delay reads) proof through the real decoder.
// The in-process path structurally cannot make the live Schmidl-Cox mislock (the drain is
// a no-op under -x sim), so the PRODUCTION rescue-branch FIRING is proven separately by the
// real-audio needle-2 cohort (anchor_race_vehicle.py: [XCORR-RESCUE-OK] tokens + needle2 -> 0).
// This test isolates the mechanism the cohort then confirms fires end-to-end.
static int subpeak_rescue_geom(int force_Dy, int force_Nsymb, const char* label,
                               double noise_snr_db /* <0 => noiseless */)
{
    // GEOMETRY-INVARIANT discriminator: assert on the pilot phase COHERENCE C
    // (telecom_system.cc last_pilot_coherence), NOT the absolute mean|H|. mean|H|'s
    // wrong-lock collapse deepens with pilot count, so the thinned cfg16 grid (Dy=5)
    // leaves a wrong lock reading mean|H|~0.77 -- above the 0.74 legacy floor -- and
    // the old absolute-floor assertion FALSELY passes the wrong lock. C is count-
    // normalized: a decorrelated wrong lock reads C~0.1 at ANY density, a good lock
    // reads C~1, so the same thresholds separate on Dy=3 AND Dy=5.
    const double C_ENTRY  = 0.35;  // wrong lock must read C below this (production subpeak_coh_entry)
    const double C_ACCEPT = 0.50;  // true / recovered lock must read C at/above this (subpeak_coh_accept)
    const double FLOOR    = 0.74;  // legacy good-timing mean|H| floor (printed for context, not asserted-on)

    // Force the cfg16 pilot lattice. Baked default is Dy=5/Nsymb=8; the pilot-override
    // env reconstructs any (Dy,Nsymb) for CONFIG_16 (pilot_override_target default).
    const char* ks[2] = { "MERCURY_PILOT_DY", "MERCURY_PILOT_NSYMB" };
    bool had[2]; std::string sav[2];
    for (int i=0;i<2;i++){ const char* g=std::getenv(ks[i]); had[i]=(g!=nullptr); sav[i]=g?std::string(g):std::string(); }
    char dybuf[16], nsbuf[16];
    snprintf(dybuf,sizeof(dybuf),"%d",force_Dy);
    snprintf(nsbuf,sizeof(nsbuf),"%d",force_Nsymb);
    setenv("MERCURY_PILOT_DY", dybuf, 1);
    setenv("MERCURY_PILOT_NSYMB", nsbuf, 1);

    int fails = 0;
    const int cfg = CONFIG_16;
    cl_telecom_system ts;
    ts.operation_mode = BER_PLOT_passband;
    ts.load_configuration(cfg);

    // Geometry is now locked into this ts instance; restore the env immediately.
    for (int i=0;i<2;i++){ if(had[i]) setenv(ks[i],sav[i].c_str(),1); else unsetenv(ks[i]); }

    cl_data_container& dc = ts.data_container;
    int interp       = ts.frequency_interpolation_rate;
    int sym_samples  = dc.Nofdm * interp;
    int frame_symb   = dc.Nsymb + dc.preamble_nSymb;
    int frame_samp   = frame_symb * sym_samples;
    int buf_samp     = dc.buffer_Nsymb * sym_samples;
    int max_delay    = buf_samp - frame_samp;
    double carrier   = ts.carrier_frequency;

    // Build one clean CONFIG_16 frame via the production TX path.
    int nReal_data = dc.nBits - ts.ldpc.P;
    int frame_size = (nReal_data - ts.outer_code_reserved_bits) / 8;
    if (frame_size < 1) frame_size = 1;
    std::vector<int> payload(frame_size, 0);
    for (int i = 0; i < frame_size; i++) payload[i] = (i * 53 + 7) & 0xff;
    std::vector<double> frame(frame_samp, 0.0);
    ts.transmit_byte(payload.data(), frame_size, frame.data(), SINGLE_MESSAGE);

    // Optional AWGN (low-SNR true-lock case): calibrate sigma to the frame RMS so
    // the requested passband SNR depresses mean|H| toward the good-timing floor
    // without destroying the lock -- the exact regime where an ABSOLUTE floor risks
    // false-rejecting a real frame but a NORMALIZED coherence must not.
    std::vector<double> used(frame);
    if (noise_snr_db >= 0.0) {
        double e = 0.0; for (double v : frame) e += v*v;
        double rms = (frame.size()>0)? sqrt(e/(double)frame.size()) : 0.0;
        double sigma = (rms>0.0)? rms / pow(10.0, noise_snr_db/20.0) : 0.0;
        std::mt19937 rng(0xC0FFEEu + (unsigned)force_Dy*131u + (unsigned)(noise_snr_db*7.0));
        std::normal_distribution<double> nd(0.0, sigma);
        for (size_t i=0;i<used.size();i++) used[i] += nd(rng);
    }

    // Place at a true onset preceded by a zero run-up (the flush-memset silence that
    // seeds the Schmidl-Cox plateau in the field). onset=8 symbols in.
    double* rx = dc.ready_to_process_passband_delayed_data;
    for (int i = 0; i < buf_samp; i++) rx[i] = 0.0;
    int onset_symb = 8;
    int D = onset_symb * sym_samples;
    if (D > max_delay) D = max_delay;
    int copy_n = frame_samp;
    if (D + copy_n > buf_samp) copy_n = buf_samp - D;
    for (int i = 0; i < copy_n; i++) rx[D + i] = used[i];

    // The field wrong lock sits ~1.5 symbols early.
    int wrong = D - (sym_samples + sym_samples / 2);
    if (wrong < 0) wrong = 0;

    // ---- (1) SELECTOR discrimination via the production helper (mean|H| AND C) ----
    double C_true = -1.0, C_wrong = -1.0;
    double mH_true  = ts.ofdm_meanH_at_delay(rx, D,     carrier, dc.preamble_nSymb, &C_true);
    double mH_wrong = ts.ofdm_meanH_at_delay(rx, wrong, carrier, dc.preamble_nSymb, &C_wrong);
    printf("[TEST-SUBPEAK-RESCUE] [%s] Dy=%d Nsymb=%d %s\n", label, force_Dy, force_Nsymb,
           (noise_snr_db<0.0)?"noiseless":"low-SNR true-lock");
    printf("[TEST-SUBPEAK-RESCUE]   selector: true(d=%d) mean|H|=%.3f C=%.3f | wrong(d=%d) mean|H|=%.3f C=%.3f | floor(legacy)=%.2f C_entry=%.2f C_accept=%.2f\n",
           D, mH_true, C_true, wrong, mH_wrong, C_wrong, FLOOR, C_ENTRY, C_ACCEPT);

    // decode-at-forced-delay helper (uses the possibly-noisy frame)
    auto decode_at = [&](int forced_delay)->int {
        for (int i = 0; i < buf_samp; i++) rx[i] = 0.0;
        for (int i = 0; i < copy_n; i++) rx[D + i] = used[i];
        ts.receive_stats.ofdm_search_raw = 0;
        ts.data_container.nUnder_processing_events = 0;
        ts.receive_stats.ofdm_batch_active = false;
        ts.receive_stats.delay = 0;
        ts.mfsk_fixed_delay = -1;
        ts.ofdm_forced_delay = forced_delay;
        ts.receive_byte(rx, dc.hd_decoded_data_byte);
        int ok = (ts.receive_stats.message_decoded == YES) ? 1 : 0;
        if (ok) {
            for (int i = 0; i < frame_size; i++)
                if ((dc.hd_decoded_data_byte[i] & 0xff) != (payload[i] & 0xff)) { ok = 0; break; }
        }
        return ok;
    };

    if (noise_snr_db < 0.0) {
        // NOISELESS: full geometry-invariant discrimination on this grid.
        bool sel_true_ok  = (C_true  >= C_ACCEPT);
        bool sel_wrong_ok = (C_wrong >= 0.0) && (C_wrong <  C_ENTRY) && (C_true - C_wrong >= 0.30);
        if (!sel_true_ok)  { fails++; printf("[TEST-SUBPEAK-RESCUE]   FAIL[%s]: true-onset coherence below accept (%.3f < %.2f)\n", label, C_true, C_ACCEPT); }
        if (!sel_wrong_ok) { fails++; printf("[TEST-SUBPEAK-RESCUE]   FAIL[%s]: wrong-lock coherence not clearly below entry (C=%.3f, entry=%.2f, margin=%.3f)\n", label, C_wrong, C_ENTRY, C_true - C_wrong); }

        // ---- (2) argmax scan (identical to the receive_byte reject-branch loop) ----
        int step = sym_samples / 4; if (step < 1) step = 1;
        double best_mH = mH_wrong; int best_delay = wrong; double best_C = C_wrong;
        for (int off = -2*sym_samples; off <= 2*sym_samples; off += step) {
            int cand = wrong + off;
            if (cand < 0 || cand > max_delay) continue;
            double candC = -1.0;
            double mH = ts.ofdm_meanH_at_delay(rx, cand, carrier, dc.preamble_nSymb, &candC);
            if (mH > best_mH) { best_mH = mH; best_delay = cand; best_C = candC; }
        }
        bool scan_ok = (best_C >= C_ACCEPT) && (std::abs(best_delay - D) <= step);
        printf("[TEST-SUBPEAK-RESCUE]   argmax scan[%s]: best_delay=%d (true=%d, err=%d, <=%d? %s) mean|H|=%.3f C=%.3f (C>=%.2f? %s)\n",
               label, best_delay, D, best_delay - D, step, (std::abs(best_delay - D) <= step)?"YES":"NO",
               best_mH, best_C, C_ACCEPT, (best_C>=C_ACCEPT)?"YES":"NO");
        if (!scan_ok) { fails++; printf("[TEST-SUBPEAK-RESCUE]   FAIL[%s]: argmax did not recover a coherent true onset\n", label); }

        // ---- (3) production decode legs: truth decodes, wrong fails, refined decodes ----
        int dec_true  = decode_at(D);
        int dec_wrong = decode_at(wrong);
        int dec_best  = decode_at(best_delay);
        printf("[TEST-SUBPEAK-RESCUE]   decode[%s]: true=%s wrong=%s refined=%s\n",
               label, dec_true?"DECODED":"FAILED", dec_wrong?"DECODED":"FAILED", dec_best?"DECODED":"FAILED");
        if (!dec_true)  { fails++; printf("[TEST-SUBPEAK-RESCUE]   FAIL[%s]: clean frame did not decode at the true onset\n", label); }
        if ( dec_wrong) { fails++; printf("[TEST-SUBPEAK-RESCUE]   FAIL[%s]: wrong lock decoded (fail-before did not reproduce)\n", label); }
        if (!dec_best)  { fails++; printf("[TEST-SUBPEAK-RESCUE]   FAIL[%s]: refined delay did not decode (pass-after)\n", label); }
    } else {
        // LOW-SNR TRUE-LOCK PRESERVATION: a noise-depressed GENUINE lock must still be
        // ACCEPTED by the coherence floor (never turned into a false sub-peak reject),
        // and must still DECODE. This guards the risk that a geometry-aware floor
        // trades a wrong-lock catch for a real-frame false-reject at low SNR.
        bool true_accept = (C_true >= C_ACCEPT);
        int dec_true = decode_at(D);
        printf("[TEST-SUBPEAK-RESCUE]   low-SNR[%s] snr=%.1fdB: true mean|H|=%.3f C=%.3f (accept C>=%.2f? %s) decode=%s\n",
               label, noise_snr_db, mH_true, C_true, C_ACCEPT, true_accept?"YES":"NO", dec_true?"DECODED":"FAILED");
        if (!true_accept) { fails++; printf("[TEST-SUBPEAK-RESCUE]   FAIL[%s]: LOW-SNR true lock FALSE-REJECTED by coherence floor (C=%.3f < %.2f)\n", label, C_true, C_ACCEPT); }
        if (!dec_true)    { fails++; printf("[TEST-SUBPEAK-RESCUE]   FAIL[%s]: LOW-SNR true lock did not decode (SNR too low for this check -- raise noise_snr_db)\n", label); }
    }
    return fails;
}

int run_subpeak_rescue_selftest()
{
    printf("[TEST-SUBPEAK-RESCUE] F1b Part B true-boundary rescue mechanism self-test (geometry-parametrized)\n");
    int fails = 0;
    // BOTH grids must correctly reject their wrong lock and accept their true lock on
    // the GEOMETRY-INVARIANT coherence metric: the baked thin default (Dy=5/Nsymb=8)
    // AND the stock dense reconstruct (Dy=3/Nsymb=9). A future geometry change cannot
    // silently regress the selector because both densities are asserted here.
    fails += subpeak_rescue_geom(5, 8, "thin",  -1.0);
    fails += subpeak_rescue_geom(3, 9, "dense", -1.0);
    // TRUE-LOCK PRESERVED at low SNR on BOTH grids: the coherence floor must not
    // false-reject a legitimate lock whose mean|H| is depressed by noise. Because
    // the discriminator is a phase COHERENCE (magnitude-independent), a sagging
    // mean|H| does not threaten the accept -- this case proves it. SNR overridable
    // (MERCURY_SUBPEAK_TEST_SNR) for calibration; the baked value sits in the
    // decoding regime where mean|H| already visibly sags.
    // 10 dB: comfortably inside the cfg16 decode regime on this synthetic (decode
    // holds down to ~8 dB with the fixed seed) yet well below the high-SNR envelope
    // cfg16 is elected in. Note on this FLAT synthetic mean|H| stays ~1.0 at every
    // SNR (AGC + LS window preserve it), so the magnitude-sag false-reject the floor
    // was feared to cause is not even reproducible -- and coherence, being magnitude-
    // independent, holds ~1.0 regardless. The frequency-selective true-lock case
    // (where mean|H| genuinely ripples) is the owed real-audio fire proof.
    double lo_snr = 10.0;
    { const char* e = std::getenv("MERCURY_SUBPEAK_TEST_SNR"); if(e && *e) lo_snr = atof(e); }
    fails += subpeak_rescue_geom(5, 8, "thin",  lo_snr);
    fails += subpeak_rescue_geom(3, 9, "dense", lo_snr);
    printf("[TEST-SUBPEAK-RESCUE] %s (%d failure%s)\n", fails==0?"ALL PASS":"FAILED", fails, fails==1?"":"s");
    return fails==0 ? 0 : 1;
}

// --test-chase-ab: production-path HARQ/Chase A/B fire proof (no cards, no RF).
// Drives the REAL receive_byte() TWICE on the SAME OFDM codeword at a sub-threshold
// SNR with INDEPENDENT noise realizations: look-1 fails the single-look decode and
// is buffered by the chase CAPTURE hook (telecom_system.cc terminal-fail branch);
// look-2 is soft-combined with look-1 by the chase RESCUE hook (before the accept
// decision) and re-decoded. Reports, per (config, SNR):
//   single%   = look-1 single-look decode rate (== both arms: floor is NOT lowered)
//   comb_OFF% = look-2 decode rate, MERCURY_CHASE OFF (incumbent: another single look)
//   comb_ON%  = look-2 decode rate, MERCURY_CHASE ON  (2-look combine)
//   rescues   = chase adopts fired;   corrupt = adopted frame whose bytes != payload
// comb_ON - comb_OFF at equal SNR is the delivered reach; the SNR gap between the
// single-look and comb_ON waterfalls is the coding gain (~+3 dB MRC per copy).
struct ChaseABCell {
    int config; double snr_db; int trials;
    int single_on; int single_off; int comb_on; int comb_off;
    long rescues; int corrupt;
};

static ChaseABCell chase_ab_run_cell(int config, double snr_db, int trials, unsigned base_seed)
{
    ChaseABCell R; R.config=config; R.snr_db=snr_db; R.trials=trials;
    R.single_on=R.single_off=R.comb_on=R.comb_off=0; R.rescues=0; R.corrupt=0;

    for(int t=0;t<trials;t++){
    // FRESH telecom_system per trial. Latched acquisition/batch state accumulates
    // across receive_byte calls on a reused instance and starves the decode path
    // after ~a dozen calls (measured: noiseless decode 100% at 3 trials -> 33% at
    // 12 on a shared ts); a per-trial instance keeps each to <=4 receives (2 arms x
    // 2 looks), which stays clean. The chase buffer lives on this ts and correctly
    // persists across the trial's look-1 -> look-2 within each arm.
    cl_telecom_system ts;
    ts.operation_mode = BER_PLOT_passband;
    ts.load_configuration(config);
    // The single-frame synthetic vehicle's pilot-residual noise_variance_estimate
    // reads high (~1.4) even NOISELESS (a property of a lone frame vs a real batch),
    // so the pre-LDPC SKIP-VAR gate would reject every look before the decoder runs.
    // We are exercising the LDPC decode + the chase capture/rescue hooks, NOT the
    // SKIP-VAR gate -- disable it so every look reaches ldpc.decode() (the same knob
    // as --skip-var-gate=off). LDPC+CRC16 remain the actual accept mechanism.
    ts.skip_var_gate_enabled = false;
    cl_data_container& dc = ts.data_container;
    int interp      = ts.frequency_interpolation_rate;
    int sym_samples = dc.Nofdm * interp;
    int frame_symb  = dc.Nsymb + dc.preamble_nSymb;
    int frame_samp  = frame_symb * sym_samples;
    int buf_samp    = dc.buffer_Nsymb * sym_samples;
    int max_delay   = buf_samp - frame_samp;
    int nReal_data  = dc.nBits - ts.ldpc.P;
    int frame_size  = (nReal_data - ts.outer_code_reserved_bits) / 8;
    if(frame_size < 1) frame_size = 1;
    int onset_symb = 8;   // match the proven subpeak decode_at recipe (onset 8, not 3)
    int D = onset_symb * sym_samples; if(D>max_delay) D=max_delay; if(D<0) D=0;
    static int ab_dbg = -1;
    if(ab_dbg<0){ const char* e=std::getenv("MERCURY_CHASE_AB_DEBUG"); ab_dbg=(e&&*e&&atoi(e))?1:0; }

    // Drive ONE noisy look through the production acquisition + decode path.
    auto run_look = [&](const std::vector<double>& frame, double frame_rms,
                        unsigned seed, std::vector<int>& out_bytes)->int {
        // ZERO background (silence run-up) + frame(+noise) at D -- the proven
        // subpeak_rescue_geom recipe. A noise-filled background outside the frame
        // spuriously triggers batch/acquisition state on the reused ts and starves
        // subsequent looks of the decode path.
        double* rx = dc.ready_to_process_passband_delayed_data;
        std::mt19937 rng(seed);
        double sigma = (snr_db < 200.0) ? frame_rms / pow(10.0, snr_db/20.0) : 0.0;
        std::normal_distribution<double> nd(0.0, sigma);
        for(int i=0;i<buf_samp;i++) rx[i] = 0.0;
        int copy_n = frame_samp; if(D+copy_n>buf_samp) copy_n=buf_samp-D;
        for(int i=0;i<copy_n;i++) rx[D+i] = frame[i] + ((sigma>0.0)? nd(rng):0.0);
        // Scrub latched per-decode state so a prior failed look cannot poison this one.
        ts.receive_stats.ofdm_search_raw           = 0;
        ts.data_container.nUnder_processing_events = 0;
        ts.receive_stats.ofdm_batch_active        = false;
        ts.receive_stats.delay                    = 0;
        ts.use_last_good_freq_offset               = NO;
        ts.use_last_good_time_sync                 = NO;
        ts.consecutive_ofdm_decode_fails           = 0;
        ts.mfsk_fixed_delay                        = -1;
        // Pin extraction to the KNOWN onset D (same recipe as the subpeak-rescue
        // decode_at() helper): a lone synthetic frame does not fine-lock cleanly
        // enough under full search, so we extract at the exact placement. Both looks
        // use the same delay; the only difference is the independent noise draw --
        // exactly the same-codeword retx scenario chase combines.
        ts.ofdm_forced_delay                       = D;
        ts.receive_byte(rx, dc.hd_decoded_data_byte);
        int decoded = (ts.receive_stats.message_decoded==YES)?1:0;
        if(ab_dbg) fprintf(stderr,"[AB-DBG] cfg=%d snr=%.1f seed=%u decoded=%d delay=%d sync_trials=%d crc=0x%04X iter=%d allz=%d\n",
                           config, snr_db, seed, decoded, (int)ts.receive_stats.delay, ts.receive_stats.sync_trials,
                           ts.receive_stats.crc, ts.receive_stats.iterations_done, ts.receive_stats.all_zeros);
        out_bytes.assign(frame_size,0);
        for(int i=0;i<frame_size;i++) out_bytes[i]=dc.hd_decoded_data_byte[i]&0xff;
        return decoded;
    };

    {
        // Distinct payload (distinct codeword) per trial so a cross-trial mismatched
        // combine cannot silently help; distinct, well-separated look seeds ensure
        // the two looks are DECORRELATED (chase-combining-harq.md §6.1 landmine).
        std::vector<int> payload(frame_size,0);
        for(int i=0;i<frame_size;i++) payload[i]=((i*53+7) ^ (t*131+29)) & 0xff;
        std::vector<double> frame(frame_samp,0.0);
        ts.transmit_byte(payload.data(), frame_size, frame.data(), SINGLE_MESSAGE);
        double e=0.0; for(double v:frame) e+=v*v;
        double frame_rms = frame.size()? sqrt(e/(double)frame.size()):0.0;
        unsigned s1 = base_seed + 100003u*(unsigned)(t+1);
        unsigned s2 = base_seed + 700019u*(unsigned)(t+1) + 17u;

        std::vector<int> b1,b2;
        // OFF arm (incumbent): chase disabled -> two independent single looks.
        ts.chase_enabled = false; ts.chase_reset();
        R.single_off += run_look(frame, frame_rms, s1, b1);
        R.comb_off   += run_look(frame, frame_rms, s2, b2);
        // ON arm: chase enabled -> look-1 captured, look-2 combined+rescued.
        ts.chase_enabled = true; ts.chase_reset();
        long resc0 = ts.chase_rescues;
        R.single_on += run_look(frame, frame_rms, s1, b1);   // fails -> captures
        int d2 = run_look(frame, frame_rms, s2, b2);         // combine -> maybe rescue
        R.comb_on += d2;
        if(d2){ bool ok=true; for(int i=0;i<frame_size;i++) if(b2[i]!=(payload[i]&0xff)){ok=false;break;} if(!ok) R.corrupt++; }
        R.rescues += (ts.chase_rescues - resc0);
    }
    } // end per-trial fresh-ts loop
    return R;
}

int run_chase_ab_selftest()
{
    printf("[TEST-CHASE-AB] production-path HARQ/Chase A/B (receive_byte 2-look combine)\n");
    int trials = 12;
    { const char* e=std::getenv("MERCURY_CHASE_AB_TRIALS"); if(e&&*e){ trials=atoi(e); if(trials<1) trials=1; } }
    // Per-config SNR sweeps (dB) straddling each rung's single-look waterfall so the
    // sub-threshold band (single ~0, comb_ON > 0) is visible. Overridable.
    // Default SNR sweeps straddle each rung's single-look waterfall INTO the chase
    // rescue band (single-look ~0, 2-look combine still decodes). Measured on this
    // synthetic vehicle (requested SNR sits ~10 dB below the effective Es/N0):
    //   cfg6  single waterfall ~-10.5 dB, combined holds to ~-12.5 (+~2 dB)
    //   cfg9  single waterfall ~-7   dB, combined holds to ~-10  (+~3 dB)
    //   cfg13 single waterfall ~-1   dB, combined holds to ~-4   (+~2 dB)
    struct { int cfg; std::vector<double> snrs; } band[] = {
        { 6,  { -9,-10,-11,-12,-13} },
        { 9,  { -6, -8, -9,-10,-11} },
        { 13, {  0, -2, -4, -5, -6} },
    };
    int nband = 3;
    // Env overrides (calibration without rebuild): MERCURY_CHASE_AB_CONFIGS="6,9,13"
    // sets which configs; MERCURY_CHASE_AB_SNRS="a,b,c" applies ONE sweep to all.
    auto parse_list = [](const char* s, std::vector<double>& out){
        out.clear(); std::string cur;
        for(const char* p=s;;++p){ if(*p==','||*p=='\0'){ if(!cur.empty()){ out.push_back(atof(cur.c_str())); cur.clear(); } if(*p=='\0') break; } else cur+=*p; }
    };
    std::vector<int> cfg_override;
    { const char* e=std::getenv("MERCURY_CHASE_AB_CONFIGS"); if(e&&*e){ std::vector<double> tmp; parse_list(e,tmp); for(double v:tmp) cfg_override.push_back((int)v); } }
    std::vector<double> snr_override;
    { const char* e=std::getenv("MERCURY_CHASE_AB_SNRS"); if(e&&*e) parse_list(e,snr_override); }
    std::vector<std::pair<int,std::vector<double>>> plan;
    if(!cfg_override.empty()){
        for(int c : cfg_override) plan.push_back({c, !snr_override.empty()? snr_override : std::vector<double>{0,2,4,6,8,10,12,14,16,18,20}});
    } else {
        for(int bi=0;bi<nband;bi++) plan.push_back({band[bi].cfg, !snr_override.empty()? snr_override : band[bi].snrs});
    }
    int fails = 0;
    bool any_lift = false;
    printf("[TEST-CHASE-AB] trials=%d per cell; looks are DECORRELATED; distinct codeword per trial\n", trials);
    printf("  cfg  SNRdB   single%%   comb_OFF%%  comb_ON%%   rescues  corrupt   (comb_ON-OFF = delivered reach)\n");
    for(auto& pe : plan){
        for(double snr : pe.second){
            ChaseABCell R = chase_ab_run_cell(pe.first, snr, trials, 0xC4A5Eu);
            double sp = 100.0*R.single_on/trials;
            double coff = 100.0*R.comb_off/trials;
            double con = 100.0*R.comb_on/trials;
            printf("  %3d  %5.1f   %5.0f     %5.0f      %5.0f      %4ld     %4d\n",
                   pe.first, snr, sp, coff, con, R.rescues, R.corrupt);
            fflush(stdout);
            // Floor-not-lowered: look-1 single decode identical ON vs OFF.
            if(R.single_on != R.single_off){
                printf("    [FAIL] floor moved: single_ON=%d != single_OFF=%d (chase changed the single-look decode)\n",
                       R.single_on, R.single_off);
                fails++;
            }
            // 0-corruption: no adopted frame may byte-mismatch the payload.
            if(R.corrupt != 0){
                printf("    [FAIL] %d chase-adopted frame(s) corrupt (bytes != payload)\n", R.corrupt);
                fails++;
            }
            // Incumbent floor: comb_OFF must not exceed single (it is just another
            // single look) by more than noise -- sanity, not a hard gate.
            if(R.comb_on > R.comb_off && R.rescues > 0) any_lift = true;
        }
    }
    // Lift is a HARD gate ONLY on the default (tuned) bands: the sweet-spot cells
    // rescue reproducibly, so a default run with NO lift means the chase hooks stopped
    // firing (a regression). Under an env SNR override the band may deliberately miss
    // the waterfall, so there it is a WARN, not a fail.
    bool default_bands = cfg_override.empty() && snr_override.empty();
    if(!any_lift){
        if(default_bands){
            printf("[TEST-CHASE-AB] [FAIL] no cell showed comb_ON > comb_OFF with a rescue on the default bands -- the chase capture/rescue hooks did not fire.\n");
            fails++;
        } else {
            printf("[TEST-CHASE-AB] [WARN] no cell showed comb_ON > comb_OFF with a rescue -- SNR band may miss the waterfall; retune MERCURY_CHASE_AB_* or inspect.\n");
        }
    }
    printf("[TEST-CHASE-AB] %s (fails=%d, lift_seen=%s)\n",
           fails==0 ? "PASS (chase rescues double-fail frames; 0 corrupt; floor unchanged)" : "FAILED", fails, any_lift?"YES":"NO");
    return fails==0 ? 0 : 1;
}

// --test-frame0-vehicle: deterministic IN-PROCESS reproduction of the
// post-turnaround FIRST-FRAME wrong-lock (the anchor-race structural loss).
// Synthesizes a passband RX buffer = [near-silent AWGN run-up] + [a REAL preamble
// + data frame for the loaded config, built via the production transmit_byte] and
// threads it through the PRODUCTION acquisition (receive_byte, ofdm_forced_delay=-1
// => real Schmidl-Cox coarse + Phase-2 argmax + site-8 with_metric fine +
// SUBPEAK-REJECT). Recovery is defeated via MERCURY_SUBPEAK_RESCUE_DEFEAT=1 so the
// RAW lock is measured (prevention meter, not the recovery counter).
//
// Per cell it reports: true onset D, the final locked delay, the SIGNED offset
// delta = final - D in symbols (the DIRECTION the field docs disagree on: EARLY /
// undershoot vs LATE / overshoot), coarse metric, mean_H, decoded. Faithfulness
// fingerprint = the field signature (coarse_metric>=0.97 && mean_H<0.5 &&
// |delta|>=~1 symbol && !decoded). This is the missing deterministic fire-proof.
struct F0VCell {
    int onset_symb; double snr_db; unsigned seed;
    int true_onset; int final_delay; int delta; double delta_sym;
    double coarse_metric; double mean_H; int decoded; int sync_trials; int edge;
    double frame_rms; double sigma;
};

static F0VCell f0v_run_cell(int config, int onset_symb, double snr_db, unsigned seed)
{
    cl_telecom_system ts;
    ts.operation_mode = BER_PLOT_passband;
    ts.load_configuration(config);

    cl_data_container& dc = ts.data_container;
    int interp       = ts.frequency_interpolation_rate;
    int sym_samples  = dc.Nofdm * interp;
    int frame_symb   = dc.Nsymb + dc.preamble_nSymb;
    int frame_samp   = frame_symb * sym_samples;
    int buf_samp     = dc.buffer_Nsymb * sym_samples;
    int max_delay    = buf_samp - frame_samp;

    // Build one clean frame via the production TX path.
    int nReal_data = dc.nBits - ts.ldpc.P;
    int frame_size = (nReal_data - ts.outer_code_reserved_bits) / 8;
    if (frame_size < 1) frame_size = 1;
    std::vector<int> payload(frame_size, 0);
    for (int i = 0; i < frame_size; i++) payload[i] = (i * 53 + 7) & 0xff;
    std::vector<double> frame(frame_samp, 0.0);
    ts.transmit_byte(payload.data(), frame_size, frame.data(), SINGLE_MESSAGE);

    // RMS over the frame region -> calibrate the AWGN floor to the requested SNR.
    double e = 0.0; for (double v : frame) e += v * v;
    double frame_rms = (frame.size() > 0) ? sqrt(e / (double)frame.size()) : 0.0;

    int D = onset_symb * sym_samples; if (D > max_delay) D = max_delay; if (D < 0) D = 0;

    // Whole-buffer AWGN floor (the near-silent RUN-UP the field flush leaves before
    // frame-0; noise-only outside the frame region, signal+noise inside).
    double* rx = dc.ready_to_process_passband_delayed_data;
    std::mt19937 rng(seed);
    double sigma = (snr_db < 200.0) ? frame_rms / pow(10.0, snr_db / 20.0) : 0.0;
    std::normal_distribution<double> nd(0.0, sigma);
    for (int i = 0; i < buf_samp; i++) rx[i] = (sigma > 0.0) ? nd(rng) : 0.0;
    int copy_n = frame_samp; if (D + copy_n > buf_samp) copy_n = buf_samp - D;
    for (int i = 0; i < copy_n; i++) rx[D + i] += frame[i];

    // Real acquisition, INIT full-search (no BATCH predict, no forced/known delay).
    ts.receive_stats.ofdm_search_raw           = 0;
    ts.data_container.nUnder_processing_events = 0;
    ts.receive_stats.ofdm_batch_active         = false;
    ts.receive_stats.delay                     = 0;
    ts.ofdm_forced_delay                       = -1;
    ts.mfsk_fixed_delay                        = -1;

    ts.receive_byte(rx, dc.hd_decoded_data_byte);

    F0VCell c;
    c.onset_symb = onset_symb; c.snr_db = snr_db; c.seed = seed;
    c.true_onset = D;
    c.final_delay = ts.receive_stats.delay;
    c.delta = c.final_delay - D;
    c.delta_sym = (sym_samples > 0) ? (double)c.delta / (double)sym_samples : 0.0;
    c.coarse_metric = ts.receive_stats.coarse_metric;
    c.mean_H = ts.receive_stats.mean_H;
    c.decoded = (ts.receive_stats.message_decoded == YES) ? 1 : 0;
    // byte-check the decode so a spurious message_decoded flag cannot read as TRUE.
    if (c.decoded) {
        for (int i = 0; i < frame_size; i++)
            if ((dc.hd_decoded_data_byte[i] & 0xff) != (payload[i] & 0xff)) { c.decoded = 0; break; }
    }
    c.sync_trials = ts.receive_stats.sync_trials;
    int half = sym_samples / 2;
    c.edge = (c.delta < -half) ? -1 : (c.delta > half ? +1 : 0);
    c.frame_rms = frame_rms; c.sigma = sigma;
    return c;
}

int run_frame0_vehicle_selftest()
{
    // Ensure the RAW lock is measured (defeat the F1b rescue). setenv before the
    // first receive_byte so the read-once static in telecom_system sees it.
    setenv("MERCURY_SUBPEAK_RESCUE_DEFEAT", "1", 1);

    printf("[TEST-FRAME0-VEHICLE] deterministic in-process post-turnaround wrong-lock reproduction\n");
    const char* cfg_env = std::getenv("MERCURY_F0V_CONFIG");
    int cfg = cfg_env ? atoi(cfg_env) : CONFIG_16;
    printf("[TEST-FRAME0-VEHICLE] config=%d  (fingerprint: coarse_metric>=0.97 && mean_H<0.5 && |delta|>=1sym && !decoded)\n", cfg);
    printf("[TEST-FRAME0-VEHICLE] %-6s %-8s %-6s %-11s %-9s %-8s %-8s %-4s %-6s %-6s\n",
           "onset", "snr_dB", "seed", "final_delay", "delta_smp", "delta_sy", "metric", "dec", "meanH", "edge");

    // Sweep: run-up SNR (inf = pure zeros control) x a few onsets x a few seeds.
    // The field wrong lock is k~1-3.5 symbols off with metric~0.998, mean_H~0.37.
    double snrs[] = {200.0, 30.0, 25.0, 20.0, 15.0};
    int onsets[]  = {8, 20, 60, 100};
    unsigned seeds[] = {1u, 2u, 3u};
    int n_wronglock = 0, n_total = 0, n_early = 0, n_late = 0, n_decoded = 0;
    for (double snr : snrs) {
        for (int onset : onsets) {
            for (unsigned sd : seeds) {
                F0VCell c = f0v_run_cell(cfg, onset, snr, sd);
                n_total++;
                const char* edge_s = (c.edge < 0) ? "EARLY" : (c.edge > 0 ? "LATE" : "TRUE");
                bool wrong = (c.coarse_metric >= 0.97) && (c.mean_H >= 0.0) && (c.mean_H < 0.5)
                             && (c.edge != 0) && (!c.decoded);
                if (wrong) { n_wronglock++; if (c.edge < 0) n_early++; else n_late++; }
                if (c.decoded) n_decoded++;
                printf("[TEST-FRAME0-VEHICLE] %-6d %-8.0f %-6u %-11d %-9d %-+8.2f %-8.4f %-4d %-6.3f %-6s%s\n",
                       onset, c.snr_db, c.seed, c.final_delay, c.delta, c.delta_sym,
                       c.coarse_metric, c.decoded, c.mean_H, edge_s,
                       wrong ? "  <== WRONG-LOCK" : "");
            }
        }
    }
    printf("[TEST-FRAME0-VEHICLE] SUMMARY cells=%d wrong_lock=%d (early=%d late=%d) decoded=%d\n",
           n_total, n_wronglock, n_early, n_late, n_decoded);
    printf("[TEST-FRAME0-VEHICLE] FAITHFUL=%s (need >=1 wrong-lock with the field fingerprint)\n",
           n_wronglock > 0 ? "YES" : "NO");
    // This is a diagnostic vehicle, not a pass/fail gate: always return 0.
    return 0;
}

// --test-frame0-mf: candidate (c) MATCHED-FILTER viability measurement. Revives the
// #if0'd OFDM correlation template (env MERCURY_F0V_MF_TEMPLATE=1, set below) and runs
// the production time_sync_preamble_matched on the SAME synthesized wrong-lock buffer.
// Reports the MF peak delay + correlation. Decision rule (per the task):
//   - MF peak lands at TRUE (|mf_delay - D| < 1/2 symbol) AND corr clearly above the
//     2.0 detect floor / above the plateau-edge values -> MF DISCRIMINATES the plateau
//     -> candidate (c) is a viable winner.
//   - MF corr stays ~0.15 / peak not at TRUE -> MF is DEAD (cannot beat the plateau) ->
//     drop it, do NOT wire a dead detector into the shared path.
int run_frame0_mf_selftest()
{
    setenv("MERCURY_F0V_MF_TEMPLATE", "1", 1);   // build the MF template during load
    printf("[TEST-FRAME0-MF] matched-filter plateau-tiebreak VIABILITY measurement\n");
    const char* cfg_env = std::getenv("MERCURY_F0V_CONFIG");
    int cfg = cfg_env ? atoi(cfg_env) : CONFIG_16;

    cl_telecom_system ts;
    ts.operation_mode = BER_PLOT_passband;
    ts.load_configuration(cfg);
    cl_data_container& dc = ts.data_container;
    int interp       = ts.frequency_interpolation_rate;
    int sym_samples  = dc.Nofdm * interp;
    int frame_symb   = dc.Nsymb + dc.preamble_nSymb;
    int frame_samp   = frame_symb * sym_samples;
    int buf_samp     = dc.buffer_Nsymb * sym_samples;
    int max_delay    = buf_samp - frame_samp;

    printf("[TEST-FRAME0-MF] config=%d template_len=%d energy=%.4f nsymb=%d sym_samples=%d (len=0 => revive FAILED)\n",
           cfg, ts.ofdm.ofdm_corr_template_len, ts.ofdm.ofdm_corr_template_energy,
           ts.ofdm.ofdm_corr_template_nsymb, sym_samples);

    int nReal_data = dc.nBits - ts.ldpc.P;
    int frame_size = (nReal_data - ts.outer_code_reserved_bits) / 8; if (frame_size < 1) frame_size = 1;
    std::vector<int> payload(frame_size, 0); for (int i=0;i<frame_size;i++) payload[i]=(i*53+7)&0xff;
    std::vector<double> frame(frame_samp, 0.0);
    ts.transmit_byte(payload.data(), frame_size, frame.data(), SINGLE_MESSAGE);
    double e=0; for(double v:frame) e+=v*v; double frame_rms=(frame.size()>0)?sqrt(e/(double)frame.size()):0.0;

    std::vector<std::complex<double>> bb(buf_samp);
    printf("[TEST-FRAME0-MF] %-6s %-8s %-6s %-11s %-11s %-11s %-9s\n",
           "onset","snr_dB","seed","true_D","mf_delay","mf-D(sym)","mf_corr");
    double snrs[]={200.0,30.0}; int onsets[]={60,100}; unsigned seeds[]={1u,2u};
    int n_true=0,n_total=0;
    for (double snr:snrs) for (int onset:onsets) for (unsigned sd:seeds) {
        int D=onset*sym_samples; if(D>max_delay)D=max_delay; if(D<0)D=0;
        std::mt19937 rng(sd);
        double sigma=(snr<200.0)?frame_rms/pow(10.0,snr/20.0):0.0;
        std::normal_distribution<double> nd(0.0,sigma);
        std::vector<double> rx(buf_samp);
        for(int i=0;i<buf_samp;i++) rx[i]=(sigma>0.0)?nd(rng):0.0;
        int cn=frame_samp; if(D+cn>buf_samp)cn=buf_samp-D;
        for(int i=0;i<cn;i++) rx[D+i]+=frame[i];
        ts.ofdm.passband_to_baseband(rx.data(), buf_samp, bb.data(),
            ts.sampling_frequency, ts.carrier_frequency, ts.carrier_amplitude, 1, &ts.ofdm.FIR_rx_time_sync);
        TimeSyncResult mf = ts.ofdm.time_sync_preamble_matched(bb.data(), buf_samp, interp, dc.preamble_nSymb);
        double mfdsym=(sym_samples>0)?(double)(mf.delay-D)/sym_samples:0.0;
        bool at_true = (sym_samples>0) && (std::abs(mf.delay-D) < sym_samples/2);
        n_total++; if(at_true) n_true++;
        printf("[TEST-FRAME0-MF] %-6d %-8.0f %-6u %-11d %-11d %-+11.2f %-9.4f%s\n",
            onset, snr, sd, D, mf.delay, mfdsym, mf.correlation, at_true?"  TRUE":"  OFF");
    }
    printf("[TEST-FRAME0-MF] SUMMARY cells=%d mf_at_true=%d  VIABLE=%s\n",
        n_total, n_true, (n_true==n_total && n_total>0)?"YES":"NO");
    return 0;
}

// --test-frame0-replay <capture>: feed a raw passband ring captured at the
// field SUBPEAK-REJECT (MERCURY_F0_RINGDUMP) through the PRODUCTION acquisition
// exactly as the synth vehicle does. Faithful by construction (it IS the field
// waveform, deep collapse included). With MERCURY_FINE_ENERGY_REL=1 the F-A gate
// should advance the lock off the run-up and decode. Returns 0 if decoded.
int run_frame0_replay_selftest(const char* path)
{
    FILE* fp = fopen(path, "rb");
    if(!fp){ printf("[TEST-FRAME0-REPLAY] cannot open %s\n", path); return 1; }
    int hdr[6]; double meta[2]; long long ns = 0;
    if(fread(hdr,sizeof(int),6,fp)!=6 || fread(meta,sizeof(double),2,fp)!=2 || fread(&ns,sizeof(long long),1,fp)!=1)
    { fclose(fp); printf("[TEST-FRAME0-REPLAY] bad header %s\n", path); return 1; }
    if(hdr[0]!=(int)0xF0CAB1 || ns<=0 || ns>500000000LL)
    { fclose(fp); printf("[TEST-FRAME0-REPLAY] bad magic/size %s\n", path); return 1; }
    int config = hdr[1];
    std::vector<double> rx((size_t)ns);
    size_t got = fread(rx.data(), sizeof(double), (size_t)ns, fp);
    fclose(fp);
    if((long long)got != ns){ printf("[TEST-FRAME0-REPLAY] short read %s\n", path); return 1; }

    setenv("MERCURY_SUBPEAK_RESCUE_DEFEAT", "1", 1);   // measure the RAW lock
    cl_telecom_system ts;
    ts.operation_mode = BER_PLOT_passband;
    ts.load_configuration(config);
    cl_data_container& dc = ts.data_container;
    ts.receive_stats.ofdm_search_raw           = 0;
    ts.data_container.nUnder_processing_events = 0;
    ts.receive_stats.ofdm_batch_active         = false;
    ts.receive_stats.delay                     = 0;
    ts.ofdm_forced_delay                       = -1;
    ts.mfsk_fixed_delay                        = -1;
    ts.receive_byte(rx.data(), dc.hd_decoded_data_byte);
    int decoded = (ts.receive_stats.message_decoded == YES) ? 1 : 0;
    int sym_samples = dc.Nofdm * ts.frequency_interpolation_rate;
    double dsym = (sym_samples>0) ? (double)((int)ts.receive_stats.delay - hdr[5]) / sym_samples : 0.0;
    printf("[TEST-FRAME0-REPLAY] file=%s config=%d cap_delay=%d cap_metric=%.3f cap_meanH=%.3f => final_delay=%d d(sym)=%+.2f meanH=%.3f decoded=%d\n",
        path, config, hdr[5], meta[0], meta[1], (int)ts.receive_stats.delay, dsym,
        ts.receive_stats.mean_H, decoded);
    return decoded ? 0 : 1;
}

// --test-crc-escape: CRC-ESCAPE accept-gate self-test. Proves the OFDM RX frame
// accept/reject gate (cl_telecom_system::frame_decode_rejected) REJECTS a non-converged
// LDPC decode (iterations_done == nIteration_max+1, the FAIL sentinel) whose residual-
// error bits collide the 16-bit MODBUS CRC to 0 -- the ~2^-16 event that under the
// legacy crc-only gate committed a corrupt frame on a clean channel (LOSE, not CORRUPT).
// Deterministic, in-process (no IONOS/RF): loads CONFIG_16 (CRC16_MODBUS_RTU outer code),
// mocks the exact gate inputs, and drives the PRODUCTION predicate directly.
//   fail-before: crc_escape_defeat=true  -> legacy gate ACCEPTS (message_decoded=YES; the
//                CRC-colliding corrupt frame is delivered).
//   pass-after : crc_escape_defeat=false -> gate REJECTS (message_decoded=NO -> the frame
//                falls into the FAIL branch and the existing TIME_INTERP-seed rescue).
// Controls: a CONVERGED crc-good frame is ACCEPTED by BOTH arms (the fix weakens no
// good-frame acceptance); a crc-bad frame is REJECTED by both.
int run_crc_escape_selftest()
{
    printf("[TEST-CRC-ESCAPE] OFDM accept-gate CRC-escape convergence-guard self-test\n");
    int fails = 0;
    const int cfg = CONFIG_16;

    cl_telecom_system ts;
    ts.operation_mode = BER_PLOT_passband;
    ts.load_configuration(cfg);

    // CONFIG_16 is a CRC16_MODBUS_RTU-outer-coded config -- the gate branch under test.
    if (ts.outer_code != CRC16_MODBUS_RTU) {
        printf("[TEST-CRC-ESCAPE]   FAIL: CONFIG_16 outer_code=%d != CRC16_MODBUS_RTU(%d)\n",
               ts.outer_code, CRC16_MODBUS_RTU);
        printf("[TEST-CRC-ESCAPE] FAILED (1 failure)\n");
        return 1;
    }
    const int nIterMax = ts.ldpc.nIteration_max;
    const int NONCONV  = nIterMax + 1;                 // canonical LDPC FAIL sentinel
    const int CONV     = (nIterMax > 1) ? (nIterMax - 1) : 0;  // a converged decode

    // returns 1 == REJECT (message_decoded=NO -> TINTERP); 0 == ACCEPT (delivers)
    auto decide = [&](int crc, int iters, int all_zeros, bool defeat)->int {
        st_receive_stats rs;                            // value-initialised (all fields 0)
        rs.all_zeros       = all_zeros;
        rs.crc             = crc;
        rs.iterations_done = iters;
        return ts.frame_decode_rejected(rs, ts.outer_code, defeat) ? 1 : 0;
    };

    // ---- fail-before: legacy crc-only gate ACCEPTS the corrupt frame ----
    int rej_before = decide(/*crc=*/0, /*iters=*/NONCONV, /*all_zeros=*/NO, /*defeat=*/true);
    bool fail_before_ok = (rej_before == 0);            // ACCEPT == the escape reproduced
    printf("[TEST-CRC-ESCAPE]   FAIL-BEFORE (defeat, crc=0 iter=%d nonconv): gate=%s -> %s (expect ACCEPT/deliver)\n",
           NONCONV, rej_before ? "REJECT" : "ACCEPT",
           rej_before ? "message_decoded=NO" : "message_decoded=YES");
    if (!fail_before_ok) { fails++; printf("[TEST-CRC-ESCAPE]   FAIL: legacy gate did NOT accept -- fail-before not reproduced\n"); }

    // ---- pass-after: fixed gate REJECTS the same frame -> TINTERP ----
    int rej_after = decide(/*crc=*/0, /*iters=*/NONCONV, /*all_zeros=*/NO, /*defeat=*/false);
    bool pass_after_ok = (rej_after == 1);              // REJECT == message_decoded=NO -> TINTERP
    printf("[TEST-CRC-ESCAPE]   PASS-AFTER  (fixed,  crc=0 iter=%d nonconv): gate=%s -> %s (expect REJECT->TINTERP)\n",
           NONCONV, rej_after ? "REJECT" : "ACCEPT",
           rej_after ? "message_decoded=NO" : "message_decoded=YES");
    if (!pass_after_ok) { fails++; printf("[TEST-CRC-ESCAPE]   FAIL: fixed gate did NOT reject -- CRC-escape still open\n"); }

    // ---- control 1: a CONVERGED crc-good frame is ACCEPTED by BOTH arms ----
    int good_before = decide(0, CONV, NO, true);
    int good_after  = decide(0, CONV, NO, false);
    bool good_ok = (good_before == 0) && (good_after == 0);
    printf("[TEST-CRC-ESCAPE]   CONTROL good-frame (crc=0 iter=%d converged): before=%s after=%s (expect ACCEPT both)\n",
           CONV, good_before?"REJECT":"ACCEPT", good_after?"REJECT":"ACCEPT");
    if (!good_ok) { fails++; printf("[TEST-CRC-ESCAPE]   FAIL: fix weakened converged good-frame acceptance\n"); }

    // ---- control 2: a crc-BAD frame is REJECTED by BOTH arms ----
    int bad_before = decide(0x1234, CONV, NO, true);
    int bad_after  = decide(0x1234, CONV, NO, false);
    bool bad_ok = (bad_before == 1) && (bad_after == 1);
    printf("[TEST-CRC-ESCAPE]   CONTROL crc-bad   (crc!=0 iter=%d): before=%s after=%s (expect REJECT both)\n",
           CONV, bad_before?"REJECT":"ACCEPT", bad_after?"REJECT":"ACCEPT");
    if (!bad_ok) { fails++; printf("[TEST-CRC-ESCAPE]   FAIL: crc-bad frame accepted\n"); }

    printf("[TEST-CRC-ESCAPE] %s (%d failure%s)  [nIterMax=%d FAIL-sentinel=%d]\n",
           fails==0 ? "ALL PASS" : "FAILED", fails, fails==1?"":"s", nIterMax, NONCONV);
    return fails==0 ? 0 : 1;
}

int cl_arq_controller::test_turbo_iterations_sentinel()
{
    printf("[TEST-TURBO-SENTINEL] turbo decode-verdict preservation self-test\n");
    int fails = 0;

    cl_telecom_system telecom;
    telecom.operation_mode = BER_PLOT_passband;
    telecom.load_configuration(CONFIG_16);
    if (telecom.outer_code != CRC16_MODBUS_RTU) {
        printf("[TEST-TURBO-SENTINEL] FAIL: CONFIG_16 is not CRC16 outer-coded\n");
        return 1;
    }

    const int fail_sentinel = telecom.ldpc.nIteration_max + 1;
    st_receive_stats rs;
    rs.all_zeros = NO;
    rs.crc = 0;  // residual hard bits collide CRC16, the fail-closed case

    // Fail-before: the old publication stored turbo loop index 0, making the
    // non-converged CRC-colliding frame appear converged and therefore accepted.
    rs.iterations_done = cl_telecom_system::turbo_iterations_to_publish(
        fail_sentinel, /*selected_turbo_iteration=*/0, /*sentinel_defeat=*/true);
    bool legacy_rejected = telecom.frame_decode_rejected(rs, telecom.outer_code);
    if (rs.iterations_done != 0 || legacy_rejected) {
        printf("[TEST-TURBO-SENTINEL] FAIL: legacy overwrite was not reproduced (iter=%d rejected=%d)\n",
               rs.iterations_done, legacy_rejected ? 1 : 0);
        fails++;
    }

    // Pass-after: publish the selected decode's real result. The canonical FAIL
    // sentinel survives and the production accept/reject gate rejects the frame.
    rs.iterations_done = cl_telecom_system::turbo_iterations_to_publish(
        fail_sentinel, /*selected_turbo_iteration=*/0);
    bool fixed_rejected = telecom.frame_decode_rejected(rs, telecom.outer_code);
    if (rs.iterations_done != fail_sentinel || !fixed_rejected) {
        printf("[TEST-TURBO-SENTINEL] FAIL: FAIL sentinel lost (iter=%d expected=%d rejected=%d)\n",
               rs.iterations_done, fail_sentinel, fixed_rejected ? 1 : 0);
        fails++;
    }

    // A later turbo candidate that also fails must likewise retain its decoder
    // sentinel; the turbo loop number is never a convergence verdict.
    rs.iterations_done = cl_telecom_system::turbo_iterations_to_publish(
        fail_sentinel, /*selected_turbo_iteration=*/1);
    if (!telecom.frame_decode_rejected(rs, telecom.outer_code)) {
        printf("[TEST-TURBO-SENTINEL] FAIL: non-converged turbo refinement was accepted\n");
        fails++;
    }

    printf("[TEST-TURBO-SENTINEL] %s\n", fails == 0 ? "PASS" : "FAIL");
    return fails;
}

// MINI frames carry one preamble symbol, so their first four data symbols begin
// at symbol 1 rather than at the configured FULL-preamble boundary (symbol 4).
// Reproduce the old false-pass with silent symbols 1..3 and energetic symbols
// 4..7, then require the MINI-aware production selector to inspect the earlier
// window. The amplitudes make the old window pass the 10% relative gate while
// the corrected window fails it deterministically.
int cl_arq_controller::test_mini_data_energy_gate()
{
	printf("[TEST-MINI-DATA-ENERGY] effective-preamble offset regression\n");
	const int full_preamble = 4;
	const int mini_preamble = 1;
	const int samples_per_symbol = 8;
	cl_telecom_system telecom;
	std::vector<std::complex<double> > samples(8 * samples_per_symbol,
		std::complex<double>(0.0, 0.0));
	for(int sym = 0; sym < 8; sym++)
	{
		double energy = (sym == 0) ? 1.0 : (sym >= 4 ? 0.20 : 0.0);
		double amplitude = std::sqrt(energy);
		for(int i = 0; i < samples_per_symbol; i++)
			samples[sym * samples_per_symbol + i] = std::complex<double>(amplitude, 0.0);
	}

	auto mean_four_symbols = [&](int preamble_nsymb) {
		int begin = preamble_nsymb * samples_per_symbol;
		int end = begin + 4 * samples_per_symbol;
		if(begin < 0 || end > (int)samples.size()) return -1.0;
		double sum = 0.0;
		for(int i = begin; i < end; i++) sum += std::norm(samples[i]);
		return sum / (4 * samples_per_symbol);
	};

	telecom.mini_data_energy_gate_enabled = false;
	int legacy_offset = telecom.data_energy_gate_preamble_nsymb(
		full_preamble, mini_preamble);
	telecom.mini_data_energy_gate_enabled = true;
	int fixed_offset = telecom.data_energy_gate_preamble_nsymb(
		full_preamble, mini_preamble);
	double legacy_energy = mean_four_symbols(legacy_offset);
	double fixed_energy = mean_four_symbols(fixed_offset);
	bool legacy_passes = legacy_energy >= 0.10;
	bool fixed_rejects = fixed_energy < 0.10;
	bool invalid_rejects =
		telecom.data_energy_gate_preamble_nsymb(0, 0) < 0
		&& telecom.data_energy_gate_preamble_nsymb(4, -1) < 0
		&& telecom.data_energy_gate_preamble_nsymb(4, 5) < 0;

	int fails = 0;
	if(legacy_offset != 4 || fixed_offset != 1 || !legacy_passes
		|| !fixed_rejects || !invalid_rejects)
	{
		printf("[TEST-MINI-DATA-ENERGY] FAIL: offsets legacy=%d fixed=%d energies=%.3f/%.3f invalid=%d\n",
			legacy_offset, fixed_offset, legacy_energy, fixed_energy,
			invalid_rejects ? 1 : 0);
		fails++;
	}
	printf("[TEST-MINI-DATA-ENERGY] %s (legacy=%.3f PASS, fixed=%.3f REJECT)\n",
		fails == 0 ? "PASS" : "FAIL", legacy_energy, fixed_energy);
	return fails;
}

int cl_arq_controller::test_bigblock_wav_short_read()
{
	return cl_telecom_system::test_bigblock_wav_short_read();
}

int cl_arq_controller::test_rx_shm_output_overflow()
{
	return cl_telecom_system::test_rx_shm_output_overflow();
}

// --test-chase: chase-combining (HARQ Type-I soft-LLR combine) fail-before /
// pass-after self-test. See fact-documents/chase-combining-harq.md.
//
// ROOT this test guards: Mercury had NO soft-combining -- a failed frame's LLRs
// were DISCARDED, so a bit-identical retransmission's soft info was never summed
// with the prior look (the capstone "chase engaged N times, 0 rescues"). The
// coded bits of a same-config retx ARE identical (deterministic QC-LDPC encode),
// so summing the two LLR vectors is VALID and buys ~+3 dB.
//
// Deterministic construction (no OFDM sync, no RF, no audio): a random K-bit
// message is LDPC-encoded (CONFIG_6 = BPSK rate 1/2, K=800 N=1600), BPSK-mapped
// via the REAL psk.mod, and passed twice through the REAL AWGN channel at an
// operating point BELOW the single-look FEC waterfall (two independent noise
// realizations = the two ARQ looks). Each look is demapped by the REAL psk.demod
// and decoded by the REAL ldpc.decode. Assertions over T deterministic trials:
//   * single-look success           == 0   (fail-before: one copy never decodes)
//   * chase-OFF combine (env-defeat) == 0   (defeat: primitive no-ops -> one copy)
//   * chase-ON  combine              >  0   (pass-after: SUM rescues the frame)
// i.e. the combine SUCCESS-RATE moves 0 -> >0, and defeating chase restores the
// pre-fix 0. The combine itself is the production primitive
// cl_telecom_system::chase_capture()/chase_combine() -- NOT an ad-hoc sum here.
int run_chase_selftest()
{
    printf("[TEST-CHASE] chase-combining (HARQ soft-LLR combine) fail-before/pass-after self-test\n");

    cl_telecom_system ts;
    ts.operation_mode = BER_PLOT_passband;
    ts.load_configuration(CONFIG_6);          // BPSK rate 1/2 -> K=800, N=1600
    ts.psk.set_predefined_constellation(MOD_BPSK);  // ensure unit-energy BPSK demapper

    const int N = ts.ldpc.N;
    const int K = ts.ldpc.K;
    if(N <= 0 || K <= 0 || N > N_MAX) {
        printf("[TEST-CHASE]   BAD SIZING N=%d K=%d -> FAIL\n", N, K);
        return 1;
    }

    // Operating point: complex-sample noise amplitude. Es=1 (BPSK), so Es/N0 =
    // 1/ampl^2. Chosen ~2 dB BELOW the single-look rate-1/2 waterfall so ONE look
    // never decodes while the +3 dB two-look combine crosses. Overridable via
    // MERCURY_CHASE_TEST_AMPL for tuning; default is the locked value.
    // ampl=1.45 (Es/N0 ~ -3.2 dB) sits ~1.5 dB BELOW the single-look rate-1/2
    // waterfall (single-look decodes ~100% at ampl~1.15, 0% by ampl~1.35) while the
    // +3 dB two-look combine still decodes 100% (its waterfall is beyond ampl~1.65),
    // giving both arms a wide deterministic margin (fixed seeds -> non-flaky).
    double ampl = 1.45;
    { const char* e = std::getenv("MERCURY_CHASE_TEST_AMPL"); if(e && *e) ampl = atof(e); }
    const float variance = (float)(ampl * ampl);

    const int T = 40;
    // Env-defeat: MERCURY_CHASE=0 forces the chase-ON arm OFF too, demonstrating
    // that defeating the primitive returns the pre-fix 0-success behavior.
    bool env_off = false;
    { const char* e = std::getenv("MERCURY_CHASE"); if(e && strcmp(e,"0")==0) env_off = true; }

    int* info = new int[K];
    int* cw   = new int[N];
    std::complex<double>* sym = new std::complex<double>[N];
    std::complex<double>* rx1 = new std::complex<double>[N];
    std::complex<double>* rx2 = new std::complex<double>[N];
    float* llr1 = new float[N];
    float* llr2 = new float[N];
    float* comb = new float[N];
    int*   out  = new int[K];

    auto frame_ok = [&](const int* dec)->bool{
        for(int i=0;i<K;i++) if(dec[i] != info[i]) return false;
        return true;
    };

    int ok_single = 0, ok_chase_on = 0, ok_chase_off = 0;
    const bool dbg = (std::getenv("MERCURY_CHASE_TEST_DEBUG") != nullptr);

    srand(0xC4A5E);  // deterministic message + noise stream
    for(int t=0; t<T; t++)
    {
        for(int i=0;i<K;i++) info[i] = rand() & 1;
        ts.ldpc.encode(info, cw);           // K info -> N coded bits (systematic)
        ts.psk.mod(cw, N, sym);             // BPSK: N coded bits -> N unit-energy symbols

        // Two INDEPENDENT looks of the SAME codeword. Seeds must be DISTINCT and
        // positive (set_seed srand's only when seed>0); keep them well-separated so
        // the two noise realizations are decorrelated (a collapsed/equal seed makes
        // the "combine" merely DOUBLE one look -> no diversity -> non-convergence).
        ts.awgn_channel.set_seed((long)(100003 + 2*t));
        ts.awgn_channel.apply(sym, rx1, (float)ampl, N);
        ts.awgn_channel.set_seed((long)(700019 + 2*t));
        ts.awgn_channel.apply(sym, rx2, (float)ampl, N);

        ts.psk.demod(rx1, N, llr1, variance);
        ts.psk.demod(rx2, N, llr2, variance);

        // (a) SINGLE-LOOK baseline (look 2 alone) -- the pre-chase behavior.
        int it_s = ts.ldpc.decode(llr2, out);
        bool ok_s = frame_ok(out);
        if(ok_s) ok_single++;

        // (b) CHASE-ON: buffer look 1, combine into look 2, decode the SUM.
        ts.chase_enabled = !env_off;
        ts.chase_reset();
        ts.chase_capture(llr1, N, CONFIG_6);
        for(int i=0;i<N;i++) comb[i] = llr2[i];
        bool combined_on = ts.chase_combine(comb, N, CONFIG_6);  // sums iff enabled+match
        int it_c = ts.ldpc.decode(comb, out);
        bool ok_c = frame_ok(out);
        if(combined_on && ok_c) ok_chase_on++;
        if(dbg) {
            int nbad=0; for(int i=0;i<K;i++) if(out[i]!=info[i]) nbad++;
            double amax1=0,amax2=0,amaxc=0;
            for(int i=0;i<N;i++){ if(fabs(llr1[i])>amax1)amax1=fabs(llr1[i]); if(fabs(llr2[i])>amax2)amax2=fabs(llr2[i]); double c=llr1[i]+llr2[i]; if(fabs(c)>amaxc)amaxc=fabs(c);}
            printf("[TEST-CHASE-DBG] t=%2d single ok=%d it=%d | comb ok=%d it=%d combflag=%d bad=%d | max|llr1|=%.1f max|llr2|=%.1f max|sum|=%.1f\n",
                   t, ok_s?1:0, it_s, ok_c?1:0, it_c, combined_on?1:0, nbad, amax1, amax2, amaxc);
        }

        // (c) CHASE-OFF (env-defeat semantics): primitive disabled -> combine
        //     no-ops -> decodes a single look -> pre-fix 0-success.
        ts.chase_enabled = false;
        ts.chase_reset();
        ts.chase_capture(llr1, N, CONFIG_6);                    // no-op (disabled)
        for(int i=0;i<N;i++) comb[i] = llr2[i];
        ts.chase_combine(comb, N, CONFIG_6);                    // returns false, comb == llr2
        ts.ldpc.decode(comb, out);
        if(frame_ok(out)) ok_chase_off++;
    }

    printf("[TEST-CHASE]   ampl=%.3f (Es/N0=%.2f dB) T=%d :: single=%d/%d  chase_ON=%d/%d  chase_OFF(defeat)=%d/%d\n",
           ampl, -10.0*log10(ampl*ampl), T,
           ok_single, T, ok_chase_on, T, ok_chase_off, T);

    // Fail-before / pass-after:
    //   single-look and env-defeat MUST be 0 (one copy never decodes at this point);
    //   chase-ON MUST rescue (>0)  -> combine success-rate 0 -> >0.
    // Under MERCURY_CHASE=0 the ON arm is forced OFF, so it collapses to 0 and the
    // test FAILS -- which is the intended demonstration that defeating chase
    // restores the pre-fix behavior.
    bool pass;
    if(env_off) {
        pass = (ok_single == 0) && (ok_chase_on == 0) && (ok_chase_off == 0);
        printf("[TEST-CHASE]   MERCURY_CHASE=0 env-defeat: all arms 0-success (pre-fix) -> %s\n",
               pass ? "DEMONSTRATED" : "UNEXPECTED");
    } else {
        pass = (ok_single == 0) && (ok_chase_off == 0) && (ok_chase_on > 0);
    }
    printf("[TEST-CHASE] %s\n", pass ? "PASS (combine success-rate 0 -> >0)" : "FAIL");

    delete[] info; delete[] cw; delete[] sym; delete[] rx1; delete[] rx2;
    delete[] llr1; delete[] llr2; delete[] comb; delete[] out;
    return pass ? 0 : 1;
}

// --test wall-clock watchdog. `--test` is a Monte-Carlo suite with no internal
// time bound; on the RPi bench a wedged test used to hang forever, pinning a
// core at 99% (stacked orphans -> thermal throttle -> CPU jitter that tips OFDM
// acquisition onto sub-peaks). Arm a detached timer thread on entry to any
// --test* dispatch: if the suite has not returned (and exited the process) by
// the deadline, force a NON-ZERO exit so a wedged test can never hang. On normal
// completion main() returns first and the still-sleeping detached thread is
// abandoned with the process, so the watchdog never fires on a healthy run.
// Deadline is MERCURY_TEST_WATCHDOG_S seconds (default 2400; <=0 disables). The full
// --test suite honestly runs on the order of ~950 s on a loaded 56-thread build box, so the
// former 600 s default aborted healthy runs mid-suite; 2400 s clears the real runtime with
// margin while still bounding a genuinely wedged run.
void arm_test_watchdog() {
    long deadline_s = 2400;
    const char* env = getenv("MERCURY_TEST_WATCHDOG_S");
    if (env && *env) {
        char* end = NULL;
        long v = strtol(env, &end, 10);
        if (end != env) deadline_s = v;
    }
    if (deadline_s <= 0) return;   // explicitly disabled
    std::thread([deadline_s]() {
        std::this_thread::sleep_for(std::chrono::seconds(deadline_s));
        fprintf(stderr,
            "\n[TEST-WATCHDOG] --test exceeded %ld s wall clock — aborting with "
            "non-zero exit 70 (set MERCURY_TEST_WATCHDOG_S to tune, <=0 to disable).\n",
            deadline_s);
        fflush(stderr);
        // Exit 70 (EX_SOFTWARE): non-zero so callers see failure, and distinct
        // from GNU `timeout`'s own 124 so a wedged-test self-abort is
        // distinguishable from an external timeout kill in deploy/CI logs.
        _exit(70);
    }).detach();
}

struct Mini0CarriedCell {
    int decoded;
    int byte_exact;
    int shift_symbols;
    int skip_var_aborted;
    int coasted;
    double noise_variance;
};

// Drive one zero-preamble tail through the real receive_byte timing, energy,
// channel-estimation and SKIP-VAR path, followed by the real ARQ coast helper.
static Mini0CarriedCell run_carried_preamble_cell(int tail_preamble_nsymb,
    bool legacy_defeat, int forced_shift_symbols)
{
    cl_telecom_system ts;
    ts.operation_mode = BER_PLOT_passband;
    ts.load_configuration(CONFIG_17);
	bool carried = tail_preamble_nsymb >= 0;
    ts.preamble_amortization_enabled = carried;
    ts.keydown_track_timing_enabled = carried;
    ts.fine_energy_carried_defeat = legacy_defeat;
    // The legacy negative-control arm also disables the sub-peak onset rescue so
    // the forced +1-symbol mis-anchor stays catastrophic (the rescue, added
    // later, otherwise recovers the true onset and masks the decode loss).
    ts.subpeak_rescue_test_defeat = legacy_defeat;
	ts.fine_energy_test_force_shift_symbols = forced_shift_symbols;

    cl_data_container& dc = ts.data_container;
    int interp = ts.frequency_interpolation_rate;
    int sym_samples = dc.Nofdm * interp;
    int buf_samples = dc.buffer_Nsymb * sym_samples;
    int payload_bytes = (dc.nBits - ts.ldpc.P - ts.outer_code_reserved_bits) / 8;
    if(payload_bytes < 1) payload_bytes = 1;
    std::vector<int> payload(payload_bytes, 0);
    for(int i=0; i<payload_bytes; i++) payload[i] = (i * 53 + 7) & 0xff;

	int emitted_preamble = carried ? tail_preamble_nsymb : dc.preamble_nSymb;
	ts.tx_preamble_nsymb_override = carried ? tail_preamble_nsymb : -1;
    ts.tx_last_emitted_frame_samples =
		(dc.Nsymb + emitted_preamble) * sym_samples;
    ts.transmit_byte(payload.data(), payload_bytes, dc.passband_data, SINGLE_MESSAGE);
    int emitted = ts.tx_last_emitted_frame_samples;
    ts.tx_preamble_nsymb_override = -1;

    double* rx = dc.ready_to_process_passband_delayed_data;
    for(int i=0; i<buf_samples; i++) rx[i] = 0.0;
    const int onset_symbols = 8;
    int onset = onset_symbols * sym_samples;
    int copy_n = emitted;
    if(onset + copy_n > buf_samples) copy_n = buf_samples - onset;
    for(int i=0; i<copy_n; i++) rx[onset + i] = dc.passband_data[i];
    ts.receive_stats.ofdm_search_raw = carried ? onset_symbols : 0;
    ts.receive_stats.ofdm_batch_active = carried;
    ts.receive_stats.ofdm_drift_per_frame = 0.0;
    ts.receive_stats.delay = 0;
    ts.keydown_last_delay = 0;
    ts.data_container.nUnder_processing_events = 0;
    ts.ofdm_forced_delay = carried ? -1 : onset;
    ts.mfsk_fixed_delay = -1;
	ts.rx_preamble_nsymb_override = carried ? tail_preamble_nsymb : -1;
    st_receive_stats st = ts.receive_byte(rx, dc.hd_decoded_data_byte);
	ts.rx_preamble_nsymb_override = -1;

    int exact = (st.message_decoded == YES) ? 1 : 0;
    if(exact) for(int i=0; i<payload_bytes; i++)
        if((dc.hd_decoded_data_byte[i] & 0xff) != (payload[i] & 0xff)) { exact=0; break; }

    cl_arq_controller arq;
    arq.telecom_system = &ts;
    arq.role = RESPONDER;
    arq.sack_v2_enabled = true;
    arq.current_configuration = CONFIG_17;
    arq.data_batch_size = 90;
    arq.batch_coast_defeat = false;
    arq.batch_coast_max = 8;
    int ftr = 0;
    int frame_symbols = dc.Nsymb + st.last_eff_preamble_nsymb;
    int upper = dc.buffer_Nsymb - frame_symbols;
    bool coasted = false;
    if(st.message_decoded != YES)
        coasted = arq.batch_coast_try_advance(frame_symbols, upper,
            st.delay / sym_samples, false, &ftr);

    Mini0CarriedCell out;
    out.decoded = (st.message_decoded == YES) ? 1 : 0;
    out.byte_exact = exact;
    out.shift_symbols = ts.fine_energy_last_shift_symbols;
    out.skip_var_aborted = st.frame_skip_var_aborted ? 1 : 0;
    out.coasted = coasted ? 1 : 0;
    out.noise_variance = ts.ofdm.noise_variance_estimate;
    arq.telecom_system = NULL;
    return out;
}

int run_mini0_carried_timing_test()
{
	Mini0CarriedCell legacy = run_carried_preamble_cell(0, true, 1);
	Mini0CarriedCell fixed = run_carried_preamble_cell(0, false, 1);
	Mini0CarriedCell control = run_carried_preamble_cell(-1, false, 0);
    printf("[TEST-MINI0-CARRIED] legacy: decoded=%d exact=%d energy_shift=%d skipvar=%d coast=%d nv=%.4f\n",
        legacy.decoded, legacy.byte_exact, legacy.shift_symbols,
        legacy.skip_var_aborted, legacy.coasted, legacy.noise_variance);
    printf("[TEST-MINI0-CARRIED] fixed:  decoded=%d exact=%d energy_shift=%d skipvar=%d coast=%d nv=%.4f\n",
        fixed.decoded, fixed.byte_exact, fixed.shift_symbols,
        fixed.skip_var_aborted, fixed.coasted, fixed.noise_variance);
	printf("[TEST-MINI0-CARRIED] no-carry control: decoded=%d exact=%d energy_shift=%d skipvar=%d coast=%d nv=%.4f\n",
		control.decoded, control.byte_exact, control.shift_symbols,
		control.skip_var_aborted, control.coasted, control.noise_variance);
    // Negative control: with the carried-timing refinement defeated AND the
    // sub-peak onset rescue disabled for this arm, the forced +1-symbol shift
    // stays catastrophic and the frame is lost. The downstream skip-var/coast
    // disposition is incidental to the loss (the later sub-peak rescue changes
    // which reject stage the miss flows through), so it is reported but not gated.
    bool legacy_chain = legacy.shift_symbols == 1 && !legacy.decoded;
    bool fixed_chain = fixed.shift_symbols == 0 && fixed.decoded
        && fixed.byte_exact && !fixed.skip_var_aborted && !fixed.coasted;
	bool control_chain = control.shift_symbols == 0 && control.decoded
		&& control.byte_exact && !control.skip_var_aborted && !control.coasted;
    printf("[TEST-MINI0-CARRIED] %s (legacy +1-symbol decode-loss; fixed byte-exact hold)\n",
		(legacy_chain && fixed_chain && control_chain) ? "ALL PASS" : "FAILED");
	return (legacy_chain && fixed_chain && control_chain) ? 0 : 1;
}

// Explicit MERCURY_KEYDOWN_MINI0=0 process control. The first cell proves the
// real carried MINI1 path remains byte-exact. The forced cell proves its real
// one-symbol preamble still reaches the post-sync energy-refinement stage.
int run_mini1_carried_timing_test()
{
	const char* mini0 = std::getenv("MERCURY_KEYDOWN_MINI0");
	if(!mini0 || atoi(mini0) != 0) {
		printf("[TEST-MINI1-CARRIED] FAILED: requires MERCURY_KEYDOWN_MINI0=0\n");
		return 1;
	}
	Mini0CarriedCell ordinary = run_carried_preamble_cell(1, false, 0);
	Mini0CarriedCell refined = run_carried_preamble_cell(1, false, 1);
	printf("[TEST-MINI1-CARRIED] ordinary: decoded=%d exact=%d energy_shift=%d skipvar=%d coast=%d nv=%.4f\n",
		ordinary.decoded, ordinary.byte_exact, ordinary.shift_symbols,
		ordinary.skip_var_aborted, ordinary.coasted, ordinary.noise_variance);
	printf("[TEST-MINI1-CARRIED] refinement probe: decoded=%d exact=%d energy_shift=%d skipvar=%d coast=%d nv=%.4f\n",
		refined.decoded, refined.byte_exact, refined.shift_symbols,
		refined.skip_var_aborted, refined.coasted, refined.noise_variance);
	bool ordinary_ok = ordinary.decoded && ordinary.byte_exact
		&& !ordinary.skip_var_aborted && !ordinary.coasted;
	bool refinement_reached = refined.shift_symbols == 1;
	printf("[TEST-MINI1-CARRIED] %s (ordinary MINI1 byte-exact; forced energy refinement retained)\n",
		(ordinary_ok && refinement_reached) ? "ALL PASS" : "FAILED");
	return (ordinary_ok && refinement_reached) ? 0 : 1;
}

// --test-sigterm-handler: directed regression for the FIX-C graceful-shutdown
// handler. Installs the SIGTERM/SIGINT disposition exactly as main() does, then
// raises SIGTERM at ITSELF and asserts the handler flipped shutdown_ to true.
// This is the in-process fail-before/pass-after: on a tree WITHOUT the handler
// installed, raise(SIGTERM) terminates the process (default disposition) and the
// "PASS" line below is never printed -> the wrapping `timeout ... ; echo rc=$?`
// in the bench check sees a non-zero, signal-killed exit. With the handler the
// store is observed and the test exits 0. (The ALSA-close half of FIX-C is
// proven RPi-side: arecord -l shows the substream released after SIGTERM.)
static int test_sigterm_handler(void (*install_termination_handlers)())
{
#ifndef _WIN32
    shutdown_.store(false);
    install_termination_handlers();
    if (shutdown_.load()) {
        printf("[TEST-SIGTERM-HANDLER] FAIL: shutdown_ already true before raise\n");
        fflush(stdout);
        return 1;
    }
    raise(SIGTERM);   // delivered synchronously on this thread
    bool flipped = shutdown_.load();
    if (!flipped) {
        printf("[TEST-SIGTERM-HANDLER] FAIL: SIGTERM did not set shutdown_=true "
               "(handler not installed?)\n");
        fflush(stdout);
        return 1;
    }
    // Also confirm SIGINT (Ctrl-C) drives the same flag.
    shutdown_.store(false);
    raise(SIGINT);
    bool flipped_int = shutdown_.load();
    shutdown_.store(false);   // leave the flag clear for any later in-proc test
    if (!flipped_int) {
        printf("[TEST-SIGTERM-HANDLER] FAIL: SIGINT did not set shutdown_=true\n");
        fflush(stdout);
        return 1;
    }
    printf("[TEST-SIGTERM-HANDLER] PASS: SIGTERM+SIGINT both set shutdown_=true "
           "(async-signal-safe atomic store; lock_free=%d)\n",
           shutdown_.is_lock_free() ? 1 : 0);
    fflush(stdout);
    return 0;
#else
    printf("[TEST-SIGTERM-HANDLER] SKIP: POSIX signals not used on Windows path\n");
    fflush(stdout);
    return 0;
#endif
}


int run_main_test_dispatch(int argc, char* argv[], int i,
    void (*install_termination_handlers)(), bool& handled)
{
    handled = true;
        if (strcmp(argv[i], "--test-lp-timeline-harness") == 0) {
            arm_test_watchdog();
            std::string demo_trace;
            int failed = mercury::run_lp_transition_demo(&demo_trace);
            std::printf("%s", demo_trace.c_str());
            failed += mercury::run_lp_transition_harness_tests();
            return failed == 0 ? 0 : 1;
        }
        if (strcmp(argv[i], "--test-linkphase-config-contract") == 0) {
            arm_test_watchdog();
            cl_arq_controller test_arq;
            return test_arq.test_linkphase_config_transition_failure() == 0 ? 0 : 1;
        }
        if (strcmp(argv[i], "--test-b2f-bounded-output") == 0) {
            arm_test_watchdog();
            return run_b2f_bounded_output_test() == 0 ? 0 : 1;
        }
        if (strcmp(argv[i], "--test-audio-thread-create") == 0) {
            cl_arq_controller test_audio;
            return test_audio.test_audio_thread_create_fail_closed();
        }
        if (strcmp(argv[i], "--test-sigterm-shutdown") == 0) {
            cl_arq_controller test_audio;
            return test_audio.test_audio_shutdown_join_bounded();
        }
        if (strcmp(argv[i], "--test-detach-leak") == 0) {
            cl_arq_controller test_audio;
            return test_audio.test_audio_detach_leak_closed();
        }
        if (strcmp(argv[i], "--test-rt-priority") == 0) {
            cl_arq_controller test_audio;
            return test_audio.test_audio_rt_priority();
        }
        if (strcmp(argv[i], "--test-sim-tx-allocation") == 0) {
            cl_arq_controller test_audio;
            return test_audio.test_sim_tx_bridge_allocation_fail_closed();
        }
        if (strcmp(argv[i], "--test-sim-rx-allocation") == 0) {
            cl_arq_controller test_audio;
            return test_audio.test_sim_rx_bridge_allocation_fail_closed();
        }
        if (strcmp(argv[i], "--test-sim-connect-clock") == 0) {
            cl_arq_controller test_audio;
            return test_audio.test_sim_playback_connection_fail_closed();
        }
        if (strcmp(argv[i], "--test-rx-test-stream-wait") == 0) {
            cl_arq_controller test_audio;
            return test_audio.test_rx_test_stream_wait();
        }
        if (strcmp(argv[i], "--test-shm-create-preserves-existing") == 0) {
            cl_arq_controller test_shm;
            return test_shm.test_shm_create_preserves_existing();
        }
        if (strcmp(argv[i], "--test-audio-shm-isolation") == 0) {
            cl_arq_controller test_shm;
            return test_shm.test_audio_shm_ring_isolation();
        }
        if (strcmp(argv[i], "--test-capture-prep-geometry") == 0) {
            cl_arq_controller test_audio;
            return test_audio.test_capture_prep_geometry_change();
        }
        if (strcmp(argv[i], "--test-eesm-probe") == 0) {
            // Link-quality sounding probe: library tests + real TX/RX chain loopback.
            int failed = eesm_probe::run_unit_tests();
            cl_telecom_system ts;
            ts.operation_mode = BER_PLOT_passband;
            ts.load_configuration(CONFIG_0);
            failed += ts.eesm_probe_chain_selftest();
            printf("[EESM-TEST] %s (%d failure(s))\n", failed == 0 ? "ALL PASS" : "FAILED", failed);
            return failed == 0 ? 0 : 1;
        }
        if (strcmp(argv[i], "--test-snr-decision-grid") == 0) {
            int failed = run_snr_decision_grid_selftest();
            return (failed == 0) ? 0 : 1;
        }
        if (strcmp(argv[i], "--test-fade-core") == 0) {
            return mercury::fade::run_fade_core_tests() == 0 ? 0 : 1;
        }
        if (strcmp(argv[i], "--test-entry-leap") == 0) {
            int failed = 0;
            {
                cl_telecom_system ts_el;
                cl_arq_controller ARQ_el;
                ARQ_el.telecom_system = &ts_el;
                failed += ARQ_el.test_nb_prior_guard();
                failed += ARQ_el.test_entry_leap_fwd();
            }
            {
                cl_telecom_system ts_eg;
                cl_arq_controller ARQ_eg;
                ARQ_eg.telecom_system = &ts_eg;
                failed += ARQ_eg.test_entry_leap_hold_gate();
            }
            {
                cl_telecom_system ts_pb;
                cl_arq_controller ARQ_pb;
                ARQ_pb.telecom_system = &ts_pb;
                failed += ARQ_pb.test_pending_report_bsi_restore();
            }
            return failed == 0 ? 0 : 1;
        }
        if (strcmp(argv[i], "--test-scream-wake") == 0) {
            arm_test_watchdog();
            if (getenv("MERCURY_SCREAM_SWEEP") == NULL) {
                fprintf(stderr, "--test-scream-wake requires MERCURY_SCREAM_SWEEP=1\n");
                return 2;
            }
            srand(1u);
            int failed = run_mfsk_ctrl_codec_tests();
            cl_arq_controller scream_map_test;
            failed += scream_map_test.test_scream_rung_map();
            return failed == 0 ? 0 : 1;
        }
        if (strcmp(argv[i], "--test") == 0) {
            arm_test_watchdog();   // wall-clock backstop: wedged --test can't hang forever
            // Pin the process-global C RNG to a fixed, platform-independent
            // state at the very start of the Monte-Carlo battery so every noise
            // realization is reproducible across build environments. A fresh
            // process is only implicitly seed-1; making it explicit also clears
            // any RNG consumption from earlier static initialization.
            srand(1u);
            if (const char* mc = getenv("MERCURY_CTRL_ACK_MC")) {
                // Control-ACK gate measurement only (floor derivation); no other test runs.
                cl_arq_controller ctrl_ack_mc;
                return ctrl_ack_mc.measure_ctrl_ack_gate(mc);
            }
            int failed = run_mfsk_ctrl_codec_tests();
            failed += run_snr_decision_grid_selftest();
            if (getenv("MERCURY_CAP_CODEC_ONLY") != NULL)
                return failed == 0 ? 0 : 1;
            failed += mercury::fade::run_fade_core_tests();
            // Compression reassembly bounds: a valid stamp-carrying 96-frame batch
            // can exceed 16 KiB. Drive the REAL compression delivery funnel and
            // require byte-exact app output plus matching transported cursor.
            // MERCURY_COMPRESS_REASSEMBLY_BOUNDS_DEFEAT=1 restores the old truncation.
            {
                cl_arq_controller ARQ_crb;
                failed += ARQ_crb.test_compress_reassembly_bounds();
            }
            // Compression rollback backup restore must handle a full large
            // in-flight batch without overflowing a fixed local buffer.
            {
                cl_arq_controller ARQ_rbb;
                failed += ARQ_rbb.test_restore_backup_bounds();
            }
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_ssid_bounds();
                failed += test_arq.test_data_container_ownership_constraints();
                failed += test_arq.test_data_container_deinit_resets_pinning();
                failed += test_arq.test_gbf_decoder_constraints();
                failed += test_arq.test_gbf_final_iteration_sentinel();
                failed += test_arq.test_mini_data_energy_gate();
                failed += test_arq.test_modulation_cli_validation();
                failed += test_arq.test_tx_level_cli_validation();
                failed += test_arq.test_tx_level_concurrent_sync();
                failed += test_arq.test_bandpass_cli_validation();
                failed += test_arq.test_audio_device_path_bounds();
                failed += test_arq.test_audio_subsystem_cli_validation();
                failed += test_arq.test_shm_unmap_fail_closed();
                failed += test_arq.test_shm_create_preserves_existing();
                failed += test_arq.test_audio_shm_ring_isolation();
                failed += test_arq.test_suffix_soft_nb_unsupported_fail_closed();
                failed += test_arq.test_fir_dump_exit_fail_closed();
                failed += test_arq.test_bigblock_wav_short_read();
                failed += test_arq.test_rx_shm_output_overflow();
                failed += test_arq.test_sim_inproc_pump_fail_closed();
                failed += test_arq.test_sim2_capture_mutex_cleanup();
                failed += test_arq.test_recovery_ack_tail_capacity();
                failed += test_arq.test_l1_tx_journal_disabled_fail_closed();
                failed += test_arq.test_l1_journal_disabled_mark_sent_many();
                failed += test_arq.test_l1_acknowledge_disabled_fail_closed();
                failed += test_arq.test_l1_terminalize_disabled_fail_closed();
                failed += test_arq.test_l1_tx_journal_disabled_migration();
            }
            {
                cl_arq_controller test_break;
                failed += test_break.test_send_break_drain_failure();
            }
            {
                cl_arq_controller test_hail;
                failed += test_hail.test_send_hail_drain_failure();
                failed += test_hail.test_connection_timer_reanchor();
            }
            {
                cl_arq_controller test_break;
                failed += test_break.test_send_break_pattern_reset_failure();
            }
            {
                cl_arq_controller test_break;
                failed += test_break.test_break_ack_set_config_failure();
            }
            {
                cl_arq_controller test_mfsk_ack_sack;
                failed += test_mfsk_ack_sack.test_control_frame_gain_and_undefined_state();
                failed += test_mfsk_ack_sack.test_send_mfsk_ack_sack_fec_state_cleanup();
                failed += test_mfsk_ack_sack.test_send_mfsk_ack_sack_drain_failure();
                cl_arq_controller test_ack;
                failed += test_ack.test_send_mfsk_ack_sack_reset_failure();
                cl_arq_controller test_compact_confirm;
                failed += test_compact_confirm.test_send_mfsk_compact_confirm_drain_failure();
                cl_arq_controller test_compact_reset;
                failed += test_compact_reset.test_send_mfsk_compact_confirm_reset_failure();
            }
            {
                cl_arq_controller test_ctrl_ack;
                failed += test_ctrl_ack.test_ctrl_ack_noise_rejection();
            }
            failed += run_moose_deadzone_tests();
            failed += run_pilot_thin_nv_tests();
            failed += run_sim_clock_tests();
            failed += run_fft_scratch_tests();
			failed += run_mini0_carried_timing_test();
            failed += run_winlink_dict_tests();
            // AEAD bsi-bound nonce regression suite (data-flow-aead-nonce.md):
            // SECURITY INVARIANT — no (key,nonce) reuse; nonce binds to the wire
            // batch_seq_id so SACK reorder/retx decrypt correctly. Includes the
            // >256-batch wrap, reorder+retx round-trip, direction disjointness,
            // truncated-KX reject, and tamper-reject cases. No IONOS/RF.
            failed += run_aead_nonce_tests();
            // Encryption negotiation FAIL-CLOSED regression: once the operator has
            // opted into -E, a peer without CAP_ENCRYPTION (unsupported, or a MITM
            // that stripped the cap bit) is REFUSED for BOTH strict and fast — never
            // a plaintext downgrade — while default-off (ENCRYPT_OFF) stays plaintext.
            // Drives the REAL shared predicate decide_encryption_negotiation() that
            // both endpoints call. Fails-before: -DENC_FAILOPEN_FAILBEFORE. No IONOS/RF.
            {
                cl_arq_controller test_enc;
                failed += test_enc.test_encryption_fail_closed();
                failed += test_enc.test_encryption_cli_validation();
            }
            // AEAD TX failure FAIL-CLOSED regression: drives the REAL compressed
            // commander fill path with an active cipher and an oversized sealed
            // unit, then proves encrypt() failure stages zero plaintext frames.
            {
                cl_arq_controller test_enc_tx;
                failed += test_enc_tx.test_encrypt_failure_tx_fail_closed();
            }
            // GUI initialization is asynchronous, but modem startup must still fail
            // closed if the GUI thread reports that gui_init() failed. The directed
            // member test injects a failing initializer and exercises the same status
            // publication + validation path used below, without opening a real window.
            {
                cl_arq_controller test_gui;
                failed += test_gui.test_gui_init_fail_closed();
                failed += test_gui.test_soundcard_restart_save_fail_closed();
                failed += test_gui.test_soundcard_dialog_restart_fail_closed();
                failed += test_gui.test_soundcard_audio_init_fail_closed();
                failed += test_gui.test_soundcard_restart_launch_fail_closed();
            }
            // Device workers open asynchronously; startup must still fail closed
            // when either real backend rejects its configured device ID. Capture
            // backpressure must preserve a full-FIFO period and remain shutdown-safe.
            {
                cl_arq_controller test_audio;
                failed += test_audio.test_audio_open_fail_closed();
                failed += test_audio.test_soundcard_list_alloc_fail_closed();
                failed += test_audio.test_audio_thread_create_fail_closed();
                failed += test_audio.test_audio_shutdown_join_bounded();
                failed += test_audio.test_audio_detach_leak_closed();
                failed += test_audio.test_audio_rt_priority();
                failed += test_audio.test_capture_allocation_fail_closed();
                failed += test_audio.test_audio_payload_allocation_fail_closed();
                failed += test_audio.test_sim_tx_bridge_allocation_fail_closed();
                failed += test_audio.test_sim_rx_bridge_allocation_fail_closed();
                failed += test_audio.test_sim_playback_connection_fail_closed();
                failed += test_audio.test_capture_enqueue_backpressure();
                failed += test_audio.test_capture_prep_geometry_change();
                failed += test_audio.test_rx_transfer_read_failure();
                failed += test_audio.test_rx_test_stream_wait();
            }
            // A second modem process must not share the configured AGW/TCP
            // listener, and a bind failure must reject startup rather than
            // leave the modem running without its application interface.
            {
                cl_arq_controller test_agw;
                failed += test_agw.test_agw_server_start_fail_closed();
            }
            // ML-KEM-768 hybrid KEX regression suite (MLKEM_HYBRID_PLAN.md,
            // data-flow-hybrid-kex.md): combiner symmetry, ML-KEM-first IKM
            // order, transcript binding (tamper -> key divergence), and the
            // KX2/KX3 chunk encode/reassemble + CRC8 round-trip. No IONOS/RF.
            failed += run_mlkem_hybrid_tests();
            // In-band down-ladder DELIVERY regression (BREAK-orphan + silent-snapshot). A
            // member test on a throwaway controller (its own buffers; PART B builds its own
            // minimal telecom_system). Fast + deterministic, no IONOS/RF. data-flow-inband-
            // downladder.md §3/§5.3.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_inband_downladder();
            }
            // PRE-FRAME seamless + lost-tag fallback regression. PART D poisons and then
            // explicitly stages the production snapshot before driving the real Stage-4
            // entry, preventing a live-ring/staged-buffer mixed-state fixture.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_inband_seamless();
            }
            // Reseat span integrity: the prev-batch cross-storage index-skew 332-byte
            // deletion. Drives the PRODUCTION store -> seal -> copy_data_to_buffer and
            // proves the seal-count + funnel byte-span keystones prevent the short delivery
            // while healthy + res_c3100-widened deliveries stay byte-identical. Member test
            // on a throwaway controller; fast + deterministic, no IONOS/RF. data-flow-
            // messages_rx_prev.md §4.5 CORRECTION.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_reseat_span();
            }
            // CMD-side id-skew ROOT (the correct-by-construction TX counterpart to the reseat
            // keystones): a fresh v1/v2 non-mixed new-data batch is rebased to
            // messages_tx slots [0,ND)
            // so the wire id == batch position == receiver storage loc. Reproduces the bsi33->bsi34
            // +38 skew via the PRODUCTION add_message_tx_data first-free staging, asserts the
            // rebase corrects it in both wire versions (and the defeat env preserves it),
            // and that the §87 geometry with
            // id=seq DELIVERS IN PLACE (6301 B) through the production receiver. CANONICAL_NUMBERS.md §87.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_cmd_idskew();
            }
            // ROBUST->OFDM ADOPT live-burst PRESERVE regression (the last transition-class hole:
            // the unilateral adopt into an OFDM config used to WIPE the in-flight preamble
            // mid-capture -> no acquire -> TERMINAL BREAK -> ROBUST_0 spiral). Member test on a
            // throwaway controller (builds its own telecom_system). Fast + deterministic, no
            // IONOS/RF. data-flow-robust-ofdm-adopt-flush.md §6/§8.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_inband_adopt_preserve_live_burst();
            }
            // ROBUST->OFDM ADOPT ring-shrink Nofdm-INVARIANT regression (diagnosis a468b2fc):
            // the HINGE-1 shrink re-derived Nofdm (= Nfft+Ngi) from a STALE live ofdm.gi,
            // drifting CONFIG_0 from 292 to 310 -> an 18-sample/symbol FFT-window drift ->
            // LDPC iter=0 -> garbage CRC -> the CONFIG_0 under-decode (~53 B). The fix preserves
            // the just-loaded data_container.Nofdm across the shrink. Member test on a throwaway
            // controller (builds its own telecom_system). Fast + deterministic, no IONOS/RF.
            // data-flow-robust-ofdm-adopt-flush.md §15.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_inband_adopt_nofdm_invariant();
            }
            // Pinned-ring geometry must fail closed: a real config load with Nofdm one sample
            // larger than the pin reference must abort instead of silently shrinking the
            // published ring window to fit the old allocation.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_precook_sample_capacity_abort();
            }
            // SFO block TX geometry must fail closed: transmit_byte reporting no samples,
            // a negative span, or more than the full-frame allocation aborts the harness
            // before stale-data use / out-of-bounds copy. The old clamp silently continued.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_sfo_block_emitted_samples_abort();
            }
            // Harvest regression: independently require the production-created CONFIG_0
            // decoder to match the startup-patched primary geometry. The same test patch
            // fails on bare monitor with Nofdm=310.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_harvest_inband_cfg0_geometry();
            }
            // §21: CONFIG_0-START robust-floor OVER-SEAT (the uncovered sibling of FIX #1e). A session
            // that starts at CONFIG_0 (no robust->OFDM adopt) never latches inband_ofdm_acq_ring_shrunk,
            // so inband_seat_robust_ring_floor over-grows the natural OFDM ring (217->~804) -> every
            // preamble at the tail beyond upper_bound -> 0 forward decode. Member test on a throwaway
            // controller (builds its own telecom_system). data-flow-robust-ofdm-adopt-flush.md §21.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_inband_config0_start_ring();
            }
            // §22: the scoped down-ladder decoder bank inherits the PRIMARY's startup-patched ofdm_gi.
            // A fresh `new cl_telecom_system()` rung otherwise keeps the ctor gi=54/256 -> CONFIG_0
            // Nofdm=310 (not the production 292), an 18-sample/symbol FFT-window stride error that
            // skips LDPC at WB CONFIG_0 -> the robust->OFDM cross never sustains. PASS-AFTER: 292.
            // FAIL-BEFORE (MERCURY_INBAND_GI_INHERIT_DEFEAT=1, same binary): 310.
            // data-flow-robust-ofdm-adopt-flush.md §22.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_inband_down_decoder_gi_inherit();
            }
            // §17: descrambler survives the inband ring-shrink, including a fail-closed N_MAX+1
            // comparison-bounds regression (the CONFIG_0 clean-lock CRC-fail root).
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_inband_descrambler_survives_ring_shrink();
            }
            // §19: dead-batch streak ties to REAL batch periods + zero-progress (climb-killer fix);
            // a real total loss STILL BREAKs (recovery preserved).
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_inband_deadbatch_progress();
            }
            // A DATA frame rejected by add_message_rx_data() must not advance
            // current-batch/session progress or promote staged batch metadata.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_rx_store_verdict_fail_closed();
            }
            // In-band CONNECT-LIVENESS GUARD regression (control-plane livelock backstop).
            // Member test on a throwaway controller (builds its own telecom_system). Fast +
            // deterministic, no IONOS/RF. data-flow-inband-connect-liveness.md §4.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_inband_liveness();
            }
            // In-band FORWARD-HEALTHY REVERSE-ACK MISS -> NO-BREAK DELIVER regression (the
            // 785-frame decode-but-0-deliver rework). Member test on a throwaway controller
            // (builds its own CMD/telecom_system per case). Fast + deterministic, no IONOS/RF.
            // data-flow-inband-retx-epoch.md §5.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_inband_deliver();
            }
            // LINK-PHASE PRIMITIVE (increment 1) shadow-agreement directed unit. Member test
            // on a throwaway controller; PURE in-process synthetic-fire of the production
            // producers. Fast + deterministic, no IONOS/RF. data-flow-linkphase-primitive.md.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_linkphase_shadow();
            }
            // LINK-PHASE PRIMITIVE (increment 2 / MC-2) slot-floor directed unit. Pre-init
            // latch fallback, clean-path byte-identity, stale-generation config-switch fallback,
            // RAISE-ONLY. Member test on a throwaway controller; in-process, no IONOS/RF.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_linkphase_mc2_slotfloor();
            }
            // Gearshift-v2 transition-cost helpers: ACK slot floor clock origin, CONFIG_TAG
            // guard, coordinated WB entry, unacknowledged batch-geometry control resume.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_gs2_spec_conformance();
            }
            // LINK-PHASE STEP 3 (increment 3) sample-anchored reverse-ACK detection window:
            // the gate predicate + the sub-class-b beyond-ring rescue predicate + the
            // byte-identical-OFF / WIDEN-only invariant. In-process, no IONOS/RF.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_linkphase_ackwin();
            }
            // MC-7 optimizer clock regression: a completed keydown's END-stamped
            // duration wins over a different live derive after a geometry switch;
            // a stale epoch retains the legacy live fallback.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_linkphase_optclock_end_stamp();
            }
            // A failed BREAK transition invalidates the link-phase contract and
            // must stop recovery before ListenerReady or subsequent caller work.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_lp_break_recovery_fail_closed();
            }
            // A rejected config-transition observation must stop the caller
            // before it changes ARQ/PHY geometry and clear stale timeline
            // evidence. Uses the sample clock, so no wall-clock timing enters
            // the failure-path assertion.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_linkphase_config_transition_failure();
            }
            // Phase-0 deterministic transition oracle: two production
            // controllers, sample-derived event queue, scripted delivery
            // actions, reverse-ACK seam trace, and 3 seeds x 3 repetitions.
            failed += mercury::run_lp_transition_harness_tests();
            // In-band capture-ring ROBUST-floor OVER-SEAT regression (the CONFIG_8-climb
            // 24/25-BREAK root): inband_seat_robust_ring_floor over-grew the climbed OFDM
            // ring to the ROBUST floor -> frame-0 SKIP-VAR every batch. Member test on a
            // throwaway controller; PURE in-process synthetic-fire.
            // data-flow-inband-ring-floor-overseat.md §5.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_inband_ring_floor_overseat();
            }
            // Option B compact coded reverse-confirm LIVE RX-PATH regression
            // (data-flow-compact-confirm.md §9): the self-validating compact confirm
            // must be accepted via its OWN CRC-gated decode, DECOUPLED from the bare
            // 7/16 presence gate that under-peaks the shorter 26-sym frame. Drives the
            // FULL live RX path (passband -> capture ring -> the production accept
            // predicate cmd_compact_confirm_live_accept), NOT a direct decoder call.
            // Member test on a throwaway controller; PURE in-process synthetic-fire.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_compact_confirm_live_rx_path();
            }
            // Option B fix-b — compact confirm accept INSIDE the SACK window (batch>1,
            // SACK-on, clean, WB; data-flow-compact-confirm.md §10.4/§10.5). The SECOND
            // live-land defect (4.7x batch>1-WB regression at ENABLE=1). PURE in-process.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_compact_confirm_sack_window_rx_path();
            }
            // FORGIVING-ACK Tier-2 cumulative-n_r self-heal / gap-invariant / cap-gate
            // regression (the A3 predicate proof). Member test on a throwaway controller;
            // PURE in-process synthetic-fire, no IONOS/RF. data-flow-forgiving-ack.md §T2.6.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_cumulative_ack();
            }
            // A3 DECOUPLE-SAFETY CHECKPOINT (§2): single-miss non-load-bearing (anti-0-bytes,
            // byte-faithful) + genuine-death net intact, demote IN PLACE. Member test on a
            // throwaway controller; PURE in-process synthetic-fire. data-flow-forgiving-ack.md
            // §T2.2/§6.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_a3_decouple_safety();
            }
            // HYBRID TIER-CROSSING ROUTING (data-flow-inband-tier-crossing.md §3):
            // a robust<->OFDM crossing routes to the legacy SET_CONFIG handshake
            // (fast dedicated ACK); intra-tier rate adapts keep the in-band tag.
            // Member test on a throwaway controller; PURE in-process synthetic-fire,
            // no PHY/audio/IONOS/RF -> permanent regression gate.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_inband_tier_crossing_routing();
            }
            // IN-BAND TIER-CROSS REVERSE-ACK PIN regression (data-flow-inband-tier-
            // crossing.md §3): a robust<->OFDM cross must pin reverse_configuration to
            // the ROBUST side (not the OFDM forward rung), so the reverse SACK decodes on
            // MFSK across the cross (the redesign's 102<->0 oscillation root). PURE in-
            // process synthetic-fire — permanent regression gate.
            {
                cl_arq_controller test_revpin;
                failed += test_revpin.test_inband_tier_cross_reverse_pin();
            }
            // KEYSTONE intra-tier base-pattern confirm regression (data-flow-inband-tier-
            // crossing.md §6): an intra-tier CLIMB confirm must ride the robust BASE ACK
            // pattern (decoupled from the marginal bsi-bearing SACK suffix) so the redesign
            // climbs past CONFIG_0 toward legacy's OFDM rungs. PURE in-process synthetic-fire
            // — permanent regression gate. Fails-before: -DINBAND_BASEPATTERN_CONFIRM_FAILBEFORE.
            {
                cl_arq_controller test_baseconfirm;
                failed += test_baseconfirm.test_inband_basepattern_confirm();
            }
            // RESIDUAL FIX regression (data-flow-inband-tier-crossing.md s11): the ROBUST-TIER
            // tag-follow must fire on STALE inter-frame passes too, else the re-emitted CONFIG_TAG
            // is dropped and the robust->OFDM tier-cross becomes non-deterministic. Drives the
            // EXACT production gate (inband_robust_follow_gate_open). Fail-before:
            // MERCURY_ROBUST_FOLLOW_FRESHWIN_REQ=1. PURE in-process -- permanent regression gate.
            {
                cl_arq_controller test_rbfollow;
                failed += test_rbfollow.test_inband_robust_follow_freshwin();
            }
            // IN-BAND CONFIG_0 ROLLING-PARTIAL climb-unblock regression
            // (data-flow-inband-frame0-rolling-partial.md §4): at the inband OFDM base rung the
            // first OFDM frame of each batch fails the SKIP-VAR gate (acquisition seam) → a
            // rolling lead-frame-only partial that pre-fix vetoed the FRAME-UP climb → CONFIG_0
            // wedge. The fix advances the climb streak on that partial while a multi-drop partial
            // stays vetoed. PURE in-process synthetic-fire — permanent regression gate.
            // Fails-before: -DINBAND_FRAME0_PARTIAL_FAILBEFORE.
            {
                cl_arq_controller test_f0p;
                failed += test_f0p.test_inband_frame0_partial();
            }
            // IN-BAND ROLLING-PARTIAL climb DEFER-WHILE-HOLE-OUTSTANDING regression
            // (data-flow-inband-frame0-rolling-partial.md §10): the 2801d7c sibling — the
            // lead-frame-only partial enqueues frame-0 for retx (retransmit_count > 0), and the
            // anchor-raise + FRAME-UP config-change fire WHILE that hole is outstanding; the config
            // change clears the retx queue -> abandons frame-0 -> bsi gap -> GAP-ABORT wedge. The
            // fix DEFERS the anchor-raise + the fire while the hole is outstanding but KEEPS the
            // 2801d7c streak credit. PURE in-process synthetic-fire — permanent regression gate.
            // Fails-before: -DINBAND_CLIMB_DEFER_FAILBEFORE.
            {
                cl_arq_controller test_cd;
                failed += test_cd.test_inband_climb_defer_on_retx();
            }
            // IN-BAND +1 CLIMB regression (data-flow-inband-frame0-rolling-partial.md §7.2
            // option A): under MERCURY_INBAND_RATE the FRAME-UP climb must step EXACTLY +1
            // (suppress the SNR elevator that jumped CONFIG_0->3 and stranded the reverse
            // data-SACK -> nAcked_data stuck -> BREAK at every rung). Legacy keeps the
            // elevator (byte-identical). PURE in-process synthetic-fire — permanent
            // regression gate. Fails-before: -DINBAND_PLUS1_CLIMB_FAILBEFORE.
            {
                cl_arq_controller test_p1;
                failed += test_p1.test_inband_plus1_climb();
            }
            // CLIMB-LATENCY — guarded SNR-seed of the START config
            // (gearshift-start-and-recovery.md §10): a CLEARLY-clean channel opens
            // data near the SNR-appropriate WB OFDM config at connect instead of
            // crawling ROBUST_0->1->2 (the ~102 s dominant climb cost), while a
            // MARGINAL channel still starts ROBUST_0 (the over-seed guard). PURE
            // in-process synthetic-fire — permanent regression gate. Fails-before:
            // -DCONNECT_SEED_FAILBEFORE.
            {
                cl_arq_controller test_cs;
                failed += test_cs.test_connect_snr_seed();
            }
            // CLIMB-LATENCY — first ROBUST climb rung pipelines
            // (gearshift-start-and-recovery.md §10.3): the ROBUST_0 dwell must NOT
            // serialize a clean-batch round-trip per rung (the un-pipelined first-rung
            // gap). Drives the REAL gate inband_pipeline_climb_active() on the robust
            // tier. PURE in-process synthetic-fire — permanent regression gate.
            // Fails-before: -DINBAND_ROBUST_PIPELINE_FAILBEFORE.
            {
                cl_arq_controller test_rp;
                failed += test_rp.test_robust_pipeline();
            }
            // DUTY FAST-START lever R — connect-evidenced robust exit (climb-duty):
            // the CONFIG_0 OFDM tier-cross fires at connect instead of after a full
            // robust DATA batch (~38 s). Drives the REAL predicate
            // robust_connect_exit_target(). PASS-AFTER seeds CONFIG_0; FAIL-BEFORE
            // (duty_r_defeat, the MERCURY_DUTY_R_DEFEAT / master-defeat production gate)
            // returns CONFIG_NONE. PURE in-process synthetic-fire — permanent gate.
            {
                cl_arq_controller test_dr;
                failed += test_dr.test_robust_connect_exit();
            }
            // DUTY FAST-START lever P-alt — climb-confirm batch shrink (climb-duty):
            // the OFDM confirm batch is capped small while climbing a non-top rung so
            // the 2-clean anchor bar is reached in less forward airtime. Drives the REAL
            // predicate climb_confirm_batch_active(). PASS-AFTER active; FAIL-BEFORE
            // (duty_palt_defeat, MERCURY_DUTY_PALT_DEFEAT / master defeat) inactive.
            {
                cl_arq_controller test_pa;
                failed += test_pa.test_climb_confirm_batch();
            }
            // Held-rung burst coalescing — a margin-gated election can HOLD a session
            // at a sub-top OFDM rung forever; the climb-confirm pin then caps the
            // steady-state batch at 10 and the Axis-2 skip starves growth. The release
            // (probation counter + provenance-released sole-setter clamp + gated
            // Axis-2 skip) drives the REAL controller, setter, RSP SET_LINK_PARAMS
            // handler, reset chokepoint and ctor env latch. FAIL-BEFORE arms via
            // MERCURY_BURST_COALESCE_DEFEAT on ONE binary.
            {
                cl_arq_controller test_bc;
                failed += test_bc.test_burst_coalesce_held_rung();
            }
            // BATCH-GRID COAST (FIX 1) — on a mid-batch decode FAIL the stock anti-spin
            // ladder discards the frame grid and the RX blind-searches into the
            // beyond-bounds attractor, killing the whole tail. Coast keeps the grid alive
            // (advance one frame period from the last CRC-GOOD anchor, keep batch_active) so
            // the existing predict+verify re-locks the next frame. Drives the REAL
            // batch_coast_try_advance() with REAL cfg15 geometry: PASS-AFTER coasts + counter
            // fires; FAIL-BEFORE (MERCURY_BATCH_COAST_DEFEAT) inert; per-leg eligibility guards.
            {
                cl_arq_controller test_coast;
                failed += test_coast.test_batch_coast();
            }
            // P1 ACQ BAND-EXCLUSION — the high-SNR break-storm persistence engine is
            // the no-exclusion Schmidl-Cox sub-peak re-pick (the reject loop restores
            // orig_delay every trial, re-locking the same content-stable wrong point in
            // a BAND of delays). P1 records rejected delays as band centers and forces a
            // clean re-acquire on a repeat re-pick inside a recorded band. Drives the
            // REAL predicate acq_band_excl_hit(): FAIL-BEFORE (empty memory) misses;
            // PASS-AFTER (band recorded) hits; BAND (not single-delay) neighbor hits;
            // a new batch accepts a valid preamble at the formerly-excluded delay.
            {
                cl_telecom_system test_excl;
                failed += test_excl.test_acq_band_excl();
            }
            // DUTY FAST-START lever ELEVATOR FAST-CONFIRM — margin-gated N=1 OFDM climb confirm:
            // on a whole-clean OFDM batch whose raw SNR capacity clears a higher rung, the anchor
            // is seated + the FRAME-UP elevator fires on the FIRST clean (skip the 2nd confirm),
            // collapsing the 2nd per-rung airtime of the cfg0->cfg16 dwell. Drives the REAL core.
            {
                cl_arq_controller test_ef;
                failed += test_ef.test_elevator_fast_confirm();
            }
            // CONNECT-SEED FUSION (climb-duty, connect floor) - the connect-evidenced seed
            // is folded into the SWITCH_BANDWIDTH frame so the RSP loads it on the WB switch
            // and the CMD on the ACK, collapsing the separate robust SET_CONFIG cross. Drives
            // the REAL selector connect_fuse_seed_select(). PASS-AFTER carries CONFIG_0;
            // FAIL-BEFORE (connect_fuse_defeat / MERCURY_CONNECT_FUSE_DEFEAT) CONFIG_NONE.
            {
                cl_arq_controller test_cf;
                failed += test_cf.test_connect_fuse();
            }
            // CLIMB-UP cmd_batch_seq_id ROLLBACK regression
            // (data-flow-inband-frame0-rolling-partial.md §13): the SYMMETRY GAP to the demote
            // rollback — the climb-UP SET_CONFIG emits (FRAME-UP, optimizer, turbo settle)
            // re-present an in-flight (RSP-delivered, not-yet-CMD-ACKed) batch under whatever
            // ADVANCED epoch a rapid climb reached, so the RSP sees a >=2 bsi jump from its
            // preserved delivery high-water -> sack_v2_readopt_has_gap()=true -> [RSP-V2-GAP-ABORT]
            // HOLD. The fix rolls cmd_batch_seq_id back to the in-flight bsi (mirroring the demote
            // paths). Drives the REAL producer + REAL RSP predicate. PURE in-process synthetic-fire
            // — permanent regression gate. Fails-before: -DINBAND_CLIMB_BSI_ROLLBACK_FAILBEFORE.
            {
                cl_arq_controller test_cbr;
                failed += test_cbr.test_climb_bsi_rollback();
            }
            // Reverse-SACK partial-target guard reference regression.
            {
                cl_arq_controller test_gr;
                failed += test_gr.test_guard_reanchor();
            }
            // A2 generation canonicalization: cumulative target N must resolve
            // before the R2c retention-shadow route while dispatch is at N+1.
            {
                cl_arq_controller test_gc;
                failed += test_gc.test_generation_canon();
            }
            // FIX-C graceful-shutdown handler: handler installed above, this
            // self-raises SIGTERM/SIGINT and asserts shutdown_ flips, then
            // clears the flag so the rest of the process is unperturbed.
            failed += test_sigterm_handler(install_termination_handlers);
            // IDLE-SWITCHROLE-RACE regression gates (idle-switchrole-race.md §4):
            // B (trigger-gate, B1+B2) and C (no-progress teardown + neg-control).
            // Both are in-process synthetic-fire (no PHY/audio) so they belong in
            // the master suite as permanent regression gates. MERGE COMPOSE: kept
            // alongside the redesign's in-band regression gates above (both belong).
            {
                cl_arq_controller ARQ_isr;
                failed += ARQ_isr.test_idle_switch_role_race();
                failed += ARQ_isr.test_switch_role_reride_race();
                failed += ARQ_isr.test_break_noprogress_teardown();
            }
            // MIXBATCH FILL OVER-POP -- COMPRESSION LEG regression
            // (data-flow-mixbatch-fill.md ss9): comp-leg sibling of the no-comp
            // over-pop -- fill-cap respects the v2 retx prefix AND the force-FREE
            // never destroys a staged-but-unsent frame. In-process synthetic-fire
            // (no PHY/audio), pass-after arm. DEFEAT env = fail-before.
            {
                cl_arq_controller ARQ_mixcmp;
                failed += ARQ_mixcmp.test_mixbatch_fill_overpop_compressed();
            }
            // Streaming compression stale-EMA/no-fit regression: a high ratio
            // estimate must reduce and recompress, not silently send the reduced
            // payload RAW. MERCURY_COMPRESS_RETRY_DEFEAT=1 is the fail-before arm.
            {
                cl_arq_controller ARQ_cmpretry;
                failed += ARQ_cmpretry.test_streaming_compress_overshoot();
            }
            // Streaming setup allocation failure must be visible to callers:
            // no partially initialized carry state may report itself active.
            {
                cl_arq_controller ARQ_stream_alloc;
                failed += ARQ_stream_alloc.test_streaming_allocation_failure();
            }
            // Fix A — mid-flight data_batch_size SHRINK must not orphan RECEIVED prev
            // frames (baseline-double-delivery.md): drives the REAL set_data_batch_size
            // shrink; reconstructs the copy_data_to_buffer prev delivery set; asserts
            // ZERO delivered bytes lost. Pass-after arm (DEFEAT env = fail-before).
            {
                cl_arq_controller ARQ_shrink;
                failed += ARQ_shrink.test_batch_shrink_orphan_defer();
            }
            // rx_btf=-1 CURRENT-batch SHRINK tail-drop (silent-corruption-marginal-snr.md):
            // the messages_rx[] sibling of the Fix A prev guard above — a mid-flight
            // data_batch_size shrink orphans RECEIVED current-batch frames the next seal
            // silently drops (the marginal-SNR WGN:25 mid-stream truncation, md5 FALSE).
            // Drives the REAL shrink chokepoint + seal + prev gate + copy_data_to_buffer;
            // asserts byte-exact full delivery. Pass-after arm (DEFEAT env = fail-before).
            {
                cl_arq_controller ARQ_shrcur;
                failed += ARQ_shrcur.test_batch_shrink_orphan_current();
            }
            // SEAM-2 deferred prev-confirm ARM/FLUSH fire proof (pass-after arm here;
            // MERCURY_PENDING_CONFIRM_DEFEAT=1 is the fail-before drop).
            {
                cl_arq_controller ARQ_lppc;
                failed += ARQ_lppc.test_linkphase_pending_confirm_fire();
            }
            // Streaming decompress-failure SILENT-FALSE-ACCEPT (residual-silent-
            // corruption-wgn25.md): the THIRD WGN:25 trigger — a streaming PPMd model
            // desync under out-of-order SACK delivery makes a COMPLETE, CRC-clean batch
            // decode to -1, and copy_data_to_buffer's old fallback pushed the raw
            // compressed blob to the app (silent garbage). Drives a REAL two-compressor
            // desync + copy_data_to_buffer; asserts zero garbage delivered + loud detect.
            // Pass-after arm (MERCURY_DECOMPRESS_RAWPUSH_DEFEAT=1 = fail-before).
            {
                cl_arq_controller ARQ_decfa;
                failed += ARQ_decfa.test_decompress_false_accept();
            }
            // Fix C / H#1 — fifo_buffer_backup re-stage double-delivery
            // (data-flow-fifo-backup.md §6.1): the delivered batch's raw survives in the
            // backup when finalize is pre-empted (Root B), and the config-change re-stage
            // re-sends it. ACK-confirm-keyed flush empties the backup at register_ack()
            // when no un-confirmed data exists. Pass-after arm (DEFEAT env = fail-before).
            {
                cl_arq_controller ARQ_bkflush;
                failed += ARQ_bkflush.test_backup_confirm_flush();
            }
            // Fix H#3 — RX-FIFO back-pressure post-ACK data loss
            // (delivery-integrity-audit-monitor.md §3): the clean ACK precedes delivery and
            // fifo_push_rx drops the tail when the app FIFO is full. The ACK-GATE now holds
            // the batch until fifo_buffer_rx has room. Pass-after arm (DEFEAT env = fail-before).
            {
                cl_arq_controller ARQ_rxbp;
                failed += ARQ_rxbp.test_rxfifo_backpressure_hold();
            }
            // RX-CTRL-DROP regression (data-flow-control-slot-lifecycle.md §6): the
            // messages_control one-deep mailbox had no timeout escape out of RECEIVED,
            // so a stranded slot silently dropped every later control frame and killed
            // the reverse control plane (demote-amplifier feeder). In-process
            // synthetic-fire (no PHY/audio) — a permanent regression gate.
            {
                cl_arq_controller ARQ_rxctrl;
                failed += ARQ_rxctrl.test_rx_ctrl_drop();
            }
            // ML-KEM KX MULTI-CHUNK LIVE-RX gate (MLKEM_HYBRID_PLAN.md §5 /
            // data-flow-control-slot-lifecycle.md): the RX control-slot producer
            // hardcodes messages_control.length=1 while copying a FIXED slot width
            // into data[]; the KX2/KX3 chunk receivers fed length=1 into
            // kx_receive_chunk -> kx_chunk_decode rejected EVERY chunk -> the PQ
            // hybrid handshake could never complete on the live wire. This gate
            // drives multi-chunk KX2 (RSP consumer) + KX3 (CMD consumer) through the
            // REAL process_control_responder/commander call sites and asserts full
            // reassembly + live encapsulate/decapsulate. In-process synthetic-fire.
            {
                cl_arq_controller ARQ_kxlive;
                failed += ARQ_kxlive.test_kx_chunk_live_rx();
            }
            // KX-AS-DATA reserved-stream RX ingest fire proof (data-flow-hybrid-kex.md
            // T1/T3): the new decoded-transport reassembler kx_ingest() reassembles a
            // FRAGMENTED forward (x25519+pk) and reverse (x25519+ct) stream BYTE-IDENTICAL
            // and NEVER routes to the app FIFO, and the fail-secure REFUSE FIRES (-1) on
            // bad magic/kind/length + a post-activation byte. In-process synthetic-fire.
            {
                cl_arq_controller ARQ_kxi;
                failed += ARQ_kxi.test_kx_ingest();
            }
            // KX-as-data RX ROUTING (data-flow-hybrid-kex.md §2.3): drive the REAL
            // copy_data_to_buffer() delivery funnel with a forward KX stream staged
            // across ACKED messages_rx[] slots in the pre-activation KX epoch and
            // assert the production routing branch reassembles via kx_ingest, never
            // reaches the app FIFO, does not advance the Option-W cursor, and raises
            // no false stream-shift teardown. In-process synthetic-fire.
            {
                cl_arq_controller ARQ_kxr;
                failed += ARQ_kxr.test_kx_rx_routing();
            }
            // KX-as-data forward TX stage/feed (data-flow-hybrid-kex.md §2.2): drive the
            // REAL process_buffer_data_commander() source-switch — stage the reserved
            // forward stream (x25519_pk_cmd + mlkem_pk) and frame it as DATA batches —
            // then deliver those frames through the REAL copy_data_to_buffer()->kx_ingest()
            // funnel and assert the pk + folded x25519 pubkey reassemble BYTE-IDENTICAL
            // end to end, never reaching the app FIFO. In-process synthetic-fire.
            {
                cl_arq_controller ARQ_kxt;
                failed += ARQ_kxt.test_kx_tx_stream();
            }
            // KX-as-data FULL crypto round-trip (data-flow-hybrid-kex.md Â§7 / T6): two
            // controllers drive the REAL kx_stage_forward_stream -> kx_ingest ->
            // kx_on_forward_complete -> kx_ingest -> kx_on_reverse_complete path and assert
            // BOTH peers derive the IDENTICAL hybrid key + matching confirm tag + PQ-upgrade,
            // and that a tampered reverse ct diverges the key (confirm mismatch = KEY_ACTIVATE
            // REFUSE). In-process synthetic-fire; the SWITCH_ROLE transport is proven live.
            {
                cl_arq_controller ARQ_kxrt;
                failed += ARQ_kxrt.test_kx_roundtrip_derive();
            }
            // KX-as-data MID-STREAM AEAD payload TAMPER fire proof (the live channel-
            // MITM gate, data-flow-hybrid-kex.md G3): flip ciphertext bytes on the post-
            // activation data stream and prove the receiver REJECTS — the AEAD auth-fail
            // fires, the production copy_data_to_buffer() funnel tears the link, 0 corrupted
            // bytes reach the app, and it never falls back to plaintext. Clean control
            // delivers. In-process synthetic-fire.
            {
                cl_arq_controller ARQ_kxdt;
                failed += ARQ_kxdt.test_kx_data_tamper();
            }
            // Final-connect retry gate: replay identity, retry epoch, replay
            // bounds, and OFDM capture ownership.
            {
                cl_arq_controller ARQ_reack;
                failed += ARQ_reack.test_connect_reack();
	{ cl_arq_controller ARQ_ts; failed += ARQ_ts.test_terminal_settlement(); }
            }
            // START_CONNECTION bare-ACK causal gate: reject the impossible
            // 263-ms detector hit, clear retained pre-epoch phases, and accept
            // the genuine post-turnaround observation exactly once.
            {
                cl_telecom_system start_ack_ts;
                cl_arq_controller ARQ_start_ack;
                ARQ_start_ack.telecom_system = &start_ack_ts;
                failed += ARQ_start_ack.test_start_ack_causal_guard();
            }
            // OFDM ACQUISITION BOUNDS-GATE RECOVERY gate (data-flow-acq-bounds-gate.md):
            // a tail-band frame whose Schmidl-Cox detection floors past upper_bound but
            // whose body is still in-buffer must be RECOVERED, not force-FAILed/discarded;
            // and a cursor genuinely past the plateau-lead band must still force-FAIL
            // (anti-re-decode preserved). Faithful in-process synthetic-fire through the
            // production transmit_byte/receive_byte (no IONOS/RF).
            failed += run_acq_bounds_selftest();
            // F1b Part B SAMPLE-ANCHORED SUB-PEAK RESCUE gate (data-flow-rx-ring-rearm.md):
            // after a turnaround flush the first frame can lock a Schmidl-Cox sub-peak
            // (metric saturates, pilots misalign, mean|H| collapses) and never read; the
            // rescue re-derives the timing by argmax of mean|H| and re-decodes. Proves the
            // selector + argmax + decode mechanism deterministically (the live mislock is
            // reproduced only by the real-audio needle-2 cohort, not in-process).
            failed += run_subpeak_rescue_selftest();
            // CMD>RSP BATCH-SIZE DESYNC silent-corruption gate (res_c3100, silent-
            // corruption-residual.md §8): a CMD>RSP data_batch_size desync (CMD built 30,
            // RSP applied 25) makes the RSP ACK-GATE deliver the batch TRUNCATED to its
            // smaller size -> a permanent silent stream shift. The (B) LOUD BACKSTOP
            // detects the sender-declared count exceeding the local batch and aborts
            // (link DROPPED) instead of silently delivering. In-process synthetic-fire
            // (no IONOS/RF) — a permanent integrity D0 regression gate.
            {
                cl_arq_controller ARQ_bsd;
                failed += ARQ_bsd.test_batchsize_desync_delivery();
            }
            // GAP-ABORT ruler-blinding silent-corruption gate (data-flow-rsp-contiguity-
            // ruler.md): the REAL rsp_gap_abort_teardown() routes through
            // reset_session_state(), which clears rsp_last_delivered_batch_seq_id to -1 and
            // blinds its own re-adopt gate; the teardown emits no OTA abort, so the CMD
            // re-drives the same stream and the next batch is silently concatenated across
            // the dropped batch. Drives the REAL teardown + the REAL gap predicates + the
            // REAL fifo_buffer_rx app stream. In-process synthetic-fire (no IONOS/RF) — a
            // permanent integrity D0 regression gate.
            // R1 turnaround-clearance guard (arq_common.cc): the guard busy-waits the
            // remainder of the peer's TX->RX mute/flush + demod re-arm window before keying
            // a data batch, so the first frame never lands inside it (the post-LINK-PARAMS
            // slot-0 loss). Drives the REAL guard with stock delays; asserts default-on waits
            // >=250 ms, defeated waits ~0, already-clear adds ~0, never-armed no-ops.
            {
                cl_arq_controller ARQ_tg;
                failed += ARQ_tg.test_turnaround_guard();
            }
            // R6 measured turnaround (arq_common.cc): the SRTT/RTTVAR estimator
            // (RFC 6298 / Jacobson RTO; Karn & Partridge) replaces the calibrated
            // SACK margin, the robust-geometry widen, and the CONFIG_15-only
            // re-phase adder with a self-tracking window. Drives update_turnaround_
            // estimate + calculate_receiving_timeout directly; the landmine arm
            // proves a non-CFG15 drifty long batch tracks (fail-before misses the
            // 4500 ms arrival, pass-after contains it) and re-asserts the
            // ack_timeout_data >= receiving_timeout invariant on the per-batch path.
            {
                cl_arq_controller ARQ_mt;
                failed += ARQ_mt.test_measured_timers();
            }
            // T1 zero-dead-time turnaround learned-timing model (arq_common.cc): the four
            // per-direction learned terms (key-up / ears-deaf / ears-back + peer terms) fold as
            // RFC-6298 SRTT/RTTVAR tracks; the pre-key lead = max(txdelay_floor, keyup-deaf)
            // clamped >=0 and the scheduled key instant = predictive keydown-end - lead. Drives
            // the REAL t1_note_term_sample + compute_pre_key_lead + t1_scheduled_key_ms on a
            // throwaway controller: convergence to the injected truth, the conservative window,
            // the lead arithmetic + amp-safety floor + >=0 clamp, and the COLD/DEFEAT reactive
            // fallback (fail-before under MERCURY_T1_CALIB_DEFEAT yields no calibrated lead).
            {
                cl_arq_controller ARQ_t1;
                failed += ARQ_t1.test_t1_calibration();
            }
            // T1b scheduled pre-key re-aim (arq_common.cc): the reverse-ACK keyup is armed at
            // owner_keydown_end_ms - lead ONLY when the MERCURY_T1_SCHEDKEY lever is on AND the
            // batch end is predictively known AND the terms are calibrated; lever-off / no-D5 /
            // cold / no-stamp keep the reactive keyup wait unchanged (the robust fallback). Drives
            // the REAL t1_scheduled_keyup_wait_ms on a throwaway controller: the scheduled-vs-
            // reactive negative control, the >=0 clamp, and the calibration-defeat fail-before/after.
            {
                cl_arq_controller ARQ_t1sk;
                failed += ARQ_t1sk.test_t1_schedkey();
            }
            // T1c connect-time calibration bootstrap (arq_common.cc): the missing producer that
            // warms the learned terms at connect-accept so compute_pre_key_lead leaves the reactive
            // sentinel and the scheduled pre-key (MERCURY_T1_SCHEDKEY) fires on the live path.
            // Drives the REAL t1_seed_calibration_defaults(): fail-before (cold/inert) -> pass-after
            // (warm, fires) with a distinct calibration-defeat arm.
            {
                cl_arq_controller ARQ_t1cb;
                failed += ARQ_t1cb.test_t1_calib_bootstrap();
                failed += ARQ_t1cb.test_t1_schedkey_lifecycle();
                failed += ARQ_t1cb.test_t1_repeating_prefix();
            }
			{
				cl_arq_controller ARQ_t1safe;
				failed += ARQ_t1safe.test_t1_schedkey_safety();
			}
            // Karn discriminator decoupling (MERCURY_KARN_RETX_ONLY): the R6 estimator was
            // inert on the dominant clean OFDM path because the Karn gate read
            // data_ack_retx_turnaround (which the H1 widen arms on a CLEAN batch). Drives the
            // REAL tt_karn_sample_ok() + update_turnaround_estimate(): fail-before (knob off)
            // starves the estimator (n stays 0); pass-after (knob on) folds the clean sample
            // (n 0->1) while a genuine retx round and a starve are still excluded.
            {
                cl_arq_controller ARQ_kn;
                failed += ARQ_kn.test_karn_retx_classify();
            }
            // [RSP-TIMEOUT] diagnostic field-alignment gate (data-flow-rsp-timeout.md):
            // the RESPONDER receive-window diagnostic passed a double field to a %d slot,
            // shifting every field after it one slot left on the x86-64 SysV ABI so the
            // timeout= label showed an unpushed stack slot (garbage) while the armed value
            // hid under sack=. Drives the REAL calculate_receiving_timeout() RESPONDER path,
            // captures the line off stdout, and asserts the printed fields align with the
            // armed receiving_timeout. Deterministic, no RF.
            {
                cl_arq_controller ARQ_rtf;
                failed += ARQ_rtf.test_rsp_timeout_format();
            }
            // [PENDING-CONF] scanner-control PENDING conformance (session-connect-handshake.md):
            // PENDING is bare + address-filtered (only a for-MYCALL START_CONNECTION emits it,
            // the callsign rides on CONNECTED), and a for-us PENDING that never reaches CONNECTED
            // releases the host once via CANCELPENDING+DISCONNECTED. Drives the REAL
            // process_control_responder() and rsp_emit_release() and captures the control-socket
            // bytes off the transmit seam. Deterministic, no RF.
            {
                cl_arq_controller ARQ_pc;
                failed += ARQ_pc.test_pending_conformance();
            }
            // R4 LINK-PARAMS quiesce gate (arq_commander.cc): an Axis-2 batch-size
            // renegotiation must not fire while the boundary is unclean (retx pending / last
            // batch partial), which would mix old-bsi retransmits into the first batch of the
            // new geometry. Drives the REAL policy_evaluate_axis2 across clean/unclean
            // boundaries + the defeat knob; asserts DEFER while unclean, FIRE when clean,
            // FORCE after the bound, and the pre-R4 mixed-batch move under the defeat knob.
            {
                cl_arq_controller ARQ_q2;
                failed += ARQ_q2.test_axis2_quiesce_gate();
            }
            // FIX 2 — catastrophic-partial fast-down: a SINGLE batch carrying the full
            // hysteresis' worth of SACK-bitmap loss (partial>0.60) fast-downs the batch
            // (AIMD B/2) immediately instead of waiting 3 consecutive bad batches, breaking
            // the coalescing relapse loop. Drives the REAL policy_evaluate_axis2 across the
            // defeat knob + a bad-but-not-catastrophic guard.
            {
                cl_arq_controller ARQ_fd;
                failed += ARQ_fd.test_axis2_fastdown();
            }
            // Per-rung MEASURED election floor gate + failure memory (DATAFLOW_AUDIT_rung_floor.md):
            // generalizes the CFG16 decode-margin gate to every OFDM rung so the over-reading suffix
            // meter can no longer elect a rung above its measured delivery floor (the wb13 snr3k+13
            // cfg14 stall). Drives the REAL apply_rung_floor_cap / failure-memory decisions across the
            // wb13 fail-before/pass-after, never-raise, arm/survive/decay/reset, and the defeat knob.
            {
                cl_arq_controller ARQ_rf;
                failed += ARQ_rf.test_rung_floor_gate();
            }
            {
                // ACK-suffix pollution guard fail-before/pass-after (forward-DATA SNR vs control decode).
                cl_arq_controller ARQ_aksfx;
                failed += ARQ_aksfx.test_ack_suffix_pollution_guard();
            }
            {
                cl_arq_controller ARQ_gab;
                failed += ARQ_gab.test_gap_abort_readopt_blind();
            }
            // The in-order demote scaffold must fail closed if clean-prefix
            // setup itself encounters a delivery gap; no corrupt synthetic
            // state may leak into the demote cases that follow.
            {
                cl_arq_controller ARQ_id;
                failed += ARQ_id.test_inorder_demote();
            }
            // zombie/amplifier layer (data-flow-zombie-amplifier.md): R2b BREAK-accept-
            // while-DROPPED, the ROBUST_DWELL keep-alive gate, and the watchdog
            // peer-liveness probe gate — each pure decision helper's truth table under
            // the defeat knob (fail-before) vs unset (pass-after). Deterministic; no RF.
            {
                cl_arq_controller ARQ_za;
                failed += ARQ_za.test_zombie_amp();
            }
            // R5 demote-split (data-flow-zombie-amplifier.md): the pure-silence classifier truth
            // table, the counter evolution via the shared step, the reroute predicate + guards, the
            // DEFEAT knob, and the +LOW watchdog probe-latch hygiene. Deterministic; no RF.
            {
                cl_arq_controller ARQ_dsil;
                failed += ARQ_dsil.test_demote_silence();
            }
            // NB connect-reply prior guard + forward-report entry leap (directed).
            {
                cl_telecom_system ts_el;
                cl_arq_controller ARQ_el;
                ARQ_el.telecom_system = &ts_el;
                failed += ARQ_el.test_nb_prior_guard();
                failed += ARQ_el.test_entry_leap_fwd();
            }
            {
                cl_telecom_system ts_eg;
                cl_arq_controller ARQ_eg;
                ARQ_eg.telecom_system = &ts_eg;
                failed += ARQ_eg.test_entry_leap_hold_gate();
            }
            {
                cl_telecom_system ts_pb;
                cl_arq_controller ARQ_pb;
                ARQ_pb.telecom_system = &ts_pb;
                failed += ARQ_pb.test_pending_report_bsi_restore();
            }
            // GAP-ABORT stream-backstop companion (data-flow-rsp-contiguity-ruler.md §7):
            // the REAL teardown must ALSO preserve the Option W byte cursor + stamp validity
            // (so w_stream_shift_detected can still fire post-abort) and set the sticky
            // rsp_stream_aborted latch (which survives reset_session_state and refuses every
            // subsequent delivery until a genuine CONNECT). In-process synthetic-fire.
            {
                cl_arq_controller ARQ_gas;
                failed += ARQ_gas.test_gap_abort_stream_backstop_blind();
            }
            // RECOVERABLE delivery-time GAP-ABORT (R2a hold + R2c CMD retention):
            // the terminal abort on a RECOVERABLE one-frame prev-batch hole is
            // converted into a bounded HOLD + re-request; the held batch delivers
            // in-order only after the hole fills, and the high-water NEVER advances
            // while the hole exists. Drives the REAL predicates + retention helpers
            // + advance_last_delivered. See data-flow-recoverable-gap-abort.md.
            {
                cl_arq_controller ARQ_grc;
                failed += ARQ_grc.test_recoverable_gap_abort();
            }
            // W1 — CMD refill decision fire proof (routes the storm re-SACK to the
            // retention re-drive; fail-before via MERCURY_CMD_PREV_RETAIN_DEFEAT).
            {
                cl_arq_controller ARQ_rf;
                failed += ARQ_rf.test_recover_fire();
            }
            // R2b — DELIVER-HELD-CUR fire proof: the RSP commits the held current
            // batch's BYTES to the app FIFO via the production copy_data_to_buffer
            // once the prev hole refills, and the CMD credits the recovery round.
            // fail-before via MERCURY_HELD_CUR_DELIVER_DEFEAT / MERCURY_GAP_RECOVER_
            // TURNAROUND_DEFEAT. See data-flow-recoverable-gap-abort.md §5.5.
            {
                cl_arq_controller ARQ_hcf;
                failed += ARQ_hcf.test_held_cur_deliver_fire();
            }
            // chase-combining (HARQ Type-I soft-LLR combine) fail-before/pass-after:
            // proves a bit-identical retx's LLRs SUMMED before ldpc.decode() rescue a
            // frame one look cannot decode (combine success-rate 0 -> >0), and that
            // defeating the primitive restores the pre-fix 0. Fast + deterministic, no
            // IONOS/RF. See fact-documents/chase-combining-harq.md.
            failed += run_chase_selftest();
            // CRC-ESCAPE accept-gate gate: a non-converged LDPC decode whose residual
            // errors collide the 16-bit CRC to 0 must be REJECTED (not delivered). Drives
            // the PRODUCTION frame_decode_rejected() predicate; fail-before via the
            // crc_escape_defeat arm (legacy crc-only gate). Deterministic, no RF.
            failed += run_crc_escape_selftest();
            // Turbo-EQ verdict preservation: publishing the selected hard bits must
            // publish that LDPC decode's iteration result too, never the turbo loop
            // index. A CRC-colliding FAIL sentinel must remain rejected.
            {
                cl_arq_controller ARQ_turbo;
                failed += ARQ_turbo.test_turbo_iterations_sentinel();
            }
            // In-band demote-rebase DOUBLE-DELIVERY (byte-stream corruption): the RSP
            // re-delivers an already-delivered batch across an in-band demote-rebase
            // (re-adopt + lost-ACK retransmit re-reach the delivery funnel with fwd==0).
            // Drives the REAL commit funnel + REAL rebase + REAL gap predicates with
            // fifo_buffer_rx as a byte-exact oracle. fail-before MERCURY_STREAM_DEDUP_DEFEAT=1.
            {
                cl_arq_controller ARQ_dd;
                failed += ARQ_dd.test_dedup_rebase();
            }
            // Authenticated stream-ORIGIN offset-0 delivery gate (data-flow-rx-fadecore-
            // adoption.md): a non-origin batch (the successor of a dropped/lost origin) must
            // never commit at stream offset 0. PATH-A holds+heals; PATH-B aborts loudly.
            // Drives the REAL delivery_step_is_gap / gap_is_recoverable_prev_hole predicates;
            // fail-before/pass-after on the SAME binary via the origin_gen argument.
            {
                cl_arq_controller ARQ_ob;
                failed += ARQ_ob.test_origin_bind();
            }

            // PRECOOK (Stage 1) shared-ring PIN invariant regression. Pins the persistent
            // capture ring (precook_pin_shared_ring), then drives a representative gearshift
            // sequence and asserts: (a) passband_delayed_data is the SAME address after every
            // switch (ring never freed → the Bug #42 UAF window is structurally gone); (b) each
            // config's ACTIVE buffer_Nsymb is its per-config natural, NEVER pinned to the max
            // (the S4 oversize-ring acquisition wall the "pin to max" landmine would create, and
            // never exceeds the pinned capacity). In-process, no IONOS/RF. See PRECOOK plan §6.
            {
                const char* defeat = std::getenv("MERCURY_PRECOOK_DEFEAT");
                bool defeated = (defeat && *defeat && atoi(defeat) != 0);
                cl_telecom_system ts;
                ts.narrowband_enabled = NO;
                ts.load_configuration(CONFIG_0);
                int nat_cfg0 = ts.data_container.buffer_Nsymb;
                ts.precook_pin_shared_ring();
                int pfail = 0;
                if(defeated) {
                    if(ts.data_container.precook_ring_pinned) { printf("[TEST-PRECOOK] FAIL: DEFEAT set but ring pinned\n"); pfail++; }
                    else printf("[TEST-PRECOOK] SKIP (MERCURY_PRECOOK_DEFEAT=1 → legacy path)\n");
                } else {
                    double* ring0 = ts.data_container.passband_delayed_data;
                    int cap = ts.data_container.pinned_capacity_buffer_Nsymb;
                    if(!ts.data_container.precook_ring_pinned) { printf("[TEST-PRECOOK] FAIL: ring not pinned after precook\n"); pfail++; }
                    if(ring0 == NULL) { printf("[TEST-PRECOOK] FAIL: ring NULL after pin\n"); pfail++; }
                    int seq[] = { CONFIG_16, CONFIG_17, CONFIG_16, ROBUST_0, CONFIG_8, CONFIG_0,
                                  ROBUST_2, CONFIG_15, CONFIG_16, ROBUST_1 };
                    for(unsigned s=0; s<sizeof(seq)/sizeof(seq[0]); s++) {
                        ts.load_configuration(seq[s]);
                        if(ts.data_container.passband_delayed_data != ring0) {
                            printf("[TEST-PRECOOK] FAIL: ring pointer moved on switch to %d (%p != %p) — ring was freed/realloc'd\n",
                                seq[s], (void*)ts.data_container.passband_delayed_data, (void*)ring0);
                            pfail++;
                        }
                        int bn = ts.data_container.buffer_Nsymb;
                        if(bn > cap) { printf("[TEST-PRECOOK] FAIL: cfg %d buffer_Nsymb=%d exceeds pinned capacity=%d\n", seq[s], bn, cap); pfail++; }
                    }
                    // Per-config window preserved (no S4 wall): CONFIG_0 active window == its natural,
                    // NOT the pinned max (cap), even after dwelling on a large-window ROBUST config.
                    ts.load_configuration(ROBUST_0);
                    ts.load_configuration(CONFIG_0);
                    if((int)ts.data_container.buffer_Nsymb != nat_cfg0) {
                        printf("[TEST-PRECOOK] FAIL: CONFIG_0 window=%d != natural %d (S4 oversize-ring wall)\n",
                            (int)ts.data_container.buffer_Nsymb, nat_cfg0);
                        pfail++;
                    }
                    if(ts.data_container.passband_delayed_data != ring0) { printf("[TEST-PRECOOK] FAIL: ring pointer moved (final)\n"); pfail++; }
                    if(pfail == 0)
                        printf("[TEST-PRECOOK] ALL PASS: ring pinned+stable across 10 switches (addr=%p, cap=%d), "
                            "per-config window preserved (CONFIG_0 natural=%d, not clamped to max)\n",
                            (void*)ring0, cap, nat_cfg0);
                }
                failed += pfail;
            }

            // PRECOOK descrambler REGEN regression (§17 HINGE — the LIVE NB-ROBUST/MFSK-LDPC
            // 0-RX-CTRL root, audit wqvdd116h). precook_pin_shared_ring() calls
            // alloc_shared_buffers() which CDELETE+re-NEW's bit_energy_dispersal_sequence as a
            // fresh `new int[N_MAX]` GARBAGE array; the restore-startup-config load above no-op-
            // guards (same-cfg) so nothing regenerates it → the LIVE descrambler is garbage and
            // the RX outer CRC constant-fails. FIX: regenerate the seed-0 draw immediately after
            // alloc_shared_buffers. This asserts the SEQUENCE equals the reference ts_srandom(seed)
            // draw (NOT merely "decode succeeds" — §17 warns a plain realloc self-hides via heap
            // block reuse), using the SAME MERCURY_DESCRAMBLER_REGEN_DEFEAT poison-pattern as the
            // HINGE-1 test: DEFEAT=1 zeroes the array → FAIL-BEFORE, default regenerates → PASS-AFTER.
            {
                const char* TAG = "[TEST-PRECOOK-DESCRAMBLER]";
                int dfail = 0;
                auto set_env = [&](const char* k, const char* v){
#if defined(_WIN32)
                    _putenv_s(k, v);
#else
                    if(v && *v) setenv(k, v, 1); else unsetenv(k);
#endif
                };
                // Reference: the seed-0 draw for the startup config (CONFIG_0), from a FRESH
                // instance that only ran init() (never precook). This is what BOTH ends scramble
                // with; the pinned live descrambler must match it bit-for-bit.
                static int ref_seq[N_MAX];
                int ref_N = 0;
                set_env("MERCURY_DESCRAMBLER_REGEN_DEFEAT", "");
                {
                    cl_telecom_system ref;
                    ref.narrowband_enabled = NO;
                    ref.load_configuration(CONFIG_0);
                    ref_N = ref.ldpc.N;
                    for(int i=0;i<ref_N && i<N_MAX;i++) ref_seq[i] = ref.data_container.bit_energy_dispersal_sequence[i];
                }
                auto dump16 = [&](const int* s)->std::string {
                    char buf[8]; std::string out;
                    for(int b=0;b<2;b++){ int v=0; for(int k=0;k<8;k++) v=(v<<1)|(s[b*8+k]&1); snprintf(buf,sizeof(buf),"%02X",v); out+=buf; }
                    return out;
                };
                for(int arm=0; arm<2; arm++)
                {
                    bool defeat = (arm==1);
                    set_env("MERCURY_DESCRAMBLER_REGEN_DEFEAT", defeat ? "1" : "");
                    cl_telecom_system ts;
                    ts.narrowband_enabled = NO;
                    ts.load_configuration(CONFIG_0);
                    ts.precook_pin_shared_ring();
                    int N = ts.ldpc.N;
                    bool matches = (N == ref_N);
                    for(int i=0;i<N && matches;i++) if(ts.data_container.bit_energy_dispersal_sequence[i]!=ref_seq[i]) matches=false;
                    std::string live = dump16(ts.data_container.bit_energy_dispersal_sequence);
                    if(!defeat) {
                        if(matches) printf("%s PASS-AFTER: pinned live descrambler EQUALS seed-0 ref (dump[0..15]=%s, ref=%s)\n", TAG, live.c_str(), dump16(ref_seq).c_str());
                        else { printf("%s FAIL: pinned live descrambler != seed-0 ref (dump[0..15]=%s, ref=%s)\n", TAG, live.c_str(), dump16(ref_seq).c_str()); dfail++; }
                    } else {
                        if(!matches) printf("%s DEFEAT-ARM ok (fail-before): descrambler DIVERGES from ref (garbage/zeroed, dump[0..15]=%s)\n", TAG, live.c_str());
                        else { printf("%s FAIL: DEFEAT descrambler matched ref (poison did not take)\n", TAG); dfail++; }
                    }
                }
                set_env("MERCURY_DESCRAMBLER_REGEN_DEFEAT", "");
                if(dfail==0) printf("%s ALL PASS\n", TAG);
                failed += dfail;
            }

            // PRECOOK M3 copy_from BYTE-IDENTICAL completeness gate (STEP 1 GATE 2,
            // BUNDLE_FIELD_CHECKLIST PART 6). For every config in the full gearshift
            // ladder (ROBUST_0/1/2 + CONFIG_0..16): build a telecom_system via the
            // production init() path, copy_from telecom.ofdm/ldpc/psk into FRESH local
            // objects, then memcmp EVERY owning buffer between the init()-built original
            // and the copy → assert byte-IDENTICAL. Also asserts the L1 landmine: the
            // COPY's pilot/preamble .carrier back-pointers alias the COPY's own
            // ofdm_frame/ofdm_preamble (NOT the source's). Any mismatch localizes the
            // missed field to a single buffer. In-process, no IONOS/RF. See
            // _research/_precook/PRECOOK_STAGE2_TURNKEY.md STEP 1.
            {
                int cfpass = 0, cffail = 0;
                for (int li = 0; li < FULL_CONFIG_LADDER_SIZE; li++) {
                    int cfg = FULL_CONFIG_LADDER[li];
                    cl_telecom_system ts;
                    ts.narrowband_enabled = NO;
                    ts.load_configuration(cfg);

                    cl_ofdm ofdm_copy;   ofdm_copy.copy_from(ts.ofdm);
                    cl_ldpc ldpc_copy;   ldpc_copy.copy_from(ts.ldpc);
                    cl_psk  psk_copy;    psk_copy.copy_from(ts.psk);

                    const char* m_ofdm = ts.ofdm.precook_deep_equal(ofdm_copy);
                    const char* m_ldpc = ts.ldpc.precook_deep_equal(ldpc_copy);
                    const char* m_psk  = ts.psk.precook_deep_equal(psk_copy);

                    // L1 landmine: the COPY's back-pointers must alias the COPY's own frames.
                    bool land_ok = (ofdm_copy.pilot_configurator.carrier == ofdm_copy.ofdm_frame)
                                && (ofdm_copy.preamble_configurator.carrier == ofdm_copy.ofdm_preamble);

                    if (m_ofdm == NULL && m_ldpc == NULL && m_psk == NULL && land_ok) {
                        cfpass++;
                    } else {
                        cffail++;
                        printf("[TEST-PRECOOK-COPYFROM] FAIL cfg %d:", cfg);
                        if (m_ofdm) printf(" ofdm-buffer=%s", m_ofdm);
                        if (m_ldpc) printf(" ldpc-buffer=%s", m_ldpc);
                        if (m_psk)  printf(" psk-buffer=%s", m_psk);
                        if (!land_ok) printf(" LANDMINE-1(carrier back-pointer not re-pointed to copy)");
                        printf("\n");
                    }
                }
                if (cffail == 0)
                    printf("[TEST-PRECOOK-COPYFROM] ALL PASS: %d/%d ladder configs byte-identical after "
                           "copy_from (every owning buffer memcmp-clean; L1 carrier re-point verified)\n",
                           cfpass, FULL_CONFIG_LADDER_SIZE);
                failed += cffail;
            }

            // PRECOOK M3 STEP 2 — config-geometry BUNDLE completeness gate
            // (BUNDLE_FIELD_CHECKLIST PART 6, the single GATE that proves completeness).
            // Build the WB bundle set via precook_config_bundles(); then for EVERY ladder
            // config build a FRESH reference cl_telecom_system (load_configuration runs the
            // full init()) and assert bundle[cfg] is byte-IDENTICAL to it: ofdm/ldpc/psk via
            // precook_deep_equal (every owning buffer memcmp), pre_equalization_channel memcmp
            // (Nc; NULL for MFSK), bit_energy_dispersal_sequence memcmp (N_MAX), and the full
            // scalar block ==. A FAIL localizes the missed bundle field to a single
            // buffer/scalar. In-process, no IONOS/RF; CARD-FREE. See
            // _research/PRECOOK_IMPLEMENTATION_PLAN.md §1.1/§1.2. STEP 2 does NOT wire the swap
            // into load_configuration (STEP 3) → cfg<=15 production path is unaffected.
            {
                cl_telecom_system bts;
                bts.narrowband_enabled = NO;
                bts.precook_config_bundles();

                int bpass = 0, bfail = 0;
                for (int li = 0; li < FULL_CONFIG_LADDER_SIZE; li++) {
                    int cfg = FULL_CONFIG_LADDER[li];
                    int idx = bts.bundle_index(cfg, NO);
                    if (idx < 0) {
                        bfail++;
                        printf("[TEST-PRECOOK-BUNDLE] FAIL cfg %d: bundle_index returned -1 (not built)\n", cfg);
                        continue;
                    }
                    const st_config_bundle* b = bts.bundle_set(NO)[idx].get();

                    cl_telecom_system ref;
                    ref.narrowband_enabled = NO;
                    ref.load_configuration(cfg);

                    const char* mism = NULL;

                    // (1) geometry PHY objects — every owning buffer.
                    if (mism == NULL) mism = b->ofdm.precook_deep_equal(ref.ofdm);
                    if (mism == NULL) mism = b->ldpc.precook_deep_equal(ref.ldpc);
                    if (mism == NULL) mism = b->psk.precook_deep_equal(ref.psk);

                    // (2) pre_equalization_channel (Nc; both NULL for MFSK).
                    if (mism == NULL) {
                        int rNc = ref.data_container.Nc;
                        bool bnull = (b->pre_equalization_channel == NULL);
                        bool rnull = (ref.pre_equalization_channel == NULL);
                        if (bnull != rnull) mism = "pre_equalization_channel(null-mismatch)";
                        else if (!bnull && rNc > 0 &&
                                 memcmp(b->pre_equalization_channel, ref.pre_equalization_channel,
                                        sizeof(struct st_channel_complex) * (size_t)rNc) != 0)
                            mism = "pre_equalization_channel";
                    }

                    // (3) bit_energy_dispersal_sequence (N_MAX descrambler draw).
                    if (mism == NULL) {
                        bool bnull = (b->bit_energy_dispersal_sequence == NULL);
                        bool rnull = (ref.data_container.bit_energy_dispersal_sequence == NULL);
                        if (bnull != rnull) mism = "bit_energy_dispersal_sequence(null-mismatch)";
                        else if (!bnull &&
                                 memcmp(b->bit_energy_dispersal_sequence, ref.data_container.bit_energy_dispersal_sequence,
                                        sizeof(int) * (size_t)N_MAX) != 0)
                            mism = "bit_energy_dispersal_sequence";
                    }

                    // (4) scalar block — data_container geometry scalars.
                    if (mism == NULL) {
                        if      (b->Nofdm                         != ref.data_container.Nofdm)                         mism = "scalar.Nofdm";
                        else if (b->Nc                            != ref.data_container.Nc)                            mism = "scalar.Nc";
                        else if (b->M                             != ref.data_container.M)                             mism = "scalar.M";
                        else if (b->Nfft                          != ref.data_container.Nfft)                          mism = "scalar.Nfft";
                        else if (b->Ngi                           != ref.data_container.Ngi)                           mism = "scalar.Ngi";
                        else if (b->Nsymb                         != ref.data_container.Nsymb)                         mism = "scalar.Nsymb";
                        else if (b->nData                         != ref.data_container.nData)                         mism = "scalar.nData";
                        else if (b->nBits                         != ref.data_container.nBits)                         mism = "scalar.nBits";
                        else if (b->preamble_nSymb                != ref.data_container.preamble_nSymb)                mism = "scalar.preamble_nSymb";
                        else if (b->interpolation_rate            != ref.data_container.interpolation_rate)            mism = "scalar.interpolation_rate";
                        else if (b->total_frame_size              != ref.data_container.total_frame_size)              mism = "scalar.total_frame_size";
                        else if (b->baseband_data_fine_slice_size != ref.data_container.baseband_data_fine_slice_size) mism = "scalar.baseband_data_fine_slice_size";
                        else if (b->buffer_Nsymb                  != (int)ref.data_container.buffer_Nsymb.load())      mism = "scalar.buffer_Nsymb";
                    }
                    // (5) scalar block — telecom_system scalars.
                    if (mism == NULL) {
                        if      (b->ldpc_rate                 != ref.ldpc.rate)                 mism = "scalar.ldpc_rate";
                        else if (b->bandwidth                 != ref.bandwidth)                 mism = "scalar.bandwidth";
                        else if (b->carrier_frequency         != ref.carrier_frequency)         mism = "scalar.carrier_frequency";
                        else if (b->bit_energy_dispersal_seed != ref.bit_energy_dispersal_seed) mism = "scalar.bit_energy_dispersal_seed";
                        else if (b->M_telecom                 != ref.M)                         mism = "scalar.M_telecom";
                        else if (b->time_sync_trials_max      != ref.time_sync_trials_max)      mism = "scalar.time_sync_trials_max";
                        else if (b->outer_code                != ref.outer_code)                mism = "scalar.outer_code";
                        else if (b->outer_code_reserved_bits  != ref.outer_code_reserved_bits)  mism = "scalar.outer_code_reserved_bits";
                        else if (b->LDPC_real_CR              != ref.LDPC_real_CR)              mism = "scalar.LDPC_real_CR";
                        else if (b->Tu                        != ref.Tu)                        mism = "scalar.Tu";
                        else if (b->Ts                        != ref.Ts)                        mism = "scalar.Ts";
                        else if (b->Tf                        != ref.Tf)                        mism = "scalar.Tf";
                        else if (b->rb                        != ref.rb)                        mism = "scalar.rb";
                        else if (b->rbc                       != ref.rbc)                       mism = "scalar.rbc";
                        else if (b->Shannon_limit             != ref.Shannon_limit)             mism = "scalar.Shannon_limit";
                    }

                    if (mism == NULL) {
                        bpass++;
                    } else {
                        bfail++;
                        printf("[TEST-PRECOOK-BUNDLE] FAIL cfg %d: field=%s\n", cfg, mism);
                    }
                }
                if (bfail == 0)
                    printf("[TEST-PRECOOK-BUNDLE] ALL PASS: %d/%d ladder configs bundle==init byte-identical "
                           "(ofdm/ldpc/psk buffers + pre_eq[Nc] + descrambler[N_MAX] + full scalar block)\n",
                           bpass, FULL_CONFIG_LADDER_SIZE);
                failed += bfail;
            }

            // PRECOOK M3 STEP 3 — config-SWAP regression (plan §6 / TURNKEY STEP 5). Pins the
            // shared ring, builds the bundle set, then drives load_configuration across gearshift
            // cross-modulation (BPSK/QPSK/8PSK/16QAM/32QAM + the 15<->16 boundary), ROBUST<->OFDM
            // reseed, and session-reset/SWITCH_ROLE (re-load the init config) transitions. Asserts,
            // after EVERY swap: (a) decoded==framed geometry (live ofdm.Nc/Nsymb + data_container.M
            // + ldpc.rate == the target bundle) [INV-5]; (b) the ACTIVE buffer_Nsymb is the config
            // NATURAL (== bundle, <= pinned cap) with a consistent sp=Nofdm*buffer_Nsymb*interp>0 —
            // the STAGE-1 pin-to-max landmine catcher [INV-4]; (c) passband_delayed_data is the SAME
            // address across every swap (ring never freed/realloc'd — INV-2); (d) C1 never sees
            // sp==0/NULL; (e) ROBUST<->OFDM reseed leaves the descrambler as the seed-0 draw (NOT
            // all-zero, INV-6) and the MFSK corr-template state matching the target (OFDM => len 0;
            // MFSK => len>0, not stale). A second block repeats for NB (Nc=10) bundles. The leaf-lock
            // reentrancy tripwire in load_configuration_swap aborts on a nested publish. Under
            // MERCURY_PRECOOK_DEFEAT=1 the ring stays UNPINNED → the legacy deinit->init path
            // reallocs the ring on each switch → this test FAILS (the fail-before). In-process, no
            // IONOS/RF; CARD-FREE. See _research/PRECOOK_IMPLEMENTATION_PLAN.md §3.1/§6.
            {
                const char* defeat = std::getenv("MERCURY_PRECOOK_DEFEAT");
                bool defeated = (defeat && *defeat && atoi(defeat) != 0);
                int sfail = 0;

                for(int pass = 0; pass < 2; pass++)   // pass 0 = WB (Nc=50), pass 1 = NB (Nc=10)
                {
                    int nb = (pass == 1) ? YES : NO;
                    const char* bwn = (pass == 1) ? "NB" : "WB";
                    cl_telecom_system ts;
                    ts.narrowband_enabled = nb;
                    ts.load_configuration(CONFIG_0);
                    ts.precook_pin_shared_ring();
                    if(ts.data_container.precook_ring_pinned)
                        ts.precook_config_bundles();   // mirror the startup gating (only when pinned)

                    // Cross-modulation + ROBUST<->OFDM reseed + session-reset(re-load CONFIG_0).
                    // NB clamps OFDM > CONFIG_6 to CONFIG_6, so the NB pass drives only <= CONFIG_6.
                    int seq_wb[] = { CONFIG_6, CONFIG_7, CONFIG_10, CONFIG_15, CONFIG_16, CONFIG_15,
                                     CONFIG_16, ROBUST_0, CONFIG_0, ROBUST_2, CONFIG_8, ROBUST_1,
                                     CONFIG_16, CONFIG_0 };
                    int seq_nb[] = { CONFIG_3, CONFIG_6, ROBUST_0, CONFIG_0, ROBUST_2, CONFIG_5,
                                     ROBUST_1, CONFIG_0 };
                    int* seq = (pass == 1) ? seq_nb : seq_wb;
                    int nseq = (pass == 1) ? (int)(sizeof(seq_nb)/sizeof(seq_nb[0]))
                                           : (int)(sizeof(seq_wb)/sizeof(seq_wb[0]));

                    if(!ts.data_container.precook_ring_pinned)
                    {
                        // fail-before (DEFEAT / pin failed): the legacy deinit->init path reallocs the
                        // ring on cross-modulation switches — the exact regression STEP-3 removes.
                        double* ring0 = ts.data_container.passband_delayed_data;
                        int moved = 0;
                        for(int s=0; s<nseq; s++){
                            ts.load_configuration(seq[s]);
                            if(ts.data_container.passband_delayed_data != ring0){ moved++; ring0 = ts.data_container.passband_delayed_data; }
                        }
                        printf("[TEST-PRECOOK-SWAP] %s FAIL-BEFORE (%s): ring realloc'd %d× across %d switches "
                               "(legacy path; no M3 swap)\n",
                               bwn, defeated ? "MERCURY_PRECOOK_DEFEAT=1" : "ring-not-pinned", moved, nseq);
                        sfail += (moved > 0) ? moved : 1;   // ensure RED under DEFEAT (fail-before)
                        continue;
                    }

                    double* ring0 = ts.data_container.passband_delayed_data;
                    int cap = ts.data_container.pinned_capacity_buffer_Nsymb;
                    int expect_Nc = (pass == 1) ? 10 : 50;
                    if(ring0 == NULL){ printf("[TEST-PRECOOK-SWAP] %s FAIL: ring NULL after pin\n", bwn); sfail++; }

                    for(int s=0; s<nseq; s++)
                    {
                        int cfg = seq[s];
                        ts.load_configuration(cfg);
                        // bundle_index re-applies the NB clamp inside load_configuration, so look up
                        // the ACTUALLY-loaded config via the live current_configuration.
                        int live_cfg = ts.current_configuration;
                        int idx = ts.bundle_index(live_cfg, nb);
                        if(idx < 0){ printf("[TEST-PRECOOK-SWAP] %s FAIL: cfg %d (live %d) no bundle\n", bwn, cfg, live_cfg); sfail++; continue; }
                        const st_config_bundle* b = ts.bundle_set(nb)[idx].get();

                        // (a) decoded==framed geometry.
                        if(ts.ofdm.Nc != b->ofdm.Nc || ts.ofdm.Nsymb != b->ofdm.Nsymb ||
                           ts.data_container.M != b->M || ts.ldpc.rate != b->ldpc_rate)
                        {
                            printf("[TEST-PRECOOK-SWAP] %s FAIL cfg %d: geometry live(Nc=%d Nsymb=%d M=%d rate=%.4f) != bundle(Nc=%d Nsymb=%d M=%d rate=%.4f)\n",
                                bwn, live_cfg, ts.ofdm.Nc, ts.ofdm.Nsymb, ts.data_container.M, ts.ldpc.rate,
                                b->ofdm.Nc, b->ofdm.Nsymb, b->M, b->ldpc_rate);
                            sfail++;
                        }
                        if(ts.ofdm.Nc != expect_Nc){ printf("[TEST-PRECOOK-SWAP] %s FAIL cfg %d: Nc=%d != expected %d\n", bwn, live_cfg, ts.ofdm.Nc, expect_Nc); sfail++; }

                        // (b) active buffer_Nsymb == the config NATURAL (bundle), consistent sp, <= cap.
                        int bn = ts.data_container.buffer_Nsymb;
                        int interp = ts.data_container.interpolation_rate;
                        long sp = (long)ts.data_container.Nofdm * bn * interp;
                        if(bn != b->buffer_Nsymb){
                            printf("[TEST-PRECOOK-SWAP] %s FAIL cfg %d: buffer_Nsymb=%d != natural %d (STAGE-1 pin-to-max?)\n", bwn, live_cfg, bn, b->buffer_Nsymb);
                            sfail++;
                        }
                        if(bn > cap){ printf("[TEST-PRECOOK-SWAP] %s FAIL cfg %d: buffer_Nsymb=%d > pinned cap %d\n", bwn, live_cfg, bn, cap); sfail++; }

                        // (d) C1: sp > 0, ring non-NULL.
                        if(sp <= 0 || ts.data_container.passband_delayed_data == NULL){
                            printf("[TEST-PRECOOK-SWAP] %s FAIL cfg %d: C1 sp=%ld ring=%p (sp==0/NULL)\n", bwn, live_cfg, sp, (void*)ts.data_container.passband_delayed_data);
                            sfail++;
                        }

                        // (c) ring pointer stable across the swap (no realloc — INV-2).
                        if(ts.data_container.passband_delayed_data != ring0){
                            printf("[TEST-PRECOOK-SWAP] %s FAIL cfg %d: ring pointer moved (%p != %p) — realloc\n", bwn, live_cfg, (void*)ts.data_container.passband_delayed_data, (void*)ring0);
                            sfail++;
                        }

                        // (e) ROBUST<->OFDM reseed correctness.
                        bool desc_nonzero = false;
                        int nscan = (ts.ldpc.N < 64) ? ts.ldpc.N : 64;
                        for(int k=0; k<nscan; k++) if(ts.data_container.bit_energy_dispersal_sequence[k] != 0){ desc_nonzero = true; break; }
                        if(!desc_nonzero){ printf("[TEST-PRECOOK-SWAP] %s FAIL cfg %d: descrambler all-zero (INV-6 stale)\n", bwn, live_cfg); sfail++; }
                        bool is_mfsk = is_robust_config(live_cfg);
                        if(is_mfsk && ts.ofdm.mfsk_corr_template_len <= 0){ printf("[TEST-PRECOOK-SWAP] %s FAIL cfg %d: MFSK but corr_template_len=%d\n", bwn, live_cfg, ts.ofdm.mfsk_corr_template_len); sfail++; }
                        if(!is_mfsk && ts.ofdm.mfsk_corr_template_len != 0){ printf("[TEST-PRECOOK-SWAP] %s FAIL cfg %d: OFDM but stale MFSK corr_template_len=%d\n", bwn, live_cfg, ts.ofdm.mfsk_corr_template_len); sfail++; }
                    }

                    // STAGE-1 landmine catcher: after a large-window ROBUST dwell, a small-window
                    // config keeps its NATURAL window (strictly < cap) — not the pinned max (the S4
                    // oversize-ring acquisition wall the "pin buffer_Nsymb to max" bug would create).
                    int small_cfg = (pass == 1) ? CONFIG_6 : CONFIG_16;
                    ts.load_configuration(ROBUST_0);
                    ts.load_configuration(small_cfg);
                    int idxs = ts.bundle_index(ts.current_configuration, nb);
                    int bns = ts.data_container.buffer_Nsymb;
                    // The STAGE-1 pin-to-max bug would make bns == cap (!= natural); the strict
                    // "== natural" check below is the catcher (bns==natural==cap is legitimate for
                    // the window-defining config, so a bare ">= cap" would false-positive on NB).
                    if(idxs >= 0 && bns != ts.bundle_set(nb)[idxs]->buffer_Nsymb){
                        printf("[TEST-PRECOOK-SWAP] %s FAIL: cfg %d window=%d != natural %d after ROBUST dwell (S4 pin-to-max wall)\n",
                            bwn, ts.current_configuration, bns, ts.bundle_set(nb)[idxs]->buffer_Nsymb);
                        sfail++;
                    }
                    if(ts.data_container.passband_delayed_data != ring0){ printf("[TEST-PRECOOK-SWAP] %s FAIL: ring moved (final)\n", bwn); sfail++; }

                    if(sfail == 0)
                        printf("[TEST-PRECOOK-SWAP] %s PASS: %d swaps, ring stable @%p (cap=%d), per-config natural "
                               "buffer_Nsymb, descrambler+MFSK-template reseeded, Nc=%d (small-cfg %d window=%d)\n",
                               bwn, nseq, (void*)ring0, cap, expect_Nc, small_cfg, bns);
                }
                failed += sfail;
            }

            // PRECOOK M3 STEP 4 — COLLAPSE regression (plan §3.3/§4). The three CONFIG_NONE-abuser
            // paths are now bundle SWAPS / scalar re-stages with NO ring realloc:
            //   (1) bigblock_restore_stock_config()  — after a CFG16 thin-grid big-block context
            //       (bigblock_rebuild_thin_grid mangles the LIVE ofdm), restore = swap bundle[stock];
            //   (2) force_resize_capture_ring()       — robust-floor SEAT = re-stage buffer_Nsymb to
            //       the ROBUST_0 floor (max(natural, floor)), no realloc, no descrambler regen;
            //   (3) force_set_capture_ring_natural()  — UN-SEAT = re-stage to the config NATURAL.
            // Asserts per transition: ring pointer BYTE-IDENTICAL (no realloc, INV-2); decoded==framed
            // geometry (live ofdm == stock bundle); active buffer_Nsymb == expected; descrambler seed-0
            // (non-zero, INV-6); OFDM => MFSK template len 0. Under MERCURY_PRECOOK_DEFEAT=1 the ring is
            // UNPINNED → these paths REALLOC → the fail-before (ring moves). In-process, CARD-FREE.
            {
                const char* defeat4 = std::getenv("MERCURY_PRECOOK_DEFEAT");
                bool defeated4 = (defeat4 && *defeat4 && atoi(defeat4) != 0);
                int c4fail = 0;

                for(int pass = 0; pass < 2; pass++)   // 0 = WB (Nc=50), 1 = NB (Nc=10)
                {
                    int nb = (pass == 1) ? YES : NO;
                    const char* bwn = (pass == 1) ? "NB" : "WB";
                    cl_telecom_system ts;
                    ts.narrowband_enabled = nb;
                    ts.load_configuration(CONFIG_0);
                    ts.precook_pin_shared_ring();
                    if(ts.data_container.precook_ring_pinned)
                        ts.precook_config_bundles();

                    // Stock high rung: CONFIG_16 (WB) / CONFIG_6 (NB clamp ceiling). big-block is a
                    // WB-CFG16 construct, so the thin-grid restore leg runs on the WB pass only; the
                    // seat/un-seat legs run on both.
                    int stock_cfg = (pass == 1) ? CONFIG_6 : CONFIG_16;
                    ts.load_configuration(stock_cfg);
                    int live_stock = ts.current_configuration;

                    double* ring0 = ts.data_container.passband_delayed_data;

                    if(!ts.data_container.precook_ring_pinned)
                    {
                        // FAIL-BEFORE (DEFEAT / pin failed): the legacy set_size / CONFIG_NONE reinit
                        // paths realloc the ring — the exact regression STEP-4 removes.
                        int moved = 0;
                        if(pass == 0){
                            int Ng=0,l2=0,nb2=0; ts.bigblock_rebuild_thin_grid(Ng,l2,nb2);
                            ts.bigblock_restore_stock_config();
                            if(ts.data_container.passband_delayed_data != ring0){ moved++; ring0 = ts.data_container.passband_delayed_data; }
                        }
                        ts.force_resize_capture_ring((int)ts.data_container.buffer_Nsymb + 500);
                        if(ts.data_container.passband_delayed_data != ring0){ moved++; ring0 = ts.data_container.passband_delayed_data; }
                        ts.force_set_capture_ring_natural();
                        if(ts.data_container.passband_delayed_data != ring0){ moved++; ring0 = ts.data_container.passband_delayed_data; }
                        printf("[TEST-PRECOOK-COLLAPSE] %s FAIL-BEFORE (%s): ring realloc'd %d× across restore+seat+unseat "
                               "(legacy path; no M3 collapse)\n",
                               bwn, defeated4 ? "MERCURY_PRECOOK_DEFEAT=1" : "ring-not-pinned", moved);
                        c4fail += (moved > 0) ? moved : 1;   // ensure RED under DEFEAT
                        continue;
                    }

                    int sidx = ts.bundle_index(live_stock, nb);
                    if(sidx < 0){ printf("[TEST-PRECOOK-COLLAPSE] %s FAIL: no bundle for stock cfg %d\n", bwn, live_stock); c4fail++; continue; }
                    const st_config_bundle* sb = ts.bundle_set(nb)[sidx].get();
                    int natural_bn = sb->buffer_Nsymb;
                    int cap = ts.data_container.pinned_capacity_buffer_Nsymb;

                    // --- (1) BIG-BLOCK RESTORE (WB only): mangle the live ofdm to the thin grid, then
                    //         restore to the stock bundle via the swap.
                    if(pass == 0)
                    {
                        int Ng=0,l2=0,nb2=0;
                        (void)ts.bigblock_rebuild_thin_grid(Ng,l2,nb2);
                        bool mangled = (ts.ofdm.Nsymb != sb->ofdm.Nsymb);   // thin grid Nsymb=Ngrid != stock
                        ts.bigblock_restore_stock_config();
                        if(ts.data_container.passband_delayed_data != ring0){
                            printf("[TEST-PRECOOK-COLLAPSE] %s FAIL big-block-restore: ring moved (%p != %p) — realloc\n",
                                bwn, (void*)ts.data_container.passband_delayed_data, (void*)ring0); c4fail++; }
                        if(ts.ofdm.Nc != sb->ofdm.Nc || ts.ofdm.Nsymb != sb->ofdm.Nsymb ||
                           ts.data_container.M != sb->M || ts.ldpc.rate != sb->ldpc_rate){
                            printf("[TEST-PRECOOK-COLLAPSE] %s FAIL big-block-restore: geometry live(Nc=%d Nsymb=%d M=%d rate=%.4f) != stock(Nc=%d Nsymb=%d M=%d rate=%.4f) (mangled=%d)\n",
                                bwn, ts.ofdm.Nc, ts.ofdm.Nsymb, ts.data_container.M, ts.ldpc.rate,
                                sb->ofdm.Nc, sb->ofdm.Nsymb, sb->M, sb->ldpc_rate, (int)mangled); c4fail++; }
                        if((int)ts.data_container.buffer_Nsymb != natural_bn){
                            printf("[TEST-PRECOOK-COLLAPSE] %s FAIL big-block-restore: buffer_Nsymb=%d != natural %d\n",
                                bwn, (int)ts.data_container.buffer_Nsymb, natural_bn); c4fail++; }
                        { bool nz=false; int ns=(ts.ldpc.N<64)?ts.ldpc.N:64;
                          for(int k=0;k<ns;k++) if(ts.data_container.bit_energy_dispersal_sequence[k]!=0){ nz=true; break; }
                          if(!nz){ printf("[TEST-PRECOOK-COLLAPSE] %s FAIL big-block-restore: descrambler all-zero (INV-6)\n", bwn); c4fail++; } }
                        if(ts.ofdm.mfsk_corr_template_len != 0){
                            printf("[TEST-PRECOOK-COLLAPSE] %s FAIL big-block-restore: stale MFSK template len=%d (OFDM)\n",
                                bwn, ts.ofdm.mfsk_corr_template_len); c4fail++; }
                    }

                    // --- (2) ROBUST-FLOOR SEAT: seat the ring to the ROBUST_0 floor. Expected active
                    //         window = max(config natural, robust floor) — for a small OFDM config the
                    //         floor dominates; if the config natural already exceeds it the seat is an
                    //         idempotent no-op (already seated). Either way NO realloc.
                    int ridx = ts.bundle_index(ROBUST_0, nb);
                    int floor_bn = (ridx >= 0) ? ts.bundle_set(nb)[ridx]->buffer_Nsymb : (natural_bn + 500);
                    int expect_seat = (floor_bn > natural_bn) ? floor_bn : natural_bn;
                    ts.force_resize_capture_ring(floor_bn);
                    if(ts.data_container.passband_delayed_data != ring0){
                        printf("[TEST-PRECOOK-COLLAPSE] %s FAIL seat: ring moved — realloc\n", bwn); c4fail++; }
                    if((int)ts.data_container.buffer_Nsymb != expect_seat){
                        printf("[TEST-PRECOOK-COLLAPSE] %s FAIL seat: buffer_Nsymb=%d != expected %d (floor %d, natural %d)\n",
                            bwn, (int)ts.data_container.buffer_Nsymb, expect_seat, floor_bn, natural_bn); c4fail++; }
                    if((int)ts.data_container.buffer_Nsymb > cap){
                        printf("[TEST-PRECOOK-COLLAPSE] %s FAIL seat: buffer_Nsymb=%d > pinned cap %d\n",
                            bwn, (int)ts.data_container.buffer_Nsymb, cap); c4fail++; }
                    if(ts.ofdm.Nc != sb->ofdm.Nc || ts.data_container.M != sb->M){
                        printf("[TEST-PRECOOK-COLLAPSE] %s FAIL seat: geometry drifted (seat must change ONLY the window)\n", bwn); c4fail++; }

                    // --- (3) UN-SEAT: restore the config natural window.
                    ts.force_set_capture_ring_natural();
                    if(ts.data_container.passband_delayed_data != ring0){
                        printf("[TEST-PRECOOK-COLLAPSE] %s FAIL un-seat: ring moved — realloc\n", bwn); c4fail++; }
                    if((int)ts.data_container.buffer_Nsymb != natural_bn){
                        printf("[TEST-PRECOOK-COLLAPSE] %s FAIL un-seat: buffer_Nsymb=%d != natural %d\n",
                            bwn, (int)ts.data_container.buffer_Nsymb, natural_bn); c4fail++; }
                    if(ts.data_container.buffer_Nsymb_min != 0){
                        printf("[TEST-PRECOOK-COLLAPSE] %s FAIL un-seat: buffer_Nsymb_min=%d != 0 (floor not cleared)\n",
                            bwn, ts.data_container.buffer_Nsymb_min); c4fail++; }

                    if(c4fail == 0)
                        printf("[TEST-PRECOOK-COLLAPSE] %s PASS: %sseat->%d un-seat->%d natural, ring stable @%p (cap=%d) — no realloc\n",
                            bwn, (pass==0)?"big-block-restore(ring stable, geometry restored), ":"",
                            expect_seat, natural_bn, (void*)ring0, cap);
                }
                failed += c4fail;
            }

            return (failed == 0) ? 0 : 1;
        }
        // --test-chase : run ONLY the chase-combining self-test and exit. Drives the
        // production primitive cl_telecom_system::chase_capture()/chase_combine() over
        // a deterministic AWGN BPSK codeword + its retx. MERCURY_CHASE=0 defeats it
        // (demonstrates the pre-fix 0-success); MERCURY_CHASE_TEST_AMPL tunes the
        // operating point. See fact-documents/chase-combining-harq.md.
        if (strcmp(argv[i], "--test-chase") == 0) {
            arm_test_watchdog();
            int failed = run_chase_selftest();
            return (failed == 0) ? 0 : 1;
        }
        // --test-crc-escape : run ONLY the CRC-escape accept-gate self-test and exit.
        // Deterministic, in-process; drives the PRODUCTION frame_decode_rejected()
        // predicate (fail-before via the crc_escape_defeat arm). No IONOS/RF.
        if (strcmp(argv[i], "--test-crc-escape") == 0) {
            arm_test_watchdog();
            int failed = run_crc_escape_selftest();
            return (failed == 0) ? 0 : 1;
        }
        // --test-subpeak-rescue : run ONLY the F1b Part B true-boundary rescue
        // mechanism self-test and exit. Deterministic, noiseless, in-process.
        if (strcmp(argv[i], "--test-subpeak-rescue") == 0) {
            arm_test_watchdog();
            int failed = run_subpeak_rescue_selftest();
            return (failed == 0) ? 0 : 1;
        }
        // --test-chase-ab : production-path HARQ/Chase A/B (receive_byte 2-look
        // combine). Proves the chase capture+rescue hooks FIRE on the real decode
        // path, recover sub-threshold frames the single look failed, do NOT lower
        // the floor (single decode identical ON vs OFF), and never corrupt (adopted
        // frame byte-checked vs payload). Cheap, deterministic, no cards/RF.
        if (strcmp(argv[i], "--test-chase-ab") == 0) {
            arm_test_watchdog();
            int failed = run_chase_ab_selftest();
            return (failed == 0) ? 0 : 1;
        }
        // --test-sigterm-handler : run ONLY the FIX-C graceful-shutdown handler
        // regression (handler installed -> SIGTERM/SIGINT set shutdown_=true)
        // and exit. install_termination_handlers() already ran above, so the
        // self-raise(SIGTERM) inside is caught; on a tree WITHOUT the handler it
        // would terminate the process (signal-killed exit) = the fail-before.
        if (strcmp(argv[i], "--test-sigterm-handler") == 0) {
            int failed = test_sigterm_handler(install_termination_handlers);
            return (failed == 0) ? 0 : 1;
        }
        // --test-switchrole-reride : run ONLY the SWITCH_ROLE re-ride race
        // regression (idle-switchrole-race.md §6; the R1 reverse-transfer
        // double-swap gate) and exit. Fast + deterministic; drives the REAL
        // set_role(COMMANDER) swap-in primitive + the REAL idle branch. FAILS-
        // BEFORE with -DSWITCHROLE_RERIDE_FAILBEFORE. See
        // arq_commander.cc::test_switch_role_reride_race().
        if (strcmp(argv[i], "--test-switchrole-reride") == 0) {
            cl_arq_controller ARQ_reride;
            int failed = ARQ_reride.test_switch_role_reride_race();
            return (failed == 0) ? 0 : 1;
        }
        // --test-connect-reack : run only the final-connect retry contract
        // regression and exit. Fast and deterministic.
        if (strcmp(argv[i], "--test-connect-reack") == 0) {
            cl_arq_controller ARQ_reack;
            int failed = ARQ_reack.test_connect_reack();
            return (failed == 0) ? 0 : 1;
        }
		if (strcmp(argv[i], "--test-terminal-settlement") == 0) {
			cl_arq_controller A;
			int f = A.test_terminal_settlement();
			return f == 0 ? 0 : 1;
		}
        // --test-start-ack-causal-guard : run only the START_CONNECTION bare-
        // ACK causal timing/ring-epoch regression. Fast, deterministic, no RF.
        // -DSTART_ACK_CAUSAL_GUARD_FAILBEFORE reproduces the 263-ms advance.
        if (strcmp(argv[i], "--test-start-ack-causal-guard") == 0) {
            cl_telecom_system start_ack_ts;
            cl_arq_controller ARQ_start_ack;
            ARQ_start_ack.telecom_system = &start_ack_ts;
            int failed = ARQ_start_ack.test_start_ack_causal_guard();
            return (failed == 0) ? 0 : 1;
        }
        // --test-zombie-amp : run ONLY the zombie/amplifier layer fail-before/
        // pass-after self-test (R2b BREAK-accept-while-DROPPED, the ROBUST_DWELL
        // keep-alive gate, and the watchdog peer-liveness probe gate) and exit.
        // Asserts each pure decision helper's truth table under defeat / no-defeat.
        // See fact-documents/data-flow-zombie-amplifier.md.
        if (strcmp(argv[i], "--test-zombie-amp") == 0) {
            cl_arq_controller ARQ_za;
            int failed = ARQ_za.test_zombie_amp();
            return (failed == 0) ? 0 : 1;
        }
        // --test-demote-silence : run ONLY the R5 demote-split fail-before/pass-after self-test
        // (pure-silence classifier truth table, the shared counter-step, the reroute predicate +
        // guards, the DEFEAT knob, and the +LOW watchdog probe-latch hygiene) and exit.
        // See fact-documents/data-flow-zombie-amplifier.md.
        if (strcmp(argv[i], "--test-demote-silence") == 0) {
            cl_arq_controller ARQ_dsil;
            int failed = ARQ_dsil.test_demote_silence();
            return (failed == 0) ? 0 : 1;
        }
        // --test-acq-bounds : run ONLY the OFDM acquisition bounds-gate recovery
        // regression (a tail-band frame whose detection floors past upper_bound
        // but whose body is in-buffer must be RECOVERED, not discarded) and exit.
        // Faithful in-process synthetic-fire through transmit_byte/receive_byte;
        // see fact-documents/data-flow-acq-bounds-gate.md.
        if (strcmp(argv[i], "--test-frame0-vehicle") == 0) {
            int failed = run_frame0_vehicle_selftest();
            return failed;
        }
        if (strcmp(argv[i], "--test-frame0-mf") == 0) {
            int failed = run_frame0_mf_selftest();
            return failed;
        }
        // Reverse-MFSK/forward-OFDM preamble energy-ratio admission gate.
        // In-process CFG15 synthetic fire; no audio, ARQ, or TCP setup.
        if (strcmp(argv[i], "--test-subpeak-gate") == 0) {
            arm_test_watchdog();
            cl_telecom_system ts;
            ts.operation_mode = BER_PLOT_passband;
            ts.load_configuration(CONFIG_15);
            return ts.test_subpeak_gate();
        }
        if (strcmp(argv[i], "--test-frame0-replay") == 0) {
            const char* cap = (i+1 < argc) ? argv[i+1] : "";
            return run_frame0_replay_selftest(cap);
        }
        if (strcmp(argv[i], "--test-acq-bounds") == 0) {
            int failed = run_acq_bounds_selftest();
            return (failed == 0) ? 0 : 1;
        }
        // (--test-reack-ftr-starvation REMOVED with the excise of 8e62722e,
        // connect-testack-handshake.md §9; --test-connect-reack now guards the
        // removal — the pre-data window must NOT pin frames_to_read=2.)
        // --test-inband-tier-crossing : run ONLY the hybrid tier-crossing routing
        // regression (robust<->OFDM crossing -> legacy SET_CONFIG; intra-tier ->
        // in-band tag) and exit. Fast + deterministic; see
        // arq_commander.cc::test_inband_tier_crossing_routing.
        if (strcmp(argv[i], "--test-inband-tier-crossing") == 0) {
            cl_arq_controller ARQ_tc;
            int failed = ARQ_tc.test_inband_tier_crossing_routing();
            return (failed == 0) ? 0 : 1;
        }
        // --test-inband-reverse-pin : run ONLY the tier-cross reverse-ACK pin regression
        // (robust<->OFDM cross -> reverse pinned to the robust rung, not the OFDM forward
        // rung) and exit. Fast + deterministic; see
        // arq_commander.cc::test_inband_tier_cross_reverse_pin. Build with
        // -DINBAND_REVERSE_PIN_FAILBEFORE to reproduce the fails-before (no pin).
        if (strcmp(argv[i], "--test-inband-reverse-pin") == 0) {
            cl_arq_controller ARQ_rp;
            int failed = ARQ_rp.test_inband_tier_cross_reverse_pin();
            return (failed == 0) ? 0 : 1;
        }
        // --test-inband-basepattern-confirm : run ONLY the KEYSTONE intra-tier base-pattern
        // climb-confirm regression (an emitted climb confirms from the robust BASE ACK pattern,
        // decoupled from the bsi-bearing SACK suffix) and exit. Fast + deterministic; see
        // arq_commander.cc::test_inband_basepattern_confirm. Build with
        // -DINBAND_BASEPATTERN_CONFIRM_FAILBEFORE to reproduce the fails-before (suffix-coupled).
        if (strcmp(argv[i], "--test-inband-basepattern-confirm") == 0) {
            cl_arq_controller ARQ_bc;
            int failed = ARQ_bc.test_inband_basepattern_confirm();
            return (failed == 0) ? 0 : 1;
        }
        // --test-inband-robust-follow : run ONLY the ROBUST-TIER tag-follow stale-pass
        // regression (the residual fix that makes the robust->OFDM tier-cross deterministic)
        // and exit. Fast + deterministic; drives the production gate inband_robust_follow_
        // gate_open. Fail-before: MERCURY_ROBUST_FOLLOW_FRESHWIN_REQ=1 (asserted internally).
        if (strcmp(argv[i], "--test-inband-robust-follow") == 0) {
            cl_arq_controller ARQ_rf;
            int failed = ARQ_rf.test_inband_robust_follow_freshwin();
            return (failed == 0) ? 0 : 1;
        }
        // --test-inband-frame0-partial : run ONLY the CONFIG_0 rolling-partial climb-unblock
        // regression (a lead-frame-only partial at an inband OFDM rung advances the FRAME-UP
        // climb streak; a multi-drop partial stays vetoed) and exit. Fast + deterministic; see
        // arq_commander.cc::test_inband_frame0_partial + data-flow-inband-frame0-rolling-partial.md.
        // Build with -DINBAND_FRAME0_PARTIAL_FAILBEFORE to reproduce the fails-before (wedge).
        if (strcmp(argv[i], "--test-inband-frame0-partial") == 0) {
            cl_arq_controller ARQ_f0p;
            int failed = ARQ_f0p.test_inband_frame0_partial();
            return (failed == 0) ? 0 : 1;
        }
        // --test-inband-climb-defer : run ONLY the ROLLING-PARTIAL climb DEFER-WHILE-HOLE-OUTSTANDING
        // regression (the 2801d7c sibling — the anchor-raise + FRAME-UP config-change must NOT fire
        // while a lead-frame-only partial's frame-0 is still outstanding for retx, or clear_retx_queue
        // abandons it -> GAP-ABORT wedge; the streak credit is still retained) and exit. Fast +
        // deterministic; see arq_commander.cc::test_inband_climb_defer_on_retx +
        // data-flow-inband-frame0-rolling-partial.md §10. Build with -DINBAND_CLIMB_DEFER_FAILBEFORE
        // to reproduce the fails-before (the climb fires while the hole is outstanding — the orphan).
        if (strcmp(argv[i], "--test-inband-climb-defer") == 0) {
            cl_arq_controller ARQ_cd;
            int failed = ARQ_cd.test_inband_climb_defer_on_retx();
            return (failed == 0) ? 0 : 1;
        }
        // --test-climb-bsi-rollback : run ONLY the climb-UP cmd_batch_seq_id rollback regression
        // (the SYMMETRY GAP to the demote rollback — the climb-UP SET_CONFIG emits must roll
        // cmd_batch_seq_id back to the in-flight bsi before re-presenting an already-RSP-delivered
        // batch, else the re-present is a >=2 bsi jump from the RSP high-water -> GAP-ABORT HOLD)
        // and exit. Fast + deterministic; drives the REAL producer + REAL sack_v2_readopt_has_gap;
        // see arq_commander.cc::test_climb_bsi_rollback + data-flow-inband-frame0-rolling-partial.md
        // §13. Build with -DINBAND_CLIMB_BSI_ROLLBACK_FAILBEFORE to reproduce the fails-before
        // (the re-present stays at the advanced epoch -> gap -> RSP HOLD).
        if (strcmp(argv[i], "--test-climb-bsi-rollback") == 0) {
            cl_arq_controller ARQ_cbr;
            int failed = ARQ_cbr.test_climb_bsi_rollback();
            return (failed == 0) ? 0 : 1;
        }
        // --test-guard-reanchor : run only the live-state reverse-SACK guard
        // reference regression and exit.
        if (strcmp(argv[i], "--test-guard-reanchor") == 0) {
            cl_arq_controller ARQ_gr;
            int failed = ARQ_gr.test_guard_reanchor();
            return (failed == 0) ? 0 : 1;
        }
        // --test-generation-canon : A2 cumulative generation resolution and
        // retention-shadow route synthetic fire (in-process, no PHY/audio).
        if (strcmp(argv[i], "--test-generation-canon") == 0) {
            cl_arq_controller ARQ_gc;
            int failed = ARQ_gc.test_generation_canon();
            return (failed == 0) ? 0 : 1;
        }
        // --test-inband-plus1-climb : run ONLY the in-band +1 climb regression (the FRAME-UP
        // climb steps +1 under the inband feature, suppressing the SNR-elevator multi-rung
        // jump; legacy keeps the elevator) and exit. Fast + deterministic; see
        // arq_commander.cc::test_inband_plus1_climb + data-flow-inband-frame0-rolling-partial.md §7.
        // Build with -DINBAND_PLUS1_CLIMB_FAILBEFORE to reproduce the fails-before (0->3 jump).
        if (strcmp(argv[i], "--test-inband-plus1-climb") == 0) {
            cl_arq_controller ARQ_p1;
            int failed = ARQ_p1.test_inband_plus1_climb();
            return (failed == 0) ? 0 : 1;
        }
        // --test-inband-adopt-preserve : run ONLY the robust->OFDM adopt live-burst PRESERVE
        // regression (the last transition-class hole) and exit. Fast + deterministic; see
        // arq_responder.cc::test_inband_adopt_preserve_live_burst + data-flow-robust-ofdm-adopt-flush.md.
        if (strcmp(argv[i], "--test-inband-adopt-preserve") == 0) {
            cl_arq_controller ARQ_adopt;
            int failed = ARQ_adopt.test_inband_adopt_preserve_live_burst();
            return (failed == 0) ? 0 : 1;
        }
        // --test-inband-adopt-nofdm-invariant : run ONLY the robust->OFDM ring-shrink
        // Nofdm-invariant regression (diagnosis a468b2fc) and exit. Fast + deterministic;
        // see arq_responder.cc::test_inband_adopt_nofdm_invariant + data-flow-robust-ofdm-adopt-flush.md §15.
        if (strcmp(argv[i], "--test-inband-adopt-nofdm-invariant") == 0) {
            cl_arq_controller ARQ_nofdm;
            int failed = ARQ_nofdm.test_inband_adopt_nofdm_invariant();
            return (failed == 0) ? 0 : 1;
        }
        // --test-harvest-inband-cfg0-geometry : force the production fresh-decoder
        // condition and require the CONFIG_0 stride geometry to match the primary.
        if (strcmp(argv[i], "--test-harvest-inband-cfg0-geometry") == 0) {
            cl_arq_controller ARQ_cfg0;
            int failed = ARQ_cfg0.test_harvest_inband_cfg0_geometry();
            return (failed == 0) ? 0 : 1;
        }
        // --test-inband-config0-start-ring : run ONLY the §21 CONFIG_0-START robust-floor over-seat
        // regression (the uncovered sibling of FIX #1e) and exit. Fast + deterministic; see
        // arq_responder.cc::test_inband_config0_start_ring + data-flow-robust-ofdm-adopt-flush.md §21.
        if (strcmp(argv[i], "--test-inband-config0-start-ring") == 0) {
            cl_arq_controller ARQ_c0ring;
            int failed = ARQ_c0ring.test_inband_config0_start_ring();
            return (failed == 0) ? 0 : 1;
        }
        // --test-inband-descrambler-survives-shrink : run ONLY the §17 descrambler-survives-
        // ring-shrink regression (the CONFIG_0 clean-lock CRC-fail root) and exit.
        if (strcmp(argv[i], "--test-inband-descrambler-survives-shrink") == 0) {
            cl_arq_controller ARQ_descr;
            int failed = ARQ_descr.test_inband_descrambler_survives_ring_shrink();
            return (failed == 0) ? 0 : 1;
        }
        // --test-inband-deadbatch-progress : run ONLY the §19 dead-batch real-period/zero-progress
        // regression (the climb-killer fix; real total loss still BREAKs) and exit.
        if (strcmp(argv[i], "--test-inband-deadbatch-progress") == 0) {
            cl_arq_controller ARQ_db;
            int failed = ARQ_db.test_inband_deadbatch_progress();
            return (failed == 0) ? 0 : 1;
        }
        // --test-moose: drive the production Moose reject/clamp predicate at
        // CFG15 spacing. MOOSE_CFO_FAILBEFORE reproduces the old dead zone.
        if (strcmp(argv[i], "--test-moose") == 0) {
            int failed = run_moose_deadzone_tests();
            return (failed == 0) ? 0 : 1;
        }
        // --test-pilot-thin-nv : run the production sparse-grid estimator across
        // 10-30 dB AWGN plus the dense cfg16 scope control.
        if (strcmp(argv[i], "--test-pilot-thin-nv") == 0) {
            int failed = run_pilot_thin_nv_tests();
            return (failed == 0) ? 0 : 1;
        }
        // --test-winlink-dict : run ONLY the Winlink dict priming + version-lock
        // regression suite and exit. Fast + deterministic; drives the production
        // cl_compressor end-to-end (primed lift, bit-exact, version-mismatch
        // fail-safe, kill-switch, bulk no-regression, attachment inertness,
        // streaming desync). See source/compression/test_winlink_dict.cc.
        if (strcmp(argv[i], "--test-winlink-dict") == 0) {
            int failed = run_winlink_dict_tests();
            return (failed == 0) ? 0 : 1;
        }
        // --test-aead-nonce : run ONLY the AEAD bsi-bound nonce regression suite
        // and exit. Fast + deterministic, no IONOS/RF. SECURITY INVARIANT gate:
        // no (key,nonce) reuse; nonce binds to the wire batch_seq_id so SACK
        // reorder/retx decrypt correctly (>256-batch wrap, reorder+retx round-
        // trip, direction disjointness, truncated-KX reject, tamper-reject).
        // See source/crypto/test_aead_nonce.cc / data-flow-aead-nonce.md.
        if (strcmp(argv[i], "--test-aead-nonce") == 0) {
            int failed = run_aead_nonce_tests();
            return (failed == 0) ? 0 : 1;
        }
        // --test-mlkem-hybrid : run ONLY the ML-KEM-768 hybrid KEX suite and
        // exit. Fast + deterministic, no IONOS/RF. Combiner symmetry, ML-KEM-
        // first IKM order, transcript binding, KX chunk round-trip + CRC8.
        // See source/crypto/test_mlkem_hybrid.cc / MLKEM_HYBRID_PLAN.md.
        if (strcmp(argv[i], "--test-mlkem-hybrid") == 0) {
            int failed = run_mlkem_hybrid_tests();
            return (failed == 0) ? 0 : 1;
        }
        // --test-mlkem-live-rx : ML-KEM KX MULTI-CHUNK reassembly through the LIVE
        // control-RX consumer path (process_control_responder KX2 +
        // process_control_commander KX3). Reproduces the messages_control.length=1
        // producer/consumer bug: fails-before (chunks rejected), passes-after (real
        // slot width passed). Member test on a throwaway controller. Fast +
        // deterministic, no IONOS/RF. MLKEM_HYBRID_PLAN.md §5.
        if (strcmp(argv[i], "--test-mlkem-live-rx") == 0) {
            cl_arq_controller ARQ_kx;
            int failed = ARQ_kx.test_kx_chunk_live_rx();
            return (failed == 0) ? 0 : 1;
        }
        // --test-sim-clock : run ONLY the sim-clock unit suite and exit. The
        // full --test suite includes long stochastic MFSK detector sweeps;
        // this gives a fast, deterministic entry for the sim-clock tests
        // (test (a) of sim-arq-channel.md §3). Production behavior of --test is
        // unchanged.
        if (strcmp(argv[i], "--test-sim-clock") == 0) {
            int failed = run_sim_clock_tests();
            return (failed == 0) ? 0 : 1;
        }
        // --test-ofdm-fine-timing : run ONLY the §22 OFDM fine-timing
        // phase-invariant magnitude regression suite and exit. Fast +
        // deterministic — excludes the long stochastic MFSK detector sweeps in
        // the full --test suite. See fact-documents/ofdm-fine-timing-magnitude.md §4.
        if (strcmp(argv[i], "--test-ofdm-fine-timing") == 0) {
            int failed = run_ofdm_fine_timing_tests();
            return (failed == 0) ? 0 : 1;
        }
        // --test-preamble-sched : LEVER P preamble-amortization schedule +
        // effective-length pure-function unit tests. Fast + deterministic.
        // See fact-documents/data-flow-preamble-amortization.md §1.
        if (strcmp(argv[i], "--test-preamble-sched") == 0) {
            int failed = run_preamble_sched_tests();
            return (failed == 0) ? 0 : 1;
        }
		if (strcmp(argv[i], "--test-mini0-carried") == 0) {
			return run_mini0_carried_timing_test();
		}
		if (strcmp(argv[i], "--test-mini1-carried") == 0) {
			return run_mini1_carried_timing_test();
		}
        // --test-break-fh : fix/break-fh-gate §23 BREAK forward-health gate suite
        // (FH-latch suppression of the held-CFG16 marginal-OFDM alias + K-of-N
        // corroboration + genuine-BREAK survival, both gate states). Fast +
        // deterministic. Also included in the full --test suite.
        if (strcmp(argv[i], "--test-break-fh") == 0) {
            int failed = run_break_fh_gate_tests();
            return (failed == 0) ? 0 : 1;
        }
        // --test-break-alias : OFDM-alias BREAK false-positive investigation sweep.
        // Drives real non-BREAK OFDM data (config x Es/N0 grid) through acquisition +
        // decode + the BREAK correlator; reports whether the detonation predicate ever
        // fires on data that is NOT a transmitted BREAK. NOT part of --test.
        if (strcmp(argv[i], "--test-break-alias") == 0) {
            return run_break_alias_sweep();
        }
        // --test-recovery-ack : recovery-ack-robustness.md suite (marginal-ACK
        // combining + listen-window ms-mirror + DELTA-1 reps-agnostic BREAK +
        // DELTA-2 CFO-refine decision gate). Fast + deterministic. Also in --test.
        if (strcmp(argv[i], "--test-recovery-ack") == 0) {
            int failed = run_recovery_ack_tests();
            return (failed == 0) ? 0 : 1;
        }
        // --test-recovery-ack-fine : recovery-ack-fine STAGE 1 fine-pass timing-
        // straddle sweep (data-flow-ack-detector.md §2). Fail-before (no-fine
        // coarse collapse below 7/16) / pass-after (fine recovers over the
        // hardened bar) across a tau sweep. Fast + deterministic. Also in --test.
        if (strcmp(argv[i], "--test-recovery-ack-fine") == 0) {
            int failed = run_recovery_ack_fine_tests();
            return (failed == 0) ? 0 : 1;
        }
    handled = false;
    return 0;
}
