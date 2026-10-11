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
