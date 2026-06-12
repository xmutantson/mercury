/*
 * Mercury: SE-reclaim grid-selector regression suite (Stages 1-2).
 *
 * CLI: --test-se-reclaim
 *
 * Paired with mercury/fact-documents/data-flow-se-reclaim.md. Drives the REAL
 * production CRC16+LDPC OFDM wire (transmit_byte -> receive_byte) and the REAL
 * load_configuration grid materializer — no IONOS, no RF, no ARQ socket plumbing.
 *
 * STAGE 1 (wire-framing): a SET_CONFIG-shaped CONTROL frame carrying the
 * per-direction grid selector at data[3]/data[4] round-trips RX==TX through a
 * clean CONFIG_15 codeword; a legacy 3-byte SET_CONFIG (no selector bytes)
 * decodes the selector positions as 0 = GRID_FULL/GRID_FULL (back-compat).
 *   FAIL-BEFORE: before the producer/consumer write data[3]/data[4], a length-5
 *   round-trip can carry the selector but the modem ignores it. The test asserts
 *   the bytes survive the wire AND that the enum/back-compat contract holds; the
 *   genuine fail-before anchor is the Stage-2 materializer (a RECLAIM selector
 *   that does NOT change the grid -> rbc unchanged -> assertion fails).
 *
 * STAGE 2 (grid materializer): load_configuration(CONFIG_15) with the RECLAIM
 * grid pending yields the exact (Ngi,Dy,Nsymb,nData,rbc) from the verdict; FULL
 * yields the stock grid byte-identically. (Filled in Stage 2.)
 *
 * Returns 0 on PASS (all cases), 1 on FAIL.
 */

#include "physical_layer/telecom_system.h"
#include "physical_layer/physical_defines.h"
#include "common/common_defines.h"
#include "datalink_layer/datalink_defines.h"
#include "datalink_layer/se_reclaim_gate.h"   // Stage 3 — the forward-link gate

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

static int se_failures = 0;
static int se_passes   = 0;

static void se_pass(const char* name) { printf("  [OK]   %s\n", name); se_passes++; }
static void se_fail(const char* name, const char* reason) {
	printf("  [FAIL] %s: %s\n", name, reason); se_failures++;
}

// =============================================================================
// PHY round-trip helper (clean, high SNR). Builds a SET_CONFIG-shaped CONTROL
// frame in data_byte[], transmits one CONFIG_15 OFDM codeword, places it at a
// known delay in an RX-sized buffer with light AWGN, and runs the production
// receive_byte acquisition. Decoded payload comes back in `out`.
//
// Wire layout of a CONTROL frame (arq_common.cc:4547-4558):
//   data_byte[0] = type (CONTROL)
//   data_byte[1] = connection_id
//   data_byte[2] = sequence_number
//   data_byte[3..] = message payload data[0..length)
// So the SET_CONFIG payload data[k] lands at data_byte[3+k]:
//   data_byte[3]=SET_CONFIG, [4]=fwd_cfg, [5]=rev_cfg, [6]=fwd_grid, [7]=rev_grid
// transmit_byte zero-pads the rest of the frame (telecom_system.cc:770-777), so a
// legacy 3-byte payload (nBytes=6) leaves data_byte[6]/[7] = 0 on the wire.
// =============================================================================
static bool se_setconfig_roundtrip(int payload_len /*3 or 5*/,
                                    int fwd_cfg, int rev_cfg,
                                    int fwd_grid, int rev_grid,
                                    int* out /*decoded bytes*/, int out_cap,
                                    const char* name)
{
	srand(12345);
	cl_telecom_system ts;
	ts.operation_mode = ARQ_MODE;
	ts.load_configuration(CONFIG_15);   // WB 16-QAM r0.875, the reliable rung
	if (ts.current_configuration != CONFIG_15) {
		se_fail(name, "load_configuration(CONFIG_15) did not take");
		return false;
	}

	int interp     = ts.frequency_interpolation_rate;
	int Nofdm      = ts.data_container.Nofdm;
	int preamble_n = ts.data_container.preamble_nSymb;
	int Nsymb      = ts.data_container.Nsymb;
	int buffer_N   = ts.data_container.buffer_Nsymb;
	int sym_samples = Nofdm * interp;

	// Build the CONTROL frame bytes: 3-byte header + SET_CONFIG payload.
	const int HDR = CONTROL_ACK_CONTROL_HEADER_LENGTH;   // 3
	int nBytes = HDR + payload_len;
	for (int i = 0; i < ts.data_container.nBits / 8 && i < 256; i++)
		ts.data_container.data_byte[i] = 0;
	ts.data_container.data_byte[0] = CONTROL;
	ts.data_container.data_byte[1] = 0x55;        // synthetic connection_id
	ts.data_container.data_byte[2] = 0;           // sequence_number
	ts.data_container.data_byte[HDR + 0] = SET_CONFIG;
	ts.data_container.data_byte[HDR + 1] = fwd_cfg & 0xFF;
	ts.data_container.data_byte[HDR + 2] = rev_cfg & 0xFF;
	if (payload_len >= 5) {
		ts.data_container.data_byte[HDR + 3] = fwd_grid & 0xFF;
		ts.data_container.data_byte[HDR + 4] = rev_grid & 0xFF;
	}
	// NOTE: for payload_len==3 we deliberately do NOT write [HDR+3]/[HDR+4];
	// transmit_byte will zero-pad them on the wire (the legacy back-compat path).

	ts.transmit_byte(ts.data_container.data_byte, nBytes,
		ts.data_container.passband_data, SINGLE_MESSAGE);

	int frame_samples = Nofdm * (Nsymb + preamble_n) * interp;
	int rx_samples = Nofdm * buffer_N * interp;
	std::vector<double> rx((size_t)rx_samples, 0.0);

	int delay = (preamble_n + 4) * sym_samples;
	if (delay + frame_samples > rx_samples) delay = rx_samples - frame_samples;
	if (delay < 0) delay = 0;

	// Calibrate light AWGN to a clean SNR3k so the codeword decodes reliably
	// (this is a wire-framing test, not a waterfall test).
	double P_sig = 0.0;
	for (int i = 0; i < frame_samples; i++)
		P_sig += ts.data_container.passband_data[i] * ts.data_container.passband_data[i];
	P_sig /= frame_samples;
	double target_snr3k_db = 30.0;
	double f_nyquist = ts.sampling_frequency / 2.0;
	double sigma = sqrt(2.0 * P_sig * f_nyquist /
		(pow(10.0, target_snr3k_db / 10.0) * ts.bandwidth));
	double ampl_val = sigma / sqrt(2.0);

	for (int i = 0; i < frame_samples; i++)
		rx[(size_t)(delay + i)] = ts.data_container.passband_data[i];
	for (int i = 0; i < rx_samples; i++)
		rx[(size_t)i] += ampl_val * ts.awgn_channel.awgn_value_generator();

	ts.ofdm_forced_delay = -1;
	st_receive_stats st = ts.receive_byte(rx.data(), ts.data_container.hd_decoded_data_byte);

	// receive_stats.crc holds the CRC16-MODBUS remainder; 0 == CRC OK
	// (telecom_system.cc:3132-3136: non-zero => decode failure).
	if (st.crc != 0) {
		char buf[96];
		snprintf(buf, sizeof(buf), "codeword CRC not OK (crc=0x%04X mean_H=%.3f)",
			st.crc, st.mean_H);
		se_fail(name, buf);
		return false;
	}

	int n = ts.data_container.nBits / 8;
	if (n > out_cap) n = out_cap;
	for (int i = 0; i < n; i++)
		out[i] = ts.data_container.hd_decoded_data_byte[i];
	return true;
}

// =============================================================================
// STAGE 1.1 — length-5 SET_CONFIG round-trips the grid selector RX==TX.
// =============================================================================
static void test_se_grid_wire_roundtrip()
{
	const char* name = "se_grid_wire_roundtrip_len5";
	int out[256];
	const int HDR = CONTROL_ACK_CONTROL_HEADER_LENGTH;
	// forward=15, reverse=15, fwd_grid=RECLAIM, rev_grid=FULL
	if (!se_setconfig_roundtrip(5, CONFIG_15, CONFIG_15, GRID_RECLAIM, GRID_FULL,
	                            out, 256, name))
		return;

	int dec_fwd  = out[HDR + 1] & 0xFF;
	int dec_rev  = out[HDR + 2] & 0xFF;
	int dec_fgrid = out[HDR + 3] & 0xFF;
	int dec_rgrid = out[HDR + 4] & 0xFF;

	if (out[HDR + 0] != SET_CONFIG) { se_fail(name, "decoded code != SET_CONFIG"); return; }
	if (dec_fwd != CONFIG_15 || dec_rev != CONFIG_15) {
		se_fail(name, "decoded cfg pair mismatch"); return;
	}
	if (dec_fgrid != GRID_RECLAIM) { se_fail(name, "decoded forward_grid != GRID_RECLAIM"); return; }
	if (dec_rgrid != GRID_FULL)    { se_fail(name, "decoded reverse_grid != GRID_FULL"); return; }
	if (!is_valid_grid(dec_fgrid) || !is_valid_grid(dec_rgrid)) {
		se_fail(name, "decoded grid not a valid enum"); return;
	}
	se_pass(name);
}

// =============================================================================
// STAGE 1.2 — legacy 3-byte SET_CONFIG decodes the selector positions as
// GRID_FULL/GRID_FULL (transmit_byte zero-pads the tail). Back-compat (INV-5).
// =============================================================================
static void test_se_grid_wire_legacy_full()
{
	const char* name = "se_grid_wire_legacy_len3_is_full";
	int out[256];
	const int HDR = CONTROL_ACK_CONTROL_HEADER_LENGTH;
	// A legacy peer: 3-byte payload, no selector bytes written by us.
	if (!se_setconfig_roundtrip(3, CONFIG_15, CONFIG_15, /*ignored*/0, /*ignored*/0,
	                            out, 256, name))
		return;

	int dec_fgrid = out[HDR + 3] & 0xFF;
	int dec_rgrid = out[HDR + 4] & 0xFF;
	if (dec_fgrid != GRID_FULL) { se_fail(name, "legacy forward_grid pos != 0/FULL"); return; }
	if (dec_rgrid != GRID_FULL) { se_fail(name, "legacy reverse_grid pos != 0/FULL"); return; }
	se_pass(name);
}

// Forward decl of the Stage-2 materializer tests (test_se_reclaim_grid.cc).
extern int run_se_reclaim_grid_materializer_tests();

// =============================================================================
// STAGE 3 — the conservative forward-link gate (data-flow-se-reclaim.md §3).
// Pure state-machine unit tests: default-FULL on startup, slow-promote after
// CONFIRM_N clean ticks, INSTANT demote on a fade/FER spike, decode-independent
// demote on a no-progress tick. The life-critical safety assertion.
// =============================================================================
static void test_se_gate_default_and_promote_demote()
{
	const char* name = "se_gate_default_promote_demote";
	cl_se_reclaim_gate gate;
	gate.SEL_RECLAIM_MAX = 0.05;
	gate.SNR_RECLAIM_MIN_DB = 12.0;
	gate.CONFIRM_N = 10;

	// (1) startup default = FULL.
	if(gate.grid() != GRID_FULL) { se_fail(name, "startup grid != GRID_FULL"); return; }

	// (2) feed CONFIRM_N-1 clean ticks: must STILL be FULL (slow up).
	for(int i = 0; i < gate.CONFIRM_N - 1; i++)
		gate.update(0.02 /*clean sel*/, 20.0 /*good snr*/, 0 /*no fer*/);
	if(gate.grid() != GRID_FULL) {
		se_fail(name, "promoted before CONFIRM_N clean ticks (too eager)"); return;
	}

	// (3) the CONFIRM_N-th clean tick promotes to RECLAIM.
	gate.update(0.02, 20.0, 0);
	if(gate.grid() != GRID_RECLAIM) {
		se_fail(name, "did not promote after CONFIRM_N clean ticks"); return;
	}

	// (4) a SINGLE fade onset (selectivity spike above the knee) => INSTANT demote.
	gate.update(0.20 /*fade: well above 0.05 and the 0.15 knee*/, 20.0, 0);
	if(gate.grid() != GRID_FULL) {
		se_fail(name, "did not INSTANT-demote on a selectivity spike (safety)"); return;
	}

	// (5) re-promotion must again take the FULL CONFIRM_N (streak reset on demote).
	for(int i = 0; i < gate.CONFIRM_N - 1; i++)
		gate.update(0.02, 20.0, 0);
	if(gate.grid() != GRID_FULL) {
		se_fail(name, "re-promoted before a fresh CONFIRM_N after demote"); return;
	}
	gate.update(0.02, 20.0, 0);
	if(gate.grid() != GRID_RECLAIM) { se_fail(name, "did not re-promote"); return; }

	// (6) a forward FER>0 (decode loss) => INSTANT demote even with clean sel/snr.
	gate.update(0.02, 20.0, 1 /*one frame error*/);
	if(gate.grid() != GRID_FULL) {
		se_fail(name, "did not demote on forward FER>0 (safety)"); return;
	}

	se_pass(name);
}

static void test_se_gate_no_progress_failsafe()
{
	const char* name = "se_gate_no_progress_failsafe";
	cl_se_reclaim_gate gate;
	gate.CONFIRM_N = 5;
	// Promote.
	for(int i = 0; i < gate.CONFIRM_N; i++) gate.update(0.01, 25.0, 0);
	if(gate.grid() != GRID_RECLAIM) { se_fail(name, "precondition: not promoted"); return; }
	// A no-progress / silent-link tick (mislabeled fade kills decode -> no
	// measurement returns) MUST demote without depending on a clean decode.
	gate.no_progress_tick();
	if(gate.grid() != GRID_FULL) {
		se_fail(name, "no_progress_tick did not fail-safe to FULL"); return;
	}
	se_pass(name);
}

static void test_se_gate_sentinel_and_margin()
{
	const char* name = "se_gate_sentinel_and_margin";
	cl_se_reclaim_gate gate;
	gate.SEL_RECLAIM_MAX = 0.05;
	gate.SNR_RECLAIM_MIN_DB = 12.0;
	gate.CONFIRM_N = 3;

	// A selectivity SENTINEL (-1.0, no measurement) is NOT clean (fail-safe).
	if(gate.is_clean(-1.0, 30.0, 0)) { se_fail(name, "sentinel selectivity treated as clean"); return; }
	// Selectivity above the deep-margin threshold (but below the 0.15 knee) is NOT clean.
	if(gate.is_clean(0.10, 30.0, 0)) { se_fail(name, "sel=0.10 (>0.05 margin) treated as clean"); return; }
	// SNR below the floor is NOT clean even with clean selectivity.
	if(gate.is_clean(0.01, 8.0, 0)) { se_fail(name, "low SNR treated as clean"); return; }
	// A clean, high-margin measurement IS clean.
	if(!gate.is_clean(0.01, 30.0, 0)) { se_fail(name, "deep-clean measurement not recognized"); return; }
	// Never promotes past RECLAIM (idempotent at the top).
	for(int i = 0; i < 20; i++) gate.update(0.01, 30.0, 0);
	if(gate.grid() != GRID_RECLAIM) { se_fail(name, "did not reach/hold RECLAIM under sustained clean"); return; }
	se_pass(name);
}

// =============================================================================
// STAGE 4 — sync / AGC decoupling VERIFICATION (data-flow-se-reclaim.md §7 H8,
// INV-9). Measures Schmidl-Cox lock-rate + first-frame mean_H on the Ngi=18
// whole-frame short-GI grid vs the stock Ngi=54 grid at the clean operating SNR.
// INV-9 risk: the TX FIR is 97 taps but the Ngi=18 GI-copy budget is 72, so the
// short-GI preamble could weaken Schmidl-Cox. If Ngi=18 regresses, the SHIPPABLE
// target stays RECLAIM-PILOTS (Ngi=54, zero sync risk) and RECLAIM-FULL is held.
// This test MEASURES + reports honestly; it ASSERTS only the non-regression of
// the SHIPPABLE Ngi=54 path (so the suite stays green regardless of the Ngi=18
// finding, which is reported as data for the ship decision).
// =============================================================================
static void se_setenv_local(const char* k, const char* v) {
#if defined(_WIN32)
	_putenv_s(k, v);
#else
	setenv(k, v, 1);
#endif
}
static void se_unsetenv_local(const char* k) {
#if defined(_WIN32)
	_putenv_s(k, "");
#else
	unsetenv(k);
#endif
}

// Run one clean CONFIG_15 OFDM frame at the env-forced grid; return lock (crc OK)
// and the production mean_H. Mirrors the §22 ofdm_ftr_roundtrip channel setup.
static bool se_sync_frame(int ngi, int dy, int nsymb, double snr3k_db,
                          unsigned int seed, bool& out_lock, double& out_mean_H)
{
	char b[16];
	snprintf(b, sizeof(b), "%d", ngi);  se_setenv_local("MERCURY_SE_NGI", b);
	snprintf(b, sizeof(b), "%d", dy);   se_setenv_local("MERCURY_SE_DY", b);
	snprintf(b, sizeof(b), "%d", nsymb);se_setenv_local("MERCURY_SE_NSYMB", b);

	srand(seed);
	cl_telecom_system ts;
	ts.operation_mode = ARQ_MODE;
	ts.load_configuration(CONFIG_15);

	se_unsetenv_local("MERCURY_SE_NGI");
	se_unsetenv_local("MERCURY_SE_DY");
	se_unsetenv_local("MERCURY_SE_NSYMB");

	if (ts.current_configuration != CONFIG_15) return false;

	int interp     = ts.frequency_interpolation_rate;
	int Nofdm      = ts.data_container.Nofdm;
	int preamble_n = ts.data_container.preamble_nSymb;
	int Nsymb      = ts.data_container.Nsymb;
	int buffer_N   = ts.data_container.buffer_Nsymb;
	int sym_samples = Nofdm * interp;

	int nReal = ts.data_container.nBits - ts.ldpc.P;
	int frame_bytes = (nReal - ts.outer_code_reserved_bits) / 8;
	for (int i = 0; i < frame_bytes; i++)
		ts.data_container.data_byte[i] = (i * 37 + 11) & 0xFF;
	ts.transmit_byte(ts.data_container.data_byte, frame_bytes,
		ts.data_container.passband_data, SINGLE_MESSAGE);

	int frame_samples = Nofdm * (Nsymb + preamble_n) * interp;
	int rx_samples = Nofdm * buffer_N * interp;
	std::vector<double> rx((size_t)rx_samples, 0.0);
	int delay = (preamble_n + 4) * sym_samples;
	if (delay + frame_samples > rx_samples) delay = rx_samples - frame_samples;
	if (delay < 0) delay = 0;

	double P_sig = 0.0;
	for (int i = 0; i < frame_samples; i++)
		P_sig += ts.data_container.passband_data[i] * ts.data_container.passband_data[i];
	P_sig /= frame_samples;
	double f_nyquist = ts.sampling_frequency / 2.0;
	double sigma = sqrt(2.0 * P_sig * f_nyquist /
		(pow(10.0, snr3k_db / 10.0) * ts.bandwidth));
	double ampl_val = sigma / sqrt(2.0);
	for (int i = 0; i < frame_samples; i++)
		rx[(size_t)(delay + i)] = ts.data_container.passband_data[i];
	for (int i = 0; i < rx_samples; i++)
		rx[(size_t)i] += ampl_val * ts.awgn_channel.awgn_value_generator();

	ts.ofdm_forced_delay = -1;
	st_receive_stats st = ts.receive_byte(rx.data(), ts.data_container.hd_decoded_data_byte);
	out_lock   = (st.crc == 0);       // CRC OK == frame locked + decoded
	out_mean_H = st.mean_H;
	return true;
}

static void se_sync_measure(int ngi, int dy, int nsymb, double snr3k_db, int nframes,
                            const char* label, int& out_locks, double& out_meanH_avg)
{
	int locks = 0;
	double mh_sum = 0.0; int mh_n = 0;
	for (int s = 0; s < nframes; s++) {
		bool lock = false; double mh = -1.0;
		if (se_sync_frame(ngi, dy, nsymb, snr3k_db, 1000u + (unsigned)s, lock, mh)) {
			if (lock) locks++;
			if (mh > 0) { mh_sum += mh; mh_n++; }
		}
	}
	out_locks = locks;
	out_meanH_avg = (mh_n > 0) ? (mh_sum / mh_n) : -1.0;
	printf("    [SE-SYNC] %-16s Ngi=%2d Dy=%d Nsymb=%2d snr3k=%.1f: lock=%d/%d mean_H=%.3f\n",
		label, ngi, dy, nsymb, snr3k_db, locks, nframes, out_meanH_avg);
}

static void test_se_sync_margin()
{
	const char* name = "se_sync_margin_ngi18_vs_ngi54";
	const int N = 12;
	const double SNR = 20.0;   // clean operating front

	int locks_full = 0, locks_pilots = 0, locks_reclaim = 0;
	double mh_full = 0, mh_pilots = 0, mh_reclaim = 0;
	// FULL (stock 54/3/12) and RECLAIM-PILOTS (54/5/10) keep Ngi=54 => zero sync risk.
	se_sync_measure(54, 3, 12, SNR, N, "FULL",           locks_full,    mh_full);
	se_sync_measure(54, 5, 10, SNR, N, "RECLAIM-PILOTS", locks_pilots,  mh_pilots);
	// RECLAIM-FULL (18/5/10) is the reduced-CP, INV-9-risk grid.
	se_sync_measure(18, 5, 10, SNR, N, "RECLAIM-FULL",   locks_reclaim, mh_reclaim);

	// Honest report for the ship decision (INV-9).
	printf("    [SE-SYNC] VERDICT: ngi54 lock=%d/%d, ngi18 lock=%d/%d (acceptance: ngi18 >= ngi54 - 1)\n",
		locks_full, N, locks_reclaim, N);
	bool ngi18_ok = (locks_reclaim >= locks_full - 1) &&
	                (mh_reclaim > 0.7 * mh_full);
	printf("    [SE-SYNC] RECLAIM-FULL (Ngi=18) sync %s the acceptance band => %s\n",
		ngi18_ok ? "WITHIN" : "OUTSIDE",
		ngi18_ok ? "RECLAIM-FULL sync-safe (still bench-gate before ship)"
		         : "SHIP PILOTS-ONLY; hold RECLAIM-FULL pending per-region GI");

	// ASSERT only the SHIPPABLE path's non-regression: RECLAIM-PILOTS (Ngi=54)
	// must lock as well as FULL at the clean SNR (it shares the full CP/preamble).
	// The Ngi=18 finding is reported (not asserted) so the suite stays green and
	// the ship decision uses real data.
	if (locks_pilots < locks_full - 1) {
		char buf[128];
		snprintf(buf, sizeof(buf),
			"RECLAIM-PILOTS regressed sync vs FULL (pilots %d/%d < full %d/%d)",
			locks_pilots, N, locks_full, N);
		se_fail(name, buf);
		return;
	}
	se_pass(name);
}


// =============================================================================
// Suite entry. Wired via main.cc --test-se-reclaim.
// =============================================================================
int run_se_reclaim_tests()
{
	se_failures = 0;
	se_passes   = 0;
	printf("=== SE-reclaim grid-selector tests ===\n");

	// Stage 1 — wire-framing
	test_se_grid_wire_roundtrip();
	test_se_grid_wire_legacy_full();

	// Stage 2 — grid materializer
	int mat_failures = run_se_reclaim_grid_materializer_tests();
	se_failures += mat_failures;

	// Stage 3 — the conservative forward-link gate (safety)
	test_se_gate_default_and_promote_demote();
	test_se_gate_no_progress_failsafe();
	test_se_gate_sentinel_and_margin();

	// Stage 4 — sync/AGC decoupling verification (Ngi=18 vs Ngi=54 sync margin)
	test_se_sync_margin();

	printf("=== SE-reclaim: %d passed, %d failed (materializer %d failed) ===\n",
		se_passes, se_failures, mat_failures);
	return (se_failures == 0) ? 0 : 1;
}
