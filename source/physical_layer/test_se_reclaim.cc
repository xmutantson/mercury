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

// Forward decl of the Stage-2 materializer tests (defined in test_se_reclaim_grid.cc-style
// section below once Stage 2 lands). Stubs return 0 here at Stage 1.
extern int run_se_reclaim_grid_materializer_tests();

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

	printf("=== SE-reclaim: %d passed, %d failed (materializer %d failed) ===\n",
		se_passes, se_failures, mat_failures);
	return (se_failures == 0) ? 0 : 1;
}
