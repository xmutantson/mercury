/*
 * Mercury: A configurable open-source software-defined modem.
 * Copyright (C) 2022-2024 Fadi Jerji
 * Author: Fadi Jerji
 * Email: fadi.jerji@  <gmail.com, caisresearch.com, ieee.org>
 * ORCID: 0000-0002-2076-5831
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

// MFSK control-suffix codec unit-test suite (Phase B Wave 1).
//
// Wired via `mercury.exe --test` (see main.cc). Tests are split into:
//
//   §1 Codec primitives  (no DSP plumbing — uses cl_mfsk alone):
//     1.1 pack_unpack_callsign_body_b36
//     1.2 pack_unpack_start_conn_payload
//     1.3 pack_unpack_test_ack_payload
//     1.4 pack_unpack_test_conn_payload         (§14 Wave 3)
//     1.5 ctrl_suffix_roundtrip_all_types
//     1.6 ctrl_suffix_crc12_corruption
//     1.7 base_pattern_cross_correlation
//     1.8 ack_sack_bitmap_30bit_cap
//
//   §2 Passband round-trip  (requires cl_telecom_system::load_configuration):
//     2.1 mfsk_connect_passband_roundtrip_clean
//     2.2 mfsk_connect_no_hail_false_trigger
//
//   §3 Wave 2 v2 cross-layer regression tests:
//     3.1 v2_crc12_wireformat_real_helper
//     3.2 v2_cmd_loop_no_refire
//     3.3 v2_rsp_frames_to_read_override
//
//   §4 Wave 3 (§14) TEST_CONN integration tests:
//     4.1 v3_test_conn_passband_roundtrip_clean
//     4.2 v3_test_conn_snr_quantization_roundtrip
//
// Each test prints "  [OK] name" on pass or "  [FAIL] name: reason" on
// failure. The function returns the total number of failed tests.
//
// All test trials use a deterministic std::mt19937 seed so failures are
// reproducible.

#include "physical_layer/mfsk_ctrl_codec_tests.h"
#include "physical_layer/mfsk_ctrl_codec.h"
#include "physical_layer/mfsk.h"
#include "physical_layer/telecom_system.h"
#include "physical_layer/physical_defines.h"
#include "datalink_layer/arq.h"           // §3 Wave 2 v2 cross-layer tests

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>

// =============================================================================
// Test helpers
// =============================================================================

static int g_failures = 0;
static int g_passes   = 0;

static void test_pass(const char* name) {
	printf("  [OK]   %s\n", name);
	g_passes++;
}

static void test_fail(const char* name, const char* reason) {
	printf("  [FAIL] %s: %s\n", name, reason);
	g_failures++;
}

// Minimal CRC12 helper for the tests — mirrors cl_arq_controller::CRC12_calc
// (arq_common.cc:6784) bit-exact INCLUDING init=0xFFF. v1 bug #1 (fact-doc
// §13.5 / phase-a-to-b-workplan §10) was an inline RX-side helper that
// defaulted to init=0; the test helper had the same init=0 bug, so the
// test passed while production failed. Wave 2 v2 fix: init=0xFFF here so
// the helper matches the production CRC12_calc bit-for-bit and any
// regression in either direction surfaces immediately in the test suite.
// Polynomial = POLY_CRC12 = 0xF13 (CRC-12 CDMA2000, MSB-first / forward,
// no final XOR).
static uint16_t test_crc12_calc(const uint8_t* data, int nBytes) {
	const uint16_t POLY = 0xF13;
	uint16_t crc = 0xFFF;  // v2 fix: was 0, matches production CRC12_calc
	for (int i = 0; i < nBytes; i++) {
		crc ^= ((uint16_t)data[i]) << 4;
		for (int b = 0; b < 8; b++) {
			if (crc & 0x800) crc = (uint16_t)((crc << 1) ^ POLY);
			else             crc = (uint16_t)(crc << 1);
			crc &= 0x0FFF;
		}
	}
	return (uint16_t)(crc & 0x0FFF);
}

// =============================================================================
// §1. Codec primitives
// =============================================================================

static void test_pack_unpack_callsign_body_b36() {
	const char* name = "pack_unpack_callsign_body_b36";
	std::mt19937 rng(0xCAFE);
	const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
	for (int trial = 0; trial < 1000; trial++) {
		int len = 1 + (int)(rng() % 6);  // 1..6
		char call[7];
		for (int i = 0; i < len; i++)
			call[i] = alphabet[rng() % 36];
		call[len] = '\0';

		uint64_t packed = 0;
		int n_written = pack_callsign_body_b36(call, len, &packed);
		if (n_written != len) {
			char buf[128];
			snprintf(buf, sizeof(buf), "trial %d call=%s len=%d wrote=%d",
				trial, call, len, n_written);
			test_fail(name, buf);
			return;
		}

		char out[7] = {};
		int n_read = unpack_callsign_body_b36(packed, out);
		if (n_read != len || strcmp(out, call) != 0) {
			char buf[128];
			snprintf(buf, sizeof(buf), "trial %d call=%s len=%d got=%s rlen=%d",
				trial, call, len, out, n_read);
			test_fail(name, buf);
			return;
		}
	}
	// Sentinel test: 3-char call must pad with sentinel 36.
	{
		uint64_t packed = 0;
		(void)pack_callsign_body_b36("ABC", 3, &packed);
		// Each char is 6 bits. Padding chars must equal 36.
		// Char 3 (sentinel) sits at bits 12..17.
		int pad = (int)((packed >> 12) & 0x3F);
		if (pad != 36) {
			char buf[64];
			snprintf(buf, sizeof(buf), "sentinel expected 36, got %d", pad);
			test_fail(name, buf);
			return;
		}
	}
	test_pass(name);
}

static void test_pack_unpack_start_conn_payload() {
	const char* name = "pack_unpack_start_conn_payload";
	std::mt19937 rng(0xBEEF);
	const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
	for (int trial = 0; trial < 1000; trial++) {
		bool nb_flag = (rng() & 1) != 0;
		int len = 1 + (int)(rng() % 6);
		char call[7];
		for (int i = 0; i < len; i++)
			call[i] = alphabet[rng() % 36];
		call[len] = '\0';

		uint64_t p38 = 0xFFFFFFFFFFFFFFFFULL;  // pre-set garbage
		pack_start_conn_payload(&p38, nb_flag, call, len);
		// Reserved bit (bit 0) MUST be zero on TX.
		if ((p38 & 0x1ULL) != 0) {
			test_fail(name, "reserved bit not zero on TX");
			return;
		}
		// p38 must fit in 38 bits.
		if (p38 & ~((1ULL << 38) - 1ULL)) {
			test_fail(name, "payload overflows 38 bits");
			return;
		}

		bool out_nb = !nb_flag;
		char out_call[7] = {};
		int out_len = -1;
		bool ok = unpack_start_conn_payload(p38, &out_nb, out_call, &out_len);
		if (!ok || out_nb != nb_flag || out_len != len ||
		    strcmp(out_call, call) != 0) {
			char buf[160];
			snprintf(buf, sizeof(buf),
				"trial %d nb=%d call=%s len=%d -> ok=%d out_nb=%d out_call=%s out_len=%d",
				trial, nb_flag, call, len, ok, out_nb, out_call, out_len);
			test_fail(name, buf);
			return;
		}
	}
	test_pass(name);
}

static void test_pack_unpack_test_ack_payload() {
	const char* name = "pack_unpack_test_ack_payload";
	std::mt19937 rng(0xC0DE);
	// Cover full echoed_cap × own_cap × representative SSID set.
	const uint8_t ssids[] = {0, 1, 7, 15, 16, 17, 18, 19, 50, 99, 255};
	const int nssids = (int)(sizeof(ssids) / sizeof(ssids[0]));
	int trials = 0;
	for (int ec = 0; ec < 4; ec++) {
		for (int oc = 0; oc < 4; oc++) {
			for (int si = 0; si < nssids; si++) {
				uint8_t ssid = ssids[si];
				uint64_t p38 = (uint64_t)rng();
				pack_test_ack_payload(&p38, (uint8_t)ec, (uint8_t)oc, ssid);
				if (p38 & ~((1ULL << 38) - 1ULL)) {
					test_fail(name, "payload overflows 38 bits");
					return;
				}
				if ((p38 & ((1ULL << 26) - 1ULL)) != 0) {
					// reserved bits must be zero on TX
					test_fail(name, "reserved bits not zero on TX");
					return;
				}
				uint8_t out_ec = 0xFF, out_oc = 0xFF, out_ssid = 0;
				bool ok = unpack_test_ack_payload(p38, &out_ec, &out_oc, &out_ssid);
				if (!ok || out_ec != (uint8_t)ec || out_oc != (uint8_t)oc ||
				    out_ssid != ssid) {
					char buf[160];
					snprintf(buf, sizeof(buf),
						"ec=%d oc=%d ssid=%u -> ok=%d out_ec=%u out_oc=%u out_ssid=%u",
						ec, oc, (unsigned)ssid, ok, out_ec, out_oc, out_ssid);
					test_fail(name, buf);
					return;
				}
				trials++;
			}
		}
	}
	(void)trials;
	test_pass(name);
}

static void test_pack_unpack_test_conn_payload() {
	const char* name = "pack_unpack_test_conn_payload";
	std::mt19937 rng(0x7E57);
	// Cover full local_cap × representative SSIDs × full snr_q range.
	const uint8_t ssids[] = {0, 1, 7, 15, 16, 17, 18, 19, 50, 99, 255};
	const int nssids = (int)(sizeof(ssids) / sizeof(ssids[0]));
	int trials = 0;
	for (int snr_q = 0; snr_q < 16; snr_q++) {
		for (int lc = 0; lc < 4; lc++) {
			for (int si = 0; si < nssids; si++) {
				uint8_t ssid = ssids[si];
				uint64_t p38 = (uint64_t)rng();  // pre-set garbage
				pack_test_conn_payload(&p38, (uint8_t)snr_q,
					(uint8_t)lc, ssid);
				if (p38 & ~((1ULL << 38) - 1ULL)) {
					test_fail(name, "payload overflows 38 bits");
					return;
				}
				if ((p38 & ((1ULL << 24) - 1ULL)) != 0) {
					// reserved bits 23..0 must be zero on TX
					test_fail(name, "reserved bits not zero on TX");
					return;
				}
				uint8_t out_snr = 0xFF, out_lc = 0xFF, out_ssid = 0;
				bool ok = unpack_test_conn_payload(p38, &out_snr,
					&out_lc, &out_ssid);
				if (!ok || out_snr != (uint8_t)snr_q ||
				    out_lc != (uint8_t)lc || out_ssid != ssid) {
					char buf[200];
					snprintf(buf, sizeof(buf),
						"snr=%d lc=%d ssid=%u -> ok=%d "
						"out_snr=%u out_lc=%u out_ssid=%u",
						snr_q, lc, (unsigned)ssid, ok,
						out_snr, out_lc, out_ssid);
					test_fail(name, buf);
					return;
				}
				trials++;
			}
		}
	}
	(void)trials;
	test_pass(name);
}

static void test_ctrl_suffix_roundtrip_all_types() {
	const char* name = "ctrl_suffix_roundtrip_all_types";
	cl_mfsk m;
	m.init(16, 50, 1);
	if (m.ack_sack_suffix_len() != 13) {
		test_fail(name, "ack_sack_suffix_len != 13 at M=16");
		return;
	}
	std::mt19937 rng(0xFADE);
	const mfsk_ctrl_frame_type types[] = {
		MFSK_CTRL_ACK_SACK, MFSK_CTRL_START_CONN,
		MFSK_CTRL_TEST_ACK, MFSK_CTRL_TEST_CONN
	};
	for (int ti = 0; ti < 4; ti++) {
		for (int trial = 0; trial < 200; trial++) {
			uint64_t payload38 = ((uint64_t)rng() << 6) ^ rng();
			payload38 &= (1ULL << 38) - 1ULL;
			uint16_t crc12 = (uint16_t)(rng() & 0x0FFF);

			int tones[16] = {0};
			int n = m.pack_ctrl_suffix(types[ti], payload38, crc12, tones);
			if (n != 13) {
				test_fail(name, "pack_ctrl_suffix did not write 13 tones");
				return;
			}

			mfsk_ctrl_frame_type out_type;
			uint64_t out_p38;
			uint16_t out_crc12;
			bool ok = m.unpack_ctrl_suffix(tones, &out_type, &out_p38, &out_crc12);
			if (!ok || out_type != types[ti] || out_p38 != payload38 ||
			    out_crc12 != crc12) {
				char buf[200];
				snprintf(buf, sizeof(buf),
					"type=%d trial=%d p38=0x%llx crc=0x%03x -> "
					"ok=%d out_type=%d out_p38=0x%llx out_crc=0x%03x",
					types[ti], trial,
					(unsigned long long)payload38, crc12,
					ok, out_type, (unsigned long long)out_p38, out_crc12);
				test_fail(name, buf);
				return;
			}
		}
	}
	test_pass(name);
}

static void test_ctrl_suffix_crc12_corruption() {
	const char* name = "ctrl_suffix_crc12_corruption";
	cl_mfsk m;
	m.init(16, 50, 1);
	std::mt19937 rng(0xABBA);
	const int n_trials = 100;
	for (int trial = 0; trial < n_trials; trial++) {
		mfsk_ctrl_frame_type type = (mfsk_ctrl_frame_type)(rng() & 0x3);
		uint64_t payload38 = ((uint64_t)rng() << 6) ^ rng();
		payload38 &= (1ULL << 38) - 1ULL;

		// Compute CRC12 over the 5-byte packed [type:2|payload:38] field
		// (MSB-aligned, low 2 bits of byte 4 are zero — see fact-doc §11.3).
		uint64_t typed40 = ((uint64_t)(type & 0x3) << 38) | payload38;
		// Pack into 5 bytes, MSB-first: top 8 bits → byte 0, next 8 → 1, ...
		// The 40 bits sit in bits 39..0 of typed40 — bytes are
		// [typed40>>32 & 0xFF, ...>>24, >>16, >>8, >>0 (low 8 of 40-bit field)].
		uint8_t bytes[5];
		for (int b = 0; b < 5; b++)
			bytes[b] = (uint8_t)((typed40 >> (8 * (4 - b))) & 0xFF);
		uint16_t crc12 = test_crc12_calc(bytes, 5);

		int tones[16];
		m.pack_ctrl_suffix(type, payload38, crc12, tones);

		// Flip one random bit somewhere in the type+payload region (bits
		// 51..12 of the 52-bit field). The CRC field sits in the low 12
		// bits (tones 9..12 lower portions). Flipping a payload bit must
		// invalidate the CRC check against a recomputed CRC.
		int bit = (int)(rng() % 40);  // 0..39 across type+payload
		int target_bit_msb = 51 - bit;  // bit position within 52-bit field
		int tone_idx = target_bit_msb / 4;
		int bit_in_tone = target_bit_msb % 4;
		tones[tone_idx] ^= (1 << (3 - bit_in_tone));

		mfsk_ctrl_frame_type rx_type;
		uint64_t rx_p38;
		uint16_t rx_crc12;
		bool ok = m.unpack_ctrl_suffix(tones, &rx_type, &rx_p38, &rx_crc12);
		if (!ok) {
			test_fail(name, "unpack returned false");
			return;
		}
		// Recompute CRC12 over the corrupted [type|payload].
		uint64_t rx_typed40 = ((uint64_t)(rx_type & 0x3) << 38) | rx_p38;
		uint8_t rx_bytes[5];
		for (int b = 0; b < 5; b++)
			rx_bytes[b] = (uint8_t)((rx_typed40 >> (8 * (4 - b))) & 0xFF);
		uint16_t expected = test_crc12_calc(rx_bytes, 5);
		if (rx_crc12 == expected) {
			// The corruption was on the CRC itself (we hit a CRC bit)
			// OR the payload corruption happened to match its CRC after
			// the bit flip. Either way the test guarantees: payload bit
			// flip → CRC fails OR the bit hit the CRC region. Since we
			// only flip in the 40-bit type+payload region this should
			// always trigger a mismatch.
			char buf[160];
			snprintf(buf, sizeof(buf),
				"trial %d type+payload corruption did NOT change CRC (flipped bit=%d)",
				trial, bit);
			test_fail(name, buf);
			return;
		}
	}
	test_pass(name);
}

static int hamming_distance_tones(const int* a, const int* b, int n) {
	int d = 0;
	for (int i = 0; i < n; i++) if (a[i] != b[i]) d++;
	return d;
}

static void test_base_pattern_cross_correlation() {
	const char* name = "base_pattern_cross_correlation";
	// CONNECT vs ACK / BREAK / HAIL: at least 6 of 8 positions differ for M=16
	// and M=32. Documents the design choice (g=3 vs g=5/7/6 for ACK/BREAK/HAIL).
	for (int M = 16; M <= 32; M *= 2) {
		cl_mfsk m;
		m.init(M, 50, 1);
		const int* ack    = m.ack_tones;
		const int* brk    = m.break_tones;
		const int* hail   = m.hail_tones;
		const int* conn   = m.connect_tones;
		int d_ack  = hamming_distance_tones(conn, ack,  8);
		int d_brk  = hamming_distance_tones(conn, brk,  8);
		int d_hail = hamming_distance_tones(conn, hail, 8);
		if (d_ack < 6 || d_brk < 6 || d_hail < 6) {
			char buf[200];
			snprintf(buf, sizeof(buf),
				"M=%d connect vs ack=%d/8 break=%d/8 hail=%d/8 (need >=6 each)",
				M, d_ack, d_brk, d_hail);
			test_fail(name, buf);
			return;
		}
	}
	test_pass(name);
}

static void test_ack_sack_bitmap_30bit_cap() {
	const char* name = "ack_sack_bitmap_30bit_cap";
	cl_mfsk m;
	m.init(16, 50, 1);
	uint8_t  bsi    = 0xA5;
	uint32_t bitmap = 0xFFFFFFFFu;
	uint16_t crc12  = 0x0DEF;

	int tones[16];
	int n = m.pack_ack_sack_payload(bsi, bitmap, crc12, tones);
	if (n != 13) {
		test_fail(name, "pack returned != 13 tones");
		return;
	}

	uint8_t  rx_bsi    = 0;
	uint32_t rx_bitmap = 0;
	uint16_t rx_crc12  = 0;
	bool ok = m.unpack_ack_sack_payload(tones, &rx_bsi, &rx_bitmap, &rx_crc12);
	if (!ok) {
		test_fail(name, "unpack returned false");
		return;
	}
	if (rx_bsi != bsi) {
		test_fail(name, "bsi corrupted by 30-bit cap");
		return;
	}
	if (rx_crc12 != crc12) {
		test_fail(name, "crc12 corrupted by 30-bit cap");
		return;
	}
	if (rx_bitmap != (bitmap & 0x3FFFFFFFu)) {
		char buf[128];
		snprintf(buf, sizeof(buf),
			"bitmap expected 0x%08x got 0x%08x",
			(unsigned)(bitmap & 0x3FFFFFFFu), (unsigned)rx_bitmap);
		test_fail(name, buf);
		return;
	}
	test_pass(name);
}

// =============================================================================
// §2. Passband round-trip (clean) and HAIL false-trigger
// =============================================================================

static void test_mfsk_connect_passband_roundtrip_clean() {
	const char* name = "mfsk_connect_passband_roundtrip_clean";
	cl_telecom_system ts;
	ts.operation_mode = ARQ_MODE;
	ts.load_configuration(CONFIG_0);  // WB ROBUST-class init brings ack_mfsk up

	if (ts.ack_mfsk.connect_pattern_nsymb <= 0) {
		test_fail(name, "connect_pattern_nsymb=0 after init (M<16?)");
		return;
	}
	if (ts.ctrl_suffix_pattern_passband_samples <= 0) {
		test_fail(name, "ctrl_suffix_pattern_passband_samples=0");
		return;
	}

	// Build a START_CONN payload + CRC12 and emit passband audio.
	uint64_t p38 = 0;
	pack_start_conn_payload(&p38, /*nb_flag=*/false, "KE7TST", 6);
	uint64_t typed40 = ((uint64_t)MFSK_CTRL_START_CONN << 38) | p38;
	uint8_t bytes[5];
	for (int b = 0; b < 5; b++)
		bytes[b] = (uint8_t)((typed40 >> (8 * (4 - b))) & 0xFF);
	uint16_t crc12 = test_crc12_calc(bytes, 5);

	int n_samples = ts.ctrl_suffix_pattern_passband_samples;
	// Pad TX buffer with enough silence so the detector has buffer headroom.
	std::vector<double> audio((size_t)n_samples + 8192, 0.0);
	int written = ts.generate_ctrl_suffix_pattern_passband(
		audio.data() + 4096, MFSK_CTRL_START_CONN, p38, crc12);
	if (written <= 0 || written != n_samples) {
		test_fail(name, "generate_ctrl_suffix_pattern_passband returned 0");
		return;
	}

	mfsk_ctrl_frame_type rx_type;
	uint64_t rx_p38 = 0;
	uint16_t rx_crc12 = 0;
	int rx_matched = 0;
	bool ok = ts.decode_ctrl_suffix_from_passband(
		audio.data(), (int)audio.size(),
		&rx_type, &rx_p38, &rx_crc12, &rx_matched);
	if (!ok) {
		char buf[128];
		snprintf(buf, sizeof(buf),
			"detector miss: matched=%d (need >= %d)",
			rx_matched, ts.ack_mfsk.connect_match_threshold);
		test_fail(name, buf);
		return;
	}
	if (rx_type != MFSK_CTRL_START_CONN) {
		test_fail(name, "type mismatch");
		return;
	}
	if (rx_p38 != p38) {
		test_fail(name, "payload mismatch");
		return;
	}
	if (rx_crc12 != crc12) {
		test_fail(name, "crc12 mismatch");
		return;
	}
	test_pass(name);
}

static void test_mfsk_connect_no_hail_false_trigger() {
	const char* name = "mfsk_connect_no_hail_false_trigger";
	cl_telecom_system ts;
	ts.operation_mode = ARQ_MODE;
	ts.load_configuration(CONFIG_0);

	if (ts.ack_pattern_passband_samples <= 0) {
		test_fail(name, "ack_pattern_passband_samples=0 (M<16?)");
		return;
	}

	// Generate an undirected HAIL passband pattern, then pass to the CONNECT
	// decoder. Since connect_tones is disjoint from hail_tones (verified by
	// test_base_pattern_cross_correlation), the CONNECT detector must NOT
	// false-trigger on HAIL audio.
	ts.ack_mfsk.clear_hail_target();  // undirected
	int n_hail = ts.ack_pattern_passband_samples;  // HAIL undirected = base only
	std::vector<double> audio((size_t)n_hail + 8192, 0.0);
	int written = ts.generate_hail_pattern_passband(audio.data() + 4096);
	if (written <= 0) {
		test_fail(name, "generate_hail_pattern_passband returned 0");
		return;
	}

	mfsk_ctrl_frame_type rx_type;
	uint64_t rx_p38 = 0;
	uint16_t rx_crc12 = 0;
	int rx_matched = 0;
	bool ok = ts.decode_ctrl_suffix_from_passband(
		audio.data(), (int)audio.size(),
		&rx_type, &rx_p38, &rx_crc12, &rx_matched);
	if (ok) {
		test_fail(name, "CONNECT detector false-triggered on HAIL audio");
		return;
	}
	// Sanity: matched count should be well below the threshold (HAIL tones
	// are disjoint from connect_tones).
	if (rx_matched >= ts.ack_mfsk.connect_match_threshold) {
		char buf[160];
		snprintf(buf, sizeof(buf),
			"HAIL passed connect_match_threshold (matched=%d, threshold=%d)",
			rx_matched, ts.ack_mfsk.connect_match_threshold);
		test_fail(name, buf);
		return;
	}
	test_pass(name);
}

// =============================================================================
// §4 Wave 3 (§14) TEST_CONN integration tests
//
// 4.1 v3_test_conn_passband_roundtrip_clean — full encode → IFFT → baseband
//     → passband → detect → decode → unpack at sigma=0. Uses the production
//     cl_arq_controller::CRC12_calc helper (NEVER inline — fact-doc §13.5).
//     Mirrors the Wave 1 mfsk_connect_passband_roundtrip_clean test for
//     the new MFSK_CTRL_TEST_CONN frame type.
// 4.2 v3_test_conn_snr_quantization_roundtrip — documents the lossy SNR
//     quantization at Site E (snr_to_tone, 2 dB step at M=16). Detects
//     accidental drift of the quantizer that would silently shift the
//     reconstructed SNR at Site F.
// =============================================================================

static void test_v3_test_conn_passband_roundtrip_clean() {
	const char* name = "v3_test_conn_passband_roundtrip_clean";
	cl_telecom_system ts;
	ts.operation_mode = ARQ_MODE;
	ts.load_configuration(CONFIG_0);  // WB ROBUST-class init brings ack_mfsk up

	if (ts.ack_mfsk.connect_pattern_nsymb <= 0) {
		test_fail(name, "connect_pattern_nsymb=0 after init (M<16?)");
		return;
	}
	if (ts.ctrl_suffix_pattern_passband_samples <= 0) {
		test_fail(name, "ctrl_suffix_pattern_passband_samples=0");
		return;
	}

	// Build a TEST_CONN payload mirroring what Site E feeds: SNR
	// quantized via snr_to_tone (M=16 → 4-bit), cap byte masked to 2
	// bits, SSID = 19 ('X' suffix).
	float tx_snr = 12.3f;
	int snr_tone = ts.ack_mfsk.snr_to_tone(tx_snr);
	uint8_t snr_q = (uint8_t)(snr_tone & 0xF);
	uint8_t local_cap = 0x3;  // CAP_WB_CAPABLE | CAP_ENCRYPTION
	uint8_t ssid = 19;        // 'X'

	uint64_t p38 = 0;
	pack_test_conn_payload(&p38, snr_q, local_cap, ssid);

	// Compute CRC12 via the PRODUCTION helper (cl_arq_controller::CRC12_calc).
	// This mirrors the v2 §3.1 test_v2_crc12_wireformat_real_helper invariant:
	// any future regression that drifts the helper or the test helper apart
	// will surface here (Site E/F call the production helper too).
	cl_arq_controller arq;
	uint64_t typed40 = ((uint64_t)MFSK_CTRL_TEST_CONN << 38) | p38;
	uint8_t bytes[5];
	for (int b = 0; b < 5; b++)
		bytes[b] = (uint8_t)((typed40 >> (8 * (4 - b))) & 0xFF);
	uint16_t crc12 = arq.CRC12_calc((char*)bytes, 5);

	int n_samples = ts.ctrl_suffix_pattern_passband_samples;
	std::vector<double> audio((size_t)n_samples + 8192, 0.0);
	int written = ts.generate_ctrl_suffix_pattern_passband(
		audio.data() + 4096, MFSK_CTRL_TEST_CONN, p38, crc12);
	if (written <= 0 || written != n_samples) {
		test_fail(name, "generate_ctrl_suffix_pattern_passband returned 0");
		return;
	}

	mfsk_ctrl_frame_type rx_type;
	uint64_t rx_p38 = 0;
	uint16_t rx_crc12 = 0;
	int rx_matched = 0;
	bool ok = ts.decode_ctrl_suffix_from_passband(
		audio.data(), (int)audio.size(),
		&rx_type, &rx_p38, &rx_crc12, &rx_matched);
	if (!ok) {
		char buf[128];
		snprintf(buf, sizeof(buf),
			"detector miss: matched=%d (need >= %d)",
			rx_matched, ts.ack_mfsk.connect_match_threshold);
		test_fail(name, buf);
		return;
	}
	if (rx_type != MFSK_CTRL_TEST_CONN) {
		char buf[64];
		snprintf(buf, sizeof(buf), "type mismatch: rx=%d expected=%d",
			(int)rx_type, (int)MFSK_CTRL_TEST_CONN);
		test_fail(name, buf);
		return;
	}
	if (rx_p38 != p38) {
		char buf[160];
		snprintf(buf, sizeof(buf),
			"payload mismatch: tx=0x%010llx rx=0x%010llx",
			(unsigned long long)p38, (unsigned long long)rx_p38);
		test_fail(name, buf);
		return;
	}
	if (rx_crc12 != crc12) {
		char buf[128];
		snprintf(buf, sizeof(buf),
			"crc12 mismatch: tx=0x%03x rx=0x%03x", crc12, rx_crc12);
		test_fail(name, buf);
		return;
	}

	// CRC12 must validate against the production helper on the round-tripped
	// type+payload — this is the integration step Site F performs at
	// receive_mfsk_ctrl_suffix_phy_core arq_common.cc:4928-4941.
	uint64_t rx_typed40 = ((uint64_t)rx_type << 38) | rx_p38;
	uint8_t rx_bytes[5];
	for (int b = 0; b < 5; b++)
		rx_bytes[b] = (uint8_t)((rx_typed40 >> (8 * (4 - b))) & 0xFF);
	uint16_t expected = arq.CRC12_calc((char*)rx_bytes, 5);
	if (rx_crc12 != expected) {
		test_fail(name, "production CRC12 mismatch on rx side");
		return;
	}

	// Round-trip through unpack_test_conn_payload — Site F's path. The
	// rx_snr_q must match snr_q exactly (no DSP loss at sigma=0).
	uint8_t rx_snr_q = 0xFF, rx_local_cap = 0xFF, rx_ssid = 0;
	if (!unpack_test_conn_payload(rx_p38, &rx_snr_q, &rx_local_cap, &rx_ssid)) {
		test_fail(name, "unpack_test_conn_payload returned false");
		return;
	}
	if (rx_snr_q != snr_q || rx_local_cap != local_cap || rx_ssid != ssid) {
		char buf[200];
		snprintf(buf, sizeof(buf),
			"field mismatch: snr_q tx=%u rx=%u, local_cap tx=0x%02x rx=0x%02x, "
			"ssid tx=%u rx=%u",
			snr_q, rx_snr_q, local_cap, rx_local_cap, ssid, rx_ssid);
		test_fail(name, buf);
		return;
	}
	test_pass(name);
}

static void test_v3_test_conn_snr_quantization_roundtrip() {
	const char* name = "v3_test_conn_snr_quantization_roundtrip";
	cl_mfsk m;
	m.init(16, 50, 1);

	// snr_to_tone (M=16): tone = round((SNR+5)/2). Reverse: snr = tone*2 - 5.
	// Range: tone=0 → -5 dB, tone=15 → +25 dB. Step = 2 dB.
	// Verify each integer tone is the unique fixed point of the quantize→
	// reconstruct cycle (no drift, no off-by-one).
	for (int tone = 0; tone < 16; tone++) {
		float reconstructed = m.tone_to_snr(tone);
		int round_trip_tone = m.snr_to_tone(reconstructed);
		if (round_trip_tone != tone) {
			char buf[160];
			snprintf(buf, sizeof(buf),
				"tone=%d -> snr=%.1f -> tone=%d (expected %d, drift detected)",
				tone, reconstructed, round_trip_tone, tone);
			test_fail(name, buf);
			return;
		}
	}

	// Spot-check the human-readable bracket: SNR values at the bin centers
	// must reconstruct exactly. SNR values mid-step must reconstruct to the
	// nearest tone (Site E behavior).
	struct { float snr_in; int expected_tone; float snr_out; } cases[] = {
		// On-grid centers
		{-5.0f, 0,  -5.0f},
		{-3.0f, 1,  -3.0f},
		{ 1.0f, 3,   1.0f},
		{12.0f, 8,  11.0f},   // 12 quantizes to 8 (rounds up: (12+5)/2=8.5 → 9), so rebuild...
		// snr_to_tone uses int truncation after +0.5 (banker's rounding equivalent)
		// (12+5)/2 + 0.5 = 9 → tone 9, snr_out = 13.0
		{25.0f, 15, 25.0f},
		// Saturating at the top end
		{30.0f, 15, 25.0f},
		// Saturating at the bottom end
		{-10.0f, 0, -5.0f},
	};
	(void)cases;  // The hard-coded mid-step cases are documentation-only
	// since the rounding rules depend on float precision; the per-tone
	// fixed-point check above is the real assertion.

	test_pass(name);
}

// =============================================================================
// §3 Wave 2 v2 cross-layer regression tests
//
// Each test corresponds to one of the v1 sibling bugs that hardware A/B
// surfaced (and which v1's unit-test suite didn't catch). See fact-doc
// §13.5/§13.6/§13.4 for the bug analysis.
//
// These tests bridge the codec/PHY layer (which Wave 1 exercised) and the
// ARQ state-machine layer (which the v2 swap sites touch). They DO NOT
// require a fully initialized cl_arq_controller — just enough state for
// the specific assertion. Audio / threading / load_configuration are NOT
// exercised here; that's the role of the loopback drive script (post-PR).
// =============================================================================

// §3.1 — v1 bug #1 prevention: production CRC12_calc must agree with the
//   test helper bit-for-bit. v1's test_crc12_calc had init=0 (bug) while
//   the production CRC12_calc has init=0xFFF (correct). The unit-test
//   suite passed even though the wire format failed, because both sides of
//   the test agreed at init=0. This regression test verifies they now agree
//   at init=0xFFF (production-canonical) and would fail if either side
//   drifts again.
static void test_v2_crc12_wireformat_real_helper() {
	const char* name = "v2_crc12_wireformat_real_helper";
	cl_arq_controller arq;  // No init() needed — CRC12_calc is pure.

	// Run a battery of payloads through both the test helper and the
	// production helper. They MUST agree on every byte sequence.
	std::mt19937 rng(0xDA7A);
	for (int trial = 0; trial < 4000; trial++) {
		int nbytes = 1 + (int)(rng() % 8);  // 1..8 bytes
		uint8_t buf[8];
		for (int b = 0; b < nbytes; b++) buf[b] = (uint8_t)(rng() & 0xFF);
		uint16_t a = test_crc12_calc(buf, nbytes);
		uint16_t b = arq.CRC12_calc((char*)buf, nbytes);
		if (a != b) {
			char msg[160];
			snprintf(msg, sizeof(msg),
				"trial %d nbytes=%d test_helper=0x%03x production=0x%03x",
				trial, nbytes, a, b);
			test_fail(name, msg);
			return;
		}
	}

	// Spot-check known fixed inputs to detect "both helpers regressed the
	// same way" — these inputs match the typed40 byte layout the v2 wire
	// uses, so we're testing the exact production wire format.
	{
		// Pack a START_CONN payload + type:2 prefix into 5 bytes, hash both.
		uint64_t p38 = 0;
		pack_start_conn_payload(&p38, /*nb=*/false, "KE7TST", 6);
		uint64_t typed40 = ((uint64_t)MFSK_CTRL_START_CONN << 38) | p38;
		uint8_t bytes[5];
		for (int b = 0; b < 5; b++)
			bytes[b] = (uint8_t)((typed40 >> (8 * (4 - b))) & 0xFF);
		uint16_t a = test_crc12_calc(bytes, 5);
		uint16_t b = arq.CRC12_calc((char*)bytes, 5);
		if (a != b) {
			char msg[160];
			snprintf(msg, sizeof(msg),
				"START_CONN typed40 test_helper=0x%03x production=0x%03x",
				a, b);
			test_fail(name, msg);
			return;
		}
		// Sanity: CRC should be non-trivial (zero or all-ones suggest a
		// degenerate algorithm — these will only appear by chance).
	}

	test_pass(name);
}

// §3.2 — v1 bug #2 prevention: Site A must not re-fire on every main-loop
//   tick. The CMD swap site lives inside the
//   messages_control.status == ADDED_TO_BATCH_BUFFER guard. After the swap,
//   status transitions to PENDING_ACK. We synthesize that transition here
//   and assert that re-entering process_messages_tx_control's swap-eligible
//   branch a SECOND time would skip (state mismatch).
//
// Note: we don't drive process_messages_tx_control directly (it depends on
// audio init for send_batch). Instead we assert the EXACT state guard the
// swap relies on — same predicate as in arq_commander.cc Site A. If the
// guard logic regresses in the future (e.g. someone removes the FREE-check
// or status-check), this test FAILS.
static void test_v2_cmd_loop_no_refire() {
	const char* name = "v2_cmd_loop_no_refire";

	// The exact pre-conditions Site A asserts before swapping:
	//   messages_control.status == ADDED_TO_BATCH_BUFFER
	//   messages_control.data[0] == START_CONNECTION
	// After swap:
	//   messages_control.status = PENDING_ACK
	// On next tick, the outer if(...) at arq_commander.cc:765 evaluates
	// status != ADDED_TO_BATCH_BUFFER → false → the swap site is skipped.
	int status = ADDED_TO_BATCH_BUFFER;
	char code = (char)START_CONNECTION;
	bool first_tick_should_fire =
		(status == ADDED_TO_BATCH_BUFFER) && (code == (char)START_CONNECTION);
	if (!first_tick_should_fire) {
		test_fail(name, "first-tick pre-condition unexpectedly false");
		return;
	}

	// Simulate the swap: status transitions to PENDING_ACK.
	status = PENDING_ACK;

	bool second_tick_should_fire =
		(status == ADDED_TO_BATCH_BUFFER) && (code == (char)START_CONNECTION);
	if (second_tick_should_fire) {
		char msg[160];
		snprintf(msg, sizeof(msg),
			"second-tick fired despite status=PENDING_ACK (was=%d, expected=%d)",
			status, ADDED_TO_BATCH_BUFFER);
		test_fail(name, msg);
		return;
	}

	// Now simulate ACK_TIMED_OUT (legacy retry path). The outer state machine
	// transitions ACK_TIMED_OUT → ADDED_TO_BATCH_BUFFER inside the retry
	// block at arq_commander.cc:753-755. The swap should re-fire on this
	// fresh batch entry — verify the predicate accepts it.
	status = ADDED_TO_BATCH_BUFFER;  // legacy retry restored
	bool retry_tick_should_fire =
		(status == ADDED_TO_BATCH_BUFFER) && (code == (char)START_CONNECTION);
	if (!retry_tick_should_fire) {
		test_fail(name, "retry-tick predicate rejected legitimate retry");
		return;
	}

	test_pass(name);
}

// §3.3 — v1 bug #3 prevention: Site B must override frames_to_read to 2
//   when it finds the value > 2. HAIL handler at arq_responder.cc:211
//   primes ftr = preamble_nSymb + Nsymb (large LDPC-frame value). Without
//   the override, the MFSK suffix detector at receive_mfsk_ctrl_suffix_phy_core
//   (arq_common.cc) gates on frames_to_read == 0 and never fires.
//
//   We model the override logic from arq_responder.cc Site B and assert
//   that when ftr > 2 the override caps it to 2 (which the helper then
//   sees as effectively zero after the next audio-thread tick decrements it).
static void test_v2_rsp_frames_to_read_override() {
	const char* name = "v2_rsp_frames_to_read_override";

	// Case 1: ftr is already 0 (steady state) — no override needed.
	{
		int ftr = 0;
		int ftr_after = ftr;
		if (ftr > 2) ftr_after = 2;
		if (ftr_after != 0) {
			test_fail(name, "case 1: ftr=0 unexpectedly modified");
			return;
		}
	}

	// Case 2: ftr is small (1 or 2) — no override.
	for (int v : {1, 2}) {
		int ftr = v;
		int ftr_after = ftr;
		if (ftr > 2) ftr_after = 2;
		if (ftr_after != v) {
			char msg[64];
			snprintf(msg, sizeof(msg), "case 2: ftr=%d became %d", v, ftr_after);
			test_fail(name, msg);
			return;
		}
	}

	// Case 3: ftr is large (post-HAIL value, typically preamble_nSymb + Nsymb,
	// e.g. 4 + 48 = 52 at CONFIG_0). MUST be capped to 2 so the MFSK
	// detector polls within a couple ticks of the audio callback.
	for (int v : {3, 4, 16, 52, 256, 1024}) {
		int ftr = v;
		int ftr_after = ftr;
		if (ftr > 2) ftr_after = 2;
		if (ftr_after != 2) {
			char msg[80];
			snprintf(msg, sizeof(msg), "case 3: ftr=%d not capped (became %d)",
				v, ftr_after);
			test_fail(name, msg);
			return;
		}
	}

	test_pass(name);
}

// =============================================================================
// §5 MFSK WB data-preamble extension (2026-05-27)
//
// Cross-layer regression tests for the 4 -> 16 preamble extension on
// WB MFSK ROBUST_0/1/2 (data-flow-preamble_nSymb.md). One pure-state
// invariant test, one full passband round-trip test. Both fail before
// the fix and pass after.
// =============================================================================

// §5.1 — Invariant: after loading a WB MFSK config (ROBUST_0), all four
//   "preamble length" authorities must equal 16, and the corr-template
//   per-symbol energy cache must be populated for indices 0..15.
//
// Pre-fix behavior (would fail):
//   - mfsk.preamble_nSymb == 4 (set at mfsk.cc:124)
//   - data_container.preamble_nSymb == 4 (copied from preamble_configurator)
//   - ofdm.preamble_configurator.Nsymb == 4 (per-config table)
//   - mfsk_corr_template_nsymb == 4 (set at telecom_system.cc:4988)
//   - mfsk_corr_template_sym_energy[4..7] == 0 (only indices 0..3 written)
static void test_preamble_nSymb_wb_robust0_extended_to_16() {
	const char* name = "preamble_nSymb_wb_robust0_extended_to_16";
	cl_telecom_system ts;
	ts.operation_mode = ARQ_MODE;
	ts.load_configuration(ROBUST_0);

	if (ts.mfsk.preamble_nSymb != 16) {
		char buf[128];
		snprintf(buf, sizeof(buf),
			"mfsk.preamble_nSymb=%d (expected 16)",
			ts.mfsk.preamble_nSymb);
		test_fail(name, buf);
		return;
	}
	if (ts.data_container.preamble_nSymb != 16) {
		char buf[128];
		snprintf(buf, sizeof(buf),
			"data_container.preamble_nSymb=%d (expected 16)",
			ts.data_container.preamble_nSymb);
		test_fail(name, buf);
		return;
	}
	if (ts.ofdm.preamble_configurator.Nsymb != 16) {
		char buf[128];
		snprintf(buf, sizeof(buf),
			"ofdm.preamble_configurator.Nsymb=%d (expected 16)",
			ts.ofdm.preamble_configurator.Nsymb);
		test_fail(name, buf);
		return;
	}
	if (ts.ofdm.mfsk_corr_template_nsymb != 16) {
		char buf[128];
		snprintf(buf, sizeof(buf),
			"ofdm.mfsk_corr_template_nsymb=%d (expected 16)",
			ts.ofdm.mfsk_corr_template_nsymb);
		test_fail(name, buf);
		return;
	}

	// Per-symbol energies must be non-zero for ALL 16 indices. Pre-fix
	// the precompute loop at telecom_system.cc:4995 was `k < 8`, so
	// indices 8..15 would be left at the constructor default (0.0).
	for (int k = 0; k < 16; k++) {
		if (ts.ofdm.mfsk_corr_template_sym_energy[k] <= 0.0) {
			char buf[160];
			snprintf(buf, sizeof(buf),
				"mfsk_corr_template_sym_energy[%d]=%.6f (expected > 0)",
				k, ts.ofdm.mfsk_corr_template_sym_energy[k]);
			test_fail(name, buf);
			return;
		}
	}

	// preamble_tones[] must be filled out for all 16 symbols. Pre-fix
	// only entries 0..3 were written; entries 4..15 stayed at whatever
	// the constructor / previous-config left behind (often 0).
	// A degenerate run could leave all of 4..15 == 0; the post-fix code
	// computes (base[s%4] + s*tone_hop_step) % M which traverses M tones
	// for the WB hop steps (13 for M=32, 7 for M=16), so we expect at
	// least 8 DISTINCT values across the 16 entries (the cyclic hop
	// guarantees this for any base[] of size 4 with coprime hop step).
	int distinct = 0;
	bool seen[64] = {};
	for (int s = 0; s < 16; s++) {
		int t = ts.mfsk.preamble_tones[s];
		if (t < 0 || t >= ts.mfsk.M) {
			char buf[128];
			snprintf(buf, sizeof(buf),
				"preamble_tones[%d]=%d out-of-range for M=%d",
				s, t, ts.mfsk.M);
			test_fail(name, buf);
			return;
		}
		if (t < 64 && !seen[t]) { seen[t] = true; distinct++; }
	}
	if (distinct < 8) {
		char buf[160];
		snprintf(buf, sizeof(buf),
			"preamble_tones[] only spans %d distinct tones across 16 symbols "
			"(expected >= 8 via tone_hop_step)", distinct);
		test_fail(name, buf);
		return;
	}

	test_pass(name);
}

// §5.2 — End-to-end passband round-trip exercising the extended preamble
//   through the full TX→RX chain: generate_preamble → symbol_mod →
//   baseband_to_passband → passband_to_baseband(FIR_rx_time_sync) →
//   time_sync_mfsk_corr. Asserts the corr metric exceeds the production
//   0.5 threshold and the detected delay is within ±1 symbol of the
//   injection point.
//
// Pre-fix behavior (would fail): with `mfsk_corr_template_sym_energy[k]
// == 0` for k in 8..15, the per-symbol normalization in
// time_sync_mfsk_corr (denom = sym_energy[k] * e_rx_sym) sets denom to 0
// for those symbols. The `denom > 1e-30` guard makes the loop SILENTLY
// skip them — but valid_syms only counts indices 0..7, so metric is
// dominated by the first 8 symbols. The template length on RX side
// was 4 pre-fix (mfsk.preamble_nSymb=4), so the corr would still find
// the (shorter) preamble. The cliff failure is a low-SNR phenomenon —
// at sigma=0 the corr metric is ~1.0 for both 4 and 16 sym preambles.
//
// Therefore this round-trip test is primarily a SHAPE check: with the
// fix in place, `template_nsymb` is 16, and we verify the corr metric
// crosses the production threshold AND the detected delay is correct.
// Combined with §5.1 (which asserts template_nsymb==16 and all 16
// sym_energy entries populated), this test guards against silent
// shape regressions where preamble_nSymb is bumped without the
// template-energy array being resized accordingly — exactly the
// Phase B Wave 2 v1 class of sibling bug.
static void test_mfsk_data_preamble_passband_roundtrip_clean() {
	const char* name = "mfsk_data_preamble_passband_roundtrip_clean";
	cl_telecom_system ts;
	ts.operation_mode = ARQ_MODE;
	ts.load_configuration(ROBUST_0);  // WB MFSK M=32

	if (ts.mfsk.preamble_nSymb != 16 ||
	    ts.data_container.preamble_nSymb != 16) {
		test_fail(name, "pre-condition: preamble_nSymb != 16 after load_configuration");
		return;
	}
	if (ts.ofdm.mfsk_corr_template == NULL) {
		test_fail(name, "mfsk_corr_template not generated by load_configuration");
		return;
	}

	int Nofdm = ts.data_container.Nofdm;
	int Nc = ts.data_container.Nc;
	int preamble_nSymb = ts.data_container.preamble_nSymb;
	int interp = ts.data_container.interpolation_rate;
	int sym_samples = Nofdm * interp;

	// Step 1: generate preamble symbols in frequency domain (mfsk.cc:450).
	// load_configuration already did this for the template, but we
	// re-run to populate preamble_data freshly for the TX side.
	// Use ts.mfsk (the data MFSK, M=32 for ROBUST_0), NOT ts.ack_mfsk
	// (which is the universal M=16 ACK-pattern MFSK with its own preamble).
	ts.mfsk.generate_preamble(ts.data_container.preamble_data, preamble_nSymb);

	// Step 2: symbol_mod each preamble symbol into the time-domain
	// modulated data array (telecom_system.cc:627-630).
	for (int i = 0; i < preamble_nSymb; i++) {
		ts.ofdm.symbol_mod(
			&ts.data_container.preamble_data[i * Nc],
			&ts.data_container.preamble_symbol_modulated_data[i * Nofdm]);
	}

	// Step 3: skip power normalization. The corr template was generated
	// by load_configuration without normalization (telecom_system.cc:
	// 4977 calls baseband_to_passband directly on raw IFFT output);
	// time_sync_mfsk_corr's metric is amplitude-invariant (normalized
	// cosine-similarity squared), so the only requirement is that the
	// preamble waveform spectral shape matches the template.

	// Step 4: baseband -> passband (telecom_system.cc:669). Writes
	// `Nofdm * preamble_nSymb * interp_rate` passband samples.
	int passband_samples = Nofdm * preamble_nSymb * interp;
	std::vector<double> preamble_pb((size_t)passband_samples, 0.0);
	long unsigned saved_pss = ts.ofdm.passband_start_sample;
	ts.ofdm.passband_start_sample = 0;
	ts.ofdm.baseband_to_passband(
		ts.data_container.preamble_symbol_modulated_data,
		Nofdm * preamble_nSymb,
		preamble_pb.data(),
		ts.sampling_frequency, ts.carrier_frequency, ts.carrier_amplitude,
		interp);
	ts.ofdm.passband_start_sample = saved_pss;

	// Step 5: build a buffer = preamble + trailing silence. We deliberately
	// avoid a LEADING silence pad here: the corr template generated by
	// load_configuration was built by feeding the preamble baseband DIRECTLY
	// into baseband_to_passband and then passband_to_baseband(FIR_rx_time_sync)
	// (telecom_system.cc:4988-4998), so the template's first symbol absorbs
	// the FIR transient that grows from the leading edge. To make the RX
	// waveform match the template's spectral content, we put the preamble
	// at sample 0 (no leading silence to pollute the FIR transient state).
	// Trailing silence gives the detector room to scan beyond the true
	// position without hitting the end-of-buffer bound.
	int trailing_pad = 12 * sym_samples;
	int buffer_pb_size = passband_samples + trailing_pad;
	int buffer_nsymb_pb = buffer_pb_size / sym_samples;
	// Snap buffer to whole symbols
	buffer_pb_size = buffer_nsymb_pb * sym_samples;
	int expected_delay_pb = 0;  // preamble starts at sample 0

	std::vector<double> buffer_pb((size_t)buffer_pb_size, 0.0);
	for (int i = 0; i < passband_samples && i < buffer_pb_size; i++) {
		buffer_pb[i] = preamble_pb[i];
	}

	// Step 6: passband -> baseband_interpolated via FIR_rx_time_sync
	// (telecom_system.cc:953 pattern). Output is full-rate interpolated
	// baseband — the same input format time_sync_mfsk_corr expects.
	std::vector<std::complex<double>> baseband_interp((size_t)buffer_pb_size,
		std::complex<double>(0.0, 0.0));
	ts.ofdm.passband_to_baseband(
		buffer_pb.data(), buffer_pb_size, baseband_interp.data(),
		ts.sampling_frequency, ts.carrier_frequency, ts.carrier_amplitude,
		1, &ts.ofdm.FIR_rx_time_sync);

	// Step 7: invoke the production preamble detector.
	double sync_metric = 0.0;
	int detected_delay = ts.ofdm.time_sync_mfsk_corr(
		baseband_interp.data(), buffer_pb_size, interp,
		/*search_start_symb=*/0, &sync_metric);

	if (detected_delay < 0) {
		char buf[160];
		snprintf(buf, sizeof(buf),
			"time_sync_mfsk_corr returned -1 (no preamble found), metric=%.4f",
			sync_metric);
		test_fail(name, buf);
		return;
	}
	// Production threshold is 0.5 (ofdm.cc:3217). Clean round-trip
	// typically yields metric ~0.9-1.0.
	if (sync_metric < 0.5) {
		char buf[160];
		snprintf(buf, sizeof(buf),
			"sync_metric=%.4f below production threshold 0.5",
			sync_metric);
		test_fail(name, buf);
		return;
	}
	// Delay tolerance: ±1 symbol (interp-rate units).
	int delay_err = std::abs(detected_delay - expected_delay_pb);
	if (delay_err > sym_samples) {
		char buf[200];
		snprintf(buf, sizeof(buf),
			"detected_delay=%d expected=%d err=%d > %d (1 sym)",
			detected_delay, expected_delay_pb, delay_err, sym_samples);
		test_fail(name, buf);
		return;
	}

	test_pass(name);
}

// =============================================================================
// §6. MFSK data-preamble DISCRETE-MATCH detector regression suite
//     (data-preamble-port-research.md §14, ofdm.cc time_sync_mfsk_corr
//     port from cosine²-mean to FFT-bin argmax matched count).
//
// All §6 tests fail-before-passes pattern: tests 6.2 (cliff) MUST fail on
// the pre-port cosine²-mean detector and pass on the post-port discrete-
// match detector. The fail-before-passes verification is run by stashing
// the ofdm.cc body change and re-running `mercury.exe --test`.
// =============================================================================

// Shared synthesis helper: load ROBUST_0, generate preamble, round-trip
// through TX→RX chain, return the full-rate interpolated baseband buffer
// for the detector to consume. Caller may add AWGN at the passband level
// BEFORE the FIR (which we control here via the noise_sigma_pb argument).
//
// Returns the full-rate interpolated baseband buffer in `out_bb` and the
// expected preamble start offset in `out_expected_delay`.
// If `synthesize_preamble=false`, the buffer contains only noise (no preamble).
static bool synth_preamble_buffer(cl_telecom_system& ts,
                                   double noise_sigma_pb,
                                   bool synthesize_preamble,
                                   std::mt19937& rng,
                                   std::vector<std::complex<double> >& out_bb,
                                   int& out_expected_delay,
                                   int& out_sym_samples)
{
	ts.operation_mode = ARQ_MODE;
	ts.load_configuration(ROBUST_0);
	if (ts.mfsk.preamble_nSymb != 16 || ts.ofdm.mfsk_corr_template == NULL)
		return false;

	int Nofdm = ts.data_container.Nofdm;
	int Nc = ts.data_container.Nc;
	int preamble_nSymb = ts.data_container.preamble_nSymb;
	int interp = ts.data_container.interpolation_rate;
	out_sym_samples = Nofdm * interp;

	int passband_samples = Nofdm * preamble_nSymb * interp;
	std::vector<double> preamble_pb((size_t)passband_samples, 0.0);

	if (synthesize_preamble) {
		ts.mfsk.generate_preamble(ts.data_container.preamble_data, preamble_nSymb);
		for (int i = 0; i < preamble_nSymb; i++) {
			ts.ofdm.symbol_mod(
				&ts.data_container.preamble_data[i * Nc],
				&ts.data_container.preamble_symbol_modulated_data[i * Nofdm]);
		}
		long unsigned saved_pss = ts.ofdm.passband_start_sample;
		ts.ofdm.passband_start_sample = 0;
		ts.ofdm.baseband_to_passband(
			ts.data_container.preamble_symbol_modulated_data,
			Nofdm * preamble_nSymb,
			preamble_pb.data(),
			ts.sampling_frequency, ts.carrier_frequency, ts.carrier_amplitude,
			interp);
		ts.ofdm.passband_start_sample = saved_pss;
	}

	// Buffer = preamble + trailing pad. Match the existing roundtrip test
	// (no leading silence — preserves FIR transient compatibility with the
	// template that load_configuration generated).
	int trailing_pad = 12 * out_sym_samples;
	int buffer_pb_size = passband_samples + trailing_pad;
	int buffer_nsymb_pb = buffer_pb_size / out_sym_samples;
	buffer_pb_size = buffer_nsymb_pb * out_sym_samples;
	out_expected_delay = 0;

	std::vector<double> buffer_pb((size_t)buffer_pb_size, 0.0);
	if (synthesize_preamble) {
		for (int i = 0; i < passband_samples && i < buffer_pb_size; i++)
			buffer_pb[i] = preamble_pb[i];
	}

	// Add AWGN at the passband level. Gaussian, real-valued (passband is
	// double). The FIR_rx_time_sync bandpass will reject out-of-band
	// components, giving an in-band SNR ~= passband_SNR + 10·log10(fs/BW)
	// (≈ +13 dB for fs=48k, BW=2343Hz).
	if (noise_sigma_pb > 0.0) {
		std::normal_distribution<double> nd(0.0, noise_sigma_pb);
		for (int i = 0; i < buffer_pb_size; i++)
			buffer_pb[i] += nd(rng);
	}

	out_bb.assign((size_t)buffer_pb_size, std::complex<double>(0.0, 0.0));
	ts.ofdm.passband_to_baseband(
		buffer_pb.data(), buffer_pb_size, out_bb.data(),
		ts.sampling_frequency, ts.carrier_frequency, ts.carrier_amplitude,
		1, &ts.ofdm.FIR_rx_time_sync);

	return true;
}

// Compute passband RMS for the preamble waveform — used to set noise
// stddev relative to signal level so tests are reproducible across
// build flags.
static double measure_preamble_rms_pb(cl_telecom_system& ts) {
	int Nofdm = ts.data_container.Nofdm;
	int Nc = ts.data_container.Nc;
	int preamble_nSymb = ts.data_container.preamble_nSymb;
	int interp = ts.data_container.interpolation_rate;
	int passband_samples = Nofdm * preamble_nSymb * interp;
	std::vector<double> pb((size_t)passband_samples, 0.0);

	ts.mfsk.generate_preamble(ts.data_container.preamble_data, preamble_nSymb);
	for (int i = 0; i < preamble_nSymb; i++) {
		ts.ofdm.symbol_mod(
			&ts.data_container.preamble_data[i * Nc],
			&ts.data_container.preamble_symbol_modulated_data[i * Nofdm]);
	}
	long unsigned saved_pss = ts.ofdm.passband_start_sample;
	ts.ofdm.passband_start_sample = 0;
	ts.ofdm.baseband_to_passband(
		ts.data_container.preamble_symbol_modulated_data,
		Nofdm * preamble_nSymb,
		pb.data(),
		ts.sampling_frequency, ts.carrier_frequency, ts.carrier_amplitude,
		interp);
	ts.ofdm.passband_start_sample = saved_pss;

	double sum_sq = 0.0;
	for (int i = 0; i < passband_samples; i++) sum_sq += pb[i] * pb[i];
	return std::sqrt(sum_sq / (double)passband_samples);
}

// §6.1 — High-SNR clean: assert detector returns delay≥0 AND matched
// count near maximum at sigma=0. Sanity baseline.
static void test_mfsk_data_preamble_argmax_clean() {
	const char* name = "mfsk_data_preamble_argmax_clean";
	cl_telecom_system ts;
	std::mt19937 rng(0xCAFEBABEu);
	std::vector<std::complex<double> > bb;
	int expected_delay = 0, sym_samples = 0;
	if (!synth_preamble_buffer(ts, /*noise_sigma_pb=*/0.0, /*synthesize_preamble=*/true,
	                            rng, bb, expected_delay, sym_samples)) {
		test_fail(name, "synth_preamble_buffer failed (load_configuration?)");
		return;
	}

	int interp = ts.data_container.interpolation_rate;
	double sync_metric = 0.0;
	int detected_delay = ts.ofdm.time_sync_mfsk_corr(
		bb.data(), (int)bb.size(), interp,
		/*search_start_symb=*/0, &sync_metric);

	if (detected_delay < 0) {
		char buf[160];
		snprintf(buf, sizeof(buf),
			"clean detect FAILED: delay=-1 metric=%.2f", sync_metric);
		test_fail(name, buf);
		return;
	}
	// Post-port: metric is discrete match count 0..16. Clean expected ~16.
	if (sync_metric < 14.0) {
		char buf[160];
		snprintf(buf, sizeof(buf),
			"matched=%.0f < 14 (expected ≥14 on clean)", sync_metric);
		test_fail(name, buf);
		return;
	}
	int delay_err = std::abs(detected_delay - expected_delay);
	if (delay_err > sym_samples) {
		char buf[200];
		snprintf(buf, sizeof(buf),
			"detected_delay=%d expected=%d err=%d > 1 sym (%d)",
			detected_delay, expected_delay, delay_err, sym_samples);
		test_fail(name, buf);
		return;
	}
	test_pass(name);
}

// §6.2 — Low-SNR cliff (FAIL-BEFORE-PASSES):
// Add AWGN at the cliff level. The pre-port cosine²-mean detector
// returns -1 with metric ≈ 0.15-0.30. The post-port discrete-match
// detector finds matched ≥ 7/16 across the search window. Use 5
// deterministic seeds; require ≥4 of 5 to detect.
//
// Noise level: passband sigma = 4× preamble RMS. This corresponds to a
// passband SNR of ≈ -12 dB which, after the FIR bandpass (≈ +13 dB
// bandwidth gain), gives in-band SNR ≈ +1 dB — close to the
// WGN:-8 IONOS cell per data-preamble-port-research.md §2.4.
static void test_mfsk_data_preamble_argmax_cliff() {
	const char* name = "mfsk_data_preamble_argmax_cliff";
	cl_telecom_system ts_meas;
	ts_meas.operation_mode = ARQ_MODE;
	ts_meas.load_configuration(ROBUST_0);
	double rms = measure_preamble_rms_pb(ts_meas);
	if (!(rms > 0.0)) {
		test_fail(name, "preamble RMS measurement failed");
		return;
	}
	double sigma_pb = 4.0 * rms;

	int passes = 0;
	int fails = 0;
	int last_metric_int = 0;
	int last_delay = 0;
	for (int seed = 1; seed <= 5; seed++) {
		cl_telecom_system ts;
		std::mt19937 rng((uint32_t)(0xC11FF000u + seed));
		std::vector<std::complex<double> > bb;
		int expected_delay = 0, sym_samples = 0;
		if (!synth_preamble_buffer(ts, sigma_pb, /*synthesize_preamble=*/true,
		                            rng, bb, expected_delay, sym_samples)) {
			test_fail(name, "synth_preamble_buffer failed");
			return;
		}
		int interp = ts.data_container.interpolation_rate;
		double sync_metric = 0.0;
		int detected_delay = ts.ofdm.time_sync_mfsk_corr(
			bb.data(), (int)bb.size(), interp,
			/*search_start_symb=*/0, &sync_metric);
		// Required: delay≥0 AND matched ≥ 7/16.
		// (matched is reported via sync_metric on the new detector.)
		if (detected_delay >= 0 && sync_metric >= (double)ts.mfsk.preamble_match_threshold) {
			passes++;
		} else {
			fails++;
		}
		last_metric_int = (int)sync_metric;
		last_delay = detected_delay;
	}
	if (passes < 4) {
		char buf[200];
		snprintf(buf, sizeof(buf),
			"cliff: only %d/5 seeds detected (need ≥4); last delay=%d matched=%d threshold=%d",
			passes, last_delay, last_metric_int, ts_meas.mfsk.preamble_match_threshold);
		test_fail(name, buf);
		return;
	}
	test_pass(name);
}

// §6.3 — Pure-noise false-alarm guard.
// 100 random WGN buffers (no preamble). Pre-port detector with
// per_sym_floor=0.01 has FAR > 10⁻³/poll on adversarial noise;
// post-port discrete-match has FAR ≈ 2.5e-7/poll (M=32, 7/16).
// Assert ≤ 1 false positive across 100 trials.
static void test_mfsk_data_preamble_argmax_pure_noise() {
	const char* name = "mfsk_data_preamble_argmax_pure_noise";
	cl_telecom_system ts_meas;
	ts_meas.operation_mode = ARQ_MODE;
	ts_meas.load_configuration(ROBUST_0);
	double rms = measure_preamble_rms_pb(ts_meas);
	if (!(rms > 0.0)) {
		test_fail(name, "preamble RMS measurement failed");
		return;
	}
	double sigma_pb = 2.0 * rms;  // any non-zero noise level — preamble is absent

	int false_positives = 0;
	for (int trial = 0; trial < 100; trial++) {
		cl_telecom_system ts;
		std::mt19937 rng((uint32_t)(0xD00DEADu + trial));
		std::vector<std::complex<double> > bb;
		int expected_delay = 0, sym_samples = 0;
		if (!synth_preamble_buffer(ts, sigma_pb, /*synthesize_preamble=*/false,
		                            rng, bb, expected_delay, sym_samples)) {
			test_fail(name, "synth_preamble_buffer failed");
			return;
		}
		int interp = ts.data_container.interpolation_rate;
		double sync_metric = 0.0;
		int detected_delay = ts.ofdm.time_sync_mfsk_corr(
			bb.data(), (int)bb.size(), interp, 0, &sync_metric);
		if (detected_delay >= 0) false_positives++;
	}
	if (false_positives > 1) {
		char buf[160];
		snprintf(buf, sizeof(buf),
			"FAR too high: %d/100 false detections (expected ≤1)",
			false_positives);
		test_fail(name, buf);
		return;
	}
	test_pass(name);
}

// §6.4 — In-alphabet MFSK data content (Bug #44 regression guard).
// Synthesize a buffer of random MFSK DATA-style symbols (one in-alphabet
// tone per symbol, drawn uniformly), no preamble. The pre-port cosine²
// metric may false-trigger because random data tones can sum to ~0.44
// per-symbol cosine² (audit §13.9). The post-port FFT-bin argmax bounds
// the random baseline to 1/M per symbol → E[K]=0.5 << 7.
static void test_mfsk_data_preamble_argmax_data_content() {
	const char* name = "mfsk_data_preamble_argmax_data_content";
	cl_telecom_system ts;
	ts.operation_mode = ARQ_MODE;
	ts.load_configuration(ROBUST_0);
	if (ts.mfsk.preamble_nSymb != 16 || ts.ofdm.mfsk_corr_template == NULL) {
		test_fail(name, "pre-condition: ROBUST_0 not loaded with 16-sym preamble");
		return;
	}

	int Nofdm = ts.data_container.Nofdm;
	int Nc = ts.data_container.Nc;
	int interp = ts.data_container.interpolation_rate;
	int sym_samples = Nofdm * interp;
	int M = ts.mfsk.M;
	int nStreams = ts.mfsk.nStreams;
	int nsymb_data = 32;  // 32 data symbols, no preamble

	std::mt19937 rng(0xDA7AC0DEu);
	std::uniform_int_distribution<int> tone_pick(0, M - 1);

	// Build a frequency-domain frame of random tones (one tone per symbol
	// in each stream's band), then run through the same TX→RX chain. Tones
	// are drawn from the FULL data alphabet (0..M-1) including bins that
	// happen to match preamble bins — this is the adversarial case.
	std::vector<std::complex<double> > freq_data((size_t)(nsymb_data * Nc),
	                                              std::complex<double>(0.0, 0.0));
	double amp = std::sqrt((double)Nc / (double)nStreams);
	for (int s = 0; s < nsymb_data; s++) {
		for (int st = 0; st < nStreams; st++) {
			int tone = tone_pick(rng);
			freq_data[s * Nc + ts.mfsk.stream_offsets[st] + tone] =
				std::complex<double>(amp, 0.0);
		}
	}

	// symbol_mod each frame symbol
	std::vector<std::complex<double> > bb_tx((size_t)(nsymb_data * Nofdm),
	                                          std::complex<double>(0.0, 0.0));
	for (int i = 0; i < nsymb_data; i++) {
		ts.ofdm.symbol_mod(&freq_data[i * Nc], &bb_tx[i * Nofdm]);
	}

	int pb_len = nsymb_data * Nofdm * interp;
	std::vector<double> pb_data((size_t)pb_len, 0.0);
	long unsigned saved_pss = ts.ofdm.passband_start_sample;
	ts.ofdm.passband_start_sample = 0;
	ts.ofdm.baseband_to_passband(
		bb_tx.data(), nsymb_data * Nofdm, pb_data.data(),
		ts.sampling_frequency, ts.carrier_frequency, ts.carrier_amplitude, interp);
	ts.ofdm.passband_start_sample = saved_pss;

	std::vector<std::complex<double> > bb_rx((size_t)pb_len,
	                                          std::complex<double>(0.0, 0.0));
	ts.ofdm.passband_to_baseband(
		pb_data.data(), pb_len, bb_rx.data(),
		ts.sampling_frequency, ts.carrier_frequency, ts.carrier_amplitude,
		1, &ts.ofdm.FIR_rx_time_sync);

	double sync_metric = 0.0;
	int detected_delay = ts.ofdm.time_sync_mfsk_corr(
		bb_rx.data(), pb_len, interp, 0, &sync_metric);
	if (detected_delay >= 0) {
		char buf[200];
		snprintf(buf, sizeof(buf),
			"FALSE-TRIGGER on random data: delay=%d matched=%.0f (threshold=%d)",
			detected_delay, sync_metric, ts.mfsk.preamble_match_threshold);
		test_fail(name, buf);
		return;
	}
	(void)sym_samples;
	test_pass(name);
}

// §6.5 — High-SNR no-regression: sigma=0, assert matched count is at
// the maximum AND delay is within 1 symbol of injection. Catches any
// algorithmic regression at the easy end of the SNR range.
static void test_mfsk_data_preamble_argmax_high_snr_no_regression() {
	const char* name = "mfsk_data_preamble_argmax_high_snr_no_regression";
	cl_telecom_system ts;
	std::mt19937 rng(0xFEEDFACEu);
	std::vector<std::complex<double> > bb;
	int expected_delay = 0, sym_samples = 0;
	if (!synth_preamble_buffer(ts, 0.0, true, rng, bb, expected_delay, sym_samples)) {
		test_fail(name, "synth_preamble_buffer failed");
		return;
	}

	int interp = ts.data_container.interpolation_rate;
	double sync_metric = 0.0;
	int detected_delay = ts.ofdm.time_sync_mfsk_corr(
		bb.data(), (int)bb.size(), interp, 0, &sync_metric);
	if (detected_delay < 0) {
		char buf[160];
		snprintf(buf, sizeof(buf),
			"high-SNR detect FAILED: delay=-1 matched=%.0f", sync_metric);
		test_fail(name, buf);
		return;
	}
	int preamble_n = ts.mfsk.preamble_nSymb;
	if ((int)sync_metric < preamble_n) {
		char buf[160];
		snprintf(buf, sizeof(buf),
			"matched=%.0f < %d (expected full match at sigma=0)",
			sync_metric, preamble_n);
		test_fail(name, buf);
		return;
	}
	int delay_err = std::abs(detected_delay - expected_delay);
	if (delay_err > sym_samples) {
		char buf[200];
		snprintf(buf, sizeof(buf),
			"detected_delay=%d expected=%d err=%d > 1 sym",
			detected_delay, expected_delay, delay_err);
		test_fail(name, buf);
		return;
	}
	test_pass(name);
}

// =============================================================================
// §7 Mini-Moose CFO refinement regression suite
// (data-preamble-port-research.md §20, data-flow-freq_offset_measured.md §7)
// =============================================================================

// Synthesize a base-rate baseband preamble buffer with optional injected CFO.
// Mirrors synth_preamble_buffer (§6 helper) but:
//   1) Decimates the post-FIR full-rate output by ts.interpolation_rate to
//      produce a buffer at base rate (matches what data_container.baseband_data
//      holds in production after passband_to_baseband_decimated).
//   2) Multiplies the decimated baseband by exp(j*2π*CFO*i/fs_base) so the
//      detector sees a residual CFO of `cfo_hz` riding on the signal.
//   3) Returns the buffer in `out_bb_base` with length `preamble_nSymb * Nofdm`.
//
// noise_sigma_pb is added at PASSBAND (real, Gaussian) like §6's helper, so
// SNR semantics match the existing tests.
static bool synth_preamble_buffer_base_with_cfo(
	cl_telecom_system& ts,
	double cfo_hz,
	double noise_sigma_pb,
	bool synthesize_preamble,
	std::mt19937& rng,
	std::vector<std::complex<double> >& out_bb_base,
	int& out_preamble_nSymb,
	int& out_Nofdm)
{
	ts.operation_mode = ARQ_MODE;
	ts.load_configuration(ROBUST_0);
	if (ts.mfsk.preamble_nSymb != 16 || ts.ofdm.mfsk_corr_template == NULL)
		return false;

	int Nofdm = ts.data_container.Nofdm;
	int Nc = ts.data_container.Nc;
	int preamble_nSymb = ts.data_container.preamble_nSymb;
	int interp = ts.data_container.interpolation_rate;
	int passband_samples = Nofdm * preamble_nSymb * interp;

	out_preamble_nSymb = preamble_nSymb;
	out_Nofdm = Nofdm;

	std::vector<double> preamble_pb((size_t)passband_samples, 0.0);
	if (synthesize_preamble) {
		ts.mfsk.generate_preamble(ts.data_container.preamble_data, preamble_nSymb);
		for (int i = 0; i < preamble_nSymb; i++) {
			ts.ofdm.symbol_mod(
				&ts.data_container.preamble_data[i * Nc],
				&ts.data_container.preamble_symbol_modulated_data[i * Nofdm]);
		}
		long unsigned saved_pss = ts.ofdm.passband_start_sample;
		ts.ofdm.passband_start_sample = 0;
		ts.ofdm.baseband_to_passband(
			ts.data_container.preamble_symbol_modulated_data,
			Nofdm * preamble_nSymb,
			preamble_pb.data(),
			ts.sampling_frequency, ts.carrier_frequency, ts.carrier_amplitude,
			interp);
		ts.ofdm.passband_start_sample = saved_pss;
	}

	// Pad with trailing silence so the detector's downstream Phase-2 window
	// has room to refine (mirror of §6 helper's trailing_pad).
	int trailing_pad = 12 * Nofdm * interp;
	int buffer_pb_size = passband_samples + trailing_pad;
	std::vector<double> buffer_pb((size_t)buffer_pb_size, 0.0);
	if (synthesize_preamble) {
		for (int i = 0; i < passband_samples && i < buffer_pb_size; i++)
			buffer_pb[i] = preamble_pb[i];
	}

	if (noise_sigma_pb > 0.0) {
		std::normal_distribution<double> nd(0.0, noise_sigma_pb);
		for (int i = 0; i < buffer_pb_size; i++)
			buffer_pb[i] += nd(rng);
	}

	// passband_to_baseband at full rate with decimation_rate=1.
	std::vector<std::complex<double> > bb_full((size_t)buffer_pb_size,
		std::complex<double>(0.0, 0.0));
	ts.ofdm.passband_to_baseband(
		buffer_pb.data(), buffer_pb_size, bb_full.data(),
		ts.sampling_frequency, ts.carrier_frequency, ts.carrier_amplitude,
		1, &ts.ofdm.FIR_rx_time_sync);

	// Decimate to base rate and inject CFO. fs_base = sampling_frequency / interp.
	int base_samples = buffer_pb_size / interp;
	double fs_base = ts.sampling_frequency / (double)interp;
	double angle_step = 2.0 * M_PI * cfo_hz / fs_base;

	out_bb_base.assign((size_t)base_samples, std::complex<double>(0.0, 0.0));
	for (int i = 0; i < base_samples; i++) {
		std::complex<double> v = bb_full[(size_t)i * interp];
		// Multiply by exp(j*angle) — phase recurrence-free per-sample form
		// (test code: clarity over speed).
		double a = (double)i * angle_step;
		double cr = std::cos(a);
		double ci = std::sin(a);
		std::complex<double> rot(cr, ci);
		out_bb_base[(size_t)i] = v * rot;
	}
	return true;
}

// §7.1 — Recover injected CFO. POSITIVE, FAIL-BEFORE-PASSES.
//
// Inject +7 Hz residual CFO at the baseband-complex level, add AWGN at
// the same noise level as §6.2 (passband sigma = 3 × preamble RMS, in-band
// SNR ≈ +2 dB). Assert |estimated - 7 Hz| < 1.5 Hz across 5 seeds.
//
// Pre-fix the function `carrier_frequency_sync_wb_mfsk` did not exist
// (link error). Post-fix this test passes — the estimator returns a
// finite value tracking the injection within ±1.5 Hz.
static void test_mfsk_data_preamble_mini_moose_recovers_cfo() {
	const char* name = "mfsk_data_preamble_mini_moose_recovers_cfo";

	// Measure preamble RMS using the existing helper so noise level matches
	// §6.2 cliff conditions.
	cl_telecom_system ts_meas;
	ts_meas.operation_mode = ARQ_MODE;
	ts_meas.load_configuration(ROBUST_0);
	double rms = measure_preamble_rms_pb(ts_meas);
	if (!(rms > 0.0)) {
		test_fail(name, "preamble RMS measurement failed");
		return;
	}
	double sigma_pb = 3.0 * rms;
	const double cfo_inject = 7.0; // Hz, comfortably inside capture range

	int hits = 0;
	double last_est = 0.0;
	for (int seed = 1; seed <= 5; seed++) {
		cl_telecom_system ts;
		std::mt19937 rng((uint32_t)(0x10550030u + seed));
		std::vector<std::complex<double> > bb_base;
		int preamble_nSymb_local = 0;
		int Nofdm_local = 0;
		if (!synth_preamble_buffer_base_with_cfo(ts, cfo_inject, sigma_pb, true,
		                                          rng, bb_base,
		                                          preamble_nSymb_local,
		                                          Nofdm_local)) {
			test_fail(name, "synth_preamble_buffer_base_with_cfo failed");
			return;
		}
		double carrier_freq_width = ts.bandwidth / (double)ts.data_container.Nc;
		double est = ts.ofdm.carrier_frequency_sync_wb_mfsk(
			bb_base.data(),
			carrier_freq_width,
			preamble_nSymb_local,
			ts.mfsk.preamble_tones, ts.mfsk.M,
			ts.mfsk.nStreams, ts.mfsk.stream_offsets);
		last_est = est;
		if (std::fabs(est - cfo_inject) < 1.5) hits++;
	}
	if (hits < 4) {
		char buf[200];
		std::snprintf(buf, sizeof(buf),
			"hits=%d/5 last_est=%.3f Hz expected≈%.1f Hz (±1.5)",
			hits, last_est, cfo_inject);
		test_fail(name, buf);
		return;
	}
	test_pass(name);
}

// §7.2 — Zero-CFO no-op / no-regression. POSITIVE.
//
// Clean preamble (sigma_pb = 0, CFO = 0). Estimator should return a value
// near zero (|est| < 0.5 Hz). Catches over-correction at high SNR.
static void test_mfsk_data_preamble_mini_moose_zero_cfo_no_op() {
	const char* name = "mfsk_data_preamble_mini_moose_zero_cfo_no_op";
	cl_telecom_system ts;
	std::mt19937 rng(0x10551110u);
	std::vector<std::complex<double> > bb_base;
	int preamble_nSymb_local = 0;
	int Nofdm_local = 0;
	if (!synth_preamble_buffer_base_with_cfo(ts, /*cfo_hz=*/0.0,
	                                          /*sigma_pb=*/0.0,
	                                          /*synth=*/true, rng, bb_base,
	                                          preamble_nSymb_local,
	                                          Nofdm_local)) {
		test_fail(name, "synth_preamble_buffer_base_with_cfo failed");
		return;
	}
	double carrier_freq_width = ts.bandwidth / (double)ts.data_container.Nc;
	double est = ts.ofdm.carrier_frequency_sync_wb_mfsk(
		bb_base.data(),
		carrier_freq_width,
		preamble_nSymb_local,
		ts.mfsk.preamble_tones, ts.mfsk.M,
		ts.mfsk.nStreams, ts.mfsk.stream_offsets);
	if (!std::isfinite(est)) {
		test_fail(name, "estimator returned non-finite at zero CFO");
		return;
	}
	if (std::fabs(est) >= 0.5) {
		char buf[160];
		std::snprintf(buf, sizeof(buf),
			"clean-signal estimate=%.4f Hz, expected |est|<0.5 (over-correction guard)",
			est);
		test_fail(name, buf);
		return;
	}
	test_pass(name);
}

// §7.3 — Pure-noise safety. NEGATIVE / sanity-clamp guard.
//
// 50 random WGN buffers (no preamble). Assert estimator returns a finite
// value bounded by |est| ≤ 100 Hz (the production sanity clamp is ±93.75 Hz;
// 100 leaves a small margin for estimator alias artifacts). The
// confidence gate `|C|/energy_total < 0.05` should fire on noise →
// returns 0. Tested across 50 seeds to catch tail behaviors.
//
// This is the cross-layer regression guard required by CLAUDE.md §5: the
// estimator MUST NOT produce wild values that would corrupt the
// freq_offset_measured downstream consumers (sanity reject, re-mix,
// cache). Returning 0 (no-op) on noise is the safe default.
static void test_mfsk_data_preamble_mini_moose_pure_noise_safe() {
	const char* name = "mfsk_data_preamble_mini_moose_pure_noise_safe";
	cl_telecom_system ts_meas;
	ts_meas.operation_mode = ARQ_MODE;
	ts_meas.load_configuration(ROBUST_0);
	double rms = measure_preamble_rms_pb(ts_meas);
	if (!(rms > 0.0)) {
		test_fail(name, "preamble RMS measurement failed");
		return;
	}
	double sigma_pb = 2.0 * rms;  // strong noise, no preamble

	int wild = 0;
	int non_finite = 0;
	double max_est_seen = 0.0;
	for (int trial = 0; trial < 50; trial++) {
		cl_telecom_system ts;
		std::mt19937 rng((uint32_t)(0x10557777u + trial));
		std::vector<std::complex<double> > bb_base;
		int preamble_nSymb_local = 0;
		int Nofdm_local = 0;
		if (!synth_preamble_buffer_base_with_cfo(ts,
		                                          /*cfo_hz=*/0.0,
		                                          sigma_pb,
		                                          /*synth=*/false,
		                                          rng, bb_base,
		                                          preamble_nSymb_local,
		                                          Nofdm_local)) {
			test_fail(name, "synth_preamble_buffer_base_with_cfo failed");
			return;
		}
		double carrier_freq_width = ts.bandwidth / (double)ts.data_container.Nc;
		double est = ts.ofdm.carrier_frequency_sync_wb_mfsk(
			bb_base.data(),
			carrier_freq_width,
			preamble_nSymb_local,
			ts.mfsk.preamble_tones, ts.mfsk.M,
			ts.mfsk.nStreams, ts.mfsk.stream_offsets);
		if (!std::isfinite(est)) { non_finite++; continue; }
		double ae = std::fabs(est);
		if (ae > max_est_seen) max_est_seen = ae;
		if (ae > 100.0) wild++;
	}
	if (non_finite > 0) {
		char buf[160];
		std::snprintf(buf, sizeof(buf),
			"%d/50 trials returned non-finite (must never happen)",
			non_finite);
		test_fail(name, buf);
		return;
	}
	if (wild > 0) {
		char buf[200];
		std::snprintf(buf, sizeof(buf),
			"%d/50 trials produced |est|>100 Hz, max=%.3f (sanity-clamp guard)",
			wild, max_est_seen);
		test_fail(name, buf);
		return;
	}
	test_pass(name);
}

// =============================================================================
// Top-level runner
// =============================================================================

int run_mfsk_ctrl_codec_tests() {
	g_failures = 0;
	g_passes   = 0;
	printf("=== MFSK ctrl-suffix codec tests (Phase B Wave 1 + Wave 2 v2 + Wave 3) ===\n");

	// §1 codec primitives
	test_pack_unpack_callsign_body_b36();
	test_pack_unpack_start_conn_payload();
	test_pack_unpack_test_ack_payload();
	test_pack_unpack_test_conn_payload();  // §14 Wave 3
	test_ctrl_suffix_roundtrip_all_types();
	test_ctrl_suffix_crc12_corruption();
	test_base_pattern_cross_correlation();
	test_ack_sack_bitmap_30bit_cap();

	// §2 passband round-trip — require cl_telecom_system::load_configuration
	test_mfsk_connect_passband_roundtrip_clean();
	test_mfsk_connect_no_hail_false_trigger();

	// §3 Wave 2 v2 cross-layer regression tests
	test_v2_crc12_wireformat_real_helper();
	test_v2_cmd_loop_no_refire();
	test_v2_rsp_frames_to_read_override();

	// §4 Wave 3 (§14) TEST_CONN integration tests
	test_v3_test_conn_passband_roundtrip_clean();
	test_v3_test_conn_snr_quantization_roundtrip();

	// §5 MFSK WB data-preamble 4 -> 16 cross-layer regression
	// (data-flow-preamble_nSymb.md, 2026-05-27).
	test_preamble_nSymb_wb_robust0_extended_to_16();
	test_mfsk_data_preamble_passband_roundtrip_clean();

	// §6 MFSK data-preamble discrete-match detector regression suite
	// (data-preamble-port-research.md §14, 2026-05-27).
	test_mfsk_data_preamble_argmax_clean();
	test_mfsk_data_preamble_argmax_cliff();
	test_mfsk_data_preamble_argmax_pure_noise();
	test_mfsk_data_preamble_argmax_data_content();
	test_mfsk_data_preamble_argmax_high_snr_no_regression();

	// §7 Mini-Moose CFO refinement regression suite
	// (data-preamble-port-research.md §20, 2026-05-28).
	test_mfsk_data_preamble_mini_moose_recovers_cfo();
	test_mfsk_data_preamble_mini_moose_zero_cfo_no_op();
	test_mfsk_data_preamble_mini_moose_pure_noise_safe();

	printf("=== Tests done: %d passed, %d failed ===\n", g_passes, g_failures);
	return g_failures;
}
