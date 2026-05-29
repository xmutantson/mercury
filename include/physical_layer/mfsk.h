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

#ifndef INC_MFSK_H_
#define INC_MFSK_H_

#include <complex>
#include <cmath>
#include <cstdint>

#include "physical_layer/mfsk_ctrl_codec.h"  // mfsk_ctrl_frame_type enum

class cl_ldpc;

#define MOD_MFSK 200

class cl_mfsk
{
private:

public:
	int M;           // Number of tones per stream (e.g., 16 or 32)
	int nBits;       // log2(M) = bits per stream per symbol
	int Nc;          // Total subcarriers in OFDM frame (typically 50)
	int nStreams;    // Parallel MFSK streams (1=ROBUST_0, 2=ROBUST_1/ROBUST_2)
	int tone_hop_step; // Tone hopping step for frequency diversity (coprime with M)

	static const int MAX_STREAMS = 4;
	int stream_offsets[MAX_STREAMS]; // Starting subcarrier bin for each stream

	// MFSK preamble: known tone indices for time sync.
	// WB data preamble extended 4 -> 16 symbols on 2026-05-27 (data-flow-
	// preamble_nSymb.md §H1) for +6 dB matched-filter integration gain at
	// the WGN:-8 cliff. WB further extended 16 -> 32 symbols on 2026-05-28
	// (data-flow-preamble_nSymb.md §11) for +1.5-3 dB additional cliff push
	// via doubled matched-filter integration length. NB preamble (M=8 / M=4)
	// stays at 8 symbols. MAX_PREAMBLE_SYMB must be >= max(preamble_nSymb
	// across all configs) and >= mfsk_corr_template_sym_energy[] size in
	// ofdm.h and >= mfsk_preamble_tones[] mirror in ofdm.h.
	static const int MAX_PREAMBLE_SYMB = 32;
	int preamble_tones[MAX_PREAMBLE_SYMB]; // Known tone indices per preamble symbol
	int preamble_nSymb;                     // Number of preamble symbols used
	// Length-scaled detection threshold for the discrete-match preamble
	// detector (`time_sync_mfsk_corr` ofdm.cc, post-2026-05-27 port per
	// fact-documents/data-preamble-port-research.md §14). Number of
	// per-symbol FFT-bin-argmax matches required to declare detection.
	// Mirrors ack_match_threshold / connect_match_threshold.
	// FAR at WB N=32 T=14 (mirror-bin baseline p=2/M):
	//   M=32: 2.22e-9/poll, M=16: 1.16e-5/poll. See §11.5.
	int preamble_match_threshold;

	// ACK/BREAK/HAIL/SACK pattern: known tone sequences for pattern-based signaling.
	// WB (M>=16): 8 Welch-Costas tones × 2 reps = 16 symbols, with tone hopping.
	// NB (M<=8): 32/48-element Sidelnikov sequences, no repetition, no hopping.
	// ACK = data acknowledged, BREAK = emergency downshift, HAIL = "I am Mercury" beacon.
	// Step 15: legacy SACK pattern (selective-ACK with bitmap suffix) removed —
	// OFDM SACK_RSP is the only SACK transport now.
	static const int MAX_ACK_TONES = 48;  // Max for M=4 NB (48 symbols)
	int ack_tones[MAX_ACK_TONES];
	int break_tones[MAX_ACK_TONES];
	int hail_tones[MAX_ACK_TONES];
	// CONNECT base pattern (Phase B Wave 1): Welch-Costas with primitive root
	// g=3 (distinct from ACK g=5, BREAK g=7, HAIL g=6). 8 base tones × 2
	// reps = 16 symbols (WB-only — NB CONNECT remains LDPC). The detector
	// reuses `ofdm.detect_ack_pattern(connect_tones, ...)`; tone hopping +
	// match threshold mirror ack_tones for shared decoder plumbing. See
	// fact-documents/phase-b-mfsk-connect-research.md §11.4.
	int connect_tones[MAX_ACK_TONES];
	int connect_pattern_nsymb;
	int connect_match_threshold;
	int ack_pattern_len;    // Base tone sequence length (8 for WB, 32/48 for NB)
	int ack_pattern_nsymb;  // Total symbols transmitted (16 for WB, 32/48 for NB)
	int ack_match_threshold;   // Min matched symbols for ACK detection
	int break_match_threshold; // Min matched symbols for BREAK detection
	int hail_match_threshold;  // Min matched symbols for undirected HAIL detection
	// Phase-2 validation: --wb-match-threshold-bias=N added to ack/break/hail
	// match thresholds for M=16 and M=32 (the WB cases). Default 0 = HEAD.
	// Pass +1 to revert b806b76+7076a4b's 8→7 reductions. Applied at end of
	// cl_mfsk::init() so it stacks with the computed defaults.
	int wb_match_threshold_bias;

	// Directed HAIL: 4-tone CRC suffix appended after the "I am Mercury" prefix.
	// Derived from FNV-1a hash of the target callsign (including SSID).
	// Only stations matching the suffix respond, preventing multi-station collisions.
	static const int HAIL_SUFFIX_LEN = 4;
	int hail_suffix[HAIL_SUFFIX_LEN];          // CRC-derived suffix tones
	bool hail_directed;                          // true when suffix is active
	int hail_detect_tones[MAX_ACK_TONES + HAIL_SUFFIX_LEN]; // flat expanded array for detection
	int hail_detect_nsymb;                       // total symbols (base + suffix when directed)
	int hail_detect_threshold;                   // adjusted threshold

	void set_hail_target(const char* callsign, int len);
	void clear_hail_target();

	// Step 15: legacy SACK bitmap-suffix helpers (sack_bitmap_nsuffix,
	// sack_total_nsymb, encode_sack_bitmap, decode_sack_bitmap,
	// generate_sack_pattern, generate_sack_bitmap_pattern, MAX_SACK_BITMAP_SYMBOLS)
	// removed — OFDM SACK_RSP replaces this entire path.

	// SNR suffix for turboshift ACK: 8 extra symbols encoding quantized SNR.
	// WB (M=16): tone 0-15 → SNR = tone*2 - 5 dB (range -5 to +25 dB, 2 dB step)
	// NB (M=8):  tone 0-7  → SNR = tone*2 - 9 dB (range -9 to +5 dB, 2 dB step)
	// All 8 symbols carry the same tone (majority vote on decode, need 3/8).
	static const int SNR_SUFFIX_LEN = 8;
	int snr_to_tone(float snr) const;
	float tone_to_snr(int tone) const;
	void generate_ack_snr_pattern(std::complex<double>* pattern_out, float snr);
	// Total symbols when SNR suffix is active
	int ack_snr_pattern_nsymb() const { return ack_pattern_nsymb + SNR_SUFFIX_LEN; }

	// MFSK control suffix: 13-symbol suffix at M=16 carrying
	//   [type:2 | payload:38 | crc12:12] = 52 bits.
	// Phase B Wave 1 (fact-doc §11) added the 2-bit type field — flag-day
	// break with pre-2026-05-26 deployed peers. Type values per
	// `mfsk_ctrl_frame_type`. CRC12 protects against false-accept after
	// pattern correlator lock (mercury/fact-documents/mfsk-robust-ack.md §3.2).
	//
	// Suffix length: NB M=8 returns 0 (deferred). WB M=16 → 4 bits/symbol
	// → 13 symbols for 52 bits. Total ACK pattern wall-clock: 16 base +
	// 13 suffix = 29 symbols ≈ 705 ms (WB).
	int ack_sack_suffix_len() const { return (M >= 16) ? 13 : 0; }  // 0 = unsupported
	int ack_sack_pattern_nsymb() const { return ack_pattern_nsymb + ack_sack_suffix_len(); }
	// Generic ctrl-suffix codec (52-bit [type:2|payload:38|crc12:12]):
	int pack_ctrl_suffix(mfsk_ctrl_frame_type type, uint64_t payload38,
	                     uint16_t crc12, int* out_tones) const;
	bool unpack_ctrl_suffix(const int* in_tones,
	                        mfsk_ctrl_frame_type* out_type,
	                        uint64_t* out_payload38,
	                        uint16_t* out_crc12) const;
	// Backward-named ACK+SACK wrappers — delegate to pack/unpack_ctrl_suffix
	// with type=MFSK_CTRL_ACK_SACK. The bitmap field is now 30 bits (down
	// from 32 in pre-2026-05-26 deployments); bits 30/31 are silently
	// dropped with a stderr warning. data_batch_size <= 30 is the new
	// invariant — verified at the producer side in arq_responder.cc.
	int pack_ack_sack_payload(uint8_t bsi, uint32_t bitmap, uint16_t crc12,
	                          int* out_tones) const;
	bool unpack_ack_sack_payload(const int* in_tones, uint8_t* out_bsi,
	                             uint32_t* out_bitmap, uint16_t* out_crc12) const;
	// Generate ACK pattern + ack_sack_suffix_len() suffix symbols.
	// Caller passes a pre-computed crc12 (12-bit CRC over [type|bsi||bitmap]
	// packed as 5 bytes [type<<6|bsi>>2, (bsi<<6)|(bitmap>>24), ...]; or
	// equivalently CRC12 over the full 40-bit [type:2|bsi:8|bitmap:30]
	// big-endian MSB-justified). For compatibility with the existing
	// Wave-1 ARQ callers (which compute CRC12 over [bsi||bitmap] = 5 bytes),
	// see mercury/fact-documents/phase-b-mfsk-connect-research.md §11.3.
	void generate_ack_sack_pattern(std::complex<double>* pattern_out,
	                               uint8_t bsi, uint32_t bitmap,
	                               uint16_t crc12);
	// Generate CONNECT base pattern + 13-symbol ctrl-suffix carrying
	// (type, payload, crc12). The base pattern uses connect_tones (NOT
	// ack_tones) so the detector can distinguish CONNECT from ACK+SACK.
	// Caller supplies crc12 (computed over the packed [type:2|payload:38]).
	void generate_ctrl_suffix_pattern(std::complex<double>* pattern_out,
	                                  mfsk_ctrl_frame_type type,
	                                  uint64_t payload38, uint16_t crc12);

	// RX-side capture buffer populated by the ACK detector hook
	// (cl_telecom_system::detect_ack_snr_from_passband). Each entry is the
	// de-hopped payload tone (0..M-1) for the corresponding SACK suffix
	// symbol — i.e. the inverse of the (payload+abs_s*hop)%M mapping the
	// transmitter applies in generate_ack_sack_pattern(). When the detector
	// declares an ACK match it writes ack_sack_suffix_len() entries here
	// (10 for WB M=16) and sets last_ack_sack_capture_valid=true. Size 16
	// is the max possible suffix length.
	static const int MAX_ACK_SACK_SUFFIX = 16;
	int  last_ack_sack_suffix_tones[MAX_ACK_SACK_SUFFIX];
	bool last_ack_sack_capture_valid;

	// CONNECT-suffix capture (separate from ACK+SACK so the two detector
	// windows can coexist without aliasing).
	int  last_connect_suffix_tones[MAX_ACK_SACK_SUFFIX];
	bool last_connect_capture_valid;

	// Decode the most-recent CONNECT-suffix capture into (type, payload,
	// crc12). Returns true on success — requires WB (M>=16) and a prior
	// CONNECT-pattern hit that populated last_connect_suffix_tones[].
	// Caller verifies crc12 separately by recomputing CRC12 over
	// [type:2|payload:38] packed as 5 bytes.
	bool decode_ctrl_suffix_from_last_capture(mfsk_ctrl_frame_type* out_type,
	                                          uint64_t* out_payload38,
	                                          uint16_t* out_crc12);

	// Test-only: stuff CONNECT-suffix payload tones directly into the
	// capture buffer (bypasses RF). Mirror of test_inject_ack_sack_capture.
	void test_inject_connect_capture(const int* tones, int count);

	// Decode the most recently captured SACK suffix into (bsi, bitmap, crc12).
	// Returns true on success — requires WB (M>=16) and a prior detector
	// hit that populated last_ack_sack_suffix_tones[]. Caller is expected
	// to clear last_ack_sack_capture_valid when consumed AND to verify
	// that the returned crc12 matches a freshly-computed CRC12 over
	// [bsi || bitmap]; this function performs the bit-level unpack only.
	bool decode_ack_sack_from_last_capture(uint8_t* out_bsi, uint32_t* out_bitmap,
	                                       uint16_t* out_crc12);

	// Test-only: stuff payload tones directly into the capture buffer
	// (bypasses the RF capture path). Used by symbol-domain round-trip
	// tests so we can exercise decode_ack_sack_from_last_capture() without
	// running passband_to_baseband + FFT. Not for production code paths.
	void test_inject_ack_sack_capture(const int* tones, int count);

	cl_mfsk();
	~cl_mfsk();

	void init(int _M, int _Nc, int _nStreams = 1);
	void deinit();

	// Effective bits per symbol period (nBits * nStreams)
	int bits_per_symbol() const { return nBits * nStreams; }

	// Generate MFSK preamble data (tones in all streams simultaneously)
	// preamble_out: nSymb * Nc complex values
	void generate_preamble(std::complex<double>* preamble_out, int nSymb);

	// Generate ACK pattern: ack_pattern_nsymb symbols of known tones
	// pattern_out: ack_pattern_nsymb * Nc complex values
	void generate_ack_pattern(std::complex<double>* pattern_out);

	// Generate BREAK pattern: same structure as ACK but with break_tones
	void generate_break_pattern(std::complex<double>* pattern_out);

	// Generate HAIL pattern: "I am Mercury" beacon, same structure as ACK but with hail_tones
	void generate_hail_pattern(std::complex<double>* pattern_out);

	// Generate CONNECT base pattern: same structure as ACK but with
	// connect_tones (Phase B Wave 1). Only the base 16 symbols are
	// written — the per-frame 13-tone ctrl-suffix is written by
	// generate_ctrl_suffix_pattern() above.
	void generate_connect_pattern(std::complex<double>* pattern_out);

	// TX: Map bits to one-hot subcarrier vectors across all streams
	// Consumes bits_per_symbol() bits per symbol period
	void mod(const int* bits_in, int total_bits,
	         std::complex<double>* symbols_out);

	// RX: Non-coherent energy detection across all streams -> soft LLRs
	// Produces bits_per_symbol() LLRs per symbol period
	void demod(const std::complex<double>* fft_in, int total_bits,
	           float* llr_out);
};

#endif
