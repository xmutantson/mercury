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

	// MFSK preamble: known tone indices for time sync
	static const int MAX_PREAMBLE_SYMB = 8;
	int preamble_tones[MAX_PREAMBLE_SYMB]; // Known tone indices per preamble symbol
	int preamble_nSymb;                     // Number of preamble symbols used

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

	// ACK+SACK suffix: extra symbols carrying [batch_seq_id(8) | bitmap(32)]
	// = 40 bits payload, replacing OFDM_ACK_CLEAN and SACK_RSP. Each symbol's
	// tone position carries log2(M) bits. WB M=16 → 4 bits/symbol → 10 symbols
	// for 40 bits. NB M=8 → 3 bits/symbol; deferred (NB uses OFDM path for now).
	// Total pattern wall-clock: 16 base + 10 suffix = 26 symbols ≈ 631 ms (WB).
	int ack_sack_suffix_len() const { return (M >= 16) ? 10 : 0; }  // 0 = unsupported
	int ack_sack_pattern_nsymb() const { return ack_pattern_nsymb + ack_sack_suffix_len(); }
	// Pack 40-bit payload [bsi:8|bitmap:32] into N tone values (each in 0..M-1).
	// Returns number of tones written (= ack_sack_suffix_len). Caller provides
	// out_tones[] of size >= ack_sack_suffix_len.
	int pack_ack_sack_payload(uint8_t bsi, uint32_t bitmap, int* out_tones) const;
	// Inverse of pack: reconstruct (bsi, bitmap) from N tones.
	// Returns true on success, false if M unsupported.
	bool unpack_ack_sack_payload(const int* in_tones, uint8_t* out_bsi, uint32_t* out_bitmap) const;
	// Generate ACK pattern + ack_sack_suffix_len() suffix symbols.
	void generate_ack_sack_pattern(std::complex<double>* pattern_out,
	                               uint8_t bsi, uint32_t bitmap);

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

	// Decode the most recently captured SACK suffix into (bsi, bitmap).
	// Returns true on success — requires WB (M>=16) and a prior detector
	// hit that populated last_ack_sack_suffix_tones[]. Caller is expected
	// to clear last_ack_sack_capture_valid when consumed.
	bool decode_ack_sack_from_last_capture(uint8_t* out_bsi, uint32_t* out_bitmap);

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
