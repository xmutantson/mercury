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

#include "physical_layer/golay24.h"

#include <cstring>

// =============================================================================
// Golay(24,12,8) — fixed systematic generator
// =============================================================================
//
// G = [ I_12 | B ]. B is the systematic parity submatrix of the [24,12,8]
// extended binary Golay code (the same code class used by the MIL-STD-188-141A
// ALE FEC, Johnson 1991; en.wikipedia.org/wiki/Binary_Golay_code).
//
// These 12 rows were DERIVED, not hand-typed: a lexicode construction
// (Wikipedia "Lexicographic code") generates the 4096-word span with proven
// d_min=8, then Gaussian elimination over all 24 coordinates picks an
// information set and emits the systematic [I|B]. encode() over this B was
// verified to have d_min == 8 over all 4096 codewords (the generator probe;
// the golay24_roundtrip unit test re-confirms d_min==8 + 3-error correction at
// runtime, so a regression in these constants FAILS LOUDLY).
//
// Each row is 12 bits, bit 11 = leftmost (col 0). parity_j = parity(info AND
// GOLAY_B[j]). The information set is a permutation of the natural Golay
// coordinates, so B is NOT the symmetric icosahedron-complement form — but the
// code is fully equivalent (d_min=8). The decoder below is exhaustive
// min-distance (it does not rely on any B symmetry).
static const uint16_t GOLAY_B[12] = {
	0xFA4, // 111110100100
	0xE4E, // 111001001110
	0xD1D, // 110100011101
	0xCF8, // 110011111000
	0xB3A, // 101100111010
	0xAE3, // 101011100011
	0x9D6, // 100111010110
	0x769, // 011101101001
	0x6D5, // 011011010101
	0x5B3, // 010110110011
	0x38F, // 001110001111
	0x07F, // 000001111111
};

// 12-bit population parity (XOR of bits).
static inline int popparity12(uint32_t x) {
	x &= 0xFFFu;
	x ^= x >> 8;
	x ^= x >> 4;
	x ^= x >> 2;
	x ^= x >> 1;
	return (int)(x & 1u);
}

static inline int popcount32(uint32_t x) {
	int c = 0;
	while (x) { x &= (x - 1); c++; }
	return c;
}

// --- Encode -------------------------------------------------------------------
// codeword = [ info(12) | parity(12) ], parity_j = parity( info AND B_row_j ).
// Returned bit layout: bit 23..12 = info (info bit 11 at bit 23), bit 11..0 =
// parity (parity bit 11 at bit 11).
uint32_t golay24_encode(uint16_t info12)
{
	uint32_t info = (uint32_t)(info12 & 0x0FFFu);
	uint32_t parity = 0;
	for (int j = 0; j < 12; j++) {
		int p = popparity12(info & GOLAY_B[j]);
		// parity bit j sits at bit (11 - j) so the parity nibble reads MSB-first
		// like the info nibble.
		parity |= ((uint32_t)p) << (11 - j);
	}
	return (info << 12) | parity;
}

// --- Hard decode (reference cross-check only) --------------------------------
// Exhaustive bounded-distance decode: find the unique codeword within Hamming
// distance 3 of `received24`. d_min=8 guarantees at most one such codeword.
// Returns the corrected info word + #errors corrected (0..3), or -1 if no
// codeword is within distance 3 (>=4 errors → decode failure, NOT a
// miscorrection). Used only to cross-check the soft decoder; O(4096) is fine.
int golay24_decode_hard(uint32_t received24, uint16_t* out_info12)
{
	received24 &= 0xFFFFFFu;
	int best_d = 99;
	uint16_t best_info = 0;
	for (uint32_t info = 0; info < 4096u; info++) {
		uint32_t cw = golay24_encode((uint16_t)info);
		int d = popcount32(cw ^ received24);
		if (d < best_d) { best_d = d; best_info = (uint16_t)info; }
	}
	if (best_d > 3) return -1;  // uncorrectable
	if (out_info12) *out_info12 = best_info;
	return best_d;
}

// --- Soft-ML decode -----------------------------------------------------------
// Exhaustive over all 4096 codewords. For codeword c (24 bits), map its bits to
// word_nsym tones (bits_per_tone bits/tone, MSB-first within the 24-bit word,
// matching the TX packing), accumulate tone_cost[s*M + tone_s], track the min.
// Returns the info word of the min-cost codeword. The cost is the normalized
// per-tone energy gap (cl_ofdm::decode_suffix_candidates convention) — minimum
// summed gap == maximum summed energy == the Proakis noncoherent-FSK ML metric.
uint16_t golay24_soft_decode(const double* tone_cost, int M, int bits_per_tone,
                             int word_nsym, double* out_best_cost)
{
	int tone_mask = M - 1;
	double best = 1.0e300;
	uint16_t best_info = 0;
	for (uint32_t info = 0; info < 4096u; info++) {
		uint32_t cw = golay24_encode((uint16_t)info);  // 24-bit codeword
		double c = 0.0;
		// Walk the word_nsym tones MSB-first across the 24-bit codeword.
		for (int s = 0; s < word_nsym; s++) {
			int shift = 24 - bits_per_tone * (s + 1);
			if (shift < 0) shift = 0;
			int tone = (int)((cw >> shift) & (uint32_t)tone_mask);
			c += tone_cost[s * M + tone];
			if (c >= best) break;  // prune: already worse than current best
		}
		if (c < best) { best = c; best_info = (uint16_t)info; }
	}
	if (out_best_cost) *out_best_cost = best;
	return best_info;
}
