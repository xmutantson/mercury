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

#ifndef INC_GOLAY24_H_
#define INC_GOLAY24_H_

#include <cstdint>

// =============================================================================
// Extended binary Golay code (24,12,8) — Tier-2 suffix FEC (SIM SPIKE)
// =============================================================================
//
// connect-suffix-fec-research.md §3 Tier 2: add REAL parity symbols to the
// control suffix so its decode cliff can track the base-pattern detection
// floor (−14.68 dB) instead of the uncoded p^13 cliff (−7.3 dB hard /
// −8.7 dB Tier-1 soft list).
//
// Why Golay(24,12,8): it is the MIL-STD-188-141A / FED-STD-1045 forward-error-
// correction code for HF AUTOMATIC LINK ESTABLISHMENT (the exact problem class
// here — waking a link at low SNR). It is trivially correct (a fixed systematic
// generator G=[I|B], 4096 codewords) and admits exhaustive soft-ML decoding
// over the per-tone energies a noncoherent square-law FSK demod already
// produces. Refs:
//   - en.wikipedia.org/wiki/Binary_Golay_code (G=[I|B], B = complement of the
//     icosahedron adjacency matrix; d_min=8 → corrects 3 / detects 4 hard).
//   - MIL-STD-188-141A ALE FEC (Johnson 1991, "An Efficient Golay Codec for
//     MIL-STD-188-141A and FED-STD-1045").
//   - Proakis, Digital Communications Ch.8 — for noncoherent M-FSK the ML
//     soft metric is the sum of received per-tone energies at the candidate
//     codeword's transmitted tones (square-law combining).
//
// This is a SIM-SPIKE module: it measures coding gain. It is NOT wired into any
// production path. The production 13-symbol suffix (ack_sack_suffix_len()) is
// untouched; mode=0 (and mode=1 Tier-1) behave byte-identically to today.

// --- Hard codec ---------------------------------------------------------------

// Encode 12 info bits (low 12 bits of `info12`, MSB = bit 11) into a 24-bit
// systematic codeword [info:12 | parity:12]. Returns the 24-bit codeword in the
// low 24 bits (bit 23 = info bit 11, bit 11 = parity bit 11). Deterministic;
// uses the fixed systematic generator G=[I|B].
uint32_t golay24_encode(uint16_t info12);

// Hard syndrome decode of a 24-bit received word (corrects <=3 bit errors).
// Returns the corrected 12-bit info word in out_info12 and the number of
// corrected bit errors (0..3), or returns -1 if the word is uncorrectable
// (>=4 errors — a decode failure, NOT a miscorrection). This is the standard
// reference hard decoder used only for cross-checking the soft decoder.
int golay24_decode_hard(uint32_t received24, uint16_t* out_info12);

// --- Soft-ML decode over per-symbol tone costs --------------------------------
//
// A Golay word occupies WORD_NSYM = 24/bits_per_tone M-FSK symbols (6 at M=16).
// `tone_cost[s*M + t]` is a NON-NEGATIVE soft cost for hypothesizing tone t in
// symbol s of this word — MONOTONE INCREASING in the per-tone log-likelihood
// gap (0 for the strongest/argmax tone). This is exactly the convention
// cl_ofdm::decode_suffix_candidates emits (normalized energy gap), so the soft
// decoder consumes the SAME soft information the Tier-1 list decoder does — the
// per-tone ENERGIES, not the hard argmax.
//
// Exhaustive ML: for each of the 4096 codewords, map its 24 bits to WORD_NSYM
// tones (bits_per_tone bits/tone, MSB-first), sum the corresponding tone_cost
// entries, and return the info word of the minimum-cost codeword. Minimizing
// summed gap-cost is equivalent to MAXIMIZING summed tone energy (the Proakis
// noncoherent-FSK ML metric), because cost is an affine-decreasing function of
// energy within each symbol.
//
// out_best_cost (optional) receives the winning codeword's summed cost (>=0);
// callers can use it as a soft reliability / combined-word metric. Returns the
// decoded 12-bit info word (always succeeds — ML always picks SOME codeword;
// the CRC over the concatenated words is the accept gate, exactly as today).
uint16_t golay24_soft_decode(const double* tone_cost, int M, int bits_per_tone,
                             int word_nsym, double* out_best_cost);

#endif // INC_GOLAY24_H_
