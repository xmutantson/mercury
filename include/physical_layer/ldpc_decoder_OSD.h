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
 *
 * --- Attribution ---
 *
 * Ported from `osd_decode` in rtmrtmrtmrtm/ft8mon (osd.cc:102-222,
 * (c) Robert T. Morris, MIT License). Ft8mon's OSD is itself adapted from
 * the WSJT-X OSD174_91 implementation (Steve Franke, K9AN, GPLv3) and
 * follows Fossorier & Lin's "Soft-Decision Decoding of Linear Block Codes
 * Based on Ordered Statistics" (IEEE Trans. Information Theory, 41(5),
 * 1995, doi:10.1109/18.412683).
 *
 * Mercury adaptations:
 *   - Generalized for arbitrary (N, K). Initially targets N=1600, K=100
 *     (rate-1/16 ROBUST_0/1).
 *   - Single-flat-array generator matrix (uint8_t G[K*N], row-major)
 *     instead of ft8mon's gen_sys[N][K] static table.
 *   - Adds apmask[] support (per-position "trust me" flag for punctures /
 *     erasures): when set, the position's reliability is boosted so it lands
 *     in the MRB even if its LLR magnitude is small (mirrors WSJT BP semantics,
 *     research doc §4.9 risk note 4).
 *   - decode_abort plumbing — checked between MRB construction and each TEP
 *     flip iteration so monitor-decode parallelism can early-exit.
 *   - depth-1 by default with structural hook for depth-2..3 (norder param).
 *     Depth-2/3 enumerate K-choose-w bit flips; for K=100, depth-2 is
 *     ~5 000 patterns, depth-3 ~162 000.
 *   - Score accumulator promoted to double (precision for sum of 1600
 *     LLR magnitudes) per research doc §4.9 risk note 3.
 *   - LLR sign convention: <0 => bit=1 (matches Mercury SPA/BP).
 *
 * This combined work is distributed under the GNU AGPLv3.
 */

#ifndef LDPC_DECODER_OSD_H_
#define LDPC_DECODER_OSD_H_

#include <atomic>
#include <cstdint>

// Return-code sentinels for the OSD decoder, see research doc §5.6.
//
// On success, returns the OSD order at which the best codeword was found
// (0 for order-0, 1..norder for higher orders). Caller can use this as a
// rough quality signal; deeper orders mean "BP-residual was further from
// the true codeword".
#define LDPC_OSD_FAIL    (-1)   // No codeword satisfied the score threshold.
#define LDPC_OSD_ABORT   (-2)   // abort_flag asserted mid-search.

// Soft-decision OSD decode.
//
// Inputs:
//   LLRi[N]         : posterior LLRs (typically from BP/SPA). Sign <0 => bit=1.
//   G[K*N]          : dense generator matrix (row-major, uint8_t 0/1 bits).
//                     G[i][j] = j'th codeword bit produced by the i'th info bit.
//   apmask[N]       : optional per-position erasure mask (NULL = none). 1 means
//                     this position should NOT enter the MRB (its LLR is
//                     untrusted) — implemented by zeroing its reliability so
//                     the sort sends it to the bottom.
//   norder          : OSD order, 0 = no flips, 1 = single-bit flips of MRB,
//                     2 = double, 3 = triple. (Cost scales as K^norder.)
//
// Outputs:
//   LLRo[K]         : decoded info bits.
//   dmin_out        : (optional, NULL allowed) Euclidean distance of best
//                     codeword from received LLR vector.
//
// Returns:
//   Order at which the best codeword was found (0..norder), or LDPC_OSD_FAIL /
//   LDPC_OSD_ABORT.
int decode_OSD(
    const float    LLRi[],
    int            LLRo[],
    const uint8_t* G,
    const int*     apmask,        // may be NULL
    int            N, int K,
    int            norder,        // 0..3 typically
    double*        dmin_out,      // may be NULL
    std::atomic<bool>* abort_flag = nullptr
);

#endif // LDPC_DECODER_OSD_H_
