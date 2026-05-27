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
 * This file is a port of the log-domain belief-propagation (BP) decoder
 * `ldpc_decode_log` from rtmrtmrtmrtm/ft8mon (https://github.com/rtmrtmrtmrtm/ft8mon),
 * file `libldpc.c` lines 215-297, copyright (c) Robert T. Morris, MIT License.
 * The original was written for the FT8 (174,91) LDPC code with sparse matrices
 * Nm/Mn following the WSJT-X lineage (FT4/FT8 protocols, Franke & Taylor 2020).
 *
 * Mercury adaptations:
 *   - Operates on Mercury's QCmatrixC (= WSJT Nm) and QCmatrixV (= WSJT Mn)
 *     sparse matrices with -1 sentinels instead of 1-based / 0-sentinel WSJT.
 *   - Code parameters generalized for any (N, K) rate-1/16 family member;
 *     specifically targets N=1600, K=100 (ROBUST_0/1).
 *   - Single-precision (float) BP messages following ft8mon; Mercury's existing
 *     SPA uses double. See research doc §4.9 risk note 3.
 *   - tanh/atanh + ±0.9999999 clamp following Mercury SPA convention at
 *     ldpc_decoder_SPA.cc:152-159, not WSJT's piecewise platanh.
 *   - LLR sign convention: <0 => bit=1 (matches Mercury SPA at
 *     ldpc_decoder_SPA.cc:59, LLRbin[i]=(LLRi[i]<0)).
 *   - Optional posterior snapshots into zsave[] for downstream OSD use.
 *   - std::atomic<bool>* abort hook (Mercury monitor-decode parallel
 *     racing — see ldpc.h:96).
 *
 * This combined work is distributed under the GNU AGPLv3 (AGPLv3 §13:
 * inclusion of MIT-licensed code is permitted; MIT attribution preserved
 * in this header per MIT terms).
 */

#ifndef LDPC_DECODER_BP_H_
#define LDPC_DECODER_BP_H_

#include <atomic>

// Mercury sentinel return values, see research doc §5.6.
#define LDPC_BP_FAIL    (-1)   // BP exhausted iter cap without satisfying all parity checks.
#define LDPC_BP_ABORT   (-2)   // abort_flag asserted (parallel monitor decode raced ahead).

// Log-domain belief-propagation LDPC decoder.
//
// Inputs:
//   LLRi[N]           : channel LLRs, float; convention <0 => bit=1.
//   C[P*CWidthMax]    : Mercury QCmatrixC (col-indices per check row, -1 sentinel).
//   V[N*VWidthMax]    : Mercury QCmatrixV (row-indices per variable col, -1 sentinel).
//   apmask[N]         : optional puncture mask (NULL = no mask). 1 = position is punctured;
//                       the BP loop substitutes posterior = channel LLR (no message sum)
//                       so a zero-LLR puncture doesn't get amplified by message-passing.
//   maxosd_snapshots  : number of zsave slots to fill (0 = none).
//   zsave             : [maxosd_snapshots][N] posterior LLR snapshots taken at fixed iters
//                       (NULL allowed iff maxosd_snapshots==0). Snapshots are taken at
//                       evenly-spaced iter indices and the final posterior. Used by OSD
//                       fallback to seed alternate MRB orderings.
//
// Outputs:
//   LLRo[K]           : hard-decided info bits (0/1).
//   LLRo_full[N]      : OPTIONAL hard-decided full codeword (NULL allowed). When OSD
//                       fallback is used, the full hard-decision vector is needed.
//
// Returns:
//   iter count (1..nIteration_max) on BP convergence with zero parity-check failures,
//   LDPC_BP_FAIL if iter cap exhausted, LDPC_BP_ABORT if abort_flag asserted.
//
// Notes:
//   - Heap-allocates two N*max(VWidth)*sizeof(float) message arrays per call. For
//     hot-path use the caller should pre-allocate (TODO §7.5 integration); for the
//     first BP+OSD ship the cost is one alloc per failed SPA decode, ~50 KB.
//   - Single-precision (float) per ft8mon. OSD's distance accumulator separately
//     uses double; this BP is the "cheap" half of the pair.
//   - Heavy structural similarity to ldpc_decoder_SPA.cc; differs in: float vs double,
//     log-domain messages directly (no tanh/atanh round-trip in v->c update; only
//     in c->v).
int decode_BP(
    const float  LLRi[],
    int          LLRo[],
    int          LLRo_full[],         // may be NULL
    float*       zsave,               // may be NULL iff maxosd_snapshots==0; [maxosd_snapshots][N]
    int          maxosd_snapshots,
    int*         C, int CWidth, int CWidthMax,
    int*         V, int VWidth, int VWidthMax,
    const int*   apmask,              // may be NULL
    int          N, int K, int P,
    int          nIteration_max,
    std::atomic<bool>* abort_flag = nullptr
);

#endif // LDPC_DECODER_BP_H_
