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

// Dense generator matrix construction for Mercury's rate-1/16 LDPC code
// (ROBUST_0/1: N=1600, K=100).
//
// Mercury's encoder (cl_ldpc::encode at ldpc.cc:100-121) is sparse — it walks
// QCmatrixEnc to XOR info bits into each parity bit. WSJT-X/ft8mon's OSD
// algorithm requires a DENSE generator matrix G[K][N] so it can sort columns
// by reliability, run Gauss-Jordan elimination on the permuted G, and re-encode
// from the Most-Reliable Basis.
//
// We construct G at first call by encoding K standard basis vectors:
//   For each i in 0..K-1:
//     msg[K] = e_i  (one-hot at position i)
//     encode(msg, codeword[N])
//     G[i*N + j] = codeword[j]   for j in 0..N-1
//
// The result is stored in a process-static buffer, owned by this module, and
// returned as a const pointer. The build is idempotent and thread-safe via
// std::call_once. Per research §4.4 the cost is ~1 ms and 20 kB for K=100.
//
// Verification: G * H^T = 0 over GF(2). Verified by
// `verify_ldpc_generator_1_16()` which the BP+OSD test harness calls.

#ifndef LDPC_GENERATOR_1_16_H_
#define LDPC_GENERATOR_1_16_H_

#include <cstdint>

// Returns a pointer to a K_RATE_1_16 * N_RATE_1_16 buffer of 0/1 bytes
// (row-major: G[i * N + j] is the j'th codeword bit produced by info bit i).
// First call builds the matrix; subsequent calls return the cached pointer.
const uint8_t* ldpc_get_dense_G_1_16();

// Constants — match cl_ldpc init for rate 1/16 (mercury_normal_1_16.cc):
//   K = 100 info bits, N = 1600 codeword bits, P = N - K = 1500 parity bits.
#define K_RATE_1_16   100
#define N_RATE_1_16   1600
#define P_RATE_1_16   1500

// Verifies G * H^T = 0 over GF(2), where H is Mercury's QCmatrixC sparse
// parity-check matrix for rate 1/16. Returns true on success.
// Optional out_failed_check (may be NULL): if verification fails, set to the
// first row/check pair that did not satisfy G·H^T = 0.
bool verify_ldpc_generator_1_16(int* out_failed_row = nullptr,
                                int* out_failed_check = nullptr);

#endif // LDPC_GENERATOR_1_16_H_
