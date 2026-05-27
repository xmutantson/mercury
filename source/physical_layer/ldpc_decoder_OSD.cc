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
 * (c) Robert T. Morris, MIT License). See header for full attribution and
 * algorithmic lineage (WSJT-X OSD174_91, Fossorier & Lin 1995).
 *
 * The core OSD algorithm comprises three steps:
 *   1. Sort codeword positions by |LLR| descending → MRB candidate ordering.
 *   2. Run Gauss-Jordan elimination over GF(2) on the permuted columns of G
 *      to bring the first K reliable positions into the identity (the MRB).
 *      If linearly dependent, swap in lower-reliability columns until full
 *      rank achieved.
 *   3. Re-encode from the K hard-decided MRB bits to get a codeword candidate
 *      ("order-0"). For higher orders, flip subsets of MRB bits and re-encode,
 *      keep the lowest-Euclidean-distance codeword.
 */

#include "physical_layer/ldpc_decoder_OSD.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <numeric>
#include <vector>

namespace {

// Compute the Euclidean-style score of a candidate codeword `xcw` against the
// received LLR vector `LLRi`. Lower = better. Sign convention <0 => bit=1.
//
// For each position i:
//   xcw[i] == 0  =>  we expect LLRi[i] >= 0; cost = -LLRi[i] if LLRi[i] < 0,
//                    else 0. Implemented as soft cost: if xcw and hard
//                    decision disagree, add |LLRi[i]|.
// Equivalent ft8mon implementation: score += LLRi*4.6 with signs, summed,
// then negated. We do the simpler equivalent: sum |LLRi[i]| over positions
// where xcw disagrees with hard decision. (4.6 is just a constant scale
// factor — irrelevant for comparing candidates.)
//
// Returns the sum of |LLR| at disagreement positions. Caller uses this to
// pick the minimum-distance candidate.
double osd_score_double(
    const int* xcw,         // candidate codeword, N bits
    const float* LLRi,      // received LLRs
    int N)
{
    double score = 0.0;
    for (int i = 0; i < N; i++)
    {
        // hard decision on LLRi[i]: <0 => bit=1.
        int hd = (LLRi[i] < 0.0f) ? 1 : 0;
        if (xcw[i] != hd)
        {
            // disagreement — add absolute magnitude.
            score += std::fabs((double)LLRi[i]);
        }
    }
    return score;
}

// Re-encode K info bits into an N-bit codeword by XORing the rows of G
// (one row per info bit) where the info bit is 1. Output codeword in `cw[N]`.
void osd_encode_from_G(
    const uint8_t* G,
    const int* msg,          // K bits
    int* cw,                 // N bits
    int N, int K)
{
    std::memset(cw, 0, sizeof(int) * (size_t)N);
    for (int i = 0; i < K; i++)
    {
        if (!msg[i]) continue;
        const uint8_t* Gi = G + (size_t)i * N;
        for (int j = 0; j < N; j++)
        {
            cw[j] ^= (int)Gi[j];
        }
    }
}

// Gauss-Jordan elimination over GF(2) on an augmented matrix [A | I], where
// A is the K x K submatrix to invert and I is the K x K identity tracked
// in the right half. The full row width is 2K columns.
//
// In our OSD context the augmented matrix actually has K rows ("logical
// rows" indexed by MRB position) and 2K columns (left K = MRB-column slice
// of G, right K = identity). Per ft8mon, however, when those K columns turn
// out to be linearly dependent we want to swap in additional codeword
// positions from the next-most-reliable bunch (rows K..N-1 of the
// permuted-G^T matrix `b`). We let the caller pass:
//
//   b[N][2K]  flat: b_data[r * (2K) + c]
//   row_perm[N] : current row ordering (mutated on swap).
//
// We perform GJ on b's first K rows; if pivot is 0 at column k we search
// row k+1..N-1 for a row with a 1 at col k, and swap that row in
// (also swapping row_perm[k] with row_perm[that_row]). Returns true on
// success (full rank achieved), false otherwise.
bool gj_eliminate(
    uint8_t* b_data,
    int* row_perm,
    int K, int N)
{
    const int width = 2 * K;

    for (int k = 0; k < K; k++)
    {
        // Find a pivot row r in [k, N-1] with b[r][k] == 1.
        if (b_data[k * width + k] == 0)
        {
            int found = -1;
            for (int r = k + 1; r < N; r++)
            {
                if (b_data[r * width + k] == 1)
                {
                    found = r;
                    break;
                }
            }
            if (found < 0)
            {
                // Column k is all zeros from row k onward — matrix is rank-
                // deficient. Caller can either error out or try harder.
                return false;
            }
            // Swap rows k and found in b, and in row_perm.
            for (int c = 0; c < width; c++)
            {
                uint8_t tmp = b_data[k * width + c];
                b_data[k * width + c] = b_data[found * width + c];
                b_data[found * width + c] = tmp;
            }
            int tmpi = row_perm[k];
            row_perm[k] = row_perm[found];
            row_perm[found] = tmpi;
        }

        // Lazy identity: flip b[k][K + k] so the right half becomes I as we go.
        b_data[k * width + K + k] ^= 1;

        // Eliminate column k in all other rows (0..N-1, excluding k).
        for (int r = 0; r < N; r++)
        {
            if (r == k) continue;
            if (b_data[r * width + k] == 0) continue;
            // XOR row k into row r.
            uint8_t* rk = b_data + (size_t)k * width;
            uint8_t* rr = b_data + (size_t)r * width;
            for (int c = 0; c < width; c++)
            {
                rr[c] ^= rk[c];
            }
        }
    }
    return true;
}

} // anonymous namespace

int decode_OSD(
    const float    LLRi[],
    int            LLRo[],
    const uint8_t* G,
    const int*     apmask,
    int            N, int K,
    int            norder,
    double*        dmin_out,
    std::atomic<bool>* abort_flag
)
{
    // Bounds-check the order so a caller bug doesn't melt the CPU.
    if (norder < 0) norder = 0;
    if (norder > 3) norder = 3;

    // === Step 1: Sort positions by reliability descending. ===
    // strength[i] = |LLR[i]|; apmask positions get strength=0 so they sort
    // last (we don't want untrusted bits in the MRB).
    std::vector<float> strength(N);
    for (int i = 0; i < N; i++)
    {
        float s = std::fabs(LLRi[i]);
        if (apmask && apmask[i])
        {
            s = 0.0f;   // demote punctured/erased positions
        }
        strength[i] = s;
    }

    std::vector<int> which(N);
    std::iota(which.begin(), which.end(), 0);
    std::sort(which.begin(), which.end(),
              [&strength](int a, int b) {
                  return strength[a] > strength[b];
              });

    if (abort_flag && abort_flag->load(std::memory_order_relaxed))
    {
        return LDPC_OSD_ABORT;
    }

    // === Step 2: Build the augmented matrix b[N][2K] then Gauss-Jordan. ===
    // Row r of b corresponds to codeword position which[r]:
    //   left half (cols 0..K-1)  = column which[r] of G — i.e.
    //                             [G[0][which[r]], G[1][which[r]], ..., G[K-1][which[r]]]
    //   right half (cols K..2K-1) = 0 initially; gj_eliminate writes the
    //                              identity into rows 0..K-1 lazily.
    std::vector<uint8_t> b((size_t)N * (size_t)(2 * K), 0);
    const int width = 2 * K;
    for (int r = 0; r < N; r++)
    {
        int p = which[r];
        for (int kk = 0; kk < K; kk++)
        {
            b[(size_t)r * width + kk] = G[(size_t)kk * N + p];
        }
        // right half already zero from vector init.
    }

    std::vector<int> row_perm(N);
    for (int i = 0; i < N; i++) row_perm[i] = which[i];

    if (!gj_eliminate(b.data(), row_perm.data(), K, N))
    {
        // Should not happen for our (1600,100) code since rank of G is K.
        // If it ever does, the codeword is unrecoverable via OSD.
        return LDPC_OSD_FAIL;
    }

    // After GJ:
    //   - rows 0..K-1 hold the inverted matrix in their right half:
    //       gen1_inv[i][j] = b[i][K + j].
    //   - row_perm has been permuted so row_perm[0..K-1] are the codeword
    //     positions that ended up in the MRB.

    if (abort_flag && abort_flag->load(std::memory_order_relaxed))
    {
        return LDPC_OSD_ABORT;
    }

    // === Step 3: Hard-decision y1[K] = received bits at MRB positions. ===
    std::vector<int> y1(K);
    for (int i = 0; i < K; i++)
    {
        int p = row_perm[i];
        y1[i] = (LLRi[p] < 0.0f) ? 1 : 0;
    }

    // Helper: solve xplain = gen1_inv * y1 (mod 2).
    auto solve_xplain = [&](const std::vector<int>& y1v, std::vector<int>& xplain_out)
    {
        for (int i = 0; i < K; i++)
        {
            int sum = 0;
            const uint8_t* row_i = b.data() + (size_t)i * width + K;  // gen1_inv row
            for (int j = 0; j < K; j++)
            {
                sum ^= ((int)row_i[j] & y1v[j]);
            }
            xplain_out[i] = sum;
        }
    };

    std::vector<int> xplain(K, 0);
    std::vector<int> xcw(N, 0);
    std::vector<int> best_plain(K, 0);
    double best_score = 0.0;
    int    best_order = -1;
    bool   got_best   = false;

    // Order-0: no flips.
    solve_xplain(y1, xplain);
    osd_encode_from_G(G, xplain.data(), xcw.data(), N, K);
    double sc = osd_score_double(xcw.data(), LLRi, N);
    if (!got_best || sc < best_score)
    {
        best_plain = xplain;
        best_score = sc;
        best_order = 0;
        got_best   = true;
    }

    // Order-w: enumerate every w-bit subset of MRB positions, flip, score.
    // We do a generic recursive enumeration so depth-2/3 are wired in for
    // later research without a rewrite.
    if (norder >= 1)
    {
        // For each w = 1..norder, enumerate K-choose-w subsets of MRB indices.
        // We use the simple "smallest combination first" iterator.
        std::vector<int> idx(norder, 0);    // current subset
        std::vector<int> y1_flip = y1;      // working copy

        auto enumerate = [&](int w, auto&& self) -> int
        {
            // Bounds: idx[0] < idx[1] < ... < idx[w-1] < K
            for (int i = 0; i < w; i++) idx[i] = i;

            while (true)
            {
                if (abort_flag && abort_flag->load(std::memory_order_relaxed))
                {
                    return LDPC_OSD_ABORT;
                }

                // Flip bits at idx[0..w-1], evaluate, then UN-flip.
                for (int i = 0; i < w; i++) y1_flip[idx[i]] ^= 1;
                solve_xplain(y1_flip, xplain);
                osd_encode_from_G(G, xplain.data(), xcw.data(), N, K);
                double score = osd_score_double(xcw.data(), LLRi, N);
                if (!got_best || score < best_score)
                {
                    best_plain = xplain;
                    best_score = score;
                    best_order = w;
                    got_best   = true;
                }
                for (int i = 0; i < w; i++) y1_flip[idx[i]] ^= 1;

                // Advance the combination.
                int pos = w - 1;
                while (pos >= 0 && idx[pos] == K - w + pos) pos--;
                if (pos < 0) break;
                idx[pos]++;
                for (int i = pos + 1; i < w; i++) idx[i] = idx[i - 1] + 1;
            }
            return 0;
        };

        for (int w = 1; w <= norder; w++)
        {
            int rc = enumerate(w, enumerate);
            if (rc == LDPC_OSD_ABORT) return LDPC_OSD_ABORT;
        }
    }

    if (!got_best)
    {
        return LDPC_OSD_FAIL;
    }

    // === Step 4: Materialize the best info bits. ===
    // best_plain[i] is the i'th info bit *in MRB ordering* — it's the
    // re-encoded message bit for MRB position i. But best_plain is already
    // in standard message-bit order because we used the inverse of the
    // K-info-bit subspace of G (the left K columns of G are e_i for the
    // i'th info bit by construction at encode time — i.e. G is systematic
    // on the first K codeword positions). Wait — actually we built the MRB
    // from codeword positions, not info positions. The xplain we computed
    // is "what info bits would, when encoded, reproduce the K MRB hard
    // decisions". So xplain IS the message in standard 0..K-1 order, because
    // matmul = gen1_inv * y1 inverts (permuted G's column-restriction to
    // MRB) and the result is the message that generated those MRB bits.
    //
    // Confirm: this matches ft8mon (osd.cc:174 `matmul(gen1_inv, y1, xplain)`
    // produces xplain that is returned to the caller as the 91 plaintext bits
    // — same semantics).
    for (int i = 0; i < K; i++)
    {
        LLRo[i] = best_plain[i] & 0x01;
    }
    if (dmin_out) *dmin_out = best_score;
    return best_order;
}
