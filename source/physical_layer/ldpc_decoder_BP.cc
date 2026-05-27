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
 * Ported from `ldpc_decode_log` in rtmrtmrtmrtm/ft8mon (libldpc.c:215-297,
 * (c) Robert T. Morris, MIT). See header for the full attribution block and
 * MIT->AGPLv3 inclusion notice. The original idea (sum-product / belief
 * propagation in the log domain) is from Sarah Johnson's "Iterative Error
 * Correction" and the Kschischang/Frey/Loeliger 2001 factor-graph paper
 * (already cited at ldpc_decoder_SPA.cc:225).
 */

#include "physical_layer/ldpc_decoder_BP.h"
#include "physical_layer/physical_defines.h"
#include <cmath>
#include <cstring>
#include <vector>

// Inline tanh/atanh helpers matching the Mercury SPA convention
// (ldpc_decoder_SPA.cc:152-159: clamp |x| to 0.9999999 before atanh).
// Single-precision throughout, matching ft8mon's `REAL = float` lineage
// for BP messages.
static inline float bp_tanh_half(float x)
{
    // tanh of half is well-conditioned for any input magnitude;
    // float is plenty since |tanh| <= 1.
    return std::tanh(0.5f * x);
}

static inline float bp_two_atanh_clamped(float a)
{
    // Mirror Mercury SPA's clamp:
    //   if (temp == 1.0)  temp = 0.9999999;
    //   if (temp == -1.0) temp = -0.9999999;
    // We extend the clamp to anything that would push atanh past ~9.2
    // to keep the message magnitudes bounded.
    if (a >=  0.9999999f) a =  0.9999999f;
    if (a <= -0.9999999f) a = -0.9999999f;
    return 2.0f * std::atanh(a);
}

// Compute the syndrome weight (count of failing parity checks) for a hard
// decision vector. Returns 0 iff every check is satisfied.
static int bp_syndrome_weight(
    const int* hard_cw,
    int* C, int CWidth, int CWidthMax,
    int P)
{
    int nOnes = 0;
    for (int j = 0; j < P; j++)
    {
        int x = hard_cw[*(C + j * CWidthMax + 0)];
        for (int k = 1; k < CWidth; k++)
        {
            int idx = *(C + j * CWidthMax + k);
            if (idx != -1)
            {
                x ^= hard_cw[idx];
            }
        }
        nOnes += x;
    }
    return nOnes;
}

int decode_BP(
    const float  LLRi[],
    int          LLRo[],
    int          LLRo_full[],
    float*       zsave,
    int          maxosd_snapshots,
    int*         C, int CWidth, int CWidthMax,
    int*         V, int VWidth, int VWidthMax,
    const int*   apmask,
    int          N, int K, int P,
    int          nIteration_max,
    std::atomic<bool>* abort_flag
)
{
    // V is unused inside the loop bodies that follow because we walk
    // edges via C (check-major) for c->v updates and read the same C
    // for v->c updates of attached checks. V (variable-major sparse
    // index) is kept in the signature for symmetry with decode_SPA and
    // because future optimizations (per-variable message caching) can
    // use it. Mark it to silence -Wunused-parameter for now.
    (void)V;
    (void)VWidth;
    (void)VWidthMax;

    // Message arrays, log-domain LLRs. Same shape as ft8mon:
    //   m[P][N]   v->c messages (variable to check), seeded from channel LLRs
    //   e[P][N]   c->v messages (check to variable), zero-initialized
    // Stack allocation guarded by the Mercury N_MAX/C_WIDTH_MAX defines —
    // for ROBUST_0 with N=1600, P=1500 this is 1500*1600*4 = ~9.6 MB per
    // array. That's too big for stack; allocate via std::vector for safety.
    // (Note: SPA gets away with smaller R/Q arrays because they're N*Vwidth,
    // not P*N. The dense ft8mon layout uses the rectangular form. A future
    // refactor could compress to N*Vwidth for memory efficiency; for now we
    // match ft8mon's shape for porting fidelity.)
    std::vector<float> m_storage((size_t)P * (size_t)N, 0.0f);
    std::vector<float> e_storage((size_t)P * (size_t)N, 0.0f);
    float* m = m_storage.data();
    float* e = e_storage.data();

    // Seed m[j][i] = channel LLR for every i, for every check j that touches i.
    // (ft8mon seeds everywhere; only the edges that participate in checks
    // matter, but seeding everywhere is simpler and harmless because the
    // unused cells are never read.)
    for (int j = 0; j < P; j++)
    {
        for (int k = 0; k < CWidth; k++)
        {
            int i = *(C + j * CWidthMax + k);
            if (i != -1)
            {
                m[(size_t)j * N + i] = LLRi[i];
            }
        }
    }

    // Working buffers.
    std::vector<int> cw(N, 0);
    int best_score = -1;
    std::vector<int> best_cw(N, 0);
    // Pre-zero posterior buffer used for zsave snapshots.
    std::vector<float> posterior(N, 0.0f);

    // Snapshot iter indices: evenly spaced through the iter budget, plus
    // the final iter. ft8mon takes 1..maxosd snapshots at increasing iter
    // counts so OSD gets diverse starting points (see research §2.5).
    // We compute snapshot_at[k] = iter index where snapshot k should fire.
    std::vector<int> snapshot_at;
    if (maxosd_snapshots > 0)
    {
        snapshot_at.resize(maxosd_snapshots);
        for (int k = 0; k < maxosd_snapshots; k++)
        {
            // First snapshot at iter ~nIteration_max/(maxosd_snapshots+1),
            // last at iter ~nIteration_max. Always at least iter 1.
            int idx = ((k + 1) * nIteration_max) / (maxosd_snapshots + 1);
            if (idx < 1) idx = 1;
            snapshot_at[k] = idx;
        }
    }
    int snapshot_filled = 0;

    int iter;
    for (iter = 1; iter <= nIteration_max; iter++)
    {
        // Early exit: another parallel decoder already succeeded.
        if (abort_flag && abort_flag->load(std::memory_order_relaxed))
        {
            return LDPC_BP_ABORT;
        }

        // === Step 1: c->v messages (e[j][i]) ===
        // For each check j and each variable i it touches, compute
        //   e[j][i] = 2 * atanh( prod over i' != i of tanh( m[j][i'] / 2 ) )
        for (int j = 0; j < P; j++)
        {
            for (int k1 = 0; k1 < CWidth; k1++)
            {
                int i1 = *(C + j * CWidthMax + k1);
                if (i1 == -1) continue;

                float a = 1.0f;
                for (int k2 = 0; k2 < CWidth; k2++)
                {
                    int i2 = *(C + j * CWidthMax + k2);
                    if (i2 == -1) continue;
                    if (i2 == i1) continue;
                    a *= bp_tanh_half(m[(size_t)j * N + i2]);
                }
                e[(size_t)j * N + i1] = bp_two_atanh_clamped(a);
            }
        }

        // === Step 2: hard-decision and syndrome check ===
        // posterior[i] = channel LLR + sum of incoming c->v messages.
        // For punctured positions (apmask), bypass the sum: posterior is
        // forced to the (zero) channel LLR so the check messages don't
        // dominate. This mirrors WSJT's apmask semantics — when the bit
        // is known to be erased we don't let unreliable messages drive
        // it. (See research §4.9 risk note 4.)
        for (int i = 0; i < N; i++)
        {
            float l = LLRi[i];
            if (!(apmask && apmask[i]))
            {
                // Sum the c->v messages from every check touching variable i.
                // We walk via the V matrix (row-indices per col).
                for (int kv = 0; kv < VWidth; kv++)
                {
                    int j = *(V + i * VWidthMax + kv);
                    if (j == -1) continue;
                    l += e[(size_t)j * N + i];
                }
            }
            posterior[i] = l;
            cw[i] = (l <= 0.0f) ? 1 : 0;   // sign convention: <=0 => bit=1
        }

        // Score = number of satisfied parity checks; 0 = all OK.
        int nFail = bp_syndrome_weight(cw.data(), C, CWidth, CWidthMax, P);

        // Snapshot this iter's posterior into zsave[k] if scheduled.
        // ft8mon snapshots zsum (running sum across iters); we snapshot
        // the per-iter posterior because it carries the same "BP's best
        // current guess" semantics with simpler bookkeeping.
        if (maxosd_snapshots > 0 && snapshot_filled < maxosd_snapshots &&
            iter == snapshot_at[snapshot_filled])
        {
            std::memcpy(zsave + (size_t)snapshot_filled * N,
                        posterior.data(),
                        sizeof(float) * (size_t)N);
            snapshot_filled++;
        }

        if (nFail == 0)
        {
            // SUCCESS: copy info bits to caller's LLRo[K] and (optionally)
            // full codeword to LLRo_full[N], then return iter count.
            for (int i = 0; i < K; i++)
            {
                LLRo[i] = cw[i];
            }
            if (LLRo_full)
            {
                for (int i = 0; i < N; i++)
                {
                    LLRo_full[i] = cw[i];
                }
            }
            // If caller wanted N snapshots but we converged early, fill
            // any remaining slots with the final posterior so OSD has
            // something to chew on.
            while (snapshot_filled < maxosd_snapshots)
            {
                std::memcpy(zsave + (size_t)snapshot_filled * N,
                            posterior.data(),
                            sizeof(float) * (size_t)N);
                snapshot_filled++;
            }
            return iter;
        }

        // Track best-so-far for the failure path (most-satisfied codeword).
        // Lower nFail = closer to a real codeword. We invert so "score"
        // matches ft8mon's "more is better" convention used at line 271.
        int score = P - nFail;
        if (score > best_score)
        {
            best_cw = cw;
            best_score = score;
        }

        // === Step 3: v->c messages (m[j][i]) ===
        // For every edge (j, i), recompute
        //   m[j][i] = channel LLR + sum of e[j'][i] for j' != j.
        // Implementation: posterior[i] - e[j][i].
        // For punctured positions (apmask), keep m[j][i] = channel LLR (=0)
        // so messages out of the punctured variable never carry false info.
        for (int i = 0; i < N; i++)
        {
            for (int kv = 0; kv < VWidth; kv++)
            {
                int j = *(V + i * VWidthMax + kv);
                if (j == -1) continue;
                if (apmask && apmask[i])
                {
                    m[(size_t)j * N + i] = LLRi[i];
                }
                else
                {
                    m[(size_t)j * N + i] = posterior[i] - e[(size_t)j * N + i];
                }
            }
        }
    }

    // Failure path: BP did not converge to a syndrome-clean codeword
    // within nIteration_max iterations. Return the best-so-far hard
    // decision so downstream OSD has a sensible starting point — but
    // the LDPC_BP_FAIL return tells the caller this is not a clean win.
    for (int i = 0; i < K; i++)
    {
        LLRo[i] = best_cw[i];
    }
    if (LLRo_full)
    {
        for (int i = 0; i < N; i++)
        {
            LLRo_full[i] = best_cw[i];
        }
    }
    // Fill remaining zsave slots with the final posterior.
    while (snapshot_filled < maxosd_snapshots)
    {
        std::memcpy(zsave + (size_t)snapshot_filled * N,
                    posterior.data(),
                    sizeof(float) * (size_t)N);
        snapshot_filled++;
    }
    return LDPC_BP_FAIL;
}
