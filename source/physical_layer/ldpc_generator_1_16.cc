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

#include "physical_layer/ldpc_generator_1_16.h"
#include "physical_layer/ldpc.h"
#include "physical_layer/mercury_normal_1_16.h"
#include "physical_layer/physical_defines.h"

#include <cstring>
#include <mutex>
#include <vector>

namespace {

// Process-static buffer for the dense generator.
// 100 * 1600 = 160 000 bytes ≈ 160 KB. (Could be bit-packed to 20 KB if
// memory ever matters; uint8_t is simpler and faster for OSD's per-element
// XOR loop.)
uint8_t g_dense_G[K_RATE_1_16 * N_RATE_1_16];
std::once_flag g_dense_G_built;

void build_dense_G_once()
{
    // Spin up a fresh cl_ldpc just for encoding. Don't mutate any caller-
    // owned ldpc instance.
    cl_ldpc ldpc;
    ldpc.standard = MERCURY;
    ldpc.framesize = MERCURY_NORMAL;            // = N_RATE_1_16 (1600)
    ldpc.rate = 1.0f / 16.0f;                   // -> K = 100
    ldpc.decoding_algorithm = SPA;              // doesn't matter — we only encode
    ldpc.nIteration_max = 1;                    // doesn't matter
    ldpc.GBF_eta = 0.0f;
    ldpc.print_nIteration = NO;
    ldpc.init();

    // Sanity: confirm the dimensions match what we expect.
    // (If a future code-table swap silently changed N or K this would
    // catch it before producing a garbage G.)
    if (ldpc.N != N_RATE_1_16 || ldpc.K != K_RATE_1_16 || ldpc.P != P_RATE_1_16)
    {
        // Don't throw — log to stderr and leave G zero-initialized. Tests
        // and verify_ldpc_generator_1_16() will detect the bad state.
        // (No exception in this codebase pattern; cl_ldpc::update_code_parameters
        // calls exit() on hard failures.)
        // We can't sensibly continue, so abort.
        fprintf(stderr,
                "[BP-OSD] ERROR ldpc_generator_1_16: cl_ldpc init returned "
                "N=%d K=%d P=%d, expected %d/%d/%d. dense_G left zero.\n",
                ldpc.N, ldpc.K, ldpc.P,
                N_RATE_1_16, K_RATE_1_16, P_RATE_1_16);
        std::memset(g_dense_G, 0, sizeof(g_dense_G));
        return;
    }

    std::vector<int> msg(K_RATE_1_16, 0);
    std::vector<int> cw(N_RATE_1_16, 0);

    for (int i = 0; i < K_RATE_1_16; i++)
    {
        // One-hot info vector
        std::memset(msg.data(), 0, sizeof(int) * K_RATE_1_16);
        msg[i] = 1;

        // Encode: writes K info bits then P parity bits into cw[N].
        ldpc.encode(msg.data(), cw.data());

        // Copy into G row i.
        for (int j = 0; j < N_RATE_1_16; j++)
        {
            g_dense_G[i * N_RATE_1_16 + j] = (uint8_t)(cw[j] & 0x01);
        }
    }

    // ldpc destructor on scope exit frees the matrices we borrowed pointers
    // to. We've finished using them by this point.
}

} // anonymous namespace

const uint8_t* ldpc_get_dense_G_1_16()
{
    std::call_once(g_dense_G_built, build_dense_G_once);
    return g_dense_G;
}

bool verify_ldpc_generator_1_16(int* out_failed_row, int* out_failed_check)
{
    const uint8_t* G = ldpc_get_dense_G_1_16();

    // H is implicit in QCmatrixC: each parity-check row j has CWidth column
    // indices listing where its 1s live (with -1 sentinel for unused slots).
    // We verify G * H^T = 0 by checking, for every G row i and every check j:
    //   xor over k of G[i][ C[j][k] ]  == 0
    const int CWidth     = mercury_normal_Cwidth_1_16;
    const int CWidthMax  = CWidth;
    const int* C = &mercury_normal_QCmatrixC_1_16[0][0];

    for (int i = 0; i < K_RATE_1_16; i++)
    {
        for (int j = 0; j < P_RATE_1_16; j++)
        {
            int x = 0;
            for (int k = 0; k < CWidth; k++)
            {
                int col = *(C + j * CWidthMax + k);
                if (col == -1) continue;
                x ^= G[i * N_RATE_1_16 + col] & 0x01;
            }
            if (x != 0)
            {
                if (out_failed_row)   *out_failed_row   = i;
                if (out_failed_check) *out_failed_check = j;
                return false;
            }
        }
    }
    return true;
}
