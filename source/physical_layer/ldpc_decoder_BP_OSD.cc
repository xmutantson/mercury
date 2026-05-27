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

#include "physical_layer/ldpc_decoder_BP_OSD.h"
#include <vector>

int decode_BP_OSD(
    const float    LLRi[],
    int            LLRo[],
    int*           C, int CWidth, int CWidthMax,
    int*           V, int VWidth, int VWidthMax,
    const uint8_t* G,
    const int*     apmask,
    int            N, int K, int P,
    int            bp_nIteration_max,
    int            osd_norder,
    int            osd_maxosd_snapshots,
    std::atomic<bool>* abort_flag
)
{
    // Clamp snapshot count to >= 0; need at least 1 if OSD is enabled,
    // otherwise OSD has nothing to chew on (we'd pass it the raw channel
    // LLR which is the strictly weaker case).
    int eff_snapshots = osd_maxosd_snapshots;
    if (eff_snapshots < 0) eff_snapshots = 0;
    if (osd_norder >= 0 && eff_snapshots < 1) eff_snapshots = 1;

    std::vector<float> zsave;
    if (eff_snapshots > 0)
    {
        zsave.assign((size_t)eff_snapshots * (size_t)N, 0.0f);
    }

    // Need the full hard-decision codeword from BP only if we're going to
    // call OSD. (Caller can ignore it via NULL in the simpler BP-only path.)
    std::vector<int> LLRo_full;
    int* full_ptr = nullptr;
    if (osd_norder >= 0)
    {
        LLRo_full.assign((size_t)N, 0);
        full_ptr = LLRo_full.data();
    }

    int bp_rc = decode_BP(
        LLRi, LLRo, full_ptr,
        zsave.empty() ? nullptr : zsave.data(),
        eff_snapshots,
        C, CWidth, CWidthMax,
        V, VWidth, VWidthMax,
        apmask,
        N, K, P,
        bp_nIteration_max,
        abort_flag
    );

    if (bp_rc == LDPC_BP_ABORT)
    {
        return LDPC_BP_OSD_ABORT;
    }
    if (bp_rc >= 0)
    {
        // BP success: pass through the iter count unchanged.
        return bp_rc;
    }

    // BP failed. If OSD disabled, propagate failure.
    if (osd_norder < 0)
    {
        return LDPC_BP_OSD_FAIL;
    }

    // Run OSD on the first zsave snapshot (BP's posterior at iter
    // ~bp_nIteration_max/(snapshots+1)).
    const float* osd_input = zsave.data();   // first snapshot

    int osd_rc = decode_OSD(
        osd_input, LLRo,
        G, apmask,
        N, K,
        osd_norder,
        /*dmin_out=*/nullptr,
        abort_flag
    );

    if (osd_rc == LDPC_OSD_ABORT)
    {
        return LDPC_BP_OSD_ABORT;
    }
    if (osd_rc < 0)
    {
        return LDPC_BP_OSD_FAIL;
    }

    // OSD success: order encoded in upper bits.
    return LDPC_BP_OSD_OSD_BASE + osd_rc;
}
