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
 * Top-level glue: runs BP first (translated from ft8mon::ldpc_decode_log),
 * then invokes OSD (translated from ft8mon::osd_decode) on failure. The
 * "BP -> OSD-fallback-with-zsave-snapshot" cascade pattern itself is from
 * WSJT-X's lib/ft8/decode174_91.f90 → osd174_91.f90 wrapper. Mercury's
 * port replaces the snapshot-driven multi-call OSD with the simpler
 * single-call-on-fail pattern; multi-snapshot upgrade is research-doc §7
 * future work.
 *
 * Per research doc §4.3 Option B, this BP+OSD entry point is intended to be
 * called as a *fallback* from cl_telecom_system::receive_msg when SPA fails
 * (Phase A.2.5 integration — not in this branch's scope). The Phase A.2
 * deliverable here is the decoder pair, not the integration call site.
 */

#ifndef LDPC_DECODER_BP_OSD_H_
#define LDPC_DECODER_BP_OSD_H_

#include "physical_layer/ldpc_decoder_BP.h"
#include "physical_layer/ldpc_decoder_OSD.h"
#include <atomic>
#include <cstdint>

// Combined return-code sentinels.
//   0..bp_nIteration_max   : BP succeeded at iter N (no OSD needed).
//   1000 + osd_order        : BP failed, OSD succeeded at order `osd_order`.
//   LDPC_BP_OSD_FAIL        : Both BP and OSD failed.
//   LDPC_BP_OSD_ABORT       : abort_flag tripped.
// (1000 offset is arbitrary — keeps the BP-success and OSD-success regions
// far apart in the integer space so diagnostics can distinguish them at a
// glance. See research doc §5.6.)
#define LDPC_BP_OSD_OSD_BASE   1000
#define LDPC_BP_OSD_FAIL       (-1)
#define LDPC_BP_OSD_ABORT      (-2)

// Run BP, then OSD on failure. Inputs identical to decode_BP except for the
// added G[K*N] (dense generator) and osd_norder (depth) knobs.
//
// If osd_norder == -1, OSD is disabled (BP only). If osd_norder >= 0,
// OSD runs on BP failure with that order. osd_maxosd_snapshots controls
// how many BP iter snapshots are taken (currently the first snapshot is
// the one fed to OSD; the others are reserved for the §7.5 multi-snapshot
// upgrade).
int decode_BP_OSD(
    const float    LLRi[],
    int            LLRo[],
    int*           C, int CWidth, int CWidthMax,
    int*           V, int VWidth, int VWidthMax,
    const uint8_t* G,
    const int*     apmask,                    // may be NULL
    int            N, int K, int P,
    int            bp_nIteration_max,
    int            osd_norder,                // -1 = OSD disabled
    int            osd_maxosd_snapshots,      // 0 = no zsave, 1 = single snapshot
    std::atomic<bool>* abort_flag = nullptr
);

#endif // LDPC_DECODER_BP_OSD_H_
