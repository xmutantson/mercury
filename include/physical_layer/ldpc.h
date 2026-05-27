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

#ifndef INC_LDPC_H_
#define INC_LDPC_H_

#include "ldpc_decoder_GBF.h"
#include "ldpc_decoder_SPA.h"
#include "ldpc_decoder_BP_OSD.h"
#include "mercury_ldpc.h"
#include "physical_defines.h"
#include <iostream>
#include <atomic>
#include <cstdint>


// Unified LDPC decode failure check covering all algorithm return-code
// conventions used in this codebase. Use this everywhere the caller would
// otherwise hard-code `rc > cap-1` against `cl_ldpc::nIteration_max`.
//
// Return-code semantics by algorithm (see source files for details):
//   GBF / SPA:
//     - success         : 1..cap-1   (iter at which all parity checks passed)
//     - failure (cap)   : == cap     (loop terminated normally without break)
//                         actually iteration ends at `cap+1` for SPA's
//                         for(iteration=1;iteration<=cap;) but historical
//                         `> cap-1` check catches both >= cap and the cap+1
//                         exit; we keep equivalent semantics here.
//     - SPA abort       : < 0        (parallel monitor decode race won)
//   BP_OSD (see ldpc_decoder_BP_OSD.h):
//     - BP success      : 1..cap     (positive, below LDPC_BP_OSD_OSD_BASE)
//     - OSD success     : LDPC_BP_OSD_OSD_BASE + order  (>= 1000)
//     - failure         : LDPC_BP_OSD_FAIL  (-1)
//     - abort           : LDPC_BP_OSD_ABORT (-2)
//
// Per Phase A.2 §7.5 / research doc §5.6: callers must treat OSD success as
// a success even though the integer return is far above `cap`. This helper
// returns true iff the decode failed.
static inline bool ldpc_decode_failed(int rc, int algo, int cap)
{
    if (algo == BP_OSD)
    {
        // BP_OSD: any non-negative value below LDPC_BP_OSD_OSD_BASE is BP-success,
        // values >= LDPC_BP_OSD_OSD_BASE are OSD-success, negatives are failure.
        return rc < 0;
    }
    // GBF and SPA: failure is "iteration cap reached".
    // (SPA also returns negative on abort_flag; treat that as failure.)
    if (rc < 0) return true;
    return rc > cap - 1;
}

class cl_ldpc
{
private:
	int Cwidth;
	int Vwidth;
	int dwidth;
	int *QCmatrixEnc;
	int *QCmatrixC;
	int *QCmatrixV;
	int *QCmatrixd;
	double* R;
	double* Q;
	int* V_pos;       // Pre-allocated SPA decoder workspace [P*Cwidth]



	float r;

	int standard_val;
	int decoding_algorithm_val;


	float eta_val;
	int nIteration_max_val;
	int print_nIteration_val;

	// BP+OSD internal mirrors of osd_norder / osd_maxosd, captured at init()
	// for the same reason eta_val/nIteration_max_val mirror their public twins.
	int osd_norder_val;
	int osd_maxosd_val;

	// Dense generator matrix pointer for OSD's MRB encode. Populated at init()
	// when decoding_algorithm == BP_OSD (and only for the rate-1/16 code today;
	// the BP_OSD path is gated to ROBUST configs at the telecom_system layer).
	// Memory is owned by ldpc_generator_1_16.cc's static buffer; we just hold
	// a non-owning const pointer. NULL when BP_OSD is not selected.
	const uint8_t* dense_G_1_16{nullptr};



	int update_code_parameters();

public:
	cl_ldpc();
	~cl_ldpc();
	int N,P,K; //!< N: the message size, P: the parity bit size, K: the information bit size (N=P+K).
	int standard;
	int framesize;
	float rate;
	int decoding_algorithm;
	float GBF_eta; //!< The GBF algorithms correction rate.
	int nIteration_max; //!< The maximum number of LDPC decoding iterations allowed.
	int print_nIteration;

	// BP+OSD knobs — used only when decoding_algorithm == BP_OSD.
	// See ldpc_decoder_BP_OSD.h for semantics. Defaults: OSD-1 (norder=1),
	// single OSD call (maxosd=0). Set per-config from
	// default_configurations_telecom_system_t in telecom_system.cc.
	int osd_norder{1};
	int osd_maxosd{0};

	void init();
	void deinit();

	//! The LDPC encoding function, calculates and annex the parity bits to the original data.
	    /*!
	      \param data is the data to be protected by the LDPC code.
	      \param encoded_data is the concatenation of the original data with the LDPC parity bits.
	      \return None
	   */
	void encode(const int* data, int*  encoded_data);

	//! The LDPC decoding function, validates the message integrity and attempts to correct bit errors.
	    /*!
	      \param data is the received message.
	      \param encoded_data is the corrected data without the LDPC parity bits.
	      \return number of iterations used to decode the message, the message maybe corrupt if this reaches the max number of iterations allowed.
	   */
	int decode(const float* data,  int*  decoded_data);

	// Abort flag for parallel monitor decode: when another decoder succeeds,
	// set this to true so remaining decoders exit their LDPC iteration loop early.
	// NULL means no abort checking (normal operation).
	std::atomic<bool>* decode_abort{nullptr};

};


#endif
