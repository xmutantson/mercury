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

#ifndef LDPC_DECODER_SPA_H_
#define LDPC_DECODER_SPA_H_

#include <cmath>
#include <atomic>
#include "physical_defines.h"

// Decoder selected by the one process-wide policy function below.  The enum is
// deliberately explicit rather than a pair of booleans so callers cannot
// accidentally request fixed-point SPA (which is not a valid implementation).
enum ldpc_decoder_kind
{
	LDPC_DECODER_SPA = 0,
	LDPC_DECODER_MINSUM = 1,
	LDPC_DECODER_MINSUM_FIXED = 2
};

// Resolve MERCURY_LDPC_MINSUM for one already-clamped, active configuration.
// Unset/"0" is production SPA, "1" is the historical global min-sum A/B arm,
// and "scoped" selects fixed-point min-sum only for the priced OFDM data set.
// Any unrecognized value fails closed to SPA.
ldpc_decoder_kind ldpc_decoder_policy_for_config(int configuration);
const char* ldpc_decoder_kind_name(ldpc_decoder_kind kind);

int decode_SPA(
		const float LLRi[],
		int LLRo[],
		int* C,
		int CWidth,
		int CWidthMax,
		int* V,
		int VWidth,
		int VWidthMax,
		int d[],
		int dWidth,
		double* R,
		double* Q,
		int* V_pos,
		int N,
		int K,
		int P,
		int nIteration_max,
		std::atomic<bool>* abort_flag = nullptr,
		double* app_llr = nullptr,  // Turbo-EQ: when non-null, receives the
		                            // a-posteriori LLR for all N coded bits
		                            // (RESEARCH_turbo-eq.md §4.2). Default null
		                            // = byte-identical for every existing caller.
		int early_term_mode = 0,    // feat/turnaround-eff (turnaround-eff.md §2):
		                            // 0=OFF (byte-identical, loop runs to the cap);
		                            // 1=#3 syndrome early-term (warmup=12,confirm=8);
		                            // 2=#1(c) eager speculative (warmup=8,confirm=6).
		                            // Both use floor=P/2 (swept lossless). On a
		                            // detector trip decode_SPA returns the canonical
		                            // FAIL sentinel nIteration_max+1.
		int* out_early_term_iter = nullptr, // measurement-only: real iter at trip,
		                            // else -1. No control-flow effect.
		ldpc_decoder_kind decoder_kind = LDPC_DECODER_SPA
);


#endif

/* F. R. Kschischang, B. J. Frey, and H. . Loeliger, “Factor graphs and the sum-product algorithm,” IEEE Transactions on Information Theory, vol. 47, no. 2, pp. 498–519, Feb 2001.
 * https://ieeexplore.ieee.org/document/910572
 */
