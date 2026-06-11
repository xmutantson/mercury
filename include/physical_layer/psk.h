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

#ifndef INC_PSK_H_
#define INC_PSK_H_

#include <complex>
#include <cmath>

#define MOD_BPSK 2
#define MOD_QPSK 4
#define MOD_8PSK 8
#define MOD_16QAM 16
#define MOD_32QAM 32
#define MOD_64QAM 64

class cl_psk
{
private:
	std::complex <double>* constellation;
	int nBits;
	int nSymbols;
	float* D_buf;    // Pre-allocated demod workspace [nSymbols]
	float* LLR_buf;  // Pre-allocated demod workspace [nBits]
public:


	cl_psk();
	~cl_psk();


	void set_constellation(std::complex <double> *_constellation, int size);
	void set_predefined_constellation(int M);
	void deinit();
	void mod(const int *in,int nItems,std::complex <double> *out);
	void demod(const std::complex <double> *in,int nItems,float *out,float variance);
	// Turbo-EQ soft re-modulation (RESEARCH_turbo-eq.md §3/§4.3). From per-bit
	// a-priori LLRs (SAME order/sign as demod() output), emit the soft symbol mean
	// xbar_out[sym] = E[x] (virtual-pilot value) and variance v_out[sym] =
	// E|x|² − |xbar|² (its reliability) via an exact sum over the constellation.
	// nItems = number of LLRs (= nData*nBits); writes nItems/nBits symbols.
	void soft_remod(const float* llr, int nItems, std::complex<double>* xbar_out, double* v_out);
	// Nearest-constellation-point hard slicer (min Euclidean distance). Used by the
	// sparse-2D channel interpolator's optional DDCE pass (grid_sparse2d_estimator).
	// Returns (0,0) if the constellation is not initialized.
	std::complex <double> slice_nearest(std::complex <double> y) const;

	// Phase-2 validation flag (--psk-var-floor=F). Default 0.001 = HEAD (b806b76);
	// pre-IONOS was 0.05. Lower-bound on variance used in LLR computation.
	// See PHASE2_FLAGS_DESIGN.md §2.2.
	float var_floor;
};

#endif
