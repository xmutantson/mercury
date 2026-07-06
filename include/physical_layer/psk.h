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

	// PRECOOK M3 (BUNDLE_FIELD_CHECKLIST PART 3): deep-copy the owning
	// constellation[nSymbols] (content IS geometry) + D_buf/LLR_buf workspace +
	// scalars (nBits,nSymbols,var_floor) from `s`.
	void copy_from(const cl_psk& s);
	// PRECOOK gate helper: byte-compare owning buffers (constellation/D_buf/
	// LLR_buf) + sizing scalars vs `o`. NULL if identical, else first-diff field.
	const char* precook_deep_equal(const cl_psk& o) const;
	// Owning-pointer class → block the shallow default copy (double-free of
	// constellation/D_buf/LLR_buf). copy_from is the only safe path.
	cl_psk(const cl_psk&) = delete;
	cl_psk& operator=(const cl_psk&) = delete;


	void set_constellation(std::complex <double> *_constellation, int size);
	void set_predefined_constellation(int M);
	void deinit();
	void mod(const int *in,int nItems,std::complex <double> *out);
	void demod(const std::complex <double> *in,int nItems,float *out,float variance);
	// PAS / PCS LLR de-shaping demapper (fact-documents/data-flow-pas-shaping.md §4).
	// Identical to demod() but adds a per-output-bit a-priori log-prior
	// log_prior[k] = ln P(bit_k=0) − ln P(bit_k=1) to each bit LLR, accounting for
	// the non-uniform (Maxwell-Boltzmann) amplitude distribution the distribution
	// matcher imposed. log_prior is indexed by the SAME per-symbol output position
	// as demod's out (0..nBits-1, MSB-first); pass 0.0 for the (uniform) sign-bit
	// positions. log_prior has nBits entries (one per per-symbol bit position).
	// demod() itself is UNCHANGED so all production / non-PAS paths are byte-identical.
	void demod_pas(const std::complex <double> *in,int nItems,float *out,float variance,
	               const double* log_prior);
	// PAS / PCS constellation power re-normalization. set_constellation() normalizes
	// to unit AVERAGE power assuming a UNIFORM symbol distribution. Under PAS the
	// symbols are NON-uniform (Maxwell-Boltzmann amplitudes), so the actual average
	// power < 1 — which at a fixed Es/N0 would LOSE SNR. This rescales every point so
	// the shaped average power E[|x|²] = 1, spending the freed energy on a LARGER
	// minimum distance — that expansion IS the shaping gain. sym_prob[i] = the prior
	// probability of constellation index i (sum to 1); the scale is 1/sqrt(sum
	// sym_prob[i]*|constellation[i]|²) applied to the (already-unit-uniform-power)
	// constellation. Call AFTER set_predefined_constellation(MOD_64QAM). No effect on
	// non-PAS paths (only invoked when PCS is enabled).
	void rescale_shaped_power(const double* sym_prob);
	int  symbol_count() const { return nSymbols; }
	int  bits_per_symbol() const { return nBits; }
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
