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

#ifndef INC_FIR_FILTER_H_
#define INC_FIR_FILTER_H_

#include <cmath>
#include <complex>
#include <iostream>

#ifndef M_PI
#define M_PI          3.14159265358979323846  /* pi */
#endif

#define RECTANGULAR 0
#define HANNING 1
#define HAMMING 2
#define BLACKMAN 3

#define LPF 0
#define HPF 1
#define BPF 2

class cl_FIR
{
private:

	double* filter_coefficients;
	double filter_cut_frequency;


public:
	cl_FIR();
	~cl_FIR();

	// PRECOOK M3 (BUNDLE_FIELD_CHECKLIST PART 2.4): deep-copy the owning
	// filter_coefficients[filter_nTaps] + all scalars from `s`.
	void copy_from(const cl_FIR& s);
	// PRECOOK gate helper: byte-compare owning buffer(s)+sizing scalars vs `o`.
	// Returns NULL if byte-identical, else the name of the first differing field.
	const char* precook_deep_equal(const cl_FIR& o) const;
	// Owning-pointer class with no user copy-ctor → block the shallow default
	// (double-free of filter_coefficients). copy_from is the only safe path.
	cl_FIR(const cl_FIR&) = delete;
	cl_FIR& operator=(const cl_FIR&) = delete;

	void design();
	void apply(std::complex <double>* in, std::complex <double>* out, int nItems);
	void apply(double* in, double* out, int nItems);
	// Combined FIR + decimation: computes only the kept outputs (every Mth
	// sample of what cl_FIR::apply would produce). Bit-exact equivalent of
	// apply() followed by picking out[m*M], but does M× less work.
	// out_size = in_size / M (caller's responsibility).
	void apply_decimate(std::complex <double>* in, std::complex <double>* out,
	                    int in_size, int M);
	void deinit();

	int filter_window;
	double filter_transition_bandwidth;
	double lpf_filter_cut_frequency;
	double hpf_filter_cut_frequency;
	double sampling_frequency;
	int type;
	int filter_nTaps;
};

// True iff the byte-exact AVX2 FIR kernels are active for eligible calls in
// this process (x86-64 with AVX2, and MERCURY_FIR_AVX2_EXACT is not `0`).
// Lever law: unset (or empty) defaults ON; `0` disables. Builds without the
// AVX2 kernels always return false.
bool fir_exact_avx2_active();




#endif
