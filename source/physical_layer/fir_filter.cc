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

#include "physical_layer/fir_filter.h"
#include <cstdlib>

#if defined(__aarch64__)
#include <arm_neon.h>
#endif

// Four independent output accumulators expose the FIR's output-parallelism to
// scalar out-of-order cores and SIMD.  Keep a process-wide A/B escape hatch so
// the original implementation can be measured from the same binary.  The new
// path is default-on; every output still visits coefficients in ascending j
// order, preserving the original accumulation order bit-for-bit.
static inline bool fir_block4_enabled()
{
	static const int v = []{
		const char* e = std::getenv("MERCURY_FIR_BLOCK4");
		return (e && *e) ? atoi(e) : 1;
	}();
	return v != 0;
}

cl_FIR::cl_FIR()
{
	filter_window=0;
	filter_nTaps=0;
	filter_transition_bandwidth=0;
	filter_cut_frequency=0;
	lpf_filter_cut_frequency=0;
	hpf_filter_cut_frequency=0;
	sampling_frequency=0;
	type=LPF;

	filter_coefficients=NULL;
}

cl_FIR::~cl_FIR()
{
	deinit();
}


void cl_FIR::design()
{
	if(type==LPF || type==BPF)
	{
		filter_cut_frequency=lpf_filter_cut_frequency;
	}
	else if(type==HPF)
	{
		filter_cut_frequency=hpf_filter_cut_frequency;
	}

	filter_nTaps=(int)(4.0/(filter_transition_bandwidth/(sampling_frequency/2.0)));

	if(filter_nTaps%2==0)
	{
		filter_nTaps++;
	}
	filter_coefficients = new double[filter_nTaps];
	double sampling_interval=1.0/(sampling_frequency);
	double temp;

	filter_coefficients[filter_nTaps/2]=1;
	for(int i=0;i<filter_nTaps/2;i++)
	{
		temp=2*M_PI*filter_cut_frequency*(double)(filter_nTaps/2-i) *sampling_interval;

		filter_coefficients[i]=sin(temp)/temp;
		filter_coefficients[filter_nTaps-i-1]=filter_coefficients[i];
	}

	temp=0;
	for(int i=0;i<filter_nTaps;i++)
	{
		temp+=filter_coefficients[i];
	}

	for(int i=0;i<filter_nTaps;i++)
	{
		filter_coefficients[i]/=temp;
	}

	if(type==HPF) //SPECTRAL_INVERSION
	{
		for(int i=0;i<filter_nTaps;i++)
		{
			filter_coefficients[i]*=-1;
		}
		filter_coefficients[(int)(filter_nTaps-1)/2]+=1;
	}
	else if (type==BPF)
	{
		filter_cut_frequency=hpf_filter_cut_frequency;
		double *filter_coefficients_hpf = NULL;
		filter_coefficients_hpf= new double[filter_nTaps];
		if(filter_coefficients_hpf==NULL)
		{
			std::cout<<"FIR filter design error.. exiting"<<std::endl;
			exit(-5);
		}
		filter_coefficients_hpf[filter_nTaps/2]=1;  // Initialize center tap (matches LPF design at line 66)
		for(int i=0;i<filter_nTaps/2;i++)
		{
			temp=2*M_PI*filter_cut_frequency*(double)(filter_nTaps/2-i) *sampling_interval;

			filter_coefficients_hpf[i]=sin(temp)/temp;
			filter_coefficients_hpf[filter_nTaps-i-1]=filter_coefficients_hpf[i];
		}

		temp=0;
		for(int i=0;i<filter_nTaps;i++)
		{
			temp+=filter_coefficients_hpf[i];
		}

		for(int i=0;i<filter_nTaps;i++)
		{
			filter_coefficients_hpf[i]/=temp;
		}

		for(int i=0;i<filter_nTaps;i++)
		{
			filter_coefficients_hpf[i]*=-1;
		}
		filter_coefficients_hpf[(int)(filter_nTaps-1)/2]+=1;


		for(int i=0;i<filter_nTaps;i++)
		{
			filter_coefficients[i]+=filter_coefficients_hpf[i];
			filter_coefficients[i]/=2;
		}

		if(filter_coefficients_hpf!=NULL)
		{
			delete[] filter_coefficients_hpf;
		}

	}

	if(filter_window==HAMMING)
	{
		for(int i=0;i<filter_nTaps;i++)
		{
			filter_coefficients[i]*=0.54-0.46*cos(2.0*M_PI*(double)i/(filter_nTaps-1));
		}
	}
	else if(filter_window==HANNING)
	{
		for(int i=0;i<filter_nTaps;i++)
		{
			filter_coefficients[i]*=0.5-0.5*cos(2.0*M_PI*(double)i/(filter_nTaps-1));
		}
	}
	else if(filter_window==BLACKMAN)
	{
		for(int i=0;i<filter_nTaps;i++)
		{
			filter_coefficients[i]*=0.42-0.5*cos(2.0*M_PI*(double)i/filter_nTaps)+0.08*cos(4.0*M_PI*(double)i/filter_nTaps);
		}
	}
}

void cl_FIR::apply(std::complex <double>* in, std::complex <double>* out, int nItems)
{
	// Three-phase implementation: prologue (input underflow), steady-state
	// (no bounds checks — autovectorizes), epilogue (input overflow).
	// Hot loop: ~30 minutes of perf showed 19.7% CPU in this function on Pi.
	// Branch-free inner loop lets gcc -O3 emit NEON code on aarch64.
	const int N = filter_nTaps;
	const int half = (N - 1) / 2;
	const double* __restrict__ coef = filter_coefficients;

	// Phase 1: prologue (k < half) — input would underflow
	int prologue_end = (half < nItems) ? half : nItems;
	for (int k = 0; k < prologue_end; k++)
	{
		double acc_r = 0.0, acc_i = 0.0;
		int j_start = half - k;
		for (int j = j_start; j < N; j++)
		{
			int in_idx = k - half + j;
			acc_r += in[in_idx].real() * coef[N - 1 - j];
			acc_i += in[in_idx].imag() * coef[N - 1 - j];
		}
		out[k].real(acc_r);
		out[k].imag(acc_i);
	}

	// Phase 2: steady-state — no branches, hot loop
	int steady_end = nItems - half;
	for (int k = (half < nItems) ? half : nItems; k < steady_end; k++)
	{
		double acc_r = 0.0, acc_i = 0.0;
		const std::complex<double>* __restrict__ window = &in[k - half];
		for (int j = 0; j < N; j++)
		{
			acc_r += window[j].real() * coef[N - 1 - j];
			acc_i += window[j].imag() * coef[N - 1 - j];
		}
		out[k].real(acc_r);
		out[k].imag(acc_i);
	}

	// Phase 3: epilogue (k >= nItems - half) — input would overflow
	int k_start = (steady_end > half) ? steady_end : half;
	if (k_start < 0) k_start = 0;
	for (int k = k_start; k < nItems; k++)
	{
		double acc_r = 0.0, acc_i = 0.0;
		int j_end = nItems - (k - half);  // input range [k-half, k-half+j_end)
		if (j_end > N) j_end = N;
		for (int j = 0; j < j_end; j++)
		{
			int in_idx = k - half + j;
			acc_r += in[in_idx].real() * coef[N - 1 - j];
			acc_i += in[in_idx].imag() * coef[N - 1 - j];
		}
		out[k].real(acc_r);
		out[k].imag(acc_i);
	}
}

// Polyphase decimation: combined FIR + decimate-by-M in one pass.
// The old chain `apply(in, tmp, in_size)` + `rational_resampler(tmp, ..., M, DECIMATION)`
// computed in_size FIR outputs and threw away (M-1)/M of them. This computes
// only the in_size/M kept outputs directly. Same boundary semantics as apply()
// (zero-pad at edges). Bit-exact equivalent within FP rounding.
// Profile on Pi RX side: FIR was 92% of CPU; decimation by 8 should drop it ~8×.
void cl_FIR::apply_decimate(std::complex <double>* in, std::complex <double>* out,
                            int in_size, int M)
{
	const int N = filter_nTaps;
	const int half = (N - 1) / 2;
	const int out_size = in_size / M;
	const double* __restrict__ coef = filter_coefficients;

	// Steady-state range: m*M - half >= 0 AND m*M - half + N - 1 < in_size
	int m_start_steady = (half + M - 1) / M;
	int m_end_steady = (in_size - N + half) / M;  // largest m with full window
	if (m_end_steady > out_size - 1) m_end_steady = out_size - 1;

	// Phase 1: prologue (output samples whose input window underflows)
	for (int m = 0; m < m_start_steady && m < out_size; m++)
	{
		double acc_r = 0.0, acc_i = 0.0;
		int center = m * M;
		for (int j = 0; j < N; j++)
		{
			int in_idx = center - half + j;
			if (in_idx >= 0 && in_idx < in_size)
			{
				acc_r += in[in_idx].real() * coef[N - 1 - j];
				acc_i += in[in_idx].imag() * coef[N - 1 - j];
			}
		}
		out[m].real(acc_r);
		out[m].imag(acc_i);
	}

	// Phase 2: steady-state (no bounds checks → autovectorizes)
	// Process four independent outputs together. Each accumulator still visits
	// taps in exactly the same order as the scalar loop below; the blocking only
	// exposes independent dependency chains and reuses each coefficient load.
	// This is portable scalar C++ (no ISA-specific intrinsics).
	int m = m_start_steady;
	for (; m + 3 <= m_end_steady; m += 4)
	{
		double acc0_r = 0.0, acc0_i = 0.0;
		double acc1_r = 0.0, acc1_i = 0.0;
		double acc2_r = 0.0, acc2_i = 0.0;
		double acc3_r = 0.0, acc3_i = 0.0;
		const std::complex<double>* __restrict__ window0 = &in[(m + 0) * M - half];
		const std::complex<double>* __restrict__ window1 = &in[(m + 1) * M - half];
		const std::complex<double>* __restrict__ window2 = &in[(m + 2) * M - half];
		const std::complex<double>* __restrict__ window3 = &in[(m + 3) * M - half];
		for (int j = 0; j < N; j++)
		{
			const double c = coef[N - 1 - j];
			acc0_r += window0[j].real() * c;
			acc0_i += window0[j].imag() * c;
			acc1_r += window1[j].real() * c;
			acc1_i += window1[j].imag() * c;
			acc2_r += window2[j].real() * c;
			acc2_i += window2[j].imag() * c;
			acc3_r += window3[j].real() * c;
			acc3_i += window3[j].imag() * c;
		}
		out[m + 0].real(acc0_r);
		out[m + 0].imag(acc0_i);
		out[m + 1].real(acc1_r);
		out[m + 1].imag(acc1_i);
		out[m + 2].real(acc2_r);
		out[m + 2].imag(acc2_i);
		out[m + 3].real(acc3_r);
		out[m + 3].imag(acc3_i);
	}

	// Scalar cleanup for zero to three remaining steady-state outputs.
	for (; m <= m_end_steady; m++)
	{
		double acc_r = 0.0, acc_i = 0.0;
		const std::complex<double>* __restrict__ window = &in[m * M - half];
		for (int j = 0; j < N; j++)
		{
			acc_r += window[j].real() * coef[N - 1 - j];
			acc_i += window[j].imag() * coef[N - 1 - j];
		}
		out[m].real(acc_r);
		out[m].imag(acc_i);
	}

	// Phase 3: epilogue (output samples whose input window overflows)
	for (int m = m_end_steady + 1; m < out_size; m++)
	{
		double acc_r = 0.0, acc_i = 0.0;
		int center = m * M;
		for (int j = 0; j < N; j++)
		{
			int in_idx = center - half + j;
			if (in_idx >= 0 && in_idx < in_size)
			{
				acc_r += in[in_idx].real() * coef[N - 1 - j];
				acc_i += in[in_idx].imag() * coef[N - 1 - j];
			}
		}
		out[m].real(acc_r);
		out[m].imag(acc_i);
	}
}

void cl_FIR::apply(double* in, double* out, int nItems)
{
	// Exact baseline/revert arm for A/B measurement.
#if defined(__aarch64__)
	if(!fir_block4_enabled())
#endif
	{
		double acc;
		for(int i=0;i<(nItems+filter_nTaps-1);i++)
		{
			acc=0;
			for(int j=0;j<filter_nTaps;j++)
			{
				if((i-j)>=0 && (i-j)<nItems)
				{
					acc+=in[i-j]*filter_coefficients[j];
				}
			}

			if(i>=((int)(filter_nTaps-1)/2) && i<(nItems+(int)(filter_nTaps-1)/2))
			{
				out[i-(int)(filter_nTaps-1)/2]=acc;
			}
		}
#if defined(__aarch64__)
		return;
#endif
	}

#if !defined(__aarch64__)
	// The measured target is AArch64.  Keep every other architecture on the
	// original implementation until it has its own code-generation/parity proof.
	return;
#else
	const int N = filter_nTaps;
	if(nItems <= 0 || N <= 0)
		return;
	const int half = (N - 1) / 2;
	const double* coef = filter_coefficients;

	// Boundary outputs have a partial input window.  Iterate j in precisely the
	// same ascending order as the old convolution and merely hoist its bounds
	// checks out of the tap loop.
	auto apply_boundary = [&](int k) {
		const int i = k + half;
		int j_begin = i - nItems + 1;
		if(j_begin < 0) j_begin = 0;
		int j_end = i + 1;
		if(j_end > N) j_end = N;
		double acc = 0.0;
		for(int j=j_begin; j<j_end; j++)
			acc = std::fma(in[i-j], coef[j], acc);
		out[k] = acc;
	};

	// A full N-tap window exists only when nItems >= N.
	if(nItems < N)
	{
		for(int k=0; k<nItems; k++) apply_boundary(k);
		return;
	}

	for(int k=0; k<half; k++) apply_boundary(k);

	const int steady_end = nItems - half; // exclusive
	int k = half;
	for(; k+3<steady_end; k+=4)
	{
		// Lanes are four adjacent output samples.  Each lane receives one fused
		// multiply-add per j, in the same order as the former scalar FMADD loop.
		float64x2_t acc01 = vdupq_n_f64(0.0);
		float64x2_t acc23 = vdupq_n_f64(0.0);
		for(int j=0; j<N; j++)
		{
			const double* x = &in[k + half - j];
			acc01 = vfmaq_n_f64(acc01, vld1q_f64(x),     coef[j]);
			acc23 = vfmaq_n_f64(acc23, vld1q_f64(x + 2), coef[j]);
		}
		vst1q_f64(out + k,     acc01);
		vst1q_f64(out + k + 2, acc23);
	}

	// Zero-to-three full-window outputs left after blocking.
	for(; k<steady_end; k++)
	{
		double acc=0.0;
		const int i=k+half;
		for(int j=0; j<N; j++) acc = std::fma(in[i-j], coef[j], acc);
		out[k]=acc;
	}

	for(k=steady_end; k<nItems; k++) apply_boundary(k);
#endif
}

void cl_FIR::deinit()
{
	filter_window=0;
	filter_nTaps=0;
	filter_transition_bandwidth=0;
	filter_cut_frequency=0;
	sampling_frequency=0;

	if(filter_coefficients!=NULL)
	{
		delete[] filter_coefficients;
		filter_coefficients=NULL;
	}
}
