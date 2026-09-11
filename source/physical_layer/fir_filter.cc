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
#include <cstdint>

#if defined(__aarch64__)
#include <arm_neon.h>
#endif

#if (defined(__x86_64__) || defined(_M_X64)) && \
	(defined(__GNUC__) || defined(__clang__))
#include <immintrin.h>
#define MERCURY_FIR_X86_AVX2_EXACT 1
#else
#define MERCURY_FIR_X86_AVX2_EXACT 0
#endif

// Four independent real outputs share one AVX2 register. Keep multiply and
// add as separate operations so each lane has the same two binary64 rounding
// points and ascending tap order as the scalar authority. The exact kernel is
// default-on and runtime-qualified; unsupported CPUs retain the scalar path.
static inline bool fir_x86_avx2_exact_enabled()
{
#if MERCURY_FIR_X86_AVX2_EXACT
	static const int cpu_ok=[] {
		__builtin_cpu_init();
		return __builtin_cpu_supports("avx2") ? 1 : 0;
	}();
	if(!cpu_ok) return false;
	// Lever law: unset (or empty) means default-ON; `0` disables and restores
	// the scalar authority. Read per call (not cached) so a lever change made
	// by a test or a pinned A/B cell takes effect without a process restart;
	// one getenv per buffer-sized call is noise next to the O(nItems*nTaps)
	// kernel it guards.
	const char* e=std::getenv("MERCURY_FIR_AVX2_EXACT");
	return !(e && *e && atoi(e)==0);
#else
	return false;
#endif
}

// Lever/CPU state query for the exactness gate and A/B tooling: true iff the
// exact AVX2 kernels are selected for eligible calls in this process right
// now (the CPU supports AVX2 and the lever is not `0`).
bool fir_exact_avx2_active()
{
	return fir_x86_avx2_exact_enabled();
}

static inline bool fir_complex_ranges_overlap(const std::complex<double>* in,
	std::size_t in_count, const std::complex<double>* out, std::size_t out_count)
{
	const std::uintptr_t in_begin=reinterpret_cast<std::uintptr_t>(in);
	const std::uintptr_t in_end=in_begin+in_count*sizeof(*in);
	const std::uintptr_t out_begin=reinterpret_cast<std::uintptr_t>(out);
	const std::uintptr_t out_end=out_begin+out_count*sizeof(*out);
	return in_begin<out_end && out_begin<in_end;
}

#if defined(MERCURY_FIR_TEST_DISPATCH)
static std::uint64_t fir_complex_avx2_dispatches=0;
static std::uint64_t fir_decimate_complex_avx2_dispatches=0;

extern "C" void mercury_fir_test_reset_dispatch_counts()
{
	fir_complex_avx2_dispatches=0;
	fir_decimate_complex_avx2_dispatches=0;
}

extern "C" std::uint64_t mercury_fir_test_complex_avx2_dispatches()
{
	return fir_complex_avx2_dispatches;
}

extern "C" std::uint64_t mercury_fir_test_decimate_complex_avx2_dispatches()
{
	return fir_decimate_complex_avx2_dispatches;
}
#endif

#if MERCURY_FIR_X86_AVX2_EXACT
__attribute__((target("avx2")))
static void fir_apply_x86_avx2_exact(const double* in, double* out, int nItems,
									 int N, const double* coef)
{
	const int half=(N-1)/2;
	auto boundary=[&](int k) {
		const int i=k+half;
		int j_begin=i-nItems+1; if(j_begin<0) j_begin=0;
		int j_end=i+1; if(j_end>N) j_end=N;
		double acc=0.0;
		for(int j=j_begin;j<j_end;j++) acc += in[i-j]*coef[j];
		out[k]=acc;
	};
	if(nItems<N) { for(int k=0;k<nItems;k++) boundary(k); return; }
	for(int k=0;k<half;k++) boundary(k);
	const int steady_end=nItems-half;
	int k=half;
	for(;k+3<steady_end;k+=4)
	{
		__m256d acc=_mm256_setzero_pd();
		for(int j=0;j<N;j++)
		{
			__m256d x=_mm256_loadu_pd(&in[k+half-j]);
			__m256d product=_mm256_mul_pd(x,_mm256_set1_pd(coef[j]));
			acc=_mm256_add_pd(acc,product);
		}
		_mm256_storeu_pd(out+k,acc);
	}
	for(;k<steady_end;k++)
	{
		const int i=k+half; double acc=0.0;
		for(int j=0;j<N;j++) acc += in[i-j]*coef[j];
		out[k]=acc;
	}
	for(;k<nItems;k++) boundary(k);
}

// std::complex<double> is stored as adjacent real/imaginary doubles.  Pair two
// independent outputs in each AVX2 register so every lane follows the scalar
// accumulator's coefficient order and binary64 rounding points.  Coefficients
// are real, therefore no horizontal operation or complex reassociation is
// involved.
__attribute__((target("avx2")))
static int fir_apply_complex_steady_x86_avx2_exact(
	const std::complex<double>* in, std::complex<double>* out, int k,
	int steady_end, int N, int half, const double* coef)
{
	for(;k+3<steady_end;k+=4)
	{
		__m256d acc01=_mm256_setzero_pd();
		__m256d acc23=_mm256_setzero_pd();
		for(int j=0;j<N;j++)
		{
			const __m256d c=_mm256_set1_pd(coef[N-1-j]);
			const double* x01=reinterpret_cast<const double*>(&in[k-half+j]);
			const double* x23=reinterpret_cast<const double*>(&in[k-half+j+2]);
			const __m256d product01=_mm256_mul_pd(_mm256_loadu_pd(x01),c);
			const __m256d product23=_mm256_mul_pd(_mm256_loadu_pd(x23),c);
			acc01=_mm256_add_pd(acc01,product01);
			acc23=_mm256_add_pd(acc23,product23);
		}
		_mm256_storeu_pd(reinterpret_cast<double*>(out+k),acc01);
		_mm256_storeu_pd(reinterpret_cast<double*>(out+k+2),acc23);
	}
	return k;
}

__attribute__((target("avx2")))
static int fir_apply_decimate_complex_steady_x86_avx2_exact(
	const std::complex<double>* in, std::complex<double>* out, int m,
	int m_end_steady, int M, int N, int half, const double* coef)
{
	for(;m+3<=m_end_steady;m+=4)
	{
		__m256d acc01=_mm256_setzero_pd();
		__m256d acc23=_mm256_setzero_pd();
		const std::complex<double>* window0=&in[(m+0)*M-half];
		const std::complex<double>* window1=&in[(m+1)*M-half];
		const std::complex<double>* window2=&in[(m+2)*M-half];
		const std::complex<double>* window3=&in[(m+3)*M-half];
		for(int j=0;j<N;j++)
		{
			const __m256d c=_mm256_set1_pd(coef[N-1-j]);
			const __m128d x0=_mm_loadu_pd(reinterpret_cast<const double*>(window0+j));
			const __m128d x1=_mm_loadu_pd(reinterpret_cast<const double*>(window1+j));
			const __m128d x2=_mm_loadu_pd(reinterpret_cast<const double*>(window2+j));
			const __m128d x3=_mm_loadu_pd(reinterpret_cast<const double*>(window3+j));
			const __m256d x01=_mm256_insertf128_pd(_mm256_castpd128_pd256(x0),x1,1);
			const __m256d x23=_mm256_insertf128_pd(_mm256_castpd128_pd256(x2),x3,1);
			const __m256d product01=_mm256_mul_pd(x01,c);
			const __m256d product23=_mm256_mul_pd(x23,c);
			acc01=_mm256_add_pd(acc01,product01);
			acc23=_mm256_add_pd(acc23,product23);
		}
		_mm256_storeu_pd(reinterpret_cast<double*>(out+m),acc01);
		_mm256_storeu_pd(reinterpret_cast<double*>(out+m+2),acc23);
	}
	return m;
}
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
#if MERCURY_FIR_X86_AVX2_EXACT
	const bool exact_avx2 = fir_x86_avx2_exact_enabled() &&
		!fir_complex_ranges_overlap(in,nItems>0?(std::size_t)nItems:0,
			out,nItems>0?(std::size_t)nItems:0);
#endif

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
	int k = (half < nItems) ? half : nItems;
#if MERCURY_FIR_X86_AVX2_EXACT
	if(exact_avx2)
	{
#if defined(MERCURY_FIR_TEST_DISPATCH)
		const int first=k;
#endif
		k=fir_apply_complex_steady_x86_avx2_exact(in,out,k,steady_end,N,half,coef);
#if defined(MERCURY_FIR_TEST_DISPATCH)
		if(k!=first) ++fir_complex_avx2_dispatches;
#endif
	}
#endif
	for (; k < steady_end; k++)
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
#if MERCURY_FIR_X86_AVX2_EXACT
	const bool exact_avx2 = fir_x86_avx2_exact_enabled() &&
		!fir_complex_ranges_overlap(in,in_size>0?(std::size_t)in_size:0,
			out,out_size>0?(std::size_t)out_size:0);
#endif

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
#if MERCURY_FIR_X86_AVX2_EXACT
	if(exact_avx2)
	{
#if defined(MERCURY_FIR_TEST_DISPATCH)
		const int first=m;
#endif
		m=fir_apply_decimate_complex_steady_x86_avx2_exact(
			in,out,m,m_end_steady,M,N,half,coef);
#if defined(MERCURY_FIR_TEST_DISPATCH)
		if(m!=first) ++fir_decimate_complex_avx2_dispatches;
#endif
	}
#endif
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
	const std::uintptr_t in_begin=(std::uintptr_t)in;
	const std::uintptr_t in_end=in_begin+(nItems>0?(std::size_t)nItems*sizeof(double):0);
	const std::uintptr_t out_begin=(std::uintptr_t)out;
	const std::uintptr_t out_end=out_begin+(nItems>0?(std::size_t)nItems*sizeof(double):0);
	const bool overlap=in_begin<out_end && out_begin<in_end;
	if(fir_x86_avx2_exact_enabled() && !overlap)
	{
		if(nItems>0 && filter_nTaps>0)
			fir_apply_x86_avx2_exact(in,out,nItems,filter_nTaps,filter_coefficients);
		return;
	}

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
