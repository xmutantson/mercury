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

#include "common/os_interop.h"
#include "physical_layer/ofdm.h"
#include "debug/canary_guard.h"
#include <algorithm>  // for std::swap in optimized FFT

// PocketFFT: high-performance FFT library (BSD license)
// Replaces hand-rolled Cooley-Tukey. ~2-3x faster for N=256.
#define POCKETFFT_NO_MULTITHREADING
#include "physical_layer/pocketfft_hdronly.h"

#include <map>
#include <vector>  // §20: per-bin power accumulator for base-pattern combining

namespace {
// FFT plan cache. pocketfft's c2c() builds a new plan on every call, allocating
// twiddle factors via sincos — profile on Pi showed pocketfft_c::cfftp ctor +
// sincos_2pibyn ctor at ~12% CPU. Caching plans by size eliminates that.
// Single-threaded access (Mercury FFT calls are all from the ARQ thread per
// the comment at line 146 — time_sync_mfsk and detect_ack_pattern never run
// concurrently). No mutex needed.
std::map<size_t, pocketfft::detail::pocketfft_c<double>>& fft_plan_cache()
{
	static std::map<size_t, pocketfft::detail::pocketfft_c<double>> cache;
	return cache;
}
const pocketfft::detail::pocketfft_c<double>& get_fft_plan(size_t n)
{
	auto& cache = fft_plan_cache();
	auto it = cache.find(n);
	if (it == cache.end())
	{
		it = cache.emplace(std::piecewise_construct,
		                   std::forward_as_tuple(n),
		                   std::forward_as_tuple(n)).first;
	}
	return it->second;
}
} // anonymous namespace


cl_ofdm::cl_ofdm()
{
	this->Nc=0;
	this->Nfft=0;
	this->Nsymb=0;
	this->gi=0;
	Ngi=0;
	ofdm_frame =NULL;
	ofdm_preamble=NULL;
	zero_padded_data=NULL;
	iffted_data=NULL;
	gi_removed_data=NULL;
	ffted_data=NULL;
	estimated_channel=NULL;
	estimated_channel_without_amplitude_restoration=NULL;
	time_sync_Nsymb=1;
	freq_offset_ignore_limit=0.1;
	start_shift=1;
	passband_start_sample=0;
	preamble_papr_cut=99;
	data_papr_cut=99;
	channel_estimator=ZERO_FORCE;
	LS_window_width=0;
	LS_window_hight=0;
	channel_estimator_amplitude_restoration=NO;
	noise_variance_estimate=0.01; // Safe default (SNR ~20dB)
	ls_nv_debug_enabled=false; // fix/cfg16-nv-restore: opt-in [LS-NV-DBG] logging
	ls_use_crosspilot_nv=false; // fix/cfg16-nv-restore: default = the fix (residual nv)
	// Optimized FFT tables
	fft_twiddle=NULL;
	fft_scratch=NULL;
	fft_bit_rev=NULL;
	fft_twiddle_size=0;
	// Pre-allocated passband_to_baseband buffers
	p2b_l_data=NULL;
	p2b_data_filtered=NULL;
	p2b_buffer_size=0;
	// Pre-allocated Nfft work buffers (Group A)
	work_buf_a=NULL;
	work_buf_b=NULL;
	// Pre-allocated time_sync_preamble buffers (Group B)
	tsync_corr_loc=NULL;
	tsync_corr_vals=NULL;
	tsync_corr_size=0;
	tsync_data=NULL;
	tsync_data_size=0;
	// Pre-allocated baseband_to_passband buffer (Group C)
	b2p_data_interpolated=NULL;
	b2p_buffer_size=0;
	// MFSK cross-correlation template (NB + WB).
	// sym_energy array sized 16 to cover the WB 16-symbol preamble
	// (data-flow-preamble_nSymb.md §H1).
	mfsk_corr_template=NULL;
	mfsk_corr_template_len=0;
	mfsk_corr_template_energy=0.0;
	mfsk_corr_template_nsymb=0;
	for(int i=0;i<16;i++) mfsk_corr_template_sym_energy[i]=0.0;
	// MFSK preamble parameters (discrete-match port, §14).
	mfsk_M=0;
	mfsk_nStreams=0;
	mfsk_preamble_nsymb=0;
	mfsk_preamble_match_threshold=0;
	for(int i=0;i<16;i++) mfsk_preamble_tones[i]=0;
	for(int i=0;i<4;i++) mfsk_stream_offsets[i]=0;
	// OFDM matched-filter template
	ofdm_corr_template=NULL;
	ofdm_corr_template_len=0;
	ofdm_corr_template_nsymb=0;
	ofdm_corr_template_energy=0.0;
	for(int i=0;i<16;i++) ofdm_corr_template_sym_energy[i]=0.0;
}

cl_ofdm::~cl_ofdm()
{
	this->deinit();
}

// Forward declaration — defined after butterfly variants below
void init_simd_dispatch();

void cl_ofdm::init(int Nfft, int Nc, int Nsymb, float gi)
{
	this->Nc=Nc;
	this->Nfft=Nfft;
	this->Nsymb=Nsymb;
	this->gi=gi;
	if(LS_window_width==0)
	{
		LS_window_width=Nc;
	}
	if(LS_window_hight==0)
	{
		LS_window_hight=Nsymb;
	}

	this->init();
}
void cl_ofdm::init()
{
	Ngi=Nfft*gi;

	ofdm_frame = CNEW(struct st_carrier, this->Nsymb*this->Nc, "ofdm.ofdm_frame");
	zero_padded_data=CNEW(std::complex<double>, Nfft, "ofdm.zero_padded_data");
	iffted_data=CNEW(std::complex<double>, Nfft, "ofdm.iffted_data");
	gi_removed_data=CNEW(std::complex<double>, Nfft, "ofdm.gi_removed_data");
	ffted_data=CNEW(std::complex<double>, Nfft, "ofdm.ffted_data");
	estimated_channel=CNEW(struct st_channel_complex, this->Nsymb*this->Nc, "ofdm.estimated_channel");
	estimated_channel_without_amplitude_restoration=CNEW(struct st_channel_complex, this->Nsymb*this->Nc, "ofdm.est_channel_noamp");
	ofdm_preamble = CNEW(struct st_carrier, this->preamble_configurator.Nsymb*this->Nc, "ofdm.ofdm_preamble");
	passband_start_sample=0;

	preamble_configurator.init(this->Nfft, this->Nc,this->ofdm_preamble, this->start_shift);
	pilot_configurator.init(this->Nfft, this->Nc,this->Nsymb,this->ofdm_frame, this->start_shift);

	// Initialize optimized FFT tables + SIMD dispatch
	init_fft_tables(this->Nfft);
	static bool simd_initialized = false;
	if (!simd_initialized) {
		init_simd_dispatch();
		simd_initialized = true;
	}

	// Pre-allocate shared Nfft work buffers (used by time_sync_mfsk and
	// detect_ack_pattern — never called concurrently)
	work_buf_a = CNEW(std::complex<double>, Nfft, "ofdm.work_buf_a");
	work_buf_b = CNEW(std::complex<double>, Nfft, "ofdm.work_buf_b");

	for(int i=0;i<Nsymb;i++)
	{
		for(int j=0;j<Nc;j++)
		{
			(estimated_channel+i*Nc+j)->value=1;
		}
	}
}

void cl_ofdm::deinit()
{
	this->Ngi=0;
	this->Nc=0;
	this->Nfft=0;
	this->Nsymb=0;
	this->gi=0;

	pilot_configurator.Dx=0;
	pilot_configurator.Dy=0;
	pilot_configurator.first_row=0;
	pilot_configurator.last_row=0;
	pilot_configurator.first_col=0;
	pilot_configurator.second_col=0;
	pilot_configurator.last_col=0;
	pilot_configurator.boost=0;

	preamble_configurator.Nsymb=0;
	preamble_configurator.nIdentical_sections=0;
	preamble_configurator.modulation=0;
	preamble_configurator.boost=0;


	CDELETE(ofdm_frame);
	CDELETE(ofdm_preamble);
	CDELETE(zero_padded_data);
	CDELETE(iffted_data);
	CDELETE(gi_removed_data);
	CDELETE(ffted_data);
	CDELETE(estimated_channel);
	CDELETE(estimated_channel_without_amplitude_restoration);
	if(p2b_l_data!=NULL){delete[] p2b_l_data; p2b_l_data=NULL;}
	if(p2b_data_filtered!=NULL){delete[] p2b_data_filtered; p2b_data_filtered=NULL;}
	p2b_buffer_size=0;
	CDELETE(work_buf_a);
	CDELETE(work_buf_b);
	if(tsync_corr_loc!=NULL){delete[] tsync_corr_loc; tsync_corr_loc=NULL;}
	if(tsync_corr_vals!=NULL){delete[] tsync_corr_vals; tsync_corr_vals=NULL;}
	tsync_corr_size=0;
	if(tsync_data!=NULL){delete[] tsync_data; tsync_data=NULL;}
	tsync_data_size=0;
	if(b2p_data_interpolated!=NULL){delete[] b2p_data_interpolated; b2p_data_interpolated=NULL;}
	b2p_buffer_size=0;
	CDELETE(mfsk_corr_template);
	mfsk_corr_template_len=0;
	mfsk_corr_template_energy=0.0;
	mfsk_corr_template_nsymb=0;
	for(int i=0;i<16;i++) mfsk_corr_template_sym_energy[i]=0.0;
	CDELETE(ofdm_corr_template);
	ofdm_corr_template_len=0;
	ofdm_corr_template_nsymb=0;
	ofdm_corr_template_energy=0.0;
	for(int i=0;i<16;i++) ofdm_corr_template_sym_energy[i]=0.0;

	pilot_configurator.deinit();
	preamble_configurator.deinit();
	deinit_fft_tables();
}

// ============================================================================
// ============================================================================
// PocketFFT-based FFT (replaces hand-rolled SIMD-dispatched Cooley-Tukey)
// PocketFFT uses mixed-radix decomposition + auto-vectorization.
// ============================================================================

void init_simd_dispatch()
{
	printf("[FFT] Using PocketFFT (mixed-radix, auto-vectorized)\n");
	fflush(stdout);
}

void cl_ofdm::init_fft_tables(int n)
{
	if (n <= 0 || (n & (n-1)) != 0) {
		// n must be power of 2
		fft_twiddle_size = 0;
		return;
	}

	fft_twiddle_size = n;

	// Allocate twiddle factors (only need n/2 for radix-2)
	fft_twiddle = CNEW(std::complex<double>, n/2, "ofdm.fft_twiddle");
	for (int k = 0; k < n/2; k++) {
		double angle = -2.0 * M_PI * k / n;
		fft_twiddle[k] = std::complex<double>(cos(angle), sin(angle));
	}

	// Allocate scratch buffer
	fft_scratch = CNEW(std::complex<double>, n, "ofdm.fft_scratch");

	// Build bit-reversal permutation table
	fft_bit_rev = CNEW(int, n, "ofdm.fft_bit_rev");
	int bits = 0;
	for (int temp = n; temp > 1; temp >>= 1) bits++;

	for (int i = 0; i < n; i++) {
		int rev = 0;
		for (int j = 0; j < bits; j++) {
			if (i & (1 << j)) {
				rev |= (1 << (bits - 1 - j));
			}
		}
		fft_bit_rev[i] = rev;
	}
}

void cl_ofdm::deinit_fft_tables()
{
	CDELETE(fft_twiddle);
	CDELETE(fft_scratch);
	CDELETE(fft_bit_rev);
	fft_twiddle_size = 0;
}

// Optimized FFT — PocketFFT with cached plans.
// pocketfft::c2c() reconstructs the plan on every call; we cache by size.
// std::complex<double> and pocketfft::detail::cmplx<double> have identical
// layout ({real, imag}), so reinterpret_cast is safe.
void cl_ofdm::_fft_fast(std::complex<double>* v, int n)
{
	const auto& plan = get_fft_plan((size_t)n);
	plan.exec(reinterpret_cast<pocketfft::detail::cmplx<double>*>(v), 1.0, true);
}

// Optimized IFFT — PocketFFT with cached plans.
void cl_ofdm::_ifft_fast(std::complex<double>* v, int n)
{
	const auto& plan = get_fft_plan((size_t)n);
	plan.exec(reinterpret_cast<pocketfft::detail::cmplx<double>*>(v), 1.0, false);
}

void cl_ofdm::zero_padder(std::complex <double>* in, std::complex <double>* out)
{
	for(int j=0;j<Nc/2;j++)
	{
		out[j+Nfft-Nc/2]=in[j];
	}

	for(int j=0;j<start_shift;j++)
	{
		out[j]=std::complex <double>(0,0);
	}

	for(int j=Nc/2+start_shift;j<Nfft-Nc/2;j++)
	{
		out[j]=std::complex <double>(0,0);
	}

	for(int j=Nc/2;j<Nc;j++)
	{
		out[j-Nc/2+start_shift]=in[j];
	}
}
void cl_ofdm::zero_depadder(std::complex <double>* in, std::complex <double>* out)
{
	for(int j=0;j<Nc/2;j++)
	{
		out[j]=in[j+Nfft-Nc/2];
	}
	for(int j=Nc/2;j<Nc;j++)
	{
		out[j]=in[j-Nc/2+start_shift];
	}
}
void cl_ofdm::gi_adder(std::complex <double>* in, std::complex <double>* out)
{
	for(int j=0;j<Nfft;j++)
	{
		out[j+Ngi]=in[j];
	}
	for(int j=0;j<Ngi;j++)
	{
		out[j]=in[j+Nfft-Ngi];
	}
}
void cl_ofdm::gi_remover(std::complex <double>* in, std::complex <double>* out)
{
	for(int j=0;j<Nfft;j++)
	{
		out[j]=in[j+Ngi];
	}
}

void cl_ofdm::fft(std::complex <double>* in, std::complex <double>* out)
{
	// Single std::copy (compiler emits memcpy), then in-place FFT with the
	// 1/Nfft scale folded into pocketfft's exec factor — removes the second
	// pass over Nfft samples that the explicit divide loop did.
	std::copy(in, in + Nfft, out);
	const auto& plan = get_fft_plan((size_t)Nfft);
	plan.exec(reinterpret_cast<pocketfft::detail::cmplx<double>*>(out),
	          1.0 / (double)Nfft, true);
}
void cl_ofdm::fft(std::complex <double>* in, std::complex <double>* out, int _Nfft)
{
	std::copy(in, in + _Nfft, out);
	const auto& plan = get_fft_plan((size_t)_Nfft);
	plan.exec(reinterpret_cast<pocketfft::detail::cmplx<double>*>(out),
	          1.0 / (double)_Nfft, true);
}

void cl_ofdm::_fft(std::complex <double> *v, int n)
{
	if(n>1) {
		std::complex <double> *tmp=new std::complex <double>[n];
		int k,m;    std::complex <double> z, w, *vo, *ve;
		ve = tmp; vo = tmp+n/2;
		for(k=0; k<n/2; k++) {
			ve[k] = v[2*k];
			vo[k] = v[2*k+1];
		}
		_fft( ve, n/2 );
		_fft( vo, n/2 );
		for(m=0; m<n/2; m++) {
			w.real( cos(2*M_PI*m/(double)n));
			w.imag( -sin(2*M_PI*m/(double)n));
			z.real( w.real()*vo[m].real() - w.imag()*vo[m].imag());
			z.imag( w.real()*vo[m].imag() + w.imag()*vo[m].real());
			v[  m  ].real( ve[m].real() + z.real());
			v[  m  ].imag( ve[m].imag() + z.imag());
			v[m+n/2].real( ve[m].real() - z.real());
			v[m+n/2].imag( ve[m].imag() - z.imag());
		}
		if(tmp!=NULL)
		{
			delete[] tmp;
		}
	}
	//Ref:Wickerhauser, Mladen Victor,Mathematics for Multimedia, Birkhäuser Boston, January 2010, DOI: 10.1007/978-0-8176-4880-0, ISBNs 978-0-8176-4880-0, 978-0-8176-4879-4
	//https://www.math.wustl.edu/~victor/mfmm/
}

void cl_ofdm::ifft(std::complex <double>* in, std::complex <double>* out)
{
	std::copy(in, in + Nfft, out);
	const auto& plan = get_fft_plan((size_t)Nfft);
	plan.exec(reinterpret_cast<pocketfft::detail::cmplx<double>*>(out), 1.0, false);
}

void cl_ofdm::ifft(std::complex <double>* in, std::complex <double>* out,int _Nfft)
{
	std::copy(in, in + _Nfft, out);
	const auto& plan = get_fft_plan((size_t)_Nfft);
	plan.exec(reinterpret_cast<pocketfft::detail::cmplx<double>*>(out), 1.0, false);
}

void cl_ofdm::_ifft(std::complex <double>* v,int n)
{
	if(n>1) {
		std::complex <double> *tmp=new std::complex <double>[n];
		int k,m;    std::complex <double> z, w, *vo, *ve;
		ve = tmp; vo = tmp+n/2;
		for(k=0; k<n/2; k++) {
			ve[k] = v[2*k];
			vo[k] = v[2*k+1];
		}
		_ifft( ve, n/2);
		_ifft( vo, n/2);
		for(m=0; m<n/2; m++) {
			w.real( cos(2*M_PI*m/(double)n));
			w.imag( sin(2*M_PI*m/(double)n));
			z.real( w.real()*vo[m].real() - w.imag()*vo[m].imag());
			z.imag( w.real()*vo[m].imag() + w.imag()*vo[m].real());
			v[  m  ].real( ve[m].real() + z.real());
			v[  m  ].imag( ve[m].imag() + z.imag());
			v[m+n/2].real( ve[m].real() - z.real());
			v[m+n/2].imag( ve[m].imag() - z.imag());
		}
		if(tmp!=NULL)
		{
			delete[] tmp;
		}
	}
	//Ref:Wickerhauser, Mladen Victor,Mathematics for Multimedia, Birkhäuser Boston, January 2010, DOI: 10.1007/978-0-8176-4880-0, ISBNs 978-0-8176-4880-0, 978-0-8176-4879-4
	//https://www.math.wustl.edu/~victor/mfmm/
}

double cl_ofdm::carrier_sampling_frequency_sync(std::complex <double>*in, double carrier_freq_width, int preamble_nSymb, double sampling_frequency)
{
	double frequency_offset_prec=0;

	std::complex <double> p1,p2,mul;
	std::complex <double> frame[Nfft];
	std::complex <double> frame_fft[Nfft],frame_depadded1[Nfft],frame_depadded2[Nfft];

	if(preamble_nSymb/2==0)
	{
		preamble_nSymb=1;
	}
	else
	{
		preamble_nSymb/=2;
	}

	// Repetition period L = Nfft / nIdentical_sections:
	//   nIS=2 (NB, every-2nd): L=128, correlate halves
	//   nIS=4 (WB, every-4th): L=64, correlate quarters
	int nIS = preamble_configurator.nIdentical_sections;
	int L = Nfft / nIS;

	mul=0;
	for(int j=0;j<preamble_nSymb;j++)
	{
		// First repetition period [0..L-1], repeated nIS times to fill Nfft
		for(int rep=0;rep<nIS;rep++)
			for(int i=0;i<L;i++)
				frame[rep*L+i]=*(in+j*(Nfft+Ngi)+i);

		fft(frame,frame_fft);
		zero_depadder(frame_fft,frame_depadded1);

		// Second repetition period [L..2L-1], repeated nIS times
		for(int rep=0;rep<nIS;rep++)
			for(int i=0;i<L;i++)
				frame[rep*L+i]=*(in+j*(Nfft+Ngi)+i+L);

		fft(frame,frame_fft);
		zero_depadder(frame_fft,frame_depadded2);

		for(int i=0;i<Nc;i++)
		{
			mul+=conj(frame_depadded2[i])*frame_depadded1[i];
		}
	}

	// Moose formula: phase = 2π × ε × L / Nfft  →  ε = phase × Nfft / (2π × L)
	// Simplifies to: ε = phase × nIS / (2π)
	// For nIS=2: ε = phase / π  (±1 subcarrier range)
	// For nIS=4: ε = 2·phase / π  (±2 subcarrier range)
	frequency_offset_prec = get_angle(mul) * nIS / (2.0 * M_PI);

	// float sampling_frequency_offset= -frequency_offset_prec*carrier_freq_width /sampling_frequency;

	return (frequency_offset_prec*carrier_freq_width);

	//Ref1: P. H. Moose, "A technique for orthogonal frequency division multiplexing frequency offset correction," in IEEE Transactions on Communications, vol. 42, no. 10, pp. 2908-2914, Oct. 1994, doi: 10.1109/26.328961.
	//Ref2: T. M. Schmidl and D. C. Cox, "Robust frequency and timing synchronization for OFDM," in IEEE Transactions on Communications, vol. 45, no. 12, pp. 1613-1621, Dec. 1997, doi: 10.1109/26.650240.
	//Ref3: M. Speth, S. Fechtel, G. Fock and H. Meyr, "Optimum receiver design for OFDM-based broadband transmission .II. A case study," in IEEE Transactions on Communications, vol. 49, no. 4, pp. 571-578, April 2001, doi: 10.1109/26.917759.
}

double cl_ofdm::carrier_frequency_sync_nb(std::complex<double>* in, double carrier_freq_width, int preamble_nSymb)
{
	/*
	 * Cross-symbol phase progression frequency estimator for narrowband OFDM.
	 *
	 * NB preamble uses ALL subcarriers (Nc=10), which breaks Moose's
	 * half-symbol repetition assumption (requires even-only bins).
	 *
	 * Instead, measure phase rotation between adjacent preamble symbols:
	 * 1. FFT each preamble symbol, remove known modulation → channel estimate
	 * 2. Cross-correlate adjacent symbols: C = Σ H[sym+1] × conj(H[sym])
	 * 3. Phase of C = 2π × freq_offset × T_symbol
	 *
	 * Capture range: ±fs/(2×Nofdm) = ±22 Hz for NB (Nfft=256, Ngi=16).
	 * 10 subcarriers × (nSymb-1) pairs gives robust averaging.
	 *
	 * Input convention matches Moose: in = &baseband_data[Ngi]
	 * Internal access: in[sym * (Nfft+Ngi) + k] for k=0..Nfft-1
	 */

	int Nofdm = Nfft + Ngi;
	std::complex<double> fft_in[256];
	std::complex<double> fft_out[256];
	std::complex<double> depadded[256];
	std::complex<double> H_prev[256];
	std::complex<double> H_cur[256];

	std::complex<double> C(0.0, 0.0);
	double energy_total = 0.0;

	for (int sym = 0; sym < preamble_nSymb; sym++)
	{
		// FFT this preamble symbol
		for (int k = 0; k < Nfft; k++)
			fft_in[k] = in[sym * Nofdm + k];

		fft(fft_in, fft_out, Nfft);
		zero_depadder(fft_out, depadded);

		// Remove known modulation → raw channel estimate
		for (int k = 0; k < Nc; k++)
		{
			if (ofdm_preamble[sym * Nc + k].type == PREAMBLE)
				H_cur[k] = depadded[k] * std::conj(ofdm_preamble[sym * Nc + k].value);
			else
				H_cur[k] = std::complex<double>(0.0, 0.0);

			energy_total += std::norm(H_cur[k]);
		}

		// Cross-symbol correlation (sym >= 1)
		if (sym > 0)
		{
			for (int k = 0; k < Nc; k++)
				C += H_cur[k] * std::conj(H_prev[k]);
		}

		for (int k = 0; k < Nc; k++)
			H_prev[k] = H_cur[k];
	}

	// Confidence gate: reject if correlation is weak relative to energy
	double C_mag = std::abs(C);
	if (energy_total < 1e-10 || C_mag / energy_total < 0.05)
		return 0.0;

	// passband_to_baseband uses exp(+j×2πfc×t), so a carrier offset δ Hz
	// produces baseband phase exp(-j×2πδ×t). Cross-symbol phase is -2πδ×T.
	// Therefore: δ = -arg(C) / (2π × T_symbol)
	// T_symbol = Nofdm / fs_base, fs_base = carrier_freq_width × Nfft
	double phase = std::arg(C);
	double freq_offset = -phase * carrier_freq_width * (double)Nfft / (2.0 * M_PI * (double)Nofdm);

	return freq_offset;
}

double cl_ofdm::carrier_frequency_sync_wb_mfsk(std::complex<double>* in,
                                               double carrier_freq_width,
                                               int preamble_nSymb,
                                               const int* preamble_tones,
                                               int M_tones,
                                               int nStreams,
                                               const int* stream_offsets)
{
	/*
	 * Mini-Moose CFO refinement for WB MFSK data preamble.
	 *
	 * Background: mercury's MFSK RX path previously had NO fine CFO sync;
	 * the line `if(M == MOD_MFSK) freq_offset_measured = 0;` in
	 * cl_telecom_system::receive_msg discarded whatever the OFDM Moose
	 * estimators left behind. Per fact-documents/data-preamble-port-research.md
	 * §17, this caused real signal energy to leak into the mirror bin (which
	 * the discrete-match detector accepted as load-bearing). Refining the
	 * residual CFO at the start of every MFSK frame brings the energy back
	 * to the expected bin.
	 *
	 * Algorithm (cross-half-symbol phase, mirror of carrier_sampling_frequency_sync's
	 * WB-OFDM Moose):
	 *   1. For each preamble symbol s, expected CW tone at baseband freq
	 *      f_s = bin_index * fs_base / Nfft, where bin_index is the centered
	 *      FFT bin for stream_offsets[st] + preamble_tones[s].
	 *   2. De-rotate the received symbol by exp(-j*2π*f_s*t) so a residual
	 *      CFO produces a slow linear phase ramp.
	 *   3. Sum the de-rotated samples in the FIRST half (skip Ngi prefix) and
	 *      the SECOND half of the Nfft window.
	 *   4. C_st_s = (B - half-sum) * conj(A - half-sum).
	 *   5. Accumulate C across all streams and all symbols.
	 *   6. arg(C) / (2π × T_half_sym) = residual CFO in Hz.
	 *
	 * Capture range: ±fs_base / (2 × Nfft/2) = ±carrier_freq_width Hz.
	 *   WB ROBUST_0 (Nfft=256, Ngi=36, fs_base=3000 Hz): ±46.875 Hz.
	 *   Capture is wide enough for crystal mismatch (~11 Hz typical per
	 *   mercury memory) + coarse-sync residual (≤5 Hz per
	 *   phase-b-mfsk-connect-research §6.7) → ~16 Hz worst case, well
	 *   inside the capture range.
	 *
	 * Confidence gate: returns 0 if |C| / energy_total < 0.05 (mirror of
	 * carrier_frequency_sync_nb's gate).
	 *
	 * Inputs:
	 *   in              : baseband_data starting at the preamble (decimated rate).
	 *                     in[s*Nofdm + Ngi + i] is the i-th Nfft-window sample
	 *                     of preamble symbol s, for i in 0..Nfft-1.
	 *   carrier_freq_width: subcarrier spacing in Hz = fs_base / Nfft.
	 *   preamble_nSymb  : number of preamble symbols available (typically 16
	 *                     for WB ROBUST_0/1/2 post-§14).
	 *   preamble_tones  : array of tone indices, one per preamble symbol.
	 *                     Same layout as cl_mfsk::preamble_tones[].
	 *   M_tones         : MFSK alphabet size (used only for sanity guards).
	 *   nStreams        : number of MFSK streams (typically 1 for WB ROBUST_0).
	 *   stream_offsets  : array of stream-band starting subcarriers, one per
	 *                     stream. Same layout as cl_mfsk::stream_offsets[].
	 *
	 * Returns: residual CFO in Hz, or 0.0 on low confidence / invalid input.
	 *
	 * Sanity guards: returns 0 if any of preamble_nSymb / M_tones / nStreams
	 * is ≤ 0, mirroring the pre-init guard in time_sync_mfsk_corr.
	 *
	 * Reference: P. H. Moose, "A technique for orthogonal frequency division
	 * multiplexing frequency offset correction," IEEE TCOM 1994 (same as
	 * carrier_sampling_frequency_sync's cite). The half-symbol-baseline
	 * cross-correlation technique is the same; only the modulation it acts
	 * on differs (CW tones with known frequency offsets vs every-other-SC
	 * Schmidl-Cox repetition).
	 */

	if (preamble_nSymb <= 0 || M_tones <= 0 || nStreams <= 0) return 0.0;
	if (preamble_tones == NULL || stream_offsets == NULL) return 0.0;
	if (Nfft <= 0) return 0.0;
	int half = Nfft / 2;
	if (half <= 0) return 0.0;

	// fs_base (the decimated/base sample rate the caller's buffer is at):
	// carrier_freq_width = fs_base / Nfft, so fs_base = carrier_freq_width * Nfft.
	double fs_base = carrier_freq_width * (double)Nfft;
	if (fs_base <= 0.0) return 0.0;

	// Stream-bin geometry mirrors time_sync_mfsk_corr (ofdm.cc:3105-3107):
	//   sub = stream_offsets[st] + tone
	//   bin = (sub < Nc/2) ? (Nfft - Nc/2 + sub) : (start_shift + (sub - Nc/2))
	// The "centered" subcarrier index relative to FFT DC is:
	//   k_centered = bin if bin < Nfft/2 else bin - Nfft
	// which corresponds to baseband freq k_centered * fs_base / Nfft.
	int Nc_half = Nc / 2;

	int Nofdm = Nfft + Ngi;

	std::complex<double> C(0.0, 0.0);
	double energy_total = 0.0;

	for (int s = 0; s < preamble_nSymb; s++)
	{
		int tone = preamble_tones[s];
		if (tone < 0 || tone >= M_tones) continue;

		for (int st = 0; st < nStreams; st++)
		{
			int sub = stream_offsets[st] + tone;
			int expected_bin = (sub < Nc_half)
				? (Nfft - Nc_half + sub)
				: (start_shift + (sub - Nc_half));
			// Map FFT bin to a centered/signed bin index in (-Nfft/2, Nfft/2].
			int k_centered = (expected_bin < Nfft / 2)
				? expected_bin
				: expected_bin - Nfft;
			double f_tone = (double)k_centered * fs_base / (double)Nfft;

			// De-rotate the received symbol by exp(-j*2π*f_tone*t) so the
			// expected tone collapses to DC; residual CFO becomes the only
			// remaining phase rotation. Sum within each half-Nfft slot
			// (post-Ngi). The two half-sums correlate to extract the
			// cross-half phase.
			//
			// The de-rotation phase advances by 2π*f_tone/fs_base per
			// sample; build it incrementally to avoid sincos in the inner
			// loop.
			double angle_step = -2.0 * M_PI * f_tone / fs_base;
			double pr_init = 1.0;
			double pi_init = 0.0;
			double sr = std::cos(angle_step);
			double si = std::sin(angle_step);

			std::complex<double> A(0.0, 0.0);
			std::complex<double> B(0.0, 0.0);
			double pr = pr_init;
			double pi = pi_init;

			int base_idx = s * Nofdm + Ngi;

			for (int i = 0; i < half; i++)
			{
				double sample_r = in[base_idx + i].real();
				double sample_i = in[base_idx + i].imag();
				// Multiply (sample) * (pr + j*pi) = de-rotated sample
				double dr = sample_r * pr - sample_i * pi;
				double di = sample_r * pi + sample_i * pr;
				A.real(A.real() + dr);
				A.imag(A.imag() + di);
				// Advance phase recurrence.
				double npr = pr * sr - pi * si;
				double npi = pr * si + pi * sr;
				pr = npr;
				pi = npi;
			}
			for (int i = half; i < Nfft; i++)
			{
				double sample_r = in[base_idx + i].real();
				double sample_i = in[base_idx + i].imag();
				double dr = sample_r * pr - sample_i * pi;
				double di = sample_r * pi + sample_i * pr;
				B.real(B.real() + dr);
				B.imag(B.imag() + di);
				double npr = pr * sr - pi * si;
				double npi = pr * si + pi * sr;
				pr = npr;
				pi = npi;
			}

			std::complex<double> contrib = B * std::conj(A);
			C += contrib;
			energy_total += std::norm(A) + std::norm(B);
		}
	}

	if (!std::isfinite(C.real()) || !std::isfinite(C.imag())) return 0.0;
	if (!std::isfinite(energy_total) || energy_total <= 0.0) return 0.0;

	// Confidence gate: low |C|/energy ratio means the tones are not present
	// or the channel is noise-dominated. Mirrors carrier_frequency_sync_nb
	// (ofdm.cc:600). 0.05 chosen identical to the NB estimator's threshold.
	double C_mag = std::abs(C);
	if (C_mag / energy_total < 0.05) return 0.0;

	// arg(C) = 2π * CFO * T_half_sym, where T_half_sym = half / fs_base.
	// → CFO = arg(C) * fs_base / (2π * half).
	// Equivalent rewrite using carrier_freq_width and Nfft (matches the
	// units pattern used in carrier_frequency_sync_nb and
	// carrier_sampling_frequency_sync):
	//   CFO = arg(C) * carrier_freq_width * Nfft / (2π * half)
	//       = arg(C) * carrier_freq_width * Nfft / (π * Nfft)
	//       = arg(C) * carrier_freq_width / π
	double phase = std::arg(C);
	double freq_offset = phase * carrier_freq_width * (double)Nfft
	                   / (2.0 * M_PI * (double)half);

	if (!std::isfinite(freq_offset)) return 0.0;
	return freq_offset;
}

double cl_ofdm::carrier_frequency_sync_wb_ctrl(std::complex<double>* in,
                                               double carrier_freq_width,
                                               int pattern_nsymb,
                                               int sym_start_offset_samples,
                                               const int* pattern_tones,
                                               int pattern_len,
                                               int tone_hop_step,
                                               int M_tones,
                                               int nStreams,
                                               const int* stream_offsets)
{
	/*
	 * Mini-Moose CFO refinement for WB MFSK control-frame patterns.
	 *
	 * Background: the data-preamble mini-Moose (carrier_frequency_sync_wb_mfsk
	 * at ofdm.cc:613) refines residual CFO using the 16-symbol data preamble.
	 * The same half-symbol cross-correlation math works on any pattern of
	 * known CW tones — including the 16-symbol ACK / CONNECT base patterns
	 * used by `detect_ack_pattern`. Refining residual CFO before the ctrl-
	 * suffix is decoded lets the suffix-FFT-bin demap operate on
	 * energy-aligned signals — the same mechanism that produces the §17 data-
	 * preamble win.
	 *
	 * v1 of this work (feat/mini-moose-ctrl, dropped) used the apply formula
	 * `effective_carrier + residual` and regressed -40% on hardware (§22).
	 * The WB MFSK estimator returns the OPPOSITE sign of the actual baseband
	 * residual under the realistic real-passband injection model (§23.11.1);
	 * the apply formula must be `effective_carrier - residual` to cancel
	 * (not double) the residual. The caller of this function MUST use the
	 * `-` sign in its `passband_to_baseband_decimated` re-mix.
	 *
	 * Algorithm (identical to carrier_frequency_sync_wb_mfsk, two diffs):
	 *   diff (a): the pattern starts at sym_start_offset_samples in the input
	 *             buffer (best_offset from detect_ack_pattern), not at sample 0.
	 *   diff (b): per-symbol expected tone uses pattern hopping:
	 *             actual_tone = (pattern_tones[s % pattern_len]
	 *                            + s * tone_hop_step) % M_tones.
	 * Everything else (half-symbol Hadamard sums, de-rotation recurrence,
	 * C = B·conj(A), 0.05 confidence gate, phase-to-Hz conversion, NaN
	 * guards) is identical to the data-preamble estimator. The capture
	 * range is the same ±carrier_freq_width Hz (≈ ±46.875 Hz at WB ROBUST_0).
	 *
	 * Inputs:
	 *   in                       : baseband buffer at decimated rate, the
	 *                              same buffer detect_ack_pattern read.
	 *   carrier_freq_width       : subcarrier spacing in Hz = fs_base / Nfft.
	 *   pattern_nsymb            : number of pattern symbols to integrate
	 *                              over (16 for WB ACK / CONNECT).
	 *   sym_start_offset_samples : start-of-pattern offset in decimated
	 *                              samples (best_offset from
	 *                              detect_ack_pattern).
	 *   pattern_tones            : base tone sequence (ack_tones,
	 *                              connect_tones, etc.).
	 *   pattern_len              : length of the base tone sequence (8 for
	 *                              WB Welch-Costas).
	 *   tone_hop_step            : per-symbol tone-hop step (coprime with M).
	 *   M_tones                  : MFSK alphabet size (16 for WB ACK / CONNECT).
	 *   nStreams                 : number of MFSK streams (1 for WB).
	 *   stream_offsets           : array of stream-band starting subcarriers.
	 *
	 * Returns: residual CFO in Hz, or 0.0 on low confidence / invalid input.
	 *
	 * Reference: P. H. Moose, "A technique for orthogonal frequency division
	 * multiplexing frequency offset correction," IEEE TCOM 1994. Same as
	 * carrier_frequency_sync_wb_mfsk; only the modulation-known-pattern
	 * differs.
	 */

	if (pattern_nsymb <= 0 || M_tones <= 0 || nStreams <= 0) return 0.0;
	if (pattern_tones == NULL || stream_offsets == NULL) return 0.0;
	if (pattern_len <= 0) return 0.0;
	if (Nfft <= 0) return 0.0;
	int half = Nfft / 2;
	if (half <= 0) return 0.0;
	if (sym_start_offset_samples < 0) return 0.0;

	// fs_base (decimated/base sample rate the caller's buffer is at):
	// carrier_freq_width = fs_base / Nfft, so fs_base = carrier_freq_width * Nfft.
	double fs_base = carrier_freq_width * (double)Nfft;
	if (fs_base <= 0.0) return 0.0;

	int Nc_half = Nc / 2;
	int Nofdm = Nfft + Ngi;

	std::complex<double> C(0.0, 0.0);
	double energy_total = 0.0;

	for (int s = 0; s < pattern_nsymb; s++)
	{
		// Tone hopping: the actually transmitted tone at symbol s.
		// Mirrors the ACK pattern generator and detect_ack_pattern's
		// expected_tone computation.
		int tone_base = pattern_tones[s % pattern_len];
		if (tone_base < 0) continue;
		int actual_tone = (tone_base + s * tone_hop_step) % M_tones;
		if (actual_tone < 0 || actual_tone >= M_tones) continue;

		for (int st = 0; st < nStreams; st++)
		{
			int sub = stream_offsets[st] + actual_tone;
			int expected_bin = (sub < Nc_half)
				? (Nfft - Nc_half + sub)
				: (start_shift + (sub - Nc_half));
			// Map FFT bin to a centered/signed bin index in (-Nfft/2, Nfft/2].
			int k_centered = (expected_bin < Nfft / 2)
				? expected_bin
				: expected_bin - Nfft;
			double f_tone = (double)k_centered * fs_base / (double)Nfft;

			// De-rotation phase recurrence: exp(-j*2π*f_tone*t).
			double angle_step = -2.0 * M_PI * f_tone / fs_base;
			double sr = std::cos(angle_step);
			double si = std::sin(angle_step);

			std::complex<double> A(0.0, 0.0);
			std::complex<double> B(0.0, 0.0);
			double pr = 1.0;
			double pi = 0.0;

			int base_idx = sym_start_offset_samples + s * Nofdm + Ngi;

			for (int i = 0; i < half; i++)
			{
				double sample_r = in[base_idx + i].real();
				double sample_i = in[base_idx + i].imag();
				double dr = sample_r * pr - sample_i * pi;
				double di = sample_r * pi + sample_i * pr;
				A.real(A.real() + dr);
				A.imag(A.imag() + di);
				double npr = pr * sr - pi * si;
				double npi = pr * si + pi * sr;
				pr = npr;
				pi = npi;
			}
			for (int i = half; i < Nfft; i++)
			{
				double sample_r = in[base_idx + i].real();
				double sample_i = in[base_idx + i].imag();
				double dr = sample_r * pr - sample_i * pi;
				double di = sample_r * pi + sample_i * pr;
				B.real(B.real() + dr);
				B.imag(B.imag() + di);
				double npr = pr * sr - pi * si;
				double npi = pr * si + pi * sr;
				pr = npr;
				pi = npi;
			}

			std::complex<double> contrib = B * std::conj(A);
			C += contrib;
			energy_total += std::norm(A) + std::norm(B);
		}
	}

	if (!std::isfinite(C.real()) || !std::isfinite(C.imag())) return 0.0;
	if (!std::isfinite(energy_total) || energy_total <= 0.0) return 0.0;

	// Confidence gate: low |C|/energy ratio means the tones are not present
	// or the channel is noise-dominated. Mirror of carrier_frequency_sync_wb_mfsk
	// and carrier_frequency_sync_nb.
	double C_mag = std::abs(C);
	if (C_mag / energy_total < 0.05) return 0.0;

	// arg(C) = 2π * CFO * T_half_sym, T_half_sym = half / fs_base
	// → CFO = arg(C) * carrier_freq_width * Nfft / (2π * half)
	double phase = std::arg(C);
	double freq_offset = phase * carrier_freq_width * (double)Nfft
	                   / (2.0 * M_PI * (double)half);

	if (!std::isfinite(freq_offset)) return 0.0;
	return freq_offset;
}

void cl_ofdm::framer(std::complex <double>* in, std::complex <double>* out)
{
	int data_index=0;
	int pilot_index=0;
	for(int j=0;j<Nsymb;j++)
	{
		for(int k=0;k<Nc;k++)
		{
			if((ofdm_frame+j*this->Nc+k)->type==DATA)
			{
				out[j*Nc+k]=in[data_index];
				data_index++;
			}
			else if ((ofdm_frame+j*this->Nc+k)->type==PILOT)
			{
				out[j*Nc+k]=pilot_configurator.sequence[pilot_index];
				pilot_index++;
			}
		}
	}

}

void cl_ofdm::deframer(std::complex <double>* in, std::complex <double>* out)
{
	int data_index=0;

	for(int j=0;j<Nsymb;j++)
	{
		for(int k=0;k<Nc;k++)
		{
			if((ofdm_frame+j*this->Nc+k)->type==DATA)
			{
				out[data_index]=in[j*Nc+k];
				data_index++;
			}
		}
	}
}


void cl_ofdm::symbol_mod(std::complex <double>*in, std::complex <double>*out)
{
	zero_padder(in,zero_padded_data);
	ifft(zero_padded_data,iffted_data);
	gi_adder(iffted_data, out);
}

void cl_ofdm::symbol_demod(std::complex <double>*in, std::complex <double>*out)
{
	gi_remover(in, gi_removed_data);
	fft(gi_removed_data,ffted_data);
	zero_depadder(ffted_data, out);
}

cl_pilot_configurator::cl_pilot_configurator()
{
	first_col=DATA;
	last_col=AUTO_SELLECT;
	second_col=DATA;
	first_row=DATA;
	last_row=DATA;
	Nc=0;
	Nsymb=0;
	Nc_max=0;
	nData=0;
	nPilots=0;
	nConfig=0;
	carrier=0;
	Dy=0;
	Dx=0;
	virtual_carrier=0;
	modulation=DBPSK;
	sequence=0;
	boost=1.0;
	Nfft=0;
	start_shift=0;
	seed=0;
	print_on=NO;
	pilot_density=HIGH_DENSITY;
}

cl_pilot_configurator::~cl_pilot_configurator()
{
	CDELETE(virtual_carrier);
}

void cl_pilot_configurator::init(int Nfft, int Nc, int Nsymb,struct st_carrier* _carrier, int start_shift)
{
	this->carrier=_carrier;
	this->Nc=Nc;
	this->Nsymb=Nsymb;
	this->Nfft=Nfft;
	this->start_shift=start_shift;
	if(Nc>Nsymb)
	{
		this->Nc_max=Nc;
	}
	else
	{
		this->Nc_max=Nsymb;
	}
	nData=Nc*Nsymb;
	virtual_carrier = CNEW(struct st_carrier, this->Nc_max*this->Nc_max, "pilot.virtual_carrier");

	for(int j=0;j<this->Nc_max;j++)
	{
		for(int i=0;i<this->Nc_max;i++)
		{
			(virtual_carrier+j*this->Nc_max+i)->type=DATA;
		}

	}

	this->configure();

	sequence = CNEW(std::complex<double>, nPilots, "pilot.sequence");

	if(print_on==YES)
	{
		this->print();
	}

	__srandom(seed);
	int last_pilot=0;
	int pilot_value;
	if(this->modulation==DBPSK)
	{
		for(int i=0;i<nPilots;i++)
		{
			pilot_value=__random()%2 ^ last_pilot;
			sequence[i]=std::complex <double>(2*pilot_value-1,0)*boost;
			last_pilot=pilot_value;
		}
	}
}

void cl_pilot_configurator::deinit()
{
	this->carrier=NULL;
	this->Nc=0;
	this->Nsymb=0;
	this->Nfft=0;
	this->Nc_max=0;
	this->nData=0;

	CDELETE(virtual_carrier);
	CDELETE(sequence);

}

void cl_pilot_configurator::configure()
{
	int x=0;
	int y=0;

	while(x<Nc_max && y<Nc_max)
	{
		(virtual_carrier+y*Nc_max+x)->type=PILOT;

		for(int j=y;j<Nc_max;j+=Dy)
		{
			(virtual_carrier+j*Nc_max+x)->type=PILOT;
		}
		for(int j=y;j>=0;j-=Dy)
		{
			(virtual_carrier+j*Nc_max+x)->type=PILOT;
		}

		y++;
		x+=Dx;

	}

	int pilot_count=0;
	for(int j=0;j<Nsymb;j++)
	{
		if ((virtual_carrier+j*Nc_max+Nc-1)->type==PILOT)
		{
			pilot_count++;
		}
	}

	if(last_col==AUTO_SELLECT && pilot_count<2)
	{
		last_col=COPY_FIRST_COL;
	}


	for(int j=0;j<Nc_max;j++)
	{
		if(first_row==PILOT)
		{
			(virtual_carrier+0*Nc_max+j)->type=PILOT;
		}
		if(last_row==PILOT)
		{
			(virtual_carrier+(Nsymb-1)*Nc_max+j)->type=PILOT;
		}
		if(first_col==PILOT)
		{
			(virtual_carrier+j*Nc_max+0)->type=PILOT;
		}
		if(last_col==PILOT)
		{
			(virtual_carrier+j*Nc_max+Nc-1)->type=PILOT;
		}
		if(last_col==COPY_FIRST_COL)
		{
			(virtual_carrier+j*Nc_max+Nc-1)->type=(virtual_carrier+j*Nc_max+0)->type;
		}
		if(second_col==CONFIG&&(virtual_carrier+j*Nc_max+1)->type!=PILOT)
		{
			(virtual_carrier+j*Nc_max+1)->type=CONFIG;
		}
	}


	nPilots=0;
	nConfig=0;
	for(int j=0;j<Nsymb;j++)
	{
		for(int i=0;i<Nc;i++)
		{

			(carrier + j*Nc+i)->type=(virtual_carrier+j*Nc_max+i)->type;

			if((virtual_carrier+j*Nc_max+i)->type==PILOT)
			{
				nPilots++;
				nData--;
			}
			if((virtual_carrier+j*Nc_max+i)->type==CONFIG)
			{
				nConfig++;
				nData--;
			}
		}
	}
}

void cl_pilot_configurator::print()
{
	for(int j=0;j<Nsymb;j++)
	{
		for(int i=0;i<Nc;i++)
		{
			if((carrier+j*Nc+i)->type==PILOT)
			{
				std::cout<<"P ";
			}
			else if((carrier+j*Nc+i)->type==CONFIG)
			{
				std::cout<<"C ";
			}
			else if((carrier+j*Nc+i)->type==ZERO)
			{
				std::cout<<"Z ";
			}
			else if((carrier+j*Nc+i)->type==PREAMBLE)
			{
				std::cout<<"R ";
			}
			else if((carrier+j*Nc+i)->type==DATA)
			{
				std::cout<<". ";
			}
			else
			{
				std::cout<<"_ ";
			}
		}
		std::cout<<std::endl;

	}

	std::cout<<"nData="<<this->nData<<std::endl;
	std::cout<<"nPilots="<<this->nPilots<<std::endl;
	std::cout<<"nConfig="<<this->nConfig<<std::endl;
}


cl_preamble_configurator::cl_preamble_configurator()
{
	Nc=0;
	Nsymb=0;
	nPreamble=0;
	carrier=0;
	modulation=MOD_BPSK;
	nIdentical_sections=0;
	sequence=0;
	boost=1.0;
	Nfft=0;
	nZeros=0;
	start_shift=0;
	seed=0;
	print_on=NO;
}

cl_preamble_configurator::~cl_preamble_configurator()
{
}

void cl_preamble_configurator::init(int Nfft, int Nc, struct st_carrier* _carrier, int start_shift)
{
	this->carrier=_carrier;
	this->Nc=Nc;
	this->Nfft=Nfft;
	this->start_shift=start_shift;

	this->configure();

	sequence = CNEW(std::complex<double>, this->Nsymb*this->Nc, "preamble.sequence");

	if(print_on==YES)
	{
		this->print();
	}

	// Zadoff-Chu preamble values. ZC sequences have zero periodic
	// auto-correlation sidelobes → sharper matched-filter peak, better
	// discrimination at low SNR. Same sequence repeated each symbol.
	int bins_per_sym = nPreamble / this->Nsymb;
	int zc_root = 7;  // coprime with bins_per_sym for WB (25) and NB (4-5)
	int seq_idx = 0;
	for(int j=0;j<this->Nsymb;j++)
	{
		int sym_preamble_idx = 0;
		for(int i=0;i<this->Nc;i++)
		{
			if ((carrier+j*this->Nc+i)->type==ZERO)
			{
				(carrier+j*this->Nc+i)->value=0;
			}
			else if ((carrier+j*this->Nc+i)->type==PREAMBLE)
			{
				double phase = -M_PI * zc_root * sym_preamble_idx * (sym_preamble_idx + 1.0) / bins_per_sym;
				std::complex<double> zc_val(cos(phase), sin(phase));
				(carrier+j*this->Nc+i)->value = zc_val;
				sequence[seq_idx] = zc_val;
				sym_preamble_idx++;
				seq_idx++;
			}
		}
	}

}

void cl_preamble_configurator::deinit()
{
	this->carrier=NULL;
	this->Nc=0;
	this->Nsymb=0;
	this->Nfft=0;

	CDELETE(sequence);

}

void cl_preamble_configurator::configure()
{
	// Reset counters — they are accumulated in the loop below.
	// Without this, deinit→init cycles cause nPreamble/nZeros to grow
	// across configurations, corrupting bins_per_sym in the ZC formula.
	nPreamble = 0;
	nZeros = 0;

	int fft_zeros_tmp[Nfft];
	int fft_zeros_depadded_tmp[Nc];

	// Subcarrier spacing pattern determines time-domain repetition period:
	//   Every-Kth bin → period = Nfft/K → K identical sections per symbol.
	//
	// WB (Nc>=50): every-4th → 4× repetition (period Nfft/4 = 64 samples).
	//   ~12 preamble bins per symbol. Doubles Moose capture range (±2 subcarrier
	//   spacings vs ±1) and enables auto-correlator pre-filter (future).
	// NB (Nc<=10): every-2nd → 2× repetition (period Nfft/2 = 128 samples).
	//   4 preamble bins per symbol — can't go sparser.
	int subcarrier_step;
	if(Nc >= 50)
	{
		subcarrier_step = 4;
		nIdentical_sections = 4;
	}
	else
	{
		subcarrier_step = 2;
		nIdentical_sections = 2;
	}

	for(int j=0;j<Nfft;j++)
	{
		fft_zeros_tmp[j] = (j % subcarrier_step == 0) ? 1 : 0;
	}

	for(int j=0;j<Nc/2;j++)
	{
		fft_zeros_depadded_tmp[j]=fft_zeros_tmp[j+Nfft-Nc/2];
	}
	for(int j=Nc/2;j<Nc;j++)
	{
		fft_zeros_depadded_tmp[j]=fft_zeros_tmp[j-Nc/2+start_shift];
	}

	for(int i=0;i<this->Nsymb;i++)
	{
		for(int j=0;j<Nc;j++)
		{
			if(fft_zeros_depadded_tmp[j]==0)
			{
				(carrier+i*Nc+j)->type=ZERO;
				nZeros++;
			}
			else
			{
				(carrier+i*Nc+j)->type=PREAMBLE;
				nPreamble++;
			}
		}
	}

	// Note: nZeros and nPreamble are already accumulated correctly in the nested loop above.
	// The *= Nsymb lines were removed as they caused double-counting (values were Nsymb^2 too large).

}

void cl_preamble_configurator::print()
{
    std::cout<<"nZeros="<<this->nZeros<<std::endl;
    std::cout<<"nPreamble="<<this->nPreamble<<std::endl;

	for(int j=0;j<Nsymb;j++)
	{
		for(int i=0;i<Nc;i++)
		{
			if((carrier+j*Nc+i)->type==ZERO)
			{
				std::cout<<"Z ";
			}
			else if((carrier+j*Nc+i)->type==PREAMBLE)
			{
				std::cout<<"R ";
			}
			else
			{
				std::cout<<"_ ";
			}
		}
		std::cout<<std::endl;
	}
}

// A.1.4: cross-pilot differential noise variance estimator.
//
// Replaces the pilot-residual estimator that previously lived inline in
// ZF_channel_estimator and LS_channel_estimator.
//
// Why the residual estimator was wrong:
//   ZF estimator: estimated_channel[pilot].value = Y/X exactly, so
//     residual = Y - (Y/X)*X = 0 identically (post-E1 commit 38f5c60).
//     noise_variance_estimate collapsed to the 1e-6 floor and LLRs went
//     ~1000x over-confident; only var_floor=0.001 in psk.cc papered over it.
//   LS estimator: window-averaged H pulls toward the centre pilot, biasing
//     residuals low by (N-1)/N. Smaller magnitude but still wrong.
//
// Why cross-pilot differential is right:
//   For two adjacent in-column pilots at rows i_a, i_b = i_a + Dy:
//     H_a_raw = Y_a / X_a = H_true_a + noise_a / X_a
//     H_b_raw = Y_b / X_b = H_true_b + noise_b / X_b
//   For Dy-separated pilots in the same column, H_true_a ≈ H_true_b (slow
//   channel variation is the design assumption of pilot-based estimation;
//   if it weren't true, the linear interpolation between them would already
//   be broken). So:
//     |H_a_raw - H_b_raw|^2 ≈ |noise_a/X_a - noise_b/X_b|^2
//   E[|·|^2] = Var(noise_a)/|X_a|^2 + Var(noise_b)/|X_b|^2 = 2σ²/|X|^2
//   (pilots have uniform amplitude). Divide by 2 to get an unbiased estimate
//   of σ²/|X|^2 — same units the consumers (MMSE equalizer, psk LLR scale)
//   already expect from the old residual estimator.
//
// The walk order is row-major (matches existing ZF/LS/CPE_correction loops)
// to keep pilot_index aligned with pilot_configurator.sequence indexing.
// Per-column state tracks the last pilot row and its raw H estimate; we
// pair only when the row delta equals Dy exactly (mirrors CPE_correction's
// guard at ofdm.cc:1384).
//
// Returns 0.01 (the previous default) if fewer than one valid pair was
// found — handles Dy<=0 and degenerate frames consistently with the old
// path's "noise_count==0" branch.
//
// Reference: Ozdemir & Arslan, "Channel Estimation for Wireless OFDM
// Systems," IEEE Comm Surveys 2007, §IV-B (cross-pilot differential
// noise estimation). Indexing pattern borrowed from CPE_correction
// at ofdm.cc:1349.
double cl_ofdm::estimate_noise_from_pilot_pairs(std::complex<double>* in)
{
	if (Nsymb <= 0 || Nc <= 0) return 0.01;
	int Dy = pilot_configurator.Dy;
	if (Dy <= 0) return 0.01;

	// Per-column state: last pilot row and raw H = Y/X for that pilot.
	int prev_row[Nc];                       // VLA, Nc <= 50
	std::complex<double> prev_H[Nc];        // VLA
	for (int j = 0; j < Nc; j++) prev_row[j] = -1;

	double noise_sum = 0.0;
	int noise_count = 0;

	int pilot_index = 0;
	for (int i = 0; i < Nsymb; i++)
	{
		for (int j = 0; j < Nc; j++)
		{
			if ((ofdm_frame + i*Nc + j)->type == PILOT)
			{
				std::complex<double> X = pilot_configurator.sequence[pilot_index];
				std::complex<double> H_raw = *(in + i*Nc + j) / X;

				if (prev_row[j] >= 0 && (i - prev_row[j]) == Dy)
				{
					std::complex<double> delta = H_raw - prev_H[j];
					double mag2 = delta.real()*delta.real() + delta.imag()*delta.imag();
					noise_sum += mag2 * 0.5;   // /2 accounts for noise on both pilots
					noise_count++;
				}

				prev_row[j] = i;
				prev_H[j] = H_raw;
				pilot_index++;
			}
		}
	}

	if (noise_count <= 0) return 0.01;
	double nv = noise_sum / noise_count;
	if (nv < 1e-6) nv = 1e-6;   // prevent division instability at very high SNR
	return nv;
}

void cl_ofdm::ZF_channel_estimator(std::complex <double>*in)
{
	int pilot_index=0;
	for(int i=0;i<Nsymb;i++)
	{
		for(int j=0;j<Nc;j++)
		{
			if((ofdm_frame+i*Nc+j)->type==PILOT)
			{
				(estimated_channel+i*Nc+j)->status=MEASURED;
				(estimated_channel+i*Nc+j)->value=*(in+i*Nc+j)/pilot_configurator.sequence[pilot_index];
				pilot_index++;
			}
			else
			{
				(estimated_channel+i*Nc+j)->status=UNKNOWN;
				(estimated_channel+i*Nc+j)->value=0;
			}
		}
	}

	if(Nc <= 10)
	{
		// NB (Nc=10): Column-wise interpolation (same as WB for Dx=1).
		// Row-wise approaches fail because VB-Cable audio path has genuine
		// frequency-selective response across 469 Hz, and complex H
		// interpolation/averaging causes phase cancellation.
		for(int j=0;j<Nc;j++)
		{
			if(j%this->pilot_configurator.Dx==0)
			{
				interpolate_linear_col(estimated_channel,Nc,Nsymb,j);
			}
			else if(j==Nc-1)
			{
				interpolate_linear_col(estimated_channel,Nc,Nsymb,j);
			}
		}
	}
	else
	{
		// WB (Nc=50): Column-wise + bilinear matrix (original approach).
		for(int j=0;j<Nc;j++)
		{
			if(j%this->pilot_configurator.Dx==0)
			{
				interpolate_linear_col(estimated_channel,Nc,Nsymb,j);
			}
			else if(j==Nc-1)
			{
				interpolate_linear_col(estimated_channel,Nc,Nsymb,j);
			}
		}

		for(int j=0;j<Nc;j+=this->pilot_configurator.Dx)
		{
			if(j+this->pilot_configurator.Dx<Nc)
			{
				interpolate_bilinear_matrix(estimated_channel,Nc,Nsymb,j,j+this->pilot_configurator.Dx,0,Nsymb-1);
			}
			else if(j!=Nc-1)
			{
				interpolate_bilinear_matrix(estimated_channel,Nc,Nsymb,j,Nc-1,0,Nsymb-1);
			}
		}
	}

	// Estimate noise variance for MMSE equalization.
	//
	// A.1.4 (this session): replaced pilot-residual estimator with cross-pilot
	// differential. For ZF, residual = Y - (Y/X)*X = 0 identically, so the
	// residual estimator collapsed to the 1e-6 floor and LLRs went ~1000x
	// over-confident (only var_floor=0.001 in psk.cc papered over it).
	// See estimate_noise_from_pilot_pairs() above for the new method and
	// rationale.
	//
	// Original residual-based code preserved below under #if 0 for git-blame
	// trail (DO NOT delete without updating fact-documents/data-flow- doc).
	noise_variance_estimate = estimate_noise_from_pilot_pairs(in);
#if 0
	// LEGACY (A.1.4): pilot-residual σ² estimate. Broken for ZF estimator
	// because estimated_channel[pilot].value = Y/X exactly, making the
	// residual identically zero. Kept here for blame/historical context.
	// (Standard practice citation; see also van de Beek/Edfors et al., VTC
	// 1995, "On Channel Estimation in OFDM Systems", §III for context on
	// LS-then-DFT smoothing — NOT the residual estimator itself.)
	{
		double noise_sum = 0.0;
		int noise_count = 0;
		int pi = 0;
		for(int i = 0; i < Nsymb; i++)
		{
			for(int j = 0; j < Nc; j++)
			{
				if((ofdm_frame + i*Nc + j)->type == PILOT)
				{
					std::complex<double> reconstructed = (estimated_channel + i*Nc + j)->value * pilot_configurator.sequence[pi];
					std::complex<double> residual = *(in + i*Nc + j) - reconstructed;
					noise_sum += residual.real()*residual.real() + residual.imag()*residual.imag();
					noise_count++;
					pi++;
				}
			}
		}
		if(noise_count > 0)
		{
			noise_variance_estimate = noise_sum / noise_count;
		}
		else
		{
			noise_variance_estimate = 0.01;
		}
		// Floor to prevent division instability at very high SNR
		if(noise_variance_estimate < 1e-6)
			noise_variance_estimate = 1e-6;
	}
#endif

	// DFT-based channel estimate smoothing: suppress estimation noise
	// by windowing the time-domain impulse response. Applied AFTER noise
	// variance estimation so noise_variance_estimate reflects honest pilot
	// noise (smoother absorbs noise into H; downstream consumers use the
	// smoothed H but expect noise_variance_estimate to track raw pilot SNR).
	smooth_channel_estimate_dft();
/*
 * Ref: R. Lucky, “The adaptive equalizer,” IEEE Signal Processing Magazine, vol. 23, no. 3, pp. 104–107, 2006.
 */
}

void cl_ofdm::LS_channel_estimator(std::complex <double>*in)
{
	std::complex <double> pilot_data[Nsymb*Nc]={std::complex <double> (0,0)};

	int pilot_index=0;
	for(int i=0;i<Nsymb;i++)
	{
		for(int j=0;j<Nc;j++)
		{
			if((ofdm_frame+i*Nc+j)->type==PILOT)
			{
				*(pilot_data+i*Nc+j)=pilot_configurator.sequence[pilot_index];
				pilot_index++;
			}
			else
			{
				(estimated_channel+i*Nc+j)->status=UNKNOWN;
				(estimated_channel+i*Nc+j)->value=0;
			}

		}
	}

	int nPilots=0;
	std::complex <double> ch_tmp;

	for(int j=0;j<Nc;j++)
	{
		for(int i=0;i<Nsymb;i++)
		{
			if((ofdm_frame+i*Nc+j)->type!=PILOT)
			{
				continue;
			}

			int window_vertical_start=i-LS_window_hight/2;
			int window_vertical_end=i+LS_window_hight/2;

			int window_horizontal_start=j-LS_window_width/2;
			int window_horizontal_end=j+LS_window_width/2;

			nPilots=0;
			for(int k=window_vertical_start;k<=window_vertical_end;k++)
			{
				if(k<0 || k>=Nsymb)
				{
					continue;
				}
				for(int l=window_horizontal_start;l<=window_horizontal_end;l++)
				{
					if(l<0 || l>=Nc)
					{
						continue;
					}

					if((ofdm_frame+k*Nc+l)->type==PILOT)
					{
						nPilots++;
					}
				}
			}

			std::complex <double> x[nPilots], y[nPilots];

			int x_y_index=0;
			for(int k=window_vertical_start;k<=window_vertical_end;k++)
			{
				if(k<0 || k>=Nsymb)
				{
					continue;
				}
				for(int l=window_horizontal_start;l<=window_horizontal_end;l++)
				{

					if(l<0 || l>=Nc)
					{
						continue;
					}
					if((ofdm_frame+k*Nc+l)->type==PILOT)
					{
						x[x_y_index]=*(pilot_data+k*Nc+l);
						y[x_y_index]=*(in+k*Nc+l);
						x_y_index++;
					}
				}
			}
			//ch_tmp=(x.transpose *x).inverse * x.transpose * y
			matrix_multiplication(x,nPilots,1,x,1,nPilots,&ch_tmp);
			ch_tmp=1.0/ch_tmp;
			for(int m=0;m<nPilots;m++)
			{
				x[m]*=ch_tmp;
			}
			matrix_multiplication(x,nPilots,1,y,1,nPilots,&ch_tmp);

			(estimated_channel+i*Nc+j)->status=MEASURED;
			(estimated_channel+i*Nc+j)->value=ch_tmp;
		}
	}


	for(int j=0;j<Nc;j++)
	{
		if(j%this->pilot_configurator.Dx==0)
		{
			interpolate_linear_col(estimated_channel,Nc,Nsymb,j);
		}
		else if(j==Nc-1)
		{
			interpolate_linear_col(estimated_channel,Nc,Nsymb,j);
		}
	}

	for(int j=0;j<Nc;j+=this->pilot_configurator.Dx)
	{
		if(j+this->pilot_configurator.Dx<Nc)
		{
			interpolate_bilinear_matrix(estimated_channel,Nc,Nsymb,j,j+this->pilot_configurator.Dx,0,Nsymb-1);
		}
		else if(j!=Nc-1)
		{
			interpolate_bilinear_matrix(estimated_channel,Nc,Nsymb,j,Nc-1,0,Nsymb-1);
		}
	}

	// DFT-based channel estimate smoothing (same as ZF estimator). For the LS
	// path this runs BEFORE the noise-variance estimate (restored pre-E1 order,
	// reverting commit 38f5c60 for THIS estimator only) so the residual is
	// measured against the SAME final smoothed+interpolated H that the data
	// carriers are equalized with. See fix/cfg16-nv-restore rationale below.
	smooth_channel_estimate_dft();

	// CFG16 32-QAM clean-channel regression fix (fix/cfg16-nv-restore,
	// fact-documents/data-flow-noise_variance_estimate.md).
	//
	// Restore the PRE-E1/PRE-A.1.4 LS pilot-residual noise estimator for the LS
	// path ONLY. A.1.4 (commit 9c3fc40) replaced this with the cross-pilot
	// differential helper estimate_noise_from_pilot_pairs(), which is correct
	// for the ZF estimator (where the residual Y-(Y/X)*X is identically zero)
	// but WRONG for LS:
	//   - cross-pilot measures only the PRE-equalization thermal floor
	//     σ²/|X|² between two raw same-column pilots (≈1.7e-4 on a clean/
	//     slowly-varying channel);
	//   - the Euclidean QAM demapper (psk.cc:325, LLR=ΔD/variance) operates on
	//     the EQUALIZED+SMOOTHED constellation, whose effective per-symbol noise
	//     is the residual EVM of the FINAL interpolated/smoothed channel
	//     estimate against the pilots — a LARGER quantity (~0.035) that does NOT
	//     vanish at high SNR.
	// Feeding 1.7e-4 to psk.demod scaled LLRs ~200× over-confident → 32-QAM
	// inner-point bit-sign flips → BP hit the iter cap (101) → CRC fail → 0 bps.
	// CFG15 16-QAM (larger min-distance) tolerated the wrong magnitude.
	//
	// Unlike LS, the ZF estimator's pre-E1 residual was identically zero, so ZF
	// (all NB configs + the unused WB-ROBUST mapping) KEEPS A.1.4's cross-pilot
	// estimator above — A.1.4's NB recovery is provably untouched (this code is
	// LS-only; MFSK never reaches any channel estimator — telecom_system.cc:2304
	// branches to mfsk.demod() with its own guard-bin noise, mfsk.cc:999).
	//
	// This is the literal restoration of the estimate that historically decoded
	// CFG16 (pre-A.1.4 4dd8ffb: nv≈0.035, iter≈2, 3182 bps) — a measured pilot
	// residual, NOT a tuned constant and NOT measure_variance() (the post-EQ
	// |Y/H−X|² mvar≈0.18 reads anomalously high on this bench and breaks CFG15;
	// see the abandoned fix/cfg16-noisevar-floor / commit 2d540d9 dead end).
	//
	// Ref J. -J. van de Beek, O. Edfors, M. Sandell, S. K. Wilson and
	// P. O. Borjesson, "On channel estimation in OFDM systems," IEEE VTC 1995,
	// §III (LS estimate + DFT smoothing; residual σ² from pilot reconstruction).
	{
		double noise_sum = 0.0;
		int noise_count = 0;
		int pi = 0;
		for(int i = 0; i < Nsymb; i++)
		{
			for(int j = 0; j < Nc; j++)
			{
				if((ofdm_frame + i*Nc + j)->type == PILOT)
				{
					std::complex<double> reconstructed = (estimated_channel + i*Nc + j)->value * pilot_configurator.sequence[pi];
					std::complex<double> residual = *(in + i*Nc + j) - reconstructed;
					noise_sum += residual.real()*residual.real() + residual.imag()*residual.imag();
					noise_count++;
					pi++;
				}
			}
		}
		if(noise_count > 0)
		{
			noise_variance_estimate = noise_sum / noise_count;
		}
		else
		{
			noise_variance_estimate = 0.01;
		}
		// Floor to prevent division instability at very high SNR.
		if(noise_variance_estimate < 1e-6)
			noise_variance_estimate = 1e-6;
	}

	// fix/cfg16-nv-restore A/B toggle (validation only, default false). When set,
	// the LS path reverts to A.1.4's cross-pilot estimator — i.e. the pre-fix
	// (monitor) behavior — so a SINGLE binary can run both arms under controlled
	// conditions. estimate_noise_from_pilot_pairs(in) reads only raw received
	// pilots (not estimated_channel), so it is invariant to the smoother order
	// above; this toggle isolates the exact quantity that changed. Production
	// path keeps the restored residual (this flag default false).
	if(ls_use_crosspilot_nv)
		noise_variance_estimate = estimate_noise_from_pilot_pairs(in);

	// [LS-NV-DBG] Validation instrumentation (fix/cfg16-nv-restore): log the
	// restored pilot-residual nv alongside what the A.1.4 cross-pilot estimator
	// would have produced on the SAME frame, to confirm the ~200× collapse and
	// its restoration on a frequency-selective channel. Cheap (one extra walk);
	// gated so it can be left in or trivially removed. Remove before merge if
	// log volume is a concern.
	if(ls_nv_debug_enabled)
	{
		double crosspilot_nv = estimate_noise_from_pilot_pairs(in);
		printf("[LS-NV-DBG] residual_nv=%.6e crosspilot_nv=%.6e Nc=%d Nsymb=%d ratio=%.1f\n",
			noise_variance_estimate, crosspilot_nv, Nc, Nsymb,
			(crosspilot_nv > 0 ? noise_variance_estimate / crosspilot_nv : -1.0));
		fflush(stdout);
	}
/*
 * Ref J. . -J. van de Beek, O. Edfors, M. Sandell, S. K. Wilson and P. O. Borjesson, "On channel estimation in OFDM systems," 1995 IEEE 45th Vehicular Technology Conference. Countdown to the Wireless Twenty-First Century, Chicago, IL, USA, 1995, pp. 815-819 vol.2, doi: 10.1109/VETEC.1995.504981.
 */
}

void cl_ofdm::CPE_correction(std::complex<double>* in)
{
	if (Nsymb <= 0 || Nc <= 0) return;

	int Dy = pilot_configurator.Dy;
	if (Dy <= 0) return;

	// Estimate residual frequency offset from pilot phase rotation.
	// For each subcarrier column, adjacent pilot rows (separated by Dy
	// symbols) give a phase-change measurement. Averaging all such pairs
	// across the frame gives a robust estimate of the per-symbol phase
	// rate, which is then removed from the received data BEFORE channel
	// estimation. This prevents LS window phase cancellation.
	//
	// With NB (Nc=10, Dy=3): ~320 pilot pairs → very robust even at low SNR.
	// With WB (Nc=50, Dy=3): ~800+ pairs → marginal extra improvement.

	// Per-column state: last pilot row and raw H value
	int prev_row[Nc];                    // VLA, Nc <= 50
	std::complex<double> prev_H[Nc];     // VLA
	for (int j = 0; j < Nc; j++) prev_row[j] = -1;

	std::complex<double> dH_sum(0, 0);
	int dH_count = 0;

	int pilot_index = 0;
	for (int i = 0; i < Nsymb; i++)
	{
		for (int j = 0; j < Nc; j++)
		{
			if ((ofdm_frame + i * Nc + j)->type == PILOT)
			{
				std::complex<double> X = pilot_configurator.sequence[pilot_index];
				std::complex<double> H_raw = *(in + i * Nc + j) / X;

				if (prev_row[j] >= 0 && (i - prev_row[j]) == Dy)
				{
					dH_sum += H_raw * std::conj(prev_H[j]);
					dH_count++;
				}

				prev_row[j] = i;
				prev_H[j] = H_raw;
				pilot_index++;
			}
		}
	}

	if (dH_count < 2) {
		return;
	}

	double phase_per_Dy = std::arg(dH_sum);
	double phase_rate = phase_per_Dy / Dy;    // radians per symbol

	// Skip correction if negligible (< 0.1 degree/symbol)
	if (std::abs(phase_rate) < 0.00175) {
		return;
	}

	// Remove linear phase rotation from all symbols (symbol 0 = reference)
	for (int i = 0; i < Nsymb; i++)
	{
		std::complex<double> correction = std::exp(std::complex<double>(0, -phase_rate * i));
		for (int j = 0; j < Nc; j++)
		{
			*(in + i * Nc + j) *= correction;
		}
	}
}

void cl_ofdm::restore_channel_amplitude()
{
	for(int i=0;i<Nsymb;i++)
	{
		for(int j=0;j<Nc;j++)
		{
			*(estimated_channel_without_amplitude_restoration+i*Nc+j)=*(estimated_channel+i*Nc+j);
			(estimated_channel+i*Nc+j)->value=set_complex(1, get_angle((estimated_channel+i*Nc+j)->value));
		}
	}
/*
 * Ref: F. Jerji and C. Akamine, "Enhanced ZF and LS channel estimators for OFDM with MPSK modulation," 2024 IEEE International Symposium on Broadband Multimedia Systems and Broadcasting (BMSB).
 */
}
void cl_ofdm::automatic_gain_control(std::complex <double>*in)
{
	int pilot_index=0;
	double pilot_amp=0;
	double agc=0;
	for(int i=0;i<Nsymb;i++)
	{
		for(int j=0;j<Nc;j++)
		{
			if((ofdm_frame+i*Nc+j)->type==PILOT)
			{
				pilot_amp+=get_amplitude(*(in+i*Nc+j));
				pilot_index++;
			}
		}
	}
	pilot_amp/=pilot_index;
	agc=pilot_configurator.boost/pilot_amp;

	for(int i=0;i<Nsymb;i++)
	{
		for(int j=0;j<Nc;j++)
		{
			*(in+i*Nc+j)*=agc;
		}

	}
}

double cl_ofdm::measure_variance(std::complex <double>*in)
{
	double variance=0;
	int pilot_index=0;
	std::complex <double> diff;
	for(int i=0;i<Nsymb;i++)
	{
		for(int j=0;j<Nc;j++)
		{
			if((ofdm_frame+i*Nc+j)->type==PILOT)
			{
				diff=*(in+i*Nc+j) -pilot_configurator.sequence[pilot_index];
				pilot_index++;
				variance+=pow(diff.real(),2)+pow(diff.imag(),2);
			}
		}

	}
	variance/=(double)pilot_index;

	return variance;
}

double cl_ofdm::measure_signal_stregth(std::complex <double>*in, int nItems)
{
	double signal_stregth=0;
	double signal_stregth_dbm=0;
	std::complex <double> value;

	for(int i=0;i<nItems;i++)
	{
		value=*(in+i);
		signal_stregth+=pow(value.real(),2)+pow(value.imag(),2);
	}
	signal_stregth/=nItems;

	signal_stregth_dbm=10.0*log10((signal_stregth)/0.001);

	return signal_stregth_dbm;
}

st_power_measurment cl_ofdm::measure_signal_power_avg_papr(double*in, int nItems)
{
	st_power_measurment power_measurment;
	power_measurment.avg=0;
	power_measurment.max=0;
	power_measurment.papr_db=0;
	double power_tmp;

	for(int i=0;i<nItems;i++)
	{
		power_tmp=pow(*(in+i),2);
		power_measurment.avg+=power_tmp;
		if(power_tmp>power_measurment.max)
		{
			power_measurment.max=power_tmp;
		}
	}
	power_measurment.avg/=nItems;

	power_measurment.papr_db=10.0*log10(power_measurment.max/power_measurment.avg);

	return power_measurment;
}

void cl_ofdm::peak_clip(double *in, int nItems, double papr)
{
	double power_measurment_avg=0;
	double power_tmp=0;
	double peak_allowed=0;
	for(int i=0;i<nItems;i++)
	{
		power_tmp=pow(*(in+i),2);
		power_measurment_avg+=power_tmp;
	}
	power_measurment_avg/=nItems;

	peak_allowed=sqrt(power_measurment_avg*pow(10,papr/10.0));

	for(int i=0;i<nItems;i++)
	{
		if(*(in+i)>0 && *(in+i)>peak_allowed)
		{
			*(in+i)=peak_allowed;
		}

		if(*(in+i)<0 && *(in+i)< -peak_allowed)
		{
			*(in+i)=-peak_allowed;
		}
	}

}

void cl_ofdm::peak_clip(std::complex <double> *in, int nItems, double papr)
{
	double power_measurment_avg=0;
	double power_tmp=0;
	double peak_allowed=0;
	std::complex <double> value;
	for(int i=0;i<nItems;i++)
	{
		value=*(in+i);
		power_tmp=pow(value.real(),2)+pow(value.imag(),2);
		power_measurment_avg+=power_tmp;
	}
	power_measurment_avg/=nItems;
	peak_allowed=power_measurment_avg*pow(10,papr/10.0);

	for(int i=0;i<nItems;i++)
	{
		value=*(in+i);
		power_tmp=pow(value.real(),2)+pow(value.imag(),2);

		if(power_tmp>peak_allowed)
		{
			*(in+i)= set_complex(sqrt(peak_allowed), get_angle(*(in+i)));
		}
	}

}

double cl_ofdm::measure_SNR(std::complex <double>*in_s, std::complex <double>*in_n, int nItems)
{
	double variance=0;
	double SNR=0;
	std::complex <double> diff;
	for(int i=0;i<nItems;i++)
	{
		diff=*(in_n+i)-*(in_s+i);
		variance+=pow(diff.real(),2)+pow(diff.imag(),2);
	}
	variance/=nItems;
	SNR=-10.0*log10(variance);
	return SNR;
}

void cl_ofdm::smooth_channel_estimate_dft()
{
	// DFT-based channel estimation noise suppression.
	// Per-symbol: IFFT to time domain, window to keep GI-proportional taps,
	// FFT back to frequency domain. Suppresses estimation noise while
	// preserving real channel structure within the guard interval.
	// Ref: Edfors et al., "On Channel Estimation in OFDM Systems," VTC 1995.
	if(Nc < 4) return;  // Too few subcarriers for meaningful smoothing

	// Window width: number of time-domain taps to keep on each side of DC.
	// gi = Ngi/Nfft. Channel delay spread fits within GI, so gi*Nc taps suffice.
	// Add margin of +2 for timing uncertainty and filter leakage.
	int window_taps = (int)(gi * Nc + 0.5) + 2;
	if(window_taps < 3) window_taps = 3;
	if(window_taps >= Nc / 2) return;  // Window too wide, smoothing won't help

	std::complex<double>* buf_in = new std::complex<double>[Nc];
	std::complex<double>* buf_out = new std::complex<double>[Nc];

	for(int i = 0; i < Nsymb; i++)
	{
		// Extract H[0..Nc-1] for this OFDM symbol
		for(int j = 0; j < Nc; j++)
			buf_in[j] = (estimated_channel + i*Nc + j)->value;

		// IFFT: frequency domain → time-domain impulse response
		// PocketFFT handles arbitrary sizes (Nc=50 = 2×5²)
		ifft(buf_in, buf_out, Nc);

		// Window: keep first window_taps (causal delay) and last window_taps
		// (acausal / timing misalignment), zero the rest (noise)
		for(int t = window_taps; t < Nc - window_taps; t++)
			buf_out[t] = std::complex<double>(0.0, 0.0);

		// FFT: smoothed time domain → smoothed frequency domain
		fft(buf_out, buf_in, Nc);

		// Write back smoothed channel estimate
		for(int j = 0; j < Nc; j++)
			(estimated_channel + i*Nc + j)->value = buf_in[j];
	}

	delete[] buf_in;
	delete[] buf_out;
}

void cl_ofdm::channel_equalizer(std::complex <double>* in, std::complex <double>* out)
{
	// Hybrid ZF/MMSE equalizer.
	// PSK modes (amplitude restoration ON): pure ZF — H has |H|=1, ZF is optimal.
	// QAM modes (amplitude restoration OFF): ZF with MMSE-informed erasure.
	// Subcarriers where |H|² < σ²_n/9 (MMSE gain α < 0.1) are erased rather
	// than noise-amplified. CSI-weighted LLRs handle the rest.
	for(int i=0;i<Nsymb;i++)
	{
		for(int j=0;j<Nc;j++)
		{
			std::complex<double> H = (estimated_channel+i*Nc+j)->value;
			double H_mag_sq = H.real()*H.real() + H.imag()*H.imag();

			if(channel_estimator_amplitude_restoration == YES)
			{
				// PSK: |H|=1 after restoration, ZF is exact
				if(H_mag_sq > 1e-12)
					*(out+i*Nc+j) = *(in+i*Nc+j) / H;
				else
					*(out+i*Nc+j) = std::complex<double>(0.0, 0.0);
			}
			else
			{
				// QAM: MMSE-informed erasure on deeply faded subcarriers.
				// alpha = |H|²/(|H|²+σ²_n). When alpha < 0.1, the subcarrier
				// carries less information than noise — erase it.
				double alpha = H_mag_sq / (H_mag_sq + noise_variance_estimate);
				if(alpha > 0.1 && H_mag_sq > 1e-12)
					*(out+i*Nc+j) = *(in+i*Nc+j) / H;
				else
					*(out+i*Nc+j) = std::complex<double>(0.0, 0.0);
			}
			(estimated_channel+i*Nc+j)->status=UNKNOWN;
		}
	}
}
void cl_ofdm::channel_equalizer_without_amplitude_restoration(std::complex <double>* in,std::complex <double>* out)
{
	for(int i=0;i<Nsymb;i++)
	{
		for(int j=0;j<Nc;j++)
		{
			std::complex<double> H = (estimated_channel_without_amplitude_restoration+i*Nc+j)->value;
			double H_mag_sq = H.real()*H.real() + H.imag()*H.imag();
			if(H_mag_sq > 1e-12)
				*(out+i*Nc+j) = *(in+i*Nc+j) / H;
			else
				*(out+i*Nc+j) = std::complex<double>(0.0, 0.0);
		}
	}
}

int cl_ofdm::time_sync(std::complex <double>*in, int size, int interpolation_rate, int location_to_return)
{

	double corss_corr=0;
	double norm_a=0;
	double norm_b=0;

	int *corss_corr_loc=new int[size];
	double *corss_corr_vals=new double[size];
	int return_val;

	std::complex <double> *a_c, *b_c;

	for(int i=0;i<size;i++)
	{
		corss_corr_loc[i]=-1;
		corss_corr_vals[i]=0;
	}

	for(int i=0;i<size-(this->Ngi+this->Nfft)*interpolation_rate;i++)
	{
		a_c=in+i;
		b_c=in+i+this->Nfft*interpolation_rate;
		corss_corr=0;
		norm_a=0;
		norm_b=0;
		for(int j=0;j<Nsymb+preamble_configurator.Nsymb;j++)
		{
			if(j<time_sync_Nsymb)
			{
				for(int m=0;m<this->Ngi*interpolation_rate;m++)
				{
					corss_corr+=a_c[m+j*(this->Ngi+this->Nfft)*interpolation_rate].real()*b_c[m+j*(this->Ngi+this->Nfft)*interpolation_rate].real();
					norm_a+=a_c[m+j*(this->Ngi+this->Nfft)*interpolation_rate].real()*a_c[m+j*(this->Ngi+this->Nfft)*interpolation_rate].real();
					norm_b+=b_c[m+j*(this->Ngi+this->Nfft)*interpolation_rate].real()*b_c[m+j*(this->Ngi+this->Nfft)*interpolation_rate].real();

					corss_corr+=a_c[m+j*(this->Ngi+this->Nfft)*interpolation_rate].imag()*b_c[m+j*(this->Ngi+this->Nfft)*interpolation_rate].imag();
					norm_a+=a_c[m+j*(this->Ngi+this->Nfft)*interpolation_rate].imag()*a_c[m+j*(this->Ngi+this->Nfft)*interpolation_rate].imag();
					norm_b+=b_c[m+j*(this->Ngi+this->Nfft)*interpolation_rate].imag()*b_c[m+j*(this->Ngi+this->Nfft)*interpolation_rate].imag();
				}
			}
		}
		corss_corr=corss_corr/sqrt(norm_a*norm_b);
		corss_corr_vals[i]=corss_corr;
		corss_corr_loc[i]=i;
	}
	double tmp;
	int tmp_int;
	for(int i=0;i<size-(this->Ngi+this->Nfft)*interpolation_rate-1;i++)
	{
		for(int j=0;j<size-(this->Ngi+this->Nfft)*interpolation_rate-1;j++)
		{
			if (corss_corr_vals[j]<corss_corr_vals[j+1])
			{
				tmp=corss_corr_vals[j];
				corss_corr_vals[j]=corss_corr_vals[j+1];
				corss_corr_vals[j+1]=tmp;

				tmp_int=corss_corr_loc[j];
				corss_corr_loc[j]=corss_corr_loc[j+1];
				corss_corr_loc[j+1]=tmp_int;
			}
		}
	}
	return_val=corss_corr_loc[location_to_return];
	if(corss_corr_loc!=NULL)
	{
		delete[] corss_corr_loc;
	}
	if(corss_corr_vals!=NULL)
	{
		delete[] corss_corr_vals;
	}
	return return_val;
}

int cl_ofdm::time_sync_preamble(std::complex <double>*in, int size, int interpolation_rate, int location_to_return, int step, int nTrials_max)
{
	double corss_corr=0;
	double norm_a=0;
	double norm_b=0;

	// Grow-as-needed correlation buffers (shared with time_sync_preamble_with_metric)
	if(size > tsync_corr_size)
	{
		if(tsync_corr_loc!=NULL) delete[] tsync_corr_loc;
		if(tsync_corr_vals!=NULL) delete[] tsync_corr_vals;
		tsync_corr_loc = new int[size];
		tsync_corr_vals = new double[size];
		tsync_corr_size = size;
	}
	int *corss_corr_loc = tsync_corr_loc;
	double *corss_corr_vals = tsync_corr_vals;
	int return_val;


	std::complex <double> *a_c, *b_c;

	for(int i=0;i<size;i++)
	{
		corss_corr_loc[i]=-1;
		corss_corr_vals[i]=0;
	}

	int data_len = preamble_configurator.Nsymb*(this->Ngi+this->Nfft)*interpolation_rate;
	if(data_len > tsync_data_size)
	{
		if(tsync_data!=NULL) delete[] tsync_data;
		tsync_data = new std::complex<double>[data_len];
		tsync_data_size = data_len;
	}
	std::complex <double> *data = tsync_data;

	for(int i=0;i<size-preamble_configurator.Nsymb*(this->Ngi+this->Nfft)*interpolation_rate;i+=step)
	{
		for(int k=0;k<preamble_configurator.Nsymb*(this->Ngi+this->Nfft)*interpolation_rate;k++)
		{
			data[k]=*(in+i+k);
		}

		corss_corr=0;
		norm_a=0;
		norm_b=0;
		// GI-only correlation: the guard interval is a copy of the last Ngi
		// samples of the OFDM symbol, so GI[n] correlates perfectly with
		// symbol[Nfft+n] for real preambles regardless of which FFT bins
		// are active. The original Schmidl-Cox half-symbol correlation
		// (x[n] vs x[n+N/2]) requires only even-indexed subcarriers,
		// which Mercury's preamble does not satisfy (it uses all Nc bins).
		// Using GI-only gives metric ~1.0 for real preambles, ~0 for noise.
		for(int l=0;l<preamble_configurator.Nsymb;l++)
		{
			a_c=data+l*(this->Ngi+this->Nfft)*interpolation_rate;
			b_c=data+l*(this->Ngi+this->Nfft)*interpolation_rate+this->Nfft*interpolation_rate;

			for(int m=0;m<this->Ngi*interpolation_rate;m++)
			{
				corss_corr+=a_c[m].real()*b_c[m].real();
				norm_a+=a_c[m].real()*a_c[m].real();
				norm_b+=b_c[m].real()*b_c[m].real();

				corss_corr+=a_c[m].imag()*b_c[m].imag();
				norm_a+=a_c[m].imag()*a_c[m].imag();
				norm_b+=b_c[m].imag()*b_c[m].imag();
			}
		}

		if(norm_a < 0.001 || norm_b < 0.001)
			corss_corr = 0.0;
		else
			corss_corr=corss_corr/sqrt(norm_a*norm_b);
		corss_corr_vals[i]=corss_corr;
		corss_corr_loc[i]=i;
	}

	// Clamp location_to_return to valid range to prevent reading uninitialized sort entries
	if(location_to_return >= nTrials_max)
		location_to_return = nTrials_max - 1;

	for(int j=0;j<nTrials_max;j++)
	{
		corss_corr_loc[j]=j;
		for(int i=j+1;i<size;i++)
		{
			if (corss_corr_vals[i]>corss_corr_vals[j])
			{
				corss_corr_vals[j]=corss_corr_vals[i];
				corss_corr_loc[j]=i;
			}
		}
	}

	return_val=corss_corr_loc[location_to_return];
	return return_val;
/*
 * 	Ref: T. M. Schmidl and D. C. Cox, "Robust frequency and timing synchronization for OFDM," in IEEE Transactions on Communications, vol. 45, no. 12, pp. 1613-1621, Dec. 1997, doi: 10.1109/26.650240.
 *
 */
}

TimeSyncResult cl_ofdm::time_sync_preamble_with_metric(std::complex <double>*in, int size, int interpolation_rate, int location_to_return, int step, int nTrials_max)
{
	/*
	 * Fine per-trial timing refinement. Selects the sample-precise delay using
	 * a PHASE-INVARIANT magnitude statistic, mirroring the coarse
	 * time_sync_preamble_halfsym detector — CFO-robust. The candidate score is
	 * the average of two per-lag magnitude coefficients |P|²/(A²·R): one for
	 * the GI/cyclic-prefix lag (Nfft) and one for the repetition-period lag
	 * (L=Nfft/nIS). They are accumulated SEPARATELY and combined incoherently —
	 * summing the two different lags into one complex P would let them cancel
	 * under CFO (different phase ramps), reintroducing the very mistiming this
	 * fix removes. result.correlation is bounded [0,1] (Cauchy-Schwarz); ~1.0
	 * at a clean preamble, small at noise/data. NOTE: result.correlation from
	 * THIS fine function is NOT consumed by any caller (only result.delay is, at
	 * telecom_system.cc:2078-2082); the load-bearing receive_stats.coarse_metric
	 * is fed exclusively by the coarse detectors. See
	 * fact-documents/ofdm-fine-timing-magnitude.md §3.
	 */
	double corss_corr=0;
	double norm_a=0;
	double norm_b=0;
	double max_correlation = 0.0;

	TimeSyncResult result;
	result.delay = 0;
	result.correlation = 0.0;

	// Grow-as-needed correlation buffers
	if(size > tsync_corr_size)
	{
		if(tsync_corr_loc!=NULL) delete[] tsync_corr_loc;
		if(tsync_corr_vals!=NULL) delete[] tsync_corr_vals;
		tsync_corr_loc = new int[size];
		tsync_corr_vals = new double[size];
		tsync_corr_size = size;
	}
	int *corss_corr_loc = tsync_corr_loc;
	double *corss_corr_vals = tsync_corr_vals;

	std::complex <double> *a_c, *b_c;

	for(int i=0;i<size;i++)
	{
		corss_corr_loc[i]=-1;
		corss_corr_vals[i]=0;
	}

	int data_len = preamble_configurator.Nsymb*(this->Ngi+this->Nfft)*interpolation_rate;
	if(data_len > tsync_data_size)
	{
		if(tsync_data!=NULL) delete[] tsync_data;
		tsync_data = new std::complex<double>[data_len];
		tsync_data_size = data_len;
	}
	std::complex <double> *data = tsync_data;

	for(int i=0;i<size-preamble_configurator.Nsymb*(this->Ngi+this->Nfft)*interpolation_rate;i+=step)
	{
		for(int k=0;k<preamble_configurator.Nsymb*(this->Ngi+this->Nfft)*interpolation_rate;k++)
		{
			data[k]=*(in+i+k);
		}

		corss_corr=0;
		norm_a=0;
		norm_b=0;
		// PHASE-INVARIANT MAGNITUDE fine-timing metric (fix/ofdm-fine-timing-
		// magnitude, 2026-06-03). Previously this loop accumulated the
		// PHASE-SENSITIVE real projection of conj(a)*b:
		//     corss_corr += a.real()*b.real() + a.imag()*b.imag();   // Re(conj(a)*b)
		// which is |a||b|cos(theta) and COLLAPSES (and can go negative) under
		// residual CFO (post-Moose ~±20 Hz) as the repetition-period phase
		// theta drifts toward ±90°. The mis-selected peak then lands ±1 OFDM
		// symbol off, pilots misalign, mean_H collapses to ~0.30, and the
		// SKIP-H gate (telecom_system.cc:2486) rejects the frame before LDPC.
		//
		// Fix: score on the magnitude statistic |P|²/(A²·R) the COARSE detector
		// time_sync_preamble_halfsym uses (ofdm.cc:2581-2631, Schmidl & Cox 1997
		// / Wilson&Shang arXiv:2010.00762; FreeDV/STANAG/liquid-dsp all use
		// magnitude fine-timing). Bounded [0,1] by Cauchy-Schwarz.
		//
		// CRITICAL: this loop correlates TWO DIFFERENT LAGS — the GI/cyclic-
		// prefix lag (Nfft samples) and the repetition-period lag (L=Nfft/nIS
		// samples). Under CFO each lag accrues a DIFFERENT phase ramp
		// (e^{j2πfΔt}, Δt ∝ lag). Summing both into ONE complex P before |·|²
		// makes the two lag families add with different phases and partially
		// CANCEL — that cross-lag cancellation, not the per-lag phase, is what
		// jumps the fine peak ±symbols under CFO. So accumulate the two lag
		// families in SEPARATE complex accumulators (each internally coherent,
		// hence phase-invariant) and combine their normalized magnitudes
		// INCOHERENTLY (average of two [0,1] coefficients → still [0,1]). This
		// is the noncoherent-combining form; it keeps both timing sources (GI =
		// sample precision, repetition = strong period lock) without the
		// cross-lag CFO cancellation. corss_corr/P_imag/norm_a/norm_b below hold
		// the *result* (so the unchanged downstream sort/return code still
		// works); the per-lag accumulators are local.
		double Pg_re=0, Pg_im=0, Ag=0, Rg=0;   // GI lag (Nfft)
		double Pr_re=0, Pr_im=0, Ar=0, Rr=0;   // repetition lag (L=Nfft/nIS)
		// GI: correlate cyclic prefix with end of FFT symbol (Ngi samples/symbol).
		// Repetition: preamble subcarrier pattern creates time-domain repetition
		// with period L = Nfft/nIS (nIS=4 for WB every-4th, nIS=2 for NB every-2nd).
		// Correlate adjacent L-sample sections within the FFT window.
		int nIS = preamble_configurator.nIdentical_sections;
		int L_interp = (this->Nfft / nIS) * interpolation_rate;
		for(int l=0;l<preamble_configurator.Nsymb;l++)
		{
			a_c=data+l*(this->Ngi+this->Nfft)*interpolation_rate;
			b_c=data+l*(this->Ngi+this->Nfft)*interpolation_rate+this->Nfft*interpolation_rate;

			for(int m=0;m<this->Ngi*interpolation_rate;m++)
			{
				// conj(a)*b = (ar*br + ai*bi) + j(ar*bi - ai*br)
				Pg_re += a_c[m].real()*b_c[m].real() + a_c[m].imag()*b_c[m].imag();
				Pg_im += a_c[m].real()*b_c[m].imag() - a_c[m].imag()*b_c[m].real();
				Ag    += a_c[m].real()*a_c[m].real() + a_c[m].imag()*a_c[m].imag();
				Rg    += b_c[m].real()*b_c[m].real() + b_c[m].imag()*b_c[m].imag();
			}

			// Correlate all (nIS-1) adjacent pairs within the FFT window.
			// For nIS=2: 1 pair of 128 samples = 128 (same as before).
			// For nIS=4: 3 pairs of 64 samples = 192 (more robust).
			for(int pair=0;pair<nIS-1;pair++)
			{
				a_c=data+l*(this->Ngi+this->Nfft)*interpolation_rate+(this->Ngi)*interpolation_rate+pair*L_interp;
				b_c=a_c+L_interp;

				for(int m=0;m<L_interp;m++)
				{
					Pr_re += a_c[m].real()*b_c[m].real() + a_c[m].imag()*b_c[m].imag();
					Pr_im += a_c[m].real()*b_c[m].imag() - a_c[m].imag()*b_c[m].real();
					Ar    += a_c[m].real()*a_c[m].real() + a_c[m].imag()*a_c[m].imag();
					Rr    += b_c[m].real()*b_c[m].real() + b_c[m].imag()*b_c[m].imag();
				}
			}
		}

		// Norm threshold: VB-Cable silence has amplitude ~1e-10 (nonzero).
		// Norms accumulate to ~1e-18 and the ratio produces unstable metrics
		// (up to 0.93) that can beat real preamble peaks. Use threshold
		// instead of exact == 0.0 to suppress these degenerate cases. Apply the
		// floor to the COMBINED per-half energy (the old norm_a/norm_b roles).
		norm_a = Ag + Ar;   // total "a-half" energy (kept for the silence gate)
		norm_b = Rg + Rr;   // total "b-half" energy
		if(norm_a < 0.001 || norm_b < 0.001)
		{
			corss_corr = 0.0;
		}
		else
		{
			// Per-lag magnitude coefficients, each bounded [0,1] (Cauchy-Schwarz),
			// each phase-invariant within its lag. Average them (still [0,1]).
			// A lag family with ~zero energy (e.g. nIS=1 → no rep pairs) is
			// dropped from the average rather than dividing by zero.
			double mg = (Ag*Rg > 1e-20) ? (Pg_re*Pg_re + Pg_im*Pg_im)/(Ag*Rg) : -1.0;
			double mr = (Ar*Rr > 1e-20) ? (Pr_re*Pr_re + Pr_im*Pr_im)/(Ar*Rr) : -1.0;
			if(mg >= 0.0 && mr >= 0.0)      corss_corr = 0.5*(mg + mr);
			else if(mg >= 0.0)              corss_corr = mg;
			else if(mr >= 0.0)              corss_corr = mr;
			else                            corss_corr = 0.0;
		}
		corss_corr_vals[i]=corss_corr;
		corss_corr_loc[i]=i;
	}

	// Clamp location_to_return to valid range to prevent reading uninitialized sort entries
	if(location_to_return >= nTrials_max)
		location_to_return = nTrials_max - 1;

	for(int j=0;j<nTrials_max;j++)
	{
		corss_corr_loc[j]=j;
		for(int i=j+1;i<size;i++)
		{
			if (corss_corr_vals[i]>corss_corr_vals[j])
			{
				corss_corr_vals[j]=corss_corr_vals[i];
				corss_corr_loc[j]=i;
			}
		}
	}

	result.delay = corss_corr_loc[location_to_return];
	// Get the correlation value at the returned location
	max_correlation = corss_corr_vals[location_to_return];
	result.correlation = max_correlation;

	return result;
}

TimeSyncResult cl_ofdm::time_sync_preamble_halfsym(std::complex<double>* in, int size, int interpolation_rate, int step, double early_exit_metric)
{
	/*
	 * Schmidl-Cox preamble detection using time-domain repetition.
	 *
	 * Preamble subcarrier pattern creates period L = Nfft/nIS:
	 *   nIS=4 (WB every-4th): L = Nfft/4
	 *   nIS=2 (NB every-2nd): L = Nfft/2
	 * r(d+m) = r(d+m+L) for all m within each symbol. Data symbols lack
	 * this periodicity. Sliding L-sample windows give |P|²/R² ≈ 1.0 at
	 * preamble, ≈ 0.0 at data, at ANY sample position (no GI alignment
	 * needed). GI extends the periodicity through the cyclic prefix.
	 *
	 * Uses magnitude-squared metric for phase-rotation invariance:
	 *   P = sum conj(r(d+m)) * r(d+m+L)
	 *   R = sum |r(d+m+L)|^2
	 *   M = |P|^2 / R^2
	 */
	int nIS = preamble_configurator.nIdentical_sections;
	int L = (this->Nfft / nIS) * interpolation_rate;
	int Nofdm = (this->Ngi + this->Nfft) * interpolation_rate;
	int nsym = preamble_configurator.Nsymb;
	int pream_len = nsym * Nofdm;

	TimeSyncResult result;
	result.delay = 0;
	result.correlation = 0.0;

	double best_weighted = -1.0;
	double best_normalized = 0.0;
	int best_pos = 0;

	for(int d = 0; d <= size - pream_len; d += step)
	{
		double P_real = 0.0, P_imag = 0.0;
		double A2 = 0.0, R = 0.0;

		for(int sym = 0; sym < nsym; sym++)
		{
			int base = d + sym * Nofdm;
			for(int m = 0; m < L; m++)
			{
				double ar = in[base + m].real();
				double ai = in[base + m].imag();
				double br = in[base + m + L].real();
				double bi = in[base + m + L].imag();

				// conj(a) * b = (ar*br + ai*bi) + j(ar*bi - ai*br)
				P_real += ar * br + ai * bi;
				P_imag += ar * bi - ai * br;

				A2 += ar * ar + ai * ai;
				R += br * br + bi * bi;
			}
		}

		// Correlation coefficient: M = |P|² / (A² · R)
		// Bounded [0, 1] by Cauchy-Schwarz. Normalizing by both halves
		// prevents metric explosion at signal/silence boundaries where
		// A² >> R (first half has signal, second half is silence).
		double metric = 0.0;
		double denom = A2 * R;
		if(denom > 1e-20)
			metric = (P_real * P_real + P_imag * P_imag) / denom;

		// Energy-weighted selection: use metric * energy to pick best position.
		// Pure normalized metric gives false peaks on silence (VB-Cable digital
		// silence has energy ~1e-12, making denom ~1e-19 which barely exceeds
		// 1e-20, producing unstable correlation ratios ~0.1-1.0).
		// Weighting by (A2+R) ensures silence (energy ~0) never beats real
		// signal, while preserving correct detection among signal positions.
		double weighted = metric * (A2 + R);
		if(weighted > best_weighted)
		{
			best_weighted = weighted;
			best_normalized = metric;
			best_pos = d;
		}

		// Early exit: return the FIRST position where normalized metric
		// exceeds threshold. Energy floor rejects false peaks on digital
		// silence. This finds the earliest preamble in the buffer rather
		// than the strongest, preventing later frames from shadowing
		// earlier ones when multiple back-to-back frames are present.
		if(early_exit_metric > 0.0 && metric >= early_exit_metric
			&& (A2 + R) > 1e-6)
		{
			result.delay = d;
			result.correlation = metric;
			return result;
		}
	}

	result.delay = best_pos;
	result.correlation = best_normalized;
	return result;
}

TimeSyncResult cl_ofdm::time_sync_preamble_halfsym_2phase(
	std::complex<double>* in, int size, int interpolation_rate,
	double early_exit_metric)
{
	/*
	 * Two-phase Schmidl-Cox preamble detection:
	 *   Phase 1: coarse search at GI stride (fast, finds approximate position)
	 *   Phase 2: fine search at baseband stride within ±1 GI of coarse peak
	 *
	 * When early_exit_metric > 0, Phase 1 returns the FIRST position with
	 * metric >= threshold (earliest preamble) instead of the maximum.
	 * Phase 2 always uses max-metric for precise sample alignment.
	 */
	int gi_interp = Ngi * interpolation_rate;
	int pream_len = preamble_configurator.Nsymb * (Ngi + Nfft) * interpolation_rate;

	// Phase 1: coarse at GI stride (early exit finds earliest preamble)
	TimeSyncResult coarse = time_sync_preamble_halfsym(
		in, size, interpolation_rate, gi_interp, early_exit_metric);

	if(coarse.correlation < 0.05)
		return coarse;  // No preamble found

	// Phase 2: fine at baseband stride around coarse peak.
	// When early exit was used, the coarse position may be at the transition
	// edge (metric ramps from 0 to 1.0 over ~3 symbol widths as the window
	// slides into the preamble). Early exit at 0.5 triggers ~2 symbols before
	// the true peak. Widen fine search to ±1 preamble length so the true peak
	// is always reachable. Without early exit, ±1 GI suffices.
	int fine_margin = (early_exit_metric > 0.0) ? pream_len : gi_interp;
	int fine_start = coarse.delay - fine_margin;
	if(fine_start < 0) fine_start = 0;
	int fine_size = 2 * fine_margin + pream_len;
	if(fine_start + fine_size > size)
		fine_size = size - fine_start;
	if(fine_size <= pream_len)
		return coarse;  // Not enough room for fine search

	TimeSyncResult fine = time_sync_preamble_halfsym(
		&in[fine_start], fine_size, interpolation_rate, interpolation_rate);
	fine.delay += fine_start;
	return fine;
}

TimeSyncResult cl_ofdm::time_sync_preamble_fft(
	std::complex<double>* baseband_interp, int buffer_size_interp,
	int interpolation_rate, int preamble_nSymb)
{
	/*
	 * FFT-based preamble detection for narrowband OFDM.
	 *
	 * Coarse search at GI-period steps. Per-bin coherent across symbols
	 * (timing-dependent phase is constant per bin across symbols), non-coherent
	 * across bins (avoids cross-bin phase spread from timing offset).
	 *
	 * PREAMBLE-SPECIFIC: correlates against known preamble subcarrier values.
	 * Data symbols produce metric ≈ 1 (random correlation), preamble ≈ Nsym.
	 *
	 * Normalized metric = Σ|bin_accum|² / Σ|FFT_bin|² × |P|²
	 *   ≈ preamble_nSymb at preamble (coherent gain)
	 *   ≈ 1 at noise or data (random walk)
	 * Threshold of ~2 gives reliable discrimination.
	 *
	 * Fine timing is handled by GI correlation in the caller (±1 symbol window).
	 */

	int Nofdm = Nfft + Ngi;
	int symbol_interp = Nofdm * interpolation_rate;
	int preamble_interp = preamble_nSymb * symbol_interp;
	int gi_interp = Ngi * interpolation_rate;

	// Precompute preamble bin list
	int n_preamble_bins_per_sym[16] = {};
	int preamble_bin_list[16][256];
	int preamble_bin_fft[16][256];

	for(int sym = 0; sym < preamble_nSymb && sym < 16; sym++)
	{
		int nb = 0;
		for(int k = 0; k < Nc; k++)
		{
			if(ofdm_preamble[sym * Nc + k].type == PREAMBLE)
			{
				int fft_bin;
				if(k < Nc / 2)
					fft_bin = k + Nfft - Nc / 2;
				else
					fft_bin = k - Nc / 2 + start_shift;
				preamble_bin_list[sym][nb] = k;
				preamble_bin_fft[sym][nb] = fft_bin;
				nb++;
			}
		}
		n_preamble_bins_per_sym[sym] = nb;
	}

	// Local FFT buffers
	std::complex<double> fft_in[256];
	std::complex<double> fft_out[256];

	// GI-period steps guarantee worst-case ±gi_interp/2 offset from symbol
	// boundary. Symbol-period steps are 17× coarser (1088 vs 64 at interp=4)
	// and cause ISI when the FFT window extends past the 64-sample GI.
	int search_step = gi_interp;
	int n_coarse = (buffer_size_interp - preamble_interp) / search_step;
	if(n_coarse < 0) n_coarse = 0;
	if(n_coarse > 1023) n_coarse = 1023;  // safety cap for local arrays

	double best_coarse_metric = -1.0;
	int best_coarse_pos = 0;
	double coarse_metrics[1024];  // stack alloc (n_coarse capped to 1023 above)

	std::complex<double> bin_accum[256];

	for(int pos = 0; pos <= n_coarse; pos++)
	{
		int sample_start = pos * search_step;

		int max_bins = n_preamble_bins_per_sym[0];
		for(int b = 0; b < max_bins; b++)
			bin_accum[b] = std::complex<double>(0.0, 0.0);

		for(int sym = 0; sym < preamble_nSymb; sym++)
		{
			int sym_start = sample_start + sym * symbol_interp;
			int fft_start = sym_start + gi_interp;

			if(fft_start + (Nfft - 1) * interpolation_rate >= buffer_size_interp)
				break;

			for(int k = 0; k < Nfft; k++)
				fft_in[k] = baseband_interp[fft_start + k * interpolation_rate];

			fft(fft_in, fft_out, Nfft);

			for(int b = 0; b < n_preamble_bins_per_sym[sym]; b++)
			{
				int sc = preamble_bin_list[sym][b];
				bin_accum[b] += fft_out[preamble_bin_fft[sym][b]] * std::conj(ofdm_preamble[sym * Nc + sc].value);
			}
		}

		double metric = 0.0;
		for(int b = 0; b < n_preamble_bins_per_sym[0]; b++)
			metric += std::norm(bin_accum[b]);

		coarse_metrics[pos] = metric;
		if(metric > best_coarse_metric)
		{
			best_coarse_metric = metric;
			best_coarse_pos = sample_start;
		}
	}

	// Prefer earliest position with metric >= 50% of max.
	// Preamble gives ~Nsymb² × E, data gives ~Nsymb × E (random walk).
	// At ratio 4:1, 50% of preamble max is 2× average data max —
	// data never reaches this, so the earliest preamble is always selected.
	// This prevents the FFT from jumping to a later frame when an earlier
	// preamble has slightly lower metric (e.g., timing offset effects).
	double early_threshold = best_coarse_metric * 0.5;
	for(int pos = 0; pos <= n_coarse; pos++)
	{
		if(coarse_metrics[pos] >= early_threshold)
		{
			best_coarse_metric = coarse_metrics[pos];
			best_coarse_pos = pos * search_step;
			break;
		}
	}

	// Normalize: re-FFT at best position and compute energy at preamble bins
	double energy = 0.0;
	{
		int sample_start = best_coarse_pos;
		for(int sym = 0; sym < preamble_nSymb; sym++)
		{
			int sym_start = sample_start + sym * symbol_interp;
			int fft_start = sym_start + gi_interp;

			if(fft_start + (Nfft - 1) * interpolation_rate >= buffer_size_interp)
				break;

			for(int k = 0; k < Nfft; k++)
				fft_in[k] = baseband_interp[fft_start + k * interpolation_rate];

			fft(fft_in, fft_out, Nfft);

			for(int b = 0; b < n_preamble_bins_per_sym[sym]; b++)
				energy += std::norm(fft_out[preamble_bin_fft[sym][b]]);
		}
	}

	TimeSyncResult result;
	result.delay = best_coarse_pos;
	result.correlation = (energy > 0.0) ? best_coarse_metric / energy : 0.0;
	return result;
}

TimeSyncResult cl_ofdm::time_sync_preamble_fft_fine(
	std::complex<double>* baseband_interp, int buffer_size_interp,
	int interpolation_rate, int preamble_nSymb,
	int coarse_pos, int search_half_window)
{
	/*
	 * FFT fine preamble detection + GI-only sample-level refinement.
	 *
	 * Stage 1: FFT search at half-GI steps in a narrow window around coarse_pos.
	 *          Same metric as time_sync_preamble_fft() but finer grid.
	 *          Resolves the "wrong symbol boundary" ambiguity that GI+halfsym
	 *          Phase 2 cannot solve (GI+halfsym gives 0.88-0.99 on ALL symbols).
	 *
	 * Stage 2: GI-only correlation at step=1 within ±gi_interp/2 of the FFT
	 *          fine position. Safe because FFT already confirmed the correct
	 *          symbol — GI just fine-tunes within the guard interval.
	 *
	 * Returns: sample-accurate position with FFT metric (the discriminating one).
	 */

	int Nofdm = Nfft + Ngi;
	int symbol_interp = Nofdm * interpolation_rate;
	int preamble_interp = preamble_nSymb * symbol_interp;
	int gi_interp = Ngi * interpolation_rate;

	// --- Precompute preamble bin list (same as time_sync_preamble_fft) ---
	int n_preamble_bins_per_sym[16] = {};
	int preamble_bin_list[16][256];
	int preamble_bin_fft[16][256];

	for(int sym = 0; sym < preamble_nSymb && sym < 16; sym++)
	{
		int nb = 0;
		for(int k = 0; k < Nc; k++)
		{
			if(ofdm_preamble[sym * Nc + k].type == PREAMBLE)
			{
				int fft_bin;
				if(k < Nc / 2)
					fft_bin = k + Nfft - Nc / 2;
				else
					fft_bin = k - Nc / 2 + start_shift;
				preamble_bin_list[sym][nb] = k;
				preamble_bin_fft[sym][nb] = fft_bin;
				nb++;
			}
		}
		n_preamble_bins_per_sym[sym] = nb;
	}

	std::complex<double> fft_in[256];
	std::complex<double> fft_out[256];
	std::complex<double> bin_accum[256];

	// --- Stage 1: FFT fine search at half-GI steps ---
	int fine_step = gi_interp / 2;
	if(fine_step < 1) fine_step = 1;

	int win_start = coarse_pos - search_half_window;
	if(win_start < 0) win_start = 0;
	int win_end = coarse_pos + search_half_window;
	if(win_end + preamble_interp > buffer_size_interp)
		win_end = buffer_size_interp - preamble_interp;
	if(win_end < win_start) win_end = win_start;

	int n_fine = (win_end - win_start) / fine_step;
	if(n_fine < 0) n_fine = 0;
	if(n_fine > 511) n_fine = 511;

	double best_fine_metric = -1.0;
	int best_fine_pos = coarse_pos;
	double fine_metrics[512];  // stack alloc (n_fine capped to 511 above)

	for(int idx = 0; idx <= n_fine; idx++)
	{
		int sample_start = win_start + idx * fine_step;

		int max_bins = n_preamble_bins_per_sym[0];
		for(int b = 0; b < max_bins; b++)
			bin_accum[b] = std::complex<double>(0.0, 0.0);

		for(int sym = 0; sym < preamble_nSymb; sym++)
		{
			int sym_start = sample_start + sym * symbol_interp;
			int fft_start = sym_start + gi_interp;

			if(fft_start + (Nfft - 1) * interpolation_rate >= buffer_size_interp)
				break;

			for(int k = 0; k < Nfft; k++)
				fft_in[k] = baseband_interp[fft_start + k * interpolation_rate];

			fft(fft_in, fft_out, Nfft);

			for(int b = 0; b < n_preamble_bins_per_sym[sym]; b++)
			{
				int sc = preamble_bin_list[sym][b];
				bin_accum[b] += fft_out[preamble_bin_fft[sym][b]] * std::conj(ofdm_preamble[sym * Nc + sc].value);
			}
		}

		double metric = 0.0;
		for(int b = 0; b < n_preamble_bins_per_sym[0]; b++)
			metric += std::norm(bin_accum[b]);

		fine_metrics[idx] = metric;
		if(metric > best_fine_metric)
		{
			best_fine_metric = metric;
			best_fine_pos = win_start + idx * fine_step;
		}
	}

	// Earliest-above-50%-of-max selection (same as coarse)
	double early_threshold = best_fine_metric * 0.5;
	for(int idx = 0; idx <= n_fine; idx++)
	{
		if(fine_metrics[idx] >= early_threshold)
		{
			best_fine_metric = fine_metrics[idx];
			best_fine_pos = win_start + idx * fine_step;
			break;
		}
	}

	// Normalize FFT metric (same as time_sync_preamble_fft)
	double energy = 0.0;
	{
		int sample_start = best_fine_pos;
		for(int sym = 0; sym < preamble_nSymb; sym++)
		{
			int sym_start = sample_start + sym * symbol_interp;
			int fft_start = sym_start + gi_interp;

			if(fft_start + (Nfft - 1) * interpolation_rate >= buffer_size_interp)
				break;

			for(int k = 0; k < Nfft; k++)
				fft_in[k] = baseband_interp[fft_start + k * interpolation_rate];

			fft(fft_in, fft_out, Nfft);

			for(int b = 0; b < n_preamble_bins_per_sym[sym]; b++)
				energy += std::norm(fft_out[preamble_bin_fft[sym][b]]);
		}
	}

	double fft_metric = (energy > 0.0) ? best_fine_metric / energy : 0.0;

	// --- Stage 2: GI-only refinement at step=1 within ±gi_interp/2 ---
	// GI correlation: correlate cyclic prefix (first Ngi samples) with the
	// matching end of the FFT window (last Ngi samples) across all preamble symbols.
	int gi_win_start = best_fine_pos - gi_interp / 2;
	if(gi_win_start < 0) gi_win_start = 0;
	int gi_win_end = best_fine_pos + gi_interp / 2;
	if(gi_win_end + preamble_interp > buffer_size_interp)
		gi_win_end = buffer_size_interp - preamble_interp;
	if(gi_win_end < gi_win_start) gi_win_end = gi_win_start;

	double best_gi_metric = -1.0;
	int best_gi_pos = best_fine_pos;

	for(int pos = gi_win_start; pos <= gi_win_end; pos++)
	{
		double corr = 0.0, na = 0.0, nb = 0.0;
		for(int sym = 0; sym < preamble_nSymb; sym++)
		{
			int base = pos + sym * symbol_interp;
			std::complex<double>* a = &baseband_interp[base];
			std::complex<double>* b = &baseband_interp[base + Nfft * interpolation_rate];
			for(int m = 0; m < gi_interp; m++)
			{
				corr += a[m].real() * b[m].real() + a[m].imag() * b[m].imag();
				na += a[m].real() * a[m].real() + a[m].imag() * a[m].imag();
				nb += b[m].real() * b[m].real() + b[m].imag() * b[m].imag();
			}
		}

		double gi_metric;
		if(na < 0.001 || nb < 0.001)
			gi_metric = -1.0;
		else
			gi_metric = corr / sqrt(na * nb);

		if(gi_metric > best_gi_metric)
		{
			best_gi_metric = gi_metric;
			best_gi_pos = pos;
		}
	}

	TimeSyncResult result;
	result.delay = best_gi_pos;
	result.correlation = fft_metric;  // Return FFT metric (the discriminating one)
	return result;
}

TimeSyncResult cl_ofdm::time_sync_preamble_matched(
	std::complex<double>* baseband_interp, int buffer_size_interp,
	int interpolation_rate, int preamble_nSymb)
{
	/*
	 * Matched-filter preamble detection: time-domain cross-correlation
	 * against FIR-round-tripped preamble template. Zero FFTs.
	 *
	 * Two phases:
	 *   Coarse: GI-period stride, ALL preamble symbols for discrimination.
	 *           Per-symbol Cauchy-Schwarz: |corr|² / (E_template × E_rx) → [0,1].
	 *           Amplitude-independent: works at any RX gain / HF fading level.
	 *
	 *   Fine:   baseband-sample stride (step = interpolation_rate) within
	 *           ±gi_interp of coarse peak. All preamble symbols.
	 *
	 * Metric convention: sum of per-symbol normalized correlations.
	 *   Preamble ≈ nSymb (e.g. 4.0), data ≈ 0.1-0.3. Threshold = 2.0.
	 */

	if (ofdm_corr_template == NULL || ofdm_corr_template_len <= 0)
	{
		TimeSyncResult r;
		r.delay = 0;
		r.correlation = 0.0;
		return r;
	}

	int Nofdm = Nfft + Ngi;
	int sym_interp = Nofdm * interpolation_rate;
	int preamble_interp = preamble_nSymb * sym_interp;
	int gi_interp = Ngi * interpolation_rate;
	int template_nsymb = ofdm_corr_template_nsymb;
	if (template_nsymb > preamble_nSymb) template_nsymb = preamble_nSymb;

	// ---- Coarse search: GI-period stride, ALL symbols ----
	// Uses all preamble symbols for discrimination. Single-symbol coarse
	// was insufficient: data can randomly correlate with one template symbol
	// at 0.4-0.5, causing the earliest-above-50% heuristic to pick false peaks.
	// All-symbol coarse: ~962K MACs (NB) — still 4x cheaper than FFT approach.
	int coarse_stride = gi_interp;
	int n_coarse = (buffer_size_interp - preamble_interp) / coarse_stride;
	if (n_coarse < 0) n_coarse = 0;

	double best_coarse_metric = -1.0;
	int best_coarse_pos = 0;


	for (int ci = 0; ci <= n_coarse; ci++)
	{
		int pos = ci * coarse_stride;

		double total_metric = 0.0;

		for (int k = 0; k < template_nsymb; k++)
		{
			int tmpl_offset = k * Nofdm;
			int rx_offset = pos + k * sym_interp;

			if (rx_offset + (Nofdm - 1) * interpolation_rate >= buffer_size_interp)
				break;

			double corr_re = 0.0, corr_im = 0.0;
			double e_rx = 0.0;

			for (int n = 0; n < Nofdm; n++)
			{
				std::complex<double> rx = baseband_interp[rx_offset + n * interpolation_rate];
				double t_re = ofdm_corr_template[tmpl_offset + n].real();
				double t_im = ofdm_corr_template[tmpl_offset + n].imag();

				corr_re += t_re * rx.real() + t_im * rx.imag();
				corr_im += t_im * rx.real() - t_re * rx.imag();
				e_rx += rx.real() * rx.real() + rx.imag() * rx.imag();
			}

			// Cauchy-Schwarz: |corr|²/(E_template × E_rx) → [0,1] per symbol.
			// Amplitude-independent: works regardless of RX gain, HF fading, etc.
			// Silence protection: energy gate in caller rejects e_rx ≈ 0 cases.
			double denom = ofdm_corr_template_sym_energy[k] * e_rx;
			if (denom > 1e-30)
				total_metric += (corr_re * corr_re + corr_im * corr_im) / denom;
		}

		if (total_metric > best_coarse_metric)
		{
			best_coarse_metric = total_metric;
			best_coarse_pos = pos;
		}
	}

	// Use peak position directly. The earliest-above-50% heuristic from FFT
	// detection doesn't work here: with K identical preamble symbols, an
	// offset-by-1-symbol position gives (K-1)/K match which exceeds 50% for
	// all K>=2, systematically selecting the wrong (one-symbol-early) position.
	int coarse_result_pos = best_coarse_pos;

	// DIAG: per-symbol breakdown at best coarse position (remove after debug)
	if (best_coarse_metric < 2.0)
	{
		printf("[MF-DIAG] best_coarse=%.4f pos=%d n_coarse=%d\n",
			best_coarse_metric, best_coarse_pos, n_coarse);
		for (int k = 0; k < template_nsymb; k++)
		{
			int tmpl_offset = k * Nofdm;
			int rx_offset = best_coarse_pos + k * sym_interp;
			if (rx_offset + (Nofdm - 1) * interpolation_rate >= buffer_size_interp)
				break;
			double corr_re = 0, corr_im = 0, e_rx = 0;
			for (int n = 0; n < Nofdm; n++)
			{
				std::complex<double> rx = baseband_interp[rx_offset + n * interpolation_rate];
				double t_re = ofdm_corr_template[tmpl_offset + n].real();
				double t_im = ofdm_corr_template[tmpl_offset + n].imag();
				corr_re += t_re * rx.real() + t_im * rx.imag();
				corr_im += t_im * rx.real() - t_re * rx.imag();
				e_rx += rx.real() * rx.real() + rx.imag() * rx.imag();
			}
			double denom = ofdm_corr_template_sym_energy[k] * e_rx;
			double cs = (denom > 1e-30) ? (corr_re*corr_re + corr_im*corr_im) / denom : 0;
			printf("  sym%d: cs=%.4f e_tmpl=%.1f e_rx=%.1f |corr|2=%.1f\n",
				k, cs, ofdm_corr_template_sym_energy[k], e_rx,
				corr_re*corr_re + corr_im*corr_im);
		}
		// Also print a few raw template and rx samples at the best position
		if (template_nsymb > 0)
		{
			printf("  tmpl[0..4]: ");
			for (int n = 0; n < 5 && n < Nofdm; n++)
				printf("(%.4f,%.4f) ", ofdm_corr_template[n].real(), ofdm_corr_template[n].imag());
			printf("\n  rx[0..4]:   ");
			for (int n = 0; n < 5 && n < Nofdm; n++)
			{
				std::complex<double> rx = baseband_interp[best_coarse_pos + n * interpolation_rate];
				printf("(%.4f,%.4f) ", rx.real(), rx.imag());
			}
			printf("\n");
		}
		fflush(stdout);
	}

	// If no preamble detected at coarse level, return early.
	// Coarse is a pre-filter; fine search makes the real decision.
	// Threshold 0.1: blocks pure noise, allows degraded preambles through.
	if (best_coarse_metric < 0.1)
	{
		TimeSyncResult r;
		r.delay = coarse_result_pos;
		r.correlation = best_coarse_metric;
		return r;
	}

	// ---- Fine search: all symbols, step = interpolation_rate, ±gi_interp ----
	int fine_start = coarse_result_pos - gi_interp;
	if (fine_start < 0) fine_start = 0;
	int fine_end = coarse_result_pos + gi_interp;
	if (fine_end + preamble_interp > buffer_size_interp)
		fine_end = buffer_size_interp - preamble_interp;
	if (fine_end < fine_start) fine_end = fine_start;

	double best_fine_metric = -1.0;
	int best_fine_pos = coarse_result_pos;

	for (int pos = fine_start; pos <= fine_end; pos += interpolation_rate)
	{
		double total_metric = 0.0;

		for (int k = 0; k < template_nsymb; k++)
		{
			int tmpl_offset = k * Nofdm;
			int rx_offset = pos + k * sym_interp;

			if (rx_offset + (Nofdm - 1) * interpolation_rate >= buffer_size_interp)
				break;

			double corr_re = 0.0, corr_im = 0.0;
			double e_rx = 0.0;

			for (int n = 0; n < Nofdm; n++)
			{
				std::complex<double> rx = baseband_interp[rx_offset + n * interpolation_rate];
				double t_re = ofdm_corr_template[tmpl_offset + n].real();
				double t_im = ofdm_corr_template[tmpl_offset + n].imag();

				corr_re += t_re * rx.real() + t_im * rx.imag();
				corr_im += t_im * rx.real() - t_re * rx.imag();
				e_rx += rx.real() * rx.real() + rx.imag() * rx.imag();
			}

			double denom = ofdm_corr_template_sym_energy[k] * e_rx;
			if (denom > 1e-30)
				total_metric += (corr_re * corr_re + corr_im * corr_im) / denom;
		}

		if (total_metric > best_fine_metric)
		{
			best_fine_metric = total_metric;
			best_fine_pos = pos;
		}
	}

	// DIAG: per-quarter energy decomposition at fine search peak
	if (best_fine_metric < 2.0 && best_fine_metric >= 0.5)
	{
		printf("[MF-DIAG2] fine_pos=%d fine_metric=%.4f interp=%d Nofdm=%d Ngi=%d\n",
			best_fine_pos, best_fine_metric, interpolation_rate, Nofdm, Ngi);
		// Analyze sym0 AND sym1 to see if Q1 dead zone is consistent
		for (int k = 0; k < template_nsymb && k < 2; k++)
		{
			int tmpl_off = k * Nofdm;
			int rx_off = best_fine_pos + k * sym_interp;
			if (rx_off + (Nofdm - 1) * interpolation_rate >= buffer_size_interp)
				break;
			int quarter = Nofdm / 4;
			printf("  sym%d:", k);
			for (int q = 0; q < 4; q++)
			{
				double e_t = 0, e_r = 0, cr = 0, ci = 0;
				for (int n = q * quarter; n < (q + 1) * quarter; n++)
				{
					std::complex<double> rx = baseband_interp[rx_off + n * interpolation_rate];
					double t_re = ofdm_corr_template[tmpl_off + n].real();
					double t_im = ofdm_corr_template[tmpl_off + n].imag();
					e_t += t_re * t_re + t_im * t_im;
					e_r += rx.real() * rx.real() + rx.imag() * rx.imag();
					cr += t_re * rx.real() + t_im * rx.imag();
					ci += t_im * rx.real() - t_re * rx.imag();
				}
				double cs = (e_t * e_r > 1e-30) ? (cr * cr + ci * ci) / (e_t * e_r) : 0;
				printf(" Q%d[Et=%.3f Er=%.3f CS=%.3f]", q + 1, e_t, e_r, cs);
			}
			printf("\n");
		}
		// Print first 8 and last 8 template+RX sample amplitudes for sym1
		int k = (template_nsymb > 1) ? 1 : 0;
		int tmpl_off = k * Nofdm;
		int rx_off = best_fine_pos + k * sym_interp;
		printf("  tmpl[0..7]:");
		for (int n = 0; n < 8; n++)
			printf(" %.4f", std::abs(ofdm_corr_template[tmpl_off + n]));
		printf("\n  rx  [0..7]:");
		for (int n = 0; n < 8; n++)
			printf(" %.4f", std::abs(baseband_interp[rx_off + n * interpolation_rate]));
		printf("\n  tmpl[%d..%d]:", Nofdm - 8, Nofdm - 1);
		for (int n = Nofdm - 8; n < Nofdm; n++)
			printf(" %.4f", std::abs(ofdm_corr_template[tmpl_off + n]));
		printf("\n  rx  [%d..%d]:", Nofdm - 8, Nofdm - 1);
		for (int n = Nofdm - 8; n < Nofdm; n++)
			printf(" %.4f", std::abs(baseband_interp[rx_off + n * interpolation_rate]));
		printf("\n");
		fflush(stdout);
	}

	TimeSyncResult result;
	result.delay = best_fine_pos;
	// Cauchy-Schwarz sum: preamble ≈ nsymb (4.0 for 4-sym), data ≈ 0.
	// Amplitude-independent: works at any RX gain level.
	result.correlation = best_fine_metric;
	return result;
}

int cl_ofdm::time_sync_mfsk(std::complex<double>* baseband_interp, int buffer_size_interp,
                            int interpolation_rate, int preamble_nSymb,
                            const int* preamble_tones, int mfsk_M,
                            int nStreams, const int* stream_offsets,
                            int search_start_symb, double* out_metric)
{
	// MFSK preamble time sync: correlate against known preamble tone sequence.
	// Multi-stream: each preamble symbol has one tone per stream band.
	// Score = sum of (target energy / total energy) across preamble symbols.
	// search_start_symb: skip positions before this to avoid re-finding old preambles.

	int Nofdm = Nfft + Ngi;
	int sym_period_interp = Nofdm * interpolation_rate;
	int buffer_nsymb = buffer_size_interp / sym_period_interp;

	std::complex<double>* decimated_sym = work_buf_a;
	std::complex<double>* fft_out = work_buf_b;

	// Map preamble tone indices to FFT bin indices for each stream
	int preamble_bins[8][4]; // [MAX_PREAMBLE_SYMB][MAX_STREAMS]
	int half = Nc / 2;
	for (int p = 0; p < preamble_nSymb; p++)
	{
		for (int st = 0; st < nStreams; st++)
		{
			int subcarrier = stream_offsets[st] + preamble_tones[p % preamble_nSymb];
			int bin;
			if (subcarrier < half)
				bin = Nfft - half + subcarrier;
			else
				bin = start_shift + (subcarrier - half);
			preamble_bins[p][st] = bin;
		}
	}

	double best_metric = -1;
	int best_sym_idx = 0;

	int s_start = (search_start_symb > 0) ? search_start_symb : 0;
	for (int s = s_start; s <= buffer_nsymb - preamble_nSymb; s++)
	{
		double metric = 0;

		for (int p = 0; p < preamble_nSymb; p++)
		{
			int sym_idx = s + p;
			int offset = sym_idx * sym_period_interp + Ngi * interpolation_rate;
			if (offset + Nfft * interpolation_rate > buffer_size_interp)
				break;

			// Decimate and FFT this symbol
			for (int i = 0; i < Nfft; i++)
			{
				decimated_sym[i] = baseband_interp[offset + i * interpolation_rate];
			}
			fft(decimated_sym, fft_out, Nfft);

			// Energy in expected preamble tone bins (all streams)
			double e_target = 0;
			for (int st = 0; st < nStreams; st++)
			{
				int bin = preamble_bins[p][st];
				e_target += fft_out[bin].real() * fft_out[bin].real() +
				            fft_out[bin].imag() * fft_out[bin].imag();
			}

			// Total energy across all Nc bins
			double e_total = 0;
			for (int k = 0; k < Nc; k++)
			{
				int bk;
				if (k < half)
					bk = Nfft - half + k;
				else
					bk = start_shift + (k - half);
				double e = fft_out[bk].real() * fft_out[bk].real() +
				           fft_out[bk].imag() * fft_out[bk].imag();
				e_total += e;
			}

			if (e_total > 0)
				metric += e_target / e_total;
		}

		if (metric > best_metric)
		{
			best_metric = metric;
			best_sym_idx = s;
		}
	}

	double threshold = (Nc <= 10) ? preamble_nSymb * 0.3 : preamble_nSymb * 0.5;  // NB: lower threshold (FIR leakage)

	if (out_metric) *out_metric = best_metric;

	if (g_verbose)
	{
		printf("[MFSK-SYNC] best_metric=%.3f threshold=%.1f best_sym=%d buffer_nsymb=%d preamble_nSymb=%d Nc=%d M=%d\n",
			best_metric, threshold, best_sym_idx, buffer_nsymb, preamble_nSymb, Nc, mfsk_M);

		// Show energy distribution at best position
		if (best_sym_idx >= 0 && best_metric > 0.01)
		{
			for (int p = 0; p < preamble_nSymb; p++)
			{
				int sym_idx = best_sym_idx + p;
				int offset = sym_idx * sym_period_interp + Ngi * interpolation_rate;
				if (offset + Nfft * interpolation_rate > buffer_size_interp)
					break;

				for (int i = 0; i < Nfft; i++)
					decimated_sym[i] = baseband_interp[offset + i * interpolation_rate];
				fft(decimated_sym, fft_out, Nfft);

				double e_target = 0, e_total = 0;
				int half = Nc / 2;
				for (int st = 0; st < nStreams; st++)
				{
					int bin = preamble_bins[p][st];
					e_target += fft_out[bin].real() * fft_out[bin].real() +
					            fft_out[bin].imag() * fft_out[bin].imag();
				}
				for (int k = 0; k < Nc; k++)
				{
					int bk = (k < half) ? (Nfft - half + k) : (start_shift + (k - half));
					e_total += fft_out[bk].real() * fft_out[bk].real() +
					           fft_out[bk].imag() * fft_out[bk].imag();
				}
				printf("  p%d: tone=%d bin=%d e_target=%.3e e_total=%.3e ratio=%.3f\n",
					p, preamble_tones[p], preamble_bins[p][0], e_target, e_total,
					e_total > 0 ? e_target / e_total : 0.0);
			}
		}
		fflush(stdout);
	}

	if (best_metric < threshold)
		return -1;  // No valid preamble found

	int delay = best_sym_idx * sym_period_interp;

	return delay;
}

// NB MFSK preamble detection via waveform cross-correlation.
// Correlates the pre-generated baseband preamble template against the received
// baseband buffer. With L=2176 samples (8 symbols × 272), noise metric ~0.0005
// vs signal ~1.0, giving ~2000:1 discrimination vs ~4:1 for FFT energy method.
//
// Per-symbol correlation: each MFSK symbol has a single tone, so per-symbol
// |corr|² is phase-invariant under timing offset. Summing per-symbol metrics
// avoids the destructive interference that occurs when correlating multi-tone
// templates coherently (different tones rotate at different rates).
//
// 2026-05-27 (data-preamble-port-research.md §14): body replaced with a
// discrete FFT-bin-argmax matcher mirroring `detect_ack_pattern`. The
// metric is now a COUNT of per-symbol matches (range 0..preamble_nSymb)
// — length-scaled by construction. The threshold `preamble_match_threshold`
// (mfsk.cc) gates detection.
//
// Function signature is preserved: returns full-rate sample offset on
// detect, -1 on no-detect. `*out_metric` now reports the matched count
// (not cosine²-mean). Diagnostic-only — no flow-control code reads it.
//
// Returns delay in interpolated samples, or -1 if no preamble found.
int cl_ofdm::time_sync_mfsk_corr(std::complex<double>* baseband_interp,
                                  int buffer_size_interp, int interpolation_rate,
                                  int search_start_symb, double* out_metric)
{
	if (out_metric) *out_metric = 0.0;

	if (mfsk_M <= 0 || mfsk_nStreams <= 0 || mfsk_preamble_nsymb <= 0)
		return -1;
	if (mfsk_preamble_match_threshold <= 0)
		return -1;
	if (work_buf_a == NULL || work_buf_b == NULL || Nfft <= 0)
		return -1;

	int Nofdm = Nfft + Ngi;
	int sym_period_interp = Nofdm * interpolation_rate;
	if (sym_period_interp <= 0) return -1;
	int buffer_nsymb = buffer_size_interp / sym_period_interp;
	int preamble_n = mfsk_preamble_nsymb;
	if (buffer_nsymb < preamble_n) return -1;

	int s_start = (search_start_symb > 0) ? search_start_symb : 0;
	if (s_start > buffer_nsymb - preamble_n) {
		// Nothing to search.
		return -1;
	}

	std::complex<double>* decimated_sym = work_buf_a;
	std::complex<double>* fft_out = work_buf_b;
	int half = Nc / 2;

	// Phase 1: coarse symbol-grid scan. For each candidate start s, count
	// the number of preamble symbols whose FFT-bin argmax (per stream)
	// matches the expected preamble tone bin. Track best (matched, metric).
	int best_pos = -1;
	int best_matched = -1;
	double best_metric = -1.0;

	for (int s = s_start; s <= buffer_nsymb - preamble_n; s++)
	{
		int matched = 0;
		double metric = 0.0;

		for (int p = 0; p < preamble_n; p++)
		{
			int sym_idx = s + p;
			int offset = sym_idx * sym_period_interp + Ngi * interpolation_rate;
			if (offset + Nfft * interpolation_rate > buffer_size_interp)
				break;

			// Decimate at stride `interpolation_rate` (full-rate input) and FFT.
			for (int i = 0; i < Nfft; i++)
				decimated_sym[i] = baseband_interp[offset + i * interpolation_rate];
			fft(decimated_sym, fft_out, Nfft);

			// Expected tone for this symbol. No hopping at emit time —
			// preamble_tones[] stores the full sequence directly
			// (mfsk.cc generate_preamble:467 reads preamble_tones[s % nsymb]).
			int actual_tone = mfsk_preamble_tones[p % 16];
			if (actual_tone < 0 || actual_tone >= mfsk_M) continue;

			// e_target: energy in the expected (+mirror) tone bins summed over all
			// streams — feeds the secondary tie-break metric only. Identical in
			// both the 1-stream and multi-stream decision paths below.
			double e_target = 0.0;
			for (int st = 0; st < mfsk_nStreams; st++)
			{
				int sub = mfsk_stream_offsets[st] + actual_tone;
				int expected_bin = (sub < half) ? (Nfft - half + sub)
				                                 : (start_shift + (sub - half));
				int mirror_bin = (Nfft - expected_bin) % Nfft;

				double ee = fft_out[expected_bin].real() * fft_out[expected_bin].real()
				          + fft_out[expected_bin].imag() * fft_out[expected_bin].imag();
				double em = fft_out[mirror_bin].real() * fft_out[mirror_bin].real()
				          + fft_out[mirror_bin].imag() * fft_out[mirror_bin].imag();
				e_target += ee + em;
			}

			// Per-symbol match decision.
			//
			// mfsk_nStreams >= 2: STREAM-ENERGY COMBINING (equal-gain noncoherent;
			// Proakis 5e §14.4, Q65 multi-tone energy sum / K1JT). The TX places the
			// SAME preamble tone in every stream's band (mfsk.cc generate_preamble),
			// so the streams are redundant copies. Summing each candidate tone's
			// energy ACROSS streams before a single argmax recovers the 2-branch
			// array gain (+4.86 dB measured, M16×2 cliff −9.03 → −13.89; P3
			// data-frame-detector-deepening-p3.md §3.2). This replaces the legacy
			// per-stream AND-gate (each stream argmax-matched independently → p²
			// per-symbol match prob, the cliff driver). Mirror handled in tone
			// space ((M−tone)%M) for carrier-image recovery, bit-equivalent to the
			// P3-measured scorer (p3_score_stream_combined).
			//
			// mfsk_nStreams == 1 (ROBUST_0 / M32×1): the combiner is a no-op
			// (one branch to sum), so we keep the legacy per-stream argmax +
			// bin-space-mirror accept path VERBATIM. The two formulations are NOT
			// bit-identical at 1 stream (tone- vs bin-space mirror differ by
			// −1.34 dB, P3 §3.1), so gating preserves ROBUST_0 byte-for-byte.
			bool symbol_matched;
			if (mfsk_nStreams >= 2)
			{
				// Combined per-tone energy across streams, then one argmax over M.
				double best_e = -1.0;
				int best_t = -1;
				for (int t = 0; t < mfsk_M; t++)
				{
					double e = 0.0;
					for (int st = 0; st < mfsk_nStreams; st++)
					{
						int tsub = mfsk_stream_offsets[st] + t;
						int b = (tsub < half) ? (Nfft - half + tsub)
						                       : (start_shift + (tsub - half));
						e += fft_out[b].real() * fft_out[b].real()
						   + fft_out[b].imag() * fft_out[b].imag();
					}
					if (e > best_e) { best_e = e; best_t = t; }
				}
				int mirror_tone = (mfsk_M - actual_tone) % mfsk_M;
				symbol_matched = (best_e > 0 &&
				                  (best_t == actual_tone || best_t == mirror_tone));
			}
			else
			{
				int sub = mfsk_stream_offsets[0] + actual_tone;
				int expected_bin = (sub < half) ? (Nfft - half + sub)
				                                 : (start_shift + (sub - half));
				int mirror_bin = (Nfft - expected_bin) % Nfft;
				// Find peak bin among the stream's M tones (argmax).
				double peak_e = -1.0;
				int peak_bin = -1;
				for (int t = 0; t < mfsk_M; t++)
				{
					int tsub = mfsk_stream_offsets[0] + t;
					int b = (tsub < half) ? (Nfft - half + tsub)
					                       : (start_shift + (tsub - half));
					double e = fft_out[b].real() * fft_out[b].real()
					         + fft_out[b].imag() * fft_out[b].imag();
					if (e > peak_e) { peak_e = e; peak_bin = b; }
				}
				// Carrier-image recovery (Bug #39 pattern in detect_ack_pattern):
				// accept expected OR mirror as peak. Energy gate prevents 0==0
				// match on silence.
				symbol_matched = (peak_e > 0 &&
				                  (peak_bin == expected_bin || peak_bin == mirror_bin));
			}

			if (!symbol_matched)
				continue;
			matched++;

			// Continuous secondary metric: energy in target bins / total energy
			// across the Nc occupied subcarriers. Diagnostic / tie-breaker only.
			double e_total = 0.0;
			for (int k = 0; k < Nc; k++)
			{
				int bk = (k < half) ? (Nfft - half + k)
				                    : (start_shift + (k - half));
				double e = fft_out[bk].real() * fft_out[bk].real()
				         + fft_out[bk].imag() * fft_out[bk].imag();
				e_total += e;
			}
			if (e_total > 0.0) metric += e_target / e_total;
		}

		// Pick winner by (matched DESC, metric DESC) — matched is the
		// length-scaled detection statistic.
		if (matched > best_matched ||
		    (matched == best_matched && metric > best_metric))
		{
			best_matched = matched;
			best_metric = metric;
			best_pos = s;
		}
	}

	if (best_pos < 0)
	{
		if (out_metric) *out_metric = 0.0;
		return -1;
	}

	// Phase 2: fine refinement at base-rate resolution within ±sym_period/2
	// of the coarse position. always_fine=true semantics — data symbols
	// downstream need sample-precise alignment, so we ALWAYS refine even
	// when coarse_matched < threshold (matches detect_ack_pattern's
	// always_fine=true branch).
	int coarse_offset = best_pos * sym_period_interp;
	int search_half = sym_period_interp / 2;
	int fine_best_matched = best_matched;
	double fine_best_metric = best_metric;
	int fine_best_offset = coarse_offset;

	for (int d = coarse_offset - search_half; d <= coarse_offset + search_half; d += interpolation_rate)
	{
		if (d < 0) continue;
		// Ensure all preamble symbols fit.
		if (d + preamble_n * sym_period_interp > buffer_size_interp) continue;

		int matched_f = 0;
		double metric_f = 0.0;
		bool oob = false;

		for (int p = 0; p < preamble_n && !oob; p++)
		{
			int offset = d + p * sym_period_interp + Ngi * interpolation_rate;
			if (offset + Nfft * interpolation_rate > buffer_size_interp) { oob = true; break; }

			for (int i = 0; i < Nfft; i++)
				decimated_sym[i] = baseband_interp[offset + i * interpolation_rate];
			fft(decimated_sym, fft_out, Nfft);

			int actual_tone = mfsk_preamble_tones[p % 16];
			if (actual_tone < 0 || actual_tone >= mfsk_M) continue;

			// e_targ: expected(+mirror) tone energy summed over streams — secondary
			// metric only. Identical in both decision paths (mirrors Phase-1).
			double e_targ = 0.0;
			for (int st = 0; st < mfsk_nStreams; st++)
			{
				int sub = mfsk_stream_offsets[st] + actual_tone;
				int ebin = (sub < half) ? (Nfft - half + sub)
				                        : (start_shift + (sub - half));
				int mbin = (Nfft - ebin) % Nfft;

				double ee = fft_out[ebin].real() * fft_out[ebin].real()
				          + fft_out[ebin].imag() * fft_out[ebin].imag();
				double em = fft_out[mbin].real() * fft_out[mbin].real()
				          + fft_out[mbin].imag() * fft_out[mbin].imag();
				e_targ += ee + em;
			}

			// Per-symbol match decision — SAME gated logic as the Phase-1 coarse
			// scan (see the long comment there). nStreams>=2: stream-energy
			// combining; nStreams==1: legacy per-stream argmax (byte-identical).
			bool symbol_ok;
			if (mfsk_nStreams >= 2)
			{
				double best_e = -1.0;
				int best_t = -1;
				for (int t = 0; t < mfsk_M; t++)
				{
					double e = 0.0;
					for (int st = 0; st < mfsk_nStreams; st++)
					{
						int tsub = mfsk_stream_offsets[st] + t;
						int b = (tsub < half) ? (Nfft - half + tsub)
						                       : (start_shift + (tsub - half));
						e += fft_out[b].real() * fft_out[b].real()
						   + fft_out[b].imag() * fft_out[b].imag();
					}
					if (e > best_e) { best_e = e; best_t = t; }
				}
				int mirror_tone = (mfsk_M - actual_tone) % mfsk_M;
				symbol_ok = (best_e > 0 &&
				             (best_t == actual_tone || best_t == mirror_tone));
			}
			else
			{
				int sub = mfsk_stream_offsets[0] + actual_tone;
				int ebin = (sub < half) ? (Nfft - half + sub)
				                        : (start_shift + (sub - half));
				int mbin = (Nfft - ebin) % Nfft;
				double pk = -1.0;
				int pkbin = -1;
				for (int t = 0; t < mfsk_M; t++)
				{
					int tsub = mfsk_stream_offsets[0] + t;
					int b = (tsub < half) ? (Nfft - half + tsub)
					                       : (start_shift + (tsub - half));
					double e = fft_out[b].real() * fft_out[b].real()
					         + fft_out[b].imag() * fft_out[b].imag();
					if (e > pk) { pk = e; pkbin = b; }
				}
				symbol_ok = (pk > 0 && (pkbin == ebin || pkbin == mbin));
			}

			if (!symbol_ok) continue;
			matched_f++;

			double e_tot = 0.0;
			for (int k = 0; k < Nc; k++)
			{
				int bk = (k < half) ? (Nfft - half + k)
				                    : (start_shift + (k - half));
				double e = fft_out[bk].real() * fft_out[bk].real()
				         + fft_out[bk].imag() * fft_out[bk].imag();
				e_tot += e;
			}
			if (e_tot > 0.0) metric_f += e_targ / e_tot;
		}

		if (oob) continue;
		if (matched_f > fine_best_matched ||
		    (matched_f == fine_best_matched && metric_f > fine_best_metric))
		{
			fine_best_matched = matched_f;
			fine_best_metric = metric_f;
			fine_best_offset = d;
		}
	}

	// Detection gate: length-scaled discrete match count.
	//
	// §13 FAR cleanup (data-frame-detector-deepening-p3.md §13): the
	// detect/no-detect DECISION is gated on the COARSE matched count, and the
	// Phase-2 fine pass is used ONLY to refine the returned sample OFFSET — it
	// does NOT re-maximize the count to RE-DECIDE detection. Rationale: the
	// fine pass takes the MAX matched count over the ±½-symbol sub-positions,
	// which lifts BOTH the signal AND the pure-noise matched-count distribution
	// (a max over correlated positions). On the M16×2 combiner path that
	// fine-pass max INFLATES FAR ~10× (T=8 1.8e-2 vs the coarse-only 1.75e-3,
	// §12) without buying genuine detection — the real acquisition gain is
	// carried by the COARSE stream-energy combining (the +4.86 dB / ~2.4×
	// matched-count lift, HW-validated §16), not by the sub-position max.
	// Deciding on the coarse count therefore recovers the ref-scorer FAR
	// (1.75e-3 @ T=8) while preserving the combiner's coarse acquisition gain.
	//
	// GATED behind nStreams>=2 (the combiner geometry):
	//  - nStreams>=2 (M16×2/M4×2 — ROBUST_1/2, future ROBUST_RA/ULTRA): decide
	//    on best_matched (coarse); report it as the decision statistic. The fine
	//    pass still ran and fine_best_offset is its alignment-refined position,
	//    returned on a positive detection (offset refinement only).
	//  - nStreams==1 (M32×1/M8×1 — ROBUST_0): UNCHANGED — gate on
	//    fine_best_matched and report it, byte-identical to monitor @8fc1211
	//    (the §11 combiner gate already preserves the 1-stream decision path;
	//    this keeps the SAME gate statistic there too).
	int decision_matched = (mfsk_nStreams >= 2) ? best_matched : fine_best_matched;

	if (decision_matched < mfsk_preamble_match_threshold)
	{
		if (out_metric) *out_metric = (double)decision_matched;
		return -1;
	}

	if (out_metric) *out_metric = (double)decision_matched;
	return fine_best_offset;
}

// ACK pattern detection: slide window across buffer, accumulate E_target/E_total
// Returns best metric (0.0 = noise, up to ack_nsymb = perfect match)
double cl_ofdm::detect_ack_pattern(std::complex<double>* baseband_interp, int buffer_size_interp,
                                   int interpolation_rate, int ack_nsymb,
                                   const int* ack_tones, int ack_pattern_len,
                                   int tone_hop_step, int mfsk_M,
                                   int nStreams, const int* stream_offsets,
                                   int* out_matched,
                                   int suffix_start, int* out_suffix_matched,
                                   int* out_best_offset, int reserve_after,
                                   uint32_t* out_match_mask,
                                   bool always_fine, int combine_reps)
{
	int Nofdm = Nfft + Ngi;
	int sym_period_interp = Nofdm * interpolation_rate;
	int buffer_nsymb = buffer_size_interp / sym_period_interp;

	// §20: noncoherent base-pattern combining. The base block is ack_nsymb
	// symbols repeated `combine_reps` times on the wire; the matched filter sums
	// the per-symbol FFT power across the R aligned reps before argmax/count.
	// reps=1 is the byte-identical single-block path. The window must hold all R
	// reps + reserve_after.
	if (combine_reps < 1) combine_reps = 1;
	int rep_stride_sym = ack_nsymb;                 // one base block, in symbols
	int total_needed = combine_reps * ack_nsymb + reserve_after;
	if (buffer_nsymb < total_needed) return 0.0;

	std::complex<double>* decimated_sym = work_buf_a;
	std::complex<double>* fft_out = work_buf_b;

	// Per-bin power accumulator (summed over reps). Only used when combining;
	// for reps=1 the per-symbol path reads fft_out directly (no extra heap on the
	// hot ACK/HAIL poll). Sized Nfft, allocated once per call (poll cadence, not
	// audio-rate).
	std::vector<double> pow_accum;
	if (combine_reps > 1) pow_accum.assign((size_t)Nfft, 0.0);

	// Fill `pow` (size Nfft) with Σ_rep |FFT(symbol p of rep r)|² for the symbol
	// whose REP-0 interpolated sample offset is `base_offset`. Returns false if
	// any rep runs off the buffer (caller skips the symbol / candidate). When
	// combine_reps==1 this is one FFT (identical math to the legacy inline path).
	auto accumulate_sym_power = [&](int base_offset, double* pow) -> bool {
		for (int b = 0; b < Nfft; b++) pow[b] = 0.0;
		for (int r = 0; r < combine_reps; r++)
		{
			int off = base_offset + r * rep_stride_sym * sym_period_interp;
			if (off < 0 || off + Nfft * interpolation_rate > buffer_size_interp)
				return false;
			for (int i = 0; i < Nfft; i++)
				decimated_sym[i] = baseband_interp[off + i * interpolation_rate];
			fft(decimated_sym, fft_out, Nfft);
			for (int b = 0; b < Nfft; b++)
				pow[b] += fft_out[b].real() * fft_out[b].real() +
				          fft_out[b].imag() * fft_out[b].imag();
		}
		return true;
	};

	int half = Nc / 2;
	double best_metric = 0.0;
	int best_pos = -1;
	int best_matched = 0;
	int best_suffix_matched = 0;
	uint32_t best_match_mask = 0;

	for (int s = 0; s <= buffer_nsymb - total_needed; s++)
	{
		double metric = 0;
		int matched = 0;
		int suffix_matched = 0;
		uint32_t match_mask = 0;

		for (int p = 0; p < ack_nsymb; p++)
		{
			int sym_idx = s + p;
			int offset = sym_idx * sym_period_interp + Ngi * interpolation_rate;
			if (offset + Nfft * interpolation_rate > buffer_size_interp)
				break;

			// §20: per-bin power for symbol p. reps>1 sums |FFT|² across the R
			// aligned base reps (noncoherent integration); reps==1 reads the single
			// FFT directly (byte-identical to the legacy path). `psp(b)` returns the
			// combined power at bin b.
			if (combine_reps > 1)
			{
				if (!accumulate_sym_power(offset, pow_accum.data()))
					break;   // a rep ran off the buffer
			}
			else
			{
				for (int i = 0; i < Nfft; i++)
					decimated_sym[i] = baseband_interp[offset + i * interpolation_rate];
				fft(decimated_sym, fft_out, Nfft);
			}
			auto psp = [&](int b) -> double {
				if (combine_reps > 1) return pow_accum[b];
				return fft_out[b].real() * fft_out[b].real() +
				       fft_out[b].imag() * fft_out[b].imag();
			};

			// Which tone is expected at symbol p?
			int tone_base = ack_tones[p % ack_pattern_len];
			int actual_tone = (tone_base + p * tone_hop_step) % mfsk_M;

			// Order-aware detection: count this symbol only if the expected ACK tone
			// is the peak bin for ALL streams. Both streams transmit the same ACK
			// tone, so both should peak at the same bin. Using "any" causes high
			// false alarm rate for multi-stream modes (P(any)=1-(1-1/M)^nS ≈ 44%
			// for M=4, nS=2; P(all)=(1/M)^nS ≈ 6%).
			int streams_matched = 0;
			double e_target = 0;
			for (int st = 0; st < nStreams; st++)
			{
				int expected_subcarrier = stream_offsets[st] + actual_tone;
				int expected_bin;
				if (expected_subcarrier < half)
					expected_bin = Nfft - half + expected_subcarrier;
				else
					expected_bin = start_shift + (expected_subcarrier - half);
				double e_expected = psp(expected_bin);

				// Carrier image recovery (Bug #39): real passband → baseband
				// creates equal-energy mirrors at (Nfft - bin) % Nfft. For NB
				// (M=8, Nc=10), mirrors fall WITHIN the stream's M bins — the
				// FIR can't reject in-band images. Without recovery, the mirror
				// competes with the expected bin for "peak" status, giving ~50%
				// match rate. Fix: accept expected OR mirror as the peak bin.
				// Metric uses max(expected, mirror) to avoid inflating noise.
				int mirror_bin = (Nfft - expected_bin) % Nfft;
				double e_mirror = psp(mirror_bin);
				e_target += e_expected + e_mirror;

				// Find peak bin (individual, not combined) among stream's M bins
				double peak_e = -1.0;
				int peak_bin = -1;
				for (int t = 0; t < mfsk_M; t++)
				{
					int sub = stream_offsets[st] + t;
					int b;
					if (sub < half)
						b = Nfft - half + sub;
					else
						b = start_shift + (sub - half);
					double e = psp(b);
					if (e > peak_e)
					{
						peak_e = e;
						peak_bin = b;
					}
				}
				// Energy gate + carrier image: accept expected OR mirror as peak.
				// In silence (zeroed buffer), all bins have e=0 — energy gate
				// prevents 0>=0 false match.
				if (peak_e > 0 && (peak_bin == expected_bin || peak_bin == mirror_bin))
					streams_matched++;
			}

			if (streams_matched < nStreams)
				continue;

			matched++;
			if (p < 32) match_mask |= (1u << p);
			if (suffix_start > 0 && p >= suffix_start)
				suffix_matched++;

			// Total energy across all Nc bins
			double e_total = 0;
			for (int k = 0; k < Nc; k++)
			{
				int bk;
				if (k < half)
					bk = Nfft - half + k;
				else
					bk = start_shift + (k - half);
				e_total += psp(bk);
			}

			if (e_total > 0)
				metric += e_target / e_total;
		}

		if (metric > best_metric)
		{
			best_metric = metric;
			best_pos = s;
			best_matched = matched;
			best_suffix_matched = suffix_matched;
			best_match_mask = match_mask;
		}
	}

	// Phase 2: Fine timing refinement (Bug #39).
	// The coarse search at symbol-period steps can be off by up to ±Nofdm/2
	// base-rate samples from the true ACK start. For NB (Ngi=16), the max
	// error of ±136 samples far exceeds the GI tolerance → ICI → low metric.
	// Search at base-rate (IR-step) resolution within ±Nofdm/2 of the coarse
	// position. Only runs when Phase 1 found a decent candidate.
	//
	// always_fine: callers that fire one-shot (e.g., BREAK detector) need the
	// fine pass even when coarse matched < 6, because the asynchronous arrival
	// phase often lands mid-symbol → FFT windows straddle two transmitted
	// symbols → coarse_matched collapses → BREAK never refines and never
	// fires. Polled callers (ACK) don't need this because they slide the
	// tail-snapshot every 2-3 ms and eventually hit alignment.
	if ((always_fine || best_matched >= 6) && best_pos >= 0)
	{
		int coarse_offset = best_pos * sym_period_interp;
		int search_half = sym_period_interp / 2;
		double fine_best_metric = -1.0;
		int fine_best_matched = 0;
		int fine_best_suffix = 0;
		int fine_best_offset = coarse_offset;

		uint32_t fine_best_mask = 0;
		for (int d = coarse_offset - search_half; d <= coarse_offset + search_half;
		     d += interpolation_rate)
		{
			if (d < 0) continue;
			// Ensure suffix (reserve_after) fits within buffer
			if (reserve_after > 0 && d + total_needed * sym_period_interp > buffer_size_interp)
				continue;

			double metric_f = 0;
			int matched_f = 0;
			int suffix_f = 0;
			uint32_t mask_f = 0;
			bool oob = false;

			for (int p = 0; p < ack_nsymb && !oob; p++)
			{
				int offset = d + p * sym_period_interp + Ngi * interpolation_rate;
				if (offset + Nfft * interpolation_rate > buffer_size_interp)
				{
					oob = true;
					break;
				}

				// §20: same per-bin combining as Phase 1 (psp reads combined power).
				if (combine_reps > 1)
				{
					if (!accumulate_sym_power(offset, pow_accum.data()))
					{ oob = true; break; }
				}
				else
				{
					for (int i = 0; i < Nfft; i++)
						decimated_sym[i] = baseband_interp[offset + i * interpolation_rate];
					fft(decimated_sym, fft_out, Nfft);
				}
				auto psp = [&](int b) -> double {
					if (combine_reps > 1) return pow_accum[b];
					return fft_out[b].real() * fft_out[b].real() +
					       fft_out[b].imag() * fft_out[b].imag();
				};

				int tone_base = ack_tones[p % ack_pattern_len];
				int actual_tone = (tone_base + p * tone_hop_step) % mfsk_M;

				int streams_ok = 0;
				double e_targ = 0;
				for (int st = 0; st < nStreams; st++)
				{
					int esub = stream_offsets[st] + actual_tone;
					int ebin = (esub < half) ? Nfft - half + esub
					                         : start_shift + (esub - half);
					double ee = psp(ebin);
					int mbin = (Nfft - ebin) % Nfft;
					double em = psp(mbin);
					e_targ += ee + em;

					double pk = -1.0;
					int pkbin = -1;
					for (int t = 0; t < mfsk_M; t++)
					{
						int sub = stream_offsets[st] + t;
						int b = (sub < half) ? Nfft - half + sub
						                     : start_shift + (sub - half);
						double e = psp(b);
						if (e > pk) { pk = e; pkbin = b; }
					}
					if (pk > 0 && (pkbin == ebin || pkbin == mbin))
						streams_ok++;
				}

				if (streams_ok < nStreams) continue;
				matched_f++;
				if (p < 32) mask_f |= (1u << p);
				if (suffix_start > 0 && p >= suffix_start)
					suffix_f++;

				double e_tot = 0;
				for (int k = 0; k < Nc; k++)
				{
					int bk = (k < half) ? Nfft - half + k
					                    : start_shift + (k - half);
					e_tot += psp(bk);
				}
				if (e_tot > 0)
					metric_f += e_targ / e_tot;
			}

			if (oob) continue;
			if (matched_f > fine_best_matched ||
			    (matched_f == fine_best_matched && metric_f > fine_best_metric))
			{
				fine_best_matched = matched_f;
				fine_best_suffix = suffix_f;
				fine_best_metric = metric_f;
				fine_best_offset = d;
				fine_best_mask = mask_f;
			}
		}

		// Use fine result if better than coarse
		if (fine_best_matched > best_matched ||
		    (fine_best_matched == best_matched && fine_best_metric > best_metric))
		{
			best_matched = fine_best_matched;
			best_suffix_matched = fine_best_suffix;
			best_metric = fine_best_metric;
			best_pos = fine_best_offset;  // keep as interpolated sample offset
			best_match_mask = fine_best_mask;
		}
		else
		{
			best_pos = best_pos * sym_period_interp;  // convert coarse to sample offset
		}
	}
	else if (best_pos >= 0)
	{
		best_pos = best_pos * sym_period_interp;  // no fine search: convert coarse
	}

	if (out_matched)
		*out_matched = best_matched;
	if (out_suffix_matched)
		*out_suffix_matched = best_suffix_matched;
	if (out_best_offset)
		*out_best_offset = best_pos;  // interpolated sample offset (-1 if not found)
	if (out_match_mask)
		*out_match_mask = best_match_mask;

	return best_metric;
}

// Decode suffix tones after a detected ACK pattern.
// pattern_offset: interpolated sample offset of the detected pattern start.
// pattern_nsymb: number of symbols in the detected pattern (e.g. 16 for WB ACK).
// suffix_len: number of suffix symbols to decode.
// out_tones: output array of decoded tone indices (size >= suffix_len).
void cl_ofdm::decode_suffix_tones(std::complex<double>* baseband_interp, int buffer_size_interp,
	int interpolation_rate, int pattern_offset, int pattern_nsymb,
	int suffix_len, int tone_hop_step, int mfsk_M,
	int nStreams, const int* stream_offsets, int* out_tones)
{
	int Nofdm_local = Nfft + Ngi;
	int sym_period_interp = Nofdm_local * interpolation_rate;
	int half = Nc / 2;

	std::complex<double>* decimated_sym = work_buf_a;
	std::complex<double>* fft_out = work_buf_b;

	for (int s = 0; s < suffix_len; s++)
	{
		out_tones[s] = -1;  // default: undecoded

		int abs_s = pattern_nsymb + s;  // absolute symbol index (for tone hopping)
		int offset = pattern_offset + abs_s * sym_period_interp + Ngi * interpolation_rate;
		if (offset + Nfft * interpolation_rate > buffer_size_interp)
			continue;

		// Decimate and FFT
		for (int i = 0; i < Nfft; i++)
			decimated_sym[i] = baseband_interp[offset + i * interpolation_rate];
		fft(decimated_sym, fft_out, Nfft);

		// Find peak energy bin across all streams (primary bin only — no carrier
		// image recovery, because mirrored bins make tone pairs indistinguishable:
		// e.g. NB tone 0 (bin 252) and tone 7 (bin 4) share mirror bins)
		double best_energy = -1.0;
		int best_tone = 0;

		for (int t = 0; t < mfsk_M; t++)
		{
			double e_combined = 0;
			for (int st = 0; st < nStreams; st++)
			{
				int sub = stream_offsets[st] + t;
				int b = (sub < half) ? Nfft - half + sub
				                     : start_shift + (sub - half);
				double e = fft_out[b].real() * fft_out[b].real() +
				           fft_out[b].imag() * fft_out[b].imag();
				e_combined += e;
			}
			if (e_combined > best_energy)
			{
				best_energy = e_combined;
				best_tone = t;
			}
		}


		// Reverse tone hopping: actual_tone = (data_tone + abs_s * hop) % M
		// So data_tone = (actual_tone - abs_s * hop) % M
		int hop = (abs_s * tone_hop_step) % mfsk_M;
		int data_tone = (best_tone - hop + mfsk_M * 256) % mfsk_M;
		out_tones[s] = data_tone;

		// Diagnostic: show top-3 tones and their energies (gated on g_verbose —
		// fires on every suffix decode otherwise; spams production logs + the
		// ctrl-suffix cliff-sweep test which runs thousands of decodes).
		if(g_verbose && suffix_len > 0 && s < 3)
		{
			double energies[32];
			for(int t2 = 0; t2 < mfsk_M && t2 < 32; t2++)
			{
				double ec = 0;
				for(int st2 = 0; st2 < nStreams; st2++)
				{
					int sub2 = stream_offsets[st2] + t2;
					int b2 = (sub2 < half) ? Nfft - half + sub2
					                       : start_shift + (sub2 - half);
					double e2 = fft_out[b2].real()*fft_out[b2].real() + fft_out[b2].imag()*fft_out[b2].imag();
					ec += e2;
				}
				energies[t2] = ec;
			}
			printf("[SUFFIX-FFT] s=%d abs_s=%d offset=%d best_phys=%d(e=%.2f) hop=%d data=%d",
				s, abs_s, offset, best_tone, best_energy, hop, data_tone);
			// Print all tone energies for comparison
			printf(" energies:");
			for(int t2 = 0; t2 < mfsk_M && t2 < 16; t2++)
				printf(" %d:%.1f", t2, energies[t2]);
			printf("\n"); fflush(stdout);
		}
	}
}

// Soft variant of decode_suffix_tones: instead of keeping only the per-symbol
// argmax, return the top-K de-hopped candidate tones ranked by energy plus a
// per-candidate soft cost. Used by the CRC-aided soft list decoder
// (connect-suffix-fec-research.md §3 Tier 1) — a zero-airtime upgrade that
// recovers suffix decodes where the correct tone landed in 2nd/3rd place.
//
// For noncoherent M-FSK the natural soft metric is the per-tone energy
// (Proakis Ch.8; see fact-doc §1.1). The cost we emit is the NORMALIZED energy
// gap to the strongest tone:  cost_k = (E_best - E_k) / (E_best + eps), so the
// argmax candidate always has cost 0 and weaker tones have cost in (0,1]. This
// is monotone in the per-symbol log-likelihood gap and needs no tuned constant.
//
// out_cand[s*K + k] = k-th most-likely de-hopped tone for symbol s (0..M-1),
//                     or -1 if the symbol ran past the buffer end / k>=valid.
// out_cost[s*K + k] = corresponding soft cost (>=0); +INF for invalid slots.
// The hard decode is exactly out_cand[s*K + 0] (bit-identical to
// decode_suffix_tones), so callers can fall back to baseline trivially.
void cl_ofdm::decode_suffix_candidates(std::complex<double>* baseband_interp,
	int buffer_size_interp, int interpolation_rate, int pattern_offset,
	int pattern_nsymb, int suffix_len, int tone_hop_step, int mfsk_M,
	int nStreams, const int* stream_offsets, int K, int* out_cand,
	double* out_cost)
{
	int Nofdm_local = Nfft + Ngi;
	int sym_period_interp = Nofdm_local * interpolation_rate;
	int half = Nc / 2;
	if (K < 1) K = 1;
	if (K > mfsk_M) K = mfsk_M;

	std::complex<double>* decimated_sym = work_buf_a;
	std::complex<double>* fft_out = work_buf_b;

	for (int s = 0; s < suffix_len; s++)
	{
		for (int k = 0; k < K; k++) {
			out_cand[s * K + k] = -1;
			out_cost[s * K + k] = 1.0e300;  // sentinel: invalid
		}

		int abs_s = pattern_nsymb + s;
		int offset = pattern_offset + abs_s * sym_period_interp + Ngi * interpolation_rate;
		if (offset + Nfft * interpolation_rate > buffer_size_interp)
			continue;  // symbol past buffer end -> all slots stay invalid

		for (int i = 0; i < Nfft; i++)
			decimated_sym[i] = baseband_interp[offset + i * interpolation_rate];
		fft(decimated_sym, fft_out, Nfft);

		// Combined per-tone energy across all streams (same bin math as the
		// hard path so the argmax candidate is identical).
		double e_tone[64];  // mfsk_M <= 64 (M=16 in production WB ctrl path)
		double best_energy = -1.0;
		for (int t = 0; t < mfsk_M; t++)
		{
			double e_combined = 0;
			for (int st = 0; st < nStreams; st++)
			{
				int sub = stream_offsets[st] + t;
				int b = (sub < half) ? Nfft - half + sub
				                     : start_shift + (sub - half);
				double e = fft_out[b].real() * fft_out[b].real() +
				           fft_out[b].imag() * fft_out[b].imag();
				e_combined += e;
			}
			e_tone[t] = e_combined;
			if (e_combined > best_energy) best_energy = e_combined;
		}

		// Partial selection: pick the K strongest tones (M is tiny, K<=M<=16,
		// so an O(K*M) selection is cheaper and simpler than a heap).
		bool taken[64] = {};
		int hop = (abs_s * tone_hop_step) % mfsk_M;
		double inv = 1.0 / (best_energy + 1.0e-12);
		for (int k = 0; k < K; k++)
		{
			int best_t = -1;
			double best_e = -1.0;
			for (int t = 0; t < mfsk_M; t++)
			{
				if (taken[t]) continue;
				if (e_tone[t] > best_e) { best_e = e_tone[t]; best_t = t; }
			}
			if (best_t < 0) break;
			taken[best_t] = true;
			// Reverse the same tone hopping the hard path uses.
			int data_tone = (best_t - hop + mfsk_M * 256) % mfsk_M;
			out_cand[s * K + k] = data_tone;
			out_cost[s * K + k] = (best_energy - best_e) * inv;  // 0 for k=0
		}
	}
}

// Full per-tone energy matrix for the soft GF(16) RA decoder
// (tier2-suffix-fec-gf16-spike.md). Same FFT + stream-combine + de-hop math as
// decode_suffix_candidates, but writes ALL mfsk_M de-hopped per-tone energies
// per symbol into out_energies[s*mfsk_M + data_tone]. Symbols past the buffer
// end leave their M energy slots at 0 (uniform -> uninformative intrinsic).
void cl_ofdm::decode_suffix_energies(std::complex<double>* baseband_interp,
	int buffer_size_interp, int interpolation_rate, int pattern_offset,
	int pattern_nsymb, int suffix_len, int tone_hop_step, int mfsk_M,
	int nStreams, const int* stream_offsets, double* out_energies)
{
	int Nofdm_local = Nfft + Ngi;
	int sym_period_interp = Nofdm_local * interpolation_rate;
	int half = Nc / 2;

	std::complex<double>* decimated_sym = work_buf_a;
	std::complex<double>* fft_out = work_buf_b;

	for (int s = 0; s < suffix_len; s++)
	{
		for (int t = 0; t < mfsk_M; t++) out_energies[s * mfsk_M + t] = 0.0;

		int abs_s = pattern_nsymb + s;
		int offset = pattern_offset + abs_s * sym_period_interp + Ngi * interpolation_rate;
		if (offset + Nfft * interpolation_rate > buffer_size_interp)
			continue;  // symbol past buffer end -> all-zero (uniform) energies

		for (int i = 0; i < Nfft; i++)
			decimated_sym[i] = baseband_interp[offset + i * interpolation_rate];
		fft(decimated_sym, fft_out, Nfft);

		int hop = (abs_s * tone_hop_step) % mfsk_M;
		for (int t = 0; t < mfsk_M; t++)
		{
			double e_combined = 0;
			for (int st = 0; st < nStreams; st++)
			{
				int sub = stream_offsets[st] + t;
				int b = (sub < half) ? Nfft - half + sub
				                     : start_shift + (sub - half);
				double e = fft_out[b].real() * fft_out[b].real() +
				           fft_out[b].imag() * fft_out[b].imag();
				e_combined += e;
			}
			// de-hop: received bin t corresponds to data tone (t - hop) mod M.
			int data_tone = (t - hop + mfsk_M * 256) % mfsk_M;
			out_energies[s * mfsk_M + data_tone] = e_combined;
		}
	}
}

int cl_ofdm::symbol_sync(std::complex <double>*in, int size, int interpolation_rate, int location_to_return)
{

	double corss_corr=0;
	double norm_a=0;
	double norm_b=0;

	int *corss_corr_loc=new int[Nsymb];
	double *corss_corr_vals=new double[Nsymb];
	int return_val;

	std::complex <double> *a_c, *b_c, a, b;

	for(int i=0;i<Nsymb;i++)
	{
		corss_corr_loc[i]=-1;
		corss_corr_vals[i]=0;
	}

	int L_interp = (this->Nfft / preamble_configurator.nIdentical_sections) * interpolation_rate;
	for(int i=0;i<Nsymb;i++)
	{
		a_c=in+i*(Nfft+Ngi)*interpolation_rate;
		b_c=in+i*(Nfft+Ngi)*interpolation_rate+L_interp;
		corss_corr=0;
		norm_a=0;
		norm_b=0;
		for(int m=0;m<L_interp;m++)
		{
			corss_corr+=a_c[m].real()*b_c[m].real();
			norm_a+=a_c[m].real()*a_c[m].real();
			norm_b+=b_c[m].real()*b_c[m].real();

			corss_corr+=a_c[m].imag()*b_c[m].imag();
			norm_a+=a_c[m].imag()*a_c[m].imag();
			norm_b+=b_c[m].imag()*b_c[m].imag();
		}
		corss_corr=corss_corr/sqrt(norm_a*norm_b);

		if(corss_corr<0)
		{
			corss_corr_vals[i]=-corss_corr;
		}
		else
		{
			corss_corr_vals[i]=corss_corr;
		}
		corss_corr_loc[i]=i;

	}
	double tmp;
	int tmp_int;
	for(int i=0;i<Nsymb-1;i++)
	{
		for(int j=0;j<Nsymb-1;j++)
		{
			if (corss_corr_vals[j]<corss_corr_vals[j+1])
			{
				tmp=corss_corr_vals[j];
				corss_corr_vals[j]=corss_corr_vals[j+1];
				corss_corr_vals[j+1]=tmp;

				tmp_int=corss_corr_loc[j];
				corss_corr_loc[j]=corss_corr_loc[j+1];
				corss_corr_loc[j+1]=tmp_int;
			}
		}
	}
	return_val=corss_corr_loc[location_to_return];
	if(corss_corr_loc!=NULL)
	{
		delete[] corss_corr_loc;
	}
	if(corss_corr_vals!=NULL)
	{
		delete[] corss_corr_vals;
	}
	return return_val;
}

void cl_ofdm::rational_resampler(std::complex <double>* in, int in_size, std::complex <double>* out, int rate, int interpolation_decimation)
{
	if (interpolation_decimation==DECIMATION)
	{
		int index=0;
		for(int i=0;i<in_size;i+=rate)
		{
			*(out+index)=*(in+i);
			index++;
		}
	}
	else if (interpolation_decimation==INTERPOLATION)
	{
		for(int i=0;i<in_size-1;i++)
		{
			for(int j=0;j<rate;j++)
			{
				*(out+i*rate+j)=interpolate_linear(*(in+i),0,*(in+i+1),rate,j);
			}
		}
		for(int j=0;j<rate;j++)
		{
			*(out+(in_size-1)*rate+j)=interpolate_linear(*(in+in_size-2),0,*(in+in_size-1),rate,rate+j);
		}
	}
}

void cl_ofdm::baseband_to_passband(std::complex <double>* in, int in_size, double* out, double sampling_frequency, double carrier_frequency, double carrier_amplitude,int interpolation_rate)
{
	double sampling_interval=1.0/sampling_frequency;

	// Grow-as-needed interpolation buffer
	int needed = in_size * interpolation_rate;
	if(needed > b2p_buffer_size)
	{
		if(b2p_data_interpolated!=NULL) delete[] b2p_data_interpolated;
		b2p_data_interpolated = new std::complex<double>[needed];
		b2p_buffer_size = needed;
	}
	std::complex <double> *data_interpolated = b2p_data_interpolated;

	rational_resampler( in, in_size, data_interpolated, interpolation_rate, INTERPOLATION);
	for(int i=0;i<in_size*interpolation_rate;i++)
	{
		out[i]=data_interpolated[i].real()*carrier_amplitude*cos(2*M_PI*carrier_frequency*(double)passband_start_sample * sampling_interval);
		out[i]+=data_interpolated[i].imag()*carrier_amplitude*sin(2*M_PI*carrier_frequency*(double)passband_start_sample * sampling_interval);
		passband_start_sample++;
	}
}
void cl_ofdm::passband_to_baseband(double* in, int in_size, std::complex <double>* out, double sampling_frequency, double carrier_frequency, double carrier_amplitude, int decimation_rate, cl_FIR* filter, int sample_offset)
{
	double sampling_interval=1.0/sampling_frequency;

	// Reuse pre-allocated buffers (reallocate only if size changed)
	if(p2b_buffer_size < in_size)
	{
		delete[] p2b_l_data;
		delete[] p2b_data_filtered;
		p2b_l_data = new std::complex<double>[in_size];
		p2b_data_filtered = new std::complex<double>[in_size];
		p2b_buffer_size = in_size;
	}

	// Phase recurrence: replace per-sample sincos with one complex multiply.
	// Pre-Pi profile showed __sincos at 10.1% CPU; this loop is the dominant
	// caller. Drift over a single in_size call (≤~325k samples) is well below
	// floating-point round-off significance for the downstream FIR.
	double angle_step = 2.0 * M_PI * carrier_frequency * sampling_interval;
	double angle_start = angle_step * (double)sample_offset;
	double pr = std::cos(angle_start);
	double pi = std::sin(angle_start);
	double sr = std::cos(angle_step);
	double si = std::sin(angle_step);
	for(int i=0;i<in_size;i++)
	{
		double a = in[i] * carrier_amplitude;
		p2b_l_data[i].real(a * pr);
		p2b_l_data[i].imag(a * pi);
		// Rotate phasor: (pr + j*pi) *= (sr + j*si)
		double npr = pr * sr - pi * si;
		double npi = pr * si + pi * sr;
		pr = npr;
		pi = npi;
	}

	// Note: callers currently pass decimation_rate=1 here, so this loop is
	// pure FIR (no decimation). Actual decimation by interpolation_rate is
	// done by a separate rational_resampler() call downstream. To unlock
	// the polyphase ~8× FIR win, callers would need to be refactored to
	// pass decimation_rate=interpolation_rate and route through
	// `filter->apply_decimate(...)`. That refactor touches many call sites
	// (telecom_system.cc and time_sync paths) and is deferred — the
	// apply_decimate function is in place and bit-exact-tested, ready for
	// use when the surrounding refactor happens. See PI_CPU_OPTIMIZATION_REPORT.
	filter->apply(p2b_l_data, p2b_data_filtered, in_size);
	rational_resampler(p2b_data_filtered, in_size, out, decimation_rate, DECIMATION);
}

// Combined mix + polyphase FIR + decimate. Writes in_size/M complex samples
// directly to `out` at the decimated rate.
//
// Used by the detector hot paths (detect_*_pattern_from_passband) that
// previously did mix -> FIR-at-high-rate -> downstream picks every Mth
// sample. The FIR portion of those paths was 92% of CPU on Pi RX side;
// the polyphase identity drops it ~M× because only the kept outputs are
// computed. Bit-exact equivalent to passband_to_baseband(... rate=1 ...)
// followed by picking every Mth sample.
void cl_ofdm::passband_to_baseband_decimated(double* in, int in_size,
	std::complex<double>* out, double sampling_frequency, double carrier_frequency,
	double carrier_amplitude, int M, cl_FIR* filter, int sample_offset)
{
	double sampling_interval = 1.0 / sampling_frequency;

	if (p2b_buffer_size < in_size)
	{
		if (p2b_l_data != NULL) delete[] p2b_l_data;
		if (p2b_data_filtered != NULL) delete[] p2b_data_filtered;
		p2b_l_data = new std::complex<double>[in_size];
		p2b_data_filtered = new std::complex<double>[in_size];
		p2b_buffer_size = in_size;
	}

	// Mix to baseband via phase recurrence (no per-sample sincos).
	double angle_step = 2.0 * M_PI * carrier_frequency * sampling_interval;
	double angle_start = angle_step * (double)sample_offset;
	double pr = std::cos(angle_start);
	double pi = std::sin(angle_start);
	double sr = std::cos(angle_step);
	double si = std::sin(angle_step);
	for (int i = 0; i < in_size; i++)
	{
		double a = in[i] * carrier_amplitude;
		p2b_l_data[i].real(a * pr);
		p2b_l_data[i].imag(a * pi);
		double npr = pr * sr - pi * si;
		double npi = pr * si + pi * sr;
		pr = npr;
		pi = npi;
	}

	// Combined FIR + decimation: produces only in_size/M outputs.
	filter->apply_decimate(p2b_l_data, out, in_size, M);
}
