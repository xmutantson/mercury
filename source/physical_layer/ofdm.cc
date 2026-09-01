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
#include <vector>   // SPARSE-OFDM decimation overlay (env-gated)
#include <cstdlib>  // SPARSE-OFDM: std::getenv/atoi

// PocketFFT: high-performance FFT library (BSD license)
// Replaces hand-rolled Cooley-Tukey. ~2-3x faster for N=256.
#define POCKETFFT_NO_MULTITHREADING
#include "physical_layer/pocketfft_hdronly.h"

#include <map>
#include <vector>  // §20: per-bin power accumulator for base-pattern combining
#include <cstdlib> // std::getenv for the MERCURY_FFT_FLOAT gate (marathon lever H/I)
#include <cstring> // std::strcmp for the MERCURY_DFTSMOOTH mode selector + memset zeroing of st_carrier/st_channel_complex padding at alloc (PRECOOK determinism); Debian g++12 requires the explicit include (MinGW pulled it in transitively)

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

// --- Marathon lever H/I: SINGLE-PRECISION OFDM FFT (env MERCURY_FFT_FLOAT) ---
// CONTAINED scope. The OFDM/channel-estimator chain stays std::complex<double>
// everywhere; only the transform kernel inside fft()/ifft() runs in float when
// the env is set. We convert complex<double> -> complex<float> at the transform
// boundary, run a pocketfft_c<float> plan (its own size-keyed cache, parallel to
// the double cache above), and convert back to complex<double> on the way out.
// DEFAULT OFF => the double path is taken bit-for-bit (render md5 unchanged).
//
// Rationale (HONEST: the FFT is NOT the OFDM bottleneck — the LDPC decode is;
// this is a completionist throughput lever): single-precision NEON on the Pi5
// A76 packs 4 floats vs 2 doubles per 128-bit reg, ~2x the FFT throughput, and
// float FFT is the HF-modem standard (VARA, codec2). Numerical impact is bounded
// — a 256-pt FFT in float carries ~1e-6 relative error, far below the post-EQ
// EVM / LDPC LLR scale, so decode is unaffected (verified by the marathon test).
//
// The env is read ONCE into a function-local static so the per-FFT hot path is a
// single branch, never a getenv() call. Single-threaded access per the cache
// comment above => no init race.
bool fft_float_enabled()
{
	static const bool on = (std::getenv("MERCURY_FFT_FLOAT") != nullptr);
	return on;
}

// E4 idle-CPU lever (detect-fft-memo): default-on memoization of the coarse ACK/HAIL
// correlator's per-symbol FFT. The coarse sliding search re-FFTs the SAME symbol
// window once per start position that spans it (~ack_nsymb times); caching each
// distinct symbol's |FFT|^2 once yields byte-identical detection with ~10x fewer
// FFTs at idle. Default ON (fleet-validated); disable with
// MERCURY_DETECT_FFT_MEMO=0. Read once into a function-local static so the poll
// hot path is a single branch, never a getenv() call.
bool detect_fft_memo_enabled()
{
	static const bool on = []{
		const char* e = std::getenv("MERCURY_DETECT_FFT_MEMO");
		return !(e && *e && atoi(e) == 0);   // DEFAULT-ON: explicit =0 restores stock
	}();
	return on;
}

std::map<size_t, pocketfft::detail::pocketfft_c<float>>& fft_plan_cache_f()
{
	static std::map<size_t, pocketfft::detail::pocketfft_c<float>> cache;
	return cache;
}
const pocketfft::detail::pocketfft_c<float>& get_fft_plan_f(size_t n)
{
	auto& cache = fft_plan_cache_f();
	auto it = cache.find(n);
	if (it == cache.end())
	{
		it = cache.emplace(std::piecewise_construct,
		                   std::forward_as_tuple(n),
		                   std::forward_as_tuple(n)).first;
	}
	return it->second;
}

// Run an n-point complex FFT in single precision: convert in -> float scratch,
// execute the cached float plan in place, convert back to the double out buffer.
// fct = the pocketfft exec factor (1/N forward-scaled fft, 1.0 ifft); fwd as in
// the double path. Caller has already copied/placed nothing — we read from `in`
// and write to `out`, matching the double-path contract.
void run_fft_float(const std::complex<double>* in, std::complex<double>* out,
                   size_t n, float fct, bool fwd)
{
	static thread_local std::vector<std::complex<float>> scratch;
	if (scratch.size() < n) scratch.resize(n);
	for (size_t k = 0; k < n; ++k)
		scratch[k] = std::complex<float>((float)in[k].real(), (float)in[k].imag());
	const auto& plan = get_fft_plan_f(n);
	plan.exec(reinterpret_cast<pocketfft::detail::cmplx<float>*>(scratch.data()),
	          fct, fwd);
	for (size_t k = 0; k < n; ++k)
		out[k] = std::complex<double>((double)scratch[k].real(),
		                              (double)scratch[k].imag());
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
	tinterp_smooth_halfwin=0; // feat/fade-tinterp: TIME_INTERP pilot pre-smooth off by default
	dd_data_conf_thresh=0.30; // Turbo-EQ: data-aided improve-only confidence threshold (read only inside the turbo loop)
	dd_seed_floor=false; // Turbo-EQ TINTERP-seed: false = pilots-only floor (byte-identical default); true = keep the it=0 (TINTERP) H as the low-confidence floor
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
	// E4 detect-fft-memo: coarse-correlator per-symbol FFT-power cache (grow-once)
	detect_memo_pow=NULL;
	detect_memo_cap=0;
	detect_ack_fft_count=0;
	detect_memo_force=-1;
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
	for(int i=0;i<48;i++) mfsk_preamble_tones[i]=0;
	mfsk_alt_preamble_nsymb=0;
	mfsk_alt_match_threshold=0;
	for(int i=0;i<48;i++) mfsk_alt_preamble_tones[i]=0;
	mfsk_matched_preamble_nsymb=0;
	mfsk_matched_alt=false;
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
	// PRECOOK determinism: st_carrier / st_channel_complex carry 4 bytes of indeterminate
	// trailing PADDING after their int member (24-byte struct = complex<double>[16] + int[4] +
	// pad[4]). init() writes every .value/.type/.status but NEVER the padding, so `new[]` leaves
	// it as heap junk. That is harmless functionally (the DSP never reads padding), but it makes
	// a raw memcmp of two INDEPENDENTLY-allocated arrays non-deterministic — which broke the
	// precook bundle==init byte-identical gate (and would make any future memcmp-based geometry
	// verification unreliable). Zeroing the padding here makes these arrays fully deterministic;
	// the on-wire signal is byte-identical (padding is never modulated). copy_from's memcpy then
	// propagates the zero padding into every bundle copy.
	memset(ofdm_frame, 0, sizeof(struct st_carrier) * (size_t)this->Nsymb * (size_t)this->Nc);
	memset(estimated_channel, 0, sizeof(struct st_channel_complex) * (size_t)this->Nsymb * (size_t)this->Nc);
	memset(estimated_channel_without_amplitude_restoration, 0, sizeof(struct st_channel_complex) * (size_t)this->Nsymb * (size_t)this->Nc);
	memset(ofdm_preamble, 0, sizeof(struct st_carrier) * (size_t)this->preamble_configurator.Nsymb * (size_t)this->Nc);
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
	if(detect_memo_pow!=NULL){delete[] detect_memo_pow; detect_memo_pow=NULL;}
	detect_memo_cap=0;
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

	// Allocate scratch buffer. PRECOOK determinism: fft_scratch is pure per-FFT working
	// storage (written each transform before read), so `new[]` leaves indeterminate content
	// the modem never reads at rest — zero it so the precook bundle==init memcmp gate is
	// meaningful (FFT output unchanged; the transform overwrites it before reading).
	fft_scratch = CNEW(std::complex<double>, n, "ofdm.fft_scratch");
	memset(fft_scratch, 0, sizeof(std::complex<double>) * (size_t)n);

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
	// Marathon lever H/I: single-precision transform when MERCURY_FFT_FLOAT set
	// (contained — buffers stay complex<double>, only the kernel runs in float).
	if (fft_float_enabled())
	{
		run_fft_float(in, out, (size_t)Nfft, 1.0f / (float)Nfft, true);
		return;
	}
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
	if (fft_float_enabled())
	{
		run_fft_float(in, out, (size_t)_Nfft, 1.0f / (float)_Nfft, true);
		return;
	}
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
	if (fft_float_enabled())
	{
		run_fft_float(in, out, (size_t)Nfft, 1.0f, false);
		return;
	}
	std::copy(in, in + Nfft, out);
	const auto& plan = get_fft_plan((size_t)Nfft);
	plan.exec(reinterpret_cast<pocketfft::detail::cmplx<double>*>(out), 1.0, false);
}

void cl_ofdm::ifft(std::complex <double>* in, std::complex <double>* out,int _Nfft)
{
	if (fft_float_enabled())
	{
		run_fft_float(in, out, (size_t)_Nfft, 1.0f, false);
		return;
	}
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
	sparse_wide_data_carriers=0;
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
	// PRECOOK determinism: zero the st_carrier padding (see cl_ofdm::init note). virtual_carrier
	// cells set only .type below; without this the trailing pad bytes are heap junk and a memcmp
	// of two independent builds diverges. On-wire byte-identical (padding never modulated).
	memset(virtual_carrier, 0, sizeof(struct st_carrier) * (size_t)this->Nc_max * (size_t)this->Nc_max);

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
	// LOW48 S20-R6 fixed sparse-wide grid.  Twenty QPSK data columns are
	// distributed from carrier 0 through 49 by nearest-integer interpolation.
	// Ten continual pilot columns are distributed over the complementary set;
	// the remaining twenty columns are zero.  Holding the column roles for all
	// 40 symbols gives exactly 800 data cells (1600 coded QPSK bits) while keeping
	// data, pilot, and zero sets disjoint and TX/RX symmetric.
	if(sparse_wide_data_carriers > 0)
	{
		const int nDataColumns = sparse_wide_data_carriers;
		const int nPilotColumns = nDataColumns / 2;
		if(Nc != 50 || Nsymb != 40 || nDataColumns != 20 || nPilotColumns != 10)
		{
			fprintf(stderr, "[LOW48-GUARD] invalid sparse geometry Nc=%d Nsymb=%d dataCols=%d pilotCols=%d\n",
				Nc, Nsymb, nDataColumns, nPilotColumns);
			fflush(stderr);
			exit(EXIT_FAILURE);
		}

		std::vector<char> data_column((size_t)Nc, 0);
		std::vector<char> pilot_column((size_t)Nc, 0);
		std::vector<int> complement;
		for(int k=0; k<nDataColumns; k++)
		{
			const int col = (k*(Nc-1) + (nDataColumns-1)/2) / (nDataColumns-1);
			if(col < 0 || col >= Nc || data_column[(size_t)col])
			{
				fprintf(stderr, "[LOW48-GUARD] duplicate/out-of-range data carrier %d\n", col);
				exit(EXIT_FAILURE);
			}
			data_column[(size_t)col] = 1;
		}
		for(int col=0; col<Nc; col++)
			if(!data_column[(size_t)col]) complement.push_back(col);
		for(int k=0; k<nPilotColumns; k++)
		{
			const int ci = (k*((int)complement.size()-1) + (nPilotColumns-1)/2) / (nPilotColumns-1);
			const int col = complement[(size_t)ci];
			if(col < 0 || col >= Nc || data_column[(size_t)col] || pilot_column[(size_t)col])
			{
				fprintf(stderr, "[LOW48-GUARD] overlapping/out-of-range pilot carrier %d\n", col);
				exit(EXIT_FAILURE);
			}
			pilot_column[(size_t)col] = 1;
		}

		nData=0;
		nPilots=0;
		nConfig=0;
		for(int row=0; row<Nsymb; row++)
		{
			for(int col=0; col<Nc; col++)
			{
				int type = ZERO;
				if(data_column[(size_t)col]) type = DATA;
				else if(pilot_column[(size_t)col]) type = PILOT;
				(virtual_carrier+row*Nc_max+col)->type=type;
				(carrier+row*Nc+col)->type=type;
				if(type==DATA) nData++;
				else if(type==PILOT) nPilots++;
			}
		}
		if(nData != 800 || nPilots != 400)
		{
			fprintf(stderr, "[LOW48-GUARD] cell-count mismatch data=%d pilots=%d\n", nData, nPilots);
			exit(EXIT_FAILURE);
		}
		printf("[LOW48-GEOMETRY] dataCols=");
		for(int col=0; col<Nc; col++) if(data_column[(size_t)col]) printf("%s%d", col?",":"", col);
		printf(" pilotCols=");
		bool first=true;
		for(int col=0; col<Nc; col++) if(pilot_column[(size_t)col]) { printf("%s%d", first?"":",", col); first=false; }
		printf(" nData=%d nPilots=%d nZero=%d\n", nData, nPilots, Nc*Nsymb-nData-nPilots);
		fflush(stdout);
		return;
	}

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

	// SPARSE-OFDM decimation overlay (env-gated; production unset => no-op, byte-identical).
	// VARA L5-9 lever: light a FIXED-LATTICE SUBSET of the DATA carriers spread WIDE for
	// frequency diversity. MERCURY_MFSK_SPARSE_NCARR=n keeps n DATA carrier columns on a
	// fixed lattice (spacing = MERCURY_MFSK_SPARSE_STRIDE, else auto = Nc/n) centered in the
	// band; every other DATA cell -> ZERO. Symmetric TX/RX (both read carrier->type), so
	// masking this one array decimates the whole framer/deframer path. PILOT/CONFIG cells
	// are left untouched so the channel estimator keeps its reference. nData is recomputed
	// so nBits/LDPC-puncturing track the sparse grid. Additive: env unset => block skipped.
	{
		const char* _spn = std::getenv("MERCURY_MFSK_SPARSE_NCARR");
		if(_spn && *_spn)
		{
			int ncarr = atoi(_spn);
			const char* _sps = std::getenv("MERCURY_MFSK_SPARSE_STRIDE");
			int stride = (_sps && *_sps) ? atoi(_sps) : 0;
			if(ncarr >= 1 && ncarr < Nc)
			{
				if(stride < 1) stride = Nc / ncarr;
				if(stride < 1) stride = 1;
				int span  = (ncarr-1)*stride;
				int start = (Nc - 1 - span) / 2;
				if(start < 0) start = 0;
				std::vector<char> keep((size_t)Nc, 0);
				for(int c=0;c<ncarr;c++){ int col = start + c*stride; if(col>=0 && col<Nc) keep[(size_t)col]=1; }
				int removed=0;
				for(int j=0;j<Nsymb;j++) for(int i=0;i<Nc;i++)
				{
					struct st_carrier* cc = carrier + j*Nc + i;
					if(cc->type==DATA && !keep[(size_t)i]) { cc->type=ZERO; nData--; removed++; }
				}
				std::cout<<"[SPARSE-OFDM] ncarr="<<ncarr<<" stride="<<stride<<" start="<<start
				         <<" Nc="<<Nc<<" removedDataCells="<<removed<<" nData="<<nData<<std::endl;
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
	// PRECOOK determinism: only PREAMBLE-type carrier cells write into sequence[] below (seq_idx
	// advances per PREAMBLE cell), leaving the trailing entries as indeterminate `new[]` content
	// the modem never reads. Zero the whole buffer so two independent builds are memcmp-identical
	// (the precook bundle==init gate); on-wire byte-identical (unwritten entries are never read).
	memset(sequence, 0, sizeof(std::complex<double>) * (size_t)this->Nsymb * (size_t)this->Nc);

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
double cl_ofdm::estimate_noise_from_pilot_pairs(std::complex<double>* in,
	bool adjacent_rows, int* pair_count_out)
{
	if (pair_count_out != nullptr) *pair_count_out = 0;
	if (Nsymb <= 0 || Nc <= 0) return 0.01;
	int Dy = pilot_configurator.Dy;
	if (!adjacent_rows && Dy <= 0) return 0.01;

	// Per-column state: last pilot row and raw H = Y/X for that pilot.
	int prev_row[Nc];                       // VLA, Nc <= 50
	std::complex<double> prev_H[Nc];        // VLA
	std::complex<double> prev_X[Nc];        // VLA
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

				int expected_delta = adjacent_rows ? 1 : Dy;
				if (prev_row[j] >= 0 && (i - prev_row[j]) == expected_delta)
				{
					std::complex<double> delta = H_raw - prev_H[j];
					double mag2 = delta.real()*delta.real() + delta.imag()*delta.imag();
					if (adjacent_rows)
					{
						double x_energy = std::norm(X);
						double prev_x_energy = std::norm(prev_X[j]);
						if (x_energy > 0.0 && prev_x_energy > 0.0)
						{
							// Var(Y/X - Y_prev/X_prev) = sigma^2
							// * (1/|X|^2 + 1/|X_prev|^2).
							noise_sum += mag2 / (1.0/x_energy + 1.0/prev_x_energy);
							noise_count++;
						}
					}
					else
					{
						// Preserve the established regular-lattice estimator units.
						noise_sum += mag2 * 0.5;
						noise_count++;
					}
				}

				prev_row[j] = i;
				prev_H[j] = H_raw;
				prev_X[j] = X;
				pilot_index++;
			}
		}
	}

	if (pair_count_out != nullptr) *pair_count_out = noise_count;
	if (noise_count <= 0) return 0.01;
	double nv = noise_sum / noise_count;
	if (nv < 1e-6) nv = 1e-6;   // prevent division instability at very high SNR
	return nv;
}

// Geometry-invariant timing-quality selector. Walks the MEASURED (raw pilot)
// cells of estimated_channel and stores the phase-coherence factor
//   C = |Sum_p H_p| / Sum_p |H_p|   in last_pilot_coherence (see ofdm.h).
// MUST be called while only pilot cells are MEASURED (before interpolation and
// smooth_channel_estimate_dft), so it reflects the raw per-pilot phase structure.
void cl_ofdm::compute_pilot_coherence()
{
	std::complex<double> vec_sum(0.0, 0.0);
	double mag_sum = 0.0;
	for(int ci = 0; ci < Nsymb * Nc; ci++)
	{
		if((estimated_channel + ci)->status == MEASURED)
		{
			vec_sum += (estimated_channel + ci)->value;
			mag_sum += std::abs((estimated_channel + ci)->value);
		}
	}
	last_pilot_coherence = (mag_sum > 1e-12) ? (std::abs(vec_sum) / mag_sum) : -1.0;
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

	// Timing-quality selector: pilot phase COHERENCE over the RAW ZF estimates,
	// captured HERE (only pilot cells are MEASURED; interpolation + DFT smoothing
	// have not run yet). See ofdm.h last_pilot_coherence.
	compute_pilot_coherence();

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

double cl_ofdm::pilot_magnitude_cv(const double* magnitudes, int count)
{
	if(magnitudes == NULL || count <= 1) return -1.0;
	double scale = 0.0;
	for(int i=0; i<count; i++)
	{
		if(!std::isfinite(magnitudes[i]) || magnitudes[i] < 0.0) return -1.0;
		if(magnitudes[i] > scale) scale = magnitudes[i];
	}
	if(!std::isfinite(scale) || scale <= 0.0) return -1.0;

	// Scale into [0,1] before accumulating. This preserves coefficient of
	// variation while preventing finite DBL_MAX inputs from overflowing m*m.
	double scaled_sum = 0.0;
	for(int i=0; i<count; i++) scaled_sum += magnitudes[i] / scale;
	double scaled_mean = scaled_sum / count;
	double physical_mean = scale * scaled_mean;
	if(!std::isfinite(scaled_mean) || !std::isfinite(physical_mean)
		|| physical_mean <= 1e-12) return -1.0;

	double variance_sum = 0.0;
	for(int i=0; i<count; i++)
	{
		double delta = magnitudes[i] / scale - scaled_mean;
		variance_sum += delta * delta;
	}
	double variance = variance_sum / count;
	if(variance < 0.0) variance = 0.0;
	double result = sqrt(variance) / scaled_mean;
	return std::isfinite(result) ? result : -1.0;
}

double cl_ofdm::measure_pilot_selectivity(const std::complex<double>* in,
	                                      int* usable_count) const
{
	if(usable_count != NULL) *usable_count = 0;
	if(in == NULL || Nsymb <= 0 || Nc <= 0) return -1.0;

	std::vector<double> magnitudes;
	int pidx = 0;
	for(int i=0; i<Nsymb; i++)
	{
		for(int j=0; j<Nc; j++)
		{
			if((ofdm_frame+i*Nc+j)->type != PILOT) continue;
			std::complex<double> x = pilot_configurator.sequence[pidx++];
			std::complex<double> y = *(in+i*Nc+j);
			if(!std::isfinite(x.real()) || !std::isfinite(x.imag())) return -1.0;
			double xmag = std::abs(x);
			if(!std::isfinite(xmag)) return -1.0;
			if(xmag <= 1e-12) continue;
			if(!std::isfinite(y.real()) || !std::isfinite(y.imag())) return -1.0;
			double ymag = std::abs(y);
			if(!std::isfinite(ymag)) return -1.0;
			double magnitude = ymag / xmag;
			if(!std::isfinite(magnitude)) return -1.0;
			magnitudes.push_back(magnitude);
		}
	}
	if(usable_count != NULL) *usable_count = (int)magnitudes.size();
	return pilot_magnitude_cv(magnitudes.empty() ? NULL : magnitudes.data(),
	                          (int)magnitudes.size());
}

// Parametric notch-resolving estimator selector (DEFAULT-ON; env unset => mode 1).
// Read ONCE into a function-local static (single-threaded per-frame call). Consulted
// only for the sparse-wide continual-pilot lattice (LOW48 / config-105 class, gated on
// pilot_configurator.sparse_wide_data_carriers > 0), so DEFAULT-ON is a proven no-op on
// every shipped ladder config (all have sparse_wide_data_carriers == 0).
//   0 = OFF     : stock LS + linear frequency interpolation + DFT smoothing.
//   1 = ON      : parametric 2-ray fit when a within-CP echo is detected, else defer
//                 to stock interpolation; the existing pilot-residual nv is kept.
//   2 = ON+nvXP : same H, but nv from the cross-pilot differential thermal floor
//                 (nv-isolation control) instead of the pilot-residual estimate.
// Env MERCURY_EST_NOTCH selects the mode: unset => 1 (ON); MERCURY_EST_NOTCH=0 disables.
static int est_notch_mode()
{
	static const int mode = []() -> int {
		const char* e = std::getenv("MERCURY_EST_NOTCH");
		return (e != nullptr) ? atoi(e) : 1;
	}();
	return mode;
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

	// Frequency selectivity from instantaneous pilot LS ratios abs(Yp)/abs(Xp),
	// captured before interpolation + DFT smoothing. This common producer is
	// independent of the estimator's own smoothing/window policy. A flat channel reads
	// low (~0.04 at operating SNR), a 2-path null gives a large pilot-to-pilot |H|
	// swing => high. The post-smoothing DATA-bin metric instead reads an
	// SNR-independent ~0.4-1.3 ripple the smoother/interpolation inject, so this
	// raw-pilot reading is what feeds last_channel_selectivity.
	{
		int pilot_count = 0;
		last_pilot_selectivity = measure_pilot_selectivity(in, &pilot_count);
		if(std::getenv("MERCURY_SEL_DIAG"))
			std::cerr << "[PILOT-SEL] npil=" << pilot_count
			          << " sel=" << last_pilot_selectivity << std::endl;
	}

	// Timing-quality selector: pilot phase COHERENCE over the RAW LS estimates,
	// captured HERE (only pilot cells are MEASURED; interpolation + DFT smoothing
	// have not run yet). See ofdm.h last_pilot_coherence.
	compute_pilot_coherence();

	// ---- Parametric 2-ray (LOS + single within-CP echo) notch estimator ----
	// Env-gated MERCURY_EST_NOTCH (DEFAULT-ON: env unset => mode 1; MERCURY_EST_NOTCH=0
	// disables). The do_notch gate below additionally requires the sparse-wide continual-
	// pilot lattice (sparse_wide_data_carriers>0, config-105 class), so on every shipped
	// ladder config this whole block is skipped and the output is byte-identical to the
	// stock estimator. The sparse continual-pilot
	// lattice under-resolves a within-guard echo whose frequency-ripple period
	// (Nfft/D carriers) approaches the pilot-grid Nyquist, so linear frequency
	// interpolation mis-places the fade null and yields a confident-wrong H between
	// pilots (the LOW48 poor-fade FER floor). The physical channel here is one LOS ray
	// at delay 0 plus one scatter ray inside the guard interval, so H(k) = g0 +
	// g1*exp(-j*2*pi*k*D/Nfft) is a 5-real-parameter model. Fit it directly to the
	// pilot LS ratios instead of interpolating: estimate the echo delay D ONCE per
	// frame (pooling the fit residual over every symbol, since the geometry is static
	// over the slow Doppler) and re-fit the complex gains PER SYMBOL to follow the
	// intra-frame Doppler; then reconstruct H analytically at every carrier -- exact
	// at the fade null, unlike interpolation. A diagonal load stabilizes the 2x2 gain
	// solve; a model-order gate accepts the echo only when it truly resolves one, and
	// on a clean channel the estimator DEFERS to the stock interpolation+smoother.
	// Refs: Y. (G.) Li, L. J. Cimini et al., "Channel Estimation for OFDM Transmission
	// in Multipath Fading Channels Based on Parametric Channel Modeling," IEEE Trans.
	// Comm. 49(3):467-479, 2001; O. Simeone, Y. Bar-Ness, U. Spagnolini, "Pilot-Based
	// Channel Estimation for OFDM Systems by Tracking the Delay-Subspace," IEEE Trans.
	// Wireless Comm. 3(1):315-325, 2004 (slow delay subspace + fast per-symbol gains);
	// diagonal loading per O. Edfors et al., "OFDM Channel Estimation by SVD," IEEE
	// Trans. Comm. 46(7):931-939, 1998.
	int  est_notch    = est_notch_mode();
	bool do_notch     = (est_notch != 0 && pilot_configurator.sparse_wide_data_carriers > 0 && Nc > 10);
	bool notch_filled = false;   // set true once the parametric writer overwrites H
	if(do_notch)
	{
		const int    Nc2   = Nc/2;
		const int    Dmax  = (Ngi > 1) ? Ngi : 1;
		const double twopi = 2.0*acos(-1.0);

		// (1) Continual-pilot columns + their SIGNED FFT-bin indices (framer map:
		//     carrier j<Nc/2 -> bin j-Nc/2 ; j>=Nc/2 -> j-Nc/2+start_shift).
		std::vector<int>    pil_col;
		std::vector<double> pil_k;
		for(int j=0; j<Nc; j++)
			if((ofdm_frame + 0*Nc + j)->type == PILOT)
			{
				pil_col.push_back(j);
				pil_k.push_back( (j < Nc2) ? (double)(j - Nc2)
				                           : (double)(j - Nc2 + start_shift) );
			}
		int nPil = (int)pil_col.size();

		if(nPil >= 3)
		{
			// (2) Snapshot the windowed-LS pilot estimates (the composite H the
			//     equalizer consumes) before any overwrite.
			std::vector<std::complex<double> > y((size_t)Nsymb*nPil);
			for(int s=0; s<Nsymb; s++)
				for(int p=0; p<nPil; p++)
					y[(size_t)s*nPil+p] = (estimated_channel + s*Nc + pil_col[p])->value;

			const double ridge = 1e-3 * (double)nPil;   // Tikhonov diagonal load

			// (3) Flat (single-ray) frame residual: the no-echo null hypothesis.
			double flat_res = 0.0;
			for(int s=0; s<Nsymb; s++)
			{
				std::complex<double> sum(0,0);
				for(int p=0; p<nPil; p++) sum += y[(size_t)s*nPil+p];
				std::complex<double> g0 = sum / (double)nPil;
				for(int p=0; p<nPil; p++)
				{ std::complex<double> d = y[(size_t)s*nPil+p]-g0; flat_res += d.real()*d.real()+d.imag()*d.imag(); }
			}

			// (4) 1-D delay search: LOS pinned at delay 0, echo delay d1 in [1,Dmax].
			//     The 2x2 normal matrix depends only on d1, so build it once per d1.
			int best_d1 = 0; double best_res = flat_res;
			std::vector<std::complex<double> > b1(nPil);
			for(int d1=1; d1<=Dmax; d1++)
			{
				for(int p=0; p<nPil; p++)
				{ double ph = -twopi*pil_k[p]*(double)d1/(double)Nfft; b1[p]=std::complex<double>(cos(ph),sin(ph)); }
				std::complex<double> M01(0,0); double M11=0.0;
				for(int p=0; p<nPil; p++){ M01+=b1[p]; M11+=std::norm(b1[p]); }
				double M00r=(double)nPil+ridge, M11r=M11+ridge;
				std::complex<double> M10=std::conj(M01);
				std::complex<double> det=M00r*M11r-M01*M10;
				if(std::abs(det)<1e-12) continue;
				double tot=0.0;
				for(int s=0; s<Nsymb; s++)
				{
					std::complex<double> r0(0,0),r1(0,0);
					for(int p=0; p<nPil; p++){ std::complex<double> yy=y[(size_t)s*nPil+p]; r0+=yy; r1+=std::conj(b1[p])*yy; }
					std::complex<double> g0=(M11r*r0-M01*r1)/det;
					std::complex<double> g1=(-M10*r0+M00r*r1)/det;
					for(int p=0; p<nPil; p++){ std::complex<double> d=y[(size_t)s*nPil+p]-(g0+g1*b1[p]); tot+=d.real()*d.real()+d.imag()*d.imag(); }
				}
				if(tot<best_res){ best_res=tot; best_d1=d1; }
			}

			// (5) Model-order gate (GLRT-style): accept the echo ONLY if it cuts the
			//     pooled residual substantially. Adding one complex gain (2 real DOF)
			//     to 10 complex pilots reduces a pure-NOISE residual by only ~11%,
			//     while a genuine within-CP echo cuts it by >90% (poor d10 97%,
			//     moderate 98%). A 50% floor cleanly separates them. On rejection
			//     (clean/echo-free channel) the parametric writer is skipped and the
			//     estimator DEFERS to the stock interpolation+smoother below, so AWGN
			//     and non-fading channels stay byte-identical to the stock estimator.
			bool use_echo = (best_d1 > 0) && (best_res < 0.5*flat_res);

			if(use_echo)
			{
				int d1 = best_d1;
				for(int p=0; p<nPil; p++)
				{ double ph=-twopi*pil_k[p]*(double)d1/(double)Nfft; b1[p]=std::complex<double>(cos(ph),sin(ph)); }
				std::complex<double> M01(0,0); double M11=0.0;
				for(int p=0; p<nPil; p++){ M01+=b1[p]; M11+=std::norm(b1[p]); }
				double M00r=(double)nPil+ridge, M11r=M11+ridge;
				std::complex<double> M10=std::conj(M01);
				std::complex<double> det=M00r*M11r-M01*M10;
				for(int s=0; s<Nsymb; s++)
				{
					std::complex<double> r0(0,0),r1(0,0);
					for(int p=0; p<nPil; p++){ std::complex<double> yy=y[(size_t)s*nPil+p]; r0+=yy; r1+=std::conj(b1[p])*yy; }
					std::complex<double> g0=(M11r*r0-M01*r1)/det;
					std::complex<double> g1=(-M10*r0+M00r*r1)/det;
					for(int j=0; j<Nc; j++)
					{
						double kj=(j<Nc2)?(double)(j-Nc2):(double)(j-Nc2+start_shift);
						double ph=-twopi*kj*(double)d1/(double)Nfft;
						(estimated_channel + s*Nc + j)->value  = g0 + g1*std::complex<double>(cos(ph),sin(ph));
						(estimated_channel + s*Nc + j)->status = MEASURED;
					}
				}
				notch_filled = true;   // parametric H written => skip interp + smoother
			}
			// else: echo rejected => leave raw LS pilots intact, fall through to stock.

			// (6) One-shot activation witness (fire proof): the estimator fired, with
			//     the recovered echo delay and null-vs-echo residuals. Poor bracket:
			//     the 2 ms echo => d1 ~ 24 baseband samples; a clean channel => d1
			//     rejected (use_echo=0), deferring to interpolation.
			static bool _notch_banner = false;
			if(!_notch_banner)
			{
				_notch_banner = true;
				printf("[EST-NOTCH] mode=%d %s Nc=%d Nsymb=%d nPil=%d d1=%d use_echo=%d "
				       "flat_res=%.4e echo_res=%.4e Dmax=%d\n",
				       est_notch, use_echo ? "parametric" : "deferred", Nc, Nsymb, nPil,
				       best_d1, use_echo?1:0, flat_res, best_res, Dmax);
				fflush(stdout);
			}
		}
		// nPil<3 (never on the LOW48 10-pilot lattice) also defers to stock below.
	}

	if(!notch_filled)
	{
	if(pilot_configurator.sparse_wide_data_carriers > 0)
	{
		// The S20 grid uses continual pilot COLUMNS rather than the production
		// diagonal Dx/Dy lattice.  Interpolate in frequency between the measured
		// pilot columns on every row and hold the two band edges.  The generic
		// Dx=1 code below assumes every column contains time pilots and therefore
		// cannot fill an all-data/all-zero column.
		for(int row=0; row<Nsymb; row++)
		{
			int first=-1, last=-1;
			for(int col=0; col<Nc; col++)
				if((estimated_channel+row*Nc+col)->status==MEASURED)
				{ if(first<0) first=col; last=col; }
			if(first < 0)
			{
				fprintf(stderr, "[LOW48-GUARD] estimator row %d has no pilot anchor\n", row);
				exit(EXIT_FAILURE);
			}
			for(int col=0; col<first; col++)
				(estimated_channel+row*Nc+col)->value=(estimated_channel+row*Nc+first)->value;
			for(int col=last+1; col<Nc; col++)
				(estimated_channel+row*Nc+col)->value=(estimated_channel+row*Nc+last)->value;
			int previous=first;
			for(int col=first+1; col<=last; col++)
			{
				if((estimated_channel+row*Nc+col)->status==MEASURED)
				{
					std::complex<double> h0=(estimated_channel+row*Nc+previous)->value;
					std::complex<double> h1=(estimated_channel+row*Nc+col)->value;
					for(int fill=previous+1; fill<col; fill++)
					{
						double t=(double)(fill-previous)/(double)(col-previous);
						(estimated_channel+row*Nc+fill)->value=h0*(1.0-t)+h1*t;
					}
					previous=col;
				}
			}
			for(int col=0; col<Nc; col++)
				(estimated_channel+row*Nc+col)->status=MEASURED;
		}
	}
	else
	{
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
	}   // end if(!notch_filled) -- stock interpolation branch

	// DFT-based channel estimate smoothing (same as ZF estimator). For the LS
	// path this runs BEFORE the noise-variance estimate (restored pre-E1 order,
	// reverting commit 38f5c60 for THIS estimator only) so the residual is
	// measured against the SAME final smoothed+interpolated H that the data
	// carriers are equalized with. See fix/cfg16-nv-restore rationale below.
	// The parametric notch estimator writes an analytic (already delay-limited) H,
	// so the DFT time-window smoother is skipped when it fired (notch_filled).
	if(!notch_filled)
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

	// Parametric notch nv-isolation control (MERCURY_EST_NOTCH=2): keep the parametric
	// H but take nv from the cross-pilot differential thermal floor instead of the
	// pilot-residual estimate above. Isolates the nv contribution; mode 1 keeps the
	// residual nv. Default (mode 0/OFF) leaves this untouched.
	if(do_notch && est_notch == 2)
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

// feat/fade-tinterp: FADE-tier per-carrier LINEAR TIME-INTERPOLATION estimator.
//
// PROMOTED, byte-for-byte in logic, from the sim-proven MERCURY_SFO_GRID_TINTERP
// prototype (compute_program/jobs/fade-estimator-prototypes/estimator_candidates.cc.frag,
// 900-cell verdict ESTIMATOR_PROTOTYPES_VERDICT.md). The prototype crossed FULL-decode
// viability on the GOOD (MPG/0.1 Hz) Watterson fade where the production LS-3x9
// window delivers ZERO codewords, and reached ARQ-viable mean-fraction on MODERATE
// (MPM/0.5 Hz). It is the cheap NONCOHERENT fade lever (no rate drop, no coherent
// demod) — see fact-documents/fade-estimator-prototypes.md.
//
// Mechanism: on the dense Dx=1/Dy=3 lattice EVERY carrier carries a pilot every Dy
// symbols, so each carrier's H(t) is a 1-D time series sampled every Dy symbols.
// Linear-interpolate H BETWEEN consecutive time-pilots per carrier (hold at the
// edges). This FOLLOWS a Doppler fade where the held LS window AVERAGES (and lags)
// it. A trailing freq-interp pass fills any all-data carrier (Dx=1 => none, but the
// pass is kept for safety / future denser-pilot or thinned lattices).
//
// nv (the cross-layer hazard): the pilot-residual EVM against the interpolated H,
// FLOORED at the cross-pilot differential AWGN estimate (estimate_noise_from_pilot_
// pairs, the same pre-EQ thermal-floor estimator the ZF path trusts). A noise-
// suppressing time-interpolation drives the pilot residual BELOW the true noise
// floor on a slow/clean channel; without the floor nv collapses toward 1e-6, the
// MMSE erasure alpha=|H|^2/(|H|^2+nv) saturates to 1 (no erasure), and psk.demod's
// LLR=dD/nv goes ~1000x over-confident -> BP iter-caps at 101 -> CRC fail. This is
// the E1/cfg16-nvfix collapse class (fade-estimator-prototypes.md §4.1). The harness
// prototype floored at the KNOWN Es/N0 10^(-EsN0/10); production has no known Es/N0,
// so we floor at the cross-pilot differential measurement of the SAME quantity. Every
// downstream consumer of noise_variance_estimate (channel_equalizer MMSE erasure
// ofdm.cc:2200, psk.demod LLR scale, the SKIP-VAR sync gate telecom_system.cc:2871,
// arq_common diag) sees an honest floor — see data-flow-noise_variance_estimate.md.
//
// Default-OFF: reached ONLY when channel_estimator == TIME_INTERP. With the FADE
// tier ungated the production decode path stays on LEAST_SQUARE => byte-identical.
//
// Ref: H. Mostofi & D. C. Cox, "Pilot-symbol aided channel estimation for OFDM with
// fast fading channels," IEEE Trans. Wireless Comm. 2005 (Xplore 1247797); codec2
// FreeDV-700D HF linear time-interpolation.
void cl_ofdm::LS_channel_estimator_tinterp(std::complex <double>*in)
{
	const int N = Nsymb;            // symbols (time)
	const int C = Nc;               // carriers (freq)
	if(N <= 0 || C <= 0)
	{
		// degenerate frame: fall back to the LS estimator so consumers still get a
		// MEASURED estimate + a valid nv (matches the harness "carrier has no pilot"
		// safety; never hit in production where N,C>0).
		LS_channel_estimator(in);
		return;
	}

	const int sm = (tinterp_smooth_halfwin > 0) ? tinterp_smooth_halfwin : 0;

	// raw LS at every pilot cell: Hp = Y/X. known[] marks pilot anchors.
	std::vector<std::complex<double>> Hp((size_t)N*C, std::complex<double>(0,0));
	std::vector<char> known((size_t)N*C, 0);
	{
		int pidx = 0;
		for(int n=0;n<N;n++) for(int j=0;j<C;j++)
			if((ofdm_frame + n*C + j)->type == PILOT)
			{
				std::complex<double> X = pilot_configurator.sequence[pidx++];
				if(std::abs(X) > 1e-12)
				{
					Hp[(size_t)n*C+j] = *(in + n*C + j) / X;
					known[(size_t)n*C+j] = 1;
				}
			}
	}

	// Publish frequency selectivity from the same instantaneous pilot LS ratios
	// the normal LS path uses, before interpolation or smoothing. receive_byte clears the member
	// before every estimator pass; assigning every exit here prevents a TINTERP
	// retry from retaining an earlier frame/config value. With fewer than two
	// finite anchors (or a zero mean), keep the fail-closed -1 sentinel so the
	// caller cannot mistake an invalid interpolation for a channel measurement.
	{
		int pilot_count = 0;
		last_pilot_selectivity = measure_pilot_selectivity(in, &pilot_count);
		if(std::getenv("MERCURY_SEL_DIAG"))
			std::cerr << "[PILOT-SEL] npil=" << pilot_count
			          << " sel=" << last_pilot_selectivity << std::endl;
	}

	// Timing-quality selector: pilot phase COHERENCE over the RAW time-pilot LS
	// values (before time/freq interpolation + smoothing). Same definition as
	// compute_pilot_coherence(), computed on the local Hp anchors here because the
	// tinterp path marks whole columns MEASURED after interpolation. See ofdm.h.
	{
		std::complex<double> vec_sum(0.0, 0.0);
		double mag_sum = 0.0;
		for(size_t idx = 0; idx < Hp.size(); idx++)
			if(known[idx]) { vec_sum += Hp[idx]; mag_sum += std::abs(Hp[idx]); }
		last_pilot_coherence = (mag_sum > 1e-12) ? (std::abs(vec_sum) / mag_sum) : -1.0;
	}

	// per carrier: gather pilot-symbol indices, linear-interpolate in time.
	std::vector<std::complex<double>> H((size_t)N*C, std::complex<double>(0,0));
	for(int j=0;j<C;j++)
	{
		std::vector<int> pn;
		for(int n=0;n<N;n++) if(known[(size_t)n*C+j]) pn.push_back(n);
		if(pn.empty())
		{
			// carrier has no time-pilot: leave zero -> filled by freq-interp below.
			for(int n=0;n<N;n++) H[(size_t)n*C+j] = std::complex<double>(0,0);
			continue;
		}
		// optional pre-smooth of the sampled pilot series (suppress pilot noise).
		std::vector<std::complex<double>> ps(pn.size());
		for(size_t a=0;a<pn.size();a++)
		{
			if(sm <= 0){ ps[a] = Hp[(size_t)pn[a]*C+j]; continue; }
			std::complex<double> acc(0,0); int cnt=0;
			for(int w=(int)a-sm; w<=(int)a+sm; w++)
				if(w>=0 && w<(int)pn.size()){ acc += Hp[(size_t)pn[w]*C+j]; cnt++; }
			ps[a] = (cnt>0) ? acc/(double)cnt : Hp[(size_t)pn[a]*C+j];
		}
		// linear interp across the band of symbols, hold at edges.
		for(int n=0;n<pn[0];n++)            H[(size_t)n*C+j] = ps[0];
		for(int n=pn.back()+1;n<N;n++)      H[(size_t)n*C+j] = ps[pn.size()-1];
		for(size_t a=0;a+1<pn.size();a++)
		{
			int n0=pn[a], n1=pn[a+1];
			std::complex<double> h0=ps[a], h1=ps[a+1];
			for(int n=n0;n<=n1;n++)
			{
				double t = (n1>n0) ? (double)(n-n0)/(double)(n1-n0) : 0.0;
				H[(size_t)n*C+j] = h0*(1.0-t) + h1*t;
			}
		}
	}

	// frequency-interp to fill any all-data carriers (Dx=1 -> none, but safe).
	for(int n=0;n<N;n++)
	{
		std::vector<int> have;
		for(int j=0;j<C;j++) if(std::abs(H[(size_t)n*C+j]) > 0.0) have.push_back(j);
		if(have.empty()) continue;
		for(int j=0;j<have[0];j++)        H[(size_t)n*C+j] = H[(size_t)n*C+have[0]];
		for(int j=have.back()+1;j<C;j++)  H[(size_t)n*C+j] = H[(size_t)n*C+have.back()];
		for(size_t a=0;a+1<have.size();a++)
		{
			int j0=have[a], j1=have[a+1];
			std::complex<double> h0=H[(size_t)n*C+j0], h1=H[(size_t)n*C+j1];
			for(int j=j0;j<=j1;j++)
			{
				double t = (j1>j0) ? (double)(j-j0)/(double)(j1-j0) : 0.0;
				H[(size_t)n*C+j] = h0*(1.0-t) + h1*t;
			}
		}
	}

	// publish the interpolated estimate (every cell MEASURED, like LS post-interp).
	for(int ci=0; ci<N*C; ci++)
	{
		(estimated_channel+ci)->value  = H[ci];
		(estimated_channel+ci)->status = MEASURED;
	}

	// DFT smoothing is INTENTIONALLY NOT applied here: the time-interpolation IS the
	// smoother for the fade case, and smooth_channel_estimate_dft() re-imposes a
	// per-symbol frequency window that would re-average across the band (the LS path
	// runs it because its per-cell window leaves high-freq estimate noise; the
	// interpolated H is already low-noise). Matches the prototype, which published H
	// directly with no DFT pass.

	// nv = pilot-residual EVM against the interpolated H, FLOORED at the cross-pilot
	// differential AWGN estimate. The harness prototype floored at the known Es/N0
	// 10^(-EsN0/10); estimate_noise_from_pilot_pairs(in) measures the SAME pre-EQ
	// thermal floor sigma^2/|X|^2 from adjacent same-column pilot deltas, so it is the
	// production-available AWGN floor. max(residual, floor) keeps nv honest both ways:
	// it cannot collapse below the true noise on a clean/slow channel (the E1 class),
	// and it still rises with real residual estimation error on a fast fade.
	{
		double nsum = 0.0; int npil = 0; int pidx = 0;
		for(int n=0;n<N;n++) for(int j=0;j<C;j++)
			if((ofdm_frame + n*C + j)->type == PILOT)
			{
				std::complex<double> X = pilot_configurator.sequence[pidx++];
				std::complex<double> resid = *(in + n*C + j) - H[(size_t)n*C+j]*X;
				nsum += resid.real()*resid.real() + resid.imag()*resid.imag();
				npil++;
			}
		double nv_resid = (npil>0) ? nsum/(double)npil : 0.01;
		double nv_floor = estimate_noise_from_pilot_pairs(in);   // cross-pilot AWGN floor
		noise_variance_estimate = (nv_resid > nv_floor) ? nv_resid : nv_floor;
		if(noise_variance_estimate < 1e-6) noise_variance_estimate = 1e-6;
	}
}

// Turbo-EQ DATA-AIDED (decision-directed) channel estimator (RESEARCH_turbo-eq.md
// §3/§4.4). The keystone of Lever #1: after the first LDPC decode, the decoded
// codeword is soft-re-modulated to per-data-symbol soft estimates (x̄, v) which act
// as VIRTUAL PILOTS at EVERY data subcarrier. Because the decoded data lives at
// every symbol, the channel is now sampled every symbol (not every Dy=3) — this is
// what breaks the Dy=3 pilot Nyquist wall that stops the LS/TINTERP estimator on
// the POOR/1 Hz Watterson fade (the documented TINTERP wall; the GENIE compass).
//
//   in    : the raw post-FFT received grid (Nsymb*Nc), same buffer LS reads.
//   xbar  : soft symbol means in DEFRAMED DATA-cell raster order (deframer order),
//           length = nData (number of DATA cells). PILOTs are NOT in this array.
//   v     : soft symbol variances, same order/length as xbar (∈[0,1] for unit-Es).
//
// Per cell (n,j):
//   PILOT : Ĥ = rx/X, treated as a PERFECT virtual pilot (the iteration-1 floor).
//   DATA  : Ĥ_raw = rx·conj(x̄)/(|x̄|²+v)  (MMSE-style; the +v term is the soft-symbol
//           uncertainty, prevents division blow-up at x̄≈0). Used as an anchor ONLY
//           when v is below dd_data_conf_thresh (the IMPROVE-ONLY guard, dossier §6.4:
//           keep pilots-only as the floor; an uncertain cell is interpolated, not
//           trusted). Worst case (all data cells uncertain) ⇒ this ≈ the pilots-only
//           LS estimate ⇒ no harm.
// Then the SAME interpolate_linear_col / interpolate_bilinear_matrix /
// smooth_channel_estimate_dft smoothing the LS estimator uses runs over the now
// time-dense anchor lattice. nv is re-estimated from the pilot + reliable-data
// residual, FLOORED at estimate_noise_from_pilot_pairs (cross-pilot AWGN) + 1e-6 —
// the EXACT I1 guard TINTERP uses (the HW-only nv-collapse guard is NOT removed; R3).
//
// Otnes/Tüchler 2004 (iterative CE for turbo equalization of time-varying freq-
// selective channels); EURASIP 2010 (per-symbol mean/var init {0,1}, pilots perfect).
void cl_ofdm::data_aided_channel_estimator(std::complex<double>* in,
                                           std::complex<double>* xbar, double* v)
{
	if(Nsymb <= 0 || Nc <= 0)
	{
		LS_channel_estimator(in);   // degenerate-frame fallback (matches TINTERP guard)
		return;
	}

	// IMPROVE-ONLY confidence threshold on the soft-symbol variance: only trust a
	// data cell as a virtual-pilot anchor when v < thresh (a reasonably converged
	// symbol). Cells above thresh are left UNKNOWN → filled by interpolation from
	// pilots + reliable neighbors. Default 0.30 (dossier §6.4 "reliable cells").
	const double conf_thresh = (dd_data_conf_thresh > 0.0) ? dd_data_conf_thresh : 0.30;

	// TINTERP-SEED floor (TURBO_EQ_VERDICT.md §5): snapshot the INCOMING estimate
	// (the it=0 seed — TINTERP on the FADE tier) BEFORE Pass 1 clobbers it, so a
	// low-confidence DATA cell can fall back to that warm seed instead of a cold
	// pilots-only interpolation. Only taken when dd_seed_floor (default false ⇒ the
	// snapshot is unused and the behavior is byte-identical to the pilots-only floor).
	std::vector<std::complex<double> > seed_H;
	double seed_nv = noise_variance_estimate;   // the it=0 (TINTERP) nv — the warm seed
	if(dd_seed_floor)
	{
		seed_H.resize((size_t)Nsymb*Nc);
		for(int i=0;i<Nsymb*Nc;i++) seed_H[i] = (estimated_channel+i)->value;
	}

	// Pass 1: write raw per-cell H at PILOTs (rx/X) and reliable DATA cells.
	// Mark everything else UNKNOWN so the interpolators fill them.
	int pilot_index = 0;
	int data_index  = 0;   // walks DATA cells in the SAME raster order deframer uses
	for(int i=0;i<Nsymb;i++)
	{
		for(int j=0;j<Nc;j++)
		{
			int t = (ofdm_frame+i*Nc+j)->type;
			if(t==PILOT)
			{
				std::complex<double> X = pilot_configurator.sequence[pilot_index++];
				if(std::abs(X) > 1e-12)
				{
					(estimated_channel+i*Nc+j)->value  = *(in+i*Nc+j) / X;
					(estimated_channel+i*Nc+j)->status  = MEASURED;
				}
				else
				{
					(estimated_channel+i*Nc+j)->status  = UNKNOWN;
					(estimated_channel+i*Nc+j)->value   = 0;
				}
			}
			else if(t==DATA)
			{
				std::complex<double> xb = xbar[data_index];
				double vv = v[data_index];
				data_index++;
				double mag2 = xb.real()*xb.real() + xb.imag()*xb.imag();
				if(vv < conf_thresh && (mag2 + vv) > 1e-12)
				{
					// MMSE-style data-aided per-cell channel observation.
					std::complex<double> Hraw = (*(in+i*Nc+j) * std::conj(xb)) / (mag2 + vv);
					(estimated_channel+i*Nc+j)->value  = Hraw;
					(estimated_channel+i*Nc+j)->status  = MEASURED;
				}
				else if(dd_seed_floor)
				{
					// TINTERP-SEED floor: an uncertain data cell falls back to the
					// it=0 (TINTERP) seed H, NOT a cold pilots-only interpolation. This
					// keeps the warm seed under the dense data-aided anchor lattice (the
					// fix that stops the it=1 regression of a 5/6 TINTERP seed → 0/6).
					(estimated_channel+i*Nc+j)->value   = seed_H[(size_t)i*Nc+j];
					(estimated_channel+i*Nc+j)->status  = MEASURED;
				}
				else
				{
					(estimated_channel+i*Nc+j)->status  = UNKNOWN;   // interpolate
					(estimated_channel+i*Nc+j)->value   = 0;
				}
			}
			else
			{
				(estimated_channel+i*Nc+j)->status = UNKNOWN;
				(estimated_channel+i*Nc+j)->value  = 0;
			}
		}
	}

	// Pass 2: fill any UNKNOWN cell by interpolation. Per carrier, linear-interpolate
	// in TIME between MEASURED anchors (hold at edges); any carrier with no anchor is
	// freq-filled. This is the dense-lattice analog of LS's column interpolation, but
	// over the time-dense data-aided anchor set (the Nyquist-wall break).
	for(int j=0;j<Nc;j++)
	{
		// gather measured rows on this carrier
		int first=-1, last=-1;
		for(int n=0;n<Nsymb;n++)
			if((estimated_channel+n*Nc+j)->status==MEASURED){ if(first<0) first=n; last=n; }
		if(first<0) continue;   // no anchor on this carrier → handled by freq-fill below
		// hold at edges
		for(int n=0;n<first;n++)
			(estimated_channel+n*Nc+j)->value = (estimated_channel+first*Nc+j)->value;
		for(int n=last+1;n<Nsymb;n++)
			(estimated_channel+n*Nc+j)->value = (estimated_channel+last*Nc+j)->value;
		// linear interp between consecutive anchors
		int prev=first;
		for(int n=first+1;n<=last;n++)
		{
			if((estimated_channel+n*Nc+j)->status==MEASURED)
			{
				int n0=prev, n1=n;
				std::complex<double> h0=(estimated_channel+n0*Nc+j)->value;
				std::complex<double> h1=(estimated_channel+n1*Nc+j)->value;
				for(int m=n0+1;m<n1;m++)
				{
					double t = (n1>n0)? (double)(m-n0)/(double)(n1-n0) : 0.0;
					(estimated_channel+m*Nc+j)->value = h0*(1.0-t) + h1*t;
				}
				prev=n;
			}
		}
		for(int n=first;n<=last;n++)
			(estimated_channel+n*Nc+j)->status = MEASURED;
	}
	// Freq-fill any all-data carrier that ended up with no anchor (Dx=1 ⇒ none in
	// production, but keep for safety / thinned lattices).
	for(int n=0;n<Nsymb;n++)
	{
		int firstc=-1,lastc=-1;
		for(int j=0;j<Nc;j++)
			if((estimated_channel+n*Nc+j)->status==MEASURED){ if(firstc<0) firstc=j; lastc=j; }
		if(firstc<0) continue;
		for(int j=0;j<firstc;j++)
			(estimated_channel+n*Nc+j)->value = (estimated_channel+n*Nc+firstc)->value;
		for(int j=lastc+1;j<Nc;j++)
			(estimated_channel+n*Nc+j)->value = (estimated_channel+n*Nc+lastc)->value;
		int prevc=firstc;
		for(int j=firstc+1;j<=lastc;j++)
		{
			if((estimated_channel+n*Nc+j)->status==MEASURED)
			{
				std::complex<double> h0=(estimated_channel+n*Nc+prevc)->value;
				std::complex<double> h1=(estimated_channel+n*Nc+j)->value;
				for(int m=prevc+1;m<j;m++)
				{
					double t=(j>prevc)? (double)(m-prevc)/(double)(j-prevc):0.0;
					(estimated_channel+n*Nc+m)->value = h0*(1.0-t)+h1*t;
				}
				prevc=j;
			}
		}
		for(int j=0;j<Nc;j++)
			(estimated_channel+n*Nc+j)->status = MEASURED;
	}

	// DFT smoothing (same denoiser the LS path applies post-interp).
	smooth_channel_estimate_dft();

	// nv = pilot+reliable-data residual against the final H, FLOORED at the cross-pilot
	// AWGN estimate (I1). The data-aided residual can only ADD to the pre-EQ floor; it
	// cannot collapse nv below the true noise (the E1/cfg16-nvfix collapse class).
	{
		double nsum = 0.0; int ncnt = 0; int pidx = 0; int didx = 0;
		for(int i=0;i<Nsymb;i++)
		{
			for(int j=0;j<Nc;j++)
			{
				int t = (ofdm_frame+i*Nc+j)->type;
				if(t==PILOT)
				{
					std::complex<double> X = pilot_configurator.sequence[pidx++];
					std::complex<double> resid = *(in+i*Nc+j) - (estimated_channel+i*Nc+j)->value * X;
					nsum += resid.real()*resid.real() + resid.imag()*resid.imag();
					ncnt++;
				}
				else if(t==DATA)
				{
					std::complex<double> xb = xbar[didx];
					double vv = v[didx];
					didx++;
					if(vv < conf_thresh)
					{
						std::complex<double> resid = *(in+i*Nc+j) - (estimated_channel+i*Nc+j)->value * xb;
						nsum += resid.real()*resid.real() + resid.imag()*resid.imag();
						ncnt++;
					}
				}
			}
		}
		double nv_resid = (ncnt>0) ? nsum/(double)ncnt : 0.01;
		double nv_floor = estimate_noise_from_pilot_pairs(in);   // cross-pilot AWGN floor
		if(dd_seed_floor)
		{
			// TINTERP-SEED nv anchor (TURBO_EQ_VERDICT.md §5): on a fast (POOR/1 Hz)
			// fade the cross-pilot DIFFERENTIAL overcounts the Doppler-driven inter-
			// pilot variation as NOISE (measured ~0.40 vs the TINTERP-honest ~0.027 that
			// produced the 5/6 it=0 decode), which alone collapses the it=1 demod LLRs
			// → a 13× nv blow-up that reverts the warm seed. When seeding from TINTERP,
			// floor the nv at the SMALLER of the cross-pilot AWGN and the incoming
			// TINTERP nv (the honest post-EQ noise of the seed) so the refinement nv
			// never EXCEEDS the warm seed's nv. The data-aided residual can still RAISE
			// it if the data genuinely disagrees — improve-only, never collapse below
			// true noise (R3 / the 1e-6 floor preserved).
			double anchor = (seed_nv < nv_floor) ? seed_nv : nv_floor;
			nv_floor = anchor;
		}
		noise_variance_estimate = (nv_resid > nv_floor) ? nv_resid : nv_floor;
		if(noise_variance_estimate < 1e-6) noise_variance_estimate = 1e-6;
	}
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

// cfg16 DFT-smoother leakage mode selector (fix/cfg16-dft-leakage).
// Read ONCE into a function-local static (single-threaded per-frame call,
// never a hot inner-loop getenv). Controls smooth_channel_estimate_dft():
//   0 = LEGACY   : rectangular IFFT over active band only (the leaky version)
//   1 = EDGEEXT  : mirror-extend the band edges before the IFFT to kill the
//                  virtual-subcarrier boundary discontinuity (default; root fix)
//   2 = OFF      : skip the smoother entirely (A/B isolation only)
// Env MERCURY_DFTSMOOTH = legacy|edgeext|off overrides the default at runtime.
static int dftsmooth_mode()
{
	static const int mode = []() -> int {
		const char* e = std::getenv("MERCURY_DFTSMOOTH");
		if(e == nullptr) return 1;              // default: root fix ON
		if(strcmp(e, "off")    == 0 || strcmp(e, "2") == 0) return 2;
		if(strcmp(e, "legacy") == 0 || strcmp(e, "0") == 0) return 0;
		return 1;                               // edgeext / "1" / anything else
	}();
	return mode;
}

void cl_ofdm::smooth_channel_estimate_dft()
{
	// DFT-based channel estimation noise suppression.
	// Per-symbol: IFFT to time domain, window to keep GI-proportional taps,
	// FFT back to frequency domain. Suppresses estimation noise while
	// preserving real channel structure within the guard interval.
	// Ref: Edfors et al., "On Channel Estimation in OFDM Systems," VTC 1995.
	//
	// ROOT FIX (fix/cfg16-dft-leakage): the naive transform runs the IFFT over
	// ONLY the Nc=50 ACTIVE subcarriers, which sit embedded (with virtual/null
	// subcarriers around them) inside the Nfft=256 grid. The active band is a
	// RECTANGULAR SLICE of the true 256-bin frequency response; on a
	// frequency-selective channel H[0] != H[Nc-1], so this slice has a hard
	// discontinuity at its edges (periodic wrap). The IFFT of a discontinuous
	// sequence LEAKS energy across ALL time taps (Gibbs), and the rectangular
	// time-window [window_taps, Nc-window_taps) then zeroes taps that carry
	// GENUINE (leaked) channel energy -> an irreducible, SNR-independent bias
	// floor in H that GROWS with multipath delay (ripple rate). 32-QAM
	// (min-distance ~2x tighter than 16-QAM) slices this bias wrong -> the
	// cfg16 frequency-selective BER floor (uncoded 0.186 vs GENIE 0.0001).
	// Refs: Dong Li et al., "Enhanced DFT Interpolation-based Channel Estimation
	// for OFDM Systems with Virtual Subcarriers," IEEE VTC 2006 (the leakage
	// error floor + edge-processing fix); "A New DFT-Based Channel Estimation
	// Approach for OFDM with Virtual Subcarriers by Leakage Estimation," IEEE
	// Trans. (Yonsei) — virtual subcarriers break Fourier orthogonality -> leakage.
	//
	// FIX: mirror-extend (even reflection) the band edges before the IFFT. The
	// reflected sequence is continuous at the wrap boundary, so the leakage
	// (Gibbs ripple from the edge step) collapses and the true delay-limited CIR
	// is recovered inside window_taps. This is the low-complexity edge-processing
	// variant of EDFTI; it needs no knowledge of the null-band and is exact for a
	// smooth in-band response.
	int mode = dftsmooth_mode();
	// One-shot audit banner: makes the PRE-FIX/POST-FIX estimator arm visible in
	// the RX log (the MERCURY_DFTSMOOTH env is otherwise silent). Fires once.
	static bool banner_done = false;
	if(!banner_done)
	{
		banner_done = true;
		const char* mn = (mode == 0) ? "LEGACY(leaky)"
		               : (mode == 2) ? "OFF"
		                             : "EDGEEXT(rootfix)";
		printf("[EST-MODE] dftsmooth=%s LS_window_width=%d LS_window_hight=%d\n",
		       mn, LS_window_width, LS_window_hight);
		fflush(stdout);
	}
	if(mode == 2) return;                 // OFF: skip smoother (A/B isolation)

	if(Nc < 4) return;  // Too few subcarriers for meaningful smoothing

	// Window width: number of time-domain taps to keep on each side of DC.
	// gi = Ngi/Nfft. Channel delay spread fits within GI, so gi*Nc taps suffice.
	// Add margin of +2 for timing uncertainty and filter leakage.
	int window_taps = (int)(gi * Nc + 0.5) + 2;
	if(window_taps < 3) window_taps = 3;
	if(window_taps >= Nc / 2) return;  // Window too wide, smoothing won't help

	if(mode == 0)
	{
		// LEGACY leaky path (A/B baseline): rectangular IFFT over active band.
		std::complex<double>* buf_in = new std::complex<double>[Nc];
		std::complex<double>* buf_out = new std::complex<double>[Nc];
		for(int i = 0; i < Nsymb; i++)
		{
			for(int j = 0; j < Nc; j++)
				buf_in[j] = (estimated_channel + i*Nc + j)->value;
			ifft(buf_in, buf_out, Nc);
			for(int t = window_taps; t < Nc - window_taps; t++)
				buf_out[t] = std::complex<double>(0.0, 0.0);
			fft(buf_out, buf_in, Nc);
			for(int j = 0; j < Nc; j++)
				(estimated_channel + i*Nc + j)->value = buf_in[j];
		}
		delete[] buf_in;
		delete[] buf_out;
		return;
	}

	// EDGEEXT (default, root fix): even-reflect the active band on both sides so
	// the extended sequence is CONTINUOUS across the periodic wrap. Extending by
	// Next = Nc/2 on each side is ample for the mirror to bridge the edge step
	// without wrapping the reflected copies into each other. The IFFT is taken on
	// the length-Ne = Nc + 2*Next sequence; the time-window keeps the delay taps
	// (scaled to the extended length: the CIR length is fixed in samples, but the
	// oversampled-by-Ne/Nc grid stretches the tap index proportionally); the FFT
	// back is cropped to the central Nc bins (the original active band).
	int Next = Nc / 2;                    // reflection length per side
	int Ne   = Nc + 2 * Next;             // extended DFT length
	// The genuine CIR spans window_taps time samples on the ORIGINAL Nc-grid.
	// On the Ne-grid the same physical delay maps to window_taps*Ne/Nc taps;
	// round UP and keep the +? margin already folded into window_taps.
	int window_taps_e = (int)((double)window_taps * Ne / Nc + 0.5);
	if(window_taps_e < 3) window_taps_e = 3;
	if(window_taps_e >= Ne / 2)
	{
		// Extended window swallows the whole grid — smoothing is a no-op; fall
		// back to leaving the interpolated estimate untouched (safer than the
		// leaky rectangular path).
		return;
	}

	std::complex<double>* buf_in  = new std::complex<double>[Ne];
	std::complex<double>* buf_out = new std::complex<double>[Ne];

	for(int i = 0; i < Nsymb; i++)
	{
		// Center: the Nc active-band estimates.
		for(int j = 0; j < Nc; j++)
			buf_in[Next + j] = (estimated_channel + i*Nc + j)->value;

		// Left even reflection about the first sample: buf_in[Next-1-k]=H[k+1].
		// (Mirror the interior so the value AT the edge is not duplicated — a
		// half-sample even reflection, which removes the first-difference step
		// at the boundary.)
		for(int k = 0; k < Next; k++)
		{
			int src = k + 1;                 // 1..Next
			if(src > Nc - 1) src = Nc - 1;   // clamp for tiny Nc
			buf_in[Next - 1 - k] = (estimated_channel + i*Nc + src)->value;
		}
		// Right even reflection about the last sample: buf_in[Next+Nc+k]=H[Nc-2-k].
		for(int k = 0; k < Next; k++)
		{
			int src = Nc - 2 - k;            // Nc-2 .. Nc-1-Next
			if(src < 0) src = 0;             // clamp for tiny Nc
			buf_in[Next + Nc + k] = (estimated_channel + i*Nc + src)->value;
		}

		// IFFT of the continuous extended sequence -> leakage-free CIR.
		ifft(buf_in, buf_out, Ne);

		// Keep the delay-limited taps (both causal head and acausal tail), zero
		// the noise/leakage middle.
		for(int t = window_taps_e; t < Ne - window_taps_e; t++)
			buf_out[t] = std::complex<double>(0.0, 0.0);

		// FFT back to the extended frequency grid, then crop the central Nc bins.
		fft(buf_out, buf_in, Ne);
		for(int j = 0; j < Nc; j++)
			(estimated_channel + i*Nc + j)->value = buf_in[Next + j];
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

	// Partial selection sort: after iteration j, slot j holds the (j+1)-th
	// largest metric and corss_corr_loc[j] its sample position, so trial N
	// (location_to_return=N) returns the TRUE N-th-best peak. The previous form
	// copied vals[i] into slot j WITHOUT swapping the displaced value out of the
	// remaining range, so every trial whose index was <= the global-argmax index
	// re-found the SAME global maximum — the multi-trial SUBPEAK/SKIP-H retry
	// ladder could never reach a distinct sub-peak. Swap BOTH value and location
	// so each selected slot is removed from later scans. location_to_return=0 is
	// byte-identical to the old form (slot 0 still receives the global argmax);
	// corss_corr_loc[i] already equals i for every scanned position and unscanned
	// positions carry vals=0 so they never win.
	int nsort = (nTrials_max < size) ? nTrials_max : size;
	for(int j=0;j<nsort;j++)
	{
		int best_i = j;
		for(int i=j+1;i<size;i++)
		{
			if (corss_corr_vals[i] > corss_corr_vals[best_i])
				best_i = i;
		}
		if(best_i != j)
		{
			double tv = corss_corr_vals[j];
			corss_corr_vals[j] = corss_corr_vals[best_i];
			corss_corr_vals[best_i] = tv;
			int tl = corss_corr_loc[j];
			corss_corr_loc[j] = corss_corr_loc[best_i];
			corss_corr_loc[best_i] = tl;
		}
	}

	return_val=corss_corr_loc[location_to_return];
	return return_val;
/*
 * 	Ref: T. M. Schmidl and D. C. Cox, "Robust frequency and timing synchronization for OFDM," in IEEE Transactions on Communications, vol. 45, no. 12, pp. 1613-1621, Dec. 1997, doi: 10.1109/26.650240.
 *
 */
}

TimeSyncResult cl_ofdm::time_sync_preamble_with_metric(std::complex <double>*in, int size, int interpolation_rate, int location_to_return, int step, int nTrials_max, int nsym_override)
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

	// LEVER P: correlate over n_sym preamble symbols (MINI = 1) instead of the
	// configured full length. Without this the fine-sync template (4 symbols)
	// re-locks a 1-symbol MINI frame onto a data subpeak, corrupting the delay.
	int n_sym = preamble_configurator.Nsymb;
	if(nsym_override > 0)
	{
		n_sym = nsym_override;
		if(n_sym > preamble_configurator.Nsymb) n_sym = preamble_configurator.Nsymb;
		if(n_sym < 1) n_sym = 1;
	}

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

	for(int i=0;i<size-n_sym*(this->Ngi+this->Nfft)*interpolation_rate;i+=step)
	{
		for(int k=0;k<n_sym*(this->Ngi+this->Nfft)*interpolation_rate;k++)
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
		for(int l=0;l<n_sym;l++)
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
		// P1 energy-weighted plateau tie-break (default ON). The bare normalized
		// metric ties ~1.0 across the silence-cancellation plateau; the earliest tie
		// member (a k-symbol-early lock into a near-silent run-up) wins the strict->
		// selection sort below, pulling the fine lock into the run-up (mean|H|
		// collapse: the post-turnaround frame-0 wrong-lock). Weight the SELECTION
		// score by window energy (norm_a+norm_b), exactly as the coarse halfsym
		// detector does (weighted = metric*(A2+R)), so a mostly-silence candidate
		// loses to the full-energy true onset. result.correlation (unused by callers,
		// see the fn header) is unaffected. Default ON; MERCURY_MF_PLATEAU_TIEBREAK_
		// DEFEAT=1 restores the bare-metric baseline (byte-identical selection).
		// REFUTED EXPERIMENT (kept env-gated, DEFAULT OFF = byte-identical baseline):
		// energy-weighting the fine selection overshoots to the LATE/loud plateau edge
		// (+3 symbols on the vehicle, 0/60 decode). MERCURY_F0V_SITE8_ENERGY=1 re-enables
		// it for A/B reproduction only. Do NOT default-ON (it regressed the fine-timing lock).
		static const int site8_energy_weight = []{ const char* e=std::getenv("MERCURY_F0V_SITE8_ENERGY"); return (e&&*e)?atoi(e):0; }();
		corss_corr_vals[i]= site8_energy_weight ? corss_corr * (norm_a + norm_b) : corss_corr;
		corss_corr_loc[i]=i;
	}

	// Clamp location_to_return to valid range to prevent reading uninitialized sort entries
	if(location_to_return >= nTrials_max)
		location_to_return = nTrials_max - 1;

	// Partial selection sort: after iteration j, slot j holds the (j+1)-th
	// largest metric and corss_corr_loc[j] its sample position, so trial N
	// (location_to_return=N) returns the TRUE N-th-best peak. The previous form
	// copied vals[i] into slot j WITHOUT swapping the displaced value out of the
	// remaining range, so every trial whose index was <= the global-argmax index
	// re-found the SAME global maximum — the multi-trial SUBPEAK/SKIP-H retry
	// ladder could never reach a distinct sub-peak. Swap BOTH value and location
	// so each selected slot is removed from later scans. location_to_return=0 is
	// byte-identical to the old form (slot 0 still receives the global argmax);
	// corss_corr_loc[i] already equals i for every scanned position and unscanned
	// positions carry vals=0 so they never win.
	int nsort = (nTrials_max < size) ? nTrials_max : size;
	for(int j=0;j<nsort;j++)
	{
		int best_i = j;
		for(int i=j+1;i<size;i++)
		{
			if (corss_corr_vals[i] > corss_corr_vals[best_i])
				best_i = i;
		}
		if(best_i != j)
		{
			double tv = corss_corr_vals[j];
			corss_corr_vals[j] = corss_corr_vals[best_i];
			corss_corr_vals[best_i] = tv;
			int tl = corss_corr_loc[j];
			corss_corr_loc[j] = corss_corr_loc[best_i];
			corss_corr_loc[best_i] = tl;
		}
	}

	result.delay = corss_corr_loc[location_to_return];
	// Get the correlation value at the returned location
	max_correlation = corss_corr_vals[location_to_return];
	result.correlation = max_correlation;

	return result;
}

TimeSyncResult cl_ofdm::time_sync_preamble_halfsym(std::complex<double>* in, int size, int interpolation_rate, int step, double early_exit_metric, int nsym_override, bool earliest_relative)
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
	// LEVER P: correlate over nsym_override symbols when set (MINI preamble),
	// else the configured full preamble length. Clamp to [1, configured].
	int nsym = preamble_configurator.Nsymb;
	if(nsym_override > 0)
	{
		nsym = nsym_override;
		if(nsym > preamble_configurator.Nsymb) nsym = preamble_configurator.Nsymb;
		if(nsym < 1) nsym = 1;
	}
	int pream_len = nsym * Nofdm;

	TimeSyncResult result;
	result.delay = 0;
	result.correlation = 0.0;

	double best_weighted = -1.0;
	double best_normalized = 0.0;
	int best_pos = 0;

	// D3 earliest-relative select: record the normalized metric (and its energy floor)
	// at every scanned position so that, AFTER the global peak is known, we can return
	// the EARLIEST position whose metric >= early_exit_metric*best (scale-invariant).
	// Only allocated when requested -> stock callers pay nothing and stay byte-identical.
	std::vector<double> er_metric;
	std::vector<double> er_energy;
	std::vector<int>    er_pos;
	if(earliest_relative)
	{
		int n_scan = (size - pream_len) / (step > 0 ? step : 1) + 2;
		if(n_scan < 0) n_scan = 0;
		er_metric.reserve(n_scan);
		er_energy.reserve(n_scan);
		er_pos.reserve(n_scan);
	}

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

		if(earliest_relative)
		{
			er_metric.push_back(metric);
			er_energy.push_back(A2 + R);
			er_pos.push_back(d);
		}

		// Early exit: return the FIRST position where normalized metric
		// exceeds threshold. Energy floor rejects false peaks on digital
		// silence. This finds the earliest preamble in the buffer rather
		// than the strongest, preventing later frames from shadowing
		// earlier ones when multiple back-to-back frames are present.
		// SUPPRESSED in earliest_relative mode: that mode must finish the full
		// scan to learn the GLOBAL peak before choosing the earliest position
		// at >= a FRACTION of it (an absolute first-crossing would fire on the
		// metric ramp ~2 sym before the true peak / on a stale-ring sub-peak).
		if(!earliest_relative && early_exit_metric > 0.0 && metric >= early_exit_metric
			&& (A2 + R) > 1e-6)
		{
			result.delay = d;
			result.correlation = metric;
			return result;
		}
	}

	// D3 EARLIEST-RELATIVE select (mirrors time_sync_preamble_fft ofdm.cc:2866-2881):
	// the global energy-weighted argmax (best_pos) FALSE-LOCKS the freshest/loudest
	// LATER co-resident copy (a retransmission near the ring end) whose body tail is
	// future. Instead walk forward and return the EARLIEST position whose normalized
	// metric reaches early_exit_metric (default 0.5) of the GLOBAL best — the original
	// (earliest) block always crosses 50% of its own peak before any later copy, and a
	// data/silence sub-peak never reaches it (preamble metric ~Nsymb^2*E vs data
	// ~Nsymb*E random walk, the 4:1 ratio the FFT path documents). The energy floor
	// rejects silence. Scale-invariant: no dependence on the absolute correlation level.
	if(earliest_relative && best_normalized > 0.0)
	{
		double frac = (early_exit_metric > 0.0) ? early_exit_metric : 0.5;
		double thr  = frac * best_normalized;
		// The halfsym metric is ~1.0 across the WHOLE preamble plateau (MEASURED: a
		// ~870-sample / ~2.8-symbol plateau in the CFG16 K=8 acquisition), so the earliest
		// >= 0.5*peak crossing lands at the plateau's LEADING EDGE — up to a preamble-length
		// before the global argmax. The big-block caller's MF-snap (widened to ±(pre_nSymb+1)
		// symbols in earliest mode, telecom_system.cc bigblock_rx_passband) then refines this
		// edge to the true preamble start. A co-resident later copy is a full block_span away,
		// far beyond the plateau, so it is never the earliest crossing -> no false-lock.
		for(size_t i = 0; i < er_metric.size(); i++)
		{
			if(er_metric[i] >= thr && er_energy[i] > 1e-6)
			{
				result.delay = er_pos[i];
				result.correlation = er_metric[i];
				return result;
			}
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

TimeSyncResult cl_ofdm::time_sync_preamble_matched_local(
	std::complex<double>* baseband_interp, int buffer_size_interp,
	int interpolation_rate, int preamble_nSymb, int center, int half_window)
{
	TimeSyncResult result;
	result.delay = center;
	result.correlation = 0.0;
	if(ofdm_corr_template == NULL || ofdm_corr_template_len <= 0
		|| interpolation_rate <= 0 || half_window < 0)
		return result;

	const int Nofdm = Nfft + Ngi;
	const int template_nsymb = std::min(ofdm_corr_template_nsymb, preamble_nSymb);
	const int preamble_interp = template_nsymb * Nofdm * interpolation_rate;
	int first = std::max(0, center - half_window);
	int last = std::min(center + half_window, buffer_size_interp - preamble_interp);
	if(template_nsymb <= 0 || last < first)
		return result;

	double best_metric = -1.0;
	for(int pos = first; pos <= last; pos++)
	{
		double total_metric = 0.0;
		for(int sym = 0; sym < template_nsymb; sym++)
		{
			double corr_re = 0.0, corr_im = 0.0, rx_energy = 0.0;
			for(int n = 0; n < Nofdm; n++)
			{
				const std::complex<double> rx = baseband_interp[
					pos + (sym * Nofdm + n) * interpolation_rate];
				const std::complex<double> ref = ofdm_corr_template[sym * Nofdm + n];
				corr_re += ref.real() * rx.real() + ref.imag() * rx.imag();
				corr_im += ref.imag() * rx.real() - ref.real() * rx.imag();
				rx_energy += std::norm(rx);
			}
			const double denominator = ofdm_corr_template_sym_energy[sym] * rx_energy;
			if(denominator > 1e-30)
				total_metric += (corr_re * corr_re + corr_im * corr_im) / denominator;
		}
		if(total_metric > best_metric)
		{
			best_metric = total_metric;
			result.delay = pos;
			result.correlation = total_metric;
		}
	}
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
	int preamble_bins[48][4]; // [MAX_PREAMBLE_SYMB][MAX_STREAMS] — sized for the 48-symbol robust preamble
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
// Detect-both wrapper (NB robust-preamble capability negotiation): search the
// PRIMARY (active/negotiated) sequence first; on a miss, search the ALTERNATE
// set when configured. Reports which sequence matched via
// mfsk_matched_preamble_nsymb / mfsk_matched_alt so the extraction uses the
// per-frame preamble length. With no alternate configured this is exactly the
// single-sequence detector.
int cl_ofdm::time_sync_mfsk_corr(std::complex<double>* baseband_interp,
                                  int buffer_size_interp, int interpolation_rate,
                                  int search_start_symb, double* out_metric)
{
	mfsk_matched_preamble_nsymb = 0;
	mfsk_matched_alt = false;
	int d = time_sync_mfsk_corr_seq(baseband_interp, buffer_size_interp,
			interpolation_rate, search_start_symb, out_metric,
			mfsk_preamble_tones, mfsk_preamble_nsymb, mfsk_preamble_match_threshold);
	if (d >= 0)
	{
		mfsk_matched_preamble_nsymb = mfsk_preamble_nsymb;
		return d;
	}
	if (mfsk_alt_preamble_nsymb > 0)
	{
		double alt_metric = 0.0;
		int da = time_sync_mfsk_corr_seq(baseband_interp, buffer_size_interp,
				interpolation_rate, search_start_symb, &alt_metric,
				mfsk_alt_preamble_tones, mfsk_alt_preamble_nsymb, mfsk_alt_match_threshold);
		if (da >= 0)
		{
			mfsk_matched_preamble_nsymb = mfsk_alt_preamble_nsymb;
			mfsk_matched_alt = true;
			if (out_metric) *out_metric = alt_metric;
			return da;
		}
	}
	return d;   // primary miss decision count already in *out_metric
}

int cl_ofdm::time_sync_mfsk_corr_seq(std::complex<double>* baseband_interp,
                                  int buffer_size_interp, int interpolation_rate,
                                  int search_start_symb, double* out_metric,
                                  const int* pre_tones, int pre_nsymb, int pre_threshold)
{
	if (out_metric) *out_metric = 0.0;

	// Default-ON global: NOMIRROR disables the tone-space mirror-accept in the MFSK
	// acquisition detector (cuts the false-accept rate; a residual CFO does not populate
	// the mirror bin). Set MERCURY_MFSK_NOMIRROR=0 to restore the legacy mirror-accept.
	bool acq_nomirror = true;
	{ const char* _nm = std::getenv("MERCURY_MFSK_NOMIRROR"); if (_nm != NULL && atoi(_nm) == 0) acq_nomirror = false; }
	if (mfsk_M <= 0 || mfsk_nStreams <= 0 || pre_nsymb <= 0)
		return -1;
	if (pre_threshold <= 0)
		return -1;
	if (work_buf_a == NULL || work_buf_b == NULL || Nfft <= 0)
		return -1;

	int Nofdm = Nfft + Ngi;
	int sym_period_interp = Nofdm * interpolation_rate;
	if (sym_period_interp <= 0) return -1;
	int buffer_nsymb = buffer_size_interp / sym_period_interp;
	int preamble_n = pre_nsymb;
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
			int actual_tone = pre_tones[p % 48];
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
				                  (best_t == actual_tone || (!acq_nomirror && best_t == mirror_tone)));
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
				                  (peak_bin == expected_bin || (!acq_nomirror && peak_bin == mirror_bin)));
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

			int actual_tone = pre_tones[p % 48];
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
				             (best_t == actual_tone || (!acq_nomirror && best_t == mirror_tone)));
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
				symbol_ok = (pk > 0 && (pkbin == ebin || (!acq_nomirror && pkbin == mbin)));
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

	if (decision_matched < pre_threshold)
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

	// recovery-ack-capture LEVER 2 (data-flow-recovery-ack-capture.md §6): when
	// ack_allow_partial_tail is set (recovery-fine only), also test start positions
	// where the base block OVERSHOOTS the tail — the per-symbol loop below already
	// break's at the buffer edge, so a partial block is scored on the symbols PRESENT
	// (the LATE-TRUNCATED class, 68% of HW misses, where 0/32 hold a full 16-sym run).
	// reps>1 partial-tail is NOT supported (the combine reps must all fit); keep the
	// strict bound there. DEFAULT FALSE → s_max = the verbatim full-fit bound → the
	// loop is byte-identical for every caller.
	int s_max = buffer_nsymb - total_needed;
	if (ack_allow_partial_tail && combine_reps == 1)
		s_max = buffer_nsymb - 1;   // allow the last 15 partial-tail start positions

	// E4 detect-fft-memo (idle-CPU lever): the coarse loop below re-FFTs the SAME
	// symbol window for every start position s that spans it — for a symbol at
	// absolute index (s+p) the FFT input/output depend ONLY on (s+p), so each
	// distinct window is transformed up to ack_nsymb times. Idle undirected HAIL:
	// ack_nsymb=16, buffer_nsymb=40, s_max=24 -> 25x16=400 coarse FFTs over only
	// ~40 distinct windows (~10x redundancy; FFT is ~68% of idle cycles per the
	// efficiency profile). Precompute each distinct window's |FFT|^2 ONCE here and
	// have the coarse psp() read the cache: the power values are BIT-IDENTICAL, so
	// matched-count / metric / best_offset and the detect decision are unchanged.
	// Only the reps==1 path is memoized (the default ACK/HAIL/BREAK poll); the
	// reps>1 recovery-combining path and the fine (sub-symbol) pass are untouched.
	const bool memo_on = (combine_reps == 1) &&
		(detect_memo_force >= 0 ? (detect_memo_force != 0) : detect_fft_memo_enabled());
	int memo_max_sym = -1;
	if (memo_on)
	{
		// Highest absolute symbol index the coarse loop can reach whose full FFT
		// window still fits the buffer (mirrors the per-symbol break bound below).
		memo_max_sym = s_max + ack_nsymb - 1;
		int last_in_buf = buffer_nsymb - 1;
		if (memo_max_sym > last_in_buf) memo_max_sym = last_in_buf;
		size_t need = (size_t)(memo_max_sym + 1) * (size_t)Nfft;
		if (detect_memo_cap < need)
		{
			if (detect_memo_pow != NULL) { delete[] detect_memo_pow; detect_memo_pow = NULL; }
			detect_memo_pow = new double[need];   // grow-once; allocation-free after warmup
			detect_memo_cap = need;
		}
		for (int j = 0; j <= memo_max_sym; j++)
		{
			int off = j * sym_period_interp + Ngi * interpolation_rate;
			for (int i = 0; i < Nfft; i++)
				decimated_sym[i] = baseband_interp[off + i * interpolation_rate];
			fft(decimated_sym, fft_out, Nfft);
			detect_ack_fft_count++;
			double* row = detect_memo_pow + (size_t)j * Nfft;
			for (int b = 0; b < Nfft; b++)
				row[b] = fft_out[b].real() * fft_out[b].real() +
				         fft_out[b].imag() * fft_out[b].imag();
		}
	}

	for (int s = 0; s <= s_max; s++)
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
			else if (memo_on)
			{
				// per-symbol |FFT|^2 already cached in detect_memo_pow above
				// (byte-identical to the inline FFT); psp() reads the cache row.
			}
			else
			{
				for (int i = 0; i < Nfft; i++)
					decimated_sym[i] = baseband_interp[offset + i * interpolation_rate];
				fft(decimated_sym, fft_out, Nfft);
				detect_ack_fft_count++;
			}
			auto psp = [&](int b) -> double {
				if (combine_reps > 1) return pow_accum[b];
				if (memo_on) return detect_memo_pow[(size_t)sym_idx * Nfft + b];
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

			// recovery-ack-capture LEVER 2: under partial-tail, an OOB symbol STOPS the
			// per-symbol count but does NOT reject the candidate — the partial score (the
			// symbols PRESENT) is kept. partial_tail tracks "ran off the tail but score
			// the partial"; oob keeps the strict "reject" meaning for the default path.
			bool partial_tail = false;
			for (int p = 0; p < ack_nsymb && !oob; p++)
			{
				int offset = d + p * sym_period_interp + Ngi * interpolation_rate;
				if (offset + Nfft * interpolation_rate > buffer_size_interp)
				{
					if (ack_allow_partial_tail && combine_reps == 1)
					{
						partial_tail = true;   // stop counting; keep matched_f so far
						break;
					}
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
