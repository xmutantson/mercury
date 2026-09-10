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

// ===========================================================================
// PRECOOK M3 (copy-from-bundle) — deep-copy of the four geometry PHY classes.
// ===========================================================================
// Implements cl_ofdm / cl_pilot_configurator / cl_preamble_configurator /
// cl_FIR / cl_ldpc / cl_psk / cl_mfsk :: copy_from(). The M3 config-swap
// mechanism copies a fully-built (init()'d) source object into a destination
// without re-running init(), so a gearshift/config switch is a memcpy rather
// than a teardown+rebuild.
//
// SPEC: _research/_precook/BUNDLE_FIELD_CHECKLIST.md (PART 2/3 field list) +
// _research/_precook/PRECOOK_STAGE2_TURNKEY.md (STEP 1). The 3 landmines the
// checklist flags are all handled here:
//   L1 back-pointer aliases — pilot/preamble .carrier is a back-pointer into
//      the PARENT cl_ofdm's ofdm_frame/ofdm_preamble; the sub-object copy_from
//      leaves it NULL and cl_ofdm::copy_from RE-POINTS it at THIS's buffers.
//   L2 grow-as-needed scratch — p2b_*/tsync_*/b2p_* are ring-driven scratch that
//      self-grow on first RX use; copy_from sets each ptr NULL + *_size 0 so they
//      cleanly regrow under the new geometry (never copy the src's throwaway).
//   L3 ldpc QC matrices — QCmatrix{Enc,C,V,d} are pointers INTO const static
//      tables (mercury_normal_*); copy the POINTER, never alloc.
//
// Every deep-copy is guarded on (src ptr != NULL && size > 0); the dest owning
// pointer is freed first. The class ctors NULL-init every owning pointer
// (verified: ofdm.cc:133-198, ofdm.cc:1143/1360, fir_filter.cc:36, ldpc.cc:47-54,
// psk.cc:29-31) so a fresh-constructed dest has no stale pointer.

#include "physical_layer/ofdm.h"
#include "physical_layer/fir_filter.h"
#include "physical_layer/ldpc.h"
#include "physical_layer/psk.h"
#include "physical_layer/mfsk.h"
#include "physical_layer/physical_defines.h"
#include "debug/canary_guard.h"
#include <cstring>

// ---------------------------------------------------------------------------
// cl_FIR — owns filter_coefficients[filter_nTaps].
// ---------------------------------------------------------------------------
void cl_FIR::copy_from(const cl_FIR& s)
{
	if(this == &s) return;

	// Free the dest's owned buffer first (self-managed here — cl_ofdm::deinit
	// does NOT touch the FIR members, so copy_from is the one place they are
	// freed on a swap).
	if(filter_coefficients != NULL) { delete[] filter_coefficients; filter_coefficients = NULL; }

	// Scalars (filter_cut_frequency is private; member fn access OK).
	filter_window               = s.filter_window;
	filter_transition_bandwidth = s.filter_transition_bandwidth;
	filter_cut_frequency        = s.filter_cut_frequency;
	lpf_filter_cut_frequency    = s.lpf_filter_cut_frequency;
	hpf_filter_cut_frequency    = s.hpf_filter_cut_frequency;
	sampling_frequency          = s.sampling_frequency;
	type                        = s.type;
	filter_nTaps                = s.filter_nTaps;

	// Owning deep-copy.
	if(s.filter_coefficients != NULL && s.filter_nTaps > 0)
	{
		filter_coefficients = new double[s.filter_nTaps];
		memcpy(filter_coefficients, s.filter_coefficients, sizeof(double) * (size_t)s.filter_nTaps);
	}
}

// ---------------------------------------------------------------------------
// cl_psk — owns constellation[nSymbols], D_buf[nSymbols], LLR_buf[nBits].
// constellation content IS the geometry (Gray map + power-norm) -> memcpy.
// ---------------------------------------------------------------------------
void cl_psk::copy_from(const cl_psk& s)
{
	if(this == &s) return;

	deinit();  // frees + NULLs constellation / D_buf / LLR_buf

	nBits     = s.nBits;
	nSymbols  = s.nSymbols;
	var_floor = s.var_floor;

	if(s.constellation != NULL && s.nSymbols > 0)
	{
		constellation = new std::complex<double>[s.nSymbols];
		memcpy(constellation, s.constellation, sizeof(std::complex<double>) * (size_t)s.nSymbols);
	}
	if(s.D_buf != NULL && s.nSymbols > 0)
	{
		D_buf = new float[s.nSymbols];
		memcpy(D_buf, s.D_buf, sizeof(float) * (size_t)s.nSymbols);
	}
	if(s.LLR_buf != NULL && s.nBits > 0)
	{
		LLR_buf = new float[s.nBits];
		memcpy(LLR_buf, s.LLR_buf, sizeof(float) * (size_t)s.nBits);
	}
}

// ---------------------------------------------------------------------------
// cl_ldpc — QCmatrix* are POINTERS into const static tables (copy the pointer);
// R,Q[N*Vwidth] and V_pos[P*Cwidth] are owned workspace (content reset per
// decode, but memcpy'd for byte-parity). Runtime state is reset, not copied.
// ---------------------------------------------------------------------------
void cl_ldpc::copy_from(const cl_ldpc& s)
{
	if(this == &s) return;

	// Free owned workspace (leaves QCmatrix* alone — they alias read-only statics).
	CDELETE(R);
	CDELETE(Q);
	CDELETE(V_pos);

	// Public scalars.
	N                  = s.N;
	P                  = s.P;
	K                  = s.K;
	standard           = s.standard;
	framesize          = s.framesize;
	rate               = s.rate;
	decoding_algorithm = s.decoding_algorithm;
	GBF_eta            = s.GBF_eta;
	nIteration_max     = s.nIteration_max;
	print_nIteration   = s.print_nIteration;
	configuration      = s.configuration;
	last_decoder_kind  = LDPC_DECODER_SPA; // runtime observation, not bundle state

	// Private scalars.
	Cwidth                 = s.Cwidth;
	Vwidth                 = s.Vwidth;
	dwidth                 = s.dwidth;
	r                      = s.r;
	standard_val           = s.standard_val;
	decoding_algorithm_val = s.decoding_algorithm_val;
	eta_val                = s.eta_val;
	nIteration_max_val     = s.nIteration_max_val;
	print_nIteration_val   = s.print_nIteration_val;

	// L3: QC matrices are pointers into the compile-time const static tables
	// (mercury_normal_*_16.h) selected by K — copy the pointer, no alloc.
	QCmatrixEnc = s.QCmatrixEnc;
	QCmatrixC   = s.QCmatrixC;
	QCmatrixV   = s.QCmatrixV;
	QCmatrixd   = s.QCmatrixd;

	// Owned workspace: alloc to src size, memcpy content for byte-parity.
	if(s.R != NULL && s.N > 0 && s.Vwidth > 0)
	{
		R = CNEW(double, (size_t)s.N * (size_t)s.Vwidth, "ldpc.R.copy");
		memcpy(R, s.R, sizeof(double) * (size_t)s.N * (size_t)s.Vwidth);
	}
	if(s.Q != NULL && s.N > 0 && s.Vwidth > 0)
	{
		Q = CNEW(double, (size_t)s.N * (size_t)s.Vwidth, "ldpc.Q.copy");
		memcpy(Q, s.Q, sizeof(double) * (size_t)s.N * (size_t)s.Vwidth);
	}
	if(s.V_pos != NULL && s.P > 0 && s.Cwidth > 0)
	{
		V_pos = CNEW(int, (size_t)s.P * (size_t)s.Cwidth, "ldpc.V_pos.copy");
		memcpy(V_pos, s.V_pos, sizeof(int) * (size_t)s.P * (size_t)s.Cwidth);
	}

	// Runtime state — do NOT copy (the abort atomic ptr is per-decode).
	decode_abort           = nullptr;
	early_term_speculative = false;
	last_early_term_iter   = -1;
}

// ---------------------------------------------------------------------------
// cl_pilot_configurator — owns virtual_carrier[Nc_max*Nc_max] (priv) and
// sequence[nPilots]. ★ L1: carrier is a back-pointer alias into the PARENT's
// ofdm_frame — NOT copied here (left NULL; cl_ofdm::copy_from re-points it).
// ---------------------------------------------------------------------------
void cl_pilot_configurator::copy_from(const cl_pilot_configurator& s)
{
	if(this == &s) return;

	CDELETE(virtual_carrier);
	CDELETE(sequence);

	Dx            = s.Dx;
	Dy            = s.Dy;
	first_col     = s.first_col;
	second_col    = s.second_col;
	last_col      = s.last_col;
	first_row     = s.first_row;
	last_row      = s.last_row;
	nData         = s.nData;
	nPilots       = s.nPilots;
	nConfig       = s.nConfig;
	Nfft          = s.Nfft;
	Nc            = s.Nc;
	Nsymb         = s.Nsymb;
	Nc_max        = s.Nc_max;
	modulation    = s.modulation;
	seed          = s.seed;
	boost         = s.boost;
	print_on      = s.print_on;
	pilot_density = s.pilot_density;
	sparse_wide_data_carriers = s.sparse_wide_data_carriers;
	start_shift   = s.start_shift;

	// ★ L1: do NOT copy s.carrier (a back-pointer into the src's ofdm_frame).
	// The parent cl_ofdm::copy_from re-points this at THIS's ofdm_frame.
	carrier = NULL;

	if(s.virtual_carrier != NULL && s.Nc_max > 0)
	{
		virtual_carrier = CNEW(struct st_carrier, (size_t)s.Nc_max * (size_t)s.Nc_max, "pilot.virtual_carrier.copy");
		memcpy(virtual_carrier, s.virtual_carrier, sizeof(struct st_carrier) * (size_t)s.Nc_max * (size_t)s.Nc_max);
	}
	if(s.sequence != NULL && s.nPilots > 0)
	{
		sequence = CNEW(std::complex<double>, s.nPilots, "pilot.sequence.copy");
		memcpy(sequence, s.sequence, sizeof(std::complex<double>) * (size_t)s.nPilots);
	}
}

// ---------------------------------------------------------------------------
// cl_preamble_configurator — owns sequence[Nsymb*Nc]. ★ L1: carrier is a
// back-pointer alias into the PARENT's ofdm_preamble — not copied (re-pointed
// by cl_ofdm::copy_from).
// ---------------------------------------------------------------------------
void cl_preamble_configurator::copy_from(const cl_preamble_configurator& s)
{
	if(this == &s) return;

	CDELETE(sequence);

	nZeros              = s.nZeros;
	nPreamble           = s.nPreamble;
	Nfft                = s.Nfft;
	Nc                  = s.Nc;
	Nsymb               = s.Nsymb;
	nIdentical_sections = s.nIdentical_sections;
	modulation          = s.modulation;
	seed                = s.seed;
	boost               = s.boost;
	print_on            = s.print_on;
	start_shift         = s.start_shift;

	// ★ L1: parent re-points carrier at THIS's ofdm_preamble.
	carrier = NULL;

	if(s.sequence != NULL && s.Nsymb > 0 && s.Nc > 0)
	{
		sequence = CNEW(std::complex<double>, (size_t)s.Nsymb * (size_t)s.Nc, "preamble.sequence.copy");
		memcpy(sequence, s.sequence, sizeof(std::complex<double>) * (size_t)s.Nsymb * (size_t)s.Nc);
	}
}

// ---------------------------------------------------------------------------
// cl_ofdm — the highest-risk deep-copy. Frees the dest via deinit() (which does
// NOT touch the FIR members — those self-manage in cl_FIR::copy_from), copies
// scalars, deep-copies every owning buffer, copies the nested sub-objects, then
// RE-POINTS the pilot/preamble back-pointers (L1).
// ---------------------------------------------------------------------------
void cl_ofdm::copy_from(const cl_ofdm& s)
{
	if(this == &s) return;

	// Free everything the dest owns EXCEPT the FIRs (deinit() frees ofdm_frame,
	// ofdm_preamble, estimated_channel*, fft tables, work bufs, templates, and
	// the grow-as-needed scratch -> NULL+0; it also deinit()s the pilot/preamble
	// configurators). The FIRs are re-copied via cl_FIR::copy_from below.
	this->deinit();

	// ---- 2.1 scalars ----
	Nfft   = s.Nfft;
	Nc     = s.Nc;
	Nsymb  = s.Nsymb;
	gi     = s.gi;
	Ngi    = s.Ngi;                 // private
	start_shift            = s.start_shift;
	passband_start_sample  = s.passband_start_sample;
	time_sync_Nsymb        = s.time_sync_Nsymb;
	freq_offset_ignore_limit = s.freq_offset_ignore_limit;
	preamble_papr_cut      = s.preamble_papr_cut;
	data_papr_cut          = s.data_papr_cut;
	channel_estimator      = s.channel_estimator;
	channel_estimator_amplitude_restoration = s.channel_estimator_amplitude_restoration;
	LS_window_width        = s.LS_window_width;
	LS_window_hight        = s.LS_window_hight;
	noise_variance_estimate = s.noise_variance_estimate;
	tinterp_smooth_halfwin = s.tinterp_smooth_halfwin;
	dd_data_conf_thresh    = s.dd_data_conf_thresh;
	dd_seed_floor          = s.dd_seed_floor;
	ls_nv_debug_enabled    = s.ls_nv_debug_enabled;
	ls_use_crosspilot_nv   = s.ls_use_crosspilot_nv;
	fft_twiddle_size       = s.fft_twiddle_size;   // private

	// MFSK mirror scalars + fixed arrays.
	mfsk_M                       = s.mfsk_M;
	mfsk_nStreams                = s.mfsk_nStreams;
	mfsk_preamble_nsymb          = s.mfsk_preamble_nsymb;
	mfsk_preamble_match_threshold = s.mfsk_preamble_match_threshold;
	memcpy(mfsk_stream_offsets, s.mfsk_stream_offsets, sizeof(mfsk_stream_offsets));
	memcpy(mfsk_preamble_tones,  s.mfsk_preamble_tones,  sizeof(mfsk_preamble_tones));
	// Detect-both alternate set + last-match report (NB robust-preamble
	// capability negotiation) — must ride the swap or a precooked config
	// loses the alternate detector arm.
	mfsk_alt_preamble_nsymb  = s.mfsk_alt_preamble_nsymb;
	mfsk_alt_match_threshold = s.mfsk_alt_match_threshold;
	memcpy(mfsk_alt_preamble_tones, s.mfsk_alt_preamble_tones, sizeof(mfsk_alt_preamble_tones));
	mfsk_matched_preamble_nsymb = s.mfsk_matched_preamble_nsymb;
	mfsk_matched_alt            = s.mfsk_matched_alt;

	// Template scalars + fixed arrays.
	mfsk_corr_template_len    = s.mfsk_corr_template_len;
	mfsk_corr_template_energy = s.mfsk_corr_template_energy;
	mfsk_corr_template_nsymb  = s.mfsk_corr_template_nsymb;
	memcpy(mfsk_corr_template_sym_energy, s.mfsk_corr_template_sym_energy, sizeof(mfsk_corr_template_sym_energy));
	ofdm_corr_template_len    = s.ofdm_corr_template_len;
	ofdm_corr_template_nsymb  = s.ofdm_corr_template_nsymb;
	ofdm_corr_template_energy = s.ofdm_corr_template_energy;
	memcpy(ofdm_corr_template_sym_energy, s.ofdm_corr_template_sym_energy, sizeof(ofdm_corr_template_sym_energy));

	// ---- 2.2 owning pointers (deep-copy; guard src!=NULL && size>0) ----
	if(s.ofdm_frame != NULL && s.Nsymb > 0 && s.Nc > 0)
	{
		ofdm_frame = CNEW(struct st_carrier, (size_t)s.Nsymb * (size_t)s.Nc, "ofdm.ofdm_frame.copy");
		memcpy(ofdm_frame, s.ofdm_frame, sizeof(struct st_carrier) * (size_t)s.Nsymb * (size_t)s.Nc);
	}
	// ofdm_preamble is sized by preamble_configurator.Nsymb * Nc (read from src).
	{
		int pre_nsymb = s.preamble_configurator.Nsymb;
		if(s.ofdm_preamble != NULL && pre_nsymb > 0 && s.Nc > 0)
		{
			ofdm_preamble = CNEW(struct st_carrier, (size_t)pre_nsymb * (size_t)s.Nc, "ofdm.ofdm_preamble.copy");
			memcpy(ofdm_preamble, s.ofdm_preamble, sizeof(struct st_carrier) * (size_t)pre_nsymb * (size_t)s.Nc);
		}
	}
	if(s.estimated_channel != NULL && s.Nsymb > 0 && s.Nc > 0)
	{
		estimated_channel = CNEW(struct st_channel_complex, (size_t)s.Nsymb * (size_t)s.Nc, "ofdm.estimated_channel.copy");
		memcpy(estimated_channel, s.estimated_channel, sizeof(struct st_channel_complex) * (size_t)s.Nsymb * (size_t)s.Nc);
	}
	if(s.estimated_channel_without_amplitude_restoration != NULL && s.Nsymb > 0 && s.Nc > 0)
	{
		estimated_channel_without_amplitude_restoration = CNEW(struct st_channel_complex, (size_t)s.Nsymb * (size_t)s.Nc, "ofdm.est_channel_noamp.copy");
		memcpy(estimated_channel_without_amplitude_restoration, s.estimated_channel_without_amplitude_restoration,
		       sizeof(struct st_channel_complex) * (size_t)s.Nsymb * (size_t)s.Nc);
	}
	if(s.mfsk_corr_template != NULL && s.mfsk_corr_template_len > 0)
	{
		mfsk_corr_template = CNEW(std::complex<double>, s.mfsk_corr_template_len, "ofdm.mfsk_corr_template.copy");
		memcpy(mfsk_corr_template, s.mfsk_corr_template, sizeof(std::complex<double>) * (size_t)s.mfsk_corr_template_len);
	}
	if(s.ofdm_corr_template != NULL && s.ofdm_corr_template_len > 0)
	{
		ofdm_corr_template = CNEW(std::complex<double>, s.ofdm_corr_template_len, "ofdm.ofdm_corr_template.copy");
		memcpy(ofdm_corr_template, s.ofdm_corr_template, sizeof(std::complex<double>) * (size_t)s.ofdm_corr_template_len);
	}

	// FFT tables (private): fft_twiddle[Nfft/2], fft_scratch[Nfft], fft_bit_rev[Nfft].
	if(s.fft_twiddle != NULL && s.Nfft > 0)
	{
		fft_twiddle = CNEW(std::complex<double>, s.Nfft / 2, "ofdm.fft_twiddle.copy");
		memcpy(fft_twiddle, s.fft_twiddle, sizeof(std::complex<double>) * (size_t)(s.Nfft / 2));
	}
	if(s.fft_scratch != NULL && s.Nfft > 0)
	{
		fft_scratch = CNEW(std::complex<double>, s.Nfft, "ofdm.fft_scratch.copy");
		memcpy(fft_scratch, s.fft_scratch, sizeof(std::complex<double>) * (size_t)s.Nfft);
	}
	if(s.fft_bit_rev != NULL && s.Nfft > 0)
	{
		fft_bit_rev = CNEW(int, s.Nfft, "ofdm.fft_bit_rev.copy");
		memcpy(fft_bit_rev, s.fft_bit_rev, sizeof(int) * (size_t)s.Nfft);
	}

	// Nfft-sized scratch (zero_padded/iffted/gi_removed/ffted + work_buf_a/b).
	if(s.zero_padded_data != NULL && s.Nfft > 0)
	{
		zero_padded_data = CNEW(std::complex<double>, s.Nfft, "ofdm.zero_padded_data.copy");
		memcpy(zero_padded_data, s.zero_padded_data, sizeof(std::complex<double>) * (size_t)s.Nfft);
	}
	if(s.iffted_data != NULL && s.Nfft > 0)
	{
		iffted_data = CNEW(std::complex<double>, s.Nfft, "ofdm.iffted_data.copy");
		memcpy(iffted_data, s.iffted_data, sizeof(std::complex<double>) * (size_t)s.Nfft);
	}
	if(s.gi_removed_data != NULL && s.Nfft > 0)
	{
		gi_removed_data = CNEW(std::complex<double>, s.Nfft, "ofdm.gi_removed_data.copy");
		memcpy(gi_removed_data, s.gi_removed_data, sizeof(std::complex<double>) * (size_t)s.Nfft);
	}
	if(s.ffted_data != NULL && s.Nfft > 0)
	{
		ffted_data = CNEW(std::complex<double>, s.Nfft, "ofdm.ffted_data.copy");
		memcpy(ffted_data, s.ffted_data, sizeof(std::complex<double>) * (size_t)s.Nfft);
	}
	if(s.work_buf_a != NULL && s.Nfft > 0)
	{
		work_buf_a = CNEW(std::complex<double>, s.Nfft, "ofdm.work_buf_a.copy");
		memcpy(work_buf_a, s.work_buf_a, sizeof(std::complex<double>) * (size_t)s.Nfft);
	}
	if(s.work_buf_b != NULL && s.Nfft > 0)
	{
		work_buf_b = CNEW(std::complex<double>, s.Nfft, "ofdm.work_buf_b.copy");
		memcpy(work_buf_b, s.work_buf_b, sizeof(std::complex<double>) * (size_t)s.Nfft);
	}

	// ---- 2.3 L2: grow-as-needed scratch -> NULL + size 0 (deinit() already did
	// this; set it explicitly so the invariant is local and obvious). These
	// regrow under the new geometry on first RX use.
	p2b_l_data = NULL; p2b_data_filtered = NULL; p2b_buffer_size = 0;
	tsync_corr_loc = NULL; tsync_corr_vals = NULL; tsync_corr_size = 0;
	tsync_data = NULL; tsync_data_size = 0;
	b2p_data_interpolated = NULL; b2p_buffer_size = 0;

	// ---- 2.4 nested sub-objects ----
	pilot_configurator.copy_from(s.pilot_configurator);
	preamble_configurator.copy_from(s.preamble_configurator);
	FIR_rx_data.copy_from(s.FIR_rx_data);
	FIR_rx_time_sync.copy_from(s.FIR_rx_time_sync);
	FIR_tx1.copy_from(s.FIR_tx1);
	FIR_tx2.copy_from(s.FIR_tx2);

	// ★ L1: re-point the back-pointer aliases at THIS's owning frames.
	pilot_configurator.carrier    = ofdm_frame;
	preamble_configurator.carrier = ofdm_preamble;
}

// ===========================================================================
// PRECOOK gate helpers — byte-compare owning buffers between an init()-built
// object and its copy_from() copy (BUNDLE_FIELD_CHECKLIST PART 6). Each returns
// NULL on byte-identical, else the name of the first differing field so the
// --test gate localizes a missed field to a single buffer.
// ===========================================================================

const char* cl_FIR::precook_deep_equal(const cl_FIR& o) const
{
	if(filter_nTaps != o.filter_nTaps) return "FIR.filter_nTaps";
	if((filter_coefficients == NULL) != (o.filter_coefficients == NULL)) return "FIR.filter_coefficients(null-mismatch)";
	if(filter_coefficients != NULL && filter_nTaps > 0)
		if(memcmp(filter_coefficients, o.filter_coefficients, sizeof(double) * (size_t)filter_nTaps) != 0)
			return "FIR.filter_coefficients";
	return NULL;
}

const char* cl_psk::precook_deep_equal(const cl_psk& o) const
{
	if(nSymbols != o.nSymbols) return "psk.nSymbols";
	if(nBits != o.nBits) return "psk.nBits";
	if((constellation == NULL) != (o.constellation == NULL)) return "psk.constellation(null-mismatch)";
	if(constellation != NULL && nSymbols > 0)
		if(memcmp(constellation, o.constellation, sizeof(std::complex<double>) * (size_t)nSymbols) != 0)
			return "psk.constellation";
	if((D_buf == NULL) != (o.D_buf == NULL)) return "psk.D_buf(null-mismatch)";
	if(D_buf != NULL && nSymbols > 0)
		if(memcmp(D_buf, o.D_buf, sizeof(float) * (size_t)nSymbols) != 0)
			return "psk.D_buf";
	if((LLR_buf == NULL) != (o.LLR_buf == NULL)) return "psk.LLR_buf(null-mismatch)";
	if(LLR_buf != NULL && nBits > 0)
		if(memcmp(LLR_buf, o.LLR_buf, sizeof(float) * (size_t)nBits) != 0)
			return "psk.LLR_buf";
	return NULL;
}

const char* cl_ldpc::precook_deep_equal(const cl_ldpc& o) const
{
	if(N != o.N)             return "ldpc.N";
	if(P != o.P)             return "ldpc.P";
	if(K != o.K)             return "ldpc.K";
	if(configuration != o.configuration) return "ldpc.configuration";
	if(Cwidth != o.Cwidth)   return "ldpc.Cwidth";
	if(Vwidth != o.Vwidth)   return "ldpc.Vwidth";
	if(dwidth != o.dwidth)   return "ldpc.dwidth";
	// QC matrices must be the SAME pointer (shared const static tables).
	if(QCmatrixEnc != o.QCmatrixEnc) return "ldpc.QCmatrixEnc(ptr)";
	if(QCmatrixC   != o.QCmatrixC)   return "ldpc.QCmatrixC(ptr)";
	if(QCmatrixV   != o.QCmatrixV)   return "ldpc.QCmatrixV(ptr)";
	if(QCmatrixd   != o.QCmatrixd)   return "ldpc.QCmatrixd(ptr)";
	if((R == NULL) != (o.R == NULL)) return "ldpc.R(null-mismatch)";
	if(R != NULL && N > 0 && Vwidth > 0)
		if(memcmp(R, o.R, sizeof(double) * (size_t)N * (size_t)Vwidth) != 0) return "ldpc.R";
	if((Q == NULL) != (o.Q == NULL)) return "ldpc.Q(null-mismatch)";
	if(Q != NULL && N > 0 && Vwidth > 0)
		if(memcmp(Q, o.Q, sizeof(double) * (size_t)N * (size_t)Vwidth) != 0) return "ldpc.Q";
	if((V_pos == NULL) != (o.V_pos == NULL)) return "ldpc.V_pos(null-mismatch)";
	if(V_pos != NULL && P > 0 && Cwidth > 0)
		if(memcmp(V_pos, o.V_pos, sizeof(int) * (size_t)P * (size_t)Cwidth) != 0) return "ldpc.V_pos";
	return NULL;
}

const char* cl_pilot_configurator::precook_deep_equal(const cl_pilot_configurator& o) const
{
	if(Nc_max != o.Nc_max)   return "pilot.Nc_max";
	if(nPilots != o.nPilots) return "pilot.nPilots";
	if(sparse_wide_data_carriers != o.sparse_wide_data_carriers) return "pilot.sparse_wide_data_carriers";
	if((virtual_carrier == NULL) != (o.virtual_carrier == NULL)) return "pilot.virtual_carrier(null-mismatch)";
	if(virtual_carrier != NULL && Nc_max > 0)
		if(memcmp(virtual_carrier, o.virtual_carrier, sizeof(struct st_carrier) * (size_t)Nc_max * (size_t)Nc_max) != 0)
			return "pilot.virtual_carrier";
	if((sequence == NULL) != (o.sequence == NULL)) return "pilot.sequence(null-mismatch)";
	if(sequence != NULL && nPilots > 0)
		if(memcmp(sequence, o.sequence, sizeof(std::complex<double>) * (size_t)nPilots) != 0)
			return "pilot.sequence";
	return NULL;
}

const char* cl_preamble_configurator::precook_deep_equal(const cl_preamble_configurator& o) const
{
	if(Nsymb != o.Nsymb) return "preamble.Nsymb";
	if(Nc != o.Nc)       return "preamble.Nc";
	if((sequence == NULL) != (o.sequence == NULL)) return "preamble.sequence(null-mismatch)";
	if(sequence != NULL && Nsymb > 0 && Nc > 0)
		if(memcmp(sequence, o.sequence, sizeof(std::complex<double>) * (size_t)Nsymb * (size_t)Nc) != 0)
			return "preamble.sequence";
	return NULL;
}

const char* cl_ofdm::precook_deep_equal(const cl_ofdm& o) const
{
	if(Nfft != o.Nfft)   return "ofdm.Nfft";
	if(Nc != o.Nc)       return "ofdm.Nc";
	if(Nsymb != o.Nsymb) return "ofdm.Nsymb";

	if((ofdm_frame == NULL) != (o.ofdm_frame == NULL)) return "ofdm.ofdm_frame(null-mismatch)";
	if(ofdm_frame != NULL && Nsymb > 0 && Nc > 0)
		if(memcmp(ofdm_frame, o.ofdm_frame, sizeof(struct st_carrier) * (size_t)Nsymb * (size_t)Nc) != 0)
			return "ofdm.ofdm_frame";

	{
		int pre_nsymb = preamble_configurator.Nsymb;
		if((ofdm_preamble == NULL) != (o.ofdm_preamble == NULL)) return "ofdm.ofdm_preamble(null-mismatch)";
		if(preamble_configurator.Nsymb != o.preamble_configurator.Nsymb)
			return "preamble.Nsymb";
		if(ofdm_preamble != NULL && pre_nsymb > 0 && Nc > 0)
			if(memcmp(ofdm_preamble, o.ofdm_preamble, sizeof(struct st_carrier) * (size_t)pre_nsymb * (size_t)Nc) != 0)
				return "ofdm.ofdm_preamble";
	}

	if((estimated_channel == NULL) != (o.estimated_channel == NULL)) return "ofdm.estimated_channel(null-mismatch)";
	if(estimated_channel != NULL && Nsymb > 0 && Nc > 0)
		if(memcmp(estimated_channel, o.estimated_channel, sizeof(struct st_channel_complex) * (size_t)Nsymb * (size_t)Nc) != 0)
			return "ofdm.estimated_channel";

	if((estimated_channel_without_amplitude_restoration == NULL) != (o.estimated_channel_without_amplitude_restoration == NULL))
		return "ofdm.est_channel_noamp(null-mismatch)";
	if(estimated_channel_without_amplitude_restoration != NULL && Nsymb > 0 && Nc > 0)
		if(memcmp(estimated_channel_without_amplitude_restoration, o.estimated_channel_without_amplitude_restoration,
		          sizeof(struct st_channel_complex) * (size_t)Nsymb * (size_t)Nc) != 0)
			return "ofdm.est_channel_noamp";

	if(mfsk_corr_template_len != o.mfsk_corr_template_len) return "ofdm.mfsk_corr_template_len";
	if((mfsk_corr_template == NULL) != (o.mfsk_corr_template == NULL)) return "ofdm.mfsk_corr_template(null-mismatch)";
	if(mfsk_corr_template != NULL && mfsk_corr_template_len > 0)
		if(memcmp(mfsk_corr_template, o.mfsk_corr_template, sizeof(std::complex<double>) * (size_t)mfsk_corr_template_len) != 0)
			return "ofdm.mfsk_corr_template";

	if(ofdm_corr_template_len != o.ofdm_corr_template_len) return "ofdm.ofdm_corr_template_len";
	if((ofdm_corr_template == NULL) != (o.ofdm_corr_template == NULL)) return "ofdm.ofdm_corr_template(null-mismatch)";
	if(ofdm_corr_template != NULL && ofdm_corr_template_len > 0)
		if(memcmp(ofdm_corr_template, o.ofdm_corr_template, sizeof(std::complex<double>) * (size_t)ofdm_corr_template_len) != 0)
			return "ofdm.ofdm_corr_template";

	// FFT tables.
	if((fft_twiddle == NULL) != (o.fft_twiddle == NULL)) return "ofdm.fft_twiddle(null-mismatch)";
	if(fft_twiddle != NULL && Nfft > 0)
		if(memcmp(fft_twiddle, o.fft_twiddle, sizeof(std::complex<double>) * (size_t)(Nfft / 2)) != 0)
			return "ofdm.fft_twiddle";
	if((fft_scratch == NULL) != (o.fft_scratch == NULL)) return "ofdm.fft_scratch(null-mismatch)";
	if(fft_scratch != NULL && Nfft > 0)
		if(memcmp(fft_scratch, o.fft_scratch, sizeof(std::complex<double>) * (size_t)Nfft) != 0)
			return "ofdm.fft_scratch";
	if((fft_bit_rev == NULL) != (o.fft_bit_rev == NULL)) return "ofdm.fft_bit_rev(null-mismatch)";
	if(fft_bit_rev != NULL && Nfft > 0)
		if(memcmp(fft_bit_rev, o.fft_bit_rev, sizeof(int) * (size_t)Nfft) != 0)
			return "ofdm.fft_bit_rev";

	// Nested sub-objects.
	const char* sub;
	if((sub = pilot_configurator.precook_deep_equal(o.pilot_configurator)) != NULL)       return sub;
	if((sub = preamble_configurator.precook_deep_equal(o.preamble_configurator)) != NULL) return sub;
	if(FIR_rx_data.precook_deep_equal(o.FIR_rx_data) != NULL)           return "ofdm.FIR_rx_data.filter_coefficients";
	if(FIR_rx_time_sync.precook_deep_equal(o.FIR_rx_time_sync) != NULL) return "ofdm.FIR_rx_time_sync.filter_coefficients";
	if(FIR_tx1.precook_deep_equal(o.FIR_tx1) != NULL)                   return "ofdm.FIR_tx1.filter_coefficients";
	if(FIR_tx2.precook_deep_equal(o.FIR_tx2) != NULL)                   return "ofdm.FIR_tx2.filter_coefficients";

	return NULL;
}
