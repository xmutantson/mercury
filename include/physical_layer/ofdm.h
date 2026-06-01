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

#ifndef INC_OFDM_H_
#define INC_OFDM_H_

#include <complex>
#include <math.h>
#include <iostream>
#include <cstdint>
#include "misc.h"
#include "physical_defines.h"
#include "fir_filter.h"
#include "plot.h"
#include "psk.h"
#include "interpolator.h"

// Result structure for time sync that includes correlation quality
struct TimeSyncResult {
	int delay;           // Sample delay to preamble start
	double correlation;  // Normalized correlation (0.0 to 1.0)
};


class cl_pilot_configurator
{

public:
	cl_pilot_configurator();
	~cl_pilot_configurator();
	void configure();
	void init(int Nfft, int Nc, int Nsymb, struct st_carrier* _carrier, int start_shift);
	void deinit();
	void print();

	int Dx,Dy;
	int first_col,second_col,last_col,first_row,last_row;
	int nData,nPilots,nConfig;
	int Nfft, Nc, Nsymb, Nc_max;
	int modulation;
	int seed;
	std::complex <double> *sequence;
	double boost;
	struct st_carrier* carrier;
	int print_on;
	int pilot_density;

private:
	struct st_carrier* virtual_carrier;
	int start_shift;


};

class cl_preamble_configurator
{

public:
	cl_preamble_configurator();
	~cl_preamble_configurator();
	void configure();
	void init(int Nfft, int Nc, struct st_carrier* _carrier, int start_shift);
	void deinit();
	void print();

	int nZeros,nPreamble;
	int Nfft, Nc, Nsymb, nIdentical_sections;
	int modulation;
	int seed;
	std::complex <double> *sequence;
	double boost;
	struct st_carrier* carrier;
	int print_on;

private:
	int start_shift;


};

class cl_ofdm
{
private:

	int Ngi;
	void zero_padder(std::complex <double>* in, std::complex <double>* out);
	void zero_depadder(std::complex <double>* in, std::complex <double>* out);
	void gi_adder(std::complex <double>* in, std::complex <double>* out);
	void gi_remover(std::complex <double>* in, std::complex <double>* out);
	void _fft(std::complex <double> *v, int n);
	void _ifft(std::complex <double> *v, int n);
	void _fft_fast(std::complex <double> *v, int n);
	void _ifft_fast(std::complex <double> *v, int n);
	void fft(std::complex <double>* in, std::complex <double>* out);
	void ifft(std::complex <double>* in, std::complex <double>* out);
	void ifft(std::complex <double>* in, std::complex <double>* out, int _Nfft);

	// Optimized FFT: precomputed twiddle factors
	std::complex<double>* fft_twiddle;     // Twiddle factors for FFT
	std::complex<double>* fft_scratch;     // Scratch buffer for in-place FFT
	int* fft_bit_rev;                      // Bit-reversal permutation table
	int fft_twiddle_size;                  // Size of twiddle table
	void init_fft_tables(int n);
	void deinit_fft_tables();

	std::complex <double> *zero_padded_data,*iffted_data;
	std::complex <double> *gi_removed_data,*ffted_data;

public:
	cl_ofdm();
	~cl_ofdm();
	void init();
	void init(int Nfft, int Nc, int Nsymb, float gi);
	void deinit();
	void symbol_mod(std::complex <double>*in, std::complex <double>*out);
	void symbol_demod(std::complex <double>*in, std::complex <double>*out);
	void framer(std::complex <double>* in, std::complex <double>* out);
	void deframer(std::complex <double>* in, std::complex <double>* out);
	void ZF_channel_estimator(std::complex <double>*in);
	void LS_channel_estimator(std::complex <double>*in);
	// A.1.4: cross-pilot differential noise variance estimator.
	// Replaces pilot-residual estimator (which collapsed to 0 for ZF post-E1
	// commit 38f5c60, biased low by (N-1)/N for LS). For adjacent pilot pairs
	// in the same column (separated by Dy symbols), computes |H_a - H_b|^2/2.
	// Adjacent column-pilots see ~the same slow-varying channel, so the delta
	// is dominated by noise. Returns σ²/|X|² in baseband-bin units, or 0.01 if
	// fewer than 1 valid pair (matches previous default).
	// Reference: Ozdemir & Arslan, "Channel Estimation for Wireless OFDM
	// Systems," IEEE Comm Surveys 2007, §IV-B.
	double estimate_noise_from_pilot_pairs(std::complex<double>* in);
	void CPE_correction(std::complex<double>* in);
	void restore_channel_amplitude();
	double carrier_sampling_frequency_sync(std::complex <double>*in, double carrier_freq_width, int preamble_nSymb, double sampling_frequency);
	double carrier_frequency_sync_nb(std::complex<double>* in, double carrier_freq_width, int preamble_nSymb);
	// Mini-Moose CFO refinement for WB MFSK data preamble. Cross-half-symbol
	// phase estimator using known CW preamble tones. Capture range
	// ±carrier_freq_width Hz (≈ ±46.875 Hz at WB ROBUST_0). Confidence gate
	// returns 0 when |C|/energy < 0.05. See data-preamble-port-research.md §20.
	double carrier_frequency_sync_wb_mfsk(std::complex<double>* in,
	                                      double carrier_freq_width,
	                                      int preamble_nSymb,
	                                      const int* preamble_tones,
	                                      int M_tones,
	                                      int nStreams,
	                                      const int* stream_offsets);
	// Mini-Moose CFO refinement for WB MFSK control-frame patterns (ACK,
	// CONNECT, BREAK, HAIL). Mirror of carrier_frequency_sync_wb_mfsk with
	// (a) a starting sample offset (best_offset from detect_ack_pattern),
	// and (b) ctrl-pattern tone hopping
	// (actual_tone = (pattern_tones[s % pattern_len] + s*tone_hop_step) % M).
	// Same half-symbol cross-correlation math, same 0.05 confidence gate,
	// same capture range ±carrier_freq_width Hz.
	// See data-preamble-port-research.md §24.2.a.
	double carrier_frequency_sync_wb_ctrl(std::complex<double>* in,
	                                      double carrier_freq_width,
	                                      int pattern_nsymb,
	                                      int sym_start_offset_samples,
	                                      const int* pattern_tones,
	                                      int pattern_len,
	                                      int tone_hop_step,
	                                      int M_tones,
	                                      int nStreams,
	                                      const int* stream_offsets);
	void channel_equalizer(std::complex <double>* in, std::complex <double>* out);
	void channel_equalizer_without_amplitude_restoration(std::complex <double>* in,std::complex <double>* out);
	void smooth_channel_estimate_dft();

	void automatic_gain_control(std::complex <double>*in);
	double measure_variance(std::complex <double>*in);
	double measure_signal_stregth(std::complex <double> *in, int nItems);
	st_power_measurment measure_signal_power_avg_papr(double *in, int nItems);
	void peak_clip(double *in, int nItems, double papr);
	void peak_clip(std::complex <double> *in, int nItems, double papr);
	double measure_SNR(std::complex <double>*in_s, std::complex <double>*in_n, int nItems);
	int time_sync(std::complex <double>*in, int size, int interpolation_rate, int location_to_return);
	int time_sync_preamble(std::complex <double>*in, int size, int interpolation_rate, int location_to_return, int step, int nTrials_max);
	TimeSyncResult time_sync_preamble_with_metric(std::complex <double>*in, int size, int interpolation_rate, int location_to_return, int step, int nTrials_max);
	TimeSyncResult time_sync_preamble_halfsym(std::complex<double>* in, int size, int interpolation_rate, int step, double early_exit_metric = 0.0);
	TimeSyncResult time_sync_preamble_halfsym_2phase(std::complex<double>* in, int size, int interpolation_rate, double early_exit_metric = 0.0);
	TimeSyncResult time_sync_preamble_fft(std::complex<double>* baseband_interp, int buffer_size_interp, int interpolation_rate, int preamble_nSymb);
	TimeSyncResult time_sync_preamble_fft_fine(std::complex<double>* baseband_interp, int buffer_size_interp, int interpolation_rate, int preamble_nSymb, int coarse_pos, int search_half_window);
	int time_sync_mfsk(std::complex<double>* baseband_interp, int buffer_size_interp, int interpolation_rate, int preamble_nSymb, const int* preamble_tones, int mfsk_M, int nStreams, const int* stream_offsets, int search_start_symb = 0, double* out_metric = nullptr);
	double detect_ack_pattern(std::complex<double>* baseband_interp, int buffer_size_interp, int interpolation_rate, int ack_nsymb, const int* ack_tones, int ack_pattern_len, int tone_hop_step, int mfsk_M, int nStreams, const int* stream_offsets, int* out_matched = nullptr, int suffix_start = 0, int* out_suffix_matched = nullptr, int* out_best_offset = nullptr, int reserve_after = 0, uint32_t* out_match_mask = nullptr, bool always_fine = false);
	void decode_suffix_tones(std::complex<double>* baseband_interp, int buffer_size_interp, int interpolation_rate, int pattern_offset, int pattern_nsymb, int suffix_len, int tone_hop_step, int mfsk_M, int nStreams, const int* stream_offsets, int* out_tones);
	// decode_suffix_tones_soft removed in §7.13.12 (only caller was the
	// deleted decode_sack_bitmap_ldpc in the legacy MFSK SACK path).
	// decode_suffix_candidates: top-K per-symbol candidate tones + soft costs
	// for the CRC-aided soft list decoder (connect-suffix-fec-research.md §3).
	// out_cand / out_cost are suffix_len*K arrays; cand[s*K+0] == the hard
	// decode_suffix_tones result for symbol s. Zero added airtime.
	void decode_suffix_candidates(std::complex<double>* baseband_interp, int buffer_size_interp, int interpolation_rate, int pattern_offset, int pattern_nsymb, int suffix_len, int tone_hop_step, int mfsk_M, int nStreams, const int* stream_offsets, int K, int* out_cand, double* out_cost);
	// decode_suffix_energies: full per-tone energy matrix E[s*mfsk_M + t] for the
	// soft GF(16) RA decoder (tier2-suffix-fec-gf16-spike.md). Same FFT + de-hop
	// math as decode_suffix_candidates, but emits ALL M energies per symbol
	// (de-hopped to data-tone index, combined across streams) rather than the
	// top-K + cost. Slots for symbols past the buffer end are set to 0.
	void decode_suffix_energies(std::complex<double>* baseband_interp, int buffer_size_interp, int interpolation_rate, int pattern_offset, int pattern_nsymb, int suffix_len, int tone_hop_step, int mfsk_M, int nStreams, const int* stream_offsets, double* out_energies);
	int symbol_sync(std::complex <double>*, int size, int interpolation_rate, int location_to_return);
	void rational_resampler(std::complex <double>* in, int in_size , std::complex <double>* out, int rate, int interpolation_decimation);
	void baseband_to_passband(std::complex <double>* in, int in_size, double* out, double sampling_frequency, double carrier_frequency, double carrier_amplitude, int interpolation_rate);
	void passband_to_baseband(double* in, int in_size, std::complex <double>* out, double sampling_frequency, double carrier_frequency, double carrier_amplitude, int decimation_rate, cl_FIR* filter, int sample_offset=0);
	// Combined mix + polyphase FIR + decimate: writes in_size/M complex samples
	// directly to `out` at the decimated rate. Use when the caller would
	// otherwise call passband_to_baseband() at the high rate then immediately
	// rational_resampler() to decimate — this fuses both and skips the
	// (M-1)/M of FIR outputs the old chain threw away. Profile on Pi RX side
	// showed FIR at 92% CPU; this drops the FIR portion ~M× on hot paths.
	void passband_to_baseband_decimated(double* in, int in_size, std::complex <double>* out, double sampling_frequency, double carrier_frequency, double carrier_amplitude, int M, cl_FIR* filter, int sample_offset=0);
	struct st_channel_complex * estimated_channel, *estimated_channel_without_amplitude_restoration;
	int Nfft,Nc,Nsymb;
	float gi;
	struct st_carrier* ofdm_frame;
	struct st_carrier* ofdm_preamble;
	cl_pilot_configurator pilot_configurator;
	cl_preamble_configurator preamble_configurator;
	void fft(std::complex <double>* in, std::complex <double>* out, int _Nfft);
	int time_sync_Nsymb;
	double freq_offset_ignore_limit;
	cl_FIR FIR_rx_data,FIR_rx_time_sync;
	cl_FIR FIR_tx1, FIR_tx2;
	int start_shift;
	long unsigned passband_start_sample;

	double preamble_papr_cut;
	double data_papr_cut;

	int channel_estimator;
	int channel_estimator_amplitude_restoration;
	int LS_window_width;
	int LS_window_hight;

	// MMSE regularization: noise variance estimated from pilot residuals
	// Set by ZF/LS channel estimator, used by channel_equalizer
	double noise_variance_estimate;

	// fix/cfg16-nv-restore validation knob: when true, LS_channel_estimator
	// prints [LS-NV-DBG] comparing the restored pilot-residual nv with the
	// A.1.4 cross-pilot value on the same frame. Default false (production
	// path unchanged). Toggled in main.cc via --ls-nv-debug for loopback BER.
	bool ls_nv_debug_enabled;

	// fix/cfg16-nv-restore A/B toggle: when true, the LS path reverts to A.1.4's
	// cross-pilot nv (pre-fix/monitor behavior) so one binary runs both arms.
	// Default false = the fix (restored pilot residual). --ls-crosspilot-nv=on.
	bool ls_use_crosspilot_nv;

	// Pre-allocated buffers for passband_to_baseband (avoids new/delete per call)
	std::complex<double>* p2b_l_data;
	std::complex<double>* p2b_data_filtered;
	int p2b_buffer_size;

	// Pre-allocated Nfft-sized work buffers shared by time_sync_mfsk and
	// detect_ack_pattern (never called concurrently)
	std::complex<double>* work_buf_a;
	std::complex<double>* work_buf_b;

	// Pre-allocated grow-as-needed buffers for time_sync_preamble[_with_metric]
	int* tsync_corr_loc;
	double* tsync_corr_vals;
	int tsync_corr_size;
	std::complex<double>* tsync_data;
	int tsync_data_size;

	// Pre-allocated grow-as-needed buffer for baseband_to_passband
	std::complex<double>* b2p_data_interpolated;
	int b2p_buffer_size;

	// MFSK cross-correlation preamble template (NB + WB).
	// Array sized 16 to support the WB ROBUST_0/1/2 16-symbol preamble
	// (data-flow-preamble_nSymb.md §H1). NB still uses 8 symbols; entries
	// 8..15 stay zero on NB and the time_sync_mfsk_corr loop reads
	// `template_nsymb` (= mfsk.preamble_nSymb) so unused slots are skipped.
	//
	// 2026-05-27 (data-preamble-port-research.md §14): `time_sync_mfsk_corr`
	// is now a discrete FFT-bin-argmax matcher (mirror of detect_ack_pattern).
	// The template waveform is kept here as DEAD STATE for revert safety;
	// the new detector reads `mfsk_preamble_*` fields below for tone bins.
	// (Template cleanup deferred to a follow-up commit per §8.4.)
	std::complex<double>* mfsk_corr_template;
	int mfsk_corr_template_len;
	double mfsk_corr_template_energy;
	int mfsk_corr_template_nsymb;
	double mfsk_corr_template_sym_energy[16]; // per-symbol energy for per-symbol correlation

	// MFSK preamble parameters consumed by `time_sync_mfsk_corr` (post-2026-05-27
	// discrete-match port). Populated by load_configuration alongside the
	// template (telecom_system.cc:4956+). Mirror of `cl_mfsk::M`,
	// `cl_mfsk::nStreams`, `cl_mfsk::stream_offsets[]`, `cl_mfsk::preamble_tones[]`,
	// `cl_mfsk::preamble_nSymb`, `cl_mfsk::preamble_match_threshold`.
	int mfsk_M;
	int mfsk_nStreams;
	int mfsk_stream_offsets[4]; // matches cl_mfsk::MAX_STREAMS
	int mfsk_preamble_tones[16]; // matches MAX_PREAMBLE_SYMB
	int mfsk_preamble_nsymb;
	int mfsk_preamble_match_threshold;

	int time_sync_mfsk_corr(std::complex<double>* baseband_interp, int buffer_size_interp, int interpolation_rate, int search_start_symb, double* out_metric);

	// OFDM matched-filter preamble template (replaces FFT-based detection)
	std::complex<double>* ofdm_corr_template;
	int ofdm_corr_template_len;       // total samples (nsymb * Nofdm)
	int ofdm_corr_template_nsymb;     // preamble symbol count
	double ofdm_corr_template_sym_energy[16]; // per-symbol energy
	double ofdm_corr_template_energy; // total energy
	TimeSyncResult time_sync_preamble_matched(std::complex<double>* baseband_interp, int buffer_size_interp, int interpolation_rate, int preamble_nSymb);
};



#endif
