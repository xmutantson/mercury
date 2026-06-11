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

#include "physical_layer/telecom_system.h"
#include "audioio/audioio.h"
#include "debug/canary_guard.h"
#include "common/sim_channel.h" // cl_sim_sfo — long-block timing-acquisition-under-SFO harness
#include <chrono>
#include <algorithm> // std::sort — bigblock pre-FFT AFC-track median smoother (Option C)
#include <vector>  // suffix-FEC soft decode candidate buffers
#include <cstdlib> // std::getenv / atoi for MERCURY_SIM2_MINI_NSYM (LEVER P MINI knob)
#include <cstdint> // bigblock WAV I/O: uint32_t/int16_t
#include <cstdio>  // bigblock WAV I/O: FILE/fopen/fread/fwrite
#include <cmath>   // bigblock: round/log2/sqrt/fabs
#include <cassert> // fact-doc §13: deterministic bounds assert on the big-block TX/RX write
#ifdef MERCURY_GUI_ENABLED
#include "gui/gui_state.h"
#endif
#if defined(_WIN32)
#include <windows.h>
#endif

extern cbuf_handle_t capture_buffer;
extern cbuf_handle_t playback_buffer;

// Test mode: artificial TX carrier offset in Hz (for testing frequency sync)
extern "C" double test_tx_carrier_offset;



cl_telecom_system::cl_telecom_system()
{
	// Per-instance RNG (single-process-sim-refactor.md §10.1, Landmine 1).
	// DEFAULT rng_own_ = false → ts_srandom/ts_random call the GLOBAL
	// __srandom/__random (the file-static unsafe_state), so production, the
	// two-process paced sim, and every single-instance suite are BYTE-IDENTICAL
	// to before this change. The 2-instance SIM_INPROC stepper opts EACH instance
	// IN via enable_per_instance_rng() AFTER construction — only then does this
	// instance draw from its own residue-free stream (closing the cross-instance
	// pre-eq-channel residue inheritance). os_rng_make binds rng_ to rng_state_
	// here so enabling is just a flag flip (no late allocation).
	os_rng_make(&rng_, rng_state_, 1u);
	rng_own_ = false;
	skip_var_gate_enabled = true;  // default = HEAD behavior; CLI --skip-var-gate=off disables
	rx_normalize_enabled  = true;  // default = HEAD behavior; CLI --rx-normalize=off disables
	csi_llr_enabled       = true;  // default = HEAD behavior; CLI --csi-llr=off disables
	fsel_test_enabled     = false; // fix/cfg16-nv-restore: --fsel-test=on enables (BER loopback only)
	fsel_amp              = 0.6;   // second-ray amplitude (linear)
	fsel_delay            = 128;   // second-ray delay in passband samples (~Nfft/8 @ interp=4, within GI)
	fsel_fd               = 0.0;   // cfg16-nvfix: 2nd-ray Doppler (Hz); 0 = static (original)
	ber_single_esn0       = -999.0f; // fix/cfg16-nv-restore: <=-900 = normal full sweep
	ber_frames_override   = 0;     // 0 = use sweep default frame count
	mean_h_gate_threshold = 0.30;  // default = HEAD (b806b76); pre-IONOS was 0.50
	energy_gate_floor    = 1e-12;  // default = HEAD (b806b76); pre-IONOS was 0.001
	ofdm_defer_overflow_enabled = true; // default = HEAD (7076a4b Fix A)
	// P3 HW VALIDATION FORCE: the CFG16-rung big-block framing flag is normally OFF
	// (default; the gearshift AUTO-election is P4). For the P3 VARA-parity HW test we
	// FORCE it on via env MERCURY_BIGBLOCK_FRAMING=1 so a real two-instance ARQ session
	// uses the one-acquisition K-codeword block path at CFG16. This only flips the
	// persistent member; sack_negotiated_recompute_batch() then pins data_batch_size=K
	// on BOTH peers (the R-B pin), and the live TX/RX paths (bigblock_send_one_block /
	// bigblock_receive_carve) engage at the CFG16 OFDM rung. Default unset = stock OFF =
	// byte-identical per-frame path. Not wired into the gearshift (that is P4).
	{ const char* e = std::getenv("MERCURY_BIGBLOCK_FRAMING");
	  if(e && *e && atoi(e) != 0) bigblock_framing_enabled = true; }
	receive_stats.iterations_done=-1;
	receive_stats.delay=0;
	receive_stats.delay_of_last_decoded_message=-1;
	receive_stats.mfsk_search_raw=0;
	receive_stats.ofdm_search_raw=0;
	receive_stats.ofdm_batch_active=false;
	receive_stats.ofdm_drift_per_frame=0.0;
	receive_stats.time_peak_symb_location=0;
	receive_stats.time_peak_subsymb_location=0;
	receive_stats.sync_trials=0;
	receive_stats.phase_error_avg=0;
	receive_stats.freq_offset=0;
	receive_stats.freq_offset_of_last_decoded_message=0;
	receive_stats.message_decoded=NO;
	receive_stats.SNR=-99.9;
	receive_stats.signal_stregth_dbm=-999;
	receive_stats.mfsk_search_raw=0;
	receive_stats.ofdm_search_raw=0;
	receive_stats.ofdm_batch_active=false;
	receive_stats.ofdm_drift_per_frame=0.0;

	time_sync_trials_max=20;
	use_last_good_time_sync=NO;
	use_last_good_freq_offset=NO;
	mfsk_fixed_delay=-1;
	ofdm_forced_delay=-1;
	test_puncture_nBits=0;
	last_coarse_freq_offset=0.0;
	consecutive_ofdm_decode_fails=0;  // STALE-CFO scoped reset (long-run-degradation.md §2.2)
	ctrl_nBits=0;
	ctrl_nsymb=0;
	mfsk_ctrl_mode=false;
	coarse_freq_sync_enabled=false;
	ack_pattern_passband_samples=0;
	ack_snr_pattern_passband_samples=0;
	ack_sack_pattern_passband_samples=0;
	connect_pattern_passband_samples=0;
	ctrl_suffix_pattern_passband_samples=0;
	// Suffix FEC prototype (connect-suffix-fec-research.md). Default OFF so the
	// baseline hard decode path is byte-identical; the *_soft entry points are
	// always callable for direct measurement.
	suffix_fec_mode=0;
	suffix_fec_K=4;
	suffix_fec_max_trials=4000;
	// Default Hamming-ball radius = 1: measured pure-noise FAR 0.25% (vs 3.4% at
	// 2, 13.9% at 3 — see test_suffix_soft_pure_noise_far / fact-doc §6). This
	// is the safe per-decode operating point; the type field + ARQ retransmit
	// are additional backstops. Captures the dominant cliff regime (1 wrong
	// symbol). Raise to 2 only if a higher FAR is acceptable for more reach.
	suffix_fec_max_flips=1;
	ack_pattern_detection_threshold=0.8;
	operation_mode=BER_PLOT_baseband;
	bit_interleaver_block_size=1;
	time_freq_interleaver_block_size=1;
	output_power_Watt=1;
	carrier_amplitude=sqrt(2.0);
	sampling_frequency=0;
	Shannon_limit=0;
	rbc=0;
	rb=0;
	Tf=0;
	frequency_interpolation_rate=0;
	Ts=0;
	Tu=0;
	LDPC_real_CR=0;
	bandwidth=0;
	M=0;
	carrier_frequency=0;
	current_configuration=CONFIG_NONE;
	last_configuration=CONFIG_NONE;
	outer_code=NO_OUTER_CODE;
	outer_code_reserved_bits=0;
	bit_energy_dispersal_seed=0;
	narrowband_enabled=NO;
	pre_equalization_channel=NULL;
	// 2D channel-state lookup helpers (see fact-doc channel-state-2d-lookup.md §8 Step 1).
	// Sentinels until the first ACK/HAIL detection / preamble channel estimate runs.
	last_correlator_metric_db = -99.0;
	last_channel_selectivity  = -1.0;
	init_tx_gain_defaults();
}


cl_telecom_system::~cl_telecom_system()
{

}

void cl_telecom_system::init_tx_gain_defaults()
{
	// Compute default boost values from the original formula:
	//   max_Nc * trim / sqrt(Nc * nStreams)
	// where max_Nc=50, trim=pow(10,-2/20)=0.7943 (OFDM clip headroom offset)
	const double max_Nc = 50.0;
	const double trim = pow(10.0, -2.0 / 20.0);  // -2 dB = 0.7943

	// WB: Nc=50, NB: Nc=10
	const double Nc_wb = 50.0, Nc_nb = 10.0;

	// All gains calibrated via VB-Cable measurement (calibrate_gain.py + play_all_modes.py)
	// Target: WB CONFIG_0 peak level (-11.4 dBFS on VB-Cable)
	// Corrections applied as gain_old * pow(10, delta_dB/20)

	// [nb_mod=0] WB modulation
	// MFSK_1S was -10.8 → need -0.6 dB: 5.617 * 0.933 = 5.24
	// MFSK_2S was -11.3 → need -0.1 dB: essentially unchanged
	tx_gain[TX_SIG_MFSK_1S][0][0] = 5.24;
	tx_gain[TX_SIG_MFSK_1S][0][1] = 5.24;
	tx_gain[TX_SIG_MFSK_2S][0][0] = max_Nc * trim / sqrt(Nc_wb * 2.0);  // 3.97, already -11.3
	tx_gain[TX_SIG_MFSK_2S][0][1] = tx_gain[TX_SIG_MFSK_2S][0][0];
	tx_gain[TX_SIG_OFDM]   [0][0] = 1.0;  // -11.4, the reference
	tx_gain[TX_SIG_OFDM]   [0][1] = 1.0;
	tx_gain[TX_SIG_ACK]    [0][0] = 5.24;
	tx_gain[TX_SIG_ACK]    [0][1] = 5.24;
	tx_gain[TX_SIG_BREAK]  [0][0] = 5.24;
	tx_gain[TX_SIG_BREAK]  [0][1] = 5.24;

	// [nb_mod=1] NB modulation
	// Scale from WB calibrated values by sqrt(Nc_wb/Nc_nb) = sqrt(5) = 2.236
	// NB has fewer subcarriers → single MFSK tone needs more boost to match OFDM power.
	// Previous values (23.8, 16.9) were ~2x too high, causing 57% clipping → BER=0.5.
	double nb_scale = sqrt(Nc_wb / Nc_nb);  // sqrt(5) = 2.236
	tx_gain[TX_SIG_MFSK_1S][1][0] = tx_gain[TX_SIG_MFSK_1S][0][0] * nb_scale;  // 5.24 * 2.236 = 11.72
	tx_gain[TX_SIG_MFSK_1S][1][1] = tx_gain[TX_SIG_MFSK_1S][1][0];
	tx_gain[TX_SIG_MFSK_2S][1][0] = tx_gain[TX_SIG_MFSK_2S][0][0] * nb_scale;  // 3.97 * 2.236 = 8.88
	tx_gain[TX_SIG_MFSK_2S][1][1] = tx_gain[TX_SIG_MFSK_2S][1][0];
	tx_gain[TX_SIG_OFDM]   [1][0] = 1.0 * pow(10.0, 7.3 / 20.0);  // 2.317, already -11.5
	tx_gain[TX_SIG_OFDM]   [1][1] = tx_gain[TX_SIG_OFDM][1][0];
	tx_gain[TX_SIG_ACK]    [1][0] = tx_gain[TX_SIG_MFSK_1S][1][0];  // single tone, same as MFSK 1S
	tx_gain[TX_SIG_ACK]    [1][1] = tx_gain[TX_SIG_ACK][1][0];
	tx_gain[TX_SIG_BREAK]  [1][0] = tx_gain[TX_SIG_MFSK_1S][1][0];
	tx_gain[TX_SIG_BREAK]  [1][1] = tx_gain[TX_SIG_BREAK][1][0];
}

double cl_telecom_system::get_tx_gain(tx_signal_type sig) const
{
	int nb = (narrowband_enabled == YES) ? 1 : 0;
	return tx_gain[sig][nb][nb];  // mod and FIR always match currently
}

void cl_telecom_system::set_tx_gain(tx_signal_type sig, int nb_mode, double value)
{
	if(sig < 0 || sig >= TX_SIG_COUNT || nb_mode < 0 || nb_mode > 1)
	{
		printf("[TX-GAIN-OVERRIDE] invalid (sig=%d, nb_mode=%d) — skip\n",
			(int)sig, nb_mode);
		fflush(stdout);
		return;
	}
	static const char* sig_names[TX_SIG_COUNT] = {
		"MFSK_1S", "MFSK_2S", "OFDM   ", "ACK    ", "BREAK  "
	};
	static const char* mode_names[2] = { "WB", "NB" };
	double prev = tx_gain[sig][nb_mode][nb_mode];
	tx_gain[sig][nb_mode][0] = value;
	tx_gain[sig][nb_mode][1] = value;  // mirror — get_tx_gain reads [nb][nb] diagonal
	printf("[TX-GAIN-OVERRIDE] %s  %s  %.4f -> %.4f (calibration override, "
		"plan §7.13.21)\n", sig_names[sig], mode_names[nb_mode], prev, value);
	fflush(stdout);
}

void cl_telecom_system::print_tx_gain_table() const
{
	static const char* sig_names[TX_SIG_COUNT] = {
		"MFSK_1S", "MFSK_2S", "OFDM   ", "ACK    ", "BREAK  "
	};
	static const char* mode_names[2] = { "WB", "NB" };

	printf("[TX-GAIN] Gain table (signal × mod × fir):\n");
	for(int sig = 0; sig < TX_SIG_COUNT; sig++)
	{
		for(int nb_mod = 0; nb_mod < 2; nb_mod++)
		{
			for(int nb_fir = 0; nb_fir < 2; nb_fir++)
			{
				const char* marker = (nb_mod == nb_fir) ? " <" : "  ";
				printf("[TX-GAIN]   %s  %s_mod  %s_fir  = %.4f%s\n",
					sig_names[sig], mode_names[nb_mod], mode_names[nb_fir],
					tx_gain[sig][nb_mod][nb_fir], marker);
			}
		}
	}
}

cl_error_rate cl_telecom_system::baseband_test_EsN0(float EsN0,int max_frame_no)
{
	cl_error_rate lerror_rate;
	float power_normalization=sqrt((double)ofdm.Nfft);
	float sigma=1.0/sqrt(pow(10,(EsN0/10)));
	float variance=1.0/(pow(10,(EsN0/10)));
	int nVirtual_data;
	int nReal_data;
	int delay=0;
	nVirtual_data=ldpc.N-data_container.nBits;
	nReal_data=data_container.nBits-ldpc.P;

	int constellation_plot_counter=0;
	int constellation_plot_nFrames=10;
	float contellation[ofdm.pilot_configurator.nData*constellation_plot_nFrames][2]={0};

	while(lerror_rate.Frames_total<max_frame_no)
	{
		for(int i=0;i<nReal_data;i++)
		{
			data_container.data_bit[i]=ts_random()%2;   // §10.1 per-instance when opted in
		}
		for(int i=0;i<nVirtual_data;i++)
		{
			data_container.data_bit[nReal_data+i]=data_container.data_bit[i];
		}

		ldpc.encode(data_container.data_bit,data_container.encoded_data);

		for(int i=0;i<ldpc.P;i++)
		{
			data_container.encoded_data[nReal_data+i]=data_container.encoded_data[i+ldpc.K];
		}

		interleaver(data_container.encoded_data,data_container.bit_interleaved_data,data_container.nBits,bit_interleaver_block_size);

		psk.mod(data_container.bit_interleaved_data,data_container.nBits,data_container.modulated_data);
		interleaver(data_container.modulated_data, data_container.ofdm_time_freq_interleaved_data, data_container.nData, time_freq_interleaver_block_size);
		ofdm.framer(data_container.ofdm_time_freq_interleaved_data,data_container.ofdm_framed_data);

		for(int i=0;i<data_container.Nsymb;i++)
		{
			ofdm.symbol_mod(&data_container.ofdm_framed_data[i*data_container.Nc],&data_container.ofdm_symbol_modulated_data[i*data_container.Nofdm]);
		}

		for(int j=0;j<data_container.Nofdm*data_container.Nsymb;j++)
		{
			data_container.ofdm_symbol_modulated_data[j]/=power_normalization;
		}


		awgn_channel.apply_with_delay(data_container.ofdm_symbol_modulated_data,data_container.baseband_data,sigma,data_container.Nofdm*data_container.Nsymb,0);


		for(int j=0;j<data_container.Nofdm*data_container.Nsymb;j++)
		{
			data_container.baseband_data[j]*=power_normalization;
		}

		for(int i=0;i<data_container.Nsymb;i++)
		{
			ofdm.symbol_demod(&data_container.baseband_data[delay*0+i*data_container.Nofdm],&data_container.ofdm_symbol_demodulated_data[i*data_container.Nc]);
		}

		if(ofdm.channel_estimator==ZERO_FORCE)
		{
			ofdm.ZF_channel_estimator(data_container.ofdm_symbol_demodulated_data);
		}
		else if (ofdm.channel_estimator==LEAST_SQUARE)
		{
			ofdm.LS_channel_estimator(data_container.ofdm_symbol_demodulated_data);
		}

		if(ofdm.channel_estimator_amplitude_restoration==YES)
		{
			ofdm.restore_channel_amplitude();
			ofdm.channel_equalizer_without_amplitude_restoration(data_container.ofdm_symbol_demodulated_data,data_container.equalized_data_without_amplitude_restoration);
			ofdm.deframer(data_container.equalized_data_without_amplitude_restoration,data_container.ofdm_deframed_data_without_amplitude_restoration);
		}

		ofdm.channel_equalizer(data_container.ofdm_symbol_demodulated_data,data_container.equalized_data);

		variance=ofdm.measure_variance(data_container.ofdm_symbol_demodulated_data);

		ofdm.deframer(data_container.equalized_data,data_container.ofdm_deframed_data);
		deinterleaver(data_container.ofdm_deframed_data, data_container.ofdm_time_freq_deinterleaved_data, data_container.nData, time_freq_interleaver_block_size);
		psk.demod(data_container.ofdm_time_freq_deinterleaved_data,data_container.nBits,data_container.demodulated_data,variance);

		deinterleaver(data_container.demodulated_data,data_container.deinterleaved_data,data_container.nBits,bit_interleaver_block_size);


		for(int i=ldpc.P-1;i>=0;i--)
		{
			data_container.deinterleaved_data[i+nReal_data+nVirtual_data]=data_container.deinterleaved_data[i+nReal_data];
		}

		for(int i=0;i<nVirtual_data;i++)
		{
			data_container.deinterleaved_data[nReal_data+i]=data_container.deinterleaved_data[i];
		}


		ldpc.decode(data_container.deinterleaved_data,data_container.hd_decoded_data_bit);

		// Always use fully-equalized data for visualization (tight clusters).
		// Amplitude restoration still helps the decoder via the separate path.
		for(int i=0;i<ofdm.pilot_configurator.nData;i++)
		{
			contellation[constellation_plot_counter*ofdm.pilot_configurator.nData+i][0]=data_container.ofdm_deframed_data[i].real();
			contellation[constellation_plot_counter*ofdm.pilot_configurator.nData+i][1]=data_container.ofdm_deframed_data[i].imag();
		}

		constellation_plot_counter++;

		if(constellation_plot_counter==constellation_plot_nFrames)
		{
			constellation_plot_counter=0;
			constellation_plot.plot_constellation(&contellation[0][0],ofdm.pilot_configurator.nData*constellation_plot_nFrames);
		}


		lerror_rate.check(data_container.data_bit,data_container.hd_decoded_data_bit,nReal_data);
	}
	return lerror_rate;
}

double cl_telecom_system::skip_var_nv_ceiling(int configuration)
{
	// FIX-1 (2026-06-04): config-rate-aware SKIP-VAR noise-variance ceiling.
	//
	// The SKIP-VAR gate in receive_byte() rejects a frame BEFORE LDPC when the
	// pilot-residual noise estimate (ofdm.noise_variance_estimate, the post-DFT LS
	// residual mean|Y-H·Xpilot|²) exceeds this ceiling. Its legitimate purpose is
	// to avoid burning decoder/trial budget on frames that genuinely cannot decode.
	// The historical flat 0.5 was WRONG because it is code-rate-INDEPENDENT while
	// the LDPC decode floor is code-rate-DEPENDENT: at low code rates (CONFIG_0-6
	// BPSK, rates 1/16..8/16) the decoder recovers frames with nv well above 0.5,
	// so 0.5 rejected decodable weak frames before LDPC ever ran.
	//
	// Measured (clean-AWGN, mercury -m PLOT_PASSBAND --skip-var-gate=off so the
	// decoder runs at every nv, --ber-frames=40 — FIX-1 report 2026-06-04):
	//   CONFIG_0 BPSK 1/16  decode floor nv≈1.49 (39/40 @ Es/N0 -12, 31/40 @ -13)
	//   CONFIG_3 BPSK 4/16  decode floor nv≈1.13 (40/40 @ -8 nv→1.04, 31/40 @ -9, 0 @ -10)
	//   CONFIG_6 BPSK 8/16  decode floor nv≈0.80 (40/40 @ -5 nv→0.68, 38/40 @ -6, 1/40 @ -7)
	// Ceilings sit just ABOVE each config's measured gate-OFF decode floor, so a
	// low-rate config admits the proven-decodable band (incl. the nv≈0.70 frames the
	// flat 0.5 rejected — verified: CONFIG_0 @ -6 dB, nv 0.71→OFDM-OK) but still
	// fast-skips beyond its real floor.
	//
	// CONFIG_7-16 (QPSK/8PSK/16QAM/32QAM) and ROBUST/NB ids KEEP the historical 0.5
	// (UNCHANGED). Their decode floor sits below 0.5 already (decoder-bound), so the
	// gate never bound them — keeping 0.5 makes this a pure LOOSENING for CONFIG_0-6
	// and a strict no-op everywhere else (NO regression). MFSK/ROBUST never reach
	// this gate (demod branch exits before the OFDM trial block) so they are 0.5 by
	// construction. HARD CAP: CONFIG_0's 1.60 bounds the loosest config, so truly-
	// hopeless frames (and any config above 1.60) are still rejected — LDPC+CRC
	// remain the actual noise-rejection mechanism downstream regardless.
	switch(configuration)
	{
		case CONFIG_0: return 1.60;  // BPSK 1/16, floor nv≈1.48 (hard cap)
		case CONFIG_1: return 1.50;  // BPSK 2/16
		case CONFIG_2: return 1.35;  // BPSK 3/16
		case CONFIG_3: return 1.20;  // BPSK 4/16
		case CONFIG_4: return 1.10;  // BPSK 5/16
		case CONFIG_5: return 1.00;  // BPSK 6/16
		case CONFIG_6: return 0.90;  // BPSK 8/16 (rate 0.5, masked↔decoder-bound boundary)
		default:       return 0.50;  // CONFIG_7-16 + ROBUST/NB: HEAD behavior (decoder-bound)
	}
}

cl_error_rate cl_telecom_system::passband_test_EsN0(float EsN0,int max_frame_no)
{
	cl_error_rate lerror_rate;
	// For OFDM: sigma = 1/sqrt(10^(EsN0/10)) is the standard Es/N0 formula.
	// For MFSK: EsN0 parameter is treated as channel SNR (dB).
	// Sigma is calibrated by measuring actual transmitted signal power so that
	// SNR = P_signal / P_noise = P_signal / (sigma^2/2).
	float sigma = 0;
	bool sigma_calibrated = (M != MOD_MFSK);
	if(M != MOD_MFSK)
	{
		sigma = 1.0f / sqrt(pow(10.0f, (EsN0 / 10.0f)));
	}
	int nReal_data=data_container.nBits-ldpc.P;
	int delay=0;

	if(data_container.Nfft==1024)
	{
		delay=100;
	}
	else
	{
		delay=50;
	}

	int constellation_plot_counter=0;
	int constellation_plot_nFrames=10;
	// For MFSK, nData can be very large (>15000) - skip constellation plot to avoid stack overflow
	int nDataPlot = (M == MOD_MFSK) ? 0 : ofdm.pilot_configurator.nData;
	float contellation[nDataPlot*constellation_plot_nFrames+1][2]={};

	while(lerror_rate.Frames_total<max_frame_no)
	{
		for(int i=0;i<nReal_data-outer_code_reserved_bits;i++)
		{
			data_container.data_bit[i]=ts_random()%2;   // §10.1 per-instance when opted in
		}
		bit_to_byte(data_container.data_bit,data_container.data_byte,nReal_data-outer_code_reserved_bits);
		this->transmit_byte(data_container.data_byte,(nReal_data-outer_code_reserved_bits)/8,data_container.passband_data,SINGLE_MESSAGE);

		// MFSK: calibrate sigma from measured signal power (first frame only)
		// In-band channel SNR: SNR = P_sig / P_noise_inband
		// where P_noise_inband = P_noise_total * (BW_signal / f_nyquist)
		// P_noise_total per sample = sigma^2/2 (from AWGN apply)
		// sigma = sqrt(2 * P_sig * f_nyquist / (SNR_linear * BW_signal))
		if(!sigma_calibrated)
		{
			int nSamples = (data_container.Nofdm * (data_container.Nsymb + data_container.preamble_nSymb)) * frequency_interpolation_rate;
			double P_sig = 0;
			for(int i = 0; i < nSamples; i++)
			{
				P_sig += data_container.passband_data[i] * data_container.passband_data[i];
			}
			P_sig /= nSamples;
			double f_nyquist = sampling_frequency / 2.0;
			sigma = (float)sqrt(2.0 * P_sig * f_nyquist / (pow(10.0, EsN0 / 10.0) * bandwidth));
			sigma_calibrated = true;
		}

		// fix/cfg16-nv-restore: optional 2-ray frequency-selective channel, applied
		// to the TX passband BEFORE AWGN. Off by default (production BER unchanged).
		// Static case (fsel_fd<=0): y[n] = x[n] + fsel_amp * x[n-fsel_delay].
		//
		// cfg16-nvfix: TIME-VARYING (Watterson 2-tap) case (fsel_fd>0): the second
		// ray's gain g(t) drifts at Doppler fsel_fd via the SAME AR(1)/Ornstein-
		// Uhlenbeck low-pass the in-tree Watterson uses (sfo_grid_test, ~:6518):
		//   g(n) = rho*g(n-1) + sqrt(1-rho^2)*N(0,1),  rho = exp(-2*pi*fd/f_sym)
		// updated ONCE per OFDM symbol (f_sym = Fs/Nofdm). This is the condition the
		// cross-pilot differential nv estimator (ofdm.cc:1499-1504, delta of same-
		// column pilots Dy symbols apart) collapses on: a SLOW fade leaves consecutive
		// same-carrier pilots highly correlated -> small delta -> nv UNDER-reports,
		// while the per-symbol post-EQ EVM (measure_var) stays large because the
		// channel MOVED between the pilot rows. That nv<<EVM gap is what feeds the
		// 32-QAM demap over-confident LLRs (the bug); a STATIC ray cannot make it
		// (its pilots are time-invariant so the cross-pilot delta is just noise).
		// g(t) is a real, slowly-varying tap gain around the mean fsel_amp (passband
		// is real; a real OU-modulated tap drives the same pilot-drift collapse).
		if(fsel_test_enabled && M != MOD_MFSK && fsel_delay > 0)
		{
			int nSamp = (data_container.Nofdm * (data_container.Nsymb + data_container.preamble_nSymb)) * this->frequency_interpolation_rate;
			if(fsel_fd <= 0.0)
			{
				// STATIC 2-ray (original behavior, byte-identical).
				for(int n = nSamp - 1; n >= fsel_delay; n--)
				{
					data_container.passband_data[n] += fsel_amp * data_container.passband_data[n - fsel_delay];
				}
			}
			else
			{
				// TIME-VARYING 2-ray: per-passband-symbol AR(1) Doppler-filtered gain.
				int sym_samp = data_container.Nofdm * this->frequency_interpolation_rate;
				double Fs    = (double)sampling_frequency;
				double f_sym = (sym_samp>0) ? (Fs / (double)sym_samp) : 1.0;
				double rho   = std::exp(-2.0*M_PI*fsel_fd / f_sym);
				if(rho > 0.999999) rho = 0.999999;
				double inn   = std::sqrt(1.0 - rho*rho);
				// Deterministic per-(esn0,frame) RNG so a cell is reproducible.
				cl_sim_xoshiro frng((uint64_t)(0x5EED1234u ^ (uint64_t)lerror_rate.Frames_total
				                    ^ ((uint64_t)(EsN0*1000.0) << 16)));
				// scatter-ray gain drifts around mean fsel_amp; std = 0.5*fsel_amp.
				double dev   = 0.5 * fsel_amp;
				double g     = fsel_amp;   // start at the mean
				int    cur_sym = -1;
				for(int n = nSamp - 1; n >= fsel_delay; n--)
				{
					int sidx = n / (sym_samp>0?sym_samp:1);
					if(sidx != cur_sym)
					{
						// advance the OU process once per OFDM symbol
						g = rho*g + (1.0-rho)*fsel_amp + (inn*dev)*frng.gauss();
						cur_sym = sidx;
					}
					data_container.passband_data[n] += g * data_container.passband_data[n - fsel_delay];
				}
			}
		}

		awgn_channel.apply_with_delay(data_container.passband_data,data_container.passband_delayed_data,sigma,(data_container.Nofdm*(data_container.Nsymb+data_container.preamble_nSymb))*this->frequency_interpolation_rate,((data_container.preamble_nSymb+2)*data_container.Nofdm+delay)*frequency_interpolation_rate);
		if(M == MOD_MFSK)
		{
			mfsk_fixed_delay = ((data_container.preamble_nSymb+2)*data_container.Nofdm+delay)*frequency_interpolation_rate;
		}
		else
		{
			// OFDM BER test: use known delay position and skip freq sync.
			// Also needed for NB to prevent the NB freq estimator from running
			// on synthetic passband data (no real channel offset to measure).
			ofdm_forced_delay = ((data_container.preamble_nSymb+2)*data_container.Nofdm+delay)*frequency_interpolation_rate;
		}
		this->receive_byte(data_container.passband_delayed_data,data_container.hd_decoded_data_byte);
		mfsk_fixed_delay = -1;
		ofdm_forced_delay = -1;
		byte_to_bit(data_container.hd_decoded_data_byte,data_container.hd_decoded_data_bit,(nReal_data-outer_code_reserved_bits)/8);

		if(nDataPlot > 0)
		{
			// Always use fully-equalized data for visualization (tight clusters).
			for(int i=0;i<nDataPlot;i++)
			{
				contellation[constellation_plot_counter*nDataPlot+i][0]=data_container.ofdm_deframed_data[i].real();
				contellation[constellation_plot_counter*nDataPlot+i][1]=data_container.ofdm_deframed_data[i].imag();
			}

			constellation_plot_counter++;

			if(constellation_plot_counter==constellation_plot_nFrames)
			{
				constellation_plot_counter=0;
				constellation_plot.plot_constellation(&contellation[0][0],nDataPlot*constellation_plot_nFrames);
			}
		}

		lerror_rate.check(data_container.data_bit,data_container.hd_decoded_data_bit,nReal_data-outer_code_reserved_bits);
	}
	return lerror_rate;
}

int cl_telecom_system::get_frame_size_bytes()
{
    return (data_container.nBits - ldpc.P - outer_code_reserved_bits) / 8;
}

int cl_telecom_system::get_frame_size_bits()
{
    return data_container.nBits - ldpc.P - outer_code_reserved_bits;
}

// ===== LEVER P: PREAMBLE AMORTIZATION — pure schedule + effective-length =====
// See fact-documents/data-flow-preamble-amortization.md §1.
// PURE: depends only on its arguments, so TX and RX derive identical schedules.
int cl_telecom_system::preamble_sched_nsymb(int frame_idx_in_batch, bool force_full, int full_nsymb)
{
	if(full_nsymb < 1) full_nsymb = 1;            // degenerate guard
	if(force_full) return full_nsymb;             // retx / after-FAIL re-anchor
	if(frame_idx_in_batch <= 0) return full_nsymb; // batch anchor (frame 0)

	// MERCURY_SIM2_MINI_NSYM (default 1): the per-frame MINI preamble length for
	// the in-process 2-instance time-domain-faithfulness sim (ARM C). The 1-sym
	// MINI under-integrates the Schmidl-Cox metric (L=nsym*Nfft/nIS) by 4x vs the
	// 4-sym FULL, so its variance (S&C 1997 eq.20, sigma^2 ~ 1/L) jitters the
	// peak-pick +-1 OFDM symbol under SFO/CFO; a 2-sym MINI halves that variance
	// and is the "recovers close to ARM A" arm. Read ONCE (function-local static)
	// so TX and RX derive bit-identical schedules — the function stays PURE. The
	// knob only RAISES the MINI floor; it never exceeds full_nsymb. Default 1 ⇒
	// byte-identical to the pre-knob schedule (no env read changes the result).
	// Production (-m ARQ) never sets this env ⇒ MINI stays 1, byte-identical.
	static const int mini_nsymb = []() {
		const char* e = std::getenv("MERCURY_SIM2_MINI_NSYM");
		if(!e) return 1;
		int v = atoi(e);
		if(v < 1) v = 1;                          // clamp: MINI is at least 1 symbol
		return v;
	}();

	int eff = mini_nsymb;
	if(eff > full_nsymb) eff = full_nsymb;        // never exceed the FULL preamble
	return eff;                                    // MINI resync (default 1 symbol)
}

int cl_telecom_system::tx_effective_preamble_nsymb() const
{
	int full = data_container.preamble_nSymb;
	if(!preamble_amortization_enabled) return full;
	if(tx_preamble_nsymb_override < 0) return full;        // legacy / unset
	int eff = tx_preamble_nsymb_override;
	if(eff < 1) eff = 1;
	if(eff > full) eff = full;
	return eff;
}

int cl_telecom_system::rx_effective_preamble_nsymb() const
{
	int full = data_container.preamble_nSymb;
	if(!preamble_amortization_enabled) return full;
	if(rx_preamble_nsymb_override < 0) return full;        // legacy / unset
	int eff = rx_preamble_nsymb_override;
	if(eff < 1) eff = 1;
	if(eff > full) eff = full;
	return eff;
}

void cl_telecom_system::transmit_byte(int *data, int nBytes, double* out, int message_location)
{
	// P1: big-block framing branch (gated; CFG16-rung framing-mode bit, default OFF).
	// When set, one transmit_byte call emits the whole big-block (one 4-sym preamble +
	// K codeword-frames under one acquisition) into `out` instead of one per-frame
	// OFDM frame. NO ARQ change in P1 — the loopback validator drives this directly.
	// P3 HW FIX: the big-block geometry is CFG16-ONLY (K=8, sub_len=ldpc.K/8). With the
	// flag FORCED on for a live ARQ session, the gearshift climbs ROBUST_0 -> CFG16 through
	// CONFIG_0..15 (all M != MFSK), and WITHOUT this config guard every stock per-frame
	// transmit at those rungs wrongly branched into transmit_bigblock with the wrong
	// geometry — corrupting the climb (HW-observed). Gate on current_configuration ==
	// CONFIG_16 so CONFIG_0..15 use the stock per-frame path and only the validated rung
	// emits a block. The PLOT_PASSBAND / unit validators run at CONFIG_16 (-s 16), so they
	// are unaffected.
	//
	// HEAP-OVERRUN ROOT-CAUSE FIX (fact-doc §13): config + flag are NOT sufficient. At the
	// CFG16 rung the gearshift and ARQ control loop still issue MANY stock per-frame /
	// single-frame / CONTROL transmits, each handing a FRAME-sized `out` slot (~15184
	// doubles). The block waveform is ~45552 doubles; branching here for those callers
	// overran their slot and smashed the heap. Require an EXPLICIT per-call block intent
	// (bigblock_emit_as_block), which ONLY the dedicated block driver (bigblock_send_one_
	// block) + the loopback validators set — and only while they pass a block-sized buffer.
	//
	// REPRODUCER HOOK (fact-doc §13.R): MERCURY_BIGBLOCK_OLDGATE=1 restores the PRE-FIX gate
	// (config alone, ignoring the per-call capacity intent) so the in-sim CASE C can
	// demonstrate the fail-before overrun with the SAME binary. Production never sets it;
	// it exists purely for the deterministic local fail-before/pass-after proof.
	bool emit_block = bigblock_emit_as_block;
	{ const char* e = std::getenv("MERCURY_BIGBLOCK_OLDGATE"); if(e && *e && atoi(e)!=0) emit_block = true; }
	if(bigblock_framing_enabled && M != MOD_MFSK && current_configuration == CONFIG_16
		&& emit_block)
	{
		transmit_bigblock(data, nBytes, out);
		return;
	}

	int nReal_data = data_container.nBits - ldpc.P;
	int msB = 0, lsB = 0;
	int frame_size = (nReal_data - outer_code_reserved_bits) / 8;

	if(nBytes > frame_size)
	{
		std::cout<<"message too long.. not sent."<<std::endl;
		return;
	}

	byte_to_bit(data, data_container.data_bit, nBytes);

	// Zero-pad data to full frame_size BEFORE CRC so that CRC covers a
	// fixed-size block.  RX self-check: CRC16([frame_size bytes + CRC]) = 0.
	for(int i = nBytes * 8; i < frame_size * 8; i++)
	{
		data_container.data_bit[i] = 0;
	}

	if(outer_code == CRC16_MODBUS_RTU)
	{
		// Pad byte array too (data may alias data_container.data_byte)
		for(int i = nBytes; i < frame_size; i++)
			data[i] = 0;
		uint16_t crc = CRC16_MODBUS_RTU_calc(data, frame_size);
		msB = (crc & 0xff00) >> 8;
		lsB = crc & 0x00ff;
		byte_to_bit(&lsB, &data_container.data_bit[frame_size * 8], 1);
		byte_to_bit(&msB, &data_container.data_bit[(frame_size + 1) * 8], 1);
	}

	// Zero any remaining non-byte-aligned waste bits after CRC
	for(int i = frame_size * 8 + outer_code_reserved_bits; i < nReal_data; i++)
	{
		data_container.data_bit[i] = 0;
	}

	transmit_bit(data_container.data_bit, out, message_location);
}

void cl_telecom_system::transmit_bit(int* data, double* out, int message_location)
{
	int nVirtual_data=ldpc.N-data_container.nBits;
	int nReal_data=data_container.nBits-ldpc.P;
	float power_normalization=sqrt((double)(ofdm.Nfft*frequency_interpolation_rate));

	for(int i=0;i<nReal_data;i++)
	{
		data_container.data_bit[i]=data[i];
	}

	bit_energy_dispersal(data_container.data_bit, data_container.bit_energy_dispersal_sequence, data_container.data_bit_energy_dispersal, nReal_data);

	for(int i=0;i<nVirtual_data;i++)
	{
		data_container.data_bit_energy_dispersal[nReal_data+i]=data_container.data_bit_energy_dispersal[i];
	}

	ldpc.encode(data_container.data_bit_energy_dispersal,data_container.encoded_data);

	for(int i=0;i<ldpc.P;i++)
	{
		data_container.encoded_data[nReal_data+i]=data_container.encoded_data[i+ldpc.K];
	}

	interleaver(data_container.encoded_data,data_container.bit_interleaved_data,data_container.nBits,bit_interleaver_block_size);

	if(M == MOD_MFSK)
	{
		// MFSK: bits → one-hot subcarrier vectors, directly to framed data
		// In ctrl mode, only modulate first ctrl_nBits interleaved bits (fewer symbols)
		int active_nbits = get_active_nbits();
		mfsk.mod(data_container.bit_interleaved_data, active_nbits, data_container.ofdm_framed_data);

#ifdef MERCURY_GUI_ENABLED
		// Accumulate tone energies across ALL symbols for full-packet view
		{
			int nSymbols = active_nbits / (mfsk.nBits * mfsk.nStreams);
			double gui_E[2][64] = {};
			int gui_peak[2] = {};
			for (int st = 0; st < mfsk.nStreams && st < 2; st++)
			{
				for (int s = 0; s < nSymbols; s++)
				{
					int hop = (s * mfsk.tone_hop_step) % mfsk.M;
					for (int m = 0; m < mfsk.M && m < 64; m++)
					{
						std::complex<double> val = data_container.ofdm_framed_data[
							s * data_container.Nc + mfsk.stream_offsets[st] + ((m + hop) % mfsk.M)];
						gui_E[st][m] += val.real() * val.real() + val.imag() * val.imag();
					}
				}
				// Peak = last symbol's active tone (current tone indicator)
				if (nSymbols > 0) {
					int last_s = nSymbols - 1;
					int hop = (last_s * mfsk.tone_hop_step) % mfsk.M;
					double max_e = -1.0;
					for (int m = 0; m < mfsk.M && m < 64; m++)
					{
						std::complex<double> val = data_container.ofdm_framed_data[
							last_s * data_container.Nc + mfsk.stream_offsets[st] + ((m + hop) % mfsk.M)];
						double e = val.real() * val.real() + val.imag() * val.imag();
						if (e > max_e) { max_e = e; gui_peak[st] = m; }
					}
				}
			}
			gui_push_mfsk_tones(gui_E, gui_peak, mfsk.M, mfsk.nStreams, true);
		}
#endif
	}
	else
	{
		psk.mod(data_container.bit_interleaved_data,data_container.nBits,data_container.modulated_data);
		interleaver(data_container.modulated_data, data_container.ofdm_time_freq_interleaved_data, data_container.nData, time_freq_interleaver_block_size);
		ofdm.framer(data_container.ofdm_time_freq_interleaved_data,data_container.ofdm_framed_data);
	}

	if(M == MOD_MFSK)
	{
		// MFSK preamble: known single-tone symbols (concentrated energy, detectable in weak signal)
		mfsk.generate_preamble(data_container.preamble_data, data_container.preamble_nSymb);
	}
	else
	{
		// OFDM preamble: broadband known symbols (all subcarriers)
		for(int i=0;i<data_container.preamble_nSymb*ofdm.Nc;i++)
		{
			data_container.preamble_data[i]=ofdm.ofdm_preamble[i].value;
		}
	}

	if(M != MOD_MFSK)
	{
		// === DIAG: pre_eq at TX time (remove after debug) ===
		{
			static int tx_preeq_count = 0;
			static int tx_preeq_last_config = -1;
			if(tx_preeq_last_config != current_configuration) { tx_preeq_count = 0; tx_preeq_last_config = current_configuration; }
			if(tx_preeq_count < 2)
			{
				tx_preeq_count++;
				printf("[TX-PREEQ] CONFIG_%d pre_eq[0..4]=(%.4f,%.4f)(%.4f,%.4f)(%.4f,%.4f)(%.4f,%.4f)(%.4f,%.4f)\n",
					current_configuration,
					pre_equalization_channel[0].value.real(), pre_equalization_channel[0].value.imag(),
					pre_equalization_channel[1].value.real(), pre_equalization_channel[1].value.imag(),
					pre_equalization_channel[2].value.real(), pre_equalization_channel[2].value.imag(),
					pre_equalization_channel[3].value.real(), pre_equalization_channel[3].value.imag(),
					pre_equalization_channel[4].value.real(), pre_equalization_channel[4].value.imag());
				fflush(stdout);
			}
		}
		// Pre-equalization (OFDM only, not used for MFSK)
		for(int i=0;i<data_container.preamble_nSymb;i++)
		{
			for(int j=0;j<data_container.Nc;j++)
			{
				data_container.preamble_data[i*data_container.Nc+j]*=pre_equalization_channel[j].value;
			}
		}

		for(int i=0;i<data_container.Nsymb;i++)
		{
			for(int j=0;j<data_container.Nc;j++)
			{
				data_container.ofdm_framed_data[i*data_container.Nc+j]*=pre_equalization_channel[j].value;
			}
		}
	}

	// LEVER P: preamble amortization. eff_preamble is the number of preamble
	// symbols THIS frame actually emits (FULL for anchor/retx/after-FAIL, 1 for
	// MINI tail frames). It is <= data_container.preamble_nSymb (the configured
	// allocation maximum). For MINI (eff=1) only preamble symbol 0 is emitted —
	// that is ofdm_preamble[0], the Schmidl-Cox resync anchor. All preamble
	// emission math below uses eff_preamble; data symbols still start right after
	// the (shortened) preamble, so the frame waveform is genuinely shorter.
	// MFSK keeps the full preamble (override is OFDM-data-only); guard on M.
	int eff_preamble = (M == MOD_MFSK) ? data_container.preamble_nSymb
	                                   : tx_effective_preamble_nsymb();

	for(int i=0;i<eff_preamble;i++)
	{
		ofdm.symbol_mod(&data_container.preamble_data[i*data_container.Nc],&data_container.preamble_symbol_modulated_data[i*data_container.Nofdm]);
	}

	int active_nsymb = get_active_nsymb();

	for(int i=0;i<active_nsymb;i++)
	{
		ofdm.symbol_mod(&data_container.ofdm_framed_data[i*data_container.Nc],&data_container.ofdm_symbol_modulated_data[i*data_container.Nofdm]);
	}

	// TX gain from calibration table (replaces computed mfsk_boost formula)
	double mfsk_boost = 1.0;
	if(M == MOD_MFSK)
	{
		tx_signal_type sig = (mfsk.nStreams == 1) ? TX_SIG_MFSK_1S : TX_SIG_MFSK_2S;
		mfsk_boost = get_tx_gain(sig);
	}
	else
	{
		mfsk_boost = get_tx_gain(TX_SIG_OFDM);
	}

	// Preamble boost: OFDM uses sqrt(2) boost for detection headroom;
	// MFSK preamble is already a concentrated single tone — no boost needed.
	double preamble_boost = (M == MOD_MFSK) ? 1.0 : ofdm.preamble_configurator.boost;

	for(int j=0;j<data_container.Nofdm*eff_preamble;j++)
	{
		data_container.preamble_symbol_modulated_data[j]/=power_normalization;
		data_container.preamble_symbol_modulated_data[j]*=sqrt(output_power_Watt)*preamble_boost*mfsk_boost;
	}

	for(int j=0;j<data_container.Nofdm*active_nsymb;j++)
	{
		data_container.ofdm_symbol_modulated_data[j]/=power_normalization;
		data_container.ofdm_symbol_modulated_data[j]*=sqrt(output_power_Watt)*mfsk_boost;
	}

	// LEVER P: emitted frame size = (eff preamble + data) symbols. This is the
	// actual length of the waveform written to `out` (<= total_frame_size, which
	// is sized for the FULL preamble). tx_last_emitted_frame_samples lets the
	// batch assembler in send_batch() pack the next frame contiguously.
	int eff_preamble_samples = data_container.Nofdm*eff_preamble*frequency_interpolation_rate;
	int eff_data_samples     = data_container.Nofdm*active_nsymb*frequency_interpolation_rate;
	int eff_frame_samples    = eff_preamble_samples + eff_data_samples;
	tx_last_emitted_frame_samples = eff_frame_samples;

	// Apply test TX carrier offset for frequency sync testing
	double tx_carrier = carrier_frequency + test_tx_carrier_offset;
	ofdm.baseband_to_passband(data_container.preamble_symbol_modulated_data,data_container.Nofdm*eff_preamble,data_container.passband_data_tx,sampling_frequency,tx_carrier,carrier_amplitude,frequency_interpolation_rate);
	ofdm.baseband_to_passband(data_container.ofdm_symbol_modulated_data,data_container.Nofdm*active_nsymb,&data_container.passband_data_tx[eff_preamble_samples],sampling_frequency,tx_carrier,carrier_amplitude,frequency_interpolation_rate);

	ofdm.peak_clip(data_container.passband_data_tx, eff_preamble_samples,ofdm.preamble_papr_cut);
	ofdm.peak_clip(&data_container.passband_data_tx[eff_preamble_samples], eff_data_samples,ofdm.data_papr_cut);

	if(message_location==NO_FILTER_MESSAGE)
	{
		for(int i=0;i<eff_frame_samples;i++)
		{
			*(out+i)=data_container.passband_data_tx[i];
		}
		return;
	}

	if(message_location==SINGLE_MESSAGE)
	{
		ofdm.FIR_tx1.apply(data_container.passband_data_tx,data_container.passband_data_tx_filtered_fir_1,data_container.total_frame_size);
		ofdm.FIR_tx2.apply(data_container.passband_data_tx_filtered_fir_1,data_container.passband_data_tx_filtered_fir_2,data_container.total_frame_size);

		for(int i=0;i<data_container.total_frame_size;i++)
		{
			*(out+i)=data_container.passband_data_tx_filtered_fir_2[i];
		}
		//		st_power_measurment power_measurment_preamble=ofdm.measure_signal_power_avg_papr(out, data_container.Nofdm*data_container.preamble_nSymb*frequency_interpolation_rate);
		//		st_power_measurment power_measurment_modulated_data=ofdm.measure_signal_power_avg_papr(&out[data_container.Nofdm*data_container.preamble_nSymb*frequency_interpolation_rate], data_container.Nofdm*data_container.Nsymb*frequency_interpolation_rate);
		//
		//		std::cout<<"preamble power: avg="<<power_measurment_preamble.avg;
		//		std::cout<<" max="<<power_measurment_preamble.max;
		//		std::cout<<" PAPR="<<power_measurment_preamble.papr_db<<" db";
		//		std::cout<<" mod_data power: avg="<<power_measurment_modulated_data.avg;
		//		std::cout<<" max="<<power_measurment_modulated_data.max;
		//		std::cout<<" PAPR="<<power_measurment_modulated_data.papr_db<<" db"<<std::endl;

		// TX-PEAK: measure peak passband amplitude (fires once per config)
		{
			static int peak_last_config = -1;
			if(peak_last_config != current_configuration)
			{
				peak_last_config = current_configuration;
				double peak = 0, rms_sum = 0;
				for(int i = 0; i < data_container.total_frame_size; i++)
				{
					double s = fabs(*(out+i));
					if(s > peak) peak = s;
					rms_sum += s * s;
				}
				double rms = sqrt(rms_sum / data_container.total_frame_size);
				printf("[TX-PEAK] CONFIG_%d peak=%.6f rms=%.6f papr=%.1fdB total_samples=%d\n",
					current_configuration, peak, rms, 20.0*log10(peak/rms), data_container.total_frame_size);
			}
		}

		return;
	}


	if(message_location==FIRST_MESSAGE)
	{
		for(int i=0;i<data_container.total_frame_size;i++)
		{
			data_container.passband_data_tx_buffer[data_container.total_frame_size + i]=data_container.passband_data_tx[i];//TODO increasing value
			data_container.passband_data_tx_buffer[2 * data_container.total_frame_size + i]=data_container.passband_data_tx[i];
		}
	}

	if(message_location==MIDDLE_MESSAGE || message_location==FLUSH_MESSAGE)
	{
		for(int i=0;i<data_container.total_frame_size;i++)
		{
			data_container.passband_data_tx_buffer[2 * data_container.total_frame_size + i]=data_container.passband_data_tx[i];
		}
	}


	ofdm.FIR_tx1.apply(&data_container.passband_data_tx_buffer[data_container.total_frame_size/2],data_container.passband_data_tx_filtered_fir_1,2*data_container.total_frame_size);
	ofdm.FIR_tx2.apply(data_container.passband_data_tx_filtered_fir_1,data_container.passband_data_tx_filtered_fir_2,2*data_container.total_frame_size);

	for(int i=0;i<data_container.total_frame_size;i++)
	{
		*(out+i)=data_container.passband_data_tx_filtered_fir_2[data_container.total_frame_size/2+i];
	}
	shift_left(data_container.passband_data_tx_buffer, 3*data_container.total_frame_size, data_container.total_frame_size);

	int PAPR_Meas=NO;

	int MER_Meas=NO;

	if(PAPR_Meas==YES)
	{
		st_power_measurment power_measurment_preamble=ofdm.measure_signal_power_avg_papr(out, data_container.Nofdm*data_container.preamble_nSymb*frequency_interpolation_rate);
		st_power_measurment power_measurment_modulated_data=ofdm.measure_signal_power_avg_papr(&out[data_container.Nofdm*data_container.preamble_nSymb*frequency_interpolation_rate], data_container.Nofdm*data_container.Nsymb*frequency_interpolation_rate);
		std::cout<<"preamble power: avg="<<power_measurment_preamble.avg;
		std::cout<<" max="<<power_measurment_preamble.max;
		std::cout<<" PAPR="<<power_measurment_preamble.papr_db<<" db";
		std::cout<<" mod_data power: avg="<<power_measurment_modulated_data.avg;
		std::cout<<" max="<<power_measurment_modulated_data.max;
		std::cout<<" PAPR="<<power_measurment_modulated_data.papr_db<<" db"<<std::endl;
	}

	if(MER_Meas==YES)
	{
		ofdm.FIR_tx1.apply(data_container.passband_data_tx_buffer,&data_container.passband_data_tx_filtered_fir_1[data_container.total_frame_size/2],2.5*data_container.total_frame_size);
		ofdm.FIR_tx2.apply(data_container.passband_data_tx_filtered_fir_1,data_container.passband_data_tx_filtered_fir_2,2.5*data_container.total_frame_size);

		ofdm.passband_to_baseband(&data_container.passband_data_tx_filtered_fir_2[data_container.total_frame_size+data_container.total_frame_size/2],data_container.total_frame_size,data_container.baseband_data_interpolated,sampling_frequency,carrier_frequency,carrier_amplitude,1,&ofdm.FIR_rx_data);

		ofdm.rational_resampler(data_container.baseband_data_interpolated, (data_container.Nofdm*(data_container.Nsymb+data_container.preamble_nSymb))*frequency_interpolation_rate, data_container.baseband_data, data_container.interpolation_rate, DECIMATION);

		for(int i=0;i<data_container.Nsymb;i++)
		{
			ofdm.symbol_demod(&data_container.baseband_data[i*data_container.Nofdm+data_container.Nofdm*data_container.preamble_nSymb],&data_container.ofdm_symbol_demodulated_data[i*data_container.Nc]);
		}
		ofdm.automatic_gain_control(data_container.ofdm_symbol_demodulated_data);
		ofdm.ZF_channel_estimator(data_container.ofdm_symbol_demodulated_data);
		ofdm.channel_equalizer(data_container.ofdm_symbol_demodulated_data,data_container.equalized_data);
		ofdm.deframer(data_container.equalized_data,data_container.ofdm_deframed_data);

		float MER=ofdm.measure_SNR(data_container.ofdm_time_freq_interleaved_data,data_container.ofdm_deframed_data,data_container.nData);
		std::cout<<"MER ="<<MER<<std::endl;
	}

}

st_receive_stats cl_telecom_system::receive_bit(double *data, int* out)
{
	int nReal_data=data_container.nBits-ldpc.P;

	st_receive_stats tmp=receive_byte(data,data_container.hd_decoded_data_byte);
	byte_to_bit(data_container.hd_decoded_data_byte, out, nReal_data/8);

	return tmp;
}

st_receive_stats cl_telecom_system::receive_byte(double *data, int* out)
{
	// P1: big-block framing branch (gated; default OFF). When set, receive_byte
	// acquires ONCE over the captured passband and decodes the K codewords with the
	// channel-adaptive estimator + CSI-LLR, returning the per-codeword decode result
	// in receive_stats (+ the decoded info bits in out). NO ARQ change in P1.
	// P3 HW FIX: CFG16-ONLY (matches transmit_byte). The flag FORCED on for a live ARQ
	// session would otherwise route every CONFIG_0..15 per-frame decode during the climb
	// into receive_bigblock with the wrong geometry, stalling the gearshift. Validators
	// run at CONFIG_16 (-s 16), so they are unaffected.
	// GAP-3 CARVE-GATE HARDENING (cfg16-controlack-hold): the big-block route is
	// SUPPRESSED for this one call when bigblock_rx_force_stock is set. The ARQ
	// carve gate sets it after a CFG16 acquisition fails the cw0 wire-header CRC
	// check (i.e. the audio is a single OFDM control frame / stale / noise, NOT a
	// real K-codeword block) and re-invokes receive_byte to decode it on the STOCK
	// per-frame path. Without this, ANY CFG16 OFDM audio (including the SET_CONFIG
	// control turnaround) is unconditionally carved as a fake block and never
	// parsed as control. Default false -> the production block decode is unchanged.
	if(bigblock_framing_enabled && M != MOD_MFSK && current_configuration == CONFIG_16
		&& !bigblock_rx_force_stock)
	{
		return receive_bigblock(data, out);
	}

	float variance = 1.0f;
	int nVirtual_data=ldpc.N-data_container.nBits;
	int nReal_data=data_container.nBits-ldpc.P;
	double freq_offset_measured=0;
	receive_stats.message_decoded=NO;
	receive_stats.frame_overflow_symbols=0;
	receive_stats.frame_data_missing=false;
	receive_stats.frame_skip_var_aborted=false;
	receive_stats.sync_trials=0;
	receive_stats.iterations_done = -1;
	receive_stats.crc = 0;
	receive_stats.SNR = -99.9;
	receive_stats.all_zeros = NO;
	receive_stats.coarse_metric = 0.0;
	receive_stats.mean_H = -1.0;
	receive_stats.last_eff_preamble_nsymb = data_container.preamble_nSymb;  // LEVER P: FULL until a MINI tail frame is extracted

	// Timing breakdown
	double timing_pb_tsync_ms = 0, timing_pb_data_ms = 0, timing_ldpc_ms = 0;
	auto timing_total_start = std::chrono::steady_clock::now();

	int step=100;
	int pream_symb_loc;

	// LEVER P: preamble amortization (OFDM only). rx_eff_preamble is the number
	// of preamble symbols the frame about to be extracted is expected to carry.
	// It starts at the configured FULL length and is dropped to MINI (1) when the
	// batch-predict verify locks a tail frame (ofdm_batch_active). On any full-
	// buffer (re-anchor / initial) search it stays/returns to FULL — which
	// matches the TX force-full-on-anchor/after-FAIL rule. Used at the data-
	// symbol demod offset and frame-extraction size below. MFSK is unaffected
	// (the amortization is OFDM-data-only). See
	// fact-documents/data-flow-preamble-amortization.md §2.
	int rx_eff_preamble = data_container.preamble_nSymb;
	if(!preamble_amortization_enabled) rx_eff_preamble = data_container.preamble_nSymb;

	// Coarse frequency offset - starts at 0, only searched on trial 1 if trial 0 fails
	double coarse_freq_offset = 0.0;

	if(mfsk_fixed_delay >= 0)
	{
		// Known delay (BER test) - bypass time_sync entirely.
		// Delay set per-frame in passband_test_EsN0(), cleared after use.
		// Adjust for nUnder: in BER mode nUnder is always 0 (no capture thread),
		// so this is a no-op, but kept for safety.
		int nUnder_adj = data_container.nUnder_processing_events.load();
		int symbol_period = data_container.Nofdm * frequency_interpolation_rate;
		int adjusted_delay = mfsk_fixed_delay - nUnder_adj * symbol_period;
		if(adjusted_delay < 0) adjusted_delay = 0;

		receive_stats.delay = adjusted_delay;
		mfsk_fixed_delay = -1;
		pream_symb_loc = receive_stats.delay / (data_container.Nofdm * data_container.interpolation_rate);
		if(pream_symb_loc < 1) { pream_symb_loc = 1; }
		receive_stats.signal_stregth_dbm = 0;
	}
	else
	{
		// Quick passband energy diagnostic (first 3 calls per config)
		{
			static int pb_diag_count = 0;
			static int pb_diag_last_key = -1;
			int pb_key = current_configuration * 10 + narrowband_enabled;
			if(pb_key != pb_diag_last_key) { pb_diag_count = 0; pb_diag_last_key = pb_key; }
			if(pb_diag_count < 3)
			{
				pb_diag_count++;
				int pb_total = data_container.Nofdm * data_container.buffer_Nsymb * frequency_interpolation_rate;
				double e_total = 0, pk = 0;
				for(int i = 0; i < pb_total; i++)
				{
					double v = ((double*)data)[i];
					e_total += v*v;
					if(fabs(v) > pk) pk = fabs(v);
				}
				printf("[PB-ENERGY] rms=%.6f peak=%.6f samples=%d output_power=%.3f\n",
					sqrt(e_total/pb_total), pk, pb_total, output_power_Watt);
			}
		}
		auto t0_pb = std::chrono::steady_clock::now();
		{
			static int p2b_diag = 0;
			if(p2b_diag < 5) {
				p2b_diag++;
				printf("[P2B-DIAG] M=%.0f carrier_freq=%.1f carrier_amp=%.6f fs=%.0f interp=%d buf_samples=%d\n",
					M, carrier_frequency, carrier_amplitude, sampling_frequency,
					frequency_interpolation_rate,
					data_container.Nofdm*data_container.buffer_Nsymb*frequency_interpolation_rate);
				fflush(stdout);
			}
		}
		// RX passband normalization + impulse noise blanking.
		// The OFDM pipeline (channel estimation, equalization, LLR computation)
		// assumes RX signal at roughly the same level as TX output_power_Watt.
		// External audio paths (SGTL5000 via IONOS, radio links) attenuate
		// the signal by 20-50 dB. Without normalization, |H| ≈ 0.01 instead
		// of ≈ 1.0, the equalizer amplifies noise, and LDPC gets garbage.
		// Normalize: scale passband so RMS matches sqrt(output_power_Watt).
		// This is a signal processing normalization, not radio AGC — it
		// preserves SNR and is equivalent to what the TX produced.
		if(M != MOD_MFSK)
		{
			// P2.2 — extracted verbatim into rx_passband_normalize_and_blank() so the
			// big-block RX path (receive_bigblock) runs the SAME normalization +
			// impulse-blanking before its estimator. Behavior here is unchanged.
			int pb_samples = data_container.Nofdm * data_container.buffer_Nsymb * frequency_interpolation_rate;
			rx_passband_normalize_and_blank((double*)data, pb_samples);
		}
		// Plan-B Step 6c: the eager full-rate time_sync FIR over the WHOLE
		// passband buffer — the ~92% RPi RX-side idle-scan CPU cost — is
		// REMOVED from the production OFDM path. Every OFDM time_sync consumer
		// has been converted: the coarse Schmidl-Cox search (sites 2-7, 2247 +
		// the primary site 3) runs on baseband_data_decimated; the fine
		// refinement (Step-5 site-3 fine slice + Step-6b site-8) mixes+FIRs a
		// small scoped full-rate slice from raw `data` on demand; every
		// energy/signal gate (Step 6a) reads baseband_data_decimated.
		// baseband_data_interpolated is no longer the full-buffer full-rate
		// buffer for OFDM — it is only used as decimated-FIR scratch by the
		// data-extraction path (passband_to_baseband_decimated writes there).
		//
		// EXCEPTION — MFSK (§7-inventory correction, Plan-B Step 6c): the
		// MFSK preamble detectors time_sync_mfsk_corr / time_sync_mfsk
		// (telecom_system.cc, MFSK branch) ALSO consume the full-rate buffer,
		// and time_sync_mfsk_corr does a 4x sub-symbol OVERSAMPLED search
		// (P1_OVERSAMPLE=4, ofdm.cc, Bug #44) — it needs FINER-than-decimated
		// resolution and cannot run on baseband_data_decimated without a DSP
		// redesign. Plan B converted only the OFDM time_sync consumers; the
		// conservative fix keeps the eager full-rate FIR ONLY when
		// M == MOD_MFSK. For every OFDM config (the point of the plan) the
		// full-rate idle-scan FIR is gone — the decimated path below replaces
		// it.
		if(M == MOD_MFSK)
		{
			ofdm.passband_to_baseband((double*)data,data_container.Nofdm*data_container.buffer_Nsymb*frequency_interpolation_rate,data_container.baseband_data_interpolated,sampling_frequency,carrier_frequency,carrier_amplitude,1,&ofdm.FIR_rx_time_sync);
		}

		// Populate the decimated-rate buffer — the hot-path time_sync FIR now
		// runs ONLY at the decimated rate (M× cheaper) for OFDM. Bit-exact
		// equivalent to the full-rate FIR followed by picking every Mth sample
		// (see fir_filter.cc:227).
		//
		// Plan-C Step 1: the WB MFSK FFT-energy detector time_sync_mfsk
		// (ofdm.cc) now also reads baseband_data_decimated (it has no
		// sub-symbol-oversampling constraint — it scans on a 1-symbol grid and
		// reads a single decimation phase). So populate the decimated buffer
		// for MFSK too, but ONLY when the FFT-energy path is selected
		// (mfsk_corr_template == NULL). When mfsk_corr_template != NULL the
		// cross-correlation detector time_sync_mfsk_corr is used instead — it
		// still consumes baseband_data_interpolated (Bug #44 4x sub-symbol
		// oversampled search; Plan-C Steps 2-4, not done here), so populating
		// the decimated buffer for that path would be pure wasted work.
		if(M != MOD_MFSK || ofdm.mfsk_corr_template == NULL)
		{
			int p2b_full_size = data_container.Nofdm * data_container.buffer_Nsymb * frequency_interpolation_rate;
			int p2b_M = data_container.interpolation_rate;
			ofdm.passband_to_baseband_decimated((double*)data, p2b_full_size,
				data_container.baseband_data_decimated,
				sampling_frequency, carrier_frequency, carrier_amplitude,
				p2b_M, &ofdm.FIR_rx_time_sync);
		}
		auto t1_pb = std::chrono::steady_clock::now();
		timing_pb_tsync_ms = std::chrono::duration<double, std::milli>(t1_pb - t0_pb).count();


		// Plan-B Step 6a (site 1, signal-strength gate): mean power per sample
		// in dBm is rate-invariant — the anti-alias FIR passes the signal band,
		// so decimating preserves mean power. Read the decimated buffer; this
		// is a whole-buffer read, so the decimated count is Nofdm*buffer_Nsymb.
		receive_stats.signal_stregth_dbm=ofdm.measure_signal_stregth(data_container.baseband_data_decimated, data_container.Nofdm*data_container.buffer_Nsymb);

		// Pre-scan passband for signal region to constrain OFDM preamble search.
		// After ACK TX + buffer flush, the buffer is: [zeros | VB-Cable silence | signal | silence].
		// VB-Cable silence has a DC offset that produces high GI+halfsym correlation
		// (metric ~0.97), competitive with real preambles. Scanning peak passband
		// amplitude identifies where real signal starts, letting us skip the silence.
		// Uses peak absolute value (like BUF-ENERGY), NOT mean squared — OFDM has
		// ~10-12 dB PAPR with 50 subcarriers, so mean squared is ~0.02 even when
		// peak is ~0.5. Threshold 0.1 peak: silence ~0, TX ramp ~0.02, signal 0.3+.
		int signal_start_symb = 0;
		if(M != MOD_MFSK)
		{
			int sym_samples = data_container.Nofdm * frequency_interpolation_rate;
			int buf_samples = data_container.Nofdm * data_container.buffer_Nsymb * frequency_interpolation_rate;
			double max_peak_seen = 0.0;
			for(int s = 0; s < data_container.buffer_Nsymb; s++)
			{
				int offset = s * sym_samples;
				double peak = 0.0;
				for(int i = 0; i < sym_samples && (offset + i) < buf_samples; i++)
				{
					double v = fabs(data[offset + i]);
					if(v > peak) peak = v;
				}
				if(peak > max_peak_seen) max_peak_seen = peak;
				if(peak > 0.1)
				{
					signal_start_symb = s;
					if(g_verbose)
						printf("[PRESCAN] signal_start_symb=%d peak=%.4f\n", s, peak);
					break;
				}
			}
			if(g_verbose && signal_start_symb == 0)
				printf("[PRESCAN] no signal found, max_peak=%.6f\n", max_peak_seen);
		}

		if(M == MOD_MFSK)
		{
			// MFSK preamble detection
			// Anti-re-decode: skip past where previous preamble sits in buffer
			int search_start = receive_stats.mfsk_search_raw - data_container.nUnder_processing_events;
			if(search_start < 0) search_start = 0;
			double mfsk_sync_metric = 0;

			if(ofdm.mfsk_corr_template != NULL)
			{
				// Waveform cross-correlation (2000:1 noise discrimination, NB+WB)
				receive_stats.delay = ofdm.time_sync_mfsk_corr(
					data_container.baseband_data_interpolated,
					data_container.Nofdm * data_container.buffer_Nsymb * frequency_interpolation_rate,
					data_container.interpolation_rate,
					search_start, &mfsk_sync_metric);
			}
			else
			{
				// WB: FFT energy ratio detection.
				// Plan-C Step 1: read the single-phase decimated buffer
				// (baseband_data_decimated) with interpolation_rate = 1 — every
				// "* interpolation_rate" inside time_sync_mfsk collapses, the
				// FFT reads contiguous decimated samples, the 1-symbol-grid scan
				// is unchanged. time_sync_mfsk has NO Bug #44 sub-symbol
				// oversampling constraint (it scans on a full-symbol grid and
				// reads decimation phase 0 only), so this is a straight
				// re-point. The returned delay is now a DECIMATED index
				// (best_sym_idx * Nofdm) — multiply by interpolation_rate to
				// restore a full-rate receive_stats.delay (frame extraction and
				// the energy gates all expect a full-rate index).
				int mfsk_delay_dec = ofdm.time_sync_mfsk(
					data_container.baseband_data_decimated,
					data_container.Nofdm * data_container.buffer_Nsymb,
					1,
					data_container.preamble_nSymb, mfsk.preamble_tones,
					mfsk.M, mfsk.nStreams, mfsk.stream_offsets,
					search_start, &mfsk_sync_metric);
				receive_stats.delay = (mfsk_delay_dec < 0) ? -1
					: mfsk_delay_dec * data_container.interpolation_rate;
			}

			if(receive_stats.delay < 0)
			{
				st_receive_stats no_preamble = {};
				no_preamble.message_decoded = NO;
				no_preamble.delay = -1;
				no_preamble.signal_stregth_dbm = receive_stats.signal_stregth_dbm;
				return no_preamble;
			}

		}
		else
		{
			// Unified OFDM preamble detection (NB + WB, initial + batch).
			//
			// Schmidl-Cox autocorrelation: exploits L-sample periodicity of the
			// preamble (even-only subcarriers WB, every-2nd NB). Correlates the
			// received signal with itself — immune to audio path distortion.
			//   Coarse: GI stride over full buffer (halfsym_2phase)
			//   Fine: baseband stride within ±1 GI of coarse peak
			//
			// Metric: energy-weighted normalized correlation → [0,1].
			// Amplitude-independent: works at any RX gain / HF fading level.
			// NB (Nc=10) has higher metric variance than WB (Nc=50) because
			// fewer subcarriers give less averaging in the autocorrelation.
			// This causes OFDM data symbols to produce false peaks at 0.15-0.45
			// instead of staying below 0.15. Raise threshold for NB to prevent
			// false detections that waste decode time and push real preambles
			// beyond the buffer boundary (causing NAcks).
			// §7.13.29 (Proposal A) — cross-check pass uses stricter threshold.
			double preamble_detect_threshold = sack_cross_check_mode
				? (narrowband_enabled ? 0.75 : 0.65)
				: (narrowband_enabled ? 0.30 : 0.15);

			// BER test: known delay bypasses detection entirely (same as mfsk_fixed_delay).
			// The forced delay positions the preamble exactly; no detection needed.
			if(ofdm_forced_delay >= 0)
			{
				// BER test: known delay bypasses detection (same as mfsk_fixed_delay).
				receive_stats.delay = ofdm_forced_delay;
				receive_stats.coarse_metric = 10.0;

				// Detection self-test: verify matched filter on first BER frame only.
				// Runs once per config to catch template mismatches without spamming
				// on low-SNR frames (where noise makes detection impossible).
				{
					static int selftest_config = -1;
					if(selftest_config != current_configuration)
					{
						selftest_config = current_configuration;
						int interp_st = data_container.interpolation_rate;
						int sym_st = data_container.Nofdm * interp_st;
						int buf_interp_st = data_container.Nofdm * data_container.buffer_Nsymb * interp_st;
						int margin = 4 * sym_st;
						int st_start = ofdm_forced_delay - margin;
						if(st_start < 0) st_start = 0;
						int st_size = 2 * margin + data_container.preamble_nSymb * sym_st;
						if(st_start + st_size > buf_interp_st) st_size = buf_interp_st - st_start;
						if(st_size > 0)
						{
							// Plan-B Step 6c (site 1, BER self-test — once per
							// config, diagnostic only): run the coarse search on
							// the decimated buffer. st_start and st_size are
							// multiples of sym_st (= Nofdm*M), so /M is exact;
							// the original used step = interp_st (= M), which
							// only ever evaluates the M-grid, so the decimated
							// step=1 search lands on the SAME grid — resolution-
							// equivalent. Result mapped back to full rate.
							// This is exactly the Step-3 recovery-site pattern.
							int st_M = data_container.interpolation_rate;
							int st_start_dec = st_start / st_M;
							int st_size_dec = st_size / st_M;
							TimeSyncResult selftest = ofdm.time_sync_preamble_halfsym(
								&data_container.baseband_data_decimated[st_start_dec],
								st_size_dec, 1, 1);
							int detected_delay = st_start + selftest.delay * st_M;
							int gi_interp_st = data_container.Ngi * interp_st;
							printf("[BER-DET] config=%d metric=%.4f delay=%d expected=%d %s\n",
								current_configuration,
								selftest.correlation, detected_delay, ofdm_forced_delay,
								(selftest.correlation < preamble_detect_threshold
								 || abs(detected_delay - ofdm_forced_delay) > gi_interp_st)
								? "WARN-lowSNR" : "OK");
						}
					}
				}
			}
			else
			{

			int interp = data_container.interpolation_rate;
			int sym_samples = data_container.Nofdm * interp;
			int buf_interp = data_container.Nofdm * data_container.buffer_Nsymb * interp;
			int ofdm_skip = receive_stats.ofdm_search_raw - data_container.nUnder_processing_events;
			if(ofdm_skip < 0) ofdm_skip = 0;

			int search_offset = 0, search_size = 0;

			bool batch_verified = false;

			// LEVER P: in BATCH-predict mode the frame about to be located is a
			// tail frame, which the TX emitted with a MINI (1-symbol) preamble
			// (anchor frame 0 came in via the INITIAL full search). Expect MINI
			// here; if the predict-verify fails and we fall through to a full
			// search below, rx_eff_preamble is reset to FULL (re-anchor).
			bool rx_batch_predict_mode = preamble_amortization_enabled
				&& M != MOD_MFSK
				&& receive_stats.ofdm_batch_active && receive_stats.ofdm_search_raw > 0;
			if(rx_batch_predict_mode) rx_eff_preamble = 1;

			if(receive_stats.ofdm_batch_active && receive_stats.ofdm_search_raw > 0)
			{
				// BATCH mode: predict + verify. After successful decode, the next
				// preamble position is predictable from ofdm_skip. Try a tiny
				// verify window first; fall back to wider search on failure.
				int gi_interp = data_container.Ngi * interp;
				// LEVER P: the verify correlation window spans the MINI preamble
				// length (rx_eff_preamble symbols), not the full 4. preamble_interp
				// is the MINI preamble's sample span.
				int verify_nsym = rx_batch_predict_mode ? rx_eff_preamble : data_container.preamble_nSymb;
				int preamble_interp = verify_nsym * sym_samples;
				int predicted_pos = ofdm_skip * sym_samples + (int)receive_stats.ofdm_drift_per_frame;
				if(predicted_pos < 0) predicted_pos = 0;

				// Verify in ±2*gi_interp window around prediction
				int verify_start = predicted_pos - 2 * gi_interp;
				if(verify_start < 0) verify_start = 0;
				int verify_size = 4 * gi_interp + preamble_interp;
				if(verify_start + verify_size > buf_interp)
					verify_size = buf_interp - verify_start;

				if(verify_size > 0)
				{
					// Plan-B Step 4 (site 2, BATCH predict-verify — HOT): run
					// the tiny verify-window coarse search on the decimated
					// buffer. verify_start is NOT guaranteed M-aligned —
					// predicted_pos carries (int)ofdm_drift_per_frame which is
					// an arbitrary integer — so floor verify_start to the
					// M-grid and extend the decimated window to still cover
					// [verify_start, verify_start+verify_size). The original
					// used step = interp (= M), so this is resolution-
					// equivalent up to the M-grid PHASE (the old grid was
					// {verify_start + k·M}, the decimated grid is {k·M}); the
					// resulting delay can differ by < M samples. That is
					// harmless: (a) the ±2·gi_interp verify margin is ≫ M,
					// (b) the trial loop's site-8 fine sync re-refines
					// receive_stats.delay, (c) drift is computed in full-rate
					// units below so the drift IIR stays in full-rate units
					// (its resolution drops from 1 to ~M/2 samples — swamped
					// by the verify margin).
					int v_M = data_container.interpolation_rate;
					int verify_start_dec = verify_start / v_M;       // floor to M-grid
					int verify_start_full = verify_start_dec * v_M;
					int verify_end = verify_start + verify_size;
					int verify_size_dec = (verify_end - verify_start_full + v_M - 1) / v_M;
					int dec_buf = data_container.Nofdm * data_container.buffer_Nsymb;
					if(verify_start_dec + verify_size_dec > dec_buf)
						verify_size_dec = dec_buf - verify_start_dec;
					// LEVER P: correlate the verify over the MINI preamble length.
					TimeSyncResult verify = ofdm.time_sync_preamble_halfsym(
						&data_container.baseband_data_decimated[verify_start_dec],
						verify_size_dec, 1, 1, 0.0,
						rx_batch_predict_mode ? verify_nsym : -1);
					verify.delay = verify.delay * v_M + verify_start_full;

					if(verify.correlation >= preamble_detect_threshold)
					{
						// Prediction verified — use directly, skip wider search
						receive_stats.delay = verify.delay;
						receive_stats.coarse_metric = verify.correlation;
						int drift = (int)receive_stats.delay - predicted_pos;
						receive_stats.ofdm_drift_per_frame = 0.8 * receive_stats.ofdm_drift_per_frame + 0.2 * drift;
						batch_verified = true;
					}
				}

				// LEVER P (INC-3): MINI re-pin. The ±2·gi verify can miss a
				// 1-symbol MINI preamble (short autocorrelation, low metric), or
				// the predicted position may sit at the buffer edge. Before
				// abandoning the batch lock and full-searching (which on a MINI
				// schedule finds DATA/anchor sub-peaks far from the real frame and
				// desyncs the whole batch), run a WIDER MINI-nsym coarse search in
				// a bounded window around the prediction. Only if that also fails
				// do we treat it as a re-anchor / loss and full-search.
				if(!batch_verified && rx_batch_predict_mode)
				{
					int wide_half = 8 * gi_interp;               // wider than the ±2·gi verify
					int repin_start = predicted_pos - wide_half;
					if(repin_start < ofdm_skip * sym_samples) repin_start = ofdm_skip * sym_samples;
					if(repin_start < 0) repin_start = 0;
					int repin_size = 2 * wide_half + preamble_interp + sym_samples;
					if(repin_start + repin_size > buf_interp)
						repin_size = buf_interp - repin_start;
					if(repin_size > preamble_interp)
					{
						int r_M = data_container.interpolation_rate;
						int repin_start_dec = repin_start / r_M;
						int repin_start_full = repin_start_dec * r_M;
						int repin_end = repin_start + repin_size;
						int repin_size_dec = (repin_end - repin_start_full + r_M - 1) / r_M;
						int rdec_buf = data_container.Nofdm * data_container.buffer_Nsymb;
						if(repin_start_dec + repin_size_dec > rdec_buf)
							repin_size_dec = rdec_buf - repin_start_dec;
						TimeSyncResult repin = ofdm.time_sync_preamble_halfsym(
							&data_container.baseband_data_decimated[repin_start_dec],
							repin_size_dec, 1, 1, 0.0, verify_nsym);
						repin.delay = repin.delay * r_M + repin_start_full;
						if(repin.correlation >= preamble_detect_threshold)
						{
							receive_stats.delay = repin.delay;
							receive_stats.coarse_metric = repin.correlation;
							int drift = (int)receive_stats.delay - predicted_pos;
							receive_stats.ofdm_drift_per_frame = 0.8 * receive_stats.ofdm_drift_per_frame + 0.2 * drift;
							batch_verified = true;
						}
					}
				}

				// LEVER P (INC-3): DEFER-not-search at the buffer edge. Before
				// abandoning the batch lock to a full-buffer search, check whether
				// the PREDICTED MINI tail frame extends beyond the samples we hold.
				// On a MINI schedule the tail preambles march toward the buffer end
				// (each frame advances only Nsymb+1 symbols), so a tail frame's
				// preamble + data routinely straddles the buffer edge: its preamble
				// is detectable but its data symbols haven't arrived yet. A full-
				// buffer search there does NOT help — it finds the just-decoded
				// frame's DATA-region Schmidl-Cox sub-peaks (metric ~0.18-0.28) and
				// desyncs the whole batch. The correct response is the SAME one the
				// FULL-preamble baseline uses: signal the ARQ layer to capture more
				// audio (frame_overflow_symbols), let the buffer shift bring the
				// complete MINI frame into the verify window, and re-enter batch-
				// predict next dispatch. ofdm_defer_overflow_enabled gates this the
				// same way as the primary defer site (telecom_system.cc:1625).
				if(!batch_verified && rx_batch_predict_mode && mfsk_fixed_delay < 0
					&& ofdm_defer_overflow_enabled)
				{
					int mini_active_nsymb = get_active_nsymb();
					// rx_eff_preamble is still the MINI length (1) here — not yet
					// reset to FULL. predicted_pos is the predicted MINI preamble
					// start (full-rate samples); the MINI frame spans
					// (rx_eff_preamble + Nsymb) symbols past it.
					int mini_frame_end = predicted_pos
						+ (rx_eff_preamble + mini_active_nsymb) * sym_samples;
					if(mini_frame_end > buf_interp)
					{
						receive_stats.message_decoded = NO;
						int overflow_samples = mini_frame_end - buf_interp;
						receive_stats.frame_overflow_symbols =
							(overflow_samples + sym_samples - 1) / sym_samples;
						return receive_stats;
					}
				}

				if(!batch_verified)
				{
					// Prediction failed — search full remaining buffer from
					// ofdm_skip onwards (same as initial search).  The narrow
					// forward_look=40 cap was causing preambles >40 symbols
					// past ofdm_skip to be missed, especially after turnaround
					// gaps where the next preamble arrives much later.
					// LEVER P: a full-buffer search re-acquires from scratch — the
					// frame it lands on is a re-anchor (TX forces FULL after any
					// gap / loss), so expect a FULL preamble again.
					rx_eff_preamble = data_container.preamble_nSymb;
					int effective_start = (ofdm_skip > signal_start_symb) ? ofdm_skip : signal_start_symb;
					search_offset = effective_start * sym_samples;
					search_size = buf_interp - search_offset;
					receive_stats.ofdm_drift_per_frame = 0.0;
				}
			}
			else
			{
				// INITIAL search: full buffer, skip past decoded frames.
				int effective_start = (ofdm_skip > signal_start_symb) ? ofdm_skip : signal_start_symb;
				search_offset = effective_start * sym_samples;
				search_size = buf_interp - search_offset;
			}

			if(!batch_verified)
			{
				// Schmidl-Cox autocorrelation: two-phase (coarse at GI stride,
				// fine at baseband stride). Correlates signal with itself —
				// immune to audio path distortion.
				//
				// Early exit (0.5): find the FIRST preamble above threshold
				// instead of the maximum. Prevents later frames with higher
				// energy-weighted metric from shadowing earlier ones (seq=00)
				// when multiple back-to-back frames are in the buffer.
				// Batch prediction handles sequential frames after first lock.
				//
				// Plan-B Step 5 (PRIMARY site — the ~92% RPi idle-scan FIR
				// cost): the dominant full-buffer search runs its COARSE phase
				// (halfsym_2phase Phase 1: GI-stride, early_exit 0.5) on the
				// DECIMATED buffer, then mixes+FIRs only a small full-rate
				// SLICE around the coarse peak and runs Phase 2 (halfsym
				// step=M, no early-exit) on that slice.
				//
				// CRITICAL — replicate halfsym_2phase EXACTLY, not via a
				// nested halfsym_2phase. The original is:
				//   Phase 1 = halfsym(buf, size, M, gi_interp, 0.5)
				//             -> first GI-stride pos from search_offset
				//                with metric >= 0.5  (coarse_delay)
				//   Phase 2 = halfsym(buf+coarse-pream_len, 3*pream_len, M, M)
				//             -> max-metric M-grid pos  (NO early-exit)
				//   result  = (coarse - pream_len) + Phase2.delay
				// Running a nested halfsym_2phase on a slice would re-run
				// Phase 1's early-exit anchored to the SLICE start, not to
				// search_offset — that re-anchoring shifts which "first"
				// preamble the early-exit lands on (observed: whole-symbol
				// offsets, same metric). So we keep Phase 1 = the decimated
				// coarse, and run ONLY Phase 2 on the slice. The decimated
				// GI-stride grid {search_offset/M + k*Ngi} maps to full rate
				// {search_offset + k*gi_interp} — the SAME grid Phase 1 used.
				// Phase 2's window is +/-pream_len, wide enough to absorb the
				// decimated coarse landing a few GI-steps off the full-rate
				// coarse; both old and new Phase 2 search the canonical
				// M-grid, so step=M finds the identical max position.
				int s5_M = interp;
				int s5_search_off_dec = search_offset / s5_M;
				int s5_search_size_dec = search_size / s5_M;
				int s5_gi_dec = data_container.Ngi;  // decimated GI stride
				TimeSyncResult s5_coarse = ofdm.time_sync_preamble_halfsym(
					&data_container.baseband_data_decimated[s5_search_off_dec],
					s5_search_size_dec, 1, s5_gi_dec, 0.5);
				int s5_coarse_full = s5_coarse.delay * s5_M + search_offset;

				int s5_pream_full = data_container.preamble_nSymb * data_container.Nofdm * s5_M;
				// Phase-2 window, matching halfsym_2phase EXACTLY:
				//   fine_start = coarse - pream_len  (clamped >= 0 RELATIVE to
				//                search_offset, i.e. absolute >= search_offset)
				//   fine_size  = 3*pream_len         (clamped so the window
				//                stays within [search_offset, +search_size))
				int s5_slice_start = s5_coarse_full - s5_pream_full;
				if(s5_slice_start < search_offset) s5_slice_start = search_offset;
				int s5_slice_len = 3 * s5_pream_full;
				if(s5_slice_start + s5_slice_len > search_offset + search_size)
					s5_slice_len = (search_offset + search_size) - s5_slice_start;

				TimeSyncResult matched;
				// halfsym_2phase edge cases, replicated:
				//   (A) coarse.correlation < 0.05  -> return coarse
				//   (B) fine_size <= pream_len     -> return coarse
				// In both, the original returns coarse.delay (GI-stride pos);
				// mapped to full rate that is s5_coarse_full.
				if(s5_coarse.correlation < 0.05 || s5_slice_len <= s5_pream_full)
				{
					matched.delay = s5_coarse_full;
					matched.correlation = s5_coarse.correlation;
				}
				else
				{
					// FIR guard margin: extend the FIR'd region by fir_margin
					// samples on each side so the FIR transient (zero-pad at
					// the slice edges) never reaches the searched window — the
					// searched samples get the SAME full-support FIR values as
					// the full-buffer FIR. fir_margin is rounded up to a
					// multiple of M for clean indexing. The full-buffer FIR at
					// :928 still has transients only at the WHOLE-buffer edges,
					// so as long as the slice's guard region stays inside the
					// buffer the slice samples are bit-identical to :928's.
					int s5_taps = ofdm.FIR_rx_time_sync.filter_nTaps;
					int s5_fir_margin = ((s5_taps + s5_M - 1) / s5_M) * s5_M;
					int s5_ext_start = s5_slice_start - s5_fir_margin;
					int s5_ext_end = s5_slice_start + s5_slice_len + s5_fir_margin;
					// Clamp the extended FIR region to the buffer. If a guard
					// margin is clipped, the searched samples nearest that
					// clipped edge see the SAME transient the full-buffer FIR
					// would (both zero-pad at the buffer boundary) — still
					// bit-identical.
					if(s5_ext_start < 0) s5_ext_start = 0;
					if(s5_ext_end > buf_interp) s5_ext_end = buf_interp;
					int s5_ext_len = s5_ext_end - s5_ext_start;
					if(s5_ext_len > data_container.baseband_data_fine_slice_size)
						s5_ext_len = data_container.baseband_data_fine_slice_size;
					// Mix + full-rate FIR the guard-extended slice. sample_offset
					// keeps the mixing phase continuous with the rest of the buffer.
					ofdm.passband_to_baseband(&((double*)data)[s5_ext_start],
						s5_ext_len, data_container.baseband_data_fine_slice,
						sampling_frequency, carrier_frequency, carrier_amplitude,
						1, &ofdm.FIR_rx_time_sync, s5_ext_start);
					// Phase 2 ONLY: halfsym step=M, no early-exit (max-metric),
					// run on the interior [s5_slice_start, +s5_slice_len) which
					// in fine_slice coordinates begins at (s5_slice_start -
					// s5_ext_start).
					int s5_interior = s5_slice_start - s5_ext_start;
					int s5_search_len = s5_slice_len;
					if(s5_interior + s5_search_len > s5_ext_len)
						s5_search_len = s5_ext_len - s5_interior;
					TimeSyncResult s5_fine = ofdm.time_sync_preamble_halfsym(
						&data_container.baseband_data_fine_slice[s5_interior],
						s5_search_len, interp, interp);
					matched.delay = s5_slice_start + s5_fine.delay;
					matched.correlation = s5_fine.correlation;
				}

				receive_stats.delay = matched.delay;
				receive_stats.coarse_metric = matched.correlation;

				if(matched.correlation < preamble_detect_threshold)
				{
					// Sub-threshold detection — skip demodulation entirely
					// to save CPU (no LDPC decode of garbage), but preserve
					// the detected delay position so the ARQ FAIL handler
					// can see WHERE the false peak was. This allows the
					// FTR-FALSE handler (metric 0.15-0.50) to zero the
					// false preamble in the ring buffer, preventing
					// re-detection on the next iteration.
					// Previously delay was set to -1, which bypassed ALL
					// smart FAIL handling (zeroing, fast-forward, batch exit)
					// and fell through to default ftr=8 anti-spin.
					receive_stats.ofdm_batch_active = false;
					receive_stats.message_decoded = NO;
					return receive_stats;
				}
			}

			if (g_verbose) printf("[OFDM-SYNC] %s %s offset=%d size=%d metric=%.4f delay=%d thr=%.1f\n",
				narrowband_enabled ? "NB" : "WB",
				receive_stats.ofdm_batch_active ? "BATCH" : "INIT",
				search_offset, search_size, receive_stats.coarse_metric, (int)receive_stats.delay,
				preamble_detect_threshold);

			} // end else (non-forced-delay detection)
		}
		pream_symb_loc=receive_stats.delay/(data_container.Nofdm*data_container.interpolation_rate);
		if(pream_symb_loc<1){pream_symb_loc=1;}


	}

	// Frame completeness check: if the frame extends beyond the buffer,
	// don't attempt decode — signal the ARQ layer to capture more audio.
	// Skip in BER test mode (mfsk_fixed_delay >= 0) where buffers are pre-sized.
	// Extended to OFDM (previously MFSK-only): on back-to-back batch TX, OFDM
	// preambles land in the ring buffer tail beyond upper_bound, triggering
	// the fast-forward `ftr=shift` branch which wastes 700ms of audio (~30
	// frames). Deferring via overflow path preserves those frames. See
	// fact-documents/SACK_LATE_SNAPSHOT_BUG.md §Preamble bounds.
	// Phase-2: --ofdm-defer-overflow=off bypasses Fix A.
	if(mfsk_fixed_delay < 0 && ofdm_defer_overflow_enabled)
	{
		int sym_samples = data_container.Nofdm * frequency_interpolation_rate;
		int active_nsymb = get_active_nsymb();
		// LEVER P: a MINI tail frame ends rx_eff_preamble+Nsymb symbols past the
		// preamble, not preamble_nSymb+Nsymb — use the actual length so a
		// complete MINI frame isn't spuriously deferred.
		int frame_end_samples = receive_stats.delay + (rx_eff_preamble + active_nsymb) * sym_samples;
		int buffer_samples = data_container.Nofdm * data_container.buffer_Nsymb * frequency_interpolation_rate;
		if(frame_end_samples > buffer_samples)
		{
			receive_stats.message_decoded = NO;
			int overflow_samples = frame_end_samples - buffer_samples;
			receive_stats.frame_overflow_symbols = (overflow_samples + sym_samples - 1) / sym_samples;
			return receive_stats;
		}
	}

	int lower_bound = data_container.preamble_nSymb;
	// LEVER P (INC-3): the bounds GATE must use the SAME per-frame preamble
	// length the EXTRACTION uses (telecom_system.cc:2348 frame_size_interp =
	// Nofdm*(Nsymb+rx_eff_preamble)). A MINI tail frame (rx_eff_preamble=1) is
	// (Nsymb+1) symbols long, so the highest preamble symbol from which a
	// complete MINI frame still fits is buffer_Nsymb-(Nsymb+1) — 3 symbols
	// HIGHER than the FULL-frame bound. Using the FULL preamble_nSymb here
	// rejected MINI tail frames at pream_symb 116..118 (frame ends <128, room
	// to extract) as "beyond-bounds", losing every tail frame past the 11th in
	// a 21-frame batch. rx_eff_preamble == preamble_nSymb on the full-search /
	// re-anchor / MFSK / amortization-off paths, so this is byte-identical when
	// the feature is off.
	int upper_bound = data_container.buffer_Nsymb-(data_container.Nsymb+rx_eff_preamble);
	// §7.13.29 (Proposal A) — SACK_RSP cross-check uses a stricter detection
	// threshold so Schmidl-Cox sub-peaks in OFDM body audio (consistently
	// metric≈0.555 in trace, vs ~0.9 for a real preamble) are rejected before
	// burning the Moose trial budget. Default 0.15 (WB) / 0.30 (NB) stays for
	// normal data RX where real preambles can have low metric under heavy
	// fading and we cannot afford to drop them.
	double preamble_detect_threshold = sack_cross_check_mode
		? (narrowband_enabled ? 0.75 : 0.65)
		: (narrowband_enabled ? 0.30 : 0.15);

	if(M != MOD_MFSK)
	{
		printf("[OFDM-SYNC] coarse: pream_symb=%d delay=%d bounds=[%d,%d] metric=%.3f bufNsymb=%d Nsymb=%d preamNsymb=%d %s\n",
			pream_symb_loc, receive_stats.delay, lower_bound, upper_bound,
			receive_stats.coarse_metric,
			(int)data_container.buffer_Nsymb, data_container.Nsymb, data_container.preamble_nSymb,
			(pream_symb_loc > lower_bound && pream_symb_loc <= upper_bound) ? "PASS" : "SKIP");
		fflush(stdout);
	}

	// Recovery for OFDM when preamble lands outside the valid bounds.
	// Even with full-buffer coarse search, Schmidl-Cox may peak at a position
	// too close to the start or end to extract a complete frame. Scan the
	// buffer for signal energy and re-run Schmidl-Cox from the signal start.
	//
	// IMPORTANT: Respect anti-re-decode. When ofdm_search_raw has advanced past
	// upper_bound, the search region has no room for complete frames. The recovery
	// must NOT rescan from the start (which would re-find already-decoded frames).
	if(M != MOD_MFSK && !(pream_symb_loc > lower_bound && pream_symb_loc <= upper_bound))
	{
		// Anti-re-decode check: if skip is past upper_bound, there are no
		// un-decoded frames left in the buffer. Skip recovery → FAIL → buffer shift.
		int ofdm_eff = receive_stats.ofdm_search_raw - data_container.nUnder_processing_events;
		if(ofdm_eff < 0) ofdm_eff = 0;
		if(ofdm_eff > upper_bound)
		{
			// No room for new frames — force FAIL
			pream_symb_loc = 0;
		}
		else
		{

		int sym_samples = data_container.Nofdm * frequency_interpolation_rate;
		int buf_samples = data_container.Nofdm * data_container.buffer_Nsymb * frequency_interpolation_rate;
		// Plan-B Step 6a (site 3, bounds-failed recovery energy scans): the
		// signal-start scan and retry-position energy check compare per-symbol
		// MEAN energy to an absolute threshold (0.001). Mean energy per sample
		// is rate-invariant (the anti-alias FIR passes the signal band), so the
		// scan can run on the decimated buffer with Nofdm-sized symbol regions.
		int sym_dec = data_container.Nofdm;                                  // decimated symbol length
		int buf_dec = data_container.Nofdm * data_container.buffer_Nsymb;     // decimated buffer length
		int p6_M = data_container.interpolation_rate;

		if (g_verbose) printf("[OFDM-SYNC] bounds-failed: pream_symb=%d, scanning full buffer for signal\n", pream_symb_loc);
		fflush(stdout);

		// Start scan from anti-re-decode position to avoid re-finding old preambles
		int scan_start = lower_bound + 1;
		if(ofdm_eff > scan_start) scan_start = ofdm_eff;
		int signal_start_symb = -1;
		for(int s = scan_start; s <= upper_bound; s++)
		{
			int offset_dec = s * sym_dec;
			double e = 0.0;
			int cnt = 0;
			for(int i = 0; i < sym_dec && (offset_dec + i) < buf_dec; i++)
			{
				double re = data_container.baseband_data_decimated[offset_dec + i].real();
				double im = data_container.baseband_data_decimated[offset_dec + i].imag();
				e += re*re + im*im;
				cnt++;
			}
			e = (cnt > 0) ? e / cnt : 0.0;
			if(e > 0.001)
			{
				signal_start_symb = s;
				break;
			}
		}

		if(signal_start_symb >= 0)
		{
			int search_start = signal_start_symb * sym_samples;
			int available = buf_samples - search_start;
			// Constrain GI retry window (same as silence-skip recovery)
			int max_search = (data_container.preamble_nSymb + 4) * sym_samples;
			if(available > max_search) available = max_search;

			if(available > data_container.preamble_nSymb * sym_samples)
			{
				// Schmidl-Cox autocorrelation: preamble L-sample periodicity
				// discriminates preamble from data (data has no L-period).
				// Plan-B Step 3 (site 4, bounds-failed recovery): run the
				// coarse search on the decimated buffer. search_start and
				// available are multiples of sym_samples (= Nofdm*M), hence
				// multiples of M, so the /M arithmetic is exact. The original
				// call used step = interpolation_rate (= M), i.e. it only ever
				// evaluated positions on the M-grid — the decimated step=1
				// search lands on that SAME grid, so this is resolution-
				// equivalent, not a precision loss.
				int p3_M = data_container.interpolation_rate;
				int search_start_dec = search_start / p3_M;
				int available_dec = available / p3_M;
				TimeSyncResult retry = ofdm.time_sync_preamble_halfsym(
					&data_container.baseband_data_decimated[search_start_dec],
					available_dec, 1, 1);
				retry.delay = retry.delay * p3_M + search_start;

				int retry_symb = retry.delay / sym_samples;
				if(retry_symb < 1) retry_symb = 1;

				// Plan-B Step 6a: retry-position energy on the decimated buffer.
				// retry.delay is a full-rate index; floor it to the decimated
				// grid (the <M-sample shift is negligible for a mean-energy
				// gate). Energy region is Nofdm decimated samples per symbol.
				double retry_energy = 0.0;
				int rcnt = 0;
				int retry_delay_dec = retry.delay / p6_M;
				for(int i = 0; i < sym_dec && (retry_delay_dec + i) < buf_dec; i++)
				{
					double re = data_container.baseband_data_decimated[retry_delay_dec + i].real();
					double im = data_container.baseband_data_decimated[retry_delay_dec + i].imag();
					retry_energy += re*re + im*im;
					rcnt++;
				}
				retry_energy = (rcnt > 0) ? retry_energy / rcnt : 0.0;

				if (g_verbose)
					printf("[OFDM-SYNC] bounds-skip: signal=%d retry=%d metric=%.3f energy=%.2e\n",
						signal_start_symb, retry_symb, retry.correlation, retry_energy);
				fflush(stdout);

				if(retry_energy >= 0.001 && retry.correlation >= preamble_detect_threshold
					&& retry_symb > lower_bound && retry_symb <= upper_bound)
				{
					receive_stats.delay = retry.delay;
					receive_stats.coarse_metric = retry.correlation;
					pream_symb_loc = retry_symb;
				}
			}
		}
		} // else: recovery with anti-re-decode scan
	}

	int skip_h_count = 0;
	if(pream_symb_loc > lower_bound && pream_symb_loc <= upper_bound)
	{
		// Signal energy gate: reject false preamble detections in silence.
		// Schmidl-Cox gives high correlation on near-zero noise (ratio of tiny
		// values is unstable). Check actual signal energy at the detected
		// preamble position before spending ~120ms on 3 LDPC decode trials.
		// Uses RELATIVE threshold: preamble energy vs buffer mean energy.
		// Old absolute threshold (0.001) was calibrated for VB-Cable (~0.1 mean)
		// and rejected SGTL5000 audio at ~1e-6 mean (40 dB lower).
		bool energy_ok = true;
		double pream_mean_energy = 0.0;  // saved for data energy gate comparison
		if(M != MOD_MFSK)
		{
			int sym_samples = data_container.Nofdm * frequency_interpolation_rate;
			int buf_samples = data_container.Nofdm * data_container.buffer_Nsymb * frequency_interpolation_rate;
			// Plan-B Step 6a (site 4, main signal energy gate): preamble mean
			// energy and buffer mean energy are both per-sample means compared
			// to an absolute floor (energy_gate_floor) / used as a ratio in the
			// data gate — rate-invariant, so read the decimated buffer.
			// receive_stats.delay is a full-rate index, floored to the
			// decimated grid (<M-sample shift, negligible for a mean gate).
			int eg_M = data_container.interpolation_rate;
			int eg_sym_dec = data_container.Nofdm;
			int eg_buf_dec = data_container.Nofdm * data_container.buffer_Nsymb;
			int eg_delay_dec = receive_stats.delay / eg_M;
			double energy_sum = 0.0;
			int count = 0;
			for(int i = 0; i < eg_sym_dec && (eg_delay_dec + i) < eg_buf_dec; i++)
			{
				double re = data_container.baseband_data_decimated[eg_delay_dec + i].real();
				double im = data_container.baseband_data_decimated[eg_delay_dec + i].imag();
				energy_sum += re*re + im*im;
				count++;
			}
			double mean_energy = (count > 0) ? energy_sum / count : 0.0;
			pream_mean_energy = mean_energy;

			// Compute buffer mean energy for relative comparison
			double buf_energy_sum = 0.0;
			for(int i = 0; i < eg_buf_dec; i++)
			{
				double re = data_container.baseband_data_decimated[i].real();
				double im = data_container.baseband_data_decimated[i].imag();
				buf_energy_sum += re*re + im*im;
			}
			double buf_mean_energy = buf_energy_sum / eg_buf_dec;

			// Reject if buffer is truly silent (no audio hardware connected)
			// AND preamble energy is indistinguishable from buffer average.
			// Real signal: preamble energy >> silence-region energy.
			// False alarm: preamble energy ≈ buffer mean (noise floor throughout).
			// Absolute floor 1e-12: below any real ADC noise floor.
			bool is_silence = (buf_mean_energy < energy_gate_floor) && (mean_energy < energy_gate_floor);
			printf("[OFDM-ENERGY] pream=%.4e buf=%.4e count=%d delay=%d symb=%d metric=%.3f %s\n",
				mean_energy, buf_mean_energy, count, receive_stats.delay, pream_symb_loc,
				receive_stats.coarse_metric,
				is_silence ? "REJECT" : "PASS");
			fflush(stdout);
			if(is_silence)
			{
				energy_ok = false;
			}

			// Metric threshold: reject weak peaks that correspond to data symbols.
			// Schmidl-Cox preamble: ~0.5-1.0. Data: ~0.01-0.05 (no L-period).
			// Threshold 0.10 blocks data peaks while allowing degraded preambles.
			if(energy_ok && receive_stats.coarse_metric < 0.10)
			{
				printf("[OFDM-ENERGY] metric=%.3f at delay=%d — weak peak, skipping decode\n",
					receive_stats.coarse_metric, receive_stats.delay);
				fflush(stdout);
				energy_ok = false;
			}

			// Silence-skip: when the best Schmidl-Cox peak lands in silence
			// (energy gate rejected), scan forward to find where signal actually
			// starts and re-run with GI correlation from there. This handles
			// silence-preceded buffers (e.g. SET_CONFIG after ACK) where silence
			// or boundary peaks beat the real preamble in the initial search.
			// Both NB and WB now use halfsym which searches the whole buffer,
			// so this rarely activates. Kept as safety net for edge cases.
			if(!energy_ok)
			{
				// Plan-B Step 6a (site 5, silence-skip recovery energy scans):
				// forward signal-start scan and retry-position energy on the
				// decimated buffer. The forward scan compares per-symbol mean
				// energy to buf_mean_energy*2 (a ratio — rate-invariant) and to
				// energy_gate_floor (absolute, rate-invariant for a mean).
				// buf_mean_energy here is the decimated buffer mean (computed at
				// site 4 above), so both sides of the ratio are decimated-rate.
				int signal_start_symb = -1;
				for(int s = pream_symb_loc + 1; s <= upper_bound; s++)
				{
					int offset_dec = s * eg_sym_dec;
					double e = 0.0;
					int cnt = 0;
					for(int i = 0; i < eg_sym_dec && (offset_dec + i) < eg_buf_dec; i++)
					{
						double re = data_container.baseband_data_decimated[offset_dec + i].real();
						double im = data_container.baseband_data_decimated[offset_dec + i].imag();
						e += re*re + im*im;
						cnt++;
					}
					e = (cnt > 0) ? e / cnt : 0.0;
					if(e > buf_mean_energy * 2.0 && e > energy_gate_floor)
					{
						signal_start_symb = s;
						break;
					}
				}

				if(signal_start_symb >= 0)
				{
					int search_start = signal_start_symb * sym_samples;
					int available = buf_samples - search_start;
					// Constrain GI retry to preamble + 4 symbols around signal start.
					// Searching the full remaining buffer can find a later frame
					// repetition past upper_bound, causing the retry to fail bounds.
					int max_search = (data_container.preamble_nSymb + 4) * sym_samples;
					if(available > max_search) available = max_search;

					if(available > data_container.preamble_nSymb * sym_samples)
					{
						// Schmidl-Cox autocorrelation: preamble L-sample periodicity
						// discriminates preamble from data (data has no L-period).
						// Plan-B Step 3 (site 5, silence-skip recovery): coarse
						// search on the decimated buffer. Same /M-exact offsets
						// and resolution-equivalent step=1 as site 4.
						int p3_M = data_container.interpolation_rate;
						int search_start_dec = search_start / p3_M;
						int available_dec = available / p3_M;
						TimeSyncResult retry = ofdm.time_sync_preamble_halfsym(
							&data_container.baseband_data_decimated[search_start_dec],
							available_dec, 1, 1);
						retry.delay = retry.delay * p3_M + search_start;

						int retry_symb = retry.delay / sym_samples;
						if(retry_symb < 1) retry_symb = 1;

						// Check energy at the retry position (Step 6a: decimated
						// buffer; retry.delay is full-rate, floored to /M).
						double retry_energy = 0.0;
						int rcnt = 0;
						int retry_delay_dec = retry.delay / eg_M;
						for(int i = 0; i < eg_sym_dec && (retry_delay_dec + i) < eg_buf_dec; i++)
						{
							double re = data_container.baseband_data_decimated[retry_delay_dec + i].real();
							double im = data_container.baseband_data_decimated[retry_delay_dec + i].imag();
							retry_energy += re*re + im*im;
							rcnt++;
						}
						retry_energy = (rcnt > 0) ? retry_energy / rcnt : 0.0;

						printf("[OFDM-SYNC] silence-skip: orig=%d signal=%d retry=%d metric=%.3f energy=%.2e\n",
							pream_symb_loc, signal_start_symb, retry_symb, retry.correlation, retry_energy);
						fflush(stdout);

						if(retry_energy > energy_gate_floor && retry.correlation >= preamble_detect_threshold
							&& retry_symb > lower_bound && retry_symb <= upper_bound)
						{
							receive_stats.delay = retry.delay;
							receive_stats.coarse_metric = retry.correlation;
							pream_symb_loc = retry_symb;
							energy_ok = true;
						}
					}
				}
			}
		}

		// Data energy gate (after all recovery paths): preamble has energy
		// but data symbols may be silence — frame partially captured, with
		// preamble at trailing edge and data still arriving in the pipeline.
		// Runs after silence-skip recovery so the final pream position is used.
		if(energy_ok && M != MOD_MFSK)
		{
			int sym_samples_de = data_container.Nofdm * frequency_interpolation_rate;
			int buf_samples_de = data_container.Nofdm * data_container.buffer_Nsymb * frequency_interpolation_rate;
			// Plan-B Step 6a (site 6, data energy gate + ENERGY-DIAG): data
			// mean energy vs energy_gate_floor (absolute) and pream_mean_energy
			// (ratio) — rate-invariant; read the decimated buffer. The raw
			// passband `data` reads in ENERGY-DIAG are unaffected (they are not
			// the decimated buffer).
			int de_M = data_container.interpolation_rate;
			int de_sym_dec = data_container.Nofdm;
			int de_buf_dec = data_container.Nofdm * data_container.buffer_Nsymb;
			int data_offset = receive_stats.delay + data_container.preamble_nSymb * sym_samples_de;
			int data_offset_dec = receive_stats.delay / de_M + data_container.preamble_nSymb * de_sym_dec;
			double data_e = 0.0;
			int d_count = 0;
			int check_len = 4 * sym_samples_de; // first 4 data symbols
			int check_len_dec = 4 * de_sym_dec;
			for(int i = 0; i < check_len_dec && (data_offset_dec + i) < de_buf_dec; i++)
			{
				double re = data_container.baseband_data_decimated[data_offset_dec + i].real();
				double im = data_container.baseband_data_decimated[data_offset_dec + i].imag();
				data_e += re*re + im*im;
				d_count++;
			}
			data_e = (d_count > 0) ? data_e / d_count : 0.0;

			// DIAG: compare passband vs baseband energy at preamble and data positions
			{
				int pream_offset = receive_stats.delay;
				int pream_offset_dec = receive_stats.delay / de_M;
				double pb_pream = 0, pb_data = 0, bb_pream = 0;
				int pream_len = data_container.preamble_nSymb * sym_samples_de;
				int pream_len_dec = data_container.preamble_nSymb * de_sym_dec;
				for(int i = 0; i < pream_len && (pream_offset + i) < buf_samples_de; i++) {
					double v = ((double*)data)[pream_offset + i];
					pb_pream += v*v;
				}
				for(int i = 0; i < pream_len_dec && (pream_offset_dec + i) < de_buf_dec; i++) {
					double re = data_container.baseband_data_decimated[pream_offset_dec + i].real();
					double im = data_container.baseband_data_decimated[pream_offset_dec + i].imag();
					bb_pream += re*re + im*im;
				}
				pb_pream /= pream_len;
				bb_pream /= pream_len_dec;
				for(int i = 0; i < check_len && (data_offset + i) < buf_samples_de; i++) {
					double v = ((double*)data)[data_offset + i];
					pb_data += v*v;
				}
				pb_data = (d_count > 0) ? pb_data / d_count : 0.0;
				printf("[ENERGY-DIAG] pream: pb=%.4e bb=%.4e | data: pb=%.4e bb=%.4e | delay=%d data_off=%d buf=%d\n",
					pb_pream, bb_pream, pb_data, data_e, receive_stats.delay, data_offset, buf_samples_de);
				fflush(stdout);
			}

			// Data missing if data energy is <10% of preamble energy (relative)
			// or truly zero (absolute floor). Old 0.001 threshold rejected
			// SGTL5000 signals at -40 dBFS.
			if(data_e < energy_gate_floor || (pream_mean_energy > energy_gate_floor && data_e < pream_mean_energy * 0.1))
			{
				printf("[OFDM-SYNC] data_energy=%.2e pream_energy=%.2e at pream=%d delay=%d — frame incomplete, skipping decode\n",
					data_e, pream_mean_energy, pream_symb_loc, receive_stats.delay);
				fflush(stdout);
				energy_ok = false;
				receive_stats.frame_data_missing = true;
			}
		}

		if(energy_ok)
		{
		if(M != MOD_MFSK && g_verbose)
		{
			printf("[OFDM-ENTRY] metric=%.3f delay=%d symb=%d %s Nc=%d Nsymb=%d\n",
				receive_stats.coarse_metric, receive_stats.delay, pream_symb_loc,
				narrowband_enabled ? "NB" : "WB", ofdm.Nc, ofdm.Nsymb);
			fflush(stdout);
		}

		skip_h_count = 0;
		double mean_H = -1.0;
		bool skip_h_recovery_attempted = false;
		int consecutive_skip_var = 0;  // Phase-F stall fix: break trial loop if N+ SKIP-VAR in a row
		// v2 OFDM sub-peak recovery (additive to the v1 reject at ~line 2310-2331).
		// v1 catches sub-peaks with mean_H<0.5. v2 catches the residual case where
		// a sub-peak lands ~1 OFDM-symbol off the real preamble: enough real-preamble
		// signal leaks into the channel estimate that mean_H~0.5-1.0 survives, but
		// the decoded data is byte-misaligned and LDPC burns to the iter cap.
		// Strategy: on LDPC fail with high coarse_metric AND high mean_H AND iter cap,
		// rewind to the data-FIR entry point and re-decode at delay ± 1 OFDM symbol.
		// State persists across trial-loop iterations so the recovery attempts are
		// counted (max 2: +sym then -sym), not re-armed every trial.
		int subpeak_recover_phase = 0;     // 0=not started, 1=tried +sym, 2=tried -sym
		int subpeak_orig_delay = -1;       // delay at which v2 was first armed (full-rate samples)
		bool subpeak_recover_in_flight = false; // true between goto-back and next LDPC verdict
		// §7.13.29 (Proposal B) — SACK_RSP cross-check pays more trials so the
		// search escapes Schmidl-Cox sub-peak false locks. Default 2 stays for
		// normal data RX where extra trials waste CPU on real LDPC failures.
		const int effective_trials_max = sack_cross_check_mode
			? (time_sync_trials_max > 5 ? time_sync_trials_max : 5)
			: time_sync_trials_max;
skip_h_retry_point:
		while (receive_stats.sync_trials<=effective_trials_max)
		{
			if(mfsk_fixed_delay >= 0)
			{
				// Known delay - skip all time_sync refinement
				receive_stats.delay = mfsk_fixed_delay;
			}
			else if(ofdm_forced_delay >= 0)
			{
				// OFDM forced delay (BER test): skip sync refinement, use known position
				receive_stats.delay = ofdm_forced_delay;
				if(receive_stats.sync_trials > 0) break;  // one trial only
			}
			else if(M == MOD_MFSK)
			{
				// MFSK: time_sync_mfsk already found optimal position, no refinement needed
				// Only one trial - spectral flatness sync is deterministic
				if(receive_stats.sync_trials > 0) break;
				// Trial 0: use delay from initial sync as-is
			}
			else if(narrowband_enabled)
			{
				// NB OFDM: halfsym detection provides timing, Moose handles freq sync.
				// NB preamble uses even-only subcarriers (Bug #41), enabling half-symbol
				// repetition. Single trial for now; coarse freq search possible in future.
				if(receive_stats.sync_trials > 0) break;
			}
			else if(receive_stats.sync_trials==time_sync_trials_max && use_last_good_time_sync==YES && receive_stats.delay_of_last_decoded_message!=-1)
			{
				receive_stats.delay=receive_stats.delay_of_last_decoded_message;
			}
			else if (receive_stats.sync_trials == 1 && coarse_freq_sync_enabled)
			{
				// Trial 0 failed - try coarse frequency search before trial 1
				// Search ±30 Hz; Moose handles ±22 Hz residual at each,
				// giving ±52 Hz total coverage
				const double freq_search[] = {-30.0, 0.0, 30.0};
				const int n_search = 3;
				double best_correlation = 0.0;
				double best_offset = 0.0;
				int best_delay = receive_stats.delay;
				double zero_hz_correlation = 0.0;

				int sym_samp = data_container.Nofdm * frequency_interpolation_rate;
				for (int i = 0; i < n_search; i++)
				{
					// Plan-B Step 3 (site 6, coarse-freq search): re-mix at the
					// trial carrier directly to the decimated rate, then run the
					// coarse search on the decimated buffer. This re-mix MUST be
					// per-trial-carrier — the top-of-function decimated buffer
					// was mixed at carrier_frequency only. base_off and avail are
					// multiples of sym_samp (= Nofdm*M), so /M is exact.
					int p3_M = data_container.interpolation_rate;
					int p3_full_size = data_container.Nofdm * data_container.buffer_Nsymb * frequency_interpolation_rate;
					ofdm.passband_to_baseband_decimated((double*)data,
						p3_full_size,
						data_container.baseband_data_decimated,
						sampling_frequency,
						carrier_frequency + freq_search[i],
						carrier_amplitude, p3_M, &ofdm.FIR_rx_time_sync);

					// FFT fine search around known preamble position.
					// FFT metric discriminates preamble vs data (4× ratio)
					// and drops sharply with frequency error — ideal for
					// comparing offsets.
					int base_off = (pream_symb_loc > 0 ? pream_symb_loc - 1 : 0) * sym_samp;
					int buf_lim = data_container.Nofdm * data_container.buffer_Nsymb * frequency_interpolation_rate;
					int avail = buf_lim - base_off;
					int need = (data_container.preamble_nSymb + 4) * sym_samp;
					if(avail > need) avail = need;
					int base_off_dec = base_off / p3_M;
					int avail_dec = avail / p3_M;
					TimeSyncResult ts_result = ofdm.time_sync_preamble_halfsym(
						&data_container.baseband_data_decimated[base_off_dec],
						avail_dec, 1, 1);
					ts_result.delay = ts_result.delay * p3_M + base_off;

					if (fabs(freq_search[i]) < 0.1)
						zero_hz_correlation = ts_result.correlation;

					if (ts_result.correlation > best_correlation)
					{
						best_correlation = ts_result.correlation;
						best_offset = freq_search[i];
						best_delay = ts_result.delay;
					}
				}

				// Apply only if non-zero offset is significantly better than 0 Hz.
				// Schmidl-Cox metric range: 0-1 (~0.8+ at preamble, ~0.05 at data).
				// Delta 0.08 = meaningful improvement over 0 Hz baseline.
				if (fabs(best_offset) > 1.0 && best_correlation > preamble_detect_threshold &&
				    best_correlation > zero_hz_correlation + 0.08)
				{
					coarse_freq_offset = best_offset;
					last_coarse_freq_offset = best_offset;  // persist for ACK/SACK detection
					receive_stats.delay = best_delay;
					pream_symb_loc = receive_stats.delay / (data_container.Nofdm * data_container.interpolation_rate);
					if (pream_symb_loc < 1) { pream_symb_loc = 1; }
				}

				// Plan-B Step 3 (site 7, fine-sync after coarse-freq): restore
				// the decimated baseband at the corrected carrier for the fine
				// Schmidl-Cox search. The full-rate baseband_data_interpolated
				// is intentionally NOT re-mixed here — its only downstream
				// consumers are energy gates (std::norm), which are invariant
				// to the carrier mixing frequency, and the data extraction
				// re-mixes from raw `data` itself.
				{
					int p3_M = data_container.interpolation_rate;
					int p3_full_size = data_container.Nofdm * data_container.buffer_Nsymb * frequency_interpolation_rate;
					ofdm.passband_to_baseband_decimated((double*)data,
						p3_full_size,
						data_container.baseband_data_decimated,
						sampling_frequency,
						carrier_frequency + coarse_freq_offset,
						carrier_amplitude, p3_M, &ofdm.FIR_rx_time_sync);

					// Schmidl-Cox fine time sync at the corrected frequency
					int sym_samp = data_container.Nofdm * frequency_interpolation_rate;
					int base_off = (pream_symb_loc > 0 ? pream_symb_loc - 1 : 0) * sym_samp;
					int buf_lim = data_container.Nofdm * data_container.buffer_Nsymb * frequency_interpolation_rate;
					int avail = buf_lim - base_off;
					int need = (data_container.preamble_nSymb + 4) * sym_samp;
					if(avail > need) avail = need;
					int base_off_dec = base_off / p3_M;
					int avail_dec = avail / p3_M;
					TimeSyncResult ts_result = ofdm.time_sync_preamble_halfsym(
						&data_container.baseband_data_decimated[base_off_dec],
						avail_dec, 1, 1);
					receive_stats.delay = base_off + ts_result.delay * p3_M;
				}
			}
			else
			{
				// GI+halfsym fine timing for Moose freq sync alignment.
				// Coarse detection finds the correct symbol, but Moose needs
				// the halfsym alignment that even-only preamble subcarriers
				// provide. GI+halfsym finds a position where mean_H≥0.30,
				// allowing LDPC decode. See PHASE2_FFT_REPLACEMENT.md §8.
				//
				// Plan-B Step 6b (site 8, trial-loop fine sync): site 8 runs
				// time_sync_preamble_with_metric at step=1 — full sub-sample
				// resolution — over a SMALL, already-bounded window
				// [(pream_symb_loc-1)*Nofdm*M, +(preamble_nSymb+4)*Nofdm*M).
				// Unlike the site-3 idle scan, site 8 already KNOWS where its
				// window is, so no decimated coarse search is needed: we mix +
				// FIR that exact window at full rate from raw `data` into the
				// Step-5 scratch slice (baseband_data_fine_slice) and run
				// with_metric on it UNCHANGED. This is bit-exact — with_metric
				// reads the identical full-rate samples it would have read from
				// baseband_data_interpolated, just sourced from a scoped slice
				// instead of the full buffer (which the eager :928 FIR populated
				// in HEAD; :928 is removed in Step 6c). The N-th-best-peak
				// trial-loop semantics (location_to_return = sync_trials) are
				// preserved exactly because with_metric still sees the whole
				// window at step=1.
				//
				// Carrier: mix the slice at carrier_frequency + coarse_freq_
				// offset. In HEAD, baseband_data_interpolated holds the eager
				// :928 mixing at carrier_frequency on trial 0 (coarse_freq_
				// offset == 0), and on later trials the site-7 re-mix at
				// carrier_frequency + coarse_freq_offset. Mixing the slice at
				// carrier_frequency + coarse_freq_offset reproduces both.
				int s8_M = data_container.interpolation_rate;
				int s8_win_start = (pream_symb_loc - 1) * data_container.Nofdm * frequency_interpolation_rate;
				// LEVER P: window/template span the MINI preamble length (+4 sym
				// search slack), not the full 4-symbol preamble.
				int s8_win_len = (rx_eff_preamble + 4) * data_container.Nofdm * s8_M;
				int s8_buf_interp = data_container.Nofdm * data_container.buffer_Nsymb * frequency_interpolation_rate;
				if(s8_win_start < 0) s8_win_start = 0;
				if(s8_win_start + s8_win_len > s8_buf_interp)
					s8_win_len = s8_buf_interp - s8_win_start;
				// FIR guard margin (rounded up to M): the slice FIR transient
				// (zero-pad at slice edges) is pushed outside the searched
				// window so the searched samples get the SAME full-support FIR
				// values the :928 full-buffer FIR produced. Same idiom as the
				// Step-5 fine slice.
				int s8_taps = ofdm.FIR_rx_time_sync.filter_nTaps;
				int s8_fir_margin = ((s8_taps + s8_M - 1) / s8_M) * s8_M;
				int s8_ext_start = s8_win_start - s8_fir_margin;
				int s8_ext_end = s8_win_start + s8_win_len + s8_fir_margin;
				if(s8_ext_start < 0) s8_ext_start = 0;
				if(s8_ext_end > s8_buf_interp) s8_ext_end = s8_buf_interp;
				int s8_ext_len = s8_ext_end - s8_ext_start;
				if(s8_ext_len > data_container.baseband_data_fine_slice_size)
					s8_ext_len = data_container.baseband_data_fine_slice_size;
				// Mix + full-rate FIR the guard-extended slice from raw `data`.
				// sample_offset = s8_ext_start keeps the mixing phase continuous
				// with the rest of the buffer (matches the :928 / site-7 mix).
				ofdm.passband_to_baseband(&((double*)data)[s8_ext_start],
					s8_ext_len, data_container.baseband_data_fine_slice,
					sampling_frequency, carrier_frequency + coarse_freq_offset,
					carrier_amplitude, 1, &ofdm.FIR_rx_time_sync, s8_ext_start);
				// with_metric on the interior [s8_win_start, +s8_win_len),
				// which in fine_slice coordinates begins at (s8_win_start -
				// s8_ext_start). Unchanged step=1, location_to_return, nTrials.
				int s8_interior = s8_win_start - s8_ext_start;
				int s8_search_len = s8_win_len;
				if(s8_interior + s8_search_len > s8_ext_len)
					s8_search_len = s8_ext_len - s8_interior;
				TimeSyncResult fine_result = ofdm.time_sync_preamble_with_metric(
					&data_container.baseband_data_fine_slice[s8_interior],
					s8_search_len,
					data_container.interpolation_rate, receive_stats.sync_trials, 1, time_sync_trials_max,
					(preamble_amortization_enabled && M != MOD_MFSK) ? rx_eff_preamble : -1);
				receive_stats.delay = s8_win_start + fine_result.delay;
			}

			if(receive_stats.delay<0){receive_stats.delay=0;}


			// Clamp delay to prevent buffer overflow in rational_resampler.
			// LEVER P (INC-3): a MINI tail frame is (Nsymb+rx_eff_preamble)
			// symbols, so its max_delay sits HIGHER than a FULL frame's. Clamping
			// a MINI to the FULL-frame max_delay would shove its delay BELOW the
			// real preamble position (mis-extracting the frame). Use rx_eff_preamble
			// — identical to preamble_nSymb on every non-MINI / amortization-off
			// path, so byte-identical when the feature is off.
			{
				int buf_size = data_container.Nofdm * data_container.buffer_Nsymb * frequency_interpolation_rate;
				int frame_size = (data_container.Nofdm*(data_container.Nsymb+rx_eff_preamble))*frequency_interpolation_rate;
				int max_delay = buf_size - frame_size;
				if(receive_stats.delay > max_delay)
				{
					receive_stats.delay = max_delay;
				}
			}

			// Post-fine-sync energy gate: Schmidl-Cox normalized correlation
			// ties at ~1.0 for positions where 3 of 4 preamble symbols overlap
			// signal (silence in the 4th cancels from numerator and denominator).
			// Sort returns earliest tied position, which may be in silence.
			// Fix: if energy at fine-sync delay is zero, advance by whole symbols
			// to find signal onset while preserving sub-symbol alignment.
			if(M != MOD_MFSK)
			{
				// Plan-B Step 6a (site 8-energy, post-fine-sync energy gate):
				// per-symbol mean energy vs energy_gate_floor — rate-invariant.
				// The energy reads move to the decimated buffer; receive_stats.
				// delay arithmetic stays full-rate (the forward step `fwd` is a
				// multiple of full-rate sym_samples, i.e. a multiple of M, so
				// the decimated index candidate/M is exact and consistent).
				int sym_samples = data_container.Nofdm * frequency_interpolation_rate;
				int fe_M = data_container.interpolation_rate;
				int fe_sym_dec = data_container.Nofdm;
				int fe_buf_dec = data_container.Nofdm * data_container.buffer_Nsymb;
				int delay_dec = receive_stats.delay / fe_M;
				double fine_energy = 0.0;
				for(int i = 0; i < fe_sym_dec && (delay_dec + i) < fe_buf_dec; i++)
					fine_energy += std::norm(data_container.baseband_data_decimated[delay_dec + i]);
				fine_energy /= fe_sym_dec;
				if(fine_energy < energy_gate_floor)
				{
					int orig_delay = receive_stats.delay;
					int orig_delay_dec = orig_delay / fe_M;
					for(int fwd = sym_samples; fwd <= 3*sym_samples; fwd += sym_samples)
					{
						int candidate = orig_delay + fwd;
						int candidate_dec = orig_delay_dec + fwd / fe_M;
						if(candidate_dec + fe_sym_dec > fe_buf_dec) break;
						double e = 0.0;
						for(int i = 0; i < fe_sym_dec; i++)
							e += std::norm(data_container.baseband_data_decimated[candidate_dec + i]);
						e /= fe_sym_dec;
						if(e > energy_gate_floor)
						{
							if (g_verbose)
								printf("[OFDM-SYNC] fine-energy-fix: delay %d->%d (fwd %d sym)\n",
									orig_delay, candidate, fwd / sym_samples);
							fflush(stdout);
							receive_stats.delay = candidate;
							break;
						}
					}
				}
			}

		ofdm_subpeak_retry_point:
			// Use corrected carrier frequency if coarse sync applied
			double effective_carrier_freq = carrier_frequency + coarse_freq_offset;

			// Compute extraction range before data FIR so we can scope it.
			// LEVER P: a MINI tail frame is (rx_eff_preamble + Nsymb) symbols
			// long, not (preamble_nSymb + Nsymb). Extract exactly the actual
			// frame so the data-symbol demod offset (Nofdm*rx_eff_preamble below)
			// stays aligned with the copied region.
			int extraction_delay = receive_stats.delay;
			int buf_size_interp = data_container.Nofdm * data_container.buffer_Nsymb * frequency_interpolation_rate;
			int frame_size_interp = (data_container.Nofdm*(data_container.Nsymb+rx_eff_preamble))*frequency_interpolation_rate;
			// LEVER P: publish the eff preamble of this frame so the ARQ layer
			// advances ofdm_search_raw / frames_to_read by the actual frame length.
			receive_stats.last_eff_preamble_nsymb = rx_eff_preamble;
			if(extraction_delay < 0) extraction_delay = 0;
			if(extraction_delay > buf_size_interp - frame_size_interp)
				extraction_delay = buf_size_interp - frame_size_interp;

			// Scoped data FIR: only process the frame region + FIR margin.
			// The time_sync FIR already processed the full buffer for preamble search;
			// the data FIR only needs the frame. Saves 80-90% for high configs
			// where frame (10 symbols) is much smaller than buffer (113 symbols).
			int fir_margin = ofdm.FIR_rx_data.filter_nTaps * frequency_interpolation_rate;
			int pb_start = extraction_delay - fir_margin;
			if(pb_start < 0) pb_start = 0;
			int pb_end = extraction_delay + frame_size_interp + fir_margin;
			if(pb_end > buf_size_interp) pb_end = buf_size_interp;
			int pb_size = pb_end - pb_start;

			auto t2_pb = std::chrono::steady_clock::now();
			// Fused polyphase: mix + FIR + decimate-by-M in one pass. Writes
			// pb_size/M decimated samples to baseband_data_interpolated[0..],
			// then we extract the frame region (skipping FIR margin) to
			// baseband_data. Bit-exact equivalent to the old apply-then-pick
			// chain. (extraction_delay - pb_start) is fir_margin, a multiple
			// of M, so the index arithmetic is exact.
			{
				int Mdec = data_container.interpolation_rate;
				int margin_dec = (extraction_delay - pb_start) / Mdec;
				int frame_dec = frame_size_interp / Mdec;
				ofdm.passband_to_baseband_decimated(&data[pb_start], pb_size,
					data_container.baseband_data_interpolated,
					sampling_frequency, effective_carrier_freq, carrier_amplitude,
					Mdec, &ofdm.FIR_rx_data, pb_start);
				for(int i = 0; i < frame_dec; i++)
					data_container.baseband_data[i] = data_container.baseband_data_interpolated[margin_dec + i];
			}
			auto t3_pb = std::chrono::steady_clock::now();
			timing_pb_data_ms += std::chrono::duration<double, std::milli>(t3_pb - t2_pb).count();

			if(ofdm_forced_delay >= 0)
			{
				// BER test: true freq offset is 0, skip estimation
				freq_offset_measured = 0;
			}
			else if(receive_stats.sync_trials==effective_trials_max && use_last_good_freq_offset==YES && receive_stats.freq_offset_of_last_decoded_message!=0)
			{
				freq_offset_measured=receive_stats.freq_offset_of_last_decoded_message;
			}
			else if(narrowband_enabled)
			{
				// NB: skip fine freq sync entirely. Both Moose and
				// carrier_frequency_sync_nb are unreliable for NB:
				// - Moose: only 2 of 5 preamble SCs survive duplication-FFT
				// - carrier_frequency_sync_nb: gives -17 Hz on zero-offset channel
				//   after preamble changed to even-only subcarriers (Bug #41)
				// Coarse sync residual (±7.5 Hz max) is handled by ZF estimator
				// (no cross-pilot averaging → immune to phase rotation).
				freq_offset_measured = 0;
				if(g_verbose)
					printf("[NB-FREQ] skipped (relying on coarse sync + ZF)\n");
			}
			else if(rx_eff_preamble < 2)
			{
				// LEVER P: MINI preamble (1 symbol) — the Moose estimator needs >=2
				// preamble symbols for its symbol-to-symbol phase difference, so
				// reuse the last decoded frame's residual CFO (anchor frame 0
				// measured it for the batch). The per-symbol CPE_correction +
				// pilot-based ZF estimator below absorb residual drift per symbol,
				// so a stale-by-one-frame CFO on a clean/short tail frame is safe.
				// Risk under heavy phase noise / fast drift flagged for the PN sim.
				freq_offset_measured = receive_stats.freq_offset_of_last_decoded_message;
				if(g_verbose)
					printf("[WB-FREQ] MINI preamble — reuse last CFO=%.4f Hz\n", freq_offset_measured);
			}
			else
			{
				// Fine frequency sync (Moose algorithm) - ±0.5 subcarrier range
				// Note: Coarse offset already applied before this loop, so Moose measures residual
				// BUG FIX: baseband_data is at decimated (base) rate after rational_resampler,
				// so guard interval skip is Ngi samples, NOT Ngi*interpolation_rate.
				// The old code skipped Ngi*4=256=Nfft samples, reading across symbol boundaries.
				freq_offset_measured=ofdm.carrier_sampling_frequency_sync(&data_container.baseband_data[data_container.Ngi],bandwidth/(double)data_container.Nc,rx_eff_preamble, sampling_frequency);
				if(g_verbose)
					printf("[WB-FREQ] Moose=%.4f Hz\n", freq_offset_measured);
			}

			// Mini-Moose CFO refinement for WB MFSK data preamble
			// (data-preamble-port-research.md §20, §17 mirror-bin verdict).
			// PRE-FIX behavior was `if(M == MOD_MFSK) freq_offset_measured = 0;`
			// — the MFSK path discarded everything the OFDM branches above
			// computed and shipped zero residual CFO to the demod. Real signal
			// energy leaked into the mirror FFT bin, which the discrete-match
			// detector accepted as load-bearing. The new estimator
			// (ofdm.cc carrier_frequency_sync_wb_mfsk) measures the residual
			// from the freshly mixed baseband preamble; the existing sanity
			// clamp + re-mix block at lines 2195-2244 applies it (the MFSK
			// skip at the old line 2220 is dropped, see below).
			//
			// NB-MFSK explicitly kept at 0 — different tone geometry, separate
			// fix per §19.4 risk register. See data-flow-freq_offset_measured.md
			// §1.6 (INV-2).
			if(M == MOD_MFSK)
			{
				if(!narrowband_enabled)
				{
					double mini_moose = ofdm.carrier_frequency_sync_wb_mfsk(
						&data_container.baseband_data[0],
						bandwidth / (double)data_container.Nc,
						data_container.preamble_nSymb,
						mfsk.preamble_tones, mfsk.M,
						mfsk.nStreams, mfsk.stream_offsets);
					freq_offset_measured = mini_moose;
					if(g_verbose)
						printf("[MFSK-MINI-MOOSE] residual=%.4f Hz\n", freq_offset_measured);
				}
				else
				{
					freq_offset_measured = 0;
				}
			}

			// Moose sanity check + clamp.
			// Crystal oscillators on SGTL5000 boards typically differ by <20 Hz.
			// A Moose estimate > 30 Hz almost certainly means the preamble timing
			// is wrong (false Schmidl-Cox peak). Skip to next trial instead of
			// applying a wild correction that makes things worse.
			{
				double subcarrier_spacing = bandwidth / (double)data_container.Nc;
				double moose_sanity_limit = subcarrier_spacing * 2.0;  // ~93.75 Hz for WB (Moose nIS/2 capture range)
				if(g_verbose)
					printf("[MOOSE-RAW] unclamped=%.4f Hz, sanity=%.1f Hz\n", freq_offset_measured, moose_sanity_limit);
				if(fabs(freq_offset_measured) > moose_sanity_limit && receive_stats.sync_trials < effective_trials_max)
				{
					printf("[MOOSE-REJECT] freq=%.1f Hz exceeds sanity limit — bad timing, advancing trial (sack_xcheck=%d trials=%d/%d)\n",
						freq_offset_measured, sack_cross_check_mode ? 1 : 0,
						receive_stats.sync_trials + 1, effective_trials_max);
					fflush(stdout);
					receive_stats.sync_trials++;
					continue;
				}
				// Clamp to ±1 subcarrier spacing (covers real offsets up to ~47 Hz)
				double max_correction = subcarrier_spacing;
				if(freq_offset_measured > max_correction) freq_offset_measured = max_correction;
				if(freq_offset_measured < -max_correction) freq_offset_measured = -max_correction;
			}

			// Pre-fix this branch had `if(M == MOD_MFSK) { /* skip */ }`
			// short-circuiting MFSK out of the fine re-mix. Mini-Moose
			// (data-preamble-port-research.md §20) made MFSK eligible
			// for the same re-mix as OFDM: the noncoherent FFT-energy
			// demap benefits from CFO correction because in-bin energy
			// concentration is its only signal axis. NB-MFSK still has
			// freq_offset_measured = 0 (set above), so the
			// freq_offset_ignore_limit gate keeps it out of the re-mix
			// path implicitly.
			//
			// §23 sign-flip experiment (data-preamble-port-research.md §23,
			// data-flow-freq_offset_measured.md §11): apply uses
			// `effective_carrier_freq - freq_offset_measured` instead of `+`.
			// Motivated by §22 control-frame mini-Moose hardware regression
			// (-40% total bytes) combined with the NB-vs-WB-MFSK estimator
			// sign-convention disagreement (ofdm.cc:603 NB derivation uses
			// `-arg(C)*...` while ofdm.cc:799 WB MFSK returns `+arg(C)*...`).
			// The new apply-sign-invariance regression test
			// (mfsk_data_preamble_mini_moose_apply_sign_invariance, §7.4 in
			// mfsk_ctrl_codec_tests.cc) ground-truths the chain end-to-end.
			// Hardware A/B at §20-verify cells will resolve which sign is
			// correct on real IONOS / RF.
			if(fabs(freq_offset_measured)>ofdm.freq_offset_ignore_limit)
			{
				// Apply fine correction on top of coarse correction (scoped to frame region)
				auto t6_pb = std::chrono::steady_clock::now();
				// Fused polyphase (fine-frequency-correction branch).
				{
					int Mdec = data_container.interpolation_rate;
					int margin_dec = (extraction_delay - pb_start) / Mdec;
					int frame_dec = frame_size_interp / Mdec;
					ofdm.passband_to_baseband_decimated(&data[pb_start], pb_size,
						data_container.baseband_data_interpolated,
						sampling_frequency,
						effective_carrier_freq - freq_offset_measured,
						carrier_amplitude,
						Mdec, &ofdm.FIR_rx_data, pb_start);
					for(int i = 0; i < frame_dec; i++)
						data_container.baseband_data[i] = data_container.baseband_data_interpolated[margin_dec + i];
				}
				auto t7_pb = std::chrono::steady_clock::now();
				timing_pb_data_ms += std::chrono::duration<double, std::milli>(t7_pb - t6_pb).count();
			}
			{
				int rx_nsymb = get_active_nsymb();
				// LEVER P: data symbols begin right after the (possibly MINI)
				// preamble — offset Nofdm*rx_eff_preamble, not Nofdm*preamble_nSymb.
				for(int i=0;i<rx_nsymb;i++)
				{
					ofdm.symbol_demod(&data_container.baseband_data[i*data_container.Nofdm+data_container.Nofdm*rx_eff_preamble],&data_container.ofdm_symbol_demodulated_data[i*data_container.Nc]);
				}
			}

			if(M == MOD_MFSK)
			{
				// MFSK: non-coherent energy detection on FFT output → soft LLRs
				int rx_nbits = get_active_nbits();
				mfsk.demod(data_container.ofdm_symbol_demodulated_data, rx_nbits, data_container.demodulated_data);

#ifdef MERCURY_GUI_ENABLED
				// Accumulate tone energies across ALL symbols for full-packet view
				{
					int nSymbols = rx_nbits / (mfsk.nBits * mfsk.nStreams);
					double gui_E[2][64] = {};
					int gui_peak[2] = {};
					for (int st = 0; st < mfsk.nStreams && st < 2; st++)
					{
						for (int s = 0; s < nSymbols; s++)
						{
							int hop = (s * mfsk.tone_hop_step) % mfsk.M;
							for (int m = 0; m < mfsk.M && m < 64; m++)
							{
								std::complex<double> val = data_container.ofdm_symbol_demodulated_data[
									s * data_container.Nc + mfsk.stream_offsets[st] + ((m + hop) % mfsk.M)];
								gui_E[st][m] += val.real() * val.real() + val.imag() * val.imag();
							}
						}
						// Peak = last symbol's active tone
						if (nSymbols > 0) {
							int last_s = nSymbols - 1;
							int hop = (last_s * mfsk.tone_hop_step) % mfsk.M;
							double max_e = -1.0;
							for (int m = 0; m < mfsk.M && m < 64; m++)
							{
								std::complex<double> val = data_container.ofdm_symbol_demodulated_data[
									last_s * data_container.Nc + mfsk.stream_offsets[st] + ((m + hop) % mfsk.M)];
								double e = val.real() * val.real() + val.imag() * val.imag();
								if (e > max_e) { max_e = e; gui_peak[st] = m; }
							}
						}
					}
					gui_push_mfsk_tones(gui_E, gui_peak, mfsk.M, mfsk.nStreams, false);
				}
#endif

				// Zero-pad LLRs beyond active bits (punctured positions = erasure)
				// This covers both ctrl mode and BER test puncturing
				int puncture_from = rx_nbits;
				if(test_puncture_nBits > 0 && test_puncture_nBits < puncture_from)
					puncture_from = test_puncture_nBits;
				for(int i = puncture_from; i < data_container.nBits; i++)
				{
					data_container.demodulated_data[i] = 0.0f;
				}

			}
			else
			{
				ofdm.automatic_gain_control(data_container.ofdm_symbol_demodulated_data);
				// CPE correction: remove residual freq offset before channel estimation.
				// Previously NB-only, but WB also benefits (reduces pilot residuals).
				ofdm.CPE_correction(data_container.ofdm_symbol_demodulated_data);

				if(ofdm.channel_estimator==ZERO_FORCE)
				{
					ofdm.ZF_channel_estimator(data_container.ofdm_symbol_demodulated_data);
				}
				else if (ofdm.channel_estimator==LEAST_SQUARE)
				{
					ofdm.LS_channel_estimator(data_container.ofdm_symbol_demodulated_data);
				}

				mean_H = -1.0;
				int h_count = 0;
				{
					double h_sum = 0;
					for(int ci = 0; ci < ofdm.Nsymb * ofdm.Nc; ci++)
					{
						if(ofdm.estimated_channel[ci].status == MEASURED)
						{
							h_sum += std::abs(ofdm.estimated_channel[ci].value);
							h_count++;
						}
					}
					if(h_count > 0) mean_H = h_sum / h_count;
				}
				// Test-observability: expose the per-trial mean(|H|) the SKIP-H
				// gate keys on. Write-once-per-trial, read by unit tests only
				// (ofdm-fine-timing-magnitude.md §3.5). No control-flow effect.
				receive_stats.mean_H = mean_H;
				// Cache channel selectivity = std(|H[k]|) / mean(|H[k]|) over DATA
				// subcarriers for the 2D channel-state lookup (§3.2 of fact-doc
				// channel-state-2d-lookup.md). At this point estimated_channel[]
				// has been smoothed by smooth_channel_estimate_dft() and the data
				// bins carry interpolated |H| values (restore_channel_amplitude
				// hasn't run yet — that would flatten all magnitudes to 1.0).
				// Data bins are identified by ofdm_frame[].type == DATA, which
				// skips pilots and guard/zero subcarriers.
				{
					double s_sum = 0.0, s_sumsq = 0.0;
					int s_count = 0;
					for(int i = 0; i < ofdm.Nsymb; i++)
					{
						for(int j = 0; j < ofdm.Nc; j++)
						{
							if((ofdm.ofdm_frame + i*ofdm.Nc + j)->type == DATA)
							{
								double mag = std::abs(ofdm.estimated_channel[i*ofdm.Nc + j].value);
								s_sum   += mag;
								s_sumsq += mag * mag;
								s_count++;
							}
						}
					}
					if(s_count > 1)
					{
						double s_mean = s_sum / s_count;
						double s_var  = (s_sumsq / s_count) - (s_mean * s_mean);
						if(s_var < 0.0) s_var = 0.0;  // numerical guard
						double s_std  = sqrt(s_var);
						last_channel_selectivity = (s_mean > 1e-12) ? (s_std / s_mean) : -1.0;
					}
					else
					{
						last_channel_selectivity = -1.0;
					}
				}
				// Schmidl-Cox sub-peak rejection (dual-condition gate).
				// Sub-peaks INSIDE the OFDM data body produce saturating
				// metric (≥0.97, often 1.000) but the resulting channel
				// estimate collapses (mean|H| well below the real-preamble
				// floor of ~0.74). Real preambles cap around metric≈0.92 in
				// clean WB (see comment near telecom_system.cc:1416), so the
				// (metric≥0.97 AND mean_H<0.5) combination is a strong
				// sub-peak fingerprint that the looser SKIP-H mean_H<0.30
				// threshold lets through. Rejecting these here saves the
				// 101-iter LDPC burn and frees the trial budget for the real
				// preamble. Counts toward skip_h_count so the existing
				// SKIP-H recovery (line ~2622) scans forward after the
				// budget is spent.
				if(receive_stats.coarse_metric >= 0.97 && mean_H < 0.5)
				{
					printf("[SUBPEAK-REJECT] trial %d metric=%.3f mean_H=%.3f delay=%d — Schmidl-Cox sub-peak rejected\n",
						receive_stats.sync_trials, receive_stats.coarse_metric, mean_H, receive_stats.delay);
					fflush(stdout);
					skip_h_count++;
					receive_stats.sync_trials++;
					continue;
				}
				{
					// Timing-quality gate: if mean|H| < 0.30, the preamble timing
				// is almost certainly wrong by 1+ OFDM symbols. Pilots land on
				// data positions where LS gives |H|≈0 (random phase cancellation).
				// Even CONFIG_0 (rate 1/16) can't decode below ~0.33.
				// Good-timing frames: meanH ≥ 0.74 (SGTL5000 at -40 dBFS).
				// Bad-timing frames: meanH = 0.07-0.24 (data/pilot misalignment).
				// Phase-2: --mean-h-gate=F overrides.
				double mean_H_threshold = mean_h_gate_threshold;
					if(mean_H < mean_H_threshold)
					{
						skip_h_count++;
						{
							printf("[OFDM-SYNC] trial %d SKIP-H: mean_H=%.4f too low (threshold=%.2f), skipping LDPC\n",
								receive_stats.sync_trials, mean_H, mean_H_threshold);
							// Diagnostic: print first 8 pilot |H| values and positions
							if(skip_h_count <= 2) {
								int p_printed = 0;
								printf("[H-DIAG] Nc=%d Nsymb=%d h_count=%d delay=%d FIR_cut=%.1f bw=%.1f\n",
									ofdm.Nc, ofdm.Nsymb, h_count, receive_stats.delay,
									ofdm.FIR_rx_data.lpf_filter_cut_frequency, bandwidth);
								for(int ci = 0; ci < ofdm.Nsymb * ofdm.Nc && p_printed < 8; ci++) {
									if(ofdm.estimated_channel[ci].status == MEASURED) {
										printf("[H-DIAG]  pilot[%d/%d] |H|=%.6f phase=%.1f\n",
											ci/ofdm.Nc, ci%ofdm.Nc,
											std::abs(ofdm.estimated_channel[ci].value),
											std::arg(ofdm.estimated_channel[ci].value) * 180.0 / M_PI);
										p_printed++;
									}
								}
								fflush(stdout);
							}
						}
						fflush(stdout);
						receive_stats.sync_trials++;
						continue;
					}
				}

				// Noise variance gate: reject the frame before LDPC when the pilot-
				// residual noise estimate exceeds this config's decode ceiling — the
				// frame is either noise (false preamble) or below this config's LDPC
				// floor. Good frames at moderate SNR: var=0.01-0.10. Skip LDPC to free
				// the receiver for real frames.
				// Phase-2 validation: --skip-var-gate=off bypasses this gate.
				//
				// FIX-1 (2026-06-04): the ceiling is now CONFIG-RATE-AWARE
				// (skip_var_nv_ceiling), not a flat 0.5. The flat 0.5 was code-rate-
				// INDEPENDENT and sat below every low-rate BPSK config's MEASURED
				// decode-floor nv (CONFIG_0 decodes to nv≈1.48), so it dropped weak
				// frames LDPC could still decode — masking real margin on CONFIG_0-6.
				// CONFIG_7-16 + robust/NB keep 0.5 (decoder-bound, strict no-op).
				// CONFIG_0's 1.60 is the hard cap; LDPC+CRC reject everything above.
				//
				// Phase-F stall fix (2026-05-12): if 3+ consecutive trials all
				// SKIP-VAR, the entire trial range is noise — abort the trial
				// loop entirely so the caller advances the buffer past this
				// noise region instead of burning 20 trials (~2 s) on it.
				double skip_var_ceiling = skip_var_nv_ceiling(current_configuration);
				if(skip_var_gate_enabled && ofdm.noise_variance_estimate > skip_var_ceiling)
				{
					printf("[OFDM-SYNC] trial %d SKIP-VAR: var=%.4f too high (>%.2f cfg=%d), skipping LDPC\n",
						receive_stats.sync_trials, ofdm.noise_variance_estimate,
						skip_var_ceiling, current_configuration);
					fflush(stdout);
					receive_stats.sync_trials++;
					consecutive_skip_var++;
					if(consecutive_skip_var >= 3)
					{
						printf("[OFDM-SYNC] %d consecutive SKIP-VAR — abort trial loop, advance buffer\n",
							consecutive_skip_var);
						fflush(stdout);
						// Signal caller to zero the false preamble region and advance
						// the search cursor past it. Without this, the next receive_byte
						// call re-locks on the same noise and we burn another 3 trials.
						receive_stats.frame_skip_var_aborted = true;
						break;
					}
					continue;
				}
				consecutive_skip_var = 0;  // reset on any non-SKIP-VAR path

				if(ofdm.channel_estimator_amplitude_restoration==YES)
				{
					ofdm.restore_channel_amplitude();
					ofdm.channel_equalizer_without_amplitude_restoration(data_container.ofdm_symbol_demodulated_data,data_container.equalized_data_without_amplitude_restoration);
					ofdm.deframer(data_container.equalized_data_without_amplitude_restoration,data_container.ofdm_deframed_data_without_amplitude_restoration);
				}

				ofdm.channel_equalizer(data_container.ofdm_symbol_demodulated_data,data_container.equalized_data);
				double measure_var=ofdm.measure_variance(data_container.equalized_data);
				// CSI-weighted LLR for ZF equalization.
				// ZF amplifies noise by 1/|H_k|² per subcarrier. Using a single
				// global variance makes the decoder overconfident on weak subcarriers.
				// Fix: demod with pre-ZF noise (σ²_n), then scale each symbol's LLRs
				// by normalized |H_k|² → tells LDPC which bits to trust.
				// CSI from DFT-smoothed estimated_channel gives per-subcarrier |H|²,
				// normalized by mean to prevent LLR saturation.
				variance = ofdm.noise_variance_estimate;

				// CFG16 32-QAM freq-selective decode fix (re-applied to monitor; equiv of 2d540d9,
				// fact-documents/data-flow-noise_variance_estimate.md §4/§5).
				//
				// noise_variance_estimate (cross-pilot differential, A.1.4 commit 9c3fc40) measures
				// the PRE-equalization channel noise σ²/|X|² in raw bins. The demapper below operates
				// on the EQUALIZED constellation (Y/H), whose per-symbol noise is σ²·E[1/|H|²] plus
				// the DFT-smoother/interpolation residual EVM — a larger quantity that does NOT vanish
				// when the channel is frequency-selective (where the post-EQ EVM diverges from the
				// pre-EQ nv). On such a channel nv UNDER-estimates the equalized noise; feeding it to
				// psk.demod scaled LLRs over-confident; the E3 clip let those wrong-sign LLRs reach
				// ±40, flipping inner 32-QAM bit signs → BP hit the iter cap → CRC fail → 0 bps.
				// 16-QAM (CFG15) tolerated the wrong magnitude (larger min-distance); 32-QAM did not.
				//
				// Root fix: substitute demap_variance = measure_var ONLY when nv has
				// catastrophically collapsed. measure_var (telecom_system.cc above =
				// ofdm.measure_variance, the mean post-EQ pilot residual |Y_pilot/H − X|²) IS the
				// noise variance on the equalized constellation the Euclidean demapper needs
				// (psk.cc LLR = ΔD/variance) — a MEASURED quantity, not a tuned constant.
				//
				// RATIO-GATE (cfg16-nvfix v2): the EARLIER UNCONDITIONAL max(nv,measure_var) REGRESSED
				// — on common frequency-selective channels measure_var legitimately exceeds nv by a
				// modest factor (~1.0× flat, up to ~3.0× under amp=0.6/dly=64; MEASURED, see
				// _cfg16hold2/meas_fsel_*.log), and forcing demap_variance up to measure_var there
				// raised the demap noise where nv was LEGITIMATELY low → CFG15/CFG16 BER 4-10× WORSE.
				// The HARDWARE bug is a different regime: nv COLLAPSES ~1000× below the true post-EQ
				// noise (HW-only post-EQ-EVM phenomenon the in-process sim never reproduces). So gate
				// the substitution on a RATIO: only override when nv < measure_var / K. With K=8 the
				// worst legitimate freq-selective ratio observed (measure_var/nv ≈ 2.98) is far below
				// the K threshold → the gate is a NO-OP on flat AND freq-selective (byte-identical to
				// base, both confirmed by A/B), engaging ONLY on the catastrophic ~1000× collapse.
				//
				// Scope (CLAUDE.md §5 — see PLAN.md §5 audit): demap_variance feeds ONLY the two
				// psk.demod LLR calls below. The local `variance` is left UNCHANGED so (a) the
				// LS-path SNR report (10*log10(1/variance)) and the gearshift that consumes it are
				// byte-identical, and (b) ofdm.noise_variance_estimate itself is untouched, so the
				// MMSE-ZF erasure (ofdm.cc alpha=H²/(H²+nv)) and the SKIP-VAR sync gate are unchanged.
				// MFSK never reaches this branch (own guard-bin estimate), so A.1.4 ROBUST is unaffected.
				// HELD UNMERGED: the improvement is HW-only-validatable (collapse isn't in sim) —
				// for the bench-9 HW A/B. In sim it is a proven NO-OP (this branch's whole point).
				const double NV_COLLAPSE_RATIO_K = 8.0;
				double demap_variance = ofdm.noise_variance_estimate;
				if(ofdm.noise_variance_estimate < measure_var / NV_COLLAPSE_RATIO_K)
					demap_variance = measure_var;

				printf("[FRAME-NV] trial=%d cfg=%d nv=%.6e mvar=%.4f demap_var=%.6e Nsymb=%d amprest=%d\n",
					receive_stats.sync_trials, current_configuration,
					ofdm.noise_variance_estimate, measure_var, demap_variance, ofdm.Nsymb,
					ofdm.channel_estimator_amplitude_restoration);
				fflush(stdout);

				if(csi_llr_enabled) {
				// Extract per-subcarrier CSI weight |H_k|² and deframe (DATA cells only).
				// DFT-smoothed estimated_channel has true |H|², giving per-subcarrier
				// reliability to LDPC. Normalized by mean to prevent LLR saturation.
				struct st_channel_complex* csi_source = ofdm.estimated_channel;
				float* csi_deframed = new float[data_container.nData];
				int csi_di = 0;
				for(int si = 0; si < ofdm.Nsymb; si++)
				{
					for(int sj = 0; sj < ofdm.Nc; sj++)
					{
						if((ofdm.ofdm_frame + si * ofdm.Nc + sj)->type == DATA)
						{
							std::complex<double> H = (csi_source + si * ofdm.Nc + sj)->value;
							csi_deframed[csi_di++] = (float)(H.real() * H.real() + H.imag() * H.imag());
						}
					}
				}
				// Deinterleave CSI weights (same path as equalized data)
				float* csi_deinterleaved = new float[data_container.nData];
				deinterleaver(csi_deframed, csi_deinterleaved, data_container.nData, time_freq_interleaver_block_size);
				delete[] csi_deframed;

				ofdm.deframer(data_container.equalized_data,data_container.ofdm_deframed_data);
				deinterleaver(data_container.ofdm_deframed_data, data_container.ofdm_time_freq_deinterleaved_data, data_container.nData, time_freq_interleaver_block_size);
				psk.demod(data_container.ofdm_time_freq_deinterleaved_data,data_container.nBits,data_container.demodulated_data,(float)demap_variance);

				// Normalize CSI weights by mean → average weight = 1.0.
				// This preserves relative per-subcarrier quality (tells LDPC which
				// bits are reliable) without changing overall LLR magnitude.
				// For phase-only EQ (PSK), LLRs already encode |H| implicitly in
				// constellation distances. Normalized CSI adds the RELATIVE variation:
				// strong subcarrier → weight > 1 → higher LLR confidence
				// weak subcarrier → weight < 1 → lower LLR confidence
				// On flat channels (AWGN), all weights ≈ 1 → no change.
				int nBps = (int)log2(M);
				float mean_w = 0;
				for(int si = 0; si < data_container.nData; si++)
					mean_w += csi_deinterleaved[si];
				mean_w /= data_container.nData;
				if(mean_w < 1e-6f) mean_w = 1.0f;

				for(int si = 0; si < data_container.nData; si++)
				{
					float w = csi_deinterleaved[si] / mean_w;
					for(int bi = 0; bi < nBps; bi++)
					{
						float llr = data_container.demodulated_data[si * nBps + bi] * w;
						if(llr > 40.0f) llr = 40.0f;
						else if(llr < -40.0f) llr = -40.0f;
						data_container.demodulated_data[si * nBps + bi] = llr;
					}
				}
				delete[] csi_deinterleaved;
				} else {
					// Phase-2: --csi-llr=off — uniform LLR path (pre-IONOS behavior).
					ofdm.deframer(data_container.equalized_data,data_container.ofdm_deframed_data);
					deinterleaver(data_container.ofdm_deframed_data, data_container.ofdm_time_freq_deinterleaved_data, data_container.nData, time_freq_interleaver_block_size);
					psk.demod(data_container.ofdm_time_freq_deinterleaved_data,data_container.nBits,data_container.demodulated_data,(float)demap_variance);
				}
			}

			deinterleaver(data_container.demodulated_data,data_container.deinterleaved_data,data_container.nBits,bit_interleaver_block_size);

			for(int i=ldpc.P-1;i>=0;i--)
			{
				data_container.deinterleaved_data[i+nReal_data+nVirtual_data]=data_container.deinterleaved_data[i+nReal_data];
			}

			for(int i=0;i<nVirtual_data;i++)
			{
				data_container.deinterleaved_data[nReal_data+i]=data_container.deinterleaved_data[i];
			}

			auto t4_ldpc = std::chrono::steady_clock::now();
			receive_stats.iterations_done=ldpc.decode(data_container.deinterleaved_data,data_container.hd_decoded_data_bit);
			auto t5_ldpc = std::chrono::steady_clock::now();
			timing_ldpc_ms += std::chrono::duration<double, std::milli>(t5_ldpc - t4_ldpc).count();

			bit_energy_dispersal(data_container.hd_decoded_data_bit, data_container.bit_energy_dispersal_sequence, data_container.hd_decoded_data_bit, nReal_data);


			bit_to_byte(data_container.hd_decoded_data_bit, data_container.hd_decoded_data_byte, nReal_data);


			receive_stats.all_zeros=YES;
			for(int i=0;i<nReal_data/8;i++)
			{
				if(data_container.hd_decoded_data_byte[i]!=0)
				{
					receive_stats.all_zeros=NO;
					break;
				}
			}

			for(int i=0;i<(nReal_data-outer_code_reserved_bits)/8;i++)
			{
				*(out+i)=data_container.hd_decoded_data_byte[i];
			}

			// CRC16 self-check: compute CRC over [data + CRC_LSB + CRC_MSB] = nReal_data/8 bytes.
			// For correct data, CRC16_MODBUS_RTU of [message || appended_CRC] = 0.
			// Check on ALL frames (not just LDPC failures) to catch wrong-codeword convergence.
			receive_stats.crc=0;
			if(outer_code == CRC16_MODBUS_RTU && receive_stats.all_zeros == NO)
			{
				receive_stats.crc=CRC16_MODBUS_RTU_calc(data_container.hd_decoded_data_byte, nReal_data/8);
			}

			if(receive_stats.all_zeros==YES ||
			   (outer_code == CRC16_MODBUS_RTU && receive_stats.crc != 0) ||
			   (outer_code != CRC16_MODBUS_RTU && receive_stats.iterations_done > (ldpc.nIteration_max-1)))
			{
				receive_stats.SNR=-99.9;
				receive_stats.message_decoded=NO;
				if(M != MOD_MFSK)
				{
					// Always log OFDM decode failures — needed for HF diagnostics
					printf("[OFDM-FAIL] t%d cfg=%d delay=%d iter=%d zeros=%d freq=%.1f var=%.4f meanH=%.3f coarse=%.1f crc=0x%04X\n",
						receive_stats.sync_trials, current_configuration,
						receive_stats.delay, receive_stats.iterations_done,
						receive_stats.all_zeros, freq_offset_measured, variance,
						mean_H, coarse_freq_offset, receive_stats.crc);
					fflush(stdout);
				}
				// v2 OFDM sub-peak recovery: LDPC failed at the iteration cap
				// despite coarse_metric saturating (sub-peak fingerprint) and
				// mean_H surviving the v1 reject (≥ 0.5). Sub-peaks on Schmidl-
				// Cox autocorrelation land at OFDM-symbol-aligned offsets, so
				// probe ±1 OFDM symbol before giving up. Cheaper than letting
				// the partial-batch SACK-retx round-trip stall PPMd+zstd
				// streaming decompression on the receiver. One-shot per frame:
				// max 2 retries (+sym then -sym), then fall through.
				if(M != MOD_MFSK
					&& receive_stats.coarse_metric >= 0.97
					&& mean_H >= 0.5
					&& receive_stats.iterations_done > (ldpc.nIteration_max-1)
					&& subpeak_recover_phase < 2)
				{
					int sym_samples = data_container.Nofdm * frequency_interpolation_rate;
					int buf_size_full = data_container.Nofdm * data_container.buffer_Nsymb * frequency_interpolation_rate;
					int frame_size_full = (data_container.Nofdm*(data_container.Nsymb+data_container.preamble_nSymb))*frequency_interpolation_rate;
					int max_delay = buf_size_full - frame_size_full;
					if(subpeak_recover_phase == 0)
						subpeak_orig_delay = receive_stats.delay;
					int candidate = (subpeak_recover_phase == 0)
						? (subpeak_orig_delay + sym_samples)
						: (subpeak_orig_delay - sym_samples);
					if(candidate >= 0 && candidate <= max_delay)
					{
						subpeak_recover_phase++;
						subpeak_recover_in_flight = true;
						printf("[SUBPEAK-PROBE] phase=%d orig_delay=%d new_delay=%d metric=%.3f mean_H=%.3f iter=%d — retry ±1 OFDM sym\n",
							subpeak_recover_phase, subpeak_orig_delay, candidate,
							receive_stats.coarse_metric, mean_H, receive_stats.iterations_done);
						fflush(stdout);
						receive_stats.delay = candidate;
						// Do NOT increment sync_trials: this is a re-do of the
						// same trial at a shifted position, not a new search.
						goto ofdm_subpeak_retry_point;
					}
				}
				subpeak_recover_in_flight = false;
				receive_stats.sync_trials++;
			}
			else
			{
				if(M == MOD_MFSK)
				{
					// MFSK: no channel estimation, skip variance-based SNR
					// TODO: estimate SNR from peak tone energy vs noise energy
					receive_stats.SNR = 0.0;
				}
				else if(ofdm.channel_estimator==LEAST_SQUARE)
				{
					if(ofdm.channel_estimator_amplitude_restoration==YES)
					{
						variance=ofdm.measure_variance(data_container.equalized_data_without_amplitude_restoration);
					}
					receive_stats.SNR=10.0*log10(1.0/variance);
				}
				else if(ofdm.channel_estimator==ZERO_FORCE)
				{
					// ZF SNR measurement: re-encode decoded bits to get ideal
					// modulated symbols, then compare with received symbols.
					// Must re-apply bit_energy_dispersal (undo the descrambling
					// done earlier) since the LDPC encoder expects scrambled data.
					// Use a temp buffer to avoid corrupting hd_decoded_data_bit
					// (which the BER test compares against the original data).
					int temp_bits[N_MAX];
					memcpy(temp_bits, data_container.hd_decoded_data_bit, nReal_data * sizeof(int));
					bit_energy_dispersal(temp_bits, data_container.bit_energy_dispersal_sequence, temp_bits, nReal_data);

					for(int i=0;i<nVirtual_data;i++)
					{
						temp_bits[nReal_data+i]=temp_bits[i];
					}
					ldpc.encode(temp_bits,data_container.encoded_data);
					for(int i=0;i<ldpc.P;i++)
					{
						data_container.encoded_data[nReal_data+i]=data_container.encoded_data[i+ldpc.K];
					}
					interleaver(data_container.encoded_data,data_container.bit_interleaved_data,data_container.nBits,bit_interleaver_block_size);
					psk.mod(data_container.bit_interleaved_data,data_container.nBits,data_container.modulated_data);
					interleaver(data_container.modulated_data, data_container.ofdm_time_freq_interleaved_data, data_container.nData, time_freq_interleaver_block_size);
					if(ofdm.channel_estimator_amplitude_restoration==YES)
					{
						receive_stats.SNR=ofdm.measure_SNR(data_container.ofdm_deframed_data_without_amplitude_restoration,data_container.ofdm_time_freq_interleaved_data,data_container.nData);
					}
					else
					{
						receive_stats.SNR=ofdm.measure_SNR(data_container.ofdm_deframed_data,data_container.ofdm_time_freq_interleaved_data,data_container.nData);
					}

				}

				receive_stats.message_decoded=YES;
				if(M != MOD_MFSK)
				{
					printf("[OFDM-OK] t%d cfg=%d delay=%d iter=%d freq=%.1f var=%.4f meanH=%.3f SNR=%.1f coarse=%.1f\n",
						receive_stats.sync_trials, current_configuration,
						receive_stats.delay, receive_stats.iterations_done,
						freq_offset_measured, variance, mean_H,
						receive_stats.SNR, coarse_freq_offset);
					fflush(stdout);
					// v2 OFDM sub-peak recovery success: log the save so we can
					// measure v2's contribution to sack_rate / compression yield
					// in post-deploy benchmarks. Counts the frames that v1 would
					// have lost but v2 reclaimed without an SACK round-trip.
					if(subpeak_recover_in_flight)
					{
						printf("[SUBPEAK-RECOVER] phase=%d orig_delay=%d saved_delay=%d shift=%+d iter=%d — frame reclaimed (no SACK retx needed)\n",
							subpeak_recover_phase, subpeak_orig_delay,
							receive_stats.delay, receive_stats.delay - subpeak_orig_delay,
							receive_stats.iterations_done);
						fflush(stdout);
					}
				}
				subpeak_recover_in_flight = false;

#ifdef MERCURY_GUI_ENABLED
				// Push fully-equalized data for visualization (tight clusters).
				// Amplitude restoration still helps the decoder via the separate path.
				if (M != MOD_MFSK) {
					gui_push_constellation(data_container.ofdm_deframed_data, data_container.nData, (int)M, false);
				} else {
					gui_push_constellation(nullptr, 0, (int)M, true);
				}
#endif

				// Only store freq offset for OFDM modes — MFSK runs the Moose estimator
				// on non-OFDM preamble data producing a garbage value (~45 Hz) that would
				// corrupt OFDM decoding after gearshift (use_last_good_freq_offset fallback).
				if(M != MOD_MFSK)
				{
					receive_stats.freq_offset_of_last_decoded_message=freq_offset_measured;
					receive_stats.freq_offset=freq_offset_measured;
					// STALE-CFO scoped reset (long-run-degradation.md §2.2): a fresh
					// OFDM decode succeeded, so the latched CFO is GOOD again — clear the
					// consecutive-fail run that gates the poison-CFO scrub at function
					// exit. On a healthy link this keeps the counter pinned at 0, so the
					// scrub NEVER fires and the deliberate last-trial / MINI-preamble
					// (LEVER P) / cfg=6 stickiness is fully preserved.
					consecutive_ofdm_decode_fails = 0;
				}

				receive_stats.delay_of_last_decoded_message=receive_stats.delay;
				break;
			}
		}

		// SKIP-H recovery: if all trials failed with low channel estimate,
		// the detected preamble was likely a false peak (e.g. residual MFSK
		// ACK tones looping back through VB-Cable). Search forward for a
		// later, real preamble and retry the trial loop.
		if(receive_stats.message_decoded != YES
			&& skip_h_count >= time_sync_trials_max + 1
			&& !skip_h_recovery_attempted)
		{
			skip_h_recovery_attempted = true;
			int sym_samples = data_container.Nofdm * frequency_interpolation_rate;
			int buf_samples = data_container.Nofdm * data_container.buffer_Nsymb * frequency_interpolation_rate;

			// Start searching 2 symbols past the false preamble
			int search_start_symb = pream_symb_loc + 2;
			int search_start = search_start_symb * sym_samples;
			int search_size = data_container.Nofdm *
				(2 * data_container.preamble_nSymb + data_container.Nsymb) *
				frequency_interpolation_rate;
			int available = buf_samples - search_start;
			if(available > search_size) available = search_size;

			if(search_start_symb < upper_bound
				&& available > data_container.preamble_nSymb * sym_samples)
			{
				// Plan-B Step 3/6c (site 2247, SKIP-H recovery): the coarse
				// search runs on a freshly re-mixed DECIMATED buffer. SKIP-H
				// resets coarse_freq_offset = 0 before the goto, so the re-mix
				// (and everything after the goto) is at carrier_frequency.
				// The old full-rate baseband_data_interpolated re-mix here was
				// removed in Step 6c — site 8 (Step 6b) sources its fine sync
				// from a scoped slice off raw `data`, and every energy gate
				// after the goto (Step 6a) reads baseband_data_decimated, so
				// only the decimated re-mix is needed.
				int p3_M = data_container.interpolation_rate;
				int p3_full_size = data_container.Nofdm * data_container.buffer_Nsymb * frequency_interpolation_rate;
				ofdm.passband_to_baseband_decimated((double*)data,
					p3_full_size,
					data_container.baseband_data_decimated,
					sampling_frequency, carrier_frequency, carrier_amplitude,
					p3_M, &ofdm.FIR_rx_time_sync);

				// Schmidl-Cox autocorrelation: preamble L-sample periodicity
				// discriminates preamble from data (data has no L-period).
				int search_start_dec = search_start / p3_M;
				int available_dec = available / p3_M;
				TimeSyncResult retry = ofdm.time_sync_preamble_halfsym(
					&data_container.baseband_data_decimated[search_start_dec],
					available_dec, 1, 1);
				retry.delay = retry.delay * p3_M + search_start;

				int retry_symb = retry.delay / sym_samples;
				if(retry_symb < 1) retry_symb = 1;

				// Check energy at the retry position (Step 6a: decimated
				// buffer; retry.delay full-rate, floored to /M).
				double retry_energy = 0.0;
				int rcnt = 0;
				int sh_sym_dec = data_container.Nofdm;
				int sh_buf_dec = data_container.Nofdm * data_container.buffer_Nsymb;
				int retry_delay_dec = retry.delay / p3_M;
				for(int i = 0; i < sh_sym_dec && (retry_delay_dec + i) < sh_buf_dec; i++)
				{
					double re = data_container.baseband_data_decimated[retry_delay_dec + i].real();
					double im = data_container.baseband_data_decimated[retry_delay_dec + i].imag();
					retry_energy += re*re + im*im;
					rcnt++;
				}
				retry_energy = (rcnt > 0) ? retry_energy / rcnt : 0.0;

				if (g_verbose)
					printf("[OFDM-SYNC] SKIP-H recovery: orig=%d retry=%d metric=%.3f energy=%.2e\n",
						pream_symb_loc, retry_symb, retry.correlation, retry_energy);
				fflush(stdout);

				// Schmidl-Cox metric >= threshold = confirmed preamble.
				// The trial loop's mean_H check provides additional validation.
				if(retry_energy >= 0.001 && retry.correlation >= preamble_detect_threshold
					&& retry_symb > lower_bound && retry_symb <= upper_bound)
				{
					receive_stats.delay = retry.delay;
					receive_stats.coarse_metric = retry.correlation;
					pream_symb_loc = retry_symb;
					receive_stats.sync_trials = 0;
					skip_h_count = 0;
					coarse_freq_offset = 0.0;
					subpeak_recover_phase = 0;
					subpeak_orig_delay = -1;
					subpeak_recover_in_flight = false;
					consecutive_skip_var = 0;
					goto skip_h_retry_point;
				}
			}
		}

		// Diagnostic: decode failure analysis (verbose only — energy scan is expensive)
		if(receive_stats.message_decoded != YES && g_verbose)
		{
			// Plan-B Step 6a (site 9, FAIL-DIAG energy scan): diagnostic-only
			// per-symbol mean energies — rate-invariant. pream_start/data_start
			// are symbol-index multiples (pream_symb_loc * sym), so on the
			// decimated buffer they map cleanly to (index * Nofdm).
			const char* fail_type = (skip_h_count > 0) ? "SKIP-H" : "LDPC";
			int sym_dec_fd = data_container.Nofdm;
			int buf_dec_fd = data_container.Nofdm * data_container.buffer_Nsymb;
			double pream_energy = 0.0, data_energy = 0.0;
			int pream_start = pream_symb_loc * sym_dec_fd;
			int data_start = (pream_symb_loc + data_container.preamble_nSymb) * sym_dec_fd;
			for(int i = 0; i < sym_dec_fd && (pream_start + i) < buf_dec_fd; i++) {
				double re = data_container.baseband_data_decimated[pream_start + i].real();
				double im = data_container.baseband_data_decimated[pream_start + i].imag();
				pream_energy += re*re + im*im;
			}
			pream_energy /= sym_dec_fd;
			for(int i = 0; i < 4*sym_dec_fd && (data_start + i) < buf_dec_fd; i++) {
				double re = data_container.baseband_data_decimated[data_start + i].real();
				double im = data_container.baseband_data_decimated[data_start + i].imag();
				data_energy += re*re + im*im;
			}
			data_energy /= (4*sym_dec_fd);
			printf("[FAIL-DIAG] %s: metric=%.3f pream_sym=%d skip_h=%d/%d delay=%d mean_H=%.4f pE=%.2e dE=%.2e\n",
				fail_type, receive_stats.coarse_metric, pream_symb_loc,
				skip_h_count, time_sync_trials_max + 1,
				receive_stats.delay, mean_H, pream_energy, data_energy);
			fflush(stdout);
		}

		// STALE-CFO SCOPED RESET (long-run-degradation.md §2.2). We are inside the
		// energy_ok block: a real OFDM preamble was DETECTED and the trial loop ran
		// (silence / sub-threshold / weak-peak / no-preamble all returned earlier or
		// skipped this block, so they are correctly EXCLUDED). If the OFDM decode
		// still failed here, every fresh fine-sync trial failed and the :2510
		// last-trial fallback mixed the NEXT acquisition at the stale
		// freq_offset_of_last_decoded_message. That stale value is refreshed ONLY on
		// success (:3179), so a run of these failures is exactly the poison-latch:
		// the demod keeps mixing at a dead CFO and can never relock. Count the run;
		// once it crosses STALE_CFO_RESET_FAILS, SCRUB the stale CFO + the companion
		// coarse offset so the next acquisition re-measures from scratch.
		//
		// SCOPING (preserves the deliberate cfg=6 / LEVER-P stickiness):
		//  - A SINGLE marginal/dropped frame does NOT trip it (threshold > 1), so the
		//    last-trial reuse (:2510) and MINI-preamble reuse (:2536) still recover an
		//    isolated loss by reusing the recent-good CFO.
		//  - On a healthy link a decode succeeds and resets the counter to 0 (:3179),
		//    so the scrub NEVER fires -> the session-long last_coarse_freq_offset
		//    stickiness (arq_common.cc:3850-3854, the ~26% cfg=6 gain) is intact.
		//  - It fires ONLY in the sustained-failure regime where the stale CFO is
		//    provably poison, not a useful anchor. This is the cure for the long-run
		//    WB-OFDM fall-off, scoped to a FAILED-sync run rather than a blanket
		//    per-receive wipe.
		if(M != MOD_MFSK && receive_stats.message_decoded != YES)
		{
			const int STALE_CFO_RESET_FAILS = 3;
			consecutive_ofdm_decode_fails++;
			if(consecutive_ofdm_decode_fails >= STALE_CFO_RESET_FAILS &&
			   receive_stats.freq_offset_of_last_decoded_message != 0)
			{
				printf("[STALE-CFO] %d consecutive OFDM decode fails — scrubbing stale CFO (was %.4f Hz, coarse %.4f Hz) for fresh re-acquisition\n",
					consecutive_ofdm_decode_fails,
					receive_stats.freq_offset_of_last_decoded_message,
					last_coarse_freq_offset);
				fflush(stdout);
				receive_stats.freq_offset_of_last_decoded_message = 0;
				last_coarse_freq_offset = 0.0;
				consecutive_ofdm_decode_fails = 0;  // armed again; one scrub per run
			}
		}

		} // end if(energy_ok)

	}

	if(ldpc.print_nIteration==YES)
	{
		std::cout<<"decoded in "<< receive_stats.iterations_done<<" iterations."<<std::endl;
	}

	// Timing breakdown (printed periodically to avoid log flood)
	{
		auto timing_total_end = std::chrono::steady_clock::now();
		double timing_total_ms = std::chrono::duration<double, std::milli>(timing_total_end - timing_total_start).count();
		double frame_samples = (double)(data_container.Nofdm * (data_container.Nsymb + data_container.preamble_nSymb) * data_container.interpolation_rate);
		double frame_ms = (frame_samples / 48000.0) * 1000.0;
		static int timing_print_counter = 0;
		if(timing_print_counter++ % 20 == 0)
		{
			printf("[TIMING] total=%.1fms (%.0f%% of %.0fms frame) | pb_tsync=%.1f pb_data=%.1f ldpc=%.1f other=%.1f | buf_symb=%d frame_symb=%d iter=%d\n",
				timing_total_ms, frame_ms > 0 ? 100.0*timing_total_ms/frame_ms : 0, frame_ms,
				timing_pb_tsync_ms, timing_pb_data_ms, timing_ldpc_ms,
				timing_total_ms - timing_pb_tsync_ms - timing_pb_data_ms - timing_ldpc_ms,
				(int)data_container.buffer_Nsymb,
				data_container.preamble_nSymb + data_container.Nsymb,
				receive_stats.iterations_done);
			fflush(stdout);
		}
	}

#ifdef MERCURY_GUI_ENABLED
	// Update GUI with receive statistics
	gui_update_receive_stats(receive_stats.SNR, receive_stats.signal_stregth_dbm, receive_stats.freq_offset);
#endif

	return receive_stats;
}

double cl_telecom_system::measure_signal_only(double *data)
{
	// Lightweight signal measurement - only passband to baseband + measure strength
	// No preamble detection or decoding.
	//
	// Plan-B Step 6c (§7-inventory correction): this function — called from
	// cl_arq_controller::process_main() when link_status == IDLE/DROPPED — is
	// the FIR_rx_time_sync hot path while the modem is TRULY idle (the :928
	// call in receive_byte is hot only while LISTENING/receiving). The §7
	// inventory and all of Plan B scoped everything to receive_byte and missed
	// this — yet an RPi1 call-graph perf profile showed THIS is the ~92%
	// cl_FIR::apply cost during an idle-after-connect session. The fix is the
	// SAME rate-invariant conversion already applied to measure_signal_stregth
	// in receive_byte (Step 6a): the FIR runs at the decimated rate (M× cheaper)
	// and the mean-power measurement reads the decimated buffer. Mean power per
	// sample is rate-invariant (the anti-alias FIR passes the signal band).
	// Writing baseband_data_decimated instead of baseband_data_interpolated
	// also fully decouples this idle path from receive_byte's scratch buffer
	// (eliminates the Bug #28 same-buffer concern entirely).
	int ms_M = data_container.interpolation_rate;
	int ms_full_size = data_container.Nofdm * data_container.buffer_Nsymb * frequency_interpolation_rate;
	ofdm.passband_to_baseband_decimated((double*)data,
		ms_full_size,
		data_container.baseband_data_decimated,
		sampling_frequency, carrier_frequency, carrier_amplitude,
		ms_M, &ofdm.FIR_rx_time_sync);

	double signal_dbm = ofdm.measure_signal_stregth(
		data_container.baseband_data_decimated,
		data_container.Nofdm * data_container.buffer_Nsymb);

	receive_stats.signal_stregth_dbm = signal_dbm;

#ifdef MERCURY_GUI_ENABLED
	gui_update_receive_stats(receive_stats.SNR, signal_dbm, receive_stats.freq_offset);
#endif

	return signal_dbm;
}

void cl_telecom_system::calculate_parameters()
{
	double nData_eff;
	double log2M_eff;

	if(M == MOD_MFSK)
	{
		nData_eff = ofdm.Nsymb;
		log2M_eff = mfsk.bits_per_symbol();
	}
	else
	{
		nData_eff = ofdm.pilot_configurator.nData;
		log2M_eff = log2(M);
	}

	LDPC_real_CR=(nData_eff*log2M_eff-(double)ldpc.P -(double)outer_code_reserved_bits)/(nData_eff*log2M_eff);
	Tu= ofdm.Nc/bandwidth;
	Ts= Tu*(1.0+ofdm.gi);
	Tf= Ts*(ofdm.Nsymb+ofdm.preamble_configurator.Nsymb);
	rb= nData_eff * log2M_eff /Tf;
	rbc= rb*LDPC_real_CR;
	if(M == MOD_MFSK)
		Shannon_limit= 0;
	else
		Shannon_limit= 10.0*log10((pow(2,(rb*ldpc.rate)/bandwidth)-1)*log2M_eff*bandwidth/rb);
	sampling_frequency=frequency_interpolation_rate*(bandwidth/ofdm.Nc)*ofdm.Nfft;
}

void cl_telecom_system::set_mfsk_ctrl_mode(bool enable)
{
	mfsk_ctrl_mode = enable && (M == MOD_MFSK) && (ctrl_nBits > 0) && (ctrl_nBits < data_container.nBits);
}

int cl_telecom_system::get_active_nsymb() const
{
	return (mfsk_ctrl_mode && ctrl_nsymb > 0) ? ctrl_nsymb : data_container.Nsymb;
}

int cl_telecom_system::get_active_nbits() const
{
	return (mfsk_ctrl_mode && ctrl_nBits > 0) ? ctrl_nBits : data_container.nBits;
}

// TX: Generate ACK pattern as passband audio (no LDPC, no interleaver — pure known tones)
// Returns number of passband samples written to out[]
int cl_telecom_system::generate_ack_pattern_passband(double* out)
{

	if(ack_pattern_passband_samples <= 0) return 0;

	int nsymb = ack_mfsk.ack_pattern_nsymb;
	float power_normalization = sqrt((double)(ofdm.Nfft * frequency_interpolation_rate));

	// Generate subcarrier-domain ACK pattern (nsymb * Nc complex values)
	// Reuse ofdm_framed_data buffer (allocated for Nsymb * Nc, nsymb=16 fits easily)
	// Always use dedicated ack_mfsk (M=16, nStreams=1) — config-independent
	ack_mfsk.generate_ack_pattern(data_container.ofdm_framed_data);


	// IFFT each symbol to time domain
	for(int i = 0; i < nsymb; i++)
	{
		ofdm.symbol_mod(&data_container.ofdm_framed_data[i * data_container.Nc],
			&data_container.ofdm_symbol_modulated_data[i * data_container.Nofdm]);
	}

	// TX gain from calibration table
	double ack_boost = get_tx_gain(TX_SIG_ACK);
	for(int j = 0; j < data_container.Nofdm * nsymb; j++)
	{
		data_container.ofdm_symbol_modulated_data[j] /= power_normalization;
		data_container.ofdm_symbol_modulated_data[j] *= sqrt(output_power_Watt) * ack_boost;
	}

	// Baseband to passband
	double tx_carrier = carrier_frequency;
	ofdm.baseband_to_passband(data_container.ofdm_symbol_modulated_data,
		data_container.Nofdm * nsymb, out,
		sampling_frequency, tx_carrier, carrier_amplitude, frequency_interpolation_rate);

	// Peak clipping
	ofdm.peak_clip(out, ack_pattern_passband_samples, ofdm.data_papr_cut);


	return ack_pattern_passband_samples;
}

// RX: Detect ACK pattern in passband audio buffer
// Returns detection metric (0.0 = noise, up to ack_pattern_nsymb = perfect)
double cl_telecom_system::detect_ack_pattern_from_passband(double* data, int size, int* out_matched, uint32_t* out_match_mask)
{
	if(ack_pattern_passband_samples <= 0) return 0.0;

	// Fused mix + polyphase FIR + decimate: only the kept samples are computed,
	// dropping the FIR portion ~M× vs the prior mix → FIR-at-high-rate → pick-every-Mth
	// chain. Detector then runs on the already-decimated stream (interp_rate=1).
	int M = data_container.interpolation_rate;
	double effective_carrier = carrier_frequency + last_coarse_freq_offset;
	ofdm.passband_to_baseband_decimated(data, size,
		data_container.baseband_data_interpolated,
		sampling_frequency, effective_carrier, carrier_amplitude,
		M, &ofdm.FIR_rx_data);

	// Run matched-filter ACK detector — always use dedicated ack_mfsk (config-independent)
	double metric = ofdm.detect_ack_pattern(
		data_container.baseband_data_interpolated, size / M,
		1,
		ack_mfsk.ack_pattern_nsymb,
		ack_mfsk.ack_tones, ack_mfsk.ack_pattern_len,
		ack_mfsk.tone_hop_step, ack_mfsk.M,
		ack_mfsk.nStreams, ack_mfsk.stream_offsets,
		out_matched, 0, nullptr, nullptr, 0, out_match_mask);

	// Cache correlator metric (normalized to dB) as the §3.1 SNR proxy for the
	// 2D channel-state lookup (fact-doc channel-state-2d-lookup.md). detect_ack_pattern
	// returns Σ(e_target/e_total) per matched symbol ∈ [0, ack_pattern_nsymb]; the
	// normalized fraction ∈ [0,1] is what scales monotonically with SNR.
	if(ack_mfsk.ack_pattern_nsymb > 0)
	{
		double frac = metric / (double)ack_mfsk.ack_pattern_nsymb;
		if(frac < 1e-9) frac = 1e-9;  // floor to avoid log(0); -90 dB sentinel-ish
		last_correlator_metric_db = 10.0 * log10(frac);
	}

	return metric;
}

// TX: Generate ACK + SNR suffix pattern as passband audio
int cl_telecom_system::generate_ack_snr_pattern_passband(double* out, float snr)
{
	if(ack_snr_pattern_passband_samples <= 0) return 0;

	int nsymb = ack_mfsk.ack_snr_pattern_nsymb();
	float power_normalization = sqrt((double)(ofdm.Nfft * frequency_interpolation_rate));

	ack_mfsk.generate_ack_snr_pattern(data_container.ofdm_framed_data, snr);

	for(int i = 0; i < nsymb; i++)
	{
		ofdm.symbol_mod(&data_container.ofdm_framed_data[i * data_container.Nc],
			&data_container.ofdm_symbol_modulated_data[i * data_container.Nofdm]);
	}

	double ack_boost = get_tx_gain(TX_SIG_ACK);
	for(int j = 0; j < data_container.Nofdm * nsymb; j++)
	{
		data_container.ofdm_symbol_modulated_data[j] /= power_normalization;
		data_container.ofdm_symbol_modulated_data[j] *= sqrt(output_power_Watt) * ack_boost;
	}

	double tx_carrier = carrier_frequency;
	ofdm.baseband_to_passband(data_container.ofdm_symbol_modulated_data,
		data_container.Nofdm * nsymb, out,
		sampling_frequency, tx_carrier, carrier_amplitude, frequency_interpolation_rate);

	ofdm.peak_clip(out, ack_snr_pattern_passband_samples, ofdm.data_papr_cut);

	return ack_snr_pattern_passband_samples;
}

// TX: Generate ACK base + 13-symbol ACK+SACK suffix as passband audio.
// Suffix carries [batch_seq_id:8 | bitmap:32 | crc12:12] (52 bits,
// 4 bits/symbol on WB M=16). Caller computes the crc12 over [bsi || bitmap]
// using cl_arq_controller::CRC12_calc and passes it through. Returns number
// of passband samples written, or 0 if unsupported (NB / M<16). See
// mercury/fact-documents/mfsk-robust-ack.md §3.2.
int cl_telecom_system::generate_ack_sack_pattern_passband(double* out,
	uint8_t batch_seq_id, uint32_t bitmap, uint16_t crc12)
{
	if(ack_sack_pattern_passband_samples <= 0) return 0;
	if(ack_mfsk.ack_sack_suffix_len() <= 0) return 0;  // NB unsupported

	int nsymb = ack_mfsk.ack_sack_pattern_nsymb();
	float power_normalization = sqrt((double)(ofdm.Nfft * frequency_interpolation_rate));

	// Generate subcarrier-domain ACK+SACK pattern (nsymb * Nc complex values).
	// ofdm_framed_data is sized for max(Nsymb, 48) symbols × Nc — 29 fits easily.
	ack_mfsk.generate_ack_sack_pattern(data_container.ofdm_framed_data,
		batch_seq_id, bitmap, crc12);

	// IFFT each symbol to time domain
	for(int i = 0; i < nsymb; i++)
	{
		ofdm.symbol_mod(&data_container.ofdm_framed_data[i * data_container.Nc],
			&data_container.ofdm_symbol_modulated_data[i * data_container.Nofdm]);
	}

	// TX gain from calibration table (reuse ACK channel — same MFSK pattern family)
	double ack_boost = get_tx_gain(TX_SIG_ACK);
	for(int j = 0; j < data_container.Nofdm * nsymb; j++)
	{
		data_container.ofdm_symbol_modulated_data[j] /= power_normalization;
		data_container.ofdm_symbol_modulated_data[j] *= sqrt(output_power_Watt) * ack_boost;
	}

	// Baseband to passband
	double tx_carrier = carrier_frequency;
	ofdm.baseband_to_passband(data_container.ofdm_symbol_modulated_data,
		data_container.Nofdm * nsymb, out,
		sampling_frequency, tx_carrier, carrier_amplitude, frequency_interpolation_rate);

	// Peak clipping
	ofdm.peak_clip(out, ack_sack_pattern_passband_samples, ofdm.data_papr_cut);

	return ack_sack_pattern_passband_samples;
}

// RX: Detect ACK pattern and decode SNR suffix tones.
// Returns decoded SNR (dB). Sets *out_snr_valid = true if suffix decoded reliably.
float cl_telecom_system::detect_ack_snr_from_passband(double* data, int size,
	int* out_matched, bool* out_snr_valid)
{
	*out_snr_valid = false;
	if(ack_pattern_passband_samples <= 0) return -99.0f;

	// Polyphase decimated path: mix + FIR + decimate fused.
	int M = data_container.interpolation_rate;
	int dec_size = size / M;
	double effective_carrier = carrier_frequency + last_coarse_freq_offset;
	ofdm.passband_to_baseband_decimated(data, size,
		data_container.baseband_data_interpolated,
		sampling_frequency, effective_carrier, carrier_amplitude,
		M, &ofdm.FIR_rx_data);

	// Detect ACK pattern and get the detected position.
	// Reserve SNR_SUFFIX_LEN symbols after the ACK so suffix always fits.
	int best_offset = -1;
	double metric = ofdm.detect_ack_pattern(
		data_container.baseband_data_interpolated, dec_size,
		1,
		ack_mfsk.ack_pattern_nsymb,
		ack_mfsk.ack_tones, ack_mfsk.ack_pattern_len,
		ack_mfsk.tone_hop_step, ack_mfsk.M,
		ack_mfsk.nStreams, ack_mfsk.stream_offsets,
		out_matched, 0, nullptr, &best_offset,
		cl_mfsk::SNR_SUFFIX_LEN);

	if(*out_matched < ack_mfsk.ack_match_threshold ||
	   metric < cl_mfsk::CTRL_DETECT_METRIC_MIN || best_offset < 0)
		return -99.0f;

	// Control-frame mini-Moose v2 (data-preamble-port-research.md §24,
	// data-flow-freq_offset_measured.md §12). Refine residual CFO using the
	// 16-symbol ACK base pattern at the detected position, then re-mix
	// from passband with the SIGN-CORRECTED apply formula
	// `effective_carrier - residual` (§23.11.1 sign verdict). v1
	// (feat/mini-moose-ctrl, dropped) used `+` and regressed -40% bytes
	// on hardware A/B at WGN ≤ -8 because the WB MFSK estimator returns
	// the OPPOSITE sign of the actual baseband residual under the real-
	// passband injection model. v2 cancels (not doubles) the residual.
	//
	// Fail-safes:
	//   - estimator's 0.05 confidence gate returns 0 on pure-noise input.
	//   - freq_offset_ignore_limit (≈ 3 Hz) skips re-mix for sub-threshold
	//     residuals → buffer used as-is, identical to pre-v2 behavior.
	//   - if the second detect_ack_pattern doesn't improve (matched drops
	//     or shifts off the original best_offset's symbol grid), the
	//     downstream decode_suffix_tones still runs at the updated
	//     best_offset because the corrected baseband always represents
	//     the same audio window — just with a different mix LO.
	double ctrl_residual = ofdm.carrier_frequency_sync_wb_ctrl(
		data_container.baseband_data_interpolated,
		bandwidth / (double)data_container.Nc,
		ack_mfsk.ack_pattern_nsymb,
		best_offset,
		ack_mfsk.ack_tones, ack_mfsk.ack_pattern_len,
		ack_mfsk.tone_hop_step, ack_mfsk.M,
		ack_mfsk.nStreams, ack_mfsk.stream_offsets);

	if (fabs(ctrl_residual) > ofdm.freq_offset_ignore_limit)
	{
		// Re-mix at the SIGN-CORRECTED LO (§24.2.b). Identical buffer math
		// to the initial mix above — same M, same size, same FIR. Only the
		// mix frequency argument changes.
		ofdm.passband_to_baseband_decimated(data, size,
			data_container.baseband_data_interpolated,
			sampling_frequency,
			effective_carrier - ctrl_residual,
			carrier_amplitude,
			M, &ofdm.FIR_rx_data);

		// Re-run detection on the corrected baseband. The matched count
		// typically increases when CFO is corrected; best_offset may shift
		// by 0-1 symbols. If the new detection drops below threshold for
		// any reason, fall back to the corrected baseband at the original
		// best_offset (the corrected mix is still better-aligned than the
		// uncorrected one for the suffix decode).
		int rematched = 0;
		int rebest_offset = -1;
		double remetric = ofdm.detect_ack_pattern(
			data_container.baseband_data_interpolated, dec_size,
			1,
			ack_mfsk.ack_pattern_nsymb,
			ack_mfsk.ack_tones, ack_mfsk.ack_pattern_len,
			ack_mfsk.tone_hop_step, ack_mfsk.M,
			ack_mfsk.nStreams, ack_mfsk.stream_offsets,
			&rematched, 0, nullptr, &rebest_offset,
			cl_mfsk::SNR_SUFFIX_LEN);
		if (rematched >= ack_mfsk.ack_match_threshold &&
		    remetric >= cl_mfsk::CTRL_DETECT_METRIC_MIN && rebest_offset >= 0)
		{
			*out_matched = rematched;
			best_offset = rebest_offset;
		}
		// else: keep the original best_offset; corrected baseband still
		// used for the downstream suffix decode.
	}

	// Decode suffix tones at the detected position (decimated buffer, rate=1).
	// SACK suffix is longer than SNR (10 vs 8 on WB), so capture the max
	// available — decode_suffix_tones writes -1 for any symbol that runs
	// past the buffer end, which is safe to ignore for the shorter SNR path.
	int sack_suffix_len = ack_mfsk.ack_sack_suffix_len();
	int capture_len = cl_mfsk::SNR_SUFFIX_LEN;
	if (sack_suffix_len > capture_len) capture_len = sack_suffix_len;
	if (capture_len > cl_mfsk::MAX_ACK_SACK_SUFFIX)
		capture_len = cl_mfsk::MAX_ACK_SACK_SUFFIX;
	int suffix_tones[cl_mfsk::MAX_ACK_SACK_SUFFIX];
	ofdm.decode_suffix_tones(
		data_container.baseband_data_interpolated, dec_size,
		1,
		best_offset, ack_mfsk.ack_pattern_nsymb,
		capture_len,
		ack_mfsk.tone_hop_step, ack_mfsk.M,
		ack_mfsk.nStreams, ack_mfsk.stream_offsets,
		suffix_tones);

	// Hook for the ACK+SACK decode path (Step 3 of MFSK-suffix redesign):
	// snapshot the first sack_suffix_len de-hopped payload tones into the
	// mfsk object so decode_ack_sack_from_last_capture() can recover the
	// 40-bit (bsi, bitmap) payload. We only populate when WB SACK is in
	// scope (suffix_len > 0) AND every suffix symbol decoded (no -1's),
	// since unpack expects a clean 40-bit packing. This runs unconditionally
	// on any ACK match so the legacy SNR path keeps working — callers that
	// don't care about SACK simply leave last_ack_sack_capture_valid alone.
	if (sack_suffix_len > 0 && sack_suffix_len <= cl_mfsk::MAX_ACK_SACK_SUFFIX)
	{
		bool clean = true;
		for (int i = 0; i < sack_suffix_len; i++) {
			if (suffix_tones[i] < 0 || suffix_tones[i] >= ack_mfsk.M) {
				clean = false;
				break;
			}
			ack_mfsk.last_ack_sack_suffix_tones[i] = suffix_tones[i];
		}
		for (int i = sack_suffix_len; i < cl_mfsk::MAX_ACK_SACK_SUFFIX; i++)
			ack_mfsk.last_ack_sack_suffix_tones[i] = -1;
		ack_mfsk.last_ack_sack_capture_valid = clean;
	}

	// Majority vote: find most common tone among the 8 suffix symbols
	int vote_counts[64] = {};  // M <= 64
	int valid_count = 0;
	for(int i = 0; i < cl_mfsk::SNR_SUFFIX_LEN; i++)
	{
		if(suffix_tones[i] >= 0 && suffix_tones[i] < ack_mfsk.M)
		{
			vote_counts[suffix_tones[i]]++;
			valid_count++;
		}
	}

	if(valid_count < 3)
		return -99.0f;  // not enough symbols decoded

	int best_tone = 0;
	int best_votes = 0;
	int second_votes = 0;
	for(int t = 0; t < ack_mfsk.M; t++)
	{
		if(vote_counts[t] > best_votes)
		{
			second_votes = best_votes;
			best_votes = vote_counts[t];
			best_tone = t;
		}
		else if(vote_counts[t] > second_votes)
		{
			second_votes = vote_counts[t];
		}
	}

	// Require: at least 3/8 agree AND winner has 2+ more votes than runner-up
	if(best_votes >= 3 && (best_votes - second_votes) >= 2)
	{
		*out_snr_valid = true;
		return ack_mfsk.tone_to_snr(best_tone);
	}

	return -99.0f;
}

// RX: Detect ACK pattern and decode the 52-bit ACK+SACK suffix in one call.
// Runs the same detector pipeline as detect_ack_snr_from_passband — which
// writes the de-hopped suffix tones into ack_mfsk.last_ack_sack_suffix_tones[]
// — then invokes ack_mfsk.decode_ack_sack_from_last_capture() to recover
// (batch_seq_id, bitmap, crc12). The caller is responsible for verifying
// crc12 against a freshly-computed CRC12 over [bsi || bitmap]; on CRC
// mismatch the caller should treat the result as "no ACK arrived" so the
// existing timeout/retransmit logic handles it. Returns false on unsupported
// M (NB), no detection, or invalid capture.
bool cl_telecom_system::decode_ack_sack_from_passband(double* data, int size,
	uint8_t* out_bsi, uint32_t* out_bitmap, uint16_t* out_crc12, int* out_matched)
{
	if (ack_mfsk.ack_sack_suffix_len() <= 0) return false;  // NB / unsupported
	if (out_bsi == nullptr || out_bitmap == nullptr || out_crc12 == nullptr)
		return false;

	// Reuse the SNR detector pipeline — it already does base-pattern detection
	// + suffix capture into ack_mfsk.last_ack_sack_suffix_tones[].
	int matched = 0;
	bool snr_valid_dummy = false;
	(void)detect_ack_snr_from_passband(data, size, &matched, &snr_valid_dummy);
	if (out_matched) *out_matched = matched;

	if (!ack_mfsk.last_ack_sack_capture_valid) return false;
	bool ok = ack_mfsk.decode_ack_sack_from_last_capture(out_bsi, out_bitmap, out_crc12);
	// Consume the capture: caller gets a one-shot view of this match.
	ack_mfsk.last_ack_sack_capture_valid = false;
	return ok;
}

// §19 (INCREMENT 1): enable/disable the Tier-2 GF(16) RA FEC on the CONNECT
// ctrl-suffix. See telecom_system.h. Must be called AFTER load_configuration.
int cl_telecom_system::set_suffix_fec(bool on, int repfact)
{
	if (on) {
		gf16ra::configure(repfact);
		gf16ra::init();
		ack_mfsk.suffix_fec_coded = true;
		suffix_fec_mode = 3;          // GF(16) RA production path
	} else {
		ack_mfsk.suffix_fec_coded = false;
		suffix_fec_mode = 0;
	}
	// Re-derive the CONNECT-suffix passband sample count for the (now possibly
	// coded) ctrl_suffix_len(). load_configuration computed it at the uncoded
	// length; the coded length differs (52 vs 13) so every TX consumer that
	// reads this member must see the updated value (§19.4 C4). §20: the base now
	// occupies connect_base_total_nsymb() (R×16 when combining) — use the accessor
	// so both FEC and combining flow through the same member.
	ctrl_suffix_pattern_passband_samples =
		(ack_mfsk.connect_base_total_nsymb() + ack_mfsk.ctrl_suffix_len())
		* data_container.Nofdm * frequency_interpolation_rate;
	return ack_mfsk.ctrl_suffix_len();
}

// §20 (INCREMENT 2): set the CONNECT base-pattern combining factor R. See
// telecom_system.h. Must be called AFTER load_configuration. R=1 = off
// (byte-identical). Re-derives the passband sample count so every TX consumer
// sees the R×16 base length (the I4 invariant, §20.3 C3/C4).
int cl_telecom_system::set_connect_preamble_reps(int reps)
{
	if (reps < 1) reps = 1;
	if (reps > cl_mfsk::MAX_CONNECT_PREAMBLE_REPS)
		reps = cl_mfsk::MAX_CONNECT_PREAMBLE_REPS;
	ack_mfsk.connect_preamble_reps = reps;
	ctrl_suffix_pattern_passband_samples =
		(ack_mfsk.connect_base_total_nsymb() + ack_mfsk.ctrl_suffix_len())
		* data_container.Nofdm * frequency_interpolation_rate;
	return ack_mfsk.connect_base_total_nsymb();
}

// =============================================================================
// Phase B Wave 1: MFSK CONNECT base + ctrl-suffix
// =============================================================================
//
// TX: emit CONNECT base pattern + ctrl-suffix as passband audio. Wraps
// ack_mfsk.generate_ctrl_suffix_pattern + IFFT + baseband-to-passband
// (same shape as generate_ack_sack_pattern_passband but uses the CONNECT
// base pattern's tones for the first 16 symbols). Returns samples
// written or 0 if unsupported (NB / M<16).
int cl_telecom_system::generate_ctrl_suffix_pattern_passband(double* out,
	mfsk_ctrl_frame_type type, uint64_t payload38, uint16_t crc12)
{
	if(ctrl_suffix_pattern_passband_samples <= 0) return 0;
	if(ack_mfsk.ack_sack_suffix_len() <= 0) return 0;       // NB unsupported
	if(ack_mfsk.connect_pattern_nsymb <= 0) return 0;

	// §19: coded suffix length (52 with FEC, 13 uncoded). §20: base occupies
	// connect_base_total_nsymb() (R×16 when combining). The member
	// ctrl_suffix_pattern_passband_samples is computed from the same base+suffix
	// total (set_suffix_fec / set_connect_preamble_reps re-derive it), so the
	// returned sample count stays consistent.
	int nsymb = ack_mfsk.connect_base_total_nsymb() + ack_mfsk.ctrl_suffix_len();
	float power_normalization = sqrt((double)(ofdm.Nfft * frequency_interpolation_rate));

	ack_mfsk.generate_ctrl_suffix_pattern(data_container.ofdm_framed_data,
		type, payload38, crc12);

	for(int i = 0; i < nsymb; i++)
	{
		ofdm.symbol_mod(&data_container.ofdm_framed_data[i * data_container.Nc],
			&data_container.ofdm_symbol_modulated_data[i * data_container.Nofdm]);
	}

	// Reuse the ACK gain channel — same MFSK pattern family on the wire.
	double ack_boost = get_tx_gain(TX_SIG_ACK);
	for(int j = 0; j < data_container.Nofdm * nsymb; j++)
	{
		data_container.ofdm_symbol_modulated_data[j] /= power_normalization;
		data_container.ofdm_symbol_modulated_data[j] *= sqrt(output_power_Watt) * ack_boost;
	}

	double tx_carrier = carrier_frequency;
	ofdm.baseband_to_passband(data_container.ofdm_symbol_modulated_data,
		data_container.Nofdm * nsymb, out,
		sampling_frequency, tx_carrier, carrier_amplitude, frequency_interpolation_rate);

	ofdm.peak_clip(out, ctrl_suffix_pattern_passband_samples, ofdm.data_papr_cut);

	return ctrl_suffix_pattern_passband_samples;
}

// RX: detect CONNECT base + decode the 52-bit ctrl-suffix.
// Reuses ofdm.detect_ack_pattern parameterized on connect_tones, then
// ofdm.decode_suffix_tones, then ack_mfsk.unpack_ctrl_suffix. Returns
// true on a clean decode (out_type / out_payload38 / out_crc12 reflect
// the transmitted values). Caller verifies crc12 separately.
bool cl_telecom_system::decode_ctrl_suffix_from_passband(double* data, int size,
	mfsk_ctrl_frame_type* out_type, uint64_t* out_payload38,
	uint16_t* out_crc12, int* out_matched,
	ctrl_crc12_fn crc12_fn, void* crc12_ctx)
{
	if (out_type) *out_type = MFSK_CTRL_ACK_SACK;
	if (out_payload38) *out_payload38 = 0;
	if (out_crc12) *out_crc12 = 0;
	if (out_matched) *out_matched = 0;
	if (ack_mfsk.ack_sack_suffix_len() <= 0) return false;       // NB
	if (ack_mfsk.connect_pattern_nsymb <= 0) return false;
	if (!out_type || !out_payload38 || !out_crc12) return false;

	// Polyphase decimated path: mix + FIR + decimate fused (identical to
	// detect_ack_snr_from_passband's preprocessing).
	int M = data_container.interpolation_rate;
	int dec_size = size / M;
	double effective_carrier = carrier_frequency + last_coarse_freq_offset;
	ofdm.passband_to_baseband_decimated(data, size,
		data_container.baseband_data_interpolated,
		sampling_frequency, effective_carrier, carrier_amplitude,
		M, &ofdm.FIR_rx_data);

	int matched = 0;
	int best_offset = -1;
	// §20: base-pattern combining — ack_nsymb is ONE base block
	// (connect_pattern_nsymb); combine_reps=connect_preamble_reps tells the
	// detector to sum the per-symbol energy across the R aligned reps before the
	// matched-count. reps=1 → byte-identical single-block detection.
	double metric = ofdm.detect_ack_pattern(
		data_container.baseband_data_interpolated, dec_size,
		1,
		ack_mfsk.connect_pattern_nsymb,
		ack_mfsk.connect_tones, /*base_len=*/8,
		ack_mfsk.tone_hop_step, ack_mfsk.M,
		ack_mfsk.nStreams, ack_mfsk.stream_offsets,
		&matched, /*suffix_start=*/0, /*out_suffix_matched=*/nullptr,
		&best_offset, /*reserve_after=*/ack_mfsk.ctrl_suffix_len(),
		/*out_match_mask=*/nullptr, /*always_fine=*/false,
		/*combine_reps=*/ack_mfsk.connect_preamble_reps);

	if (out_matched) *out_matched = matched;

	if (matched < ack_mfsk.connect_match_threshold ||
	    metric < cl_mfsk::CTRL_DETECT_METRIC_MIN || best_offset < 0)
		return false;

	// Control-frame mini-Moose v2 (data-preamble-port-research.md §24.2.c,
	// data-flow-freq_offset_measured.md §12). Mirror of the
	// detect_ack_snr_from_passband wire-up but for the CONNECT base pattern.
	// Same SIGN-CORRECTED apply formula `effective_carrier - residual`
	// (§23.11.1). Same fail-safes.
	double ctrl_residual = ofdm.carrier_frequency_sync_wb_ctrl(
		data_container.baseband_data_interpolated,
		bandwidth / (double)data_container.Nc,
		ack_mfsk.connect_pattern_nsymb,
		best_offset,
		ack_mfsk.connect_tones, /*pattern_len=*/8,
		ack_mfsk.tone_hop_step, ack_mfsk.M,
		ack_mfsk.nStreams, ack_mfsk.stream_offsets);

	if (fabs(ctrl_residual) > ofdm.freq_offset_ignore_limit)
	{
		ofdm.passband_to_baseband_decimated(data, size,
			data_container.baseband_data_interpolated,
			sampling_frequency,
			effective_carrier - ctrl_residual,
			carrier_amplitude,
			M, &ofdm.FIR_rx_data);

		int rematched = 0;
		int rebest_offset = -1;
		double remetric = ofdm.detect_ack_pattern(
			data_container.baseband_data_interpolated, dec_size,
			1,
			ack_mfsk.connect_pattern_nsymb,
			ack_mfsk.connect_tones, /*base_len=*/8,
			ack_mfsk.tone_hop_step, ack_mfsk.M,
			ack_mfsk.nStreams, ack_mfsk.stream_offsets,
			&rematched, /*suffix_start=*/0, /*out_suffix_matched=*/nullptr,
			&rebest_offset, /*reserve_after=*/ack_mfsk.ctrl_suffix_len(),
			/*out_match_mask=*/nullptr, /*always_fine=*/false,
			/*combine_reps=*/ack_mfsk.connect_preamble_reps);
		if (rematched >= ack_mfsk.connect_match_threshold &&
		    remetric >= cl_mfsk::CTRL_DETECT_METRIC_MIN && rebest_offset >= 0)
		{
			matched = rematched;
			best_offset = rebest_offset;
			if (out_matched) *out_matched = matched;
		}
	}

	// §19 (INCREMENT 1) + §21.5 (TRY-BOTH): Tier-2 GF(16) RA FEC decode path. Base
	// detection + mini-Moose + the 1.2 metric gate above are UNCHANGED — they admit
	// the decode. When FEC is on (this RX is at the robust tier, so its capture
	// window is coded-sized) we FIRST try the cheap uncoded 13-tone decode (so a
	// legacy peer's uncoded CONNECT still decodes), then — on miss — extract the
	// FULL per-tone ENERGY matrix over the coded suffix (N=ctrl_suffix_len()
	// symbols) and run the soft Q-ary BP decoder, which carries its own
	// CRC12+2-bit-type accept gate (the FAR backstop, §16/§17.2). The hard-tone
	// capture (last_connect_suffix_tones[]) is NOT written on this path — it is the
	// uncoded-only path's snapshot and is unused by the CONNECT production caller.
	if (ack_mfsk.suffix_fec_coded)
	{
		if (!crc12_fn) return false;   // FEC needs the production CRC12 callback

		// §21.5 TRY-BOTH: run the UNCODED 13-tone decode FIRST (cost ~0 — a single
		// argmax read + bit unpack at the same matched offset). This lets an
		// upgraded robust-tier RX (whose capture window is coded-sized because its
		// OWN tier gate enabled FEC) still decode a LEGACY peer's uncoded 13-tone
		// CONNECT suffix. Verify CRC + a valid 2-bit type inline; on success return
		// the uncoded result. On miss (the usual case when the peer sent the 52-tone
		// GF(16) codeword — the first 13 of which are NOT the hard bit-pack), fall
		// through to the FEC decode below. The uncoded read is a strict sub-window
		// of the coded capture (§21.4 C6), so it is always in-bounds.
		{
			int ulen = ack_mfsk.ack_sack_suffix_len();   // 13 at M=16
			if (ulen > 0 && ulen <= cl_mfsk::MAX_ACK_SACK_SUFFIX) {
				int utones[cl_mfsk::MAX_ACK_SACK_SUFFIX];
				ofdm.decode_suffix_tones(
					data_container.baseband_data_interpolated, dec_size, 1,
					best_offset, ack_mfsk.connect_base_total_nsymb(), ulen,
					ack_mfsk.tone_hop_step, ack_mfsk.M,
					ack_mfsk.nStreams, ack_mfsk.stream_offsets, utones);
				bool uclean = true;
				for (int i = 0; i < ulen; i++)
					if (utones[i] < 0 || utones[i] >= ack_mfsk.M) { uclean = false; break; }
				if (uclean) {
					mfsk_ctrl_frame_type ut; uint64_t up38 = 0; uint16_t uc = 0;
					if (ack_mfsk.unpack_ctrl_suffix(utones, &ut, &up38, &uc)) {
						unsigned char typed[5];
						pack_ctrl_typed40_msb(typed, (uint8_t)ut, up38);
						uint16_t exp = crc12_fn(crc12_ctx, typed, 5) & 0x0FFF;
						if (exp == (uc & 0x0FFF)) {
							// Uncoded (legacy-compatible) decode validated.
							*out_type = ut; *out_payload38 = up38; *out_crc12 = uc;
							return true;
						}
					}
				}
			}
		}

		int N = ack_mfsk.ctrl_suffix_len();   // = gf16ra::codeword_len()
		if (N <= 0 || N > gf16ra::GF16RA_MAX_N) return false;
		std::vector<double> energies((size_t)N * ack_mfsk.M, 0.0);
		// §20: the suffix follows ALL R base reps. pattern_nsymb =
		// connect_base_total_nsymb() (R×16) is BOTH the symbol offset to the suffix
		// and the hop base — must match the TX abs_s in generate_ctrl_suffix_pattern.
		ofdm.decode_suffix_energies(
			data_container.baseband_data_interpolated, dec_size,
			1,
			best_offset, ack_mfsk.connect_base_total_nsymb(),
			N,
			ack_mfsk.tone_hop_step, ack_mfsk.M,
			ack_mfsk.nStreams, ack_mfsk.stream_offsets,
			energies.data());

		// soft_decode requires a 2-bit expected type. The production caller
		// (receive_mfsk_ctrl_suffix_phy_core) already routes by type AFTER this
		// returns; to preserve that flow we accept ANY of the 4 types here, then
		// hand the decoded type up. soft_decode itself only accepts when its
		// internal CRC over [type|payload38] matches the decoded CRC, so each
		// type attempt is CRC-backstopped (FAR per type ≈ 2^-12; ×4 types still
		// ≈ 2^-10, far below the count-gate FAR). First CRC-valid type wins.
		uint64_t p38 = 0;
		int iters = -1;
		for (int t = 0; t < 4; t++) {
			if (gf16ra::soft_decode(energies.data(),
				/*maxiter=*/50, /*esno_metric=*/4.0,
				(uint8_t)t, crc12_fn, crc12_ctx, &p38, &iters))
			{
				*out_type = (mfsk_ctrl_frame_type)t;
				*out_payload38 = p38;
				// Recompute the CRC over the decoded field so the caller's outer
				// CRC re-check (arq_common.cc) passes by construction — soft_decode
				// already proved decoded_crc == CRC12_calc([type|p38]).
				unsigned char typed[5];
				pack_ctrl_typed40_msb(typed, (uint8_t)t, p38);
				*out_crc12 = crc12_fn(crc12_ctx, typed, 5) & 0x0FFF;
				return true;
			}
		}
		return false;   // no CRC-valid type within the FEC decode
	}

	// ---- Uncoded (baseline) hard path: byte-identical to before §19. ----
	// Decode the 13-tone ctrl-suffix at the matched position.
	int suffix_len = ack_mfsk.ack_sack_suffix_len();
	if (suffix_len > cl_mfsk::MAX_ACK_SACK_SUFFIX)
		suffix_len = cl_mfsk::MAX_ACK_SACK_SUFFIX;
	int suffix_tones[cl_mfsk::MAX_ACK_SACK_SUFFIX];
	// §20: suffix offset = connect_base_total_nsymb() (R×16) so the hop base and
	// buffer position match the TX layout when combining (=16 when reps=1).
	ofdm.decode_suffix_tones(
		data_container.baseband_data_interpolated, dec_size,
		1,
		best_offset, ack_mfsk.connect_base_total_nsymb(),
		suffix_len,
		ack_mfsk.tone_hop_step, ack_mfsk.M,
		ack_mfsk.nStreams, ack_mfsk.stream_offsets,
		suffix_tones);

	// Snapshot into the CONNECT capture buffer (mirror of ACK+SACK hook).
	bool clean = true;
	for (int i = 0; i < suffix_len; i++) {
		if (suffix_tones[i] < 0 || suffix_tones[i] >= ack_mfsk.M) {
			clean = false;
			break;
		}
		ack_mfsk.last_connect_suffix_tones[i] = suffix_tones[i];
	}
	for (int i = suffix_len; i < cl_mfsk::MAX_ACK_SACK_SUFFIX; i++)
		ack_mfsk.last_connect_suffix_tones[i] = -1;
	ack_mfsk.last_connect_capture_valid = clean;

	if (!clean) return false;

	bool ok = ack_mfsk.decode_ctrl_suffix_from_last_capture(
		out_type, out_payload38, out_crc12);
	// Consume the capture: caller gets a one-shot view of this match.
	ack_mfsk.last_connect_capture_valid = false;
	return ok;
}

// =============================================================================
// Suffix FEC — CRC-aided SOFT list decode (connect-suffix-fec-research.md §3)
// =============================================================================
//
// RX: detect CONNECT base + SOFT-decode the 52-bit ctrl-suffix. Same detector
// pipeline as decode_ctrl_suffix_from_passband (passband→baseband, base-pattern
// detect, control mini-Moose v2), but the suffix is decoded with
// ofdm.decode_suffix_candidates (top-K per symbol) + soft_list_decode_ctrl_suffix
// (CRC-gated best-first search) instead of a single hard argmax per symbol.
// Returns true iff a type-matched, CRC-valid assignment is found. ZERO airtime
// (the suffix bytes on the wire are byte-identical to baseline). The CRC check
// uses the caller-supplied production CRC-12 (NEVER inline — v1 bug #1).
bool cl_telecom_system::decode_ctrl_suffix_from_passband_soft(double* data, int size,
	mfsk_ctrl_frame_type expected_type, ctrl_crc12_fn crc12_fn, void* crc12_ctx,
	uint64_t* out_payload38, int* out_matched, int* out_flips)
{
	if (out_payload38) *out_payload38 = 0;
	if (out_matched) *out_matched = 0;
	if (out_flips) *out_flips = -1;
	if (!out_payload38 || !crc12_fn) return false;
	if (ack_mfsk.ack_sack_suffix_len() <= 0) return false;       // NB
	if (ack_mfsk.connect_pattern_nsymb <= 0) return false;

	int M = data_container.interpolation_rate;
	int dec_size = size / M;
	double effective_carrier = carrier_frequency + last_coarse_freq_offset;
	ofdm.passband_to_baseband_decimated(data, size,
		data_container.baseband_data_interpolated,
		sampling_frequency, effective_carrier, carrier_amplitude,
		M, &ofdm.FIR_rx_data);

	int matched = 0;
	int best_offset = -1;
	double metric = ofdm.detect_ack_pattern(
		data_container.baseband_data_interpolated, dec_size, 1,
		ack_mfsk.connect_pattern_nsymb,
		ack_mfsk.connect_tones, /*base_len=*/8,
		ack_mfsk.tone_hop_step, ack_mfsk.M,
		ack_mfsk.nStreams, ack_mfsk.stream_offsets,
		&matched, /*suffix_start=*/0, /*out_suffix_matched=*/nullptr,
		&best_offset, /*reserve_after=*/ack_mfsk.ack_sack_suffix_len(),
		/*out_match_mask=*/nullptr);

	if (out_matched) *out_matched = matched;
	if (matched < ack_mfsk.connect_match_threshold || metric < 3.0 || best_offset < 0)
		return false;

	// Control-frame mini-Moose v2 (identical to the hard path).
	double ctrl_residual = ofdm.carrier_frequency_sync_wb_ctrl(
		data_container.baseband_data_interpolated,
		bandwidth / (double)data_container.Nc,
		ack_mfsk.connect_pattern_nsymb, best_offset,
		ack_mfsk.connect_tones, /*pattern_len=*/8,
		ack_mfsk.tone_hop_step, ack_mfsk.M,
		ack_mfsk.nStreams, ack_mfsk.stream_offsets);

	if (fabs(ctrl_residual) > ofdm.freq_offset_ignore_limit)
	{
		ofdm.passband_to_baseband_decimated(data, size,
			data_container.baseband_data_interpolated,
			sampling_frequency, effective_carrier - ctrl_residual,
			carrier_amplitude, M, &ofdm.FIR_rx_data);
		int rematched = 0, rebest_offset = -1;
		double remetric = ofdm.detect_ack_pattern(
			data_container.baseband_data_interpolated, dec_size, 1,
			ack_mfsk.connect_pattern_nsymb,
			ack_mfsk.connect_tones, /*base_len=*/8,
			ack_mfsk.tone_hop_step, ack_mfsk.M,
			ack_mfsk.nStreams, ack_mfsk.stream_offsets,
			&rematched, 0, nullptr, &rebest_offset,
			ack_mfsk.ack_sack_suffix_len(), nullptr);
		if (rematched >= ack_mfsk.connect_match_threshold && remetric >= 3.0 && rebest_offset >= 0)
		{
			matched = rematched; best_offset = rebest_offset;
			if (out_matched) *out_matched = matched;
		}
	}

	// SOFT suffix decode: top-K candidates per symbol, then CRC-gated search.
	int n = ack_mfsk.ack_sack_suffix_len();
	int K = suffix_fec_K; if (K < 1) K = 1; if (K > ack_mfsk.M) K = ack_mfsk.M;
	int bits_per_tone = 0; for (int m = ack_mfsk.M; m > 1; m >>= 1) bits_per_tone++;
	std::vector<int> cand((size_t)n * K);
	std::vector<double> cost((size_t)n * K);
	ofdm.decode_suffix_candidates(
		data_container.baseband_data_interpolated, dec_size, 1,
		best_offset, ack_mfsk.connect_pattern_nsymb, n,
		ack_mfsk.tone_hop_step, ack_mfsk.M,
		ack_mfsk.nStreams, ack_mfsk.stream_offsets,
		K, cand.data(), cost.data());

	return soft_list_decode_ctrl_suffix(cand.data(), cost.data(), n, K,
		bits_per_tone, (uint8_t)expected_type, suffix_fec_max_trials,
		suffix_fec_max_flips, crc12_fn, crc12_ctx, out_payload38, out_flips);
}

// RX: detect ACK base + SOFT-decode the 52-bit ACK+SACK suffix. Mirror of
// decode_ctrl_suffix_from_passband_soft but on the ACK base pattern + SNR-path
// preprocessing (reuses detect_ack_snr_from_passband, which already runs the
// base detect + mini-Moose and leaves the corrected baseband at best_offset).
// Because ACK_SACK uses type=0 the search pins expected_type=MFSK_CTRL_ACK_SACK.
bool cl_telecom_system::decode_ack_sack_from_passband_soft(double* data, int size,
	ctrl_crc12_fn crc12_fn, void* crc12_ctx,
	uint8_t* out_bsi, uint32_t* out_bitmap, int* out_matched, int* out_flips)
{
	if (out_bsi) *out_bsi = 0;
	if (out_bitmap) *out_bitmap = 0;
	if (out_matched) *out_matched = 0;
	if (out_flips) *out_flips = -1;
	if (!out_bsi || !out_bitmap || !crc12_fn) return false;
	if (ack_mfsk.ack_sack_suffix_len() <= 0) return false;       // NB

	int M = data_container.interpolation_rate;
	int dec_size = size / M;
	double effective_carrier = carrier_frequency + last_coarse_freq_offset;
	ofdm.passband_to_baseband_decimated(data, size,
		data_container.baseband_data_interpolated,
		sampling_frequency, effective_carrier, carrier_amplitude,
		M, &ofdm.FIR_rx_data);

	int matched = 0;
	int best_offset = -1;
	double metric = ofdm.detect_ack_pattern(
		data_container.baseband_data_interpolated, dec_size, 1,
		ack_mfsk.ack_pattern_nsymb,
		ack_mfsk.ack_tones, ack_mfsk.ack_pattern_len,
		ack_mfsk.tone_hop_step, ack_mfsk.M,
		ack_mfsk.nStreams, ack_mfsk.stream_offsets,
		&matched, /*suffix_start=*/0, /*out_suffix_matched=*/nullptr,
		&best_offset, /*reserve_after=*/ack_mfsk.ack_sack_suffix_len(),
		/*out_match_mask=*/nullptr);

	if (out_matched) *out_matched = matched;
	if (matched < ack_mfsk.ack_match_threshold || metric < 3.0 || best_offset < 0)
		return false;

	double ctrl_residual = ofdm.carrier_frequency_sync_wb_ctrl(
		data_container.baseband_data_interpolated,
		bandwidth / (double)data_container.Nc,
		ack_mfsk.ack_pattern_nsymb, best_offset,
		ack_mfsk.ack_tones, ack_mfsk.ack_pattern_len,
		ack_mfsk.tone_hop_step, ack_mfsk.M,
		ack_mfsk.nStreams, ack_mfsk.stream_offsets);

	if (fabs(ctrl_residual) > ofdm.freq_offset_ignore_limit)
	{
		ofdm.passband_to_baseband_decimated(data, size,
			data_container.baseband_data_interpolated,
			sampling_frequency, effective_carrier - ctrl_residual,
			carrier_amplitude, M, &ofdm.FIR_rx_data);
		int rematched = 0, rebest_offset = -1;
		double remetric = ofdm.detect_ack_pattern(
			data_container.baseband_data_interpolated, dec_size, 1,
			ack_mfsk.ack_pattern_nsymb,
			ack_mfsk.ack_tones, ack_mfsk.ack_pattern_len,
			ack_mfsk.tone_hop_step, ack_mfsk.M,
			ack_mfsk.nStreams, ack_mfsk.stream_offsets,
			&rematched, 0, nullptr, &rebest_offset,
			ack_mfsk.ack_sack_suffix_len(), nullptr);
		if (rematched >= ack_mfsk.ack_match_threshold && remetric >= 3.0 && rebest_offset >= 0)
		{
			matched = rematched; best_offset = rebest_offset;
			if (out_matched) *out_matched = matched;
		}
	}

	int n = ack_mfsk.ack_sack_suffix_len();
	int K = suffix_fec_K; if (K < 1) K = 1; if (K > ack_mfsk.M) K = ack_mfsk.M;
	int bits_per_tone = 0; for (int m = ack_mfsk.M; m > 1; m >>= 1) bits_per_tone++;
	std::vector<int> cand((size_t)n * K);
	std::vector<double> cost((size_t)n * K);
	ofdm.decode_suffix_candidates(
		data_container.baseband_data_interpolated, dec_size, 1,
		best_offset, ack_mfsk.ack_pattern_nsymb, n,
		ack_mfsk.tone_hop_step, ack_mfsk.M,
		ack_mfsk.nStreams, ack_mfsk.stream_offsets,
		K, cand.data(), cost.data());

	uint64_t p38 = 0;
	bool ok = soft_list_decode_ctrl_suffix(cand.data(), cost.data(), n, K,
		bits_per_tone, (uint8_t)MFSK_CTRL_ACK_SACK, suffix_fec_max_trials,
		suffix_fec_max_flips, crc12_fn, crc12_ctx, &p38, out_flips);
	if (!ok) return false;
	// payload38 = [bsi:8 | bitmap:30] (mfsk.cc:691 pack_ack_sack_payload).
	*out_bitmap = (uint32_t)(p38 & 0x3FFFFFFFu);
	*out_bsi    = (uint8_t)((p38 >> 30) & 0xFFu);
	return true;
}

// TX: Generate BREAK pattern as passband audio (identical to ACK but with break_tones)
int cl_telecom_system::generate_break_pattern_passband(double* out)
{
	if(ack_pattern_passband_samples <= 0) return 0;

	int nsymb = ack_mfsk.ack_pattern_nsymb;
	float power_normalization = sqrt((double)(ofdm.Nfft * frequency_interpolation_rate));

	ack_mfsk.generate_break_pattern(data_container.ofdm_framed_data);

	for(int i = 0; i < nsymb; i++)
	{
		ofdm.symbol_mod(&data_container.ofdm_framed_data[i * data_container.Nc],
			&data_container.ofdm_symbol_modulated_data[i * data_container.Nofdm]);
	}

	// TX gain from calibration table
	double brk_boost = get_tx_gain(TX_SIG_BREAK);
	for(int j = 0; j < data_container.Nofdm * nsymb; j++)
	{
		data_container.ofdm_symbol_modulated_data[j] /= power_normalization;
		data_container.ofdm_symbol_modulated_data[j] *= sqrt(output_power_Watt) * brk_boost;
	}

	double tx_carrier = carrier_frequency;
	ofdm.baseband_to_passband(data_container.ofdm_symbol_modulated_data,
		data_container.Nofdm * nsymb, out,
		sampling_frequency, tx_carrier, carrier_amplitude, frequency_interpolation_rate);

	ofdm.peak_clip(out, ack_pattern_passband_samples, ofdm.data_papr_cut);

	return ack_pattern_passband_samples;
}

// RX: Detect BREAK pattern in passband audio buffer (uses break_tones instead of ack_tones)
double cl_telecom_system::detect_break_pattern_from_passband(double* data, int size, int* out_matched)
{
	if(ack_pattern_passband_samples <= 0) return 0.0;

	int M = data_container.interpolation_rate;
	double effective_carrier = carrier_frequency + last_coarse_freq_offset;
	ofdm.passband_to_baseband_decimated(data, size,
		data_container.baseband_data_interpolated,
		sampling_frequency, effective_carrier, carrier_amplitude,
		M, &ofdm.FIR_rx_data);

	// always_fine=true: BREAK is a one-shot single-burst (no polling loop),
	// so the asynchronous arrival phase often lands mid-symbol. Without
	// fine sample-rate refinement, FFT windows straddle two transmitted
	// symbols and coarse-only match collapses well below the 12/16 BREAK
	// threshold — manifested as 0 BREAK detections across 10 transmissions
	// in gearshift_v9 smoke. ACK detector polls every ~2-3 ms with sliding
	// tail snapshots so its alignment lottery eventually wins, but BREAK
	// fires once per ~100 ms full RX-TIMING FAIL — one bad alignment per
	// burst is the typical case.
	double metric = ofdm.detect_ack_pattern(
		data_container.baseband_data_interpolated, size / M,
		1,
		ack_mfsk.ack_pattern_nsymb,
		ack_mfsk.break_tones, ack_mfsk.ack_pattern_len,
		ack_mfsk.tone_hop_step, ack_mfsk.M,
		ack_mfsk.nStreams, ack_mfsk.stream_offsets,
		out_matched,
		/*suffix_start=*/0, /*out_suffix_matched=*/nullptr,
		/*out_best_offset=*/nullptr, /*reserve_after=*/0,
		/*out_match_mask=*/nullptr, /*always_fine=*/true);

	return metric;
}

// TX: Generate HAIL pattern as passband audio (prefix + optional directed suffix)
int cl_telecom_system::generate_hail_pattern_passband(double* out)
{
	if(ack_pattern_passband_samples <= 0) return 0;

	int nsymb = ack_mfsk.hail_detect_nsymb;
	int hail_samples = nsymb * data_container.Nofdm * frequency_interpolation_rate;
	float power_normalization = sqrt((double)(ofdm.Nfft * frequency_interpolation_rate));

	ack_mfsk.generate_hail_pattern(data_container.ofdm_framed_data);

	for(int i = 0; i < nsymb; i++)
	{
		ofdm.symbol_mod(&data_container.ofdm_framed_data[i * data_container.Nc],
			&data_container.ofdm_symbol_modulated_data[i * data_container.Nofdm]);
	}

	double hail_boost = get_tx_gain(TX_SIG_ACK);  // same gain as ACK
	for(int j = 0; j < data_container.Nofdm * nsymb; j++)
	{
		data_container.ofdm_symbol_modulated_data[j] /= power_normalization;
		data_container.ofdm_symbol_modulated_data[j] *= sqrt(output_power_Watt) * hail_boost;
	}

	double tx_carrier = carrier_frequency;
	ofdm.baseband_to_passband(data_container.ofdm_symbol_modulated_data,
		data_container.Nofdm * nsymb, out,
		sampling_frequency, tx_carrier, carrier_amplitude, frequency_interpolation_rate);

	ofdm.peak_clip(out, hail_samples, ofdm.data_papr_cut);

	return hail_samples;
}

// RX: Detect HAIL pattern in passband audio buffer (prefix + optional directed suffix)
double cl_telecom_system::detect_hail_pattern_from_passband(double* data, int size, int* out_matched,
                                                            int suffix_start, int* out_suffix_matched)
{
	if(ack_pattern_passband_samples <= 0) return 0.0;

	int M = data_container.interpolation_rate;
	double effective_carrier = carrier_frequency + last_coarse_freq_offset;
	ofdm.passband_to_baseband_decimated(data, size,
		data_container.baseband_data_interpolated,
		sampling_frequency, effective_carrier, carrier_amplitude,
		M, &ofdm.FIR_rx_data);

	double metric = ofdm.detect_ack_pattern(
		data_container.baseband_data_interpolated, size / M,
		1,
		ack_mfsk.hail_detect_nsymb,
		ack_mfsk.hail_detect_tones, ack_mfsk.hail_detect_nsymb,
		ack_mfsk.tone_hop_step, ack_mfsk.M,
		ack_mfsk.nStreams, ack_mfsk.stream_offsets,
		out_matched, suffix_start, out_suffix_matched);

	// Cache correlator metric (normalized to dB) — see ACK detector above and
	// fact-doc channel-state-2d-lookup.md §3.1. HAIL uses hail_detect_nsymb as
	// the per-symbol score denominator.
	if(ack_mfsk.hail_detect_nsymb > 0)
	{
		double frac = metric / (double)ack_mfsk.hail_detect_nsymb;
		if(frac < 1e-9) frac = 1e-9;
		last_correlator_metric_db = 10.0 * log10(frac);
	}

	return metric;
}

// ACK pattern detection test: sweep SNR, measure detection metric and false alarm rate
void cl_telecom_system::ack_pattern_detection_test()
{
	if(ack_pattern_passband_samples <= 0)
	{
		printf("[ACK_TEST] ack_pattern not configured\n");
		return;
	}

	int nsymb = ack_mfsk.ack_pattern_nsymb;
	int passband_samples = ack_pattern_passband_samples;
	int delay_samples = 2 * data_container.Nofdm * frequency_interpolation_rate;
	int rx_buffer_size = passband_samples + 2 * delay_samples;

	double* tx_passband = new double[passband_samples];
	double* rx_buffer = new double[rx_buffer_size];

	// Generate ACK pattern
	generate_ack_pattern_passband(tx_passband);

	// Measure signal power for sigma calibration
	double P_sig = 0;
	for(int i = 0; i < passband_samples; i++)
		P_sig += tx_passband[i] * tx_passband[i];
	P_sig /= passband_samples;

	double f_nyquist = sampling_frequency / 2.0;
	int nTrials = 20;

	// With carrier image recovery (direct + mirror bin energy), max metric ≈ ack_pattern_nsymb
	double max_clean_metric = (double)nsymb;

	printf("ACK_DETECT_TEST;max_clean=%.1f;SNR;mean_metric;min_metric;max_metric\n", max_clean_metric);
	fflush(stdout);

	// Sweep SNR from -20 to +5 dB
	for(double snr_db = -20.0; snr_db <= 5.0; snr_db += 1.0)
	{
		float sigma = (float)sqrt(2.0 * P_sig * f_nyquist / (pow(10.0, snr_db / 10.0) * bandwidth));
		double metric_sum = 0;
		double metric_min = 1e30;
		double metric_max = -1e30;

		for(int trial = 0; trial < nTrials; trial++)
		{
			// Zero buffer first (apply_with_delay only writes delay+nItems samples)
			for(int i = 0; i < rx_buffer_size; i++) rx_buffer[i] = 0.0;

			// Place TX signal with delay, add AWGN
			awgn_channel.apply_with_delay(tx_passband, rx_buffer, sigma,
				passband_samples, delay_samples);

			// Detect
			double metric = detect_ack_pattern_from_passband(rx_buffer, rx_buffer_size);
			metric_sum += metric;
			if(metric < metric_min) metric_min = metric;
			if(metric > metric_max) metric_max = metric;
		}

		printf("%.0f;%.3f;%.3f;%.3f\n", snr_db, metric_sum / nTrials, metric_min, metric_max);
		fflush(stdout);
	}

	// False alarm test: noise only (no signal)
	printf("ACK_FALSE_ALARM_TEST;threshold=%.1f\n", ack_pattern_detection_threshold);
	fflush(stdout);
	{
		int noise_trials = 20;
		int false_alarms = 0;
		double noise_metric_max = 0;
		// Use sigma for -10 dB SNR level noise power
		float sigma = (float)sqrt(2.0 * P_sig * f_nyquist / (pow(10.0, -10.0 / 10.0) * bandwidth));

		for(int trial = 0; trial < noise_trials; trial++)
		{
			// Generate noise-only buffer
			for(int i = 0; i < rx_buffer_size; i++)
				rx_buffer[i] = (sigma / sqrtf(2.0f)) * awgn_channel.awgn_value_generator();

			double metric = detect_ack_pattern_from_passband(rx_buffer, rx_buffer_size);
			if(metric > noise_metric_max) noise_metric_max = metric;
			if(metric >= ack_pattern_detection_threshold) false_alarms++;
		}

		printf("FALSE_ALARM;%d/%d;max_noise_metric=%.3f\n", false_alarms, noise_trials, noise_metric_max);
		fflush(stdout);
	}

	delete[] tx_passband;
	delete[] rx_buffer;
}

void cl_telecom_system::init()
{
	if(ofdm.Nc==AUTO_SELLECT)
	{
		ofdm.Nc = narrowband_enabled ? 10 : 50;
	}

	// Recompute bandwidth-dependent parameters from actual Nc
	// physical_config.cc computed these with hardcoded Nc=50; recompute for actual Nc.
	// IMPORTANT: Do NOT modify default_configurations_telecom_system here — it holds the
	// original physical config values and must remain pristine across NB/WB transitions.
	// Only update member variables; load_configuration() + init() will recompute each time.
	{
		double bw = 48000.0 * ofdm.Nc / ofdm.Nfft / frequency_interpolation_rate;
		double bw_original = 48000.0 * 50.0 / ofdm.Nfft / frequency_interpolation_rate;
		// Keep carrier_frequency at the WB center (~1472 Hz) regardless of bandwidth.
		// NB signal spans cf ± bw/2 (e.g. 1238-1706 Hz), centered near 1500 Hz.
		// Old formula: cf = offset + bw/2 + 300, which put NB at ~534 Hz (bottom of passband).
		double cf = default_configurations_telecom_system.carrier_frequency;

		// FIR transition bandwidth scaling for narrowband (Option A — split FIR).
		//
		// GI preservation constraint: filter_nTaps <= Ngi*interp = 64 samples.
		// nTaps = 2*fs/transition_BW, so transition_BW >= 1500 Hz for 64 taps.
		// Any filter in the TX→RX chain that processes the passband/baseband signal
		// with more taps than the GI window will destroy the GI-copy property,
		// making Schmidl-Cox preamble detection fail.
		//
		// TIME-SYNC RX FIR: Keep transition at 3000 Hz (33 taps). GI-safe.
		// DATA RX FIR: Narrow transition (600 Hz, 161 taps). Runs AFTER time sync
		//   determines the delay, so GI structure is irrelevant.
		// TX FIRs: Keep transition at 1000 Hz (97 taps, same as WB). The WB TX
		//   filter at 97 taps slightly exceeds the 64-sample GI, but the outermost
		//   Hamming-windowed taps contribute negligible energy. 481 taps (from
		//   scaling to 200 Hz) would be catastrophic — 7.5x the GI window.
		double bw_ratio = bw / bw_original;

		// Set member variables directly for correct NB/WB FIR design.
		// load_configuration() already copied defaults → members with WB values;
		// we override them here before FIR design() (called below in init).
		bandwidth = bw;
		carrier_frequency = cf;
		ofdm.FIR_rx_time_sync.lpf_filter_cut_frequency = 0.9 * bw / 2;
		// Transition must end before the -2fc conjugate image to avoid
		// carrier-phase-dependent matched-filter degradation (Bug #54).
		// Heterodyne produces image at 2*cf ± cutoff. Lowest image freq:
		//   2*cf - cutoff = 2*1500 - 0.9*bw/2.
		// Transition must fit between cutoff and that image frequency:
		//   max_transition = (2*cf - cutoff) - cutoff = 2*(cf - cutoff).
		// WB: 2*(1500-1055)=891 Hz, use 85% → ~757 Hz (~127 taps).
		//   Stopband starts at 1055+757=1812 Hz, image at 1945 Hz.
		//   133 Hz of stopband before image → ~20 dB rejection.
		//   50% (215 taps) gave identical results, not worth the CPU.
		// NB: 2*(1500-211)=2578 Hz, cap at 2000 → 49 taps (unchanged).
		{
			double cutoff_ts = 0.9 * bw / 2.0;
			double image_gap = 2.0 * (cf - cutoff_ts);
			ofdm.FIR_rx_time_sync.filter_transition_bandwidth = std::min(image_gap * 0.85, 2000.0);
		}
		ofdm.FIR_rx_data.lpf_filter_cut_frequency = 1.0 * bw / 2;
		ofdm.FIR_rx_data.filter_transition_bandwidth = 3000 * bw_ratio;  // narrow: 161 taps, tight
		ofdm.FIR_tx1.lpf_filter_cut_frequency = cf + bw / 2;
		ofdm.FIR_tx1.hpf_filter_cut_frequency = cf - bw / 2;
		ofdm.FIR_tx1.filter_transition_bandwidth = 1000;  // WB default: 97 taps, GI-tolerable
		ofdm.FIR_tx2.lpf_filter_cut_frequency = cf + bw / 2;
		ofdm.FIR_tx2.hpf_filter_cut_frequency = cf - bw / 2;
		ofdm.FIR_tx2.filter_transition_bandwidth = 1000;  // WB default: 97 taps, GI-tolerable
		if(g_verbose) {
			printf("[INIT-DIAG] Nc=%d bw=%.1f cf=%.1f FIR_data_cut=%.1f FIR_ts_cut=%.1f nb=%d\n",
				ofdm.Nc, bw, cf, ofdm.FIR_rx_data.lpf_filter_cut_frequency,
				ofdm.FIR_rx_time_sync.lpf_filter_cut_frequency, narrowband_enabled);
			fflush(stdout);
		}
	}

	// Nsymb scales with Nc: narrowband (Nc=10) needs 5× more symbols than wideband (Nc=50)
	int nc_scale = 50 / ofdm.Nc;

	if(ofdm.Nsymb==AUTO_SELLECT)
	{
		if(M==MOD_MFSK)
		{
			// MFSK: bits per symbol period = nBits * nStreams
			// Nsymb = LDPC codeword length / bits per symbol period
			ofdm.Nsymb = N_MAX / mfsk.bits_per_symbol();
		}
		else if(ofdm.pilot_configurator.pilot_density==HIGH_DENSITY)
		{
			if(M==MOD_BPSK){ofdm.Nsymb=48 * nc_scale;}
			if(M==MOD_QPSK){ofdm.Nsymb=24 * nc_scale;}
			if(M==MOD_8PSK){ofdm.Nsymb=16 * nc_scale;}
			if(M==MOD_16QAM){ofdm.Nsymb=12 * nc_scale;}
			if(M==MOD_32QAM){ofdm.Nsymb=9 * nc_scale;}
			if(M==MOD_64QAM){ofdm.Nsymb=8 * nc_scale;}
		}
		else if(ofdm.pilot_configurator.pilot_density==LOW_DENSITY)
		{
			if(M==MOD_BPSK){ofdm.Nsymb=40 * nc_scale;}
			if(M==MOD_QPSK){ofdm.Nsymb=20 * nc_scale;}
			if(M==MOD_8PSK){ofdm.Nsymb=16 * nc_scale;}
			if(M==MOD_16QAM){ofdm.Nsymb=10 * nc_scale;}
			if(M==MOD_32QAM){ofdm.Nsymb=9 * nc_scale;}
			if(M==MOD_64QAM){ofdm.Nsymb=8 * nc_scale;}
		}
	}

	if(M!=MOD_MFSK && ofdm.pilot_configurator.Dx==AUTO_SELLECT)
	{
		if(M==MOD_BPSK){ofdm.pilot_configurator.Dx=1;}
		if(M==MOD_QPSK){ofdm.pilot_configurator.Dx=1;}
		if(M==MOD_8PSK){ofdm.pilot_configurator.Dx=1;}
		if(M==MOD_16QAM){ofdm.pilot_configurator.Dx=1;}
		if(M==MOD_32QAM){ofdm.pilot_configurator.Dx=1;}
		if(M==MOD_64QAM){ofdm.pilot_configurator.Dx=1;}
	}

	if(M!=MOD_MFSK && ofdm.pilot_configurator.Dy==AUTO_SELLECT)
	{
		if(ofdm.pilot_configurator.pilot_density==HIGH_DENSITY)
		{
			if(M==MOD_BPSK){ofdm.pilot_configurator.Dy=3;}
			if(M==MOD_QPSK){ofdm.pilot_configurator.Dy=3;}
			if(M==MOD_8PSK){ofdm.pilot_configurator.Dy=3;}
			if(M==MOD_16QAM){ofdm.pilot_configurator.Dy=3;}
			if(M==MOD_32QAM){ofdm.pilot_configurator.Dy=3;}
			if(M==MOD_64QAM){ofdm.pilot_configurator.Dy=3;}
		}
		else if(ofdm.pilot_configurator.pilot_density==LOW_DENSITY)
		{
			if(M==MOD_BPSK){ofdm.pilot_configurator.Dy=5;}
			if(M==MOD_QPSK){ofdm.pilot_configurator.Dy=5;}
			if(M==MOD_8PSK){ofdm.pilot_configurator.Dy=3;}
			if(M==MOD_16QAM){ofdm.pilot_configurator.Dy=5;}
			if(M==MOD_32QAM){ofdm.pilot_configurator.Dy=3;}
			if(M==MOD_64QAM){ofdm.pilot_configurator.Dy=3;}
		}

	}

	// MFSK doesn't use pilots, but pilot_configurator needs valid Dx/Dy
	if(M == MOD_MFSK)
	{
		if(ofdm.pilot_configurator.Dx == AUTO_SELLECT) ofdm.pilot_configurator.Dx = 1;
		if(ofdm.pilot_configurator.Dy == AUTO_SELLECT) ofdm.pilot_configurator.Dy = ofdm.Nsymb;
	}

	if(operation_mode==ARQ_MODE)
	{
		ofdm.pilot_configurator.print_on=NO;
		ofdm.preamble_configurator.print_on=NO;
	}

	if(reinit_subsystems.ofdm==YES)
	{
		ofdm.init();
		reinit_subsystems.ofdm=NO;
	}

	if(reinit_subsystems.ldpc==YES)
	{
		ldpc.init();
		// Step 15: SACK LDPC (rate-1/4 N=128 K=32) initialization removed —
		// legacy MFSK SACK bitmap encoding is gone; OFDM SACK_RSP uses the
		// regular OFDM LDPC codeword.
		reinit_subsystems.ldpc=NO;
	}
	calculate_parameters();

	if(reinit_subsystems.ofdm_FIR_rx_data==YES)
	{
		ofdm.FIR_rx_data.sampling_frequency=this->sampling_frequency;
		ofdm.FIR_rx_data.design();
		reinit_subsystems.ofdm_FIR_rx_data=NO;
	}

	if(reinit_subsystems.ofdm_FIR_rx_time_sync==YES)
	{
		ofdm.FIR_rx_time_sync.sampling_frequency=this->sampling_frequency;
		ofdm.FIR_rx_time_sync.design();
		reinit_subsystems.ofdm_FIR_rx_time_sync=NO;
	}

	if(reinit_subsystems.ofdm_FIR_tx1==YES)
	{
		ofdm.FIR_tx1.sampling_frequency=this->sampling_frequency;
		ofdm.FIR_tx1.design();
		reinit_subsystems.ofdm_FIR_tx1=NO;
	}

	if(reinit_subsystems.ofdm_FIR_tx2==YES)
	{
		ofdm.FIR_tx2.sampling_frequency=this->sampling_frequency;
		ofdm.FIR_tx2.design();
		reinit_subsystems.ofdm_FIR_tx2=NO;
	}

	if(reinit_subsystems.data_container==YES)
	{
		if(M == MOD_MFSK)
		{
			// MFSK: nData = Nsymb (no pilots)
			// Effective M = 2^(nBits*nStreams) so that nData*log2(M_eff) = N_MAX
			int M_eff = 1 << mfsk.bits_per_symbol();
			data_container.set_size(ofdm.Nsymb, ofdm.Nc, M_eff, ofdm.Nfft, ofdm.Nfft*(1+ofdm.gi), ofdm.Nsymb, ofdm.preamble_configurator.Nsymb, frequency_interpolation_rate);
		}
		else
		{
			data_container.set_size(ofdm.pilot_configurator.nData, ofdm.Nc, M, ofdm.Nfft, ofdm.Nfft*(1+ofdm.gi), ofdm.Nsymb, ofdm.preamble_configurator.Nsymb, frequency_interpolation_rate);
		}
		reinit_subsystems.data_container=NO;
	}

	if(reinit_subsystems.pre_equalization_channel==YES && M != MOD_MFSK)
	{
		pre_equalization_channel=CNEW(struct st_channel_complex, data_container.Nc, "ts.pre_eq_channel");
		get_pre_equalization_channel();
		reinit_subsystems.pre_equalization_channel=NO;
	}

	ts_srandom (bit_energy_dispersal_seed);   // §10.1: per-instance when opted in
	bit_energy_dispersal_seed = default_configurations_telecom_system.bit_energy_dispersal_seed;
	for(int i=0;i<ldpc.N;i++)
	{
		data_container.bit_energy_dispersal_sequence[i]=ts_random()%2;
	}

	// Print active gain entry for this config (verbose only)
	if(g_verbose) {
		int nb = (narrowband_enabled == YES) ? 1 : 0;
		const char* mode = nb ? "NB" : "WB";
		if(M == MOD_MFSK)
		{
			tx_signal_type sig = (mfsk.nStreams == 1) ? TX_SIG_MFSK_1S : TX_SIG_MFSK_2S;
			printf("[TX-GAIN] %s MFSK %dS boost=%.4f\n", mode, mfsk.nStreams, tx_gain[sig][nb][nb]);
		}
		else
		{
			printf("[TX-GAIN] %s OFDM boost=%.4f\n", mode, tx_gain[TX_SIG_OFDM][nb][nb]);
		}
		printf("[TX-GAIN] %s ACK boost=%.4f  BREAK boost=%.4f\n",
			mode, tx_gain[TX_SIG_ACK][nb][nb], tx_gain[TX_SIG_BREAK][nb][nb]);
	}

	receive_stats.iterations_done=-1;
	receive_stats.delay=0;
	receive_stats.delay_of_last_decoded_message=-1;
	receive_stats.mfsk_search_raw=0;
	receive_stats.ofdm_search_raw=0;
	receive_stats.ofdm_batch_active=false;
	receive_stats.ofdm_drift_per_frame=0.0;
	receive_stats.time_peak_symb_location=0;
	receive_stats.time_peak_subsymb_location=0;
	receive_stats.sync_trials=0;
	receive_stats.phase_error_avg=0;
	receive_stats.freq_offset=0;
	receive_stats.freq_offset_of_last_decoded_message=0;
	receive_stats.message_decoded=NO;
	receive_stats.SNR=-99.9;
	receive_stats.signal_stregth_dbm=-999;
	consecutive_ofdm_decode_fails=0;  // STALE-CFO scoped reset (long-run-degradation.md §2.2)

}

void cl_telecom_system::deinit()
{
	// Check all canary guards before freeing — if any canary is corrupted,
	// the buffer it guards was overflowed during RX processing
	canary_check_all();
	canary_clear();

	if(reinit_subsystems.data_container==YES)
	{
		data_container.deinit();
	}
	if(reinit_subsystems.ofdm_FIR_rx_data==YES)
	{
		ofdm.FIR_rx_data.deinit();
	}
	if(reinit_subsystems.ofdm_FIR_rx_time_sync==YES)
	{
		ofdm.FIR_rx_time_sync.deinit();
	}
	if(reinit_subsystems.ofdm_FIR_tx1==YES)
	{
		ofdm.FIR_tx1.deinit();
	}
	if(reinit_subsystems.ofdm_FIR_tx2==YES)
	{
		ofdm.FIR_tx2.deinit();
	}
	if(reinit_subsystems.ldpc==YES)
	{
		ldpc.deinit();
	}
	if(reinit_subsystems.ofdm==YES)
	{
		ofdm.deinit();
	}
	if(reinit_subsystems.pre_equalization_channel==YES)
	{
		CDELETE(pre_equalization_channel);
	}

}

void cl_telecom_system::TX_RAND_process_main()
{
	static int is_first_message=YES;
	for(int i=0;i<data_container.nBits-ldpc.P;i++)
	{
		data_container.data_bit[i]=rand()%2;
	}
	if(is_first_message==YES)
	{
		transmit_bit(data_container.data_bit,data_container.passband_data,FIRST_MESSAGE);
		is_first_message=NO;
	}
	else
	{
		transmit_bit(data_container.data_bit,data_container.passband_data,MIDDLE_MESSAGE);
	}
	tx_transfer(data_container.passband_data,data_container.Nofdm*data_container.interpolation_rate*(ofdm.Nsymb+ofdm.preamble_configurator.Nsymb));
}

void cl_telecom_system::TX_TEST_process_main()
{
    int nReal_data = data_container.nBits - ldpc.P;
    int frame_size = (nReal_data - outer_code_reserved_bits) / 8;

    static int counter = 0;

    for (int i = 0; i < frame_size; i++)
    {
        data_container.data_byte[i] = 0; // data_byte is an integer
    }
    data_container.data_byte[counter % frame_size] = 1;
    counter++;

    transmit_byte(data_container.data_byte, (nReal_data - outer_code_reserved_bits) / 8, data_container.passband_data, SINGLE_MESSAGE);

    tx_transfer(data_container.passband_data, data_container.Nofdm * data_container.interpolation_rate * (ofdm.Nsymb + ofdm.preamble_configurator.Nsymb));

}

void cl_telecom_system::TX_SHM_process_main(cbuf_handle_t buffer)
{
    static uint32_t spinner_anim = 0; char spinner[] = ".oOo";
    int nReal_data = data_container.nBits - ldpc.P;
    // int frame_size_bits = nReal_data - outer_code_reserved_bits;
    int frame_size = (nReal_data - outer_code_reserved_bits) / 8;
    // int input_buffer_size = 0;
    // std::cout<<"Extra unused bits: "<< frame_size_bits - (frame_size * 8)<<",";
    // std::cout<<std::endl;

    uint8_t data[frame_size];

    // check the data in the buffer, if smaller than frame size, transmits 0
    if ((int) size_buffer(buffer) >= frame_size)
    {
        // memset(data, 0, frame_size);
        read_buffer(buffer, data, frame_size);

        for (int i = 0; i < frame_size; i++)
        {
            data_container.data_byte[i] = data[i];
        }
    }
    // if there is no data in the buffer, just do nothing
    else
    {
		msleep(10);
		return;
    }

    transmit_byte(data_container.data_byte, (nReal_data - outer_code_reserved_bits) / 8, data_container.passband_data, SINGLE_MESSAGE);

    tx_transfer(data_container.passband_data, data_container.Nofdm * data_container.interpolation_rate * (ofdm.Nsymb + ofdm.preamble_configurator.Nsymb));

    printf("%c\033[1D", spinner[spinner_anim % 4]); spinner_anim++;
    fflush(stdout);
}


void cl_telecom_system::RX_RAND_process_main()
{
	std::complex <double> data_fft[ofdm.pilot_configurator.nData];
	int constellation_plot_counter=0;
	int constellation_plot_nFrames=1;
	float contellation[ofdm.pilot_configurator.nData*constellation_plot_nFrames][2]={0};
    int nReal_data = data_container.nBits - ldpc.P;
    int frame_size = (nReal_data - outer_code_reserved_bits) / 8;
    int out_data[N_MAX];

	int signal_period = data_container.Nofdm * data_container.buffer_Nsymb * data_container.interpolation_rate; // in samples
	int symbol_period = data_container. Nofdm * data_container.interpolation_rate;

	if(data_container.data_ready == 0)
	{
		msleep(1);
		return;
	}

	MUTEX_LOCK(&capture_prep_mutex);
	if (data_container.frames_to_read == 0)
	{

		int rwi = data_container.ring_write_index;
		memcpy(data_container.ready_to_process_passband_delayed_data, &data_container.passband_delayed_data[rwi], signal_period * sizeof(double));

		st_receive_stats received_message_stats = receive_byte(data_container.ready_to_process_passband_delayed_data, out_data);

		if(received_message_stats.message_decoded == YES)
		{
			printf("Frame decoded in %d iterations. Data: \n", received_message_stats.iterations_done);

			for(int i = 0; i < frame_size; i++)
				printf("0x%x, ", out_data[i]);

			std::cout << std::endl;
			std::cout << std::dec;
			std::cout << " sync_trial=" << receive_stats.sync_trials;
			std::cout << " time_peak_subsymb_location=" << received_message_stats.delay % (data_container.Nofdm * data_container.interpolation_rate);
			std::cout << " time_peak_symb_location=" << received_message_stats.delay / (data_container.Nofdm * data_container.interpolation_rate);
			std::cout << " freq_offset=" << receive_stats.freq_offset;
			std::cout << " SNR=" << receive_stats.SNR << " dB";
			std::cout << " Signal Strength=" << receive_stats.signal_stregth_dbm << " dBm ";
			std::cout << std::endl;

			int end_of_current_message = received_message_stats.delay / symbol_period + data_container.Nsymb + data_container.preamble_nSymb;
			int frames_left_in_buffer = data_container.buffer_Nsymb - end_of_current_message;
			if(frames_left_in_buffer < 0)
				frames_left_in_buffer = 0;

			data_container.frames_to_read = data_container.Nsymb + data_container.preamble_nSymb - frames_left_in_buffer - data_container.nUnder_processing_events;

			if(data_container.frames_to_read > (data_container.Nsymb + data_container.preamble_nSymb) || data_container.frames_to_read < 0)
				data_container.frames_to_read = data_container.Nsymb + data_container.preamble_nSymb - frames_left_in_buffer;

			receive_stats.delay_of_last_decoded_message += (data_container.Nsymb + data_container.preamble_nSymb - data_container.frames_to_read) * symbol_period;

			data_container.nUnder_processing_events = 0;
		}
		else
		{
			if(data_container.frames_to_read == 0 && receive_stats.delay_of_last_decoded_message != -1)
			{
				receive_stats.delay_of_last_decoded_message -= symbol_period;
				if(receive_stats.delay_of_last_decoded_message < 0)
				{
					receive_stats.delay_of_last_decoded_message = -1;
				}
			}
			//				std::cout<<" Signal Strength="<<receive_stats.signal_stregth_dbm<<" dBm ";
			//				std::cout<<std::endl;
		}
		for(int i=0;i<ofdm.pilot_configurator.nData;i++)
		{
			contellation[constellation_plot_counter*ofdm.pilot_configurator.nData+i][0]=data_fft[i].real();
			contellation[constellation_plot_counter*ofdm.pilot_configurator.nData+i][1]=data_fft[i].imag();
		}

		constellation_plot_counter++;

		if(constellation_plot_counter==constellation_plot_nFrames)
		{
			constellation_plot_counter=0;
			constellation_plot.plot_constellation(&contellation[0][0],ofdm.pilot_configurator.nData*constellation_plot_nFrames);
		}
	}
	data_container.data_ready = 0;
	MUTEX_UNLOCK(&capture_prep_mutex);
}

void cl_telecom_system::RX_TEST_process_main()
{
    int out_data[N_MAX];
    int nReal_data = data_container.nBits - ldpc.P;
    int frame_size = (nReal_data - outer_code_reserved_bits) / 8;
	// int buff_size = data_container.Nofdm * data_container.buffer_Nsymb * data_container.interpolation_rate * 2;

	int signal_period = data_container.Nofdm * data_container.buffer_Nsymb * data_container.interpolation_rate; // in samples
	int symbol_period = data_container. Nofdm * data_container.interpolation_rate;

	if(data_container.data_ready == 0)
	{
		msleep(1);
		return;
	}

	MUTEX_LOCK(&capture_prep_mutex);
	if (data_container.frames_to_read == 0)
	{

		int rwi = data_container.ring_write_index;
		memcpy(data_container.ready_to_process_passband_delayed_data, &data_container.passband_delayed_data[rwi], signal_period * sizeof(double));

		st_receive_stats received_message_stats = receive_byte(data_container.ready_to_process_passband_delayed_data, out_data);

		if(received_message_stats.message_decoded == YES)
		{
			printf("Frame decoded in %d iterations. Data: \n", received_message_stats.iterations_done);

			for(int i = 0; i < frame_size; i++)
				printf("0x%x, ", out_data[i]);

			std::cout << std::endl;
			std::cout << std::dec;
			std::cout << " sync_trial=" << receive_stats.sync_trials;
			std::cout << " time_peak_subsymb_location=" << received_message_stats.delay % (data_container.Nofdm * data_container.interpolation_rate);
			std::cout << " time_peak_symb_location=" << received_message_stats.delay / (data_container.Nofdm * data_container.interpolation_rate);
			std::cout << " freq_offset=" << receive_stats.freq_offset;
			std::cout << " SNR=" << receive_stats.SNR << " dB";
			std::cout << " Signal Strength=" << receive_stats.signal_stregth_dbm << " dBm ";
			std::cout << std::endl;

			int end_of_current_message = received_message_stats.delay / symbol_period + data_container.Nsymb + data_container.preamble_nSymb;
			int frames_left_in_buffer = data_container.buffer_Nsymb - end_of_current_message;
			if(frames_left_in_buffer < 0)
				frames_left_in_buffer = 0;

			data_container.frames_to_read = data_container.Nsymb + data_container.preamble_nSymb - frames_left_in_buffer - data_container.nUnder_processing_events;

			if(data_container.frames_to_read > (data_container.Nsymb + data_container.preamble_nSymb) || data_container.frames_to_read < 0)
				data_container.frames_to_read = data_container.Nsymb + data_container.preamble_nSymb - frames_left_in_buffer;

			receive_stats.delay_of_last_decoded_message += (data_container.Nsymb + data_container.preamble_nSymb - data_container.frames_to_read) * symbol_period;

			data_container.nUnder_processing_events = 0;
		}
		else
		{
			if(data_container.frames_to_read == 0 && receive_stats.delay_of_last_decoded_message != -1)
			{
				receive_stats.delay_of_last_decoded_message -= symbol_period;
				if(receive_stats.delay_of_last_decoded_message < 0)
				{
					receive_stats.delay_of_last_decoded_message = -1;
				}
			}
			//				std::cout<<" Signal Strength="<<receive_stats.signal_stregth_dbm<<" dBm ";
			//				std::cout<<std::endl;
		}

	}
	data_container.data_ready = 0;
	MUTEX_UNLOCK(&capture_prep_mutex);
}


void cl_telecom_system::RX_SHM_process_main(cbuf_handle_t buffer)
{
    static uint32_t spinner_anim = 0; char spinner[] = ".oOo";
	int out_data[N_MAX];
    int nReal_data = data_container.nBits - ldpc.P;
    int frame_size = (nReal_data - outer_code_reserved_bits) / 8;

	int signal_period = data_container.Nofdm * data_container.buffer_Nsymb * data_container.interpolation_rate; // in samples
	int symbol_period = data_container.Nofdm * data_container.interpolation_rate;

	// lock
	if(data_container.data_ready == 0)
	{
		msleep(1);
		return;
	}

	MUTEX_LOCK(&capture_prep_mutex);
	// Guard: deinit path zeros Nofdm/buffer_Nsymb under this mutex before freeing
	// buffers. If we see zero, buffers are being freed — skip processing.
	if(data_container.Nofdm == 0 || data_container.buffer_Nsymb == 0)
	{
		data_container.data_ready = 0;
		MUTEX_UNLOCK(&capture_prep_mutex);
		return;
	}
	// Recompute under mutex for consistency with guard check
	signal_period = data_container.Nofdm * data_container.buffer_Nsymb * data_container.interpolation_rate;
	symbol_period = data_container.Nofdm * data_container.interpolation_rate;
	if (data_container.frames_to_read == 0)
	{
#ifdef MERCURY_GUI_ENABLED
		// Apply live LDPC iteration limit from GUI.
		// GOTCHA (Q3 per-config override): this overwrite is unconditional and
		// runs after load_configuration(), so a GUI update can clobber the
		// per-config nIteration_max=200 that ROBUST tier configs (100/101/102)
		// install in their load path. If you ever see ROBUST tier underperform
		// at low SNR while the GUI is showing a smaller iteration cap, suspect
		// this line. Safer fix (deferred — medium risk): guard with
		// `!is_robust_config(current_configuration)` so the override only fires
		// for OFDM configs.
		int gui_ldpc_max = g_gui_state.ldpc_iterations_max.load();
		if (gui_ldpc_max >= 5 && gui_ldpc_max <= 100)
			ldpc.nIteration_max = gui_ldpc_max;
#endif

		int rwi = data_container.ring_write_index;
		memcpy(data_container.ready_to_process_passband_delayed_data, &data_container.passband_delayed_data[rwi], signal_period * sizeof(double));

		auto proc_start = std::chrono::steady_clock::now();
		st_receive_stats received_message_stats = receive_byte(data_container.ready_to_process_passband_delayed_data, out_data);
		auto proc_end = std::chrono::steady_clock::now();
		double proc_ms = std::chrono::duration<double, std::milli>(proc_end - proc_start).count();
		canary_check_all();

		// Frame period = (preamble + data symbols) in wall clock time
		double frame_samples = (double)(data_container.Nofdm * (data_container.Nsymb + data_container.preamble_nSymb) * data_container.interpolation_rate);
		double frame_ms = (frame_samples / 48000.0) * 1000.0;
		float load = (frame_ms > 0) ? (float)(proc_ms / frame_ms) : 0.0f;

#ifdef MERCURY_GUI_ENABLED
		g_gui_state.processing_load.store(load);
		size_t buf_used = size_buffer(capture_buffer);
		size_t buf_cap = circular_buf_capacity(capture_buffer);
		g_gui_state.buffer_fill_pct.store(buf_cap > 0 ? 100.0f * (float)buf_used / (float)buf_cap : 0.0f);
#endif

		if(received_message_stats.message_decoded == YES)
		{
			// printf("Frame decoded in %d iterations. Data: \n", received_message_stats.iterations_done);
			uint8_t data[frame_size];
			for(int i = 0; i < frame_size; i++)
			{
				data[i] = (uint8_t) out_data[i];
			}

			if ( frame_size <= (int) circular_buf_free_size(buffer) )
				write_buffer(buffer, data, frame_size);
			else
				printf("Decoded frame lost because of full buffer!\n");


			// Only display signal strength if in reasonable range (-150 to +50 dBm)
			if (receive_stats.signal_stregth_dbm >= -150 && receive_stats.signal_stregth_dbm <= 50)
				printf("\rSNR: %5.1f db  Level: %5.1f dBm  Load: %.2fx  Buf: %.0f%%  RX: %c",
					receive_stats.SNR, receive_stats.signal_stregth_dbm, load,
					(circular_buf_capacity(capture_buffer) > 0 ? 100.0 * size_buffer(capture_buffer) / circular_buf_capacity(capture_buffer) : 0.0),
					spinner[spinner_anim % 4]);
			else
				printf("\rSNR: %5.1f db  Load: %.2fx  Buf: %.0f%%  RX: %c",
					receive_stats.SNR, load,
					(circular_buf_capacity(capture_buffer) > 0 ? 100.0 * size_buffer(capture_buffer) / circular_buf_capacity(capture_buffer) : 0.0),
					spinner[spinner_anim % 4]);
			spinner_anim++;
			fflush(stdout);

			int end_of_current_message = received_message_stats.delay / symbol_period + data_container.Nsymb + data_container.preamble_nSymb;
			int frames_left_in_buffer = data_container.buffer_Nsymb - end_of_current_message;
			if(frames_left_in_buffer < 0)
				frames_left_in_buffer = 0;

			data_container.frames_to_read = data_container.Nsymb + data_container.preamble_nSymb - frames_left_in_buffer - data_container.nUnder_processing_events;

			if(data_container.frames_to_read > (data_container.Nsymb + data_container.preamble_nSymb) || data_container.frames_to_read < 0)
				data_container.frames_to_read = data_container.Nsymb + data_container.preamble_nSymb - frames_left_in_buffer;

			receive_stats.delay_of_last_decoded_message += (data_container.Nsymb + data_container.preamble_nSymb - data_container.frames_to_read) * symbol_period;

			data_container.nUnder_processing_events = 0;
		}
		else
		{
			if(data_container.frames_to_read == 0 && receive_stats.delay_of_last_decoded_message != -1)
			{
				receive_stats.delay_of_last_decoded_message -= symbol_period;
				if(receive_stats.delay_of_last_decoded_message < 0)
				{
					receive_stats.delay_of_last_decoded_message = -1;
				}
			}
			// Periodic status while scanning (every ~4 frames)
			if (spinner_anim % 4 == 0) {
				printf("\rLoad: %.2fx  Buf: %.0f%%  Scanning... %c",
					load,
					(circular_buf_capacity(capture_buffer) > 0 ? 100.0 * size_buffer(capture_buffer) / circular_buf_capacity(capture_buffer) : 0.0),
					spinner[spinner_anim % 4]);
				fflush(stdout);
			}
			spinner_anim++;
		}

	}
	data_container.data_ready = 0;
	MUTEX_UNLOCK(&capture_prep_mutex);
}


void cl_telecom_system::BER_PLOT_baseband_process_main()
{
	if(M == MOD_MFSK)
	{
		std::cout<<"PLOT_BASEBAND not supported for MFSK configs. Use PLOT_PASSBAND instead."<<std::endl;
		return;
	}
	BER_plot.open("BER");
	BER_plot.reset("BER");
	int nPoints=25;
	float data_plot[nPoints][2];
	float data_plot_theo[nPoints][2];
	output_power_Watt=1;
	int start_location=-10;

	for(int ind=0;ind<nPoints;ind++)
	{
		float EsN0=(float)(ind/2.0+start_location);

		data_plot[ind][0]=EsN0;

		data_plot[ind][1]=baseband_test_EsN0(EsN0,100).BER;

		data_plot_theo[ind][0]=EsN0 ;

		if(M==MOD_BPSK)
		{
			data_plot_theo[ind][1]=0.5*erfc(sqrt(pow(10,EsN0/10)));
		}
		else
		{
			data_plot_theo[ind][1]=(2.0/log2(M))*(1.0-1.0/sqrt(M))*erfc(sqrt(((3.0* log2(M))/(2.0*(M-1))) *pow(10,EsN0/10)/log2(M)));
		}
		std::cout<<EsN0<<";"<<data_plot[ind][1]<<std::endl;
	}

	BER_plot.plot("BER Simulation",&data_plot[0][0],nPoints,"BER theoretical",&data_plot_theo[0][0],nPoints);
	BER_plot.close();
}
void cl_telecom_system::BER_PLOT_passband_process_main()
{
	// Step 15: legacy SACK pattern roundtrip test removed (function deleted
	// alongside the MFSK SACK bitmap path).

	// ---- TIMING-ACQUISITION-UNDER-SFO HARNESS (F1 fix) -------------------------
	// MERCURY_SFO_BLOCK_TEST=1 reuses the PLOT_PASSBAND mode (-m PLOT_PASSBAND -s
	// <cfg>) as the entry point for the long-block-decode-under-SFO harness. Unlike
	// the standard BER sweep below (which sets ofdm_forced_delay = a KNOWN position
	// and bypasses Schmidl-Cox / Moose — telecom_system.cc:504,2137,2462) and unlike
	// the in-process 2-instance pump (which hands the decoder a FRAME-ALIGNED window
	// at a pinned ring offset — arq_commander.cc:~9822), this harness makes the RX
	// ACQUIRE timing from a long, SFO-drifted continuous block, so SFO actually
	// matters. See fact-documents/data-flow-sim2-time-domain-faithfulness.md §10.
	const char* sgt = std::getenv("MERCURY_SFO_GRID");
	if(sgt && atoi(sgt) != 0)
	{
		sfo_grid_test();
		return;
	}
	const char* sft = std::getenv("MERCURY_SFO_BLOCK_TEST");
	if(sft && atoi(sft) != 0)
	{
		sfo_block_test();
		return;
	}
	// BIG-BLOCK HW DE-RISK (PHY-only): emit the validated big-block to a WAV, or
	// decode a recorded WAV. See bigblock_tx_to_wav / bigblock_decode_from_wav and
	// fact-documents/bigblock-hw-wav-derisk.md. Entry shares -m PLOT_PASSBAND -s 16.
	const char* bbtx = std::getenv("MERCURY_BIGBLOCK_TX_WAV");
	if(bbtx && *bbtx)
	{
		bigblock_tx_to_wav(bbtx);
		return;
	}
	const char* bbdec = std::getenv("MERCURY_BIGBLOCK_DECODE_WAV");
	if(bbdec && *bbdec)
	{
		bigblock_decode_from_wav(bbdec);
		return;
	}
	// P1: LIVE-PATH big-block loopback validation. Unlike the *_WAV hooks (which call
	// the standalone harness methods), this drives the big-block through the
	// PRODUCTION transmit_byte / receive_byte (gated on bigblock_framing_enabled), so
	// it proves the PHY is wired into the live path — TX emits one block, an in-memory
	// passband round-trip (NO channel; clean loopback) feeds it back through
	// receive_byte, and the per-codeword gate asserts 8/8 byte-correct. Optional
	// MERCURY_BIGBLOCK_LIVE_ESN0 adds AWGN to confirm the estimator/LLR path runs.
	const char* bblive = std::getenv("MERCURY_BIGBLOCK_LIVE");
	if(bblive && atoi(bblive) != 0)
	{
		bigblock_livepath_loopback();
		return;
	}

	BER_plot.open("BER");
	BER_plot.reset("BER");
	// MFSK: sweep channel SNR from -25 to +5 dB in 1 dB steps
	// OFDM: sweep Es/N0 from -10 to +15 dB in 0.5 dB steps
	// MFSK: sweep channel SNR from -25 to +5 dB in 1 dB steps
	// OFDM: sweep Es/N0 from -10 to +15 dB in 0.5 dB steps
	int nPoints = (M == MOD_MFSK) ? 31 : 51;
	int nFrames_per_point = (M == MOD_MFSK) ? 3 : 100;
	float data_plot[nPoints][2];
	float data_plot_theo[nPoints][2];
	output_power_Watt=1;
	float start_location = (M == MOD_MFSK) ? -25.0f : -10.0f;
	float step_size = (M == MOD_MFSK) ? 1.0f : 0.5f;

	// fix/cfg16-nv-restore: fast single-point / windowed BER override for
	// validation (e.g. high-SNR clean-channel nv measurement). When
	// ber_single_esn0 > -900, evaluate one Es/N0 point and return immediately.
	// Production sweep behavior is unchanged when the override is unset.
	if(ber_single_esn0 > -900.0f)
	{
		int nf = (ber_frames_override > 0) ? ber_frames_override : nFrames_per_point;
		float b = passband_test_EsN0(ber_single_esn0, nf).BER;
		std::cout<<ber_single_esn0<<";"<<b<<std::endl;
		BER_plot.close();
		return;
	}

	for(int ind=0;ind<nPoints;ind++)
	{
		float EsN0=(float)(ind * step_size + start_location);

		data_plot[ind][0]=EsN0;

		data_plot[ind][1]=passband_test_EsN0(EsN0,nFrames_per_point).BER;

		data_plot_theo[ind][0]=EsN0 ;

		if(M==MOD_BPSK)
		{
			data_plot_theo[ind][1]=0.5*erfc(sqrt(pow(10,EsN0/10)));
		}
		else if(M==MOD_MFSK)
		{
			// MFSK theoretical: no simple closed-form, use 0 placeholder
			data_plot_theo[ind][1]=0;
		}
		else
		{
			data_plot_theo[ind][1]=(2.0/log2(M))*(1.0-1.0/sqrt(M))*erfc(sqrt(((3.0* log2(M))/(2.0*(M-1))) *pow(10,EsN0/10)/log2(M)));
		}
		std::cout<<EsN0<<";"<<data_plot[ind][1]<<std::endl;
	}

	BER_plot.plot("BER Simulation",&data_plot[0][0],nPoints,"BER theoretical",&data_plot_theo[0][0],nPoints);
	BER_plot.close();

	// Run ACK pattern detection test for MFSK modes
	if(M == MOD_MFSK)
	{
		ack_pattern_detection_test();
	}
	// Step 15: legacy SACK roundtrip test removed.
}

// ============================================================================
// TIMING-ACQUISITION-UNDER-SFO HARNESS  (F1 fix — the durable infra investment)
// ============================================================================
// PURPOSE. The two existing decode paths CANNOT exhibit timing-acquisition
// failures, because both PIN the FFT window:
//   • the standard BER sweep (passband_test_EsN0) sets ofdm_forced_delay to a
//     KNOWN nominal position and the RX then bypasses Schmidl-Cox AND Moose
//     (telecom_system.cc:504 sets it; :1010/:2137/:2462 bypass on it);
//   • the in-process 2-instance pump hands the decoder a frame-aligned window
//     at a deterministic ring offset (arq_commander.cc:~9822). Injecting
//     cl_sim_sfo there leaves ARM A == ARM B byte-identical to SFO-off, because
//     the pinned window cannot move (fact-doc §9/F1).
// So SFO has ZERO effect in either path. This harness instead makes the RX
// ACQUIRE timing from a LONG, SFO-DRIFTED, CONTINUOUS block (a real OFDM batch
// is back-to-back frames on ONE continuous TX clock), so SFO matters:
//   1. TX N back-to-back CFG OFDM frames into one contiguous passband buffer
//      (the SAME transmit_byte the production TX uses; per-frame preamble length
//      governed by LEVER P's tx override / MERCURY_SIM2_MINI_NSYM).
//   2. Pass the WHOLE buffer through ONE cl_sim_sfo instance — a stateful,
//      phase-continuous fractional resampler (sim_channel.h). The integer part
//      of the accumulated SFO phase skips/repeats whole input samples, so the
//      start-of-frame CREEPS across the block; the fractional part sub-sample-
//      misaligns the Schmidl-Cox half-symbol repeat. This is the impairment the
//      pinned paths structurally cannot show.
//   3. For each frame, present a buffer_Nsymb window to the REAL receive_byte
//      with ofdm_forced_delay = -1 (so it runs Schmidl-Cox time sync + Moose
//      carrier sync + channel est + equalizer + LDPC), and compare the decoded
//      bits against the known TX bits. Record the detected (drifting) preamble
//      position + coarse metric per frame to expose the timing creep.
//
// Two arms (env-selected, NOT hard-coded here):
//   • FULL : preamble_amortization_enabled = false  ⇒ every frame 4-sym preamble
//            (long Schmidl-Cox integration L; should HOLD under SFO).
//   • MINI : preamble_amortization_enabled = true, MERCURY_SIM2_MINI_NSYM = 1
//            ⇒ tail frames carry a 1-sym preamble (L 4× shorter ⇒ metric
//            variance 4× ⇒ peak-pick jitters under SFO; should FAIL on the tail).
//
// Knobs (all default to a sensible value; NONE change the production BER sweep —
// that path is gated out above by MERCURY_SFO_BLOCK_TEST):
//   MERCURY_SFO_BLOCK_NFRAMES : frames in the block (default 60 — the make-or-
//                               break TEST 1 length for big-block framing).
//   MERCURY_SFO_BLOCK_ESN0    : channel Es/N0 in dB (default 900 = clean cell;
//                               isolates the SFO timing effect from AWGN).
//   MERCURY_SFO_BLOCK_SEED    : TX-data + AWGN RNG seed (default 12345).
//   MERCURY_SIM2_SFO_PPM / _SFO_WALK_PPM / _SFO_MAX_PPM / _SFO_SEED : the SFO
//                               itself (cl_sim_sfo knobs; default OFF ⇒ a clean
//                               control run that MUST decode the whole block).
//   MERCURY_SIM2_MINI_NSYM    : MINI-arm per-frame preamble length (LEVER P).
//
// Determinism: TX data + AWGN seeded from MERCURY_SFO_BLOCK_SEED; the SFO stage
// has its OWN cl_sim_xoshiro (sfo_seed). Same env ⇒ bit-reproducible.
void cl_telecom_system::sfo_block_test()
{
	if(M == MOD_MFSK)
	{
		std::cout << "[SFO-BLOCK] MFSK not supported (OFDM timing-acquisition harness); use -s 15/16." << std::endl;
		return;
	}

	auto env_i = [](const char* k, int def) { const char* e = std::getenv(k); return (e && *e) ? atoi(e) : def; };
	auto env_f = [](const char* k, double def) { const char* e = std::getenv(k); return (e && *e) ? atof(e) : def; };

	int    nFrames = env_i("MERCURY_SFO_BLOCK_NFRAMES", 60);
	if(nFrames < 1) nFrames = 1;
	double esn0_db = env_f("MERCURY_SFO_BLOCK_ESN0", 900.0);
	uint64_t seed  = (uint64_t)env_i("MERCURY_SFO_BLOCK_SEED", 12345);

	// LEVER P arm selection: MINI engages preamble amortization (the per-frame
	// MINI preamble length is read from MERCURY_SIM2_MINI_NSYM inside
	// preamble_sched_nsymb). FULL forces every frame to the configured 4-sym
	// preamble. The arm is chosen from preamble_amortization_enabled, which the
	// in-process pump path sets via the CLI (arq_commander.cc:10150). Standalone
	// (this PLOT_PASSBAND harness has no ARQ pump to set it), select the arm with
	// MERCURY_SFO_BLOCK_ARM=MINI|FULL (default = the member flag = FULL).
	{
		const char* arm = std::getenv("MERCURY_SFO_BLOCK_ARM");
		if(arm && (*arm=='M' || *arm=='m')) preamble_amortization_enabled = true;
		else if(arm && (*arm=='F' || *arm=='f')) preamble_amortization_enabled = false;
	}
	bool mini_arm = preamble_amortization_enabled;
	int  full_pre = data_container.preamble_nSymb;

	// Geometry (one frame, FULL preamble = the allocation maximum).
	int interp        = frequency_interpolation_rate;
	int sym_samples   = data_container.Nofdm * interp;            // one OFDM symbol, passband
	int data_nsymb    = data_container.Nsymb;                     // data symbols / frame
	int full_frame    = (data_nsymb + full_pre) * sym_samples;    // FULL frame, passband samples
	int buf_interp    = data_container.Nofdm * data_container.buffer_Nsymb * interp; // RX window

	int nReal_data    = data_container.nBits - ldpc.P;
	int payload_bytes = (nReal_data - outer_code_reserved_bits) / 8;
	int payload_bits  = payload_bytes * 8;   // byte-aligned: matches transmit_byte(payload_bytes)

	std::cout << "[SFO-BLOCK] cfg=" << current_configuration
	          << " M=" << M << " Nsymb=" << data_nsymb
	          << " full_pre=" << full_pre
	          << " arm=" << (mini_arm ? "MINI" : "FULL")
	          << " mini_nsym=" << preamble_sched_nsymb(1, false, full_pre)
	          << " nFrames=" << nFrames
	          << " EsN0=" << esn0_db
	          << " SFO_ppm=" << env_f("MERCURY_SIM2_SFO_PPM", 0.0)
	          << " SFO_walk_ppm=" << env_f("MERCURY_SIM2_SFO_WALK_PPM", 0.0)
	          << " seed=" << seed << std::endl;

	// --- TX: build N back-to-back frames into one contiguous passband buffer. ---
	// Capacity: N FULL frames + headroom; emitted MINI frames are shorter so this
	// is a safe upper bound regardless of arm.
	std::vector<double> tx_block((size_t)nFrames * (size_t)full_frame + (size_t)sym_samples, 0.0);
	// Per-frame: TX bit pattern (for BER) + the emitted-frame sample offset/length.
	std::vector<std::vector<int>> frame_bits(nFrames);
	std::vector<long>             frame_off(nFrames, 0);
	std::vector<int>              frame_len(nFrames, 0);
	std::vector<int>              frame_pre(nFrames, 0);   // preamble symbols this frame emitted

	output_power_Watt = 1;
	ts_srandom((unsigned int)seed);

	long write_pos = 0;
	for(int f = 0; f < nFrames; f++)
	{
		// Per-frame known random payload (reproducible from the block seed).
		frame_bits[f].resize(payload_bits);
		for(int i = 0; i < payload_bits; i++)
			frame_bits[f][i] = (int)(ts_random() % 2);
		bit_to_byte(frame_bits[f].data(), data_container.data_byte, payload_bits);

		// LEVER P: drive the per-frame TX preamble length exactly like the ARQ
		// batch assembler does (arq_common.cc:3918). Frame 0 = anchor (FULL);
		// tail frames = MINI when the MINI arm is on. preamble_sched_nsymb is the
		// SAME pure schedule the production TX/RX both derive, so this faithfully
		// reproduces LEVER P's emitted-frame geometry.
		int emit_pre = full_pre;
		if(mini_arm)
		{
			tx_preamble_nsymb_override = preamble_sched_nsymb(f, /*force_full=*/false, full_pre);
			emit_pre = tx_preamble_nsymb_override;
		}
		else
		{
			tx_preamble_nsymb_override = -1; // FULL every frame
		}

		// Emit one frame into passband_data; transmit_byte populates it and sets
		// tx_last_emitted_frame_samples to the ACTUAL emitted length (preamble+data).
		tx_last_emitted_frame_samples = full_frame;
		this->transmit_byte(data_container.data_byte, payload_bytes,
		                    data_container.passband_data, SINGLE_MESSAGE);
		int emitted = tx_last_emitted_frame_samples;
		if(emitted <= 0 || emitted > full_frame) emitted = full_frame;

		frame_off[f] = write_pos;
		frame_len[f] = emitted;
		frame_pre[f] = emit_pre;
		for(int i = 0; i < emitted; i++)
			tx_block[(size_t)write_pos + i] = data_container.passband_data[i];
		write_pos += emitted;
	}
	tx_preamble_nsymb_override = -1; // defensive reset
	long block_len = write_pos;

	// --- CHANNEL: drift the WHOLE contiguous block through one cl_sim_sfo. ---
	// One stateful instance ⇒ the fractional accumulator + integer SOF-creep carry
	// across the entire block (frame boundaries are NOT realigned — exactly the HW
	// continuous-clock drift). cl_sim_sfo is n-in/n-out exact; the read pointer's
	// accumulated drift across block_len samples is the cumulative timing error the
	// per-frame acquisition must track. SFO knobs come from the env (default OFF).
	{
		cl_sim_sfo sfo(/*seed=*/ (seed ^ 0x2545F4914F6CDD1DULL) + 0xC2B2AE3D27D4EB4FULL,
		               env_f("MERCURY_SIM2_SFO_PPM", 0.0),
		               env_f("MERCURY_SIM2_SFO_WALK_PPM", 0.0),
		               env_f("MERCURY_SIM2_SFO_MAX_PPM", 90.0),
		               48000.0);
		// Process in one shot over the whole block (phase-continuous; equivalent to
		// streaming since the resampler carries state, but one call is simplest and
		// has no inter-block edge since there are no edges).
		sfo.process(tx_block.data(), (size_t)block_len);
		std::cout << "[SFO-BLOCK] SFO stage " << (sfo.enabled() ? "ON" : "OFF (control run)")
		          << " block_samples=" << block_len << std::endl;
	}

	// AWGN sigma (Es/N0), measured from the actual block power (same calibration as
	// passband_test_EsN0). Clean cell (esn0>=900) ⇒ no additive noise.
	double sigma = 0.0;
	bool clean = (esn0_db >= 900.0);
	if(!clean)
		sigma = 1.0 / sqrt(pow(10.0, esn0_db / 10.0));

	// --- RX: per-frame REAL acquisition over a buffer_Nsymb window. ---
	// Window layout: [lead margin of zeros | frame's drifted samples | trailing
	// zeros], placed in passband_delayed_data. The lead margin (a couple symbols)
	// gives the Schmidl-Cox search room and lets the SFO-creep move the preamble
	// AWAY from a fixed sample so timing acquisition is genuinely exercised. We map
	// frame f to its TX offset; the SFO has already drifted the content so the
	// actual preamble sits at frame_off[f] +/- accumulated creep, which the RX must
	// re-find. ofdm_forced_delay = -1 ⇒ full Schmidl-Cox + Moose.
	// Lead margin: place the preamble at symbol (full_pre+2) so it clears the RX
	// coarse-bounds gate (lower_bound = preamble_nSymb; the gate requires
	// pream_symb_loc > preamble_nSymb — telecom_system.cc:1701,1730). This mirrors
	// the standard BER path's forced-delay convention
	// ((preamble_nSymb+2)*Nofdm+delay). The SFO-creep then moves the actual
	// preamble away from this nominal sample, which the RX must re-acquire.
	int lead = (full_pre + 2) * sym_samples;            // preamble lands at symbol full_pre+2
	int win_frame_max = full_frame + 4 * sym_samples;   // FULL frame + slack
	if(lead + win_frame_max > buf_interp)
	{
		// Should not happen for WB CFG15/16 (buffer_Nsymb >> frame), but clamp.
		lead = 0;
		if(win_frame_max > buf_interp) win_frame_max = buf_interp;
	}

	int    decoded_ok = 0;
	long   total_bit_errors = 0;
	long   total_bits = 0;
	int    frames_zero_ber = 0;
	double metric_sum_full = 0.0, metric_sum_mini = 0.0;
	int    n_full = 0, n_mini = 0;

	// AWGN seeded once (separate from TX data stream) for reproducibility.
	awgn_channel.set_seed((long)(seed | 1));

	// ---- BIG-BLOCK ONE-ACQUISITION MODE (MERCURY_SFO_BIGBLOCK) ----------------
	// The task's "big block" = ONE timing acquisition on the HEAD preamble, then
	// decode the whole contiguous K-frame run WITHOUT per-frame re-acquisition (the
	// anti-LEVER-P model: VARA-style single-acquire-and-hold). The per-frame loop
	// above RE-ACQUIRES every frame (Schmidl-Cox each time) and so structurally
	// cannot show a tail timing failure — the very limitation §11 flagged.
	//
	// ARCHITECTURE NOTE (code-verified, fact-doc §12): a SINGLE LDPC codeword is
	// fixed at N=MERCURY_NORMAL=1600 bits (ldpc.cc:66, physical_defines.h:31), and
	// CFG16's Nsymb=9 is DERIVED so nData*log2(M)==1600 (telecom_system.cc:4598).
	// A literal "one preamble + 60 DATA symbols carrying one codeword" is therefore
	// IMPOSSIBLE without a new ~10k-bit LDPC code. The faithful realization of a
	// 60-data-symbol block is K=ceil(60/Nsymb) back-to-back codeword-frames under
	// ONE head acquisition: each frame is a valid 1600-bit codeword (real LDPC
	// block-decode metric), the SFO drifts the WHOLE run on one clock, and the RX
	// anchors all frames to the head-acquired alignment + NOMINAL frame stride
	// (no Schmidl-Cox after frame 0). The SFO creep then walks the nominal window
	// off the true (drifted) content across the block — the tail must HOLD (the
	// per-frame pilot estimate + equalizer absorb it) or FAIL (the GO/NO-GO).
	bool bigblock = (env_i("MERCURY_SFO_BIGBLOCK", 0) != 0);
	// NEGATIVE CONTROL (faithfulness): MERCURY_SFO_BIGBLOCK_NOTRACK forces the RX
	// to anchor every tail frame to the head alignment WITHOUT advancing by the
	// per-frame SFO-creep that a real tracker/re-acquisition would apply. In the
	// default bigblock mode the nominal stride is exact for SFO=0 and only drifts
	// under SFO — i.e. the per-frame pilot estimate + equalizer are the ONLY thing
	// holding the tail. To PROVE the harness is not too lenient, NOTRACK injects a
	// fixed extra misalignment per frame (a few samples) that NO per-frame pilot
	// estimate can absorb, so the tail MUST fail. If even the default bigblock tail
	// holds at 50ppm but NOTRACK fails, the "holds" result is real discrimination.
	bool notrack_ctrl = (env_i("MERCURY_SFO_BIGBLOCK_NOTRACK", 0) != 0);
	// Per-frame extra misalignment (interp samples) injected in the NOTRACK control.
	int  notrack_skew = env_i("MERCURY_SFO_BIGBLOCK_NOTRACK_SKEW", 8);

	long   head_delay_interp = -1;   // acquired ONCE on frame 0 (bigblock mode)

	for(int f = 0; f < nFrames; f++)
	{
		st_receive_stats st;
		double metric;
		long   det;

		if(bigblock)
		{
			// Place the WHOLE contiguous drifted block once (frame 0), anchored so
			// the head preamble lands at symbol (full_pre+2) like the standard path.
			// Subsequent frames are NOT re-placed/re-acquired: we decode each frame
			// from the SAME window at the head-anchored NOMINAL stride.
			if(f == 0)
			{
				for(int i = 0; i < buf_interp; i++) data_container.passband_delayed_data[i] = 0.0;
				int copy_n = (int)(block_len);
				if(lead + copy_n > buf_interp) copy_n = buf_interp - lead;
				for(int i = 0; i < copy_n; i++)
				{
					double s = tx_block[(size_t)i];
					if(!clean && sigma > 0.0)
						s += (double)((sigma / sqrtf(2.0f)) * awgn_channel.awgn_value_generator());
					data_container.passband_delayed_data[lead + i] = s;
				}
				// ONE real acquisition on the head preamble (Schmidl-Cox + Moose).
				ofdm_forced_delay = -1;
				mfsk_fixed_delay  = -1;
				st = this->receive_byte(data_container.passband_delayed_data,
				                        data_container.hd_decoded_data_byte);
				head_delay_interp = st.delay;   // acquired head position (interp samples)
			}
			else
			{
				// NO re-acquisition: decode this frame at the head-anchored nominal
				// position. The block was TX'd as equal-length FULL frames, so the
				// nominal stride is `full_frame` interp-samples. The SFO has drifted
				// the real content by ~ppm*cumulative_samples; the receiver, having
				// only the head lock, does NOT know this drift -> the window is
				// progressively misaligned across the tail (the big-block timing
				// wall, if any).
				ofdm_forced_delay = head_delay_interp + (long)f * (long)full_frame;
				// NEGATIVE CONTROL: inject a cumulative per-frame misalignment that
				// no per-frame pilot estimate can absorb (proves the harness bites).
				if(notrack_ctrl)
					ofdm_forced_delay += (long)f * (long)notrack_skew;
				mfsk_fixed_delay  = -1;
				st = this->receive_byte(data_container.passband_delayed_data,
				                        data_container.hd_decoded_data_byte);
			}
			ofdm_forced_delay = -1;
			metric = st.coarse_metric;
			det    = st.delay;
			if(frame_pre[f] >= full_pre) { metric_sum_full += metric; n_full++; }
			else                         { metric_sum_mini += metric; n_mini++; }
			goto bigblock_score;
		}

		// Zero the RX window, then copy this frame's drifted samples after `lead`.
		for(int i = 0; i < buf_interp; i++) data_container.passband_delayed_data[i] = 0.0;

		{
		long src = frame_off[f];
		// Pull the frame plus a little of the NEXT frame's lead-in (so the data
		// tail + any SFO over-read is present); bounded by the block end.
		int copy_n = win_frame_max;
		if(src + copy_n > block_len) copy_n = (int)(block_len - src);
		for(int i = 0; i < copy_n && (lead + i) < buf_interp; i++)
		{
			double s = tx_block[(size_t)src + i];
			if(!clean && sigma > 0.0)
				s += (double)((sigma / sqrtf(2.0f)) * awgn_channel.awgn_value_generator());
			data_container.passband_delayed_data[lead + i] = s;
		}
		}

		// REAL acquisition: -1 ⇒ Schmidl-Cox time sync + Moose carrier sync run.
		ofdm_forced_delay = -1;
		mfsk_fixed_delay  = -1;
		// The RX MINI extraction (rx_eff_preamble=1) is gated behind the ARQ
		// batch-predict state machine, which this standalone harness does not run;
		// here every frame is acquired by the FULL-buffer Schmidl-Cox search, whose
		// integration length is set by the preamble actually present in the signal.
		// That is the physically correct discriminator: a frame TX'd with a 1-sym
		// preamble (MINI arm) presents a 4x-shorter half-symbol repeat to the SAME
		// search, so its metric under-integrates under SFO regardless of ARQ state.
		st = this->receive_byte(data_container.passband_delayed_data,
		                        data_container.hd_decoded_data_byte);
		ofdm_forced_delay = -1;
		metric = st.coarse_metric;
		det    = st.delay;
		if(frame_pre[f] >= full_pre) { metric_sum_full += metric; n_full++; }
		else                         { metric_sum_mini += metric; n_mini++; }

	bigblock_score:;

		// BER for this frame: decode bytes -> bits, compare to TX bits.
		byte_to_bit(data_container.hd_decoded_data_byte, data_container.hd_decoded_data_bit, payload_bytes);
		int errs = 0;
		for(int i = 0; i < payload_bits; i++)
			if(frame_bits[f][i] != data_container.hd_decoded_data_bit[i]) errs++;
		total_bit_errors += errs;
		total_bits       += payload_bits;
		if(errs == 0) frames_zero_ber++;
		if(st.message_decoded == YES) decoded_ok++;

		if(f < 8 || f >= nFrames - 4 || (f % 10) == 0)
		{
			std::cout << "[SFO-BLOCK] frame=" << f
			          << " pre=" << frame_pre[f]
			          << (bigblock ? " nominal_off=" : " expect_off=")
			          << (bigblock ? (head_delay_interp + (long)f * (long)full_frame) : (long)lead)
			          << (bigblock ? "(no-reacq)" : "(+drift)")
			          << " det=" << det
			          << " metric=" << metric
			          << " decoded=" << (st.message_decoded == YES ? 1 : 0)
			          << " biterr=" << errs << "/" << payload_bits << std::endl;
		}
	}

	double block_ber = (total_bits > 0) ? (double)total_bit_errors / (double)total_bits : 1.0;
	std::cout << "[SFO-BLOCK] ===== RESULT (" << (mini_arm ? "MINI" : "FULL") << " arm) ====="  << std::endl;
	std::cout << "[SFO-BLOCK]   frames_decoded=" << decoded_ok << "/" << nFrames
	          << "  frames_zero_ber=" << frames_zero_ber << "/" << nFrames << std::endl;
	std::cout << "[SFO-BLOCK]   block_BER=" << block_ber
	          << "  total_biterr=" << total_bit_errors << "/" << total_bits << std::endl;
	if(n_full > 0) std::cout << "[SFO-BLOCK]   mean_metric_FULLpre=" << (metric_sum_full / n_full) << std::endl;
	if(n_mini > 0) std::cout << "[SFO-BLOCK]   mean_metric_MINIpre=" << (metric_sum_mini / n_mini) << std::endl;
	std::cout << "[SFO-BLOCK]   tail-frame decode is the make-or-break: a clean control"
	          << " run (SFO OFF) MUST be ~all-decoded; SFO ON must DEGRADE the MINI tail." << std::endl;
}

// ============================================================================
// GENUINE single-grid big-block timing test. The sfo_block_test above tiles K
// independent 9-symbol codeword-frames (per-frame channel estimate); this builds
// ONE Nsymb-symbol grid carried by ONE channel estimate interpolated across the
// whole block (interpolate_bilinear_matrix, 33% pilots), which is the literal
// "one preamble + N data symbols, one estimate" the design asks about.
//
// SFO model (frequency-domain, exact): a sample-timing slip of tau samples at
// OFDM symbol n rotates subcarrier k by exp(-j 2*pi*k*tau_n / Nfft) — a phase RAMP
// in k whose slope GROWS with n (tau_n = ppm*1e-6 * n * (Nfft+Ngi)). This is the
// omega+k*delta per-symbol ramp the design names (Speth/Fechtel/Meyr 1999). The
// pilot lattice samples this ramp every Dy symbols; LS_channel_estimator's bilinear
// interpolation MAY already track it (STEP 1). MERCURY_SFO_GRID_TRACK=1 adds the
// STEP-2 anti-P CPE/PEG corrector: per symbol, LS line-fit pilot phase-error vs k
// (intercept=CPE, slope=PEG), de-rotate every subcarrier BEFORE the equalizer.
// ============================================================================

// ----------------------------------------------------------------------------
// SPARSE-CAPABLE 2D CHANNEL INTERPOLATOR (TEST 3 — the §13.5 production gap).
// ----------------------------------------------------------------------------
// Estimate a real per-subcarrier-per-symbol channel H[n][j] from the sparse 6%
// continual+scattered pilot lattice, on a frequency-SELECTIVE channel (|H| and
// phase varying across the 50 subcarriers even after the SFO ramp is removed).
// The stock per-cell-LS+DFT smoother (ofdm.cc LS_channel_estimator) assumes a
// DENSE REGULAR lattice and SMEARS a sparse one; the flat-ML H̄=mean(Y/X) shortcut
// (the TEST-2 path) collapses the whole band to one scalar and so cannot represent
// a selective channel. This is the DVB-T-style scattered-pilot SEPARABLE 2D
// interpolation VARA uses (Hoeher/Kaiser/Robertson, "Two-dimensional pilot-symbol-
// aided channel estimation by Wiener filtering", ICASSP 1997 — the canonical
// separable freq×time pilot interpolator):
//
//   (1) RAW LS at every pilot cell:  Hp[n][j] = rx[n][j] / X(pilot).
//   (2) TIME interpolation per carrier: for each carrier j that carries ≥1 pilot
//       in time (continual carriers: every symbol; scattered carriers: the scatter
//       symbols the diagonal lands on), fill ALL symbols n by linear interpolation
//       between consecutive time-pilots (hold at the edges), then OPTIONALLY apply a
//       short Wiener/MMSE moving-average time-smoother (MERCURY_SFO_GRID_WIENER=1)
//       to suppress pilot noise. This is the "time interpolation across the scatter
//       lattice's dy spacing" the design names.
//   (3) FREQUENCY interpolation per symbol: now every symbol has H known at the
//       carriers that received a time-estimate; linear-interpolate across carriers
//       to fill the interior data carriers (hold at the band edges). This is the
//       "frequency interpolation across the scatter+continual pilots within a
//       symbol's neighborhood" the design names. Together (2)+(3) are the separable
//       2D estimate.
//   (4) DDCE (MERCURY_SFO_GRID_DDCE=1, optional): one decision-directed pass —
//       equalize data cells with the (2)+(3) estimate, hard-decide the 32-QAM
//       symbol, treat the decision as an extra "pilot", and re-run a light freq
//       smooth. Refines H between scatter updates (the design's named DDCE).
//
// nv = pilot residual EVM against the FINAL interpolated H (NOT a global scalar) so
// it tracks the true noise and the continual columns keep ≥N pilot pairs → the
// TEST-2 nv that holds is preserved (no E1/cfg16-nvfix collapse).
void cl_telecom_system::grid_sparse2d_estimator(std::complex<double>* rx, int Ngrid, int Nc)
{
	auto env_i = [](const char* k, int def){ const char* e=std::getenv(k); return (e&&*e)?atoi(e):def; };
	bool wiener = (env_i("MERCURY_SFO_GRID_WIENER", 1) != 0);   // default Wiener time-smooth ON
	// DDCE default OFF (env-overridable for A/B). NOTE 2026-06-07 (ddce_finalize): a blanket
	// DDCE-default-ON was ATTEMPTED and REVERTED — it crosses the deterministic-floor 32-QAM
	// cliff on the single-block chanest decode (phaseRMS 0.124->0.100, bytes_ok 0->1) but
	// REGRESSES the clean-channel LIVE big-block path: --test-bigblock-multicw drops 8/8->7/8 +
	// ARM-C byte-faithful->corrupt, and --test-bigblock-fullpath partial-block GATE-HANGS (the
	// exactly-one-codeword-of-margin signature documented at arq_common.cc:4062-4067). The
	// data-cell decision-EVM does NOT separate the helped case (single-block decode) from the
	// hurt case (live 2-instance decode): BOTH run the always-on Schroeder floor at dataEVM
	// ~0.0137-0.015, so DDCE's benefit is decode-PATH-dependent, not channel-dependent — no
	// simple measured gate separates them. Conditional DDCE needs an estimator-design pass
	// (per-codeword convergence-aware DDCE, or restricting it to bigblock_rx_passband and not
	// the live receive_bigblock carve). Surfaced as path+cost; NOT shipped as a blind default.
	bool ddce   = (env_i("MERCURY_SFO_GRID_DDCE",   0) != 0);
	int  wlen   = env_i("MERCURY_SFO_GRID_WIENER_LEN", 5);      // moving-avg half-window (taps=2*L+1)

	const int NG = Ngrid, NC = Nc;
	std::vector<std::complex<double>> H((size_t)NG*NC, std::complex<double>(0,0));
	std::vector<char> known((size_t)NG*NC, 0);   // 1 = has a usable estimate

	// (1) RAW LS at every pilot. pilot_configurator.sequence is indexed in
	//     (symbol,carrier) raster order over PILOT cells (the framer convention).
	{
		int pidx = 0;
		for(int n=0;n<NG;n++) for(int j=0;j<NC;j++)
			if((ofdm.ofdm_frame+n*NC+j)->type==PILOT)
			{
				std::complex<double> X = ofdm.pilot_configurator.sequence[pidx++];
				H[(size_t)n*NC+j] = rx[(size_t)n*NC+j] / X;
				known[(size_t)n*NC+j] = 1;
			}
	}

	// (1b) PER-SYMBOL COMMON-PHASE (CPE) de-rotation from the CONTINUAL columns
	//      (802.11 continual-pilot CPE; FreeDV-700D pilot-assisted-coherent). The
	//      CONTINUAL carriers carry a pilot at EVERY symbol, so they DIRECTLY observe
	//      the per-symbol common rotation the channel applies (residual CFO/SFO drift
	//      + slow time-variation), the dominant un-tracked term: on the HW-faithful
	//      vector the per-symbol common phase walks NON-LINEARLY by ~2.16 rad over the
	//      block (measured [AFC-PILTHETA] resid_after_ramp), which the AFC tracker's
	//      linear-ramp model and the sparse scattered TIME-interp (pilots every ~Dy
	//      symbols) cannot follow. By measuring cpe[n] from the dense continual columns
	//      and de-rotating the WHOLE pilot grid by it BEFORE interpolation, the
	//      time/freq interpolation only has to track the SLOWLY-varying per-carrier
	//      residual (which sparse pilots CAN follow); cpe[n] is then re-applied to the
	//      final per-cell H so the published estimate carries the true per-symbol phase
	//      the equalizer divides out. cpe[n] is referenced to EACH continual carrier's
	//      OWN time-mean phase, so a static per-carrier channel phase is NOT folded into
	//      the common term (only the time-VARYING common rotation is captured). On a
	//      time-invariant channel (per-frame regime) cpe[n]≈0 ⇒ no-op (per-frame path
	//      does not call this estimator anyway; see SS5 audit §16). MERCURY_SFO_GRID_CPE
	//      default ON; needs ≥1 continual column (a carrier pilot-known at every symbol).
	std::vector<double> cpe(NG, 0.0);
	bool cpe_on = (env_i("MERCURY_SFO_GRID_CPE", 1) != 0);
	if(cpe_on)
	{
		// continual carriers = those with a pilot at EVERY symbol
		std::vector<int> ccols;
		for(int j=0;j<NC;j++){ bool all=true;
			for(int n=0;n<NG;n++) if(!known[(size_t)n*NC+j]){ all=false; break; }
			if(all) ccols.push_back(j); }
		if(!ccols.empty())
		{
			// per continual carrier, its time-mean H (the static reference phase)
			std::vector<std::complex<double>> cref(ccols.size(), std::complex<double>(0,0));
			for(size_t c=0;c<ccols.size();c++){ std::complex<double> a(0,0);
				for(int n=0;n<NG;n++) a += H[(size_t)n*NC+ccols[c]];
				cref[c] = a/(double)NG; }
			// per symbol, common rotation = arg(Σ_c H[n,col_c]·conj(cref_c)) (the phasor
			// rotating each continual carrier off its own time-mean, magnitude-weighted).
			for(int n=0;n<NG;n++){
				std::complex<double> acc(0,0);
				for(size_t c=0;c<ccols.size();c++)
					acc += H[(size_t)n*NC+ccols[c]] * std::conj(cref[c]);
				cpe[n] = (std::abs(acc)>1e-18) ? std::arg(acc) : 0.0;
			}
			// de-rotate the whole RAW pilot grid by cpe[n] (interp now tracks only the
			// slow per-carrier residual; cpe re-applied after step (3)).
			for(int n=0;n<NG;n++){
				std::complex<double> r = std::polar(1.0, -cpe[n]);
				for(int j=0;j<NC;j++) if(known[(size_t)n*NC+j]) H[(size_t)n*NC+j] *= r;
			}
		}
	}

	// (2) TIME interpolation per carrier (symbol axis), then optional Wiener smooth.
	//
	// CRITICAL (fix/bigblock-chanest §17): interpolate AND smooth the TIME axis in
	// POLAR form (magnitude + UNWRAPPED phase), mirroring step (3)'s frequency-axis
	// polar interp. The block-fold defect was here: a slowly time-varying channel
	// (det-floor dispersion + residual CFO/SFO walk) rotates each carrier's pilot
	// phase across the 60-symbol block (measured genie col_phaseRMS = 0.83 rad on the
	// HW-faithful vector). Complex-LINEAR time interpolation cuts a CHORD across that
	// rotation (shrinking |H| and losing the ramp), and the complex moving-average
	// Wiener smooth (±wlen) destructively AVERAGES the walking phasor — exactly the
	// fold that drove mean|H| -> 0 and bytes_ok=0. Polar interp/smooth tracks the
	// per-symbol phase ramp instead of folding it, so the per-cell H the equalizer
	// divides by (out=in/H) captures each cell's TRUE per-symbol phase (the invariant
	// channel_equalizer already assumes — SS5 audit §16). When the per-symbol step is
	// small (the per-frame WB/NB regime: a re-acquired ~12-sym frame has negligible
	// phase walk) polar == complex-linear to first order, so the shared per-frame path
	// is preserved (A/B verified). Std two-1D scattered-pilot polar interpolation
	// (Hoeher/Kaiser/Robertson 1997); per-symbol pilot phase tracking is the
	// FreeDV-700D / 802.11 continual-pilot recipe.
	bool time_polar = (env_i("MERCURY_SFO_GRID_TIME_POLAR", 1) != 0);   // default polar time axis ON
	for(int j=0;j<NC;j++)
	{
		// collect this carrier's time-pilot symbol indices
		std::vector<int> ts;
		for(int n=0;n<NG;n++) if(known[(size_t)n*NC+j]) ts.push_back(n);
		if(ts.empty()) continue;                      // no time samples — handled by (3)
		// interpolate between consecutive time-pilots (polar: track the phase ramp)
		for(size_t s=0;s+1<ts.size();s++)
		{
			int n0=ts[s], n1=ts[s+1];
			std::complex<double> H0=H[(size_t)n0*NC+j], H1=H[(size_t)n1*NC+j];
			if(time_polar)
			{
				double m0=std::abs(H0), m1=std::abs(H1);
				double p0=std::arg(H0), p1=std::arg(H1);
				double dp=p1-p0;                         // shortest-rotation unwrap
				while(dp> M_PI) dp-=2.0*M_PI;
				while(dp<-M_PI) dp+=2.0*M_PI;
				for(int n=n0+1;n<n1;n++)
				{
					double t = (double)(n-n0)/(double)(n1-n0);
					double m = m0*(1.0-t)+m1*t;
					double p = p0+dp*t;
					H[(size_t)n*NC+j] = std::polar(m,p);
					known[(size_t)n*NC+j] = 1;
				}
			}
			else
			{
				for(int n=n0+1;n<n1;n++)
				{
					double t = (double)(n-n0)/(double)(n1-n0);
					H[(size_t)n*NC+j] = H0*(1.0-t) + H1*t;
					known[(size_t)n*NC+j] = 1;
				}
			}
		}
		// edge hold (extrapolate flat past the first/last time-pilot)
		for(int n=0;n<ts.front();n++){ H[(size_t)n*NC+j]=H[(size_t)ts.front()*NC+j]; known[(size_t)n*NC+j]=1; }
		for(int n=ts.back()+1;n<NG;n++){ H[(size_t)n*NC+j]=H[(size_t)ts.back()*NC+j]; known[(size_t)n*NC+j]=1; }
		// Wiener/MMSE time-smoother: short centered moving average over the now-dense
		// column (suppresses pilot noise). Smooth ONLY columns that had ≥2 time-pilots
		// (a single-pilot scattered carrier has nothing to average and would just blur
		// the freq-interp seed). POLAR smooth (magnitude + UNWRAPPED phase separately):
		// a complex moving-average of a per-symbol phase RAMP shrinks the magnitude and
		// flattens the ramp (the §3.3/§9 "averaging a ramp is harmful" fold); smoothing
		// the unwrapped phase preserves the ramp's slope while still suppressing pilot
		// noise. On a time-INVARIANT channel (per-frame regime) the unwrapped phase is
		// flat, so the polar smooth == the complex smooth (per-frame preserved).
		//
		// CONTINUAL columns (a pilot at EVERY symbol, ts.size()==NG) DIRECTLY observe the
		// per-symbol channel phase — do NOT time-smooth them (§17 step 3): an 11-tap
		// moving average over a 0.83-rad/60-sym ramp blurs the per-symbol CPE the decode
		// needs. Smoothing is for noise suppression on the INTERPOLATED scattered columns
		// (sparse time-pilots), where the interpolated phase is already a straight line
		// that benefits from de-noising without losing a per-symbol observation. Gate via
		// MERCURY_SFO_GRID_SMOOTH_CONT (default 0 = skip smoothing continual columns).
		bool is_continual = ((int)ts.size() == NG);   // pilot at every symbol → continual
		bool smooth_cont  = (env_i("MERCURY_SFO_GRID_SMOOTH_CONT", 0) != 0);
		bool do_smooth    = wiener && ts.size()>=2 && wlen>0 && (!is_continual || smooth_cont);
		if(do_smooth)
		{
			std::vector<std::complex<double>> col(NG);
			for(int n=0;n<NG;n++) col[n]=H[(size_t)n*NC+j];
			if(time_polar)
			{
				// build a CONTIGUOUS unwrapped-phase track over the dense column
				std::vector<double> mag(NG), ph(NG);
				double acc_ph = std::arg(col[0]); mag[0]=std::abs(col[0]); ph[0]=acc_ph;
				double prev = std::arg(col[0]);
				for(int n=1;n<NG;n++){
					double a=std::arg(col[n]); double d=a-prev;
					while(d> M_PI) d-=2.0*M_PI;
					while(d<-M_PI) d+=2.0*M_PI;
					acc_ph += d; prev = a;
					mag[n]=std::abs(col[n]); ph[n]=acc_ph;
				}
				for(int n=0;n<NG;n++)
				{
					double ms=0.0, ps=0.0; int cnt=0;
					for(int w=n-wlen; w<=n+wlen; w++) if(w>=0&&w<NG){ ms+=mag[w]; ps+=ph[w]; cnt++; }
					if(cnt>0) H[(size_t)n*NC+j]=std::polar(ms/(double)cnt, ps/(double)cnt);
				}
			}
			else
			{
				for(int n=0;n<NG;n++)
				{
					std::complex<double> acc(0,0); int cnt=0;
					for(int w=n-wlen; w<=n+wlen; w++) if(w>=0&&w<NG){ acc+=col[w]; cnt++; }
					if(cnt>0) H[(size_t)n*NC+j]=acc/(double)cnt;
				}
			}
		}
	}

	// (3) FREQUENCY interpolation per symbol (carrier axis). After (2), the carriers
	//     that carry ANY pilot in time are "known" at every symbol; interpolate across
	//     carriers to fill the interior data carriers, hold at the band edges.
	//
	// CRITICAL: interpolate in POLAR form (magnitude + UNWRAPPED phase), NOT in the
	// complex plane. A frequency-selective channel (esp. the det-floor all-pass) rotates
	// its phase by up to ±π across the band; if the phase between two adjacent pilots
	// exceeds π, complex-linear interpolation cuts a CHORD through the origin and the
	// interpolated |H| collapses toward 0 (measured: |H| min 0.016 on the all-pass) →
	// the MMSE erasure drops whole bands → BER ~0.35. Unwrapping the phase (pick the
	// shortest rotation each pilot step) and interpolating |H| and φ separately keeps the
	// estimate ON the channel's actual locus. This is the standard polar/Wiener pilot
	// interpolation (Hoeher/Kaiser/Robertson 1997). Note: if the TRUE phase rotation
	// between two consecutive freq-pilots exceeds π the channel is UNDERSAMPLED in
	// frequency (aliasing) — no interpolator can recover it; that sets the pilot-density
	// floor reported by the density sweep.
	bool polar = (env_i("MERCURY_SFO_GRID_POLAR", 1) != 0);   // default polar interp ON
	for(int n=0;n<NG;n++)
	{
		std::vector<int> ks;
		for(int j=0;j<NC;j++) if(known[(size_t)n*NC+j]) ks.push_back(j);
		if(ks.empty()){ for(int j=0;j<NC;j++){ H[(size_t)n*NC+j]=std::complex<double>(1,0); } continue; }
		for(size_t s=0;s+1<ks.size();s++)
		{
			int j0=ks[s], j1=ks[s+1];
			std::complex<double> H0=H[(size_t)n*NC+j0], H1=H[(size_t)n*NC+j1];
			if(polar)
			{
				double m0=std::abs(H0), m1=std::abs(H1);
				double p0=std::arg(H0), p1=std::arg(H1);
				double dp=p1-p0;                        // shortest-rotation unwrap
				while(dp> M_PI) dp-=2.0*M_PI;
				while(dp<-M_PI) dp+=2.0*M_PI;
				for(int j=j0+1;j<j1;j++)
				{
					double t=(double)(j-j0)/(double)(j1-j0);
					double m=m0*(1.0-t)+m1*t;
					double p=p0+dp*t;
					H[(size_t)n*NC+j]=std::polar(m,p);
				}
			}
			else
			{
				for(int j=j0+1;j<j1;j++)
				{
					double t=(double)(j-j0)/(double)(j1-j0);
					H[(size_t)n*NC+j]=H0*(1.0-t)+H1*t;
				}
			}
		}
		for(int j=0;j<ks.front();j++) H[(size_t)n*NC+j]=H[(size_t)n*NC+ks.front()];
		for(int j=ks.back()+1;j<NC;j++) H[(size_t)n*NC+j]=H[(size_t)n*NC+ks.back()];
	}

	// (3b) RE-APPLY the per-symbol CPE removed at (1b): the interpolation tracked the
	//      slow per-carrier residual on the common-phase-stabilized grid; re-rotating by
	//      +cpe[n] restores the TRUE per-symbol channel phase into every cell so the
	//      equalizer's out=in/H divides out the actual per-symbol rotation.
	if(cpe_on)
	{
		for(int n=0;n<NG;n++){
			if(cpe[n]==0.0) continue;
			std::complex<double> r = std::polar(1.0, +cpe[n]);
			for(int j=0;j<NC;j++) H[(size_t)n*NC+j] *= r;
		}
	}

	// (4) DDCE (optional): one decision-directed refinement pass. Equalize data cells
	//     with the (2)+(3) estimate, hard-decide the 32-QAM constellation point, then
	//     re-estimate H_data = Y/decision and blend a freq-smoothed version back in.
	if(ddce)
	{
		std::vector<std::complex<double>> Hd((size_t)NG*NC);
		for(int n=0;n<NG;n++) for(int j=0;j<NC;j++)
		{
			std::complex<double> Hcur=H[(size_t)n*NC+j];
			if((ofdm.ofdm_frame+n*NC+j)->type==PILOT){ Hd[(size_t)n*NC+j]=Hcur; continue; }
			std::complex<double> eq = (std::norm(Hcur)>1e-12) ? rx[(size_t)n*NC+j]/Hcur : std::complex<double>(0,0);
			std::complex<double> dec = psk.slice_nearest(eq);   // nearest 32-QAM point
			Hd[(size_t)n*NC+j] = (std::norm(dec)>1e-12) ? rx[(size_t)n*NC+j]/dec : Hcur;
		}
		// light freq smooth of the decision-directed estimate, blended 50/50 with the
		// pilot estimate (keeps the pilot-anchored truth dominant; DDCE only nudges).
		for(int n=0;n<NG;n++)
		{
			for(int j=0;j<NC;j++)
			{
				std::complex<double> acc(0,0); int cnt=0;
				for(int w=j-1;w<=j+1;w++) if(w>=0&&w<NC){ acc+=Hd[(size_t)n*NC+w]; cnt++; }
				std::complex<double> sm=(cnt>0)?acc/(double)cnt:Hd[(size_t)n*NC+j];
				if((ofdm.ofdm_frame+n*NC+j)->type!=PILOT)
					H[(size_t)n*NC+j] = 0.5*H[(size_t)n*NC+j] + 0.5*sm;
			}
		}
	}

	// Publish to estimated_channel (MEASURED everywhere → channel_equalizer uses it).
	for(int n=0;n<NG;n++) for(int j=0;j<NC;j++)
	{
		(ofdm.estimated_channel+n*NC+j)->value  = H[(size_t)n*NC+j];
		(ofdm.estimated_channel+n*NC+j)->status = MEASURED;
	}

	// nv = pilot residual EVM against the FINAL interpolated H (NOT a global scalar):
	// resid = Y - H[pilot]*X over every pilot cell. The continual columns alone give
	// 2*Ngrid pilot pairs, so this never collapses to the 1e-6 floor (preserves the
	// TEST-2 nv that holds — no E1/cfg16-nvfix over-confident-LLR collapse).
	{
		double nsum=0.0; int pidx=0, npil=0;
		for(int n=0;n<NG;n++) for(int j=0;j<NC;j++)
			if((ofdm.ofdm_frame+n*NC+j)->type==PILOT)
			{
				std::complex<double> X = ofdm.pilot_configurator.sequence[pidx++];
				std::complex<double> resid = rx[(size_t)n*NC+j] - H[(size_t)n*NC+j]*X;
				nsum += resid.real()*resid.real() + resid.imag()*resid.imag();
				npil++;
			}
		ofdm.noise_variance_estimate = (npil>0) ? (nsum/(double)npil) : 0.01;
		if(ofdm.noise_variance_estimate < 1e-6) ofdm.noise_variance_estimate = 1e-6;
	}
}

void cl_telecom_system::sfo_grid_test()
{
	if(M == MOD_MFSK){ std::cout << "[SFO-GRID] MFSK unsupported; use -s 15/16." << std::endl; return; }

	auto env_i = [](const char* k, int def){ const char* e=std::getenv(k); return (e&&*e)?atoi(e):def; };
	auto env_f = [](const char* k, double def){ const char* e=std::getenv(k); return (e&&*e)?atof(e):def; };

	int    Ngrid   = env_i("MERCURY_SFO_GRID_NSYMB", 60);   // data symbols in the ONE grid
	double ppm     = env_f("MERCURY_SIM2_SFO_PPM", 0.0);
	bool   track   = (env_i("MERCURY_SFO_GRID_TRACK", 0) != 0);     // STEP 2: CPE/PEG corrector
	bool   no_interp = (env_i("MERCURY_SFO_GRID_NOINTERP", 0) != 0);// negative control
	int    win      = env_i("MERCURY_SFO_GRID_TRACK_WIN", 9);       // CPE/PEG sliding window
	uint64_t seed   = (uint64_t)env_i("MERCURY_SFO_GRID_SEED", 12345);

	// --- Rebuild the OFDM grid at Nsymb=Ngrid (32QAM, 33% pilots Dx=1/Dy=3). ---
	int saved_Nsymb = ofdm.Nsymb;
	int Nc    = ofdm.Nc;
	int Nfft  = ofdm.Nfft;
	float gi  = ofdm.gi;
	int Ngi   = (int)round((double)gi * (double)Nfft);
	int Nofdm = Nfft + Ngi;
	// Preserve the preamble_configurator fields that ofdm.deinit() zeros but the
	// 4-arg ofdm.init() does NOT restore (Nsymb/modulation/nIdentical_sections/
	// boost). Without this, preamble_configurator::init divides nPreamble/Nsymb=0
	// (ofdm.cc:1304) -> crash. We only change the DATA grid (Nsymb), not the
	// preamble, so carry the preamble config across the rebuild verbatim.
	int   sav_pre_Nsymb = ofdm.preamble_configurator.Nsymb;
	int   sav_pre_mod   = ofdm.preamble_configurator.modulation;
	int   sav_pre_nIS   = ofdm.preamble_configurator.nIdentical_sections;
	double sav_pre_boost= ofdm.preamble_configurator.boost;
	int   sav_start_shift = ofdm.start_shift;
	// Pilot configurator: deinit() zeros boost (->1.33) which makes the pilot
	// sequence all-zero -> H=Y/0=NaN -> equalizer erases everything. Preserve the
	// pilot boost/modulation/seed too.
	double sav_pil_boost = ofdm.pilot_configurator.boost;
	int   sav_pil_mod    = ofdm.pilot_configurator.modulation;
	int   sav_pil_seed   = ofdm.pilot_configurator.seed;
	ofdm.deinit();
	ofdm.start_shift = sav_start_shift;
	ofdm.preamble_configurator.Nsymb = sav_pre_Nsymb;
	ofdm.preamble_configurator.modulation = sav_pre_mod;
	ofdm.preamble_configurator.nIdentical_sections = sav_pre_nIS;
	ofdm.preamble_configurator.boost = sav_pre_boost;
	ofdm.pilot_configurator.boost = sav_pil_boost;
	ofdm.pilot_configurator.modulation = sav_pil_mod;
	ofdm.pilot_configurator.seed = sav_pil_seed;
	ofdm.pilot_configurator.Dx = 1; ofdm.pilot_configurator.Dy = 3;   // 33% pilots
	ofdm.pilot_configurator.pilot_density = HIGH_DENSITY;
	ofdm.channel_estimator = LEAST_SQUARE;
	ofdm.channel_estimator_amplitude_restoration = NO;
	// MERCURY_SFO_GRID_TRACK_PROD=1 (fade-map commit 7d5c114): engage the GENUINE
	// PRODUCTION LS tracker window (default_configurations_telecom_system
	// ofdm_LS_window_width=2 / ofdm_LS_window_hight=8, odd-bumped exactly as
	// set_configs does) instead of 0/0 -> full-grid. The localized window slides
	// with the channel symbol-by-symbol so the estimate TRACKS a time-varying fade
	// the way the real RX decodes (full-grid emits one held estimate that cannot
	// follow the fade -> measurement artifact). Reused verbatim from the fade-map
	// branch (sim/sfo-grid-prod-tracker) so this is the SAME tracker, not a bespoke
	// corrector. Byte-identical when unset (else-branch = original 0/0).
	if(env_i("MERCURY_SFO_GRID_TRACK_PROD", 0) != 0)
	{
		int pw = default_configurations_telecom_system.ofdm_LS_window_width;
		int ph = default_configurations_telecom_system.ofdm_LS_window_hight;
		if(pw % 2 == 0) pw++;   // LS window must be odd (symmetric i +/- win/2),
		if(ph % 2 == 0) ph++;   // mirroring the production odd-bump in set_configs.
		ofdm.LS_window_width = pw; ofdm.LS_window_hight = ph;  // production tracker
	}
	else
	{
		ofdm.LS_window_width = 0; ofdm.LS_window_hight = 0;   // re-derive in init
	}
	ofdm.init(Nfft, Nc, Ngrid, gi);   // sets Nfft/Nc/Nsymb/gi/Ngi + allocs + configures lattice

	// ====================================================================
	// TEST 2: PILOT THINNING 33% -> ~6% (CONTINUAL + SCATTERED lattice).
	// ====================================================================
	// The net-PHY-recovery step. The default configure() lays Dy=3 FULL columns
	// (every column gets 1/3 of its rows = 33% pilots). To raise the data-carrying
	// fraction we OVERWRITE the lattice in-harness (NOT the production configure(),
	// which would re-derive nData->nBits and break CFG16's 1600-bit codeword sizing
	// across all 17 configs) with the design's CONTINUAL + SCATTERED layout:
	//
	//   CONTINUAL columns  : a few carriers carrying a pilot on EVERY symbol. These
	//                        anchor the CPE/PEG per-symbol LS phase fit (omega+k*delta
	//                        needs >=2 pilots PER symbol) AND feed the LS-path residual
	//                        nv estimator (every pilot counted, ofdm.cc:1816) so it
	//                        cannot collapse to the 1e-6 floor (the E1/cfg16-nvfix
	//                        sibling bug).
	//   SCATTERED lattice  : interior pilots every SCAT_DX carriers, repeating every
	//                        SCAT_DY symbols, offset by symbol -> the 2D samples the
	//                        channel estimate needs without spending a full column.
	//
	// Because LS_window=(Nc x Nsymb) (full grid), LS_channel_estimator gives EVERY
	// data cell the global LS fit over ALL pilots (ofdm.cc:1684-1745) and the
	// column/bilinear interpolation is a no-op (all cells already MEASURED) -> the
	// sparse layout needs NO interpolation rewrite. After the tracker removes the SFO
	// ramp the channel is flat-unity, so the global LS fit is near-exact.
	bool   thin      = (env_i("MERCURY_SFO_GRID_THIN", 0) != 0);
	int    cont_cols = env_i("MERCURY_SFO_GRID_CONT_COLS", 3);   // continual pilot columns
	int    scat_dx   = env_i("MERCURY_SFO_GRID_SCAT_DX", 12);    // scatter carrier step
	int    scat_dy   = env_i("MERCURY_SFO_GRID_SCAT_DY", 4);     // scatter symbol step
	if(thin)
	{
		// 1) Everything DATA.
		for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
			(ofdm.ofdm_frame+n*Nc+j)->type = DATA;

		// 2) CONTINUAL columns: cont_cols carriers spread across [0,Nc-1] incl. edges.
		//    carrier_c = round(c*(Nc-1)/(cont_cols-1)). Pilot on every symbol.
		std::vector<int> cont(cont_cols);
		for(int c=0;c<cont_cols;c++)
			cont[c] = (cont_cols<=1) ? 0
			        : (int)llround((double)c*(double)(Nc-1)/(double)(cont_cols-1));
		for(int c=0;c<cont_cols;c++)
			for(int n=0;n<Ngrid;n++)
				(ofdm.ofdm_frame+n*Nc+cont[c])->type = PILOT;

		// 3) SCATTERED lattice: at symbols n%scat_dy==0, place interior pilots every
		//    scat_dx carriers, offset by (n/scat_dy) so the diagonal walks across the
		//    band over successive scatter-rows (full 2D coverage for the channel est).
		if(scat_dx > 0 && scat_dy > 0)
		{
			for(int n=0;n<Ngrid;n++)
			{
				if(n % scat_dy != 0) continue;
				int off = (n/scat_dy) * (scat_dx/2 > 0 ? scat_dx/2 : 1);
				for(int j = off % scat_dx; j < Nc; j += scat_dx)
					(ofdm.ofdm_frame+n*Nc+j)->type = PILOT;
			}
		}

		// 4) Recount nPilots/nData. The framer/deframer/LS_channel_estimator and the
		//    CPE/PEG tracker ALL read ofdm.ofdm_frame[].type (NOT virtual_carrier, which
		//    is only consumed inside configure(), bypassed here). pilot_configurator.
		//    carrier IS ofdm_frame (same buffer, ofdm.cc:175), so the lattice we wrote
		//    above is already what every consumer sees. nData = Nc*Nsymb - nPilots,
		//    matching configure()'s convention.
		int np=0;
		for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
			if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT) np++;
		ofdm.pilot_configurator.nPilots = np;
		ofdm.pilot_configurator.nConfig = 0;
		ofdm.pilot_configurator.nData   = Ngrid*Nc - np;

		// 5) Re-allocate + re-seed the pilot DBPSK sequence to the NEW nPilots. The
		//    framer/estimator/tracker all index sequence[] by running pilot count in
		//    raster order, so it MUST have exactly nPilots entries.
		CDELETE(ofdm.pilot_configurator.sequence);
		ofdm.pilot_configurator.sequence =
		    CNEW(std::complex<double>, np, "pilot.sequence.thin");
		__srandom(ofdm.pilot_configurator.seed);
		int last_pilot=0;
		for(int i=0;i<np;i++)
		{
			int pv = (__random()%2) ^ last_pilot;   // DBPSK, same generator as init()
			ofdm.pilot_configurator.sequence[i] =
			    std::complex<double>(2*pv-1,0) * ofdm.pilot_configurator.boost;
			last_pilot = pv;
		}
	}

	int nData = ofdm.pilot_configurator.nData;     // data carriers across the 60-grid
	int log2M = (int)round(log2((double)M));
	int nBits = nData * log2M;

	std::cout << "[SFO-GRID] cfg=" << current_configuration << " M=" << M
	          << " Nsymb=" << Ngrid << " Nc=" << Nc
	          << (thin ? " THIN(cont+scatter)" : " Dx=1 Dy=3")
	          << (thin ? " cont_cols=" : " ") << (thin ? cont_cols : 0)
	          << (thin ? " scat_dx=" : "") << (thin ? scat_dx : 0)
	          << (thin ? " scat_dy=" : "") << (thin ? scat_dy : 0)
	          << " nData=" << nData << " nBits=" << nBits
	          << " pilots=" << ofdm.pilot_configurator.nPilots
	          << " (" << (100.0*ofdm.pilot_configurator.nPilots/(double)(Ngrid*Nc)) << "% of grid)"
	          << " ppm=" << ppm << " track=" << (track?1:0)
	          << " no_interp=" << (no_interp?1:0) << std::endl;

	// --- TX: data symbols + known pilots into the 60-grid. ---
	// Two modes:
	//  - UNCODED (default): nBits random bits (requirement (a), the timing/BER probe).
	//  - CODED (MERCURY_SFO_GRID_CODED=1): fill the FIRST K*N bits with K LDPC
	//    codewords (K = floor(nBits / ldpc.N)), each = [1400 info | 200 parity]
	//    (systematic, ldpc.cc:107-120), so the decode is the REAL rate-0.875 LDPC
	//    block-decode at the achieved pilot fraction. This is the "coded K-codeword
	//    path at 6% pilots + tracker, under SFO" the GO/NO-GO names. AWGN is added in
	//    the channel stage when MERCURY_SFO_GRID_ESN0 < 900 so noise_variance_estimate
	//    is a meaningful quantity to check for the E1/cfg16-nvfix collapse.
	bool coded = (env_i("MERCURY_SFO_GRID_CODED", 0) != 0);
	int  Kcw   = coded ? (nBits / ldpc.N) : 0;     // whole codewords that fit
	std::vector<int> tx_bits(nBits);
	std::vector<std::vector<int>> cw_info(Kcw);    // per-codeword info bits (for BER/CRC)
	ts_srandom((unsigned int)seed);
	if(coded)
	{
		std::vector<int> enc(ldpc.N);
		std::vector<int> info(ldpc.K);
		for(int c=0;c<Kcw;c++)
		{
			for(int i=0;i<ldpc.K;i++) info[i] = (int)(ts_random()%2);
			cw_info[c] = info;
			ldpc.encode(info.data(), enc.data());   // enc = [K info | P parity] = N bits
			for(int i=0;i<ldpc.N;i++) tx_bits[(size_t)c*ldpc.N + i] = enc[i];
		}
		// Any leftover bits past K*N are random filler (not scored).
		for(int i=Kcw*ldpc.N; i<nBits; i++) tx_bits[i] = (int)(ts_random()%2);
	}
	else
	{
		for(int i=0;i<nBits;i++) tx_bits[i] = (int)(ts_random()%2);
	}

	std::vector<std::complex<double>> tx_syms(nData);
	psk.mod(tx_bits.data(), nBits, tx_syms.data());

	std::vector<std::complex<double>> grid((size_t)Ngrid*Nc);
	ofdm.framer(tx_syms.data(), grid.data());   // data + pilots placed by lattice

	// --- CHANNEL: flat unity H + the SFO per-symbol subcarrier phase ramp. ---
	// tau_n = accumulated timing slip (samples) at symbol n; phase(k) = -2*pi*k*tau_n/Nfft.
	// k index runs over the active subcarriers as the modulator places them
	// (zero_padder maps logical carrier j to FFT bin; the ramp is in the SAME
	// logical-carrier index the estimator/equalizer use, so a per-carrier index j
	// suffices — the bin mapping is a fixed permutation that cancels in est/eq).
	for(int n=0; n<Ngrid; n++)
	{
		double tau_n = ppm*1e-6 * (double)n * (double)Nofdm;   // samples slipped by symbol n
		for(int j=0;j<Nc;j++)
		{
			double ph = -2.0*M_PI*(double)j*tau_n/(double)Nfft;
			grid[(size_t)n*Nc+j] *= std::complex<double>(cos(ph), sin(ph));
		}
	}

	// --- FREQUENCY-SELECTIVE CHANNEL (the §13.5 production gap) -----------------
	// TEST-2 ran a FLAT unity channel: after the tracker removes the SFO ramp the
	// residual is |H|=1 everywhere, so the flat-ML H=mean(Y/X) estimate is exact.
	// A REAL HF/soundcard channel is frequency-SELECTIVE: |H| and phase VARY across
	// the 50 subcarriers even after the SFO timing ramp is removed, so the equalizer
	// needs a real per-subcarrier H interpolated from the sparse 6% pilots. This block
	// injects that selective channel as a per-carrier complex coefficient T(j) (the
	// channel is time-invariant across the block — frequency-selective only — so the
	// same T(j) multiplies every symbol; the SFO ramp above sits ON TOP of it).
	//
	// Two faithful selective models (gated by MERCURY_SFO_GRID_CHAN):
	//   1 = DET-FLOOR (phase-dispersive, |T|=1): the SHIPPED Schroeder all-pass
	//       cl_sim_det_floor (sim_channel.h:750, g=0.50/D=16/N=3). The frequency
	//       response of ONE all-pass section is the closed form
	//         A(e^{jw}) = (-g + e^{-jwD}) / (1 - g·e^{-jwD})   (|A|≡1, dispersive φ),
	//       cascaded ap_n times. |T|=1 (no amplitude null → 32-QAM tolerant) but the
	//       per-subcarrier PHASE varies → the flat-ML estimate (one global H̄) cannot
	//       represent it. This is the EVM-ceiling model the §10 bench shipped.
	//   2 = TWO-RAY (magnitude-selective): the fsel_test model (telecom_system.h:324,
	//       amp 0.6 @ delay 128 passband samples), T(j) = 1 + a·e^{-jw_j·Δ}, unit-power
	//       normalized. |T| FADES across the band (deep nulls) — the HARDER case the
	//       sparse interpolator must handle (it stresses BOTH magnitude and phase).
	// w_j is the TRUE centered FFT-bin angular frequency of logical carrier j (the
	// zero_padder mapping, ofdm.cc:331/697-723) so the ripple period is physical:
	//   k_centered(j) = (j < Nc/2) ? (j - Nc/2) : (j - Nc/2 + start_shift)
	//   w_j = 2*pi*k_centered / Nfft.
	int    chan_sel  = env_i("MERCURY_SFO_GRID_CHAN", 0);          // 0=flat,1=det-floor,2=two-ray
	double ap_g      = env_f("MERCURY_SFO_GRID_AP_G",   0.50);     // all-pass coeff (SHIPPED 0.50)
	int    ap_dly    = env_i("MERCURY_SFO_GRID_AP_DLY", 16);       // all-pass delay D (SHIPPED 16)
	int    ap_n      = env_i("MERCURY_SFO_GRID_AP_N",   3);        // all-pass cascade depth (SHIPPED 3)
	double tr_amp    = env_f("MERCURY_SFO_GRID_FSEL_AMP",   0.6);  // two-ray 2nd-ray amplitude
	int    tr_dly    = env_i("MERCURY_SFO_GRID_FSEL_DLY", 128);    // two-ray delay (passband samp)
	// chan_sel==3: TIME-VARYING WATTERSON (2-path Gaussian-Doppler). The
	// production-finalize question (§15): the freq-focused thin lattice (dy=8,
	// time-SPARSE) was tuned on a STATIC selective channel. A time-varying fade
	// makes H change ACROSS the ~1.3 s / 60-symbol block, so the per-symbol channel
	// estimate needs TIME-axis pilots (smaller dy) to track it. This block injects a
	// faithful 2-path Watterson channel so the layout sweep can find the slow-fade
	// operating envelope (depth × Doppler) the big-block holds vs hands off to the
	// gearshift. Model (Watterson 1970, CCIR 520; the standard HF channel sim):
	//   H(n,j) = g0(n) + g1(n)·e^{-j·w_j·Δ}
	//   g0,g1 = INDEPENDENT complex-Gaussian (Rayleigh) tap gains, each Doppler-
	//           filtered to spread fd via a first-order AR(1) low-pass (the
	//           Ornstein-Uhlenbeck spectrum, matching cl_sim_phase_noise's pole
	//           form): g(n) = ρ·g(n-1) + sqrt(1-ρ²)·CN(0,1), ρ = exp(-2π·fd/f_sym),
	//           f_sym = Fs/Nofdm (one tap update per OFDM symbol). fd is the Doppler
	//           in Hz; ρ→1 as fd→0 (a SLOW fade is ~constant across the block).
	//   FADE DEPTH D (dB): path 0 is a CONSTANT unit LOS ray (g0≡1, the stable
	//           direct path); path 1 is the RAYLEIGH-FADING scatter ray with mean
	//           amplitude a = 1 - 10^(-D/20) so its destructive combine drives the
	//           |H| null toward ~10^(-D/20) (≈ D dB below the 1.0 LOS reference) and
	//           its constructive combine to ~(1+a). This is the standard Rician /
	//           one-fixed-one-Doppler-spread-tap Watterson used for mild/slow fade
	//           characterization. D=0 ⇒ a=0 ⇒ H≡1 (TRULY FLAT, no fade — the clean
	//           baseline); D=6 ⇒ a≈0.50 (mild-deep, nulls to ~-6 dB); D=10 ⇒ a≈0.68
	//           (the deep fade the HW showed is out-of-scope, 0/8). ONLY g1 varies
	//           (at the Doppler rate); g0 is the constant LOS so depth/Doppler are
	//           independent axes. Es/N0 stays honest: the channel is unit-power-
	//           normalized over the block (mean Σ|H|²/(Ngrid·Nc) == 1).
	double watt_depth = env_f("MERCURY_SFO_GRID_WATT_DEPTH_DB", 0.0); // fade depth (dB)
	double watt_fd    = env_f("MERCURY_SFO_GRID_WATT_FD_HZ",    0.5); // Doppler spread (Hz)
	int    watt_dly   = env_i("MERCURY_SFO_GRID_WATT_DLY",     128);  // 2nd-path delay (passband samp)
	std::vector<std::complex<double>> Tchan(Nc, std::complex<double>(1.0,0.0));
	std::vector<std::complex<double>> Hwatt;   // (Ngrid×Nc) per-symbol time-varying H, chan_sel==3
	if(chan_sel == 3)
	{
		// --- per-symbol Doppler-filtered tap gains ---
		double Fs    = (double)sampling_frequency;            // passband sample rate
		double f_sym = (Nofdm>0) ? (Fs / (double)Nofdm) : 1.0; // OFDM symbol rate (Hz)
		double rho   = std::exp(-2.0*M_PI*watt_fd / f_sym);    // AR(1) Doppler pole
		if(rho > 0.999999) rho = 0.999999;
		double inn   = std::sqrt(1.0 - rho*rho);               // innovation scale (unit-var)
		double a     = 1.0 - std::pow(10.0, -watt_depth/20.0); // 2nd-ray MEAN amplitude
		if(a < 0.0) a = 0.0;
		if(a > 0.99) a = 0.99;
		cl_sim_xoshiro wrng(seed ^ 0xC0FFEEu);
		auto cgauss = [&](){ return std::complex<double>(wrng.gauss(), wrng.gauss())/std::sqrt(2.0); };
		// g0 = CONSTANT unit LOS ray; g1 = Rayleigh-fading scatter ray (mean amp a),
		// init at its stationary distribution (start already faded, not from 0).
		std::complex<double> g0(1.0, 0.0);              // fixed LOS (D=0 ⇒ H≡1)
		std::complex<double> g1 = a * cgauss();         // mean power a²
		int sshift = ofdm.start_shift;
		// precompute per-carrier 2nd-path phase e^{-j w_j Δ}
		std::vector<std::complex<double>> ph2(Nc);
		for(int j=0;j<Nc;j++)
		{
			int k_centered = (j < Nc/2) ? (j - Nc/2) : (j - Nc/2 + sshift);
			double w = 2.0*M_PI*(double)k_centered/(double)Nfft;
			ph2[j] = std::polar(1.0, -w*(double)watt_dly);
		}
		Hwatt.assign((size_t)Ngrid*Nc, std::complex<double>(1.0,0.0));
		double psum=0.0;
		for(int n=0;n<Ngrid;n++)
		{
			// advance ONLY the scatter tap one symbol (AR(1) Doppler low-pass); the
			// LOS tap g0 stays fixed so the depth and Doppler axes are independent.
			g1 = rho*g1 + (a*inn)*cgauss();
			for(int j=0;j<Nc;j++)
			{
				std::complex<double> H = g0 + g1*ph2[j];
				Hwatt[(size_t)n*Nc+j] = H;
				psum += std::norm(H);
			}
		}
		// unit-power normalize over the whole block so the AWGN Es/N0 axis is exact
		double gnorm = (psum>0.0) ? std::sqrt((double)(Ngrid*Nc)/psum) : 1.0;
		for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
		{
			Hwatt[(size_t)n*Nc+j] *= gnorm;
			grid[(size_t)n*Nc+j]  *= Hwatt[(size_t)n*Nc+j];
		}
		// expose the time-averaged |T| envelope in Tchan[] for the diagnostics print
		for(int j=0;j<Nc;j++)
		{
			std::complex<double> acc(0,0);
			for(int n=0;n<Ngrid;n++) acc += Hwatt[(size_t)n*Nc+j];
			Tchan[j] = acc / (double)Ngrid;
		}
	}
	else if(chan_sel != 0)
	{
		int sshift = ofdm.start_shift;
		for(int j=0;j<Nc;j++)
		{
			int k_centered = (j < Nc/2) ? (j - Nc/2) : (j - Nc/2 + sshift);
			double w = 2.0*M_PI*(double)k_centered/(double)Nfft;
			std::complex<double> T(1.0,0.0);
			if(chan_sel == 1)
			{
				// Schroeder all-pass cascade A(e^{jw})^ap_n.
				std::complex<double> z_mD = std::polar(1.0, -w*(double)ap_dly);   // e^{-jwD}
				std::complex<double> A = (-ap_g + z_mD) / (1.0 - ap_g*z_mD);
				T = std::pow(A, ap_n);
			}
			else // chan_sel == 2: two-ray magnitude-selective
			{
				T = 1.0 + tr_amp * std::polar(1.0, -w*(double)tr_dly);
			}
			Tchan[j] = T;
		}
		// Two-ray: unit-power normalize so the AWGN Es/N0 axis stays exact
		// (Σ|T|²/Nc == 1). The all-pass is already unit-magnitude → norm == 1.
		if(chan_sel == 2)
		{
			double p=0.0; for(int j=0;j<Nc;j++) p += std::norm(Tchan[j]);
			double g = (p>0.0) ? sqrt((double)Nc/p) : 1.0;
			for(int j=0;j<Nc;j++) Tchan[j] *= g;
		}
		for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
			grid[(size_t)n*Nc+j] *= Tchan[j];
	}
	// Channel diagnostics: |T| min/max (selectivity depth) + phase spread.
	{
		double tmin=1e9, tmax=-1e9; double phmin=1e9, phmax=-1e9;
		for(int j=0;j<Nc;j++){ double m=std::abs(Tchan[j]); if(m<tmin)tmin=m; if(m>tmax)tmax=m;
			double p=std::arg(Tchan[j]); if(p<phmin)phmin=p; if(p>phmax)phmax=p; }
		// chan_sel==3 (Watterson): also report the INSTANTANEOUS |H| span over the
		// whole space-time block (the wandering null's true depth) + the per-carrier
		// TIME variation (how much H drifts symbol-to-symbol = what the time-axis
		// pilots must track). max time-drift = max_j std_n(|H(n,j)|).
		if(chan_sel==3 && !Hwatt.empty())
		{
			double hmn=1e9, hmx=-1e9, maxdrift=0.0;
			for(int j=0;j<Nc;j++)
			{
				double s=0,ss=0;
				for(int n=0;n<Ngrid;n++){ double m=std::abs(Hwatt[(size_t)n*Nc+j]);
					if(m<hmn){hmn=m;} if(m>hmx){hmx=m;} s+=m; ss+=m*m; }
				double mean=s/Ngrid, var=ss/Ngrid-mean*mean; if(var<0)var=0;
				double sd=std::sqrt(var); if(sd>maxdrift)maxdrift=sd;
			}
			std::cout << "[SFO-GRID] watterson depth=" << watt_depth << "dB fd=" << watt_fd
			          << "Hz dly=" << watt_dly << " |H|inst[" << hmn << ".." << hmx << "]"
			          << " max_time_drift(sd|H|)=" << maxdrift << std::endl;
		}
		const char* cname = (chan_sel==1)?"DET-FLOOR(all-pass,|T|=1,phase-disp)"
		                  : (chan_sel==2)?"TWO-RAY(magnitude-selective)"
		                  : (chan_sel==3)?"WATTERSON(time-varying 2-path)" : "FLAT(unity)";
		bool spx = (env_i("MERCURY_SFO_GRID_SPARSE2D",0)!=0);
		std::cout << "[SFO-GRID] channel=" << cname
		          << " |T|[" << tmin << ".." << tmax << "]"
		          << " arg(T)[" << phmin << ".." << phmax << "] rad"
		          << "  estimator=" << (no_interp?"NOINTERP" : (thin?(spx?"SPARSE-2D-INTERP":"FLAT-ML(control)"):"LS+DFT"))
		          << std::endl;
	}

	// AWGN (Es/N0): added per data/pilot subcarrier so noise_variance_estimate is a
	// meaningful quantity for the CODED path. Es/N0 >= 900 => clean (no noise, default
	// for the uncoded timing probe). The 32-QAM symbols carry unit average Es here
	// (psk.mod normalization), so sigma^2 = 1/(2*Es/N0) per real dimension.
	double grid_esn0 = env_f("MERCURY_SFO_GRID_ESN0", 900.0);
	if(grid_esn0 < 900.0)
	{
		double snr_lin = pow(10.0, grid_esn0/10.0);
		double sigma   = 1.0 / sqrt(snr_lin);   // per-complex-sample std (Es=1)
		awgn_channel.set_seed((long)((seed ^ 0xA5A5A5A5u) | 1));
		for(int ci=0; ci<Ngrid*Nc; ci++)
		{
			double nr = (sigma/sqrt(2.0)) * awgn_channel.awgn_value_generator();
			double ni = (sigma/sqrt(2.0)) * awgn_channel.awgn_value_generator();
			grid[ci] += std::complex<double>(nr, ni);
		}
	}

	// --- RX: ONE channel estimate across the whole 60-grid, then equalize. ---
	// (Acquisition is not re-tested here — sfo_block_test proved the head preamble
	//  is found; this isolates the GRID estimate's ability to hold the ramp.)
	// Copy the (drifted) grid into a demod buffer the estimator reads.
	std::vector<std::complex<double>> rx(grid);

	// STEP 2 corrector: per-symbol CPE/PEG LS de-rotation BEFORE the estimator/equalizer.
	if(track)
	{
		// For each symbol n, gather pilot residual phase err vs carrier index k,
		// LS-fit a line (intercept=CPE omega, slope=PEG delta), average over a
		// +/- win/2 sliding window of symbols, de-rotate exp(-j[omega+k*delta]).
		// Pilot value is known (pilot_configurator.sequence); residual phase =
		// arg(rx / pilot) at each pilot carrier (flat H => 0 without SFO).
		std::vector<double> sym_omega(Ngrid,0.0), sym_delta(Ngrid,0.0);
		// first pass: per-symbol raw LS fit from that symbol's pilots
		int pilot_index_base = 0;
		// Need the pilot_index offset per symbol; recompute by walking the lattice.
		std::vector<int> pidx_at_symbol(Ngrid,0);
		{
			int pidx=0;
			for(int n=0;n<Ngrid;n++){ pidx_at_symbol[n]=pidx;
				for(int j=0;j<Nc;j++) if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT) pidx++; }
		}
		(void)pilot_index_base;
		for(int n=0;n<Ngrid;n++)
		{
			int pidx = pidx_at_symbol[n];
			double Sx=0,Sy=0,Sxx=0,Sxy=0; int np=0;
			for(int j=0;j<Nc;j++)
			{
				if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT)
				{
					std::complex<double> X = ofdm.pilot_configurator.sequence[pidx++];
					std::complex<double> r = rx[(size_t)n*Nc+j] / X;   // = H_eff = unity*ramp
					double phi = atan2(r.imag(), r.real());            // wrapped phase err
					double x=(double)j;
					Sx+=x; Sy+=phi; Sxx+=x*x; Sxy+=x*phi; np++;
				}
			}
			if(np>=2){ double den=np*Sxx-Sx*Sx;
				if(fabs(den)>1e-12){ sym_delta[n]=(np*Sxy-Sx*Sy)/den; sym_omega[n]=(Sy-sym_delta[n]*Sx)/np; } }
		}
		// sliding-window average + de-rotate every carrier of every symbol
		for(int n=0;n<Ngrid;n++)
		{
			double om=0,dl=0; int cnt=0;
			for(int w=n-win/2; w<=n+win/2; w++){ if(w>=0&&w<Ngrid){ om+=sym_omega[w]; dl+=sym_delta[w]; cnt++; } }
			if(cnt>0){ om/=cnt; dl/=cnt; }
			for(int j=0;j<Nc;j++)
			{
				double ph = -(om + dl*(double)j);
				rx[(size_t)n*Nc+j] *= std::complex<double>(cos(ph), sin(ph));
			}
		}
	}

	// Channel estimate + equalize over the ONE 60-grid.
	if(no_interp)
	{
		// NEGATIVE CONTROL: estimate ONLY from the head symbols' pilots and FREEZE
		// (no per-block bilinear interpolation tracking the ramp). Implemented by
		// assigning a flat unity estimate everywhere — the ramp is then NOT removed,
		// so the tail MUST fail. Proves the harness is not too lenient.
		for(int ci=0; ci<Ngrid*Nc; ci++){ (ofdm.estimated_channel+ci)->value = std::complex<double>(1.0,0.0);
			(ofdm.estimated_channel+ci)->status = MEASURED; }
	}
	else if(thin && env_i("MERCURY_SFO_GRID_GENIE", 0))
	{
		// GENIE estimate: hand the equalizer the EXACT channel (Tchan[j] × the SFO ramp
		// at symbol n, if the tracker is OFF). Isolates the LDPC/SNR decodability limit on
		// the selective channel from the estimator quality — if genie still FAILS to decode
		// at a given EsN0, the channel+SNR is past the waterfall and NO estimator helps.
		for(int n=0;n<Ngrid;n++)
		{
			double tau_n = ppm*1e-6 * (double)n * (double)Nofdm;
			for(int j=0;j<Nc;j++)
			{
				double ph = (track ? 0.0 : -2.0*M_PI*(double)j*tau_n/(double)Nfft);
				// chan_sel==3: the TRUE channel is the per-symbol time-varying Hwatt[n][j];
				// otherwise the static Tchan[j]. The SFO ramp sits on top in both cases.
				std::complex<double> Hsel = (chan_sel==3 && !Hwatt.empty())
				                          ? Hwatt[(size_t)n*Nc+j] : Tchan[j];
				std::complex<double> Hg = Hsel * std::complex<double>(cos(ph), sin(ph));
				(ofdm.estimated_channel+n*Nc+j)->value = Hg;
				(ofdm.estimated_channel+n*Nc+j)->status = MEASURED;
			}
		}
		// genie nv = the true AWGN variance (so CSI/MMSE weighting is correctly scaled).
		double ge = env_f("MERCURY_SFO_GRID_ESN0", 900.0);
		ofdm.noise_variance_estimate = (ge<900.0) ? pow(10.0,-ge/10.0) : 1e-6;
		if(ofdm.noise_variance_estimate < 1e-6) ofdm.noise_variance_estimate = 1e-6;
	}
	else if(thin && env_i("MERCURY_SFO_GRID_SPARSE2D", 0))
	{
		grid_sparse2d_estimator(rx.data(), Ngrid, Nc);
	}
	else if(thin)
	{
		// FLAT-ML CONTROL (the §13.2 flat-channel shortcut). The stock
		// LS_channel_estimator assumes a DENSE REGULAR (Dx=1/Dy=3) lattice: per-cell
		// global-window scalar + a per-symbol DFT smoother (smooth_channel_estimate_dft,
		// ofdm.cc:2127). On a SPARSE IRREGULAR lattice it SMEARS (|H| 0.05..1.37 on a
		// UNITY channel -> BER 0.34 @0ppm). On a FLAT channel (TEST 2) the ML estimate
		// is the pilot-averaged complex gain H̄ = mean(Y_pilot/X_pilot) assigned to every
		// cell — EXACT for flat. On a frequency-SELECTIVE channel this single global
		// scalar CANNOT represent |H| and phase varying across the 50 subcarriers, so it
		// MUST FAIL — this is the negative control that proves the sparse 2D interpolator
		// (MERCURY_SFO_GRID_SPARSE2D=1) is needed and works.
		std::complex<double> Hsum(0,0); int pidx=0; int npil=0;
		for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
			if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT)
			{
				std::complex<double> X = ofdm.pilot_configurator.sequence[pidx++];
				Hsum += rx[(size_t)n*Nc+j] / X;   // Y/X per pilot (post-tracker ~unity on FLAT)
				npil++;
			}
		std::complex<double> Hbar = (npil>0) ? (Hsum / (double)npil) : std::complex<double>(1.0,0.0);
		// Honest residual noise variance against the flat estimate (pilot EVM).
		double nsum=0.0; pidx=0;
		for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
			if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT)
			{
				std::complex<double> X = ofdm.pilot_configurator.sequence[pidx++];
				std::complex<double> resid = rx[(size_t)n*Nc+j] - Hbar*X;
				nsum += resid.real()*resid.real() + resid.imag()*resid.imag();
			}
		for(int ci=0; ci<Ngrid*Nc; ci++){ (ofdm.estimated_channel+ci)->value = Hbar;
			(ofdm.estimated_channel+ci)->status = MEASURED; }
		ofdm.noise_variance_estimate = (npil>0) ? (nsum/(double)npil) : 0.01;
		if(ofdm.noise_variance_estimate < 1e-6) ofdm.noise_variance_estimate = 1e-6;
	}
	else
	{
		ofdm.LS_channel_estimator(rx.data());   // pilots -> interpolate across 60-grid
	}

	// Capture per-data-carrier CSI weight |H_k|² (in deframed raster order) BEFORE the
	// equalizer wipes estimated_channel[].status. On a frequency-SELECTIVE channel the
	// deep-magnitude-null carriers are noise-amplified by ZF (1/|H|²); a SCALAR nv makes
	// the LDPC over-confident on those carriers and decode fails. The production path
	// (telecom_system.cc:2874-2927) scales each carrier's LLRs by normalized |H_k|² so the
	// decoder discounts the weak carriers — the SAME CSI weighting is required here for the
	// selective channel (on a flat channel all weights ≈ 1 → no change, so TEST-2 holds).
	std::vector<double> csi_data(nData, 1.0);
	{
		int di=0;
		for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
			if((ofdm.ofdm_frame+n*Nc+j)->type==DATA)
			{
				std::complex<double> H=(ofdm.estimated_channel+n*Nc+j)->value;
				if(di<nData) csi_data[di]=H.real()*H.real()+H.imag()*H.imag();
				di++;
			}
	}

	std::vector<std::complex<double>> eq((size_t)Ngrid*Nc);
	ofdm.channel_equalizer(rx.data(), eq.data());

	std::vector<std::complex<double>> deframed(nData);
	ofdm.deframer(eq.data(), deframed.data());

	if(env_i("MERCURY_SFO_GRID_DIAG",0))
	{
		std::cout << "[SFO-GRID-DIAG] nv=" << ofdm.noise_variance_estimate << std::endl;
		for(int d=0; d<4 && d<nData; d++)
			std::cout << "[SFO-GRID-DIAG] data["<<d<<"] tx=(" << tx_syms[d].real()<<","<<tx_syms[d].imag()
			          << ") eqdeframed=(" << deframed[d].real()<<","<<deframed[d].imag()<<")" << std::endl;
		// also dump a couple of estimated_channel values at DATA positions
		std::cout << "[SFO-GRID-DIAG] H[sym0,c0]=(" << (ofdm.estimated_channel+0)->value.real()
		          << "," << (ofdm.estimated_channel+0)->value.imag() << ")"
		          << " H[sym30,c25]=(" << (ofdm.estimated_channel+30*Nc+25)->value.real()
		          << "," << (ofdm.estimated_channel+30*Nc+25)->value.imag() << ")" << std::endl;
		// Lattice histogram (pilots per symbol) + |H| stats across the grid. The
		// pilots/sym min is the tracker's per-symbol LS-fit support (needs >=2). The
		// |H| min/max/mean is the channel-estimate sanity: on this flat-unity grid a
		// healthy estimate reads ~1.0 everywhere; the per-cell-LS+DFT-smoother path
		// SMEARS it on a sparse lattice (|H| 0.05..1.37, mean 0.38 -> BER 0.34), which
		// is why the thin path uses the flat-channel pilot-averaged estimator instead.
		int pmin=Nc+1, pmax=-1; double psum=0;
		for(int n=0;n<Ngrid;n++){ int pc=0; for(int j=0;j<Nc;j++){ if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT) pc++; }
			if(pc<pmin){ pmin=pc; } if(pc>pmax){ pmax=pc; } psum+=pc; }
		double hmin=1e9,hmax=-1e9,hsum=0;
		for(int ci=0;ci<Ngrid*Nc;ci++){ std::complex<double> H=(ofdm.estimated_channel+ci)->value;
			double m=std::abs(H); if(m<hmin){ hmin=m; } if(m>hmax){ hmax=m; } hsum+=m; }
		std::cout << "[SFO-GRID-DIAG] pilots/sym min="<<pmin<<" max="<<pmax<<" mean="<<(psum/Ngrid)
		          << " |H| min="<<hmin<<" max="<<hmax<<" mean="<<(hsum/(Ngrid*Nc))<<std::endl;
	}

	// --- SCORE: per-symbol uncoded symbol-error-rate, head vs tail. ---
	// Hard-decision: re-mod the known bits per data carrier and compare nearest.
	// Use psk.demod (LLR) -> hard bits and count bit errors, bucketed by symbol.
	std::vector<float> llr(nBits);
	double variance = ofdm.measure_variance(rx.data());
	psk.demod(deframed.data(), nBits, llr.data(), (float)variance);

	// Map each data carrier back to its symbol index to bucket errors head vs tail.
	// data carriers are filled by framer in (symbol,carrier) raster order over DATA
	// positions; reconstruct that order to know which symbol each data index lands in.
	std::vector<int> data_symbol_of_idx(nData,0);
	{
		int di=0;
		for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
			if((ofdm.ofdm_frame+n*Nc+j)->type==DATA){ if(di<nData) data_symbol_of_idx[di]=n; di++; }
	}
	long head_err=0, head_tot=0, tail_err=0, tail_tot=0, all_err=0;
	int tail_start = (Ngrid*3)/4;   // last quarter = "tail"
	for(int b=0;b<nBits;b++)
	{
		int hard = (llr[b] < 0.0f) ? 1 : 0;   // LLR sign -> bit (demod convention)
		int bit_err = (hard != tx_bits[b]) ? 1 : 0;
		all_err += bit_err;
		int data_idx = b / log2M;
		int sym = (data_idx<nData)?data_symbol_of_idx[data_idx]:0;
		if(sym < Ngrid/4){ head_tot++; head_err+=bit_err; }
		else if(sym >= tail_start){ tail_tot++; tail_err+=bit_err; }
	}
	double ber_all  = (double)all_err/(double)nBits;
	double ber_head = head_tot? (double)head_err/(double)head_tot : 0.0;
	double ber_tail = tail_tot? (double)tail_err/(double)tail_tot : 0.0;

	std::cout << "[SFO-GRID] ===== RESULT (one estimate across " << Ngrid << " symbols) =====" << std::endl;
	std::cout << "[SFO-GRID]   uncoded_BER_all=" << ber_all
	          << "  HEAD(sym0.." << (Ngrid/4-1) << ")=" << ber_head
	          << "  TAIL(sym" << tail_start << ".." << (Ngrid-1) << ")=" << ber_tail << std::endl;
	std::cout << "[SFO-GRID]   tot biterr=" << all_err << "/" << nBits
	          << "  mode=" << (no_interp?"NOINTERP(neg-ctrl)":(track?"CPE/PEG-TRACK":"2D-PILOT-EST"))
	          << std::endl;
	std::cout << "[SFO-GRID]   make-or-break: TAIL BER ~0 => grid holds the SFO ramp;"
	          << " TAIL >> HEAD => ramp walks the tail off (needs tracker)." << std::endl;

	// ====================================================================
	// CODED K-CODEWORD DECODE (requirement (b)): real rate-0.875 LDPC block
	// decode at the achieved pilot fraction, under SFO, with the tracker.
	// ====================================================================
	// The first Kcw*N data bits carry Kcw systematic codewords. Demap the SAME
	// deframed data carriers with the production noise_variance_estimate (the
	// quantity the E1/cfg16-nvfix bug collapses), then LDPC-decode each 1600-bit
	// codeword and report decoded/iter. nv is REPORTED so the GO/NO-GO can confirm
	// it did NOT collapse to ~1e-6 (over-confident LLRs -> BP iter caps at 101).
	if(coded)
	{
		// Per-carrier LLR for the whole grid (production demapper + production nv).
		std::vector<float> clr(nBits);
		double cvar = ofdm.noise_variance_estimate;   // the estimator's nv (NOT measure_variance)
		if(cvar < 1e-9) cvar = 1e-9;
		psk.demod(deframed.data(), nBits, clr.data(), (float)cvar);

		// cfg16-nvfix A/B INSTRUMENTATION (sim-validatability of the ~1000x nv-collapse).
		// nv (cross-pilot differential, ofdm.noise_variance_estimate) is the PRE-equalization
		// noise the demap is fed; measure_var (ofdm.measure_variance over the post-EQ
		// constellation, `variance` above) is the TRUE post-EQ noise the Euclidean demapper
		// needs. The HW collapse = nv << measure_var (over-confident LLR -> 32-QAM inner-bit
		// flips). Print the ratio per cell and gate the same RATIO-FIX (a0e22c8, K=8) the
		// production demap uses, so this harness can show whether (1) the collapse REPRODUCES
		// under tracker+freq-selective, and (2) the ratio-fix RECOVERS the decode where base
		// collapses. Env unset / no collapse => demap_var==nv => fix==base (proven no-op).
		double nv_meas    = ofdm.noise_variance_estimate;   // the estimator's nv
		// PRODUCTION-FAITHFUL measure_var: the production demap (telecom_system.cc:2948) computes
		// measure_var = ofdm.measure_variance(data_container.equalized_data) i.e. the POST-EQ pilot
		// residual |Y_pilot/H_est - X_pilot|^2 -- NOT the pre-EQ measure_variance(rx) the harness
		// uses for its uncoded-BER score above. To make this nvfix A/B identical to the real fix
		// we recompute measure_var on the EQUALIZED grid `eq` (== data_container.equalized_data in
		// production). [eq is filled by ofdm.channel_equalizer(rx, eq) above.]
		double mvar_meas  = ofdm.measure_variance(eq.data());   // POST-EQ, == production measure_var
		double mvar_preeq = variance;                            // (pre-EQ measure_variance(rx), for reference)
		double nv_ratio   = (nv_meas > 1e-12) ? (mvar_meas / nv_meas) : 1e12;
		// GROUND TRUTH: the actual post-EQ data-carrier EVM (|deframed - tx_syms|^2 over DATA
		// carriers). This is the quantity the Euclidean demap's `variance` SHOULD equal. If it
		// tracks nv (small), nv is correct and the fix (raising to measure_var) is a regression;
		// if it tracks measure_var (large), the fix is right. tx_syms[] is the known TX
		// constellation; deframed[] the post-EQ RX constellation (both in DATA raster order).
		double true_evm_sum = 0.0; int true_evm_n = 0;
		for(int d=0; d<nData && d<(int)deframed.size(); d++)
		{
			std::complex<double> e = deframed[d] - tx_syms[d];
			true_evm_sum += e.real()*e.real() + e.imag()*e.imag();
			true_evm_n++;
		}
		double true_data_evm = (true_evm_n>0) ? (true_evm_sum/(double)true_evm_n) : 0.0;
		printf("[TRUE-EVM] cfg=%d nv=%.6e measure_var_postEQ=%.6e measure_var_preEQ=%.6e true_data_postEQ_evm=%.6e  evm/nv=%.3f  measvarPostEQ/evm=%.3f\n",
			current_configuration, nv_meas, mvar_meas, mvar_preeq, true_data_evm,
			(nv_meas>1e-12? true_data_evm/nv_meas : -1.0),
			(true_data_evm>1e-12? mvar_meas/true_data_evm : -1.0));
		fflush(stdout);
		const double NV_COLLAPSE_RATIO_K = 8.0;
		double demap_variance = nv_meas;
		bool   nvfix_engaged  = false;
		if(nv_meas < mvar_meas / NV_COLLAPSE_RATIO_K) { demap_variance = mvar_meas; nvfix_engaged = true; }
		printf("[FRAME-NV] cfg=%d nv=%.6e mvar=%.6e ratio_mvar_over_nv=%.3f K=%.1f collapse=%s demap_var=%.6e amprest=%d Nsymb=%d\n",
			current_configuration, nv_meas, mvar_meas, nv_ratio, NV_COLLAPSE_RATIO_K,
			(nv_ratio > NV_COLLAPSE_RATIO_K ? "YES" : "no"),
			demap_variance, ofdm.channel_estimator_amplitude_restoration, ofdm.Nsymb);
		fflush(stdout);
		// LLR set for the FIX arm (ratio-gated demap_variance). Base arm = `clr` above.
		std::vector<float> clr_fix(nBits);
		double cvfix = demap_variance; if(cvfix < 1e-9) cvfix = 1e-9;
		psk.demod(deframed.data(), nBits, clr_fix.data(), (float)cvfix);

		// CSI-weighted LLR (production telecom_system.cc:2901-2927): scale each data
		// carrier's bit-LLRs by its normalized |H_k|² so the LDPC discounts the deep-null
		// carriers a frequency-selective channel produces. On a flat channel all weights
		// ≈ 1 → no change (TEST-2 byte-identical). Off via MERCURY_SFO_GRID_CSI=0.
		bool csi = (env_i("MERCURY_SFO_GRID_CSI", 1) != 0);
		if(csi)
		{
			double mean_w=0.0; for(int d=0;d<nData;d++) mean_w+=csi_data[d];
			mean_w = (nData>0)?mean_w/(double)nData:1.0; if(mean_w<1e-9) mean_w=1.0;
			for(int d=0;d<nData;d++)
			{
				double w = csi_data[d]/mean_w;
				for(int b=0;b<log2M;b++)
				{
					size_t bi=(size_t)d*log2M+b;
					if(bi>=(size_t)nBits) break;
					float v = clr[bi]*(float)w;
					if(v> 40.0f) v= 40.0f; else if(v<-40.0f) v=-40.0f;
					clr[bi]=v;
					// FIX arm gets the identical CSI weighting (only the demap variance differs).
					float vf = clr_fix[bi]*(float)w;
					if(vf> 40.0f) vf= 40.0f; else if(vf<-40.0f) vf=-40.0f;
					clr_fix[bi]=vf;
				}
			}
		}

		int    cw_ok = 0, cw_crcfail = 0;
		long   cw_infoerr = 0, cw_infobits = 0;
		long   iter_sum = 0; int iter_min = 1<<30, iter_max = -1;
		int    fix_ok = 0, fix_crcfail = 0;      // FIX arm (ratio-gated demap_variance)
		long   fix_infoerr = 0; int fix_iter_max = -1;
		std::vector<float> cwllr(ldpc.N);
		std::vector<int>   dec(ldpc.N);
		std::vector<float> cwllr_f(ldpc.N);
		std::vector<int>   dec_f(ldpc.N);
		for(int c=0;c<Kcw;c++)
		{
			for(int i=0;i<ldpc.N;i++) cwllr[i] = clr[(size_t)c*ldpc.N + i];
			int iters = ldpc.decode(cwllr.data(), dec.data());
			iter_sum += iters;
			if(iters < iter_min) iter_min = iters;
			if(iters > iter_max) iter_max = iters;
			// Info-bit errors of THIS codeword (dec[0..K-1] vs the TX info bits).
			int ierr=0;
			for(int i=0;i<ldpc.K;i++) if(dec[i] != cw_info[c][i]) ierr++;
			cw_infoerr  += ierr;
			cw_infobits += ldpc.K;
			// "decoded" = converged before the iter cap AND info matches (the harness
			// has no CRC; exact info recovery is the strict success criterion).
			bool capped = (iters > (ldpc.nIteration_max - 1));
			if(ierr == 0 && !capped) cw_ok++; else cw_crcfail++;

			// FIX arm: identical codeword, ratio-gated demap_variance LLR.
			for(int i=0;i<ldpc.N;i++) cwllr_f[i] = clr_fix[(size_t)c*ldpc.N + i];
			int iters_f = ldpc.decode(cwllr_f.data(), dec_f.data());
			if(iters_f > fix_iter_max) fix_iter_max = iters_f;
			int ierr_f=0;
			for(int i=0;i<ldpc.K;i++) if(dec_f[i] != cw_info[c][i]) ierr_f++;
			fix_infoerr += ierr_f;
			bool capped_f = (iters_f > (ldpc.nIteration_max - 1));
			if(ierr_f == 0 && !capped_f) fix_ok++; else fix_crcfail++;
		}
		double cw_ber = cw_infobits ? (double)cw_infoerr/(double)cw_infobits : 1.0;
		double fix_ber = cw_infobits ? (double)fix_infoerr/(double)cw_infobits : 1.0;
		std::cout << "[SFO-GRID-CODED] ===== CODED RESULT (K=" << Kcw
		          << " x " << ldpc.N << "-bit rate-" << ldpc.rate << " codewords) =====" << std::endl;
		std::cout << "[SFO-GRID-CODED]   nv=" << ofdm.noise_variance_estimate
		          << "  EsN0=" << grid_esn0
		          << "  iter_mean=" << (Kcw? (double)iter_sum/(double)Kcw : 0.0)
		          << " iter_min=" << (iter_max<0?0:iter_min) << " iter_max=" << (iter_max<0?0:iter_max)
		          << "  iter_cap=" << ldpc.nIteration_max << std::endl;
		std::cout << "[SFO-GRID-CODED]   codewords_decoded=" << cw_ok << "/" << Kcw
		          << "  fail=" << cw_crcfail
		          << "  post_FEC_info_BER=" << cw_ber << std::endl;
		std::cout << "[SFO-GRID-CODED]   nv_check: " << (ofdm.noise_variance_estimate > 1e-5 ? "OK (not collapsed)" : "COLLAPSED (<1e-5 -> E1 bug)")
		          << "  iter_cap_check: " << (iter_max < ldpc.nIteration_max ? "OK (no codeword hit cap)" : "CAPPED (BP at 101 -> over-confident LLR)") << std::endl;
		// nvfix A/B verdict: BASE arm (demap with nv) vs FIX arm (ratio-gated demap_variance).
		std::cout << "[SFO-GRID-NVFIX-AB]   nvfix_engaged=" << (nvfix_engaged?"YES":"no")
		          << "  ratio_mvar_over_nv=" << nv_ratio
		          << "  BASE decoded=" << cw_ok << "/" << Kcw << " post_FEC_BER=" << cw_ber
		          << "  FIX decoded=" << fix_ok << "/" << Kcw << " post_FEC_BER=" << fix_ber
		          << "  delta_decoded=" << (fix_ok - cw_ok) << std::endl;
	}

	// Restore the original grid geometry so the rest of the process is unaffected.
	ofdm.deinit();
	ofdm.start_shift = sav_start_shift;
	ofdm.preamble_configurator.Nsymb = sav_pre_Nsymb;
	ofdm.preamble_configurator.modulation = sav_pre_mod;
	ofdm.preamble_configurator.nIdentical_sections = sav_pre_nIS;
	ofdm.preamble_configurator.boost = sav_pre_boost;
	ofdm.pilot_configurator.boost = sav_pil_boost;
	ofdm.pilot_configurator.modulation = sav_pil_mod;
	ofdm.pilot_configurator.seed = sav_pil_seed;
	ofdm.pilot_configurator.Dx=1; ofdm.pilot_configurator.Dy=3;
	ofdm.pilot_configurator.pilot_density=HIGH_DENSITY;
	ofdm.channel_estimator=LEAST_SQUARE;
	ofdm.channel_estimator_amplitude_restoration=NO;
	ofdm.LS_window_width=0; ofdm.LS_window_hight=0;
	ofdm.init(Nfft, Nc, saved_Nsymb, gi);
}

// ============================================================================
// BIG-BLOCK HW DE-RISK — PHY-only WAV emit / decode (no ARQ, no live audio dev)
// ============================================================================
// fact-documents/bigblock-hw-wav-derisk.md. These three functions emit the
// validated big-block (§13-§14) over a REAL passband round-trip into a WAV file
// (S16LE 48 kHz mono = the butler PLAY/RECORD format), and decode a recorded WAV
// with REAL acquisition + the channel-adaptive sparse-2D/flat-ML estimator + LDPC.
// The grid-build and the estimator/equalizer/LDPC are COPIED from sfo_grid_test so
// a loopback decode failure isolates to the new passband bridge, not the validated
// estimator. Knobs (env, all defaulted to the §14.4 freq-focused 7.2% layout):
//   MERCURY_BIGBLOCK_NSYMB   (60)    data symbols in the block
//   MERCURY_BIGBLOCK_SEED    (12345) known-payload PRBS seed (TX and decode share)
//   MERCURY_BIGBLOCK_CONT_COLS (2)   continual pilot columns
//   MERCURY_BIGBLOCK_SCAT_DX (3)     scatter carrier step (FINAL freeze; §7.4)
//   MERCURY_BIGBLOCK_SCAT_DY (4)     scatter symbol step  (FINAL freeze; §7.4)
//   FREEZE NOTE (fact-doc §7): the layout was dx=4/dy=8 (7.2%, freq-focused) — tuned on a
//   STATIC selective channel; it holds ONLY a FLAT fade and collapses at the first dB of
//   TIME-VARYING (Watterson) fade. The Watterson envelope sweep (watterson_sweep.py) found
//   the slow-fade limiter is FREQUENCY resolution (dx) at low Doppler + TIME tracking (dy)
//   at mild Doppler. dx 4->3 + dy 8->4 (12% pilots) extends the envelope to ~4 dB @0.1 Hz /
//   ~2 dB @0.5 Hz at ZERO net-PHY cost (K=8 unchanged -> 7226 bps data-payload, > VARA 7050;
//   K only drops to 7 at ~16% pilots). Loopback bigblock TX->decode 8/8 BER=0 at this layout.
//   MERCURY_BIGBLOCK_SPARSE2D (1)    1=sparse-2D estimator, 0=flat-ML control
//   MERCURY_BIGBLOCK_LEAD_MS (250)   lead silence ms (acquisition + HW key-up room)
//   MERCURY_BIGBLOCK_TRAIL_MS (250)  trail silence ms (HW key-down room)
//   MERCURY_BIGBLOCK_K       (0)     0 = all whole codewords that fit; else cap at K
//   MERCURY_BIGBLOCK_DIAG    (0)     1 = extra per-codeword diagnostics

// --- minimal WAV (RIFF/PCM S16LE) writer/reader, file-scope helpers ----------
namespace {
struct wav_hdr_le {
	// little-endian PCM mono S16 @ given sample rate
	static void put_u32(std::vector<unsigned char>& b, uint32_t v){
		b.push_back(v&0xff); b.push_back((v>>8)&0xff); b.push_back((v>>16)&0xff); b.push_back((v>>24)&0xff); }
	static void put_u16(std::vector<unsigned char>& b, uint16_t v){
		b.push_back(v&0xff); b.push_back((v>>8)&0xff); }
};

// Write doubles (already clamped/scaled NOT — we scale here) as S16LE mono WAV.
// Scaling matches production audio (audioio.c:733): clamp [-1,1] then *32767.0.
static bool bigblock_write_wav(const char* path, const std::vector<double>& pb, int sample_rate)
{
	std::vector<unsigned char> hdr;
	uint32_t nSamp = (uint32_t)pb.size();
	uint32_t dataBytes = nSamp * 2u;        // 16-bit mono
	uint32_t byteRate  = (uint32_t)sample_rate * 2u;
	hdr.insert(hdr.end(), {'R','I','F','F'});
	wav_hdr_le::put_u32(hdr, 36u + dataBytes);
	hdr.insert(hdr.end(), {'W','A','V','E'});
	hdr.insert(hdr.end(), {'f','m','t',' '});
	wav_hdr_le::put_u32(hdr, 16u);          // fmt chunk size
	wav_hdr_le::put_u16(hdr, 1u);           // PCM
	wav_hdr_le::put_u16(hdr, 1u);           // mono
	wav_hdr_le::put_u32(hdr, (uint32_t)sample_rate);
	wav_hdr_le::put_u32(hdr, byteRate);
	wav_hdr_le::put_u16(hdr, 2u);           // block align
	wav_hdr_le::put_u16(hdr, 16u);          // bits/sample
	hdr.insert(hdr.end(), {'d','a','t','a'});
	wav_hdr_le::put_u32(hdr, dataBytes);
	FILE* f = fopen(path, "wb");
	if(!f) return false;
	fwrite(hdr.data(), 1, hdr.size(), f);
	std::vector<int16_t> s16(nSamp);
	for(uint32_t i=0;i<nSamp;i++){
		double c = pb[i];
		if(c >  1.0) c =  1.0;
		if(c < -1.0) c = -1.0;
		s16[i] = (int16_t)(c * 32767.0);
	}
	fwrite(s16.data(), sizeof(int16_t), nSamp, f);
	fclose(f);
	return true;
}

// Read S16LE mono (or take channel 0 of multi-ch) WAV into doubles in [-1,1].
// Parses the RIFF/fmt/data chunks; tolerant of extra chunks before "data".
static bool bigblock_read_wav(const char* path, std::vector<double>& out, int& sample_rate_out, int& ch_out)
{
	FILE* f = fopen(path, "rb");
	if(!f) return false;
	auto rd_u32=[&](uint32_t& v)->bool{ unsigned char b[4]; if(fread(b,1,4,f)!=4) return false;
		v=(uint32_t)b[0]|((uint32_t)b[1]<<8)|((uint32_t)b[2]<<16)|((uint32_t)b[3]<<24); return true; };
	auto rd_u16=[&](uint16_t& v)->bool{ unsigned char b[2]; if(fread(b,1,2,f)!=2) return false;
		v=(uint16_t)b[0]|((uint16_t)b[1]<<8); return true; };
	char tag[4];
	if(fread(tag,1,4,f)!=4 || tag[0]!='R'||tag[1]!='I'||tag[2]!='F'||tag[3]!='F'){ fclose(f); return false; }
	uint32_t riffsz; rd_u32(riffsz);
	if(fread(tag,1,4,f)!=4 || tag[0]!='W'||tag[1]!='A'||tag[2]!='V'||tag[3]!='E'){ fclose(f); return false; }
	uint16_t fmt=0, nch=0, bits=0; uint32_t srate=0; bool haveFmt=false;
	std::vector<int16_t> pcm; bool haveData=false;
	while(true)
	{
		if(fread(tag,1,4,f)!=4) break;
		uint32_t csz; if(!rd_u32(csz)) break;
		if(tag[0]=='f'&&tag[1]=='m'&&tag[2]=='t'&&tag[3]==' ')
		{
			uint16_t blockalign=0; uint32_t byterate=0;
			rd_u16(fmt); rd_u16(nch); rd_u32(srate); rd_u32(byterate); rd_u16(blockalign); rd_u16(bits);
			haveFmt=true;
			// skip any extra fmt bytes
			if(csz>16) fseek(f, (long)(csz-16), SEEK_CUR);
		}
		else if(tag[0]=='d'&&tag[1]=='a'&&tag[2]=='t'&&tag[3]=='a')
		{
			uint32_t nbytes=csz;
			pcm.resize(nbytes/2);
			if(fread(pcm.data(),1,nbytes,f)!=nbytes){ /* short read tolerated */ }
			haveData=true;
			if(csz & 1) fseek(f, 1, SEEK_CUR); // pad byte
		}
		else
		{
			fseek(f, (long)csz + (long)(csz&1), SEEK_CUR); // skip unknown chunk (+pad)
		}
	}
	fclose(f);
	if(!haveFmt || !haveData || fmt!=1 || bits!=16 || nch==0) return false;
	sample_rate_out = (int)srate;
	ch_out = (int)nch;
	size_t frames = pcm.size() / nch;
	out.resize(frames);
	for(size_t i=0;i<frames;i++)
		out[i] = (double)pcm[i*nch + 0] / 32767.0;   // channel 0
	return true;
}
} // anon namespace

// Shared lattice rebuild — IDENTICAL to sfo_grid_test's thin-lattice setup, so the
// TX framer, the RX deframer/estimator, and the pilot DBPSK sequence match exactly.
int cl_telecom_system::bigblock_rebuild_thin_grid(int& Ngrid_out, int& log2M_out, int& nBits_out)
{
	auto env_i = [](const char* k, int def){ const char* e=std::getenv(k); return (e&&*e)?atoi(e):def; };
	int Ngrid     = env_i("MERCURY_BIGBLOCK_NSYMB", 60);
	int cont_cols = env_i("MERCURY_BIGBLOCK_CONT_COLS", 2);
	int scat_dx   = env_i("MERCURY_BIGBLOCK_SCAT_DX", 3);   // FINAL freeze (fact-doc §7.4)
	int scat_dy   = env_i("MERCURY_BIGBLOCK_SCAT_DY", 4);   // FINAL freeze (fact-doc §7.4)

	int Nc    = ofdm.Nc;
	int Nfft  = ofdm.Nfft;
	float gi  = ofdm.gi;
	int Ngi   = (int)round((double)gi * (double)Nfft);
	// preserve preamble + pilot configurator fields ofdm.deinit() zeros (see sfo_grid_test)
	int   sav_pre_Nsymb = ofdm.preamble_configurator.Nsymb;
	int   sav_pre_mod   = ofdm.preamble_configurator.modulation;
	int   sav_pre_nIS   = ofdm.preamble_configurator.nIdentical_sections;
	double sav_pre_boost= ofdm.preamble_configurator.boost;
	int   sav_start_shift = ofdm.start_shift;
	double sav_pil_boost = ofdm.pilot_configurator.boost;
	int   sav_pil_mod    = ofdm.pilot_configurator.modulation;
	int   sav_pil_seed   = ofdm.pilot_configurator.seed;
	ofdm.deinit();
	ofdm.start_shift = sav_start_shift;
	ofdm.preamble_configurator.Nsymb = sav_pre_Nsymb;
	ofdm.preamble_configurator.modulation = sav_pre_mod;
	ofdm.preamble_configurator.nIdentical_sections = sav_pre_nIS;
	ofdm.preamble_configurator.boost = sav_pre_boost;
	ofdm.pilot_configurator.boost = sav_pil_boost;
	ofdm.pilot_configurator.modulation = sav_pil_mod;
	ofdm.pilot_configurator.seed = sav_pil_seed;
	ofdm.pilot_configurator.Dx = 1; ofdm.pilot_configurator.Dy = 3;
	ofdm.pilot_configurator.pilot_density = HIGH_DENSITY;
	ofdm.channel_estimator = LEAST_SQUARE;
	ofdm.channel_estimator_amplitude_restoration = NO;
	ofdm.LS_window_width = 0; ofdm.LS_window_hight = 0;
	ofdm.init(Nfft, Nc, Ngrid, gi);

	// THIN lattice (cont + scatter), exactly as sfo_grid_test:
	for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
		(ofdm.ofdm_frame+n*Nc+j)->type = DATA;
	std::vector<int> cont(cont_cols);
	for(int c=0;c<cont_cols;c++)
		cont[c] = (cont_cols<=1) ? 0
		        : (int)llround((double)c*(double)(Nc-1)/(double)(cont_cols-1));
	for(int c=0;c<cont_cols;c++)
		for(int n=0;n<Ngrid;n++)
			(ofdm.ofdm_frame+n*Nc+cont[c])->type = PILOT;
	if(scat_dx > 0 && scat_dy > 0)
	{
		for(int n=0;n<Ngrid;n++)
		{
			if(n % scat_dy != 0) continue;
			int off = (n/scat_dy) * (scat_dx/2 > 0 ? scat_dx/2 : 1);
			for(int j = off % scat_dx; j < Nc; j += scat_dx)
				(ofdm.ofdm_frame+n*Nc+j)->type = PILOT;
		}
	}
	int np=0;
	for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
		if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT) np++;
	ofdm.pilot_configurator.nPilots = np;
	ofdm.pilot_configurator.nConfig = 0;
	ofdm.pilot_configurator.nData   = Ngrid*Nc - np;
	CDELETE(ofdm.pilot_configurator.sequence);
	ofdm.pilot_configurator.sequence =
	    CNEW(std::complex<double>, np, "pilot.sequence.bigblock");
	__srandom(ofdm.pilot_configurator.seed);
	int last_pilot=0;
	for(int i=0;i<np;i++)
	{
		int pv = (__random()%2) ^ last_pilot;
		ofdm.pilot_configurator.sequence[i] =
		    std::complex<double>(2*pv-1,0) * ofdm.pilot_configurator.boost;
		last_pilot = pv;
	}

	Ngrid_out  = Ngrid;
	log2M_out  = (int)round(log2((double)M));
	nBits_out  = ofdm.pilot_configurator.nData * log2M_out;
	(void)Ngi;
	return ofdm.pilot_configurator.nData;
}

// Build the TX bits: K systematic 1600-bit LDPC codewords plus random filler past
// K*N. Shared by TX (emit) and decode (compare). Returns K (codewords that fit).
// cw_info[c] = the K info bits of codeword c.
//
// payload (P1 live-path / P2 ARQ): when non-null it supplies the info bits to encode
// (length >= Kcw*ldpc.K consumed); the filler past K*N is still seeded-PRBS so the
// grid is fully populated and the (deterministic) filler matches at decode. When
// payload is null the info bits come from the seeded-PRBS (the validated known
// payload — TX and decode reconstruct identically from the seed, used by the WAV
// loopback + the P1 byte-correct gate).
// PHASE 1 (fact-doc §11.6): big-block payload ENERGY DISPERSAL (whitening). The stock
// per-frame OFDM path XORs its info bits with a PRBS before LDPC encode
// (bit_energy_dispersal, telecom_system.cc:665) so an arbitrary payload — incl. long
// runs of zeros from a SHORT compressed frame zero-padded to sub_len — modulates to a
// well-conditioned signal. The big-block real-bytes path skipped it, so a zero-padded
// payload decoded to GARBAGE (788/1400 byte errors in the in-sim loopback; full-entropy
// payloads decode 0 errors). This helper applies the SAME self-inverse XOR (a fixed-seed
// LCG PRBS, deterministic so TX and RX agree) over the whole K*ldpc.K-bit payload. It is
// a NO-OP for the seeded-PRBS validator path (nBytes==0; that payload is already random
// and uses bigblock_tx_passband's own PRBS, never this scrambler). Self-inverse: applying
// it on TX before encode and on RX after decode recovers the exact payload.
static void bigblock_whiten_payload_bits(int* payload_bits, int nbits, unsigned int seed)
{
	uint64_t s = (uint64_t)seed * 6364136223846793005ULL + 1442695040888963407ULL;
	for(int i=0;i<nbits;i++)
	{
		s = s*6364136223846793005ULL + 1442695040888963407ULL;
		int w = (int)((s >> 33) & 1ULL);
		payload_bits[i] ^= w;
	}
}
// the whitening seed (distinct from MERCURY_BIGBLOCK_SEED used for the PRBS validator
// payload so the two streams never alias). Fixed constant => TX and RX agree with no
// wire negotiation.
#define BIGBLOCK_WHITEN_SEED 0x5A3C96E1u

// 1C — BIG-BLOCK TIME/FREQ SYMBOL INTERLEAVER (fact-doc §20; CW0_GAP_VERDICT amplifier D).
// The big-block packs K LDPC codewords CODEWORD-CONTIGUOUSLY into the QAM-symbol stream
// (bigblock_build_tx_bits: cw c == tx_bits[c*N..]; psk.mod then maps log2M bits/symbol so
// cw c == a CONTIGUOUS run of ~N/log2M symbols), and ofdm.framer lays the symbol stream
// into the grid in symbol-MAJOR DATA-cell raster order. So codeword c occupies a CONTIGUOUS
// block of grid cells (~7 symbol-rows). A localized channel-estimate error (the deterministic
// Schroeder all-pass per-cell phase curvature is worst in specific grid regions) therefore
// piles CONTIGUOUS coded-bit errors into ONE codeword -> that codeword exceeds the LDPC's
// error-correcting radius -> wire-CRC fail, even though the SAME number of errors SPREAD
// across all K codewords would be cleared. The per-frame path already block-interleaves
// (telecom_system.cc:304/352, block_size=nData/10); the big-block had NO interleaver. This
// adds the SAME classic block (matrix-transpose) time/freq interleaver to the big-block TX
// (after psk.mod, before framer) and the mirror deinterleaver to every big-block RX demap
// (after deframer, before psk.demod; CSI weights deinterleaved identically — exactly the
// per-frame pattern). It spreads each codeword's symbols across the WHOLE time x freq grid so
// a localized estimate error diffuses across all K codewords. Block (de)interleaver refs:
// Forney 1971; the existing Mercury per-frame interleaver (interleaver.cc). TX+RX symmetric;
// the block size is derived from nData IDENTICALLY at TX and every RX site (below) so no wire
// negotiation is needed. Default-OFF outside the big-block path (only these functions call it).
//
// Block size: consecutive symbols (one codeword's run) must land far apart after the transpose
// so a contiguous bad region scatters across rows AND codewords. The transpose maps input
// index i*B+j -> output j*nBlocks+i, so two consecutive inputs land nBlocks=nData/B cells apart.
// Choosing B near K (the codeword count) gives nBlocks ~= symbols-per-codeword, tiling each
// codeword's run across the full grid (set MERCURY_BIGBLOCK_TFILV=8/=K to enable). DEFAULT = 1
// (identity / OFF) — see bigblock_tf_block_size below for the measured reason. A degenerate B
// (<=1, or that does not divide into >1 block) falls back to identity, so the interleaver is a
// safe no-op when disabled or when the geometry is too small.
static int bigblock_tf_block_size(int nData)
{
	auto env_i = [](const char* k, int def){ const char* e=std::getenv(k); return (e&&*e)?atoi(e):def; };
	// DEFAULT = 1 (identity / OFF). MEASURED (fact-doc §20, --test-bigblock-chanest A/B): on
	// the off-bench arbiter — whose failing mode is the DETERMINISTIC, spatially-SMOOTH
	// Schroeder all-pass per-cell phase curvature (~0.10-0.19 rad), NOT a localized BURST —
	// no non-identity B is a net win: B=8 REGRESSES the DDCE-recovered PASS-AFTER arm
	// (bytes_ok 1->0), and B>=16 DEFEATS the FAIL-BEFORE regression-catcher (the DDCE-OFF arm
	// starts decoding). The interleaver only helps when errors are CONCENTRATED in a contiguous
	// region (CW0_GAP_VERDICT amplifier D, a BURSTY channel); the deterministic-floor sim has no
	// such burst, so spreading does nothing and only disturbs DDCE's per-cell coherence. The
	// REAL HW channel (HW §3.1: localized fades / impulse noise) IS bursty, so the interleaver is
	// the correct mechanism THERE and is kept fully implemented + TX/RX-symmetric, gated by
	// MERCURY_BIGBLOCK_TFILV for the HW bench A/B (set =8 or =K to enable). Shipping it ON by
	// default would mask the off-bench gate (CLAUDE.md §2 / What-NOT) and is unvalidated on HW.
	int B = env_i("MERCURY_BIGBLOCK_TFILV", 1);
	if(B < 1) B = 1;
	if(B > nData) B = nData;
	// require at least 2 blocks for the transpose to actually spread; else identity.
	if(nData / B < 2) B = 1;
	return B;
}

static int bigblock_build_tx_bits(cl_ldpc& ldpc, int nBits, unsigned int seed, int kcap,
                                  std::vector<int>& tx_bits,
                                  std::vector<std::vector<int>>& cw_info,
                                  const int* payload = nullptr)
{
	int Kcw = nBits / ldpc.N;
	if(kcap > 0 && kcap < Kcw) Kcw = kcap;
	tx_bits.assign(nBits, 0);
	cw_info.assign(Kcw, std::vector<int>());
	// local LCG PRBS so the payload is independent of the global ts_random state and
	// bit-reproducible from the seed alone (TX and decode reconstruct identically).
	uint64_t s = (uint64_t)seed * 2862933555777941757ULL + 3037000493ULL;
	auto nextbit=[&]()->int{ s = s*6364136223846793005ULL + 1442695040888963407ULL;
		return (int)((s >> 33) & 1ULL); };
	std::vector<int> enc(ldpc.N), info(ldpc.K);
	for(int c=0;c<Kcw;c++)
	{
		for(int i=0;i<ldpc.K;i++)
			info[i] = payload ? (payload[(size_t)c*ldpc.K + i] & 1) : nextbit();
		cw_info[c] = info;
		ldpc.encode(info.data(), enc.data());
		for(int i=0;i<ldpc.N;i++) tx_bits[(size_t)c*ldpc.N + i] = enc[i];
	}
	for(int i=Kcw*ldpc.N;i<nBits;i++) tx_bits[i] = nextbit();
	return Kcw;
}

// ===== P1: SHARED IN-MEMORY BIG-BLOCK PHY WORKERS =====
// These hold the validated big-block DSP (grid build + passband bridge on TX; real
// acquisition + channel-adaptive estimate + CSI-LLR + per-codeword LDPC on RX). The
// WAV harness methods (bigblock_tx_to_wav / bigblock_decode_from_wav) and the live
// path (transmit_bigblock / receive_bigblock) BOTH call these, so the framing /
// estimator / LDPC are bit-identical across every entry point — the WAV harness is
// now WAV-I/O + lead/trail silence only. The PHY below is COPIED verbatim from the
// previous bigblock_tx_to_wav / bigblock_decode_from_wav bodies (validated 8/8 on
// real Fe-Pi clocks); only the data source (WAV file vs in-memory double*) changed.

int cl_telecom_system::bigblock_tx_passband(double* out_pb, int& nSamples_out,
                                            std::vector<std::vector<int>>& cw_info_out,
                                            const int* payload_bits)
{
	nSamples_out = 0;
	if(M == MOD_MFSK){ std::cout << "[BIGBLOCK] MFSK unsupported; use cfg 15/16." << std::endl; return 0; }
	auto env_i = [](const char* k, int def){ const char* e=std::getenv(k); return (e&&*e)?atoi(e):def; };

	int Nfft=ofdm.Nfft; float gi=ofdm.gi;
	int Ngi=(int)round((double)gi*(double)Nfft);
	int Nofdm = Nfft + Ngi;
	int interp = frequency_interpolation_rate;
	int pre_nSymb = data_container.preamble_nSymb;

	int Ngrid=0, log2M=0, nBits=0;
	int nData = bigblock_rebuild_thin_grid(Ngrid, log2M, nBits);
	int Nc = ofdm.Nc;

	unsigned int seed = (unsigned int)env_i("MERCURY_BIGBLOCK_SEED", 12345);
	int kcap = env_i("MERCURY_BIGBLOCK_K", 0);
	std::vector<int> tx_bits;
	int Kcw = bigblock_build_tx_bits(ldpc, nBits, seed, kcap, tx_bits, cw_info_out, payload_bits);

	// freq-domain grid (data + pilots placed by the thin lattice).
	std::vector<std::complex<double>> tx_syms(nData);
	psk.mod(tx_bits.data(), nBits, tx_syms.data());
	// 1C — time/freq symbol interleave: spread codeword-contiguous symbols across the whole
	// grid (deinterleaved at every big-block RX demap; see bigblock_tf_block_size). NO-OP
	// when B==1 (degenerate geometry). TX and RX derive B identically from nData.
	{
		int B = bigblock_tf_block_size(nData);
		if(B > 1){
			std::vector<std::complex<double>> ilv(nData);
			interleaver(tx_syms.data(), ilv.data(), nData, B);
			tx_syms.swap(ilv);
		}
	}
	std::vector<std::complex<double>> grid((size_t)Ngrid*Nc);
	ofdm.framer(tx_syms.data(), grid.data());

	// TX-LEVEL PARITY (P3 HW): the stock CFG16 DATA path applies a per-subcarrier
	// pre_equalization_channel multiply (transmit_byte:813, ~+8 dB CFG16 boost) on the
	// framed grid (data + pilots) BEFORE symbol_mod, then scales the whole frame by
	// get_tx_gain(TX_SIG_OFDM) (:866) and the batch band-limits with FIR_tx1/FIR_tx2
	// (arq_common.cc:4591-4592). The big-block emitted at the RAW modulator level —
	// bypassing all three — so it transmitted ~6.7 dB peak / ~5.7 dB RMS QUIETER than a
	// stock CFG16 batch frame (results_bigblock_txlevel.json). Apply the SAME conditioning
	// here so the block transmits identically-conditioned, just longer under one preamble.
	// Pre-eq goes on pilots too (the framer interleaves them): the RX pilot-based estimate
	// captures it and divides it back out (telecom_system.cc:7345-7379), so the perfect-
	// channel roundtrip still equalizes to the unit constellation — exactly the mechanism
	// the stock RX relies on. The big-block preamble matched-filter reference is updated to
	// match (bigblock_preamble_mf_snap, this file) since pre-eq reshapes the preamble too.
	bool apply_preeq = (M != MOD_MFSK && pre_equalization_channel != NULL);
	{ const char* e=std::getenv("MERCURY_BIGBLOCK_NOPREEQ"); if(e && atoi(e)!=0) apply_preeq=false; }
	if(apply_preeq)
	{
		for(int i=0;i<Ngrid;i++)
			for(int j=0;j<Nc;j++)
				grid[(size_t)i*Nc+j] *= pre_equalization_channel[j].value;
	}

	float power_normalization = sqrt((double)(ofdm.Nfft*interp));
	double preamble_boost = ofdm.preamble_configurator.boost;
	// TX_SIG_OFDM calibration gain (stock applies it to BOTH preamble and data, :860/:866).
	double ofdm_tx_gain = get_tx_gain(TX_SIG_OFDM);
	double tx_carrier = carrier_frequency + test_tx_carrier_offset;

	// --- PASSBAND BRIDGE (mirrors transmit_byte:781-836) ----------------------
	std::vector<std::complex<double>> pre_bb((size_t)Nofdm*pre_nSymb);
	{
		std::vector<std::complex<double>> pre_grid((size_t)pre_nSymb*Nc);
		for(int i=0;i<pre_nSymb*Nc;i++) pre_grid[i]=ofdm.ofdm_preamble[i].value;
		// pre-eq on the preamble subcarriers (mirrors transmit_byte:805). The RX
		// matched-filter reference is reshaped identically so acquisition stays matched.
		if(apply_preeq)
			for(int i=0;i<pre_nSymb;i++)
				for(int j=0;j<Nc;j++)
					pre_grid[(size_t)i*Nc+j] *= pre_equalization_channel[j].value;
		for(int i=0;i<pre_nSymb;i++)
			ofdm.symbol_mod(&pre_grid[(size_t)i*Nc], &pre_bb[(size_t)i*Nofdm]);
	}
	for(size_t j=0;j<(size_t)Nofdm*pre_nSymb;j++){
		pre_bb[j] /= power_normalization;
		pre_bb[j] *= sqrt(output_power_Watt)*preamble_boost*ofdm_tx_gain;
	}
	std::vector<std::complex<double>> dat_bb((size_t)Nofdm*Ngrid);
	for(int i=0;i<Ngrid;i++)
		ofdm.symbol_mod(&grid[(size_t)i*Nc], &dat_bb[(size_t)i*Nofdm]);
	for(size_t j=0;j<(size_t)Nofdm*Ngrid;j++){
		dat_bb[j] /= power_normalization;
		dat_bb[j] *= sqrt(output_power_Watt)*ofdm_tx_gain;
	}
	int pre_pb_samples = Nofdm*pre_nSymb*interp;
	int dat_pb_samples = Nofdm*Ngrid*interp;
	ofdm.baseband_to_passband(pre_bb.data(), Nofdm*pre_nSymb, out_pb,
	                          sampling_frequency, tx_carrier, carrier_amplitude, interp);
	ofdm.baseband_to_passband(dat_bb.data(), Nofdm*Ngrid, &out_pb[pre_pb_samples],
	                          sampling_frequency, tx_carrier, carrier_amplitude, interp);
	ofdm.peak_clip(out_pb, pre_pb_samples, ofdm.preamble_papr_cut);
	ofdm.peak_clip(&out_pb[pre_pb_samples], dat_pb_samples, ofdm.data_papr_cut);

	nSamples_out = pre_pb_samples + dat_pb_samples;
	return Kcw;
}

long cl_telecom_system::bigblock_preamble_mf_snap(const std::complex<double>* bb_dec,
                                                  int bb_dec_len, long coarse_dec,
                                                  int pre_nSymb, int Nofdm, int Nc,
                                                  int search_dec)
{
	// Build the DECIMATED reference preamble baseband EXACTLY as the TX did (symbol_mod of
	// ofdm_preamble[].value WITH pre_equalization_channel) — the absolute level cancels in
	// the normalized correlation, but the per-subcarrier pre-eq RESHAPES the preamble, so
	// the reference MUST carry the same pre-eq the TX applies (bigblock_tx_passband). The
	// stock OFDM RX matched-filter template bakes in pre-eq identically (telecom_system.cc:
	// 9100). Without this the snap correlation collapses against a pre-eq'd TX preamble.
	// ref_len = pre_nSymb*Nofdm decimated samples.
	int ref_len = pre_nSymb * Nofdm;
	if(ref_len <= 0 || bb_dec_len < ref_len) return coarse_dec;
	std::vector<std::complex<double>> ref((size_t)ref_len);
	{
		std::vector<std::complex<double>> pre_grid((size_t)pre_nSymb*Nc);
		for(int i=0;i<pre_nSymb*Nc;i++) pre_grid[i]=ofdm.ofdm_preamble[i].value;
		if(M != MOD_MFSK && pre_equalization_channel != NULL)
			for(int i=0;i<pre_nSymb;i++)
				for(int j=0;j<Nc;j++)
					pre_grid[(size_t)i*Nc+j] *= pre_equalization_channel[j].value;
		for(int i=0;i<pre_nSymb;i++)
			ofdm.symbol_mod(&pre_grid[(size_t)i*Nc], &ref[(size_t)i*Nofdm]);
	}
	double ref_e=0.0; for(int m=0;m<ref_len;m++) ref_e += std::norm(ref[m]);
	if(ref_e < 1e-30) return coarse_dec;

	// Search ±search_dec decimated samples around the SC coarse pick for the position that
	// MAXIMIZES |<ref, rx>|^2 / (||ref||^2 * ||rx_window||^2) — normalized matched filter.
	long lo = coarse_dec - search_dec; if(lo < 0) lo = 0;
	long hi = coarse_dec + search_dec;
	if(hi > bb_dec_len - ref_len) hi = bb_dec_len - ref_len;
	double best_c = -1.0; long best_d = coarse_dec;
	for(long d=lo; d<=hi; d++)
	{
		std::complex<double> acc(0,0); double rx_e=0.0;
		for(int m=0;m<ref_len;m++)
		{
			std::complex<double> r = bb_dec[d+m];
			acc += std::conj(ref[m]) * r;     // matched filter (mag => CFO-robust locally)
			rx_e += std::norm(r);
		}
		double denom = ref_e * rx_e;
		double c = (denom>1e-30) ? (std::norm(acc)/denom) : 0.0;
		if(c > best_c){ best_c = c; best_d = d; }
	}
	return best_d;
}

int cl_telecom_system::bigblock_rx_passband(const double* pb, int nSamples,
                                            int* out_infobits, int& K_out,
                                            std::vector<int>& cw_ok_out,
                                            double* acq_metric_out,
                                            const std::vector<std::vector<int>>* cw_info_ref)
{
	K_out = 0;
	// §19: reset the located-head stash (acq-fail leaves -1; the guard treats <0 as
	// "no locatable block" and does NOT defer). Stamped from head_delay once acquired.
	bigblock_last_rx_head_delay_samples = -1;
	if(M == MOD_MFSK){ std::cout << "[BIGBLOCK] MFSK unsupported; use cfg 15/16." << std::endl; return 0; }
	auto env_i = [](const char* k, int def){ const char* e=std::getenv(k); return (e&&*e)?atoi(e):def; };

	int Nfft=ofdm.Nfft; float gi=ofdm.gi;
	int Ngi=(int)round((double)gi*(double)Nfft);
	int Nofdm = Nfft + Ngi;
	int interp = frequency_interpolation_rate;
	int sym_samples = Nofdm * interp;
	int pre_nSymb = data_container.preamble_nSymb;

	int Ngrid=0, log2M=0, nBits=0;
	int nData = bigblock_rebuild_thin_grid(Ngrid, log2M, nBits);
	int Nc = ofdm.Nc;

	// rxpb as a vector view (the demod lambda indexes by sample, tolerating OOB)
	std::vector<double> rxpb(pb, pb + nSamples);

	// --- ACQUIRE the head preamble: real Schmidl-Cox over the whole rx buffer. ---
	long head_delay = -1; double head_metric = 0.0;
	{
		int need_syms = (int)(rxpb.size() / sym_samples) + 2;
		int buf_syms = need_syms;
		int buf_interp = Nofdm * buf_syms * interp;
		std::vector<double> pad(buf_interp, 0.0);
		int copy_n = (int)rxpb.size(); if(copy_n > buf_interp) copy_n = buf_interp;
		for(int i=0;i<copy_n;i++) pad[i]=rxpb[i];
		std::vector<std::complex<double>> bb_interp(buf_interp);
		ofdm.passband_to_baseband(pad.data(), buf_interp, bb_interp.data(),
		                          sampling_frequency, carrier_frequency, carrier_amplitude, 1,
		                          &ofdm.FIR_rx_time_sync);
		std::vector<std::complex<double>> bb_dec(Nofdm*buf_syms);
		ofdm.rational_resampler(bb_interp.data(), buf_interp, bb_dec.data(), interp, DECIMATION);
		// D3 EARLIEST-PREAMBLE EARLY-EXIT (fix/bigblock-d3-carve): the LIVE big-block ring
		// can hold 2-3 CO-RESIDENT copies of the same one-shot block (the CMD re-emits when
		// it gets no accepted SACK — trace_falselock §1). With early_exit=0 the Schmidl-Cox
		// returned the GLOBAL energy-weighted argmax (ofdm.cc weighted=metric*(A2+R)), which
		// false-locks the FRESHEST/LOUDEST LATER copy near the ring end whose body tail is
		// FUTURE -> bb_at zero-pads the late codewords -> per-cw CRC-8 demote -> PARTIAL ->
		// 0 delivered, AND defeats the §22 wait-for-tail recovery (each defer re-snapshots
		// and re-false-locks a still-later copy, so the head DRIFTS FORWARD, HW 116664->
		// 132932 — the "inverted §22"). earliest_relative=true makes the coarse SC return the
		// EARLIEST position at >= 50% of the GLOBAL metric peak (the ORIGINAL block, always
		// the earliest copy), so a later retransmission cannot shadow it. Now the §22 re-arm
		// slides the SAME earliest block's tail in-range as designed (head moves EARLIER, not
		// later). Env MERCURY_BIGBLOCK_EARLIEST=0 reverts to the global argmax for A/B.
		bool earliest = (std::getenv("MERCURY_BIGBLOCK_EARLIEST")==NULL
		                 || atoi(std::getenv("MERCURY_BIGBLOCK_EARLIEST"))!=0);
		TimeSyncResult coarse = ofdm.time_sync_preamble_halfsym(
			bb_dec.data(), Nofdm*buf_syms, 1, 1, earliest ? 0.5 : 0.0, pre_nSymb, earliest);
		// MATCHED-FILTER SNAP: disambiguate the Schmidl-Cox plateau (±~half-symbol noise-
		// fragile argmax) by snapping the coarse decimated pick to the preamble matched-
		// filter peak within ±1 symbol. Sharp single peak at the true start; recovers the
		// spurious-lobe flip the SC argmax suffers under AWGN. Env-disable for A/B.
		bool mfsnap = (std::getenv("MERCURY_BIGBLOCK_MFSNAP")==NULL || atoi(std::getenv("MERCURY_BIGBLOCK_MFSNAP"))!=0);
		long coarse_dec = coarse.delay;
		// In earliest_relative mode the coarse SC returns the EARLIEST position whose
		// normalized Schmidl-Cox metric crosses 50% of the global peak — which is the
		// LEADING EDGE of the preamble plateau (the halfsym metric is ~1.0 across the
		// WHOLE ~pre_nSymb-symbol preamble, MEASURED: a ~870-sample / ~2.8-symbol plateau
		// 16214..17087 in the CFG16 K=8 SIM_INPROC acquisition, with the global argmax at
		// the plateau's far end). The matched-filter snap must therefore search the FULL
		// plateau width FORWARD of that leading edge to land on the true preamble start.
		// Widen the MF-snap search to ±(pre_nSymb+1) symbols in earliest mode (covers the
		// whole preamble plateau + 1 symbol margin); a co-resident retransmission copy is a
		// FULL block_span (~64 symbols) away, so this window cannot reach it (no re-introduced
		// false-lock). ±1 symbol when earliest mode is off (the global argmax sits AT the peak).
		int mf_search_dec = earliest ? ((pre_nSymb+1)*Nofdm) : Nofdm;
		if(mfsnap)
			coarse_dec = bigblock_preamble_mf_snap(bb_dec.data(), Nofdm*buf_syms,
			                                       coarse.delay, pre_nSymb, Nofdm, Nc, mf_search_dec);
		long coarse_full = (long)coarse_dec * interp;
		if(mfsnap)
		{
			// The MF snap already locked the true preamble start at decimation resolution.
			// SKIP the plateau-prone fine SC re-correlation (it re-flips to the spurious
			// lobe inside its slice); the demod's pilot-EVM ±GI search recovers the
			// sub-decimation residual. head_delay = the snapped full-rate start.
			head_delay  = coarse_full;
			head_metric = coarse.correlation;
		}
		else
		{
			long slice_start = coarse_full - 2*sym_samples; if(slice_start<0) slice_start=0;
			long slice_size  = (long)(pre_nSymb+4)*sym_samples;
			if(slice_start+slice_size > buf_interp) slice_size = buf_interp - slice_start;
			TimeSyncResult fine = ofdm.time_sync_preamble_halfsym(
				&bb_interp[slice_start], (int)slice_size, interp, 1, 0.0, pre_nSymb);
			head_delay  = slice_start + fine.delay;
			head_metric = fine.correlation;
			if(head_metric < coarse.correlation*0.5){ head_delay=coarse_full; head_metric=coarse.correlation; }
		}
	}
	if(acq_metric_out) *acq_metric_out = head_metric;
	if(head_delay < 0){
		std::cout << "[BIGBLOCK] RX acquisition FAILED (no preamble found)" << std::endl;
		return 0;
	}
	// §19: stash the located head start (full-rate samples) so the ARQ window-position
	// guard can test whether the FULL block (head + preamble + Ngrid) fit the captured
	// window or whether its tail was zero-padded by bb_at (block landed too late / tail
	// not yet in the ring at snapshot time -> defer one arming cycle, do not carve garbage).
	bigblock_last_rx_head_delay_samples = head_delay;

	// --- DEMOD: from the data start (head + preamble), rebuild the Ngrid grid. ---
	long data_start0 = head_delay + (long)pre_nSymb*sym_samples;
	int span_interp = Nofdm*Ngrid*interp;
	int fir_margin  = ofdm.FIR_rx_data.filter_nTaps * interp;
	int margin_syms = (fir_margin + sym_samples - 1) / sym_samples;
	int margin_interp = margin_syms * sym_samples;
	float power_normalization = sqrt((double)ofdm.Nfft);

	// PRE-FFT TRACKED-CFO de-rotation (attempt #3, Option C): an optional per-
	// decimated-baseband-sample phase array. When supplied, demod_at de-rotates the
	// (decimated, post-resampler) baseband block by exp(-j*phase[k]) BEFORE symbol_demod's
	// FFT. Applied in the TIME domain pre-FFT, so a CONTINUOUS phase ramp (= a genuine
	// frequency correction) removes BOTH the inter-symbol phase ramp AND the intra-symbol
	// magnitude loss (the FFT of a frequency-offset signal smears bins; removing the offset
	// pre-FFT eliminates the smear) — the two obstructions §9 proved a POST-FFT per-symbol
	// phase fix cannot touch. phase[k] is built by the inter-symbol-increment tracker below.
	// nullptr => byte-identical to the stock pre-fix demod (no de-rotation).
	const std::vector<double>* cfo_track_phase = nullptr;
	// bb_at: produce the decimated, power-normalized TIME-DOMAIN baseband block (Nofdm*Ngrid
	// complex samples) for a window offset, with the optional pre-FFT CFO de-rotation already
	// applied. Split out of demod_at so the tracked-CFO estimator (below) can read the time-
	// domain samples (cyclic-prefix CFO estimate) before/without the FFT.
	auto bb_at = [&](long data_start, std::vector<std::complex<double>>& dat_bb)
	{
		long ms = data_start - margin_interp;
		int  mi = margin_interp;
		if(ms < 0){ ms = 0; mi = (int)data_start; }
		// BB-1 (PIPELINE_AUDIT BB-1, rx-baseband D1): add the TRAILING FIR warm-up margin
		// too, mirroring the per-frame reference (pb_end = extraction_delay + frame_size +
		// fir_margin, :2480). Without it the FIR epilogue zero-pads/truncates the tail
		// (~half_taps decimated samples) of the FINAL OFDM data symbol -> last codeword
		// residual. The trailing read is bounds-guarded (src<rxpb.size() zero-pads) so a
		// short rxpb just pads zeros into the margin, never the data span. The grid
		// extraction below still starts at mdec=mi/interp for Nofdm*Ngrid samples, so the
		// right margin only feeds the FIR lookahead — it does NOT shift the symbol grid.
		int slice_size = mi + span_interp + margin_interp;
		std::vector<double> dat_pb(slice_size, 0.0);
		for(int i=0;i<slice_size;i++){
			long src=ms+i; dat_pb[i] = (src>=0 && src<(long)rxpb.size()) ? rxpb[src] : 0.0; }
		std::vector<std::complex<double>> dat_bb_interp(slice_size);
		ofdm.passband_to_baseband(dat_pb.data(), slice_size, dat_bb_interp.data(),
		                          sampling_frequency, carrier_frequency, carrier_amplitude, 1,
		                          &ofdm.FIR_rx_data, (int)ms);
		std::vector<std::complex<double>> dat_bb_full((size_t)(slice_size/interp) + 1);
		int dec_total = slice_size / interp;
		ofdm.rational_resampler(dat_bb_interp.data(), slice_size, dat_bb_full.data(), interp, DECIMATION);
		dat_bb.assign((size_t)Nofdm*Ngrid, std::complex<double>(0,0));
		int mdec = mi / interp;
		for(int j=0;j<Nofdm*Ngrid && (mdec+j)<dec_total;j++) dat_bb[j] = dat_bb_full[mdec + j];
		for(int j=0;j<Nofdm*Ngrid;j++) dat_bb[j] *= power_normalization;
		// PRE-FFT time-domain de-rotation by the tracked CFO phase (Option C).
		if(cfo_track_phase && (int)cfo_track_phase->size() == Nofdm*Ngrid)
			for(int k=0;k<Nofdm*Ngrid;k++){
				double ph = -(*cfo_track_phase)[k];
				dat_bb[k] *= std::complex<double>(cos(ph), sin(ph)); }
	};
	auto demod_at = [&](long data_start, std::vector<std::complex<double>>& rx)
	{
		std::vector<std::complex<double>> dat_bb;
		bb_at(data_start, dat_bb);
		rx.assign((size_t)Ngrid*Nc, std::complex<double>(0,0));
		for(int i=0;i<Ngrid;i++) ofdm.symbol_demod(&dat_bb[(size_t)i*Nofdm], &rx[(size_t)i*Nc]);
	};

	auto pilot_evm = [&](const std::vector<std::complex<double>>& rx)->double
	{
		std::complex<double> Hsum(0,0); int pidx=0, np=0;
		for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
			if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT){
				std::complex<double> X=ofdm.pilot_configurator.sequence[pidx++];
				Hsum += rx[(size_t)n*Nc+j]/X; np++; }
		std::complex<double> Hbar=(np>0)?Hsum/(double)np:std::complex<double>(1,0);
		double e=0.0; pidx=0; int cnt=0;
		for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
			if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT){
				std::complex<double> X=ofdm.pilot_configurator.sequence[pidx++];
				std::complex<double> r=rx[(size_t)n*Nc+j]/X;
				double dr=std::abs(r)-std::abs(Hbar); e+=dr*dr; cnt++; }
		return cnt? e/cnt : 1e9;
	};

	// FINE TIMING SEARCH (pilot-EVM minimizing window nudge).
	long data_start = data_start0;
	int tadj_force = env_i("MERCURY_BIGBLOCK_TADJ", 0);
	int gi_interp  = Ngi * interp;
	std::vector<std::complex<double>> rx;
	if(tadj_force != 0)
	{
		data_start = data_start0 + tadj_force;
		demod_at(data_start, rx);
	}
	else if(env_i("MERCURY_BIGBLOCK_TSEARCH", 1) != 0)
	{
		double best_e = 1e18; long best_off = 0;
		std::vector<std::complex<double>> rx_try;
		int lo = -gi_interp + 1, hi = gi_interp/4;
		int step = env_i("BIGBLOCK_TSEARCH_STEP", 2);
		for(int off=lo; off<=hi; off+=step)
		{
			demod_at(data_start0 + off, rx_try);
			double e = pilot_evm(rx_try);
			if(e < best_e){ best_e = e; best_off = off; rx = rx_try; }
		}
		data_start = data_start0 + best_off;
		if((int)rx.size() != Ngrid*Nc) demod_at(data_start, rx);
	}
	else
	{
		demod_at(data_start, rx);
	}

	// ============================================================================
	// PRE-FFT TRACKED-CFO AFC (attempt #3, Option C) — the principled fix.
	// ----------------------------------------------------------------------------
	// The big-block RX does ONE head Schmidl-Cox TIMING acquire and (pre-fix) NO carrier-
	// frequency correction, so a residual CFO ramps a multi-cycle phasor across the ~1.56 s
	// block; the block-wide pilot estimate destructively integrates it → mean|H| → 0 → LDPC
	// decodes noise (fact-doc §3). The per-frame receive_byte path is immune because it re-
	// runs Moose every ~12-symbol frame and re-mixes in the TIME domain (telecom_system.cc
	// :2546 / :2645). This block mirrors that recipe WITHIN the big block, with NO TX change:
	//
	//   (1) PER-SYMBOL CYCLIC-PREFIX CFO (van de Beek 1997). Each OFDM symbol's guard interval
	//       (CP) is a copy of the symbol's LAST Ngi samples (gi_adder: out[j]=in[j+Nfft-Ngi]).
	//       Under a residual CFO δ the CP and its body copy differ in phase by 2π·δ·Nfft/fs.
	//       C[n] = Σ_{i<Ngi} y[n,i]·conj(y[n, Nfft+i]); δ[n] = arg(C[n])·fs/(2π·Nfft). FULL-BAND,
	//       NO pilot dependency, high-SNR (Ngi×Nc carriers' energy), unambiguous to ±fs/(2·Nfft)
	//       ≈ ±23 Hz (WB) — covers the post-Moose residual band. This is a TIME-DOMAIN, per-
	//       symbol, INSTANTANEOUS-frequency estimate: exactly what tracks the drift, and exactly
	//       what neither refuted attempt had (attempt #2 was post-FFT 2-band-edge-pilot phase).
	//   (2) HEAD Moose seeds/sanity-bounds δ and resolves the (mild) CP wrap ambiguity.
	//   (3) Median-smooth δ[n] (the drift is band-limited; rejects per-symbol jitter), INTEGRATE
	//       to a CONTINUOUS per-decimated-sample phase (a true frequency ramp, piecewise-constant
	//       rate per symbol), and RE-DEMOD with that phase removed PRE-FFT in the TIME domain.
	//       Pre-FFT + continuous tracked ramp removes BOTH the inter-symbol phase ramp AND the
	//       intra-symbol magnitude loss (the two §9 obstructions a post-FFT fix cannot touch).
	//
	// Refs: van de Beek/Sandell/Börjesson 1997 (10.1109/78.611811, ML CP-based CFO/timing for
	// OFDM); Moose 1994 (10.1109/26.328961); Speth et al. 2001 (10.1109/26.917759). Env
	// MERCURY_BIGBLOCK_AFCTRACK (default 1) toggles for A/B; =0 ⇒ stock path. Confined to
	// bigblock_rx_passband (dormant default-off bigblock_framing_enabled); the per-frame path is
	// untouched. §8 audit: only the PHASE of the time-domain block fed to the unchanged FFT/
	// estimators changes — no pilot.sequence / per-frame / consumer-invariant change.
	// ============================================================================
	std::vector<double> afc_phase;  // owns the storage cfo_track_phase points at
	if(env_i("MERCURY_BIGBLOCK_AFCTRACK", 1) != 0 && Ngrid >= 3)
	{
		// Decimated baseband sample rate and the per-symbol period in those samples.
		double fs_base = (interp > 0) ? (sampling_frequency / (double)interp) : sampling_frequency;
		double Tsym    = (fs_base > 0.0) ? ((double)Nofdm / fs_base) : 0.0;  // seconds

		// (1) HEAD Moose: residual CFO at block start (seeds δf[0]). Same estimator the per-
		// frame path uses; the head preamble lives at [head_delay .. data_start0) full-rate.
		double cfo0_hz = 0.0;
		{
			long pre_start = data_start - (long)pre_nSymb*sym_samples;  // preamble start (full rate)
			if(pre_start < 0) pre_start = 0;
			int pre_slice = (pre_nSymb + 1) * sym_samples;
			std::vector<double> pre_pb(pre_slice, 0.0);
			for(int i=0;i<pre_slice;i++){ long src=pre_start+i;
				pre_pb[i] = (src>=0 && src<(long)rxpb.size()) ? rxpb[src] : 0.0; }
			std::vector<std::complex<double>> pre_bb_i(pre_slice);
			ofdm.passband_to_baseband(pre_pb.data(), pre_slice, pre_bb_i.data(),
			                          sampling_frequency, carrier_frequency, carrier_amplitude, 1,
			                          &ofdm.FIR_rx_data, (int)pre_start);
			std::vector<std::complex<double>> pre_bb((size_t)(pre_slice/interp) + 1);
			ofdm.rational_resampler(pre_bb_i.data(), pre_slice, pre_bb.data(), interp, DECIMATION);
			// carrier_sampling_frequency_sync expects in = &baseband[Ngi] (skip the GI).
			if((int)pre_bb.size() > Ngi + pre_nSymb*Nofdm)
				cfo0_hz = ofdm.carrier_sampling_frequency_sync(
					&pre_bb[Ngi], bandwidth/(double)ofdm.Nc, pre_nSymb, sampling_frequency);
			// Sanity clamp (mirror the per-frame ±1-subcarrier clamp at :2606-2608).
			double clamp = bandwidth / (double)ofdm.Nc;
			if(cfo0_hz >  clamp) cfo0_hz =  clamp;
			if(cfo0_hz < -clamp) cfo0_hz = -clamp;
		}

		bool diag = (env_i("MERCURY_BIGBLOCK_AFC_DIAG",0) != 0);
		bool stage2_on = (env_i("MERCURY_BIGBLOCK_AFC_STAGE2", 1) != 0);

		// (2) STAGE 1 — remove the BULK residual with the ACCURATE head Moose (a constant pre-FFT
		// frequency shift, the same recipe the per-frame path uses). Proven accurate (the recovery
		// peak coincides with the head-Moose value); shrinks the residual so the continual-pilot
		// common phase is small + unwraps cleanly for Stage 2.
		std::vector<double> dfs(Ngrid, cfo0_hz);
		afc_phase.assign((size_t)Nofdm*Ngrid, 0.0);
		auto build_and_demod = [&](double acc0){
			afc_phase.assign((size_t)Nofdm*Ngrid, 0.0);
			double acc = acc0;
			for(int n=0;n<Ngrid;n++){
				double step = (fs_base>0.0) ? (2.0*M_PI*dfs[n]/fs_base) : 0.0;
				for(int s=0;s<Nofdm;s++){ afc_phase[(size_t)n*Nofdm+s] = acc; acc += step; }
			}
			cfo_track_phase = &afc_phase; demod_at(data_start, rx); cfo_track_phase = nullptr;
		};
		build_and_demod(0.0);   // rx now Stage-1-corrected (~residual-drift only)

		// (3) STAGE 2 — track the residual DRIFT from the continual pilots' CUMULATIVE common
		// phase ψ[n]=arg(Σ_{continual j} rx[n,j]·conj(X[n,j])). After Stage 1 ψ is small/slow, so
		// it UNWRAPS cleanly (the §9 aliasing was on the LARGE un-corrected ramp). Convert the
		// SMOOTHED ψ to a per-symbol residual frequency (its derivative) and ADD to Stage-1; re-
		// demod once. dfs then carries Stage1 + Stage2 = the total tracked CFO(t).
		std::vector<int> cont_cols;
		for(int j=0;j<Nc;j++){ bool all=true;
			for(int n=0;n<Ngrid;n++) if((ofdm.ofdm_frame+n*Nc+j)->type!=PILOT){ all=false; break; }
			if(all) cont_cols.push_back(j); }
		double psi0=0.0;
		if(stage2_on && !cont_cols.empty())
		{
			std::vector<std::complex<double>> P(Ngrid, std::complex<double>(0,0));
			{ int pidx=0; for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
				if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT){
					std::complex<double> X=ofdm.pilot_configurator.sequence[pidx++];
					for(int c:cont_cols) if(c==j){ P[n]+=rx[(size_t)n*Nc+j]*std::conj(X); break; } } }
			std::vector<double> psi(Ngrid,0.0); double prev=0.0; bool hp=false;
			for(int n=0;n<Ngrid;n++){
				if(std::abs(P[n])<1e-18){ psi[n]=hp?prev:0.0; continue; }
				double a=atan2(P[n].imag(),P[n].real());
				if(hp){ while(a-prev> M_PI)a-=2.0*M_PI; while(a-prev<-M_PI)a+=2.0*M_PI; }
				psi[n]=a; prev=a; hp=true;
			}
			int sw = env_i("MERCURY_BIGBLOCK_AFC_WIN", 9); if(sw<1) sw=1;
			std::vector<double> psis(Ngrid,0.0);
			for(int n=0;n<Ngrid;n++){ double s=0; int c=0;
				for(int w=n-sw/2; w<=n+sw/2; w++) if(w>=0&&w<Ngrid){ s+=psi[w]; c++; }
				psis[n]= c? s/c : psi[n]; }
			psi0 = psis[0];
			for(int n=0;n<Ngrid;n++){
				double dpsi = (n==0)?(psis[1]-psis[0]) : (n==Ngrid-1)?(psis[Ngrid-1]-psis[Ngrid-2])
				                                                     : 0.5*(psis[n+1]-psis[n-1]);
				dfs[n] += (Tsym>0.0) ? (dpsi/(2.0*M_PI*Tsym)) : 0.0;   // Stage1 + Stage2(drift)
			}
			build_and_demod(psi0);  // start phase = ψ[0] removes the residual common-phase offset too
		}

		if(diag){
			double mn=1e9,mx=-1e9,mean=0; for(int n=0;n<Ngrid;n++){ mean+=dfs[n]; if(dfs[n]<mn)mn=dfs[n]; if(dfs[n]>mx)mx=dfs[n]; } mean/=(Ngrid>0?Ngrid:1);
			std::cout << "[AFC-TRACK] head_cfo=" << cfo0_hz << " cont_cols=" << cont_cols.size()
			          << " dfs[" << mn << ".." << mx << "] mean=" << mean << " psi0=" << psi0
			          << " Tsym=" << Tsym << "s | fs_base=" << fs_base
			          << " Nfft=" << Nfft << " Ngi=" << Ngi << " Nofdm=" << Nofdm << std::endl;
			// GROUND-TRUTH residual per-symbol common phase from ALL known pilots on the
			// AFC-corrected rx (this is what a post-FFT CPE corrector would see/remove).
			// Per symbol n: theta[n]=arg(Σ_pilots rx[n,j]·conj(X)). Unwrap, report span +
			// RMS of the residual AFTER removing a best-fit linear ramp (= what's left for
			// a per-symbol CPE corrector to chase that a constant CFO cannot).
			std::vector<double> th(Ngrid,0.0); { int pidx=0; double prev=0; bool hp=false;
			  for(int n=0;n<Ngrid;n++){ std::complex<double> acc(0,0);
			    for(int j=0;j<Nc;j++) if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT){
			      std::complex<double> X=ofdm.pilot_configurator.sequence[pidx++]; acc+=rx[(size_t)n*Nc+j]*std::conj(X); }
			    double a=(std::abs(acc)>1e-18)?atan2(acc.imag(),acc.real()):(hp?prev:0.0);
			    if(hp){ while(a-prev>M_PI)a-=2*M_PI; while(a-prev<-M_PI)a+=2*M_PI; } th[n]=a; prev=a; hp=true; } }
			double thmn=1e9,thmx=-1e9; for(int n=0;n<Ngrid;n++){ if(th[n]<thmn)thmn=th[n]; if(th[n]>thmx)thmx=th[n]; }
			// best-fit linear ramp residual RMS
			double Sx=0,Sy=0,Sxx=0,Sxy=0; for(int n=0;n<Ngrid;n++){ Sx+=n; Sy+=th[n]; Sxx+=(double)n*n; Sxy+=(double)n*th[n]; }
			double den=Ngrid*Sxx-Sx*Sx, sl=0,ic=0; if(fabs(den)>1e-12){ sl=(Ngrid*Sxy-Sx*Sy)/den; ic=(Sy-sl*Sx)/Ngrid; }
			double rr=0; for(int n=0;n<Ngrid;n++){ double r=th[n]-(ic+sl*n); rr+=r*r; } rr=sqrt(rr/(Ngrid>0?Ngrid:1));
			std::cout << "[AFC-PILTHETA] theta_span[" << thmn << ".." << thmx << "] slope=" << sl
			          << " rad/sym  resid_after_ramp_RMS=" << rr << " rad" << std::endl;
			// WITHIN-SYMBOL pilot phase spread: per symbol, the RMS deviation of each pilot's
			// phase from that symbol's mean pilot phase. SMALL ⇒ residual is a per-symbol COMMON
			// phase (CPE-fixable). LARGE ⇒ residual is per-subcarrier (ICI / estimate phase err,
			// NOT removable by a per-symbol CPE). Averaged over symbols.
			{ int pidx2=0; double wsum=0; double rsum=0; int wn=0;
			  for(int n=0;n<Ngrid;n++){ std::vector<double> ph, kk; double cmean=th[n];
			    for(int j=0;j<Nc;j++) if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT){
			      std::complex<double> X=ofdm.pilot_configurator.sequence[pidx2++];
			      std::complex<double> r=rx[(size_t)n*Nc+j]/X;
			      double p=atan2(r.imag(),r.real()); double d=p-cmean; while(d>M_PI)d-=2*M_PI; while(d<-M_PI)d+=2*M_PI;
			      ph.push_back(cmean+d); kk.push_back((double)j); }
			    if(ph.size()>=2){ double s=0; for(double p:ph){ double d=p-cmean; s+=d*d; }
			      wsum+=sqrt(s/ph.size());
			      // per-symbol linear fit phi = a + b*k; residual RMS = the NON-linear part
			      // (NOT removable by a per-symbol CPE+slope corrector = ICI/selectivity).
			      double Sx=0,Sy=0,Sxx=0,Sxy=0; int m=ph.size();
			      for(int t=0;t<m;t++){ Sx+=kk[t]; Sy+=ph[t]; Sxx+=kk[t]*kk[t]; Sxy+=kk[t]*ph[t]; }
			      double dn=m*Sxx-Sx*Sx, b=0,a=0; if(fabs(dn)>1e-12){ b=(m*Sxy-Sx*Sy)/dn; a=(Sy-b*Sx)/m; }
			      double rs=0; for(int t=0;t<m;t++){ double e=ph[t]-(a+b*kk[t]); rs+=e*e; } rsum+=sqrt(rs/m);
			      wn++; } }
			  std::cout << "[AFC-WITHINSYM] mean within-symbol pilot phase RMS=" << (wn?wsum/wn:-1.0)
			            << " rad | residual_after_per-sym_LINEAR_fit=" << (wn?rsum/wn:-1.0)
			            << " rad (small=fixable by CPE+slope, large=ICI/selectivity)" << std::endl; }
		}
	}

	// --- CPE/PEG residual-timing de-rotation (STEP-2 tracker) --------
	bool track = (env_i("MERCURY_BIGBLOCK_TRACK", 1) != 0);
	int  twin  = env_i("MERCURY_BIGBLOCK_TRACK_WIN", 9);
	if(track)
	{
		std::vector<double> sym_omega(Ngrid,0.0), sym_delta(Ngrid,0.0);
		std::vector<int> pidx_at_symbol(Ngrid,0);
		{ int pidx=0; for(int n=0;n<Ngrid;n++){ pidx_at_symbol[n]=pidx;
			for(int j=0;j<Nc;j++) if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT) pidx++; } }
		for(int n=0;n<Ngrid;n++)
		{
			int pidx = pidx_at_symbol[n];
			double Sx=0,Sy=0,Sxx=0,Sxy=0; int np=0;
			for(int j=0;j<Nc;j++)
				if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT)
				{
					std::complex<double> X = ofdm.pilot_configurator.sequence[pidx++];
					std::complex<double> r = rx[(size_t)n*Nc+j] / X;
					double phi = atan2(r.imag(), r.real());
					double x=(double)j; Sx+=x; Sy+=phi; Sxx+=x*x; Sxy+=x*phi; np++;
				}
			if(np>=2){ double den=np*Sxx-Sx*Sx;
				if(fabs(den)>1e-12){ sym_delta[n]=(np*Sxy-Sx*Sy)/den; sym_omega[n]=(Sy-sym_delta[n]*Sx)/np; } }
		}
		for(int n=0;n<Ngrid;n++)
		{
			double om=0,dl=0; int cnt=0;
			for(int w=n-twin/2; w<=n+twin/2; w++) if(w>=0&&w<Ngrid){ om+=sym_omega[w]; dl+=sym_delta[w]; cnt++; }
			if(cnt>0){ om/=cnt; dl/=cnt; }
			for(int j=0;j<Nc;j++){ double ph=-(om+dl*(double)j);
				rx[(size_t)n*Nc+j] *= std::complex<double>(cos(ph), sin(ph)); }
		}
	}

	// --- CHANNEL-ADAPTIVE ESTIMATE (sparse-2D on selective; flat-ML on flat) -----
	// Channel-adaptive selection per the FINAL design: sparse-2D DVB-T polar-interp on
	// a frequency-SELECTIVE channel; flat-ML (lower noise) on a flat/clean channel.
	// last_channel_selectivity (std|H|/mean|H| from the most recent estimate) keys the
	// switch; MERCURY_BIGBLOCK_SPARSE2D forces (1=sparse-2D, 0=flat-ML) for A/B.
	bool sparse2d;
	{
		const char* e = std::getenv("MERCURY_BIGBLOCK_SPARSE2D");
		if(e && *e) sparse2d = (atoi(e) != 0);
		else {
			// adaptive: selective -> sparse-2D, flat -> flat-ML. Sentinel -1 (no prior
			// estimate) defaults to sparse-2D (the robust choice; flat-ML is the
			// optimization only when the channel is confirmed flat).
			double sel = last_channel_selectivity;
			sparse2d = (sel < 0.0) ? true : (sel >= 0.15);
		}
	}
	if(sparse2d)
	{
		grid_sparse2d_estimator(rx.data(), Ngrid, Nc);
	}
	else
	{
		std::complex<double> Hsum(0,0); int pidx=0; int npil=0;
		for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
			if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT){
				std::complex<double> X = ofdm.pilot_configurator.sequence[pidx++];
				Hsum += rx[(size_t)n*Nc+j] / X; npil++; }
		std::complex<double> Hbar = (npil>0)?(Hsum/(double)npil):std::complex<double>(1,0);
		double nsum=0.0; pidx=0;
		for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
			if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT){
				std::complex<double> X = ofdm.pilot_configurator.sequence[pidx++];
				std::complex<double> resid = rx[(size_t)n*Nc+j]-Hbar*X;
				nsum += resid.real()*resid.real()+resid.imag()*resid.imag(); }
		for(int ci=0;ci<Ngrid*Nc;ci++){ (ofdm.estimated_channel+ci)->value=Hbar; (ofdm.estimated_channel+ci)->status=MEASURED; }
		ofdm.noise_variance_estimate = (npil>0)?(nsum/(double)npil):0.01;
		if(ofdm.noise_variance_estimate<1e-6) ofdm.noise_variance_estimate=1e-6;
	}

	// per-data-carrier CSI weight |H|^2 (deframed raster order) BEFORE equalize.
	std::vector<double> csi_data(nData, 1.0);
	{
		int di=0;
		for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
			if((ofdm.ofdm_frame+n*Nc+j)->type==DATA){
				std::complex<double> H=(ofdm.estimated_channel+n*Nc+j)->value;
				if(di<nData) csi_data[di]=H.real()*H.real()+H.imag()*H.imag(); di++; }
	}

	std::vector<std::complex<double>> eq((size_t)Ngrid*Nc);
	ofdm.channel_equalizer(rx.data(), eq.data());
	std::vector<std::complex<double>> deframed(nData);
	ofdm.deframer(eq.data(), deframed.data());

	// 1C — time/freq symbol DE-interleave: invert the TX interleaver (bigblock_tx_passband)
	// so the QAM-symbol stream is back in codeword-contiguous order before psk.demod, and
	// deinterleave the per-cell CSI |H|^2 the SAME way so each symbol's reliability weight
	// follows its symbol (exactly the per-frame pattern, telecom_system.cc:2936/2940). The
	// estimate/equalize/nv all ran in GRID order above (unchanged); only the demap order is
	// restored here. NO-OP when B==1. The DIAG blocks below read eq/rx in GRID order, so they
	// remain valid measurements of the (still grid-ordered) channel.
	{
		int B = bigblock_tf_block_size(nData);
		if(B > 1){
			std::vector<std::complex<double>> dil(nData);
			deinterleaver(deframed.data(), dil.data(), nData, B);
			deframed.swap(dil);
			// CSI is std::vector<double> (no free deinterleaver overload); apply the SAME
			// block-transpose inverse inline (matches deinterleaver() in interleaver.cc).
			std::vector<double> cil(nData);
			int nBlocks = nData / B;
			for(int i=0;i<nBlocks;i++)
				for(int j=0;j<B;j++)
					cil[(size_t)i*B+j] = csi_data[(size_t)j*nBlocks+i];
			for(int i=nBlocks*B;i<nData;i++) cil[i] = csi_data[i];
			csi_data.swap(cil);
		}
	}

	// CHANNEL-ESTIMATION HEALTH stash (fix/bigblock-chanest): always-on mean|H| over the
	// estimated channel grid so the genuine 2-instance regression can assert the estimate
	// did not collapse (the DIAG print below is env-gated; this stash is unconditional).
	{
		double hmag=0.0; for(int ci=0;ci<Ngrid*Nc;ci++) hmag+=std::abs((ofdm.estimated_channel+ci)->value);
		bigblock_last_rx_meanh = (Ngrid*Nc>0) ? hmag/(double)(Ngrid*Nc) : -1.0;
	}

	// [DIAG-RXPB] instrument nv, mean|H|, post-EQ deframed constellation RMS.
	if(env_i("MERCURY_BIGBLOCK_RXPB_DIAG",0))
	{
		double Hmag=0.0, Hmin=1e9, Hmax=0.0; for(int ci=0;ci<Ngrid*Nc;ci++){ double a=std::abs((ofdm.estimated_channel+ci)->value); Hmag+=a; if(a<Hmin)Hmin=a; if(a>Hmax)Hmax=a;} Hmag/=(Ngrid*Nc);
		// std of |H| (selectivity numerator) + count of near-zero |H| (polar collapse)
		double Hvar=0.0; long nzero=0; for(int ci=0;ci<Ngrid*Nc;ci++){ double a=std::abs((ofdm.estimated_channel+ci)->value); Hvar+=(a-Hmag)*(a-Hmag); if(a<0.3*Hmag)nzero++; } Hvar=sqrt(Hvar/(Ngrid*Nc));
		// raw pilot Y/X mean magnitude + max adjacent-pilot phase step in a mid symbol
		double pilraw=0.0; int pidx=0,np=0; for(int n=0;n<Ngrid;n++)for(int j=0;j<Nc;j++) if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT){ std::complex<double> X=ofdm.pilot_configurator.sequence[pidx++]; pilraw+=std::abs(rx[(size_t)n*Nc+j]/X); np++; } pilraw/=(np>0?np:1);
		double crms=0.0; for(int d=0;d<nData;d++) crms+=std::norm(deframed[d]); crms=sqrt(crms/(nData>0?nData:1));
		double rxrms=0.0; int dn=0; for(int n=0;n<Ngrid;n++)for(int j=0;j<Nc;j++) if((ofdm.ofdm_frame+n*Nc+j)->type==DATA){ rxrms+=std::norm(rx[(size_t)n*Nc+j]); dn++; } rxrms=sqrt(rxrms/(dn>0?dn:1));
		// DECISION-DIRECTED post-EQ EVM + per-symbol-row residual common phase.
		// slice each equalized DATA cell to the nearest 32-QAM point, accumulate the
		// residual vector (y - ŷ) for EVM and the residual common phase arg(Σ y·conj(ŷ))
		// per symbol-row n. If Option C left a per-row CPE that the block-avg H cannot
		// remove, the per-row phases SPREAD even when mean|H| looks healthy.
		double evm_num=0.0, evm_den=0.0; int evcnt=0;
		double cpe_min=1e9, cpe_max=-1e9, cpe_abs_sum=0.0; int cpe_rows=0;
		for(int n=0;n<Ngrid;n++){
			std::complex<double> rowacc(0,0); int rown=0;
			for(int j=0;j<Nc;j++) if((ofdm.ofdm_frame+n*Nc+j)->type==DATA){
				std::complex<double> y = eq[(size_t)n*Nc+j];
				if(std::abs(y)<1e-12) continue;          // skip erased cells
				std::complex<double> yhat = psk.slice_nearest(y);
				std::complex<double> e = y - yhat;
				evm_num += std::norm(e); evm_den += std::norm(yhat); evcnt++;
				rowacc += y*std::conj(yhat); rown++;
			}
			if(rown>0){ double ph=atan2(rowacc.imag(),rowacc.real());
				if(ph<cpe_min)cpe_min=ph; if(ph>cpe_max)cpe_max=ph; cpe_abs_sum+=fabs(ph); cpe_rows++; }
		}
		double evm = (evm_den>0.0)? sqrt(evm_num/evm_den) : -1.0;
		double cpe_mean_abs = (cpe_rows>0)? cpe_abs_sum/cpe_rows : 0.0;
		// GENIE decomposition: per DATA cell, the true channel H_g = rx / (ŷ·cscale),
		// where ŷ is the sliced constellation point and cscale maps the unit-power
		// constellation to the rx scale (cscale = pilot Hbar magnitude proxy). Decompose
		// the equalizer error into (i) intra-column TIME variation of H_g (residual
		// ICI/SFO/CFO the per-cell estimate cannot be constant against) vs (ii) the
		// estimate's per-cell bias. csi_data already holds |H_est|^2 in deframed order.
		{
			// constellation rx-scale: mean |rx_data| / mean |ŷ| (decision-directed)
			double sr=0.0, sy=0.0; int sc=0;
			std::vector<std::complex<double>> Hg((size_t)Ngrid*Nc, std::complex<double>(0,0));
			std::vector<char> Hg_ok((size_t)Ngrid*Nc, 0);
			for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++) if((ofdm.ofdm_frame+n*Nc+j)->type==DATA){
				std::complex<double> y = eq[(size_t)n*Nc+j]; if(std::abs(y)<1e-12) continue;
				std::complex<double> yhat = psk.slice_nearest(y);
				sr += std::abs(rx[(size_t)n*Nc+j]); sy += std::abs(yhat); sc++;
			}
			double cscale = (sy>0.0)? sr/sy : 1.0;
			for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++) if((ofdm.ofdm_frame+n*Nc+j)->type==DATA){
				std::complex<double> y = eq[(size_t)n*Nc+j]; if(std::abs(y)<1e-12) continue;
				std::complex<double> yhat = psk.slice_nearest(y);
				std::complex<double> Xs = yhat*cscale;
				if(std::abs(Xs)>1e-12){ Hg[(size_t)n*Nc+j]=rx[(size_t)n*Nc+j]/Xs; Hg_ok[(size_t)n*Nc+j]=1; }
			}
			// intra-column TIME variation of the genie channel (std/mean of |Hg| per column,
			// and the per-column phase spread) — averaged over data columns.
			double col_magcv_sum=0.0, col_phspread_sum=0.0; int col_n=0;
			for(int j=0;j<Nc;j++){
				bool isdata=false; for(int n=0;n<Ngrid;n++) if((ofdm.ofdm_frame+n*Nc+j)->type==DATA){ isdata=true; break; }
				if(!isdata) continue;
				std::complex<double> macc(0,0); double m1=0,m2=0; int cn=0; double pmin=1e9,pmax=-1e9;
				for(int n=0;n<Ngrid;n++){ size_t id=(size_t)n*Nc+j; if(!Hg_ok[id]) continue;
					double mag=std::abs(Hg[id]); m1+=mag; m2+=mag*mag; macc+=Hg[id]; cn++; }
				if(cn>=4){ double mean=m1/cn; double var=m2/cn-mean*mean; if(var<0)var=0;
					double cv=(mean>1e-9)?sqrt(var)/mean:0.0;
					// per-column residual phase spread: derotate by column-mean phase, measure RMS
					double cmph=atan2(macc.imag(),macc.real()); double ps=0; int pc=0;
					for(int n=0;n<Ngrid;n++){ size_t id=(size_t)n*Nc+j; if(!Hg_ok[id]) continue;
						double d=atan2(Hg[id].imag(),Hg[id].real())-cmph;
						while(d>M_PI)d-=2*M_PI; while(d<-M_PI)d+=2*M_PI; ps+=d*d; pc++; }
					double phrms=(pc>0)?sqrt(ps/pc):0.0;
					col_magcv_sum+=cv; col_phspread_sum+=phrms; col_n++; }
			}
			double col_magcv = (col_n>0)? col_magcv_sum/col_n : -1.0;
			double col_phrms = (col_n>0)? col_phspread_sum/col_n : -1.0;
			// ESTIMATE-vs-GENIE per-DATA-cell phase error: how well the PUBLISHED estimate
			// H_est tracks the TRUE channel H_g. phase err = arg(H_g·conj(H_est)). This is
			// the DECISIVE estimator-accuracy metric and the §18 block localizer: clean
			// ~0.034 rad (estimate≈truth) vs the HW-faithful vector ~0.21 rad (the estimate
			// undersamples the per-cell non-linear time-walk at the 6-8% lattice) — above the
			// 32-QAM ~0.1-rad EVM cliff, which is why bytes_ok stays 0 even after the polar
			// time-axis magnitude-fold fix recovers mean|H|. DIAG-ONLY (env-gated print);
			// no production behavior change. magRMS carries a decision-directed cscale
			// offset (genie reconstructs on a unit-power constellation scale) so it is NOT a
			// real-error metric — read the PHASE only.
			double estg_ph2=0.0; int estg_n=0;
			for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++) if((ofdm.ofdm_frame+n*Nc+j)->type==DATA){
				size_t id=(size_t)n*Nc+j; if(!Hg_ok[id]) continue;
				std::complex<double> He=(ofdm.estimated_channel+id)->value;
				if(std::abs(He)<1e-12) continue;
				double dph=std::arg(Hg[id]*std::conj(He));
				while(dph>M_PI)dph-=2*M_PI; while(dph<-M_PI)dph+=2*M_PI;
				estg_ph2+=dph*dph; estg_n++;
			}
			double estg_phrms=(estg_n>0)?sqrt(estg_ph2/estg_n):-1.0;
			std::cout << "[DIAG-GENIE] cscale=" << cscale
			          << " col_magCV=" << col_magcv << " col_phaseRMS=" << col_phrms
			          << " est_vs_genie_phaseRMS=" << estg_phrms
			          << " (data cols=" << col_n << ", n=" << estg_n << ")" << std::endl;
		}
		std::cout << "[DIAG-RXPB] nv=" << ofdm.noise_variance_estimate
		          << " mean|H|=" << Hmag << " |H|[" << Hmin << ".." << Hmax << "]"
		          << " std|H|=" << Hvar << " nzero(<0.3mean)=" << nzero << "/" << (Ngrid*Nc)
		          << " pilraw|Y/X|=" << pilraw
		          << " deframed_rms=" << crms
		          << " rx_data_rms=" << rxrms
		          << " DDevm=" << evm << " (n=" << evcnt << ")"
		          << " rowCPE[" << cpe_min << ".." << cpe_max << "] meanabs=" << cpe_mean_abs
		          << " last_sel=" << last_channel_selectivity << std::endl;
	}

	// --- LLR + CSI weighting + LDPC per codeword ---
	std::vector<float> clr(nBits);
	double cvar = ofdm.noise_variance_estimate; if(cvar<1e-9) cvar=1e-9;
	psk.demod(deframed.data(), nBits, clr.data(), (float)cvar);
	{
		double mean_w=0.0; for(int d=0;d<nData;d++) mean_w+=csi_data[d];
		mean_w=(nData>0)?mean_w/(double)nData:1.0; if(mean_w<1e-9) mean_w=1.0;
		for(int d=0;d<nData;d++){ double w=csi_data[d]/mean_w;
			for(int b=0;b<log2M;b++){ size_t bi=(size_t)d*log2M+b; if(bi>=(size_t)nBits) break;
				float v=clr[bi]*(float)w; if(v>40.0f)v=40.0f; else if(v<-40.0f)v=-40.0f; clr[bi]=v; } }
	}

	int Kcw = nBits / ldpc.N;
	{ int kcap = env_i("MERCURY_BIGBLOCK_K", 0); if(kcap>0 && kcap<Kcw) Kcw=kcap; }

	// FAILURE-2 TEST HOOK (the --test-bigblock-cw-crc producer test): flip the sign of a
	// deterministic run of ONE codeword's LLRs BEFORE the per-codeword ldpc.decode, so the
	// REAL decode emits a corrupted (miscorrected / residual-error) info-bit sub-unit for
	// EXACTLY that codeword while the others decode clean. This drives the genuine
	// bigblock_last_rx_cw_ok producer (NOT the cw_ok array): on the live 2-instance path
	// (cw_info_ref==NULL) the producer FORCES cw_ok=1 for the corrupted codeword, and only
	// the wire-CRC recompute in the carve can demote it. MERCURY_BIGBLOCK_CORRUPT_CW = the
	// target codeword (-1 = off); MERCURY_BIGBLOCK_CORRUPT_NBITS = how many leading LLRs of
	// that codeword to slam to a strong wrong sign (enough to force a miscorrection on the
	// clean grid, default ldpc.N/4). Production never sets these; this is test-only.
	{
		int corrupt_cw = env_i("MERCURY_BIGBLOCK_CORRUPT_CW", -1);
		if(corrupt_cw >= 0 && corrupt_cw < Kcw)
		{
			int nbits_flip = env_i("MERCURY_BIGBLOCK_CORRUPT_NBITS", ldpc.N/4);
			if(nbits_flip < 1)        nbits_flip = 1;
			if(nbits_flip > ldpc.N)   nbits_flip = ldpc.N;
			long base = (long)corrupt_cw * ldpc.N;
			for(int i=0;i<nbits_flip;i++)
			{
				size_t bi = (size_t)(base + i);
				if(bi >= (size_t)nBits) break;
				// slam to a strong WRONG-sign LLR (clean LLRs are huge & correct, so an
				// equally-huge wrong sign forces the decoder off the true codeword).
				clr[bi] = (clr[bi] >= 0.0f) ? -40.0f : 40.0f;
			}
		}
	}

	cw_ok_out.assign(Kcw, 0);
	int cw_ok=0;
	std::vector<float> cwllr(ldpc.N); std::vector<int> dec(ldpc.N);
	for(int c=0;c<Kcw;c++)
	{
		for(int i=0;i<ldpc.N;i++) cwllr[i]=clr[(size_t)c*ldpc.N+i];
		ldpc.decode(cwllr.data(), dec.data());
		// extract the K info bits of this codeword into out_infobits (sub-unit c)
		for(int i=0;i<ldpc.K;i++)
			out_infobits[(size_t)c*ldpc.K + i] = dec[i];
		// per-codeword clean gate: compare against the known info bits when the caller
		// supplied them (loopback byte-correct); else (P2) the caller validates via CRC.
		int ierr=0;
		if(cw_info_ref && (int)cw_info_ref->size() > c)
			for(int i=0;i<ldpc.K;i++) if(dec[i]!=(*cw_info_ref)[c][i]){ ierr++; }
		cw_ok_out[c] = (ierr==0) ? 1 : 0;
		if(ierr==0) cw_ok++;
	}
	K_out = Kcw;
	return cw_ok;
}

int cl_telecom_system::bigblock_tx_to_wav(const char* wav_path)
{
	if(M == MOD_MFSK){ std::cout << "[BIGBLOCK-WAV] MFSK unsupported; use -s 15/16." << std::endl; return 0; }
	auto env_i = [](const char* k, int def){ const char* e=std::getenv(k); return (e&&*e)?atoi(e):def; };

	int saved_Nsymb = ofdm.Nsymb;
	int Nc=0, Nfft=ofdm.Nfft; float gi=ofdm.gi;
	int Ngi=(int)round((double)gi*(double)Nfft);
	int Nofdm = Nfft + Ngi;
	int interp = frequency_interpolation_rate;
	int sym_samples = Nofdm * interp;            // one OFDM symbol at 48 kHz
	int pre_nSymb = data_container.preamble_nSymb;

	int Ngrid=0, log2M=0, nBits=0;
	int nData = bigblock_rebuild_thin_grid(Ngrid, log2M, nBits);
	Nc = ofdm.Nc;

	unsigned int seed = (unsigned int)env_i("MERCURY_BIGBLOCK_SEED", 12345);
	int kcap = env_i("MERCURY_BIGBLOCK_K", 0);
	std::vector<int> tx_bits; std::vector<std::vector<int>> cw_info;
	int Kcw = bigblock_build_tx_bits(ldpc, nBits, seed, kcap, tx_bits, cw_info);

	// freq-domain grid (data + pilots placed by the thin lattice).
	std::vector<std::complex<double>> tx_syms(nData);
	psk.mod(tx_bits.data(), nBits, tx_syms.data());
	// 1C — time/freq symbol interleave (WAV TX; mirror of bigblock_tx_passband). Deinterleaved
	// in bigblock_decode_from_wav. NO-OP when B==1.
	{
		int B = bigblock_tf_block_size(nData);
		if(B > 1){
			std::vector<std::complex<double>> ilv(nData);
			interleaver(tx_syms.data(), ilv.data(), nData, B);
			tx_syms.swap(ilv);
		}
	}
	std::vector<std::complex<double>> grid((size_t)Ngrid*Nc);
	ofdm.framer(tx_syms.data(), grid.data());

	float power_normalization = sqrt((double)(ofdm.Nfft*interp));
	double preamble_boost = ofdm.preamble_configurator.boost;
	double tx_carrier = carrier_frequency + test_tx_carrier_offset;

	// --- PASSBAND BRIDGE (mirrors transmit_byte:781-836) ----------------------
	// (1) preamble symbols -> symbol_mod (baseband) -> scale.
	// ofdm_preamble[] is an array of st_carrier (value+type), so copy the .value
	// fields into a contiguous complex grid before symbol_mod (mirrors
	// transmit_byte:726-729 building data_container.preamble_data).
	std::vector<std::complex<double>> pre_bb((size_t)Nofdm*pre_nSymb);
	{
		std::vector<std::complex<double>> pre_grid((size_t)pre_nSymb*Nc);
		for(int i=0;i<pre_nSymb*Nc;i++) pre_grid[i]=ofdm.ofdm_preamble[i].value;
		for(int i=0;i<pre_nSymb;i++)
			ofdm.symbol_mod(&pre_grid[(size_t)i*Nc], &pre_bb[(size_t)i*Nofdm]);
	}
	for(size_t j=0;j<(size_t)Nofdm*pre_nSymb;j++){
		pre_bb[j] /= power_normalization;
		pre_bb[j] *= sqrt(output_power_Watt)*preamble_boost;   // OFDM TX_SIG gain folded into level cal; loopback uses raw level
	}
	// (2) data symbols -> symbol_mod (baseband) -> scale
	std::vector<std::complex<double>> dat_bb((size_t)Nofdm*Ngrid);
	for(int i=0;i<Ngrid;i++)
		ofdm.symbol_mod(&grid[(size_t)i*Nc], &dat_bb[(size_t)i*Nofdm]);
	for(size_t j=0;j<(size_t)Nofdm*Ngrid;j++){
		dat_bb[j] /= power_normalization;
		dat_bb[j] *= sqrt(output_power_Watt);
	}
	// (3) baseband -> passband (upconvert + interpolate to 48 kHz)
	int pre_pb_samples = Nofdm*pre_nSymb*interp;
	int dat_pb_samples = Nofdm*Ngrid*interp;
	std::vector<double> block_pb((size_t)pre_pb_samples + dat_pb_samples, 0.0);
	ofdm.baseband_to_passband(pre_bb.data(), Nofdm*pre_nSymb, block_pb.data(),
	                          sampling_frequency, tx_carrier, carrier_amplitude, interp);
	ofdm.baseband_to_passband(dat_bb.data(), Nofdm*Ngrid, &block_pb[pre_pb_samples],
	                          sampling_frequency, tx_carrier, carrier_amplitude, interp);
	// (4) PAPR clip per transmit_byte:835-836
	ofdm.peak_clip(block_pb.data(), pre_pb_samples, ofdm.preamble_papr_cut);
	ofdm.peak_clip(&block_pb[pre_pb_samples], dat_pb_samples, ofdm.data_papr_cut);

	// lead/trail silence (acquisition room + HW key-up/down margin)
	int lead_ms  = env_i("MERCURY_BIGBLOCK_LEAD_MS", 250);
	int trail_ms = env_i("MERCURY_BIGBLOCK_TRAIL_MS", 250);
	int lead_n   = (int)((double)lead_ms  * sampling_frequency / 1000.0);
	int trail_n  = (int)((double)trail_ms * sampling_frequency / 1000.0);
	std::vector<double> wav_pb((size_t)lead_n + block_pb.size() + trail_n, 0.0);
	for(size_t i=0;i<block_pb.size();i++) wav_pb[(size_t)lead_n+i] = block_pb[i];

	// peak (level sanity — should be < 1.0 so int16 doesn't clip)
	double pk=0.0; for(double v: wav_pb) if(fabs(v)>pk) pk=fabs(v);

	bool ok = bigblock_write_wav(wav_path, wav_pb, (int)sampling_frequency);

	std::cout << "[BIGBLOCK-WAV] TX cfg=" << current_configuration << " M=" << M
	          << " Nsymb=" << Ngrid << " Nc=" << Nc
	          << " pilots=" << ofdm.pilot_configurator.nPilots
	          << " (" << (100.0*ofdm.pilot_configurator.nPilots/(double)(Ngrid*Nc)) << "%)"
	          << " nData=" << nData << " nBits=" << nBits
	          << " K=" << Kcw << " (ldpc.N=" << ldpc.N << " K=" << ldpc.K << " P=" << ldpc.P << ")"
	          << " seed=" << seed << std::endl;
	std::cout << "[BIGBLOCK-WAV] TX preamble_nSymb=" << pre_nSymb
	          << " block_samples=" << block_pb.size()
	          << " lead=" << lead_n << " trail=" << trail_n
	          << " total=" << wav_pb.size()
	          << " (" << ((double)wav_pb.size()/sampling_frequency) << " s @ "
	          << sampling_frequency << " Hz)"
	          << " peak=" << pk << std::endl;
	std::cout << "[BIGBLOCK-WAV] TX wav=" << wav_path << " write=" << (ok?"OK":"FAIL")
	          << "  (decode with MERCURY_BIGBLOCK_DECODE_WAV=<path>, SAME seed/layout)" << std::endl;

	// teardown: restore the production ofdm config (as sfo_grid_test does)
	ofdm.deinit();
	ofdm.pilot_configurator.Dx=1; ofdm.pilot_configurator.Dy=3;
	ofdm.pilot_configurator.pilot_density=HIGH_DENSITY;
	ofdm.channel_estimator=LEAST_SQUARE;
	ofdm.channel_estimator_amplitude_restoration=NO;
	ofdm.LS_window_width=0; ofdm.LS_window_hight=0;
	ofdm.init(Nfft, Nc, saved_Nsymb, gi);
	return ok ? 1 : 0;
}

int cl_telecom_system::bigblock_decode_from_wav(const char* wav_path)
{
	if(M == MOD_MFSK){ std::cout << "[BIGBLOCK-WAV] MFSK unsupported; use -s 15/16." << std::endl; return 0; }
	auto env_i = [](const char* k, int def){ const char* e=std::getenv(k); return (e&&*e)?atoi(e):def; };

	// read WAV
	std::vector<double> rxpb; int srate=0, nch=0;
	if(!bigblock_read_wav(wav_path, rxpb, srate, nch)){
		std::cout << "[BIGBLOCK-WAV] DECODE read FAIL wav=" << wav_path << std::endl; return 0; }
	if(srate != (int)sampling_frequency)
		std::cout << "[BIGBLOCK-WAV] WARN wav sample_rate=" << srate
		          << " != expected " << (int)sampling_frequency << " Hz (resample upstream)" << std::endl;

	// [DIAG] STEP-1 controlled AWGN injection into the WAV-read passband so the SAME
	// decoder can be tested at a KNOWN Es/N0 (computed from the BLOCK signal power).
	// MERCURY_BIGBLOCK_WAV_ESN0 = label dB. MERCURY_BIGBLOCK_WAV_AWGN_CONV selects the
	// per-sample noise scale: 0 = canonical sigma/sqrt(2) (apply_with_delay/passband_test),
	// 1 = bare sigma (the live validator's convention at telecom_system.cc:7988).
	{
		auto env_d2 = [](const char* k, double def){ const char* e=std::getenv(k); return (e&&*e)?atof(e):def; };
		double wav_esn0 = env_d2("MERCURY_BIGBLOCK_WAV_ESN0", -999.0);
		if(wav_esn0 > -900.0)
		{
			// signal region = non-silent samples (rough): use the loudest contiguous run.
			// Measure P_sig over samples whose |x| exceeds 5% of peak (the block body),
			// matching the live validator's block-only P_sig.
			double pk=0.0; for(double v: rxpb) if(fabs(v)>pk) pk=fabs(v);
			double thr=0.05*pk; double sumsq=0.0; long cnt=0;
			for(double v: rxpb){ if(fabs(v)>thr){ sumsq+=v*v; cnt++; } }
			double P_sig = (cnt>0)? sumsq/(double)cnt : 0.0;
			double f_nyquist = sampling_frequency/2.0;
			double sigma = sqrt(2.0 * P_sig * f_nyquist / (pow(10.0, wav_esn0/10.0) * bandwidth));
			int conv = env_i("MERCURY_BIGBLOCK_WAV_AWGN_CONV", 0);
			double per_sample = (conv==1) ? sigma : (sigma/sqrt(2.0));
			awgn_channel.set_seed(rand());
			for(size_t i=0;i<rxpb.size();i++) rxpb[i] += per_sample * awgn_channel.awgn_value_generator();
			std::cout << "[DIAG-WAV] STEP1 AWGN label=" << wav_esn0 << " conv=" << conv
			          << " P_sig=" << P_sig << " sigma=" << sigma
			          << " per_sample_std=" << per_sample
			          << " (signal_samples=" << cnt << "/" << rxpb.size() << ")" << std::endl;
		}
	}

	int saved_Nsymb = ofdm.Nsymb;
	int Nc=0, Nfft=ofdm.Nfft; float gi=ofdm.gi;
	int Ngi=(int)round((double)gi*(double)Nfft);
	int Nofdm = Nfft + Ngi;
	int interp = frequency_interpolation_rate;
	int sym_samples = Nofdm * interp;
	int pre_nSymb = data_container.preamble_nSymb;

	int Ngrid=0, log2M=0, nBits=0;
	int nData = bigblock_rebuild_thin_grid(Ngrid, log2M, nBits);
	Nc = ofdm.Nc;

	unsigned int seed = (unsigned int)env_i("MERCURY_BIGBLOCK_SEED", 12345);
	int kcap = env_i("MERCURY_BIGBLOCK_K", 0);
	std::vector<int> tx_bits; std::vector<std::vector<int>> cw_info;
	int Kcw = bigblock_build_tx_bits(ldpc, nBits, seed, kcap, tx_bits, cw_info);

	// --- ACQUIRE the head preamble: real Schmidl-Cox over the whole rx buffer. ---
	// We reuse the production acquisition by placing the buffer in the receive path
	// and reading the detected delay. The block (preamble + Ngrid data symbols) must
	// fit inside the RX buffer window; size buffer_Nsymb to cover lead+block.
	int block_syms = pre_nSymb + Ngrid;
	int need_syms = (int)(rxpb.size() / sym_samples) + 2;
	// passband_to_baseband over the whole rx, decimate, then Schmidl-Cox.
	// We mirror receive_byte's front end directly (no ARQ state).
	long head_delay = -1; double head_metric = 0.0;
	{
		int buf_syms = need_syms;
		int buf_interp = Nofdm * buf_syms * interp;
		std::vector<double> pad(buf_interp, 0.0);
		int copy_n = (int)rxpb.size(); if(copy_n > buf_interp) copy_n = buf_interp;
		for(int i=0;i<copy_n;i++) pad[i]=rxpb[i];
		// baseband (interp rate) + decimated buffers
		std::vector<std::complex<double>> bb_interp(buf_interp);
		ofdm.passband_to_baseband(pad.data(), buf_interp, bb_interp.data(),
		                          sampling_frequency, carrier_frequency, carrier_amplitude, 1,
		                          &ofdm.FIR_rx_time_sync);
		std::vector<std::complex<double>> bb_dec(Nofdm*buf_syms);
		ofdm.rational_resampler(bb_interp.data(), buf_interp, bb_dec.data(), interp, DECIMATION);
		// Coarse (GI-stride, decimated) then fine (full-rate slice) — like receive_byte.
		TimeSyncResult coarse = ofdm.time_sync_preamble_halfsym(
			bb_dec.data(), Nofdm*buf_syms, 1, 1, 0.0, pre_nSymb);
		// MATCHED-FILTER SNAP (see bigblock_preamble_mf_snap): disambiguate the Schmidl-Cox
		// plateau (the noise-fragile ±half-symbol argmax flip that half-symbols the FFT
		// window and kills the decode). Sharp single peak at the true preamble start.
		bool mfsnap = (std::getenv("MERCURY_BIGBLOCK_MFSNAP")==NULL || atoi(std::getenv("MERCURY_BIGBLOCK_MFSNAP"))!=0);
		long coarse_dec = coarse.delay;
		if(mfsnap)
			coarse_dec = bigblock_preamble_mf_snap(bb_dec.data(), Nofdm*buf_syms,
			                                       coarse.delay, pre_nSymb, Nofdm, Nc, Nofdm);
		long coarse_full = (long)coarse_dec * interp;
		long fine_delay_dbg = -1;
		if(mfsnap)
		{
			// MF snap locked the true start; SKIP the plateau-prone fine SC re-correlation
			// (re-flips to the spurious lobe). Demod pilot-EVM ±GI recovers the residual.
			head_delay  = coarse_full;
			head_metric = coarse.correlation;
		}
		else
		{
			long slice_start = coarse_full - 2*sym_samples; if(slice_start<0) slice_start=0;
			long slice_size  = (long)(pre_nSymb+4)*sym_samples;
			if(slice_start+slice_size > buf_interp) slice_size = buf_interp - slice_start;
			TimeSyncResult fine = ofdm.time_sync_preamble_halfsym(
				&bb_interp[slice_start], (int)slice_size, interp, 1, 0.0, pre_nSymb);
			head_delay  = slice_start + fine.delay; fine_delay_dbg = fine.delay;
			head_metric = fine.correlation;
			if(head_metric < coarse.correlation*0.5){ head_delay=coarse_full; head_metric=coarse.correlation; }
		}
		if(env_i("MERCURY_BIGBLOCK_RXPB_DIAG",0))
			std::cout << "[DIAG-WAV-ACQ] coarse.delay=" << coarse.delay << " coarse_dec_snapped=" << coarse_dec
			          << " coarse_full=" << coarse_full << " coarse.corr=" << coarse.correlation
			          << " fine.delay=" << fine_delay_dbg << " head_delay=" << head_delay
			          << " head_metric=" << head_metric
			          << " sym_samples=" << sym_samples << std::endl;
	}
	if(head_delay < 0){
		std::cout << "[BIGBLOCK-WAV] DECODE acquisition FAILED (no preamble found)" << std::endl;
		ofdm.deinit(); ofdm.pilot_configurator.Dx=1; ofdm.pilot_configurator.Dy=3;
		ofdm.pilot_configurator.pilot_density=HIGH_DENSITY; ofdm.channel_estimator=LEAST_SQUARE;
		ofdm.channel_estimator_amplitude_restoration=NO; ofdm.LS_window_width=0; ofdm.LS_window_hight=0;
		ofdm.init(Nfft, Nc, saved_Nsymb, gi); return 0;
	}

	// --- DEMOD: from the data start (head + preamble), rebuild the Ngrid grid. ---
	// Mirror the production data extraction (telecom_system.cc:2434-2482): process a
	// slice that includes a FIR_rx_data warm-up margin BOTH BEFORE and AFTER the window
	// (so the FIR start transient lands in the LEFT margin and the FIR epilogue lands in
	// the RIGHT margin, not in the data), then skip the left margin. The margin is a
	// whole number of symbols so the demod symbol grid stays aligned. BB-1 fix: the
	// trailing (right) margin was previously omitted, truncating the tail of the final
	// OFDM symbol (see :2480 pb_end = extraction_delay + frame_size + fir_margin).
	// sample_offset = the slice's absolute passband start so the downconvert oscillator
	// phase is CONTINUOUS with the TX upconvert (which ran from passband sample 0).
	long data_start0 = head_delay + (long)pre_nSymb*sym_samples;
	int span_interp = Nofdm*Ngrid*interp;
	int fir_margin  = ofdm.FIR_rx_data.filter_nTaps * interp;
	int margin_syms = (fir_margin + sym_samples - 1) / sym_samples;
	int margin_interp = margin_syms * sym_samples;
	float power_normalization = sqrt((double)ofdm.Nfft);

	// Demod the Ngrid grid for a given window offset (full-rate samples) into rx.
	auto demod_at = [&](long data_start, std::vector<std::complex<double>>& rx)
	{
		long ms = data_start - margin_interp;
		int  mi = margin_interp;
		if(ms < 0){ ms = 0; mi = (int)data_start; }
		// BB-1 (PIPELINE_AUDIT BB-1): trailing FIR margin too (mirror per-frame :2480).
		// Bounds-guarded read below zero-pads past rxpb end; grid extraction still starts
		// at mdec=mi/interp for Nofdm*Ngrid samples, so the right margin only feeds the
		// FIR lookahead and does NOT shift the symbol grid.
		int slice_size = mi + span_interp + margin_interp;
		std::vector<double> dat_pb(slice_size, 0.0);
		for(int i=0;i<slice_size;i++){
			long src=ms+i; dat_pb[i] = (src>=0 && src<(long)rxpb.size()) ? rxpb[src] : 0.0; }
		std::vector<std::complex<double>> dat_bb_interp(slice_size);
		ofdm.passband_to_baseband(dat_pb.data(), slice_size, dat_bb_interp.data(),
		                          sampling_frequency, carrier_frequency, carrier_amplitude, 1,
		                          &ofdm.FIR_rx_data, (int)ms);
		std::vector<std::complex<double>> dat_bb_full((size_t)(slice_size/interp) + 1);
		int dec_total = slice_size / interp;
		ofdm.rational_resampler(dat_bb_interp.data(), slice_size, dat_bb_full.data(), interp, DECIMATION);
		std::vector<std::complex<double>> dat_bb(Nofdm*Ngrid);
		int mdec = mi / interp;
		for(int j=0;j<Nofdm*Ngrid && (mdec+j)<dec_total;j++) dat_bb[j] = dat_bb_full[mdec + j];
		for(int j=0;j<Nofdm*Ngrid;j++) dat_bb[j] *= power_normalization;
		rx.assign((size_t)Ngrid*Nc, std::complex<double>(0,0));
		for(int i=0;i<Ngrid;i++) ofdm.symbol_demod(&dat_bb[(size_t)i*Nofdm], &rx[(size_t)i*Nc]);
	};

	// Pilot-residual EVM for a grid (flat-ML residual; lower = better window placement).
	auto pilot_evm = [&](const std::vector<std::complex<double>>& rx)->double
	{
		std::complex<double> Hsum(0,0); int pidx=0, np=0;
		for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
			if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT){
				std::complex<double> X=ofdm.pilot_configurator.sequence[pidx++];
				Hsum += rx[(size_t)n*Nc+j]/X; np++; }
		std::complex<double> Hbar=(np>0)?Hsum/(double)np:std::complex<double>(1,0);
		// per-symbol CPE/PEG-free residual: use |rx/X - Hbar*(per-symbol phase)|? Keep it
		// simple — residual against the global mean magnitude (timing ISI raises it).
		double e=0.0; pidx=0; int cnt=0;
		for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
			if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT){
				std::complex<double> X=ofdm.pilot_configurator.sequence[pidx++];
				std::complex<double> r=rx[(size_t)n*Nc+j]/X;
				double dr=std::abs(r)-std::abs(Hbar); e+=dr*dr; cnt++; }
		return cnt? e/cnt : 1e9;
	};

	// FINE TIMING SEARCH (principled OFDM fine-timing): Schmidl-Cox lands the window at
	// integer-sample resolution and the time-sync vs data FIR + carrier path leave a
	// FIXED few-sample residual (loopback showed optimum ~6 samp earlier). Rather than a
	// magic constant, search a small ±window for the offset MINIMIZING pilot-EVM. This is
	// self-calibrating across the FIR/oscillator path AND robust to real-HW jitter — the
	// data window lands ISI-free regardless of the acquisition's integer pick. Override
	// MERCURY_BIGBLOCK_TADJ forces a fixed nudge (diagnostic); _TSEARCH=0 disables.
	long data_start = data_start0;
	int tadj_force = env_i("MERCURY_BIGBLOCK_TADJ", 0);
	int gi_interp  = Ngi * interp;
	std::vector<std::complex<double>> rx;
	if(tadj_force != 0)
	{
		data_start = data_start0 + tadj_force;
		demod_at(data_start, rx);
	}
	else if(env_i("MERCURY_BIGBLOCK_TSEARCH", 1) != 0)
	{
		double best_e = 1e18; long best_off = 0;
		std::vector<std::complex<double>> rx_try;
		// search a window that comfortably spans the GI back-off region.
		int lo = -gi_interp + 1, hi = gi_interp/4;
		int step = env_i("BIGBLOCK_TSEARCH_STEP", 2);
		for(int off=lo; off<=hi; off+=step)
		{
			demod_at(data_start0 + off, rx_try);
			double e = pilot_evm(rx_try);
			if(e < best_e){ best_e = e; best_off = off; rx = rx_try; }
		}
		data_start = data_start0 + best_off;
		if((int)rx.size() != Ngrid*Nc) demod_at(data_start, rx);
		if(env_i("MERCURY_BIGBLOCK_DIAG",0))
			std::cout << "[BIGBLOCK-WAV]   fine_timing best_off=" << best_off
			          << " (full-rate samp, "<<(best_off/(double)interp)<<" dec) pilotEVM=" << best_e << std::endl;
	}
	else
	{
		demod_at(data_start, rx);
	}

	// --- CPE/PEG residual-timing de-rotation (the §13/§14 STEP-2 tracker) --------
	// Schmidl-Cox lands the FFT window at INTEGER-sample resolution; any residual
	// sub-sample / few-sample timing error (the loopback showed the optimum window is
	// ~6 samples earlier than the acquired delay) appears as a per-symbol linear phase
	// ramp across carriers (slope ∝ timing error). The thin lattice's CONTINUAL columns
	// give ≥2 pilots/symbol → a per-symbol LS line-fit (intercept=CPE ω, slope=PEG δ)
	// removes that ramp, exactly as the validated tracker does for SFO. Default ON
	// (MERCURY_BIGBLOCK_TRACK=1); the sparse-2D per-symbol estimate handles slow drift,
	// but a CONSTANT few-sample window offset needs the explicit slope fit because the
	// dx4 scatter undersamples a steep ramp in the band interior. This is principled
	// (Speth/Fechtel/Meyr residual-timing correction), not a magic alignment constant.
	bool track = (env_i("MERCURY_BIGBLOCK_TRACK", 1) != 0);
	int  twin  = env_i("MERCURY_BIGBLOCK_TRACK_WIN", 9);
	if(track)
	{
		std::vector<double> sym_omega(Ngrid,0.0), sym_delta(Ngrid,0.0);
		std::vector<int> pidx_at_symbol(Ngrid,0);
		{ int pidx=0; for(int n=0;n<Ngrid;n++){ pidx_at_symbol[n]=pidx;
			for(int j=0;j<Nc;j++) if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT) pidx++; } }
		for(int n=0;n<Ngrid;n++)
		{
			int pidx = pidx_at_symbol[n];
			double Sx=0,Sy=0,Sxx=0,Sxy=0; int np=0;
			for(int j=0;j<Nc;j++)
				if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT)
				{
					std::complex<double> X = ofdm.pilot_configurator.sequence[pidx++];
					std::complex<double> r = rx[(size_t)n*Nc+j] / X;
					double phi = atan2(r.imag(), r.real());
					double x=(double)j; Sx+=x; Sy+=phi; Sxx+=x*x; Sxy+=x*phi; np++;
				}
			if(np>=2){ double den=np*Sxx-Sx*Sx;
				if(fabs(den)>1e-12){ sym_delta[n]=(np*Sxy-Sx*Sy)/den; sym_omega[n]=(Sy-sym_delta[n]*Sx)/np; } }
		}
		for(int n=0;n<Ngrid;n++)
		{
			double om=0,dl=0; int cnt=0;
			for(int w=n-twin/2; w<=n+twin/2; w++) if(w>=0&&w<Ngrid){ om+=sym_omega[w]; dl+=sym_delta[w]; cnt++; }
			if(cnt>0){ om/=cnt; dl/=cnt; }
			for(int j=0;j<Nc;j++){ double ph=-(om+dl*(double)j);
				rx[(size_t)n*Nc+j] *= std::complex<double>(cos(ph), sin(ph)); }
		}
	}

	// --- CHANNEL-ADAPTIVE ESTIMATE (sparse-2D default; flat-ML control) ---------
	bool sparse2d = (env_i("MERCURY_BIGBLOCK_SPARSE2D", 1) != 0);
	if(sparse2d)
	{
		grid_sparse2d_estimator(rx.data(), Ngrid, Nc);
	}
	else
	{
		// flat-ML control (one global H̄ = mean(Y/X) over pilots) — exact on flat.
		std::complex<double> Hsum(0,0); int pidx=0; int npil=0;
		for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
			if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT){
				std::complex<double> X = ofdm.pilot_configurator.sequence[pidx++];
				Hsum += rx[(size_t)n*Nc+j] / X; npil++; }
		std::complex<double> Hbar = (npil>0)?(Hsum/(double)npil):std::complex<double>(1,0);
		double nsum=0.0; pidx=0;
		for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
			if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT){
				std::complex<double> X = ofdm.pilot_configurator.sequence[pidx++];
				std::complex<double> resid = rx[(size_t)n*Nc+j]-Hbar*X;
				nsum += resid.real()*resid.real()+resid.imag()*resid.imag(); }
		for(int ci=0;ci<Ngrid*Nc;ci++){ (ofdm.estimated_channel+ci)->value=Hbar; (ofdm.estimated_channel+ci)->status=MEASURED; }
		ofdm.noise_variance_estimate = (npil>0)?(nsum/(double)npil):0.01;
		if(ofdm.noise_variance_estimate<1e-6) ofdm.noise_variance_estimate=1e-6;
	}

	// per-data-carrier CSI weight |H|^2 (deframed raster order) BEFORE equalize.
	std::vector<double> csi_data(nData, 1.0);
	{
		int di=0;
		for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
			if((ofdm.ofdm_frame+n*Nc+j)->type==DATA){
				std::complex<double> H=(ofdm.estimated_channel+n*Nc+j)->value;
				if(di<nData) csi_data[di]=H.real()*H.real()+H.imag()*H.imag(); di++; }
	}

	std::vector<std::complex<double>> eq((size_t)Ngrid*Nc);
	ofdm.channel_equalizer(rx.data(), eq.data());
	std::vector<std::complex<double>> deframed(nData);
	ofdm.deframer(eq.data(), deframed.data());

	// 1C — time/freq symbol DE-interleave (WAV decode; mirror of bigblock_rx_passband).
	// Restore codeword-contiguous order before demap; deinterleave CSI the same way. NO-OP B==1.
	{
		int B = bigblock_tf_block_size(nData);
		if(B > 1){
			std::vector<std::complex<double>> dil(nData);
			deinterleaver(deframed.data(), dil.data(), nData, B);
			deframed.swap(dil);
			std::vector<double> cil(nData);
			int nBlocks = nData / B;
			for(int i=0;i<nBlocks;i++)
				for(int j=0;j<B;j++)
					cil[(size_t)i*B+j] = csi_data[(size_t)j*nBlocks+i];
			for(int i=nBlocks*B;i<nData;i++) cil[i] = csi_data[i];
			csi_data.swap(cil);
		}
	}

	// [DIAG-WAV] instrument nv, mean|H|, post-EQ deframed constellation RMS.
	if(env_i("MERCURY_BIGBLOCK_RXPB_DIAG",0))
	{
		double Hmag=0.0; for(int ci=0;ci<Ngrid*Nc;ci++){ std::complex<double> H=(ofdm.estimated_channel+ci)->value; Hmag+=std::abs(H);} Hmag/=(Ngrid*Nc);
		double crms=0.0; for(int d=0;d<nData;d++) crms+=std::norm(deframed[d]); crms=sqrt(crms/(nData>0?nData:1));
		double rxrms=0.0; int dn=0; for(int n=0;n<Ngrid;n++)for(int j=0;j<Nc;j++) if((ofdm.ofdm_frame+n*Nc+j)->type==DATA){ rxrms+=std::norm(rx[(size_t)n*Nc+j]); dn++; } rxrms=sqrt(rxrms/(dn>0?dn:1));
		// ACHIEVED per-subcarrier Es/N0: signal energy per data carrier (mean |H*X|^2 over
		// pilots = noiseless per-SC symbol energy) over nv (per-SC noise variance). This is
		// the axis the LDPC waterfall lives on (sfo_grid_test EsN0). Compares the passband
		// validator's full-band Es/N0 LABEL to the per-SC Es/N0 the code actually sees.
		double sig_sc=0.0; int pidx_d=0, npd=0;
		for(int n=0;n<Ngrid;n++)for(int j=0;j<Nc;j++) if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT){
			std::complex<double> X=ofdm.pilot_configurator.sequence[pidx_d++];
			std::complex<double> H=(ofdm.estimated_channel+n*Nc+j)->value;
			sig_sc += std::norm(H)*std::norm(X); npd++; }
		sig_sc = npd? sig_sc/npd : 0.0;
		double nv_sc = ofdm.noise_variance_estimate;
		double persc_esn0_db = (nv_sc>0.0)? 10.0*log10(sig_sc/nv_sc) : 999.0;
		std::cout << "[DIAG-WAV] nv=" << ofdm.noise_variance_estimate
		          << " mean|H|=" << Hmag
		          << " deframed_rms=" << crms
		          << " rx_data_rms=" << rxrms
		          << " sig_per_SC=" << sig_sc
		          << " achieved_perSC_EsN0_dB=" << persc_esn0_db << std::endl;
	}

	// --- LLR + CSI weighting + LDPC per codeword (mirrors sfo_grid_test coded) ---
	std::vector<float> clr(nBits);
	double cvar = ofdm.noise_variance_estimate; if(cvar<1e-9) cvar=1e-9;
	psk.demod(deframed.data(), nBits, clr.data(), (float)cvar);
	{
		double mean_w=0.0; for(int d=0;d<nData;d++) mean_w+=csi_data[d];
		mean_w=(nData>0)?mean_w/(double)nData:1.0; if(mean_w<1e-9) mean_w=1.0;
		for(int d=0;d<nData;d++){ double w=csi_data[d]/mean_w;
			for(int b=0;b<log2M;b++){ size_t bi=(size_t)d*log2M+b; if(bi>=(size_t)nBits) break;
				float v=clr[bi]*(float)w; if(v>40.0f)v=40.0f; else if(v<-40.0f)v=-40.0f; clr[bi]=v; } }
	}

	int cw_ok=0; long cw_infoerr=0, cw_infobits=0; long iter_sum=0; int iter_min=1<<30, iter_max=-1;
	bool diag = (env_i("MERCURY_BIGBLOCK_DIAG",0)!=0);
	std::vector<float> cwllr(ldpc.N); std::vector<int> dec(ldpc.N);
	for(int c=0;c<Kcw;c++)
	{
		for(int i=0;i<ldpc.N;i++) cwllr[i]=clr[(size_t)c*ldpc.N+i];
		int iters = ldpc.decode(cwllr.data(), dec.data());
		iter_sum += iters; if(iters<iter_min) iter_min=iters; if(iters>iter_max) iter_max=iters;
		int ierr=0;
		for(int i=0;i<ldpc.K;i++){ cw_infobits++; if(dec[i]!=cw_info[c][i]){ ierr++; cw_infoerr++; } }
		if(ierr==0) cw_ok++;
		if(diag) std::cout << "[BIGBLOCK-WAV]   cw=" << c << " iters=" << iters
		                   << " infoerr=" << ierr << "/" << ldpc.K
		                   << " decoded=" << (ierr==0?1:0) << std::endl;
	}
	double post_fec_ber = cw_infobits? (double)cw_infoerr/(double)cw_infobits : 1.0;

	std::cout << "[BIGBLOCK-WAV] DECODE cfg=" << current_configuration
	          << " Nsymb=" << Ngrid << " Nc=" << Nc
	          << " pilots=" << ofdm.pilot_configurator.nPilots
	          << " (" << (100.0*ofdm.pilot_configurator.nPilots/(double)(Ngrid*Nc)) << "%)"
	          << " est=" << (sparse2d?"SPARSE-2D":"FLAT-ML")
	          << " seed=" << seed << std::endl;
	std::cout << "[BIGBLOCK-WAV] DECODE acq_delay=" << head_delay
	          << " acq_metric=" << head_metric
	          << " data_window=" << data_start << " (off=" << (data_start-data_start0) << ")"
	          << " nv=" << ofdm.noise_variance_estimate
	          << " iter[min/mean/max]=" << iter_min << "/"
	          << (Kcw?(double)iter_sum/Kcw:0.0) << "/" << iter_max << std::endl;
	std::cout << "[BIGBLOCK-WAV] ===== RESULT =====" << std::endl;
	std::cout << "[BIGBLOCK-WAV]   codewords_decoded=" << cw_ok << "/" << Kcw
	          << "  post_FEC_BER=" << post_fec_ber
	          << "  infoerr=" << cw_infoerr << "/" << cw_infobits << std::endl;
	std::cout << "[BIGBLOCK-WAV]   VERDICT=" << ((cw_ok==Kcw && Kcw>0) ? "PASS(8/8 clean)" : "FAIL")
	          << std::endl;

	// teardown
	ofdm.deinit();
	ofdm.pilot_configurator.Dx=1; ofdm.pilot_configurator.Dy=3;
	ofdm.pilot_configurator.pilot_density=HIGH_DENSITY;
	ofdm.channel_estimator=LEAST_SQUARE;
	ofdm.channel_estimator_amplitude_restoration=NO;
	ofdm.LS_window_width=0; ofdm.LS_window_hight=0;
	ofdm.init(Nfft, Nc, saved_Nsymb, gi);
	return (cw_ok==Kcw && Kcw>0) ? 1 : 0;
}

// P2.2 — stock RX passband normalization + impulse-noise blanking, factored out of
// receive_byte (was inlined at telecom_system.cc:1081-1124). The OFDM estimator/
// equalizer/LLR pipeline assumes the RX passband sits near the TX output_power level;
// external paths attenuate 20-50 dB. This rescales RMS to sqrt(output_power)*0.5 and
// blanks impulses at 10×RMS — EXACTLY what the per-frame path runs. The big-block RX
// (receive_bigblock) skipped it, so the live AWGN validator decoded garbage at 30 dB
// (BER 0.43). Both paths now call this; the extraction is byte-identical to the inline
// block (same code, same pb_samples) so the stock per-frame path is unchanged (verified
// by --test-partial-bsi-advance=ofdm/mfsk + the bigblock clean live validator, all
// green). MFSK is excluded (the per-frame block is guarded on M!=MFSK).
void cl_telecom_system::rx_passband_normalize_and_blank(double* pb, int pb_samples)
{
	if(M == MOD_MFSK) return;
	if(pb == NULL || pb_samples <= 0) return;
	double sum_sq = 0.0;
	for(int i = 0; i < pb_samples; i++)
		sum_sq += pb[i] * pb[i];
	double rms = sqrt(sum_sq / pb_samples);
	double rms_in = rms; double applied_scale = 1.0;
	if(rms > 1e-8) {
		// Phase-2: --rx-normalize=off bypasses this auto-rescaling block.
		if(rx_normalize_enabled) {
			// Target RMS: sqrt(output_power / 2) for passband signal
			// (factor /2 because passband has carrier modulation overhead)
			double target_rms = sqrt(output_power_Watt) * 0.5;
			double scale = target_rms / rms;
			// Clamp scale to prevent insane amplification on near-silence
			if(scale > 10000.0) scale = 10000.0;
			if(scale < 0.001) scale = 0.001;
			// Only normalize if significantly off (>3 dB)
			if(scale > 1.5 || scale < 0.67)
			{
				for(int i = 0; i < pb_samples; i++)
					pb[i] *= scale;
				// Recalculate RMS after scaling
				rms *= scale; applied_scale = scale;
			}
		}
		// Impulse noise blanking: clip at 10× RMS (always on)
		double clip_threshold = 10.0 * rms;
		long nclip=0;
		for(int i = 0; i < pb_samples; i++) {
			if(pb[i] > clip_threshold) { pb[i] = clip_threshold; nclip++; }
			else if(pb[i] < -clip_threshold) { pb[i] = -clip_threshold; nclip++; }
		}
		if(std::getenv("MERCURY_BIGBLOCK_RXPB_DIAG"))
			std::cout << "[DIAG-NORM] pb_samples=" << pb_samples << " rms_in=" << rms_in
			          << " scale=" << applied_scale << " final_rms=" << rms
			          << " clip_thr=" << clip_threshold
			          << " nclip=" << nclip << " norm_enabled=" << rx_normalize_enabled
			          << " target=" << (sqrt(output_power_Watt)*0.5) << std::endl;
	}
}

// ===== P1: LIVE-PATH BIG-BLOCK ENTRY POINTS =====
// These wire the validated big-block PHY (the shared bigblock_*_passband workers)
// into the production transmit_byte/receive_byte. They are reached ONLY when
// bigblock_framing_enabled is set (a CFG16-rung framing-mode bit; default OFF), so
// the stock per-frame path is byte-identical when the flag is clear.
//
// RISK-1 (harness divergence): the workers rebuild the thin lattice via
// bigblock_rebuild_thin_grid() (which deinit()/init()s ofdm to Nsymb=Ngrid). In P1
// — with NO ARQ change and the loopback validator driving a single block per
// direction — these methods OWN their teardown (restore the stock CFG16 ofdm grid),
// so the OFDM config the rest of the modem sees is unchanged after the call. P2 moves
// the rebuild to config-load (once per big-block-framing election) so a multi-block
// ARQ session does not deinit/init per block; that is the documented P2 hot-path fix,
// NOT a P1 change.
//
// HARNESS-HID BUG (surfaced by the live-path refactor): the WAV harness's teardown
// (bigblock_tx_to_wav/decode_from_wav) hand-restores ONLY a few pilot_configurator
// fields (Dx/Dy/density/estimator) after deinit() — but deinit() ALSO zeros
// first_row/last_row/first_col/second_col/last_col/boost (ofdm.cc:207-219). The
// partial restore leaves those stale, and the subsequent ofdm.init()->
// pilot_configurator.configure() faults (integer div, exit 0xC0000094). In the WAV
// harness this is invisible because the teardown runs at process exit AFTER the
// VERDICT prints, so it never blocked the result (verified: baseline b619423 crashes
// here too). On the LIVE path the modem must KEEP RUNNING (TX then RX then stock
// frames), so a clean restore is mandatory. ROOT-CAUSE FIX: do NOT hand-restore the
// pilot configurator — force a FULL stock config reload (CONFIG_NONE -> load) which
// sets EVERY pilot field from the config defaults and re-inits cleanly. This is the
// exact path every real config switch uses (validated).
void cl_telecom_system::bigblock_restore_stock_config()
{
	int stock = current_configuration;

	// RX-RING PRESERVATION (data-flow-sim2-ofdm-delivery-cadence.md §8) — a geometry-
	// derivation helper must NEVER destroy the live RX accumulation buffer.
	//
	// The CONFIG_NONE -> load_configuration(stock) reload below forces a FULL reinit
	// (telecom_system.cc:8664-8674) -> data_container.set_size(), which deinit()s and
	// REALLOCS + memset-ZEROES passband_delayed_data + ready_to_process_passband_delayed_data
	// (data_container.cc:170-173). On the continuous-stream production RX this is harmless
	// (the helper fires only at quiescent inter-block / TX-turnaround boundaries; CLAUDE.md
	// data-flow audit §8.3 INV-2). But under the in-process SIM single-symbol pacing the
	// next block's samples are ALREADY accumulated in the ring when a control-turnaround
	// restore (arq_responder.cc:1213) fires -> the realloc wipes the in-flight block and the
	// subsequent decode snapshots rms=0 SILENCE -> cw0-CRC fails -> 0 bytes delivered.
	//
	// Restore to the SAME stock config => the data_container ring dimensions
	// (Nofdm * buffer_Nsymb * interp) are IDENTICAL before and after, so a snapshot+restore
	// of the ring contents + accumulation bookkeeping across the reload is always valid.
	// This makes the helper non-destructive to shared RX state on BOTH paths (defense-in-
	// depth on production: a no-op when the ring was already quiescent, since saved==loaded).
	int sp_full = 2 * data_container.Nofdm * data_container.buffer_Nsymb * frequency_interpolation_rate;
	int sp_rtp  =     data_container.Nofdm * data_container.buffer_Nsymb * frequency_interpolation_rate;
	std::vector<double> save_pdd, save_rtp;
	int  save_rwi = data_container.ring_write_index;
	int  save_ftr = data_container.frames_to_read.load();
	int  save_dr  = data_container.data_ready;
	int  save_nupe= data_container.nUnder_processing_events.load();
	bool ring_saved = false;
	if(sp_full > 0 && data_container.passband_delayed_data != NULL
	   && data_container.ready_to_process_passband_delayed_data != NULL)
	{
		save_pdd.assign(data_container.passband_delayed_data, data_container.passband_delayed_data + sp_full);
		save_rtp.assign(data_container.ready_to_process_passband_delayed_data,
		                data_container.ready_to_process_passband_delayed_data + sp_rtp);
		ring_saved = true;
	}

	// CONFIG_NONE forces load_configuration's full-reinit branch (all pilot fields set
	// from defaults), avoiding the early-return when configuration==current.
	current_configuration = CONFIG_NONE;
	load_configuration(stock);

	// Restore the RX accumulation buffer + bookkeeping the realloc zeroed. Dimensions
	// are unchanged (same stock config), so the saved extents fit the reallocated buffers.
	if(ring_saved)
	{
		int sp_full2 = 2 * data_container.Nofdm * data_container.buffer_Nsymb * frequency_interpolation_rate;
		int sp_rtp2  =     data_container.Nofdm * data_container.buffer_Nsymb * frequency_interpolation_rate;
		if(sp_full2 == sp_full && sp_rtp2 == sp_rtp
		   && data_container.passband_delayed_data != NULL
		   && data_container.ready_to_process_passband_delayed_data != NULL)
		{
			memcpy(data_container.passband_delayed_data, save_pdd.data(), (size_t)sp_full * sizeof(double));
			memcpy(data_container.ready_to_process_passband_delayed_data, save_rtp.data(), (size_t)sp_rtp * sizeof(double));
			data_container.ring_write_index = save_rwi;
			data_container.frames_to_read   = save_ftr;
			data_container.data_ready       = save_dr;
			data_container.nUnder_processing_events = save_nupe;
		}
	}
}

int cl_telecom_system::bigblock_tx_total_samples()
{
	// preamble + K*frame passband samples at the frozen layout. Computed without a
	// full TX by re-deriving Ngrid + K from the lattice rebuild, then restoring stock.
	if(M == MOD_MFSK) return 0;
	int Nfft=ofdm.Nfft; float gi=ofdm.gi;
	int Ngi=(int)round((double)gi*(double)Nfft);
	int Nofdm = Nfft + Ngi;
	int interp = frequency_interpolation_rate;
	int pre_nSymb = data_container.preamble_nSymb;
	int Ngrid=0, log2M=0, nBits=0;
	bigblock_rebuild_thin_grid(Ngrid, log2M, nBits);
	int total = (Nofdm*pre_nSymb + Nofdm*Ngrid) * interp;
	bigblock_restore_stock_config();
	return total;
}

int cl_telecom_system::bigblock_rx_block_nsymb()
{
	// USE-AFTER-FREE / PARTIAL-BLOCK FIX (bigblock-whiten-align): symbols ONE big-block
	// spans on the wire = preamble_nSymb + Ngrid (the thin-grid data-symbol count for the
	// WHOLE K-codeword block). The live RX arms frames_to_read to this so the decode
	// snapshot fires only AFTER the full block is captured (mirrors bigblock_tx_total_samples
	// which returns the SAME extent in samples: (Nofdm*pre + Nofdm*Ngrid)*interp). Geometry-
	// only: rebuild thin grid -> read Ngrid -> restore stock CFG16.
	if(M == MOD_MFSK) return 0;
	int pre_nSymb = data_container.preamble_nSymb;
	int Ngrid=0, log2M=0, nBits=0;
	bigblock_rebuild_thin_grid(Ngrid, log2M, nBits);
	bigblock_restore_stock_config();
	if(Ngrid <= 0) return 0;
	return pre_nSymb + Ngrid;
}

int cl_telecom_system::bigblock_codeword_count()
{
	// SACK-GATE P1 (R-B): the big-block codeword count K at the current CFG16 rung.
	// IDENTICAL geometry to the TX/RX workers (transmit_bigblock:7832,
	// receive_bigblock:7890): rebuild the thin grid, K = nBits/ldpc.N, capped by
	// MERCURY_BIGBLOCK_K. The ARQ batch-size election PINS data_batch_size = K on
	// BOTH peers from THIS one source so they cannot diverge (bug #9). Geometry-only
	// (no I/O); rebuild then restore stock CFG16 (same as bigblock_tx_total_samples).
	if(M == MOD_MFSK) return 0;
	int Ngrid=0, log2M=0, nBits=0;
	bigblock_rebuild_thin_grid(Ngrid, log2M, nBits);
	int K = (ldpc.N > 0) ? (nBits / ldpc.N) : 0;
	{ const char* e = std::getenv("MERCURY_BIGBLOCK_K");
	  if(e && *e){ int kcap = atoi(e); if(kcap > 0 && kcap < K) K = kcap; } }
	bigblock_restore_stock_config();
	if(K < 0) K = 0;
	return K;
}

void cl_telecom_system::bigblock_whiten_bits(int* bits, int nbits)
{
	// public wrapper so the ARQ RX (bigblock_receive_carve) can de-whiten the decoded
	// info bits with the SAME PRBS the real-bytes TX applied. Self-inverse XOR.
	bigblock_whiten_payload_bits(bits, nbits, BIGBLOCK_WHITEN_SEED);
}

void cl_telecom_system::transmit_bigblock(int* data, int nBytes, double* out)
{
	// P2.1 — feed REAL ARQ bytes as the block's systematic info bits. When the ARQ
	// layer hands a payload (data != null, nBytes > 0), pack the K=8 codewords from
	// those bytes (LSB-first, the byte_to_bit convention) into the worker's external
	// payload_bits; the worker LDPC-encodes them. When nBytes == 0 (the P1 loopback
	// validator, which passes a dummy 0-byte payload), fall back to the seeded-PRBS
	// known payload so the byte-correct loopback gate is unchanged.
	//
	// The payload bit buffer must be K*ldpc.K bits; K = nBits/ldpc.N at the thin grid.
	// Derive K via the same rebuild+restore the worker uses (cheap; integer geometry),
	// so we size the buffer exactly. Bytes beyond nBytes (or beyond the block payload
	// capacity) are zero-padded; bytes past capacity are dropped (the ARQ layer sizes a
	// block to the capacity, so this only guards a mis-sized caller).
	std::vector<std::vector<int>> cw_info;
	int nSamples = 0;
	int Kcw = 0;

	// HEAP-OVERRUN ROOT-CAUSE FIX (fact-doc §13): the block waveform is preamble + K*frame
	// passband samples (~45552 doubles at the CFG16 thin grid) — 3-5x one stock OFDM frame
	// slot. bigblock_tx_passband writes that whole extent into `out`. If a (declined /
	// control / per-frame) caller routed here with a FRAME-sized `out`, the write smashed
	// adjacent heap chunks (the original "free(): invalid next size" abort). Compute the
	// required extent from the REAL block geometry and HARD-REFUSE (no write) when it
	// exceeds the caller-declared capacity. This fires on ANY platform regardless of the
	// OS allocator's slack — it is the deterministic bounds check the reproducer asserts on.
	int required_samples = bigblock_tx_total_samples();   // rebuild+restore internally
	if(bigblock_emit_out_capacity > 0 && required_samples > bigblock_emit_out_capacity)
	{
		std::cout << "[BIGBLOCK-TX-GUARD] REFUSED: block needs " << required_samples
		          << " passband samples but out capacity is only " << bigblock_emit_out_capacity
		          << " — NOT writing (would overrun). cfg=" << current_configuration << std::endl;
		// assert in debug builds so a CI/reproducer catches the mis-sizing at the source.
		assert(required_samples <= bigblock_emit_out_capacity
		       && "transmit_bigblock: block waveform exceeds out buffer capacity");
		bigblock_last_tx_K = 0;
		bigblock_last_tx_samples = 0;
		bigblock_restore_stock_config();
		return;
	}

	if(data != NULL && nBytes > 0)
	{
		int Ngrid = 0, log2M = 0, nBits = 0;
		bigblock_rebuild_thin_grid(Ngrid, log2M, nBits);
		bigblock_restore_stock_config();
		int kcap = 0; { const char* e = std::getenv("MERCURY_BIGBLOCK_K"); if(e && *e) kcap = atoi(e); }
		int Kpack = nBits / ldpc.N;
		if(kcap > 0 && kcap < Kpack) Kpack = kcap;
		int payload_bits_len = Kpack * ldpc.K;       // systematic info bits the block carries
		int payload_bytes_cap = payload_bits_len / 8; // byte capacity of the block

		std::vector<int> payload(payload_bits_len, 0);
		int use_bytes = (nBytes < payload_bytes_cap) ? nBytes : payload_bytes_cap;
		// Unpack each ARQ byte (data[i] in 0..255) LSB-first into the bit buffer.
		byte_to_bit(data, payload.data(), use_bytes);
		// (remaining bits already 0 from the vector init = zero pad)

		// PHASE 1 (fact-doc §11.6): energy-disperse the WHOLE payload (incl. the zero pad)
		// so a short/zero-heavy compressed frame still modulates to a well-conditioned
		// signal. Self-inverse; the RX de-whitens after decode. Without this a zero-padded
		// payload decoded to garbage (788/1400 errors). MUST cover all payload_bits_len.
		bigblock_whiten_payload_bits(payload.data(), payload_bits_len, BIGBLOCK_WHITEN_SEED);

		Kcw = bigblock_tx_passband(out, nSamples, cw_info, payload.data());
	}
	else
	{
		// P1 loopback / no external payload: seeded-PRBS known payload (validated 8/8).
		Kcw = bigblock_tx_passband(out, nSamples, cw_info, nullptr);
	}

	// stash the payload codeword-info for the loopback RX byte-correct gate. The
	// production/ARQ path compares against ARQ truth instead, but this stash also
	// carries the per-codeword info the RX carve maps back to sub-units.
	bigblock_last_tx_cw_info = cw_info;
	bigblock_last_tx_K = Kcw;
	bigblock_last_tx_samples = nSamples;

	// Defense-in-depth (fact-doc §13): make the per-frame bookkeeping reflect the ACTUAL
	// emitted length. Stock transmit_bit sets this; transmit_bigblock previously left it
	// stale, so a caller's frame_len clamp (arq_common.cc:4321) under-advanced and MASKED
	// the raw overrun in software. With the producer constraint a block can only be emitted
	// into a block-sized buffer, but stamp the true length so the bookkeeping never lies.
	tx_last_emitted_frame_samples = nSamples;

	bigblock_restore_stock_config();
}

st_receive_stats cl_telecom_system::receive_bigblock(double* data, int* out)
{
	// initialize stats (mirror the receive_byte defaults that downstream reads)
	receive_stats.message_decoded = NO;
	receive_stats.iterations_done = -1;
	receive_stats.crc = 0;
	receive_stats.SNR = -99.9;
	receive_stats.coarse_metric = 0.0;
	receive_stats.mean_H = -1.0;
	receive_stats.frame_overflow_symbols = 0;

	// the captured passband buffer spans the same window the live capture loop hands
	// receive_byte: Nofdm*buffer_Nsymb*interp samples.
	int interp = frequency_interpolation_rate;
	int nSamples = data_container.Nofdm * data_container.buffer_Nsymb * interp;
	if(nSamples <= 0) nSamples = (bigblock_last_tx_samples > 0) ? bigblock_last_tx_samples : 0;
	// §19: stash the captured-window length (samples) the decode runs over, so the ARQ
	// window-position guard can compare the located block extent against it WITHOUT
	// re-deriving buffer_Nsymb at carve time (bigblock_restore_stock_config may have
	// changed buffer_Nsymb by then). Captured here, BEFORE any rebuild/restore.
	bigblock_last_rx_capture_nsamples = nSamples;

	// USE-AFTER-FREE ROOT-CAUSE FIX (bigblock-whiten-align): `data` is the caller's
	// data_container.ready_to_process_passband_delayed_data (live ARQ path,
	// arq_common.cc:6864). Below we derive K via bigblock_rebuild_thin_grid +
	// bigblock_restore_stock_config — and restore_stock_config calls load_configuration,
	// which DEINITS+REINITS the data_container, FREEING and REALLOCATING
	// ready_to_process_passband_delayed_data. That left `data` DANGLING (points at freed
	// memory), so the subsequent rx_passband_normalize_and_blank(data,...) read freed/
	// re-used heap → SIM_INPROC SIGSEGV / HW silent garbage decode (wire_bsi=159, every
	// per-codeword CRC fails → PARTIAL clean=0 → 0 app bytes delivered). The standalone
	// BIGBLOCK_LIVE validator + the CASE A-D unit tests never hit this: they hand a
	// caller-owned std::vector as `data` (NOT the data_container buffer), so the realloc
	// could not dangle it. Snapshot the captured passband into a stable LOCAL buffer NOW,
	// before any rebuild/restore can move the data_container allocation, and run the whole
	// decode (normalize + bigblock_rx_passband) against the snapshot. nSamples doubles are
	// in-bounds here (data is sized Nofdm*buffer_Nsymb*interp at the live config, == nSamples).
	// REPRODUCER HOOK (bigblock-whiten-align): MERCURY_BIGBLOCK_DEFEAT_FIX_UAF=1 SKIPS this
	// snapshot (restores the pre-fix dangling-`data` use-after-free) for the standalone UAF
	// fail-before demo. The full-path regression (test_sim_inproc_bigblock_fullpath) does
	// NOT set this — a UAF SEGV would abort the whole test process before the pass-after arm
	// — it sets MERCURY_BIGBLOCK_DEFEAT_FIX (the partial-block bug, a clean non-crashing
	// 0-delivery) instead. The UAF fail-before is independently evidenced by the crash this
	// hook reproduces. Production never sets either.
	bool defeat_uaf = false;
	{ const char* e = std::getenv("MERCURY_BIGBLOCK_DEFEAT_FIX_UAF"); if(e && *e && atoi(e)!=0) defeat_uaf = true; }
	std::vector<double> data_snapshot;
	if(!defeat_uaf && data != NULL && nSamples > 0)
	{
		data_snapshot.assign(data, data + nSamples);
		data = data_snapshot.data();   // all readers below use the stable copy
	}

	// decoded info bits land here (carved into K sub-units by the caller in P2). Size
	// for the BIG-BLOCK K (= thin-grid nBits/ldpc.N), NOT the stock config's nBits —
	// the thin grid packs many more codewords than one stock frame. Use the TX stash
	// when present (loopback), else a generous bound covering the frozen layout's K.
	// (Bug: sizing from data_container.nBits/ldpc.N gave Kcap=1 → the worker, which
	// writes K*ldpc.K bits, overran the buffer → heap corruption. Surfaced by the
	// live-path refactor; the WAV harness never wrote bits back so never hit it.)
	int K_expected = (bigblock_last_tx_K > 0) ? bigblock_last_tx_K : 0;
	if(K_expected <= 0)
	{
		// derive K from the thin-grid layout without a full TX: rebuild + restore.
		int Ng=0,l2=0,nb=0; bigblock_rebuild_thin_grid(Ng,l2,nb);
		K_expected = nb / ldpc.N;
		bigblock_restore_stock_config();
	}
	if(K_expected <= 0) K_expected = 1;
	// HEAP-OVERRUN ROOT-CAUSE FIX (fact-doc §13): decode the K*ldpc.K (= 8*1400 = 11200)
	// info bits into the DEDICATED member buffer, NOT the caller's `out` (which on the live
	// ARQ path is data_container.data_byte[N_MAX=1600]). bigblock_rx_passband writes
	// Kout*ldpc.K ints; sizing for (K_expected+1)*ldpc.K + ldpc.K gives slack for a Kout
	// that exceeds the expected K (the worker caps Kout to the grid's K, but be defensive).
	bigblock_rx_infobits.assign((size_t)(K_expected + 1) * ldpc.K + ldpc.K, 0);
	int* info_bits = bigblock_rx_infobits.data();
	int  info_bits_cap = (int)bigblock_rx_infobits.size();

	std::vector<int> cw_ok;
	int Kout = 0;
	double acq_metric = 0.0;
	// P2.2 (normalization-bypass fix): run the SAME stock RX passband normalization +
	// impulse-noise blanking the per-frame receive_byte applies, BEFORE the big-block
	// estimator. Without this the captured passband sits at the wrong level for the
	// OFDM estimator/LLR and the live AWGN validator decoded garbage at 30 dB Es/N0
	// (0/8 codewords, BER 0.43). data IS the captured passband (mutable); normalize it
	// in place, then bigblock_rx_passband reads the normalized buffer.
	rx_passband_normalize_and_blank(data, nSamples);
	// P1 loopback: pass the known TX info bits so the per-codeword gate is byte-exact.
	const std::vector<std::vector<int>>* ref =
		(bigblock_last_tx_K > 0) ? &bigblock_last_tx_cw_info : nullptr;
	int cw_ok_count = bigblock_rx_passband(data, nSamples, info_bits, Kout,
	                                       cw_ok, &acq_metric, ref);

	receive_stats.coarse_metric = acq_metric;
	receive_stats.delay = 0;
	// HEAP-OVERRUN ROOT-CAUSE FIX (fact-doc §13): the decode already landed in the
	// dedicated bigblock_rx_infobits member (the ARQ carve reads from THERE, not `out`).
	// Copy into the caller's `out` ONLY a SAFE prefix bounded by N_MAX so the stock
	// data_byte[N_MAX] is NEVER overrun (pre-fix this copied Kout*ldpc.K=11200 ints into the
	// 1600-int data_byte -> 9600-int forward smash -> abort at the next config-switch free).
	// `out` is only a legacy convenience copy now; bigblock_receive_carve uses the member.
	int copy_bits = Kout * ldpc.K;
	if(copy_bits > info_bits_cap) copy_bits = info_bits_cap;   // never read past the decode buffer
	assert(copy_bits <= info_bits_cap && "receive_bigblock: decode exceeds info-bit buffer");
	int out_copy = copy_bits;
	if(out_copy > N_MAX) out_copy = N_MAX;                     // never overrun a stock data_byte[N_MAX]
	// REPRODUCER HOOK (fact-doc §13.R): MERCURY_BIGBLOCK_OLDGATE=1 restores the PRE-FIX
	// UNBOUNDED copy-out so CASE C can show the data_byte[N_MAX] forward overrun fail-before.
	{ const char* e = std::getenv("MERCURY_BIGBLOCK_OLDGATE");
	  if(e && *e && atoi(e)!=0) out_copy = copy_bits; }
	for(int i=0;i<out_copy;i++) out[i] = info_bits[i];

	// stash per-codeword result for the loopback validator / P2 SACK bitmap.
	bigblock_last_rx_cw_ok = cw_ok;
	bigblock_last_rx_K = Kout;
	bigblock_last_rx_cw_ok_count = cw_ok_count;

	// Restore stock config FIRST (it does a full reload; keep the result fields we set
	// AFTER it so nothing it touches clobbers them), then stamp the decode result.
	bigblock_restore_stock_config();

	// message_decoded = whole block clean (P1 gate). P2 redefines this as a per-block
	// ACK whose bitmap = cw_ok (one bad codeword does NOT fail the block).
	receive_stats.message_decoded = (Kout>0 && cw_ok_count==Kout) ? YES : NO;
	receive_stats.coarse_metric = acq_metric;
	receive_stats.iterations_done = -1;
	return receive_stats;
}

void cl_telecom_system::bigblock_livepath_loopback()
{
	auto env_i = [](const char* k, int def){ const char* e=std::getenv(k); return (e&&*e)?atoi(e):def; };
	auto env_d = [](const char* k, double def){ const char* e=std::getenv(k); return (e&&*e)?atof(e):def; };
	if(M == MOD_MFSK){ std::cout << "[BIGBLOCK-LIVE] MFSK unsupported; use -s 15/16." << std::endl; return; }

	std::cout << "[BIGBLOCK-LIVE] ===== LIVE-PATH big-block loopback (production transmit_byte/receive_byte) =====" << std::endl;

	// 1) GATE the production path on. Stock config stays CFG16 (32-QAM/0.875).
	bigblock_framing_enabled = true;

	// 2) size the TX output buffer = preamble + K*frame passband samples, plus lead so
	//    Schmidl-Cox has acquisition room (mirrors the WAV lead silence).
	int interp = frequency_interpolation_rate;
	int lead_n  = (int)(env_d("MERCURY_BIGBLOCK_LIVE_LEAD_MS", 100.0) * sampling_frequency / 1000.0);
	int block_n = bigblock_tx_total_samples();
	if(block_n <= 0){ std::cout << "[BIGBLOCK-LIVE] tx_total_samples=0; abort." << std::endl; bigblock_framing_enabled=false; return; }
	int trail_n = (int)(env_d("MERCURY_BIGBLOCK_LIVE_TRAIL_MS", 50.0) * sampling_frequency / 1000.0);

	std::vector<double> tx_pb(block_n, 0.0);
	// 3) PRODUCTION TX through transmit_byte (which branches to transmit_bigblock).
	//    NO_FILTER_MESSAGE writes raw passband. data/nBytes unused in P1 (known payload).
	//    HEAP-OVERRUN FIX (fact-doc §13): arm the per-call block-emit intent so the CFG16
	//    transmit_byte branch fires; tx_pb is block-sized (block_n).
	int dummy[1] = {0};
	{
		bigblock_emit_scope emit_guard(this, block_n);
		transmit_byte(dummy, 0, tx_pb.data(), NO_FILTER_MESSAGE);
	}
	int K_tx = bigblock_last_tx_K;
	int n_tx = bigblock_last_tx_samples;
	std::cout << "[BIGBLOCK-LIVE] TX via transmit_byte: K=" << K_tx
	          << " block_samples=" << n_tx << " (buf=" << block_n << ")" << std::endl;
	if(K_tx <= 0 || n_tx <= 0){ std::cout << "[BIGBLOCK-LIVE] TX produced no block; abort." << std::endl; bigblock_framing_enabled=false; return; }

	// 4) build the captured RX passband buffer the way the live capture loop would hand
	//    it to receive_byte: [lead silence | block | trail silence], sized to
	//    Nofdm*buffer_Nsymb*interp so receive_bigblock reads the whole window.
	//    Make the capture window at least lead+block+trail.
	int rx_window = lead_n + n_tx + trail_n;
	std::vector<double> rx_pb(rx_window, 0.0);
	for(int i=0;i<n_tx;i++) rx_pb[lead_n + i] = tx_pb[i];

	// optional clean->AWGN to confirm the estimator/LLR path runs (default clean).
	double live_esn0 = env_d("MERCURY_BIGBLOCK_LIVE_ESN0", -999.0);
	if(live_esn0 > -900.0)
	{
		// passband AWGN at the requested Es/N0, using the SAME convention as the
		// production BER path (passband_test_EsN0, telecom_system.cc:480):
		//   sigma = sqrt( 2 * P_sig * f_nyquist / (10^(EsN0/10) * bandwidth) )
		// where P_sig is the mean passband power of the BLOCK (signal region only).
		double sumsq=0.0; for(int i=0;i<n_tx;i++) sumsq += tx_pb[i]*tx_pb[i];
		double P_sig = sumsq / (n_tx>0?n_tx:1);
		double f_nyquist = sampling_frequency / 2.0;
		double sigma = sqrt(2.0 * P_sig * f_nyquist / (pow(10.0, live_esn0/10.0) * bandwidth));
		// AWGN-scale calibration (this-session diag): the shipped validator added
		// `sigma * awgn_value_generator()` per REAL passband sample, but the canonical
		// passband AWGN (apply_with_delay, awgn.cc:68/75; passband_test_EsN0,
		// telecom_system.cc:480/498; sfo_block_test :5618; the -10 path :4484) all add
		// `(sigma/sqrt(2)) * awgn_value_generator()` because awgn_value_generator() is
		// UNIT-variance and the convention is per-real-sample noise variance = sigma^2/2.
		// Bare sigma => 2x noise power => the labeled Es/N0 was 3.01 dB OPTIMISTIC (a TRUE
		// label-3 dB channel). Default is now the canonical convention so the dB label is
		// EXACT; MERCURY_BIGBLOCK_LIVE_AWGN_CONV=1 restores the old bare-sigma for A/B.
		int conv = env_d("MERCURY_BIGBLOCK_LIVE_AWGN_CONV", 0.0) != 0.0 ? 1 : 0;
		double per_sample = (conv==1) ? sigma : (sigma/sqrt(2.0));
		awgn_channel.set_seed(rand());
		double noise_sumsq=0.0;
		for(int i=0;i<rx_window;i++){
			double nz = per_sample * awgn_channel.awgn_value_generator();
			rx_pb[i] += nz; noise_sumsq += nz*nz;
		}
		double P_noise = noise_sumsq/(rx_window>0?rx_window:1);
		// TRUE per-sample Es/N0: passband Es/N0 = P_sig*f_nyquist/(P_noise*bw) under the
		// production convention (P_noise=sigma^2/2). Report what was ACTUALLY injected.
		double true_esn0 = 10.0*log10( P_sig*f_nyquist / (P_noise*bandwidth) );
		std::cout << "[BIGBLOCK-LIVE] AWGN Es/N0(label)=" << live_esn0 << " dB conv=" << conv
		          << " sigma=" << sigma << " per_sample_std=" << per_sample
		          << " P_sig=" << P_sig << " P_noise=" << P_noise
		          << " TRUE_injected_EsN0=" << true_esn0 << " dB bw=" << bandwidth << std::endl;
	}

	// 5) hand the capture buffer to receive_byte through a data_container whose window
	//    matches rx_window. receive_bigblock reads nSamples = Nofdm*buffer_Nsymb*interp;
	//    override buffer_Nsymb so that == rx_window. Save+restore so stock is untouched.
	int Nofdm = data_container.Nofdm;
	int saved_buffer_Nsymb = data_container.buffer_Nsymb;
	if(Nofdm > 0)
	{
		int need_syms = (rx_window + Nofdm*interp - 1) / (Nofdm*interp);
		data_container.buffer_Nsymb = need_syms;
		// re-pad rx_pb to exactly need_syms*Nofdm*interp so the read is in-bounds.
		int exact = need_syms * Nofdm * interp;
		if((int)rx_pb.size() < exact) rx_pb.resize(exact, 0.0);
	}

	// 6) PRODUCTION RX through receive_byte (branches to receive_bigblock).
	std::vector<int> out_bits((size_t)(K_tx+1) * ldpc.K + ldpc.K, 0);
	st_receive_stats rs = receive_byte(rx_pb.data(), out_bits.data());
	data_container.buffer_Nsymb = saved_buffer_Nsymb;

	int K_rx = bigblock_last_rx_K;
	int cw_ok = bigblock_last_rx_cw_ok_count;
	std::cout << "[BIGBLOCK-LIVE] RX via receive_byte: acq_metric=" << rs.coarse_metric
	          << " K=" << K_rx << " codewords_decoded=" << cw_ok << "/" << K_rx
	          << " message_decoded=" << (rs.message_decoded==YES?1:0) << std::endl;

	// 7) explicit byte-correct re-check: every decoded info bit == the known TX info bit.
	// HEAP-OVERRUN FIX (fact-doc §13): receive_bigblock now lands the full K*ldpc.K decode
	// in the dedicated member (out_bits/`out` is only an N_MAX-bounded legacy copy). Compare
	// against the member so the full-block byte-correct gate is unaffected by the bound.
	const std::vector<int>& rx_bits = bigblock_rx_infobits;
	long bit_err=0, bit_tot=0;
	int Kcmp = (K_rx<K_tx?K_rx:K_tx);
	for(int c=0;c<Kcmp && c<(int)bigblock_last_tx_cw_info.size();c++)
		for(int i=0;i<ldpc.K;i++){ bit_tot++;
			size_t bi=(size_t)c*ldpc.K+i;
			int got = (bi < rx_bits.size()) ? rx_bits[bi] : -1;
			if(got != bigblock_last_tx_cw_info[c][i]) bit_err++; }
	(void)out_bits;
	double ber = bit_tot? (double)bit_err/(double)bit_tot : 1.0;

	bool pass = (K_rx==K_tx && K_tx>0 && cw_ok==K_tx && bit_err==0);
	std::cout << "[BIGBLOCK-LIVE] ===== RESULT =====" << std::endl;
	std::cout << "[BIGBLOCK-LIVE]   codewords_decoded=" << cw_ok << "/" << K_tx
	          << "  post_FEC_BER=" << ber << "  infoerr=" << bit_err << "/" << bit_tot << std::endl;
	std::cout << "[BIGBLOCK-LIVE]   VERDICT=" << (pass ? "PASS(live-path 8/8 byte-correct)" : "FAIL") << std::endl;

	bigblock_framing_enabled = false;
}

void cl_telecom_system::load_configuration()
{
	this->load_configuration(default_configurations_telecom_system.init_configuration);
}

void cl_telecom_system::load_configuration(int configuration)
{
	if(configuration==current_configuration)
	{
		return;
	}

	if(configuration<0 || (configuration>=NUMBER_OF_CONFIGS && !is_robust_config(configuration)))
	{
		return;
	}

	// NB mode: clamp OFDM configs to CONFIG_6 max (QPSK/QAM need more pilots than Nc=10 provides)
	if(narrowband_enabled == YES && is_ofdm_config(configuration) && configuration > NB_CONFIG_MAX)
	{
		printf("[PHY] NB mode: clamping config %d to NB max %d\n", configuration, NB_CONFIG_MAX);
		configuration = NB_CONFIG_MAX;
	}

	printf("[PHY] Loading configuration %d (was %d)\n", configuration, current_configuration);
	fflush(stdout);

	int _modulation = MOD_BPSK;
	float _ldpc_rate = 1/16.0f;
	int ofdm_preamble_configurator_Nsymb = 4;
	int ofdm_channel_estimator = LEAST_SQUARE;

	if(configuration==CONFIG_0)
	{
		_modulation=MOD_BPSK;
		_ldpc_rate=1/16.0;
		ofdm_preamble_configurator_Nsymb=4;
		ofdm_channel_estimator=LEAST_SQUARE;
	}
	else if(configuration==CONFIG_1)
	{
		_modulation=MOD_BPSK;
		_ldpc_rate=2/16.0;
		ofdm_preamble_configurator_Nsymb=4;
		ofdm_channel_estimator=LEAST_SQUARE;
	}
	else if(configuration==CONFIG_2)
	{
		_modulation=MOD_BPSK;
		_ldpc_rate=3/16.0;
		ofdm_preamble_configurator_Nsymb=4;
		ofdm_channel_estimator=LEAST_SQUARE;
	}
	else if(configuration==CONFIG_3)
	{
		_modulation=MOD_BPSK;
		_ldpc_rate=4/16.0;
		ofdm_preamble_configurator_Nsymb=4;
		ofdm_channel_estimator=LEAST_SQUARE;
	}
	else if(configuration==CONFIG_4)
	{
		_modulation=MOD_BPSK;
		_ldpc_rate=5/16.0;
		ofdm_preamble_configurator_Nsymb=4;
		ofdm_channel_estimator=LEAST_SQUARE;
	}
	else if(configuration==CONFIG_5)
	{
		_modulation=MOD_BPSK;
		_ldpc_rate=6/16.0;
		ofdm_preamble_configurator_Nsymb=4;
		ofdm_channel_estimator=LEAST_SQUARE;
	}
	else if(configuration==CONFIG_6)
	{
		_modulation=MOD_BPSK;
		_ldpc_rate=8/16.0;
		ofdm_preamble_configurator_Nsymb=4;
		ofdm_channel_estimator=LEAST_SQUARE;
	}
	else if(configuration==CONFIG_7)
	{
		_modulation=MOD_QPSK;
		_ldpc_rate=5/16.0;
		ofdm_preamble_configurator_Nsymb=4;
		ofdm_channel_estimator=LEAST_SQUARE;
	}
	else if(configuration==CONFIG_8)
	{
		_modulation=MOD_QPSK;
		_ldpc_rate=6/16.0;
		ofdm_preamble_configurator_Nsymb=4;
		ofdm_channel_estimator=LEAST_SQUARE;
	}
	else if(configuration==CONFIG_9)
	{
		_modulation=MOD_QPSK;
		_ldpc_rate=8/16.0;
		ofdm_preamble_configurator_Nsymb=4;
		ofdm_channel_estimator=LEAST_SQUARE;
	}
	else if(configuration==CONFIG_10)
	{
		_modulation=MOD_8PSK;
		_ldpc_rate=6/16.0;
		ofdm_preamble_configurator_Nsymb=4;
		ofdm_channel_estimator=LEAST_SQUARE;
	}
	else if(configuration==CONFIG_11)
	{
		_modulation=MOD_8PSK;
		_ldpc_rate=8/16.0;
		ofdm_preamble_configurator_Nsymb=4;
		ofdm_channel_estimator=LEAST_SQUARE;
	}
	else if(configuration==CONFIG_12)
	{
		_modulation=MOD_QPSK;
		_ldpc_rate=14/16.0;
		ofdm_preamble_configurator_Nsymb=4;
		ofdm_channel_estimator=LEAST_SQUARE;
	}
	else if(configuration==CONFIG_13)
	{
		_modulation=MOD_8PSK;
		_ldpc_rate=12/16.0;
		ofdm_preamble_configurator_Nsymb=4;
		ofdm_channel_estimator=LEAST_SQUARE;
	}
	else if(configuration==CONFIG_14)
	{
		_modulation=MOD_8PSK;
		_ldpc_rate=14/16.0;
		ofdm_preamble_configurator_Nsymb=4;
		ofdm_channel_estimator=LEAST_SQUARE;
	}
	else if(configuration==CONFIG_15)
	{
		_modulation=MOD_16QAM;
		_ldpc_rate=14/16.0;
		ofdm_preamble_configurator_Nsymb=4;
		ofdm_channel_estimator=LEAST_SQUARE;
	}
	else if(configuration==CONFIG_16)
	{
		_modulation=MOD_32QAM;
		_ldpc_rate=14/16.0;
		ofdm_preamble_configurator_Nsymb=4;
		ofdm_channel_estimator=LEAST_SQUARE;
	}
	else if(configuration==ROBUST_0)
	{
		_modulation=MOD_MFSK;
		_ldpc_rate=1/16.0;
		ofdm_preamble_configurator_Nsymb=4;
		ofdm_channel_estimator=LEAST_SQUARE;
	}
	else if(configuration==ROBUST_1)
	{
		_modulation=MOD_MFSK;
		_ldpc_rate=1/16.0;  // Same FEC as ROBUST_0; speed comes from 2x parallel streams
		ofdm_preamble_configurator_Nsymb=4;
		ofdm_channel_estimator=LEAST_SQUARE;
	}
	else if(configuration==ROBUST_2)
	{
		_modulation=MOD_MFSK;
		_ldpc_rate=4/16.0;  // Rate 1/4: 4x throughput vs ROBUST_1, waterfall at -8 dB
		ofdm_preamble_configurator_Nsymb=4;
		ofdm_channel_estimator=LEAST_SQUARE;
	}

	// Amplitude restoration disabled for all modes: full ZF equalization
	// preserves |H| for MMSE erasure and CSI weighting on frequency-selective channels.
	// Previously PSK modes forced |H|=1, losing 35 dB SNR on analog channels with ~8 dB variation.
	ofdm.channel_estimator_amplitude_restoration=NO;

	if(current_configuration!=CONFIG_NONE)
	{
		reinit_subsystems.microphone=NO;
		reinit_subsystems.speaker=NO;
		reinit_subsystems.telecom_system=NO;
		reinit_subsystems.data_container=NO;
		reinit_subsystems.ofdm_FIR_rx_data=NO;
		reinit_subsystems.ofdm_FIR_rx_time_sync=NO;
		reinit_subsystems.ofdm_FIR_tx1=NO;
		reinit_subsystems.ofdm_FIR_tx2=NO;
		reinit_subsystems.ofdm=NO;
		reinit_subsystems.ldpc=NO;
		reinit_subsystems.psk=NO;
		reinit_subsystems.pre_equalization_channel=NO;
	}

	if(current_configuration==CONFIG_NONE)
	{
		last_configuration=configuration;
		current_configuration=configuration;
		// Force full reinit when coming from CONFIG_NONE.
		// This covers first-time init (flags already YES from struct defaults)
		// and NB/WB switches (switch_narrowband_mode sets CONFIG_NONE, but
		// previous init left all flags NO — same config number + same modulation
		// wouldn't trigger any of the checks below, leaving Nc/bandwidth stale).
		reinit_subsystems = st_reinit_subsystems();
	}
	else
	{
		last_configuration=current_configuration;
		current_configuration=configuration;
	}
	if(_modulation!=M || ofdm_preamble_configurator_Nsymb!=ofdm.preamble_configurator.Nsymb)
	{
		reinit_subsystems.microphone=YES;
		reinit_subsystems.speaker=YES;
		reinit_subsystems.telecom_system=YES;
		reinit_subsystems.data_container=YES;
		reinit_subsystems.ofdm=YES;
		reinit_subsystems.psk=YES;
		reinit_subsystems.pre_equalization_channel=YES;
	}

	// Bug #61: LDPC parity matrix must be regenerated when rate changes.
	// Without this, turboshift config changes leave the decoder using the
	// old config's parity matrix, causing LDPC failure at every rate change.
	// Previous tests passed because -s <config> starts from CONFIG_NONE
	// (full init), but turboshift's incremental path never reinitialized LDPC.
	if(_ldpc_rate != ldpc.rate)
	{
		reinit_subsystems.ldpc=YES;
	}

	// MFSK: different ROBUST configs may change M or nStreams, need full reinit
	if(_modulation==MOD_MFSK && M==MOD_MFSK)
	{
		int new_mfsk_M, new_nStreams;
		if(configuration == ROBUST_0) { new_mfsk_M = narrowband_enabled ? 8 : 32; new_nStreams = 1; }
		else { new_mfsk_M = narrowband_enabled ? 4 : 16; new_nStreams = 2; } // ROBUST_1, ROBUST_2
		if(new_mfsk_M != mfsk.M || new_nStreams != mfsk.nStreams)
		{
			reinit_subsystems.telecom_system=YES;
			reinit_subsystems.data_container=YES;
			reinit_subsystems.ofdm=YES;
			reinit_subsystems.psk=YES;
		}
	}

	if(_ldpc_rate!=ldpc.rate)
	{
		reinit_subsystems.telecom_system=YES;
		reinit_subsystems.ldpc=YES;
	}

	if(reinit_subsystems.microphone==YES)
	{
        // why do we need this?
		// microphone.deinit();
	}
	if(reinit_subsystems.speaker==YES)
	{
        // why do we need this?
		// speaker.deinit();
	}

	if(reinit_subsystems.psk==YES)
	{
		psk.deinit();
	}
	bool capture_mutex_held = false;  // Bug #42: track if we're holding the mutex

	if(reinit_subsystems.telecom_system==YES)
	{
		// Bug #42: Hold capture_prep_mutex across the entire deinit→init cycle.
		// The audio capture_prep thread reads Nofdm/buffer_Nsymb outside the
		// mutex, then re-checks inside. Without holding the mutex for the full
		// cycle, CPU store reordering or compiler optimizations can let the
		// capture thread see partially-initialized state (new Nofdm set by
		// set_size() but buffers not yet allocated), leading to heap corruption.
		// The mutex is held through deinit, parameter updates, and init, so the
		// capture thread always sees either the old consistent state or the new
		// consistent state — never an intermediate mix.
		if(reinit_subsystems.data_container==YES)
		{
			printf("[PHY-SWITCH] Taking capture_prep_mutex, zeroing Nofdm/buffer_Nsymb\n");
			fflush(stdout);
			MUTEX_LOCK(&capture_prep_mutex);
			capture_mutex_held = true;
			data_container.Nofdm = 0;
			data_container.buffer_Nsymb = 0;
			data_container.data_ready = 0;
		}

		printf("[PHY-SWITCH] deinit() start\n");
		fflush(stdout);
		this->deinit();
		printf("[PHY-SWITCH] deinit() done\n");
		fflush(stdout);
	}

	M=_modulation;
	ldpc.rate=_ldpc_rate;
	ofdm.preamble_configurator.Nsymb=ofdm_preamble_configurator_Nsymb;
	// NB MFSK: 8-symbol preamble for cross-correlation detection.
	// WB MFSK: 16-symbol preamble (raised from 4 on 2026-05-27 per
	// data-frame-cliff-audit-2026-05-27.md §H1) for +6 dB matched-filter
	// integration gain at the WGN:-8 cliff. The cl_mfsk::init() path
	// (mfsk.cc:122-137) sets mfsk.preamble_nSymb=16 for M=32 and M=16
	// alphabets; this override mirrors that into the data_container size
	// authority (cl_telecom_system.cc:3872 passes
	// ofdm.preamble_configurator.Nsymb into data_container::set_size).
	// All four authorities (mfsk.preamble_nSymb, data_container.
	// preamble_nSymb, ofdm.preamble_configurator.Nsymb,
	// mfsk_corr_template_nsymb) end up == 16 — see data-flow-preamble_nSymb.md
	// §4 INV-PROD-1.
	if(M == MOD_MFSK)
		ofdm.preamble_configurator.Nsymb = narrowband_enabled ? 8 : 16;
	// NB estimator: blanket ZF for all NB configs.
	// ZF (per-pilot H=Y/P) is immune to inter-symbol phase jitter that makes
	// LS cross-pilot averaging destructive on VB-Cable/HF. With Nc=10 and only
	// 3-4 pilots per row, LS averaging can't reduce noise without destroying
	// phase coherence. Tested: LS gives mean_H=0.45 on VB-Cable CONFIG_15
	// while ZF gives mean_H=1.0. CPE_correction pre-estimator handles residual
	// freq offset. CONFIG_14+ NB is a design limitation of sparse pilot density.
	if(narrowband_enabled && ofdm_channel_estimator == LEAST_SQUARE)
	{
		ofdm_channel_estimator = ZERO_FORCE;
	}
	ofdm.channel_estimator=ofdm_channel_estimator;

	awgn_channel.set_seed(rand());

	ofdm.Nc=default_configurations_telecom_system.ofdm_Nc;
	ofdm.Nfft=default_configurations_telecom_system.ofdm_Nfft;
	ofdm.gi=default_configurations_telecom_system.ofdm_gi;
	ofdm.Nsymb=default_configurations_telecom_system.ofdm_Nsymb;

	ofdm.pilot_configurator.Dx=default_configurations_telecom_system.ofdm_pilot_configurator_Dx;
	ofdm.pilot_configurator.Dy=default_configurations_telecom_system.ofdm_pilot_configurator_Dy;
	ofdm.pilot_configurator.first_row=default_configurations_telecom_system.ofdm_pilot_configurator_first_row;
	ofdm.pilot_configurator.last_row=default_configurations_telecom_system.ofdm_pilot_configurator_last_row;
	ofdm.pilot_configurator.first_col=default_configurations_telecom_system.ofdm_pilot_configurator_first_col;
	ofdm.pilot_configurator.second_col=default_configurations_telecom_system.ofdm_pilot_configurator_second_col;
	ofdm.pilot_configurator.last_col=default_configurations_telecom_system.ofdm_pilot_configurator_last_col;
	ofdm.pilot_configurator.boost=default_configurations_telecom_system.ofdm_pilot_configurator_pilot_boost;
	ofdm.pilot_configurator.seed=default_configurations_telecom_system.ofdm_pilot_configurator_seed;
	ofdm.pilot_configurator.pilot_density=default_configurations_telecom_system.ofdm_pilot_density;

	// nIdentical_sections derives from subcarrier spacing in configure().
	// WB (Nc>=50): every-4th → 4 identical sections (period Nfft/4)
	// NB (Nc<=10): every-2nd → 2 identical sections (period Nfft/2)
	// Must compute dynamically here — the old default (physical_config.cc:51 = 2)
	// overwrote the value set by configure() in init(), breaking Stage 4a.
	ofdm.preamble_configurator.nIdentical_sections = (ofdm.Nc >= 50) ? 4 : 2;
	ofdm.preamble_configurator.modulation=default_configurations_telecom_system.ofdm_preamble_configurator_modulation;
	ofdm.preamble_configurator.boost=default_configurations_telecom_system.ofdm_preamble_configurator_boost;
	ofdm.preamble_configurator.seed=default_configurations_telecom_system.ofdm_preamble_configurator_seed;

	ofdm.freq_offset_ignore_limit=default_configurations_telecom_system.ofdm_freq_offset_ignore_limit;
	ofdm.start_shift=default_configurations_telecom_system.ofdm_start_shift;

	ofdm.preamble_papr_cut=default_configurations_telecom_system.ofdm_preamble_papr_cut;
	ofdm.data_papr_cut=default_configurations_telecom_system.ofdm_data_papr_cut;

	ofdm.LS_window_width=default_configurations_telecom_system.ofdm_LS_window_width;
	ofdm.LS_window_hight=default_configurations_telecom_system.ofdm_LS_window_hight;

	if(ofdm.LS_window_width%2==0)
	{
		ofdm.LS_window_width++;
	}
	if(ofdm.LS_window_hight%2==0)
	{
		ofdm.LS_window_hight++;
	}

	// NB LS window: with CPE correction (pre-LS residual freq offset removal),
	// the full default window height can be used for NB too — CPE removes the
	// phase rotation that previously caused H cancellation in the LS window.

	bit_energy_dispersal_seed=default_configurations_telecom_system.bit_energy_dispersal_seed;

	ldpc.standard=default_configurations_telecom_system.ldpc_standard;
	ldpc.framesize=default_configurations_telecom_system.ldpc_framesize;

	ldpc.decoding_algorithm=default_configurations_telecom_system.ldpc_decoding_algorithm;
	ldpc.GBF_eta=default_configurations_telecom_system.ldpc_GBF_eta;
	ldpc.nIteration_max=default_configurations_telecom_system.ldpc_nIteration_max;
	// Q3: ROBUST tier (rate-1/16 LDPC) needs more SPA iterations to converge at
	// the waterfall. OFDM configs are above the cliff and 100 iter is plenty.
	// See mfsk-vara-parity-plan.md §2.1 Q3.
	if(is_robust_config(configuration))
		ldpc.nIteration_max = 200;
	ldpc.print_nIteration=default_configurations_telecom_system.ldpc_print_nIteration;

	outer_code=default_configurations_telecom_system.outer_code;

	if(outer_code==CRC16_MODBUS_RTU)
	{
		outer_code_reserved_bits=16;
	}
	else
	{
		outer_code_reserved_bits=0;
	}

	bandwidth=default_configurations_telecom_system.bandwidth;
	time_sync_trials_max=default_configurations_telecom_system.time_sync_trials_max;
	// MFSK non-coherent detection has no channel estimation to compensate
	// for timing errors, so we need to try more preamble correlation peaks.
	if (_modulation == MOD_MFSK) {
		time_sync_trials_max = 5;
	}
	use_last_good_time_sync=default_configurations_telecom_system.use_last_good_time_sync;
	use_last_good_freq_offset=default_configurations_telecom_system.use_last_good_freq_offset;
	frequency_interpolation_rate=default_configurations_telecom_system.frequency_interpolation_rate;
	carrier_frequency=default_configurations_telecom_system.carrier_frequency;
	output_power_Watt=default_configurations_telecom_system.output_power_Watt;

	ofdm.FIR_rx_data.filter_window=default_configurations_telecom_system.ofdm_FIR_rx_data_filter_window;
	ofdm.FIR_rx_data.filter_transition_bandwidth=default_configurations_telecom_system.ofdm_FIR_rx_data_filter_transition_bandwidth;
	ofdm.FIR_rx_data.lpf_filter_cut_frequency=default_configurations_telecom_system.ofdm_FIR_rx_data_lpf_filter_cut_frequency;
	ofdm.FIR_rx_data.type=default_configurations_telecom_system.ofdm_FIR_rx_data_filter_type;

	ofdm.FIR_rx_time_sync.filter_window=default_configurations_telecom_system.ofdm_FIR_rx_time_sync_filter_window;
	ofdm.FIR_rx_time_sync.filter_transition_bandwidth=default_configurations_telecom_system.ofdm_FIR_rx_time_sync_filter_transition_bandwidth;
	ofdm.FIR_rx_time_sync.lpf_filter_cut_frequency=default_configurations_telecom_system.ofdm_FIR_rx_time_sync_lpf_filter_cut_frequency;
	ofdm.FIR_rx_time_sync.type=default_configurations_telecom_system.ofdm_FIR_rx_time_sync_filter_type;


	ofdm.FIR_tx1.filter_window=default_configurations_telecom_system.ofdm_FIR_tx1_filter_window;
	ofdm.FIR_tx1.filter_transition_bandwidth=default_configurations_telecom_system.ofdm_FIR_tx1_filter_transition_bandwidth;
	ofdm.FIR_tx1.lpf_filter_cut_frequency=default_configurations_telecom_system.ofdm_FIR_tx1_lpf_filter_cut_frequency;
	ofdm.FIR_tx1.hpf_filter_cut_frequency=default_configurations_telecom_system.ofdm_FIR_tx1_hpf_filter_cut_frequency;
	ofdm.FIR_tx1.type=default_configurations_telecom_system.ofdm_FIR_tx1_filter_type;

	ofdm.FIR_tx2.filter_window=default_configurations_telecom_system.ofdm_FIR_tx2_filter_window;
	ofdm.FIR_tx2.filter_transition_bandwidth=default_configurations_telecom_system.ofdm_FIR_tx2_filter_transition_bandwidth;
	ofdm.FIR_tx2.lpf_filter_cut_frequency=default_configurations_telecom_system.ofdm_FIR_tx2_lpf_filter_cut_frequency;
	ofdm.FIR_tx2.hpf_filter_cut_frequency=default_configurations_telecom_system.ofdm_FIR_tx2_hpf_filter_cut_frequency;
	ofdm.FIR_tx2.type=default_configurations_telecom_system.ofdm_FIR_tx2_filter_type;


	constellation_plot.folder=default_configurations_telecom_system.plot_folder;
	BER_plot.folder=default_configurations_telecom_system.plot_folder;
	constellation_plot.plot_active=default_configurations_telecom_system.plot_plot_active;
	BER_plot.plot_active=default_configurations_telecom_system.plot_plot_active;


	if(reinit_subsystems.psk==YES)
	{
		if(M == MOD_MFSK)
		{
			int mfsk_M, mfsk_nStreams;
			if(current_configuration == ROBUST_0) {
				mfsk_M = narrowband_enabled ? 8 : 32;
				mfsk_nStreams = 1;
			} else {
				mfsk_M = narrowband_enabled ? 4 : 16;
				mfsk_nStreams = 2;
			}
			mfsk.init(mfsk_M, ofdm.Nc, mfsk_nStreams);
		}
		else
		{
			psk.set_predefined_constellation(M);
		}
		reinit_subsystems.psk=NO;
	}
	if(reinit_subsystems.telecom_system==YES)
	{
		printf("[PHY-SWITCH] init() start (nb=%d M=%.0f config=%d)\n",
			narrowband_enabled, M, current_configuration);
		fflush(stdout);
		this->init();
		printf("[PHY-SWITCH] init() done (Nc=%d Nsymb=%d Nofdm=%d buffer_Nsymb=%d)\n",
			ofdm.Nc, ofdm.Nsymb, (int)data_container.Nofdm, (int)data_container.buffer_Nsymb);
		fflush(stdout);

		// Bug #42: Release the mutex after init() has fully set up the new
		// data_container buffers and parameters. The capture_prep thread will
		// now see a fully consistent new state.
		if(capture_mutex_held)
		{
			printf("[PHY-SWITCH] Releasing capture_prep_mutex\n");
			fflush(stdout);
			MUTEX_UNLOCK(&capture_prep_mutex);
			capture_mutex_held = false;
		}

		reinit_subsystems.telecom_system=NO;

		// Note: bandwidth-dependent parameters (bandwidth, carrier_frequency, FIR cutoffs)
		// are set directly by init() in the NB/WB recomputation block above.
		// No re-apply from default_configurations_telecom_system needed.
	}

	// Re-init MFSK now that ofdm.Nc is finalized (was AUTO_SELLECT during mfsk.init above)
	// This recalculates stream_offsets with the correct Nc value
	if(M == MOD_MFSK)
	{
		int mfsk_M, mfsk_nStreams;
		if(current_configuration == ROBUST_0) {
			mfsk_M = narrowband_enabled ? 8 : 32;
			mfsk_nStreams = 1;
		} else {
			mfsk_M = narrowband_enabled ? 4 : 16;
			mfsk_nStreams = 2;
		}
		mfsk.init(mfsk_M, ofdm.Nc, mfsk_nStreams);
	}

	// Generate MFSK cross-correlation template for preamble detection (NB+WB).
	// Round-trip through passband signal chain so template matches RX exactly:
	// symbol_mod → baseband_to_passband → passband_to_baseband(FIR) → decimate
	if(M == MOD_MFSK)
	{
		if(ofdm.mfsk_corr_template != NULL) { delete[] ofdm.mfsk_corr_template; ofdm.mfsk_corr_template = NULL; }

		// Generate preamble in frequency domain
		mfsk.generate_preamble(data_container.preamble_data, data_container.preamble_nSymb);

		// Modulate to baseband: zero_pad → IFFT → GI add (Nofdm samples per symbol)
		int template_nsymb = data_container.preamble_nSymb;
		int Nofdm = data_container.Nofdm;
		int bb_len = template_nsymb * Nofdm;
		std::complex<double>* bb_template = new std::complex<double>[bb_len];
		for(int i = 0; i < template_nsymb; i++)
		{
			ofdm.symbol_mod(&data_container.preamble_data[i * data_container.Nc],
			                &bb_template[i * Nofdm]);
		}

		// Round-trip: baseband → passband → FIR-filtered baseband
		// This ensures the template has the same spectral shaping as the RX signal
		int interp = frequency_interpolation_rate;
		int pb_len = bb_len * interp;
		double* pb_data = new double[pb_len];

		long unsigned saved_pss = ofdm.passband_start_sample;
		ofdm.passband_start_sample = 0;
		ofdm.baseband_to_passband(bb_template, bb_len, pb_data,
			sampling_frequency, carrier_frequency, carrier_amplitude, interp);
		ofdm.passband_start_sample = saved_pss;

		// Demodulate with FIR_rx_time_sync (same filter used in receive_byte)
		std::complex<double>* filtered = new std::complex<double>[pb_len];
		ofdm.passband_to_baseband(pb_data, pb_len, filtered,
			sampling_frequency, carrier_frequency, carrier_amplitude, 1, &ofdm.FIR_rx_time_sync);

		// Store decimated (baseband-rate) template — correlation steps by interp_rate
		ofdm.mfsk_corr_template_len = bb_len;
		ofdm.mfsk_corr_template_nsymb = template_nsymb;
		ofdm.mfsk_corr_template = CNEW(std::complex<double>, bb_len, "ofdm.mfsk_corr_template");
		for(int i = 0; i < bb_len; i++)
			ofdm.mfsk_corr_template[i] = filtered[i * interp];

		// Precompute total and per-symbol template energies for normalization.
		// Cap raised 8 -> 16 on 2026-05-27 (data-flow-preamble_nSymb.md §H1):
		// WB MFSK now uses a 16-symbol preamble. NB still uses 8 (loop body
		// executes 8 times); template_nsymb is the runtime authority.
		ofdm.mfsk_corr_template_energy = 0.0;
		for(int k = 0; k < template_nsymb && k < 16; k++)
		{
			double sym_energy = 0.0;
			for(int n = 0; n < Nofdm; n++)
			{
				int idx = k * Nofdm + n;
				sym_energy +=
					ofdm.mfsk_corr_template[idx].real() * ofdm.mfsk_corr_template[idx].real() +
					ofdm.mfsk_corr_template[idx].imag() * ofdm.mfsk_corr_template[idx].imag();
			}
			ofdm.mfsk_corr_template_sym_energy[k] = sym_energy;
			ofdm.mfsk_corr_template_energy += sym_energy;
		}

		delete[] pb_data;
		delete[] filtered;
		delete[] bb_template;

		printf("[PHY] MFSK corr template: %d symbols, %d samples, energy=%.3f (per-sym corr, FIR round-tripped)\n",
			template_nsymb, bb_len, ofdm.mfsk_corr_template_energy);
		fflush(stdout);

		// Populate MFSK preamble parameters consumed by the discrete-match
		// `time_sync_mfsk_corr` detector (post-2026-05-27 port per
		// data-preamble-port-research.md §14). Mirror of cl_mfsk fields.
		ofdm.mfsk_M = mfsk.M;
		ofdm.mfsk_nStreams = mfsk.nStreams;
		for(int st = 0; st < 4; st++)
			ofdm.mfsk_stream_offsets[st] = (st < cl_mfsk::MAX_STREAMS) ? mfsk.stream_offsets[st] : 0;
		ofdm.mfsk_preamble_nsymb = mfsk.preamble_nSymb;
		for(int s = 0; s < 16; s++)
			ofdm.mfsk_preamble_tones[s] = (s < cl_mfsk::MAX_PREAMBLE_SYMB) ? mfsk.preamble_tones[s] : 0;
		ofdm.mfsk_preamble_match_threshold = mfsk.preamble_match_threshold;
	}
	else
	{
		if(ofdm.mfsk_corr_template != NULL) { delete[] ofdm.mfsk_corr_template; ofdm.mfsk_corr_template = NULL; }
		ofdm.mfsk_corr_template_len = 0;
		ofdm.mfsk_corr_template_energy = 0.0;
		ofdm.mfsk_corr_template_nsymb = 0;

		// Reset MFSK preamble params on non-MFSK configs.
		ofdm.mfsk_M = 0;
		ofdm.mfsk_nStreams = 0;
		ofdm.mfsk_preamble_nsymb = 0;
		ofdm.mfsk_preamble_match_threshold = 0;
		for(int s = 0; s < 16; s++) ofdm.mfsk_preamble_tones[s] = 0;
		for(int st = 0; st < 4; st++) ofdm.mfsk_stream_offsets[st] = 0;

#if 0 // Template generation disabled: using Schmidl-Cox autocorrelation
		// Generate OFDM matched-filter template for preamble detection.
		// Must replicate the full TX→RX chain so the template matches what
		// receive_byte actually sees:
		//   preamble × pre_eq → symbol_mod → boost → b2p → FIR_tx1 → FIR_tx2 → p2b(FIR_rx_time_sync)
		// Pre-equalization applies per-subcarrier complex rotations that completely
		// reshape the time-domain waveform. Without it, the template has ~0.02
		// correlation with the received signal (essentially random).
		if(ofdm.ofdm_corr_template != NULL) { delete[] ofdm.ofdm_corr_template; ofdm.ofdm_corr_template = NULL; }

		int template_nsymb = data_container.preamble_nSymb;
		int Nofdm = data_container.Nofdm;
		int bb_len = template_nsymb * Nofdm;
		std::complex<double>* bb_template = new std::complex<double>[bb_len];

		// Extract preamble subcarrier values WITH pre-equalization (same as transmit_byte).
		// pre_equalization_channel is computed earlier in load_configuration (line ~2396).
		std::complex<double> preamble_sc[256];
		for(int i = 0; i < template_nsymb; i++)
		{
			for(int k = 0; k < ofdm.Nc; k++)
				preamble_sc[k] = ofdm.ofdm_preamble[i * ofdm.Nc + k].value
					* pre_equalization_channel[k].value;
			ofdm.symbol_mod(preamble_sc, &bb_template[i * Nofdm]);
		}

		// === DIAG: pre_eq at template generation (remove after debug) ===
		printf("[TMPL-PREEQ] CONFIG_%d preamble_nSymb=%d pre_eq[0..4]=(%.4f,%.4f)(%.4f,%.4f)(%.4f,%.4f)(%.4f,%.4f)(%.4f,%.4f)\n",
			current_configuration, template_nsymb,
			pre_equalization_channel[0].value.real(), pre_equalization_channel[0].value.imag(),
			pre_equalization_channel[1].value.real(), pre_equalization_channel[1].value.imag(),
			pre_equalization_channel[2].value.real(), pre_equalization_channel[2].value.imag(),
			pre_equalization_channel[3].value.real(), pre_equalization_channel[3].value.imag(),
			pre_equalization_channel[4].value.real(), pre_equalization_channel[4].value.imag());
		fflush(stdout);

		// Apply power normalization + output power + preamble boost (same as transmit_bit lines 601-602).
		// sqrt(output_power_Watt) MUST be included so peak_clip applies at the same
		// absolute threshold as the TX path. CS is amplitude-invariant, so the
		// extra sqrt(output_power_Watt) factor doesn't affect the final metric,
		// but peak_clip is a nonlinear operation that depends on absolute amplitude.
		// Without this scaling, the template is clipped at a different PAPR level
		// than the TX signal → waveform mismatch → CS metric ~0.15 instead of ~1.0.
		double power_normalization = sqrt((double)(ofdm.Nfft * frequency_interpolation_rate));
		double preamble_boost = ofdm.preamble_configurator.boost;
		double ofdm_tx_gain = get_tx_gain(TX_SIG_OFDM);
		for(int i = 0; i < bb_len; i++)
			bb_template[i] = bb_template[i] / power_normalization * sqrt(output_power_Watt) * preamble_boost * ofdm_tx_gain;

		// Round-trip through full TX→RX passband chain:
		// baseband_to_passband → peak_clip → FIR_tx1 → FIR_tx2 → passband_to_baseband(FIR_rx_time_sync)
		int interp = frequency_interpolation_rate;
		int pb_len = bb_len * interp;
		double* pb_data = new double[pb_len];
		double* pb_fir1 = new double[pb_len];
		double* pb_fir2 = new double[pb_len];

		long unsigned saved_pss = ofdm.passband_start_sample;
		ofdm.passband_start_sample = 0;
		ofdm.baseband_to_passband(bb_template, bb_len, pb_data,
			sampling_frequency, carrier_frequency, carrier_amplitude, interp);
		ofdm.passband_start_sample = saved_pss;

		// Apply PAPR clipping to match TX path (transmit_bit line 616).
		// pre_equalization_channel boosts high-frequency preamble subcarriers
		// (above FIR_rx_data cutoff) to very large amplitudes. Without matching
		// peak_clip in the template, the clipped TX waveform diverges from the
		// unclipped template → CS metric drops from ~1.0 to ~0.15.
		ofdm.peak_clip(pb_data, pb_len, ofdm.preamble_papr_cut);

		// TX shaping filters (same as transmit_byte SINGLE_MESSAGE path)
		ofdm.FIR_tx1.apply(pb_data, pb_fir1, pb_len);
		ofdm.FIR_tx2.apply(pb_fir1, pb_fir2, pb_len);

		// Demodulate with FIR_rx_time_sync (same filter used in receive_byte line 767)
		std::complex<double>* filtered = new std::complex<double>[pb_len];
		ofdm.passband_to_baseband(pb_fir2, pb_len, filtered,
			sampling_frequency, carrier_frequency, carrier_amplitude, 1, &ofdm.FIR_rx_time_sync);

		// Store decimated (baseband-rate) template
		ofdm.ofdm_corr_template_len = bb_len;
		ofdm.ofdm_corr_template_nsymb = template_nsymb;
		ofdm.ofdm_corr_template = CNEW(std::complex<double>, bb_len, "ofdm.ofdm_corr_template");
		for(int i = 0; i < bb_len; i++)
			ofdm.ofdm_corr_template[i] = filtered[i * interp];

		// Precompute per-symbol and total template energies
		ofdm.ofdm_corr_template_energy = 0.0;
		for(int k = 0; k < template_nsymb && k < 16; k++)
		{
			double sym_energy = 0.0;
			for(int n = 0; n < Nofdm; n++)
			{
				int idx = k * Nofdm + n;
				sym_energy +=
					ofdm.ofdm_corr_template[idx].real() * ofdm.ofdm_corr_template[idx].real() +
					ofdm.ofdm_corr_template[idx].imag() * ofdm.ofdm_corr_template[idx].imag();
			}
			ofdm.ofdm_corr_template_sym_energy[k] = sym_energy;
			ofdm.ofdm_corr_template_energy += sym_energy;
		}

		delete[] pb_data;
		delete[] pb_fir1;
		delete[] pb_fir2;
		delete[] filtered;
		delete[] bb_template;

		printf("[PHY] OFDM corr template: %d symbols, %d samples, energy=%.3f (matched filter, FIR round-tripped)\n",
			template_nsymb, bb_len, ofdm.ofdm_corr_template_energy);
		printf("[TMPL-INIT] t[0]=(%.6f,%.6f) t[1]=(%.6f,%.6f) t[2]=(%.6f,%.6f)\n",
			ofdm.ofdm_corr_template[0].real(), ofdm.ofdm_corr_template[0].imag(),
			ofdm.ofdm_corr_template[1].real(), ofdm.ofdm_corr_template[1].imag(),
			ofdm.ofdm_corr_template[2].real(), ofdm.ofdm_corr_template[2].imag());
		printf("[TMPL-INIT] FIR_ts: nTaps=%d cut=%.1f trans=%.1f\n",
			ofdm.FIR_rx_time_sync.filter_nTaps,
			ofdm.FIR_rx_time_sync.lpf_filter_cut_frequency,
			ofdm.FIR_rx_time_sync.filter_transition_bandwidth);
		printf("[TMPL-INIT] carrier_freq=%.1f output_power=%.3f pre_eq[0]=(%.4f,%.4f) pre_eq[1]=(%.4f,%.4f)\n",
			carrier_frequency, output_power_Watt,
			pre_equalization_channel[0].value.real(), pre_equalization_channel[0].value.imag(),
			pre_equalization_channel[1].value.real(), pre_equalization_channel[1].value.imag());
		fflush(stdout);
	}
#endif
	}

	bit_interleaver_block_size=data_container.nBits/10;
	time_freq_interleaver_block_size=data_container.nData/10;

	if(default_configurations_telecom_system.ofdm_time_sync_Nsymb==AUTO_SELLECT)
	{
		ofdm.time_sync_Nsymb=ofdm.Nsymb;
	}

	if(reinit_subsystems.microphone==YES)
	{
        // TODO: Do we need this?
#if 0
		microphone.baudrate = sampling_frequency;
		microphone.nbuffer_Samples = 2 * ofdm.Nfft*(1+ofdm.gi)*frequency_interpolation_rate*(ofdm.Nsymb+ofdm.preamble_configurator.Nsymb);
		microphone.frames_per_period = ofdm.Nfft*(1+ofdm.gi)*frequency_interpolation_rate;
		if(operation_mode != BER_PLOT_baseband &&
           operation_mode != BER_PLOT_passband &&
           operation_mode != TX_TEST)
		{
			microphone.init();
		}
		reinit_subsystems.microphone=NO;
#endif
	}

	if(reinit_subsystems.speaker==YES)
	{
        // TODO: Do we need this?
#if 0
		speaker.baudrate = sampling_frequency;
		speaker.nbuffer_Samples = 2 * ofdm.Nfft*(1+ofdm.gi)*frequency_interpolation_rate*(ofdm.Nsymb+ofdm.preamble_configurator.Nsymb);
		speaker.frames_per_period = ofdm.Nfft*(1+ofdm.gi)*frequency_interpolation_rate;
		if(operation_mode != BER_PLOT_baseband &&
           operation_mode != BER_PLOT_passband &&
           operation_mode != RX_TEST)
		{
			speaker.init();
		}
		reinit_subsystems.speaker=NO;
#endif
	}

	// Invalidate cached sync state from previous config - frame timing differs
	// between configs (preamble_nSymb varies 4 OFDM / 8 NB MFSK / 16 WB MFSK),
	// so old values would be wrong
	receive_stats.delay_of_last_decoded_message = -1;
	receive_stats.freq_offset_of_last_decoded_message = 0;
	consecutive_ofdm_decode_fails = 0;  // STALE-CFO scoped reset: config change re-acquires fresh
	receive_stats.mfsk_search_raw = 0;
	receive_stats.ofdm_search_raw = 0;
	receive_stats.ofdm_batch_active = false;
	receive_stats.ofdm_drift_per_frame = 0.0;

	printf("[PHY] Config %d active: M=%.0f LDPC_rate=%.3f BW=%.0fHz Nc=%d Nsymb=%d nBits=%d\n",
		current_configuration, M, ldpc.rate, bandwidth,
		data_container.Nc, data_container.Nsymb, data_container.nBits);
	if(M == MOD_MFSK)
	{
		const double _max_Nc = 50.0;
		double _mfsk_boost = _max_Nc * pow(10.0, -2.0 / 20.0) / sqrt((double)data_container.Nc * mfsk.nStreams);
		printf("[PHY] MFSK: M=%d nStreams=%d bps=%d offsets=[", mfsk.M, mfsk.nStreams, mfsk.bits_per_symbol());
		for(int i = 0; i < mfsk.nStreams; i++) printf("%s%d", i?",":"", mfsk.stream_offsets[i]);
		printf("] Nc=%d boost=%.1fdB\n", mfsk.Nc, 20.0*log10(_mfsk_boost));

		// Set up short control frame parameters for MFSK modes.
		// BER testing determined safe ctrl_nBits values (same waterfall as full frame):
		//   ROBUST_0 (rate 1/16, 32-MFSK×1): 1200 bits, waterfall -13 dB
		//   ROBUST_1 (rate 1/16, 16-MFSK×2): 1400 bits, waterfall -11 dB
		//   ROBUST_2 (rate 1/4): no puncturing (rate 1/4 can't tolerate it)
		int bps = mfsk.bits_per_symbol();
		if(current_configuration == ROBUST_0)
		{
			ctrl_nBits = 1200;
			ctrl_nsymb = ctrl_nBits / bps;  // 240
		}
		else if(current_configuration == ROBUST_1)
		{
			ctrl_nBits = 1400;
			ctrl_nsymb = ctrl_nBits / bps;  // 175
		}
		else
		{
			ctrl_nBits = 0;  // no puncturing
			ctrl_nsymb = 0;
		}
		mfsk_ctrl_mode = false;

		if(ctrl_nBits > 0)
			printf("[PHY] Ctrl frame: nBits=%d nsymb=%d (%.0f%% of data)\n",
				ctrl_nBits, ctrl_nsymb, 100.0 * ctrl_nsymb / data_container.Nsymb);

	}
	else
	{
		ctrl_nBits = 0;
		ctrl_nsymb = 0;
		mfsk_ctrl_mode = false;
	}

	// Universal ACK pattern: dedicated ack_mfsk with fixed M, nStreams=1 for ALL modes.
	// Config-independent: both sides always agree on ACK tone parameters,
	// so no config switching needed for ACK pattern TX/RX.
	{
		int ack_M = narrowband_enabled ? 8 : 16;  // M=8 fits in Nc=10, M=16 fits in Nc=50
		ack_mfsk.init(ack_M, data_container.Nc, 1);
		// NB: Sidelnikov sequences have intrinsic frequency diversity — disable hopping
		// so transmitted tones match the pre-computed sequence exactly.
		if (narrowband_enabled)
			ack_mfsk.tone_hop_step = 0;
	}

	ack_pattern_passband_samples = ack_mfsk.ack_pattern_nsymb * data_container.Nofdm * frequency_interpolation_rate;
	ack_snr_pattern_passband_samples = ack_mfsk.ack_snr_pattern_nsymb() * data_container.Nofdm * frequency_interpolation_rate;
	// ACK+SACK pattern (WB-only — NB has M=8 and ack_sack_suffix_len()=0,
	// which makes ack_sack_pattern_nsymb() == ack_pattern_nsymb. Multiplying
	// is still safe; downstream callers check ack_sack_suffix_len() > 0 before
	// invoking the TX path so NB falls back to OFDM_ACK_CLEAN / SACK_RSP.)
	ack_sack_pattern_passband_samples = ack_mfsk.ack_sack_pattern_nsymb() * data_container.Nofdm * frequency_interpolation_rate;
	// Phase B Wave 1: CONNECT base + ctrl-suffix (WB only — NB has
	// connect_pattern_nsymb=0 so this is also 0 on NB and downstream callers
	// gate via that). §19: ctrl_suffix_len() is the CODED length (52 with the
	// Tier-2 GF(16) FEC, 13 uncoded) — so this precomputed sample count, and
	// every TX consumer derived from it, sizes for the coded suffix when FEC is
	// on. FEC must be enabled (gf16ra::configure+init, ack_mfsk.suffix_fec_coded)
	// BEFORE load_configuration recomputes this, OR re-enabled after — the
	// session FEC-enable path re-derives this member (telecom enable hook §19).
	connect_pattern_passband_samples = ack_mfsk.connect_pattern_nsymb * data_container.Nofdm * frequency_interpolation_rate;
	// §20: base-pattern combining — the on-wire base is connect_base_total_nsymb()
	// (R×16 when combining, 16 when reps=1 → byte-identical). The session
	// combining-enable hook (set_connect_preamble_reps) re-derives this member, as
	// set_suffix_fec does for the coded suffix length.
	ctrl_suffix_pattern_passband_samples =
		(ack_mfsk.connect_base_total_nsymb() + ack_mfsk.ctrl_suffix_len()) * data_container.Nofdm * frequency_interpolation_rate;

	// Per-mode detection threshold (all using ack_mfsk: M=16, nStreams=1):
	// ROBUST_0 (-13 dB): low SNR, need conservative threshold
	// ROBUST_1/2 (-11/-8 dB): moderate SNR, standard threshold
	// OFDM (0 to +20 dB): high SNR, easy detection
	if(current_configuration == ROBUST_0)
		ack_pattern_detection_threshold = 0.65;
	else if(is_robust_config(current_configuration))
		ack_pattern_detection_threshold = 1.0;
	else
		ack_pattern_detection_threshold = 1.0;

	printf("[PHY] ACK pattern: %d symbols (M=%d, %s), %d passband samples (%.0f ms), "
		"match_threshold=%d/%d, detection_threshold=%.2f\n",
		ack_mfsk.ack_pattern_nsymb, ack_mfsk.M,
		narrowband_enabled ? "NB Sidelnikov" : "WB Welch-Costas",
		ack_pattern_passband_samples,
		1000.0 * ack_pattern_passband_samples / sampling_frequency,
		ack_mfsk.ack_match_threshold, ack_mfsk.ack_pattern_nsymb,
		ack_pattern_detection_threshold);
}

void cl_telecom_system::return_to_last_configuration()
{
	int tmp;
	this->load_configuration(last_configuration);
	tmp= last_configuration;
	last_configuration=current_configuration;
	current_configuration=tmp;
}

// SNR-to-config mapping for supershift (BER waterfall + 2 dB margin).
// Callers apply SUPERSHIFT_MARGIN_DB (3 dB) on top, so effective margin = 5 dB.
// BER waterfalls (100 frames, passband, EsN0):
//   C0:-14 C1:-11 C2:-10 C3:-9 C4:-8 C5:-7 C6:-6 C7:-5
//   C8:-4  C9:-2  C10:-1 C11:+1 C12:+2 C13:+4 C14:+7 C15:+9 C16:+12
int cl_telecom_system::get_configuration(double SNR)
{
	int configuration;

	if(SNR>13)
		configuration=CONFIG_16;
	else if(SNR>11)
		configuration=CONFIG_15;
	else if(SNR>9)
		configuration=CONFIG_14;
	else if(SNR>6)
		configuration=CONFIG_13;
	else if(SNR>4)
		configuration=CONFIG_12;
	else if(SNR>3)
		configuration=CONFIG_11;
	else if(SNR>1)
		configuration=CONFIG_10;
	else if(SNR>0)
		configuration=CONFIG_9;
	else if(SNR>-2)
		configuration=CONFIG_8;
	else if(SNR>-3)
		configuration=CONFIG_7;
	else if(SNR>-4)
		configuration=CONFIG_6;
	else if(SNR>-5)
		configuration=CONFIG_5;
	else if(SNR>-6)
		configuration=CONFIG_4;
	else if(SNR>-7)
		configuration=CONFIG_3;
	else if(SNR>-8)
		configuration=CONFIG_2;
	else if(SNR>-9)
		configuration=CONFIG_1;
	else
		configuration=CONFIG_0;

	return configuration;
}

// Per-instance RNG routing (single-process-sim-refactor.md §10.1). When
// rng_own_ is false (production default) these are the verbatim global
// __srandom/__random → byte-identical. When opted in (2-instance stepper) they
// drive this instance's independent rng_ stream.
void cl_telecom_system::ts_srandom(unsigned int seed)
{
	if (rng_own_) __srandom_r2(seed, &rng_);
	else          __srandom(seed);
}

long int cl_telecom_system::ts_random()
{
	if (rng_own_) return __random_r2(&rng_);
	return __random();
}

void cl_telecom_system::enable_per_instance_rng(unsigned int seed)
{
	os_rng_make(&rng_, rng_state_, seed);
	rng_own_ = true;
}

void cl_telecom_system::get_pre_equalization_channel()
{
	int nTries=1000;
	for(int i=0;i<data_container.Nc;i++)
	{
		pre_equalization_channel[i].value=0;
	}

	for(int j=0;j<nTries;j++)
	{
		for(int i=0;i<data_container.Nc*log2(data_container.M);i++)
		{
			data_container.bit_interleaved_data[i]=ts_random()%2;   // §10.1 pre-eq loop (per-instance when opted in)
		}
		psk.mod(data_container.bit_interleaved_data,data_container.Nc*log2(data_container.M),data_container.modulated_data);

		ofdm.symbol_mod(data_container.modulated_data,data_container.preamble_symbol_modulated_data);
		ofdm.passband_start_sample=0;
		ofdm.baseband_to_passband(data_container.preamble_symbol_modulated_data,data_container.Nofdm,data_container.passband_data_tx,sampling_frequency,carrier_frequency,carrier_amplitude,frequency_interpolation_rate);

		ofdm.FIR_tx1.apply(data_container.passband_data_tx,data_container.passband_data_tx_filtered_fir_1,data_container.Nofdm*frequency_interpolation_rate);
		ofdm.FIR_tx2.apply(data_container.passband_data_tx_filtered_fir_1,data_container.passband_data_tx_filtered_fir_2,data_container.Nofdm*frequency_interpolation_rate);

		ofdm.passband_to_baseband(data_container.passband_data_tx_filtered_fir_2,data_container.Nofdm*frequency_interpolation_rate,data_container.baseband_data,sampling_frequency,carrier_frequency,carrier_amplitude,data_container.interpolation_rate,&ofdm.FIR_rx_data);
		ofdm.symbol_demod(data_container.baseband_data,data_container.ofdm_symbol_demodulated_data);

		for(int i=0;i<data_container.Nc;i++)
		{
			pre_equalization_channel[i].value+=data_container.modulated_data[i]/data_container.ofdm_symbol_demodulated_data[i];
		}
	}

	for(int i=0;i<data_container.Nc;i++)
	{
		pre_equalization_channel[i].value/=nTries;
	}

	// DIAG: print pre-eq magnitude range
	{
		double min_mag = 1e30, max_mag = 0;
		for(int i=0;i<data_container.Nc;i++)
		{
			double m = std::abs(pre_equalization_channel[i].value);
			if(m < min_mag) min_mag = m;
			if(m > max_mag) max_mag = m;
		}
		printf("[PRE-EQ] Nc=%d min_mag=%.4f max_mag=%.4f\n", data_container.Nc, min_mag, max_mag);
		fflush(stdout);
	}
}

// 2D channel-state lookup accessors. Read-only — see fact-doc
// mercury/fact-documents/channel-state-2d-lookup.md §8 Step 1.
// The cached values are updated opportunistically inside the existing
// detect_ack/hail_pattern_from_passband() and the receive_byte() preamble
// channel-estimate path; no new computation paths are introduced.
double cl_telecom_system::get_correlator_snr_proxy() const
{
	return last_correlator_metric_db;
}

double cl_telecom_system::get_channel_selectivity() const
{
	return last_channel_selectivity;
}
