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
#include <chrono>
#include <vector>  // suffix-FEC soft decode candidate buffers
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
	skip_var_gate_enabled = true;  // default = HEAD behavior; CLI --skip-var-gate=off disables
	rx_normalize_enabled  = true;  // default = HEAD behavior; CLI --rx-normalize=off disables
	csi_llr_enabled       = true;  // default = HEAD behavior; CLI --csi-llr=off disables
	fsel_test_enabled     = false; // fix/cfg16-nv-restore: --fsel-test=on enables (BER loopback only)
	fsel_amp              = 0.6;   // second-ray amplitude (linear)
	fsel_delay            = 128;   // second-ray delay in passband samples (~Nfft/8 @ interp=4, within GI)
	ber_single_esn0       = -999.0f; // fix/cfg16-nv-restore: <=-900 = normal full sweep
	ber_frames_override   = 0;     // 0 = use sweep default frame count
	mean_h_gate_threshold = 0.30;  // default = HEAD (b806b76); pre-IONOS was 0.50
	energy_gate_floor    = 1e-12;  // default = HEAD (b806b76); pre-IONOS was 0.001
	ofdm_defer_overflow_enabled = true; // default = HEAD (7076a4b Fix A)
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
			data_container.data_bit[i]=__random()%2;
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
			data_container.data_bit[i]=__random()%2;
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

		// fix/cfg16-nv-restore: optional static 2-ray frequency-selective channel,
		// applied to the TX passband BEFORE AWGN. Off by default (production BER
		// unchanged). y[n] = x[n] + fsel_amp * x[n-fsel_delay]; iterate backward so
		// the in-place tap reads only unmodified earlier samples. This is the
		// condition the LS-path nv bug needs (flat AWGN cannot reproduce it).
		if(fsel_test_enabled && M != MOD_MFSK && fsel_delay > 0)
		{
			int nSamp = (data_container.Nofdm * (data_container.Nsymb + data_container.preamble_nSymb)) * this->frequency_interpolation_rate;
			for(int n = nSamp - 1; n >= fsel_delay; n--)
			{
				data_container.passband_data[n] += fsel_amp * data_container.passband_data[n - fsel_delay];
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

void cl_telecom_system::transmit_byte(int *data, int nBytes, double* out, int message_location)
{
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

	for(int i=0;i<data_container.preamble_nSymb;i++)
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

	for(int j=0;j<data_container.Nofdm*data_container.preamble_nSymb;j++)
	{
		data_container.preamble_symbol_modulated_data[j]/=power_normalization;
		data_container.preamble_symbol_modulated_data[j]*=sqrt(output_power_Watt)*preamble_boost*mfsk_boost;
	}

	for(int j=0;j<data_container.Nofdm*active_nsymb;j++)
	{
		data_container.ofdm_symbol_modulated_data[j]/=power_normalization;
		data_container.ofdm_symbol_modulated_data[j]*=sqrt(output_power_Watt)*mfsk_boost;
	}

	// Apply test TX carrier offset for frequency sync testing
	double tx_carrier = carrier_frequency + test_tx_carrier_offset;
	ofdm.baseband_to_passband(data_container.preamble_symbol_modulated_data,data_container.Nofdm*data_container.preamble_nSymb,data_container.passband_data_tx,sampling_frequency,tx_carrier,carrier_amplitude,frequency_interpolation_rate);
	ofdm.baseband_to_passband(data_container.ofdm_symbol_modulated_data,data_container.Nofdm*active_nsymb,&data_container.passband_data_tx[data_container.Nofdm*data_container.preamble_nSymb*frequency_interpolation_rate],sampling_frequency,tx_carrier,carrier_amplitude,frequency_interpolation_rate);

	ofdm.peak_clip(data_container.passband_data_tx, data_container.Nofdm*data_container.preamble_nSymb*frequency_interpolation_rate,ofdm.preamble_papr_cut);
	ofdm.peak_clip(&data_container.passband_data_tx[data_container.Nofdm*data_container.preamble_nSymb*frequency_interpolation_rate], data_container.Nofdm*active_nsymb*frequency_interpolation_rate,ofdm.data_papr_cut);

	if(message_location==NO_FILTER_MESSAGE)
	{
		for(int i=0;i<data_container.total_frame_size;i++)
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

	// Timing breakdown
	double timing_pb_tsync_ms = 0, timing_pb_data_ms = 0, timing_ldpc_ms = 0;
	auto timing_total_start = std::chrono::steady_clock::now();

	int step=100;
	int pream_symb_loc;

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
			int pb_samples = data_container.Nofdm * data_container.buffer_Nsymb * frequency_interpolation_rate;
			double* pb = (double*)data;
			double sum_sq = 0.0;
			for(int i = 0; i < pb_samples; i++)
				sum_sq += pb[i] * pb[i];
			double rms = sqrt(sum_sq / pb_samples);
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
						rms *= scale;
					}
				}
				// Impulse noise blanking: clip at 10× RMS (always on)
				double clip_threshold = 10.0 * rms;
				for(int i = 0; i < pb_samples; i++) {
					if(pb[i] > clip_threshold) { pb[i] = clip_threshold; }
					else if(pb[i] < -clip_threshold) { pb[i] = -clip_threshold; }
				}
			}
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

			if(receive_stats.ofdm_batch_active && receive_stats.ofdm_search_raw > 0)
			{
				// BATCH mode: predict + verify. After successful decode, the next
				// preamble position is predictable from ofdm_skip. Try a tiny
				// verify window first; fall back to wider search on failure.
				int gi_interp = data_container.Ngi * interp;
				int preamble_interp = data_container.preamble_nSymb * sym_samples;
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
					TimeSyncResult verify = ofdm.time_sync_preamble_halfsym(
						&data_container.baseband_data_decimated[verify_start_dec],
						verify_size_dec, 1, 1);
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

				if(!batch_verified)
				{
					// Prediction failed — search full remaining buffer from
					// ofdm_skip onwards (same as initial search).  The narrow
					// forward_look=40 cap was causing preambles >40 symbols
					// past ofdm_skip to be missed, especially after turnaround
					// gaps where the next preamble arrives much later.
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
		int frame_end_samples = receive_stats.delay + (data_container.preamble_nSymb + active_nsymb) * sym_samples;
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
	int upper_bound = data_container.buffer_Nsymb-(data_container.Nsymb+data_container.preamble_nSymb);
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
				int s8_win_len = (ofdm.preamble_configurator.Nsymb + 4) * data_container.Nofdm * s8_M;
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
					data_container.interpolation_rate, receive_stats.sync_trials, 1, time_sync_trials_max);
				receive_stats.delay = s8_win_start + fine_result.delay;
			}

			if(receive_stats.delay<0){receive_stats.delay=0;}


			// Clamp delay to prevent buffer overflow in rational_resampler
			{
				int buf_size = data_container.Nofdm * data_container.buffer_Nsymb * frequency_interpolation_rate;
				int frame_size = (data_container.Nofdm*(data_container.Nsymb+data_container.preamble_nSymb))*frequency_interpolation_rate;
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
			int extraction_delay = receive_stats.delay;
			int buf_size_interp = data_container.Nofdm * data_container.buffer_Nsymb * frequency_interpolation_rate;
			int frame_size_interp = (data_container.Nofdm*(data_container.Nsymb+data_container.preamble_nSymb))*frequency_interpolation_rate;
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
			else
			{
				// Fine frequency sync (Moose algorithm) - ±0.5 subcarrier range
				// Note: Coarse offset already applied before this loop, so Moose measures residual
				// BUG FIX: baseband_data is at decimated (base) rate after rational_resampler,
				// so guard interval skip is Ngi samples, NOT Ngi*interpolation_rate.
				// The old code skipped Ngi*4=256=Nfft samples, reading across symbol boundaries.
				freq_offset_measured=ofdm.carrier_sampling_frequency_sync(&data_container.baseband_data[data_container.Ngi],bandwidth/(double)data_container.Nc,data_container.preamble_nSymb, sampling_frequency);
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
				for(int i=0;i<rx_nsymb;i++)
				{
					ofdm.symbol_demod(&data_container.baseband_data[i*data_container.Nofdm+data_container.Nofdm*data_container.preamble_nSymb],&data_container.ofdm_symbol_demodulated_data[i*data_container.Nc]);
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

				// Noise variance gate: if noise_variance > 0.5 (SNR < ~3 dB), the
				// signal is either noise (false preamble detection) or completely
				// unusable. Good frames: var=0.01-0.10. Garbage: var=1.7-3.3.
				// Skip LDPC to free receiver for real frames.
				// Phase-2 validation: --skip-var-gate=off bypasses this gate.
				//
				// Phase-F stall fix (2026-05-12): if 3+ consecutive trials all
				// SKIP-VAR, the entire trial range is noise — abort the trial
				// loop entirely so the caller advances the buffer past this
				// noise region instead of burning 20 trials (~2 s) on it.
				if(skip_var_gate_enabled && ofdm.noise_variance_estimate > 0.5)
				{
					printf("[OFDM-SYNC] trial %d SKIP-VAR: var=%.4f too high (>0.5), skipping LDPC\n",
						receive_stats.sync_trials, ofdm.noise_variance_estimate);
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
				printf("[FRAME-NV] trial=%d cfg=%d nv=%.6e mvar=%.4f Nsymb=%d amprest=%d\n",
					receive_stats.sync_trials, current_configuration,
					ofdm.noise_variance_estimate, measure_var, ofdm.Nsymb,
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
				psk.demod(data_container.ofdm_time_freq_deinterleaved_data,data_container.nBits,data_container.demodulated_data,variance);

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
					psk.demod(data_container.ofdm_time_freq_deinterleaved_data,data_container.nBits,data_container.demodulated_data,variance);
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
		(ack_mfsk.connect_base_total_nsymb() + ack_mfsk.ctrl_suffix_total_nsymb())
		* data_container.Nofdm * frequency_interpolation_rate;
	return ack_mfsk.ctrl_suffix_len();
}

// ULTRA reframe (Change 2): set the CONNECT suffix-combining factor R_suffix. Must
// be called AFTER load_configuration (and after set_suffix_fec, since the suffix
// length depends on whether FEC is on). R_suffix=1 = off (byte-identical to §20).
// Re-derives the passband sample count so every TX consumer sees the
// base + R_suffix×suffix length. CONNECT-only.
int cl_telecom_system::set_connect_suffix_reps(int reps)
{
	if (reps < 1) reps = 1;
	if (reps > cl_mfsk::MAX_CONNECT_SUFFIX_REPS)
		reps = cl_mfsk::MAX_CONNECT_SUFFIX_REPS;
	ack_mfsk.connect_suffix_reps = reps;
	ctrl_suffix_pattern_passband_samples =
		(ack_mfsk.connect_base_total_nsymb() + ack_mfsk.ctrl_suffix_total_nsymb())
		* data_container.Nofdm * frequency_interpolation_rate;
	return ack_mfsk.ctrl_suffix_total_nsymb();
}

// ULTRA tier per-tier CONNECT ctrl-suffix params (the §22.4 stacked table). SOLE
// owner of the ULTRA PHY numbers. Returns false for non-ULTRA configs (out params
// untouched) so the robust/OFDM tiers can never inherit ULTRA's deep settings.
bool cl_telecom_system::ultra_tier_suffix_params(int config, int& repfact, int& K,
                                                 int& R_base, int& R_suffix, int& R_frame)
{
	// §22.4 measured/interpolated stacked configs (per-frame content + base-combine
	// co-limit): ULTRA_0 establishment ~−17.8, ULTRA_2 ~−19.9 dB SNR3k (sim, AWGN).
	// R_frame = whole-CONNECT-frame repetition (Lever D, §2.5/§4.1): the CMD emits
	// START_CONN R_frame× back-to-back per HAIL cycle; the RSP window covers all
	// reps (INCR-2 choreography fix for the §9 HW timeout). ULTRA_0=2 / 1=3 / 2=4.
	switch (config)
	{
		case ULTRA_0: repfact = 6; K = 8; R_base = 8; R_suffix = 8;  R_frame = 2; return true;
		case ULTRA_1: repfact = 7; K = 6; R_base = 8; R_suffix = 10; R_frame = 3; return true; // interpolated
		case ULTRA_2: repfact = 8; K = 5; R_base = 8; R_suffix = 12; R_frame = 4; return true;
		default: return false;
	}
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
		(ack_mfsk.connect_base_total_nsymb() + ack_mfsk.ctrl_suffix_total_nsymb())
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
	// ULTRA reframe (Change 2): suffix on-wire length is R_suffix × coded N.
	int nsymb = ack_mfsk.connect_base_total_nsymb() + ack_mfsk.ctrl_suffix_total_nsymb();
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

// =============================================================================
// ULTRA tier — count-based ctrl-suffix admission (tier2-suffix-fec-design.md §22).
// =============================================================================
// INCR-0 found two blockers below ~−14 dB SNR3k on the CONNECT establishment
// path: (1) the scale-invariant `metric >= CTRL_DETECT_METRIC_MIN` ratio gate
// (combining sums R copies but the RATIO is unchanged → it gates the decode off
// at ~−14 regardless of FEC/combining strength), and (2) combining was on the
// PREAMBLE/base only, not the suffix FEC content. The §22 reframe removes BOTH:
// suffix-energy combining (cl_mfsk::connect_suffix_reps, always-on plumbing,
// applied at the ULTRA tier) + COUNT-BASED ADMISSION (this gate).
//
// COUNT-BASED ADMISSION: the ctrl-suffix decode admits on the HARD matched-COUNT
//   gate alone (matched >= connect_match_threshold), DROPPING the scale-invariant
//   metric-ratio sub-gate. The count gate (7/16) is the combining-aware
//   acquisition statistic (alive to −20 R=16 / −23 R=32, §22.1); CRC12 (2⁻¹²) +
//   2-bit type are the FAR backstop (§22.5: count-admission FAR 0/4000 pure-noise
//   even at the deepest ULTRA_2 R/rate). The §16 isolation already proved the
//   gate fully OFF is FAR-safe on the CRC-backstopped FEC path.
//
// PRODUCTION GAP #2 — TIER-GATED, NEVER A GLOBAL: the spike used a process-global
//   test flag, which would change merged acquisition + throughput behavior if it
//   leaked to the OFDM (CONFIG_0..16) / ROBUST tiers. Count-admission is therefore
//   active ONLY when the RX session is at an ULTRA config (is_ultra_config(
//   current_configuration)). At OFDM/ROBUST the scale-invariant ratio gate stays
//   FULLY INTACT (byte-identical decode admission, no throughput/no-regression
//   impact). The decision is `ultra_count_admission_active()`:
//     override == FOLLOW_TIER (default) → is_ultra_config(current_configuration);
//     override == 0/1 → forced off/on (TEST-ONLY isolation A/B, e.g. the §22
//                       per-change breakdown — production never sets it).
// The esno-scale knob the spike carried is DROPPED: §22.3 measured esno ×1 / ×√R
// / ×R all give the identical −16.07 cliff (the self-normalizing soft_decode
// captures the combining gain from the energy-RATIO separation), so esno stays
// 4.0 and the scale-invariance trap is avoided by construction.
namespace {
	enum { ULTRA_ADMIT_FOLLOW_TIER = -1 };
	int g_ultra_count_admission_override = ULTRA_ADMIT_FOLLOW_TIER;  // TEST-only
}
// TEST-only override for the §22 per-change isolation arms. -1 = follow the tier
// gate (production). 0/1 = force off/on. Production code never calls this.
void cl_telecom_system_set_ultra_count_admission_override(int v)
{
	g_ultra_count_admission_override = (v < 0) ? ULTRA_ADMIT_FOLLOW_TIER : (v ? 1 : 0);
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
		&best_offset, /*reserve_after=*/ack_mfsk.ctrl_suffix_total_nsymb(),
		/*out_match_mask=*/nullptr, /*always_fine=*/false,
		/*combine_reps=*/ack_mfsk.connect_preamble_reps);

	if (out_matched) *out_matched = matched;

	// ULTRA tier (Change 1 — count-based admission, §22): the metric-ratio sub-gate
	// is scale-invariant (combining can't lift it), so below ~−14 it gates the
	// decode off even though the combining-aware matched-COUNT is still alive (to
	// −20/−23). At the ULTRA tier ONLY (production gap #2: NEVER at OFDM/ROBUST —
	// there the ratio gate stays fully intact, byte-identical admission) we admit on
	// the HARD count gate alone and let CRC12 + 2-bit type be the FAR backstop
	// (§22.5: 0/4000 false-accepts even at the deepest ULTRA_2 R/rate).
	const bool count_admit =
		(g_ultra_count_admission_override == ULTRA_ADMIT_FOLLOW_TIER)
			? is_ultra_config(current_configuration)
			: (g_ultra_count_admission_override == 1);
	if (matched < ack_mfsk.connect_match_threshold ||
	    (!count_admit && metric < cl_mfsk::CTRL_DETECT_METRIC_MIN) ||
	    best_offset < 0)
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
			&rebest_offset, /*reserve_after=*/ack_mfsk.ctrl_suffix_total_nsymb(),
			/*out_match_mask=*/nullptr, /*always_fine=*/false,
			/*combine_reps=*/ack_mfsk.connect_preamble_reps);
		if (rematched >= ack_mfsk.connect_match_threshold &&
		    (count_admit || remetric >= cl_mfsk::CTRL_DETECT_METRIC_MIN) &&
		    rebest_offset >= 0)
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
		//
		// ULTRA reframe (Change 2 — SUFFIX combining): the TX emits the N-symbol
		// codeword connect_suffix_reps times back-to-back after the base. Rep r sits
		// at symbol offset connect_base_total_nsymb() + r*N; its tone-hop used the
		// continued abs index (base_total + r*N + s), which is EXACTLY the abs_s
		// decode_suffix_energies computes when passed pattern_nsymb = base_total + r*N.
		// So we extract rep r's N×M energy matrix and ADD it into `energies` — the
		// de-hop lands every rep's symbol s on the same data-tone slot, so this is a
		// per-tone noncoherent energy SUM across reps (square-law combining,
		// +2.2-2.5 dB/doubling). reps=1 → exactly the §20 single-codeword extraction.
		const int R_suffix = ack_mfsk.connect_suffix_reps >= 1 ? ack_mfsk.connect_suffix_reps : 1;
		const int base_total = ack_mfsk.connect_base_total_nsymb();
		if (R_suffix <= 1) {
			ofdm.decode_suffix_energies(
				data_container.baseband_data_interpolated, dec_size,
				1,
				best_offset, base_total,
				N,
				ack_mfsk.tone_hop_step, ack_mfsk.M,
				ack_mfsk.nStreams, ack_mfsk.stream_offsets,
				energies.data());
		} else {
			std::vector<double> rep_e((size_t)N * ack_mfsk.M, 0.0);
			for (int r = 0; r < R_suffix; r++) {
				ofdm.decode_suffix_energies(
					data_container.baseband_data_interpolated, dec_size,
					1,
					best_offset, base_total + r * N,
					N,
					ack_mfsk.tone_hop_step, ack_mfsk.M,
					ack_mfsk.nStreams, ack_mfsk.stream_offsets,
					rep_e.data());
				for (size_t i = 0; i < (size_t)N * ack_mfsk.M; i++)
					energies[i] += rep_e[i];
			}
		}

		// soft_decode requires a 2-bit expected type. The production caller
		// (receive_mfsk_ctrl_suffix_phy_core) already routes by type AFTER this
		// returns; to preserve that flow we accept ANY of the 4 types here, then
		// hand the decoded type up. soft_decode itself only accepts when its
		// internal CRC over [type|payload38] matches the decoded CRC, so each
		// type attempt is CRC-backstopped (FAR per type ≈ 2^-12; ×4 types still
		// ≈ 2^-10, far below the count-gate FAR). First CRC-valid type wins.
		//
		// ULTRA Change 2 (suffix combining): the COMBINED energy matrix (summed over
		// R_suffix reps) has R_suffix× the per-rep Es/No, but soft_decode auto-
		// estimates sigma from the matrix MEAN, which self-normalizes. §22.3 measured
		// that esno ×1 / ×√R / ×R all give the IDENTICAL −16.07 cliff — the combining
		// gain comes from the improved per-tone energy-RATIO separation (summed signal
		// ∝R, noise std ∝√R → decision SNR ∝√R), which the self-normalizing decode
		// already captures. So esno stays 4.0 (no rescale knob — dropped to avoid the
		// scale-invariance trap the §22 spike noted).
		const double esno_metric = 4.0;
		uint64_t p38 = 0;
		int iters = -1;
		for (int t = 0; t < 4; t++) {
			if (gf16ra::soft_decode(energies.data(),
				/*maxiter=*/50, /*esno_metric=*/esno_metric,
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

	__srandom (bit_energy_dispersal_seed);
	bit_energy_dispersal_seed = default_configurations_telecom_system.bit_energy_dispersal_seed;
	for(int i=0;i<ldpc.N;i++)
	{
		data_container.bit_energy_dispersal_sequence[i]=__random()%2;
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

	if(configuration<0 || (configuration>=NUMBER_OF_CONFIGS && !is_robust_config(configuration)
	                        && !is_ultra_config(configuration)))
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
	else if(is_ultra_config(configuration))
	{
		// ULTRA tier (ultra-tier-design.md §4.1 / tier2 §22): the deep-SNR survival
		// tier reuses the ROBUST_0-class MFSK DATA PHY (single-stream, M=32 WB / M=8
		// NB, LDPC 1/16 — the deepest robust data modcod). ULTRA's reach is NOT in
		// the data modcod; it is in the CONNECT ctrl-suffix establishment, pushed
		// deeper by the ULTRA enable hook (count-admission + low-rate GF(16) RA +
		// suffix combining, applied via ultra_tier_suffix_params()). So the PHY load
		// here is identical to ROBUST_0; the tier-distinguishing behavior is layered
		// on at the ctrl-suffix path. M/nStreams set in the MFSK reinit block below.
		_modulation=MOD_MFSK;
		_ldpc_rate=1/16.0;
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
		// ULTRA configs reuse the ROBUST_0 single-stream MFSK data PHY (M=32 WB /
		// M=8 NB) — see the is_ultra_config load block above.
		if(configuration == ROBUST_0 || is_ultra_config(configuration))
		                              { new_mfsk_M = narrowband_enabled ? 8 : 32; new_nStreams = 1; }
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
	// ULTRA reframe (Change 2): ctrl_suffix_total_nsymb() = R_suffix × coded N
	// (= ctrl_suffix_len() when R_suffix=1 → byte-identical to §20).
	ctrl_suffix_pattern_passband_samples =
		(ack_mfsk.connect_base_total_nsymb() + ack_mfsk.ctrl_suffix_total_nsymb()) * data_container.Nofdm * frequency_interpolation_rate;

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
			data_container.bit_interleaved_data[i]=__random()%2;
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
