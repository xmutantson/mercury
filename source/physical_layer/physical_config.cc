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

#include "physical_layer/physical_config.h"

extern double carrier_frequency_offset;
extern int radio_type;
extern char *input_dev;
extern char *output_dev;

cl_configuration_telecom_system::cl_configuration_telecom_system()
{
    // TODO: parametrize most important parameters here
	init_configuration=CONFIG_0;

	ofdm_Nc=AUTO_SELLECT;
	ofdm_Nfft=256;
	ofdm_gi=54.0/256.0;  // Ngi=54 → 4.5ms GI (covers most HF multipath, ITU moderate+poor)
	ofdm_Nsymb=AUTO_SELLECT;
	ofdm_pilot_configurator_Dx=AUTO_SELLECT;
	ofdm_pilot_configurator_Dy=AUTO_SELLECT;
	ofdm_pilot_configurator_first_row=DATA;
	ofdm_pilot_configurator_last_row=DATA;
	ofdm_pilot_configurator_first_col=DATA;
	ofdm_pilot_configurator_second_col=DATA;
	ofdm_pilot_configurator_last_col=AUTO_SELLECT;
	ofdm_pilot_configurator_pilot_boost=1.33;
	ofdm_pilot_configurator_seed=0;
	ofdm_pilot_density=HIGH_DENSITY;

	ofdm_preamble_configurator_Nsymb=4;
	ofdm_preamble_configurator_nIdentical_sections=2;
	ofdm_preamble_configurator_modulation=MOD_QPSK;
	ofdm_preamble_configurator_boost=sqrt(2);
	ofdm_preamble_configurator_seed=1;

	ofdm_time_sync_Nsymb=AUTO_SELLECT;


	ofdm_freq_offset_ignore_limit=0.1;
	ofdm_start_shift=1;

	ofdm_channel_estimator=LEAST_SQUARE;
	ofdm_channel_estimator_amplitude_restoration=NO;
	// LS frequency-window width = 1 (per-subcarrier LS, NO horizontal boxcar).
	// ROOT FIX (fix/cfg16-dft-leakage): width=2 averaged the LS pilot estimate
	// over ±1 adjacent subcarrier (a 3-tap frequency boxcar). On a FLAT channel
	// that harmlessly denoises (all H equal); on a FREQUENCY-SELECTIVE channel it
	// LOW-PASS-FILTERS H in frequency, smearing the ripple and biasing faded/edge
	// carriers. 32-QAM (min-distance ~2x tighter than 16-QAM) then slices the
	// biased H wrong -> an SNR-INDEPENDENT cfg16 error floor on any multipath
	// (uncoded fsel BER 0.186 vs GENIE 0.0001; measured floor 0.02 @Es/N0=30dB).
	// Decisive A/B: --ls-window=1x8 collapses the cfg16 fsel floor to EXACTLY 0
	// (BER waterfalls to 0 from Es/N0>=13dB, matching the flat-AWGN threshold and
	// the GENIE bound). Per-subcarrier LS is the unbiased estimator; the
	// coherence-bandwidth-matched frequency denoising is done CORRECTLY (and
	// leakage-free) by the DFT delay-domain smoother below, which the 3-tap boxcar
	// was crudely and biasedly approximating. cfg13/15, cfg6/12 (WB climb path)
	// and NB cfg10 show no threshold regression; the only cost is a sub-1-dB
	// flat-AWGN softening on cfg15/16 BELOW their operating SNR (does not move the
	// clean-decode threshold). Ref Y. Li, L. J. Cimini, N. R. Sollenberger,
	// "Robust channel estimation for OFDM systems with rapid dispersive fading
	// channels," IEEE Trans. Commun. 46(7):902-915, 1998 (frequency averaging must
	// match the coherence bandwidth); O. Edfors et al., "On channel estimation in
	// OFDM systems," IEEE VTC 1995 (DFT delay-domain denoiser).
	ofdm_LS_window_width=1;
	ofdm_LS_window_hight=8;

	bit_energy_dispersal_seed=0;

	ldpc_standard=MERCURY;
	ldpc_framesize=MERCURY_NORMAL;

	ldpc_decoding_algorithm=SPA;
	ldpc_GBF_eta=0.5;
	ldpc_nIteration_max=100;
	ldpc_print_nIteration=NO;

	outer_code=CRC16_MODBUS_RTU;

	frequency_interpolation_rate=4; // should we change to 8 when samplerate is 96 kHz?

	bandwidth=48000.0*50.0/ofdm_Nfft/frequency_interpolation_rate;

	printf("Bandwidth: %f Hz\n", bandwidth);

	time_sync_trials_max=2;
	use_last_good_time_sync=YES;
	use_last_good_freq_offset=YES;
	carrier_frequency = carrier_frequency_offset + 1500.0;  // Center at 1500 Hz (SSB passband center)
	output_power_Watt=0.1;

	printf("Center frequency: %f Hz low: %f Hz high: %f Hz\n", carrier_frequency, carrier_frequency - bandwidth/2, carrier_frequency + bandwidth/2);

	ofdm_FIR_rx_time_sync_filter_window=HAMMING;
	ofdm_FIR_rx_time_sync_filter_transition_bandwidth=3000;
	ofdm_FIR_rx_time_sync_lpf_filter_cut_frequency=0.9*bandwidth/2;
	ofdm_FIR_rx_time_sync_filter_type=LPF;

	ofdm_FIR_rx_data_filter_window=HAMMING;
	ofdm_FIR_rx_data_filter_transition_bandwidth=3000;
	ofdm_FIR_rx_data_lpf_filter_cut_frequency=1.0*bandwidth/2;
	ofdm_FIR_rx_data_filter_type=LPF;

	ofdm_FIR_tx1_filter_window=HAMMING;
	ofdm_FIR_tx1_filter_transition_bandwidth=1000;
	ofdm_FIR_tx1_lpf_filter_cut_frequency=carrier_frequency+bandwidth/2;
	ofdm_FIR_tx1_hpf_filter_cut_frequency=carrier_frequency-bandwidth/2;
	ofdm_FIR_tx1_filter_type=HPF;

	ofdm_FIR_tx2_filter_window=BLACKMAN;
	ofdm_FIR_tx2_filter_transition_bandwidth=1000;
	ofdm_FIR_tx2_lpf_filter_cut_frequency=carrier_frequency+bandwidth/2;
	ofdm_FIR_tx2_hpf_filter_cut_frequency=carrier_frequency-bandwidth/2;
	ofdm_FIR_tx2_filter_type=LPF;

	ofdm_preamble_papr_cut=7;
	ofdm_data_papr_cut=10;

	// This folder can be in a ramdisk (eg: mount -t tmpfs -o size=128M tmpfs /mnt/ramDisk)
	plot_folder="./";
	plot_plot_active=NO;

}

cl_configuration_telecom_system::~cl_configuration_telecom_system()
{

}







