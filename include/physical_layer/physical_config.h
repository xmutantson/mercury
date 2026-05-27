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

#ifndef INC_PHYSICAL_CONFIG_H_
#define INC_PHYSICAL_CONFIG_H_

#include "physical_defines.h"
#include "ofdm.h"
#include "ldpc.h"
#include "fir_filter.h"
#include "psk.h"
#include "crc16_modbus_rtu.h"

struct cl_configuration_telecom_system
{
private:
public:
	cl_configuration_telecom_system();
	~cl_configuration_telecom_system();

	int init_configuration;   // config ID (CONFIG_0..16); int for type-consistency with the signed config members it feeds

	int ofdm_Nc;
	int ofdm_Nfft;
	float ofdm_gi;
	int ofdm_Nsymb;
	int ofdm_pilot_configurator_Dx;
	int ofdm_pilot_configurator_Dy;
	int ofdm_pilot_configurator_first_row;
	int ofdm_pilot_configurator_last_row;
	int ofdm_pilot_configurator_first_col;
	int ofdm_pilot_configurator_second_col;
	int ofdm_pilot_configurator_last_col;
	float ofdm_pilot_configurator_pilot_boost;
	int ofdm_pilot_configurator_seed;
	int ofdm_pilot_density;

	int ofdm_preamble_configurator_Nsymb;
	int ofdm_preamble_configurator_nIdentical_sections;
	int ofdm_preamble_configurator_modulation;
	double ofdm_preamble_configurator_boost;
	int ofdm_preamble_configurator_seed;

	double ofdm_preamble_papr_cut;
	double ofdm_data_papr_cut;

	int ofdm_channel_estimator;
	int ofdm_channel_estimator_amplitude_restoration;
	int ofdm_LS_window_width;
	int ofdm_LS_window_hight;

	float ofdm_freq_offset_ignore_limit;
	int ofdm_start_shift;

	int ldpc_standard;
	int ldpc_framesize;

	int ldpc_decoding_algorithm;
	float ldpc_GBF_eta;
	int ldpc_nIteration_max;
	int ldpc_print_nIteration;

	// Phase A.2 §7.5 BP+OSD knobs. Plumbed onto cl_ldpc::osd_norder /
	// cl_ldpc::osd_maxosd at load_configuration time (telecom_system.cc:~4694).
	// Defaults match research §4.6 / cl_ldpc constructor defaults.
	int ldpc_osd_norder;     // 0..3; -1 = OSD disabled (BP-only).
	int ldpc_osd_maxosd;     // 0 = single OSD call on BP fail; >0 reserved.

	int outer_code;

	double bandwidth;
	int time_sync_trials_max;
	int use_last_good_time_sync;
	int use_last_good_freq_offset;
	int frequency_interpolation_rate;
	double carrier_frequency;
	double output_power_Watt;

	int ofdm_FIR_rx_data_filter_window;
	double ofdm_FIR_rx_data_filter_transition_bandwidth;
	double ofdm_FIR_rx_data_lpf_filter_cut_frequency;
	int ofdm_FIR_rx_data_filter_type;

	int ofdm_FIR_rx_time_sync_filter_window;
	double ofdm_FIR_rx_time_sync_filter_transition_bandwidth;
	double ofdm_FIR_rx_time_sync_lpf_filter_cut_frequency;
	int ofdm_FIR_rx_time_sync_filter_type;

	int ofdm_FIR_tx1_filter_window;
	double ofdm_FIR_tx1_filter_transition_bandwidth;
	double ofdm_FIR_tx1_lpf_filter_cut_frequency;
	double ofdm_FIR_tx1_hpf_filter_cut_frequency;
	int ofdm_FIR_tx1_filter_type;

	int ofdm_FIR_tx2_filter_window;
	double ofdm_FIR_tx2_filter_transition_bandwidth;
	double ofdm_FIR_tx2_lpf_filter_cut_frequency;
	double ofdm_FIR_tx2_hpf_filter_cut_frequency;
	int ofdm_FIR_tx2_filter_type;

	int ofdm_time_sync_Nsymb;

	int bit_energy_dispersal_seed;


	std::string plot_folder;
	int plot_plot_active;
};




#endif
