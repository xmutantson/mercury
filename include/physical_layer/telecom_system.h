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

#ifndef INC_TELECOM_SYSTEM_H_
#define INC_TELECOM_SYSTEM_H_

#include "data_container.h"
#include "psk.h"
#include "mfsk.h"
#include "awgn.h"
#include "error_rate.h"
#include "plot.h"
#include "ofdm.h"
#include "ldpc.h"
#include "interleaver.h"
#include "physical_config.h"
#include "physical_defines.h"
#include "misc.h"
#include "common/ring_buffer_posix.h"
#include <iomanip>


#if defined(_WIN32)
#define msleep(a) Sleep(a)
#else
#define msleep(a) usleep(a * 1000)
#endif

// TX gain table: amplitude boost per signal type × NB/WB mode
// Indexed as tx_gain[signal_type][nb_mode][nb_fir]
// nb_mode: 0=WB modulation (Nc=50), 1=NB modulation (Nc=10)
// nb_fir:  0=WB FIR filter,         1=NB FIR filter
// Currently mod and FIR always match; cross-entries exist for future per-mode tuning.
enum tx_signal_type {
	TX_SIG_MFSK_1S = 0,  // MFSK data frame, 1 stream  (ROBUST_0)
	TX_SIG_MFSK_2S,       // MFSK data frame, 2 streams (ROBUST_1, ROBUST_2)
	TX_SIG_OFDM,          // OFDM data frame (CONFIG_0..CONFIG_16)
	TX_SIG_ACK,           // ACK pattern
	TX_SIG_BREAK,         // BREAK pattern
	TX_SIG_COUNT
};

struct st_reinit_subsystems{
	int microphone=YES;
	int speaker=YES;
	int telecom_system=YES;
	int data_container=YES;
	int ofdm_FIR_rx_data=YES;
	int ofdm_FIR_rx_time_sync=YES;
	int ofdm_FIR_tx1=YES;
	int ofdm_FIR_tx2=YES;
	int ofdm=YES;
	int ldpc=YES;
	int psk=YES;
	int pre_equalization_channel=YES;
};

struct st_receive_stats{
	int iterations_done;
	int delay;
	int delay_of_last_decoded_message;
	int time_peak_symb_location;
	int time_peak_subsymb_location;
	int sync_trials;
	double phase_error_avg;
	double freq_offset;
	double freq_offset_of_last_decoded_message;
	int message_decoded;
	double SNR;
	double signal_stregth_dbm;
	st_power_measurment power_measurment;
	int crc;
	int all_zeros;
	int mfsk_search_raw;  // MFSK anti-re-decode: base search position (symbol units, pre-nUnder adjustment)
	int ofdm_search_raw;  // OFDM anti-re-decode: base search position (symbol units, pre-nUnder adjustment)
	bool ofdm_batch_active;  // true when consecutive OFDM frames expected (narrow BATCH window)
	int frame_overflow_symbols;  // >0: MFSK frame extends beyond captured audio by this many symbols
	bool frame_data_missing;  // true: preamble found but data symbols are silence (incomplete capture)
	bool frame_skip_var_aborted;  // true: trial loop aborted on consecutive SKIP-VAR — caller should zero false preamble and advance cursor past noise region
	double coarse_metric;  // Schmidl-Cox correlation metric from coarse time_sync (diagnostic)
	double ofdm_drift_per_frame;  // IIR-filtered prediction error (interp samples) for BATCH verify
};


class cl_telecom_system
{
private:


public:
	cl_telecom_system();
	~cl_telecom_system();
	cl_data_container data_container;
	cl_psk psk;
	cl_mfsk mfsk;
	cl_mfsk ack_mfsk;  // Dedicated MFSK instance for ACK pattern (always initialized, all modes)
	cl_awgn awgn_channel;
	cl_error_rate error_rate;
	cl_ofdm ofdm;
	cl_error_rate passband_test_EsN0(float EsN0,int max_frame_no);
	cl_error_rate baseband_test_EsN0(float EsN0,int max_frame_no);
	cl_ldpc ldpc;
	double sampling_frequency;
	double carrier_frequency;
	double carrier_amplitude;
	int frequency_interpolation_rate;
	int time_sync_trials_max;
	int use_last_good_time_sync;
	int use_last_good_freq_offset;
	int mfsk_fixed_delay;  // >= 0: bypass time_sync with this delay (BER test); -1: use time_sync
	int ofdm_forced_delay; // >= 0: override time_sync result (BER test, keeps passband_to_baseband); -1: normal
	int test_puncture_nBits;  // > 0: zero out LLRs past this position (punctured LDPC BER test); 0: disabled

	// Last coarse frequency offset from OFDM preamble detection.
	// Persisted so ACK MFSK detectors use the same corrected carrier as
	// OFDM data demodulation. Without this, USB audio clock mismatch
	// (~24 Hz on RPi CM108) puts MFSK tones at FFT half-bin boundary.
	double last_coarse_freq_offset;

	// MFSK short control frames: punctured LDPC for ACK/control messages
	int ctrl_nBits;    // interleaved bits to transmit for ctrl frames (0 = no puncturing)
	int ctrl_nsymb;    // MFSK symbols for ctrl frames
	bool mfsk_ctrl_mode;  // true: TX/RX uses shorter ctrl frame parameters
	void set_mfsk_ctrl_mode(bool enable);
	int get_active_nsymb() const;  // ctrl_nsymb when mfsk_ctrl_mode, else Nsymb
	int get_active_nbits() const;  // ctrl_nBits when mfsk_ctrl_mode, else nBits

	// ACK pattern: short known-tone sequence for pattern-based ACK
	int ack_pattern_passband_samples;    // = ack_mfsk.ack_pattern_nsymb * Nofdm * freq_interp_rate
	int ack_snr_pattern_passband_samples;  // = (ack_pattern_nsymb + SNR_SUFFIX_LEN) * Nofdm * freq_interp_rate
	int ack_sack_pattern_passband_samples; // = ack_sack_pattern_nsymb() * Nofdm * freq_interp_rate (WB-only; 0 on NB)
	double ack_pattern_detection_threshold;  // metric threshold for detection
	int generate_ack_pattern_passband(double* out);  // TX: returns samples written
	int generate_ack_snr_pattern_passband(double* out, float snr);  // TX: ACK + SNR suffix, returns samples
	// TX: ACK base + 13-symbol ACK+SACK suffix carrying
	// [bsi:8 | bitmap:32 | crc12:12]. Returns samples written, or 0 if
	// unsupported (NB / M<16). Caller supplies the crc12.
	int generate_ack_sack_pattern_passband(double* out, uint8_t batch_seq_id, uint32_t bitmap, uint16_t crc12);
	double detect_ack_pattern_from_passband(double* data, int size, int* out_matched = nullptr, uint32_t* out_match_mask = nullptr);  // RX: returns metric
	float detect_ack_snr_from_passband(double* data, int size, int* out_matched, bool* out_snr_valid);  // RX: detect ACK + decode SNR
	// RX: detect ACK pattern and decode the 40-bit ACK+SACK suffix (WB M>=16
	// only). Runs detector + ofdm.decode_suffix_tones + unpack in one call —
	// also useful as a unit-test entry point. Returns true on clean decode
	// (bsi/bitmap/crc12 reflect transmitted values); false on no detection
	// or unsupported M. *out_matched (optional) gets the base-pattern match
	// count for diagnostics. Caller verifies crc12 by re-computing CRC12
	// over [bsi || bitmap] (mismatch → treat as no-ACK).
	bool decode_ack_sack_from_passband(double* data, int size,
	                                   uint8_t* out_bsi, uint32_t* out_bitmap,
	                                   uint16_t* out_crc12,
	                                   int* out_matched = nullptr);
	void ack_pattern_detection_test();  // SNR sweep + false alarm test

	// BREAK pattern: emergency "drop to ROBUST_0" signal (different tones from ACK)
	int generate_break_pattern_passband(double* out);  // TX: returns samples written
	double detect_break_pattern_from_passband(double* data, int size, int* out_matched = nullptr);  // RX: returns metric

	// HAIL pattern: "I am Mercury" beacon (different tones from ACK and BREAK)
	int generate_hail_pattern_passband(double* out);  // TX: returns samples written
	double detect_hail_pattern_from_passband(double* data, int size, int* out_matched = nullptr, int suffix_start = 0, int* out_suffix_matched = nullptr);  // RX: returns metric

	// Step 15: legacy MFSK SACK pattern (sack_pattern_passband_samples,
	// generate_sack_bitmap_pattern_passband, detect_sack_pattern_from_passband,
	// decode_sack_bitmap_ldpc, sack_pattern_detection_test) deleted —
	// OFDM SACK_RSP (arq_common.cc send_sack_v2_frame / decode_sack_v2_frame)
	// is now the only SACK transport.

	st_receive_stats receive_stats;

	int operation_mode;

	// Phase-2 validation flag (set via --skip-var-gate=on|off CLI). Default true
	// keeps HEAD behavior. When false, the noise_variance > 0.5 SKIP-VAR gate
	// in receive_byte() is bypassed so LDPC trials run regardless. See
	// IONOS_ERA_VALIDATION_PLAN.md §16.2 (b806b76 drift-catalog item).
	bool skip_var_gate_enabled;

	// Phase-2 validation flag (--rx-normalize=on|off). Default true = HEAD. When
	// false, the b806b76 RX passband auto-rescaling block is bypassed (impulse
	// noise blanking still runs). See PHASE2_FLAGS_DESIGN.md §2.8.
	bool rx_normalize_enabled;

	// Phase-2 validation flag (--csi-llr=on|off). Default true = HEAD. When
	// false, the b806b76 CSI-weighted LLR loop is skipped — LLRs from psk.demod
	// are passed straight to deinterleaver (uniform weighting). Tests whether
	// CSI weighting helps or hurts on flat channels. See PHASE2_FLAGS_DESIGN.md §2.6.
	bool csi_llr_enabled;

	// Phase-2 validation flag (--mean-h-gate=F). Default 0.30 = HEAD (b806b76).
	// Pre-IONOS was 0.50. Threshold below which frames are rejected as
	// bad-timing (pilots land on data positions). See PHASE2_FLAGS_DESIGN.md §2.1.
	double mean_h_gate_threshold;

	// Phase-2 validation flag (--energy-gate-floor=F). Default 1e-12 = HEAD
	// (b806b76 effectively disables absolute silence detection).
	// Pre-IONOS was 0.001. Used at 7 sites in receive_byte() as the absolute
	// energy floor for is_silence / data-energy gates.
	// See PHASE2_FLAGS_DESIGN.md §2.4.
	double energy_gate_floor;

	// Phase-2 validation flag (--ofdm-defer-overflow=on|off). Default true =
	// HEAD (7076a4b Fix A). When false, the frame-end-past-buffer detector at
	// receive_byte() is bypassed; overflow frames proceed through normal
	// processing (likely failing decode, then triggering ftr=shift fast-fwd).
	// See PHASE2_FLAGS_DESIGN.md §3.6.
	bool ofdm_defer_overflow_enabled;

	// §7.13.29 — caller-set hint that the next receive() is a SACK_RSP cross
	// check. When true, the OFDM-SYNC search uses stricter parameters:
	//   • preamble_detect_threshold bumped (0.15 → 0.65 WB, 0.30 → 0.75 NB) so
	//     Schmidl-Cox sub-peaks in OFDM body audio are rejected outright
	//     instead of consuming Moose trial budget.
	//   • time_sync_trials_max effective limit raised to 5 (vs default 2) so
	//     if a sub-peak slips past the threshold and triggers a Moose-reject,
	//     the search can still find the real preamble at a later position.
	// The flag is read inside cl_telecom_system::receive_byte()/process_decoder
	// and DOES NOT persist across calls — arq_commander sets it before the
	// cross-check this->receive() and clears it immediately after.
	// arq_commander may also set the existing ofdm_forced_delay to a stashed
	// metric=1.0 position from a previous INCOMPLETE cross-check; OFDM-SYNC
	// already bypasses search in that case (BER-test path), giving us a
	// deterministic retry without re-running the Schmidl-Cox lottery.
	bool sack_cross_check_mode = false;

	double output_power_Watt;

	void transmit_bit(int *data, double *out, int message_location);
	st_receive_stats receive_bit(double *data, int *out);

	void transmit_byte(int* data, int nBytes, double *out, int message_location);
	st_receive_stats receive_byte(double *data, int* out);

	// Lightweight signal measurement only (no decoding)
	double measure_signal_only(double *data);


	double M;
	double bandwidth;
	double LDPC_real_CR;
	double Tu;
	double Ts;
	double Tf;
	double rb;
	double rbc;
	double Shannon_limit;

	int bit_interleaver_block_size;
	int time_freq_interleaver_block_size;

	void calculate_parameters();

	void init();
	void deinit();
	cl_plot BER_plot, constellation_plot;

	void TX_RAND_process_main();
	void RX_RAND_process_main();
	void TX_TEST_process_main();
	void RX_TEST_process_main();
	void TX_SHM_process_main(cbuf_handle_t buffer);
	void RX_SHM_process_main(cbuf_handle_t buffer);
	void BER_PLOT_baseband_process_main();
	void BER_PLOT_passband_process_main();

	void load_configuration();
	void load_configuration(int configuration);
	int last_configuration;
	int current_configuration;
	void return_to_last_configuration();
	char get_configuration(double SNR);

	int get_frame_size_bytes();
	int get_frame_size_bits();

	struct st_channel_complex *pre_equalization_channel;
	void get_pre_equalization_channel();

	cl_configuration_telecom_system default_configurations_telecom_system;

	int outer_code;
	int outer_code_reserved_bits;

	int bit_energy_dispersal_seed;

	int narrowband_enabled;  // 0=wideband (Nc=50, BW=2344 Hz), 1=narrowband (Nc=10, BW=469 Hz)
	bool coarse_freq_sync_enabled;  // Coarse freq search (±30 Hz) for HF radio drift

	// TX gain table: per signal-type × NB/WB mode amplitude scalars
	double tx_gain[TX_SIG_COUNT][2][2];  // [signal_type][nb_mod][nb_fir]
	double get_tx_gain(tx_signal_type sig) const;
	void init_tx_gain_defaults();
	void print_tx_gain_table() const;
	// Calibration override (plan §7.13.21). Sets both [sig][nb_mode][0] and
	// [sig][nb_mode][1] (currently always paired — see get_tx_gain's [nb][nb]
	// diagonal). Logs the override so the calibration audit trail is in the
	// process log alongside [TX-GAIN] table dump.
	void set_tx_gain(tx_signal_type sig, int nb_mode, double value);

	st_reinit_subsystems reinit_subsystems;

	// 2D channel-state lookup helpers (Step 1 of design — see
	// mercury/fact-documents/channel-state-2d-lookup.md §3, §8).
	// These accessors expose pre-decode channel-state metrics for a future
	// (SNR_proxy, selectivity) → optimal-config lookup table. They are
	// read-only and have no call sites yet (Step 5 wires them in). The
	// underlying values are cached opportunistically wherever the existing
	// DSP already computes them — no new computation paths are added.
	//
	// Sentinels:
	//   correlator dB:  -99.0 → no ACK/HAIL detection attempted yet
	//   selectivity:    -1.0  → no preamble channel estimate yet
	//                          (selectivity is physically >= 0)
	double last_correlator_metric_db;     // cached by detect_ack/hail_pattern_from_passband
	double last_channel_selectivity;      // cached after preamble channel estimate

	// Returns last ACK/HAIL correlator metric, normalized to dB:
	//   metric_normalized = best_metric / ack_pattern_nsymb  ∈ [0, 1]
	//   metric_db         = 10 * log10(metric_normalized)
	// Higher = better channel. Sentinel -99.0 means no measurement yet.
	double get_correlator_snr_proxy() const;

	// Returns std(|H[k]|) / mean(|H[k]|) across OFDM DATA subcarriers from
	// the most recent preamble channel estimate. Flat AWGN → ~0;
	// selective multipath → > 0.3. Sentinel -1.0 means no estimate yet.
	double get_channel_selectivity() const;

};



#endif
