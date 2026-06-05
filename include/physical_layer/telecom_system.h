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
#include "common/os_interop.h"
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
	double mean_H;  // mean(|estimated_channel|) over MEASURED subcarriers for the last OFDM trial; -1 if not computed. Test-observability for the SKIP-H gate (write-once per receive, read by unit tests only). See fact-documents/ofdm-fine-timing-magnitude.md §3.5.
	int last_eff_preamble_nsymb;  // LEVER P: actual preamble-symbol count of the most recently extracted OFDM frame (FULL anchor vs MINI tail). Read by the ARQ position-advance (rx_frame = last_eff_preamble_nsymb + Nsymb). Defaults to preamble_nSymb when amortization is off.
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

	// Phase B Wave 1: CONNECT base + 13-symbol ctrl-suffix carrying
	//   [type:2 | payload:38 | crc12:12]. Uses a distinct Welch-Costas
	// base from ACK (g=3 vs g=5) so detectors can route by base correlation.
	// See fact-documents/phase-b-mfsk-connect-research.md §11.4.
	int connect_pattern_passband_samples;     // = connect_pattern_nsymb * Nofdm * freq_interp_rate
	int ctrl_suffix_pattern_passband_samples; // = (connect_pattern_nsymb + ack_sack_suffix_len) * Nofdm * freq_interp_rate
	// TX: emit CONNECT base + ctrl-suffix. Caller supplies (type, payload38,
	// crc12 over [type:2|payload:38] packed as 5 bytes). Returns samples
	// written, or 0 if unsupported (NB / M<16).
	int generate_ctrl_suffix_pattern_passband(double* out,
	                                          mfsk_ctrl_frame_type type,
	                                          uint64_t payload38,
	                                          uint16_t crc12);
	// RX: detect CONNECT base + decode the 52-bit ctrl-suffix. Returns
	// true on a clean decode (out_type/out_payload38/out_crc12 reflect the
	// transmitted values). Caller verifies crc12 by recomputing
	// CRC12 over [type:2|payload:38] packed as 5 bytes; on mismatch treat
	// as "no CONNECT arrived" (the caller's timeout/retransmit logic
	// handles it). *out_matched (optional) gets the base-pattern match
	// count for diagnostics.
	// crc12_fn/crc12_ctx (§19, INCREMENT 1): the production CRC-12 callback
	// (cl_arq_controller::CRC12_calc, init=0xFFF — NEVER inline, v1 bug #1).
	// Required ONLY when suffix_fec_mode selects the GF(16) FEC decode (the
	// soft_decode CRC accept gate needs it); the uncoded hard path ignores it
	// (it returns the unpacked crc12 for the caller to re-check). When the FEC
	// path runs, *out_crc12 is set to CRC12_calc([type|payload38]) so the
	// caller's outer CRC re-check passes by construction (consistent because
	// soft_decode only succeeds when the decoded CRC equalled that recompute).
	bool decode_ctrl_suffix_from_passband(double* data, int size,
	                                       mfsk_ctrl_frame_type* out_type,
	                                       uint64_t* out_payload38,
	                                       uint16_t* out_crc12,
	                                       int* out_matched = nullptr,
	                                       ctrl_crc12_fn crc12_fn = nullptr,
	                                       void* crc12_ctx = nullptr);

	// ---- Suffix FEC (connect-suffix-fec-research.md) — MEASURED PROTOTYPE ----
	// CRC-aided SOFT list decode of the 13-symbol ctrl-suffix. ZERO airtime
	// (Tier 1): no wire-format change, the suffix bytes are byte-identical to
	// baseline. These run the same base-pattern detector + mini-Moose as the
	// hard decode, then cl_ofdm::decode_suffix_candidates + the CRC-gated
	// best-first search (soft_list_decode_ctrl_suffix) to correct a few
	// wrong-argmax symbols. The caller passes the production CRC-12 (NEVER
	// inline; v1 bug #1). out_flips = #symbols that differ from the hard
	// argmax (0 ⇒ hard decode would also have passed).
	//
	// suffix_fec_mode gates production wiring (0 = baseline hard path only;
	// the *_soft entry points are always available for direct measurement).
	// Tier-2 sim spikes reserve distinct mode IDs so a later combined wire can
	// carry both: 2 = Golay(24,12) soft-ML (parallel spike), 3 = GF(16) RA
	// (tier2-suffix-fec-gf16-spike.md; measurement-only — the gf16ra codec has
	// no production caller, exercised via the §10 cliff-sweep harness).
	int  suffix_fec_mode;       // 0 = off (baseline). 1 = soft list decode. 2/3 = Tier-2 spikes.
	int  suffix_fec_K;          // top-K candidates per symbol (default 4).
	int  suffix_fec_max_trials; // CRC-trial cap (bounds runtime; default 4000).
	int  suffix_fec_max_flips;  // Hamming-ball radius (primary FAR lever; default 3).

	// §19 (INCREMENT 1): enable/disable the Tier-2 GF(16) RA FEC on the CONNECT
	// ctrl-suffix. on=true → gf16ra::configure(repfact)+init(), set
	// ack_mfsk.suffix_fec_coded=true, set suffix_fec_mode=3, and RE-DERIVE
	// ctrl_suffix_pattern_passband_samples (the coded length changed). MUST be
	// called AFTER load_configuration (which computes that member at the uncoded
	// length). repfact 3 = R=1/4 (N=52, the −14.03 reach, §12). Idempotent.
	// FORCE-on for this increment (no CAP negotiation yet). Returns the coded N.
	int  set_suffix_fec(bool on, int repfact = 3);

	// §20 (INCREMENT 2): set the CONNECT base-pattern noncoherent combining factor
	// R. R>1 → ack_mfsk.connect_preamble_reps=R (TX emits the base block R times,
	// RX sums per-symbol energy across the R aligned reps before the matched-count)
	// and RE-DERIVE ctrl_suffix_pattern_passband_samples (the on-wire base grew to
	// R×16). MUST be called AFTER load_configuration. R clamped to
	// [1, cl_mfsk::MAX_CONNECT_PREAMBLE_REPS]. R=1 = byte-identical (off). FORCE-on
	// for this increment (no CAP negotiation yet). Returns the on-wire base symbol
	// count (connect_base_total_nsymb()).
	int  set_connect_preamble_reps(int reps);

	bool decode_ctrl_suffix_from_passband_soft(double* data, int size,
	                                            mfsk_ctrl_frame_type expected_type,
	                                            ctrl_crc12_fn crc12_fn, void* crc12_ctx,
	                                            uint64_t* out_payload38,
	                                            int* out_matched = nullptr,
	                                            int* out_flips = nullptr);
	bool decode_ack_sack_from_passband_soft(double* data, int size,
	                                         ctrl_crc12_fn crc12_fn, void* crc12_ctx,
	                                         uint8_t* out_bsi, uint32_t* out_bitmap,
	                                         int* out_matched = nullptr,
	                                         int* out_flips = nullptr);

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

	// FIX-1 (2026-06-04): config-rate-aware SKIP-VAR noise-variance ceiling.
	// The historical flat 0.5 SKIP-VAR threshold (receive_byte trial loop) is
	// code-rate-INDEPENDENT, but the LDPC decode floor is strongly code-rate-
	// DEPENDENT. Measured (clean-AWGN PLOT_PASSBAND, gate OFF): rate-1/16 CONFIG_0
	// decodes up to nv≈1.48 (50% floor); the flat 0.5 sat far below that, rejecting
	// genuinely-decodable rate-1/16..8/16 (CONFIG_0-6 BPSK) frames before LDPC even
	// ran. Returns the per-config nv above which the frame may be fast-rejected
	// (the gate's legitimate compute-saving role). High-rate configs (CONFIG_7-16,
	// QPSK/8PSK/QAM) and all robust/NB ids keep 0.5 (decoder-bound — their decode
	// floor sits below 0.5 already, so the gate never bound them → no regression).
	// A hard cap (CONFIG_0's 1.60) bounds the loosest config so truly-hopeless
	// frames are still skipped. See skip_var_nv_ceiling() definition for the table
	// and the measured anchors it is derived from.
	double skip_var_nv_ceiling(int configuration);

	// Phase-2 validation flag (--rx-normalize=on|off). Default true = HEAD. When
	// false, the b806b76 RX passband auto-rescaling block is bypassed (impulse
	// noise blanking still runs). See PHASE2_FLAGS_DESIGN.md §2.8.
	bool rx_normalize_enabled;

	// Phase-2 validation flag (--csi-llr=on|off). Default true = HEAD. When
	// false, the b806b76 CSI-weighted LLR loop is skipped — LLRs from psk.demod
	// are passed straight to deinterleaver (uniform weighting). Tests whether
	// CSI weighting helps or hurts on flat channels. See PHASE2_FLAGS_DESIGN.md §2.6.
	bool csi_llr_enabled;

	// fix/cfg16-nv-restore validation hook (--fsel-test=on, default off): inject a
	// static 2-ray frequency-selective channel into the PLOT_PASSBAND BER path
	// (passband_test_EsN0) BEFORE the AWGN add. A flat AWGN loopback cannot
	// reproduce the LS-path nv collapse because the cross-pilot differential and
	// the pilot residual agree on a flat channel; a freq-selective channel forces
	// the estimator to interpolate/smooth, exposing the residual EVM the QAM
	// demapper actually suffers. Second ray: amp fsel_amp at delay fsel_delay
	// passband samples (within the cyclic prefix → no ISI, pure freq-selectivity).
	// Production paths never set this; default false.
	bool fsel_test_enabled;
	double fsel_amp;     // second-ray amplitude (linear), default 0.6
	int    fsel_delay;   // second-ray delay in passband samples, default 128

	// fix/cfg16-nv-restore: fast single-point BER override for PLOT_PASSBAND.
	// ber_single_esn0 <= -900 (default) = normal full sweep. Otherwise evaluate
	// one Es/N0 point with ber_frames_override frames and return.
	float ber_single_esn0;
	int   ber_frames_override;

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

	// ===== LEVER P: PREAMBLE AMORTIZATION =====
	// A batch is one gapless PTT waveform of N concatenated OFDM frames; today
	// every frame carries a full preamble_nSymb (=4) preamble, so 24 of 25
	// preambles in a batch are pure redundancy. The deterministic schedule below
	// emits a FULL preamble only on the batch anchor (frame 0) + on retx/after-
	// FAIL frames, and a 1-symbol MINI Schmidl-Cox resync on the rest. TX and RX
	// compute the schedule INDEPENDENTLY from (frame index, force-full) — no
	// per-frame wire flag. The pilot-derived channel estimate (33%-density
	// pilots in every DATA symbol) is unchanged, so SKIP-H / SKIP-VAR / partial-
	// SACK gates survive untouched; the preamble was only ever timing + CFO.
	// See fact-documents/data-flow-preamble-amortization.md.
	//
	// Pure schedule predicate (INC-0): how many preamble symbols frame
	// frame_idx_in_batch emits. force_full forces FULL for retx + after-FAIL
	// re-anchor frames. full_nsymb is the configured preamble_nSymb. Returns 1
	// (MINI) for non-anchor non-forced frames; full_nsymb otherwise. STATIC /
	// PURE so TX and RX get bit-identical results.
	static int preamble_sched_nsymb(int frame_idx_in_batch, bool force_full, int full_nsymb);

	// Master enable for preamble amortization. DEFAULT OFF pending INC-3 (the
	// batch-predict position-chain handoff for MINI frames that land beyond the
	// current buffer needs a defer-instead-of-full-search fix — see
	// fact-documents/data-flow-preamble-amortization.md §3/§6). When OFF the
	// schedule collapses to FULL on every frame: TX and RX are byte-identical to
	// the pre-LEVER-P baseline (verified: pinned CFG16 sim delivers 468.2 bps,
	// byte-correct, unchanged). When ON it activates the variable-preamble TX +
	// RX MINI handling. Toggle via the env knob below for A/B + INC-3 work.
	bool preamble_amortization_enabled = false;

	// TX per-frame override: number of preamble symbols THIS transmit_bit call
	// emits. -1 = use the full configured preamble_nSymb (legacy). send_batch
	// sets this per frame from the schedule before each transmit_byte.
	int tx_preamble_nsymb_override = -1;

	// RX per-frame override: number of preamble symbols the frame about to be
	// decoded is expected to carry (drives the data-symbol offset + frame
	// extraction size). -1 = use the full configured preamble_nSymb (legacy).
	// The ARQ receive driver sets this from the schedule (frame index counted
	// within the current batch) before each receive_byte.
	int rx_preamble_nsymb_override = -1;

	// Effective preamble length helpers: clamp an override into [1, full] or
	// fall back to the configured length when the override is -1.
	int tx_effective_preamble_nsymb() const;
	int rx_effective_preamble_nsymb() const;

	// Set by transmit_bit to the number of passband samples it actually wrote to
	// `out` for the most recent frame = Nofdm*(eff_preamble+active_nsymb)*interp.
	// send_batch reads this to pack the next frame contiguously (variable-length
	// frames in one gapless PTT waveform). Equals total_frame_size for FULL
	// frames; smaller for MINI frames.
	int tx_last_emitted_frame_samples = 0;

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
	// Timing-acquisition-under-SFO harness (F1 fix). Entry via -m PLOT_PASSBAND -s
	// <cfg> with MERCURY_SFO_BLOCK_TEST=1. Builds a long back-to-back OFDM block,
	// drifts it through cl_sim_sfo, and decodes each frame via REAL Schmidl-Cox +
	// Moose acquisition (ofdm_forced_delay=-1) so SFO actually affects timing —
	// the impairment the pinned BER/in-proc paths structurally cannot show. See
	// fact-documents/data-flow-sim2-time-domain-faithfulness.md §10.
	void sfo_block_test();
	// GENUINE single-grid big-block timing test (MERCURY_SFO_GRID=1). Builds ONE
	// 60-data-symbol OFDM grid (32QAM, 33% pilots Dx=1/Dy=3), applies the SFO as the
	// exact per-symbol subcarrier phase ramp (the omega+k*delta growth across the
	// block), and decodes with ONE channel estimate interpolated across all 60
	// symbols (interpolate_bilinear_matrix). Measures per-symbol uncoded SER head vs
	// tail. Optional CPE/PEG LS per-symbol de-rotation (MERCURY_SFO_GRID_TRACK=1) is
	// the STEP-2 anti-P tracker. Unlike sfo_block_test (K tiled 9-sym codeword-frames,
	// per-frame estimate), this is the literal "one estimate across 60 symbols" case.
	//
	// TEST 2 (pilot thinning, net-PHY recovery): MERCURY_SFO_GRID_THIN=1 rewrites the
	// lattice in-harness to a CONTINUAL+SCATTERED ~6% layout (cont columns on every
	// symbol anchor the per-symbol tracker fit + nv; a scattered diagonal feeds the
	// channel est). The thin path uses a flat-channel pilot-averaged estimator (the
	// per-cell-LS+DFT-smoother smears a sparse lattice). MERCURY_SFO_GRID_CODED=1 adds
	// a real rate-0.875 LDPC K-codeword block decode (with AWGN via MERCURY_SFO_GRID_
	// ESN0) to verify noise_variance_estimate does NOT collapse (E1/cfg16-nvfix). Knobs:
	// CONT_COLS, SCAT_DX, SCAT_DY. See fact-doc §13.
	void sfo_grid_test();

	// TEST 3 (sparse-capable 2D channel interpolator — the §13.5 production gap).
	// On a frequency-SELECTIVE channel (MERCURY_SFO_GRID_CHAN=1 det-floor / =2 two-ray)
	// the flat-ML H̄ shortcut FAILS (one global scalar cannot represent |H|+phase varying
	// across 50 subcarriers). This estimator interpolates a real per-subcarrier-per-symbol
	// H[n][j] from the 6% continual+scattered lattice by SEPARABLE 2D interpolation:
	// (a) raw LS H=Y/X at every pilot; (b) TIME interp/Wiener-smooth across the scatter
	// lattice's symbol spacing (dy) per carrier; (c) FREQUENCY interp across carriers
	// within each symbol; (d) optional DDCE refinement between scatter updates. nv is the
	// pilot residual against the interpolated H (preserves the TEST-2 nv that holds).
	// Knobs: MERCURY_SFO_GRID_WIENER (1=Wiener time-smoother, 0=linear),
	// MERCURY_SFO_GRID_DDCE (1=decision-directed refine). Driven from sfo_grid_test only.
	void grid_sparse2d_estimator(std::complex<double>* rx, int Ngrid, int Nc);

	// BIG-BLOCK HW DE-RISK (PHY-only, no ARQ) — emit/decode the validated big-block
	// (one 4-sym preamble + K=8 1600-bit LDPC codeword-frames under ONE acquisition,
	// ~7.2% freq-focused pilots, channel-adaptive flat-ML/sparse-2D, TRACK=0) over a
	// REAL passband round-trip via a WAV file (S16LE 48 kHz mono), so the sim PHY win
	// can be HW-validated on real Fe-Pi clocks (PLAY on RPi1 -> IONOS -> RECORD RPi2)
	// BEFORE the production+ARQ build. The grid-build + estimator/LDPC are COPIED from
	// sfo_grid_test (validated, §13-§14); only the passband bridge (symbol_mod/
	// baseband_to_passband and the inverse + acquisition) is new — the SAME OFDM modem
	// code transmit_byte/receive_byte use. See fact-documents/bigblock-hw-wav-derisk.md.
	// Entry: -m PLOT_PASSBAND -s 16 with env MERCURY_BIGBLOCK_TX_WAV=<path> (emit) or
	// MERCURY_BIGBLOCK_DECODE_WAV=<path> (decode). Returns 1 on full success.
	int bigblock_tx_to_wav(const char* wav_path);
	int bigblock_decode_from_wav(const char* wav_path);
	// Shared builder: rebuild ofdm at Nsymb=Ngrid with the thin freq-focused lattice
	// (cont_cols/scat_dx/scat_dy) + reseed the pilot DBPSK sequence, exactly as
	// sfo_grid_test does. Returns nData; out-params give Ngrid/log2M/nBits. Both the TX
	// and decode sides call this so the lattice/pilot sequence match bit-for-bit.
	int bigblock_rebuild_thin_grid(int& Ngrid_out, int& log2M_out, int& nBits_out);

	void load_configuration();
	void load_configuration(int configuration);
	int last_configuration;
	int current_configuration;
	void return_to_last_configuration();
	int get_configuration(double SNR);  // returns CONFIG_0..16 (never CONFIG_NONE)

	int get_frame_size_bytes();
	int get_frame_size_bits();

	struct st_channel_complex *pre_equalization_channel;
	void get_pre_equalization_channel();

	cl_configuration_telecom_system default_configurations_telecom_system;

	int outer_code;
	int outer_code_reserved_bits;

	int bit_energy_dispersal_seed;

	// Per-instance RNG (single-process-sim-refactor.md §10.1, Landmine 1). Each
	// cl_telecom_system owns an INDEPENDENT glibc-TYPE_3 stream so two modem
	// instances in one process (the 2-instance SIM_INPROC stepper) never
	// cross-contaminate pre-eq channel / pilot / dispersal sequence generation
	// (get_pre_equalization_channel runs a 1000-draw NO-RESEED loop that would
	// otherwise inherit the OTHER instance's residual stream state). Bound +
	// seeded in the ctor via os_rng_make. ts_srandom/ts_random route through it
	// when rng_own_ (always true post-ctor); a single instance reproduces a clean
	// run byte-for-byte because there is no other instance to inherit residue
	// from and the TYPE_3 walk is deterministic from the seed.
	int32_t           rng_state_[OS_RNG_STATE_WORDS];
	struct random_data_t rng_;
	bool              rng_own_;
	void          ts_srandom(unsigned int seed);  // routed __srandom
	long int      ts_random();                    // routed __random
	// Opt this instance into its own residue-free RNG stream (re-seeds rng_ with
	// `seed` and flips rng_own_ true). Called ONLY by the 2-instance SIM_INPROC
	// stepper. Production never calls it → rng_own_ stays false → byte-identical.
	void          enable_per_instance_rng(unsigned int seed);

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
