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
// LEVER C (feat/decode-marathon): forward-declare the big-block multi-core decode
// pool so the threading headers stay out of this widely-included header. The pool
// is heap-owned + lazily constructed ONLY when MERCURY_LDPC_MULTICORE is set; the
// default (serial) path never touches it (fact-documents/decode-marathon-C.md).
class cl_ldpc_decode_pool;
#include "physical_config.h"
#include "physical_defines.h"
#include "misc.h"
#include "common/ring_buffer_posix.h"
#include "common/os_interop.h"
#include <iomanip>
#include <vector>
#include <memory>


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

// Value-init every member (default member initializers) so a default-constructed
// st_receive_stats — the persistent cl_telecom_system::receive_stats AND every
// returned-by-value instance — starts fully zeroed. This closes the stale/
// uninitialized-scalar read (e.g. delay_of_last_decoded_message,
// freq_offset_of_last_decoded_message) that the receive path can consume on an
// early-return/first-frame path before any producer writes it (diagnosis §5/H3).
// UBSan cannot see this class (it's not a bool/enum invalid-value load), so it
// is a defensive root-cause init, not a band-aid. Purely additive: the normal
// receive path already overwrites these before use; this only defines the
// before-first-write state.
struct st_receive_stats{
	int iterations_done = 0;
	int delay = 0;
	int delay_of_last_decoded_message = 0;
	int time_peak_symb_location = 0;
	int time_peak_subsymb_location = 0;
	int sync_trials = 0;
	double phase_error_avg = 0.0;
	double freq_offset = 0.0;
	double freq_offset_of_last_decoded_message = 0.0;
	int message_decoded = 0;
	double SNR = 0.0;
	double signal_stregth_dbm = 0.0;
	st_power_measurment power_measurment = {};
	int crc = 0;
	int all_zeros = 0;
	int mfsk_search_raw = 0;  // MFSK anti-re-decode: base search position (symbol units, pre-nUnder adjustment)
	int ofdm_search_raw = 0;  // OFDM anti-re-decode: base search position (symbol units, pre-nUnder adjustment)
	bool ofdm_batch_active = false;  // true when consecutive OFDM frames expected (narrow BATCH window)
	int frame_overflow_symbols = 0;  // >0: MFSK frame extends beyond captured audio by this many symbols
	bool frame_data_missing = false;  // true: preamble found but data symbols are silence (incomplete capture)
	bool frame_skip_var_aborted = false;  // true: trial loop aborted on consecutive SKIP-VAR — caller should zero false preamble and advance cursor past noise region
	double coarse_metric = 0.0;  // Schmidl-Cox correlation metric from coarse time_sync (diagnostic)
	double ofdm_drift_per_frame = 0.0;  // IIR-filtered prediction error (interp samples) for BATCH verify
	double mean_H = -1.0;  // mean(|estimated_channel|) over MEASURED subcarriers for the last OFDM trial; -1 if not computed (default matches the per-receive reset at telecom_system.cc:1052). Test-observability for the SKIP-H gate (write-once per receive, read by unit tests only). See fact-documents/ofdm-fine-timing-magnitude.md §3.5.
	int last_eff_preamble_nsymb = 0;  // LEVER P: actual preamble-symbol count of the most recently extracted OFDM frame (FULL anchor vs MINI tail). Read by the ARQ position-advance (rx_frame = last_eff_preamble_nsymb + Nsymb). Defaults to preamble_nSymb when amortization is off.
	int coast_frames_since_anchor = 0;  // BATCH-GRID COAST (DATAFLOW_AUDIT_batch_coast.md): consecutive forward-DATA frames the RX has COASTED (grid advanced one frame period past a decode FAIL, batch kept alive) since the last CRC-GOOD anchor. Reset to 0 on any successful decode and at session reset. Bounds the coast to the declared batch B so a keydown-end cannot coast into silence forever.
};


// PRECOOK (Stage 2): a fully-built per-config geometry BUNDLE. Holds the expensive
// init() outputs for ONE FULL_CONFIG_LADDER config so a config switch becomes a
// copy_from swap (M3) instead of a deinit→init rebuild. The build path (STEP 2,
// cl_telecom_system::precook_config_bundles) reuses the proven init_monitor_decoders
// pattern: build a scratch cl_telecom_system per config, init()/load_configuration,
// then copy_from its geometry into the slot.
//
// NON-copyable / NON-movable: it embeds cl_ofdm / cl_ldpc / cl_psk by value, all of
// which =delete their copy-ctor/assign AND declare no move-ctor, so the whole struct
// is neither copyable nor movable. It is therefore ONLY ever heap-owned via
// std::unique_ptr (see cl_telecom_system::config_bundles) — a plain value vector
// would fail to compile (resize/push_back need a move). See
// _research/PRECOOK_IMPLEMENTATION_PLAN.md §1.1 and
// _research/_precook/BUNDLE_FIELD_CHECKLIST.md PART 4/5/6.
struct st_config_bundle
{
	int  configuration = CONFIG_NONE;  // which ladder entry this slot holds
	bool narrowband    = false;        // NB (Nc=10) vs WB (Nc=50) variant

	// ---- Fully-built per-config geometry (the expensive init() outputs) ----
	cl_ofdm ofdm;   // pilot layout, preamble, FFT twiddles, 4 FIR designs, mfsk_* mirror + corr template
	cl_ldpc ldpc;   // parity/generator matrix pointers + decode workspace for this rate
	cl_psk  psk;    // constellation (OFDM configs)
	cl_mfsk mfsk;   // alphabet/streams + tone tables (ROBUST configs)

	// ---- Cached per-config arrays (deep-copied here; freed by the dtor) ----
	// pre_equalization_channel: get_pre_equalization_channel() runs a 1000-draw
	// NO-RESEED RNG loop (LANDMINE-3) — it MUST be CACHED, never regenerated on a
	// swap. Sized data_container.Nc; NULL for MFSK configs.
	struct st_channel_complex* pre_equalization_channel = nullptr;
	// bit_energy_dispersal_sequence: the seed-derived descrambler draw (INV-6),
	// N_MAX ints (set_size zeroes the live array, so the swap must restore this).
	int*  bit_energy_dispersal_sequence = nullptr;

	// ---- data_container geometry SCALARS (C1/C2/acquisition read; the swap
	// publishes these under capture_prep_mutex in STEP 3) ----
	int Nofdm = 0, Nc = 0, M = 0, Nfft = 0, Ngi = 0, Nsymb = 0, nData = 0, nBits = 0,
	    preamble_nSymb = 0, interpolation_rate = 0, total_frame_size = 0,
	    baseband_data_fine_slice_size = 0;
	int   buffer_Nsymb = 0;   // the config's NATURAL window (INV-4; always <= the pinned max)

	// ---- telecom_system-level per-config SCALARS ----
	float  ldpc_rate = 0.0f;
	double bandwidth = 0.0, carrier_frequency = 0.0;
	int    bit_energy_dispersal_seed = 0;
	double M_telecom = 0.0;                          // cl_telecom_system::M (modulation order, double)
	int    time_sync_trials_max = 0;
	int    outer_code = 0, outer_code_reserved_bits = 0;
	// calculate_parameters()-derived per-config scalars (telecom_system.cc:3775-3802).
	double LDPC_real_CR = 0.0, Tu = 0.0, Ts = 0.0, Tf = 0.0, rb = 0.0, rbc = 0.0, Shannon_limit = 0.0;

	st_config_bundle() {}
	~st_config_bundle()
	{
		if(pre_equalization_channel != nullptr)      { delete[] pre_equalization_channel;      pre_equalization_channel = nullptr; }
		if(bit_energy_dispersal_sequence != nullptr) { delete[] bit_energy_dispersal_sequence; bit_energy_dispersal_sequence = nullptr; }
	}
	// Explicitly non-copyable / non-movable (the embedded geometry classes already
	// forbid it; stated here so any accidental value-copy is a clear diagnostic).
	st_config_bundle(const st_config_bundle&) = delete;
	st_config_bundle& operator=(const st_config_bundle&) = delete;
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

	// ---- Chase-combining (HARQ Type-I soft-LLR combining) ---------------------
	// ROOT of the "chase engaged N times, 0 rescues" capstone finding: Mercury had
	// NO soft-combining code at all -- each received frame's decoder-order LLR
	// vector (data_container.deinterleaved_data) is consumed ONCE by ldpc.decode()
	// (telecom_system.cc combine point) and then OVERWRITTEN by the next frame, so
	// the failed soft info is DISCARDED. A bit-identical (same-config) retransmit's
	// LLRs were never summed with the prior look. The retx DOES carry the identical
	// coded bits (systematic QC-LDPC encode is deterministic, ldpc.cc:100; the retx
	// payload is a verbatim copy of messages_tx[i].data), so summing the two LLR
	// vectors is VALID and yields ~+3 dB coding gain per combined copy. This buffer
	// + config-gated sum is that missing soft-combine core.  See
	// fact-documents/chase-combining-harq.md.
	//
	// chase_enabled defaults OFF in production: the SAFE auto-combine TRIGGER needs
	// the ARQ missing-slot / decoded batch_seq_id identity key (option (c),
	// data-flow-chase-buffer.md) so two UNRELATED failed frames are never paired.
	// That ARQ integration is the follow-up; the primitive + its deterministic
	// proof (--test-chase) land first. MERCURY_CHASE=1 arms the gated receive_byte
	// hook for A/B once the identity key is wired.
	bool chase_enabled;                  // gate (read once from MERCURY_CHASE; default OFF)
	std::vector<float> chase_llr_buffer; // buffered failed-look LLRs, decoder input order (len<=N_MAX)
	int  chase_buf_config;               // config the buffered vector was captured under (-1 = none)
	int  chase_buf_len;                  // valid length of the buffered vector (== ldpc.N at capture)
	bool chase_buf_occupied;             // a candidate is buffered
	long chase_captures;                 // diagnostic-only (INV-CHASE-5: must NOT feed the optimizer)
	long chase_combines;                 // diagnostic-only
	long chase_rescues = 0;              // diagnostic-only: combined decodes that CRC-passed and were adopted
	// Scratch for the receive_byte() rescue hook (§ chase-combining-harq.md): a COPY
	// of the live decoder-input LLRs (so the native single-look decode is untouched)
	// plus the combined decode's bit/byte output. Members (not stack) to avoid a
	// per-frame N_MAX alloc; sized lazily on first use.
	std::vector<float> chase_combine_scratch;
	std::vector<int>   chase_rescue_bits;
	std::vector<int>   chase_rescue_bytes;
	void chase_reset();                                        // void the buffer (config change / BREAK / reset)
	void chase_capture(const float* llr, int len, int config);// buffer a failed look's LLR vector
	// If chase_enabled AND a config-compatible candidate is buffered, add it
	// element-wise into live[] IN PLACE (post-add clamp +/-40) and return true so
	// the caller re-decodes the summed vector. Returns false and leaves live[]
	// UNTOUCHED when disabled / empty / config-or-length mismatch (INV-CHASE-2:
	// only ever combine two looks of the SAME codeword).
	bool chase_combine(float* live, int len, int config);

	// Last coarse frequency offset from OFDM preamble detection.
	// Persisted so ACK MFSK detectors use the same corrected carrier as
	// OFDM data demodulation. Without this, USB audio clock mismatch
	// (~24 Hz on RPi CM108) puts MFSK tones at FFT half-bin boundary.
	double last_coarse_freq_offset;

	// STALE-CFO SCOPED RESET (long-run-degradation.md §2.2): counts CONSECUTIVE
	// full OFDM-decode failures. The receive() last-trial fallback
	// (telecom_system.cc:2510) reuses freq_offset_of_last_decoded_message as the
	// demod mixer when every fresh sync fails; that stale value is written ONLY
	// on a successful OFDM decode (:3171) and was NEVER cleared on failure, so a
	// marginal frame that latches an edge-of-range CFO poisons every subsequent
	// acquisition (the link can never relock -> WB dies -> parks ROBUST until a
	// process restart). This counter SCOPES the cure: incremented on each failed
	// OFDM receive, reset to 0 on any OFDM success; when it crosses
	// STALE_CFO_RESET_FAILS it clears the stale CFO + last_coarse_freq_offset so
	// the NEXT acquisition re-measures from scratch. A SINGLE dropped frame does
	// NOT trip it, preserving the deliberate last-trial / MINI-preamble (LEVER P)
	// reuse and the cfg=6 throughput stickiness (arq_common.cc:3850-3854) on a
	// healthy link (where the counter stays at 0). Reset in init()/load_configuration.
	int consecutive_ofdm_decode_fails;

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
	// [bsi:8 | bitmap:30 | crc12:12]. Returns samples written, or 0 if
	// unsupported (NB / M<16). Caller supplies the crc12.
	int generate_ack_sack_pattern_passband(double* out, uint8_t batch_seq_id, uint32_t bitmap, uint16_t crc12);
	// use_fine (recovery-ack-fine, STAGE 1): when true, the underlying
	// detect_ack_pattern runs its always_fine sub-window timing pass even when
	// coarse matched<6 — the recovery control-ACK 16-sym block straddles the
	// detection window in time (whole-block energy collapse → matched 1-6/16),
	// and only the fine pass can re-align it. Default false → the verbatim
	// integer no-fine path → byte-identical. ONLY the ARQ recovery-poll caller
	// (RECEIVING_ACKS_CONTROL + MERCURY_RECOVERY_ACK_FINE) passes true; data-ACK
	// / BREAK / SACK / CONNECT keep false.
	double detect_ack_pattern_from_passband(double* data, int size, int* out_matched = nullptr, uint32_t* out_match_mask = nullptr, bool use_fine = false);  // RX: returns metric
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

	// Option B (data-flow-compact-confirm.md): compact coded reverse-confirm.
	// TX: ACK base (16 sym) + the K=5 GF(16)-RA compact codeword (N=10 sym)
	// carrying [bsi:8|crc12:12]. crc12 = production CRC12 over the single [bsi]
	// byte. Returns samples written, or 0 if unsupported (NB / M<16). CLEAN-batch
	// confirm only (all-ones bitmap is implicit in the confirm type).
	int generate_compact_confirm_passband(double* out, uint8_t bsi, uint16_t crc12);
	int generate_topgear_confirm_passband(double* out, uint8_t bsi, uint16_t crc12,
	                                      uint8_t report, uint16_t report_crc12);
	// RX: detect the ACK base, extract the per-tone ENERGY matrix for the first
	// N=10 suffix symbols (decode_suffix_energies), and soft-decode the compact
	// codeword (gf16ra::soft_decode_compact) with a CRC12 accept gate. Returns
	// true iff the codeword decodes AND its recomputed CRC12 over [bsi] matches.
	// *out_matched (optional) gets the base-pattern match count.
	bool decode_compact_confirm_from_passband(double* data, int size,
	                                          ctrl_crc12_fn crc12_fn, void* crc12_ctx,
	                                          uint8_t* out_bsi, int* out_matched = nullptr,
	                                          uint8_t* out_report = nullptr,
	                                          bool* out_report_valid = nullptr);
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

	// ---- In-band rate adaptation (Stage 3a) — CONFIG_TAG on the real passband ----
	// TX: emit the CONFIG_TAG MFSK ctrl-suffix burst as passband audio. MIRROR of
	// generate_ctrl_suffix_pattern_passband: CONNECT base pattern + a one-tone-per-
	// symbol suffix, but the suffix is the gf16ra::encode_config_tag codeword
	// (configure(2) = N=39 R=1/3) built by the ARQ layer. The burst is SELF-SIZED
	// (it computes its own sample count from connect_base_total_nsymb()+N — it does
	// NOT read ctrl_suffix_pattern_passband_samples, which is sized for the CONNECT
	// FEC mode, not the tag). `tones` is N=gf16ra::codeword_len() GF(16) tones.
	// Returns samples written, or 0 if unsupported (NB / M<16). Stage 3a is a self-
	// contained robust burst; the in-line append into send_batch is Stage 3b.
	int generate_config_tag_pattern_passband(double* out,
	                                          const int* tones, int n_suffix);

	// RX: detect the CONFIG_TAG burst on the real passband and extract the per-tone
	// ENERGY matrix + soft FWHT chips, then hand them to the ARQ wrap-decoder.
	// MIRROR of decode_ctrl_suffix_from_passband's front half: passband→baseband
	// decimate, detect_ack_pattern base correlator (THE real always-on presence
	// detector — gate on matched>=connect_match_threshold && metric>=
	// CTRL_DETECT_METRIC_MIN, design §3.1), control mini-Moose v2 CFO correction,
	// then decode_suffix_energies over N symbols. Writes the N*M energy matrix into
	// out_energies (caller-sized to >= N*M) and the 16 RM soft chips into
	// out_chips[16]; sets *out_n_syms = N and *out_matched. Returns true iff the
	// base correlator locked (a burst is present); false = no tag present (the
	// steady-state no-tag frame on a real noise floor). The CRC/FWHT/binding
	// acceptance is the ARQ layer's config_tag_wrap_decode, NOT here.
	bool decode_config_tag_from_passband(double* data, int size,
	                                      double* out_energies, double* out_chips,
	                                      int* out_n_syms, int* out_matched);

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

	// RECOVERY-ACK robustness (recovery-ack-robustness.md §4). Set the bare-ACK
	// base-block rep count for the recovery / control-ACK turnaround; recomputes
	// ack_pattern_passband_samples for R×16 symbols. reps=1 (default) → byte-
	// identical. Returns the on-wire base symbol count (ack_base_total_nsymb()).
	int  set_recovery_ack_reps(int reps);

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
	// NB M8 connect wrong-lock instrumentation (env MERCURY_WRONGLOCK_STATS;
	// see receive_byte). A wrong-lock = the M8 preamble detector fired (a lock
	// position was returned) but the extracted frame failed to decode (CRC
	// reject) — the garbage-decode signature of a wrong-position lock on the
	// NB connect (LDPC-fallback) path. Emit is env-gated: unset => untouched.
	long long mfsk8_acq_fired = 0;
	long long mfsk8_acq_wronglock = 0;

	// ACQ BAND-EXCLUSION (data-flow-linkphase-break-storm.md §5 P1). The
	// high-SNR break-storm's persistence engine is the no-exclusion Schmidl-Cox
	// sub-peak re-pick: after a [SUBPEAK-REJECT]/[XCORR-RESCUE-FAIL] the reject
	// loop restores orig_delay (telecom_system.cc, the fine window seeds off
	// pream_symb_loc = delay/sym, so the next trial re-locks the SAME
	// content-stable wrong point) with no record of the rejected offset, so a
	// momentary onset dip becomes a sustained absorbing lock on a BAND of delays
	// (field: ~141k-147k full-rate samples, ~21 distinct offsets). P1 records
	// each rejected delay as a small band CENTER and, when a subsequent
	// sub-peak reject lands inside an already-recorded band (a confirmed repeat
	// re-pick, the storm signature), forces a clean grid re-acquire past the band
	// instead of restoring the wrong point again. Ring of recent band centers;
	// reset at each fresh acquisition epoch and on any successful decode
	// (re-anchor). Env-gated DEFAULT-ON
	// (MERCURY_ACQ_BAND_EXCL, =0 disables): unset => the block is armed;
	// MERCURY_ACQ_BAND_EXCL=0 => the whole block is skipped => byte-identical to
	// base. It fires ONLY inside the sub-peak reject branch, so
	// a real preamble (mean|H|~1.0, never a sub-peak) can never trigger it — no
	// clean-path regression by construction. RESPONDER receive-loop local; no
	// COMMANDER RTO state touched.
	static const int ACQ_EXCL_RING = 8;   // recent rejected-band centers retained
	int acq_excl_center[ACQ_EXCL_RING] = {0};  // full-rate sample offsets
	int acq_excl_count = 0;               // number of valid centers (<= ring)
	int acq_excl_head = 0;                // next write slot (wrap-evict oldest)
	long long acq_band_excl_fires = 0;    // production counter: P1 re-acquire escapes

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

	// P1 ACQ BAND-EXCLUSION decision (data-flow-linkphase-break-storm.md §5 P1).
	// Extracted from receive_byte()'s sub-peak reject branch so the deterministic
	// self-test (test_acq_band_excl) drives the SAME predicate the production path
	// runs. Returns true iff `delay` falls within `radius` of an already-recorded
	// rejected-band center (a confirmed repeat re-pick) — the caller then forces a
	// clean grid re-acquire; on a miss it records `delay` as a new band center
	// (wrap-evicting the oldest) and returns false. radius<=0 derives ~2 OFDM
	// symbols from the loaded geometry. When hit, *matched_center receives the
	// band center that matched (for the [ACQ-EXCL] fire line).
	bool acq_band_excl_hit(int delay, int radius, int* matched_center);
	void acq_band_excl_begin_epoch();
	int test_acq_band_excl();

	// Default-off rejection of reverse-direction narrowband MFSK bursts that
	// Schmidl-Cox can admit as forward OFDM preambles. The discriminator is the
	// preamble baseband-decimated/passband mean-energy ratio: HW real OFDM is
	// 0.565-0.584 while the false-lock population is 0.904-0.907.
	static constexpr double SUBPEAK_PREAM_RATIO_REJECT = 0.74;
	static bool subpeak_metric_gate_enabled();
	bool subpeak_reject_out_of_band(double bb_pream, double pb_pream) const;
	bool subpeak_admission_reject(double bb_pream, double pb_pream) const;
	// Accept/reject gate for a decoded frame, extracted from receive_byte() so the
	// deterministic gate self-test exercises the SAME predicate (see telecom_system.cc).
	// Returns true when the frame must be REJECTED (routed to the FAIL branch + the
	// TIME_INTERP-seed rescue). The CRC16 branch requires crc==0 AND a CONVERGED LDPC
	// decode. crc_escape_defeat restores the legacy CRC-only CRC16 gate for the
	// fail-before arm of the self-test; the receive path always passes false.
	bool frame_decode_rejected(const st_receive_stats& rs, int outer_code_in, bool crc_escape_defeat=false) const;
	int test_subpeak_gate();

	// F1b Part B (true-boundary rescue selector). Re-estimates the OFDM channel at
	// a candidate full-rate delay and returns mean(|H|) over MEASURED pilots, with
	// NO frequency-sync re-run and NO receive_stats side effects — a pure selector
	// for the SUBPEAK-REJECT sample-anchored rescue. See telecom_system.cc impl and
	// fact-documents/data-flow-rx-ring-rearm.md.
	double ofdm_meanH_at_delay(double *data, int cand_delay, double eff_carrier_freq, int rx_eff_preamble, double *out_coherence=nullptr);

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

	// Structural guard for pilot-grid overrides. The OFDM grid fixes the coded
	// bit count at nData*log2(M); it must fit the active LDPC codeword before any
	// TX/RX buffer is allowed to consume the geometry. Static/pure so the focused
	// regression drives the exact production predicate.
	static bool pilot_geometry_fits_ldpc(int nData, int modulation_order, int ldpc_n);
	static int pilot_override_target_config();

	// Moose CFO sanity decision (STATIC / PURE so the production receive loop
	// and the --test-moose regression call the identical predicate — the
	// no-divergence guarantee, same pattern as preamble_sched_nsymb).
	//
	// The Moose half-symbol estimator (ofdm.cc:carrier_sampling_frequency_sync)
	// has a WB capture range of +-2 subcarriers (nIS=4 => +-93.75 Hz at
	// subcarrier_spacing 46.875), but the applied correction is CLAMPED to +-1
	// subcarrier (+-subcarrier_spacing). Any |estimate| in the half-correction
	// DEAD ZONE [subcarrier_spacing, 2*subcarrier_spacing] was previously NOT
	// rejected (old reject threshold was 2*subcarrier_spacing) yet only HALF
	// corrected by the clamp, committing a residual CFO up to ~subcarrier_spacing
	// (~47 Hz). That residual rotates the per-subcarrier channel estimate H
	// symbol-to-symbol; the cross-pilot noise estimator
	// (ofdm.cc:estimate_noise_from_pilot_pairs) reads the rotation as
	// catastrophic noise -> SKIP-VAR -> 3-consecutive abort -> FTR-FAIL, so the
	// link never establishes. Root cause: the reject threshold and the clamp
	// ceiling were 2:1 instead of equal.
	//
	// This predicate closes the dead zone: when |estimate| exceeds the clamp
	// ceiling (subcarrier_spacing) AND the search can still advance to another
	// timing candidate, REJECT (the caller advances the trial to hunt the true
	// peak) instead of committing a known-bad half-clamped residual. Legitimate
	// small residuals (|estimate| <= subcarrier_spacing, real crystal offsets
	// <~20 Hz) are clamped/accepted exactly as before.
	enum moose_decision_t { MOOSE_REJECT_ADVANCE = 0, MOOSE_CLAMP = 1 };
	static moose_decision_t moose_clamp_decision(double freq_offset_measured,
	                                              double subcarrier_spacing,
	                                              bool can_advance_trial,
	                                              double& corrected_out);

	// Master enable for preamble amortization. DEFAULT OFF pending INC-3 (the
	// batch-predict position-chain handoff for MINI frames that land beyond the
	// current buffer needs a defer-instead-of-full-search fix — see
	// fact-documents/data-flow-preamble-amortization.md §3/§6). When OFF the
	// schedule collapses to FULL on every frame: TX and RX are byte-identical to
	// the pre-LEVER-P baseline (verified: pinned CFG16 sim delivers 468.2 bps,
	// byte-correct, unchanged). When ON it activates the variable-preamble TX +
	// RX MINI handling. Toggle via the env knob below for A/B + INC-3 work.
	bool preamble_amortization_enabled = false;
	// NB robust-preamble capability negotiation (CAP_ROBUST_PREAMBLE_NB).
	// Session-scoped: true once BOTH peers advertised the capability in the
	// CONNECT handshake. The session layer (arq) owns the transitions:
	// set at TEST_CONNECTION / TEST_CONNECTION_ACK capability receipt, cleared
	// at session reset. While false, NB robust TX emits the LEGACY 8-symbol
	// preamble (the interop floor); while true, the sidelnikov set. RX runs
	// detect-both either way (see cl_ofdm::time_sync_mfsk_corr), so the flip
	// instant needs no cross-peer synchronization. Survives load_configuration
	// (config switches keep the session's negotiated preamble).
	bool robust_preamble_negotiated = false;
	// Apply a negotiation transition NOW (re-installs the active mfsk set and
	// the detector mirror tables when the current config is NB MFSK).
	void set_robust_preamble_negotiated(bool on);
	// (Re)build the MFSK preamble runtime consumed by the detector: the corr
	// template (dead state, kept for revert safety) + the cl_ofdm primary and
	// alternate tone-table mirrors. Called by load_configuration and by
	// set_robust_preamble_negotiated.
	void rebuild_mfsk_preamble_runtime();
	// CONTINUOUS-KEYDOWN: carry fine timing across the keydown instead of re-deriving per MINI frame
	bool keydown_track_timing_enabled = false;
	int keydown_last_delay = -1; // MINI=0: last decoded frame delay; its sub-symbol phase seeds the next tail prediction

	// TX per-frame override: number of preamble symbols THIS transmit_bit call
	// emits. -1 = use the full configured preamble_nSymb (legacy). send_batch
	// sets this per frame from the schedule before each transmit_byte.
	int tx_preamble_nsymb_override = -1;

	// RX per-frame override: number of preamble symbols the frame about to be
	// decoded is expected to carry (drives the data-symbol offset + frame
	// extraction size). -1 = use the full configured preamble_nSymb (legacy).
	// Known-layout block receivers set this from the emitted frame schedule
	// before each receive_byte.
	int rx_preamble_nsymb_override = -1;

	// Effective preamble length helpers: clamp an override into [0, full] or
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

	// CFG17 composition self-test result snapshot (--test-cfg17 / run_cfg17_selftest).
	// Populated at the end of sfo_grid_test()'s coded block (additive — does NOT alter
	// any printed output or production behavior). Lets the in-process composition test
	// read the decode outcome without parsing stdout. -1 = not-run / no coded block.
	int sfo_grid_last_cw_ok  = -1;   // codewords decoded clean this run
	int sfo_grid_last_cw_tot = -1;   // total codewords this run (Kcw)
	double sfo_grid_last_noise_variance = -1.0; // estimator output before demap backstops

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

	// ===== P1: BIG-BLOCK PHY IN THE LIVE PATH (gated; NO ARQ change) =====
	// The big-block PHY validated in the WAV harness (one 4-sym preamble + K=8
	// 1600-bit LDPC codeword-frames under ONE acquisition, frozen 12% lattice
	// cont2/dx3/dy4, channel-ADAPTIVE flat-ML/sparse-2D estimator + CSI-LLR) moved
	// into the production transmit_byte/receive_byte. It is the SAME DSP the WAV
	// harness exercises — the WAV methods are now thin WAV-I/O wrappers over the
	// shared in-memory PHY workers below; transmit_byte/receive_byte branch to the
	// live entry points (transmit_bigblock / receive_bigblock) when this flag is
	// set. The flag is a FRAMING-MODE bit on the CFG16 rung (same 32-QAM rate-0.875
	// modulation; only the one-acquisition K-codeword framing + 12% layout differ —
	// NOT a new config). DEFAULT OFF so stock CFG15/16 per-frame paths are
	// byte-identical to the pre-P1 baseline. See P1 plan / bigblock-hw-wav-derisk.md.
	bool bigblock_framing_enabled = false;

	// ===== HEAP-OVERRUN ROOT-CAUSE FIX (fact-doc §13) =====
	// The CFG16 big-block writes a K-codeword concatenated waveform (~45552 doubles)
	// that is MUCH larger than one stock OFDM frame (~15184 doubles). The transmit_byte
	// branch below must therefore fire ONLY when the CALLER actually handed a block-sized
	// `out` buffer — NOT for every stock per-frame / single-frame / CONTROL-frame TX that
	// happens at CONFIG_16 (those pass a frame-sized slot). bigblock_framing_enabled +
	// current_configuration==CONFIG_16 are necessary but NOT sufficient: the gearshift
	// (and ARQ control traffic) emit many frame-sized transmits at CFG16. The ONLY producer
	// that passes a correctly block-sized buffer is bigblock_send_one_block() (+ the two
	// loopback validators); they set bigblock_emit_as_block=true via the scope guard below
	// for the duration of their transmit_byte() call. Any other CFG16 transmit_byte keeps
	// the stock per-frame OFDM geometry. bigblock_emit_out_capacity is the block buffer's
	// real sample capacity; transmit_bigblock/bigblock_tx_passband REFUSE to write (no
	// overrun) if the block needs more than this. (Pre-fix: the branch fired on config
	// alone -> a declined/control/per-frame batch overran its frame-sized slot -> heap
	// metadata smash -> abort at the next delete[] / config-switch free.)
	bool bigblock_emit_as_block = false;
	int  bigblock_emit_out_capacity = 0;   // sample capacity of `out` when emit_as_block

	// RAII guard: arm the block-emit intent for exactly one transmit_byte() call. Only the
	// dedicated block driver constructs it; it auto-clears on scope exit (exception-safe).
	struct bigblock_emit_scope
	{
		cl_telecom_system* ts; bool prev_flag; int prev_cap;
		bigblock_emit_scope(cl_telecom_system* t, int out_capacity_samples)
			: ts(t), prev_flag(t->bigblock_emit_as_block), prev_cap(t->bigblock_emit_out_capacity)
		{ ts->bigblock_emit_as_block = true; ts->bigblock_emit_out_capacity = out_capacity_samples; }
		~bigblock_emit_scope()
		{ ts->bigblock_emit_as_block = prev_flag; ts->bigblock_emit_out_capacity = prev_cap; }
	};

	// Dedicated RX output buffer for the decoded K-codeword info bits. receive_bigblock
	// decodes K*ldpc.K (= 8*1400 = 11200) info bits — WAY past N_MAX(1600). It MUST NOT
	// be funneled through data_container.data_byte[N_MAX] (the stock per-frame output): the
	// 9600-int forward overrun smashed adjacent data_container chunks and aborted at the
	// next config-switch free. receive_bigblock sizes THIS to bigblock_codeword_count()*
	// ldpc.K and writes the decode here; the ARQ carve (bigblock_receive_carve) reads from
	// here. data_byte stays <= N_MAX so the stock-path N_MAX consumers need no re-audit.
	std::vector<int> bigblock_rx_infobits;

	// In-memory big-block PHY workers (the validated DSP, no WAV I/O). Both the WAV
	// harness and the live path call these so the framing/estimator/LDPC are
	// bit-identical across entry points.
	//   TX  worker: emit one 4-sym preamble + K codeword-frames into out_pb (raw
	//               passband doubles, NO lead/trail silence). cw_info_out[c] = the
	//               ldpc.K info bits of codeword c (so the live RX side can compare /
	//               the caller can map sub-units). Returns K (codewords emitted);
	//               *nSamples_out = passband samples written. payload_bits (length
	//               >= nBits) supplies the systematic info bits; pass nullptr to use
	//               the seeded-PRBS known payload (loopback validation, matches the
	//               WAV harness byte-for-byte).
	int bigblock_tx_passband(double* out_pb, int& nSamples_out,
	                         std::vector<std::vector<int>>& cw_info_out,
	                         const int* payload_bits = nullptr);
	//   RX  worker: acquire ONCE over [pb, pb+nSamples), then decode K codewords
	//               with the channel-adaptive estimator + CSI-LLR + frozen layout.
	//               out_infobits (length >= K*ldpc.K) receives the decoded info bits
	//               of every codeword (carved into K sub-units by the caller).
	//               cw_ok_out[c] = 1 if codeword c decoded clean (CRC/known-payload
	//               gate); else 0 — this K-bit vector IS the per-codeword SACK
	//               granularity P2 will use. *K_out = K. Returns #codewords that
	//               decoded clean. acq_metric_out (optional) gets the Schmidl-Cox
	//               acquisition metric. cw_info_ref (optional, non-null) supplies the
	//               KNOWN info bits per codeword for the loopback byte-correct gate;
	//               when null the CRC/all-decode path is used (P2).
	int bigblock_rx_passband(const double* pb, int nSamples,
	                         int* out_infobits, int& K_out,
	                         std::vector<int>& cw_ok_out,
	                         double* acq_metric_out = nullptr,
	                         const std::vector<std::vector<int>>* cw_info_ref = nullptr);

	// LEVER C (feat/decode-marathon): multi-core big-block codeword decode pool.
	// Heap-owned + lazily constructed ONLY when MERCURY_LDPC_MULTICORE>=2; nullptr
	// (and never instantiated) on the default serial path => byte-identical render.
	// bigblock_decode_codewords() is the single replacement for the serial codeword
	// loop in bigblock_rx_passband: it reads MERCURY_LDPC_MULTICORE, and on >=2 it
	// ensures the pool (cloned for the active config, clamped to cores-1) and
	// decodes across it; on 0/1/unset it runs the original serial loop bit-for-bit.
	// Either way it fills DISJOINT out_infobits[c*K..]/cw_ok_out[c] and returns the
	// cw_ok count, JOINING before return (slot-order serial layout preserved).
	cl_ldpc_decode_pool* ldpc_decode_pool = nullptr;
	int bigblock_decode_codewords(const float* clr, int Kcw,
	                              int* out_infobits, std::vector<int>& cw_ok_out,
	                              const std::vector<std::vector<int>>* cw_info_ref);

	// Big-block preamble matched-filter SNAP. The Schmidl-Cox autocorrelation metric is
	// flat across the whole 4-symbol preamble plateau, so its (energy-weighted) argmax is
	// noise-fragile and can flip to a spurious plateau lobe ~half a symbol off the true
	// preamble start (DIAG bigblock-livepath-awgn-cliff §X). The pilot-EVM fine search
	// cannot recover a >GI coarse error (and pilot-EVM is nearly blind to the sharp true
	// timing peak). This SNAP cross-correlates the captured baseband (DECIMATED rate,
	// Nofdm/sym) against the KNOWN reference preamble baseband over a small ±search_dec
	// window around the SC coarse pick and returns the decimated position MAXIMIZING the
	// normalized matched-filter magnitude — a SHARP single peak at the true preamble start
	// (CFO-robust over the ~1-symbol local window). Returns the refined decimated start, or
	// coarse_dec unchanged if no qualified peak. bb_dec/bb_dec_len: captured decimated bb.
	long bigblock_preamble_mf_snap(const std::complex<double>* bb_dec, int bb_dec_len,
	                               long coarse_dec, int pre_nSymb, int Nofdm, int Nc,
	                               int search_dec);

	// LIVE-PATH entry points (branched from transmit_byte/receive_byte when
	// bigblock_framing_enabled). transmit_bigblock emits the block into out (raw
	// passband, NO_FILTER_MESSAGE-style contiguous samples); receive_bigblock
	// acquires + decodes the K codewords from the captured passband buffer and
	// returns the per-codeword decode result in receive_stats + the decoded info
	// bits in out. P1 keeps the seeded-PRBS known payload so the loopback gate can
	// assert byte-correctness exactly as the WAV harness does; feeding real ARQ
	// bytes + the K-sub-unit ACK granularity is P2 (no ARQ change in P1).
	void transmit_bigblock(int* data, int nBytes, double* out);
	st_receive_stats receive_bigblock(double* data, int* out);
	// Restore the stock OFDM config after a big-block rebuild (full CONFIG_NONE ->
	// load_configuration reload — sets every pilot field cleanly; see the
	// HARNESS-HID-BUG note at the definition). Replaces the WAV harness's fragile
	// partial pilot-field hand-restore that faulted in the live path.
	void bigblock_restore_stock_config();
	// P2.2 (normalization-bypass fix): the EXACT stock receive_byte RX passband
	// normalization + impulse-noise blanking (telecom_system.cc:1081-1124), factored
	// so the big-block RX path (receive_bigblock) runs it on the captured passband
	// BEFORE the big-block estimator — the OFDM estimator/LLR assume a normalized
	// level, and bigblock_rx_passband skipped this, so the live AWGN validator FAILED
	// at 30 dB (BER 0.43). Called from BOTH receive_byte (extraction is byte-identical
	// to the inline block — no stock-path drift) and receive_bigblock. No-op for MFSK.
	void rx_passband_normalize_and_blank(double* pb, int pb_samples);
	// IN-BAND ADOPT CLEAN-LOCK GATE (data-flow-inband-adopt-metric-gate.md §2) — compute the
	// NORMALIZED Schmidl-Cox timing metric (|P|²/R² ∈ [0,1], SNR-invariant) over a captured
	// passband SNAPSHOT, at the geometry the in-band tag ANNOUNCES, WITHOUT mutating any RX
	// state (a LOCAL pad/baseband copy; never touches passband_delayed_data, the ring cursors,
	// or receive_stats). Used by the in-band tag-follow adopt to REJECT a contaminated /
	// overlapping re-air window (metric ~0.5) and only ADOPT a clean single-burst lock
	// (metric ~0.997) — the "prominent peak" discriminant the codebase already cites
	// (arq_common.cc:12586). `announced_cfg` is the config the tag announces (the metric is
	// computed at the CURRENT loaded OFDM geometry — the caller has not yet load_configuration'd
	// the new cfg, and the base-rung re-air geometry the gate must judge IS the current one;
	// announced_cfg is advisory/logging only). Returns the metric in [0,1], or -1.0 if it
	// cannot be computed (MFSK / no snapshot / not an OFDM config).
	double inband_snapshot_clean_lock_metric(const double* snapshot, int len, int announced_cfg);
	// #samples one big-block TX writes to `out` (= preamble + K*frame passband
	// samples at the frozen layout). The ARQ/capture sizing needs this in P2; for
	// P1 the loopback validator uses it to size buffers. Computed from the frozen
	// layout; valid once a CFG16 grid is loaded.
	int bigblock_tx_total_samples();

	// USE-AFTER-FREE / PARTIAL-BLOCK FIX (bigblock-whiten-align): the number of OFDM
	// symbols ONE big-block occupies on the wire (preamble + Ngrid data symbols). The
	// live RX must wait for THIS MANY symbols before snapshotting+decoding the block —
	// the stock per-frame arming (preamble_nSymb + Nsymb = ONE stock frame, ~13 sym)
	// snapshots after only the block's head is captured, so the later codewords (cw1..K-1)
	// read silence and the decode garbles them (cw0 clean, cw1..7 CRC-fail -> 0 delivered).
	// Geometry-only (rebuild thin grid then restore stock); returns 0 for MFSK / no grid.
	int bigblock_rx_block_nsymb();

	// SACK-GATE P1 (data-flow-bigblock-arq-unit.md R-B): the codeword count K the
	// big-block framing carries at the current CFG16 rung — the SAME geometry both
	// the TX (transmit_bigblock:7832 nBits/ldpc.N capped by MERCURY_BIGBLOCK_K) and
	// the RX (receive_bigblock:7890 nBits/ldpc.N) derive. The ARQ layer's shared
	// batch-size election (sack_negotiated_recompute_batch) calls this to PIN
	// data_batch_size == K at the big-block rung so CMD's clean-ACK target
	// all_ones=(1<<data_batch_size)-1 EQUALS the RSP's K-bit big-block bitmap
	// (closes bug #9: CMD batch=25 vs RSP K=8 -> 0x1FFFFFF != 0xFF -> no clean
	// credit). Geometry-only (no I/O); rebuilds the thin grid to read nBits/ldpc.N
	// then restores the stock CFG16 config (same rebuild+restore as
	// bigblock_tx_total_samples). Valid once a CFG16 grid is loaded; returns 0 for
	// MFSK or when the grid yields no codewords (caller must not pin on 0).
	int bigblock_codeword_count();

	// PHASE 1 (fact-doc §11.6): apply (self-inverse) the big-block payload energy
	// dispersal / whitening to a bit buffer. The real-bytes TX (transmit_bigblock)
	// whitens the payload before LDPC encode so a zero-padded short compressed frame
	// still modulates to a well-conditioned signal; the ARQ RX (bigblock_receive_carve)
	// calls this on the decoded info bits to recover the exact payload. Same fixed-seed
	// PRBS on both ends (no wire negotiation). XOR => calling it twice is a no-op.
	void bigblock_whiten_bits(int* bits, int nbits);

	// Big-block cross-call stash (P1 loopback validation + P2 handoff). The TX side
	// records the K known info-bit groups it emitted + the sample count; the RX side
	// records the per-codeword clean vector (the K-bit SACK granularity P2 consumes)
	// + how many decoded clean. In P1 the same-process loopback validator reads these
	// to assert 8/8 byte-correct; the production/ARQ path in P2 replaces the
	// known-payload compare with ARQ truth.
	std::vector<std::vector<int>> bigblock_last_tx_cw_info;
	int bigblock_last_tx_K = 0;
	int bigblock_last_tx_samples = 0;
	std::vector<int> bigblock_last_rx_cw_ok;
	int bigblock_last_rx_K = 0;
	int bigblock_last_rx_cw_ok_count = 0;
	// CHANNEL-ESTIMATION HEALTH stash (fix/bigblock-chanest): the mean |estimated_channel|
	// over the last big-block RX. Healthy ~0.24 (the big-block raw |H| scale on the clean
	// calibrated sim cell); collapses toward ~0 when an un-tracked CFO/SFO ramps a rotating
	// phasor across the 133-symbol block (the genuine-path defect). Set unconditionally in
	// bigblock_rx_passband so the genuine 2-instance test can assert on it directly (the
	// DIAG print is env-gated; this stash is always live).
	double bigblock_last_rx_meanh = -1.0;

	// ACQUISITION-WINDOW POSITION stash (fix/bigblock-chanest §19): the located head
	// preamble start (FULL-RATE samples) of the last big-block RX, and the captured-window
	// length (samples) the decode ran over. The ARQ window-position guard reads BOTH to
	// decide whether the FULL block (head + preamble + Ngrid) fit inside the captured
	// window or whether its tail was zero-padded (the block landed too late in the window /
	// its tail had not yet arrived in the ring at snapshot time). When the block would
	// overrun the captured samples, the guard DEFERS the carve one arming cycle instead of
	// decoding a truncated block (§19.3). bigblock_rx_passband resets head_delay to -1 at
	// entry (acq-fail leaves -1); receive_bigblock stamps capture_nsamples to the nSamples
	// it decoded. Diagnostic-stash convention (mirrors bigblock_last_rx_meanh); off-rung
	// they are never read -> production byte-identical.
	long bigblock_last_rx_head_delay_samples = -1;
	int  bigblock_last_rx_capture_nsamples   = 0;

	// CFG16 CARVE-GATE HARDENING (cfg16-controlack-hold, GAP3): a one-shot RX
	// intent override that SUPPRESSES the big-block route in receive_byte for the
	// NEXT call only, so the captured passband is decoded by the STOCK per-frame
	// path instead of receive_bigblock. The ARQ carve gate (arq_common.cc) sets
	// this when a CFG16 acquisition fails the cw0 wire-header CRC check — i.e. the
	// audio is NOT a real big-block (a single OFDM control frame, stale audio, or
	// noise mis-routed by the unconditional CFG16->receive_bigblock gate). It then
	// re-decodes via the stock path so control frames received at CFG16 (e.g. a
	// SET_CONFIG/ACK turnaround) are parsed normally rather than carved into a fake
	// K-codeword block (the GAP-3 red-herring "whitening misalignment" source).
	// Mirrors the TX-side bigblock_emit_as_block intent flag (the TX already
	// declines control via bigblock_send_one_block); this makes the RX symmetric.
	// Auto-cleared by the caller after the one stock re-decode. Default false ->
	// production big-block decode path is byte-identical when the flag is unused.
	bool bigblock_rx_force_stock = false;

	// P1 LIVE-PATH loopback validator (env MERCURY_BIGBLOCK_LIVE=1 under -m
	// PLOT_PASSBAND -s 16). Sets bigblock_framing_enabled, drives ONE block through
	// the production transmit_byte -> in-memory passband round-trip -> receive_byte,
	// and asserts K/K codewords decode byte-correct. Proves the PHY is in the live
	// path (not just the standalone WAV harness). MERCURY_BIGBLOCK_LIVE_ESN0 (<= -900
	// = clean) optionally adds AWGN.
	void bigblock_livepath_loopback();

	void load_configuration();
	void load_configuration(int configuration);
	// PRECOOK (Stage 1): allocate the persistent shared capture ring ONCE at the MAX geometry
	// across every FULL_CONFIG_LADDER config in BOTH bandwidths, then pin it
	// (data_container.precook_ring_pinned=true) so load_configuration never frees/reallocs it —
	// the switch becomes a scalar publish under a sub-µs leaf lock instead of a ~100-200 ms
	// deinit→init held under capture_prep_mutex (the audio-deaf window). Call ONCE at startup,
	// AFTER ARQ.init's first load_configuration and BEFORE the capture thread spawns
	// (main.cc, between ARQ.init and audioio_init_internal). No-op if MERCURY_PRECOOK_DEFEAT is
	// set (leaves the legacy deinit→init-under-lock path = the fail-before). Idempotent.
	void precook_pin_shared_ring();

	// ---- PRECOOK (Stage 2): pre-built per-config geometry bundles ----
	// config_bundles is indexed by FULL_CONFIG_LADDER order (20 slots: ROBUST_0/1/2
	// + CONFIG_0..16) for the ACTIVE bandwidth. Each slot heap-owns a fully-built
	// st_config_bundle (unique_ptr because st_config_bundle is non-copyable/
	// non-movable — see its declaration). active_bundle_idx tracks the last swapped
	// slot (-1 = none). STEP 2 only BUILDS + gates these; the load_configuration swap
	// that consumes them is STEP 3 (production path stays byte-identical here).
	// PRECOOK V2 (Step B) — DUAL bundle sets. One bandwidth per session was the attempt-3 killer
	// (an -R NB hail adopts WB mid-session → no WB bundle → legacy rebuild under the NB-sized pinned
	// ring → Nc=50 overruns the Nc=10 scratch → 0/3 climb). Both sets are built at startup
	// (reachable slots only per band) so bundle_index(cfg, narrowband) stops returning -1 at the
	// NB→WB adopt. Each is indexed by FULL_CONFIG_LADDER order; unreachable slots (NB CONFIG_15/16)
	// stay nullptr. bundle_set(narrowband) selects the right vector.
	std::vector<std::unique_ptr<st_config_bundle>> config_bundles_wb;
	std::vector<std::unique_ptr<st_config_bundle>> config_bundles_nb;
	bool precook_bundles_built = false;   // true once precook_config_bundles() built BOTH sets
	long precook_miss_count = 0;          // Step C: # of under-pin legacy fallbacks (live gate needs 0)
	int active_bundle_idx = -1;
	std::vector<std::unique_ptr<st_config_bundle>>& bundle_set(int narrowband)
		{ return (narrowband != NO) ? config_bundles_nb : config_bundles_wb; }
	const std::vector<std::unique_ptr<st_config_bundle>>& bundle_set(int narrowband) const
		{ return (narrowband != NO) ? config_bundles_nb : config_bundles_wb; }
	// Build the bundle set for the current narrowband_enabled bandwidth. Reuses the
	// init_monitor_decoders pattern (arq_common.cc:1849): for each ladder config,
	// construct a fresh scratch cl_telecom_system, load_configuration(cfg) (runs the
	// full init()), then copy_from its ofdm/ldpc/psk/mfsk + cache pre_equalization_
	// channel (deep copy — NEVER regenerate, LANDMINE-3) + bit_energy_dispersal_
	// sequence + the scalar block into config_bundles[ladder_index]. The scratch (and
	// its ring) is discarded per config. Idempotent (rebuilds the set). Does NOT wire
	// into the switch (STEP 3). See _research/PRECOOK_IMPLEMENTATION_PLAN.md §1.2.
	void precook_config_bundles();
	// Index into config_bundles for (configuration, narrowband). Returns the
	// FULL_CONFIG_LADDER index if a matching-bandwidth bundle is built at that slot,
	// else -1 (config not on the ladder, bundles not built, or bandwidth mismatch).
	int  bundle_index(int configuration, int narrowband) const;

	// ---- PRECOOK (Stage 2) STEP 3: the M3 config swap ----
	// Installs config_bundles[idx] as the live geometry WITHOUT a deinit()->init()
	// rebuild: copy_from the 4 PHY objects + pre_eq + descrambler + the telecom/
	// data_container scalar block (all OUTSIDE capture_prep_mutex — C1 reads none of
	// it), reproduce the telecom-level derived tail (interleaver block sizes, MFSK
	// ctrl-frame params, ack_mfsk + ack-pattern sample counts, receive_stats reset),
	// then publish the C1-visible ring geometry under a sub-µs LEAF lock
	// (publish_active_ring). `configuration` is the (already NB-clamped) target;
	// `idx` == bundle_index(configuration, narrowband_enabled) (caller-verified >=0).
	// The ONLY caller is load_configuration(int)'s STEP-3 fast-path. See
	// _research/PRECOOK_IMPLEMENTATION_PLAN.md §3.1 + PRECOOK_STAGE2_TURNKEY.md STEP 3.
	void load_configuration_swap(int configuration, int idx);

	// Grow the capture ring to >= min_nsymb symbols WITHOUT a config change (a same-config
	// re-load is skipped). Re-runs data_container.set_size with the current geometry + the
	// raised buffer_Nsymb_min. Used to seat the in-band down-ladder robust ring floor
	// (data-flow-inband-ondemote-zerobyte.md §7). Mutex-protected; idempotent.
	void force_resize_capture_ring(int min_nsymb);
	// Reset the capture ring to the CURRENT config's NATURAL buffer_Nsymb (un-seat any raised
	// buffer_Nsymb_min). Restores the legacy OFDM-acquisition geometry after a robust->OFDM
	// in-band adopt so the re-aired burst lands within the coarse-search bounds
	// (data-flow-robust-ofdm-adopt-flush.md §10). Mutex-protected.
	void force_set_capture_ring_natural();
	// Regenerate data_container.bit_energy_dispersal_sequence from bit_energy_dispersal_seed.
	// init() runs this after its set_size; the ring-resize helpers (force_resize_capture_ring /
	// force_set_capture_ring_natural) call set_size DIRECTLY (without init), and set_size
	// CDELETE+reallocates the sequence array (data_container.cc:243->145) as fresh ZEROED pages —
	// wiping the descrambler. Both helpers must regenerate it or the RX descrambles with all-zeros
	// (RX_msg = TX_msg XOR descrambler_seq -> constant CRC fail). data-flow-robust-ofdm-adopt-flush.md §17.
	void regenerate_bit_energy_dispersal_sequence();
	int last_configuration;
	int current_configuration;
	void return_to_last_configuration();
	int get_configuration(double SNR);  // returns CONFIG_0..16 (never CONFIG_NONE)

	int get_frame_size_bytes();
	int get_frame_size_bits();

	struct st_channel_complex *pre_equalization_channel;
	void get_pre_equalization_channel();

	// Turbo-EQ (RESEARCH_turbo-eq.md §4.2): persistent per-frame buffer that
	// receives the LDPC a-posteriori LLR (N coded bits) from ldpc.decode when the
	// iterative decision-directed CE loop is enabled (MERCURY_TURBO_ITERS>1). Empty
	// + unused when turbo is OFF (default) ⇒ zero allocation, byte-identical path.
	std::vector<double> turbo_app_llr;

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
