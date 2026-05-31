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

#ifndef ARQ_H_
#define ARQ_H_

#include "timer.h"
#include <unistd.h>
#include <cstdint>
#include "tcp_socket.h"
#include "fifo_buffer.h"
#include "physical_layer/telecom_system.h"
#include "datalink_config.h"
#include "datalink_defines.h"
#include "common/common_defines.h"
#include "audioio/audioio.h"
#include "compression/mercury_compress.h"
#include "datalink_layer/b2f_handler.h"
#include "datalink_layer/rate_optimizer.h"
#include "datalink_layer/channel_state_lookup.h"
#include "crypto/mercury_crypto.h"
#include <iomanip>
#include <thread>
#include <atomic>
#include <chrono>

union u_SNR {
  float f_SNR;
  char char4_SNR[4];
};

// Base-36 callsign packing: fits up to 6 chars (A-Z, 0-9) into 5 bytes.
// Used by START_CONNECTION to avoid callsign truncation on small frames.
// Format: [1-bit flags][3-bit length][6 chars x 6 bits] = 40 bits = 5 bytes.
// Bit 39: narrowband flag (0=wideband, 1=narrowband).
// Bits 38-36: length (0-6). Bits 35-0: 6 chars x 6 bits.
//
// SSID is NOT carried in the pack — it's sent separately in TEST_CONNECTION
// (data[6]) so that 6-char callsigns are not truncated.
#define CALLSIGN_PACK_SIZE  5
#define SSID_NONE           0xFF

// SSID helpers: parse, format, and get SSID from "CALLSIGN-SSID" strings.
// SSID mapping: 0-15 = numeric (AX.25), 16=L, 17=T, 18=R, 19=X (Winlink/VARA)

inline int callsign_get_ssid(const std::string& callsign)
{
	size_t hyp = callsign.rfind('-');
	if(hyp == std::string::npos || hyp == callsign.size() - 1 || hyp == 0)
		return SSID_NONE;
	std::string ssid_str = callsign.substr(hyp + 1);
	if(ssid_str.size() == 1)
	{
		char c = ssid_str[0];
		if(c >= '0' && c <= '9') return c - '0';
		if(c == 'L' || c == 'l') return 16;
		if(c == 'T' || c == 't') return 17;
		if(c == 'R' || c == 'r') return 18;
		if(c == 'X' || c == 'x') return 19;
		return SSID_NONE;
	}
	else if(ssid_str.size() == 2 && ssid_str[0] >= '0' && ssid_str[0] <= '1'
	        && ssid_str[1] >= '0' && ssid_str[1] <= '9')
	{
		return (ssid_str[0] - '0') * 10 + (ssid_str[1] - '0');
	}
	return SSID_NONE;
}

inline std::string callsign_strip_ssid(const std::string& callsign)
{
	int ssid = callsign_get_ssid(callsign);
	if(ssid == SSID_NONE) return callsign;
	size_t hyp = callsign.rfind('-');
	return callsign.substr(0, hyp);
}

inline std::string callsign_format_ssid(const std::string& base, int ssid)
{
	if(ssid == SSID_NONE || ssid < 0) return base;
	std::string result = base + "-";
	if(ssid <= 15)
	{
		if(ssid >= 10) { result += (char)('0' + ssid / 10); result += (char)('0' + ssid % 10); }
		else result += (char)('0' + ssid);
	}
	else if(ssid == 16) result += 'L';
	else if(ssid == 17) result += 'T';
	else if(ssid == 18) result += 'R';
	else if(ssid == 19) result += 'X';
	else { result += (char)('0' + ssid / 10); result += (char)('0' + ssid % 10); }
	return result;
}

inline void callsign_pack(const char* callsign, int len, char* out, int flags = 0)
{
	if(len > 6) len = 6;
	uint64_t packed = ((uint64_t)(len & 0x7)) << 36;
	if(flags & 0x01) packed |= ((uint64_t)1) << 39;  // narrowband flag
	for(int i = 0; i < 6; i++)
	{
		int val = 0;
		if(i < len)
		{
			char c = callsign[i];
			if(c >= 'A' && c <= 'Z') val = c - 'A';
			else if(c >= 'a' && c <= 'z') val = c - 'a';
			else if(c >= '0' && c <= '9') val = c - '0' + 26;
		}
		packed |= ((uint64_t)(val & 0x3F)) << (30 - i * 6);
	}
	out[0] = (char)((packed >> 32) & 0xFF);
	out[1] = (char)((packed >> 24) & 0xFF);
	out[2] = (char)((packed >> 16) & 0xFF);
	out[3] = (char)((packed >> 8) & 0xFF);
	out[4] = (char)(packed & 0xFF);
}

inline std::string callsign_unpack(const char* data, int* out_flags = nullptr)
{
	uint64_t packed = 0;
	packed |= ((uint64_t)(unsigned char)data[0]) << 32;
	packed |= ((uint64_t)(unsigned char)data[1]) << 24;
	packed |= ((uint64_t)(unsigned char)data[2]) << 16;
	packed |= ((uint64_t)(unsigned char)data[3]) << 8;
	packed |= ((uint64_t)(unsigned char)data[4]);
	int len = (int)((packed >> 36) & 0x7);  // 3 bits for length
	if(len > 6) len = 6;
	if(out_flags)
	{
		*out_flags = 0;
		if(packed & (((uint64_t)1) << 39)) *out_flags |= 0x01;  // narrowband
	}
	std::string result;
	for(int i = 0; i < len; i++)
	{
		int val = (int)((packed >> (30 - i * 6)) & 0x3F);
		if(val < 26) result += (char)('A' + val);
		else if(val < 36) result += (char)('0' + val - 26);
	}
	return result;
}


struct st_message
{
	int ack_timeout;
	int nResends;
	int length;
	char* data;
	char type;
	char id;
	char sequence_number;
	int status;
	cl_timer ack_timer;
	// SACK Design A Step 3 — batch_seq_id field (mod 256 counter). On TX,
	// carries the batch_seq_id this frame should advertise on the wire
	// (set by the new-data-batch builder or by the retransmit-queue
	// builder when re-sending a frame). On RX, populated from the wire
	// when sack_v2_enabled. Sentinel value -1 means "unset" (v1 frame
	// where the field is not on the wire). Pure scaffolding at Step 3:
	// no decision branches on this value yet.
	int batch_seq_id;
};

// SACK Design A Steps 1 + 2 — effective header length helpers.
// Returns the runtime DATA_LONG / DATA_SHORT header length on the wire,
// gated on the session's sack_v2_enabled. When v2 is OFF (default), these
// return the legacy macro values (4 / 5) — wire format identical to
// pre-Step-1. When v2 is ON (both peers advertised CAP_SACK_V2 in
// TEST_CONNECTION), the header grows by 1 byte to make room for the
// batch_seq_id field (offset 3 in the header).
// See SACK_DESIGN_A_PLAN.md §4.2.1, §6 reversibility analysis (these
// helpers are the irreversibility mitigation — the new wire bytes are
// ONLY emitted when sack_v2_enabled is true on both peers).
inline int effective_data_long_header_length(bool sack_v2)
{
	return sack_v2 ? DATA_LONG_HEADER_LENGTH_V2 : DATA_LONG_HEADER_LENGTH;
}
inline int effective_data_short_header_length(bool sack_v2)
{
	return sack_v2 ? DATA_SHORT_HEADER_LENGTH_V2 : DATA_SHORT_HEADER_LENGTH;
}

// SACK: Double-buffered crypto batch storage for partial batch handling.
// Responder stores frames from up to 2 crypto batches (the one being completed
// via retransmits and the new one arriving in the same radio batch).
#define MAX_SACK_BATCH_SIZE  32   // Max frames per crypto batch
#define MAX_SACK_FRAME_SIZE  256  // Max frame payload size
// §7.13.39 Fix 1 — must be at least 2*MAX_SACK_BATCH_SIZE so the
// "channel collapse" safety net (retransmit_count > 2*data_batch_size →
// trigger BREAK) actually has headroom to detect a runaway queue without
// silently dropping retx beforehand. The v2 mixed-batch path now requeues
// overflow rather than dropping, so the queue can legitimately exceed one
// batch worth of slots during loss episodes.
#define MAX_RETRANSMIT_HEADROOM (2 * MAX_SACK_BATCH_SIZE) // Was 8; see §7.13.39 Fix 1

// §7.13.39 Fix 2 — sequence_number is a uint8 on the wire (low 7 bits = slot,
// bit 7 = EOB). The collision check masks with 0x7F; batches larger than 128
// would alias slots. Belt-and-braces compile-time guard.
static_assert(MAX_SACK_BATCH_SIZE <= 128,
	"§7.13.39 Fix 2: sequence_number low 7 bits cap batch size at 128");

// SACK_FIX_PLAN §7 step 3 — geometry-derived CMD post-TX timeout.
// See SACK_FIX_PLAN.md §4.1 for derivation and §11.1 for calibration:
//   sack_arrival_ms = ptt_off_delay + RSP_DECODE_MARGIN_MS
//                   + sack_pattern_ms(batch) + ptt_on_delay
// SACK_ARRIVAL_MARGIN_MS covers scheduling jitter + LDPC decode tail
// past the geometric estimate. Calibrated from post-Plan-A traces:
// observed worst arrival 2102 ms vs geometric estimate 1768 ms; required
// margin 334 ms + 500 ms safety floor = 834 ms; rounded up to 1000 ms.
// The pre-fix two hardcoded 3000-ms adders are replaced by this single
// margin (see SACK_FIX_PLAN.md §2.7 / §11.1).
#define RSP_DECODE_MARGIN_MS   300  // one-frame RSP decode budget
#define SACK_ARRIVAL_MARGIN_MS 1000 // jitter + LDPC decode tail safety

struct st_crypto_batch_buffer {
	int batch_id;           // crypto batch counter mod 8, or -1 if empty
	int expected_frames;    // from crypto_batch_size or end-of-batch detection
	bool frame_received[MAX_SACK_BATCH_SIZE];
	unsigned char frame_data[MAX_SACK_BATCH_SIZE][MAX_SACK_FRAME_SIZE];
	int frame_lengths[MAX_SACK_BATCH_SIZE];
	int frame_types[MAX_SACK_BATCH_SIZE];  // DATA_LONG or DATA_SHORT
	int frames_received_count;

	void clear() {
		batch_id = -1;
		expected_frames = 0;
		frames_received_count = 0;
		for (int i = 0; i < MAX_SACK_BATCH_SIZE; i++) {
			frame_received[i] = false;
			frame_lengths[i] = 0;
			frame_types[i] = 0;
		}
	}
};

struct st_stats
{
	  int nSent_data;
	  int nAcked_data;
	  int nReceived_data;
	  int nLost_data;
	  int nReSent_data;
	  int nAcks_sent_data;
	  int nNAcked_data;
	  int nBatches_sent;      // total data batches transmitted
	  int nBatches_acked;     // data batches that received ACK (clean OR partial)
	  int nBatches_fully_acked; // CLEAN-BATCH VIABILITY (§9): batches confirmed
	                            // FULLY delivered (all-ones bitmap) — the only
	                            // signal that may drive up-promotion. A partial
	                            // SACK bumps nBatches_acked but NOT this. The
	                            // LADDER-UP success_rate_data is computed from
	                            // this counter so a partial-only run reads 0%, not
	                            // 100%. See gearshift-start-and-recovery.md §9.

	  int nSent_control;
	  int nAcked_control;
	  int nReceived_control;
	  int nLost_control;
	  int nReSent_control;
	  int nAcks_sent_control;
	  int nNAcked_control;

	  float success_rate_data;
};

struct st_measurements
{
	  double SNR_uplink;
	  double SNR_downlink;
	  double signal_stregth_dbm;
	  double frequency_offset;;
};


class cl_arq_controller
{

public:
	cl_arq_controller();
  ~cl_arq_controller();


  void set_nResends(int nResends);
  void set_ack_timeout_control(int ack_timeout_control);
  void set_ack_timeout_data(int ack_timeout_data);
  void set_receiving_timeout(int receiving_timeout);
  void set_link_timeout(int link_timeout);
  void set_nMessages(int nMessages);
  void set_max_buffer_length(int max_data_length, int max_message_length, int max_header_length);
  void set_ack_batch_size(int ack_batch_size);
  void set_data_batch_size(int data_batch_size);
  void set_control_batch_size(int control_batch_size);
  void set_role(int role);
  void calculate_receiving_timeout();
  void recalculate_ack_timeout_for_batch();
  // SACK-negotiation batch recompute, shared by the CMD (TEST_CONNECTION_ACK)
  // and RSP (TEST_CONNECTION) handlers so the two sides run IDENTICAL code and
  // cannot diverge on data_batch_size. Gates on current_configuration (the
  // live-PHY config — ROBUST_0 on a robust connect, an OFDM config otherwise);
  // robust ⇒ leave batch at the pinned 1, OFDM ⇒ scale to the 30s/radio_batch
  // floor. The set_data_batch_size() chokepoint backstops the invariant.
  // See data-flow-batch-size.md §5. `who` is "CMD"/"RSP" for the log line only.
  void sack_negotiated_recompute_batch(const char* who);
  void set_call_sign(std::string call_sign);

  int get_nOccupied_messages();
  int get_nFree_messages();
  int get_nTotal_messages();
  int get_nToSend_messages();
  int get_nPending_Ack_messages();
  int get_nReceived_messages();
  int get_nAcked_messages();

  void messages_control_backup();
  void messages_control_restore();

  int init(int tcp_base_port, int gear_shift_on, int initial_mode);

  uint8_t CRC8_calc(char* data_byte, int nItems);

  // CRC-12 over `nBytes` bytes, MSB-first, returning a 12-bit value in the
  // low 12 bits of the uint16_t. Polynomial = POLY_CRC12 = 0xF13
  // (CRC-12-CDMA2000 forward), init = 0xFFF, no final XOR. Used to protect
  // the 40-bit MFSK ACK+SACK payload (`bsi:8 | bitmap:32`) against false-
  // accept after pattern correlator lock. See
  // mercury/fact-documents/mfsk-robust-ack.md §3.2.
  uint16_t CRC12_calc(const char* data_byte, int nBytes);

	//! Updates timers values and check for timeouts.
	    /*!
	      \return None
	   */
  void update_status();
	//! removes any acked of failed messages.
	    /*!
	      \return None
	   */
  void cleanup();
  void finish_turbo_direction();

	//! registers the ack of a data message.
	    /*!
	      \param message_id is the id of the received message (its location in the buffer).
	      \return None
	   */
  void register_ack(int message_id);
  void pad_messages_batch_tx(int size);

  void process_main();

  void process_user_command(std::string command);
	//! Sends PPT on to the user.
	    /*!
	      \return None
	   */
  void ptt_on();
	//! Sends PPT off to the user.
	    /*!
	      \return None
	   */
  void ptt_off();

  void process_messages();


  void process_messages_commander();
  int add_message_control(char code);
  void process_messages_tx_control();
  int add_message_tx_data(char type, int length, char* data);
  void process_messages_tx_data();
	//! Sends a data or a control message to the other end (via ALSA driver).
	    /*!
	     * \param message the st_message structure to be sent.
	      \return None
	   */
  void send(st_message* message, int message_location);
  void send_batch();
  void send_ack_pattern();   // Level 3: TX short tone pattern instead of LDPC ACK
  void send_ack_pattern_with_snr(float snr);  // TX ACK + 4 MFSK symbols encoding SNR
  // Level 3: RX + detect ACK pattern, returns true if detected.
  //
  // defer_audio_advance (§7.13.29): when true, a positive detection does NOT
  // commit the usual frames_to_read=4 + search_raw resets. The audio in the
  // ring is left UNCHANGED so a follow-up decoder pass (e.g. the v2 SACK_RSP
  // cross-check in process_messages_rx_acks_data) sees pristine samples
  // instead of audio the capture thread already advanced past. The caller
  // MUST invoke commit_ack_pattern_consumed() once it has accepted the ACK
  // outcome, otherwise the next poll re-detects the same pattern.
  bool receive_ack_pattern(bool defer_audio_advance = false);

  // §7.13.29 — apply the ftr=4 + search_raw resets that
  // receive_ack_pattern(defer_audio_advance=true) skipped. Idempotent.
  void commit_ack_pattern_consumed();

  // §7.13.29 — null out the audio range in the ring buffer that the
  // MFSK ACK matched filter scanned (the tail used by
  // receive_ack_pattern). Called after a strict-threshold MFSK ACK is
  // accepted so any subsequent OFDM Schmidl-Cox scan this poll cycle
  // can't false-fire on the Welch-Costas pattern (which has periodic
  // structure that looks like an OFDM preamble). Locks
  // capture_prep_mutex while it zeros so capture-thread writes don't
  // race. Cheap: just memset on tail_samples doubles.
  void zero_mfsk_ack_audio_tail();
  // Step 15: legacy MFSK SACK pattern (send_sack_pattern / receive_sack_pattern)
  // has been deleted. OFDM SACK_RSP via send_sack_v2_frame is the only
  // partial-batch SACK transport now.
  //
  // SACK Design A Step 7 — OFDM SACK_RSP TX (RSP side). Builds the control
  // frame [batch_seq_id, bitmap_bytes, CRC8] and TX's it via send_batch() at
  // the data configuration. Returns the wall-clock TX duration in ms
  // (measured from the moment send_batch() is invoked until it returns;
  // this is the wire-occupancy figure compared against the legacy MFSK
  // SACK pattern's ~1168 ms — see SACK_DESIGN_A_PLAN.md §7.7).
  // Pre-conditions: caller has verified sack_v2_enabled && nframes > 0.
  // bitmap[i] = true iff frame i of the batch was RECEIVED.
  long long send_sack_v2_frame(const bool* bitmap, int nframes,
                               unsigned char batch_seq_id);
  // SACK Design A Step 8a — bsi-bump-and-transfer-prev helper, hoisted out of
  // send_sack_v2_frame() so the invariant fires regardless of which transport
  // (OFDM SACK_RSP or MFSK suffix ACK+SACK) carries the partial bitmap on the
  // wire. Per fact-documents/sack_partial_bsi_advance.md §6g (verified
  // 2026-05-21), the MFSK suffix branch at arq_responder.cc:1198-1204 used to
  // set used_mfsk_path=true and bypass send_sack_v2_frame() entirely, leaving
  // rsp_current_expected_batch_seq_id stuck on the partial's bsi. Callers must
  // invoke this BEFORE choosing a transport so the bump runs unconditionally
  // for any successful partial-SACK dispatch. Internally gated by
  // (sack_v2_enabled && rsp_current_expected_batch_seq_id >= 0); safe to call
  // unconditionally.
  void bump_bsi_and_transfer_prev();
  // SACK Design A Step 7 — OFDM SACK_RSP RX decode (CMD side). Called when
  // receive() landed a frame with messages_rx_buffer.type == SACK_RSP. The
  // function:
  //   - Validates CRC8. On mismatch: increments cmd_sack_v2_crc_fail_count,
  //     emits [CMD-SACK-V2-CRC-FAIL], and returns false (caller continues
  //     waiting / falls back to existing ACK-timeout path; nothing is
  //     fabricated, §9.4/A2 lesson).
  //   - On CRC pass: writes the bitmap (true = RECEIVED) into out_bitmap[0..nframes-1],
  //     out_batch_seq_id, increments cmd_sack_v2_rx_count, returns true.
  bool decode_sack_v2_frame(bool* out_bitmap, int nframes,
                            unsigned char* out_batch_seq_id);

  // RSP-side TX wrapper. Send the MFSK ACK+SACK pattern (16 base +
  // 13 suffix = 29 symbols on WB) carrying [bsi:8 | bitmap:32 | crc12:12].
  // WB-only — caller must check ack_sack_suffix_len() > 0 before
  // calling (or this returns 0 and the caller falls back to the
  // legacy MFSK ACK pattern for NB / unsupported configurations).
  // OFDM_ACK_CLEAN was removed 2026-05-24 — see mfsk-robust-ack.md.
  //
  // Returns wall-clock TX time in ms, or 0 if the feature is unavailable
  // (NB session, M < 16, or compile-time gate MFSK_ACK_SACK_ENABLED=0).
  // Computes CRC12 over [bsi || bitmap] internally and emits the 16-symbol
  // pattern + 13-symbol MFSK suffix carrying [bsi:8 | bitmap:32 | crc12:12].
  long long send_mfsk_ack_sack(unsigned char batch_seq_id, uint32_t bitmap);

  // Phase B Wave 2 v2 — PHY-level helpers for MFSK CONNECT.
  // These are called from inside the legacy state-machine dispatchers
  // (process_messages_tx_control / process_messages_acknowledging_control on
  // TX, process_messages_rx_data_control / process_messages_rx_acks_control
  // on RX) at the moment the legacy code would otherwise emit/decode an
  // LDPC frame. The legacy messages_control + messages_rx_buffer state flow
  // is unchanged — only the bits on the wire differ. See fact-documents/
  // phase-b-mfsk-connect-research.md §13 for the architectural pivot.
  //
  // All four return 0 / false when the codec is unavailable (NB session
  // with M<8 / connect_pattern_nsymb<=0 / passive_monitor). Callers MUST
  // fall back to the legacy LDPC path in that case.
  long long send_mfsk_start_conn_phy(const std::string& sender_call);
  long long send_mfsk_test_ack_phy(uint8_t echoed_cap, uint8_t own_cap,
                                    uint8_t ssid);
  bool receive_mfsk_start_conn_phy(char out_call[7], int* out_call_len,
                                    bool* out_nb_flag);
  bool receive_mfsk_test_ack_phy(uint8_t* out_echoed_cap,
                                  uint8_t* out_own_cap,
                                  uint8_t* out_ssid);

  // Phase B Wave 3 (§14) — TEST_CONNECTION (CMD→RSP) PHY swap.
  // Site E (TX): CMD encodes [snr_q:4 | local_cap:2 | ssid:8] via
  //   pack_test_conn_payload (snr_q computed from
  //   telecom_system->ack_mfsk.snr_to_tone(snr) at M=16, 4-bit).
  // Site F (RX): RSP decodes; caller reconstructs float SNR via
  //   tone_to_snr(snr_q) and synthesizes messages_rx_buffer to the
  //   legacy LDPC TEST_CONNECTION layout (data[0]=TEST_CONNECTION,
  //   data[1..4]=u_SNR.char4_SNR float bytes, data[5]=local_cap,
  //   data[6]=ssid, length=7). See fact-doc §14.
  long long send_mfsk_test_conn_phy(float snr, uint8_t local_cap,
                                     uint8_t ssid);
  bool receive_mfsk_test_conn_phy(uint8_t* out_snr_q,
                                   uint8_t* out_local_cap,
                                   uint8_t* out_ssid);

  void send_break_pattern(); // Emergency BREAK: TX "drop to ROBUST_0" tone pattern
  void send_hail_pattern();    // TX "I am Mercury" beacon
  bool receive_hail_pattern(); // RX + detect HAIL beacon, returns true if detected
  void process_messages_rx_acks_control();
  void process_messages_rx_acks_data();
  void process_control_commander();
  void process_buffer_data_commander();
  void finalize_block_commander();

  // SACK Design A Step 9 — Multi-axis policy framework, Axis 1 entry point.
  //
  // policy_evaluate_axis1() wraps the existing SUCCESS_BASED_LADDER block from
  // finalize_block_commander() (originally arq_commander.cc:3135-3239). It is
  // the named entry point for Axis 1 (modulation) per §4.3.2 of
  // SACK_DESIGN_A_PLAN.md. No behavior change vs the inline path — same
  // observable (`last_transmission_block_stats.success_rate_data`), same
  // action (`config_ladder_{up,down}()`), same hysteresis. New side-effects:
  //
  //   1. `[POLICY-MOVE] axis=1 from=Y to=Z reason=R` log line on every
  //      modulation move (up / down). §4.3.4 invariant #5: "No move is silent."
  //   2. `policy_axis1_supremacy_on_move()` hook fires on every move.
  //      §4.3.4 invariant #6: Axis-1 moves override any in-flight Axes 2/3
  //      decision. At Step 9 the hook is a logging stub (Axes 2/3 do not
  //      exist yet); Steps 10/11 plug in the actual batch_size / sack_mode
  //      reset. The hook is named here so the wire surface for Axes 2/3 is
  //      already in place when those steps land.
  //
  // Gated on `sack_v2_enabled` at the call site in finalize_block_commander().
  // v1 sessions take the original inline SUCCESS_BASED_LADDER path verbatim —
  // no [POLICY-MOVE] emitted, no supremacy hook called, byte-identical
  // log surface for v1<->v1 sessions (proven by WAV harness sha256).
  void policy_evaluate_axis1();

  // SACK Design A Step 9 — Axis 1 supremacy hook (§4.3.4 invariant #6).
  // Called from policy_evaluate_axis1() on every modulation move (Step 9)
  // AND from every BREAK initiation site (Step 12) — BREAK is a more drastic
  // Axis-1 move, so Axes 2/3 must be reset just as aggressively. Step 10
  // filled in the Axis-2 ring + cooldown reset; Step 11 added the Axis-3
  // ring + cooldown + mode-to-PROBE transition; Step 12 adds the
  // batch_size_proven_ceiling reset (§4.3.4 invariant #7) and the BREAK
  // call-sites. Gated on `sack_v2_enabled` at the call site — v1 sessions
  // never enter.
  void policy_axis1_supremacy_on_move(int from_cfg, int to_cfg, const char* reason);

  // SACK Design A Step 9 — synthetic Axis 1 fire (test-only).
  // CLI: --test-policy-axis1-fire=up|down. Primes the LADDER state with a
  // synthetic observable (success_rate=100% for up, 0% for down) and the
  // hysteresis counters at the move threshold, then calls
  // policy_evaluate_axis1() once. Demonstrates that the wrapper is wired
  // and that [POLICY-MOVE] + [POLICY-SUPREMACY] fire on a real move.
  // direction: 1=up, 2=down. No-op for any other value.
  // Default builds never call this; production paths are unaffected.
  void test_fire_policy_axis1(int direction);

  // Option B (data-anchored promotion, 2026-05-29): apply the
  // last_data_viable_config recovery floor to a raw BREAK target. Returns the
  // raw target unchanged under the panic-jump (breaks_since_last_data_success
  // >= 2); otherwise never returns a config below last_data_viable_config (by
  // ladder index). Used by both BREAK recovery sites (arq_commander.cc:81 and
  // the retries-exhausted path) and the synthetic-fire test. See §6/§7.
  int break_target_with_anchor(int raw_target) const;

  // Option B synthetic-fire test (CLI --test-data-anchored-promote). Drives the
  // real BREAK-floor helper and the real policy_evaluate_axis1() up-shifter with
  // last_data_viable_config primed, asserting the link parks at the anchor rung
  // instead of climbing/falling past it. Returns 0 on pass, 1 on fail. Default
  // builds never call this. See fact-documents/gearshift-start-and-recovery.md §6.4.
  int test_data_anchored_promote();

  // Phantom-ACK content gate (2026-05-29). PURE policy predicate for the
  // clean-batch DATA-ACK bare-pattern acceptance at arq_commander.cc:2809.
  // receive_ack_pattern() returns a BARE bool on pattern-match-only (matched
  // >= ack_match_threshold && metric >= ack_metric_threshold; arq_common.cc:5556)
  // with NO CRC and NO content check, so structured noise / an rx-tail self-match
  // can fake a data ACK. The clean discriminator is CONTENT: a real WB data ACK
  // carries a CRC12-valid MFSK suffix. This predicate gates the bare-pattern arm:
  //   suffix_capable  = ack_sack_suffix_len() > 0 (WB; M>=16 carries a CRC suffix).
  //   crc_suffix_valid= a CRC12-valid, in-window, clean-batch (all-ones) suffix
  //                     was decoded from the current passband tail.
  // On CRC-capable (WB) sessions a bare match is accepted ONLY with a CRC-valid
  // suffix (rejects the phantom; the timeout-retransmit path is the safe fallback).
  // On NB / suffix-incapable sessions there is no suffix to validate (the RSP
  // sends a bare ACK pattern — arq_responder.cc:1679-1683), so the bare pattern
  // stays the acceptor. Control-ACK detection (arq_commander.cc:1702) and the
  // emergency-BREAK poll (arq_commander.cc:92) do NOT use this gate — control
  // frames carry no suffix and must keep bare-pattern behavior unchanged.
  // See fact-documents/gearshift-start-and-recovery.md §2 Bug 3 + §8.
  bool data_ack_bare_pattern_acceptable(bool suffix_capable,
                                        bool crc_suffix_valid) const
  { return suffix_capable ? crc_suffix_valid : true; }

  // Phantom-ACK content gate — DSP half. Peek the current passband tail for a
  // CRC12-valid, in-window, clean-batch (all-ones bitmap) MFSK ACK+SACK suffix.
  // Read-only (no frames_to_read mutation — mirrors the §7.13.30 no-side-effect
  // peek at arq_commander.cc:2366-2375). Returns false on NB / suffix-incapable
  // (ack_sack_suffix_len()==0), no decode, CRC mismatch, out-of-window bsi, or a
  // non-clean (partial) bitmap. Used by the bare-pattern data-ACK arm only.
  bool cmd_clean_data_ack_crc_valid();

  // Phantom-ACK content-gate synthetic-fire test (CLI --test-phantom-ack-gate).
  // Drives the PURE acceptance policy (data_ack_bare_pattern_acceptable) across
  // the WB/NB x CRC-valid/CRC-absent matrix AND asserts the cross-layer invariant
  // that a phantom (bare match, NO CRC suffix, WB) leaves data_ack_received NO,
  // does NOT raise last_data_viable_config, does NOT reset the BREAK panic counter
  // (breaks_since_last_data_success), and that BREAK can therefore still reach
  // ROBUST_0. Returns 0 on pass, 1 on fail. Default builds never call this.
  // See fact-documents/gearshift-start-and-recovery.md §8.
  int test_phantom_ack_gate();

  // CLEAN-BATCH VIABILITY (§9, 2026-05-29) — PURE policy predicate. A batch may
  // drive the four gearshift promotion consumers (anchor-raise, panic reset,
  // break_drop_step reset, FRAME-UP) ONLY if it was confirmed FULLY delivered
  // (all-ones bitmap). A PARTIAL SACK keeps the link alive (retransmit of the
  // missing frames is UNCHANGED) but must NOT promote the rung — otherwise a
  // single CRC-valid partial at a marginal config pins the BREAK floor there and
  // the link oscillates at the deep-SNR cliff (CONFIG_0 ↔ ROBUST_0). This is the
  // predicate the consumers gate on and the unit test drives. See §9.
  bool promotion_allowed_on_batch(bool batch_fully_acked) const
  { return batch_fully_acked; }

  // climb-engine Bug 1 (gearshift-climb-engine.md §4) — split SACK dedupe by
  // event class. The CMD MFSK ACK+SACK decode used a SINGLE tracker
  // (cmd_last_applied_sack_bsi): a PARTIAL SACK for bsi=B set it, then the later
  // all-ones CLEAN confirmation for the SAME bsi=B (emitted by the RSP
  // prev-delivered path after a retransmit completes the batch) was dropped as a
  // "duplicate" before it could reach the clean funnel — so a batch that DID
  // fully deliver (via retransmit) was never credited, last_batch_fully_acked
  // stayed false, and the rung never promoted. This PURE predicate is the decode
  // gate: a CLEAN (all-ones) confirmation supersedes the partial and is deduped
  // ONLY against cmd_last_applied_clean_bsi; a PARTIAL is deduped against
  // cmd_last_applied_sack_bsi as before. A repeated clean for the same bsi is
  // still rejected (no double-count of nBatches_fully_acked). It does NOT relax
  // the clean requirement — all-ones means "every frame delivered", which IS the
  // definition of clean; a batch that never completes never emits all-ones.
  // last_applied_clean_bsi / last_applied_sack_bsi are passed in (the test drives
  // them; production reads the members). Returns true iff the frame should be
  // accepted (not deduped).
  static bool sack_clean_confirmation_accepted(int rx_bsi, bool is_all_ones,
                                               int last_applied_clean_bsi,
                                               int last_applied_sack_bsi)
  {
    if(is_all_ones)
      return rx_bsi != last_applied_clean_bsi;   // clean: dedupe vs clean tracker
    return rx_bsi != last_applied_sack_bsi;      // partial: dedupe vs partial tracker
  }
  // CLEAN-confirmation dedupe tracker (climb Bug 1). Ctor-init -1. Distinct from
  // cmd_last_applied_sack_bsi (the partial tracker) so a clean confirmation for a
  // bsi whose partial was already applied is NOT dropped. Set when an all-ones
  // CLEAN MFSK ACK+SACK is accepted at arq_commander.cc (the :2516 funnel).
  int cmd_last_applied_clean_bsi;

  // CLEAN-BATCH VIABILITY synthetic-fire test (CLI --test-clean-batch-viability).
  // Drives the PURE promotion predicate AND the real gearshift consumer logic:
  // asserts a PARTIAL batch does NOT raise last_data_viable_config, does NOT reset
  // breaks_since_last_data_success, does NOT advance the FRAME-UP counter, and
  // reads 0% up-promotion success (nBatches_fully_acked/nBatches_sent); a CLEAN
  // batch does all of those; and that after a partial-only run BREAK can still
  // reach ROBUST_0 (panic latches). Returns 0 on pass, 1 on fail. Default builds
  // never call this. See gearshift-start-and-recovery.md §9.8.
  int test_clean_batch_viability();

  // climb-engine integrated 3-bug regression (CLI --test-climb-engine).
  // (a) Bug 1: sack_clean_confirmation_accepted() split-dedupe — an all-ones
  //     CLEAN confirmation for a bsi whose PARTIAL was already applied is
  //     ACCEPTED (pre-fix: dropped as duplicate); a repeated clean is rejected.
  // (b) Bug 2/3: the REAL policy_evaluate_axis2() + the batch-floor predicate
  //     keep data_batch_size==1 at robust configs across 6 good batches
  //     (pre-fix: Axis-2 steps 1->6); OFDM grows.
  // (c) Bug 3 (the assertion the singles lacked): an end-to-end multi-rung
  //     climb — the REAL anchor-advance gate + the REAL +1 clamp + the REAL
  //     Axis-2 — promotes ROBUST_0 -> ROBUST_1 -> ROBUST_2 -> CONFIG_0, NOT
  //     stuck one rung up. Pre-fix (Axis-2 grows robust batch -> no clean
  //     credit) the anchor freezes at ROBUST_0 and the climb stalls.
  // Returns 0 on pass, 1 on fail. See gearshift-climb-engine.md §7.
  int test_climb_engine();

  // SACK Design A Step 10 — Axis 2 controller (adaptive batch size).
  //
  // policy_evaluate_axis2() implements the per-batch §4.3.2 controller:
  //   - observable: mean(recent_partial_rate) over rolling 5-batch ring;
  //     partial_rate = frames_lost / batch_size (from SACK_RSP bitmap on
  //     SACK-partial, 0.0 on clean full-ACK).
  //   - cadence: per-batch (called from process_messages_rx_acks_data after
  //     SACK_RSP receipt + after clean full-batch ACK).
  //   - action: step `data_batch_size` ±5, clamped to [AXIS2_BATCH_FLOOR,
  //     AXIS2_BATCH_CEIL].
  //   - hysteresis: up after >=8 consecutive good batches (partial<0.05) AND
  //     mean<0.05; down after >=3 consecutive bad batches (partial>0.20) AND
  //     mean>0.20; reset counters + ring on any move.
  //   - axis-1 supremacy: skip when axis2_cooldown_batches > 0 (set by
  //     policy_axis1_supremacy_on_move()). Cooldown decremented per call.
  //
  // On move: CMD calls add_message_control(SET_LINK_PARAMS) carrying the
  // new batch size; RSP applies via set_data_batch_size() on receipt
  // (arq_responder.cc SET_LINK_PARAMS handler). CMD applies locally on the
  // same call. Both sides update before the NEXT batch starts (the existing
  // RECEIVING_ACKS_CONTROL handshake gates the next data TX until SET_LINK_PARAMS
  // is ACKed).
  //
  // Emits `[POLICY-MOVE] axis=2 from=N to=M reason=R mean_partial=X good=G bad=B`
  // per §4.3.4 invariant #5 on every move. Gated on `sack_v2_enabled` at the
  // call site; v1 sessions never call this.
  //
  // rx_count is the number of frames the receiver acknowledged for the just-
  // completed batch (= batch_size on a clean full ACK, < batch_size when
  // SACK reported missing slots). batch_size_observed is the data_batch_size
  // value at the time this batch was sent (used to compute partial_rate
  // *before* applying any new batch size).
  void policy_evaluate_axis2(int rx_count, int batch_size_observed);

  // SACK Design A Step 10 — synthetic Axis 2 fire (test-only).
  // CLI: --test-policy-axis2-fire=up|down. Primes the partial-rate ring + the
  // consecutive-good / consecutive-bad counters at the move threshold and
  // calls policy_evaluate_axis2() once with a synthetic rx_count consistent
  // with the requested direction. Demonstrates the [POLICY-MOVE] axis=2 + the
  // SET_LINK_PARAMS TX path on a single one-shot fire. direction: 1=up,
  // 2=down. No-op for any other value. Default builds never call this.
  void test_fire_policy_axis2(int direction);

  // SACK Design A Step 10 — Axis-2 cooldown helper. Decrement the cooldown
  // counter by one (clamped at 0) and return the post-decrement value. Used
  // by the Axis-1 supremacy hook to express "Axis 2 must wait N batches
  // after any Axis 1 move." Pure-state, no logging.
  int axis2_cooldown_tick();

  // SACK Design A Step 12 — synthetic Axis-2 ceiling fire (test-only).
  // CLI: --test-policy-axis2-ceiling-fire=1. Drives a synthetic
  // "K just failed → ceiling=K-1" scenario, then attempts a synthetic
  // Axis-2 up-move from below K back toward K and asserts the up-move
  // is VETOED by the batch_size_proven_ceiling check. Default builds
  // never call this.
  void test_fire_policy_axis2_ceiling();

  // SACK Design A Step 12 — synthetic BREAK supremacy fire (test-only).
  // CLI: --test-policy-break-supremacy=1. Primes Axis-2 + Axis-3 state
  // to clean (no cooldown) then invokes the supremacy hook with a
  // synthetic BREAK reason tag. Demonstrates the new BREAK → supremacy
  // integration AND that 3 subsequent Axis-2 evaluations are suppressed.
  // Default builds never call this.
  void test_fire_policy_break_supremacy();

  // SACK Partial-Path BSI Non-Advance — in-process reproducer (test-only).
  // CLI: --test-partial-bsi-advance=mfsk|ofdm. Reproduces the bug in
  // fact-documents/sack_partial_bsi_advance.md §5.3 / §5.4: the MFSK
  // suffix ACK+SACK path bypasses send_sack_v2_frame() at
  // arq_responder.cc:1213, skipping Step 8a's bsi-bump-and-transfer-to-prev
  // block (arq_common.cc:4054-4137). When a mixbatch (retx-of-old-bsi +
  // new-bsi frames at a larger data_batch_size) arrives next, every
  // new-bsi frame is dropped as out_of_window.
  // 'mfsk' variant: should FAIL on HEAD (drop_count >= 26 for bsi=4).
  // 'ofdm' variant: should PASS on HEAD (regression guard).
  // Returns 0=PASS, 1=FAIL. Default builds never call this.
  int test_partial_bsi_advance(const char* transport);

  // SACK Design A Step 11 — Axis 3 controller (SACK mode ON↔PROBE↔OFF).
  //
  // policy_evaluate_axis3() implements the per-SACK-event §4.3.2 controller.
  //   - Observable: `sack_ok_rate = mean(recent_sack_ok)` over 10 most-recent
  //     SACK events; AND `consecutive_sack_misses`.
  //   - SACK event: every time CMD attempts a SACK_RSP decode at the
  //     expected window (post Step-7 decode_sack_v2_frame() call) OR the
  //     SACK window closes without ever receiving a SACK_RSP frame (= "no
  //     SACK heard within the window").
  //   - ok = LDPC/CRC decoded valid; miss = no SACK heard OR CRC8/LDPC failed.
  //   - Cadence: per-SACK-event.
  //   - Action: three-state {ON, PROBE, OFF}.
  //   - Hysteresis (§4.3.2):
  //       ON   → PROBE on consecutive_sack_misses >= 3
  //       PROBE→ ON    on the very next ok event
  //       PROBE→ OFF   on consecutive_sack_misses >= 5
  //       OFF  → PROBE every 20 batches (periodic re-probe — driven by
  //                    axis3_batch_tick(), not policy_evaluate_axis3())
  //
  // On state change: CMD writes pending_link_params_sack_mode and calls
  // add_message_control(SET_LINK_PARAMS) carrying the new mode (subject
  // to Axis-1 supremacy cooldown — see axis3_cooldown_batches). RSP applies
  // via its existing SET_LINK_PARAMS handler.
  //
  // SACK_MODE_OFF effect:
  //   CMD-side: do NOT enter decode_sack_v2_frame() (skip the SACK window
  //             v2 branch). Falls back to the existing receive_ack_pattern()
  //             full-batch ACK detection and, on timeout, the legacy
  //             retransmit path that marks PENDING_ACK → ACK_TIMED_OUT for
  //             process_messages_tx_data() to resend the whole batch. This
  //             is graceful degradation, not a protocol break.
  //   RSP-side: do NOT call send_sack_v2_frame() on partial batches. Lets
  //             CMD's ACK-timeout drive a full-batch retransmit. No
  //             NULL_SACK is sent; we simply skip the SACK window TX.
  //
  // SACK_MODE_PROBE behaves identically to ON for the next batch — RSP
  // sends SACK_RSP, CMD attempts decode. The probe outcome determines the
  // next transition (success → ON, miss → consecutive_sack_misses increments;
  // reaching 5 → OFF).
  //
  // Gated on `sack_v2_enabled` at every call site; v1 sessions never call
  // any Axis-3 function.
  //
  // ok parameter: true if decode_sack_v2_frame() returned true (CRC8 valid);
  // false if CRC8 failed OR the SACK window closed without any SACK_RSP
  // frame received.
  void policy_evaluate_axis3(bool ok);

  // SACK Design A Step 11 — helper used by policy_evaluate_axis3() and
  // axis3_batch_tick() to stage and send a SET_LINK_PARAMS carrying the
  // current data_batch_size + the new sack_mode. Private contract.
  void axis3_send_set_link_params(int new_sack_mode, const char* reason_tag);

  // SACK Design A Step 11 — per-batch tick for Axis 3.
  // Called once per batch completion regardless of whether a SACK event
  // fired (covers the OFF-state "every 20 batches re-probe" cadence and
  // tracks total batches for diagnostics).
  // Increments axis3_batches_since_off when sack_mode==OFF; on reaching
  // AXIS3_OFF_TO_PROBE_BATCHES (20), transitions OFF → PROBE and emits a
  // SET_LINK_PARAMS. Gated on sack_v2_enabled at call sites.
  void axis3_batch_tick();

  // SACK Design A Step 11 — synthetic Axis 3 fire (test-only).
  // CLI: --test-policy-axis3-fire={ok,miss}. Feeds one synthetic SACK event
  // into policy_evaluate_axis3(). Plus a composite flag fires 3 consecutive
  // misses to drive ON→PROBE, then 2 more for PROBE→OFF, then a final ok
  // to demonstrate would-be PROBE→ON (skipped due to SET_LINK_PARAMS init
  // dependency, similar to the Axis-2 demo). Default off; CLI-gated.
  void test_fire_policy_axis3(int kind);

  void process_messages_responder();
	//! Adds the received data message to the buffer.
	    /*!
	     * \param type is the message type.
	     * \param id is message id.
	     * \param length is the message content length.
	     * \param data is the message content.
	     *  \return SUCESSFUL or ERROR
	   */
  int add_message_rx_data(char type, char id, int length, char* data);
	//! Prepares control ack message.
	    /*!
	      \return None
	   */
  void process_messages_rx_data_control();
	//! Prepares control ack message.
	    /*!
	      \return None
	   */
  void process_messages_acknowledging_control();
	//! Prepares data ack message.
	    /*!
	      \return None
	   */
  void process_messages_acknowledging_data();
  void process_control_responder();
  void process_buffer_data_responder();

  void copy_data_to_buffer();
  void restore_backup_buffer_data();
  void restore_tx_from_compressed();  // Decompress messages_tx back to raw in fifo_buffer_tx

	//! Receives a data or a control message from the other end (via ALSA driver).
	    /*!
	      \return None
	   */
  void receive();

	//! Prints debug information.
	    /*!
	      \return None
	   */
  void print_stats();

  void reset_all_timers();
  void reset_session_state();

  cl_configuration_arq default_configuration_ARQ;


  int message_transmission_time_ms;
  int ctrl_transmission_time_ms;
  int ack_pattern_time_ms;  // Level 3: ACK pattern TX duration (ms)
  int data_batch_size;
  int nominal_batch_size;   // Max batch size for current config (12s target ceiling)
  int batch_consec_acks;    // Consecutive successful batch ACKs (for adaptive growth)
  int control_batch_size;
  int ack_batch_size;
  int batch_rx_frame_count;  // Total data frames decoded in current RX batch (including padding duplicates)
  bool batch_data_delivered; // True after copy_data_to_buffer() — prevents re-delivery on retransmission
  int block_ready;
  int block_under_tx;

  // SACK: Selective ACK for partial batch retransmission
  bool sack_enabled;                   // Negotiated: both sides have CAP_SACK
  bool sack_v2_enabled;                // SACK Design A scaffolding (Step 6): both sides have CAP_SACK_V2.
                                       // NEGOTIATE-ONLY at this step — gates no behavior yet.
  bool enable_sack_v2;                 // Harness-compat no-op since CAP_SACK_V2 was removed
                                       // (2026-05-24). SACK v2 is unconditional; --enable-sack-v2
                                       // and --disable-sack-v2 still toggle this for tools that
                                       // read it, but no wire behavior depends on it.
  int radio_batch_size;                // Total frames per radio TX (e.g., 25)
  int crypto_batch_size;               // Frames per encryption unit (e.g., 20)
  int retransmit_headroom;             // radio_batch_size - crypto_batch_size (e.g., 5)
  int crypto_batch_counter_tx;         // Monotonic crypto batch ID for TX (mod 8 in header)
  int crypto_batch_counter_rx;         // Expected crypto batch ID for RX

  // Commander: retransmit queue (missing frames from last SACK)
  bool sack_retransmit_active;         // True during retransmit batch TX (skip seq renumbering)
  int retransmit_count;                // Number of frames to retransmit
  int retransmit_batch_id;             // Crypto batch ID of frames being retransmitted
  unsigned char retransmit_frames[MAX_RETRANSMIT_HEADROOM][MAX_SACK_FRAME_SIZE];
  // Per-instance scratch buffers for the v2 mixed-batch retx prefix.
  // Each messages_batch_tx[i].data slot ALIASES messages_tx[i].data via the
  // struct-copies at arq_commander.cc:1225/1277 (and other sites). If the
  // retx-prefix loop memcpys retx bytes INTO messages_batch_tx[r].data
  // directly, it clobbers the underlying messages_tx[r] buffer; then the
  // new-data loop's struct-copy propagates the corrupted pointer onto
  // messages_batch_tx[r+R].data, causing the first R "new-data" frames to
  // carry byte-identical retx content on the wire. Fix: retx-prefix memcpys
  // INTO this dedicated scratch buffer instead, then points
  // messages_batch_tx[r].data at retx_scratch[r]. The original messages_tx
  // buffer is never touched. Verified by runtime [TX-ALIAS-CHECK-POST]
  // diagnostic firing identical=YES-BUG on the pre-fix binary, then no-bug
  // on the fixed binary.
  unsigned char retx_scratch[MAX_RETRANSMIT_HEADROOM][MAX_SACK_FRAME_SIZE];
  int retransmit_frame_lengths[MAX_RETRANSMIT_HEADROOM];
  int retransmit_frame_positions[MAX_RETRANSMIT_HEADROOM]; // Original frame_pos within crypto batch
  int retransmit_frame_types[MAX_RETRANSMIT_HEADROOM];     // DATA_LONG or DATA_SHORT
  int retransmit_frame_batch_seq_ids[MAX_RETRANSMIT_HEADROOM]; // SACK Design A Step 3 —
                                                               // original batch_seq_id of the
                                                               // batch this frame was first
                                                               // sent under. Retransmits carry
                                                               // their original value; never the
                                                               // current cmd_batch_seq_id.
  // §7.13.39 Fix 3 — preserve the ORIGINAL sequence_number byte (including
  // bit 7 = EOB) at the moment the frame was captured into the retx queue.
  // Without this, retx of a frame that was the last in its original batch
  // loses the EOB marker, causing RSP to mis-size the batch on the retx path.
  // Bit layout matches messages_tx[i].sequence_number: low 7 bits = original
  // slot in the prev batch, bit 7 = EOB on that original transmission.
  unsigned char retransmit_frame_seq_with_eob[MAX_RETRANSMIT_HEADROOM];

  // SACK Design A Step 3 — batch_seq_id plumbing (TX side: CMD-only counter;
  // RX side: diagnostic store; no decision branches on this value yet).
  // §4.1 + §4.3.4 invariant 2: increments by 1 mod 256 per NEW-DATA batch.
  // Retransmits use the captured original value (retransmit_frame_batch_seq_ids
  // or, for retransmit-only batches built from messages_tx, the saved
  // captured_batch_seq_id_for_retransmit). NOT reset on set_data_batch_size()
  // moves (the field is independent of batch size, §4.3.4 invariant 2).
  int cmd_batch_seq_id;                // CMD: counter for next NEW-DATA batch (mod 256).
                                       //      First new-data batch sent under value 0;
                                       //      subsequent batches under 1, 2, 3, ...
  int captured_batch_seq_id_for_retransmit;
                                       // CMD: snapshot of cmd_batch_seq_id at the moment a
                                       //      SACK retransmit queue is populated. The
                                       //      subsequent retransmit-only batch carries this
                                       //      value (the ORIGINAL batch's id), never the
                                       //      live cmd_batch_seq_id. -1 = unset.
  int last_received_batch_seq_id;      // RSP / monitor: last batch_seq_id parsed off a
                                       //      DATA_LONG/DATA_SHORT wire byte. Diagnostic
                                       //      only; no decision branches on it at Step 3.
                                       //      -1 = never received (v1 session, or no DATA yet).

  // SACK Design A Step 4 — RSP cross-batch routing decision state.
  // All members gated on sack_v2_enabled; v1 path never reads these.
  // Per §4.2.3 + §4.3.4 invariant #3: RSP routes DATA frames using
  // batch_seq_id. Unknown / out-of-window ids are discarded + logged
  // ([RSP-V2-DROP]). The "prev" slot is the SACK-retransmit window
  // (mechanism (b) lands in Step 8 — for Step 4 it is defensive
  // scaffolding; in today's mechanism (a) retransmits carry the
  // CURRENT batch_seq_id and always match `rsp_current_expected_batch_seq_id`).
  int rsp_current_expected_batch_seq_id; // RSP: expected current batch_seq_id.
                                         //      Adopted from first v2 DATA frame seen
                                         //      (-1 sentinel = not yet adopted). Bumped
                                         //      by +1 mod 256 at ACK-GATE-PASS (full
                                         //      batch ACKed). NEVER bumped on partial /
                                         //      SACK paths.
  int rsp_prev_batch_seq_id;             // RSP: previous batch_seq_id (one back from
                                         //      current_expected, mod 256). Set at the
                                         //      same ACK-GATE-PASS moment that bumps
                                         //      current_expected. -1 = no prior batch
                                         //      yet (first session batch in progress).
  long long rsp_v2_drop_count;           // RSP: counter of [RSP-V2-DROP] events
                                         //      (frames discarded for unknown
                                         //      batch_seq_id). Validates the Step 4
                                         //      "must NOT fire in clean traffic"
                                         //      property. v2-loopback tests assert
                                         //      this is 0.
  // Test-scaffold fault injection (CLI --test-rsp-bsi-corrupt-at=N). When > 0,
  // corrupts the Nth v2 DATA frame's parsed batch_seq_id by adding 7 (mod 256)
  // BEFORE routing — guaranteed to fall outside {current_expected, prev}.
  // Lets the synthetic discard test demonstrate the new branch fires. Default 0
  // (off); only the Nth frame is touched, subsequent frames pass through clean.
  int test_rsp_bsi_corrupt_at;           // RSP: 1-indexed frame number to corrupt;
                                         //      0 = scaffold off.
  int test_rsp_bsi_v2_frame_counter;     // RSP: count of v2 DATA frames received
                                         //      (1-indexed); used to match
                                         //      test_rsp_bsi_corrupt_at exactly once.

  // SACK Design A Step 7 — SACK_RSP OFDM control-frame state.
  // ALL gated on sack_v2_enabled. v1 path never reads or writes these.
  // §4.2.2: SACK_RSP replaces the ~1168 ms MFSK SACK pattern with a single
  // OFDM LDPC control frame (~390 ms at WB_CFG10). Wire payload after the
  // standard 3-byte msg header: [batch_seq_id : u8][bitmap : ceil(N/8) bytes][CRC8 : u8].
  // The TX-side ground truth bitmap is logged via [TX-SACK-V2]; the RX-side
  // decoded bitmap is logged via [CMD-SACK-V2]; a v2<->v2 round-trip asserts
  // the two are byte-identical (the Gate-2 deliverable).
  long long rsp_sack_v2_tx_count;        // RSP: count of SACK_RSP frames TX'd.
                                         //      Diagnostic only; v2-loopback tests
                                         //      assert this is > 0 when the SACK
                                         //      partial path was exercised.
  long long cmd_sack_v2_rx_count;        // CMD: count of SACK_RSP frames received
                                         //      AND CRC-validated (CRC-failed frames
                                         //      do NOT increment this — they bump
                                         //      cmd_sack_v2_crc_fail_count instead).
  long long cmd_sack_v2_crc_fail_count;  // CMD: count of SACK_RSP frames whose
                                         //      CRC8 did not match. Discarded with
                                         //      [CMD-SACK-V2-CRC-FAIL] log; bitmap
                                         //      is NOT applied to retransmit queue
                                         //      (§9.4/A2 "no fabrication" lesson —
                                         //      the missing SACK falls back to the
                                         //      existing ACK timeout / retransmit
                                         //      path, exactly as if the OFDM
                                         //      control frame had been lost on
                                         //      the air).
  unsigned char cmd_sack_v2_last_rx_bitmap[MAX_SACK_BATCH_SIZE / 8 + 1];
                                         // CMD: most recent decoded bitmap bytes
                                         //      (post-CRC). For test scaffold
                                         //      observability. Sized to fit any
                                         //      Design A batch size (max 50 → 7
                                         //      bytes; we provision MAX_SACK_BATCH_SIZE/8+1
                                         //      = 5 bytes which covers
                                         //      data_batch_size up to 32).
  int           cmd_sack_v2_last_rx_nbits;
                                         // CMD: number of valid bits in
                                         //      cmd_sack_v2_last_rx_bitmap (= the
                                         //      data_batch_size for which the
                                         //      SACK_RSP was generated). -1 = no
                                         //      SACK_RSP received this session.
  int           cmd_sack_v2_last_rx_batch_seq_id;
                                         // CMD: batch_seq_id field from the most
                                         //      recent CRC-validated SACK_RSP.
                                         //      -1 = none.
  // Test-scaffold fault injection (CLI --test-rsp-sack-rsp-crc-corrupt). When
  // true, the next SACK_RSP frame the RSP transmits has its trailing CRC8 byte
  // XOR'd with 0xFF before TX. One-shot; clears after firing. Used to
  // demonstrate the CMD's CRC8 discard branch fires (§4.2.2 / §9.4/A2). Default
  // false; production builds never pass this flag.
  bool test_rsp_sack_rsp_crc_corrupt;
  bool test_rsp_sack_rsp_crc_corrupt_armed;   // one-shot arm flag (mirrors
                                              // test_rsp_sack_rsp_crc_corrupt at
                                              // configure time; cleared when
                                              // the corruption fires).
  // SACK Design A Step 11 — N-shot CRC8 fault injection. When >0, the next N
  // SACK_RSP frames will have their CRC8 XOR'd with 0xFF; decrements per
  // SACK_RSP TX. Independent of the one-shot armed flag above.
  // CLI: --test-rsp-sack-rsp-crc-corrupt-count=N. Default 0 (no corruption).
  int test_rsp_sack_rsp_crc_corrupt_count;

  // SACK Design A Step 8a — RSP-side prev-batch parallel storage.
  // ALL gated on sack_v2_enabled. v1 path never reads or writes any of these
  // (the parallel buffer is allocated unconditionally for simplicity but is
  // only ever populated or routed-into when sack_v2_enabled).
  //
  // §7.8.3 named the architectural problem this solves: when CMD will (in a
  // future Step 8b) send mixed retransmit+new-data batches, the in-flight
  // new-data batch (current) and the prior batch's retransmits (prev) must
  // not collide on the single flat `messages_rx[]` array. Step 8a adds the
  // parallel `messages_rx_prev[]` storage and the bump-at-SACK-RSP-send hook
  // so prev-batch retransmits land in their own buffer, independent from
  // current-batch frames. Step 8a does NOT change CMD behavior — CMD still
  // sends standalone retransmit-only batches; the new path is exercised by
  // those v2 standalone retransmits flowing into the prev buffer.
  //
  // Lifecycle of `messages_rx_prev[]`:
  //   1. SACK partial branch (`arq_responder.cc:914-975`) calls
  //      `send_sack_v2_frame()` for batch N.
  //   2. Inside `send_sack_v2_frame()`, BEFORE TX: transfer (not copy) the
  //      in-flight `messages_rx[]` contents for slots 0..data_batch_size-1
  //      into `messages_rx_prev[]`; mark `messages_rx[]` slots FREE; record
  //      `rsp_prev_batch_active=true`, `rsp_prev_batch_seq_id = N`, bump
  //      `rsp_current_expected_batch_seq_id = N+1`. This is the NEW bump
  //      site that §7.8.2 named as missing. The Step 4 ACK-GATE-PASS bump
  //      (`arq_responder.cc:1029-1037`) is preserved in addition (it fires
  //      on clean-batch full delivery — the prev buffer stays inactive in
  //      that case).
  //   3. CMD sends standalone retransmit-only batch for N (today's
  //      mechanism (a) — unchanged in Step 8a). Frames carry bsi=N.
  //   4. RSP routing: `match_prev` is now true → frames are stored into
  //      `messages_rx_prev[]` (NOT `messages_rx[]`). This is the routing
  //      modification: Step 4 routed match-prev hits into `messages_rx[]`
  //      which §7.8.3 named as the silent-corruption hazard.
  //   5. When `rsp_prev_batch_received_count >= rsp_prev_batch_expected_count`,
  //      deliver the prev batch via `copy_data_to_buffer()` with a
  //      `messages_rx`/`messages_rx_prev` pointer swap, then clear
  //      `messages_rx_prev[]` slots back to FREE and set
  //      `rsp_prev_batch_active = false`.
  //   6. Subsequent batch N+1 new-data frames carry bsi=N+1 → match_current
  //      → routed to `messages_rx[]` (now empty, no collision).
  struct st_message* messages_rx_prev;
                                         // RSP: parallel prev-batch buffer.
                                         //      Allocated identically to
                                         //      `messages_rx` (size nMessages,
                                         //      each slot's `data` of size
                                         //      N_MAX/8 + CANARY_SIZE). Frames
                                         //      whose bsi matches the *prev*
                                         //      window are stored here instead
                                         //      of in `messages_rx`. NULL until
                                         //      `init_messages_buffers()` runs.
  bool rsp_prev_batch_active;            // RSP: true while a prev-batch is
                                         //      waiting for retransmits to
                                         //      complete. Set when
                                         //      `send_sack_v2_frame()` transfers
                                         //      messages_rx → messages_rx_prev;
                                         //      cleared when prev delivery
                                         //      completes (or when forcibly
                                         //      discarded — see prev-stale
                                         //      branch). Defaults to false.
  int rsp_prev_batch_received_count;     // RSP: count of slots currently in
                                         //      RECEIVED state in
                                         //      messages_rx_prev for the
                                         //      active prev batch.
  int rsp_prev_batch_expected_count;     // RSP: expected RECEIVED count for
                                         //      prev-batch completion. Set
                                         //      from `last_received_end_of_batch_seq + 1`
                                         //      (if known) or data_batch_size
                                         //      at SACK_RSP-send time. The
                                         //      same EOB-inference logic
                                         //      `process_messages_acknowledging_data`
                                         //      uses for the *current* batch.
  long long rsp_prev_batch_delivered_count; // RSP: count of prev batches
                                         //      successfully delivered via
                                         //      the prev path (diagnostic;
                                         //      validates "no silent loss" on
                                         //      with-losses tests).
  long long rsp_prev_batch_stale_count;  // RSP: count of times a new SACK_RSP
                                         //      send would have overwritten
                                         //      an active prev batch (rare —
                                         //      means CMD never finished
                                         //      retransmits for the prior
                                         //      prev batch within RSP's
                                         //      visibility window). Indicates
                                         //      a stalled retransmit cycle;
                                         //      logged via [RSP-V2-PREV-STALE].

  // SACK Design A Step 10 — Axis 2 controller state (adaptive batch size).
  // ALL CMD-side; gated on `sack_v2_enabled` at the call sites. v1 sessions
  // never read or write these (they stay at sentinels). RSP keeps no Axis-2
  // state of its own — RSP only RECEIVES SET_LINK_PARAMS and applies the
  // new batch size; CMD owns the decision (§3.8 initiator-controls-flow).
  //
  // §4.3.1 state-space row 2: data_batch_size already exists; this block
  // adds the observation window (rolling 5-batch ring) and the hysteresis
  // counters. §4.3.2 thresholds: up at mean<0.05 AND >=8 good; down at
  // mean>0.20 AND >=3 bad. Reset on any move. §4.3.3 cross-axis cooldown:
  // skip when axis2_cooldown_batches > 0 (set to 3 by the Axis-1 supremacy
  // hook on every Axis-1 move).
  //
  // §4.3.1 batch-size range nominally [10, 50]; Mercury's existing
  // MAX_SACK_BATCH_SIZE=32 bitmap allocation caps the runtime ceiling to 32
  // for Step 10 — documented in §7.10 RESULT. AXIS2_BATCH_FLOOR and
  // AXIS2_BATCH_CEIL provide a single source of truth for clamping.
  static const int AXIS2_BATCH_FLOOR = 10;
  static const int AXIS2_BATCH_CEIL  = 32;  // capped by MAX_SACK_BATCH_SIZE
  static const int AXIS2_STEP        = 5;
  static const int AXIS2_RING_DEPTH  = 5;
  static const int AXIS2_UP_GOOD_RUN = 4;  // §4.3.2 hysteresis: up after N good
                                           // batches. §7.13.32 hardened the
                                           // SET_LINK_PARAMS handshake (RSP
                                           // now releases messages_control
                                           // after every v2 OFDM control TX),
                                           // so the 8→4 ramp is now safe.
                                           // History: §7.13.31.1 first
                                           // attempted the ramp at v18+v19
                                           // and surfaced the RX-CTRL-DROP
                                           // bug on RSP — root cause was
                                           // messages_control left in
                                           // ADDED_TO_BATCH_BUFFER state by
                                           // send_ofdm_ack_clean() and
                                           // send_sack_v2_frame(). Fixed in
                                           // §7.13.32 (arq_common.cc).

  static const int AXIS2_DOWN_BAD_RUN = 3; // §4.3.2 hysteresis: down after this many bad batches
  static const int AXIS2_CROSS_AXIS_COOLDOWN_BATCHES = 3; // §4.3.3 set by Axis-1 supremacy hook
  // SACK Design A Step 12 — Axis-2 `batch_size_proven_ceiling` analogue
  // (§4.3.4 invariant #7). After a down-move at K (the batch_size that just
  // produced > 20 % partial rate), Axis-2 is forbidden from proposing any
  // value above (K-1) for AXIS2_CEILING_RECOVERY_BATCHES subsequent
  // evaluations. Mirrors the §2.1 supershift_proven_ceiling discipline for
  // Axis 1 (modulation): a config that just failed cannot be re-tried
  // without an explicit recovery interval. The ceiling RESETS on any
  // Axis-1 move (channel changed, prior ceiling stale).
  static const int AXIS2_CEILING_RECOVERY_BATCHES = 20;

  float axis2_partial_rate_ring[AXIS2_RING_DEPTH];
  int   axis2_partial_rate_count;        // [0..AXIS2_RING_DEPTH]; pre-fill before mean
  int   axis2_partial_rate_pos;          // ring write index
  int   axis2_consecutive_good_batches;  // partial_rate < 0.05 hits
  int   axis2_consecutive_bad_batches;   // partial_rate > 0.20 hits
  int   axis2_cooldown_batches;          // §4.3.3 Axis-1 supremacy cooldown — Axis-2
                                         //      skips evaluation while >0; decremented
                                         //      per evaluation call.
  // §4.3.4 invariant #7 — proven-ceiling state (Step 12).
  //  ceiling == -1 ⇒ no cap (default; either no prior failure or already recovered).
  //  ceiling >=  0 ⇒ Axis-2 may NOT propose to > ceiling until recovery_remaining == 0.
  // Recovery counter decrements on every policy_evaluate_axis2() call; on reaching
  // 0, the ceiling clears (-1). Axis-1 supremacy resets the ceiling unconditionally.
  int   batch_size_proven_ceiling;       // mirrors supershift_proven_ceiling (Axis-1)
  int   batch_size_ceiling_recovery_batches; // batches remaining before ceiling clears
  long long axis2_ceiling_blocks_count;  // count of up-moves vetoed by the ceiling
  long long axis2_evaluations;           // count of policy_evaluate_axis2() calls
  long long axis2_move_up_count;         // count of step-up moves
  long long axis2_move_down_count;       // count of step-down moves
  long long axis2_skipped_in_cooldown;   // count of evaluations skipped due to cooldown
  // Test-scaffold for synthetic Axis-2 fire (CLI --test-policy-axis2-fire=up|down).
  // 0 = off; 1 = up; 2 = down. One-shot — cleared after firing.
  int   test_policy_axis2_fire_armed;
  // Test-scaffold to demonstrate the cross-axis cooldown:
  // --test-cmd-axis2-suppressed-after-axis1=1 fires synthetic Axis-1
  // moves followed by attempted Axis-2 fires, and asserts they are
  // suppressed for the cooldown duration.
  int   test_cmd_axis2_suppressed_after_axis1;
  // RSP-side diagnostic counters for SET_LINK_PARAMS receipt.
  long long rsp_set_link_params_rx_count;       // CRC-valid SET_LINK_PARAMS received
  long long rsp_set_link_params_crc_fail_count; // CRC failed; ignored

  // CMD-side staging fields used by add_message_control(SET_LINK_PARAMS)
  // to read the target batch size + sack_mode that the Axis-2 (Step 10) /
  // Axis-3 (Step 11) controllers decided on. policy_evaluate_axis2() writes
  // these before calling add_message_control(SET_LINK_PARAMS); the control
  // frame's data payload is built from them. Sentinels: -1 = unset (caller
  // will substitute current data_batch_size / 1=ON).
  int pending_link_params_batch_size;
  int pending_link_params_sack_mode;

  // SACK Design A Step 11 — Axis 3 controller state (SACK mode adaptation).
  // BOTH peers track sack_mode (CMD decides, RSP obeys via SET_LINK_PARAMS).
  // All ring/counter/diagnostic state is CMD-side only; RSP only needs the
  // mode value itself (axis3_sack_mode) to gate send_sack_v2_frame().
  //
  // §4.3.1 sack_mode encoding (matches SET_LINK_PARAMS wire byte):
  //   0 = SACK_MODE_OFF   (RSP suppresses SACK_RSP; CMD ignores SACK window)
  //   1 = SACK_MODE_ON    (default when sack_v2_enabled negotiated)
  //   2 = SACK_MODE_PROBE (next batch behaves as ON; outcome drives transition)
  //
  // §4.3.2 hysteresis thresholds + §4.3.3 cross-axis cooldown:
  static const int SACK_MODE_OFF   = 0;
  static const int SACK_MODE_ON    = 1;
  static const int SACK_MODE_PROBE = 2;
  static const int AXIS3_RING_DEPTH               = 10; // ring of 10 recent SACK events
  static const int AXIS3_ON_TO_PROBE_MISSES       = 3;  // ON→PROBE on 3 consecutive misses
  static const int AXIS3_PROBE_TO_OFF_MISSES      = 5;  // PROBE→OFF on 5 consecutive misses (since the ON→PROBE entry)
  static const int AXIS3_OFF_TO_PROBE_BATCHES     = 20; // OFF→PROBE every 20 batches
  static const int AXIS3_CROSS_AXIS_COOLDOWN_BATCHES = 3; // §4.3.3 Axis-1 supremacy cooldown — Axis-3

  int  axis3_sack_mode;                  // current Axis-3 state (SACK_MODE_*)
  bool axis3_recent_sack_ok[AXIS3_RING_DEPTH]; // ring of 10 most-recent events (true=ok, false=miss)
  int  axis3_recent_sack_ok_count;       // [0..AXIS3_RING_DEPTH]; pre-fill before mean
  int  axis3_recent_sack_ok_pos;         // ring write index
  int  axis3_consecutive_sack_misses;    // streak counter (reset on any ok event)
  int  axis3_batches_since_off;          // OFF→PROBE re-probe timer (counts batches in OFF)
  int  axis3_cooldown_batches;           // §4.3.3 Axis-1 supremacy cooldown — Axis-3 skips
                                         //      moves while >0; decremented per batch tick.
  long long axis3_evaluations;           // count of policy_evaluate_axis3() calls
  long long axis3_ok_events;             // count of ok events
  long long axis3_miss_events;           // count of miss events
  long long axis3_move_on_to_probe_count;
  long long axis3_move_probe_to_on_count;
  long long axis3_move_probe_to_off_count;
  long long axis3_move_off_to_probe_count;
  long long axis3_skipped_in_cooldown;   // moves suppressed by cross-axis cooldown
  // Test-scaffold for synthetic Axis-3 fire.
  int  test_policy_axis3_fire_armed;     // 0=off, 1=ok-event, 2=miss-event
  int  test_policy_axis3_walk_armed;     // 0=off, 1=walk ON→PROBE→OFF→PROBE (composite demo)

  // Responder: double-buffered crypto batch storage
  st_crypto_batch_buffer crypto_buf[2]; // [0] = oldest pending, [1] = current
  int max_message_length;
  int max_data_length;
  int max_header_length;

  int connection_status;
  int link_status;
  int role;
  int original_role;
  char connection_id;
  char assigned_connection_id;


  cl_tcp_socket tcp_socket_control;
  cl_tcp_socket tcp_socket_data;


  cl_timer watchdog_timer;
  cl_timer link_timer;
  cl_timer receiving_timer;
  cl_timer print_stats_timer;
  cl_timer gear_shift_timer;
  cl_timer switch_role_timer;
  cl_timer switch_role_test_timer;
  cl_timer connection_attempt_timer;

  float print_stats_frequency_hz;

  int message_batch_counter_tx;

  char* message_TxRx_byte_buffer;
  struct st_message messages_rx_buffer;

  struct st_message messages_last_ack_bu;
  struct st_message messages_control_bu;
  struct st_message messages_control;
  struct st_message* messages_batch_tx;

  int ack_timeout_control;
  int ack_timeout_data;
  int link_timeout;
  int watchdog_timeout;
  int receiving_timeout;
  int switch_role_timeout;
  int switch_role_test_timeout;
  int gearshift_timeout;
  int connection_timeout;

  std::string destination_call_sign;

  cl_fifo_buffer fifo_buffer_tx;
  cl_fifo_buffer fifo_buffer_rx;
  cl_fifo_buffer fifo_buffer_backup;

  cl_telecom_system* telecom_system;

  char data_configuration;
  char init_configuration;
  char last_data_configuration;
  char current_configuration;
  char ack_configuration;
  char negotiated_configuration;
  char forward_configuration;   // Commander→Responder TX speed (asymmetric gearshift)
  char reverse_configuration;   // Responder→Commander TX speed (after SWITCH_ROLE)

  int gear_shift_on;
  int robust_enabled;
  int narrowband_enabled;  // 0=wideband (2344 Hz), 1=narrowband (469 Hz)
  int commander_configured_nb;  // commander's original NB setting (-1=unset, YES/NO)
  int nb_probe_max;             // max NB probe attempts before fallback (default 2)
  bool session_narrowband;      // negotiated NB for this session (NB always wins)
  int bandwidth_mode;           // BW_AUTO=0, BW_NB_ONLY=1
  uint8_t local_capability;    // CAP_WB_CAPABLE | CAP_COMPRESSION
  uint8_t peer_capability;     // Received from peer via TEST_CONNECTION
  bool wb_upgrade_pending;     // True between SWITCH_BANDWIDTH send and ACK

  // v9 handshake echo state. handshake_confirmed gates CMD's
  // CONNECTION_ACCEPTED → NEGOTIATING/CONNECTED transition. retries_left
  // counts failed echo validations before dropping with explicit error.
  bool handshake_confirmed;
  int  handshake_retries_left;
  static const int MAX_HANDSHAKE_RETRIES = 5;

  cl_compressor compressor;           // Block compression (PPMd + zstd)
  bool compression_enabled;           // Negotiated: both sides have CAP_COMPRESSION
  bool force_compress;                // CLI -F on: always enable compression (skip B2F detection)
  bool b2f_compression_pending;       // B2F SID detected, arm compression on next data ACK
  cl_b2f_handler b2f_handler;         // B2F protocol handler (Winlink LZHUF unroll/reroll)
  float compress_ratio_estimate;      // Running compression ratio (raw/compressed), init 2.0
  int batch_uncompressed_size;        // Uncompressed bytes in current TX batch (for throughput)

  // Encryption (hybrid PQ: X25519 + ML-KEM-768 + ChaCha20-Poly1305)
  cl_cipher_suite cipher_suite;       // Per-connection cipher state (ephemeral keys, session key)
  int encryption_mode;                // ENCRYPT_OFF, ENCRYPT_STRICT, ENCRYPT_FAST
  bool encryption_enabled;            // Negotiated: both sides have CAP_ENCRYPTION and mode != OFF
  uint64_t tx_batch_counter;          // Monotonic counter for encrypt nonces (TX direction)
  uint64_t rx_batch_counter;          // Monotonic counter for decrypt nonces (RX direction)
  int consecutive_auth_failures;      // Auth failures since last success (3 → disconnect)
  uint8_t* kx_data_buf;              // Buffer for ML-KEM key exchange data (1184 or 1088 bytes)
  int kx_data_len;                    // Length of pending key exchange data
  char psk_hex[129];                  // Pre-shared key (hex string, up to 64 bytes = 128 hex chars)
  bool psk_mismatch_pending;          // Commander detected PSK mismatch, KEY_ACTIVATE sent for responder notification

  int gear_shift_algorithm;
  double gear_shift_up_success_rate_precentage;
  double gear_shift_down_success_rate_precentage;
  int gear_shift_block_for_nBlocks_total;
  int gear_shift_blocked_for_nBlocks;
  int gear_shift_down_consecutive_fails;  // Consecutive bad blocks before downshift
  // CLEAN-BATCH VIABILITY (§9): the UP-promotion success rate, computed from
  // nBatches_fully_acked (CLEAN, all-ones batches) — distinct from
  // last_transmission_block_stats.success_rate_data (nBatches_acked, counts
  // partial-SACK recoveries too). The two LADDER-UP gates read THIS so a
  // partial-only run reads ~0% and does NOT climb; the DOWN-shift trigger /
  // logging keep reading success_rate_data (partial is a real delivery there).
  // Recomputed every finalize_block_commander() alongside success_rate_data.
  // See fact-documents/gearshift-start-and-recovery.md §9.
  double success_rate_data_clean;
  int consecutive_data_acks;       // Frame-level gearshift: consecutive successful data ACKs
  int frame_shift_threshold;       // Shift up after this many consecutive ACKs (default 3)
  bool frame_gearshift_just_applied;  // true after frame upshift ACKed — BREAK on first data failure
  int  frame_gearshift_retry_count;   // §7.13.33: retries on PHY-switched first batch before BREAK (rx_mute timing race)

  // Turboshift: bidirectional probing phase before data exchange
  enum TurboshiftPhase { TURBO_FORWARD, TURBO_REVERSE, TURBO_DONE };
  TurboshiftPhase turboshift_phase;
  bool turboshift_active;          // true = currently probing (climbing the ladder)
  int turboshift_last_good;        // last config that decoded successfully (-1 = none)
  bool turboshift_initiator;       // true = I started turboshift (original commander)
  int turboshift_retries;          // retries left at current config (0 = ceiling)
  bool turbo_settle_pending;       // waiting for settle SET_CONFIG ACK before finish
  bool turbo_supershift_announce_pending; // SUPERSHIFT entry queued SET_CONFIG; gates re-entry until ACK lands or BREAK clears turboshift_active. Without this, the entry block fired up to 50× within ~50ms on tight state-machine ticks, blindly walking the ladder past what the channel can hold (observed at WGN:-10 2026-05-29).
  int supershift_proven_ceiling;   // highest config that failed BREAK — caps all SUPERSHIFT targets (-1 = no ceiling)
  // Option B (data-anchored gearshift promotion, 2026-05-29): highest ladder
  // rung (by config_ladder_index) at which a DATA batch was CONFIRMED delivered
  // this session. Init = init_configuration (ctor + reset_session_state); raised
  // ONLY at the data-success path (arq_commander.cc:3284, data_ack_received==YES).
  // Consumers: BREAK target floor (arq_commander.cc:81, gated off under panic),
  // up-shifter anchor gate (legacy ladder :4583, axis1 :4731, retrigger :4344) —
  // no upward move may exceed index+1 unless optimizer_is_in_control(). A FAILED
  // probe never raises it. See fact-documents/gearshift-start-and-recovery.md §6/§7.
  int last_data_viable_config;
  // CLEAN-BATCH VIABILITY (§9, 2026-05-29): TRUE iff the batch that set
  // data_ack_received=YES this epoch was confirmed FULLY delivered (all-ones
  // bitmap clean ACK / LDPC ACK_RANGE/ACK_MULTI). FALSE on a PARTIAL SACK (which
  // still keeps the link alive + drives retransmit, but must NOT promote the
  // rung). Same lifecycle as data_ack_received: reset FALSE at each batch-TX
  // start (arq_commander.cc:1244/1739), ctor, and reset_session_state; set TRUE
  // ONLY at the clean acceptance sites (:2935 ACK_PAT, :3032 ACK_RANGE, :3052
  // ACK_MULTI). Gates the four promotion consumers (anchor-raise :3394, panic
  // reset :3387, break_drop_step :3386, FRAME-UP :3454). See §9.
  bool last_batch_fully_acked;
  bool skip_turbo_reverse;         // CLI --skip-turbo-reverse: skip TURBO_REVERSE phase
  int max_config_override;         // CLI --max-config: hard ceiling on turboshift (-1 = use default)
  bool optimizer_disabled;         // CLI --no-optimizer: disable Phase 3c effective-rate optimizer (calibration runs)
  void set_optimizer_disabled(bool b) { optimizer_disabled = b; }

  // Gearshift ↔ Q-table optimizer handoff. Above the lowest calibrated
  // config in the Q-table, the optimizer is the sole authority for upward
  // config changes — gearshift's FRAME UP / LADDER UP must yield. Below
  // that, the optimizer has no data and gearshift's SNR-based ladder runs.
  // BREAK fallback (downward to ROBUST_0) stays available to both bands as
  // the safety net. Returns false if optimizer is disabled, table is unloaded,
  // or current_configuration is below the calibrated band.
  bool optimizer_is_in_control() const {
    if (optimizer_disabled || !rate_opt.is_enabled()) return false;
    int handoff = rate_opt.min_calibrated_cfg(narrowband_enabled == YES);
    if (handoff <= 0) return false;
    return config_ladder_index(current_configuration) >= config_ladder_index(handoff);
  }

  // Cap a turboshift SNR-derived target at the handoff config so the
  // turboshift probe stops where the optimizer takes over. Without this,
  // turboshift overshoots to CFG14-16 and the optimizer immediately moves
  // Mercury back down. The cap is a no-op when the optimizer is disabled
  // or no calibrated cells exist.
  void apply_optimizer_handoff_cap_to_target(int *snr_target) const {
    if (optimizer_disabled || !rate_opt.is_enabled()) return;
    int handoff = rate_opt.min_calibrated_cfg(narrowband_enabled == YES);
    if (handoff <= 0) return;
    if (*snr_target > handoff) {
      printf("[TURBO] SNR target %d capped at handoff config %d (Q-table takes over)\n",
        *snr_target, handoff);
      fflush(stdout);
      *snr_target = handoff;
    }
  }
  bool turbo_snr_ack_enabled;      // true during turboshift: send/receive SNR in ACK suffix
  float turbo_received_snr;        // SNR decoded from ACK suffix (-99 = not available)
  float turbo_best_snr;            // Best SNR seen across entire turbo phase (-99 = none)
  cl_timer turbo_snr_defer_timer;  // defer ACK return until suffix arrives
  int turbo_switch_role_retries;   // consecutive SWITCH_ROLE failures during turbo (Bug #60)

  // ACK detection diagnostics (per-window tracking, no printf in hot loop)
  int ack_diag_peak_matched;
  double ack_diag_peak_metric;
  int ack_diag_poll_count;
  uint32_t ack_diag_peak_mask;  // bit i set = symbol i matched at peak detection

  // Bug A fix (§7.13.1 SACK_DESIGN_A_PLAN): bounded defer counter for the v2
  // SACK_RSP dispatch in process_messages_rx_acks_data(). When the v2 path
  // probes legacy ACK pattern first (to avoid this->receive() destroying the
  // MFSK ACK audio), a non-zero ack_diag_peak_matched can come from a real
  // ACK pattern in progress OR from spectral coincidence on OFDM SACK_RSP
  // audio at the MFSK ACK tone frequencies. To keep the partial-batch
  // SACK_RSP decoder from being permanently starved, the defer is bounded
  // to v2_ackpat_defer_count_this_window < 5 polls (~225 ms at WB symbol
  // rate). Reset at TX-end (arq_commander.cc:891 / :1141) alongside the
  // other per-window diag counters. v1 path never sets or reads this.
  int v2_ackpat_defer_count_this_window;

  // §7.13.29 — last SACK_RSP bsi (mod 256) we applied to messages_tx[].
  // Prevents duplicate apply if the same SACK_RSP preamble audio is
  // still in the ring on a subsequent poll. -1 = no SACK_RSP applied
  // yet this session. Init in init_messages_buffers.
  int cmd_last_applied_sack_bsi;

  // §7.13.30 — v2 OFDM dispatch new-audio throttle. The v2 SACK window
  // no longer calls receive_ack_pattern() (which used to drive ftr
  // implicitly). Instead we track ring_write_index advance ourselves
  // and only call receive() when at least N OFDM symbols of new
  // audio have accumulated. Without this throttle, receive() would
  // either early-return forever on stale ftr (v14 r1 bug: 4208 calls,
  // 0 decodes) or run every poll wasting CPU on identical audio.
  // Value: last ring_write_index sample at which we ran receive().
  // -1 = "no prior run this window" → first poll processes.
  int v2_dispatch_last_rwi;

  // §7.13.30 — how many OFDM symbols of new audio to require before
  // the next receive() call. Default 1 (one symbol of progress per
  // call). Bumped to (overflow + 4) after receive() returns INCOMPLETE
  // so the next call has enough fresh audio to complete the frame —
  // without this, each retry only adds 1 symbol and overflow drops by
  // 1 per call, wasting ~12 polls on the same frame (v15 bug).
  int v2_dispatch_min_advance_syms;

  // §7.13.29 (Option 2) — self-calibrating SACK_RSP arrival predictor.
  // Each successful SACK_RSP decode records the receiving_timer ms value
  // at which the cross-check returned RECEIVED. Future use: hint OFDM
  // search to the expected arrival time. Currently recording-only.
  static const int SACK_ARRIVAL_HISTORY = 8;
  int sack_arrival_history_ms[SACK_ARRIVAL_HISTORY];
  int sack_arrival_history_count;     // 0..SACK_ARRIVAL_HISTORY
  int sack_arrival_history_next_idx;  // 0..SACK_ARRIVAL_HISTORY-1 (ring)

  // Step 15: legacy SACK pattern detection diagnostics (sack_diag_*) removed —
  // the MFSK SACK correlator they tracked is gone.

  // Phase-2 validation flag: PHY reinit settle delay after SET_CONFIG ACK.
  // Default 300000 us (= 300 ms, HEAD behavior, b806b76 Bug #60). CLI:
  // --phy-reinit-settle-ms=N. Pass 0 to disable the settle entirely.
  // See IONOS_ERA_VALIDATION_PLAN.md §15b.4 / PHASE2_FLAGS_DESIGN.md §2.7.
  int phy_reinit_settle_us;

  // Phase-2 validation flag: ACK detection metric threshold (7076a4b 3.0→0.5).
  // CLI --ack-metric-threshold=F. Default 0.5 = HEAD; pass 3.0 to revert.
  // PHASE2_FLAGS_DESIGN.md §3.1.
  double ack_metric_threshold;

  // Phase-2 validation flag: extra timeout added to receive_timeout when
  // sack_enabled (7076a4b new). CLI --sack-timeout-extra-ms=N. Default 3000
  // = HEAD; pass 0 to revert pre-7076a4b behavior. Suspected contributor to
  // the NB_CFG10 162→54 bps cliff at HEAD.
  int sack_timeout_extra_ms;

  // CAP_SACK gating. Default false (SACK negotiated by default) after
  // SACK Design A shipped (e73968a..1acdb3c on monitor, May 16-17 2026)
  // and resolved the regressions that motivated the original B2 disable
  // (2026-05-12). SACK is now load-bearing for the effective-rate
  // optimizer's calibrated operating range (CFG6+).
  // CLI overrides:
  //   --no-sack       (forces disable; opt-out)
  //   --enable-sack   (forces enable; no-op since default is now enabled)
  bool disable_sack;

  // Step 15: --test-sack-ldpc-fail / force_sack_ldpc_fail removed alongside
  // the legacy MFSK SACK receive path.

  // Emergency BREAK: drop to ROBUST_0 when current config is undecodable
  int emergency_nack_count;       // consecutive failed data blocks
  int emergency_nack_threshold;   // trigger threshold (default 2)
  int emergency_break_active;     // 1 = BREAK sent, waiting for ACK
  int emergency_break_retries;    // retries left for current BREAK attempt
  int emergency_previous_config;  // config that was failing
  int break_drop_step;            // ladder steps to drop. Doubles on each BREAK
                                  // (1,2,4,8,16,...); resets to 1 on data success
                                  // (arq_commander.cc:3074). Uncapped 2026-05-24
                                  // so the ladder reaches ROBUST_0 in 4-5 cycles
                                  // regardless of starting config.
  int breaks_since_last_data_success;  // BREAK panic-mode counter. Incremented on
                                       // each BREAK trigger (arq_commander.cc:3039),
                                       // reset to 0 on data success (line 3074).
                                       // When >= 2 (second BREAK with no data
                                       // between), break_drop_step is force-set
                                       // high enough to jump straight to ROBUST_0
                                       // instead of walking the ladder.
  int break_recovery_phase;       // 0=off, 1=coord at ROBUST_0, 2=probing target
  int break_recovery_retries;     // probe attempts remaining (2 total)
  int ceiling_success_count;      // consecutive successful blocks at ceiling (for ceiling recovery)
  int break_detected;             // YES if BREAK pattern detected by responder
  int hail_detected;              // YES if HAIL beacon detected (responder LISTENING)
  int hail_sent;                  // YES if commander has sent HAIL in current CONNECTING phase

  int ptt_on_delay_ms;
  int ptt_off_delay_ms;
  int pilot_tone_ms;   // Duration of pilot tone before OFDM (0=disabled)
  int pilot_tone_hz;   // Frequency of pilot tone (250=out of band, 1500=in band)
  double time_left_to_send_last_frame;

  int disconnect_requested;

  int connection_attempts;
  int max_connection_attempts;

  int exit_on_disconnect;
  int had_control_connection;
  bool passive_monitor;  // Third-party monitor mode: accept all frames, never TX
  bool monitor_stdout;   // Output decoded plaintext to stdout (headless monitor)
  int monitor_consec_ofdm_fail{0};  // Consecutive OFDM decode failures (for opportunistic scan)

  // Parallel monitor decoders — one cl_telecom_system per OFDM config.
  // All share the same-sized audio buffer (buffer_Nsymb_min override).
  // Decode attempts run on parallel threads, one per config.
  cl_telecom_system* monitor_decoders[NUMBER_OF_CONFIGS];
  bool monitor_decoders_ready{false};
  int monitor_primary_buffer_nsymb{0};  // buffer_Nsymb of largest config (for sizing)
  int monitor_decoded_data[N_MAX / 8];  // Staging buffer: decoded data_byte saved here
  int monitor_decoded_len{0};            // Frame size in ints (from winning decoder)
  void init_monitor_decoders();
  void reinit_monitor_decoders();  // Re-create decoders after NB/WB switch
  int parallel_monitor_decode(double* audio, int audio_len,
                              st_receive_stats& out_stats);

  // GUI measurement getters
  double get_snr_uplink() const { return measurements.SNR_uplink; }
  double get_snr_downlink() const { return measurements.SNR_downlink; }

  // Phase 3a (Effective-Rate Optimizer) — measurement plumbing.
  // Rolling window of last OPTIMIZER_WINDOW_SIZE batches' per-batch stats.
  // CMD-side only; populated by arq_commander.cc. Decision logic (Phase 3c)
  // reads via these public getters — no policy lives here.
  //
  // See: mercury/fact-documents/EFFECTIVE_RATE_OPTIMIZER_DESIGN.md §4.1
  static const int OPTIMIZER_WINDOW_SIZE = 50;

  double get_current_effective_rate_bps() const {
      if (opt_window_count == 0) return 0.0;
      unsigned long long total_bytes = 0;
      unsigned long long total_ms = 0;
      for (int i = 0; i < opt_window_count; i++) {
          int idx = (opt_window_head - 1 - i + OPTIMIZER_WINDOW_SIZE) % OPTIMIZER_WINDOW_SIZE;
          total_bytes += opt_batch_bytes_delivered[idx];
          total_ms += opt_batch_wire_ms[idx];
      }
      if (total_ms == 0) return 0.0;
      return (double)total_bytes * 8000.0 / (double)total_ms;
  }
  double get_current_sack_rate() const {
      if (opt_window_count == 0) return 0.0;
      int sack_count = 0;
      for (int i = 0; i < opt_window_count; i++) {
          int idx = (opt_window_head - 1 - i + OPTIMIZER_WINDOW_SIZE) % OPTIMIZER_WINDOW_SIZE;
          sack_count += opt_batch_sack_count[idx];
      }
      return (double)sack_count / (double)opt_window_count;
  }
  int get_current_window_count() const { return opt_window_count; }

  // Phase 3a helpers — invoked from CMD-side TX/ACK/BREAK sites in
  // arq_commander.cc. Defined inline to avoid an extra .o churn for what is
  // a thin instrumentation layer; no decision logic lives in any of them.
  unsigned long long opt_now_ms() const {
      return (unsigned long long)
          std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  // Called at the cmd_batch_tx_start instrumentation point. If a previous
  // batch already published its slot, back-fill its wire_ms with the cycle
  // delta. Always refresh opt_batch_tx_start_ms to the current wall-clock so
  // the next batch can do the same.
  void opt_on_batch_tx_start() {
      unsigned long long now = opt_now_ms();
      if (opt_batch_tx_start_ms != 0 && opt_window_count > 0) {
          int prior = (opt_window_head - 1 + OPTIMIZER_WINDOW_SIZE) % OPTIMIZER_WINDOW_SIZE;
          unsigned long long delta = now - opt_batch_tx_start_ms;
          opt_batch_wire_ms[prior] = (unsigned int)delta;
      }
      opt_batch_tx_start_ms = now;
  }
  // Advance the ring head with one slot describing the just-finished batch.
  // bytes_delivered is the number of payload bytes the receiver actually got
  // (zero for failed batches). sack_used = 1 iff this batch closed via
  // SACK_RSP (partial recovery), 0 if via OFDM_ACK_CLEAN / ACK_PAT / fallback
  // ACK / failure. failed = 1 iff the batch dropped without any ACK / BREAK
  // fired against it.
  void opt_record_batch(unsigned int bytes_delivered,
                        bool sack_used,
                        bool failed) {
      int slot = opt_window_head;
      opt_batch_bytes_delivered[slot] = bytes_delivered;
      // wire_ms is not known yet — the NEXT batch's tx_start back-fills it.
      // Seed with 0 so a never-back-filled trailing slot contributes nothing
      // to the rate sum (defensive: get_current_effective_rate_bps() also
      // checks total_ms == 0).
      opt_batch_wire_ms[slot] = 0;
      opt_batch_sack_count[slot] = sack_used ? 1 : 0;
      opt_batch_failed[slot] = failed ? 1 : 0;
      opt_batch_config[slot] = (unsigned char)current_configuration;
      opt_window_head = (opt_window_head + 1) % OPTIMIZER_WINDOW_SIZE;
      if (opt_window_count < OPTIMIZER_WINDOW_SIZE) opt_window_count++;
      opt_diag_emit_counter++;
      // Emit every 3 batches (was 10) so short or lossy sessions still produce
      // wire-bps samples for downstream harnesses. The previous threshold of
      // 10 meant sessions that hit BREAK or ended <10 batches in produced
      // ZERO [OPT-WINDOW] lines, forcing the harness to fall back to TCP
      // rx_bytes — which is the SAME counter the harness uses for the
      // user-facing throughput field, making the wire-vs-user ratio always
      // 1.00× by construction. See Phase 4 v1 wgn18/mpp18/mpm18 false-alarm.
      if (opt_diag_emit_counter >= 3 && opt_window_count > 0) {
          opt_diag_emit_counter = 0;
          printf("[OPT-WINDOW] eff_bps=%.0f sack_rate=%.2f window_n=%d cfg=%d\n",
              get_current_effective_rate_bps(),
              get_current_sack_rate(),
              opt_window_count,
              current_configuration);
          fflush(stdout);
      }
  }
  // Phase 3c (Effective-Rate Optimizer) — decision logic owner. The class
  // is self-contained; cl_arq_controller holds it as a member and queries
  // `evaluate()` once per batch-end on CMD. Inert until load() succeeds.
  // See: include/datalink_layer/rate_optimizer.h
  //      mercury/fact-documents/EFFECTIVE_RATE_OPTIMIZER_DESIGN.md §4.3
  cl_rate_optimizer rate_opt;

  // Phase 2 Step 5 — 2D channel-state → optimal-config lookup. Held here so
  // the commander's per-batch [CHANNEL-STATE] log site can also emit a
  // [CHANNEL-LOOKUP] proposal line. CLI-opt-in via --channel-lookup <path>;
  // when unloaded, lookup() returns SENTINEL_NO_DATA and the commander
  // emits no [CHANNEL-LOOKUP] line. OBSERVATION ONLY in Step 5 — the
  // proposal is never acted on.
  // See: include/datalink_layer/channel_state_lookup.h
  //      mercury/fact-documents/channel-state-2d-lookup.md §8 Step 5
  cl_channel_state_lookup channel_lookup;

  // Called once at mercury startup (from main.cc / cl_arq_controller::init
  // wherever capabilities are negotiated) to load the calibration table.
  // Path is `mercury/effective_rate_table.json` relative to the working dir;
  // override via MERCURY_RATE_TABLE env var. Optimizer auto-disables on miss.
  void opt_load_rate_table();

  // Called at every batch-end tick on CMD (from the three opt_record_batch
  // success sites in arq_commander.cc). Runs the gate check + lookup; if
  // the optimizer wants to switch, sets `*out_recommended_cfg` to the new
  // config and returns true. Caller queues SET_CONFIG via the standard path.
  // Returns false otherwise (and out param is left at current cfg).
  //
  // Gating (per §4.3 + spec):
  //   - sack_v2_enabled must be true (caller may double-gate)
  //   - turboshift_active must be false
  //   - emergency_break_active must be 0
  //   - link_status must be CONNECTED (caller's responsibility)
  //
  // Cooldown ticking happens HERE, regardless of decision — so the counter
  // drains on every batch.
  bool opt_evaluate_batch_end(int* out_recommended_cfg);

  // Zero the rolling window. Called from reset_session_state() on link
  // disconnect — prior-session stats describe a different channel and would
  // mislead the optimizer.
  void opt_reset_window() {
      for (int i = 0; i < OPTIMIZER_WINDOW_SIZE; i++) {
          opt_batch_bytes_delivered[i] = 0;
          opt_batch_wire_ms[i] = 0;
          opt_batch_sack_count[i] = 0;
          opt_batch_failed[i] = 0;
          opt_batch_config[i] = 0;
      }
      opt_window_head = 0;
      opt_window_count = 0;
      opt_batch_tx_start_ms = 0;
      opt_diag_emit_counter = 0;
  }

private:
  int nMessages;
  struct st_message* messages_tx;
  struct st_message* messages_rx;


  struct st_message* messages_batch_ack;


  std::string my_call_sign;
  std::string user_command_buffer;


  st_stats stats, last_transmission_block_stats;
  st_measurements measurements;

  int nResends;

  char get_configuration(double SNR);
  void load_configuration(int configuration, int level, int backup_configuration);
  void switch_narrowband_mode(int nb_enabled);
  void return_to_last_configuration();
  int init_messages_buffers();
  int deinit_messages_buffers();
  void check_buffer_canaries(const char* caller);

  char last_received_message_sequence;
  int last_received_end_of_batch_seq;  // End-of-batch flag: seq# of frame with bit 7 set, or -1
  char last_message_sent_type;
  char last_message_sent_code;

  char last_message_received_type;
  char last_message_received_code;

  int data_ack_received;
  int repeating_last_ack;

  // Phase 3a (Effective-Rate Optimizer) — rolling-window storage. CMD-side
  // only; RSP never touches these. Sized at OPTIMIZER_WINDOW_SIZE (=50).
  // All values initialized in cl_arq_controller() / opt_reset_window().
  unsigned int  opt_batch_bytes_delivered[OPTIMIZER_WINDOW_SIZE];
  unsigned int  opt_batch_wire_ms[OPTIMIZER_WINDOW_SIZE];
  unsigned char opt_batch_sack_count[OPTIMIZER_WINDOW_SIZE];
  unsigned char opt_batch_failed[OPTIMIZER_WINDOW_SIZE];
  unsigned char opt_batch_config[OPTIMIZER_WINDOW_SIZE];
  int opt_window_head;
  int opt_window_count;
  unsigned long long opt_batch_tx_start_ms;
  int opt_diag_emit_counter;

  // Phase 3c — deferred optimizer-recommended config switch. Set by
  // opt_evaluate_batch_end() at the SACK_RSP / clean-ACK / fallback-ACK
  // success sites; consumed at the next TRANSMITTING_DATA dispatch tick
  // which queues SET_CONFIG via the standard add_message_control() path.
  // Sentinel value -1 means "no pending switch". Cleared after the switch
  // request is enqueued OR if the link becomes ineligible (BREAK fires,
  // turboshift resumes, session ends).
  int opt_pending_switch_cfg;

};


#endif
