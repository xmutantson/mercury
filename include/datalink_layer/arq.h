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
#include "common/sim_clock.h"
#include <unistd.h>
#include <cstdint>
#include <vector>
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

// ---------------------------------------------------------------------------
// TX-path blocking-wait helpers (defined in arq_common.cc, external linkage).
//
// These are the two spin-loop shapes that recur across the send path:
//   ptt_busy_wait(t, delay_ms) — block until virtual/wall time crosses delay_ms
//   drain_playback_wait()      — block until the playback ring is fully drained
//
// Declared here (rather than re-forward-declared per .cc) so EVERY ARQ
// translation unit routes through the SAME definition — in particular the
// SWITCH_ROLE PTT-off wait in arq_responder.cc, which previously open-coded an
// empty-body busy-spin that hard-deadlocks under the single-thread virtual
// clock (single-process-sim-refactor.md §5.6(a) / §5.7-B6). Behavior is
// byte-identical on every production / two-process-paced-sim path (the
// step-pump hook inside these helpers is null unless -m SIM_INPROC installs
// it); routing through them only makes those waits step-pumpable.
void ptt_busy_wait(cl_timer& t, int delay_ms);
void drain_playback_wait();

// ---------------------------------------------------------------------------
// SIM_INPROC settle-wait helpers (single-process-sim-refactor.md §5.7 / §7).
//
// arq_sim_inproc_active() — true iff the -m SIM_INPROC step-pump is installed
// (i.e. g_sim_inproc_pump != nullptr). On EVERY production path and the
// two-process paced sim it returns false, so the gated branches below take the
// verbatim wall-clock body. The pump is installed ONLY by the SIM_INPROC
// stepper (arq_commander.cc test_sim_inproc), so this is the authoritative
// "are we the single-thread in-process stepper?" query.
//
// pumped_settle_wait(wait_ms) — virtual-clock-ify a wall settle-wait. When the
// pump is NOT installed it is byte-identical to msleep(wait_ms) (production +
// paced sim). When the pump IS installed it runs a cl_timer + step-pump loop
// with the SAME exit predicate (elapsed >= wait_ms), advancing the shared
// virtual clock through the pump so a peer instance sees time pass. The exit
// SEMANTICS are unchanged — only the clock-advance mechanism differs. Used for
// the B1-B4 / B7 turnaround + HAIL-race settle guards. For B7 the CALLER keeps
// the delay FORMULA verbatim (Bug #55 HAIL reliability); this helper only
// routes the already-computed wait_ms through the pump.
//
// sim_inproc_rx_mute_settle(wait_ms) — gate-off the RX_MUTE drain guard (B5 /
// ADD-ON1). The msleep there waits for ASYNC AUDIO CALLBACKS to drain before
// circular_buf_reset(); under SIM_INPROC the single-thread stepper owns RX —
// there is NO async audio thread, nothing is in flight — so the drain wait is
// MOOT and becomes a no-op. The instantaneous circular_buf_reset() that
// follows the caller keeps verbatim. On production/paced-sim it is the verbatim
// msleep(wait_ms).
bool arq_sim_inproc_active();
void pumped_settle_wait(int wait_ms);
void sim_inproc_rx_mute_settle(int wait_ms);

// SIM_INPROC TCP-poll gate (single-process-sim-refactor.md §10.5). Set true ONLY
// while the 2-instance stepper runs; makes process_main() skip its TCP control +
// data poll blocks (the stepper injects commands + data directly). Default false
// → production + paced sim run the verbatim blocks (byte-identical).
void arq_set_sim_inproc_skip_tcp(bool on);
bool arq_sim_inproc_skip_tcp();

// SIM_INPROC post-delivery spin-abort (fact-documents/SIMFTR_ROOTCAUSE.md §7 fix #1).
// Set true by the 2-instance step-pump (sim_inproc_pump_2) the instant the RESPONDER
// holds the full payload, so the COMMANDER's post-transfer idle-keepalive ↔ pumped-
// wait spin (which otherwise never returns control to the outer stepper) unwinds: the
// spin helpers (ptt_busy_wait / drain_playback_wait / pumped_settle_wait) early-out,
// process_main returns, and the outer loop's delivery check fires. Double-gated on
// (pump installed) AND (this flag) — both false on every production / paced-sim path,
// so it is a no-op off the SIM_INPROC stepper. Reset at the start of each stepper run.
void arq_set_sim_inproc_deliver_done(bool on);
bool arq_sim_inproc_deliver_done();

// SIM_INPROC OUTER-STEPPER gate (sim2-stepper-rewrite Phase b). Set true ONLY while
// the 2-instance OUTER-loop stepper (MERCURY_SIM2_STEPPER=outer) is driving; makes
// drain_playback_wait() QUEUE-and-return (no spin-drain) because the outer loop is the
// SOLE DAC-drain driver (one symbol/instance/iter, both directions, like the two real
// audio threads). The TX symbols are queued by tx_transfer and drained by the outer
// loop on subsequent iters, so no TX site needs to block. Default false → production +
// paced sim + the LEGACY pump stepper keep the verbatim blocking drain (byte-identical;
// gated separately from arq_sim_inproc_active() so the legacy pump path is unaffected
// until Phase d removes it). See data-flow-sim2-ofdm-delivery-cadence.md §10.
void arq_set_sim_inproc_outer_stepper(bool on);
bool arq_sim_inproc_outer_stepper_active();

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
  // R038 (race audit 2026-06-06) — per-frame v2 EOB staging consumed by the
  // match-current storage block; declared near last_received_end_of_batch_seq.
  // R035 (race audit 2026-06-06) — called from the set_data_batch_size()
  // chokepoint when the data batch SHRINKS while an RSP prev-batch is active.
  // Re-derives rsp_prev_batch_{received,expected}_count against the NEW (smaller)
  // batch so the live prev-completion gate stays reachable, and -- if any
  // already-RECEIVED prev slot is orphaned in [new_batch, old_batch) -- fires the
  // streaming desync defense BEFORE the inevitable stale-discard (which FREEs
  // messages_rx_prev[] without a streaming_reset, the asymmetry vs the delivery
  // leg). `new_batch` is the post-clamp value about to be stored. See
  // data-flow-arq-recovery-cluster.md §4.3 / §5.2.
  void rescan_prev_on_batch_shrink(int new_batch);
  // R029 (race audit 2026-06-06) — the SINGLE owner of zeroing the TX retransmit
  // queue. The retransmit_frames[] / retransmit_count parallel arrays hold frames
  // captured (and, under encryption, byte-encoded) for the LIVE crypto epoch +
  // current bsi window. Every messages_tx[]-freeing recovery site (watchdog,
  // gearshift-down, BREAK, config-change re-encode, reset_session_state,
  // restore_tx_from_compressed) re-queues PLAINTEXT to fifo_buffer_tx so it
  // re-sends under the NEW epoch — but historically NONE of them cleared
  // retransmit_count, leaving stale OLD-epoch/foreign-bsi frames that the v2
  // mixbatch builder (which has no epoch guard) would prepend to the first
  // post-recovery batch. clear_retx_queue() restores INV-R029. Mirrors the
  // runaway-BREAK clear (arq_commander.cc:1502). If R030 later adds a separate
  // retx-prefix structure it MUST be zeroed here too (single-owner contract).
  // See data-flow-arq-recovery-cluster.md §2.2 / §4.1 / §5.1.
  void clear_retx_queue();
  // R030 (race audit 2026-06-06) — resolve the messages_tx[] slot the post-TX
  // PENDING_ACK flip must mark for messages_batch_tx[batch_idx], or -1 to skip.
  // Shared by send_batch()'s flip and the --test-v2-pendingack-flip-alias test.
  // See data-flow-arq-recovery-cluster.md §4.2 / §5.5.
  int v2_flip_resolve_slot(int batch_idx);
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

  // BLOCK-CRC (D2_BLOCKCRC, fix/bigblock-d3-carve): CRC-32 over `nItems` bytes,
  // reflected (LSB-first) IEEE 802.3 — polynomial 0xEDB88320, init 0xFFFFFFFF,
  // final XOR 0xFFFFFFFF. The whole-block integrity anchor stacked on top of the
  // per-codeword CRC-8 (datalink_defines.h BIGBLOCK_BLOCK_CRC_*). Catches a K-block
  // the 8-bit per-cw gates false-pass; on mismatch the block routes to PARTIAL/SACK
  // and is never delivered.
  uint32_t CRC32_calc(const char* data_byte, int nItems);

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

  // REAL FAST-PROBE follow-up (gearshift-climb-engine.md §18) — the SNR-decode
  // arm MUST be FALSE on every data-ACK wait. §14 (A1) widened the arm
  // (turbo_snr_ack_armed_for_gearshift) to fire on a steady-state +1 gearshift
  // SET_CONFIG; on the common +1 step the SET_CONFIG-ACK apply takes the
  // steady-state re-trigger `else` branch (arq_commander.cc:4702), the
  // re-trigger does NOT fire (gap<3), and connection_status=TRANSMITTING_DATA
  // (:4753) WITHOUT clearing the flag — so the next DATA ACK was decoded with the
  // arm stale-true and a SACK-suffixed data ACK polluted measurements.SNR_uplink.
  // finish_turbo_direction() (:3792, the only non-init clearer) is NOT on the
  // steady-state FRAME-UP path, so the original "guard (3)" no longer holds for
  // the widened arm. This single chokepoint restores the invariant: it clears the
  // arm at EVERY entry into a data-ACK wait. All three RECEIVING_ACKS_DATA entries
  // call it (the data-TX setups arq_commander.cc:~1317 / :~1813 and the
  // REPEAT_LAST_ACK re-wait :~1148); every TRANSMITTING_DATA transition funnels
  // through process_messages_tx_data() before any data-ACK wait, so the data-TX
  // setups cover them all, and the REPEAT_LAST_ACK site covers the one direct
  // entry that bypasses process_messages_tx_data(). Called AFTER the control-ACK
  // suffix is already decoded in process_messages_rx_acks_control() (:1855), so
  // A1's mid-climb SNR decode is PRESERVED — only the post-control, pre-data clear
  // is added. Part N drives this exact method. See §18.
  void clear_snr_arm_for_data_ack_wait() { turbo_snr_ack_enabled = false; }

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

  // FIX-8 (data-integrity): advance rsp_last_delivered_batch_seq_id to `bsi`
  // ONLY if `bsi` is a forward step (mod-256 forward distance in [1,128]) from
  // the current mark, OR the mark is unset (-1). MONOTONIC-with-wrap (audit R1):
  // a late out-of-order prev (older) batch delivered after a newer current
  // already advanced the mark must NOT regress it. Called at the two real
  // delivery commits only (BATCH-DONE + PREV-DELIVERED). Internally v2-scoped by
  // the call sites; the helper itself is pure arithmetic.
  void advance_last_delivered(int bsi);

  // D3.1 (data-integrity): the shared LOUD GAP-ABORT teardown. The
  // case-independent action both the FIX-8 re-adopt gate and the new
  // delivery-time gate route to: control-port error, DROPPED, clear the bsi
  // family + prev-buffer + carve-arm, reset_session_state(). Centralizes the
  // block previously inlined at arq_responder.cc:646-688 so the delivery-time
  // commits and the SET_CONFIG re-baseline path use the IDENTICAL teardown (no
  // drift between sites). `reason` is the control-port error string.
  // See bigblock_p3_hw/_d31_fade/D31_INORDER_DESIGN.md §2.
  void rsp_gap_abort_teardown(const char* reason);

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

  // DEEP-SNR DOWN-HYSTERESIS (gearshift-climb-engine.md §10) — PURE anchor-DEMOTION
  // decision. Given the current anchor, the consecutive anchor-rung BREAK-fail
  // count, and robust_enabled, return the anchor AFTER this BREAK: if the count has
  // reached K (ANCHOR_DEMOTE_BREAK_FAILS) lower the anchor one rung
  // (config_ladder_down); otherwise return it unchanged. ONLY ever lowers (or holds
  // at the ladder floor) — never raises — so the +1 up-clamp gets stricter, never
  // looser. No side effects (the caller resets the counter); the unit test drives
  // this directly to prove fail-before/pass-after. See §10.
  int anchor_demote_target(int anchor, int consec_anchor_break_fails,
                           bool robust_en) const
  {
    if(consec_anchor_break_fails >= ANCHOR_DEMOTE_BREAK_FAILS)
      return config_ladder_down(anchor, robust_en);
    return anchor;
  }

  // Option B synthetic-fire test (CLI --test-data-anchored-promote). Drives the
  // real BREAK-floor helper and the real policy_evaluate_axis1() up-shifter with
  // last_data_viable_config primed, asserting the link parks at the anchor rung
  // instead of climbing/falling past it. Returns 0 on pass, 1 on fail. Default
  // builds never call this. See fact-documents/gearshift-start-and-recovery.md §6.4.
  int test_data_anchored_promote();

  // FIX-B — FLOOR-PROBE BACK-OFF synthetic-fire test (CLI --test-probe-backoff).
  // Drives the REAL arm/gate/reset/predicate machinery (no channel) for the five
  // PB cases (gearshift-floor-probe-backoff.md §7): PB1 FAIL-BEFORE/PASS-AFTER
  // (armed rung is suppressed AND the UP gate blocks; reverting the conjunct
  // fails), PB2 virtual-clock elapse lifts suppression (via sim_clock), PB3
  // double-arm exponential+cap, PB4 reset zeroes, PB5 INV-2 (panic/demote targets
  // unchanged with back-off armed). Returns 0 on pass, 1 on fail. Default builds
  // never call this. See fact-documents/gearshift-floor-probe-backoff.md §7.
  int test_probe_backoff();

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

  // SUSTAINED-ANCHOR GATE (gearshift-climb-engine.md §11) — the number of
  // CONSECUTIVE clean batches a rung must deliver before it may RAISE the anchor.
  // Robust (batch=1): one clean MFSK frame is strong proof (keep the off-ROBUST_0
  // climb fast). OFDM: two — a single SACK-retransmit-rescued batch can't anchor a
  // non-sustainable rung (this is the WGN:-10 thrash leak). PURE predicate (no
  // side effects) so the unit test can drive it directly. config is the rung being
  // evaluated (current_configuration in production).
  static int sustained_anchor_threshold(int config)
  { return is_robust_config(config) ? SUSTAINED_ANCHOR_N_ROBUST
                                     : SUSTAINED_ANCHOR_N_OFDM; }

  // ANCHOR-TIER-CROSSING DISCIPLINE (gearshift-climb-engine.md §16) — the PURE
  // policy that decides the new value of last_data_viable_config from a CONFIRMED
  // clean batch. It is the SOLE on-delivery anchor-raise decision (production calls
  // it at arq_commander.cc:3617; the unit test replays it directly). ROOT-1 of the
  // anchor-tier-corruption fix: the anchor must reflect delivery PROVEN PER TIER, and
  // it must be credited to the config the batch was ACTUALLY DELIVERED at — NOT a
  // config the same-pass / cross-pass machinery has since advanced to.
  //   streak_config    = clean_batches_config — the config at which the clean STREAK
  //                      accumulated (the SOLE authoritative "delivered config"; §11
  //                      pins it to current_configuration WHEN the cleans land, so it
  //                      survives a later config advance even if current_configuration
  //                      has moved on). Using THIS, not the live current_configuration,
  //                      is the "capture the delivered config before the advance" fix.
  //   current_anchor   = last_data_viable_config (the af14a9e anchor).
  //   clean_streak     = clean_batches_at_current_config (consecutive cleans AT
  //                      streak_config; §11 producer).
  // Returns the (possibly raised) anchor. RULES:
  //   1. Never LOWERS the anchor (a candidate not strictly higher is a no-op —
  //      demotion is §10's separate producer).
  //   2. A higher streak_config needs >= sustained_anchor_threshold(streak_config)
  //      consecutive cleans (§11 — robust N=1, OFDM N=2).
  //   3. THE TIER GATE (§16 ROOT-1): the anchor may CROSS from a ROBUST anchor into
  //      the OFDM tier ONLY when the clean STREAK accumulated AT an OFDM config
  //      (is_ofdm_config(streak_config)). A ROBUST fragment-ACK can advance the anchor
  //      WITHIN robust (ROBUST_0->1->2) but can NEVER push it past the top ROBUST rung
  //      into CONFIG_0+. This makes the producer STRUCTURALLY incapable of seating an
  //      OFDM anchor on robust evidence — closing the deep-SNR over-climb at its root
  //      (a ROBUST-tier clean ACK can no longer poison the anchor into the OFDM tier,
  //      which is what unlocked the §15 elevator gate + the §16 ROOT-2 turbo ladder at
  //      WGN:-10). PURE; no side effects.
  static int data_anchor_raise_target(int streak_config, int live_config,
                                      int current_anchor, int clean_streak)
  {
    // Rule 2: enough consecutive cleans AT the streak's home rung (§11).
    if(clean_streak < sustained_anchor_threshold(streak_config))
      return current_anchor;
    // Rule 1: only ever RAISE — a candidate at or below the anchor is a no-op
    // (demotion is §10's separate producer).
    if(config_ladder_index(streak_config) <= config_ladder_index(current_anchor))
      return current_anchor;
    // THE §16 ROOT-1 TIER GATE: the anchor may CROSS from a ROBUST anchor into the
    // OFDM tier ONLY when the clean streak was earned AT an OFDM config. We credit
    // streak_config (the cleans' HOME — the authoritative "delivered config"), NOT
    // the live current_configuration. The pre-§16 producer credited the LIVE config:
    // if a ROBUST-tier batch's clean credit fired while current_configuration had
    // already crossed to CONFIG_0 (e.g. a late/duplicate robust ACK arriving after
    // the FRAME-UP SET_CONFIG advanced the live config), the anchor was poisoned to
    // CONFIG_0 on robust evidence — which then unlocked the §15 elevator gate AND the
    // §16 ROOT-2 turbo ladder. Crediting streak_config closes that: a ROBUST-tier
    // delivery has a ROBUST streak_config, so streak_config can only seat a ROBUST
    // anchor and NEVER crosses the boundary. The is_robust(current_anchor) &&
    // is_ofdm(streak_config) crossing therefore requires a genuine OFDM streak_config
    // (N_OFDM cleans AT that OFDM config via Rule 2). The explicit assertion below is
    // belt-and-suspenders: if a caller ever passed a streak_config that DISAGREES with
    // the live tier on a ROBUST->OFDM cross (the corruption signature), refuse it.
    // It does NOT block a within-ROBUST advance (ROBUST_1->ROBUST_2): there
    // streak_config is robust, is_ofdm_config(streak_config) is false, so the cross
    // condition is false and the advance proceeds.
    bool crossing_robust_to_ofdm =
      is_robust_config(current_anchor) && is_ofdm_config(streak_config);
    if(crossing_robust_to_ofdm && !is_ofdm_config(live_config))
      return current_anchor;   // live tier disagrees with the streak's OFDM claim
    return streak_config;
  }

  // ADAPTIVE FRAME-UP THRESHOLD (gearshift-climb-engine.md §12, Option 3, climb
  // follow-up ③) — the clean-streak evidence bar that ARMS fast-probing at a rung.
  // Reuses the SAME viability bar §11 uses to ANCHOR a rung
  // (sustained_anchor_threshold): fast-stepping may fire ONLY at a rung already
  // proven viable enough to anchor — so a marginal / deep-SNR-cliff rung (where
  // every failed block resets clean_batches_at_current_config to 0) NEVER goes
  // fast, preserving #2's (e3d818d) WGN:-10 anti-thrash. PURE; the unit test (Part
  // I) replays it directly. config = current_configuration. TUNABLE (it IS the
  // anchor bar today; split the constant if the two bars ever need to differ).
  static int fast_probe_clean_streak(int config)
  { return sustained_anchor_threshold(config); }

  // FIX-A — ROBUST-tier dwell-batch eligibility (data-flow-robust-tier-arq-batch.md
  // §5.1). TRUE iff a ROBUST dwell may SAFELY run batch > 1 (the FIX-A relaxation).
  // The robust batch is pinned to 1 by default because at the MFSK cliff
  // P(batch clean)=p^N and only batch=1 makes the strict all-ones clean target
  // achievable WHILE THE CLIMB IS STILL EARNING THE RUNG (the central tension /
  // landmine L8 — lifting batch while the climb owns the rung freezes the anchor
  // and is the literal Bug-3 dormancy). This predicate lifts the pin ONLY when the
  // rung is PROVEN and the climb is PARKED (not actively probing up), so the p^N
  // penalty is acceptable and the M=16 MFSK SACK suffix patches any partial.
  //
  // PURE core — takes every input explicitly so the unit test (Parts D'1/D'3) can
  // drive it with no live telecom_system / channel. ALL conjuncts must hold:
  //  (a) live PHY is a ROBUST config            — is_robust_config(current_cfg)
  //  (b) WB session (SACK suffix exists)        — suffix_capable
  //        (NB: M=8 => ack_sack_suffix_len()==0 => NO bitmap => a multi-frame
  //         partial is UNRECOVERABLE => MUST stay batch=1; landmine L2 / OR guard b)
  //  (c) this rung is PROVEN delivered          — ladder_idx(current) <= ladder_idx(anchor)
  //        (the anchor has reached/passed this rung => clean batch(es) already
  //         confirmed here at batch=1; the climb has earned this rung)
  //  (d) the clean streak is ESTABLISHED + parked HERE —
  //        streak_cfg == current_cfg && clean_streak >= ROBUST_DWELL_PROOF_BATCHES
  //        (proves we are PARKED on a SUSTAINED-clean robust rung, not transiently)
  //  (e) PARKED at the proven ceiling, NOT actively climbing up —
  //        proven_ceiling >= 0 && ladder_idx(current) >= ladder_idx(proven_ceiling)
  //        (OR-1 / L8 LOAD-BEARING: if a higher rung is still reachable+unproven the
  //         climb still OWNS the batch — keep 1 so the strict clean credit keeps
  //         advancing the anchor. proven_ceiling<0 means no ceiling has been bounded
  //         yet => the link is NOT established at a parked ceiling => keep 1.)
  static bool robust_dwell_batch_eligible_core(
      int current_cfg, int anchor, int streak_cfg, int clean_streak,
      int proven_ceiling, bool suffix_capable)
  {
    if(!is_robust_config(current_cfg)) return false;                  // (a)
    if(!suffix_capable) return false;                                 // (b)
    if(config_ladder_index(current_cfg)
       > config_ladder_index(anchor)) return false;                   // (c) rung not proven
    if(streak_cfg != current_cfg) return false;                       // (d) streak not here
    if(clean_streak < ROBUST_DWELL_PROOF_BATCHES) return false;       // (d) streak too short
    if(proven_ceiling < 0) return false;                              // (e) no parked ceiling yet
    if(config_ladder_index(current_cfg)
       < config_ladder_index(proven_ceiling)) return false;           // (e) higher rung reachable
    return true;
  }

  // Member wrapper — supplies the live climb state + the WB/NB suffix capability
  // from the dedicated config-independent ack_mfsk (M=16 WB => suffix_len==13;
  // NB M=8 => 0). CMD-only state (clean_batches_*, supershift_proven_ceiling), so
  // ONLY the CMD evaluates this; the RSP mirrors the resulting batch via the
  // dedicated ROBUST_DWELL_BATCH_OP transport (it has no climb state — identical to
  // how Axis-2's batch decision is CMD-only / RSP-applied). PURE (const).
  bool robust_dwell_batch_eligible() const {
    bool suffix_capable = (telecom_system != NULL)
      && (telecom_system->ack_mfsk.ack_sack_suffix_len() > 0);
    return robust_dwell_batch_eligible_core(
      current_configuration, last_data_viable_config,
      clean_batches_config, clean_batches_at_current_config,
      supershift_proven_ceiling, suffix_capable);
  }

  // ADAPTIVE FRAME-UP THRESHOLD (gearshift-climb-engine.md §12) — the EFFECTIVE
  // threshold the FRAME-UP comparison (arq_commander.cc:3637) uses, computed at
  // READ time so it NEVER mutates / caps the frame_shift_threshold member (which
  // the AARF back-off DOUBLES on FRAME-UP failure at :2302/:3180/:3359 and the
  // logs print). When the rung has PROVEN sustained-clean delivery
  // (clean_streak_at_config >= fast_probe_clean_streak(config)) return the FAST
  // value (FRAME_SHIFT_FAST=1 → step on the next clean batch); OTHERWISE return the
  // base_threshold member UNCHANGED (the conservative 3, or the AARF-doubled 6/12/…
  // — so the reduction does NOT fight the back-off). The back-off and this read are
  // mutually exclusive per process_main() pass (back-off is on the data_ack==NO
  // failure paths; this read is on the data_ack==YES+clean path) and a failure
  // resets clean_streak to 0 in the SAME handler, so the very next read returns the
  // freshly-doubled member, not FAST. base_threshold = frame_shift_threshold,
  // config = current_configuration, clean_streak = clean_batches_at_current_config.
  // PURE (no side effects); the unit test replays it directly. See §12.1/§12.3.
  int effective_frame_shift_threshold(int base_threshold, int config,
                                      int clean_streak_at_config) const
  {
    if(clean_streak_at_config >= fast_probe_clean_streak(config))
      return FRAME_SHIFT_FAST;
    return base_threshold;
  }

  // CFG16-acq2 (data-flow-snr-measurements.md §9) -- the SACK-trusted, REGIME-AWARE
  // climb SNR margin. PURE (the bool member + the passed SNR). The UP-climb target sites
  // that compute get_configuration(SNR - <margin>) -- elevator_target_from_snr() (the
  // FRAME-UP + re-trigger elevator) and the in-turbo SNR-SUPERSHIFT -- call THIS helper
  // (passing the SAME SNR they map) instead of the bare SUPERSHIFT_MARGIN_DB constant.
  //
  // THE DOUBLE-COUNT, only in the SATURATED regime: the responder's POST-EQ EVM-SNR
  // (ofdm.cc:2288) FLOORS at ~14.5 dB on a clean channel (channel-INDEPENDENT: WGN:40 ==
  // WGN:50, var=0.0355) and round-trips through the 4-bit MFSK suffix to a hard 15.0.
  // That is NOT a channel-SNR estimate -- it is an equalizer/pilot residual that has
  // SATURATED. The 6.0 dB AWGN fading margin is designed for an estimate that TRACKS the
  // channel; subtracting it from a SATURATED EVM number double-counts the margin and caps
  // the natural climb at CONFIG_13 (get_configuration(15-6=9)=13) even where pinned CFG16
  // 32-QAM is PROVEN viable (decisive verdict §2a). When SACK Design A is negotiated the
  // channel recovers partial-batch loss (SACK_RSP patches missing frames), so the fading
  // margin is redundant -- but ONLY where the estimate has saturated. Below the saturation
  // knee the EVM-SNR still TRACKS the channel (e.g. SNR=2.0 is a genuine marginal reading,
  // not a floor), where the fading margin is still EARNED. So the reduced margin is gated
  // on snr >= CFG16_EVM_SATURATION_KNEE_DB (13.0, == the get_configuration CFG16 boundary):
  // only an estimate that already claims CFG16-capable-by-the-table is trusted without the
  // extra margin. This is regime-aware, not a flat reduction -- FP-J3c (marginal SNR=2.0 ->
  // no spurious jump) stays BYTE-IDENTICAL even under SACK.
  //
  // OFF SACK (NB / legacy / non-SACK): full SUPERSHIFT_MARGIN_DB at every SNR -> byte-
  // identical. Over-climb safety (§15 WGN:-10): the deep-SNR (SNR~1.0) reading is BELOW
  // the knee -> full margin anyway; and even above the knee the is_ofdm_config(anchor) gate
  // in supershift_retrigger_target (anchor-gated, NOT value-gated -- §8.3) clamps to +1
  // while the anchor is ROBUST. BOTH backstops hold.
  double climb_effective_snr_margin_db(double snr) const
  {
#ifdef CFG16ACQ2_FAILBEFORE
    // FAIL-BEFORE: the pre-fix behavior -- ALWAYS the full 6.0 dB margin, so the SACK
    // climb still caps at CONFIG_13 (Part NM1 FAILs). PASS-AFTER: the gated branch below
    // returns the reduced margin in the saturated regime and the climb elects CFG16.
    (void)snr;
    return (double)SUPERSHIFT_MARGIN_DB;
#else
    if(sack_v2_enabled && snr >= (double)CFG16_EVM_SATURATION_KNEE_DB)
      return (double)SACK_CLIMB_SNR_MARGIN_DB;
    return (double)SUPERSHIFT_MARGIN_DB;
#endif
  }

  // CONTROLLED ELEVATOR (fork (1), gearshift-climb-engine.md sec 13) -- PURE
  // policy for the SUPERSHIFT re-trigger target. af14a9e HARD-CLAMPED the
  // SNR-driven re-trigger to last_data_viable_config+1 (anchor+1), which made
  // the SNR-ideal multi-rung jump a no-op and left the unpinned climb
  // wall-clock-bound to the +1 FRAME-UP ladder. (1) RELAXES that clamp ON THIS
  // PATH ONLY, under a HIGH-CONFIDENCE-SNR predicate, so at clearly-high SNR the
  // modem jumps multiple rungs toward the SNR-appropriate config in one shot.
  // Inputs:
  //   snr_ideal      = get_configuration(SNR - SUPERSHIFT_MARGIN_DB), ALREADY
  //                    capped at min(supershift_proven_ceiling, WB/NB ceiling)
  //                    by the caller (arq_commander.cc:4587-4592) -- so the
  //                    returned target can NEVER exceed proven-safe (SAFETY #2).
  //   snr_uplink     = measurements.SNR_uplink (the OptA-populated live SNR; the
  //                    caller enclosing gate already requires it > -90, but we
  //                    re-check so the helper is correct in isolation).
  //   anchor         = last_data_viable_config (the af14a9e anchor).
  //   optimizer_owns = optimizer_is_in_control() -- when the Q-table owns the
  //                    band the anchor clamp does not apply (return snr_ideal).
  // HIGH-CONFIDENCE-SNR predicate (§15 hardened): SNR valid (> -90) AND the anchor
  // is a PROVEN OFDM rung (is_ofdm_config(anchor)) AND the ceiling-capped snr_ideal
  // genuinely lands MORE than +1 rung past the anchor (else there is no multi-rung
  // jump to make -- fall through to the conservative +1 clamp, which is a NO-OP
  // there). The `is_ofdm_config(anchor)` conjunct is the §15 DEEP-SNR over-climb
  // fix: a CONTROL-plane MFSK-suffix SNR over-reports the OFDM-DATA-viable rate at
  // deep SNR (it decodes when OFDM data cannot), so `snr_uplink > -90` ALONE is a
  // FALSE premise for "safe to jump". The data-viable anchor reaching the OFDM tier
  // is the correct discriminator -- it has PROVEN the channel carries OFDM data.
  // DEEP-SNR / ROBUST-ANCHOR INERT: at the sentinel / low SNR / a ROBUST anchor the
  // predicate is FALSE and this returns EXACTLY anchor+1 (or the lower snr_ideal) --
  // BYTE-IDENTICAL to the af14a9e clamp. When the jump IS licensed it is BOUNDED to
  // anchor + RETRIGGER_MAX_LEAP (§15 secondary) so a marginal-OFDM channel cannot
  // overshoot the ladder in one shot. The landing is SPECULATIVE: this helper READS
  // the anchor but NEVER raises it (SAFETY #3 -- the anchor follows CONFIRMED clean
  // delivery only, sec 1.1 + sec 11; a failed jump -> BREAK ->
  // break_target_with_anchor recovers to the still-low anchor, sec 10 demotion
  // backstops repeated overshoot). FRAME-UP / LADDER-UP keep their own strict +1
  // clamps (config_ladder_up is inherently +1) -- UNTOUCHED (SAFETY #1). PURE (no
  // side effects); Part J / J'' replays it directly. See sec 13 / sec 15.
  int supershift_retrigger_target(int snr_ideal, double snr_uplink, int anchor,
                                  bool optimizer_owns, bool robust_en,
                                  bool narrowband) const
  {
    if(optimizer_owns)
      return snr_ideal;            // Q-table owns the band -- no anchor clamp
    int anchor_cap = config_ladder_up_n(anchor, 1, robust_en, narrowband);
    // §15 DEEP-SNR over-climb regression fix (re-assert the af14a9e data-anchor at
    // the elevator chokepoint). The pre-§15 predicate ("snr_uplink > -90 =>
    // safe to jump") admits a FALSE high-confidence jump at the deep-SNR cliff: A1
    // populates SNR_uplink from the CONTROL-plane MFSK ACK suffix, which decodes at
    // ~1 dB even when OFDM DATA cannot, so the >-90 half is satisfied AND
    // get_configuration(1.0-6.0) -> CONFIG_4 lands several rungs above a ROBUST
    // anchor's anchor_cap (CONFIG_0) -> the af14a9e +1 clamp is BYPASSED and the
    // modem jumps CFG_0->CFG_4 (then ratchets to CFG_9) on PHANTOM ACK matches with
    // NO real OFDM data and NO BREAK. The true discriminator is whether the channel
    // has PROVEN it can carry OFDM DATA -- i.e. whether the data-viable anchor has
    // reached the OFDM tier. Add `is_ofdm_config(anchor)`: at the WGN:-10 cliff the
    // anchor stays ROBUST (OFDM never delivers) -> high_confidence_jump=false -> the
    // +1 clamp re-applies -> no jump -> the data-fail BREAK path re-engages and falls
    // back to ROBUST_0 (af14a9e restored). At WGN:30 the anchor reaches CONFIG_0 once
    // clean OFDM batches deliver (§1.1 sustained-gate) -> the jump is permitted (the
    // fast multi-rung climb preserved, at most a small confirm-at-CFG_0 delay).
    bool high_confidence_jump = (snr_uplink > -90) &&
        is_ofdm_config(anchor) &&
        config_ladder_index(snr_ideal) > config_ladder_index(anchor_cap);
    if(high_confidence_jump)
    {
      // §15 SECONDARY (defense-in-depth): even once the anchor is a proven OFDM
      // rung, BOUND a single jump to anchor + RETRIGGER_MAX_LEAP so a marginal-OFDM
      // channel (CONFIG_0 holds but CONFIG_13 does not) cannot overshoot the whole
      // ladder in one shot. It leaps in bounded steps as the anchor ratchets up; the
      // proven-ceiling cap (applied by the caller) + the §10 anchor-demotion backstop
      // any residual overshoot. NEVER lowers snr_ideal below anchor_cap (it is still
      // at least a +1 move). TUNABLE via RETRIGGER_MAX_LEAP (common_defines.h).
      int leap_cap = config_ladder_up_n(anchor, RETRIGGER_MAX_LEAP, robust_en, narrowband);
      if(config_ladder_index(snr_ideal) > config_ladder_index(leap_cap))
        snr_ideal = leap_cap;
    }
    // Keep the conservative +1 clamp UNLESS the high-SNR predicate licenses the
    // (now MAX_LEAP-bounded) multi-rung jump. (At low/invalid SNR, or a ROBUST
    // anchor, the predicate is false and this clamp is the ORIGINAL af14a9e clamp
    // -> DEEP-SNR / ROBUST-ANCHOR INERT.)
    if(!high_confidence_jump &&
       config_ladder_index(snr_ideal) > config_ladder_index(anchor_cap))
      snr_ideal = anchor_cap;
    return snr_ideal;
  }

  // CFG16-acq2 D2/D3 (bigblock_p3_hw/_cfg16acqd2d3/AUDIT_AND_DESIGN.md §4) — the
  // CLAMP-TO-SELF / YIELD-TO-DATA discriminator. PURE (no member writes; the unit
  // test Part D2D3 replays it directly). ROOT CAUSE of the natural-climb CONFIG_13
  // wedge (D2+D3): in the in-turbo SUPERSHIFT branch (arq_commander.cc:5343) the
  // D1 acq-fix correctly computes the SNR target = CONFIG_16, but
  // supershift_retrigger_target RE-CLAMPS it to leap_cap = config_ladder_up_n(anchor,
  // RETRIGGER_MAX_LEAP) — and from an un-ratcheted CONFIG_0 anchor (idx 3) that
  // leap_cap = config_ladder_up_n(CONFIG_0, 13) = CONFIG_13 == the CURRENT config.
  // The code then UNCONDITIONALLY emits add_message_control(SET_CONFIG) (:5476) and
  // stays TRANSMITTING_CONTROL — a NO-OP SET_CONFIG-to-self that the peer ACKs, turbo
  // re-triggers, and the SAME 16->clamp-to-13 fires again: an infinite CONTROL spin
  // that STARVES data TX (executed: 72 "Transmitting control" vs 8 "Transmitting
  // data", ~113 B delivered, ab2/runs/fix_WGN_40_s1). No clean DATA batch flows, so
  // the delivery-gated anchor-raise (data_anchor_raise_target, arq_commander.cc:4278)
  // never fires, the anchor stays CONFIG_0, and the leap_cap stays CONFIG_13 forever.
  //
  // This predicate detects that exact clamp-to-self: the FULLY-clamped turbo target
  // (after the leap_cap + every ceiling/cooldown cap) lands AT OR BELOW the current
  // config -> there is NO forward config change to announce -> emitting a SET_CONFIG
  // is a pure no-op spin. When TRUE the caller SETTLES the turbo at the current rung
  // and YIELDS to DATA TX (the SAME terminal state the CFG16-HOLD top-config branch
  // uses, arq_commander.cc:5515-5543) so a clean CONFIG_13 batch can DELIVER and
  // ratchet the anchor CONFIG_0 -> CONFIG_13; the next SUPERSHIFT re-trigger then has
  // leap_cap = config_ladder_up_n(CONFIG_13, 13) = CONFIG_16 (Part NM6 asserts this
  // exact site) and the climb reaches CFG16 (the H1 reverse-ACK lever then holds it).
  //
  // The caller must NOT pin supershift_proven_ceiling on this yield: the clamp is a
  // TEMPORARY leap_cap bound that lifts once the anchor ratchets — pinning a proven
  // ceiling here would forbid the later climb to CFG16. (Contrast the CFG16-HOLD
  // branch, which legitimately pins the ceiling because CFG16 IS the top.)
  //
  // BYTE-IDENTICAL on a real upshift: when the clamped target is STRICTLY ABOVE the
  // current config (a genuine config change, the normal climb), this returns FALSE
  // and the caller emits the SET_CONFIG exactly as before. The SNR-capped-step-1
  // path (negotiated = current+1) is likewise > current -> FALSE -> unchanged. The
  // branch fires ONLY on the genuine no-op clamp-to-self that is today a control spin.
  // PURE; index-only comparison (no member reads beyond the two passed configs).
  bool supershift_clamp_yields_to_data(int clamped_target, int current_config) const
  {
#ifdef CFG16ACQ2_D2D3_FAILBEFORE
    // FAIL-BEFORE: the pre-fix behavior -- NEVER yield (always emit the SET_CONFIG,
    // even on a clamp-to-self). Part D2D3-1 FAILs (the wedge is not broken).
    (void)clamped_target; (void)current_config;
    return false;
#else
    return config_ladder_index(clamped_target) <= config_ladder_index(current_config);
#endif
  }

  // WALL-B FIX-7A (KEYSTONE) — turbo CEILING SETTLE-vs-BREAK discriminator
  // (fix7/FIX7_DESIGN.md §1). PURE decision (no member writes, no side effects;
  // Part U replays it directly, the same idiom as supershift_retrigger_target /
  // bigblock_carve_cooldown_ceiling). The turbo CEILING handler
  // (arq_commander.cc:2311-2388) is reached from TWO situations it today treats
  // identically (UNCONDITIONAL send_break_pattern() → ROBUST_0):
  //   (1) GENUINE top-config verification ceiling (Bug #59): the link probed the
  //       top of the proven range, no lower OFDM rung is proven, the probe failed.
  //       A BREAK-and-fall-back is CORRECT — real uncertainty about the channel.
  //   (2) SPECULATIVE RE-TRIGGER over-reach (THIS latch): the link was DELIVERING
  //       DATA at a PROVEN OFDM rung (last_data_viable_config is a real OFDM rung
  //       raised by clean batches), then the saturating post-EQ EVM-SNR licensed a
  //       MULTI-rung speculative jump to a config the channel can't carry. The
  //       probe failure proves only that the SPECULATIVE rung is unreachable — it
  //       says NOTHING bad about the proven rung the link was just running on.
  //       BREAKing to ROBUST_0 here is a pure regression: it discards proven
  //       progress and (in SIM_INPROC / a slow-wire peer) deadlocks on the
  //       un-drained BREAK-ACK tail and PARKS at ROBUST_0 on a CLEAN channel.
  // This predicate isolates case (2): only then should the CEILING SETTLE at the
  // proven anchor and resume DATA there instead of BREAKing. Inputs are exactly
  // the members live at the CEILING (arq_commander.cc:2311): the turbo phase, the
  // last-good rung (turboshift_last_good), the data-viability anchor
  // (last_data_viable_config), and the rung that failed (current_configuration).
  //   - phase == TURBO_FORWARD : a forward probe (REVERSE already returns before
  //     the forward BREAK block at :2328; the discriminator must NOT fire on it).
  //   - is_ofdm_config(settle_config): the last-good rung is a real OFDM rung
  //     (not ROBUST / not the CONFIG_NONE sentinel that maps to init_configuration).
  //   - is_ofdm_config(anchor) && !is_robust_config(anchor): the anchor PROVED an
  //     OFDM rung. At session start the anchor is the ROBUST_0 floor
  //     (arq_common.cc:722) → FALSE → INERT until a clean OFDM batch genuinely
  //     raised it (the §16 tier gate in data_anchor_raise_target prevents a ROBUST
  //     clean from poisoning the anchor into the OFDM tier). No early-session
  //     misfire (Part M/M2 already prove the t=0 anchor is ROBUST_0 on -R).
  //   - config_ladder_index(failed) > config_ladder_index(anchor) + 1: the probe
  //     leapt MORE than +1 rung past the proven anchor. A +1 FRAME-UP edge that
  //     fails is the normal ladder edge → keep today's BREAK/anchor-demote
  //     machinery (the index(failed)>index(anchor)+1 clause is the guard that
  //     leaves the genuine top-config / +1-edge ceiling BREAKing exactly as today).
  // When TRUE the caller settles INLINE to last_data_viable_config (the
  // guaranteed-decodable proven rung) and resumes DATA — NO send_break_pattern(),
  // NO finish_turbo_direction() (which on FORWARD kicks a REVERSE SWITCH_ROLE probe,
  // or with --skip-turbo-reverse re-installs proven_ceiling=turboshift_last_good,
  // fighting the ceiling-lowering — R1). When FALSE the caller falls through to the
  // BYTE-IDENTICAL BREAK. The genuine deep-SNR escape is preserved: a top-config
  // ceiling (anchor still ROBUST, no proven OFDM rung) and a +1-edge probe both
  // return FALSE → BREAK to ROBUST_0 as today. See §4 cross-layer audit.
  bool turbo_ceiling_should_settle(int phase, int settle_config,
                                   int anchor, int failed_config) const
  {
#ifdef WALLB_FIX7A_FAILBEFORE
    (void)phase; (void)settle_config; (void)anchor; (void)failed_config;
    return false;   // FAIL-BEFORE stub: no settle path exists -> CEILING always BREAKs
                    // to ROBUST_0 (the pre-fix behavior; Part U0/U2b/U4 FAIL).
#else
    return phase == TURBO_FORWARD
        && is_ofdm_config(settle_config)
        && is_ofdm_config(anchor)
        && !is_robust_config(anchor)
        && config_ladder_index(failed_config)
             > config_ladder_index(anchor) + 1;
#endif
  }

  // REAL FAST-PROBE piece (B) — the SHARED elevator-target method
  // (gearshift-climb-engine.md §14). Computes the SNR-ideal config the controlled
  // elevator (§13) should target, applying the SAME ceiling cap-chain and the SAME
  // supershift_retrigger_target() high-confidence-SNR gate the SUPERSHIFT
  // re-trigger already used (arq_commander.cc:4587-4608), extracted VERBATIM into
  // ONE place so the two call sites cannot drift (the documented DSP-commit
  // anti-pattern). Reads members: measurements.SNR_uplink, narrowband_enabled,
  // supershift_proven_ceiling, last_data_viable_config, robust_enabled, and calls
  // get_configuration()/optimizer_is_in_control()/supershift_retrigger_target().
  // NON-const (get_configuration is non-const). The caller GATES this on
  // `gear_shift_on==YES && is_ofdm_config(current_configuration) &&
  // measurements.SNR_uplink > -90` (the re-trigger's own enclosing gate); inside,
  // supershift_retrigger_target keeps the conservative +1 result unless the
  // high-SNR predicate licenses a multi-rung jump, so at deep/invalid SNR this
  // returns the +1-clamped value (DEEP-SNR INERT). READS the anchor but never
  // RAISES it (SAFETY #3). Defined in arq_commander.cc next to both call sites.
  // See §14.
  int elevator_target_from_snr();

  // WALL-B FIX-5 (fix5/FIX5_DESIGN.md §4.4): apply the CFG16 big-block carve COOLDOWN
  // as an additional index-cap on a proposed climb target. When the cooldown is armed
  // (bigblock_carve_cooldown_batches > 0) and `proposed` is above the cooldown ceiling
  // (CFG15), returns CFG15; otherwise returns `proposed` UNCHANGED. INDEX-MONOTONE
  // never-raise clamp — composes order-independently with supershift_proven_ceiling /
  // WB-NB ceiling / max_config_override (all "never raise"). Off the cooldown
  // (batches==0, the normal case) it is the IDENTITY, so non-bigblock and clean-CFG16
  // operation is byte-identical. Defined in arq_commander.cc next to
  // elevator_target_from_snr(). See §4.4 + the §5 audit family-B (INV-B1).
  int apply_bigblock_cooldown_cap(int proposed) const;

  // SUPERSHIFT SNR-sentinel fix (climb follow-up #1, Option A;
  // data-flow-snr-measurements.md §1.5). The CMD's forward MFSK-ACK climb
  // decodes NO LDPC data, so the canonical SNR_uplink producer
  // (arq_common.cc:6051) never runs on the CMD's climb → SNR_uplink stays at
  // its ctor sentinel -99.9 → the SUPERSHIFT re-trigger gate
  // (measurements.SNR_uplink > -90) can never fire → the modem crawls up the
  // ladder one rung at a time. The RSP encodes its measured SNR of the CMD's
  // signal in the ACK suffix; the CMD decodes it (stored in
  // turbo_received_snr). This PURE helper is the value the new producer writes
  // into measurements.SNR_uplink at the suffix-decode site — the SAME decoded
  // value, as a double, matching the canonical producer's field (SNR_uplink is
  // written for ALL roles by :6051; SNR_downlink only on RESPONDER, so the
  // CMD-only suffix path writes SNR_uplink only). No side effects so the
  // unit test (Part G) replays the identical expression. See §1.5 / §6.
  static double snr_uplink_from_suffix(float decoded_snr)
  { return (double)decoded_snr; }

  // SUPERSHIFT SNR-sentinel ENABLEMENT (climb follow-up #1b, Option 1;
  // data-flow-snr-measurements.md §1.7 / §7). snr_uplink_from_suffix() (above)
  // shipped the PRODUCER, but it only runs inside receive_ack_pattern()'s
  // `if(turbo_snr_ack_enabled)` branch (arq_common.cc:5516) — and on the CMD
  // turbo_snr_ack_enabled is set TRUE in exactly ONE place: the SUPERSHIFT
  // re-trigger (arq_commander.cc:4579), itself gated by
  // `measurements.SNR_uplink > -90` (:4548). DEADLOCK: the producer is the only
  // thing that lifts SNR_uplink off the -99.9 sentinel on the CMD's forward
  // pattern-ACK climb, but it cannot run until SNR_uplink > -90, which only it
  // provides. Result on hardware: 0× [CMD-ACK-SNR], 0× [TURBO], SNR_uplink stuck
  // at -99.9, SUPERSHIFT never armed (the modem crawls one rung at a time).
  //
  // The FIX enables the CMD's SNR-suffix decode when it EXPECTS a SET_CONFIG ACK
  // during turboshift — the SYMMETRIC counterpart to the RSP's SNR-suffix SEND
  // gate (arq_responder.cc:1122-1124:
  //   (turboshift_active || turboshift_phase != TURBO_DONE)
  //     && messages_control.data[0] == SET_CONFIG && SNR_uplink > -90).
  // We drop the RSP's `SNR_uplink > -90` conjunct: that term is the RSP's "do I
  // HAVE a measured SNR to PUT in the suffix" check (the RSP gets SNR_uplink from
  // decoding the SET_CONFIG LDPC frame, arq_responder.cc:2046). The CMD side is
  // the opposite end of the loop — it just needs the DECODER armed to RECEIVE
  // whatever the RSP sends; gating CMD enablement on SNR_uplink > -90 would
  // re-introduce the very deadlock (the CMD has no SNR_uplink yet — that is what
  // the suffix is FOR). So the CMD predicate is the RSP gate MINUS that conjunct.
  //
  // NARROW BY DESIGN (the §5 SACK-vs-SNR collision crux): the `control_code ==
  // SET_CONFIG` conjunct restricts enablement to a SET_CONFIG ACK wait. A DATA
  // ACK (which carries the SACK suffix, decoded by the separate
  // process_messages_rx_acks_data() path) NEVER satisfies this — it is not a
  // control frame — so a data ACK's SACK suffix is never fed to the SNR decoder
  // (and vice-versa). The flag is committed at the SET_CONFIG control-TX→wait
  // transition (arq_commander.cc:1050) and is unconditionally CLEARED by
  // finish_turbo_direction() (arq_commander.cc:3657) before any TRANSMITTING_DATA
  // transition, so it is provably false on every data-ACK wait. PURE (no side
  // effects) so Part H replays the identical expression. See §1.7 / §7.
  // phase is taken as int (not the TurboshiftPhase enum) because this inline
  // helper precedes the enum's declaration in the class; the enumerator
  // TURBO_DONE is visible in the BODY (complete-class context) but a named
  // parameter TYPE is not. TurboshiftPhase implicitly converts to int at the
  // call site (arq_commander.cc:1050 passes turboshift_phase directly).
  static bool turbo_snr_ack_expected_on_control(bool turbo_active,
                                                int phase,
                                                int control_code)
  { return (turbo_active || phase != TURBO_DONE) && control_code == SET_CONFIG; }

  // REAL FAST-PROBE piece (A1) — repair the CMD arm asymmetry
  // (gearshift-climb-engine.md §14). ROOT CAUSE (diagnosed, not re-investigated):
  // the forward-link SNR is ALREADY on the wire — the RSP suffixes the measured
  // SNR onto its SET_CONFIG ACKs whenever `(turboshift_active || turboshift_phase
  // != TURBO_DONE) && data[0]==SET_CONFIG && SNR_uplink > -90`
  // (arq_responder.cc:1122-1124). But on the unpinned data-anchored +1 ladder the
  // RSP's `turboshift_phase` stays at its init TURBO_FORWARD (0 SWITCH_ROLE
  // swaps), so the RSP KEEPS suffixing; meanwhile the CMD's own phase is
  // TURBO_DONE in steady state, so turbo_snr_ack_expected_on_control() (above)
  // returns FALSE → turbo_snr_ack_enabled stays false → receive_ack_pattern()
  // takes the bare-ACK else branch (arq_common.cc:5640) instead of the SNR branch
  // (:5516) → the producer (arq_common.cc:5555,
  // measurements.SNR_uplink = snr_uplink_from_suffix(...)) NEVER runs → SNR_uplink
  // stays at the -99.9 ctor sentinel → the §13 elevator's `SNR_uplink > -90` gate
  // can never fire. The send/arm predicates are ASYMMETRIC on `turboshift_phase`.
  //
  // THE FIX (no wire change — the RSP already sends): widen the CMD arm with an OR
  // clause so it ALSO arms for the gearshift-ladder SET_CONFIG ACKs the RSP
  // already suffixes — a ladder promotion in steady state (gear_shift_on==YES,
  // the SET_CONFIG raises the config index). `config_up` is the caller's
  // up-decision (config_ladder_index(negotiated) > config_ladder_index(current)),
  // computed at the SET_CONFIG-TX→wait transition (arq_commander.cc:1085).
  //
  // §5 SACK-PRESERVATION CRUX (must NOT leak onto data ACKs): like the existing
  // helper, the new disjunct REQUIRES `control_code == SET_CONFIG`. A DATA ACK
  // (which carries the SACK suffix, decoded by the SEPARATE
  // process_messages_rx_acks_data() path) is NOT a control frame → can NEVER
  // satisfy either disjunct → its SACK suffix is never routed to the SNR decoder.
  // This is structurally enforced THREE ways: (1) the `control_code==SET_CONFIG`
  // gate here; (2) the arm is written only inside the `data[0]==SET_CONFIG`
  // control-TX block (arq_commander.cc:1082); (3) clear_snr_arm_for_data_ack_wait()
  // clears turbo_snr_ack_enabled at EVERY entry into a data-ACK wait (the three
  // RECEIVING_ACKS_DATA sites), so the flag is provably false on every data ACK
  // wait. [CORRECTED 2026-05-31, gearshift-climb-engine.md §18: guard (3) USED to
  // cite finish_turbo_direction() (arq_commander.cc:3792), which clears the arm on
  // every TURBO teardown. That held for the ORIGINAL turbo-only arm, but A1 widened
  // the arm to fire on a steady-state +1 gearshift SET_CONFIG — a path that takes
  // the re-trigger `else` branch (:4702→:4753) and NEVER calls
  // finish_turbo_direction(). The widened arm therefore LEAKED into the next data
  // ACK (a §5 sibling bug an adversarial review caught). The dedicated
  // data-ACK-wait clear is the correct guard (3) for the widened arm.] The widening
  // adds NO path that arms on a non-SET_CONFIG code — the
  // existing Part H tests H2/H2b/H3 (DATA ACK / SWITCH_ROLE / non-turbo-non-up
  // SET_CONFIG do NOT arm) stay green by construction. PURE (no side effects) so
  // Part J' (FP-J1) replays the identical expression. See §14.
  static bool turbo_snr_ack_armed_for_gearshift(bool turbo_active,
                                                int phase,
                                                int control_code,
                                                bool gear_shift_enabled,
                                                bool config_up)
  {
    if(turbo_snr_ack_expected_on_control(turbo_active, phase, control_code))
      return true;                                   // the existing turbo arm
    // The gearshift-ladder disjunct: a steady-state +1 (or elevator) promotion.
    // `control_code == SET_CONFIG` keeps it OFF data ACKs (SACK preserved).
    return gear_shift_enabled && control_code == SET_CONFIG && config_up;
  }

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

  // R039 (race audit 2026-06-06): the SACK-v2 accept "window" check. A decoded
  // SACK_RSP's rx_bsi must be the current or just-prior CMD batch (mod 256),
  // because RSP only ACKs frames whose batch_seq_id is one of those. The OFDM
  // SACK_RSP arm (arq_commander.cc:2814) had NO such guard (only an exact-dup
  // reject), unlike the MFSK ACK+SACK arms (arq_commander.cc:2642-2645 partial,
  // :121-126 clean). decode_sack_v2_frame() is CRC8-only and never validates
  // rx_bsi, so a double-checksum (LDPC+CRC8) false-decode out-of-window SACK_RSP
  // would be applied by slot index against a messages_tx[] describing a
  // DIFFERENT batch -> silent mis-ACK / needless retransmit. PURE + static so
  // --test-sack-oow-reject exercises the EXACT production predicate.
  // cmd_bsi = cmd_batch_seq_id & 0xFF, prev_bsi = (cmd_bsi - 1) & 0xFF.
  static bool sack_v2_bsi_in_window(int rx_bsi, int cmd_batch_seq_id)
  {
    unsigned cmd_bsi  = (unsigned)(cmd_batch_seq_id & 0xFF);
    unsigned prev_bsi = (cmd_bsi - 1u) & 0xFFu;
    unsigned rx       = (unsigned)(rx_bsi & 0xFF);
    return (rx == cmd_bsi || rx == prev_bsi);
  }

  // FIX-8 (data-integrity): the post-reset re-adopt contiguity predicate. After a
  // BREAK / FULL-config reset wipes rsp_current_expected_batch_seq_id to -1, the
  // adopt site (arq_responder.cc:618) takes the next arriving bsi as the new
  // baseline. If a prior batch was DELIVERED (last_delivered >= 0) and the
  // adopted bsi is NEITHER the same batch (idempotent duplicate, INV-4) NOR its
  // mod-256 contiguous successor, then batches between last_delivered and adopted
  // were silently dropped -> a HOLE. Returns true iff there is a gap (caller must
  // refuse silent concatenation). PURE + static so --test-gap-abort drives the
  // EXACT production predicate. last_delivered < 0 (nothing delivered yet) =>
  // never a gap (the session-start adopt is always legitimate).
  // See bigblock_p3_hw/_fix8/FIX8_DESIGN.md §4.4-§4.5.
  static bool sack_v2_readopt_has_gap(int adopted_bsi, int last_delivered_bsi)
  {
    if(last_delivered_bsi < 0) return false;
    unsigned last = (unsigned)(last_delivered_bsi & 0xFF);
    unsigned succ = (last + 1u) & 0xFFu;
    unsigned adopted = (unsigned)(adopted_bsi & 0xFF);
    return !(adopted == last || adopted == succ);
  }

  // D3.1 (data-integrity, the UNIFIED keystone): the DELIVERY-TIME contiguity
  // predicate. Unlike sack_v2_readopt_has_gap (which guards the cur<0 RE-ADOPT
  // site, reachable only from BREAK/FULL-reset), this guards the TWO real
  // delivery commits themselves (BATCH-DONE arq_responder.cc:1879 + PREV
  // arq_responder.cc:905) at the ONE choke point both already call
  // (advance_last_delivered). It is delivery-time-anchored, NOT
  // config-transition-anchored: a forward step of >=2 in the high-water mark
  // means a batch between the last DELIVERED batch and `bsi` was never delivered
  // -> a HOLE -> the app FIFO would silently concatenate non-contiguous bytes.
  // This fires REGARDLESS of how rsp_current_expected_batch_seq_id got to `bsi`
  // (BREAK, any of the 4 SET_CONFIG demotes, the PREV-BUMP/STALE strand, or the
  // per-frame path), so it is the inviolable safety net that catches even a
  // residual handshake edge the 5 reverted config-transition point-patches
  // missed (FIX9_D3_DESIGN §7.2). Returns true iff `bsi` would skip a batch.
  // Step semantics (mod-256 forward distance fwd = (bsi - last) & 0xFF):
  //   last < 0     -> false (session-start first delivery is always legal)
  //   fwd == 0     -> false (duplicate re-delivery of the SAME bsi, INV-4)
  //   fwd == 1     -> false (contiguous successor, the ONLY legal forward step)
  //   fwd in [2,128] -> TRUE  (a real forward SKIP = a hole)
  //   fwd in [129,255] -> false (backward / late OLDER prev — advance() ignores
  //                       it anyway, no regress, no hole; audit R1)
  // PURE + static so the SIM_INPROC test drives the EXACT production decision.
  // See bigblock_p3_hw/_d31_fade/D31_INORDER_DESIGN.md §2.
  static bool delivery_step_is_gap(int bsi, int last_delivered_bsi)
  {
    if(last_delivered_bsi < 0) return false;
    unsigned last = (unsigned)(last_delivered_bsi & 0xFF);
    unsigned b    = (unsigned)(bsi & 0xFF);
    unsigned fwd  = (b - last) & 0xFFu;
    return (fwd >= 2u && fwd <= 128u);
  }

  // TURBO step-1 SNR-capability pre-truncation gate (gearshift-climb-engine.md
  // §20 — the CFG15->CFG16 under-climb on clean). PURE so --test-climb-engine can
  // drive it with no live telecom_system / channel.
  //
  // THE BUG: at the turbo forward-probe +1 step (arq_commander.cc:4757 sets
  // negotiated = current+1; the guard at :4803 then runs), a guard truncated the
  // probe whenever the CONTROL-PLANE SNR estimate (effective_snr) mapped to a
  // config BELOW the negotiated one. On a CLEAN channel that estimate is the
  // post-EQ EVM-SNR, which UNDERREPORTS the channel by 1-3 dB and saturates near
  // ~14.5 dB; it routinely jitters to ~9.0 dB → get_configuration(9.0)=CONFIG_13.
  // With the link negotiated at CONFIG_15 the guard fired, pinned
  // supershift_proven_ceiling at CONFIG_14, and CFG16 was NEVER reached even
  // though a pinned CFG16 decodes flawlessly on the SAME channel (3048 bps,
  // 400/400 nAcked, 0 retx) — i.e. the truncation is a FALSE PHY-capability
  // verdict produced by an underreporting estimate.
  //
  // THE FIX (mirrors the DOWNSTREAM §7.13.38 SACK-trust the author already wrote
  // at arq_commander.cc:3953 verbatim): only truncate when SACK Design A is NOT
  // negotiated. With SACK on, the channel can absorb partial-batch loss (SACK_RSP
  // patches missing frames) and the link's data-carrying capability is PROVEN by
  // SACK delivery + the top-config verification probe (:4824-4845) — so the
  // control-plane SNR estimate must NOT pre-truncate the climb. With SACK OFF the
  // legacy fading-margin safety net is preserved UNCHANGED (truncate as before).
  // This does NOT loosen any threshold/magic-number; it removes a false verdict on
  // the exact path where the downstream SACK-trust has nothing to recover because
  // this guard returned before finish_turbo_direction() ran.
  //
  // OVER-CLIMB STAYS FIXED: the SACK exemption only removes the SNR
  // PRE-truncation; the top config must still ACK end-to-end (verification probe
  // arq_commander.cc:4824-4845) or it BREAKs (:3483) and proven_ceiling is LOWERED
  // (:3621-3624); the turbo target is still routed through
  // supershift_retrigger_target (:4797) whose is_ofdm_config(anchor) gate clamps
  // ROBUST->OFDM at anchor+1 (the 730ffca over-climb mechanism). Caller still
  // guards effective_snr > -90 and computes snr_max_cfg = get_configuration(snr).
  // Returns true iff the turbo probe should be TRUNCATED (finish at current).
  static bool turbo_snr_truncates_probe(int snr_max_cfg, int negotiated_cfg,
                                        bool sack_v2_enabled)
  {
    // SACK negotiated => trust the proven delivery path; do NOT pre-truncate on
    // the underreporting control-plane SNR estimate (mirrors §7.13.38 :3953).
    if(sack_v2_enabled) return false;
    // Legacy fading-margin safety net (SACK off): truncate when the SNR-mapped
    // ceiling is below the probe target.
    return config_ladder_index(snr_max_cfg) < config_ladder_index(negotiated_cfg);
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

  // climb-engine integrated regression (CLI --test-climb-engine). Parts A-H,
  // each fail-before / pass-after its fix:
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
  // (d) climb follow-up #2 (Parts D-F): the connect-path CMD/RSP batch
  //     invariant, deep-SNR anchor DEMOTION (the WGN:-10 CONFIG_0<->ROBUST_0
  //     thrash escape), and the SUSTAINED-ANCHOR gate.
  // (e) climb follow-up #1, Option A (Part G): the SUPERSHIFT SNR-sentinel —
  //     a simulated SNR-suffix decode (snr_uplink_from_suffix) populates
  //     measurements.SNR_uplink > -90 so the re-trigger gate becomes eligible
  //     (pre-fix it stays at the -99.9 sentinel on the CMD's MFSK-ACK climb),
  //     plus the anti-storm bound (a live SNR admits exactly one re-entry).
  //     See data-flow-snr-measurements.md §6.
  // (f) climb follow-up #1b, Option 1 (Part H): the SNR-sentinel ENABLEMENT —
  //     the REAL turbo_snr_ack_expected_on_control() arms the CMD's SNR decode
  //     on a turbo SET_CONFIG control-TX WITHOUT first requiring SNR_uplink>-90
  //     (breaking the bootstrap deadlock that kept turbo_snr_ack_enabled false
  //     on the forward climb so the §1.5 producer never ran), and the §7
  //     SACK-vs-SNR collision guard (a DATA ACK / SWITCH_ROLE / non-turbo
  //     SET_CONFIG does NOT arm). See data-flow-snr-measurements.md §1.7 / §7.
  // Returns 0 on pass, 1 on fail. See gearshift-climb-engine.md §7.
  int test_climb_engine();

  // ROBUST_0 + streaming-compression deadlock regression
  // (data-flow-compress-frame-fill.md). Drives the REAL
  // process_buffer_data_commander() data-fill path at ROBUST_0 frame
  // dimensions (max_frame == 7 == COMPRESS_HEADER_SIZE) with streaming
  // compression enabled and a real compressible payload staged. Asserts the
  // batch carries > 0 application bytes (FAIL-BEFORE on fef293f: every batch
  // stages 0 payload → 0 throughput forever). One-shot, exits rc. See §5 audit.
  int test_robust0_compress_deadlock();

  // SIM_INPROC feasibility prototype (single-process-sim-refactor.md).
  // Single-instance in-process self-loopback: keys PTT, emits a real frame,
  // and proves the TX-path spin-loops (ptt_busy_wait + drain_playback_wait)
  // become STEP-PUMPABLE under a single-thread stepper with NO concurrent
  // drainer thread, while the spin-exit timing stays IDENTICAL to the
  // two-process paced sim (clock past delay / ring drained). Sets up its own
  // PHY + audio ring buffers (no device, no bridge/prep threads, no TCP, no
  // relay). Returns 0 on a clean inline cycle, 1 on any failure. -m SIM_INPROC.
  int test_sim_inproc();

  // 2-INSTANCE SIM_INPROC stepper (single-process-sim-refactor.md §10.5). Static
  // because it constructs its OWN two cl_telecom_system + two cl_arq_controller
  // (A=COMMANDER, B=RESPONDER) + two cl_sim_awgn (one per direction), drives the
  // real handshake via process_user_command + the §3 lockstep A<->B loop on the
  // shared virtual clock, and validates G-SMOKE (CONNECT + data B<-A), GATE-2
  // (byte-identical determinism + switch_seq), GATE-3-light. Returns 0 on PASS.
  // Selected by -m SIM_INPROC with MERCURY_SIM_2INST=1 / --sim-2inst.
  static int test_sim_inproc_2();

  // FULL-PATH REGRESSION (bigblock-whiten-align): the cross-layer test the CASE A-D
  // synthetic carve tests could NOT catch (they hand a caller-owned vector as the RX
  // passband and never exercise the LIVE receive_bigblock buffer-realloc / the per-block
  // frames_to_read arming / the FIFO delivery). Drives the 2-instance SIM_INPROC CFG16
  // big-block transfer through the REAL TX-encode -> whiten -> PHY -> receive_bigblock
  // de-whiten -> arq carve -> copy_data_to_buffer FIFO deliver, and asserts the full
  // message is delivered BYTE-FAITHFUL. FAIL-BEFORE (MERCURY_BIGBLOCK_DEFEAT_FIX=1:
  // dangling-data UAF + one-frame wait) delivers 0 bytes; PASS-AFTER delivers all.
  // Returns 0 on PASS. Selected by --test-bigblock-fullpath.
  static int test_sim_inproc_bigblock_fullpath();
  // STEPPER-CORE REWRITE Phase b durable regression (--test-sim-sustain): drives the OUTER-loop
  // stepper through a live ROBUST_0->CFG16 SET_CONFIG + OFDM big-block transfer and asserts the
  // Phase-b headline — the (iii) nested-drain DATA-path wedge is GONE (ZERO [SIM2-DEADLOCK-BREAK])
  // and the CFG16 K=8 block carves CLEAN 8/8 (per-symbol feed preserves the §8 decode cadence).
  // FAIL-BEFORE = legacy stepper wedges (deadlock-break, 0 OFDM bytes); PASS-AFTER = no wedge,
  // clean carve, bytes reach RX. Full multi-batch byte-correct sustain is the documented Phase-c
  // turnaround-timing item (recorded as a DIAGNOSTIC, not asserted). Returns 0 on PASS.
  // (MERGE: renamed *_outer() so it coexists with the PINNED-CFG15 sustain test below; both
  //  run under --test-sim-sustain.)
  static int test_sim_inproc_sustain_outer();
  // MULTI-CW WINDOW REGRESSION (fact-doc §17): the K>1 full-block byte-faithfulness test
  // the 622-byte synthetic cases and the single-arming fullpath could NOT catch. Drives a
  // FULL K=8 block (1200B, all 8 codewords) through the LIVE receive_bigblock+de-whiten+
  // per-cw-CRC carve in THREE arms: (A) CRC-ON block-window -> clean=8/8 + byte-faithful;
  // (B) NOCRC stock-window (MERCURY_BIGBLOCK_DEFEAT_FIX=1) -> forced-clean but BYTES WRONG
  // (cw0 ok, cw1..cw7 stale-ring corruption = the HW signature); (C) NOCRC block-window ->
  // byte-faithful. Proves the root cause is the RX capture WINDOW (not whiten/offset).
  // Returns 0 on PASS. Selected by --test-bigblock-multicw.
  static int test_sim_inproc_bigblock_multicw();
  // TX-LEVEL parity (HW over-level diag): builds a CFG16 telecom_system, emits a
  // production K=8 big-block waveform AND a stock per-frame CONFIG_16 OFDM waveform
  // through the SAME transmit_byte entry, and measures peak (Vp-p proxy = max|s|) +
  // RMS over the DATA span (post-preamble) of each. Prints peak/RMS ratios + PAPR.
  // Diagnoses whether the bench-observed +3.2 dB big-block over-level is a GAIN
  // difference (RMS ratio != 1) or PAPR/length peak-vs-RMS (RMS ~1, peak ratio >1).
  // Returns 0 always (measurement, not pass/fail). Selected by --test-bigblock-txlevel.
  static int test_bigblock_txlevel();
  // GAP-2 LIVE-PATH REGRESSION (diag/livepath-sim): the cross-layer test the PINNED
  // fullpath/multicw could NOT catch — they hand-pin CFG16 via load_configuration
  // (MERCURY_SIM2_PIN=1, gear_shift_on=NO) so they BYPASS the live config transition
  // (no SET_CONFIG control frame, no control-ACK turnaround — the exact path the
  // 79207f7 TX-hold fix protects). This drives a REAL CONNECT at the robust start, then
  // fires ONE production SET_CONFIG handshake robust->CFG16 over the live wire (the
  // gearshift's own negotiated_configuration + add_message_control(SET_CONFIG) sequence,
  // NOT a load_configuration hand-pin), then transfers a real K=8 (1374B) payload through
  // the REAL process_messages_tx_data -> send_batch -> bigblock_send_one_block emit and
  // the REAL receive_byte -> bigblock_rx_cw0_header_valid() gate -> carve -> FIFO. Asserts
  // the cw0-CRC gate ACCEPTS the real emitted blocks and the message delivers byte-faithful
  // (the KEY question: does sim REPRODUCE the HW cw0-CRC reject, or is the bug HW-only?).
  // Returns 0 on PASS. Selected by --test-bigblock-livepath.
  static int test_sim_inproc_bigblock_livepath();
  // SIM_INPROC stepper-wedge regression (SIMFTR_ROOTCAUSE.md §7/§8 fix #1). Drives a
  // PINNED-CFG15 clean (SNR3K=900) deterministic transfer for payloads {600,2000,4000,
  // 8000} through test_sim_inproc_2() and asserts each terminates byte-correct via the
  // genuine delivery break (not the wedge). FAIL-BEFORE on the SAME binary via
  // MERCURY_SIM2_DEFEAT_SIMFTR_FIX=1 (reproduces the post-transfer keepalive spin →
  // watchdog-stalled). Returns 0 on PASS. Selected by --test-sim-sustain.
  static int test_sim_inproc_sustain();
  // Capture of the last test_sim_inproc_2() run's delivery (read by the full-path
  // regression to assert byte-faithful delivery without re-parsing stdout).
  static long sim2_last_rx_have;
  static long sim2_last_payload_len;
  static bool sim2_last_bytes_ok;
  // SIM_INPROC fix #1 (SIMFTR_ROOTCAUSE.md §7): true iff the last test_sim_inproc_2()
  // run did NOT terminate via the genuine delivery break — i.e. it hit the stall
  // cutoff, the stop-after hooks, OR the post-delivery-abort fail-before watchdog
  // (the stepper-wedge reproduced). The sustained-delivery test asserts this is
  // false for every payload (no wedge) when the fix is present.
  static bool sim2_last_stalled;
  // GAP-2 LIVE-PATH: cw0-CRC gate decision tally for the last receive run. accepts =
  // real big-blocks that PASSED bigblock_rx_cw0_header_valid() and were carved; rejects =
  // CFG16 acquisitions that FAILED the cw0 wire-CRC and were re-decoded on the stock
  // per-frame path. The live-path regression reads these to answer reproduces_cw0crc_reject.
  static long sim2_gate_accepts;
  static long sim2_gate_rejects;
  // GAP-2 LIVE-PATH: count of real big-blocks EMITTED by bigblock_send_one_block in the
  // current run (the TX switch engaged + tx_transfer'd a block). The live-path regression
  // asserts emits>0 so the cw0-CRC-gate assert is meaningful (the gate ran on real blocks).
  static long sim2_tx_block_emits;
  // Capture of the FIRST big-block carved in the current run (bigblock_block_to_arq):
  // n_clean / K of the first decoded block. The full-path regression reads these to
  // assert fail-before (first block decodes PARTIAL, n_clean<K) vs pass-after (CLEAN,
  // n_clean==K) WITHOUT running the unstable post-partial retry loop to completion.
  // Reset to -1 by the regression before each arm.
  static int  bigblock_first_clean;
  static int  bigblock_first_K;
  // CHANNEL-ESTIMATION HEALTH (fix/bigblock-chanest): the big-block RX mean|H| of the FIRST
  // block carved this run. The genuine 2-instance regression (test_sim_inproc_bigblock_chanest)
  // asserts it stays healthy (collapses toward 0 when an un-tracked CFO/SFO ramps a rotating
  // phasor across the 133-symbol block — the off-bench reproduction of the HW [RXACQ] defect).
  static double bigblock_first_meanh;
  // D2 DELIVERY-ARMING capture (fix/bigblock-chanest, fact-doc bigblock-delivery-handoff §3/§7):
  // for the FIRST big-block carved this run, whether the carve ARMED the prev-batch delivery
  // machinery on a PARTIAL outcome. -1 = no partial block carved yet (or clean); 0 = PARTIAL
  // but prev NOT armed (the D2 BUG: clean slots stranded -> INV-B violated); 1 = PARTIAL and
  // the responder was routed to the audited ACK-GATE which arms rsp_prev_batch_active + sends
  // the SACK (the D2 FIX). test_sim_inproc_bigblock_multicw ARM-D reads this to assert the
  // delivery-arming fail-before (0) -> pass-after (1) deterministically on the FIRST block,
  // without depending on the (separate, flaky) D3 selective-repeat recovery transport.
  static int bigblock_first_partial_prev_armed;

  // GENUINE big-block channel-estimation regression: drive the 2-instance SIM_INPROC CFG16
  // big-block decode through the REAL ref==NULL path with a CFO/SFO-impaired channel; assert
  // mean|H| collapse + 0-delivery (fail-before) and recovery to byte-faithful (pass-after).
  static int test_sim_inproc_bigblock_chanest();

  // ACQUISITION-WINDOW POSITION regression (fact-doc §19): drive ONE genuine K=8 CFG16
  // big-block into a FIXED production-sized capture window at several IN-WINDOW preamble
  // offsets (head, mid, near-end) and assert the §19 window-position guard DEFERS a
  // late-landing block (tail past the window) and decodes 8/8 once the full block fits,
  // vs the pre-fix DEFEAT_ACQGUARD arm which carves a TRUNCATED block (bytes_ok=0). This
  // is the off-bench fail-before/pass-after for the HW ~5.6% acquisition-fraction defect
  // the existing chanest harness cannot reproduce (it custom-sizes buffer_Nsymb to fit).
  static int test_sim_inproc_bigblock_acqwindow();

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

  // FIX-8 (data-integrity) — silent lost-batch GAP on post-reset re-adopt.
  // In-process SIM_INPROC synthetic-fire (CLI --test-gap-abort). Reproduces the
  // bench-4 sequence: deliver batches 0..4 (high-water=4), force the BREAK reset
  // (cur=-1, prev=-1) with batches 5,6,7 UNDELIVERED, present a bsi=8 v2 DATA
  // frame at the adopt site. fail-before (MERCURY_GAP_ABORT_DEFEAT=1): silent
  // concatenation — fifo_buffer_rx = [0-4 bytes][8 bytes], oracle compare FAILS.
  // pass-after (defeat off): [RSP-V2-GAP-ABORT], link_status=DROPPED, NO batch-8
  // bytes appended, delivered prefix == EXACTLY batches 0-4. Variants:
  // CONTIGUOUS-NOOP (bsi=5 -> no abort), DUPLICATE (bsi=4 -> no abort, INV-4),
  // PREV-ORDERING (R1 high-water no-regress), NB (R5 batch=1 same gate).
  // Returns 0=PASS, 1=FAIL. Default builds never call this.
  // See bigblock_p3_hw/_fix8/FIX8_DESIGN.md + FIX8_AUDIT.md.
  int test_gap_abort_on_readopt();

  // D3.1 (data-integrity) — UNIFIED in-order-delivery across EVERY demote case.
  // In-process SIM_INPROC synthetic-fire (CLI --test-inorder-demote). Drives the
  // REAL delivery-time predicate (delivery_step_is_gap) + the REAL re-adopt
  // predicate (sack_v2_readopt_has_gap) + the REAL fifo_buffer_rx app stream as
  // the oracle, through ALL FIVE demote cases with a PARTIAL in-flight batch:
  //   (1) BREAK demote (cur=-1 re-adopt),
  //   (2) FIX-4 carve demote CFG16->CFG15 (SET_CONFIG-only, cur>=0),
  //   (3) FIX-9 D3 demote CFG16->CFG15 (SET_CONFIG-only, cur>=0),
  //   (4) FIX-3 verification-probe-skip demote (SET_CONFIG-only, cur>=0),
  //   (5) plain gearshift step-down (SET_CONFIG-only, cur>=0),
  //   PLUS the PREV-BUMP/STALE strand within a SET_CONFIG demote.
  // Each asserts the delivered app stream is CONTIGUOUS+IN-ORDER+byte-faithful
  // (md5/memcmp) OR loudly REFUSED (DROPPED, no silent concat) — NEVER a silent
  // [0-4][8-] concatenation. fail-before (MERCURY_GAP_ABORT_DEFEAT=1): the
  // SET_CONFIG demotes keep cur>=0, the FIX-8 re-adopt gate is bypassed, and the
  // delivery-time gate is the ONLY backstop — with the gate defeated the cases
  // reproduce the silent concat (md5-false). pass-after (defeat off): #1 (the
  // delivery-time assertion) + #2 (the SET_CONFIG re-baseline) catch every case.
  // Returns 0=PASS, 1=FAIL. Default builds never call this.
  // See bigblock_p3_hw/_d31_fade/D31_INORDER_DESIGN.md.
  int test_inorder_demote();

  // ---- P2 big-block ARQ re-granularization (see
  // fact-documents/data-flow-bigblock-arq-unit.md) ----------------------------
  //
  // bigblock_block_to_arq(): the PRODUCTION block->ARQ delivery entry. At the
  // CFG16-bigblock rung (bigblock_framing_enabled), ONE receive_bigblock decode
  // yields K=8 per-codeword info-bit sub-units + a K-bit cw_ok clean vector (the
  // SACK granularity, telecom_system.h:595). This entry carves cw_ok into
  // messages_rx[0..K-1] (RECEIVED iff cw_ok[c]==1), sets the synthetic EOB=K-1
  // (RISK-4, BEFORE the prev branch), and drives the ONE-ACK / partial-SACK /
  // bsi-once flow — replacing the K per-frame add_message_rx_data writes.
  //   cw_ok          : length-K per-codeword clean bitmap (1=clean, 0=failed).
  //   block_bsi      : the batch_seq_id this block advertises (one block=one batch).
  //   tx_payload     : K*sub_len bytes the TX block carried (for the
  //                    delivered==TX assertion); sub_len = bytes/codeword.
  //   sub_len        : payload bytes per codeword sub-unit.
  // Returns SUCCESSFUL when the block was delivered to messages_rx[] + the ARQ
  // state advanced; BIGBLOCK_ARQ_NOT_WIRED (the P2.0 stub return) when the
  // block->ARQ logic is not yet wired. P2.0 ships this as a one-line stub (NO ARQ
  // logic) so --test-bigblock-arq-unit FAILS; P2.4/2.5/2.6 implement the body so
  // it PASSES (bisectable, see fact-doc §6).
  // PHASE 1 (fact-doc §11): `sub_lengths` is the per-codeword APP byte length (from the
  // wire length table) so each delivered slot byte-matches its TX frame (INV-6 with
  // VARIABLE-length compressed frames). `cw0_offset` is codeword 0's app-byte base in
  // `tx_payload` (= hdr_total; the wire header occupies cw0's prefix); codewords c>=1
  // start at c*sub_len. When sub_lengths==NULL the carve falls back to a uniform sub_len
  // length per codeword and cw0_offset=0 (the legacy fixed-length unit-test path).
  int bigblock_block_to_arq(const int* cw_ok, int K, unsigned char block_bsi,
                            const unsigned char* tx_payload, int sub_len,
                            const int* sub_lengths = nullptr, int cw0_offset = 0);

  // LIVE-PATH DELIVERY FIX (bigblock-whiten-align): when a CLEAN big-block is carved,
  // bigblock_block_to_arq delivers the K decoded sub-units to the app FIFO via
  // copy_data_to_buffer() (RECEIVED->ACKED then push) — the live ARQ path had no
  // downstream gate doing this, so the block decoded byte-faithfully but 0 app bytes
  // reached fifo_buffer_rx. The CASE A-D unit tests (test_bigblock_arq_unit.cc) assert
  // on messages_rx[] DIRECTLY (bigblock_test_delivered_varlen) and have no real
  // FIFO/compression context, so they set this flag true to KEEP the slots in
  // messages_rx[] (skip the FIFO push). Default false = live behavior (deliver).
  bool bigblock_skip_fifo_delivery = false;

  // ---- STEP 2: live send-path wiring (P3 prereq, see arq_common.cc) ----------
  // bigblock_send_one_block(): emit the current new-data batch as ONE big-block
  // via transmit_byte -> transmit_bigblock (gated on
  // telecom_system->bigblock_framing_enabled; FORCED true for validation, AUTO-
  // election DEFERRED to P4). Returns true when it HANDLED the batch (caller skips
  // the per-frame loop); false when it declined (MFSK / retx / mixed-control /
  // oversized batch -> stock per-frame path). Retx stays STOCK CFG16 per-frame.
  bool bigblock_send_one_block();
  // bigblock_pack_block(): build the K*sub_len on-wire block payload (cw0 header prefix +
  // per-codeword app bytes + the whole-block CRC-32 in cw(K-1)'s trailer + per-cw CRC-8
  // tails) from the current new-data batch (messages_batch_tx[]). Extracted from
  // bigblock_send_one_block so the V2 FIX-1 cap-reservation rule (the LAST codeword reserves
  // BIGBLOCK_BLOCK_CRC_BYTES for the block-CRC field, fact-doc §9) lives in ONE place
  // exercised by BOTH production AND the MAX-PAYLOAD test arm. Returns true + fills
  // out_payload/out_K/out_sub_len/out_ndata/out_lengths (and stashes bigblock_tx_block_*)
  // on success; false (decline -> stock per-frame path) when the geometry is too small or
  // frame 0 does not fit cw0's reduced capacity.
  bool bigblock_pack_block(int n_data, std::vector<unsigned char>& out_payload,
                           int& out_K, int& out_sub_len, int& out_ndata,
                           std::vector<int>& out_lengths);
  // bigblock_receive_carve(): the RX side — after receive_byte()->receive_bigblock()
  // decoded ONE block (stashing telecom_system->bigblock_last_rx_cw_ok + the K
  // decoded info-bit sub-units in `info_bits`), translate it into the ARQ data unit
  // via bigblock_block_to_arq() (carve cw_ok -> messages_rx[], synthetic EOB, one
  // ACK / partial SACK / bsi-once). Returns the bigblock_block_to_arq rc.
  //
  // PHASE 1 (fact-doc §11): the block carries a SELF-DESCRIBING header in cw0's prefix
  // [bsi, n_data, length[0..K-1]]. When `use_wire_header` is true (the live path) the
  // carve PARSES that header off the decoded payload and uses the WIRE bsi (authoritative
  // — drift-proof across a multi-block session) + the per-codeword length table (so each
  // delivered slot byte-matches its TX frame; compression transparency). `fallback_bsi`
  // is used only if use_wire_header is false (legacy path) or the header is unusable.
  int bigblock_receive_carve(const int* info_bits, unsigned char fallback_bsi,
                             bool use_wire_header = true);
  // GAP-3 CARVE-GATE HARDENING (cfg16-controlack-hold): returns true iff the just-
  // decoded CFG16 acquisition (in telecom_system->bigblock_rx_infobits, K codewords)
  // is a REAL big-block — i.e. cw0's de-whitened wire-CRC-8 matches its tail byte.
  // A real block's cw0 always carries the FEC+CRC-protected [bsi,n_data,length-table]
  // header; a single OFDM control frame / stale audio / noise mis-routed into the
  // block carver does NOT produce a valid cw0 CRC. Used by process_messages_data to
  // REJECT a mis-carve and re-decode the audio on the stock per-frame path (so a
  // SET_CONFIG/ACK control turnaround received at CFG16 is parsed as control, not
  // carved). Reuses the SAME de-whiten + CRC8_calc + BIGBLOCK_CW_CRC_* the carve and
  // the TX use, so it cannot drift from the on-wire format.
  bool bigblock_rx_cw0_header_valid();
  // MULTI-CW WINDOW FIX (fact-doc §17): a big-block decode snapshots buffer_Nsymb
  // samples but the snapshot only fires when frames_to_read hits 0, so frames_to_read
  // controls how many FRESH symbols are accumulated before the block is handed to the
  // decoder. The block spans bigblock_rx_block_nsymb() (~64) OFDM symbols; a stock
  // CFG16 frame is ~13. Every per-block ACK-turnaround / FAIL re-arm that arms a STOCK
  // frame (send_ack_pattern et al.) truncates the next block's window so cw1..cw7 read
  // a stale ring and decode to deterministic garbage (cw0 — the early symbols — stays
  // byte-correct). This helper raises a stock frames_to_read to the FULL block span
  // when the bigblock rung is active (framing on, M!=MFSK, CFG16), and returns
  // stock_ftr UNCHANGED on every other path (byte-identical to baseline). It reads the
  // SAME bigblock_rx_block_nsymb() the §15.2 sites use — no parallel mechanism. The
  // MERCURY_BIGBLOCK_DEFEAT_FIX=1 reproducer hook bypasses the clamp (returns stock_ftr)
  // so the SAME binary reproduces the pre-fix truncated-window corruption for the A/B.
  int bigblock_block_ftr_or(int stock_ftr);
  // ACQUISITION-WINDOW POSITION GUARD — WAIT-FOR-TAIL (fact-doc §22, supersedes the §19
  // defer-and-re-arm REGRESSION). The §17 ftr clamp made the snapshot WAIT for a block-span
  // of FRESH symbols (the COUNT), but the snapshot still fires at a RANDOM ring write-head
  // phase (the POSITION), so a block whose preamble lands late in the captured window has
  // its tail STILL ARRIVING (future samples, not yet in the ring) when frames_to_read hits
  // 0 -> bb_at zero-pads the tail -> the block-wide estimate collapses -> cw0 wire-CRC fails
  // even on a perfect timing lock. bigblock_acq_window_fits() returns true when the FULL
  // located block (head + preamble + Ngrid) fits inside the captured window (head_delay +
  // block_nsymb*sym_samples <= capture_nsamples), i.e. the carve will see a complete block.
  // When it returns false the receive() guard WAITS-FOR-TAIL: it re-arms frames_to_read to
  // ONLY ceil(overrun/symbol_period)+1 fresh symbols (NOT a block-span — a block-span re-arm
  // scrolls the head off the back of the ring, the §19 deadlock), leaves the ring INTACT
  // (head must survive) and does NOT touch ring_write_index. The ring slides forward by that
  // short wait so the SAME single transmission's tail arrives in-ring and the block re-lands
  // earlier in the window -> fits -> carves 8/8 (no NAK/retransmit needed; geometry guarantees
  // the head stays in-ring since block_span 64 sym <= ring 133 sym). Gated on the CFG16
  // big-block rung; off-rung the guard is never entered (byte-identical).
  // bigblock_rx_defer_count caps consecutive WAITS so a genuinely absent block falls through
  // to the stock cw0-CRC gate instead of spinning. Reset on every accept.
  bool bigblock_acq_window_fits();
  int  bigblock_rx_defer_count = 0;
  // WALL-B FIX-3 — RSP CARVE-SUSPEND WATCHDOG (bigblock_p3_hw/_wallb/fix3). While parked
  // at CFG16 with the big-block rung elected, the RSP routes ALL CFG16 OFDM audio into the
  // K=8 carve and block-spans EVERY frames_to_read re-arm to a ~74-symbol block window. The
  // CMD FIX-4 demote SET_CONFIG (a ~13-symbol control frame, sent ON the CFG16 PHY per
  // arq_commander.cc:3685-3692/4819-4822) and the BREAK burst then land mid-window: the
  // carve rejects them on cw0-CRC and the GAP-3 stock fallback re-decodes the SAME oversized
  // snapshot (control preamble mis-aligned -> FTR fail), so the RSP is structurally deaf to
  // any control/BREAK at a rung it can only leave via the global LINK watchdog (HW 2026-06-09:
  // WALLB_HW_VERDICT.json — RSP load_configuration tail …,15,16,16,100,100). After K
  // consecutive cw0-CRC carve REJECTS with 0 accepts the RSP SUSPENDS the carve route AND
  // the block-span re-arm FOR THE CFG16 RUNG, but STAYS at CFG16 PHY (it does NOT demote its
  // own config — the CMD's demote/retransmits are CFG16-PHY until the ACK, so the RSP must
  // stay at CFG16 to decode them). Suspended, the RSP behaves like the stock CFG16 OFDM
  // receiver the climb already proved decodes SET_CONFIGs 13->14->15->16, so it decodes the
  // CFG16-PHY demote, loads CFG15, ACKs; both peers run per-frame CFG15 instead of 0 bytes.
  // INCREMENTED in the cw0-CRC reject branch (arq_common.cc:8186-8212); RESET to 0 on a real
  // carve accept (arq_common.cc:8229, the recovery event) and on any arq-layer config change
  // (load_configuration, arq_common.cc:1839 — only past the no-op guard). On the carve-SUCCESS
  // path the first accept resets the streak to 0 so it never reaches K -> all three consumers
  // are NO-OPs (carve-success byte-identical, INV-6). Provably safe: gated on CFG16 &&
  // big-block-framing && streak>=K, unreachable on the carve-success path; no wire/TX change.
  int  bigblock_rx_carve_fail_streak = 0;
  static const int BIGBLOCK_CARVE_SUSPEND_K = 3;  // same scoping idiom as STALE_CFO_RESET_FAILS
  // True iff the CFG16 carve route should be SUSPENDED (K consecutive cw0-CRC rejects, 0
  // accepts). MERCURY_BIGBLOCK_DEFEAT_CARVESUSPEND=1 forces FALSE (restores the pre-fix deaf
  // RSP) for the fail-before A/B arm; production never sets it.
  bool bigblock_carve_suspended();
  // WALL-B FIX-3 streak state-machine (ONE source of truth, called by the receive() carve-gate
  // branches AND the unit test). note_carve_reject(): a real CFG16 cw0-CRC carve REJECT ->
  // ++streak (and log [BB-CARVE-SUSPEND] on the K-th, returning true once it crosses K).
  // note_carve_accept(): a real block carved -> reset streak to 0 (the recovery event).
  bool bigblock_note_carve_reject();
  void bigblock_note_carve_accept();
  // TX block stash (set by bigblock_send_one_block): the K*sub_len payload bytes the
  // block carried + its geometry, so the in-process single-block harness can carve
  // it back byte-faithfully (the delivered==TX ground truth, INV-6).
  std::vector<unsigned char> bigblock_tx_block_payload;
  int           bigblock_tx_block_K       = 0;
  int           bigblock_tx_block_sub_len = 0;
  unsigned char bigblock_tx_block_bsi     = 0;
  int           bigblock_tx_block_ndata   = 0;
  // PHASE 1 (fact-doc §11): per-codeword app byte length (the wire length table), so
  // the in-process harness carves each sub-codeword its EXACT TX length byte-faithfully
  // (INV-6 with VARIABLE lengths; the production RX reads the same table off the wire).
  std::vector<int> bigblock_tx_block_lengths;

  // ---- V2 FIX-2 (fact-doc §10): assembled-block integrity stash for the PARTIAL /
  // SACK-completed delivery gate. The whole-block CRC-32 only gates the FULL-CLEAN carve
  // (bigblock_receive_carve, n_clean==K). A KEPT codeword that the per-cw CRC-8 FALSE-PASSES
  // (wrong bytes at the ~2^-8 floor) inside a PARTIAL block is transferred to messages_rx_prev[]
  // and, once the GENUINE gap codewords are recovered, delivered via copy_data_to_buffer at the
  // prev-batch completion (arq_responder.cc:738-784) with NO block-CRC check. FIX-2 stashes the
  // block-integrity context HERE at a big-block PARTIAL carve so the completion can REASSEMBLE the
  // K*sub_len codeword-aligned image from the prev slots and re-verify the SAME block-CRC-32
  // before delivery; on mismatch the completion does NOT deliver (re-requests the block).
  // Armed ONLY when cw(K-1) decoded CRC-8-clean at carve (it carries the block-CRC field, so its
  // value is then trustworthy) -> NO false reject of a genuinely-clean completion. One-shot:
  // consumed at completion / reset at the start of each carve. Big-block-scoped: a non-big-block
  // prev completion (armed=false or bsi mismatch) is byte-identical to before.
  bool          bigblock_partial_armed         = false;
  int           bigblock_partial_block_bsi     = -1;   // the bsi (0..255) of the stashed block
  int           bigblock_partial_K             = 0;
  int           bigblock_partial_sub_len       = 0;
  int           bigblock_partial_hdr_total     = 0;
  int           bigblock_partial_cw0_offset    = 0;
  unsigned int  bigblock_partial_expected_crc32= 0;    // TX block-CRC-32 read from cw(K-1) trailer
  // V3 FIX (fact-doc §10/§14): the REAL decoded n_data (cw0 header byte payload[1], the count of
  // filled codewords the TX emitted), parsed at the carve when cw0 is clean. The TX writes
  // block_payload_bytes[1]=n_data (arq_common.cc:3987), which is <K for an UNDER-FILLED block
  // (FIFO-drained / end-of-document tick). bigblock_partial_block_crc_ok() MUST reconstruct the
  // header byte with THIS value (not the hard-coded K) or a genuinely-clean n_data<K PARTIAL block
  // CRC-mismatches -> false-reject LIVELOCK (the §10 blocker). Stashed at the arm site alongside the
  // expected CRC-32. Default -1 (unset, defensive) is treated as K at the consumer.
  int           bigblock_partial_n_data        = -1;   // decoded cw0 n_data (filled-codeword count)
  std::vector<int> bigblock_partial_lengths;            // per-codeword wire app lengths

  // ---- C6 MEASURE-ONLY: cw(K-1)-gap PARTIAL block-CRC RESIDUAL EXPOSURE counter -------
  // (block-crc-upgrade-design.md §7 [?] + bigblock-integrity.md §5/§12 — QUANTIFY-FIRST).
  // The whole-block CRC-32 (FIX-2) re-verifies the assembled block at prev-batch completion,
  // but it is ARMED ONLY when cw(K-1) decoded CRC-8-clean (cw(K-1) carries the CRC-32 field at
  // BIGBLOCK_BLOCK_CRC_OFFSET — when it is ITSELF the gap the field is never recovered, so the
  // stash cannot arm; the carve's arm precondition `cwlast_clean` is FALSE, arq_common.cc:5097).
  // In that sub-case the block is delivered WITHOUT the block-CRC-32 gate, so any OTHER kept
  // codeword that FALSE-PASSED its per-cw CRC-8 falls back to the 2^-8 floor the CRC-32 exists to
  // eliminate. This is the residual exposure §7 asks to MEASURE before deciding the CLOSE.
  // MEASURE-ONLY: this stash arms a one-shot signal at the carve (when cw(K-1) is the gap AND
  // >=1 other kept slot is delivered) and the prev-batch completion increments the counter +
  // logs [PARTIAL-CRC-RESIDUAL] when such a block is actually DELIVERED. NO delivery behaviour
  // changes; the per-cw CRC-8 is NOT weakened; nothing is refused. Sibling one-shot of the
  // bigblock_partial_* stash (set at the carve, consumed at prev-batch completion, reset per carve).
  bool          bigblock_residual_armed         = false; // cw(K-1)-gap PARTIAL with >=1 other kept slot
  int           bigblock_residual_block_bsi      = -1;    // bsi (0..255) of the residual-exposed block
  int           bigblock_residual_kept_slots     = 0;     // count of kept (cw_ok==1) slots besides cw(K-1)
  // Count of DELIVERED PARTIAL blocks where cw(K-1) was in the gap (block-CRC-32 NOT armable) AND
  // >=1 other kept slot was delivered relying only on the per-cw CRC-8 floor — the residual-exposure
  // population §7 asks to quantify. Diagnostic only; logged via [PARTIAL-CRC-RESIDUAL]. 0 at init.
  long long     bigblock_partial_crc_residual_count = 0;

  // Verify the stashed block-CRC-32 over the K-codeword payload reassembled from messages_rx_prev[]
  // (app bytes per slot + reconstructed cw0 header, with per-cw CRC tails + the block-CRC field
  // zeroed — the SAME image the TX computed over). Returns true when the block is byte-consistent
  // (deliver) and false on mismatch (do NOT deliver). Called at the prev-batch completion.
  bool bigblock_partial_block_crc_ok();

  // C6 MEASURE-ONLY: count a DELIVERED PARTIAL big-block whose cw(K-1) was the gap (block-CRC-32
  // NOT armable) AND >=1 other kept codeword rode the per-cw CRC-8 floor — the residual-exposure
  // population block-crc-upgrade-design.md §7 asks to QUANTIFY. Called from the prev-batch
  // completion AFTER delivery; increments bigblock_partial_crc_residual_count + logs
  // [PARTIAL-CRC-RESIDUAL] when bigblock_residual_armed and the delivered bsi matches. Clears the
  // one-shot. Changes NO delivery behaviour (the block is already delivered by the caller).
  void note_bigblock_partial_crc_residual(int delivered_bsi);

  // ---- STEP 3: single-block end-to-end in the 2-instance in-process sim -------
  // test_sim_inproc_bigblock(): a dedicated single-block 2-instance ARQ harness
  // (CMD + RSP, pinned CFG16-bigblock). Drives the PRODUCTION send-path
  // (bigblock_send_one_block -> transmit_byte -> transmit_bigblock) through the
  // PROVEN PHY block loopback into the RSP's receive_byte -> receive_bigblock ->
  // bigblock_receive_carve, then the ACK-GATE / clean-ACK match (R-B). Asserts a
  // single block ARQ-drives CMD->RSP->ACK->CMD byte-faithful, plus a one-bad-
  // codeword partial -> selective-repeat completes the block. Returns 0=PASS.
  // Optimizer/gearshift authority UNTOUCHED. CLI: --test-sim-inproc-bigblock.
  int test_sim_inproc_bigblock();

  // In-process big-block ARQ-granularization regression (one-shot, then exit rc).
  // CLI: --test-bigblock-arq-unit. THREE cases per fact-doc §6:
  //   1 clean K=8 -> one ACK / all-ones K-bit bitmap / bsi bumps ONCE;
  //   2 one-bad-codeword -> partial K-bit SACK + selective-repeat of EXACTLY that
  //     codeword (stock CFG16 per-frame retx + messages_rx_prev);
  //   3 lost-EOB -> synthetic EOB=K-1 holds, RSP sizes batch=K, prev completes.
  // Asserts RX delivered bytes == TX bytes at every transition. Returns 0=PASS,
  // 1=FAIL. MUST FAIL before P2 wiring (the stub above), PASS after. Default
  // builds never call this.
  int test_bigblock_arq_unit();
  // Test-only helpers for test_bigblock_arq_unit (member methods because
  // messages_rx[]/nMessages are private). Count RECEIVED slots in
  // messages_rx[0..K-1]; count delivered bytes that match the expected TX
  // payload byte-for-byte (the "RX delivered == TX" measure, INV-6).
  int bigblock_test_count_received(int K);
  int bigblock_test_delivered_bytes(int K, int sub_len,
                                    const unsigned char* tx_payload);
  // PHASE 1 (fact-doc §11): VARIABLE-length delivered measure — count bytes that match
  // app_flat[app_off[c]+j] AND require messages_rx[c].length == app_len[c] (so a slot
  // delivering sub_len pad bytes instead of its real frame length FAILS). Equals
  // sum(app_len[0..K-1]) iff every sub-codeword delivered its exact TX frame bytes.
  int bigblock_test_delivered_varlen(int K, const int* app_len, const int* app_off,
                                     const unsigned char* app_flat);

  // SACK-GATE T6 (R-B / bug #9): CMD/RSP election symmetry against the
  // PRODUCTION setter. Constructs TWO independent cl_telecom_system +
  // cl_arq_controller (one CMD-role, one RSP-role), loads a REAL CFG16 grid into
  // each (so bigblock_codeword_count() runs the live nBits/ldpc.N geometry, NOT a
  // hardcoded K), turns on bigblock_framing_enabled, and runs the SHARED
  // production sack_negotiated_recompute_batch() on BOTH. Asserts both elect
  // data_batch_size == K == BB_TEST_K(8), both derive the identical all-ones
  // target (1<<data_batch_size)-1 == 0xFF (the cmd_clean_data_ack_crc_valid:136-139
  // expression), and the RSP's all-clean K-bit bitmap 0xFF is accepted by that
  // gate (rx_bitmap==all_ones) AND by sack_clean_confirmation_accepted(). Returns
  // 1 on PASS, 0 on FAIL (caller increments cases_passed). This is the #9 GO/NO-GO:
  // without the R-B pin the non-robust 30s formula elects ~25 -> CMD all_ones
  // 0x1FFFFFF != RSP 0xFF -> clean ACK never matches (the "4 wire failures").
  int bigblock_test_election_symmetry();

  // CLIMB-ELECTION (fact-doc data-flow-bigblock-arq-unit.md §16): proves the
  // big-block rung is ELECTED by the GEARSHIFT CONFIG TRANSITION (load_configuration
  // landing on CFG16 with framing on), NOT only by an explicit
  // sack_negotiated_recompute_batch() at connect. Builds two instances (CMD+RSP),
  // seeds the bug-#9 state (load_configuration(CFG15) -> 30s formula -> batch=25),
  // turns on bigblock_framing_enabled, then fires the SAME gearshift entry the climb
  // uses (load_configuration(CFG16)) and asserts the transition elected K==8 on BOTH
  // peers (symmetric, all_ones==0xFF) WITHOUT any explicit election call, then drives
  // the REAL emit (bigblock_send_one_block) + carve + delivers byte-faithful.
  // fail-before/pass-after on the SAME binary via MERCURY_BIGBLOCK_DEFEAT_ELECTION=1
  // (skips the load_configuration tail election). Returns 0 on all-pass, 1 on failure.
  static int test_bigblock_climb_election();
  // WALL-B FIX-3 — RSP carve-suspend watchdog UNIT test (bigblock_p3_hw/_wallb/fix3).
  // CLI: --test-bigblock-carve-suspend-unit. Deterministic in-process test on a REAL RSP
  // cl_arq_controller at CFG16 (big-block framing on): drives the SHARED streak state machine
  // (bigblock_note_carve_reject / _accept — the SAME methods the receive() carve-gate branches
  // call) + the three consumers (bigblock_carve_suspended, bigblock_block_ftr_or revert, the
  // BREAK-gate predicate) through every transition the AUDIT enumerates. Asserts: streak builds
  // to K -> suspended; block-span re-arm reverts to stock; DEFEAT env restores the pre-fix deaf
  // RSP (fail-before); reset-on-accept + reset-on-config-change (INV-3/RISK-A); off-rung NO-OP
  // (INV-2); first-block-of-fresh-visit carves (RISK-D). Plus a REAL block loopback
  // (transmit_byte -> receive_byte) under MERCURY_BIGBLOCK_SIM_CARVEFAIL=all confirming the
  // receive() reject path drives the streak through the actual code. Returns 0 on all-pass.
  static int test_bigblock_carve_suspend_unit();
  // R039 (race audit 2026-06-06) — OFDM SACK_RSP out-of-window reject test.
  // CLI: --test-sack-oow-reject. Builds a CRC8-VALID SACK_RSP payload with an
  // out-of-window batch_seq_id, drives the REAL decode_sack_v2_frame() (which is
  // CRC8-only and DOES accept it — the root-cause gap), then applies the REAL
  // production window predicate sack_v2_bsi_in_window() (the fix). Asserts: the
  // OOW frame decodes (proving the decode layer doesn't guard), the new guard
  // REJECTS it (pre-fix: no guard -> bitmap applied -> mis-ACK), and an
  // in-window frame is still ACCEPTED (regression guard). Returns 0=PASS,1=FAIL.
  int test_sack_oow_reject();

  // R038 (race audit 2026-06-06) — EOB-poison-from-prev-retransmit test.
  // CLI: --test-eob-poison-prev-retx. Drives the real EOB staging/promotion
  // members (last_received_end_of_batch_seq, rx_buffer_eob_seq) through a
  // receive->stage->route->promote->gate sequence where a prev-retransmit of a
  // SHORTER batch's EOB arrives while the current batch's own EOB is lost.
  // Asserts the PRE-FIX single-stage capture reproduces the early ACK-GATE PASS
  // (truncated delivery) AND the POST-FIX staged+match-current-gated promotion
  // prevents it. Returns 0=PASS, 1=FAIL.
  int test_eob_poison_prev_retx();

  // R035 (race audit 2026-06-06) — data_batch_size SHRINK strands active prev.
  // CLI: --test-batch-shrink-strands-prev. Arms an active RSP prev with a frozen
  // expected_count=15 and a RECEIVED slot in [10,15), then SHRINKS via the REAL
  // set_data_batch_size(10) chokepoint (not a direct assign). Asserts the pre-fix
  // gate was unreachable (expected frozen at 15), that the fix re-derives
  // expected/received against the new batch (gate reachable), and that the
  // orphaned slot fires a single streaming_reset (streaming stays active).
  // Returns 0=PASS, 1=FAIL.
  int test_batch_shrink_strands_prev();

  // R029 (race audit 2026-06-06) — stale-retx-queue-cleared-on-recovery test.
  // CLI: --test-retx-clear-on-recovery. Populates retransmit_count>0 with a known
  // OLD bsi, calls the REAL clear_retx_queue() (the single owner every recovery
  // site now invokes), and asserts the queue empties (no pre-recovery bsi
  // reachable), is idempotent on empty, and repeatable across recoveries.
  // Returns 0=PASS, 1=FAIL.
  int test_retx_clear_on_recovery();

  // FIX-6 — RX-delivery drain backpressure regression (the deterministic
  // 61,621-byte stall). CLI: --test-rx-drain-backpressure. FAILS at HEAD
  // 62cb3dc (lossy drain drops popped bytes on a back-pressured non-blocking
  // socket), PASSES after the non-lossy drain. See source/datalink_layer/
  // test_rx_drain.cc + bigblock_p3_hw/_wallb/fix6/.
  int test_rx_drain_backpressure();

  // R030 (race audit 2026-06-06) — v2 PENDING_ACK flip aliasing test.
  // CLI: --test-v2-pendingack-flip-alias. Builds a v2 MIXED batch with the
  // messages_tx[] array-index space DIVERGED from the wire positions (holes + a
  // retx prefix), drives the REAL v2_flip_resolve_slot() for every batch slot,
  // and asserts retx-prefix slots are skipped (-1), new-data slots resolve to the
  // correct diverged array index (not the wire id), no FREE/foreign slot is left
  // PENDING_ACK, and the pre-fix wire-id flip WOULD have poisoned a non-owning
  // slot. Returns 0=PASS, 1=FAIL.
  int test_v2_pendingack_flip_alias();

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
  // FIX-6: non-lossy RX-delivery send (handles non-blocking-socket back-pressure
  // by stashing the unsent tail in rx_deliver_pending). Returns false when the
  // app socket back-pressured and the caller must stop draining for this tick.
  bool rx_deliver_send(const char* src, int length);

  void copy_data_to_buffer();
  // FIX-6 Mouth B: non-lossy producer push into fifo_buffer_rx (drains the app
  // socket to free room, surfaces any residual instead of silently dropping).
  // Returns bytes actually stored (== len on success).
  int fifo_push_rx(const char* buf, int len);
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
  // WALL-B FIX-9 D2 REFINE (_fix9/d2refine/D2_REFINE_DESIGN.md §2): gate the D2 robust
  // reverse-ACK geometry (RSP pre-TX settle + CMD listen-window widen) so it fires ONLY on a
  // RETRANSMIT turnaround, not on a clean first-pass batch (recovers the ~15% clean cost
  // FIX9_D2_RESULTS.md §4.2 measured). data_ack_retx_turnaround = CMD-side: TRUE when the batch
  // the CMD just sent contained retransmitted frames OR a recent CFG16 reverse-ACK was lost
  // (cfg16_revack_starve_fails>0). Set fresh before every post-batch calculate_receiving_timeout;
  // read by calculate_receiving_timeout to gate the widen. ack_tx_retx_turnaround = RSP-side:
  // TRUE on the partial/prev send_mfsk_ack_sack call sites (retx turnarounds), FALSE on the clean
  // first-pass path; read+cleared at the top of send_mfsk_ack_sack to gate the settle. BOTH init
  // FALSE (ctor + session resets) -> a pure-clean session never fires the geometry (byte-identical
  // to D2-off / D3-base). NOT on the wire. See §4 cross-layer audit.
  bool data_ack_retx_turnaround;       // CMD: the data-ACK we are waiting for is a retx turnaround
  bool ack_tx_retx_turnaround;         // RSP: the ACK we are about to key is a retx turnaround
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
  // FIX-8 (data-integrity): reset-surviving high-water mark of the highest
  // batch_seq_id whose bytes were actually DELIVERED to the app FIFO. Advanced
  // ONLY at the two real delivery commits (BATCH-DONE arq_responder.cc:1789-1797
  // and PREV-DELIVERED arq_responder.cc:823) via advance_last_delivered()
  // (monotonic-with-wrap). UNLIKE rsp_current_expected_batch_seq_id this field
  // SURVIVES the BREAK reset (arq_responder.cc:474) and the FULL
  // load_configuration reset — it is cleared ONLY at a true session boundary
  // (ctor + reset_session_state()). The post-reset adopt site
  // (arq_responder.cc:618) reads it to REFUSE silent concatenation across a
  // dropped-batch hole (non-contiguous re-adopt -> loud abort). -1 = nothing
  // delivered yet this LINK. v2-scoped (sack_v2_enabled). See
  // bigblock_p3_hw/_fix8/FIX8_DESIGN.md + FIX8_AUDIT.md.
  int rsp_last_delivered_batch_seq_id;
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
  // V2 FIX-2 (fact-doc §10): count of SACK-completed big-blocks REJECTED at the
  // prev-batch completion because the reassembled-block CRC-32 mismatched (a kept
  // codeword false-passed its per-cw CRC-8). Each reject drops the prev-batch
  // undelivered so the CMD re-emits the block. Diagnostic; logged via
  // [RSP-V2-PREV-BLOCKCRC-REJECT]. Initialized to 0 (init_messages_buffers reset path).
  long long rsp_prev_batch_blockcrc_reject_count = 0;

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

  // FIX-A — ROBUST-tier dwell-batch transport state (data-flow-robust-tier-arq-batch.md
  // §5.2/§5.3). CMD-side staging read by add_message_control(ROBUST_DWELL_BATCH_OP).
  // pending_robust_dwell_batch = the batch the CMD wants the RSP to mirror (the raise
  // target ROBUST_DWELL_BATCH, or 1 on the revert). -1 = unset. robust_dwell_batch_active
  // = true once the CMD has raised to a multi-frame robust batch (so the revert fires
  // exactly once when eligibility is lost). Both reset on connection init / BREAK /
  // config change (reset_session_state + load_configuration revert the batch to 1).
  int pending_robust_dwell_batch;
  bool robust_dwell_batch_active;
  // CMD: decide whether to raise/revert the robust dwell batch and stage the
  // symmetric ROBUST_DWELL_BATCH_OP frame. Called from the clean-data-ACK PARKED
  // path (arq_commander.cc, after FRAME-UP declined to promote). No-op unless the
  // batch actually needs to change. Returns TRUE iff it queued a control frame
  // (connection_status moved to TRANSMITTING_CONTROL) — the caller must then NOT
  // overwrite connection_status with TRANSMITTING_DATA. See §5.2/§5.3.
  bool evaluate_robust_dwell_batch();

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
  // R030 (race audit 2026-06-06): number of leading messages_batch_tx[] entries
  // that are the v2 retx PREFIX (their payload lives in retx_scratch[], NOT in
  // any live messages_tx[] slot — the original slot was freed to ACKED at SACK
  // capture, arq_commander.cc:2983). Set by process_messages_tx_data() on a v2
  // MIXED batch (=R), 0 otherwise. The post-TX PENDING_ACK flip in send_batch()
  // reads it to (a) SKIP retx-prefix frames (no messages_tx slot to flip) and
  // (b) for new-data frames route by (batch_seq_id, low7-seq) instead of the
  // overwritten wire .id. See data-flow-arq-recovery-cluster.md §4.2 / §5.5.
  int v2_retx_prefix_count;

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

  // FIX-6 (non-lossy RX delivery drain). The responder drains fifo_buffer_rx into
  // the app data socket inside process_buffer_data_responder(). The socket is
  // non-blocking; on a full OS send buffer send() returns short / would-block.
  // Pre-fix the popped-and-transformed bytes were DISCARDED (the deterministic
  // 61,621-byte stall). These hold the transformed-but-unsent TAIL so the next
  // ARQ tick re-sends it IN ORDER before popping more raw bytes — converting
  // silent loss into bounded buffering + natural backpressure. The pending unit
  // is the POST-transform stream (correct for B2F, where the raw bytes are
  // consumed by the parser and cannot be pushed back into the raw FIFO — audit
  // R4). Sized MAX_BUFFER_SIZE: a single send unit is what the drain memcpy's into
  // tcp_socket_data.message->buffer (also MAX_BUFFER_SIZE), so the stashed tail can
  // never exceed it; the pop budget (~172 B) and the B2F reroll of one popped chunk
  // are both far under this bound.
  char rx_deliver_pending[MAX_BUFFER_SIZE];
  int  rx_deliver_pending_len = 0;

  cl_telecom_system* telecom_system;

  // CONFIG-HOLDING MEMBERS — MUST be a SIGNED type. These hold config IDs in the
  // ranges CONFIG_0..CONFIG_16 (0..16), ROBUST_0..2 (100..102) AND the sentinel
  // CONFIG_NONE = -1 (common_defines.h:53). They were `char`, which is UNSIGNED on
  // ARM (the Pi testbed) but SIGNED on x86. With unsigned char, a stored
  // CONFIG_NONE reads back as 255, so every `== CONFIG_NONE` / `!= CONFIG_NONE`
  // comparison (arq_commander.cc:239/331/659/683/4405, arq_common.cc:1217/7057,
  // arq_responder.cc:1252-1253) silently inverted on ARM (255 != -1 is TRUE),
  // and a 255 sentinel could be written onto the SET_CONFIG wire (data[2] =
  // reverse_configuration). x86 unit tests pass because signed char compares
  // correctly. Widened to `int` (the type EVERY consumer already takes:
  // config_ladder_index/up/down(int), is_ofdm_config/is_robust_config(int),
  // load_configuration(int,...), and the int CONFIG_* macros) so the sentinel
  // round-trips identically on both ABIs. No memcpy/sizeof/address-of/wire-struct
  // layout depends on the width (audited — scalar by-value use only). See the
  // -Wtype-limits warnings these comparisons used to emit.
  int data_configuration;
  int init_configuration;
  int last_data_configuration;
  int current_configuration;
  int ack_configuration;
  int negotiated_configuration;
  int forward_configuration;   // Commander→Responder TX speed (asymmetric gearshift)
  int reverse_configuration;   // Responder→Commander TX speed (after SWITCH_ROLE)

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

  // The per-batch ACK enhanced-suffix gate (§4 / §21.3): enhanced ONLY at the
  // robust tier (the deep-floor proxy, reusing the gearshift config — no new
  // state). At CONFIG_6+ this is false → the ACK is byte-identical uncoded (the
  // hard throughput-neutrality constraint). This is a THROUGHPUT gate, not a
  // capability negotiation: the enhanced ctrl-suffix is the unconditional
  // default at the robust tier (no CAP_SUFFIX_FEC bit — see
  // datalink_defines.h). NOTE: this is the GATE PREDICATE; the enhanced-ACK TX
  // ENABLE is held off (ARQ_ACK_SUFFIX_FEC_ENABLE=0, §21.3) so the ACK is
  // byte-identical in 100% of cases — flip the enable behind its own HW test.
  bool ack_suffix_fec_eligible() const {
    return is_robust_config(current_configuration);
  }

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

  // ROBUST_0 compression-deadlock fix (data-flow-compress-frame-fill.md §5).
  // Compression can only carry a payload byte if the per-batch budget
  // (data_batch_size * max_frame, minus the crypto tag when active) EXCEEDS the
  // streaming compression header. At ROBUST_0 the budget equals the header
  // (1 * 7 == COMPRESS_HEADER_SIZE), leaving 0 payload room → compress_block()
  // returns -1 forever and the RAW fallback degenerates to a 0-byte payload
  // (the deadlock). This predicate gates BOTH the TX fill (process_buffer_data_
  // commander) and the RX assembly (copy_data_to_buffer) so the two stay a
  // matched pair: when false (robust / tiny-frame), both sides use the
  // headerless uncompressed path and deliver raw bytes; when true (OFDM rungs),
  // both compress. Computed identically on both peers from the shared config +
  // batch invariant (data-flow-batch-size.md §1) — no wire negotiation needed.
  bool compression_viable_for_batch() const
  {
    if(!compression_enabled) return false;
    int eff_long = effective_data_long_header_length(sack_v2_enabled);
    int max_frame = max_data_length + max_header_length - eff_long;
    int batch_capacity = data_batch_size * max_frame;
    if(cipher_suite.is_active()) batch_capacity -= AUTH_TAG_SIZE;
    return batch_capacity > compressor.get_header_size();
  }

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

  // FIX-B — FLOOR-PROBE BACK-OFF state (gearshift-floor-probe-backoff.md §3).
  // probe_backoff_until_ms[i] is the opt_now_ms() timestamp BEFORE which the
  // config at FULL_CONFIG_LADDER index i must NOT be re-probed UP-ward (0 = no
  // back-off armed). Indexed by config_ladder_index(cfg) (range
  // [0,FULL_CONFIG_LADDER_SIZE)). probe_backoff_ms is the CURRENT exponential
  // back-off window — doubled on each repeat up-probe fail, capped at
  // PROBE_BACKOFF_MS_CAP, reset to PROBE_BACKOFF_MS_INIT the instant a clean
  // OFDM batch is delivered. CMD-only state (the climb-control loop lives on the
  // commander; the RSP has no climb decision). All three accessors are PURE
  // helpers (no I/O) so the synthetic-fire unit test drives them with no live
  // channel. INV-4: probe_rung_suppressed() reads opt_now_ms() ONLY — the SAME
  // virtual clock the FTRT sim drives — NEVER cl_timer (which is wall-clock).
  unsigned long long probe_backoff_until_ms[FULL_CONFIG_LADDER_SIZE];
  int probe_backoff_ms;

  // FIX-B predicate — TRUE iff `cfg` is currently under a floor-probe back-off
  // (a proven-failed up-probe whose suppression window has not yet elapsed).
  // PURE / const; reads opt_now_ms() ONLY (INV-4). A cfg not on the ladder
  // (config_ladder_index < 0) is never suppressed (defensive). INV-1: this is
  // AND-ed into the EXISTING up-gate (turning a PERMITTED probe OFF), so it can
  // only ADD suppression — it never unblocks a probe the anchor/+1/ceiling
  // clamps already blocked.
  bool probe_rung_suppressed(int cfg) const {
    int idx = config_ladder_index(cfg);
    if(idx < 0) return false;
    return opt_now_ms() < probe_backoff_until_ms[idx];
  }

  // FIX-B producer — ARM the back-off on `cfg` after a PROVEN-FAILED up-probe.
  // Sets the suppression deadline to now + the current window, then DOUBLES the
  // window (capped) so a repeat fail of the same/another rung waits longer. A
  // cfg not on the ladder is ignored (defensive). NOT const.
  void probe_backoff_arm(int cfg) {
    int idx = config_ladder_index(cfg);
    if(idx < 0) return;
    probe_backoff_until_ms[idx] = opt_now_ms() + (unsigned long long)probe_backoff_ms;
    long long next = (long long)probe_backoff_ms * 2;
    if(next > (long long)PROBE_BACKOFF_MS_CAP) next = (long long)PROBE_BACKOFF_MS_CAP;
    probe_backoff_ms = (int)next;
  }

  // FIX-B producer — RESET the entire back-off (called on a clean OFDM batch:
  // the channel proved an OFDM rung recovered, so no rung is "proven-failed"
  // anymore and the window returns to its INIT value). NOT const.
  void probe_backoff_reset() {
    for(int i=0; i<FULL_CONFIG_LADDER_SIZE; i++) probe_backoff_until_ms[i] = 0ULL;
    probe_backoff_ms = PROBE_BACKOFF_MS_INIT;
  }

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
  // DEEP-SNR DOWN-HYSTERESIS (gearshift-climb-engine.md §10/§11, 2026-05-30) —
  // the MISSING anchor DEMOTION producer. last_data_viable_config (above) only
  // ever RISES (anchor-raise :3454 + ctor/reset init); there was no path to lower
  // it. At the WGN:-10 cliff a slow retransmit-rescued ROBUST/CONFIG_0 batch emits
  // an all-ones completion ACK (arq_responder.cc:~805 prev-path) → CMD raises the
  // anchor to CONFIG_0; CONFIG_0 data then fails → BREAK → break_target_with_anchor
  // (:147) clamps recovery UP to the CONFIG_0 anchor. The breaks>=2 panic bypass
  // NEVER latches because every slow completion resets breaks_since_last_data_
  // success to 0 (:3445) → the counter oscillates 1→0→1→0 → infinite
  // CONFIG_0↔ROBUST_0 thrash (15 BREAKs on hardware). This counter is the SECOND,
  // complementary escape: it counts consecutive BREAKs that fire WHILE
  // current_configuration == last_data_viable_config (i.e. the anchor rung itself
  // is breaking). At K=ANCHOR_DEMOTE_BREAK_FAILS the anchor is DEMOTED one rung
  // (config_ladder_down) and the counter resets — so break_target_with_anchor
  // permits the drop on the next BREAK and the link escapes toward ROBUST_0.
  // Demotion ONLY ever LOWERS the anchor → the +1 up-clamp gets STRICTER, never
  // looser (no af14a9e/3b1726a over-climb regression). Producers: ++ at the BREAK
  // trigger (:3390) when current==anchor; reset to 0 on ANY clean confirmation /
  // anchor-raise (:3445 block). Consumer: the demotion site itself. See §10.
  int anchor_consec_break_fails;
  // K threshold for anchor demotion — matches the existing
  // gear_shift_down_consecutive_fails < 3 house pattern (arq_commander.cc:4793/
  // 4954): require 3 consecutive anchor-rung BREAK failures before demoting, so a
  // single transient anchor-rung failure does not lower a genuinely-viable anchor.
  // TUNABLE.
  static const int ANCHOR_DEMOTE_BREAK_FAILS = 3;
  // SUSTAINED-ANCHOR GATE (gearshift-climb-engine.md §11, 2026-05-30) — closes the
  // leak at the source. The anchor-RAISE (:3454) credited ANY clean confirmation,
  // including the prev-path all-ones completion ACK that fires AFTER a batch is
  // rescued by SACK retransmit ("retransmit-rescued ≠ sustainably viable"). This
  // counts CONSECUTIVE clean confirmations AT THE SAME RUNG; the anchor-raise now
  // requires >= the per-tier threshold (robust=1, OFDM=2). clean_batches_config
  // tracks which rung the streak belongs to: when a clean is credited at a config
  // != clean_batches_config the streak resets to 1 (new rung). Reset to 0 on ANY
  // failed block / BREAK (:3146, :3276 — co-located with consecutive_data_acks=0),
  // which also covers the BREAK trigger downstream of :3276. We do NOT neuter the
  // prev-path completion ACK itself (it keeps the link alive on a genuinely-good-
  // but-lossy channel) — "retransmit-rescued ≠ viable" is encoded HERE in the CMD
  // anchor gate, which still lets a rung that REPEATEDLY delivers clean promote.
  // Producers: incremented in the clean-credit block (:3442); reset at the failure
  // sites + on rung change. Consumer: the anchor-raise gate (:3454). See §11.
  int clean_batches_at_current_config;
  int clean_batches_config;  // the rung clean_batches_at_current_config counts for
  // Per-tier sustained-anchor thresholds. Robust (batch=1): ONE clean MFSK frame
  // is strong proof the rung carries data — keep the climb fast off ROBUST_0. OFDM
  // tier: a single retransmit-rescued batch can't anchor a non-sustainable rung —
  // require TWO consecutive clean batches. Both TUNABLE.
  static const int SUSTAINED_ANCHOR_N_ROBUST = 1;
  static const int SUSTAINED_ANCHOR_N_OFDM   = 2;
  // ADAPTIVE FRAME-UP THRESHOLD (gearshift-climb-engine.md §12, 2026-05-30) — the
  // FAST-probe target for frame_shift_threshold once a rung is PROVEN sustained-
  // clean. 1 = step up on the very next clean batch (the forward-AARF half af14a9e
  // deleted). The back-off half still DOUBLES the member on FRAME-UP failure
  // (:2302/:3180/:3359); effective_frame_shift_threshold() returns this value ONLY
  // when fast_probe_clean_streak() is met, else the (possibly doubled) member —
  // so the climb is fast when clean and conservative when marginal. TUNABLE.
  static const int FRAME_SHIFT_FAST = 1;
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
  // WALL-B FIX-5 (fix5/FIX5_DESIGN.md §4.1, WALLB_DIAGNOSIS.md §1.6): CFG16 big-block
  // carve COOLDOWN. CMD-ONLY (never on the wire — INV-B2). After a FIX-4 carve-viability
  // demote (arq_commander.cc:3669), the CFG16 big-block rung is proven non-viable on THIS
  // channel; cap the climb at CFG15 for `bigblock_carve_cooldown_batches` completed batches
  // so the SNR re-trigger / turbo forward / ladder-up CANNOT re-elect the carve-dead rung.
  // UNLIKE supershift_proven_ceiling (which finish_turbo_direction() resets to the probe
  // top at :4147), this field is NOT touched by the turbo state machine, so it SURVIVES the
  // BREAK->ROBUST_0 collapse + full turbo re-climb that is the limit cycle.
  // WALL-B FIX-9 D3 GENERALIZES the role: the cooldown now means "CFG16 not viable on THIS
  // channel" from EITHER cause — the big-block carve being dead (FIX-4) OR the per-frame reverse-
  // ACK being STARVED by clock-drift turnaround de-alignment (D3, FIX9_D3_DESIGN.md). Both arm
  // sites mean the same thing to every consumer below.
  // Producers: ARM/extend at the FIX-4 carve demote (arq_commander.cc:~3672) AND at the FIX-9 D3
  // reverse-ACK-starvation demote (the per-frame sibling, same helper + prev-span discipline);
  // DECREMENT per completed batch at the per-batch eval tail (the ceiling_success_count cadence);
  // CLEAR-on-success at the data-ACK reset (:3804) GATED on
  // current_configuration==CONFIG_16 && big-block framing live (INV-B3 — a per-frame CFG15
  // data-ACK must NOT clear it; under D3's per-frame regime NO carve ever lands so the hold
  // survives the CFG15 window and expires by batch-count for an optimistic re-probe); INIT 0 in
  // ctor + reset_session_state (R3). Consumers:
  // apply_bigblock_cooldown_cap() at the climb hooks (gearshift LADDER-UP/CEILING-RECOVERY,
  // turbo SNR-SUPERSHIFT, elevator_target_from_snr, finish_turbo_direction start_config).
  int bigblock_carve_cooldown_batches{0};  // >0 = CFG16 election suppressed this many more batches
  int bigblock_carve_cooldown_span{0};     // last window length, for AARF exponential growth
  // WALL-B FIX-9 D3 (bigblock_p3_hw/_fix9/FIX9_D3_DESIGN.md §3): consecutive block-failures at
  // CONFIG_16 whose reverse MFSK ACK+SACK correlator was PURE SILENT (ack_diag_peak_metric==0.0).
  // This is the PER-FRAME reverse-ACK STARVATION discriminator: the inter-Pi clock drift de-aligns
  // the half-duplex CFG16 turnaround so the reverse ACK lands outside the CMD's window (a ZERO, not
  // a garbled match). CMD-ONLY (never on the wire). Producers: ++ at the block-failure path
  // (arq_commander.cc ~3666) gated on current==CFG16 && ack_diag_peak_metric==0.0; RESET to 0 on
  // ANY data-ACK (clean OR partial — a partial proves the reverse channel is not silent) and on any
  // config != CFG16 and when D3 consumes the deadline; INIT 0 in ctor + both session resets (R3
  // parity with bigblock_carve_cooldown_batches). Consumer: the D3 discriminator
  // (cfg16_revack_starve_fallback_target) at the block-failure decision point. The demote it
  // triggers REUSES the FIX-5 bigblock_carve_cooldown_* machinery to hold the climb at CFG15
  // across cycles (the field's role generalizes from "carve-dead" to "CFG16-not-viable-on-this-
  // channel"; both arm sites mean the same thing to every cooldown consumer). See FIX9_D3_AUDIT.md.
  int cfg16_revack_starve_fails{0};
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
      // §5 LANDMINE (sim-arq-channel.md): the Q-table effective-rate optimizer
      // measures bytes/time using THIS clock (opt_on_batch_tx_start ->
      // opt_batch_wire_ms -> get_current_effective_rate_bps). Under -x sim the
      // channel runs faster than wall-clock, so reading the wall-clock here
      // would make the optimizer see bytes/wall-time and report a rate ~50x
      // too high -> garbage config selection. Route it through the SAME virtual
      // clock the cl_timers use so the optimizer measures bytes/CHANNEL-time.
      // sim_clock_now_ns() falls back to steady_clock when sim is disabled, so
      // production is byte-identical to the previous steady_clock read.
      return (unsigned long long)(sim_clock_now_ns() / 1000000ULL);
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

  int get_configuration(double SNR);  // returns CONFIG_0..16 (never CONFIG_NONE); int for type-consistency with the config members it feeds
  void load_configuration(int configuration, int level, int backup_configuration);
  void switch_narrowband_mode(int nb_enabled);
  void return_to_last_configuration();
  int init_messages_buffers();
  int deinit_messages_buffers();
  void check_buffer_canaries(const char* caller);

  char last_received_message_sequence;
  int last_received_end_of_batch_seq;  // End-of-batch flag: seq# of frame with bit 7 set, or -1
  // R038 (race audit 2026-06-06): per-frame EOB STAGING for the v2 path.
  // receive() decodes the EOB bit BEFORE the responder knows whether the frame
  // is match-current / match-prev / drop. Writing last_received_end_of_batch_seq
  // pre-routing let a CRC-valid prev-retransmit / late-duplicate of a SHORTER
  // batch poison the CURRENT batch's effective_batch (early ACK-GATE PASS ->
  // truncated delivery). For v2, receive() now stages the decoded EOB seq here
  // (or -1 if the frame has no EOB bit) and the responder promotes it to
  // last_received_end_of_batch_seq ONLY inside the confirmed match-current
  // storage block. v1 (no bsi routing) keeps writing last_received_end_of_batch
  // _seq directly in receive() and leaves this field unused (-1).
  int rx_buffer_eob_seq;               // staged EOB seq for current v2 frame, or -1
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
