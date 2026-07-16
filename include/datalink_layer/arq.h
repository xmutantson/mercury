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
#include <cstdlib>
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
// pumped_settle_wait(wait_ms) — clock-faithful settle-wait, THREE paths, all
// sharing the SAME exit predicate (elapsed >= wait_ms); only the clock-advance
// MECHANISM differs:
//   (1) Production / HW (sim_clock_enabled()==0): verbatim msleep(wait_ms) —
//       BYTE-IDENTICAL to the stock wall settle-wait.
//   (2) Two-process paced sim (-x sim, no pump, sim_clock_enabled()==1): a
//       cl_timer loop on the VIRTUAL clock (sim_spin_sleep() yields to the
//       concurrent capture/RX-bridge thread that advances the shared
//       sample-counter clock). The handshake DEADLINES this wait pairs with are
//       virtual cl_timer reads; the OLD code wall-slept here, which desynced
//       the wait from its deadline under host CPU load and dropped the RSP
//       reply outside the CMD window (the connect-under-load race —
//       fix/sim-connect-virtual-clock).
//   (3) SIM_INPROC (pump installed): a cl_timer + step-pump loop, the pump
//       advancing the shared clock so a peer instance sees time pass.
// The exit SEMANTICS are unchanged on all three. Used for the B1-B4 / B7/B8
// turnaround + HAIL-race settle guards. For B7 the CALLER keeps the delay
// FORMULA verbatim (Bug #55 HAIL reliability); this helper only routes the
// already-computed wait_ms through the correct clock.
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
// D5 (TRACK_C_D2D3D5_DESIGN.md §5.3): `with_d5` selects whether the v2 header
// includes the batch_total_frames byte. It is TRUE for OFDM multi-frame configs
// (where lost-EOB batch truncation is possible and the +1 byte is negligible),
// FALSE for robust / batch=1 configs (D5 is meaningless at batch=1 and the byte
// would steal the scarce ROBUST_0 payload — see test_robust0_compress_deadlock
// C0). Default TRUE so the prevailing OFDM call sites are unchanged; the robust
// path + the load_configuration sizing pass the live header_carries_d5 flag.
inline int effective_data_long_header_length(bool sack_v2, bool with_d5 = true)
{
	if(!sack_v2) return DATA_LONG_HEADER_LENGTH;
	return with_d5 ? DATA_LONG_HEADER_LENGTH_V2 : DATA_LONG_HEADER_LENGTH_V2_NO_D5;
}
inline int effective_data_short_header_length(bool sack_v2, bool with_d5 = true)
{
	if(!sack_v2) return DATA_SHORT_HEADER_LENGTH;
	return with_d5 ? DATA_SHORT_HEADER_LENGTH_V2 : DATA_SHORT_HEADER_LENGTH_V2_NO_D5;
}

// (B) LOUD BACKSTOP — CMD>RSP batch-size desync detector (silent-corruption-residual.md
// §7-B, data-flow-batch-size.md §8). `sender_total_frames` is the SENDER-declared per-batch
// frame count carried on EVERY DATA frame (D5 batch_total_frames = message_batch_counter_tx),
// as the RSP sees it at the ACK-GATE (rx_batch_total_frames). `local_batch` is the RSP's own
// data_batch_size. When the load-bearing invariant CMD data_batch_size == RSP data_batch_size
// holds, the CMD can never pack MORE frames than its own (== the RSP's) batch size, so
// sender_total_frames > local_batch is a PRECISE, false-positive-free signal that the CMD
// built a LARGER batch than the RSP applied (the CMD>RSP desync — CMD=30, RSP=25 in res_c3100).
// Delivering the batch at the smaller local size TRUNCATES it and silently ORPHANS the
// sender's surplus frames -> a permanent one-batch stream shift. D5 is sack_v2-only (robust /
// batch=1 configs never carry the byte, so sender_total_frames stays <=0 there and this
// returns false — the backstop is inert at robust, where the batch is pinned symmetric).
// true => the RSP must NOT silently deliver; raise [RSP-V2-BATCHSIZE-DESYNC] + abort so the
// mismatch surfaces LOUD and the link resyncs, never a silent orphan (integrity D0).
inline bool batchsize_desync_detected(int sender_total_frames, int local_batch, bool sack_v2)
{
	return sack_v2 && sender_total_frames > 0 && sender_total_frames > local_batch;
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

// R2a/R2c — bounded recovery depth for a recoverable one-frame prev-batch hole.
// RSP holds the completed current batch and re-requests the prev hole this many
// times before falling through to the terminal gap-abort; the CMD retains the
// prev-batch frame bytes for the same number of re-drive rounds. Small: HF loses
// the same frame 3× in a row rarely, and a larger bound just delays the loud
// abort on a genuinely dead peer. See data-flow-recoverable-gap-abort.md.
#define RSP_GAP_RECOVER_MAX  3
#define CMD_PREV_RETAIN_MAX  8   // retention-shadow slots (>= one batch's worst-case holes)

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
// R6 measured-turnaround estimator (RFC 6298 / Jacobson RTO). TT_WINDOW_FLOOR_MS
// is a small fixed guard added to SRTT + K*RTTVAR; TT_CLOCK_G_MS is the RFC 6298
// §4 clock granularity G so the variance term never rounds below it. See
// fact-documents/data-flow-turnaround-timers.md.
#define TT_WINDOW_FLOOR_MS     300  // fixed guard on the measured RTO window
#define TT_CLOCK_G_MS          20   // RFC 6298 §4 clock granularity G

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
  // Fix A (baseline-double-delivery.md): apply a data_batch_size shrink that was
  // DEFERRED because it would have orphaned already-RECEIVED prev-batch frames.
  // No-op unless a shrink is pending AND the prev batch is now inactive.
  void rsp_apply_deferred_batch_shrink();
  // Fix A: if shrinking data_batch_size to `target` would orphan already-RECEIVED
  // prev-batch frames in [target,old), record the deferred target and return true
  // (caller must NOT apply the shrink now). Returns false (apply as normal) when
  // there is no active prev, no orphan, it is not a shrink, or the env defeat is set.
  bool defer_shrink_if_would_orphan_prev(int target);
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
  // Timing redesign (cross-layer data-flow audit, retx-queue cluster) — after
  // the v2 mixed-batch fill leaves the retx block at slots [0..R-1] and the
  // new-data frames at [R..R+ND-1], rotate the sub-array [0..R] right by one so
  // the wire order becomes [new0][retx0..retxR-1][new1..newND-1(EOB)]. This keeps
  // a retransmit frame off wire slot 0 (the only slot exposed to the peer's
  // post-turnaround mute/flush window) while the EOB-marked frame stays
  // wire-final. No-op (legacy layout) unless it is a v2 mixed batch with >=2
  // new-data frames. Returns true iff the rotation was applied. Shared by the
  // production builder and the --test-retx-slot-order regression.
  bool v2_rotate_retx_behind_lead();
  void set_control_batch_size(int control_batch_size);
  void set_role(int role);
  void calculate_receiving_timeout();
  // R6 measured-turnaround estimator (RFC 6298 §2; Karn & Partridge 1987): map
  // the live config + forward-batch airtime to an SRTT/RTTVAR bucket and fold a
  // clean reverse-ACK arrival sample. See fact-documents/
  // data-flow-turnaround-timers.md.
  int  tt_class_of(int cfg) const;
  int  tt_batchbk_of(int batch_airtime_ms) const;
  bool tt_karn_sample_ok() const;
  void update_turnaround_estimate(int cfg, int batch_airtime_ms, int rtt_ms);
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

  // --- Hybrid ML-KEM KX chunk transport helpers (MLKEM_HYBRID_PLAN.md §4) ----
  // Per-frame chunk payload capacity = the control-frame data width minus the
  // 4-byte KX chunk header. Derived from the SAME geometry the X25519 KX uses
  // (max_data_length + max_header_length - CONTROL_ACK_CONTROL_HEADER_LENGTH).
  int  kx_chunk_payload_capacity();
  // Begin a chunked send of kx_mlkem_pk (KEY_EXCHANGE_2) or kx_mlkem_ct
  // (KEY_EXCHANGE_3). Sets up kx_tx_* state and queues the FIRST chunk frame.
  // Returns 0 on success, -1 on error (e.g. source not ready).
  int  kx_begin_chunk_send(int kind);
  // Queue the next pending KX chunk control frame (kx_tx_next_index). Returns 1
  // if a frame was queued, 0 if all chunks have been sent, -1 on error.
  int  kx_send_next_chunk();
  // Process a received KEY_EXCHANGE_2/3 control frame (data[] holds the 4-byte
  // header + chunk payload). Validates + reassembles into kx_data_buf. Returns
  // 1 when the full buffer is now complete (last needed chunk arrived), 0 when
  // more chunks are still needed, -1 on a rejected chunk (CRC/kind/index/len).
  int  kx_receive_chunk(const uint8_t* frame, int frame_len, int expect_kind);

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
  // R1 turnaround-clearance guard consumer (called at send_batch() top before keying)
  // and its in-process regression driver (--test-turnaround-guard; also runs in --test).
  void turnaround_clearance_wait();
  // R1-rescope arming (producer, CMD-side): stamp the clearance timer + mark the next data
  // batch as post-control-turnaround. Called at a control-ACK -> data transition when the
  // ACKed op renegotiated the link geometry (batch size / config).
  void arm_control_turnaround_guard();
  int  test_turnaround_guard();
  int  test_measured_timers();  // R6 SRTT/RTTVAR estimator + ack-timeout invariant regression
  // Level 3: TX short tone pattern instead of LDPC ACK. control_ack=true marks a
  // BREAK-recovery / SET_CONFIG control-ACK turnaround — the ONLY caller that
  // opts into the robust noncoherent-repeat ACK when MERCURY_RECOVERY_ACK_ROBUST
  // is set (recovery-ack-robustness.md §4). Data-ACK callers pass false (default)
  // → single block, no airtime change. Default false → byte-identical.
  void send_ack_pattern(bool control_ack = false);

  // RECOVERY-ACK robustness (recovery-ack-robustness.md §4/§6.2): set the RX
  // combine_reps the next ACK-pattern wait will correlate with. control_ack=true
  // + MERCURY_RECOVERY_ACK_ROBUST + ack_pattern_time_ms>0 → RECOVERY_ACK_REPS
  // (combine the repeated control-ACK); otherwise → 1 (single block). No-op /
  // byte-identical when the flag is off. CMD-side.
  void set_recovery_ack_reps_for_wait(bool control_ack);
  // RECOVERY-ACK robustness (recovery-ack-robustness.md §6.3, recovery-window
  // coupling): re-derive the ms-mirror ack_pattern_time_ms from the (possibly
  // rep-bumped) telecom ack_pattern_passband_samples, using the SAME ceil formula
  // as load_configuration. The bare reps bump updates ack_pattern_passband_samples
  // but leaves ack_pattern_time_ms at its R=1 value; calculate_receiving_timeout's
  // recovery/CMD listen-window geometry reads the ms-mirror, so without this the
  // window is sized for a 390 ms ACK while the RSP keys a 1557 ms R=4 ACK. Called
  // from set_recovery_ack_reps_for_wait after every rep change. Byte-identical when
  // reps stay 1 (recomputes the same 390). CMD-side.
  void recompute_ack_pattern_time_ms();
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
  // multiwindow_scan (CONNECT round-2 fix #2(a)): when true, on a miss in the
  // newest-tail snapshot, additionally run the SAME unchanged ACK correlator at
  // several OLDER tail-offset phases stepping back through the retained ring
  // history, accepting on the first phase that clears the SAME thresholds. This
  // closes the CMD-side control-ACK capture-window-phase miss (sub-mode B): the
  // ACK burst persists in the ~1301-symbol ring far longer than the newest
  // 80-symbol tail, so a gap-displaced ACK is still found regardless of which
  // ftr==0 phase the snapshot fired on. Scoped to the CONNECT handshake control
  // ACK only (arq_commander.cc:1981); DATA-ACK/BREAK/HAIL pass false ->
  // byte-identical. Does NOT restart receiving_timer or shrink ftr.
  bool receive_ack_pattern(bool defer_audio_advance = false,
                           bool multiwindow_scan = false);

  // Multi-window DATA-ACK/SACK correlator (Track A, mwcorr;
  // fact-documents/data-flow-data-ack-sack-correlator.md). The steady
  // SACK-suffix probe in process_messages_rx_acks_data() only correlates the
  // NEWEST tail of the capture ring; on a long held-CFG16 forward batch the
  // reverse ACK+SACK arrives once, late and mis-phased, then trailing idle
  // silence scrolls it out of the newest tail before the snapshot fires
  // (bench-9 matched=0/7, peak_metric=0.00). The burst is NOT lost — the
  // double-mapped ring (data_container.cc:170) retains ~buffer_Nsymb (~1301)
  // symbols, far more than the ACK round-trip, so it sits at an OLDER phase.
  //
  // mw_find_ack_sack_phase() steps the search phase back through the retained
  // ring history in ACK-pattern-length strides, energy-gating each phase, and
  // runs the SAME decode_ack_sack_from_passband() + CRC12 check at each
  // energetic phase. It accepts the FIRST older phase whose decode passes CRC12
  // and returns its tail-offset in *chosen_off (>=0 on a hit, -1 on a genuine
  // all-silence/all-CRC-fail miss). It ONLY locates a phase — the caller
  // re-snapshots that phase and re-runs the EXACT existing decode body + the
  // bsi-window/bitmap/dedupe sanity verbatim, so acceptance semantics are
  // unchanged. It writes NOTHING to the ring or frames_to_read; it reads the
  // ring under capture_prep_mutex exactly as the existing snapshot does.
  // Gated by mw_data_ack_multiwindow_enabled() (env MERCURY_DATA_ACK_MULTIWINDOW,
  // default OFF -> caller is byte-identical to monitor 627c370).
  bool mw_find_ack_sack_phase(int rwi, int tail_offset, int tail_samples,
                              int sym_samples, int pattern_len, int* chosen_off);

  // Cached env gate for the multi-window DATA-ACK/SACK scan. Reads
  // MERCURY_DATA_ACK_MULTIWINDOW once (no getenv in the hot poll loop).
  bool mw_data_ack_multiwindow_enabled();

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

  // R2a — re-send the PARTIAL SACK for the already-sealed prev batch
  // (rsp_prev_batch_seq_id) so the CMD re-drives the missing frame that is
  // holding the current batch. Rebuilds the hole bitmap from messages_rx_prev[]
  // RECEIVED status and emits it on the same MFSK/OFDM SACK transport the live
  // partial path uses, but for the PREV bsi. Unlike the normal partial-SACK
  // path it does NOT call bump_bsi_and_transfer_prev() (the prev is already
  // sealed) and does NOT touch the current batch. No new wire format. Returns
  // true if a SACK was emitted. See data-flow-recoverable-gap-abort.md §5.
  bool rsp_resend_prev_partial_sack();

  // FIX-8 (data-integrity): advance rsp_last_delivered_batch_seq_id to `bsi`
  // ONLY if `bsi` is a forward step (mod-256 forward distance in [1,128]) from
  // the current mark, OR the mark is unset (-1). MONOTONIC-with-wrap (audit R1):
  // a late out-of-order prev (older) batch delivered after a newer current
  // already advanced the mark must NOT regress it. Called at the two real
  // delivery commits only (BATCH-DONE + PREV-DELIVERED). Internally v2-scoped by
  // the call sites; the helper itself is pure arithmetic.
  void advance_last_delivered(int bsi);

  // R2b — the SINGLE in-order current-batch delivery commit, shared by the
  // BATCH-DONE gate (arq_responder.cc process_messages_acknowledging_data) and
  // the PREV-completion deliver-held-cur site (process_messages_rx_data_control).
  // Marks messages_rx[] RECEIVED->ACKED over the batch span, advances the
  // contiguity high-water to the current bsi, rolls prev<-cur / cur<-cur+1,
  // resets the wired frame count, and pushes the ACKED slots to the app FIFO via
  // the production copy_data_to_buffer() primitive. ONE delivery implementation
  // under the D3.1 gates — the caller has already cleared the contiguity ruler
  // (delivery_step_is_gap == not-a-gap) by construction. See
  // fact-documents/data-flow-recoverable-gap-abort.md §5.5.
  void rsp_commit_cur_batch_delivery();

  // In-band unilateral DEMOTE-REBASE (arq_responder.cc SET_CONFIG handler): a real
  // mid-transfer config change re-baselines the RSP bsi window (cur=prev=-1, drop
  // the in-flight prev partial storage) so the next data frame re-adopts through
  // the gap-gate, while DELIBERATELY preserving rsp_last_delivered_batch_seq_id and
  // rx_stream_emitted_bsi_hw. Extracted so the directed regression (test_dedup_rebase)
  // drives the EXACT production rebase, not a state poke. Guard-internal: a no-op
  // when sack_v2 is off or the window is already re-baselined.
  void rsp_inband_demote_rebase();

  // Option B' (data-flow-batch-size.md §9): the RX ACCEPTANCE WINDOW. Trusts the
  // CRC-protected per-batch sender-declared frame count (D5, rx_batch_total_frames
  // / rx_buffer_batch_total_frames) when it EXCEEDS the stale command-synced
  // data_batch_size, so a lost/late SET_LINK_PARAMS can no longer truncate a batch
  // the sender built LARGER (the res_c3100 CMD>RSP silent-shift fault). Returns
  // max(data_batch_size, min(sender_total_frames, MAX_SACK_BATCH_SIZE)) — it NEVER
  // narrows below data_batch_size, so every non-desync batch (matched, step-down,
  // compression, encryption, robust, all-retx, D5-absent) is BYTE-IDENTICAL to the
  // pre-B' behavior. MERCURY_BPRIME_DEFEAT restores the pre-B' window (==
  // data_batch_size) AND the pre-B' (B)-backstop trigger (fire on > data_batch_size).
  int  rx_effective_window(int sender_total_frames) const;
  // MERCURY_BPRIME_DEFEAT env gate (byte-identical-to-pre-B' when set).
  static bool bprime_defeat_active();

  // D3.1 (data-integrity): the shared LOUD GAP-ABORT teardown. The
  // case-independent action both the FIX-8 re-adopt gate and the new
  // delivery-time gate route to: control-port error, DROPPED, clear the bsi
  // family + prev-buffer + carve-arm, reset_session_state(). Centralizes the
  // block previously inlined at arq_responder.cc:646-688 so the delivery-time
  // commits and the SET_CONFIG re-baseline path use the IDENTICAL teardown (no
  // drift between sites). `reason` is the control-port error string.
  // See bigblock_p3_hw/_d31_fade/D31_INORDER_DESIGN.md §2.
  void rsp_gap_abort_teardown(const char* reason);

  // Reconnect-continuity fail-closed arm (data-flow-reconnect-continuity.md §5b). Called at the
  // fresh RSP START_CONNECTION accept (both the callsign-matched and passive-monitor sites). Arms
  // rsp_cross_session_seam_armed iff the app DATA socket is persistent (tcp_socket_data ACCEPTED,
  // or the test-forced signal), the prior session delivered app bytes (rsp_prev_session_app_
  // delivered > 0), and no negotiated resume proved byte-continuity. MERCURY_RECONNECT_FAILCLOSED_
  // DEFEAT=1 leaves it disarmed (the pre-fix silent splice). No effect on a fresh transfer / a
  // continuous session (prev-delivered 0 -> stays disarmed).
  void rsp_reconnect_seam_arm_on_accept();

  // Reconnect-continuity fail-closed F2 (data-flow-reconnect-continuity.md 5b). Called ONLY on a
  // PROVEN-CLEAN end-of-transfer (the CLOSE_CONNECTION EOT-verified branch), AFTER reset_session_
  // state() has re-snapshotted the app-delivered high-water. Clears rsp_prev_session_app_delivered
  // (and any stale arm) so a legitimate back-to-back transfer on the SAME persistent app socket is
  // byte-identical. An abort / short / absent-EOT close does NOT call this (fail-closed default).
  void rsp_seam_clear_on_clean_eot();

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

  // Option B (data-flow-compact-confirm.md): emit the COMPACT coded reverse
  // confirm for a CLEAN (all-ones) batch — ACK base (16) + K=5 GF(16)-RA
  // codeword (10) carrying [bsi:8|crc12:12]. ~3 symbols (~70ms) shorter than the
  // 13-uncoded ACK suffix AND ~4 dB more robust (cliff measured -4.2 dB deeper).
  // Returns wall-clock TX ms, or 0 if unsupported (NB / compiled out). CRC12 is
  // over the single [bsi] byte. CLEAN-batch only — partial loss uses send_sack.
  long long send_mfsk_compact_confirm(unsigned char batch_seq_id);

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
  // Fix C / H#1 (data-flow-fifo-backup.md §6.1): ACK-confirm-keyed backup flush.
  // finalize_block_commander() is the normal backup flush but it is SKIPPED whenever a
  // control frame is queued (a SUPER-ACK/turboshift SET_CONFIG queued in the SAME poll
  // the last data frame ACKs — Root B) or new-data is staged, so the delivered batch's
  // raw survives in fifo_buffer_backup and the config-change re-stage re-sends it
  // (double-delivery; the backup also ACCUMULATES delivered raw across a lossy run).
  // This flushes the backup at the TRUE ACK-confirm boundary, but ONLY when there is
  // ZERO un-confirmed data anywhere (no PENDING_ACK/ACK_TIMED_OUT/ADDED_TO_LIST/
  // ADDED_TO_BATCH_BUFFER frame and retransmit_count==0) so the backup can hold only
  // already-delivered raw — INV3 (never drop un-confirmed) is preserved by construction.
  // Idempotent (finalize's later flush is a no-op). MERCURY_BACKUP_CONFIRMFLUSH_DEFEAT=1
  // reverts on the SAME binary (the fail-before arm reproduces the re-stage re-delivery).
  void maybe_backup_confirm_flush();
  int test_backup_confirm_flush();  // --test-backup-confirm-flush: Fix C/H#1 re-stage double-delivery regression

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

  // Option B (data-flow-compact-confirm.md §5): peek the passband tail for a
  // CRC12-valid, in-window COMPACT confirm (K=5 GF(16)-RA, N=10, carrying
  // [bsi:8|crc12:12]). Structurally cloned from cmd_clean_data_ack_crc_valid but
  // (a) decodes the compact codeword via decode_compact_confirm_from_passband,
  // (b) CRC12 is over the single [bsi] byte (CLEAN/all-ones is implicit in the
  // confirm type — no bitmap), (c) bsi-in-window gate identical. Read-only peek.
  // Returns false on NB, no decode, CRC mismatch, or out-of-window bsi. The
  // commander tries this FIRST (cheaper/shorter/deeper) then the 13-uncoded
  // clean-data-ACK path; the two CRC12s are over different fields so neither can
  // cross-validate the other.
  //
  // out_bsi (optional): on a TRUE return, the decoded in-window batch_seq_id LSB
  // is written here so the SACK-window Branch-2 caller (data-flow §10.5 fix-b) can
  // dedupe (sack_clean_confirmation_accepted) and disarm the climb re-tag
  // (inband_retag_confirm_from_sack) on the SAME bsi the 13-uncoded clean branch
  // would. Untouched on a FALSE return. Pass NULL (the default) when the decoded
  // bsi is not needed (the SACK-closed Branch-3 arm).
  bool cmd_compact_confirm_crc_valid(uint8_t* out_bsi = nullptr);

  // Option B fix-b (data-flow-compact-confirm.md §10.4/§10.5): the SHARED predicate
  // the Branch-2 SACK-window probe (process_messages_rx_acks_data) consumes to accept
  // a COMPACT confirm INSIDE the SACK window — the SECOND live-land defect for a
  // batch>1, SACK-on, clean, WB session (RSP emits compact, but the window probe
  // decoded only the 13-uncoded ACK+SACK -> compact's [bsi]-CRC12 fails the 5-byte
  // CRC12 -> dropped -> 4.7x batch>1-WB regression at ENABLE=1). Tries the
  // SELF-VALIDATING compact decode FIRST (cmd_compact_confirm_crc_valid: own snapshot
  // + base detect + GF(16) soft-decode + CRC12-over-[bsi] + bsi-in-window — the CRC12,
  // not the SACK window, is the false-confirm guard), then routes a valid CLEAN confirm
  // through the EXACT 13-uncoded CLEAN-branch state (split-dedupe, cmd_last_applied_clean_bsi,
  // inband_retag_confirm_from_sack, v2_ack_pat_pre_detected=true). Returns true iff a
  // compact confirm was accepted/de-duplicated this poll (caller sets mfsk_handled_this_poll
  // to suppress the 13-uncoded decode AND the OFDM dispatch); false on compact MISS / not
  // enabled / NB / out-of-window bsi (caller falls through to the UNCHANGED 13-uncoded
  // decode). No commit_ack_pattern_consumed()/frames_to_read mutation (v2 clean path
  // routes via v2_ack_pat_pre_detected; the skipped OFDM dispatch owns frames_to_read).
  //   compact_enabled : ARQ_COMPACT_CONFIRM_ENABLE in production (held off => returns
  //                     false immediately, byte-identical to the pre-fix path); the test
  //                     forces true. Shared by test_compact_confirm_sack_window_rx_path.
  //   out_pre_detected: the caller's v2_ack_pat_pre_detected LOCAL (the CLEAN-funnel
  //                     routing flag) — set true ONLY on a FRESH (non-duplicate) CLEAN
  //                     compact accept; left untouched on a duplicate / miss. May be NULL.
  bool cmd_compact_confirm_sack_window_accept(bool compact_enabled,
                                              bool* out_pre_detected = nullptr);

  // Option B clean DATA-ACK accept arm (the SHARED predicate consumed by the
  // "Data ACK pattern detected" else-if in process_messages_commander, and driven
  // directly by test_compact_confirm_live_rx_path). Returns true iff a CLEAN data
  // ACK is acceptable THIS poll on the bare-arm path (outside the SACK window).
  //
  // ROOT-CAUSE FIX (data-flow-compact-confirm.md §9): the compact confirm is a
  // SELF-VALIDATING frame (its own 16-sym base detect + GF(16) soft-decode + CRC12
  // over [bsi] + bsi-in-window, all inside cmd_compact_confirm_crc_valid()). It must
  // be tried DECOUPLED from receive_ack_pattern() — the CRC-less bare 7/16 presence
  // gate that exists ONLY to protect the legacy bare ACK. The 26-sym compact frame
  // the bare gate's 8-symbol energy pre-gate misses (the §9 defect) so gating the compact
  // decode behind it (the old `receive_ack_pattern() && (...compact...)` chain)
  // missed every confirm. The CRC12 — not the bare count — is the false-confirm
  // protection for the compact path, so decoupling does NOT re-open false-confirm.
  // On a compact accept the ring is advanced via commit_ack_pattern_consumed() (the
  // bare gate's frames_to_read=4 that the compact path now bypasses).
  //
  // The legacy bare ACK arm (receive_ack_pattern() + WB CRC content gate
  // cmd_clean_data_ack_crc_valid()) is UNCHANGED.
  //   sack_window_open  : the caller's local SACK-window flag (the bare arm is
  //                       outside the SACK window — passed in, not a member).
  //   compact_enabled   : ARQ_COMPACT_CONFIRM_ENABLE in production; the test forces
  //                       true to exercise the wiring with the master gate held off.
  //   use_legacy_chain  : FAIL-BEFORE only — restore the OLD compact-behind-bare-gate
  //                       order so the test can reproduce the missed confirm.
  bool cmd_compact_confirm_live_accept(bool sack_window_open,
                                       bool compact_enabled,
                                       bool use_legacy_chain = false);

  // Option B compact coded reverse-confirm LIVE RX-PATH regression (CLI
  // --test-compact-confirm-rx; also in `--test`). Drives the FULL live RX path
  // (passband -> commander capture ring -> the production accept predicate
  // cmd_compact_confirm_live_accept), NOT a direct decoder call. FAIL-BEFORE
  // (use_legacy_chain via MERCURY_COMPACT_RX_FAILBEFORE=1): the compact frame is
  // rejected by the bare 7/16 gate. PASS-AFTER: accepted via its own CRC-gated
  // decode; ACK+SACK still accepts; noise/out-of-window/cross-frame all reject.
  // source/datalink_layer/test_compact_confirm_rx.cc. Returns 0 PASS / 1 FAIL.
  int test_compact_confirm_live_rx_path();

  // Option B fix-b LIVE RX-PATH regression for the SACK-window-open case
  // (data-flow-compact-confirm.md §10.4/§10.5). Drives the SHARED Branch-2 predicate
  // cmd_compact_confirm_sack_window_accept() exactly as production does, for the
  // batch>1, SACK-on, clean, WB scenario. FAIL-BEFORE
  // (MERCURY_COMPACT_SACK_WINDOW_FAILBEFORE=1): simulates the pre-fix Branch-2 (only
  // the 13-uncoded decode runs in-window) -> the compact tail is REJECTED. PASS-AFTER
  // (default): the decoupled compact decode accepts it (v2_ack_pat_pre_detected set);
  // a corrupted-suffix compact is rejected; a 13-uncoded ACK+SACK is NOT cross-accepted
  // as compact; out-of-window bsi rejected. source/datalink_layer/test_compact_confirm_rx.cc.
  // Returns 0 PASS / 1 FAIL.
  int test_compact_confirm_sack_window_rx_path();

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

  // INBAND CEILING RE-RAISE (data-flow-inband-ceiling-reraise.md §3) — the PURE
  // policy that decides the new value of supershift_proven_ceiling after a
  // sustained-clean batch UNDER THE IN-BAND NO-BREAK REDESIGN. ROOT of the
  // inband deep-stall: inband_route_failure_demote() PINS
  // supershift_proven_ceiling = demote_target on every tag-demote
  // (arq_commander.cc:3073), but the ONLY production sites that RE-RAISE the
  // ceiling live in the turbo / SUPERSHIFT / BREAK-recovery paths that the
  // no-BREAK inband demote deliberately never fires. So after a degrade the
  // ceiling stays pinned at the demoted rung and the FRAME-UP gate
  // (arq_commander.cc:5340: index(proposed_frame) > index(ceiling) => blocked)
  // walls ALL upward probing — the link re-climbs NOTHING once the channel
  // recovers. Legacy re-climbs only because its BREAK->turbo recovery re-raises
  // the ceiling; the inband path broke that borrowed pairing.
  //
  // THE FIX (option (b)): mirror data_anchor_raise_target — once the demoted
  // rung has delivered N CONSECUTIVE clean batches (the SAME sustained-anchor
  // bar §11 uses, robust N=1 / OFDM N=2), RAISE the ceiling to TRACK the proven
  // anchor (current_anchor == last_data_viable_config). The +1 anchor clamp
  // (arq_commander.cc:5348-5350) then permits a probe EXACTLY ONE rung above
  // proven ground — never a leap back to the failed rung — and the gearshift's
  // own decode-failure demote (frame_gearshift_data_failed_nack/_pat) re-pins
  // the ceiling if that +1 probe fails. So this restores re-climb WITHOUT
  // over-climb: the ceiling never exceeds the PROVEN anchor, and the existing
  // +1 clamp remains the sole over-climb bound (audit §5). PURE; no side
  // effects; the unit test (Part X) replays it directly.
  //   current_ceiling  = supershift_proven_ceiling (the af-demote pin; <0 == no cap).
  //   anchor           = last_data_viable_config (the §1.1 confirmed-delivery anchor;
  //                      the SOLE target — we never raise the ceiling above proven ground).
  //   streak_config    = clean_batches_config (the rung the clean STREAK accumulated at).
  //   clean_streak     = clean_batches_at_current_config.
  // Returns the (possibly raised) ceiling. RULES (mirror data_anchor_raise_target):
  //   1. Enough consecutive cleans AT the streak's home rung (§11 bar).
  //   2. Only ever RAISE — a candidate at or below the current cap is a no-op
  //      (a NEGATIVE/absent cap is already "no ceiling": leave it; the demote is the
  //      sole pin producer, so a <0 ceiling means no inband demote happened).
  //   3. Cap the raise at the PROVEN anchor — never above last_data_viable_config
  //      (so the +1 clamp stays the sole over-climb bound).
  static int inband_ceiling_raise_target(int current_ceiling, int anchor,
                                         int streak_config, int clean_streak)
  {
    // No active pin (the demote is the sole producer) => nothing to re-raise.
    if(current_ceiling < 0)
      return current_ceiling;
    // Rule 1: the demoted rung must have re-proven itself sustained-clean.
    if(clean_streak < sustained_anchor_threshold(streak_config))
      return current_ceiling;
    // ── SECONDARY FIX (data-flow-robust-ofdm-adopt-flush.md §13): the robust/OFDM
    //    BOUNDARY tier-cross exemption. ROOT of the post-demote deadlock at WGN:40:
    //    after a (false) demote pins both the ceiling AND the anchor at robust-top
    //    (ROBUST_2), the anchor can NEVER re-cross into the OFDM tier — the §16 TIER
    //    GATE in data_anchor_raise_target refuses a robust->OFDM anchor cross on robust
    //    evidence (a ROBUST clean ACK has a robust streak_config). So Rule 3 below caps
    //    the ceiling AT that robust-top anchor forever, the FRAME-UP gate
    //    (arq_commander.cc:5460: index(proposed) > index(ceiling) => blocked) walls the
    //    +1 probe to CONFIG_0, and the link can NEVER re-attempt the cross even once the
    //    ring is healthy and the channel recovered. The PRIMARY fix (ring-shrink) makes
    //    the false-demote not happen in the first place; THIS is belt-and-suspenders for a
    //    demote that already landed. EXEMPTION: when the re-proven anchor sits AT the top
    //    robust rung (is_robust_config(anchor) && the next ladder rung is OFDM — i.e. the
    //    natural CONFIG_0 entry), permit the ceiling to reach EXACTLY that one OFDM-entry
    //    rung above robust-top — never higher. The existing +1 FRAME-UP anchor clamp
    //    (arq_commander.cc:5468: proposed <= anchor+1) already permits CONFIG_0 above a
    //    robust-top anchor, so this only lifts the CEILING wall; it probes ONE rung above
    //    proven ground. If that CONFIG_0 probe fails, the gearshift decode-failure demote
    //    (frame_gearshift_data_failed_nack/_pat / emergency_nack) re-pins the ceiling — so
    //    no over-climb is opened (the +1 clamp remains the sole over-climb bound). Bounded,
    //    inband-only (the sole caller is inband-gated, arq_commander.cc:5406).
    {
      int ai = config_ladder_index(anchor);
      if(is_robust_config(anchor) && ai >= 0
         && ai + 1 < FULL_CONFIG_LADDER_SIZE
         && is_ofdm_config(FULL_CONFIG_LADDER[ai + 1]))
      {
        int ofdm_entry = FULL_CONFIG_LADDER[ai + 1];   // the natural CONFIG_0 entry
        if(config_ladder_index(ofdm_entry) > config_ladder_index(current_ceiling))
          return ofdm_entry;   // lift the ceiling one rung into the OFDM tier
        return current_ceiling;
      }
    }
    // Rule 3: the target is the PROVEN anchor — never above it.
    if(config_ladder_index(anchor) <= config_ladder_index(current_ceiling))
      return current_ceiling;   // Rule 2: only ever RAISE
    return anchor;
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
      // C2 (data-flow-gearshift-climb.md): RETRIGGER_MAX_LEAP is 16 (the full
      // CONFIG_0->CONFIG_16 span) so a proven-cfg0 anchor may reach the top rung in ONE leap.
      // C2 is OPT-IN (default OFF): MERCURY_CLIMB_TIER2=1 raises the cap to 16. The default ships
      // Tier-1 (C1 only, cap 13); ACCEL_DEFEAT=1 reverts C1 too. C2/C3 deferred pending a decode-margin gate.
      int max_leap = (!climb_accel_defeat && climb_tier2) ? RETRIGGER_MAX_LEAP : 13;
      int leap_cap = config_ladder_up_n(anchor, max_leap, robust_en, narrowband);
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

  // CONNECT-SEED of the START config (gearshift-start-and-recovery.md §10.2). PURE
  // core: from the connect-time control-plane SNR choose a WB OFDM start config
  // STRICTLY above init_cfg, or CONFIG_NONE when the channel is not clearly clean
  // (the no-over-seed guard). Takes every input explicitly so the directed test
  // (--test-connect-snr-seed) drives it with no live telecom_system:
  //   snr_uplink   = measurements.SNR_uplink (the control-plane MFSK suffix SNR)
  //   snr_mapped   = get_configuration(snr_uplink - CONNECT_SEED_MARGIN_DB)
  //                  (caller supplies; the test injects the mapping)
  //   init_cfg     = init_configuration (the un-seeded start, ROBUST_0 for -R)
  //   proven_ceil  = supershift_proven_ceiling (-1 = none)
  //   nb           = narrowband_enabled == YES
  // Returns CONFIG_NONE unless ALL hold: feature/gearshift on (caller-gated), SNR
  // valid (> -90), snr_mapped index >= CONNECT_SEED_CONFIG_MIN, and the capped
  // result (min of snr_mapped, CONNECT_SEED_CONFIG_CAP, proven_ceil, NB ceiling)
  // is STRICTLY above init_cfg by ladder index. NB sessions never seed past
  // NB_CONFIG_MAX. PURE (no member writes); replayed directly by the test.
  static int connect_seed_target_core(double snr_uplink, int snr_mapped,
                                      int init_cfg, int proven_ceil, bool nb);
  // Member wrapper: gates on the feature + gear_shift + valid SNR, computes
  // snr_mapped via get_configuration(), applies the bigblock cooldown cap, and
  // returns the seed config (or CONFIG_NONE). NON-const (get_configuration is
  // non-const). Defined in arq_commander.cc next to elevator_target_from_snr().
  int connect_seed_target();

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

  // ── FORGIVING-ACK Tier-2 PARTIAL-SACK DE-DUP KEY ──────────────────────
  // (data-flow-inband-dataplane-stall-post-leap.md §3/§7). Under the cumulative
  // cap the reverse-SACK wire bsi carries n_r (the contiguous delivery high-water
  // rsp_last_delivered_batch_seq_id), FROZEN across successive in-flight PARTIAL
  // batches until a NEW delivery advances it. The CMD partial de-dup
  // (cmd_last_applied_sack_bsi) was keyed on that raw wire bsi — written for the
  // per-batch semantics where each batch's bsi is DISTINCT. Under the cap every
  // post-first partial repeats the same frozen n_r, so the guard false-collides it
  // as a "duplicate", the bitmap is never applied, stats.nAcked_data freezes, and the
  // REVSACK-CHEAPMISS discriminator escalates to BREAK/demote (the post-leap one-
  // batch/600s stall). ROOT FIX: key the partial de-dup on the IN-FLIGHT BATCH
  // IDENTITY the bitmap is applied to (cmd_batch_seq_id — the CMD's current batch,
  // which advances per batch), NOT the frozen wire n_r. Cap OFF: the per-batch wire
  // bsi IS the identity, so key on rx_bsi UNCHANGED (legacy byte-identical; the whole
  // non-Tier-2 fleet + every existing test stay bit-for-bit). This decouples the
  // DE-DUP key ONLY; it does NOT alter the wire encoding NOR cumulative_ack_covers'
  // backward-window SELF-HEAL — both still read the raw frozen n_r, so the Tier-2
  // self-heal that RELIES on n_r is fully preserved. The CLEAN tracker keeps using
  // rx_bsi (the clean wire bsi advances on every delivery, never frozen). PURE +
  // static so --test-climb-engine drives the exact production decision.
  // -DCUMULATIVE_ACK_DEDUP_FAILBEFORE pins the frozen-n_r key (reproduces the stall)
  // so the regression fails-before / passes-after in the SAME binary.
  static int partial_sack_dedup_key(int rx_bsi, int cmd_batch_seq_id, bool cap_on)
  {
#ifdef CUMULATIVE_ACK_DEDUP_FAILBEFORE
    (void)cmd_batch_seq_id; (void)cap_on;
    return rx_bsi & 0xFF;                       // FAIL-BEFORE: frozen-n_r key (the stall).
#else
    if(cap_on) return cmd_batch_seq_id & 0xFF;  // cap ON: in-flight batch identity (advances per batch)
    return rx_bsi & 0xFF;                        // cap OFF: legacy per-batch bsi (unchanged)
#endif
  }

  // ── FORGIVING-ACK Tier-2 STALE-PARTIAL FALSE-ACK GUARD ────────────────────
  // (data-flow-inband-dataplane-stall-post-leap.md §8.3). The companion to
  // partial_sack_dedup_key, closing the latent robustness gap that key OPENED.
  // A PARTIAL's 30-bit selective bitmap is applied BY SLOT INDEX to the in-flight
  // batch, so it describes EXACTLY ONE batch: the contiguous successor n_r+1
  // (cumulative_ack_covers arm b — Mercury is batch-level stop-and-wait, so the
  // only batch above the delivery high-water n_r is n_r+1). Under the cap the
  // window gate (bsi_in_window, arq_commander.cc:4384-4392) ADMITS any report
  // whose frozen n_r merely lands in the bounded backward self-heal window of
  // cmd_bsi OR prev_bsi — CORRECT for a CLEAN cumulative delivery confirmation
  // (arm a retires batches <= n_r) but NOT a discriminator of which batch a
  // PARTIAL bitmap targets. Since the night stall fix keys the partial de-dup on
  // the in-flight batch identity (cmd_batch_seq_id), a STALE partial for batch k,
  // LATE-decoded (MW late-window re-decode, arq_commander.cc:4265) AFTER the CMD
  // advanced to k+1, is NOT a duplicate (cmd-key is k+1, fresh) and its n_r=k-1
  // still satisfies the backward window — so its batch-k bitmap would be
  // FALSE-APPLIED by slot index to k+1's PENDING_ACK frames, a FALSE ACK that
  // suppresses their retransmit (silent data loss; §8.3 latent gap). This is the
  // missing PARTIAL-ONLY validation: accept the bitmap apply iff the batch it
  // describes (n_r+1) IS the current in-flight batch (cmd_bsi). A legit current
  // partial ALWAYS satisfies this (n_r = cmd_bsi-1: the high-water is the batch
  // just below the in-flight one), INCLUDING the frozen-n_r stall the night fix
  // targets (there cmd_bsi = n_r+1 too), so the stall fix is fully preserved.
  // Cap OFF: the wire bsi IS the per-batch identity and the legacy {cmd_bsi,
  // prev_bsi} window + per-batch de-dup already fence it -> return true
  // UNCONDITIONALLY (byte-identical; the whole non-Tier-2 fleet + every existing
  // test stay bit-for-bit). Does NOT touch the wire encoding, the de-dup key, or
  // cumulative_ack_covers' backward self-heal (CLEAN confirmations still ride it).
  // PURE + static so --test-climb-engine drives the EXACT production decision.
  // -DCUMULATIVE_ACK_STALEPARTIAL_FAILBEFORE pins accept (reproduces the false-ACK)
  // so the regression fails-before / passes-after in the SAME binary.
  static bool partial_sack_target_is_inflight(int rx_bsi, int cmd_batch_seq_id, bool cap_on)
  {
#ifdef CUMULATIVE_ACK_STALEPARTIAL_FAILBEFORE
    (void)rx_bsi; (void)cmd_batch_seq_id; (void)cap_on;
    return true;                                 // FAIL-BEFORE: no stale-partial discard (the false-ACK).
#else
    if(!cap_on) return true;                     // cap OFF: legacy window+de-dup fence it (byte-identical)
    unsigned succ    = ((unsigned)(rx_bsi & 0xFF) + 1u) & 0xFFu;  // batch the bitmap describes (n_r+1)
    unsigned cmd_bsi = (unsigned)(cmd_batch_seq_id & 0xFF);
    return succ == cmd_bsi;                       // apply iff that batch IS the in-flight batch
#endif
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

  // R2a — the RECOVERABLE-hole predicate. Distinguishes the ONE gap topology
  // that a bounded hold can heal (the completed current batch is exactly the
  // +2 successor of last_delivered, and the single missing batch between them
  // is the armed, partially-received prev) from a genuine multi-batch desync
  // (which must still abort). PURE + static so the SIM_INPROC test drives the
  // EXACT production decision. It is a STRICT SUBSET of delivery_step_is_gap:
  // it can only be true where delivery_step_is_gap(cur,last) is already true
  // with fwd==2, so it never widens the abort surface — it only diverts the
  // fwd==2/armed-prev case to the hold. See
  // fact-documents/data-flow-recoverable-gap-abort.md §5.
  static bool gap_is_recoverable_prev_hole(int cur_bsi, int last_delivered_bsi,
                                           bool prev_active, int prev_bsi,
                                           int prev_received_count)
  {
    if(last_delivered_bsi < 0) return false;
    if(!prev_active) return false;
    if(prev_received_count <= 0) return false;
    unsigned last = (unsigned)(last_delivered_bsi & 0xFF);
    unsigned cur  = (unsigned)(cur_bsi & 0xFF);
    unsigned fwd  = (cur - last) & 0xFFu;
    if(fwd != 2u) return false;                 // exactly one missing batch
    unsigned want_prev = (last + 1u) & 0xFFu;   // the missing batch == prev
    return ((unsigned)(prev_bsi & 0xFF) == want_prev);
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

  // IN-BAND CONFIG_0 ROLLING-PARTIAL climb-unblock regression (CLI --test-inband-frame0-partial).
  // A lead-frame-only partial (bitmap bit0 clear, rest set) at an inband OFDM rung ADVANCES the
  // FRAME-UP climb streak; a multi-frame-drop partial stays vetoed (§9). Returns 0 pass / 1 fail.
  // Default builds never call this. Fails-before: -DINBAND_FRAME0_PARTIAL_FAILBEFORE.
  // See fact-documents/data-flow-inband-frame0-rolling-partial.md.
  int test_inband_frame0_partial();

  // IN-BAND ROLLING-PARTIAL climb DEFER-WHILE-HOLE-OUTSTANDING regression
  // (CLI --test-inband-climb-defer). The 2801d7c sibling: the lead-frame-only partial enqueues
  // frame-0 for retx (retransmit_count > 0), and the anchor-raise + FRAME-UP config-change fire
  // WHILE that hole is outstanding; the config change clears the retx queue and abandons frame-0
  // -> bsi gap -> GAP-ABORT wedge. The fix DEFERS the anchor-raise + the fire while the hole is
  // outstanding but KEEPS the 2801d7c streak credit; the next whole batch (retransmit_count==0)
  // fires the built streak. Returns 0 pass / 1 fail. Default builds never call this.
  // Fails-before: -DINBAND_CLIMB_DEFER_FAILBEFORE.
  // See fact-documents/data-flow-inband-frame0-rolling-partial.md §10.
  int test_inband_climb_defer_on_retx();


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

  // FORGIVING-ACK Tier-2 cumulative-n_r regression (--test-cumulative-ack).
  // Drives the SELF-HEAL (a lost report recovered by the next n_r), the
  // contiguous-high-water GAP-INVARIANT (n_r NEVER ACKs a gap — driven through the
  // REAL advance_last_delivered + delivery_step_is_gap producers), the CAPABILITY
  // GATE (cap-off → per-batch fallback, no misapply). Replays the PURE
  // cumulative_ack_bsi_field() / cumulative_ack_covers() helpers; the gap arm uses
  // the production high-water producer. Returns 0 on pass, 1 on fail.
  // NOTE: ported onto a monitor-based tree that does NOT carry the Tier-1 decouple
  // (test_forgiving_ack / forgiving_ack_should_decouple) — A3 does NOT touch the
  // demote; the Part-D "composes with Tier-1" arm uses only the PURE A3 helpers.
  // See fact-documents/data-flow-forgiving-ack.md §T2.6.
  int test_cumulative_ack();

  // T5 — the §2 DECOUPLE-SAFETY CHECKPOINT (--test-a3-decouple-safety). The gate
  // that MUST be GREEN before the Phase-2 demote-decouple is attempted. With A3
  // ENABLED and the demote UNTOUCHED, proves at the TRANSFER level (byte
  // accounting) that (T5a) a SINGLE forward-healthy reverse-ACK miss is
  // NON-LOAD-BEARING — the next turn's cumulative n_r supersedes the dropped
  // report, the cursor advances PAST the dropped batch, and the multi-batch
  // transfer COMPLETES byte-faithful with ZERO re-air of received frames (the
  // explicit anti-0-bytes proof; FAIL-BEFORE -DCUMULATIVE_ACK_FAILBEFORE STALLS) —
  // and (T5b) SUSTAINED loss (dead reverse channel: high-water frozen, A3 cannot
  // cover the undelivered batch) STILL exhausts the production nResends countdown
  // -> FAILED_ -> demote/BREAK (the genuine-death net is intact, A3 untouched).
  // Drives the REAL advance_last_delivered / delivery_step_is_gap producers + the
  // REAL cumulative_ack_covers consumer apply. Returns 0 (gate GREEN) / 1 (RED).
  int test_a3_decouple_safety();

  // ROBUST_0 + streaming-compression deadlock regression
  // (data-flow-compress-frame-fill.md). Drives the REAL
  // process_buffer_data_commander() data-fill path at ROBUST_0 frame
  // dimensions (max_frame == 7 == COMPRESS_HEADER_SIZE) with streaming
  // compression enabled and a real compressible payload staged. Asserts the
  // batch carries > 0 application bytes (FAIL-BEFORE on fef293f: every batch
  // stages 0 payload → 0 throughput forever). One-shot, exits rc. See §5 audit.
  int test_robust0_compress_deadlock();
  int test_mixbatch_fill_overpop();  // --test-mixbatch-fill-overpop: mixbatch fill over-pop reorder regression
  int test_mixbatch_fill_overpop_compressed();  // --test-mixbatch-fill-overpop-compressed: comp-leg over-pop + force-FREE data-loss regression

  // Idle SWITCH_ROLE race regression (FAILS-BEFORE evidence for the
  // connected-but-0-deliver bench bug). Drives the REAL
  // process_buffer_data_commander() idle branch (arq_commander.cc:15238-15251)
  // with a freshly-CONNECTED COMMANDER, EMPTY tx FIFO, block_under_tx==NO. Two
  // calls advance past switch_role_timeout; asserts SWITCH_ROLE is queued on an
  // empty-tx Commander BEFORE any data frame. Demonstrates the channel-free
  // root cause: if the app withholds its first data write past
  // switch_role_timeout, the Commander hands its role away with an empty FIFO.
  // One-shot, exits rc.
  int test_idle_switch_role_race();

  // SWITCH_ROLE re-ride race (idle-switchrole-race.md §6) — a peer re-acquiring
  // COMMANDER via SWITCH_ROLE must not inherit a stale session_data_frame_sent
  // from its previous commander stint (the R1 reverse-transfer double-swap gate).
  // FAILS-BEFORE with -DSWITCHROLE_RERIDE_FAILBEFORE. One-shot, exits rc.
  int test_switch_role_reride_race();

  // IDLE-SWITCHROLE-RACE recovery path (idle-switchrole-race.md §3/§4 Part C).
  // Drives the REAL BREAK-EXHAUSTED re-arm site (arq_commander.cc:~410) on a
  // never-fed Commander (no RX data, empty tx FIFO, zero-byte re-queue) K+1 times.
  // FAILS-BEFORE: the spiral re-arms the watchdog forever, role stays COMMANDER.
  // PASSES-AFTER: at K no-progress cycles it stops re-arming and tears down to the
  // RESPONDER/LISTENING FORCED-fallback post-state. Negative control: inject RX
  // data / a non-empty re-queue and assert the counter RESETS and NO teardown
  // happens. One-shot, exits rc.
  int test_break_noprogress_teardown();

  // CONNECT-REACK EXCISE regression (--test-connect-reack, also in master
  // --test; connect-testack-handshake.md §9). Guards the removal of 8e62722e:
  // drives a REAL OFDM-config RX in the CONNECTED pre-data RECEIVING window and
  // asserts the SHARED frames_to_read the OFDM data path consumes is NOT pinned
  // to 2 (PASS-AFTER), while a defeat arm (MERCURY_REACK_CLAMP_DEFEAT) that
  // re-introduces the removed clamp DOES pin it to 2 (FAIL-BEFORE). The removed
  // pre-data re-ACK clamped frames_to_read=2 across the first WB batch, starving
  // OFDM data RX so the gearshift never climbed off config100. 0 PASS / 1 FAIL.
  int test_connect_reack();

  // RX-CTRL-DROP fix (data-flow-control-slot-lifecycle.md). PURE predicate: the
  // RECEIVED-state watchdog decision for the one-deep messages_control mailbox.
  // Returns true iff the slot is RECEIVED, its ack_timer is counting, and elapsed
  // has met the bound (a generous multiple of ack_timeout_control, floored). Shared
  // by update_status() (real elapsed) and --test-rx-ctrl-drop (synthetic elapsed),
  // so the watchdog is testable with no wall-clock sleep. Honours
  // RX_CTRL_DROP_FAILBEFORE (returns false when defined).
  bool rx_ctrl_received_watchdog_expired(int status, int counting, int elapsed_ms) const;

  // RX-CTRL-DROP regression (--test-rx-ctrl-drop, also in master --test;
  // data-flow-control-slot-lifecycle.md §6). In-process synthetic-fire (no PHY /
  // IONOS / RF). Asserts: (A) the RECEIVED-state watchdog predicate frees a stranded
  // slot at the bound and a subsequent control frame then lands at the produce gate;
  // (B) the V1 fall-through frees an unhandled control code; (C) reset_session_state()
  // frees a RECEIVED slot. FAILS-BEFORE with -DRX_CTRL_DROP_FAILBEFORE (the slot stays
  // stuck and the later control frame is dropped). 0 PASS / 1 FAIL.
  int test_rx_ctrl_drop();

  // ML-KEM KX MULTI-CHUNK LIVE-RX REASSEMBLY regression
  // (MLKEM_HYBRID_PLAN.md §5 / data-flow-control-slot-lifecycle.md). Drives a
  // multi-chunk KX2 (encaps key, RSP consumer) and KX3 (ciphertext, CMD consumer)
  // through the REAL live control-RX consumer functions (process_control_responder
  // / process_control_commander) after staging each chunk through the REAL slot
  // producer that hardcodes messages_control.length = 1. FAILS-BEFORE: the call
  // sites fed length=1 into kx_receive_chunk -> kx_chunk_decode rejected every
  // chunk -> reassembly never completed. PASSES-AFTER: the call sites pass the real
  // slot width -> full reassembly + live encapsulate/decapsulate. In-process
  // synthetic-fire, no IONOS/RF/telecom_system. 0 PASS / N = failure count.
  int test_kx_chunk_live_rx();
  // KX-as-data reserved-stream RX ingest fire proof (data-flow-hybrid-kex.md T1/T3):
  // reassembles a fragmented forward/reverse stream BYTE-IDENTICAL + the fail-secure
  // REFUSE fires on bad magic/kind/length and on a post-activation byte.
  int test_kx_ingest();

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
  // IN-BAND DOWN-LADDER RESYNC REGRESSION (data-flow-inband-ondemote-zerobyte.md §6):
  // drives the LIVE 2-instance SIM_INPROC through a CMD demote-to-ROBUST_0 with the
  // announce CONFIG_TAG SUPPRESSED, forcing the RESPONDER's production down-ladder to
  // resync from a PRIMARY-derived snapshot over a window spanning the MFSK ROBUST rung.
  // FAIL-BEFORE (MERCURY_INBAND_DOWN_DEFEAT_SNAPFIX=1): the primary-sized snapshot
  // truncates the ROBUST_0 frame -> 0-byte delivery (the HW defect, in sim). PASS-AFTER
  // (Rank-1 snapshot+ring sizing fix): the down-ladder decodes ROBUST_0 and the payload
  // delivers byte-faithful. Returns 0 on PASS. Selected by --test-inband-down-resync.
  static int test_inband_down_resync();
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
  // R4 LINK-PARAMS quiesce-gate regression (--test-axis2-quiesce-gate; also in --test).
  int  test_axis2_quiesce_gate();

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

  // ====================================================================
  // In-band rate adaptation — Stage 2 emit / detect+follow / directed test
  // (unilateral-config-tag-design.md §3/§5/§6; data-flow-perbatch-config.md)
  // ====================================================================
  // ALL three are no-ops (early-return) unless MERCURY_INBAND_RATE is set, so a
  // default-off build is byte-identical to the SET_CONFIG baseline. The codec
  // primitives live in include/physical_layer/mfsk_ctrl_codec.h (Stage 1).

  // Resolve + cache the MERCURY_INBAND_RATE env flag. Returns true iff the
  // feature is enabled. Cheap after the first call (cached in inband_rate_enabled).
  bool inband_rate_feature_enabled();

  // A3 demote-decouple gate (data-flow-inband-a3-decouple.md §1.2). Returns true iff
  // BOTH the env opt-in MERCURY_INBAND_A3_DECOUPLE is set (cached in
  // inband_a3_decouple_env) AND the A3 cumulative-ACK capability is NEGOTIATED for this
  // session (cumulative_ack_enabled). When true, a forward-healthy reverse-ACK miss
  // RE-AIRS the same config (relying on the cumulative n_r to self-heal the missed ACK)
  // instead of demoting one rung — removing the unbounded config-walk that strands the
  // RSP down-window. The cumulative_ack_enabled half is the STRICT SEQUENCING guard: we
  // refuse to decouple (remove the demote) unless the self-heal spine is present, else
  // the link would crawl/dead. Default-off ≡ byte-identical (the demote stays in place).
  bool inband_a3_decouple_enabled();

  // TX EMIT (design §6). Decide whether the batch about to be sent at config
  // `batch_cfg` differs from the last-announced config and, if so, build the
  // CONFIG_TAG ctrl-suffix artifacts for the FIRST frame of the batch:
  //   - the GF(16) RA energy matrix (out_energies, N*16 row-major)
  //   - the RM(1,4) FWHT soft-chip block (out_chips[16])
  //   - the binding fields actually encoded (out_bsi_lsb, out_parity)
  // `batch_seq_id` is the cmd_batch_seq_id of the batch (its low 3 bits bind the
  // tag, design §2.1). cfg_index encoded into the tag is config_ladder_index(
  // batch_cfg). On a committed change this toggles inband_tx_epoch_parity and
  // updates inband_last_announced_config. `hi`/`lo` are the per-tone clean
  // one-hot energies (Stage 2 uses an idealized energy block; the production
  // passband attach is Stage 3+ per §4.6). Returns 1 if a tag was emitted (config
  // changed), 0 if not (config unchanged, or feature off, or batch_cfg invalid).
  int emit_config_tag_if_changed(int batch_cfg, int batch_seq_id,
                                 double hi, double lo,
                                 double* out_energies /*N*16*/,
                                 double* out_chips /*[16]*/,
                                 uint8_t* out_bsi_lsb, uint8_t* out_parity);

  // RX DETECT+FOLLOW (design §3.2 outcome 1). Cheap always-on suffix-PRESENCE
  // check on the first frame of a batch; only if a suffix is present run the
  // expensive config_tag_wrap_decode. On a VALID tag whose ladder-index maps to a
  // config != current_configuration AND whose bsi_lsb/epoch_parity bind, FOLLOW:
  // load_configuration(followed_config, PHYSICAL_LAYER_ONLY, NO) — which switches
  // BOTH the ARQ current_configuration AND the PHY twin
  // (cl_telecom_system::current_configuration) coherently in one call (the audit's
  // D1 coherent switch). Writes *out_followed_config (the raw config id) on a
  // follow. Returns 1 on follow, 0 on no-tag / unchanged / feature-off.
  //   energies/chip_soft : the captured CONFIG_TAG energy matrix (N*16) + soft
  //                        chips (the same blocks emit_config_tag_if_changed builds)
  //   n_syms             : N = gf16ra::codeword_len() (presence check scans these)
  //   expect_bsi_lsb     : the cmd bsi low-3 the RX is adopting (bind gate-3)
  //   expect_parity      : the epoch_parity the RX expects, or 0xFF to skip
  int detect_and_follow_config_tag(const double* energies, const double* chip_soft,
                                   int n_syms, uint8_t expect_bsi_lsb,
                                   uint8_t expect_parity, int* out_followed_config);

  // Directed in-process loopback test (Stage 2). Forces a config switch at a
  // batch boundary (CONFIG_10 -> CONFIG_8) and asserts the RX follows the config
  // FROM THE TAG (load_configuration via detect_and_follow_config_tag), with the
  // PHY twin switching coherently. Builds a real cl_telecom_system per side so the
  // follow exercises the production load_configuration path. fail-before
  // (-DSTAGE2_FAILBEFORE): the RX ignores the tag and stays at CONFIG_10 -> the
  // switched batch's config NEVER follows -> FAIL. Returns 0=PASS, 1=FAIL.
  int test_config_tag_follow();

  // ---- In-band rate adaptation (Stage 3a) — make the tag ride the REAL passband.
  // Build the combined CONFIG_TAG suffix tone array that the passband keyer
  // (cl_telecom_system::generate_config_tag_pattern_passband) transmits. The tag
  // rides TWO concatenated suffix blocks (tag-codeword-design.md §1.3 / the WRAP
  // detector): [ RM(1,4) Walsh codeword : CFG_TAG_RM_N=16 symbols ]
  // [ GF(16) RA + CRC-12 message : gf16ra::codeword_len()=39 symbols ]. The RM
  // tones come from cfg_tag_rm_encode (the perm-tone realization,
  // cfg_tag_energies_from_cfg); the gf16ra tones from gf16ra::encode_config_tag.
  // Both carry the SAME cfg_index so the FWHT detector and the CRC field
  // corroborate (Gate-3). out_tones must hold >= CFG_TAG_RM_N + gf16ra
  // codeword_len() ints; *out_n returns that count (55). Also returns the bsi_lsb
  // and parity it bound (for the RX binding gates). gf16ra::configure(2) is set
  // internally (the gf16ra block is the N=39 R=1/3 substrate). Returns true on
  // success; false if cfg_index off the ladder or M<16. Shared by the production
  // emit and the Stage-3a passband round-trip test.
  bool build_config_tag_tones(int batch_cfg, int batch_seq_id, uint8_t parity,
                              int* out_tones, int* out_n,
                              uint8_t* out_bsi_lsb);

  // Stage-3a PASSBAND ROUND-TRIP test (CLI --test-config-tag-passband). TX builds
  // the combined RM+gf16ra suffix, keys it to real passband audio
  // (generate_config_tag_pattern_passband), passes it through CLEAN and AWGN
  // channels, then the RX detects the burst on the passband
  // (decode_config_tag_from_passband — the real base-correlator presence detector,
  // NOT an energy artifact) and decodes it via config_tag_wrap_decode. Asserts the
  // right cfg_index decodes AND that an OFDM data frame preceding the suffix still
  // LDPC-decodes (the suffix does not corrupt the payload). fail-before
  // (-DSTAGE3A_FAILBEFORE): the RX ignores the passband suffix -> no decode -> FAIL.
  // Returns 0=PASS, 1=FAIL.
  int test_config_tag_passband_roundtrip();

  // ---- In-band rate adaptation (Stage 3b W1) — EMIT the tag on the wire.
  // Called from send_batch right AFTER the first OFDM data frame's tx_transfer, at
  // a DETERMINISTIC offset (Stage-3c trims the acquisition sync later). When
  // MERCURY_INBAND_RATE is set AND batch_cfg differs from the last-announced config,
  // builds the combined RM+gf16ra CONFIG_TAG suffix (build_config_tag_tones), keys it
  // to passband audio (generate_config_tag_pattern_passband — the Stage-3a keyer),
  // and tx_transfers the burst so it rides the real wire after frame 0. Toggles the
  // epoch parity + latches inband_last_announced_config (the same committed-change
  // bookkeeping as emit_config_tag_if_changed). No-op (returns 0) when the feature is
  // off, the config is unchanged, the config is off-ladder, or the robust layer is
  // unavailable (M<16 / NB). Returns the number of passband samples emitted, 0 if no
  // tag was sent. `batch_cfg` is the config of the batch being sent (=
  // current_configuration here); `batch_seq_id` is its bsi (binds the tag).
  int emit_config_tag_passband(int batch_cfg, int batch_seq_id);

  // STAGE 4d (D1) — the CONFIG_TAG FIRING-POLICY DECISION + announce state machine,
  // factored out of emit_config_tag_passband (the production emit calls this; the
  // directed test --test-inband-retag drives it directly without the passband side
  // effects). Returns true to EMIT (a CHANGE or an armed REPEAT) / false for steady
  // state. On true it has mutated the announce state (parity toggle/hold, last-announced
  // latch, announce-bsi anchor, R counter) exactly as the production emit needs, and
  // written *out_parity. See the definition comment for the change-vs-repeat parity rule.
  bool inband_tag_firing_decision(int batch_cfg, int batch_seq_id, uint8_t* out_parity);

  // ---- In-band rate adaptation (Stage 3b W2) — DETECT+FOLLOW from the capture.
  // Called from the RX first-frame path at/near the [RSP-V2-ADOPT] adopt site. Pulls
  // the CONFIG_TAG burst out of the captured passband tail (the burst W1 keyed right
  // after frame 0), runs the REAL base-correlator presence detector
  // (decode_config_tag_from_passband), and on a valid+bound tag FOLLOWS via
  // detect_and_follow_config_tag — which load_configuration()s the announced config
  // (ARQ + PHY twin coherent) AND ports the SET_CONFIG HINGE side-effects (capture-
  // flush + D3.1 re-baseline). No-op when MERCURY_INBAND_RATE is off. `expect_bsi_lsb`
  // is the low-3 of the bsi the RX is adopting (the binding gate). `expect_parity` is
  // the epoch_parity to require, or 0xFF to skip (the RX does not track the TX parity
  // across a lost tag — Stage 4 — so the production wiring passes 0xFF). Returns 1 if
  // the RX followed a new config, 0 otherwise. *out_followed_config (if non-NULL)
  // receives the followed raw config id (or current_configuration if no follow).
  int inband_detect_follow_from_capture(uint8_t expect_bsi_lsb,
                                        uint8_t expect_parity,
                                        int* out_followed_config);

  // ── STAGE 3d — PRE-FRAME detect+follow from a snapshot (data-flow-perbatch-
  // config.md §15) ── The DVB-S2 PLHEADER twin of inband_detect_follow_from_capture:
  // the TX now keys the CONFIG_TAG burst BEFORE frame 0, so the RX runs this over the
  // SAME captured snapshot the OFDM acquisition is about to consume (receive() passes
  // ready_to_process_passband_delayed_data + signal_period) BEFORE receive_byte
  // demodulates frame 0. On a valid tag for a config != current it switches BOTH config
  // copies + runs the HINGE; the HINGE flushes the LIVE ring, NOT this `snapshot`
  // buffer, so frame 0 is preserved and decodes SEAMLESSLY at the new config. Binds
  // NEITHER bsi NOR parity (the pre-frame detect runs before frame 0 decodes, so the
  // batch bsi is unknown) — the FWHT peak + GF(16)+CRC-12 + cfg_index corroboration are
  // the ~1e-8-FAR accept gates. A no-tag window is cheaply rejected (zero added latency,
  // INV-3d-B). Returns 1 if it followed a new config, 0 otherwise. No-op (returns 0)
  // when MERCURY_INBAND_RATE is off — byte-identical default.
  int inband_detect_follow_from_snapshot(double* snapshot, int len,
                                         int* out_followed_config);

  // ── STAGE 4 — the bounded down-ladder lost-tag resync (design §4 / §7) ──
  // The §3.2 outcome-3 recovery: the RX's first-frame decode FAILED at
  // current_configuration AND no CRC-valid CONFIG_TAG was heard (a lost tag in a
  // fade). Run a BOUNDED down-window blind decode over the captured first-frame
  // snapshot and ADOPT the config that actually decodes (CRC/LDPC pass) — never a
  // guess. Search set = FULL_CONFIG_LADDER[max(0,cur_idx-D) .. cur_idx], from cur
  // DOWNWARD (closest-to-current first; a drop only moves toward robust). At most
  // D+1 scoped decoders / decode attempts (RPi bound — NOT the NUMBER_OF_CONFIGS
  // monitor bank). On a win: load_configuration(winner) (ARQ+PHY-twin coherent) +
  // the SAME Stage-3b HINGE side-effects (capture-flush + D3.1 re-baseline); the
  // decoded bytes go out via *out_decoded / *out_decoded_len for delivery through
  // the existing SACK path (the returning SACK is the implicit confirm).
  // Returns the WINNING raw config id (>=0) on a decode pass, or -1 if NONE of the
  // D+1 window configs decoded. No-op (returns -1) when MERCURY_INBAND_RATE is off.
  // `audio`/`audio_len` is the captured first-frame passband snapshot. `D` is the
  // down-window depth (clamped to [1, INBAND_DOWN_D_MAX]). `expect_bsi_lsb` binds
  // (currently advisory — the CRC/LDPC pass is the adopt gate). When out_decoded is
  // non-NULL it receives the winner's decoded bytes (length in *out_decoded_len).
  int inband_down_ladder_resync(const double* audio, int audio_len, int D,
                                uint8_t expect_bsi_lsb,
                                int* out_decoded, int* out_decoded_len);

  // Coherent ARQ+PHY-twin config switch + the SET_CONFIG HINGE side-effects
  // (capture-flush + D3.1 re-baseline). Shared by the tag-follow path
  // (detect_and_follow_config_tag) and the Stage-4 down-ladder so the two adopt
  // paths are behaviourally IDENTICAL (one HINGE implementation, no divergence).
  // Caller guarantees followed_config != current_configuration (a real change).
  void inband_adopt_resynced_config(int followed_config);

  // SHARED OFDM-ENTRY ADOPT SETUP (data-flow-robust-ofdm-adopt-flush.md §12) — the
  // capture-ring geometry reconciliation an adopt INTO a config must run AFTER the PHY
  // load: HINGE-1 stale-vs-live-burst flush/preserve, OFDM cursor re-anchor, FTR/anti-
  // scroll re-init (fix #1b), and the natural-OFDM ring SHRINK + inband_ofdm_acq_ring_shrunk
  // gate (fix #1c/#1d). Factored OUT of inband_adopt_resynced_config so BOTH adopt routes
  // run IDENTICAL setup: (a) the redesign's UNILATERAL CONFIG_TAG-follow adopt, and (b) the
  // HYBRID legacy SET_CONFIG cross (the responder's data-config adopt, arq_responder.cc:
  // 1723/1751/1764) — which previously called only plain load_configuration and STRANDED the
  // ring-shrink, leaving the CONFIG_0 ring oversized at the robust floor so every re-aired
  // preamble landed beyond the search upper bound (`OFDM beyond-bounds`) -> 0 forward decode
  // -> false-demote -> 36x loss to legacy at WGN:40. Caller MUST have already run
  // load_configuration(adopted_config, ...) (so the PHY + ring geometry are at the new config)
  // and MUST gate on inband_rate_feature_enabled() (legacy is byte-identical — never calls
  // this). Honors the same MERCURY_ADOPT_* defeat knobs as the unilateral path.
  void inband_finalize_ofdm_adopt_ring(int adopted_config);

  // Lazily (re)build the scoped down-window decoder bank for the configs in
  // FULL_CONFIG_LADDER[lo_idx .. hi_idx] (hi_idx-lo_idx+1 <= INBAND_DOWN_D_MAX+1).
  // Each decoder is sized to its OWN config (NOT CONFIG_0's max buffer). Cached
  // across batches keyed by config id; only (re)allocates slots whose config
  // changed. Never allocates the full NUMBER_OF_CONFIGS bank. Returns the count of
  // live decoders in [lo_idx,hi_idx].
  int inband_ensure_down_decoders(int lo_idx, int hi_idx);

  // Stage-3b LOOPBACK DROP TEST (CLI --test-inband-drop). Two-instance in-process
  // loopback (CMD+RSP, real passband, shared virtual clock). With MERCURY_INBAND_RATE
  // on, the gearshift DROPS one rung; W1 keys the tag onto the real passband after
  // frame 0; W2 follows from the passband tag; the SACK returns + confirms (bsi). The
  // test asserts: RX follows from the tag, current_configuration tracks on BOTH ends
  // (ARQ + PHY-twin coherent), ZERO SET_CONFIG frames on the wire, byte-faithful
  // delivery across the drop. Plus the R7 mixed-config-consecutive-batches gap-gate
  // case. fail-before (MERCURY_INBAND_RATE off OR -DINBAND_STAGE3B_FAILBEFORE): the
  // gearshift queues SET_CONFIG / the RX does not follow -> SET_CONFIG count > 0 / RX
  // stuck at the old config. Returns 0=PASS, 1=FAIL.
  int test_inband_drop();

  // STAGE 4 LOST-TAG DOWN-LADDER TEST (CLI --test-inband-fallback). In-process: an
  // RX at CONFIG_x, the TX drops to CONFIG_(x-k) and keys the first frame at the
  // dropped config but the CONFIG_TAG is FORCED LOST (suffix omitted/corrupted).
  // Asserts: (1) the OLD-config first-frame decode fails; (2) the bounded down-
  // ladder decodes at the TRUE config within D rungs (message_decoded==YES, a real
  // CRC/LDPC pass — never a guess); (3) the SACK bsi confirms; (4) BREAK-count==0;
  // (5) decode attempts <= D+1 (RPi bound); (6) sweep D in {1..5} -> resync-success
  // vs D (k-rung drop resyncs iff D>=k); (7) sweep SESSION_DEAD_BATCHES -> BREAK
  // fires at exactly the Nth consecutive total-loss batch. fail-before
  // (-DINBAND_STAGE4_FAILBEFORE or flag-off): the ladder is disabled -> the lost-tag
  // drop is unrecoverable -> BREAK-count>0. Returns 0=PASS, 1=FAIL. design §4/§7,
  // data-flow-perbatch-config.md §13.6.
  int test_inband_fallback();

  // IN-BAND DOWN-LADDER ROBUST RESYNC — DIRECTED DECODE PROOF
  // (data-flow-inband-ondemote-zerobyte.md §2.4/§6/§7, Rank-1 fix). Lays a REAL ROBUST_0
  // frame into a PRODUCTION RX whose primary config is a LOW OFDM rung (CONFIG_1, small
  // ring) and runs the PRODUCTION inband_try_down_ladder_on_decode_fail (the buggy
  // snapshot-sizing site) with REAL acquisition. defeat=true: the truncated primary-sized
  // snapshot -> ROBUST_0 does NOT decode -> no adopt (the HW 0-byte root, direct).
  // defeat=false (fix): the robust ring floor is seated + the snapshot is window-largest-
  // sized -> ROBUST_0 DECODES -> RX adopts ROBUST_0. Returns 0 on the expected outcome.
  static int test_inband_down_resync_directed(bool defeat);

  // STAGE 3d PRE-FRAME (SEAMLESS) TEST (CLI --test-inband-seamless). Builds the REAL
  // wire window [tag burst][OFDM frame] (the DVB-S2 PLHEADER pre-frame order) and drives
  // the PRODUCTION RX pre-frame path (inband_detect_follow_from_snapshot then receive_byte
  // over the SAME snapshot). Asserts the four §15 invariants: (a) SEAMLESS — on a
  // CONFIG_10->CONFIG_9 change the FIRST OFDM frame decodes BYTE-FAITHFULLY at CONFIG_9
  // (fail-before: it is LOST at CONFIG_10); (b) NO-DEAD-TIME — a no-change window adds
  // zero latency (the absent-tag detect is a bounded cheap reject, no wait); (c)
  // CORRECT-CODE — the tag cfg_index == the config the frames are modulated at; (d)
  // LOST-TAG -> the Stage-4 down-ladder still resyncs, BREAK-count==0. fail-before
  // (-DINBAND_STAGE3D_FAILBEFORE or flag-off): the pre-frame detect is a no-op -> the
  // first frame is LOST. Returns 0=PASS, 1=FAIL. data-flow-perbatch-config.md §15.
  int test_inband_seamless();

  // IN-BAND DOWN-LADDER DELIVERY REGRESSION (CLI --test-inband-downladder). PART A: a
  // COMPLETE in-flight prev batch survives a TERMINAL-BREAK -> ROBUST_0 reshrink (fail-
  // before MERCURY_PREBREAK_DELIVER_DEFEAT=1 orphans -> 0 bytes; pass-after flushes ->
  // N*SUB_LEN bytes). PART B: a silent (0-peak) snapshot does NOT tick the dead-batch
  // streak. Drives the production deliver_complete_inflight_before_break + reshrink +
  // inband_try_down_ladder_on_decode_fail. data-flow-inband-downladder.md §3/§5.3.
  int test_inband_downladder();

  // ROBUST->OFDM ADOPT: PRESERVE THE LIVE IN-FLIGHT BURST (CLI --test-inband-adopt-preserve).
  // The last transition-class hole: the unilateral adopt INTO an OFDM config wiped the in-flight
  // OFDM preamble already mid-capture (HINGE-1 unconditional ring memset) -> FTR search_raw=0 ->
  // never acquires -> 3 total-loss -> TERMINAL BREAK -> ROBUST_0 spiral (53B vs legacy 5645B).
  // PART A: a live burst SURVIVES the adopt (fail-before MERCURY_ADOPT_FLUSH_DEFEAT=1 wipes it).
  // PART B: a cold/silent ring STILL flushes. PART C: a robust(MFSK) target STILL flushes
  // (preserve is OFDM-target-only). data-flow-robust-ofdm-adopt-flush.md §6/§8.
  int test_inband_adopt_preserve_live_burst();

  // IN-BAND ROBUST->OFDM RING-SHRINK Nofdm-INVARIANT REGRESSION (CLI
  // --test-inband-adopt-nofdm-invariant). The HINGE-1 ring-shrink
  // (force_set_capture_ring_natural) re-derived the per-symbol OFDM geometry
  // (Nofdm = Nfft+Ngi) from the LIVE ofdm.gi, which can be STALE at the cross
  // (54/256 -> 310) while the just-loaded data_container.Nofdm holds the correct
  // config geometry (3.0 ms GI -> Ngi=36 -> 292). The recompute overwrote 292 with
  // 310 -> an 18-sample/symbol FFT-window drift -> LDPC iter=0 -> garbage CRC -> the
  // CONFIG_0 under-decode (~53 B). The fix PRESERVES data_container.Nofdm across the
  // shrink (Approach A). PASS-AFTER: Nofdm invariant (292). FAIL-BEFORE
  // (MERCURY_ADOPT_NOFDM_PRESERVE_DEFEAT=1, same binary): Nofdm drifts 292->310.
  // data-flow-robust-ofdm-adopt-flush.md §15, diagnosis a468b2fc.
  int test_inband_adopt_nofdm_invariant();
  // §21 CONFIG_0-START robust-floor OVER-SEAT (CLI --test-inband-config0-start-ring): the uncovered
  // sibling of FIX #1e. A session that STARTS at CONFIG_0 (no robust->OFDM adopt) never latches
  // inband_ofdm_acq_ring_shrunk, so inband_seat_robust_ring_floor over-grows the natural OFDM ring
  // (217->~804) -> every preamble at the tail beyond upper_bound -> 0 forward decode. PASS-AFTER: ring
  // stays natural + tail preamble in-bounds. FAIL-BEFORE (MERCURY_CONFIG0_RING_GUARD_DEFEAT=1, same
  // binary): ring balloons to the robust floor + tail preamble beyond upper_bound.
  // data-flow-robust-ofdm-adopt-flush.md §21.
  int test_inband_config0_start_ring();
  // CONFIG_0 clean-lock CRC-fail ROOT: descrambler survives the inband ring-shrink
  // (set_size realloc wiped bit_energy_dispersal_sequence). data-flow-robust-ofdm-adopt-flush.md §17.
  int test_inband_descrambler_survives_ring_shrink();
  // Dead-batch streak ties to REAL batch periods + ZERO-PROGRESS (the climb-killer fix); a real
  // total loss STILL BREAKs. data-flow-robust-ofdm-adopt-flush.md §19.
  int test_inband_deadbatch_progress();
  // §19 dead-batch tick classifier (the PRODUCTION decision, called by
  // inband_try_down_ladder_on_decode_fail and driven directly by the test). Applies the streak
  // side-effects and returns: 0=PROGRESS_RESET (link alive), 1=RATE_LIMITED (same batch period),
  // 2=TICK (a genuine zero-progress real-batch-period dead batch).
  enum { INBAND_DB_PROGRESS_RESET = 0, INBAND_DB_RATE_LIMITED = 1, INBAND_DB_TICK = 2 };
  int inband_deadbatch_classify();

  // IN-BAND FORWARD-HEALTHY REVERSE-ACK MISS -> NO-BREAK DELIVER REGRESSION (CLI
  // --test-inband-deliver). The 785-frame decode-but-0-deliver rework: a forward-healthy
  // reverse-ACK turnaround MISS (nAcked_data flat) tripped the connect-liveness guard's
  // BREAK, detonating the three coupled holes (retx-clear/bsi-advance, RX partial-prev
  // wipe, config-NO-OP teardown). PART A: the guard routes a forward-healthy miss (an
  // in-flight DATA batch + a lower rung) to the NO-BREAK re-present + rolls the bsi back
  // contiguous. PART B: the RX consequence — a 24/25 PARTIAL prev is PRESERVED (no BREAK ->
  // no ROBUST_0 reshrink orphan) and delivers; fail-before the reshrink orphans it -> 0
  // bytes. PART C: a GENUINE total loss (no in-flight DATA, or at the ladder bottom) STILL
  // BREAKs. PART D: a GENUINE config-change demote STILL clears/epochs. fail-before
  // (-DINBAND_DELIVER_FAILBEFORE removes the discriminator): PART A/B FAIL (the 0-deliver),
  // PART C/D PASS (unchanged). Returns 0=PASS, 1=FAIL. data-flow-inband-retx-epoch.md §5.
  int test_inband_deliver();

  // IN-BAND CAPTURE-RING ROBUST-FLOOR OVER-SEAT TEST (CLI --test-inband-ring-floor).
  // Drives inband_seat_robust_ring_floor() at CONFIG_8 (a climbed OFDM rung holding its natural
  // ring, no adopt) and ROBUST_0; asserts the seat is SUPPRESSED at the OFDM rung (ring stays
  // natural) and STILL grows to the robust floor at ROBUST_0. The fail-before A1-FB sub-case
  // uses MERCURY_CONFIG0_RING_GUARD_DEFEAT=1 (the §21 knob, now disabling the generalized guard)
  // to reproduce the over-seat. Returns 0=PASS, 1=FAIL. data-flow-inband-ring-floor-overseat.md §5.
  int test_inband_ring_floor_overseat();

  // STAGE 4c D5 BREAK-OBSOLETE TEST (CLI --test-inband-no-break). Synthetic-fire of the
  // COMMANDER Class-A degradation routing: PART A drives inband_route_failure_demote (the
  // body all four Class-A sites call) and asserts the link DEMOTES one rung and stays
  // alive with BREAK-count==0; PART B drives the at-bottom dead-batch floor and asserts a
  // GENUINE total loss STILL reaches the SESSION_DEAD_BATCHES BREAK at exactly the Nth;
  // PART C asserts a delivery resets the floor; PART D asserts the OFF gate is false (the
  // legacy BREAK is unchanged). fail-before (-DINBAND_NOBREAK_FAILBEFORE): the gate is
  // removed -> a Class-A degradation BREAKs under inband (BREAK-count>0). Returns 0=PASS,
  // 1=FAIL. inband-reliability-design.md §5.6, data-flow-perbatch-config.md §S4C.
  int test_inband_no_break();

  // STAGE 4d D1+D4 TEST (CLI --test-inband-retag). Drives the PRODUCTION repeat-until-
  // followed + climb/auto-demote functions: PART A a CONFIG_9->CONFIG_11 chokepoint climb
  // the RX FOLLOWS UP (both ends + PHY twin); PART B inband_tag_firing_decision re-emits
  // each batch until a SACK confirms then STOPS (parity HELD across the repeats, anchor
  // latched once); PART C an un-confirmed climb past R AUTO-DEMOTES to last-confirmed with
  // BREAK-count==0; PART D the body the gated turbo BREAK sites call routes a turbo-climb-
  // fail to a tag-demote (BREAK-count==0). fail-before (-DINBAND_RETAG_FAILBEFORE): fire-
  // once + no auto-demote -> the lost climb is never followed / the hopeless climb stays
  // armed -> B/C FAIL. Returns 0=PASS, 1=FAIL. inband-reliability-design.md §1.6 / §4.6.
  int test_inband_retag();

  // STAGE 4e D2 NACK TEST (CLI --test-inband-nack). PART A NACK-EMIT: drive the RX to a
  // down-ladder total-loss -> assert a NACK is emitted (type 5, reason DECODE_FAIL,
  // correct rx_cfg + parity) via the REAL build_nack_tones round-trip (the sender
  // decodes it from the passband); drive a tag announcing an un-adoptable config ->
  // assert NACK reason UNFOLLOWABLE_CLIMB; drive a NORMAL follow -> assert NO NACK (no
  // chatter). PART B NACK-HANDLE: feed the sender a NACK whose rx_cfg is below the
  // announced config -> assert it AUTO-DEMOTES to the RX config IMMEDIATELY
  // (BREAK-count==0) WITHOUT inband_retag_count>=R (faster than the R-retry give-up).
  // fail-before (-DINBAND_NACK_FAILBEFORE): the NACK emit is a no-op + the handle is
  // removed -> the EMIT and the EARLY-demote asserts FAIL. Returns 0=PASS, 1=FAIL.
  // inband-reliability-design.md §2.6.
  int test_inband_nack();

  // STAGE 4e D3 PERIODIC RE-ANNOUNCE TEST (CLI --test-inband-reannounce). Run N+1
  // PRODUCTION firing decisions at a STEADY config with no change -> assert the tag is
  // SILENT for batches 1..N-1 and FIRES on batch N (periodic), parity HELD across the
  // re-announce, the counter resets. Inject a change at batch 3 -> assert the periodic
  // clock resets (next periodic at 3+N). fail-before (-DINBAND_REANNOUNCE_FAILBEFORE):
  // force N=0 -> the periodic never fires -> the FIRES-on-N assert FAILS. Returns
  // 0=PASS, 1=FAIL. inband-reliability-design.md §3.6.
  int test_inband_reannounce();

  // LEVER #2 — SPECULATIVE / PROMPT SACK (env MERCURY_SPEC_SACK).
  // In-process synthetic-fire (CLI --test-spec-sack), modelled on
  // test_partial_bsi_advance. Forces frame-k still-decoding at the
  // window-fraction deadline -> asserts the SACK fires IN-WINDOW with bit_k=0 ->
  // CMD retransmits k -> RSP re-receives k byte-faithful -> asserts NO
  // double-delivery and NO silent loss. turnaround-eff.md §9.
  // Returns 0=PASS, 1=FAIL. Default builds never call this.
  int test_spec_sack();

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

  // ---- zombie/amplifier layer decision helpers (data-flow-zombie-amplifier.md) ----
  // Pure predicates, shared by production + the --test-zombie-amp self-test. Each
  // reads its own MERCURY_*_DEFEAT knob so the test's fail-before arm exercises the
  // SAME code the production path runs.
  // FIX 1 (R2b, §1): should a decoded BREAK be acted on given link_status? CONNECTED
  // always; DROPPED unless MERCURY_BREAK_DROPPED_DEFEAT; every other state never.
  bool break_frame_actionable(int ls) const;
  // FIX 4 (§4): may the ROBUST_DWELL_BATCH_OP restart link/watchdog timers? Only when
  // CONNECTED (a DROPPED link must not prolong itself) unless
  // MERCURY_ROBUST_DWELL_KEEPALIVE_DEFEAT restores the unconditional restart.
  bool robust_dwell_keepalive_ok(int ls) const;
  // FIX 3 (§3): the CMD watchdog resurrection branch selector.
  enum { WD_TELEPORT = 0, WD_PROBE = 1, WD_RESUME = 2, WD_RECONNECT = 3 };
  int  watchdog_resurrect_decision(bool probe_pending, bool rx_advanced) const;

  // R5 -- split the demote trigger ("no ACK" is not "bad channel"). Pure classifier of an
  // ACK-window expiry + the one shared counter-step + the reroute predicate/gate/reconnect.
  enum { ACKFAIL_SILENT = 0, ACKFAIL_FORWARD_LOSS = 1, ACKFAIL_LATE_ACK = 2 };
  int  classify_ack_failure(int ack_pattern_ms, double peak_metric,
                            int peak_matched, bool sack_this_round) const;
  int  ackfail_classifier_step(int ack_pattern_ms, double peak_metric,
                               int peak_matched, bool sack_this_round);
  static bool silence_reroute_should_fire(int consec_pure_silent, int max_rounds,
                                          bool emergency_break_active_flag, int turbo_phase);
  bool demote_silence_reroute_enabled() const;
  void commander_clean_reconnect(const char* reason);
  void watchdog_probe_clear_on_liveness();
  int  test_demote_silence();
  // The consolidated fail-before/pass-after self-test for the three implemented
  // fixes (asserts each helper's truth table under defeat / no-defeat). CLI
  // --test-zombie-amp; also run inside --test. Returns 0=PASS, else #failures.
  int test_zombie_amp();

  // GAP-ABORT ruler-blinding regression (CLI --test-gap-abort-blind; also in
  // --test). fact-documents/data-flow-rsp-contiguity-ruler.md. Drives the REAL
  // rsp_gap_abort_teardown() (not a modeled abort), the REAL delivery-time /
  // re-adopt gap predicates, and the REAL fifo_buffer_rx app stream. Delivers a
  // contiguous prefix, drops a batch, fires the delivery-time gate through the
  // real teardown, keeps the transmitter driving the SAME stream, and asserts NO
  // byte is delivered at the post-hole stream offset. The teardown routes through
  // reset_session_state(), which clears rsp_last_delivered_batch_seq_id to -1 and
  // blinds the contiguity ruler, so the next re-adopt sees last=-1 -> no-gap ->
  // silent concatenation across the dropped batch. Self-contained (builds its own
  // telecom_system for the reset). Returns 0=PASS, 1=FAIL. Default builds never
  // call this.
  int test_gap_abort_readopt_blind();

  // GAP-ABORT stream-backstop-blinding regression (CLI --test-gap-abort-stream-blind;
  // also runs inside --test). The companion to test_gap_abort_readopt_blind: that one
  // proves the bsi contiguity ruler survives the teardown; THIS one proves the Option W
  // byte-level BACKSTOP survives it. It delivers a stamped prefix (advancing
  // rx_stream_delivered + latching rx_stream_stamp[].valid), fires the REAL
  // rsp_gap_abort_teardown(), then presents a wire stamp whose start diverges from the
  // preserved rx_stream_delivered and asserts w_stream_shift_detected() FIRES. Before the
  // fix the teardown zeroed rx_stream_delivered and invalidated every stamp, so the
  // detector could not fire (the detector blinds itself via its own teardown call at
  // arq_common.cc). Also asserts the rsp_stream_aborted latch is set by the teardown and
  // survives reset_session_state(). MERCURY_GAP_STREAM_BLIND=1 restores the pre-fix clear
  // (fail-before arm). Self-contained. Returns 0=PASS, 1=FAIL. Default builds never call this.
  int test_gap_abort_stream_backstop_blind();

  // RECOVERABLE delivery-time GAP-ABORT regression (CLI --test-gap-recover; also
  // runs inside --test). Drives the REAL gap_is_recoverable_prev_hole +
  // delivery_step_is_gap predicates, the REAL CMD retention-shadow helpers, and
  // the REAL advance_last_delivered producer. fail-before (MERCURY_GAP_RECOVER_
  // DEFEAT=1) = terminal abort, high-water frozen; pass-after = HOLD + refill +
  // in-order 6->7->8, high-water never advances while the hole exists. Returns
  // 0=PASS, 1=FAIL. See fact-documents/data-flow-recoverable-gap-abort.md.
  int test_recoverable_gap_abort();
  int test_recover_fire();

  // R2b DELIVER-HELD-CUR fire proof (CLI --test-held-cur-deliver-fire). Drives
  // the REAL production delivery path end-to-end: feeds a +2 storm topology
  // through process_messages_acknowledging_data() so the RECOVERABLE HOLD fires
  // in production code, then feeds the CMD's re-driven prev refill through
  // add_message_rx_data() + process_messages_rx_data_control() so the
  // PREV-completion + deliver-held-cur commit runs, and pops fifo_buffer_rx as
  // the app-delivered oracle (NO manual advance_last_delivered / status poke).
  // fail-before (both DEFEAT knobs=1) = held cur never delivered; pass-after =
  // both prev and held-cur bytes delivered in order via copy_data_to_buffer.
  // Returns 0=PASS, 1=FAIL. See data-flow-recoverable-gap-abort.md §5.5.
  int test_held_cur_deliver_fire();

  // Multi-window DATA-ACK/SACK correlator regression (Track A, mwcorr;
  // CLI --test-data-ack-multiwindow). Self-contained, in-process, no IONOS/RF.
  // Loads a WB config, synthesizes a real ACK+SACK passband burst via
  // generate_ack_sack_pattern_passband(), places it at an OLDER ring phase with
  // the newest tail filled with silence, and asserts:
  //   fail-before  : the newest-tail decode MISSES (decoded=false / matched<thr);
  //   pass-after   : mw_find_ack_sack_phase() (env ON) recovers the SAME
  //                  bsi/bitmap with CRC12 pass at the older phase;
  //   no-false-acc : a pure-silence ring yields no phase in BOTH modes.
  // Returns 0=PASS, 1=FAIL. Default builds never call this.
  // See fact-documents/data-flow-data-ack-sack-correlator.md §7.
  int test_data_ack_multiwindow();

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

  // CLIMB-CHURN producer-side bsi rollback (data-flow-climb-up-bsi-rollback.md
  // §6). The COMMANDER-side mirror of test_gap_abort_on_readopt: drives the REAL
  // roll_back_cmd_bsi_to_inflight() producer + the REAL sack_v2_readopt_has_gap()
  // predicate as the RSP-side delivery oracle. Reproduces a rapid mid-transfer
  // climb where cmd_batch_seq_id outran an in-flight (RSP-delivered, not yet
  // CMD-ACKed) batch. fail-before (MERCURY_CLIMB_BSI_ROLLBACK_DEFEAT=1): no
  // rollback -> re-present bsi is >=2 past last_delivered -> readopt_has_gap=true
  // -> RSP would HOLD delivery. pass-after (defeat off): the helper rolls
  // cmd_batch_seq_id back to the in-flight bsi -> contiguous successor ->
  // readopt_has_gap=false -> delivery proceeds. Variants: multi-frame same-batch
  // (earliest mod-256), mod-256 WRAP, compression-gated no-op, no-in-flight
  // no-op. Returns 0=PASS, 1=FAIL. Default builds never call this.
  int test_climb_bsi_rollback();

  // D5 — EOB-inference batch truncation (TRACK_C_D2D3D5_DESIGN.md §5.3 /
  // data-flow-prev-bump.md §8). CLI: --test-eob-loss-batch-truncation. Drives
  // the REAL bump_bsi_and_transfer_prev() (the prev_expected producer), the REAL
  // prev-completion count gate, the REAL copy_data_to_buffer() prev delivery, and
  // the REAL fifo_buffer_rx as a byte-exact oracle. A 30-frame batch loses its
  // EOB-marked tail frame (slot 29); the EOB inference latches a SHORT length
  // (29) while the wired batch_total_frames carries the true 30.
  //   fail-before (MERCURY_D5_INFER_DEFEAT=1): prev_expected=29, the count gate
  //     fires with slot 29 FREE, copy_data_to_buffer delivers the 29-frame batch —
  //     the lost tail is SILENTLY skipped, the delivered bytes != the 30-frame
  //     tx prefix (md5-false).
  //   pass-after (defeat off): prev_expected=30, received_count=29<30 → the gate
  //     HOLDS (nothing delivered); inject the slot-29 retransmit → received=30 →
  //     the gate fires → the full 30-frame batch delivers in order (faithful).
  // Returns 0=PASS, 1=FAIL. Default builds never call this.
  int test_eob_loss_batch_truncation();

  // In-band demote-rebase DOUBLE-DELIVERY (byte-stream CORRUPTION) regression
  // (data-flow-stream-offset.md -- demote-rebase double-delivery). CLI:
  // --test-dedup-rebase. Drives the REAL rsp_commit_cur_batch_delivery() delivery
  // funnel, the REAL rsp_inband_demote_rebase(), the REAL re-adopt/delivery-time
  // gap predicates, and fifo_buffer_rx as a byte-exact oracle. A batch B is
  // delivered once; a mid-transfer demote-rebase wipes cur/prev (preserving the
  // delivered high-water); the CMD retransmits B (a legitimate un-ACKed re-send)
  // and B re-adopts + re-completes -> the funnel is re-reached with fwd==0.
  //   fail-before (MERCURY_STREAM_DEDUP_DEFEAT=1): the funnel re-appends B's bytes
  //     -> fifo_buffer_rx = S1 ++ B (a duplicate segment), rx_stream_delivered
  //     inflates (the witnessed corruption).
  //   pass-after (defeat off): the emit de-dup drops the re-emit ([RSP-V2-DEDUP-
  //     DROP]); fifo_buffer_rx == S1, rx_stream_delivered held. Byte-identical.
  // Returns 0=PASS, 1=FAIL. Default builds never call this.
  int test_dedup_rebase();

  // CMD>RSP batch-size desync SILENT-CORRUPTION regression (res_c3100, silent-
  // corruption-residual.md §8 / data-flow-batch-size.md §8). Drives the REAL shared
  // batchsize_desync_detected() decision + REAL copy_data_to_buffer() delivery +
  // rsp_gap_abort_teardown() abort with fifo_buffer_rx as a byte-exact oracle.
  //   fail-before (MERCURY_BATCHSIZE_DESYNC_DEFEAT=1): the RSP delivers the 30-frame
  //     batch truncated to 25 -> the delivered stream is a silent one-batch SHIFT.
  //   pass-after (default): the (B) backstop raises [RSP-V2-BATCHSIZE-DESYNC] and
  //     aborts (link DROPPED) -> nothing silently delivered. Returns 0=PASS, 1=FAIL.
  int test_batchsize_desync_delivery();

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

  // Fix A (baseline-double-delivery.md) — a mid-flight data_batch_size SHRINK must
  // never orphan already-RECEIVED prev-batch frames (silent user-byte loss / HOLE).
  // CLI: --test-batch-shrink-orphan-defer. Drives the REAL set_data_batch_size()
  // shrink through the chokepoint with a prev batch holding RECEIVED slots in
  // [new,old), then RECONSTRUCTS the copy_data_to_buffer() delivery set (the exact
  // [0,data_batch_size) bound) and asserts ZERO delivered bytes lost/reordered.
  // MERCURY_BATCHSHRINK_ORPHAN_DEFEAT=1 reverts the fix on the SAME binary
  // (fail-before: the shrink applies and the orphaned tail bytes are dropped).
  // Returns 0=PASS, 1=FAIL.
  int test_batch_shrink_orphan_defer();

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

  // silent-corruption-residual.md §12/§13 — RE-STAGE re-queue orphan test.
  // CLI: --test-restage-requeue-orphan. Fills fifo_buffer_tx with newer app data,
  // stages an in-flight batch in messages_tx[], invokes restage_requeue_tx_messages(),
  // and asserts the re-queued in-flight bytes come out CONTIGUOUS and IN-ORDER ahead
  // of the newer data with NO loss. FAIL-BEFORE (MERCURY_RESTAGE_ORPHAN_DEFEAT=1):
  // the pre-fix push()-to-BACK reorders (and drops when full) -> shift -> FAIL.
  int test_restage_requeue_orphan();
  // Option W FOUNDATION regression (--test-stream-offset): drives the cursor
  // helpers + the production re-stage funnel through every transition, asserting
  // the latched per-bsi stamp == the true cumulative transported origin offset.
  int test_stream_offset();
  // Streaming decompress-failure SILENT-FALSE-ACCEPT regression
  // (residual-silent-corruption-wgn25.md). CLI: --test-decompress-false-accept.
  // Drives two REAL cl_compressor instances into a streaming-model desync (RX
  // reset out of lockstep with the TX — the WGN:25 out-of-order SACK regime),
  // routes the resulting undecodable compressed batch through the REAL
  // copy_data_to_buffer() reassembler with fifo_buffer_rx as a byte oracle.
  // FAIL-BEFORE (MERCURY_DECOMPRESS_RAWPUSH_DEFEAT=1): the raw compressed blob is
  // pushed to the app FIFO (silent garbage). PASS-AFTER (default): zero bytes
  // delivered + [RSP-DECOMPRESS-FALSE-ACCEPT-BLOCKED] loud detect. See
  // source/datalink_layer/test_decompress_false_accept.cc. Returns 0=PASS,1=FAIL.
  int test_decompress_false_accept();

  // R030 (race audit 2026-06-06) — v2 PENDING_ACK flip aliasing test.
  // CLI: --test-v2-pendingack-flip-alias. Builds a v2 MIXED batch with the
  // messages_tx[] array-index space DIVERGED from the wire positions (holes + a
  // retx prefix), drives the REAL v2_flip_resolve_slot() for every batch slot,
  // and asserts retx-prefix slots are skipped (-1), new-data slots resolve to the
  // correct diverged array index (not the wire id), no FREE/foreign slot is left
  // PENDING_ACK, and the pre-fix wire-id flip WOULD have poisoned a non-owning
  // slot. Returns 0=PASS, 1=FAIL.
  int test_v2_pendingack_flip_alias();

  // Timing redesign — retx-never-rides-wire-slot-0 regression.
  // CLI: --test-retx-slot-order. Reconstructs the v2 mixed-batch layout exactly
  // as the fill loops leave it (retx block at [0..R-1] carrying original bsi +
  // original sequence bytes incl. the EOB bit7; new-data at [R..R+ND-1] with
  // pos_in_new_batch sequence numbers, EOB on the last new-data slot), then
  // drives the REAL v2_rotate_retx_behind_lead() and v2_flip_resolve_slot().
  // Asserts A1 slot 0 carries current-bsi new-data (fail-before: with the rotate
  // defeated via MERCURY_RETX_SLOT0_ROTATE_DEFEAT, slot 0 is the retx bsi ->
  // FAIL), A2 EOB set only on the final slot, A3 retx block bytes verbatim, A4
  // v2_flip_resolve_slot == -1 exactly for the retx slots + a unique valid slot
  // per new-data slot, A5 ND==1 layout byte-identical to the legacy prefix.
  // Returns 0=PASS, 1=FAIL.
  int test_retx_slot_order();

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
  // Fix H#3 (delivery-integrity-audit-monitor.md §3): worst-case number of app-FIFO
  // bytes the CURRENT batch will deliver via copy_data_to_buffer(). Used to GATE the
  // clean data-ACK + delivery on fifo_buffer_rx having room, so the ACK is never sent
  // (and the batch never freed) while the un-stored tail would be dropped under app
  // back-pressure (fifo_push_rx short) = post-ACK silent loss the CMD never retransmits.
  // Compression leg: the decompressed size is unknown pre-delivery, so the safe bound is
  // the decompress workspace (COMPRESS_WORKSPACE_SIZE). No-comp leg: exact = sum of the
  // RECEIVED slot lengths in [0,data_batch_size).
  int rx_fifo_batch_need();
  int test_rxfifo_backpressure_hold();  // --test-rxfifo-backpressure-hold: Fix H#3 post-ACK loss regression
  void restore_backup_buffer_data();
  void restore_tx_from_compressed();  // Decompress messages_tx back to raw in fifo_buffer_tx
  // RE-STAGE re-queue (silent-corruption-residual.md §12): re-queue every
  // in-flight (non-FREE) messages_tx[] frame back into fifo_buffer_tx for
  // re-framing at the demoted/BREAK config. ORDER-PRESERVING (reverse-iter +
  // push_front so the re-sent block is CONTIGUOUS ahead of any newer app data)
  // and LOSSLESS (a would-be drop when the fifo lacks room is surfaced LOUD via
  // [RESTAGE-ORPHAN], never a silent orphan -> reassembly shift). Replaces the
  // 10 open-coded re-stage loops. MERCURY_RESTAGE_ORPHAN_DEFEAT=1 restores the
  // pre-fix buggy behavior (forward-iter push()-to-BACK, unconditional FREE) for
  // the --test-restage-requeue-orphan fail-before arm.
  void restage_requeue_tx_messages();

  // Option W cursor helpers (fact-documents/data-flow-stream-offset.md §2).
  // stream_tx_latch: THE build-commit — latch stamp[bsi] = {tx_stream_committed,
  //   transported_len} and advance the cursor. Called once per new-data batch build
  //   from the ONE pop funnel process_buffer_data_commander().
  // stream_tx_rollback_inflight: the re-stage un-commit — reset tx_stream_committed to
  //   the in-flight batch's latched start (LIFO, idempotent, guarded). Called at every
  //   re-stage byte-restore point (the two funnels + the open-coded BREAK legs).
  void stream_tx_latch(int bsi, uint32_t transported_len);
  void stream_tx_rollback_inflight();

  // Option W STEP 3 (data-flow-stream-offset.md §8.6): portable reflected CRC-32 (IEEE
  // 802.3, poly 0xEDB88320) byte-fold. Pure/static; both peers fold identically so the
  // running registers compare directly. Used by the TX build-leg folds, the RX delivery
  // fold, and the deterministic test — one implementation, no table-init race.
  static uint32_t crc32_update(uint32_t crc, const void* data, int len);

  // Option W STEP 3 — the EOT (end-of-transfer) decision predicate (pure; no side effects;
  // no socket). At a clean disconnect the CLOSE_CONNECTION frame carries the SENDER's final
  // total_committed_bytes + running tx_stream_crc; this RECEIVER asserts its rx_stream_delivered
  // + rx_stream_crc match. true ⇒ a genuine last-batch tail-drop / final-batch content
  // corruption (the one blind spot the per-batch W checks cannot see — a truncated FINAL batch
  // has no next-batch backstop). FALSE-FIRE safe: a complete transfer (delivered==committed,
  // crc==crc) returns false; a no-data close (0==0, INIT==INIT) returns false; a clean
  // robust-only session matches (both cursors are config-independent). Production (the RSP
  // CLOSE handler) AND test_stream_offset Part U both call this — same decision on both paths.
  bool w_eot_mismatch(uint64_t peer_committed, uint32_t peer_crc) const;

  // Option W CORE (F4.2, silent-corruption-residual.md §16/§17.5): invalidate every
  // parsed RX wire stamp on a config change. A config change ALWAYS re-stages the
  // sender's in-flight batch, so any rx_stream_stamp[] parsed-but-not-yet-delivered is
  // STALE by construction; a demote-to-ROBUST rebuild is the killer (robust frames carry
  // NO stamp → a stale OFDM-sized stamp[bsi].length can never refresh → the PRIMARY
  // byte-gate WITHHOLDs forever → BREAK spiral). Called from load_configuration() on the
  // RX config-apply. Both W predicates no-op on an invalid stamp. Also kills the 256-wrap
  // stale-start false teardown. MERCURY_W_CFG_STAMP_KEEP=1 = fail-before (keep the stale).
  void rx_stream_invalidate_stamps();

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
  // D5 (TRACK_C_D2D3D5_DESIGN.md §5.3): does the v2 DATA header carry the
  // batch_total_frames byte on THIS config? TRUE for OFDM multi-frame configs,
  // FALSE for robust / batch=1 (D5 is meaningless at batch=1 and the byte would
  // steal the scarce ROBUST_0 payload, re-opening the streaming-compression
  // deadlock floor). Recomputed in load_configuration() from the config being
  // loaded so TX and RX (both re-run load_configuration on the SET_CONFIG
  // handshake) agree on the wire header length. Default true (set in init).
  bool header_carries_d5;
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

  // R2c — PREV-BATCH RETENTION SHADOW. The mixbatch builder REMOVES a retx frame
  // from retransmit_frames[] the instant it is SENT (arq_commander.cc:2360-2370,
  // shift-down + count-=R). If that retx is then LOST, its bytes are gone and a
  // later prev-bsi re-SACK cannot refill the hole (the verified inert-root:
  // the SACK apply reads messages_tx[i] BY SLOT INDEX, which now holds the NEXT
  // batch). This shadow RETAINS a copy of each retx frame's original bytes,
  // keyed by its ORIGINAL batch_seq_id, until that bsi is confirmed delivered.
  // On a prev-bsi re-SACK whose bsi is NOT the in-flight batch (the case the
  // OOW/STALE guards currently DROP), the retained bytes are re-queued into
  // retransmit_frames[] so the next mixbatch re-drives the frame. Bounded to
  // CMD_PREV_RETAIN_MAX slots and RSP_GAP_RECOVER_MAX re-drive rounds per bsi.
  // ADDITIVE: the existing OOW/STALE/DUP guards keep their exact behavior on the
  // in-flight batch; this only adds a recovery branch on the dropped path — no
  // bitmap is applied to messages_tx[] by index, so no false-ACK surface is
  // added. Knob MERCURY_CMD_PREV_RETAIN_DEFEAT disables capture + re-queue.
  // See data-flow-recoverable-gap-abort.md §5 (R2c).
  int           cmd_prev_retain_count;                               // live shadow entries
  int           cmd_prev_retain_bsi[CMD_PREV_RETAIN_MAX];            // original batch_seq_id
  int           cmd_prev_retain_slot[CMD_PREV_RETAIN_MAX];           // original slot in that batch
  int           cmd_prev_retain_len[CMD_PREV_RETAIN_MAX];            // payload length
  int           cmd_prev_retain_type[CMD_PREV_RETAIN_MAX];           // DATA_LONG / DATA_SHORT
  unsigned char cmd_prev_retain_seq_eob[CMD_PREV_RETAIN_MAX];        // original seq byte (bit7=EOB)
  int           cmd_prev_retain_rounds[CMD_PREV_RETAIN_MAX];         // re-drive rounds used
  unsigned char cmd_prev_retain_bytes[CMD_PREV_RETAIN_MAX][MAX_SACK_FRAME_SIZE];

  // R2c helpers (arq_commander.cc). capture: stash a just-SENT retx frame keyed
  // by its original bsi (dedup by bsi+slot). evict: drop all shadow entries for
  // a bsi once it is confirmed delivered. requeue: on a prev-bsi re-SACK, push
  // the still-missing retained frames for that bsi back into retransmit_frames[]
  // (bounded); returns the number re-queued.
  void cmd_prev_retain_capture(int bsi, int slot, int len, int type,
                               unsigned char seq_eob, const unsigned char* bytes);
  void cmd_prev_retain_evict(int bsi);
  int  cmd_prev_retain_requeue(int bsi, const bool* got_bitmap, int nframes);
  // R2c-W1 (data-flow-recoverable-gap-abort.md 5.1). has: is any shadow entry
  // keyed by this bsi still live? is_shadow_target: route a decoded PARTIAL
  // re-SACK to the shadow re-drive (instead of the by-slot-index apply) iff its
  // bsi is NOT the current new-data batch AND its frames are shadow-retained --
  // covers the storm the !inflight gate MISSED (rx_bsi == prev_bsi, one batch
  // past the armed-prev hole: in the {cmd,prev} window so NOT OOW/STALE-dropped,
  // yet its frames left messages_tx[] on their retx send so a by-slot apply would
  // false-ACK the current batch). Gated by MERCURY_CMD_PREV_RETAIN_DEFEAT.
  bool cmd_prev_retain_has(int bsi);
  bool cmd_prev_resack_is_shadow_target(int rx_bsi);

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

  // ===== Option W — absolute-byte-stream cursors (fact-documents/data-flow-stream-offset.md) =====
  // TCP-style end-to-end stream-position invariant: the RSP's cumulative delivered
  // TRANSPORTED-byte offset at the start of each batch == the CMD's committed offset
  // at that batch's build. Catches every positional shift/hole/reorder/dup — the 4
  // silent byte-corruption mechanisms of silent-corruption-residual.md §11-§14 — at
  // the FIRST divergent byte, regardless of which producer/layer caused it. Offset
  // domain = TRANSPORTED (post-compression) bytes counted at the pop/push funnels
  // (data-flow-stream-offset.md §0). FOUNDATION = cursors only (no wire, no gate).
  // Option W STEP 3 (data-flow-stream-offset.md §8.6): the `.crc` field is the per-bsi
  // rollback ANCHOR for the running stream CRC-32 (the CRC value BEFORE this batch's bytes
  // were folded). Snapshotted at latch, restored on re-stage rollback — mirrors `.start`
  // EXACTLY so a rebuild re-folds from the correct anchor (INV4 for the CRC). Unused on the
  // rx_stream_stamp[] (wire-parse) side.
  struct StreamStamp { uint64_t start; uint32_t length; bool valid; uint32_t crc; };
  uint64_t tx_stream_committed;        // CMD: cumulative transported bytes committed to built
                                       //      batches, in build order (== next new-batch start).
  StreamStamp tx_stream_stamp[256];    // CMD: per-bsi latched {start,length}. Latched at BUILD,
                                       //      re-emitted verbatim on retx/mixbatch, start restored
                                       //      on re-stage rollback. Index = batch_seq_id & 0xFF.
  uint64_t rx_stream_delivered;        // RSP: cumulative transported bytes delivered through
                                       //      copy_data_to_buffer(), in delivery order.
  // Option W STEP 3 — the running stream CRC-32 accumulators (data-flow-stream-offset.md §8.6).
  // tx_stream_crc folds every committed transported byte at the ONE build funnel (both legs),
  // with the per-bsi rollback anchor above; rx_stream_crc folds every delivered transported
  // byte at the ONE receiver funnel (copy_data_to_buffer). Config-INDEPENDENT (folded at the
  // funnels, not gated by w_stamp_rides), so — unlike the per-batch wire stamp — the running
  // CRC (and the byte counters) are maintained end-to-end at ALL configs incl. robust/cfg0.
  // Compared at end-of-transfer by the EOT check (w_eot_mismatch). Reset to CRC32_INIT at
  // session reset + ctor. Running register (no final XOR); both peers fold identically.
  uint32_t tx_stream_crc;              // CMD: running CRC-32 over committed transported bytes.
  uint32_t rx_stream_crc;              // RSP: running CRC-32 over delivered transported bytes.
  // Option W CORE (STEP 2, data-flow-stream-offset.md §8): the per-bsi stamp PARSED
  // from the wire EOB frame on the RSP. .start holds start_lo32 (low 32 bits; high
  // bits 0), .length holds length16, .valid set on parse. The RSP PRIMARY byte-gate
  // (arq_responder ACK-GATE) and the BACKSTOP (copy_data_to_buffer) read this by
  // decrypt_delivered_bsi / rsp_current_expected_batch_seq_id. Index = batch_seq_id&0xFF.
  StreamStamp rx_stream_stamp[256];

  // Option W: does the EOB-frame byte-stream stamp ride at the CURRENT config? Both
  // peers run load_configuration() on the SET_CONFIG handshake, so max_data_length /
  // header_carries_d5 / sack_v2_enabled agree — the predicate is DETERMINISTIC on both
  // ends with NO wire negotiation (mirrors header_carries_d5). Robust / batch=1 configs
  // (header_carries_d5==false) carry no stamp; tiny-frame OFDM configs where reserving
  // W_EOB_RESERVE would starve payload are also skipped (the 4 captured mechanisms all
  // occur at cfg13-16 with ample frame size). max_frame here == the build-side
  // max_frame at arq_commander.cc:20156 (same operands), so TX reserve == RX parse gate.
  bool w_stamp_rides() const
  {
    if(!sack_v2_enabled || !header_carries_d5) return false;
    int mf = max_data_length + max_header_length
             - effective_data_long_header_length(sack_v2_enabled, header_carries_d5);
    return mf >= W_STAMP_MIN_MAXFRAME;
  }
  // Option W CORE: if the current RX frame (message_TxRx_byte_buffer) is an EOB DATA
  // frame carrying the byte-stream stamp (w_stamp_rides()), parse {start_lo32,length16}
  // at [stamp_off..stamp_off+5] into rx_stream_stamp[batch_seq_id] and return the bytes
  // consumed (W_EOB_STAMP_BYTES); else return 0 (payload offset unchanged). The caller
  // is receive()'s DATA_LONG/DATA_SHORT parse; stamp_off = the effective header length
  // (right after the D5 byte). messages_rx_buffer.batch_seq_id must be set first.
  int w_parse_eob_stamp(int stamp_off);

  // Option W CORE — the TX twin of w_parse_eob_stamp: write the {start_lo32,length16}
  // stamp for batch `bsi` into message_TxRx_byte_buffer[stamp_off..stamp_off+5] from the
  // LATCHED tx_stream_stamp[bsi], returning the bytes written (W_EOB_STAMP_BYTES if the
  // stamp rides at this config, else 0). Called from send_batch on the EOB frame.
  int w_emit_eob_stamp(int stamp_off, int bsi);

  // Option W CORE — the two RSP-check DECISION predicates (pure; no side effects; no
  // socket). Production (ACK-GATE / copy_data_to_buffer) AND the deterministic
  // regression (test_stream_offset) both call these, so the test exercises the SAME
  // decision the wire path uses (the batchsize_desync_detected() pattern).
  //   w_bytegate_shortfall: PRIMARY — do the DELIVERED bytes for batch wbsi (Σ
  //     RECEIVED/ACKED messages_rx[i].length, i<data_batch_size) fall SHORT of the
  //     committed wire stamp.length? true ⇒ WITHHOLD the clean ACK. Absent/zero-length
  //     stamp ⇒ false (no-op).
  //   w_stream_shift_detected: BACKSTOP — does batch wbsi's wire stamp.start diverge
  //     from the receiver's absolute delivered cursor (positional shift/hole/dup)?
  //     true ⇒ LOUD teardown. Absent stamp ⇒ false (no-op).
  bool w_bytegate_shortfall(int wbsi);                        // reads messages_rx[] (in-order path)
  // Array-parameterized twin: the cross-storage PREV completion (arq_responder.cc:1287)
  // delivers messages_rx_prev[], so its byte-gate must reconcile bytes over THAT array
  // (not messages_rx[]). Identical logic; reused so the PREV gate matches the primary gate.
  bool w_bytegate_shortfall(int wbsi, struct st_message* arr);
  // Option B' completeness (data-flow-batch-size.md §9): window-parameterized worker.
  // The delivered-byte sum must span the EFFECTIVE window (rx_effective_window), not the
  // stale data_batch_size, or a genuine CMD>RSP over-count (res_c3100 widen) drops the
  // tail frames [data_batch_size, D5) from the sum -> a deterministic FALSE byte-shortfall
  // -> permanent WITHHOLD. Production passes the eff_window explicitly (current-batch
  // ACK-GATE and cross-storage PREV completion); the two thin overloads above default to
  // data_batch_size (byte-identical for every non-desync caller and the regression test).
  bool w_bytegate_shortfall(int wbsi, struct st_message* arr, int win);
  bool w_stream_shift_detected(int wbsi);

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
  // INV-DEDUP (data-flow-stream-offset.md -- demote-rebase double-delivery): the
  // mod-256 batch_seq_id of the batch whose bytes were LAST ACTUALLY APPENDED to
  // fifo_buffer_rx (advanced at copy_data_done, AFTER the append). DISTINCT from
  // rsp_last_delivered_batch_seq_id, which the commit helpers advance BEFORE they
  // call copy_data_to_buffer() (so it cannot be the de-dup key without a bootstrap
  // error). The single byte-delivery funnel copy_data_to_buffer() refuses to
  // re-emit a batch whose wire bsi (decrypt_delivered_bsi) equals this high-water
  // (fwd==0) " the demote-rebase re-adopt + lost-ACK retransmit both re-reach the
  // funnel with fwd==0 and would otherwise double-append. Like
  // rsp_last_delivered_batch_seq_id it SURVIVES the in-band demote-rebase
  // (arq_responder.cc) and the BREAK reset; it is cleared ONLY at a true session
  // boundary (ctor + reset_session_state + KEY_ACTIVATE), mirrored wherever
  // rx_stream_delivered re-anchors to 0. -1 = nothing emitted yet this LINK.
  int rx_stream_emitted_bsi_hw;
  // Data-integrity latch (the gap-abort "unrepresentable unsafe state" keystone).
  // A gap-abort teardown (rsp_gap_abort_teardown) is a MID-TRANSFER abort of a
  // stream the peer keeps driving (no OTA abort frame is sent), so the CMD re-drives
  // into a torn-down RSP. Preserving the contiguity ruler catches a NON-contiguous
  // re-adopt, but a re-adopt at exactly last+1 looks contiguous to the ruler and
  // would be delivered onto a DROPPED transfer — a residual hole the ruler alone
  // cannot close. This sticky flag makes ALL delivery decisions refuse
  // UNCONDITIONALLY after an abort, independent of any ruler/cursor/stamp value:
  //   set    true  inside rsp_gap_abort_teardown() (BEFORE reset_session_state()).
  //   SURVIVES reset_session_state() (that function never touches this field — a
  //            mid-transfer abort is NOT a session boundary).
  //   cleared false in EXACTLY ONE place: the RSP START_CONNECTION accept
  //            (arq_responder.cc) — a genuine NEW session. Nowhere else.
  // Consumers (OR this in ahead of the ruler predicate): the cur<0 re-adopt gate
  // (arq_responder.cc), the BATCH-DONE + PREV delivery-time gates, and the pre-BREAK
  // prev flush (arq_common.cc). ctor-init false. See the cross-layer data-flow audit
  // for the delivery contiguity ruler.
  bool rsp_stream_aborted;
  // Reconnect-continuity fail-closed (data-flow-reconnect-continuity.md §5b). A fresh
  // START_CONNECTION re-anchors both absolute-byte cursors at 0 (reset_session_state), but
  // the RSP app DATA socket is PERSISTENT across the modem-link reconnect (the FIX-6 note at
  // reset_session_state: "A fresh session must not re-emit bytes from the previous
  // connection's stream"). So if the prior session delivered N>0 app bytes to that socket and
  // a fresh session then streams the sender's CURRENT position onto it, those bytes land at
  // app position N with NO proof they equal corpus[N] — a SILENT cross-session skip that every
  // per-session guard misses (each session's cursor is self-consistent). Two fields close it:
  //   rsp_prev_session_app_delivered — the app-delivered high-water of the PRIOR session.
  //     Snapshotted from rx_stream_delivered inside reset_session_state() BEFORE it zeroes the
  //     cursor, and SURVIVES that reset (never re-zeroed there). ctor-init 0.
  //   rsp_cross_session_seam_armed — set at the fresh START_CONNECTION accept (the sole arm
  //     point) when the app socket is persistent AND prev-delivered>0 AND no negotiated resume
  //     proved continuity; consumed (and cleared) at the first delivery in copy_data_to_buffer,
  //     which REFUSES it (loud clean drop via rsp_gap_abort_teardown + DISCONNECTED) rather
  //     than splice. Disarmed at every session boundary (reset_session_state). ctor-init false.
  // Byte-identical when not armed: a continuous session never crosses reset (prev stays 0); a
  // fresh transfer has prev 0. MERCURY_RECONNECT_FAILCLOSED_DEFEAT=1 keeps it disarmed (the
  // pre-fix silent splice — the fail-before A/B arm). See the cross-layer data-flow audit.
  uint64_t rsp_prev_session_app_delivered;
  bool     rsp_cross_session_seam_armed;
  // Test-only harness input (in-process --test-stream-offset has no real TCP socket): forces the
  // "app DATA socket is persistent" signal the production arm reads from tcp_socket_data status,
  // so the test can drive the REAL reset-snapshot -> arm -> refuse path. ctor-init false; never
  // set on any production path.
  bool     rsp_test_force_app_persistent;
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
  // R2a — RECOVERABLE delivery-time GAP-ABORT. When the BATCH-DONE / PREV gate
  // would fire on a RECOVERABLE one-frame hole (the completed current batch is
  // exactly +2 past last_delivered AND the single missing batch is the armed,
  // partially-received prev), HOLD the current batch (revert its slots
  // ACKED->RECEIVED, do NOT deliver / clean-ACK / advance the high-water) and
  // re-send the prev-bsi partial SACK so the CMD re-drives the missing frame.
  // Bounded: after RSP_GAP_RECOVER_MAX unanswered rounds fall through to the
  // UNCHANGED terminal rsp_gap_abort_teardown. Reset to 0 on any forward
  // delivery (BATCH-DONE / PREV commit), teardown, and ctor/session reset.
  // The delivery-time contiguity ruler (delivery_step_is_gap) stays the
  // inviolable assert BENEATH the hold — the hold is strictly UPSTREAM and
  // delivers/advances nothing while the hole exists. See
  // fact-documents/data-flow-recoverable-gap-abort.md.
  int rsp_gap_recover_rounds;            // RSP: consecutive recoverable-HOLD rounds
                                         //      for the current prev-batch hole.
  // R2b — the effective batch size (frame-count completeness oracle) the
  // BATCH-DONE gate evaluated at the instant it HELD the current batch. The hold
  // deliberately clears last_received_end_of_batch_seq / rx_batch_total_frames,
  // so the oracle must be captured when it is still known-true. Used at
  // PREV-completion to gate the deliver-held-cur commit (all [0,expected-1] slots
  // of messages_rx[] must still be RECEIVED). 0 = no held batch pending. Cleared
  // on delivery, teardown, and ctor/session reset. See
  // fact-documents/data-flow-recoverable-gap-abort.md §5.5.
  int rsp_gap_hold_cur_expected;
  int rsp_deferred_batch_shrink;         // RSP: a data_batch_size SHRINK that was
                                         //      DEFERRED because applying it now
                                         //      would orphan already-RECEIVED
                                         //      prev-batch frames in [new,old)
                                         //      (baseline-double-delivery.md Fix A).
                                         //      -1 = none pending. Applied by
                                         //      rsp_apply_deferred_batch_shrink()
                                         //      once the prev batch delivers/clears.
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

  // RSP: count of batches whose streaming decompress FAILED and whose undecodable
  // raw compressed bytes were REFUSED delivery to the app (copy_data_to_buffer's
  // former silent-false-accept raw-push — residual-silent-corruption-wgn25.md).
  // Each increment is a LOUD [RSP-DECOMPRESS-FALSE-ACCEPT-BLOCKED] detect: a would-be
  // silent byte-corruption turned into a detectable zero-byte gap + stream resync.
  // Monotonic diagnostic; the --test-decompress-false-accept oracle reads it.
  long long rsp_decompress_false_accept_blocked = 0;

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
  // R4 (LINK-PARAMS quiesce gate): defer an Axis-2 move whose batch boundary is UNCLEAN
  // (retransmit_count > 0 || !last_batch_fully_acked) so a renegotiation is not queued
  // mid-transfer — which would mix old-bsi retransmits into the first, riskiest batch of
  // the new geometry (RFC 1191/8201: new params apply to fresh data; AX.25 v2.2 XID
  // renegotiates only outside active recovery). Bounded so a sustained-loss link that never
  // reaches a clean boundary cannot stall the move forever: after this many consecutive
  // deferrals the move is FORCED (and logged).
  static const int AXIS2_MAX_MOVE_DEFER = 3;

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
  // R4 quiesce gate: consecutive deferrals of a pending Axis-2 move because the batch
  // boundary was unclean (retx pending / last batch not fully ACKed). Reset to 0 when a
  // move fires (clean or forced) or when no move is pending. Bounded by AXIS2_MAX_MOVE_DEFER.
  int   axis2_deferred_moves;
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
  //
  // Timing redesign — the retx block is no longer necessarily a LEADING prefix.
  // v2_retx_block_start is the first slot of the contiguous retx block (0 =
  // legacy leading prefix; 1 after v2_rotate_retx_behind_lead() moves it behind
  // one leading new-data frame). v2_retx_block_count == the old v2_retx_prefix_count
  // (number of retx-block slots). v2_slot_is_retx(idx) is the single predicate
  // every consumer uses to test "is this batch slot a retx-block frame" — it is
  // position-independent, so it is correct both before and after the rotation.
  int v2_retx_block_start;
  int v2_retx_block_count;
  inline bool v2_slot_is_retx(int idx) const
  {
    return v2_retx_block_count > 0
        && idx >= v2_retx_block_start
        && idx <  v2_retx_block_start + v2_retx_block_count;
  }

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

  // IDLE-SWITCHROLE-RACE (idle-switchrole-race.md §2/§5.5) — per-session "this
  // connection has emitted at least one NEW data frame". Gates the idle
  // SWITCH_ROLE handoff (process_buffer_data_commander) so a freshly-connected
  // empty-tx Commander that never sent data does NOT hand its role away (the
  // connected-but-0-deliver race). NOT reuse stats.nSent_data: that is CUMULATIVE
  // across sessions (reset only at init, arq_common.cc:527) and CONNECT skips
  // reset_session_state, so it would re-open the race on session #2. SET at the
  // single new-data-frame commit (arq_commander.cc:1738, beside nSent_data++);
  // RESET in reset_session_state() AND at Commander connect-accept.
  bool session_data_frame_sent;
  // Symmetric per-session "this connection has DELIVERED at least one RX data
  // frame" — the discriminator for the BREAK no-progress teardown (§3). NOT raw
  // stats.nReceived_data (also cumulative-across-sessions). SET at the RX-data
  // delivery commit (arq_responder.cc:101, beside nReceived_data++); RESET with
  // the other session flags.
  bool session_data_frame_received;

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

  // ====================================================================
  // In-band rate adaptation — Stage 2 (unilateral-config-tag-design.md §5)
  // ====================================================================
  // ADDITIVE state, gated by the MERCURY_INBAND_RATE env flag. With the flag
  // unset NONE of this is read or written on a production path (the emit/detect
  // helpers early-return before touching it), so default-off is byte-identical
  // to the SET_CONFIG baseline. cfg_index in the tag is a LADDER INDEX into
  // FULL_CONFIG_LADDER[] (design §2.1), NOT a raw config id.
  //
  // inband_last_announced_config: the raw config id the TX last attached a
  //   CONFIG_TAG for. CONFIG_NONE = nothing announced yet (the first emit on a
  //   fresh session always tags). Updated only inside emit_config_tag_if_changed.
  // inband_tx_epoch_parity: toggles on every committed config CHANGE (the ARDOP
  //   Even/Odd analog, design §2.1) — distinguishes a fresh change from a stale
  //   re-detect of the previous tag. RX honours it via the wrap-decode bind gate.
  // inband_rate_enabled: cached MERCURY_INBAND_RATE flag (resolved once via
  //   inband_rate_feature_enabled()).
  int     inband_last_announced_config; // CONFIG_NONE until the first tag
  uint8_t inband_tx_epoch_parity;       // 0/1, toggles per committed change
  int     inband_rate_enabled;          // -1 = unresolved, 0 = off, 1 = on
  // A3 demote-decouple env cache (data-flow-inband-a3-decouple.md): the env half of
  //   inband_a3_decouple_enabled(). -1 = unresolved, 0 = off, 1 = on. Env-keyed (resolved
  //   once + ctor-cached, NOT reset per-session — same discipline as inband_rate_enabled).
  //   The live gate ALSO requires cumulative_ack_enabled (the negotiated A3 self-heal spine).
  int     inband_a3_decouple_env;       // -1 = unresolved, 0 = off, 1 = on

  // ── STAGE 4d — D1 repeat-until-followed + D4 climb/auto-demote (inband-reliability-
  //    design.md §1/§4) ──
  // After a committed config change the sender RE-EMITS the CONFIG_TAG on EVERY
  // subsequent batch (emit_config_tag_passband firing-policy clause (b)) until a
  // returning SACK proves the RX is operating at the announced config; then it STOPS
  // (back to zero steady-state overhead). The confirm is IMPLICIT (design §1.1/§5.2): a
  // SACK acking a bsi at-or-after inband_announce_bsi proves the RX demodulated a batch
  // sent AT the announced config (it could not have produced that SACK otherwise). The
  // R floor (inband_retag_min, default 3, MERCURY_INBAND_RETAG_MIN) is the give-up
  // trigger: a CLIMB still un-confirmed after R re-tags AUTO-DEMOTES to the last-
  // confirmed config via the tag (NEVER a BREAK, design §4.3). All members ADDITIVE,
  // gated by MERCURY_INBAND_RATE (default-off byte-identical). Init in arq_common.cc
  // next to the other inband state + reset_session_state.
  bool inband_retag_armed   = false;        // a change announced but NOT yet confirmed -> re-emit
  int  inband_retag_config  = CONFIG_NONE;  // the announced config being repeated
  int  inband_announce_bsi  = -1;           // bsi of the FIRST batch at retag_config (confirm anchor)
  int  inband_retag_count   = 0;            // re-tags emitted since arm (the R counter)
  int  inband_retag_min     = -1;           // cached MERCURY_INBAND_RETAG_MIN (-1=unresolved, default 3)
  int  inband_last_confirmed_config = CONFIG_NONE; // highest config a SACK has CONFIRMED (demote floor)
  int  inband_pre_announce_config   = CONFIG_NONE; // config BEFORE the announced change (climb-up basis)
  // Resolve+cache the R floor (>=1). MERCURY_INBAND_RETAG_MIN, default 3.
  int  inband_retag_min_count();
  // PIPELINE-THE-CLIMB predicate (inband-reliability-design.md §1.8 — the climb-latency
  // fix). True when a CLIMB-UP re-tag is armed under the inband feature: in that state
  // FRAME-UP (arq_commander.cc:5424) advances OPTIMISTICALLY on a forward-healthy data ACK
  // (no wait for the climbed-to rung's CLEAN fully-acked confirm), so the climb PIPELINES
  // CONFIG_N->N+1->N+2 over consecutive batches and a single trailing SACK confirms the
  // whole ramp — instead of the ~12.4s/rung serialization that pinned the redesign low.
  // The overshoot net (inband_retag_escalate_if_climb_exhausted) recovers a too-eager
  // climb to inband_last_confirmed_config, so the §9 clean-batch protection is preserved
  // (and the legacy strict gate is byte-identical: false when off / no climb armed).
  bool inband_pipeline_climb_active();
  // IN-BAND CONFIG_0 ROLLING-PARTIAL climb unblock
  // (fact-documents/data-flow-inband-frame0-rolling-partial.md §2). PURE predicate (no I/O):
  // returns true iff the in-band feature is on, the live config is an OFDM-tier config, and the
  // LAST partial SACK was a LEAD-FRAME-ONLY loss (last_partial_lead_frame_only). At CONFIG_0+
  // the first OFDM frame of each batch bears the Schmidl-Cox acquisition burden and can fail the
  // pre-LDPC SKIP-VAR gate while frames 1..N-1 ride the locked timing → a rolling 5/6 partial
  // the retx recovers within one batch. Such a batch is a VIABLE rung: it must ADVANCE the
  // FRAME-UP clean-streak, not veto it. A multi-frame-drop partial (a genuinely marginal rung)
  // leaves last_partial_lead_frame_only false → this returns false → strict-clean gate stands.
  // Flag-off / legacy → false (byte-identical). Replayed by --test-inband-frame0-partial.
  bool inband_lead_frame_only_partial();
  // IN-BAND ROLLING-PARTIAL climb DEFER-while-hole-outstanding
  // (fact-documents/data-flow-inband-frame0-rolling-partial.md §10). PURE predicate (no I/O):
  // returns true iff the in-band feature is on AND the retransmit queue is non-empty
  // (retransmit_count > 0) — i.e. the current/prev batch still has an UNFILLED HOLE that the
  // lead-frame-only partial enqueued for retx (arq_commander.cc:4106 / the MFSK-suffix big block)
  // but which has NOT yet been re-delivered (the mixbatch drains it at arq_commander.cc:1919).
  // The ROLLING-PARTIAL anchor-raise + the FRAME-UP config-change must NOT fire while this is true
  // — firing the config-change calls clear_retx_queue() (arq_commander.cc:5708), which ABANDONS the
  // outstanding frame-0 under the new epoch → the RSP never receives it → bsi gap → GAP-ABORT wedge.
  // 2801d7c's streak CREDIT (consecutive_data_acks++) is preserved; only the anchor-raise + the
  // config-change-FIRE are DEFERRED until the hole drains (retransmit_count==0), at which point the
  // already-built streak fires the climb on the next whole batch. A genuinely CLEAN/WHOLE batch
  // leaves retransmit_count==0 → this is false → the climb fires promptly (2801d7c forward-climb
  // preserved). Flag-off / legacy → false (byte-identical). Replayed by --test-inband-frame0-partial.
  bool inband_climb_hole_outstanding();
  // D1 implicit-confirm consumer: a returning SACK acked bsi `rx_bsi`. If the re-tag is
  // armed and rx_bsi is at-or-after inband_announce_bsi (mod-256 forward distance), the
  // announced config is CONFIRMED FOLLOWED: DISARM the re-tag, record
  // inband_last_confirmed_config, and STOP re-emitting. No-op when not armed / stale bsi.
  // Returns true if it disarmed (confirmed). Called from every SACK accept site.
  bool inband_retag_confirm_from_sack(int rx_bsi);
  // KEYSTONE (data-flow-inband-tier-crossing.md §6): DATA-DECOUPLED intra-tier CLIMB confirm.
  // When an EMITTED CLIMB re-tag is armed and the robust BASE ACK pattern matched
  // (mfsk_matched >= ack_match_threshold), CONFIRM the climb even if the bsi-bearing SACK
  // suffix CRC FAILED — the base pattern is DSP-more-robust than the suffix and proves the
  // RX ACKed a forward batch at the announced config. Disarm post-state identical to
  // inband_retag_confirm_from_sack. No-op when feature-off / not armed / not yet announced /
  // sub-threshold / a DROP. Called at the CRC12-fail branch (arq_commander.cc:~3669).
  bool inband_retag_confirm_from_base_pattern(int mfsk_matched, int ack_match_threshold);
  // D4 escalation: called after an armed re-tag emit. If the re-tag has reached the R
  // floor with NO confirm AND the announcement was a CLIMB-UP (the down-ladder cannot
  // rescue a lost climb), AUTO-DEMOTE to inband_last_confirmed_config via the chokepoint
  // tag (inband_route_failure_demote) — NEVER a BREAK. A DROP that is un-confirmed is
  // rescued by the RX down-ladder + the continuing re-tag, so it does not escalate here.
  // Returns true if it routed an auto-demote. COMMANDER-only (uses the demote helper).
  bool inband_retag_escalate_if_climb_exhausted();

  // ── STAGE 4e — D2 NACK first-class (inband-reliability-design.md §2,
  //    data-flow-perbatch-config.md §S4E) ──
  // A new reverse-direction ctrl frame (MFSK_CTRL_NACK=5) lets the RX FAST-signal
  // "I could not follow / could not decode" instead of staying silent until the
  // sender's R-retry timeout. It rides the SAME RM(1,4)+gf16ra+CRC-12 substrate as the
  // CONFIG_TAG (the RM prefix carries rx_cfg_index, the gf16ra/CRC-12 message carries
  // the 37-bit NACK payload), keyed via generate_config_tag_pattern_passband. The
  // sender, on a valid NACK whose rx_cfg is BELOW the announced config, AUTO-DEMOTES
  // IMMEDIATELY to the RX config (reusing inband_route_failure_demote, BREAK-count==0)
  // — accelerating the Stage-4d R-retry give-up to a single batch. D2 is an
  // OPTIMIZATION: with NO NACK the Stage-4d R-retry auto-demote still fires.
  // All members ADDITIVE, gated by MERCURY_INBAND_RATE (default-off byte-identical).

  // The epoch parity the RX last SAW on an adopted/followed tag — echoed in the NACK so
  // the sender can reject a stale NACK that crosses a fresh change. Init 0.
  uint8_t inband_rx_seen_parity = 0;
  // Throttle: one DECODE_FAIL NACK per dead-batch streak segment. Set on the first
  // DECODE_FAIL emit; cleared on any down-ladder resync / data delivery (so a NEW
  // dead-streak re-emits one NACK). Init false.
  bool inband_nack_emitted_for_dead_streak = false;

  // Build the NACK suffix tones (RM(1,4) Walsh prefix carrying rx_cfg_index +
  // gf16ra RA + CRC-12 message carrying the NACK payload), type=MFSK_CTRL_NACK.
  // Mirrors build_config_tag_tones. Returns true on success; false if rx_cfg off the
  // ladder or M<16. out_tones holds RM16+gf16ra39 tones; *out_n the total count.
  bool build_nack_tones(int rx_cfg, uint8_t rx_expected_bsi_lsb, uint8_t reason,
                        uint8_t epoch_parity, int* out_tones, int* out_n);

  // RX EMIT (producer): key a NACK onto the reverse robust passband layer (the SAME
  // keyer the CONFIG_TAG uses). `reason` is mfsk_ctrl_nack_reason. The payload is
  // computed from current_configuration (-> ladder index), rsp_current_expected_batch_
  // seq_id, the reason, and inband_rx_seen_parity. No-op (returns 0) when the feature is
  // off / M<16 / current off-ladder. Returns the passband samples emitted (0 if none).
  // Called ONLY on a genuine cannot-follow (INV-E1): a down-ladder total-loss
  // (DECODE_FAIL) or an un-adoptable tag (UNFOLLOWABLE_CLIMB).
  int inband_emit_nack(uint8_t reason);

  // SENDER decode (consumer): pull the reverse ctrl tail (SAME window math as the MFSK
  // ACK/SACK decode), run decode_config_tag_from_passband + nack_wrap_decode
  // (type=MFSK_CTRL_NACK). On a CRC-valid NACK writes *out_rx_cfg_index / *out_reason /
  // *out_parity / *out_bsi_lsb and returns 1; 0 otherwise. No-op when the feature is off.
  // Called from the commander reverse-frame poll ONLY AFTER the ACK/SACK decode misses
  // (INV-E3 mutual exclusion).
  int inband_decode_nack_from_capture(uint8_t* out_rx_cfg_index, uint8_t* out_reason,
                                      uint8_t* out_bsi_lsb, uint8_t* out_parity);

  // SENDER handle (consumer): apply the NACK policy. If a re-tag is armed and the RX's
  // raw config (from rx_cfg_index) is BELOW the announced inband_retag_config by ladder
  // index (could not follow a climb), OR reason==UNFOLLOWABLE_CLIMB, AUTO-DEMOTE to the
  // RX config via inband_route_failure_demote (BREAK-count==0) IMMEDIATELY (does NOT
  // wait for inband_retag_count>=R) and set inband_last_confirmed_config = the RX config.
  // Rejects a stale NACK whose echoed parity != inband_tx_epoch_parity (INV-E3). Returns
  // true if it routed the accelerated demote. COMMANDER-only (uses the demote helper).
  bool inband_handle_nack(uint8_t rx_cfg_index, uint8_t reason, uint8_t epoch_parity);

  // ── STAGE 4e — D3 periodic re-announce backstop (inband-reliability-design.md §3,
  //    OD-3) ── Re-emit the CURRENT-config tag every N DATA batches independent of
  // change, HOLDING the epoch parity (NOT a change), so a desynced/late-joining peer
  // re-syncs without waiting for the next config change. The counter increments per
  // DATA-batch firing decision and resets to 0 on ANY emit (change/repeat/periodic) so
  // the periodic clause never double-emits. N is MERCURY_INBAND_REANNOUNCE_N (default 8,
  // 0=disabled). Init 0 (ctor + reset_session_state).
  int  inband_batches_since_announce = 0;
  int  inband_reannounce_n_cached    = -1;   // cached MERCURY_INBAND_REANNOUNCE_N (-1=unresolved)
  // Resolve+cache N (>=0; 0=disabled). MERCURY_INBAND_REANNOUNCE_N, default 8.
  int  inband_reannounce_n();

  // ── STAGE 4 — the bounded down-ladder lost-tag resync state (design §4/§7) ──
  // The down-window depth D is owner-tunable via MERCURY_INBAND_DOWN_D (default 4,
  // per the codeword study, §4.2); SESSION_DEAD_BATCHES via
  // MERCURY_INBAND_DEAD_BATCHES (default 3). Both resolved once + cached. The scoped
  // decoder bank holds AT MOST INBAND_DOWN_D_MAX+1 decoders — never the full
  // NUMBER_OF_CONFIGS monitor bank (the RPi bound, INV-S4-2). Each slot is sized to
  // its OWN config, lazily (re)built keyed by config id, cached across batches.
  static const int INBAND_DOWN_D_MAX = 8;     // hard cap on D (window <= D+1 wide)
  cl_telecom_system* inband_down_decoders[INBAND_DOWN_D_MAX + 1] = {};  // scoped bank
  int  inband_down_decoder_cfg[INBAND_DOWN_D_MAX + 1] = {};             // config id per slot (-1=empty)
  bool inband_down_decoders_built = false;    // any slot allocated yet?
  int  inband_down_buffer_nsymb = 0;          // common buffer_Nsymb for the bank (largest window cfg)
  int  inband_down_d = -1;                    // cached MERCURY_INBAND_DOWN_D (-1=unresolved)
  int  inband_session_dead_batches = 0;       // consecutive ZERO-PROGRESS REAL-batch-period total losses -> terminal BREAK
  int  inband_dead_batches_limit = -1;        // cached MERCURY_INBAND_DEAD_BATCHES (-1=unresolved)
  // §19 dead-batch tick guard: a session-monotonic forward-DATA-frame counter + the snapshot at the
  // last tick + a TIME rate-limit, so the tick counts CONSECUTIVE ZERO-PROGRESS REAL BATCH PERIODS
  // (not sub-second partial-SACK-turnaround RX-loop passes). A real total loss decodes NO frame, so
  // the bsi (rsp_current_expected_batch_seq_id) does NOT advance — the rate-limit MUST be TIME-based
  // (a cl_timer that advances regardless of decode) or it would suppress the LEGITIMATE total-loss
  // BREAK. data-flow-robust-ofdm-adopt-flush.md §19.
  long      inband_total_data_frames_rx = 0;  // monotonic per session; ++ on every forward DATA frame decoded
  long      inband_dead_tick_frames_snap = 0; // inband_total_data_frames_rx at the last dead-batch tick
  bool      inband_dead_tick_timer_armed = false; // false until the first tick arms inband_dead_tick_timer
  cl_timer  inband_dead_tick_timer;           // VIRTUAL-time since the last tick (real-batch-period rate-limit)
  bool inband_terminal_break_due = false;     // set when the dead-batch streak hit the limit (caller fires BREAK)
  int  inband_test_forced_down_delay = -1;    // TEST-ONLY: forced preamble delay for scoped decoders (-1=real acquisition)
  // TEST-ONLY: when true, emit_config_tag_passband runs its firing-decision state
  // machine (parity/latch/R-counter advance as if announced) but DOES NOT key the
  // announce burst onto the wire (the tx_transfer is skipped). This forces the RX to
  // never hear the CONFIG_TAG, so its current_configuration LAGS the TX and the
  // production down-ladder must resync from a primary-derived capture snapshot — the
  // exact HW 0-byte path (data-flow-inband-ondemote-zerobyte.md §6). Default false →
  // production keys the burst normally (byte-identical when off).
  bool inband_test_suppress_announce_tx = false;
  // RX receive-loop entry: when receive() returned with NO decoded data frame at the
  // current OFDM config (a possible lost tag), run the bounded down-ladder. On a
  // resync it adopts the true config (BREAK avoided); on a total-loss batch it
  // advances the dead-batch streak and, at SESSION_DEAD_BATCHES, sets
  // inband_terminal_break_due so the caller's existing BREAK machinery fires. The
  // ONLY remaining BREAK trigger on the inband path. No-op when the flag is off.
  void inband_try_down_ladder_on_decode_fail();
  // FIX #3 (data-flow-inband-downladder.md §3/§5.3): flush a COMPLETE in-flight prev batch
  // to the app BEFORE a BREAK -> ROBUST_0 reseed reshrink can orphan its RECEIVED frames.
  // Feature-gated (inband only); returns 1 if a prev batch was delivered, else 0.
  int  deliver_complete_inflight_before_break();
  int  inband_down_decode_attempts = 0;       // diagnostic: decode attempts in the LAST ladder run
  int  inband_down_decoded_buf[N_MAX / 8] = {};  // staging for the winning decoder's bytes
  // Resolve+cache the down-window depth D (clamped to [1,INBAND_DOWN_D_MAX]).
  int  inband_down_window_depth();
  // Resolve+cache SESSION_DEAD_BATCHES (>=1).
  int  inband_session_dead_limit();
  // Tear down the scoped down-window bank (NB/WB switch or session reset).
  void inband_free_down_decoders();
  // RANK-1 FIX (data-flow-inband-ondemote-zerobyte.md §2.4/§7): the buffer_Nsymb the
  // down-window decoder BANK uses for the CURRENT RX config = the largest (most-robust)
  // config in the window [cur-D..cur]. The captured snapshot is sized to THIS (not the
  // primary) so a robust-rung frame is not truncated. Returns 0 if cur is off-ladder.
  int  inband_down_window_buffer_nsymb();
  // RANK-1 FIX: the buffer_Nsymb of the deepest reachable rung (ROBUST_0). The primary
  // capture ring is seated to at least this (in symbols) so a full robust frame fits.
  int  inband_robust_floor_buffer_nsymb();
  // FIX #1c (data-flow-robust-ofdm-adopt-flush.md §10): the NATURAL buffer_Nsymb of an OFDM
  // config (no raised robust floor). The robust->OFDM adopt shrinks the oversized robust-floor
  // ring back to this so the re-aired OFDM burst lands within the coarse-search bounds.
  int  inband_natural_ofdm_buffer_nsymb(int ofdm_cfg);
  // RANK-1 FIX: seat the primary capture ring's buffer_Nsymb_min to the ROBUST floor
  // (re-applying the PHY config if the ring is currently smaller) so the down-ladder can
  // read a full robust frame. Idempotent; no-op when off / defeat set / already seated.
  void inband_seat_robust_ring_floor();
  // FAIL-BEFORE / A-B knob: MERCURY_INBAND_DOWN_DEFEAT_SNAPFIX (cached). 1 = pre-fix
  // primary-config snapshot sizing + no ring seat (reproduces the 0-byte truncation).
  bool inband_down_defeat_snapfix();
  int  inband_down_defeat_snapfix_cached = -1;   // -1=unresolved, 0=off (fixed), 1=defeat

  // FAIL-BEFORE / A-B knob (data-flow-inband-downladder.md §2.1): MERCURY_INBAND_FRESHWIN_DEFEAT
  // (cached). 1 = the down-ladder firing gate IGNORES rx_fresh_window_decoded_this_pass, i.e.
  // the PRE-FIX behavior that fired on EVERY stale inter-frame pass during an active batch (the
  // 3127 "all silent" HW firings). Default 0 = the fresh-window gate is active. Production never
  // sets it; the regression flips it to reproduce the firing-on-silence then confirm the gate.
  bool inband_freshwin_gate_defeat();
  int  inband_freshwin_gate_defeat_cached = -1;  // -1=unresolved, 0=off (gated), 1=defeat

  // RESIDUAL FIX (data-flow-inband-tier-crossing.md §11 — the §6 false-confirm retirement
  // surfaced this): the ROBUST-TIER tag-follow (arq_responder.cc ~:669) was GATED on
  // rx_fresh_window_decoded_this_pass, a term copied from the OFDM blind down-ladder. But
  // inband_detect_follow_from_capture re-snapshots the ring tail under its OWN mutex and
  // self-gates on a REAL base-correlator presence detect (returns 0 when no burst present),
  // and detect_and_follow_config_tag no-ops when decoded cfg == current. So the fresh-window
  // term is unnecessary HERE and DROPS the re-emitted CONFIG_TAG on every stale inter-frame
  // pass (frames_to_read!=0) -> the climb tier-cross becomes NON-DETERMINISTIC (catches the
  // tag only when a fresh-window pass coincides). Default required==false (fix: check every
  // in-flight robust pass). MERCURY_ROBUST_FOLLOW_FRESHWIN_REQ=1 restores the PRE-FIX gate
  // (fail-before: the non-deterministic miss). Cached on first call.
  bool inband_robust_follow_freshwin_required();
  int  inband_robust_follow_freshwin_required_cached = -1;  // -1=unresolved, 0=fix(not req), 1=pre-fix(req)

  // RESIDUAL FIX: the ROBUST-TIER tag-follow firing predicate (arq_responder.cc ~:669),
  // factored out so test_inband_robust_follow_freshwin drives the EXACT production gate.
  // `fresh` is the caller's rx_fresh_window_decoded_this_pass. Returns true when the block
  // should run inband_detect_follow_from_capture. Reads link_status/connection_status/
  // passive_monitor/messages_rx_buffer.status/current_configuration/rsp_current_expected_
  // batch_seq_id from members. Default: fresh-window NOT required (the fix).
  bool inband_robust_follow_gate_open(bool fresh);

  // DIRECTED fail-before/pass-after regression for the residual fix (returns 0 on pass).
  int  test_inband_robust_follow_freshwin();

  // FIX #1d FAIL-BEFORE / A-B knob (data-flow-robust-ofdm-adopt-flush.md §11):
  // MERCURY_ADOPT_RING_DURABILITY_DEFEAT (cached). 1 = the down-ladder does NOT skip a robust
  // trial while holding a fresh OFDM lock (the PRE-FIX behavior: a transient robust glimpse
  // decodes + adopts -> the primary reloads ROBUST_0 -> the shrink flag clears -> the ring
  // re-grows -> the live OFDM lock collapses). Default 0 = the guard is active (the fresh OFDM
  // lock is durable through a transient robust probe). Production never sets it; the regression
  // flips it to reproduce the lock-collapse then confirm the guard.
  bool inband_adopt_ring_durability_defeat();
  int  inband_adopt_ring_durability_defeat_cached = -1;  // -1=unresolved, 0=off (guarded), 1=defeat

  // IN-BAND ADOPT CLEAN-LOCK METRIC GATE (data-flow-inband-adopt-metric-gate.md §2/§3).
  // Gate the in-band tag-follow adopt on the NORMALIZED Schmidl-Cox metric over the SAME
  // captured snapshot the OFDM acquisition is about to consume: a CRC-valid CONFIG_TAG is
  // adopted ONLY when the OFDM lock is CLEAN (metric >= threshold). A contaminated / overlapping
  // re-air window (metric ~0.5) is REJECTED → no follow → the RX retries on the next pass (the
  // window refills toward a clean single-burst snapshot). Returns true = ADOPT may proceed,
  // false = REJECT (contaminated; retry). Feature-gated (legacy byte-identical). `announced_cfg`
  // is the tag-announced config (the metric is judged at the CURRENT loaded geometry — the
  // re-aired base-rung burst the gate must judge — so announced_cfg is advisory/logging).
  bool inband_adopt_metric_gate_ok(const double* snapshot, int len, int announced_cfg);
  // The snapshot context the metric gate judges, set by the snapshot/capture adopt callers
  // (inband_detect_follow_from_snapshot / inband_detect_follow_from_capture) immediately
  // before they invoke detect_and_follow_config_tag, and cleared after. detect_and_follow_
  // config_tag reads these at the single adopt-commit point so BOTH adopt routes share ONE
  // gate (no divergence). NULL/0 = no snapshot context available → the gate passes through
  // (the down-ladder adopt, which already self-gates on a CRC/LDPC decode, INV-C).
  const double* inband_adopt_gate_snapshot     = NULL;
  int           inband_adopt_gate_snapshot_len = 0;
  // FAIL-BEFORE / A-B knob: MERCURY_INBAND_ADOPT_METRIC_GATE_DEFEAT=1 (cached) SKIPS the gate
  // (the PRE-FIX behavior: a contaminated 0.5 lock is adopted → SKIP-VAR → 0 forward decode).
  // Default 0 = gate ACTIVE. Mirrors the MERCURY_ADOPT_*_DEFEAT pattern. -1 = unresolved.
  int  inband_adopt_metric_gate_defeat_cached = -1;
  // Threshold (cached): MERCURY_INBAND_ADOPT_METRIC_GATE, default 0.9 (the "prominent peak"
  // discriminant the codebase cites at arq_common.cc:12586). Sweep-only override; <0 = unresolved.
  double inband_adopt_metric_gate_threshold_cached = -1.0;

  // PER-PASS PHY-REBUILD LEAK FIX (data-flow-inband-downladder-delivery): the ROBUST-floor
  // buffer_Nsymb is a CONSTANT for a given bandwidth (the config is always FULL_CONFIG_LADDER[0]),
  // so probe the throwaway cl_telecom_system ONCE and memo it keyed by narrowband_enabled. Without
  // this, inband_seat_robust_ring_floor() (called every CONNECTED+RECEIVING pass) did a full
  // M=200 MFSK load_configuration on the hot RX path EVERY pass (HW: 2793x) -> the OFDM decode PHY
  // was starved -> ofdm_ok=0 -> 0 bytes delivered on the ON arm. -1 = unmemoized; >=0 = cached
  // floor Nsymb; cache_nb records the bandwidth the value was probed for (invalidated on NB/WB
  // switch). Same memo for inband_down_window_buffer_nsymb() keyed by (lo_idx, nb).
  int  inband_robust_floor_nsymb_cached = -1;    // -1=unmemoized, >=0=cached floor buffer_Nsymb
  int  inband_robust_floor_nsymb_cache_nb = -1;  // narrowband_enabled the cache was built for
  // ROBUST->OFDM CROSSING FIX (data-flow-robust-ofdm-adopt-flush.md §10): set TRUE when the
  // robust->OFDM in-band adopt shrinks the capture ring back to the OFDM config's NATURAL size
  // (un-seating the raised robust floor) so the re-aired OFDM burst lands within the coarse-search
  // bounds (the oversized robust-floor ring put every freshest preamble at the tail, beyond
  // upper_bound -> permanent `OFDM beyond-bounds`). While TRUE, inband_seat_robust_ring_floor()
  // MUST NOT re-grow the ring (it would re-introduce the oversize and re-block acquisition). Cleared
  // when the RX leaves the OFDM tier (demote to a robust config), so the next down-ladder re-seats
  // the floor BEFORE reading a robust frame. Off (legacy/!inband) -> always false -> byte-identical.
  bool inband_ofdm_acq_ring_shrunk = false;
  int  inband_down_window_nsymb_cached = -1;     // -1=unmemoized, >=0=cached window buffer_Nsymb
  int  inband_down_window_nsymb_cache_lo = -1;   // lo_idx the cache was built for
  int  inband_down_window_nsymb_cache_nb = -1;   // narrowband_enabled the cache was built for
  // SAME LEAK CLASS, instance #2: inband_ensure_down_decoders() probed the bank's common
  // buffer_Nsymb with a throwaway cl_telecom_system + load_configuration EVERY call. The bank
  // SLOTS were already reused, but the want_buffer_nsymb PROBE ran unconditionally — a full
  // CONFIG-11 OFDM PHY init per down-ladder fire (v6 cycle1 ON: 44x) on the hot capture thread,
  // the SAME starvation as the floor leak. The bank's common buffer depends ONLY on (lo_idx,
  // bandwidth), so memo it keyed by that pair and skip the tmp construction when unchanged.
  // -1 = unmemoized; >=0 = cached bank buffer_Nsymb. Invalidated on lo_idx / NB-WB change.
  int  inband_ensure_bank_nsymb_cached  = -1;    // -1=unmemoized, >=0=cached bank buffer_Nsymb
  int  inband_ensure_bank_nsymb_cache_lo = -1;   // capped lo_idx the cache was built for
  int  inband_ensure_bank_nsymb_cache_nb = -1;   // narrowband_enabled the cache was built for
  // TEST-ONLY diagnostic: counts throwaway cl_telecom_system PHY probes (load_configuration) the
  // nsymb helpers AND inband_ensure_down_decoders' bank-buffer probe perform. The regression in
  // test_inband_deliver asserts this stays flat (0) across a steady-receive seat / down-ladder
  // loop (pass-after) vs N (fail-before). Production never reads it.
  long inband_floor_probe_count = 0;

  // Stage 3b GEARSHIFT DRIVE (unilateral drop). When MERCURY_INBAND_RATE is set,
  // add_message_control(SET_CONFIG) takes the UNILATERAL path (inband_unilateral_config_change)
  // instead of queueing a SET_CONFIG control handshake: it loads the gearshift
  // target config directly (so the next send_batch transmits at the new rung and
  // the W1 emit announces it via the passband tag) and re-fills TX. Because EVERY
  // gearshift/optimizer/demote producer funnels through add_message_control(SET_CONFIG)
  // AND its callers force connection_status=TRANSMITTING_CONTROL after it returns,
  // the builder sets this one-shot flag; process_messages_tx_control() reads it at
  // the SINGLE control-TX entry, clears it, and re-routes connection_status back to
  // TRANSMITTING_DATA (neutralising the caller's forced control transition in one
  // place). Default-off: the flag is never set, so the SET_CONFIG handshake path is
  // byte-identical. Declared/init in arq_common.cc next to the other inband state.
  bool    inband_unilateral_armed;      // one-shot: builder took the unilateral path
  // Run the unilateral config drop on the CMD: load `target_cfg` directly
  // (PHYSICAL_LAYER_ONLY), advance forward_configuration/data_configuration, and
  // re-fill the TX messages for the new config's frame sizes (mirroring the
  // SET_CONFIG ACK-apply refill at arq_commander.cc:5291-5293). NO control frame is
  // queued. Returns true if the drop applied (target valid + a real change), false
  // if it was a no-op (same config / invalid target) so the builder can fall back.
  bool inband_unilateral_config_change(int target_cfg);

  // HYBRID TIER-CROSSING ROUTING (data-flow-inband-tier-crossing.md §2). PURE predicate:
  // does a config change from current_configuration to target_cfg CROSS the robust<->OFDM
  // tier boundary (is_robust_config differs)? A crossing must use the legacy SET_CONFIG
  // control handshake (fast dedicated ACK) instead of the in-band unilateral CONFIG_TAG
  // (which serializes each rung behind the slow data-SACK turnaround). CONFIG_NONE target
  // is never a crossing. No side effects — drives the chokepoint routing AND its directed
  // regression (test_inband_tier_crossing_routing) identically.
  bool inband_config_change_is_tier_crossing(int target_cfg);

  // Directed regression for the hybrid tier-crossing routing (fails-before/passes-after).
  int test_inband_tier_crossing_routing();

  // IN-BAND TIER-CROSSING REVERSE-ACK PIN (data-flow-inband-tier-crossing.md §3). PURE:
  // on an in-band robust<->OFDM tier-cross, return the ROBUST rung the reverse SACK must
  // ride so it decodes reliably across the cross (mirroring legacy's reverse-robust hold).
  // The robust side of the crossing is `from` on a robust->OFDM up-cross (the live robust
  // rung that just carried data) or `to` on an OFDM->robust down-cross (the target IS
  // robust). Returns CONFIG_NONE when it is NOT an in-band crossing (the caller then keeps
  // the legacy reverse seed). `inband_on` lets the directed test drive the feature gate
  // without touching the env. No side effects — drives the production pin AND its directed
  // regression (test_inband_tier_cross_reverse_pin) identically.
  int inband_tier_cross_reverse_config(int from_cfg, int to_cfg, bool inband_on) const;

  // IN-BAND +1 CLIMB TARGET (data-flow-inband-frame0-rolling-partial.md §7.2 option A) — the
  // pure FRAME-UP climb-target selector. inband_plus1_on=true -> strict +1 (proposed_frame,
  // suppress the SNR elevator so the reverse data-SACK decodes at the shared rung);
  // inband_plus1_on=false -> legacy elevator-OR-+1 max (byte-identical). snr_elevator<0 means
  // no elevator this poll. Drives the production decision AND test_inband_plus1_climb.
  int inband_climb_target(int proposed_frame, int snr_elevator, bool inband_plus1_on) const;

  // Directed regression for the tier-cross reverse-ACK pin (fails-before/passes-after).
  int test_inband_tier_cross_reverse_pin();

  // Directed regression for the in-band +1 climb (suppress the SNR-elevator jump).
  int test_inband_plus1_climb();

  // CLIMB-LATENCY regressions (gearshift-start-and-recovery.md §10.5).
  // test_connect_snr_seed: guarded SNR-seed of the start config — clean channel
  // seeds a WB start, marginal channel does NOT over-seed. Fails-before under
  // -DCONNECT_SEED_FAILBEFORE. test_robust_pipeline: the first ROBUST climb rung
  // pipelines (no clean-batch serialization). Fails-before under
  // -DINBAND_ROBUST_PIPELINE_FAILBEFORE.
  int test_connect_snr_seed();
  int test_robust_pipeline();

  // KEYSTONE (data-flow-inband-tier-crossing.md §6) — directed regression for the
  // data-decoupled intra-tier climb confirm (inband_retag_confirm_from_base_pattern).
  // Fails-before under -DINBAND_BASEPATTERN_CONFIRM_FAILBEFORE.
  int test_inband_basepattern_confirm();

  // IN-BAND TIER-CROSSING LIVENESS-GUARD EXEMPTION (data-flow-inband-tier-crossing.md §3
  // PART B). PURE: should the connect-liveness guard NOT accrue a stall this poll because a
  // deliberate robust<->OFDM tier-cross control handshake is in flight? THREE conjuncts: a
  // control phase AND is_tier_crossing(target_cfg) AND NO forward DATA batch in flight
  // (has_inflight_data==false — the discriminator that keeps a genuine post-data livelock,
  // which carries an in-flight batch, from being swallowed). Drives the production exemption
  // AND its directed regression identically. Non-const (calls the non-const tier-crossing
  // predicate which reads current_configuration).
  bool inband_tiercross_handshake_exempts_liveness(int conn_status, int target_cfg,
                                                   bool has_inflight_data);

  // ── STAGE 4c — D5: BREAK truly obsolete (inband-reliability-design.md §5,
  //    data-flow-perbatch-config.md §S4C) ──
  // When MERCURY_INBAND_RATE is ON, a COMMANDER Class-A degradation/failure that
  // today calls send_break_pattern() instead routes through a TAG-DEMOTE (one rung
  // down via the chokepoint; the RX down-ladder catches a missed tag), REUSING the
  // CFG16 D3 demote machinery (arq_commander.cc:3978-4117). Only a GENUINE total
  // loss — the link is already at the ladder bottom AND the commander dead-batch
  // streak hits SESSION_DEAD_BATCHES — is permitted to BREAK (the §7 floor). When
  // the feature is OFF every Class-A site fires send_break_pattern() byte-identically.
  //
  // inband_route_failure_demote: lift the D3 demote body into one reusable helper.
  // Preserves in-flight bytes (FIFO push-back / restore_tx_from_compressed), does
  // the lossless bsi rollback (cmd_batch_seq_id <- earliest in-flight bsi, no D3.1
  // GAP-ABORT), sets the config owners to demote_target so the chokepoint reads it
  // (negotiated_configuration), pins supershift_proven_ceiling, resets the nack/
  // starve streaks, runs the optimizer supremacy hook, then add_message_control(
  // SET_CONFIG) (which under inband becomes inband_unilateral_config_change + the tag) and
  // sets connection_status=TRANSMITTING_CONTROL. Returns true if it routed the
  // demote (caller must `return` — the demote owns the next transition). MUST be
  // called only with a valid lower rung (caller checks !config_is_at_bottom).
  bool inband_route_failure_demote(int demote_target, const char* reason);
  // roll_back_cmd_bsi_to_inflight: the CLIMB-UP counterpart of the demote bsi rollback
  // (data-flow-inband-frame0-rolling-partial.md §13). The demote/BREAK paths
  // (inband_route_failure_demote :3134, CFG16-HOLD :5037, M6 BREAK :5242) roll
  // cmd_batch_seq_id back to the earliest in-flight batch_seq_id before freeing
  // messages_tx[]; the CLIMB-UP SET_CONFIG emits (FRAME-UP gearshift, optimizer, turbo
  // settle) did NOT — a SYMMETRY GAP. On a rapid mid-transfer climb the in-flight
  // (already-RSP-delivered, not-yet-CMD-ACKed) batch is re-presented under whatever
  // ADVANCED epoch the climb reached, so sack_v2_readopt_has_gap()/delivery_step_is_gap()
  // see a >=2 jump from the RSP's preserved delivery high-water -> [RSP-V2-GAP-ABORT].
  // This helper scans messages_tx[] for the EARLIEST (mod-256) in-flight (non-FREE,
  // length>0) batch_seq_id and rolls cmd_batch_seq_id back to it so the climb-UP
  // re-present is CONTIGUOUS. Gated IDENTICALLY to the demote rollback
  // (sack_v2_enabled && !compression_enabled — the compression path's
  // restore_tx_from_compressed() owns its own re-stage; v1 never reads the v2 gap-gate).
  // MUST be called BEFORE the caller frees messages_tx[]. Returns the rolled-to bsi, or
  // -1 (no-op: gated off / nothing in flight). DISTINCT from the FIX-1 hole-defer
  // (inband_climb_hole_outstanding gates retransmit_count>0 retx holes; this gates the
  // in-flight epoch LABEL — the two are orthogonal).
  int roll_back_cmd_bsi_to_inflight(const char* tag);
  // Commander-side true-session-loss floor (the ONLY commander BREAK permitted under
  // inband). Counts consecutive Class-A total-loss batches that occur WHILE already
  // at the ladder bottom (nowhere left to demote). Reset to 0 on any data-ACK
  // success. Keyed to the SAME MERCURY_INBAND_DEAD_BATCHES as the RX terminal floor
  // (inband_session_dead_limit()), so both sides BREAK together. Init 0 (ctor +
  // reset_session_state). Returns true when the floor has been reached and the
  // genuine send_break_pattern() should fire.
  int  cmd_inband_session_dead_batches = 0;
  bool inband_cmd_dead_batch_floor_reached();
  // FORWARD-HEALTHY discriminator (data-flow-inband-retx-epoch.md §5). True iff the
  // commander still holds an in-flight forward DATA batch in messages_tx[] (any non-FREE,
  // length>0 slot). The CMD-side proxy for "still delivering forward / not genuinely
  // dead": a forward-healthy reverse-ACK turnaround MISS leaves the just-aired batch in
  // messages_tx[] awaiting its missed reverse-ACK; a genuine connect/negotiate livelock
  // holds NO in-flight DATA batch. Used by inband_connect_liveness_guard() to route a
  // forward-healthy miss to the NO-BREAK re-present instead of the BREAK->ROBUST cascade.
  bool cmd_has_inflight_data_batch() const;
  // ---- In-band CONNECT-LIVENESS GUARD (data-flow-inband-connect-liveness.md) ----
  // The retained true-loss BREAK is wired only to a DATA-loss tick
  // (inband_cmd_dead_batch_floor_reached). A connect/negotiate handshake that stalls
  // with ZERO forward DATA progress (the HW livelock: link_status==CONNECTED, ~92% of
  // polls in TRANSMITTING_CONTROL, nAcked_data flat at 0) never ticks that floor, and
  // link_timer is kicked by every control-ACK so the 10s session drop never fires.
  // The guard is the BACKSTOP: when stats.nAcked_data makes NO advance across N
  // consecutive polls WHILE NOT in a data-bearing phase, it fires the SAME §7 true-loss
  // send_break_pattern() recovery (BREAK->ROBUST_0 resync, exactly how legacy recovers).
  // cmd_inband_liveness_last_acked: snapshot of stats.nAcked_data at the last advance.
  // cmd_inband_liveness_no_progress_polls: consecutive no-data-progress control polls.
  // cmd_inband_liveness_breaks: liveness-BREAKs fired this session (bounded). All three
  // init 0 (ctor + reset_session_state) and reset on any data delivery.
  int  cmd_inband_liveness_last_acked = 0;
  int  cmd_inband_liveness_no_progress_polls = 0;
  int  cmd_inband_liveness_breaks = 0;
  int  inband_liveness_stall_polls = -1;   // unresolved; cached from env on first use
  // liveness-BREAKs per session before a hard session reset (shared by the guard +
  // its regression test in arq_responder.cc).
  #define INBAND_LIVENESS_MAX_BREAKS 3
  int  inband_liveness_stall_polls_count();   // MERCURY_INBAND_LIVENESS_POLLS, default 200
  bool inband_connect_liveness_guard();       // once-per-poll commander watchdog (gated ON);
                                              // returns true if it fired a recovery (caller returns)
  int  test_inband_liveness();                // --test-inband-liveness regression
  // Diagnostic / test instrument: total send_break_pattern() invocations on this
  // controller (incremented at the top of send_break_pattern). The
  // --test-inband-no-break harness reads it to assert BREAK-count==0 on a degradation
  // and ==1 at the true-loss floor. Pure observation; no behavior depends on it.
  long send_break_pattern_count = 0;

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
    // KX-as-data bypass: the reserved pre-activation KX stream carries
    // ciphertext-random public key material — compressing it wins nothing and
    // would desync the streaming compressor context that must stay pristine for
    // the first user byte. Both peers compute kx_stream_epoch() deterministically
    // from the shared pre-activation state, so they bypass on the SAME batches
    // (no headerless-vs-header mis-parse). Inert (false-returning branch never
    // taken) on every non-encrypted session.
    if(kx_stream_epoch()) return false;
    if(!compression_enabled) return false;
    int eff_long = effective_data_long_header_length(sack_v2_enabled, header_carries_d5);
    int max_frame = max_data_length + max_header_length - eff_long;
    int batch_capacity = data_batch_size * max_frame;
    if(cipher_suite.is_active()) batch_capacity -= AUTH_TAG_SIZE;
    return batch_capacity > compressor.get_header_size();
  }

  // Encryption (hybrid PQ: X25519 + ML-KEM-768 + ChaCha20-Poly1305)
  cl_cipher_suite cipher_suite;       // Per-connection cipher state (ephemeral keys, session key)
  int encryption_mode;                // ENCRYPT_OFF, ENCRYPT_STRICT, ENCRYPT_FAST
  bool encryption_enabled;            // Negotiated: both sides have CAP_ENCRYPTION and mode != OFF
  // Encryption-negotiation policy (single source of truth for the commander AND
  // responder). FAIL-CLOSED: an -E opt-in against a peer that does not advertise
  // CAP_ENCRYPTION (unsupported, or a MITM stripped the bit) REFUSES — it never
  // downgrades to plaintext. DEFAULT-OFF is preserved: with encryption_mode ==
  // ENCRYPT_OFF the outcome is PLAINTEXT_OK, so an operator who did not opt in is
  // unaffected. Defined in arq_common.cc.
  enc_negotiation_outcome_t decide_encryption_negotiation(int encryption_mode,
                                                          uint8_t local_capability,
                                                          uint8_t peer_capability);
  // In-process regression for the fail-closed policy (runs in --test). Returns
  // 0=PASS, else the fail count. Fails-before: -DENC_FAILOPEN_FAILBEFORE (which
  // compiles the historical opportunistic-plaintext FAST downgrade back in).
  int test_encryption_fail_closed();
  // FORGIVING-ACK Tier 2 (fact-documents/data-flow-forgiving-ack.md §T2.1):
  // negotiated session flag — both ends advertised CAP_CUMULATIVE_ACK (which is itself
  // gated by the env opt-in MERCURY_CUMULATIVE_ACK on the local advertise). When true,
  // the RSP writes n_r (the contiguous delivery high-water) into the SACK bsi field and
  // the CMD interprets a received bsi as "everything <= n_r is acknowledged" (bounded
  // backward window). Default-off ≡ byte-identical + interop-safe (any non-Tier-2 peer
  // leaves the bit clear → both_support false → per-batch fallback). Computed once at
  // the TEST_CONNECTION / TEST_CONNECTION_ACK negotiation, cleared on session reset.
  bool cumulative_ack_enabled;        // Negotiated: both sides have CAP_CUMULATIVE_ACK
  // AEAD nonce sequence state (data-flow-aead-nonce.md). The nonce binds to the
  // UNWRAPPED WIRE batch_seq_id (epoch<<8 | wire_bsi), NOT a local encrypt/
  // delivery-order counter, so it survives SACK reorder/retx (both peers derive
  // the same nonce from the same wire bsi). These hold the per-direction
  // epoch/last-seen-bsi used by cl_cipher_suite::unwrap_batch_index(). Reset to
  // (0, -1) at activate() and on session reset.
  uint64_t tx_nonce_epoch;            // TX-direction wrap count for encrypt nonce
  int      tx_nonce_last_bsi;         // TX-direction last wire bsi (-1 = unset)
  uint64_t rx_nonce_epoch;            // RX-direction wrap count for decrypt nonce
  int      rx_nonce_last_bsi;         // RX-direction last wire bsi (-1 = unset)
  // --- Re-seal nonce-reuse guarantee (data-flow-aead-nonce.md §11) ----------
  // The OLD bsi-bound nonce was UNSAFE on the compression+encryption recovery
  // paths (BREAK-rebuild / config-rebuild / demote): a recovery FREEs the built-
  // but-unsent batch and re-queues PLAINTEXT *without* advancing cmd_batch_seq_id
  // (the +1 only runs after a COMPLETED send_batch, a later tick), so the next
  // build RE-SEALS the SAME wire bsi over RE-COMPRESSED (different) plaintext ->
  // unwrap returns the SAME index -> SAME (key,nonce), DIFFERENT plaintext ->
  // KEYSTREAM REUSE (catastrophic). The wire bsi MUST roll back to the in-flight
  // value for RSP delivery contiguity (delivery_step_is_gap), so the nonce index
  // can NOT be a pure function of the wire bsi. Fix = a per-direction GENERATION
  // counter, orthogonal to unwrap's wrap-epoch, folded into the nonce index high
  // bits: TX bumps tx_nonce_gen on every recovery that re-queues plaintext for a
  // re-seal; RX bumps rx_nonce_gen on the matching config-transition re-adopt it
  // processes BEFORE decrypting the re-sent batch (the two peers stay aligned
  // through the ONE wire-observable transition that separates seal from re-seal).
  // SAFETY DOES NOT DEPEND ON THAT ALIGNMENT: tx_nonce_sealed_high_water records
  // the highest index ever sealed; the encrypt site refuses to seal at or below
  // it, bumping the gen until the index clears the high-water. So a nonce is
  // NEVER reused even if the gen counters ever drift — a drift only makes RX
  // reconstruct a wrong index, which AUTH-FAILS (a safe drop/retx), never a
  // reuse. RFC-4303-ESN / RFC-9001-§5.4 regime: implicit high-order sequence
  // bits the receiver reconstructs; a wrong guess fails the tag, never weakens it.
  uint64_t tx_nonce_gen;              // TX re-seal generation (bumped per recovery)
  uint64_t rx_nonce_gen;             // RX re-seal generation (bumped per transition)
  uint64_t tx_nonce_sealed_high_water;// highest unwrapped+gen index ever sealed (TX);
                                      // UINT64_MAX = unset (nothing sealed yet)
  bool     rx_nonce_adopted_once;    // RX has adopted an expected-bsi at least once
                                      // this session — so the NEXT adopt-from -1 is a
                                      // genuine config-transition RE-adopt (bump
                                      // rx_nonce_gen) rather than the first-ever adopt
                                      // (gen 0). 1:1 with each TX recovery re-queue.
  int      decrypt_delivered_bsi;     // wire bsi of the batch being delivered in
                                      // copy_data_to_buffer() (set immediately
                                      // before each call per the §5 source table;
                                      // -1 = unknown -> decrypt is skipped/safe)
  int consecutive_auth_failures;      // Auth failures since last success (3 → disconnect)
  uint8_t* kx_data_buf;              // Buffer for ML-KEM key exchange data (1184 or 1088 bytes)
  int kx_data_len;                    // Length of pending key exchange data
  // --- Hybrid ML-KEM KX chunk transport (MLKEM_HYBRID_PLAN.md §4-§5,
  //     data-flow-hybrid-kex.md) ----------------------------------------------
  // The 1184B encaps key (KX2, CMD->RSP) and 1088B ciphertext (KX3, RSP->CMD)
  // ride a sequence of KEY_EXCHANGE_2/3 control frames over the OFDM data
  // configuration (each frame = one 4-byte-headed chunk; the existing per-frame
  // control ACK/retransmit covers loss). State below drives the chunked send and
  // the receive-side reassembly. All cleared on session reset / KX completion.
  // The reassembly target is kx_data_buf (MLKEM_PK_SIZE bytes, lazily alloc'd).
  uint8_t kx_mlkem_pk[1184];          // CMD: generated encaps key (also bind input)
  uint8_t kx_mlkem_ct[1088];          // RSP: encapsulated ciphertext (also bind input)
  bool kx_mlkem_pk_ready;             // CMD generated / RSP reassembled the pk
  bool kx_mlkem_ct_ready;             // RSP encapsulated / CMD reassembled the ct
  int  kx_tx_kind;                    // KX kind currently being SENT (0x3F/0x40/0=idle)
  int  kx_tx_total;                   // total bytes of the buffer being sent
  int  kx_tx_count;                   // chunk count for the send
  int  kx_tx_next_index;             // next chunk index to transmit
  int  kx_tx_chunk_cap;              // per-frame chunk payload capacity (sender)
  const uint8_t* kx_tx_src;          // points at kx_mlkem_pk or kx_mlkem_ct
  int  kx_rx_kind;                   // KX kind currently being REASSEMBLED (0=idle)
  int  kx_rx_count;                  // expected chunk count (from first chunk)
  int  kx_rx_chunk_cap;             // per-frame chunk payload capacity (receiver)
  int  kx_rx_total;                 // expected total bytes (MLKEM_PK_SIZE/CT_SIZE)
  int  kx_rx_received;             // distinct chunks received so far
  bool kx_rx_got[256];            // per-index arrival bitmap

  // ---- KX-as-data reserved pre-activation stream (data-flow-hybrid-kex.md) ----
  // The classical X25519 pubkey + the large ML-KEM artifact ride the DATA engine
  // as a reserved, self-identifying stream (decoded frames + real SACK sequence +
  // in-order + selective retransmit), consumed by the crypto state machine and
  // NEVER delivered to the app. Dissolves the control-plane pattern-ACK wall (the
  // per-chunk control ACK carries no decoded index, so a lost chunk cannot be
  // selectively re-requested). On the wire the transfer's first bytes are:
  //   [0..3] KX_STREAM_MAGIC (BE)  [4] kind (KX_STREAM_FWD/REV)  [5..6] total_len (LE u16)
  // total_len counts the PAYLOAD bytes that follow the 7-byte header. The stream is
  // interpreted as KX iff the receiver is PRE-ACTIVATION (encryption_enabled &&
  // !cipher_suite.is_active()), where user data is held, so routing is unambiguous;
  // the magic is the belt-and-suspenders wire distinguisher cross-checked for
  // fail-secure. KX bytes are public key material -> plaintext (encryption not yet
  // active), ciphertext-random -> compression bypassed, and carry NO Option-W wire
  // stamp (so the positional backstop is a safe no-op) so KEY_ACTIVATE can re-anchor
  // the delivery/bsi/nonce state to a fresh bsi=0 for the first user batch.
  static const uint32_t KX_STREAM_MAGIC = 0x4B585331u; // "KXS1"
  static const uint8_t  KX_STREAM_FWD   = 0x01;        // CMD->RSP: x25519_pk_cmd(32)+mlkem_pk(1184)
  static const uint8_t  KX_STREAM_REV   = 0x02;        // RSP->CMD: x25519_pk_rsp(32)+mlkem_ct(1088)
  static const int      KX_STREAM_HDR   = 7;           // magic(4)+kind(1)+total_len(2)
  static const int      KX_STREAM_PAYLOAD_MAX = X25519_KEY_SIZE + MLKEM_PK_SIZE; // 32+1184=1216
  static const int      KX_STREAM_BUFSZ = KX_STREAM_HDR + KX_STREAM_PAYLOAD_MAX; // 1223
  static const int      KX_STREAM_FWD_LEN = X25519_KEY_SIZE + MLKEM_PK_SIZE;     // 1216 payload
  static const int      KX_STREAM_REV_LEN = X25519_KEY_SIZE + MLKEM_CT_SIZE;     // 1120 payload
  // TX staging (dedicated buffer, NOT fifo_buffer_tx — user data stays untouched)
  uint8_t kx_stream_tx_buf[KX_STREAM_BUFSZ];
  int     kx_stream_tx_len;      // bytes staged (header + payload), 0 = none
  int     kx_stream_tx_sent;     // bytes already fed into the data engine
  bool    kx_stream_tx_active;   // this session is streaming KX-as-data (feed past the hold)
  // RX reassembly of the reserved KX stream
  uint8_t kx_stream_rx_buf[KX_STREAM_BUFSZ];
  int     kx_stream_rx_have;     // bytes accumulated so far (incl the 7-byte header)
  int     kx_stream_rx_expected; // KX_STREAM_HDR + total_len (known once the header lands)
  uint8_t kx_stream_rx_kind;     // 0 / KX_STREAM_FWD / KX_STREAM_REV currently reassembling
  bool    kx_stream_rx_is_kx;    // the current RX transfer is the reserved KX stream
  uint8_t kx_stream_peer_x25519[X25519_KEY_SIZE]; // folded classical pubkey from the last completed KX stream

  // RX ingest of the reserved KX stream: parse+validate the header on the first
  // bytes, accumulate payload, and on completion hand the pk/ct to the crypto
  // buffers (kx_mlkem_pk/kx_mlkem_ct + the folded x25519 pubkey). NEVER routes to
  // fifo_buffer_rx (the app socket). Returns the number of bytes CONSUMED as KX
  // (== len on the happy path), or -1 on a fail-secure REFUSE (bad magic/kind/len /
  // marker-present-while-active / overflow) — the caller must tear the link down,
  // never fall through to plaintext. On completion sets kx_stream_rx_done_kind.
  int  kx_ingest(const uint8_t* buf, int len);
  int  kx_stream_rx_done_kind;   // set to KX_STREAM_FWD/REV when a transfer completes; 0 otherwise
  bool kx_as_data_path() const   // runtime A/B gate for the new transport (opt-in while unproven)
  {
    if(!encryption_enabled) return false;
    const char* e = std::getenv("MERCURY_KX_AS_DATA");
    return e && *e && atoi(e) != 0;
  }
  // TRUE while this session is in the KX-as-data pre-activation epoch on EITHER
  // side (TX staging active, or RX reassembling a KX stream, or simply
  // pre-activation under the new path). Consulted by the compression-bypass and
  // Option-W stamp/cursor suppression so both peers act deterministically from the
  // shared pre-activation state.
  bool kx_stream_epoch() const
  {
    return kx_stream_tx_active || kx_stream_rx_is_kx
        || (kx_as_data_path() && !cipher_suite.is_active());
  }
  // Re-anchor the delivery/bsi/nonce/Option-W state to a fresh bsi=0 baseline at
  // KEY_ACTIVATE on BOTH peers, so the first user-data batch begins byte-identical
  // to a normal (unencrypted) session start after KX consumed wire bsi 0..N.
  void kx_stream_reanchor();

  char psk_hex[129];                  // Pre-shared key (hex string, up to 64 bytes = 128 hex chars)
  bool psk_mismatch_pending;          // Commander detected PSK mismatch, KEY_ACTIVATE sent for responder notification

  // Reset all hybrid-KX chunk state (idle). Defined inline; called from the
  // session-reset path and at the start/end of each KX2/KX3 phase.
  void kx_chunk_state_reset()
  {
    kx_mlkem_pk_ready = false;
    kx_mlkem_ct_ready = false;
    kx_tx_kind = 0; kx_tx_total = 0; kx_tx_count = 0;
    kx_tx_next_index = 0; kx_tx_chunk_cap = 0; kx_tx_src = nullptr;
    kx_rx_kind = 0; kx_rx_count = 0; kx_rx_chunk_cap = 0;
    kx_rx_total = 0; kx_rx_received = 0;
    for (int i = 0; i < 256; i++) kx_rx_got[i] = false;
    // KX-as-data reserved-stream transport state (idle).
    kx_stream_tx_len = 0; kx_stream_tx_sent = 0; kx_stream_tx_active = false;
    kx_stream_rx_have = 0; kx_stream_rx_expected = 0;
    kx_stream_rx_kind = 0; kx_stream_rx_is_kx = false; kx_stream_rx_done_kind = 0;
  }

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
  // IN-BAND CONFIG_0 ROLLING-PARTIAL climb unblock
  // (fact-documents/data-flow-inband-frame0-rolling-partial.md §2). The LAST PARTIAL SACK's
  // signature: true iff the partial reported EXACTLY the LEAD frame (frame-0 / bit0) missing
  // while every OTHER frame of the batch decoded (an acquisition-seam loss the retx machinery
  // recovers within one batch). Set at BOTH partial producers (the MFSK-ACK-SACK and the OFDM
  // SACK_RSP partial paths); cleared at every per-batch TX start and on every clean/full ACK.
  // Read ONLY by inband_lead_frame_only_partial() at the FRAME-UP gate so the rolling CONFIG_0
  // partial advances the climb streak instead of vetoing it. A MULTI-frame-drop partial leaves
  // this false (the §9 anti-thrash veto is preserved). CMD-only.
  bool last_partial_lead_frame_only = false;
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
    // C3 (data-flow-gearshift-climb.md): the handoff cap is DEMOTE-DIRECTION-ONLY. Turbo may
    // DELIVER the SNR-indicated rung UP in one shot; the Q-table optimizer still owns
    // steady-state / demote. Skip the cap when the (already ceiling-bounded) target is a CLIMB
    // above the current rung. C2/C3 are OPT-IN (MERCURY_CLIMB_TIER2=1); the default keeps the cap
    // (deferred pending a decode-margin gate). ACCEL_DEFEAT=1 also keeps the cap (full incumbent).
    if (!climb_accel_defeat && climb_tier2 &&
        config_ladder_index(*snr_target) > config_ladder_index(current_configuration))
      return;
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
  // Climb-acceleration (data-flow-gearshift-climb.md).
  // A/B defeat knob (MERCURY_CLIMB_ACCEL_DEFEAT=1): restores the pre-accel INCUMBENT crawl —
  // legacy leap cap 13, unconditional handoff cap, no robust tier-cross. Env-latched in the
  // ctor so the fire-proof runs FIX vs DEFEAT(incumbent) on the SAME binary.
  bool climb_accel_defeat;
  // TIER-2 ENABLE knob (MERCURY_CLIMB_TIER2=1, default 0/OFF): opts IN to C2+C3 (leap cap 13->16,
  // handoff cap demote-only). The DEFAULT ships Tier-1 (C1 only); C2/C3 DEFERRED pending a decode-margin
  // gate (they over-climb the marginal boundary). C1 stays LIVE in the default and under Tier-2.
  // (The P0-gate SNR-provenance latch was DROPPED in v2: it was an unsatisfiable bootstrap
  // deadlock; the anchor tier gate is_ofdm_config(anchor) at arq.h:1441-1443 is the real safety.)
  bool climb_tier2;
  // LOW-SNR CEILING PIN (data-flow-gearshift-climb.md) — the partial-pattern up-probe data-fail
  // twin (arq_commander.cc pat path) was MISSING the decode-learned ceiling pin its pure-silence
  // sibling has, so a partial (e.g. 0.22) batch at an over-leaped rung never pinned
  // supershift_proven_ceiling and the elevator re-leaped into the dead rung (WGN:15 collapse).
  // The pin is DEFAULT-ON. A/B defeat knob (MERCURY_CEILING_PIN_DEFEAT=1): restores the old
  // missing-pin behavior so the fire-proof runs FIX vs DEFEAT on the SAME binary. Env-latched in
  // the ctor. Independent of climb_accel_defeat (this fix is on the WITHIN-ladder failure path,
  // not the leap itself).
  bool ceiling_pin_defeat;
  // C1 (data-flow-gearshift-climb.md) — the ROBUST tier-cross probe target. Returns CONFIG_0
  // when a robust climb should PROPOSE the OFDM tier directly (skip ROBUST_1/2 — they carry no
  // OFDM evidence, pure delay), or -1 to keep the +1 robust ladder. -1 when: defeated, not at a
  // robust config, or a prior failed tier-cross floored the proven ceiling below CONFIG_0. Pure:
  // production (the FRAME-UP elevator compose) and the directed test drive the SAME logic.
  int robust_climb_probe_target() const {
    if(climb_accel_defeat) return -1;
    if(!is_robust_config(current_configuration)) return -1;
    if(supershift_proven_ceiling >= 0 &&
       config_ladder_index(supershift_proven_ceiling) < config_ladder_index(CONFIG_0))
      return -1;
    return CONFIG_0;
  }
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

  // R6 measured-turnaround estimator (RFC 6298 §2; Karn & Partridge 1987).
  // Supersedes the recording-only sack_arrival_history_ms[] ring above with a
  // per-{turnaround-geometry class x forward-batch-airtime bucket} SRTT/RTTVAR
  // window. All integer ms (no float on the ARQ poll path). n==0 => COLD (the
  // consumer falls back to geometric_floor + calibrated margin, byte-identical
  // to the pre-R6 window). Reset with the ring, in the ctor.
  static const int TT_NUM_CLASS   = 3;   // turnaround-geometry classes
  static const int TT_NUM_BATCHBK = 4;   // forward-batch-airtime buckets
  struct turnaround_rtt_est {
    int srtt_ms;      // smoothed RTT, ms
    int rttvar_ms;    // RTT mean deviation, ms
    int n;            // samples folded (0 => cold)
  };
  turnaround_rtt_est tt_rtt[TT_NUM_CLASS][TT_NUM_BATCHBK];

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
  // IDLE-SWITCHROLE-RACE recovery path (idle-switchrole-race.md §3/§5.3): count
  // of consecutive BREAK-EXHAUSTED re-arms that made NO forward progress (no RX
  // data delivered this session AND tx FIFO empty AND a zero-byte re-queue). The
  // watchdog never disconnects (arq_common.cc:2985 re-arms itself), so a never-fed
  // link spins BREAK->SET_CONFIG->EXHAUSTED forever. At bound K
  // (BREAK_NOPROGRESS_TEARDOWN_K) this stops re-arming the watchdog and routes to
  // the existing FORCED-fallback teardown (arq_common.cc:3053-3065). RESET to 0 on
  // ANY real progress (RX data OR a non-empty re-queue) and in reset_session_state().
  int break_noprogress_cycles;
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

  // ---- zombie/amplifier layer (data-flow-zombie-amplifier.md) -------------
  // FIX 3 (watchdog probe gate, §3): defer the CMD watchdog teleport behind a
  // peer-liveness probe. probe_pending latches across one watchdog cycle; the
  // rx-index snapshot distinguishes "peer answered" (rx advanced) from
  // "pure silence" (confirmed dead -> clean reconnect, never an in-place teleport).
  bool watchdog_resurrect_probe_pending{false};
  long long watchdog_probe_rx_index_snap{0};
  // R5 -- consecutive pure-silent ACK rounds (zero reverse correlator activity). Only the
  // all-silent-to-threshold case (peer unreachable at every attempted config) is rerouted to a
  // clean reconnect; any reverse activity (partial SACK / sub-threshold correlator / late pattern)
  // resets it, so the forward-loss and window-problem demote paths stay byte-identical. Reset on
  // every credited ACK and inside ackfail_classifier_step.
  int  consec_pure_silent_rounds{0};
  static const int DEMOTE_SILENCE_MAX_ROUNDS = 3;   // == emergency_nack_threshold default
  static constexpr double ACK_SILENCE_EPS   = 0.10; // metric below this AND matched==0 => silence

  // ---- BREAK forward-health gate (fix/break-fh-gate) ---------------------
  // ROOT CAUSE (workflow w2ee37gd6): the responder BREAK probe
  // (detect_break_pattern_from_passband, arq_common.cc:9292) runs on the SAME
  // failed passband buffer in the decode-FAIL else-branch. Its only OFDM-alias
  // guard is the entry-gate coarse_metric<0.30 (arq_common.cc:9288) which is
  // INVERTED on the failure path: a marginal CFG16 frame has LOW coarse_metric
  // so the gate PASSES, the 50 OFDM subcarriers argmax against the 8 WB
  // break_tones (mfsk.cc:316) reaching matched>=10 (mfsk.cc:328) under
  // always_fine, and ONE probe detonates a self-demote to ROBUST_0 + SACK wipe
  // (arq_responder.cc:439-489). Two corroborating mitigations, BOTH gated on
  // MERCURY_BREAK_FH_GATE (default-off -> the new code is unread -> byte-identical):
  //   FIX-A forward-health LATCH: a forward OFDM frame decoded within the last
  //         BREAK_FH_LATCH_FRAMES receive() iterations SUPPRESSES the probe. A
  //         real commander BREAK comes AFTER the commander STOPS forward OFDM
  //         (emergency_break_active) so the latch ages out -> probe still runs.
  //   FIX-B K-of-N: require BREAK_KOFN_K consecutive probe matches before
  //         break_detected=YES. A real BREAK is retried/sustained -> survives;
  //         the per-batch alias is a one-frame transient -> does not.
  // rx_receive_frame_index: monotonic, incremented once per receive() call. It
  // is ALWAYS maintained (not env-gated) but its ONLY consumer is the env-gated
  // FH suppressor, so when the env is unset nothing reads it -> behavior is
  // bit-identical. last_forward_ofdm_decode_frame snapshots it at every
  // successful forward OFDM decode (arq_common.cc:8976 else-branch).
  static const long long BREAK_FH_LATCH_FRAMES = 8;  // recent-forward-decode window (receive() iters); bench-tunable
  static const int       BREAK_KOFN_K          = 2;  // consecutive probe matches required to detonate; bench-tunable
  long long rx_receive_frame_index{0};
  long long last_forward_ofdm_decode_frame{-1000000};  // far in the past => not recent at start
  // FIX (data-flow-inband-downladder.md §2.1): true iff THIS receive() pass actually
  // staged a FRESH capture window and attempted a primary decode (the frames_to_read==0
  // branch ran). On a benign inter-frame pass receive() takes the frames_to_read!=0
  // early-exit (arq_common.cc:12158) WITHOUT re-staging, so the staged buffer holds the
  // LAST decoded frame (stale-but-loud, energy>=0.05). The inband down-ladder gate
  // (arq_responder.cc) requires this so a lost-tag resync fires ONLY on a genuine
  // fresh-window decode-FAIL, NOT on every stale inter-frame re-probe (the 3127 "all
  // silent" HW firings). Set in receive(); cleared at the top of every receive() pass.
  // ALWAYS maintained but read ONLY inside the inband-gated block -> legacy byte-identical.
  bool      rx_fresh_window_decoded_this_pass{false};
  int       break_probe_consec_match{0};               // K-of-N accumulator (reset on non-match / consume / reset)
  // True iff the env MERCURY_BREAK_FH_GATE is set (cached once). The ONE gate-enable
  // source of truth shared by break_fh_suppress(), break_kofn_corroborate(),
  // break_fh_carve_lift(). break_fh_gate_test_override is a UNIT-TEST seam (-1 = honor
  // env, 0/1 = force) so the test can exercise both gate states in one process; production
  // leaves it at -1, so the env path is unchanged (default-off byte-identical).
  static int  break_fh_gate_test_override;
  static bool break_fh_gate_enabled();
  // recovery_ack_robust_test_override: UNIT-TEST seam (-1 = honor env, 0/1 = force) for
  // set_recovery_ack_reps_for_wait, mirroring break_fh_gate_test_override. Lets
  // test_recovery_window_covers_robust_ack drive the robust rep bump in-process despite the
  // cached MERCURY_RECOVERY_ACK_ROBUST read. Production leaves it at -1 → env path unchanged
  // → default-off byte-identical.
  static int  recovery_ack_robust_test_override;
  // True iff the gate is enabled AND a forward OFDM frame decoded within the last
  // BREAK_FH_LATCH_FRAMES receive() iterations (probe should be suppressed). When the
  // env is unset, returns false unconditionally (byte-identical).
  bool break_fh_suppress() const;
  // K-of-N corroboration. probe_matched = the per-frame match decision (metric &&
  // matched>=threshold already evaluated by the caller). Returns whether
  // break_detected should be SET this frame. When the env is unset it is a
  // pass-through (returns probe_matched, single-shot -> byte-identical); when set
  // it requires BREAK_KOFN_K consecutive matches. A non-match resets the streak.
  bool break_kofn_corroborate(bool probe_matched);
  // FIX-D: the WALL-B FIX-3 carve-suspend gate-lift, gated. When the FH gate is
  // enabled, returns false (do NOT lift the coarse<0.30 OFDM-alias guard in
  // carve-suspend state). When the env is unset, returns bigblock_carve_suspended()
  // exactly -> the receive() BREAK gate is bit-identical to 48103fa.
  bool break_fh_carve_lift();
  int hail_detected;              // YES if HAIL beacon detected (responder LISTENING)
  int hail_sent;                  // YES if commander has sent HAIL in current CONNECTING phase

  int ptt_on_delay_ms;
  int ptt_off_delay_ms;
  // Turnaround-clearance guard (cross-layer data-flow audit: TX-start vs the peer's
  // TX->RX mute/flush window). send_batch() consults turnaround_clearance_timer at the top
  // and, if too little time has elapsed since the peer's audio ended, busy-waits the
  // remainder BEFORE keying so the first data frame never lands inside the peer's
  // capture-flush/demod re-arm window.
  // R1-rescope: the WAIT fires ONLY before a data batch that follows a CONTROL turnaround
  // (a control-ACK preceding a renegotiated / new-geometry data batch — SET_LINK_PARAMS /
  // ROBUST_DWELL_BATCH_OP batch-size change, SET_CONFIG config change), armed by
  // arm_control_turnaround_guard(). A routine data-SACK turnaround leaves
  // turnaround_clearance_from_control==false and is BYTE-IDENTICAL (no wait): those
  // turnarounds already deliver slot 0 cleanly, so the original arm-on-every-reception R1
  // only taxed throughput and risked the RSP's post-SACK reverse window. receive() still
  // stamps the timer on every decoded reverse frame (routine arm) so the
  // MERCURY_TURNAROUND_GUARD_SCOPE_ALL=1 A/B arm can restore the broad behaviour.
  cl_timer turnaround_clearance_timer;
  bool     turnaround_clearance_armed = false;
  // R1-rescope discriminant: set true ONLY at a CONTROL-ACK -> data transition
  // (arm_control_turnaround_guard); a routine reverse reception leaves it false. Consumed
  // one-shot by turnaround_clearance_wait() when it keys the guarded batch.
  bool     turnaround_clearance_from_control = false;
  // Test-visible instrumentation for --test-turnaround-guard (set by
  // turnaround_clearance_wait): the ms the guard busy-waited this call (-1 = guard not
  // entered, i.e. never armed), and the clearance elapsed captured at the key point.
  long long tg_test_waited_ms        = -1;
  long long tg_test_elapsed_at_key_ms = -1;
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
  // D5 (EOB-inference batch truncation, TRACK_C_D2D3D5_DESIGN.md §5.3): the
  // TX-authoritative per-batch frame count carried on the wire (batch_total_frames
  // header byte) on EVERY v2 DATA frame of a batch. Unlike last_received_end_of_batch
  // _seq (single-frame EOB evidence — erased when the last frame is lost), this
  // survives the loss of any single frame, so a lost-EOB tail leaves received_count <
  // expected_count and the batch is held (then SACK-recovered) or loud-aborted rather
  // than silently truncated. Staged per-frame in receive() (rx_buffer_batch_total
  // _frames, mirroring rx_buffer_eob_seq), promoted to rx_batch_total_frames ONLY
  // inside the confirmed match-current / match-prev storage block. -1 = unknown
  // (v1/legacy/NB, or no frame of this batch seen yet) → consumers fall back to the
  // EOB inference. MERCURY_D5_INFER_DEFEAT=1 forces the fallback on the SAME binary
  // (the fail-before arm). Reset to -1 at session init + on every bsi bump/teardown.
  int rx_buffer_batch_total_frames;    // staged batch_total_frames for current v2 frame, or -1
  int rx_batch_total_frames;           // promoted authoritative per-batch frame count, or -1
  // Option B' (data-flow-batch-size.md §9): per-call DELIVERY window override for
  // copy_data_to_buffer(). -1 (default) => copy_data_to_buffer bounds delivery by
  // data_batch_size (byte-identical). The current-batch commit path sets this to the
  // eff_window (== rx_effective_window()) immediately before delivering a batch the
  // sender built LARGER than data_batch_size, then resets it to -1. Set/consumed like
  // decrypt_delivered_bsi (one delivery at a time; no reentrancy).
  int rx_copy_window;                  // per-call copy_data_to_buffer delivery bound, or -1
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
