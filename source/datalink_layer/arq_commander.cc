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

#include "datalink_layer/arq.h"
#include "common/snr_decision_grid.h"
#include "common/shm_posix.h"
#include "common/timing_log.h"
#include "common/sim_channel.h"   // §10.6 in-process scalar-AWGN channel (2-instance SIM_INPROC)
#include "physical_layer/mfsk_ctrl_codec.h"  // §10.2 gf16ra reconcile
#include <cerrno>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstdint>     // uint32_t/uint8_t (2-instance stepper deterministic payload)
#include <vector>      // std::vector (2-instance stepper large-payload buffer)
#include <algorithm>   // std::min (2-instance stepper RX drain)

#if !defined(_WIN32)
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

extern "C" { extern std::atomic<bool> shutdown_; }

#ifdef MERCURY_GUI_ENABLED
#include "gui/gui_main.h"
#include "gui/gui_state.h"
#include "gui/dialogs/soundcard_dialog.h"
#endif

// MERCURY_TURN_TRACE (link-phase Step 1, MEASURE-ONLY): defined in arq_common.cc.
// Emits one read-only trace line per CMD reverse-ACK turn; no-op when off.
bool arq_turn_trace_on();

// STAGE R (block-ACK pipelining, MERCURY_L1_BLOCKACK_PIPELINE): within a negotiated
// block the intermediate 1..N-1 windows carry no reverse ACK (silence is the success
// signal), so waiting a full receiving_timeout on each is dead air (~40-48% at
// cfg13-15). With pipelining the commander drops PTT and listens only a SHORT gap
// before advancing the forward burst; a real receive window opens only at the block
// boundary where the aggregate BLOCK_SACK is actually due. Default off => the gap ms
// is never consulted and the legacy full-window path runs byte-identically.
static const unsigned int L1_BLOCKACK_PIPELINE_GAP_MS = 300;
// Time-based watchdog boundary bound (the reverse-silence latent regime). With
// pipelining reverse silence per block ~= block forward air + one boundary window.
// To keep it below link_timeout by construction for ALL configs/N, force an early
// boundary (via the existing tail-commit path) once (block forward air + the boundary
// window it will need) reaches NUM/DEN of link_timeout. At 3/5 with the 30 s per-batch
// airtime cap, worst-case reverse silence stays below (0.6*lt + 30 s) < lt whenever
// lt >= 75 s (the operative data-transfer link_timeout is 90 s).
static const long long L1_BLOCKACK_PIPELINE_WD_NUM = 3;
static const long long L1_BLOCKACK_PIPELINE_WD_DEN = 5;

// STAGE R decision helpers (single source of truth; unit-tested on a real
// controller). The intermediate advance threshold: a SHORT gap when pipelining,
// else the full receiving_timeout (byte-identical legacy path).
unsigned int cl_arq_controller::l1_pipeline_intermediate_advance_timeout() const
{
	return l1_blockack_pipeline_active()
		? L1_BLOCKACK_PIPELINE_GAP_MS : (unsigned int)receiving_timeout;
}
// The time-based watchdog boundary predicate: force an early block boundary when
// the projected reverse silence (block forward air + the boundary window it will
// need) reaches NUM/DEN of link_timeout. Only meaningful mid-block under pipelining.
bool cl_arq_controller::l1_pipeline_watchdog_would_force(long long block_air_ms) const
{
	return l1_blockack_pipeline_active()
		&& l1_blockack.intermediate_silence_expected()
		&& (block_air_ms + (long long)receiving_timeout) * L1_BLOCKACK_PIPELINE_WD_DEN
			>= (long long)link_timeout * L1_BLOCKACK_PIPELINE_WD_NUM;
}
void arq_turn_trace_emit_ack(const char* kind, unsigned bsi, int arrival_ms, int karn_ok);

// SACK_RX_TRACE: env-gated diagnostic for the SACK_RSP receive path.
// Enable with MERCURY_SACK_RX_TRACE=1. No output when unset.
// Cached on first call; one branch per macro hit when disabled.
static inline bool sack_rx_trace_enabled()
{
	static int cached = -1;
	if(cached < 0)
	{
		const char* e = std::getenv("MERCURY_SACK_RX_TRACE");
		cached = (e && *e && *e != '0') ? 1 : 0;
	}
	return cached != 0;
}

// Duty-time diagnostic trace: OFF unless MERCURY_DUTY_TRACE=1 in the environment. Inert on the
// production path (a single cached getenv, no output when disabled).
static bool duty_trace_enabled()
{
	static int v = -1;
	if(v < 0)
	{
		const char* e = std::getenv("MERCURY_DUTY_TRACE");
		v = (e && e[0] == '1') ? 1 : 0;
	}
	return v != 0;
}

// M6 — BREAK-path lossless requeue (data-flow-arq-recovery-cluster.md §5.6).
// The Anchor-rung emergency BREAK (~:4071-4177) calls send_break_pattern() and
// returns WITHOUT rolling cmd_batch_seq_id back to the stranded in-flight batch
// -> on recovery the re-sent batch carries a FRESH (advanced) epoch bsi -> the RSP
// re-adopts a NON-CONTIGUOUS bsi (>=2 jump from its delivery high-water) ->
// [RSP-V2-GAP-ABORT] terminally kills the transfer. The FIX-9 D3 SET_CONFIG demote
// (~:3950-4018) already captures the earliest in-flight bsi BEFORE freeing
// messages_tx[] and rolls cmd_batch_seq_id back to it, so the re-send is contiguous.
// This knob ports that capture+rollback into the BREAK path. DEFAULT-ON 2026-06-18
// (proven fix; owner policy = proven fixes ship default-on). The fix is ENABLED unless
// the escape hatch MERCURY_BREAK_LOSSLESS_REQUEUE_DISABLE is set (any non-empty value),
// which restores the pre-M6 BYTE-IDENTICAL BREAK behavior for an A/B revert.
static inline bool break_lossless_requeue_enabled()
{
	static int cached = -1;
	if(cached < 0)
	{
		// Default-ON: enabled unless the *_DISABLE escape hatch is present.
		const char* dis = std::getenv("MERCURY_BREAK_LOSSLESS_REQUEUE_DISABLE");
		cached = (dis == nullptr) ? 1 : 0;
	}
	return cached != 0;
}
#define SACK_TRACE(fmt, ...) do { \
	if(sack_rx_trace_enabled()) { \
		printf("[SACK-RX-TRACE] " fmt "\n", ##__VA_ARGS__); \
		fflush(stdout); \
	} \
} while(0)

// ============================================================================
// R2c — PREV-BATCH RETENTION SHADOW (data-flow-recoverable-gap-abort.md §5)
//
// The mixbatch builder REMOVES a retx frame from retransmit_frames[] the instant
// it is SENT (arq_commander.cc mixbatch loop: shift-down + count-=R). If that
// retx is then LOST, its bytes are gone and a later prev-bsi re-SACK cannot
// refill the hole — the SACK apply reads messages_tx[i] BY SLOT INDEX, which by
// then holds the NEXT batch (the verified inert-root). These helpers keep
// a bounded, bsi-keyed copy of each just-SENT retx frame so a prev-bsi re-SACK
// (the case the OOW/STALE guards drop) can re-queue the ORIGINAL bytes into
// retransmit_frames[] for the next mixbatch. ADDITIVE: no bitmap is applied to
// messages_tx[] by index, so no false-ACK surface is added; the existing guards
// keep their exact behavior on the in-flight batch.
// ----------------------------------------------------------------------------

// RETENTION-SHADOW REPAIR (data-flow audit). The R2c credited-recovery path shipped
// but was PRODUCTION-INERT: the 8-slot shadow could not hold one batch's 19-24 holes
// (data_batch_size=25), cmd_prev_retain_evict() had ZERO production callers so the
// shadow silted with stale entries, overflow overwrote a fixed slot, and dedup
// re-capture never reset the re-drive budget -- so cmd_prev_retain_requeue() returned
// 0 (the measured rq0-storm) and the recoverable prev-hole deadlocked into a BREAK.
// This flag arms the four-part repair (effective cap 8->32 + production evict at
// batch-confirm + drop-oldest ring overflow + dedup round reset). Read once, cached
// per process. Default OFF => the EXACT pre-fix behavior (byte-identical control /
// fail-before arm). Independent of MERCURY_CMD_PREV_RETAIN_DEFEAT (which disables the
// shadow entirely). See the retention-shadow cross-layer data-flow audit.
static bool retain_shadow_fix_on()
{
	static int cached = -1;
	if(cached < 0)
	{
		const char* e = std::getenv("MERCURY_RETAIN_SHADOW_FIX");
		cached = (e && *e && atoi(e) == 0) ? 0 : 1;  // DEFAULT-ON: explicit =0 restores stock
	}
	return cached != 0;
}

// LINK-PHASE STEP 6 / MC-4 — derive the RSP's recoverable batch-generation
// depth from the storage that actually exists. messages_rx[] holds one CURRENT
// generation. messages_rx_prev[] adds one PREV generation only in the repaired
// retention-shadow era, where the CMD shadow can retain a complete worst-case
// batch of holes (CMD_PREV_RETAIN_MAX >= MAX_SACK_BATCH_SIZE). Thus the shipped
// repaired geometry derives 1+1=2; disabling the retention repair conservatively
// collapses admission to current-only depth 1 instead of guessing a magic depth.
static int rxwindow_rsp_reorder_depth()
{
	const int current_generation = 1;
	const int recoverable_prev_generation =
		(retain_shadow_fix_on() && CMD_PREV_RETAIN_MAX >= MAX_SACK_BATCH_SIZE) ? 1 : 0;
	return current_generation + recoverable_prev_generation;
}

// Consume the raw cumulative prefix after integrity, target ownership/span, and
// class validation.  This is deliberately the wire field (target-1), while all
// routing and bitmap consumers use the separately resolved target.
void cl_arq_controller::cmd_rxwindow_note_delivered(int n_r, bool clean)
{
	if(!linkphase_rxwindow_on() || !sack_v2_enabled || !cumulative_ack_enabled)
		return;

	const int nr = n_r & 0xFF;
	const int highest_sent = (cmd_batch_seq_id - 1) & 0xFF;
	if(!generation_canon_enabled() && !clean && nr == highest_sent)
		return;

	if(cmd_rxwindow_delivered_high_water < 0)
	{
		cmd_rxwindow_delivered_high_water = nr;
	}
	else
	{
		const int fwd = (nr - (cmd_rxwindow_delivered_high_water & 0xFF)) & 0xFF;
		if(fwd < 1 || fwd > 128)
			return;  // duplicate or stale/backward report
		cmd_rxwindow_delivered_high_water = nr;
	}

	// Re-evaluate capacity from a fresh clock after real delivery progress.
	cmd_rxwindow_hold_timer.stop();
	cmd_rxwindow_hold_timer.reset();
}

// Return true only for NEW-batch admission. Pure retransmit turns never call this
// helper, and this helper deliberately does not touch link/watchdog/receiving
// timers, retransmit queues, BREAK state, or connection state.
bool cl_arq_controller::cmd_rxwindow_hold_new_batch()
{
	if(!linkphase_rxwindow_on() || !sack_v2_enabled || !cumulative_ack_enabled)
		return false;

	const int delivered_next = (cmd_rxwindow_delivered_high_water < 0)
		? 0 : ((cmd_rxwindow_delivered_high_water + 1) & 0xFF);
	const int outstanding = ((cmd_batch_seq_id & 0xFF) - delivered_next) & 0xFF;
	const int depth = rxwindow_rsp_reorder_depth();

	// A mod-256 distance beyond the half-window is inconsistent with this bounded
	// protocol (or an enable/reset discontinuity). Fail open rather than deadlock.
	if(outstanding > 128 || outstanding < depth)
	{
		cmd_rxwindow_hold_timer.stop();
		cmd_rxwindow_hold_timer.reset();
		return false;
	}

	if(cmd_rxwindow_hold_timer.counting == NO)
	{
		cmd_rxwindow_hold_timer.reset();
		cmd_rxwindow_hold_timer.start();
		printf("[LINKPHASE-RXWINDOW] HOLD new bsi=%d highest_sent=%d delivered_hw=%d "
			"outstanding=%d depth=%d (retx/recovery remain live)\n",
			cmd_batch_seq_id & 0xFF, (cmd_batch_seq_id - 1) & 0xFF,
			cmd_rxwindow_delivered_high_water, outstanding, depth);
		fflush(stdout);
		return true;
	}

	// Deadlock safety: use the modem's existing link-loss horizon, not a new magic
	// timeout. After one link_timeout with no n_r progress, fail open for one
	// admission and restart the bound. Existing BREAK/link recovery has remained
	// runnable for the entire hold and remains authoritative.
	int fallback_ms = link_timeout;
	if(fallback_ms <= 0) fallback_ms = receiving_timeout;
	if(fallback_ms <= 0)
	{
		cmd_rxwindow_hold_timer.stop();
		cmd_rxwindow_hold_timer.reset();
		return false;
	}
	if(cmd_rxwindow_hold_timer.get_elapsed_time_ms() >= fallback_ms)
	{
		printf("[LINKPHASE-RXWINDOW] FALLBACK new bsi=%d after %dms without n_r progress "
			"(outstanding=%d depth=%d); fail-open one admission so recovery cannot hang\n",
			cmd_batch_seq_id & 0xFF, fallback_ms, outstanding, depth);
		fflush(stdout);
		cmd_rxwindow_hold_timer.stop();
		cmd_rxwindow_hold_timer.reset();
		return false;
	}
	return true;
}

// Stash a just-SENT retx frame keyed by its ORIGINAL batch_seq_id + slot. Dedup:
// updates an existing (bsi,slot) entry in place. On overflow (repaired), drops the
// OLDEST entry (ring); pre-fix (OFF) evicts an exhausted
// (rounds>=RSP_GAP_RECOVER_MAX) entry, else the oldest (index 0).
void cl_arq_controller::cmd_prev_retain_capture(int bsi, int slot, int len,
	int type, unsigned char seq_eob, const unsigned char* bytes)
{
	{ const char* e = std::getenv("MERCURY_CMD_PREV_RETAIN_DEFEAT");
	  if(e && *e && atoi(e)!=0) return; }
	if(!sack_v2_enabled) return;
	if(bytes == NULL || len <= 0 || len > MAX_SACK_FRAME_SIZE) return;
	if(bsi < 0) return;

	// Dedup by (bsi, slot).
	for(int k=0; k<cmd_prev_retain_count; k++)
	{
		if(cmd_prev_retain_bsi[k]==(bsi&0xFF) && cmd_prev_retain_slot[k]==slot)
		{
			cmd_prev_retain_len[k]     = len;
			cmd_prev_retain_type[k]    = type;
			cmd_prev_retain_seq_eob[k] = seq_eob;
			memcpy(cmd_prev_retain_bytes[k], bytes, len);
			// REPAIR part 4: a fresh retransmit of this (bsi,slot) restarts its
			// re-drive budget -- a stale round count from a prior recovery episode
			// must not permanently disqualify a live frame the peer still demands.
			// The RSP_GAP_RECOVER_MAX bound still bites a genuinely dead peer (no
			// fresh retransmit arrives to re-capture, so rounds are never reset;
			// I-1 verified by the WGN:8 dead-link gate). OFF => unchanged (base).
			if(retain_shadow_fix_on()) cmd_prev_retain_rounds[k] = 0;
			return;
		}
	}

	// REPAIR part 1: effective cap is runtime-gated. OFF -> 8 (byte-identical to
	// pre-fix, array over-sized but only 8 slots ever used). ON -> 32 (holds one
	// batch's worst-case 19-24 holes so requeue can actually find the missing slots).
	const int cap = retain_shadow_fix_on() ? CMD_PREV_RETAIN_MAX : CMD_PREV_RETAIN_BASE;
	int idx;
	if(cmd_prev_retain_count < cap)
	{
		idx = cmd_prev_retain_count++;
	}
	else if(retain_shadow_fix_on())
	{
		// REPAIR part 3: drop-oldest RING. Shift entries [1..cap-1] down by one
		// (evicting index 0, the oldest) and insert the new frame at the tail. A
		// full shadow now drops the STALEST hole, never a fixed slot -- so the
		// most-recent (still-recoverable) holes are the ones retained. count stays
		// at cap. (Combined with the production evict at batch-confirm, the shadow
		// no longer silts, so this ring is a backstop, not the primary un-silt.)
		for(int k=1; k<cap; k++)
		{
			cmd_prev_retain_bsi[k-1]     = cmd_prev_retain_bsi[k];
			cmd_prev_retain_slot[k-1]    = cmd_prev_retain_slot[k];
			cmd_prev_retain_len[k-1]     = cmd_prev_retain_len[k];
			cmd_prev_retain_type[k-1]    = cmd_prev_retain_type[k];
			cmd_prev_retain_seq_eob[k-1] = cmd_prev_retain_seq_eob[k];
			cmd_prev_retain_rounds[k-1]  = cmd_prev_retain_rounds[k];
			int cl = cmd_prev_retain_len[k];
			memcpy(cmd_prev_retain_bytes[k-1], cmd_prev_retain_bytes[k],
				(cl > 0 && cl <= MAX_SACK_FRAME_SIZE) ? cl : 0);
		}
		idx = cap - 1;
	}
	else
	{
		// pre-fix (OFF): prefer evicting an exhausted entry; else the oldest (0).
		idx = 0;
		for(int k=0; k<cmd_prev_retain_count; k++)
			if(cmd_prev_retain_rounds[k] >= RSP_GAP_RECOVER_MAX) { idx=k; break; }
	}
	cmd_prev_retain_bsi[idx]     = (bsi & 0xFF);
	cmd_prev_retain_slot[idx]    = slot;
	cmd_prev_retain_len[idx]     = len;
	cmd_prev_retain_type[idx]    = type;
	cmd_prev_retain_seq_eob[idx] = seq_eob;
	cmd_prev_retain_rounds[idx]  = 0;
	memcpy(cmd_prev_retain_bytes[idx], bytes, len);
}

// Drop every retained entry for `bsi` (called when that batch is confirmed
// delivered, so its frames can never be a recoverable hole again).
void cl_arq_controller::cmd_prev_retain_evict(int bsi)
{
	int w = 0;
	for(int k=0; k<cmd_prev_retain_count; k++)
	{
		if(cmd_prev_retain_bsi[k] == (bsi & 0xFF)) continue;   // drop
		if(w != k)
		{
			cmd_prev_retain_bsi[w]     = cmd_prev_retain_bsi[k];
			cmd_prev_retain_slot[w]    = cmd_prev_retain_slot[k];
			cmd_prev_retain_len[w]     = cmd_prev_retain_len[k];
			cmd_prev_retain_type[w]    = cmd_prev_retain_type[k];
			cmd_prev_retain_seq_eob[w] = cmd_prev_retain_seq_eob[k];
			cmd_prev_retain_rounds[w]  = cmd_prev_retain_rounds[k];
			memcpy(cmd_prev_retain_bytes[w], cmd_prev_retain_bytes[k],
				cmd_prev_retain_len[k] > 0 && cmd_prev_retain_len[k] <= MAX_SACK_FRAME_SIZE
					? cmd_prev_retain_len[k] : 0);
		}
		w++;
	}
	cmd_prev_retain_count = w;
}

// On a prev-bsi re-SACK (rx_bsi is a retained batch, NOT the in-flight one),
// re-queue the retained frames of that bsi that the SACK still reports MISSING
// (got_bitmap[slot]==false) into retransmit_frames[]. Skips a frame already
// queued for the same (bsi,slot) so a repeated re-SACK does not bloat the queue,
// and caps each entry at RSP_GAP_RECOVER_MAX re-drive rounds. Returns the count
// re-queued. No new wire format — it reuses the existing retx queue + mixbatch.
int cl_arq_controller::cmd_prev_retain_requeue(int bsi, const bool* got_bitmap,
	int nframes)
{
	{ const char* e = std::getenv("MERCURY_CMD_PREV_RETAIN_DEFEAT");
	  if(e && *e && atoi(e)!=0) return 0; }
	if(!sack_v2_enabled || got_bitmap == NULL) return 0;
	int requeued = 0;
	for(int k=0; k<cmd_prev_retain_count; k++)
	{
		if(cmd_prev_retain_bsi[k] != (bsi & 0xFF)) continue;
		int slot = cmd_prev_retain_slot[k];
		if(slot < 0 || slot >= nframes) continue;
		if(got_bitmap[slot]) continue;                         // RX has it — nothing to send
		if(cmd_prev_retain_rounds[k] >= RSP_GAP_RECOVER_MAX) continue;   // bound
		// Already queued for this (bsi,slot)? Skip (append-mode dedup).
		bool already = false;
		for(int q=0; q<retransmit_count; q++)
			if(retransmit_frame_batch_seq_ids[q] == (bsi & 0xFF)
			   && retransmit_frame_positions[q] == slot) { already = true; break; }
		if(already) continue;
		if(retransmit_count >= MAX_RETRANSMIT_HEADROOM) break;
		int len = cmd_prev_retain_len[k];
		if(len <= 0 || len > MAX_SACK_FRAME_SIZE) continue;
		memcpy(retransmit_frames[retransmit_count], cmd_prev_retain_bytes[k], len);
		retransmit_frame_lengths[retransmit_count]      = len;
		retransmit_frame_positions[retransmit_count]    = slot;
		retransmit_frame_types[retransmit_count]        = cmd_prev_retain_type[k];
		retransmit_frame_batch_seq_ids[retransmit_count]= (bsi & 0xFF);
		retransmit_frame_seq_with_eob[retransmit_count] = cmd_prev_retain_seq_eob[k];
		retransmit_count++;
		cmd_prev_retain_rounds[k]++;
		requeued++;
	}
	return requeued;
}

// R2c-W1 (data-flow-recoverable-gap-abort.md 5.1): is any live shadow entry
// keyed by `bsi`? A live entry means the CMD sent that bsi frame(s) as retx and
// they are NOT yet confirmed (eviction clears the shadow on confirm), so the
// frame bytes exist ONLY here, never in messages_tx[] (which now describes the
// current new-data batch).
bool cl_arq_controller::cmd_prev_retain_has(int bsi)
{
	for(int k=0; k<cmd_prev_retain_count; k++)
		if(cmd_prev_retain_bsi[k] == (bsi & 0xFF)) return true;
	return false;
}

// R2c-W1: route a decoded PARTIAL re-SACK to the retention-shadow re-drive
// (skipping the by-slot-index apply) iff rx_bsi is NOT the current new-data batch
// AND its frames are shadow-retained. The original R2c requeue only fired when
// rx_bsi was NEITHER cmd_bsi NOR prev_bsi (!inflight), but the recoverable-hold
// storm leaves the CMD exactly ONE batch past the armed-prev hole: cmd_bsi=N+1,
// the re-advertised hole is prev_bsi=N. That rx_bsi is in the {cmd,prev} window,
// so it is NOT OOW/STALE-dropped and the by-slot apply would FALSE-ACK batch N+1
// PENDING_ACK frames by index (they never retransmit -> the held batch never
// re-completes). N real frames are gone from messages_tx[] (consumed on their
// retx send); they survive ONLY in the shadow. So: shadow-retained AND not the
// current batch => re-drive the retained bytes and CONSUME the SACK. Gated by
// MERCURY_CMD_PREV_RETAIN_DEFEAT (fail-before restores the drop/mis-apply = 0/39).
bool cl_arq_controller::cmd_prev_resack_is_shadow_target(int rx_bsi)
{
	{ const char* e = std::getenv("MERCURY_CMD_PREV_RETAIN_DEFEAT");
	  if(e && *e && atoi(e)!=0) return false; }
	if(!sack_v2_enabled) return false;
	if((rx_bsi & 0xFF) == (cmd_batch_seq_id & 0xFF)) return false;  // current batch -> by-slot apply
	return cmd_prev_retain_has(rx_bsi);
}

void cl_arq_controller::register_ack(int message_id)
{
	register_acks(std::vector<int>(1, message_id));
}

bool cl_arq_controller::register_acks(const std::vector<int>& message_ids,
	bool accept_timed_out)

{
	std::vector<std::pair<uint8_t, uint16_t> > journal_keys;
	std::vector<int> accepted;
	for(int message_id : message_ids)
	{
		if(message_id<0 || message_id>=this->nMessages) continue;
		const int status = messages_tx[message_id].status;
		if(status!=PENDING_ACK &&
		   !(status==ACK_TIMED_OUT &&
		     (l1_tx_journal.enabled() || accept_timed_out)))
			continue;
		accepted.push_back(message_id);
		journal_keys.push_back({(uint8_t)messages_tx[message_id].batch_seq_id,
			(uint8_t)messages_tx[message_id].id});
	}
	if(l1_tx_journal.enabled() &&
	   !l1_tx_journal.acknowledge_many(journal_keys))
	{
		fprintf(stderr, "[L1-JOURNAL] rejected whole positive ACK (%zu key(s))\n",
			journal_keys.size());
		return false;
	}
	for(int message_id : accepted)
	{
		messages_tx[message_id].status=ACKED;
		stats.nAcked_data++;
	}
	// Fix C / H#1: if this ACK completed the block (no un-confirmed data left), flush
	// the delivered raw from fifo_buffer_backup now, before any config-change re-stage
	// (finalize may be pre-empted by a queued SET_CONFIG / relaxed-mixbatch staging).
	maybe_backup_confirm_flush();
	return true;
}

// Fix C / H#1 (data-flow-fifo-backup.md §6.1). See the arq.h declaration for the full
// rationale. Flushes fifo_buffer_backup at the ACK-confirm boundary IFF the backup can
// hold ONLY already-delivered raw — i.e. there is NO un-confirmed data anywhere:
//   - retransmit_count == 0 (no frame pending re-transmit), AND
//   - no messages_tx[] slot is PENDING_ACK / ACK_TIMED_OUT (un-ACKed in-flight) or
//     ADDED_TO_LIST / ADDED_TO_BATCH_BUFFER (a staged-but-unsent NEXT batch whose raw
//     is ALSO in the backup — the relaxed-mixbatch accumulation case).
// Under that predicate every backup byte belongs to a delivered batch, so flushing it
// cannot lose un-confirmed data (INV3). This fixes Root B (a delivered batch re-staged
// on a SUPER-ACK/turboshift SET_CONFIG leap) and, whenever a lossy run momentarily
// fully-confirms, empties the accumulated backup (bounds H#1). NOTE (scoped residual):
// a SUSTAINED, UNBROKEN loss run keeps retransmit_count>0 so this never fires mid-run;
// removing an individual delivered batch's raw WHILE other batches are still un-confirmed
// needs per-batch raw framing (data-flow-fifo-backup.md §6.3) and is a deliberate
// follow-on — a byte-offset flush there risks dropping interleaved un-confirmed raw
// (worse than the bug), so it is intentionally not attempted here.
void cl_arq_controller::maybe_backup_confirm_flush()
{
	{ const char* e = std::getenv("MERCURY_BACKUP_CONFIRMFLUSH_DEFEAT");
	  if(e && *e && atoi(e)!=0) return; }          // fail-before: leave delivered raw in the backup
	if(role != COMMANDER)                 return;
	int backup_occupancy = fifo_buffer_backup.get_size() - fifo_buffer_backup.get_free_size();
	if(backup_occupancy <= 0)             return;   // nothing staged
	if(retransmit_count != 0)             return;   // un-confirmed retx pending -> its raw is in the backup
	for(int i=0; i<this->nMessages; i++)
	{
		int s = messages_tx[i].status;
		if(s==PENDING_ACK || s==ACK_TIMED_OUT || s==ADDED_TO_LIST || s==ADDED_TO_BATCH_BUFFER)
			return;                                 // un-confirmed data present -> NOT safe to whole-flush
	}
	printf("[CMD-BACKUP-CONFIRM-FLUSH] block ACK-confirmed (no unconfirmed frames, retx=0) — "
		"flushing %d B delivered raw from backup to prevent re-stage double-delivery (Fix C/H#1)\n",
		backup_occupancy);
	fflush(stdout);
	fifo_buffer_backup.flush();
}

// Phantom-ACK content gate — DSP half (2026-05-29). Peek the current passband
// tail for a CRC12-valid, in-window, CLEAN-batch (all-ones bitmap) MFSK ACK+SACK
// suffix. This is the CONTENT discriminator the bare-pattern data-ACK arm needs:
// a real WB clean data ACK carries this suffix; the phantom (structured noise /
// rx-tail self-match that passes receive_ack_pattern()'s bare matched/metric
// gate) does NOT. Read-only peek — does NOT touch frames_to_read (mirrors the
// §7.13.30 no-side-effect peek at the v2 MFSK pre-detect, ~line 2370). Window
// math + decode + CRC12 + bsi-in-window + all-ones checks mirror that block
// (~lines 2351-2433) exactly. Returns false on NB / suffix-incapable
// (ack_sack_suffix_len()==0), no decode, CRC mismatch, out-of-window bsi, or a
// non-clean (partial) bitmap. See fact-documents/gearshift-start-and-recovery.md §8.
bool cl_arq_controller::cmd_clean_data_ack_crc_valid()
{
#if MFSK_ACK_SACK_ENABLED
	if(telecom_system->ack_mfsk.ack_sack_suffix_len() <= 0)
		return false;  // NB / suffix-incapable: no suffix to validate.

	// Tail window — identical math to the v2 MFSK pre-detect (arq_commander.cc
	// ~2351-2364). Suffix capture needs the longer of SNR vs SACK suffix tail.
	int ack_nsymb   = telecom_system->ack_mfsk.ack_pattern_nsymb;
	int pattern_len = telecom_system->ack_mfsk.ack_snr_pattern_nsymb();
	int sack_suffix_len = telecom_system->ack_mfsk.ack_sack_suffix_len();
	if(sack_suffix_len > pattern_len - ack_nsymb)
		pattern_len = ack_nsymb + sack_suffix_len;
	const int mfsk_tail_nsymb = ack_nsymb + pattern_len + 16;
	int sym_samples = telecom_system->data_container.Nofdm
	                * telecom_system->data_container.interpolation_rate;
	int signal_period = sym_samples * telecom_system->data_container.buffer_Nsymb;
	int tail_samples = mfsk_tail_nsymb * sym_samples;
	if(tail_samples > signal_period)
		tail_samples = signal_period;
	int tail_offset = signal_period - tail_samples;

	// Read-only snapshot of the tail (no frames_to_read mutation).
	MUTEX_LOCK(&capture_prep_mutex);
	int rwi_mfsk = telecom_system->data_container.ring_write_index;
	memcpy(telecom_system->data_container.ready_to_process_passband_delayed_data,
		&telecom_system->data_container.passband_delayed_data[rwi_mfsk + tail_offset],
		tail_samples * sizeof(double));
	MUTEX_UNLOCK(&capture_prep_mutex);

	uint8_t  rx_bsi = 0;
	uint32_t rx_bitmap = 0;
	uint16_t rx_crc12 = 0;
	int      mfsk_matched = 0;
	bool decoded = telecom_system->decode_ack_sack_from_passband(
		telecom_system->data_container.ready_to_process_passband_delayed_data,
		tail_samples, &rx_bsi, &rx_bitmap, &rx_crc12, &mfsk_matched);
	if(!decoded)
		return false;
	const uint8_t wire_bsi = rx_bsi;

	// CRC12 verification (mercury/fact-documents/mfsk-robust-ack.md §3.2).
	char crc_input[5];
	crc_input[0] = (char)wire_bsi;
	crc_input[1] = (char)((rx_bitmap >> 24) & 0xFF);
	crc_input[2] = (char)((rx_bitmap >> 16) & 0xFF);
	crc_input[3] = (char)((rx_bitmap >>  8) & 0xFF);
	crc_input[4] = (char)( rx_bitmap        & 0xFF);
	if(rx_crc12 != CRC12_calc(crc_input, 5))
		return false;
	bool reverse_demand = false;
	rx_bsi = role_demand_restore_ack_bsi(wire_bsi, &reverse_demand);

	const int target = (int)generation_ack_resolve_target(
		rx_bsi, cumulative_ack_enabled);
	const int inflight = current_inflight_bsi();
	if(!generation_ack_target_owned(target, data_batch_size,
		/*allow_shadow=*/false, nullptr)
	   || inflight < 0 || target != (inflight & 0xFF))
		return false;

	// CLEAN-batch only: this arm accepts full-batch ACKs (all-ones bitmap). A
	// partial bitmap belongs to the SACK_RSP path, which runs inside the SACK
	// window (not this bare arm); reject it here so it is not mis-accepted.
	bool clean = mfsk_sack_bitmap_is_clean(rx_bitmap, data_batch_size);
	if(clean)
	{
		cmd_rxwindow_note_delivered((int)rx_bsi, /*clean=*/true);
		role_demand_note_acknowledged(reverse_demand, "mfsk-ack-sack-clean");
		if(std::getenv("MERCURY_GEN_CANON_WITNESS"))
			printf("[GEN-CANON-RESOLVE] transport=MFSK-CLEAN wire=%u target=%u "
			       "canon=%d cumulative=%d clean=1 span=%d owner=live\n",
				(unsigned)rx_bsi, (unsigned)target,
				generation_canon_enabled() ? 1 : 0,
				cumulative_ack_enabled ? 1 : 0, data_batch_size);
	}
	return clean;
#else
	return false;
#endif
}

// LINK-PHASE STEP 3 (increment 3, sub-class a) — the sample-anchored older-phase reverse-ACK
// rescan. This is a PURELY ADDITIVE OR-term appended at the DATA-ACK accept site: it is reached
// ONLY after the incumbent newest-tail accept (cmd_compact_confirm_live_accept) has already
// returned false, and it returns false (byte-identical to stock) unless the step-3 knob is on,
// the CMD reverse-ACK window is open under the strong primitive guard, and we are in a DATA-ACK
// wait. The §158 storm signature is a reverse ACK that was physically received but scrolled out
// of the newest audio tail before the poll fired, so every newest-tail snapshot read a partial
// pattern (matched can peak >=8 on a poll yet metric stays below accept — the correlation is
// diluted by the misaligned/missing portion). The incumbent already correlates the newest tail;
// this steps BACK through older ring phases (the retained history holds far more than the ACK
// round-trip) and runs BOTH the bare-pattern gate AND the clean-batch CRC content gate on the
// SAME snapshot per phase — so the bare match and the CRC decode necessarily agree on ONE phase.
// The search is POSITIONED/BOUNDED by the sample-derived primitive window (next_listen_open_ms
// -> a look-back cap) and GATED to owner==TURNAROUND, so it can only credit a reverse ACK that
// is genuinely expected on this session/config/keydown; the EXACT phase is chosen by sample
// content (the correlator + CRC pass), never by a wall-clock poll instant. WIDENS acceptance,
// never narrows: a miss at every phase returns false and the caller's incumbent miss/break path
// runs exactly as today. NB (suffix-incapable) credits on the bare pattern alone, mirroring the
// incumbent `!suffix_capable` arm.
bool cl_arq_controller::cmd_data_ack_anchored_rescan_accept()
{
	long long next_listen_open_ms = 0;
	if(!linkphase_ackwin_on()
	   || this->connection_status != RECEIVING_ACKS_DATA
	   || !lp_ack_window_active(&next_listen_open_ms))
		return false;
#ifdef LINKPHASE_ACKWIN_NOSCAN
	// FAIL-BEFORE build: the older-phase step is compiled out, so the anchored rescan can never
	// credit -> the incumbent newest-tail miss stands (the ARM-1 fail-before of test_linkphase_ackwin).
	return false;
#else
	// Tail window — identical geometry to cmd_clean_data_ack_crc_valid (the newest-tail content
	// gate this rescans at older phases).
	int ack_nsymb   = telecom_system->ack_mfsk.ack_pattern_nsymb;
	int pattern_len = telecom_system->ack_mfsk.ack_snr_pattern_nsymb();
	int sack_suffix_len = telecom_system->ack_mfsk.ack_sack_suffix_len();
	if(sack_suffix_len > pattern_len - ack_nsymb)
		pattern_len = ack_nsymb + sack_suffix_len;
	const int mfsk_tail_nsymb = ack_nsymb + pattern_len + 16;
	int sym_samples = telecom_system->data_container.Nofdm
	                * telecom_system->data_container.interpolation_rate;
	if(sym_samples <= 0) return false;
	int signal_period = sym_samples * telecom_system->data_container.buffer_Nsymb;
	int tail_samples = mfsk_tail_nsymb * sym_samples;
	if(tail_samples > signal_period)
		tail_samples = signal_period;
	int tail_offset = signal_period - tail_samples;
	if(tail_offset <= 0) return false;   // no retained history to rescan

	// Stride one ACK-pattern length per phase (overlapping windows; no ACK falls entirely
	// between two phases), the same stride the existing multi-window scan uses.
	int stride = ack_nsymb * sym_samples;
	if(stride < sym_samples) stride = sym_samples;

	// Sample-derived look-back bound from the primitive: how far back the reverse ACK could sit,
	// = the time since the reverse listener opened, in 48 kHz audio samples. Clamp to the ring
	// and hard-cap the phase count so the per-poll correlator cost stays modest.
	const int LP_ACKWIN_MAX_PHASES = 24;
	long long ms_since_open = lp_now() - next_listen_open_ms;
	int lookback_samples = tail_samples;
	if(ms_since_open > 0 && ms_since_open < 60000)
		lookback_samples += (int)(ms_since_open * 48);   // 48 samples/ms @ 48 kHz audio ring
	if(lookback_samples > tail_offset) lookback_samples = tail_offset;
	int max_phases = lookback_samples / stride + 1;
	if(max_phases > LP_ACKWIN_MAX_PHASES) max_phases = LP_ACKWIN_MAX_PHASES;

	const bool suffix_capable = (sack_suffix_len > 0);
	const unsigned cmd_bsi  = (unsigned)(cmd_batch_seq_id & 0xFF);
	const unsigned prev_bsi = (cmd_bsi - 1u) & 0xFFu;

	for(int ph = 1; ph <= max_phases; ph++)
	{
		int off = tail_offset - ph * stride;
		if(off < 0) break;
		// Read-only snapshot of this older phase (no frames_to_read mutation).
		MUTEX_LOCK(&capture_prep_mutex);
		int rwi = telecom_system->data_container.ring_write_index;
		memcpy(telecom_system->data_container.ready_to_process_passband_delayed_data,
			&telecom_system->data_container.passband_delayed_data[rwi + off],
			tail_samples * sizeof(double));
		MUTEX_UNLOCK(&capture_prep_mutex);

		// Bare-pattern gate (the SAME accept bar as the incumbent newest-tail poll).
		int matched = 0; uint32_t mask = 0;
		double metric = telecom_system->detect_ack_pattern_from_passband(
			telecom_system->data_container.ready_to_process_passband_delayed_data,
			tail_samples, &matched, &mask, /*use_fine=*/false);
		if(!(matched >= telecom_system->ack_mfsk.ack_match_threshold
		     && metric >= ack_metric_threshold))
			continue;

		if(suffix_capable)
		{
#if MFSK_ACK_SACK_ENABLED
			// WB content gate: the clean-batch CRC decode, on the SAME older-phase buffer, so the
			// bare match and the CRC agree on ONE phase. Byte-for-byte the cmd_clean_data_ack_crc_valid
			// decision (decode -> CRC12 -> coverage -> clean/all-ones), only at this phase.
			uint8_t  rx_bsi = 0;
			uint32_t rx_bitmap = 0;
			uint16_t rx_crc12 = 0;
			int      mfsk_matched = 0;
			bool decoded = telecom_system->decode_ack_sack_from_passband(
				telecom_system->data_container.ready_to_process_passband_delayed_data,
				tail_samples, &rx_bsi, &rx_bitmap, &rx_crc12, &mfsk_matched);
			if(!decoded)
				continue;
			const uint8_t wire_bsi = rx_bsi;
			char crc_input[5];
			crc_input[0] = (char)wire_bsi;
			crc_input[1] = (char)((rx_bitmap >> 24) & 0xFF);
			crc_input[2] = (char)((rx_bitmap >> 16) & 0xFF);
			crc_input[3] = (char)((rx_bitmap >>  8) & 0xFF);
			crc_input[4] = (char)( rx_bitmap        & 0xFF);
			if(rx_crc12 != CRC12_calc(crc_input, 5))
				continue;
			bool reverse_demand = false;
			rx_bsi = role_demand_restore_ack_bsi(wire_bsi, &reverse_demand);
			const int target = (int)generation_ack_resolve_target(
				rx_bsi, cumulative_ack_enabled);
			const int inflight = current_inflight_bsi();
			if(!generation_ack_target_owned(target, data_batch_size,
				/*allow_shadow=*/false, nullptr)
			   || inflight < 0 || target != (inflight & 0xFF))
				continue;
			if(!mfsk_sack_bitmap_is_clean(rx_bitmap, data_batch_size))
				continue;
			cmd_rxwindow_note_delivered((int)rx_bsi, /*clean=*/true);
			role_demand_note_acknowledged(reverse_demand, "mfsk-ack-sack-rescan");
#else
			continue;   // suffix-capable but SACK decode unavailable -> cannot validate; do not credit
#endif
		}

		// Credit at this anchored older phase. Advance the ring the same way the incumbent
		// accept branch would (frames_to_read=4 via commit_ack_pattern_consumed), account the
		// prevention counter, and emit the wire-activation witness.
		commit_ack_pattern_consumed();
		linkphase_false_break_averted++;
		printf("[LINKPHASE-ACKWIN] rescan credited reverse-ACK subclass=a ph=%d "
			"back_samples=%d matched=%d metric=%.3f cmd_bsi=%u\n",
			ph, ph * stride, matched, metric, cmd_bsi);
		fflush(stdout);
		return true;
	}
	return false;
#endif  // LINKPHASE_ACKWIN_NOSCAN
}

// Production CRC12 callback for the compact-confirm decode (ctx = this).
// File-local clone of arq_common.cc's arq_ctrl_crc12_cb (that one is static
// there). NEVER inline the CRC (v1 bug #1 init mismatch). Matches ctrl_crc12_fn.
static uint16_t cmd_compact_crc12_cb(void* ctx, const unsigned char* data, int n)
{
	cl_arq_controller* self = static_cast<cl_arq_controller*>(ctx);
	return self->CRC12_calc((const char*)data, n) & 0x0FFF;
}

// Option B (data-flow-compact-confirm.md §5): peek the passband tail for a
// CRC12-valid, in-window COMPACT confirm. Structurally cloned from
// cmd_clean_data_ack_crc_valid; differences: shorter capture window (16+10 vs
// 16+max(8,13)), decode via decode_compact_confirm_from_passband, CRC12 over the
// single [bsi] byte (gated INSIDE soft_decode_compact), CLEAN/all-ones implicit.
bool cl_arq_controller::cmd_compact_confirm_crc_valid(uint8_t* out_bsi)
{
#if MFSK_ACK_SACK_ENABLED
	if(telecom_system->ack_mfsk.compact_confirm_suffix_len() <= 0)
		return false;  // NB / suffix-incapable.

	// Tail window — same math as cmd_clean_data_ack_crc_valid but sized to the
	// (shorter) compact confirm. The compact suffix (10) <= the ACK/SACK suffix
	// (13), so the existing ring is strictly safe (data-flow §6 I1/I4).
	int ack_nsymb   = telecom_system->ack_mfsk.ack_pattern_nsymb;
	int compact_suffix = telecom_system->ack_mfsk.compact_confirm_suffix_len();
	// Report-transport capture window. When either legacy topgear or Gearshift-v2
	// quality telemetry can append a tail, the responder may emit a two-codeword
	// confirm (ack_nsymb + 2*compact_suffix
	// symbols) rather than the plain compact confirm (ack_nsymb + compact_suffix).
	// Sizing the tail for the plain confirm leaves only 16-compact_suffix (=6) symbols
	// of positioning margin for the longer topgear confirm, so whenever the detected
	// ACK base lands >6 symbols into the window the report codeword is truncated out of
	// the capture (needed_end>dec_size in decode_compact_confirm_from_passband) and
	// topgear_report_valid never becomes true. Give the topgear confirm the SAME
	// 16-symbol margin. Default-off feature => byte-identical unless MERCURY_TOPGEAR_ELECT.
	bool tail_failbefore = false;
	{ const char* e = std::getenv("MERCURY_TOPGEAR_TAIL_FAILBEFORE"); tail_failbefore = (e && *e && atoi(e) != 0); }
	const int compact_span = (gearshift_quality_report_transport_enabled() && !tail_failbefore) ? 2 * compact_suffix : compact_suffix;
	const int mfsk_tail_nsymb = ack_nsymb + compact_span + 16;
	int sym_samples = telecom_system->data_container.Nofdm
	                * telecom_system->data_container.interpolation_rate;
	int signal_period = sym_samples * telecom_system->data_container.buffer_Nsymb;
	int tail_samples = mfsk_tail_nsymb * sym_samples;
	if(tail_samples > signal_period)
		tail_samples = signal_period;
	int tail_offset = signal_period - tail_samples;

	// Read-only snapshot of the tail (no frames_to_read mutation).
	MUTEX_LOCK(&capture_prep_mutex);
	int rwi_mfsk = telecom_system->data_container.ring_write_index;
	memcpy(telecom_system->data_container.ready_to_process_passband_delayed_data,
		&telecom_system->data_container.passband_delayed_data[rwi_mfsk + tail_offset],
		tail_samples * sizeof(double));
	MUTEX_UNLOCK(&capture_prep_mutex);

	uint8_t rx_bsi = 0;
	uint8_t quality_report = 0;
	bool    quality_report_valid = false;
	bool    want_quality_report = gearshift_quality_report_transport_enabled();
	int     mfsk_matched = 0;
	bool decoded = telecom_system->decode_compact_confirm_from_passband(
		telecom_system->data_container.ready_to_process_passband_delayed_data,
		tail_samples, cmd_compact_crc12_cb, this, &rx_bsi, &mfsk_matched,
		want_quality_report ? &quality_report : nullptr,
		want_quality_report ? &quality_report_valid : nullptr);
	if(!decoded)
		return false;  // soft_decode_compact already gated CRC12-over-[bsi].
	bool reverse_demand = false;
	rx_bsi = role_demand_restore_ack_bsi(rx_bsi, &reverse_demand);

	bool cumulative = scalable_sack_on() && cumulative_ack_enabled;
	const int target = (int)generation_ack_resolve_target(rx_bsi, cumulative);
	const int inflight = current_inflight_bsi();
	if(!generation_ack_target_owned(target, data_batch_size,
		/*allow_shadow=*/false, nullptr)
	   || inflight < 0 || target != (inflight & 0xFF))
		return false;
	if(std::getenv("MERCURY_GEN_CANON_WITNESS"))
	{
		printf("[GEN-CANON-RESOLVE] transport=COMPACT wire=%u target=%u "
		       "canon=%d cumulative=%d clean=1 span=%d owner=live/journal\n",
			(unsigned)rx_bsi, (unsigned)target,
			generation_canon_enabled() ? 1 : 0,
			cumulative ? 1 : 0, data_batch_size);
		fflush(stdout);
	}
	cmd_rxwindow_note_delivered((int)rx_bsi, /*clean=*/true);
	role_demand_note_acknowledged(reverse_demand, "compact-ack");
	if(quality_report_valid)
	{
		if(gearshift_quality_report_v2_enabled())
			gearshift_apply_quality_report(quality_report, target);
		else
			topgear_apply_report(quality_report, target);
		// A consume-time report supersedes any older pending one (fresher evidence).
		topgear_pending_report_clear("consume-time-report");
	}
	else if(want_quality_report
	        && (gearshift_quality_report_v2_enabled()
	            || current_configuration == CONFIG_16 || current_configuration == CONFIG_17))
	{
		// CONSUME-RACE fix: the confirm is being accepted at FIRST-codeword CRC
		// validity, but on the live vehicle the trailing report codeword's audio has
		// not fully arrived yet (audio-bracketed: accept fires with ~2 of 36 symbols
		// still unplayed, every time). Do NOT hold the accept (ACK timing is
		// protocol-critical); arm the DEFERRED report decode instead — subsequent
		// polls re-snapshot the tail and decode the report once its audio exists.
		// The cfg16/17 gate mirrors the responder's append condition
		// (send_mfsk_compact_confirm) so plain 26-sym confirms never arm pending.
		// MERCURY_TOPGEAR_CONSUME_FAILBEFORE=1 restores the single-poll consume
		// (the pre-fix behavior) for the regression's fail-before arm and live A/B.
		bool consume_failbefore = false;
		{ const char* e = std::getenv("MERCURY_TOPGEAR_CONSUME_FAILBEFORE"); consume_failbefore = (e && *e && atoi(e) != 0); }
		if(!consume_failbefore)
			topgear_pending_report_arm(target);
	}

	// CLEAN-batch implicit: a compact confirm MEANS all-ones (the responder only
	// emits it for a fully-received batch; partial loss uses the SACK path).
	if(out_bsi)
		*out_bsi = (uint8_t)target;
	return true;
#else
	(void)out_bsi;
	return false;
#endif
}

// ── Topgear report CONSUME-RACE fix: deferred stashed-tail report decode ──
// See the member-block note in arq.h and the TOPGEAR_PENDING_REPORT_DEADLINE_MS
// note in common_defines.h for the full mechanism. Summary: the confirm is
// consumed on time (accept/ACK/RTO timing untouched); the report codeword —
// whose audio provably had not arrived at consume time (audio-bracketed on the
// live vehicle) — is decoded LATER from a dedicated tail stash that is
// re-snapshotted each poll while the ring is live and frozen before any
// ring-destroying event. The same mechanism now also services the optional
// Gearshift-v2 quality tail; it never delays acceptance of the ACK prefix.

void cl_arq_controller::topgear_pending_report_arm(int bsi)
{
#if MFSK_ACK_SACK_ENABLED
	topgear_clear_forward_verdict();
	topgear_pending_report_bsi = bsi & 0xFF;
	topgear_pending_cfg = current_configuration;
	topgear_pending_stash_frozen = false;
	topgear_pending_stash_dirty = false;
	topgear_pending_stash_samples = 0;
	topgear_pending_timer.stop();
	topgear_pending_timer.reset();
	topgear_pending_timer.start();
	const char* qtrace = std::getenv("MERCURY_GS2_QUALITY_TRACE");
	if(!gearshift_quality_report_v2_enabled() || (qtrace && *qtrace && atoi(qtrace) != 0))
	{
		printf("[TOPGEAR-PENDING] arm bsi=%d cfg=%d (report codeword not yet in capture; deferring decode)\n",
			topgear_pending_report_bsi, topgear_pending_cfg);
		fflush(stdout);
	}
	// First snapshot right away — on a fast capture path the report may already be
	// complete by the next poll, and the freshest-overwrite ticks only improve it.
	topgear_pending_stash_refresh();
#else
	(void)bsi;
#endif
}

void cl_arq_controller::topgear_pending_report_clear(const char* reason)
{
#if MFSK_ACK_SACK_ENABLED
	if(topgear_pending_report_bsi < 0)
		return;
	const char* qtrace = std::getenv("MERCURY_GS2_QUALITY_TRACE");
	if(!gearshift_quality_report_v2_enabled() || (qtrace && *qtrace && atoi(qtrace) != 0))
	{
		printf("[TOPGEAR-PENDING] clear bsi=%d (%s)\n", topgear_pending_report_bsi, reason);
		fflush(stdout);
	}
	if(strcmp(reason, "applied") != 0)
		topgear_clear_forward_verdict();
	topgear_pending_report_bsi = -1;
	topgear_pending_cfg = CONFIG_NONE;
	topgear_pending_stash_frozen = false;
	topgear_pending_stash_dirty = false;
	topgear_pending_stash_samples = 0;
	topgear_pending_timer.stop();
	topgear_pending_timer.reset();
	// Complete any DEFERRED SACK-V2 listen-PHY restore this pending was holding
	// open (see restore_sack_v2_rx_phy). With the pending gone the deferral
	// condition is false, so this either restores now or no-ops.
	restore_sack_v2_rx_phy();
#else
	(void)reason;
#endif
}

// Live-ring tail -> stash. SAME tail geometry as cmd_compact_confirm_crc_valid's
// snapshot (the widened topgear span — pending only ever arms with the feature on),
// but into the DEDICATED stash buffer: ready_to_process_passband_delayed_data is
// shared scratch that any OFDM/MFSK dispatch may overwrite between polls. No-op
// once frozen (the ring is gone or about to be) and while rx_mute has the capture
// thread writing zeros (a muted-ring copy would clobber good stash audio).
void cl_arq_controller::topgear_pending_stash_refresh()
{
#if MFSK_ACK_SACK_ENABLED
	if(topgear_pending_report_bsi < 0 || topgear_pending_stash_frozen)
		return;
	if(telecom_system == NULL || telecom_system->data_container.rx_mute)
		return;
	if(telecom_system->ack_mfsk.compact_confirm_suffix_len() <= 0)
		return;
	int ack_nsymb      = telecom_system->ack_mfsk.ack_pattern_nsymb;
	int compact_suffix = telecom_system->ack_mfsk.compact_confirm_suffix_len();
	const int mfsk_tail_nsymb = ack_nsymb + 2 * compact_suffix + 16;
	int sym_samples = telecom_system->data_container.Nofdm
	                * telecom_system->data_container.interpolation_rate;
	int signal_period = sym_samples * telecom_system->data_container.buffer_Nsymb;
	int tail_samples = mfsk_tail_nsymb * sym_samples;
	if(tail_samples > signal_period)
		tail_samples = signal_period;
	int tail_offset = signal_period - tail_samples;
	if((int)topgear_pending_stash.size() < tail_samples)
		topgear_pending_stash.resize((size_t)tail_samples);
	MUTEX_LOCK(&capture_prep_mutex);
	int rwi = telecom_system->data_container.ring_write_index;
	memcpy(topgear_pending_stash.data(),
		&telecom_system->data_container.passband_delayed_data[rwi + tail_offset],
		(size_t)tail_samples * sizeof(double));
	MUTEX_UNLOCK(&capture_prep_mutex);
	topgear_pending_stash_samples = tail_samples;
	topgear_pending_stash_dirty = true;
#endif
}

// Final refresh + freeze, called BEFORE a ring-destroying event (send_batch()'s
// entry flush zeroes the ring ~209 ms after the accept — before the report has
// always traversed the capture pipeline — and every other TX path's post-drain
// flush is fronted by ptt_on()). ~0.1 ms memcpy; the ONLY thing this fix ever
// adds to the ARQ critical path. Decode happens off-path at the next tick.
void cl_arq_controller::topgear_pending_stash_freeze()
{
#if MFSK_ACK_SACK_ENABLED
	if(topgear_pending_report_bsi < 0 || topgear_pending_stash_frozen)
		return;
	topgear_pending_stash_refresh();
	topgear_pending_stash_frozen = true;
	// Complete any DEFERRED SACK-V2 listen-PHY restore now that the tail is
	// stashed (the accept-time restore is held while a pending report's audio is
	// still arriving — see restore_sack_v2_rx_phy). Ordering: the tail snapshot
	// above happened in the listen epoch; the restore's ring re-anchor can no
	// longer take the audio with it. TX paths call this hook BEFORE keying, so
	// no transmission ever runs in the listen PHY.
	restore_sack_v2_rx_phy();
#endif
}

// The compact confirm's bsi byte carries the reverse-demand flag in its top bit
// (ROLE_DEMAND_BSI_FLAG; the generation bit is masked off the wire). The consume-time
// accept restores the generation before resolving the batch target; the deferred
// report decode must do the same, or its target never equals the pending bsi and
// every deferred report is dropped at the deadline. The demand bit itself was
// already noted by the consume-time accept of the same confirm.
int cl_arq_controller::topgear_pending_resolve_target(unsigned char wire_bsi)
{
	const unsigned char rx = role_demand_restore_ack_bsi(wire_bsi, NULL);
	return (int)generation_ack_resolve_target(rx, scalable_sack_on() && cumulative_ack_enabled);
}

// Per-poll deferred decode driver (top of process_messages_commander). While a
// pending report is armed: refresh the stash from the live ring, and whenever the
// stash changed, re-attempt the FULL confirm decode ON THE STASH. On a decode whose
// bsi matches the pending confirm and whose report codeword validates, apply it —
// topgear_apply_report de-dupes per-bsi and validates the marker nibble, so a late
// or repeated apply is idempotent by design. Staleness is bounded three ways: a
// NEWER confirm accept replaces/clears the pending (bsi-match keeps a stale stash
// from crediting the wrong batch), a config change clears it (decode geometry and
// election context both moved), and the deadline expires it (a missed report
// self-heals on the next clean confirm's fresh report).
void cl_arq_controller::topgear_pending_report_tick()
{
#if MFSK_ACK_SACK_ENABLED
	if(topgear_pending_report_bsi < 0)
		return;
	if(!gearshift_quality_report_transport_enabled())
	{
		topgear_pending_report_clear("feature-off");
		return;
	}
	if(current_configuration != topgear_pending_cfg)
	{
		topgear_pending_report_clear("config-change");
		return;
	}
	topgear_pending_stash_refresh();
	if(topgear_pending_stash_dirty && topgear_pending_stash_samples > 0)
	{
		topgear_pending_stash_dirty = false;
		uint8_t rx_bsi = 0, report = 0;
		bool report_valid = false;
		int matched = 0;
		bool decoded = telecom_system->decode_compact_confirm_from_passband(
			topgear_pending_stash.data(), topgear_pending_stash_samples,
			cmd_compact_crc12_cb, this, &rx_bsi, &matched, &report, &report_valid);
		int resolved_target = topgear_pending_resolve_target(rx_bsi);
		if(decoded && resolved_target == topgear_pending_report_bsi && report_valid)
		{
			if(gearshift_quality_report_v2_enabled())
				gearshift_apply_quality_report(report, resolved_target);
			else
				topgear_apply_report(report, resolved_target);
			topgear_pending_report_clear("applied");
			return;
		}
	}
	{
		int deadline_ms = TOPGEAR_PENDING_REPORT_DEADLINE_MS;
		const char* e = std::getenv("MERCURY_TOPGEAR_PENDING_DEADLINE_MS");
		if(e && *e && atoi(e) > 0)
			deadline_ms = atoi(e);
		if(topgear_pending_timer.get_elapsed_time_ms() > deadline_ms)
			topgear_pending_report_clear("deadline");
	}
#endif
}

// Option B fix-b (data-flow-compact-confirm.md §10.4/§10.5): the SHARED predicate
// the Branch-2 SACK-window probe consumes to accept a COMPACT confirm INSIDE the
// SACK window — extracted so production (process_messages_rx_acks_data) and the
// live-RX regression (test_compact_confirm_sack_window_rx_path) drive ONE copy (no
// comment/code drift; the cl_arq_controller §5 precedent of cmd_compact_confirm_live_accept).
//
// THE SECOND live-land defect: for a batch>1, SACK-on, clean, WB session the RSP
// emits the compact confirm (arq_responder.cc:2576), but the Branch-2 probe decoded
// ONLY the 13-uncoded ACK+SACK (CRC12 over 5-byte [bsi||bitmap]); the compact tail
// (CRC12 over the single [bsi] byte) FAILS that 5-byte CRC12 and is dropped, and the
// SACK-closed compact arm (cmd_compact_confirm_live_accept) HARD-BAILS at
// :sack_window_open. The confirm is lost -> receiving_timeout -> retransmit -> the
// 4.7x batch>1-WB delivery regression at ENABLE=1.
//
// THE FIX (root cause): try the SELF-VALIDATING compact decode FIRST, inside the
// window, BEFORE the 13-uncoded decode. cmd_compact_confirm_crc_valid() does its OWN
// tail snapshot + 16-sym base detect + GF(16) K=5 soft-decode + CRC12-over-[bsi] +
// bsi-in-window — the CRC12 (NOT the SACK window) is the false-confirm guard. On a
// valid compact CRC we route through the EXACT clean-confirmation state the 13-uncoded
// CLEAN branch sets (split-dedupe, cmd_last_applied_clean_bsi, inband_retag_confirm_
// from_sack, v2_ack_pat_pre_detected=true) so the downstream CLEAN funnel is identical.
// v2_ack_pat_pre_detected routes to the proven CLEAN accept (the "Data ACK pattern
// detected" else-if) — there is NO commit_ack_pattern_consumed()/frames_to_read
// mutation here (the v2 clean path does not advance the ring that way; the OFDM dispatch
// the caller skips owns frames_to_read=0), so NO double-advance vs the Branch-3 arm.
//
// Returns true iff a compact confirm was decoded+accepted (or de-duplicated) this poll
// — the caller sets mfsk_handled_this_poll to suppress BOTH the 13-uncoded decode and
// the OFDM dispatch for this poll. Returns false (compact MISS / not enabled / not
// WB-capable / out-of-window bsi) so the caller falls through to the UNCHANGED 13-uncoded
// decode. Gated on compact_enabled (ARQ_COMPACT_CONFIRM_ENABLE, held off) => returns
// false immediately when 0, so the caller is byte-identical to the pre-fix path.
//
// §5 INVARIANTS: (I3/§6 no-cross-validate) the compact CRC12-over-[bsi] cannot validate
// a 13-uncoded [bsi||bitmap] suffix and vice-versa -> FAR unchanged, a 13-uncoded clean
// ACK still takes the existing path; (partial-SACK intact) the compact decode REJECTS a
// SACK partial frame (different field layout) so the partial branch is untouched and a
// partial is NEVER routed here; (clean/partial dedupe split) we dedupe vs
// cmd_last_applied_clean_bsi exactly like the 13-uncoded clean branch -> a repeated
// compact for the same bsi never double-counts nBatches_fully_acked.
bool cl_arq_controller::cmd_compact_confirm_sack_window_accept(bool compact_enabled,
                                                               bool* out_pre_detected)
{
#if MFSK_ACK_SACK_ENABLED
	if(!compact_enabled)
		return false;   // master gate held off -> byte-identical to the pre-fix path.
	if(telecom_system->ack_mfsk.compact_confirm_suffix_len() <= 0)
		return false;   // NB / M<16 — compact confirm is WB-only.

	uint8_t cc_bsi = 0;
	// Self-validating: own snapshot + base detect + GF(16) soft-decode + CRC12-over-[bsi]
	// + bsi-in-window. The CRC12 (not the SACK window) is the false-confirm guard.
	if(!cmd_compact_confirm_crc_valid(&cc_bsi))
		return false;   // compact MISS -> fall through to the 13-uncoded decode.
	// The CRC-valid compact codeword can be decoded while the peer is still
	// draining its reverse playback. Causally arm routine turnaround clearance
	// before any next DATA batch is allowed to key.
	arm_routine_turnaround_guard();

	// CLEAN confirmation — mirror the 13-uncoded CLEAN branch state-for-state.
	// Split-dedupe: a CLEAN supersedes a partial for the same bsi (climb-engine Bug 1)
	// and a repeated CLEAN for the same bsi is rejected (no double-count).
	bool duplicate = !sack_clean_confirmation_accepted(
		(int)cc_bsi, /*is_all_ones=*/true,
		cmd_last_applied_clean_bsi, cmd_last_applied_sack_bsi);
	if(!duplicate)
	{
		// v2_ack_pat_pre_detected is a CALLER LOCAL (the CLEAN-funnel routing flag),
		// not a member — set it through the out-param so a fresh CLEAN compact lands in
		// the proven clean accept (the "Data ACK pattern detected" else-if). A DUPLICATE
		// must NOT set it (no re-credit) but is still "handled" (return true) so the
		// doomed 13-uncoded re-decode of the same tail is skipped.
		if(out_pre_detected)
			*out_pre_detected = true;
		cmd_last_applied_clean_bsi = (int)cc_bsi;
		// STAGE 4d (D1 CONFIRM): a CLEAN confirm at-or-after the announce bsi proves the
		// RX demodulated the batch at the announced config -> DISARM the re-tag (no-op
		// when not armed / stale bsi / inband off).
		inband_retag_confirm_from_sack((int)cc_bsi);
		int arrival_ms = (int)receiving_timer.get_elapsed_time_ms();
		// R6: fold this reverse-ACK arrival into the measured-turnaround estimator,
		// Karn-gated (RFC 6298 §3): skip retransmit rounds (ambiguous turnaround).
		// arrival_ms is in the receiving_timer frame = the receiving_timeout deadline
		// frame, so it is a direct RTO sample. See data-flow-turnaround-timers.md.
		if(tt_karn_sample_ok())
			update_turnaround_estimate(current_configuration,
				data_batch_size * message_transmission_time_ms, arrival_ms);
		printf("[CMD-COMPACT-CONFIRM] CLEAN (in-SACK-window) batch_seq_id=%u "
			"(cmd_batch_seq_id=%d) arrival_ms=%d\n",
			(unsigned)cc_bsi, cmd_batch_seq_id, arrival_ms);
		fflush(stdout);
		if(arq_turn_trace_on())
			arq_turn_trace_emit_ack("clean-compact", (unsigned)cc_bsi, arrival_ms, tt_karn_sample_ok()?1:0);
	}
	// Whether fresh or de-duplicated, the compact tail was consumed this poll — return
	// true so the caller skips the (doomed) 13-uncoded decode of the same tail.
	return true;
#else
	(void)compact_enabled; (void)out_pre_detected;
	return false;
#endif
}

// Option B clean DATA-ACK accept arm — the SHARED predicate the "Data ACK pattern
// detected" else-if consumes, extracted so production and the live-RX regression
// (test_compact_confirm_live_rx_path) drive ONE copy (no comment/code drift).
//
// ROOT-CAUSE FIX (data-flow-compact-confirm.md §9). The OLD wiring gated the
// self-validating compact confirm BEHIND the CRC-less bare-pattern presence gate:
//   receive_ack_pattern() && (... || (compact_enabled && cmd_compact_confirm_crc_valid()) || ...)
// receive_ack_pattern() has an 8-SYMBOL ENERGY PRE-GATE (it probes only the last 8
// symbols of the tail for energy before running the FFT correlator). When the reverse
// confirm lands EARLIER in the tail window — which the live capture-prep timing does
// (inter-Pi clock drift + PTT/turnaround drain scroll post-frame idle silence in
// behind the frame before the frames_to_read==0 snapshot fires) — that probe reads
// SILENCE, SKIPS the correlator, and returns FALSE even though the frame is fully
// present. So cmd_compact_confirm_crc_valid() is never reached -> every confirm missed
// -> BREAK / false-confirm storm. (Verified, NOT assumed: the IN-WINDOW correlator
// peak is 16/16 for BOTH the 26-sym compact AND the 29-sym ACK+SACK frame — the base
// 16-sym pattern is byte-identical, both call generate_ack_pattern. Over a 31-phase
// sweep the bare gate accepts 8 phases, the full-tail compact decode 17 — the bare
// gate's 8-sym pre-gate is phase-FRAGILE, it is not a correlator peak deficiency on
// the shorter frame. See test_compact_confirm_live_rx_path DIAG.)
//
// FIX: the compact confirm carries CRC12 (over [bsi]) and runs its OWN 16-sym base
// detect + GF(16) soft-decode + bsi-in-window check inside cmd_compact_confirm_crc_valid().
// That is the self-validation the bare ACK lacks, so the compact path is tried FIRST,
// DECOUPLED from receive_ack_pattern(). The CRC12 (not the 7/16 count) is the
// false-confirm protection, so this does NOT weaken the bare-pattern gate for the
// legacy bare ACK and does NOT re-open false-confirm (the no-cross-validate invariant
// holds: the compact CRC12 over [bsi] cannot validate a 13-uncoded [bsi||bitmap] ACK).
// On a compact accept we advance the ring via commit_ack_pattern_consumed() — the
// frames_to_read=4 bookkeeping that receive_ack_pattern() would have done and that the
// decoupled compact path now bypasses.
//
// The legacy bare ACK arm (receive_ack_pattern() + the WB CRC content gate
// cmd_clean_data_ack_crc_valid()) is UNCHANGED — byte-identical when compact_enabled
// is false (ARQ_COMPACT_CONFIRM_ENABLE==0, the held-off default).
bool cl_arq_controller::cmd_compact_confirm_live_accept(bool sack_window_open,
                                                        bool compact_enabled,
                                                        bool use_legacy_chain)
{
#if MFSK_ACK_SACK_ENABLED
	// The bare-arm path is OUTSIDE the SACK window (inside the window the lenient
	// bare match false-fires on OFDM SACK_RSP body audio — see the §8 silent-drop
	// note). Identical guard to the production else-if (sack_window_open is a caller
	// local, passed in).
	if(sack_window_open)
		return false;

	const bool suffix_capable =
		(telecom_system->ack_mfsk.ack_sack_suffix_len() > 0);

	if(use_legacy_chain)
	{
		// FAIL-BEFORE (test/diagnostic only): the OLD compact-behind-bare-gate order.
		// Reproduces the missed confirm — when the frame sits earlier in the tail,
		// receive_ack_pattern()'s 8-symbol energy pre-gate reads silence and returns
		// FALSE, so the compact CRC decode is never reached.
		return receive_ack_pattern()
		       && (!suffix_capable
		           || (compact_enabled && cmd_compact_confirm_crc_valid())
		           || cmd_clean_data_ack_crc_valid());
	}

	// PASS-AFTER (production): try the SELF-VALIDATING compact confirm FIRST,
	// decoupled from the bare-pattern presence gate. cmd_compact_confirm_crc_valid()
	// does its own tail snapshot + base detect + GF(16) soft-decode + CRC12 +
	// bsi-in-window, so the bare 7/16 count never gates it.
	if(compact_enabled && cmd_compact_confirm_crc_valid())
	{
		// Advance the ring the same way the bare gate's accept branch would
		// (frames_to_read=4). The compact peek is read-only (no frames_to_read
		// mutation), so without this the ring would be left un-consumed.
		commit_ack_pattern_consumed();
		return true;
	}

	// Legacy bare ACK arm — UNCHANGED. The bare-pattern presence gate protects the
	// CRC-less bare ACK; on WB (suffix-capable) it is content-gated by
	// cmd_clean_data_ack_crc_valid().
	return receive_ack_pattern()
	       && (!suffix_capable || cmd_clean_data_ack_crc_valid());
#else
	(void)compact_enabled; (void)use_legacy_chain;
	return false;
#endif
}

// Option B (data-anchored gearshift promotion, 2026-05-29): floor a raw BREAK
// recovery target at last_data_viable_config so BREAK never drops BELOW the
// highest rung that has carried data this session. EXCEPTION: the panic-jump
// (breaks_since_last_data_success >= 2 → break_drop_step forced large at
// arq_commander.cc:3253) must still reach ROBUST_0, so the floor is bypassed
// under panic and the raw target is returned unchanged.
// See fact-documents/gearshift-start-and-recovery.md §6/§7.
int cl_arq_controller::break_target_with_anchor(int raw_target) const
{
	if(breaks_since_last_data_success >= 2)
		return raw_target;  // panic-jump safety net — let it reach ROBUST_0
	if(config_ladder_index(raw_target) < config_ladder_index(last_data_viable_config))
		return last_data_viable_config;
	return raw_target;
}

int cl_arq_controller::break_weld_target(int proposed_target) const
{
	const char* text = std::getenv("MERCURY_BREAK_PIN_CONFIG");
	if(text == NULL || *text == '\0') return proposed_target;
	char* end = NULL;
	errno = 0;
	long value = std::strtol(text, &end, 10);
	if(end == text || *end != '\0' || errno == ERANGE
		|| value < INT_MIN || value > INT_MAX)
		return proposed_target;
	int pinned = static_cast<int>(value);
	return config_ladder_index(pinned) >= 0 ? pinned : proposed_target;
}

// Keep the two diagnostic attributions tied to the production recovery stages.
// These helpers are deliberately separate from BREAK-WELD attribution: a pin may
// change final_target without the anchor floor or rung-floor cap having fired.
static inline bool break_raise_cap_log_needed(int raw_target, int clamped_target,
	int capped_target, int final_target)
{
	(void)raw_target;
	(void)final_target;
	return capped_target != clamped_target;
}

static inline bool break_anchor_floor_log_needed(int raw_target, int clamped_target,
	int capped_target, int final_target)
{
	(void)capped_target;
	(void)final_target;
	return clamped_target != raw_target;
}

// REAL FAST-PROBE piece (B) — the SHARED elevator-target computation
// (gearshift-climb-engine.md §14). Extracted VERBATIM from the SUPERSHIFT
// re-trigger (formerly inline at arq_commander.cc:4587-4608) so that site AND the
// FRAME-UP elevator share ONE copy of the cap-chain + the high-confidence-SNR
// gate — no comment/code drift. The sequence is byte-equivalent to the pre-extract
// re-trigger:
//   1. snr_ideal = get_configuration(measurements.SNR_uplink - SUPERSHIFT_MARGIN_DB)
//   2. NB cap:               if NB && snr_ideal > NB_CONFIG_MAX -> NB_CONFIG_MAX
//   3. proven-ceiling cap:   if proven>=0 && snr_ideal > proven -> proven
//   4. the §13 helper supershift_retrigger_target() — keeps the conservative +1
//      result unless the high-confidence-SNR predicate (SNR>-90 AND the
//      ceiling-capped snr_ideal lands > anchor+1) licenses a multi-rung jump.
// The result NEVER exceeds proven-safe (step 2/3), and the helper READS but never
// RAISES last_data_viable_config (SAFETY #2/#3). The CALLER gates this on
// gear_shift_on==YES && is_ofdm_config(current_configuration) &&
// measurements.SNR_uplink > -90, so at the deep-SNR cliff the >-90 gate is unmet
// and the elevator never fires (DEEP-SNR INERT). Non-const (get_configuration is
// non-const). See §13/§14.
int cl_arq_controller::elevator_target_from_snr()
{
	return elevator_target_from_snr_value(measurements.SNR_uplink, true);
}

int cl_arq_controller::elevator_target_from_snr_value(double snr, bool reverse_meter_caps)
{
	if(!reverse_meter_caps)
	{
		// FORWARD-report election: same map, NB/proven-ceiling/retrigger/cooldown caps, and the
		// CFG16 decode-margin gate evaluated on the forward value (never-raise).
		int t = get_configuration(snr - SUPERSHIFT_MARGIN_DB);
		if(narrowband_enabled == YES && t > NB_CONFIG_MAX)
			t = NB_CONFIG_MAX;
		if(supershift_proven_ceiling >= 0
		   && config_ladder_index(t) > config_ladder_index(supershift_proven_ceiling))
			t = supershift_proven_ceiling;
		t = apply_bigblock_cooldown_cap(t);
		if(narrowband_enabled != YES && !(snr > CFG16_MIN_SNR_DB)
		   && config_ladder_index(t) > config_ladder_index(CONFIG_15))
			t = CONFIG_15;
		return t;
	}
	int snr_ideal = get_configuration(snr - SUPERSHIFT_MARGIN_DB);
	if(narrowband_enabled == YES && snr_ideal > NB_CONFIG_MAX)
		snr_ideal = NB_CONFIG_MAX;
	// Enforce proven ceiling from prior BREAK failures.
	if(supershift_proven_ceiling >= 0
	   && config_ladder_index(snr_ideal) > config_ladder_index(supershift_proven_ceiling))
		snr_ideal = supershift_proven_ceiling;
	snr_ideal = supershift_retrigger_target(snr_ideal, measurements.SNR_uplink,
		last_data_viable_config, optimizer_is_in_control(),
		robust_enabled, narrowband_enabled == YES);
	// WALL-B FIX-5: the SNR re-trigger elevator (consumed :5180) and the FRAME-UP elevator
	// share this method. Folding the cooldown cap HERE covers both climb paths from one
	// place (the §4.4 hook #3 + the design's "shared by the SNR re-trigger and FRAME-UP
	// elevator" note). When the cooldown is disarmed (the normal case) this is the identity.
	snr_ideal = apply_bigblock_cooldown_cap(snr_ideal);
	// CFG16 decode-margin gate: the SNR re-trigger elevator AND the FRAME-UP elevator share
	// this method, so folding the gate here holds both off CFG16 until the reverse-path SNR
	// supports it. Never-raise; identity when the gate is open (high SNR) or WB-off.
	snr_ideal = apply_cfg16_margin_cap(snr_ideal);
	// Per-rung MEASURED election floor (generalizes the cfg16 gate to every OFDM rung): cap a
	// climb target that the over-reading suffix meter elects above its measured delivery floor
	// (the wb13 snr3k+13 cfg14 stall). Never-raise; identity below CONFIG_9 / at high SNR / WB-off.
	snr_ideal = rung_floor_cap_fire(snr_ideal);
	if(snr_track_feature_enabled())
		snr_ideal = snr_track_apply_climb_cap(snr_ideal);
	return snr_ideal;
}


// ---- NB connect-reply prior guard + forward-report entry leap ----
bool cl_arq_controller::nb_prior_guard_enabled()
{
	const char* e = std::getenv("MERCURY_NB_PRIOR_GUARD");
	return !(e && *e && atoi(e) == 0);
}

// The commander's cold-start SNR_uplink / reverse prior comes from the single narrowband
// cfg0 connect reply. That reading does not track the channel (NB receive-chain ceiling,
// bimodal ~4/~12 dB at every true SNR) and describes a different geometry than the WB
// session about to run. On the switch to wideband it is invalidated: SNR_uplink returns to
// the unmeasured sentinel and the Gearshift-v2 channel reports are cleared, so the WB prior
// and the elevator wait for a WB-geometry measurement.
void cl_arq_controller::nb_prior_guard_on_wb_switch()
{
	entry_leap_wait_done = false;
	entry_leap_wait_start_ms = 0;
	if(!nb_prior_guard_enabled() || role != COMMANDER)
		return;
	printf("[NB-PRIOR-GUARD] WB switch: drop NB prior SNR_uplink=%.1f rsnr=%.1f fsnr=%.1f\n",
		measurements.SNR_uplink, gearshift_reverse_snr_db, gearshift_forward_snr_db);
	fflush(stdout);
	measurements.SNR_uplink = -99.9;
	gearshift_reset_channel_reports();
}

bool cl_arq_controller::entry_leap_fwd_enabled()
{
	const char* e = std::getenv("MERCURY_ENTRY_LEAP_FWD");
	return !(e && *e && atoi(e) == 0);
}

int cl_arq_controller::entry_leap_target_from_fwd(double fwd_snr_db)
{
	if(!(fwd_snr_db > -90.0) || !std::isfinite(fwd_snr_db))
		return CONFIG_NONE;
	int t = elevator_target_from_snr_value(fwd_snr_db, false);
	if(!is_ofdm_config(t))
		return CONFIG_NONE;
	return t;
}

// Hold the FIRST wideband cold-start probe dispatch until the peer's forward quality report
// riding that confirm has been decoded (the confirm is accepted at first-codeword CRC while
// the report tail is still in flight; dispatching immediately changes config and discards
// it). Bounded by MERCURY_ENTRY_LEAP_WAIT_MS (default 1500 ms), once per WB session.
bool cl_arq_controller::entry_leap_report_wait_hold()
{
	if(entry_leap_wait_done || !entry_leap_fwd_enabled())
		return false;
	if(!rate_opt.controls_link() || narrowband_enabled == YES || role != COMMANDER)
		return false;
	if(rate_opt.entry_leap_dispatched() || opt_pending_switch_action != GEARSHIFT_ACTION_PROBE)
		return false;
	const bool fwd_fresh = gearshift_forward_snr_db > -90.0 &&
		gearshift_forward_snr_age_batches <= rate_opt.get_policy().forward_quality_max_age_batches;
	const unsigned long long now = opt_now_ms();
	if(fwd_fresh)
	{
		entry_leap_wait_done = true;
		int rec = current_configuration;
		const int before = opt_pending_switch_cfg;
		if(opt_evaluate_batch_end(&rec))
			opt_pending_switch_cfg = rec;
		printf("[ENTRY-LEAP] report in: fsnr=%.1f waited=%llums pending %d -> %d\n",
			gearshift_forward_snr_db,
			entry_leap_wait_start_ms ? (now - entry_leap_wait_start_ms) : 0ULL,
			before, opt_pending_switch_cfg);
		fflush(stdout);
		return false;
	}
	if(topgear_pending_report_bsi < 0)
	{
		entry_leap_wait_done = true;
		printf("[ENTRY-LEAP] no pending report at first dispatch; organic target %d\n",
			opt_pending_switch_cfg);
		fflush(stdout);
		return false;
	}
	int wait_ms = 1500;
	{
		const char* e = std::getenv("MERCURY_ENTRY_LEAP_WAIT_MS");
		if(e && *e && atoi(e) > 0) wait_ms = atoi(e);
	}
	if(entry_leap_wait_start_ms == 0)
	{
		entry_leap_wait_start_ms = now;
		printf("[ENTRY-LEAP] holding cold-start dispatch %d -> %d for forward report (<=%dms)\n",
			current_configuration, opt_pending_switch_cfg, wait_ms);
		fflush(stdout);
	}
	if(now - entry_leap_wait_start_ms > (unsigned long long)wait_ms)
	{
		entry_leap_wait_done = true;
		printf("[ENTRY-LEAP] report wait expired; organic target %d\n", opt_pending_switch_cfg);
		fflush(stdout);
		return false;
	}
	return true;
}

// Commander poll gate for the entry-leap report hold. Same preconditions as the
// config-switch dispatch it defers (a pending switch, idle control slot, no batch
// under TX, connected, no BREAK). A hold granted here is poll-scoped: it suppresses
// the dispatch, the data send AND the staging of the next batch on this poll, so the
// link stays idle while the report tail decodes and the hold is re-entered (and its
// deadline re-checked) on every following poll.
bool cl_arq_controller::entry_leap_poll_gate()
{
	entry_leap_hold_this_poll = false;
	if (opt_pending_switch_cfg >= 0
	    && opt_pending_switch_cfg != current_configuration
	    && messages_control.status == FREE && block_under_tx == NO
	    && link_status == CONNECTED && emergency_break_active == 0
	    && entry_leap_report_wait_hold())
		entry_leap_hold_this_poll = true;
	return entry_leap_hold_this_poll;
}

// WALL-B FIX-5 (fix5/FIX5_DESIGN.md §4.4): apply the CFG16 big-block carve COOLDOWN as an
// additional index-cap. When the cooldown is armed (bigblock_carve_cooldown_batches > 0)
// and `proposed` is ABOVE the cooldown ceiling (CFG15), clamp DOWN to CFG15 (the rung
// election is REFUSED); otherwise return `proposed` UNCHANGED. INDEX-MONOTONE never-raise
// clamp — composes order-independently with supershift_proven_ceiling / WB-NB ceiling /
// max_config_override (all "never raise"), so it can only LOWER a target and never
// introduces a new over-climb (audit §5.3 composition note). Off the cooldown
// (batches==0, the normal case) bigblock_carve_cooldown_ceiling returns -1 and this is the
// IDENTITY, so non-bigblock and clean-CFG16 operation is byte-identical (INV-B1 no-op).
int cl_arq_controller::apply_bigblock_cooldown_cap(int proposed) const
{
	int cd = bigblock_carve_cooldown_ceiling(bigblock_carve_cooldown_batches, robust_enabled);
	if(cd >= 0 && config_ladder_index(proposed) > config_ladder_index(cd))
		return cd;
	return proposed;
}

// CFG16 decode-margin election gate — the channel-viability verdict for the CFG16 top rung.
// Returns true iff the reverse-path SNR (measurements.SNR_uplink, the peer's report of OUR
// forward link, carried in the MFSK ACK suffix — the SAME signal the SUCCESS_BASED_LADDER climb
// already trusts) clears CFG16_MIN_SNR_DB. A decisive moderate-vs-high-SNR delivery cohort +
// cross-layer data-flow audit showed CFG16 is a STRUCTURAL loss below ~snr3k 23-24 dB (no decode
// margin -> hard BREAK -> ladder storm, 4-22x fewer delivered bytes than holding CFG15); the
// gate holds CFG15 there and admits CFG16 only where the margin is real. Fail-OPEN on the -99.9
// sentinel (no ACK suffix decoded yet) so the pre-ACK connect/climb path is byte-identical.
// MERCURY_DECODE_MARGIN_GATE_DEFEAT forces true (blanket CFG16 = the pre-gate over-election,
// the fail-before observable). Const: reads a member + a threshold, no get_configuration.
bool cl_arq_controller::cfg16_decode_margin_ok() const
{
	const char* defeat = std::getenv("MERCURY_DECODE_MARGIN_GATE_DEFEAT");
	if(defeat && *defeat && atoi(defeat) != 0) return true;   // blanket CFG16 (fail-before)
	double snr = measurements.SNR_uplink;
	if(snr <= -90.0) return true;                             // unmeasured -> byte-identical
	return snr > CFG16_MIN_SNR_DB;
}

// INDEX-MONOTONE never-raise clamp that holds a WB climb target at CFG15 while the CFG16
// decode-margin gate is closed; otherwise returns `proposed` UNCHANGED. Same shape/composition
// as apply_bigblock_cooldown_cap() (order-independent with supershift_proven_ceiling / WB-NB
// ceiling / max_config_override — all "never raise"), and applied at the SAME climb hooks so
// every CFG16-reaching producer honors one consistent ceiling. WB-only: an NB session (its own
// NB_CONFIG_MAX ceiling governs) is returned unchanged. When the gate is open (high SNR, the
// -90 sentinel, or the defeat env) this is the IDENTITY, so high-SNR CFG16 election and the
// pre-ACK path are byte-identical.
int cl_arq_controller::apply_cfg16_margin_cap(int proposed) const
{
	if(narrowband_enabled == YES) return proposed;   // WB-only gate
	if(cfg16_decode_margin_ok()) return proposed;    // gate open -> identity
	if(config_ladder_index(proposed) > config_ladder_index(CONFIG_15))
		return CONFIG_15;
	return proposed;
}

// ── Per-rung MEASURED election floor gate (generalizes apply_cfg16_margin_cap) ─────────────
// CAL VERSION 3 — a per-rung floor on the LIVE in-transfer meter grid, read through a freshness-
// qualified climb-grade SNAPSHOT (rung_meter_db), not the raw measurements.SNR_uplink. The anchor
// cohort measured the suffix meter as quantized to a ~2 dB integer grid {9,11,13,15,17,21,23,25}
// and config/trajectory-dependent (true 13->11, 14.8->13, 18->15 with a ~25% bimodal tail to 11-13,
// 24.79->17/21/23 stratified by config, 28->25).
//
// rung_floor_ok compares with a STRICT `>`, so every gated row is placed HALF-GRID BELOW the grid
// point it must admit — a row placed ON a grid point would refuse the very read it names (a 15.0 row
// admits only meter>=17). Each row therefore names an ADMIT set and is derived from the measured 8/8
// delivery floor + 1.5 dB hold margin, mapped through the live transfer:
//   cfg9  = 10.0  admits meter >= 11   (10.4+1.5 true; the wb13 read of 11 clears it, +2.6 dB margin)
//   cfg10 = 12.0  admits meter >= 13   (13.1+1.5 true)
//   cfg11 = 12.0  admits meter >= 13   (12.8+1.5 true)
//   cfg12 = 12.0  admits meter >= 13   (13.4+1.5 true)
//   cfg13 = 12.0  admits meter >= 13   (13.4+1.5 true — the 14.8 envelope rung, read 13, cfg13-hold)
//   cfg14 = 14.0  admits meter >= 15   (16.4+1.5 true — the true-18 read of 15 opens it, blocks 14.8)
//   cfg15 = 16.0  admits meter >= 17   (18.0+1.5 true — the true-24.79 cfg15-regime read of 21 opens it)
//   cfg16 = 22.0  admits meter >= 23   (== CFG16_MIN_SNR_DB, unchanged, live-meter-calibrated)
//   cfg17 = 22.0  mirrors cfg16 (top-gear, default-off). TODO raise to ~25.0 once the
//                 R7-64 BER/FER curve lands: 64-QAM's ~+3.2 dB d_min cost puts cfg17's
//                 honest decode floor a bracket above cfg16 (monotone preserved; the J0
//                 grid-placement assert pins row 16 only, so raising row 17 is safe).
//   cfg0..cfg8    ungated (-90.0): low rungs decode deep -> identity on OFDM-entry / robust re-climb.
//
// The row is MONOTONE (10,12,12,12,12,14,16,22 for cfg9..cfg16) and no gated row is a member of the
// meter grid (a J0-style grid-placement assert enforces this). The whole band is refused when the
// snapshot is stale (connect / recovery / outage), which holds the honest low config until an
// in-transfer read legitimizes the next rung; the failure memory below raises a row one bracket if a
// gated rung the table admitted breaks on a non-WGN channel. RUNG_MIN_SNR_METER[16] == CFG16_MIN_SNR_DB
// so the J0 version assert pins the calibration and the cap composition stays order-independent.
static const double RUNG_MIN_SNR_METER[NUMBER_OF_CONFIGS] = {
	/* 0*/ -90.0, /* 1*/ -90.0, /* 2*/ -90.0, /* 3*/ -90.0, /* 4*/ -90.0,
	/* 5*/ -90.0, /* 6*/ -90.0, /* 7*/ -90.0, /* 8*/ -90.0,
	/* 9*/  10.0, /*10*/  12.0, /*11*/  12.0, /*12*/  12.0, /*13*/  12.0,
	/*14*/  14.0, /*15*/  16.0, /*16*/  22.0, /*17*/  24.0
};

// CAL VERSION STEADY (4) — the EARNED cfg14/cfg15 recalibration, active ONLY under
// MERCURY_RUNG_STEADY_REFRESH (default-off => the cal-v3 table above is used, byte-identical).
// Derivation (delivery evidence, RUNGFLOOR_DEFEAT_20260816, n=16/arm at true-14.8): the live suffix
// meter reads {13,15} at 14.8; 16/16 defeat cells reached+held cfg15 and 15/16 delivered md5-verified
// full payloads at 1.25x the VARA bar with FEWER storms, while the cal-v3 gate capped cfg14 at
// meter 13 (floor 14.0, even when FRESH) and cfg15 at meter 15 (floor 16.0). The cfg15 floor 16.0
// (admit meter>=17 = the true-24.79 regime) blocked a config viable ~3 dB lower. Lower cfg14 and
// cfg15 to 12.0 so BOTH admit meter>=13 (true-14.8+) and refuse meter<=11 (true-13-) under the
// strict-`>` half-grid rule. cfg9..cfg13 and cfg16/cfg17 are UNCHANGED (the evidence does not cover
// them); RUNG_MIN_SNR_METER_STEADY[16]==CFG16_MIN_SNR_DB retains the J0 pin. Monotone + off-grid.
static const double RUNG_MIN_SNR_METER_STEADY[NUMBER_OF_CONFIGS] = {
	/* 0*/ -90.0, /* 1*/ -90.0, /* 2*/ -90.0, /* 3*/ -90.0, /* 4*/ -90.0,
	/* 5*/ -90.0, /* 6*/ -90.0, /* 7*/ -90.0, /* 8*/ -90.0,
	/* 9*/  10.0, /*10*/  12.0, /*11*/  12.0, /*12*/  12.0, /*13*/  12.0,
	/*14*/  12.0, /*15*/  12.0, /*16*/  22.0, /*17*/  24.0
};

// TRUE iff the default-off steady-state climb-grade refresh + earned cfg14/cfg15 recalibration is
// armed (MERCURY_RUNG_STEADY_REFRESH set non-zero). Read per-call to match the defeat-env pattern;
// static so rung_floor_min_meter (static) and the const gate can both consult it.
bool cl_arq_controller::rung_steady_refresh_active()
{
	const char* e = std::getenv("MERCURY_RUNG_STEADY_REFRESH");
	return (e && *e && atoi(e) != 0);
}

double cl_arq_controller::rung_floor_min_meter(int cfg)
{
	if(cfg < 0 || cfg >= NUMBER_OF_CONFIGS) return -90.0;   // robust / out-of-range -> ungated
	if(rung_steady_refresh_active()) return RUNG_MIN_SNR_METER_STEADY[cfg];   // cal STEADY (env-gated)
	return RUNG_MIN_SNR_METER[cfg];
}

// B2 TEMPORAL HYSTERESIS (default-off): the value the floor gate judges the climb-grade meter
// against. cal-v3 (env unset) => the raw latest snapshot, byte-identical to the pre-B2 gate. Under
// MERCURY_RUNG_STEADY_REFRESH => the bounded latch-max: the recent windowed peak of qualified reads
// while that peak is still within its window (rides a transient quantized down-dip below the floor
// without lowering the meter the gate sees), else the latest raw read (a sustained decline that ages
// the peak out of its window decays it back to the fresh, lower value). The peak is invalidated on a
// BREAK (re-earned after recovery), so this never carries a stale-high value across a storm.
double cl_arq_controller::rung_meter_floor_value() const
{
	if(!rung_steady_refresh_active()) return rung_meter_db;   // cal-v3: raw snapshot, byte-identical
	if(rung_meter_hold_age_batches <= RUNG_METER_HOLD_WINDOW_BATCHES
	   && rung_meter_hold_db > rung_meter_db)
		return rung_meter_hold_db;                            // windowed peak rides a transient dip
	return rung_meter_db;                                     // no valid peak above the latest -> raw
}

// V3 monotonic wall clock (steady_clock ms). Used only to stamp/age the climb-grade snapshot; the
// suffix meter's staleness in a real-audio run is real wall time (the audio clock is the HW timer).
long long cl_arq_controller::rung_meter_now_ms()
{
	return (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count();
}

// V3 freshness predicate. The climb-grade snapshot is STALE when it has no in-transfer sample yet
// (-90 sentinel), when more than N data-ACK-wait attempts have passed since the last fresh sample
// (the over-election storm's measured stale window), or when its wall age exceeds the bound (covers
// idle, where the batch counter freezes). A stale snapshot fails the gate CLOSED for climbs above
// current — the demote/BREAK path (which never consults the gate) still runs at full authority.
bool cl_arq_controller::rung_meter_stale() const
{
	if(rung_meter_db <= -90.0) return true;                            // no in-transfer sample yet
	if(rung_meter_age_batches > RUNG_METER_STALE_AGE_BATCHES) return true;
	if(rung_meter_wall_ms != 0 &&
	   rung_meter_now_ms() - rung_meter_wall_ms > RUNG_METER_STALE_WALL_MS) return true;
	return false;
}

// V3 producer qualifier. Called from the two raw SNR_uplink producers (the decoded-LDPC-frame path
// and the pattern-ACK-suffix path). Latch a fresh climb-grade sample ONLY when the read is in the
// in-transfer regime: (a) COMMANDER (the elector) on a WB link, (b) a data batch has been ACKed this
// session (first-data latch — kills the connect ~34 dB read, which precedes any data), and (c) not
// awaiting BREAK recovery (breaks_since_last_data_success==0 — kills the recovery ~25 dB read, which
// the code itself notes goes garbage-high). Non-qualifying reads leave the snapshot untouched; the
// raw measurements.SNR_uplink still updates for every existing consumer (display / recovery / the
// incumbent get_configuration inputs are unchanged — v3 only changes what the ELECTION gate reads).
void cl_arq_controller::rung_meter_note_read(double snr_value)
{
	if(role != COMMANDER) return;                       // commander-side election snapshot only
	if(narrowband_enabled == YES) return;               // WB-only gate
	if(!rung_meter_data_latched) return;                // (b) connect-regime read (no data batch yet)
	if(breaks_since_last_data_success > 0) return;      // (c) BREAK/recovery-regime garbage-high read
	snr_value = quantize_snr_decision_db(snr_value);     // explicit derived-state ingress seam
	if(snr_value <= -90.0) return;                       // sentinel guard
	rung_meter_db = snr_value;
	rung_meter_age_batches = 0;
	rung_meter_wall_ms = rung_meter_now_ms();
	// B2 TEMPORAL HYSTERESIS (default-off consumer): maintain the bounded latch-max of qualified
	// climb-grade reads. A strictly higher grid cell — or one arriving after the hold's window
	// has expired — becomes the new hold and restarts the window; a read BELOW the hold WITHIN the
	// window is a transient quantized down-dip and does NOT lower the hold (the batch clock keeps
	// aging it, so a SUSTAINED decline eventually crosses the window and the next read decays the
	// hold to that fresh, lower value). The floor gate reads this via rung_meter_floor_value(); cal-v3
	// never consults it, so this write is behavior-neutral when the env is unset.
	// Equality is not new peak evidence: resetting on an equal cell lets raw
	// strict declines collapsed by publication quantization extend the hold.
	if(snr_value > rung_meter_hold_db || rung_meter_hold_age_batches > RUNG_METER_HOLD_WINDOW_BATCHES)
	{
		rung_meter_hold_db = snr_value;
		rung_meter_hold_age_batches = 0;
	}
	if(snr_track_feature_enabled())
		snr_track_note_meter_read();
}

// V3 age tick. Called once per data-ACK-wait entry (clear_snr_arm_for_data_ack_wait sites — the
// cmd_batch_tx_done boundary). A batch attempt that does NOT latch a fresh climb-grade read ages the
// snapshot; the last fresh read resets the age to 0 (rung_meter_note_read). Saturates so it cannot
// wrap. WB commander only (matches the gate).
void cl_arq_controller::rung_meter_note_batch_attempt()
{
	if(role != COMMANDER) return;
	if(rung_meter_age_batches < 1000000000) rung_meter_age_batches++;
	// B2 (default-off consumer): the clean-batch clock ages alongside the SNR-read clock. It is
	// reset to 0 ONLY by a clean fully-acked forward batch (rung_floor_note_clean site), so it
	// distinguishes a HEALTHY hold (clean confirms every batch -> stays small) from an OUTAGE /
	// idle (no clean confirm -> grows). The healthy-hold exemption reads it; cal-v3 ignores it.
	if(rung_meter_clean_age < 1000000000) rung_meter_clean_age++;
	// B2 (default-off consumer): the latch-max window ages alongside the SNR-read clock. rung_meter_
	// floor_value() trusts the held peak only while this stays <= RUNG_METER_HOLD_WINDOW_BATCHES.
	if(rung_meter_hold_age_batches < 1000000000) rung_meter_hold_age_batches++;
}

// V3 session reset — re-arm the connect-regime latch and clear the snapshot at each session boundary
// (init() + reset_session_state()), so a fresh session starts in the connect regime (stale -> the
// gate holds the low config until the first in-transfer read).
void cl_arq_controller::rung_meter_reset()
{
	rung_meter_db = -99.9;
	rung_meter_age_batches = 1000000000;
	rung_meter_wall_ms = 0;
	rung_meter_data_latched = false;
	rung_meter_clean_age = 1000000000;         // B2: no clean batch yet this session
	rung_meter_healthy_hold_batches = 0;       // B2: no healthy-hold streak yet
	rung_meter_hold_db = -99.9;                // B2: no latch-max peak yet this session
	rung_meter_hold_age_batches = 1000000000;
	snr_track_meter_window_reset();
}

void cl_arq_controller::snr_track_meter_window_reset()
{
	snr_track_prev_meter_cap = CONFIG_NONE;
	snr_track_sustained_cap = CONFIG_NONE;
	snr_track_fresh_batches = 0;
}

// Record an actual fresh suffix-meter read. The rolling minimum of two reads rejects an
// alternating 9/15 dB MPG peak; a 2 dB margin on cfg14 and above also prevents one held
// 15 dB read from licensing the unsustainable peak rung.
void cl_arq_controller::snr_track_note_meter_read()
{
	int sample_cap = WB_CONFIG_MAX;
	while(sample_cap >= CONFIG_9
	      && !(rung_meter_db > rung_floor_min_meter(sample_cap)
	           + rung_floor_bump_db[sample_cap]
	           + (sample_cap >= CONFIG_14 ? SNR_TRACK_PEAK_MARGIN_DB : 0.0)))
		sample_cap = config_ladder_down(sample_cap, robust_enabled);

	if(snr_track_fresh_batches == 0 || snr_track_prev_meter_cap == CONFIG_NONE)
	{
		snr_track_fresh_batches = 1;
		snr_track_sustained_cap = sample_cap;
	}
	else
	{
		snr_track_fresh_batches = SNR_TRACK_SUSTAINED_BATCHES;
		snr_track_sustained_cap =
			config_ladder_index(sample_cap) < config_ladder_index(snr_track_prev_meter_cap)
			? sample_cap : snr_track_prev_meter_cap;
	}
	snr_track_prev_meter_cap = sample_cap;
}

// A clean data batch may confirm one fresh held read while it remains inside the normal
// freshness window. It cannot start a window from a cached value; partial or stale data
// breaks the confirmation run.
void cl_arq_controller::snr_track_note_meter_batch(bool clean_batch)
{
	if(!clean_batch || rung_meter_stale())
	{
		snr_track_meter_window_reset();
		return;
	}
	if(snr_track_fresh_batches == 1)
		snr_track_fresh_batches = SNR_TRACK_SUSTAINED_BATCHES;
}

int cl_arq_controller::snr_track_sustained_target() const
{
	if(rung_meter_stale() || snr_track_fresh_batches < SNR_TRACK_SUSTAINED_BATCHES)
		return CONFIG_NONE;
	return snr_track_sustained_cap;
}

int cl_arq_controller::snr_track_apply_climb_cap(int proposed) const
{
	if(!rung_meter_data_latched)
		return proposed;
	int cap = snr_track_sustained_target();
	if(cap == CONFIG_NONE)
		cap = config_ladder_index(current_configuration) < config_ladder_index(CONFIG_8)
			? CONFIG_8 : current_configuration;
	else if(config_ladder_index(cap) < config_ladder_index(current_configuration))
		cap = current_configuration;
	if(config_ladder_index(proposed) > config_ladder_index(cap))
		return cap;
	return proposed;
}

// V3 floor+freshness check WITHOUT the at/below-current shortcut. The shortcut in rung_floor_ok (a
// hold/down-move is never gated) references current_configuration; the BREAK-recovery anchor clamp
// (T11) instead references the raw demote target, so it needs the raise-only check. Fail-OPEN on the
// defeat env / NB / cfg<CONFIG_9 / out-of-range / cfg17; fail-CLOSED on a stale snapshot; else the
// qualified snapshot must clear the rung's meter-referred floor + any session bump.
bool cl_arq_controller::rung_floor_meter_clears(int cfg) const
{
	const char* defeat = std::getenv("MERCURY_RUNG_FLOOR_GATE_DEFEAT");
	if(defeat && *defeat && atoi(defeat) != 0) return true;   // blanket admit (fail-before)
	if(narrowband_enabled == YES) return true;                // WB-only gate
	if(cfg < CONFIG_9) return true;                           // low OFDM rungs ungated (decode deep)
	if(cfg < 0 || cfg >= NUMBER_OF_CONFIGS) return true;      // out of range (robust) -> ungated
	if(cfg == CONFIG_17)
	{
		// BLOCKER A — row-17 election floor (MERCURY_ROW17_BUMP). Default-off =>
		// returns true (ungated, byte-identical). cfg17 is a FORWARD-decode config: its
		// 64-QAM viability is a DOWNLINK property, already measured by the top-gear guard's
		// un-clipped SNR_downlink floor verdict (topgear_cfg17_floor_ok, packed as report
		// flat_state 11 by the responder). The REVERSE uplink suffix meter (rung_meter_db,
		// clamped at 25) is the WRONG instrument here and OVER-BLOCKS: at a winning forward
		// anchor (delivered ~24, SNR_downlink ~25.67) the reverse meter can read a grid 23,
		// and 23 > 24 is false, so the old reverse gate refused the very cfg17 the forward
		// guard has earned (a saturated 25 read was the only reverse value that admitted).
		// Gate on the FORWARD verdict instead; it defaults false (fail-closed) until a clean
		// forward report clears the guard floor, so a stale/unmeasured forward channel still
		// refuses. RUNG_MIN_SNR_METER[17] (=24.0) is retained as the documented row + J0 pin.
		// Arm the guard (MERCURY_CFG17_SNR_FLOOR) alongside ROW17_BUMP: without a forward-floor
		// report the responder never packs code 11, so cfg17 stays fail-closed by design.
		const char* r17 = std::getenv("MERCURY_ROW17_BUMP");
		if(!(r17 && atoi(r17) != 0)) return true;              // default: ungated
		const char* floor = std::getenv("MERCURY_CFG17_SNR_FLOOR");
		if(!(floor && *floor && atof(floor) > 0.0)) return false;
		return topgear_forward_report_is_fresh();
	}
	if(rung_meter_stale())
	{
		// ── B2 HEALTHY-HOLD FRESHNESS EXEMPTION (MERCURY_RUNG_STEADY_REFRESH, default-off) ─────
		// The stale gate closes climbs during an OUTAGE (a rung frozen high after its ACKs stop
		// decoding). It cannot, alone, distinguish that from a HEALTHY forward HOLD, where the
		// suffix meter simply has no steady-state refresh producer. During a demonstrably healthy
		// hold — a real last-known read, no active BREAK recovery, clean forward batches STILL
		// confirming (clean-age fresh, the anti-outage + activity guard that subsumes any idle/wall
		// test), and a SUSTAINED streak of them — a snapshot stale by AGE is allowed to admit a
		// BOUNDED one-rung climb, provided the last-known read STILL CLEARS the target floor. The
		// climb changes current_configuration (restarting the one-rung window) and, via the
		// turnaround, triggers a fresh SNR read, so it cannot ratchet multiple rungs on one stale
		// sample. Storm-safe: when ACKs stop, clean-age grows past the window and the streak resets
		// on the first break, so the exemption disarms and the gate fails closed exactly as cal-v3.
		if(rung_steady_refresh_active()
		   && rung_meter_db > -90.0                                                     // real last-known read
		   && breaks_since_last_data_success == 0                                       // no active recovery
		   && rung_meter_clean_age <= RUNG_METER_STALE_AGE_BATCHES                      // clean confirms still flowing
		   && rung_meter_healthy_hold_batches >= RUNG_STEADY_HEALTHY_HOLD_BATCHES       // sustained health
		   && config_ladder_index(cfg) == config_ladder_index(current_configuration) + 1)  // BOUNDED one rung
		{
			double xbump = rung_floor_bump_db[cfg];
			return rung_meter_floor_value() > rung_floor_min_meter(cfg) + xbump;   // windowed max must still clear the floor
		}
		return false;                                        // stale climb-grade snapshot -> fail closed
	}
	double bump = rung_floor_bump_db[cfg];
	return rung_meter_floor_value() > rung_floor_min_meter(cfg) + bump;  // qualified snapshot / latch-max (cal-v3 or STEADY)
}

// TRUE iff the QUALIFIED climb-grade snapshot admits OFDM rung `cfg` at its meter-referred floor + any
// session bump. v3 layers an at/below-current shortcut on top of rung_floor_meter_clears: a hold or a
// down-move (cfg at/below current_configuration) is structurally NEVER gated, so a stale snapshot only
// ever HOLDS current — it can never block a demote / BREAK / panic (the af14a9e bound). A climb above
// current is admitted only on a fresh snapshot that clears the floor. Const.
bool cl_arq_controller::rung_floor_ok(int cfg) const
{
	if(config_ladder_index(cfg) <= config_ladder_index(current_configuration))
		return true;                                          // hold / down-move -> never gated
	return rung_floor_meter_clears(cfg);
}

// INDEX-MONOTONE never-raise clamp: walk a proposed climb target DOWN to the highest floor-cleared
// rung (floored at the ladder bottom). Same shape/composition as apply_cfg16_margin_cap /
// apply_bigblock_cooldown_cap (order-independent, can only LOWER a target). Identity when the gate
// is open (fast path). Const (no side effects); the ELECTION sites use rung_floor_cap_fire, which
// counts + logs; the predicate sites (ceiling-recovery / ladder-up) call THIS directly.
int cl_arq_controller::apply_rung_floor_cap(int proposed) const
{
	if(narrowband_enabled == YES) return proposed;           // WB-only gate
	if(rung_floor_ok(proposed)) return proposed;             // gate open -> identity
	int t = proposed;
	for(int i = 0; i < NUMBER_OF_CONFIGS; i++)
	{
		int down = config_ladder_down(t, robust_enabled);
		if(down == t) break;                                 // at the ladder bottom
		t = down;
		if(rung_floor_ok(t)) return t;                       // highest floor-cleared rung <= proposed
	}
	return t;
}

// V3 RAISE-only never-raise clamp for the BREAK-recovery anchor clamp (T11). Same walk as
// apply_rung_floor_cap but uses rung_floor_meter_clears (NO at/below-current shortcut) because the
// clamp raises the raw demote target UP toward the last data-viable rung, and at the clamp site
// current_configuration is still the FAILING rung — the shortcut would spuriously admit the raise.
// Walks `proposed` down to the highest fresh-meter-cleared rung; const, order-independent.
int cl_arq_controller::apply_rung_floor_raise_cap(int proposed) const
{
	if(narrowband_enabled == YES) return proposed;           // WB-only gate
	if(rung_floor_meter_clears(proposed)) return proposed;   // gate open -> identity
	int t = proposed;
	for(int i = 0; i < NUMBER_OF_CONFIGS; i++)
	{
		int down = config_ladder_down(t, robust_enabled);
		if(down == t) break;                                 // at the ladder bottom
		t = down;
		if(rung_floor_meter_clears(t)) return t;             // highest fresh-cleared rung <= proposed
	}
	return t;
}

// Counting wrapper for the ELECTION (transform) producer sites: apply the cap and, on a non-identity
// clamp, bump the run-lifetime floor_cap_fires counter + emit the fire-proof line (counted-at-source).
// The fire line reports the QUALIFIED snapshot (the actual gate input in cal v3) + its stale flag.
int cl_arq_controller::rung_floor_cap_fire(int proposed)
{
	int capped = apply_rung_floor_cap(proposed);
	if(capped != proposed)
	{
		floor_cap_fires++;
		printf("[GEARSHIFT] RUNG-FLOOR: capped %d -> %d (meter %.1f hold %.1f holdage %d age %d stale %d, floor %.1f, fires=%lld)\n",
			proposed, capped, rung_meter_db, rung_meter_hold_db, rung_meter_hold_age_batches,
			rung_meter_age_batches, rung_meter_stale() ? 1 : 0,
			rung_floor_min_meter(proposed), floor_cap_fires);
		fflush(stdout);
	}
	else if(rung_steady_refresh_active()
	        && config_ladder_index(proposed) > config_ladder_index(current_configuration))
	{
		// B2 LATCH-MAX WIRE WITNESS (default-off): this climb was ADMITTED. Check whether the windowed
		// peak — not the raw latest snapshot — is what cleared the floor (the raw read alone would have
		// capped it). That is the temporal-hysteresis fix FIRING: a transient quantized dip below the
		// floor did not cap a viable climb. Counted-at-source so the cohort res JSON carries the count.
		double fl = rung_floor_min_meter(proposed) + rung_floor_bump_db[proposed];
		double fv = rung_meter_floor_value();
		if(rung_meter_db <= fl && fv > fl)
		{
			floor_hold_admits++;
			printf("[GEARSHIFT] RUNG-FLOOR-HOLD: admitted %d (raw meter %.1f <= floor %.1f; windowed hold %.1f holdage %d cleared it) hold_admits=%lld\n",
				proposed, rung_meter_db, fl, fv, rung_meter_hold_age_batches, floor_hold_admits);
			fflush(stdout);
		}
	}
	return capped;
}

// PER-RUNG FAILURE MEMORY — arm. A BREAK is committing at `cfg`. If `cfg` is a gated OFDM rung the
// table ADMITTED (rung_floor_ok held), that is evidence the static floor is optimistic for THIS
// channel (fading / meter bias / non-WGN): count it; at K consecutive such at-rung BREAKs, raise
// `cfg`'s session floor one bracket. WB-only, inert under the defeat env. Writes ONLY the failure-
// memory arrays (never supershift_proven_ceiling / breaks_since_last_data_success / the anchor), so
// the af14a9e ratchet + panic bypass are byte-identical (audit §4).
void cl_arq_controller::rung_floor_note_break(int cfg)
{
	const char* defeat = std::getenv("MERCURY_RUNG_FLOOR_GATE_DEFEAT");
	if(defeat && *defeat && atoi(defeat) != 0) return;       // fail-before: no memory
	if(narrowband_enabled == YES) return;                    // WB-only
	if(cfg < CONFIG_9 || cfg > CONFIG_15) return;            // only gated OFDM rungs carry a bump
	if(!rung_floor_ok(cfg)) return;                          // arm only when the table ADMITTED cfg
	rung_fail_count[cfg]++;
	if(rung_fail_count[cfg] >= RUNG_FLOOR_BUMP_ARM_FAILS)
	{
		if(rung_floor_bump_db[cfg] < RUNG_FLOOR_BUMP_MAX_DB)
		{
			rung_floor_bump_db[cfg] += RUNG_FLOOR_BUMP_STEP_DB;
			printf("[GEARSHIFT] RUNG-FLOOR MEMORY: cfg%d disproven at meter %.1f (%d at-rung BREAKs) "
				"-> floor bump +%.1f (now +%.1f)\n", cfg, measurements.SNR_uplink,
				RUNG_FLOOR_BUMP_ARM_FAILS, RUNG_FLOOR_BUMP_STEP_DB, rung_floor_bump_db[cfg]);
			fflush(stdout);
		}
		rung_fail_count[cfg] = 0;
	}
}

// PER-RUNG FAILURE MEMORY — decay. A CLEAN fully-delivered batch at `cfg` proves the current rung
// recovered (reset ITS fail streak). For each bumped rung R: if we are sustaining delivery BELOW R,
// accumulate a clean streak and, after DECAY_CLEAN batches, relax R's bump one bracket (cooldown);
// operating at/above R re-tests it and resets its streak. WB-only, inert under the defeat env. This
// decay is INDEPENDENT of the :6898 breaks_since_last_data_success reset -> a slow completion at the
// anchor does not wipe the disproof (audit §4, erasure site 2).
void cl_arq_controller::rung_floor_note_clean(int cfg)
{
	const char* defeat = std::getenv("MERCURY_RUNG_FLOOR_GATE_DEFEAT");
	if(defeat && *defeat && atoi(defeat) != 0) return;
	if(narrowband_enabled == YES) return;
	if(cfg >= CONFIG_9 && cfg <= CONFIG_15) rung_fail_count[cfg] = 0;   // a rung that recovers is not disproven
	int ci = config_ladder_index(cfg);
	for(int R = CONFIG_9; R <= CONFIG_15; R++)
	{
		if(rung_floor_bump_db[R] <= 0.0) { rung_bump_clean_streak[R] = 0; continue; }
		if(ci >= 0 && ci < config_ladder_index(R))            // sustaining delivery BELOW the disproven rung
		{
			rung_bump_clean_streak[R]++;
			if(rung_bump_clean_streak[R] >= RUNG_FLOOR_BUMP_DECAY_CLEAN)
			{
				rung_floor_bump_db[R] -= RUNG_FLOOR_BUMP_STEP_DB;
				if(rung_floor_bump_db[R] < 0.0) rung_floor_bump_db[R] = 0.0;
				rung_bump_clean_streak[R] = 0;
				printf("[GEARSHIFT] RUNG-FLOOR MEMORY: cfg%d bump decayed to +%.1f after %d clean batches below\n",
					R, rung_floor_bump_db[R], RUNG_FLOOR_BUMP_DECAY_CLEAN);
				fflush(stdout);
			}
		}
		else
		{
			rung_bump_clean_streak[R] = 0;                    // at/above R -> re-testing it
		}
	}
}

// Zero the per-rung failure memory. Called at init() and commander_clean_reconnect() (the two seats
// that reset the gearshift counters). floor_cap_fires is a run-lifetime diagnostic reset at init() only.
void cl_arq_controller::rung_floor_memory_reset()
{
	for(int i = 0; i < NUMBER_OF_CONFIGS; i++)
	{
		rung_fail_count[i] = 0;
		rung_floor_bump_db[i] = 0.0;
		rung_bump_clean_streak[i] = 0;
	}
}

// CONNECT-SEED of the START config (gearshift-start-and-recovery.md §10.2) — PURE
// core. See the arq.h header for the contract. Returns a WB OFDM start config
// STRICTLY above init_cfg ONLY when the channel is clearly clean, else CONFIG_NONE
// (the no-over-seed guard -> start ROBUST_0 byte-identical). The caller has already
// gated feature/gearshift; this enforces the SNR-confidence + cap chain.
int cl_arq_controller::connect_seed_target_core(double snr_uplink, int snr_mapped,
	int init_cfg, int proven_ceil, bool nb)
{
	// GUARD 0 — SNR sentinel: an un-populated SNR (ctor -99.9) is NOT a clean read.
	if(snr_uplink <= -90.0)
		return CONFIG_NONE;
	// GUARD 1 — minimum confidence: the margin-mapped OFDM config must clear the
	// CONNECT_SEED_CONFIG_MIN floor. At the deep-SNR cliff the control-plane suffix
	// SNR (~1 dB) minus CONNECT_SEED_MARGIN_DB maps BELOW this -> no seed (the
	// control-vs-data over-report cannot plant a WB start on a marginal channel).
	if(config_ladder_index(snr_mapped) < config_ladder_index(CONNECT_SEED_CONFIG_MIN))
		return CONFIG_NONE;
	// Conservative cap chain (never-raise; same family as the elevator's caps):
	int target = snr_mapped;
	// (a) the mid-ladder ceiling — one connect read must not plant the top config.
	if(config_ladder_index(target) > config_ladder_index(CONNECT_SEED_CONFIG_CAP))
		target = CONNECT_SEED_CONFIG_CAP;
	// (b) NB ceiling (an NB session cannot exceed NB_CONFIG_MAX).
	if(nb && config_ladder_index(target) > config_ladder_index(NB_CONFIG_MAX))
		target = NB_CONFIG_MAX;
	// (c) proven ceiling from any prior BREAK failures this session.
	if(proven_ceil >= 0 && config_ladder_index(target) > config_ladder_index(proven_ceil))
		target = proven_ceil;
	// GUARD 2 — only a STRICT climb above the un-seeded start is a seed. If the cap
	// chain pulled the target back to/below init_cfg, there is nothing to seed.
	if(config_ladder_index(target) <= config_ladder_index(init_cfg))
		return CONFIG_NONE;
	return target;
}

// CONNECT-SEED member wrapper (gearshift-start-and-recovery.md §10.2). Gates on the
// in-band feature + gearshift + a valid control-plane SNR, maps the SNR through
// get_configuration() with the large CONNECT_SEED_MARGIN_DB, applies the shared
// bigblock cooldown cap, then defers the confidence/cap/strict-climb decision to
// the pure core. Returns the seed config or CONFIG_NONE (no seed -> start ROBUST_0).
int cl_arq_controller::connect_seed_target()
{
	// Gearshift-v2 ACTIVE performs data-plane acquisition itself; do not let
	// the legacy control-plane SNR seed become a second normal authority.
	if(rate_opt.controls_link()) return CONFIG_NONE;
#ifdef CONNECT_SEED_FAILBEFORE
	// FAIL-BEFORE arm: the start is NEVER seeded (the pre-fix crawl-from-ROBUST_0
	// behaviour). The directed test's pass-after (a clean channel seeds a WB start)
	// then FAILS, proving the seed is load-bearing.
	return CONFIG_NONE;
#else
	// QUARANTINE (gearshift-start-and-recovery.md §10.7, faithful real-audio A/B
	// 2026-06-28): the live CONNECT-SEED is DEFAULT-OFF. It is correct that the seed
	// now FIRES (7e348d5f fixed the NB-start bypass) and it WINS on clean channels
	// (WGN:40/30: time-to-cross 127s->91s, climb-past-cfg0 0/6->6/6, 7-13x bytes).
	// BUT the OVER-SEED GUARD IS UNSOUND on the MFSK control-plane SNR proxy: that
	// SNR reads ~32 dB at WGN:10 and ~37 dB at WGN:15 (vs ~53-60 dB at WGN:30/40),
	// so the fixed CONNECT_SEED_MARGIN_DB=9.0 cannot separate a marginal channel from
	// a clean one — both seed (capped) to CONFIG_8. On WGN:10/15 the seeded CONFIG_8
	// CANNOT be carried: the session thrashes BREAK and delivers 0 bytes vs the
	// monitor baseline's 53-182 (TOTAL DATA LOSS on marginal HF — unacceptable for
	// life-critical). A fixed-dB margin on this proxy is a THRESHOLD BAND-AID (a
	// symptom-mask, not a root fix); the real fix needs a data-plane viability signal, not a
	// control-plane SNR guess. DO NOT default-enable until §10.7 is resolved. Opt-in for
	// continued investigation only: MERCURY_CONNECT_SEED=1.
	{
		const char* cs = getenv("MERCURY_CONNECT_SEED");
		if(!(cs && cs[0] == '1'))
			return CONFIG_NONE;
	}
	if(!inband_rate_feature_enabled())
		return CONFIG_NONE;
	if(gear_shift_on != YES || gear_shift_algorithm != SUCCESS_BASED_LADDER)
		return CONFIG_NONE;
	if(measurements.SNR_uplink <= -90.0)
		return CONFIG_NONE;
	int snr_mapped = get_configuration(measurements.SNR_uplink - CONNECT_SEED_MARGIN_DB);
	// Fold the bigblock carve cooldown cap (shared chokepoint) BEFORE the core so a
	// suspended-CFG16 cooldown also caps the seed (composes order-independently).
	snr_mapped = apply_bigblock_cooldown_cap(snr_mapped);
	// CFG16 decode-margin gate: the connect control-plane SNR proxy over-reads, so one seed
	// read must never plant CFG16 unless the reverse-path SNR clears the margin (never-raise).
	snr_mapped = apply_cfg16_margin_cap(snr_mapped);
	// Per-rung floor gate on the CONNECT seed: one control-plane SNR read must not seed a START
	// config above its measured floor (never-raise; identity pre-ACK / low rung / high SNR / NB).
	snr_mapped = rung_floor_cap_fire(snr_mapped);
	int seed = connect_seed_target_core(measurements.SNR_uplink, snr_mapped,
		init_configuration, supershift_proven_ceiling, narrowband_enabled == YES);
	if(seed != CONFIG_NONE)
	{
		printf("[GEARSHIFT] CONNECT-SEED: clean channel (control SNR=%.1f dB, "
			"mapped@-%.1fdB=%d) -> seed START config %d -> %d (vs crawl-from-%d); "
			"routes via SET_CONFIG tier-cross\n",
			measurements.SNR_uplink, CONNECT_SEED_MARGIN_DB, snr_mapped,
			init_configuration, seed, init_configuration);
		fflush(stdout);
	}
	return seed;
#endif
}

// Latch the COMMANDER's HAIL response and anchor the timers whose budgets begin
// with the OFDM handshake.  hail_detected is the per-attempt one-shot guard:
// retries may poll this helper again, but cannot move either anchor a second time.
void cl_arq_controller::accept_hail_detection()
{
	if(hail_detected == YES)
		return;

	hail_detected = YES;

	// The outer connection budget historically started at CONNECT.  Opt in to
	// excluding config-independent, variable-duration HAIL acquisition from it.
	const char* reanchor = std::getenv("MERCURY_CONNECT_TIMER_REANCHOR");
	if(reanchor && *reanchor && atoi(reanchor) != 0)
	{
		connection_attempt_timer.reset();
		connection_attempt_timer.start();
	}

	// CONNECT-FAST-CONFIG: anchor the SHORT fast-attempt budget HERE, at
	// HAIL-detected — the instant the CONFIG_0 OFDM handshake begins. HAIL
	// acquisition is config-INDEPENDENT (same for fast and robust) and its
	// duration is variable (channel + host-load dependent), so it must NOT
	// count against the fast budget: anchoring at the CONNECT command let a
	// slow HAIL acquisition consume the whole budget and revert a handshake
	// that was actually succeeding. Started once (counting==0 guard); the
	// revert clears connect_fast_active so it never re-arms.
	if(connect_fast_active && connect_fast_timer.counting == 0)
	{
		connect_fast_timer.reset();
		connect_fast_timer.start();
	}
}


int cl_arq_controller::queue_break_recovery_set_config()
{
	// Force-clear: cleanup() skips PENDING_ACK status.
	messages_control.status = FREE;
	if(test_break_set_config_status_after_clear >= 0)
		messages_control.status = test_break_set_config_status_after_clear;

	const int result = add_message_control(SET_CONFIG);
	if(result != SUCCESSFUL)
	{
		printf("[BREAK] ERROR: SET_CONFIG enqueue rejected (control status=%d)\n",
			(int)messages_control.status);
		fflush(stdout);
		return result;
	}

	printf("[BREAK] SET_CONFIG queued: data[1]=%d data[2]=%d\n",
		(int)messages_control.data[1], (int)messages_control.data[2]);
	fflush(stdout);
	connection_status = TRANSMITTING_CONTROL;
	link_timer.start();
	watchdog_timer.start();
	return SUCCESSFUL;
}

bool cl_arq_controller::gearshift_v2_finish_break_at_floor()
{
	if(!rate_opt.controls_link()) return false;

	// A v2-authorized hard recovery has exactly one policy outcome: re-synchronize
	// both peers at the robust floor, preserve/re-stage application bytes, and let
	// Gearshift-v2 perform every subsequent upward acquisition. The historical BREAK
	// recovery ladder is transport policy from another controller and is not allowed
	// to choose a target while ACTIVE owns Axis-1.
	const int floor_cfg = robust_enabled ? ROBUST_0 : CONFIG_0;
	printf("[GEARSHIFT-V2-AUTHORITY] BREAK transport complete: settling at floor CONFIG_%d; "
		"legacy BREAK ladder disabled, v2 will reacquire upward\n", floor_cfg);
	fflush(stdout);

	emergency_break_active = 0;
	emergency_nack_count = 0;
	emergency_break_retries = 0;
	break_recovery_phase = 0;
	break_recovery_retries = 0;
	break_drop_step = 0;

	messages_control_backup();
	data_configuration = floor_cfg;
	negotiated_configuration = floor_cfg;
	forward_configuration = floor_cfg;
	reverse_configuration = floor_cfg;
	load_configuration(floor_cfg, PHYSICAL_LAYER_ONLY, YES);
	messages_control_restore();

	if(compression_enabled)
		restore_tx_from_compressed();
	else
	{
		restage_requeue_tx_messages();
		fifo_buffer_backup.flush();
		clear_retx_queue();
	}
	block_under_tx = NO;
	opt_reset_window();
	connection_status = TRANSMITTING_DATA;
	watchdog_timer.start();
	link_timer.start();
	return true;
}

void cl_arq_controller::commander_handle_connected_disconnect()
{
	// Graceful disconnect (VARA-compatible): wait for TX FIFO to drain before
	// closing the link. Winlink sends FQ data then DISCONNECT; deliver FQ first.
	int fifo_pending = fifo_buffer_tx.get_size() - fifo_buffer_tx.get_free_size();
	if(role_demand_switch_owns_precedence())
	{
		// A validated demand is older causal authority than this request. Complete
		// the already-earned transfer; the promoted peer becomes sole commander.
		static int demand_disconnect_wait_prints = 0;
		if(demand_disconnect_wait_prints++ % 50 == 0)
		{
			printf("[DISCONNECT] acknowledged reverse demand owns precedence; completing SWITCH_ROLE\n");
			fflush(stdout);
		}
	}
	else if(fifo_pending > 0 || block_under_tx == YES)
	{
		static int disconnect_wait_prints = 0;
		if(disconnect_wait_prints++ % 50 == 0)
		{
			printf("[DISCONNECT] Waiting for TX drain: FIFO=%d bytes, block_under_tx=%d\n",
				fifo_pending, block_under_tx);
			fflush(stdout);
		}
	}
	else if(!role_demand_control_slot_safe())
	{
		// Never overwrite a sent or ACK-pending control record. The request stays
		// latched and is retried after the one-deep mailbox becomes FREE.
		printf("[DISCONNECT] control mailbox busy status=%d code=%u; CLOSE deferred\n",
			messages_control.status,
			messages_control.data != NULL
				? (unsigned)(unsigned char)messages_control.data[0] : 0u);
		fflush(stdout);
	}
	else
	{
		printf("[DISCONNECT] TX drained, sending CLOSE_CONNECTION\n");
		fflush(stdout);
		disconnect_requested=NO;
		this->link_status=DISCONNECTING;
		// Terminal-settlement convergence: arm the bounded settle deadline. The
		// retry train composes here and owns lost/duplicate CLOSE behavior.
		disconnecting_settle_timer.stop();
		disconnecting_settle_timer.reset();
		disconnecting_settle_timer.start();
		add_message_control(CLOSE_CONNECTION);
	}
}

void cl_arq_controller::process_messages_commander()
{
	// Poll-scoped: only a hold granted by entry_leap_poll_gate() on THIS poll may
	// suppress new-batch staging in process_buffer_data_commander().
	entry_leap_hold_this_poll = false;
	// ACTIVE-v2 single-owner purge: turboshift is a legacy Axis-1 controller, not
	// a sensor. It contains direct load_configuration() settle paths that can bypass
	// SET_CONFIG/CONFIG_TAG entirely, so merely fencing the transport is insufficient.
	// Once the session is CONNECTED, retire the legacy state machine completely.
	// Connection/capability negotiation before CONNECTED is untouched.
	if(rate_opt.controls_link() && link_status == CONNECTED && turboshift_active)
	{
		printf("[GEARSHIFT-V2-AUTHORITY] retiring legacy TURBOSHIFT controller; "
			"ACTIVE owns all connected Axis-1 decisions\n");
		fflush(stdout);
		turboshift_active = false;
		turbo_supershift_announce_pending = false;
		turboshift_phase = TURBO_DONE;
		turboshift_retries = 0;
		turbo_switch_role_retries = 0;
		gear_shift_timer.stop();
		gear_shift_timer.reset();

		// If turboshift had already staged an unowned SET_CONFIG at the exact
		// CONNECTED boundary, cancel it. Never touch a matching v2-owned crossing.
		if(messages_control.status != FREE && messages_control.data != NULL
		   && messages_control.length > 0
		   && (unsigned char)messages_control.data[0] == (unsigned char)SET_CONFIG)
		{
			const int staged_target = messages_control.length > 1
				? (unsigned char)messages_control.data[1] : CONFIG_NONE;
			if(!rate_opt.transition_matches(current_configuration, staged_target))
			{
				printf("[GEARSHIFT-V2-AUTHORITY] cancelling staged legacy TURBOSHIFT "
					"SET_CONFIG %d->%d at ownership handoff\n",
					current_configuration, staged_target);
				fflush(stdout);
				messages_control.ack_timeout = 0;
				messages_control.id = 0;
				messages_control.length = 0;
				messages_control.nResends = 0;
				messages_control.status = FREE;
				messages_control.type = NONE;
				if(connection_status == TRANSMITTING_CONTROL ||
				   connection_status == RECEIVING_ACKS_CONTROL)
					connection_status = TRANSMITTING_DATA;
			}
		}
	}

	// Topgear deferred-report driver (consume-race fix): drive any pending
	// stashed-tail report decode once per poll. First-line early-return unless
	// MERCURY_TOPGEAR_ELECT armed a pending — the default path does not move.
	topgear_pending_report_tick();

	// In-band CONNECT-LIVENESS GUARD (data-flow-inband-connect-liveness.md §2). Once per
	// poll, observe forward-DATA progress; if a connect/negotiate handshake is livelocked
	// (no nAcked_data advance for N control-plane polls) fire the retained true-loss BREAK.
	// Gated ON inband (byte-identical OFF). Skip while a BREAK is already recovering — the
	// emergency-break state machine below owns the resync; do not stack a second BREAK.
	if(!emergency_break_active && inband_connect_liveness_guard())
		return;   // recovery fired; the emergency-break SM serves the next poll

	// Emergency BREAK state machine: poll for ACK after sending BREAK pattern
	if(emergency_break_active)
	{
		if(disconnect_requested==YES)
		{
			emergency_break_active=0;
			emergency_nack_count=0;
			// Fall through to normal disconnect handling below
		}
		else
		{
		// RECOVERY-ACK robustness (recovery-ack-robustness.md §4): the BREAK-ACK
		// (RSP confirming it heard the BREAK, arq_responder.cc:486) is a recovery
		// control-ACK turnaround — combine the SAME R base reps the RSP emits.
		// Gated MERCURY_RECOVERY_ACK_ROBUST; default-off → reps=1 → byte-identical.
		set_recovery_ack_reps_for_wait(/*control_ack=*/true);
		if(receiving_timer.get_elapsed_time_ms() < receiving_timeout)
		{
			if(break_ack_poll())
			{
				if(gearshift_v2_finish_break_at_floor())
					return;
				// Use ROBUST_0 as coordination layer, then probe target config.
				// Phase 1: send SET_CONFIG at ROBUST_0 (guaranteed delivery).
				// Phase 2: send SET_CONFIG at target to verify it works (2 tries).
				int raw_target = config_ladder_down_n(emergency_previous_config, break_drop_step, robust_enabled);
				// Option B (data-anchored promotion): floor at last_data_viable_config
				// (bypassed under panic). See break_target_with_anchor() / §6/§7.
				// Option B anchor clamp (may RAISE raw_target up to the last data-viable rung).
				int clamped = break_target_with_anchor(raw_target);
				// V3 T11: a STALE/garbage-high meter must not re-raise recovery to a rung that just
				// failed (the frozen-25 recovery loop). Cap the RAISE at a fresh-meter-cleared rung,
				// floored at the demote's own raw_target so the demote is never undercut — the result
				// can never go below raw_target, nor above a floor+freshness-cleared rung. Byte-identical
				// when the anchor is fresh-cleared or under the panic bypass (clamped==raw_target).
				int t11_capped = apply_rung_floor_raise_cap(clamped);
				int target = (config_ladder_index(t11_capped) < config_ladder_index(raw_target))
					? raw_target : t11_capped;
				int unwelded_target = target;
				target = break_weld_target(target);
				if(target != unwelded_target)
				{
					printf("[BREAK-WELD] recovery target %d -> pinned config %d\n",
						unwelded_target, target);
					fflush(stdout);
				}
				if(break_raise_cap_log_needed(raw_target, clamped, unwelded_target, target))
				{
					printf("[BREAK] RUNG-FLOOR raise-cap: anchor %d -> %d (meter %.1f age %d stale %d)\n",
						clamped, unwelded_target, rung_meter_db, rung_meter_age_batches, rung_meter_stale() ? 1 : 0);
					fflush(stdout);
				}
				if(break_anchor_floor_log_needed(raw_target, clamped, unwelded_target, target))
				{
					printf("[BREAK] Anchor floor: target %d below last_data_viable_config %d — clamping up to %d\n",
						raw_target, last_data_viable_config, clamped);
					fflush(stdout);
				}
				printf("[BREAK] ACK received! Dropping %d step(s): config %d -> %d (robust_enabled=%d)\n",
					break_drop_step, emergency_previous_config, target, robust_enabled);
				fflush(stdout);
				// WALL-B FIX-2: multiplicative-increase half of the AARF, now
				// CAPPED at the OFDM ladder span (BREAK_DROP_STEP_MAX=16) so one
				// recovery can never skip the whole OFDM span into ROBUST_0. The
				// panic crater (=100, :3608) is set at a separate site and consumed
				// before this doubling, so the cap never blocks the panic jump.
				break_drop_step = break_drop_step_after_double(break_drop_step);

				emergency_break_active = 0;
				emergency_nack_count = 0;
				break_recovery_phase = 1;
				break_recovery_retries = 2;

				int robust_0 = robust_enabled ? ROBUST_0 : CONFIG_0;
				messages_control_backup();
				data_configuration = target;
				load_configuration(robust_0, PHYSICAL_LAYER_ONLY, YES);
				messages_control_restore();

				negotiated_configuration = target;
				forward_configuration = target;
				if(reverse_configuration == CONFIG_NONE)
					reverse_configuration = target;

				printf("[BREAK] Recovery phase 1: negotiated=%d forward=%d reverse=%d "
					"data_cfg=%d current=%d target=%d\n",
					negotiated_configuration, forward_configuration,
					reverse_configuration, data_configuration,
					current_configuration, target);
				fflush(stdout);

				{
					if(compression_enabled)
					{
						// Decompress messages_tx back to raw, push to FIFO
						// for re-compression at new config.
						restore_tx_from_compressed();
					}
					else
					{
						// Re-stage through the ordered, capacity-checked funnel.
						restage_requeue_tx_messages();
						fifo_buffer_backup.flush();
						clear_retx_queue();  // R029: recovery (non-compressed) re-queues plaintext; drop stale retx
					}
					block_under_tx = NO;
					int fifo_load = fifo_buffer_tx.get_size() - fifo_buffer_tx.get_free_size();
					printf("[BREAK] Saved data to FIFO (%d bytes total)\n", fifo_load);
					fflush(stdout);
				}

				if(queue_break_recovery_set_config() != SUCCESSFUL)
					return;
			}
		}
		else
		{
			// Timeout — retry BREAK
			l1_apply_reset_event(mercury::L1ResetEvent::BREAK_RECOVERY_TIMEOUT);
			emergency_break_retries--;
			if(emergency_break_retries > 0)
			{
				printf("[BREAK] Retry (%d left)\n", emergency_break_retries);
				fflush(stdout);
				send_break_pattern();
				telecom_system->data_container.frames_to_read = 4;
				calculate_receiving_timeout();
				receiving_timer.start();
			}
			else
			{
				printf("[BREAK] All retries exhausted — assuming responder already at ROBUST_0\n");
				printf("[BREAK] EXHAUSTED state: emergency_prev=%d break_drop=%d robust=%d\n",
					emergency_previous_config, break_drop_step, robust_enabled);
				fflush(stdout);
				if(gearshift_v2_finish_break_at_floor())
					return;
				emergency_break_active = 0;
				emergency_nack_count = 0;
				break_recovery_phase = 1;
				break_recovery_retries = 2;

				int raw_target = config_ladder_down_n(emergency_previous_config, break_drop_step, robust_enabled);
				// Option B (data-anchored promotion): same anchor floor as the
				// ACK-received recovery site above — never undercut the highest
				// data-viable rung, except under the panic-jump. See §6/§7.
				// Option B anchor clamp + V3 T11 raise-cap (same shape as the ACK-received site
				// above): the anchor clamp may raise raw_target, but a stale/garbage meter cannot
				// re-raise recovery to a failing rung; floored at raw_target so the demote is never
				// undercut. Byte-identical when the anchor is fresh-cleared or under the panic bypass.
				int clamped = break_target_with_anchor(raw_target);
				int t11_capped = apply_rung_floor_raise_cap(clamped);
				int target = (config_ladder_index(t11_capped) < config_ladder_index(raw_target))
					? raw_target : t11_capped;
				int unwelded_target = target;
				target = break_weld_target(target);
				if(target != unwelded_target)
				{
					printf("[BREAK-WELD] exhausted recovery target %d -> pinned config %d\n",
						unwelded_target, target);
					fflush(stdout);
				}
				if(break_raise_cap_log_needed(raw_target, clamped, unwelded_target, target))
				{
					printf("[BREAK] RUNG-FLOOR raise-cap (exhausted): anchor %d -> %d (meter %.1f age %d stale %d)\n",
						clamped, unwelded_target, rung_meter_db, rung_meter_age_batches, rung_meter_stale() ? 1 : 0);
					fflush(stdout);
				}
				if(break_anchor_floor_log_needed(raw_target, clamped, unwelded_target, target))
				{
					printf("[BREAK] Anchor floor (exhausted): target %d below last_data_viable_config %d — clamping up to %d\n",
						raw_target, last_data_viable_config, clamped);
					fflush(stdout);
				}
				printf("[BREAK] Dropping %d step(s): config %d -> %d\n",
					break_drop_step, emergency_previous_config, target);
				fflush(stdout);
				// WALL-B FIX-2: CAPPED doubling — see ACK-received site above.
				break_drop_step = break_drop_step_after_double(break_drop_step);

				int robust_0 = robust_enabled ? ROBUST_0 : CONFIG_0;
				messages_control_backup();
				data_configuration = target;
				load_configuration(robust_0, PHYSICAL_LAYER_ONLY, YES);
				messages_control_restore();

				negotiated_configuration = target;
				forward_configuration = target;
				if(reverse_configuration == CONFIG_NONE)
					reverse_configuration = target;

				printf("[BREAK] EXHAUSTED recovery: negotiated=%d forward=%d data_cfg=%d target=%d\n",
					negotiated_configuration, forward_configuration, data_configuration, target);
				fflush(stdout);

				if(compression_enabled)
				{
					restore_tx_from_compressed();
				}
				else
				{
					// Re-stage through the ordered, capacity-checked funnel.
					restage_requeue_tx_messages();
					fifo_buffer_backup.flush();
					clear_retx_queue();  // R029: recovery (non-compressed) re-queues plaintext; drop stale retx
				}
				block_under_tx = NO;

				// IDLE-SWITCHROLE-RACE RECOVERY PATH (Part C, idle-switchrole-race.md
				// §3/§5.3/§5.4). The watchdog never disconnects (arq_common.cc:2985
				// re-arms itself), so a never-fed link spins BREAK->SET_CONFIG->
				// EXHAUSTED forever; the only legacy escape is the 180 s FORCED
				// fallback. Discriminate the DEAD loop (no RX data this session AND
				// tx FIFO empty AND a zero-byte re-queue = nothing came in and
				// nothing is going out) from a LIVE recovery (any RX data, or a
				// non-empty re-queue = real bytes still in flight). On a dead cycle
				// count it; on ANY progress RESET to 0 (negative control). At bound
				// K STOP re-arming the watchdog and route into the SAME FORCED-
				// fallback teardown (arq_common.cc:3053-3065) instead of re-queuing
				// SET_CONFIG — a graceful disconnect/return-to-listen in ~K cycles
				// rather than the 180 s wall.
				int requeue_fifo_load = fifo_buffer_tx.get_size() - fifo_buffer_tx.get_free_size();
				bool no_progress = (session_data_frame_received == false) &&
				                   (requeue_fifo_load == 0) &&
				                   (block_under_tx == NO);
				// Shared PURE kernel (common_defines.h) — mutates break_noprogress_cycles
				// and returns whether bound K is reached. The unit test replays this same
				// kernel so production and test cannot diverge.
				bool noprogress_teardown = break_noprogress_step(break_noprogress_cycles, no_progress);
				if(no_progress)
				{
					printf("[BREAK] NO-PROGRESS cycle %d/%d (no RX data, empty TX, zero-byte re-queue)\n",
						break_noprogress_cycles, BREAK_NOPROGRESS_TEARDOWN_K);
					fflush(stdout);
				}
				else if(requeue_fifo_load > 0 || session_data_frame_received)
				{
					// Real forward progress (RX data delivered OR bytes re-queued):
					// a LIVE BREAK recovery — the kernel already reset the streak.
					printf("[BREAK] progress observed (rx_data=%d requeue=%dB) — no-progress streak reset\n",
						session_data_frame_received ? 1 : 0, requeue_fifo_load);
					fflush(stdout);
				}

				if(noprogress_teardown)
				{
					printf("[BREAK] NO-PROGRESS teardown: %d consecutive dead BREAK cycles — "
						"forcing graceful disconnect/return-to-listen\n", break_noprogress_cycles);
					fflush(stdout);
					// Route into the SAME post-state as the 180 s FORCED_ROLE_SWITCH
					// fallback (arq_common.cc:3053-3065): clean session reset, drop to
					// RESPONDER/LISTENING, reload init config, flush FIFOs. The watchdog
					// is NOT re-armed (reset_all_timers stops it), so the spiral ends.
					reset_session_state();
					set_role(RESPONDER);
					link_status = LISTENING;
					connection_status = RECEIVING;
					load_configuration(init_configuration, FULL, YES);
					reset_all_timers();
					fifo_buffer_tx.flush();
					fifo_buffer_backup.flush();
					fifo_buffer_rx.flush();
					messages_control.status = FREE;
				}
				else
				{
					// Force-clear: cleanup() skips PENDING_ACK status
					messages_control.status = FREE;
					add_message_control(SET_CONFIG);
					printf("[BREAK] EXHAUSTED SET_CONFIG queued: data[1]=%d data[2]=%d\n",
						(int)messages_control.data[1], (int)messages_control.data[2]);
					fflush(stdout);
					connection_status = TRANSMITTING_CONTROL;
					link_timer.start();
					watchdog_timer.start();
				}
			}
		}
		return;
		} // else (not disconnect_requested)
	}

	if(this->link_status==CONNECTING)
	{
		// NB/WB auto-negotiation Phase 1: WB commander switches to NB
		// to probe for NB responders (first nb_probe_max attempts)
		if(connection_attempts == 0 && messages_control.status == FREE &&
		   commander_configured_nb >= 0 && commander_configured_nb != YES &&
		   nb_probe_max > 0)
		{
			printf("[NB-NEG] Commander: Phase 1 NB probe (WB commander)\n");
			fflush(stdout);
			switch_narrowband_mode(YES);
		}

		// NB probe switch-back: after nb_probe_max HAIL attempts in NB,
		// switch to WB so HAIL can reach the WB responder.
		// The START_CONNECTION retry path (line ~468) has its own switch-back,
		// but it's never reached when HAIL fails — HAIL failure prevents
		// START_CONNECTION from being queued.
		if(commander_configured_nb >= 0 && commander_configured_nb != YES &&
		   nb_probe_max > 0 && connection_attempts >= nb_probe_max &&
		   narrowband_enabled == YES)
		{
			printf("[NB-NEG] Commander: restoring WB after %d NB HAIL attempts\n",
				connection_attempts);
			fflush(stdout);
			switch_narrowband_mode(NO);
			// hail_detected is already NO — WB HAIL will be attempted below
		}

		// "I am Mercury" HAIL phase: fast MFSK beacons replace slow LDPC probes.
		// Send HAIL, listen for response. Repeat up to max_connection_attempts.
		// Only proceed to START_CONNECTION after HAIL response detected.
		// Directed HAIL: CRC suffix derived from responder's callsign prevents
		// multi-station collisions — only the target responder responds.
		if(ack_pattern_time_ms > 0 && hail_detected == NO)
		{
			telecom_system->ack_mfsk.set_hail_target(
				destination_call_sign.c_str(), destination_call_sign.length());
			send_hail_pattern();
			connection_attempts++;
			printf("[HAIL] Sent beacon %d of %d\n", connection_attempts, max_connection_attempts);
			fflush(stdout);

			// Listen for response (pattern time + responder turnaround)
			int listen_ms = 2 * ack_pattern_time_ms + 1500;
			cl_timer hail_listen;
			hail_listen.start();
			while(hail_listen.get_elapsed_time_ms() < listen_ms)
			{
				if(receive_hail_pattern())
				{
					printf("[HAIL] Response received — peer is Mercury\n");
					fflush(stdout);
					accept_hail_detection();
					break;
				}
				// §5.7-B8 + CLOCK-FIDELITY FIX (fix/sim-connect-virtual-clock):
				// the poll cadence MUST run on the same clock as this loop's
				// hail_listen deadline (get_elapsed_time_ms(), a virtual cl_timer
				// read under -x sim / SIM_INPROC). pumped_settle_wait gives that on
				// ALL three paths: production -> verbatim wall msleep(50); two-
				// process -x sim -> a 50ms VIRTUAL-clock spin (the concurrent RX
				// bridge advances the shared sample clock, so the deadline advances
				// toward listen_ms and the peer's HAIL reply flows into RX before
				// the next receive_hail_pattern() check — immune to host CPU load,
				// closing the connect-under-load race); SIM_INPROC -> a 50ms
				// step-pumped virtual wait. A bare wall msleep(50) here desynced
				// the wall poll from the virtual deadline under load and dropped
				// the RSP beacon outside the window. The outer hail_listen deadline
				// + the receive_hail_pattern() early-exit are UNCHANGED.
				pumped_settle_wait(50);
			}

			if(hail_detected == NO)
				return; // Retry next cycle (max_connection_attempts check in update_status)
		}

		add_message_control(START_CONNECTION);
	}
	else if(this->link_status==CONNECTION_ACCEPTED)
	{
		add_message_control(TEST_CONNECTION);
	}
	else if (this->link_status==NEGOTIATING)
	{
		add_message_control(SET_CONFIG);
	}

	if(disconnect_requested==YES)
	{
		if(this->link_status==CONNECTED)
		{
			commander_handle_connected_disconnect();
		}
		else
		{
			reset_session_state();
			load_configuration(init_configuration,FULL,YES);

			// Switch to RESPONDER/LISTENING so we can receive incoming connections
			set_role(RESPONDER);
			this->link_status=LISTENING;
			this->connection_status=RECEIVING;

			reset_all_timers();
			// Reset RX state machine - wait for fresh data (prevents decode of self-received TX audio)
			telecom_system->data_container.frames_to_read =
				telecom_system->data_container.preamble_nSymb + telecom_system->data_container.Nsymb;
			telecom_system->data_container.nUnder_processing_events = 0;

			fifo_buffer_tx.flush();
			fifo_buffer_backup.flush();
			fifo_buffer_rx.flush();

			// Reset messages_control so new CONNECT commands can work
			messages_control.status=FREE;

			emit_disconnected_once();
		}
	}


	if(this->connection_status==TRANSMITTING_CONTROL)
	{
		if(g_verbose) print_stats();
		process_messages_tx_control();
	}
	else if(this->connection_status==RECEIVING_ACKS_CONTROL)
	{
		process_messages_rx_acks_control();
	}
	else if(this->connection_status==TRANSMITTING_DATA)
	{
		// Key exchange gate: if encryption negotiated but not active, run key exchange first
		if(encryption_enabled && !cipher_suite.is_active() && cipher_suite.get_kx_phase() == KX_IDLE)
		{
			if(!kx_as_data_path())
			{
				printf("[CRYPTO] Turboshift done, initiating key exchange\n");
				fflush(stdout);
				add_message_control(KEY_EXCHANGE_1);
#ifdef MERCURY_GUI_ENABLED
				// Display-only: surface KX start so STRICT does not look hung.
				gui_set_kx_progress(cipher_suite.get_kx_phase(), 0, 0);
#endif
				// connection_status set to TRANSMITTING_CONTROL by add_message_control
				return;
			}
			// KX-AS-DATA (data-flow-hybrid-kex.md §2.2): the reserved forward stream
			// (x25519_pk_cmd + mlkem_pk) is staged + fed as DATA batches by
			// process_buffer_data_commander() — NOT the legacy one-shot KEY_EXCHANGE_1
			// control frame (which deadlocks at ROBUST's tiny geometry). Fall through to
			// the hold below; only the KX stream flows before activation, user data held.
		}
		// Hold user data until key exchange completes (encryption negotiated but not yet
		// active). EXCEPTION (KX-as-data): while the reserved forward KX stream is active,
		// allow the data-send path (process_messages_tx_data) to run so the staged KX
		// batches reach the wire and drive the gearshift climb. User data is still held —
		// only the KX stream (staged in kx_stream_tx_buf, never fifo_buffer_tx) flows
		// before activation. Env-gated OFF (kx_stream_tx_active never set) -> unchanged.
		if(encryption_enabled && !cipher_suite.is_active() && !kx_stream_tx_active)
		{
			// KX-as-data: the source-switch fills the WHOLE forward/reverse stream into
			// messages_tx[] in ONE pass, clearing kx_stream_tx_active BEFORE the batch is
			// even sent. Do NOT re-engage the hold while a KX batch is staged / in flight /
			// awaiting retransmit -> let process_messages_tx_data() drain it to the wire and
			// collect ACKs (otherwise the staged forward stream never reaches the RSP and
			// the handshake stalls). During the KX epoch messages_tx[] holds ONLY KX frames
			// (user data stays in fifo_buffer_tx; process_buffer_data_commander's KX branch
			// never pops it), so this is safe. Env off -> kx_as_data_path() false -> the hold
			// is byte-identical.
			bool kx_batch_in_flight = kx_as_data_path()
				&& (block_under_tx == YES || get_nOccupied_messages() > 0 || retransmit_count > 0);
			if(!kx_batch_in_flight)
				return;
		}

		// Phase 3c — consume any pending optimizer-recommended config
		// switch BEFORE starting the next data batch. Re-verify all the
		// hard gates: turboshift may have re-armed and BREAK may have
		// fired since the recommendation was made. If still eligible,
		// queue SET_CONFIG; the standard ACK handler then loads the new
		// config (same pattern as SUPERSHIFT at line ~3193 + turbo path
		// at line ~2785).

		// -- Top-gear (CONFIG_17, 64-QAM) channel-clean election producer -----------------
		// (topgear-stack-productionize.md 3; mirrors big-block clean-election 931a0c73.)
		// A DEDICATED SET_CONFIG producer for the 64-QAM top rung that OWNS cfg17, sharing
		// the optimizer producer's precondition gate (turbo/break/CONNECTED/OFDM/idle):
		//   ENGAGE   : election held clean+flat at CONFIG_16 -> step CONFIG_16 -> CONFIG_17.
		//   DISENGAGE: verdict dropped while live at CONFIG_17 -> demote 17 -> CONFIG_16.
		//   HELD     : engaged + already at CONFIG_17 -> SUPPRESS the general optimizer
		//              producer below (it caps at WB_CONFIG_MAX=16 and would demote-thrash
		//              the top rung - the pull-back the effective-ceiling cannot reach here).
		// The election is the SOLE producer of a cfg17 SET_CONFIG (SNR map/optimizer/climb
		// all cap at 16). No-op unless MERCURY_TOPGEAR_ELECT is set => byte-identical.
		// SCOPE: cfg17 election is STRUCTURALLY REFUSED when MERCURY_INBAND_RATE is on. The
		// in-band CONFIG_TAG codec does not carry cfg17's synthetic ladder index; the legacy
		// SET_CONFIG handshake packs the
		// RAW config and negotiates cfg17 correctly - topgear-stack-productionize.md 5).
		if (!rate_opt.controls_link()
		    && topgear_elect_feature_enabled()
		    && !inband_rate_feature_enabled()   // structural refusal: cfg17 has no in-band CONFIG_TAG ladder index
		    && !turboshift_active
		    && emergency_break_active == 0
		    && link_status == CONNECTED
		    && is_ofdm_config(current_configuration)
		    && block_under_tx == NO
		    && messages_control.status == FREE)
		{
			int tg_target = -1;
			if (topgear_elect_engaged && current_configuration == CONFIG_16
			    && (max_config_override < 0 || max_config_override >= CONFIG_17)
			    && rung_floor_ok(CONFIG_17))   // BLOCKER A: row-17 floor gate (MERCURY_ROW17_BUMP; identity when off)
			{
				tg_target = CONFIG_17;   // ENGAGE: climb the top rung
			}
			else if (!topgear_elect_engaged && current_configuration == CONFIG_17)
			{
				tg_target = CONFIG_16;   // DISENGAGE: demote to the validated cfg16 rung
			}
			else if (topgear_elect_engaged && current_configuration == CONFIG_17)
			{
				// HELD at the top rung - OWN it: cancel any general optimizer proposal that
				// would clamp to WB_CONFIG_MAX (16) and pull the top rung back down.
				opt_pending_switch_cfg = -1;
				opt_pending_switch_action = GEARSHIFT_ACTION_HOLD;
				opt_pending_switch_fallback = -1;
			}
			if (tg_target >= 0)
			{
				printf("[TOPGEAR] queue SET_CONFIG: %d -> %d (channel-clean election; engaged=%d "
				       "flatness=%.3f snr_dl=%.1f streak=%d)\n",
				       current_configuration, tg_target, topgear_elect_engaged ? 1 : 0,
				       topgear_channel_flatness, measurements.SNR_downlink, topgear_elect_clean_streak);
				fflush(stdout);
				negotiated_configuration = tg_target;
				// Same climb-churn bsi rollback the other SET_CONFIG climb producers use, so a
				// partial-SACK in-flight batch re-presents contiguously at the new config.
				roll_back_cmd_bsi_to_inflight("TOPGEAR");
				cleanup();
				add_message_control(SET_CONFIG);
				connection_status = TRANSMITTING_CONTROL;
				return;
			}
		}

		if (entry_leap_poll_gate())
			return;   // hold this poll; the deferred report tick runs first next poll
		if (opt_pending_switch_cfg >= 0
		    && opt_pending_switch_cfg != current_configuration
		    && (!turboshift_active || rate_opt.controls_link())
		    && emergency_break_active == 0
		    && link_status == CONNECTED
		    && (is_ofdm_config(current_configuration) || rate_opt.controls_link())
		    && block_under_tx == NO
		    && messages_control.status == FREE)
		{
			const int from_cfg = current_configuration;
			int target = opt_pending_switch_cfg;
			const int mode_ceiling = (narrowband_enabled == YES) ? NB_CONFIG_MAX :
				(rate_opt.controls_link()
				 ? (topgear_elect_feature_enabled() ? CONFIG_17 : WB_CONFIG_MAX)
				 : topgear_wb_ceiling());
			// ACTIVE v2 is the sole ordinary Axis-1 authority.  The legacy topgear
			// election may not veto a cfg17 action that v2 has explicitly priced and
			// admitted; only the administrative MERCURY_TOPGEAR_ELECT opt-in remains.
			// OFDM admission ceilings are not numeric ceilings on the robust tier
			// (ROBUST_* intentionally use IDs 100+).  Apply OFDM-only climb caps
			// only to an OFDM destination; robust 0/1/2 retain ladder ordering.
			if (is_ofdm_config(target))
			{
				if (target > mode_ceiling) target = mode_ceiling;
				if (max_config_override >= 0 && target > max_config_override)
					target = max_config_override;
				target = apply_bigblock_cooldown_cap(target);
				if (!rate_opt.controls_link())
				{
					// Legacy-only empirical SNR floors. ACTIVE v2 prices the
					// uncertainty/goodput directly and must be able to disprove stale
					// thresholds with bounded probes.
					target = apply_cfg16_margin_cap(target);
					target = rung_floor_cap_fire(target);
				}
			}
			const bool v2_target_ok = is_ofdm_config(target) ||
				(is_robust_config(target) && !is_robust3_config(target));
			if (target != current_configuration && v2_target_ok)
			{
				const char* move_reason =
					(opt_pending_switch_action == GEARSHIFT_ACTION_PROBE) ? "gearshift_v2_probe" :
					(opt_pending_switch_action == GEARSHIFT_ACTION_ROLLBACK) ? "gearshift_v2_rollback" :
					"gearshift_optimizer";
				printf("[GEARSHIFT-V2] queue SET_CONFIG: %d -> %d action=%d fallback=%d\n",
				       from_cfg, target, (int)opt_pending_switch_action,
				       opt_pending_switch_fallback);
				fflush(stdout);
				policy_axis1_supremacy_on_move(from_cfg, target, move_reason);
				negotiated_configuration = target;
				roll_back_cmd_bsi_to_inflight("GEARSHIFT-V2");
				cleanup();
				// Cross-axis control-slot arbitration (per-batch config data-flow audit):
				// policy_axis1_supremacy_on_move() above transitions Axis-3 ON/OFF -> PROBE and
				// queues a best-effort SET_LINK_PARAMS into the single control slot. cleanup()
				// does not free an ADDED_TO_LIST frame, so add_message_control(SET_CONFIG) below
				// would find the slot busy and silently return ERROR_ -- leaving switch_inflight
				// set with NO Axis-1 transition on the wire (the cold-start climb then hangs the
				// full switch-inflight timeout and demotes to robust). The Axis-1 transition is
				// load-bearing and takes the slot; the SET_LINK_PARAMS is recovered by the next
				// SET_LINK_PARAMS cycle / the EOB-self-correct safety net. Same arbitration as
				// inband_route_failure_demote().
				if(messages_control.status != FREE && messages_control.data != NULL
				   && messages_control.length > 0
				   && messages_control.data[0] == SET_LINK_PARAMS)
				{
					messages_control.status = FREE;
					messages_control.type   = NONE;
				}
				rate_opt.notify_switch_dispatched(from_cfg, target,
					opt_pending_switch_action, opt_pending_switch_fallback, opt_now_ms(),
					narrowband_enabled == YES);
				add_message_control(SET_CONFIG);
				opt_reset_window();
				connection_status = TRANSMITTING_CONTROL;
				opt_pending_switch_cfg = -1;
				opt_pending_switch_action = GEARSHIFT_ACTION_HOLD;
				opt_pending_switch_fallback = -1;
				return;
			}
			rate_opt.notify_switch_failed();
			opt_pending_switch_cfg = -1;
			opt_pending_switch_action = GEARSHIFT_ACTION_HOLD;
			opt_pending_switch_fallback = -1;
		}

		if(g_verbose) print_stats();
		process_messages_tx_data();
	}
	else if(this->connection_status==RECEIVING_ACKS_DATA)
	{
		process_messages_rx_acks_data();
	}
}

int cl_arq_controller::add_message_control(char code)
{
	int success=ERROR_;
	if (messages_control.status==FREE)
	{
		messages_control.type=CONTROL;
		messages_control.nResends=this->nResends;
		messages_control.ack_timeout=this->ack_timeout_control;
		messages_control.status=ADDED_TO_LIST;

		if(code==START_CONNECTION)
		{
			messages_control.data[0]=code;
			// CRC8 on full destination callsign (including SSID if present)
			messages_control.data[1]=CRC8_calc((char*)destination_call_sign.c_str(), destination_call_sign.length());
			// Pack base callsign (strip SSID — pack only supports A-Z, 0-9, 6 chars)
			// SSID is sent separately in TEST_CONNECTION
			std::string base_call = callsign_strip_ssid(my_call_sign);
			int pack_flags = 0;
			if (narrowband_enabled == YES || commander_configured_nb == YES)
				pack_flags |= 0x01;
			callsign_pack(base_call.c_str(), base_call.length(), &messages_control.data[2], pack_flags);
			messages_control.length=7;  // cmd(1) + CRC8(1) + packed_callsign(5)
			messages_control.id=0;
			connection_id=BROADCAST_ID;
		}
		else if(code==TEST_CONNECTION)
		{
			u_SNR tmp_SNR;
			tmp_SNR.f_SNR=(float)measurements.SNR_uplink;

			messages_control.data[0]=code;
			for(int i=0;i<4;i++)
			{
				messages_control.data[i+1]=tmp_SNR.char4_SNR[i];;
			}
			messages_control.data[5]=(char)local_capability;
			messages_control.data[6]=(char)callsign_get_ssid(my_call_sign);
			messages_control.length=7;
			if(t1_calibration_wire_enabled())
			{
				t1_calibration_encode(&messages_control.data[7], ptt_on_delay_ms,
					0, ptt_off_delay_ms);
				messages_control.length=12;
			}
			messages_control.id=0;
		}
		else if(code==TEST_CONNECTION_ACK)
		{
			// v9 handshake echo. RSP-initiated LDPC reply to CMD's TEST_CONNECTION.
			// Echoes back the cap byte RSP parsed from CMD's TEST_CONNECTION +
			// RSP's own caps + CRC8. CMD verifies before completing handshake.
			messages_control.data[0]=code;
			messages_control.data[1]=(char)peer_capability;   // echo CMD's caps
			messages_control.data[2]=(char)local_capability;  // RSP's own caps
			messages_control.data[3]=(char)CRC8_calc(
				(char*)&messages_control.data[1], 2);
			messages_control.length=4;
			messages_control.id=0;
		}
		else if(code==SWITCH_BANDWIDTH)
		{
			messages_control.data[0]=code;
			messages_control.data[1]=0;  // 0 = switch to WB
			messages_control.length=2;
			messages_control.id=0;
			// Fast fail: nb_only responders silently reject (don't ACK).
			// 2 retries keeps detection under ~10 seconds on MFSK modes.
			messages_control.nResends=2;
		}
		else if(code==SET_CONFIG)
		{
			// Defensive null-guard for synthetic test mode (matches the
			// SET_LINK_PARAMS pattern below). messages_control.data is
			// allocated lazily in init_messages_buffers(); the synthetic
			// --test-policy-axis1-fire entry point skips ARQ.init() so
			// messages_control.data is still NULL when add_message_control
			// is called from policy_evaluate_axis1's cleanup+SET_CONFIG path.
			if(messages_control.data == NULL)
			{
				int peek_fwd = negotiated_configuration;
				int peek_rev = (reverse_configuration == CONFIG_NONE)
					? peek_fwd : reverse_configuration;
				printf("[CMD-SET-CONFIG] SKIP TX: messages_control.data is NULL "
					"(pre-init synthetic test mode; no real wire frame). "
					"WOULD HAVE SENT: forward=%d reverse=%d\n",
					peek_fwd, peek_rev);
				fflush(stdout);
				messages_control.status = FREE;
				messages_control.type = NONE;
				return success;
			}
			// ACTIVE-v2 authority firewall. Determine the requested forward target once,
			// then require Gearshift ownership before either CONFIG_TAG or legacy SET_CONFIG
			// transport is allowed to move Axis-1. Normal v2 moves already carry a matching
			// notify_switch_dispatched() transaction; legacy safety producers can only REQUEST
			// a downward move, which authorize_external_transition() may own or reject.
			int v2_owned_target = (gear_shift_algorithm==SNR_BASED)
				? get_configuration(measurements.SNR_downlink)
				: negotiated_configuration;
			if(rate_opt.controls_link() && link_status == CONNECTED
			   && v2_owned_target != current_configuration)
			{
				int authorized = rate_opt.authorize_external_transition(
					current_configuration, v2_owned_target, "set-config-chokepoint",
					opt_now_ms(), narrowband_enabled == YES);
				if(authorized < 0)
				{
					printf("[GEARSHIFT-V2-AUTHORITY] BLOCK SET_CONFIG transport %d->%d: no owner\n",
						current_configuration, v2_owned_target);
					fflush(stdout);
					messages_control.status = FREE;
					messages_control.type = NONE;
					return ERROR_;
				}
				v2_owned_target = authorized;
				negotiated_configuration = authorized;
			}

			// STAGE 3b GEARSHIFT DRIVE (data-flow-perbatch-config.md §3.1 / §12 W3):
			// Gearshift-v2 ACTIVE uses CONFIG_TAG transport canonically; legacy selectors
			// use it when MERCURY_INBAND_RATE is set. The selected rate change is
			// announced by a passband CONFIG_TAG on the next batch (W1 emit) and followed unilaterally
			// (W2), NOT by a SET_CONFIG control handshake. EVERY gearshift/optimizer/
			// demote/BREAK producer funnels through THIS builder (the §3.1 chokepoint),
			// so intercepting here catches them all with one change. The target is the
			// gearshift's chosen forward config: negotiated_configuration for the ladder
			// path (the producers set it before calling us), or get_configuration(SNR)
			// for the SNR-based path. On a successful unilateral drop we DO NOT queue a
			// control frame: free the slot, arm the one-shot re-route flag (consumed in
			// process_messages_tx_control to put connection_status back to
			// TRANSMITTING_DATA — neutralising the caller's forced TRANSMITTING_CONTROL),
			// and return. Flag-off / no-op target -> fall through to the legacy builder
			// (byte-identical). Placed AFTER the messages_control.data null-guard above.
			if(inband_rate_feature_enabled())
			{
				int inband_target = rate_opt.controls_link()
					? v2_owned_target
					: ((gear_shift_algorithm==SNR_BASED)
						? get_configuration(measurements.SNR_downlink)
						: negotiated_configuration);
				// SET_CONFIG is legacy and is reserved here for two transport exceptions:
				//   1. CROSS_TIER: robust<->OFDM cannot use a common data/SACK PHY while
				//      changing tiers; its dedicated ACK provides the acquisition bridge.
				//   2. TAG_UNAVAILABLE: NB/M<16 has no robust M=16 CONFIG_TAG carrier.
				// Every ordinary intra-tier GS2 probe, switch, rollback, and coast-down
				// takes the unilateral CONFIG_TAG path below. Its transition remains live
				// until config-discriminating peer evidence confirms or rejects it.
				bool tier_crossing = inband_config_change_is_tier_crossing(inband_target);
				bool tag_transport_unavailable = narrowband_enabled == YES
					|| telecom_system == NULL
					|| telecom_system->ack_mfsk.ack_sack_suffix_len() <= 0;
				if(tier_crossing)
				{
					printf("[GEARSHIFT-V2] SET_CONFIG SPECIAL_CASE=CROSS_TIER %d -> %d: "
						"dedicated ACK bridges robust<->OFDM acquisition\n",
						current_configuration, inband_target);
					fflush(stdout);
				}
				else if(tag_transport_unavailable)
				{
					printf("[GEARSHIFT-V2] SET_CONFIG SPECIAL_CASE=TAG_UNAVAILABLE %d -> %d: "
						"NB/M<16 exposes no CONFIG_TAG carrier\n",
						current_configuration, inband_target);
					fflush(stdout);
				}
				else if(inband_unilateral_config_change(inband_target))
				{
					// Unilateral path took it: NO control frame on the wire.
					messages_control.status = FREE;
					messages_control.type   = NONE;
					inband_unilateral_armed = true;   // consumed in process_messages_tx_control
					success = SUCCESSFUL;
					return success;
				}
				// The two enumerated exceptions fall through to the legacy builder.
				// A no-op/invalid request also falls through but is not an Axis-1 move.
			}

			messages_control.data[0]=code;
			messages_control.id=0;

			if(rate_opt.controls_link())
			{
				forward_configuration = v2_owned_target;
				if(reverse_configuration == CONFIG_NONE)
					reverse_configuration = forward_configuration;
			}
			else if(gear_shift_algorithm==SNR_BASED)
			{
				forward_configuration = get_configuration(measurements.SNR_downlink);
				reverse_configuration = get_configuration(measurements.SNR_uplink);
			}
			else
			{
				// SUCCESS_BASED_LADDER: asymmetric — only update forward (TX direction)
				forward_configuration = negotiated_configuration;
				// IN-BAND TIER-CROSSING REVERSE-ACK PIN (data-flow-inband-tier-crossing.md
				// §3, the cross-oscillation root). VERIFIED off-bench (_pipe_ab/redesign360):
				// under MERCURY_INBAND_RATE=1 the intra-ROBUST climb (100->101->102) rides
				// the in-band CONFIG_TAG (unilateral, NO SET_CONFIG), so reverse_configuration
				// is NEVER seeded to a robust rung and sits at its ctor sentinel CONFIG_NONE.
				// At the robust->OFDM tier-cross the hybrid routes HERE (legacy SET_CONFIG
				// builder); the generic fallthrough below would then set reverse to
				// forward_configuration = the FORWARD OFDM rung (0). The reverse SACK suffix
				// must then decode on OFDM-0 in a tight turnaround and TIMES OUT
				// ("suffix timeout") on every cross -> the BREAK SM clamps back to ROBUST_2
				// -> 102<->0 oscillation, capped at CONFIG_0, 35 BREAKs. Legacy NEVER hits
				// this: it pins reverse=101 (ROBUST_1) for the WHOLE climb (its SET_CONFIG
				// climb seeds reverse robust at the first step), so its reverse SACK decodes
				// on a rock-solid MFSK rung across the cross (legacy log: forward=0 reverse=101).
				// FIX: on an in-band tier-cross, pin reverse to the ROBUST side of the
				// crossing (the robust rung mirrors legacy's reverse-robust hold) instead of
				// letting it fall to the OFDM forward rung. The robust side is the live config
				// on a robust->OFDM up-cross (current_configuration, the rung that JUST carried
				// data reliably) or the target on an OFDM->robust down-cross. Strictly gated on
				// inband_rate_feature_enabled() AND inband_config_change_is_tier_crossing ->
				// flag-off and intra-tier are byte-identical (the legacy seed below is untouched).
				int robust_reverse = inband_tier_cross_reverse_config(
					current_configuration, forward_configuration,
					inband_rate_feature_enabled());
				if(robust_reverse != CONFIG_NONE
					&& reverse_configuration != robust_reverse)
				{
					printf("[INBAND-TX] TIER-CROSS reverse-ACK pin: reverse_configuration "
						"%d -> %d (hold the robust rung across the cross so the reverse SACK "
						"decodes on MFSK, mirroring legacy's reverse-robust hold)\n",
						reverse_configuration, robust_reverse);
					fflush(stdout);
					reverse_configuration = robust_reverse;
				}
				// reverse_configuration preserved from other direction's gearshift
				if(reverse_configuration == CONFIG_NONE)
					reverse_configuration = forward_configuration;
			}

			negotiated_configuration = forward_configuration;

			// Universal Axis-1 transition contract.  A producer may have prepared
			// the lower-axis invalidation already; if not, repair the omission here.
			// The preparation token makes this exactly-once for a given move.
			if(link_status == CONNECTED && forward_configuration != current_configuration)
			{
				const bool prepared = axis1_supremacy_prepared
					&& axis1_supremacy_prepared_from == current_configuration
					&& axis1_supremacy_prepared_to == forward_configuration;
				if(!prepared)
					policy_axis1_supremacy_on_move(current_configuration,
						forward_configuration, "set-config-chokepoint");
				axis1_supremacy_prepared = false;
				axis1_supremacy_prepared_from = CONFIG_NONE;
				axis1_supremacy_prepared_to = CONFIG_NONE;
			}

			// Gearshift-v2 transition synchronization chokepoint. Every on-wire
			// SET_CONFIG producer (normal v2, implementation safety demote, BREAK
			// recovery, etc.) passes through this builder. A matching v2-owned
			// switch is recognized and ignored; any other connected-session move
			// invalidates probe/context state so the selector cannot keep reasoning
			// as though the old modulation were still in force.
			if(rate_opt.controls_link() && link_status == CONNECTED
			   && forward_configuration != current_configuration)
			{
				rate_opt.notify_external_axis1_transition(
					current_configuration, forward_configuration,
					"set-config-chokepoint", narrowband_enabled == YES);
			}

			messages_control.data[1] = forward_configuration;
			messages_control.data[2] = reverse_configuration;
			messages_control.length = 3;

			printf("[GEARSHIFT] SET_CONFIG: forward=%d reverse=%d (SNR down=%.1f up=%.1f) link_status=%d\n",
				forward_configuration, reverse_configuration,
				measurements.SNR_downlink, measurements.SNR_uplink, (int)link_status);
			fflush(stdout);
#ifdef MERCURY_GUI_ENABLED
			{
				char buf[80];
				snprintf(buf, sizeof(buf), "[GEARSHIFT -> CONFIG_%d]", forward_configuration);
				gui_push_monitor_event(buf, true);
			}
#endif
		}
		else if(code==KEY_EXCHANGE_1)
		{
			// X25519 public key exchange (32 bytes)
			uint8_t pubkey[X25519_KEY_SIZE];
			if(cipher_suite.generate_x25519_keypair(pubkey) != 0)
			{
				printf("[CRYPTO] ERROR: X25519 keypair generation failed (RNG)\n");
				fflush(stdout);
				messages_control.status = FREE;
				return ERROR_;
			}
			messages_control.data[0] = code;
			memcpy(&messages_control.data[1], pubkey, X25519_KEY_SIZE);
			messages_control.length = 1 + X25519_KEY_SIZE;  // 33 bytes
			messages_control.id = 0;
			printf("[CRYPTO] Sending X25519 pubkey (32 bytes)\n");
			fflush(stdout);
		}
		else if(code==KEY_EXCHANGE_2)
		{
			// Hybrid ML-KEM KX chunk frame (MLKEM_HYBRID_PLAN.md §4). The
			// current chunk index is kx_tx_next_index; the chunk codec writes
			// [kind|idx|count|CRC8 | payload] into messages_control.data. The
			// kx_tx_* state was armed by kx_begin_chunk_send(). The frame is a
			// normal control frame from here on (LDPC + per-frame control ACK on
			// the OFDM data_configuration, same transport as KEY_EXCHANGE_1).
			// NOTE: only KEY_EXCHANGE_2 chunk-encodes here. The CMD NEVER sends
			// KX3 CHUNKS (the RSP does, via its ACK transport); a CMD-built
			// KEY_EXCHANGE_3 is the BARE per-chunk KX3 ACK
			// (data-flow-control-slot-lifecycle.md §8.2(2)) which falls into the
			// length=1 `else` builder below (kx_send_next_chunk's COMMANDER branch
			// only ever calls this with KEY_EXCHANGE_2).
			int cap = kx_tx_chunk_cap;
			int wrote = cl_cipher_suite::kx_chunk_encode(
				(uint8_t)kx_tx_kind, kx_tx_src, kx_tx_total,
				kx_tx_next_index, cap,
				(uint8_t*)messages_control.data,
				(N_MAX/8));
			if(wrote < 0)
			{
				printf("[CRYPTO] ERROR: KX chunk encode failed (kind=0x%02x idx=%d cap=%d)\n",
					kx_tx_kind, kx_tx_next_index, cap);
				fflush(stdout);
				messages_control.status = FREE;
				return ERROR_;
			}
			messages_control.length = wrote;
			messages_control.id = 0;
			printf("[CRYPTO] Sending KX chunk kind=0x%02x idx=%d/%d (%d bytes)\n",
				kx_tx_kind, kx_tx_next_index, kx_tx_count, wrote);
			fflush(stdout);
		}
		else if(code==KEY_ACTIVATE)
		{
			messages_control.data[0] = code;
			// Append 8-byte key confirmation tag for PSK verification
			uint8_t confirm_tag[8];
			cipher_suite.compute_key_confirmation(confirm_tag);
			memcpy(&messages_control.data[1], confirm_tag, 8);
			messages_control.length = 9;
			messages_control.id = 0;
			printf("[CRYPTO] Sending KEY_ACTIVATE with confirmation tag\n");
			fflush(stdout);
		}
		else if(code==REPEAT_LAST_ACK)
		{
			messages_control.length=1;
			messages_control.data[0]=code;
			messages_control.id=0;
			messages_control.nResends=1;
		}
		else if(code==SET_LINK_PARAMS)
		{
			// SACK Design A Step 10 — CMD-side SET_LINK_PARAMS encoder.
			//
			// Wire format (§4.4):
			//   data[0] = SET_LINK_PARAMS (0x43)
			//   data[1] = batch (u8, clamped [10, AXIS2_BATCH_CEIL])
			//   data[2] = sack_mode (u8, 0=OFF, 1=ON, 2=PROBE) — Step 11 will
			//             populate. Step 10 always writes 1 (ON) for v2 sessions.
			//   data[3] = CRC8 over data[1..2] using POLY_CRC8 (matches existing
			//             CRC8_calc(data, 2) usage).
			//   length  = 4
			//
			// Reads pending_link_params_{batch_size,sack_mode} from the calling
			// controller (set by policy_evaluate_axis2() / Step 11's Axis-3).
			// Sentinels (-1) substitute current state.
			//
			// Defensive null-guard: messages_control.data is allocated lazily
			// in init_messages_buffers(). In normal v2 sessions this has run
			// before any Axis-2 move fires, but synthetic-fire test mode
			// (--test-policy-axis2-fire=*) skips ARQ.init() — so guard.
			if(messages_control.data == NULL)
			{
				// Pre-init synthetic test mode (messages_control.data not yet
				// allocated). Print the values the controller staged so the
				// test harness can verify the controller wired the right
				// targets without needing a real wire frame.
				int peek_batch = pending_link_params_batch_size;
				if(peek_batch < 0) peek_batch = data_batch_size;
				int peek_sack  = pending_link_params_sack_mode;
				if(peek_sack < 0 || peek_sack > 2) peek_sack = 1;
				printf("[CMD-LINK-PARAMS] SKIP TX: messages_control.data is NULL "
					"(pre-init synthetic test mode; no real wire frame). "
					"WOULD HAVE SENT: batch=%d sack_mode=%d\n",
					peek_batch, peek_sack);
				fflush(stdout);
				// Roll back the status set above so the caller treats this as
				// no-op (don't leave the slot in ADDED_TO_LIST forever).
				messages_control.status = FREE;
				messages_control.type = NONE;
				pending_link_params_batch_size = -1;
				pending_link_params_sack_mode = -1;
				return success;  // == ERROR_ from this fast-out path
			}
			int target_batch = pending_link_params_batch_size;
			if(target_batch < 0) target_batch = data_batch_size;
			if(target_batch < AXIS2_BATCH_FLOOR) target_batch = AXIS2_BATCH_FLOOR;
			if(target_batch > axis2_batch_ceiling())
				target_batch = axis2_batch_ceiling();

			int target_sack = pending_link_params_sack_mode;
			if(target_sack < 0 || target_sack > 2) target_sack = 1;  // default ON

			messages_control.data[0] = code;
			messages_control.data[1] = (char)(unsigned char)target_batch;
			messages_control.data[2] = (char)(unsigned char)target_sack;
			// CRC8 over the (batch, sack_mode) bytes only — NOT the type byte
			// (whose integrity is already protected by the OFDM LDPC codeword's
			// CRC16 on the msg header). Polynomial: POLY_CRC8 = 0xF4 per
			// datalink_defines.h:175. Matches the SACK_RSP frame's CRC8
			// coverage rule (§4.2.2 / Step 7's send_sack_v2_frame).
			messages_control.data[3] = (char)CRC8_calc(
				(char*)&messages_control.data[1], 2);
			messages_control.length = 4;
			messages_control.id = 0;

			printf("[CMD-LINK-PARAMS] SET_LINK_PARAMS TX: batch=%d sack_mode=%d crc8=0x%02x\n",
				target_batch, target_sack, (unsigned char)messages_control.data[3]);
			fflush(stdout);

			// Clear the staging so a subsequent (unrelated) add_message_control
			// call cannot pick up stale targets.
			pending_link_params_batch_size = -1;
			pending_link_params_sack_mode = -1;
		}
		else if(code==ROBUST_DWELL_BATCH_OP)
		{
			// FIX-A — ROBUST-tier dwell-batch encoder (data-flow-robust-tier-arq-batch.md
			// §5.2). Carries the CMD's chosen robust dwell batch to the RSP so BOTH
			// peers run the SAME data_batch_size (the 4-wire-failure symmetry invariant).
			// DELIBERATELY NOT SET_LINK_PARAMS: that op's RSP handler clamps to
			// [AXIS2_BATCH_FLOOR=10,32] and would force a 4-8 batch UP to 10 on the RSP
			// only (OR-2 / L4). This op applies the value straight through the relaxed
			// set_data_batch_size() chokepoint (clamped only to [1..ROBUST_DWELL_BATCH_MAX]).
			//
			// Wire format:
			//   data[0] = ROBUST_DWELL_BATCH_OP (0x44)
			//   data[1] = batch (u8, clamped [1..ROBUST_DWELL_BATCH_MAX])
			//   data[2] = CRC8 over data[1], POLY_CRC8=0xF4 (CRC8_calc usage matches
			//             SACK_RSP / SET_LINK_PARAMS coverage rule).
			//   length  = 3
			//
			// Reads pending_robust_dwell_batch (set by evaluate_robust_dwell_batch()).
			// Defensive null-guard mirrors SET_CONFIG / SET_LINK_PARAMS for the
			// synthetic-fire test path (messages_control.data lazily allocated).
			if(messages_control.data == NULL)
			{
				int peek_batch = pending_robust_dwell_batch;
				if(peek_batch < 0) peek_batch = data_batch_size;
				printf("[CMD-ROBUST-DWELL] SKIP TX: messages_control.data is NULL "
					"(pre-init synthetic test mode; no real wire frame). "
					"WOULD HAVE SENT: batch=%d\n", peek_batch);
				fflush(stdout);
				messages_control.status = FREE;
				messages_control.type = NONE;
				pending_robust_dwell_batch = -1;
				return success;  // == ERROR_ fast-out
			}
			int target_batch = pending_robust_dwell_batch;
			if(target_batch < 0) target_batch = data_batch_size;
			if(target_batch < 1) target_batch = 1;
			if(target_batch > ROBUST_DWELL_BATCH_MAX) target_batch = ROBUST_DWELL_BATCH_MAX;

			messages_control.data[0] = code;
			messages_control.data[1] = (char)(unsigned char)target_batch;
			messages_control.data[2] = (char)CRC8_calc(
				(char*)&messages_control.data[1], 1);
			messages_control.length = 3;
			messages_control.id = 0;

			printf("[CMD-ROBUST-DWELL] ROBUST_DWELL_BATCH_OP TX: batch=%d crc8=0x%02x\n",
				target_batch, (unsigned char)messages_control.data[2]);
			fflush(stdout);

			pending_robust_dwell_batch = -1;
		}
		else if(code==CLOSE_CONNECTION)
		{
			// Option W STEP 3 (data-flow-stream-offset.md §8.6) — EOT (end-of-transfer) stamp.
			// Ride this SENDER's final total_committed_bytes + running stream CRC-32 on the
			// EXISTING graceful-disconnect frame (NO new handshake — the §8.6 teardown-race
			// constraint). By the time this frame is built the CMD has drained the TX FIFO and
			// every data batch is ACKed (arq_commander.cc:992), so both accumulators are FINAL.
			// The peer (the CLOSE receiver = the data receiver) reconciles them against its
			// delivered cursor/CRC, catching a truncated FINAL batch that the per-batch W checks
			// (inherently one batch behind) cannot see. A pure-receiver / no-data closer emits
			// committed=0, crc=INIT ⇒ the peer's 0/INIT matches (no false-fire).
			// Defensive null-guard for the synthetic pre-init test path (mirrors SET_CONFIG).
			if(messages_control.data == NULL)
			{
				messages_control.status = FREE;
				messages_control.type = NONE;
				return success;
			}
			messages_control.data[0] = code;
			uint64_t eot_committed = tx_stream_committed;
			for(int i=0;i<8;i++)
				messages_control.data[1+i] = (char)((eot_committed >> (8*i)) & 0xFFu);
			uint32_t eot_crc = tx_stream_crc;
			for(int i=0;i<4;i++)
				messages_control.data[9+i] = (char)((eot_crc >> (8*i)) & 0xFFu);
			// crc8 over the 12 EOT payload bytes (data[1..12]), POLY_CRC8 (matches SET_LINK_PARAMS /
			// SACK_RSP coverage rule). Lets the RX reject a short/legacy/garbled frame as ABSENT-EOT
			// (safe no-op) instead of parsing stale bytes into a false teardown.
			messages_control.data[W_EOT_PAYLOAD_BYTES] =
				(char)CRC8_calc((char*)&messages_control.data[1], W_EOT_PAYLOAD_BYTES - 1);
			messages_control.length = W_EOT_FRAME_LENGTH;
			messages_control.id = 0;
#ifndef TERMINAL_EOT_RETRY_FAILBEFORE
			// Initial transmission plus a small, terminal-specific retry train.
			// The ordinary nResends=20 data budget is inappropriate here: coupled
			// to the long control timeout it produced only one retry before the
			// 45 s final backstop. CLOSE+EOT is immutable, so every retry is the
			// exact same committed-length/CRC descriptor.
			messages_control.nResends = TERMINAL_EOT_RETRIES + 1;
#endif
			printf("[CMD-EOT] CLOSE_CONNECTION+EOT TX: committed=%llu crc=0x%08x crc8=0x%02x\n",
				(unsigned long long)eot_committed, eot_crc,
				(unsigned char)messages_control.data[W_EOT_PAYLOAD_BYTES]);
			fflush(stdout);
		}
		else
		{
			messages_control.length=1;
			for(int j=0;j<messages_control.length;j++)
			{
				messages_control.data[j]=code;
			}
			messages_control.id=0;
		}

		success=SUCCESSFUL;
		if(code == SET_CONFIG && duty_trace_enabled())
		{
			printf("[DUTY-TRACE] set-config-enqueue turboshift_active=%d turboshift_phase=%d "
				"break_recovery_phase=%d data_ack_received=%d snr_uplink=%.1f "
				"current=%d negotiated=%d\n",
				(int)turboshift_active, (int)turboshift_phase, break_recovery_phase,
				data_ack_received, measurements.SNR_uplink, current_configuration,
				negotiated_configuration);
			fflush(stdout);
		}
		this->connection_status=TRANSMITTING_CONTROL;
	}
	return success;
}

void cl_arq_controller::process_messages_tx_control()
{
	// STAGE 3b GEARSHIFT DRIVE neutraliser (data-flow-perbatch-config.md §12 W3):
	// the unilateral SET_CONFIG builder (add_message_control) queued NO control frame
	// but its callers force connection_status=TRANSMITTING_CONTROL after it returns.
	// When the one-shot flag is armed, re-route straight back to TRANSMITTING_DATA so
	// the next batch goes out at the dropped config (with the W1 passband tag) instead
	// of spinning in an empty control-TX. Single chokepoint on the consuming side
	// (mirrors the single chokepoint in the builder). Default-off: never armed ->
	// byte-identical. messages_control is already FREE (the builder freed it).
	if(inband_unilateral_armed)
	{
		inband_unilateral_armed = false;
		connection_status = TRANSMITTING_DATA;
		printf("[INBAND-TX] unilateral drop applied — resuming DATA at CONFIG_%d "
			"(no control handshake)\n", current_configuration);
		fflush(stdout);
		return;
	}

	if(messages_control.status==ADDED_TO_LIST&&message_batch_counter_tx<control_batch_size)
	{
		messages_batch_tx[message_batch_counter_tx]=messages_control;
		message_batch_counter_tx++;
		messages_control.status=ADDED_TO_BATCH_BUFFER;
		stats.nSent_control++;
	}
	else if(messages_control.status==ACK_TIMED_OUT)
	{
		if(--messages_control.nResends>0&&message_batch_counter_tx<control_batch_size)
		{
			if(link_status==DISCONNECTING && messages_control.data!=NULL
			   && (unsigned char)messages_control.data[0]==CLOSE_CONNECTION)
			{
				int ordinal = stats.nReSent_control + 1;
				printf("[CMD-EOT-RETRY] retransmitting identical CLOSE+EOT retry=%d/%d\n",
					ordinal, (int)TERMINAL_EOT_RETRIES);
				fflush(stdout);
			}
			// A final-connect retry is a new causal attempt, just like a
			// START_CONNECTION retry.  Without this re-anchor, the fixed phase
			// watchdog can expire while the retry's ACK is already on the wire.
			if(connect_final_retry_reanchors((unsigned char)messages_control.data[0]))
			{
				connection_attempt_timer.reset();
				connection_attempt_timer.start();
				printf("[CONNECT-RETRY] final TEST_CONNECTION retry re-anchored phase watchdog\n");
				fflush(stdout);
			}

			// Increment connection attempts counter if trying to connect
			if((link_status==CONNECTING || link_status==NEGOTIATING || link_status==CONNECTION_ACCEPTED) &&
			   messages_control.data[0]==START_CONNECTION)
			{
				connection_attempts++;
				std::cout<<"Connection attempt "<<connection_attempts<<" of "<<max_connection_attempts<<std::endl;
				// Reset timer for this new attempt (making connection_timeout a per-attempt timeout)
				connection_attempt_timer.reset();
				connection_attempt_timer.start();

				// NB/WB auto-negotiation: phase transitions
				if(commander_configured_nb >= 0 && nb_probe_max > 0)
				{
					if(commander_configured_nb == NO)
					{
						// WB commander: NB probes for attempts 0..nb_probe_max-1, then WB forever
						if(connection_attempts == nb_probe_max)
						{
							printf("[NB-NEG] Commander: Phase 2 - restoring WB\n");
							fflush(stdout);
							switch_narrowband_mode(NO);
							messages_control.status = FREE;
							return;
						}
					}
					else
					{
						// NB commander: alternate NB/WB in blocks of nb_probe_max
						// NB probes (0..1), WB probes (2..3), NB probes (4..5), ...
						// This ensures both NB and WB responders eventually get reached
						int phase = connection_attempts / nb_probe_max;
						int phase_start = phase * nb_probe_max;
						bool should_be_wb = (phase % 2 == 1);  // odd phases = WB
						if(connection_attempts == phase_start)
						{
							if(should_be_wb && narrowband_enabled == YES)
							{
								printf("[NB-NEG] Commander: switching to WB probe (attempt %d)\n", connection_attempts);
								fflush(stdout);
								switch_narrowband_mode(NO);
								messages_control.status = FREE;
								return;
							}
							else if(!should_be_wb && narrowband_enabled == NO)
							{
								printf("[NB-NEG] Commander: switching back to NB (attempt %d)\n", connection_attempts);
								fflush(stdout);
								switch_narrowband_mode(YES);
								messages_control.status = FREE;
								return;
							}
						}
					}
				}
			}

			messages_batch_tx[message_batch_counter_tx]=messages_control;
			message_batch_counter_tx++;
			messages_control.status=ADDED_TO_BATCH_BUFFER;
			stats.nReSent_control++;
		}
		else
		{
			stats.nLost_control++;
			messages_control.status=FAILED_;
#ifndef TERMINAL_SETTLE_FAILBEFORE
				// A graceful CLOSE that exhausted its fast retry train must remain
				// receptive to a late terminal ACK until the independent 45 s final
				// backstop. Do not turn retry-budget exhaustion into early give-up.
				if(link_status==DISCONNECTING && messages_control.data!=NULL
				   && (unsigned char)messages_control.data[0]==CLOSE_CONNECTION)
				{
					cmd_terminal_eot_wait_for_backstop();
					return;
				}
#endif
		}
	}

	if(messages_control.status==ADDED_TO_BATCH_BUFFER)
	{
		// Phase B Wave 2 v2 — PHY swap site A (fact-doc §13.2).
		// When the queued control frame is START_CONNECTION AND the MFSK
		// codec is available, emit the MFSK CONNECT suffix instead of an
		// LDPC frame. The legacy state machine continues unchanged — this
		// just swaps the bits on the wire. messages_control transitions
		// from ADDED_TO_BATCH_BUFFER → PENDING_ACK exactly as the LDPC
		// path would, via the post-send block below.
		//
		// NB sessions (M=8) fall back to LDPC because connect_pattern_nsymb<=0.
		// passive_monitor never reaches here (it never builds CMD-side
		// control frames).
		bool mfsk_connect_path =
			messages_control.data[0] == START_CONNECTION
			&& narrowband_enabled != YES
			&& telecom_system->ack_mfsk.connect_pattern_nsymb > 0;
		if(messages_control.data[0] == START_CONNECTION)
		{
			// Every START transmission owns a fresh causal epoch. A fallback
			// LDPC START leaves the MFSK-only guard disarmed; a successful MFSK
			// send below anchors its causal timer at START audio transfer.
			start_ack_causal_guard_armed = false;
			start_ack_causal_timer.stop();
			start_ack_causal_timer.reset();
		}
		// Phase B Wave 3 — PHY swap site E (fact-doc §14).
		// TEST_CONNECTION (CMD→RSP) gets the same MFSK suffix treatment as
		// START_CONNECTION. The legacy LDPC build at arq_commander.cc:463-477
		// has already populated messages_control.data[1..6] with float SNR +
		// local_capability + SSID; Site E reads those fields and emits the
		// MFSK CONNECT suffix instead. RSP side: Site F (§14 / arq_responder.cc).
		bool mfsk_test_conn_path =
			messages_control.data[0] == TEST_CONNECTION
			&& narrowband_enabled != YES
			&& telecom_system->ack_mfsk.connect_pattern_nsymb > 0;
		bool mfsk_tx_done = false;
		if(mfsk_connect_path)
		{
			// Sender callsign as the legacy LDPC path builds it at
			// arq_commander.cc:454 — strip SSID before packing.
			std::string base_call = callsign_strip_ssid(my_call_sign);
			long long elapsed = send_mfsk_start_conn_phy(base_call);
			if(elapsed > 0)
			{
				messages_control.ack_timer.start();
				printf("[CMD-CONNECT-V2] MFSK START_CONN sent (%lld ms wall-clock)\n",
					elapsed);
				fflush(stdout);
				// Mirror the messages_control bookkeeping send_batch() performs
				// for CONTROL frames at arq_common.cc:3668-3672 (post-batch
				// loop), so the legacy state machine sees the same transition
				// it would after an LDPC TX.
				messages_control.status = PENDING_ACK;
				start_ack_causal_guard_armed = true;
				// Clear the batch slot we filled at line :753 / :688 — the
				// LDPC TX path resets it inside send_batch (arq_common.cc:3674-3679).
				for(int i = 0; i < message_batch_counter_tx; i++)
				{
					messages_batch_tx[i].ack_timeout = 0;
					messages_batch_tx[i].id          = 0;
					messages_batch_tx[i].length      = 0;
					messages_batch_tx[i].nResends    = 0;
					messages_batch_tx[i].status      = FREE;
					messages_batch_tx[i].type        = NONE;
				}
				message_batch_counter_tx = 0;
				mfsk_tx_done = true;
			}
			else
			{
				printf("[CMD-CONNECT-V2] MFSK START_CONN unavailable (codec guard "
					"tripped) — falling back to LDPC START_CONNECTION\n");
				fflush(stdout);
			}
		}
		else if(mfsk_test_conn_path)
		{
			// Site E: extract the legacy LDPC TEST_CONNECTION fields from
			// messages_control.data (built at arq_commander.cc:466-475).
			u_SNR tmp_SNR;
			for(int i = 0; i < 4; i++)
				tmp_SNR.char4_SNR[i] = messages_control.data[i+1];
			float snr_uplink = tmp_SNR.f_SNR;
			uint8_t local_cap = (uint8_t)messages_control.data[5];
			uint8_t ssid      = (uint8_t)messages_control.data[6];
			long long elapsed = send_mfsk_test_conn_phy(snr_uplink, local_cap, ssid);
			if(elapsed > 0)
			{
				printf("[CMD-TEST-CONN-V3] MFSK TEST_CONN sent (%lld ms wall-clock) "
					"snr=%.1f dB local_cap=0x%02X ssid=%u\n",
					elapsed, snr_uplink, local_cap, ssid);
				fflush(stdout);
				// Same bookkeeping as Site A — messages_control transitions
				// to PENDING_ACK; the post-TX state machine continues
				// unchanged (Site D handles the TEST_CONNECTION_ACK echo).
				messages_control.ack_timer.start();
				messages_control.status = PENDING_ACK;
				for(int i = 0; i < message_batch_counter_tx; i++)
				{
					messages_batch_tx[i].ack_timeout = 0;
					messages_batch_tx[i].id          = 0;
					messages_batch_tx[i].length      = 0;
					messages_batch_tx[i].nResends    = 0;
					messages_batch_tx[i].status      = FREE;
					messages_batch_tx[i].type        = NONE;
				}
				message_batch_counter_tx = 0;
				mfsk_tx_done = true;
			}
			else
			{
				printf("[CMD-TEST-CONN-V3] MFSK TEST_CONN unavailable (codec guard "
					"tripped) — falling back to LDPC TEST_CONNECTION\n");
				fflush(stdout);
			}
		}

		// Legacy LDPC path: same as pre-v2 when neither MFSK path fired
		// OR the MFSK send returned 0 (codec runtime guard tripped).
		if(!mfsk_tx_done)
		{
			// Commander CONTROL TX: full-length frames (responder can't predict frame type)
			telecom_system->set_mfsk_ctrl_mode(false);
			pad_messages_batch_tx(control_batch_size);
			send_batch();
		}

		// Post-TX flush handled inside send_batch() via rx_mute.

		if(messages_control.data[0] == KEY_EXCHANGE_1)
		{
			// KEY_EXCHANGE_1: receive LDPC ACK on data_configuration (OFDM).
			// Responder sends full OFDM frame with pubkey data.
			telecom_system->set_mfsk_ctrl_mode(false);
			telecom_system->data_container.frames_to_read =
				telecom_system->data_container.preamble_nSymb + telecom_system->data_container.Nsymb;
			printf("[CMD-RX] KEY_EXCHANGE_1: expecting LDPC ACK on config %d, ftr=%d\n",
				data_configuration,
				telecom_system->data_container.frames_to_read.load());
			fflush(stdout);
		}
		else if(ack_pattern_time_ms > 0)
		{
			// Expect ACK tone pattern. Start polling quickly (4 symbols ~90ms);
			// receive_ack_pattern() re-polls every 2 symbols until pattern arrives.
			// Large initial ftr delays first poll and pushes the OFDM turnaround
			// past the responder's buffer upper_bound (CONFIG_0 entry regression).
			telecom_system->data_container.frames_to_read = 4;
		}
		else
		{
			// Fallback: expect short LDPC ctrl frame on ack_configuration
			load_configuration(ack_configuration, PHYSICAL_LAYER_ONLY,NO);
			telecom_system->set_mfsk_ctrl_mode(true);
			telecom_system->data_container.frames_to_read =
				telecom_system->data_container.preamble_nSymb + telecom_system->get_active_nsymb();
		}
		connection_status=RECEIVING_ACKS_CONTROL;

		// RECOVERY-ACK robustness (recovery-ack-robustness.md §4/§6.2): the control-
		// ACK wait (the BREAK-recovery SET_CONFIG turnaround) combines the SAME R
		// base reps the RSP control-code arm emits (arq_responder.cc:1500), so a
		// turnaround-straddled clean ACK clears the 7/16 bar. Gated
		// MERCURY_RECOVERY_ACK_ROBUST + ack_pattern_time_ms>0 (pattern-ACK path);
		// default-off → reps stay 1 → BYTE-IDENTICAL. Data-ACK waits clear it back
		// to 1 (set_recovery_ack_reps(1)) so the data turnaround stays single-block.
		set_recovery_ack_reps_for_wait(/*control_ack=*/true);

		// Recalculate timeout: guard delays from prior ACK detection can leave
		// receiving_timeout stale (e.g. 900ms), too short for the control round-trip.
		calculate_receiving_timeout();
		if(link_status==DISCONNECTING && messages_control.data!=NULL
		   && (unsigned char)messages_control.data[0]==CLOSE_CONNECTION)
			cmd_arm_terminal_eot_arq();
		receiving_timer.start();
		if(g_verbose) { printf("[CMD-RX] Entering receive mode: ack_cfg=%d recv_timeout=%d msg_tx_time=%d ctrl_tx_time=%d ack_batch=%d ftr=%d\n", ack_configuration, receiving_timeout, message_transmission_time_ms, ctrl_transmission_time_ms, ack_batch_size, telecom_system->data_container.frames_to_read.load()); fflush(stdout); }

		if(messages_control.data[0]==SWITCH_BANDWIDTH)
		{
			// Responder must fill its RX buffer before it can decode the
			// SWITCH_BANDWIDTH control frame.  For NB CONFIG_0 the buffer
			// fill time alone is ~14 s, so use the same 2×msg_tx basis as
			// the normal ACK timeout (calculate_receiving_timeout).
			receiving_timeout = 2 * message_transmission_time_ms
				+ ack_pattern_time_ms
				+ 2*ptt_on_delay_ms + 2*ptt_off_delay_ms + 3000;
			printf("[BW-NEG] SWITCH_BANDWIDTH recv_timeout=%d msg_tx=%d\n",
				receiving_timeout, message_transmission_time_ms);
			fflush(stdout);
		}

		if(messages_control.data[0]==SET_CONFIG)
		{
			if(negotiated_configuration!= current_configuration)
			{
				data_configuration=negotiated_configuration;
				gear_shift_timer.start();
				// Reset batch stats so gearshift evaluation only reflects new config
				last_transmission_block_stats.nBatches_sent = 0;
				last_transmission_block_stats.nBatches_acked = 0;
				last_transmission_block_stats.nBatches_fully_acked = 0;  // CLEAN-BATCH VIABILITY (§9)
				last_transmission_block_stats.nReSent_data = 0;
				last_transmission_block_stats.nSent_data = 0;
			}
			// Always wait for SET_CONFIG ACK (stay in RECEIVING_ACKS_CONTROL).
			// Previously jumped to TRANSMITTING_DATA when negotiated==current,
			// causing collision after BREAK (commander sent data before responder
			// finished processing SET_CONFIG).

			// SUPERSHIFT SNR-sentinel ENABLEMENT (climb follow-up #1b, Option 1;
			// data-flow-snr-measurements.md §1.7 / §7). We are committing a
			// SET_CONFIG and entering RECEIVING_ACKS_CONTROL. If turboshift is
			// active (or mid-direction-switch), the RSP will reply with the
			// ACK+SNR suffix (its SEND gate, arq_responder.cc:1122-1124, on the
			// SYMMETRIC `data[0]==SET_CONFIG` condition). Arm the CMD's SNR-suffix
			// DECODE so the producer at arq_common.cc:5555 can prime
			// measurements.SNR_uplink mid-climb — BREAKING the deadlock that kept
			// turbo_snr_ack_enabled false (it was set TRUE only by the re-trigger
			// at :4579, itself gated on SNR_uplink > -90, which only the producer
			// supplies). For a NON-turbo SET_CONFIG (e.g. break-recovery at
			// TURBO_DONE) this evaluates FALSE — symmetric with the RSP, which
			// also sends a bare ACK (no suffix) there. The flag is cleared by
			// clear_snr_arm_for_data_ack_wait() at every data-ACK-wait entry (sec
			// 18) before any data flows. [CORRECTED 2026-05-31: previously cited
			// finish_turbo_direction() (:3792) as the clearer -- true for the
			// turbo-only arm, but the A1 widening (below) arms on a steady-state +1
			// gearshift SET_CONFIG whose ACK-apply (:4702->:4753) never calls
			// finish_turbo_direction(); the dedicated data-ACK-wait clear is the
			// correct guard. See sec 18.] So a data
			// ACK's SACK suffix is NEVER routed to the SNR decoder (§7 collision
			// audit). Mirrors the re-trigger's own true-write; no clobber (the
			// re-trigger sets turboshift_active before queueing this SET_CONFIG).
			//
			// REAL FAST-PROBE (A1, gearshift-climb-engine.md sec 14): the bare
			// turbo predicate is ASYMMETRIC with the RSP send-gate — the RSP keeps
			// suffixing the SNR on gearshift-ladder SET_CONFIG ACKs (its phase
			// stays TURBO_FORWARD) while the CMD's phase is TURBO_DONE in steady
			// state, so the CMD never armed -> SNR_uplink stuck at -99.9 -> the
			// sec 13 elevator's `SNR_uplink > -90` gate never fired. Widen the arm
			// with an OR clause (in the pure helper, unit-testable) so it ALSO
			// arms for an UPWARD gearshift SET_CONFIG (gear_shift_on==YES,
			// negotiated index > current index). The `data[0]==SET_CONFIG`
			// enclosing block + the helper's own `control_code==SET_CONFIG`
			// conjunct keep this OFF every DATA ACK (the SACK-suffix decode is
			// never routed to the SNR decoder). CMD-only decode-enable: NO wire
			// change (the RSP already sends the suffix).
			bool fastprobe_config_up =
				config_ladder_index(negotiated_configuration) >
				config_ladder_index(current_configuration);
			turbo_snr_ack_enabled = turbo_snr_ack_armed_for_gearshift(
				turboshift_active, turboshift_phase, messages_control.data[0],
				gear_shift_on == YES, fastprobe_config_up);
		}

		if(messages_control.data[0]==REPEAT_LAST_ACK)
		{
			messages_control.ack_timeout=0;
			messages_control.id=0;
			messages_control.length=0;
			messages_control.nResends=0;
			messages_control.status=FREE;
			messages_control.type=NONE;

			// §18: this is the ONE RECEIVING_ACKS_DATA entry that bypasses
			// process_messages_tx_data() (the data-TX setups at :~1317/:~1813 clear
			// the arm for every TRANSMITTING_DATA→data-wait route). Clear here too so
			// the arm is provably false on EVERY data-ACK wait without any transitive
			// "a data-TX always preceded this" assumption (cross-layer data-flow audit).
			clear_snr_arm_for_data_ack_wait();
			connection_status=RECEIVING_ACKS_DATA;
		}
	}
}

int cl_arq_controller::add_message_tx_data(char type, int length, char* data)
{
	int success=ERROR_;
	if(length<0)
	{
		success=MESSAGE_LENGTH_ERROR;
		return success;
	}

	// SACK Design A Step 1 — DATA_LONG payload capacity is reduced by 1 byte
	// when sack_v2_enabled (the new batch_seq_id byte occupies that space).
	// In v1 mode the effective value is identical to the legacy 4-byte macro.
	if(type==DATA_LONG && length>(max_data_length+max_header_length-effective_data_long_header_length(sack_v2_enabled, header_carries_d5)))
	{
		success=MESSAGE_LENGTH_ERROR;
		return success;
	}

	// SACK Design A Step 2 — DATA_SHORT payload capacity is reduced by 1 byte
	// when sack_v2_enabled (the new batch_seq_id byte occupies that space).
	// In v1 mode the effective value is identical to the legacy 5-byte macro.
	if(type==DATA_SHORT && length>(max_data_length+max_header_length-effective_data_short_header_length(sack_v2_enabled, header_carries_d5)))
	{
		success=MESSAGE_LENGTH_ERROR;
		return success;
	}


	for(int i=0;i<nMessages;i++)
	{
		if (messages_tx[i].status==FREE)
		{
			messages_tx[i].type=type;
			messages_tx[i].length=length;
			for(int j=0;j<messages_tx[i].length;j++)
			{
				messages_tx[i].data[j]=data[j];
			}
			messages_tx[i].id=i;
			messages_tx[i].nResends=this->nResends;
			messages_tx[i].ack_timeout=this->ack_timeout_data;
			messages_tx[i].status=ADDED_TO_LIST;
			// SACK Design A Step 3 — batch_seq_id is assigned by
			// process_messages_tx_data() at the moment the frame is moved into
			// the outgoing batch (so that the value matches the actual batch
			// being keyed). Initialize to -1 (sentinel "unset") here so that
			// a defensive read before assignment is visible in logs.
			messages_tx[i].batch_seq_id=-1;
			success=SUCCESSFUL;
			break;
		}
	}
	return success;
}

// ID-SKEW ROOT FIX (CANONICAL_NUMBERS.md §87; cross-storage index-skew).
// ------------------------------------------------------------------------------------
// The wire id byte a DATA frame carries (send_batch, arq_common.cc emits
// messages_batch_tx[i].id verbatim) IS the messages_tx[] slot the frame was staged into:
// add_message_tx_data (:3098) assigns .id = the FIRST-FREE slot index. The receiver stores
// each frame at loc = wire id, and on the sender the SACK read (:~6094) and the prev-batch
// retention requeue (cmd_prev_retain_requeue, :~366) both index the received-bitmap by the
// SAME slot number. Every one of those consumers assumes the load-bearing invariant
//   a new-data batch's frames occupy messages_tx slots [0, data_batch_size),
// i.e. slot index == batch position == receiver storage loc. That invariant BREAKS when
// the PREVIOUS batch was still unconfirmed (its frames PENDING_ACK / ACK_TIMED_OUT) in the
// low slots at THIS batch's staging tick: the first-free allocator then places this batch's
// frames at slots >= the previous batch's frame count, so the wire id is skewed by exactly
// that count (the observed bsi33->bsi34 +38 — bsi33's 38 unconfirmed frames held [0,37]
// through its BREAK storm, so bsi34 staged at [38,75] and shipped id=seq+38; its two
// in-window frames reseated at loc 38/39 and were counted-but-never-delivered -> the
// 332-byte prev-batch deletion the reseat keystones now loud-fail).
//
// Rather than teach every id/loc consumer to distinguish slot from batch-position (a wide,
// error-prone decoupling), RESTORE the invariant at the build transition: compact the fresh
// batch's frames down to slots [0, ND). Then slot == id == batch position == loc for every
// consumer, correct-by-construction, and the batch DELIVERS IN PLACE. Applies to non-mixed v1
// and v2 batches (the v2 mixed path already assigns the ordinal id via set_batch_tx_slot).
// Byte-identical when
// the frames are already contiguous at [0, ND) — the healthy case does no work. Moves in
// ascending target order (each source slot >= its target, so a slot is always free by the
// time it is written) and is ALL-OR-NOTHING: it bails without moving if any target slot holds
// a live non-batch frame (never clobbers unconfirmed data; the reseat keystones then loud-fail
// the residual skew — no corruption). Copies scalar fields + memcpy's the payload into each
// destination slot's OWN pre-allocated buffer (NEVER the .data pointer — copying it would alias
// two slots onto one buffer, the 184fdcc TX-alias hazard).
int cl_arq_controller::cmd_rebase_newdata_slots()
{
	{ const char* e = std::getenv("MERCURY_CMD_IDSKEW_DEFEAT");
	  if(e && *e && atoi(e) != 0) return 0; }        // fail-before arm: leave the skew in place

	// Gather this batch's staged new-data frames in ASCENDING slot order (one batch's worth).
	int src[MAX_SACK_BATCH_SIZE];
	int m = 0;
	for(int i = 0; i < nMessages && m < data_batch_size && m < MAX_SACK_BATCH_SIZE; i++)
		if(messages_tx[i].status == ADDED_TO_LIST)
			src[m++] = i;

	bool need = false;
	for(int k = 0; k < m; k++)
		if(src[k] != k) { need = true; break; }
	if(!need) return 0;                              // already [0, ND) -> byte-identical

	// SAFE iff every out-of-place target slot is FREE or holds one of THIS batch's own
	// frames (ADDED_TO_LIST, guaranteed a source that will be moved away before its slot is
	// written, since sources move in ascending order and each source slot >= its target).
	for(int k = 0; k < m; k++)
		if(src[k] != k
		   && messages_tx[k].status != FREE
		   && messages_tx[k].status != ADDED_TO_LIST)
		{
			printf("[CMD-IDSKEW-COMPACT] SKIP: target slot %d holds a live frame (status=%d) "
				"— %d new-data frame(s) left at skewed slots; the RSP reseat keystones will "
				"loud-fail the residual skew (no corruption)\n",
				k, (int)messages_tx[k].status, m);
			fflush(stdout);
			return 0;
		}

	int moved = 0;
	for(int k = 0; k < m; k++)
	{
		int s = src[k];
		if(s == k) continue;                        // already in its ordinal slot
		messages_tx[k].ack_timeout     = messages_tx[s].ack_timeout;
		messages_tx[k].nResends        = messages_tx[s].nResends;
		messages_tx[k].length          = messages_tx[s].length;
		messages_tx[k].type            = messages_tx[s].type;
		messages_tx[k].sequence_number = messages_tx[s].sequence_number;
		messages_tx[k].batch_seq_id    = messages_tx[s].batch_seq_id;
		if(messages_tx[s].length > 0 && messages_tx[s].data != NULL && messages_tx[k].data != NULL)
			memcpy(messages_tx[k].data, messages_tx[s].data, (size_t)messages_tx[s].length);
		messages_tx[k].id     = (char)k;            // the reset: slot == batch position
		messages_tx[k].status = ADDED_TO_LIST;
		messages_tx[s].status = FREE;               // vacate the skewed high slot
		moved++;
	}
	if(moved > 0)
	{
		printf("[CMD-IDSKEW-COMPACT] rebased %d new-data frame(s) to slots [0,%d) — wire id "
			"now == batch position (was skewed by the prior batch's unconfirmed slot base)\n",
			moved, m);
		fflush(stdout);
	}
	return moved;
}

void cl_arq_controller::commander_poll_late_role_demand()
{
	if(role != COMMANDER || link_status != CONNECTED
	   || connection_status != TRANSMITTING_DATA
	   || reverse_data_demand_acknowledged
	   || block_under_tx == YES || get_nOccupied_messages() != 0
	   || retransmit_count != 0 || messages_control.status != FREE
	   || fifo_buffer_tx.get_size() != fifo_buffer_tx.get_free_size()
	   || messages_rx_buffer.status == RECEIVED)
		return;
	if(telecom_system->data_container.frames_to_read > 0)
		return;

	this->receive();
	if(messages_rx_buffer.status == RECEIVED
	   && messages_rx_buffer.type == SACK_RSP)
	{
		int span = data_batch_size;
		if(span < 1) span = 1;
		if(span > MAX_SACK_BATCH_SIZE) span = MAX_SACK_BATCH_SIZE;
		bool bitmap[MAX_SACK_BATCH_SIZE] = {};
		unsigned char bsi = 0;
		bool demand = false;
		if(decode_sack_v2_frame(bitmap, span, &bsi, &demand) && demand)
			role_demand_note_acknowledged(true, "late-full-sack");
		messages_rx_buffer.status = FREE;
	}
}

void cl_arq_controller::process_messages_tx_data()
{
	// Phase D timing — entry to the batch builder. The gap between
	// cmd_ack_post_work_done and this marker is "state-machine idle"
	// (one or more main-loop ticks that did NOT build a batch). The
	// gap between this marker and cmd_batch_tx_start is the actual
	// batch-prep work (retx prefix, compression of new-data frames,
	// pad to size, etc.).
	mtl::log_event("cmd_tx_data_entry");
	commander_poll_late_role_demand();

	// STAGE 4d (D4 ESCALATION): before building the next batch, check whether an armed
	// CLIMB re-tag has exhausted its R floor with NO returning SACK confirm. The previous
	// send_batch() emitted the re-tag (incrementing inband_retag_count); a SACK confirm
	// would have DISARMED it via inband_retag_confirm_from_sack at the ACK-poll. If it is
	// STILL armed past R AND it was a CLIMB-UP (the down-ladder is down-only and cannot
	// rescue a lost climb), AUTO-DEMOTE to the last-confirmed config via the chokepoint tag
	// — NEVER a BREAK (design §4.1(b)/§4.3). The demote routes a SET_CONFIG through the
	// chokepoint (-> inband_unilateral_config_change + the W1 tag) and forces
	// TRANSMITTING_CONTROL; the inband_unilateral_armed one-shot re-routes back to
	// TRANSMITTING_DATA in process_messages_tx_control. Return so the demote owns the next
	// transition (do not also build a climb batch this cycle). No-op when inband is off /
	// not armed / not a climb / within R.
	if(inband_rate_feature_enabled() && inband_retag_escalate_if_climb_exhausted())
	{
		this->connection_status = TRANSMITTING_CONTROL;
		return;
	}

	// SACK retransmit path (v1 only): send only the missing frames from last SACK
	// as a standalone retransmit-only batch.
	//
	// SACK Design A Step 8b — this early-return is the first of the two
	// CMD-side blockers identified by SACK_RETRANSMIT_BATCHING_INVESTIGATION.md
	// (mechanism (a)). Gated on `!sack_v2_enabled` so v1 sessions take this
	// path unchanged (v1 wire-byte-identical preserved per §5.1 gate #1).
	// On v2 sessions, do NOT take this early-return; the mixed-batch builder
	// below (§7.8b) prepends retx as the head of the next new-data batch,
	// amortizing per-cycle ARQ overhead across retx + new-data on the wire
	// (the throughput lever §7.8.4 named as the Step 8b goal).
	if(sack_enabled && !sack_v2_enabled && retransmit_count > 0)
	{
		printf("[CMD-RETX] Sending %d retransmit frames\n", retransmit_count);
		fflush(stdout);

		message_batch_counter_tx = 0;
		for(int r = 0; r < retransmit_count; r++)
		{
			// Bug B latent corollary (SACK_DESIGN_A_PLAN §7.13.19,
			// commit e73968a): messages_batch_tx[i].data is NULL at init
			// (arq_common.cc:1525 — only batch slot that is not pre-
			// allocated). In production today this site is unreachable
			// with .data NULL because every code path that enters the
			// retransmit loop has first gone through a struct-copy at
			// arq_commander.cc:590 / :655 / :992 / :1025 which inherits
			// the .data pointer from messages_control or messages_tx[i].
			// Defensive guard: convert a potential SIGSEGV into a logged
			// skip if that invariant is ever broken (e.g. a future code
			// path enters retx-only before any prior batch has dispatched).
			// Lazy-alloc was rejected because the next struct-copy after
			// the lazy allocation overwrites the .data pointer and leaks
			// the freshly-allocated buffer — the asymmetric allocation
			// pattern is the root anti-pattern, not the missing buffer.
			if(messages_batch_tx[message_batch_counter_tx].data == NULL)
			{
				printf("[CMD-LATENT-GUARD] retx slot %d/%d (site=v1-retx-only) "
					".data is NULL — skipping; investigate as a regression of "
					"the struct-copy-before-retx invariant (SACK_DESIGN_A_PLAN "
					"§7.13.19)\n", r, retransmit_count);
				fflush(stdout);
				continue;
			}
			messages_batch_tx[message_batch_counter_tx].type = retransmit_frame_types[r];
			messages_batch_tx[message_batch_counter_tx].length = retransmit_frame_lengths[r];
			memcpy(messages_batch_tx[message_batch_counter_tx].data,
				retransmit_frames[r], retransmit_frame_lengths[r]);
			messages_batch_tx[message_batch_counter_tx].id = retransmit_frame_positions[r];
			messages_batch_tx[message_batch_counter_tx].nResends = nResends;
			messages_batch_tx[message_batch_counter_tx].ack_timeout = ack_timeout_data;
			messages_batch_tx[message_batch_counter_tx].status = ADDED_TO_BATCH_BUFFER;
			// Use ORIGINAL sequence number so responder places frame in correct slot.
			// Do NOT set end-of-batch flag — RSP already has the expected count
			// from the original batch's compression header or end-of-batch frame.
			messages_batch_tx[message_batch_counter_tx].sequence_number = retransmit_frame_positions[r];
			// SACK Design A Step 3 — retransmit frames carry their ORIGINAL
			// batch_seq_id (the one the frame was first sent under). This
			// preserves the §4.3.4 invariant 2 "retransmits carry original
			// batch_seq_id, never current cmd_batch_seq_id". For v1 sessions
			// (sack_v2_enabled=false) the field is unused (not on the wire).
			messages_batch_tx[message_batch_counter_tx].batch_seq_id = retransmit_frame_batch_seq_ids[r];
			message_batch_counter_tx++;
			stats.nReSent_data++;
			last_transmission_block_stats.nReSent_data++;
		}
		if(sack_v2_enabled)
		{
			printf("[CMD-RETX-V2] retransmit batch carries original batch_seq_ids:");
			for(int r = 0; r < message_batch_counter_tx; r++)
				printf(" %d", messages_batch_tx[r].batch_seq_id);
			printf(" (current cmd_batch_seq_id=%d — NOT used for retransmits)\n",
				cmd_batch_seq_id & 0xFF);
			fflush(stdout);
		}
		retransmit_count = 0;  // Consumed

		// Mark retransmit frames as PENDING_ACK in messages_tx so ACK/SACK
		// detection can find them. Use the first N slots.
		for(int r = 0; r < message_batch_counter_tx; r++)
		{
			messages_tx[r] = messages_batch_tx[r];
			messages_tx[r].status = PENDING_ACK;
		}
		block_under_tx = YES;

		telecom_system->set_mfsk_ctrl_mode(false);
		pad_messages_batch_tx(message_batch_counter_tx); // No padding needed — exact count
		sack_retransmit_active = true;
		send_batch();
		sack_retransmit_active = false;

		stats.nBatches_sent++;
		last_transmission_block_stats.nBatches_sent++;

		// Post-TX flush handled inside send_batch() via rx_mute.

		if(ack_pattern_time_ms > 0)
			telecom_system->data_container.frames_to_read = 4;
		data_ack_received = NO;
		last_batch_fully_acked = false;  // CLEAN-BATCH VIABILITY (§9) — per-batch reset
		last_partial_lead_frame_only = false;  // ROLLING-PARTIAL unblock — fresh per batch
		clear_snr_arm_for_data_ack_wait(); // §18: arm MUST be false on every data-ACK wait
		connection_status = RECEIVING_ACKS_DATA;
		ack_diag_peak_matched = 0;
		ack_diag_peak_metric = 0.0;
		ack_diag_poll_count = 0;
		v2_ackpat_defer_count_this_window = 0;  // Bug A fix (§7.13.1)
		v2_dispatch_last_rwi = -1;               // §7.13.30 v2 dispatch throttle
		v2_dispatch_min_advance_syms = 1;        // §7.13.30 default throttle interval
		// WALL-B FIX-9 D2 REFINE (§2.1): this is the v1 retransmit-ONLY batch path -> the data-ACK
		// we are about to wait for IS a retransmit turnaround -> arm the D2 robust reverse-ACK widen
		// (calculate_receiving_timeout reads this flag). Set BEFORE the calc so the widen fires.
		data_ack_retx_turnaround = true;
		// Karn (MERCURY_KARN_RETX_ONLY): a v1 retransmit-only batch DOES carry retx frames ->
		// the turnaround is genuinely retransmit-ambiguous -> exclude its RTT sample.
		data_ack_round_carried_retx = true;
		calculate_receiving_timeout();
		// LINK-PHASE STEP 5 / MC-3: arm one close edge only when Step 2 supplied
		// a valid derived ACK_SLOT for this actual retransmit keydown.
		if(linkphase_slotliveness_on())
		{
			linkphase_ack_slot_wait_armed =
				linkphase_ackslot_on() && linkphase_last_kd_frames > 0;
			linkphase_ack_slot_miss_counted = false;
		}
		printf("[CMD-POST-TX] receiving_timeout=%dms msg_tx_time=%dms batch=%d sack=%d\n",
			receiving_timeout, message_transmission_time_ms, data_batch_size, sack_enabled ? 1 : 0);
		fflush(stdout);
		receiving_timer.start();
		return;
	}

	// SACK Design A Step 8b — v2 mixed-batch retx prefix builder.
	//
	// On v2 sessions with a populated retransmit queue, prepend the retx
	// frames as the head of the next batch (retransmits-first, then new-data).
	// Each retx carries its ORIGINAL batch_seq_id (captured at Step 3 into
	// retransmit_frame_batch_seq_ids[]) so RSP's match-prev branch routes them
	// into messages_rx_prev[] (Step 8a's parallel storage). New-data follows
	// with the current cmd_batch_seq_id and lands in messages_rx[] via
	// match-current.
	//
	// The two CMD-side blockers identified by SACK_RETRANSMIT_BATCHING_INVESTIGATION.md
	// (the v1 retransmit-only early-return above + the new-data staging
	// gate in process_buffer_data_commander()) are now both lifted for v2.
	// The mixed batch eliminates the per-cycle standalone-retransmit ARQ
	// overhead (§7.8.4 / §7.2 of the investigation doc), which is the
	// throughput lever Step 8b engages.
	//
	// §4.3.4 invariants honored:
	//   #1 (one outstanding batch): connection_status state machine still
	//      gates TX (TRANSMITTING_DATA → RECEIVING_ACKS_DATA → ack/sack →
	//      TRANSMITTING_DATA); see arq_commander.cc:347-365. CMD does not
	//      key N+2 before N+1 has been ACK/SACK'd. The mixed batch is one
	//      TX containing "complete N (via retx) + start N+1 (via new-data)".
	//   #2 (batch_seq_id monotonicity): retx carries original bsi (not
	//      cmd_batch_seq_id); new-data fill loop below assigns current
	//      cmd_batch_seq_id; cmd_batch_seq_id increment happens AFTER
	//      send_batch only if batch_includes_new_data (unchanged).
	//   #3 (no silent corruption): retx routes to messages_rx_prev[] via
	//      Step 8a's match-prev path; new-data routes to messages_rx[]
	//      via match-current. Different physical buffers — no cross-batch
	//      slot collisions (§7.8.3's hazard structurally eliminated).
	// R030 (race audit 2026-06-06): the retx-block descriptor is a MEMBER (was a
	// local) so the post-TX PENDING_ACK flip in send_batch() can see which
	// messages_batch_tx[] entries are the retx block. Reset here at the start of
	// every batch build; set below only on a v2 mixed batch.
	// (v2_mixed_batch stays local — only this function needs it.)
	// Timing redesign — the block is described by (start,count) rather than a bare
	// leading count, because v2_rotate_retx_behind_lead() may move it to start=1.
	v2_retx_block_start = 0;
	v2_retx_block_count = 0;
	bool v2_mixed_batch = false;

	// §7.13.39 Fix 2 — single source of truth for batch_tx slot identity.
	// Both the retx prefix loop and the new-data fill loop must agree on the
	// formula for (sequence_number, id, batch_seq_id) so a future edit to one
	// can't drift out of sync with the other. The helper does NOT touch
	// .data/.length/.type — those are loop-specific. See assertion sweep
	// below send_batch() for the duplicate-tuple guard.
	//
	// Inputs:
	//   batch_idx          — slot in messages_batch_tx[] being populated (0..data_batch_size-1)
	//   is_retx            — true for retx prefix, false for new-data fill
	//   original_seq_eob   — for retx: the captured ORIGINAL sequence_number byte
	//                        (low 7 bits = original slot, bit 7 = EOB on first send).
	//                        Ignored when is_retx=false.
	//   slot_in_new_batch  — for new-data: position in the new batch (0..ND-1).
	//                        Ignored when is_retx=true.
	//   bsi                — batch_seq_id to assign (retx: original; new-data: current).
	auto set_batch_tx_slot = [&](int batch_idx, bool is_retx,
	                             unsigned char original_seq_eob,
	                             int slot_in_new_batch, int bsi)
	{
		if(is_retx)
		{
			// Carry the original sequence_number byte verbatim — preserves both
			// the original slot (low 7 bits) AND the EOB bit (bit 7) per
			// §7.13.39 Fix 3. RSP routes via messages_rx_prev[loc=low7] and
			// receives the EOB marker if it was set on first send.
			messages_batch_tx[batch_idx].sequence_number = (int)original_seq_eob;
			messages_batch_tx[batch_idx].id = (int)(original_seq_eob & 0x7F);
		}
		else
		{
			messages_batch_tx[batch_idx].sequence_number = slot_in_new_batch;
			messages_batch_tx[batch_idx].id = slot_in_new_batch;
		}
		messages_batch_tx[batch_idx].batch_seq_id = bsi;
	};

	if(sack_v2_enabled && retransmit_count > 0)
	{
		// §7.13.39 Fix 1 safety net (PRE-POP check) — if the queue is already
		// >= 2*data_batch_size BEFORE we even try to drain it via this batch,
		// the channel has collapsed and trying to drain via mixed-batches will
		// only widen the gap. The pre-pop check (rather than post-pop) lets us
		// trigger BREAK even if MAX_RETRANSMIT_HEADROOM is exactly 2*batch:
		// the queue can fill to the cap but the safety net still fires before
		// silent dropping at the SACK_RSP capture site (where the bound check
		// `retransmit_count < MAX_RETRANSMIT_HEADROOM` would otherwise hide
		// the runaway). Comparison is >= so the equal-to-cap edge case
		// triggers cleanly. Pattern follows the BREAK trigger at :~2370.
		if(retransmit_count >= 2 * data_batch_size
		   && !emergency_break_active
		   && turboshift_phase == TURBO_DONE
		   && gear_shift_on == YES)
		{
			// STAGE 4c — Class-A site A1 (retx-queue runaway). Under inband, the collapsed
			// retx queue DEMOTES one rung via the CONFIG_TAG (the demote's clear_retx_queue
			// flushes the runaway queue) instead of BREAKing; at the bottom it ticks the
			// true-loss floor and BREAKs only at SESSION_DEAD_BATCHES. Byte-identical OFF.
			if(inband_rate_feature_enabled())
			{
				int working_config = config_ladder_down(data_configuration, robust_enabled);
				if(working_config != data_configuration
				   && inband_route_failure_demote(working_config, "retx_queue_runaway"))
					return;
				// At the bottom (no lower rung): tick the dead-batch floor; BREAK only at N.
				if(inband_cmd_dead_batch_floor_reached())
				{
					restage_requeue_tx_messages();   // §12 lossless+ordered re-stage (was forward push()-to-BACK)
					fifo_buffer_backup.flush();
					block_under_tx = NO;
					retransmit_count = 0;
					emergency_previous_config = current_configuration;
					emergency_break_active = 1;
					emergency_break_retries = 3;
					emergency_nack_count = 0;
					if(sack_v2_enabled)
						policy_axis1_supremacy_on_move(current_configuration,
							working_config, "inband_session_dead_batches_floor");
					send_break_pattern();
					telecom_system->data_container.frames_to_read = 4;
					calculate_receiving_timeout();
					receiving_timer.start();
				}
				return;
			}

			printf("[BREAK] retx queue runaway (count=%d, threshold=2*batch=%d) — "
				"channel collapsed, forcing BREAK\n",
				retransmit_count, 2 * data_batch_size);
			fflush(stdout);
			int working_config = config_ladder_down(data_configuration, robust_enabled);
			// Push pending payloads back to the FIFO for resend after BREAK
			// recovery (parallels the gearshift BREAK path at :2344-2349).
			restage_requeue_tx_messages();   // §12 lossless+ordered re-stage (was forward push()-to-BACK)
			fifo_buffer_backup.flush();
			block_under_tx = NO;
			// Clear the runaway queue — the BREAK path restarts at a working
			// config; the retx queue's contents (encrypted under the old config's
			// crypto batch) are no longer meaningful after BREAK.
			retransmit_count = 0;
			data_configuration = working_config;
			negotiated_configuration = working_config;
			emergency_previous_config = working_config;
			break_drop_step = 0;
			emergency_break_active = 1;
			emergency_break_retries = 3;
			emergency_nack_count = 0;
			if(sack_v2_enabled)
				policy_axis1_supremacy_on_move(current_configuration,
					working_config, "retx_queue_runaway");
			send_break_pattern();
			telecom_system->data_container.frames_to_read = 4;
			calculate_receiving_timeout();
			receiving_timer.start();
			return;
		}

		v2_mixed_batch = true;
		// §7.13.39 Fix 1 — never drop. Fill R = min(count, batch_size) retx
		// from the head of the queue; survivors at [R..count-1] shift down so
		// the next batch consumes them first.
		int R = retransmit_count;
		if(R > data_batch_size) R = data_batch_size;
		message_batch_counter_tx = 0;
		for(int r = 0; r < R; r++)
		{
			// Bug B latent corollary — see arq_commander.cc:832 above.
			// Same defensive guard for the v2 mixbatch retx prefix.
			if(messages_batch_tx[message_batch_counter_tx].data == NULL)
			{
				printf("[CMD-LATENT-GUARD] retx slot %d/%d (site=v2-mixbatch-retx) "
					".data is NULL — skipping; investigate as a regression of "
					"the struct-copy-before-retx invariant (SACK_DESIGN_A_PLAN "
					"§7.13.19)\n", r, R);
				fflush(stdout);
				continue;
			}
			messages_batch_tx[message_batch_counter_tx].type = retransmit_frame_types[r];
			messages_batch_tx[message_batch_counter_tx].length = retransmit_frame_lengths[r];
			// BUFFER-ALIAS FIX: don't memcpy into messages_batch_tx[r].data —
			// that pointer aliases messages_tx[r].data (from a prior cycle's
			// struct-copy at line ~1225), and clobbering it would corrupt
			// the new-data buffer that the upcoming new-data loop will struct-
			// copy out of messages_tx[r] into messages_batch_tx[r+R]. Instead,
			// memcpy into a dedicated per-instance scratch buffer and point
			// messages_batch_tx[r].data at the scratch slot. Runtime-confirmed
			// via [TX-ALIAS-CHECK-POST] = YES-BUG before this fix.
			memcpy(retx_scratch[r], retransmit_frames[r], retransmit_frame_lengths[r]);
			messages_batch_tx[message_batch_counter_tx].data = (char*)retx_scratch[r];
			messages_batch_tx[message_batch_counter_tx].nResends = nResends;
			messages_batch_tx[message_batch_counter_tx].ack_timeout = ack_timeout_data;
			messages_batch_tx[message_batch_counter_tx].status = ADDED_TO_BATCH_BUFFER;
			// §7.13.39 Fix 2 + Fix 3 — slot identity assigned via the helper.
			// Carries the original sequence_number byte (preserves EOB bit 7
			// per Fix 3) and the original batch_seq_id (§4.3.4 invariant 2).
			set_batch_tx_slot(message_batch_counter_tx,
				/*is_retx=*/true,
				/*original_seq_eob=*/retransmit_frame_seq_with_eob[r],
				/*slot_in_new_batch=*/0,  // unused for retx
				/*bsi=*/retransmit_frame_batch_seq_ids[r]);
			// R2c — stash this just-SENT retx frame in the retention shadow, keyed
			// by its ORIGINAL bsi + slot, so a later prev-bsi re-SACK can re-drive
			// it if it is lost again (the retx queue slot is consumed on send).
			cmd_prev_retain_capture(retransmit_frame_batch_seq_ids[r],
				retransmit_frame_positions[r], retransmit_frame_lengths[r],
				retransmit_frame_types[r], retransmit_frame_seq_with_eob[r],
				retransmit_frames[r]);
			message_batch_counter_tx++;
			stats.nReSent_data++;
			last_transmission_block_stats.nReSent_data++;
		}
		// Retx block occupies the LEADING R slots at fill time. If the batch also
		// carries >=2 new-data frames, v2_rotate_retx_behind_lead() (after the
		// new-data fill) shifts start to 1 so wire slot 0 is a new-data frame.
		v2_retx_block_start = 0;
		v2_retx_block_count = R;

		// §7.13.39 Fix 1 — shift survivors down. memmove on each parallel
		// array. Leftover = retransmit_count - R; these stay at [0..leftover-1]
		// and will be sent at the head of the NEXT batch (FIFO order
		// preserved). The SACK_RSP capture site is APPEND-mode on v2 (see
		// arq_commander.cc:~1962), so any new retx captured between now and
		// the next batch is concatenated after these survivors.
		int leftover = retransmit_count - R;
		if(leftover > 0)
		{
			memmove(&retransmit_frames[0],            &retransmit_frames[R],            leftover * sizeof(retransmit_frames[0]));
			memmove(&retransmit_frame_lengths[0],     &retransmit_frame_lengths[R],     leftover * sizeof(retransmit_frame_lengths[0]));
			memmove(&retransmit_frame_positions[0],   &retransmit_frame_positions[R],   leftover * sizeof(retransmit_frame_positions[0]));
			memmove(&retransmit_frame_types[0],       &retransmit_frame_types[R],       leftover * sizeof(retransmit_frame_types[0]));
			memmove(&retransmit_frame_batch_seq_ids[0], &retransmit_frame_batch_seq_ids[R], leftover * sizeof(retransmit_frame_batch_seq_ids[0]));
			memmove(&retransmit_frame_seq_with_eob[0], &retransmit_frame_seq_with_eob[R], leftover * sizeof(retransmit_frame_seq_with_eob[0]));
		}
		retransmit_count = leftover;

		// Per-frame retx bsi log (for [CMD-V2-MIXBATCH] post-TX summary)
		printf("[CMD-V2-MIXBATCH-RETX] R=%d (bsi=", R);
		for(int r = 0; r < R; r++)
			printf(" %d", messages_batch_tx[r].batch_seq_id);
		printf(") of %d total queued; %d requeued for next batch (data_batch_size=%d)\n",
			R + leftover, leftover, data_batch_size);
		fflush(stdout);
	}

	// SACK Design A Step 3 — batch_seq_id assignment on new-data batch build.
	// New-data frames (ADDED_TO_LIST) get assigned cmd_batch_seq_id (the
	// counter for the batch they're about to ride). Retransmit frames
	// (ACK_TIMED_OUT) keep their existing batch_seq_id (set on first send) —
	// retransmits MUST carry their ORIGINAL batch_seq_id per §4.3.4 invariant 2.
	// The counter increments after the batch is dispatched, and only if at
	// least one new-data frame was included (so a pure-resends-no-new-data
	// batch in this mixed path does not bump the counter — that path is
	// rare because the SACK retransmit-only path above handles the common case).
	//
	// ID-SKEW ROOT FIX (§87): before assigning batch positions, rebase a fresh non-mixed
	// new-data batch's frames down to messages_tx slots [0, ND) so the wire id (= slot) equals
	// the batch position the receiver stores at. This applies to v1 and v2: both serialize .id
	// and store by it. The v2 mixed path already assigns the ordinal id
	// via set_batch_tx_slot, so skip it there. No-op / byte-identical for an already-contiguous
	// batch. See cmd_rebase_newdata_slots() above for the full mechanism + invariant.
	if(!v2_mixed_batch)
		cmd_rebase_newdata_slots();
	bool batch_includes_new_data = false;
	// A scalable partial round drains the outstanding generation before any
	// fresh application frames are admitted. This prevents a second receive
	// generation from forcing the responder's prev-retention discard path.
	bool admit_new_generation = !(scalable_sack_on() && v2_mixed_batch);
	int last_new_data_messages_tx_idx = -1;  // §7.13.39 Fix 3 — for EOB mirror
	for(int i=0;i<this->nMessages;i++)
	{
		if(messages_tx[i].status==ADDED_TO_LIST && admit_new_generation)
		{
			if(message_batch_counter_tx<data_batch_size)
			{
				// Assign the current new-data batch's batch_seq_id.
				messages_tx[i].batch_seq_id = (cmd_batch_seq_id & 0xFF);
				messages_batch_tx[message_batch_counter_tx]=messages_tx[i];
				last_new_data_messages_tx_idx = i;  // track for EOB mirror
				// §7.13.39 Fix 2 — on v2 mixed batches, route slot identity
				// through the single helper so the formula matches the retx
				// prefix loop above. The collision validation pass below
				// send_batch() catches any drift. For v2 non-mixed batches
				// (retransmit_count was 0), v2_retx_block_count == 0, so the
				// helper assigns the same value the struct-copy already
				// provided (id = messages_tx slot index = message_batch_counter_tx).
				if(v2_mixed_batch)
				{
					// pos_in_new_batch is 0..ND-1 in ARRIVAL order; the retx block
					// is contiguous at [0..R-1] here (pre-rotation), so subtracting
					// the count yields the correct new-data ordinal. The subsequent
					// rotation permutes physical slots only — these sequence bytes
					// (which travel with the struct) stay position-independent.
					int pos_in_new_batch = message_batch_counter_tx - v2_retx_block_count;
					set_batch_tx_slot(message_batch_counter_tx,
						/*is_retx=*/false,
						/*original_seq_eob=*/0,  // unused for new-data
						/*slot_in_new_batch=*/pos_in_new_batch,
						/*bsi=*/(cmd_batch_seq_id & 0xFF));
					// §7.13.39 Fix 3 — also write the slot/EOB byte BACK to
					// messages_tx[i] so that if this frame is later listed in
					// a SACK_RSP as missing, the capture site (~line 1962+) can
					// read messages_tx[i].sequence_number and preserve the
					// original byte (slot in low7 + EOB in bit 7) into the
					// retx queue. Without this write-back, sequence_number on
					// messages_tx[i] is uninitialized — losing the EOB info.
					// EOB bit is applied later (line ~1230) on
					// messages_batch_tx[last_idx]; mirror that here for the
					// final new-data slot.
					messages_tx[i].sequence_number = pos_in_new_batch;
					messages_tx[i].id = pos_in_new_batch;
				}
				else if(sack_v2_enabled)
				{
					// v2 non-mixed: send_batch() will renumber sequence_number
					// = message_batch_counter_tx and OR 0x80 on the last frame.
					// Mirror that to messages_tx[i] for §7.13.39 Fix 3.
					messages_tx[i].sequence_number = message_batch_counter_tx;
				}
				message_batch_counter_tx++;
				messages_tx[i].status=ADDED_TO_BATCH_BUFFER;
				batch_includes_new_data = true;
				stats.nSent_data++;
				last_transmission_block_stats.nSent_data++;
				// IDLE-SWITCHROLE-RACE (idle-switchrole-race.md §2): this session
				// has now emitted a NEW data frame -> the idle SWITCH_ROLE handoff
				// is legitimate from here on (end-of-data handoff preserved). The
				// gate in process_buffer_data_commander() suppresses the handoff
				// only while this is false (never-fed Commander = the race).
				session_data_frame_sent = true;
			}
		}
		else if(messages_tx[i].status==ACK_TIMED_OUT)
		{
			if(--messages_tx[i].nResends>0)
			{
				if(message_batch_counter_tx<data_batch_size)
				{
					// Retransmit: keep existing batch_seq_id (original value
					// from first send). Do not reassign.
					messages_batch_tx[message_batch_counter_tx]=messages_tx[i];
					message_batch_counter_tx++;
					messages_tx[i].status=ADDED_TO_BATCH_BUFFER;
					stats.nReSent_data++;
					last_transmission_block_stats.nReSent_data++;
				}
			}
			else
			{
				stats.nLost_data++;
				messages_tx[i].status=FAILED_;
			}

		}

		if(message_batch_counter_tx==data_batch_size)
		{
			break;
		}
	}
	if(message_batch_counter_tx<=data_batch_size && message_batch_counter_tx!=0)
	{
		// Timing redesign — retx never rides wire slot 0. The fill loops leave the
		// retx block at the LEADING slots [0..R-1]; slot 0 is the only slot exposed
		// to the peer's post-turnaround mute/flush window (a width-sweep observed
		// slot-0 loss with slots 1+ clean). Rotate the sub-array [0..R] right by one
		// so wire slot 0 becomes the first new-data frame and the retx block moves
		// to [1..R]. The EOB-marked final new-data frame is at
		// message_batch_counter_tx-1 (>= R+1 when ND>=2) which the rotation never
		// touches, so it stays wire-final (the RSP treats an EOB decode as proof the
		// batch ended and shortens its receive window — a retx tail after it would
		// collide with the RSP keying its SACK). No-op (legacy layout) for non-mixed
		// v2, v1, robust batch=1, pure-retx (ND==0), and ND==1 (its single new-data
		// frame IS the EOB frame and must stay wire-final). Runs BEFORE the EOB
		// assignment below so v2_slot_is_retx() reflects the rotated layout.
		v2_rotate_retx_behind_lead();
		telecom_system->set_mfsk_ctrl_mode(false);  // data TX (full-length frames)
		// SACK Design A Step 8b — for v2 mixed batches, set EOB bit on the
		// last new-data frame manually (since we use sack_retransmit_active=true
		// below to suppress send_batch's renumbering loop, which is where the
		// EOB bit is normally set). For retx-only fallback (R == data_batch_size,
		// no new-data), no EOB bit — matches the v1 retransmit-only path
		// (which also omits EOB; RSP infers prev batch size from the original
		// batch's compression header or EOB on the original transmission).
		if(v2_mixed_batch)
		{
			int last_idx = message_batch_counter_tx - 1;
			// Migrated to the position-independent predicate: after the rotation
			// the retx block is at [1..R], so a bare "last_idx >= R" would be wrong.
			// The final slot is always the last new-data frame (the rotation permutes
			// only slots [0..R]), so !v2_slot_is_retx(last_idx) holds and the EOB bit
			// lands on it — wire-final in every batch shape.
			if(!v2_slot_is_retx(last_idx)
			   && (messages_batch_tx[last_idx].type == DATA_LONG
			       || messages_batch_tx[last_idx].type == DATA_SHORT))
			{
				// Set EOB bit on the last NEW-DATA frame.
				messages_batch_tx[last_idx].sequence_number |= 0x80;
				// §7.13.39 Fix 3 — mirror the EOB bit to messages_tx so the
				// SACK_RSP capture site can preserve it into the retx queue
				// if this last-frame is later listed as missing.
				if(last_new_data_messages_tx_idx >= 0)
					messages_tx[last_new_data_messages_tx_idx].sequence_number |= 0x80;
			}
		}
		else if(!sack_v2_enabled)
		{
			// v1 only: pad to full data_batch_size. The legacy ACK_MULTI
			// bitmap path assumed every batch carries exactly data_batch_size
			// frames. v2 uses the EOB bit on the last frame plus
			// last_received_end_of_batch_seq on the RX side to derive the
			// real effective batch size, so v2 does not need padding.
			pad_messages_batch_tx(data_batch_size);
		}
		else
		{
			// v2 non-mixed: do NOT pad. Padded frames are wire-duplicates of
			// earlier slots with new (unique) slot ids — when a padded slot is
			// lost on the wire, the SACK_RSP bitmap reports it missing, but
			// messages_tx[padded_id] holds init values (length=0, bsi=-1) so
			// retx capture can't recover real bytes. With the 76f6185 padded-
			// slot guard the retx is skipped, but then prev_batch_received
			// stays below expected forever — RX never decompresses the prev
			// batch, the streaming PPMd model on RX falls one batch behind
			// the TX model, and every subsequent batch's decompress fails.
			// Skipping padding entirely fixes this by ensuring every
			// transmitted slot maps to a real messages_tx entry.
			//
			// EOB / batch-size signalling: send_batch at arq_common.cc:3336-3338
			// sets EOB bit on messages_batch_tx[message_batch_counter_tx - 1]
			// (the actual last new-data frame), and last_received_end_of_batch_seq
			// on the RX side resolves effective_batch from the EOB-bearing
			// frame's sequence_number, not from data_batch_size. Verified
			// safe via arq_responder.cc:599-604 + 1163-1170.
			//
			// §7.13.39 Fix 3 — mirror EOB bit to messages_tx so a later
			// SACK_RSP capture preserves the EOB marker on the retx queue
			// entry for this slot.
			if(last_new_data_messages_tx_idx >= 0)
			{
				int last_idx = message_batch_counter_tx - 1;
				if(last_idx >= 0
				   && (messages_batch_tx[last_idx].type == DATA_LONG
				       || messages_batch_tx[last_idx].type == DATA_SHORT))
				{
					messages_tx[last_new_data_messages_tx_idx].sequence_number |= 0x80;
				}
			}
		}
		mtl::log_event_kv("cmd_batch_tx_start", "batch=%lld nframes=%d cfg=%d",
		                  stats.nBatches_sent + 1, data_batch_size, current_configuration);
		// Start one Gearshift-v2 transaction interval and capture its attempt
		// lineage. The ACK/no-ACK close records the complete cycle directly, so
		// terminal batches never depend on a later batch to supply elapsed time.
		opt_on_batch_tx_start((unsigned int)message_batch_counter_tx,
			(unsigned int)std::max(0, std::min(v2_retx_block_count, message_batch_counter_tx)));
		if(sack_v2_enabled && batch_includes_new_data)
		{
			printf("[CMD-BATCH-SEQ] new-data batch_seq_id=%d (frames in batch=%d)\n",
				cmd_batch_seq_id & 0xFF, message_batch_counter_tx);
			fflush(stdout);
		}
		// SACK Design A Step 8b — v2 mixed batch summary log.
		// Demonstrates the engaged lever: BOTH retx (with original bsi) AND
		// new-data (with current bsi) in the same TX. Per the prompt's Gate 3:
		// "TX batch: N retx bsi=X + M new bsi=Y" — log present means lever
		// engaged; log absent on no-loss path (mixed-batch builder is not
		// entered when retransmit_count == 0).
		if(v2_mixed_batch)
		{
			int new_data_count = message_batch_counter_tx - v2_retx_block_count;
			// After a rotate the retx block starts at v2_retx_block_start (1), not 0,
			// so read the block head, not the fixed slot 0 (which is now new-data).
			int retx_bsi = (v2_retx_block_count > 0) ? messages_batch_tx[v2_retx_block_start].batch_seq_id : -1;
			printf("[CMD-V2-MIXBATCH] TX batch: %d retx bsi=%d + %d new bsi=%d = %d total "
				"(data_batch_size=%d)\n",
				v2_retx_block_count, retx_bsi,
				new_data_count, cmd_batch_seq_id & 0xFF,
				message_batch_counter_tx, data_batch_size);
			fflush(stdout);
			// Suppress send_batch's sequence_number renumbering (we've pre-
			// assigned them). For pure-retx fallback (no new-data), this
			// matches the v1 retransmit-only path's use of the same flag.
			sack_retransmit_active = true;

			// §7.13.39 Fix 2 — duplicate (bsi, seq) tuple validation. Two
			// frames in the same TX with identical (batch_seq_id, low-7-seq)
			// would alias on RSP's match-prev/match-current routing and
			// silently corrupt one of them. Sweep all populated slots; if any
			// collision is found, ABORT the TX (skip send_batch). Better a
			// missed batch than a corrupted one — the next SACK_RSP / timeout
			// path will resync. The mask matches the wire format: bit 7 of
			// sequence_number is the EOB flag, low 7 bits are the slot id.
			bool collision_found = false;
			for(int ii = 0; ii < message_batch_counter_tx && !collision_found; ii++)
			{
				for(int jj = ii + 1; jj < message_batch_counter_tx; jj++)
				{
					if(messages_batch_tx[ii].batch_seq_id == messages_batch_tx[jj].batch_seq_id
					   && (messages_batch_tx[ii].sequence_number & 0x7F)
					      == (messages_batch_tx[jj].sequence_number & 0x7F))
					{
						printf("[BATCH-SLOT-COLLISION] i=%d j=%d bsi=%d seq=%d — "
							"aborting TX (§7.13.39 Fix 2)\n",
							ii, jj,
							messages_batch_tx[ii].batch_seq_id,
							messages_batch_tx[ii].sequence_number & 0x7F);
						fflush(stdout);
						collision_found = true;
						break;
					}
				}
			}
			if(collision_found)
			{
				printf("[BATCH-SLOT-COLLISION] WARNING — this indicates a "
					"programming error in slot-id assignment (retx prefix vs "
					"new-data fill drift). Skipping send_batch() to avoid "
					"silent wire corruption. The retx prefix payloads consumed "
					"this cycle are lost; the next SACK_RSP / timeout will "
					"resync. File a bug against §7.13.39 Fix 2.\n");
				fflush(stdout);
				sack_retransmit_active = false;
				// Roll back stats counters bumped during slot population so
				// the bug doesn't masquerade as successful sends.
				stats.nReSent_data -= v2_retx_block_count;
				last_transmission_block_stats.nReSent_data -= v2_retx_block_count;
				int new_data_in_batch = message_batch_counter_tx - v2_retx_block_count;
				stats.nSent_data -= new_data_in_batch;
				last_transmission_block_stats.nSent_data -= new_data_in_batch;
				// Note: messages_tx[i].status entries flipped to
				// ADDED_TO_BATCH_BUFFER are NOT rolled back; they will be
				// cleaned up by the next process tick. This is intentional:
				// resending a batch with a known slot collision would just
				// re-trigger the assertion. The bug must be fixed in code.
				message_batch_counter_tx = 0;
				return;
			}
		}
		const uint8_t l1_block_tx_bsi = (uint8_t)(cmd_batch_seq_id & 0xFF);
		bool l1_block_tail_candidate = false;
		if(l1_blockack_data_active() && batch_includes_new_data)
		{
			bool staged_more = false;
			for(int i=0;i<nMessages;i++)
				if(messages_tx[i].status == ADDED_TO_LIST) staged_more = true;
			const int fifo_occupied = fifo_buffer_tx.get_size()
				- fifo_buffer_tx.get_free_size();
			l1_block_tail_candidate = !staged_more && fifo_occupied == 0;
		}
		send_batch();
		if(v2_mixed_batch)
		{
			sack_retransmit_active = false;
		}
		mtl::log_event_kv("cmd_batch_tx_done", "batch=%lld", stats.nBatches_sent + 1);
		// SACK Design A Step 3 — increment cmd_batch_seq_id mod 256 only if
		// this batch actually carried new-data frames. §4.3.4 invariant 2:
		// the counter is independent of set_data_batch_size() — that function
		// only mutates data_batch_size and never touches cmd_batch_seq_id.
		if(batch_includes_new_data)
		{
			cmd_batch_seq_id = (cmd_batch_seq_id + 1) & 0xFF;
			if(l1_blockack_data_active())
			{
				// STAGE R accumulator: forward-burst airtime in the current block.
				// Reset when a fresh block starts (before this batch neither silence
				// window is open). Estimated as per-frame tx time * batch frame count.
				if(!l1_blockack.intermediate_silence_expected()
				   && !l1_blockack.aggregate_response_outstanding())
					block_forward_air_ms = 0;
				block_forward_air_ms += (long long)message_transmission_time_ms
					* (long long)(data_batch_size > 0 ? data_batch_size : 1);
				l1_blockack.note_transmitted_batch(l1_block_tx_bsi);
				// Force the block boundary early when continuing to accumulate silence
				// would push the reverse gap toward link_timeout (the watchdog contract).
				// Pipeline-gated; legacy block-ACK leaves this false => byte-identical.
				bool l1_pipeline_wd_boundary =
					l1_pipeline_watchdog_would_force(block_forward_air_ms);
				if((l1_block_tail_candidate || l1_pipeline_wd_boundary)
				   && l1_blockack.intermediate_silence_expected())
				{
					std::vector<uint8_t> commit;
					if(l1_blockack.build_tail_commit(&commit))
					{
						l1_send_block_control(commit);
						printf(l1_pipeline_wd_boundary
							? "[L1-BLOCKACK-PIPELINE] watchdog boundary forced bsi=%u\n"
							: "[L1-BLOCKACK-CMD] tail commit after bsi=%u\n",
							(unsigned)l1_block_tx_bsi);
						fflush(stdout);
					}
				}
			}
		}
		stats.nBatches_sent++;
		last_transmission_block_stats.nBatches_sent++;

		// Post-TX flush handled inside send_batch() via rx_mute.

		if(ack_pattern_time_ms > 0 || l1_blockack_data_active())
		{
			// Expect ACK tone pattern — start polling quickly (same as control path).
			telecom_system->data_container.frames_to_read = 4;
		}
		else
		{
			// Fallback: expect short LDPC ctrl frame
			telecom_system->set_mfsk_ctrl_mode(true);
			telecom_system->data_container.frames_to_read =
				telecom_system->data_container.preamble_nSymb + telecom_system->get_active_nsymb();
		}
		data_ack_received=NO;
		last_batch_fully_acked = false;  // CLEAN-BATCH VIABILITY (§9) — per-batch reset
		last_partial_lead_frame_only = false;  // ROLLING-PARTIAL unblock — fresh per batch
		clear_snr_arm_for_data_ack_wait(); // §18: arm MUST be false on every data-ACK wait
		connection_status=RECEIVING_ACKS_DATA;
		// RECOVERY-ACK robustness (§6.2): the DATA-ACK turnaround stays single-block
		// (the RSP data-ACK arm does not repeat). Clear any rep count left set by a
		// prior control-ACK wait so the data turnaround correlates combine_reps=1.
		// No-op when robust is off (reps already 1) → byte-identical.
		set_recovery_ack_reps_for_wait(/*control_ack=*/false);
		ack_diag_peak_matched = 0;
		ack_diag_peak_metric = 0.0;
		ack_diag_poll_count = 0;
		v2_ackpat_defer_count_this_window = 0;  // Bug A fix (§7.13.1)
		v2_dispatch_last_rwi = -1;               // §7.13.30 v2 dispatch throttle
		v2_dispatch_min_advance_syms = 1;        // §7.13.30 default throttle interval
		// A wide partial report rides the fixed robust full-width SACK_RSP
		// configuration. This changes only the PHY decoder; current_configuration
		// remains the live data rung for every ARQ and stale-partial decision.
		arm_sack_v2_robust_rx();
		// ACK pattern detection uses dedicated ack_mfsk — no config switch needed
		if(ack_pattern_time_ms <= 0)
			load_configuration(ack_configuration, PHYSICAL_LAYER_ONLY,NO);
		// WALL-B FIX-9 D2 REFINE (§2.1): this is the new-data / v2-mixed batch path. The data-ACK we
		// are about to wait for is a RETRANSMIT turnaround iff the batch carried a retx prefix
		// (v2_mixed_batch) OR a recent CFG16 reverse-ACK was lost (cfg16_revack_starve_fails>0 — arm
		// the geometry on a degrading ceiling even if THIS batch is pure new-data).
		//
		// WALL-B FIX-9 H1 (data-flow-revack-turnaround-geometry.md §5.1, INV-1 lockstep): ALSO pre-arm
		// the widen on a FRESH first-pass OFDM batch big enough to go PARTIAL. The D2-REFINE gate
		// above (v2_mixed || starve>0) is FALSE on the first partial of a fresh CFG16 batch, yet the
		// RSP arms its pre-TX settle on that partial (arq_responder.cc:1789) -> the settled partial
		// SACK lands at/outside the still-NARROW CMD window -> the silent reverse-ACK miss that
		// accumulates the D3 starvation deadline (the ~133s clean CFG16 demote). A missed partial is
		// NOT a full block-failure, so cfg16_revack_starve_fails does NOT increment — the D2-REFINE
		// degrading-ceiling arm only fires AFTER a full failure, too late. H1 restores lockstep BEFORE
		// the partial is known: on any OFDM forward config with a multi-frame batch (where a partial
		// SACK is possible) the CMD window is pre-widened by ROBUST_ACK_DRIFT_MARGIN_MS (~600ms,
		// bounded cost, recovered many-fold vs a D3 demote+re-climb of ~25-40s of wire). On a clean
		// FULL OFDM batch the RSP stays narrow (arq_responder.cc:1980), so the CMD is merely over-wide
		// (cost, not correctness). Robust/NB are UNTOUCHED: the new disjunct is is_ofdm_config-gated
		// and a single-frame batch (cannot go partial) stays below BATCH_MAY_BE_PARTIAL_THRESHOLD.
		// FAIL-BEFORE (-DH1_FAILBEFORE): drop the is_ofdm disjunct -> the fresh-partial widen does not
		// fire -> Part W2e fails (lockstep INV-1 violated).
		data_ack_retx_turnaround = (v2_mixed_batch || cfg16_revack_starve_fails > 0
#ifndef H1_FAILBEFORE
			|| (is_ofdm_config(current_configuration)
			    && data_batch_size >= BATCH_MAY_BE_PARTIAL_THRESHOLD)
#endif
			);
		// Karn (MERCURY_KARN_RETX_ONLY): the round carries retx frames iff this is a v2
		// mixed batch (a retx prefix ahead of the new data). A clean first-pass batch — the
		// H1 partial-capable widen included — carries NO retx frames, so its RTT sample is a
		// valid Karn sample (starve>0 is handled belt-and-braces inside tt_karn_sample_ok).
		data_ack_round_carried_retx = v2_mixed_batch;
		// Recalculate timeout: guard delays from prior ACK detection can leave
		// receiving_timeout stale, too short for the next ACK round-trip.
		calculate_receiving_timeout();
		// LINK-PHASE STEP 5 / MC-3: the slot is valid only when the shipped
		// derived ACK_SLOT floor owns this data-ACK wait. OFF/DEFEAT writes nothing.
		if(linkphase_slotliveness_on())
		{
			linkphase_ack_slot_wait_armed =
				linkphase_ackslot_on() && linkphase_last_kd_frames > 0;
			linkphase_ack_slot_miss_counted = false;
		}
		printf("[CMD-POST-TX] receiving_timeout=%dms msg_tx_time=%dms batch=%d sack=%d\n",
			receiving_timeout, message_transmission_time_ms, data_batch_size, sack_enabled ? 1 : 0);
		fflush(stdout);
		// STAGE R boundary witness: the ONE real receive window per block (the aggregate
		// BLOCK_SACK is due). Distinct + countable vs the per-batch gap-advance witness.
		if(l1_blockack_pipeline_active() && l1_blockack.aggregate_response_outstanding())
		{
			printf("[L1-BLOCKACK-PIPELINE] boundary window open cfg=%d timeout=%dms air=%lldms\n",
				current_configuration, receiving_timeout, block_forward_air_ms);
			fflush(stdout);
		}
		receiving_timer.start();
	}
}

// Timing redesign — retx never rides wire slot 0 (cross-layer data-flow audit,
// retx-queue cluster). At call time the v2 mixed-batch fill has left the retx
// block contiguous at the LEADING slots [0 .. R-1] (R = v2_retx_block_count) and
// the ND new-data frames at [R .. R+ND-1], with the EOB bit on the final
// new-data slot (message_batch_counter_tx-1). Rotate the sub-array [0 .. R]
// right by one so the wire order becomes:
//     [new0] [retx0 .. retxR-1] [new1 .. newND-1 (EOB)]
// Only wire slot 0's leading preamble is inside the peer's ~200-400 ms
// post-turnaround mute/flush window (a width-sweep observed slot-0 loss 9/9 with
// slots 1+ clean; frames are >= hundreds of ms), so parking one new-data frame
// there — and the retx block one slot behind it — keeps every retransmit out of
// the exposed slot while the EOB-marked frame stays wire-final (the RSP shortens
// its receive window on an EOB decode; a retx tail after it would collide with
// the RSP keying its SACK). The struct copy moves each frame's .data pointer with
// it, so retx payloads (in retx_scratch[]) and the new-data payloads follow their
// slots; the pre-assigned sequence bytes are position-independent, so no wire
// byte other than frame ORDER changes.
//
// No-op (legacy start=0 layout, byte-identical wire) unless this is a v2 mixed
// batch (v2_retx_block_count > 0) with at least two new-data frames (ND >= 2).
// ND==1 keeps the legacy prefix because its single new-data frame IS the EOB
// frame and must stay wire-final; ND==0 (pure retx) has no leading new-data
// frame to promote. Returns true iff the rotation was applied.
bool cl_arq_controller::v2_rotate_retx_behind_lead()
{
	int R = v2_retx_block_count;
	if(R <= 0)
		return false;                        // not a v2 mixed batch
	int ND = message_batch_counter_tx - R;
	if(ND < 2)
		return false;                        // ND in {0,1}: keep legacy layout
	// Fail-before / pass-after toggle (off by default; production never sets it).
	// With the rotation defeated the layout stays legacy (retx at wire slot 0),
	// which is what --test-retx-slot-order asserts against for its fail-before arm.
	static const bool rotate_defeat =
		(std::getenv("MERCURY_RETX_SLOT0_ROTATE_DEFEAT") != nullptr);
	if(rotate_defeat)
		return false;
	// Rotate [0..R] right by one: save new0 (slot R), shift the R retx frames up
	// by one into [1..R], drop new0 into slot 0.
	st_message tmp = messages_batch_tx[R];
	memmove(&messages_batch_tx[1], &messages_batch_tx[0],
		(size_t)R * sizeof(messages_batch_tx[0]));
	messages_batch_tx[0] = tmp;
	v2_retx_block_start = 1;
	return true;
}

// Control-ACK gate selection (cross-layer control-ACK audit, extension to every
// control code). The responder answers every control frame routed through the
// pattern wait below (SET_CONFIG, SWITCH_BANDWIDTH, SET_LINK_PARAMS, START_CONNECTION,
// CLOSE_CONNECTION, ...) with the same content-free MFSK base tone, so the accept has
// no content check behind it. Only CLOSE used the control floor + concentration gate;
// the other codes accepted on the data floor (metric >= 0.5), which white noise clears
// by matching many weak symbols (per-symbol concentration ~0.1 against ~1.0 for a
// real tone). A false SET_CONFIG accept moves the commander to the new configuration
// alone and desyncs the link until link_timeout. A rejected poll is not a failure:
// the wait keeps polling until receiving_timeout, and a timed-out wait takes the
// normal control resend path (process_messages_tx_control ACK_TIMED_OUT branch).
// MERCURY_CTRL_ACK_STRICT=0 restores the CLOSE-only gate (byte-identical accept).
bool cl_arq_controller::ctrl_ack_strict_enabled()
{
	if(ctrl_ack_strict_force >= 0)
		return ctrl_ack_strict_force != 0;
	static int cached = -1;
	if(cached < 0)
	{
		const char* e = std::getenv("MERCURY_CTRL_ACK_STRICT");
		cached = (e && *e && *e == '0') ? 0 : 1;
		// One-time arm attestation for run logs.
		printf("[CTRL-ACK-GATE] enabled=%d conc_floor=%.4f match_thr=%d (MERCURY_CTRL_ACK_STRICT=%s)\n",
			cached, telecom_system ? telecom_system->ack_mfsk.ack_conc_floor : -1.0,
			telecom_system ? telecom_system->ack_mfsk.ack_match_threshold : -1,
			e ? e : "unset");
		fflush(stdout);
	}
	return cached != 0;
}

bool cl_arq_controller::control_ack_strict_for(unsigned char code)
{
	if(code == CLOSE_CONNECTION)
		return true;
	return ctrl_ack_strict_enabled();
}

// BREAK-ACK poll. The responder confirms a BREAK with the same bare ACK tone every
// control wait listens for, and a false accept here is as harmful as a false
// SET_CONFIG accept: the commander settles at the recovery rung alone (or, under
// GS2, at the floor) while the responder never heard the BREAK. Previously this
// poll used the data floor. MERCURY_CTRL_ACK_STRICT=0 restores that
// (receive_ack_pattern() with default arguments).
bool cl_arq_controller::break_ack_poll()
{
	return receive_ack_pattern(false, false, -1,
		/*control_ack_strict=*/ctrl_ack_strict_enabled());
}

void cl_arq_controller::process_messages_rx_acks_control()
{
	if (receiving_timer.get_elapsed_time_ms()<receiving_timeout)
	{
		// Handshake echo (formerly v9 CAP_HANDSHAKE_ECHO, now unconditional):
		// when we sent TEST_CONNECTION, the responder always replies with an
		// LDPC TEST_CONNECTION_ACK on data_configuration instead of the MFSK
		// ACK pattern. CMD must use the LDPC RX path to receive this —
		// same exception pattern as KEY_EXCHANGE_1.
		bool expects_ldpc_handshake_ack =
			(messages_control.data[0] == TEST_CONNECTION);
		if(ack_pattern_time_ms > 0
		   && messages_control.data[0] != KEY_EXCHANGE_1
		   && !expects_ldpc_handshake_ack)
		{
			// Detect ACK tone pattern instead of decoding LDPC frame.
			// Keep checking until ACKED (not just PENDING_ACK): update_status()
			// may set ACK_TIMED_OUT before the receive window expires, but the
			// ACK pattern can still arrive within receiving_timeout.
			// KEY_EXCHANGE_1 must use LDPC path to receive responder's pubkey.
			if(messages_control.status != ACKED)
			{
				// CONNECT round-2 fix #2(a): for the START_CONNECTION control-ACK
				// wait on the slow 2-phase MFSK handshake, scan multiple
				// capture-window phases across the retained ring so a gap-displaced
				// ACK is caught regardless of which ftr==0 phase the snapshot fired
				// on (sub-mode B miss; CMD [CAP-PEAK] pk=0 with the ACK sitting at
				// an older phase). Scoped to START_CONNECTION only — the steady
				// DATA-ACK and other control waits are unaffected.
				bool mw_scan = (messages_control.data[0] == START_CONNECTION);
				int causal_ring_samples = -1;
				if(mw_scan && start_ack_causal_guard_armed)
				{
					cl_data_container& dc = telecom_system->data_container;
					if(dc.start_ack_causal_tracking_available)
					{
						const uint32_t generation =
							dc.start_ack_causal_generation.load();
						causal_ring_samples =
							dc.start_ack_causal_ring_generation.load()
								== generation
							? dc.start_ack_causal_ring_samples.load()
							: 0;
					}
					else if(arq_sim_inproc_active())
					{
						// An installed in-process pump is the authoritative
						// signal for both single- and two-instance harnesses.
						// They own no async capture producer/tag FIFO; the pump
						// drains each synthetic symbol synchronously, so retain
						// their sample-count model without enabling this
						// fallback in any production/audio-backed mode.
						const uint64_t now_ns = sim_clock_now_ns();
						const uint64_t deadline_ns =
							dc.start_ack_causal_deadline_ns.load();
						const uint64_t causal_ns =
							now_ns >= deadline_ns
								? now_ns - deadline_ns : 0;
						const uint64_t samples =
							(causal_ns / 1000000ULL) * 48ULL
							+ ((causal_ns % 1000000ULL) * 48ULL)
								/ 1000000ULL;
						causal_ring_samples =
							samples > 0x7FFFFFFFULL
								? 0x7FFFFFFF : (int)samples;
					}
					else
					{
						// If capture metadata could not be initialized, never
						// accept a CRC-less START ACK from unclassified audio.
						causal_ring_samples = 0;
					}

					if(!start_ack_causal_should_poll(
						causal_ring_samples,
						telecom_system->ack_pattern_passband_samples))
					{
						// Keep the capture-prep ring advancing while the
						// causal window matures; a stuck ftr==0 would freeze
						// the exact ACK audio this guard is waiting for.
						MUTEX_LOCK(&capture_prep_mutex);
						telecom_system->data_container.frames_to_read = 2;
						telecom_system->data_container.nUnder_processing_events = 0;
						MUTEX_UNLOCK(&capture_prep_mutex);
						return;
					}
#ifdef START_ACK_CAUSAL_GUARD_FAILBEFORE
					// Exact pre-correction behavior for deterministic RED coverage:
					// no timing gate and no causal history mask.
					causal_ring_samples = -1;
#endif
				}
				if(receive_ack_pattern(
					false, mw_scan, causal_ring_samples,
					/*control_ack_strict=*/control_ack_strict_for(
						(unsigned char)messages_control.data[0])))
				{
					printf("[CMD-ACK-PAT] Control ACK for code=%d detected! elapsed=%dms link=%d status=%d\n",
					(int)messages_control.data[0], (int)receiving_timer.get_elapsed_time_ms(), (int)link_status, (int)messages_control.status);
					fflush(stdout);
					// Flush old batch audio from playback buffer so responder
					// doesn't demodulate stale frames before the new batch.
					clear_buffer(playback_buffer);
					link_timer.start();
					watchdog_timer.start();
					gear_shift_timer.stop();
					gear_shift_timer.reset();
					messages_control.status=ACKED;
					stats.nAcked_control++;
					if(mw_scan)
						start_ack_causal_guard_armed = false;

					// Guard delay: wait for the radio TX→RX transition to settle.
					// v9.2: dropped the extra +200ms software margin. The control-ACK
					// branch detected the ACK pattern the moment its trailing sample
					// reached the demod; the only physical wait we still need is
					// ptt_off_delay_ms (default 200ms), which covers PA tail/RX-mute
					// release. The +200ms margin was an early-Mercury safety net
					// added when ACK-edge detection was less precise.
					int guard = ptt_off_delay_ms;
					receiving_timeout = (int)receiving_timer.get_elapsed_time_ms() + guard;
				}
			}
		}
		else
		{
			// Phase B Wave 2 v2 — PHY swap site D (fact-doc §13.2).
			// When we're waiting for the TEST_CONNECTION_ACK echo AND the
			// MFSK codec is available, poll for the MFSK TEST_ACK suffix
			// BEFORE the LDPC receive() runs. On detection, synthesize the
			// messages_control fields the legacy consumer at :3344-3392
			// expects so process_control_commander() runs unchanged.
			//
			// We don't fall through to receive() when MFSK fires — that
			// would consume audio meant for the next poll. We don't fall
			// through to receive() on a miss either — the next tick will
			// re-poll the suffix. NB sessions skip the MFSK detector
			// entirely (codec guard returns 0).
			bool mfsk_test_ack_path =
				expects_ldpc_handshake_ack
				&& narrowband_enabled != YES
				&& telecom_system->ack_mfsk.connect_pattern_nsymb > 0
				&& messages_control.status != ACKED;
			if(mfsk_test_ack_path)
			{
				uint8_t echoed_cap = 0, own_cap = 0, ssid = 0;
				if(receive_mfsk_test_ack_phy(&echoed_cap, &own_cap, &ssid))
				{
					printf("[CMD-TEST-ACK-V2] MFSK echoed=0x%02X own=0x%02X ssid=%u\n",
						echoed_cap, own_cap, ssid);
					fflush(stdout);
					// Synthesize messages_control.data[] to the LDPC
					// TEST_CONNECTION_ACK layout the legacy consumer at
					// arq_commander.cc:3344-3392 expects:
					//   data[0] = TEST_CONNECTION_ACK
					//   data[1] = echoed_cap   (CRC8 check at :3350-3352)
					//   data[2] = own_cap
					//   data[3] = CRC8(data[1..2])   — fresh-computed so
					//                                  the consumer's check
					//                                  passes; we just
					//                                  validated via CRC12.
					//   data[5] = own_cap            (read at :3410 as
					//                                  peer_capability)
					//   data[6] = ssid              (read at :3420 log)
					messages_control.data[0] = (char)TEST_CONNECTION_ACK;
					messages_control.data[1] = (char)echoed_cap;
					messages_control.data[2] = (char)own_cap;
					messages_control.data[3] = (char)CRC8_calc(
						(char*)&messages_control.data[1], 2);
					messages_control.data[4] = 0;
					messages_control.data[5] = (char)own_cap;
					messages_control.data[6] = (char)ssid;
					messages_control.length = 7;
					messages_control.type = ACK_CONTROL;

					// Mirror the post-LDPC-decode bookkeeping at :1657-1687.
					clear_buffer(playback_buffer);
					link_timer.start();
					watchdog_timer.start();
					gear_shift_timer.stop();
					gear_shift_timer.reset();
					messages_control.status = ACKED;
					stats.nAcked_control++;
					int guard = ptt_off_delay_ms;
					receiving_timeout = (int)receiving_timer.get_elapsed_time_ms() + guard;
					return;
				}
				// MFSK didn't fire this poll. DON'T fall through to LDPC
				// receive() — that would consume the audio. Re-enter on
				// next tick. (NB note: this branch is only entered when
				// codec is available, so this 'return' doesn't strand
				// NB sessions — they skip the MFSK block above and run
				// the legacy LDPC path below.)
				return;
			}

			// Decode LDPC ACK frame (also used for KEY_EXCHANGE_1 which
			// carries the responder's pubkey in the ACK data payload, and
			// for v9 TEST_CONNECTION_ACK which carries the capability echo).
			this->receive();
			// v9: TEST_CONNECTION_ACK is a valid reply to TEST_CONNECTION
			// (different data[0] but same control-frame ACK semantics).
			bool data0_match = (messages_rx_buffer.data[0]==messages_control.data[0])
				|| (messages_control.data[0]==TEST_CONNECTION
				    && messages_rx_buffer.data[0]==TEST_CONNECTION_ACK);
			if(messages_rx_buffer.status==RECEIVED && messages_rx_buffer.type==ACK_CONTROL)
			{
				if(data0_match && messages_control.status==PENDING_ACK)
				{
					// Flush old batch audio from playback buffer so responder
					// doesn't demodulate stale frames before the new batch.
					clear_buffer(playback_buffer);
					{
					int copy_len = max_data_length+max_header_length-CONTROL_ACK_CONTROL_HEADER_LENGTH;
					if(copy_len > N_MAX/8) copy_len = N_MAX/8;
					for(int j=0;j<copy_len;j++)
					{
						messages_control.data[j]=messages_rx_buffer.data[j];
					}
				}
					link_timer.start();
					watchdog_timer.start();
					gear_shift_timer.stop();
					gear_shift_timer.reset();
					messages_control.status=ACKED;
					stats.nAcked_control++;

					// Wait for responder to finish remaining ACK batch frames.
					// v9.2: dropped the +200ms software margin — the per-frame
					// transmission time and ptt_off_delay_ms together already
					// cover the audio drain, and the +200 was uniformly a
					// no-op cushion in IONOS / VB-Cable measurement.
					{
						int drain = (int)receiving_timer.get_elapsed_time_ms()
							+ (ack_batch_size - 1) * ctrl_transmission_time_ms
							+ ptt_off_delay_ms;
						if (drain < receiving_timeout)
							receiving_timeout = drain;
					}
				}
			}
			messages_rx_buffer.status=FREE;
		}
	}
	else
	{
		// Restore receiving_timeout if batch drain logic adjusted it
		calculate_receiving_timeout();
		// Restore data config if we switched to ack config (LDPC ACK path)
		if(ack_pattern_time_ms <= 0)
			load_configuration(data_configuration, PHYSICAL_LAYER_ONLY,YES);
		// KEY_EXCHANGE_1 stays on data_configuration — no restore needed
		if(messages_control.status==ACKED)
		{
			emergency_nack_count = 0;  // Channel working — reset BREAK counter
			consec_pure_silent_rounds = 0;   // R5: a credited control-ACK is a live peer -- reset the silent streak
			watchdog_probe_clear_on_liveness();   // +LOW: clear any stale WD_PROBE latch on a genuine liveness credit
			process_control_commander();
		}
		else
		{
			// Break recovery phase 2: probe at target config failed
			if(break_recovery_phase == 2)
			{
				break_recovery_retries--;
				if(break_recovery_retries > 0)
				{
					printf("[BREAK-RECOVERY] Probe retry (%d left) at config %d\n",
						break_recovery_retries, current_configuration);
					fflush(stdout);
					receiving_timer.stop();
					receiving_timer.reset();
					// Force-clear: cleanup() skips PENDING_ACK status
					messages_control.status = FREE;
					add_message_control(SET_CONFIG);
					connection_status = TRANSMITTING_CONTROL;
					return;
				}
				else
				{
					// Probe failed — this config doesn't work.
					// BREAK back to ROBUST_0 and try lower target.
					printf("[BREAK-RECOVERY] Config %d failed probe, sending BREAK\n",
						current_configuration);
					fflush(stdout);

					// Lower the proven ceiling — this config failed too
					int new_ceil = config_ladder_down(current_configuration, robust_enabled);
					if(supershift_proven_ceiling < 0 || new_ceil < supershift_proven_ceiling)
						supershift_proven_ceiling = new_ceil;

					receiving_timer.stop();
					receiving_timer.reset();
					// Force-clear: cleanup() skips PENDING_ACK status
					messages_control.status = FREE;
					emergency_previous_config = current_configuration;
					emergency_break_active = 1;
					emergency_break_retries = 1;
					break_recovery_phase = 0;  // BREAK ACK handler will set to 1
					// SACK Design A Step 12 — BREAK supremacy (§4.3.4 invariant #6).
					if(sack_v2_enabled)
						policy_axis1_supremacy_on_move(current_configuration,
							current_configuration, "break_recovery_phase2_probe_fail");
					send_break_pattern();
					telecom_system->data_container.frames_to_read = 4;
					calculate_receiving_timeout();
					receiving_timer.start();
					return;
				}
			}

			// Break recovery phase 1: coordination SET_CONFIG at ROBUST_0 failed
			if(break_recovery_phase == 1)
			{
				break_recovery_retries--;
				if(break_recovery_retries > 0)
				{
					printf("[BREAK-RECOVERY] Phase 1 retry (%d left) at config %d\n",
						break_recovery_retries, current_configuration);
					fflush(stdout);
					receiving_timer.stop();
					receiving_timer.reset();
					messages_control.status = FREE;
					add_message_control(SET_CONFIG);
					connection_status = TRANSMITTING_CONTROL;
					return;
				}
				else
				{
					// Phase 1 exhausted — BREAK again to resync
					printf("[BREAK-RECOVERY] Phase 1 failed, re-sending BREAK\n");
					fflush(stdout);
					receiving_timer.stop();
					receiving_timer.reset();
					messages_control.status = FREE;
					// Keep emergency_previous_config unchanged. Phase-1 failure
					// means RSP did not HEAR the SET_CONFIG on the coordination
					// layer — the target was never disproved, only un-delivered.
					// Retry the SAME plan. The descent mechanism lives in the
					// Phase-2 exhaust path at the line ~1278 refresh (target
					// disproved end-to-end → advance descent anchor) and the
					// break_drop_step doubling at line 64. See
					// EMERGENCY_PREVIOUS_CONFIG_INVESTIGATION.md §1, §3, §6.
					emergency_break_active = 1;
					emergency_break_retries = 1;
					break_recovery_phase = 0;
					// SACK Design A Step 12 — BREAK supremacy (§4.3.4 invariant #6).
					if(sack_v2_enabled)
						policy_axis1_supremacy_on_move(current_configuration,
							current_configuration, "break_recovery_phase1_exhausted");
					send_break_pattern();
					telecom_system->data_container.frames_to_read = 4;
					calculate_receiving_timeout();
					receiving_timer.start();
					return;
				}
			}

			// SWITCH_BANDWIDTH failure: responder is nb_only, didn't ACK.
			// Stay NB and start turboshift normally.
			if(wb_upgrade_pending && link_status == CONNECTED)
			{
				printf("[BW-NEG] SWITCH_BANDWIDTH not ACKed (peer is nb_only), staying NB\n");
				fflush(stdout);
				wb_upgrade_pending = false;

				// Force-clear control message
				messages_control.ack_timeout = 0;
				messages_control.id = 0;
				messages_control.length = 0;
				messages_control.nResends = 0;
				messages_control.status = FREE;
				messages_control.type = NONE;

				receiving_timer.stop();
				receiving_timer.reset();

				// Option B (data-anchored promotion, 2026-05-29): the control-only
				// SNR-SUPERSHIFT probe is removed. The peer rejected WB (nb_only),
				// so we stay NB; start data at the current config (ROBUST_0 for -R
				// gearshift) and let FRAME-UP climb rung-by-rung ONLY on confirmed
				// data delivery. wb_upgrade_pending=false and the messages_control
				// force-clear above already ran. See
				// fact-documents/gearshift-start-and-recovery.md §6.
				turboshift_active = false;
				turbo_supershift_announce_pending = false;
				turboshift_phase = TURBO_DONE;
				connection_status = TRANSMITTING_DATA;
				return;
			}

			// PSK mismatch: KEY_ACTIVATE was sent to notify the responder.
			// Whether ACKed or timed out, disconnect now.
			if(psk_mismatch_pending && link_status == CONNECTED)
			{
				printf("[CRYPTO] KEY_ACTIVATE sent (PSK mismatch), disconnecting\n");
				fflush(stdout);
				psk_mismatch_pending = false;
				this->link_status = DROPPED;
				reset_session_state();
				return;
			}

			// Turboshift probe failure: handle directly (1 retry, then ceiling+BREAK).
			// Bypasses nResends sub-retries and gearshift_timeout for fast OTA response.
			// Guard: only during CONNECTED — during CONNECTING, the failed message is
			// START_CONNECTION, not a turboshift SET_CONFIG. Without this guard,
			// turboshift_active (true from init) hijacks the first NAck and replaces
			// START_CONNECTION with SET_CONFIG, preventing connection. (Bug #35)
			if(turboshift_active && this->role == COMMANDER && link_status == CONNECTED)
			{
				// Force-clear control message (prevent nResends sub-retries)
				messages_control.ack_timeout=0;
				messages_control.id=0;
				messages_control.length=0;
				messages_control.nResends=0;
				messages_control.status=FREE;
				messages_control.type=NONE;

				receiving_timer.stop();
				receiving_timer.reset();
				gear_shift_timer.stop();
				gear_shift_timer.reset();

				if(turboshift_retries > 0)
				{
					turboshift_retries--;
					printf("[TURBO] RETRY config %d (retries left: %d)\n",
						current_configuration, turboshift_retries);
					fflush(stdout);
					add_message_control(SET_CONFIG);
					connection_status = TRANSMITTING_CONTROL;
					return;
				}

				// Ceiling — config failed, handle based on turboshift phase
				int failed_config = current_configuration;
				int settle_config = (turboshift_last_good >= 0) ?
					turboshift_last_good : init_configuration;

				printf("[TURBO] CEILING at config %d, settle=%d (proven_ceiling=%d, phase=%d)\n",
					failed_config, settle_config, supershift_proven_ceiling,
					turboshift_phase);
				printf("[TURBO] CEILING state: turboshift_last_good=%d init_config=%d "
					"negotiated=%d data_cfg=%d current=%d\n",
					turboshift_last_good, init_configuration,
					negotiated_configuration, data_configuration, current_configuration);
				fflush(stdout);

				// During TURBO_REVERSE: if the reverse path can't even do CONFIG_0,
				// skip BREAK (which confuses the role-swapped state) and finish
				// immediately. The reverse path is MFSK-only for ACKs.
				if(turboshift_phase == TURBO_REVERSE && is_robust_config(settle_config))
				{
					printf("[TURBO] REVERSE path MFSK-only (ceiling=%d), finishing without BREAK\n",
						settle_config);
					fflush(stdout);

					// Restore to ROBUST_0 and finish
					data_configuration = settle_config;
					negotiated_configuration = settle_config;
					load_configuration(settle_config, PHYSICAL_LAYER_ONLY, YES);
					turboshift_last_good = settle_config;
					finish_turbo_direction();
					return;
				}

				// WALL-B FIX-7A (KEYSTONE, fix7/FIX7_DESIGN.md §1) — discriminate a
				// SPECULATIVE RE-TRIGGER over-reach from a genuine top-config ceiling.
				// The SNR RE-TRIGGER (:5258-5294) can leap MANY rungs up from a freshly
				// proven OFDM anchor on the saturating post-EQ EVM-SNR (CONFIG_0 anchor →
				// CFG13 in the repro). CFG13's single OFDM control frame does NOT decode
				// here ([FTR-FAIL] CONFIG_13 metric=0.000), so the probe NAcks twice and
				// lands in this CEILING. UNCONDITIONALLY BREAKing to ROBUST_0 here throws
				// away the proven OFDM rung (last_data_viable_config) the link was just
				// delivering on — and in SIM_INPROC / a slow-wire peer the BREAK-ACK tail
				// deadlocks and PARKS at ROBUST_0 on a CLEAN channel. When the failed probe
				// was a speculative MULTI-rung leap from a PROVEN OFDM anchor, SETTLE at
				// that anchor and resume DATA there instead. The deep-SNR escape is
				// preserved: a genuine top-config ceiling (anchor still ROBUST) and a
				// +1-edge probe fail the discriminator and BREAK exactly as today. See
				// the PURE helper turbo_ceiling_should_settle (arq.h) + Part U.
				if(turbo_ceiling_should_settle((int)turboshift_phase, settle_config,
						last_data_viable_config, failed_config))
				{
					int settle_rung = last_data_viable_config;  // the PROVEN OFDM anchor
					printf("[TURBO] SETTLE (FIX-7A): probe over-reach %d -> settling at "
						"proven anchor %d, resuming DATA (NO BREAK)\n",
						failed_config, settle_rung);
					fflush(stdout);

					// Settle INLINE (NOT via finish_turbo_direction — R1: on FORWARD it
					// kicks a REVERSE SWITCH_ROLE probe, or with --skip-turbo-reverse
					// re-installs proven_ceiling=turboshift_last_good, fighting the
					// ceiling-lowering). This is the same inline settle the BW-NEG fallback
					// (:2260-2263) uses: phase=TURBO_DONE + connection_status=TRANSMITTING_DATA.
					turboshift_active = false;
					turbo_supershift_announce_pending = false;
					turboshift_phase = TURBO_DONE;
					data_configuration       = settle_rung;
					negotiated_configuration = settle_rung;
					turboshift_last_good     = settle_rung;
					load_configuration(settle_rung, PHYSICAL_LAYER_ONLY, YES);

					// Lower the proven ceiling below the just-failed speculative rung so the
					// very next RE-TRIGGER cannot immediately re-leap straight back to it
					// (the limit-cycle guard, arq.h re-trigger ceiling cap). Matches the
					// existing CEILING ceiling-lowering intent (:2369-2370).
					supershift_proven_ceiling = config_ladder_down(failed_config, robust_enabled);

					// Resume DATA at the settled, proven rung — NO send_break_pattern(),
					// NO ROBUST_0 drop. The BREAK-ladder fields (emergency_break_active,
					// break_drop_step, emergency_previous_config, breaks_since_last_data_
					// success) are DELIBERATELY untouched (§4.2 audit / Part U4): the settle
					// does not enter the BREAK ladder, so it cannot perturb the FIX-2 AARF.
					connection_status = TRANSMITTING_DATA;
					watchdog_timer.start();
					link_timer.start();
					return;
				}

				// STAGE 4d (RESIDUAL — turbo speculative-climb fail gate, design §4.3 /
				// Stage-4c residual). A TURBO_FORWARD speculative climb that the RX could
				// not follow must NOT cascade to ROBUST_0 under inband — it AUTO-DEMOTES to
				// the last-confirmed config via the CONFIG_TAG, exactly like a D1 climb that
				// exhausted its R floor (this IS a speculative climb the RX did not follow).
				// Route to inband_route_failure_demote (BREAK-count stays 0); only if the
				// demote is a no-op (already at the floor / invalid) fall through to the
				// legacy BREAK below. Cancel the failed turbo state first so the demote starts
				// clean. Gated OFF -> byte-identical legacy BREAK.
				if(inband_rate_feature_enabled())
				{
					int demote_target = (inband_last_confirmed_config != CONFIG_NONE)
						? inband_last_confirmed_config
						: config_ladder_down(failed_config, robust_enabled);
					turboshift_active = false;
					turbo_supershift_announce_pending = false;
					// Free the failed turbo control slot so the demote's add_message_control(
					// SET_CONFIG) can claim it (add_message_control requires status==FREE).
					messages_control.ack_timeout=0;
					messages_control.id=0;
					messages_control.length=0;
					messages_control.nResends=0;
					messages_control.status=FREE;
					messages_control.type=NONE;
					if(inband_route_failure_demote(demote_target, "turbo_forward_climb_unfollowable"))
						return;   // demoted via the tag; the demote owns the next transition
					// no-op demote (no lower rung): fall through to the legacy BREAK -> the
					// SESSION_DEAD_BATCHES floor is the genuine true-loss path.
				}

				// TURBO_FORWARD or higher ceiling: BREAK to ROBUST_0, then drop to
				// the SNR-predicted start config (not just 1 step below ceiling).
				// Dropping 1 step at a time wastes time probing configs that can't work.
				turboshift_active = false;
				turbo_supershift_announce_pending = false;
				data_configuration = settle_config;
				// Compute SNR-based target so BREAK drops far enough
				int snr_target_config = -1;
				if(turbo_best_snr > -90)
					snr_target_config = config_ladder_down_n(
						get_configuration(turbo_best_snr - SUPERSHIFT_MARGIN_DB), 2, robust_enabled);
				if(snr_target_config >= 0)
				{
					// Calculate how many steps to drop from failed_config to snr_target
					int steps = config_ladder_index(failed_config) - config_ladder_index(snr_target_config);
					if(steps < 1) steps = 1;
					emergency_previous_config = failed_config;
					break_drop_step = steps;
				}
				else
				{
					emergency_previous_config = failed_config;
					break_drop_step = 1;
				}

				// Remember this ceiling so re-trigger never jumps back above it
				supershift_proven_ceiling = (snr_target_config >= 0) ?
					snr_target_config : config_ladder_down(failed_config, robust_enabled);
				emergency_break_active = 1;
				emergency_break_retries = 1;
				emergency_nack_count = 0;

				for(int i=0; i<nMessages; i++)
					messages_tx[i].status = FREE;
				fifo_buffer_backup.flush();

				// SACK Design A Step 12 — BREAK supremacy (§4.3.4 invariant #6).
				if(sack_v2_enabled)
					policy_axis1_supremacy_on_move(failed_config,
						supershift_proven_ceiling, "turbo_forward_break");
				send_break_pattern();
				telecom_system->data_container.frames_to_read = 4;
				calculate_receiving_timeout();
				receiving_timer.start();
				return;
			}

			// Turboshift SWITCH_ROLE failure: if the role-swap frame isn't ACKed
			// during turboshift, retry twice then BREAK. Without this, the commander
			// retransmits SWITCH_ROLE forever because the normal BREAK counter
			// (line ~1194) requires turboshift_phase == TURBO_DONE. (Bug #60)
			if(messages_control.data[0] == SWITCH_ROLE
				&& turboshift_phase != TURBO_DONE
				&& link_status == CONNECTED
				&& !emergency_break_active)
			{
				turbo_switch_role_retries++;
				printf("[TURBO] SWITCH_ROLE NAck #%d at config %d (phase=%d)\n",
					turbo_switch_role_retries, current_configuration,
					(int)turboshift_phase);
				fflush(stdout);

				if(turbo_switch_role_retries <= 2)
				{
					// Retry — the frame may have been corrupted by noise
					connection_status = TRANSMITTING_CONTROL;
				}
				else
				{
					// Retries exhausted — BREAK and end turboshift at last good config
					int settle_config = (turboshift_last_good >= 0) ?
						turboshift_last_good : init_configuration;

					printf("[TURBO] SWITCH_ROLE failed %d times, BREAK to settle at config %d\n",
						turbo_switch_role_retries, settle_config);
					fflush(stdout);

					// STAGE 4d (RESIDUAL — turbo speculative-climb fail gate, design §4.3 /
					// Stage-4c residual). A failed turbo SWITCH_ROLE is the role-swap arm of a
					// speculative climb the RX could not complete; under inband it AUTO-DEMOTES
					// to the last-confirmed config via the CONFIG_TAG, NOT the ROBUST_0 cascade.
					// End the turbo state, free the pending control slot (so the demote's
					// SET_CONFIG wins it), then route the tag-demote (BREAK-count stays 0). A
					// no-op demote falls through to the legacy BREAK -> SESSION_DEAD_BATCHES floor.
					if(inband_rate_feature_enabled())
					{
						int demote_target = (inband_last_confirmed_config != CONFIG_NONE)
							? inband_last_confirmed_config : settle_config;
						turbo_switch_role_retries = 0;
						turboshift_phase = TURBO_DONE;
						turboshift_active = false;
						turbo_supershift_announce_pending = false;
						turbo_snr_ack_enabled = false;
						turbo_received_snr = -99.0f;
						messages_control.ack_timeout=0;
						messages_control.id=0;
						messages_control.length=0;
						messages_control.nResends=0;
						messages_control.status=FREE;
						messages_control.type=NONE;
						if(inband_route_failure_demote(demote_target, "turbo_switch_role_unfollowable"))
							return;   // demoted via the tag; the demote owns the next transition
						// no-op demote (no lower rung): fall through to the legacy BREAK.
					}

					turbo_switch_role_retries = 0;
					turboshift_phase = TURBO_DONE;
					turboshift_active = false;
					turbo_supershift_announce_pending = false;
					turbo_snr_ack_enabled = false;
					turbo_received_snr = -99.0f;

					// Cancel pending control message
					messages_control.ack_timeout=0;
					messages_control.id=0;
					messages_control.length=0;
					messages_control.nResends=0;
					messages_control.status=FREE;
					messages_control.type=NONE;

					data_configuration = settle_config;
					emergency_previous_config = current_configuration;
					break_drop_step = 1;
					supershift_proven_ceiling = config_ladder_down(current_configuration, robust_enabled);
					emergency_break_active = 1;
					emergency_break_retries = 1;
					emergency_nack_count = 0;

					for(int i=0; i<nMessages; i++)
						messages_tx[i].status = FREE;
					fifo_buffer_backup.flush();

					// SACK Design A Step 12 — BREAK supremacy (§4.3.4 invariant #6).
					if(sack_v2_enabled)
						policy_axis1_supremacy_on_move(current_configuration,
							settle_config, "turbo_switch_role_break");
					send_break_pattern();
					telecom_system->data_container.frames_to_read = 4;
					calculate_receiving_timeout();
					receiving_timer.start();
					return;
				}
			}

			// Frame gearshift up failure: BREAK immediately and double threshold on
			// the legacy path. MC-3 waits for repeated failed control retransmissions.
			if(messages_control.data[0] == SET_CONFIG
				&& turboshift_phase == TURBO_DONE
				&& !turboshift_active
				&& break_recovery_phase == 0
				&& !emergency_break_active
				// An unheard reverse control ACK is not forward-modulation evidence.
				// Under slot-liveness, demote only after the real retry budget failed.
				&& (!linkphase_slotliveness_on() || messages_control.nResends <= 1))
			{
				rate_opt.notify_switch_failed();
				gear_shift_timer.stop();
				gear_shift_timer.reset();

				// FIX-B — ARM the floor-probe back-off on the FAILED up-probe rung
				// (gearshift-floor-probe-backoff.md §5.1, arm-site #1). This is the
				// SET_CONFIG-ACK-timeout fail: the SET_CONFIG to negotiated_configuration
				// (the proposed UP rung) was NAcked. Arm BEFORE the working_config
				// overwrite below clobbers negotiated_configuration, so the back-off is
				// keyed to the rung the climb actually tried (not the recovered rung).
				probe_backoff_arm(negotiated_configuration);

				int working_config = config_ladder_down(negotiated_configuration, robust_enabled);
				frame_shift_threshold *= 2;
				consecutive_data_acks = 0;

				{
					int fifo_load = fifo_buffer_tx.get_size() - fifo_buffer_tx.get_free_size();
					printf("[GEARSHIFT] FRAME UP FAILED: %d->%d NAck, BREAK to %d (threshold now %d, fifo=%d bytes)\n",
						working_config, negotiated_configuration, working_config, frame_shift_threshold, fifo_load);
				}
				fflush(stdout);

				// Cancel the failed control message
				messages_control.ack_timeout=0;
				messages_control.id=0;
				messages_control.length=0;
				messages_control.nResends=0;
				messages_control.status=FREE;
				messages_control.type=NONE;

				// STAGE 4d (RESIDUAL — turbo/frame speculative-climb fail gate, design §4.3
				// / Stage-4c residual). A FRAME-UP climb the RX could not follow AUTO-DEMOTES
				// to the last-confirmed config via the CONFIG_TAG under inband, NOT the
				// ROBUST_0 BREAK cascade. (Under inband a config change rides the tag, not a
				// SET_CONFIG handshake, so this NAck path is normally unreachable for a config
				// change — but gate it anyway so a residual SET_CONFIG fail demotes, never
				// cascades.) The failed control slot was freed just above. Route the tag-demote;
				// a no-op falls through to the legacy BREAK -> the SESSION_DEAD_BATCHES floor.
				// Gated OFF -> byte-identical legacy BREAK.
				if(inband_rate_feature_enabled())
				{
					int demote_target = (inband_last_confirmed_config != CONFIG_NONE)
						? inband_last_confirmed_config : working_config;
					if(inband_route_failure_demote(demote_target, "frame_up_climb_unfollowable"))
						return;   // demoted via the tag; the demote owns the next transition
					// no-op demote (no lower rung): fall through to the legacy BREAK.
				}

				// Reset config state to the working config
				data_configuration = working_config;
				negotiated_configuration = working_config;

				// BREAK to resync — recover to the working config
				emergency_previous_config = working_config;
				break_drop_step = 0;
				emergency_break_active = 1;
				emergency_break_retries = 1;
				emergency_nack_count = 0;

				// SACK Design A Step 12 — BREAK supremacy (§4.3.4 invariant #6).
				if(sack_v2_enabled)
					policy_axis1_supremacy_on_move(current_configuration,
						working_config, "frame_gearshift_up_failed");
				send_break_pattern();
				telecom_system->data_container.frames_to_read = 4;
				calculate_receiving_timeout();
				receiving_timer.start();
				return;
			}

			// RANK-3 FIX (data-flow-inband-ondemote-zerobyte.md §3.1/§7): under inband, a
			// generic control-ACK failure (a missed reverse-link / recovery SET_CONFIG ACK)
			// must NOT escalate to the legacy BREAK->ROBUST_0 cascade. Config changes ride the
			// passband CONFIG_TAG, not the SET_CONFIG handshake; rate/loss handling belongs to
			// the forward-data tag-demote (arq_commander.cc:4513), the RX down-ladder, and the
			// SESSION_DEAD_BATCHES floor — NOT this control-ACK miss. The un-gated escalation
			// here is what fired send_break_pattern() on the ON arm (parking the TX at cfg=100)
			// while the inband path was still recovering, RACING it. Gate it OFF: cancel the
			// stale control slot benignly (it was already managed by the caller) and do NOT
			// count toward / fire the BREAK. The control twin to the gated forward-data path
			// (:4513). Byte-identical when the feature is OFF (the legacy block below runs).
			if(inband_rate_feature_enabled())
			{
				printf("[INBAND-TX] control-ACK miss at config %d: BREAK escalation SUPPRESSED "
					"(inband owns rate/loss via tag-demote + down-ladder + dead-batch floor)\n",
					current_configuration);
				fflush(stdout);
				// A batch-geometry control (Axis-2 SET_LINK_PARAMS / robust dwell batch) whose
				// reverse ACK was never heard has no successor message: freeing its slot below
				// and then parking in TRANSMITTING_CONTROL leaves nothing to send, so the data
				// plane never resumes and every later rate decision (which is actuated at a
				// data-batch boundary) is stranded until the link timer expires.  The commander
				// already applied the new batch geometry locally at decision time, exactly as on
				// the ACKed path, so resume DATA with it; the data batches carry their own window
				// and the next ACK re-establishes agreement.  MERCURY_INBAND_CTRL_MISS_RESUME=0
				// restores the previous behaviour.
				bool ctrl_miss_resume = false;
				{
					int miss_code = (messages_control.length > 0) ? (int)(unsigned char)messages_control.data[0] : -1;
					ctrl_miss_resume = link_status == CONNECTED && inband_ctrl_miss_resumes_data(miss_code);
					if(ctrl_miss_resume)
					{
						printf("[INBAND-CTRL-MISS-RESUME] control code=%d unacknowledged after its retry "
							"budget at config %d: resuming DATA with local batch=%d\n",
							miss_code, current_configuration, data_batch_size);
						fflush(stdout);
					}
				}
				messages_control.ack_timeout = 0;
				messages_control.id          = 0;
				messages_control.length      = 0;
				messages_control.nResends    = 0;
				messages_control.status      = FREE;
				messages_control.type        = NONE;
				// Do NOT bump emergency_nack_count, do NOT send_break_pattern. Fall through to
				// the normal cleanup at the end of this handler (receiving_timer stop/reset).
				if(ctrl_miss_resume)
				{
					arm_control_turnaround_guard();
					this->connection_status = TRANSMITTING_DATA;
					receiving_timer.stop();
					receiving_timer.reset();
					this->cleanup();
					return;
				}
			}
			// Track control failures toward BREAK threshold.
			// Only during connected data exchange (not connection setup or turboshift).
			else if(!linkphase_slotliveness_on()
				&& link_status == CONNECTED && turboshift_phase == TURBO_DONE
				&& gear_shift_on == YES && !emergency_break_active)
			{
				emergency_nack_count++;
				printf("[BREAK] Control failure #%d at config %d (threshold=%d)\n",
					emergency_nack_count, current_configuration, emergency_nack_threshold);
				fflush(stdout);

				if(emergency_nack_count >= emergency_nack_threshold
					&& !config_is_at_bottom(current_configuration, robust_enabled))
				{
					// Lower ceiling to prevent climbing back to failing config
					int new_ceiling = config_ladder_down(current_configuration, robust_enabled);
					if(supershift_proven_ceiling < 0 || new_ceiling < supershift_proven_ceiling)
					{
						supershift_proven_ceiling = new_ceiling;
						ceiling_success_count = 0;
						printf("[BREAK] Lowered ceiling to %d\n", new_ceiling);
						fflush(stdout);
					}
					printf("[BREAK] Sending emergency BREAK pattern (control failure)\n");
					fflush(stdout);

					// Cancel pending control message
					messages_control.ack_timeout=0;
					messages_control.id=0;
					messages_control.length=0;
					messages_control.nResends=0;
					messages_control.status=FREE;
					messages_control.type=NONE;

					emergency_previous_config = current_configuration;
					emergency_break_active = 1;
					emergency_break_retries = 1;
					// SACK Design A Step 12 — BREAK supremacy (§4.3.4 invariant #6).
					if(sack_v2_enabled)
						policy_axis1_supremacy_on_move(current_configuration,
							current_configuration, "break_control_failure_threshold");
					send_break_pattern();
					telecom_system->data_container.frames_to_read = 4;
					calculate_receiving_timeout();
					receiving_timer.start();
					return;
				}
			}
			connection_status=TRANSMITTING_CONTROL;
		}

		receiving_timer.stop();
		receiving_timer.reset();
		this->cleanup();
	}
}

// ============================================================================
// STAGE 4c — D5: BREAK truly obsolete. The reusable tag-demote that replaces a
// COMMANDER Class-A send_break_pattern() under inband.
// (inband-reliability-design.md §5.3 / data-flow-perbatch-config.md §S4C.)
// ============================================================================
// Lifts the CFG16 D3 reverse-ACK demote body (arq_commander.cc:3978-4117) into ONE
// helper so each Class-A site routes a degradation to a TAGGED demote instead of the
// BREAK->ROBUST cascade. The structure is VERBATIM from D3:
//   1. lossless bsi rollback (cmd_batch_seq_id <- earliest in-flight bsi) so the
//      re-sent batch is contiguous with the RSP delivery high-water (no D3.1 GAP-ABORT),
//   2. FIFO push-back / restore_tx_from_compressed (preserve in-flight bytes),
//   3. set the config owners to demote_target (so the chokepoint at :736-739 reads
//      negotiated_configuration -> inband_unilateral_config_change(demote_target) -> tag),
//   4. pin supershift_proven_ceiling, reset the nack + revack-starve streaks,
//   5. policy_axis1_supremacy_on_move (keep the optimizer cooldown coherent),
//   6. add_message_control(SET_CONFIG) (under inband: the unilateral drop + W1 tag)
//      + connection_status=TRANSMITTING_CONTROL.
// NO send_break_pattern, NO emergency_break_active. The caller `return`s after this.
bool cl_arq_controller::scream_rollback_slot_eligible(int status, int length,
	int batch_seq_id) const
{
	return status != FREE && length > 0 && batch_seq_id >= 0;
}

// Default-on defeat gate for the deterministic fail-before arm.  Read fresh so
// a single test binary can exercise either side in separate invocations.
static bool probe_full_failure_coastdown_on()
{
	const char* e = std::getenv("MERCURY_PROBE_FULL_FAILURE_COASTDOWN_DEFEAT");
	return !(e && *e && atoi(e) != 0);
}

bool cl_arq_controller::inband_route_failure_demote(int demote_target, const char* reason,
	bool pin_ceiling, bool gearshift_owned)
{
	if(demote_target < 0 || demote_target == current_configuration)
		return false;   // no lower rung / no-op — caller must route to the dead-batch floor

	// ACTIVE-v2 single-owner contract. Legacy recovery code supplies EVIDENCE,
	// never a destination. There are only three ways an Axis-1 mutation may pass:
	//   1) the normal v2 selector already owns a matching switch transaction;
	//   2) a concrete failure during probe probation is converted by the owner to
	//      that probe's own fallback;
	//   3) a Gearshift transport-failure path explicitly marks this helper call as
	//      gearshift_owned, in which case we create the rollback transaction here.
	// All other legacy demote requests are consumed and the full v2 evaluator
	// chooses (or HOLDs) from current evidence BEFORE any FIFO/config mutation.
	if(rate_opt.controls_link() && link_status == CONNECTED)
	{
		const unsigned long long now_ms = opt_now_ms();
		const bool owned_probe_failure_coastdown =
			probe_full_failure_coastdown_on() && gearshift_owned
			&& rate_opt.owns_coastdown_transition(current_configuration, demote_target);
		// A canonical transition is not committed until peer evidence closes its armed
		// re-tag state.  Do not replace a lost-tag transaction with another demotion:
		// keep the current target, let the ordinary retransmission re-emit its tag, and
		// treat this degradation only as evidence.  Chaining 13->12->11 after the first
		// tag was lost guarantees PHY desynchronization because the peer is still at 13.
		if(inband_retag_armed && rate_opt.owns_link_experiment(now_ms)
		   && !owned_probe_failure_coastdown)
		{
			printf("[GEARSHIFT-V2-AUTHORITY] degradation deferred while CONFIG_TAG "
				"transition %d->%d awaits peer evidence; retransmission will re-tag target\n",
				inband_pre_announce_config, inband_retag_config);
			fflush(stdout);
			// This threshold episode has been consumed.  Leaving either the counter or
			// expired receive timer latched re-enters this branch every poll and starves
			// the very DATA retransmission that carries the repeat tag.
			emergency_nack_count = 0;
			receiving_timer.stop();
			receiving_timer.reset();
			connection_status = TRANSMITTING_DATA;
			return true;
		}
		// A concrete full-batch failure at an unconfirmed probe target is the
		// terminal result of that experiment, not a second competing detector.
		// consume_failure_signal() has already closed the probe and opened this
		// exact owner-selected rollback.  Retire the old climb tag before the
		// chokepoint below arms the fallback tag; otherwise the generic deferral
		// above preserves the failed target until the link timer tears down.
		if(inband_retag_armed && owned_probe_failure_coastdown)
		{
			printf("[GEARSHIFT-V2-PROBE-FAILFAST] retire failed probe tag %d->%d; "
			       "owner coast-down %d->%d proceeds\n",
				inband_pre_announce_config, inband_retag_config,
				current_configuration, demote_target);
			fflush(stdout);
			inband_retag_armed = false;
			inband_retag_config = CONFIG_NONE;
			inband_announce_bsi = -1;
			inband_retag_count = 0;
		}
		if(gearshift_owned)
		{
			if(!rate_opt.transition_matches(current_configuration, demote_target))
			{
				rate_opt.notify_switch_dispatched(
					current_configuration, demote_target, GEARSHIFT_ACTION_ROLLBACK,
					demote_target, now_ms, narrowband_enabled == YES);
			}
		}
		else if(rate_opt.probe_is_active())
		{
			int owned_target = rate_opt.authorize_external_transition(
				current_configuration, demote_target, reason, now_ms,
				narrowband_enabled == YES);
			if(owned_target < 0)
			{
				printf("[GEARSHIFT-V2-AUTHORITY] failure signal consumed during owned "
					"experiment without Axis-1 mutation: hint=%d->%d reason=%s\n",
					current_configuration, demote_target, reason ? reason : "?");
				fflush(stdout);
				return true;
			}
			demote_target = owned_target;
		}
		else if(rate_opt.owns_link_experiment(now_ms))
		{
			rate_opt.authorize_external_transition(
				current_configuration, demote_target, reason, now_ms,
				narrowband_enabled == YES);
			return true;
		}
		else
		{
			int owner_target = current_configuration;
			if(opt_evaluate_batch_end(&owner_target)
			   && owner_target != current_configuration)
			{
				opt_pending_switch_cfg = owner_target;
				printf("[GEARSHIFT-V2-AUTHORITY] telemetry failure reason=%s requested "
					"hint=%d->%d; owner selected %d->%d action=%d\n",
					reason ? reason : "?", current_configuration, demote_target,
					current_configuration, owner_target,
					(int)opt_pending_switch_action);
				fflush(stdout);
			}
			else
			{
				printf("[GEARSHIFT-V2-AUTHORITY] telemetry failure reason=%s requested "
					"hint=%d->%d; owner verdict=HOLD/ABSTAIN (no foreign mutation)\n",
					reason ? reason : "?", current_configuration, demote_target);
				fflush(stdout);
			}
			return true;
		}
	}

	printf("[INBAND-NOBREAK] Class-A degradation (%s): demoting %d -> %d via the CONFIG_TAG "
		"(NOT the BREAK->ROBUST cascade); the RX down-ladder catches a missed tag\n",
		reason ? reason : "?", current_configuration, demote_target);
	fflush(stdout);

	// (1) LOSSLESS DEMOTE bsi rollback — capture the earliest in-flight bsi BEFORE we
	// free messages_tx[] (the v2 in-order path only; the compression path owns its own
	// re-stage). Identical to the D3 demote (:4018-4038). All non-FREE frames of the
	// stranded block share one batch_seq_id on the per-frame path; keep the mod-256
	// earliest.
	int min_inflight_bsi = -1;
	if(sack_v2_enabled && !compression_enabled)
	{
		for(int i=0; i<nMessages; i++)
		{
			if(scream_rollback_slot_eligible(messages_tx[i].status,
				messages_tx[i].length, messages_tx[i].batch_seq_id))
			{
				// A slot staged for the NEXT build carries batch_seq_id=-1.  It is
				// pending data, not an in-flight wire generation; masking it to 255
				// can roll a SCREAM demote from bsi 1 to 255 and manufacture a gap.
				// Only already-aired frames own the rollback identity.
				int b = messages_tx[i].batch_seq_id & 0xFF;
				if(min_inflight_bsi < 0)
					min_inflight_bsi = b;
				else
				{
					unsigned fwd = ((unsigned)(min_inflight_bsi - b)) & 0xFFu;
					if(fwd >= 1u && fwd <= 128u)
						min_inflight_bsi = b;
				}
			}
		}
		// A preceding ordinary timeout may already have losslessly returned the
		// failed batch to fifo_buffer_tx and freed every messages_tx slot before
		// the asynchronous scream arrives.  In that state the next generation
		// after the last clean cumulative delivery is the stranded batch's wire
		// identity.  Its still-valid Option-W stamp proves that generation was
		// built; use it as the rollback anchor instead of leaving the advanced
		// dispatch counter in place.
		if(min_inflight_bsi < 0)
		{
			int delivered_hw = cmd_rxwindow_delivered_high_water;
			if(delivered_hw < 0) delivered_hw = cmd_last_applied_clean_bsi;
			int candidate = (delivered_hw < 0) ? 0 : ((delivered_hw + 1) & 0xFF);
			if(candidate != (cmd_batch_seq_id & 0xFF)
			   && tx_stream_stamp[candidate].valid)
			{
				printf("[INBAND-NOBREAK] RESTAGED DEMOTE: no live frame slots; "
					"clean_hw=%d selects stamped pending bsi=%d (dispatch=%d)\n",
					delivered_hw, candidate, cmd_batch_seq_id & 0xFF);
				fflush(stdout);
				min_inflight_bsi = candidate;
			}
		}
	}

	// (2) Preserve all pending data for resend at the demoted rung — the SAME FIFO
	// push-back the BREAK paths + D3 demote use, so no bytes are dropped.
	if(compression_enabled)
	{
		restore_tx_from_compressed();
	}
	else
	{
		restage_requeue_tx_messages();   // §12 lossless+ordered re-stage (was forward push()-to-BACK)
		fifo_buffer_backup.flush();
		clear_retx_queue();   // recovery re-queues plaintext; drop stale retx
	}
	block_under_tx = NO;

	// Roll the bsi epoch back to the in-flight batch so the re-sent batch is CONTIGUOUS
	// with the RSP delivery high-water (D3 :4062-4069). Only when an in-flight bsi was
	// actually captured (else leave the counter untouched).
	if(min_inflight_bsi >= 0)
	{
		printf("[INBAND-NOBREAK] LOSSLESS DEMOTE: rolling cmd_batch_seq_id %d -> %d "
			"so the re-sent batch carries the in-flight (contiguous) bsi (no D3.1 GAP-ABORT)\n",
			cmd_batch_seq_id & 0xFF, min_inflight_bsi);
		fflush(stdout);
		cmd_batch_seq_id = min_inflight_bsi;
	}

	// (3) Advance the config owners. negotiated_configuration is what the chokepoint
	// (add_message_control SET_CONFIG, :736-739, SUCCESS_BASED_LADDER path) reads as the
	// inband target. (4) Pin the proven ceiling so the SNR re-trigger does not
	// immediately re-elect the failing rung. (D3 :4075-4077.)
	data_configuration       = demote_target;
	negotiated_configuration = demote_target;
	// pin_ceiling default true (every BREAK/NACK caller) PINS the proven ceiling at the
	// demoted rung so the SNR re-trigger cannot immediately re-elect the rung that just
	// failed (D3 :4075-4077). The MERCURY_SNR_TRACK proactive demote passes false: it is a
	// precautionary SNR-sensed move, not a proven failure, so leaving the ceiling where it
	// was lets a recovered meter climb straight back (the RUNG_MIN_SNR_METER floor gate, not
	// a pinned ceiling, is the over-climb guard on that path).
	if(pin_ceiling)
		supershift_proven_ceiling = demote_target;

	// Consume the deadline: clear BOTH the nack streak and the revack-starve streak so a
	// fresh per-frame failure re-accumulates from 1. (D3 :4100-4101.)
	cfg16_revack_starve_fails = 0;
	emergency_nack_count = 0;

	// (5) Optimizer Axis-1 supremacy hook (a config MOVE; mirrors the BREAK paths + D3).
	// CROSS-LAYER NOTE (data-flow audit): when Axis-3 was ON/OFF this hook transitions it to
	// PROBE and tries to queue a SET_LINK_PARAMS (axis3_send_set_link_params, :14834), which
	// would OCCUPY messages_control and block the SET_CONFIG demote below. The SET_LINK_PARAMS
	// is best-effort (the helper's own busy-check + the EOB-self-correct safety net cover a
	// skipped one, :6628), but the SET_CONFIG demote is LOAD-BEARING (it IS the tag/transition).
	// So the demote takes priority: we clear any supremacy-queued SET_LINK_PARAMS before
	// queueing the SET_CONFIG. (In the production D3-demote context Axis-3 is already PROBE so
	// nothing is queued; this guard makes the reused helper robust to the ON/OFF case too.)
	if(sack_v2_enabled)
		policy_axis1_supremacy_on_move(current_configuration, demote_target, reason);
	if(messages_control.status != FREE && messages_control.data != NULL
	   && messages_control.length > 0 && messages_control.data[0] == SET_LINK_PARAMS)
	{
		// A supremacy-queued SET_LINK_PARAMS — defer it to the next cycle so the SET_CONFIG
		// demote (the load-bearing transition) wins the single control slot this batch.
		messages_control.status = FREE;
		messages_control.type   = NONE;
	}

	// (6) Emit the demote through the CHOKEPOINT. Under inband, add_message_control(
	// SET_CONFIG) takes the unilateral path (inband_unilateral_config_change(negotiated_configuration))
	// which applies the config NOW and the next send_batch W1-emits the CONFIG_TAG; the
	// inband_unilateral_armed one-shot re-routes connection_status back to TRANSMITTING_DATA
	// in process_messages_tx_control. Mirrors the D3 demote (:4113-4115) exactly.
	cleanup();
	add_message_control(SET_CONFIG);
	this->connection_status = TRANSMITTING_CONTROL;
	return true;
}

// STAGE 4c — the commander-side true-session-loss floor (the ONLY commander BREAK
// permitted under inband). A Class-A degradation that occurs WHILE already at the
// ladder bottom (nowhere to demote to) is a true-loss tick. Increment the streak; the
// genuine send_break_pattern() fires ONLY when it reaches SESSION_DEAD_BATCHES (the SAME
// MERCURY_INBAND_DEAD_BATCHES the RX terminal floor uses, so both sides break together).
bool cl_arq_controller::inband_cmd_dead_batch_floor_reached()
{
	cmd_inband_session_dead_batches++;
	int limit = inband_session_dead_limit();   // shared MERCURY_INBAND_DEAD_BATCHES, default 3
	printf("[INBAND-NOBREAK] true-loss tick: at ladder bottom, no rung to demote — "
		"commander dead-batch streak %d/%d\n",
		cmd_inband_session_dead_batches, limit);
	fflush(stdout);
	if(cmd_inband_session_dead_batches >= limit)
	{
		printf("[INBAND-NOBREAK] SESSION_DEAD_BATCHES floor reached (%d) — the link is "
			"genuinely dead; the ONE permitted BREAK fires (design §7)\n", limit);
		fflush(stdout);
		cmd_inband_session_dead_batches = 0;   // armed once; reset so BREAK isn't re-fired
		return true;
	}
	return false;
}

// ============================================================================
// In-band CONNECT-LIVENESS GUARD — the control-plane livelock backstop.
// data-flow-inband-connect-liveness.md §2. The retained §7 true-loss BREAK is
// wired only to a DATA-loss tick; a connect/negotiate handshake that stalls with
// ZERO forward DATA progress (HW: link_status==CONNECTED, ~92% TRANSMITTING_CONTROL,
// nAcked_data flat at 0) never ticks it, and link_timer is kicked by every control-ACK
// so the 10s session drop never fires. The guard observes stats.nAcked_data (the
// commander's monotonic forward-DATA-delivered counter) once per poll: if it makes NO
// advance across N consecutive polls WHILE NOT in a data-bearing phase, it fires the
// SAME true-loss send_break_pattern() recovery (BREAK->ROBUST_0, exactly how legacy
// recovers). Bounded to INBAND_LIVENESS_MAX_BREAKS per session, then a real session reset.
// (INBAND_LIVENESS_MAX_BREAKS is defined in arq.h so the regression test in
// arq_responder.cc sees the SAME bound.)
// ============================================================================

// FORWARD-HEALTHY discriminator (data-flow-inband-retx-epoch.md §5). True iff the
// commander still holds an in-flight forward DATA batch in messages_tx[] (one or more
// non-FREE, non-empty slots). This is the CMD-side proxy for "the link is still
// delivering forward / not genuinely dead": a forward-healthy reverse-ACK turnaround
// MISS leaves the just-aired batch sitting in messages_tx[] awaiting its (missed)
// reverse-ACK, so the slots are still occupied; a genuine connect/negotiate livelock
// (zero forward DATA ever) has NO in-flight DATA batch. The detector is the SAME
// non-FREE && length>0 scan every BREAK/demote re-stage uses (e.g. :298, :386, :2923).
// Const, side-effect-free; safe to call from the once-per-poll liveness guard.
bool cl_arq_controller::cmd_has_inflight_data_batch() const
{
	for(int i=0; i<nMessages; i++)
		if(messages_tx[i].status != FREE && messages_tx[i].length > 0)
			return true;
	return false;
}

// NOTE: inband_a3_decouple_enabled() is defined in arq_common.cc (next to the sibling
// inband_rate_feature_enabled gate); the declaration lives in arq.h. The re-target that
// consumes it is in inband_connect_liveness_guard() below.

int cl_arq_controller::inband_liveness_stall_polls_count()
{
	if(inband_liveness_stall_polls < 0)
	{
		int n = 200;  // owner default: ~tens of polls is a normal connect; 200 data-less
		              // control-plane polls is unambiguously a livelock (design §2)
		const char* e = std::getenv("MERCURY_INBAND_LIVENESS_POLLS");
		if(e && *e) { int v = atoi(e); if(v != 0) n = v; }
		if(n < 1) n = 1;
		inband_liveness_stall_polls = n;
	}
	return inband_liveness_stall_polls;
}

bool cl_arq_controller::inband_connect_liveness_guard()
{
	// OFF -> never entered (byte-identical default). Guard is commander-only.
	if(!inband_rate_feature_enabled())
		return false;

	// ACTIVE-v2 owns the complete experiment lifetime, not merely CONFIG_TAG dispatch.
	// The generic poll-count watchdog is telemetry-only while a switch is in flight OR
	// a confirmed probe remains in probation. It may not reinterpret Gearshift's own
	// bounded zero-progress window as link death.
	if(rate_opt.owns_link_experiment(opt_now_ms()))
	{
		if(cmd_inband_liveness_no_progress_polls != 0)
		{
			printf("[GEARSHIFT-V2-AUTHORITY] liveness watchdog yielded to owned experiment "
				"(streak=%d reset)\n", cmd_inband_liveness_no_progress_polls);
			fflush(stdout);
		}
		cmd_inband_liveness_no_progress_polls = 0;
		return false;
	}

#ifdef INBAND_LIVENESS_FAILBEFORE
	// FAIL-BEFORE arm: the guard is INERT — it only tracks the streak (so the test can
	// observe it growing unbounded) but NEVER fires a recovery. Rebuild with
	// -DINBAND_LIVENESS_FAILBEFORE to reproduce the livelock.
#endif

	// A data-bearing phase (actively pushing or awaiting data) is forward progress by
	// definition for liveness purposes: hold the streak at 0 so a slow data batch can
	// never trip the guard, and so a normal rate change (which stays in / quickly returns
	// to the DATA phases, with nAcked_data climbing) is invisible to it.
	bool data_phase = (connection_status == TRANSMITTING_DATA
	                || connection_status == RECEIVING_ACKS_DATA);

	// Forward-DATA-delivered advance since the last sample?
	if(stats.nAcked_data != cmd_inband_liveness_last_acked)
	{
		cmd_inband_liveness_last_acked    = stats.nAcked_data;
		cmd_inband_liveness_no_progress_polls = 0;
		cmd_inband_liveness_breaks        = 0;   // a delivery proves liveness; rearm the bound
		return false;
	}
	if(data_phase)
	{
		// In a data phase but nAcked_data hasn't ticked yet (mid-batch) — NOT a stall.
		cmd_inband_liveness_no_progress_polls = 0;
		return false;
	}

	// The guard ONLY arms once the link is CONNECTED (the ESTABLISHED-but-stalled
	// control-plane livelock it backstops; arq.h §"CONNECT-LIVENESS GUARD"). While
	// CONNECTING/NEGOTIATING/Idle/Dropped there is LEGITIMATELY no forward-DATA progress
	// yet (the handshake hasn't established the link), so accruing a no-data streak there
	// would false-fire a BREAK mid-handshake and prevent the link from ever connecting
	// (HW A/B: ON arm fired at link=CONNECTING and delivered 0 bytes). Hold the streak at
	// 0 until CONNECTED so a slow connect can never trip the guard.
	if(link_status != CONNECTED)
	{
		cmd_inband_liveness_no_progress_polls = 0;
		return false;
	}

	// CONNECT/NEGOTIATE CONTROL HANDSHAKE — the guard must NOT arm until a DATA session
	// is actually ESTABLISHED (data-flow-inband-connect-liveness.md §5; data-flow-inband-
	// a3-decouple.md). link_status==CONNECTED is reached BEFORE the post-connect WB-bandwidth
	// negotiate runs: when both ends are WB-capable, [BW-NEG] "initiating WB upgrade" queues a
	// single SWITCH_BANDWIDTH control op (one CONFIG_100/ROBUST_0 frame, ~9s on the wire) and
	// sits in TRANSMITTING_CONTROL/RECEIVING_ACKS_CONTROL with nAcked_data flat at 0 for the
	// whole negotiate (arq_commander.cc:5872-5879). That is a LEGITIMATE control handshake —
	// NOT a livelock — but its state signature is identical to the dead-control case, so the
	// bare link_status==CONNECTED arming false-fired a true-loss BREAK ~0.56s before the RSP
	// even received the SWITCH_BANDWIDTH, aborting the negotiate and delivering 0 bytes (the
	// legacy/OFF path runs the IDENTICAL ~9s negotiate then reaches DATA). The guard's purpose
	// is to backstop an ESTABLISHED-but-stalled DATA session; gate arming on "a DATA frame has
	// been queued/acked at least once," not bare CONNECTED. "Data has ever flowed" ⇔ a data ACK
	// was seen (cmd_inband_liveness_last_acked>0, monotonic per session) OR a data batch is
	// currently in flight (cmd_has_inflight_data_batch()). Until then hold the streak at 0.
	// BACKSTOP PRESERVED (the genuine-death net is intact): a POST-DATA control-plane livelock
	// (RECEIVING_ACKS_CONTROL stuck forever AFTER data flowed) ALWAYS satisfies one of the two —
	// last_acked>0 if any data was acked, OR an inflight batch is still queued — so the early-out
	// is skipped and the guard arms exactly as before. Only the pristine PRE-DATA connect/
	// negotiate handshake (no ack ever AND no batch queued) is exempted.
	// FAIL-BEFORE (-DINBAND_NEGOTIATE_FAILBEFORE, --test-inband-liveness PART F): the
	// connect/negotiate exemption is compiled OUT so a pre-DATA control handshake accrues
	// the streak and false-fires a BREAK at N — reproducing the WB-negotiate 0-deliver.
#ifndef INBAND_NEGOTIATE_FAILBEFORE
	bool data_ever_flowed = (cmd_inband_liveness_last_acked > 0)
	                     || cmd_has_inflight_data_batch();
	if(!data_ever_flowed
	   && (connection_status == TRANSMITTING_CONTROL
	    || connection_status == RECEIVING_ACKS_CONTROL))
	{
		cmd_inband_liveness_no_progress_polls = 0;
		return false;
	}
#endif

	// IN-BAND TIER-CROSSING CONTROL HANDSHAKE — the same exemption the WB-negotiate gets
	// (data-flow-inband-tier-crossing.md §3 PART B). VERIFIED root (off-bench,
	// _xfix/on_partA.arqlog): after data has flowed during the intra-ROBUST climb
	// (cmd_inband_liveness_last_acked=3), the redesign promotes robust->OFDM via the §1
	// hybrid SET_CONFIG control handshake. That cross legitimately sits in
	// TRANSMITTING_CONTROL / RECEIVING_ACKS_CONTROL with nAcked_data FLAT for the whole
	// cross (the SET_CONFIG ACK + the slow ~12.4s data-SACK turnaround at the robust rung),
	// but data_ever_flowed is now TRUE so the connect/negotiate exemption above no longer
	// covers it -> the guard accrues 200 polls and FALSE-FIRES a true-loss BREAK
	// ("connect/negotiate LIVELOCK"). That BREAK then anchor-clamps the cross back to the
	// robust anchor -> the 102<->0 oscillation (8/8 remaining BREAKs were this false-fire
	// with PART A's reverse-pin live). A deliberate UPWARD tier-cross is NOT a livelock to
	// retreat from — it is the intended move. EXEMPT it: while in a control phase AND the
	// in-flight config change (negotiated_configuration) CROSSES the tier relative to the
	// live config, hold the streak at 0 (mirrors the WB-negotiate hold). BOUNDED + genuine-
	// death net intact: a truly stuck cross is owned by the control-ACK-miss / FRAME-UP-
	// FAILED machinery (arq_commander.cc:2897/:2984 route it to the tag-demote), NOT this
	// guard; once the cross ACKs or the gearshift abandons it the state leaves the control
	// phase and the streak resumes. Strictly scoped to inband (the function early-returns
	// OFF) + a real crossing -> intra-tier rate adapts and the legacy path are unaffected.
	// FAIL-BEFORE (-DINBAND_TIERCROSS_LIVENESS_FAILBEFORE): the exemption is compiled OUT so
	// the cross accrues the streak and false-fires — reproducing the oscillation.
#ifndef INBAND_TIERCROSS_LIVENESS_FAILBEFORE
	if(inband_tiercross_handshake_exempts_liveness(connection_status,
			negotiated_configuration, cmd_has_inflight_data_batch()))
	{
		cmd_inband_liveness_no_progress_polls = 0;
		return false;
	}
#endif

	// A live control exchange (a queued control message awaiting its ACK within its own
	// timeout and retry budget) is not a livelock: its ACK wait, retries and terminal
	// miss handling own it.  Counting its polls fired this guard ~1.3 s into every
	// ordinary SET_LINK_PARAMS round trip (200 polls at the few-ms control-poll cadence)
	// and fed Gearshift a fabricated whole-transaction failure, which it correctly read
	// as a worse regime and so suppressed upward probes after each batch-size change.
	// MERCURY_GS2_LIVENESS_CTRL_EXEMPT=0 restores the previous accrual.
	if(gs2_liveness_control_exchange_exempt())
	{
		cmd_inband_liveness_no_progress_polls = 0;
		return false;
	}
	// Control-TX / Idle / control-ACK-wait with NO forward DATA progress: accrue.
	cmd_inband_liveness_no_progress_polls++;
	if(cmd_inband_liveness_no_progress_polls < inband_liveness_stall_polls_count())
		return false;

	// --- Stall confirmed: the connect/negotiate handshake is livelocked. ---
	cmd_inband_liveness_no_progress_polls = 0;   // re-arm the window for any next stall

	// Under ACTIVE this is a FAILURE SIGNAL, not an independent recovery decision.
	// If probe probation has exhausted its OWN zero-progress deadline, route the
	// signal to the probe owner, which may only choose its declared fallback.
	// Otherwise record one outcome-only failed transaction and run the full v2
	// selector. The watchdog never nominates a rung.
	if(rate_opt.controls_link())
	{
		if(rate_opt.probe_is_active())
		{
			int hint = config_ladder_down(current_configuration, robust_enabled);
			printf("[GEARSHIFT-V2-AUTHORITY] liveness signal after owner probation deadline "
				"at cfg=%d; requesting probe-owned resolution\n", current_configuration);
			fflush(stdout);
			if(hint != current_configuration)
				return inband_route_failure_demote(hint, "liveness_stall_after_probe_budget");
		}

		opt_record_batch(/*transport_bytes_banked=*/0, /*sack_used=*/false,
			/*failed=*/true, /*frames_acked=*/0, /*frames_sent_override=*/1,
			/*outcome_only=*/true);
		int owner_target = current_configuration;
		// Gearshift decisions are actuated at a data-batch boundary.  A commander parked in
		// TRANSMITTING_CONTROL with an EMPTY control slot (a control exchange that was
		// abandoned, or a legacy SET_CONFIG request the ownership firewall refused) has
		// nothing to send and never reaches that boundary, so every owner decision below
		// would be stranded until the link timer expires.  Return such a commander to the
		// data plane so the owner's action is actually dispatched.
		// MERCURY_GS2_EMPTY_CTRL_RESUME=0 restores the previous behaviour.
		const bool empty_ctrl_resume = gs2_empty_control_state_resumes_data();
		if(opt_evaluate_batch_end(&owner_target)
		   && owner_target != current_configuration)
		{
			if(empty_ctrl_resume) this->connection_status = TRANSMITTING_DATA;
			opt_pending_switch_cfg = owner_target;
			printf("[GEARSHIFT-V2-AUTHORITY] liveness telemetry -> owner selected "
				"%d->%d action=%d; generic BREAK suppressed\n",
				current_configuration, owner_target, (int)opt_pending_switch_action);
			fflush(stdout);
			return true;
		}
		if(!config_is_at_bottom(current_configuration, robust_enabled))
		{
			if(empty_ctrl_resume) this->connection_status = TRANSMITTING_DATA;
			printf("[GEARSHIFT-V2-AUTHORITY] liveness telemetry -> owner HOLD/ABSTAIN "
				"at cfg=%d above floor; generic BREAK suppressed\n",
				current_configuration);
			fflush(stdout);
			return true;
		}
		// At the actual floor, fall through to the recovery actuator. The
		// send_break_pattern() chokepoint still requires Gearshift's explicit
		// no-admissible-candidate verdict before RF emission.
	}

#ifdef INBAND_LIVENESS_FAILBEFORE
	printf("[INBAND-LIVENESS] FAILBEFORE: stall detected but recovery DISABLED — livelock "
		"persists (cmd_inband_liveness_breaks=%d)\n", cmd_inband_liveness_breaks);
	fflush(stdout);
	return false;   // no recovery: the livelock is reproduced (test asserts BREAK-count==0)
#else
	// ── FORWARD-HEALTHY REVERSE-ACK MISS → NO-BREAK RE-PRESENT (data-flow-inband-retx-
	//    epoch.md §5; closes HOLE A/B/C of the in-band 0-deliver). ──────────────────────
	// stats.nAcked_data advances ONLY when the CMD RECEIVES a reverse-ACK/SACK (the only
	// producers are arq_commander.cc:61/:3659/:3688/:3714). A forward-HEALTHY reverse-ACK
	// turnaround MISS — the RX is decoding our forward DATA fine, but its SACK lands
	// outside the CMD listen window — therefore leaves nAcked_data FLAT and trips this
	// stall, even though the link is alive. Firing send_break_pattern() here detonates the
	// three coupled holes (the EXHAUSTED-BREAK clears the retx queue + advances the bsi
	// epoch; the RX break_detected handler wipes the in-flight PARTIAL prev; a config-NO-OP
	// teardown of a healthy link), delivering 0 bytes of a still-decoding 785-frame stream.
	//
	// DISCRIMINATOR (correct at BOTH edges):
	//   forward-HEALTHY  ⇔ an in-flight forward DATA batch is still queued in messages_tx[]
	//                      (cmd_has_inflight_data_batch()) AND a lower rung exists
	//                      (!config_is_at_bottom). Route to inband_route_failure_demote():
	//                      it PRESERVES the in-flight retx queue + partial prev (no
	//                      send_break_pattern ⇒ the RX never wipes the prev), rolls
	//                      cmd_batch_seq_id back to the in-flight bsi (the re-sent batch is
	//                      CONTIGUOUS ⇒ no GAP-ABORT), and re-presents on the lower rung —
	//                      so the partial batch DELIVERS. NOT a liveness BREAK; does NOT
	//                      consume the INBAND_LIVENESS_MAX_BREAKS budget; does NOT tick the
	//                      true-loss floor (a re-present is not a death).
	//   genuine DEAD/livelock ⇔ NO in-flight DATA batch (a connect/negotiate handshake stuck
	//                      with zero forward DATA ever) OR already at the ladder bottom (no
	//                      rung to demote to). Falls through UNCHANGED to the bounded
	//                      §7 true-loss BREAK→ROBUST_0 below — the genuine recovery is intact.
	// Too-narrow edge (re-opens the 0-deliver) is closed because the demote preserves the
	// partial prev + contiguous bsi. Too-broad edge (stranding a dead session in a no-break
	// loop) is closed because each demote LOWERS the rung; at the bottom the demote returns
	// false and the genuine BREAK fires — and the INBAND_LIVENESS_MAX_BREAKS hard-reset below
	// still backstops a link the demotes cannot rescue.
	// FAIL-BEFORE (-DINBAND_DELIVER_FAILBEFORE, --test-inband-deliver): the discriminator is
	// REMOVED so a forward-healthy miss falls through to the BREAK below — reproducing the
	// retx-clear/bsi-advance/partial-prev-wipe 0-deliver the rework closes.
#ifndef INBAND_DELIVER_FAILBEFORE
	if(cmd_has_inflight_data_batch()
	   && !config_is_at_bottom(current_configuration, robust_enabled))
	{
		// ── A3 DEMOTE-DECOUPLE (data-flow-inband-a3-decouple.md §1.2; owner-gated, DEFAULT-OFF). ──
		// When the gate is ON (MERCURY_INBAND_A3_DECOUPLE set AND cumulative_ack_enabled negotiated)
		// a forward-healthy reverse-ACK MISS is NOT a reason to demote: the missed EOB SACK is
		// SUPERSEDED for free by the next turn's cumulative n_r (STANAG-5066 acknowledged-through),
		// so demoting one rung per stall (the HW CFG15->6 config-walk that escapes the RSP D=4 window
		// and desyncs the session) is the WRONG remedy. Instead RE-AIR THE SAME CONFIG: leave
		// current_configuration, cmd_batch_seq_id, messages_tx[] and the retx queue UNTOUCHED (the
		// in-flight batch stays queued; the normal per-frame retx/re-air machinery re-presents it on
		// the next send), re-arm the stall window, and DO NOT tick cmd_inband_liveness_breaks (a
		// re-air is not a death). The genuine-death nets are intact: a truly dead reverse channel
		// never advances the cumulative n_r, so its in-flight batch stays uncovered and the real
		// decode-failure demote (frame_gearshift_data_failed_*) + the INBAND_LIVENESS_MAX_BREAKS
		// hard-reset below still fire. CRITICAL: this path must NOT advance cmd_batch_seq_id (that
		// would orphan the in-flight batch into a gap); we touch NEITHER the epoch NOR the config.
#ifndef INBAND_A3_DECOUPLE_FAILBEFORE
		if(inband_a3_decouple_enabled())
		{
			printf("[INBAND-A3-DECOUPLE] forward-healthy reverse-ACK MISS at config %d — RE-AIRING "
				"the SAME config (NOT demoting): the missed ACK self-heals via the cumulative n_r "
				"(MERCURY_INBAND_A3_DECOUPLE + cumulative_ack negotiated). cmd_batch_seq_id=%d "
				"UNCHANGED, in-flight batch preserved for the normal retx re-air.\n",
				current_configuration, cmd_batch_seq_id & 0xFF);
			fflush(stdout);
			// Re-arm the stall window (same as the demote path) so the next miss re-accumulates
			// from 1; do NOT tick cmd_inband_liveness_breaks (a re-air is not a BREAK). Leave
			// current_configuration / cmd_batch_seq_id / messages_tx[] / retx queue untouched.
			cmd_inband_liveness_no_progress_polls = 0;
			return true;   // the re-air owns the next transition (caller returns)
		}
#endif // INBAND_A3_DECOUPLE_FAILBEFORE
		int demote_target = config_ladder_down(current_configuration, robust_enabled);
		if(inband_route_failure_demote(demote_target, "forward_healthy_revack_miss"))
		{
			printf("[INBAND-LIVENESS] forward-healthy reverse-ACK MISS (nAcked_data flat but a "
				"forward DATA batch is in flight at config %d) — routed to the NO-BREAK "
				"re-present (demote -> %d), NOT the BREAK->ROBUST cascade; the in-flight partial "
				"batch is preserved and delivers contiguously\n",
				current_configuration, demote_target);
			fflush(stdout);
			// A re-present is forward progress, not a death — re-arm the stall window and the
			// liveness-break budget so a recovered link is not later mis-classified as dead.
			cmd_inband_liveness_no_progress_polls = 0;
			cmd_inband_liveness_breaks            = 0;
			return true;   // the demote owns the next transition (caller returns)
		}
		// demote was a no-op (raced to a bottom rung) — fall through to the genuine BREAK.
	}
#endif // INBAND_DELIVER_FAILBEFORE

	if(rate_opt.controls_link()
	   && !rate_opt.authorize_hard_recovery(current_configuration,
			config_is_at_bottom(current_configuration, robust_enabled),
			"liveness-floor-recovery"))
	{
		printf("[GEARSHIFT-V2-AUTHORITY] liveness hard-recovery request denied by owner; "
			"BREAK/reset budget not advanced\n");
		fflush(stdout);
		return true;
	}

	if(cmd_inband_liveness_breaks >= INBAND_LIVENESS_MAX_BREAKS)
	{
		// Bounded escalation: repeated liveness BREAKs did not recover the link -> it is
		// genuinely dead. Drop the session via the SAME path the link_timer watchdog uses
		// (reset_session_state + reset_all_timers) so the guard cannot thrash forever.
		printf("[INBAND-LIVENESS] %d liveness BREAKs did not recover — hard session reset "
			"(DROPPED)\n", cmd_inband_liveness_breaks);
		fflush(stdout);
		link_status = DROPPED;
		reset_session_state();
		reset_all_timers();
		cmd_inband_liveness_breaks = 0;
		cmd_inband_liveness_last_acked = stats.nAcked_data;
		return true;
	}

	cmd_inband_liveness_breaks++;
	printf("[INBAND-LIVENESS] no forward-DATA progress for %d control-plane polls "
		"(nAcked_data=%d, connection_status=%d, link=%d) — connect/negotiate LIVELOCK; "
		"firing the retained true-loss BREAK (#%d/%d) to resync\n",
		inband_liveness_stall_polls_count(), stats.nAcked_data,
		(int)connection_status, (int)link_status,
		cmd_inband_liveness_breaks, INBAND_LIVENESS_MAX_BREAKS);
	fflush(stdout);

	// Fire the SAME §7 true-loss BREAK recovery (arq_commander.cc:4665-4678): BREAK ->
	// ROBUST_0 resync via the emergency-break state machine. Cancel any stale control slot
	// first (it owns nothing the resync needs).
	messages_control.ack_timeout = 0;
	messages_control.id          = 0;
	messages_control.length      = 0;
	messages_control.nResends    = 0;
	messages_control.status      = FREE;
	messages_control.type        = NONE;

	emergency_previous_config = current_configuration;
	emergency_break_active     = 1;
	emergency_break_retries    = 3;
	// Axis-1 supremacy re-asserts the config on the wire for the resync (it may queue a
	// FRESH SET_LINK_PARAMS via axis3 — that is the recovery's new control op, NOT the
	// stale handshake we just cancelled above).
	if(sack_v2_enabled)
		policy_axis1_supremacy_on_move(current_configuration,
			current_configuration, "inband_connect_liveness_stall");
	send_break_pattern();
	if(telecom_system != NULL)
		telecom_system->data_container.frames_to_read = 4;
	calculate_receiving_timeout();
	receiving_timer.start();
	return true;   // recovery fired; caller returns (the emergency-break SM owns next poll)
#endif
}

void cl_arq_controller::process_messages_rx_acks_data()
{
	// Watch the newest causal audio for one exact acquisition repetition. Once
	// acquired, only the CRC-bearing compact decoder may inspect this response.
	// With no prefix (partial/retx response), the existing reactive OFDM path is
	// never delayed. send_batch() arms the watch only for original D5 data.
	if(t1_cmd_capture_hold_active && !t1_cmd_prefix_acquired)
	{
		int rwi = 0, signal_period = 0, sym_samples = 0;
		MUTEX_LOCK(&capture_prep_mutex);
		rwi = telecom_system->data_container.ring_write_index;
		sym_samples = telecom_system->data_container.Nofdm
			* telecom_system->data_container.interpolation_rate;
		signal_period = sym_samples
			* telecom_system->data_container.buffer_Nsymb;
		MUTEX_UNLOCK(&capture_prep_mutex);
		if(rwi != t1_cmd_capture_hold_start_rwi && signal_period > 0
		   && sym_samples > 0)
		{
			const int watch_syms = 24;
			int watch_samples = watch_syms * sym_samples;
			if(watch_samples > signal_period) watch_samples = signal_period;
			const int watch_offset = signal_period - watch_samples;
			MUTEX_LOCK(&capture_prep_mutex);
			memcpy(telecom_system->data_container.ready_to_process_passband_delayed_data,
				&telecom_system->data_container.passband_delayed_data[rwi + watch_offset],
				(size_t)watch_samples * sizeof(double));
			MUTEX_UNLOCK(&capture_prep_mutex);
			t1_cmd_capture_hold_start_rwi = rwi;
			int matched = 0;
			double metric = telecom_system->detect_prekey_prefix_from_passband(
				telecom_system->data_container.ready_to_process_passband_delayed_data,
				watch_samples, &matched);
			// Exact 5/5 plus the same strong metric floor used by the production
			// scream token. A 5/5 bin coincidence at a weak metric must not fence
			// a legitimate partial OFDM response.
			if(matched == cl_mfsk::PREKEY_PREFIX_LEN && metric >= 3.0)
			{
				t1_cmd_prefix_acquired = true;
				mtl::log_event_kv("t1_cmd_prefix_acquired",
					"matched=%d metric=%.3f rwi=%d", matched, metric, rwi);
			}
		}
	}
	// Silence is the expected success signal between clean members of a
	// negotiated block. Once this bounded window closes, transfer the legacy
	// one-batch cache to the already-SENT journal owner and build the next BSI.
	// No ACK counters, delivery flags, optimizer credit, or journal settlement
	// are touched here; only a validated BLOCK_SACK can do those things.
	// STAGE R: intermediate advance threshold is the FULL receiving_timeout on the
	// legacy path (byte-identical) but only a SHORT gap under pipelining. The boundary
	// window (aggregate_response_outstanding()) is UNCHANGED: intermediate_silence_
	// expected() is false there, so this early advance never fires at the boundary and
	// the site-1/site-3 escalation is untouched.
	unsigned int intermediate_advance_timeout =
		l1_pipeline_intermediate_advance_timeout();
	if(l1_blockack.intermediate_silence_expected()
	   && receiving_timer.get_elapsed_time_ms() >= intermediate_advance_timeout)
	{
		restore_sack_v2_rx_phy();
		l1_release_current_batch_to_journal();
		receiving_timer.stop();
		receiving_timer.reset();
		load_configuration(data_configuration, PHYSICAL_LAYER_ONLY, YES);
		connection_status = TRANSMITTING_DATA;
		if(l1_blockack_pipeline_active())
			printf("[L1-BLOCKACK-PIPELINE] intermediate gap advance gap_ms=%u cfg=%d\n",
				intermediate_advance_timeout, current_configuration);
		else
			printf("[L1-BLOCKACK-CMD] silent intermediate window closed; "
				"journal retains ownership, advancing forward burst\n");
		fflush(stdout);
		return;
	}
	if(receiving_timer.get_elapsed_time_ms() >= receiving_timeout)
		restore_sack_v2_rx_phy();
	if (receiving_timer.get_elapsed_time_ms()<receiving_timeout)
	{
		// T2 SCREAM WAKE rides the RSP's normal reverse response slot.  Sniff it
		// before ACK/SACK dispatch so a full scream cannot be misclassified as a
		// partial control response.  The demotion helper restages the in-flight
		// bytes and emits CONFIG_TAG; it owns the resulting state transition.
		if(data_ack_received == NO && !scream_resume_pending
		   && !t1_cmd_prefix_acquired
		   && is_ofdm_config(current_configuration))
		{
			int scream_rung = -1;
			if(receive_scream_pattern(&scream_rung)
			   && apply_scream_wake(scream_rung))
				return;
		}

		if(ack_pattern_time_ms > 0 || l1_blockack_data_active())
		{
			// Detection strategy: check SACK alongside ACK from the start.
			// We only require a short minimum delay (ack_pattern_time_ms) so
			// the RSP has time to send its response. Step 15: the legacy
			// receive_sack_pattern() correlator is gone — SACK detection is
			// now exclusively the OFDM SACK_RSP decode in the v2 branch below.
			bool sack_window_open = (sack_enabled || l1_blockack_data_active()
				|| link_status == CONNECTED)
				&& data_ack_received == NO
				&& receiving_timer.get_elapsed_time_ms() > (unsigned int)(ack_pattern_time_ms);

			SACK_TRACE("poll bsi=%d cfg=%d rx_t=%ums ack_t=%dms sack_en=%d v2_en=%d window=%d axis3=%d dar=%d peak_match=%d",
				cmd_batch_seq_id, current_configuration,
				receiving_timer.get_elapsed_time_ms(), ack_pattern_time_ms,
				sack_enabled ? 1 : 0, sack_v2_enabled ? 1 : 0,
				sack_window_open ? 1 : 0, (int)axis3_sack_mode,
				(int)data_ack_received, ack_diag_peak_matched);

			bool sack_detected = false;
			bool sack_bitmap[MAX_SACK_BATCH_SIZE];
			// §7.13.29 — set true when the strict-threshold MFSK detector
			// confirmed an ACK this poll. The fallthrough ACK_PAT handler
			// reads this to accept the ACK without re-running receive_ack_pattern
			// (which would use the default lenient threshold and risk a
			// false-fire on OFDM SACK_RSP body audio).
			bool v2_ack_pat_pre_detected = false;

			if(sack_window_open)
			{
				memset(sack_bitmap, 0, sizeof(sack_bitmap));
				if(l1_blockack_data_active()
				   || (sack_v2_enabled && axis3_sack_mode != SACK_MODE_OFF)
				   || !sack_v2_enabled)
				{
					// Step 6 of MFSK-suffix ACK+SACK redesign — CMD-side MFSK probe.
					// Before the OFDM dispatch below, sniff the passband tail for an
					// MFSK ACK pattern + 10-symbol SACK suffix carrying (batch_seq_id,
					// 30-bit bitmap). On a clean decode we short-circuit straight to
					// the same downstream handlers that OFDM_ACK_CLEAN / SACK_RSP feed
					// (v2_ack_pat_pre_detected flag for clean, sack_detected flag for
					// partial). On miss we fall through to the existing OFDM path
					// untouched. Gated on:
					//   - MFSK_ACK_SACK_ENABLED  (compile-time master switch)
					//   - ack_sack_suffix_len() > 0  (WB only; NB has M<16)
					// (optimizer_is_in_control() gate dropped 2026-05-24 — the
					//  pattern correlator floor + CRC12 false-accept guard make
					//  this path safe below the Q-table band too, and we want
					//  ROBUST_0-grade ACK survival precisely where the optimizer
					//  isn't yet active. See mfsk-robust-ack.md §3 / §8 step 5.)
					bool mfsk_handled_this_poll = false;
#if MFSK_ACK_SACK_ENABLED
					if(telecom_system->ack_mfsk.ack_sack_suffix_len() > 0)
					{
						// ── Option B fix-b (data-flow-compact-confirm.md §10.4/§10.5):
						// COMPACT-CONFIRM accept INSIDE the SACK window ──────────────
						// THE SECOND live-land defect. For a batch>1, SACK-on, clean, WB
						// session the RSP emits the COMPACT confirm (arq_responder.cc:2576);
						// but this Branch-2 SACK-window probe historically decoded ONLY the
						// 13-uncoded ACK+SACK (decode_ack_sack_from_passband below, CRC12
						// over the 5-byte [bsi||bitmap]). The compact tail (CRC12 over the
						// single [bsi] byte) FAILS that 5-byte CRC12 (:3841) and is dropped;
						// the SACK-closed compact arm (cmd_compact_confirm_live_accept) HARD-
						// BAILS at :287 if(sack_window_open). Result: the confirm is lost,
						// CMD waits out receiving_timeout and retransmits -> the 4.7x
						// batch>1-WB delivery regression at ENABLE=1.
						//
						// FIX (root cause, not a band-aid): try the SELF-VALIDATING compact
						// decode FIRST, here, INSIDE the window, BEFORE the 13-uncoded decode.
						// cmd_compact_confirm_crc_valid() does its OWN tail snapshot + 16-sym
						// base detect + GF(16) K=5 soft-decode + CRC12-over-[bsi] + bsi-in-
						// window — the CRC12 (NOT the SACK window) is the false-confirm guard.
						// On a valid compact CRC we route through the EXACT clean-confirmation
						// state the 13-uncoded CLEAN branch (:3943-3966) sets: split-dedupe,
						// cmd_last_applied_clean_bsi, inband_retag_confirm_from_sack,
						// v2_ack_pat_pre_detected=true, mfsk_handled_this_poll=true. That
						// flag suppresses the OFDM dispatch AND the 13-uncoded decode for this
						// poll, and v2_ack_pat_pre_detected routes to the proven CLEAN funnel
						// (:4448) — NO commit_ack_pattern_consumed()/frames_to_read mutation
						// here (the v2 clean path does not advance the ring that way; the OFDM
						// dispatch it skips owns frames_to_read=0), so there is NO double-
						// advance vs the SACK-closed Branch-3 arm. On a compact MISS we fall
						// straight through to the UNCHANGED 13-uncoded decode below.
						//
						// §5 INVARIANTS re-verified: (1) no-cross-validate (I3/§6) — the
						// compact CRC12-over-[bsi] cannot validate a 13-uncoded [bsi||bitmap]
						// suffix and vice-versa, so the FAR is unchanged and a 13-uncoded clean
						// ACK still takes the existing path. (2) partial-SACK intact — the
						// compact decode REJECTS the SACK partial frame (different field
						// layout/length), so the partial branch (:3967) is untouched; a partial
						// is NEVER routed through here. (3) clean/partial routing + the dedupe
						// clean-vs-partial split (:3937) preserved: we dedupe vs
						// cmd_last_applied_clean_bsi exactly like the 13-uncoded clean branch,
						// so a repeated compact for the same bsi does not double-count
						// nBatches_fully_acked. Gated on ARQ_COMPACT_CONFIRM_ENABLE (held off)
						// => byte-identical when 0 (the if() short-circuits on the macro).
						// Extracted to ONE shared predicate (data-flow §10.5 fix-b) so production
						// and the live-RX regression (test_compact_confirm_sack_window_rx_path) drive
						// the SAME copy — no comment/code drift (the cl_arq_controller §5 precedent,
						// cmd_compact_confirm_live_accept). Returns true iff a compact confirm was
						// accepted/consumed this poll (v2_ack_pat_pre_detected set inside on a CLEAN).
						if(cmd_compact_confirm_sack_window_accept(
							ARQ_COMPACT_CONFIRM_ENABLE != 0,
							&v2_ack_pat_pre_detected))
						{
							mfsk_handled_this_poll = true;
							if(t1_cmd_prefix_acquired)
								mtl::log_event("t1_cmd_prefix_confirmed");
							t1_cmd_capture_hold_active = false;
							t1_cmd_prefix_acquired = false;
							t1_cmd_capture_hold_samples = 0;
						}
						else if(t1_cmd_prefix_acquired)
						{
							// A real acquisition unit was received but the compact CRC
							// is not complete/valid yet. Consume nothing and retry on
							// later audio; timeout/retransmit remains the safe fallback.
							mfsk_handled_this_poll = true;
						}
						// On a compact accept we are DONE for this poll; skip the 13-uncoded
						// ACK+SACK decode + OFDM dispatch entirely (the compact tail is NOT a
						// valid 5-byte [bsi||bitmap] frame, so decoding it as one only burns
						// a CRC12 fail). The 13-uncoded path runs verbatim on a compact MISS.
						if(!mfsk_handled_this_poll)
						{
						// Snapshot the passband tail — same window math as
						// receive_ack_pattern() (arq_common.cc:4966-4983).
						int ack_nsymb = telecom_system->ack_mfsk.ack_pattern_nsymb;
						int pattern_len = telecom_system->ack_mfsk.ack_snr_pattern_nsymb();
						// Suffix capture needs the longer of SNR vs SACK suffix tail.
						int sack_suffix_len = telecom_system->ack_mfsk.ack_sack_suffix_len();
						if(sack_suffix_len > pattern_len - ack_nsymb)
							pattern_len = ack_nsymb + sack_suffix_len;
						const int mfsk_tail_nsymb = ack_nsymb + pattern_len + 16;
						int sym_samples = telecom_system->data_container.Nofdm
						                * telecom_system->data_container.interpolation_rate;
						int signal_period = sym_samples * telecom_system->data_container.buffer_Nsymb;
						int tail_samples = mfsk_tail_nsymb * sym_samples;
						if(tail_samples > signal_period)
							tail_samples = signal_period;
						int tail_offset = signal_period - tail_samples;

						// Take a read-only snapshot of the tail. We deliberately
						// do NOT touch frames_to_read here — the v2 OFDM dispatch
						// below owns audio-advance bookkeeping; the MFSK probe is
						// a no-side-effect peek into the same ring window.
						MUTEX_LOCK(&capture_prep_mutex);
						int rwi_mfsk = telecom_system->data_container.ring_write_index;
						memcpy(telecom_system->data_container.ready_to_process_passband_delayed_data,
							&telecom_system->data_container.passband_delayed_data[rwi_mfsk + tail_offset],
							tail_samples * sizeof(double));
						MUTEX_UNLOCK(&capture_prep_mutex);

						uint8_t  rx_bsi = 0;
						uint32_t rx_bitmap = 0;
						uint16_t rx_crc12 = 0;
						int      mfsk_matched = 0;
						bool decoded = telecom_system->decode_ack_sack_from_passband(
							telecom_system->data_container.ready_to_process_passband_delayed_data,
							tail_samples, &rx_bsi, &rx_bitmap, &rx_crc12, &mfsk_matched);
						int guard_ref_bsi = partial_guard_reference_bsi();

						// Multi-window recovery (Track A, mwcorr;
						// fact-documents/data-flow-data-ack-sack-correlator.md §6).
						// The newest-tail decode just MISSED (decoded==false). On a
						// long held-CFG16 forward batch the reverse ACK+SACK arrives
						// once, late and mis-phased, then trailing idle silence
						// scrolls it out of the newest tail before this poll fired
						// (bench-9 matched=0/7, peak_metric=0.00 PURE-SILENT). The
						// burst is NOT lost — it sits at an OLDER ring phase. Scan
						// back through the retained history (ACK-pattern strides,
						// energy-gated, SAME decode + CRC12), and if a CRC-valid
						// ACK+SACK is found at an older phase, re-snapshot that phase
						// and re-run the decode here so the UNCHANGED CRC12 +
						// bsi-window + bitmap + dedupe body below validates it
						// verbatim. Env-gated (MERCURY_DATA_ACK_MULTIWINDOW, default
						// OFF -> byte-identical to monitor 627c370). A genuine
						// all-silence miss returns no phase and leaves the snapshot
						// at the newest tail (this poll falls through unchanged).
						if(!decoded && mw_data_ack_multiwindow_enabled())
						{
							int chosen_off = -1;
							if(mw_find_ack_sack_phase(rwi_mfsk, tail_offset,
								tail_samples, sym_samples, pattern_len, &chosen_off)
								&& chosen_off >= 0)
							{
								MUTEX_LOCK(&capture_prep_mutex);
								memcpy(telecom_system->data_container.ready_to_process_passband_delayed_data,
									&telecom_system->data_container.passband_delayed_data[rwi_mfsk + chosen_off],
									tail_samples * sizeof(double));
								MUTEX_UNLOCK(&capture_prep_mutex);
								rx_bsi = 0; rx_bitmap = 0; rx_crc12 = 0; mfsk_matched = 0;
								decoded = telecom_system->decode_ack_sack_from_passband(
									telecom_system->data_container.ready_to_process_passband_delayed_data,
									tail_samples, &rx_bsi, &rx_bitmap, &rx_crc12, &mfsk_matched);
							}
						}
						const bool ack_frame_seen = decoded;
						const uint8_t wire_bsi = rx_bsi;

						// CRC12 verification (mercury/fact-documents/mfsk-robust-ack.md §3.2).
						// On mismatch, treat as no-ACK — the timeout-retransmit path
						// is the safe fallback when a corrupted "looks like a clean
						// ACK" frame could otherwise cause silent data loss.
						int resolved_target = -1;
						bool is_clean_confirmation = false;
						bool bitmap_ok = false;
						bool bitmap_all = false;
						bool target_is_shadow = false;
						bool target_owned = false;
						bool reverse_demand = false;
						if(decoded)
						{
							char crc_input[5];
							crc_input[0] = (char)wire_bsi;
							crc_input[1] = (char)((rx_bitmap >> 24) & 0xFF);
							crc_input[2] = (char)((rx_bitmap >> 16) & 0xFF);
							crc_input[3] = (char)((rx_bitmap >>  8) & 0xFF);
							crc_input[4] = (char)( rx_bitmap        & 0xFF);
							uint16_t expected_crc12 = CRC12_calc(crc_input, 5);
							if(rx_crc12 != expected_crc12)
							{
								printf("[CMD-MFSK-ACK-SACK] CRC12 fail "
									"bsi=%u bitmap=0x%08x rx_crc=0x%03x expected=0x%03x "
									"matched=%d — discarding\n",
									(unsigned)rx_bsi, (unsigned)rx_bitmap,
									(unsigned)rx_crc12, (unsigned)expected_crc12,
									mfsk_matched);
								fflush(stdout);
								decoded = false;
							}
							else
								rx_bsi = role_demand_restore_ack_bsi(wire_bsi, &reverse_demand);
						}
						if(decoded)
						{
							is_clean_confirmation =
								mfsk_sack_bitmap_is_clean(rx_bitmap, data_batch_size);
							bitmap_all = data_batch_size > 0;
							for(int i=0; i<data_batch_size && i<MAX_SACK_BATCH_SIZE; i++)
							{
								const bool bit = mfsk_sack_bitmap_bit(rx_bitmap, i);
								bitmap_ok = bitmap_ok || bit;
								bitmap_all = bitmap_all && bit;
							}
							st_resolved_generation_ack resolved = {};
							target_owned = resolve_and_validate_generation_ack(
								rx_bsi, data_batch_size, /*integrity_ok=*/true,
								is_clean_confirmation,
								bitmap_ok, bitmap_all, &resolved);
							resolved_target = resolved.target;
							target_is_shadow = resolved.shadow_owned;
							if(!target_owned)
							{
								printf("[CMD-MFSK-ACK-SACK-REJECT] wire=%u target=%u owner=0 "
								       "bitmap_any=%d clean=%d bitmap_all=%d span=%d -- no credit\n",
									(unsigned)rx_bsi, (unsigned)resolved_target,
									bitmap_ok ? 1 : 0, is_clean_confirmation ? 1 : 0,
									bitmap_all ? 1 : 0, data_batch_size);
								fflush(stdout);
								decoded = false;
							}
							else if(std::getenv("MERCURY_GEN_CANON_WITNESS"))
							{
								printf("[GEN-CANON-RESOLVE] transport=MFSK wire=%u target=%u "
								       "canon=%d cumulative=%d clean=%d span=%d owner=%s\n",
									(unsigned)rx_bsi, (unsigned)resolved_target,
									generation_canon_enabled() ? 1 : 0,
									cumulative_ack_enabled ? 1 : 0,
									is_clean_confirmation ? 1 : 0, data_batch_size,
									target_is_shadow ? "shadow" : "live/journal");
								fflush(stdout);
							}
							if(decoded && target_owned)
								role_demand_note_acknowledged(reverse_demand,
									"mfsk-ack-sack");
						}

						// RETIRED §6 base-pattern climb-confirm (data-flow-inband-basepattern-
						// confirm-falseconfirm.md). The §6 KEYSTONE confirmed a CLIMB re-tag here
						// on the robust BASE ACK pattern when the bsi-bearing SACK suffix decode
						// FAILED. That was a FALSE-CONFIRM: the base ACK tone pattern is config-
						// INVARIANT, so a match proves "an ACK came back," NOT "the RX followed to
						// the announced config." It disarmed the re-tag before the RX actually
						// followed, killing D1 repeat-until-followed + pipeline-the-climb, capping
						// the climb at the ROBUST floor (the tier-cross was never reached; MEASURED
						// rx 15 B vs 424 B with it OFF, WGN:40). inband_retag_confirm_from_base_
						// pattern now ALWAYS returns false (a no-op) — the climb CONFIRM rides the
						// config-DISCRIMINATING suffix bsi (inband_retag_confirm_from_sack, the
						// CRC-valid confirm at :3758/:3783/:3953, untouched) + the D4 auto-demote
						// net. The call is retained so -DINBAND_BASEPATTERN_CONFIRM_FAILBEFORE can
						// reproduce the BUG (the fails-before arm). Flag-off byte-identical.
						if(!decoded && !ack_frame_seen)
							inband_retag_confirm_from_base_pattern(mfsk_matched,
								telecom_system->ack_mfsk.ack_match_threshold);

						// STAGE 4e (D2 NACK first-class, INV-E3 mutual exclusion): the
						// ACK/SACK decode MISSED (decoded==false). Try a NACK decode of the
						// SAME reverse tail (type=MFSK_CTRL_NACK) ONLY now -- try ACK/SACK
						// FIRST so a real ACK/SACK is never stolen. On a CRC-valid NACK whose
						// echoed parity matches the current epoch, the sender ACCELERATES the
						// Stage-4d auto-demote to the RX config (BREAK-count==0), short-
						// circuiting the R-retry wait. No-op when inband off / no NACK present.
#ifndef INBAND_NACK_FAILBEFORE
						if(!decoded && !ack_frame_seen && inband_rate_feature_enabled())
						{
							uint8_t nack_rx_cfg = 0, nack_reason = 0, nack_bsi = 0, nack_parity = 0;
							if(inband_decode_nack_from_capture(&nack_rx_cfg, &nack_reason,
								&nack_bsi, &nack_parity) == 1)
							{
								printf("[CMD-NACK] decoded NACK rx_cfg_idx=%u reason=%u "
									"bsi_lsb=%u parity=%u -- applying accelerated-demote policy\n",
									(unsigned)nack_rx_cfg, (unsigned)nack_reason,
									(unsigned)nack_bsi, (unsigned)nack_parity);
								fflush(stdout);
								if(inband_handle_nack(nack_rx_cfg, nack_reason, nack_parity))
									mfsk_handled_this_poll = true;
							}
						}
#endif

						if(decoded)
						{
								// The shared gate above has already resolved wire to the exact
								// bitmap/clean target and validated its owner, span, and class.
							unsigned cmd_bsi = (unsigned)(cmd_batch_seq_id & 0xFF);
							unsigned prev_bsi = (cmd_bsi - 1u) & 0xFFu;
							// R2c-W1 (data-flow-recoverable-gap-abort.md 5.1): a PARTIAL re-SACK (has a
							// hole) whose bsi is a shadow-retained NON-current batch -- the armed-prev
							// hole re-advertised by the recoverable HOLD, delivered here via the MFSK
							// ACK+SACK suffix (the WB transport rsp_resend_prev_partial_sack() prefers).
							// Re-drive the RETAINED bytes and CONSUME it: skip the by-slot apply (which
							// would FALSE-ACK the current batch) AND the OFDM dispatch. Clean cumulative
							// confirmations (all-ones) are NEVER routed here (they ride the self-heal).
							bool w1_shadow_consumed = false;
							{
								if(!is_clean_confirmation && bitmap_ok && target_owned
								   && target_is_shadow
								   && cmd_prev_resack_is_shadow_target(resolved_target))
								{
									bool sackbm_w1[MAX_SACK_BATCH_SIZE];
									for(int i=0; i<data_batch_size && i<MAX_SACK_BATCH_SIZE; i++)
										sackbm_w1[i] = mfsk_sack_bitmap_bit(rx_bitmap, i);
									int rq = cmd_prev_retain_requeue(resolved_target, sackbm_w1, data_batch_size);
									printf("[CMD-PREV-RETAIN-REQUEUE] (mfsk) wire=%u target=%u not current batch "
										"cmd=%u -- re-queued %d retained frame(s) (retx queue=%d); consuming "
										"SACK (skip by-slot apply; next retransmit re-drives the hole)\n",
										(unsigned)rx_bsi, (unsigned)resolved_target, cmd_bsi, rq, retransmit_count);
									fflush(stdout);
									// R2b — RECOVERY TURNAROUND (data-flow-recoverable-gap-abort.md 5.5). A
									// shadow-retained partial re-SACK with rq>0 is a CREDITED retransmit REQUEST,
									// not a block failure: the peer is provably alive AND re-requesting the exact
									// retained bytes. Give it the SAME turnaround the normal SACK path takes
									// (mirror :5022-5023 + :5093-5095) -- credit the round + reset the silent
									// streak so the ACK-FAILURE bookkeeping (emergency_nack_count /
									// cfg16_revack_starve_fails / the FIX-9-D3 demote / the clear_retx_queue flush
									// of the refill) NEVER runs, and CLOSE the receive window so the end-of-round
									// transition (:6693) sends the requeued refill as a 1-frame pure-retx batch in
									// one guard interval instead of burning the full receiving_timeout as dead
									// air. rq==0 (shadow evicted) -> consume only, no credit: the bounded hold
									// exhausts into today's safe terminal abort. last_batch_fully_acked /
									// last_partial_lead_frame_only are FORCED false so a credited recovery round is
									// never mistaken for a promotable / climb-forward batch (5.5 consumer audit).
									// MERCURY_GAP_RECOVER_TURNAROUND_DEFEAT=1 restores the pre-fix turnaround-less
									// non-event (the fail-before arm).
									bool gap_turn_defeat = false;
									{ const char* e = std::getenv("MERCURY_GAP_RECOVER_TURNAROUND_DEFEAT");
									  if(e && *e && atoi(e)!=0) gap_turn_defeat = true; }
									if(!gap_turn_defeat && rq > 0)
									{
										cmd_rxwindow_note_delivered((int)rx_bsi, /*clean=*/false);
										link_timer.start();
										watchdog_timer.start();
										policy_evaluate_axis3(true);
										mfsk_handled_this_poll = true;
										w1_shadow_consumed = true;
										data_ack_received            = YES;
										consec_pure_silent_rounds    = 0;
										last_batch_fully_acked       = false;
										last_partial_lead_frame_only = false;
										receiving_timeout = (int)receiving_timer.get_elapsed_time_ms()
										                  + ptt_off_delay_ms + 400;
										printf("[CMD-GAP-RECOVERY-TURNAROUND] rq=%d elapsed=%d receiving_timeout=%d "
											"-- credited retransmit request; window closed for pure-retx turnaround "
											"(no demote/nack bookkeeping)\n",
											rq, (int)receiving_timer.get_elapsed_time_ms(), receiving_timeout);
										fflush(stdout);
									}
								}
							}
							bool bsi_in_window = target_owned;
							// Sanity 3: dedupe vs the last SACK we already applied
							// (mirrors the OFDM SACK_RSP duplicate guard at ~2141).
							// climb-engine Bug 1 (gearshift-climb-engine.md §4): SPLIT the dedupe by
								// event class. A CLEAN (all-ones) confirmation SUPERSEDES the
								// partial for the same bsi; deduping it vs the partial tracker
								// (cmd_last_applied_sack_bsi) dropped the RSP prev-delivered clean
								// ACK confirming a retransmit-completed batch -> the delivered
								// rung never promoted. all_ones computed here so the dedupe can
								// branch clean-vs-partial; CLEAN dedupes vs cmd_last_applied_clean_bsi,
								// PARTIAL vs cmd_last_applied_sack_bsi (a repeated clean for the
								// same bsi is still rejected -> no double-count).
									// A2 de-dup is keyed on the resolved target for both clean and
									// partial reports; the raw predecessor never becomes an identity.
								int dedup_bsi = resolved_target;
								bool duplicate = !sack_clean_confirmation_accepted(
									dedup_bsi, is_clean_confirmation,
									cmd_last_applied_clean_bsi, cmd_last_applied_sack_bsi);
									// A partial bitmap is legal only for its resolved current owner.
									// A late prior-generation report cannot apply by slot to a newer batch.
								bool partial_target_ok = is_clean_confirmation
									|| resolved_target == (guard_ref_bsi & 0xFF);
								// MC-4 consumes n_r after CRC + semantic window validation, but
								// before bitmap de-dup: a repeated bitmap may still carry a newer
								// cumulative delivered high-water.
								if(!w1_shadow_consumed && bsi_in_window && bitmap_ok && partial_target_ok)
									cmd_rxwindow_note_delivered((int)rx_bsi, is_clean_confirmation);

							if(!w1_shadow_consumed && bsi_in_window && bitmap_ok && !duplicate && partial_target_ok)
							{
								if(is_clean_confirmation)
								{
									// CLEAN BATCH — mirror OFDM_ACK_CLEAN handler
									// (~line 2178). Only state change there is
									// setting v2_ack_pat_pre_detected = true; the
									// fallthrough ACK_PAT block at ~line 2359
									// owns register_ack(), stats, opt_record_batch,
									// policy_evaluate_axis2, axis3_batch_tick, etc.
									v2_ack_pat_pre_detected = true;
									// climb-engine Bug 1: record this clean batch bsi so a REPEATED clean for
									// the same bsi is deduped (no double-count of nBatches_fully_acked).
									cmd_last_applied_clean_bsi = resolved_target;
									// REPAIR part 2: PRODUCTION EVICT at batch-confirm. A CLEAN
									// confirmation proves the RX has the WHOLE batch rx_bsi, so its
									// retained retx frames can never be a recoverable hole again --
									// evict them here (this was the ONLY missing production caller of
									// cmd_prev_retain_evict(); without it the shadow silted with stale
									// confirmed-batch entries and overflowed within one batch). OFF ->
									// not called (base: shadow silts, the pre-fix defect). Bounded and
									// safe: only drops entries for an ALREADY-delivered bsi.
									if(retain_shadow_fix_on()) cmd_prev_retain_evict(resolved_target);
									// STAGE 4d (D1 CONFIRM): a CLEAN SACK at-or-after the announce bsi proves the
									// RX demodulated a batch sent at the announced config -> DISARM the re-tag
									// (design §1.1/§1.3 consumer 1). No-op when not armed / stale bsi / inband off.
									inband_retag_confirm_from_sack(resolved_target);
									int arrival_ms = (int)receiving_timer.get_elapsed_time_ms();
									// R6: fold this reverse-ACK arrival into the measured-turnaround estimator,
									// Karn-gated (RFC 6298 §3): skip retransmit rounds (ambiguous turnaround).
									// arrival_ms is in the receiving_timer frame = the receiving_timeout deadline
									// frame, so it is a direct RTO sample. See data-flow-turnaround-timers.md.
									if(tt_karn_sample_ok())
										update_turnaround_estimate(current_configuration,
											data_batch_size * message_transmission_time_ms, arrival_ms);
									printf("[CMD-MFSK-ACK-SACK] CLEAN batch_seq_id=%u (cmd_batch_seq_id=%d) "
										"bitmap=0x%08x matched=%d arrival_ms=%d\n",
										(unsigned)resolved_target, cmd_batch_seq_id,
										(unsigned)rx_bitmap, mfsk_matched, arrival_ms);
									fflush(stdout);
									if(arq_turn_trace_on())
										arq_turn_trace_emit_ack("clean-mfsksack", (unsigned)resolved_target, arrival_ms, tt_karn_sample_ok()?1:0);
									mfsk_handled_this_poll = true;
								}
								else
								{
									// PARTIAL BATCH — mirror OFDM SACK_RSP handler
									// (~line 2131). Populate sack_bitmap from the
									// 30-bit bitmap, set sack_detected, record
									// arrival history, dedupe state, axis3 ok event.
									// The big block at ~line 2198 (if(sack_detected))
									// handles retransmit queue / stats / Axis-2
									// state from there.
									for(int i = 0; i < data_batch_size && i < MAX_SACK_BATCH_SIZE; i++)
										sack_bitmap[i] = mfsk_sack_bitmap_bit(rx_bitmap, i);
									sack_detected = true;
									cmd_last_applied_sack_bsi = dedup_bsi;   // Tier-2 stall fix: key on in-flight batch identity
									// ROLLING-PARTIAL unblock (data-flow-inband-frame0-rolling-partial.md
									// §1/§2): record whether THIS partial is a LEAD-FRAME-ONLY loss
									// (exactly bit0 clear, every other batch frame set). all_ones was
									// computed just above (:3748). SIGNATURE: frame-0 (bit0) missing AND
									// at most 2 frames missing total (the lead acquisition-seam frame plus
									// at most the tail/EOB frame the seam can clip — the observed 0x3e and
									// 0x1e rolling partials). A marginal rung drops >2 -> false (§9).
									{
										int n_miss = data_batch_size
											- __builtin_popcount(rx_bitmap
												& mfsk_sack_mask_for_frames(data_batch_size));
										last_partial_lead_frame_only =
											((rx_bitmap & 1u) == 0u) && (n_miss >= 1) && (n_miss <= 2);
									}
									// STAGE 4d (D1 CONFIRM): a PARTIAL SACK still PROVES the RX demodulated the
									// batch at the announced config (it decoded SOME frames of it), so a PARTIAL
									// confirms exactly like a CLEAN (design §1.7 ruling). DISARM the re-tag.
									inband_retag_confirm_from_sack(resolved_target);
									int arrival_ms = (int)receiving_timer.get_elapsed_time_ms();
									// R6: fold this reverse-ACK arrival into the measured-turnaround estimator,
									// Karn-gated (RFC 6298 §3): skip retransmit rounds (ambiguous turnaround).
									// arrival_ms is in the receiving_timer frame = the receiving_timeout deadline
									// frame, so it is a direct RTO sample. See data-flow-turnaround-timers.md.
									if(tt_karn_sample_ok())
										update_turnaround_estimate(current_configuration,
											data_batch_size * message_transmission_time_ms, arrival_ms);
									sack_arrival_history_ms[sack_arrival_history_next_idx] = arrival_ms;
									sack_arrival_history_next_idx =
										(sack_arrival_history_next_idx + 1) % SACK_ARRIVAL_HISTORY;
									if(sack_arrival_history_count < SACK_ARRIVAL_HISTORY)
										sack_arrival_history_count++;
									printf("[CMD-MFSK-ACK-SACK] PARTIAL batch_seq_id=%u (cmd_batch_seq_id=%d) "
										"bitmap=0x%08x matched=%d arrival_ms=%d\n",
										(unsigned)resolved_target, cmd_batch_seq_id,
										(unsigned)rx_bitmap, mfsk_matched, arrival_ms);
									fflush(stdout);
									if(arq_turn_trace_on())
										arq_turn_trace_emit_ack("partial-mfsksack", (unsigned)resolved_target, arrival_ms, tt_karn_sample_ok()?1:0);
									policy_evaluate_axis3(true);
									mfsk_handled_this_poll = true;
								}
							}
							else
							{
								// Decoded suffix bits but they failed sanity —
								// log it and fall through to OFDM so we don't
								// silently drop a real ACK arriving via the
								// OFDM path on the same poll.
								SACK_TRACE("MFSK-ACK-SACK decoded but rejected: "
									"rx_bsi=%u cmd_bsi=%u prev_bsi=%u bitmap=0x%08x "
									"in_window=%d bitmap_ok=%d duplicate=%d target_ok=%d",
									(unsigned)rx_bsi, cmd_bsi, prev_bsi,
									(unsigned)rx_bitmap,
									bsi_in_window ? 1 : 0,
									bitmap_ok ? 1 : 0,
									duplicate ? 1 : 0,
									partial_target_ok ? 1 : 0);
							}
						}
						} // end if(!mfsk_handled_this_poll) — fix-b compact pre-check skips the 13-uncoded decode
					}
#endif // MFSK_ACK_SACK_ENABLED

					if(!mfsk_handled_this_poll)
					{
					// §7.13.30 — pure OFDM dispatch. v2 sessions never send
					// MFSK ACK in the SACK window (RSP uses OFDM_ACK_CLEAN
					// for clean batches and SACK_RSP for partial). CMD only
					// runs OFDM decode; no MFSK detector to false-fire on
					// SACK_RSP body audio, no OFDM Schmidl-Cox to false-fire
					// on Welch-Costas pattern. Both ambiguities eliminated
					// at the source.
					//
					// New-audio throttle: only call receive() when enough
					// new audio has arrived since the last call.
					//   - Normal case: 1 OFDM symbol of advance (~24 ms).
					//   - After receive() returned INCOMPLETE (overflow):
					//     wait for (overflow + 4) symbols of new audio
					//     before retry. Without this, each retry would see
					//     overflow that decreased by only 1 symbol per
					//     poll — burning ~12 fruitless polls of
					//     Schmidl-Cox scans (v15 bug, observed in R1+).
					int rwi = telecom_system->data_container.ring_write_index;
					int sym_samples =
						telecom_system->data_container.Nofdm
						* telecom_system->data_container.interpolation_rate;
					int sp = sym_samples * telecom_system->data_container.buffer_Nsymb;
					int advance = (v2_dispatch_last_rwi >= 0)
						? ((rwi - v2_dispatch_last_rwi + sp) % sp)
						: sym_samples * v2_dispatch_min_advance_syms;  // first poll → process

					int needed_advance =
						sym_samples * v2_dispatch_min_advance_syms;

					if(advance < needed_advance)
					{
						// Not enough new audio yet; skip this poll.
						SACK_TRACE("v2 OFDM dispatch: skipping poll (advance=%d/%d, need=%d syms)",
							advance, needed_advance, v2_dispatch_min_advance_syms);
					}
					else
					{
						v2_dispatch_last_rwi = rwi;
						MUTEX_LOCK(&capture_prep_mutex);
						telecom_system->data_container.frames_to_read = 0;
						MUTEX_UNLOCK(&capture_prep_mutex);
						// §7.13.31 — sack_cross_check_mode REMOVED here. It
						// raised the OFDM detection threshold to 0.65 which
						// was useful when MFSK ACK could false-fire OFDM
						// Schmidl-Cox; in the §7.13.30 pure-OFDM architecture
						// there is no MFSK signal in the SACK window so the
						// strict threshold is unnecessary AND was rejecting
						// legitimate CONTROL frames (e.g., SET_LINK_PARAMS
						// from policy_evaluate_axis2) whose preamble metric
						// is below 0.65. v18 r1 showed CMD sending
						// SET_LINK_PARAMS 10x with 0 ACKs — RSP couldn't
						// decode because sack_cross_check_mode rejected it.
						SACK_TRACE("v2 OFDM dispatch: calling receive() advance=%d need_syms=%d",
							advance, v2_dispatch_min_advance_syms);
						this->receive();
						if(messages_rx_buffer.status == RECEIVED
						   && messages_rx_buffer.type == BLOCK_SACK
						   && !l1_blockack.last_ack_all_received())
						{
							// The dispatcher already settled every positive bit atomically.
							// Missing entries remain journal-owned. This first production
							// version closes the session loudly rather than advancing across
							// an undelivered generation or fabricating retransmission state.
							messages_rx_buffer.status = FREE;
							if(!l1_terminalize_queued("block-ack-partial-terminal"))
								std::abort();
							printf("[L1-BLOCKACK-ARQ] partial aggregate: retained misses "
								"transferred to terminal owner; dropping session loudly\n");
							fflush(stdout);
							link_status = DROPPED;
							reset_session_state();
							reset_all_timers();
							return;
						}
						else if(messages_rx_buffer.status == RECEIVED
						   && messages_rx_buffer.type == BLOCK_SACK
						   && l1_blockack.last_ack_all_received())
						{
							// The real RX dispatcher has already validated and atomically
							// settled every covered transmitted-BSI journal key. Route the
							// current live batch through the existing clean-ACK funnel so
							// ARQ status/stats/timers stay single-sourced.
							v2_ack_pat_pre_detected = true;
							mfsk_handled_this_poll = true;
							messages_rx_buffer.status = FREE;
							printf("[L1-BLOCKACK-ARQ] clean aggregate routed to data_ack_received=YES funnel\n");
							fflush(stdout);
						}

						// After receive(): if INCOMPLETE-overflow fired,
						// the next dispatch must wait for the missing
						// tail to arrive. Reset throttle to 1 sym by
						// default; bump to overflow+4 if we just hit one.
						int overflow_syms =
							telecom_system->receive_stats.frame_overflow_symbols;
						if(overflow_syms > 0)
							v2_dispatch_min_advance_syms = overflow_syms + 4;
						else
							v2_dispatch_min_advance_syms = 1;

						SACK_TRACE("v2 OFDM dispatch done: status=%d type=%d overflow=%d next_advance=%d syms",
							(int)messages_rx_buffer.status,
							(int)messages_rx_buffer.type,
							overflow_syms,
							v2_dispatch_min_advance_syms);
					}

					if(messages_rx_buffer.status == RECEIVED
					   && messages_rx_buffer.type == SACK_RSP)
					{
						// Partial-batch SACK_RSP path — decode the bitmap.
						unsigned char rx_bsi = 0;
						bool reverse_demand = false;
						bool decoded = decode_sack_v2_frame(
							sack_bitmap, data_batch_size, &rx_bsi,
							&reverse_demand);
						int guard_ref_bsi = partial_guard_reference_bsi();
						bool bitmap_any = false;
						bool bitmap_clean = data_batch_size > 0;
						for(int i=0; i<data_batch_size && i<MAX_SACK_BATCH_SIZE; i++)
						{
							bitmap_any = bitmap_any || sack_bitmap[i];
							bitmap_clean = bitmap_clean && sack_bitmap[i];
						}
						st_resolved_generation_ack resolved = {};
						bool target_owned = decoded && resolve_and_validate_generation_ack(
							rx_bsi, data_batch_size, /*integrity_ok=*/true,
							/*clean=*/bitmap_clean,
							bitmap_any, bitmap_clean, &resolved);
						const int resolved_target = resolved.target;
						const bool target_is_shadow = resolved.shadow_owned;
						SACK_TRACE("decode_sack_v2: ok=%d rx_bsi=%u cmd_bsi=%d",
							decoded ? 1 : 0, (unsigned)rx_bsi, cmd_batch_seq_id);
						if(decoded && !target_owned)
						{
							printf("[CMD-SACK-V2-REJECT] wire=%u target=%u owner=%d "
							       "bitmap_any=%d partial=%d span=%d -- no credit\n",
								(unsigned)rx_bsi, (unsigned)resolved_target,
								target_owned ? 1 : 0, bitmap_any ? 1 : 0,
								bitmap_clean ? 0 : 1, data_batch_size);
							fflush(stdout);
							decoded = false;
						}
						if(decoded && std::getenv("MERCURY_GEN_CANON_WITNESS"))
						{
							printf("[GEN-CANON-RESOLVE] transport=OFDM wire=%u target=%u "
							       "canon=%d cumulative=%d clean=%d span=%d owner=%s\n",
								(unsigned)rx_bsi, (unsigned)resolved_target,
								generation_canon_enabled() ? 1 : 0,
								cumulative_ack_enabled ? 1 : 0, bitmap_clean ? 1 : 0,
								data_batch_size,
								target_is_shadow ? "shadow" : "live/journal");
							fflush(stdout);
						}
						if(decoded && target_owned)
							role_demand_note_acknowledged(reverse_demand, "full-sack");
						if(decoded && target_owned && bitmap_clean)
						{
							cmd_rxwindow_note_delivered((int)rx_bsi, /*clean=*/true);
							v2_ack_pat_pre_detected = true;
							cmd_last_applied_clean_bsi = resolved_target;
							printf("[CMD-SACK-V2] CLEAN demand record batch_seq_id=%u demand=%d\n",
								(unsigned)resolved_target, reverse_demand ? 1 : 0);
							fflush(stdout);
							decoded = false; // clean funnel owns settlement; never enter partial apply
						}
						// R2c — RECOVERABLE PREV-BATCH re-SACK (data-flow-recoverable-gap-abort.md
						// §5). If the decoded SACK's bsi is NOT the in-flight batch (the case the
						// OOW/STALE guards below DROP) but IS a retained prev batch still reporting
						// a missing frame, the RSP is HOLDING a downstream batch behind this bsi's
						// hole. Re-queue the RETAINED original bytes for the next mixbatch. This
						// does NOT apply the bitmap to messages_tx[] by index (no false-ACK) and
						// does NOT touch the in-flight ack state — the re-queued frame rides the
						// in-flight batch's next retransmit turnaround. Consume the SACK
						// (decoded=false) so the in-flight guards stay inert.
						bool prev_retain_consumed = false;
						if(decoded && target_is_shadow
						   && cmd_prev_resack_is_shadow_target(resolved_target))
						{
							// R2c-W1: rx_bsi is a shadow-retained NON-current batch (the armed-prev
							// hole re-advertised by the recoverable HOLD, or a batch 2+ behind). Its
							// frames are gone from messages_tx[], so re-drive the RETAINED bytes and
							// CONSUME the SACK -- the by-slot apply below would FALSE-ACK the current
							// batch frames by index (they would never retransmit).
							int rq = cmd_prev_retain_requeue(resolved_target, sack_bitmap, data_batch_size);
							printf("[CMD-PREV-RETAIN-REQUEUE] wire=%u target=%u not current batch cmd=%u -- "
								"re-queued %d retained frame(s) (retx queue=%d); consuming SACK "
								"(skip by-slot apply; next retransmit re-drives the hole)\n",
								(unsigned)rx_bsi, (unsigned)resolved_target,
								(unsigned)(cmd_batch_seq_id & 0xFF), rq, retransmit_count);
							fflush(stdout);
							decoded = false;               // skip the by-slot apply (would false-ACK cmd_bsi)
							// R2b — RECOVERY TURNAROUND (data-flow-recoverable-gap-abort.md 5.5). A
							// shadow-retained partial re-SACK with rq>0 is a CREDITED retransmit REQUEST,
							// not a block failure: the peer is provably alive AND re-requesting the exact
							// retained bytes. Give it the SAME turnaround the normal SACK path takes
							// (mirror :5022-5023 + :5093-5095) -- credit the round + reset the silent
							// streak so the ACK-FAILURE bookkeeping (emergency_nack_count /
							// cfg16_revack_starve_fails / the FIX-9-D3 demote / the clear_retx_queue flush
							// of the refill) NEVER runs, and CLOSE the receive window so the end-of-round
							// transition (:6693) sends the requeued refill as a 1-frame pure-retx batch in
							// one guard interval instead of burning the full receiving_timeout as dead
							// air. rq==0 (shadow evicted) -> consume only, no credit: the bounded hold
							// exhausts into today's safe terminal abort. last_batch_fully_acked /
							// last_partial_lead_frame_only are FORCED false so a credited recovery round is
							// never mistaken for a promotable / climb-forward batch (5.5 consumer audit).
							// MERCURY_GAP_RECOVER_TURNAROUND_DEFEAT=1 restores the pre-fix turnaround-less
							// non-event (the fail-before arm).
							bool gap_turn_defeat = false;
							{ const char* e = std::getenv("MERCURY_GAP_RECOVER_TURNAROUND_DEFEAT");
							  if(e && *e && atoi(e)!=0) gap_turn_defeat = true; }
							if(!gap_turn_defeat && rq > 0)
							{
								cmd_rxwindow_note_delivered((int)rx_bsi, /*clean=*/false);
								link_timer.start();
								watchdog_timer.start();
								policy_evaluate_axis3(true);
								prev_retain_consumed = true;
								data_ack_received            = YES;
								consec_pure_silent_rounds    = 0;
								last_batch_fully_acked       = false;
								last_partial_lead_frame_only = false;
								receiving_timeout = (int)receiving_timer.get_elapsed_time_ms()
								                  + ptt_off_delay_ms + 400;
								printf("[CMD-GAP-RECOVERY-TURNAROUND] rq=%d elapsed=%d receiving_timeout=%d "
									"-- credited retransmit request; window closed for pure-retx turnaround "
									"(no demote/nack bookkeeping)\n",
									rq, (int)receiving_timer.get_elapsed_time_ms(), receiving_timeout);
								fflush(stdout);
							}
						}
						// R039 (race audit 2026-06-06): bsi-in-window guard.
						// decode_sack_v2_frame() is CRC8-only and never validates
						// rx_bsi. The MFSK arms reject rx_bsi outside the
						// {cmd_bsi, prev_bsi} window (arq_commander.cc:2642-2645,
						// :121-126); the OFDM arm previously had only the exact-dup
						// reject below, so a double-checksum (LDPC+CRC8) false-decode
						// of an out-of-window SACK_RSP would be applied by slot index
						// against a messages_tx[] describing a DIFFERENT batch ->
						// silent mis-ACK / needless retransmit. Treat OOW as a CRC
						// fail (fall through to the timeout-driven full-batch
						// retransmit, exactly as if the OFDM frame had been lost).
						// PER-BATCH: rx_bsi must be cmd_bsi or prev_bsi (sack_v2_bsi_in_window).
						// Target ownership and span were validated above, before routing.
						// Stale-partial FALSE-ACK guard (data-flow-inband-dataplane-stall-post-leap.md
						// §8.3): the SACK_RSP bitmap applies BY SLOT INDEX to exactly the batch it
						// describes — the contiguous successor n_r+1. A LATE-decoded straggler from a
						// PRIOR in-flight batch passes the OOW/backward-window check above (its n_r covers
						// prev_bsi) and is NOT a de-dup hit (the current in-flight key is fresh), so its
						// bitmap would FALSE-ACK the current batch's frames by slot index (they never
						// retransmit -> silent loss). Discard iff its resolved target (n_r+1) is NOT the
						// current in-flight batch (cmd_bsi), in either negotiated mode. Same
						// resolved-target guard as the MFSK partial path.
						if(decoded && resolved_target != (guard_ref_bsi & 0xFF))
						{
							printf("[CMD-SACK-V2-STALE] rx_bsi=%u resolves to batch %u != in-flight cmd_bsi=%d "
								"— discarding stale partial (treat as CRC fail)\n",
								(unsigned)rx_bsi, ((unsigned)rx_bsi + 1u) & 0xFFu, cmd_batch_seq_id & 0xFF);
							fflush(stdout);
							decoded = false;
						}
						// MC-4: this is a CRC-valid, in-window PARTIAL n_r. Record it
						// before SACK bitmap de-dup, which is independent of high-water
						// progress. The helper rejects the n_r<0 per-batch fallback.
						if(decoded)
							cmd_rxwindow_note_delivered((int)rx_bsi, /*clean=*/false);
						// OFDM shares the MFSK resolved-target de-dup key.
						int sackv2_dedup_bsi = resolved_target;
						if(decoded
						   && sackv2_dedup_bsi == cmd_last_applied_sack_bsi)
						{
							printf("[CMD-SACK-V2-DUPLICATE] bsi=%u already applied — ignoring\n",
								(unsigned)rx_bsi);
							fflush(stdout);
							decoded = false;
						}
						if(decoded)
						{
							sack_detected = true;
							cmd_last_applied_sack_bsi = sackv2_dedup_bsi;   // Tier-2 stall fix: shared key
							// STAGE 4d (D1 CONFIRM): an OFDM SACK_RSP at-or-after the announce bsi
							// proves the RX demodulated the batch at the announced config -> DISARM
							// the re-tag (design §1.1/§1.3 consumer 1). No-op when not armed / inband off.
							inband_retag_confirm_from_sack(resolved_target);
							int arrival_ms = (int)receiving_timer.get_elapsed_time_ms();
							// R6: fold this reverse-ACK arrival into the measured-turnaround estimator,
							// Karn-gated (RFC 6298 §3): skip retransmit rounds (ambiguous turnaround).
							// arrival_ms is in the receiving_timer frame = the receiving_timeout deadline
							// frame, so it is a direct RTO sample. See data-flow-turnaround-timers.md.
							if(tt_karn_sample_ok())
								update_turnaround_estimate(current_configuration,
									data_batch_size * message_transmission_time_ms, arrival_ms);
							sack_arrival_history_ms[sack_arrival_history_next_idx] = arrival_ms;
							sack_arrival_history_next_idx =
								(sack_arrival_history_next_idx + 1) % SACK_ARRIVAL_HISTORY;
							if(sack_arrival_history_count < SACK_ARRIVAL_HISTORY)
								sack_arrival_history_count++;
							printf("[CMD-SACK-V2] decoded SACK_RSP batch_seq_id=%u (cmd_batch_seq_id=%d) arrival_ms=%d — applying to retransmit queue\n",
								(unsigned)resolved_target, cmd_batch_seq_id, arrival_ms);
							fflush(stdout);
							if(arq_turn_trace_on())
								arq_turn_trace_emit_ack("sackv2", (unsigned)resolved_target, arrival_ms, tt_karn_sample_ok()?1:0);
							policy_evaluate_axis3(true);
						}
						messages_rx_buffer.status = FREE;
					}
					// OFDM_ACK_CLEAN dispatch removed 2026-05-24. Clean-batch
					// ACKs now arrive via the MFSK ACK+SACK pattern handled
					// above; no OFDM frame ever bears type==OFDM_ACK_CLEAN.
					} // end if(!mfsk_handled_this_poll) — Step 6 MFSK probe gate
				}
				// Step 15: legacy MFSK SACK receive path deleted. When
				// !sack_v2_enabled we simply don't attempt a partial-batch
				// SACK decode — sack_detected stays false and CMD falls
				// through to the timeout-driven full-batch retransmit path.
			}

			if(sack_detected)
			{
				restore_sack_v2_rx_phy();
				printf("[CMD-SACK] Partial batch ACK detected!\n");
				printf("[CMD-SACK-DIAG] messages_tx status:");
				for(int d = 0; d < data_batch_size; d++)
					printf(" %d:%d", d, (int)messages_tx[d].status);
				printf("\n");
				fflush(stdout);
				clear_buffer(playback_buffer);
				link_timer.start();
				watchdog_timer.start();
				gear_shift_timer.stop();
				gear_shift_timer.reset();

				// Build retransmit queue: save missing frames' encrypted payloads.
				//
				// §7.13.39 Fix 1 — APPEND mode (v2 only). Do NOT reset retransmit_count
				// here on v2 sessions: any survivors carried over from the previous
				// batch's overflow shift-down (see §7.13.39 Fix 1 at the v2 mixbatch
				// builder, ~line 1027) live at indices [0..retransmit_count-1] and
				// MUST be preserved. The new SACK_RSP can only refer to frames in the
				// just-sent batch (whose messages_tx[i] are still PENDING_ACK), which
				// have no overlap with the older survivors (those were marked ACKED
				// at their original capture). v1 path is untouched (sack_v2_enabled=false
				// still resets — the v1 standalone-retx path consumes the queue fully
				// on every cycle so there is never anything to preserve).
				int leftover_retx_at_capture = sack_v2_enabled ? retransmit_count : 0;
				if(!sack_v2_enabled) retransmit_count = 0;
				int rx_count = 0;
				// Phase 3a — sum payload bytes of frames the receiver actually
				// got (sack_bitmap[i] == true). Bytes saved into the retransmit
				// queue are NOT delivered yet — exclude them.
				unsigned long long opt_sack_bytes_delivered = 0;
				std::vector<int> sack_positive_ids;
				std::vector<bool> sack_just_acked(nMessages, false);
				for(int i = 0; i < nMessages; i++)
					if((messages_tx[i].status == PENDING_ACK ||
					    messages_tx[i].status == ACK_TIMED_OUT) &&
					   i < data_batch_size && sack_bitmap[i])
					{
						sack_positive_ids.push_back(i);
						sack_just_acked[i] = true;
					}
				// Legacy SACK handling accepted ACK_TIMED_OUT frames even with the
				// journal disabled; preserve that default-path transition exactly.
				if(!register_acks(sack_positive_ids, true)) return;
				for(int i = 0; i < nMessages; i++)
				{
					if(sack_just_acked[i])
					{
						opt_sack_bytes_delivered += (unsigned)messages_tx[i].length;
						rx_count++;
						continue;
					}
					if(messages_tx[i].status != PENDING_ACK && messages_tx[i].status != ACK_TIMED_OUT)
						continue;

					if(retransmit_count < MAX_RETRANSMIT_HEADROOM)
					{
						// Frame missing — save encrypted payload for retransmit.
						//
						// PADDED-SLOT GUARD: pad_messages_batch_tx() at
						// arq_common.cc:2282 sets messages_batch_tx[slot].id = slot
						// for slots beyond ToSend_data (filling the batch out to
						// data_batch_size with duplicates of earlier frames).
						// send_batch() then flips messages_tx[slot].status =
						// PENDING_ACK on those padded ids — but the corresponding
						// messages_tx slot was never written by add_message_tx_data,
						// so it holds init values (length=0, type=NONE, bsi=-1).
						// If SACK_RSP reports the padded slot missing, enqueuing
				                        // it produces a retx with bsi=-1 and zero-length payload,
						// which makes frame 0 of the next mixbatch arrive at RSP
						// as type=0/len=0/empty — RSP can't decompress; PPMd context
						// resets; all subsequent batches cascade-fail. The padded
						// frame's wire bytes are already present on the wire as a
						// duplicate of an earlier (real) slot; if that earlier slot
						// was received, the padded "loss" is meaningless. Mark
						// ACKED and skip enqueue.
						if(messages_tx[i].length == 0
						   || messages_tx[i].batch_seq_id < 0
						   || messages_tx[i].type == NONE)
						{
							messages_tx[i].status = ACKED;
							stats.nAcked_data++;
							continue;
						}
						int len = messages_tx[i].length;
						if(len > MAX_SACK_FRAME_SIZE) len = MAX_SACK_FRAME_SIZE;
						memcpy(retransmit_frames[retransmit_count], messages_tx[i].data, len);
						retransmit_frame_lengths[retransmit_count] = len;
						retransmit_frame_positions[retransmit_count] = i;
						retransmit_frame_types[retransmit_count] = messages_tx[i].type;
						// SACK Design A Step 3 — capture the ORIGINAL batch_seq_id this
						// frame was first sent under. The retransmit-only batch (built
						// at arq_commander.cc:727+) will resurrect this value rather
						// than use the current cmd_batch_seq_id (which now points at
						// the NEXT new-data batch).
						retransmit_frame_batch_seq_ids[retransmit_count] = messages_tx[i].batch_seq_id;
						// §7.13.39 Fix 3 — preserve the original sequence_number byte
						// (low 7 bits = slot, bit 7 = EOB). If this frame was the last
						// in its original batch, EOB must survive the retx so the RSP's
						// match-prev path receives an end-of-batch marker on the retx
						// slot. Without this, retransmitting the original last frame
						// resurrects only the slot position and never the EOB bit.
						retransmit_frame_seq_with_eob[retransmit_count] =
							(unsigned char)messages_tx[i].sequence_number;
						retransmit_count++;
						// Mark ACKED so cleanup() frees the slot (payload saved above)
						messages_tx[i].status = ACKED;
						stats.nAcked_data++;
					}
				}
				if(sack_v2_enabled && leftover_retx_at_capture > 0)
				{
					printf("[CMD-V2-MIXBATCH-RETX] SACK_RSP capture: appended %d new "
						"to %d leftover; total queue=%d\n",
						retransmit_count - leftover_retx_at_capture,
						leftover_retx_at_capture, retransmit_count);
					fflush(stdout);
				}

				printf("[CMD-SACK] %d/%d received, %d queued for retransmit\n",
					rx_count, data_batch_size, retransmit_count);
				fflush(stdout);

				SACK_TRACE("dar=YES via SACK_RSP path rx_count=%d batch=%d retx=%d",
					rx_count, data_batch_size, retransmit_count);
				data_ack_received = YES;
				consec_pure_silent_rounds = 0;   // R5: a credited data-ACK is a live peer -- reset the silent streak
				// CLEAN-BATCH VIABILITY (§9): a PARTIAL SACK keeps the link alive and
				// drives retransmit of the missing frames (above), but the batch was
				// NOT fully delivered — it must NOT promote the rung. Leave the
				// promotion-gating flag FALSE (explicit; the per-batch TX-start reset
				// already cleared it) and do NOT bump nBatches_fully_acked. nBatches_acked
				// is still bumped below for its existing (stats) meaning — unchanged.
				// ROLLING-PARTIAL unblock (data-flow-inband-frame0-rolling-partial.md
				// §1/§2): record whether THIS OFDM SACK_RSP partial is a LEAD-FRAME-ONLY
				// loss (frame-0 missing, every other batch frame present). Computed from
				// sack_bitmap[] (filled by decode_sack_rsp above). Mirrors the MFSK partial
				// site so BOTH partial transports feed the same FRAME-UP unblock predicate.
				{
					// frame-0 missing AND at most 2 frames missing total (lead acquisition-seam
					// frame + at most the tail). Mirrors the MFSK-site signature.
					int n_miss = 0;
					for(int i = 0; i < data_batch_size && i < MAX_SACK_BATCH_SIZE; i++)
						if(!sack_bitmap[i]) n_miss++;
					last_partial_lead_frame_only =
						(data_batch_size > 1) && !sack_bitmap[0] && (n_miss >= 1) && (n_miss <= 2);
				}
				last_batch_fully_acked = false;
				stats.nBatches_acked++;
				last_transmission_block_stats.nBatches_acked++;

				// SACK Design A Step 10 — Axis 2 evaluation on SACK_RSP receipt.
				// partial_rate = (batch - rx_count) / batch derived from the
				// SACK_RSP bitmap (= what fraction of frames the receiver
				// reported missing). Gated on sack_v2_enabled — v1 SACK paths
				// don't feed this controller.
				if(sack_v2_enabled)
				{
					policy_evaluate_axis2(rx_count, data_batch_size);
					// SACK Design A Step 11 — per-batch tick (Axis-3).
					// Drains Axis-1 cooldown for Axis-3 and increments the
					// OFF-state batches-counter for the 20-batch re-probe.
					// Axis-3 ok event itself was already fired in the SACK
					// decode-success branch above.
					axis3_batch_tick();
				}

				// Phase 3a — record this batch as a SACK-recovered partial.
				// sack_used=true marks "had to spend a SACK_RSP cycle to close
				// the batch"; bytes_delivered = sum of bytes the receiver
				// actually got (the bitmap-true frames). Failed=false.
				opt_record_batch((unsigned int)opt_sack_bytes_delivered,
				                 /*sack_used=*/true,
				                 /*failed=*/false,
				                 /*frames_acked=*/rx_count);
				// Phase 3c — ask the optimizer whether to switch. Defers the
				// queued SET_CONFIG to the next TRANSMITTING_DATA tick (clean
				// dispatch site) instead of mid-receive to avoid stepping on
				// the Axis-2/3 evaluations below.
				{
					int rec = current_configuration;
					if (opt_evaluate_batch_end(&rec))
						opt_pending_switch_cfg = rec;
				}

				if(messages_control.data[0]==REPEAT_LAST_ACK &&
				   (messages_control.status==PENDING_ACK || messages_control.status==ACK_TIMED_OUT))
				{
					this->messages_control.ack_timeout=0;
					this->messages_control.id=0;
					this->messages_control.length=0;
					this->messages_control.nResends=0;
					this->messages_control.status=FREE;
					this->messages_control.type=NONE;
					stats.nAcked_control++;
				}

				// Guard delay (SACK is longer than ACK — extra margin)
				int guard = ptt_off_delay_ms + 400;
				receiving_timeout = (int)receiving_timer.get_elapsed_time_ms() + guard;
			}
			// ACK detection: check when SACK not detected (or before SACK window).
			// Bug A fix (§7.13.1): on a v2-enabled session we may have ALREADY
			// detected the legacy ACK pattern in the v2 dispatch above (the
			// pre-detect happens before this->receive() to avoid destroying
			// the ACK audio with a doomed OFDM demod). Short-circuit with the
			// flag so we don't re-call receive_ack_pattern() (which would now
			// return false due to frames_to_read=4 throttle set by the
			// successful first call).
			// §7.13.29 — fallthrough MFSK ACK handler. Accept if either:
			// (a) the strict MFSK-first probe (threshold 12) already
			//     confirmed this poll, recorded via v2_ack_pat_pre_detected
			//     (CRC-gated: set at ~line 2441 only after a CRC12-valid,
			//     in-window, clean-batch suffix decode);
			// (b) we're OUTSIDE the SACK window AND receive_ack_pattern()
			//     matched the bare MFSK pattern.
			// Inside the SACK window the lenient threshold is DELIBERATELY
			// avoided: it false-fires on OFDM SACK_RSP body audio (the
			// silent-drop bug). If strict missed but lenient would catch,
			// we let timeout-driven retransmit handle it — better a retx
			// than a silent drop.
			//
			// PHANTOM-ACK CONTENT GATE (2026-05-29). receive_ack_pattern()
			// returns a BARE bool on pattern-match-only (matched/metric, NO
			// CRC, NO content; arq_common.cc:5556). At deep SNR a structured-
			// noise / rx-tail self-match faked a clean data ACK through this
			// arm (matched=7, metric=0.66, on CONFIG_0, with NO RSP TX), set
			// data_ack_received=YES, raised last_data_viable_config to CONFIG_0
			// and reset breaks_since_last_data_success — defeating BREAK
			// recovery (cascade_diag_wgn-10/...cmd.log:11507). The clean
			// discriminator is CONTENT: a real WB data ACK carries a CRC12-valid
			// MFSK suffix; the phantom does not. data_ack_bare_pattern_acceptable()
			// requires a CRC-valid clean-batch suffix on CRC-capable (WB) sessions
			// and lets NB / suffix-incapable sessions (RSP sends a bare ACK —
			// arq_responder.cc:1679-1683) through unchanged. An earlier ENERGY
			// floor (commit aecb561) was reverted as non-viable (real/phantom
			// energy overlap); this is a CONTENT gate. The control-ACK detector
			// (arq_commander.cc:1702) and the emergency-BREAK poll
			// (arq_commander.cc:92) are NOT touched — control frames carry no
			// suffix and keep bare-pattern behavior. See
			// fact-documents/gearshift-start-and-recovery.md §2 Bug 3 + §8.
			// NOTE on evaluation order: the content gate is the
			// short-circuiting inline form of data_ack_bare_pattern_acceptable()
			// (the PURE policy predicate the unit test drives):
			//   suffix_capable ? crc_valid : true  ≡  !suffix_capable || crc_valid.
			// Written inline with && / || so the DSP peek cmd_clean_data_ack_crc_valid()
			// runs ONLY on WB (suffix-capable) AND only after a bare pattern match
			// — NB never pays the FFT cost and its bare-pattern acceptance is byte-
			// for-byte unchanged.
			// Option B (data-flow-compact-confirm.md §5): try the COMPACT confirm
			// FIRST (cheaper/shorter/deeper) then the 13-uncoded clean-data ACK. The
			// two CRC12s are over different fields so neither cross-validates the
			// other (the false-confirm invariant; test_compact_confirm_no_cross_validate).
			// Gated OFF until the faithful real-audio re-verify (the predicate compiles
			// to the unchanged condition when ARQ_COMPACT_CONFIRM_ENABLE==0).
			// SHARED-PREDICATE FIX (data-flow-compact-confirm.md): the bare/compact data-ACK
			// arm is now cmd_compact_confirm_live_accept() - it tries the SELF-VALIDATING
			// compact confirm FIRST, DECOUPLED from receive_ack_pattern()'s CRC-less bare
			// 7/16 presence gate (whose 8-symbol energy pre-gate misses the confirm when it
			// the compact CRC decode behind it missed every confirm), then falls back to the
			// legacy bare-pattern + WB CRC content gate (UNCHANGED). compact_enabled =
			// ARQ_COMPACT_CONFIRM_ENABLE (held off) => byte-identical to the legacy bare arm
			// when 0. The v2 pre-detect short-circuit is preserved (evaluated first, no side
			// effects). The two CRC12s are over different fields so neither cross-validates
			// the other (false-confirm invariant; test_compact_confirm_live_rx_path).
			else if(data_ack_received==NO
			        && (v2_ack_pat_pre_detected
			            || cmd_compact_confirm_live_accept(sack_window_open,
			                                               ARQ_COMPACT_CONFIRM_ENABLE != 0)
			            // LINK-PHASE STEP 3 (sub-class a): additive sample-anchored older-phase
			            // rescan. Reached ONLY when the incumbent newest-tail accept above failed;
			            // returns false (byte-identical) unless MERCURY_LINKPHASE_ACKWIN is set and
			            // the strong-guarded reverse-ACK window is open. WIDENS acceptance only.
			            || cmd_data_ack_anchored_rescan_accept()))
			{
				printf("[CMD-ACK-PAT] Data ACK pattern detected!\n");
				fflush(stdout);
				mtl::log_event_kv("cmd_ack_detected", "batch=%lld", stats.nBatches_sent);
				clear_buffer(playback_buffer);
				link_timer.start();
				watchdog_timer.start();
				gear_shift_timer.stop();
				gear_shift_timer.reset();
				SACK_TRACE("dar=YES via MFSK ACK_PAT path pre_detected=%d",
					v2_ack_pat_pre_detected ? 1 : 0);
				data_ack_received=YES;
				restore_sack_v2_rx_phy();
				consec_pure_silent_rounds = 0;   // R5: a credited data-ACK is a live peer -- reset the silent streak
				// CLEAN-BATCH VIABILITY (§9): this is the CLEAN (all-ones) ACK funnel
				// — the MFSK all-ones suffix (v2_ack_pat_pre_detected, set at the
				// rx_bitmap==all_ones branch ~:2521) and the bare-pattern data-ACK arm
				// (WB CRC-gated by §8) both land here. The whole batch is delivered,
				// so this batch MAY drive the four gearshift promotion consumers.
				last_batch_fully_acked = true;
				last_partial_lead_frame_only = false;  // ROLLING-PARTIAL: clean batch is not a lead-only partial
				stats.nBatches_acked++;
				stats.nBatches_fully_acked++;
				last_transmission_block_stats.nBatches_acked++;
				last_transmission_block_stats.nBatches_fully_acked++;

				// Phase 3a — clean full-batch ACK: every PENDING_ACK / ACK_TIMED_OUT
				// frame is about to be delivered. Sum lengths BEFORE register_ack
				// mutates statuses.
				unsigned long long opt_clean_bytes_delivered = 0;
				std::vector<int> clean_ack_ids;
				for(int i=0; i<nMessages; i++)
				{
					if(messages_tx[i].status==PENDING_ACK || messages_tx[i].status==ACK_TIMED_OUT)
					{
						opt_clean_bytes_delivered += (unsigned)messages_tx[i].length;
						clean_ack_ids.push_back(i);
					}
				}
				register_acks(clean_ack_ids);
				// Record as clean (sack_used=false, failed=false). v2_ack_pat_pre_detected
				// = 1 means we got here via the MFSK-suffix clean ACK; either way the
				// batch closed with NO SACK_RSP cycle, so this is the "clean" bucket.
				// (Historical: an OFDM_ACK_CLEAN frame type carried this signal before
				// 2026-05-24; deleted in the mfsk-robust-ack ship — MFSK suffix now
				// handles all clean-batch acknowledgements.)
				opt_record_batch((unsigned int)opt_clean_bytes_delivered,
				                 /*sack_used=*/false,
				                 /*failed=*/false);
				// Phase 3c — same deferred-switch protocol as the SACK path.
				{
					int rec = current_configuration;
					if (opt_evaluate_batch_end(&rec))
						opt_pending_switch_cfg = rec;
				}

				// SACK Design A Step 10 — Axis 2 evaluation on clean full-batch ACK.
				// A full-batch ACK (no SACK_RSP) means receiver got all frames →
				// partial_rate = 0. Feeds a "good" observation into the ring +
				// bumps the consecutive-good counter. Gated on sack_v2_enabled
				// so v1 sessions don't accumulate Axis-2 state.
				if(sack_v2_enabled)
				{
					policy_evaluate_axis2(data_batch_size, data_batch_size);
					// SACK Design A Step 11 — per-batch tick (Axis-3).
					// Clean full-ACK is NOT a SACK event (no partial batch → RSP
					// never tried to send SACK_RSP). Only run the batch tick so
					// the OFF-state re-probe timer and cooldown drain advance.
					axis3_batch_tick();
				}
				// Phase D timing — mark end of CMD's post-ACK bookkeeping (clean
				// batch path). The gap between this and cmd_batch_tx_start is
				// "prep + PTT-on + audio buffer fill" — i.e., what the user
				// identified as the 407ms prep gap to drill into.
				// [L1-GEARFEED] feed the gearshift the N-1 silent intermediate batches
				// of this aggregate (ladder clean-rate + Axis-2/3 + optimizer ring).
				l1_gearfeed_replay_intermediate();
				mtl::log_event("cmd_ack_post_work_done");

				if(messages_control.data[0]==REPEAT_LAST_ACK &&
				   (messages_control.status==PENDING_ACK || messages_control.status==ACK_TIMED_OUT))
				{
					this->messages_control.ack_timeout=0;
					this->messages_control.id=0;
					this->messages_control.length=0;
					this->messages_control.nResends=0;
					this->messages_control.status=FREE;
					this->messages_control.type=NONE;
					stats.nAcked_control++;
				}

				// Guard delay: wait for the radio TX→RX transition to settle.
				// v9.2: dropped the +200ms software margin. Clean-batch ACK is
				// now carried by the MFSK ACK suffix (post-2026-05-24 ship; the
				// historical OFDM_ACK_CLEAN frame type was deleted entirely),
				// detected the moment its trailing sample reaches the demod —
				// the only physical wait we still need is ptt_off_delay_ms
				// (default 200ms) for PA tail / RX-mute release. Combined with
				// the move to a pure MFSK suffix, this saves roughly 800ms +
				// 200ms = ~1s of dead-air per clean batch at cfg=15 WB.
				int guard = ptt_off_delay_ms;
				receiving_timeout = (int)receiving_timer.get_elapsed_time_ms() + guard;
			}
		}
		else
		{
			// Fallback: decode LDPC ACK frame
			this->receive();
			if(messages_rx_buffer.status==RECEIVED)
			{
				// Flush old batch audio from playback buffer so responder
				// doesn't demodulate stale frames before the new batch.
				clear_buffer(playback_buffer);
				link_timer.start();
				watchdog_timer.start();
				gear_shift_timer.stop();
				gear_shift_timer.reset();
				// Phase 3a — accumulate delivered bytes across whichever fallback
				// LDPC ACK type fires. Both ACK_RANGE and ACK_MULTI are clean
				// (no SACK_RSP cycle), so sack_used=false.
				unsigned long long opt_ldpc_bytes_delivered = 0;
				bool opt_ldpc_ack_fired = false;
				if(messages_rx_buffer.type==BLOCK_SACK && l1_blockack.enabled())
				{
					if(!l1_blockack.last_ack_all_received())
					{
						messages_rx_buffer.status=FREE;
						if(!l1_terminalize_queued("block-ack-partial-terminal"))
							std::abort();
						printf("[L1-BLOCKACK-ARQ] partial aggregate on LDPC fallback; "
							"terminal owner accepted misses\n");
						fflush(stdout);
						link_status=DROPPED;
						reset_session_state();
						reset_all_timers();
						return;
					}
					data_ack_received=YES;
					last_batch_fully_acked=true;
					std::vector<int> block_ack_ids;
					for(int i=0;i<nMessages;i++)
						if(messages_tx[i].status==PENDING_ACK
						   || messages_tx[i].status==ACK_TIMED_OUT)
						{
							opt_ldpc_bytes_delivered +=
								(unsigned)messages_tx[i].length;
							block_ack_ids.push_back(i);
						}
					register_acks(block_ack_ids, true);
					stats.nBatches_acked += (long long)l1_blockack.last_ack_batch_count();
					stats.nBatches_fully_acked +=
						(long long)l1_blockack.last_ack_batch_count();
					opt_ldpc_ack_fired=true;
					printf("[L1-BLOCKACK-ARQ] clean aggregate accepted on LDPC fallback\n");
					fflush(stdout);
				}
				else if(messages_rx_buffer.type==ACK_RANGE)
				{
					data_ack_received=YES;
					consec_pure_silent_rounds = 0;   // R5: a credited data-ACK is a live peer -- reset the silent streak
					// CLEAN-BATCH VIABILITY (§9): LDPC full-batch range ACK — clean
					// (no SACK cycle), so this batch may drive promotion.
					last_batch_fully_acked = true;
					last_partial_lead_frame_only = false;  // ROLLING-PARTIAL: clean batch
					stats.nBatches_acked++;
					stats.nBatches_fully_acked++;
					last_transmission_block_stats.nBatches_acked++;
					last_transmission_block_stats.nBatches_fully_acked++;
					int start=(unsigned char)messages_rx_buffer.data[0];
					int end=(unsigned char)messages_rx_buffer.data[1];
					// Guard: start > end under garbage frames wraps unsigned char → infinite loop
					if(start <= end)
					{
						std::vector<int> range_ack_ids;
						for(int i=start;i<=end;i++)
						{
							if(i >= 0 && i < nMessages
							   && messages_tx[i].status == PENDING_ACK)
								opt_ldpc_bytes_delivered += (unsigned)messages_tx[i].length;
							range_ack_ids.push_back(i);
						}
						register_acks(range_ack_ids);
					}
					opt_ldpc_ack_fired = true;
				}
				else if(messages_rx_buffer.type==ACK_MULTI)
				{
					data_ack_received=YES;
					consec_pure_silent_rounds = 0;   // R5: a credited data-ACK is a live peer -- reset the silent streak
					// CLEAN-BATCH VIABILITY (§9): LDPC full-batch multi ACK — clean
					// (no SACK cycle), so this batch may drive promotion.
					last_batch_fully_acked = true;
					last_partial_lead_frame_only = false;  // ROLLING-PARTIAL: clean batch
					stats.nBatches_acked++;
					stats.nBatches_fully_acked++;
					last_transmission_block_stats.nBatches_acked++;
					last_transmission_block_stats.nBatches_fully_acked++;
					// Clamp count to buffer bounds — garbage frames can have data[0]=255,
					// reading past the 200-byte messages_rx_buffer.data (Bug #15)
					int ack_count = (unsigned char)messages_rx_buffer.data[0];
					int max_acks = max_data_length + max_header_length - ACK_MULTI_ACK_RANGE_HEADER_LENGTH - 1;
					if(max_acks < 0) max_acks = 0;
					if(ack_count > max_acks) ack_count = max_acks;
					std::vector<int> multi_ack_ids;
					for(int i=0;i<ack_count;i++)
					{
						int msg_id = (unsigned char)messages_rx_buffer.data[i+1];
						if(msg_id >= 0 && msg_id < nMessages
						   && messages_tx[msg_id].status == PENDING_ACK)
							opt_ldpc_bytes_delivered += (unsigned)messages_tx[msg_id].length;
						multi_ack_ids.push_back(msg_id);
					}
					register_acks(multi_ack_ids);
					opt_ldpc_ack_fired = true;
				}
				messages_rx_buffer.status=FREE;

				if(opt_ldpc_ack_fired)
				{
					opt_record_batch((unsigned int)opt_ldpc_bytes_delivered,
					                 /*sack_used=*/false,
					                 /*failed=*/false);
					// Phase 3c — same deferred-switch protocol as the SACK +
					// ACK_PAT paths above. Defers any optimizer-recommended
					// SET_CONFIG to the next TRANSMITTING_DATA dispatch.
					{
						int rec = current_configuration;
						if (opt_evaluate_batch_end(&rec))
							opt_pending_switch_cfg = rec;
					}
				}

				if(messages_control.data[0]==REPEAT_LAST_ACK && messages_control.status==PENDING_ACK)
				{
					this->messages_control.ack_timeout=0;
					this->messages_control.id=0;
					this->messages_control.length=0;
					this->messages_control.nResends=0;
					this->messages_control.status=FREE;
					this->messages_control.type=NONE;
					stats.nAcked_control++;
				}
			}
		}
	}
	else if (data_ack_received==NO && ack_pattern_time_ms <= 0
		&& !(linkphase_slotliveness_on() && linkphase_ack_slot_wait_armed)
		&& !(last_message_sent_type==CONTROL && last_message_sent_code==REPEAT_LAST_ACK))
	{
		consecutive_data_acks = 0;  // Reset on failure
		// SUSTAINED-ANCHOR GATE (gearshift-climb-engine.md §11): a failed block
		// breaks the consecutive-clean run at this rung — the next clean must
		// re-accumulate from 1 before it can raise the anchor. (The BREAK trigger
		// at :3390 is downstream of this and the :3276 sibling, so the demotion
		// path also sees a reset clean-streak.)
		clean_batches_at_current_config = 0;
		// R5: count this failed round for the pure-silence classifier. This branch guarantees
		// ack_pattern_time_ms<=0, so the classifier splits SILENT vs a sub-threshold LATE_ACK via
		// ack_diag_peak_matched. Read-only on existing state + one counter; byte-identical on its own.
		ackfail_classifier_step(ack_pattern_time_ms, ack_diag_peak_metric, ack_diag_peak_matched, /*sack_this_round=*/false);

		// Frame gearshift just applied but data failed — BREAK immediately, no retry
		if(frame_gearshift_just_applied)
		{
			frame_gearshift_just_applied = false;
			frame_gearshift_retry_count = 0;
			// FIX-B — ARM the floor-probe back-off on the FAILED up-probe rung
			// (gearshift-floor-probe-backoff.md §5.1, arm-site #2). NACK data-fail:
			// the just-applied FRAME-UP to data_configuration could not pass DATA.
			// Arm BEFORE the working_config overwrite below clobbers
			// data_configuration, so the back-off is keyed to the rung that failed.
			probe_backoff_arm(data_configuration);
			int working_config = config_ladder_down(data_configuration, robust_enabled);
			frame_shift_threshold *= 2;

			// STAGE 4c — Class-A site A2 (FRAME-UP data fail, nack). Under inband, the
			// failed up-probe DEMOTES one rung via the CONFIG_TAG instead of BREAKing. The
			// gearshift bookkeeping above (frame_gearshift reset + probe_backoff_arm +
			// frame_shift_threshold) already ran (it must — the probe DID fail). If
			// working_config is a real lower rung, tag-demote; if it equals data_configuration
			// (already at the bottom), route to the true-loss floor. Byte-identical OFF.
			if(inband_rate_feature_enabled())
			{
				if(working_config != data_configuration
				   && inband_route_failure_demote(working_config, "frame_gearshift_data_failed_nack"))
					return;
				// At the bottom (no lower rung): tick the dead-batch floor; BREAK only at N.
				if(inband_cmd_dead_batch_floor_reached())
				{
					emergency_previous_config = current_configuration;
					emergency_break_active = 1;
					emergency_break_retries = 1;
					emergency_nack_count = 0;
					cfg16_revack_starve_fails = 0;
					data_ack_retx_turnaround = false;
					data_ack_round_carried_retx = false;   // Karn: fresh epoch, next send re-arms
					if(sack_v2_enabled)
						policy_axis1_supremacy_on_move(current_configuration,
							working_config, "inband_session_dead_batches_floor");
					send_break_pattern();
					telecom_system->data_container.frames_to_read = 4;
					calculate_receiving_timeout();
					receiving_timer.start();
				}
				// Below the floor (or demote took it): do not run the legacy BREAK.
				return;
			}

			// This config failed during data — cap future SUPERSHIFT attempts
			if(supershift_proven_ceiling < 0 || working_config < supershift_proven_ceiling)
				supershift_proven_ceiling = working_config;

			printf("[GEARSHIFT] FRAME UP DATA FAILED: config %d can't pass data, BREAK to %d (threshold now %d, ceiling=%d)\n",
				data_configuration, working_config, frame_shift_threshold, supershift_proven_ceiling);
			fflush(stdout);

			// Preserve all pending data for resend at working config
			if(compression_enabled)
			{
				restore_tx_from_compressed();
			}
			else
			{
				restage_requeue_tx_messages();   // §12 lossless+ordered re-stage (was forward push()-to-BACK)
				fifo_buffer_backup.flush();
				clear_retx_queue();  // R029: recovery (non-compressed) re-queues plaintext; drop stale retx
			}
			block_under_tx = NO;

			int fifo_load = fifo_buffer_tx.get_size() - fifo_buffer_tx.get_free_size();
			printf("[GEARSHIFT] Saved data to FIFO: %d bytes pending\n", fifo_load);
			fflush(stdout);

			data_configuration = working_config;
			negotiated_configuration = working_config;

			emergency_previous_config = working_config;
			break_drop_step = 0;
			emergency_break_active = 1;
			emergency_break_retries = 1;
			emergency_nack_count = 0;
			cfg16_revack_starve_fails = 0;  // WALL-B FIX-9 D3 (P4): demote off CFG16 -> streak fresh
			// WALL-B FIX-9 D2 REFINE (§4 Q3 INV-6): BREAK starts a fresh epoch -> clear the
			// retx-turnaround arm so a stale TRUE cannot widen the post-BREAK control window. The
			// next data batch re-derives it at the send path.
			data_ack_retx_turnaround = false;
			data_ack_round_carried_retx = false;   // Karn: fresh epoch, next send re-arms

			// SACK Design A Step 12 — BREAK supremacy (§4.3.4 invariant #6).
			if(sack_v2_enabled)
				policy_axis1_supremacy_on_move(current_configuration,
					working_config, "frame_gearshift_data_failed_nack");
			send_break_pattern();
			telecom_system->data_container.frames_to_read = 4;
			calculate_receiving_timeout();
			receiving_timer.start();
			return;
		}

		load_configuration(data_configuration, PHYSICAL_LAYER_ONLY,YES);
		add_message_control(REPEAT_LAST_ACK);
	}
	else
	{
		// LINK-PHASE STEP 5 / MC-3 — this outer edge is the ACK_SLOT CLOSE: the
		// receive timer is no longer before receiving_timeout. Credit at most one miss
		// for this armed derived slot, and only if no matching data ACK decoded. Silence
		// observed by any earlier poll cannot reach this counter. Mirror the qualified
		// count into the retained emergency counter so all existing demote/BREAK recovery
		// consumers keep their threshold and recovery path unchanged.
		const bool slot_liveness_active =
			linkphase_slotliveness_on() && linkphase_ack_slot_wait_armed;
		bool slot_qualified_miss = false;
		if(slot_liveness_active && data_ack_received == NO
		   && !linkphase_ack_slot_miss_counted)
		{
			// Site 3 (demote temporal hysteresis, MERCURY_DEMOTE_TEMPORAL_HYSTERESIS=1): a reverse-
			// ACK timeout whose correlator still saw sub-threshold activity (ack_diag_peak_matched
			// >= EMERGENCY_ACKMISS_ACTIVITY_MIN) is a DEFERRED / marginal reverse ACK present on the
			// channel, NOT a channel ACK-absence (a genuine channel-cliff presents as matched==0 and
			// is never held). Hold up to EMERGENCY_ACKMISS_DEFER_MAX such qualified-slot misses
			// WITHOUT crediting missed_ack_slots / emergency_nack_count; the slot is still consumed
			// and slot_qualified_miss stays true so the ordinary retransmit below still fires. Budget
			// spent or pure silence => credited as before so sustained absence still BREAKs. Byte-
			// identical when the gate is off (emergency_ackmiss_defer_hold()==false).
			if(emergency_ackmiss_defer_hold(ack_diag_peak_matched))
			{
				linkphase_ack_slot_miss_counted = true;   // slot consumed; miss HELD (0 demote credit)
				slot_qualified_miss = true;               // retransmit still fires (deferral recovers via resend)
			}
			else
			{
			if(missed_ack_slots < 1000000) missed_ack_slots++;
			linkphase_ack_slot_miss_counted = true;
			slot_qualified_miss = true;
			emergency_nack_count = missed_ack_slots;
			printf("[LINKPHASE-SLOT-LIVENESS] ACK_SLOT closed without decoded ACK: "
			       "missed_ack_slots=%d/%d (config=%d)\n",
				missed_ack_slots, emergency_nack_threshold, current_configuration);
			fflush(stdout);
			}
		}

		if(ack_pattern_time_ms <= 0)
			load_configuration(data_configuration, PHYSICAL_LAYER_ONLY,YES);
		if (last_message_sent_type==CONTROL && last_message_sent_code==REPEAT_LAST_ACK)
		{
			stats.nNAcked_control++;
		}
		if(data_ack_received == NO && (ack_pattern_time_ms > 0 || slot_qualified_miss))
		{
			// Pattern ACK timeout: REPEAT_LAST_ACK is useless (responder can't
			// receive control frames while waiting for data). Skip directly to
			// retransmit — avoids stuck loop where REPEAT_LAST_ACK was queued
			// but never sent (connection_status stayed RECEIVING_ACKS_DATA).
			{
				// Per-symbol mask diagnostic: which positions matched at peak.
				int nsymb = telecom_system->ack_mfsk.ack_pattern_nsymb;
				if(nsymb > 32) nsymb = 32;
				char mask_str[80]; int mp = 0;
				for(int i = 0; i < nsymb && mp < (int)sizeof(mask_str) - 2; i++)
					mask_str[mp++] = (ack_diag_peak_mask & (1u << i)) ? '1' : '0';
				mask_str[mp] = '\0';
				printf("[CMD-ACK-PAT] Timeout: no ACK detected, peak_matched=%d/%d peak_metric=%.1f polls=%d mask=%s\n",
					ack_diag_peak_matched, telecom_system->ack_mfsk.ack_match_threshold,
					ack_diag_peak_metric, ack_diag_poll_count, mask_str);
				// Step 15: legacy MFSK SACK detector diagnostic removed
				// (sack_diag_* state no longer exists).
				fflush(stdout);
			}
			stats.nNAcked_data++;
			// Force all PENDING_ACK messages to ACK_TIMED_OUT so
			// process_messages_tx_data() can resend them immediately.
			// Without this, messages stay PENDING_ACK until their
			// individual ack_timeout fires (~13s), stalling the commander.
			for(int i=0; i<nMessages; i++)
			{
				if(messages_tx[i].status == PENDING_ACK)
				{
					messages_tx[i].status = ACK_TIMED_OUT;
					l1_apply_reset_event(mercury::L1ResetEvent::ACK_TIMEOUT);
				}
			}
			// SACK Design A Step 11 — Axis 3 per-batch tick on full ACK timeout.
			//
			// We do NOT count this path as a SACK miss event by default. The
			// timeout fires for ANY batch where neither SACK_RSP nor ACK
			// pattern was decoded — including full-batch losses (channel
			// dropped every frame, so RSP never saw a partial batch and
			// never sent SACK_RSP in the first place). Counting full-batch
			// loss as a "reverse path SACK unreliability" event would false-
			// positive Axis 3 on adverse forward-channel conditions and
			// race against Axis 1 / emergency BREAK.
			//
			// The §4.3.2 spec's "no SACK heard within the window" case is
			// preserved by the decode_sack_v2_frame() CRC-fail path (which
			// IS specifically a SACK-LDPC/CRC fault) and by future plumbing
			// of partial-batch detection. For Step 11, the cleaner and safer
			// model is: only CRC8/LDPC fails on decoded SACK_RSP frames
			// count as SACK misses. The batch_tick still runs so Axis-1
			// supremacy cooldowns drain and the OFF→PROBE 20-batch timer
			// advances correctly.
			if(sack_v2_enabled)
			{
				axis3_batch_tick();
			}
		}
		this->cleanup();

		// Emergency BREAK: track consecutive complete failures (data + REPEAT_LAST_ACK)
		if(data_ack_received == NO)
		{
			consecutive_data_acks = 0;
			// SUSTAINED-ANCHOR GATE (gearshift-climb-engine.md §11): a failed block
			// breaks the consecutive-clean run at this rung. This reset is UPSTREAM
			// of the BREAK trigger (:3390) — so when a BREAK fires, the clean-streak
			// is already 0 and the next clean re-accumulates from 1.
			clean_batches_at_current_config = 0;
			// R5: count this failed round for the pure-silence classifier (after-probe / late path). Any
			// reverse activity (late pattern, sub-threshold correlator, partial SACK) resets the streak
			// here, so the forward-loss and window-problem demote paths stay byte-identical.
			ackfail_classifier_step(ack_pattern_time_ms, ack_diag_peak_metric, ack_diag_peak_matched, /*sack_this_round=*/false);

			// Record this completed no-ACK transaction as a known zero-delivery
			// outcome.  opt_record_batch() closes its cycle immediately using the
			// TX-start timestamp, so terminal failures remain in the goodput
			// denominator even if no later DATA batch is ever started.
			opt_record_batch(/*bytes_delivered=*/0,
			                 /*sack_used=*/false,
			                 /*failed=*/true);
			// ACTIVE v2 owns ordinary performance adaptation, including known-zero
			// no-ACK outcomes. Evaluate the failed transaction now so repeated
			// forward failures can select a safer evidenced action before BREAK is
			// required. Emergency BREAK below still has precedence: if it fires it
			// cancels this pending normal move. Legacy mode preserves the historical
			// behavior of leaving failure triage to BREAK/ladder machinery.
			opt_pending_switch_cfg = -1;
			opt_pending_switch_action = GEARSHIFT_ACTION_HOLD;
			opt_pending_switch_fallback = -1;
			if(rate_opt.controls_link())
			{
				int rec = current_configuration;
				if(opt_evaluate_batch_end(&rec))
					opt_pending_switch_cfg = rec;
			}

			// An upward Gearshift probe that has not produced even one
			// config-discriminating response has now completed a full batch with
			// zero ACK.  Feed that concrete failure to the existing single owner
			// immediately: it closes the failed probe, selects its recorded
			// fallback, and the lossless CONFIG_TAG coast-down below preserves the
			// batch.  Waiting for the generic NACK threshold can take longer than
			// the link horizon at cfg16 (two 90-frame attempts), which is the
			// observed +10 teardown.  No timer or failure threshold is changed.
			if(probe_full_failure_coastdown_on()
			   && rate_opt.owns_unconfirmed_upward_probe_at(current_configuration))
			{
				int owner_target = rate_opt.consume_failure_signal(
					current_configuration, "unconfirmed_probe_full_batch_failure",
					opt_now_ms(), narrowband_enabled == YES, robust_enabled == YES);
				if(owner_target >= 0 && inband_route_failure_demote(owner_target,
					"unconfirmed_probe_full_batch_failure", true, true))
					return;
			}

			// Frame gearshift just applied but data failed — BREAK immediately.
			// §7.13.33 — retry once before BREAK: on a CFG7→CFG15 PHY-switch,
			// the FIRST batch can fail not because CFG15 is too aggressive
			// but because RSP's clean-batch ACK (now an MFSK suffix; was the
			// deleted OFDM_ACK_CLEAN frame pre-2026-05-24) lands inside CMD's
			// still-muted post-TX window (audioio.c:1255-1258 zeros samples
			// while rx_mute is set). The ACK preamble gets wiped before the
			// decoder sees it, and CMD wrongly concludes the config is
			// unworkable. Retrying the batch once gives RSP a chance to
			// retransmit the clean ACK (mechanism-(a) dedup matches same
			// batch_seq_id) at a time when CMD is fully in RX mode. If the
			// retry also fails, BREAK as before — config really is too
			// aggressive.
			if(frame_gearshift_just_applied && ack_pattern_time_ms > 0
			   && frame_gearshift_retry_count == 0)
			{
				frame_gearshift_retry_count = 1;
				printf("[GEARSHIFT] First-batch ACK miss after PHY-switch on cfg %d — retrying batch once before BREAK (suspected rx_mute timing race)\n",
					data_configuration);
				fflush(stdout);
				// Don't trigger BREAK or downshift here. Fall through to
				// normal nack handling below which will retransmit the
				// batch. frame_gearshift_just_applied stays TRUE so a
				// second failure will hit the BREAK path below.
			}
			else if(frame_gearshift_just_applied && ack_pattern_time_ms > 0)
			{
				frame_gearshift_just_applied = false;
				frame_gearshift_retry_count = 0;
				// FIX-B — ARM the floor-probe back-off on the FAILED up-probe rung
				// (gearshift-floor-probe-backoff.md §5.1, arm-site #3). pat data-fail:
				// the just-applied FRAME-UP to data_configuration could not pass DATA
				// (the MFSK-ACK-PAT path, after the §7.13.33 single retry already
				// failed). Arm BEFORE the working_config overwrite below clobbers
				// data_configuration, so the back-off is keyed to the rung that failed.
				probe_backoff_arm(data_configuration);
				int working_config = config_ladder_down(data_configuration, robust_enabled);
				frame_shift_threshold *= 2;

				// STAGE 4c — Class-A site A3 (FRAME-UP data fail, pat). Same routing as A2:
				// under inband the failed up-probe DEMOTES one rung via the CONFIG_TAG; at the
				// bottom it ticks the true-loss floor and BREAKs only at SESSION_DEAD_BATCHES.
				// Byte-identical OFF.
				if(inband_rate_feature_enabled())
				{
					if(working_config != data_configuration
					   && inband_route_failure_demote(working_config, "frame_gearshift_data_failed_pat"))
						return;
					if(inband_cmd_dead_batch_floor_reached())
					{
						emergency_previous_config = current_configuration;
						emergency_break_active = 1;
						emergency_break_retries = 3;
						emergency_nack_count = 0;
						cfg16_revack_starve_fails = 0;
						if(sack_v2_enabled)
							policy_axis1_supremacy_on_move(current_configuration,
								working_config, "inband_session_dead_batches_floor");
						send_break_pattern();
						telecom_system->data_container.frames_to_read = 4;
						calculate_receiving_timeout();
						receiving_timer.start();
					}
					return;
				}

				// LOW-SNR CEILING PIN (data-flow-gearshift-climb.md): a partial-pattern up-probe
				// data-fail is a FAILED rung EXACTLY like the pure-silence twin (:5493-5494) — a
				// partial (e.g. 0.22-decode) batch is NOT a viable rung. Pin the decode-learned
				// ceiling to the retreat rung so the elevator cannot re-leap above the last
				// decode-supported config. WITHOUT this pin the ceiling stayed -1 → cfg13 (pat) fail
				// → cfg12 anchor-raise → the elevator recomputed a high target bounded only by the
				// leap cap → re-leap cfg12→cfg16 (dead at low SNR) → BREAK → collapse. This is the
				// SHARED-INVARIANT restore the cross-layer audit found missing on ONLY this twin (the
				// SET_CONFIG-ACK NAck path :3602 is a distinct control-handshake class with its own
				// probe_backoff and is intentionally left unchanged). DEFAULT-ON;
				// MERCURY_CEILING_PIN_DEFEAT=1 restores the missing-pin behavior for the A/B.
				if(!ceiling_pin_defeat)
				{
					if(supershift_proven_ceiling < 0 || working_config < supershift_proven_ceiling)
						supershift_proven_ceiling = working_config;
					printf("[GEARSHIFT] CEILING PIN (pat): config %d failed -> ceiling %d\n",
						data_configuration, working_config);
					fflush(stdout);
				}

				printf("[GEARSHIFT] FRAME UP DATA FAILED (pat): config %d -> BREAK to %d (threshold %d)\n",
					data_configuration, working_config, frame_shift_threshold);
				fflush(stdout);

				restage_requeue_tx_messages();   // §12 lossless+ordered re-stage (was forward push()-to-BACK)
				fifo_buffer_backup.flush();
				block_under_tx = NO;

				data_configuration = working_config;
				negotiated_configuration = working_config;
				emergency_previous_config = working_config;
				break_drop_step = 0;
				emergency_break_active = 1;
				// Defense-in-depth: even with always_fine BREAK refinement on
				// RSP, allow 3 BREAK transmissions before exhausted-recovery
				// drops CMD to CFG0 unilaterally. The other-direction emergency
				// path (around line ~2377) already uses higher retries; this
				// brings the gearshift_data_failed_pat path in line.
				emergency_break_retries = 3;
				emergency_nack_count = 0;
				cfg16_revack_starve_fails = 0;  // WALL-B FIX-9 D3 (P4): demote off CFG16 -> streak fresh

				// SACK Design A Step 12 — BREAK supremacy (§4.3.4 invariant #6).
				if(sack_v2_enabled)
					policy_axis1_supremacy_on_move(current_configuration,
						working_config, "frame_gearshift_data_failed_pat");
				send_break_pattern();
				telecom_system->data_container.frames_to_read = 4;
				calculate_receiving_timeout();
				receiving_timer.start();
				return;
			}

			// Count toward emergency BREAK. With Step 5 active the ACK_SLOT-close
			// edge above already mirrored missed_ack_slots into this retained counter;
			// OFF/DEFEAT executes the original raw increment byte-for-byte.
			if(!slot_liveness_active)
			{
				// Site 1 (demote temporal hysteresis, MERCURY_DEMOTE_TEMPORAL_HYSTERESIS=1):
				// a data-ACK timeout while a COMPLETE L1 block aggregate is legitimately
				// outstanding is a DEFERRED / still-in-flight aggregate, NOT ACK-absence.
				// Hold it (do not count) for a bounded budget so the ordinary retransmit
				// (which triggers the responder aggregate replay) can recover; sustained
				// absence (budget spent) still counts and reaches the BREAK threshold.
				// Byte-identical when the gate is off (l1_aggregate_defer_hold()==false).
				if(!l1_aggregate_defer_hold() && !emergency_ackmiss_defer_hold(ack_diag_peak_matched))
					emergency_nack_count++;
			}

			// WALL-B FIX-9 D3 (FIX9_D3_DESIGN.md §3, producer P2): count CONSECUTIVE block-failures
			// AT CONFIG_16 where the reverse MFSK ACK+SACK correlator FAILED TO DECODE (we are in the
			// data_ack_received==NO branch — the reverse ACK timed out). This is "reverse-ACK loss":
			// on HW the correlator window goes pure-silent (peak_metric=0.0); in the drift-sim it
			// degrades to a SUB-THRESHOLD partial (peak_matched 5/7, peak_metric~0.5) — BOTH mean the
			// ACK did not decode (matched < ack_match_threshold). We do NOT gate on the residual
			// metric magnitude (that was HW-incidental and never holds in the sim). The CFG16
			// restriction (D3-1) + the OFDM-proven gate (D3-2, applied at the consumer below) are what
			// keep this from being a generic ACK-timeout rebrand. On any non-CFG16 rung the streak
			// resets so a later return to CFG16 starts fresh (P4); the deadline is consumed (reset to
			// 0) in the D3 demote block below.
			if(current_configuration == CONFIG_16)
				cfg16_revack_starve_fails++;
			else
				cfg16_revack_starve_fails = 0;

			// Batch halving disabled (batch size is fixed at negotiated value).
			// Halving causes CMD/RSP batch size mismatch → ACK-GATE desync.
			printf("[BREAK] Block failure #%d at config %d (threshold=%d, batch=%d)\n",
				emergency_nack_count, current_configuration, emergency_nack_threshold, data_batch_size);
			fflush(stdout);
			if(cfg16_revack_starve_fails > 0)
			{
				printf("[CFG16-HOLD] FIX-9 D3: CFG16 reverse-ACK-loss streak %d/%d "
					"(anchor=%d ofdm_proven=%d peak_metric=%.2f matched=%d/%d)\n",
					cfg16_revack_starve_fails, CFG16_REVACK_STARVE_FAILS, last_data_viable_config,
					is_ofdm_config(last_data_viable_config) ? 1 : 0, ack_diag_peak_metric,
					ack_diag_peak_matched, telecom_system->ack_mfsk.ack_match_threshold);
				fflush(stdout);
			}

			// === WALL-B FIX-4 — CARVE-VIABILITY DEADLINE -> per-frame CFG15 fallback ===
			// (WALLB_DIAGNOSIS.md §7 FIX-4, §1.1, §1.6.) On a CLEAN channel the link
			// holds CFG16 on SACK-trust (the CFG16-HOLD skip at :5006), but the big-block
			// carve fires 0x (wall-C) so the RSP never sends a data-ACK. emergency_nack_count
			// counts CONSECUTIVE total block-failures and RESETS to 0 on ANY data-ACK
			// (clean OR partial, :3699), so reaching emergency_nack_threshold (3) on the
			// CFG16 big-block rung means ZERO carves landed within the deadline. The
			// EXISTING behavior here is the BREAK cascade below — which, on this rung with
			// a CFG16-deaf RSP, is the wall-B free-flow collapse to ROBUST_0 (0 bytes on
			// long transfers). PRE-EMPT it: treat "CFG16 big-block won't carve" as "CFG16
			// big-block not viable, fall back to PER-FRAME CFG15" (the highest decodable
			// per-frame rung), NOT "channel bad, BREAK toward ROBUST_0".
			//
			// The RSP CAN follow this demote (unlike the deaf BREAK): per-frame CFG15 is
			// NOT carve-gated (the RX carve gate at arq_common.cc:8186 and the TX
			// bigblock_send_one_block at :4233 BOTH require current_configuration==CONFIG_16),
			// and the demote is signalled via the EXISTING, decodable SET_CONFIG control
			// exchange — the SET_CONFIG frame is sent on the live CFG16 PHY and re-decoded by
			// the RSP on the STOCK per-frame path via the GAP-3 carve-gate fallback
			// (arq_common.cc:8186-8211 sets bigblock_rx_force_stock and re-runs receive_byte;
			// comment :8066-8077 documents this exact "single OFDM control frame parsed
			// normally and ACKed" path). HW evidence: cb_rsp_a1.log followed 9 SET_CONFIGs
			// up the ladder (100..16) including 15->16. So the RSP loads CFG15, ACKs, and
			// both peers run per-frame CFG15 -> DATA flows at the ~2479 per-frame ceiling.
			//
			// This is FIX-1's policy expressed at the block-failure decision point WITHOUT
			// touching FIX-1's demote targeting (config_ladder_down_n / emergency_previous_config
			// re-pin are UNCHANGED) or FIX-3's RSP receive routing (no RSP code changes — it
			// reuses the GAP-3 fallback already in bb-d3). The generic fade-down BREAK on
			// non-bigblock rungs is byte-identical (the helper returns -1 there), and the
			// deep-SNR escape for a genuinely-cratered channel is preserved (FIX-4 only
			// pre-empts on the CFG16 big-block rung). It fires only when the BREAK would
			// otherwise fire, so it is gated on the SAME outer conditions as the BREAK below.
			{
				int carve_fallback = bigblock_carve_fallback_target(
					current_configuration,
					(telecom_system != NULL
					 && telecom_system->bigblock_framing_enabled
					 && telecom_system->M != MOD_MFSK),
					emergency_nack_count, emergency_nack_threshold, robust_enabled);
				if(carve_fallback >= 0
				   && !config_is_at_bottom(current_configuration, robust_enabled)
				   && !emergency_break_active
				   && turboshift_phase == TURBO_DONE
				   && gear_shift_on == YES)
				{
					printf("[CFG16-HOLD] carve-viability DEADLINE: %d consecutive CFG16 "
						"big-block failures with 0 carves -> falling back to PER-FRAME "
						"config %d via SET_CONFIG (NOT the BREAK cascade); per-frame CFG15 "
						"is not carve-gated so the RSP follows via the GAP-3 stock decode\n",
						emergency_nack_count, carve_fallback);
					fflush(stdout);

					// Preserve all pending data for resend at the per-frame rung — the SAME
					// FIFO push-back the gearshift BREAK paths use (:3367-3381), so no bytes
					// are dropped by the path switch.
					if(compression_enabled)
					{
						restore_tx_from_compressed();
					}
					else
					{
						restage_requeue_tx_messages();   // §12 lossless+ordered re-stage (was forward push()-to-BACK)
						fifo_buffer_backup.flush();
						clear_retx_queue();   // recovery re-queues plaintext; drop stale retx
					}
					block_under_tx = NO;

					// Demote to the per-frame rung. SET_CONFIG encodes
					// forward=negotiated_configuration (add_message_control SET_CONFIG,
					// :702/:709), so set negotiated_configuration to the fallback. Pin the
					// proven ceiling so the SNR re-trigger does not immediately re-elect the
					// CFG16 carve that just proved non-viable.
					data_configuration = carve_fallback;
					negotiated_configuration = carve_fallback;
					supershift_proven_ceiling = carve_fallback;

					// WALL-B FIX-5: ARM/EXTEND the CFG16 big-block carve COOLDOWN. The
					// supershift_proven_ceiling pin above has a lifetime of ~one cycle
					// (finish_turbo_direction() resets it to the probe top, CFG16, at :4147 on
					// the next ROBUST->CFG16 re-climb), so on pg84 the loop re-climbs and
					// re-elects the carve-dead rung -> 0 bytes. This SEPARATE field is NOT
					// touched by the turbo state machine, so it survives that reset and caps the
					// climb at CFG15 across cycles. AARF: re-demoting WHILE a prior cooldown is
					// still active (the carve is STILL dead) GROWS the window; a fresh
					// (post-expiry) failure starts at BASE. Cleared on a real CFG16 carve
					// (data-ACK reset, :3804). See fix5/FIX5_DESIGN.md §4.3 + the §5 audit.
					bigblock_carve_cooldown_span = bigblock_carve_cooldown_next_span(
						(bigblock_carve_cooldown_batches > 0) ? bigblock_carve_cooldown_span : 0,
						BB_CARVE_COOLDOWN_BASE, BB_CARVE_COOLDOWN_MAX);
					bigblock_carve_cooldown_batches = bigblock_carve_cooldown_span;
					printf("[CFG16-HOLD] FIX-5 ARM: CFG16 big-block carve cooldown ARMED for %d "
						"batches (span now %d) — climb capped at per-frame CFG%d until a real "
						"CFG16 carve lands\n",
						bigblock_carve_cooldown_batches, bigblock_carve_cooldown_span,
						config_ladder_down(CONFIG_16, robust_enabled));
					fflush(stdout);

					// Consumed: we are acting on the deadline. Clear the streak so a fresh
					// per-frame failure re-accumulates from 1 (and we do NOT fall through to
					// the BREAK gate below).
					emergency_nack_count = 0;

					// SACK Design A Step 12 — BREAK supremacy hook (a config MOVE; mirrors the
					// BREAK paths). Keeps the optimizer Axis-1 cooldown coherent across the
					// path switch. This is NOT a BREAK — no send_break_pattern, no
					// emergency_break_active — it is the decodable SET_CONFIG demote.
					if(sack_v2_enabled)
						policy_axis1_supremacy_on_move(current_configuration,
							carve_fallback, "bigblock_carve_viability_deadline");

					// Emit the SET_CONFIG control frame on the live CFG16 PHY (the RSP
					// re-decodes it via the GAP-3 stock fallback) and wait for its ACK; the
					// :4707 SET_CONFIG-ACK handler then loads CFG15 on the CMD. Mirrors the
					// climb-UP SET_CONFIG emit (:4965-4968) exactly.
					cleanup();
					add_message_control(SET_CONFIG);
					this->connection_status = TRANSMITTING_CONTROL;
					return;
				}
			}

			// === WALL-B FIX-9 D3 — CFG16 PER-FRAME REVERSE-ACK STARVATION -> per-frame CFG15 ===
			// (bigblock_p3_hw/_fix9/FIX9_ROOTCAUSE.md §1/§2 Rank-1, FIX9_D3_DESIGN.md, FIX9_D3_AUDIT.md.)
			// The PER-FRAME (FRAMING-UNSET) sibling of FIX-4. On a CLEAN channel the link climbs to
			// CONFIG_16 and SUSTAINS the 25-frame batch; inter-Pi sample-clock drift (-670 ppm,
			// pg_wb_rsp_clean_A1.log:13365) de-aligns the half-duplex turnaround so the reverse MFSK
			// ACK+SACK lands OUTSIDE the CMD's window -> the correlator goes PURE SILENT
			// (peak_metric=0.0, pg_wb_cmd_clean_A1.log:9674) even though the FORWARD data was
			// delivering (CLEAN/PARTIAL SACK bitmaps). 3 such block-failures BREAK off CFG16 -> the
			// ROBUST cascade -> 0 bytes. PRE-EMPT it: when (D3-1) the failing rung is CFG16, (D3-2)
			// the forward link was sustainably healthy there (last_data_viable_config==CFG16, raised
			// ONLY after >=2 consecutive CLEAN CFG16 batches — the signal that separates this from a
			// deep-SNR collapse where the anchor never reaches CFG16), (D3-3/4) the reverse ACK has
			// been PURE SILENT for CFG16_REVACK_STARVE_FAILS consecutive block-failures, and (D3-5)
			// we are on the per-frame path (NOT the FIX-4 big-block carve), demote DIRECTLY to the
			// highest per-frame OFDM rung (config_ladder_down(CFG16)==CFG15, which has the turnaround
			// margin CFG16 lacks) via the EXISTING decodable SET_CONFIG control exchange, and ARM the
			// FIX-5 CFG16 cooldown so the climb is held at CFG15 across cycles (the SNR re-trigger
			// must NOT immediately re-elect the starved rung). NOT a BREAK toward ROBUST. The deep-SNR
			// escape for a genuinely-cratered channel is PRESERVED (D3-2 is false there). Mutually
			// exclusive with FIX-4 (D3-5). Fires only when the BREAK would otherwise fire (same outer
			// guards), so non-CFG16 / non-starved / framing-live operation is byte-identical.
			{
				bool d3_bigblock_live = (telecom_system != NULL
					&& telecom_system->bigblock_framing_enabled
					&& telecom_system->M != MOD_MFSK);
				int d3_target = cfg16_revack_starve_fallback_target(
					current_configuration,
					/*ofdm_proven=*/is_ofdm_config(last_data_viable_config),
					/*bigblock_rung_live=*/d3_bigblock_live,
					cfg16_revack_starve_fails, CFG16_REVACK_STARVE_FAILS, robust_enabled);
				if(d3_target >= 0
				   && !config_is_at_bottom(current_configuration, robust_enabled)
				   && !emergency_break_active
				   && turboshift_phase == TURBO_DONE
				   && gear_shift_on == YES)
				{
					printf("[CFG16-HOLD] FIX-9 D3 reverse-ACK STARVATION: %d consecutive CFG16 "
						"reverse-ACK-loss block-failures (OFDM proven on this channel, anchor=%d) -> "
						"demoting to config %d via SET_CONFIG (NOT the BREAK->ROBUST cascade); CFG15 "
						"has the half-duplex turnaround margin CFG16 lacks under clock drift\n",
						cfg16_revack_starve_fails, last_data_viable_config, d3_target);
					fflush(stdout);

					// WALL-B FIX-9 LOSSLESS DEMOTE (data-flow-revack-turnaround-geometry.md §5.3): the
					// in-flight CFG16 batch that starved the reverse-ACK was ALREADY DELIVERED to the
					// app by the RSP (copy_data_to_buffer fires right after the RSP sends the ACK the
					// CMD missed), advancing rsp_last_delivered. The bytes are NOT lost (the FIFO
					// push-back below preserves them) — but the PRE-fix re-send rebuilt the batch under
					// a FRESH epoch (cmd_batch_seq_id had advanced past the in-flight bsi), so the RSP
					// saw the re-presented (duplicate) batch as a NON-CONTIGUOUS hole and the D3.1
					// in-order guard GAP-ABORTed (the _cfg16hold2 clean-cell md5-mismatch). The bytes
					// were correct; the bsi-epoch LABEL was wrong. Roll cmd_batch_seq_id back to the
					// in-flight batch's ORIGINAL bsi so the re-sent batch carries the CONTIGUOUS bsi
					// the RSP last expected -> sack_v2_readopt_has_gap sees a duplicate-of-delivered
					// (==last) or next (==last+1), NOT a >=2 jump -> NO abort (or a clean dedup).
					// Capture the in-flight batch's bsi BEFORE freeing messages_tx[] (the v2 in-order
					// path only; the compression path's restore_tx_from_compressed() owns its own
					// re-stage and is unchanged). All non-FREE frames of the stranded block share the
					// SAME batch_seq_id (one batch is in flight on the per-frame path), so the EARLIEST
					// (mod-256) non-FREE bsi is the in-flight batch. Robust/NB never reach this
					// CFG16-only leg.
					int min_inflight_bsi = -1;
					if(sack_v2_enabled && !compression_enabled)
					{
						for(int i=0; i<nMessages; i++)
						{
							if(messages_tx[i].status != FREE && messages_tx[i].length > 0)
							{
								int b = messages_tx[i].batch_seq_id & 0xFF;
								if(min_inflight_bsi < 0)
									min_inflight_bsi = b;
								else
								{
									// keep the mod-256-EARLIER bsi (forward distance b->cur in [1,128]
									// means cur is later, so b is earlier).
									unsigned fwd = ((unsigned)(min_inflight_bsi - b)) & 0xFFu;
									if(fwd >= 1u && fwd <= 128u)
										min_inflight_bsi = b;
								}
							}
						}
					}

					// Preserve all pending data for resend at the per-frame rung — the SAME FIFO
					// push-back the gearshift BREAK paths + FIX-4 use, so no bytes are dropped.
					if(compression_enabled)
					{
						restore_tx_from_compressed();
					}
					else
					{
						restage_requeue_tx_messages();   // §12 lossless+ordered re-stage (was forward push()-to-BACK)
						fifo_buffer_backup.flush();
						clear_retx_queue();   // recovery re-queues plaintext; drop stale retx
					}
					block_under_tx = NO;

					// Roll the bsi epoch back to the in-flight batch so the re-sent batch is
					// CONTIGUOUS with the RSP's delivery high-water (the lossless re-present). Only
					// when an in-flight bsi was actually captured (else leave the counter untouched).
					if(min_inflight_bsi >= 0)
					{
						printf("[CFG16-HOLD] FIX-9 LOSSLESS DEMOTE: rolling cmd_batch_seq_id %d -> %d "
							"so the re-sent batch carries the in-flight (contiguous) bsi (no D3.1 GAP-ABORT)\n",
							cmd_batch_seq_id & 0xFF, min_inflight_bsi);
						fflush(stdout);
						cmd_batch_seq_id = min_inflight_bsi;
					}

					// Demote to the per-frame rung. Pin the proven ceiling (one-cycle) so the SNR
					// re-trigger does not immediately re-elect the starved CFG16 rung, and ARM the
					// FIX-5 cooldown (cross-cycle hold; survives finish_turbo_direction()'s
					// supershift_proven_ceiling reset at :4147). Identical to the FIX-4 demote.
					data_configuration = d3_target;
					negotiated_configuration = d3_target;
					supershift_proven_ceiling = d3_target;

					// WALL-B FIX-9 D3 REUSES the FIX-5 cooldown machinery: the field's role
					// generalizes from "carve-dead" to "CFG16-not-viable-on-this-channel" (here:
					// reverse-ACK starved). AARF: re-demoting WHILE a prior cooldown is still active
					// (CFG16 STILL starved) GROWS the window; a fresh (post-expiry) failure starts at
					// BASE. The cooldown's CLEAR-on-success gate (INV-B3: only a CFG16 big-block carve
					// data-ACK clears) is UNTOUCHED — on the per-frame path no carve ever lands, so the
					// hold survives the whole CFG15 delivery window and expires by batch-count for an
					// optimistic CFG16 re-probe. See FIX9_D3_AUDIT.md Q4 INV-2.
					bigblock_carve_cooldown_span = bigblock_carve_cooldown_next_span(
						(bigblock_carve_cooldown_batches > 0) ? bigblock_carve_cooldown_span : 0,
						BB_CARVE_COOLDOWN_BASE, BB_CARVE_COOLDOWN_MAX);
					bigblock_carve_cooldown_batches = bigblock_carve_cooldown_span;
					printf("[CFG16-HOLD] FIX-9 D3 ARM: CFG16 cooldown ARMED for %d batches (span now "
						"%d) — climb capped at per-frame CFG%d until the cooldown expires for a re-probe\n",
						bigblock_carve_cooldown_batches, bigblock_carve_cooldown_span,
						config_ladder_down(CONFIG_16, robust_enabled));
					fflush(stdout);

					// Consumed the deadline: clear BOTH the starve streak and the nack streak so a
					// fresh per-frame failure re-accumulates from 1 (and we do NOT fall through to the
					// BREAK gate below). Mirrors the FIX-4 emergency_nack_count=0.
					cfg16_revack_starve_fails = 0;
					emergency_nack_count = 0;

					// SACK Design A Step 12 — BREAK supremacy hook (a config MOVE; mirrors the BREAK
					// paths + FIX-4). Keeps the optimizer Axis-1 cooldown coherent across the path
					// switch. NOT a BREAK — no send_break_pattern, no emergency_break_active.
					if(sack_v2_enabled)
						policy_axis1_supremacy_on_move(current_configuration,
							d3_target, "cfg16_revack_starvation_deadline");

					// Emit the SET_CONFIG control frame on the live CFG16 PHY (the RSP re-decodes it
					// and follows to per-frame CFG15) and wait for its ACK. Mirrors the FIX-4 demote
					// and the climb-UP SET_CONFIG emit exactly.
					cleanup();
					add_message_control(SET_CONFIG);
					this->connection_status = TRANSMITTING_CONTROL;
					return;
				}
			}

			// R5 -- pure-silence reroute. If EVERY counted round to this point had ZERO reverse correlator
			// activity (peer unreachable at every attempted config), grinding the demote ladder to ROBUST is
			// useless. Probe-then-clean-reconnect instead. Evidence-bearing paths (partial SACK -> data_ack_
			// received=YES upstream; sub-threshold/late correlator -> counter already reset) never satisfy this,
			// so they stay byte-identical. Guards match the BREAK gate own guards (emergency_break_active /
			// turboshift_phase). See the cross-layer data-flow audit (data-flow-zombie-amplifier.md).
			if(demote_silence_reroute_enabled()
			   && silence_reroute_should_fire(consec_pure_silent_rounds, DEMOTE_SILENCE_MAX_ROUNDS,
			                                  emergency_break_active, turboshift_phase))
			{
				printf("[DEMOTE-SILENCE] %d consecutive pure-silent rounds at config %d -- peer unreachable "
				       "at every attempted config; clean reconnect instead of grinding the demote ladder\n",
				       consec_pure_silent_rounds, current_configuration);
				fflush(stdout);
				consec_pure_silent_rounds = 0;
				if(rate_opt.controls_link())
				{
					int hint = config_ladder_down(current_configuration, robust_enabled);
					if(hint != current_configuration)
					{
						inband_route_failure_demote(hint, "demote_silence_peer_unreachable");
					}
					else if(rate_opt.authorize_hard_recovery(
						current_configuration, true, "demote_silence_peer_unreachable"))
					{
						emergency_previous_config = current_configuration;
						emergency_break_active = 1;
						emergency_break_retries = 3;
						emergency_nack_count = 0;
						send_break_pattern();
						telecom_system->data_container.frames_to_read = 4;
						calculate_receiving_timeout();
						receiving_timer.start();
					}
					else
					{
						printf("[GEARSHIFT-V2-AUTHORITY] pure-silence reconnect request "
							"denied by owner at cfg=%d\n", current_configuration);
						fflush(stdout);
					}
					return;
				}
				commander_clean_reconnect("demote_silence_peer_unreachable");
				return;
			}
			
			// Trigger BREAK when threshold reached and not already at bottom
			if(emergency_nack_count >= emergency_nack_threshold
			   && !config_is_at_bottom(current_configuration, robust_enabled)
			   && !emergency_break_active
			   && turboshift_phase == TURBO_DONE
			   && gear_shift_on == YES)
			{
				// STAGE 4c — Class-A site A4 (emergency-NACK threshold). Under inband, a
				// degradation that today BREAKs instead routes to a TAG-DEMOTE one rung down
				// (the RX down-ladder catches a missed tag) — NEVER the BREAK->ROBUST cascade.
				// We are NOT at the ladder bottom here (the outer guard), so a lower rung
				// exists. Byte-identical when the feature is OFF (the legacy BREAK below runs
				// unchanged). (data-flow-perbatch-config.md §S4C, design §5.3.)
				if(inband_rate_feature_enabled())
				{
					if(rate_opt.controls_link())
					{
						// Preserve the detector as a telemetry producer. It supplies no
						// destination: GS2 consumes the threshold signal, selects the
						// coast-down rung, opens its own rollback transaction, and the
						// existing helper carries that owned move losslessly.
						int owner_target = rate_opt.consume_failure_signal(
							current_configuration, "emergency_nack_threshold",
							opt_now_ms(), narrowband_enabled == YES,
							robust_enabled == YES);
						if(owner_target >= 0 && inband_route_failure_demote(owner_target,
							"emergency_nack_threshold", true, true)) return;
					}
					else if(inband_route_failure_demote(
						config_ladder_down(current_configuration, robust_enabled),
						"emergency_nack_threshold")) return;
					// Same-config / no-op (should not happen with the outer not-at-bottom
					// guard) — fall through to the legacy BREAK as a safety net.
				}
				// WALL-B FIX-9 D3 (P4): a genuine BREAK is firing (D3 did NOT pre-empt — either not
				// CFG16, the anchor was not CFG16, the ACK was garbled-not-silent, or framing was
				// live). The link is about to leave CFG16 via BREAK recovery, so the reverse-ACK
				// starvation streak's premise (sustained CFG16) is gone — clear it so a later
				// re-climb to CFG16 starts fresh.
				cfg16_revack_starve_fails = 0;
				// Panic-mode jump: if a previous BREAK fired with no data success
				// between, the channel cratered hard and the ladder isn't keeping
				// up. Skip the doubling — set break_drop_step large so the
				// recovery path (arq_commander.cc:81) lands directly on ROBUST_0
				// (config_ladder_down_n clamps to the floor). The line 3074 data-
				// success reset clears the counter; until then every consecutive
				// BREAK bottoms out.
				breaks_since_last_data_success++;
				// B2 (default-off consumer): a BREAK ends the healthy hold — disarm the freshness
				// exemption immediately (the recovery path is never gated regardless; this only
				// prevents a stale-meter one-rung climb from firing during recovery).
				rung_meter_healthy_hold_batches = 0;
				// B2 (default-off consumer): a BREAK also invalidates the latch-max — a peak captured
				// before the storm must not ride a climb during/after recovery. The hold is re-earned
				// only once fresh qualified reads flow again (rung_meter_note_read gates on breaks==0).
				rung_meter_hold_db = -99.9;
				rung_meter_hold_age_batches = 1000000000;
				if(breaks_since_last_data_success >= 2)
				{
					printf("[BREAK-PANIC] %d BREAKs without data success — "
						"forcing jump to ROBUST_0 (break_drop_step=100)\n",
						breaks_since_last_data_success);
					fflush(stdout);
					break_drop_step = 100;  // clamp at floor of ladder
				}

				// DEEP-SNR DOWN-HYSTERESIS (gearshift-climb-engine.md §10) — anchor
				// DEMOTION, the SECOND escape complementary to the breaks>=2 panic.
				// At the WGN:-10 cliff a slow retransmit-rescued batch at the anchor
				// rung emits an all-ones completion ACK (arq_responder.cc prev-path)
				// → CMD raises the anchor AND resets breaks_since_last_data_success
				// to 0 (:3445). So the panic counter oscillates 1→0→1→0 and NEVER
				// reaches 2; break_target_with_anchor (:147) then clamps every BREAK
				// recovery UP to the anchor rung → infinite CONFIG_0↔ROBUST_0 thrash.
				// This counter is independent of that reset: it increments ONLY when
				// the BREAK fires WHILE current_configuration == last_data_viable_config
				// (the anchor rung itself is breaking), and after K consecutive such
				// failures it LOWERS the anchor one rung. break_target_with_anchor
				// then permits the drop on the next BREAK and the link escapes toward
				// ROBUST_0 — even though the slow completions keep resetting the panic
				// counter. Demotion ONLY lowers the anchor (config_ladder_down), so the
				// +1 up-clamp gets STRICTER, never looser (no af14a9e over-climb regression).
				if(current_configuration == last_data_viable_config)
				{
					anchor_consec_break_fails++;
					if(anchor_consec_break_fails >= ANCHOR_DEMOTE_BREAK_FAILS)
					{
						// PURE decision shared with --test-climb-engine Part E.
						int demoted = anchor_demote_target(last_data_viable_config,
							anchor_consec_break_fails, robust_enabled);
						if(demoted != last_data_viable_config)
						{
							printf("[BREAK] Anchor DEMOTE: %d consecutive BREAKs at anchor rung %d — "
								"lowering anchor %d -> %d (escapes the deep-SNR thrash)\n",
								anchor_consec_break_fails, current_configuration,
								last_data_viable_config, demoted);
							fflush(stdout);
							last_data_viable_config = demoted;
						}
						else
						{
							// Already at the ladder floor (ROBUST_0 / CONFIG_0) — nothing
							// lower to demote to. The panic-jump bypass owns the escape here.
							printf("[BREAK] Anchor DEMOTE: anchor already at floor %d — "
								"panic-jump owns the escape\n", last_data_viable_config);
							fflush(stdout);
						}
						anchor_consec_break_fails = 0;
					}
					else
					{
						printf("[BREAK] Anchor-rung BREAK %d/%d at config %d (anchor=%d) — "
							"demote pending\n",
							anchor_consec_break_fails, ANCHOR_DEMOTE_BREAK_FAILS,
							current_configuration, last_data_viable_config);
						fflush(stdout);
					}
				}

				// PER-RUNG FAILURE MEMORY (audit §4): a BREAK is committing at the live rung. If the
				// per-rung floor table ADMITTED this rung, arm/advance its disproof so a re-climb does
				// not re-elect it (the runtime hedge where the static table is optimistic). Writes ONLY
				// the failure-memory arrays -> the af14a9e ratchet + panic bypass are byte-identical.
				rung_floor_note_break(current_configuration);

				// Lower ceiling to prevent climbing back to failing config
				int new_ceiling = config_ladder_down(current_configuration, robust_enabled);
				if(supershift_proven_ceiling < 0 || new_ceiling < supershift_proven_ceiling)
				{
					supershift_proven_ceiling = new_ceiling;
					ceiling_success_count = 0;
					printf("[BREAK] Lowered ceiling to %d\n", new_ceiling);
					fflush(stdout);
				}
				// === M6 — BREAK-PATH LOSSLESS REQUEUE (data-flow-arq-recovery-cluster.md §5.6) ===
				// The stranded in-flight batch(es) live in messages_tx[] right now (the FIFO
				// push-back + FREE that recovers their bytes happens LATER, in the BREAK recovery
				// handler at :288-296 / :376-384, and that handler does NOT touch cmd_batch_seq_id).
				// PRE-FIX the recovery re-sends those bytes under a FRESH epoch bsi (cmd_batch_seq_id
				// had advanced past the in-flight bsi), so after the RSP's BREAK self-heal
				// (rsp_current/prev reset to -1, arq_responder.cc:474; delivery high-water SURVIVES)
				// the re-adopt sees sack_v2_readopt_has_gap(fresh_bsi, high-water)=true (a >=2 jump)
				// -> [RSP-V2-GAP-ABORT] terminally kills the transfer. The bytes were preserved; the
				// bsi-epoch LABEL was wrong. Roll cmd_batch_seq_id back to the EARLIEST (mod-256)
				// in-flight batch's ORIGINAL bsi — by construction the next bsi the RSP expects after
				// its delivery high-water (un-ACKed-but-delivered => ==high-water, a dedup; or
				// un-ACKed-and-undelivered => ==high-water+1, the contiguous successor) — so the
				// recovery re-send is CONTIGUOUS and the GAP-ABORT does NOT fire. Identical capture
				// to the FIX-9 D3 demote (:3968-3988) under the SAME preconditions
				// (sack_v2_enabled && !compression_enabled — the compression path's
				// restore_tx_from_compressed() owns its own re-stage and is unchanged). DEFAULT-OFF
				// (break_lossless_requeue_enabled): unset => this whole block is skipped =>
				// BYTE-IDENTICAL to the pre-M6 BREAK. NOT a change to the demote/BREAK itself — only
				// the bsi bookkeeping is corrected.
				if(break_lossless_requeue_enabled() && sack_v2_enabled && !compression_enabled)
				{
					int min_inflight_bsi = -1;
					for(int i=0; i<nMessages; i++)
					{
						if(messages_tx[i].status != FREE && messages_tx[i].length > 0)
						{
							int b = messages_tx[i].batch_seq_id & 0xFF;
							if(min_inflight_bsi < 0)
								min_inflight_bsi = b;
							else
							{
								// keep the mod-256-EARLIER bsi (forward distance b->min in
								// [1,128] means min is later, so b is earlier).
								unsigned fwd = ((unsigned)(min_inflight_bsi - b)) & 0xFFu;
								if(fwd >= 1u && fwd <= 128u)
									min_inflight_bsi = b;
							}
						}
					}
					if(min_inflight_bsi >= 0)
					{
						printf("[BREAK] M6 LOSSLESS REQUEUE: rolling cmd_batch_seq_id %d -> %d "
							"so the post-BREAK re-send carries the in-flight (contiguous) bsi "
							"(no RSP-V2-GAP-ABORT on recovery)\n",
							cmd_batch_seq_id & 0xFF, min_inflight_bsi);
						fflush(stdout);
						cmd_batch_seq_id = min_inflight_bsi;
					}
				}

				printf("[BREAK] Sending emergency BREAK pattern\n");
				fflush(stdout);
				// LINK-PHASE PRIMITIVE (increment 1): BREAK re-stages the sender -> owner
				// CMD_KEYED; re-stamp the epoch from the (M6-requeue-rolled) bsi so any
				// pre-break timeline stamp reads STALE. Write-only; no consumer reads it.
				lp_note_break(cmd_batch_seq_id);
				emergency_previous_config = current_configuration;
				emergency_break_active = 1;
				emergency_break_retries = 3;
				// SACK Design A Step 12 — BREAK supremacy (§4.3.4 invariant #6).
				if(sack_v2_enabled)
					policy_axis1_supremacy_on_move(current_configuration,
						current_configuration, "break_block_failure_threshold");
				send_break_pattern();
				if(!lp_note_break_recovery(cmd_batch_seq_id))
				{
					fprintf(stderr, "[LP-CONTRACT] BREAK recovery transition rejected; "
						"listener remains closed\n");
					return;
				}
				// Poll for ACK from responder
				telecom_system->data_container.frames_to_read = 4;
				calculate_receiving_timeout();
				receiving_timer.start();
				return;
			}
			// STAGE 4c — the commander-side TRUE-SESSION-LOSS floor (design §5.4 / §7).
			// The BREAK guard above is FALSE because the link is already at the ladder
			// bottom (config_is_at_bottom) — there is no rung left to tag-demote to. Under
			// inband this is the ONLY place a commander BREAK is permitted: tick the
			// dead-batch streak and fire send_break_pattern() ONLY when it reaches
			// SESSION_DEAD_BATCHES (the SAME MERCURY_INBAND_DEAD_BATCHES the RX terminal
			// floor uses). When the feature is OFF this branch is inert (the OFF path never
			// reaches here under inband semantics — config_is_at_bottom OFF just retransmits,
			// byte-identical). Gated on inband + the same BREAK preconditions so non-inband
			// at-bottom operation is unchanged.
			else if(inband_rate_feature_enabled()
			        && emergency_nack_count >= emergency_nack_threshold
			        && config_is_at_bottom(current_configuration, robust_enabled)
			        && !emergency_break_active
			        && turboshift_phase == TURBO_DONE
			        && gear_shift_on == YES)
			{
				if(inband_cmd_dead_batch_floor_reached())
				{
					// Genuinely dead link at the floor — the ONE permitted BREAK (design §7).
					emergency_previous_config = current_configuration;
					emergency_break_active = 1;
					emergency_break_retries = 3;
					if(sack_v2_enabled)
						policy_axis1_supremacy_on_move(current_configuration,
							current_configuration, "inband_session_dead_batches_floor");
					send_break_pattern();
					telecom_system->data_container.frames_to_read = 4;
					calculate_receiving_timeout();
					receiving_timer.start();
					return;
				}
				// Below the floor: keep retrying at the bottom rung (no BREAK yet).
			}
		}
		else
		{
			if(duty_trace_enabled())
			{
				printf("[DUTY-TRACE] data-batch-done turboshift_active=%d turboshift_phase=%d "
					"break_recovery_phase=%d data_ack_received=%d snr_uplink=%.1f "
					"current=%d negotiated=%d\n",
					(int)turboshift_active, (int)turboshift_phase, break_recovery_phase,
					data_ack_received, measurements.SNR_uplink, current_configuration,
					negotiated_configuration);
				fflush(stdout);
			}
			// data_ack_received==YES (clean OR partial). emergency_nack_count tracks
			// CONSECUTIVE TOTAL block failures (threshold 3 at :3334); any delivery —
			// even a partial — breaks that streak, so it resets UNGATED. (§9.7.)
			emergency_nack_count = 0;  // Reset on success
			l1_aggregate_defer_streak = 0;  // deferred aggregate arrived (clean/partial) -> recovery, disarm the hold
			emergency_ackmiss_defer_streak = 0;  // reverse ACK decoded -> recovery, re-arm the sub-threshold hold
			if(linkphase_slotliveness_on())
			{
				missed_ack_slots = 0;
				linkphase_ack_slot_wait_armed = false;
				linkphase_ack_slot_miss_counted = false;
			}
			// LINK-PHASE PRIMITIVE (increment 1): the reverse ACK decoded / the batch delivered
			// -> the CMD's slot closed, ownership back to TURNAROUND. Write-only; no consumer.
			lp_note_ack_decoded();
			// STAGE 4c: any delivery (clean OR partial) proves the link is NOT dead, so
			// the commander true-loss floor streak resets UNGATED (symmetric with
			// emergency_nack_count). Keeps the SESSION_DEAD_BATCHES BREAK reachable ONLY
			// for a genuinely dead link.
			cmd_inband_session_dead_batches = 0;

			// WALL-B FIX-9 D3 (producer P3): ANY data-ACK (clean OR partial) proves the reverse
			// channel is NOT pure-silent, so the CFG16 reverse-ACK starvation streak resets UNGATED.
			// (A partial SACK that arrives is itself a reverse-ACK that decoded — the de-alignment
			// has not yet starved the turnaround to silence.) Symmetric with emergency_nack_count.
			cfg16_revack_starve_fails = 0;

			// WALL-B FIX-5: CLEAR the CFG16 big-block carve cooldown on CARVE SUCCESS. A
			// data-ACK while current_configuration==CONFIG_16 with big-block framing live
			// means a real CFG16 carve LANDED (the RSP decoded a big-block codeword and
			// data-ACKed), so the rung is viable again on THIS channel -> re-enable CFG16.
			// INV-B3 (the single most load-bearing gate, audit R2): this MUST be gated on
			// CFG16 + framing-live — a per-frame CFG15 data-ACK (which the CMD gets CONSTANTLY
			// during the productive cooldown window) must NOT clear it, else the fix becomes a
			// no-op and CFG16 re-opens immediately. The clear also resets the AARF span so the
			// NEXT genuine fade is treated fresh (mirrors break_drop_step_after_handshake_decay
			// decay-on-progress). See fix5/FIX5_DESIGN.md §4.5 + the §5 audit family-B.
			if(bigblock_carve_cooldown_batches > 0
			   && current_configuration == CONFIG_16
			   && telecom_system != NULL
			   && telecom_system->bigblock_framing_enabled
			   && telecom_system->M != MOD_MFSK)
			{
				printf("[CFG16-HOLD] FIX-5 CLEAR: CFG16 big-block carve LANDED (data-ACK at "
					"config 16, framing live) -> cooldown cleared (%d->0), CFG16 re-enabled\n",
					bigblock_carve_cooldown_batches);
				fflush(stdout);
				bigblock_carve_cooldown_batches = 0;
				bigblock_carve_cooldown_span = 0;
			}

			// ── PROACTIVE SNR-TRACK DEMOTE (MERCURY_SNR_TRACK, default-off) ─────────────────
			// Fade-tracking DOWN-shift. The link is ALIVE this batch (it produced a data-ACK,
			// clean or partial) and the freshly-latched climb-grade suffix meter now reads BELOW
			// the CURRENT rung's measured-viability floor — the SNR has dropped under the rung we
			// are airing. The incumbent ladder NEVER demotes here: rung_floor_ok() shortcuts any
			// at/below-current move and every real demote is failure-triggered, so it rides the
			// failing rung into the trough and BREAKs ~8 s late. Demote NOW, before the frames
			// fail, to the highest rung the live meter still clears (apply_rung_floor_raise_cap
			// walks CURRENT down to it — never into a needless stall). Route through the shipped
			// in-band CONFIG_TAG demote transport (inband_route_failure_demote — the SAME ~0.76 s
			// unilateral tag the failure demotes use; legacy routes it via the fast SET_CONFIG
			// handshake) but pin_ceiling=false: a precautionary SNR move is not a proven BREAK, so
			// a recovered meter must be free to climb back (the RUNG_MIN_SNR_METER floor, not a
			// pinned ceiling, guards over-climb). WB commander, FRESH meter, gated OFDM rung only
			// (cfg<CONFIG_9 floors are -90 -> rung_floor_meter_clears is always true there -> this
			// never fires). Off-flag: byte-identical (the whole block is behind the gate).
			if(snr_track_feature_enabled())
				snr_track_note_meter_batch(
					promotion_allowed_on_batch(last_batch_fully_acked));
			if(snr_track_feature_enabled()
			   && role == COMMANDER
			   && narrowband_enabled != YES
			   && gear_shift_on == YES
			   && is_ofdm_config(current_configuration)
			   && !rung_meter_stale()
			   && !rung_floor_meter_clears(current_configuration))
			{
				int snr_demote_target = apply_rung_floor_raise_cap(current_configuration);
				if(snr_demote_target >= 0
				   && config_ladder_index(snr_demote_target)
				      < config_ladder_index(current_configuration))
				{
					printf("[GEARSHIFT] SNR-TRACK proactive demote: live meter %.1f (hold %.1f) "
						"below config %d floor %.1f -> demoting %d->%d BEFORE loss "
						"(cause=snr, not failure)\n",
						rung_meter_db, rung_meter_hold_db, current_configuration,
						rung_floor_min_meter(current_configuration),
						current_configuration, snr_demote_target);
					fflush(stdout);
					snr_track_meter_window_reset();
					if(inband_route_failure_demote(snr_demote_target, "snr_track_floor",
						/*pin_ceiling=*/false))
						return;   // batch re-staged at the demoted rung; skip the climb this cycle
				}
			}

			// CLEAN-BATCH VIABILITY (§9): the panic/aggression resets AND the
			// data-viable anchor-raise are PROMOTION decisions — they must fire ONLY
			// on a CLEAN, fully-delivered batch. A genuine PARTIAL SACK keeps the link
			// alive (retransmit is unchanged) but must NOT (a) clear the BREAK panic
			// counter — otherwise a marginal CONFIG_0 that passes 1/25 frames forever
			// prevents BREAK from ever latching to ROBUST_0 — nor (b) raise
			// last_data_viable_config, which would pin the BREAK floor + re-probe
			// target at the marginal rung (the CONFIG_0 ↔ ROBUST_0 oscillation at the
			// WGN:-10 cliff). See gearshift-start-and-recovery.md §9.
			if(promotion_allowed_on_batch(last_batch_fully_acked))
			{
				break_drop_step = 2;       // Reset to initial aggression (2 steps).
				breaks_since_last_data_success = 0;  // panic-mode counter resets on
				                                     // CLEAN data flow (see arq.h / §9).
				// DEEP-SNR DOWN-HYSTERESIS (gearshift-climb-engine.md §10): a CLEAN
				// batch proves the anchor rung recovered, so clear the anchor-rung
				// BREAK streak (piece A's reset producer). This is what makes the
				// demotion require K *consecutive* anchor-rung BREAKs with NO clean
				// in between — a rung that recovers between failures is not demoted.
				anchor_consec_break_fails = 0;
				// PER-RUNG FAILURE MEMORY (audit §4): a CLEAN batch resets the live rung's fail streak
				// and, if we are sustaining delivery below a disproven rung, decays its floor bump
				// (cooldown). INDEPENDENT of the breaks_since_last_data_success reset above, so a slow
				// completion at the anchor cannot wipe a disproof (erasure site 2 neutralized).
				rung_floor_note_clean(current_configuration);
				// V3 first-data latch: a clean data batch has been ACKed this session, so subsequent
				// suffix-meter reads are in-transfer-grade (the connect-regime ~34 dB reads are behind
				// us). Enables rung_meter_note_read to latch the climb-grade snapshot from here on.
				rung_meter_data_latched = true;
				// B2 (default-off consumer): a clean fully-acked forward batch. Extend the healthy-
				// hold streak when the previous batch also confirmed with no gap (clean_age<=1); a
				// gap restarts it at 1. Reset the clean-batch clock to 0. Read ONLY by the healthy-
				// hold freshness exemption in rung_floor_meter_clears (cal-v3 ignores both counters).
				if(rung_meter_clean_age <= 1)
				{
					if(rung_meter_healthy_hold_batches < 1000000000) rung_meter_healthy_hold_batches++;
				}
				else
				{
					rung_meter_healthy_hold_batches = 1;
				}
				rung_meter_clean_age = 0;
				// AARF DECAY (long-run-degradation.md §2.3): frame_shift_threshold
				// is the FRAME-UP gate denominator (:3869). It is multiplicatively
				// INCREASED (*=2 at :2447/:3352/:3539) on every FRAME-UP failure but
				// -- until now -- had NO decrease half anywhere in the live path, so a
				// run of OFDM-crossing failures marched it 3->6->12->24->... and the
				// gate consecutive_data_acks >= eff_frame_shift_threshold became
				// permanently unsatisfiable, walling off the re-climb and PARKING at
				// the highest MFSK rung (the long-run "stuck-at-ROBUST" latch). AARF
				// is multiplicative-increase / multiplicative-DECREASE; the decrease
				// half was missing. A CLEAN, fully-delivered batch is proof the
				// channel improved, so HALVE the threshold back toward its base floor
				// (3, arq_common.cc:702) here, alongside the sibling recovery resets
				// (break_drop_step, anchor_consec_break_fails). This is
				// decay-on-SUCCESS, NOT a blanket reset: a genuinely-bad channel keeps
				// failing FRAME-UP and never reaches this clean-batch branch, so the
				// doubling still suppresses over-climb on a bad channel; only sustained
				// clean delivery relaxes the gate. Gated inside
				// promotion_allowed_on_batch() so a partial SACK never decays it.
				if(frame_shift_threshold > 3)
				{
					frame_shift_threshold /= 2;
					if(frame_shift_threshold < 3)
						frame_shift_threshold = 3;
					printf("[GEARSHIFT] AARF decay: clean batch at config %d, frame_shift_threshold now %d\n",
						current_configuration, frame_shift_threshold);
					fflush(stdout);
				}
				// SUSTAINED-ANCHOR GATE (gearshift-climb-engine.md §11): track the run
				// of CONSECUTIVE clean batches AT THIS RUNG. A clean at a NEW rung
				// (config != the streak's rung) starts the count fresh at 1; otherwise
				// it extends. This is the input the anchor-raise gate below consumes.
				if(current_configuration != clean_batches_config)
				{
					clean_batches_config = current_configuration;
					clean_batches_at_current_config = 1;
				}
				else if(clean_batches_at_current_config < 1000000)  // saturate, no overflow
				{
					clean_batches_at_current_config++;
				}
				// [L1-GEARFEED] credit the N-1 silent intermediate batches to the
				// sustained-anchor streak so a clean block seats the anchor as N
				// per-batch clean batches would (0 off/unarmed/not-all-received).
				clean_batches_at_current_config += l1_gearfeed_extra_batches();
				// Option B (data-anchored promotion): a DATA batch was confirmed
				// FULLY delivered at this config. Record it as the highest data-viable
				// rung — anchors BREAK recovery (arq_commander.cc:81) and the
				// up-shifter gates. SOLE on-delivery producer. A failed probe / partial
				// batch never reaches here. See gearshift-start-and-recovery.md §6/§7/§9.
				//
				// SUSTAINED-ANCHOR GATE (§11): close the WGN:-10 thrash leak at its
				// source. The anchor RAISE is now gated on N CONSECUTIVE clean batches
				// at this rung — robust=1 (one clean MFSK frame is strong proof; keep
				// the off-ROBUST_0 climb fast), OFDM=2 (a single SACK-retransmit-
				// rescued batch can't anchor a non-sustainable rung — that single
				// prev-path completion ACK is exactly what raised the CONFIG_0 anchor
				// and started the thrash). A rung that REPEATEDLY delivers clean still
				// promotes (the count crosses the threshold on the 2nd clean). We do
				// NOT neuter the prev-path completion ACK itself — it keeps the link
				// alive on a genuinely-good-but-lossy channel; "retransmit-rescued ≠
				// viable" is encoded HERE in the CMD anchor gate.
				//
				// ANCHOR-TIER-CROSSING DISCIPLINE (gearshift-climb-engine.md §16,
				// ROOT-1): the raise now goes through the PURE
				// data_anchor_raise_target() helper keyed on clean_batches_config (the
				// config the clean STREAK accumulated at — the authoritative "delivered
				// config"), NOT the live current_configuration. The helper (a) keeps the
				// §11 sustained-N gate, (b) only ever raises, and (c) ADDS the §16 TIER
				// GATE: a ROBUST-anchor -> OFDM-tier crossing is licensed ONLY when the
				// streak accumulated at an OFDM config. A ROBUST-tier clean ACK therefore
				// can NEVER poison the anchor into the OFDM tier — even if a same-/cross-
				// pass advance had moved current_configuration to CONFIG_0 before this
				// robust credit fired (the pre-§16 hole that unlocked the §15 elevator +
				// the §16 ROOT-2 turbo ladder at WGN:-10). clean_batches_config == the
				// streak's home; §11 pins it to current_configuration on the same clean
				// that updated the streak just above, so on the steady-state data path it
				// equals current_configuration — the change BITES only when the live
				// config has advanced ahead of the streak (the corruption scenario).
				// ELEVATOR FAST-CONFIRM (climb-duty, gearshift-elevator-dwell.md): on a whole-clean
				// OFDM batch whose RAW SNR capacity clears a higher rung, raise the anchor to THIS
				// rung on the FIRST clean (boost the anchor-raise streak to the N=2 bar). The
				// elevator leap needs an OFDM anchor (supershift_retrigger_target's is_ofdm_config
				// (anchor) gate) and the +1 clamp needs the anchor at this rung; both are satisfied
				// 1 batch sooner. This reaches the SAME anchor progression the incumbent reaches at
				// N=2 (the incumbent also seats the anchor at each rung before leaping) — just
				// sooner — so BREAK-recovery behaviour is unchanged. The §16 TIER GATE inside
				// data_anchor_raise_target still applies (only an OFDM streak_config crosses).
				// Off / marginal (no raw-SNR headroom, or retx>0) -> streak unchanged -> incumbent N=2.
				int eff_anchor_streak = clean_batches_at_current_config;
				if(elevator_fast_confirm_eligible() && eff_anchor_streak < SUSTAINED_ANCHOR_N_OFDM)
					eff_anchor_streak = SUSTAINED_ANCHOR_N_OFDM;
				last_data_viable_config = data_anchor_raise_target(
					clean_batches_config, current_configuration,
					last_data_viable_config, eff_anchor_streak);
				// INBAND CEILING RE-RAISE (data-flow-inband-ceiling-reraise.md §3,
				// option (b)): under the no-BREAK inband redesign the demote
				// (inband_route_failure_demote, :3073) PINS supershift_proven_ceiling
				// at the demoted rung, and the turbo/SUPERSHIFT/BREAK re-raise sites
				// never fire on the inband path — so the FRAME-UP gate (:5340) walls
				// ALL upward probing after a degrade (the deep_stall). Mirror the
				// anchor raise above: once this (demoted) rung has re-proven itself
				// sustained-clean, RAISE the ceiling to TRACK the freshly-updated
				// proven anchor (never above it). The +1 anchor clamp (:5348-5350)
				// then permits a single-rung probe above proven ground, and the
				// gearshift's decode-failure demote re-pins if that probe fails —
				// restoring re-climb without over-climb. SCOPED to inband only so
				// legacy is BYTE-IDENTICAL (the helper is a no-op when ceiling<0, but
				// the gate makes the intent explicit and guarantees the legacy
				// turbo/BREAK ceiling discipline is untouched). PURE helper; replayed
				// by --test-climb-engine Part X.
				if(inband_rate_feature_enabled())
					supershift_proven_ceiling = inband_ceiling_raise_target(
						supershift_proven_ceiling, last_data_viable_config,
						clean_batches_config, clean_batches_at_current_config);
				// FIX-B — RESET the floor-probe back-off on a CLEAN OFDM batch
				// (gearshift-floor-probe-backoff.md §5.3). This is the sole reset
				// producer: a fully-delivered batch at an OFDM config proves an OFDM
				// rung recovered, so no rung is "proven-failed" anymore — clear every
				// per-rung deadline and return the window to INIT. Gated on
				// is_ofdm_config(current_configuration): a clean ROBUST-tier batch does
				// NOT lift the back-off (the failed up-probe rung is OFDM; only OFDM
				// success is evidence the boundary improved). Inside the
				// promotion_allowed_on_batch() clean branch, so a partial SACK never
				// resets the back-off.
				if(is_ofdm_config(current_configuration))
					probe_backoff_reset();
			}
			// ROLLING-PARTIAL ANCHOR RAISE (data-flow-inband-frame0-rolling-partial.md §2,
			// the §5 sibling). The clean-batch branch above did NOT fire (the rolling CONFIG_0
			// batch is a LEAD-FRAME-ONLY partial, not a true clean), so last_data_viable_config
			// stays at the ROBUST tier (idx 2). The FRAME-UP +1 anchor clamp (:5510) then BLOCKS
			// CONFIG_0(idx3)→CONFIG_1(idx4) because idx4 > anchor_idx(2)+1 — the streak fix alone
			// cannot climb. ROOT: a lead-frame-only batch IS eventually FULLY delivered (every
			// frame, frame-0 via retx within one batch), so the rung carries full data and is a
			// valid anchor. Raise the anchor to THIS rung (and track the ceiling) so the +1 clamp
			// permits the next-rung probe. NARROW: this is the ONLY clean-only producer extended;
			// the panic/AARF/back-off resets above stay true-clean-gated (a multi-drop partial
			// leaves last_partial_lead_frame_only false → this block is skipped → §9 anti-thrash
			// for a genuinely marginal rung is untouched). The +1 clamp still bounds the climb to
			// one rung above proven ground, and the gearshift's decode-failure demote re-pins if
			// the next rung fails — restoring re-climb without over-climb. Inband + OFDM only.
			//
			// DEFER-WHILE-HOLE-OUTSTANDING (data-flow-inband-frame0-rolling-partial.md §10): the
			// lead-frame-only partial just ENQUEUED the missing frame-0 for retx
			// (retransmit_count > 0; arq_commander.cc:4106 / the MFSK big block). Raising the anchor
			// here lets the FRAME-UP +1 clamp permit the next-rung probe, and the resulting
			// config-change calls clear_retx_queue() (:5708) which would ABANDON that outstanding
			// frame-0 → bsi gap → RSP-V2-GAP-ABORT wedge (the 2801d7c sibling). So DEFER the anchor
			// raise while the hole is outstanding; the mixbatch drains the retx within one batch
			// (:1919), and the NEXT (whole) batch — retransmit_count==0 — raises the anchor then.
			// inband_lead_frame_only_partial() still recorded the rung viability; only the
			// orphaning anchor-raise is held. A genuinely whole batch never has an outstanding hole,
			// so the clean-batch anchor-raise above is unaffected (byte-identical when off).
			else if(inband_lead_frame_only_partial() && !inband_climb_hole_outstanding())
			{
				if(current_configuration != clean_batches_config)
				{
					clean_batches_config = current_configuration;
					clean_batches_at_current_config = 1;
				}
				else if(clean_batches_at_current_config < 1000000)
					clean_batches_at_current_config++;
				int prev_anchor = last_data_viable_config;
				last_data_viable_config = data_anchor_raise_target(
					clean_batches_config, current_configuration,
					last_data_viable_config, clean_batches_at_current_config);
				supershift_proven_ceiling = inband_ceiling_raise_target(
					supershift_proven_ceiling, last_data_viable_config,
					clean_batches_config, clean_batches_at_current_config);
				if(last_data_viable_config != prev_anchor)
				{
					printf("[GEARSHIFT] ROLLING-PARTIAL anchor raise: lead-frame-only batch fully "
						"delivered at config %d -> last_data_viable %d->%d (FRAME-UP +1 clamp now "
						"permits the next-rung probe)\n",
						current_configuration, prev_anchor, last_data_viable_config);
					fflush(stdout);
				}
			}
			// Don't reset ceiling_success_count here — it accumulates across blocks
			frame_gearshift_just_applied = false;  // upshift survived — clear flag
			frame_gearshift_retry_count = 0;       // §7.13.33 reset
		}

		// Mirror the responder's post-send watermark only after this side has
		// validated delivery of a complete clean DATA batch.
		if(data_ack_received == YES && last_batch_fully_acked)
			t1_data_phase_proven = true;

		if(data_ack_received == YES)
			scream_link_timeout_grace_used = false;

		if(data_ack_received == YES && scream_resume_pending)
		{
			scream_resume_pending = false;
			scream_resume_count++;
			printf("[SCREAM-RESUME] count=%ld first post-wake data ACK cfg=%d "
				"acked_frames=%d tx_fifo_pending=%d\n",
				scream_resume_count, current_configuration,
				stats.nAcked_data,
				fifo_buffer_tx.get_size() - fifo_buffer_tx.get_free_size());
			fflush(stdout);
		}

		// Auto-arm compression after B2F SID is ACKed
		if(data_ack_received==YES && b2f_compression_pending)
		{
			compression_enabled = true;
			b2f_compression_pending = false;
			printf("[COMPRESS] Armed after B2F ACK (commander)\n");
			fflush(stdout);
			// Streaming compression is unconditional (CAP_STREAMING removed).
			if(!compressor.is_streaming())
				compressor.streaming_enable();
		}

		// Adaptive batch growth disabled: CMD batch growth causes desync with RSP.
		// RSP's data_batch_size stays at the negotiated value, so CMD sending more
		// frames than RSP expects corrupts slot state and triggers ACK-GATE suppression.
		// TODO: communicate batch size to RSP (e.g., in frame header) before re-enabling.
		if(data_ack_received == YES && turboshift_phase == TURBO_DONE
			&& !is_robust_config(current_configuration))
		{
			batch_consec_acks++;
		}
		else if(data_ack_received == NO && turboshift_phase == TURBO_DONE
			&& !is_robust_config(current_configuration))
		{
			batch_consec_acks = 0;
		}

		// TEST-ONLY, default-off (silent-corruption-residual.md §7 / data-flow-batch-size.md
		// §9 LIVE fire proof): reproduce the res_c3100 CMD>RSP batch-size desync on the wire.
		// After the first clean OFDM batch (so the RSP has settled at its config default), force
		// the CMD to build MERCURY_BATCHSIZE_DESYNC frames per batch WITHOUT sending a
		// SET_LINK_PARAMS — i.e. exactly the observed state of a lost/unapplied Axis-2 step-up
		// (CMD=30, RSP=25). The RSP (no knob) stays at its default, so its wire D5 count exceeds
		// its own data_batch_size and Option B' must WIDEN to deliver the batch in full. Fires
		// ONCE (the grow makes the guard false thereafter). Grow-only (never shrinks), OFDM-only.
		{
			const char* e = std::getenv("MERCURY_BATCHSIZE_DESYNC");
			if(e && *e && atoi(e) > 0
			   && !is_robust_config(current_configuration)
			   && batch_consec_acks >= 1
			   && data_batch_size >= 20          // only at a settled WB batch (~cfg16, batch 25) — not mid-climb
			   && data_batch_size < atoi(e))
			{
				int forced = atoi(e);
				if(forced > MAX_SACK_BATCH_SIZE) forced = MAX_SACK_BATCH_SIZE;
				printf("[TEST-BATCHSIZE-DESYNC-FORCE] CMD data_batch_size %d -> %d "
					"(no SET_LINK_PARAMS emitted; RSP stays at its config default = the res_c3100 desync)\n",
					data_batch_size, forced);
				fflush(stdout);
				set_data_batch_size(forced);
				recalculate_ack_timeout_for_batch();
			}
		}

		// Frame-level gearshift: after N consecutive successful data ACKs, shift up immediately
		// Respect proven ceiling — don't re-try configs above what turboshift/BREAK verified.
		{
			int proposed_frame = config_ladder_up(current_configuration, robust_enabled, narrowband_enabled == YES);
			bool frame_ceiling_blocked = (supershift_proven_ceiling >= 0 &&
				config_ladder_index(proposed_frame) > config_ladder_index(supershift_proven_ceiling))
				|| (max_config_override >= 0 && proposed_frame > max_config_override);
			// Option B (data-anchored promotion): never leap more than +1 rung
			// past the highest data-viable rung. The Q-table owns CONFIG_6+ and is
			// exempt (optimizer_is_in_control()). Since FRAME-UP advances exactly
			// one rung, this clamps it to last_data_viable_config+1 — it can probe
			// the next rung but never skip ahead of unproven ground. See §6/§7.
			if(!optimizer_is_in_control() &&
			   config_ladder_index(proposed_frame) > config_ladder_index(last_data_viable_config) + 1)
				frame_ceiling_blocked = true;
			// FIX-B — floor-probe back-off (gearshift-floor-probe-backoff.md §5.2,
			// gate-site #1). AND the per-rung suppression into the EXISTING up gate:
			// a proven-failed up-probe is not re-hammered until its back-off elapses
			// (or a clean OFDM batch resets it). INV-1: this only turns a PERMITTED
			// probe OFF — it never unblocks a probe the anchor/+1/ceiling clamps above
			// already blocked. INV-2: this is the UP path only; no BREAK/demote/panic
			// (downward) site references the back-off, so the deep-SNR escape is intact.
			if(probe_rung_suppressed(proposed_frame))
				frame_ceiling_blocked = true;
		// Handoff: above the lowest calibrated Q-table cell, the optimizer
		// is the sole authority for upward config changes. Gearshift's
		// FRAME UP must yield. Downward moves (BREAK) remain available.
		bool optimizer_owns_upward_frame = optimizer_is_in_control();
		// CLEAN-BATCH VIABILITY (§9): only a CLEAN, fully-delivered batch counts
		// toward the FRAME-UP climb. A partial SACK (data_ack_received==YES but
		// last_batch_fully_acked==false) neither advances nor resets
		// consecutive_data_acks — it is link-keepalive, not proof the rung carries
		// full data. Without this gate a string of partials at a marginal rung
		// climbs into a config that can't pass data. See §9.
		//
		// PIPELINE-THE-CLIMB (inband-reliability-design.md §1.8): the strict
		// clean-batch gate SERIALIZES the redesign's intra-OFDM climb at ~12.4s/rung
		// — while a climb's re-tag is armed the climbed-to rung needs a full slow
		// data-SACK round-trip to produce a CLEAN batch, so the next rung cannot fire
		// until that confirm lands (legacy climbs fast via a decoupled SET_CONFIG
		// control ACK; the redesign pins low). So: when a CLIMB re-tag is armed under
		// the inband feature (inband_pipeline_climb_active), advance OPTIMISTICALLY on
		// a FORWARD-HEALTHY data ACK (data_ack_received==YES, partial or clean) without
		// waiting for the clean fully-acked confirm — the climb then pipelines and a
		// single trailing SACK confirms the whole ramp. The overshoot net
		// (inband_retag_escalate_if_climb_exhausted) recovers a too-eager climb to the
		// last-confirmed floor, so §9's "climb into a config that can't pass data" is
		// RECOVERABLE here (it was not under legacy — legacy keeps the strict gate,
		// byte-identical: inband_pipeline_climb_active() returns false when off).
		bool inband_climb_pipeline = inband_pipeline_climb_active();
		// ROLLING-PARTIAL unblock (data-flow-inband-frame0-rolling-partial.md §1/§2): at
		// CONFIG_0 (and any inband OFDM rung entered via the SET_CONFIG tier-cross, where NO
		// re-tag is armed so inband_climb_pipeline is false) the FIRST OFDM frame of each batch
		// bears the Schmidl-Cox acquisition burden and can fail the pre-LDPC SKIP-VAR gate while
		// frames 1..N-1 ride the locked timing → a rolling 5/6 LEAD-FRAME-ONLY partial the retx
		// recovers within one batch. Such a batch is a VIABLE rung: ADVANCE the climb streak on
		// it (optimistic, like the pipeline-climb path) so FRAME-UP can fire CONFIG_0→1 instead
		// of wedging on the rolling partial. inband_lead_frame_only_partial() is OFDM-only +
		// gated on the lead-frame-only bitmap signature, so a MULTI-frame-drop partial (a
		// genuinely marginal rung) stays vetoed (§9 anti-thrash) and legacy/flag-off is
		// byte-identical (the predicate returns false). The retx still recovers frame-0 (the
		// SACK machinery is UNCHANGED), and the +1 anchor clamp below still bounds the climb.
		bool inband_lead_frame_partial = inband_lead_frame_only_partial();
		bool batch_promotable = (inband_climb_pipeline || inband_lead_frame_partial)
			? (data_ack_received==YES)
			: promotion_allowed_on_batch(last_batch_fully_acked);
		if(data_ack_received==YES && batch_promotable &&
			gear_shift_on==YES && gear_shift_algorithm==SUCCESS_BASED_LADDER &&
			messages_control.status==FREE &&
			!config_is_at_top(current_configuration, robust_enabled, narrowband_enabled == YES) &&
			!frame_ceiling_blocked &&
			!optimizer_owns_upward_frame)
		{
			consecutive_data_acks++;
			// [L1-GEARFEED] credit the N-1 silent intermediate batches to the
			// FRAME-UP streak so a clean block advances the climb as N per-batch
			// clean batches would (0 off/unarmed/not-all-received).
			consecutive_data_acks += l1_gearfeed_extra_batches();
			// ADAPTIVE FRAME-UP THRESHOLD (gearshift-climb-engine.md §12, Option 3,
			// climb follow-up ③): step up FASTER when the channel is PROVEN
			// sustained-clean at this rung, conservative when marginal. The EFFECTIVE
			// threshold is read-time only — it never mutates frame_shift_threshold,
			// so the AARF back-off (:3180/:3359/:2302, ×2 on FRAME-UP failure) is
			// preserved and the reduction can't fight it. fast-probe (→1) arms ONLY
			// once clean_batches_at_current_config crosses the rung's anchor-viability
			// bar (fast_probe_clean_streak); a failed block resets that streak to 0
			// (:3172/:3307), so at the deep-SNR cliff this stays at the conservative
			// (possibly doubled) member → #2's WGN:-10 anti-thrash (e3d818d) intact.
			// This changes only HOW MANY clean batches trigger a +1 — never how far a
			// +1 reaches (the +1 anchor clamp above is untouched). See §12.1/§12.3.
			int eff_frame_shift_threshold = effective_frame_shift_threshold(
				frame_shift_threshold, current_configuration,
				clean_batches_at_current_config);
			// ELEVATOR FAST-CONFIRM (climb-duty): the anchor-raise above already seated this rung
			// as the OFDM anchor on the FIRST clean margin batch (clearing the +1 clamp and
			// un-capping the elevator target). Fire the FRAME-UP elevator on THIS first clean by
			// arming the fast threshold, instead of waiting for the 2nd clean (the N=2 fast-probe
			// bar). Off / marginal / defeated -> byte-identical N=2.
			if(elevator_fast_confirm_eligible())
			{
				eff_frame_shift_threshold = FRAME_SHIFT_FAST;
				printf("[GEARSHIFT] ELEVATOR FAST-CONFIRM: 1-batch climb armed at config %d "
					"(whole-clean retx=0, SNR capacity clears a higher rung) -> firing FRAME-UP on "
					"the first clean batch (skip the 2nd confirm)\n", current_configuration);
				fflush(stdout);
			}
			// SNR-TRACK fast up-climb (MERCURY_SNR_TRACK, default-off): ride the recovery up
			// only after a fresh read is confirmed by another read or a clean held batch. The
			// rolling fresh-read minimum and peak margin cap the jump.
			// Off-flag: byte-identical.
			int snr_track_climb_cap = snr_track_feature_enabled()
				? snr_track_sustained_target() : CONFIG_NONE;
			if(snr_track_feature_enabled()
			   && snr_track_climb_cap != CONFIG_NONE
			   && config_ladder_index(snr_track_climb_cap)
			      > config_ladder_index(current_configuration))
			{
				eff_frame_shift_threshold = FRAME_SHIFT_FAST;
				printf("[GEARSHIFT] SNR-TRACK sustained-confirm: %d fresh batches at config %d "
					"(meter %.1f, cap %d) -> firing FRAME-UP\n",
					snr_track_fresh_batches, current_configuration, rung_meter_db,
					snr_track_climb_cap);
				fflush(stdout);
			}
			// DEFER-WHILE-HOLE-OUTSTANDING (data-flow-inband-frame0-rolling-partial.md §10):
			// the streak CREDIT (consecutive_data_acks++) above is UNCONDITIONAL — 2801d7c's
			// purpose (the lead-frame-only partial still builds the climb streak) is preserved.
			// But the config-change FIRE below calls clear_retx_queue() (:5708), which ABANDONS
			// the frame-0 the rolling partial just enqueued for retx (retransmit_count > 0) ->
			// bsi gap -> RSP-V2-GAP-ABORT wedge (the 2801d7c sibling). So when the in-band path
			// still has an OUTSTANDING retx hole, HOLD the fire: the already-built streak waits
			// while the mixbatch drains the retx (:1919, within one batch); the NEXT whole batch
			// (retransmit_count==0) lets the same streak fire the climb promptly. A CLEAN/WHOLE
			// batch has no outstanding hole -> inband_climb_hole_outstanding() is false -> the
			// climb fires immediately (2801d7c forward-climb-on-clean preserved). Legacy/flag-off:
			// the predicate is false -> byte-identical (this AND-term is always true off-flag).
			if(consecutive_data_acks >= eff_frame_shift_threshold &&
			   !inband_climb_hole_outstanding())
			{
				// CONTROLLED ELEVATOR from the data-anchored FRAME-UP path
				// (REAL FAST-PROBE piece B, gearshift-climb-engine.md §14). The
				// SUPERSHIFT re-trigger (:4587) is the only OTHER elevator site,
				// but on the unpinned +1 ladder turbo is TURBO_DONE and the
				// re-trigger's gap>=3 self-suppression often keeps it dormant. So
				// also fire the elevator HERE, INSIDE the clean-batch-CONFIRMED
				// branch (promotion_allowed_on_batch already gated this block):
				// instead of the unconditional +1 (proposed_frame), target the
				// SNR-ideal config when a high-confidence forward SNR is available.
				// REUSES ①'s machinery VERBATIM via the shared
				// elevator_target_from_snr() (same cap-chain + the
				// supershift_retrigger_target high-SNR gate). Gated on the SAME
				// preconditions as the re-trigger: gear_shift_on==YES, the live
				// config is OFDM, and SNR_uplink is populated (> -90). At deep SNR
				// (the WGN:-10 cliff) the >-90 gate is unmet OR the helper keeps
				// the conservative +1 -> BYTE-IDENTICAL to the current +1 ladder
				// (SAFETY #1). The elevator only RAISES the target above
				// proposed_frame; it never lowers below the +1 (it is an
				// elevator-OR-+1 max). The anchor is NOT raised here (the §1.1
				// confirmed-delivery producer is the sole anchor-raise; SAFETY #3),
				// and this path does NOT enter turbo (no SUPERSHIFT storm; SAFETY
				// #5). The rest of the FRAME-UP block (FIFO restore, the
				// add_message_control(SET_CONFIG), the state transition) is
				// UNCHANGED — the elevator just sets a higher negotiated_configuration
				// before the SAME SET_CONFIG goes out. See §13/§14.
				// IN-BAND +1 CLIMB (data-flow-inband-frame0-rolling-partial.md §7.2
				// option A — the REVERSE-SACK DELIVERY unblock). Under MERCURY_INBAND_RATE
				// the reverse data-SACK rides the robust-MFSK suffix (the d28f02d pin), and
				// the climb-CONFIRM rides the base ACK pattern (§6 keystone, decoupled). A
				// MULTI-RUNG elevator jump (CONFIG_0->3 at WGN:40, verified
				// _msab/keystone/_logsnap/modem_064839.log T+201.166 "FRAME UP config 0 -> 3")
				// keeps the CONFIG_0-SIZED 24-frame batch but airs it at the much slower
				// CONFIG_3 rung -> a giant forward batch whose reverse-SACK turnaround the
				// robust-MFSK suffix CRC cannot survive (CMD-ACK-PAT peak_matched=5/7,
				// suffix CRC fails) -> the data-SACK never applies -> stats.nAcked_data
				// STUCK at 14 -> BREAK at every rung. Legacy never hits this: it only airs
				// 24-frame batches at CONFIG_15/16 (short airtime), CONFIG_3/4 batches are
				// 1 frame, and its nAcked_data advances 0->15 (modem_032735.log).
				// FIX: on the in-band path climb EXACTLY ONE rung per FRAME-UP (keep
				// proposed_frame, suppress the elevator) so the CMD/RSP rungs stay aligned
				// and each rung's forward batch + reverse turnaround stays small enough for
				// the reliable reverse SACK to decode at the shared rung. The elevator only
				// RAISES; suppressing it never lowers below the +1, so the floor/anti-thrash
				// nets are untouched. Legacy/flag-off keeps the elevator (byte-identical).
				// The decision is the pure inband_climb_target() — production and the
				// directed regression (test_inband_plus1_climb) drive the EXACT same logic.
				int snr_elevator = -1;
				// The OFDM elevator may consume the relayed SNR_uplink only from an OFDM config
				// with a valid estimate (the tip triad). NOTE: the P0-gate SNR-provenance latch
				// was DROPPED in v2 -- it was an unsatisfiable bootstrap deadlock (no OFDM-
				// provenance SNR source exists at cfg0 on the forward climb, so the gate pinned
				// the climb at cfg2, a 12x regression). The anchor tier gate is_ofdm_config(anchor)
				// (arq.h:1441-1443) + the ceiling ratchet are the real over-climb safety. See
				// data-flow-gearshift-climb.md.
				if(gear_shift_on==YES && is_ofdm_config(current_configuration) &&
				   measurements.SNR_uplink > -90)
					snr_elevator = elevator_target_from_snr();
				// C1 (data-flow-gearshift-climb.md): ROBUST tier-cross probe. At a robust config
				// NO OFDM-calibrated SNR exists (the MFSK proxy is the known over-reader), so no
				// SNR may direct the target -- but visiting ROBUST_1/2 yields NO OFDM evidence
				// (same MFSK family) and is pure delay. robust_climb_probe_target() PROPOSES
				// CONFIG_0 directly (the only OFDM question a robust climb can ask is "does
				// CONFIG_0 decode?"). Raise it through the SAME snr_elevator slot so proposed_frame
				// stays the robust +1 and the +1 anchor clamp (:6578, keyed on proposed_frame)
				// still passes -- the raise happens INSIDE the block exactly as the OFDM
				// elevator's does (no clamp bypass). A wrong probe costs one batch + one BREAK,
				// once per session (break_target_with_anchor floors + the proven-ceiling ratchet
				// disables the re-probe).
				{
					int tier_cross = robust_climb_probe_target();
					if(tier_cross >= 0 &&
					   config_ladder_index(tier_cross) > config_ladder_index(snr_elevator))
						snr_elevator = tier_cross;
				}
				// SNR-TRACK capacity-cap fix (MERCURY_SNR_TRACK): the elevator's get_configuration(SNR
				// - SUPERSHIFT_MARGIN_DB) capacity estimate parked the climb well below the live-meter-
				// supported rung (the probe stuck it at cfg8 in a good window). RUNG_MIN_SNR_METER is the
				// MEASURED per-rung viability verdict, so let it LIFT the climb target to the highest rung
				// the FRESH meter clears (apply_rung_floor_raise_cap walks down from the WB ceiling),
				// still capped by a real-BREAK proven ceiling. This only RAISES toward a meter-cleared
				// rung (never above the floor -- that IS the guard); the inband +1 clamp below still steps
				// one rung/batch so CMD/RSP stay aligned (no reverse-SACK stranding), and legacy takes the
				// multi-rung jump via the fast SET_CONFIG. WB commander, fresh meter. Off-flag: byte-identical.
				if(snr_track_feature_enabled() && narrowband_enabled != YES
				   && is_ofdm_config(current_configuration)
				   && snr_track_climb_cap != CONFIG_NONE)
				{
					int meter_target = snr_track_climb_cap;
					if(supershift_proven_ceiling >= 0
					   && config_ladder_index(meter_target) > config_ladder_index(supershift_proven_ceiling))
						meter_target = supershift_proven_ceiling;
					snr_elevator = meter_target;
				}
#ifdef INBAND_PLUS1_CLIMB_FAILBEFORE
				bool inband_plus1_on = false;  // fails-before: elevator jump NOT suppressed
#else
				bool inband_plus1_on = inband_rate_feature_enabled();
#endif
				negotiated_configuration =
					inband_climb_target(proposed_frame, snr_elevator, inband_plus1_on);
				// CFG16 decode-margin gate: proposed_frame is the SNR-blind +1 ladder step, so a
				// 15->16 FRAME-UP can reach CFG16 on ACK success alone even when the elevator
				// (already gated) holds CFG15. Cap the FRAME-UP target so the top rung is elected
				// only when the reverse-path SNR supports it (never-raise; identity at high SNR).
				negotiated_configuration = apply_cfg16_margin_cap(negotiated_configuration);
				// Per-rung floor gate on the SNR-blind +1 FRAME-UP: a 13->14 ACK-success step can
				// reach a below-floor rung the (already-gated) elevator held; cap it (never-raise).
				negotiated_configuration = rung_floor_cap_fire(negotiated_configuration);
				// SNR tracking may climb only as far as the rolling fresh-meter window
				// supports. Until that window is complete, hold the current rung.
				if(snr_track_feature_enabled())
				{
					int sustained_cap = snr_track_climb_cap;
					if(sustained_cap == CONFIG_NONE)
						sustained_cap = config_ladder_index(current_configuration)
							< config_ladder_index(CONFIG_8) ? CONFIG_8 : current_configuration;
					if(config_ladder_index(negotiated_configuration)
					   > config_ladder_index(sustained_cap))
						negotiated_configuration = sustained_cap;
				}
				// NO-OP FRAME-UP SUPPRESSION (data-flow-gearshift-climb.md). The margin cap
				// (and the moderate-SNR CFG16 decode-margin gate) can pull the FRAME-UP target
				// back to the CURRENT config — a NO-OP "climb": proposed_frame was CFG15->16 but
				// the gate holds CFG15, so negotiated == current. The commit block below still
				// emits a FULL SET_CONFIG control round-trip (FIFO re-stage + clear_retx_queue +
				// control-ACK + re-send) that changes NOTHING on the wire, burning a whole batch
				// cycle EVERY eff_thresh clean batches while the channel sits below the CFG16
				// viability floor. When the negotiated forward is IDENTICAL to the live config AND
				// the reverse-ACK config is already established (reverse_configuration != CONFIG_NONE
				// — the SET_CONFIG builder's LADDER path leaves reverse UNCHANGED in that case, so
				// the frame would announce no forward AND no reverse change), SUPPRESS the redundant
				// round-trip: reset the streak and resume DATA, PRESERVING the retx queue (NO
				// clear_retx_queue — the in-flight batch stays valid at the unchanged config). A
				// REAL config change (negotiated != current, e.g. the high-SNR CFG15->16 election
				// once the margin gate opens, or any tier-cross) fails this guard and still commits
				// the SET_CONFIG below, so config-sync is intact. Defeat: MERCURY_NOOP_SETCONFIG_KEEP
				// restores the unconditional emit for A/B isolation.
				{
					static const bool noop_setconfig_keep =
						(std::getenv("MERCURY_NOOP_SETCONFIG_KEEP") != nullptr);
					if(!noop_setconfig_keep &&
					   negotiated_configuration == current_configuration &&
					   reverse_configuration != CONFIG_NONE)
					{
						printf("[GEARSHIFT] FRAME-UP NO-OP SUPPRESSED: negotiated config %d == "
							"current (margin-gated climb target), reverse=%d unchanged -> skip "
							"redundant SET_CONFIG round-trip, PRESERVE retx queue, resume DATA\n",
							negotiated_configuration, reverse_configuration);
						fflush(stdout);
						consecutive_data_acks = 0;
						connection_status = TRANSMITTING_DATA;
						return;
					}
				}
				printf("[GEARSHIFT] FRAME UP: %d consecutive ACKs (eff_thresh %d, base %d, clean-streak %d), config %d -> %d\n",
					consecutive_data_acks, eff_frame_shift_threshold, frame_shift_threshold,
					clean_batches_at_current_config, current_configuration, negotiated_configuration);
				fflush(stdout);
				if(snr_track_feature_enabled())
					snr_track_meter_window_reset();
				// Fire-proof diagnostic (data-flow-gearshift-climb.md): a greppable positive signal
				// that the FRAME-UP elevator/tier-cross produced a MULTI-RUNG leap (the negotiated
				// target outran the conservative +1 proposed_frame). The first attempt's gate failed
				// SILENTLY; this line makes the leap visible on the wire for the fire-proof.
				if(config_ladder_index(negotiated_configuration) > config_ladder_index(proposed_frame))
					printf("[GEARSHIFT] ELEVATOR leap: config %d -> %d (snr=%.1f, anchor=%d, elevator=%d)\n",
						current_configuration, negotiated_configuration, measurements.SNR_uplink,
						last_data_viable_config, snr_elevator);
				fflush(stdout);
				consecutive_data_acks = 0;

				// CLIMB-CHURN bsi rollback (data-flow-climb-up-bsi-rollback.md §5):
				// capture the in-flight batch's bsi BEFORE the FIFO-restore loop
				// below frees messages_tx[], then roll cmd_batch_seq_id back to it so
				// the re-encoded batch re-presents CONTIGUOUSLY with the RSP's
				// delivery high-water (no >=2 gap-gate HOLD on a rapid mid-transfer
				// climb). No-op off the v2 in-order path (gated inside the helper).
				roll_back_cmd_bsi_to_inflight("GEARSHIFT");

				// Put all pending data back into TX FIFO for re-encoding at new config
				if(compression_enabled)
				{
					restore_tx_from_compressed();
				}
				else
				{
					// Re-stage through the ordered, capacity-checked funnel.
					restage_requeue_tx_messages();
					fifo_buffer_backup.flush();
					clear_retx_queue();  // R029: recovery (non-compressed) re-queues plaintext; drop stale retx
				}
				block_under_tx = NO;

				add_message_control(SET_CONFIG);
				connection_status = TRANSMITTING_CONTROL;
				return;
			}
		}
		} // frame-level gearshift ceiling scope

		// ── TOP-GEAR ACK-SEAM DISPATCH (BLOCKER D) ─────────────────────────────────
		// The cfg17 election's idle-window SET_CONFIG producer (process_buffer_data_commander,
		// the [TOPGEAR] queue producer above) can NEVER observe an engaged verdict during a
		// CONTINUOUS transfer: block_under_tx is BLOCK-scope (YES from first-stage to finalize),
		// and the between-batch empty instant is consumed by process_buffer_data_commander() in
		// the SAME poll, so `block_under_tx==NO` never coincides with engagement. Consume the live
		// engaged verdict HERE, at the clean-ACK seam, using the FRAME-UP promotion protocol
		// VERBATIM (the modem's own proven mid-transfer switch, ~40 lines up). The idle producer
		// STAYS (paced traffic + the HELD arm). No-op unless MERCURY_TOPGEAR_ELECT => byte-identical.
		if(!rate_opt.controls_link() && topgear_elect_feature_enabled() && data_ack_received == YES)
		{
			// B1: count down clean batches since the last seam demote (once per clean seam).
			if(last_batch_fully_acked && topgear_reengage_cooldown > 0)
				topgear_reengage_cooldown--;
			// Seam gate diagnostic (env-gated; silent + byte-identical when MERCURY_TOPGEAR_SEAM_DIAG unset).
			if(std::getenv("MERCURY_TOPGEAR_SEAM_DIAG") != nullptr
			   && (topgear_elect_engaged || current_configuration == CONFIG_17))
			{
				int fl_d = fifo_buffer_tx.get_size() - fifo_buffer_tx.get_free_size();
				printf("[TOPGEAR-SEAM-DIAG] engaged=%d cfg=%d fully_acked=%d cooldown=%d ctrl_free=%d "
				       "rung17=%d rungmeter=%.1f rungstale=%d rungage=%d fifo=%d override=%d inband=%d turbo=%d brk=%d\n",
				       topgear_elect_engaged?1:0, current_configuration, last_batch_fully_acked?1:0,
				       topgear_reengage_cooldown, messages_control.status==FREE?1:0,
				       rung_floor_ok(CONFIG_17)?1:0, rung_meter_db, rung_meter_stale()?1:0, rung_meter_age_batches,
				       fl_d, max_config_override,
				       inband_rate_feature_enabled()?1:0, turboshift_active?1:0, emergency_break_active);
				fflush(stdout);
			}
			if(!inband_rate_feature_enabled()   // cfg17 has no in-band CONFIG_TAG ladder index
			   && !turboshift_active
			   && emergency_break_active == 0
			   && link_status == CONNECTED
			   && is_ofdm_config(current_configuration)
			   && messages_control.status == FREE)
			{
				int tg_seam_target = -1;
				if(topgear_elect_engaged && current_configuration == CONFIG_16
				   && last_batch_fully_acked            // CLIMB only on a fully-clean seam (nothing in flight)
				   && topgear_reengage_cooldown == 0    // B1 anti-thrash: no re-climb for N batches after a demote
				   && (max_config_override < 0 || max_config_override >= CONFIG_17)
				   && rung_floor_ok(CONFIG_17))         // BLOCKER A row-17 floor (identity when ROW17_BUMP off)
				{
					int fifo_load = fifo_buffer_tx.get_size() - fifo_buffer_tx.get_free_size();
					if(fifo_load > 0)                        // r3: don't burn a switch at end-of-transfer
						tg_seam_target = CONFIG_17;          // CLIMB 16 -> 17
				}
				else if(!topgear_elect_engaged && current_configuration == CONFIG_17)
				{
					tg_seam_target = CONFIG_16;              // DEMOTE 17 -> 16 (deterministic soft-degrade)
				}
				if(tg_seam_target >= 0)
				{
					printf("[TOPGEAR] queue SET_CONFIG: %d -> %d (ACK-seam dispatch; engaged=%d "
					       "flatness=%.3f snr_dl=%.1f streak=%d fully_acked=%d)\n",
					       current_configuration, tg_seam_target, topgear_elect_engaged ? 1 : 0,
					       topgear_channel_flatness, measurements.SNR_downlink,
					       topgear_elect_clean_streak, last_batch_fully_acked ? 1 : 0);
					fflush(stdout);
					if(tg_seam_target == CONFIG_16)
						topgear_reengage_cooldown = TOPGEAR_REENGAGE_COOLDOWN_BATCHES;   // B1: arm the re-climb bar
					negotiated_configuration = tg_seam_target;
					// FRAME-UP promotion protocol VERBATIM: capture+roll the in-flight bsi, re-stage
					// pending TX for re-encode at the new config, release the block, queue the control
					// SET_CONFIG round-trip. On a clean fully-acked seam the roll/restage are no-ops
					// (nothing in flight, frames just freed); on the DEMOTE arm they handle any partial.
					roll_back_cmd_bsi_to_inflight("TOPGEAR-SEAM");
					if(compression_enabled)
					{
						restore_tx_from_compressed();
					}
					else
					{
						restage_requeue_tx_messages();
						fifo_buffer_backup.flush();
						clear_retx_queue();   // R029: recovery re-queues plaintext; drop stale retx
					}
					block_under_tx = NO;
					add_message_control(SET_CONFIG);
					connection_status = TRANSMITTING_CONTROL;
					return;
				}
			}
		}
		// ───────────────────────────────────────────────────────────────────────────

		// FIX-A — ROBUST-tier dwell-batch raise/revert (data-flow-robust-tier-arq-batch.md
		// §5.2). We reach here ONLY when FRAME-UP DECLINED to promote (a promotion does
		// add_message_control(SET_CONFIG)+return above), i.e. the climb is PARKED this
		// poll. evaluate_robust_dwell_batch() lifts the robust batch to a multi-frame
		// dwell when the rung is proven+parked (gate (e) keeps it at 1 while a higher
		// rung is still reachable — OR-1/L8), or reverts to 1 the instant eligibility is
		// lost. It is a no-op on the OFDM tier and on every non-change poll. When it
		// fires it sets connection_status=TRANSMITTING_CONTROL (the op TX), so the line
		// below is superseded and the control-ACK handshake gates the next DATA batch.
		if(evaluate_robust_dwell_batch())
			return;  // op queued (connection_status=TRANSMITTING_CONTROL) — do not fall to DATA

		connection_status=TRANSMITTING_DATA;
	}
}

// Cross-layer audit: cmd_batch_seq_id advances immediately after send_batch()
// dispatch and is reseated by the lossless-demote, CFG16-HOLD, BREAK, and
// CLIMB-CHURN rollback paths; frame-0 and stream stamps also consume that
// counter. messages_tx[].batch_seq_id is stamped when frames are staged and is
// the same ground truth all rollback paths scan. It therefore continues to
// identify the stranded batch after a rollback, and advances to the next batch
// (or no batch) as clean confirmation frees staged frames. Both transport guard
// calls and both partial de-dup keys consume partial_guard_reference_bsi(), so
// all four move together. With the cumulative cap, partial_sack_dedup_key()
// returns this in-flight identity, preserving repeat de-dup and per-batch
// distinction. Wire format, staging, and the pure predicates are unchanged.
//
// Returns the mod-256 identity of the batch currently awaiting ACK (the earliest
// non-FREE, length>0 staged batch_seq_id in messages_tx[]), or -1 if nothing is
// in flight. This is the batch a reverse-SACK PARTIAL bitmap targets by slot index.
int cl_arq_controller::current_inflight_bsi() const
{
	int min_inflight_bsi = -1;
	for(int i=0; i<nMessages; i++)
	{
		if(messages_tx[i].status != FREE && messages_tx[i].length > 0)
		{
			if(messages_tx[i].batch_seq_id < 0) continue;
			int b = messages_tx[i].batch_seq_id & 0xFF;
			if(min_inflight_bsi < 0)
				min_inflight_bsi = b;
			else
			{
				// keep the mod-256-EARLIER bsi (forward distance b->min in
				// [1,128] means min is later, so b is earlier).
				unsigned fwd = ((unsigned)(min_inflight_bsi - b)) & 0xFFu;
				if(fwd >= 1u && fwd <= 128u)
					min_inflight_bsi = b;
			}
		}
	}
	return min_inflight_bsi;
}

// Reference batch identity for the reverse-SACK partial-target guard and its de-dup key.
// Normally the actual in-flight batch (messages_tx[] ground truth); falls back to
// cmd_batch_seq_id when nothing is staged. MERCURY_GUARD_REANCHOR_DEFEAT=1 restores the
// legacy (mis-anchored) behavior of passing the dispatch counter, for A/B isolation.
int cl_arq_controller::partial_guard_reference_bsi() const
{
	static const bool defeat =
		(std::getenv("MERCURY_GUARD_REANCHOR_DEFEAT") != nullptr);
	if(defeat)
		return cmd_batch_seq_id;
	int ib = current_inflight_bsi();
	return (ib >= 0) ? ib : cmd_batch_seq_id;
}

// A2 ownership/span gate for compact per-batch reports.  The target must be
// owned by the live per-batch staging array, the R2c retention shadow, or an
// explicitly journaled generation.  Aggregate directory BSIs never call this.
bool cl_arq_controller::generation_ack_target_owned(int target_bsi, int span,
	bool allow_shadow, bool* is_shadow) const
{
	if(is_shadow) *is_shadow = false;
	if(span < 1 || span > MAX_SACK_BATCH_SIZE || span != data_batch_size)
		return false;
	const int target = target_bsi & 0xFF;
	bool live = false;
	bool live_mismatch = false;
	for(int i=0; i<nMessages; i++)
	{
		if(messages_tx[i].status == FREE || messages_tx[i].length <= 0)
			continue;
		if((messages_tx[i].batch_seq_id & 0xFF) == target)
			live = true;
		else
			live_mismatch = true;
	}
	// The per-batch staging array is one generation.  A mixed live array cannot
	// safely be indexed by a generation bitmap, even if one slot happens to own
	// the resolved target.
	if(live) return !live_mismatch;
	if(allow_shadow)
	{
		for(int i=0; i<cmd_prev_retain_count; i++)
		{
			if(cmd_prev_retain_bsi[i] == target
			   && cmd_prev_retain_slot[i] >= 0
			   && cmd_prev_retain_slot[i] < span)
			{
				if(is_shadow) *is_shadow = true;
				return true;
			}
		}
	}
	if(l1_tx_journal.enabled())
	{
		for(const auto& entry : l1_tx_journal.entries())
			if(entry.key.bsi == target && entry.span == (uint16_t)span)
				return true;
	}
	return false;
}

bool cl_arq_controller::resolve_and_validate_generation_ack(uint8_t wire_field,
	int span, bool integrity_ok, bool clean, bool bitmap_any, bool bitmap_all,
	st_resolved_generation_ack* out) const
{
	if(!out || !integrity_ok)
		return false;
	st_resolved_generation_ack resolved = {};
	resolved.wire_field = wire_field;
	resolved.target = generation_ack_resolve_target(
		wire_field, cumulative_ack_enabled);
	resolved.cumulative = cumulative_ack_enabled;
	resolved.clean = clean;
	resolved.span = span;
	resolved.shadow_owned = false;
	*out = resolved;
	if(!generation_ack_target_owned(resolved.target, span,
		/*allow_shadow=*/!clean, &resolved.shadow_owned))
		return false;
	if(clean)
	{
		const int inflight = current_inflight_bsi();
		if(inflight < 0 || resolved.target != (inflight & 0xFF))
			return false;
	}
	if(!bitmap_any || clean != bitmap_all)
		return false;
	*out = resolved;
	return true;
}

// CLIMB-CHURN bsi rollback (data-flow-climb-up-bsi-rollback.md §5). Shared
// capture+rollback ported VERBATIM from the FIX-9 D3 demote (:4057-4108) and
// M6 BREAK (:4271-4300): scan messages_tx[] for the EARLIEST (mod-256) in-flight
// (non-FREE, length>0) batch_seq_id and roll cmd_batch_seq_id back to it so a
// climb-UP re-present of that already-RSP-delivered batch is CONTIGUOUS with the
// RSP's delivery high-water (sack_v2_readopt_has_gap sees ==last / ==last+1, not
// a >=2 jump -> NO [RSP-V2-GAP-ABORT] hold). Gated IDENTICALLY to the demote
// rollback (sack_v2_enabled && !compression_enabled): the compression path's
// restore_tx_from_compressed() owns its own re-stage and is unchanged; v1
// sessions never read the v2 gap-gate. MUST run BEFORE the caller frees
// messages_tx[] (else nothing is in-flight to capture). All non-FREE frames of
// the stranded block share the SAME batch_seq_id (one batch in flight on the
// per-frame path), so the EARLIEST mod-256 non-FREE bsi IS the in-flight batch.
int cl_arq_controller::roll_back_cmd_bsi_to_inflight(const char* tag)
{
	if(!(sack_v2_enabled && !compression_enabled))
		return -1;

	int min_inflight_bsi = current_inflight_bsi();

	if(min_inflight_bsi >= 0)
	{
		printf("[%s] CLIMB-CHURN LOSSLESS PROMOTE: rolling cmd_batch_seq_id %d -> %d "
			"so the re-sent batch carries the in-flight (contiguous) bsi "
			"(no RSP-V2-GAP-ABORT hold)\n",
			tag ? tag : "CLIMB", cmd_batch_seq_id & 0xFF, min_inflight_bsi);
		fflush(stdout);
		cmd_batch_seq_id = min_inflight_bsi;
	}
	return min_inflight_bsi;
}

void cl_arq_controller::finish_turbo_direction()
{
	turboshift_active = false;
	turbo_supershift_announce_pending = false;
	turbo_snr_ack_enabled = false;
	// turbo_best_snr preserves the best SNR seen across ALL turbo probes,
	// unlike turbo_received_snr which is reset after each probe step.
	turbo_received_snr = -99.0f;

	// KX-as-data (data-flow-hybrid-kex.md Â§7): in the pre-activation KX epoch the
	// turboshift MUST NOT run its bidirectional REVERSE-direction probe. That probe fires
	// its OWN SWITCH_ROLE (below) which collides with the KX reverse-ct role-swap handshake:
	// it hands the commander role to the peer BEFORE that peer has finished the forward-pk
	// receipt + reverse-ct staging, so the peer (now commander, kx_stream_tx_len==0) stages a
	// SECOND fresh forward stream and desyncs the crypto transcript -- activation then never
	// lands (a cross-layer data-flow audit measured fwd_staged==2, KEY_ACTIVATE==0 on 8/8
	// robust-start cells). Route the KX-epoch forward-complete into the SAME proven settle
	// path as --skip-turbo-reverse so the climbed config SETTLES and the KX handshake owns the
	// role swaps; the forward stream still DROVE the climb off robust (that win is preserved).
	// Env off / post-activation -> kx_stream_epoch() false -> byte-identical.
	if(turboshift_phase == TURBO_FORWARD
	   && (skip_turbo_reverse || kx_stream_epoch()
	       || !reverse_data_demand_acknowledged))
	{
		// Forward direction probed. Skip REVERSE probe (--skip-turbo-reverse).
		// Reverse path only needs MFSK ACKs on asymmetric channels.
		turboshift_phase = TURBO_DONE;

		// Turbo probing with single frames is unreliable — it may skip configs
		// (via SNR supershift) leaving gaps in the tested range. Use the SNR
		// measurement as the primary guidance for both ceiling and start config.
		double effective_snr = (turbo_best_snr > -90) ? turbo_best_snr : measurements.SNR_uplink;
		int snr_config = -1;
		if(effective_snr > -90)
			snr_config = get_configuration(effective_snr - SUPERSHIFT_MARGIN_DB);

		// Start at the SUPERSHIFT-verified config (turboshift_last_good).
		// Rationale: the verification probe at line ~2483 just confirmed this
		// config can pass a frame end-to-end. Picking a lower start config
		// (former: snr_config - 2 ladder steps for fading margin) forces an
		// UNNECESSARY PHY-switch to a lower config, which on the CMD side
		// breaks clean-batch ACK detection on the first post-switch batch
		// (gearshift_v11 finding: RSP gets 21/25 frames at CFG13, sends a
		// clean ACK — historically OFDM_ACK_CLEAN, now the MFSK suffix —
		// CMD's poll never matches → FRAME UP DATA FAILED → BREAK chain
		// pulls config down to 11). Cost on clean: 7-8× lower throughput
		// vs v22 baseline (2535 vs 343 bps). Trust the probe; if
		// production traffic fails, ladder will drop on its own.
		//
		// §7.13.38 — SAC-aware ceiling trust: when SACK Design A is
		// negotiated, the channel can absorb partial-batch loss (SACK_RSP
		// patches missing frames), so we trust the verified ceiling
		// unconditionally. Without SACK, the SNR-derived snr_config caps
		// us as a fading-margin safety net (legacy behavior). This lets
		// CFG16 actually be reached on clean: measured SNR at the modem
		// caps around 15 dB; SNR-3 = 12 → snr_config = CFG15, which
		// would cap us below the verified CFG16 ceiling. With SACK on,
		// just use the verified ceiling.
		int start_config = turboshift_last_good;
		if(!sack_v2_enabled && snr_config >= 0 && snr_config < start_config)
			start_config = snr_config;
		if(start_config < 0)
			start_config = init_configuration;

		// Enforce --max-config CLI ceiling
		if(max_config_override >= 0 && start_config > max_config_override)
			start_config = max_config_override;

		// WALL-B FIX-5 hook #4 (NON-NEGOTIABLE — finish_turbo_direction start_config):
		// THIS is the exact site that DEFEATS FIX-4. start_config = turboshift_last_good
		// (=CFG16 after a clean climb), and the lines below re-install it as BOTH the
		// proven ceiling AND the data/negotiated/reverse config — unconditionally, on every
		// ROBUST->CFG16 turbo re-climb — erasing FIX-4's supershift_proven_ceiling=CFG15 pin
		// (WALLB_DIAGNOSIS.md §1; audit §5.5 hook #4). Because the cooldown field is NOT
		// touched by the turbo state machine, capping start_config HERE makes the carve-dead
		// memory survive the re-climb: the link restarts at CFG15 (not CFG16), no re-park.
		// No-op off the cooldown (apply_bigblock_cooldown_cap is the identity when
		// bigblock_carve_cooldown_batches==0), so non-bigblock turbo completion is identical.
		start_config = apply_bigblock_cooldown_cap(start_config);
		// CFG16 decode-margin gate: with SACK on the start trusts the verified ceiling
		// (turboshift_last_good, possibly CFG16) UNCONDITIONALLY — so a turbo re-climb must
		// not restart at CFG16 when the reverse-path SNR no longer supports it (never-raise).
		start_config = apply_cfg16_margin_cap(start_config);
		// Per-rung floor gate on the turbo-forward-complete start_config — the ERASURE-SITE-1 fix
		// (audit §4): capping start_config HERE floor-caps the supershift_proven_ceiling assigned on
		// the next line, so the ROBUST->re-climb cannot re-open the ceiling above a closed floor and
		// re-elect the disproven rung. Never-raise; identity below CONFIG_9 / at high SNR / WB-off.
		start_config = rung_floor_cap_fire(start_config);

		// Set ceiling = start config. The SNR→config table is calibrated for AWGN
		// but fading channels need 3-6 dB more margin. Setting ceiling at start
		// prevents the ladder from immediately climbing into failing configs.
		// Ceiling recovery (20 good blocks) allows gradual upward exploration.
		supershift_proven_ceiling = start_config;

		printf("[TURBO] FORWARD complete (skip-reverse): ceiling=%d, proven_ceiling=%d, snr=%.1f, snr_cfg=%d, starting at config %d\n",
			turboshift_last_good, supershift_proven_ceiling, effective_snr, snr_config, start_config);
		fflush(stdout);

		data_configuration = start_config;
		negotiated_configuration = start_config;
		reverse_configuration = start_config;

		// CRITICAL: Do NOT load_configuration locally here. The SET_CONFIG
		// ACK handler at line ~3155 is the canonical place that loads the
		// new config — only AFTER RSP acks on the current (verified) PHY.
		// Loading the new config locally first means the SET_CONFIG frame
		// itself would be transmitted on the NEW PHY, which RSP can't
		// decode (it's still at the verified probe config). RSP never
		// hears the announcement, never switches, and CMD's data bounces
		// off a peer at a different config — manifests as 100x audio
		// underruns + LDPC iter=101 max-out on RSP (gearshift_v7 finding).
		// See SUPERSHIFT path at line ~1444-1460 for the correct pattern
		// (sets neg cfg, queues SET_CONFIG, never load_configuration locally).
		//
		// CLIMB-CHURN bsi rollback (data-flow-climb-up-bsi-rollback.md §5): this is
		// the turbo OVER-CLIMB emitter. cleanup() below frees only ACKED/FAILED
		// frames; any un-ACKed in-flight batch SURVIVES into the re-present at the
		// new config. Capture+roll BEFORE cleanup so that re-present is CONTIGUOUS
		// with the RSP high-water (no >=2 gap-gate HOLD). No-op when no batch is in
		// flight (single-frame turbo probe) or off the v2 in-order path.
		roll_back_cmd_bsi_to_inflight("TURBO");
		cleanup();
		add_message_control(SET_CONFIG);
		connection_status = TRANSMITTING_CONTROL;
	}
	else if(turboshift_phase == TURBO_FORWARD)
	{
		// Forward direction probed. Advance to REVERSE (other side will probe).
		// Reaching this branch proves a reverse demand was already acknowledged;
		// the no-demand case settled through the skip-reverse branch above.
		turboshift_phase = TURBO_REVERSE;
		printf("[TURBO] FORWARD complete: ceiling=%d, switching roles for REVERSE\n",
			turboshift_last_good);
		fflush(stdout);
		cleanup();
		add_message_control(SWITCH_ROLE);
		connection_status = TRANSMITTING_CONTROL;
	}
	else if(turboshift_phase == TURBO_REVERSE)
	{
		if(!reverse_data_demand_acknowledged)
		{
			// The reverse sender retains terminal authority unless its receiver has
			// independently advertised queued data.  A completed probe alone is not
			// permission for a speculative handback.
			printf("[ROLE-DEMAND] turbo reverse complete with no demand; retaining commander authority\n");
			fflush(stdout);
			turboshift_phase = TURBO_DONE;
			connection_status = TRANSMITTING_DATA;
			return;
		}
		// Reverse direction probed. Switch back to original roles.
		printf("[TURBO] REVERSE complete: ceiling=%d, switching back\n",
			turboshift_last_good);
		fflush(stdout);
		turboshift_phase = TURBO_DONE;
		cleanup();
		add_message_control(SWITCH_ROLE);
		connection_status = TRANSMITTING_CONTROL;
	}
	else
	{
		// Already done (shouldn't happen)
		turboshift_phase = TURBO_DONE;
		connection_status = TRANSMITTING_DATA;
	}
}

void cl_arq_controller::cmd_arm_terminal_eot_arq()
{
#ifdef TERMINAL_EOT_RETRY_FAILBEFORE
	// Exact pre-fix timing: the generic cfg-dependent ack_timeout_control stays
	// in force, so cfg16 cannot re-solicit within the short receive window.
	return;
#else
	if(link_status!=DISCONNECTING || messages_control.data==NULL
	   || (unsigned char)messages_control.data[0]!=CLOSE_CONNECTION)
		return;
	const int retry_ms = terminal_eot_retry_window_ms(receiving_timeout);
	messages_control.ack_timeout = retry_ms;
	printf("[CMD-EOT-ARQ] armed retry window=%dms remaining_retries=%d/%d "
	       "(final_backstop=%dms)\n",
		retry_ms, messages_control.nResends > 0 ? messages_control.nResends - 1 : 0,
		(int)TERMINAL_EOT_RETRIES, terminal_settle_deadline_ms());
	fflush(stdout);
#endif
}

void cl_arq_controller::cmd_terminal_eot_wait_for_backstop()
{
	int elapsed = disconnecting_settle_timer.counting
		? disconnecting_settle_timer.get_elapsed_time_ms() : 0;
	int remaining = terminal_settle_deadline_ms() - elapsed;
	if(remaining < 1) remaining = 1;

	// Keep the immutable CLOSE descriptor and the ACK detector live, but arm no
	// more transmissions. update_status() owns the independent 45 s give-up.
	messages_control.nResends = 0;
	messages_control.status = PENDING_ACK;
	messages_control.ack_timeout = remaining;
	messages_control.ack_timer.stop();
	messages_control.ack_timer.reset();
	messages_control.ack_timer.start();
	receiving_timeout = remaining;
	receiving_timer.stop();
	receiving_timer.reset();
	receiving_timer.start();
	if(telecom_system != NULL)
		telecom_system->data_container.frames_to_read = 2;
	connection_status = RECEIVING_ACKS_CONTROL;
	printf("[CMD-EOT-ARQ] fast retry budget exhausted after %d retransmits; "
	       "listening %dms for late ACK before 45s final backstop\n",
		(int)TERMINAL_EOT_RETRIES, remaining);
	fflush(stdout);
}

void cl_arq_controller::cmd_terminal_settle_converge(const char* reason)
{
	// See data-flow-stream-offset.md 8.6. Reached only after the fast bounded
	// CLOSE+EOT retry train and the independent 45 s peer-gone backstop both
	// expire. DATA frames were ACKed, but terminal EOT delivery is unconfirmed;
	// release fail-closed rather than claiming successful terminal settlement.
	disconnecting_settle_timer.stop();
	disconnecting_settle_timer.reset();
	cmd_terminal_converge_events++;
	printf("[CMD-EOT-CONVERGE] terminal EOT/ACK unconfirmed after fast bounded retransmits "
		"and final backstop (%s); releasing INCOMPLETE via DISCONNECTED (converge #%d)\n",
		reason, cmd_terminal_converge_events);
	fflush(stdout);

	reset_session_state();
	load_configuration(init_configuration,FULL,YES);
	this->link_status=LISTENING;
	this->connection_status=RECEIVING;
	reset_all_timers();
	telecom_system->data_container.frames_to_read =
		telecom_system->data_container.preamble_nSymb + telecom_system->data_container.Nsymb;
	telecom_system->data_container.nUnder_processing_events = 0;
	fifo_buffer_tx.flush();
	fifo_buffer_backup.flush();
	fifo_buffer_rx.flush();
	messages_control.status=FREE;
	set_role(RESPONDER);

	emit_disconnected_once();
}

void cl_arq_controller::process_control_commander()
{
	if(this->connection_status==RECEIVING_ACKS_CONTROL)
	{
		if(this->link_status==CONNECTING && messages_control.data[0]==START_CONNECTION)
		{
			// NB/WB auto-negotiation: NB commander in WB Phase 2 — switch back to NB
			if(commander_configured_nb == YES && narrowband_enabled == NO)
			{
				printf("[NB-NEG] Commander: NB commander connected via WB, switching to NB\n");
				fflush(stdout);
				switch_narrowband_mode(YES);
			}
			watchdog_timer.start();
			this->link_status=CONNECTION_ACCEPTED;
			connection_status=TRANSMITTING_CONTROL;
			// IDLE-SWITCHROLE-RACE (idle-switchrole-race.md §2/§5.5): CONNECT skips
			// reset_session_state (arq_common.cc:801) so the per-session race/teardown
			// flags MUST be cleared here at connect-accept too — otherwise a prior
			// session's session_data_frame_sent==true would re-open the race on this
			// fresh connection, and a stale break_noprogress_cycles would mis-count.
			session_data_frame_sent = false;
			session_data_frame_received = false;
			reverse_data_demand_acknowledged = false;
			reverse_data_demand_last_advertised = false;
			role_demand_refresh_timer.stop();
			role_demand_refresh_timer.reset();
			break_noprogress_cycles = 0;
			// CONNECT does not call reset_session_state(): clear all per-session
			// topgear evidence here as well, so a prior peer/channel cannot leave
			// cfg17 armed on this fresh connection.
			telecom_system->invalidate_channel_selectivity();
			topgear_elect_engaged = false;
			topgear_elect_clean_streak = 0;
			topgear_drop_streak = 0;
			l1_aggregate_defer_streak = 0;
			emergency_ackmiss_defer_streak = 0;
			topgear_channel_flatness = -1.0;
			topgear_clear_forward_verdict();
			topgear_last_report_bsi = -1;
			// CONNECT skips reset_session_state(): establish MC-4's virtual
			// delivered predecessor for this fresh session explicitly.
			if(linkphase_rxwindow_on())
			{
				cmd_rxwindow_delivered_high_water = (cmd_batch_seq_id - 1) & 0xFF;
				cmd_rxwindow_hold_timer.stop();
				cmd_rxwindow_hold_timer.reset();
			}
			// Reset per-phase so each handshake step gets its own timeout window
			connection_attempt_timer.reset();
			connection_attempt_timer.start();
			if(ack_pattern_time_ms > 0)
			{
				// ACK pattern carries no data, keep BROADCAST_ID
				// (responder's assigned_connection_id is in the ACK frame payload
				// which doesn't exist for tone patterns)
				this->connection_id=BROADCAST_ID;
				this->assigned_connection_id=BROADCAST_ID;
			}
			else
			{
				// Fallback: ACK frame carries responder's assigned connection_id
				this->connection_id=messages_control.data[1];
				this->assigned_connection_id=messages_control.data[1];
			}
			l1_tx_journal.begin_session((uint8_t)this->connection_id);
		}
		else if((this->link_status==CONNECTION_ACCEPTED || this->link_status==CONNECTED)
		        && (messages_control.data[0]==TEST_CONNECTION
		            || messages_control.data[0]==TEST_CONNECTION_ACK))
		{
			// v9 handshake: branch on incoming type.
			// - TEST_CONNECTION_ACK: RSP-initiated LDPC frame carrying caps
			//   echo + CRC8. Validate before transition.
			// - TEST_CONNECTION: legacy ACK-pattern emulation. peer_capability
			//   comes from data[5] = our own TX content (symmetric assumption).
			bool is_ack = (messages_control.data[0]==TEST_CONNECTION_ACK);
			if(is_ack)
			{
				unsigned char echoed_cap = (unsigned char)messages_control.data[1];
				unsigned char rsp_own    = (unsigned char)messages_control.data[2];
				unsigned char rx_crc     = (unsigned char)messages_control.data[3];
				unsigned char calc_crc   = (unsigned char)CRC8_calc(
					(char*)&messages_control.data[1], 2);
				if(rx_crc != calc_crc)
				{
					printf("[HANDSHAKE-ECHO] FAIL crc8 mismatch: rx=0x%02X calc=0x%02X "
						"(retries_left=%d)\n",
						rx_crc, calc_crc, handshake_retries_left);
					fflush(stdout);
					if(handshake_retries_left > 0)
					{
						handshake_retries_left--;
						messages_control.status = FREE;
						return;
					}
					printf("[HANDSHAKE-ECHO] DROP: persistent CRC mismatch on echo\n");
					fflush(stdout);
					this->link_status = DROPPED;
					reset_session_state();
					return;
				}
				if(!handshake_cap_echo_compatible(
						(unsigned char)local_capability,
						echoed_cap,
						rsp_own))
				{
					printf("[HANDSHAKE-ECHO] FAIL cap mismatch: echoed=0x%02X local=0x%02X "
						"(silent corruption suspected; retries_left=%d)\n",
						echoed_cap, (unsigned char)local_capability,
						handshake_retries_left);
					fflush(stdout);
					if(handshake_retries_left > 0)
					{
						handshake_retries_left--;
						messages_control.status = FREE;
						return;
					}
					printf("[HANDSHAKE-ECHO] DROP: persistent capability echo mismatch\n");
					fflush(stdout);
					this->link_status = DROPPED;
					reset_session_state();
					return;
				}
				printf("[HANDSHAKE-ECHO] OK echoed_cap=0x%02X own=0x%02X — handshake confirmed\n",
					echoed_cap, rsp_own);
				fflush(stdout);
				if(echoed_cap != (unsigned char)local_capability)
				{
					printf("[HANDSHAKE-ECHO] COMPAT optional bit3 absent: "
						"echoed=0x%02X local=0x%02X peer=0x%02X\n",
						echoed_cap, (unsigned char)local_capability, rsp_own);
					fflush(stdout);
				}
				handshake_confirmed = true;
				peer_capability = rsp_own;
				int pk = -1, pd = -1, pu = -1;
				bool t1_ldpc_calibration = messages_control.length >= 9
					&& t1_calibration_decode(
						&messages_control.data[4], 5, &pk, &pd, &pu);
				if(t1_ldpc_calibration)
				{
					t1_peer_calibration_valid = true;
					t1_peer_keyup_ms = pk;
					t1_peer_deaf_ms = pd;
					t1_peer_undeaf_ms = pu;
				}
				if(t1_peer_calibration_valid)
				{
					t1_seed_calibration_defaults();
					printf("[T1-CALIB-WIRE] peer keyup=%d deaf=%d undeaf=%d ms; PREKEY negotiated\n",
						t1_peer_keyup_ms, t1_peer_deaf_ms, t1_peer_undeaf_ms);
					fflush(stdout);
				}
				// No SNR carried in TEST_CONNECTION_ACK — leave SNR unchanged.
				// NB robust-preamble negotiation: the responder's own caps just
				// arrived — flip to sidelnikov on both-support.
				update_robust_preamble_negotiation();
			}
			else
			{
				u_SNR tmp_SNR;
				for(int i=0;i<4;i++)
				{
					tmp_SNR.char4_SNR[i]=messages_control.data[i+1];
				}
				measurements.SNR_downlink=quantize_snr_decision_db((double)tmp_SNR.f_SNR);
				// Read responder's capability from byte 5.
				// With LDPC ACK: this is the responder's reply (correct).
				// With ACK pattern: no data payload, so this is our own TX data (assumes
				// symmetric capability — works when both sides use same bandwidth_mode).
				// The legacy inference helper preserves established bits but clears
				// CAP_RETX_TURN_TAIL because no responder wire proof exists.
				// Responder's SWITCH_BANDWIDTH handler rejects if nb_only as a safety net.
				peer_capability = legacy_ack_inferred_peer_cap(
					(uint8_t)messages_control.data[5]);
				// NB robust-preamble negotiation: a bare legacy ACK carries no
				// responder capability proof, so the inference helper cleared the
				// preamble bit -> this resolves to LEGACY (the interop floor).
				update_robust_preamble_negotiation();
			}
			// Only the CRC-checked capability echo can enable Stage-3. The legacy
			// inferred-capability branch therefore always resolves to legacy ACK.
			l1_complete_blockack_handshake(is_ack && handshake_confirmed);
			printf("[BW-NEG] Responder capability: 0x%02X (WB=%s, ENCRYPT=%s)\n",
				peer_capability,
				(peer_capability & CAP_WB_CAPABLE) ? "yes" : "no",
				(peer_capability & CAP_ENCRYPTION) ? "yes" : "no");
			fflush(stdout);

			// Read responder's SSID from byte 6 (confirmation — commander already knows it)
			{
				int peer_ssid = (uint8_t)messages_control.data[6];
				printf("[SSID] Responder SSID=%d (0xFF=none)\n", peer_ssid);
				fflush(stdout);
			}

			// Compression is unconditional (CAP_COMPRESSION removed; -F off opts out).
			if(force_compress)
			{
				compression_enabled = true;
				compressor.init();
				printf("[COMPRESS] Force-enabled (--compress flag)\n");
				fflush(stdout);
			}
			else
			{
				compressor.init();  // Pre-init contexts, arm later on B2F detection
				printf("[COMPRESS] Deferred (waiting for B2F detection)\n");
				fflush(stdout);
			}

			// B2F handler: init for Winlink LZHUF unroll/reroll.
			// CAP_B2F_UNROLL removed — always on.
			b2f_handler.init();
			b2f_handler.unroll_enabled = true;

			// Encryption negotiation (FAIL-CLOSED; single source of truth in
			// decide_encryption_negotiation()). DEFAULT-OFF is preserved: when the
			// operator did not opt in the decision is PLAINTEXT_OK and this block is
			// a no-op, so legal Part-97 plaintext operation is unchanged.
			{
				enc_negotiation_outcome_t enc_dec = decide_encryption_negotiation(
					encryption_mode, local_capability, peer_capability);
				if (enc_dec == ENC_NEG_ENABLED)
				{
					encryption_enabled = true;
					// Encryption requires batch-level assembly (compression path)
					if (!compression_enabled)
					{
						compression_enabled = true;
						printf("[CRYPTO] Forced compression ON (required for batch encryption)\n");
					}
					printf("[CRYPTO] Encryption negotiated (%s mode), key exchange after turboshift\n",
						encryption_mode == ENCRYPT_STRICT ? "SNDL-safe" : "classical-first");
				}
				else if (enc_dec == ENC_NEG_REFUSE)
				{
					// Operator opted into -E but the peer does not advertise
					// CAP_ENCRYPTION (unsupported, or a MITM stripped the cap bit to
					// force a plaintext downgrade). Fail closed for BOTH strict and
					// fast: refuse rather than deliver plaintext under an -E opt-in.
					printf("[CRYPTO] %s mode: peer lacks encryption — refusing connection (fail-closed, no plaintext downgrade)\n",
						encryption_mode == ENCRYPT_STRICT ? "STRICT" : "FAST");
					fflush(stdout);
					const char* err_msg = "ENCRYPTION FAILURE PEER UNSUPPORTED\r";
					int elen = (int)strlen(err_msg);
					for(int e=0; e<elen; e++)
						tcp_socket_control.message->buffer[e] = err_msg[e];
					tcp_socket_control.message->length = elen;
					tcp_socket_control.transmit();
					this->link_status = DROPPED;
					reset_session_state();
					return;
				}
				// ENC_NEG_PLAINTEXT_OK: operator did not opt in — plaintext ops (legal default-off).
			}

			// FORGIVING-ACK Tier 2 negotiation (fact-documents/data-flow-forgiving-ack.md
			// §T2.1): mirror of the RSP-side agreement (arq_responder.cc). The cumulative-
			// n_r SACK reshape engages ONLY when BOTH ends advertise CAP_CUMULATIVE_ACK
			// (env-gated locally by MERCURY_CUMULATIVE_ACK). Interop-safe: a non-Tier-2
			// responder leaves bit 2 clear → cumulative_ack_enabled false → per-batch.
			cumulative_ack_enabled = (local_capability & CAP_CUMULATIVE_ACK)
			                      && (peer_capability  & CAP_CUMULATIVE_ACK);
			printf("[FORGIVING-ACK-T2] cumulative-n_r SACK %s (local_cap=0x%02X peer_cap=0x%02X)\n",
				cumulative_ack_enabled ? "NEGOTIATED" : "off",
				(unsigned)local_capability, (unsigned)peer_capability);
			fflush(stdout);

			// Streaming compression is unconditional (CAP_STREAMING removed).
			// Still requires compression to be on; -F off disables both.
			if(compression_enabled)
			{
				compressor.streaming_enable();
			}

			// SACK / SACK_V2 are unconditional (CAP_SACK + CAP_SACK_V2 removed).
			// --no-sack still flips disable_sack which takes sack_enabled offline.
			sack_enabled = !disable_sack;
			sack_v2_enabled = sack_enabled;
			if(sack_enabled)
			{
				// Update batch size now that SACK is negotiated. 30s target
				// matches the formula in arq_common.cc batch sizing.
				//
				// climb-engine Bug 2 (gearshift-climb-engine.md §3 / data-flow-
				// batch-size.md): ROBUST/MFSK configs are EXCLUDED from the >=5
				// floor. load_configuration() (arq_common.cc:1229-1234) deliberately
				// sets data_batch_size=1 for robust, and its batch-scaling path
				// (arq_common.cc:1269) is already !is_robust_config-gated. Re-applying
				// the unconditional SACK floor here clobbers that and forces batch
				// back to >=5 at ROBUST_0, where a clean all-ones MFSK ACK requires
				// all 5 frames to survive first-pass at the floor SNR (it never does)
				// — so the climb never gets a single clean batch to START with. At
				// batch=1 every delivered MFSK frame is itself an all-ones batch ->
				// clean ACKs accumulate.
				//
				// 4th-wire-failure ROOT CAUSE (data-flow-batch-size.md §4, this fix):
				// the gate previously read negotiated_configuration, which on a fresh
				// unpinned `-g -R` connect is still its CTOR DEFAULT CONFIG_0
				// (arq_common.cc:281) — reset_session_state() only writes it on the
				// teardown branch (arq_commander.cc:443) and the other writes
				// (205/297/530) are BREAK-recovery / OFDM-optimizer only, never the
				// connect path. So is_robust_config(CONFIG_0)=false -> guard bypassed
				// -> CMD batch=5 while the RSP (current_configuration=ROBUST_0)
				// correctly kept batch=1 -> first ROBUST_0 block fails the clean-ACK
				// match -> LINK-TIMEOUT -> climb never starts. FIX: gate on
				// current_configuration — the live-PHY config, == ROBUST_0 here (set
				// by the startup load_configuration(data_configuration=ROBUST_0)) and
				// the EXACT SAME variable+predicate the RSP recompute (arq_responder.cc)
				// and the Axis-2 robust guard use. Now CMD and RSP read the same var
				// with the same predicate and CANNOT diverge (an asymmetric override
				// was historical Bug #9). The set_data_batch_size() chokepoint
				// (arq_common.cc) is the belt-and-suspenders backstop. OFDM
				// (CONFIG_0..16) keeps the >=5 floor unchanged.
				sack_negotiated_recompute_batch("CMD");
			}
			else
			{
				printf("[SACK] Disabled by --no-sack\n");
			}
			fflush(stdout);

			switch_role_test_timer.stop();
			switch_role_test_timer.reset();

			if(gear_shift_on==YES && gear_shift_algorithm==SNR_BASED)
			{
				this->link_status=NEGOTIATING;
				connection_status=TRANSMITTING_CONTROL;
				link_timer.start();
				watchdog_timer.start();
				gear_shift_timer.start();
			}
			else
			{
				// gear_shift off, or SUCCESS_BASED_LADDER (defers SET_CONFIG to post-block)
				if(this->link_status==CONNECTION_ACCEPTED)
				{
					std::string str="CONNECTED "+this->my_call_sign+" "+this->destination_call_sign+" "+ std::to_string(telecom_system->bandwidth)+"\r";
					tcp_socket_control.message->length=str.length();

					for(int i=0;i<tcp_socket_control.message->length;i++)
					{
						tcp_socket_control.message->buffer[i]=str[i];
					}
					tcp_socket_control.transmit();
				}

				this->link_status=CONNECTED;
#ifdef MERCURY_GUI_ENABLED
				gui_set_monitor_callsigns(this->my_call_sign.c_str(),
				                          this->destination_call_sign.c_str());
				{
					char buf[128];
					snprintf(buf, sizeof(buf), "[CONNECTED %s <-> %s]",
					         this->my_call_sign.c_str(),
					         this->destination_call_sign.c_str());
					gui_push_monitor_event(buf, true);
				}
#endif
				watchdog_timer.start();
				link_timer.start();
				connection_attempt_timer.stop();
				connection_attempt_timer.reset();

				int connect_fuse_floor_config = current_configuration;
				if(duty_trace_enabled())
				{
					printf("[DUTY-TRACE] fusion-apply turboshift_active=%d turboshift_phase=%d "
						"break_recovery_phase=%d data_ack_received=%d snr_uplink=%.1f "
						"current=%d negotiated=%d\n",
						(int)turboshift_active, (int)turboshift_phase, break_recovery_phase,
						data_ack_received, measurements.SNR_uplink, current_configuration,
						negotiated_configuration);
					fflush(stdout);
				}
				if(connect_fast_active)
				{
					robust_enabled = connect_fast_fallback_robust;
					init_configuration = connect_fast_fallback_config;
					data_configuration = connect_fast_fallback_config;
					ack_configuration  = connect_fast_fallback_config;
					last_data_viable_config = session_floor_anchor(robust_enabled, init_configuration);
					// A fast connect completed the handshake at the fast config (CONFIG_0); reseat the
					// live PHY to the restored pin/fallback so forward DATA is keyed there. The budgeted
					// revert slow path already reseats via load_configuration; the success path must too,
					// or a quick connect leaves the commander transmitting at CONFIG_0 while the intent
					// vars read the pin (connect-fast success-vs-revert restore asymmetry; cross-layer
					// data-flow audit of the connect_fast restore paths).
					// [WBDIRECT-REGRESSION] fail-before hook: MERCURY_CONNECT_FAST_DEFEAT_RESEAT=1
					// SKIPS this live-PHY reseat so the SAME binary reproduces the pre-fix strand
					// (commander keeps keying DATA at the fast config while the intent vars read the
					// pin). Production NEVER sets it — only the loopback regression's fail-before arm.
					{
						const char* cfdr = std::getenv("MERCURY_CONNECT_FAST_DEFEAT_RESEAT");
						if(!(cfdr && *cfdr && atoi(cfdr) != 0))
							load_configuration(connect_fast_fallback_config, FULL, YES);
						else
							printf("[CONNECT-FAST] DEFEAT_RESEAT: commander live-PHY reseat SKIPPED "
								"(fail-before; stranded at CONFIG_%d)\n", current_configuration);
					}
					if(robust_enabled == YES && is_robust_config(init_configuration))
						connect_fuse_floor_config = init_configuration;
					connect_fast_active = false;
					connect_fast_timer.stop();
					connect_fast_timer.reset();
				}

				// BW negotiation: if we support WB and currently NB, propose WB upgrade.
				// Don't check peer_capability here — MFSK ACK patterns don't carry data,
				// so peer_capability is unreliable. The responder's accept/reject comes
				// back in the SWITCH_BANDWIDTH LDPC ACK (data[1]).
				bool we_want_wb = (local_capability & CAP_WB_CAPABLE);
				bool currently_nb = (narrowband_enabled == YES);

				if(we_want_wb && currently_nb)
				{
					printf("[BW-NEG] Both WB-capable, initiating WB upgrade\n");
					fflush(stdout);
					wb_upgrade_pending = true;
					cleanup();
					add_message_control(SWITCH_BANDWIDTH);
					// CONNECT-SEED FUSION: the connect-evidenced seed is deterministic here, so
					// embed it in the SWITCH_BANDWIDTH frame (data[2]) - the RSP loads it on the
					// deferred WB switch and the CMD on the ACK, collapsing the separate robust
					// SET_CONFIG cross (~10 s at CONFIG_100 WB). Defeated -> length stays 2 (BASE).
					connect_fuse_seed_tx = CONFIG_NONE;
					if(connect_fuse_active() && messages_control.status != FREE &&
					   messages_control.data[0] == SWITCH_BANDWIDTH)
					{
						int fs = connect_fuse_seed_select(connect_seed_target(),
							connect_fuse_floor_config);
						// Always occupy data[2] on the FIX path so the RSP never reads a stale byte: a valid
						// config = the fused seed; 0xFF = no seed (fall back to the two-frame cross). length=3
						// matches SET_CONFIG so data[2] is buffer-delivered to the RSP (its length field is not).
						messages_control.data[2] = (char)(fs != CONFIG_NONE ? fs : 0xFF);
						messages_control.length = 3;
						if(fs != CONFIG_NONE)
						{
							connect_fuse_seed_tx = fs;
							printf("[BW-NEG] FUSE: embedding connect-seed config %d in SWITCH_BANDWIDTH (collapse SET_CONFIG cross)\n", fs);
							fflush(stdout);
						}
					}
					this->connection_status=TRANSMITTING_CONTROL;
				}
				// Option B (data-anchored promotion, 2026-05-29): the control-only
				// SNR-SUPERSHIFT probe is removed. Start data at the current config
				// (ROBUST_0 for -R gearshift) and let FRAME-UP climb rung-by-rung
				// ONLY on confirmed data delivery — control-ACK success no longer
				// promotes (§3 disease). The BW-negotiation branches above (the
				// `we_want_wb && currently_nb` SWITCH_BANDWIDTH path) still run
				// first; this is the no-BW-upgrade fall-through. Reaching here means
				// no SWITCH_BANDWIDTH was queued, so just begin data.
				// See fact-documents/gearshift-start-and-recovery.md §6.
				else
				{
					turboshift_active = false;
					turbo_supershift_announce_pending = false;
					turboshift_phase = TURBO_DONE;
					// CONNECT-SEED (gearshift-start-and-recovery.md §10.2): on a
					// CLEARLY-clean channel skip the ROBUST_0->1->2 crawl (the ~102 s
					// dominant climb cost) and open data near the SNR-appropriate WB
					// config. connect_seed_target() returns CONFIG_NONE on anything
					// not clearly clean (the over-seed guard: a marginal control-plane
					// SNR maps back to ROBUST_0), so the Option-B no-blind-climb start
					// is byte-identical for weak channels. A real seed is a robust->OFDM
					// TIER CROSS: route it through the EXISTING SET_CONFIG builder (the
					// proven fast cross with the dedicated ACK + the §3 reverse-robust
					// pin) — NOT a raw data start. The anchor is NOT raised here (the
					// seed is speculative, like the elevator); a non-viable seed BREAKs
					// back to the ROBUST floor (§10.4 invariant 2).
					int seed_cfg = connect_seed_target();
					// R (DUTY fast-start, climb-duty): connect-evidenced robust exit. When
					// connect_seed left us at the ROBUST floor (the common case -- connect_seed
					// rides the floor-pinned SNR proxy and is quarantined default-OFF), the
					// completed MFSK CONNECT handshake still proves the robust tier end-to-end.
					// So cross to the CONFIG_0 OFDM floor NOW via the SAME proven SET_CONFIG
					// tier-cross instead of airing a full robust DATA batch (~38 s) to earn the
					// identical CONFIG_0 probe. Meter-independent; guarded by the BREAK floor +
					// proven-ceiling ratchet; never lowers a (higher) connect_seed pick.
					if(is_robust_config(current_configuration))
					{
						int rce = robust_connect_exit_target();
						if(rce != CONFIG_NONE &&
						   (seed_cfg == CONFIG_NONE ||
						    config_ladder_index(rce) > config_ladder_index(seed_cfg)))
						{
							printf("[DUTY-R] connect-evidenced robust exit: seeding CONFIG_0 (skip robust dwell), config %d -> %d\n", current_configuration, rce);
							fflush(stdout);
							seed_cfg = rce;
						}
					}
					if(seed_cfg != CONFIG_NONE)
					{
						negotiated_configuration = seed_cfg;
						cleanup();
						add_message_control(SET_CONFIG);
						this->connection_status=TRANSMITTING_CONTROL;
					}
					else
					{
						this->connection_status=TRANSMITTING_DATA;
					}
				}
			}

		}
		else if(this->link_status==NEGOTIATING && messages_control.data[0]==SET_CONFIG)
		{
			this->link_status=CONNECTED;
			this->connection_status=TRANSMITTING_DATA;
			link_timer.start();
			watchdog_timer.start();
			gear_shift_timer.stop();
			gear_shift_timer.reset();
			connection_attempt_timer.stop();
			connection_attempt_timer.reset();

			std::string str="CONNECTED "+this->my_call_sign+" "+this->destination_call_sign+" "+ std::to_string(telecom_system->bandwidth)+"\r";
			tcp_socket_control.message->length=str.length();

			for(int i=0;i<tcp_socket_control.message->length;i++)
			{
				tcp_socket_control.message->buffer[i]=str[i];
			}
			tcp_socket_control.transmit();
		}
		else if(this->link_status==CONNECTED && messages_control.data[0]==KEY_EXCHANGE_1)
		{
			// KEY_EXCHANGE_1 ACK: responder's X25519 pubkey (32 bytes at data[1..32])
			// + confirmation tag (8 bytes at data[33..40])
			printf("[CRYPTO] KEY_EXCHANGE_1 ACKed — received responder's X25519 pubkey\n");
			fflush(stdout);

			// Length-checked KX read: the KEY_EXCHANGE_1 ACK carries pubkey(32) +
			// confirm tag(8) at data[1..40]. Verify the fixed control-frame copy
			// width holds at least the 32-byte pubkey before computing the shared
			// secret; reject a truncated frame instead of running over stale tail
			// bytes (the tag read at :6945 is bounded by the same width).
			int kx_avail = (max_data_length+max_header_length
			                - CONTROL_ACK_CONTROL_HEADER_LENGTH) - 1;
			if(cipher_suite.compute_x25519_shared_checked(
			       (const uint8_t*)&messages_control.data[1], kx_avail) != 0)
			{
				printf("[CRYPTO] FATAL: X25519 shared secret invalid (zero/low-order point or truncated KX frame)\n");
				fflush(stdout);
				this->link_status = DROPPED;
				reset_session_state();
				return;
			}

			// Derive session key from X25519 (classical-only for now)
			cipher_suite.derive_session_key(
				my_call_sign.c_str(), destination_call_sign.c_str(),
				(psk_hex[0] != '\0') ? (const uint8_t*)psk_hex : NULL,
				(psk_hex[0] != '\0') ? (int)strlen(psk_hex) : 0,
				false);  // mlkem_done=false (X25519-only for now)

#ifdef MERCURY_GUI_ENABLED
			// Display-only: X25519 shared computed -> advance the KX phase line.
			gui_set_kx_progress(cipher_suite.get_kx_phase(), 0, 0);
#endif

			// Verify responder's key confirmation tag (PSK mismatch detection)
			uint8_t our_tag[8];
			cipher_suite.compute_key_confirmation(our_tag);
			const uint8_t* peer_tag = (const uint8_t*)&messages_control.data[1 + X25519_KEY_SIZE];

			if(memcmp(our_tag, peer_tag, 8) != 0)
			{
				printf("[CRYPTO] PSK MISMATCH detected from KEY_EXCHANGE_1 ACK\n");
				fflush(stdout);

				// Report on control port
				const char* err_msg = "ENCRYPTION FAILURE PSK MISMATCH\r";
				int elen = (int)strlen(err_msg);
				for(int e=0; e<elen; e++)
					tcp_socket_control.message->buffer[e] = err_msg[e];
				tcp_socket_control.message->length = elen;
				tcp_socket_control.transmit();

#ifdef MERCURY_GUI_ENABLED
				g_gui_state.encryption_psk_mismatch.store(true);
				gui_push_monitor_event("[PSK MISMATCH — pre-shared key does not match, disconnecting]", true);
#endif
				// Still send KEY_ACTIVATE so the responder can also verify
				// the tag mismatch and disconnect cleanly on its side.
				messages_control.status = FREE;
				add_message_control(KEY_ACTIVATE);
				psk_mismatch_pending = true;

				watchdog_timer.start();
				link_timer.start();
			}
			else if(encryption_mode == ENCRYPT_STRICT)
			{
				// HYBRID ML-KEM upgrade (MLKEM_HYBRID_PLAN.md §5, STRICT/SNDL-safe).
				// X25519 confirm matched; now run KX2/KX3 over the data plane
				// BEFORE deriving the final hybrid key and activating. Generate the
				// ML-KEM keypair and start streaming the encaps key (KX2) chunks.
				// The classical key derived above is NOT activated (STRICT holds
				// data until the hybrid key is live) — it only seeded the X25519
				// confirm-tag check just performed. KEY_ACTIVATE is deferred to the
				// KX3-complete handler.
				printf("[CRYPTO] X25519 confirmed — beginning ML-KEM hybrid upgrade (KX2)\n");
				fflush(stdout);
				messages_control.status = FREE;
				if(cipher_suite.generate_mlkem_keypair(kx_mlkem_pk) != 0)
				{
					printf("[CRYPTO] FATAL: ML-KEM keypair generation failed\n");
					fflush(stdout);
					this->link_status = DROPPED;
					reset_session_state();
					return;
				}
				kx_mlkem_pk_ready = true;
				cipher_suite.set_kx_phase(KX_MLKEM_PK_SENT);
				if(kx_begin_chunk_send(KEY_EXCHANGE_2) < 0)
				{
					printf("[CRYPTO] FATAL: KX2 send setup failed\n");
					fflush(stdout);
					this->link_status = DROPPED;
					reset_session_state();
					return;
				}
				watchdog_timer.start();
				link_timer.start();
			}
			else
			{
				printf("[CRYPTO] Key confirmation matches — sending KEY_ACTIVATE\n");
				fflush(stdout);

				messages_control.status = FREE;
				add_message_control(KEY_ACTIVATE);

				watchdog_timer.start();
				link_timer.start();
			}
		}
		else if(this->link_status==CONNECTED
		        && (messages_control.data[0]==KEY_EXCHANGE_2
		            || messages_control.data[0]==KEY_EXCHANGE_3))
		{
			// HYBRID KX chunk ACK handling (CMD side). Two cases:
			//  (a) KEY_EXCHANGE_2 ACK: the RSP acknowledged a KX2 chunk we sent.
			//      Queue the next KX2 chunk; when all are sent, the RSP will
			//      encapsulate and stream KX3 (ciphertext) back — its ACK frames
			//      carry the KX3 chunks (data[] holds the 4B-headed chunk).
			//  (b) The ACK frame ALSO carries a KX3 chunk payload (RSP->CMD
			//      ciphertext). Feed it to the reassembler; on completion, decap
			//      + derive the hybrid key + send KEY_ACTIVATE.
			int code_in = (unsigned char)messages_control.data[0];
			messages_control.status = FREE;
			if(code_in == KEY_EXCHANGE_2)
			{
				kx_tx_next_index++;
				int more = kx_send_next_chunk();
				if(more == 1)
				{
					// next KX2 chunk queued
				}
				else
				{
					// All KX2 chunks sent; await KX3 from RSP. The RSP drives KX3
					// chunk frames; they arrive as KEY_EXCHANGE_3 control frames.
					cipher_suite.set_kx_phase(KX_MLKEM_CT_SENT);
					printf("[CRYPTO] KX2 complete (encaps key sent); awaiting KX3 ciphertext\n");
					fflush(stdout);
				}
				watchdog_timer.start();
				link_timer.start();
			}
			else // KEY_EXCHANGE_3 chunk arriving (RSP -> CMD ciphertext)
			{
				// FIX (data-flow-control-slot-lifecycle.md): the RX control-slot
				// producer (arq_responder.cc:830) hardcodes messages_control.length=1
				// while copying the FULL fixed slot width into data[] (:846). Pass the
				// REAL slot width — the same geometry the codec uses for capacity
				// (kx_chunk_payload_capacity() + the 4-byte chunk header) — NOT the
				// length=1 sentinel, which made kx_chunk_decode reject every chunk.
				int kx_frame_len = kx_chunk_payload_capacity()
				                   + cl_cipher_suite::KX_CHUNK_HEADER_LEN;
				int done = kx_receive_chunk((const uint8_t*)messages_control.data,
				                            kx_frame_len, KEY_EXCHANGE_3);
				if(done == 1)
				{
					// Full ciphertext reassembled — decapsulate + derive hybrid key.
					if(cipher_suite.decapsulate_mlkem(kx_mlkem_ct) != 0)
					{
						printf("[CRYPTO] FATAL: ML-KEM decapsulation failed\n");
						fflush(stdout);
						this->link_status = DROPPED;
						reset_session_state();
						return;
					}
					// Hybrid derive with full transcript binding (pk order is
					// ALWAYS commander-then-responder).
					const uint8_t* pk_cmd = cipher_suite.get_x25519_pubkey();
					const uint8_t* pk_rsp = cipher_suite.get_x25519_peer_pubkey();
					cipher_suite.derive_session_key(
						my_call_sign.c_str(), destination_call_sign.c_str(),
						(psk_hex[0] != '\0') ? (const uint8_t*)psk_hex : NULL,
						(psk_hex[0] != '\0') ? (int)strlen(psk_hex) : 0,
						true,                 // mlkem_done -> hybrid + pq_active
						kx_mlkem_ct, kx_mlkem_pk, pk_cmd, pk_rsp);
					cipher_suite.set_kx_phase(KX_HYBRID_DONE);
					printf("[CRYPTO] Hybrid session key derived — sending KEY_ACTIVATE (PQ)\n");
					fflush(stdout);
					add_message_control(KEY_ACTIVATE);
				}
				else if(done == 0)
				{
					// KX3 PER-CHUNK ACK (data-flow-control-slot-lifecycle.md §8.2(2)):
					// a NON-FINAL ciphertext chunk reassembled. Send a BARE
					// KEY_EXCHANGE_3 control request (length=1) back to the RSP — the
					// trigger for the RSP to advance kx_tx_next_index and queue the
					// next KX3 chunk (mirror of the CMD KX2-advance at :7184 driving
					// the RSP KX2 ACK). The slot is FREE here (set at :7183), so
					// add_message_control queues + enters TRANSMITTING_CONTROL. Only
					// the FINAL chunk path (done==1, above) sends KEY_ACTIVATE — the
					// CMD never KX3-ACKs the last chunk (mirror of the RSP not
					// bare-ACKing KX2-final). done<0 (chunk rejected) -> no ACK; the
					// RSP's pending chunk times out and retransmits via existing ARQ.
					add_message_control(KEY_EXCHANGE_3);
				}
				watchdog_timer.start();
				link_timer.start();
			}
		}
		else if(this->link_status==CONNECTED && messages_control.data[0]==KEY_ACTIVATE)
		{
			if(psk_mismatch_pending)
			{
				// KEY_ACTIVATE ACKed despite mismatch? Shouldn't happen, but disconnect.
				printf("[CRYPTO] KEY_ACTIVATE ACKed (PSK mismatch), disconnecting\n");
				fflush(stdout);
				psk_mismatch_pending = false;
				this->link_status = DROPPED;
				reset_session_state();
				return;
			}

			// KEY_ACTIVATE ACKed — encryption is now active on both sides
			cipher_suite.activate();
			l1_apply_reset_event(mercury::L1ResetEvent::CRYPTO_REKEY);
			// AEAD nonce sequence state: fresh per session+direction. First
			// batch (wire bsi=0) -> index 0. (data-flow-aead-nonce.md §init)
			tx_nonce_epoch = 0;
			tx_nonce_last_bsi = -1;
			rx_nonce_epoch = 0;
			rx_nonce_last_bsi = -1;
			tx_nonce_gen = 0;
			rx_nonce_gen = 0;
			tx_nonce_sealed_high_water = UINT64_MAX;
			rx_nonce_adopted_once = false;
			decrypt_delivered_bsi = -1;
			consecutive_auth_failures = 0;
			// KX-as-data (data-flow-hybrid-kex.md Â§3.4/Â§7): the KX streams consumed wire
			// bsi 0..N across the two R1 SWITCH_ROLE swaps; re-anchor the Option-W cursors +
			// de-dup high-water + wire batch_seq_id to a fresh bsi=0 baseline so the first
			// USER batch (this peer is now the commander) is byte-identical to a normal
			// session start. Env-gated OFF -> never called on the legacy pump path.
			if(kx_as_data_path()) kx_stream_reanchor();
#ifdef MERCURY_GUI_ENABLED
			g_gui_state.encryption_active.store(true);
			g_gui_state.encryption_psk_mismatch.store(false);
			// Display-only: PQ status, KX phase, and session fingerprint. Read
			// live so PQ auto-flips once ML-KEM transport merges.
			g_gui_state.encryption_pq_active.store(cipher_suite.is_pq_upgraded());
			gui_set_kx_progress(cipher_suite.get_kx_phase(), 0, 0);
			{
				char fp_hex[24];
				cipher_suite.get_fingerprint_hex(fp_hex, sizeof(fp_hex));
				gui_set_enc_fingerprint(fp_hex);
			}
#endif
			printf("[CRYPTO] KEY_ACTIVATE ACKed — encryption active (X25519 + ChaCha20-Poly1305)\n");
			fflush(stdout);

			// Report encryption state on control port
			{
				std::string str = "ENCRYPTION CLASSICAL\r";
				if (tcp_socket_control.get_status() == TCP_STATUS_ACCEPTED)
				{
					tcp_socket_control.message->length = str.length();
					for (int i = 0; i < (int)str.length(); i++)
						tcp_socket_control.message->buffer[i] = str[i];
					tcp_socket_control.transmit();
				}
			}

#ifdef MERCURY_GUI_ENABLED
			gui_push_monitor_event("[ENCRYPTION ACTIVE: X25519 + ChaCha20-Poly1305]", true);
#endif

			this->connection_status = TRANSMITTING_DATA;
			watchdog_timer.start();
			link_timer.start();
		}
		else if(this->link_status==CONNECTED && messages_control.data[0]==SWITCH_BANDWIDTH)
		{
			// ACK received = responder accepted (nb_only responders don't ACK at all)
			printf("[BW-NEG] SWITCH_BANDWIDTH accepted, switching to WB\n");
			fflush(stdout);
			wb_upgrade_pending = false;
			switch_narrowband_mode(NO);

			// Option B (data-anchored promotion, 2026-05-29): the control-only
			// SNR-SUPERSHIFT probe is removed. Start data at the current
			// (post-WB-upgrade) config — ROBUST_0 for -R gearshift — and let
			// FRAME-UP climb rung-by-rung ONLY on confirmed data delivery. The
			// WB-upgrade side effects above (switch_narrowband_mode etc.) already
			// ran. See fact-documents/gearshift-start-and-recovery.md §6.
			turboshift_active = false;
			turbo_supershift_announce_pending = false;
			turboshift_phase = TURBO_DONE;

			// CONNECT-SEED on the NB-start -> WB-upgrade path (gearshift-start-and-
			// recovery.md §10.5). The §10.2 seed lives only in the NO-BW-upgrade
			// fall-through (the `else` at the post-connect data-start, ~:6979). But
			// the COMMON live path STARTS NARROWBAND and takes the SWITCH_BANDWIDTH
			// upgrade above, so that seed was structurally bypassed (currently_nb was
			// true -> the BW-upgrade branch won; the seed `else` never ran). After
			// switch_narrowband_mode(NO) the CMD is WB and pinned at the ROBUST floor
			// (init_configuration, narrowband_enabled==NO) — the IDENTICAL seed
			// preconditions as :6979. So apply the SAME guarded seed here: a clearly-
			// clean connect SNR opens data near the SNR-appropriate WB config via the
			// proven SET_CONFIG tier-cross; connect_seed_target() returns CONFIG_NONE
			// (start ROBUST_0, byte-identical) on anything not clearly clean, so the
			// over-seed guard and weak-channel behaviour are unchanged. The anchor is
			// NOT raised (speculative, BREAK-recoverable, §10.4 invariant 2).
			if(connect_fuse_seed_tx != CONFIG_NONE)
			{
				// CONNECT-SEED FUSION apply (CMD): the RSP loaded this seed as part of the WB
				// switch (carried in the SWITCH_BANDWIDTH frame), so apply it here too via the
				// SAME cross steps - the separate SET_CONFIG frame airtime is eliminated.
				int seed = connect_fuse_seed_tx;
				connect_fuse_seed_tx = CONFIG_NONE;
				apply_connect_seed_cross(seed);
				// The responder loaded the same seed on the ACKed switch: a coordinated change,
				// not one to re-announce with a CONFIG_TAG on the first batch.
				if(current_configuration == seed)
					gs2_coordinated_wb_entry(current_configuration);
			}
			else
			{
				int seed_cfg = connect_seed_target();
				// R (DUTY fast-start, climb-duty): connect-evidenced robust exit. When
				// connect_seed left us at the ROBUST floor (the common case -- connect_seed
				// rides the floor-pinned SNR proxy and is quarantined default-OFF), the
				// completed MFSK CONNECT handshake still proves the robust tier end-to-end.
				// So cross to the CONFIG_0 OFDM floor NOW via the SAME proven SET_CONFIG
				// tier-cross instead of airing a full robust DATA batch (~38 s) to earn the
				// identical CONFIG_0 probe. Meter-independent; guarded by the BREAK floor +
				// proven-ceiling ratchet; never lowers a (higher) connect_seed pick.
				if(is_robust_config(current_configuration))
				{
					int rce = robust_connect_exit_target();
					if(rce != CONFIG_NONE &&
					   (seed_cfg == CONFIG_NONE ||
					    config_ladder_index(rce) > config_ladder_index(seed_cfg)))
					{
						printf("[DUTY-R] connect-evidenced robust exit: seeding CONFIG_0 (skip robust dwell), config %d -> %d\n", current_configuration, rce);
						fflush(stdout);
						seed_cfg = rce;
					}
				}
				if(seed_cfg != CONFIG_NONE)
				{
					negotiated_configuration = seed_cfg;
					cleanup();
					add_message_control(SET_CONFIG);
					this->connection_status=TRANSMITTING_CONTROL;
				}
				else
				{
					// The ACKed SWITCH_BANDWIDTH is itself the coordinated config change (see
					// gs2_coordinated_wb_entry): do not repeat it as a CONFIG_TAG on the first batch.
					gs2_coordinated_wb_entry(current_configuration);
					this->connection_status=TRANSMITTING_DATA;
				}
			}
		}
		else if(this->link_status==CONNECTED)
		{
			if (messages_control.data[0]==FILE_END_)
			{
				this->connection_status=TRANSMITTING_DATA;
				std::cout<<"end of file acked"<<std::endl;
			}
			else if (messages_control.data[0]==SET_LINK_PARAMS)
			{
				// SACK Design A Step 10 — SET_LINK_PARAMS ACKed by RSP.
				// CMD has already applied the new batch_size locally at decision
				// time (see policy_evaluate_axis2). The ACK confirms RSP also
				// applied. Both sides now agree on data_batch_size for the next
				// batch — proceed with DATA TX.
				printf("[CMD-LINK-PARAMS-ACKED] SET_LINK_PARAMS round-trip complete "
					"(local batch=%d) — resuming data TX\n", data_batch_size);
				fflush(stdout);
				// R1-rescope: this is THE control turnaround the guard is for — a new-
				// geometry (renegotiated batch-size) data batch about to key inside the
				// peer's TX->RX mute/flush window. Arm the post-control-turnaround wait.
				arm_control_turnaround_guard();
				this->connection_status=TRANSMITTING_DATA;
			}
			else if (messages_control.data[0]==ROBUST_DWELL_BATCH_OP)
			{
				// FIX-A P0 (adversarial-review fix, 2026-06-03) — CMD-side
				// ROBUST_DWELL_BATCH_OP ACK consumer. Exact parallel of the
				// SET_LINK_PARAMS branch above (the proven Axis-2 transport this op
				// mirrors): the CMD already applied the new robust dwell batch
				// locally at decision time (evaluate_robust_dwell_batch() →
				// set_data_batch_size()); this ACK confirms the RSP adopted it too
				// (arq_responder.cc ROBUST_DWELL_BATCH_OP handler), so both sides now
				// agree on data_batch_size for the next DATA batch.
				//
				// Without this branch the CMD had NO transition out of
				// RECEIVING_ACKS_CONTROL after the RSP ACKed → control-timeout →
				// spurious emergency BREAK → load_configuration(ROBUST_0) reverted
				// the batch to 1, on EVERY dwell raise AND revert (worse than the
				// batch=1 floor). data-flow-robust-tier-arq-batch.md §5.4.
				//
				// Restart both timers to match the RSP-side handler, which restarts
				// link_timer + watchdog_timer when it adopts the op
				// (arq_responder.cc:2762-2763) — keeps the round-trip symmetric.
				printf("[CMD-ROBUST-DWELL-ACKED] ROBUST_DWELL_BATCH_OP round-trip "
					"complete (local batch=%d) — resuming data TX\n", data_batch_size);
				fflush(stdout);
				// R1-rescope: same new-geometry (renegotiated batch-size) control
				// turnaround as SET_LINK_PARAMS — arm the post-control-turnaround wait.
				arm_control_turnaround_guard();
				this->connection_status=TRANSMITTING_DATA;
				watchdog_timer.start();
				link_timer.start();
			}
			// BLOCK_END eliminated — pattern ACK / silence is sole flow control.
			// finalize_block_commander() called directly after data ACK.
			else if (messages_control.data[0]==SWITCH_ROLE)
			{
				turbo_switch_role_retries = 0;  // Reset on success
				if(disconnect_requested==YES)
				{
					printf("[DISCONNECT] role transfer acknowledged; terminal authority ceded to new commander\n");
					fflush(stdout);
					disconnect_requested=NO;
				}
				// Asymmetric gearshift: swap forward/reverse for the return path.
				// KX-as-data: SKIP the swap in the pre-activation epoch — the reverse ct rides
				// the CURRENT settled config on BOTH peers (the RSP transmits it there too,
				// arq_responder.cc SWITCH_ROLE KX guard), so a swap here would desync the
				// receive config from the RSP's transmit config. data-flow-hybrid-kex.md §7.
				bool kx_epoch_swapback = (encryption_enabled && !cipher_suite.is_active() && kx_as_data_path());
				if(!kx_epoch_swapback && forward_configuration != CONFIG_NONE && reverse_configuration != CONFIG_NONE)
				{
					int tmp = forward_configuration;
					forward_configuration = reverse_configuration;
					reverse_configuration = tmp;

					// During turboshift, both sides are at the same mutual config.
					// Loading the swapped forward_configuration would corrupt it
					// with a stale reverse_configuration from earlier probes.
					if(turboshift_phase == TURBO_DONE)
					{
						if(forward_configuration != current_configuration)
						{
							data_configuration = forward_configuration;
							load_configuration(data_configuration, PHYSICAL_LAYER_ONLY, YES);
							printf("[GEARSHIFT] SWITCH_ROLE: loaded config %d for return path\n",
								forward_configuration);
						}
					}
					else
					{
						printf("[GEARSHIFT] SWITCH_ROLE during turboshift: staying at config %d\n",
							current_configuration);
						fflush(stdout);
					}
				}

				set_role(RESPONDER);
				this->link_status=CONNECTED;
				this->connection_status=RECEIVING;
				watchdog_timer.start();
				link_timer.start();
				last_message_received_type=NONE;
				last_message_sent_type=NONE;
				last_received_message_sequence=-1;
				// Clear messages_rx[] to prevent stale frames from previous phases
				// from being ACKed alongside legitimate data in the next batch.
				for(int i=0;i<nMessages;i++) messages_rx[i].status=FREE;
				messages_rx_buffer.status=FREE;
				// After SWITCH_ROLE, the buffer contains stale ACK pattern audio
				// from the ACK detection polling. MFSK ACK tones create false
				// OFDM preamble correlations → 10-15 LDPC FAILs before the real
				// frame shifts in. Flush with rx_mute to prevent capture thread
				// race (same pattern as send_ack_pattern).
				telecom_system->data_container.rx_mute = 1;
				// §5.7 ADD-ON1 (B5-class): gate-off the RX_MUTE drain guard — under
				// SIM_INPROC there is no async audio-callback thread to drain, so the
				// wait is moot (no-op); the circular_buf_reset below still fires.
				// Verbatim msleep(50) on production / paced sim.
				sim_inproc_rx_mute_settle(50); // RX_MUTE_GUARD_MS — let in-flight audio callbacks drain
				capture_reset_samples();
				{
					int buf_samples = telecom_system->data_container.Nofdm
						* telecom_system->data_container.buffer_Nsymb
						* telecom_system->data_container.interpolation_rate;
					MUTEX_LOCK(&capture_prep_mutex);
					memset(telecom_system->data_container.passband_delayed_data, 0,
						2 * buf_samples * sizeof(double));
					telecom_system->data_container.ring_write_index = 0;
					MUTEX_UNLOCK(&capture_prep_mutex);
				}
				telecom_system->data_container.rx_mute = 0;
				telecom_system->data_container.rx_mute_samples = 0;
				// SWITCH_ROLE: new commander needs ~120-170 symbols to process
				// role switch, fill batch, ptt_on, encode, TX. Ring buffer
				// preserves data in place (no shift_left drift), so we just
				// need enough callbacks for the turnaround.
				{
					int rx_frame = telecom_system->data_container.preamble_nSymb
						+ telecom_system->get_active_nsymb();
					// §17.5/§17.8 (§5 cross-layer fix): SWITCH_ROLE turns THIS peer into
					// the RESPONDER/receiver. data_configuration was (re)loaded at :4565
					// for the return path and may be CONFIG_16 + big-block framing. The
					// new receiver now awaits the new TX side's first DATA — at the
					// big-block rung that DATA is a big-block spanning
					// bigblock_rx_block_nsymb() (~64) OFDM symbols. Arming a STOCK frame
					// (~13) here snapshots only the block HEAD: cw0 fresh -> byte-correct,
					// cw1..cw7 decoded from stale ring -> deterministic garbage (the §17.2
					// truncated-window signature, on the REVERSE-direction / bidirectional
					// path the single-direction RSP test never exercised). Route through
					// bigblock_block_ftr_or() so the window spans the whole block whenever
					// the CFG16 big-block rung is live; off-rung (framing-off / M==MFSK /
					// config != CONFIG_16) the helper returns rx_frame+10 UNCHANGED, so the
					// stock per-frame turnaround is byte-identical.
					telecom_system->data_container.frames_to_read =
						bigblock_block_ftr_or(rx_frame + 10);
				}
				telecom_system->data_container.nUnder_processing_events = 0;
				telecom_system->receive_stats.delay_of_last_decoded_message = -1;
				telecom_system->receive_stats.mfsk_search_raw = 0;
				telecom_system->receive_stats.ofdm_search_raw = 0;
				telecom_system->receive_stats.ofdm_batch_active = false;
				batch_rx_frame_count = 0;
				last_received_end_of_batch_seq = -1;
			}
			else if (messages_control.data[0]==SET_CONFIG)
			{
				// SET_CONFIG ACK received: apply the new config now
				turbo_supershift_announce_pending = false;
				gear_shift_timer.stop();
				gear_shift_timer.reset();
				int prev_configuration = current_configuration;  // save before load
				if(data_configuration != current_configuration)
				{
					messages_control_backup();
					load_configuration(data_configuration, PHYSICAL_LAYER_ONLY, YES);
					messages_control_restore();
					printf("[GEARSHIFT] SET_CONFIG ACKed, loaded config %d\n", data_configuration);
					fflush(stdout);
					rate_opt.notify_switch_confirmed(opt_now_ms());
					if(rate_opt.controls_link())
						inband_confirm_coordinated_config(current_configuration);
					if(rate_opt.probe_is_ladder_step() && is_ofdm_config(current_configuration))
					{
						// Intermediate compatibility rungs need only one ordinary
						// DATA/SACK transaction. CONFIG_16 big-block framing is atomic at
						// its native K; truncating it to one frame leaves the following
						// full block in a stale decoder epoch.
						if(current_configuration == CONFIG_16)
						{
							printf("[GEARSHIFT-V2] ladder probe target=%d: native big-block goodput/SACK confirmation armed\n",
								current_configuration);
						}
						else
						{
							set_data_batch_size(1);
							printf("[GEARSHIFT-V2] ladder probe target=%d: one-frame goodput/SACK confirmation batch armed\n",
								current_configuration);
						}
						fflush(stdout);
					}
					// R1-rescope: a SET_CONFIG that CHANGED the geometry is a control
					// turnaround preceding a new-geometry data batch — arm the post-control-
					// turnaround wait so the first frame does not key inside the peer's
					// mute/PHY-reinit window (belt-and-suspenders with phy_reinit_settle_us,
					// and covers configs where that settle is 0). Consumed one-shot by the
					// next data keying; a downstream TRANSMITTING_CONTROL probe just defers it
					// to the eventual data batch.
					arm_control_turnaround_guard();
					// CFG16 CONTROL-ACK HOLD (cfg16-controlack-hold): mark the moment the
					// climb SET_CONFIG to the big-block rung is CONFIRMED (RSP ACKed on the
					// old PHY, both peers now loaded CFG16). After this the CMD must HOLD
					// CFG16 to DATA without emitting another OFDM control frame (the top-
					// config verification probe is skipped on the big-block rung; see the
					// "at the top config" branch). This log lets a HW capture SEE the rung
					// confirm independently of the DATA transition.
					if(telecom_system != NULL
						&& telecom_system->bigblock_framing_enabled
						&& telecom_system->M != MOD_MFSK
						&& data_configuration == CONFIG_16)
					{
						printf("[CFG16-HOLD] climb SET_CONFIG to CONFIG_16 (big-block rung) "
							"CONFIRMED — holding; verification probe will be skipped\n");
						fflush(stdout);
					}

					// Bug #60: PHY reinit race condition.
					// When modulation changes (e.g. 8PSK->32QAM), both sides do a full
					// PHY deinit+init. The responder sends its ACK BEFORE reinitializing
					// (the ACK uses the old config's frame structure). Without this delay,
					// the commander's next frame arrives while the responder is still
					// reinitializing -- lost frame -> BREAK -> cycle repeats forever.
					// 300ms covers worst-case RPi5 reinit (~100-200ms) with margin.
					// Phase-2 validation: --phy-reinit-settle-ms=N overrides.
					if(phy_reinit_settle_us > 0)
					{
						printf("[GEARSHIFT] Settling %dms for responder PHY reinit\n",
						       phy_reinit_settle_us / 1000);
						fflush(stdout);
						// §5.7 ADD-ON2: virtual-clock-ify the peer-PHY-reinit settle.
						// On production / paced sim this is the verbatim
						// usleep(phy_reinit_settle_us). Under SIM_INPROC a wall usleep
						// would freeze the shared clock while the peer (same thread)
						// reinits synchronously; route the settle through the pump so
						// the shared clock advances. Same wait semantics (elapsed >=
						// settle). Default phy_reinit_settle_us == 0 -> branch skipped.
						if(arq_sim_inproc_active())
							pumped_settle_wait(phy_reinit_settle_us / 1000);
						else
							usleep(phy_reinit_settle_us);
					}

					// Re-fill TX messages for the new config's message sizes
					for(int i=0;i<nMessages;i++)
					{
						messages_tx[i].status=FREE;
					}
					int data_read_size;
					for(int i=0;i<get_nTotal_messages();i++)
					{
						data_read_size=fifo_buffer_backup.pop(message_TxRx_byte_buffer,max_data_length+max_header_length);
						if(data_read_size!=0)
						{
							fifo_buffer_tx.push(message_TxRx_byte_buffer,data_read_size);
						}
						else
						{
							break;
						}
					}
					fifo_buffer_backup.flush();
				}

				// Break recovery: reverse-turboshift probing
				if(break_recovery_phase == 1)
				{
					// Phase 1 complete: coordination at ROBUST_0 succeeded.
					// Target config loaded. Now probe it with SET_CONFIG at target.
					printf("[BREAK-RECOVERY] Phase 1 done, probing config %d (2 tries)\n",
						current_configuration);
					fflush(stdout);
					break_recovery_phase = 2;
					break_recovery_retries = 2;
					cleanup();
					add_message_control(SET_CONFIG);
					this->connection_status = TRANSMITTING_CONTROL;
				}
				else if(break_recovery_phase == 2)
				{
					break_recovery_phase = 0;
					// 2026-05-24 fix: do NOT reset break_drop_step here. This site
					// fires when the SET_CONFIG ACK at the new target config is
					// received — i.e. the BREAK *handshake* succeeded — not when
					// data actually flows. The real "success" reset is at
					// arq_commander.cc:3074 in the data-ACK path (gated on
					// data_ack_received != NO). Resetting here defeated the
					// 1→2→4→4→4 escalation ladder: each consecutive BREAK was
					// crawling down one config step at a time, taking ~30s/step,
					// because the SET_CONFIG ACK between BREAKs always reset the
					// ladder. Observed in axis_walk_robust_v2 (WGN:14→0 sweep):
					// mercury walked cfg14→cfg6 in 8 single-step BREAKs over
					// 4+ minutes and ran out of dwell time before reaching
					// ROBUST_0. Now the ladder escalates across consecutive
					// BREAKs and only resets after a clean data batch.
					//
					// WALL-B FIX-2 (the missing AARF DECAY half, mirrors the
					// frame_shift_threshold decay at :3737): a Phase-2 ACK is
					// FORWARD PROGRESS (the peer followed the demote and ACKed the
					// recovery SET_CONFIG), so relax the step ONE NOTCH toward its
					// base floor. This is a single HALVE, NOT the reset the comment
					// above warns against -- the escalation ladder still grows
					// across consecutive UN-ACKed BREAKs (the WGN:14->0 sweep above
					// only ACKs once, at the end), but a link making coordination
					// progress no longer keeps a maximally-aggressive step. Floors
					// at the base aggression 2.
					break_drop_step = break_drop_step_after_handshake_decay(break_drop_step);

					if(turboshift_phase != TURBO_DONE)
					{
						// Turboshift ceiling recovery: both sides now at settle config.
						// Send SWITCH_ROLE to continue turboshift.
						printf("[BREAK-RECOVERY] Config %d verified, continuing turboshift\n",
							current_configuration);
						fflush(stdout);
						finish_turbo_direction();
					}
					else
					{
						int fifo_load = fifo_buffer_tx.get_size() - fifo_buffer_tx.get_free_size();
						printf("[BREAK-RECOVERY] Config %d verified, resuming data exchange (fifo=%d bytes, block_tx=%d)\n",
							current_configuration, fifo_load, block_under_tx);
						fflush(stdout);
						this->connection_status = TRANSMITTING_DATA;
					}
				}
				// Turboshift: keep climbing or finish direction
				else if(turboshift_active)
				{
					turboshift_last_good = prev_configuration;
					turboshift_retries = 1;  // reset retry for next config
					if(!config_is_at_top(current_configuration, robust_enabled, narrowband_enabled == YES)
						&& !(max_config_override >= 0 && current_configuration >= max_config_override))
					{
						// SNR-based supershift: prefer turbo_received_snr (from ACK suffix).
						// During TURBO_REVERSE, measurements.SNR_uplink is the FORWARD path
						// SNR (stale, from before role swap) — do NOT use it as fallback.
						// Only use turbo_received_snr (actual reverse-path feedback).
						double effective_snr;
						if(turbo_received_snr > -90)
							effective_snr = turbo_received_snr;
						else if(turboshift_phase != TURBO_REVERSE)
							effective_snr = measurements.SNR_uplink;
						else
							effective_snr = -99.0;  // Force incremental probing
						int snr_target = -1;
						if(is_ofdm_config(current_configuration) && effective_snr > -90)
						{
							snr_target = get_configuration(effective_snr - SUPERSHIFT_MARGIN_DB);
							int cfg_ceiling = (narrowband_enabled == YES) ? NB_CONFIG_MAX : topgear_wb_ceiling();
							if(snr_target > cfg_ceiling)
								snr_target = cfg_ceiling;
							if(supershift_proven_ceiling >= 0
							   && config_ladder_index(snr_target) > config_ladder_index(supershift_proven_ceiling))
								snr_target = supershift_proven_ceiling;
							if(max_config_override >= 0 && snr_target > max_config_override)
								snr_target = max_config_override;
							// Q-table handoff: turboshift stops where the optimizer takes over.
							apply_optimizer_handoff_cap_to_target(&snr_target);
						}

						if(snr_target > 0 && config_ladder_index(snr_target) > config_ladder_index(current_configuration))
						{
							negotiated_configuration = snr_target;
							printf("[TURBO] SNR-SUPERSHIFT: SNR=%.1f dB (turbo_rx=%.1f) -> config %d -> %d (direct, ceiling=%d/%d)\n",
								effective_snr, turbo_received_snr, current_configuration, negotiated_configuration, supershift_proven_ceiling, max_config_override);
						}
						else if(effective_snr > -90)
						{
							// SNR says ceiling is at or below current -- step-1 (cautious)
							negotiated_configuration = config_ladder_up_n(current_configuration, 1, robust_enabled, narrowband_enabled == YES);
							printf("[TURBO] SNR-capped step-1: config %d -> %d (snr=%.1f, target=%d)\n",
								current_configuration, negotiated_configuration, effective_snr, snr_target);
						}
						else
						{
							// 2026-05-29: cap blind no-SNR step at +1 (was +3); see twin sites ~3791, ~3986
							negotiated_configuration = config_ladder_up_n(current_configuration, 1, robust_enabled, narrowband_enabled == YES);
							printf("[TURBO] SUPERSHIFT: config %d -> %d (step 1, no SNR; caps applied)\n",
								current_configuration, negotiated_configuration);
						}
						// Enforce WB/NB ceiling on turboshift probe target
						{
							int turbo_cap = (narrowband_enabled == YES) ? NB_CONFIG_MAX : topgear_wb_ceiling();
							if(negotiated_configuration > turbo_cap)
								negotiated_configuration = turbo_cap;
							if(max_config_override >= 0 && negotiated_configuration > max_config_override)
								negotiated_configuration = max_config_override;
						}
						// ANCHOR-TIER-CROSSING DISCIPLINE (gearshift-climb-engine.md §16,
						// ROOT-2): the SNR-SUPERSHIFT / SNR-capped-step-1 / blind-step
						// targets above had NO last_data_viable_config clamp — only the
						// proven-ceiling + WB/NB ceiling. So once a turbo was launched at the
						// OFDM-entry rung on a CONTROL-plane SNR over-report (the §15 deep-SNR
						// mechanism), it ratcheted CONFIG_4 -> 9 on PHANTOM ACK matches with
						// no anchor bound — the residual breach edb8600's §15 elevator-only
						// gate did not cover. Route the turbo target through the SAME shared
						// chokepoint the §13/§14/§15 elevator uses (supershift_retrigger_target):
						//   - ROBUST anchor (WGN:-10): is_ofdm_config(anchor)=false ->
						//     high_confidence_jump=false -> the target is clamped to anchor+1,
						//     so a turbo launched at the OFDM boundary CANNOT ratchet up the
						//     OFDM tier; the data-fail BREAK path re-engages and falls back to
						//     ROBUST_0 (af14a9e restored on the turbo path too).
						//   - OFDM anchor (WGN:30): clamped to anchor + RETRIGGER_MAX_LEAP, so
						//     the fast multi-rung climb is PRESERVED (bounded), never past the
						//     proven anchor + MAX_LEAP.
						//   - optimizer_is_in_control(): returns the target unchanged (Q-table
						//     authority preserved — same exemption the elevator has).
						// The helper only ever LOWERS the target (never raises), so no new
						// over-climb is introduced; it READS but never RAISES the anchor.
						negotiated_configuration = supershift_retrigger_target(
							negotiated_configuration, effective_snr,
							last_data_viable_config, optimizer_is_in_control(),
							robust_enabled, narrowband_enabled == YES);
						// WALL-B FIX-5 hook #2 (turbo SNR-SUPERSHIFT probe target): the
						// LOAD-BEARING hook — this is the path that survives the BREAK->ROBUST_0
						// collapse and re-climbs (the limit cycle). After every other ceiling
						// clamp, AND the CFG16 big-block carve cooldown cap so the turbo probe
						// cannot re-elect the carve-dead CFG16 rung while the cooldown holds.
						// Index-monotone never-raise; no-op off the cooldown (batches==0).
						negotiated_configuration = apply_bigblock_cooldown_cap(negotiated_configuration);
						// CFG16 decode-margin gate on the LOAD-BEARING turbo re-climb hook: after
						// every other ceiling clamp, hold the probe target at CFG15 unless the
						// reverse-path SNR supports CFG16 (never-raise; identity at high SNR).
						negotiated_configuration = apply_cfg16_margin_cap(negotiated_configuration);
						// Per-rung floor gate on the LOAD-BEARING turbo re-climb hook (the path that
						// survives the BREAK->ROBUST crater and re-climbs): floor-cap the probe target
						// so the loop cannot re-elect the disproven rung (never-raise; audit §1 P6).
						negotiated_configuration = rung_floor_cap_fire(negotiated_configuration);
						if(snr_track_feature_enabled())
							negotiated_configuration =
								snr_track_apply_climb_cap(negotiated_configuration);
						// Guard: if target config is beyond SNR capability, do not probe.
						// Probing to an undecodable config leaves both sides stuck.
						//
						// §20 SACK-trust (gearshift-climb-engine.md): the truncation
						// decision is the PURE turbo_snr_truncates_probe() predicate. When
						// SACK Design A is negotiated it returns false — the
						// CONTROL-PLANE EVM-SNR estimate (which underreports 1-3 dB and
						// saturates ~14.5, jittering to ~9.0 on clean → get_configuration
						// = CONFIG_13 < a negotiated CFG15) must NOT pre-truncate the
						// climb, because the channel's data-carrying capability is PROVEN
						// by SACK delivery + the top-config verification probe below
						// (:4824-4845). This mirrors the DOWNSTREAM §7.13.38 SACK-trust at
						// :3953 — without this, the guard returned BEFORE
						// finish_turbo_direction() and the §7.13.38 SACK branch had
						// nothing to recover, pinning the ceiling one rung short (CFG14)
						// and never reaching CFG16 on clean. SACK OFF => legacy
						// fading-margin truncation preserved UNCHANGED.
						if(effective_snr > -90)
						{
							int snr_max_cfg = get_configuration(effective_snr);
							if(turbo_snr_truncates_probe(snr_max_cfg, negotiated_configuration, sack_v2_enabled))
							{
								printf("[TURBO] SNR %.1f too low for config %d (max=%d), finishing at %d\n",
									effective_snr, negotiated_configuration, snr_max_cfg, current_configuration);
								fflush(stdout);
								turboshift_last_good = current_configuration;
								finish_turbo_direction();
								return;
							}
						}
						// Retrigger clamp livelock guard: the clamp chain above can lower the target to at or below the
						// current config. Emitting SET_CONFIG then re-announces the config we are already on -- the responder
						// ACKs it, we re-enter here, and no DATA batch is ever dispatched (a turnaround livelock). When the
						// target is not a genuine climb, settle straight to DATA at the current config instead of emitting a
						// no-op control frame. Mirror the CFG16-HOLD terminal state, but do NOT pin the proven ceiling
						// (nothing was proven) and do NOT call finish_turbo_direction() (it would cap the ceiling at
						// start_config and block a later legitimate climb).
						if(config_ladder_index(negotiated_configuration) <= config_ladder_index(current_configuration))
						{
							printf("[TURBO] Retrigger clamp: target %d <= current %d -- suppressing no-op SET_CONFIG, "
								"settling to DATA at config %d\n",
								negotiated_configuration, current_configuration, current_configuration);
							fflush(stdout);
							turboshift_active = false;
							turbo_supershift_announce_pending = false;
							turbo_snr_ack_enabled = false;
							turbo_received_snr = -99.0f;
							turboshift_phase = TURBO_DONE;
							turboshift_last_good = current_configuration;
							negotiated_configuration = current_configuration;
							data_configuration = current_configuration;
							reverse_configuration = current_configuration;
							this->connection_status = TRANSMITTING_DATA;
							return;
						}
						fflush(stdout);
						cleanup();
						turbo_supershift_announce_pending = true;
						add_message_control(SET_CONFIG);
						this->connection_status=TRANSMITTING_CONTROL;
					}
					else
					{
						// At the top config. The SET_CONFIG that brought us here was decoded
						// at the PREVIOUS config's LDPC rate — so the top config is unverified.
						// Verify by sending SET_CONFIG(top) AT the top config. If the responder
						// can decode it, the config works and we can finish. If not, the existing
						// turboshift retry/BREAK recovery will settle at turboshift_last_good.
						//
						// CFG16 CONTROL-ACK HOLD (cfg16-controlack-hold, GAP1 root fix): the
						// verification probe (the last else-branch below) re-sends
						// SET_CONFIG(top) AS A SINGLE OFDM CONTROL FRAME on the live PHY. At
						// the CFG16 big-block rung that control frame is STRUCTURALLY
						// UNDECODABLE by the responder: the RSP's receive_byte routes ANY
						// CFG16 OFDM audio into the block carver (telecom_system.cc:1024 /
						// the arq_common.cc carve gate), so the single-frame SET_CONFIG is
						// carved as a fake K-codeword block, never parsed as control, never
						// ACKed -> [TURBO] RETRY -> CEILING -> [TX-BREAK] back to ROBUST_0,
						// BEFORE any DATA batch is dispatched at CFG16 ([TXCW]=0). The
						// big-block TX is innocent; the verification turnaround is the
						// failure. NO control turnaround survives on the CFG16 OFDM PHY (any
						// SET_CONFIG/ACK is a single frame the carver eats), so the ONLY way
						// to HOLD CFG16 is to NOT emit a control frame here: skip the
						// redundant probe and transition straight to DATA. CFG16 viability
						// is then proven by the FIRST DATA big-block + its K-bit SACK -- the
						// SAME SACK-trust authority finish_turbo_direction() already uses for
						// the verified ceiling (sec 7.13.38, arq_common.cc:3958-3969) and that
						// turbo_snr_truncates_probe() honors above (:4862-4874). GATED
						// STRICTLY on the big-block rung being live; off the rung the
						// verification handshake is UNCHANGED (byte-identical) -- at lower
						// OFDM configs the SET_CONFIG control frame IS decoded normally (the
						// carve gate is false below CONFIG_16).
						bool bigblock_rung_live =
							(telecom_system != NULL
							 && telecom_system->bigblock_framing_enabled
							 && telecom_system->M != MOD_MFSK
							 && current_configuration == CONFIG_16);
						if(bigblock_rung_live)
						{
							printf("[CFG16-HOLD] Top config %d on big-block rung -- skipping "
								"undecodable OFDM verification probe, holding CFG16 (SACK-trust "
								"ceiling; first DATA block + K-bit SACK proves viability)\n",
								current_configuration);
							fflush(stdout);
							// Mirror finish_turbo_direction()'s skip-reverse terminal state
							// (turbo done, ceiling pinned at the verified top), but transition
							// DIRECTLY to DATA instead of emitting a SET_CONFIG control frame
							// the responder cannot decode. No config change (top==current), so
							// no PHY reload / SET_CONFIG announcement is needed.
							turboshift_active = false;
							turbo_supershift_announce_pending = false;
							turbo_snr_ack_enabled = false;
							turbo_received_snr = -99.0f;
							turboshift_phase = TURBO_DONE;
							turboshift_last_good = current_configuration;
							supershift_proven_ceiling = current_configuration;
							data_configuration = current_configuration;
							negotiated_configuration = current_configuration;
							reverse_configuration = current_configuration;
							printf("[BBTX-GATE] CFG16 held -- transition TRANSMITTING_DATA "
								"(ceiling=%d, proven_ceiling=%d); next send_batch routes a DATA "
								"big-block to bigblock_send_one_block\n",
								turboshift_last_good, supershift_proven_ceiling);
							fflush(stdout);
							this->connection_status = TRANSMITTING_DATA;
						}
						else if(turboshift_last_good == current_configuration)
						{
							// Second pass: verification probe ACKed. Top config works.
							printf("[TURBO] Top config %d verified, finishing\n", current_configuration);
							fflush(stdout);
							finish_turbo_direction();
						}
						else
						{
							printf("[TURBO] Reached top at config %d, sending verification probe\n",
								current_configuration);
							fflush(stdout);
							negotiated_configuration = current_configuration;
							cleanup();
							add_message_control(SET_CONFIG);
							this->connection_status = TRANSMITTING_CONTROL;
						}
					}
				}
				else
				{
					// Supershift re-trigger: after a ladder gearshift SET_CONFIG success,
					// fresh OFDM SNR is available. If it suggests we're far below optimal,
					// re-enter turboshift to jump ahead (rise fast, fall slow).
					bool retriggered = false;
					if(!rate_opt.controls_link() &&
					   turboshift_phase == TURBO_DONE && gear_shift_on == YES &&
					   is_ofdm_config(current_configuration) && measurements.SNR_uplink > -90)
					{
						// CONTROLLED ELEVATOR (fork (1), gearshift-climb-engine.md sec 13/14):
						// the SNR-driven re-trigger MAY jump multiple rungs toward the SNR-ideal
						// config in ONE shot -- but ONLY under the high-confidence-SNR predicate
						// inside supershift_retrigger_target(). The cap-chain
						// (get_configuration -> NB cap -> supershift_proven_ceiling cap -> the
						// helper) is now extracted VERBATIM into the SHARED
						// elevator_target_from_snr() method so this site and the FRAME-UP elevator
						// (piece B) cannot drift. Pre-(1) (af14a9e) this path was HARD-CLAMPED to
						// last_data_viable_config+1, making the re-trigger a no-op; the helper keeps
						// the +1 clamp at LOW/INVALID SNR (DEEP-SNR INERT) and relaxes it only at
						// clearly-high SNR, capped at min(supershift_proven_ceiling, WB/NB ceiling),
						// READING but never RAISING the anchor. See arq.h + sec 6/7/13/14.
						int snr_ideal = elevator_target_from_snr();
						int gap = config_ladder_index(snr_ideal) - config_ladder_index(current_configuration);
						if(gap >= SUPERSHIFT_RETRIGGER_CONFIGS)
						{
							printf("[TURBO] RE-TRIGGER: SNR=%.1f dB suggests config %d (current %d, gap=%d, ceiling=%d)\n",
								measurements.SNR_uplink, snr_ideal, current_configuration, gap, supershift_proven_ceiling);
							fflush(stdout);
							turboshift_active = true;
							turboshift_phase = TURBO_FORWARD;
							turboshift_initiator = true;
							turbo_snr_ack_enabled = true;
							turbo_received_snr = -99.0f;
							turbo_best_snr = -99.0f;
							turboshift_last_good = current_configuration;
							turboshift_retries = 1;
							negotiated_configuration = snr_ideal;
							if(snr_track_feature_enabled())
								snr_track_meter_window_reset();
							cleanup();
							add_message_control(SET_CONFIG);
							this->connection_status = TRANSMITTING_CONTROL;
							retriggered = true;
						}
					}
					if(!retriggered)
					{
						// Frame gearshift applied — if data fails immediately, BREAK
						if(data_configuration != prev_configuration)
						{
							frame_gearshift_just_applied = true;
							frame_gearshift_retry_count = 0;  // §7.13.33 fresh attempt
						}
						this->connection_status=TRANSMITTING_DATA;
					}
				}
				watchdog_timer.start();
				link_timer.start();
			}
		}

		else if(this->link_status==DISCONNECTING && messages_control.data[0]==CLOSE_CONNECTION)
		{
			reset_session_state();
			load_configuration(init_configuration,FULL,YES);
			this->link_status=LISTENING;
			this->connection_status=RECEIVING;
			reset_all_timers();
			// Reset RX state machine - wait for fresh data (prevents decode of self-received TX audio)
			telecom_system->data_container.frames_to_read =
				telecom_system->data_container.preamble_nSymb + telecom_system->data_container.Nsymb;
			telecom_system->data_container.nUnder_processing_events = 0;

			fifo_buffer_tx.flush();
			fifo_buffer_backup.flush();
			fifo_buffer_rx.flush();

			// Reset messages_control so new CONNECT commands can work
			messages_control.status=FREE;

			set_role(RESPONDER);

			emit_disconnected_once();
		}

		// Bug #41: After processing any ACKed control message, free the slot so
		// the next handshake message can be queued. Without this, messages_control
		// stays ACKED after START_CONNECTION ACK, and add_message_control(TEST_CONNECTION)
		// in process_messages_commander returns ERROR_ (status != FREE).
		// cleanup() is idempotent: branches that already call it (SET_CONFIG, etc.)
		// will find status=FREE on the second call, which is a no-op.
		cleanup();
	}
}

// Shared BLOCK_END state transitions: reset block, flush backup, stats, gearshift.
// Called from both explicit BLOCK_END ACK handler and implicit path (batch_size==1).
// ---------------------------------------------------------------------------
// Block-ACK gearshift feed (MERCURY_L1_BLOCKACK_GEARFEED). Under block-ACK the
// responder returns ONE aggregate BLOCK_SACK per N forward batches, so the
// commander sees a single clean-ACK pass for a whole block. The gearshift climb
// feeds on PER-BATCH evidence -- the FRAME-UP consecutive_data_acks streak, the
// sustained-anchor clean_batches_at_current_config streak, the block-level clean
// rate (last_transmission_block_stats), and the Axis-2/3 controllers -- so a
// block collapses N batches into ONE observation and the climb starves at ~1/N
// its rate: armed cells cap at cfg15. This feed reconstructs the N-1 SILENT
// intermediate batches' evidence at aggregate-apply time so the optimizer ends
// each block in the state N per-batch responses would have produced.
//
// A partial aggregate DROPs the session at both apply sites, so an APPLIED
// aggregate that reaches the climb path is all-clean (every covered batch fully
// delivered); the intermediate batches are therefore replayed as clean. The
// gate returns 0 unless the feed is armed AND the aggregate was all-received, so
// a degraded aggregate (which never reaches here in production) yields no climb
// credit -- the protection direction holds by construction. Off, every consumer
// adds 0 / runs 0 iterations => byte-identical.
int cl_arq_controller::l1_gearfeed_extra_batches() const
{
	if(!l1_block::gearfeed_enabled() || !l1_blockack.enabled()
	   || !l1_blockack.last_ack_all_received())
		return 0;
	const std::size_t n = l1_blockack.last_ack_batch_count();
	return (n >= 2) ? (int)(n - 1) : 0;
}

void cl_arq_controller::l1_gearfeed_replay_intermediate()
{
	if(l1_gearfeed_extra_batches() <= 0)
		return;
	// Replay the SILENT intermediate batches from the aggregate's actual
	// per-batch breakdown, in transmit order. The clean-ACK funnel already
	// contributed the boundary (last) batch's evidence, so replay batches
	// 0..n-2. Each batch is credited clean/partial from its own acked/span (an
	// applied aggregate is all-clean today, since a partial DROPs; carrying the
	// per-batch truth keeps this correct if that policy ever changes).
	// Per-batch bytes and elapsed cycle time do not ride the aggregate. Replay
	// therefore contributes outcome-valid clean/partial frame evidence only; it
	// never fabricates rate evidence or changes the application-goodput numerator.
	const std::vector<l1_block::PerBatchAck>& batches =
		l1_blockack.last_applied_batches();
	int replayed = 0;
	for(std::size_t i = 0; i + 1 < batches.size(); ++i)
	{
		const l1_block::PerBatchAck& b = batches[i];
		const bool clean = (b.acked_slots >= b.span_slots);
		last_transmission_block_stats.nBatches_acked++;
		if(clean)
			last_transmission_block_stats.nBatches_fully_acked++;
		opt_record_batch(0u, /*sack_used=*/!clean, /*failed=*/false,
		                 /*frames_acked=*/(int)b.acked_slots,
		                 /*frames_sent_override=*/(unsigned int)b.span_slots,
		                 /*outcome_only=*/true);
		{
			int rec = current_configuration;
			if(opt_evaluate_batch_end(&rec))
				opt_pending_switch_cfg = rec;
		}
		if(sack_v2_enabled)
		{
			policy_evaluate_axis2((int)b.acked_slots, (int)b.span_slots);
			axis3_batch_tick();
		}
		replayed++;
	}
	printf("[L1-GEARFEED] replayed intermediate batches=%d block_batches=%zu cfg=%d\n",
		replayed, batches.size(), current_configuration);
	fflush(stdout);
}


void cl_arq_controller::finalize_block_commander()
{
	for(int i=0;i<this->nMessages;i++)
	{
		this->messages_tx[i].ack_timeout=0;
		this->messages_tx[i].id=0;
		this->messages_tx[i].length=0;
		this->messages_tx[i].nResends=0;
		this->messages_tx[i].status=FREE;
		this->messages_tx[i].type=NONE;
	}
	block_under_tx=NO;
	fifo_buffer_backup.flush();

	// Streaming: commit context after successful data ACK
	if(compressor.is_streaming())
		compressor.commit_pending();

	// Application-goodput objective: credit original useful bytes once, only
	// after the atomic batch has been acknowledged/assembled. SACK-banked
	// transport fragments remain a separate progress signal.
	if(batch_uncompressed_size > 0)
	{
		const int committed_application_bytes = batch_uncompressed_size;
		opt_commit_application_bytes((unsigned int)committed_application_bytes);
		// The atomic unit is now delivered.  Clear it before evaluating remaining
		// work so a final transfer cannot justify a switch using bytes that have
		// already reached the application.
		batch_uncompressed_size = 0;
		int rec = current_configuration;
		if(opt_evaluate_batch_end(&rec))
			opt_pending_switch_cfg = rec;
#ifdef MERCURY_GUI_ENABLED
		gui_add_throughput_bytes_tx(committed_application_bytes);
#endif
	}
	else
	{
		batch_uncompressed_size = 0;
	}

	// Success rate: use batch-level metric for pattern ACK (all-or-nothing ACK).
	// Frame-level nReSent/nSent is poisoned by ACK-loss retransmissions that aren't
	// real OFDM failures. Batch-level = % of batches that got ACKed.
	//
	// success_rate_data counts BOTH clean and partial-SACK-recovered batches
	// (nBatches_acked) — UNCHANGED. It drives the DOWN-shift trigger / reset
	// (:4667/:4736 + axis1 twin) and all [GEARSHIFT] logging; a partial SACK is a
	// genuine delivery for those, so its semantics must not change.
	//
	// CLEAN-BATCH VIABILITY (§9): the UP-promotion gate uses a SEPARATE clean-only
	// rate (success_rate_data_clean, from nBatches_fully_acked). A partial-only run
	// at a marginal rung would otherwise read success_rate_data=100% (every batch
	// "acked" via a partial SACK) and clear the 85% LADDER-UP gate, climbing into a
	// config that can't pass full data. The clean rate makes such a run read ~0% so
	// the up-gate holds, WITHOUT perturbing the down-shift/logging path. See §9.
	if(ack_pattern_time_ms > 0 && last_transmission_block_stats.nBatches_sent > 0)
	{
		last_transmission_block_stats.success_rate_data = 100.0 *
			last_transmission_block_stats.nBatches_acked /
			last_transmission_block_stats.nBatches_sent;
		success_rate_data_clean = 100.0 *
			last_transmission_block_stats.nBatches_fully_acked /
			last_transmission_block_stats.nBatches_sent;
	}
	else if(last_transmission_block_stats.nSent_data > 0)
	{
		last_transmission_block_stats.success_rate_data=100*(1-((float)last_transmission_block_stats.nReSent_data/(float)last_transmission_block_stats.nSent_data));
		if(last_transmission_block_stats.success_rate_data < 0)
			last_transmission_block_stats.success_rate_data = 0;
		// CLEAN-BATCH VIABILITY (§9): non-pattern-ACK (frame-level) path has no
		// batch clean/partial distinction — the clean rate tracks the same metric.
		success_rate_data_clean = last_transmission_block_stats.success_rate_data;
	}
	else
	{
		last_transmission_block_stats.success_rate_data=100;
		success_rate_data_clean = 100.0;  // CLEAN-BATCH VIABILITY (§9) — no data sent
	}

	// 2D channel-state observability — see fact-doc channel-state-2d-lookup.md §8 Step 2.
	// Step 5: when --channel-lookup loaded a table, also emit a [CHANNEL-LOOKUP]
	// proposal line. OBSERVATION ONLY — proposal never acted on; existing
	// gearshift / Q-table optimizer remain in control.
	{
		double cs_snr = telecom_system->get_correlator_snr_proxy();
		double cs_sel = telecom_system->get_channel_selectivity();
		bool snr_real = (cs_snr != -99.0);
		bool sel_real = (cs_sel != -1.0);
		if(snr_real || sel_real)
		{
			printf("[CHANNEL-STATE] snr_proxy=%+6.1f selectivity=%5.3f cfg=%d\n",
				cs_snr, cs_sel, current_configuration);
			fflush(stdout);
		}
		// Only consult the table when it loaded AND both axes are real
		// measurements (one sentinel + one real would clamp into a bin
		// that doesn't reflect the channel and emit a misleading proposal).
		if(channel_lookup.is_loaded() && snr_real && sel_real)
		{
			int proposed = channel_lookup.lookup(cs_snr, cs_sel);
			if(proposed == cl_channel_state_lookup::SENTINEL_NO_DATA)
			{
				printf("[CHANNEL-LOOKUP] proposed_cfg=-1 (NO_DATA) current=%d\n",
					current_configuration);
			}
			else if(proposed == cl_channel_state_lookup::SENTINEL_DEAD)
			{
				printf("[CHANNEL-LOOKUP] proposed_cfg=-2 (DEAD) current=%d\n",
					current_configuration);
			}
			else
			{
				printf("[CHANNEL-LOOKUP] proposed_cfg=%d (current=%d, delta=%+d)\n",
					proposed, current_configuration, proposed - current_configuration);
			}
			fflush(stdout);
		}
	}

	last_transmission_block_stats.nReSent_data=0;
	last_transmission_block_stats.nSent_data=0;
	last_transmission_block_stats.nBatches_sent=0;
	last_transmission_block_stats.nBatches_acked=0;
	last_transmission_block_stats.nBatches_fully_acked=0;  // CLEAN-BATCH VIABILITY (§9)
	std::string str="BUFFER ";
	str+=std::to_string(fifo_buffer_tx.get_size()-fifo_buffer_tx.get_free_size());
	str+='\r';
	for(long unsigned int i=0;i<str.length();i++)
	{
		tcp_socket_control.message->buffer[i]=str[i];
	}
	tcp_socket_control.message->length=str.length();
	tcp_socket_control.transmit();

	if(gear_shift_on==YES)
	{
		// An ACTIVE-v2 evaluation may have queued an Axis-2 confirmation
		// while finalizing the just-ACKed application unit. Preserve that
		// control transaction; the legacy post-block policy must not overwrite
		// TRANSMITTING_CONTROL with TRANSMITTING_DATA before it reaches the peer.
		if(rate_opt.controls_link() && messages_control.status != FREE)
		{
			printf("[GEARSHIFT-V2] preserving queued control code=%d across post-block policy\n",
				(int)(unsigned char)messages_control.data[0]);
			fflush(stdout);
		}
		else if(gear_shift_algorithm==SNR_BASED)
		{
			cleanup();
			add_message_control(TEST_CONNECTION);
		}
		else if(gear_shift_algorithm==SUCCESS_BASED_LADDER)
		{
			// WALL-B FIX-5: DECREMENT the CFG16 big-block carve cooldown ONCE PER COMPLETED
			// BATCH. finalize_block_commander() runs exactly once per block/batch (both the
			// SACK-v2 policy path and the v1 inline path below flow through here), the SAME
			// per-batch cadence as gear_shift_blocked_for_nBlocks / ceiling_success_count.
			// This is the EXPIRY half (audit R6): the unit is BATCHES not receive() ticks —
			// decrementing per tick would collapse the window to milliseconds. On reaching 0
			// the CFG16 rung is re-armed for an optimistic re-probe (cheap: one CFG16 election
			// attempt); a carve-success then clears it permanently, a re-failure re-arms at
			// BASE. See fix5/FIX5_DESIGN.md §4.5.
			if(bigblock_carve_cooldown_batches > 0)
			{
				bigblock_carve_cooldown_batches--;
				if(bigblock_carve_cooldown_batches == 0)
				{
					printf("[CFG16-HOLD] FIX-5 EXPIRED: CFG16 big-block carve cooldown elapsed "
						"(span was %d) — CFG16 rung re-armed for an optimistic re-probe\n",
						bigblock_carve_cooldown_span);
					fflush(stdout);
				}
			}

			// SACK Design A Step 9 — Multi-axis policy framework dispatch.
			//
			// v2 sessions route through the new policy framework entry point
			// `policy_evaluate_axis1()`. The body is functionally identical to
			// the v1 inline path below (same observable, same action, same
			// hysteresis); the wrapper adds the §4.3.4 invariant-5
			// `[POLICY-MOVE]` log line and the invariant-6 supremacy hook so
			// when Axes 2/3 land (Steps 10/11) they can observe Axis-1 moves
			// and the supremacy contract is already enforced.
			//
			// v1 sessions (sack_v2_enabled==false) take the inline path
			// verbatim — no behavior change, no new log lines, byte-identical
			// log surface (proven by WAV harness v1<->v1 sha256 stability).
			//
			// Per §3.8 "initiator controls flow": both branches are CMD-side
			// only; RSP never participates in Axis 1 decision-making — it
			// just executes the resulting SET_CONFIG.
			if(sack_v2_enabled)
			{
				policy_evaluate_axis1();
			}
			else
			{
			gear_shift_blocked_for_nBlocks++;
			// Reset downshift failure counter on any good block
			//
			// SACK_DESIGN_A_PLAN §7.13.24: a decay variant (counter--) was tried
			// and REVERTED — it triggered axis=1 ladder_down too early in an
			// SNR descent, hit the axis-1 cooldown, and then BREAK (a separate
			// failsafe that pre-fix carried the descent the rest of the way)
			// stopped firing because some shared state was disturbed. Measured
			// outcome: bps dropped to 0 at WGN:28 (vs WGN:24 pre-fix) and link
			// never reached ROBUST_0 floor. The original strict-reset is the
			// right policy until the interaction with BREAK descent is
			// understood and the cooldown is rethought.
			if(last_transmission_block_stats.success_rate_data >= gear_shift_down_success_rate_precentage
			   || optimizer_owns_normal_downshift())
				gear_shift_down_consecutive_fails = 0;
			// Handoff: upward moves retain the monitor branch's calibrated-band
			// optimizer ownership. Ordinary success-rate LADDER DOWN yields only when
			// optimizer_owns_normal_downshift() has enough aligned evidence; BREAK
			// remains independent.
			// CLEAN-BATCH VIABILITY (§9): the UP gate uses the CLEAN-only rate so a
			// partial-only run (success_rate_data may read 100% from SACK recoveries)
			// does NOT clear the 85% threshold and climb into a non-viable config.
			if(success_rate_data_clean>gear_shift_up_success_rate_precentage
				&& gear_shift_blocked_for_nBlocks>= gear_shift_block_for_nBlocks_total
				&& !optimizer_is_in_control())
			{
				{
					int proposed = config_ladder_up(current_configuration, robust_enabled, narrowband_enabled == YES);
				// Respect proven ceiling — don't re-try configs that already failed during turboshift
				bool ceiling_blocked = (supershift_proven_ceiling >= 0 &&
					config_ladder_index(proposed) > config_ladder_index(supershift_proven_ceiling))
					|| (max_config_override >= 0 && proposed > max_config_override);
				// Option B (data-anchored promotion): block any LADDER UP whose
				// destination is > 1 rung above the highest data-viable rung,
				// unless the Q-table optimizer owns the band. See §6/§7.
				if(!optimizer_is_in_control() &&
					config_ladder_index(proposed) > config_ladder_index(last_data_viable_config) + 1)
					ceiling_blocked = true;
				// FIX-B — floor-probe back-off (gearshift-floor-probe-backoff.md §5.2,
				// gate-site #3, legacy v1 inline LADDER-UP twin). AND the per-rung
				// suppression into the EXISTING up gate. INV-1: only turns a PERMITTED
				// probe OFF. INV-2: UP path only; no downward site references back-off.
				if(probe_rung_suppressed(proposed))
					ceiling_blocked = true;
				// WALL-B FIX-5 hook #1 (LADDER UP, v1 inline twin): block a step above the
				// CFG16 big-block carve cooldown ceiling (CFG15) while the cooldown is armed.
				if(apply_bigblock_cooldown_cap(proposed) != proposed)
					ceiling_blocked = true;
				// CFG16 decode-margin gate (LADDER UP, v1): the success-based ladder step is
				// SNR-blind, so block the 15->16 step until the reverse-path SNR supports CFG16.
				if(apply_cfg16_margin_cap(proposed) != proposed)
					ceiling_blocked = true;
				// Per-rung floor gate (LADDER UP, v1): block a success-based up-step to a rung the
				// reverse-path SNR does not support at its measured floor (audit §1 P7; never-raise).
				if(apply_rung_floor_cap(proposed) != proposed)
					ceiling_blocked = true;
				if(!config_is_at_top(current_configuration, robust_enabled, narrowband_enabled == YES) && !ceiling_blocked)
				{
					negotiated_configuration=proposed;
					printf("[GEARSHIFT] LADDER UP: success=%.0f%% > %.0f%%, config %d -> %d\n",
						last_transmission_block_stats.success_rate_data, gear_shift_up_success_rate_precentage,
						current_configuration, negotiated_configuration);
					fflush(stdout);
					cleanup();
					add_message_control(SET_CONFIG);
					opt_reset_window();
					rate_opt.force_cooldown(5);
				}
				else
				{
					// Ceiling recovery: after N consecutive good blocks at ceiling, raise ceiling by 1.
					// Threshold reduced 20 → 5 (2026-05-24, band-aid for panic-stuck-at-floor):
					// at ROBUST_0 with 1-3 bps, 20 good blocks takes 10+ min and a full
					// ceiling-climb back to original takes 2+ hours. With 5 blocks the
					// recovery window shrinks ~4x — still gives oscillation protection
					// because each raise must accumulate fresh successes at the new
					// ceiling. The proper fix is the 2D channel-measurement table; this
					// is the band-aid until that's built.
					// WALL-B FIX-5 hook #1b (CEILING RECOVERY, v1): never raise the proven ceiling
					// above the cooldown ceiling — else it would walk the ceiling back to CFG16.
					// AND never raise it while the CFG16 decode-margin gate holds CFG15 (same reason).
					// AND never raise it above a rung whose per-rung measured floor is not cleared
					// (audit §1 P8: the proven ceiling can never walk back above a closed floor).
					if(ceiling_blocked && apply_bigblock_cooldown_cap(proposed) == proposed
						&& apply_cfg16_margin_cap(proposed) == proposed
						&& apply_rung_floor_cap(proposed) == proposed)
					{
						ceiling_success_count++;
						if(ceiling_success_count >= 5)
						{
							int old_ceiling = supershift_proven_ceiling;
							supershift_proven_ceiling = proposed;  // raise ceiling to what we wanted to try
							ceiling_success_count = 0;
							printf("[GEARSHIFT] CEILING RECOVERY: %d -> %d after %d good blocks\n",
								old_ceiling, supershift_proven_ceiling, 5);
							fflush(stdout);
							// Don't shift up yet — let the next block's ladder evaluation do it
						}
					}
					else
					{
						ceiling_success_count = 0;
					}
					printf("[GEARSHIFT] LADDER: at top (config %d), success=%.0f%%%s\n",
						current_configuration, last_transmission_block_stats.success_rate_data,
						ceiling_blocked ? " [ceiling-limited]" : "");
					fflush(stdout);
					this->connection_status=TRANSMITTING_DATA;
				}
				}
			}
			else if(last_transmission_block_stats.success_rate_data<gear_shift_down_success_rate_precentage
				&& optimizer_owns_normal_downshift())
			{
				printf("[GEARSHIFT] LADDER DOWN deferred to effective-rate optimizer: "
				       "success=%.0f%% cfg=%d eff=%.0f sack=%.3f partial_loss=%.3f\n",
				       last_transmission_block_stats.success_rate_data, current_configuration,
				       get_current_effective_rate_bps(), get_current_sack_rate(),
				       get_current_partial_frame_loss_rate());
				fflush(stdout);
				this->connection_status=TRANSMITTING_DATA;
			}
			else if(last_transmission_block_stats.success_rate_data<gear_shift_down_success_rate_precentage
				&& !optimizer_owns_normal_downshift())
			{
				// In the calibrated band, selective-repeat losses are priced by the
				// effective-goodput optimizer.  Do not independently downshift just
				// because a block's first-pass success percentage is low: the faster
				// config may still deliver more new bytes/sec after repair. BREAK and
				// below/out-of-calibration fallback remain untouched safety paths.
				// Outside optimizer ownership retain the legacy 3-bad-block descent.
				// Require 3 consecutive bad blocks before downshifting.
				// A single ACK timeout on an otherwise good channel shouldn't
				// trigger a config downshift (the retry will succeed).
				gear_shift_down_consecutive_fails++;
				if(gear_shift_down_consecutive_fails < 3)
				{
					printf("[GEARSHIFT] LADDER: poor block %d/3 (success=%.0f%%), holding config %d\n",
						gear_shift_down_consecutive_fails, last_transmission_block_stats.success_rate_data,
						current_configuration);
					fflush(stdout);
					this->connection_status=TRANSMITTING_DATA;
				}
				else if(!config_is_at_bottom(current_configuration, robust_enabled))
				{
					negotiated_configuration=config_ladder_down(current_configuration, robust_enabled);
					// Lower ceiling to prevent immediate re-upshift to the failing config.
					// Ceiling recovery (8 good blocks) will raise it if channel improves.
					if(supershift_proven_ceiling < 0 ||
					   config_ladder_index(current_configuration) <= config_ladder_index(supershift_proven_ceiling))
					{
						supershift_proven_ceiling = negotiated_configuration;
						ceiling_success_count = 0;
						printf("[GEARSHIFT] LADDER DOWN: ceiling lowered to %d\n", negotiated_configuration);
						fflush(stdout);
					}
					printf("[GEARSHIFT] LADDER DOWN: success=%.0f%% < %.0f%%, config %d -> %d (batch=1)\n",
						last_transmission_block_stats.success_rate_data, gear_shift_down_success_rate_precentage,
						current_configuration, negotiated_configuration);
					fflush(stdout);
					cleanup();
					add_message_control(SET_CONFIG);
					opt_reset_window();
					rate_opt.force_cooldown(5);
				}
				else
				{
					printf("[GEARSHIFT] LADDER: at bottom (config %d), success=%.0f%%\n",
						current_configuration, last_transmission_block_stats.success_rate_data);
					fflush(stdout);
					this->connection_status=TRANSMITTING_DATA;
				}
				gear_shift_blocked_for_nBlocks=0;
			}
			else
			{
				printf("[GEARSHIFT] LADDER: hold config %d, success=%.0f%%\n",
					current_configuration, last_transmission_block_stats.success_rate_data);
				fflush(stdout);
				this->connection_status=TRANSMITTING_DATA;
			}
			}  // end of v1 inline SUCCESS_BASED_LADDER branch
		}
	}
	else
	{
		this->connection_status=TRANSMITTING_DATA;
	}
}

// SACK Design A Step 9 — Multi-axis policy framework, Axis 1 entry point.
//
// Wraps the v1 SUCCESS_BASED_LADDER block above. Functionally identical
// (same observable, action, hysteresis) — the body is a copy of the v1
// inline path with two additions per §4.3.4:
//
//   - invariant #5: `[POLICY-MOVE] axis=1 from=Y to=Z reason=R` log on
//     every modulation move (up, down). The pre-existing `[GEARSHIFT]`
//     log lines are preserved alongside; external parsers are not broken.
//
//   - invariant #6: `policy_axis1_supremacy_on_move()` fires on every
//     modulation move, naming the contract point that Steps 10/11 fill
//     in (reset Axes 2/3 to safe state). At Step 9 the body is a
//     logging stub — Axes 2/3 controllers do not exist yet.
//
// CMD-side only (§3.8 initiator-controls-flow): there is no RSP-side
// Axis 1 controller; RSP just executes the SET_CONFIG handed to it.
void cl_arq_controller::policy_evaluate_axis1()
{
	gear_shift_blocked_for_nBlocks++;
	// Reset downshift failure counter on any good block.
	// SACK_DESIGN_A_PLAN §7.13.24 reverted — see comment on the parallel
	// site in the legacy ladder above. Strict-reset is the right policy
	// until the interaction with BREAK descent is understood.
	if(last_transmission_block_stats.success_rate_data >= gear_shift_down_success_rate_precentage
	   || optimizer_owns_normal_downshift())
		gear_shift_down_consecutive_fails = 0;

	// Handoff: upward moves retain the monitor branch's calibrated-band
	// optimizer ownership. Ordinary success-rate LADDER DOWN yields only when
	// optimizer_owns_normal_downshift() has enough aligned evidence; emergency
	// BREAK remains independent.
	// CLEAN-BATCH VIABILITY (§9): UP gate uses the CLEAN-only rate (see the legacy
	// ladder twin in finalize_block_commander) so partial-only runs don't promote.
	if(success_rate_data_clean>gear_shift_up_success_rate_precentage
		&& gear_shift_blocked_for_nBlocks>= gear_shift_block_for_nBlocks_total
		&& !optimizer_is_in_control())
	{
		int proposed = config_ladder_up(current_configuration, robust_enabled, narrowband_enabled == YES);
		// Respect proven ceiling — don't re-try configs that already failed during turboshift
		bool ceiling_blocked = (supershift_proven_ceiling >= 0 &&
			config_ladder_index(proposed) > config_ladder_index(supershift_proven_ceiling))
			|| (max_config_override >= 0 && proposed > max_config_override);
		// Option B (data-anchored promotion): block any LADDER UP whose
		// destination is > 1 rung above the highest data-viable rung, unless the
		// Q-table optimizer owns the band. (This wrapper is already gated on
		// !optimizer_is_in_control() at entry; the explicit re-check keeps the
		// predicate self-contained.) See §6/§7.
		if(!optimizer_is_in_control() &&
			config_ladder_index(proposed) > config_ladder_index(last_data_viable_config) + 1)
			ceiling_blocked = true;
		// FIX-B — floor-probe back-off (gearshift-floor-probe-backoff.md §5.2,
		// gate-site #2, v2 policy_evaluate_axis1). AND the per-rung suppression into
		// the EXISTING up gate. INV-1: only turns a PERMITTED probe OFF. INV-2: UP
		// path only; no BREAK/demote/panic (downward) site references the back-off.
		if(probe_rung_suppressed(proposed))
			ceiling_blocked = true;
		// WALL-B FIX-5 hook #1 (LADDER UP, v2 policy twin): block a step above the
		// CFG16 big-block carve cooldown ceiling (CFG15) while the cooldown is armed.
		if(apply_bigblock_cooldown_cap(proposed) != proposed)
			ceiling_blocked = true;
		// CFG16 decode-margin gate (LADDER UP, v2): SNR-blind ladder step -> block the
		// 15->16 promotion until the reverse-path SNR supports CFG16.
		if(apply_cfg16_margin_cap(proposed) != proposed)
			ceiling_blocked = true;
		// Per-rung floor gate (LADDER UP, v2): block a success-based up-step to a rung the
		// reverse-path SNR does not support at its measured floor (audit §1 P9; never-raise).
		if(apply_rung_floor_cap(proposed) != proposed)
			ceiling_blocked = true;
		if(!config_is_at_top(current_configuration, robust_enabled, narrowband_enabled == YES) && !ceiling_blocked)
		{
			negotiated_configuration=proposed;
			printf("[GEARSHIFT] LADDER UP: success=%.0f%% > %.0f%%, config %d -> %d\n",
				last_transmission_block_stats.success_rate_data, gear_shift_up_success_rate_precentage,
				current_configuration, negotiated_configuration);
			// §4.3.4 invariant #5: every move emits a [POLICY-MOVE] line.
			printf("[POLICY-MOVE] axis=1 from=%d to=%d reason=ladder_up "
				"success=%.0f%% threshold=%.0f%% blocks_held=%d\n",
				current_configuration, negotiated_configuration,
				last_transmission_block_stats.success_rate_data,
				gear_shift_up_success_rate_precentage,
				gear_shift_blocked_for_nBlocks);
			fflush(stdout);
			// §4.3.4 invariant #6: supremacy hook — Axes 2/3 must yield.
			policy_axis1_supremacy_on_move(current_configuration, negotiated_configuration, "ladder_up");
			cleanup();
			add_message_control(SET_CONFIG);
		}
		else
		{
			// Ceiling recovery: 20 → 5 blocks (see comment at first ceiling-recovery
			// site, ~line 4346). Band-aid until 2D channel measurement lands.
			// WALL-B FIX-5 hook #1b (CEILING RECOVERY, v2): never raise the proven ceiling
			// above the cooldown ceiling — else it would walk the ceiling back to CFG16.
			// AND never raise it while the CFG16 decode-margin gate holds CFG15 (same reason).
			// AND never raise it above a rung whose per-rung measured floor is not cleared
			// (audit §1 P10: the proven ceiling can never walk back above a closed floor).
			if(ceiling_blocked && apply_bigblock_cooldown_cap(proposed) == proposed
				&& apply_cfg16_margin_cap(proposed) == proposed
				&& apply_rung_floor_cap(proposed) == proposed)
			{
				ceiling_success_count++;
				if(ceiling_success_count >= 5)
				{
					int old_ceiling = supershift_proven_ceiling;
					supershift_proven_ceiling = proposed;  // raise ceiling to what we wanted to try
					ceiling_success_count = 0;
					printf("[GEARSHIFT] CEILING RECOVERY: %d -> %d after %d good blocks\n",
						old_ceiling, supershift_proven_ceiling, 5);
					fflush(stdout);
					// Ceiling recovery is a STATE change (cap raised) but NOT a
					// modulation move — current_configuration is unchanged. No
					// [POLICY-MOVE] line here; the next block evaluation will
					// emit [POLICY-MOVE] axis=1 reason=ladder_up if it then
					// climbs.
				}
			}
			else
			{
				ceiling_success_count = 0;
			}
			printf("[GEARSHIFT] LADDER: at top (config %d), success=%.0f%%%s\n",
				current_configuration, last_transmission_block_stats.success_rate_data,
				ceiling_blocked ? " [ceiling-limited]" : "");
			fflush(stdout);
			this->connection_status=TRANSMITTING_DATA;
		}
	}
	else if(last_transmission_block_stats.success_rate_data<gear_shift_down_success_rate_precentage
		&& optimizer_owns_normal_downshift())
	{
		printf("[GEARSHIFT] LADDER DOWN deferred to effective-rate optimizer: "
		       "success=%.0f%% cfg=%d eff=%.0f sack=%.3f partial_loss=%.3f\n",
		       last_transmission_block_stats.success_rate_data, current_configuration,
		       get_current_effective_rate_bps(), get_current_sack_rate(),
		       get_current_partial_frame_loss_rate());
		fflush(stdout);
		this->connection_status=TRANSMITTING_DATA;
	}
	else if(last_transmission_block_stats.success_rate_data<gear_shift_down_success_rate_precentage
		&& !optimizer_owns_normal_downshift())
	{
		// In the calibrated band the effective-goodput optimizer owns normal
		// performance downshifts as well as climbs. A low first-pass success
		// percentage can still be optimal when SACK repair is cheap. BREAK is
		// still independent, and this legacy descent remains the fallback while
		// the optimizer is unready or outside its calibrated loss envelope.
		// Require 3 consecutive bad blocks before downshifting.
		// A single ACK timeout on an otherwise good channel shouldn't
		// trigger a config downshift (the retry will succeed).
		gear_shift_down_consecutive_fails++;
		if(gear_shift_down_consecutive_fails < 3)
		{
			printf("[GEARSHIFT] LADDER: poor block %d/3 (success=%.0f%%), holding config %d\n",
				gear_shift_down_consecutive_fails, last_transmission_block_stats.success_rate_data,
				current_configuration);
			fflush(stdout);
			this->connection_status=TRANSMITTING_DATA;
		}
		else if(!config_is_at_bottom(current_configuration, robust_enabled))
		{
			negotiated_configuration=config_ladder_down(current_configuration, robust_enabled);
			// Lower ceiling to prevent immediate re-upshift to the failing config.
			// Ceiling recovery (8 good blocks) will raise it if channel improves.
			if(supershift_proven_ceiling < 0 ||
			   config_ladder_index(current_configuration) <= config_ladder_index(supershift_proven_ceiling))
			{
				supershift_proven_ceiling = negotiated_configuration;
				ceiling_success_count = 0;
				printf("[GEARSHIFT] LADDER DOWN: ceiling lowered to %d\n", negotiated_configuration);
				fflush(stdout);
			}
			printf("[GEARSHIFT] LADDER DOWN: success=%.0f%% < %.0f%%, config %d -> %d (batch=1)\n",
				last_transmission_block_stats.success_rate_data, gear_shift_down_success_rate_precentage,
				current_configuration, negotiated_configuration);
			// §4.3.4 invariant #5: every move emits a [POLICY-MOVE] line.
			printf("[POLICY-MOVE] axis=1 from=%d to=%d reason=ladder_down "
				"success=%.0f%% threshold=%.0f%% consecutive_fails=%d\n",
				current_configuration, negotiated_configuration,
				last_transmission_block_stats.success_rate_data,
				gear_shift_down_success_rate_precentage,
				gear_shift_down_consecutive_fails);
			fflush(stdout);
			// §4.3.4 invariant #6: supremacy hook — Axes 2/3 must yield.
			policy_axis1_supremacy_on_move(current_configuration, negotiated_configuration, "ladder_down");
			cleanup();
			add_message_control(SET_CONFIG);
		}
		else
		{
			printf("[GEARSHIFT] LADDER: at bottom (config %d), success=%.0f%%\n",
				current_configuration, last_transmission_block_stats.success_rate_data);
			fflush(stdout);
			this->connection_status=TRANSMITTING_DATA;
		}
		gear_shift_blocked_for_nBlocks=0;
	}
	else
	{
		printf("[GEARSHIFT] LADDER: hold config %d, success=%.0f%%\n",
			current_configuration, last_transmission_block_stats.success_rate_data);
		fflush(stdout);
		this->connection_status=TRANSMITTING_DATA;
	}
}

// SACK Design A Step 9 — Axis 1 supremacy hook (§4.3.4 invariant #6).
//
// "If Axis 1 decides to move config (or BREAK to ROBUST_0), that decision
//  overrides any in-flight Axis 2 / Axis 3 move."
//
// At Step 9 the body is a logging stub — Axes 2 (batch size) and 3 (SACK
// mode) controllers do not yet exist. Steps 10/11 will fill in:
//
//   - reset `data_batch_size` to `radio_batch_size_floor = 10`
//   - set Axis 3 sack_mode to PROBE
//   - cancel in-flight Axes 2/3 timers / cooldowns
//
// The hook is named at Step 9 so the contract point is explicit and
// Steps 10/11 have a stable insertion site. Naming it now also means
// `[POLICY-SUPREMACY]` log events appear in v2 sessions from Step 9
// forward — external policy-trace tooling can be wired up before the
// downstream axes land.
//
// Called from policy_evaluate_axis1() on every ladder up/down move.
// Steps 12+ will also call this from the emergency-BREAK code path.
// Forward declaration — defined further down in this TU. Used by both
// policy_axis1_supremacy_on_move() (for the Step-11 Axis-3 reset log) and
// the Step-11 controllers themselves.
static const char* axis3_mode_str(int m);

void cl_arq_controller::policy_axis1_supremacy_on_move(int from_cfg, int to_cfg, const char* reason)
{
	// A real Axis-1 move is prepared exactly once here and consumed by the
	// universal SET_CONFIG/in-band executor.  Producers that already call this
	// hook therefore do not get a second lower-axis reset at the chokepoint;
	// producers that forgot it are repaired centrally there.
	if(from_cfg != to_cfg)
	{
		axis1_supremacy_prepared = true;
		axis1_supremacy_prepared_from = from_cfg;
		axis1_supremacy_prepared_to = to_cfg;
	}
	// SACK Design A Step 10 — Axis 2 reset (§4.3.3 cross-axis cooldown).
	//
	// Axis 1's move changed the modulation, which invalidates the partial-rate
	// history we accumulated on the old config. Wipe Axis 2's ring + counters
	// and engage a 3-batch cooldown so Axis 2 cannot fire before re-observing
	// loss on the new config. Per §4.3.3:
	//   "If Axis 1 moves config down (channel got worse), Axis 2's partial-rate
	//    window is invalidated (the loss pattern changes when config changes),
	//    so Axis 2 enters a 3-batch cooldown before its next decision.
	//    Implementation: axis2_cooldown_batches = 3 after any Axis 1 move;
	//    ignore observations inside the cooldown."
	for(int i=0;i<AXIS2_RING_DEPTH;i++) axis2_partial_rate_ring[i]=0.0f;
	axis2_partial_rate_count=0;
	axis2_partial_rate_pos=0;
	axis2_consecutive_good_batches=0;
	axis2_consecutive_bad_batches=0;
	axis2_cooldown_batches = AXIS2_CROSS_AXIS_COOLDOWN_BATCHES;

	// SACK Design A Step 12 — clear Axis-2 proven-ceiling on any Axis-1 move.
	// Per §4.3.4 invariant #7 + prompt §2: "On a config (Axis 1) move, the
	// ceiling RESETS (channel changed, prior ceiling stale)." A new modulation
	// has a different SNR / fade profile; the batch-size that failed on the
	// prior config may be perfectly fine now (and vice versa). Forget what
	// we proved at the old config.
	int prev_axis2_ceiling = batch_size_proven_ceiling;
	int prev_axis2_recovery = batch_size_ceiling_recovery_batches;
	batch_size_proven_ceiling = -1;
	batch_size_ceiling_recovery_batches = 0;

	// SACK Design A Step 11 — Axis 3 reset (§4.3.3 cross-axis cooldown).
	//
	// Axis 1's move invalidates the SACK-decode history on the old config.
	// Clear the ring + consecutive_sack_misses counter, set a 3-batch
	// cooldown that suppresses Axis-3 MOVES (observations still record).
	// If we were in ON or OFF, transition to PROBE so the very next decoded
	// SACK on the new config drives a meaningful re-evaluation. PROBE stays
	// PROBE (already the "test" state).
	for(int i=0;i<AXIS3_RING_DEPTH;i++) axis3_recent_sack_ok[i]=false;
	axis3_recent_sack_ok_count=0;
	axis3_recent_sack_ok_pos=0;
	axis3_consecutive_sack_misses=0;
	axis3_batches_since_off=0;
	axis3_cooldown_batches = AXIS3_CROSS_AXIS_COOLDOWN_BATCHES;
	int axis3_prev_mode = axis3_sack_mode;
	if(axis3_sack_mode == SACK_MODE_ON || axis3_sack_mode == SACK_MODE_OFF)
	{
		axis3_sack_mode = SACK_MODE_PROBE;
	}

	printf("[POLICY-SUPREMACY] axis=1 move reason=%s — "
		"Axis 2 reset (ring+counters cleared, cooldown=%d batches, "
		"proven_ceiling %d->-1 recovery %d->0); "
		"Axis 3 reset (ring+misses cleared, cooldown=%d batches, "
		"mode %s -> %s)\n",
		reason, axis2_cooldown_batches,
		prev_axis2_ceiling, prev_axis2_recovery,
		axis3_cooldown_batches,
		axis3_mode_str(axis3_prev_mode), axis3_mode_str(axis3_sack_mode));
	fflush(stdout);

	// If Axis-1 moved us into PROBE (from ON or OFF), inform RSP via
	// SET_LINK_PARAMS so RSP's SACK_RSP-TX behavior aligns. Guarded by
	// the busy-check inside the helper — if the control channel is in use
	// the EOB-self-correct safety net (§4.3.4 invariant 4) covers it.
	if(axis3_prev_mode != axis3_sack_mode)
	{
		axis3_send_set_link_params(axis3_sack_mode, "axis1_supremacy_reset");
	}

	opt_reset_window();
	rate_opt.force_cooldown(5);
}

// SACK Design A Step 10 — Axis 2 controller (adaptive batch size).
//
// Per-batch §4.3.2 controller. Fires from process_messages_rx_acks_data()
// after every SACK_RSP receipt (= partial batch) and after every clean
// full-batch ACK. Both call sites are gated on sack_v2_enabled; v1 sessions
// never enter this code.
//
// rx_count = frames the receiver successfully delivered for the just-completed
// batch (= batch_size_observed for a clean ACK; < batch_size_observed when
// the SACK_RSP bitmap reported missing slots). batch_size_observed is the
// data_batch_size value at the time the batch was sent.
//
// §4.3.2: partial_rate = frames_lost / batch_size. Ring of 5. Hysteresis:
//   - up: mean<0.05 AND >=8 consecutive good batches (partial_rate < 0.05).
//   - down: mean>0.20 AND >=3 consecutive bad batches (partial_rate > 0.20).
//   - reset counters on any move.
//   - step ±5; clamp [10, AXIS2_BATCH_CEIL].
//
// §4.3.3 / §4.3.4 invariant #6: skip when axis2_cooldown_batches > 0
// (set to 3 by policy_axis1_supremacy_on_move() on every Axis-1 move).
// Cooldown decremented per evaluation call.
//
// On move: log [POLICY-MOVE] axis=2 ... per §4.3.4 invariant #5, send
// SET_LINK_PARAMS to the peer, AND apply locally. The handshake gates
// the next data TX in RECEIVING_ACKS_CONTROL — both sides update before
// the next batch.
void cl_arq_controller::policy_evaluate_axis2(int rx_count, int batch_size_observed)
{
	// Held-rung probation counter (burst coalescing): one increment per controller
	// call — both production call sites fire once per completed batch. MUST precede
	// every early-return guard below: an increment placed after the climb-confirm
	// skip would starve the counter forever and the held-rung release could never
	// arm. Reset in load_configuration() / reset_session_state() via
	// burst_coalesce_reset_probation(). If the call cadence ever changes, keep this
	// per-BATCH, not per-call.
	batches_at_current_config++;
	// climb-engine Bug 3 root cause (gearshift-climb-engine.md §2): ROBUST/MFSK
	// configs pin data_batch_size=1 by design (arq_common.cc:1229-1234, "MFSK
	// modes keep batch_size=1 for pattern-ACK optimization"; the OFDM batch-
	// scaling at arq_common.cc:1269 is already !is_robust_config-gated). Axis-2
	// grows the batch after AXIS2_UP_GOOD_RUN clean batches (floor 10, step 5).
	// At batch=1 a clean MFSK delivery is partial_rate=0 ("good"), so without
	// this guard, after the climb promotes ROBUST_0->ROBUST_1 (FRAME-UP, which
	// does NOT call the Axis-1 supremacy hook that would reset the good-run),
	// the carried-over good-run trips Axis-2 at the new rung and steps batch
	// 1->6. The RSP clamps SET_LINK_PARAMS to [AXIS2_BATCH_FLOOR=10, 32] -> CMD=6
	// / RSP=10 MISMATCH -> p^N clean-batch collapse -> no clean credit at
	// ROBUST_1 -> last_data_viable_config frozen at ROBUST_0 -> the +1 anchor
	// clamp blocks ROBUST_2 -> the promotion engine goes DORMANT (climbs exactly
	// one rung). Skip the whole controller on robust links so batch stays 1 at
	// EVERY robust rung and clean MFSK ACKs keep accumulating to drive the climb.
	// (This is the standalone-C2 sibling fix that the combined C3 dropped — that
	// omission IS the dormancy. OFDM CONFIG_0..16 is unchanged: Axis-2 runs.)
	if(is_robust_config(current_configuration)) return;
	// BIG-BLOCK RUNG: data_batch_size is GEOMETRY-LOCKED to K (= one acquisition's
	// codeword count), NOT link-adaptive (fact-doc data-flow-bigblock-arq-unit.md
	// §16.5, INV-5b). The gearshift CFG16 transition elects data_batch_size=K via the
	// shared election body (arq_common.cc load_configuration tail). If Axis-2 ran here
	// it would step K=8 -> 13 after AXIS2_UP_GOOD_RUN clean batches and push the RSP to
	// match via SET_LINK_PARAMS, moving BOTH peers off K while the RX carve still emits
	// a K=8 cw_ok bitmap -> all_ones (1<<13)-1 != 0xFF -> bug #9 re-diverges MID-SESSION.
	// Suppress the whole controller at the bigblock rung (same shape as the robust guard
	// above) so K stays geometry-locked. This is NOT a threshold tune — the rung's batch
	// is a PHY fact, so the adaptive controller has no valid axis to act on there. Every
	// other OFDM config (CONFIG_0..15, framing off) is UNCHANGED: Axis-2 runs.
	if(telecom_system != NULL
		&& telecom_system->bigblock_framing_enabled
		&& current_configuration == CONFIG_16) return;
	// P-alt (DUTY fast-start): while climbing an OFDM rung below the ceiling the batch is
	// PINNED small (the shared election cap). Skip the adaptive controller so it does NOT
	// SET_LINK_PARAMS-grow the confirm batch back up mid-climb (which would re-inflate the
	// per-rung dwell AND pay a ~2 s round-trip pause). Same shape as the robust / bigblock
	// guards above. At the ladder TOP (steady-state) Axis-2 runs -> full batch for throughput.
	// Held-rung burst coalescing release: a rung that has been HELD past probation
	// (burst_coalesce_eligible() — the margin-gated election parking a session below
	// the top with gearshift ON) is steady state, not a climb, so the controller
	// runs there: stock Axis-2 growth (+5 per 4-clean run, quiesce gate, proven
	// ceiling, floor 10 = today's pinned size) grows the batch toward the nominal
	// election, and down-moves become live at held rungs too. During a real climb
	// the counter resets on every FRAME-UP load, so the skip still holds mid-climb.
	if(climb_confirm_batch_active() && !burst_coalesce_eligible()) return;
	axis2_evaluations++;
	if(batch_size_observed <= 0) return;  // defensive — no observation
	if(rx_count < 0) rx_count = 0;
	if(rx_count > batch_size_observed) rx_count = batch_size_observed;

	int frames_lost = batch_size_observed - rx_count;
	float partial_rate = (float)frames_lost / (float)batch_size_observed;

	// Update ring + counters BEFORE the cooldown gate so observations are
	// recorded; the gate only suppresses MOVES. (Per §4.3.3 "ignore
	// observations inside the cooldown" — but resetting the ring at the
	// supremacy hook already clears history. Subsequent observations inside
	// the cooldown DO go into the ring; this is a finer-grained read of the
	// spec — the relevant prohibition is "no Axis 2 move inside the
	// cooldown", which we enforce strictly below.)
	axis2_partial_rate_ring[axis2_partial_rate_pos] = partial_rate;
	axis2_partial_rate_pos = (axis2_partial_rate_pos + 1) % AXIS2_RING_DEPTH;
	if(axis2_partial_rate_count < AXIS2_RING_DEPTH) axis2_partial_rate_count++;

	// Hysteresis counters: "good" = partial<0.05, "bad" = partial>0.20.
	// Any single bad observation resets the good-run counter, and vice versa.
	bool is_good = (partial_rate < 0.05f);
	bool is_bad  = (partial_rate > 0.20f);
	if(is_good)
	{
		axis2_consecutive_good_batches++;
		axis2_consecutive_bad_batches = 0;
	}
	else if(is_bad)
	{
		axis2_consecutive_bad_batches++;
		axis2_consecutive_good_batches = 0;
	}
	else
	{
		// In the [0.05, 0.20] middle band: neither good nor bad. Reset BOTH
		// counters — neither run is still "consecutive."
		axis2_consecutive_good_batches = 0;
		axis2_consecutive_bad_batches = 0;
	}

	// Cooldown: skip MOVE decision while the Axis-1 supremacy timer is active.
	if(axis2_cooldown_batches > 0)
	{
		axis2_cooldown_batches--;
		axis2_skipped_in_cooldown++;
		printf("[POLICY-AXIS2] eval rx=%d/%d partial=%.3f good=%d bad=%d "
			"COOLDOWN_REMAINING=%d (no move)\n",
			rx_count, batch_size_observed, partial_rate,
			axis2_consecutive_good_batches, axis2_consecutive_bad_batches,
			axis2_cooldown_batches);
		fflush(stdout);
		return;
	}

	// SACK Design A Step 12 — §4.3.4 invariant #7: drain proven-ceiling recovery.
	// The ceiling is set on down-moves (the batch_size that just failed must
	// not be re-proposed immediately). The recovery counter decrements on
	// every evaluation; on reaching zero the ceiling clears. Axis-1 supremacy
	// resets the ceiling unconditionally (see policy_axis1_supremacy_on_move).
	if(batch_size_proven_ceiling >= 0 && batch_size_ceiling_recovery_batches > 0)
	{
		batch_size_ceiling_recovery_batches--;
		if(batch_size_ceiling_recovery_batches == 0)
		{
			printf("[POLICY-AXIS2-CEILING] recovery period elapsed — clearing "
				"proven_ceiling=%d (was set after prior down-move)\n",
				batch_size_proven_ceiling);
			fflush(stdout);
			batch_size_proven_ceiling = -1;
		}
	}

	// Compute mean over the ring (only over filled slots — avoid skewing
	// early-session decisions with zero-padded entries).
	float sum = 0.0f;
	for(int i=0;i<axis2_partial_rate_count;i++) sum += axis2_partial_rate_ring[i];
	float mean_partial = (axis2_partial_rate_count > 0)
		? sum / (float)axis2_partial_rate_count : 0.0f;

	// Up-move test: requires both the ring mean AND the consecutive-good
	// counter at threshold. Both thresholds are non-overlapping with the
	// down-move thresholds (§4.3.3 "different thresholds for up-moves vs
	// down-moves").
	bool want_up   = (mean_partial < 0.05f)
	                 && (axis2_consecutive_good_batches >= AXIS2_UP_GOOD_RUN)
	                 && (data_batch_size + AXIS2_STEP <= axis2_batch_ceiling());
	bool want_down = (mean_partial > 0.20f)
	                 && (axis2_consecutive_bad_batches >= AXIS2_DOWN_BAD_RUN)
	                 && (data_batch_size - AXIS2_STEP >= AXIS2_BATCH_FLOOR);

	// FIX 2 — CATASTROPHIC-PARTIAL FAST-DOWN (DATAFLOW_AUDIT_batch_coast.md §6).
	// A SINGLE batch whose SACK-bitmap partial_rate exceeds the composition of the
	// two existing hysteresis constants (AXIS2_FASTDOWN_PARTIAL = 0.20 bad-thresh ×
	// AXIS2_DOWN_BAD_RUN 3 = 0.60) already carries the full "3 consecutive bad"
	// worth of loss evidence in one observation, so an AIMD multiplicative decrease
	// (target = B/2 snapped to the AXIS2_STEP grid, floored at AXIS2_BATCH_FLOOR)
	// fires immediately instead of waiting three catastrophic keydowns. This breaks
	// the coalescing relapse loop (probation re-climb +5/4-clean back to the same
	// fatal width) that the −5 step + ring-reset could not. Signal class:
	// partial_rate is measured channel loss from the SACK bitmap, not ACK-absence.
	// Ranked BELOW prevention (batch-coast); judged only on the attribution
	// sub-cohort. Own defeat knob MERCURY_AXIS2_FASTDOWN_DEFEAT restores stock.
	bool want_fastdown = false;
	int  fastdown_target = data_batch_size;
	if(!axis2_fastdown_defeat
	   && partial_rate > AXIS2_FASTDOWN_PARTIAL
	   && data_batch_size > AXIS2_BATCH_FLOOR)
	{
		int halved  = data_batch_size / 2;
		int snapped = ((halved + AXIS2_STEP / 2) / AXIS2_STEP) * AXIS2_STEP;  // nearest step-grid multiple
		if(snapped < AXIS2_BATCH_FLOOR) snapped = AXIS2_BATCH_FLOOR;
		if(snapped < data_batch_size)     // must actually decrease
		{
			fastdown_target = snapped;
			want_fastdown   = true;
			want_up         = false;      // a catastrophic partial is never an up-move
			want_down       = false;      // fast-down supersedes the incremental down path
		}
	}

	// SACK Design A Step 12 — §4.3.4 invariant #7: enforce proven-ceiling.
	// If the up-move would propose a batch_size above the ceiling, VETO the
	// move. The ceiling represents the cap set by a prior down-move; we may
	// not re-climb above it until the recovery period elapses (handled at
	// the top of this function) or an Axis-1 supremacy event resets it.
	if(want_up && batch_size_proven_ceiling >= 0)
	{
		int proposed = data_batch_size + AXIS2_STEP;
		if(proposed > batch_size_proven_ceiling)
		{
			axis2_ceiling_blocks_count++;
			printf("[POLICY-AXIS2-CEILING] up-move VETOED: proposed=%d > "
				"proven_ceiling=%d recovery_remaining=%d (no move; ceiling "
				"will clear on recovery expiry or Axis-1 move)\n",
				proposed, batch_size_proven_ceiling,
				batch_size_ceiling_recovery_batches);
			fflush(stdout);
			want_up = false;
			// Reset the good-run counter so the same ring of clean batches
			// does NOT immediately re-trigger the same vetoed up-move on the
			// next evaluation (preventing a no-op log spam). Per §4.3.3 the
			// counter is meant to gate counted-event moves; we treat the veto
			// as having "consumed" the consecutive-good run.
			axis2_consecutive_good_batches = 0;
		}
	}

	if(want_up || want_down || want_fastdown)
	{
		// R4 — LINK-PARAMS quiesce gate (cross-layer data-flow audit: Axis-2 vs the retx
		// queue / partial last batch). Renegotiating batch size mid-transfer applies the new
		// geometry to the FIRST batch of the new size while old-bsi retransmits are still in
		// flight, mixing them into the riskiest (post-turnaround) batch of the new geometry.
		// Standard contract: parameter changes apply to FRESH data only (RFC 1191/8201 PMTUD;
		// AX.25 v2.2 XID renegotiates only outside active recovery; TCP never re-frames
		// in-flight segments after an MSS change). So defer the move until the boundary is
		// CLEAN — no retransmits pending AND the last batch fully ACKed — re-evaluating on the
		// next batch (the ring/counters are retained by the early return, so a still-valid
		// move re-triggers). BOUNDED: a sustained-loss link that never reaches a clean
		// boundary forces the move after AXIS2_MAX_MOVE_DEFER deferrals so it cannot stall.
		{
			// MERCURY_AXIS2_QUIESCE_DEFEAT=1 bypasses the gate (the A/B / fail-before arm;
			// same house pattern as MERCURY_GAP_ABORT_DEFEAT — read on this production path
			// => LIVE). Ships default-ON (gate active). With the gate defeated a move fires
			// even with retx pending, reproducing the pre-R4 mixed old-bsi / new-size batch.
			bool quiesce_defeat = false;
			{ const char* e = std::getenv("MERCURY_AXIS2_QUIESCE_DEFEAT"); if(e && e[0]=='1') quiesce_defeat = true; }
			bool clean_boundary = (retransmit_count == 0) && last_batch_fully_acked;
			if(!quiesce_defeat && !clean_boundary)
			{
				if(axis2_deferred_moves < AXIS2_MAX_MOVE_DEFER)
				{
					axis2_deferred_moves++;
					printf("[POLICY-AXIS2-DEFER] want_%s move but boundary UNCLEAN "
						"(retransmit_count=%d last_batch_fully_acked=%d) — deferring %d/%d, "
						"re-evaluate next batch (ring retained)\n",
						want_up ? "up" : "down", retransmit_count,
						(int)last_batch_fully_acked, axis2_deferred_moves,
						AXIS2_MAX_MOVE_DEFER);
					fflush(stdout);
					return;   // ring + hysteresis counters retained; move re-triggers
				}
				printf("[POLICY-AXIS2-DEFER] defer budget exhausted (%d/%d) — FORCING want_%s "
					"move despite unclean boundary (retransmit_count=%d "
					"last_batch_fully_acked=%d)\n",
					axis2_deferred_moves, AXIS2_MAX_MOVE_DEFER, want_up ? "up" : "down",
					retransmit_count, (int)last_batch_fully_acked);
				fflush(stdout);
			}
			axis2_deferred_moves = 0;   // move proceeds (clean boundary, forced, or defeated)
		}

		int from = data_batch_size;
		int to = want_up ? (from + AXIS2_STEP)
		       : want_fastdown ? fastdown_target
		       : (from - AXIS2_STEP);
		const char* reason = want_up ? "ring_clean"
		                   : want_fastdown ? "catastrophic_partial"
		                   : "ring_lossy";
		const char* dir    = want_up ? "up" : "down";

		printf("[POLICY-MOVE] axis=2 from=%d to=%d direction=%s reason=%s "
			"mean_partial=%.3f good=%d bad=%d cooldown=%d\n",
			from, to, dir, reason, mean_partial,
			axis2_consecutive_good_batches, axis2_consecutive_bad_batches,
			axis2_cooldown_batches);
		fflush(stdout);

		// Reset ring + counters on any move (§4.3.2 hysteresis rule).
		// We keep the ring sized at AXIS2_RING_DEPTH; just clear contents +
		// counts so the next 5 batches re-populate it under the new
		// data_batch_size.
		for(int i=0;i<AXIS2_RING_DEPTH;i++) axis2_partial_rate_ring[i]=0.0f;
		axis2_partial_rate_count=0;
		axis2_partial_rate_pos=0;
		axis2_consecutive_good_batches=0;
		axis2_consecutive_bad_batches=0;

		// Stash the target so the SET_LINK_PARAMS encoder picks it up.
		// add_message_control(SET_LINK_PARAMS) reads pending_link_params_*
		// fields (see arq.h Step 10 state). SACK Design A Step 11: carry the
		// CURRENT Axis-3 sack_mode through so an unrelated Axis-2 move does
		// not reset RSP's sack_mode to a stale value.
		pending_link_params_batch_size = to;
		pending_link_params_sack_mode  = axis3_sack_mode;

		// Apply locally NOW so the next batch builds with the new
		// data_batch_size. RSP receives SET_LINK_PARAMS and applies in its
		// own handler (arq_responder.cc). The control-frame ACK handshake
		// gates the next DATA TX in RECEIVING_ACKS_CONTROL, so the two
		// sides converge before the next batch.
		//
		// set_data_batch_size() clamps to a max derived from
		// (max_data_length + max_header_length - ACK_MULTI_ACK_RANGE_HEADER_LENGTH - 1)
		// — that ceiling is in the hundreds, well above 32, so no
		// additional clamping is needed for our [10, 32] range.
		set_data_batch_size(to);
		recalculate_ack_timeout_for_batch();

		if(want_up)            axis2_move_up_count++;
		else if(want_fastdown) axis2_move_fastdown_count++;
		else                   axis2_move_down_count++;

		// SACK Design A Step 12 — §4.3.4 invariant #7: on a down-move,
		// remember the batch_size that just failed (the value we are MOVING
		// AWAY FROM) as the proven-ceiling for the next 20 evaluations.
		// "Failed" = ring-mean > 0.20 with >=3 consecutive bad batches.
		// Setting the cap to (from - 1) means Axis-2 may not re-propose
		// `from` (or any larger value) within the recovery window. Per the
		// plan: "preventing the batch_size from immediately re-climbing into
		// the same failure regime." On an up-move we do NOT set a ceiling
		// (an up-move proves the new K is at least as good as from; no
		// failure observed). On Axis-1 supremacy the ceiling resets to -1.
		if(want_down || want_fastdown)
		{
			int new_ceiling = from - 1;
			// Adopt the more restrictive of the existing ceiling and the new
			// failure point. -1 ⇒ no prior cap; take the new one.
			if(batch_size_proven_ceiling < 0 || new_ceiling < batch_size_proven_ceiling)
			{
				batch_size_proven_ceiling = new_ceiling;
			}
			batch_size_ceiling_recovery_batches = AXIS2_CEILING_RECOVERY_BATCHES;
			printf("[POLICY-AXIS2-CEILING] down-move at batch=%d set "
				"proven_ceiling=%d recovery=%d batches "
				"(no re-climb to >%d until recovery expires or Axis-1 supremacy)\n",
				from, batch_size_proven_ceiling,
				batch_size_ceiling_recovery_batches, batch_size_proven_ceiling);
			fflush(stdout);
		}

		// Send SET_LINK_PARAMS to the peer. add_message_control() bails out
		// when messages_control.status != FREE (a control frame is in flight);
		// if that happens we miss this round's TX but the local batch size
		// already changed. The Axis-1 EOB-derived recovery (§4.3.4 invariant 4)
		// is the safety net: even if the peer's data_batch_size lags ours by
		// one batch, EOB on the last DATA frame self-corrects the receiver.
		if(messages_control.status == FREE)
		{
			add_message_control(SET_LINK_PARAMS);
		}
		else
		{
			printf("[POLICY-AXIS2] WARNING: messages_control busy (status=%d) — "
				"SET_LINK_PARAMS NOT sent this cycle; relying on EOB self-correct "
				"(§4.3.4 invariant 4).\n", messages_control.status);
			fflush(stdout);
		}
	}
	else
	{
		// R4: no move is pending this evaluation — clear any stale deferral count so a
		// LATER move starts its defer budget fresh.
		axis2_deferred_moves = 0;
		printf("[POLICY-AXIS2] eval rx=%d/%d partial=%.3f mean=%.3f "
			"good=%d/%d bad=%d/%d batch=%d (no move)\n",
			rx_count, batch_size_observed, partial_rate, mean_partial,
			axis2_consecutive_good_batches, AXIS2_UP_GOOD_RUN,
			axis2_consecutive_bad_batches, AXIS2_DOWN_BAD_RUN,
			data_batch_size);
		fflush(stdout);
	}
}

// FIX-A — ROBUST-tier dwell-batch decision (data-flow-robust-tier-arq-batch.md
// §5.2/§5.3). CMD-side. Called from the clean-data-ACK PARKED path (after FRAME-UP
// has DECLINED to promote this poll — see arq_commander.cc, just before
// connection_status=TRANSMITTING_DATA). Decides whether the robust dwell batch
// should be RAISED (proven+parked rung) or REVERTED (eligibility lost), and on a
// change applies it locally + stages the symmetric ROBUST_DWELL_BATCH_OP frame.
//
// Transport model = the proven Axis-2 pattern (policy_evaluate_axis2 / SET_LINK_PARAMS):
// CMD applies locally NOW and add_message_control() sets connection_status=
// TRANSMITTING_CONTROL, so the control-frame ACK handshake (RECEIVING_ACKS_CONTROL)
// gates the next DATA TX — the RSP adopts the new batch (via the op handler) and
// ACKs BEFORE the next DATA batch is built. No DATA batch is ever built with
// CMD≠RSP (L6 atomicity w.r.t. DATA TX). The EOB bit-7 self-correct is the same
// safety net Axis-2 relies on if a control frame is lost.
//
// The function is a NO-OP unless the target batch actually differs from the live
// data_batch_size, so it is cheap to call on every clean robust batch. Returns TRUE
// iff it queued a ROBUST_DWELL_BATCH_OP control frame (the caller must then leave
// connection_status at TRANSMITTING_CONTROL, NOT overwrite it with TRANSMITTING_DATA).
bool cl_arq_controller::evaluate_robust_dwell_batch()
{
	// Gated to the robust tier + WB SACK suffix + sack_v2 selective-retransmit
	// (the partial path that makes batch>1 recoverable). On a non-sack_v2 session
	// a robust partial cannot be SACK-patched, so keep the pin at 1.
	if(!is_robust_config(current_configuration)) return false;
	if(!sack_v2_enabled) return false;

	bool eligible = robust_dwell_batch_eligible();
	int target = eligible ? ROBUST_DWELL_BATCH : 1;
	if(target < 1) target = 1;
	if(target > ROBUST_DWELL_BATCH_MAX) target = ROBUST_DWELL_BATCH_MAX;

	// Only act on a genuine change. (data_batch_size is the authoritative live
	// value; robust_dwell_batch_active tracks whether we have RAISED so a revert
	// fires exactly once when eligibility is lost.)
	if(target == data_batch_size && (eligible == robust_dwell_batch_active))
		return false;
	if(target == data_batch_size)
	{
		// Value already matches but the flag is stale — sync the flag, no wire frame.
		robust_dwell_batch_active = eligible;
		return false;
	}

	// L7 (revert symmetry) + L6: do NOT credit clean promotion on the transition
	// batch — the all-ones target is mid-change. Mark this batch non-promoting so a
	// stray clean-credit cannot fire while the raise/revert is in flight.
	last_batch_fully_acked = false;

	printf("[ROBUST-DWELL] %s robust dwell batch %d -> %d at config %d "
		"(eligible=%d anchor=%d streak_cfg=%d streak=%d ceiling=%d)\n",
		eligible ? "RAISE" : "REVERT", data_batch_size, target,
		current_configuration, (int)eligible, last_data_viable_config,
		clean_batches_config, clean_batches_at_current_config,
		supershift_proven_ceiling);
	fflush(stdout);

	// Apply locally NOW (the relaxed chokepoint re-validates the [1..MAX] range and
	// recomputes the ACK timeout — L3). The next DATA batch then builds at the new
	// size; the RSP converges via the op + control-ACK handshake before that TX.
	set_data_batch_size(target);
	nominal_batch_size = data_batch_size;
	robust_dwell_batch_active = eligible;

	// Stage + send the symmetric op. add_message_control() bails if a control frame
	// is already in flight (status != FREE); if so we miss this round's TX but the
	// local size already changed — the EOB self-correct (§3.6) keeps the RSP within
	// one batch, and the next clean poll re-attempts the op (target still differs).
	pending_robust_dwell_batch = target;
	if(messages_control.status == FREE)
	{
		add_message_control(ROBUST_DWELL_BATCH_OP);
		// add_message_control set connection_status=TRANSMITTING_CONTROL; tell the
		// caller to leave it there (the op TX + ACK handshake gates the next DATA TX).
		return true;
	}
	else
	{
		printf("[ROBUST-DWELL] WARNING: messages_control busy (status=%d) — "
			"ROBUST_DWELL_BATCH_OP NOT sent this cycle; relying on EOB self-correct "
			"+ retry next clean poll.\n", messages_control.status);
		fflush(stdout);
		pending_robust_dwell_batch = -1;
		return false;
	}
}

// SACK Design A Step 10 — Axis-2 cooldown helper.
// Decrement cooldown counter by one (clamped at zero) and return the
// post-decrement value. Currently unused by the controller body (which
// inlines its own decrement); kept as part of the public API for Step 11+
// and for unit-test access.
int cl_arq_controller::axis2_cooldown_tick()
{
	if(axis2_cooldown_batches > 0) axis2_cooldown_batches--;
	return axis2_cooldown_batches;
}

// START_CONNECTION bare-ACK causal guard.
//
// The responder's deliberate post-decode race delay is:
//   D = (connect base + suffix - connect threshold) * floor(symbol_ms) + 200.
// D is clamped to START_ACK_RESPONDER_MIN_DELAY_MS in the responder, making it
// a protocol minimum rather than a station-local threshold-tuning side effect.
//
// The commander timer is anchored immediately before the first START waveform
// sample enters tx_transfer(). A peer cannot complete the CRC-protected START
// decode before the whole START waveform has been transferred, and then must
// wait at least the fixed protocol minimum. With remote PTT/pilot minima of
// zero, the peer-independent earliest ACK audio is therefore:
//   ceil(start_pattern_samples * 1000 / 48000) + protocol_min.
// No commander-local threshold, symbol geometry, PTT value, capture reset,
// logging, lock, scheduler, or post-TX delay enters this lower bound.
//
// The acceptance floor adds one complete ACK-pattern duration using ceiling
// sample arithmetic. At and after that floor, receive_ack_pattern() masks every
// retained sample older than the causal audio epoch in both newest and older
// multiwindow phases. No delayed/destructive ring reset is involved, so a late
// first scheduler tick cannot erase a genuine ACK.
int cl_arq_controller::start_ack_responder_delay_ms(
	int connect_pattern_nsymb,
	int connect_suffix_nsymb,
	int connect_match_threshold,
	int symbol_samples)
{
	long long remaining = (long long)connect_pattern_nsymb
		+ (long long)connect_suffix_nsymb
		- (long long)connect_match_threshold;
	if(remaining < 0) remaining = 0;
	const long long symbol_ms = symbol_samples > 0
		? ((long long)symbol_samples * 1000LL) / 48000LL
		: 0LL;
	long long responder_race_delay_ms =
		remaining * symbol_ms + 200LL;
	if(responder_race_delay_ms < START_ACK_RESPONDER_MIN_DELAY_MS)
		responder_race_delay_ms = START_ACK_RESPONDER_MIN_DELAY_MS;
	if(responder_race_delay_ms > 0x7FFFFFFFLL) return 0x7FFFFFFF;
	return (int)responder_race_delay_ms;
}

int cl_arq_controller::start_ack_causal_audio_epoch_ms(
	int start_pattern_samples)
{
	const long long samples =
		start_pattern_samples > 0 ? (long long)start_pattern_samples : 0LL;
	const long long start_pattern_ms =
		(samples * 1000LL + 47999LL) / 48000LL;
	const long long epoch_ms =
		start_pattern_ms + (long long)START_ACK_RESPONDER_MIN_DELAY_MS;
	if(epoch_ms > 0x7FFFFFFFLL) return 0x7FFFFFFF;
	return (int)epoch_ms;
}

int cl_arq_controller::start_ack_causal_accept_floor_ms(
	int start_pattern_samples,
	int ack_pattern_samples)
{
	const int epoch_ms = start_ack_causal_audio_epoch_ms(
		start_pattern_samples);
	const long long ack_samples =
		ack_pattern_samples > 0 ? (long long)ack_pattern_samples : 0LL;
	const long long ack_pattern_ms =
		(ack_samples * 1000LL + 47999LL) / 48000LL;
	const long long accept_floor_ms =
		(long long)epoch_ms + ack_pattern_ms;
	if(accept_floor_ms > 0x7FFFFFFFLL) return 0x7FFFFFFF;
	return (int)accept_floor_ms;
}

uint32_t cl_arq_controller::arm_start_ack_causal_capture(
	int samples_through_start_egress)
{
	cl_data_container& dc = telecom_system->data_container;
	const uint64_t now_ns = sim_clock_now_ns();
	uint64_t epoch_ns = 0;
	if(arq_sim_inproc_active())
	{
		// The in-process pump has no output device queue. Its shared virtual
		// clock advances at exactly 48 kHz.
		epoch_ns = (uint64_t)start_ack_causal_audio_epoch_ms(
			samples_through_start_egress) * 1000000ULL;
	}
	else
	{
		// The count includes everything already queued ahead of START (for
		// example an optional pilot), plus the START waveform itself. The
		// audio boundary adds the opened sink's retained-device upper bound.
		epoch_ns = playback_causal_egress_bound_ns(
			samples_through_start_egress);
		const uint64_t response_ns =
			(uint64_t)START_ACK_RESPONDER_MIN_DELAY_MS * 1000000ULL;
		if(epoch_ns > UINT64_MAX - response_ns)
			epoch_ns = UINT64_MAX;
		else
			epoch_ns += response_ns;
	}
	uint64_t deadline_ns = now_ns + epoch_ns;
	if(deadline_ns < now_ns) deadline_ns = UINT64_MAX;

	uint32_t generation = dc.start_ack_causal_generation.load() + 1U;
	if(generation == 0) generation = 1U;

	// Publish payload first and generation last. The capture producer treats a
	// newly observed generation's entire current chunk as pre-causal, so even a
	// producer already returning from audio->read at this instant is excluded.
	dc.start_ack_causal_deadline_ns = deadline_ns;
	dc.start_ack_causal_ring_generation = 0;
	dc.start_ack_causal_ring_samples = 0;
	dc.start_ack_causal_generation = generation;

	start_ack_causal_timer.stop();
	start_ack_causal_timer.reset();
	start_ack_causal_timer.start();
	return generation;
}

void cl_arq_controller::refresh_start_ack_causal_after_playback_drain()
{
#ifndef START_ACK_CAUSAL_GUARD_FAILBEFORE
	cl_data_container& dc = telecom_system->data_container;
	const uint64_t now_ns = sim_clock_now_ns();
	uint64_t remaining_ns = playback_causal_egress_bound_ns(0);
	const uint64_t response_ns =
		(uint64_t)START_ACK_RESPONDER_MIN_DELAY_MS * 1000000ULL;
	if(remaining_ns > UINT64_MAX - response_ns)
		remaining_ns = UINT64_MAX;
	else
		remaining_ns += response_ns;

	uint64_t refreshed_deadline_ns = UINT64_MAX;
	if(remaining_ns <= UINT64_MAX - now_ns)
		refreshed_deadline_ns = now_ns + remaining_ns;

	// A nominal queue-duration estimate made before START cannot bound an
	// unexpectedly stalled playback consumer. Once the ring is observed
	// drained, retain the later of that estimate and a fresh sink/responder
	// boundary. The capture reset that follows may advance the generation, but
	// it must not inherit an already-expired deadline.
	const uint64_t current_deadline_ns =
		dc.start_ack_causal_deadline_ns.load();
	if(refreshed_deadline_ns > current_deadline_ns)
		dc.start_ack_causal_deadline_ns = refreshed_deadline_ns;
#endif
}

int cl_arq_controller::start_ack_causal_eligible_tail_samples(
	int causal_ring_samples,
	int phase_shift_samples,
	int tail_samples)
{
	if(causal_ring_samples < 0) return tail_samples;
	long long eligible = (long long)causal_ring_samples
		- (long long)phase_shift_samples;
	if(eligible <= 0) return 0;
	if(eligible >= tail_samples) return tail_samples;
	return (int)eligible;
}

bool cl_arq_controller::start_ack_causal_should_poll(
	int causal_ring_samples,
	int ack_pattern_samples)
{
	// A complete current-generation ACK waveform must have reached the ring
	// before the detector may inspect the newest or any retained older phase.
#ifdef START_ACK_CAUSAL_GUARD_FAILBEFORE
	(void)causal_ring_samples;
	(void)ack_pattern_samples;
	return true;
#else
	return causal_ring_samples >= ack_pattern_samples;
#endif
}

// ROBUST-TIER FIRST-RUNG PIPELINE regression (gearshift-start-and-recovery.md §10.5).
// Drives the REAL production gate inband_pipeline_climb_active() on the robust tier. The
// FIRST ROBUST rung must pipeline (return true) while climbing, so the ROBUST_0->1->2
// dwell does not serialize a clean-batch round-trip per rung. Off-feature / OFDM-tier /
// at-top -> false (legacy). PURE in-process synthetic-fire — permanent regression gate.
// Fails-before: -DINBAND_ROBUST_PIPELINE_FAILBEFORE pins the robust-tier branch off.
void cl_arq_controller::apply_connect_seed_cross(int seed_cfg)
{
	// CONNECT-SEED FUSION apply (CMD). Load the seed config the RSP already loaded as part
	// of the fused SWITCH_BANDWIDTH switch, running the SAME apply steps the SET_CONFIG-ACK
	// cross runs (gearshift-timer reset, geometry load, control-turnaround guard, per-config
	// TX re-chunk) MINUS the eliminated SET_CONFIG frame airtime. Mirrors the proven
	// SET_CONFIG-ACK apply body; the original branch is left untouched.
	gear_shift_timer.stop();
	gear_shift_timer.reset();
	negotiated_configuration = seed_cfg;
	data_configuration = seed_cfg;
	if(data_configuration != current_configuration)
	{
		messages_control_backup();
		load_configuration(data_configuration, PHYSICAL_LAYER_ONLY, YES);
		messages_control_restore();
		printf("[GEARSHIFT] FUSED connect-seed loaded config %d (SET_CONFIG cross collapsed)\n", data_configuration);
		fflush(stdout);
		// R1-rescope: arm the post-control-turnaround wait so the first data frame does not
		// key inside the peer's PHY-reinit window (same as the SET_CONFIG cross).
		arm_control_turnaround_guard();
		if(phy_reinit_settle_us > 0)
		{
			if(arq_sim_inproc_active())
				pumped_settle_wait(phy_reinit_settle_us / 1000);
			else
				usleep(phy_reinit_settle_us);
		}
		// Re-fill TX messages for the new config's message sizes (per-config re-chunk).
		for(int i=0;i<nMessages;i++)
			messages_tx[i].status=FREE;
		int data_read_size;
		for(int i=0;i<get_nTotal_messages();i++)
		{
			data_read_size=fifo_buffer_backup.pop(message_TxRx_byte_buffer,max_data_length+max_header_length);
			if(data_read_size!=0)
				fifo_buffer_tx.push(message_TxRx_byte_buffer,data_read_size);
			else
				break;
		}
		fifo_buffer_backup.flush();
	}
	this->connection_status=TRANSMITTING_DATA;
}

int cl_arq_controller::test_break_weld()
{
	const char* TAG = "[TEST-BREAK-WELD]";
	int failed = 0;
	auto check = [&](bool ok, const char* what, int got, int want) {
		if(ok) printf("%s PASS: %s (got=%d want=%d)\n", TAG, what, got, want);
		else { printf("%s FAIL: %s (got=%d want=%d)\n", TAG, what, got, want); failed++; }
		fflush(stdout);
	};
	auto put_env = [](const char* name, const char* value) {
#ifdef _WIN32
		_putenv_s(name, value);
#else
		if(value && *value) setenv(name, value, 1);
		else unsetenv(name);
#endif
	};

	put_env("MERCURY_BREAK_PIN_CONFIG", "");
	check(break_weld_target(CONFIG_15) == CONFIG_15,
		"control: unset knob preserves the recovery target",
		break_weld_target(CONFIG_15), CONFIG_15);
	put_env("MERCURY_BREAK_PIN_CONFIG", "not-a-config");
	check(break_weld_target(CONFIG_14) == CONFIG_14,
		"invalid text fails open to the recovery target",
		break_weld_target(CONFIG_14), CONFIG_14);
	put_env("MERCURY_BREAK_PIN_CONFIG", "9999");
	check(break_weld_target(CONFIG_13) == CONFIG_13,
		"off-ladder config fails open to the recovery target",
		break_weld_target(CONFIG_13), CONFIG_13);
	put_env("MERCURY_BREAK_PIN_CONFIG", "4294967312");
	check(break_weld_target(CONFIG_13) == CONFIG_13,
		"64-bit overflow text cannot wrap through int to config 16",
		break_weld_target(CONFIG_13), CONFIG_13);
	put_env("MERCURY_BREAK_PIN_CONFIG", "16");
	check(break_weld_target(CONFIG_0) == CONFIG_16,
		"armed knob returns a demoting BREAK to the pinned config",
		break_weld_target(CONFIG_0), CONFIG_16);
	check(break_weld_target(ROBUST_0) == CONFIG_16,
		"armed knob also defeats a panic target",
		break_weld_target(ROBUST_0), CONFIG_16);
	{
		const int raw = CONFIG_0;
		const int clamped = CONFIG_0;
		const int capped = CONFIG_0;
		const int final_target = break_weld_target(capped);
		bool raise_cap_log = break_raise_cap_log_needed(raw, clamped, capped, final_target);
		bool anchor_floor_log = break_anchor_floor_log_needed(raw, clamped, capped, final_target);
		check(!raise_cap_log,
			"weld-only target change is not attributed to the rung-floor raise-cap",
			raise_cap_log ? 1 : 0, 0);
		check(!anchor_floor_log,
			"weld-only target change is not attributed to the anchor floor",
			anchor_floor_log ? 1 : 0, 0);
	}
	put_env("MERCURY_BREAK_PIN_CONFIG", "");
	printf("%s %s: failures=%d\n", TAG, failed ? "FAIL" : "PASS", failed);
	fflush(stdout);
	return failed == 0 ? 0 : 1;
}

// ===========================================================================
// SIM_INPROC feasibility prototype — single-process in-process self-loopback.
// See fact-documents/single-process-sim-refactor.md.
//
// Proves the make-or-break risk of the single-process sim refactor: that the
// TX-path blocking spin-loops (ptt_busy_wait + drain_playback_wait, the two
// shapes that recur ~24x) become STEP-PUMPABLE under a single-thread stepper
// with NO concurrent drainer thread, while the spin-EXIT timing is IDENTICAL
// to the two-process paced sim (clock past delay / ring drained — never early).
// ===========================================================================

// The pump-setter is defined in arq_common.cc next to the spin helpers. The
// two production spin-loop helpers (ptt_busy_wait / drain_playback_wait) are
// now declared in arq.h, so they are reachable here AND inside send_batch().
// SIM_INPROC drives the REAL send_batch() (arq_common.cc) end-to-end rather
// than re-invoking the helpers from a synthetic sequence — the helpers run
// inside production code, proving the actual send-path waits are step-pumpable.
typedef void (*sim_inproc_pump_fn)(void* ctx);
extern void arq_set_sim_inproc_pump(sim_inproc_pump_fn fn, void* ctx);
// STEPPER-CORE REWRITE Phase b: sim2_activate records the active instance's modulation
// so the outer-stepper TX-wait seams pace OFDM data per-symbol but keep the legacy
// co-routine pump for the MFSK handshake (arq_common.cc §10.4). SIM-internal (not in arq.h).
extern void arq_set_sim2_active_is_ofdm(bool on);
// STEPPER-CORE REWRITE Phase b: the drain-ONLY pump the OFDM-data drain_playback_wait spins to
// egress a big-block play -> wire without bursting it to the RX (§10.4). SIM-internal.
extern void arq_set_sim_inproc_drain_only_pump(sim_inproc_pump_fn fn);
// STEPPER-CORE REWRITE Phase b: [SIM2-DEADLOCK-BREAK] floor fire counter (--test-sim-sustain).
extern void arq_reset_sim2_deadlock_break_count();
extern long arq_get_sim2_deadlock_break_count();

namespace {
// Step-pump context: the single-thread stepper's view of the loopback channel.
// One rx_transfer-sized "symbol" of channel time per pump step, mirroring the
// capture-prep thread's symbol_period reads (audioio.c:1279,1295).
struct SimInprocPumpCtx {
	int    symbol_period_samples = 0;  // == Nofdm * interpolation_rate
	double* scratch = nullptr;         // symbol_period_samples doubles
	long long pump_calls = 0;
	long long looped_samples = 0;      // playback bytes looped back into capture
	long long idle_samples = 0;        // silence injected when playback empty
	long long clock_samples = 0;       // total samples advanced via rx_transfer
};

// The step-pump. Invoked from INSIDE ptt_busy_wait / drain_playback_wait when
// installed. Does, per call, exactly what the two-process sibling threads do:
//   1. Loopback-drain: move all of playback_buffer into capture_buffer.
//   2. Clock-advance:  drain capture_buffer in symbol-sized rx_transfer reads,
//      which calls sim_clock_add_samples(len) — the SAME accounting as the
//      capture-prep thread (audioio.c:1295 -> 1689). The virtual clock thus
//      advances by exactly the samples that flowed.
//   3. Idle-fill: if there was nothing to loop AND nothing in capture, inject
//      one symbol of silence so the clock still advances during the PTT
//      pre-key delay (the two-process RX bridge always delivers channel audio,
//      so virtual time keeps ticking even before any TX). Without this the
//      ptt_busy_wait before the first tx_transfer could never advance time.
//
// This NEVER changes a wait's exit predicate — it only makes the existing
// predicate eventually true (clock crosses delay / playback empties).
void sim_inproc_pump(void* ctxv)
{
	SimInprocPumpCtx* ctx = static_cast<SimInprocPumpCtx*>(ctxv);
	const int sp = ctx->symbol_period_samples;
	if (sp <= 0 || ctx->scratch == nullptr) return;
	const size_t sp_bytes = (size_t)sp * sizeof(double);

	ctx->pump_calls++;

	// (1) Loopback-drain: playback -> capture, one symbol at a time, only
	// while capture has room (mirrors the RX-bridge backpressure guard).
	bool moved = false;
	while (size_buffer(playback_buffer) >= sp_bytes &&
	       circular_buf_free_size(capture_buffer) >= sp_bytes)
	{
		read_buffer(playback_buffer, (uint8_t*)ctx->scratch, sp_bytes);
		write_buffer(capture_buffer, (uint8_t*)ctx->scratch, sp_bytes);
		ctx->looped_samples += sp;
		moved = true;
	}

	// (2) Clock-advance: drain capture via rx_transfer (advances sim clock).
	bool drained = false;
	while (size_buffer(capture_buffer) >= sp_bytes)
	{
		rx_transfer(ctx->scratch, sp);   // -> sim_clock_add_samples(sp)
		ctx->clock_samples += sp;
		drained = true;
	}

	// (3) Idle-fill: nothing moved/drained -> advance one symbol of silence so
	// the PTT pre-key delay can elapse (no playback yet, empty capture).
	if (!moved && !drained)
	{
		memset(ctx->scratch, 0, sp_bytes);
		write_buffer(capture_buffer, (uint8_t*)ctx->scratch, sp_bytes);
		rx_transfer(ctx->scratch, sp);
		ctx->idle_samples += sp;
		ctx->clock_samples += sp;
	}
}

// A pump that cannot advance a symbol turns every SIM_INPROC TX wait into an
// unbounded spin. Validate the complete context before publishing the callback
// globally so callers can fail closed without entering send_batch().
bool sim_inproc_validate_pump(SimInprocPumpCtx* ctx)
{
	if (ctx == nullptr || ctx->symbol_period_samples <= 0 || ctx->scratch == nullptr)
	{
		arq_set_sim_inproc_pump(nullptr, nullptr);
		fprintf(stderr, "[SIM_INPROC] ERROR: refusing unusable step-pump "
		        "(symbol_period=%d, scratch=%s)\n",
		        ctx == nullptr ? 0 : ctx->symbol_period_samples,
		        (ctx != nullptr && ctx->scratch != nullptr) ? "set" : "NULL");
		fflush(stderr);
		return false;
	}
	return true;
}
}  // namespace

int cl_arq_controller::test_sim_inproc_pump_fail_closed()
{
	arq_set_sim_inproc_pump(nullptr, nullptr);

	SimInprocPumpCtx pump_ctx;
	pump_ctx.symbol_period_samples = 128;
	const bool null_scratch_rejected = !sim_inproc_validate_pump(&pump_ctx);
	const bool null_scratch_closed = !arq_sim_inproc_active();

	double scratch = 0.0;
	pump_ctx.symbol_period_samples = 0;
	pump_ctx.scratch = &scratch;
	const bool zero_period_rejected = !sim_inproc_validate_pump(&pump_ctx);
	const bool zero_period_closed = !arq_sim_inproc_active();

	arq_set_sim_inproc_pump(nullptr, nullptr);
	const bool passed = null_scratch_rejected && null_scratch_closed
	                 && zero_period_rejected && zero_period_closed;
	printf("[TEST-SIM-INPROC-PUMP] invalid scratch rejected=%d closed=%d; "
	       "zero period rejected=%d closed=%d: %s\n",
	       null_scratch_rejected, null_scratch_closed,
	       zero_period_rejected, zero_period_closed, passed ? "PASS" : "FAIL");
	fflush(stdout);
	return passed ? 0 : 1;
}

int cl_arq_controller::test_sim_inproc()
{
	int failed = 0;
	auto check = [&](bool cond, const char* name) {
		printf("[TEST-SIM-INPROC] %s: %s\n", cond ? "PASS" : "FAIL", name);
		if(!cond) failed++;
		fflush(stdout);
	};

	printf("[TEST-SIM-INPROC] single-instance in-process self-loopback "
	       "(no audio device, no bridge/prep threads, no TCP, no relay)\n");
	fflush(stdout);

	// --- 1. PHY + ARQ setup via the REAL production load_configuration ---
	// INCREMENT step (3) (single-process-sim-refactor.md §5.6 step 3): drive the
	// REAL production send path end-to-end, not the prior synthetic 3-call
	// sequence. We therefore must bring up the SAME ARQ state the production
	// commander has before send_batch(): message buffers (messages_tx[],
	// messages_batch_tx[], message_TxRx_byte_buffer), max_header_length,
	// nMessages, data_container PHY + passband_delayed_data, etc.
	//
	// cl_arq_controller::load_configuration(cfg, FULL, NO) does ALL of that
	// (set_max_buffer_length + set_nMessages + init_messages_buffers at
	// arq_common.cc:1455/1456/1659) AND calls telecom_system->load_configuration
	// internally (arq_common.cc:1421) — WITHOUT touching the TCP sockets (those
	// are only init()'d in cl_arq_controller::init(), which we deliberately do
	// NOT call: SIM_INPROC binds NO socket). The ctor seated
	// current_configuration=CONFIG_0 (arq_common.cc:393), and ROBUST_0 (100) !=
	// CONFIG_0, so load_configuration proceeds rather than skipping.
	//
	// ROBUST_0 (MFSK) is the smallest, fastest-to-emit frame; it exercises the
	// same PTT/TX/drain spin sites send_batch() runs for every OFDM config.
	robust_enabled     = YES;
	narrowband_enabled = NO;
	telecom_system->narrowband_enabled = NO;
	load_configuration(ROBUST_0, FULL, NO);   // ARQ-side: buffers + PHY, no TCP

	cl_data_container* dc = &telecom_system->data_container;
	int active_nsymb = telecom_system->get_active_nsymb();
	int symbol_period = dc->Nofdm * dc->interpolation_rate;
	int frame_output_size = dc->Nofdm * dc->interpolation_rate *
	                        (active_nsymb + dc->preamble_nSymb);
	check(symbol_period > 0, "A0 PHY config loaded (symbol_period > 0)");
	check(frame_output_size > 0, "A1 frame output size > 0");
	check(current_configuration == ROBUST_0, "A1b ARQ load_configuration set current_configuration=ROBUST_0");
	check(message_TxRx_byte_buffer != nullptr && messages_batch_tx != nullptr && messages_tx != nullptr,
	      "A1c ARQ message buffers allocated (init_messages_buffers ran, no TCP)");
	printf("[TEST-SIM-INPROC] symbol_period=%d frame_output=%d active_nsymb=%d "
	       "preamble_nSymb=%d max_header_length=%d nMessages=%d\n", symbol_period,
	       frame_output_size, active_nsymb, dc->preamble_nSymb, max_header_length,
	       nMessages);
	fflush(stdout);

	// Validate the allocation and derived geometry before installing the pump
	// or enabling the deterministic clock. Otherwise send_batch() would enter
	// blocking waits whose installed pump can make no progress.
	SimInprocPumpCtx pump_ctx;
	pump_ctx.symbol_period_samples = symbol_period;
	if (symbol_period > 0)
		pump_ctx.scratch = (double*)malloc((size_t)symbol_period * sizeof(double) * 2);
	if (!sim_inproc_validate_pump(&pump_ctx))
	{
		check(false, "A2 step-pump context valid before installation");
		if (pump_ctx.scratch != nullptr) free(pump_ctx.scratch);
		return 1;
	}

	// passband_delayed_data + capture_prep_mutex are normally set up by
	// audioio_init (audioio.c:1719) and the PHY load. send_batch() locks
	// capture_prep_mutex and zeroes passband_delayed_data; the PHY load
	// allocated passband_delayed_data (data_container.cc:170). The mutex is a
	// device-init artifact SIM_INPROC skips — create it so the production
	// send_batch() MUTEX_LOCK/UNLOCK are real (uncontended: single thread).
#if defined(_WIN32)
	bool created_mutex = false;
	if (capture_prep_mutex == NULL)
	{
		capture_prep_mutex = CreateMutex(NULL, FALSE, NULL);
		created_mutex = true;
	}
#endif

	// --- 2. Audio ring buffers WITHOUT any device / bridge / prep thread ---
	// These are the same globals tx_transfer/rx_transfer use; we create them
	// here directly so the modem talks to ITSELF through them. No threads.
	bool created_buffers = false;
	if (capture_buffer == nullptr || playback_buffer == nullptr)
	{
		uint8_t* cap = (uint8_t*)malloc(AUDIO_PAYLOAD_BUFFER_SIZE);
		uint8_t* play = (uint8_t*)malloc(AUDIO_PAYLOAD_BUFFER_SIZE);
		capture_buffer  = circular_buf_init(cap,  AUDIO_PAYLOAD_BUFFER_SIZE);
		playback_buffer = circular_buf_init(play, AUDIO_PAYLOAD_BUFFER_SIZE);
		created_buffers = true;
	}
	clear_buffer(capture_buffer);
	clear_buffer(playback_buffer);
	check(capture_buffer != nullptr && playback_buffer != nullptr,
	      "A2 audio ring buffers ready (no device, no threads)");

	// --- 3. Engage the virtual clock (the -x sim time source) ---
	sim_clock_set_enabled(1);
	check(sim_clock_enabled() != 0, "A3 virtual clock engaged");

	// --- 4. Install the step-pump (the inline single-thread drainer) ---
	arq_set_sim_inproc_pump(sim_inproc_pump, &pump_ctx);
	// Ensure the SIM_INPROC post-delivery spin-abort (fix #1) is disarmed for this
	// single-instance stepper: sim_inproc_pump (NOT _2) never arms it, but a prior
	// test_sim_inproc_2() run in the same process may have left it set — reset so the
	// spin helpers here behave byte-identically to the unmodified single-instance path.
	arq_set_sim_inproc_deliver_done(false);
	check(true, "A4 valid step-pump installed");

	// Non-zero PTT delays so the two PTT waits inside send_batch() MUST advance
	// the virtual clock to exit — this is what proves the clock-advance pump
	// works (delay=0 would pass trivially without ever pumping). 100/200 ms ==
	// ini_parser defaults. send_batch() reads ptt_on_delay_ms / ptt_off_delay_ms.
	ptt_on_delay_ms  = 100;
	ptt_off_delay_ms = 200;

	// --- 5. Stage a REAL DATA_SHORT batch into the production batch array ---
	// This is exactly how the production commander stages a frame before
	// send_batch() (arq_commander.cc:1323-1341): set the messages_batch_tx[i]
	// fields + message_batch_counter_tx, then call send_batch(). The frame's
	// .data points at a pre-allocated buffer (messages_tx[0].data, allocated by
	// init_messages_buffers) — messages_batch_tx[i].data is NULL at init and in
	// production inherits a valid pointer via a struct-copy (arq_common.cc:1314
	// comment); we mirror that by aiming it at messages_tx[0].data.
	{
		const char* msg = "MERCURY";
		int nb = (int)strlen(msg);
		messages_batch_tx[0].type   = DATA_SHORT;
		messages_batch_tx[0].length = nb;
		messages_batch_tx[0].id     = 0;
		messages_batch_tx[0].nResends = nResends;
		messages_batch_tx[0].ack_timeout = ack_timeout_data;
		messages_batch_tx[0].status = ADDED_TO_BATCH_BUFFER;
		messages_batch_tx[0].sequence_number = 0;
		messages_batch_tx[0].batch_seq_id = 0;
		messages_batch_tx[0].data = messages_tx[0].data;  // real allocated buffer
		for(int i=0;i<nb;i++) messages_batch_tx[0].data[i] = msg[i];
		// Keep messages_tx[0] coherent so send_batch()'s post-TX ack bookkeeping
		// (arq_common.cc:4020-4033, indexed by .id) writes into a real slot.
		messages_tx[0].status = PENDING_ACK;
		message_batch_counter_tx = 1;
		connection_id = 1;
	}
	check(message_batch_counter_tx == 1, "A5 DATA_SHORT batch staged into messages_batch_tx[]");

	// --- 6. Run the REAL production send_batch() inline (no concurrent thread) ---
	// send_batch() (arq_common.cc:3629) is the EXACT production send core. It
	// keys PTT, frames + FIR-filters the batch, runs ptt_busy_wait(on),
	// tx_transfers each frame, drain_playback_wait()s the ring, resets the
	// capture ring, then ptt_busy_wait(off) + ptt_off. ALL THREE spin sites
	// (ptt_busy_wait@3930, drain_playback_wait@3967, ptt_busy_wait@3999) now
	// run inline, step-pumped by the installed pump. If ANY of them failed to
	// become step-pumpable, send_batch() would never return (hard deadlock) —
	// so a clean return is itself the proof of inline drive through production
	// code. We bracket the call with virtual-clock snapshots to confirm the
	// waits did NOT exit early (total >= on + airtime + off) and the clock did
	// not run away.
	uint64_t t_start = sim_clock_now_samples();
	long long looped_before = pump_ctx.looped_samples;

	send_batch();   // REAL production send-path core, driven inline.

	uint64_t t_end = sim_clock_now_samples();
	long long looped_in_batch = pump_ctx.looped_samples - looped_before;

	// FIDELITY: a clean return proves no deadlock at any of the 3 spin sites.
	check(true, "B1 send_batch() returned (all 3 internal spin sites stepped, NO deadlock)");
	// The playback ring was fully drained by the inline pump (drain_playback_wait
	// exited at size_buffer(playback)==0 — its UNCHANGED exit predicate).
	check(size_buffer(playback_buffer) == 0,
	      "B2 playback ring EMPTY after send_batch (drain_playback_wait exit predicate held)");
	// send_batch() consumed + reset the staged batch (message_batch_counter_tx
	// is cleared at arq_common.cc:4048 only after the full PTT/drain sequence).
	check(message_batch_counter_tx == 0,
	      "B3 send_batch consumed the batch (reached post-drain bookkeeping)");
	// The frame's audio samples looped back through the RX boundary inside the
	// real drain — i.e. real channel time, not a synthetic poke.
	check(looped_in_batch >= (long long)frame_output_size,
	      "B4 the batch's frame samples looped back through the RX boundary (real airtime)");

	// --- 7. Timing fidelity: the total virtual time send_batch() spanned is at
	// least on-delay + frame airtime + off-delay — NOT short-circuited. This is
	// the SAME analytic floor the synthetic prototype proved, now measured
	// across the REAL send_batch() body. (send_batch on ROBUST_0 emits the one
	// staged frame; airtime == frame_output_size samples.)
	double total_virtual_ms = (t_end - t_start) * 1000.0 / SIM_CLOCK_SAMPLE_RATE_HZ;
	double frame_airtime_ms = frame_output_size * 1000.0 / SIM_CLOCK_SAMPLE_RATE_HZ;
	double expected_min_ms  = (double)ptt_on_delay_ms + (double)ptt_off_delay_ms
	                        + frame_airtime_ms;
	printf("[TEST-SIM-INPROC] [REAL send_batch] virtual-time accounting: total=%.1fms "
	       "expected_min=%.1fms (on=%d off=%d frame_airtime=%.1fms) clk %llu->%llu\n",
	       total_virtual_ms, expected_min_ms, ptt_on_delay_ms, ptt_off_delay_ms,
	       frame_airtime_ms, (unsigned long long)t_start, (unsigned long long)t_end);
	printf("[TEST-SIM-INPROC] pump stats: calls=%lld looped=%lld idle=%lld "
	       "clock_samples=%lld looped_in_batch=%lld\n", pump_ctx.pump_calls,
	       pump_ctx.looped_samples, pump_ctx.idle_samples, pump_ctx.clock_samples,
	       looped_in_batch);
	fflush(stdout);
	// Allow one symbol of poll overshoot on each of the two PTT waits (the
	// pump advances in whole symbols — same quantization the prep thread has),
	// plus headroom for the start-of-batch capture-ring-reset idle pumping.
	double overshoot_tol_ms = 2.0 * symbol_period * 1000.0 / SIM_CLOCK_SAMPLE_RATE_HZ
	                        + 1.0;
	check(total_virtual_ms >= expected_min_ms - 0.001,
	      "B6 total virtual time >= on+airtime+off (send_batch waits did NOT exit early)");
	check(total_virtual_ms <= expected_min_ms + overshoot_tol_ms + frame_airtime_ms,
	      "B7 total virtual time bounded (no runaway clock; overshoot < 1 symbol/wait)");

	// --- 7b. Settle-wait FIX validation (single-process-sim-refactor.md §7) ---
	// The B1-B8 settle-wait fixes are single-instance-testable in two classes:
	//
	//  (i) GATE-OFF (B5 / ADD-ON1): sim_inproc_rx_mute_settle() must be a no-op
	//      under SIM_INPROC (the pump is installed -> no async audio thread to
	//      drain) AND the instantaneous circular_buf_reset() that follows it in
	//      every caller must still fire. We exercise the helper directly here
	//      (the pump is still installed at this point), measure that it consumes
	//      ~0 wall time, then verify a real circular_buf_reset() empties the ring.
	//
	//  (ii) VIRTUAL-CLOCK-IFY (B1-B4/B7/ADD-ON2): pumped_settle_wait(N) must
	//      advance the SHARED virtual clock by >= N ms (its exit predicate held,
	//      not exited early) when the pump is installed — the property that lets
	//      a peer instance see time pass during the wait. (The B1-B4/B7 CALL
	//      SITES themselves are responder/peer-side ACK/HAIL paths a single
	//      instance does not reach — those are audit-only this increment, §6.5 —
	//      but the shared mechanism is validated here.)
	{
		// (i) gate-off no-op under SIM_INPROC + reset still fires.
		// RX_MUTE_GUARD_MS is file-local to arq_common.cc; its value is 50ms.
		const int rx_mute_guard_ms = 50;
		uint64_t g0 = sim_clock_now_samples();
		auto wall0 = std::chrono::steady_clock::now();
		sim_inproc_rx_mute_settle(rx_mute_guard_ms);  // pump installed -> no-op
		auto wall1 = std::chrono::steady_clock::now();
		uint64_t g1 = sim_clock_now_samples();
		double gate_wall_ms = std::chrono::duration<double, std::milli>(wall1 - wall0).count();
		check(gate_wall_ms < (double)rx_mute_guard_ms,
		      "G1 sim_inproc_rx_mute_settle is a NO-OP under SIM_INPROC (no 50ms wall pause)");
		check(g1 == g0,
		      "G1b gate-off does not touch the virtual clock (drain is moot in-process)");
		// circular_buf_reset still fires: stage one symbol, reset, confirm empty.
		{
			memset(pump_ctx.scratch, 0, (size_t)symbol_period * sizeof(double));
			write_buffer(capture_buffer, (uint8_t*)pump_ctx.scratch,
			             (size_t)symbol_period * sizeof(double));
			bool had_data = size_buffer(capture_buffer) > 0;
			capture_reset_samples();
			check(had_data && size_buffer(capture_buffer) == 0,
			      "G2 circular_buf_reset still empties the capture ring after the gated no-op");
		}

		// (ii) pumped_settle_wait advances the shared virtual clock by >= N ms.
		const int settle_ms = 60;
		uint64_t p0 = sim_clock_now_samples();
		pumped_settle_wait(settle_ms);   // pump installed -> step-pumped wait
		uint64_t p1 = sim_clock_now_samples();
		double advanced_ms = (p1 - p0) * 1000.0 / SIM_CLOCK_SAMPLE_RATE_HZ;
		printf("[TEST-SIM-INPROC] pumped_settle_wait(%dms) advanced virtual clock %.1fms\n",
		       settle_ms, advanced_ms);
		fflush(stdout);
		check(advanced_ms >= (double)settle_ms - 0.001,
		      "G3 pumped_settle_wait advances the SHARED clock >= wait_ms (exit predicate held, no early exit)");
		// Bounded: one symbol of poll overshoot (same quantization as the pump).
		double settle_tol_ms = symbol_period * 1000.0 / SIM_CLOCK_SAMPLE_RATE_HZ + 1.0;
		check(advanced_ms <= (double)settle_ms + settle_tol_ms,
		      "G4 pumped_settle_wait does not over-advance (overshoot < 1 symbol)");
	}

	// --- 8. Teardown: uninstall pump, leave globals as we found them ---
	arq_set_sim_inproc_pump(nullptr, nullptr);
	if(pump_ctx.scratch) free(pump_ctx.scratch);
	if(created_buffers)
	{
		free(capture_buffer->buffer);
		circular_buf_free(capture_buffer);
		capture_buffer = nullptr;
		free(playback_buffer->buffer);
		circular_buf_free(playback_buffer);
		playback_buffer = nullptr;
	}
#if defined(_WIN32)
	if(created_mutex && capture_prep_mutex != NULL)
	{
		CloseHandle(capture_prep_mutex);
		capture_prep_mutex = NULL;
	}
#endif
	sim_clock_set_enabled(0);

	printf("[TEST-SIM-INPROC] %s (%d failure%s) — inline cycle %s\n",
	       failed == 0 ? "ALL PASS" : "FAILURES", failed, failed == 1 ? "" : "s",
	       failed == 0 ? "COMPLETED with NO concurrent drainer (CLEAN_NO_DRAINER)"
	                   : "did NOT complete cleanly");
	fflush(stdout);
	return failed == 0 ? 0 : 1;
}

// ===========================================================================
// 2-INSTANCE SIM_INPROC stepper (single-process-sim-refactor.md §10.5).
//
// Two full modems (A=COMMANDER, B=RESPONDER) + an in-process scalar-AWGN
// channel per direction, driven by a SINGLE-THREAD lockstep loop on a SHARED
// virtual clock. NO audio device, NO bridge/prep threads, NO TCP, NO relay.
// Generalizes the proven single-instance Stage-2 stepper (test_sim_inproc) to
// a real peer: the peer-dependent settle-waits (B4 CONNECT-suffix, B7 HAIL/
// CONNECT self-detect race, B8 HAIL RX-poll) are now exercised against an
// actual responder.
// ===========================================================================
namespace {

// Per-instance audio transport (single-process-sim-refactor.md §10.3). Two
// instances own genuinely independent rings; the stepper swaps the audioio.c
// globals to the ACTIVE instance's rings before driving it (single thread → the
// swap precedes every consumer). Production never swaps.
struct AudioCtx {
	cbuf_handle_t cap  = nullptr;
	cbuf_handle_t play = nullptr;
#if defined(_WIN32)
	HANDLE        mutex = NULL;
#else
	pthread_mutex_t mutex;
#endif
	uint8_t* cap_mem  = nullptr;
	uint8_t* play_mem = nullptr;

	void alloc() {
		cap_mem  = (uint8_t*)malloc(AUDIO_PAYLOAD_BUFFER_SIZE);
		play_mem = (uint8_t*)malloc(AUDIO_PAYLOAD_BUFFER_SIZE);
		cap  = circular_buf_init(cap_mem,  AUDIO_PAYLOAD_BUFFER_SIZE);
		play = circular_buf_init(play_mem, AUDIO_PAYLOAD_BUFFER_SIZE);
		clear_buffer(cap);
		clear_buffer(play);
#if defined(_WIN32)
		mutex = CreateMutex(NULL, FALSE, NULL);
#else
		pthread_mutex_init(&mutex, NULL);
#endif
	}
	void free_all() {
		if(cap)  { free(cap->buffer);  circular_buf_free(cap);  cap = nullptr; }
		if(play) { free(play->buffer); circular_buf_free(play); play = nullptr; }
#if defined(_WIN32)
		if(mutex) { CloseHandle(mutex); mutex = NULL; }
#else
		pthread_mutex_destroy(&mutex);
#endif
	}
};

// One full modem instance for the 2-instance stepper.
struct MercuryInstance {
	cl_telecom_system ts;
	cl_arq_controller arq;
	AudioCtx          audio;
	const char*       tag = "?";

	void wire() { arq.telecom_system = &ts; }
};

// The 2-instance step-pump context. The stepper sets {tx, rx, channels} before
// driving the active (tx) instance. The pump (installed via the SAME
// arq_set_sim_inproc_pump hook the single-instance stepper uses) is invoked from
// inside tx's blocking waits (ptt_busy_wait / drain_playback_wait / pacing
// floor). Per call it:
//   1. Drains tx.play -> tx->rx channel -> rx.cap (the real tx->rx handoff),
//      advancing the SHARED clock; idle-fills one symbol of silence when tx has
//      nothing (the two-process TX bridge always sends silence on idle).
//   2. Runs rx's prep-pull (rx.cap -> rx.passband_delayed_data).
//   3. CO-ROUTINE INTERLEAVE: at depth 0, drives rx->process_main() ONCE so the
//      PEER can react to the audio it just received WHILE tx is still blocked in
//      its wait (mirrors the two-process concurrency where rx's prep+ARQ run
//      while tx transmits). Without this, tx's long PTT/response waits would
//      pump tens of silence symbols into rx and BURY the just-arrived beacon
//      before rx ever scanned it (the §10.5 single-thread hazard). A depth guard
//      (g_sim2_depth) bounds reentrancy to ONE peer level: when rx's own waits
//      fire the pump (depth 1), it only feeds rx->tx + advances the clock (so
//      rx's reply reaches tx's capture) — it does NOT recurse into tx again.
// Exit predicates are UNCHANGED (drain exits at tx.play empty; ptt exits at
// clock past delay) — the pump only makes them eventually true.
struct SimInproc2Ctx {
	MercuryInstance* tx = nullptr;     // active instance (drains its playback)
	MercuryInstance* rx = nullptr;     // peer (receives the channel output)
	cl_sim_awgn*     ch_tx2rx = nullptr; // tx -> rx direction
	cl_sim_awgn*     ch_rx2tx = nullptr; // rx -> tx direction (peer-drive reply)
	// Per-direction WIRE rings (single-process-sim §10.5). Drained TX samples
	// (post-channel) are buffered here, NOT written straight to the receiver's
	// capture. The receiver is fed from its incoming wire only at controlled
	// points (its own drive turn / its listen polls), so a sender's post-TX
	// capture FLUSH (send_*_pattern self-echo reset) cannot wipe a reply that the
	// peer emitted while the sender was still transmitting. wire_t2r is the
	// tx->rx wire, wire_r2t the rx->tx wire (re-pointed each half-step).
	cl_sim_awgn*     dummy = nullptr;
	cbuf_handle_t    wire_t2r = nullptr;
	cbuf_handle_t    wire_r2t = nullptr;
	double* scratch = nullptr;         // >= 3 symbols of doubles
	int    sp_max = 0;                 // max symbol_period across both instances
	long long pump_calls = 0;
	long long clock_samples = 0;
	long long looped_samples = 0;      // real airtime moved tx->rx
	long long idle_samples = 0;
	// SIM_INPROC post-delivery spin-abort (SIMFTR_ROOTCAUSE.md §7 fix #1). The pump
	// runs DURING the commander's post-transfer keepalive spin; these let it observe
	// the RESPONDER's delivered-byte count from inside that spin and arm the abort
	// (arq_set_sim_inproc_deliver_done) the instant the full payload has landed, so
	// process_main unwinds back to the outer stepper. deliver_rx = the RESPONDER's
	// arq (whose fifo_buffer_rx the outer loop drains); deliver_target = payload_len;
	// deliver_drained = pointer to the outer loop's running rx_have (bytes ALREADY
	// popped out of the FIFO this run) so the cumulative count (drained + currently
	// buffered) is compared against the target. Null deliver_rx (the single-instance
	// stepper, sim_inproc_pump) disables the probe — that path is unaffected.
	cl_arq_controller* deliver_rx = nullptr;
	long  deliver_target = 0;
	const int* deliver_drained = nullptr;
	bool  deliver_defeat = false;      // fail-before A/B hook: skip arming the abort
	// Pump-call wall watchdog (fail-before arm only). When deliver_defeat is set the
	// post-delivery abort never arms, so the wedge reproduces. To make that arm
	// terminate DETERMINISTICALLY (instead of hanging until an outer timeout) the pump
	// force-aborts after this many calls and sets deliver_stalled so the test reports
	// the wedge as a FAIL rather than a clean delivery. 0 = disabled (the real fix arm
	// never needs it — it aborts on the genuine delivery predicate within a few calls).
	long long deliver_watchdog_calls = 0;
	long long deliver_watchdog_anchor = -1;  // pump_calls at first full-delivery (fail-before)
	bool  deliver_stalled = false;     // set true iff the watchdog tripped (wedge seen)
};

// Reentrancy depth for the co-routine peer-drive (§10.5). 0 = top (tx) level;
// 1 = inside a peer rx->process_main() drive (do not recurse further).
static int g_sim2_depth = 0;

// Reentrancy guard for the per-FRAME RX decode-drive inside sim2_deliver_from_wire
// (wf-sim-controlloop OFDM fix, bounded follow-on). >0 = a decode-drive is already
// on the stack; a nested deliver (pump-fired from dst's pacing wait) must not
// recurse the drive again.
static int g_sim2_decode_drive_depth = 0;

void prep_pull_inline(MercuryInstance* inst, double* buffer_temp);  // fwd decl
void sim2_activate(MercuryInstance* m);                             // fwd decl

// Drain src's playback through the channel into the src->dst WIRE (whole symbols),
// advancing the shared clock by the drained airtime; idle-fills ONE symbol of
// silence into the wire when src has nothing (so the clock keeps ticking and the
// receiver sees the noise floor). Returns true if real (non-silence) signal moved.
// The wire decouples TX-drain from RX-feed so a sender's post-TX capture flush
// cannot wipe a reply in flight (single-process-sim §10.5).
bool sim2_drain_to_wire(MercuryInstance* src, cl_sim_awgn* ch, cbuf_handle_t wire,
                        SimInproc2Ctx* c)
{
	cl_data_container* sdc = &src->ts.data_container;
	int sp = sdc->Nofdm * sdc->interpolation_rate;
	if (sp <= 0) return false;
	size_t sp_bytes = (size_t)sp * sizeof(double);
	bool moved = false;
	while (size_buffer(src->audio.play) >= sp_bytes &&
	       circular_buf_free_size(wire) >= sp_bytes)
	{
		read_buffer(src->audio.play, (uint8_t*)c->scratch, sp_bytes);
		if (ch) ch->process(c->scratch, (size_t)sp);
		write_buffer(wire, (uint8_t*)c->scratch, sp_bytes);
		c->looped_samples += sp;
		c->clock_samples  += sp;
		sim_clock_add_samples((uint64_t)sp);
		moved = true;
	}
	if (!moved)
	{
		if (circular_buf_free_size(wire) >= sp_bytes)
		{
			memset(c->scratch, 0, sp_bytes);
			if (ch) ch->process(c->scratch, (size_t)sp);
			write_buffer(wire, (uint8_t*)c->scratch, sp_bytes);
		}
		c->idle_samples  += sp;
		c->clock_samples += sp;
		sim_clock_add_samples((uint64_t)sp);
	}
	return moved;
}

// Deliver pending WIRE samples into dst's capture (respecting free space) then
// run dst's prep-pull (wire -> dst.cap -> dst.passband_delayed_data). Called for
// the RECEIVER at controlled points so a sender's flush never races a reply.
//
// SINGLE-SYMBOL PACING (wf-sim-controlloop OFDM fix): move EXACTLY ONE symbol
// wire->cap, then prep that one symbol into the ring, per loop iteration. The
// old form drained the WHOLE wire into cap and ran prep_pull_inline ONCE, so a
// VARIABLE run of idle-silence symbols (sim2_drain_to_wire idle-fills one
// silence symbol per pump but drains the WHOLE play buffer when sending) plus
// the entire data frame advanced the RX ring_write_index by a variable count
// between two consecutive OFDM decode attempts. The OFDM data preamble's
// ABSOLUTE ring offset therefore jittered by whole symbols (delay 171119/
// 174839/172359, var=nan), the Schmidl-Cox first-peak coarse search chased a
// picture shifting under it, and CFG15/16 never decoded (FTR 0.0-0.62).
// MFSK/ROBUST_0 survived the same jitter via its wide-margin 1-D symbol-grid
// re-lock. Pacing one symbol per prep makes the ring advance exactly one
// symbol per prep opportunity, mirroring the production capture-prep thread's
// real-time 1-symbol-per-sim_paced_wait cadence (audioio.c:1295-1376), so the
// preamble offset is deterministic + frame-aligned every attempt. The data_ready/
// frames_to_read gate the decode consumer reads (arq_common.cc:6214) then fires
// at a stable ring_write_index. prep_pull_inline itself is unchanged.
//
// BOUNDED FOLLOW-ON — per-FRAME DECODE DRIVE (drive_decode=true). Single-symbol
// pacing alone is NECESSARY but not SUFFICIENT: the RX runs its OFDM decode only
// when its process_main() executes, and that happens ONCE per top-level half-step
// (and once per pump co-routine drive). Without a decode pass BETWEEN symbol
// deliveries, the per-symbol prep loop still drives frames_to_read down past 0
// (clamped to 0 in prep_pull_inline) and keeps advancing ring_write_index across
// the WHOLE delivered run before any decode snapshot fires (arq_common.cc:6214
// snapshots ONLY when frames_to_read==0) — so the decode still sees a jittered,
// over-advanced window and CFG15/16 never decode (verified: clean single-symbol
// pacing reaches the data phase but yields t2 var=nan / 0 t2-OK). In PRODUCTION
// the decode thread polls data_ready CONCURRENTLY with the 1-symbol-per-tick
// capture-prep feed, so it consumes a frame the moment frames_to_read hits 0, at
// the frame-aligned ring_write_index. We reproduce that here: on the FALLING EDGE
// of frames_to_read to 0 (a full frame just completed; the RX armed frames_to_read
// to preamble_nSymb+Nsymb at arq_common.cc:2531) drive dst->process_main() ONCE so
// it snapshots+decodes at that exact, deterministic ring offset before more symbols
// shift the window. Per-FRAME (not per-symbol) keeps the wall cost bounded.
// Guards:
//   - GATE: link_status==CONNECTED && is_ofdm_config(current_configuration). The
//     is_ofdm_config term is the PRIMARY guard — the MFSK CONNECT/HAIL handshake
//     runs at ROBUST_0 (is_ofdm_config=false), so the drive can NEVER fire during
//     the handshake (which works 400/400 and desynced when driven). ROBUST/MFSK
//     DATA also stays on the legacy once-per-half-step cadence (byte-identical).
//     NOTE: broadening link_status==CONNECTED to also accept
//     connection_status==RECEIVING was tested twice and REGRESSED the OFDM decode
//     (RX-BATCH=0) — keep the state guard strictly link_status==CONNECTED.
//   - drive_decode is true ONLY at call sites where dst is NOT already executing its
//     own process_main() (every site except the §10.5 reply-into-tx deliver, where
//     dst==the running tx instance — driving it would recurse into itself).
//   - g_sim2_decode_drive_depth blocks a nested deliver (pump-fired from dst's own
//     pacing wait) from recursing the decode-drive.
//   - g_sim2_depth is bumped across the drive so any nested pump only drains+clocks
//     (its depth-0 deliver/drive block is skipped), exactly like the existing
//     co-routine peer-drive at sim_inproc_pump_2.
void sim2_deliver_from_wire(MercuryInstance* dst, cbuf_handle_t wire, SimInproc2Ctx* c,
                            bool drive_decode = false)
{
	cl_data_container* ddc = &dst->ts.data_container;
	int sp = ddc->Nofdm * ddc->interpolation_rate;
	if (sp <= 0) return;
	size_t sp_bytes = (size_t)sp * sizeof(double);
	while (size_buffer(wire) >= sp_bytes &&
	       circular_buf_free_size(dst->audio.cap) >= sp_bytes)
	{
		read_buffer(wire, (uint8_t*)c->scratch, sp_bytes);
		write_buffer(dst->audio.cap, (uint8_t*)c->scratch, sp_bytes);
		// Prep the single symbol just delivered (cap holds exactly one symbol now,
		// so prep_pull_inline's internal while-loop runs exactly once and advances
		// ring_write_index by ONE symbol). One-symbol-per-prep == production cadence.
		prep_pull_inline(dst, c->scratch + c->sp_max);

		// Per-frame decode drive at the frame-complete boundary (data_ready==1 &&
		// frames_to_read==0). The production OFDM decode consumer (arq_common.cc:6214)
		// snapshots the ring ONLY when frames_to_read==0; the RX arms frames_to_read to
		// preamble_nSymb+Nsymb (one full frame) on entry to RECEIVING, so the boundary
		// lands the snapshot on a frame-aligned window. Driving the decode HERE (rather
		// than letting the per-symbol prep loop pour symbols past the boundary, which
		// over-advances ring_write_index before the once-per-half-step decode runs)
		// reproduces production's concurrent decode-thread timing. Gated to
		// link_status==CONNECTED (past the MFSK CONNECT/HAIL handshake — works 400/400,
		// must not be re-entered mid-flight; broadening this to connection_status==
		// RECEIVING was tested and REGRESSED the decode — keep it strictly CONNECTED)
		// and to OFDM configs (ROBUST/MFSK survive jitter via the 1-D grid re-lock and
		// stay on the legacy cadence, byte-identical). VERIFIED: B decodes the OFDM data
		// batch byte-correct (RX-BATCH-SEQ DATA_LONG, [OFDM-OK] t2 var=0.0022
		// meanH=1.000); pinned CFG16 delivers the full payload bytes_ok=1 in iters=4.
		if (drive_decode && g_sim2_decode_drive_depth == 0 &&
		    ddc->data_ready == 1 && ddc->frames_to_read == 0 &&
		    dst->arq.link_status == CONNECTED &&
		    is_ofdm_config(dst->arq.current_configuration))
		{
			static const bool ddbg = (getenv("MERCURY_SIM2_DBG") != nullptr);
			if (ddbg) {
				printf("[SIM2-DRIVE] %s cfg=%d conn=%d link=%d rwi=%d\n",
				       dst->tag ? dst->tag : "?", dst->arq.current_configuration,
				       dst->arq.connection_status, dst->arq.link_status,
				       (int)ddc->ring_write_index);
				fflush(stdout);
			}
			g_sim2_decode_drive_depth++;
			g_sim2_depth++;
			sim2_activate(dst);
			dst->arq.process_main();
			g_sim2_depth--;
			g_sim2_decode_drive_depth--;
		}
	}
}

// prep_pull_inline: one capture-prep PASS for `inst` (the body of
// radio_capture_prep_thread, audioio.c:1295-1376, minus the while(!shutdown_)
// and the sim_paced_wait). Moves whole symbols from inst's capture ring into
// inst's data_container.passband_delayed_data (double-mapped ring write +
// frames_to_read/data_ready/nUnder bookkeeping). Uses inst's OWN mutex + rings
// + data_container EXPLICITLY (not the globals) so it is correct regardless of
// which instance the globals currently point at.
void prep_pull_inline(MercuryInstance* inst, double* buffer_temp)
{
	cl_data_container* dc = &inst->ts.data_container;
	int symbol_period = dc->Nofdm * dc->interpolation_rate;
	if (symbol_period <= 0) return;
	size_t sp_bytes = (size_t)symbol_period * sizeof(double);

	while (size_buffer(inst->audio.cap) >= sp_bytes)
	{
		read_buffer(inst->audio.cap, (uint8_t*)buffer_temp, sp_bytes);
		// Per-instance virtual clock is NOT advanced here (the pump advances the
		// SHARED clock once per moved sample, §10.5). rx_transfer's clock-add is
		// NOT used on this path — we read inst's capture directly.

		if (dc->rx_mute) {
			memset(buffer_temp, 0, sp_bytes);
			dc->rx_mute_samples += symbol_period;
		}

		MUTEX_LOCK(&inst->audio.mutex);
		int sp = dc->Nofdm * dc->buffer_Nsymb * dc->interpolation_rate;
		if (sp == 0 || dc->passband_delayed_data == NULL || sp <= symbol_period) {
			MUTEX_UNLOCK(&inst->audio.mutex);
			continue;
		}
		if (dc->data_ready == 1 && dc->frames_to_read <= 0)
			dc->nUnder_processing_events++;

		int wi = dc->ring_write_index;
		int remaining = sp - wi;
		if (remaining >= symbol_period) {
			memcpy(&dc->passband_delayed_data[wi], buffer_temp, sp_bytes);
			memcpy(&dc->passband_delayed_data[wi + sp], buffer_temp, sp_bytes);
		} else {
			memcpy(&dc->passband_delayed_data[wi], buffer_temp, (size_t)remaining * sizeof(double));
			memcpy(&dc->passband_delayed_data[wi + sp], buffer_temp, (size_t)remaining * sizeof(double));
			int wrap = symbol_period - remaining;
			memcpy(&dc->passband_delayed_data[0], &buffer_temp[remaining], (size_t)wrap * sizeof(double));
			memcpy(&dc->passband_delayed_data[sp], &buffer_temp[remaining], (size_t)wrap * sizeof(double));
		}
		dc->ring_write_index = (wi + symbol_period) % sp;
		dc->frames_to_read--;
		if (dc->frames_to_read < 0) dc->frames_to_read = 0;
		dc->data_ready = 1;
		MUTEX_UNLOCK(&inst->audio.mutex);
	}
}

void sim2_activate(MercuryInstance* m);   // fwd decl (defined below)

// The 2-instance step-pump (see SimInproc2Ctx above for the full contract).
void sim_inproc_pump_2(void* ctxv)
{
	SimInproc2Ctx* c = static_cast<SimInproc2Ctx*>(ctxv);
	if (c->scratch == nullptr || c->tx == nullptr || c->rx == nullptr)
		return;
	c->pump_calls++;

	// SIM_INPROC post-delivery spin-abort probe (SIMFTR_ROOTCAUSE.md §7 fix #1). Run on
	// EVERY pump call, REGARDLESS of co-routine depth: the post-transfer wedge spins inside
	// the RESPONDER's own process_main, which is driven at depth 1 by the COMMANDER's depth-0
	// pump call — so that depth-0 call never returns and a depth-0-only probe is never
	// reached. The delivery state (responder's fifo_buffer_rx occupancy) is global, so the
	// abort can and must be armed from the depth-1 call too. The outer-loop delivery check
	// (rx_have >= payload_len) lives at the loop BOTTOM, unreachable while either peer is
	// wedged; this is the one place that CAN observe completion mid-spin. When the cumulative
	// delivered count (bytes already drained into the outer loop's rx_have PLUS bytes
	// currently in the responder's fifo_buffer_rx) reaches the payload target, arm the abort
	// so the spin helpers unwind BOTH peers' process_main back to the outer loop. The transfer
	// has SUCCEEDED at this point (responder holds every byte), so there is no in-flight PHY
	// frame whose pacing could be perturbed. deliver_defeat reproduces the pre-fix wedge.
	if (c->deliver_rx != nullptr && !arq_sim_inproc_deliver_done())
	{
		long buffered = (long)c->deliver_rx->fifo_buffer_rx.get_size()
		              - (long)c->deliver_rx->fifo_buffer_rx.get_free_size();
		long drained  = (c->deliver_drained != nullptr) ? (long)*c->deliver_drained : 0;
		bool fully_delivered = (buffered + drained >= c->deliver_target);
		if (!c->deliver_defeat)
		{
			// REAL FIX: arm the post-delivery abort the instant the responder holds the
			// whole payload, so the spin helpers unwind process_main back to the outer loop.
			if (fully_delivered)
				arq_set_sim_inproc_deliver_done(true);
		}
		else if (fully_delivered && c->deliver_watchdog_calls > 0)
		{
			// FAIL-BEFORE arm: the responder ALREADY holds the full payload (the transfer
			// SUCCEEDED) but we deliberately do NOT arm the abort — reproducing the wedge.
			// DELIVERY-ANCHORED watchdog: count pump calls SINCE first-full-delivery; once
			// the commander/responder have spun deliver_watchdog_calls past the delivery
			// point without the outer loop ever breaking (== the wedge: process_main never
			// returned), give up, mark stalled, and force-unwind so the run terminates as a
			// FAIL. Anchoring on delivery (not absolute pump_calls) makes it batch-size-
			// independent and guarantees it can only trip on a genuine POST-delivery spin.
			if (c->deliver_watchdog_anchor < 0)
				c->deliver_watchdog_anchor = c->pump_calls;
			else if (c->pump_calls - c->deliver_watchdog_anchor >= c->deliver_watchdog_calls)
			{
				c->deliver_stalled = true;
				arq_set_sim_inproc_deliver_done(true);
			}
		}
	}

	// (1) Drain tx's playback (real signal or idle silence) into the tx->rx wire,
	//     advancing the shared clock. NEVER writes to a capture here. `sending`
	//     = real signal was draining this call (tx is actively transmitting a
	//     frame, NOT idle/listening).
	bool sending = sim2_drain_to_wire(c->tx, c->ch_tx2rx, c->wire_t2r, c);

	if (g_sim2_depth == 0)
	{
		// (2) DEPTH 0 (tx is the top-level driven instance): deliver the tx->rx
		//     wire into rx's capture + prep rx, then co-routine-drive rx ONCE so
		//     rx can react WHILE tx is blocked. drive_decode=true: this is the
		//     PRIMARY OFDM data-frame delivery path (the data frame is drained to the
		//     wire and delivered to rx DURING tx's send wait); the per-frame decode
		//     at the frame-aligned ring offset must fire here (gated CONNECTED+OFDM).
		sim2_deliver_from_wire(c->rx, c->wire_t2r, c, /*drive_decode=*/true);

		// Deliver the rx->tx reply into tx ONLY when tx is NOT actively sending a
		// frame (tx.play empty AND no real signal drained this call). This is the
		// §10.5 deferral made precise: during a frame send the post-TX capture
		// FLUSH (send_*_pattern) would wipe an in-flight reply, so we hold it in
		// the wire; once tx finishes the frame and sits in a LISTEN/idle wait
		// (tx.play empty, the flush already done — no pump fires between drain-exit
		// and the flush), it is safe to deliver and tx's next scan sees the reply.
		if (!sending && size_buffer(c->tx->audio.play) == 0)
			sim2_deliver_from_wire(c->tx, c->wire_r2t, c);

		g_sim2_depth++;
		MercuryInstance* save_tx = c->tx;
		MercuryInstance* save_rx = c->rx;
		cl_sim_awgn* save_t2r = c->ch_tx2rx;
		cl_sim_awgn* save_r2t = c->ch_rx2tx;
		cbuf_handle_t save_wt2r = c->wire_t2r;
		cbuf_handle_t save_wr2t = c->wire_r2t;
		(void)sending;
		// Flip the ctx so rx's own waits (depth 1) drain rx->tx wire + clock.
		c->tx = save_rx; c->rx = save_tx;
		c->ch_tx2rx = save_r2t; c->ch_rx2tx = save_t2r;
		c->wire_t2r = save_wr2t; c->wire_r2t = save_wt2r;
		sim2_activate(save_rx);
		save_rx->arq.process_main();
		// Restore tx's view. Do NOT deliver rx->tx here (the §10.5 deferral):
		// the reply stays in the wire and is delivered to tx by a later pump fired
		// from tx's LISTEN/idle wait (tx.play empty) or by the outer loop.
		c->tx = save_tx; c->rx = save_rx;
		c->ch_tx2rx = save_t2r; c->ch_rx2tx = save_r2t;
		c->wire_t2r = save_wt2r; c->wire_r2t = save_wr2t;
		sim2_activate(save_tx);
		g_sim2_depth--;
	}
	// DEPTH 1 (we ARE the peer being co-routine-driven): only drained tx->rx wire
	// above (the reply path). Do NOT deliver into the original sender's capture.
	// (The post-delivery spin-abort probe runs at the TOP of this function so it fires on
	// every pump call regardless of co-routine depth — see there.)
}

// STEPPER-CORE REWRITE Phase b: DRAIN-TO-WIRE-ONLY pump (the OFDM DATA-path drain seam).
// Under the outer stepper, send_batch's drain_playback_wait must EGRESS the OFDM big-block out
// of the TX play ring BEFORE the same process_main's post-TX ACK path runs (which
// clear_buffer(playback_buffer)s any residual -> would DISCARD a block left in play). But it
// must NOT burst-DELIVER the block to the RX (that is the §8 cadence break + the depth>0
// reentrancy the rewrite kills). The resolution: drain tx.play -> the in-flight WIRE here
// (one symbol per call; the wire is 12.288 MB, ~19 blocks, so a single block never saturates
// it), then let the OUTER loop FEED the wire -> RX one symbol per iter (per-frame decode
// pacing preserved). This callback ONLY drains tx->wire (no deliver, no decode-drive, no
// peer-recurse), so play empties deterministically with zero RX-side burst. tx orientation
// (c->tx / c->wire_t2r) is set by the outer loop's current half-step, so it always drains the
// instance whose process_main is running.
void sim_inproc_drain_to_wire_only(void* ctxv)
{
	SimInproc2Ctx* c = static_cast<SimInproc2Ctx*>(ctxv);
	if (c->scratch == nullptr || c->tx == nullptr)
		return;
	c->pump_calls++;
	// Drain exactly one TX symbol (or one idle-fill) into the tx->rx wire. NO deliver.
	sim2_drain_to_wire(c->tx, c->ch_tx2rx, c->wire_t2r, c);
}

// ============================================================================
// OUTER-LOOP DRIVER per-symbol helpers (sim2-stepper-rewrite Phase a).
//
// These are thin re-slices of the EXISTING, validated drain/feed helpers,
// BOUNDED to exactly ONE symbol per call, so the new outer-loop driver
// (MERCURY_SIM2_STEPPER=outer) can advance the whole system by one symbol per
// instance per direction per outer iteration — mirroring production's two
// continuous audio threads (DAC drain + capture-prep feed). The DSP / prep /
// decode bodies are UNCHANGED from sim2_drain_to_wire / sim2_deliver_from_wire
// (that is what keeps GATE-2 + production byte-identical); the ONLY change is
// the per-symbol bound (the `while` becomes an `if`).
//
// PHASE a SCOPE: drain_playback_wait is NOT yet a no-op (that is Phase b), so the
// pump (sim_inproc_pump_2) still drains TX inside the blocking waits. These
// helpers only do the OUTER-loop top-level catch-up drain/feed that the legacy
// body does via the whole-buffer sim2_drain_to_wire / sim2_deliver_from_wire
// catch-up calls — now one symbol at a time. This phase proves the per-symbol
// helpers move bytes IDENTICALLY to the whole-buffer catch-up on the paths that
// do NOT wedge (the legacy 19-B MFSK arm), with no behavior change otherwise.

// Drain EXACTLY ONE symbol of `src`'s playback through `ch` into `wire`,
// advancing the shared clock by one symbol of airtime; idle-fill ONE symbol of
// silence when src has nothing queued (so the clock keeps ticking and the
// receiver sees the noise floor). One-symbol-bounded re-slice of
// sim2_drain_to_wire (arq_commander.cc:9927-9959). Returns true iff a real
// (non-silence) symbol moved this call.
bool sim_step_drain_one_symbol(MercuryInstance* src, cl_sim_awgn* ch,
                               cbuf_handle_t wire, SimInproc2Ctx* c)
{
	cl_data_container* sdc = &src->ts.data_container;
	int sp = sdc->Nofdm * sdc->interpolation_rate;
	if (sp <= 0) return false;
	size_t sp_bytes = (size_t)sp * sizeof(double);
	// Real symbol available AND room on the wire -> move exactly one.
	if (size_buffer(src->audio.play) >= sp_bytes &&
	    circular_buf_free_size(wire) >= sp_bytes)
	{
		read_buffer(src->audio.play, (uint8_t*)c->scratch, sp_bytes);
		if (ch) ch->process(c->scratch, (size_t)sp);
		write_buffer(wire, (uint8_t*)c->scratch, sp_bytes);
		c->looped_samples += sp;
		c->clock_samples  += sp;
		sim_clock_add_samples((uint64_t)sp);
		return true;
	}
	// Nothing to send -> idle-fill exactly one silence symbol (clock ticks once).
	if (circular_buf_free_size(wire) >= sp_bytes)
	{
		memset(c->scratch, 0, sp_bytes);
		if (ch) ch->process(c->scratch, (size_t)sp);
		write_buffer(wire, (uint8_t*)c->scratch, sp_bytes);
	}
	c->idle_samples  += sp;
	c->clock_samples += sp;
	sim_clock_add_samples((uint64_t)sp);
	return false;
}

// Consume EXACTLY ONE symbol from `wire` into `dst`'s capture, prep it into the
// ring (advancing ring_write_index by one symbol), and on the frames_to_read==0
// falling edge drive dst's OFDM decode ONCE (the GATE-2 per-frame decode pacing).
// One-iteration re-slice of the sim2_deliver_from_wire `while` body
// (arq_commander.cc:10024-10069) — SAME prep, SAME decode-drive gate + depth
// guards, only bounded to one symbol per call.
void sim_step_feed_one_symbol(MercuryInstance* dst, cbuf_handle_t wire,
                              SimInproc2Ctx* c, bool drive_decode = false)
{
	cl_data_container* ddc = &dst->ts.data_container;
	int sp = ddc->Nofdm * ddc->interpolation_rate;
	if (sp <= 0) return;
	size_t sp_bytes = (size_t)sp * sizeof(double);
	if (size_buffer(wire) < sp_bytes ||
	    circular_buf_free_size(dst->audio.cap) < sp_bytes)
		return;
	read_buffer(wire, (uint8_t*)c->scratch, sp_bytes);
	write_buffer(dst->audio.cap, (uint8_t*)c->scratch, sp_bytes);
	// Prep the single symbol just delivered (cap holds exactly one symbol now,
	// so prep_pull_inline's internal while-loop runs exactly once and advances
	// ring_write_index by ONE symbol). One-symbol-per-prep == production cadence.
	prep_pull_inline(dst, c->scratch + c->sp_max);

	// Per-frame decode drive at the frame-complete boundary (UNCHANGED gate +
	// depth guards from sim2_deliver_from_wire arq_commander.cc:10049-10068).
	if (drive_decode && g_sim2_decode_drive_depth == 0 &&
	    ddc->data_ready == 1 && ddc->frames_to_read == 0 &&
	    dst->arq.link_status == CONNECTED &&
	    is_ofdm_config(dst->arq.current_configuration))
	{
		g_sim2_decode_drive_depth++;
		g_sim2_depth++;
		sim2_activate(dst);
		dst->arq.process_main();
		g_sim2_depth--;
		g_sim2_decode_drive_depth--;
	}
}

// Configure one instance for the 2-instance stepper exactly like the non-TCP
// portion of cl_arq_controller::init() (arq_common.cc:1099-1170), then opt it
// into its own residue-free RNG. The two private load_configuration() PHY calls
// are issued by the caller (a cl_arq_controller member).
void sim2_setup_instance(MercuryInstance* m, int role, bool robust, int start_cfg,
                         unsigned int rng_seed)
{
	m->wire();
	m->audio.alloc();
	m->ts.enable_per_instance_rng(rng_seed);   // §10.1 residue-free stream

	// Mirror init()'s non-TCP setup.
	m->arq.fifo_buffer_tx.set_size(m->arq.default_configuration_ARQ.fifo_buffer_tx_size);
	m->arq.fifo_buffer_rx.set_size(m->arq.default_configuration_ARQ.fifo_buffer_rx_size);
	m->arq.fifo_buffer_backup.set_size(m->arq.default_configuration_ARQ.fifo_buffer_backup_size);
	m->arq.set_link_timeout(m->arq.default_configuration_ARQ.link_timeout);
	m->arq.robust_enabled     = robust ? YES : NO;
	m->arq.narrowband_enabled = NO;
	m->ts.narrowband_enabled  = NO;
	m->arq.bandwidth_mode     = BW_AUTO;
	m->arq.local_capability   = CAP_WB_CAPABLE;
	// Skip the NB HAIL probe (== -Q 0 benchmark mode): both peers controlled,
	// start directly in WB. Avoids the NB/WB negotiation mismatch where the
	// commander HAILs in NB, the responder replies in NB, then the commander
	// reverts to WB and can no longer match the NB reply (single-process-sim §10.5).
	m->arq.nb_probe_max       = 0;
	m->arq.gear_shift_on      = YES;
	m->arq.gear_shift_algorithm = m->arq.default_configuration_ARQ.gear_shift_algorithm;
	m->arq.current_configuration = CONFIG_NONE;
	if (robust) {
		m->arq.init_configuration = start_cfg;
		m->arq.data_configuration = start_cfg;
		m->arq.ack_configuration  = start_cfg;
	} else {
		m->arq.init_configuration = start_cfg;
		m->arq.data_configuration = start_cfg;
		m->arq.ack_configuration  = m->arq.default_configuration_ARQ.ack_configuration;
	}
	m->arq.last_data_viable_config =
		session_floor_anchor(m->arq.robust_enabled, m->arq.init_configuration);
	(void)role;
	// NOTE: the two PHY load_configuration() calls (private) are issued by the
	// caller test_sim_inproc_2 (a cl_arq_controller member) right after this.
}

// Point the audioio.c globals at this instance's rings (§10.3 swap). Single
// thread → the swap precedes every consumer (tx_transfer/rx_transfer/
// drain_playback_wait/the IDLE measure block) that runs during this instance's
// process_main. gf16ra reconcile (§10.2): re-apply this instance's suffix-FEC
// repfact (idempotent when unchanged) so the shared codec matches before drive.
void sim2_activate(MercuryInstance* m)
{
	capture_buffer    = m->audio.cap;
	playback_buffer   = m->audio.play;
	capture_prep_mutex = m->audio.mutex;
	// STEPPER-CORE REWRITE Phase b: tag the active instance's modulation so the
	// outer-stepper TX-wait seams (drain_playback_wait / ptt_busy_wait / pacing-floor)
	// pace OFDM data per-symbol but keep the legacy co-routine pump for the MFSK
	// handshake (arq_common.cc §10.4). No-op outside the outer stepper.
	arq_set_sim2_active_is_ofdm(is_ofdm_config(m->arq.current_configuration));
	// gf16ra reconcile: only meaningful if the suffix-FEC path is enabled; both
	// peers run the same repfact under the no-negotiation invariant, so this is
	// a no-op idempotent re-apply in the common case (FEC off → not configured).
	if (m->ts.ack_mfsk.suffix_fec_coded) {
		// Both peers run the same repfact under the no-negotiation invariant;
		// repfact 3 is the only production value set_suffix_fec uses. Re-applying
		// it is idempotent (gf16ra::init short-circuits when unchanged).
		gf16ra::configure(3);
		gf16ra::init();
	}
}

#if !defined(_WIN32)
// sim2_activate() copies the active instance's opaque pthread mutex into the
// process-global used by the production ARQ paths.  Once the two instance
// mutexes have been destroyed that copy is no longer usable.  Reinitialize the
// global from uninitialized storage and validate it before any later --test
// case can take the lock.  The defeat flag is test-only and default-off.
int sim2_reset_capture_prep_mutex()
{
	if (std::getenv("MERCURY_SIM2_CAPTURE_MUTEX_RESET_DEFEAT") != nullptr)
		return ECANCELED;

	int rc = pthread_mutex_init(&capture_prep_mutex, nullptr);
	if (rc != 0)
		return rc;
	rc = pthread_mutex_trylock(&capture_prep_mutex);
	if (rc != 0)
		return rc;
	return pthread_mutex_unlock(&capture_prep_mutex);
}
#endif

}  // namespace

// FULL-PATH REGRESSION (bigblock-whiten-align): capture of the last 2-instance run's
// delivery so test_sim_inproc_bigblock_fullpath() can assert without re-parsing stdout.
long cl_arq_controller::sim2_last_rx_have     = -1;
long cl_arq_controller::sim2_last_payload_len = -1;
bool cl_arq_controller::sim2_last_bytes_ok    = false;
// SIM_INPROC fix #1 (SIMFTR_ROOTCAUSE.md §7): last run's stalled/wedge flag.
bool cl_arq_controller::sim2_last_stalled     = false;
// GAP-2 LIVE-PATH (diag/livepath-sim): cw0-CRC gate decision tally (see arq.h).
long cl_arq_controller::sim2_gate_accepts = 0;
long cl_arq_controller::sim2_gate_rejects = 0;
long cl_arq_controller::sim2_tx_block_emits = 0;

int cl_arq_controller::test_sim_inproc_2()
{
	int failed = 0;
	auto check = [&](bool cond, const char* name) {
		printf("[TEST-SIM-2INST] %s: %s\n", cond ? "PASS" : "FAIL", name);
		if(!cond) failed++;
		fflush(stdout);
	};

	printf("[TEST-SIM-2INST] two-instance in-process lockstep stepper "
	       "(A=CMD, B=RSP, scalar-AWGN channel both directions, shared virtual "
	       "clock, NO device/threads/TCP/relay)\n");
	fflush(stdout);

	// --- Parameters (env-overridable for GATE-2 determinism re-runs) ---
	auto env_d = [](const char* k, double def)->double {
		const char* e = std::getenv(k); return (e && *e) ? atof(e) : def; };
	auto env_i = [](const char* k, long def)->long {
		const char* e = std::getenv(k); return (e && *e) ? atol(e) : def; };

	const double snr3k_db   = env_d("MERCURY_SIM2_SNR3K", 900.0);   // 900 = clean
	const unsigned seed     = (unsigned)env_i("MERCURY_SIM2_SEED", 12345);
	const int  start_cfg    = (int)env_i("MERCURY_SIM2_CFG", ROBUST_0);
	const bool robust       = env_i("MERCURY_SIM2_ROBUST", 1) != 0;

	// CONTROL-LOOP PROBE additions (wf-sim-controlloop) — all env-gated and
	// ADDITIVE; default values reproduce the legacy 19-byte smoke exactly so the
	// regression (MERCURY_SIM_2INST=1 with no extra env) is byte-identical.
	//   MERCURY_SIM2_PAYLOAD_BYTES : 0 = legacy short "HELLO" payload (default);
	//                                >0 = deterministic pseudo-random payload of
	//                                that many bytes (drives the gearshift climb).
	//   MERCURY_SIM2_OPT           : 0 = optimizer OFF (== --no-optimizer, pure
	//                                ladder gearshift, default); 1 = optimizer ON
	//                                (load the calibration table via
	//                                opt_load_rate_table(), honoring
	//                                MERCURY_RATE_TABLE).
	const long payload_bytes = env_i("MERCURY_SIM2_PAYLOAD_BYTES", 0);
	const bool opt_on        = env_i("MERCURY_SIM2_OPT", 0) != 0;

	// GAP-2 LIVE-PATH (diag/livepath-sim): MERCURY_SIM2_FORCE_SETCONFIG=<cfg> drives ONE
	// REAL config transition robust->cfg over the live wire AFTER CONNECT, using the
	// gearshift's OWN production mechanism (negotiated_configuration + add_message_control
	// (SET_CONFIG) + connection_status=TRANSMITTING_CONTROL — arq_commander.cc:3867/3899),
	// NOT a load_configuration hand-pin. This exercises the SET_CONFIG control frame + the
	// control-ACK turnaround (the path the 79207f7 TX-hold fix protects) that the PINNED
	// fullpath/multicw tests bypass. After the jump is issued the commander's gearshift is
	// turned OFF so it HOLDS the target rung (no further churn) and the payload flows as
	// real big-blocks. Default -1 = disabled (every existing arm is byte-identical).
	const long force_setconfig = env_i("MERCURY_SIM2_FORCE_SETCONFIG", -1);
	// IN-BAND DOWN-LADDER RESYNC repro (data-flow-inband-ondemote-zerobyte.md §6 /
	// --test-inband-down-resync). MERCURY_SIM2_INBAND_LAG=1 drives the EXACT HW 0-byte
	// path the directed test_inband_fallback structurally cannot reach: under
	// MERCURY_INBAND_RATE (the wrapper sets it), the COMMANDER (A) DEMOTES to a real
	// ROBUST rung via the production chokepoint while its announce CONFIG_TAG is
	// SUPPRESSED (inband_test_suppress_announce_tx) — so the RESPONDER (B) never hears
	// the tag, its current_configuration LAGS at the start OFDM rung, its primary OFDM
	// decode fails, and its production inband_try_down_ladder_on_decode_fail() must
	// resync from a PRIMARY-config-sized capture snapshot over a window that includes the
	// ROBUST rung. Before the Rank-1 snapshot-sizing fix the ROBUST frame overruns the
	// primary-sized snapshot -> EVERY trial-decode fails -> 0 bytes (the wedge). After the
	// fix the down-ladder decodes ROBUST, B adopts it, and the batch delivers
	// byte-faithful. Default 0 = disabled (every existing arm is byte-identical). The
	// demote target is MERCURY_SIM2_INBAND_DEMOTE_CFG (default ROBUST_0=100).
	const bool inband_lag = env_i("MERCURY_SIM2_INBAND_LAG", 0) != 0;
	const long inband_demote_cfg = env_i("MERCURY_SIM2_INBAND_DEMOTE_CFG", ROBUST_0);
	// FAIL-BEFORE A/B HOOK for the first-block-race harness fix (see the payload-stage block):
	// MERCURY_SIM2_DEFEAT_FIRSTBLOCK_FIX=1 reproduces the PRE-FIX artifact behavior so the SAME
	// binary shows fail-before (gate_ran==false). Production never sets it.
	const bool defeat_firstblock_fix = env_i("MERCURY_SIM2_DEFEAT_FIRSTBLOCK_FIX", 0) != 0;
	// STEPPER-CORE REWRITE (sim2-stepper-rewrite Phase a): MERCURY_SIM2_STEPPER selects the
	// outer-loop driver. "legacy" (default) = the existing per-half-step pump loop. "outer" =
	// the new symbol-pump driver that advances BOTH instances by ONE symbol in BOTH directions
	// per outer iteration (drain-before-tick), routing the top-level catch-up drain/feed through
	// the one-symbol helpers (sim_step_drain_one_symbol / sim_step_feed_one_symbol). Phase a does
	// NOT yet no-op drain_playback_wait (the pump still drains TX inside the blocking waits), so
	// "outer" must be byte-identical to "legacy" on the non-wedging paths (G2). Default legacy =
	// fully reversible; every existing arm is byte-identical when the flag is unset.
	const char* stepper_env = std::getenv("MERCURY_SIM2_STEPPER");
	const bool use_outer_stepper = (stepper_env != nullptr && strcmp(stepper_env, "outer") == 0);
	// Reset the cw0-CRC gate decision tally + TX-emit count for this run (read by the
	// live-path regression).
	sim2_gate_accepts = 0;
	sim2_gate_rejects = 0;
	sim2_tx_block_emits = 0;

	// Build the payload buffer. Legacy default (payload_bytes==0): the original
	// 19-byte greeting (verbatim, so the GATE-2 regression is unchanged). Large
	// payload: a deterministic LCG byte stream keyed on the run seed so GATE-2
	// (same seed twice -> byte-identical) holds for the data-heavy run too.
	static const char* legacy_payload = "MERCURY-2INST-HELLO";
	std::vector<char> payload_vec;
	const char* payload;
	int payload_len;
	if (payload_bytes <= 0) {
		payload     = legacy_payload;
		payload_len = (int)strlen(legacy_payload);
	} else {
		payload_vec.resize((size_t)payload_bytes);
		uint32_t lcg = 0x9E3779B9u ^ seed;   // deterministic, seed-keyed
		for (long i = 0; i < payload_bytes; i++) {
			lcg = lcg * 1664525u + 1013904223u;
			payload_vec[(size_t)i] = (char)((lcg >> 24) & 0xFF);
		}
		payload     = payload_vec.data();
		payload_len = (int)payload_bytes;
	}

	// max_iters: keep the legacy 60000 default for the short payload; scale the
	// default headroom up for a large payload (the data-heavy climb needs far
	// more ticks). Still env-overridable via MERCURY_SIM2_MAXITERS.
	const long default_max_iters = (payload_bytes > 0) ? 4000000 : 60000;
	const long max_iters    = env_i("MERCURY_SIM2_MAXITERS", default_max_iters);

	printf("[TEST-SIM-2INST] params: snr3k=%.1f seed=%u max_iters=%ld start_cfg=%d "
	       "robust=%d payload_len=%d opt=%s\n", snr3k_db, seed, max_iters, start_cfg,
	       (int)robust, payload_len, opt_on ? "ON" : "OFF");
	fflush(stdout);

	// --- Construct two instances + two channels (one per direction) ---
	MercuryInstance* A = new MercuryInstance();  A->tag = "A/CMD";
	MercuryInstance* B = new MercuryInstance();  B->tag = "B/RSP";
	// main.cc bridges an explicit SIM_INPROC --gi through this test-only seam.
	// Apply it before sim2_setup_instance() performs either peer's first PHY
	// load.  The variable is absent unless the user supplied --gi, preserving
	// the legacy simulator geometry and every existing regression by default.
	const double sim2_gi_ms = env_d("MERCURY_SIM2_GI_MS", -1.0);
	if (sim2_gi_ms >= 1.0 && sim2_gi_ms <= 8.0) {
		const int sim2_ngi = (int)(sim2_gi_ms * 12.0 + 0.5);
		A->ts.default_configurations_telecom_system.ofdm_gi =
			(float)sim2_ngi / 256.0f;
		B->ts.default_configurations_telecom_system.ofdm_gi =
			(float)sim2_ngi / 256.0f;
		printf("[TEST-SIM-2INST] explicit GI: %.2f ms (Ngi=%d) on both peers\n",
		       sim2_gi_ms, sim2_ngi);
		fflush(stdout);
	}
	sim2_setup_instance(A, COMMANDER, robust, start_cfg, seed ^ 0xA5A5u);
	sim2_setup_instance(B, RESPONDER, robust, start_cfg, seed ^ 0x5A5Au);
	// [WBDIRECT-REGRESSION] connect-fast pin-RESEAT loopback regression (cross-layer
	// data-flow audit of the connect-fast restore paths). MERCURY_SIM2_CONNECT_FAST=<cfg>
	// (default -1 = OFF, every existing arm byte-identical) escalates the CONNECT
	// handshake to <cfg> (e.g. CONFIG_0) and remembers start_cfg as the pin/fallback —
	// the SAME latch init() installs when MERCURY_CONNECT_FAST_CONFIG is set
	// (arq_common.cc, the connect_fast_config!=CONFIG_NONE block). On a successful fast
	// connect the commander (and responder) MUST reseat the LIVE PHY off the fast config
	// onto the pin; the pre-fix code reseated only the intent vars and left
	// current_configuration at the fast config, so forward DATA was keyed at CONFIG_0
	// while the batch/SACK geometry expected the pin (the WB-direct start stall: the
	// session strands at ~30 delivered bytes). The fail-before hook
	// MERCURY_CONNECT_FAST_DEFEAT_RESEAT=1 (read at the production reseat sites in
	// arq_commander.cc / arq_responder.cc) skips the reseat so the SAME binary
	// reproduces that strand. Configured here, BEFORE the PHY bring-up below, so the
	// handshake PHY loads at the fast config. Use with the gearshift PINNED off
	// (MERCURY_SIM2_PIN=1) — the connect-fast success reseat lives in the gear-shift-off
	// branch of process_control_commander().
	const long connect_fast_cfg = env_i("MERCURY_SIM2_CONNECT_FAST", -1);
	if (connect_fast_cfg >= 0) {
		MercuryInstance* cf_peers[2] = { A, B };
		for (int pi = 0; pi < 2; pi++) {
			cl_arq_controller& a = cf_peers[pi]->arq;
			a.connect_fast_config          = (int)connect_fast_cfg;
			a.connect_fast_budget_ms       = (int)env_i("MERCURY_SIM2_CONNECT_FAST_BUDGET_MS", 60000);
			a.connect_fast_fallback_robust = a.robust_enabled;
			a.connect_fast_fallback_config = a.init_configuration;   // the TRUE pin (== start_cfg)
			a.connect_fast_active          = true;
			a.robust_enabled               = NO;
			a.init_configuration           = (int)connect_fast_cfg;
			a.data_configuration           = (int)connect_fast_cfg;
			a.ack_configuration            = (int)connect_fast_cfg;
			a.last_data_viable_config      =
				session_floor_anchor(a.robust_enabled, a.init_configuration);
		}
		printf("[TEST-SIM-2INST] CONNECT-FAST mode: handshake escalated to CONFIG_%ld, "
		       "pin/fallback=CONFIG_%d (budget=%d ms, defeat_reseat=%s)\n",
		       connect_fast_cfg, A->arq.connect_fast_fallback_config,
		       A->arq.connect_fast_budget_ms,
		       (std::getenv("MERCURY_CONNECT_FAST_DEFEAT_RESEAT") &&
		        atoi(std::getenv("MERCURY_CONNECT_FAST_DEFEAT_RESEAT")) != 0) ? "YES" : "no");
		fflush(stdout);
	}
	// PHY bring-up (no TCP) — mirrors init()'s two load_configuration calls. Done
	// here (not in the free helper) because load_configuration is a private member;
	// a cl_arq_controller member fn can call it on ANY instance.
	A->arq.load_configuration(A->arq.ack_configuration,  FULL,                 NO);
	A->arq.load_configuration(A->arq.data_configuration, PHYSICAL_LAYER_ONLY,  YES);
	B->arq.load_configuration(B->arq.ack_configuration,  FULL,                 NO);
	B->arq.load_configuration(B->arq.data_configuration, PHYSICAL_LAYER_ONLY,  YES);
	bool okA = A->ts.data_container.Nofdm * A->ts.data_container.interpolation_rate > 0;
	bool okB = B->ts.data_container.Nofdm * B->ts.data_container.interpolation_rate > 0;
	check(okA && okB, "S1 both instances brought up (PHY + buffers, no TCP)");

	// --- CONTROL-LOOP PROBE: optimizer toggle (wf-sim-controlloop) ---
	// OFF (default): set_optimizer_disabled(true) on both instances. This is the
	//   exact same state main.cc:2401 installs for --no-optimizer: opt_load_rate_table()
	//   no-ops and opt_evaluate_batch_end() hard-short-circuits, so config selection
	//   is owned ENTIRELY by the pure-ladder gearshift (gear_shift_on stays YES).
	// ON: set_optimizer_disabled(false) and load the calibration table on the
	//   COMMANDER (A) — the optimizer only acts on the commander (the role!=COMMANDER
	//   gate in opt_evaluate_batch_end). Loaded on B too for symmetry/harmlessness
	//   (B never evaluates). opt_load_rate_table() honors MERCURY_RATE_TABLE, which
	//   the probe points at effective_rate_table.v13.json.
	// Diagnostic knob: MERCURY_SIM2_PIN=1 disables the gearshift on both
	// instances so the config is HELD at start_cfg. Used to isolate sustained
	// multi-batch data flow from config-switch effects. Additive/env-gated.
	const bool pin_cfg = env_i("MERCURY_SIM2_PIN", 0) != 0;
	if (pin_cfg) {
		A->arq.gear_shift_on = NO;
		B->arq.gear_shift_on = NO;
		printf("[TEST-SIM-2INST] gearshift PINNED off (config held at start_cfg=%d)\n", start_cfg);
		fflush(stdout);
	}

	// GAP-2 LIVE-PATH (diag/livepath-sim): when the harness will force-fire ONE real
	// SET_CONFIG jump (MERCURY_SIM2_FORCE_SETCONFIG>=0), disable the gearshift on BOTH
	// peers up front so the harness is the SOLE config driver — A's own FRAME-UP climb
	// must not race a competing SET_CONFIG (it would grab messages_control + target a
	// +1 rung instead of the requested jump, leaving the harness fire blocked on the
	// messages_control.status==FREE guard and the test non-deterministic). The handshake
	// (HAIL/CONNECT) does not depend on the gearshift, so CONNECT still completes; the
	// harness fire then drives the exact robust->target transition over the live wire.
	if (force_setconfig >= 0 && !pin_cfg && !defeat_firstblock_fix) {
		A->arq.gear_shift_on = NO;
		B->arq.gear_shift_on = NO;
		printf("[TEST-SIM-2INST] FORCE_SETCONFIG=%ld: gearshift off up front (harness is the "
		       "sole config driver; CONNECT unaffected)\n", force_setconfig);
		fflush(stdout);
	}

	// IN-BAND DOWN-LADDER RESYNC repro setup. The harness is the SOLE config driver
	// (gearshift off both peers) so A's demote-to-ROBUST is the only transition, and the
	// COMMANDER's announce CONFIG_TAG is dropped so B never follows -> B's production
	// down-ladder is forced. CONNECT (the MFSK handshake) is gearshift-independent so it
	// still completes. inband_test_suppress_announce_tx only affects emit_config_tag_passband
	// (it skips the tx_transfer); B receives no announce and lags. The recover leg
	// (un-suppress so A re-announces once B has resynced) is driven inside the loop.
	if (inband_lag) {
		A->arq.gear_shift_on = NO;
		B->arq.gear_shift_on = NO;
		A->arq.inband_test_suppress_announce_tx = true;   // CMD->RSP announce dropped
		printf("[TEST-SIM-2INST] INBAND_LAG=1: gearshift off both peers; A announce SUPPRESSED "
		       "(B will lag -> production down-ladder resync). demote_cfg=%ld\n", inband_demote_cfg);
		fflush(stdout);
	}

	A->arq.set_optimizer_disabled(!opt_on);
	B->arq.set_optimizer_disabled(!opt_on);

	// LEVER P: preamble amortization A/B knob. MERCURY_SIM2_PREAMBLE_AMORT=1
	// enables the variable per-frame OFDM preamble on BOTH instances. Default 0
	// (legacy full preamble, byte-identical baseline). See
	// fact-documents/data-flow-preamble-amortization.md.
	{
		const bool amort_on = env_i("MERCURY_SIM2_PREAMBLE_AMORT", 0) != 0;
		A->ts.preamble_amortization_enabled = amort_on;
		B->ts.preamble_amortization_enabled = amort_on;
		printf("[TEST-SIM-2INST] preamble amortization %s\n", amort_on ? "ON" : "OFF");
		fflush(stdout);
	}

	if (opt_on) {
		printf("[TEST-SIM-2INST] optimizer ON: loading calibration table (A=CMD)\n");
		fflush(stdout);
		A->arq.opt_load_rate_table();
		B->arq.opt_load_rate_table();
	} else {
		printf("[TEST-SIM-2INST] optimizer OFF (== --no-optimizer): pure-ladder gearshift owns config\n");
		fflush(stdout);
	}
	// test_sim_inproc_2 is a cl_arq_controller member -> may read A->arq's private
	// optimizer_disabled directly (same class).
	check(A->arq.optimizer_disabled == !opt_on,
	      opt_on ? "S1b optimizer ENABLED (table loaded, ON arm)"
	             : "S1b optimizer DISABLED (pure ladder, OFF arm)");

	cl_sim_awgn ch_a2b(((uint64_t)seed << 1) | 1u, snr3k_db);   // A->B
	cl_sim_awgn ch_b2a(((uint64_t)seed << 1) | 0u, snr3k_db);   // B->A

	// --- Engage the SHARED virtual clock + the TCP skip gate ---
	sim_clock_set_enabled(1);
	arq_set_sim_inproc_skip_tcp(true);
	check(sim_clock_enabled() != 0, "S2 shared virtual clock engaged");

	// --- Step-pump context + scratch (>= 2 symbols: move buffer + prep buffer;
	//     allocate 3x sp_max for headroom). The pump holds BOTH instances + BOTH
	//     channels so it can co-routine-drive the peer during the active wait. ---
	int sp_max = A->ts.data_container.Nofdm * A->ts.data_container.interpolation_rate;
	int sp_b   = B->ts.data_container.Nofdm * B->ts.data_container.interpolation_rate;
	if (sp_b > sp_max) sp_max = sp_b;
	SimInproc2Ctx pump;
	pump.sp_max  = sp_max;
	pump.scratch = (double*)malloc((size_t)sp_max * sizeof(double) * 3);
	pump.ch_tx2rx = &ch_a2b;   // initial (overwritten each half-step)
	pump.ch_rx2tx = &ch_b2a;
	// Per-direction WIRE rings (§10.5): a2b (A->B) and b2a (B->A). Sized like the
	// audio rings so a full frame fits in flight.
	uint8_t* wmem_a2b = (uint8_t*)malloc(AUDIO_PAYLOAD_BUFFER_SIZE);
	uint8_t* wmem_b2a = (uint8_t*)malloc(AUDIO_PAYLOAD_BUFFER_SIZE);
	cbuf_handle_t wire_a2b = circular_buf_init(wmem_a2b, AUDIO_PAYLOAD_BUFFER_SIZE);
	cbuf_handle_t wire_b2a = circular_buf_init(wmem_b2a, AUDIO_PAYLOAD_BUFFER_SIZE);
	clear_buffer(wire_a2b); clear_buffer(wire_b2a);
	pump.wire_t2r = wire_a2b;   // initial (overwritten each half-step)
	pump.wire_r2t = wire_b2a;
	arq_set_sim_inproc_pump(sim_inproc_pump_2, &pump);
	// STEPPER-CORE REWRITE Phase b: install the drain-ONLY pump (egress big-block play -> wire
	// without RX burst) + arm the OUTER-stepper gate so the OFDM-data drain_playback_wait egresses
	// to the wire (and ptt_busy_wait clock-only), with the outer loop the SOLE RX-feed driver. Only
	// armed when MERCURY_SIM2_STEPPER=outer; legacy keeps the pump-driven blocking waits.
	arq_set_sim_inproc_drain_only_pump(sim_inproc_drain_to_wire_only);
	arq_set_sim_inproc_outer_stepper(use_outer_stepper);
	check(pump.scratch != nullptr && wire_a2b != nullptr && wire_b2a != nullptr,
	      "S3 step-pump + wires installed");

	// --- Drive the handshake via the REAL process_user_command (no TCP) ---
	// B first: MYCALL + LISTEN ON (loads init_configuration, RESPONDER/LISTENING).
	sim2_activate(B);
	B->arq.process_user_command("MYCALL TESTB");
	B->arq.process_user_command("LISTEN ON");
	// A: MYCALL + CONNECT TESTA TESTB (COMMANDER/CONNECTING).
	sim2_activate(A);
	A->arq.process_user_command("MYCALL TESTA");
	A->arq.process_user_command("CONNECT TESTA TESTB");
	check(B->arq.link_status == LISTENING, "S4 B is LISTENING after LISTEN ON");
	check(A->arq.link_status == CONNECTING, "S5 A is CONNECTING after CONNECT");

	// --- Stage the test payload into A's TX FIFO (== the TCP data socket push) ---
	// GAP-2 LIVE-PATH (diag/livepath-sim first-block-race fix): when the harness will
	// force-fire a SET_CONFIG jump, DEFER the payload push until AFTER the transition
	// fully applies on A (current_configuration == target). Pushing the payload up front
	// makes A start sending ROBUST_0 data frames the instant it CONNECTs; in this
	// single-process sim those frames are drained off the wire into B's CAPTURE RING in
	// the SAME iteration the harness fires the SET_CONFIG, so B's decode snapshots the
	// stale in-flight ROBUST_0 DATA frame (byte0=0x10) instead of the SET_CONFIG control
	// frame (byte0=0x3B) and never runs its SET_CONFIG handler — A then mistakes B's
	// data-SACK for the control-ACK, switches to CFG16 while B stays on ROBUST_0, and the
	// post-switch big-block is undecodable ("First-batch ACK miss" -> BREAK). Production
	// never sends data BEFORE the SET_CONFIG (it issues SET_CONFIG at a drained batch
	// boundary, arq_commander.cc:3840), so deferring the push restores that invariant:
	// CONNECT completes with an EMPTY pipeline, the SET_CONFIG is the only frame on the
	// wire, B receives it in sequence, and the payload then flows purely as CFG16
	// big-blocks. On every non-force-setconfig arm the push is up front (byte-identical).
	//
	// FAIL-BEFORE A/B HOOK: MERCURY_SIM2_DEFEAT_FIRSTBLOCK_FIX=1 reproduces the PRE-FIX
	// (artifact) behavior — payload pushed up front + gearshift NOT pre-disabled + the
	// SET_CONFIG fired without the wire-quiescent gate — so the SAME binary demonstrates the
	// fail-before (B decodes the stale in-flight ROBUST_0 DATA frame, never runs its
	// SET_CONFIG handler, gate never runs -> gate_ran==false). Production never sets it; the
	// live-path test leaves it off so the regression locks in the pass-after.
	bool payload_pushed = false;
	if (force_setconfig < 0 || defeat_firstblock_fix) {
		A->arq.fifo_buffer_tx.push((char*)payload, payload_len);
		payload_pushed = true;
	}

	// --- Lockstep step loop (§3): A.process_main -> ch_a2b -> B.cap,
	//     B.process_main -> ch_b2a -> A.cap, shared clock advanced by the pump. ---
	bool connected_seen = false;
	long iters = 0;
	int  rx_have = 0;
	// RX buffer sized to the full payload (+1 NUL) so the large-payload arm can
	// hold tens of KB; the legacy 19-byte arm fits trivially.
	std::vector<char> rx_vec((size_t)payload_len + 1, 0);
	char* rx_buf = rx_vec.data();
	const int rx_cap = payload_len + 1;

	// --- CONTROL-LOOP PROBE: config switch-sequence tracker (wf-sim-controlloop).
	//     Records every change of the COMMANDER's live current_configuration as
	//     (iter, cfg). This is the climb path the probe reports: does OFF climb to
	//     and HOLD an optimal config, while ON over-climbs past the cliff and
	//     collapses back to ROBUST_0? Tracking the LIVE field (not load_configuration)
	//     catches gearshift AND optimizer-driven switches uniformly. ---
	std::vector<std::pair<long,int>> cfg_seq;
	int last_cfg = A->arq.current_configuration;
	cfg_seq.push_back({-1, last_cfg});   // initial config at loop entry
	int max_cfg_reached = last_cfg;      // highest OFDM cfg the climb touched
	long collapse_iter  = -1;            // first iter cfg fell back to a ROBUST_x

	const bool dbg = (getenv("MERCURY_SIM2_DBG") != nullptr);
	// Stall cutoff (large-payload arm only): if no new RX byte arrives for this
	// many iters, terminate (the over-climb-collapse arm can otherwise crawl for
	// millions of ticks). 0/legacy arm: disabled (legacy break-on-deliver only).
	const long stall_cutoff = env_i("MERCURY_SIM2_STALL_ITERS",
	                                 (payload_bytes > 0) ? 200000 : 0);
	long last_progress_iter = 0;
	int  prev_rx_have = 0;
	uint64_t t0 = sim_clock_now_samples();
	bool stalled = false;
	// GAP-2 LIVE-PATH: one-shot guard for the forced SET_CONFIG jump (fired at most once,
	// once the commander is CONNECTED and its control channel is idle).
	bool force_setconfig_done = false;
	// IN-BAND DOWN-LADDER RESYNC: one-shot guard for the forced demote-to-ROBUST + a
	// flag for the recover leg (un-suppress A's announce once B has resynced, i.e. the
	// first byte delivered means B adopted the demoted config via the down-ladder).
	bool inband_lag_demote_done = false;
	bool inband_lag_recovered   = false;
	// GAP-2 LIVE-PATH: iter at which the stop-after-tx-emits threshold was first met (the
	// grace window is measured from here). -1 = not yet armed. Loop-local (reset per run).
	long emit_break_arm_iter = -1;

	// --- SIM_INPROC post-delivery spin-abort wiring (SIMFTR_ROOTCAUSE.md §7 fix #1). ---
	// Arm the pump's delivery probe so it can unwedge the commander's post-transfer
	// keepalive spin from INSIDE process_main (the outer delivery check at the loop
	// bottom is unreachable while A is wedged). The pump compares (rx_have already
	// drained here) + (bytes currently in B's fifo_buffer_rx) against payload_len and
	// sets the SIM_INPROC abort flag the instant B holds the whole payload.
	//   MERCURY_SIM2_DEFEAT_SIMFTR_FIX=1 : fail-before A/B hook — reproduce the PRE-FIX
	//     wedge on the SAME binary (skip arming the genuine abort). A pump-call watchdog
	//     then force-terminates the wedged run and marks it stalled so the test reports
	//     the defect as a FAIL. Production never sets it.
	const bool defeat_simftr_fix = env_i("MERCURY_SIM2_DEFEAT_SIMFTR_FIX", 0) != 0;
	arq_set_sim_inproc_deliver_done(false);   // reset for this run
	pump.deliver_rx       = &B->arq;
	pump.deliver_target   = payload_len;
	pump.deliver_drained  = &rx_have;
	pump.deliver_defeat   = defeat_simftr_fix;
	pump.deliver_stalled  = false;
	pump.deliver_watchdog_anchor = -1;
	// Watchdog only matters for the fail-before arm (the real fix aborts on the genuine
	// predicate at the delivery instant). It is a DELIVERY-ANCHORED count: pump calls AFTER
	// the responder holds the full payload, so it can only catch a genuine post-delivery
	// spin (the wedge), never a transfer still in flight. Default 80 (well into the first
	// post-transfer idle wait, before the expensive keepalive TX). Env-overridable via
	// MERCURY_SIM2_SIMFTR_WATCHDOG.
	pump.deliver_watchdog_calls = env_i("MERCURY_SIM2_SIMFTR_WATCHDOG", 80);

	// C0-a (fact-doc §10.8): iter at which the FIRST block was carved, for the
	// STOP_AFTER_FIRST_BLOCK grace window. The clean big-block now DELIVERS via the
	// responder ACK-GATE on a LATER tick (not in-carve), so the stop-after-first-block
	// break must give the RSP a bounded grace to run that ACK-GATE before terminating —
	// otherwise it stops the instant the carve fires and reads rx_have=0 (the clean arms
	// would never see their delivery). -1 = not yet armed. Loop-local (reset per run).
	long first_block_break_arm_iter = -1;
	for (; iters < max_iters; iters++)
	{
		if (dbg && (iters % 200 == 0)) {
			printf("[SIM2-DBG] it=%ld A.link=%d A.conn=%d B.link=%d B.conn=%d "
			       "B.cap=%zu B.ftr=%d B.dr=%d A.cap=%zu A.play=%zu B.play=%zu hail_det=%d\n",
			       iters, A->arq.link_status, A->arq.connection_status,
			       B->arq.link_status, B->arq.connection_status,
			       size_buffer(B->audio.cap),
			       (int)B->ts.data_container.frames_to_read,
			       (int)B->ts.data_container.data_ready,
			       size_buffer(A->audio.cap), size_buffer(A->audio.play),
			       size_buffer(B->audio.play), B->arq.hail_detected);
			fflush(stdout);
		}
		if (use_outer_stepper)
		{
			// ================= OUTER-LOOP DRIVER (Phase a) =================
			// One outer iteration advances BOTH instances by symbols in BOTH
			// directions, top-level catch-up routed through the one-symbol helpers
			// (sim_step_drain_one_symbol / sim_step_feed_one_symbol). PHASE a: the
			// pump still drains TX inside the blocking waits (drain_playback_wait is
			// NOT yet a no-op — that is Phase b), so this loop performs the SAME
			// top-level catch-up the legacy body does (drain residual play -> wire,
			// feed wire -> cap), just one symbol at a time. The fixed A-before-B
			// service order + the per-symbol quantum preserve determinism (INV-1).
			// Equivalence to legacy: where legacy drains the WHOLE play buffer and
			// delivers the WHOLE wire in one catch-up call, this drains/feeds the
			// same residual one symbol per inner step until the rings are empty.

			// PHASE b — drain GRANULARITY is the headline of the rewrite. On the OFDM DATA
			// path the TX big-block was QUEUED by send_batch (drain_playback_wait no-op'd) and
			// must be drained ONE SYMBOL PER OUTER ITER so (a) OFDM decode sees the §8 per-symbol
			// cadence and (b) the CMD's ACK poll (process_messages_rx_acks_data) runs BETWEEN
			// symbols within the receiving_timeout window (a whole-buffer drain jumps the virtual
			// clock ~1.65 s/block in one iter BEFORE the CMD polls -> polls=0 -> ACK timeout ->
			// nAcked_data=0 -> re-send bsi=0 forever). On the ROBUST/MFSK HANDSHAKE path the
			// legacy co-routine pump already drained+delivered everything INTRA-process_main, so
			// the residual play is empty and the whole-buffer catch-up is a harmless no-op — keep
			// it (G2 byte-identical). The per-iter quantum is therefore: OFDM active -> ONE
			// symbol; else -> drain the residual (legacy catch-up). bigblock_one_symbol selects it.
			const bool ofdm_phase = is_ofdm_config(A->arq.current_configuration)
			                     || is_ofdm_config(B->arq.current_configuration);

			// --- A's half-step (tx=A, rx=B). The pump (in A's blocking waits) uses
			//     these ctx fields, so set them exactly as the legacy body did. ---
			pump.tx = A; pump.rx = B;
			pump.ch_tx2rx = &ch_a2b; pump.ch_rx2tx = &ch_b2a;
			pump.wire_t2r = wire_a2b; pump.wire_r2t = wire_b2a;
			sim2_activate(A);
			A->arq.process_main();
			// (1) DRAIN A.play -> ch_a2b -> wire_a2b. OFDM: exactly ONE symbol/iter (or one
			//     idle-fill if empty) so the clock advances one symbol at a time. ROBUST/MFSK:
			//     whole-buffer catch-up (the pump already drained it -> typically a no-op).
			{
				cl_data_container* sdc = &A->ts.data_container;
				size_t sp_bytes = (size_t)(sdc->Nofdm * sdc->interpolation_rate) * sizeof(double);
				if (ofdm_phase) {
					sim_step_drain_one_symbol(A, &ch_a2b, wire_a2b, &pump);   // one symbol/iter
				} else if (sp_bytes > 0 && size_buffer(A->audio.play) >= sp_bytes) {
					while (size_buffer(A->audio.play) >= sp_bytes &&
					       circular_buf_free_size(wire_a2b) >= sp_bytes)
						sim_step_drain_one_symbol(A, &ch_a2b, wire_a2b, &pump);
				} else {
					sim_step_drain_one_symbol(A, &ch_a2b, wire_a2b, &pump);   // idle-fill tick
				}
			}
			// (2) FEED wire_a2b -> B.cap + per-frame decode drive. OFDM: ONE symbol/iter (so the
			//     ring never over-advances a frame boundary before the decode fires — §8/INV-A).
			//     ROBUST/MFSK: whole wire (pump already delivered -> typically a no-op).
			{
				cl_data_container* ddc = &B->ts.data_container;
				size_t sp_bytes = (size_t)(ddc->Nofdm * ddc->interpolation_rate) * sizeof(double);
				if (ofdm_phase) {
					if (sp_bytes > 0 && size_buffer(wire_a2b) >= sp_bytes &&
					    circular_buf_free_size(B->audio.cap) >= sp_bytes)
						sim_step_feed_one_symbol(B, wire_a2b, &pump, /*drive_decode=*/true);
				} else {
					while (sp_bytes > 0 && size_buffer(wire_a2b) >= sp_bytes &&
					       circular_buf_free_size(B->audio.cap) >= sp_bytes)
						sim_step_feed_one_symbol(B, wire_a2b, &pump, /*drive_decode=*/true);
				}
			}
			// Deliver any reply waiting in b2a into A if A is now idle (legacy parity). OFDM: one
			//     symbol/iter; ROBUST/MFSK: whole wire.
			if (size_buffer(A->audio.play) == 0)
			{
				cl_data_container* ddc = &A->ts.data_container;
				size_t sp_bytes = (size_t)(ddc->Nofdm * ddc->interpolation_rate) * sizeof(double);
				if (ofdm_phase) {
					if (sp_bytes > 0 && size_buffer(wire_b2a) >= sp_bytes &&
					    circular_buf_free_size(A->audio.cap) >= sp_bytes)
						sim_step_feed_one_symbol(A, wire_b2a, &pump, /*drive_decode=*/false);
				} else {
					while (sp_bytes > 0 && size_buffer(wire_b2a) >= sp_bytes &&
					       circular_buf_free_size(A->audio.cap) >= sp_bytes)
						sim_step_feed_one_symbol(A, wire_b2a, &pump, /*drive_decode=*/false);
				}
			}

			// --- B's half-step (tx=B, rx=A; symmetric). ---
			pump.tx = B; pump.rx = A;
			pump.ch_tx2rx = &ch_b2a; pump.ch_rx2tx = &ch_a2b;
			pump.wire_t2r = wire_b2a; pump.wire_r2t = wire_a2b;
			sim2_activate(B);
			B->arq.process_main();
			{
				cl_data_container* sdc = &B->ts.data_container;
				size_t sp_bytes = (size_t)(sdc->Nofdm * sdc->interpolation_rate) * sizeof(double);
				if (ofdm_phase) {
					sim_step_drain_one_symbol(B, &ch_b2a, wire_b2a, &pump);   // one symbol/iter
				} else if (sp_bytes > 0 && size_buffer(B->audio.play) >= sp_bytes) {
					while (size_buffer(B->audio.play) >= sp_bytes &&
					       circular_buf_free_size(wire_b2a) >= sp_bytes)
						sim_step_drain_one_symbol(B, &ch_b2a, wire_b2a, &pump);
				} else {
					sim_step_drain_one_symbol(B, &ch_b2a, wire_b2a, &pump);   // idle-fill tick
				}
			}
			{
				cl_data_container* ddc = &A->ts.data_container;
				size_t sp_bytes = (size_t)(ddc->Nofdm * ddc->interpolation_rate) * sizeof(double);
				if (ofdm_phase) {
					if (sp_bytes > 0 && size_buffer(wire_b2a) >= sp_bytes &&
					    circular_buf_free_size(A->audio.cap) >= sp_bytes)
						sim_step_feed_one_symbol(A, wire_b2a, &pump, /*drive_decode=*/false);
				} else {
					while (sp_bytes > 0 && size_buffer(wire_b2a) >= sp_bytes &&
					       circular_buf_free_size(A->audio.cap) >= sp_bytes)
						sim_step_feed_one_symbol(A, wire_b2a, &pump, /*drive_decode=*/false);
				}
			}
			if (size_buffer(B->audio.play) == 0)
			{
				cl_data_container* ddc = &B->ts.data_container;
				size_t sp_bytes = (size_t)(ddc->Nofdm * ddc->interpolation_rate) * sizeof(double);
				if (ofdm_phase) {
					if (sp_bytes > 0 && size_buffer(wire_a2b) >= sp_bytes &&
					    circular_buf_free_size(B->audio.cap) >= sp_bytes)
						sim_step_feed_one_symbol(B, wire_a2b, &pump, /*drive_decode=*/true);
				} else {
					while (sp_bytes > 0 && size_buffer(wire_a2b) >= sp_bytes &&
					       circular_buf_free_size(B->audio.cap) >= sp_bytes)
						sim_step_feed_one_symbol(B, wire_a2b, &pump, /*drive_decode=*/true);
				}
			}

			// PHASE b — POST-TX TIMER RX-CONSUMPTION ALIGNMENT (production-faithful). Under the
			// OFDM-data egress (§10.3) the big-block is drained play -> the in-flight WIRE inside
			// send_batch (clock +~1.65 s) and the RECEIVING_ACKS_DATA / receiving_timer.start()
			// fires immediately after — but the RSP has NOT yet CONSUMED the block (the outer loop
			// feeds wire -> RX one symbol per iter over the following iters). In PRODUCTION the
			// blocking drain holds until the DAC drains AND the RSP consumes concurrently, so the
			// ACK window measures from the moment the last sample reaches the RSP. Mirror that:
			// while an instance is in RECEIVING_ACKS_DATA and its OUTGOING wire still holds the
			// block (the RSP has not consumed it all), RESTART that instance's receiving_timer.
			// Once the wire empties (block fully fed to the RSP) the timer free-runs and the
			// window covers only the genuine ACK round-trip — exactly production. OFDM phase only;
			// idempotent, deterministic (A drains -> wire_a2b, B drains -> wire_b2a).
			if (ofdm_phase) {
				if (A->arq.connection_status == RECEIVING_ACKS_DATA &&
				    (size_buffer(A->audio.play) > 0 || size_buffer(wire_a2b) > 0))
					A->arq.receiving_timer.start();
				if (B->arq.connection_status == RECEIVING_ACKS_DATA &&
				    (size_buffer(B->audio.play) > 0 || size_buffer(wire_b2a) > 0))
					B->arq.receiving_timer.start();
			}
		}
		else
		{
		// ================= LEGACY per-half-step pump loop (default) =================
		// --- A's half-step (tx=A, rx=B; the pump co-routine-drives B in A's waits,
		//     and delivers B's reply into A during A's listen waits — §10.5). ---
		pump.tx = A; pump.rx = B;
		pump.ch_tx2rx = &ch_a2b; pump.ch_rx2tx = &ch_b2a;
		pump.wire_t2r = wire_a2b; pump.wire_r2t = wire_b2a;
		sim2_activate(A);
		A->arq.process_main();
		// Catch any A TX queued without hitting a wait this tick: drain to wire +
		// deliver to B. Also deliver any reply waiting in b2a into A if A is now
		// idle (A.play empty post-process_main).
		sim2_drain_to_wire(A, &ch_a2b, wire_a2b, &pump);
		// Top-level "catch-up" delivery into B (the data receiver). drive_decode=true:
		// fires the per-frame OFDM decode (gated to CONNECTED+OFDM) for a data frame
		// that finished feeding via the top-level drain rather than mid-pump. The
		// A-target "reply into A" delivery (B's ACKs) is left WITHOUT a decode-drive:
		// A receives ACK/control here, the jitter fix is for OFDM DATA, and keeping A's
		// decode on the legacy once-per-half-step cadence avoids perturbing A's
		// commander-side gearshift loop (A was observed to climb 15->16 when driven).
		sim2_deliver_from_wire(B, wire_a2b, &pump, /*drive_decode=*/true);
		if (size_buffer(A->audio.play) == 0)
			sim2_deliver_from_wire(A, wire_b2a, &pump);

		// --- B's half-step (tx=B, rx=A; symmetric). ---
		pump.tx = B; pump.rx = A;
		pump.ch_tx2rx = &ch_b2a; pump.ch_rx2tx = &ch_a2b;
		pump.wire_t2r = wire_b2a; pump.wire_r2t = wire_a2b;
		sim2_activate(B);
		B->arq.process_main();
		sim2_drain_to_wire(B, &ch_b2a, wire_b2a, &pump);
		sim2_deliver_from_wire(A, wire_b2a, &pump);
		if (size_buffer(B->audio.play) == 0)
			sim2_deliver_from_wire(B, wire_a2b, &pump, /*drive_decode=*/true);
		}

		if (!connected_seen &&
		    (A->arq.link_status == CONNECTED || B->arq.link_status == CONNECTED))
			connected_seen = true;

		// --- GAP-2 LIVE-PATH: fire ONE real SET_CONFIG jump robust->target once both
		//     peers are CONNECTED and the commander's control channel is idle. This is
		//     the gearshift's own production mechanism (NOT a load_configuration pin):
		//     set negotiated_configuration, queue a real SET_CONFIG control frame, enter
		//     TRANSMITTING_CONTROL. The peers run the real control-ACK turnaround (the
		//     79207f7-protected path), apply the config on both sides, then the queued
		//     payload flows as real big-blocks. After issuing, the commander's gearshift
		//     is turned OFF so it HOLDS the target rung (the test isolates the emit+gate
		//     from further climb churn — the climb itself is covered by the election test).
		//
		// CLEAN-BOUNDARY GATE (diag/livepath-sim first-block-race localization, 2026-06-06):
		// production issues SET_CONFIG ONLY at a DRAINED data-batch boundary — the FRAME-UP
		// promotion fires (arq_commander.cc:3840/3899) when consecutive_data_acks >=
		// threshold, i.e. the prior data batch was already DELIVERED to the RSP AND ACKed
		// back to the CMD, so NO data frame is in flight when the SET_CONFIG control frame
		// goes onto the wire. The original force-fire guard fired the instant both peers were
		// CONNECTED, which (in this single-process sim, where the wire DECOUPLES TX-drain from
		// RX-feed, §10.5) collided the SET_CONFIG with the FIRST ROBUST_0 DATA frame still in
		// flight: B decoded the stale in-flight DATA_LONG frame (byte0=0x10) instead of the
		// SET_CONFIG (byte0=0x3B), SACK-ACKed it as data, A mistook that data-SACK for the
		// control-ACK and switched to CFG16 while B stayed on ROBUST_0 — so the post-switch
		// big-block was undecodable by B and the gate never ran ("First-batch ACK miss" ->
		// BREAK). That was a HARNESS-injection-TIMING artifact, NOT a production race (the
		// production control loop structurally cannot fire SET_CONFIG with data in flight).
		// Faithful fix (two parts): (1) the payload is DEFERRED until AFTER this transition
		// applies (see the deferred-push block below), so at CONNECT A has NOTHING to send
		// and the pipeline is empty; (2) this fire is additionally gated on the WIRE being
		// QUIESCENT (both sim wires + both play buffers drained) as a belt-and-suspenders
		// guard so the SET_CONFIG control frame is genuinely the next/only frame on the
		// wire. With the payload deferred, wire_quiescent holds on the first CONNECTED poll,
		// so B receives the SET_CONFIG in sequence and runs its real SET_CONFIG handler
		// (arq_responder.cc:2497) — no ROBUST_0 DATA frame collides with it. Harness-only;
		// production code is untouched (production already issues SET_CONFIG at a drained
		// batch boundary, arq_commander.cc:3840).
		bool wire_quiescent =
		    size_buffer(wire_a2b) == 0 && size_buffer(wire_b2a) == 0 &&
		    size_buffer(A->audio.play) == 0 && size_buffer(B->audio.play) == 0;
		if (force_setconfig >= 0 && !force_setconfig_done &&
		    A->arq.link_status == CONNECTED && B->arq.link_status == CONNECTED &&
		    A->arq.connection_status != TRANSMITTING_CONTROL &&
		    A->arq.connection_status != RECEIVING_ACKS_CONTROL &&
		    A->arq.messages_control.status == FREE &&
		    (wire_quiescent || defeat_firstblock_fix)) // no in-flight frame at injection
		                                               // (defeat: fire immediately = artifact)
		{
			sim2_activate(A);
			printf("[SIM2-LIVEPATH] CONNECTED at cfg=%d; firing REAL SET_CONFIG jump "
			       "-> CONFIG_%ld via the production gearshift mechanism (no pin)\n",
			       A->arq.current_configuration, force_setconfig);
			fflush(stdout);
			// EXACT gearshift FRAME-UP sequence (arq_commander.cc:3867/3899-3900): put any
			// pending TX-staged data back in the FIFO for re-encode at the new config, set
			// the negotiated config, queue the real SET_CONFIG control frame, transition.
			for (int i = A->arq.nMessages - 1; i >= 0; i--) {
				if (A->arq.messages_tx[i].status != FREE && A->arq.messages_tx[i].length > 0)
					A->arq.fifo_buffer_tx.push_front(A->arq.messages_tx[i].data,
					                                 A->arq.messages_tx[i].length);
				A->arq.messages_tx[i].status = FREE;
			}
			// PUSH THE DEFERRED PAYLOAD NOW (diag/livepath-sim first-block-race fix):
			// stage the payload into A's FIFO at the SAME point production restores pending
			// data before SET_CONFIG (arq_commander.cc:3889-3899) — so A carries the data
			// THROUGH the transition and re-encodes it at the new config after the control-
			// ACK. If we left the FIFO empty here, A would reach CFG16 with nothing to send
			// and its ARQ state machine would issue a spurious SET_CONFIG-retransmit then a
			// SWITCH_ROLE (reverse turboshift) on the idle link, never delivering. Staging
			// the data here makes the post-ACK data path flow as real CFG16 big-blocks.
			if (force_setconfig >= 0 && !payload_pushed) {
				A->arq.fifo_buffer_tx.push((char*)payload, payload_len);
				payload_pushed = true;
				printf("[SIM2-LIVEPATH] staged deferred %d-byte payload into A's FIFO at the "
				       "SET_CONFIG boundary (carries through the transition; flows as CFG16 "
				       "big-blocks post-ACK)\n", payload_len);
				fflush(stdout);
			}
			A->arq.block_under_tx = NO;
			A->arq.negotiated_configuration = (int)force_setconfig;
			A->arq.add_message_control(SET_CONFIG);
			A->arq.connection_status = TRANSMITTING_CONTROL;
			// HOLD the rung after the jump applies (no further climb churn).
			A->arq.gear_shift_on = NO;
			B->arq.gear_shift_on = NO;
			force_setconfig_done = true;
		}

		// --- IN-BAND DOWN-LADDER RESYNC: fire ONE production demote A->ROBUST once both
		//     peers are CONNECTED and A's control channel is idle, with A's announce tag
		//     SUPPRESSED (set up front). This is the gearshift's OWN demote mechanism (the
		//     §3.1 chokepoint): set negotiated_configuration to the ROBUST target and call
		//     add_message_control(SET_CONFIG) — under MERCURY_INBAND_RATE that takes the
		//     UNILATERAL path (inband_unilateral_config_change loads ROBUST NOW; the next
		//     send_batch W1-emits the tag, which we drop). A then transmits the payload as
		//     ROBUST data frames; B, still at the start OFDM rung (announce never heard),
		//     fails its primary OFDM decode and runs the production down-ladder over a window
		//     that includes the ROBUST rung. The wire is left quiescent at injection (no
		//     in-flight frame) exactly as the FORCE_SETCONFIG path requires. ---
		if (inband_lag && !inband_lag_demote_done &&
		    A->arq.link_status == CONNECTED && B->arq.link_status == CONNECTED &&
		    A->arq.connection_status != TRANSMITTING_CONTROL &&
		    A->arq.connection_status != RECEIVING_ACKS_CONTROL &&
		    A->arq.messages_control.status == FREE &&
		    size_buffer(wire_a2b) == 0 && size_buffer(wire_b2a) == 0 &&
		    size_buffer(A->audio.play) == 0 && size_buffer(B->audio.play) == 0)
		{
			sim2_activate(A);
			printf("[SIM2-INBAND-LAG] CONNECTED at cfg=%d; firing production DEMOTE -> "
			       "CONFIG_%ld (announce suppressed -> B must down-ladder resync)\n",
			       A->arq.current_configuration, inband_demote_cfg);
			fflush(stdout);
			// Restore any TX-staged frames to the FIFO so the demote re-encodes them at the
			// ROBUST config (mirrors the production demote refill / the FORCE_SETCONFIG path).
			for (int i = A->arq.nMessages - 1; i >= 0; i--) {
				if (A->arq.messages_tx[i].status != FREE && A->arq.messages_tx[i].length > 0)
					A->arq.fifo_buffer_tx.push_front(A->arq.messages_tx[i].data,
					                                 A->arq.messages_tx[i].length);
				A->arq.messages_tx[i].status = FREE;
			}
			A->arq.block_under_tx = NO;
			A->arq.negotiated_configuration = (int)inband_demote_cfg;
			A->arq.add_message_control(SET_CONFIG);          // unilateral drop + (suppressed) tag
			A->arq.connection_status = TRANSMITTING_CONTROL; // neutralised by inband_unilateral_armed
			inband_lag_demote_done = true;
		}

		// RECOVER leg: once B has delivered its FIRST byte the down-ladder DID resync B onto
		// the demoted config (a total-loss down-ladder delivers nothing), so the §6
		// demote->down-ladder->recover sequence has crossed its pivot. Un-suppress A's
		// announce so any subsequent config change re-announces normally (B follows by tag,
		// the implicit-confirm recover path) — proving the link is healthy post-resync, not
		// permanently leaning on the down-ladder.
		if (inband_lag && inband_lag_demote_done && !inband_lag_recovered && rx_have > 0)
		{
			A->arq.inband_test_suppress_announce_tx = false;
			inband_lag_recovered = true;
			printf("[SIM2-INBAND-LAG] B resynced via down-ladder (rx_have=%d) -> announce "
			       "UN-suppressed (recover leg armed)\n", rx_have);
			fflush(stdout);
		}

		// --- Track the commander's config switches (the climb path). ---
		int cur_cfg = A->arq.current_configuration;
		if (cur_cfg != last_cfg) {
			cfg_seq.push_back({iters, cur_cfg});
			// max_cfg_reached only over OFDM configs (ROBUST_x are id>=100; a
			// numeric max would wrongly rank ROBUST over CONFIG_16). Track the
			// highest OFDM config touched, and note the first collapse to ROBUST.
			if (is_ofdm_config(cur_cfg) && cur_cfg > max_cfg_reached) max_cfg_reached = cur_cfg;
			if (cur_cfg >= ROBUST_0 && collapse_iter < 0 && max_cfg_reached > last_cfg && is_ofdm_config(last_cfg))
				collapse_iter = iters;
			if (dbg) {
				printf("[SIM2-CFG] it=%ld cfg %d -> %d (max_ofdm=%d)\n",
				       iters, last_cfg, cur_cfg, max_cfg_reached);
				fflush(stdout);
			}
			last_cfg = cur_cfg;
		}

		// Drain delivered bytes from B's RX FIFO.
		int avail = B->arq.fifo_buffer_rx.get_size() - B->arq.fifo_buffer_rx.get_free_size();
		if (avail > 0 && rx_have < rx_cap - 1)
		{
			int got = B->arq.fifo_buffer_rx.pop(rx_buf + rx_have,
			              std::min(avail, rx_cap - 1 - rx_have));
			if (got > 0) rx_have += got;
		}

		if (rx_have >= payload_len) break;   // delivered

		// FULL-PATH REGRESSION (bigblock-whiten-align): MERCURY_SIM2_STOP_AFTER_FIRST_BLOCK=1
		// breaks the loop as soon as the FIRST big-block has been carved (bigblock_first_K>0).
		// The fail-before arm uses this so it captures the first block's PARTIAL outcome and
		// exits FAST — it must NOT run the slow/unstable post-partial stock per-frame retry
		// loop. Default off (regression/HW sessions run to completion).
		//
		// C0-a GRACE (fact-doc §10.8): the CLEAN big-block now delivers via the responder
		// ACK-GATE on a LATER responder tick (the carve transitions to ACKNOWLEDGING_DATA;
		// the ACK-GATE TX's the data-ACK then copy_data_to_buffer()'s to the FIFO) — it is
		// NO LONGER delivered in-carve. So after the first block is carved, give the RSP a
		// bounded grace for that ACK-GATE tick to deliver before terminating; the loop top
		// drains B's FIFO every iter, so a clean block fills rx_have within a handful of
		// iters and hits the rx_have>=payload_len break above FIRST. A PARTIAL/fail-before
		// block legitimately delivers 0 (its ACK-GATE sends a SACK, pushes nothing to the
		// FIFO), so it exits after the grace with rx_have=0 — the SAME fail-before signature
		// as before, just one ACK-GATE tick later. Grace bounds wall time (each grace iter is
		// one cheap per-symbol step). Mirrors the GAP-2 STOP_AFTER_TX_EMITS grace pattern.
		if (env_i("MERCURY_SIM2_STOP_AFTER_FIRST_BLOCK", 0) != 0 && bigblock_first_K > 0) {
			if (first_block_break_arm_iter < 0) first_block_break_arm_iter = iters;
			const long fb_grace = env_i("MERCURY_SIM2_FIRST_BLOCK_GRACE_ITERS", 64);
			bool fb_grace_expired = (iters - first_block_break_arm_iter) >= fb_grace;
			if (fb_grace_expired) {
				stalled = true;   // mark so the byte-correct G-SMOKE assert is skipped
				break;
			}
		}

		// GAP-2 LIVE-PATH: MERCURY_SIM2_STOP_AFTER_TX_EMITS=N breaks the loop once N real
		// big-blocks have been EMITTED (sim2_tx_block_emits>=N) AND the RX cw0-CRC gate has
		// rendered at least one decision (accepts+rejects>=1). The live-path regression only
		// needs the TX switch to have engaged + the gate to have run on real emitted blocks —
		// it does NOT need full delivery (the first-block-after-PHY-switch ACK race blocks
		// delivery in this sim, and the CMD's internal ACK-timeout retransmit spin keeps A
		// inside process_main for many seconds of WALL time per virtual-ACK-timeout, so the
		// iter-based stall cutoff is too slow). Breaking at the FIRST loop-bottom after the
		// emit+gate observables are captured gives the test a deterministic, fast exit.
		// Default 0 = disabled (every existing arm is byte-identical).
		{
			long stop_emits = env_i("MERCURY_SIM2_STOP_AFTER_TX_EMITS", 0);
			if (stop_emits > 0 && sim2_tx_block_emits >= stop_emits) {
				// Give the RSP a bounded GRACE WINDOW after the Nth emit to acquire/decode
				// the block so the cw0-CRC gate can render its decision (accepts/rejects)
				// BEFORE we terminate — UNLESS the gate has already decided (then break now).
				// The grace bounds wall time (A's post-emit ACK-wait spin is pump-driven, so
				// each grace iter is cheap relative to a full virtual-ACK-timeout). If the RSP
				// never decodes within the grace, that itself is the finding (the gate never
				// runs on the live post-switch block — the RSP first-block-after-switch race).
				const long grace = env_i("MERCURY_SIM2_TX_EMIT_GRACE_ITERS", 0);
				if (emit_break_arm_iter < 0) emit_break_arm_iter = iters;   // first time threshold met
				bool gate_decided = (sim2_gate_accepts + sim2_gate_rejects) >= 1;
				bool grace_expired = (iters - emit_break_arm_iter) >= grace;
				if (gate_decided || grace_expired) {
					printf("[SIM2-LIVEPATH] stop-after-tx-emits: emits=%ld gate(accepts=%ld "
					       "rejects=%ld) at iter=%ld (grace_iters=%ld, gate_decided=%d) "
					       "B.rx_have=%d — terminating (delivery not asserted on this arm)\n",
					       sim2_tx_block_emits, sim2_gate_accepts, sim2_gate_rejects, iters,
					       iters - emit_break_arm_iter, (int)gate_decided, rx_have);
					fflush(stdout);
					stalled = true;             // delivery not asserted on this arm
					break;
				}
			}
		}

		// Stall detector (large-payload arm): break out if no new byte for
		// stall_cutoff iters. Records the stall so the report can flag it.
		if (rx_have != prev_rx_have) { prev_rx_have = rx_have; last_progress_iter = iters; }
		if (stall_cutoff > 0 && (iters - last_progress_iter) >= stall_cutoff) {
			stalled = true;
			break;
		}
	}
	uint64_t t1 = sim_clock_now_samples();
	double sim_ms = (t1 - t0) * 1000.0 / SIM_CLOCK_SAMPLE_RATE_HZ;
	bool bytes_ok = (rx_have >= payload_len && memcmp(rx_buf, payload, payload_len) == 0);
	// SIM_INPROC fix #1: fold the pump-watchdog wedge signal (fail-before arm) into the
	// stalled flag so the wedge is reported as a FAIL by the byte-correct G-SMOKE assert.
	if (pump.deliver_stalled) stalled = true;
	// Disarm the post-delivery abort flag now this run owns it no longer (the next
	// stepper run re-resets it; clearing here keeps the global clean for any follow-on).
	arq_set_sim_inproc_deliver_done(false);

	// FULL-PATH REGRESSION capture (bigblock-whiten-align): record this run's delivery so
	// test_sim_inproc_bigblock_fullpath() can assert byte-faithful delivery directly.
	sim2_last_rx_have     = rx_have;
	sim2_last_payload_len = payload_len;
	sim2_last_bytes_ok    = bytes_ok;
	sim2_last_stalled     = stalled;

	if (payload_bytes <= 0) {
		// Legacy short-text arm: print the string (regression output unchanged).
		rx_buf[rx_have] = '\0';
		printf("[TEST-SIM-2INST] loop done: iters=%ld sim_ms=%.0f connected=%d "
		       "A.link=%d B.link=%d rx_have=%d rx=\"%s\"\n", iters, sim_ms,
		       (int)connected_seen, A->arq.link_status, B->arq.link_status,
		       rx_have, rx_buf);
	} else {
		// Large/binary arm: no string print (binary); report length + correctness.
		printf("[TEST-SIM-2INST] loop done: iters=%ld sim_ms=%.0f connected=%d "
		       "A.link=%d B.link=%d rx_have=%d/%d bytes_ok=%d\n", iters, sim_ms,
		       (int)connected_seen, A->arq.link_status, B->arq.link_status,
		       rx_have, payload_len, (int)bytes_ok);
	}
	printf("[TEST-SIM-2INST] pump: calls=%lld looped=%lld idle=%lld clock=%lld\n",
	       pump.pump_calls, pump.looped_samples, pump.idle_samples, pump.clock_samples);

	// --- CONTROL-LOOP PROBE report (wf-sim-controlloop) ---
	// Config switch sequence (the climb path). Tag the final config + whether it
	// is OFDM (held a real data config) or fell back to a ROBUST_x (the collapse).
	{
		int final_cfg = A->arq.current_configuration;
		printf("[SIM2-PROBE] opt=%s snr3k=%.1f payload_len=%d : switch_seq=",
		       opt_on ? "ON" : "OFF", snr3k_db, payload_len);
		for (size_t i = 0; i < cfg_seq.size(); i++) {
			printf("%s%d", (i ? "->" : ""), cfg_seq[i].second);
		}
		printf("  (final=%d ofdm=%d max_ofdm=%d collapse_iter=%ld)\n",
		       final_cfg, (int)is_ofdm_config(final_cfg), max_cfg_reached, collapse_iter);
		// Per-switch (iter,cfg) detail for the climb timeline.
		printf("[SIM2-PROBE] switch_detail=");
		for (size_t i = 0; i < cfg_seq.size(); i++) {
			printf("%s(%ld:%d)", (i ? "," : ""), cfg_seq[i].first, cfg_seq[i].second);
		}
		printf("\n");
		// Delivered rate in SIM UNITS: bits delivered / virtual-channel seconds.
		// This is a sim-internal relative metric for OFF-vs-ON comparison at the
		// SAME seed/SNR — NOT an absolute bps-vs-VARA figure (clean/AWGN, not
		// GATE-3-validated vs PACED/HW).
		double sim_s = sim_ms / 1000.0;
		double delivered_bps_sim = (sim_s > 0.0) ? (rx_have * 8.0 / sim_s) : 0.0;
		printf("[SIM2-PROBE] delivered: rx_bytes=%d sim_ms=%.0f delivered_bps_sim=%.1f "
		       "final_cfg=%d held_ofdm=%d stalled=%d\n",
		       rx_have, sim_ms, delivered_bps_sim, final_cfg, (int)is_ofdm_config(final_cfg),
		       (int)stalled);
		// GAP-2 LIVE-PATH: TX block emits + cw0-CRC gate decision tally for this run (the
		// live-path regression reads sim2_tx_block_emits / sim2_gate_* directly; logged too).
		printf("[SIM2-PROBE] bigblock: tx_emits=%ld cw0crc_gate(accepts=%ld rejects=%ld)\n",
		       sim2_tx_block_emits, sim2_gate_accepts, sim2_gate_rejects);
		fflush(stdout);
	}
	fflush(stdout);

	// --- G-SMOKE asserts ---
	// CONNECT + no-hang are hard asserts for ALL arms. The byte-correct delivery
	// assert is a hard assert only for the LEGACY short arm and for any LARGE arm
	// that completed without stalling: an over-climb-collapse arm may intentionally
	// FAIL to deliver the full payload at moderate SNR (it crawls in ROBUST_0 and
	// hits the stall cutoff) — that is the scientific RESULT of the probe, not a
	// code failure. The probe report above carries bytes_ok/stalled for analysis.
	check(connected_seen, "G-SMOKE: 2-instance CONNECT completed (no deadlock, single thread)");
	if (payload_bytes <= 0 || !stalled) {
		check(bytes_ok,
		      "G-SMOKE: payload delivered B<-A byte-correct (RX bytes match TX)");
	} else {
		printf("[TEST-SIM-2INST] NOTE: large-payload arm STALLED (rx=%d/%d, "
		       "final_cfg=%d) — over-climb/collapse regime, byte-correct assert "
		       "skipped (this is the probe result, not a failure)\n",
		       rx_have, payload_len, A->arq.current_configuration);
		fflush(stdout);
	}
	check(iters < max_iters, "G-SMOKE: terminated before iteration cap (no hang)");

	// --- Teardown ---
	arq_set_sim_inproc_outer_stepper(false);   // disarm the Phase-b queue-and-return gate
	arq_set_sim_inproc_drain_only_pump(nullptr);
	arq_set_sim_inproc_pump(nullptr, nullptr);
	arq_set_sim_inproc_skip_tcp(false);
	sim_clock_set_enabled(0);
	if (pump.scratch) free(pump.scratch);
	free(wire_a2b->buffer); circular_buf_free(wire_a2b);
	free(wire_b2a->buffer); circular_buf_free(wire_b2a);
	// Restore globals to a clean null state (this run owned them).
	capture_buffer = nullptr; playback_buffer = nullptr;
#if defined(_WIN32)
	capture_prep_mutex = NULL;
#endif
	A->audio.free_all();
	B->audio.free_all();
#if !defined(_WIN32)
	const int mutex_reset_rc = sim2_reset_capture_prep_mutex();
	check(mutex_reset_rc == 0,
	      "G-CLEANUP: capture_prep_mutex reinitialized and lock-validated after instance teardown");
#endif
	delete A; delete B;

	printf("[TEST-SIM-2INST] %s (%d failure%s)\n",
	       failed == 0 ? "ALL PASS" : "FAILURES", failed, failed == 1 ? "" : "s");
	fflush(stdout);
	return failed == 0 ? 0 : 1;
}

// POSIX regression for the SIM_INPROC two-instance mutex teardown.  Copying a
// locked mutex reproduces sim2_activate's by-value global swap while making the
// stale state deterministic even on pthread implementations that do not poison
// destroyed mutexes.  The defeat arm leaves that copy locked; the fixed helper
// must replace it with a fresh, usable mutex.
int cl_arq_controller::test_sim2_capture_mutex_cleanup()
{
#if defined(_WIN32)
	printf("[TEST-SIM2-MUTEX-CLEANUP] SKIP (POSIX-only regression)\n");
	return 0;
#else
	int failed = 0;
	auto check = [&](bool cond, const char* name) {
		printf("[TEST-SIM2-MUTEX-CLEANUP] %s: %s\n", cond ? "PASS" : "FAIL", name);
		if (!cond) failed++;
	};

	const char* old_defeat = std::getenv("MERCURY_SIM2_CAPTURE_MUTEX_RESET_DEFEAT");
	const bool had_old_defeat = old_defeat != nullptr;
	const std::string saved_defeat = had_old_defeat ? old_defeat : "";

	pthread_mutex_t instance_mutex;
	int rc = pthread_mutex_init(&instance_mutex, nullptr);
	check(rc == 0, "instance pthread_mutex_init succeeds");
	if (rc != 0)
		return 1;
	rc = pthread_mutex_lock(&instance_mutex);
	check(rc == 0, "instance mutex locks before sim2_activate-style copy");
	if (rc != 0) {
		const int destroy_rc = pthread_mutex_destroy(&instance_mutex);
		check(destroy_rc == 0, "failed-lock instance mutex cleanup succeeds");
		return 1;
	}
	capture_prep_mutex = instance_mutex;
	rc = pthread_mutex_unlock(&instance_mutex);
	check(rc == 0, "instance mutex unlock succeeds");
	rc = pthread_mutex_destroy(&instance_mutex);
	check(rc == 0, "instance mutex destroy succeeds");

	rc = setenv("MERCURY_SIM2_CAPTURE_MUTEX_RESET_DEFEAT", "1", 1);
	check(rc == 0, "fail-before defeat arm enabled");
	const int defeated_reset_rc = sim2_reset_capture_prep_mutex();
	check(defeated_reset_rc == ECANCELED, "fail-before cleanup is deterministically bypassed");
	int stale_lock_rc = pthread_mutex_trylock(&capture_prep_mutex);
	check(stale_lock_rc != 0, "fail-before copied mutex is not usable after instance destroy");
	if (stale_lock_rc == 0) {
		const int stale_unlock_rc = pthread_mutex_unlock(&capture_prep_mutex);
		check(stale_unlock_rc == 0, "unexpected fail-before lock is safely released");
	}

	rc = unsetenv("MERCURY_SIM2_CAPTURE_MUTEX_RESET_DEFEAT");
	check(rc == 0, "fail-before defeat arm disabled");
	const int reset_rc = sim2_reset_capture_prep_mutex();
	check(reset_rc == 0, "pass-after cleanup reinitializes and validates the global mutex");
	if (reset_rc == 0) {
		const int lock_rc = pthread_mutex_trylock(&capture_prep_mutex);
		check(lock_rc == 0, "pass-after production lock path can take capture_prep_mutex");
		if (lock_rc == 0) {
			const int unlock_rc = pthread_mutex_unlock(&capture_prep_mutex);
			check(unlock_rc == 0, "pass-after production lock path releases capture_prep_mutex");
		}
	}

	const int restore_rc = had_old_defeat
		? setenv("MERCURY_SIM2_CAPTURE_MUTEX_RESET_DEFEAT", saved_defeat.c_str(), 1)
		: unsetenv("MERCURY_SIM2_CAPTURE_MUTEX_RESET_DEFEAT");
	check(restore_rc == 0, "defeat environment restored");
	fflush(stdout);
	return failed == 0 ? 0 : 1;
#endif
}


// SACK Design A Step 11 — helper: stage and send a SET_LINK_PARAMS for a
// new Axis-3 sack_mode value. Carries the CURRENT data_batch_size in the
// batch field so the RSP-side controller (which already handles
// SET_LINK_PARAMS for Axis-2 moves) does not inadvertently revert the
// batch size when only the sack_mode changes. Returns true if the control
// frame was queued; false if messages_control is busy (caller logs).
static const char* axis3_mode_str(int m)
{
	switch(m)
	{
		case 0: return "OFF";
		case 1: return "ON";
		case 2: return "PROBE";
		default: return "?";
	}
}

void cl_arq_controller::axis3_send_set_link_params(int new_sack_mode, const char* reason_tag)
{
	pending_link_params_batch_size = data_batch_size; // do not change batch on a pure-Axis-3 move
	pending_link_params_sack_mode  = new_sack_mode;
	if(messages_control.status == FREE)
	{
		add_message_control(SET_LINK_PARAMS);
	}
	else
	{
		printf("[POLICY-AXIS3] WARNING: messages_control busy (status=%d) — "
			"SET_LINK_PARAMS NOT sent this cycle for %s; RSP will adopt mode on "
			"the next SET_LINK_PARAMS cycle (or via the EOB-self-correct safety "
			"net for the data path). axis3_sack_mode=%s (CMD-side already applied).\n",
			messages_control.status, reason_tag, axis3_mode_str(axis3_sack_mode));
		fflush(stdout);
	}
}

// SACK Design A Step 11 — Axis 3 controller (SACK mode adaptation).
//
// §4.3.2 per-SACK-event controller. Fires from:
//   - process_messages_rx_acks_data() at the decode_sack_v2_frame() success
//     branch (ok=true).
//   - process_messages_rx_acks_data() at the decode_sack_v2_frame() CRC-fail
//     branch (ok=false).
//   - process_messages_rx_acks_data() ACK-timeout path when data_ack_received
//     stayed NO (= "no SACK heard within the window") for sack_v2 sessions
//     (ok=false).
//
// §4.3.2 hysteresis:
//   ON   → PROBE on consecutive_sack_misses >= 3
//   PROBE→ ON    on the next ok event (reset counter)
//   PROBE→ OFF   on consecutive_sack_misses >= 5 (the counter is NOT
//                reset on PROBE entry — 5 is total consecutive misses
//                from the streak that took ON → PROBE)
//   OFF  → PROBE every 20 batches (axis3_batch_tick(); not this function)
//
// §4.3.3 / §4.3.4 invariant #6: skip MOVES (not observation recording)
// when axis3_cooldown_batches > 0 (set to 3 by Axis-1 supremacy hook).
// Observations still flow into the ring so the controller has up-to-date
// state when the cooldown drains; only the state transition + control
// frame TX are suppressed.
//
// On state change: log [POLICY-MOVE] axis=3 from=X to=Y reason=... per
// §4.3.4 invariant #5 AND fire SET_LINK_PARAMS to inform RSP.
void cl_arq_controller::policy_evaluate_axis3(bool ok)
{
	axis3_evaluations++;

	// Record the event into the ring + bump streak counter.
	axis3_recent_sack_ok[axis3_recent_sack_ok_pos] = ok;
	axis3_recent_sack_ok_pos = (axis3_recent_sack_ok_pos + 1) % AXIS3_RING_DEPTH;
	if(axis3_recent_sack_ok_count < AXIS3_RING_DEPTH) axis3_recent_sack_ok_count++;
	if(ok)
	{
		axis3_ok_events++;
		axis3_consecutive_sack_misses = 0;
	}
	else
	{
		axis3_miss_events++;
		axis3_consecutive_sack_misses++;
	}

	// Compute sack_ok_rate over the filled portion of the ring (diagnostic).
	int ok_n = 0;
	for(int i=0;i<axis3_recent_sack_ok_count;i++) if(axis3_recent_sack_ok[i]) ok_n++;
	float sack_ok_rate = (axis3_recent_sack_ok_count > 0)
		? (float)ok_n / (float)axis3_recent_sack_ok_count : 0.0f;

	// Cooldown gate (Axis-1 supremacy): observations recorded above, MOVES skipped.
	if(axis3_cooldown_batches > 0)
	{
		axis3_skipped_in_cooldown++;
		printf("[POLICY-AXIS3] eval ok=%d consec_misses=%d/%d ok_rate=%.2f mode=%s "
			"COOLDOWN_REMAINING=%d (no move)\n",
			(int)ok, axis3_consecutive_sack_misses,
			(axis3_sack_mode == SACK_MODE_ON ? AXIS3_ON_TO_PROBE_MISSES
				: AXIS3_PROBE_TO_OFF_MISSES),
			sack_ok_rate, axis3_mode_str(axis3_sack_mode),
			axis3_cooldown_batches);
		fflush(stdout);
		return;
	}

	int from_mode = axis3_sack_mode;
	int to_mode   = from_mode;
	const char* reason = NULL;

	if(from_mode == SACK_MODE_ON)
	{
		if(axis3_consecutive_sack_misses >= AXIS3_ON_TO_PROBE_MISSES)
		{
			to_mode = SACK_MODE_PROBE;
			reason  = "consecutive_misses>=3";
		}
	}
	else if(from_mode == SACK_MODE_PROBE)
	{
		if(ok)
		{
			to_mode = SACK_MODE_ON;
			reason  = "probe_recovered";
		}
		else if(axis3_consecutive_sack_misses >= AXIS3_PROBE_TO_OFF_MISSES)
		{
			to_mode = SACK_MODE_OFF;
			reason  = "consecutive_misses>=5";
		}
	}
	else if(from_mode == SACK_MODE_OFF)
	{
		// While OFF, decode_sack_v2_frame is not even called → policy_evaluate_axis3
		// is normally not invoked. If it IS (synthetic test fire), any ok event
		// in OFF is an unambiguous signal that the reverse path recovered — the
		// spec says "PROBE→ON: any successful SACK while in PROBE", not OFF;
		// here we route OFF→PROBE so the next batch's outcome confirms.
		// The periodic OFF→PROBE re-probe is driven by axis3_batch_tick().
		// We do NOT transition on miss events while OFF (the controller
		// already chose OFF; further misses don't change the verdict).
		if(ok)
		{
			to_mode = SACK_MODE_PROBE;
			reason  = "off_unexpected_ok";
		}
	}

	if(to_mode == from_mode)
	{
		printf("[POLICY-AXIS3] eval ok=%d consec_misses=%d ok_rate=%.2f mode=%s "
			"(no move)\n",
			(int)ok, axis3_consecutive_sack_misses, sack_ok_rate,
			axis3_mode_str(from_mode));
		fflush(stdout);
		return;
	}

	// State transition.
	printf("[POLICY-MOVE] axis=3 from=%s to=%s reason=%s consec_misses=%d "
		"ok_rate=%.2f ring_n=%d\n",
		axis3_mode_str(from_mode), axis3_mode_str(to_mode), reason,
		axis3_consecutive_sack_misses, sack_ok_rate, axis3_recent_sack_ok_count);
	fflush(stdout);

	axis3_sack_mode = to_mode;

	// Reset counters appropriately. On PROBE→ON, reset consecutive_misses
	// (the probe succeeded). On ON→PROBE, KEEP consecutive_misses (we need
	// it to count up to 5 for PROBE→OFF). On OFF→PROBE, reset.
	if(from_mode == SACK_MODE_ON && to_mode == SACK_MODE_PROBE)
	{
		// Keep consecutive_misses; ring stays so PROBE→OFF can fire on +2 more.
		axis3_move_on_to_probe_count++;
	}
	else if(from_mode == SACK_MODE_PROBE && to_mode == SACK_MODE_ON)
	{
		axis3_consecutive_sack_misses = 0;
		axis3_move_probe_to_on_count++;
	}
	else if(from_mode == SACK_MODE_PROBE && to_mode == SACK_MODE_OFF)
	{
		axis3_move_probe_to_off_count++;
		axis3_batches_since_off = 0; // start counting 20 batches before re-probe
	}
	else if(from_mode == SACK_MODE_OFF && to_mode == SACK_MODE_PROBE)
	{
		axis3_consecutive_sack_misses = 0;
		axis3_move_off_to_probe_count++;
	}

	// Inform RSP via SET_LINK_PARAMS (CMD-side flow control — §3.8).
	axis3_send_set_link_params(to_mode, reason ? reason : "axis3_move");
}

// SACK Design A Step 11 — per-batch tick. Drives the OFF→PROBE periodic
// re-probe (every 20 batches). Decrements the Axis-1 supremacy cooldown
// (so 3 batches after an Axis-1 move, Axis-3 is free to move again).
//
// Called once per batch completion from process_messages_rx_acks_data()
// regardless of whether a SACK event fired (covers OFF state too). Gated
// on sack_v2_enabled at the call site; v1 sessions never call this.
void cl_arq_controller::axis3_batch_tick()
{
	// Drain Axis-1 supremacy cooldown for Axis 3.
	if(axis3_cooldown_batches > 0)
	{
		axis3_cooldown_batches--;
	}

	// OFF-state re-probe timer.
	if(axis3_sack_mode != SACK_MODE_OFF) return;
	axis3_batches_since_off++;
	if(axis3_batches_since_off < AXIS3_OFF_TO_PROBE_BATCHES) return;

	// Skip move if Axis-1 supremacy cooldown is active (consistent with
	// policy_evaluate_axis3); we'll re-attempt next tick.
	if(axis3_cooldown_batches > 0)
	{
		axis3_skipped_in_cooldown++;
		return;
	}

	// 20 batches in OFF → fire OFF→PROBE.
	printf("[POLICY-MOVE] axis=3 from=OFF to=PROBE reason=periodic_reprobe_20_batches "
		"consec_misses=%d ring_n=%d\n",
		axis3_consecutive_sack_misses, axis3_recent_sack_ok_count);
	fflush(stdout);
	axis3_sack_mode = SACK_MODE_PROBE;
	axis3_consecutive_sack_misses = 0;
	axis3_batches_since_off = 0;
	axis3_move_off_to_probe_count++;
	axis3_send_set_link_params(SACK_MODE_PROBE, "periodic_reprobe");
}

// SACK Design A Step 11 — synthetic Axis 3 fire (test-only).
// CLI: --test-policy-axis3-fire=ok|miss → single event into
// policy_evaluate_axis3.
// CLI: --test-policy-axis3-walk → composite demo: drive 3 misses
// (ON→PROBE), then 2 more (PROBE→OFF), then OFF→PROBE via batch_tick (20x).
// Default builds never enter this; production paths unaffected.
void cl_arq_controller::test_fire_policy_axis3(int kind)
{
	sack_v2_enabled = true;
	if(kind == 1)
	{
		printf("[TEST-AXIS3-FIRE] kind=ok: feeding single ok event into "
			"policy_evaluate_axis3 (starting mode=%s)\n",
			axis3_mode_str(axis3_sack_mode));
		fflush(stdout);
		policy_evaluate_axis3(true);
	}
	else if(kind == 2)
	{
		printf("[TEST-AXIS3-FIRE] kind=miss: feeding single miss event into "
			"policy_evaluate_axis3 (starting mode=%s)\n",
			axis3_mode_str(axis3_sack_mode));
		fflush(stdout);
		policy_evaluate_axis3(false);
	}
	else if(kind == 3)
	{
		// Composite walk: ON → PROBE → OFF → PROBE.
		// Note: SET_LINK_PARAMS TX is GUARDED by the messages_control.data NULL
		// check in add_message_control(); in pre-init synthetic mode this fast-
		// outs cleanly without crashing. We still observe the [POLICY-MOVE]
		// log lines on every transition (the load-bearing evidence).
		printf("[TEST-AXIS3-WALK] starting walk demo from mode=%s "
			"(expect ON→PROBE on 3rd miss, PROBE→OFF on 5th miss, "
			"OFF→PROBE on the 20-batch tick).\n",
			axis3_mode_str(axis3_sack_mode));
		fflush(stdout);

		// Phase 1: 3 consecutive misses → ON→PROBE.
		for(int i=1;i<=3;i++)
		{
			printf("[TEST-AXIS3-WALK] miss #%d (mode before=%s, consec=%d)\n",
				i, axis3_mode_str(axis3_sack_mode),
				axis3_consecutive_sack_misses);
			fflush(stdout);
			policy_evaluate_axis3(false);
		}

		// Phase 2: 2 more consecutive misses → PROBE→OFF (total 5 misses).
		for(int i=4;i<=5;i++)
		{
			printf("[TEST-AXIS3-WALK] miss #%d (mode before=%s, consec=%d)\n",
				i, axis3_mode_str(axis3_sack_mode),
				axis3_consecutive_sack_misses);
			fflush(stdout);
			policy_evaluate_axis3(false);
		}

		// Phase 3: 20 batch ticks while OFF → OFF→PROBE periodic re-probe.
		printf("[TEST-AXIS3-WALK] now in mode=%s; ticking %d batches to drive "
			"periodic OFF→PROBE re-probe.\n",
			axis3_mode_str(axis3_sack_mode), AXIS3_OFF_TO_PROBE_BATCHES);
		fflush(stdout);
		for(int b=1;b<=AXIS3_OFF_TO_PROBE_BATCHES;b++)
		{
			axis3_batch_tick();
		}

		// Phase 4: from PROBE, single ok event → PROBE→ON.
		printf("[TEST-AXIS3-WALK] now in mode=%s; feeding single ok to "
			"drive PROBE→ON.\n", axis3_mode_str(axis3_sack_mode));
		fflush(stdout);
		policy_evaluate_axis3(true);

		printf("[TEST-AXIS3-WALK] composite walk complete. Final mode=%s. "
			"Counters: on→probe=%lld, probe→on=%lld, probe→off=%lld, "
			"off→probe=%lld.\n",
			axis3_mode_str(axis3_sack_mode),
			axis3_move_on_to_probe_count, axis3_move_probe_to_on_count,
			axis3_move_probe_to_off_count, axis3_move_off_to_probe_count);
		fflush(stdout);
	}
	else
	{
		printf("[TEST-AXIS3-FIRE] kind=%d not in {1=ok, 2=miss, 3=walk}; no-op\n",
			kind);
		fflush(stdout);
	}
}

void cl_arq_controller::process_buffer_data_commander()
{
	int data_read_size;
	if(role==COMMANDER && link_status==CONNECTED && connection_status==TRANSMITTING_DATA)
	{
		// Key exchange gate: don't fill FIFO data while key exchange is in progress.
		// process_commander() gates process_messages_tx_data(), but this function
		// is called separately from process_messages() and needs its own gate.
		if(encryption_enabled && !cipher_suite.is_active())
		{
			// KX-AS-DATA forward source-switch (data-flow-hybrid-kex.md §2.2). Under the
			// opt-in KX-as-data path, stage the reserved forward stream on first entry and
			// let the raw-leg fill below feed it as DATA batches (driving the climb).
			// Legacy pump path (env off): hold exactly as before.
			if(kx_as_data_path())
			{
				if(!kx_stream_tx_active && kx_stream_tx_len == 0
				   && cipher_suite.get_kx_phase() == KX_IDLE)
				{
					if(kx_stage_forward_stream() != 0)
						return;   // stage (RNG) failure -> hold, fail-secure (no plaintext)
					cipher_suite.set_kx_phase(KX_MLKEM_PK_SENT);  // KX started; don't re-init
				}
				// Once the KX stream drains, FALL THROUGH (do NOT return): have_new_tx below
				// excludes the held user FIFO in the KX epoch, so the fill loop only feeds the
				// KX stream when active; when it is drained control reaches the normal finalize
				// branch (clears block_under_tx) and the idle branch (which hosts the R1
				// SWITCH_ROLE reverse-ct + KEY_ACTIVATE handshake). data-flow-hybrid-kex.md Â§7.
			}
			else
				return;
		}

		// SACK retransmit pending: don't create new crypto batch from FIFO.
		// process_messages_tx_data() will send retransmit-only batch.
		//
		// SACK Design A Step 8b — second of the two CMD-side blockers
		// identified by SACK_RETRANSMIT_BATCHING_INVESTIGATION.md. Gated on
		// `!sack_v2_enabled` so v1 sessions keep the early-return unchanged
		// (preserves v1 mechanism (a) standalone retx; v1 wire-byte-identical).
		// On v2 sessions, DO NOT short-circuit: allow new-data staging into
		// messages_tx[] in parallel with the pending retransmit queue. The
		// mixed-batch builder in process_messages_tx_data() (above) prepends
		// retx as the head of the next new-data batch.
		//
		// Safety: §7.8.3's silent-corruption hazard is structurally
		// eliminated by Step 8a's messages_rx_prev[] parallel storage —
		// retx routes to the prev buffer, new-data routes to messages_rx[],
		// no slot collision.
		if(sack_enabled && !sack_v2_enabled && retransmit_count > 0)
			return;

		// SACK Design A Step 8b — relaxed staging guard for v2 mixed batches.
		//
		// The block_under_tx==NO requirement is overly conservative for v2:
		// after SACK detection, messages_tx[] has been fully drained by
		// SACK processing + cleanup() (covered slots → ACKED → FREE;
		// missing slots → queued in retransmit_count → ACKED → FREE).
		// block_under_tx stays YES until finalize_block_commander() fires,
		// which only happens on the next iteration via the else-if branch
		// below — too late: process_messages_tx_data() runs first on the
		// next iteration and builds the mixed batch with messages_tx[] empty.
		//
		// For v2 with retransmit_count > 0, the prev block is conceptually
		// complete from CMD's perspective (retx is fire-and-forget via
		// RSP's messages_rx_prev[] path). Relax the guard so new-data is
		// staged in time for the next mixed batch. v1 path unchanged
		// (block_under_tx==NO requirement preserved).
		bool stage_ok = (block_under_tx == NO)
		             || (sack_v2_enabled && retransmit_count > 0);
		// KX-as-data: the reserved forward stream lives in kx_stream_tx_buf (not
		// fifo_buffer_tx), which may be empty during the KX epoch — allow the fill when
		// the KX stream is active too. Env-gated OFF -> kx_stream_tx_active never set.
		// In the KX-as-data pre-activation epoch the user FIFO is HELD: exclude it here so
		// the fill loop only ever feeds the KX stream (kx_stream_tx_active) and NEVER pops
		// user data before activation. Once the stream drains this is false, so control
		// reaches the finalize/idle branches for the role-swap handshake. Env off / post-
		// activation: byte-identical (the user-FIFO term is restored).
		bool kx_hold_user = (encryption_enabled && !cipher_suite.is_active() && kx_as_data_path());
		bool have_new_tx = ((!kx_hold_user)
		                    && (fifo_buffer_tx.get_size()!=fifo_buffer_tx.get_free_size()))
		                   || kx_stream_tx_active;
		// MC-4 gates only this NEW-data admission edge. process_messages_tx_data()
		// has already had its chance to drain a pending pure-retx turn, and a hold
		// leaves every existing timeout/BREAK/recovery path untouched.
		// Entry-leap report hold: while the commander holds the first wideband probe
		// dispatch for the peer's forward report (entry_leap_poll_gate on this poll), do
		// not stage the next batch either. A batch staged here would key on top of the
		// report tail being waited for, and block_under_tx=YES would stop the hold and
		// its deadline from being re-checked until that whole batch completed.
		if(have_new_tx && stage_ok && entry_leap_hold_this_poll)
			return;
		bool rxwindow_hold = have_new_tx && stage_ok && cmd_rxwindow_hold_new_batch();
		if( have_new_tx && stage_ok && !rxwindow_hold)
		{
			// Option W (data-flow-stream-offset.md §2.1): total TRANSPORTED payload bytes
			// framed for the new-data batch built THIS call (comp_size in the compressed
			// leg, Σ data_read_size in the raw leg). 0 = no new bytes committed (re-entry
			// with data already staged / fifo dry) → no latch. Latched at the end of this if.
			int batch_committed_len = 0;
			// Option W STEP 3 (data-flow-stream-offset.md §8.6): snapshot the running stream
			// CRC-32 BEFORE this batch's bytes are folded (in the legs below) — this is the
			// per-bsi rollback ANCHOR, stored into tx_stream_stamp[bsi].crc at the latch, and
			// restored by stream_tx_rollback_inflight on a re-stage so a rebuild re-folds from
			// here (INV4 for the CRC). Captured unconditionally; only USED when a batch is
			// actually latched (batch_committed_len>0).
			uint32_t batch_crc_anchor = tx_stream_crc;
			// SACK Design A Step 1 — effective DATA_LONG header drives per-frame
			// payload budget. In v1 (default) identical to legacy macro; in v2
			// loses 1 byte to the batch_seq_id field.
			int max_frame = max_data_length+max_header_length-effective_data_long_header_length(sack_v2_enabled, header_carries_d5);

			// ROBUST_0 compression-deadlock fix (data-flow-compress-frame-fill.md
			// §5). Use compression ONLY when the per-batch budget can hold the
			// streaming header PLUS a payload byte. At ROBUST_0 batch_capacity ==
			// get_header_size() == 7, so compress_block() would return -1 every
			// batch and the RAW fallback would stage a 0-payload frame forever
			// (0 bps, FIFO never drains, no clean batch, no gearshift climb). The
			// SAME predicate gates the RX assembly in copy_data_to_buffer(), so
			// both peers agree (no wire negotiation): at robust both run the
			// headerless uncompressed path below; at OFDM rungs both compress.
			if(compression_viable_for_batch())
			{
				// Phase D timing — bracket the compression+encrypt+frame-split
				// work for this batch. Suspected to be the bulk of the 406ms
				// prep gap on Pi (PPMd is CPU-intensive).
				long long _comp_t0_ms = mtl::now_ms();
				// --- Batch-level compression with adaptive sizing ---
				// Pop raw data based on estimated compression ratio, compress,
				// iteratively add more data until batch is 85%+ full.
				//
				// ENC BATCH-SIZE FIX (data-flow-encrypted-batch-size.md §3/§5): when
				// encryption is active, BOUND the AEAD unit's frame span to the
				// NEGOTIATED crypto_batch_size (radio_batch_size - retransmit_headroom),
				// never the full data_batch_size. The whole-batch AEAD MAC (§2) covers
				// the EXACT ciphertext+length, so the unit must fit in frames that
				// always land — capping at crypto_batch_size leaves retransmit_headroom
				// slots free for SACK retx so the batch reliably completes inside the
				// negotiated unit and the RX reassembles a byte-exact ciphertext (no
				// short-delivery auth-fail). min() because data_batch_size legitimately
				// drops below 20 at runtime (Axis-2 floor 10, robust pin 1, big-block
				// K): the AEAD unit can never span more frames than the batch holds.
				// crypto_batch_size is a stable constant on both peers (set once in
				// reset()/CLI, never mutated at runtime), so no wire negotiation is
				// added. Non-encrypted path is byte-identical (crypto_frames ==
				// data_batch_size).
				// MIXBATCH FILL FIX -- compression leg (data-flow-mixbatch-fill.md ss9).
				// The no-compression leg (:~21333 pre-patch) caps its per-batch fill to
				// (data_batch_size - R) so the surplus that breaks the "slot order ==
				// content order" invariant is never created. The compression leg had the
				// SAME defect UNFIXED: batch_capacity was sized from the FULL data_batch_size
				// (crypto_frames below), never subtracting the v2 mixbatch retx prefix
				// R = min(retransmit_count, data_batch_size) that process_messages_tx_data()
				// prepends into the first R slots. Only the encryption path ACCIDENTALLY
				// reserved headroom (crypto_batch_size); a compressed + NON-encrypted +
				// sack_v2 batch with retx pending could split the unit into up to
				// data_batch_size frames while only (data_batch_size - R) can ride. A
				// compressed unit is ATOMIC (the RX decompresses the whole batch's new-data
				// payload as one blob), so a unit that does not fit ONE batch is undecodable
				// and its surplus tail frames are DESTROYED by the force-FREE loop below
				// (ss9) = DATA LOSS. Cap the available new-data frame count so the whole unit
				// always fits the mixbatch. retransmit_count==0 (or !sack_v2) => avail ==
				// data_batch_size => byte-identical to pre-fix. Env MERCURY_MIXBATCH_OVERPOP_
				// DEFEAT restores the over-pop (fail-before arm), the no-comp leg's toggle.
				bool mixfill_overpop_defeat = false;
				{ const char* e = std::getenv("MERCURY_MIXBATCH_OVERPOP_DEFEAT");
				  if(e && *e && atoi(e)!=0) mixfill_overpop_defeat = true; }
				int avail_newdata_frames = data_batch_size;
				if(sack_v2_enabled && !mixfill_overpop_defeat)
				{
					int retx_prefix = retransmit_count;
					if(retx_prefix > data_batch_size) retx_prefix = data_batch_size;
					if(retx_prefix < 0) retx_prefix = 0;
					avail_newdata_frames = data_batch_size - retx_prefix;
					if(avail_newdata_frames < 0) avail_newdata_frames = 0;
				}
				int crypto_frames = avail_newdata_frames;
				if(cipher_suite.is_active()
				   && crypto_batch_size > 0
				   && crypto_batch_size < crypto_frames)
					crypto_frames = crypto_batch_size;
				int batch_capacity = crypto_frames * max_frame;

				// Reserve space for encryption auth tag if active
				int crypto_overhead = 0;
				if(cipher_suite.is_active())
					crypto_overhead = AUTH_TAG_SIZE;
				batch_capacity -= crypto_overhead;

				// Option W CORE (data-flow-stream-offset.md §8.1): reserve W_EOB_RESERVE so
				// the EOB (last) chunk, capped in the frame-split below, still fits within
				// crypto_frames frames. Capacity of crypto_frames frames with the last one
				// reserved = crypto_frames*max_frame - W_EOB_RESERVE, so bounding comp_size
				// by this guarantees the reserved split never needs an extra frame.
				if(w_stamp_rides())
					batch_capacity -= W_EOB_RESERVE;
				// The compressor workspace and 16-bit original-size field are the
				// hard unit ceiling. Some maximum SACK geometries exceed the old
				// 16 KB local output array by 25-32 bytes; advertise only storage
				// that actually exists.
				if(batch_capacity > COMPRESS_WORKSPACE_SIZE)
					batch_capacity = COMPRESS_WORKSPACE_SIZE;

				// Initial guess based on the running ratio estimate. Target the
				// existing 85% fill policy against PAYLOAD capacity, not total
				// capacity: at an accurate estimate, C * ratio compresses to C,
				// then the streaming header makes the result C+header and guarantees
				// a no-fit. The headroom also absorbs local ratio variation.
				// Staging can be up to COMPRESS_WORKSPACE_SIZE (64KB) because
				// high-ratio compressors (zstd/PPMd) can shrink 33KB+ to <1.5KB.
				const int staging_max = 65535;  // Header orig_size is uint16; cap to prevent overflow
				int chdr_size = compressor.get_header_size();
				int payload_capacity = batch_capacity - chdr_size;
				double estimated_pop = (double)payload_capacity
					* (double)compress_ratio_estimate * 0.85;
				int initial_pop = estimated_pop >= (double)staging_max
					? staging_max : (int)estimated_pop;
				if(initial_pop > staging_max) initial_pop = staging_max;
				if(initial_pop < payload_capacity)
					initial_pop = payload_capacity;

				char staging[COMPRESS_WORKSPACE_SIZE];
				// MIXBATCH FILL FIX (ii) -- force-FREE data-loss guard
				// (data-flow-mixbatch-fill.md ss9). A compressed unit is laid onto
				// messages_tx starting at slot 0 (add_message_tx_data first-free) and the
				// force-FREE loop below clears slots 0..data_batch_size-1 to guarantee
				// contiguous ids. If messages_tx ALREADY holds a staged-but-unsent
				// (ADDED_TO_LIST) or in-flight (ADDED_TO_BATCH_BUFFER) new-data frame -- the
				// v2 mixbatch relaxation (stage_ok while block_under_tx==YES with retx
				// pending) can re-enter this leg mid-batch -- force-FREEing it DESTROYS
				// content whose bytes were already popped from fifo_buffer_tx =
				// unrecoverable DATA LOSS, and laying a 2nd atomic unit over the 1st
				// corrupts both. SKIP staging this tick (no FIFO pop, no free): the batch
				// clears (ACK -> FREE) and the next fresh batch stages the pending FIFO
				// data. retx==0 / no-mixbatch never re-enters with live new-data
				// (block_under_tx==NO => all slots FREE) => byte-identical. Env DEFEAT keeps
				// the old destructive path (fail-before arm).
				bool live_newdata_staged = false;
				if(!mixfill_overpop_defeat)
					for(int _i=0; _i<nMessages; _i++)
						if(messages_tx[_i].status==ADDED_TO_LIST
						   || messages_tx[_i].status==ADDED_TO_BATCH_BUFFER)
						{ live_newdata_staged = true; break; }
				int raw_size = live_newdata_staged ? 0 : fifo_buffer_tx.pop(staging, initial_pop);
				if(raw_size == 0)
				{
					last_transmission_block_stats.nSent_data=0;
					last_transmission_block_stats.nReSent_data=0;
				}
				else
				{
					char comp_buf[COMPRESS_WORKSPACE_SIZE];
					static_assert(sizeof(comp_buf) >= COMPRESS_WORKSPACE_SIZE,
						"compression output buffer must cover advertised capacity");
					int comp_size = 0;
					bool compress_ok = false;

					// Adaptive fill loop: compress, check fill ratio, add more.
					// A SUCCESSFUL streaming PPMd attempt is never recompressed: it
					// advanced the live model and RX will decode only that result.
					// A NO-FIT attempt is different: compress_block() either rolls a
					// carried PPMd model back to its pre-probe checkpoint or resets a
					// non-checkpointed model. We may therefore reduce the unsent prefix
					// and retry without desynchronizing the receiver.
					bool compress_retry_defeat = false;
					{ const char* e = std::getenv("MERCURY_COMPRESS_RETRY_DEFEAT");
					  if(e && *e && atoi(e) != 0) compress_retry_defeat = true; }
					int max_iter = compressor.is_streaming()
						? (compress_retry_defeat ? 1 : 16) : 4;
					for(int iter = 0; iter < max_iter; iter++)
					{
						comp_size = compressor.compress_block(
							staging, raw_size, comp_buf, batch_capacity);

						if(comp_size > 0)
						{
							// Compression succeeded — check fill ratio
							float fill_ratio = (float)comp_size / (float)batch_capacity;
							if(compressor.is_streaming()
							   || fill_ratio >= 0.85f || iter == max_iter - 1)
							{
								compress_ok = true;
								break;  // Good enough
							}
							// Under-filled: estimate how much more raw data to add
							int comp_payload = comp_size - chdr_size;
							if(comp_payload <= 0) { compress_ok = true; break; }
							float current_ratio = (float)raw_size / (float)comp_payload;
							int remaining_comp = batch_capacity - comp_size;
							int more_raw = (int)(remaining_comp * current_ratio);
							if(more_raw < 1) { compress_ok = true; break; }
							if(raw_size + more_raw > staging_max)
								more_raw = staging_max - raw_size;
							if(more_raw <= 0) { compress_ok = true; break; }

							int got = fifo_buffer_tx.pop(staging + raw_size, more_raw);
							if(got == 0) { compress_ok = true; break; }  // FIFO empty
							raw_size += got;
							// Loop back to compress with more data
						}
						else if(comp_size == -1)
						{
							// Doesn't fit: push back excess and retry the smaller
							// unsent prefix. compress_block() either rolled a warm
							// PPMd model back transactionally or reset it safely.
							if(compressor.is_streaming())
							{
								// Push back 40%; the codec has made its model safe to retry.
								int pushback = raw_size * 2 / 5;
								if(pushback < 1) pushback = 1;
								fifo_buffer_tx.push_front(
									staging + raw_size - pushback, pushback);
								raw_size -= pushback;
								// No-fit directly proves this estimate was too high.
								// Back it off now; a successful retry updates the EMA.
								if(!compress_retry_defeat && raw_size > 0
								   && iter + 1 < max_iter)
								{
									compress_ratio_estimate *= 0.6f;
									if(compress_ratio_estimate < 1.0f)
										compress_ratio_estimate = 1.0f;
									printf("[COMPRESS-TX] no-fit retry: raw=%d estimate=%.2f attempt=%d\n",
										raw_size, compress_ratio_estimate, iter + 1);
									fflush(stdout);
									continue;
								}
								// The manual RAW fallback below carries no streaming/dict
								// flags, so reset TX to mirror RX's fresh-frame recovery
								// when retries are disabled or exhausted.
								compressor.streaming_reset();
								break;
							}
							int pushback = raw_size * 2 / 5;
							if(pushback < 1) pushback = 1;
							fifo_buffer_tx.push_front(
								staging + raw_size - pushback, pushback);
							raw_size -= pushback;
							if(raw_size < 1) break;
							// Loop back to retry compression
						}
						else
						{
							// Error (0) — fall through to raw
							break;
						}
					}

					if(compress_ok && comp_size > 0)
					{
						// Update running ratio estimate (EMA)
						int comp_payload = comp_size - chdr_size;
						if(comp_payload > 0)
						{
							float measured = (float)raw_size / (float)comp_payload;
							compress_ratio_estimate = 0.7f * compress_ratio_estimate + 0.3f * measured;
						}
						fifo_buffer_backup.push(staging, raw_size);
						batch_uncompressed_size = raw_size;
						// Streaming: save raw data pending ACK confirmation
						if(compressor.is_streaming())
							compressor.set_pending_raw((unsigned char*)staging, raw_size);
#ifdef MERCURY_GUI_ENABLED
						// Monitor tap: plaintext before compression
						gui_push_monitor_text(staging, raw_size, true);
						// Push algo to GUI (read from compressed header byte 0).
						// Mask off the dict-version bits (5-7) so the algo nibble (bits 0-1)
						// is not mislabeled as RAW when a dict-primed frame stamps a version.
						g_gui_state.compression_algo.store((int)((unsigned char)comp_buf[0] & COMPRESS_ALGO_MASK));
#endif
					}
					else
					{
						// compress_block() error fallback — wrap raw data in ALGO_RAW.
						// Cap to batch capacity minus header.
						int hdr_sz = compressor.get_header_size();
						int max_raw = batch_capacity - hdr_sz;
						if(raw_size > max_raw)
						{
							fifo_buffer_tx.push_front(
								staging + max_raw,
								raw_size - max_raw);
							raw_size = max_raw;
						}
						comp_buf[0] = COMPRESS_ALGO_RAW;
						comp_buf[1] = (char)(raw_size & 0xFF);
						comp_buf[2] = (char)((raw_size >> 8) & 0xFF);
						comp_buf[3] = (char)(raw_size & 0xFF);
						comp_buf[4] = (char)((raw_size >> 8) & 0xFF);
						if(compressor.is_streaming())
						{
							// CRC16 for streaming desync detection
							uint16_t crc = 0xFFFF;
							for(int j = 0; j < raw_size; j++)
							{
								crc ^= (unsigned char)staging[j];
								for(int b = 0; b < 8; b++)
									crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
							}
							comp_buf[5] = (char)(crc & 0xFF);
							comp_buf[6] = (char)((crc >> 8) & 0xFF);
						}
						memcpy(comp_buf + hdr_sz, staging, raw_size);
						comp_size = hdr_sz + raw_size;
						fifo_buffer_backup.push(staging, raw_size);
						batch_uncompressed_size = raw_size;
						// Streaming: save raw data pending ACK confirmation
						if(compressor.is_streaming())
							compressor.set_pending_raw((unsigned char*)staging, raw_size);
#ifdef MERCURY_GUI_ENABLED
						// Monitor tap: plaintext (RAW fallback path)
						gui_push_monitor_text(staging, raw_size, true);
#endif
					}

					// --- Encrypt batch (after compression, before frame split) ---
					if(cipher_suite.is_active() && comp_size > 0)
					{
						// Always use full 16-byte auth tag — encryption only runs
						// after turboshift at OFDM speeds where 16 bytes is negligible.
						int tag_size = AUTH_TAG_SIZE;

						// Pad compressed data so the encrypted output fills WHOLE
						// frames up to the bounded crypto-unit span (crypto_frames =
						// min(data_batch_size, crypto_batch_size); see the batch_capacity
						// fix above). MUST match the batch_capacity cap so the AEAD unit
						// never re-expands past crypto_batch_size frames here — otherwise
						// the cap is defeated and the unit re-spans data_batch_size frames
						// (data-flow-encrypted-batch-size.md §5). The RX no longer needs a
						// full data_batch_size span: the wired D5 batch_total_frames count
						// (arq_common.cc:8892) is the authoritative frame count the ACK gate
						// keys off, so a sub-data_batch_size encrypted batch is gated
						// correctly. Receiver decrypts, reads comp_size from the header, and
						// ignores the zero padding beyond it.
						int target_comp = crypto_frames * max_frame - tag_size;
						if(comp_size < target_comp && target_comp <= (int)sizeof(comp_buf))
						{
							memset(comp_buf + comp_size, 0, target_comp - comp_size);
							comp_size = target_comp;
						}

						uint32_t tx_direction = (original_role == COMMANDER)
							? DIRECTION_CMD_TO_RSP : DIRECTION_RSP_TO_CMD;
						// AEAD nonce binds to the WIRE batch_seq_id this batch carries
						// (cmd_batch_seq_id, CURRENT pre-increment value; the +1 at :2464 is
						// post-send and the per-frame stamp at :2216 uses this same value),
						// UNWRAPPED to a monotone index then folded with the re-seal generation
						// (see the RE-SEAL guard below). A SACK retx replays the STORED
						// ciphertext verbatim (it does not re-enter this path).
						int tx_wire_bsi = cmd_batch_seq_id & 0xFF;
						// RE-SEAL NONCE-REUSE GUARANTEE (data-flow-aead-nonce.md §11): the
						// §5-inv4 claim "a rebuild always re-queues fresh -> NEW bsi -> NEW
						// nonce" was FALSE on the compression+encryption recovery paths. A
						// BREAK/config-rebuild/demote FREEs the built-but-unsent batch and
						// re-queues PLAINTEXT WITHOUT advancing cmd_batch_seq_id (the +1 is
						// post-send, a later tick), and the wire bsi MUST roll back to the
						// in-flight value for RSP delivery contiguity (delivery_step_is_gap) -
						// so a pure bsi-bound nonce would re-seal the SAME index over
						// re-compressed plaintext = KEYSTREAM REUSE. The per-direction
						// GENERATION counter (tx_nonce_gen, bumped per recovery re-queue in
						// restore_tx_from_compressed) is folded into the index HIGH bits so a
						// re-seal lands in a strictly-higher, DISJOINT band at the same bsi.
						// HARD BACKSTOP (safety independent of gen alignment):
						// tx_nonce_sealed_high_water is the highest index ever sealed; if the
						// candidate is NOT strictly above it (the reuse condition), bump
						// tx_nonce_gen until it clears the high-water BEFORE sealing. A nonce is
						// NEVER reused even if the gen bookkeeping drifts - a wrong gen on RX
						// only auth-fails (safe drop/retx), never a reuse (RFC-4303 ESN /
						// RFC-9001 §5.4: implicit high-order seq bits; wrong guess fails tag).
						uint64_t tx_unwrap = cl_cipher_suite::unwrap_batch_index(
							tx_wire_bsi, &tx_nonce_epoch, &tx_nonce_last_bsi);
						uint64_t tx_batch_index =
							cl_cipher_suite::fold_gen_index(tx_nonce_gen, tx_unwrap);
						if(tx_nonce_sealed_high_water != UINT64_MAX
						   && tx_batch_index <= tx_nonce_sealed_high_water)
						{
							uint64_t old_gen = tx_nonce_gen;
							uint64_t hw_band = tx_nonce_sealed_high_water /
								cl_cipher_suite::NONCE_GEN_STRIDE;
							tx_nonce_gen = hw_band + 1;
							uint64_t old_idx = tx_batch_index;
							tx_batch_index = cl_cipher_suite::fold_gen_index(
								tx_nonce_gen, tx_unwrap);
							printf("[CRYPTO-TX] RE-SEAL GUARD: wire_bsi=%d index<=high_water "
								"(%llu<=%llu) - bumped gen %llu->%llu, new index=%llu\n",
								tx_wire_bsi,
								(unsigned long long)old_idx,
								(unsigned long long)tx_nonce_sealed_high_water,
								(unsigned long long)old_gen,
								(unsigned long long)tx_nonce_gen,
								(unsigned long long)tx_batch_index);
							fflush(stdout);
						}
						tx_nonce_sealed_high_water = tx_batch_index;
						printf("[CRYPTO-TX] Encrypting %d bytes, wire_bsi=%d index=%llu dir=%u tag=%d config=%d\n",
							comp_size, tx_wire_bsi,
							(unsigned long long)tx_batch_index,
							tx_direction, tag_size, (int)data_configuration);
						fflush(stdout);
						char enc_buf[16384];
						int enc_size = cipher_suite.encrypt(
							(const uint8_t*)comp_buf, comp_size,
							(uint8_t*)enc_buf, sizeof(enc_buf),
							tx_batch_index, tx_direction,
							tag_size);
						if(enc_size > 0)
						{
							memcpy(comp_buf, enc_buf, enc_size);
							comp_size = enc_size;
						}
						else
						{
							printf("[CRYPTO] Encrypt failed (wire_bsi %d index %llu)\n",
								tx_wire_bsi, (unsigned long long)tx_batch_index);
							fflush(stdout);
							return;  // Fail closed: never frame the plaintext comp_buf.
						}
					}

					// With encryption, comp_buf is padded + encrypted to fill
					// all data_batch_size frames exactly (no duplicate IDs).
					// Without encryption, pad_messages_batch_tx() fills
					// remaining batch slots with duplicates.

					// Ensure contiguous IDs 0..data_batch_size-1 for the fresh compressed
					// unit (add_message_tx_data fills the first FREE slot). MIXBATCH FILL
					// FIX (ii): NEVER FREE a staged-but-unsent (ADDED_TO_LIST) or in-flight
					// (ADDED_TO_BATCH_BUFFER) new-data frame -- its bytes were already popped
					// from fifo_buffer_tx, so clearing it is unrecoverable DATA LOSS. The
					// live-newdata guard above guarantees this leg is only reached with no
					// such frame present; this is defense-in-depth. Env DEFEAT restores the
					// unconditional free (fail-before arm) so the regression can exhibit the
					// destroyed-frame loss.
					for(int i = 0; i < data_batch_size; i++)
					{
						if(messages_tx[i].status != FREE)
						{
							if(!mixfill_overpop_defeat
							   && (messages_tx[i].status==ADDED_TO_LIST
								   || messages_tx[i].status==ADDED_TO_BATCH_BUFFER))
								continue;
							messages_tx[i].status = FREE;
						}
					}

					// Split comp_buf into frames
					int pos = 0;
					int frame_num = 0;
					while(pos < comp_size)
					{
						int chunk = comp_size - pos;
						if(chunk > max_frame) chunk = max_frame;
						// Option W CORE (§8.1): reserve stamp room on the FINAL (EOB) chunk so
						// header+payload+stamp fits C. If the natural last chunk is too full,
						// shrink it (leaving a small remainder that becomes a new, smaller final
						// chunk — self-correcting). batch_capacity was reduced by W_EOB_RESERVE
						// above so this never overflows crypto_frames frames.
						if(w_stamp_rides() && pos + chunk == comp_size
						   && chunk > max_frame - W_EOB_RESERVE)
							chunk = max_frame - W_EOB_RESERVE;
						block_under_tx = YES;
						int result;
						if(chunk == max_frame)
							result = add_message_tx_data(DATA_LONG, chunk, comp_buf + pos);
						else
							result = add_message_tx_data(DATA_SHORT, chunk, comp_buf + pos);
						if(result != SUCCESSFUL)
						{
							printf("[COMPRESS-TX] ERROR: add_message_tx_data failed at frame %d (result=%d)\n",
								frame_num, result);
							fflush(stdout);
						}
						pos += chunk;
						frame_num++;
					}
					// Option W (§2.1): the compressed leg framed exactly comp_size transported
					// bytes (post-compress → crypto-pad → encrypt(+tag)) for this new-data batch.
					batch_committed_len = comp_size;
					// Option W STEP 3 (§8.6): fold the exact framed transported bytes (comp_buf,
					// the ciphertext/compressed stream — the SAME domain the RX folds from the
					// reassembled messages_rx[i].data, INV5) into the running stream CRC-32. Folded
					// as one blob here; the RX folds per-frame in slot order — CRC-32 is
					// chunk-independent so the running registers agree byte-for-byte.
					tx_stream_crc = crc32_update(tx_stream_crc, comp_buf, comp_size);
					printf("[COMPRESS-TX] %d raw -> %d comp (%d frames), ratio_est=%.2f, fill=%.0f%%\n",
						raw_size, comp_size, frame_num, compress_ratio_estimate,
						100.0f * comp_size / batch_capacity);
					fflush(stdout);
				}
				mtl::log_event_kv("cmd_tx_compress_done",
					"cpu_ms=%lld", mtl::now_ms() - _comp_t0_ms);
			}
			else
			{
				// --- No compression (or compression not viable for this batch):
				// original per-frame loop, headerless raw frames. ---
				// Reached when compression is OFF, OR when it is ENABLED but the
				// per-batch budget can't hold the streaming header (robust /
				// tiny-frame; see compression_viable_for_batch()). When streaming
				// is armed but bypassed for this batch, freeze the model: drop any
				// pending raw a prior OFDM fill may have staged so the next data
				// ACK's commit_pending() can't mis-commit it (data-flow-compress-
				// frame-fill.md §6.1). No-op when nothing is pending / not streaming.
				if(compression_enabled && compressor.is_streaming())
					compressor.clear_pending();
				int filled = 0;
				// MIXBATCH FILL FIX (data-flow-mixbatch-fill.md): on a v2 mixed batch the
				// NEXT TX prepends R = min(retransmit_count, data_batch_size) retx frames
				// as the batch's FIRST slots (process_messages_tx_data v2 retx-prefix loop,
				// arq_commander.cc ~2414-2461), leaving only (data_batch_size - R) slots for
				// new data. Popping the full data_batch_size NEW frames here over-stages by
				// R: the surplus frames stay ADDED_TO_LIST and, because add_message_tx_data()
				// fills the first-FREE slot, they land in HIGH messages_tx slots while the
				// NEXT batch's fresh frames fill the LOW (freed) slots -> the new-data assign
				// loop (which numbers pos/id by ASCENDING slot) gives the OLDER surplus
				// content a HIGHER seq/id than the newer content -> the batch is delivered
				// OUT OF ORDER (byte-integrity FAIL, rx>tx double-delivery). Cap the fill to
				// the slots the mixbatch can actually carry so NO surplus is ever created
				// (the 'messages_tx slot order == content order' invariant the assign loop
				// relies on is preserved). No-retx / v1 path: retransmit_count == 0 ->
				// fill_limit == data_batch_size (byte-identical to pre-fix). Env DEFEAT
				// restores the pre-fix over-pop on the SAME binary (the fail-before arm).
				int fill_limit = data_batch_size;
				bool slotcap_defeat = false;
				{ const char* e = std::getenv("MERCURY_MIXBATCH_SLOTCAP_DEFEAT");
				  if(e && *e && atoi(e)!=0) slotcap_defeat = true; }
				if(sack_v2_enabled)
				{
					bool overpop_defeat = false;
					{ const char* e = std::getenv("MERCURY_MIXBATCH_OVERPOP_DEFEAT");
					  if(e && *e && atoi(e)!=0) overpop_defeat = true; }
					if(!overpop_defeat)
					{
						int retx_prefix = retransmit_count;
						if(retx_prefix > data_batch_size) retx_prefix = data_batch_size;
						if(retx_prefix < 0) retx_prefix = 0;
						int already_staged = 0;
						for(int _i=0; _i<nMessages; _i++)
							if(messages_tx[_i].status==ADDED_TO_LIST) already_staged++;
						fill_limit = data_batch_size - retx_prefix - already_staged;
						if(fill_limit < 0) fill_limit = 0;
					}
				}
				// Batch arithmetic cannot reserve slots that are still occupied by
				// a prior in-flight batch. Use the exact FREE predicate consumed by
				// add_message_tx_data() before removing any source bytes.
				if(!slotcap_defeat)
				{
					int free_slots = get_nFree_messages();
					if(fill_limit > free_slots) fill_limit = free_slots;
				}
				batch_uncompressed_size = 0;
				for(int i=0;i<fill_limit;i++)
				{
					// KX-AS-DATA forward source-switch (data-flow-hybrid-kex.md §2.2).
					// Feed the reserved forward KX stream from its dedicated buffer, NOT
					// fifo_buffer_tx (user data stays untouched + held). KX batches carry
					// NO Option-W wire stamp (w_stamp_rides() is false in the KX epoch) and
					// do NOT advance the Option-W cursor / stream CRC / backup FIFO —
					// KEY_ACTIVATE re-anchors delivery to a fresh bsi=0, and the RX routes
					// these frames to kx_ingest() before any stamp/cursor logic. They are
					// otherwise ordinary DATA frames, so the whole windowing / SACK /
					// retransmit engine carries them (dissolving the pattern-ACK wall).
					if(kx_stream_tx_active)
					{
						int kx_remain = kx_stream_tx_len - kx_stream_tx_sent;
						if(kx_remain <= 0){ kx_stream_tx_active = false; break; }
						data_read_size = (kx_remain < max_frame) ? kx_remain : max_frame;
						memcpy(message_TxRx_byte_buffer,
						       kx_stream_tx_buf + kx_stream_tx_sent, data_read_size);
						int add_result;
						if(data_read_size==max_frame)
							add_result = add_message_tx_data(DATA_LONG, data_read_size, message_TxRx_byte_buffer);
						else
							add_result = add_message_tx_data(DATA_SHORT, data_read_size, message_TxRx_byte_buffer);
						if(add_result != SUCCESSFUL && !slotcap_defeat)
						{
							printf("[KX-TX] ERROR: no message slot for staged KX data (result=%d)\n",
								add_result);
							fflush(stdout);
							break;
						}
						kx_stream_tx_sent += data_read_size;
						block_under_tx = YES;
						filled++;
						if(kx_stream_tx_sent >= kx_stream_tx_len)
						{
							kx_stream_tx_active = false;   // whole forward stream staged
							break;
						}
						continue;
					}
					// Option W CORE (data-flow-stream-offset.md §8.1): reserve W_EOB_RESERVE
					// payload bytes on the EOB (last) frame of the batch so its
					// header+payload+stamp fits the codeword C. The last frame is the one at
					// the batch-frame cap (i==fill_limit-1) OR the pop that drains the fifo
					// (used_before<=max_frame). Capping its pop below max_frame forces a
					// DATA_SHORT with room; any leftover simply rides the next (smaller) final
					// frame (self-correcting). Non-EOB frames are UNCHANGED (full max_frame).
					int pop_budget = max_frame;
					if(w_stamp_rides())
					{
						int used_before = fifo_buffer_tx.get_size() - fifo_buffer_tx.get_free_size();
						bool last_by_cap   = (i == fill_limit - 1);
						bool last_by_drain = (used_before <= max_frame);
						if(last_by_cap || last_by_drain)
							pop_budget = max_frame - W_EOB_RESERVE;   // > 0 (w_stamp_rides ⇒ max_frame ≥ 15)
					}
					data_read_size=fifo_buffer_tx.pop(message_TxRx_byte_buffer, pop_budget);
					if(data_read_size==0)
					{
						last_transmission_block_stats.nSent_data=0;
						last_transmission_block_stats.nReSent_data=0;
						break;
					}
					int add_result;
					if(data_read_size==max_frame)
						add_result = add_message_tx_data(DATA_LONG, data_read_size, message_TxRx_byte_buffer);
					else
						add_result = add_message_tx_data(DATA_SHORT, data_read_size, message_TxRx_byte_buffer);
					if(add_result != SUCCESSFUL && !slotcap_defeat)
					{
						// Preflight and enqueue use the same FREE predicate. Restore an
						// unexpectedly rejected frame before changing any commit state.
						int restored = fifo_buffer_tx.push_front(message_TxRx_byte_buffer, data_read_size);
						printf("[TX-STAGE] ERROR: add_message_tx_data failed (result=%d restored=%d/%d)\n",
							add_result, restored, data_read_size);
						fflush(stdout);
						if(restored != data_read_size)
						{
							printf("[TX-STAGE] FATAL: source restore failed; dropping link\n");
							fflush(stdout);
							link_status = DROPPED;
						}
						break;
					}
					fifo_buffer_backup.push(message_TxRx_byte_buffer, data_read_size);
					batch_uncompressed_size += data_read_size;
					// Option W (§2.1): raw leg — frame payload IS the transported payload
					// (headerless, no compression), so accumulate the committed length.
					batch_committed_len += data_read_size;
					// Option W STEP 3 (§8.6): fold this frame's raw transported bytes into the
					// running stream CRC-32, in pop (== stream) order. The RX folds the identical
					// bytes (messages_rx[i].data) in the same order at copy_data_to_buffer, so the
					// registers agree. (These are the exact bytes the 4 captured WGN:25 mechanisms
					// corrupt — compression OFF ⇒ transported == app bytes.)
					tx_stream_crc = crc32_update(tx_stream_crc, message_TxRx_byte_buffer, data_read_size);
#ifdef MERCURY_GUI_ENABLED
					// Monitor tap: plaintext (no compression path)
					gui_push_monitor_text(message_TxRx_byte_buffer, data_read_size, true);
#endif
					block_under_tx=YES;
					filled++;
				}
			}
			// Option W (data-flow-stream-offset.md §2.1) — THE build-commit latch. When new
			// transported bytes were framed this call (either leg), record the batch's
			// {start,length} under the bsi it will carry on the wire (cmd_batch_seq_id, not yet
			// advanced — the +1 is post-send at :2591) and advance the cursor. Fires once per
			// (bsi, build-generation): the re-entry guards keep batch_committed_len==0 on a
			// re-entry that stages nothing. NEVER keyed off frame/expected counts (false-fire
			// rule) — only the LATCHED committed byte length.
			if(batch_committed_len > 0)
			{
				stream_tx_latch(cmd_batch_seq_id & 0xFF, (uint32_t)batch_committed_len);
				// Option W STEP 3 (§8.6): store the TRUE pre-fold CRC anchor (the latch set a
				// benign default = the post-fold running crc). This is the value a re-stage
				// rollback restores tx_stream_crc to, so a rebuild re-folds the rebuilt bytes
				// from the correct origin (INV4 for the CRC) — mirrors .start exactly.
				tx_stream_stamp[cmd_batch_seq_id & 0xFF].crc = batch_crc_anchor;
			}
		}
		else if(block_under_tx==YES && message_batch_counter_tx==0 && get_nOccupied_messages()==0 && messages_control.status==FREE)
		{
			// BLOCK_END eliminated: pattern ACK / silence is sole flow control.
			// Commander finalizes immediately after all data frames ACKed.
			// Responder flushes data to app after sending pattern ACK.
			// Downshift on decode failure handled by emergency BREAK (threshold=2).
			finalize_block_commander();
		}
		else if(block_under_tx==NO && message_batch_counter_tx==0 && get_nOccupied_messages()==0 && messages_control.status==FREE)
		{
			// KX-as-data role-swap / activation handshake (data-flow-hybrid-kex.md §7). The
			// forward/reverse KX batch is fully delivered + ACKed and nothing is in flight
			// (this branch's exact precondition), so drive the two R1 SWITCH_ROLE swaps +
			// KEY_ACTIVATE. Three mutually-exclusive states keyed on kx_phase +
			// kx_reverse_consumed:
			//   KX_MLKEM_PK_SENT (only the CMD)            -> SWITCH_ROLE (swap #1: RSP streams ct)
			//   KX_HYBRID_DONE & !consumed (only the RSP)  -> SWITCH_ROLE (swap #2: hand role back)
			//   KX_HYBRID_DONE &  consumed (only the CMD)  -> KEY_ACTIVATE (activate)
			// The idle-switchrole re-ride race is cleared by set_role()'s
			// session_data_frame_sent reset (arq_common.cc:1669) so neither earned swap fires
			// an empty handoff; an "awaiting the peer" phase simply holds. Env off ->
			// kx_as_data_path() false -> the generic handoff below is byte-identical.
			if(encryption_enabled && !cipher_suite.is_active() && kx_as_data_path())
			{
				int ph = cipher_suite.get_kx_phase();
				if(ph == KX_MLKEM_PK_SENT && !kx_role_swap_sent
				   && reverse_data_demand_acknowledged)
				{
					kx_role_swap_sent = true;
					printf("[CRYPTO] KX-as-data: forward pk delivered -> SWITCH_ROLE (RSP streams reverse ct)\n");
					fflush(stdout);
					add_message_control(SWITCH_ROLE);
				}
				else if(ph == KX_HYBRID_DONE && !kx_reverse_consumed && !kx_role_swap_sent
				   && reverse_data_demand_acknowledged)
				{
					kx_role_swap_sent = true;
					printf("[CRYPTO] KX-as-data: reverse ct delivered -> SWITCH_ROLE (hand role back to CMD)\n");
					fflush(stdout);
					add_message_control(SWITCH_ROLE);
				}
				else if(ph == KX_HYBRID_DONE && kx_reverse_consumed && !kx_key_activate_sent)
				{
					kx_key_activate_sent = true;
					printf("[CRYPTO] KX-as-data: reverse ct decapsulated -> sending KEY_ACTIVATE\n");
					fflush(stdout);
					add_message_control(KEY_ACTIVATE);
				}
			}
			else
			{
			// Demand-driven role-transfer trigger.  The existing timer/send/ACK/
			// promotion machinery is retained, but it may arm only after a reverse
			// ACK/control record has passed its integrity and ownership checks with
			// D=1.  The timer controls latency only; it creates no authority.
#ifdef ROLE_DEMAND_FAILBEFORE
			// Directed fail-before arm: restore the speculative post-data trigger.
			if(session_data_frame_sent)
#else
			if(reverse_data_demand_acknowledged)
#endif
			{
				if(switch_role_timer.counting==NO)
				{
					switch_role_timer.reset();
					switch_role_timer.start();
				}
				else if(switch_role_timer.get_elapsed_time_ms()>switch_role_timeout)
				{
					switch_role_timer.stop();
					switch_role_timer.reset();
					add_message_control(SWITCH_ROLE);
				}
			}
			}
		}
	}
}
