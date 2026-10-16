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
#include "common/timing_log.h"
#include "physical_layer/mfsk_ctrl_codec.h"  // Stage 2 config-tag follow test
#include <vector>
#include <random>   // test_recovery_ack_capture: written-but-silent ring noise (ARM3b)
#if !defined(_WIN32)
#include <cerrno>
#include <csignal>
#include <sys/wait.h>
#endif

#ifdef MERCURY_GUI_ENABLED
#include "gui/gui_state.h"
#endif

// RECEIVED slots found outside the clean ACK gate's effective window (freed,
// never ACKed); printed with every [RSP-ACK-OOW-SLOT] line.
static long long g_rsp_ack_oow_slot_count = 0;

void cl_arq_controller::process_messages_responder()
{

	if(this->connection_status==ACKNOWLEDGING_CONTROL)
	{
		if(g_verbose) print_stats();
		process_messages_acknowledging_control();
	}
	else if(this->connection_status==ACKNOWLEDGING_DATA)
	{
		if(g_verbose) print_stats();
		process_messages_acknowledging_data();
	}
	else if(this->connection_status==RECEIVING)
	{
		process_messages_rx_data_control();
	}

}

void cl_arq_controller::responder_maybe_advertise_late_role_demand()
{
	if(!local_reverse_data_demand())
	{
		reverse_data_demand_last_advertised = false;
		role_demand_refresh_timer.stop();
		role_demand_refresh_timer.reset();
		return;
	}
	if(role != RESPONDER || link_status != CONNECTED
	   || connection_status != RECEIVING || passive_monitor
	   || messages_control.status != FREE || messages_rx_buffer.status == RECEIVED
	   || batch_rx_frame_count != 0)
		return;
	if(role_demand_refresh_timer.counting == NO)
	{
		role_demand_refresh_timer.reset();
		role_demand_refresh_timer.start();
		return;
	}
	if(role_demand_refresh_timer.get_elapsed_time_ms() < ROLE_DEMAND_REFRESH_MS)
		return;

	int span = data_batch_size;
	if(span < 1) span = 1;
	if(span > MAX_SACK_BATCH_SIZE) span = MAX_SACK_BATCH_SIZE;
	bool clean_bitmap[MAX_SACK_BATCH_SIZE];
	for(int i=0; i<span; i++) clean_bitmap[i] = true;
	unsigned char target = (unsigned char)(
		rsp_prev_batch_seq_id >= 0 ? rsp_prev_batch_seq_id
		: (rsp_last_delivered_batch_seq_id >= 0 ? rsp_last_delivered_batch_seq_id : 0));
	printf("[ROLE-DEMAND-TX] late/refresh target_bsi=%u span=%d count=%lld\n",
		(unsigned)target, span, reverse_data_demand_tx_count + 1);
	fflush(stdout);
	send_sack_v2_frame(clean_bitmap, span, target);
}

int cl_arq_controller::add_message_rx_data(char type, char id, int length, char* data)
{
	int success=ERROR_;
	int loc=(int)((unsigned char)id);
	// Option B' (data-flow-batch-size.md §9): bound the store slot by the effective
	// RX acceptance window, NOT the stale command-synced data_batch_size. The window
	// widens ONLY when this batch's CRC-protected D5 count (the current frame's staged
	// count, or the already-latched authoritative count) declares MORE frames than
	// data_batch_size — the res_c3100 CMD>RSP case — in which case frames with
	// seq in [data_batch_size, D5) MUST be stored or nothing downstream can deliver
	// them. Byte-identical (== data_batch_size) in every non-desync / non-v2 case, and
	// under MERCURY_BPRIME_DEFEAT. Hard-bounded by nMessages (messages_rx[] alloc).
	int sender_cnt = (rx_batch_total_frames > rx_buffer_batch_total_frames)
	                 ? rx_batch_total_frames : rx_buffer_batch_total_frames;
	int store_win  = rx_effective_window(sender_cnt);
	if(store_win > nMessages) store_win = nMessages;
	if(loc >= store_win || loc < 0)
	{
		success = MESSAGE_ID_ERROR;
		return success;
	}

	if(length<0)
	{
		success=MESSAGE_LENGTH_ERROR;
		return success;
	}

	// SACK Design A Step 1 — DATA_LONG payload capacity is reduced by 1 byte
	// when sack_v2_enabled. v1 path uses the legacy 4-byte macro; bytes-on-the
	// -wire and bounds are identical to pre-Step-1.
	if(type==DATA_LONG && length>(max_data_length+max_header_length-effective_data_long_header_length(sack_v2_enabled, header_carries_d5)))
	{
		success=MESSAGE_LENGTH_ERROR;
		return success;
	}

	// SACK Design A Step 2 — DATA_SHORT payload capacity is reduced by 1 byte
	// when sack_v2_enabled. v1 path uses the legacy 5-byte macro; bytes-on-the
	// -wire and bounds are identical to pre-Step-2.
	if(type==DATA_SHORT && length>(max_data_length+max_header_length-effective_data_short_header_length(sack_v2_enabled, header_carries_d5)))
	{
		success=MESSAGE_LENGTH_ERROR;
		return success;
	}

	messages_rx[loc].type=type;
	messages_rx[loc].length=length;
	for(int j=0;j<messages_rx[loc].length;j++)
	{
		messages_rx[loc].data[j]=data[j];
	}
	{
		// SACK Design A Step 1 — zero-pad up to effective DATA_LONG payload size.
		int fill_end = max_data_length+max_header_length-effective_data_long_header_length(sack_v2_enabled, header_carries_d5);
		if(fill_end > N_MAX/8) fill_end = N_MAX/8;
		for(int j=messages_rx[loc].length;j<fill_end;j++)
		{
			messages_rx[loc].data[j]=0;
		}
	}
	// Candidate semantics: persistent emptiness means no accepted CRC-valid DATA
	// activity at all.  A duplicate which overwrites a RECEIVED partial slot is
	// not unique byte progress, but it still proves the forward RF path is active;
	// therefore it must break the continuous-empty interval just like a new slot.
	if(scream_empty_holdoff_armed)
	{
		scream_empty_holdoff_armed = false;
		scream_empty_holdoff_timer.stop();
		scream_empty_holdoff_timer.reset();
		scream_empty_holdoff_ms = 0;
		printf("[SCREAM-HOLDOFF] CRC-valid DATA activity; persistent-empty candidate cleared\n");
		fflush(stdout);
	}
	if(messages_rx[loc].status==FREE || messages_rx[loc].status==ACKED)
	{
		stats.nReceived_data++;
		if(scream_reentry_recovery_pending)
		{
			scream_reentry_recovery_pending = false;
			printf("[SCREAM-REENTRY] first post-tag DATA frame stored cfg=%d; wake re-armed\n",
				current_configuration);
			fflush(stdout);
		}
		// IDLE-SWITCHROLE-RACE recovery (idle-switchrole-race.md §3): this session
		// has DELIVERED an RX data frame -> real forward progress. The BREAK
		// no-progress teardown discriminator (Part C) keys on this per-session bool
		// (NOT raw stats.nReceived_data, which is cumulative across sessions).
		session_data_frame_received = true;
	}
	messages_rx[loc].status=RECEIVED;
	success=SUCCESSFUL;
	return success;
}

// A prior duplicate-TEST_CONNECTION repair polled before the OFDM consumer and
// repeatedly clamped the shared frames_to_read countdown to 2, starving the
// first data batch.  The current replay path is deliberately after receive():
// it inspects only the already-staged failed OFDM snapshot and never changes the
// capture countdown while probing.


// SACK Design A Step 8a -- match-prev store (the REFILL), factored out of
// process_messages_rx_data_control so the reseat-span regression drives the
// PRODUCTION prev-match store (store -> honest completion count) instead of
// transcribing it. Reads messages_rx_buffer (the just-received prev-routed
// frame) + member state; writes messages_rx_prev[loc] and the honest count.
// Returns true iff a frame was stored (len_ok): the caller then runs the
// completion/delivery block, and on false logs [RSP-V2-PREV-DROP]
// length_or_loc_out_of_range -- byte-identical to the pre-refactor inline
// if(len_ok){...}else{...}.
bool cl_arq_controller::store_prev_match_frame()
{
	int loc = (int)((unsigned char)messages_rx_buffer.id);
	int eff_long  = effective_data_long_header_length(sack_v2_enabled, header_carries_d5);
	int eff_short = effective_data_short_header_length(sack_v2_enabled, header_carries_d5);
	int max_long  = max_data_length + max_header_length - eff_long;
	int max_short = max_data_length + max_header_length - eff_short;
	bool len_ok = true;
	if(messages_rx_buffer.type == DATA_LONG
	   && messages_rx_buffer.length > max_long) len_ok = false;
	if(messages_rx_buffer.type == DATA_SHORT
	   && messages_rx_buffer.length > max_short) len_ok = false;
	(void)eff_short;  // referenced via max_short above
	// R7 fix (data-flow-messages_rx_prev.md §5):
	// Constrain loc to [0, data_batch_size) — NOT [0, nMessages).
	// The prev-batch completion trigger at :513 fires when
	// rsp_prev_batch_received_count >= rsp_prev_batch_expected_count,
	// and expected_count is capped at data_batch_size by the bsi-bump
	// path (arq_common.cc:4072). A bit-errored ID in the range
	// [data_batch_size, nMessages) used to (a) write to a slot that
	// is never read by the swap-and-deliver loop (arq_responder.cc:537
	// iterates i<data_batch_size && i<nMessages) and (b) bump the
	// received_count, prematurely triggering completion when real
	// frames had not all arrived. Matches the new-data path bound
	// at arq_responder.cc:54 which uses data_batch_size.
	// Option B' (data-flow-batch-size.md §9): bound the prev slot by the
	// effective window derived from THIS prev frame's own CRC-protected D5
	// count (rx_buffer_batch_total_frames = the PREV batch's declared span),
	// so a prev batch the sender built larger than data_batch_size is not
	// truncated at the store gate. == data_batch_size in every non-desync
	// case. The nMessages check below stays the hard storage bound.
	int prev_store_win = rx_effective_window(rx_buffer_batch_total_frames);
	if(loc < 0 || loc >= prev_store_win)          len_ok = false;
	if(loc >= this->nMessages)                    len_ok = false;
	if(messages_rx_buffer.length < 0)             len_ok = false;
	if(!len_ok) return false;
	// Capture pre-store status so we can detect newly
	// RECEIVED slots (avoid double-counting on repeat
	// retransmits — RSP may see the same retx multiple
	// times if CMD couldn't decode the SACK_RSP).
	char prev_status = messages_rx_prev[loc].status;
	messages_rx_prev[loc].type   = messages_rx_buffer.type;
	messages_rx_prev[loc].length = messages_rx_buffer.length;
	for(int j=0; j<messages_rx_buffer.length; j++)
		messages_rx_prev[loc].data[j] = messages_rx_buffer.data[j];
	{
		int fill_end = max_long;
		if(fill_end > N_MAX/8) fill_end = N_MAX/8;
		for(int j=messages_rx_buffer.length; j<fill_end; j++)
			messages_rx_prev[loc].data[j] = 0;
	}
	messages_rx_prev[loc].status = RECEIVED;
	messages_rx_prev[loc].batch_seq_id = messages_rx_buffer.batch_seq_id;
	bool prev_was_fresh = (prev_status != RECEIVED && prev_status != ACKED);
	// D5: a prev-routed frame carries the PREV batch's authoritative
	// frame count. If the prev was armed with an EOB-INFERRED (too
	// short) expected_count because the original EOB frame was lost,
	// re-derive expected_count from the wired count the moment a
	// surviving frame of that batch reveals it — so the lost-EOB tail
	// is now INSIDE expected_count, the completion gate waits for it,
	// and the SACK span (below) requests it instead of silently
	// dropping it. Clamp to [received_count, data_batch_size]: never
	// shrink below what we already hold, never exceed the storage
	// bound. Gated off under MERCURY_D5_INFER_DEFEAT (fail-before).
	// Reseat-integrity ordering (data-flow-messages_rx_prev.md §4.5
	// CORRECTION): this re-derivation runs BEFORE the count bump below,
	// so a fresh frame that arrives at a loc the short EOB-inference had
	// excluded is credited within the just-GROWN delivery window (it
	// would otherwise be dropped by FIX (a) producer 2's loc<expected
	// gate on the lost-EOB grow path).
	if(rsp_prev_batch_active && rx_buffer_batch_total_frames > 0)
	{
		bool d5_infer_defeat = false;
		{ const char* e = std::getenv("MERCURY_D5_INFER_DEFEAT");
		  if(e && *e && atoi(e)!=0) d5_infer_defeat = true; }
		if(!d5_infer_defeat)
		{
			int wired = rx_buffer_batch_total_frames;
			// Option B' (§9): cap by eff_window (== data_batch_size unless
			// the sender over-declared this prev batch) rather than the stale
			// data_batch_size, so the prev-completion gate waits for the TRUE
			// span. Byte-identical when D5 <= data_batch_size.
			int wcap = rx_effective_window(rx_buffer_batch_total_frames);
			if(wired > wcap) wired = wcap;
			if(wired < rsp_prev_batch_received_count)
				wired = rsp_prev_batch_received_count;
			if(wired > rsp_prev_batch_expected_count)
			{
				printf("[RSP-V2-D5-PREVEXP] prev_batch_seq_id=%d "
					"expected %d -> %d (wired batch_total_frames=%d; "
					"EOB-inference was short)\n",
					rsp_prev_batch_seq_id,
					rsp_prev_batch_expected_count, wired,
					rx_buffer_batch_total_frames);
				fflush(stdout);
				rsp_prev_batch_expected_count = wired;
			}
		}
	}
	// FIX (a) producer 2 — honest completion count = DELIVERY window
	// (data-flow-messages_rx_prev.md §4.5 CORRECTION). Bump received_count
	// ONLY when this fresh slot is INSIDE the delivery window
	// [0, prev_expected). The store above already accepted loc over the
	// wider store window (rx_effective_window), so without this gate a
	// wire-id skew that lands a frame in the gap [prev_expected,
	// store_win) would over-credit the completion count and seal a
	// short-delivering "complete" batch. prev_expected is the true batch
	// span (the wired D5 count, just re-derived above), so this is
	// byte-identical whenever the store window equals the delivery window
	// (every non-short-fill case). MERCURY_W_PREVCOUNT_DEFEAT=1 restores
	// the store-window count (the fail-before arm on the SAME binary).
	{
		bool w_prevcount_defeat = false;
		{ const char* e = std::getenv("MERCURY_W_PREVCOUNT_DEFEAT");
		  if(e && *e && atoi(e)!=0) w_prevcount_defeat = true; }
		if(prev_was_fresh
		   && (w_prevcount_defeat || loc < rsp_prev_batch_expected_count))
			rsp_prev_batch_received_count++;
	}

	printf("[RSP-V2-PREV-RX] bsi=%d id=%d seq=%d/%d len=%d "
		"prev_received=%d/%d\n",
		(int)(unsigned char)messages_rx_buffer.batch_seq_id,
		(int)(unsigned char)messages_rx_buffer.id,
		messages_rx_buffer.sequence_number, data_batch_size,
		messages_rx_buffer.length,
		rsp_prev_batch_received_count,
		rsp_prev_batch_expected_count);
	fflush(stdout);
	return true;
}

void cl_arq_controller::process_messages_rx_data_control()
{
	long long demand_tx_before = reverse_data_demand_tx_count;
	responder_maybe_advertise_late_role_demand();
	if(reverse_data_demand_tx_count != demand_tx_before)
		return;
	// Fast HAIL scanning while LISTENING: use receive_hail_pattern() (~32ms cycles)
	// instead of slow receive() (multi-second LDPC frame captures).
	// Directed HAIL: only respond to beacons targeting our callsign (CRC suffix match).
	if(link_status == LISTENING && ack_pattern_time_ms > 0 && hail_detected == NO)
	{
		if(passive_monitor)
		{
			// Monitor: detect ALL HAILs (undirected), don't respond
			telecom_system->ack_mfsk.clear_hail_target();
		}
		else
		{
			// Set directed HAIL target to our own callsign (including SSID)
			telecom_system->ack_mfsk.set_hail_target(
				my_call_sign.c_str(), my_call_sign.length());
		}

		// Override large frames_to_read from LISTENING init — HAIL scanning
		// needs frames_to_read==0 to check the buffer. The audio callback
		// continuously fills the buffer regardless, so audio is always fresh.
		if(telecom_system->data_container.frames_to_read > 2)
		{
			MUTEX_LOCK(&capture_prep_mutex);
			telecom_system->data_container.frames_to_read = 2;
			MUTEX_UNLOCK(&capture_prep_mutex);
		}

		if(receive_hail_pattern())
		{
			printf("[HAIL] 'I am Mercury' beacon detected!\n");
			fflush(stdout);

			// PENDING at the EARLIEST address-matched detection. This HAIL was
			// suffix-matched to MYCALL (set_hail_target(my_call_sign) above, so a
			// beacon for anyone else never reaches this branch -- a non-MYCALL
			// hail is silence, not a false hold). Tell a scanner-control host to
			// stop scanning and prepare PTT now -- one control-frame ahead of the
			// START_CONNECTION decode -- so the scan-stop window matches the
			// address-directed HAIL, a token-timing the peer's undirected beacon
			// cannot express. Latched: single-shots PENDING and makes this site and
			// the START_CONNECTION crc-match site mutually exclusive (first wins).
			// A monitor uses an undirected target and never announces.
			if(!passive_monitor)
				rsp_emit_pending();

			// Wait for commander to finish TX before responding.
			// We detected HAIL mid-TX — remaining CMD symbols + flush could
			// overlap our response, causing truncated capture on CMD side.
			// Delay = (pattern_len - threshold) symbols + flush margin.
			{
				int sym_ms = (telecom_system->data_container.Nofdm
					* telecom_system->data_container.interpolation_rate * 1000) / 48000;
				int remaining_syms = telecom_system->ack_mfsk.hail_detect_nsymb
					- telecom_system->ack_mfsk.hail_detect_threshold;
				int delay_ms = remaining_syms * sym_ms + 200;
				printf("[HAIL] Waiting %d ms for commander TX to finish\n", delay_ms);
				fflush(stdout);
				// §5.7-B7 + CLOCK-FIDELITY FIX (fix/sim-connect-virtual-clock):
				// virtual-clock-ify ONLY (Bug #55 delay FORMULA above is verbatim).
				// This turnaround pause (RSP waits for the CMD's trailing HAIL TX to
				// finish before replying) must advance on the SAME virtual clock as
				// the CMD's hail_listen deadline, or under host CPU load the RSP
				// reply beacon lands outside the CMD window. pumped_settle_wait runs
				// a VIRTUAL-clock spin under -x sim (concurrent RX bridge advances
				// the shared sample clock), a step-pumped wait under SIM_INPROC, and
				// a verbatim wall msleep on production / HW (byte-identical).
				pumped_settle_wait(delay_ms);
			}
			// Respond with our own HAIL (suppressed in monitor mode)
			send_hail_pattern();
			if(!passive_monitor)
			{
				printf("[HAIL] Responded with beacon\n");
				fflush(stdout);
			}

			if(passive_monitor)
			{
				// Monitor: flush ring buffer so decode starts with fresh audio.
				// In normal mode, send_hail_pattern() does this after TX.
				// Without flush, old HAIL tones confuse MFSK preamble detection.
				int buf_samples = telecom_system->data_container.Nofdm
					* telecom_system->data_container.buffer_Nsymb
					* telecom_system->data_container.interpolation_rate;
				MUTEX_LOCK(&capture_prep_mutex);
				capture_reset_samples();
				memset(telecom_system->data_container.passband_delayed_data, 0,
					2 * buf_samples * sizeof(double));
				telecom_system->data_container.ring_write_index = 0;
				telecom_system->data_container.nUnder_processing_events = 0;
				telecom_system->receive_stats.mfsk_search_raw = 0;
				telecom_system->receive_stats.ofdm_search_raw = 0;
				telecom_system->receive_stats.ofdm_batch_active = false;
				telecom_system->receive_stats.delay_of_last_decoded_message = -1;
				MUTEX_UNLOCK(&capture_prep_mutex);
				printf("[MONITOR] Flushed capture buffer for clean decode\n");
				fflush(stdout);
			}

			hail_detected = YES;

			// Prepare for START_CONNECTION (generous timeout for commander turnaround)
			// Monitor needs extra time: must wait for real responder's HAIL + commander processing
			int hail_timeout = passive_monitor
				? 3 * message_transmission_time_ms + 10000
				: 2 * message_transmission_time_ms + 3000;
			set_receiving_timeout(hail_timeout);
			receiving_timer.start();
			connection_status = RECEIVING;

			// Restore frames_to_read for full LDPC frame capture
			// (send_hail_pattern leaves ftr=2 which is too short for START_CONNECTION)
			telecom_system->data_container.frames_to_read =
				telecom_system->data_container.preamble_nSymb + telecom_system->data_container.Nsymb;
		}
		return; // Keep scanning (fast cycle) or just responded
	}

	if (receiving_timer.get_elapsed_time_ms()<receiving_timeout)
	{
		// Phase B Wave 2 v2 — PHY swap site B (fact-doc §13.2).
		// Detect MFSK CONNECT START_CONN suffix BEFORE the legacy LDPC
		// receive() runs. On a clean detection: synthesize messages_rx_buffer
		// to look as if LDPC had just decoded a START_CONNECTION frame, then
		// fall through to the existing consumer at :287-336 (which copies
		// into messages_control and calls process_control_responder() —
		// the legacy state mutations at :1657-1747 run UNCHANGED).
		//
		// Gates:
		//  - link_status == LISTENING: waiting for a fresh CONNECT. We
		//    deliberately do NOT include CONNECTION_RECEIVED here — that's
		//    Site F's bucket (TEST_CONN). If both gates accepted the same
		//    state, Site B's helper would demod the TEST_CONN suffix,
		//    reject the wire-type, and consume the audio before Site F got
		//    a chance. (Hardware bug 2026-05-27: Wave 3 regressed every
		//    cell because of this overlap.) CMD-retry of START_CONN during
		//    CONNECTION_RECEIVED is handled by the legacy retry/timeout
		//    path which can also push RSP back to LISTENING.
		//  - messages_control.status == FREE: defer if a control frame is
		//    already in-flight (the next-tick FREE state will pick this up).
		//  - messages_rx_buffer.status != RECEIVED: the consumer at :287
		//    hasn't processed the prior buffer yet.
		//  - !passive_monitor: monitor uses LDPC for full-frame visibility.
		//  - connect_pattern_nsymb > 0: WB-only; NB falls back to LDPC.
		if(link_status == LISTENING
		   && messages_control.status == FREE
		   && !passive_monitor
		   && telecom_system->ack_mfsk.connect_pattern_nsymb > 0
		   && messages_rx_buffer.status != RECEIVED)
		{
			// frames_to_read override (v1 bug #3 prevention, fact-doc §13.4).
			// HAIL handler at :211 primes ftr = preamble_nSymb + Nsymb. The
			// MFSK detector requires ftr==0. Cap at 2 — mirrors the HAIL
			// detector's own override at :128-136. Audio buffer is filled
			// continuously by the capture callback regardless of ftr.
			if(telecom_system->data_container.frames_to_read > 2)
			{
				MUTEX_LOCK(&capture_prep_mutex);
				telecom_system->data_container.frames_to_read = 2;
				MUTEX_UNLOCK(&capture_prep_mutex);
			}

			char rx_call[7] = {};
			int rx_call_len = 0;
			bool rx_nb_flag = false;
			if(receive_mfsk_start_conn_phy(rx_call, &rx_call_len, &rx_nb_flag))
			{
				// HAIL self-detect race delay (fact-doc §6.5). CONNECT
				// pattern = connect_pattern_nsymb base + suffix_len suffix.
				// Wait (pattern_len - threshold) symbols + 200 ms margin so
				// CMD's trailing TX doesn't echo into our pattern ACK.
				const int sym_samples =
					telecom_system->data_container.Nofdm
					* telecom_system->data_container.interpolation_rate;
				const int delay_ms = start_ack_responder_delay_ms(
					telecom_system->ack_mfsk.connect_pattern_nsymb,
					telecom_system->ack_mfsk.ack_sack_suffix_len(),
					telecom_system->ack_mfsk.connect_match_threshold,
					sym_samples);
				printf("[RSP-CONNECT-V2] Waiting %d ms (HAIL race delay) "
					"before synthesizing messages_rx_buffer\n", delay_ms);
				fflush(stdout);
				// §5.7-B7: virtual-clock-ify ONLY (delay FORMULA above verbatim) —
				// route through the pump so the shared clock advances; same exit
				// predicate; verbatim msleep on production.
				pumped_settle_wait(delay_ms);

				// Synthesize messages_rx_buffer to match the LDPC
				// START_CONNECTION layout the legacy code builds at
				// arq_commander.cc:447-462:
				//   data[0]   = START_CONNECTION
				//   data[1]   = CRC8(my_call_sign)         (so the consumer
				//               at :1659-1695 PASSES — we just confirmed via
				//               MFSK CRC12, and the consumer's CRC8 check is
				//               against MY callsign, not RX'd material)
				//   data[2..6]= callsign_pack(sender, nb_flag)
				//   length    = 7
				//   sequence_number = control_batch_size - 1 so the
				//                     consumer at :327 calls
				//                     process_control_responder() immediately.
				messages_rx_buffer.type = CONTROL;
				messages_rx_buffer.sequence_number = (char)(control_batch_size - 1);
				messages_rx_buffer.length = 7;
				messages_rx_buffer.data[0] = (char)START_CONNECTION;
				messages_rx_buffer.data[1] = (char)CRC8_calc(
					(char*)my_call_sign.c_str(), my_call_sign.length());
				int pack_flags = rx_nb_flag ? 0x01 : 0;
				callsign_pack(rx_call, rx_call_len,
					&messages_rx_buffer.data[2], pack_flags);
				messages_rx_buffer.status = RECEIVED;
				printf("[RSP-CONNECT-V2] Synthesized messages_rx_buffer: "
					"sender='%s' (len=%d) nb=%d seq=%d/%d — falling through to "
					"legacy consumer\n",
					rx_call, rx_call_len, rx_nb_flag ? 1 : 0,
					(int)messages_rx_buffer.sequence_number, control_batch_size);
				fflush(stdout);
				// Fall through: legacy block at :287-336 copies this into
				// messages_control and calls process_control_responder()
				// (arq_responder.cc:1657-1747) which performs ALL state
				// mutations bit-identically with the LDPC path.
			}
		}

		// Phase B Wave 3 — PHY swap site F (fact-doc §14).
		// Detect MFSK CONNECT TEST_CONN suffix BEFORE the legacy LDPC
		// receive() runs. Mirror Site B but for TEST_CONNECTION (CMD→RSP):
		// on a clean detection, synthesize messages_rx_buffer to look as
		// if LDPC had decoded a TEST_CONNECTION frame, then fall through
		// to the existing consumer at :380-430 (copies into messages_control
		// and calls process_control_responder() — the legacy state mutations
		// at :1880-2079 run UNCHANGED).
		//
		// Gate is link_status == CONNECTION_RECEIVED ONLY. We deliberately
		// do NOT include CONNECTED here — once the handshake completes,
		// Site F's helper would keep polling forever and reset
		// frames_to_read = 2 on every miss (arq_common.cc:4902/4920/4938),
		// starving the LDPC data-RX path which needs ftr =
		// preamble_nSymb + Nsymb. Hardware bug 2026-05-27: with CONNECTED
		// in the gate, RSP reached "Connected to TESTA" but zero
		// [RX-TIMING] events ever fired. Same shape as eec768e's Site B
		// narrowing. A lost TEST_CONN_ACK is handled after the OFDM consumer
		// stages and rejects the duplicate request, without taking capture ownership.
		//
		// Other gates (messages_control.status == FREE, !passive_monitor,
		// connect_pattern_nsymb > 0, messages_rx_buffer.status != RECEIVED)
		// match Site B exactly.
		if(link_status == CONNECTION_RECEIVED
		   && messages_control.status == FREE
		   && !passive_monitor
		   && telecom_system->ack_mfsk.connect_pattern_nsymb > 0
		   && messages_rx_buffer.status != RECEIVED)
		{
			// ftr override — same rationale as Site B (fact-doc §13.4).
			// Either the post-Site-C TX path or the post-Site-B fall-through
			// can leave ftr large; the MFSK suffix detector at
			// receive_mfsk_ctrl_suffix_phy_core requires ftr==0 to sample.
			if(telecom_system->data_container.frames_to_read > 2)
			{
				MUTEX_LOCK(&capture_prep_mutex);
				telecom_system->data_container.frames_to_read = 2;
				MUTEX_UNLOCK(&capture_prep_mutex);
			}

			uint8_t rx_snr_q = 0, rx_local_cap = 0, rx_ssid = 0;
			if(receive_mfsk_test_conn_phy(&rx_snr_q, &rx_local_cap, &rx_ssid))
			{
				// HAIL self-detect race delay — same shape as Site B.
				int sym_ms = (telecom_system->data_container.Nofdm
					* telecom_system->data_container.interpolation_rate * 1000) / 48000;
				int remaining_syms =
					telecom_system->ack_mfsk.connect_pattern_nsymb
					+ telecom_system->ack_mfsk.ack_sack_suffix_len()
					- telecom_system->ack_mfsk.connect_match_threshold;
				if(remaining_syms < 0) remaining_syms = 0;
				int delay_ms = remaining_syms * sym_ms + 200;
				printf("[RSP-TEST-CONN-V3] Waiting %d ms (HAIL race delay) "
					"before synthesizing messages_rx_buffer\n", delay_ms);
				fflush(stdout);
				// §5.7-B7: virtual-clock-ify ONLY (delay FORMULA above verbatim) —
				// route through the pump so the shared clock advances; same exit
				// predicate; verbatim msleep on production.
				pumped_settle_wait(delay_ms);

				// Reconstruct float SNR from 4-bit quantization via the
				// inverse of cl_mfsk::snr_to_tone(M=16).
				float snr_uplink =
					telecom_system->ack_mfsk.tone_to_snr((int)rx_snr_q);
				u_SNR tmp_SNR;
				tmp_SNR.f_SNR = snr_uplink;

				// Synthesize messages_rx_buffer to match the LDPC
				// TEST_CONNECTION layout the legacy CMD builds at
				// arq_commander.cc:466-475:
				//   data[0]   = TEST_CONNECTION
				//   data[1..4]= u_SNR.char4_SNR (float SNR_uplink)
				//   data[5]   = local_capability (peer's cap byte; MFSK ctrl carries the
				//               six negotiable bits CAP_NEGOTIABLE_MASK=0x3F)
				//   data[6]   = peer SSID
				//   length    = 7
				//   sequence_number = control_batch_size - 1 so the consumer
				//     at :420 calls process_control_responder() immediately
				//     (audit finding §13.9 / §14).
				messages_rx_buffer.type = CONTROL;
				messages_rx_buffer.sequence_number = (char)(control_batch_size - 1);
				messages_rx_buffer.length = 7;
				messages_rx_buffer.data[0] = (char)TEST_CONNECTION;
				for(int i = 0; i < 4; i++)
					messages_rx_buffer.data[i+1] = tmp_SNR.char4_SNR[i];
				messages_rx_buffer.data[5] = (char)rx_local_cap;
				messages_rx_buffer.data[6] = (char)rx_ssid;
				messages_rx_buffer.status = RECEIVED;
				printf("[RSP-TEST-CONN-V3] Synthesized messages_rx_buffer: "
					"snr=%.1f dB local_cap=0x%02X ssid=%u seq=%d/%d — "
					"falling through to legacy consumer\n",
					snr_uplink, (unsigned)rx_local_cap, (unsigned)rx_ssid,
					(int)messages_rx_buffer.sequence_number, control_batch_size);
				fflush(stdout);
				// Fall through: legacy block at :380-430 copies into
				// messages_control and calls process_control_responder()
				// (arq_responder.cc:1880-2079 TEST_CONNECTION branch) which
				// performs ALL state mutations bit-identically with the LDPC
				// path: peer_capability, destination_call_sign SSID, SNR_uplink,
				// compression/encryption/SACK setup, TCP CONNECTED message,
				// link_status=CONNECTED, ACKNOWLEDGING_CONTROL with the
				// TEST_CONNECTION_ACK queued for Site C TX.
			}
		}

		this->receive();

		// The responder commits CONNECTED before the commander can acknowledge
		// receipt of TEST_CONNECTION_ACK.  If that one ACK is lost, the commander
		// repeats TEST_CONNECTION, but Site F above is no longer active.  Inspect
		// only an OFDM window the data owner has already staged and failed; this
		// adds no capture countdown writes and cannot starve the first data batch.
		if(connect_reack_pre_data_window()
		   && rx_fresh_window_decoded_this_pass
		   && messages_rx_buffer.status != RECEIVED
		   && telecom_system->ack_mfsk.connect_pattern_nsymb > 0)
		{
			uint8_t rx_snr_q = 0, rx_local_cap = 0, rx_ssid = 0;
			if(receive_mfsk_test_conn_from_staged_phy(
					&rx_snr_q, &rx_local_cap, &rx_ssid)
			   && connect_reack_duplicate_matches(rx_local_cap, rx_ssid))
			{
				long long elapsed = send_mfsk_test_ack_phy(
					connect_ack_cache.echoed_cap,
					connect_ack_cache.own_cap,
					connect_ack_cache.ssid);
				connect_ack_cache.replays++;
				printf("[CONNECT-REACK] duplicate TEST_CONNECTION -> cached TEST_ACK "
					"(%lld ms), replay=%d/%d\n", elapsed,
					connect_ack_cache.replays, nResends);
				fflush(stdout);

				// The ACK transmitter leaves its short MFSK capture countdown in
				// place.  Hand ownership straight back to the OFDM data receiver.
				MUTEX_LOCK(&capture_prep_mutex);
				telecom_system->data_container.frames_to_read =
					telecom_system->data_container.preamble_nSymb
					+ telecom_system->data_container.Nsymb + 10;
				telecom_system->data_container.nUnder_processing_events = 0;
				MUTEX_UNLOCK(&capture_prep_mutex);
			}
		}

		// A short final group is closed by an authenticated/session-bound
		// BLOCK_COMMIT. The RX dispatcher has already validated it before any
		// legacy parser mutation; flush the retained batch bitmaps now.
		if(messages_rx_buffer.status == RECEIVED
		   && messages_rx_buffer.type == BLOCK_COMMIT
		   && l1_blockack_data_active())
		{
			std::vector<uint8_t> wire;
			const bool requested = l1_blockack.take_flush_request();
			messages_rx_buffer.status = FREE;
			if(requested && l1_blockack.flush_received_batches(&wire))
			{
				l1_send_block_control(wire);
				printf("[L1-BLOCKACK-RSP] tail commit flushed retained group\n");
				fflush(stdout);
			}
			telecom_system->set_mfsk_ctrl_mode(false);
			calculate_receiving_timeout();
			receiving_timer.start();
			connection_status = RECEIVING;
			return;
		}

		// Emergency BREAK: commander signals "drop to ROBUST_0"
		// Only act on a BREAK aimed at THIS session — prevents idle radios on a
		// frequency from ACKing someone else's BREAK. R2b (zombie/amplifier layer,
		// data-flow-zombie-amplifier.md §1): a DROPPED responder (silent gap-abort
		// teardown zombie) is now ACTIONABLE so it can accept the CMD's BREAK and run
		// the existing reset-to-ROBUST_0 + ACK below — the wire notice the CMD's
		// recovery arsenal already understands — instead of the CMD grinding its
		// demote ladder against a peer that discards the BREAK. LISTENING / CONNECTING
		// and every other non-session state still discard. MERCURY_BREAK_DROPPED_DEFEAT=1
		// restores the CONNECTED-only accept (fail-before). The handler leaves
		// link_status unchanged (DROPPED stays DROPPED at ROBUST_0/RECEIVING); the CMD
		// re-establishes via the existing post-BREAK SET_CONFIG / reconnect flow.
		if(break_detected == YES && !break_frame_actionable(link_status))
			break_detected = NO;
		if(break_detected == YES && break_frame_actionable(link_status))
		{
			l1_apply_reset_event(mercury::L1ResetEvent::PEER_BREAK);
			printf("[BREAK] %s (link_status=%d), dropping to ROBUST_0\n",
				passive_monitor ? "Observed" : "Responding with ACK", link_status);
			fflush(stdout);
			break_detected = NO;
			telecom_system->invalidate_channel_selectivity();

			// Bug fix (POST_BREAK_STUCK_INVESTIGATION.md §5.1 / §8.1,
			// SACK_DESIGN_A_PLAN §7.13.14): force-FREE the control slot.
			// BREAK is a hard reset signal — any in-flight control exchange
			// is moot, the commander is changing config and will re-issue.
			// Without this reset, a slot stuck at ACKED/RECEIVED/PENDING_ACK
			// (e.g. from a control batch that just landed pre-BREAK) causes
			// every post-BREAK SET_CONFIG to be silently dropped at the
			// FREE gate at :262 below — process_control_responder() never
			// runs, mercury sits at the post-BREAK config never accepting
			// commander instructions. Symmetric with the CMD-side BREAK ACK
			// handlers at arq_commander.cc:113, :184 which already
			// force-FREE for the same reason ("cleanup() skips PENDING_ACK
			// status"). The race window is widened by short frame time at
			// higher OFDM configs; ROBUST_0's longer frames usually close
			// it before the bug bites, which is why the prior
			// BREAK_FAILSAFE_INVESTIGATION's -R workaround appeared to fix
			// the deadlock without addressing this root cause.
			messages_control.status = FREE;

			// §6f (fact-doc sack_partial_bsi_advance.md, 2026-05-21): defensive
			// reset of SACK v2 bsi state on BREAK. Symmetric with
			// reset_session_state at arq_common.cc:186-188. Without this, a
			// stuck-bsi condition (e.g. from a §6g-class bypass that escaped
			// the fix, or a future regression) would survive the BREAK and
			// cause continued [RSP-V2-DROP] storms post-config-drop. With
			// this, BREAK self-heals: the next data frame re-bootstraps the
			// bsi window via [RSP-V2-ADOPT] at arq_responder.cc:393-398.
			// Cost: one batch of bootstrap latency on every BREAK (acceptable
			// — BREAK is already a hard recovery event).
			// FIX #3 (data-flow-inband-downladder.md §3): under inband, deliver a COMPLETE
			// in-flight prev batch BEFORE the ROBUST_0 reseed reshrink orphans its RECEIVED
			// frames. Feature-gated -> no-op (returns 0) on the legacy BREAK path, so the
			// default/legacy behaviour here is byte-identical. Must run BEFORE the bsi reset
			// just below clears the prev state.
			deliver_complete_inflight_before_break();
			rsp_current_expected_batch_seq_id = -1;
			rsp_prev_batch_seq_id = -1;

#ifdef MERCURY_GUI_ENABLED
			if(passive_monitor)
				gui_push_monitor_event("[BREAK -> ROBUST_0]", false);
#endif

			// Send ACK to confirm BREAK received (suppressed in monitor mode).
			// control_ack=true: this BREAK-recovery turnaround opts into the robust
			// noncoherent-repeat ACK when MERCURY_RECOVERY_ACK_ROBUST is set
			// (recovery-ack-robustness.md §4); default-off → single block.
			send_ack_pattern(/*control_ack=*/true);

			// Drop to ROBUST_0 (commander will send SET_CONFIG at ROBUST_0)
			int target = robust_enabled ? ROBUST_0 : CONFIG_0;
			data_configuration = target;
			load_configuration(target, PHYSICAL_LAYER_ONLY, YES);

			// Wait for SET_CONFIG from commander
			calculate_receiving_timeout();
			receiving_timer.start();
			batch_rx_frame_count = 0;
			connection_status = RECEIVING;
			link_timer.start();
			return;
		}

		// ─── STAGE 4 RANK-1 PREREQUISITE: seat the robust ring floor (once) ───
		// The down-ladder reads the PRIMARY capture ring; that ring must physically hold a
		// full ROBUST-rung frame or no snapshot-length change can recover the (never-captured)
		// samples (data-flow-inband-ondemote-zerobyte.md §7/§8). Seat buffer_Nsymb_min to the
		// ROBUST floor the FIRST time the responder is CONNECTED+RECEIVING under inband and the
		// ring is between frames (no active batch) so the re-apply flush is harmless. Idempotent
		// (the seat short-circuits once buffer_Nsymb_min >= floor); no-op when off / defeat set.
		if(inband_rate_feature_enabled()
		   && link_status == CONNECTED
		   && connection_status == RECEIVING
		   && !passive_monitor
		   && !telecom_system->receive_stats.ofdm_batch_active)   // between frames: flush is safe
		{
			inband_seat_robust_ring_floor();
		}

		// ─── STAGE 4 RANK-2: ROBUST-TIER LOST-TAG FOLLOW (data-flow-inband-downladder.md §10) ───
		// The OFDM down-ladder below is GATED is_ofdm_config(current_configuration) (the
		// blind DATA-frame down-window only makes sense when the RX is at an OFDM rung).
		// But the rate-adapt legitimately operates in the ROBUST tier on a degraded channel
		// (WGN:18: the CMD probes ROBUST_0 -> ROBUST_1 via the CONFIG_TAG). When the RX is
		// parked at ROBUST_0 (is_ofdm_config==false) and the CMD has moved to ROBUST_1, the
		// W2 adopt-follow (:974) cannot fire (no DATA frame decodes across the ROBUST_0/
		// ROBUST_1 MFSK PHY mismatch) AND the OFDM down-ladder is gated off -> the RX stays at
		// ROBUST_0, logs `[RX-TIMEOUT] No frames decoded` for ~60s, INBAND-LIVENESS arms and
		// fires a true-loss BREAK (measured: _fixrun/fb — CMD emits CONFIG_TAG cfg=101 x3, RSP
		// follow count 0, 10x RX-TIMEOUT cfg=100, 15 bytes delivered then BREAK).
		//
		// RECOVERY (the on-design counterpart of the OFDM down-ladder): run the SHARED
		// direction-agnostic real-tag follow inband_detect_follow_from_capture, which reads the
		// ACTUAL CONFIG_TAG burst from the captured robust frame tail (the tag rides the M=16
		// robust suffix — emit_config_tag_passband fires for ROBUST_0/ROBUST_1 too) and follows
		// it UP or DOWN via detect_and_follow_config_tag -> FULL_CONFIG_LADDER[cfg_index]. This
		// is the correct mechanism for the robust tier because the up-probe (ROBUST_0->ROBUST_1)
		// is an UPWARD move the downward-only inband_down_ladder_resync structurally cannot
		// recover. The robust-ring floor seat at :567-574 guarantees the live ring physically
		// holds a full robust frame, so inband_detect_follow_from_capture's tail snapshot (sized
		// from the current robust PHY geometry, read under capture_prep_mutex) holds the tag.
		// On a real CRC/LDPC-bound follow the HINGE re-baselines the window and the next pass
		// acquires ROBUST_1 + delivers. No tag present in the tail (steady state / genuine
		// silence) -> a cheap false, no action, no BREAK. Feature-gated -> legacy byte-identical.
		// RESIDUAL FIX: the firing predicate is factored into inband_robust_follow_gate_open()
		// so the directed regression (test_inband_robust_follow_freshwin) drives the EXACT same
		// production gate. It DROPS the fresh-window requirement (vs the OFDM blind down-ladder)
		// unless MERCURY_ROBUST_FOLLOW_FRESHWIN_REQ=1 restores the pre-fix (non-deterministic) gate.
		if(inband_robust_follow_gate_open(rx_fresh_window_decoded_this_pass))
		{
			// bsi binding 0xFF: across a possibly-lost robust tag the RX does not track the
			// TX bsi (mirrors the Stage-4 OFDM resync, arq_common.cc:4275). The FWHT peak +
			// GF(16)+CRC-12 + cfg_index corroboration are the accept gates. expect_parity=0xFF.
			int rb_fc = current_configuration;
			int rb_fol = inband_detect_follow_from_capture(
				/*expect_bsi_lsb=*/0xFF, /*expect_parity=*/0xFF, &rb_fc);
			if(rb_fol == 1)
			{
				printf("[INBAND-RX] ROBUST-TIER tag-follow to CONFIG_%d (was robust); "
					"BREAK avoided (real CONFIG_TAG, not a blind guess)\n", rb_fc);
				fflush(stdout);
				// A successful follow re-arms the receiving window for the new config and
				// resets the DECODE_FAIL NACK throttle (inband_adopt_resynced_config already
				// re-armed the throttle); re-arm the receiving timer so the loop keeps pumping.
				calculate_receiving_timeout();
				receiving_timer.start();
			}
		}

		// ─── STAGE 4: LOST-TAG BOUNDED DOWN-LADDER (design §4 / §7) ───
		// The §3.2 outcome-3 recovery. receive() above (:433) returned with NO decoded
		// DATA frame at current_configuration AND no CRC-valid CONFIG_TAG followed (the
		// W2 tag-follow at the [RSP-V2-ADOPT] site only fires on a SUCCESSFUL adopt). A
		// signal-present first-frame decode FAIL with no tag is the ONLY thing that can
		// mean "the change tag was lost in a fade" (design §3.2). Run the bounded
		// down-window blind decode; on a real CRC/LDPC pass adopt the true config and
		// re-feed the snapshot so the batch delivers (the returning SACK is the implicit
		// confirm). None-pass steps the terminal-BREAK dead-batch streak; only
		// SESSION_DEAD_BATCHES consecutive total-losses reach BREAK — the ONLY remaining
		// BREAK path on the inband path. No-op unless MERCURY_INBAND_RATE is set.
		// Default-off: byte-identical (the whole block is feature-gated).
		if(inband_rate_feature_enabled()
		   && link_status == CONNECTED
		   && connection_status == RECEIVING
		   && !passive_monitor
		   // A receiver cannot know that a tag was lost: inband_last_announced_config
		   // is TX-only state and is never a valid RX transition predicate.  Permit
		   // one blind decoder-bank search for this active wire batch; any later
		   // failure at the same bsi waits for retransmission/the canonical re-tag.
		   // This bounds work by a protocol generation, not by a timing threshold.
		   && inband_down_probe_batch_seq_id != rsp_current_expected_batch_seq_id
		   && (rx_fresh_window_decoded_this_pass              // a FRESH window was staged+decoded this pass
		       || inband_freshwin_gate_defeat())              // (A/B fail-before knob restores pre-fix firing)
		   // The PHY must have admitted a real OFDM preamble using its existing
		   // detector.  Buffer energy alone can be the leading edge of an arriving
		   // steady-state frame; probing then delays capture and a false NACK collides
		   // with the very block that would have decoded.
		   && telecom_system->receive_stats.ofdm_preamble_detected
		   // Correlation admission is still not a payload failure.  Require the
		   // primary decoder to have reached an actual FEC attempt; iterations_done
		   // remains -1 for leading edges, incomplete frames and SKIP-VAR candidates.
		   // This is decoder provenance, not a controller threshold.
		   && telecom_system->receive_stats.iterations_done >= 0
		   // A located preamble whose DATA tail is not captured yet is timing
		   // evidence, not lost-tag evidence. Let the short wait-for-tail path
		   // complete the same snapshot before trying alternate configurations.
		   && !telecom_system->receive_stats.frame_data_missing
		   && messages_rx_buffer.status != RECEIVED          // no frame decoded this pass
		   && is_ofdm_config(current_configuration)           // tag only rides OFDM batches
		   && rsp_current_expected_batch_seq_id >= 0)         // IN-FLIGHT active batch only
		{
			// Consume this batch's single blind-probe allowance before entering the
			// decoder bank.  Even a silent/malformed snapshot must not spin another
			// five decoders on every fresh capture for the same outstanding batch.
			inband_down_probe_batch_seq_id = rsp_current_expected_batch_seq_id;
			// FIRING GATE (data-flow-inband-downladder.md §2/§5.2, defect #2 + §2.1 the
			// fresh-window term). THREE terms guard the firing frequency:
			//   (a) rx_fresh_window_decoded_this_pass — receive() actually re-staged a
			//       FRESH capture window and attempted a primary decode THIS pass
			//       (arq_common.cc:10624 frames_to_read==0 branch). Without it the gate
			//       fired on EVERY benign inter-frame pass (the ~500 Hz ARQ loop, receive()
			//       taking the frames_to_read!=0 early-exit at arq_common.cc:12158 WITHOUT
			//       re-staging) — re-probing the STALE staged buffer (the last decoded frame,
			//       energy>=0.05 -> the window energy gate PASSES). That was the 3127 "all
			//       silent" HW firings burning the receive loop. A lost CONFIG_TAG can only
			//       be diagnosed on a pass where a fresh frame STAGED and FAILED to decode,
			//       so this term does NOT suppress a genuine signal-present loss (a real
			//       lost-tag frame stages a fresh window -> the flag is true).
			//   (b) `messages_rx_buffer.status != RECEIVED` alone also fires on idle /
			//       no-active-batch passes; require an IN-FLIGHT ACTIVE batch
			//       (rsp_current_expected_batch_seq_id >= 0 i.e. bsi_lsb != 255): a lost
			//       CONFIG_TAG can only happen WITHIN a batch we are tracking. The
			//       window-level + per-decoder energy prescans inside the resync remain the
			//       signal-present-but-silent-snapshot backstop.
			//   (c) inband_down_probe_batch_seq_id binds blind work to the active wire
			//       generation.  A retry at the same bsi is handled by retransmission and
			//       the sender's repeated CONFIG_TAG, not another decoder-bank scan.
			inband_try_down_ladder_on_decode_fail();

			// TERMINAL BREAK (design §7): the dead-batch streak reached
			// SESSION_DEAD_BATCHES — true session loss. Fire the EXISTING BREAK→ROBUST_0
			// reset (mechanism unchanged; the trigger moved from a per-batch ACK miss to
			// this terminal floor). This is the ONLY remaining BREAK path on the inband
			// path. Mirrors the break_detected handler above (:487-498): drop to
			// ROBUST_0, re-baseline the bsi window, re-arm the receiving timer.
			if(inband_terminal_break_due)
			{
				inband_terminal_break_due = false;
				printf("[INBAND-RX] TERMINAL BREAK: SESSION_DEAD_BATCHES reached -> "
					"ROBUST_0 (the only inband BREAK path)\n");
				fflush(stdout);
				// FIX #3 (data-flow-inband-downladder.md §3): deliver a COMPLETE in-flight
				// prev batch BEFORE the ROBUST_0 reseed reshrinks data_batch_size and orphans
				// its RECEIVED frames into a stale-discard (0 bytes). Runs while the prev state
				// is still live (before the bsi reset / load_configuration below).
				deliver_complete_inflight_before_break();
				messages_control.status = FREE;
				rsp_current_expected_batch_seq_id = -1;
				rsp_prev_batch_seq_id = -1;
				int tgt = robust_enabled ? ROBUST_0 : CONFIG_0;
				data_configuration = tgt;
				load_configuration(tgt, PHYSICAL_LAYER_ONLY, YES);
				calculate_receiving_timeout();
				receiving_timer.start();
				batch_rx_frame_count = 0;
				connection_status = RECEIVING;
				link_timer.start();
				return;
			}
		}

		// ─── LEVER #2: SPECULATIVE / PROMPT SACK (env MERCURY_SPEC_SACK) ───
		// turnaround-eff.md §8. Fire the reverse-ACK (SACK) on a WINDOW-FRACTION
		// DEADLINE rather than waiting for the whole batch — including any
		// still-decoding / non-converging frame — to finish the serial receive()
		// pump. receive() (above, :432) is BLOCKING, so the deadline can only be
		// checked at THIS loop boundary (the first line after receive() returns,
		// covering frame-stored / FAIL / nothing-decoded alike). At the deadline,
		// frames that have NOT yet reached messages_rx[].status==RECEIVED are
		// reported bit-0 by the EXISTING ACK-GATE bitmap build (:1689-1691) and
		// recovered via the bench-validated idempotent partial-SACK + CMD-retx
		// path (arq_commander.cc:3098-3164). We do NOT duplicate the ACK-GATE
		// logic — we only advance connection_status to ACKNOWLEDGING_DATA one
		// instant EARLIER (the same value the natural timer-expiry path at :1150
		// sets), so the next pump tick runs process_messages_acknowledging_data().
		// Default-off (env unset) ⇒ this block is skipped ⇒ the SACK fires after
		// the serial loop exactly as today (BYTE-IDENTICAL). See §8.4 cross-layer
		// audit (no producer of messages_rx[].status is altered) and §8.5 (the
		// D5 prev-bump chain is NOT amplified — a missing/late EOB leaves
		// prev_expected at the full data_batch_size, never a truncated EOB+1).
		{
			static const bool spec_sack_on =
				(std::getenv("MERCURY_SPEC_SACK") != nullptr
				 && atoi(std::getenv("MERCURY_SPEC_SACK")) != 0);
			if(spec_sack_on
			   && link_status == CONNECTED
			   && connection_status == RECEIVING
			   && !passive_monitor
			   && sack_enabled
			   && data_batch_size > 1          // single-frame batches use EOB receipt (§8.5)
			   && batch_rx_frame_count >= 1)   // never SACK an empty batch (§8.2)
			{
				// rx_received + expected — computed IDENTICALLY to the ACK-GATE
				// handler (arq_responder.cc:1636-1669) so the "incomplete?" test
				// agrees with the SACK it will build.
				int rx_received = 0;
				for(int i = 0; i < data_batch_size; i++)
					if(messages_rx[i].status == RECEIVED) rx_received++;
				int expected = data_batch_size;
				if(last_received_end_of_batch_seq >= 0)
				{
					expected = last_received_end_of_batch_seq + 1;
					if(expected > data_batch_size) expected = data_batch_size;
				}
				else if(compression_enabled
					&& !cipher_suite.is_active()
					&& !compressor.is_streaming()
					&& messages_rx[0].status == RECEIVED
					&& messages_rx[0].length >= compressor.get_header_size())
				{
					const unsigned char* hdr = (const unsigned char*)messages_rx[0].data;
					int hdr_comp = hdr[1] | (hdr[2] << 8);
					int gate_hdr_size = compressor.get_header_size();
					int total_compressed = gate_hdr_size + hdr_comp;
					int mf = max_data_length + max_header_length
						- effective_data_long_header_length(sack_v2_enabled);
					expected = (mf > 0) ? (total_compressed + mf - 1) / mf : data_batch_size;
					if(expected > data_batch_size) expected = data_batch_size;
					if(expected < 1) expected = 1;
				}

				// Window-fraction DEADLINE (NOT EOB-arrival-only, §8.2): a slow /
				// lost / non-converging EOB frame must STILL trigger. Default 3/4
				// of the (dynamically per-frame re-armed, :1130) receiving window;
				// env-overridable for the bench sweep. A healthy in-progress batch
				// keeps pushing receiving_timeout forward on each frame, so the
				// gate only bites once the channel/decode has genuinely stalled
				// relative to the remaining-batch estimate.
				static const int spec_num = []{
					const char* e = std::getenv("MERCURY_SPEC_SACK_NUM");
					int v = (e && *e) ? atoi(e) : 3;
					return (v >= 1) ? v : 3;
				}();
				static const int spec_den = []{
					const char* e = std::getenv("MERCURY_SPEC_SACK_DEN");
					int v = (e && *e) ? atoi(e) : 4;
					return (v >= 1 && v >= spec_num) ? v : 4;
				}();
				long long deadline_ms =
					(long long)receiving_timeout * spec_num / spec_den;

				// NEAR-COMPLETENESS gate (BUGFIX, §8.8): the deadline must ONLY
				// prompt-fire when the batch is MOSTLY decoded (a few stragglers),
				// never on a STRUGGLING first batch where most frames are still
				// decoding. Without this, a near-empty batch at the deadline reports
				// most slots bit-0 -> CMD enqueues every missing slot
				// (arq_commander.cc:3098-3164) -> trips the runaway-BREAK
				// retransmit_count>=2*data_batch_size (arq_commander.cc:1549) ->
				// ROBUST_0 demote; AND the early deadline truncates the decode the
				// slow frames still needed, MANUFACTURING the misses. We require
				// rx_received >= minfrac*expected (percent, env MERCURY_SPEC_SACK_MINFRAC,
				// default 70). Below the fraction we DO NOT prompt-fire and fall
				// through to the existing natural path (let decode catch up / the
				// normal timer at :1150). This confines lever #2 to its intended
				// regime — a few stragglers, not a mostly-empty batch.
				static const int spec_minfrac_pct = []{
					const char* e = std::getenv("MERCURY_SPEC_SACK_MINFRAC");
					int v = (e && *e) ? atoi(e) : 70;
					return (v >= 0 && v <= 100) ? v : 70;
				}();
				// Integer near-completeness test: rx_received*100 >= minfrac*expected.
				bool near_complete =
					((long long)rx_received * 100 >= (long long)spec_minfrac_pct * expected);

				if(expected >= 2
				   && rx_received < expected
				   && near_complete
				   && receiving_timer.get_elapsed_time_ms() >= deadline_ms)
				{
					printf("[RSP-SPEC-SACK] deadline fired: elapsed=%d >= %lld "
						"(=%d*%d/%d) rx=%d/%d expected=%d (minfrac=%d%% near_complete=1) "
						"eob=%d batch=%d — "
						"advancing to ACK-GATE (still-decoding frames -> bit-0 -> retx)\n",
						receiving_timer.get_elapsed_time_ms(), deadline_ms,
						receiving_timeout, spec_num, spec_den,
						rx_received, data_batch_size, expected, spec_minfrac_pct,
						last_received_end_of_batch_seq, data_batch_size);
					fflush(stdout);
					mtl::log_event_kv("rsp_spec_sack_fired",
						"elapsed=%d deadline=%lld rx=%d expected=%d",
						receiving_timer.get_elapsed_time_ms(),
						deadline_ms, rx_received, expected);
					receiving_timer.stop();
					receiving_timer.reset();
					connection_status = ACKNOWLEDGING_DATA;
					return;
				}
			}
		}

		if(messages_rx_buffer.status==RECEIVED)
		{
			if(messages_rx_buffer.type==CONTROL)
			{
				printf("[RX] CONTROL message received on CONFIG_%d, code=%d seq=%d/%d\n",
					current_configuration, (int)messages_rx_buffer.data[0],
					messages_rx_buffer.sequence_number, control_batch_size);
				if(messages_control.status==FREE)
				{
					messages_control.type=messages_rx_buffer.type;
					messages_control.id=0;
					messages_control.status=RECEIVED;
					messages_control.length=1;
					messages_control.sequence_number=messages_rx_buffer.sequence_number;
					// RX-CTRL-DROP fix (data-flow-control-slot-lifecycle.md §5 change A):
					// arm the per-slot ack_timer as a RECEIVED-state watchdog. The slot
					// is a one-deep mailbox; if a consume path strands it in RECEIVED
					// (V1 fall-through / V2 RESPONDER-timeout / V3 watchdog / V4 crypto
					// teardown) every later control frame is silently dropped (:848) and
					// the reverse control plane dies. update_status() force-FREEs a slot
					// whose ack_timer exceeds the watchdog bound — mirroring the CMD
					// PENDING_ACK escape (arq_common.cc:5718). ack_timer is otherwise
					// idle in the RSP RECEIVED state (it only arms for the CMD PENDING_ACK
					// use), so reusing it here is safe.
					messages_control.ack_timer.stop();
					messages_control.ack_timer.reset();
					messages_control.ack_timer.start();
					{
					int copy_len = max_data_length+max_header_length-CONTROL_ACK_CONTROL_HEADER_LENGTH;
					if(copy_len > N_MAX/8) copy_len = N_MAX/8;
					for(int j=0;j<copy_len;j++)
					{
						messages_control.data[j]=messages_rx_buffer.data[j];
					}
				}
					stats.nReceived_control++;
				}
				else
				{
					// POST_BREAK_STUCK_INVESTIGATION.md §8.2: silent
					// discard at this gate hid the BREAK-leaves-slot-stuck
					// bug for a long time. Log every drop so any future
					// stuck-state regression surfaces immediately.
					printf("[RX-CTRL-DROP] CONTROL frame dropped: messages_control.status=%d "
						"(prev code=%d), incoming code=%d seq=%d/%d cfg=%d\n",
						messages_control.status, (int)messages_control.data[0],
						(int)messages_rx_buffer.data[0],
						messages_rx_buffer.sequence_number,
						control_batch_size, current_configuration);
					fflush(stdout);
				}
				// BUG FIX: Process control message immediately when batch is complete
				// instead of waiting for timer (which kept getting reset by retransmissions)
				if(messages_rx_buffer.sequence_number >= control_batch_size - 1)
				{
					// Last frame in batch received - process immediately
					printf("[RX] Batch complete, processing control message immediately\n");
					receiving_timer.stop();
					receiving_timer.reset();
					if(messages_control.status==RECEIVED)
					{
						process_control_responder();
					}
				}
				else
				{
					// More frames expected in this batch - wait for them
					set_receiving_timeout((control_batch_size-messages_rx_buffer.sequence_number-1)*message_transmission_time_ms+time_left_to_send_last_frame+ptt_on_delay_ms);
					receiving_timer.start();
				}
			}
			else if(messages_rx_buffer.type==DATA_LONG || messages_rx_buffer.type==DATA_SHORT)
			{
				// STAGE 4 (design §7): a DATA frame decoded at the current config -> the
				// link is ALIVE. Reset the terminal-BREAK dead-batch streak so it only
				// counts CONSECUTIVE total-loss batches. No-op when the feature is off.
				if(inband_rate_feature_enabled())
					inband_session_dead_batches = 0;

				// Monitor: auto-adopt session if we receive data while still LISTENING
				// (missed START_CONNECTION — joined mid-session)
				if(passive_monitor && link_status == LISTENING)
				{
					printf("[MONITOR] Data frame received while LISTENING — adopting session mid-stream\n");
					fflush(stdout);
					begin_radio_session_attempt();
					link_status = CONNECTED;
					connection_status = RECEIVING;
					compression_enabled = true;
					watchdog_timer.start();
					link_timer.start();
#ifdef MERCURY_GUI_ENABLED
					gui_set_monitor_callsigns("STA_A", "STA_B");
					gui_push_monitor_event("[MONITOR: joined session mid-stream]", false);
#endif
				}

				// SACK Design A Step 4 — RSP cross-batch routing decision.
				// Gated on sack_v2_enabled (v1 path takes the same code path as
				// pre-Step-4: v2_route_drop and v2_route_to_prev both stay false,
				// all storage proceeds into messages_rx[]).
				// Per §4.2.3 + §4.3.4 invariant #3:
				//   • match `rsp_current_expected_batch_seq_id`  → route to current
				//   • match `rsp_prev_batch_seq_id`              → route to prev
				//     (Step 8a — when rsp_prev_batch_active is true; otherwise
				//      the match-prev branch is a stale-window late retransmit
				//      with no live storage, treated as a duplicate and
				//      DISCARDED to avoid silent corruption.)
				//   • unknown / out-of-window                     → discard + log
				// In Step 8a, mechanism-(a) standalone retransmits hit the
				// match-prev branch (the SACK-RSP-send bump has already
				// advanced current to N+1 by the time CMD's retransmit-only
				// batch for N arrives). Those frames now land in
				// `messages_rx_prev[]` — the parallel storage that §7.8.3
				// named as the missing piece. The match-current branch
				// routes to `messages_rx[]` (the v1-shaped path; unchanged).
				bool v2_route_drop    = false;
				bool v2_route_to_prev = false;
				bool fix8_gap_aborted = false;  // FIX-8: set when the gap gate tore the session down
				if(sack_v2_enabled)
				{
					int bsi = messages_rx_buffer.batch_seq_id;
					test_rsp_bsi_v2_frame_counter++;
					// Test scaffold: corrupt the Nth v2 DATA frame's bsi to a
					// known-bad value (parsed + 7 mod 256). Falls outside both
					// current_expected and prev, so it must trigger the discard
					// branch. One-shot: cleared after firing exactly once.
					if(test_rsp_bsi_corrupt_at > 0
					   && test_rsp_bsi_v2_frame_counter == test_rsp_bsi_corrupt_at)
					{
						int orig = bsi;
						bsi = (bsi + 7) & 0xFF;
						printf("[RSP-V2-TEST-CORRUPT] frame#%d: bsi %d → %d "
							"(synthetic discard test fault injection)\n",
							test_rsp_bsi_v2_frame_counter, orig, bsi);
						fflush(stdout);
						test_rsp_bsi_corrupt_at = 0;  // one-shot
					}
					// Adopt current_expected from the first v2 DATA frame seen
					// this session (initial state -1 = unset).
					if(rsp_current_expected_batch_seq_id < 0)
					{
						// FIX-8 (data-integrity): GAP GATE. A BREAK / FULL-config
						// reset wiped current_expected to -1 mid-transfer; this
						// re-adopt takes whatever bsi arrives next. If batches were
						// already DELIVERED (rsp_last_delivered_batch_seq_id >= 0,
						// which survives the reset) and the arriving bsi is NOT
						// contiguous with the last delivered batch, then batches in
						// between were silently dropped. Delivering this batch would
						// concatenate non-contiguous bytes into the app stream (and
						// desync the streaming decompressor). REFUSE: fail the
						// transfer LOUDLY (control-port error + DROPPED + session
						// reset) — the same loud-fail mechanism as the PSK-mismatch
						// path at arq_common.cc:9513-9518. The CMD re-sends the
						// abandoned bytes under fresh bsi (FIX8_DESIGN §3), so a
						// fresh transfer recovers everything; aborting here is
						// non-lossy at the system level and guarantees the hard
						// invariant: NEVER silently deliver non-contiguous bytes.
						// MERCURY_GAP_ABORT_DEFEAT=1 disables the abort on the SAME
						// binary (restores pre-fix silent-concat) for the
						// --test-gap-abort fail-before arm.
						// QUARANTINED 2026-06-17: proven-broken (=1 defeats the loud
						// D3.1 in-order integrity guard => silent non-contiguous
						// delivery; BENCH-9 + cfg15_stall confirm the guard is the
						// no-silent-wrong-bytes backstop). do-not-enable.
						bool gap_defeat = false;
						{ const char* e = std::getenv("MERCURY_GAP_ABORT_DEFEAT");
						  if(e && *e && atoi(e)!=0) gap_defeat = true; }
						// rsp_stream_aborted: once a gap-abort has latched the session as
						// torn down, refuse EVERY re-adopt unconditionally — even one at
						// last+1 that looks contiguous to the ruler — until a genuine new
						// session (START_CONNECTION) clears the latch. The unsafe re-drive
						// into a DROPPED stream is thereby unrepresentable.
						if(!gap_defeat
						   && (rsp_stream_aborted
						       || sack_v2_readopt_has_gap(bsi, rsp_last_delivered_batch_seq_id)))
						{
							// D3.1: shared loud-abort teardown (was inlined here;
							// now the IDENTICAL action the delivery-time gate uses).
							char reason[96];
							snprintf(reason, sizeof(reason),
								"re-adopt bsi=%d non-contiguous with last_delivered=%d (dropped-batch hole)",
								bsi, rsp_last_delivered_batch_seq_id);
							rsp_gap_abort_teardown(reason);

							// R7: do NOT deliver/store this frame; let the common tail
							// (messages_rx_buffer.status=FREE; link_timer.start(); …
							// at arq_responder.cc:1024) free the buffer and the
							// function return naturally — no further processing of the
							// torn-down frame.
							v2_route_drop    = true;
							fix8_gap_aborted = true;
						}
						else
						{
							// RE-SEAL NONCE GENERATION (data-flow-aead-nonce.md §11):
							// expected was wiped to -1 by a config transition (BREAK /
							// SET_CONFIG / CONFIG_TAG follow). If this peer had ALREADY
							// adopted earlier this session, this is a genuine RE-adopt =
							// the same transition that drove the TX restore_tx_from_compressed
							// (one recovery -> one SET_CONFIG that lands -> one re-adopt,
							// 1:1 across BREAK retries). Bump rx_nonce_gen so the decrypt of
							// the re-sealed batch (folded gen, copy_data_to_buffer) matches
							// the TX seal's bumped gen — at the SAME wire bsi. The first-ever
							// adopt (adopted_once==false) is gen 0 (no transition yet). On a
							// gen drift the decrypt simply auth-fails (safe); the TX
							// high-water guard owns no-reuse.
							if(cipher_suite.is_active() && rx_nonce_adopted_once)
							{
								rx_nonce_gen++;
								printf("[RSP-V2-ADOPT] re-adopt after transition: "
									"rx_nonce_gen -> %llu\n",
									(unsigned long long)rx_nonce_gen);
								fflush(stdout);
							}
							rx_nonce_adopted_once = true;
							rsp_current_expected_batch_seq_id = bsi;
							printf("[RSP-V2-ADOPT] current_expected_batch_seq_id=%d "
								"(first v2 DATA frame this session)\n", bsi);
							fflush(stdout);

							// STAGE 3b W2 (data-flow-perbatch-config.md §12 W2): on the
							// first DATA frame of a batch, attempt to FOLLOW a CONFIG_TAG
							// from the captured passband (the burst W1 keyed after frame 0).
							// On a valid+bound tag this switches config (ARQ + PHY twin) AND
							// fires the HINGE side-effects (capture-flush + D3.1 re-baseline),
							// which re-baseline rsp_current_expected_batch_seq_id back to -1
							// so the NEXT data frame re-adopts at the new config through the
							// gap-gate. No-op unless MERCURY_INBAND_RATE is set + a tag is
							// present. expect_parity=0xFF: the RX does not track the TX parity
							// across a possibly-lost tag (Stage 4); the bsi_lsb binding +
							// CRC-12 + FWHT peak margin are the accept gates.
							if(inband_rate_feature_enabled())
							{
								int fc = current_configuration;
								int fol = inband_detect_follow_from_capture(
									(uint8_t)(bsi & 0x7), /*expect_parity=*/0xFF, &fc);
								if(fol == 1)
								{
									printf("[RSP-V2-ADOPT] tag-follow to CONFIG_%d on adopt "
										"of bsi=%d (HINGE re-baselined window)\n", fc, bsi);
									fflush(stdout);
								}
							}
						}
					}
					if(!fix8_gap_aborted)
					{
					bool match_current = (bsi == rsp_current_expected_batch_seq_id);
					bool match_prev    = (rsp_prev_batch_seq_id >= 0
					                      && bsi == rsp_prev_batch_seq_id);
					if(!match_current && !match_prev)
					{
						v2_route_drop = true;
						rsp_v2_drop_count++;
						printf("[RSP-V2-DROP] batch_seq_id=%d expected=%d prev=%d "
							"reason=%s (drop_count=%lld)\n",
							bsi, rsp_current_expected_batch_seq_id,
							rsp_prev_batch_seq_id,
							"unknown_or_out_of_window",
							rsp_v2_drop_count);
						fflush(stdout);
					}
					else if(match_prev && !match_current)
					{
						// A previous-generation retransmission is still the commander's
						// active physical keydown. Keep the deferred-confirm boundary
						// timer aligned with every decoded frame in that turn.
						linkphase_rearm_prev_boundary_timer(
							messages_rx_buffer.sequence_number,
							rx_buffer_batch_total_frames,
							rx_buffer_eob_seq >= 0);
						// Step 8a: route to messages_rx_prev[] if the prev
						// buffer is live. If a Step 4 ACK-GATE-PASS bump
						// (clean batch path) advanced prev without
						// activating the prev buffer, late retransmits for
						// that already-delivered batch have no live storage
						// — drop them as duplicates (preserves §4.3.4 #3
						// "no silent corruption"; symmetric with the v1
						// path which also relies on idempotent delivery).
						if(rsp_prev_batch_active)
						{
							v2_route_to_prev = true;
						}
						else
						{
							v2_route_drop = true;
							rsp_v2_drop_count++;
							printf("[RSP-V2-DROP] batch_seq_id=%d expected=%d prev=%d "
								"reason=prev_inactive_late_retransmit (drop_count=%lld)\n",
								bsi, rsp_current_expected_batch_seq_id,
								rsp_prev_batch_seq_id, rsp_v2_drop_count);
							fflush(stdout);
						}
					}
					}
				}

				// SACK Design A Step 8a — match-prev branch: store the frame
				// into messages_rx_prev[] without touching messages_rx[]. The
				// generation pipeline remains independent, while the shared
				// physical receive-boundary timer above follows the active keydown.
				if(v2_route_to_prev)
				{
					if(store_prev_match_frame())
					{

						// Prev-batch completion: deliver via copy_data_to_buffer
						// using a temporary pointer swap so the existing
						// compression/decryption/delivery loop runs verbatim
						// against messages_rx_prev[] (the same delivery code
						// path the current-batch uses — no parallel pipeline,
						// no duplicated logic). After delivery, clear prev
						// slots back to FREE and deactivate.
						if(rsp_prev_batch_active
						   && rsp_prev_batch_received_count >= rsp_prev_batch_expected_count)
						{
							// V2 FIX-2 (fact-doc §10): the SACK-completed assembled-block delivery
							// gate. If THIS completing prev-batch is a stashed big-block (armed at
							// the PARTIAL carve, bsi matches), re-verify the whole-block CRC-32 over
							// the K-codeword image reassembled from messages_rx_prev[] BEFORE
							// delivering — a KEPT codeword that the per-cw CRC-8 FALSE-PASSED would
							// otherwise deliver wrong bytes here at the unchanged ~2^-8 floor (the §5
							// residual). On mismatch: do NOT deliver — clear the prev-batch so the
							// CMD's ACK-timeout re-emits the whole block (fresh decode). Big-block-
							// scoped: a non-big-block / non-armed prev completion (bigblock_partial_armed
							// false or bsi mismatch) is byte-identical to before (gate passes through).
							// MERCURY_BIGBLOCK_DEFEAT_PARTIALCRC=1 disables the reject on the SAME binary
							// (restores the pre-fix silent-wrong-byte delivery) for the ARM-G fail-before.
							bool prev_deliver_ok = true;
							if(bigblock_partial_armed
							   && bigblock_partial_block_bsi == rsp_prev_batch_seq_id)
							{
								bool defeat_partialcrc = false;
								{ const char* e = std::getenv("MERCURY_BIGBLOCK_DEFEAT_PARTIALCRC");
								  if(e && *e && atoi(e)!=0) defeat_partialcrc = true; }
								bool crc_ok = bigblock_partial_block_crc_ok();
								bigblock_partial_armed = false;   // one-shot: consumed at completion
								if(!crc_ok && !defeat_partialcrc)
									prev_deliver_ok = false;
							}
							if(!prev_deliver_ok)
							{
								// REJECT: a kept codeword false-passed its per-cw CRC-8 -> the assembled
								// block is byte-wrong. Drop the prev-batch WITHOUT delivering and WITHOUT
								// a clean ACK, so the CMD never sees this block ACKed and its ACK-timeout
								// re-emits the whole block (a fresh decode is independent of this carve).
								for(int i=0; i<this->nMessages; i++)
									messages_rx_prev[i].status = FREE;
								rsp_prev_batch_active            = false;
								rsp_prev_batch_received_count    = 0;
								rsp_prev_batch_expected_count    = 0;
								rsp_prev_batch_blockcrc_reject_count++;
								printf("[RSP-V2-PREV-BLOCKCRC-REJECT] prev_batch_seq_id=%d NOT delivered "
									"(assembled-block CRC-32 mismatch: a kept codeword FALSE-PASSED per-cw "
									"CRC-8); prev dropped -> CMD re-emits the block (reject_count=%lld)\n",
									rsp_prev_batch_seq_id, rsp_prev_batch_blockcrc_reject_count);
								fflush(stdout);
							}
							else
							{
							// D3.1 (data-integrity, the UNIFIED keystone): DELIVERY-TIME
							// gap gate on the PREV commit. UNLIKE the BATCH-DONE site the
							// prev copy (below) normally runs BEFORE advance_last_delivered
							// (:877); here we move the contiguity check BEFORE the copy so a
							// gapped prev is NEVER pushed to the app FIFO (check-then-deliver,
							// symmetric with BATCH-DONE). A prev commit that steps the
							// high-water by >=2 means an EARLIER batch (between last_delivered
							// and this prev) was never delivered -> a HOLE. NOTE: a backward
							// late-older-prev (fwd in [129,255]) is NOT a gap (it lands behind
							// the high-water; advance() ignores it, audit R1) so this gate is
							// inert on the legitimate out-of-order-prev recovery — it ONLY
							// fires on a genuine forward skip. MERCURY_GAP_ABORT_DEFEAT=1
							// disables it (the --test-inorder-demote fail-before arm).
							bool prev_gap_aborted = false;
							{
								bool gap_defeat = false;
								{ const char* e = std::getenv("MERCURY_GAP_ABORT_DEFEAT");
								  if(e && *e && atoi(e)!=0) gap_defeat = true; }
								// rsp_stream_aborted: refuse the PREV delivery unconditionally
								// once the session is latched torn-down (see the re-adopt gate).
								if(!gap_defeat
								   && (rsp_stream_aborted
								       || delivery_step_is_gap(rsp_prev_batch_seq_id,
								                               rsp_last_delivered_batch_seq_id, rsp_stream_origin_gen)))
								{
									char reason[112];
									snprintf(reason, sizeof(reason),
										"delivery-time PREV bsi=%d non-contiguous with last_delivered=%d (dropped-batch hole)",
										rsp_prev_batch_seq_id, rsp_last_delivered_batch_seq_id);
									rsp_gap_abort_teardown(reason);
									// Torn down: do NOT deliver this prev (the teardown already
									// FREE'd messages_rx_prev[] + cleared the bsi family).
									v2_route_drop = true;
									prev_gap_aborted = true;
								}
							}
							// Option W CORE — RSP PREV byte-gate (the cross-storage twin of the
							// PRIMARY byte-gate at :2565; data-flow-stream-offset.md §3-PREV / §9 /
							// silent-corruption-residual.md §PREV-gate). The in-order BATCH-DONE
							// path byte-gates the clean ACK against the wire stamp.length, but THIS
							// cross-storage PREV completion delivers messages_rx_prev[] via
							// copy_data_to_buffer (below) UN-gated — a short PREV (mechanism-4:
							// complete-at-shrunk-count) pushes a short/shifted tail that the next
							// batch's positional guard (BACKSTOP / GAP-ABORT) only catches AFTER the
							// leak (c11w006: 12 wrong bytes at stream offset 289; res_c11w006 was
							// cfg0-robust so the stamp did NOT ride there — this gate closes the
							// site for every stamp-riding config cfg13-16 where the 4 mechanisms
							// occur). Gate on DELIVERED bytes for THIS prev bsi over messages_rx_prev[]
							// (the array copy_data_to_buffer delivers after the swap) vs the committed
							// stamp.length; on a shortfall WITHHOLD — do NOT swap/deliver, do NOT
							// advance, do NOT emit the clean ACK — leaving messages_rx_prev[] RECEIVED
							// + rsp_prev_batch_active so the CMD's ACK-timeout retransmits the tail
							// (COMPLETE-or-LOUD, never a silent short credit — the SAME discipline as
							// the primary gate). FALSE-FIRE: an absent/invalid stamp (robust / cfg0
							// tiny-frame / lost EOB / F4.2-invalidated) makes w_bytegate_shortfall
							// return false ⇒ safe no-op; a LEGITIMATE full PREV (delivered==committed)
							// does not trip it. MERCURY_W_PREV_BYTEGATE_DEFEAT=1 restores the pre-fix
							// un-gated PREV delivery (the --test-stream-offset Part-T fail-before arm).
							// Option B' completeness (data-flow-batch-size.md §9): the EFFECTIVE
							// window for THIS cross-storage PREV batch. == data_batch_size in every
							// non-desync / non-v2 case and under MERCURY_BPRIME_DEFEAT; only a
							// genuine CMD>RSP over-count (res_c3100) widens it (<= 32). Derived from
							// the prev batch's authoritative frame count (rsp_prev_batch_expected_
							// count, widened to the wired D5 at :1224 / arq_common.cc:10914). Threaded
							// into the PREV byte-gate, the RECEIVED->ACKED mark loop, the copy_data_
							// to_buffer delivery bound (rx_copy_window), and the clean-ACK bitmap
							// width below, so a widened PREV delivers + credits in FULL instead of
							// truncating at the stale data_batch_size (the on-air withhold, run
							// weioyt1zk: the byte-gate summed over data_batch_size and dropped the
							// widened tail -> permanent [RSP-V2-PREV-BYTE-SHORTFALL]).
							int prev_eff_window = rx_effective_window(rsp_prev_batch_expected_count);
							// GROWTH-TRANSITION integrity: clamp the PREV delivery window DOWN to the
							// prev batch's GENUINE span, never floored UP to a data_batch_size that GREW
							// since the prev was sealed (same stale-tail substitution as rsp_commit).
							{
								bool growth_deliver_defeat = false;
								{ const char* e = std::getenv("MERCURY_GROWTH_DELIVER_DEFEAT");
								  if(e && *e && atoi(e)!=0) growth_deliver_defeat = true; }
								if(!growth_deliver_defeat
								   && rsp_prev_batch_expected_count > 0
								   && rsp_prev_batch_expected_count < prev_eff_window)
									prev_eff_window = rsp_prev_batch_expected_count;
							}
							bool prev_byte_withheld = false;
							if(!prev_gap_aborted)
							{
								bool w_prev_bytegate_defeat = false;
								{ const char* e = std::getenv("MERCURY_W_PREV_BYTEGATE_DEFEAT");
								  if(e && *e && atoi(e)!=0) w_prev_bytegate_defeat = true; }
								if(!w_prev_bytegate_defeat
								   && w_bytegate_shortfall(rsp_prev_batch_seq_id, messages_rx_prev, prev_eff_window))
								{
									printf("[RSP-V2-PREV-BYTE-SHORTFALL] WITHHOLD prev delivery: "
										"bsi=%d committed=%u (frame-count complete but byte shortfall "
										"— short/shrunk PREV); keeping prev pending + RECEIVED, NOT "
										"delivering, NOT crediting. CMD ACK-timeout retransmits the "
										"tail (no leaked short tail).\n",
										rsp_prev_batch_seq_id & 0xFF,
										rx_stream_stamp[rsp_prev_batch_seq_id & 0xFF].length);
									fflush(stdout);
									prev_byte_withheld = true;
								}
							}
							// FIX (a) loud detector — pre-deliver non-FREE assertion
							// (data-flow-messages_rx_prev.md §4.5 CORRECTION). With the honest
							// completion count (== RECEIVED slots in the delivery window),
							// received >= expected IMPLIES every slot in [0, prev_eff_window) is
							// non-FREE. Assert it so a HOLE (a FREE slot inside the delivery window
							// whose count was satisfied by an out-of-window strand) becomes a LOUD
							// withhold, never a silent short/holed delivery — keep the prev RECEIVED
							// + active for the CMD ACK-timeout refill (COMPLETE-or-LOUD). Byte-
							// identical when the window is intact (the count fix keeps it so).
							// MERCURY_W_PREVCOUNT_DEFEAT=1 disables it (the fail-before arm).
							bool prev_hole_withheld = false;
							if(!prev_gap_aborted && !prev_byte_withheld)
							{
								bool w_prevcount_defeat = false;
								{ const char* e = std::getenv("MERCURY_W_PREVCOUNT_DEFEAT");
								  if(e && *e && atoi(e)!=0) w_prevcount_defeat = true; }
								if(!w_prevcount_defeat)
								{
									int hole = -1;
									for(int i=0; i<prev_eff_window && i<this->nMessages; i++)
										if(messages_rx_prev[i].status == FREE) { hole = i; break; }
									if(hole >= 0)
									{
										printf("[RSP-V2-PREV-HOLE-WITHHOLD] prev_batch_seq_id=%d "
											"received=%d/%d has a FREE slot at %d inside the delivery "
											"window [0,%d) — withholding (count satisfied out-of-"
											"window); keeping prev RECEIVED for retx, delivering 0 "
											"bytes (COMPLETE-or-LOUD).\n",
											rsp_prev_batch_seq_id, rsp_prev_batch_received_count,
											rsp_prev_batch_expected_count, hole, prev_eff_window);
										fflush(stdout);
										prev_hole_withheld = true;
									}
								}
							}
							if(!prev_gap_aborted && !prev_byte_withheld && !prev_hole_withheld) {
							if(compressor.is_streaming() && batch_data_delivered) {
								// V1 defense: out-of-order prev-batch delivery would desync the
								// streaming PPMd model. Current batch already committed; prev-batch
								// commit would be against an advanced model. Reset both sides via
								// the bit-2 handshake — next TX batch will detect RX-cold and reset.
								compressor.streaming_reset();
								printf("[STREAMING] Reset: out-of-order prev-batch delivery (V1 defense)\n");
							}
							// Preserve current-batch state during the swap.
							struct st_message* saved_rx     = messages_rx;
							bool saved_data_delivered       = batch_data_delivered;
							messages_rx                     = messages_rx_prev;
							batch_data_delivered            = false;
							printf("[RSP-V2-PREV-DELIVER-BEGIN] prev_batch_seq_id=%d "
								"received=%d/%d (swapping messages_rx pointer for delivery)\n",
								rsp_prev_batch_seq_id,
								rsp_prev_batch_received_count,
								rsp_prev_batch_expected_count);
							fflush(stdout);
							// Mark RECEIVED slots as ACKED so copy_data_to_buffer's
							// ACKED-only iteration picks them up. Option B' (§9): iterate the
							// EFFECTIVE (possibly widened) window, NOT the stale data_batch_size —
							// a res_c3100 widened PREV holds frames in slots [data_batch_size, D5)
							// and those slots must be flipped ACKED or the reassembler (which
							// delivers ACKED-only) silently drops the widened tail. == data_batch_
							// size when not widened (byte-identical). (messages_rx has just been
							// swapped to point at messages_rx_prev.)
							for(int i=0; i<prev_eff_window && i<this->nMessages; i++)
							{
								if(messages_rx[i].status == RECEIVED)
									messages_rx[i].status = ACKED;
							}
							// AEAD nonce source (data-flow-aead-nonce.md §5): the
							// out-of-order PREV completion delivers messages_rx_prev[],
							// whose wire bsi is rsp_prev_batch_seq_id.
							decrypt_delivered_bsi = rsp_prev_batch_seq_id;
							// Option B' (§9): bound the reassembler at the EFFECTIVE (possibly
							// widened) window so a res_c3100 widened PREV delivers ALL declared
							// frames instead of truncating at data_batch_size (the missing
							// consumer that made the widen INERT: delivery stopped at 25 of 30,
							// short-advanced rx_stream_delivered, and the NEXT batch's positional
							// BACKSTOP then correctly caught the shift). Restored to -1 after.
							this->rx_copy_window = prev_eff_window;
							copy_data_to_buffer();
							this->rx_copy_window = -1;
							// Restore current-batch pointer + delivery state.
							messages_rx                     = saved_rx;
							batch_data_delivered            = saved_data_delivered;
							// Clear prev slots back to FREE (copy_data_to_buffer
							// already freed them via the swapped pointer, but
							// double-ensure safety).
							for(int i=0; i<this->nMessages; i++)
								messages_rx_prev[i].status = FREE;
							rsp_prev_batch_active            = false;
							rsp_prev_batch_received_count    = 0;
							rsp_prev_batch_expected_count    = 0;
							rsp_prev_batch_delivered_count++;
							// FIX-8 (data-integrity): the prev (recovered, possibly
							// out-of-order) batch was just delivered to the app.
							// Advance the high-water mark MONOTONIC-with-wrap so a
							// late older prev cannot regress it below a newer current
							// already delivered (audit R1).
							advance_last_delivered(rsp_prev_batch_seq_id);
							rsp_gap_recover_rounds = 0;   // R2a: the prev hole just filled — clear the recoverable-hold counter
							printf("[RSP-V2-PREV-DELIVERED] prev_batch_seq_id=%d "
								"deliveries_total=%lld last_delivered=%d (cross-storage "
								"path drained; current-batch storage untouched)\n",
								rsp_prev_batch_seq_id, rsp_prev_batch_delivered_count,
								rsp_last_delivered_batch_seq_id);
							fflush(stdout);
							// Fix A (baseline-double-delivery.md): the prev batch just
							// delivered its full framed span — now apply any data_batch_size
							// shrink that was deferred to avoid orphaning it.
							rsp_apply_deferred_batch_shrink();

							// C6 MEASURE-ONLY (block-crc-upgrade-design.md §7 [?], bigblock-integrity.md
							// §5/§12): if THIS just-delivered prev-batch is a big-block whose cw(K-1) was
							// the gap (so the whole-block CRC-32 FIX-2 gate could NOT arm — cw(K-1) carries
							// the CRC-32 field) AND >=1 OTHER kept codeword was delivered relying only on
							// the per-cw CRC-8, COUNT it (residual-exposure population §7 quantifies before
							// deciding the CLOSE). DELIVERY IS UNCHANGED — the block was already delivered
							// above; the helper only COUNTS + LOGS. Same production helper the unit test
							// drives, so the increment is exercised, not copied.
							note_bigblock_partial_crc_residual(rsp_prev_batch_seq_id);
							
							// R2b -- DELIVER-HELD-CUR (data-flow-recoverable-gap-abort.md 5.5). The prev
							// hole just filled and delivered (advance_last_delivered above stepped the
							// high-water to prev == last+1). If a current batch is being HELD behind this
							// hole (rsp_gap_hold_cur_expected>0) and it is now CONTIGUOUS (the delivery
							// ruler reads not-a-gap by construction: prev was last+1, cur is last+2 ==
							// new-last+1) and still fully present in messages_rx[], commit it through the
							// SAME production delivery path the BATCH-DONE gate uses -- HERE, BEFORE the
							// prev-delivered clean-ACK emit below -- so that emit's cumulative_ack_bsi_
							// field(n_r) confirms the held cur in the SAME reverse frame. No false-ACK
							// window: the CMD's registration of the held batch is semantically TRUE because
							// the held batch IS delivered first. The prev-N-specific bookkeeping
							// (rsp_apply_deferred_batch_shrink, note_bigblock_partial_crc_residual) has
							// ALREADY run above with rsp_prev_batch_seq_id==prev, so the commit's bump of
							// rsp_prev_batch_seq_id to the held bsi is safe. D3.1 keystone UNTOUCHED: the
							// inline delivery_step_is_gap ruler stays the inviolable guard and
							// rsp_stream_aborted refuses unconditionally. Knob
							// MERCURY_HELD_CUR_DELIVER_DEFEAT=1 restores the pre-fix never-delivered hold
							// (fail-before: the held cur strands and the bounded hold aborts).
							{
								bool held_deliver_defeat = false;
								{ const char* e = std::getenv("MERCURY_HELD_CUR_DELIVER_DEFEAT");
								  if(e && *e && atoi(e)!=0) held_deliver_defeat = true; }
								if(!held_deliver_defeat
								   && sack_v2_enabled
								   && rsp_gap_hold_cur_expected > 0
								   && rsp_current_expected_batch_seq_id >= 0
								   && !rsp_stream_aborted
								   && !delivery_step_is_gap(rsp_current_expected_batch_seq_id,
								                            rsp_last_delivered_batch_seq_id, rsp_stream_origin_gen))
								{
									// The held cur must still be FULLY present: every slot [0, expected-1]
									// of messages_rx[] RECEIVED. The retx that filled the prev hole routed
									// to PREV storage (messages_rx_prev[]) and must not have disturbed the
									// held CURRENT storage.
									bool all_present = true;
									for(int i=0; i<rsp_gap_hold_cur_expected && i<this->data_batch_size
									             && i<this->nMessages; i++)
										if(messages_rx[i].status != RECEIVED) { all_present = false; break; }
									if(all_present)
									{
										int held_bsi = rsp_current_expected_batch_seq_id;
										rsp_commit_cur_batch_delivery();   // SAME production commit as BATCH-DONE
										rsp_gap_hold_cur_expected = 0;
										printf("[RSP-V2-HELD-CUR-DELIVERED] held cur bsi=%d delivered in-order "
											"after prev refill; last_delivered=%d next_expected=%d\n",
											held_bsi, rsp_last_delivered_batch_seq_id,
											rsp_current_expected_batch_seq_id);
										fflush(stdout);
									}
									else
									{
										// Held storage was disturbed (an unrelated demote/BREAK freed
										// messages_rx[], retx mis-routing, etc.). Do NOT deliver a partial:
										// drop the held marker and let the CMD ACK-timeout re-drive the cur
										// batch through the normal (now-contiguous) BATCH-DONE path.
										printf("[RSP-V2-HELD-CUR-INCOMPLETE] held cur bsi=%d not fully present "
											"(expected=%d) -- NOT delivering; CMD re-drives via BATCH-DONE\n",
											rsp_current_expected_batch_seq_id, rsp_gap_hold_cur_expected);
										fflush(stdout);
										rsp_gap_hold_cur_expected = 0;
									}
								}
							}

							// climb-engine Bug 1 (gearshift-climb-engine.md §4): emit a CLEAN
							// (all-ones) MFSK ACK+SACK for the prev batch we just FULLY delivered
							// via the retransmit/prev-storage path. Historically this path delivered
							// to the app (copy_data_to_buffer above) but sent NO ACK, so the CMD never
							// saw a clean ACK for the batch: last_batch_fully_acked stayed FALSE and the
							// rung that recovered a frame via SACK could never promote. Reuse the SAME
							// clean-ACK transport as the no-loss ACK-GATE funnel (arq_responder.cc:1644).
							// The CMD-side split dedupe (sack_clean_confirmation_accepted) now accepts
							// this all-ones confirmation even though a PARTIAL for the same bsi was
							// already applied. Promotion still REQUIRES full delivery: a rung that never
							// completes a batch never reaches this block, so the +1 anchor clamp +
							// clean-batch-viability guard are unchanged (no deep-SNR over-climb). This
							// path runs only at batch>=5 (OFDM tier); robust is batch=1 and never uses
							// the prev path.
							{
								unsigned char prev_ack_bsi = (unsigned char)(
									rsp_prev_batch_seq_id >= 0 ? rsp_prev_batch_seq_id : 0);
								// Capability or scalable mode alone is not enough: a lost
								// partial SACK may use the ordinary timeout builder with fresh
								// frames trailing the retry. Immediate confirm is safe only if
								// this completing frame carries the negotiated D5 marker proving
								// it is the physical selective-retry turn tail. The existing D2
								// settle below then supplies PTT-off/PTT-on before keying.
								bool linkphase_defer_prev_ack = this->linkphase_defer_prev_ack(
									linkphase_ackslot_on(), rx_buffer_retx_turn_tail);
								if(linkphase_defer_prev_ack)
								{
									printf("[LINKPHASE-ACK-DEFER] prev-delivered clean-ACK bsi=%u deferred to "
										"block boundary (mid-keydown collision avoided); last_delivered=%d\n",
										(unsigned)prev_ack_bsi, rsp_last_delivered_batch_seq_id);
									fflush(stdout);
									linkphase_prev_ack_deferred++;
									// SEAM-2 ARM (post-delivery: the prev/held-cur bytes are ALREADY committed
									// above, so the invariant "armed only after delivered" holds). The base only
									// bumps the counter here and emits nothing, so the CMD closes its ACK slot
									// with no ACK = a false block failure. Arm a pending flush carrying the
									// cumulative n_r (+ the clean-bitmap width for the MFSK-suffix fallback) so a
									// later boundary emits it EXACTLY ONCE via a bsi-carrying transport.
									// MERCURY_PENDING_CONFIRM_DEFEAT=1 restores the base drop (fail-before).
									linkphase_arm_pending_prev_confirm(prev_ack_bsi, prev_eff_window);
								}
								bool prev_used_mfsk_path = false;
								if (!linkphase_defer_prev_ack
									&& MFSK_ACK_SACK_ENABLED
									&& telecom_system->ack_mfsk.ack_sack_suffix_len() > 0)
								{
									// Phase B Wave 1 flag-day (fact-doc §11.2): bitmap is 30 bits (was 32).
									// Option B' (§9): the clean (all-ones) bitmap must be prev_eff_window
									// wide, NOT data_batch_size wide — a res_c3100 widened PREV (30 frames)
									// delivered in full above must be CREDITED CLEAN across all 30 slots or
									// the CMD (which built the larger batch, all_ones = (1<<30)-1) sees a
									// too-narrow bitmap, never registers the batch as fully acked, and
									// retransmits/desyncs the tail. == data_batch_size when not widened.
									int clean_bits = prev_eff_window;
									uint32_t bitmap_u32 = mfsk_sack_mask_for_frames(clean_bits);
									// FORGIVING-ACK Tier 2 (§T2.0/§T2.3): n_r in the bsi field when
									// negotiated (the prev-deliver high-water was just advanced at :911).
									unsigned char wire_bsi = cumulative_ack_bsi_field(
										prev_ack_bsi, rsp_last_delivered_batch_seq_id, cumulative_ack_enabled);
									printf("[RSP-MFSK-SACK] prev-delivered path: batch_seq_id=%u (wire_bsi=%u n_r=%d cum=%d) bitmap=0x%08x nframes=%d\n",
										(unsigned)prev_ack_bsi, (unsigned)wire_bsi,
										rsp_last_delivered_batch_seq_id, cumulative_ack_enabled ? 1 : 0,
										(unsigned)bitmap_u32, data_batch_size);
									fflush(stdout);
									// WALL-B FIX-9 D2 REFINE (§2.1): prev-delivered = a batch recovered via the
									// SACK/retransmit prev-storage path -> a RETRANSMIT turnaround -> arm the D2
									// robust reverse-ACK geometry (settle) for this ACK.
									ack_tx_retx_turnaround = true;
									long long mfsk_ms = 0;
									bool compact_sent = false;
									if(scalable_sack_on()
									   && telecom_system->ack_mfsk.compact_confirm_suffix_len() > 0)
									{
										mfsk_ms = send_mfsk_compact_confirm(prev_ack_bsi);
										compact_sent = mfsk_ms > 0;
									}
									if(mfsk_ms <= 0)
										mfsk_ms = send_mfsk_ack_sack(prev_ack_bsi, bitmap_u32);
									if (mfsk_ms > 0)
									{
										printf("[TX-ACK-SACK] prev-delivered via %s wire_ms=%lld\n",
											compact_sent ? "COMPACT confirm" : "MFSK suffix", mfsk_ms);
										fflush(stdout);
										prev_used_mfsk_path = true;
									}
									else
									{
										printf("[RSP-MFSK-SACK] MFSK suffix returned 0 on prev-delivered -- falling back to legacy MFSK ACK pattern\n");
										fflush(stdout);
									}
								}
								if (!prev_used_mfsk_path && !linkphase_defer_prev_ack)
								{
									// NB or MFSK-suffix unavailable: legacy MFSK ACK pattern (receiver
									// treats any pattern hit as a clean ACK -- same as the clean funnel).
									send_ack_pattern();
								}
							}
							}   // end D3.1 if(!prev_gap_aborted) prev-delivery body
							}   // end V2 FIX-2 SACK-completed delivery (prev_deliver_ok else-branch close)
						}
					}
					else
					{
						printf("[RSP-V2-PREV-DROP] bsi=%d id=%d len=%d "
							"reason=length_or_loc_out_of_range\n",
							(int)(unsigned char)messages_rx_buffer.batch_seq_id,
							(int)(unsigned char)messages_rx_buffer.id,
							messages_rx_buffer.length);
						fflush(stdout);
					}
					// Frame fully handled by the prev path — DO NOT fall
					// through to match-current storage. The existing
					// `messages_rx_buffer.status=FREE; link_timer.start(); ...`
					// tail (after the v2_route_drop block) runs as today;
					// we just skip the messages_rx[] write + receiving-timer
					// re-arm path. The current-batch receiving timer is left
					// undisturbed (prev path is independent of current-batch
					// ACK-GATE pacing — §4.3.4 #1 "one outstanding batch"
					// still holds via data_ack_received on the CMD side).
				}
				else if(!v2_route_drop)
				{

				{
					static cl_timer batch_rx_stopwatch;
					if(batch_rx_frame_count == 0) batch_rx_stopwatch.start();
					printf("[RX-DATA] type=%d id=%d seq=%d/%d len=%d t=%dms\n",
						messages_rx_buffer.type, (int)(unsigned char)messages_rx_buffer.id,
						messages_rx_buffer.sequence_number, data_batch_size,
						messages_rx_buffer.length,
						(int)batch_rx_stopwatch.get_elapsed_time_ms());
					fflush(stdout);
					mtl::log_event_kv("rsp_data_frame_rxed", "id=%d seq=%d/%d len=%d",
						(int)(unsigned char)messages_rx_buffer.id,
						messages_rx_buffer.sequence_number, data_batch_size,
						messages_rx_buffer.length);
				}
				int store_result = add_message_rx_data(messages_rx_buffer.type,
					messages_rx_buffer.id, messages_rx_buffer.length,
					messages_rx_buffer.data);
				// Fleet-validation gate: unset preserves the legacy ARQ response;
				// enabled rejects the frame before any success-only state advances.
				bool enforce_store_verdict = false;
				{ const char* e = std::getenv("MERCURY_RX_STORE_FAIL_CLOSED");
				  if(e && *e && atoi(e) != 0) enforce_store_verdict = true; }
				if(store_result != SUCCESSFUL && enforce_store_verdict)
				{
					printf("[RSP-RX-STORE-REJECT] type=%d id=%d seq=%d len=%d verdict=%d\n",
						messages_rx_buffer.type,
						(int)(unsigned char)messages_rx_buffer.id,
						messages_rx_buffer.sequence_number,
						messages_rx_buffer.length, store_result);
					fflush(stdout);
				}
				if(store_result == SUCCESSFUL || !enforce_store_verdict)
				{
				batch_rx_frame_count++;
				// §19: session-monotonic forward-progress counter. batch_rx_frame_count RESETS at every
				// batch boundary, so the dead-batch tick guard (arq_common.cc §19) needs a counter that
				// only ever INCREASES while the link delivers DATA. Incremented here, at the SAME
				// confirmed-storage point batch_rx_frame_count advances; reset only at session start/BREAK.
				inband_total_data_frames_rx++;
				// R038 (race audit 2026-06-06): promote the v2 EOB ONLY now, inside
				// the confirmed match-current storage block. For v2, receive()
				// STAGED the decoded EOB seq on rx_buffer_eob_seq instead of writing
				// last_received_end_of_batch_seq pre-routing, so a prev-retransmit /
				// late-duplicate of a shorter batch can no longer poison the current
				// batch's effective_batch (the gate below + the final ACK-GATE read
				// last_received_end_of_batch_seq). This block is reached for v2 ONLY
				// when match_current (match-prev routes to the prev block; OOW/drop
				// sets v2_route_drop) — see arq_responder.cc:615-654. v1 has no bsi
				// routing: receive() already set last_received_end_of_batch_seq and
				// rx_buffer_eob_seq stays -1, so v1 is left byte-for-byte unchanged.
				if(sack_v2_enabled && rx_buffer_eob_seq >= 0)
					last_received_end_of_batch_seq = rx_buffer_eob_seq;
				// D5 (TRACK_C_D2D3D5_DESIGN.md §5.3): promote the wired per-batch frame
				// count ONLY now, inside the confirmed match-current block (same staging
				// discipline as the EOB above). A non-zero count on ANY frame of the batch
				// makes the authoritative length survive the loss of the EOB frame. -1
				// (unknown) leaves rx_batch_total_frames unchanged so a later frame that
				// DOES carry it can still latch it.
				if(sack_v2_enabled && rx_buffer_batch_total_frames > 0)
					rx_batch_total_frames = rx_buffer_batch_total_frames;
				int rx_timeout = 0;
				int effective_batch = data_batch_size;
				// D5 (TRACK_C_D2D3D5_DESIGN.md §5.3): does the wired per-batch frame
				// count override the EOB inference for THIS current batch's ACK gate?
				bool d5_infer_defeat_cur = false;
				{ const char* e = std::getenv("MERCURY_D5_INFER_DEFEAT");
				  if(e && *e && atoi(e)!=0) d5_infer_defeat_cur = true; }
				{
					// Determine actual expected frame count.
					// With adaptive batch sizing, commander may send fewer frames
					// than data_batch_size. Use compression header to detect this.
					// D5: PREFER the wired authoritative count. With a lost EOB frame
					// the inference (last_received_end_of_batch_seq+1) is the highest
					// OTHER seq seen, which is SHORT — it would ACK-GATE the current
					// batch as complete with the tail missing (silent truncation). The
					// wired count keeps effective_batch at the true length, so
					// batch_rx_frame_count < effective_batch and the batch falls to the
					// SACK path (which now spans + requests the lost tail). Fallback to
					// the EOB inference when the count is unknown / under DEFEAT.
					if(rx_batch_total_frames > 0 && !d5_infer_defeat_cur)
					{
						int wired = rx_batch_total_frames;
						if(wired < effective_batch)
							effective_batch = wired;
					}
					// ENC BATCH-SIZE FIX (data-flow-encrypted-batch-size.md §4/§5): for an
					// ENCRYPTED batch with no wired count, do NOT let the EOB inference LOWER
					// effective_batch — a short effective_batch makes the receiving-window timer
					// ACK-GATE the batch early with the tail missing, reassembling a TRUNCATED
					// ciphertext -> whole-batch AEAD auth-fail -> false PSK-mismatch disconnect.
					// Hold effective_batch at the deterministic crypto span (min(data_batch_size,
					// crypto_batch_size), the TX seal bound) so the timer waits for the full
					// (decryptable) set; SACK then recovers the missing tail.
					else if(cipher_suite.is_active())
					{
						int crypto_span = data_batch_size;
						if(crypto_batch_size > 0 && crypto_batch_size < crypto_span)
							crypto_span = crypto_batch_size;
						if(crypto_span < effective_batch)
							effective_batch = crypto_span;
					}
					// End-of-batch flag: commander marks last frame with bit 7
					else if(last_received_end_of_batch_seq >= 0)
					{
						int eob = last_received_end_of_batch_seq + 1;
						if(eob < effective_batch)
							effective_batch = eob;
					}
					else if(compression_enabled
						&& !cipher_suite.is_active()
						&& !compressor.is_streaming()  // §7.13.37 — same fix as line 1071 (which §7.13.35 guarded); this per-frame-timer copy was missed. With streaming, frame-0's compression header describes only the first compressed message in the batch (e.g., 3424 bytes → 21 frames) but CMD continues packing more messages into frames 22-24. Without the guard, effective_batch was being lowered to 21 here, causing batch_rx_frame_count >= effective_batch to be true at frame 20 → rx_timeout=ptt_on_delay_ms=100ms → RSP times out and ACKs early DURING CMD's still-active TX → ACK lost in CMD's mute window → bps=0 (gearshift_v22 finding; mediator missed this code path because exp=25 from the LATER ACK-GATE-DIAG calc fired AFTER this early timer had already triggered).
						&& messages_rx[0].status == RECEIVED
						&& messages_rx[0].length >= compressor.get_header_size())
					{
						// Fallback: compression header (non-streaming compression only).
						const unsigned char* hdr = (const unsigned char*)messages_rx[0].data;
						int hdr_comp = hdr[1] | (hdr[2] << 8);
						int gate_hdr_size = compressor.get_header_size();
						int total_compressed = gate_hdr_size + hdr_comp;
						// SACK Design A Step 1 — effective DATA_LONG header.
						int mf = max_data_length + max_header_length - effective_data_long_header_length(sack_v2_enabled, header_carries_d5);
						int hdr_expected = (total_compressed + mf - 1) / mf;
						if(hdr_expected < 1) hdr_expected = 1;
						if(hdr_expected < effective_batch)
							effective_batch = hdr_expected;
					}

					if(batch_rx_frame_count >= effective_batch)
					{
						// All expected frames decoded -- ACK immediately.
						rx_timeout = ptt_on_delay_ms;
					}
					else if(sack_enabled && last_received_end_of_batch_seq >= 0)
					{
						// Fix B (SACK turnaround, SACK_TURNAROUND_FIX_PLAN.md §8.3):
						// EOB-frame fast path. The commander marks its LAST DATA
						// frame with bit 7 of sequence_number; decoding it sets
						// last_received_end_of_batch_seq >= 0. That is POSITIVE
						// PROOF the commander has finished transmitting this batch
						// -- so RSP must NOT wait out the full remaining-batch
						// estimate before SACKing. Use a short turnaround
						// (ptt_on + ptt_off, ~300 ms) so the receiving timer
						// expires just after the EOB frame and RSP SACKs the gaps
						// immediately. send_sack_v2_frame()'s own TX path supplies
						// the precise CMD-drain margin. Only correct paired with Fix A:
						// if the EOB frame itself is lost this branch never runs
						// and Fix A's per-frame rx_timeout is the fallback.
						rx_timeout = ptt_on_delay_ms + ptt_off_delay_ms;
					}
					else
					{
						// More frames expected.  Add one msg_time margin for
						// FAIL recovery (false peaks in old frame body).
						int remaining = effective_batch - messages_rx_buffer.sequence_number - 1;
						if(remaining < 0) remaining = 0;
						rx_timeout = remaining * message_transmission_time_ms
							+ time_left_to_send_last_frame + ptt_on_delay_ms
							+ message_transmission_time_ms;
					}
				}
				// Fix A (SACK turnaround, SACK_TURNAROUND_FIX_PLAN.md §8.2):
				// re-arm the receiving timer on EVERY decoded DATA frame with the
				// per-frame rx_timeout computed above. rx_timeout is the
				// remaining-batch estimate anchored to which seq just arrived and
				// to the measured frame geometry (message_transmission_time_ms) --
				// NOT an open-loop budget. This is the §4.1 idle timer in its
				// structurally-correct form: each frame re-arms the timer for "the
				// whole rest of the batch", so RSP only ACK-GATEs once the channel
				// has genuinely been idle longer than the entire remaining batch
				// could take. Previously the sack_enabled+incomplete path threw
				// rx_timeout away (KEEP did nothing; RESTART restarted the timer
				// but not the timeout), so the timer ran on a stale value started
				// mid-batch -- it expired 6-9 s before CMD finished (§6.4). The
				// non-SACK path already did exactly this every frame; Fix A makes
				// the SACK path identical to it, so the --enable-sack-OFF path is
				// byte-for-byte unchanged.
				printf("[RSP-TIMER] SET: sack=%d rxcnt=%d eff=%d seq=%d eob=%d rx_t=%d old_t=%d\n",
					sack_enabled?1:0, batch_rx_frame_count, effective_batch,
					messages_rx_buffer.sequence_number,
					last_received_end_of_batch_seq, rx_timeout, receiving_timeout);
				fflush(stdout);
					set_receiving_timeout(rx_timeout);
					receiving_timer.start();
					// A complete D5 batch is now in storage.  Produce PREKEY authority
					// only if every clean-ACK withhold gate agrees, then let the normal
					// scheduler overlap key-up/prefix audio with the observed clean tail.
					t1_authorize_clean_ack_if_ready();
					t1_schedkey_poll();
					}  // end accepted (or compatibility-mode) add_message_rx_data store

				}  // end if(!v2_route_drop) — SACK Design A Step 4 routing decision
			}
			messages_rx_buffer.status=FREE;
			link_timer.start();
			watchdog_timer.start();
			gear_shift_timer.stop();
			gear_shift_timer.reset();
		}
	}
	else
	{
		if (messages_control.status==RECEIVED)
		{
			process_control_responder();
		}
		if ( get_nReceived_messages()!=0)
		{
			connection_status=ACKNOWLEDGING_DATA;
		}
		// SEAM-2 FLUSH-B: the RX-timeout expired with NO current-batch receive work
		// (get_nReceived_messages()==0 = the seed106 prev-only topology). The channel
		// has been idle longer than the whole remaining batch, so the CMD is parked in
		// its ACK slot, not mid-keydown (collision-safe). A deferred prev-confirm has no
		// covering ACK-gate emit to ride, so flush it exactly once here.
		else if(linkphase_pending_prev_confirm)
		{
			linkphase_flush_pending_prev_confirm();
		}

		receiving_timer.stop();
		receiving_timer.reset();

		// M4 (SACK turnaround trace): the RX-timeout just expired — this is the
		// root trigger that pushes the RSP state machine into
		// ACKNOWLEDGING_DATA. M4 = timer expired; M3 = ACK-GATE handler
		// running. Both are kept so scheduler latency between them is visible.
		mtl::log_event_kv("rsp_rx_timeout_fired", "timeout=%d rx_count=%d",
			receiving_timeout, batch_rx_frame_count);

		if(link_status == CONNECTED && batch_rx_frame_count == 0)
		{
			printf("[RX-TIMEOUT] No frames decoded. cfg=%d Nsymb=%d M=%.0f nBits=%d ftr=%d batch=%d timeout=%d\n",
				current_configuration,
				telecom_system->data_container.Nsymb,
				telecom_system->M,
				telecom_system->data_container.nBits,
				telecom_system->data_container.frames_to_read.load(),
				data_batch_size,
				receiving_timeout);
			fflush(stdout);

			// Keep the CONFIG_TAG accept override armed across ordinary receive-window
			// restarts: the commander's repeat-until-followed tag cadence can be longer
			// than one low-rate receive timeout.  The 60 s cap is still finite; after
			// it expires a new scream may request a fresh recovery attempt.
			if(scream_reentry_listen_armed
			   && scream_reentry_listen_timer.get_elapsed_time_ms() >= 60000)
			{
				scream_reentry_listen_armed = false;
				scream_reentry_listen_timer.stop();
				scream_reentry_listen_timer.reset();
				printf("[SCREAM-LISTEN] 60000ms CONFIG_TAG window expired without accept; retry eligible\n");
				fflush(stdout);
			}
			else if(scream_reentry_listen_armed)
			{
				printf("[SCREAM-LISTEN] CONFIG_TAG arm retained across RX timeout (%d/60000ms)\n",
					scream_reentry_listen_timer.get_elapsed_time_ms());
				fflush(stdout);
			}

			// T2 SCREAM WAKE: an empty scheduled receive window is only a proposal.
			// Arm after three in-order delivered batches, then require continuous
			// emptiness for a hardware-scaled holdoff. This makes the burst a last-
			// resort dead-forward-path wake rather than an early-session reflex.
			// CONFIG_0 has no lower re-entry rung, so do not scream at the floor.
			if(scream_wake_feature_enabled()
			   && !scream_reentry_listen_armed
			   && !scream_reentry_recovery_pending
			   && is_ofdm_config(current_configuration)
			   && config_ladder_index(current_configuration)
			      > config_ladder_index(CONFIG_0))
			{
				if(scream_forward_batches_delivered < 3)
				{
					printf("[SCREAM-SUPPRESS] empty window before forward-progress watermark (%d/3) cfg=%d\n",
						scream_forward_batches_delivered, current_configuration);
					fflush(stdout);
				}
				else if(!scream_empty_holdoff_armed)
				{
					scream_empty_holdoff_ms =
						scream_trigger_holdoff_for_timeout(receiving_timeout);
					scream_empty_holdoff_armed = true;
					scream_empty_holdoff_timer.start();
					printf("[SCREAM-HOLDOFF] first empty window after %d delivered batches; require %dms persistent emptiness cfg=%d\n",
						scream_forward_batches_delivered, scream_empty_holdoff_ms,
						current_configuration);
					fflush(stdout);
				}
				else
				{
					int empty_ms = scream_empty_holdoff_timer.get_elapsed_time_ms();
					if(scream_trigger_policy_ready(scream_forward_batches_delivered,
							scream_empty_holdoff_armed, empty_ms,
							scream_empty_holdoff_ms))
					{
						int rung = scream_choose_rung();
						printf("[SCREAM-TRIGGER] persistent forward path empty for %dms cfg=%d rung=%d delivered=%d\n",
							empty_ms, current_configuration, rung,
							scream_forward_batches_delivered);
						fflush(stdout);
						scream_empty_holdoff_armed = false;
						scream_empty_holdoff_timer.stop();
						scream_empty_holdoff_timer.reset();
						scream_empty_holdoff_ms = 0;
						send_scream_pattern(rung);
					}
				}
			}

			// Restart timer so we can receive the next CMD retransmit.
			// Without this, the stopped timer returns 0 forever and the
			// RSP is stuck in RECEIVING (0 < timeout always true, but
			// the else block never triggers again).
			calculate_receiving_timeout();
			if(scream_empty_holdoff_armed)
			{
				int remaining = scream_empty_holdoff_ms
					- scream_empty_holdoff_timer.get_elapsed_time_ms();
				if(remaining < 1) remaining = 1;
				if(remaining < receiving_timeout) receiving_timeout = remaining;
			}
			receiving_timer.start();
		}
		else if(responder_terminal_hold_timer.counting)
		{
			// A settled responder is deliberately not CONNECTED, but it must
			// continue opening receive windows until the bounded duplicate-CLOSE
			// hold expires. Without this branch the first empty window stops the
			// timer at zero and a later decoded duplicate is never consumed.
			calculate_receiving_timeout();
			int remaining = TERMINAL_EOT_RSP_HOLD_MS
				- responder_terminal_hold_timer.get_elapsed_time_ms();
			if(remaining < 1) remaining = 1;
			if(receiving_timeout > remaining) receiving_timeout = remaining;
			receiving_timer.start();
		}

		// If we responded to HAIL but START_CONNECTION never arrived,
		// go back to HAIL scanning for the next beacon.
		if(link_status == LISTENING && hail_detected == YES)
		{
			printf("[HAIL] Timeout waiting for START_CONNECTION, resuming HAIL scan\n");
			fflush(stdout);
			hail_detected = NO;
			// Release the scanner: we announced a bare PENDING when the directed
			// HAIL suffix-matched MYCALL, but the START_CONNECTION never decoded
			// (a genuine turnaround miss, or a false-positive suffix match). Emit
			// CANCELPENDING+DISCONNECTED once (idempotent, latched) so the host
			// resumes scanning -- a HAIL-accept that never connects self-heals.
			rsp_emit_release();
			// CONNECT-FAST-CONFIG revert (RESPONDER): we HAILed back but the fast-config
			// START_CONNECTION never decoded on this channel. Revert the listen config to the
			// incumbent ROBUST_0 so the reverted commander's ROBUST_0 HAIL + START_CONNECTION
			// re-syncs (HAIL is config-independent — a mismatch degrades to a re-HAIL, never a
			// strand). Latched: fires once per session, symmetric with the COMMANDER revert.
			if(connect_fast_active)
			{
				printf("[CONNECT-FALLBACK] responder: fast connect (CONFIG_%d) listen timed out "
					"— reverting listen to CONFIG_%d (connect fallback)\n",
					connect_fast_config, connect_fast_fallback_config);
				fflush(stdout);
				connect_fast_active = false;
				robust_enabled = connect_fast_fallback_robust;
				init_configuration = connect_fast_fallback_config;
				data_configuration = connect_fast_fallback_config;
				ack_configuration  = connect_fast_fallback_config;
				last_data_viable_config = session_floor_anchor(robust_enabled, init_configuration);
				load_configuration(connect_fast_fallback_config, FULL, YES);
			}
		}
	}
}

void cl_arq_controller::rsp_reset_session_preserve_close_for_ack()
{
	// reset_session_state() correctly clears every control slot at a session
	// boundary. This CLOSE is the sole exception: the responder still owes the
	// peer the ACK proving it consumed the integrity-checked terminal frame.
	// Keep the received bytes and re-arm only the primary slot; the ACK-complete
	// branch frees it before entering the bounded duplicate-CLOSE receive hold.
	reset_session_state();
#ifndef RSP_CLOSE_ACK_REARM_FAILBEFORE
	messages_control.status = RECEIVED;
#endif
}

bool cl_arq_controller::rsp_terminal_reack_hold_expired(int elapsed_ms) const
{
	return elapsed_ms >= TERMINAL_EOT_RSP_HOLD_MS;
}

void cl_arq_controller::rsp_enter_terminal_reack_hold()
{
	// The session state and application boundary were already finalized before
	// the terminal ACK. Keep only the receive PHY at the just-settled data config
	// long enough to decode the commander's bounded identical CLOSE retries.
	// The mailbox is FREE, so the next-session invariant remains true.
	link_status = DISCONNECTING;
	connection_status = RECEIVING;
	messages_control.status = FREE;
	messages_control_bu.status = FREE;
	receiving_timer.stop();
	receiving_timer.reset();
	receiving_timer.start();
	if(!responder_terminal_hold_timer.counting)
	{
		responder_terminal_hold_timer.reset();
		responder_terminal_hold_timer.start();
	}
	printf("[RSP-EOT-HOLD] terminal ACK sent; retaining settled PHY for %dms "
	       "(replays=%d/%d, mailbox=FREE)\n",
		(int)TERMINAL_EOT_RSP_HOLD_MS, settled_close_cache.replays,
		(int)TERMINAL_EOT_RETRIES);
	fflush(stdout);
}

void cl_arq_controller::rsp_finish_terminal_reack_hold(const char* reason)
{
	responder_terminal_hold_timer.stop();
	responder_terminal_hold_timer.reset();
	settled_close_cache.valid = false;
	load_configuration(init_configuration,FULL,YES);
	link_status = LISTENING;
	connection_status = RECEIVING;
	reset_all_timers();
	telecom_system->data_container.frames_to_read =
		telecom_system->data_container.preamble_nSymb +
		telecom_system->data_container.Nsymb;
	telecom_system->data_container.nUnder_processing_events = 0;
	fifo_buffer_tx.flush();
	fifo_buffer_backup.flush();
	fifo_buffer_rx.flush();
	messages_control.status = FREE;
	messages_control_bu.status = FREE;
	printf("[RSP-EOT-HOLD] settled PHY hold complete (%s); LISTENING on init config, mailbox=FREE\n",
		reason != NULL ? reason : "bounded");
	fflush(stdout);
}

void cl_arq_controller::process_messages_acknowledging_control()
{
	message_batch_counter_tx=0;
	printf("[ACK-CTRL] status=%d (need %d=RECEIVED), code=%d, ack_cfg=%d\n",
		messages_control.status, RECEIVED, (int)messages_control.data[0], ack_configuration);
	fflush(stdout);
	if(messages_control.status==RECEIVED)
	{
		messages_control.type=ACK_CONTROL;
		messages_control.status=ACKED;
		stats.nAcks_sent_control++;

		// Bug #36: reset nUnder BEFORE ACK TX so only turnaround-period
		// nUnder is counted (not accumulated LISTENING nUnder).
		telecom_system->data_container.nUnder_processing_events = 0;

		if(messages_control.data[0] == KEY_EXCHANGE_1
		   || messages_control.data[0] == KEY_EXCHANGE_2
		   || messages_control.data[0] == KEY_EXCHANGE_3)
		{
			// KEY_EXCHANGE_1: LDPC ACK carries responder's pubkey.
			// KEY_EXCHANGE_2: bare LDPC ACK (length=1) confirming one received KX2
			//   encaps-key chunk so the commander queues the next chunk
			//   (per-chunk stop-and-wait, arq_commander.cc:7184). No payload back —
			//   the responder only sends data on KX3. Sent on data_configuration
			//   (OFDM), the same config the commander listens on for the KX2 ACK.
			// KEY_EXCHANGE_3: LDPC ACK carries a chunk of the ML-KEM ciphertext
			//   (hybrid upgrade, MLKEM_HYBRID_PLAN.md §4) — same LDPC reply
			//   transport on the OFDM data_configuration. messages_control was
			//   populated by kx_send_next_chunk() (RSP role) with the 4B-headed
			//   chunk. After this frame is ACKed by the CMD, the CMD's KX3 chunk
			//   handler advances; the RSP queues the next chunk on its next tick.
			// Send on data_configuration (OFDM) — NOT ack_configuration (MFSK).
			// Commander will load data_configuration to receive this.
			printf("[ACK-CTRL] Sending LDPC ACK for code 0x%02X on config %d\n",
				(unsigned char)messages_control.data[0], data_configuration);
			fflush(stdout);
			telecom_system->set_mfsk_ctrl_mode(false);  // full OFDM frame
			messages_batch_tx[message_batch_counter_tx]=messages_control;
			message_batch_counter_tx++;
			pad_messages_batch_tx(control_batch_size);
			send_batch();
		}
		else if(messages_control.data[0] == TEST_CONNECTION_ACK)
		{
			// Phase B Wave 2 v2 — PHY swap site C (fact-doc §13.2).
			// When WB and the MFSK codec is available, emit the MFSK
			// TEST_ACK suffix instead of an LDPC TEST_CONNECTION_ACK frame.
			// messages_control.data[1..2] were populated by the
			// TEST_CONNECTION consumer at arq_responder.cc:1930-1934 with
			// the echoed_cap / own_cap pair we need. SSID comes from
			// my_call_sign's SSID. The CMD-side detector (Site D) will
			// synthesize an LDPC-shaped messages_control on its end so the
			// existing process_control_commander() runs unchanged.
			bool mfsk_test_ack_path =
				narrowband_enabled != YES
				&& telecom_system->ack_mfsk.connect_pattern_nsymb > 0;
			bool mfsk_tx_done = false;
			if(mfsk_test_ack_path)
			{
				uint8_t echoed_cap = (uint8_t)messages_control.data[1];
				uint8_t own_cap    = (uint8_t)messages_control.data[2];
				uint8_t ssid       = (uint8_t)callsign_get_ssid(my_call_sign);
				long long elapsed = send_mfsk_test_ack_phy(echoed_cap, own_cap, ssid);
				if(elapsed > 0)
				{
					printf("[ACK-CTRL-V2] MFSK TEST_ACK sent (%lld ms) "
						"echoed=0x%02X own=0x%02X ssid=%u\n",
						elapsed, echoed_cap, own_cap, ssid);
					fflush(stdout);
					mfsk_tx_done = true;
				}
				else
				{
					printf("[ACK-CTRL-V2] MFSK TEST_ACK unavailable (codec guard "
						"tripped) — falling back to LDPC TEST_CONNECTION_ACK\n");
					fflush(stdout);
				}
			}

			if(!mfsk_tx_done)
			{
				// Legacy LDPC TEST_CONNECTION_ACK: must use LDPC ACK to carry
				// caps echo + CRC. Mirror the legacy ACK dichotomy:
				//   ack_pattern_time_ms > 0  → OFDM ARQ session: send LDPC on
				//                              data_configuration (same as
				//                              KEY_EXCHANGE_1 path)
				//   ack_pattern_time_ms == 0 → MFSK/ROBUST session: send LDPC on
				//                              ack_configuration with mfsk_ctrl_mode
				//                              (same as legacy LDPC fallback below).
				//                              4-byte payload fits easily in either.
				if(ack_pattern_time_ms > 0)
				{
					printf("[ACK-CTRL] Sending LDPC TEST_CONNECTION_ACK on data_config %d (OFDM)\n",
						data_configuration);
					fflush(stdout);
					telecom_system->set_mfsk_ctrl_mode(false);  // full OFDM frame
					messages_batch_tx[message_batch_counter_tx]=messages_control;
					message_batch_counter_tx++;
					pad_messages_batch_tx(control_batch_size);
					send_batch();
				}
				else
				{
					printf("[ACK-CTRL] Sending LDPC TEST_CONNECTION_ACK on ack_config %d (MFSK)\n",
						ack_configuration);
					fflush(stdout);
					load_configuration(ack_configuration, PHYSICAL_LAYER_ONLY, NO);
					telecom_system->set_mfsk_ctrl_mode(true);
					messages_batch_tx[message_batch_counter_tx]=messages_control;
					message_batch_counter_tx++;
					pad_messages_batch_tx(ack_batch_size);
					send_batch();
					load_configuration(data_configuration, PHYSICAL_LAYER_ONLY, YES);
					// HYBRID SET_CONFIG CROSS: run the SAME OFDM-entry ring setup the
					// unilateral adopt runs (shrink the oversized robust-floor ring back to
					// the natural OFDM geometry) so a robust->OFDM data-config cross does not
					// strand the ring-shrink (data-flow-robust-ofdm-adopt-flush.md §12).
					if(inband_rate_feature_enabled() && is_ofdm_config(data_configuration))
						inband_finalize_ofdm_adopt_ring(data_configuration);
				}
			}
			// A fast connect completed the handshake at the fast config; the TEST_CONNECTION_ACK
			// has now been sent at that config (its geometry is config-dependent, so it MUST air
			// BEFORE any reseat). Reseat the responder to the restored pin/data config here,
			// symmetric with the commander success restore: the MFSK-suffix and OFDM TEST_ACK
			// paths above do not, which stranded the responder at the fast config while the
			// commander transmitted DATA at the pin (block failures / BREAK churn until a late
			// precook swap caught up). No-op for a non-fast connect (current == data already).
			// [WBDIRECT-REGRESSION] fail-before hook: MERCURY_CONNECT_FAST_DEFEAT_RESEAT=1
			// SKIPS the responder live-PHY reseat too, symmetric with the commander, so the
			// SAME binary reproduces the pre-fix strand. Production NEVER sets it.
			if(data_configuration != current_configuration)
			{
				const char* cfdr = std::getenv("MERCURY_CONNECT_FAST_DEFEAT_RESEAT");
				if(!(cfdr && *cfdr && atoi(cfdr) != 0))
				{
					load_configuration(data_configuration, PHYSICAL_LAYER_ONLY, YES);
					if(inband_rate_feature_enabled() && is_ofdm_config(data_configuration))
						inband_finalize_ofdm_adopt_ring(data_configuration);
				}
				else
					printf("[CONNECT-FAST] DEFEAT_RESEAT: responder live-PHY reseat SKIPPED "
						"(fail-before; stranded at CONFIG_%d)\n", current_configuration);
			}
		}
		else if(ack_pattern_time_ms > 0)
		{
			// During turboshift: send ACK + SNR suffix so commander can SUPERSHIFT.
			// ACTIVE-v2 reaches SET_CONFIG only for the enumerated cross-tier or
			// tag-carrier-unavailable exceptions. Those dedicated-ACK transitions
			// still need the extended response; sending a bare ACK would strand the
			// commander after this responder adopts the target. The SNR payload is
			// advisory telemetry; it is not an admission gate for the v2 policy.
			bool is_coordinated_setconfig =
				(turboshift_active || turboshift_phase != TURBO_DONE ||
				 rate_opt.controls_link()) &&
				messages_control.data[0] == SET_CONFIG &&
				measurements.SNR_uplink > -90;
			if(is_coordinated_setconfig)
			{
				// POLLUTION GUARD (default-off): encode the honest forward-DATA-frame SNR instead of
				// measurements.SNR_uplink, which the immediately-preceding SET_CONFIG control-frame
				// decode overwrote with a ~34 dB read (ceiling-tone clamp => spurious 25 at the CMD).
				// rsp_suffix_snr_value() returns SNR_uplink unchanged when the guard is off.
				double suffix_snr = rsp_suffix_snr_value();
				if(rsp_suffix_data_snr_active() && suffix_snr != measurements.SNR_uplink)
					printf("[ACK-CTRL] Sending ACK+SNR pattern (SNR=%.1f dB; guard: forward-DATA %.1f replaces control-frame %.1f)\n",
						suffix_snr, suffix_snr, measurements.SNR_uplink);
				else
					printf("[ACK-CTRL] Sending ACK+SNR pattern (SNR=%.1f dB)\n", suffix_snr);
				fflush(stdout);
				send_ack_pattern_with_snr((float)suffix_snr);
			}
			else
			{
				if(g_verbose) { printf("[ACK-CTRL] Sending ACK pattern (no config switch)\n"); fflush(stdout); }
				// control_ack=true: this is the SET_CONFIG / control-code ACK
				// response — the BREAK-recovery Phase-1 turnaround (the SUSTAINING
				// crawl, cfg15_stall_root_cause.md). Opts into the robust
				// noncoherent-repeat ACK when MERCURY_RECOVERY_ACK_ROBUST is set
				// (recovery-ack-robustness.md §4); default-off → single block.
				send_ack_pattern(/*control_ack=*/true);
			}
			// If config changed (e.g., SET_CONFIG), load the new data config now.
			// ACK was sent on old config (correct — commander is still on old config),
			// but we need to switch to new config before receiving data.
			if(data_configuration != current_configuration)
			{
				if(g_verbose) { printf("[ACK-CTRL] Loading new data config %d (was %d)\n", data_configuration, current_configuration); fflush(stdout); }
				load_configuration(data_configuration, PHYSICAL_LAYER_ONLY, YES);
				// HYBRID SET_CONFIG CROSS: run the shared OFDM-entry ring setup (ring-shrink)
				// so a robust->OFDM cross is not stranded with an oversized acquisition ring.
				if(inband_rate_feature_enabled() && is_ofdm_config(data_configuration))
					inband_finalize_ofdm_adopt_ring(data_configuration);
				if(rate_opt.controls_link() && messages_control.data[0] == SET_CONFIG)
					inband_confirm_coordinated_config(current_configuration);
			}
		}
		else
		{
			// Fallback: LDPC ACK needs ack_configuration for correct modulation
			if(g_verbose) { printf("[ACK-CTRL] Sending LDPC ACK, loading config %d...\n", ack_configuration); fflush(stdout); }
			load_configuration(ack_configuration, PHYSICAL_LAYER_ONLY,NO);
			messages_batch_tx[message_batch_counter_tx]=messages_control;
			message_batch_counter_tx++;
			telecom_system->set_mfsk_ctrl_mode(true);
			pad_messages_batch_tx(ack_batch_size);
			send_batch();
			load_configuration(data_configuration, PHYSICAL_LAYER_ONLY,YES);
			// HYBRID SET_CONFIG CROSS: run the shared OFDM-entry ring setup (ring-shrink)
			// so a robust->OFDM cross is not stranded with an oversized acquisition ring.
			if(inband_rate_feature_enabled() && is_ofdm_config(data_configuration))
				inband_finalize_ofdm_adopt_ring(data_configuration);
		}
		// Capture frame + turnaround gap: CMD processing overhead only.
		// See acknowledging_data for detailed comment.
		// Must match buffer allocation (data_container.cc).
		telecom_system->set_mfsk_ctrl_mode(false);
		{
			// During load_configuration(), the capture thread keeps shifting
			// the buffer (frames_to_read=0, data_ready=1 → nUnder accumulates).
			// These shifts count toward the turnaround — the commander's ACK
			// detection + encode + TX happens concurrently with our config load.
			// Subtract the elapsed symbols so the total countdown (load_time +
			// ftr) matches the intended turnaround, keeping the preamble near
			// the right edge of the buffer instead of buried in silence.
			int nUnder_during_load = telecom_system->data_container.nUnder_processing_events.load();
			telecom_system->data_container.nUnder_processing_events = 0;

			int frame_symb = telecom_system->data_container.preamble_nSymb
				+ telecom_system->data_container.Nsymb;
			// Ring buffer: preamble stays at fixed ring position (no shift_left
			// drift), so we don't need buffer_Nsymb-sized waits. Just enough
			// for the peer's turnaround + one frame arrival.
			// Old shift_left values: SWITCH_ROLE=223 (4.7s!), other=171 (3.6s!).
			int ftr_val;
			{
				char cmd = messages_control.data[0];
				if(cmd == SWITCH_ROLE || cmd == SWITCH_BANDWIDTH)
					ftr_val = frame_symb + 20;  // ~1.3s: role switch turnaround
				else
					ftr_val = frame_symb + 10;  // ~1.3s: normal turnaround
			}
			// PARTIAL-BLOCK FIX (bigblock-whiten-align): when the next thing we receive is
			// ONE big-block (CFG16 framing), the decode snapshot fires when frames_to_read
			// hits 0 — so the wait MUST span the WHOLE block (preamble + Ngrid data symbols,
			// ~64 sym), not one stock frame (~13). The stock arming snapshotted after only
			// the block's head was captured, so cw1..K-1 read silence and CRC-failed (cw0
			// clean, 0 app bytes delivered). Extend the wait to cover a full block.
			// REPRODUCER HOOK (bigblock-whiten-align): MERCURY_BIGBLOCK_DEFEAT_FIX=1 keeps the
			// stock one-stock-frame wait so the full-path regression shows its fail-before
			// (decode fires on a partial block -> cw1..K-1 garbage). Production never sets it.
			bool defeat_block_ftr = false;
			{ const char* e = std::getenv("MERCURY_BIGBLOCK_DEFEAT_FIX"); if(e && *e && atoi(e)!=0) defeat_block_ftr = true; }
			if(!defeat_block_ftr
				&& telecom_system->bigblock_framing_enabled
				&& telecom_system->M != MOD_MFSK
				&& current_configuration == CONFIG_16)
			{
				int block_nsymb = telecom_system->bigblock_rx_block_nsymb();
				if(block_nsymb > 0)
				{
					int block_ftr = block_nsymb + 10;   // block span + turnaround margin
					if(block_ftr > ftr_val) ftr_val = block_ftr;
				}
			}
			telecom_system->data_container.frames_to_read = ftr_val;
			telecom_system->data_container.nUnder_processing_events = 0;

			// === DIAG: gearshift ftr trace (verbose only) ===
			if(g_verbose) { int buf_Nsymb = telecom_system->data_container.buffer_Nsymb.load(); printf("[FTR-GEAR] CONFIG_%d ftr=%d (buffer_Nsymb=%d frame_symb=%d ofdm=%d)\n", current_configuration, ftr_val, buf_Nsymb, frame_symb, is_ofdm_config(current_configuration)); fflush(stdout); }
		}

		char ack_command = messages_control.data[0];  // Save before potential NB switch
		messages_control.status=FREE;
		batch_rx_frame_count = 0;
		connection_status=RECEIVING;
		connection_id=assigned_connection_id;

		// NB/WB auto-negotiation: deferred switch after START_CONNECTION ACK
		// Must happen after ACK is sent in WB (so commander can hear it)
		// but before receiving next message (TEST_CONNECTION in NB)
		if(link_status == CONNECTION_RECEIVED && session_narrowband && narrowband_enabled == NO)
		{
			printf("[NB-NEG] Responder: switching to narrowband after START_CONNECTION ACK\n");
			fflush(stdout);
			commander_configured_nb = NO;  // Save original WB for restore on disconnect
			switch_narrowband_mode(YES);
		}

		// BW negotiation: deferred WB switch after SWITCH_BANDWIDTH ACK
		if(wb_upgrade_pending)
		{
			// Save pre-upgrade NB mode for restore on disconnect.
			// When responder started in NB and no NB negotiation switch occurred,
			// commander_configured_nb is still -1 — save it now so
			// reset_session_state() can restore NB after the session ends.
			if(commander_configured_nb < 0)
				commander_configured_nb = narrowband_enabled;
			printf("[BW-NEG] Responder: switching to WB after SWITCH_BANDWIDTH ACK\n");
			fflush(stdout);
			wb_upgrade_pending = false;
			switch_narrowband_mode(NO);
			// CONNECT-SEED FUSION: load the seed config carried by the SWITCH_BANDWIDTH frame
			// now (after the WB switch), collapsing the separate SET_CONFIG cross. Same
			// load_configuration() + ring setup the SET_CONFIG cross runs; the ftr reset below
			// then reflects the loaded config geometry.
			if(connect_fuse_seed_rx != CONFIG_NONE)
			{
				data_configuration = connect_fuse_seed_rx;
				connect_fuse_seed_rx = CONFIG_NONE;
				if(data_configuration != current_configuration &&
				   (config_negotiable_to(data_configuration, is_robust3_config(init_configuration) || is_robust3_config(connect_fast_fallback_config))))
				{
					load_configuration(data_configuration, PHYSICAL_LAYER_ONLY, YES);
					if(inband_rate_feature_enabled() && is_ofdm_config(data_configuration))
						inband_finalize_ofdm_adopt_ring(data_configuration);
					printf("[BW-NEG] FUSE: loaded connect-seed config %d on WB switch (SET_CONFIG cross collapsed)\n", data_configuration);
					fflush(stdout);
				}
			}
			// After WB config loads, the NB ftr=264 is still active but the
			// WB buffer is only 223 symbols. The leftover NB ftr causes the
			// Ring buffer: preamble stays put, no scroll-out. Small margin.
			// Old shift_left value: rx_frame + 40 (= 92, ~1.9s).
			{
				int rx_frame = telecom_system->data_container.preamble_nSymb
				             + telecom_system->data_container.Nsymb;
				telecom_system->data_container.frames_to_read = rx_frame + 10;
				telecom_system->data_container.nUnder_processing_events = 0;
				printf("[BW-NEG] WB ftr reset to %d\n",
					telecom_system->data_container.frames_to_read.load());
				fflush(stdout);
			}
		}

		if (ack_command==SWITCH_ROLE)
		{
			set_role(COMMANDER);
			this->link_status=CONNECTED;
			// Clear messages_rx[] to prevent stale frames from previous phases
			// from being ACKed alongside legitimate data in the next batch.
			for(int i=0;i<nMessages;i++) messages_rx[i].status=FREE;
			messages_rx_buffer.status=FREE;
			cl_timer ptt_off_wait;
			ptt_off_wait.reset();
			ptt_off_wait.start();
			// 3rd-spin inversion (single-process-sim-refactor.md §5.6(a) /
			// §5.7-B6): this was an EMPTY-body busy-spin
			//   while(ptt_off_wait.get_elapsed_time_ms()<ptt_off_delay_ms);
			// which hard-deadlocks under the single-thread virtual clock (the
			// sim clock advances ONLY via rx_transfer; nothing drives it during
			// an empty spin, so get_elapsed_time_ms() never increases). Route
			// it through ptt_busy_wait so the SIM_INPROC step-pump drives the
			// clock-advance from inside the wait. The EXIT PREDICATE is
			// byte-identical (elapsed >= ptt_off_delay_ms), and on every
			// production / two-process-paced-sim path the helper's behavior is
			// unchanged (the pump hook is null there) — so this only un-deadlocks
			// the single-thread stepper, it does not alter the live link.
			ptt_busy_wait(ptt_off_wait, ptt_off_delay_ms);

			// KX-as-data reverse ct: the config already settled during the forward stream,
			// so stream the staged reverse ct at the CURRENT config immediately and SKIP the
			// turboshift/gearshift probe below (it consumes the reverse transfer window and
			// the switch_role_test_timer would force the role back to RESPONDER before the ct
			// is sent, so the reverse ct never reaches the CMD). The two peers are symmetric
			// in the KX epoch (no asymmetric config negotiation). Env off / post-activation:
			// the full turbo path runs unchanged. data-flow-hybrid-kex.md §7.
			if(encryption_enabled && !cipher_suite.is_active() && kx_as_data_path())
			{
				this->connection_status = TRANSMITTING_DATA;
			}
			else
			{
			bool has_asymmetric = (forward_configuration != CONFIG_NONE &&
				reverse_configuration != CONFIG_NONE);

			// Save pre-swap config: this is the mutual config both sides
			// agreed on (e.g. settle config after BREAK recovery). The swap
			// below may load a stale reverse_configuration, corrupting
			// current_configuration before turboshift_last_good is set.
			int pre_switch_config = current_configuration;

			if(has_asymmetric)
			{
				// Asymmetric gearshift: swap forward/reverse for the return path
				int tmp = forward_configuration;
				forward_configuration = reverse_configuration;
				reverse_configuration = tmp;

				// During turboshift, skip the config load — both sides are at the
				// same mutual config and we'll probe from there. Loading the swapped
				// forward_configuration would corrupt current_configuration.
				if(turboshift_phase == TURBO_DONE || turboshift_phase == TURBO_REVERSE)
				{
					if(forward_configuration != current_configuration)
					{
						data_configuration = forward_configuration;
						load_configuration(data_configuration, PHYSICAL_LAYER_ONLY, YES);
					}

					printf("[GEARSHIFT] SWITCH_ROLE: transmitting at config %d\n",
						forward_configuration);
					fflush(stdout);
				}
				else
				{
					printf("[GEARSHIFT] SWITCH_ROLE during turboshift: staying at config %d\n",
						current_configuration);
					fflush(stdout);
				}
			}

			// Turboshift: decide whether to probe the reverse direction.
			// skip_turbo_reverse: skip entirely (symmetric assumption).
			// Otherwise: start at forward-settled config and verify (fast on
			// symmetric channels, falls back on asymmetric).
			if(has_asymmetric && turboshift_phase == TURBO_FORWARD && gear_shift_on == YES)
			{
				if(skip_turbo_reverse)
				{
					// Skip reverse probe — assume reverse path matches forward.
					turboshift_phase = TURBO_DONE;
					turboshift_active = false;
					turbo_snr_ack_enabled = false;
					printf("[TURBO] REVERSE: skipped (skip_turbo_reverse)\n");
					fflush(stdout);
					this->connection_status = TRANSMITTING_DATA;
				}
				else
				{
					// Start reverse probe at forward-settled config (not ROBUST_0).
					// If symmetric, first probe succeeds immediately. If asymmetric,
					// the normal ladder fall-back handles it.
					turboshift_phase = TURBO_REVERSE;
					turboshift_active = true;
					turboshift_last_good = pre_switch_config;
					turbo_snr_ack_enabled = true;
					turbo_received_snr = -99.0f;

					negotiated_configuration = config_ladder_up_n(
						current_configuration, 1, robust_enabled,
						narrowband_enabled == YES);
					printf("[TURBO] Phase: REVERSE — probing from config %d -> %d\n",
						current_configuration, negotiated_configuration);
					fflush(stdout);
					add_message_control(SET_CONFIG);
					this->connection_status = TRANSMITTING_CONTROL;
				}
			}
			else if(has_asymmetric &&
				(turboshift_phase == TURBO_REVERSE || turboshift_phase == TURBO_DONE))
			{
				// Returning to original roles after reverse probe
				turboshift_phase = TURBO_DONE;
				turboshift_active = false;
				turbo_snr_ack_enabled = false;
				turbo_received_snr = -99.0f;
				printf("[TURBO] DONE — starting data exchange\n");
				fflush(stdout);
				this->connection_status = TRANSMITTING_DATA;
			}
			else if(!has_asymmetric)
			{
				// No asymmetric negotiation (old firmware): fall back to TEST_CONNECTION
				add_message_control(TEST_CONNECTION);
				this->connection_status = TRANSMITTING_CONTROL;
			}
			else
			{
				this->connection_status = TRANSMITTING_DATA;
			}

			// Don't start the switch_role_test_timer during turboshift —
			// turboshift probes with SET_CONFIG which doesn't reset this timer,
			// causing it to force role=RESPONDER mid-probe or after TURBO_DONE.
			if(!turboshift_active && turboshift_phase != TURBO_DONE)
			{
				switch_role_test_timer.reset();
				switch_role_test_timer.start();
			}
			}
			last_message_received_type=NONE;
			last_message_sent_type=NONE;
			last_received_message_sequence=-1;
		}
		else if(ack_command==CLOSE_CONNECTION)
		{
#ifdef TERMINAL_EOT_RETRY_FAILBEFORE
			// Exact pre-retry teardown: move immediately to the initial PHY, so
			// a lost ACK cannot be re-solicited at the settled data config.
			reset_session_state();
			load_configuration(init_configuration,FULL,YES);
			this->link_status=LISTENING;
			batch_rx_frame_count = 0;
			batch_data_delivered = false;
			this->connection_status=RECEIVING;
			reset_all_timers();
			// Reset RX state machine - wait for fresh data (prevents decode of self-received TX audio)
			telecom_system->data_container.frames_to_read =
				telecom_system->data_container.preamble_nSymb + telecom_system->data_container.Nsymb;
			telecom_system->data_container.nUnder_processing_events = 0;

			fifo_buffer_tx.flush();
			fifo_buffer_backup.flush();
			fifo_buffer_rx.flush();
			messages_control.status=FREE;
#else
			rsp_enter_terminal_reack_hold();
#endif
		}
	}
}


// R2b — the SINGLE in-order current-batch delivery commit (arq.h). Extracts the
// BATCH-DONE else-branch state transitions (mark still-RECEIVED slots ACKED over
// the batch span, advance the contiguity high-water to the current bsi, roll
// prev<-cur / cur<-cur+1, reset the wired frame count) AND the app-FIFO push
// (copy_data_to_buffer over messages_rx[]) into ONE routine, shared by the
// BATCH-DONE gate below and the PREV-completion deliver-held-cur site
// (process_messages_rx_data_control). ONE delivery implementation under the D3.1
// gates. The CALLER is responsible for having cleared the contiguity ruler first
// (delivery_step_is_gap(cur, last) == false); this routine only commits — it
// NEVER re-checks the ruler, so it must never be called while a hole exists.
void cl_arq_controller::rsp_commit_cur_batch_delivery()
{
	int delivered_bsi = rsp_current_expected_batch_seq_id;
	// Option B' (data-flow-batch-size.md §9): the effective DELIVERY window for THIS
	// batch. Computed BEFORE the rx_batch_total_frames reset below (that reset would
	// collapse it). == data_batch_size in every non-desync / non-v2 case and under
	// MERCURY_BPRIME_DEFEAT; only a genuine CMD>RSP over-count widens it (<= 32). At the
	// held-cur delivery site rx_batch_total_frames is already -1, so eff_window falls
	// back to data_batch_size (the widened-AND-held corner degrades to Option W's loud
	// byte-stream-shift abort, never a silent orphan).
	int eff_window = rx_effective_window(rx_batch_total_frames);
	// GROWTH-TRANSITION integrity (scalable-SACK batch-size UP-transition): the DELIVERY
	// window must be the batch's GENUINE frame span, never floored UP to a data_batch_size
	// that GREW (Axis-2 up / climb) since this batch's frames were stored. rx_effective_
	// window() floors to data_batch_size for the ACCEPTANCE window; reused as the DELIVERY
	// bound it would mark+emit stale tail slots [genuine_span, data_batch_size) a prior
	// LARGER batch left ACKED -> a count-preserving boundary substitution the positional
	// byte-offset backstops (offset, not content) cannot see. Clamp DOWN to the sender-
	// declared count (rx_batch_total_frames / D5) when known, else the held span; NEVER
	// widen -> byte-identical unless the genuine span < the grown data_batch_size.
	{
		bool growth_deliver_defeat = false;
		{ const char* e = std::getenv("MERCURY_GROWTH_DELIVER_DEFEAT");
		  if(e && *e && atoi(e)!=0) growth_deliver_defeat = true; }
		if(!growth_deliver_defeat)
		{
			int genuine_span = rx_batch_total_frames;
			if(genuine_span <= 0 && rsp_gap_hold_cur_expected > 0)
				genuine_span = rsp_gap_hold_cur_expected;
			if(genuine_span > 0 && genuine_span < eff_window)
				eff_window = genuine_span;
		}
	}
	// Mark any still-RECEIVED slots ACKED so copy_data_to_buffer's ACKED-only
	// iteration delivers them (idempotent: at the BATCH-DONE site the slots were
	// already flipped ACKED before the gap gate, so this is a no-op there; at the
	// PREV-completion held-cur site the held slots are RECEIVED and this marks them).
	for(int i=0;i<eff_window && i<this->nMessages;i++)
		if(messages_rx[i].status==RECEIVED) messages_rx[i].status=ACKED;
	// FIX-8 (data-integrity): advance the reset-surviving high-water to the
	// delivered batch (monotonic-with-wrap) so a later post-reset re-adopt can
	// detect a hole.
	advance_last_delivered(delivered_bsi);
	rsp_gap_recover_rounds = 0;   // R2a: a forward delivery clears recoverable-hold state
	rsp_prev_batch_seq_id = delivered_bsi;
	rsp_current_expected_batch_seq_id = (delivered_bsi + 1) & 0xFF;
	// D5: the next batch must latch its OWN authoritative frame count.
	rx_batch_total_frames = -1;
	// AEAD nonce source (data-flow-aead-nonce.md §5): bind to the DELIVERED wire bsi.
	decrypt_delivered_bsi = delivered_bsi;
	this->rx_copy_window = eff_window;   // Option B' (§9): deliver the full (possibly widened) batch
	copy_data_to_buffer();
	this->rx_copy_window = -1;
}

// In-band unilateral DEMOTE-REBASE (data-flow-stream-offset.md - demote-rebase
// double-delivery / D31_INORDER_DESIGN.md ?3). Extracted from the SET_CONFIG
// responder so the directed regression (test_dedup_rebase) drives the EXACT
// production rebase, not a state poke. A REAL mid-transfer config change
// re-baselines the RSP bsi window (cur=prev=-1 + drop the in-flight prev partial
// storage) so the next data frame re-adopts through the gap-gate, DELIBERATELY
// preserving rsp_last_delivered_batch_seq_id AND rx_stream_emitted_bsi_hw (the
// reset-surviving high-waters the re-adopt gate and the funnel de-dup read).
// Guard-internal: a no-op when sack_v2 is off or the window is already
// re-baselined, so no-op/same-config SET_CONFIGs never perturb the window.
void cl_arq_controller::rsp_inband_demote_rebase()
{
	if(!(sack_v2_enabled && rsp_current_expected_batch_seq_id >= 0))
		return;
	printf("[RSP-V2-DEMOTE-REBASE] config change %d->%d mid-transfer: "
		"re-baselining bsi window (cur=%d prev=%d -> -1) so the next frame "
		"re-adopts through the gap-gate (last_delivered=%d preserved)\n",
		current_configuration, forward_configuration,
		rsp_current_expected_batch_seq_id, rsp_prev_batch_seq_id,
		rsp_last_delivered_batch_seq_id);
	fflush(stdout);
	rsp_current_expected_batch_seq_id = -1;
	rsp_prev_batch_seq_id             = -1;
	rsp_prev_batch_active             = false;
	rsp_prev_batch_received_count     = 0;
	rsp_prev_batch_expected_count     = 0;
	bigblock_partial_armed            = false;
	for(int i=0; i<this->nMessages; i++)
		messages_rx_prev[i].status = FREE;
	// NOTE: rsp_last_delivered_batch_seq_id AND rx_stream_emitted_bsi_hw are
	// DELIBERATELY preserved (the reset-surviving high-waters the re-adopt gate
	// and the funnel de-dup read).
}

void cl_arq_controller::process_messages_acknowledging_data()
{
	// PREKEY safety invariant: this handler may return to RECEIVING only after the
	// scheduled key has either been consumed by the clean ACK for its exact BSI or
	// cancelled (which physically unkeys and restores RX).  Keep this whole-handler
	// backstop in addition to the explicit withhold-site cancels below so a future
	// early return cannot recreate a stuck-keyed/stuck-deaf link.
	struct t1_keyed_receiving_exit_guard {
		cl_arq_controller* arq;
		explicit t1_keyed_receiving_exit_guard(cl_arq_controller* p) : arq(p) {}
		~t1_keyed_receiving_exit_guard()
		{
			if(arq->connection_status == RECEIVING && arq->t1_schedkey.keyed)
				arq->t1_cancel_schedkey("ack-handler-receiving-exit");
		}
	} t1_exit_guard(this);

	printf("[RSP-RX-TIMEOUT] Entering ACK-GATE: rx_count=%d timeout=%d batch=%d\n",
		batch_rx_frame_count, receiving_timeout, data_batch_size);
	fflush(stdout);

	// M3 (SACK turnaround trace): the collision STARTS the moment RSP decides
	// to ACK-GATE. Anchors the decision instant on the common clock — the
	// existing line above is a plain printf the parser cannot ingest.
	mtl::log_event_kv("rsp_ack_gate_entry", "rx_count=%d batch=%d",
		batch_rx_frame_count, data_batch_size);

	int nAck_messages=0;
	// Option B' (data-flow-batch-size.md §9): the RX ACCEPTANCE WINDOW for THIS batch,
	// captured at FUNCTION SCOPE and BEFORE the BATCH-DONE commit (which resets
	// rx_batch_total_frames to -1). The clean-ACK all-ones bitmap below must be
	// eff_window-wide so a widened batch (CMD>RSP over-count) is credited CLEAN by the
	// CMD (which built the larger batch); recomputing after the commit would read -1 ->
	// data_batch_size and emit a too-narrow bitmap. eff_window == data_batch_size in
	// every non-desync / non-v2 case and under MERCURY_BPRIME_DEFEAT (byte-identical).
	int eff_window = rx_effective_window(rx_batch_total_frames);
	// D3.1: set true if the BATCH-DONE delivery-time gap gate fired — guards the
	// copy_data_to_buffer() below so the gapped batch is NOT pushed to the app.
	bool batch_gap_aborted = false;
	receiving_timer.stop();
	receiving_timer.reset();

	// Bug #36: reset nUnder BEFORE ACK TX so only turnaround-period
	// nUnder is counted (not accumulated frame-processing nUnder).
	telecom_system->data_container.nUnder_processing_events = 0;

	if(ack_pattern_time_ms > 0 || l1_blockack_data_active())
	{
		// Send ACK tone pattern (universal, all modes)
		if(repeating_last_ack==NO)
		{
			// Gate: require all expected frames before sending ACK.
			// Pattern ACK carries no per-frame info — commander marks ALL
			// pending frames as ACKED. If we only decoded a partial batch,
			// suppress ACK so commander times out and retransmits.
			//
			// Use actual RECEIVED slot count (immune to OFDM re-decode).
			// With compression + no zero-padding, expected frames < data_batch_size.
			// Derive expected count from compression header in frame 0.
			//
			// Option B' (data-flow-batch-size.md §9): the RX ACCEPTANCE WINDOW for
			// THIS batch trusts the CRC-protected sender-declared D5 count when it
			// exceeds the stale command-synced data_batch_size (the res_c3100 CMD>RSP
			// case). eff_window == data_batch_size in EVERY non-desync / non-v2 case
			// and under MERCURY_BPRIME_DEFEAT, so the whole ACK-GATE below is
			// byte-identical there; only a genuine over-count widens it (<= 32).
			// (eff_window is declared at function scope above so the clean-ACK bitmap,
			// which runs AFTER the commit resets rx_batch_total_frames, still sees it.)
			int rx_received = 0;
			for(int i = 0; i < eff_window; i++)
				if(messages_rx[i].status == RECEIVED) rx_received++;

			int expected = eff_window;  // Default for non-compressed (B' §9: was data_batch_size)
			// D5 (TRACK_C_D2D3D5_DESIGN.md §5.3): the SACK partial gate compares
			// rx_received < expected; the bitmap then advertises which slots in
			// [0, data_batch_size) are missing. With a lost EOB the inference
			// (last_received_end_of_batch_seq+1) collapses `expected` to exclude
			// the lost tail → rx_received >= expected → NO SACK is sent for it →
			// the lost frame is NEVER retransmitted (the SACK dead-end, §4.3). The
			// wired authoritative count keeps the lost tail INSIDE `expected`, so the
			// gate fires, the bitmap marks the tail not-received, and CMD retransmits
			// it → faithful recovery. Fallback to the EOB inference / compression-
			// header derivation when the count is unknown / under DEFEAT.
			bool d5_infer_defeat_sack = false;
			{ const char* e = std::getenv("MERCURY_D5_INFER_DEFEAT");
			  if(e && *e && atoi(e)!=0) d5_infer_defeat_sack = true; }
			// End-of-batch flag: commander marks last frame with bit 7 in
			// sequence_number, giving us the actual batch size sent.
			// Works for all modes (compressed, uncompressed, encrypted).
			if(rx_batch_total_frames > 0 && !d5_infer_defeat_sack)
			{
				// Option B' (§9): honor the sender-declared count up to eff_window
				// (== rx_batch_total_frames when it widened) instead of collapsing it
				// to the stale data_batch_size — the exact clamp that truncated
				// res_c3100. eff_window >= data_batch_size, so this never LOWERS
				// expected relative to pre-B'.
				expected = rx_batch_total_frames;
				if(expected > eff_window) expected = eff_window;
			}
			else if(cipher_suite.is_active())
			{
				// ENC BATCH-SIZE FIX (data-flow-encrypted-batch-size.md §4/§5): the
				// EOB inference is UNSAFE for an encrypted batch — a short `expected`
				// makes the gate ACK/deliver before the full ciphertext arrives, and
				// the whole-batch AEAD MAC over the truncated buffer auth-FAILS ->
				// false PSK-mismatch DISCONNECT. The TX seals each encrypted batch to
				// EXACTLY crypto_frames = min(data_batch_size, crypto_batch_size)
				// frames (the batch_capacity cap), a deterministic span both peers
				// compute from shared constants. Hold `expected` at that span so the
				// gate keeps SACKing the missing tail until the full (decryptable) set
				// lands — NEVER deliver short. (Symmetric with the prev-expected
				// derivation in arq_common.cc bump_bsi_and_transfer_prev.)
				expected = data_batch_size;
				if(crypto_batch_size > 0 && crypto_batch_size < expected)
					expected = crypto_batch_size;
			}
			else if(last_received_end_of_batch_seq >= 0)
			{
				expected = last_received_end_of_batch_seq + 1;
				if(expected > eff_window) expected = eff_window;   // B' §9 (== data_batch_size here: D5 absent)
			}
			else if(compression_enabled
				&& !cipher_suite.is_active()  // Can't peek header when encrypted
				&& !compressor.is_streaming()  // §7.13.35: streaming packs multiple messages per batch — frame-0 header describes only the first; falling through to here yields under-estimate, RSP ACKs early (gearshift_v19 finding)
				&& messages_rx[0].status == RECEIVED
				&& messages_rx[0].length >= compressor.get_header_size())
			{
				// Fallback: derive from compression header (old commanders
				// without end-of-batch flag).
				// Only safe for non-streaming compression where one batch =
				// one compressed message. Streaming case waits for EOB or
				// times out at data_batch_size (treated as full batch).
				const unsigned char* hdr = (const unsigned char*)messages_rx[0].data;
				int hdr_comp = hdr[1] | (hdr[2] << 8);
				int gate_hdr_size = compressor.get_header_size();
				int total_compressed = gate_hdr_size + hdr_comp;
				// SACK Design A Step 1 — effective DATA_LONG header.
				int mf = max_data_length + max_header_length - effective_data_long_header_length(sack_v2_enabled, header_carries_d5);
				expected = (total_compressed + mf - 1) / mf;
				if(expected > eff_window) expected = eff_window;   // B' §9 (== data_batch_size here: D5 absent)
				if(expected < 1) expected = 1;
			}

			// Diagnostic: show which sequence numbers were received. Report over
			// eff_window (B' §9) so a widened batch's tail slots (>= data_batch_size)
			// are visible in the [ACK-GATE-DIAG] rx=N/N line — the LIVE fire-proof
			// oracle. eff_window == data_batch_size in the common case.
			{
				printf("[ACK-GATE-DIAG] rx=%d/%d exp=%d seqs:", rx_received, eff_window, expected);
				for(int i = 0; i < eff_window; i++)
					if(messages_rx[i].status == RECEIVED) printf(" %d", i);
				printf("\n"); fflush(stdout);
			}

			// (B) LOUD BACKSTOP — CMD>RSP batch-size desync integrity gate
			// (silent-corruption-residual.md §7-B / data-flow-batch-size.md §8). The
			// SENDER-declared per-batch frame count (rx_batch_total_frames — D5, carried on
			// EVERY DATA frame = the CMD's message_batch_counter_tx) EXCEEDS our own
			// data_batch_size => the CMD built a LARGER batch than the RSP applied (CMD Axis-2
			// stepped up but the RSP never durably APPLIED the SET_LINK_PARAMS — the §1
			// CMD==RSP invariant is BROKEN, CMD>RSP). The `expected` clamp above just collapsed
			// that count to our smaller size, so the PASS path below would DELIVER the batch
			// TRUNCATED and treat it complete, ORPHANING the sender's surplus frames -> a
			// permanent silent stream shift (res_c3100, WGN:25 ~3%). Integrity D0: the RSP must
			// NEVER silently deliver a batch truncated below the sender-declared size. Raise
			// LOUD + abort via the SAME tested teardown the D3.1 delivery-gap gate uses so the
			// link resyncs cleanly. INERT unless CMD>RSP (matched / step-down batches keep
			// rx_batch_total_frames <= data_batch_size; robust batch=1 carries no D5 byte).
			// MERCURY_BATCHSIZE_DESYNC_DEFEAT=1 disables it (fail-before arm, mirrors
			// MERCURY_GAP_ABORT_DEFEAT / MERCURY_D5_INFER_DEFEAT — restores the silent truncation).
			bool batchsize_desync_defeat = false;
			{ const char* e = std::getenv("MERCURY_BATCHSIZE_DESYNC_DEFEAT");
			  if(e && *e && atoi(e)!=0) batchsize_desync_defeat = true; }
			if(!batchsize_desync_defeat && !passive_monitor
			   // Option B' (data-flow-batch-size.md §9): B' WIDENS the window (eff_window) to deliver a
			   // CMD>RSP over-count IN FULL, so the pre-B' > data_batch_size condition is now benign.
			   // Narrow the trigger to the TRUE impossibility D5 > MAX_SACK_BATCH_SIZE (the
			   // Axis-2 ceiling, unreachable for a legitimate non-crypto OFDM batch). Under
			   // MERCURY_BPRIME_DEFEAT the
			   // window collapses AND this reverts to the pre-B' > data_batch_size abort (fail-before).
			   && batchsize_desync_detected(rx_batch_total_frames,
			          bprime_defeat_active() ? data_batch_size : MAX_SACK_BATCH_SIZE,
			          sack_v2_enabled))
			{
				printf("[RSP-V2-BATCHSIZE-DESYNC] sender batch_total_frames=%d > local "
					"data_batch_size=%d (bsi=%d rx=%d) — CMD built a LARGER batch than RSP "
					"applied; REFUSING to deliver a batch truncated below the sender size "
					"(would orphan %d frames -> silent stream shift). Aborting link for clean "
					"resync.\n",
					rx_batch_total_frames, data_batch_size,
					rsp_current_expected_batch_seq_id, rx_received,
					rx_batch_total_frames - data_batch_size);
				fflush(stdout);
				char reason[128];
				snprintf(reason, sizeof(reason),
					"BATCHSIZE-DESYNC sender_total=%d > local_batch=%d (CMD>RSP truncation)",
					rx_batch_total_frames, data_batch_size);
				rsp_gap_abort_teardown(reason);   // link DROPPED + session reset; NO delivery, NO ACK
				// Re-arm the receiver + return WITHOUT ACKing/delivering — identical telecom
				// discipline to the back-pressure HOLD / SACK-suppress early returns below. No
				// clean ACK is emitted, so the CMD never falsely credits the truncated batch;
				// the DROPPED link_status the teardown set drives the main loop to handle the
				// drop on the next cycle (the SAME RECEIVING+DROPPED post-abort state the D3.1
				// GAP-ABORT produces).
				batch_rx_frame_count = 0;
				last_received_end_of_batch_seq = -1;
				telecom_system->data_container.frames_to_read =
					telecom_system->data_container.preamble_nSymb
					+ telecom_system->get_active_nsymb();
				telecom_system->data_container.nUnder_processing_events = 0;
				calculate_receiving_timeout();
				receiving_timer.start();
				connection_status = RECEIVING;
				return;
			}

			// A clean batch may decode before the commander's sacrificial tail reaches its
			// scheduled boundary. Keep ACK audio gated and return to the outer loop so the
			// independently armed frame-0 scheduler can key there. No busy wait is introduced:
			// update_status() continues polling freshness/cancellation, and partial batches
			// bypass this gate to the reactive D2 path below.
			if(t1_schedkey.armed && rx_received == expected
			   && t1_schedkey.bsi == rsp_current_expected_batch_seq_id)
			{
				t1_authorize_clean_ack_if_ready();
				t1_schedkey_poll();
				if(t1_schedkey.clean_ack_authorized && t1_schedkey.armed)
					return;
			}

			if(data_batch_size > 1 && rx_received < expected && !passive_monitor)
			{
				t1_cancel_schedkey("partial-ack-gate");
				if(sack_enabled && rx_received > 0)
				{
					// SACK: send selective ACK with bitmap of received frames
					printf("[ACK-GATE] SACK: received %d/%d (expected %d)\n",
						rx_received, data_batch_size, expected);
					fflush(stdout);

					// Build bitmap: true = frame received
					bool sack_bitmap[MAX_SACK_BATCH_SIZE];
					for(int i = 0; i < eff_window && i < MAX_SACK_BATCH_SIZE; i++)   // Option B' (§9)
						sack_bitmap[i] = (messages_rx[i].status == RECEIVED);

					// Stage-3 uses the same production ACK gate, but a partial batch
					// closes the group immediately. Positive slots are reported; the
					// commander's journal retains every miss and both peers abort
					// loudly rather than sending later data across an undelivered hole.
					if(l1_blockack_data_active())
					{
						const uint8_t bsi = (uint8_t)(
							rsp_current_expected_batch_seq_id >= 0
								? rsp_current_expected_batch_seq_id : 0);
						std::vector<bool> received((std::size_t)eff_window, false);
						for(int i=0;i<eff_window && i<MAX_SACK_BATCH_SIZE;i++)
							received[(std::size_t)i] = sack_bitmap[i];
						std::vector<uint8_t> wire;
						if(l1_blockack.observe_received_batch(bsi, received,
							/*final=*/true, &wire))
							l1_send_block_control(wire);
						printf("[L1-BLOCKACK-RSP] partial bsi=%u flushed; "
							"aborting before any later-batch delivery\n", (unsigned)bsi);
						fflush(stdout);
						rsp_gap_abort_teardown("L1 block partial retained by sender journal");
						return;
					}

					// SACK Design A Step 7 — OFDM SACK_RSP control frame
					// (~390 ms wire occupancy). Step 15: legacy MFSK SACK
					// pattern branch has been deleted; only the v2 path
					// remains. When sack_v2_enabled is false, no partial-batch
					// SACK is sent — CMD falls through to ACK-timeout +
					// full-batch retransmit.
					if(sack_v2_enabled)
					{
						// SACK Design A Step 11 — Axis 3 SACK_MODE_OFF gate.
						// CMD has signaled (via SET_LINK_PARAMS) that the
						// reverse path is too unreliable for SACK_RSP to be
						// worth the airtime. Suppress SACK_RSP TX entirely;
						// CMD's ACK-timeout will drive a full-batch retransmit
						// on this partial. PROBE behaves as ON here (RSP TX's
						// SACK_RSP; CMD's decode outcome drives PROBE→ON or
						// PROBE→OFF on its side).
						if(axis3_sack_mode == SACK_MODE_OFF)
						{
							printf("[ACK-GATE-V2-OFF] sack_mode=OFF — suppressing "
								"SACK_RSP TX on partial batch (%d/%d received); "
								"CMD will rely on ACK-timeout + full-batch "
								"retransmit (Axis-3 Step 11 fallback).\n",
								rx_received, data_batch_size);
							fflush(stdout);
							// Keep partial state — CMD's retransmit will land
							// new copies of missing frames, the existing rx loop
							// will merge them. No SACK_RSP wire frame emitted.
						}
						else if(data_batch_size <= 1)
						{
							// SACK Design A §7.13.12 / Step 15: SACK_RSP is
							// meaningless on a single-frame batch (which is the
							// default in MFSK / ROBUST modes per
							// arq_common.cc:1163-1198). The EOB bit-7 on the lone
							// DATA frame is the all-or-nothing receipt indicator.
							// Suppress the dispatch entirely.
							printf("[ACK-GATE-V2] SACK_RSP suppressed (data_batch_size=%d, multi-frame batches only)\n",
								data_batch_size);
							fflush(stdout);
						}
						else
						{
							// batch_seq_id field anchors the bitmap to the
							// outstanding CMD batch. RSP echoes whatever it
							// adopted at Step 4 (rsp_current_expected_batch_seq_id).
							// If still -1 (first-frame edge case where SACK
							// fires before adopt), fall back to 0 — the CMD
							// will compare against its cmd_batch_seq_id-1 (the
							// in-flight batch's id).
							unsigned char bsi = (unsigned char)
								((rsp_current_expected_batch_seq_id >= 0)
									? (rsp_current_expected_batch_seq_id & 0xFF)
									: 0);
							printf("[ACK-GATE-V2] dispatching OFDM SACK_RSP (batch_seq_id=%u, %d/%d received, sack_mode=%s)\n",
								(unsigned)bsi, rx_received, data_batch_size,
								(axis3_sack_mode == SACK_MODE_PROBE) ? "PROBE" : "ON");
							fflush(stdout);

							// §6g (fact-doc sack_partial_bsi_advance.md, 2026-05-21):
							// Fire Step 8a bsi-bump-and-transfer-prev BEFORE the
							// transport-choice branch so the invariant runs
							// regardless of whether the MFSK suffix wins
							// (used_mfsk_path=true → send_sack_v2_frame skipped)
							// or the OFDM SACK_RSP wins. Capture the pre-bump
							// bsi for the wire frames; after the bump,
							// rsp_current_expected_batch_seq_id is +=1 and the
							// old value is in rsp_prev_batch_seq_id. Both wire
							// paths must transmit the PARTIAL batch's bsi
							// (= pre-bump value), not the new current_expected.
							unsigned char sacked_bsi = bsi;
#if !defined(NDEBUG) && defined(MERCURY_GEN_CANON_ASSERT)
							for(int i=0; i<eff_window && i<nMessages; i++)
								if(messages_rx[i].status == RECEIVED
								   && messages_rx[i].batch_seq_id >= 0
								   && (messages_rx[i].batch_seq_id & 0xFF) != sacked_bsi)
									std::abort();
#endif
							bump_bsi_and_transfer_prev();

							// MFSK-suffix ACK+SACK — WB-only (suffix_len()>0).
							// No optimizer-territory gate: the pattern correlator
							// gives this path ROBUST_0-grade detection at any
							// config, which is exactly where partial-batch
							// recovery matters most. Pack the per-frame received
							// bitmap LSB-first into a uint32_t (matches
							// send_sack_v2_frame's byte-LSB-first convention at
							// send_sack_v2_frame's byte-LSB-first convention).
							bool used_mfsk_path = false;
							if (eff_window <= MFSK_SACK_BITMAP_BITS
								&& MFSK_ACK_SACK_ENABLED
								&& telecom_system->ack_mfsk.ack_sack_suffix_len() > 0)
							{
								uint32_t bitmap_u32 = 0;
								// Phase B Wave 1 flag-day (fact-doc §11.2): MFSK
								// ctrl-suffix bitmap is 30 bits (was 32). Cap nbits
								// to 30 so we never set bits 30/31 — those would be
								// silently dropped by pack_ack_sack_payload and the
								// receiver would never see them.
								int nbits = eff_window;   // Option B' (§9)
								if (nbits > MFSK_SACK_BITMAP_BITS)
									nbits = MFSK_SACK_BITMAP_BITS;
								for (int i = 0; i < nbits; i++)
								{
									if (sack_bitmap[i])
										bitmap_u32 |= (1u << i);
								}
									// A2: sacked_bsi is the bitmap source/target. The encoder below derives
									// wire=target-1 in negotiated cumulative mode (mod 256), or wire=target
									// in per-batch mode. No producer supplies a pre-shaped/high-water octet.
								unsigned char wire_bsi = cumulative_ack_bsi_field(
									sacked_bsi, rsp_last_delivered_batch_seq_id, cumulative_ack_enabled);
								printf("[RSP-MFSK-SACK] partial path: batch_seq_id=%u (wire_bsi=%u n_r=%d cum=%d) bitmap=0x%08x nframes=%d\n",
									(unsigned)sacked_bsi, (unsigned)wire_bsi,
									rsp_last_delivered_batch_seq_id, cumulative_ack_enabled ? 1 : 0,
									(unsigned)bitmap_u32, data_batch_size);
								fflush(stdout);
								// WALL-B FIX-9 D2 REFINE (§2.1): partial = NAcking an incomplete batch -> the
								// CMD will retransmit -> THIS is the retransmit turnaround the drift slip rides
								// on -> arm the D2 robust reverse-ACK geometry (settle) for this ACK.
								ack_tx_retx_turnaround = true;
								long long mfsk_ms = send_mfsk_ack_sack(sacked_bsi, bitmap_u32);
								if (mfsk_ms > 0)
								{
									printf("[TX-ACK-SACK] partial via MFSK suffix wire_ms=%lld\n",
										mfsk_ms);
									fflush(stdout);
									used_mfsk_path = true;
								}
								else
								{
									printf("[RSP-MFSK-SACK] MFSK path returned 0 — falling back to OFDM SACK_RSP\n");
									fflush(stdout);
								}
							}
							if (!used_mfsk_path)
							{
									// Same target-owned encoder rule on the OFDM SACK_RSP transport.
								send_sack_v2_frame(sack_bitmap, eff_window, sacked_bsi);   // Option B' (§9)
							}
						}
					}
					// else: !sack_v2_enabled → no partial-batch SACK transport
					// remains. CMD's ACK-timeout drives the full-batch retransmit.

					// Keep partial messages_rx (DON'T free) - retransmit fills gaps
					stats.nNAcked_data++;
					batch_rx_frame_count = 0;
					last_received_end_of_batch_seq = -1;

					// Same post-TX cleanup as send_ack_pattern
					telecom_system->set_mfsk_ctrl_mode(false);
					telecom_system->data_container.nUnder_processing_events = 0;
					calculate_receiving_timeout();
					// LINK-PHASE STEP 4 (MC-6): derive the post-SACK retx-turn listen window from
					// the popcount of the SACK we JUST authored, replacing the hardcoded 1.5x
					// cadence. popcount_missing = clear bits of our bitmap over the ACK-gate window
					// (eff_window - rx_received). MERCURY_LINKPHASE_RETXSLOT default OFF => the
					// bounded-idle fallback below (stock 1.5x), byte-identical to base.
					if(linkphase_retxslot_on() && (eff_window - rx_received) > 0)
					{
						int popcount_missing = eff_window - rx_received;
						receiving_timeout = linkphase_derive_retx_slot(popcount_missing, receiving_timeout);
					}
					else
					{
						// Bounded-idle FALLBACK (flag OFF, or no authored bitmap): stock 1.5x —
						// cover CMD extended listen + retransmit TX (~17.6s cycle). 1.5x is enough:
						// CMD extended listen (~7.8s) + retransmit (~9.85s); RSP responds before
						// CMD's next retransmit TX starts (2x exceeded the cycle -> half-duplex
						// collision). Byte-identical to base under the default-OFF flag.
						receiving_timeout = receiving_timeout * 3 / 2;
						printf("[RSP-POST-SACK] timeout=%dms (1.5x normal for retransmit)\n",
							receiving_timeout);
						fflush(stdout);
					}
					receiving_timer.start();
					connection_status=RECEIVING;
					return;
				}

				printf("[ACK-GATE] Suppressing: received %d/%d (expected %d)\n",
					rx_received, data_batch_size, expected);
				fflush(stdout);
				// Keep partial messages_rx (DON'T free) - add_message_rx_data
				// overwrites unconditionally, so retransmit fills in missing
				// frames while already-received frames are preserved.
				stats.nNAcked_data++;
				batch_rx_frame_count = 0;
				last_received_end_of_batch_seq = -1;
				// Reset RX state for fresh retransmission capture.
				// §17.6 / §17.8 (§5 cross-layer audit): this is the ACK-GATE
				// partial-batch retx re-arm. It is DELIBERATELY left STOCK-frame
				// (NOT routed through bigblock_block_ftr_or). When a big-block decoded
				// PARTIAL (some cw demoted by the per-cw wire-CRC), the CMD sends the
				// selective-repeat as STOCK per-frame frames — bigblock_send_one_block()
				// declines while sack_retransmit_active (arq_common.cc:3718, CMD sets it
				// at arq_commander.cc:1803). So the RX legitimately expects per-frame
				// retx frames here; block-spanning this arming would OVER-WAIT for a
				// big-block that is not coming and stall the retx. The NEXT NEW-DATA
				// block IS re-armed to full block-span by the wrapped ACK-send paths
				// (send_mfsk_ack_sack arq_common.cc:5526 etc.), so the new-data path
				// still gets its full window. DO NOT "fix" this by wrapping it.
				telecom_system->data_container.frames_to_read =
					telecom_system->data_container.preamble_nSymb
					+ telecom_system->get_active_nsymb();
				telecom_system->data_container.nUnder_processing_events = 0;
				// DON'T reset search_raw here. If the timer fires mid-batch
				// while frames are still arriving, resetting to 0 causes
				// re-decode of already-received frames. Keep position so
				// the decode loop continues forward through the buffer.
				// Return to RECEIVING - commander will timeout and retransmit
				calculate_receiving_timeout();
				receiving_timer.start();
				connection_status=RECEIVING;
				return;
			}
			if(passive_monitor && rx_received < expected)
			{
				printf("[MONITOR] Partial batch: received %d/%d (expected %d) — accepting anyway\n",
					rx_received, data_batch_size, expected);
				fflush(stdout);
			}
			printf("[ACK-GATE] PASS: received %d/%d (expected %d)\n",
				rx_received, data_batch_size, expected);
			fflush(stdout);

			// Option W CORE — RSP PRIMARY byte-gate (data-flow-stream-offset.md §8.2 step 5).
			// The batch is FRAME-COUNT complete (rx_received >= expected), but `expected` is a
			// PROXY (data_batch_size / rx_batch_total_frames) that mechanisms 1 (EOB-undercount
			// tail-drop) and 4 (complete-at-shrunk-count) can make wrongly small — the batch
			// then PASSes with FEWER bytes than the sender committed, orphaning the surplus →
			// a permanent stream shift. Gate on DELIVERED BYTES vs the wire stamp.length: on a
			// shortfall WITHHOLD the clean ACK (keep messages_rx RECEIVED, do NOT mark ACKED,
			// do NOT bump the bsi, do NOT deliver) and re-arm — the SAME discipline as the
			// partial-batch SACK-suppress / backpressure holds above. This makes "declare
			// complete at a shrunk count" IMPOSSIBLE: the byte count, not the frame count, gates
			// completion.
			// RECOVERY (F2, silent-corruption-residual.md §16/§17.6 — corrected): this path
			// does NOT emit a SACK. On a byte shortfall the batch is FRAME-COUNT complete (every
			// slot in [0,data_batch_size) is present), so a SACK bitmap over that window would be
			// ALL-ONES and would FALSE-CREDIT the batch — the exact silent accept this gate
			// prevents; the short/surplus bytes live OUTSIDE the RSP's bitmap. Recovery is
			// therefore the CMD's ACK-timeout (the clean ACK looks lost, an existing recoverable
			// state) → full-batch retransmit → the sender re-stages+rebuilds (a fresh EOB stamp
			// re-arrives; a demote/config-change also invalidates the stale stamp, F4.2) → the
			// shortfall clears and the batch completes; if it persists the retx-storm escalates
			// to BREAK (LOUD, bounded). So the outcome is COMPLETE or LOUD — never a silent
			// credit, never a permanent hang (§8.2-arm test_stream_offset Part S drives it).
			// FALSE-FIRE guard: compare ONLY to the LATCHED stamp.length (never expected/
			// data_batch_size); absent stamp (robust / old peer / lost EOB frame) = safe no-op.
			// MERCURY_W_BYTEGATE_DEFEAT=1 restores the pre-fix count-only PASS (the STEP-2c
			// fail-before arm).
			if(!passive_monitor && sack_v2_enabled && rsp_current_expected_batch_seq_id >= 0)
			{
				int wbsi = rsp_current_expected_batch_seq_id & 0xFF;
				bool w_bytegate_defeat = false;
				{ const char* e = std::getenv("MERCURY_W_BYTEGATE_DEFEAT");
				  if(e && *e && atoi(e)!=0) w_bytegate_defeat = true; }
				if(!w_bytegate_defeat && w_bytegate_shortfall(wbsi))
				{
					{
						t1_cancel_schedkey("byte-shortfall-withhold");
						printf("[RSP-V2-BYTE-SHORTFALL] WITHHOLD clean ACK: bsi=%d committed=%u "
							"(frame-count PASS but byte shortfall — tail-drop / shrunk-count); "
							"re-arm so SACK re-requests. NOT crediting, NOT flushing.\n",
							wbsi, rx_stream_stamp[wbsi].length);
						fflush(stdout);
						// Identical re-arm to the partial-batch SACK-suppress hold above.
						stats.nNAcked_data++;
						batch_rx_frame_count = 0;
						last_received_end_of_batch_seq = -1;
						telecom_system->data_container.frames_to_read =
							telecom_system->data_container.preamble_nSymb
							+ telecom_system->get_active_nsymb();
						telecom_system->data_container.nUnder_processing_events = 0;
						calculate_receiving_timeout();
						receiving_timer.start();
						connection_status=RECEIVING;
						return;
					}
				}
			}

			// Fix H#3 (delivery-integrity-audit-monitor.md §3): the clean data-ACK is sent
			// below BEFORE copy_data_to_buffer() delivers, and fifo_push_rx() DROPS the tail
			// when fifo_buffer_rx is full under app back-pressure -> those bytes are LOST with
			// a committed ACK and the CMD never retransmits them (silent post-ACK data loss).
			// HOLD the whole batch-complete ACK+deliver until the RX app FIFO has room for the
			// batch's worst-case delivered size: keep messages_rx RECEIVED (do NOT mark ACKED,
			// do NOT bump the bsi, do NOT ACK, do NOT deliver) and re-arm the receive timer, the
			// SAME re-arm the partial-batch SACK-suppress path above uses. The CMD's ACK-timeout
			// retransmits and this handler re-fires once the app drains -> the batch delivers +
			// ACKs then, with NO byte ever ACKed-but-undelivered (back-pressure correctly stalls
			// the sender instead of dropping data). passive_monitor never ACKs, so it is exempt.
			// Env MERCURY_RXFIFO_BACKPRESSURE_DEFEAT=1 reverts on the SAME binary (fail-before:
			// ACK+deliver regardless -> the short-store loss reproduces).
			if(!passive_monitor)
			{
				bool bp_defeat = false;
				{ const char* e = std::getenv("MERCURY_RXFIFO_BACKPRESSURE_DEFEAT");
				  if(e && *e && atoi(e)!=0) bp_defeat = true; }
				int rx_free = fifo_buffer_rx.get_free_size();
				int need    = rx_fifo_batch_need();
				if(!bp_defeat && rx_free < need)
				{
					t1_cancel_schedkey("rxfifo-backpressure-hold");
					printf("[ACK-GATE-BACKPRESSURE] HOLD: app FIFO free=%d < batch need=%d — "
						"NOT ACKing/delivering; CMD retransmits, re-deliver when app drains "
						"(Fix H#3, no post-ACK loss)\n", rx_free, need);
					fflush(stdout);
					// Keep messages_rx RECEIVED (do not free), re-arm for the retransmit —
					// identical discipline to the partial-batch SACK-suppress re-arm above.
					stats.nNAcked_data++;
					batch_rx_frame_count = 0;
					last_received_end_of_batch_seq = -1;
					telecom_system->data_container.frames_to_read =
						telecom_system->data_container.preamble_nSymb
						+ telecom_system->get_active_nsymb();
					telecom_system->data_container.nUnder_processing_events = 0;
					calculate_receiving_timeout();
					receiving_timer.start();
					connection_status = RECEIVING;
					return;
				}
			}

			// Mark the received messages inside the effective window as ACKED and
			// count them. The hold revert and the commit below use the same range;
			// a RECEIVED slot outside it can never be delivered (the copy frees it
			// without delivery), so it is freed here, loudly, and not counted.
			for(int i=0; i<this->nMessages; i++)
			{
				if(messages_rx[i].status!=RECEIVED)
					continue;
				if(i < eff_window)
				{
					messages_rx[i].status=ACKED;
					nAck_messages++;
				}
				else
				{
					messages_rx[i].status=FREE;
					g_rsp_ack_oow_slot_count++;
					printf("[RSP-ACK-OOW-SLOT] slot=%d outside eff_window=%d "
						"(bsi=%d batch=%d) freed, not ACKed (count=%lld)\n",
						i, eff_window, rsp_current_expected_batch_seq_id,
						data_batch_size, g_rsp_ack_oow_slot_count);
					fflush(stdout);
				}
			}
			stats.nAcks_sent_data += nAck_messages;

			// SACK Design A Step 4 — full batch ACKed → bump current_expected.
			// This is the ONLY site that bumps; SACK-partial paths above
			// (line ~852-889) leave current_expected unchanged because the
			// in-flight batch is not yet complete. Mechanism-(a) retransmits
			// carry the same batch_seq_id, so they continue to match current.
			// Per §4.3.4 invariant #2: monotonic +1 mod 256, never reset.
			if(sack_v2_enabled && rsp_current_expected_batch_seq_id >= 0)
			{
				// D3.1 (data-integrity, the UNIFIED keystone): DELIVERY-TIME gap
				// gate. The batch about to be delivered to the app at
				// copy_data_to_buffer() (below) is the one whose bsi is in
				// rsp_current_expected_batch_seq_id. If committing it would step
				// the high-water by >=2, a batch between last_delivered and this
				// one was never delivered -> a HOLE -> silent concat. This fires
				// REGARDLESS of how cur got here (any of the 4 SET_CONFIG demotes
				// that bypass the cur<0 FIX-8 re-adopt gate, the PREV-BUMP/STALE
				// strand, etc.) — the inviolable safety net the 5 reverted
				// config-transition point-patches lacked (FIX9_D3_DESIGN §7.2).
				// MERCURY_GAP_ABORT_DEFEAT=1 disables it (the --test-inorder-demote
				// fail-before arm), restoring pre-fix silent concat.
				bool gap_defeat = false;
				{ const char* e = std::getenv("MERCURY_GAP_ABORT_DEFEAT");
				  if(e && *e && atoi(e)!=0) gap_defeat = true; }
				// rsp_stream_aborted: refuse the BATCH-DONE delivery unconditionally once
				// the session is latched torn-down (see the re-adopt gate).
				if(!gap_defeat
				   && (rsp_stream_aborted
				       || delivery_step_is_gap(rsp_current_expected_batch_seq_id,
				                               rsp_last_delivered_batch_seq_id, rsp_stream_origin_gen)))
				{
					// R2a — RECOVERABLE HOLD (data-flow-recoverable-gap-abort.md).
					// Before the terminal abort, check whether this is the ONE healable
					// topology: the completed current batch is exactly +2 past
					// last_delivered AND the single missing batch is the armed,
					// partially-received prev. If so — and the session is NOT P0-latched
					// (rsp_stream_aborted forces the abort), and we are within the round
					// bound — HOLD the current batch instead of aborting: revert its
					// slots ACKED->RECEIVED (so it is NOT a deliverable ACKED batch),
					// deliver NOTHING, clean-ACK NOTHING, advance the high-water NOTHING,
					// and re-advertise the prev hole so the CMD re-drives the missing
					// frame (R2c re-queues the retained bytes). The contiguity ruler
					// (delivery_step_is_gap) stays the inviolable assert BELOW — the hold
					// is strictly UPSTREAM and never lets a byte pass the hole; the held
					// batch delivers ONLY after the prev hole fills and the ruler shows
					// contiguity (deliver-held-cur at the PREV-completion commit).
					// MERCURY_GAP_RECOVER_DEFEAT=1 forces the pre-fix terminal abort.
					bool recover_defeat = false;
					{ const char* e = std::getenv("MERCURY_GAP_RECOVER_DEFEAT");
					  if(e && *e && atoi(e)!=0) recover_defeat = true; }
						if(!recover_defeat
					   && !rsp_stream_aborted
					   && rsp_gap_recover_rounds < RSP_GAP_RECOVER_MAX
					   && gap_is_recoverable_prev_hole(
							rsp_current_expected_batch_seq_id,
							rsp_last_delivered_batch_seq_id,
							rsp_prev_batch_active, rsp_prev_batch_seq_id,
							rsp_prev_batch_received_count, rsp_stream_origin_gen))
						{
							printf("[T1-SAFETY-COVER] recoverable-prev-hole-hold entered "
								"keyed=%d bsi=%d\n", t1_schedkey.keyed ? 1 : 0,
								rsp_current_expected_batch_seq_id);
							fflush(stdout);
							t1_cancel_schedkey("recoverable-prev-hole-hold");
							// Revert the just-marked ACKED current-batch slots back to
						// RECEIVED so this batch is HELD (survives for delivery once the
						// hole fills) and is NOT mistaken for a deliverable ACKED batch.
						int reverted = 0;
						for(int i=0; i<eff_window && i<this->nMessages; i++)   // Option B' (§9): eff_window is the local ACK-gate window
						{
							if(messages_rx[i].status==ACKED)
							{ messages_rx[i].status=RECEIVED; reverted++; }
						}
						nAck_messages = 0;
						rsp_gap_recover_rounds++;
							// R2b -- capture the frame-count completeness oracle NOW, while it is
							// still known-true. The hold below clears last_received_end_of_batch_seq /
							// rx_batch_total_frames (the two inputs to `expected`), so the
							// deliver-held-cur gate at PREV-completion could not otherwise re-derive
							// how many slots this held batch requires. `expected` is the exact count
							// the BATCH-DONE gate just used to declare this batch frame-complete, so
							// all slots [0, expected-1] of messages_rx[] are RECEIVED right now.
							rsp_gap_hold_cur_expected = expected;
						printf("[RSP-V2-GAP-HOLD] recoverable prev hole: HOLD cur bsi=%d "
							"(reverted %d ACKED->RECEIVED) prev bsi=%d %d/%d "
							"last_delivered=%d round=%d/%d — re-advertising prev hole, "
							"NOT delivering/advancing\n",
							rsp_current_expected_batch_seq_id, reverted,
							rsp_prev_batch_seq_id, rsp_prev_batch_received_count,
							rsp_prev_batch_expected_count,
							rsp_last_delivered_batch_seq_id,
							rsp_gap_recover_rounds, RSP_GAP_RECOVER_MAX);
						fflush(stdout);
						rsp_resend_prev_partial_sack();
						// Re-arm for the CMD's re-driven retx — mirror the partial-SACK
						// re-arm below (:2481-2502). Do NOT override frames_to_read (the
						// SACK TX already set the turnaround window); keep messages_rx
						// RECEIVED (do NOT free — the retx fills the prev hole).
						repeating_last_ack       = NO;
						messages_control.status  = FREE;
						stats.nNAcked_data++;
						batch_rx_frame_count           = 0;
						last_received_end_of_batch_seq = -1;
						telecom_system->set_mfsk_ctrl_mode(false);
						telecom_system->data_container.nUnder_processing_events = 0;
						calculate_receiving_timeout();
						// LINK-PHASE STEP 4 (MC-6): the prev-hole re-advertise just re-sent the
						// PREV partial SACK (rsp_resend_prev_partial_sack above), so the missing
						// count is the prev batch's shortfall. Derive the retx-turn listen window
						// from it (flag OFF => stock 1.5x, byte-identical).
						{
							int prev_missing = rsp_prev_batch_expected_count
							                 - rsp_prev_batch_received_count;
							if(linkphase_retxslot_on() && prev_missing > 0)
								receiving_timeout = linkphase_derive_retx_slot(prev_missing, receiving_timeout);
							else
								receiving_timeout = receiving_timeout * 3 / 2;
						}
						receiving_timer.start();
						connection_status = RECEIVING;
						return;
					}
					char reason[112];
					snprintf(reason, sizeof(reason),
						"delivery-time BATCH-DONE bsi=%d non-contiguous with last_delivered=%d (dropped-batch hole)",
						rsp_current_expected_batch_seq_id, rsp_last_delivered_batch_seq_id);
					rsp_gap_abort_teardown(reason);
					// Skip the delivery entirely: do NOT advance, do NOT bump, do
					// NOT copy_data_to_buffer below (guarded by batch_gap_aborted).
					batch_gap_aborted = true;
				}
				else
				{
					// R2b: the in-order BATCH-DONE commit is now the SHARED
					// rsp_commit_cur_batch_delivery() helper (advance high-water,
					// roll prev<-cur / cur<-cur+1, reset the wired count, and push
					// messages_rx[] to the app FIFO). The slots were already flipped
					// ACKED at :2650 (before the gap gate, so the recoverable HOLD
					// could revert them), so the helper's mark is a no-op here. It
					// delivers via copy_data_to_buffer BEFORE the clean-ACK TX below;
					// the RXFIFO back-pressure gate above already guaranteed FIFO room,
					// so no byte is ever ACKed-but-undelivered. batch_data_delivered is
					// set so the tail copy_data_to_buffer at the ACK-emit site skips
					// (single delivery). The PREV-completion deliver-held-cur site calls
					// the SAME helper — ONE delivery implementation under the D3.1 gates.
					rsp_commit_cur_batch_delivery();
					batch_data_delivered = true;
					printf("[RSP-V2-BATCH-DONE] prev=%d next_expected=%d last_delivered=%d\n",
						rsp_prev_batch_seq_id, rsp_current_expected_batch_seq_id,
						rsp_last_delivered_batch_seq_id);
					fflush(stdout);
				}
			}
		}
		repeating_last_ack=NO;
		messages_control.status=FREE;

		// Clean-batch ACK transport. WB sessions use the MFSK-suffix
		// ACK+SACK frame (16-symbol Welch-Costas pattern + 13 MFSK data
		// symbols carrying [bsi:8 | bitmap:30 | crc12:12]). NB sessions
		// (suffix_len==0) fall back to the legacy MFSK ACK pattern with
		// no SACK payload — NB doesn't have the symbol-rate budget for
		// SACK and the receiver implicitly treats any pattern hit as a
		// clean ACK. See mercury/fact-documents/mfsk-robust-ack.md §3.4.
		//
		// D3.1 (data-integrity): if the delivery-time gap gate just tore the
		// session down (batch_gap_aborted), the RSP is in a reset/DROPPED state
		// whose bsi window no longer describes a live batch. Emitting a clean
		// ACK/SACK from here would be a ZOMBIE confirm — rsp_prev_batch_seq_id was
		// cleared to -1 by the teardown, so ack_bsi falls back to 0 and the RSP
		// would tell the CMD "batch 0 received cleanly" for a transfer it just
		// aborted. Never confirm from a torn-down state; the aborted link stays
		// silent and the peer times out.
		if(batch_gap_aborted)
		{
			// no confirm emitted — see the gap-abort rationale above.
		}
		else if(l1_blockack_data_active())
		{
			const uint8_t ack_bsi = (uint8_t)(
				rsp_prev_batch_seq_id >= 0 ? rsp_prev_batch_seq_id : 0);
			std::vector<bool> received((std::size_t)eff_window, true);
			std::vector<uint8_t> wire;
			if(l1_blockack.observe_received_batch(ack_bsi, received,
				/*final=*/false, &wire))
			{
				l1_send_block_control(wire);
			}
			else
			{
				// No reverse keying for an intermediate clean batch. Re-arm the
				// production LDPC receive geometry for the next forward batch.
				telecom_system->data_container.frames_to_read =
					telecom_system->data_container.preamble_nSymb
					+ telecom_system->get_active_nsymb();
				printf("[L1-BLOCKACK-RSP] clean bsi=%u retained; ACK deferred\n",
					(unsigned)ack_bsi);
				fflush(stdout);
			}
		}
		else if(sack_v2_enabled)
		{
			unsigned char ack_bsi = (unsigned char)(
				rsp_prev_batch_seq_id >= 0 ? rsp_prev_batch_seq_id : 0);

			bool used_mfsk_path = false;
			if (MFSK_ACK_SACK_ENABLED
				&& telecom_system->ack_mfsk.ack_sack_suffix_len() > 0)
			{
				// Phase B Wave 1 flag-day (fact-doc §11.2): bitmap is 30 bits
				// (was 32). Producer side cap so we never set bits 30/31.
				uint32_t bitmap_u32 = mfsk_sack_mask_for_frames(eff_window);
					// A2: ack_bsi is the delivered clean target. The encoder derives
					// wire=target-1 in cumulative mode; the implicit/all-ones result credits
					// target only after the commander resolves and validates that identity.
				unsigned char wire_bsi = cumulative_ack_bsi_field(
					ack_bsi, rsp_last_delivered_batch_seq_id, cumulative_ack_enabled);
				printf("[RSP-MFSK-SACK] clean path: batch_seq_id=%u (wire_bsi=%u n_r=%d cum=%d) bitmap=0x%08x nframes=%d\n",
					(unsigned)ack_bsi, (unsigned)wire_bsi,
					rsp_last_delivered_batch_seq_id, cumulative_ack_enabled ? 1 : 0,
					(unsigned)bitmap_u32, data_batch_size);
				fflush(stdout);
				// WALL-B FIX-9 D2 REFINE (§2.1): clean = a fully-received first-pass batch ACK (the
				// dominant clean-channel case) -> NOT a retransmit turnaround -> NO robust settle (key
				// on the tight OFDM turnaround, byte-identical to D2-off / D3-base; recovers the cost).
				ack_tx_retx_turnaround = false;

				// Option B (data-flow-compact-confirm.md): for a CLEAN batch, prefer the
				// COMPACT coded confirm — ~70ms shorter AND ~4 dB more robust than the
				// 13-uncoded suffix. Gated OFF until the faithful real-audio re-verify
				// (ARQ_COMPACT_CONFIRM_ENABLE). Scalable mode carries the negotiated
				// cumulative high-water in the same compact bsi field.
				long long mfsk_ms = 0;
				if (ARQ_COMPACT_CONFIRM_ENABLE
					&& telecom_system->ack_mfsk.compact_confirm_suffix_len() > 0
					&& (!cumulative_ack_enabled || scalable_sack_on()))
				{
					mfsk_ms = send_mfsk_compact_confirm(ack_bsi);
					if (mfsk_ms > 0)
					{
						printf("[TX-ACK-SACK] clean via COMPACT confirm wire_ms=%lld\n", mfsk_ms);
						fflush(stdout);
						used_mfsk_path = true;
					}
				}
				if (!used_mfsk_path)
				{
					mfsk_ms = send_mfsk_ack_sack(ack_bsi, bitmap_u32);
				}
				if (mfsk_ms > 0)
				{
					if (!used_mfsk_path)
						printf("[TX-ACK-SACK] clean via MFSK suffix wire_ms=%lld\n",
							mfsk_ms);
					fflush(stdout);
					used_mfsk_path = true;
				}
				else
				{
					printf("[RSP-MFSK-SACK] MFSK suffix returned 0 — falling back to legacy MFSK ACK pattern\n");
					fflush(stdout);
				}
			}
			if (!used_mfsk_path)
			{
				// NB has no content-bearing bare ACK.  A queued reverse stream therefore
				// upgrades this clean response to the CRC/LDPC-protected full-SACK
				// format, whose flags octet carries D=1.  With D=0 the legacy bare ACK
				// remains unchanged.
				if(local_reverse_data_demand())
				{
					bool clean_bitmap[MAX_SACK_BATCH_SIZE];
					for(int i=0; i<eff_window && i<MAX_SACK_BATCH_SIZE; i++)
						clean_bitmap[i] = true;
					send_sack_v2_frame(clean_bitmap, eff_window, ack_bsi);
				}
				else
					send_ack_pattern();
			}
			// SEAM-2 FLUSH-A: a deferred prev clean-confirm rides for free if this
			// bsi-carrying clean/current ACK we just emitted cumulatively covers it.
			// Only via the MFSK/compact transport (used_mfsk_path); a bsi-less legacy
			// pattern credits the CMD's CURRENT batch, never a stale prev, so it must
			// NOT clear the pending. Clear without re-emitting; a lost repeat self-heals
			// on the next cumulative n_r.
			if(linkphase_pending_prev_confirm && used_mfsk_path && cumulative_ack_enabled)
			{
				unsigned char emitted_nr = cumulative_ack_bsi_field(
					ack_bsi, rsp_last_delivered_batch_seq_id, cumulative_ack_enabled);
				linkphase_clear_pending_if_covered(emitted_nr);
			}
		}
		else
		{
			// Legacy/NB bare ACK carries D=0 only.  D=1 uses the same singleton
			// full-SACK demand record as negotiated SACK sessions.
			if(local_reverse_data_demand())
			{
				bool clean_bitmap[MAX_SACK_BATCH_SIZE];
				for(int i=0; i<eff_window && i<MAX_SACK_BATCH_SIZE; i++)
					clean_bitmap[i] = true;
				unsigned char ack_bsi = (unsigned char)(
					rsp_prev_batch_seq_id >= 0 ? rsp_prev_batch_seq_id : 0);
				send_sack_v2_frame(clean_bitmap, eff_window, ack_bsi);
			}
			else
				send_ack_pattern();
		}

		// The first clean turnaround is intentionally baseline-only.  Publish
		// PREKEY eligibility only after its clean ACK has left this endpoint;
		// batch 2 is the earliest batch that may carry/schedule the overlap.
		if(!passive_monitor && !batch_gap_aborted && !l1_blockack_data_active())
			t1_data_phase_proven = true;

		if(passive_monitor)
		{
			// send_ack_pattern was suppressed — set generous ftr for turnaround.
			// Monitor must wait through: real responder's ACK TX (~750ms) +
			// commander ACK detection + processing (~500ms) + next batch TX.
			// Use 2x rx_frame to cover the full turnaround gap.
			int rx_frame = telecom_system->data_container.preamble_nSymb
			             + telecom_system->data_container.Nsymb;
			telecom_system->data_container.frames_to_read = rx_frame * 2;
			telecom_system->data_container.nUnder_processing_events = 0;
		}

		// send_ack_pattern() sets ftr = rx_frame + turnaround_symbols to cover
		// the full turnaround gap (ACK TX → commander ACK detect → encode →
		// batch TX → preamble arrives).  DO NOT override ftr here — the old
		// ftr=rx_frame approach caused the preamble to always land 19 symbols
		// past upper_bound (position=buf-33, upper=buf-52), producing the
		// persistent 22 OK / 9 FAIL pattern on VB-Cable benchmarks.
		telecom_system->set_mfsk_ctrl_mode(false);
		telecom_system->data_container.nUnder_processing_events = 0;

		if(g_verbose) {
			printf("[ACK-DATA] ftr=%d (from send_ack_pattern turnaround)\n",
				telecom_system->data_container.frames_to_read.load());
			fflush(stdout);
		}

		// BLOCK_END eliminated: flush data to application immediately after
		// sending pattern ACK. Commander finalizes locally in parallel.
		// D3.1: if the BATCH-DONE delivery-time gap gate fired, the transfer is
		// DROPPED — do NOT push this gapped batch to the app FIFO (the whole
		// point of the gate is to refuse the silent concat).
		if(!batch_data_delivered && !batch_gap_aborted)
		{
			// AEAD nonce source (data-flow-aead-nonce.md §5): the in-order BATCH-
			// DONE delivery. rsp_current_expected_batch_seq_id was ALREADY advanced
			// at :2429 before this call, so the DELIVERED batch's wire bsi lives in
			// rsp_prev_batch_seq_id (set at :2428). Binding to current here would
			// use the WRONG (next) bsi.
			decrypt_delivered_bsi = rsp_prev_batch_seq_id;
			copy_data_to_buffer();
			batch_data_delivered = true;  // Prevent duplicate delivery on retransmit
			// silent-corruption-marginal-snr.md §3: the in-order batch just delivered its
			// full framed span — now apply any data_batch_size shrink that was deferred to
			// avoid orphaning THIS (current) batch's RECEIVED tail. No-op if none pending or
			// a prev is still holding (rsp_apply_deferred_batch_shrink self-guards). Mirrors
			// the prev-deliver apply site (arq_responder.cc:1346).
			rsp_apply_deferred_batch_shrink();
		}
		messages_last_ack_bu.type=NONE;

		if(passive_monitor)
		{
			// Monitor: generous timeout covering real responder ACK +
			// commander turnaround + next batch TX
			int monitor_timeout = data_batch_size * message_transmission_time_ms
				+ time_left_to_send_last_frame + ptt_on_delay_ms + 5000;
			set_receiving_timeout(monitor_timeout);
		}
		else
		{
			calculate_receiving_timeout();
		}
		receiving_timer.start();
		batch_rx_frame_count = 0;
		last_received_end_of_batch_seq = -1;
		batch_data_delivered = false;  // Ready for next batch (e924d89 bug: was never reset → all batches after first silently dropped)
		connection_status=RECEIVING;
	}
	else
	{
		// Fallback: send LDPC-encoded ACK_MULTI frame (not currently reachable)
		if(repeating_last_ack==YES)
		{
			messages_control.status=FREE;
			message_batch_counter_tx=0;
			if(messages_last_ack_bu.type==ACK_MULTI ||messages_last_ack_bu.type==ACK_RANGE)
			{
				messages_batch_ack[message_batch_counter_tx].type=messages_last_ack_bu.type;
				messages_batch_ack[message_batch_counter_tx].id=messages_last_ack_bu.id;
				messages_batch_ack[message_batch_counter_tx].length=messages_last_ack_bu.length;
				for(int i=0;i<messages_batch_ack[message_batch_counter_tx].length;i++)
				{
					messages_batch_ack[message_batch_counter_tx].data[i]=messages_last_ack_bu.data[i];
				}
			}
			else
			{
				messages_batch_ack[message_batch_counter_tx].type=NONE;
				messages_batch_ack[message_batch_counter_tx].id=0;
				messages_batch_ack[message_batch_counter_tx].length=0;
			}
			repeating_last_ack=NO;
		}
		else
		{
			nAck_messages=0;
			for(int i=0;i<this->nMessages;i++)
			{
				if(messages_rx[i].status==RECEIVED)
				{
					nAck_messages++;
				}
			}
			message_batch_counter_tx=0;
			messages_batch_ack[message_batch_counter_tx].type=ACK_MULTI;
			messages_batch_ack[message_batch_counter_tx].id=0;
			// Clamp to buffer size — init_messages_buffers allocated N_MAX/8 bytes.
			// Don't reallocate: reuse existing buffer to avoid memory leak (Bug #17).
			if(nAck_messages + 1 > N_MAX / 8)
				nAck_messages = N_MAX / 8 - 1;
			messages_batch_ack[message_batch_counter_tx].length=nAck_messages+1;
			messages_batch_ack[message_batch_counter_tx].data[0]=nAck_messages;

			int counter=1;
			for(int i=0;i<this->nMessages;i++)
			{
				if(messages_rx[i].status==RECEIVED)
				{
					messages_rx[i].status=ACKED;
					messages_batch_ack[message_batch_counter_tx].data[counter]=i;
					counter++;
				}
			}

			messages_last_ack_bu.type=messages_batch_ack[message_batch_counter_tx].type;
			messages_last_ack_bu.id=messages_batch_ack[message_batch_counter_tx].id;
			messages_last_ack_bu.length=messages_batch_ack[message_batch_counter_tx].length;
			for(int i=0;i<messages_last_ack_bu.length;i++)
			{
				messages_last_ack_bu.data[i]=messages_batch_ack[message_batch_counter_tx].data[i];
			}
			stats.nAcks_sent_data+=nAck_messages;
		}
		messages_batch_tx[message_batch_counter_tx]=messages_batch_ack[message_batch_counter_tx];
		message_batch_counter_tx++;

		load_configuration(ack_configuration, PHYSICAL_LAYER_ONLY,NO);

		telecom_system->set_mfsk_ctrl_mode(true);  // data ACK TX (short ctrl frame)
		pad_messages_batch_tx(ack_batch_size);
		send_batch();

		load_configuration(data_configuration, PHYSICAL_LAYER_ONLY,YES);
		// Expect data frames next: use full Nsymb for capture.
		// Frame completeness gating handles late arrivals adaptively.
		// MULTI-CW WINDOW FIX (fact-doc §17): at the bigblock rung the next thing we
		// receive is ONE K-codeword block (~64 sym), not a stock frame (~13) — block-span
		// the window so the decode snapshot waits for the WHOLE block (cw1..7 fresh).
		telecom_system->set_mfsk_ctrl_mode(false);
		telecom_system->data_container.frames_to_read = bigblock_block_ftr_or(
			telecom_system->data_container.preamble_nSymb + telecom_system->data_container.Nsymb + 10);

		batch_rx_frame_count = 0;
		connection_status=RECEIVING;
	}

	// ACK_RANGE
	//	int nAcks_sent=0;
	//	int nAck_messages=0;
	//	receiving_timer.stop();
	//	receiving_timer.reset();
	//	for(int j=0;j<ack_batch_size;j++)
	//	{
	//		int start=-1;
	//		int end=-1;
	//		for(int i=0;i<this->nMessages;i++)
	//		{
	//			if(messages_rx[i].status==RECEIVED)
	//			{
	//				start=i;
	//				end=i;
	//				break;
	//			}
	//		}
	//		for(int i=start+1;i<this->nMessages;i++)
	//		{
	//			if(messages_rx[i].status==RECEIVED)
	//			{
	//				end=i;
	//			}
	//			else
	//			{
	//				break;
	//			}
	//		}
	//
	//		if(start!=-1)
	//		{
	//			messages_batch_ack[message_batch_counter_tx].type=ACK_RANGE;
	//			messages_batch_ack[message_batch_counter_tx].id=(char)start;
	//			messages_batch_ack[message_batch_counter_tx].length=2;
	//			messages_batch_ack[message_batch_counter_tx].data=new char[2];
	//			messages_batch_ack[message_batch_counter_tx].data[0]=(char)start;
	//			messages_batch_ack[message_batch_counter_tx].data[1]=(char)end;
	//			nAcks_sent=end-start+1;
	//
	//			for(int i=start;i<=end;i++)
	//			{
	//				messages_rx[i].status=ACKED;
	//			}
	//
	//			messages_batch_tx[message_batch_counter_tx]=messages_batch_ack[message_batch_counter_tx];
	//			message_batch_counter_tx++;
	//			nAck_messages++;
	//			stats.nAcks_sent_data+=nAcks_sent;
	//		}
	//
	//		if(nAcks_sent>=ack_batch_size || get_nReceived_messages()==0)
	//		{
	//			pad_messages_batch_tx(ack_batch_size);
	//
	//			send_batch();
	//			connection_status=RECEIVING;
	//			break;
	//		}
	//	}
}

void cl_arq_controller::process_control_responder()
{
	char code=messages_control.data[0];
	printf("[RX-CTRL] Processing control message: code=%d (0=START, 1=TEST, 2=SET_CFG, 3=BLOCK_END, 4=FILE_END, 5=SWITCH, 6=CLOSE, 7=REPEAT)\n", (int)code);
	if((link_status==LISTENING || link_status==CONNECTION_RECEIVED) && code==START_CONNECTION)
	{
		unsigned char received_crc = (unsigned char)messages_control.data[1];
		unsigned char my_crc = CRC8_calc((char*)my_call_sign.c_str(), my_call_sign.length());
		printf("[RX-CTRL] START_CONNECTION received. CRC check: received=0x%02X, my_call='%s' (len=%d), my_crc=0x%02X\n",
			received_crc, my_call_sign.c_str(), (int)my_call_sign.length(), my_crc);

		if(passive_monitor)
		{
			begin_radio_session_attempt();
			// Monitor mode: accept ANY START_CONNECTION, extract callsigns, fast-track to CONNECTED
			int peer_flags = 0;
			destination_call_sign = callsign_unpack(&messages_control.data[2], &peer_flags);
			printf("[MONITOR] Observed START_CONNECTION from '%s'\n", destination_call_sign.c_str());
			fflush(stdout);

			bool peer_narrowband = (peer_flags & 0x01) != 0;
			session_narrowband = peer_narrowband || (narrowband_enabled == YES);
			compression_enabled = true;  // Assume compression (almost always on)

			// Genuine new session: clear the gap-abort data-integrity latch (the SOLE
			// clear point). A prior mid-transfer abort latched rsp_stream_aborted so no
			// delivery could ride the torn-down stream; a fresh CONNECT re-authorises delivery.
			rsp_stream_aborted = false;
			// Reconnect-continuity fail-closed (data-flow-reconnect-continuity.md §5b): the sole
			// arm point, co-located with the sole clear of the abort latch. A fresh session on a
			// persistent app data socket with a prior-session delivered high-water must fail-closed
			// its first (byte-unprovable) delivery rather than splice.
			rsp_reconnect_seam_arm_on_accept();

			link_status = CONNECTED;
			connection_status = RECEIVING;
			messages_control.status = FREE;  // Critical: free so next control frame can be received
			watchdog_timer.start();
			link_timer.start();

			// Prepare for next frame capture
			calculate_receiving_timeout();
			receiving_timer.start();

#ifdef MERCURY_GUI_ENABLED
			gui_set_monitor_callsigns(destination_call_sign.c_str(), "???");
			{
				char buf[128];
				snprintf(buf, sizeof(buf), "[MONITOR: session %s -> ???]", destination_call_sign.c_str());
				gui_push_monitor_event(buf, false);
			}
#endif
		}
		else if(received_crc == my_crc)
		{
			int peer_flags = 0;
			destination_call_sign = callsign_unpack(&messages_control.data[2], &peer_flags);
			printf("[RX-CTRL] Unpacked commander callsign: '%s', flags=0x%02X\n", destination_call_sign.c_str(), peer_flags);
			fflush(stdout);

			// NB/WB auto-negotiation: NB always wins
			bool peer_narrowband = (peer_flags & 0x01) != 0;
			bool local_narrowband = (narrowband_enabled == YES);
			session_narrowband = peer_narrowband || local_narrowband;
			if(session_narrowband && !local_narrowband)
			{
				printf("[NB-NEG] Responder: commander wants NB, will switch after ACK\n");
				fflush(stdout);
			}
			else if(session_narrowband && local_narrowband && !peer_narrowband)
			{
				printf("[NB-NEG] Responder: local is NB, peer is WB, session=NB\n");
				fflush(stdout);
			}

			// PENDING: a connect addressed to MYCALL is being received. Bare token
			// (the callsign is delivered on CONNECTED), emitted here at the
			// START_CONNECTION crc-match unless the directed-HAIL site already
			// announced this attempt (the shared rsp_emit_pending latch keeps the
			// two sites mutually exclusive -- exactly one PENDING per inbound
			// attempt). Lets a scanner-control host hold and prepare PTT.
			rsp_emit_pending();

			// Genuine new session: clear the gap-abort data-integrity latch (the SOLE
			// clear point, mirroring the passive-monitor accept above). A prior
			// mid-transfer abort latched rsp_stream_aborted so no delivery could ride the
			// torn-down stream; a fresh CONNECT re-authorises delivery for this new session.
			rsp_stream_aborted = false;
			// Reconnect-continuity fail-closed (data-flow-reconnect-continuity.md §5b): the sole
			// arm point, co-located with the sole clear of the abort latch. A fresh session on a
			// persistent app data socket with a prior-session delivered high-water must fail-closed
			// its first (byte-unprovable) delivery rather than splice.
			rsp_reconnect_seam_arm_on_accept();

			link_status=CONNECTION_RECEIVED;
			connection_status=ACKNOWLEDGING_CONTROL;
			if(ack_pattern_time_ms > 0)
			{
				// ACK pattern carries no data, both sides use BROADCAST_ID
				messages_control.data[1]=BROADCAST_ID;
			}
			else
			{
				// Fallback: assign random connection_id sent back in ACK frame
				messages_control.data[1]=1+rand()%0xfe;
			}
			messages_control.length=2;
			assigned_connection_id=messages_control.data[1];
			watchdog_timer.start();
		}
		else
		{
			printf("[RX-CTRL] START_CONNECTION REJECTED - callsign CRC mismatch! Is MYCALL set correctly?\n");
			messages_control.status=FREE;
		}
	}
	else if((link_status==CONNECTION_RECEIVED || link_status==CONNECTED) && code==TEST_CONNECTION)
	{
		u_SNR tmp_SNR;
		for(int i=0;i<4;i++)
		{
			tmp_SNR.char4_SNR[i]=messages_control.data[i+1];
		}
		measurements.SNR_uplink=quantize_snr_decision_db((double)tmp_SNR.f_SNR);

		// Read commander's capability from byte 5 of decoded LDPC frame.
		// Always present (LDPC decodes full block; unused bytes are zero-padded).
		// Backwards-compatible: old firmware doesn't fill byte 5 → decodes as 0 = no WB.
		peer_capability = (uint8_t)messages_control.data[5];
		int t1_pk = -1, t1_pd = -1, t1_pu = -1;
		bool t1_ldpc_calibration = messages_control.length >= 12
			&& t1_calibration_decode(
				&messages_control.data[7], 5, &t1_pk, &t1_pd, &t1_pu);
		if(t1_ldpc_calibration)
		{
			t1_peer_calibration_valid = true;
			t1_peer_keyup_ms = t1_pk;
			t1_peer_deaf_ms = t1_pd;
			t1_peer_undeaf_ms = t1_pu;
			printf("[T1-CALIB-WIRE] peer keyup=%d deaf=%d undeaf=%d ms\n",
				t1_pk, t1_pd, t1_pu);
			fflush(stdout);
		}
		printf("[BW-NEG] Commander capability: 0x%02X (WB=%s, ENCRYPT=%s)\n",
			peer_capability,
			(peer_capability & CAP_WB_CAPABLE) ? "yes" : "no",
			(peer_capability & CAP_ENCRYPTION) ? "yes" : "no");
		fflush(stdout);
		// NB robust-preamble negotiation: the commander's advertise just arrived.
		// From here on this responder may emit the sidelnikov robust preamble
		// (both-support), starting with the TEST_CONNECTION_ACK it queues below
		// on robust configs — the commander's RX runs detect-both either way.
		update_robust_preamble_negotiation();
		// The decoded TEST_CONNECTION is the authenticated peer capability
		// proof on the responder. Stage-3 still requires both advertised bits.
		l1_complete_blockack_handshake(true);

		// Read commander's SSID from byte 6 (sent separately from packed callsign)
		{
			int peer_ssid = (uint8_t)messages_control.data[6];
			if(peer_ssid != SSID_NONE)
			{
				destination_call_sign = callsign_format_ssid(destination_call_sign, peer_ssid);
				printf("[SSID] Commander SSID=%d, full callsign: '%s'\n", peer_ssid, destination_call_sign.c_str());
			}
			else
			{
				printf("[SSID] Commander has no SSID, callsign: '%s'\n", destination_call_sign.c_str());
			}
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
		// operator did not opt in the decision is PLAINTEXT_OK and this block is a
		// no-op, so legal Part-97 plaintext operation is unchanged.
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
				// Commander opted into -E but this responder does not advertise
				// CAP_ENCRYPTION to it (unsupported, or a MITM stripped the cap bit
				// to force a plaintext downgrade). Fail closed for BOTH strict and
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
		// §T2.1): the cumulative-n_r SACK reshape engages ONLY when BOTH ends advertise
		// CAP_CUMULATIVE_ACK (the local advertise is itself env-gated by
		// MERCURY_CUMULATIVE_ACK, so default-off ≡ both_support false ≡ per-batch +
		// byte-identical). SAME both_support pattern as encryption above. Interop-safe:
		// a non-Tier-2 commander leaves bit 2 clear → cumulative_ack_enabled false →
		// per-batch fallback on the RSP send AND the CMD apply.
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
			// matches the formula in arq_common.cc + arq_commander.cc.
			//
			// climb-engine Bug 2 (gearshift-climb-engine.md §3): MUST mirror the
			// commander gate (arq_commander.cc SACK block) so CMD and RSP agree on
			// data_batch_size for the negotiated config. ROBUST/MFSK configs are
			// EXCLUDED from the >=5 floor — load_configuration() (arq_common.cc:
			// 1229-1234) pins data_batch_size=1 for robust; re-applying the floor
			// here would force RSP to batch>=5 while CMD stays at 1, a CMD/RSP
			// batch mismatch. The responder gates on current_configuration (== the
			// established connect config at TEST_CONNECTION time, == ROBUST_0 for a
			// robust connect, < 100 for OFDM). OFDM keeps the >=5 floor unchanged.
			// Shared with the CMD TEST_CONNECTION_ACK handler so both sides run
			// IDENTICAL code and cannot diverge on data_batch_size (the 4-time
			// wire-failure mode). Gates on current_configuration. See
			// data-flow-batch-size.md §5 and the helper in arq_common.cc.
			sack_negotiated_recompute_batch("RSP");
		}
		else
		{
			printf("[SACK] Disabled by --no-sack\n");
		}
		fflush(stdout);

		tmp_SNR.f_SNR=(float)measurements.SNR_downlink;
		for(int i=0;i<4;i++)
		{
			messages_control.data[i+1]=tmp_SNR.char4_SNR[i];;
		}
		messages_control.data[5]=(char)local_capability;
		messages_control.data[6]=(char)callsign_get_ssid(my_call_sign);
		messages_control.length=7;

		if(this->link_status==CONNECTION_RECEIVED)
		{
			std::string str="CONNECTED "+this->destination_call_sign+" "+this->my_call_sign+" "+ std::to_string(telecom_system->bandwidth)+"\r";
			tcp_socket_control.message->length=str.length();

			for(int i=0;i<tcp_socket_control.message->length;i++)
			{
				tcp_socket_control.message->buffer[i]=str[i];
			}
			tcp_socket_control.transmit();
			// Session established and announced (CONNECTED): no scanner-release is
			// owed; clear the latch so a later teardown emits DISCONNECTED only.
			pending_emitted=false;
		}

		link_status=CONNECTED;
		// Arm the authenticated stream ORIGIN generation at the RSP connect-accept
		// (data-flow-rx-fadecore-adoption.md). A freshly accepted session begins its
		// first user-data batch at wire bsi 0, so the offset-0 delivery gate now
		// admits ONLY bsi 0 as the legal stream origin on this session (plaintext and
		// the universal case). kx_stream_reanchor re-arms on an authenticated re-key;
		// reset_session_state disarms on a session boundary, so unconnected / direct
		// funnel-poke states keep legacy (unarmed, byte-identical) behavior.
		rsp_stream_origin_gen = 0;
		// T1c CONNECT-TIME CALIBRATION BOOTSTRAP (SPEC T1 s2): warm the learned turnaround timing
		// terms at the RSP connect-accept so the reverse-ACK scheduled pre-key is not inert on the
		// live path. Local-only (no wire frame, no added round-trip) and run AFTER the session is
		// accepted, so it can never delay or fail a connect; wire-byte-identical with the
		// MERCURY_T1_SCHEDKEY lever off. The peer-measured keyup refinement is the HELD wire exchange.
		t1_seed_calibration_defaults();
				settled_close_cache.valid = false;  // session-start disarm (never re-ACK a stale descriptor)
				responder_terminal_hold_timer.stop();
				responder_terminal_hold_timer.reset();
		if(connect_fast_active)
		{
			robust_enabled = connect_fast_fallback_robust;
			init_configuration = connect_fast_fallback_config;
			data_configuration = connect_fast_fallback_config;
			ack_configuration  = connect_fast_fallback_config;
			last_data_viable_config = session_floor_anchor(robust_enabled, init_configuration);
			connect_fast_active = false;
			connect_fast_timer.stop();
			connect_fast_timer.reset();
		}
#ifdef MERCURY_GUI_ENABLED
		if(passive_monitor)
		{
			// Monitor: don't use my_call_sign as responder (it's the monitor's own)
			// Keep "???" from START_CONNECTION, just log the event
			gui_push_monitor_event("[TEST_CONNECTION observed]", false);
		}
		else
		{
			gui_set_monitor_callsigns(this->destination_call_sign.c_str(),
			                          this->my_call_sign.c_str());
			{
				char buf[128];
				snprintf(buf, sizeof(buf), "[CONNECTED %s <-> %s]",
				         this->destination_call_sign.c_str(),
				         this->my_call_sign.c_str());
				gui_push_monitor_event(buf, false);
			}
		}
#endif
		if(passive_monitor)
		{
			// Monitor: skip ACK, go directly to RECEIVING
			messages_control.status = FREE;
			connection_status = RECEIVING;
			calculate_receiving_timeout();
			receiving_timer.start();
		}
		else
		{
			// Handshake echo (formerly v9 CAP_HANDSHAKE_ECHO, now unconditional).
			// Replace the legacy ACK pattern with an LDPC TEST_CONNECTION_ACK
			// frame carrying caps echo + CRC8. CMD validates before transitioning
			// out of CONNECTION_ACCEPTED. Mirrors the KEY_EXCHANGE_1 LDPC ACK
			// pattern below.
			messages_control.data[0] = TEST_CONNECTION_ACK;
			messages_control.data[1] = (char)peer_capability;   // echo CMD's caps
			messages_control.data[2] = (char)local_capability;  // RSP's own caps
			messages_control.data[3] = (char)CRC8_calc(
				(char*)&messages_control.data[1], 2);
			messages_control.length = 4;
			if(t1_peer_calibration_valid)
			{
				t1_calibration_encode(&messages_control.data[4], ptt_on_delay_ms,
					0, ptt_off_delay_ms);
				messages_control.length = 9;
			}
			connection_status = ACKNOWLEDGING_CONTROL;
			printf("[HANDSHAKE-ECHO] RSP queued TEST_CONNECTION_ACK: "
				"echoed_cap=0x%02X own_cap=0x%02X crc8=0x%02X\n",
				(unsigned char)peer_capability,
				(unsigned char)local_capability,
				(unsigned char)messages_control.data[3]);
			fflush(stdout);
			connect_ack_cache.valid = true;
			connect_ack_cache.echoed_cap = (uint8_t)peer_capability;
			connect_ack_cache.own_cap = (uint8_t)local_capability;
			connect_ack_cache.ssid = (uint8_t)callsign_get_ssid(my_call_sign);
			connect_ack_cache.replays = 0;
		}
		watchdog_timer.start();
		link_timer.start();


	}
	else if(link_status==CONNECTED && (code==KEY_EXCHANGE_2 || code==KEY_EXCHANGE_3))
	{
		// HYBRID ML-KEM KX chunk RX (RSP side, MLKEM_HYBRID_PLAN.md §5).
		// KEY_EXCHANGE_2 chunks carry the commander's 1184B encaps key. When the
		// full key reassembles, RSP encapsulates -> 1088B ciphertext, then streams
		// it back as KEY_EXCHANGE_3 chunks. (KEY_EXCHANGE_3 inbound is not
		// expected on the RSP — the RSP is the KX3 sender; treat as a stray.)
		if(passive_monitor)
		{
			messages_control.status = FREE;
			connection_status = RECEIVING;
			link_timer.start();
			watchdog_timer.start();
		}
		else if(code==KEY_EXCHANGE_2)
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
			                            kx_frame_len, KEY_EXCHANGE_2);
			// SIBLING-BUG FIX (data-flow-control-slot-lifecycle.md): do NOT FREE
			// the slot here. Every KX2 chunk MUST be LDPC-ACKed so the commander's
			// process_control_commander() KX2-advance path (arq_commander.cc:7184)
			// queues the NEXT chunk — KX2 is per-chunk stop-and-wait, exactly like
			// KX1/KX3. The ACK builder (process_messages_acknowledging_control)
			// only fires when messages_control.status==RECEIVED and frees the slot
			// itself after send_batch() (arq_responder.cc:1895). Freeing here left
			// status=FREE -> the ACK path spun "[ACK-CTRL] status=0" forever, the
			// commander never advanced, and only the stray chunk that happened to
			// align with a retransmit got through (live KX never completed). Stamp
			// data[0]=KEY_EXCHANGE_2 so the ACK builder routes it onto the OFDM
			// data_configuration (the config the commander listens on for this ACK).
			messages_control.data[0] = KEY_EXCHANGE_2;
			messages_control.length  = 1;     // bare confirmation ACK (no payload back)
			if(done == 1)
			{
				// Full encaps key reassembled — encapsulate to produce the
				// ciphertext + ML-KEM shared secret.
				if(cipher_suite.encapsulate_mlkem(kx_mlkem_pk, kx_mlkem_ct) != 0)
				{
					printf("[CRYPTO] FATAL: ML-KEM encapsulation failed (bad encaps key)\n");
					fflush(stdout);
					this->link_status = DROPPED;
					reset_session_state();
					return;
				}
				kx_mlkem_ct_ready = true;
				cipher_suite.set_kx_phase(KX_MLKEM_CT_SENT);
				// Derive the hybrid key NOW (RSP holds ct (it encapsulated) + pk
				// (reassembled) + both X25519 pubs from KX1). pk order is ALWAYS
				// commander-then-responder; on the RSP, peer = commander.
				const uint8_t* pk_rsp = cipher_suite.get_x25519_pubkey();
				const uint8_t* pk_cmd = cipher_suite.get_x25519_peer_pubkey();
				cipher_suite.derive_session_key(
					destination_call_sign.c_str(), my_call_sign.c_str(),
					(psk_hex[0] != '\0') ? (const uint8_t*)psk_hex : NULL,
					(psk_hex[0] != '\0') ? (int)strlen(psk_hex) : 0,
					true,                  // mlkem_done -> hybrid + pq_active
					kx_mlkem_ct, kx_mlkem_pk, pk_cmd, pk_rsp);
				cipher_suite.set_kx_phase(KX_HYBRID_DONE);
				printf("[CRYPTO] RSP encapsulated + hybrid key derived; streaming KX3 ciphertext\n");
				fflush(stdout);
				// Begin streaming the ciphertext chunks back to the commander.
				if(kx_begin_chunk_send(KEY_EXCHANGE_3) < 0)
				{
					printf("[CRYPTO] FATAL: KX3 send setup failed\n");
					fflush(stdout);
					this->link_status = DROPPED;
					reset_session_state();
					return;
				}
			}
			connection_status = ACKNOWLEDGING_CONTROL;
			link_timer.start();
			watchdog_timer.start();
		}
		else if(kx_tx_kind == KEY_EXCHANGE_3)
		{
			// KX3 REVERSE-PUMP ADVANCE (data-flow-control-slot-lifecycle.md §8.2(3)).
			// The CMD ACKed the KX3 chunk we just sent by sending this bare
			// KEY_EXCHANGE_3 control request. Advance to the next ciphertext chunk
			// and queue it — the exact mirror of the CMD KX2-advance
			// (arq_commander.cc:7184: kx_tx_next_index++; kx_send_next_chunk()) that
			// drives the RSP KX2 ACK. kx_send_next_chunk() (RESPONDER branch) encodes
			// the chunk into messages_control with status=RECEIVED (§8.2(1));
			// ACKNOWLEDGING_CONTROL then runs the ACK builder, which transmits it on
			// data_configuration and FREEs the slot. The CMD never sends a
			// KEY_EXCHANGE_3 for the FINAL chunk (it sends KEY_ACTIVATE instead), so
			// the all-sent (rc==0) arm is a defensive idle, not the normal terminator.
			kx_tx_next_index++;
			int more = kx_send_next_chunk();
			if(more == 1)
			{
				printf("[CRYPTO] RSP advancing KX3 -> chunk idx=%d/%d queued\n",
					kx_tx_next_index, kx_tx_count);
				fflush(stdout);
				connection_status = ACKNOWLEDGING_CONTROL;
			}
			else
			{
				// All KX3 chunks sent (rc==0) or encode error (rc<0). Nothing more
				// to ACK back — idle the slot and await KEY_ACTIVATE.
				messages_control.status = FREE;
				connection_status = RECEIVING;
			}
			link_timer.start();
			watchdog_timer.start();
		}
		else
		{
			// Stray KEY_EXCHANGE_3 inbound on the RSP — ignore, free the slot.
			messages_control.status = FREE;
			connection_status = RECEIVING;
			link_timer.start();
			watchdog_timer.start();
		}
	}
	else if(link_status==CONNECTED && (code==KEY_EXCHANGE_1 || code==KEY_ACTIVATE))
	{
		if(passive_monitor)
		{
			// Monitor: can't participate in key exchange, note encryption
			printf("[MONITOR] Encryption negotiation observed (code=%d) — cannot decode encrypted data\n", code);
			fflush(stdout);
#ifdef MERCURY_GUI_ENABLED
			if(code==KEY_EXCHANGE_1)
				gui_push_monitor_event("[KEY_EXCHANGE observed — encryption negotiating]", false);
			else
				gui_push_monitor_event("[ENCRYPTION ACTIVE — monitor cannot decode encrypted payload]", false);
#endif
			messages_control.status = FREE;
			connection_status = RECEIVING;
			link_timer.start();
			watchdog_timer.start();
		}
		else if(code==KEY_EXCHANGE_1)
		{
			// Commander's X25519 public key (32 bytes at data[1..32])
			printf("[CRYPTO] Received KEY_EXCHANGE_1 (X25519 pubkey from commander)\n");
			fflush(stdout);

			// Generate our own keypair
			uint8_t our_pubkey[X25519_KEY_SIZE];
			if(cipher_suite.generate_x25519_keypair(our_pubkey) != 0)
			{
				printf("[CRYPTO] FATAL: X25519 keypair generation failed (RNG)\n");
				fflush(stdout);
				this->link_status = DROPPED;
				reset_session_state();
				return;
			}

			// Compute shared secret from commander's pubkey. Length-checked: the
			// control-frame RX copies a FIXED width into messages_control.data[]
			// (arq_responder.cc:846-851), so the available pubkey bytes after the
			// 1-byte type field = that width - 1. Reject a too-short frame rather
			// than run X25519 over stale tail bytes. (data-flow-aead-nonce.md §KX.)
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
			// ML-KEM upgrade will be added in follow-up (requires data-channel transport)
			cipher_suite.derive_session_key(
				destination_call_sign.c_str(), my_call_sign.c_str(),
				(psk_hex[0] != '\0') ? (const uint8_t*)psk_hex : NULL,
				(psk_hex[0] != '\0') ? (int)strlen(psk_hex) : 0,
				false);  // mlkem_done=false (X25519-only for now)

#ifdef MERCURY_GUI_ENABLED
			// Display-only: X25519 shared computed -> advance the KX phase line.
			gui_set_kx_progress(cipher_suite.get_kx_phase(), 0, 0);
#endif

			// Put our pubkey + confirmation tag in the ACK data
			messages_control.data[0] = KEY_EXCHANGE_1;
			memcpy(&messages_control.data[1], our_pubkey, X25519_KEY_SIZE);
			// Append 8-byte key confirmation tag for early PSK verification
			uint8_t confirm_tag[8];
			cipher_suite.compute_key_confirmation(confirm_tag);
			memcpy(&messages_control.data[1 + X25519_KEY_SIZE], confirm_tag, 8);
			messages_control.length = 1 + X25519_KEY_SIZE + 8;

			printf("[CRYPTO] X25519 shared secret computed, session key derived\n");
			printf("[CRYPTO] Setting ACKNOWLEDGING_CONTROL, msg_status=%d\n",
				messages_control.status);
			fflush(stdout);

			connection_status = ACKNOWLEDGING_CONTROL;
			link_timer.start();
			watchdog_timer.start();
		}
		else if(code==KEY_ACTIVATE)
		{
			// Verify key confirmation tag (PSK mismatch detection)
			// Note: messages_control.length is always 1 (hardcoded in receive path),
			// but data[1..8] is populated from the full LDPC frame decode.
			uint8_t our_tag[8];
			cipher_suite.compute_key_confirmation(our_tag);
			const uint8_t* peer_tag = (const uint8_t*)&messages_control.data[1];

			if(memcmp(our_tag, peer_tag, 8) == 0)
			{
				printf("[CRYPTO] KEY_ACTIVATE confirmed — key confirmation matches\n");
				fflush(stdout);
				cipher_suite.activate();
				l1_apply_reset_event(mercury::L1ResetEvent::CRYPTO_REKEY);
				// AEAD nonce sequence state: fresh per session+direction.
				// (data-flow-aead-nonce.md §init)
				tx_nonce_epoch = 0;
				tx_nonce_last_bsi = -1;
				rx_nonce_epoch = 0;
				rx_nonce_last_bsi = -1;
				tx_nonce_gen = 0;
				rx_nonce_gen = 0;
				tx_nonce_sealed_high_water = UINT64_MAX;
				rx_nonce_adopted_once = false;
				decrypt_delivered_bsi = -1;
				rx_stream_emitted_bsi_hw = -1;   // INV-DEDUP: KEY_ACTIVATE re-establishes the session; reset the emit high-water
				consecutive_auth_failures = 0;

				// KX-as-data (data-flow-hybrid-kex.md Â§3.4/Â§7): re-anchor the Option-W cursors +
				// de-dup high-water + wire batch_seq_id to a fresh bsi=0 baseline (mirror of the
				// commander) so this receiver re-adopts the first USER batch at wire bsi 0.
				// Env-gated OFF -> never called on the legacy pump path.
				if(kx_as_data_path()) kx_stream_reanchor();

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
				g_gui_state.encryption_active.store(true);
				// Display-only: PQ status, KX phase, and session fingerprint.
				g_gui_state.encryption_pq_active.store(cipher_suite.is_pq_upgraded());
				gui_set_kx_progress(cipher_suite.get_kx_phase(), 0, 0);
				{
					char fp_hex[24];
					cipher_suite.get_fingerprint_hex(fp_hex, sizeof(fp_hex));
					gui_set_enc_fingerprint(fp_hex);
				}
				gui_push_monitor_event("[ENCRYPTION ACTIVE: X25519 + ChaCha20-Poly1305]", false);
#endif

				connection_status = ACKNOWLEDGING_CONTROL;
				link_timer.start();
				watchdog_timer.start();
			}
			else
			{
				printf("[CRYPTO] KEY_ACTIVATE FAILED — key confirmation mismatch (PSK wrong?)\n");
				printf("[CRYPTO]   our_tag:  %02x%02x%02x%02x%02x%02x%02x%02x\n",
					our_tag[0], our_tag[1], our_tag[2], our_tag[3],
					our_tag[4], our_tag[5], our_tag[6], our_tag[7]);
				printf("[CRYPTO]   peer_tag: %02x%02x%02x%02x%02x%02x%02x%02x\n",
					peer_tag[0], peer_tag[1], peer_tag[2], peer_tag[3],
					peer_tag[4], peer_tag[5], peer_tag[6], peer_tag[7]);
				fflush(stdout);

				// Report failure on control port
				const char* err_msg = "ENCRYPTION FAILURE PSK MISMATCH\r";
				int elen = (int)strlen(err_msg);
				for(int e=0; e<elen; e++)
					tcp_socket_control.message->buffer[e] = err_msg[e];
				tcp_socket_control.message->length = elen;
				tcp_socket_control.transmit();

#ifdef MERCURY_GUI_ENABLED
				g_gui_state.encryption_psk_mismatch.store(true);
				gui_push_monitor_event("[PSK MISMATCH — pre-shared key does not match, disconnecting]", false);
#endif
				this->link_status = DROPPED;
				reset_session_state();
			}
		}
	}
	else if(link_status==CONNECTED && (code==SET_CONFIG || code==BLOCK_END || code==FILE_END_ || code==SWITCH_ROLE || code==REPEAT_LAST_ACK || code==SWITCH_BANDWIDTH))
	{
		if(code==SWITCH_BANDWIDTH)
		{
			printf("[BW-NEG] Received SWITCH_BANDWIDTH (target=%d) my_mode=%d\n",
				(int)(unsigned char)messages_control.data[1], bandwidth_mode);
			fflush(stdout);
			if(passive_monitor)
			{
				// Monitor: immediately switch bandwidth, no ACK needed
				printf("[MONITOR] SWITCH_BANDWIDTH observed — switching immediately\n");
				fflush(stdout);
#ifdef MERCURY_GUI_ENABLED
				gui_push_monitor_event("[SWITCH_BANDWIDTH: NB -> WB]", false);
#endif
				switch_narrowband_mode(NO);
				// Parallel decoders were initialized for NB — recreate for WB
				reinit_monitor_decoders();
				messages_control.status = FREE;
				batch_rx_frame_count = 0;
				connection_status = RECEIVING;
				link_timer.start();
				watchdog_timer.start();
			}
			else if(bandwidth_mode == BW_NB_ONLY)
			{
				// Reject: don't ACK — commander will timeout and stay NB.
				// MFSK ACK patterns carry no data, so we can't signal rejection
				// through ACK content. Silence = rejection.
				printf("[BW-NEG] Rejecting WB upgrade (nb_only mode), not ACKing\n");
				fflush(stdout);
				wb_upgrade_pending = false;
				messages_control.status = FREE;  // Discard so next message can be received
				batch_rx_frame_count = 0;
				connection_status = RECEIVING;
				link_timer.start();
				watchdog_timer.start();
			}
			else
			{
				// Accept: standard ACK → deferred WB switch after ACK sent
				wb_upgrade_pending = true;
				// CONNECT-SEED FUSION: a length>=3 SWITCH_BANDWIDTH carries a seed config in data[2].
				// Stage it so the deferred WB switch (after this ACK) loads it directly, collapsing
				// the separate SET_CONFIG cross. Defeated peers send length 2 -> no seed (BASE).
				connect_fuse_seed_rx = CONFIG_NONE;
				// data[2] is buffer-copied on RX regardless of the recovered length field (SET_CONFIG
				// reads data[2] the same way). A FIX-arm CMD always sets data[2] (seed or 0xFF sentinel).
				if(!connect_fuse_defeat)
				{
					int s_seed = (int)(unsigned char)messages_control.data[2];
					if(config_negotiable_to(s_seed, is_robust3_config(init_configuration) || is_robust3_config(connect_fast_fallback_config)))
					{
						connect_fuse_seed_rx = s_seed;
						printf("[BW-NEG] FUSE: SWITCH_BANDWIDTH carries connect-seed config %d\n", s_seed);
						fflush(stdout);
					}
				}
				connection_status = ACKNOWLEDGING_CONTROL;
				link_timer.start();
				watchdog_timer.start();
			}
		}
		else if(code==SET_CONFIG)
		{
			// Asymmetric gearshift: extract forward and reverse configs
			// data[0]=SET_CONFIG, data[1]=forward, data[2]=reverse
			// Always 3-byte payload from our fork; data[2] is always present
			// in messages_rx_buffer (full buffer copied at arq_common.cc:2437)
			forward_configuration = messages_control.data[1];
			reverse_configuration = messages_control.data[2];

			printf("[GEARSHIFT] Received SET_CONFIG: forward=%d reverse=%d\n",
				forward_configuration, reverse_configuration);

#ifdef MERCURY_GUI_ENABLED
			if(passive_monitor)
			{
				char buf[64];
				snprintf(buf, sizeof(buf), "[GEARSHIFT -> CONFIG_%d]", forward_configuration);
				gui_push_monitor_event(buf, false);
			}
#endif

			if(passive_monitor)
			{
				// Monitor: load config immediately (no ACK to send first)
				data_configuration = forward_configuration;
				if(forward_configuration != current_configuration &&
					(config_negotiable_to(forward_configuration, is_robust3_config(init_configuration) || is_robust3_config(connect_fast_fallback_config))))
				{
					load_configuration(data_configuration, PHYSICAL_LAYER_ONLY, YES);

					// Flush capture buffer: stale preambles from old config
					// would cause Schmidl-Cox false detections (same preamble
					// structure across all OFDM configs). Without flush, the
					// monitor keeps re-detecting the old preamble and failing
					// LDPC decode because data symbols are wrong config.
					int buf_samples = telecom_system->data_container.Nofdm
						* telecom_system->data_container.buffer_Nsymb
						* telecom_system->data_container.interpolation_rate;
					MUTEX_LOCK(&capture_prep_mutex);
					capture_reset_samples();
					memset(telecom_system->data_container.passband_delayed_data, 0,
						2 * buf_samples * sizeof(double));
					telecom_system->data_container.ring_write_index = 0;
					telecom_system->receive_stats.ofdm_search_raw = 0;
					telecom_system->receive_stats.ofdm_batch_active = false;
					telecom_system->receive_stats.delay_of_last_decoded_message = -1;
					MUTEX_UNLOCK(&capture_prep_mutex);
					printf("[MONITOR] Flushed capture buffer for config switch to CONFIG_%d\n",
						forward_configuration);
					fflush(stdout);
				}
				messages_control.status = FREE;
				batch_rx_frame_count = 0;
				connection_status = RECEIVING;
				monitor_consec_ofdm_fail = 0;  // Fresh start on new config

				// Monitor must wait through: real responder's ACK (~750ms) +
				// commander config load + processing (~500ms) + data batch TX.
				// Use generous timeout to avoid premature expiry.
				int monitor_timeout = data_batch_size * message_transmission_time_ms
					+ time_left_to_send_last_frame + ptt_on_delay_ms + 5000;
				set_receiving_timeout(monitor_timeout);
				receiving_timer.start();

				// Set frames_to_read to 2x frame size. The frame could start
				// anywhere in the buffer, so we need enough room for a full
				// frame even in the worst-case alignment.
				int rx_frame = telecom_system->data_container.preamble_nSymb
					+ telecom_system->data_container.Nsymb;
				telecom_system->data_container.frames_to_read = rx_frame * 2;
				telecom_system->data_container.nUnder_processing_events = 0;
			}
			else if(forward_configuration != current_configuration &&
				(config_negotiable_to(forward_configuration, is_robust3_config(init_configuration) || is_robust3_config(connect_fast_fallback_config))))
			{
				// Don't load_configuration here — ack_configuration must stay on
				// the OLD config so the ACK reaches the commander (still on old config).
				// Just save data_configuration; acknowledging_control will call
				// load_configuration(data_configuration, ...) after the ACK is sent.
				data_configuration = forward_configuration;

				// D3.1 (data-integrity, fix #2): a REAL config change mid-transfer
				// (forward != current) is a DEMOTE/PROMOTE that may strand an
				// incomplete batch in the RSP window (the FIX-4 carve demote, the
				// FIX-9 D3 demote, the FIX-3 probe-skip, a plain gearshift step-down).
				// UNLIKE a BREAK these SET_CONFIG-only paths do NOT reset the RSP bsi
				// window, so the RSP keeps cur>=0 and PREV-BUMPs past the incomplete
				// batch -> silent concat (FIX9_D3_DESIGN section 7). RE-BASELINE
				// cur=prev=-1 SYMMETRIC with the BREAK self-heal (arq_responder.cc:474)
				// + drop the in-flight prev partial storage, so the NEXT data frame
				// re-adopts through the FIX-8 gap-gate (sack_v2_readopt_has_gap): a
				// CONTIGUOUS re-adopt (the normal climb, CMD keeps numbering) accepts
				// cleanly; a NON-CONTIGUOUS one (stranded incomplete batch skipped)
				// aborts LOUDLY. Gated on a REAL config change, so no-op/same-config
				// SET_CONFIGs do not perturb the window. The delivery-time gate (#1) is
				// the inviolable backstop if any frame still slips past this re-baseline.
				// See bigblock_p3_hw/_d31_fade/D31_INORDER_DESIGN.md section 3.
				bool _ss_do_rebase = (sack_v2_enabled && rsp_current_expected_batch_seq_id >= 0);
				rsp_inband_demote_rebase();   // INV-DEDUP: REAL production rebase (preserves last_delivered + emit high-water)
				if(_ss_do_rebase) rsp_rebase_seam_armed = true;   // SS: arm rebase-seam fail-closed (silent-corruption P0)
			}

			if(!passive_monitor)
			{
				connection_status=ACKNOWLEDGING_CONTROL;
				link_timer.start();
				watchdog_timer.start();
				gear_shift_timer.start();
			}
		}
		// BLOCK_END eliminated — data flush now happens after pattern ACK in
		// process_messages_acknowledging_data(). Commander never sends BLOCK_END.
		else if(code==FILE_END_)
		{
			connection_status=ACKNOWLEDGING_CONTROL;
			printf("end of file\n");
			// AEAD nonce source (data-flow-aead-nonce.md §5): end-of-session flush
			// of any residual ACKED batch in messages_rx[]. cur is NOT advanced at
			// this site, so an undelivered residual batch still carries bsi ==
			// rsp_current_expected_batch_seq_id. (Normally the in-order BATCH-DONE
			// path already delivered + FREE'd the last batch, so this is a no-op
			// with assembled_size==0 and decrypt is skipped.)
			decrypt_delivered_bsi = rsp_current_expected_batch_seq_id;
			copy_data_to_buffer();
			batch_data_delivered = false;
			messages_last_ack_bu.type=NONE;
			link_timer.start();
			watchdog_timer.start();
		}
		else if(code==SWITCH_ROLE)
		{
			printf("switch role\n");
			// AEAD nonce source (data-flow-aead-nonce.md §5): SWITCH_ROLE flush —
			// same residual-batch semantics as FILE_END_ (cur not advanced here).
			decrypt_delivered_bsi = rsp_current_expected_batch_seq_id;
			copy_data_to_buffer();
			batch_data_delivered = false;

			if(passive_monitor)
			{
				// Monitor: don't switch to commander, just note direction swap
				printf("[MONITOR] SWITCH_ROLE observed — direction swap\n");
				fflush(stdout);
#ifdef MERCURY_GUI_ENABLED
				gui_push_monitor_event("[SWITCH_ROLE]", false);
				// Swap callsign labels (A becomes B)
				{
					GuiLockGuard lock(g_gui_state.monitor_mutex);
					std::string tmp = g_gui_state.monitor_callsign_a;
					g_gui_state.monitor_callsign_a = g_gui_state.monitor_callsign_b;
					g_gui_state.monitor_callsign_b = tmp;
				}
#endif
				messages_control.status = FREE;
				batch_rx_frame_count = 0;
				connection_status = RECEIVING;
				// Generous timeout: wait through real responder's ACK + commander
				// switch role processing + next batch TX start
				{
					int monitor_timeout = data_batch_size * message_transmission_time_ms
						+ time_left_to_send_last_frame + ptt_on_delay_ms + 5000;
					set_receiving_timeout(monitor_timeout);
				}
				receiving_timer.start();
				// Set ftr so OFDM capture accumulates enough audio
				{
					int rx_frame = telecom_system->data_container.preamble_nSymb
						+ telecom_system->data_container.Nsymb;
					telecom_system->data_container.frames_to_read = rx_frame * 2;
					telecom_system->data_container.nUnder_processing_events = 0;
				}
				link_timer.start();
				watchdog_timer.start();
			}
			else
			{
			connection_status=ACKNOWLEDGING_CONTROL;
			link_timer.start();
			watchdog_timer.start();
			// Received data test code
//			char data,data2;
//			int error=NO;
//			srand(5);
//			int nRec= fifo_buffer_rx.get_size()-fifo_buffer_rx.get_free_size();
//			std::cout<<"nRec= "<<nRec<<std::endl;
//			for(int i=0;i<nRec;i++)
//			{
//				fifo_buffer_rx.pop(&data, 1);
//				data2=(char)(rand()%0xff);
//				if(data!=data2)
//				{
//					std::cout<<"error @" <<i<<" data="<<(int)data<<" data2="<<(int)data2<<std::endl;
//					error=YES;
//				}
//			}
//			if(error==YES)
//			{
//				exit(0);
//			}
//			else
//			{
//				std::cout<<"all is good"<<std::endl;
//				exit(0);
//			}
		} // end else (non-monitor SWITCH_ROLE)
		}
		else if(code==REPEAT_LAST_ACK)
		{
			repeating_last_ack=YES;
			connection_status=ACKNOWLEDGING_DATA;
		}
	}
	else if(link_status==CONNECTED && code==SET_LINK_PARAMS)
	{
		// SACK Design A Step 10 — RSP-side SET_LINK_PARAMS handler.
		// R4 (LINK-PARAMS quiesce gate): DROPPED is NO LONGER accepted here. A dead link
		// must not HALF-ABSORB new geometry: accepting SET_LINK_PARAMS while DROPPED let a
		// zombie session adopt a new batch size instead of forcing reconnection, and the
		// stated "faster recovery" rationale was the zombie's accomplice (a dropped responder
		// that half-applies params keeps grinding rather than resetting). The CMD-side R4
		// quiesce gate now only renegotiates at a CLEAN boundary — when the link is delivering
		// and the RSP reads CONNECTED — so a DROPPED read at SET_LINK_PARAMS time means the
		// link is genuinely down; ignoring the op (CMD times out + retransmits, exactly as if
		// the OFDM control frame had been lost) is the correct, non-corrupting response.
		//
		// Wire format UNCHANGED — this only narrows the STATE in which the RSP applies the op.
		//
		// Wire format (§4.4): [code, batch_size_u8, sack_mode_u8, CRC8]
		// CRC8 covers data[1..2] only (the standard 3-byte msg header has its
		// own LDPC + CRC16 integrity; that's already validated by the time
		// we reach this code path).
		//
		// On CRC-pass: apply the new batch size via set_data_batch_size()
		// and log [RSP-LINK-PARAMS] APPLIED. Step 11 will also pick up
		// sack_mode; Step 10 only consumes batch_size.
		// On CRC-fail: ignore + log + bump fail counter (§4.3.4 invariant 3
		// "no silent corruption" — CMD will time out and retransmit, exactly
		// as if the OFDM control frame had been lost on the air).
		//
		// Note: messages_control.length is hardcoded to 1 on the RX path
		// (arq_responder.cc:267) for all incoming control frames — the
		// authoritative payload length is in messages_rx_buffer at receive
		// time but is not preserved across the dispatch boundary. SET_CONFIG
		// works around this by reading data[1] and data[2] unconditionally
		// (knowing the format is fixed). SET_LINK_PARAMS has a fixed 4-byte
		// payload — so we likewise read data[0..3] unconditionally, gating
		// the CRC check on sack_v2_enabled (the only path that can produce
		// these wire bytes).
		if(sack_v2_enabled)
		{
			int new_batch_u8   = (unsigned char)messages_control.data[1];
			int new_sack_u8    = (unsigned char)messages_control.data[2];
			unsigned char rx_crc = (unsigned char)messages_control.data[3];
			unsigned char computed_crc = CRC8_calc(
				(char*)&messages_control.data[1], 2);
			if(rx_crc != computed_crc)
			{
				rsp_set_link_params_crc_fail_count++;
				printf("[RSP-LINK-PARAMS-CRC-FAIL] rx_crc=0x%02x computed=0x%02x "
					"batch=%d sack_mode=%d (discarding; CMD will retransmit) "
					"fail_count=%lld\n",
					rx_crc, computed_crc, new_batch_u8, new_sack_u8,
					rsp_set_link_params_crc_fail_count);
				fflush(stdout);
				// Discard frame; do NOT ACK. CMD's existing control-frame
				// timeout will fire and resend.
				messages_control.status = FREE;
			}
			else
			{
				// Clamp to the AXIS2 range — defensive. RSP trusts CMD's
				// decision but rejects out-of-range values.
				int target = new_batch_u8;
				if(target < AXIS2_BATCH_FLOOR) target = AXIS2_BATCH_FLOOR;
				if(target > axis2_batch_ceiling())
					target = axis2_batch_ceiling();

				int old_batch = data_batch_size;
				// from_link_params=true — the ONLY production call site with wire
				// provenance (data-flow audit grep contract: keep it unique; the
				// directed regression also drives the flag to prove the robust
				// clamp ignores it). The CRC8-passed op
				// bypasses the climb-confirm clamp at the sole setter: the CMD only
				// emits above-pin values when held-rung eligible, and the RSP carries
				// no eligibility counter of its own (no Axis-2 caller), so without
				// the provenance leg every applied grow would re-clamp to 10 here =
				// a permanent CMD/RSP batch divergence. Robust range clamp and the
				// [AXIS2_BATCH_FLOOR, axis2_batch_ceiling()] defensive clamp above
				// still apply.
				set_data_batch_size(target, /*from_link_params=*/true);
				recalculate_ack_timeout_for_batch();

				// SACK Design A Step 11 — apply sack_mode from CMD.
				// Validate the byte is in {0=OFF, 1=ON, 2=PROBE}; anything
				// else falls back to ON defensively (§4.3.4 invariant 3:
				// no silent corruption — log + clamp). RSP-side state mirrors
				// the CMD's authoritative decision (§3.8 initiator-controls-flow).
				int old_mode = axis3_sack_mode;
				int new_mode = new_sack_u8;
				if(new_mode < 0 || new_mode > 2)
				{
					printf("[RSP-LINK-PARAMS-WARN] sack_mode=%d out of {0,1,2} — "
						"defaulting to ON\n", new_sack_u8);
					new_mode = SACK_MODE_ON;
				}
				axis3_sack_mode = new_mode;
				rsp_set_link_params_rx_count++;
				printf("[RSP-LINK-PARAMS] APPLIED batch %d -> %d sack_mode=%d "
					"(prev sack_mode=%d) (crc8=0x%02x rx_count=%lld)\n",
					old_batch, data_batch_size, new_mode, old_mode,
					rx_crc, rsp_set_link_params_rx_count);
				fflush(stdout);
				// ACK the control frame via the normal control-ACK path —
				// the existing process_messages_acknowledging_control() flow
				// will TX a control ACK once we mark it RECEIVED.
				connection_status = ACKNOWLEDGING_CONTROL;
				link_timer.start();
				watchdog_timer.start();
			}
		}
		else
		{
			// v2 not negotiated — should not happen on a well-behaved peer
			// (a v1-only peer never emits 0x43). Log and free.
			printf("[RSP-LINK-PARAMS] received len=%d v2=%d — IGNORED (v2 not negotiated)\n",
				messages_control.length, (int)sack_v2_enabled);
			fflush(stdout);
			messages_control.status = FREE;
		}
	}
	else if((link_status==CONNECTED || link_status==DROPPED) && code==ROBUST_DWELL_BATCH_OP)
	{
		// FIX-A — RSP-side ROBUST_DWELL_BATCH_OP handler (data-flow-robust-tier-arq-batch.md
		// §5.2). The CMD decides the robust dwell batch (it owns the climb state); the
		// RSP simply MIRRORS the value so both sides hold the SAME data_batch_size (the
		// 4-wire-failure all-ones-target symmetry invariant). Applied straight through
		// set_data_batch_size() — whose RELAXED robust chokepoint clamps only to
		// [1..ROBUST_DWELL_BATCH_MAX]. There is DELIBERATELY no [AXIS2_BATCH_FLOOR,CEIL]
		// clamp here (that is the SET_LINK_PARAMS / Axis-2 OFDM contract; applying it
		// would force a 4-8 robust batch UP to 10 and re-create the Bug-3 mismatch —
		// OR-2 / L4).
		//
		// Wire format: [code, batch_u8, CRC8]. CRC8 covers data[1] only (the standard
		// 3-byte msg header is already LDPC+CRC16 protected). messages_control.length
		// is hardcoded to 1 on the RX path (arq_responder.cc:267) for control frames, so
		// (like SET_CONFIG / SET_LINK_PARAMS) we read the fixed-format payload bytes
		// unconditionally, gating the CRC on sack_v2_enabled (the only path that emits
		// these bytes).
		if(sack_v2_enabled)
		{
			int new_batch_u8 = (unsigned char)messages_control.data[1];
			unsigned char rx_crc = (unsigned char)messages_control.data[2];
			unsigned char computed_crc = CRC8_calc(
				(char*)&messages_control.data[1], 1);
			if(rx_crc != computed_crc)
			{
				// §4.3.4 invariant 3 "no silent corruption": discard, do NOT ACK; the
				// CMD's control-frame timeout fires and resends, exactly as if the OFDM
				// control frame had been lost on the air. The CMD's robust dwell batch
				// is unchanged for the RSP until a clean frame applies — and the CMD
				// applies locally only after add_message_control, so a lost op leaves
				// CMD ahead by one; the EOB bit-7 self-correct + the CMD retry recover.
				printf("[RSP-ROBUST-DWELL-CRC-FAIL] rx_crc=0x%02x computed=0x%02x "
					"batch=%d (discarding; CMD will retransmit)\n",
					rx_crc, computed_crc, new_batch_u8);
				fflush(stdout);
				messages_control.status = FREE;
			}
			else
			{
				int old_batch = data_batch_size;
				// set_data_batch_size() re-validates the [1..ROBUST_DWELL_BATCH_MAX]
				// range at the relaxed chokepoint and recomputes the ACK timeout (L3).
				set_data_batch_size(new_batch_u8);
				robust_dwell_batch_active = (data_batch_size > 1);
				printf("[RSP-ROBUST-DWELL] APPLIED batch %d -> %d (rx=%d crc8=0x%02x) "
					"config=%d\n",
					old_batch, data_batch_size, new_batch_u8, rx_crc,
					current_configuration);
				fflush(stdout);
				// ACK the control frame via the normal control-ACK path. FIX 4
				// (data-flow-zombie-amplifier.md §4): the batch value was MIRRORED
				// above via set_data_batch_size (4-wire symmetry, data-flow-robust-
				// tier-arq-batch.md §5.2) — that must always run. But on a DROPPED read
				// do NOT restart the link/watchdog keep-alive: a dead link must be
				// allowed to reach link_timeout (arq_common.cc:6043) and die, not
				// prolong the zombie. The control-ACK still airs; only the timer restart
				// is gated. MERCURY_ROBUST_DWELL_KEEPALIVE_DEFEAT=1 = fail-before.
				connection_status = ACKNOWLEDGING_CONTROL;
				if(robust_dwell_keepalive_ok(link_status))
				{
					link_timer.start();
					watchdog_timer.start();
				}
				else
				{
					printf("[RSP-ROBUST-DWELL] link DROPPED: batch mirrored + ACKed, "
						"keep-alive SUPPRESSED (zombie must expire at link_timeout)\n");
					fflush(stdout);
				}
			}
		}
		else
		{
			printf("[RSP-ROBUST-DWELL] received v2=%d — IGNORED (v2 not negotiated)\n",
				(int)sack_v2_enabled);
			fflush(stdout);
			messages_control.status = FREE;
		}
	}
	else
	{
		if(code==CLOSE_CONNECTION)
		{
				// Terminal-settlement re-ACK (idempotent settlement; mirror of connect_ack_cache). A
				// duplicate CLOSE+EOT matching the cached settled descriptor means our reverse CLOSE-ACK was
				// lost and the CMD is re-soliciting. Replay the terminal ACK, bounded by
				// TERMINAL_EOT_RETRIES and the active hold timer, WITHOUT
				// re-running teardown / EOT-verify (rx_stream_delivered is 0 post-reset) and WITHOUT taking
				// capture-countdown ownership or re-arming the origin gate. Requires the EOT payload present.
				// This branch owns the complete post-settlement lifetime: the valid cache
				// identifies the active/just-settled session, and LISTENING identifies a
				// frame decoded after bounded hold expiry (when the cache is invalidated).
				// Keep a first CLOSE received while DROPPED on the ordinary teardown path.
				if(link_status != CONNECTED
				   && (settled_close_cache.valid || link_status == LISTENING))
				{
					unsigned char dup_crc8 = (unsigned char)CRC8_calc(
						(char*)&messages_control.data[1], W_EOT_PAYLOAD_BYTES - 1);
					bool dup_eot_present =
						((unsigned char)messages_control.data[W_EOT_PAYLOAD_BYTES] == dup_crc8);
					uint64_t dup_committed = 0;
					uint32_t dup_crc = 0;
					if(dup_eot_present)
					{
						for(int i=0;i<8;i++)
							dup_committed |= ((uint64_t)(unsigned char)messages_control.data[1+i]) << (8*i);
						for(int i=0;i<4;i++)
							dup_crc |= ((uint32_t)(unsigned char)messages_control.data[9+i]) << (8*i);
						if(settled_reack_window_open()
						   && settled_close_duplicate_matches(dup_committed, dup_crc))
						{
							settled_close_cache.replays++;
							rsp_terminal_reack_replays++;
							printf("[RSP-EOT-REACK] duplicate CLOSE -> cached terminal ACK replay=%d/%d "
								"committed=%llu crc=0x%08x\n",
								settled_close_cache.replays, (int)TERMINAL_EOT_RETRIES,
								(unsigned long long)dup_committed, dup_crc);
							fflush(stdout);
							// Re-ACK via the existing control-ACK builder without re-running teardown:
							// keep messages_control RECEIVED and route to ACKNOWLEDGING_CONTROL (the SAME
							// path that sent the first reverse CLOSE-ACK). No reset_session_state, no
							// frames_to_read write, no rsp_stream_origin_gen change.
							connection_status = ACKNOWLEDGING_CONTROL;
							return;
						}
					}
					// Once a session is settled, a malformed, mismatched, passive,
					// or replay-budget-exhausted CLOSE must never fall through and
					// re-run EOT verification against the reset byte/CRC cursors.
					printf("[RSP-EOT-REACK] rejected post-settlement CLOSE "
					       "(present=%d match=%d replays=%d/%d passive=%d); mailbox=FREE\n",
						dup_eot_present ? 1 : 0,
						dup_eot_present && settled_close_duplicate_matches(dup_committed, dup_crc) ? 1 : 0,
						settled_close_cache.replays, (int)TERMINAL_EOT_RETRIES,
						passive_monitor ? 1 : 0);
					fflush(stdout);
					messages_control.status = FREE;
					return;
				}

			// Reconnect-continuity fail-closed F2 (data-flow-reconnect-continuity.md 5b): a PROVEN-CLEAN
			// end-of-transfer makes the persistent app socket a legitimate message boundary. Track it
			// here and clear the app-delivered high-water AFTER reset_session_state() re-snapshots it
			// (below), so a back-to-back new transfer on the same socket is byte-identical. An aborted /
			// short / absent-EOT close leaves the high-water set (fail-closed default).
			bool clean_eot_verified = false;
			// Option W STEP 3 (data-flow-stream-offset.md §8.6) — the EOT end-to-end LAST-BATCH
			// tail-drop / final-content reconciliation. MUST run BEFORE reset_session_state()
			// zeroes rx_stream_delivered / rx_stream_crc. The CLOSE frame carries the SENDER's
			// final total_committed_bytes + running stream CRC-32 (add_message_control appended
			// them; the RX msg-decode populates data[1..] even though length is hardcoded to 1 —
			// same as the KEY_ACTIVATE tag). A crc8 over the 12 payload bytes distinguishes a real
			// EOT from a short/legacy/garbled CLOSE ⇒ absent-EOT is a SAFE NO-OP (never a false
			// teardown). This is the ONLY W check that sees a truncated FINAL batch (the per-batch
			// stamp gates are one batch behind) — and, unlike them, it covers robust/cfg0 too
			// (both accumulators fold at config-independent funnels). MERCURY_W_EOT_DEFEAT=1 =
			// fail-before (the short final delivery is silently declared complete).
			{
				unsigned char eot_crc8 = (unsigned char)CRC8_calc(
					(char*)&messages_control.data[1], W_EOT_PAYLOAD_BYTES - 1);
				bool eot_present = ((unsigned char)messages_control.data[W_EOT_PAYLOAD_BYTES] == eot_crc8);
				bool eot_defeat = false;
				{ const char* e = std::getenv("MERCURY_W_EOT_DEFEAT");
				  if(e && *e && atoi(e)!=0) eot_defeat = true; }
				if(eot_present && !eot_defeat)
				{
					uint64_t peer_committed = 0;
					for(int i=0;i<8;i++)
						peer_committed |= ((uint64_t)(unsigned char)messages_control.data[1+i]) << (8*i);
					uint32_t peer_crc = 0;
					for(int i=0;i<4;i++)
						peer_crc |= ((uint32_t)(unsigned char)messages_control.data[9+i]) << (8*i);
					if(w_eot_mismatch(peer_committed, peer_crc))
					{
						printf("[RSP-V2-EOT-SHORT] LOUD: end-of-transfer shortfall/corruption — "
							"delivered=%llu committed=%llu (delta=%lld) rx_crc=0x%08x tx_crc=0x%08x. "
							"The FINAL batch was truncated/corrupted and has NO next-batch backstop; "
							"the transfer is NOT complete (never a silent short/wrong final delivery).\n",
							(unsigned long long)rx_stream_delivered,
							(unsigned long long)peer_committed,
							(long long)((int64_t)rx_stream_delivered - (int64_t)peer_committed),
							rx_stream_crc, peer_crc);
						fflush(stdout);
#ifdef MERCURY_GUI_ENABLED
						gui_push_monitor_event("[TRANSFER INCOMPLETE — EOT byte/CRC mismatch]", false);
#endif
					}
					else
					{
						printf("[RSP-V2-EOT-OK] end-of-transfer verified: delivered==committed=%llu "
							"crc=0x%08x (complete, in order, uncorrupted)\n",
							(unsigned long long)rx_stream_delivered, rx_stream_crc);
						fflush(stdout);
						clean_eot_verified = true;   // reconnect-seam F2: proven-clean boundary
						settled_close_cache.valid = true;
						settled_close_cache.peer_committed = peer_committed;
						settled_close_cache.peer_crc = peer_crc;
						settled_close_cache.replays = 0;
					}
				}
			}
#ifdef MERCURY_GUI_ENABLED
			if(passive_monitor)
				gui_push_monitor_event("[DISCONNECT]", false);
#endif
			if(passive_monitor)
				reset_session_state();
			else
				rsp_reset_session_preserve_close_for_ack();
			// Reconnect-continuity fail-closed F2: on a PROVEN-CLEAN EOT only, clear the app-delivered
			// high-water that reset_session_state() just re-snapshotted (monotonic-max), so the next
			// transfer on this persistent socket starts a fresh un-armed byte boundary (no false refuse).
			if(clean_eot_verified)
				rsp_seam_clear_on_clean_eot();
			if(passive_monitor)
			{
				// Monitor: go back to LISTENING for next session
				link_status=LISTENING;
				connection_status=RECEIVING;
				messages_control.status = FREE;
				hail_detected = NO;
			}
			else
			{
				link_status=DISCONNECTING;
				connection_status=ACKNOWLEDGING_CONTROL;
			}
			reset_all_timers();

			fifo_buffer_tx.flush();
			fifo_buffer_backup.flush();
			fifo_buffer_rx.flush();

			emit_disconnected_once();
		}
		else
		{
			// RX-CTRL-DROP fix (data-flow-control-slot-lifecycle.md §5 change B,
			// vector V1): the terminal outer `else` previously handled ONLY
			// CLOSE_CONNECTION, so any OTHER code that reached here (an unknown /
			// corrupted code byte, or a code whose handler guard above did not
			// match the current link_status) returned with the slot LEFT in
			// RECEIVED forever — wedging the one-deep mailbox so every later
			// control frame is dropped at the produce gate (:825 / :848). FREE it
			// now: the frame was not actionable in this state, so discarding it is
			// safe (the CMD will retransmit if it needed an ACK — identical to a
			// control frame lost on the air).
			printf("[RX-CTRL-DROP] FREE: unhandled control code=%d in link_status=%d "
				"(no handler matched; freeing stuck slot)\n",
				(int)code, link_status);
			fflush(stdout);
#ifndef RX_CTRL_DROP_FAILBEFORE
			messages_control.status = FREE;
#endif
		}
	}

	// RX-CTRL-DROP fix (data-flow-control-slot-lifecycle.md §5 change B, terminal
	// catch-all): a control code can also fall through EVERY outer guard above
	// (e.g. a valid code arriving in a link_status none of the outer `if`s match —
	// START_CONNECTION while CONNECTED, SET_CONFIG while not CONNECTED) and never
	// reach the terminal `else` (its CLOSE_CONNECTION branch). Such a path leaves
	// the slot RECEIVED with connection_status untouched (still RECEIVING). Any
	// handler that INTENDS to keep the slot RECEIVED for the control-ACK handoff
	// set connection_status to ACKNOWLEDGING_CONTROL / ACKNOWLEDGING_DATA, so we
	// only force-FREE a still-RECEIVED slot that is NOT heading into an ACK. This
	// preserves the normal RECEIVED→ACKED→FREE consume while closing the stuck-slot
	// leak deterministically (no need to wait for the update_status() watchdog).
	if(messages_control.status == RECEIVED
	   && connection_status != ACKNOWLEDGING_CONTROL
	   && connection_status != ACKNOWLEDGING_DATA)
	{
		printf("[RX-CTRL-DROP] FREE: control code=%d fell through all guards in "
			"link_status=%d connection_status=%d (freeing stuck slot)\n",
			(int)code, link_status, connection_status);
		fflush(stdout);
#ifndef RX_CTRL_DROP_FAILBEFORE
		messages_control.status = FREE;
#endif
	}
}

// FIX-6 (the deterministic 61,621-byte stall): send `length` bytes from `src` to
// the app data socket NON-LOSSILY. transmit() is a bare non-blocking send(); on a
// full OS send buffer it returns short (0..length-1) or would-block (<0). Pre-fix
// the caller IGNORED that return and the already-popped bytes were DISCARDED → the
// CMD had already ACKed the batch (no retransmit) → permanent byte-deterministic
// plateau. Here we capture the result and stash the UNSENT TAIL in
// rx_deliver_pending so the next ARQ tick re-sends it IN ORDER before popping more
// raw bytes — bounded buffering + natural backpressure, zero loss.
//   returns true  = fully sent (continue draining),
//           false = back-pressured, tail stashed, caller must stop draining.
bool cl_arq_controller::rx_deliver_send(const char* src, int length)
{
	if(length <= 0) return true;

	// Defensive: a single send unit can never exceed the socket message buffer
	// (MAX_BUFFER_SIZE). The original drain assumed this; clamp so a pathological
	// B2F over-expansion cannot overflow message->buffer. The remainder (if any)
	// is handled exactly like a short write below.
	if(length > MAX_BUFFER_SIZE)
		length = MAX_BUFFER_SIZE;

	memcpy(tcp_socket_data.message->buffer, src, length);
	tcp_socket_data.message->length = length;

	int n = tcp_socket_data.transmit();

	if(n >= length)
	{
		return true;                       // fully sent
	}

	// Short write (0<=n<length) or would-block (n<0): stash the unsent tail so it
	// re-sends, in order, next tick. The pending buffer holds at most one send
	// unit (MAX_BUFFER_SIZE, the socket message buffer bound).
	int unsent = (n < 0) ? length : (length - n);
	const char* tail = (n < 0) ? src : (src + n);
	if(unsent > (int)sizeof(rx_deliver_pending))
		unsent = (int)sizeof(rx_deliver_pending);   // safety clamp (cannot trigger: length<=buf)
	// memmove (NOT memcpy): when re-sending the pending tail, src IS
	// rx_deliver_pending, so tail (src+n) overlaps the destination.
	memmove(rx_deliver_pending, tail, unsent);
	rx_deliver_pending_len = unsent;
	return false;
}

void cl_arq_controller::process_buffer_data_responder()
{
	if(link_status==CONNECTED)
	{
		if (tcp_socket_data.get_status()==TCP_STATUS_ACCEPTED)
		{
			// FIX-6: first re-send any tail the previous tick could not push
			// (app socket was back-pressured). Stop the whole drain if it is
			// still congested — do NOT pop more raw bytes ahead of it (INV-1
			// in-order delivery).
			if(rx_deliver_pending_len > 0)
			{
				if(!rx_deliver_send(rx_deliver_pending, rx_deliver_pending_len))
					return;                 // still congested; tail re-stashed
				rx_deliver_pending_len = 0;  // tail fully drained
			}

			// Drain a completed reroll before popping newer raw FIFO bytes. One
			// socket-sized chunk is returned per tick; the suffix remains owned by
			// the handler and cannot be silently deleted by the output cap.
			if(b2f_handler.is_initialized() && b2f_handler.has_pending_rx_work())
			{
				char b2f_pending[MAX_BUFFER_SIZE];
				int b2f_len = b2f_handler.filter_rx(nullptr, 0,
					b2f_pending, sizeof(b2f_pending));
				if(b2f_len < 0)
				{
					abort_b2f_transfer("RX retained-output drain failed");
					return;
				}
				if(b2f_len > 0)
					rx_deliver_send(b2f_pending, b2f_len);
				return;
			}

			while(fifo_buffer_rx.get_size()!=fifo_buffer_rx.get_free_size())
			{
				// Pop raw data from RX FIFO
				char rx_raw[MAX_BUFFER_SIZE];
				// SACK Design A Step 1 — effective DATA_LONG header drives per-frame
				// data-pop size; v1 (default) identical to legacy macro.
				int rx_raw_len = fifo_buffer_rx.pop(rx_raw, max_data_length+max_header_length-effective_data_long_header_length(sack_v2_enabled, header_carries_d5));

				// Build the to-send unit (`send_buf`/`send_len`). For B2F the unit is
				// the POST-transform stream (the raw bytes are consumed by the parser
				// and cannot be re-popped — audit R4); otherwise it is rx_raw verbatim.
				char b2f_buf[MAX_BUFFER_SIZE];
				const char* send_buf = rx_raw;
				int send_len = 0;

				// B2F filter: parse incoming stream, reroll plaintext to LZHUF
				if(b2f_handler.is_initialized())
				{
					int b2f_len = b2f_handler.filter_rx(rx_raw, rx_raw_len,
						b2f_buf, sizeof(b2f_buf));
					if(b2f_len < 0)
					{
						abort_b2f_transfer("RX parser/reroll failed");
						return;
					}

					// Auto-arm compression when B2F detected on RX path
					// (CAP_COMPRESSION removed — always unconditional).
					if(!compression_enabled && !force_compress && b2f_handler.is_b2f_session())
					{
						compression_enabled = true;
						printf("[COMPRESS] Armed by B2F detection (responder)\n");
						fflush(stdout);
						if(!compressor.is_streaming())
							compressor.streaming_enable();
					}

					if(b2f_len > 0)
					{
						send_buf = b2f_buf;
						send_len = b2f_len;
					}
					else if(!b2f_handler.is_b2f_session())
					{
						send_buf = rx_raw;
						send_len = rx_raw_len;
					}
					else
					{
						// B2F active, parser accumulating partial line -- don't send raw
						send_len = 0;
					}
				}
				else
				{
					send_buf = rx_raw;
					send_len = rx_raw_len;
				}

				// FIX-6: non-lossy send. On back-pressure the unsent tail is stashed
				// in rx_deliver_pending and we STOP draining (the popped raw bytes are
				// already transformed into send_buf, so nothing is lost — the tail is
				// re-sent next tick). The B2F parser state already advanced for the
				// raw bytes we popped, so we must NOT re-pop them; stashing the
				// transformed tail is the correct non-lossy unit.
				if(send_len > 0)
				{
					if(!rx_deliver_send(send_buf, send_len))
						return;              // congested; tail stashed, resume next tick
				}
			}
		}

	}
}
