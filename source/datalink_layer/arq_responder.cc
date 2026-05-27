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
#include "common/timing_log.h"

#ifdef MERCURY_GUI_ENABLED
#include "gui/gui_state.h"
#endif

void cl_arq_controller::process_messages_responder()
{

	if(this->connection_status==ACKNOWLEDGING_CONTROL)
	{
		print_stats();
		process_messages_acknowledging_control();
	}
	else if(this->connection_status==ACKNOWLEDGING_DATA)
	{
		print_stats();
		process_messages_acknowledging_data();
	}
	else if(this->connection_status==RECEIVING)
	{
		process_messages_rx_data_control();
	}

}

int cl_arq_controller::add_message_rx_data(char type, char id, int length, char* data)
{
	int success=ERROR_;
	int loc=(int)((unsigned char)id);
	if(loc >= data_batch_size || loc < 0)
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
	if(type==DATA_LONG && length>(max_data_length+max_header_length-effective_data_long_header_length(sack_v2_enabled)))
	{
		success=MESSAGE_LENGTH_ERROR;
		return success;
	}

	// SACK Design A Step 2 — DATA_SHORT payload capacity is reduced by 1 byte
	// when sack_v2_enabled. v1 path uses the legacy 5-byte macro; bytes-on-the
	// -wire and bounds are identical to pre-Step-2.
	if(type==DATA_SHORT && length>(max_data_length+max_header_length-effective_data_short_header_length(sack_v2_enabled)))
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
		int fill_end = max_data_length+max_header_length-effective_data_long_header_length(sack_v2_enabled);
		if(fill_end > N_MAX/8) fill_end = N_MAX/8;
		for(int j=messages_rx[loc].length;j<fill_end;j++)
		{
			messages_rx[loc].data[j]=0;
		}
	}
	if(messages_rx[loc].status==FREE || messages_rx[loc].status==ACKED)
	{
		stats.nReceived_data++;
	}
	messages_rx[loc].status=RECEIVED;
	success=SUCCESSFUL;
	return success;
}


void cl_arq_controller::process_messages_rx_data_control()
{
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

			if(!passive_monitor)
			{
				// Notify Winlink/trimode that incoming traffic detected
				std::string pending_str = "PENDING\r";
				tcp_socket_control.message->length = pending_str.length();
				for(int i = 0; i < (int)pending_str.length(); i++)
					tcp_socket_control.message->buffer[i] = pending_str[i];
				tcp_socket_control.transmit();
			}

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
				msleep(delay_ms);
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
				circular_buf_reset(capture_buffer);
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
				int sym_ms = (telecom_system->data_container.Nofdm
					* telecom_system->data_container.interpolation_rate * 1000) / 48000;
				int remaining_syms =
					telecom_system->ack_mfsk.connect_pattern_nsymb
					+ telecom_system->ack_mfsk.ack_sack_suffix_len()
					- telecom_system->ack_mfsk.connect_match_threshold;
				if(remaining_syms < 0) remaining_syms = 0;
				int delay_ms = remaining_syms * sym_ms + 200;
				printf("[RSP-CONNECT-V2] Waiting %d ms (HAIL race delay) "
					"before synthesizing messages_rx_buffer\n", delay_ms);
				fflush(stdout);
				msleep(delay_ms);

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
		// Gates differ from Site B by link_status:
		//  - link_status ∈ {CONNECTION_RECEIVED, CONNECTED}: we're either
		//    waiting for the first TEST_CONNECTION (post-START_CONN ACK) or
		//    repeating a previously-decoded one (legacy retry path).
		//  - messages_rx_buffer.status != RECEIVED: Site B didn't already
		//    synthesize a frame on this tick.
		// Other gates (messages_control.status == FREE, !passive_monitor,
		// connect_pattern_nsymb > 0) match Site B exactly.
		if((link_status == CONNECTION_RECEIVED || link_status == CONNECTED)
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
				msleep(delay_ms);

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
				//   data[5]   = local_capability (peer's cap byte; gated to 2 bits)
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

		// Emergency BREAK: commander signals "drop to ROBUST_0"
		// Only respond if we're actually connected — prevents all radios on a
		// frequency from ACKing someone else's BREAK.
		if(break_detected == YES && link_status != CONNECTED)
			break_detected = NO;
		if(break_detected == YES && link_status == CONNECTED)
		{
			printf("[BREAK] %s, dropping to ROBUST_0\n",
				passive_monitor ? "Observed" : "Responding with ACK");
			fflush(stdout);
			break_detected = NO;

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
			rsp_current_expected_batch_seq_id = -1;
			rsp_prev_batch_seq_id = -1;

#ifdef MERCURY_GUI_ENABLED
			if(passive_monitor)
				gui_push_monitor_event("[BREAK -> ROBUST_0]", false);
#endif

			// Send ACK to confirm BREAK received (suppressed in monitor mode)
			send_ack_pattern();

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
				// Monitor: auto-adopt session if we receive data while still LISTENING
				// (missed START_CONNECTION — joined mid-session)
				if(passive_monitor && link_status == LISTENING)
				{
					printf("[MONITOR] Data frame received while LISTENING — adopting session mid-stream\n");
					fflush(stdout);
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
						rsp_current_expected_batch_seq_id = bsi;
						printf("[RSP-V2-ADOPT] current_expected_batch_seq_id=%d "
							"(first v2 DATA frame this session)\n", bsi);
						fflush(stdout);
					}
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

				// SACK Design A Step 8a — match-prev branch: store the frame
				// into messages_rx_prev[] without touching messages_rx[] or
				// the current-batch receive timer. The prev path runs to
				// completion independently from the current-batch ACK-GATE.
				if(v2_route_to_prev)
				{
					int loc = (int)((unsigned char)messages_rx_buffer.id);
					int eff_long  = effective_data_long_header_length(sack_v2_enabled);
					int eff_short = effective_data_short_header_length(sack_v2_enabled);
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
					if(loc < 0 || loc >= this->data_batch_size)   len_ok = false;
					if(loc >= this->nMessages)                    len_ok = false;
					if(messages_rx_buffer.length < 0)             len_ok = false;
					if(len_ok)
					{
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
						if(prev_status != RECEIVED && prev_status != ACKED)
							rsp_prev_batch_received_count++;

						printf("[RSP-V2-PREV-RX] bsi=%d id=%d seq=%d/%d len=%d "
							"prev_received=%d/%d\n",
							(int)(unsigned char)messages_rx_buffer.batch_seq_id,
							(int)(unsigned char)messages_rx_buffer.id,
							messages_rx_buffer.sequence_number, data_batch_size,
							messages_rx_buffer.length,
							rsp_prev_batch_received_count,
							rsp_prev_batch_expected_count);
						fflush(stdout);

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
							// ACKED-only iteration picks them up.
							for(int i=0; i<this->data_batch_size && i<this->nMessages; i++)
							{
								if(messages_rx[i].status == RECEIVED)
									messages_rx[i].status = ACKED;
							}
							copy_data_to_buffer();
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
							printf("[RSP-V2-PREV-DELIVERED] prev_batch_seq_id=%d "
								"deliveries_total=%lld (cross-storage path drained; "
								"current-batch storage untouched)\n",
								rsp_prev_batch_seq_id, rsp_prev_batch_delivered_count);
							fflush(stdout);
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
				add_message_rx_data(messages_rx_buffer.type, messages_rx_buffer.id, messages_rx_buffer.length, messages_rx_buffer.data);
				batch_rx_frame_count++;
				int rx_timeout = 0;
				int effective_batch = data_batch_size;
				{
					// Determine actual expected frame count.
					// With adaptive batch sizing, commander may send fewer frames
					// than data_batch_size. Use compression header to detect this.
					// End-of-batch flag: commander marks last frame with bit 7
					if(last_received_end_of_batch_seq >= 0)
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
						int mf = max_data_length + max_header_length - effective_data_long_header_length(sack_v2_enabled);
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

			// Restart timer so we can receive the next CMD retransmit.
			// Without this, the stopped timer returns 0 forever and the
			// RSP is stuck in RECEIVING (0 < timeout always true, but
			// the else block never triggers again).
			calculate_receiving_timeout();
			receiving_timer.start();
		}

		// If we responded to HAIL but START_CONNECTION never arrived,
		// go back to HAIL scanning for the next beacon.
		if(link_status == LISTENING && hail_detected == YES)
		{
			printf("[HAIL] Timeout waiting for START_CONNECTION, resuming HAIL scan\n");
			fflush(stdout);
			hail_detected = NO;
		}
	}
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

		if(messages_control.data[0] == KEY_EXCHANGE_1)
		{
			// KEY_EXCHANGE_1: must use LDPC ACK to carry responder's pubkey.
			// Send on data_configuration (OFDM) — NOT ack_configuration (MFSK).
			// Commander will load data_configuration to receive this.
			printf("[ACK-CTRL] Sending LDPC ACK for KEY_EXCHANGE_1 on config %d\n",
				data_configuration);
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
				}
			}
		}
		else if(ack_pattern_time_ms > 0)
		{
			// During turboshift: send ACK + SNR suffix so commander can SUPERSHIFT
			bool is_turbo_setconfig = (turboshift_active || turboshift_phase != TURBO_DONE) &&
				messages_control.data[0] == SET_CONFIG &&
				measurements.SNR_uplink > -90;
			if(is_turbo_setconfig)
			{
				printf("[ACK-CTRL] Sending ACK+SNR pattern (SNR=%.1f dB)\n",
					measurements.SNR_uplink);
				fflush(stdout);
				send_ack_pattern_with_snr((float)measurements.SNR_uplink);
			}
			else
			{
				if(g_verbose) { printf("[ACK-CTRL] Sending ACK pattern (no config switch)\n"); fflush(stdout); }
				send_ack_pattern();
			}
			// If config changed (e.g., SET_CONFIG), load the new data config now.
			// ACK was sent on old config (correct — commander is still on old config),
			// but we need to switch to new config before receiving data.
			if(data_configuration != current_configuration)
			{
				if(g_verbose) { printf("[ACK-CTRL] Loading new data config %d (was %d)\n", data_configuration, current_configuration); fflush(stdout); }
				load_configuration(data_configuration, PHYSICAL_LAYER_ONLY, YES);
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
			while(ptt_off_wait.get_elapsed_time_ms()<ptt_off_delay_ms);

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
				char tmp = forward_configuration;
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
			last_message_received_type=NONE;
			last_message_sent_type=NONE;
			last_received_message_sequence=-1;
		}
		else if(ack_command==CLOSE_CONNECTION)
		{
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
		}
	}
}


void cl_arq_controller::process_messages_acknowledging_data()
{
	printf("[RSP-RX-TIMEOUT] Entering ACK-GATE: rx_count=%d timeout=%d batch=%d\n",
		batch_rx_frame_count, receiving_timeout, data_batch_size);
	fflush(stdout);

	// M3 (SACK turnaround trace): the collision STARTS the moment RSP decides
	// to ACK-GATE. Anchors the decision instant on the common clock — the
	// existing line above is a plain printf the parser cannot ingest.
	mtl::log_event_kv("rsp_ack_gate_entry", "rx_count=%d batch=%d",
		batch_rx_frame_count, data_batch_size);

	int nAck_messages=0;
	receiving_timer.stop();
	receiving_timer.reset();

	// Bug #36: reset nUnder BEFORE ACK TX so only turnaround-period
	// nUnder is counted (not accumulated frame-processing nUnder).
	telecom_system->data_container.nUnder_processing_events = 0;

	if(ack_pattern_time_ms > 0)
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
			int rx_received = 0;
			for(int i = 0; i < data_batch_size; i++)
				if(messages_rx[i].status == RECEIVED) rx_received++;

			int expected = data_batch_size;  // Default for non-compressed
			// End-of-batch flag: commander marks last frame with bit 7 in
			// sequence_number, giving us the actual batch size sent.
			// Works for all modes (compressed, uncompressed, encrypted).
			if(last_received_end_of_batch_seq >= 0)
			{
				expected = last_received_end_of_batch_seq + 1;
				if(expected > data_batch_size) expected = data_batch_size;
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
				int mf = max_data_length + max_header_length - effective_data_long_header_length(sack_v2_enabled);
				expected = (total_compressed + mf - 1) / mf;
				if(expected > data_batch_size) expected = data_batch_size;
				if(expected < 1) expected = 1;
			}

			// Diagnostic: show which sequence numbers were received
			{
				printf("[ACK-GATE-DIAG] rx=%d/%d exp=%d seqs:", rx_received, data_batch_size, expected);
				for(int i = 0; i < data_batch_size; i++)
					if(messages_rx[i].status == RECEIVED) printf(" %d", i);
				printf("\n"); fflush(stdout);
			}

			if(data_batch_size > 1 && rx_received < expected && !passive_monitor)
			{
				if(sack_enabled && rx_received > 0)
				{
					// SACK: send selective ACK with bitmap of received frames
					printf("[ACK-GATE] SACK: received %d/%d (expected %d)\n",
						rx_received, data_batch_size, expected);
					fflush(stdout);

					// Build bitmap: true = frame received
					bool sack_bitmap[MAX_SACK_BATCH_SIZE];
					for(int i = 0; i < data_batch_size && i < MAX_SACK_BATCH_SIZE; i++)
						sack_bitmap[i] = (messages_rx[i].status == RECEIVED);

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
							bump_bsi_and_transfer_prev();

							// MFSK-suffix ACK+SACK — WB-only (suffix_len()>0).
							// No optimizer-territory gate: the pattern correlator
							// gives this path ROBUST_0-grade detection at any
							// config, which is exactly where partial-batch
							// recovery matters most. Pack the per-frame received
							// bitmap LSB-first into a uint32_t (matches
							// send_sack_v2_frame's byte-LSB-first convention at
							// arq_common.cc:4147-4151 for the first 32 frames).
							bool used_mfsk_path = false;
							if (MFSK_ACK_SACK_ENABLED
								&& telecom_system->ack_mfsk.ack_sack_suffix_len() > 0)
							{
								uint32_t bitmap_u32 = 0;
								// Phase B Wave 1 flag-day (fact-doc §11.2): MFSK
								// ctrl-suffix bitmap is 30 bits (was 32). Cap nbits
								// to 30 so we never set bits 30/31 — those would be
								// silently dropped by pack_ack_sack_payload and the
								// receiver would never see them.
								int nbits = data_batch_size;
								if (nbits > 30) nbits = 30;
								for (int i = 0; i < nbits; i++)
								{
									if (sack_bitmap[i])
										bitmap_u32 |= (1u << i);
								}
								printf("[RSP-MFSK-SACK] partial path: batch_seq_id=%u bitmap=0x%08x nframes=%d\n",
									(unsigned)sacked_bsi, (unsigned)bitmap_u32, data_batch_size);
								fflush(stdout);
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
								send_sack_v2_frame(sack_bitmap, data_batch_size, sacked_bsi);
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
					// Post-SACK timeout: cover CMD listen + CMD retransmit TX.
					// 1.5x is enough: CMD extended listen (~7.8s) + retransmit
					// (~9.85s) = ~17.6s cycle. 1.5x * 9850 = 14775ms, so RSP
					// responds before CMD's next retransmit TX starts.
					// (Was 2x=19700ms which exceeded CMD cycle, causing RSP ACK
					// to arrive during CMD retransmit TX — half-duplex collision.)
					receiving_timeout = receiving_timeout * 3 / 2;
					printf("[RSP-POST-SACK] timeout=%dms (1.5x normal for retransmit)\n",
						receiving_timeout);
					fflush(stdout);
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
				// Reset RX state for fresh retransmission capture
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

			// Mark all received messages as ACKED and count for stats
			for(int i=0; i<this->nMessages; i++)
			{
				if(messages_rx[i].status==RECEIVED)
				{
					messages_rx[i].status=ACKED;
					nAck_messages++;
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
				rsp_prev_batch_seq_id = rsp_current_expected_batch_seq_id;
				rsp_current_expected_batch_seq_id =
					(rsp_current_expected_batch_seq_id + 1) & 0xFF;
				printf("[RSP-V2-BATCH-DONE] prev=%d next_expected=%d\n",
					rsp_prev_batch_seq_id, rsp_current_expected_batch_seq_id);
				fflush(stdout);
			}
		}
		repeating_last_ack=NO;
		messages_control.status=FREE;

		// Clean-batch ACK transport. WB sessions use the MFSK-suffix
		// ACK+SACK frame (16-symbol Welch-Costas pattern + 13 MFSK data
		// symbols carrying [bsi:8 | bitmap:32 | crc12:12]). NB sessions
		// (suffix_len==0) fall back to the legacy MFSK ACK pattern with
		// no SACK payload — NB doesn't have the symbol-rate budget for
		// SACK and the receiver implicitly treats any pattern hit as a
		// clean ACK. See mercury/fact-documents/mfsk-robust-ack.md §3.4.
		if(sack_v2_enabled)
		{
			unsigned char ack_bsi = (unsigned char)(
				rsp_prev_batch_seq_id >= 0 ? rsp_prev_batch_seq_id : 0);

			bool used_mfsk_path = false;
			if (MFSK_ACK_SACK_ENABLED
				&& telecom_system->ack_mfsk.ack_sack_suffix_len() > 0)
			{
				// Phase B Wave 1 flag-day (fact-doc §11.2): bitmap is 30 bits
				// (was 32). Producer side cap so we never set bits 30/31.
				uint32_t bitmap_u32;
				if (data_batch_size >= 30)
					bitmap_u32 = 0x3FFFFFFFu;
				else if (data_batch_size <= 0)
					bitmap_u32 = 0u;
				else
					bitmap_u32 = (1u << data_batch_size) - 1u;
				printf("[RSP-MFSK-SACK] clean path: batch_seq_id=%u bitmap=0x%08x nframes=%d\n",
					(unsigned)ack_bsi, (unsigned)bitmap_u32, data_batch_size);
				fflush(stdout);
				long long mfsk_ms = send_mfsk_ack_sack(ack_bsi, bitmap_u32);
				if (mfsk_ms > 0)
				{
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
				// NB or MFSK-suffix unavailable: legacy MFSK pattern.
				send_ack_pattern();
			}
		}
		else
		{
			// Legacy MFSK ACK (M=16, nStreams=1) — dedicated ack_mfsk
			// path, no config switch needed.
			send_ack_pattern();
		}

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
		if(!batch_data_delivered)
		{
			copy_data_to_buffer();
			batch_data_delivered = true;  // Prevent duplicate delivery on retransmit
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
		telecom_system->set_mfsk_ctrl_mode(false);
		telecom_system->data_container.frames_to_read =
			telecom_system->data_container.preamble_nSymb + telecom_system->data_container.Nsymb + 10;

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
			// Monitor mode: accept ANY START_CONNECTION, extract callsigns, fast-track to CONNECTED
			int peer_flags = 0;
			destination_call_sign = callsign_unpack(&messages_control.data[2], &peer_flags);
			printf("[MONITOR] Observed START_CONNECTION from '%s'\n", destination_call_sign.c_str());
			fflush(stdout);

			bool peer_narrowband = (peer_flags & 0x01) != 0;
			session_narrowband = peer_narrowband || (narrowband_enabled == YES);
			compression_enabled = true;  // Assume compression (almost always on)

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

			// Send PENDING to Winlink to notify incoming connection
			// This allows Winlink to stop scanning and prepare PTT
			std::string pending_str="PENDING "+destination_call_sign+"\r";
			tcp_socket_control.message->length=pending_str.length();
			for(int i=0;i<(int)pending_str.length();i++)
			{
				tcp_socket_control.message->buffer[i]=pending_str[i];
			}
			tcp_socket_control.transmit();

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
		measurements.SNR_uplink=(double)tmp_SNR.f_SNR;

		// Read commander's capability from byte 5 of decoded LDPC frame.
		// Always present (LDPC decodes full block; unused bytes are zero-padded).
		// Backwards-compatible: old firmware doesn't fill byte 5 → decodes as 0 = no WB.
		peer_capability = (uint8_t)messages_control.data[5];
		printf("[BW-NEG] Commander capability: 0x%02X (WB=%s, ENCRYPT=%s)\n",
			peer_capability,
			(peer_capability & CAP_WB_CAPABLE) ? "yes" : "no",
			(peer_capability & CAP_ENCRYPTION) ? "yes" : "no");
		fflush(stdout);

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

		// Encryption negotiation
		{
			bool both_support = (local_capability & CAP_ENCRYPTION) &&
			                    (peer_capability & CAP_ENCRYPTION);
			if (encryption_mode != ENCRYPT_OFF && both_support)
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
			else if (encryption_mode != ENCRYPT_OFF && !both_support)
			{
				if (encryption_mode == ENCRYPT_STRICT)
				{
					printf("[CRYPTO] STRICT mode: peer lacks encryption — refusing connection\n");
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
				printf("[CRYPTO] WARNING: Peer does not support encryption (peer_cap=0x%02X)\n",
					peer_capability);
			}
		}

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
			int max_batch = (message_transmission_time_ms > 0)
				? (int)(30000.0 / message_transmission_time_ms + 0.5) : 31;
			if(max_batch < 5) max_batch = 5;
			if(max_batch > nMessages) max_batch = nMessages;
			int new_batch = radio_batch_size;
			if(new_batch > max_batch) new_batch = max_batch;
			set_data_batch_size(new_batch);
			nominal_batch_size = new_batch;
			recalculate_ack_timeout_for_batch();
			printf("[SACK] Enabled (radio_batch=%d crypto_batch=%d headroom=%d batch=%d)\n",
				radio_batch_size, crypto_batch_size, retransmit_headroom, data_batch_size);
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
		}

		link_status=CONNECTED;
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
			connection_status = ACKNOWLEDGING_CONTROL;
			printf("[HANDSHAKE-ECHO] RSP queued TEST_CONNECTION_ACK: "
				"echoed_cap=0x%02X own_cap=0x%02X crc8=0x%02X\n",
				(unsigned char)peer_capability,
				(unsigned char)local_capability,
				(unsigned char)messages_control.data[3]);
			fflush(stdout);
		}
		watchdog_timer.start();
		link_timer.start();


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

			// Compute shared secret from commander's pubkey
			if(cipher_suite.compute_x25519_shared((const uint8_t*)&messages_control.data[1]) != 0)
			{
				printf("[CRYPTO] FATAL: X25519 shared secret is zero (low-order point attack?)\n");
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
				tx_batch_counter = 0;
				rx_batch_counter = 0;
				consecutive_auth_failures = 0;

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
					(is_ofdm_config(forward_configuration) || is_robust_config(forward_configuration)))
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
					circular_buf_reset(capture_buffer);
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
				(is_ofdm_config(forward_configuration) || is_robust_config(forward_configuration)))
			{
				// Don't load_configuration here — ack_configuration must stay on
				// the OLD config so the ACK reaches the commander (still on old config).
				// Just save data_configuration; acknowledging_control will call
				// load_configuration(data_configuration, ...) after the ACK is sent.
				data_configuration = forward_configuration;
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
			copy_data_to_buffer();
			batch_data_delivered = false;
			messages_last_ack_bu.type=NONE;
			link_timer.start();
			watchdog_timer.start();
		}
		else if(code==SWITCH_ROLE)
		{
			printf("switch role\n");
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
	else if((link_status==CONNECTED || link_status==DROPPED) && code==SET_LINK_PARAMS)
	{
		// SACK Design A Step 10 — RSP-side SET_LINK_PARAMS handler.
		// Note: link_status DROPPED accepted too because the lossy paths that
		// motivate Axis-2 (channel just dropped batches → CMD is shrinking
		// batch_size) often coincide with the RSP transiently reading the
		// link as DROPPED. The CMD's intent is to push the new batch size
		// regardless — accepting in DROPPED gives the link a faster recovery.
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
				if(target > AXIS2_BATCH_CEIL)  target = AXIS2_BATCH_CEIL;

				int old_batch = data_batch_size;
				set_data_batch_size(target);
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
	else
	{
		if(code==CLOSE_CONNECTION)
		{
#ifdef MERCURY_GUI_ENABLED
			if(passive_monitor)
				gui_push_monitor_event("[DISCONNECT]", false);
#endif
			reset_session_state();
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

			std::string str="DISCONNECTED\r";
			tcp_socket_control.message->length=str.length();

			for(int i=0;i<tcp_socket_control.message->length;i++)
			{
				tcp_socket_control.message->buffer[i]=str[i];
			}
			tcp_socket_control.transmit();
		}
	}

}

void cl_arq_controller::process_buffer_data_responder()
{
	if(link_status==CONNECTED)
	{
		if (tcp_socket_data.get_status()==TCP_STATUS_ACCEPTED)
		{
			while(fifo_buffer_rx.get_size()!=fifo_buffer_rx.get_free_size())
			{
				// Pop raw data from RX FIFO
				char rx_raw[MAX_BUFFER_SIZE];
				// SACK Design A Step 1 — effective DATA_LONG header drives per-frame
				// data-pop size; v1 (default) identical to legacy macro.
				int rx_raw_len = fifo_buffer_rx.pop(rx_raw, max_data_length+max_header_length-effective_data_long_header_length(sack_v2_enabled));

				// B2F filter: parse incoming stream, reroll plaintext to LZHUF
				if(b2f_handler.is_initialized())
				{
					char b2f_buf[MAX_BUFFER_SIZE * 4]; // LZHUF can be larger than plaintext (rare)
					int b2f_len = b2f_handler.filter_rx(rx_raw, rx_raw_len,
						b2f_buf, sizeof(b2f_buf));

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
						memcpy(tcp_socket_data.message->buffer, b2f_buf, b2f_len);
						tcp_socket_data.message->length = b2f_len;
					}
					else if(!b2f_handler.is_b2f_session())
					{
						memcpy(tcp_socket_data.message->buffer, rx_raw, rx_raw_len);
						tcp_socket_data.message->length = rx_raw_len;
					}
					else
					{
						// B2F active, parser accumulating partial line -- don't send raw
						tcp_socket_data.message->length = 0;
					}
				}
				else
				{
					memcpy(tcp_socket_data.message->buffer, rx_raw, rx_raw_len);
					tcp_socket_data.message->length = rx_raw_len;
				}

				if(tcp_socket_data.message->length > 0)
					tcp_socket_data.transmit();
			}
		}

	}
}

// ============================================================================
// SACK Partial-Path BSI Non-Advance — in-process reproducer (test-only)
// ============================================================================
//
// CLI: --test-partial-bsi-advance=mfsk|ofdm
//
// Reproduces the bug documented in mercury/fact-documents/sack_partial_bsi_advance.md
// (§5.3 / §5.4). The bug: when the MFSK suffix ACK+SACK path is taken for a
// partial batch (arq_responder.cc:1198-1204, used_mfsk_path=true), the
// subsequent send_sack_v2_frame() call at arq_responder.cc:1213 is SKIPPED.
// That call is where Step 8a's bsi-bump-and-transfer-to-prev block lives
// (arq_common.cc:4054-4137). Skipping it leaves `rsp_current_expected_batch_seq_id`
// stuck on the partial's bsi, so when CMD sends a mixbatch (retx of old-bsi +
// new-bsi frames) under a larger data_batch_size from a coincident Axis-2
// up-move, every new-bsi frame is dropped as out_of_window at
// arq_responder.cc:403-413.
//
// Variants:
//   mfsk → exercises the MFSK suffix bypass: should FAIL on HEAD with
//          drop_count >= 26 (byte-identical to r4's [RSP-V2-DROP] batch_seq_id=4
//          expected=3 lines).
//   ofdm → forces used_mfsk_path=false (no bypass): should PASS on HEAD because
//          send_sack_v2_frame's bump-and-transfer runs. Regression guard.
//
// We do NOT call send_sack_v2_frame() directly here — it does real OFDM TX via
// send_batch(), requiring a fully-initialized telecom_system. Instead, we
// REPLICATE the Step 8a state mutations (lines 4054-4137 of arq_common.cc)
// inline for the OFDM variant. The bug is purely state-machine; no DSP/timing
// dependence. Per fact-doc §5.2 — this is the surgically correct unit-test
// approach.
//
// PASS:  drop_count < 2 for bsi=4 arrivals.
// FAIL:  drop_count >= 2 (in practice 26 — all bsi=4 frames after the retx fill).
//
// Returns: 0 on PASS, 1 on FAIL.
int cl_arq_controller::test_partial_bsi_advance(const char* transport)
{
	if(transport == NULL) transport = "mfsk";
	bool variant_mfsk = (strcmp(transport, "mfsk") == 0);
	bool variant_ofdm = (strcmp(transport, "ofdm") == 0);
	if(!variant_mfsk && !variant_ofdm)
	{
		printf("[TEST-PARTIAL-BSI] ERROR: transport must be 'mfsk' or 'ofdm' (got '%s')\n",
			transport);
		fflush(stdout);
		return 1;
	}

	// --- Step 0: allocate buffers ------------------------------------------
	// We must avoid load_configuration() (depends on a fully-initialized
	// telecom_system); set just the fields init_messages_buffers() reads.
	this->nMessages = 255;
	this->max_data_length = 170;
	this->max_message_length = 200;
	this->max_header_length = 6;
	int alloc_rc = init_messages_buffers();
	if(alloc_rc != SUCCESSFUL)
	{
		printf("[TEST-PARTIAL-BSI] ERROR: init_messages_buffers() failed (rc=%d)\n",
			alloc_rc);
		fflush(stdout);
		return 1;
	}

	// --- Step 1: prime SACK v2 state ---------------------------------------
	// Bypass set_data_batch_size()'s clamp (depends on max_*_length values that
	// would be initialized by load_configuration). The values we set above are
	// large enough that the clamp at arq_common.cc:548 won't trigger; assign
	// directly anyway to match the pattern used by test_fire_policy_axis2().
	this->sack_v2_enabled              = true;
	this->sack_enabled                 = true;
	this->axis3_sack_mode              = 1;  // SACK_MODE_ON
	this->data_batch_size              = 25;
	this->rsp_current_expected_batch_seq_id = 3;
	this->rsp_prev_batch_seq_id        = 2;
	this->rsp_prev_batch_active        = false;
	this->rsp_v2_drop_count            = 0;
	this->compression_enabled          = false;  // Avoid header-fallback in expected calc

	// --- Step 2: populate 24 RECEIVED slots in messages_rx[] ---------------
	// bsi=3, slots 0..12 + 14..24 (slot 13 is the missing frame). Slot 24
	// carries the end-of-batch flag (sequence_number bit 7 in §2.2). On the
	// wire, EOB is encoded by setting `last_received_end_of_batch_seq` at
	// arq_common.cc:5587 when the bit-7 frame is decoded — replicate that.
	for(int i = 0; i < this->nMessages; i++)
	{
		messages_rx[i].status       = FREE;
		messages_rx[i].length       = 0;
		messages_rx[i].batch_seq_id = -1;
	}
	int received_slots = 0;
	for(int i = 0; i < 25; i++)
	{
		if(i == 13) continue;  // missing frame
		messages_rx[i].type             = DATA_LONG;
		messages_rx[i].id               = (char)(unsigned char)i;
		messages_rx[i].length           = 16;  // small payload
		messages_rx[i].status           = RECEIVED;
		messages_rx[i].batch_seq_id     = 3;
		messages_rx[i].sequence_number  = (char)(unsigned char)i;
		for(int j = 0; j < 16; j++) messages_rx[i].data[j] = (char)(i * 7 + j);
		received_slots++;
	}
	this->last_received_end_of_batch_seq = 24;  // slot 24 carried bit-7
	this->batch_rx_frame_count           = received_slots;
	printf("[TEST-PARTIAL-BSI] setup: bsi=3 partial %d/25 (slot 13 missing) batch=25 transport=%s\n",
		received_slots, transport);
	fflush(stdout);

	// --- Step 3: dispatch the partial-path SACK (the bug site) -------------
	// Replicate arq_responder.cc:1102-1242 in compressed form. The CRITICAL
	// branch is the MFSK suffix path: when used_mfsk_path=true,
	// send_sack_v2_frame() is bypassed (skipping Step 8a's bump-and-transfer).
	//
	// For 'mfsk' variant: simulate the MFSK suffix succeeding → bypass.
	// For 'ofdm' variant: simulate used_mfsk_path=false → run the Step 8a
	//   bump-and-transfer block (inlined from arq_common.cc:4054-4137).
	{
		int rx_received = 0;
		for(int i = 0; i < this->data_batch_size; i++)
			if(messages_rx[i].status == RECEIVED) rx_received++;
		int expected = this->data_batch_size;
		if(this->last_received_end_of_batch_seq >= 0)
		{
			int eob = this->last_received_end_of_batch_seq + 1;
			if(eob < expected) expected = eob;
		}
		printf("[ACK-GATE] SACK: received %d/%d (expected %d)\n",
			rx_received, this->data_batch_size, expected);

		unsigned char bsi = (unsigned char)
			((this->rsp_current_expected_batch_seq_id >= 0)
				? (this->rsp_current_expected_batch_seq_id & 0xFF) : 0);
		printf("[ACK-GATE-V2] dispatching OFDM SACK_RSP (batch_seq_id=%u, %d/%d received, sack_mode=ON)\n",
			(unsigned)bsi, rx_received, this->data_batch_size);
		fflush(stdout);

		// §6g fix (2026-05-21): both transports now share the Step 8a
		// bump-and-transfer via the hoisted bump_bsi_and_transfer_prev()
		// helper, fired from arq_responder.cc:~1181 BEFORE the transport
		// choice. The test mirrors production: invoke the production helper
		// once for both variants. The variant only changes the synthetic
		// wire-log line printed (which transport "won"); the state mutations
		// are identical, which is the whole point of §6g.
		//
		// PRE-FIX behavior (for reference, to understand FAIL→PASS):
		//   mfsk variant: helper was NOT called → bsi stuck at 3 → bsi=4
		//     mixbatch arrivals all dropped as out_of_window (FAIL).
		//   ofdm variant: helper was called inline → bsi advanced to 4 →
		//     bsi=4 arrivals routed correctly (PASS, regression guard).
		// POST-FIX: production calls the helper for both transports, so the
		// test now calls the helper for both transports. Both variants PASS.
		if(variant_mfsk)
		{
			printf("[RSP-MFSK-SACK] partial path: batch_seq_id=%u nframes=%d (synthetic)\n",
				(unsigned)bsi, this->data_batch_size);
			printf("[TX-ACK-SACK] partial via MFSK suffix wire_ms=991\n");
			fflush(stdout);
		}
		else
		{
			printf("[TX-SACK-V2] (synthetic OFDM SACK_RSP TX for batch_seq_id=%u nframes=%d)\n",
				(unsigned)bsi, this->data_batch_size);
			fflush(stdout);
		}
		// Production call: this is the SAME helper invoked from
		// arq_responder.cc:~1181 in real-world partial-batch dispatch.
		bump_bsi_and_transfer_prev();
		// Both paths clear last_received_end_of_batch_seq (arq_responder.cc:1223).
		this->last_received_end_of_batch_seq = -1;
		this->batch_rx_frame_count           = 0;
	}

	// --- Step 4: simulate SET_LINK_PARAMS apply (batch 25 -> 30) -----------
	// Mirror arq_responder.cc:2305 set_data_batch_size(target). Direct assign
	// because pre-config max_*_length clamp would corrupt the value.
	int old_batch = this->data_batch_size;
	this->data_batch_size = 30;
	printf("[RSP-LINK-PARAMS] APPLIED batch %d -> 30 sack_mode=1 (prev sack_mode=1) (synthetic)\n",
		old_batch);
	fflush(stdout);

	// --- Step 5: synthesize the mixbatch arrival ---------------------------
	// 1 retx of bsi=3 slot 13, then 26 new-bsi-4 frames (slots 0..25; slot 25
	// carries EOB). Run them through the bsi-routing block from
	// arq_responder.cc:332-441 in replicated form (we cannot invoke the full
	// process_messages_rx_data_control() — it depends on too much real
	// telecom_system state). The routing logic itself (match_current /
	// match_prev / drop) is what we're exercising; the storage details after
	// routing are irrelevant for the bug signature.
	struct arrival { int bsi; int seq; bool eob; };
	struct arrival arrivals[27];
	arrivals[0].bsi = 3; arrivals[0].seq = 13; arrivals[0].eob = false;
	for(int i = 0; i < 26; i++)
	{
		arrivals[1 + i].bsi = 4;
		arrivals[1 + i].seq = i;
		arrivals[1 + i].eob = (i == 25);
	}

	long long bsi4_drops = 0;
	int bsi3_retx_routed = 0;  // matched prev or current → not dropped
	int bsi4_routed      = 0;
	int batch_done_count = 0;

	for(int k = 0; k < 27; k++)
	{
		int  abs_bsi = arrivals[k].bsi;
		int  abs_seq = arrivals[k].seq;
		bool eob     = arrivals[k].eob;
		bool match_current = (abs_bsi == this->rsp_current_expected_batch_seq_id);
		bool match_prev    = (this->rsp_prev_batch_seq_id >= 0
		                      && abs_bsi == this->rsp_prev_batch_seq_id);
		if(!match_current && !match_prev)
		{
			this->rsp_v2_drop_count++;
			printf("[RSP-V2-DROP] batch_seq_id=%d expected=%d prev=%d "
				"reason=unknown_or_out_of_window (drop_count=%lld)\n",
				abs_bsi, this->rsp_current_expected_batch_seq_id,
				this->rsp_prev_batch_seq_id, this->rsp_v2_drop_count);
			fflush(stdout);
			if(abs_bsi == 4) bsi4_drops++;
			continue;
		}
		// Routed (to current or prev). Simulate the storage write so the
		// post-loop ACK-GATE can compute rx_received correctly. EOB-bearing
		// frame updates last_received_end_of_batch_seq just like the real
		// decoder at arq_common.cc:5587.
		if(match_current)
		{
			int loc = abs_seq;
			if(loc >= 0 && loc < this->nMessages)
			{
				messages_rx[loc].type            = DATA_LONG;
				messages_rx[loc].id              = (char)(unsigned char)loc;
				messages_rx[loc].length          = 16;
				messages_rx[loc].status          = RECEIVED;
				messages_rx[loc].batch_seq_id    = abs_bsi;
				messages_rx[loc].sequence_number = (char)(unsigned char)abs_seq;
			}
			if(eob && (abs_bsi == this->rsp_current_expected_batch_seq_id))
				this->last_received_end_of_batch_seq = abs_seq;
			if(abs_bsi == 4) bsi4_routed++;
			if(abs_bsi == 3) bsi3_retx_routed++;
		}
		else if(match_prev && this->rsp_prev_batch_active)
		{
			int loc = abs_seq;
			if(loc >= 0 && loc < this->nMessages)
			{
				char prev_status = messages_rx_prev[loc].status;
				messages_rx_prev[loc].type            = DATA_LONG;
				messages_rx_prev[loc].id              = (char)(unsigned char)loc;
				messages_rx_prev[loc].length          = 16;
				messages_rx_prev[loc].status          = RECEIVED;
				messages_rx_prev[loc].batch_seq_id    = abs_bsi;
				messages_rx_prev[loc].sequence_number = (char)(unsigned char)abs_seq;
				if(prev_status != RECEIVED && prev_status != ACKED)
					this->rsp_prev_batch_received_count++;
			}
			if(abs_bsi == 3) bsi3_retx_routed++;
			// Prev-batch completion: when received_count >= expected_count,
			// the production code performs the swap + delivery + bump (the
			// "prev complete" path). Simulate the bsi-bump effect: prev
			// deactivates. We do NOT bump current_expected here — the prev
			// completion path doesn't either; it just delivers.
			if(this->rsp_prev_batch_active
			   && this->rsp_prev_batch_received_count >= this->rsp_prev_batch_expected_count)
			{
				printf("[RSP-V2-PREV-DELIVER-BEGIN] prev_batch_seq_id=%d "
					"received=%d/%d (synthetic — would deliver here)\n",
					this->rsp_prev_batch_seq_id,
					this->rsp_prev_batch_received_count,
					this->rsp_prev_batch_expected_count);
				fflush(stdout);
				this->rsp_prev_batch_active = false;
			}
		}
	}

	// --- Step 6: post-arrival ACK-GATE on current batch --------------------
	// If the bsi was correctly bumped (OFDM variant), the bsi=4 frames are now
	// in messages_rx[] and the EOB-bearing slot=25 updated
	// last_received_end_of_batch_seq=25 → expected=26, rx_received=26 → PASS,
	// which would fire [RSP-V2-BATCH-DONE]. If the bsi was NOT bumped (MFSK
	// variant), the bsi=4 frames are all dropped, rx_received from messages_rx
	// still reflects the partial bsi=3 (with slot 13 now filled by the retx)
	// → expected=data_batch_size=30 (EOB cleared at step 3), rx_received=25
	// (24 original + 1 retx) → still partial.
	{
		int rx_received = 0;
		for(int i = 0; i < this->data_batch_size; i++)
			if(messages_rx[i].status == RECEIVED) rx_received++;
		int expected = this->data_batch_size;
		if(this->last_received_end_of_batch_seq >= 0)
		{
			int eob = this->last_received_end_of_batch_seq + 1;
			if(eob < expected) expected = eob;
		}
		printf("[ACK-GATE] post-arrival: rx_received=%d expected=%d batch=%d\n",
			rx_received, expected, this->data_batch_size);
		if(rx_received >= expected
		   && this->sack_v2_enabled
		   && this->rsp_current_expected_batch_seq_id >= 0)
		{
			this->rsp_prev_batch_seq_id = this->rsp_current_expected_batch_seq_id;
			this->rsp_current_expected_batch_seq_id =
				(this->rsp_current_expected_batch_seq_id + 1) & 0xFF;
			printf("[RSP-V2-BATCH-DONE] prev=%d next_expected=%d\n",
				this->rsp_prev_batch_seq_id, this->rsp_current_expected_batch_seq_id);
			batch_done_count++;
		}
		fflush(stdout);
	}

	// --- Step 7: verdict ---------------------------------------------------
	// PASS criterion per fact-doc §5.4: drop_count < 2 for bsi=4 arrivals.
	bool pass = (bsi4_drops < 2);
	printf("[TEST-PARTIAL-BSI] %s: transport=%s drop_count=%lld (bsi=4) bsi3_retx_routed=%d "
		"bsi4_routed=%d batch_done_count=%d current_expected=%d prev=%d\n",
		pass ? "PASS" : "FAIL",
		transport, bsi4_drops, bsi3_retx_routed, bsi4_routed, batch_done_count,
		this->rsp_current_expected_batch_seq_id, this->rsp_prev_batch_seq_id);
	fflush(stdout);
	return pass ? 0 : 1;
}
