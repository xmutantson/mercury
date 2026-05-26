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
#include <cstdlib>

#ifdef MERCURY_GUI_ENABLED
#include "gui/gui_state.h"
#endif

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
#define SACK_TRACE(fmt, ...) do { \
	if(sack_rx_trace_enabled()) { \
		printf("[SACK-RX-TRACE] " fmt "\n", ##__VA_ARGS__); \
		fflush(stdout); \
	} \
} while(0)

void cl_arq_controller::register_ack(int message_id)
{
	if(message_id>=0 && message_id<this->nMessages && messages_tx[message_id].status==PENDING_ACK)
	{
		messages_tx[message_id].status=ACKED;
		stats.nAcked_data++;
	}
}


void cl_arq_controller::process_messages_commander()
{
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
		if(receiving_timer.get_elapsed_time_ms() < receiving_timeout)
		{
			if(receive_ack_pattern())
			{
				// Use ROBUST_0 as coordination layer, then probe target config.
				// Phase 1: send SET_CONFIG at ROBUST_0 (guaranteed delivery).
				// Phase 2: send SET_CONFIG at target to verify it works (2 tries).
				int target = config_ladder_down_n(emergency_previous_config, break_drop_step, robust_enabled);
				printf("[BREAK] ACK received! Dropping %d step(s): config %d -> %d (robust_enabled=%d)\n",
					break_drop_step, emergency_previous_config, target, robust_enabled);
				fflush(stdout);
				break_drop_step *= 2;  // 2026-05-24: uncapped doubling
				                       // (was capped at 4). config_ladder_down_n
				                       // clamps at ROBUST_0 / CONFIG_0.

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
						for(int i=nMessages-1; i>=0; i--)
						{
							if(messages_tx[i].status != FREE && messages_tx[i].length > 0)
								fifo_buffer_tx.push_front(messages_tx[i].data, messages_tx[i].length);
							messages_tx[i].status = FREE;
						}
						fifo_buffer_backup.flush();
					}
					block_under_tx = NO;
					int fifo_load = fifo_buffer_tx.get_size() - fifo_buffer_tx.get_free_size();
					printf("[BREAK] Saved data to FIFO (%d bytes total)\n", fifo_load);
					fflush(stdout);
				}

				// Force-clear: cleanup() skips PENDING_ACK status
				messages_control.status = FREE;
				add_message_control(SET_CONFIG);
				printf("[BREAK] SET_CONFIG queued: data[1]=%d data[2]=%d\n",
					(int)messages_control.data[1], (int)messages_control.data[2]);
				fflush(stdout);
				connection_status = TRANSMITTING_CONTROL;
				link_timer.start();
				watchdog_timer.start();
			}
		}
		else
		{
			// Timeout — retry BREAK
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
				emergency_break_active = 0;
				emergency_nack_count = 0;
				break_recovery_phase = 1;
				break_recovery_retries = 2;

				int target = config_ladder_down_n(emergency_previous_config, break_drop_step, robust_enabled);
				printf("[BREAK] Dropping %d step(s): config %d -> %d\n",
					break_drop_step, emergency_previous_config, target);
				fflush(stdout);
				break_drop_step *= 2;  // uncapped — see ACK-received site above

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
					for(int i=nMessages-1; i>=0; i--)
					{
						if(messages_tx[i].status != FREE && messages_tx[i].length > 0)
							fifo_buffer_tx.push_front(messages_tx[i].data, messages_tx[i].length);
						messages_tx[i].status = FREE;
					}
					fifo_buffer_backup.flush();
				}
				block_under_tx = NO;

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
					hail_detected = YES;
					break;
				}
				msleep(50);
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
			// Graceful disconnect (VARA-compatible): wait for TX FIFO to drain
			// before closing the link. Winlink sends FQ data then DISCONNECT;
			// we must deliver the FQ before tearing down the ARQ session.
			int fifo_pending = fifo_buffer_tx.get_size() - fifo_buffer_tx.get_free_size();
			if(fifo_pending > 0 || block_under_tx == YES)
			{
				// Data still pending — let normal data exchange drain it.
				// disconnect_requested stays YES, re-checked next cycle.
				static int disconnect_wait_prints = 0;
				if(disconnect_wait_prints++ % 50 == 0)
				{
					printf("[DISCONNECT] Waiting for TX drain: FIFO=%d bytes, block_under_tx=%d\n",
						fifo_pending, block_under_tx);
					fflush(stdout);
				}
			}
			else
			{
				printf("[DISCONNECT] TX drained, sending CLOSE_CONNECTION\n");
				fflush(stdout);
				disconnect_requested=NO;
				this->link_status=DISCONNECTING;
				messages_control.status=FREE;
				add_message_control(CLOSE_CONNECTION);
			}
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

			std::string str="DISCONNECTED\r";
			tcp_socket_control.message->length=str.length();

			for(int i=0;i<tcp_socket_control.message->length;i++)
			{
				tcp_socket_control.message->buffer[i]=str[i];
			}
			tcp_socket_control.transmit();
		}
	}


	if(this->connection_status==TRANSMITTING_CONTROL)
	{
		print_stats();
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
			printf("[CRYPTO] Turboshift done, initiating key exchange\n");
			fflush(stdout);
			add_message_control(KEY_EXCHANGE_1);
			// connection_status set to TRANSMITTING_CONTROL by add_message_control
			return;
		}
		// Hold data until key exchange completes (encryption negotiated but not yet active)
		if(encryption_enabled && !cipher_suite.is_active())
		{
			return;
		}

		// Phase 3c — consume any pending optimizer-recommended config
		// switch BEFORE starting the next data batch. Re-verify all the
		// hard gates: turboshift may have re-armed and BREAK may have
		// fired since the recommendation was made. If still eligible,
		// queue SET_CONFIG; the standard ACK handler then loads the new
		// config (same pattern as SUPERSHIFT at line ~3193 + turbo path
		// at line ~2785).
		if (opt_pending_switch_cfg >= 0
		    && opt_pending_switch_cfg != current_configuration
		    && !turboshift_active
		    && emergency_break_active == 0
		    && link_status == CONNECTED
		    && is_ofdm_config(current_configuration)
		    && block_under_tx == NO
		    && messages_control.status == FREE)
		{
			int target = opt_pending_switch_cfg;
			// Clamp to mode-appropriate ceiling + max-config override.
			const int mode_ceiling =
				(narrowband_enabled == YES) ? NB_CONFIG_MAX : WB_CONFIG_MAX;
			if (target > mode_ceiling) target = mode_ceiling;
			if (max_config_override >= 0 && target > max_config_override)
				target = max_config_override;
			if (target != current_configuration && is_ofdm_config(target))
			{
				printf("[OPT] queue SET_CONFIG: %d -> %d (effective-rate optimizer)\n",
				       current_configuration, target);
				fflush(stdout);
				negotiated_configuration = target;
				cleanup();
				add_message_control(SET_CONFIG);
				opt_reset_window();
				rate_opt.force_cooldown(5);
				connection_status = TRANSMITTING_CONTROL;
				opt_pending_switch_cfg = -1;
				return;
			}
			// Target became invalid between recording and dispatch — drop.
			opt_pending_switch_cfg = -1;
		}

		print_stats();
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
			messages_control.data[0]=code;
			messages_control.id=0;

			if(gear_shift_algorithm==SNR_BASED)
			{
				forward_configuration = get_configuration(measurements.SNR_downlink);
				reverse_configuration = get_configuration(measurements.SNR_uplink);
			}
			else
			{
				// SUCCESS_BASED_LADDER: asymmetric — only update forward (TX direction)
				forward_configuration = negotiated_configuration;
				// reverse_configuration preserved from other direction's gearshift
				if(reverse_configuration == CONFIG_NONE)
					reverse_configuration = forward_configuration;
			}

			negotiated_configuration = forward_configuration;
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
			if(target_batch > AXIS2_BATCH_CEIL)  target_batch = AXIS2_BATCH_CEIL;

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
		this->connection_status=TRANSMITTING_CONTROL;
	}
	return success;
}

void cl_arq_controller::process_messages_tx_control()
{
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
		}
	}

	if(messages_control.status==ADDED_TO_BATCH_BUFFER)
	{
		// Commander CONTROL TX: full-length frames (responder can't predict frame type)
		telecom_system->set_mfsk_ctrl_mode(false);
		pad_messages_batch_tx(control_batch_size);
		send_batch();

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

		// Recalculate timeout: guard delays from prior ACK detection can leave
		// receiving_timeout stale (e.g. 900ms), too short for the control round-trip.
		calculate_receiving_timeout();
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
				last_transmission_block_stats.nReSent_data = 0;
				last_transmission_block_stats.nSent_data = 0;
			}
			// Always wait for SET_CONFIG ACK (stay in RECEIVING_ACKS_CONTROL).
			// Previously jumped to TRANSMITTING_DATA when negotiated==current,
			// causing collision after BREAK (commander sent data before responder
			// finished processing SET_CONFIG).
		}

		if(messages_control.data[0]==REPEAT_LAST_ACK)
		{
			messages_control.ack_timeout=0;
			messages_control.id=0;
			messages_control.length=0;
			messages_control.nResends=0;
			messages_control.status=FREE;
			messages_control.type=NONE;

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
	if(type==DATA_LONG && length>(max_data_length+max_header_length-effective_data_long_header_length(sack_v2_enabled)))
	{
		success=MESSAGE_LENGTH_ERROR;
		return success;
	}

	// SACK Design A Step 2 — DATA_SHORT payload capacity is reduced by 1 byte
	// when sack_v2_enabled (the new batch_seq_id byte occupies that space).
	// In v1 mode the effective value is identical to the legacy 5-byte macro.
	if(type==DATA_SHORT && length>(max_data_length+max_header_length-effective_data_short_header_length(sack_v2_enabled)))
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

void cl_arq_controller::process_messages_tx_data()
{
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
		connection_status = RECEIVING_ACKS_DATA;
		ack_diag_peak_matched = 0;
		ack_diag_peak_metric = 0.0;
		ack_diag_poll_count = 0;
		v2_ackpat_defer_count_this_window = 0;  // Bug A fix (§7.13.1)
		v2_dispatch_last_rwi = -1;               // §7.13.30 v2 dispatch throttle
		v2_dispatch_min_advance_syms = 1;        // §7.13.30 default throttle interval
		calculate_receiving_timeout();
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
	int v2_retx_prefix_count = 0;
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
			printf("[BREAK] retx queue runaway (count=%d, threshold=2*batch=%d) — "
				"channel collapsed, forcing BREAK\n",
				retransmit_count, 2 * data_batch_size);
			fflush(stdout);
			int working_config = config_ladder_down(data_configuration, robust_enabled);
			// Push pending payloads back to the FIFO for resend after BREAK
			// recovery (parallels the gearshift BREAK path at :2344-2349).
			for(int i=0; i<nMessages; i++)
			{
				if(messages_tx[i].status != FREE && messages_tx[i].length > 0)
					fifo_buffer_tx.push(messages_tx[i].data, messages_tx[i].length);
				messages_tx[i].status = FREE;
			}
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
			message_batch_counter_tx++;
			stats.nReSent_data++;
			last_transmission_block_stats.nReSent_data++;
		}
		v2_retx_prefix_count = R;

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
	bool batch_includes_new_data = false;
	int last_new_data_messages_tx_idx = -1;  // §7.13.39 Fix 3 — for EOB mirror
	for(int i=0;i<this->nMessages;i++)
	{
		if(messages_tx[i].status==ADDED_TO_LIST)
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
				// (retransmit_count was 0), v2_retx_prefix_count == 0, so the
				// helper assigns the same value the struct-copy already
				// provided (id = messages_tx slot index = message_batch_counter_tx).
				if(v2_mixed_batch)
				{
					int pos_in_new_batch = message_batch_counter_tx - v2_retx_prefix_count;
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
			if(last_idx >= v2_retx_prefix_count
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
		// Phase 3a (Effective-Rate Optimizer) — back-fill the prior batch slot's
		// wire_ms with the cycle-time delta, and refresh the tx-start stamp for
		// the batch about to leave. No decision logic here — pure measurement.
		opt_on_batch_tx_start();
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
			int new_data_count = message_batch_counter_tx - v2_retx_prefix_count;
			int retx_bsi = (v2_retx_prefix_count > 0) ? messages_batch_tx[0].batch_seq_id : -1;
			printf("[CMD-V2-MIXBATCH] TX batch: %d retx bsi=%d + %d new bsi=%d = %d total "
				"(data_batch_size=%d)\n",
				v2_retx_prefix_count, retx_bsi,
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
				stats.nReSent_data -= v2_retx_prefix_count;
				last_transmission_block_stats.nReSent_data -= v2_retx_prefix_count;
				int new_data_in_batch = message_batch_counter_tx - v2_retx_prefix_count;
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
		}
		stats.nBatches_sent++;
		last_transmission_block_stats.nBatches_sent++;

		// Post-TX flush handled inside send_batch() via rx_mute.

		if(ack_pattern_time_ms > 0)
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
		connection_status=RECEIVING_ACKS_DATA;
		ack_diag_peak_matched = 0;
		ack_diag_peak_metric = 0.0;
		ack_diag_poll_count = 0;
		v2_ackpat_defer_count_this_window = 0;  // Bug A fix (§7.13.1)
		v2_dispatch_last_rwi = -1;               // §7.13.30 v2 dispatch throttle
		v2_dispatch_min_advance_syms = 1;        // §7.13.30 default throttle interval
		// ACK pattern detection uses dedicated ack_mfsk — no config switch needed
		if(ack_pattern_time_ms <= 0)
			load_configuration(ack_configuration, PHYSICAL_LAYER_ONLY,NO);
		// Recalculate timeout: guard delays from prior ACK detection can leave
		// receiving_timeout stale, too short for the next ACK round-trip.
		calculate_receiving_timeout();
		printf("[CMD-POST-TX] receiving_timeout=%dms msg_tx_time=%dms batch=%d sack=%d\n",
			receiving_timeout, message_transmission_time_ms, data_batch_size, sack_enabled ? 1 : 0);
		fflush(stdout);
		receiving_timer.start();
	}
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
				if(receive_ack_pattern())
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

					// Guard delay: wait for responder to finish ACK TX + settle.
					// ptt_off covers the radio TX→RX transition; +200ms margin.
					int guard = ptt_off_delay_ms + 200;
					receiving_timeout = (int)receiving_timer.get_elapsed_time_ms() + guard;
				}
			}
		}
		else
		{
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

					// Wait for responder to finish remaining ACK batch frames
					{
						int drain = (int)receiving_timer.get_elapsed_time_ms()
							+ (ack_batch_size - 1) * ctrl_transmission_time_ms
							+ ptt_off_delay_ms + 200;
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

				// Start turboshift in NB. Skip when already in the Q-table
				// optimizer's band — the optimizer will pick the right config
				// from current_configuration; no need to overshoot via SUPERSHIFT.
				if(turboshift_active && gear_shift_on == YES &&
					!config_is_at_top(current_configuration, robust_enabled, narrowband_enabled == YES) &&
					!optimizer_is_in_control())
				{
					turboshift_initiator = true;
					turboshift_phase = TURBO_FORWARD;
					turboshift_last_good = current_configuration;
					turbo_snr_ack_enabled = true;
					turbo_received_snr = -99.0f;
					turbo_best_snr = -99.0f;

					int snr_target = -1;
					if(is_ofdm_config(current_configuration) && measurements.SNR_uplink > -90)
					{
						snr_target = get_configuration(measurements.SNR_uplink - SUPERSHIFT_MARGIN_DB);
						int cfg_ceiling = (narrowband_enabled == YES) ? NB_CONFIG_MAX : WB_CONFIG_MAX;
						if(snr_target > cfg_ceiling)
							snr_target = cfg_ceiling;
						if(supershift_proven_ceiling >= 0 && snr_target > supershift_proven_ceiling)
							snr_target = supershift_proven_ceiling;
						if(max_config_override >= 0 && snr_target > max_config_override)
							snr_target = max_config_override;
						// Q-table handoff: turboshift stops where the optimizer takes over.
						apply_optimizer_handoff_cap_to_target(&snr_target);
					}

					if(snr_target > 0 && config_ladder_index(snr_target) > config_ladder_index(current_configuration))
					{
						negotiated_configuration = snr_target;
						printf("[TURBO] Phase: FORWARD — probing commander->responder (NB)\n");
						printf("[TURBO] SNR-SUPERSHIFT: SNR=%.1f dB -> config %d -> %d (direct, ceiling=%d/%d)\n",
							measurements.SNR_uplink, current_configuration, negotiated_configuration, supershift_proven_ceiling, max_config_override);
					}
					else
					{
						negotiated_configuration = config_ladder_up_n(current_configuration, 3, robust_enabled, narrowband_enabled == YES);
						printf("[TURBO] Phase: FORWARD — probing commander->responder (NB)\n");
						printf("[TURBO] SUPERSHIFT: config %d -> %d (step 3)\n",
							current_configuration, negotiated_configuration);
					}
					fflush(stdout);
					cleanup();
					add_message_control(SET_CONFIG);
					connection_status = TRANSMITTING_CONTROL;
				}
				else
				{
					turboshift_active = false;
					turboshift_phase = TURBO_DONE;
					connection_status = TRANSMITTING_DATA;
				}
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

				// TURBO_FORWARD or higher ceiling: BREAK to ROBUST_0, then drop to
				// the SNR-predicted start config (not just 1 step below ceiling).
				// Dropping 1 step at a time wastes time probing configs that can't work.
				turboshift_active = false;
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

					turbo_switch_role_retries = 0;
					turboshift_phase = TURBO_DONE;
					turboshift_active = false;
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

			// Frame gearshift up failure: BREAK immediately, double threshold.
			// Only one attempt — no retries. Recover to the working config and
			// require 2x consecutive ACKs before trying to upshift again.
			if(messages_control.data[0] == SET_CONFIG
				&& turboshift_phase == TURBO_DONE
				&& !turboshift_active
				&& break_recovery_phase == 0
				&& !emergency_break_active)
			{
				gear_shift_timer.stop();
				gear_shift_timer.reset();

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

			// Track control failures toward BREAK threshold.
			// Only during connected data exchange (not connection setup or turboshift).
			if(link_status == CONNECTED && turboshift_phase == TURBO_DONE
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

void cl_arq_controller::process_messages_rx_acks_data()
{
	if (receiving_timer.get_elapsed_time_ms()<receiving_timeout)
	{
		if(ack_pattern_time_ms > 0)
		{
			// Detection strategy: check SACK alongside ACK from the start.
			// We only require a short minimum delay (ack_pattern_time_ms) so
			// the RSP has time to send its response. Step 15: the legacy
			// receive_sack_pattern() correlator is gone — SACK detection is
			// now exclusively the OFDM SACK_RSP decode in the v2 branch below.
			bool sack_window_open = sack_enabled && data_ack_received == NO
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
				if(sack_v2_enabled && axis3_sack_mode != SACK_MODE_OFF)
				{
					// Step 6 of MFSK-suffix ACK+SACK redesign — CMD-side MFSK probe.
					// Before the OFDM dispatch below, sniff the passband tail for an
					// MFSK ACK pattern + 10-symbol SACK suffix carrying (batch_seq_id,
					// 32-bit bitmap). On a clean decode we short-circuit straight to
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

						// CRC12 verification (mercury/fact-documents/mfsk-robust-ack.md §3.2).
						// On mismatch, treat as no-ACK — the timeout-retransmit path
						// is the safe fallback when a corrupted "looks like a clean
						// ACK" frame could otherwise cause silent data loss.
						if(decoded)
						{
							char crc_input[5];
							crc_input[0] = (char)rx_bsi;
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
						}

						if(decoded)
						{
							// Sanity 1: bsi must be the current or just-prior
							// batch (mod 256). RSP only ACKs frames whose
							// batch_seq_id matches one of those.
							unsigned cmd_bsi = (unsigned)(cmd_batch_seq_id & 0xFF);
							unsigned prev_bsi = (cmd_bsi - 1u) & 0xFFu;
							bool bsi_in_window =
								((unsigned)rx_bsi == cmd_bsi || (unsigned)rx_bsi == prev_bsi);
							// Sanity 2: bitmap=0 means "received nothing" — RSP
							// never sends a SACK in that case (no batch_started),
							// so treat as a false decode.
							bool bitmap_ok = (rx_bitmap != 0u);
							// Sanity 3: dedupe vs the last SACK we already applied
							// (mirrors the OFDM SACK_RSP duplicate guard at ~2141).
							bool duplicate = ((int)rx_bsi == cmd_last_applied_sack_bsi);

							if(bsi_in_window && bitmap_ok && !duplicate)
							{
								uint32_t all_ones = (data_batch_size >= 32)
									? 0xFFFFFFFFu
									: ((1u << data_batch_size) - 1u);
								if(rx_bitmap == all_ones)
								{
									// CLEAN BATCH — mirror OFDM_ACK_CLEAN handler
									// (~line 2178). Only state change there is
									// setting v2_ack_pat_pre_detected = true; the
									// fallthrough ACK_PAT block at ~line 2359
									// owns register_ack(), stats, opt_record_batch,
									// policy_evaluate_axis2, axis3_batch_tick, etc.
									v2_ack_pat_pre_detected = true;
									int arrival_ms = (int)receiving_timer.get_elapsed_time_ms();
									printf("[CMD-MFSK-ACK-SACK] CLEAN batch_seq_id=%u (cmd_batch_seq_id=%d) "
										"bitmap=0x%08x matched=%d arrival_ms=%d\n",
										(unsigned)rx_bsi, cmd_batch_seq_id,
										(unsigned)rx_bitmap, mfsk_matched, arrival_ms);
									fflush(stdout);
									mfsk_handled_this_poll = true;
								}
								else
								{
									// PARTIAL BATCH — mirror OFDM SACK_RSP handler
									// (~line 2131). Populate sack_bitmap from the
									// 32-bit bitmap, set sack_detected, record
									// arrival history, dedupe state, axis3 ok event.
									// The big block at ~line 2198 (if(sack_detected))
									// handles retransmit queue / stats / Axis-2
									// state from there.
									for(int i = 0; i < data_batch_size && i < MAX_SACK_BATCH_SIZE; i++)
										sack_bitmap[i] = ((rx_bitmap >> i) & 1u) ? true : false;
									sack_detected = true;
									cmd_last_applied_sack_bsi = (int)rx_bsi;
									int arrival_ms = (int)receiving_timer.get_elapsed_time_ms();
									sack_arrival_history_ms[sack_arrival_history_next_idx] = arrival_ms;
									sack_arrival_history_next_idx =
										(sack_arrival_history_next_idx + 1) % SACK_ARRIVAL_HISTORY;
									if(sack_arrival_history_count < SACK_ARRIVAL_HISTORY)
										sack_arrival_history_count++;
									printf("[CMD-MFSK-ACK-SACK] PARTIAL batch_seq_id=%u (cmd_batch_seq_id=%d) "
										"bitmap=0x%08x matched=%d arrival_ms=%d\n",
										(unsigned)rx_bsi, cmd_batch_seq_id,
										(unsigned)rx_bitmap, mfsk_matched, arrival_ms);
									fflush(stdout);
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
									"in_window=%d bitmap_ok=%d duplicate=%d",
									(unsigned)rx_bsi, cmd_bsi, prev_bsi,
									(unsigned)rx_bitmap,
									bsi_in_window ? 1 : 0,
									bitmap_ok ? 1 : 0,
									duplicate ? 1 : 0);
							}
						}
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
						bool decoded = decode_sack_v2_frame(
							sack_bitmap, data_batch_size, &rx_bsi);
						SACK_TRACE("decode_sack_v2: ok=%d rx_bsi=%u cmd_bsi=%d",
							decoded ? 1 : 0, (unsigned)rx_bsi, cmd_batch_seq_id);
						if(decoded
						   && (int)rx_bsi == cmd_last_applied_sack_bsi)
						{
							printf("[CMD-SACK-V2-DUPLICATE] bsi=%u already applied — ignoring\n",
								(unsigned)rx_bsi);
							fflush(stdout);
							decoded = false;
						}
						if(decoded)
						{
							sack_detected = true;
							cmd_last_applied_sack_bsi = (int)rx_bsi;
							int arrival_ms = (int)receiving_timer.get_elapsed_time_ms();
							sack_arrival_history_ms[sack_arrival_history_next_idx] = arrival_ms;
							sack_arrival_history_next_idx =
								(sack_arrival_history_next_idx + 1) % SACK_ARRIVAL_HISTORY;
							if(sack_arrival_history_count < SACK_ARRIVAL_HISTORY)
								sack_arrival_history_count++;
							printf("[CMD-SACK-V2] decoded SACK_RSP batch_seq_id=%u (cmd_batch_seq_id=%d) arrival_ms=%d — applying to retransmit queue\n",
								(unsigned)rx_bsi, cmd_batch_seq_id, arrival_ms);
							fflush(stdout);
							policy_evaluate_axis3(true);
						}
						else
						{
							policy_evaluate_axis3(false);
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
				for(int i = 0; i < nMessages; i++)
				{
					if(messages_tx[i].status != PENDING_ACK && messages_tx[i].status != ACK_TIMED_OUT)
						continue;

					if(i < data_batch_size && sack_bitmap[i])
					{
						// Frame received by responder — mark ACKED
						opt_sack_bytes_delivered += (unsigned)messages_tx[i].length;
						messages_tx[i].status = ACKED;
						stats.nAcked_data++;
						rx_count++;
					}
					else if(retransmit_count < MAX_RETRANSMIT_HEADROOM)
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
				                 /*failed=*/false);
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
			//     confirmed this poll, recorded via v2_ack_pat_pre_detected;
			// (b) we're OUTSIDE the SACK window — receive_ack_pattern() at
			//     default threshold 7 is fine for non-data ACK paths.
			// Inside the SACK window the lenient threshold is DELIBERATELY
			// avoided: it false-fires on OFDM SACK_RSP body audio (the
			// silent-drop bug). If strict missed but lenient would catch,
			// we let timeout-driven retransmit handle it — better a retx
			// than a silent drop.
			else if(data_ack_received==NO
			        && (v2_ack_pat_pre_detected
			            || (!sack_window_open && receive_ack_pattern())))
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
				stats.nBatches_acked++;
				last_transmission_block_stats.nBatches_acked++;

				// Phase 3a — clean full-batch ACK: every PENDING_ACK / ACK_TIMED_OUT
				// frame is about to be delivered. Sum lengths BEFORE register_ack
				// mutates statuses.
				unsigned long long opt_clean_bytes_delivered = 0;
				for(int i=0; i<nMessages; i++)
				{
					if(messages_tx[i].status==PENDING_ACK || messages_tx[i].status==ACK_TIMED_OUT)
					{
						opt_clean_bytes_delivered += (unsigned)messages_tx[i].length;
						register_ack(i);
					}
				}
				// Record as clean (sack_used=false, failed=false). v2_ack_pat_pre_detected
				// = 1 means we got here via OFDM_ACK_CLEAN; either way the batch
				// closed with NO SACK_RSP cycle, so this is the "clean" bucket.
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

				// Guard delay: wait for responder to finish full ACK TX + settle.
				int guard = ptt_off_delay_ms + 200;
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
				if(messages_rx_buffer.type==ACK_RANGE)
				{
					data_ack_received=YES;
					stats.nBatches_acked++;
					last_transmission_block_stats.nBatches_acked++;
					int start=(unsigned char)messages_rx_buffer.data[0];
					int end=(unsigned char)messages_rx_buffer.data[1];
					// Guard: start > end under garbage frames wraps unsigned char → infinite loop
					if(start <= end)
					{
						for(int i=start;i<=end;i++)
						{
							if(i >= 0 && i < nMessages
							   && messages_tx[i].status == PENDING_ACK)
								opt_ldpc_bytes_delivered += (unsigned)messages_tx[i].length;
							register_ack(i);
						}
					}
					opt_ldpc_ack_fired = true;
				}
				else if(messages_rx_buffer.type==ACK_MULTI)
				{
					data_ack_received=YES;
					stats.nBatches_acked++;
					last_transmission_block_stats.nBatches_acked++;
					// Clamp count to buffer bounds — garbage frames can have data[0]=255,
					// reading past the 200-byte messages_rx_buffer.data (Bug #15)
					int ack_count = (unsigned char)messages_rx_buffer.data[0];
					int max_acks = max_data_length + max_header_length - ACK_MULTI_ACK_RANGE_HEADER_LENGTH - 1;
					if(max_acks < 0) max_acks = 0;
					if(ack_count > max_acks) ack_count = max_acks;
					for(int i=0;i<ack_count;i++)
					{
						int msg_id = (unsigned char)messages_rx_buffer.data[i+1];
						if(msg_id >= 0 && msg_id < nMessages
						   && messages_tx[msg_id].status == PENDING_ACK)
							opt_ldpc_bytes_delivered += (unsigned)messages_tx[msg_id].length;
						register_ack(msg_id);
					}
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
	else if (data_ack_received==NO && ack_pattern_time_ms <= 0 && !(last_message_sent_type==CONTROL && last_message_sent_code==REPEAT_LAST_ACK))
	{
		consecutive_data_acks = 0;  // Reset on failure

		// Frame gearshift just applied but data failed — BREAK immediately, no retry
		if(frame_gearshift_just_applied)
		{
			frame_gearshift_just_applied = false;
			frame_gearshift_retry_count = 0;
			int working_config = config_ladder_down(data_configuration, robust_enabled);
			frame_shift_threshold *= 2;

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
				for(int i=0; i<nMessages; i++)
				{
					if(messages_tx[i].status != FREE && messages_tx[i].length > 0)
						fifo_buffer_tx.push(messages_tx[i].data, messages_tx[i].length);
					messages_tx[i].status = FREE;
				}
				fifo_buffer_backup.flush();
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
		if(ack_pattern_time_ms <= 0)
			load_configuration(data_configuration, PHYSICAL_LAYER_ONLY,YES);
		if (last_message_sent_type==CONTROL && last_message_sent_code==REPEAT_LAST_ACK)
		{
			stats.nNAcked_control++;
		}
		if(data_ack_received == NO && ack_pattern_time_ms > 0)
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
					messages_tx[i].status = ACK_TIMED_OUT;
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

			// Phase 3a — record this batch as failed (no ACK received before
			// receiving_timeout). Whether or not BREAK fires below, the batch
			// itself delivered no bytes; capture that here so each failed batch
			// is recorded exactly once. Bytes delivered = 0, sack_used = false,
			// failed = true. The wire_ms will be back-filled by the NEXT batch's
			// opt_on_batch_tx_start() — or stay 0 if the link terminates here
			// (in which case opt_reset_window() wipes the slot anyway).
			opt_record_batch(/*bytes_delivered=*/0,
			                 /*sack_used=*/false,
			                 /*failed=*/true);
			// Phase 3c — drain the optimizer's cooldown on failure too so it
			// doesn't get stuck after a transient channel hiccup. We do NOT
			// run evaluate() here: failed batches feed noisy zero-rate
			// samples into the window and a switch in the middle of BREAK
			// triage would compete with the recovery logic below.
			rate_opt.notify_cooldown_tick();
			// Cancel any stale pending switch — BREAK / NACK handling owns
			// the next config transition.
			opt_pending_switch_cfg = -1;

			// Frame gearshift just applied but data failed — BREAK immediately.
			// §7.13.33 — retry once before BREAK: on a CFG7→CFG15 PHY-switch,
			// the FIRST batch can fail not because CFG15 is too aggressive
			// but because RSP's OFDM_ACK_CLEAN lands inside CMD's still-muted
			// post-TX window (audioio.c:1255-1258 zeros samples while rx_mute
			// is set). The ACK preamble gets wiped before the decoder sees it,
			// and CMD wrongly concludes the config is unworkable. Retrying
			// the batch once gives RSP a chance to retransmit OFDM_ACK_CLEAN
			// (mechanism-(a) dedup matches same batch_seq_id) at a time when
			// CMD is fully in RX mode. If the retry also fails, BREAK as
			// before — config really is too aggressive.
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
				int working_config = config_ladder_down(data_configuration, robust_enabled);
				frame_shift_threshold *= 2;

				printf("[GEARSHIFT] FRAME UP DATA FAILED (pat): config %d -> BREAK to %d (threshold %d)\n",
					data_configuration, working_config, frame_shift_threshold);
				fflush(stdout);

				for(int i=0; i<nMessages; i++)
				{
					if(messages_tx[i].status != FREE && messages_tx[i].length > 0)
						fifo_buffer_tx.push(messages_tx[i].data, messages_tx[i].length);
					messages_tx[i].status = FREE;
				}
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

			// Count toward emergency BREAK. Batch halving doesn't bypass this.
			emergency_nack_count++;

			// Batch halving disabled (batch size is fixed at negotiated value).
			// Halving causes CMD/RSP batch size mismatch → ACK-GATE desync.
			printf("[BREAK] Block failure #%d at config %d (threshold=%d, batch=%d)\n",
				emergency_nack_count, current_configuration, emergency_nack_threshold, data_batch_size);
			fflush(stdout);

			// Trigger BREAK when threshold reached and not already at bottom
			if(emergency_nack_count >= emergency_nack_threshold
			   && !config_is_at_bottom(current_configuration, robust_enabled)
			   && !emergency_break_active
			   && turboshift_phase == TURBO_DONE
			   && gear_shift_on == YES)
			{
				// Panic-mode jump: if a previous BREAK fired with no data success
				// between, the channel cratered hard and the ladder isn't keeping
				// up. Skip the doubling — set break_drop_step large so the
				// recovery path (arq_commander.cc:81) lands directly on ROBUST_0
				// (config_ladder_down_n clamps to the floor). The line 3074 data-
				// success reset clears the counter; until then every consecutive
				// BREAK bottoms out.
				breaks_since_last_data_success++;
				if(breaks_since_last_data_success >= 2)
				{
					printf("[BREAK-PANIC] %d BREAKs without data success — "
						"forcing jump to ROBUST_0 (break_drop_step=100)\n",
						breaks_since_last_data_success);
					fflush(stdout);
					break_drop_step = 100;  // clamp at floor of ladder
				}

				// Lower ceiling to prevent climbing back to failing config
				int new_ceiling = config_ladder_down(current_configuration, robust_enabled);
				if(supershift_proven_ceiling < 0 || new_ceiling < supershift_proven_ceiling)
				{
					supershift_proven_ceiling = new_ceiling;
					ceiling_success_count = 0;
					printf("[BREAK] Lowered ceiling to %d\n", new_ceiling);
					fflush(stdout);
				}
				printf("[BREAK] Sending emergency BREAK pattern\n");
				fflush(stdout);
				emergency_previous_config = current_configuration;
				emergency_break_active = 1;
				emergency_break_retries = 3;
				// SACK Design A Step 12 — BREAK supremacy (§4.3.4 invariant #6).
				if(sack_v2_enabled)
					policy_axis1_supremacy_on_move(current_configuration,
						current_configuration, "break_block_failure_threshold");
				send_break_pattern();
				// Poll for ACK from responder
				telecom_system->data_container.frames_to_read = 4;
				calculate_receiving_timeout();
				receiving_timer.start();
				return;
			}
		}
		else
		{
			emergency_nack_count = 0;  // Reset on success
			break_drop_step = 2;       // Reset to initial aggression (2 steps).
			breaks_since_last_data_success = 0;  // panic-mode counter resets on
			                                     // real data flow (see arq.h).
			// Don't reset ceiling_success_count here — it accumulates across blocks
			frame_gearshift_just_applied = false;  // upshift survived — clear flag
			frame_gearshift_retry_count = 0;       // §7.13.33 reset
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

		// Frame-level gearshift: after N consecutive successful data ACKs, shift up immediately
		// Respect proven ceiling — don't re-try configs above what turboshift/BREAK verified.
		{
			int proposed_frame = config_ladder_up(current_configuration, robust_enabled, narrowband_enabled == YES);
			bool frame_ceiling_blocked = (supershift_proven_ceiling >= 0 &&
				config_ladder_index(proposed_frame) > config_ladder_index(supershift_proven_ceiling))
				|| (max_config_override >= 0 && proposed_frame > max_config_override);
		// Handoff: above the lowest calibrated Q-table cell, the optimizer
		// is the sole authority for upward config changes. Gearshift's
		// FRAME UP must yield. Downward moves (BREAK) remain available.
		bool optimizer_owns_upward_frame = optimizer_is_in_control();
		if(data_ack_received==YES && gear_shift_on==YES && gear_shift_algorithm==SUCCESS_BASED_LADDER &&
			messages_control.status==FREE &&
			!config_is_at_top(current_configuration, robust_enabled, narrowband_enabled == YES) &&
			!frame_ceiling_blocked &&
			!optimizer_owns_upward_frame)
		{
			consecutive_data_acks++;
			if(consecutive_data_acks >= frame_shift_threshold)
			{
				negotiated_configuration = proposed_frame;
				printf("[GEARSHIFT] FRAME UP: %d consecutive ACKs, config %d -> %d\n",
					consecutive_data_acks, current_configuration, negotiated_configuration);
				fflush(stdout);
				consecutive_data_acks = 0;

				// Put all pending data back into TX FIFO for re-encoding at new config
				if(compression_enabled)
				{
					restore_tx_from_compressed();
				}
				else
				{
					for(int i=nMessages-1; i>=0; i--)
					{
						if(messages_tx[i].status != FREE && messages_tx[i].length > 0)
							fifo_buffer_tx.push_front(messages_tx[i].data, messages_tx[i].length);
						messages_tx[i].status = FREE;
					}
					fifo_buffer_backup.flush();
				}
				block_under_tx = NO;

				add_message_control(SET_CONFIG);
				connection_status = TRANSMITTING_CONTROL;
				return;
			}
		}
		} // frame-level gearshift ceiling scope

		connection_status=TRANSMITTING_DATA;
	}
}

void cl_arq_controller::finish_turbo_direction()
{
	turboshift_active = false;
	turbo_snr_ack_enabled = false;
	// turbo_best_snr preserves the best SNR seen across ALL turbo probes,
	// unlike turbo_received_snr which is reset after each probe step.
	turbo_received_snr = -99.0f;

	if(turboshift_phase == TURBO_FORWARD && skip_turbo_reverse)
	{
		// Forward direction probed. Skip REVERSE probe (--skip-turbo-reverse).
		// Reverse path only needs MFSK ACKs on asymmetric channels.
		turboshift_phase = TURBO_DONE;

		// Turbo probing with single frames is unreliable — it may skip configs
		// (via SNR supershift) leaving gaps in the tested range. Use the SNR
		// measurement as the primary guidance for both ceiling and start config.
		float effective_snr = (turbo_best_snr > -90) ? turbo_best_snr : measurements.SNR_uplink;
		int snr_config = -1;
		if(effective_snr > -90)
			snr_config = get_configuration(effective_snr - SUPERSHIFT_MARGIN_DB);

		// Start at the SUPERSHIFT-verified config (turboshift_last_good).
		// Rationale: the verification probe at line ~2483 just confirmed this
		// config can pass a frame end-to-end. Picking a lower start config
		// (former: snr_config - 2 ladder steps for fading margin) forces an
		// UNNECESSARY PHY-switch to a lower config, which on the CMD side
		// breaks OFDM_ACK_CLEAN detection on the first post-switch batch
		// (gearshift_v11 finding: RSP gets 21/25 frames at CFG13, sends
		// OFDM_ACK_CLEAN, CMD's poll never matches → FRAME UP DATA FAILED →
		// BREAK chain pulls config down to 11). Cost on clean: 7-8× lower
		// throughput vs v22 baseline (2535 vs 343 bps). Trust the probe; if
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
		cleanup();
		add_message_control(SET_CONFIG);
		connection_status = TRANSMITTING_CONTROL;
	}
	else if(turboshift_phase == TURBO_FORWARD)
	{
		// Forward direction probed. Advance to REVERSE (other side will probe).
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
				if(echoed_cap != (unsigned char)local_capability)
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
				handshake_confirmed = true;
				peer_capability = rsp_own;
				// No SNR carried in TEST_CONNECTION_ACK — leave SNR unchanged.
			}
			else
			{
				u_SNR tmp_SNR;
				for(int i=0;i<4;i++)
				{
					tmp_SNR.char4_SNR[i]=messages_control.data[i+1];
				}
				measurements.SNR_downlink=tmp_SNR.f_SNR;

				// Read responder's capability from byte 5.
				// With LDPC ACK: this is the responder's reply (correct).
				// With ACK pattern: no data payload, so this is our own TX data (assumes
				// symmetric capability — works when both sides use same bandwidth_mode).
				// Responder's SWITCH_BANDWIDTH handler rejects if nb_only as a safety net.
				peer_capability = (uint8_t)messages_control.data[5];
			}
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
				// matches the formula in arq_common.cc batch sizing.
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
					this->connection_status=TRANSMITTING_CONTROL;
				}
				// Turboshift: start probing instead of jumping to data. Skip
				// when already in the Q-table optimizer's band (cfg ≥ handoff).
				else if(turboshift_active && gear_shift_on==YES &&
					!config_is_at_top(current_configuration, robust_enabled, narrowband_enabled == YES) &&
					!optimizer_is_in_control())
				{
					turboshift_initiator = true;
					turboshift_phase = TURBO_FORWARD;
					turboshift_last_good = current_configuration;
					turbo_snr_ack_enabled = true;
					turbo_received_snr = -99.0f;
					turbo_best_snr = -99.0f;

					int snr_target = -1;
					if(is_ofdm_config(current_configuration) && measurements.SNR_uplink > -90)
					{
						snr_target = get_configuration(measurements.SNR_uplink - SUPERSHIFT_MARGIN_DB);
						int cfg_ceiling = (narrowband_enabled == YES) ? NB_CONFIG_MAX : WB_CONFIG_MAX;
						if(snr_target > cfg_ceiling)
							snr_target = cfg_ceiling;
						if(supershift_proven_ceiling >= 0 && snr_target > supershift_proven_ceiling)
							snr_target = supershift_proven_ceiling;
						if(max_config_override >= 0 && snr_target > max_config_override)
							snr_target = max_config_override;
						// Q-table handoff: turboshift stops where the optimizer takes over.
						apply_optimizer_handoff_cap_to_target(&snr_target);
					}

					if(snr_target > 0 && config_ladder_index(snr_target) > config_ladder_index(current_configuration))
					{
						negotiated_configuration = snr_target;
						printf("[TURBO] Phase: FORWARD — probing commander->responder\n");
						printf("[TURBO] SNR-SUPERSHIFT: SNR=%.1f dB -> config %d -> %d (direct, ceiling=%d/%d)\n",
							measurements.SNR_uplink, current_configuration, negotiated_configuration, supershift_proven_ceiling, max_config_override);
					}
					else
					{
						negotiated_configuration = config_ladder_up_n(current_configuration, 3, robust_enabled, narrowband_enabled == YES);
						printf("[TURBO] Phase: FORWARD — probing commander->responder\n");
						printf("[TURBO] SUPERSHIFT: config %d -> %d (step 3)\n", current_configuration, negotiated_configuration);
					}
					fflush(stdout);
					cleanup();
					add_message_control(SET_CONFIG);
					this->connection_status=TRANSMITTING_CONTROL;
				}
				else
				{
					turboshift_active = false;
					turboshift_phase = TURBO_DONE;
					this->connection_status=TRANSMITTING_DATA;
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

			if(cipher_suite.compute_x25519_shared((const uint8_t*)&messages_control.data[1]) != 0)
			{
				printf("[CRYPTO] FATAL: X25519 shared secret is zero (low-order point attack?)\n");
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
			tx_batch_counter = 0;
			rx_batch_counter = 0;
			consecutive_auth_failures = 0;
#ifdef MERCURY_GUI_ENABLED
			g_gui_state.encryption_active.store(true);
			g_gui_state.encryption_psk_mismatch.store(false);
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

			// Start turboshift in WB. Skip when in Q-table optimizer band.
			if(turboshift_active && gear_shift_on==YES &&
				!config_is_at_top(current_configuration, robust_enabled, narrowband_enabled == YES) &&
				!optimizer_is_in_control())
			{
				turboshift_initiator = true;
				turboshift_phase = TURBO_FORWARD;
				turboshift_last_good = current_configuration;
				turbo_snr_ack_enabled = true;
				turbo_received_snr = -99.0f;
				turbo_best_snr = -99.0f;

				int snr_target = -1;
				if(is_ofdm_config(current_configuration) && measurements.SNR_uplink > -90)
				{
					snr_target = get_configuration(measurements.SNR_uplink - SUPERSHIFT_MARGIN_DB);
					// Enforce bandwidth ceiling
					int cfg_ceiling = (narrowband_enabled == YES) ? NB_CONFIG_MAX : WB_CONFIG_MAX;
					if(snr_target > cfg_ceiling)
						snr_target = cfg_ceiling;
					if(supershift_proven_ceiling >= 0 && snr_target > supershift_proven_ceiling)
						snr_target = supershift_proven_ceiling;
					if(max_config_override >= 0 && snr_target > max_config_override)
						snr_target = max_config_override;
					// Q-table handoff: turboshift stops where the optimizer takes over.
					apply_optimizer_handoff_cap_to_target(&snr_target);
				}

				if(snr_target > 0 && config_ladder_index(snr_target) > config_ladder_index(current_configuration))
				{
					negotiated_configuration = snr_target;
					printf("[TURBO] Phase: FORWARD — probing commander->responder (post WB upgrade)\n");
					printf("[TURBO] SNR-SUPERSHIFT: SNR=%.1f dB -> config %d -> %d (direct, ceiling=%d/%d)\n",
						measurements.SNR_uplink, current_configuration, negotiated_configuration, supershift_proven_ceiling, max_config_override);
				}
				else
				{
					negotiated_configuration = config_ladder_up_n(current_configuration, 3, robust_enabled, narrowband_enabled == YES);
					printf("[TURBO] Phase: FORWARD — probing commander->responder (post WB upgrade)\n");
					printf("[TURBO] SUPERSHIFT: config %d -> %d (step 3)\n", current_configuration, negotiated_configuration);
				}
				fflush(stdout);
				cleanup();
				add_message_control(SET_CONFIG);
				this->connection_status=TRANSMITTING_CONTROL;
			}
			else
			{
				turboshift_active = false;
				turboshift_phase = TURBO_DONE;
				this->connection_status=TRANSMITTING_DATA;
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
				this->connection_status=TRANSMITTING_DATA;
			}
			// BLOCK_END eliminated — pattern ACK / silence is sole flow control.
			// finalize_block_commander() called directly after data ACK.
			else if (messages_control.data[0]==SWITCH_ROLE)
			{
				turbo_switch_role_retries = 0;  // Reset on success
				// Asymmetric gearshift: swap forward/reverse for the return path
				if(forward_configuration != CONFIG_NONE && reverse_configuration != CONFIG_NONE)
				{
					char tmp = forward_configuration;
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
				msleep(50); // RX_MUTE_GUARD_MS — let in-flight audio callbacks drain
				circular_buf_reset(capture_buffer);
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
					telecom_system->data_container.frames_to_read = rx_frame + 10;
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
							int cfg_ceiling = (narrowband_enabled == YES) ? NB_CONFIG_MAX : WB_CONFIG_MAX;
							if(snr_target > cfg_ceiling)
								snr_target = cfg_ceiling;
							if(supershift_proven_ceiling >= 0 && snr_target > supershift_proven_ceiling)
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
							// No valid SNR: blind step-3
							negotiated_configuration = config_ladder_up_n(current_configuration, 3, robust_enabled, narrowband_enabled == YES);
							printf("[TURBO] SUPERSHIFT: config %d -> %d (step 3, no SNR)\n",
								current_configuration, negotiated_configuration);
						}
						// Enforce WB/NB ceiling on turboshift probe target
						{
							int turbo_cap = (narrowband_enabled == YES) ? NB_CONFIG_MAX : WB_CONFIG_MAX;
							if(negotiated_configuration > turbo_cap)
								negotiated_configuration = turbo_cap;
							if(max_config_override >= 0 && negotiated_configuration > max_config_override)
								negotiated_configuration = max_config_override;
						}
						// Guard: if target config is beyond SNR capability, do not probe.
						// Probing to an undecodable config leaves both sides stuck.
						if(effective_snr > -90)
						{
							int snr_max_cfg = get_configuration(effective_snr);
							if(config_ladder_index(snr_max_cfg) < config_ladder_index(negotiated_configuration))
							{
								printf("[TURBO] SNR %.1f too low for config %d (max=%d), finishing at %d\n",
									effective_snr, negotiated_configuration, snr_max_cfg, current_configuration);
								fflush(stdout);
								turboshift_last_good = current_configuration;
								finish_turbo_direction();
								return;
							}
						}
						fflush(stdout);
						cleanup();
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
						if(turboshift_last_good == current_configuration)
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
					if(turboshift_phase == TURBO_DONE && gear_shift_on == YES &&
						is_ofdm_config(current_configuration) && measurements.SNR_uplink > -90)
					{
						int snr_ideal = get_configuration(measurements.SNR_uplink - SUPERSHIFT_MARGIN_DB);
						if(narrowband_enabled == YES && snr_ideal > NB_CONFIG_MAX)
							snr_ideal = NB_CONFIG_MAX;
						// Enforce proven ceiling from prior BREAK failures
						if(supershift_proven_ceiling >= 0 && snr_ideal > supershift_proven_ceiling)
							snr_ideal = supershift_proven_ceiling;
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

			std::string str="DISCONNECTED\r";
			tcp_socket_control.message->length=str.length();

			for(int i=0;i<tcp_socket_control.message->length;i++)
			{
				tcp_socket_control.message->buffer[i]=str[i];
			}
			tcp_socket_control.transmit();
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

#ifdef MERCURY_GUI_ENABLED
	if(batch_uncompressed_size > 0)
		gui_add_throughput_bytes_tx(batch_uncompressed_size);
	batch_uncompressed_size = 0;
#endif

	// Success rate: use batch-level metric for pattern ACK (all-or-nothing ACK).
	// Frame-level nReSent/nSent is poisoned by ACK-loss retransmissions that aren't
	// real OFDM failures. Batch-level = % of batches that got ACKed.
	if(ack_pattern_time_ms > 0 && last_transmission_block_stats.nBatches_sent > 0)
	{
		last_transmission_block_stats.success_rate_data = 100.0 *
			last_transmission_block_stats.nBatches_acked /
			last_transmission_block_stats.nBatches_sent;
	}
	else if(last_transmission_block_stats.nSent_data > 0)
	{
		last_transmission_block_stats.success_rate_data=100*(1-((float)last_transmission_block_stats.nReSent_data/(float)last_transmission_block_stats.nSent_data));
		if(last_transmission_block_stats.success_rate_data < 0)
			last_transmission_block_stats.success_rate_data = 0;
	}
	else
		last_transmission_block_stats.success_rate_data=100;

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
		if(gear_shift_algorithm==SNR_BASED)
		{
			cleanup();
			add_message_control(TEST_CONNECTION);
		}
		else if(gear_shift_algorithm==SUCCESS_BASED_LADDER)
		{
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
			if(last_transmission_block_stats.success_rate_data >= gear_shift_down_success_rate_precentage)
				gear_shift_down_consecutive_fails = 0;
			// Handoff: above the lowest calibrated Q-table cell, gearshift's
			// LADDER UP must yield to the optimizer (which suggests targets
			// via opt_pending_switch_cfg). Downward moves remain available.
			if(last_transmission_block_stats.success_rate_data>gear_shift_up_success_rate_precentage
				&& gear_shift_blocked_for_nBlocks>= gear_shift_block_for_nBlocks_total
				&& !optimizer_is_in_control())
			{
				{
					int proposed = config_ladder_up(current_configuration, robust_enabled, narrowband_enabled == YES);
				// Respect proven ceiling — don't re-try configs that already failed during turboshift
				bool ceiling_blocked = (supershift_proven_ceiling >= 0 &&
					config_ladder_index(proposed) > config_ladder_index(supershift_proven_ceiling))
					|| (max_config_override >= 0 && proposed > max_config_override);
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
					if(ceiling_blocked)
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
			else if(last_transmission_block_stats.success_rate_data<gear_shift_down_success_rate_precentage)
			{
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
	if(last_transmission_block_stats.success_rate_data >= gear_shift_down_success_rate_precentage)
		gear_shift_down_consecutive_fails = 0;

	// Handoff: above the lowest calibrated Q-table cell, gearshift's
	// LADDER UP must yield to the optimizer. See optimizer_is_in_control().
	if(last_transmission_block_stats.success_rate_data>gear_shift_up_success_rate_precentage
		&& gear_shift_blocked_for_nBlocks>= gear_shift_block_for_nBlocks_total
		&& !optimizer_is_in_control())
	{
		int proposed = config_ladder_up(current_configuration, robust_enabled, narrowband_enabled == YES);
		// Respect proven ceiling — don't re-try configs that already failed during turboshift
		bool ceiling_blocked = (supershift_proven_ceiling >= 0 &&
			config_ladder_index(proposed) > config_ladder_index(supershift_proven_ceiling))
			|| (max_config_override >= 0 && proposed > max_config_override);
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
			if(ceiling_blocked)
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
	else if(last_transmission_block_stats.success_rate_data<gear_shift_down_success_rate_precentage)
	{
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
	(void)from_cfg;
	(void)to_cfg;
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
	                 && (data_batch_size + AXIS2_STEP <= AXIS2_BATCH_CEIL);
	bool want_down = (mean_partial > 0.20f)
	                 && (axis2_consecutive_bad_batches >= AXIS2_DOWN_BAD_RUN)
	                 && (data_batch_size - AXIS2_STEP >= AXIS2_BATCH_FLOOR);

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

	if(want_up || want_down)
	{
		int from = data_batch_size;
		int to = want_up ? (from + AXIS2_STEP) : (from - AXIS2_STEP);
		const char* reason = want_up ? "ring_clean" : "ring_lossy";
		const char* dir    = want_up ? "up"          : "down";

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

		if(want_up) axis2_move_up_count++;
		else        axis2_move_down_count++;

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
		if(want_down)
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
		printf("[POLICY-AXIS2] eval rx=%d/%d partial=%.3f mean=%.3f "
			"good=%d/%d bad=%d/%d batch=%d (no move)\n",
			rx_count, batch_size_observed, partial_rate, mean_partial,
			axis2_consecutive_good_batches, AXIS2_UP_GOOD_RUN,
			axis2_consecutive_bad_batches, AXIS2_DOWN_BAD_RUN,
			data_batch_size);
		fflush(stdout);
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

// SACK Design A Step 10 — synthetic Axis 2 fire (test-only).
//
// CLI: --test-policy-axis2-fire=up|down. Primes the partial-rate ring and
// hysteresis counters at the move threshold then calls
// policy_evaluate_axis2() once with a synthetic (rx_count, batch) pair
// consistent with the requested direction. Demonstrates that the controller
// is wired and that [POLICY-MOVE] axis=2 + the SET_LINK_PARAMS TX fire.
//
// Default builds never enter this; production paths unaffected.
void cl_arq_controller::test_fire_policy_axis2(int direction)
{
	sack_v2_enabled = true;
	// Reset ring + counters to a deterministic starting state.
	for(int i=0;i<AXIS2_RING_DEPTH;i++) axis2_partial_rate_ring[i]=0.0f;
	axis2_partial_rate_count=0;
	axis2_partial_rate_pos=0;
	axis2_consecutive_good_batches=0;
	axis2_consecutive_bad_batches=0;
	axis2_cooldown_batches=0;

	// Pick a deterministic starting batch size mid-range. Bypass
	// set_data_batch_size()'s clamp (which depends on max_data_length +
	// max_header_length having been initialized by a config load — the
	// synthetic test runs before any config is loaded so those are 0 and
	// the clamp produces a nonsense negative value).
	int starting_batch = 25;
	if(starting_batch < AXIS2_BATCH_FLOOR) starting_batch = AXIS2_BATCH_FLOOR;
	if(starting_batch > AXIS2_BATCH_CEIL) starting_batch = AXIS2_BATCH_CEIL;
	data_batch_size = starting_batch;

	if(direction == 1)
	{
		// UP synthetic: prime ring with 5 zero-partial observations + good_run
		// at AXIS2_UP_GOOD_RUN - 1. The single evaluate call will bump good
		// run to AXIS2_UP_GOOD_RUN and trigger an up move.
		for(int i=0;i<AXIS2_RING_DEPTH;i++) axis2_partial_rate_ring[i] = 0.0f;
		axis2_partial_rate_count = AXIS2_RING_DEPTH;
		axis2_consecutive_good_batches = AXIS2_UP_GOOD_RUN - 1;
		printf("[TEST-AXIS2-FIRE] direction=up: ring primed clean (mean=0), "
			"good_run=%d/%d, batch=%d → expect UP\n",
			axis2_consecutive_good_batches, AXIS2_UP_GOOD_RUN, data_batch_size);
		fflush(stdout);
		// Synthetic observation: full batch RX (rx_count == batch_size).
		policy_evaluate_axis2(starting_batch, starting_batch);
	}
	else if(direction == 2)
	{
		// DOWN synthetic: prime ring with 5 high-partial observations + bad_run
		// at AXIS2_DOWN_BAD_RUN - 1. The single evaluate call will bump bad
		// run to AXIS2_DOWN_BAD_RUN and trigger a down move.
		for(int i=0;i<AXIS2_RING_DEPTH;i++) axis2_partial_rate_ring[i] = 0.4f;
		axis2_partial_rate_count = AXIS2_RING_DEPTH;
		axis2_consecutive_bad_batches = AXIS2_DOWN_BAD_RUN - 1;
		printf("[TEST-AXIS2-FIRE] direction=down: ring primed lossy (mean=0.4), "
			"bad_run=%d/%d, batch=%d → expect DOWN\n",
			axis2_consecutive_bad_batches, AXIS2_DOWN_BAD_RUN, data_batch_size);
		fflush(stdout);
		// Synthetic observation: rx_count = batch * 0.6 → partial_rate = 0.4
		int synth_rx = (int)(starting_batch * 0.6f);
		policy_evaluate_axis2(synth_rx, starting_batch);
	}
	else
	{
		printf("[TEST-AXIS2-FIRE] direction=%d not in {1=up, 2=down}; no-op\n",
			direction);
		fflush(stdout);
	}
}

// SACK Design A Step 12 — synthetic Axis-2 proven-ceiling fire (test-only).
//
// CLI: --test-policy-axis2-ceiling-fire=1. Demonstrates §4.3.4 invariant #7
// end-to-end:
//   1. Prime an Axis-2 down-move from 25 → 20 (lossy ring). After the move,
//      batch_size_proven_ceiling = 24 (= from-1 = 25-1) and recovery=20.
//   2. Bring the channel back to clean (ring all zeros, good_run = 8).
//   3. Attempt a synthetic Axis-2 up-move from 20 → 25 (proposed=25 >
//      ceiling=24) → MUST be VETOED by the ceiling check.
//   4. Confirm `axis2_ceiling_blocks_count` incremented exactly once and
//      the [POLICY-AXIS2-CEILING] up-move VETOED log line fires.
//
// Default builds never call this; production paths unaffected.
void cl_arq_controller::test_fire_policy_axis2_ceiling()
{
	sack_v2_enabled = true;
	gear_shift_on = YES;
	gear_shift_algorithm = SUCCESS_BASED_LADDER;

	// Reset Axis-2 state to clean baseline.
	for(int i=0;i<AXIS2_RING_DEPTH;i++) axis2_partial_rate_ring[i]=0.0f;
	axis2_partial_rate_count=0;
	axis2_partial_rate_pos=0;
	axis2_consecutive_good_batches=0;
	axis2_consecutive_bad_batches=0;
	axis2_cooldown_batches=0;
	batch_size_proven_ceiling=-1;
	batch_size_ceiling_recovery_batches=0;
	axis2_ceiling_blocks_count=0;
	data_batch_size = 25;

	// Step 1 — synthetic down-move from 25.
	// NOTE: set_data_batch_size() clamps against (max_data_length +
	// max_header_length - ...) which is 0 in pre-init synthetic mode and
	// produces a nonsense negative value. We bypass that clamp by restoring
	// data_batch_size manually after policy_evaluate_axis2(). The ceiling
	// state itself is what's load-bearing for this test — not the actual
	// in-memory batch size value.
	printf("[TEST-AXIS2-CEILING] step 1: priming lossy ring at batch=25 → "
		"expect [POLICY-MOVE] axis=2 from=25 to=20 AND ceiling set to 24\n");
	fflush(stdout);
	for(int i=0;i<AXIS2_RING_DEPTH;i++) axis2_partial_rate_ring[i] = 0.4f;
	axis2_partial_rate_count = AXIS2_RING_DEPTH;
	axis2_consecutive_bad_batches = AXIS2_DOWN_BAD_RUN - 1;
	int synth_rx = (int)(25 * 0.6f);
	policy_evaluate_axis2(synth_rx, 25);
	// Restore data_batch_size to 20 (the value the down-move WOULD have set
	// in production) — bypassing the set_data_batch_size() clamp bug.
	data_batch_size = 20;
	long long ceiling_blocks_before = axis2_ceiling_blocks_count;
	int ceiling_after_step1 = batch_size_proven_ceiling;
	int recovery_after_step1 = batch_size_ceiling_recovery_batches;
	int batch_after_step1 = data_batch_size;
	printf("[TEST-AXIS2-CEILING] step 1 result: data_batch_size=%d (forced to 20 "
		"to bypass pre-init clamp bug) proven_ceiling=%d recovery=%d "
		"ceiling_blocks=%lld\n",
		batch_after_step1, ceiling_after_step1, recovery_after_step1,
		(long long)axis2_ceiling_blocks_count);
	fflush(stdout);

	// Step 2 — prime clean ring + good_run at threshold for up-move.
	// The up-move would propose 20 + AXIS2_STEP = 25, which exceeds
	// proven_ceiling=24 → MUST be VETOED.
	printf("[TEST-AXIS2-CEILING] step 2: priming clean ring at batch=%d, "
		"good_run=%d → expect up-move to %d to be VETOED by ceiling=%d\n",
		batch_after_step1, AXIS2_UP_GOOD_RUN,
		batch_after_step1 + AXIS2_STEP, ceiling_after_step1);
	fflush(stdout);
	for(int i=0;i<AXIS2_RING_DEPTH;i++) axis2_partial_rate_ring[i] = 0.0f;
	axis2_partial_rate_count = AXIS2_RING_DEPTH;
	axis2_consecutive_good_batches = AXIS2_UP_GOOD_RUN - 1;
	axis2_consecutive_bad_batches = 0;
	// Synthetic clean observation: rx_count = batch_size_observed (partial=0).
	policy_evaluate_axis2(batch_after_step1, batch_after_step1);
	long long ceiling_blocks_after = axis2_ceiling_blocks_count;
	int batch_after_step2 = data_batch_size;
	bool veto_ok = (batch_after_step2 == batch_after_step1
	                && ceiling_blocks_after == ceiling_blocks_before + 1);
	printf("[TEST-AXIS2-CEILING] step 2 result: data_batch_size=%d (was %d) "
		"ceiling_blocks=%lld (was %lld) — %s\n",
		batch_after_step2, batch_after_step1,
		(long long)axis2_ceiling_blocks_count, ceiling_blocks_before,
		veto_ok ? "PASS: up-move VETOED, batch unchanged, ceiling-block counter +1"
		        : "FAIL: expected up-move to be vetoed by ceiling");
	fflush(stdout);

	// Step 3 — confirm Axis-1 supremacy clears the ceiling.
	printf("[TEST-AXIS2-CEILING] step 3: invoking Axis-1 supremacy "
		"(reason=ladder_test) — expect proven_ceiling reset to -1\n");
	fflush(stdout);
	policy_axis1_supremacy_on_move(5, 6, "ladder_test_ceiling_reset");
	printf("[TEST-AXIS2-CEILING] step 3 result: proven_ceiling=%d recovery=%d — %s\n",
		batch_size_proven_ceiling, batch_size_ceiling_recovery_batches,
		(batch_size_proven_ceiling == -1 && batch_size_ceiling_recovery_batches == 0)
		? "PASS: ceiling cleared by Axis-1 supremacy"
		: "FAIL: expected ceiling cleared");
	fflush(stdout);
}

// SACK Design A Step 12 — synthetic BREAK supremacy fire (test-only).
//
// CLI: --test-policy-break-supremacy=1. Demonstrates §4.3.4 invariant #6
// for the BREAK code path:
//   1. Prime Axis-2 ring + good_run to threshold so an Axis-2 up-move would
//      normally fire on the next evaluation.
//   2. Invoke the supremacy hook with a synthetic BREAK reason tag (the
//      same call the new BREAK init sites make in production).
//   3. Confirm [POLICY-SUPREMACY] log emits with reason=break_synthetic AND
//      that Axis-2 ring + counters are cleared AND that 3 subsequent
//      Axis-2 evaluations are SUPPRESSED by the cooldown.
//
// Default builds never call this; production paths unaffected.
void cl_arq_controller::test_fire_policy_break_supremacy()
{
	sack_v2_enabled = true;
	gear_shift_on = YES;
	gear_shift_algorithm = SUCCESS_BASED_LADDER;
	current_configuration = 5;
	negotiated_configuration = 5;

	// Step 1 — prime Axis-2 + Axis-3 to non-default state.
	printf("[TEST-BREAK-SUPREMACY] step 1: priming Axis-2 ring + Axis-3 mode "
		"so supremacy reset is observable\n");
	fflush(stdout);
	for(int i=0;i<AXIS2_RING_DEPTH;i++) axis2_partial_rate_ring[i] = 0.1f;
	axis2_partial_rate_count = AXIS2_RING_DEPTH;
	axis2_partial_rate_pos = 2;
	axis2_consecutive_good_batches = AXIS2_UP_GOOD_RUN - 1;
	axis2_consecutive_bad_batches = 0;
	axis2_cooldown_batches = 0;
	batch_size_proven_ceiling = 20;
	batch_size_ceiling_recovery_batches = 15;
	axis3_sack_mode = SACK_MODE_ON;
	for(int i=0;i<AXIS3_RING_DEPTH;i++) axis3_recent_sack_ok[i] = true;
	axis3_recent_sack_ok_count = AXIS3_RING_DEPTH;
	axis3_consecutive_sack_misses = 0;
	axis3_cooldown_batches = 0;
	data_batch_size = 25;

	// Step 2 — invoke supremacy hook with a synthetic BREAK reason.
	printf("[TEST-BREAK-SUPREMACY] step 2: invoking supremacy hook "
		"(reason=break_synthetic) — expect [POLICY-SUPREMACY] reason=break_synthetic\n");
	fflush(stdout);
	policy_axis1_supremacy_on_move(current_configuration,
		current_configuration, "break_synthetic");
	printf("[TEST-BREAK-SUPREMACY] post-hook state: axis2_cooldown=%d "
		"axis3_cooldown=%d axis3_mode=%s proven_ceiling=%d recovery=%d\n",
		axis2_cooldown_batches, axis3_cooldown_batches,
		(axis3_sack_mode==SACK_MODE_OFF) ? "OFF"
		: (axis3_sack_mode==SACK_MODE_ON) ? "ON"
		: (axis3_sack_mode==SACK_MODE_PROBE) ? "PROBE" : "?",
		batch_size_proven_ceiling, batch_size_ceiling_recovery_batches);
	fflush(stdout);

	// Step 3 — attempt 3 synthetic Axis-2 evaluations during cooldown.
	// Re-prime good_run to threshold each time; cooldown MUST suppress the
	// move. We use clean observations (partial=0) so the only reason no
	// [POLICY-MOVE] fires is the cooldown gate (not a missing threshold).
	for(int i=1;i<=3;i++)
	{
		axis2_consecutive_good_batches = AXIS2_UP_GOOD_RUN;
		printf("[TEST-BREAK-SUPREMACY] step 3.%d: attempting Axis-2 eval — "
			"expect SUPPRESSED (cooldown_remaining=%d on entry)\n",
			i, axis2_cooldown_batches);
		fflush(stdout);
		policy_evaluate_axis2(data_batch_size, data_batch_size);
	}
	printf("[TEST-BREAK-SUPREMACY] complete: axis2_skipped_in_cooldown=%lld "
		"axis2_move_up_count=%lld axis2_move_down_count=%lld\n",
		axis2_skipped_in_cooldown, axis2_move_up_count, axis2_move_down_count);
	fflush(stdout);
}

// SACK Design A Step 9 — synthetic Axis 1 fire (test-only entry point).
//
// CLI: --test-policy-axis1-fire=up|down. Primes the LADDER state with a
// synthetic observable and the hysteresis counters at the move threshold,
// then calls policy_evaluate_axis1() once. Demonstrates that the wrapper
// is wired and that [POLICY-MOVE] + [POLICY-SUPREMACY] fire on a real
// modulation move.
//
// Default builds never enter this function; production paths are
// unaffected. Implemented inside cl_arq_controller so the priming touches
// the private state members directly (last_transmission_block_stats,
// gear_shift_blocked_for_nBlocks, etc.) without weakening encapsulation.
void cl_arq_controller::test_fire_policy_axis1(int direction)
{
	// Force the v2-gated branch so policy_evaluate_axis1() is reached.
	sack_v2_enabled = true;
	gear_shift_on = YES;
	gear_shift_algorithm = SUCCESS_BASED_LADDER;
	// Start mid-ladder so both directions are reachable.
	current_configuration = 4;
	negotiated_configuration = 4;
	robust_enabled = NO;
	narrowband_enabled = NO;
	max_config_override = -1;
	supershift_proven_ceiling = -1;
	ceiling_success_count = 0;

	if(direction == 1)
	{
		// LADDER UP synthetic: 100% success, block-counter at threshold.
		last_transmission_block_stats.success_rate_data = 100.0f;
		gear_shift_blocked_for_nBlocks = gear_shift_block_for_nBlocks_total;
		gear_shift_down_consecutive_fails = 0;
		printf("[TEST-AXIS1-FIRE] direction=up: success_rate=100%%, "
			"blocked_for=%d (threshold=%d) → expect LADDER UP from %d\n",
			gear_shift_blocked_for_nBlocks, gear_shift_block_for_nBlocks_total,
			current_configuration);
	}
	else if(direction == 2)
	{
		// LADDER DOWN synthetic: 0% success, consecutive_fails primed to
		// (threshold - 1) — the wrapper increments then compares against 3.
		last_transmission_block_stats.success_rate_data = 0.0f;
		gear_shift_blocked_for_nBlocks = 0;
		gear_shift_down_consecutive_fails = 2;  // wrapper bumps to 3 → triggers move
		printf("[TEST-AXIS1-FIRE] direction=down: success_rate=0%%, "
			"consecutive_fails will bump 2→3 → expect LADDER DOWN from %d\n",
			current_configuration);
	}
	else
	{
		printf("[TEST-AXIS1-FIRE] direction=%d not in {1=up, 2=down}; no-op\n", direction);
		return;
	}
	fflush(stdout);
	policy_evaluate_axis1();
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
			return;

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
		if( fifo_buffer_tx.get_size()!=fifo_buffer_tx.get_free_size() && stage_ok)
		{
			// SACK Design A Step 1 — effective DATA_LONG header drives per-frame
			// payload budget. In v1 (default) identical to legacy macro; in v2
			// loses 1 byte to the batch_seq_id field.
			int max_frame = max_data_length+max_header_length-effective_data_long_header_length(sack_v2_enabled);

			if(compression_enabled)
			{
				// --- Batch-level compression with adaptive sizing ---
				// Pop raw data based on estimated compression ratio, compress,
				// iteratively add more data until batch is 85%+ full.
				int batch_capacity = data_batch_size * max_frame;

				// Reserve space for encryption auth tag if active
				int crypto_overhead = 0;
				if(cipher_suite.is_active())
					crypto_overhead = AUTH_TAG_SIZE;
				batch_capacity -= crypto_overhead;

				// Initial guess based on running ratio estimate.
				// Staging can be up to COMPRESS_WORKSPACE_SIZE (64KB) because
				// high-ratio compressors (zstd/PPMd) can shrink 33KB+ to <1.5KB.
				const int staging_max = 65535;  // Header orig_size is uint16; cap to prevent overflow
				int chdr_size = compressor.get_header_size();
				int initial_pop = (int)(batch_capacity * compress_ratio_estimate);
				if(initial_pop > staging_max) initial_pop = staging_max;
				if(initial_pop < batch_capacity - chdr_size)
					initial_pop = batch_capacity - chdr_size;

				char staging[COMPRESS_WORKSPACE_SIZE];
				int raw_size = fifo_buffer_tx.pop(staging, initial_pop);
				if(raw_size == 0)
				{
					last_transmission_block_stats.nSent_data=0;
					last_transmission_block_stats.nReSent_data=0;
				}
				else
				{
					char comp_buf[16384];
					int comp_size = 0;
					bool compress_ok = false;

					// Adaptive fill loop: compress, check fill ratio, add more.
					// Streaming mode: single compress call (no retry). The PPMd
					// model advances on each compress_block call. Retrying with
					// different data sizes causes TX/RX model desync because RX
					// only decompresses the final data once.
					int max_iter = compressor.is_streaming() ? 1 : 4;
					for(int iter = 0; iter < max_iter; iter++)
					{
						comp_size = compressor.compress_block(
							staging, raw_size, comp_buf, batch_capacity);

						if(comp_size > 0)
						{
							// Compression succeeded — check fill ratio
							float fill_ratio = (float)comp_size / (float)batch_capacity;
							if(fill_ratio >= 0.85f || iter == max_iter - 1)
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
							// Doesn't fit — push back excess and accept as-is
							if(compressor.is_streaming())
							{
								// Streaming: push back 40%, use raw (model already advanced)
								int pushback = raw_size * 2 / 5;
								if(pushback < 1) pushback = 1;
								fifo_buffer_tx.push_front(
									staging + raw_size - pushback, pushback);
								raw_size -= pushback;
								// Model is desynced — reset streaming
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
						// Push algo to GUI (read from compressed header byte 0)
						g_gui_state.compression_algo.store((int)(unsigned char)comp_buf[0]);
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

						// Pad compressed data so encrypted output fills all
						// data_batch_size frames.  The ACK gate can't peek at
						// the compression header when it's encrypted, so it
						// expects data_batch_size unique frames.  Without
						// padding, compression may produce fewer frames and
						// the duplicate-ID padding causes ACK gate suppression.
						// Receiver decrypts, reads comp_size from header, and
						// ignores the zero padding beyond it.
						int target_comp = data_batch_size * max_frame - tag_size;
						if(comp_size < target_comp && target_comp <= (int)sizeof(comp_buf))
						{
							memset(comp_buf + comp_size, 0, target_comp - comp_size);
							comp_size = target_comp;
						}

						uint32_t tx_direction = (original_role == COMMANDER)
							? DIRECTION_CMD_TO_RSP : DIRECTION_RSP_TO_CMD;
						printf("[CRYPTO-TX] Encrypting %d bytes, counter=%llu dir=%u tag=%d config=%d\n",
							comp_size, (unsigned long long)tx_batch_counter,
							tx_direction, tag_size, (int)data_configuration);
						fflush(stdout);
						char enc_buf[16384];
						int enc_size = cipher_suite.encrypt(
							(const uint8_t*)comp_buf, comp_size,
							(uint8_t*)enc_buf, sizeof(enc_buf),
							tx_batch_counter, tx_direction,
							tag_size);
						if(enc_size > 0)
						{
							memcpy(comp_buf, enc_buf, enc_size);
							comp_size = enc_size;
							tx_batch_counter++;
						}
						else
						{
							printf("[CRYPTO] Encrypt failed (batch %llu)\n",
								(unsigned long long)tx_batch_counter);
							fflush(stdout);
						}
					}

					// With encryption, comp_buf is padded + encrypted to fill
					// all data_batch_size frames exactly (no duplicate IDs).
					// Without encryption, pad_messages_batch_tx() fills
					// remaining batch slots with duplicates.

					// Ensure contiguous IDs 0..data_batch_size-1
					for(int i = 0; i < data_batch_size; i++)
					{
						if(messages_tx[i].status != FREE)
							messages_tx[i].status = FREE;
					}

					// Split comp_buf into frames
					int pos = 0;
					int frame_num = 0;
					while(pos < comp_size)
					{
						int chunk = comp_size - pos;
						if(chunk > max_frame) chunk = max_frame;
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
					printf("[COMPRESS-TX] %d raw -> %d comp (%d frames), ratio_est=%.2f, fill=%.0f%%\n",
						raw_size, comp_size, frame_num, compress_ratio_estimate,
						100.0f * comp_size / batch_capacity);
					fflush(stdout);
				}
			}
			else
			{
				// --- No compression: original per-frame loop ---
				int filled = 0;
				int fill_limit = data_batch_size;
				batch_uncompressed_size = 0;
				for(int i=0;i<fill_limit;i++)
				{
					data_read_size=fifo_buffer_tx.pop(message_TxRx_byte_buffer, max_frame);
					if(data_read_size==0)
					{
						last_transmission_block_stats.nSent_data=0;
						last_transmission_block_stats.nReSent_data=0;
						break;
					}
					fifo_buffer_backup.push(message_TxRx_byte_buffer, data_read_size);
					batch_uncompressed_size += data_read_size;
#ifdef MERCURY_GUI_ENABLED
					// Monitor tap: plaintext (no compression path)
					gui_push_monitor_text(message_TxRx_byte_buffer, data_read_size, true);
#endif
					block_under_tx=YES;
					if(data_read_size==max_frame)
						add_message_tx_data(DATA_LONG, data_read_size, message_TxRx_byte_buffer);
					else
						add_message_tx_data(DATA_SHORT, data_read_size, message_TxRx_byte_buffer);
					filled++;
				}
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
