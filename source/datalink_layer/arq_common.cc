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
#include "audioio/audioio.h"
#include "debug/canary_guard.h"
#include "common/timing_log.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <chrono>

extern "C" {
    extern double noise_snr_db;
    extern double noise_signal_dbfs;
}

#ifdef MERCURY_GUI_ENABLED
#include "gui/gui_state.h"
#endif

extern cbuf_handle_t capture_buffer;
extern cbuf_handle_t playback_buffer;

static const int RX_MUTE_GUARD_MS = 50;

cl_arq_controller::cl_arq_controller()
{
	connection_status=IDLE;
	link_status=IDLE;
	nMessages=0;
	max_message_length=0;
	max_data_length=0;
	max_header_length=0;

	messages_last_ack_bu.data=NULL;
	messages_control.data=NULL;
	messages_rx_buffer.data=NULL;
	messages_tx=NULL;
	messages_rx=NULL;
	messages_batch_tx=NULL;
	messages_batch_ack=NULL;
	message_TxRx_byte_buffer=NULL;

	message_batch_counter_tx=0;
	ack_timeout_data=1000;
	ack_timeout_control=1000;
	link_timeout=10000;
	watchdog_timeout=1000;
	receiving_timeout=10000;
	switch_role_timeout=1000;
	switch_role_test_timeout=1000;
	gearshift_timeout=1000;
	connection_timeout=30000;
	nResends=3;
	stats.nSent_data=0;
	stats.nAcked_data=0;
	stats.nReceived_data=0;
	stats.nLost_data=0;
	stats.nReSent_data=0;
	stats.nAcks_sent_data=0;
	stats.nBatches_sent=0;
	stats.nBatches_acked=0;
	stats.nNAcked_data=0;

	stats.nSent_control=0;
	stats.nAcked_control=0;
	stats.nReceived_control=0;
	stats.nLost_control=0;
	stats.nReSent_control=0;
	stats.nAcks_sent_control=0;
	stats.nNAcked_control=0;
	stats.success_rate_data=0;


	last_transmission_block_stats.nSent_data=0;
	last_transmission_block_stats.nAcked_data=0;
	last_transmission_block_stats.nReceived_data=0;
	last_transmission_block_stats.nLost_data=0;
	last_transmission_block_stats.nReSent_data=0;
	last_transmission_block_stats.nAcks_sent_data=0;
	last_transmission_block_stats.nBatches_sent=0;
	last_transmission_block_stats.nBatches_acked=0;
	last_transmission_block_stats.nNAcked_data=0;

	last_transmission_block_stats.nSent_control=0;
	last_transmission_block_stats.nAcked_control=0;
	last_transmission_block_stats.nReceived_control=0;
	last_transmission_block_stats.nLost_control=0;
	last_transmission_block_stats.nReSent_control=0;
	last_transmission_block_stats.nAcks_sent_control=0;
	last_transmission_block_stats.nNAcked_control=0;
	last_transmission_block_stats.success_rate_data=0;

	measurements.SNR_uplink=-99.9;
	measurements.SNR_downlink=-99.9;
	measurements.signal_stregth_dbm=-99.9;
	measurements.frequency_offset=-99.9;

	data_batch_size=1;
	nominal_batch_size=1;
	batch_consec_acks=0;
	control_batch_size=1;
	ack_batch_size=1;
	batch_rx_frame_count=0;
	batch_data_delivered=false;

	// SACK state initialization
	sack_enabled=false;
	sack_v2_enabled=false;   // Step 6 scaffolding (negotiate-only; gates nothing yet)
	enable_sack_v2=false;    // Step 6 scaffolding (CLI opt-in; default off → byte 5 unchanged)
	radio_batch_size=25;
	crypto_batch_size=20;
	retransmit_headroom=5;
	crypto_batch_counter_tx=0;
	crypto_batch_counter_rx=0;
	retransmit_count=0;
	sack_retransmit_active=false;
	retransmit_batch_id=-1;
	for(int i=0;i<MAX_RETRANSMIT_HEADROOM;i++) {
		retransmit_frame_lengths[i]=0;
		retransmit_frame_positions[i]=0;
		retransmit_frame_types[i]=0;
		retransmit_frame_batch_seq_ids[i]=-1;  // Step 3: unset until SACK populates
	}
	// SACK Design A Step 3 — batch_seq_id state (CMD counter, RSP store).
	// cmd_batch_seq_id starts at 0; first new-data batch goes out under value 0.
	// captured_batch_seq_id_for_retransmit is -1 (no SACK retransmit pending).
	// last_received_batch_seq_id is -1 (no DATA frame parsed yet).
	cmd_batch_seq_id=0;
	captured_batch_seq_id_for_retransmit=-1;
	last_received_batch_seq_id=-1;
	// SACK Design A Step 4 — RSP cross-batch routing state. All gated on
	// sack_v2_enabled; v1 path leaves these at their sentinels. Per §4.2.3,
	// `current_expected` is adopted from the first v2 DATA frame received;
	// `prev` stays -1 until the first ACK-GATE-PASS. `drop_count` tracks
	// [RSP-V2-DROP] events for test assertions.
	rsp_current_expected_batch_seq_id=-1;
	rsp_prev_batch_seq_id=-1;
	rsp_v2_drop_count=0;
	test_rsp_bsi_corrupt_at=0;
	test_rsp_bsi_v2_frame_counter=0;
	// SACK Design A Step 7 — SACK_RSP OFDM control-frame counters.
	// All start at 0 / -1 / false sentinels; only written when sack_v2_enabled.
	rsp_sack_v2_tx_count=0;
	cmd_sack_v2_rx_count=0;
	cmd_sack_v2_crc_fail_count=0;
	for(int i=0; i<(int)sizeof(cmd_sack_v2_last_rx_bitmap); i++)
		cmd_sack_v2_last_rx_bitmap[i]=0;
	cmd_sack_v2_last_rx_nbits=-1;
	cmd_sack_v2_last_rx_batch_seq_id=-1;
	test_rsp_sack_rsp_crc_corrupt=false;
	test_rsp_sack_rsp_crc_corrupt_armed=false;
	crypto_buf[0].clear();
	crypto_buf[1].clear();
	message_transmission_time_ms=500;
	ctrl_transmission_time_ms=500;
	ack_pattern_time_ms=0;
	role=RESPONDER;
	original_role=RESPONDER;
	connection_id=0;
	assigned_connection_id=0;
	block_ready=0;
	block_under_tx=NO;
	ack_batch_size=1;
	my_call_sign="";
	destination_call_sign="";
	user_command_buffer="";
	telecom_system=NULL;

	print_stats_frequency_hz=2;

	init_configuration=CONFIG_0;
	current_configuration=CONFIG_0;
	negotiated_configuration=CONFIG_0;
	last_data_configuration=CONFIG_0;
	ack_configuration=CONFIG_0;
	data_configuration=CONFIG_0;
	forward_configuration=CONFIG_NONE;
	reverse_configuration=CONFIG_NONE;

	gear_shift_on=NO;
	robust_enabled=NO;
	narrowband_enabled=NO;
	commander_configured_nb=-1;
	nb_probe_max=2;
	session_narrowband=false;
	bandwidth_mode=BW_AUTO;
	disable_sack=true;  // B2 fix (2026-05-12): SACK off by default. See arq.h.
	force_sack_ldpc_fail=false;  // CLI --test-sack-ldpc-fail repro toggle (off in production)
	local_capability=CAP_COMPRESSION | CAP_B2F_UNROLL | CAP_STREAMING | CAP_SACK;
	if(disable_sack) local_capability &= ~CAP_SACK;
	if(enable_sack_v2) local_capability |= CAP_SACK_V2;  // Step 6 scaffolding (default-off opt-in)
	peer_capability=0;
	wb_upgrade_pending=false;
	psk_mismatch_pending=false;
	compression_enabled=false;
	force_compress=false;
	b2f_compression_pending=false;
	compress_ratio_estimate=2.0f;
	batch_uncompressed_size=0;
	encryption_mode=ENCRYPT_OFF;
	encryption_enabled=false;
	tx_batch_counter=0;
	rx_batch_counter=0;
	consecutive_auth_failures=0;
	kx_data_buf=NULL;
	kx_data_len=0;
	memset(psk_hex, 0, sizeof(psk_hex));
	passive_monitor=false;
	monitor_stdout=false;
	monitor_consec_ofdm_fail=0;
	for(int i=0; i<NUMBER_OF_CONFIGS; i++) monitor_decoders[i]=NULL;
	monitor_decoders_ready=false;
	monitor_primary_buffer_nsymb=0;
	gear_shift_algorithm=SUCCESS_BASED_LADDER;

	gear_shift_up_success_rate_precentage=85;
	gear_shift_down_success_rate_precentage=55;
	gear_shift_block_for_nBlocks_total=5;
	gear_shift_blocked_for_nBlocks=0;
	gear_shift_down_consecutive_fails=0;
	consecutive_data_acks=0;
	frame_shift_threshold=3;
	frame_gearshift_just_applied=false;

	turboshift_phase=TURBO_FORWARD;
	turboshift_active=true;
	turboshift_last_good=-1;
	turboshift_initiator=false;
	turboshift_retries=1;
	turbo_settle_pending=false;
	supershift_proven_ceiling=-1;
	skip_turbo_reverse=false;
	max_config_override=-1;
	turbo_snr_ack_enabled=false;
	turbo_received_snr=-99.0f;
	turbo_best_snr=-99.0f;
	turbo_switch_role_retries=0;
	ack_diag_peak_matched=0;
	ack_diag_peak_metric=0.0;
	ack_diag_poll_count=0;
	ack_diag_peak_mask=0;
	sack_diag_peak_matched=0;
	sack_diag_peak_metric=0.0;
	sack_diag_poll_count=0;
	sack_diag_peak_max_e=0.0;
	sack_diag_peak_max_seg=-1;

	phy_reinit_settle_us=300000;  // Phase-2 flag default = HEAD (b806b76 Bug #60)
	ack_metric_threshold=0.5;     // Phase-2 flag default = HEAD (7076a4b 3.0→0.5)
	sack_timeout_extra_ms=0;      // SACK_FIX_PLAN §7 step 3: default 0 post-(a).
	                              // Pre-(a) default was 3000 (commit 7076a4b)
	                              // — a band-aid for the now-closed Plan-A
	                              // turnaround race. CLI override
	                              // --sack-timeout-extra-ms=N is preserved.
	// Note: disable_sack initialized at top of constructor (~line 175) before
	// the local_capability assignment so the mask there sees the correct value.
	emergency_nack_count=0;
	emergency_nack_threshold=3;
	emergency_break_active=0;
	emergency_break_retries=3;
	emergency_previous_config=CONFIG_0;
	break_drop_step=1;
	break_recovery_phase=0;
	break_recovery_retries=0;
	ceiling_success_count=0;
	break_detected=NO;
	hail_detected=NO;
	hail_sent=NO;

	ptt_on_delay_ms=0;
	ptt_off_delay_ms=0;
	time_left_to_send_last_frame=0;

	last_message_sent_type=NONE;
	last_message_sent_code=NONE;

	last_message_received_type=NONE;
	last_message_received_code=NONE;

	last_received_message_sequence=255;
	last_received_end_of_batch_seq=-1;
	data_ack_received=NO;
	repeating_last_ack=NO;
	disconnect_requested=NO;
	connection_attempts=0;
	max_connection_attempts=15;
	exit_on_disconnect=NO;
	had_control_connection=NO;

	this->messages_control_bu.status=FREE;
	this->messages_control_bu.data=NULL;
	this->messages_control_bu.data=new char[N_MAX / 8];
	if(this->messages_control_bu.data==NULL)
	{
		exit(MEMORY_ERROR);
	}

}



cl_arq_controller::~cl_arq_controller()
{
	if(messages_control_bu.data!=NULL)
	{
		delete[] messages_control_bu.data;
	}
	this->deinit_messages_buffers();
}


void cl_arq_controller::set_nResends(int nResends)
{
	if(nResends>0)
	{
		this->nResends=nResends;
	}
}


void cl_arq_controller::set_ack_timeout_control(int ack_timeout_control)
{
	if(ack_timeout_control>0)
	{
		this->ack_timeout_control=ack_timeout_control;
	}

	if(messages_control.status==PENDING_ACK)
	{
		messages_control.ack_timeout=this->ack_timeout_control;
	}
	if(messages_control_bu.status==PENDING_ACK)
	{
		messages_control_bu.ack_timeout=this->ack_timeout_control;
	}
}

void cl_arq_controller::set_ack_timeout_data(int ack_timeout_data)
{
	if(ack_timeout_data>0)
	{
		this->ack_timeout_data=ack_timeout_data;
	}

	if(messages_tx!=NULL)
	{
		for(int i=0;i<nMessages;i++)
		{
			if(messages_tx[i].status==PENDING_ACK)
			{
				messages_tx[i].ack_timeout=this->ack_timeout_data;
			}
		}
	}
}

void cl_arq_controller::set_receiving_timeout(int receiving_timeout)
{
	if(receiving_timeout>0)
	{
		this->receiving_timeout=receiving_timeout;
	}
}

void cl_arq_controller::set_link_timeout(int link_timeout)
{
	if(link_timeout>0)
	{
		this->link_timeout=link_timeout;
	}
}


void cl_arq_controller::set_nMessages(int nMessages)
{
	if(nMessages>0 && nMessages<256)
	{
		this->nMessages=nMessages;
	}
	else
	{
		this->nMessages=255;
	}
}

void cl_arq_controller::set_max_buffer_length(int max_data_length, int max_message_length, int max_header_length)
{
	if(max_data_length>0 && max_data_length<256 && max_data_length<max_message_length)
	{
		this->max_data_length=max_data_length;
	}

	if(max_message_length>0 && max_message_length<256)
	{
		this->max_message_length=max_message_length;
	}

	if(max_header_length>0)
	{
		this->max_header_length=max_header_length;
	}
}

void cl_arq_controller::set_ack_batch_size(int ack_batch_size)
{
	if (ack_batch_size>0)
	{
		this->ack_batch_size=ack_batch_size;
	}
}

void cl_arq_controller::set_data_batch_size(int data_batch_size)
{
	if (data_batch_size>0)
	{
		if(data_batch_size<(max_data_length+max_header_length-ACK_MULTI_ACK_RANGE_HEADER_LENGTH-1))
		{
			this->data_batch_size=data_batch_size;
		}
		else
		{
			this->data_batch_size=(max_data_length+max_header_length-ACK_MULTI_ACK_RANGE_HEADER_LENGTH-1);
		}
	}
}

void cl_arq_controller::set_control_batch_size(int control_batch_size)
{
	if (control_batch_size>0)
	{
		this->control_batch_size=control_batch_size;
	}
}

void cl_arq_controller::set_role(int role)
{
	if(role==COMMANDER)
	{
		this->role=COMMANDER;
	}
	else
	{
		this->role=RESPONDER;
	}
	calculate_receiving_timeout();
}

void cl_arq_controller::calculate_receiving_timeout()
{
	if(this->role==COMMANDER)
	{
		if(ack_pattern_time_ms > 0)
		{
			// SACK_FIX_PLAN §7 step 3 (Candidate (a)) — geometry-derived
			// post-TX timeout. Replaces the pre-fix formula whose two
			// hardcoded 3000-ms adders were band-aids for the pre-Plan-A
			// turnaround race (now structurally closed by f2dbf34).
			//
			// timeout = frame_drain + sack_arrival + margin
			//   frame_drain  = 2 * message_transmission_time_ms
			//                  (CMD's last frame still in channel)
			//   sack_arrival = ptt_off_delay + RSP_DECODE_MARGIN_MS
			//                  + pattern_time + ptt_on_delay
			//                  (geometry of: CMD->silence; RSP decodes;
			//                   RSP keys SACK; SACK LE at CMD)
			//   margin       = SACK_ARRIVAL_MARGIN_MS (calibrated:
			//                  SACK_FIX_PLAN.md §11.1, worst observed
			//                  arrival 2102 ms vs geometric 1768 ms)
			//
			// sack_timeout_extra_ms is preserved as a runtime override
			// (default now 0); set with --sack-timeout-extra-ms=N if a
			// field deployment surfaces a regression.
			int pattern_time = ack_pattern_time_ms;
			if(sack_enabled && telecom_system)
			{
				int sack_samples = telecom_system->sack_pattern_passband_samples(data_batch_size);
				int sack_ms = (int)ceil(1000.0 * sack_samples / telecom_system->sampling_frequency);
				if(sack_ms > pattern_time)
					pattern_time = sack_ms;
			}
			int frame_drain  = 2 * message_transmission_time_ms;
			int sack_arrival = ptt_off_delay_ms + RSP_DECODE_MARGIN_MS
			                 + pattern_time + ptt_on_delay_ms;
			int margin       = SACK_ARRIVAL_MARGIN_MS;
			int timeout = frame_drain + sack_arrival + margin;
			// During turboshift, RSP calls load_configuration() on every probe,
			// adding ~200-500ms overhead. Extend receive window to prevent
			// premature timeout before ACK arrives.
			if(gear_shift_on && turboshift_phase != TURBO_DONE)
				timeout += 2000;
			// Runtime override safety net (--sack-timeout-extra-ms=N).
			// Default 0 post-fix; re-inflate at runtime if needed.
			if(sack_enabled)
				timeout += sack_timeout_extra_ms;
			printf("[CMD-POST-TX-CALIB] timeout=%dms = frame_drain=%d + sack_arrival=%d (ptt_off=%d + rsp_decode=%d + pattern=%d + ptt_on=%d) + margin=%d + extra=%d batch=%d sack=%d\n",
				timeout, frame_drain, sack_arrival,
				ptt_off_delay_ms, RSP_DECODE_MARGIN_MS, pattern_time, ptt_on_delay_ms,
				margin, sack_enabled ? sack_timeout_extra_ms : 0,
				data_batch_size, sack_enabled ? 1 : 0);
			fflush(stdout);
			set_receiving_timeout(timeout);
		}
		else
		{
			set_receiving_timeout((ack_batch_size+1)*ctrl_transmission_time_ms+time_left_to_send_last_frame+ptt_on_delay_ms);
		}
	}
	else
	{
		int rsp_timeout = (data_batch_size)*message_transmission_time_ms+time_left_to_send_last_frame+ptt_on_delay_ms;
		// RSP timeout: base timeout covers batch reception
		printf("[RSP-TIMEOUT] batch=%d msg_time=%d time_left=%d ptt=%d sack=%d -> timeout=%d\n",
			data_batch_size, message_transmission_time_ms, time_left_to_send_last_frame, ptt_on_delay_ms, sack_enabled ? 1 : 0, rsp_timeout);
		fflush(stdout);
		set_receiving_timeout(rsp_timeout);
	}
}

void cl_arq_controller::recalculate_ack_timeout_for_batch()
{
	if(ack_pattern_time_ms > 0)
	{
		int pattern_time = ack_pattern_time_ms;
		if(sack_enabled && telecom_system)
		{
			int sack_samples = telecom_system->sack_pattern_passband_samples(data_batch_size);
			int sack_ms = (int)ceil(1000.0 * sack_samples / telecom_system->sampling_frequency);
			if(sack_ms > pattern_time)
				pattern_time = sack_ms;
		}
		set_ack_timeout_data((data_batch_size+2)*message_transmission_time_ms + pattern_time
			+ 4*ptt_on_delay_ms + 4*ptt_off_delay_ms + 3000);
	}
	else
		set_ack_timeout_data((data_batch_size+1)*message_transmission_time_ms+control_batch_size*message_transmission_time_ms+2*ack_batch_size*ctrl_transmission_time_ms+time_left_to_send_last_frame+4*ptt_on_delay_ms+4*ptt_off_delay_ms);
}

void cl_arq_controller::set_call_sign(std::string call_sign)
{
	if(call_sign!= "")
	{
		this->my_call_sign=call_sign;
	}
}

int cl_arq_controller::get_nOccupied_messages()
{
	int nOccupied_messages=0;
	for(int i=0;i<this->nMessages;i++)
	{
		if(this->messages_tx[i].status!=FREE)
		{
			nOccupied_messages++;
		}
	}
	return nOccupied_messages;
}

int cl_arq_controller::get_nFree_messages()
{
	int nFree_messages=0;
	for(int i=0;i<this->nMessages;i++)
	{
		if(this->messages_tx[i].status==FREE)
		{
			nFree_messages++;
		}
	}
	return nFree_messages;
}

int cl_arq_controller::get_nTotal_messages()
{
	return this->nMessages;
}

int cl_arq_controller::get_nToSend_messages()
{
	int nMessages_to_send=0;
	for(int i=0;i<this->nMessages;i++)
	{
		if(messages_tx[i].status==ADDED_TO_LIST)
		{
			nMessages_to_send++;
		}
		else if (messages_tx[i].status==ACK_TIMED_OUT && messages_tx[i].nResends>0)
		{
			nMessages_to_send++;
		}
	}
	return nMessages_to_send;
}

int cl_arq_controller::get_nPending_Ack_messages()
{
	int nPending_Ack_messages=0;
	for(int i=0;i<this->nMessages;i++)
	{
		if(this->messages_tx[i].status==PENDING_ACK)
		{
			nPending_Ack_messages++;
		}
	}
	return nPending_Ack_messages;
}

int cl_arq_controller::get_nReceived_messages()
{
	int nReceived_messages=0;
	for(int i=0;i<this->nMessages;i++)
	{
		if(this->messages_rx[i].status==RECEIVED)
		{
			nReceived_messages++;
		}
	}
	return nReceived_messages;
}

int cl_arq_controller::get_nAcked_messages()
{
	int nAcked_messages=0;
	for(int i=0;i<this->nMessages;i++)
	{
		if(this->messages_rx[i].status==ACKED)
		{
			nAcked_messages++;
		}
	}
	return nAcked_messages;
}


void cl_arq_controller::messages_control_backup()
{
	messages_control_bu.ack_timeout=messages_control.ack_timeout;
	messages_control_bu.id=messages_control.id;
	messages_control_bu.length=messages_control.length;
	messages_control_bu.nResends=messages_control.nResends;
	messages_control_bu.status=messages_control.status;
	messages_control_bu.type=messages_control.type;
	int copy_len=max_data_length+max_header_length-CONTROL_ACK_CONTROL_HEADER_LENGTH;
	if(copy_len > N_MAX/8) copy_len = N_MAX/8;
	for(int i=0;i<copy_len;i++)
	{
		messages_control_bu.data[i]=messages_control.data[i];
	}
}
void cl_arq_controller::messages_control_restore()
{
	int copy_len=max_data_length+max_header_length-CONTROL_ACK_CONTROL_HEADER_LENGTH;
	if(copy_len > N_MAX/8) copy_len = N_MAX/8;
	for(int i=0;i<copy_len;i++)
	{
		messages_control.data[i]=messages_control_bu.data[i];
	}
	messages_control.ack_timeout=messages_control_bu.ack_timeout;
	messages_control.id=messages_control_bu.id;
	messages_control.length=messages_control_bu.length;
	messages_control.nResends=messages_control_bu.nResends;
	messages_control.status=messages_control_bu.status;
	messages_control.type=messages_control_bu.type;
	messages_control.ack_timer.start();
}


int cl_arq_controller::init(int tcp_base_port, int gear_shift_on, int initial_mode)
{
	int success=SUCCESSFUL;

	fifo_buffer_tx.set_size(default_configuration_ARQ.fifo_buffer_tx_size);
	fifo_buffer_rx.set_size(default_configuration_ARQ.fifo_buffer_rx_size);
	fifo_buffer_backup.set_size(default_configuration_ARQ.fifo_buffer_backup_size);

	set_link_timeout(default_configuration_ARQ.link_timeout);

	if (tcp_base_port)
	{
		tcp_socket_control.port = tcp_base_port;
		tcp_socket_data.port = tcp_base_port + 1;
	}
	else
	{
		tcp_socket_control.port = default_configuration_ARQ.tcp_socket_control_port;
		tcp_socket_data.port = default_configuration_ARQ.tcp_socket_data_port;
	}

	tcp_socket_control.timeout_ms=default_configuration_ARQ.tcp_socket_control_timeout_ms;
	tcp_socket_data.timeout_ms=default_configuration_ARQ.tcp_socket_data_timeout_ms;

	this->gear_shift_on = gear_shift_on;
	gear_shift_algorithm=default_configuration_ARQ.gear_shift_algorithm;
	current_configuration=CONFIG_NONE;

	if(robust_enabled)
	{
		// ROBUST mode: use requested ROBUST config for all roles
		// Gearshift between ROBUST levels handled separately
		init_configuration = initial_mode;
		data_configuration = initial_mode;
		ack_configuration = initial_mode;
	}
	else
	{
		init_configuration = initial_mode;
		data_configuration = initial_mode;
		ack_configuration=default_configuration_ARQ.ack_configuration;
	}

	if(tcp_socket_data.init()!=SUCCESS || tcp_socket_control.init()!=SUCCESS )
	{
		printf("Error initializing the TCP sockets. Exiting..\n");
		exit(-1);
	}

	load_configuration(ack_configuration,FULL,NO);
	load_configuration(data_configuration,PHYSICAL_LAYER_ONLY,YES);


	print_stats_timer.start();

//	TEST TX data
//		process_user_command("MYCALL rx001");
//		process_user_command("LISTEN ON");
//
//		process_user_command("MYCALL tx001");
//		process_user_command("CONNECT tx001 rx001");


//		std::string str="sent_quest1234";
//		char data;
//
//		for(int i=0;i<str.length();i++)
//		{
//			data=(char)str[i];
//			fifo_buffer_tx.push(&data,1);
//		}

	return success;
}

void cl_arq_controller::init_monitor_decoders()
{
	if(!passive_monitor || monitor_decoders_ready) return;

	printf("[MONITOR] Initializing %d parallel OFDM decoders...\n", NUMBER_OF_CONFIGS);
	fflush(stdout);

	// First, determine the largest buffer_Nsymb we need.
	// Init a temporary decoder as CONFIG_0 (BPSK 1/16 = largest OFDM frame)
	// to read its auto-calculated buffer_Nsymb.
	{
		cl_telecom_system tmp;
		tmp.narrowband_enabled = telecom_system->narrowband_enabled;
		tmp.load_configuration(CONFIG_0);
		monitor_primary_buffer_nsymb = tmp.data_container.buffer_Nsymb.load();
		printf("[MONITOR] CONFIG_0 buffer_Nsymb = %d (used as minimum for all decoders)\n",
			monitor_primary_buffer_nsymb);
		// tmp destructs here, freeing its buffers
	}

	for(int cfg = 0; cfg < NUMBER_OF_CONFIGS; cfg++)
	{
		monitor_decoders[cfg] = new cl_telecom_system();
		monitor_decoders[cfg]->narrowband_enabled = telecom_system->narrowband_enabled;
		// Force all decoders to use the largest buffer size so they can all
		// process the same audio snapshot (CONFIG_0 has the most symbols).
		monitor_decoders[cfg]->data_container.buffer_Nsymb_min = monitor_primary_buffer_nsymb;
		monitor_decoders[cfg]->load_configuration(cfg);
		printf("[MONITOR] Decoder CONFIG_%d ready (Nsymb=%d buffer_Nsymb=%d)\n",
			cfg, monitor_decoders[cfg]->data_container.Nsymb,
			monitor_decoders[cfg]->data_container.buffer_Nsymb.load());
	}

	// Set buffer_Nsymb_min on the primary telecom_system so the capture
	// buffer is large enough for the largest OFDM frame. Takes effect on
	// next load_configuration() call (first OFDM config switch via SET_CONFIG).
	// During MFSK phase, parallel OFDM decode is not used (MFSK uses the
	// regular single-decoder path), so the MFSK buffer size is fine.
	telecom_system->data_container.buffer_Nsymb_min = monitor_primary_buffer_nsymb;

	monitor_decoders_ready = true;
	printf("[MONITOR] All %d parallel decoders initialized.\n", NUMBER_OF_CONFIGS);
	fflush(stdout);
}

void cl_arq_controller::reinit_monitor_decoders()
{
	if(!passive_monitor) return;

	printf("[MONITOR] Reinitializing parallel decoders for %s mode...\n",
		telecom_system->narrowband_enabled ? "NB" : "WB");
	fflush(stdout);

	// Destroy existing decoders
	for(int cfg = 0; cfg < NUMBER_OF_CONFIGS; cfg++)
	{
		if(monitor_decoders[cfg])
		{
			delete monitor_decoders[cfg];
			monitor_decoders[cfg] = NULL;
		}
	}
	monitor_decoders_ready = false;

	// Recalculate buffer_Nsymb for the new bandwidth
	{
		cl_telecom_system tmp;
		tmp.narrowband_enabled = telecom_system->narrowband_enabled;
		tmp.load_configuration(CONFIG_0);
		monitor_primary_buffer_nsymb = tmp.data_container.buffer_Nsymb.load();
		printf("[MONITOR] CONFIG_0 buffer_Nsymb = %d (new bandwidth)\n",
			monitor_primary_buffer_nsymb);
	}

	// Recreate all decoders with correct bandwidth
	for(int cfg = 0; cfg < NUMBER_OF_CONFIGS; cfg++)
	{
		monitor_decoders[cfg] = new cl_telecom_system();
		monitor_decoders[cfg]->narrowband_enabled = telecom_system->narrowband_enabled;
		monitor_decoders[cfg]->data_container.buffer_Nsymb_min = monitor_primary_buffer_nsymb;
		monitor_decoders[cfg]->load_configuration(cfg);
		printf("[MONITOR] Decoder CONFIG_%d ready (Nsymb=%d buffer_Nsymb=%d)\n",
			cfg, monitor_decoders[cfg]->data_container.Nsymb,
			monitor_decoders[cfg]->data_container.buffer_Nsymb.load());
	}

	telecom_system->data_container.buffer_Nsymb_min = monitor_primary_buffer_nsymb;
	monitor_decoders_ready = true;
	printf("[MONITOR] Parallel decoders reinitialized (%d decoders, %s).\n",
		NUMBER_OF_CONFIGS, telecom_system->narrowband_enabled ? "NB" : "WB");
	fflush(stdout);
}

int cl_arq_controller::parallel_monitor_decode(double* audio, int audio_len,
                                                st_receive_stats& out_stats)
{
	if(!monitor_decoders_ready) return -1;

	// === GATE 1: Energy check ===
	// Quick scan for signal presence. If all samples below threshold,
	// no OFDM frame exists — skip everything. Eliminates decode attempts
	// during silence (~50% of iterations).
	{
		double peak = 0;
		int step = 64;
		for(int i = 0; i < audio_len; i += step)
		{
			double v = fabs(audio[i]);
			if(v > peak) peak = v;
		}
		if(peak < 0.05)
		{
			out_stats.message_decoded = NO;
			out_stats.delay = -1;
			return -1;
		}
	}

	// === Sequential decode with config memory + protocol knowledge ===
	// Try order priority:
	//   1. forward_configuration (from SET_CONFIG — the config commander is sending)
	//   2. reverse_configuration (responder→commander config after SWITCH_ROLE)
	//   3. Last 2 successfully decoded configs (handles alternation patterns)
	//   4. Remaining configs (rare — only on first encounter or after long gap)
	// The monitor doesn't need real-time decode — the ring buffer holds
	// ~5s (WB) to ~54s (NB) of audio. Worst case: 17 sequential attempts
	// × ~30ms = ~500ms. Still far faster than real-time and uses 1 CPU core.
	static int last_config_a = 0;  // Most recent successful config
	static int last_config_b = -1; // Second most recent (different from a)

	// Build try order: protocol-known configs first, then history, then rest
	int try_order[NUMBER_OF_CONFIGS];
	int idx = 0;
	bool used[NUMBER_OF_CONFIGS] = {};

	// Protocol-level knowledge: SET_CONFIG tells us exactly which configs are active
	int fwd = (int)this->forward_configuration;
	int rev = (int)this->reverse_configuration;
	if(fwd >= 0 && fwd < NUMBER_OF_CONFIGS && !used[fwd])
	{
		try_order[idx++] = fwd;
		used[fwd] = true;
	}
	if(rev >= 0 && rev < NUMBER_OF_CONFIGS && !used[rev])
	{
		try_order[idx++] = rev;
		used[rev] = true;
	}
	// History-based: last 2 successful configs
	if(last_config_a >= 0 && last_config_a < NUMBER_OF_CONFIGS && !used[last_config_a])
	{
		try_order[idx++] = last_config_a;
		used[last_config_a] = true;
	}
	if(last_config_b >= 0 && last_config_b < NUMBER_OF_CONFIGS && !used[last_config_b])
	{
		try_order[idx++] = last_config_b;
		used[last_config_b] = true;
	}
	// Fill remaining
	for(int cfg = 0; cfg < NUMBER_OF_CONFIGS; cfg++)
	{
		if(!used[cfg])
			try_order[idx++] = cfg;
	}

	bool preamble_seen = false;

	for(int t = 0; t < NUMBER_OF_CONFIGS; t++)
	{
		int cfg = try_order[t];
		cl_telecom_system* dec = monitor_decoders[cfg];

		int dec_buf_len = dec->data_container.Nofdm
			* dec->data_container.buffer_Nsymb.load()
			* dec->data_container.interpolation_rate;
		int copy_len = (audio_len < dec_buf_len) ? audio_len : dec_buf_len;
		memcpy(dec->data_container.ready_to_process_passband_delayed_data,
			audio, copy_len * sizeof(double));
		if(copy_len < dec_buf_len)
			memset(&dec->data_container.ready_to_process_passband_delayed_data[copy_len],
				0, (dec_buf_len - copy_len) * sizeof(double));

		st_receive_stats stats = dec->receive_byte(
			dec->data_container.ready_to_process_passband_delayed_data,
			dec->data_container.data_byte);

		if(stats.message_decoded == YES)
		{
			if(cfg != last_config_a)
			{
				last_config_b = last_config_a;
				last_config_a = cfg;
			}
			// Save decoded data to staging buffer — NOT to primary's data_byte.
			// load_configuration() may deinit/reinit the primary on cross-modulation
			// switches (BPSK→QPSK etc), destroying data_byte. The caller copies
			// from monitor_decoded_data to primary AFTER load_configuration.
			monitor_decoded_len = dec->get_frame_size_bytes();
			if(monitor_decoded_len > N_MAX / 8) monitor_decoded_len = N_MAX / 8;
			memcpy(monitor_decoded_data,
				dec->data_container.data_byte,
				monitor_decoded_len * sizeof(int));
			out_stats = stats;
			printf("[MONITOR] CONFIG_%d DECODED (SNR=%.1f iter=%d, tried %d/%d)\n",
				cfg, stats.SNR, stats.iterations_done, t + 1, NUMBER_OF_CONFIGS);
			fflush(stdout);
			return cfg;
		}

		if(stats.delay >= 0)
			preamble_seen = true;

		// Preamble gate: if the first decoder found no preamble,
		// no other config will either (Schmidl-Cox is config-independent).
		if(t == 0 && !preamble_seen)
		{
			out_stats.message_decoded = NO;
			out_stats.delay = -1;
			return -1;
		}
	}

	// All 17 configs tried, none decoded (interference / noise)
	out_stats.message_decoded = NO;
	out_stats.delay = -1;
	return -1;
}

char cl_arq_controller::get_configuration(double SNR)
{
	char configuration;
	configuration =telecom_system->get_configuration(SNR);
	return configuration;
}

void cl_arq_controller::load_configuration(int configuration, int level, int backup_configuration)
{
	printf("[CFG] load_configuration(%d) current=%d level=%s backup=%s\n",
		configuration, this->current_configuration,
		level == FULL ? "FULL" : "PHYS_ONLY",
		backup_configuration == YES ? "YES" : "NO");
	if(configuration==this->current_configuration)
	{
		printf("[CFG] Already on config %d, skipping\n", configuration);
		return;
	}
	if(current_configuration!=CONFIG_NONE)
	{
		if(level==FULL)
		{
			printf("[CANARY] Pre-FULL-deinit canary check (config %d -> %d)\n",
				current_configuration, configuration);
			fflush(stdout);
			check_buffer_canaries("pre_FULL_deinit");
			this->restore_backup_buffer_data();
			this->deinit_messages_buffers();
		}
		if(backup_configuration==YES)
		{
			this->last_data_configuration= this->current_configuration;
		}
	}
	else
	{
		if(level==FULL)
		{
			printf("[CFG] CONFIG_NONE path: deinit_messages_buffers\n");
			fflush(stdout);
			this->deinit_messages_buffers();
			printf("[CFG] CONFIG_NONE path: deinit_messages_buffers done\n");
			fflush(stdout);
		}
		if(backup_configuration==YES)
		{
			this->last_data_configuration=configuration;
		}
	}

	this->current_configuration=configuration;

	// Canary check during PHYS_ONLY transitions — track when corruption first appears
	if(level != FULL)
	{
		check_buffer_canaries("PHYS_ONLY_transition");
	}

	// Pause audio capture during PHY reconfig to prevent race condition:
	// telecom_system->load_configuration may deinit/reinit buffers that
	// the audio callback accesses (passband_delayed_data, etc.)
	telecom_system->data_container.frames_to_read = 0;
	printf("[CFG] Calling telecom_system->load_configuration(%d) nb=%d\n",
		configuration, telecom_system->narrowband_enabled);
	fflush(stdout);
	telecom_system->load_configuration(configuration);
	printf("[CFG] telecom_system->load_configuration done\n");
	fflush(stdout);

	// Note: after config switch (e.g. MFSK→OFDM), the zeroed buffer may still
	// receive stale audio from VB-Cable's internal buffer (~7 symbols). This can
	// cause Schmidl-Cox false triggers on MFSK remnants. However, extended buffer
	// flushing causes gearshift timeouts (NB CONFIG_0 buffer_Nsymb=581 → 13.2s).
	// The OFDM decoder's energy gate + mean_H threshold handle stale data adequately.
	// WB OFDM fails on VB-Cable (cause under investigation — likely TX clipping
	// or Moose freq sync issue, not VB-Cable itself). On real radio, the buffer
	// starts zeroed (memset in set_size) and VB-Cable latency is not an issue.
	int nBytes_header=0;
	if (ACK_MULTI_ACK_RANGE_HEADER_LENGTH>nBytes_header) nBytes_header=ACK_MULTI_ACK_RANGE_HEADER_LENGTH;
	if (CONTROL_ACK_CONTROL_HEADER_LENGTH>nBytes_header) nBytes_header=CONTROL_ACK_CONTROL_HEADER_LENGTH;
	// SACK Design A Steps 1+2 — DATA_LONG header is 4 bytes (v1) or 5 bytes (v2);
	// DATA_SHORT header is 5 bytes (v1) or 6 bytes (v2). Effective helpers gate
	// buffer sizing on sack_v2_enabled — v1 path picks the legacy macro values
	// (byte-identical wire to pre-Step-1/2), v2 path picks the grown values.
	// Post-negotiation, the next load_configuration() call picks up v2 sizing.
	{
		int eff_long = effective_data_long_header_length(sack_v2_enabled);
		if (eff_long>nBytes_header) nBytes_header=eff_long;
		int eff_short = effective_data_short_header_length(sack_v2_enabled);
		if (eff_short>nBytes_header) nBytes_header=eff_short;
	}

	int nBytes_data=(telecom_system->data_container.nBits-telecom_system->ldpc.P-telecom_system->outer_code_reserved_bits)/8 - nBytes_header;
	int nBytes_message=(telecom_system->data_container.nBits)/8 ;


	set_max_buffer_length(nBytes_data, nBytes_message, nBytes_header);
	set_nMessages(default_configuration_ARQ.nMessages);
	set_nResends(default_configuration_ARQ.nResends);

	if(level==FULL)
	{
		set_data_batch_size(default_configuration_ARQ.batch_size);
	}
	set_ack_batch_size(default_configuration_ARQ.ack_batch_size);
	set_control_batch_size(default_configuration_ARQ.control_batch_size);

	// MFSK modes: single ACK/control frame per batch (no redundant copies).
	// LDPC provides cliff-effect protection; if a frame decodes, it's correct.
	// Saves 1 frame per ACK cycle + 1 per control cycle = major time savings
	// at 4.6-7.3s per frame.
	if(is_robust_config(configuration))
	{
		set_data_batch_size(1);
		set_ack_batch_size(1);
		set_control_batch_size(1);
	}
	
	gear_shift_up_success_rate_precentage=default_configuration_ARQ.gear_shift_up_success_rate_limit_precentage;
	gear_shift_down_success_rate_precentage=default_configuration_ARQ.gear_shift_down_success_rate_limit_precentage;

	gear_shift_block_for_nBlocks_total=default_configuration_ARQ.gear_shift_block_for_nBlocks_total;
	gear_shift_blocked_for_nBlocks=0;
	consecutive_data_acks=0;
	// NOTE: turboshift state is NOT reset here — it persists across config changes.
	// Only reset at connection init (see init code above).

	message_transmission_time_ms=ceil((1000.0*(telecom_system->data_container.Nsymb+telecom_system->data_container.preamble_nSymb)*telecom_system->data_container.Nofdm*telecom_system->frequency_interpolation_rate)/(float)(telecom_system->frequency_interpolation_rate*(telecom_system->bandwidth/telecom_system->ofdm.Nc)*telecom_system->ofdm.Nfft));
	if(telecom_system->ctrl_nsymb > 0)
	{
		ctrl_transmission_time_ms=ceil((1000.0*(telecom_system->ctrl_nsymb+telecom_system->data_container.preamble_nSymb)*telecom_system->data_container.Nofdm*telecom_system->frequency_interpolation_rate)/(float)(telecom_system->frequency_interpolation_rate*(telecom_system->bandwidth/telecom_system->ofdm.Nc)*telecom_system->ofdm.Nfft));
	}
	else
	{
		ctrl_transmission_time_ms=message_transmission_time_ms;
	}
    // TODO: After audio I/O rewrite we don't use this anymore. Was:
	// time_left_to_send_last_frame=(float)telecom_system->speaker.frames_to_leave_transmit_fct/(float)(telecom_system->frequency_interpolation_rate*(telecom_system->bandwidth/telecom_system->ofdm.Nc)*telecom_system->ofdm.Nfft);
    time_left_to_send_last_frame=0;

	// Scale data_batch_size based on block duration (OFDM modes only).
	// MFSK modes keep batch_size=1 for pattern ACK optimization.
	// Batch sizing: with all-or-nothing MFSK ACK, P(batch success) = p_ofdm^N × p_ack.
	// Start conservative (5 frames), adaptive growth finds optimal batch for link quality.
	// nominal_batch_size = ceiling from 12s target — adaptive mechanism grows toward it.
	if(!is_robust_config(configuration) && message_transmission_time_ms > 0)
	{
		int target_time_ms = 12000;
		int max_batch = (int)((float)target_time_ms / message_transmission_time_ms + 0.5);
		if(max_batch < 5) max_batch = 5;
		if(max_batch > nMessages) max_batch = nMessages;
		// Batch sizing: with SACK, partial batches are retransmitted selectively,
		// so larger batches improve duty cycle without risking full retransmission.
		// Without SACK: batch=10 balances duty cycle vs batch failure probability.
		int fixed_batch = sack_enabled ? radio_batch_size : 10;
		if(fixed_batch > max_batch) fixed_batch = max_batch;
		set_data_batch_size(fixed_batch);
		nominal_batch_size = fixed_batch;
		printf("[CFG] Batch scaling: msg_time=%dms initial=%d max=%d nMessages=%d\n",
			message_transmission_time_ms, data_batch_size, max_batch, nMessages);
		fflush(stdout);
	}
	else
	{
		nominal_batch_size = data_batch_size;
	}

	// ACK pattern transmission time (universal: all modes)
	if(telecom_system->ack_pattern_passband_samples > 0)
	{
		ack_pattern_time_ms = (int)ceil(1000.0 * telecom_system->ack_pattern_passband_samples / telecom_system->sampling_frequency);
	}
	else
	{
		ack_pattern_time_ms = 0;
	}

	// With pattern-based ACK, control_batch_size=1: LDPC cliff effect makes
	// redundant copies wasteful, and halving TX means faster ACK round-trip.
	// ack_batch_size=1: irrelevant for pattern ACK but keeps consistency.
	if(ack_pattern_time_ms > 0)
	{
		set_control_batch_size(1);
		set_ack_batch_size(1);
	}

	if(ack_pattern_time_ms > 0)
	{
		// ACK is a short tone pattern, not a full LDPC frame.
		// Bug #44: responder turnaround includes CMD frame TX time, so
		// ack_timeout must cover: TX + responder ftr (frame_TX + 4000ms) + ACK.
		set_ack_timeout_data((data_batch_size+2)*message_transmission_time_ms + ack_pattern_time_ms + 4*ptt_on_delay_ms + 4*ptt_off_delay_ms + 3000);
		set_ack_timeout_control((control_batch_size+1)*message_transmission_time_ms + ack_pattern_time_ms + 2*ptt_on_delay_ms + 2*ptt_off_delay_ms + 3000);
	}
	else
	{
		set_ack_timeout_data((data_batch_size+1)*message_transmission_time_ms+control_batch_size*message_transmission_time_ms+2*ack_batch_size*ctrl_transmission_time_ms+time_left_to_send_last_frame+4*ptt_on_delay_ms+4*ptt_off_delay_ms);
		set_ack_timeout_control(control_batch_size*message_transmission_time_ms+ack_batch_size*ctrl_transmission_time_ms+time_left_to_send_last_frame+2*ptt_on_delay_ms+2*ptt_off_delay_ms);
	}

	// During turboshift, extend ack_timeout_control (the overall NAck deadline).
	// receiving_timeout margin is handled inside calculate_receiving_timeout().
	if(gear_shift_on && turboshift_phase != TURBO_DONE)
	{
		set_ack_timeout_control(ack_timeout_control + 2000);
	}

	ptt_on_delay_ms=default_configuration_ARQ.ptt_on_delay_ms;
	ptt_off_delay_ms=default_configuration_ARQ.ptt_off_delay_ms;
	pilot_tone_ms=default_configuration_ARQ.pilot_tone_ms;
	pilot_tone_hz=default_configuration_ARQ.pilot_tone_hz;
	switch_role_timeout=default_configuration_ARQ.switch_role_timeout_ms;
	// MFSK modes: frame durations are 4-7s, so both sides have ample prep time
	// during the frame itself. Reduce role-switch wait from 1500ms to 200ms.
	if(is_robust_config(configuration))
		switch_role_timeout = 200;

	switch_role_test_timeout=(nResends/3)*ack_timeout_control;
	watchdog_timeout=(nResends/3)*ack_timeout_data;
	gearshift_timeout=(nResends/3)*ack_timeout_data;

	// Ensure connection_timeout and link_timeout are adequate for MFSK frame durations.
	{
		int ack_time = (ack_pattern_time_ms > 0) ? ack_pattern_time_ms : (ack_batch_size * ctrl_transmission_time_ms);

		// Connection handshake: 2 round-trips of control+ack batches
		int min_ct = 2 * (control_batch_size * message_transmission_time_ms + ack_time)
			+ 4 * ptt_on_delay_ms + 4 * ptt_off_delay_ms + 5000;
		if (connection_timeout < min_ct)
			connection_timeout = min_ct;

		// Link timeout: must survive a full data+ack round-trip
		int min_lt = (data_batch_size + 2) * message_transmission_time_ms + ack_time
			+ 2 * ptt_on_delay_ms + 2 * ptt_off_delay_ms + 5000;
		if (link_timeout < min_lt)
			link_timeout = min_lt;
	}

	calculate_receiving_timeout();

	// Reset OFDM batch prediction state on config change.
	// After turboshift, ofdm_batch_active/ofdm_search_raw hold stale positions
	// from control frames decoded at the old config.  The new config has different
	// Nsymb/frame geometry, so the old ofdm_skip prediction lands in the wrong
	// place → wrong delay → wrong Moose freq offset → LDPC fails on real HF.
	telecom_system->receive_stats.ofdm_batch_active = false;
	telecom_system->receive_stats.ofdm_search_raw = 0;
	telecom_system->receive_stats.ofdm_drift_per_frame = 0.0;

	// Guard: ack_timeout must cover the full TX + receive window.
	// ack_timer starts at frame send (T=0); receiving_timer starts after TX + PTT.
	// Without this, ack_timeout can expire while CMD is still polling for ACK.
	if(this->role == COMMANDER)
	{
		int min_ack = message_transmission_time_ms + ptt_off_delay_ms + receiving_timeout + 500;
		if(ack_timeout_control < min_ack)
			set_ack_timeout_control(min_ack);
		if(ack_timeout_data < min_ack)
			set_ack_timeout_data(min_ack);
	}

	if(level==FULL)
	{
		printf("[CFG] init_messages_buffers (nMessages=%d)\n", nMessages);
		fflush(stdout);
		this->init_messages_buffers();
		printf("[CFG] init_messages_buffers done\n");
		fflush(stdout);
	}
}

void cl_arq_controller::return_to_last_configuration()
{
	if(last_data_configuration==this->current_configuration)
	{
		return;
	}
	int tmp;
	this->load_configuration(last_data_configuration,FULL,YES);
	tmp= last_data_configuration;
	last_data_configuration=current_configuration;
	current_configuration=tmp;
}

// Canary: 16 bytes of 0xCC appended to each buffer to detect overflow.
// check_canaries() validates them; any corruption pinpoints the overflow target.
#define CANARY_SIZE 16
#define CANARY_BYTE 0xCC

static void set_canary(char* buf, int data_size)
{
	memset(buf + data_size, CANARY_BYTE, CANARY_SIZE);
}

// Diagnostic globals for crash handler — set before each canary read
// so we know which buffer was being checked when the crash occurs.
volatile const char* g_canary_check_name = NULL;
volatile int g_canary_check_idx = -1;
volatile const char* g_canary_check_ptr = NULL;

static int check_canary(const char* buf, int data_size, const char* name, int idx)
{
	g_canary_check_name = name;
	g_canary_check_idx = idx;
	g_canary_check_ptr = buf;
	if(buf == NULL) return 0;
	for(int j=0; j<CANARY_SIZE; j++)
	{
		if((unsigned char)buf[data_size + j] != CANARY_BYTE)
		{
			printf("[CANARY] OVERFLOW %s[%d] at offset %d (byte=0x%02x, expected 0xCC)\n",
				name, idx, data_size + j, (unsigned char)buf[data_size + j]);
			fflush(stdout);
			return 1;
		}
	}
	return 0;
}

void cl_arq_controller::check_buffer_canaries(const char* caller)
{
	const int alloc_size = N_MAX / 8;
	int corrupted = 0;

	if(messages_tx != NULL)
	{
		for(int i=0; i<nMessages; i++)
			corrupted += check_canary(messages_tx[i].data, alloc_size, "messages_tx", i);
	}
	if(messages_rx != NULL)
	{
		for(int i=0; i<nMessages; i++)
			corrupted += check_canary(messages_rx[i].data, alloc_size, "messages_rx", i);
	}
	if(messages_batch_ack != NULL)
	{
		for(int i=0; i<255; i++)
			corrupted += check_canary(messages_batch_ack[i].data, alloc_size, "messages_batch_ack", i);
	}
	corrupted += check_canary(messages_last_ack_bu.data, alloc_size, "messages_last_ack_bu", 0);
	corrupted += check_canary(messages_control.data, alloc_size, "messages_control", 0);
	corrupted += check_canary(messages_rx_buffer.data, alloc_size, "messages_rx_buffer", 0);
	corrupted += check_canary(message_TxRx_byte_buffer, alloc_size, "message_TxRx_byte_buffer", 0);

	if(corrupted > 0)
	{
		printf("[CANARY] %d canary violations detected! caller=%s\n", corrupted, caller);
		fflush(stdout);
	}
}

int cl_arq_controller::init_messages_buffers()
{
	int success=SUCCESSFUL;

	// Allocate all message buffers with N_MAX/8 (= 200 bytes) instead of
	// the current config's max_message_length. This prevents heap overflow
	// when PHYS_ONLY config transitions (e.g., turboshift or gearshift)
	// increase max_message_length without reallocating these buffers.
	// N_MAX = 1600 bits is the absolute maximum LDPC codeword size.
	const int alloc_size = N_MAX / 8;

	this->messages_tx=new st_message[nMessages];

	if(this->messages_tx==NULL)
	{
		success=MEMORY_ERROR;
	}
	else
	{
		for(int i=0;i<this->nMessages;i++)
		{
			this->messages_tx[i].ack_timeout=0;
			this->messages_tx[i].id=0;
			this->messages_tx[i].length=0;
			this->messages_tx[i].nResends=0;
			this->messages_tx[i].status=FREE;
			this->messages_tx[i].type=NONE;
			this->messages_tx[i].data=NULL;
			this->messages_tx[i].batch_seq_id=-1;  // Step 3: unset until TX path assigns

			this->messages_tx[i].data=new char[alloc_size + CANARY_SIZE];
			set_canary(this->messages_tx[i].data, alloc_size);

			if(this->messages_tx[i].data==NULL)
			{
				success=MEMORY_ERROR;
			}
		}
	}

	this->messages_rx=new st_message[nMessages];

	if(this->messages_rx==NULL)
	{
		success=MEMORY_ERROR;
	}
	else
	{
		for(int i=0;i<this->nMessages;i++)
		{
			this->messages_rx[i].ack_timeout=0;
			this->messages_rx[i].id=0;
			this->messages_rx[i].length=0;
			this->messages_rx[i].nResends=0;
			this->messages_rx[i].status=FREE;
			this->messages_rx[i].type=NONE;
			this->messages_rx[i].data=NULL;
			this->messages_rx[i].batch_seq_id=-1;  // Step 3: unset until RX parses (v2)

			this->messages_rx[i].data=new char[alloc_size + CANARY_SIZE];
			set_canary(this->messages_rx[i].data, alloc_size);

			if(this->messages_rx[i].data==NULL)
			{
				success=MEMORY_ERROR;
			}
		}
	}

	// Allocate 255 elements (absolute max nMessages) rather than current
	// data_batch_size, because PHYS_ONLY config transitions can increase
	// both nMessages and data_batch_size without reallocating (Bug #15).
	const int max_batch_alloc = 255;
	this->messages_batch_tx=new st_message[max_batch_alloc];

	if(this->messages_batch_tx==NULL)
	{
		success=MEMORY_ERROR;
	}
	else
	{
		for(int i=0;i<max_batch_alloc;i++)
		{
			this->messages_batch_tx[i].ack_timeout=0;
			this->messages_batch_tx[i].id=0;
			this->messages_batch_tx[i].length=0;
			this->messages_batch_tx[i].nResends=0;
			this->messages_batch_tx[i].status=FREE;
			this->messages_batch_tx[i].type=NONE;
			this->messages_batch_tx[i].data=NULL;
			this->messages_batch_tx[i].batch_seq_id=-1;  // Step 3: unset until build_batch assigns
		}
	}

	// Same fix for ack batch array (Bug #15).
	this->messages_batch_ack=new st_message[max_batch_alloc];

	if(this->messages_batch_ack==NULL)
	{
		success=MEMORY_ERROR;
	}
	else
	{
		for(int i=0;i<max_batch_alloc;i++)
		{
			this->messages_batch_ack[i].ack_timeout=0;
			this->messages_batch_ack[i].id=0;
			this->messages_batch_ack[i].length=0;
			this->messages_batch_ack[i].nResends=0;
			this->messages_batch_ack[i].status=FREE;
			this->messages_batch_ack[i].type=NONE;
			this->messages_batch_ack[i].data=NULL;
			this->messages_batch_ack[i].batch_seq_id=-1;  // Step 3: ACK frames never carry batch_seq_id

			this->messages_batch_ack[i].data=new char[alloc_size + CANARY_SIZE];
			set_canary(this->messages_batch_ack[i].data, alloc_size);

			if(this->messages_batch_ack[i].data==NULL)
			{
				success=MEMORY_ERROR;
			}
		}
	}

	this->messages_last_ack_bu.status=FREE;
	this->messages_last_ack_bu.data=NULL;
	this->messages_last_ack_bu.batch_seq_id=-1;  // Step 3
	this->messages_last_ack_bu.data=new char[alloc_size + CANARY_SIZE];
	set_canary(this->messages_last_ack_bu.data, alloc_size);

	if(this->messages_last_ack_bu.data==NULL)
	{
		success=MEMORY_ERROR;
	}

	this->messages_control.status=FREE;
	this->messages_control.data=NULL;
	this->messages_control.batch_seq_id=-1;  // Step 3: control frames never carry batch_seq_id
	this->messages_control.data=new char[alloc_size + CANARY_SIZE];
	set_canary(this->messages_control.data, alloc_size);

	if(this->messages_control.data==NULL)
	{
		success=MEMORY_ERROR;
	}

	this->messages_rx_buffer.status=FREE;
	this->messages_rx_buffer.data=NULL;
	this->messages_rx_buffer.batch_seq_id=-1;  // Step 3: populated on v2 DATA RX
	this->messages_rx_buffer.data=new char[alloc_size + CANARY_SIZE];
	set_canary(this->messages_rx_buffer.data, alloc_size);

	if(this->messages_rx_buffer.data==NULL)
	{
		success=MEMORY_ERROR;
	}

	this->messages_control.status=FREE;
	this->message_TxRx_byte_buffer=new char[alloc_size + CANARY_SIZE];
	set_canary(this->message_TxRx_byte_buffer, alloc_size);

	if(this->message_TxRx_byte_buffer==NULL)
	{
		success=MEMORY_ERROR;
	}
	return success;
}
int cl_arq_controller::deinit_messages_buffers()
{
	int success=SUCCESSFUL;

	// Check all canaries before freeing — any corruption reveals the overflow target
	check_buffer_canaries("deinit_messages_buffers");

	if(messages_tx!=NULL)
	{
		for(int i=0;i<nMessages;i++)
		{
			if(messages_tx[i].data!=NULL)
			{
				delete[] messages_tx[i].data;
				messages_tx[i].data=NULL;
			}
		}
		delete[] messages_tx;
		messages_tx=NULL;
	}

	if(messages_rx!=NULL)
	{
		for(int i=0;i<nMessages;i++)
		{
			if(messages_rx[i].data!=NULL)
			{
				delete[] messages_rx[i].data;
				messages_rx[i].data=NULL;
			}
		}
		delete[] messages_rx;
		messages_rx=NULL;
	}

	if(messages_batch_ack!=NULL)
	{
		for(int i=0;i<255;i++)
		{
			if(messages_batch_ack[i].data!=NULL)
			{
				delete[] messages_batch_ack[i].data;
				messages_batch_ack[i].data=NULL;
			}
		}
		delete[] messages_batch_ack;
		messages_batch_ack=NULL;
	}

	if(messages_last_ack_bu.data!=NULL)
	{
		delete[] messages_last_ack_bu.data;
		messages_last_ack_bu.data=NULL;
	}
	if(messages_control.data!=NULL)
	{
		delete[] messages_control.data;
		messages_control.data=NULL;
	}
	if(messages_rx_buffer.data!=NULL)
	{
		delete[] messages_rx_buffer.data;
		messages_rx_buffer.data=NULL;
	}
	if(messages_batch_tx!=NULL)
	{
		delete[] messages_batch_tx;
		messages_batch_tx=NULL;
	}
	if(message_TxRx_byte_buffer!=NULL)
	{
		delete[] message_TxRx_byte_buffer;
		message_TxRx_byte_buffer=NULL;
	}

	return success;
}

void cl_arq_controller::update_status()
{
	for(int i=0;i<nMessages;i++)
	{
		if(messages_tx[i].status==PENDING_ACK && messages_tx[i].ack_timer.get_elapsed_time_ms()>=messages_tx[i].ack_timeout)
		{
			messages_tx[i].status=ACK_TIMED_OUT;
			stats.nNAcked_data++;
		}
	}

	if(messages_control.status==PENDING_ACK && messages_control.ack_timer.get_elapsed_time_ms()>=messages_control.ack_timeout)
	{
		messages_control.status=ACK_TIMED_OUT;
		stats.nNAcked_control++;
	}

	// Check connection attempt timeout - separate from link_timer which gets restarted on every message
	if((link_status==CONNECTING || link_status==NEGOTIATING || link_status==CONNECTION_ACCEPTED) &&
	   connection_attempt_timer.counting==1 &&
	   connection_attempt_timer.get_elapsed_time_ms()>=connection_timeout)
	{
		std::cout<<"Connection attempt timeout after "<<connection_timeout<<" ms"<<std::endl;

		// NB/WB auto-negotiation: if commander is still in NB probe mode at
		// timeout, switch to WB and retry. Covers two cases:
		//   1. connection_attempts < nb_probe_max (original Phase 1→2)
		//   2. connection_attempts >= nb_probe_max but still NB (HAIL retry
		//      loop consumed all probe attempts without triggering switch-back)
		if(role == COMMANDER && link_status == CONNECTING &&
		   commander_configured_nb >= 0 &&
		   narrowband_enabled == YES && commander_configured_nb != YES)
		{
			connection_attempts = nb_probe_max;  // Skip remaining probes
			printf("[NB-NEG] Commander: timeout — restoring WB (attempts=%d)\n",
				connection_attempts);
			fflush(stdout);
			switch_narrowband_mode(NO);  // Switch to WB
			hail_detected = NO;  // Reset so WB HAIL is attempted
			// Reset timer and control message for fresh WB attempt
			connection_attempt_timer.reset();
			connection_attempt_timer.start();
			messages_control.status = FREE;
			// Stay in CONNECTING — process_messages_commander will send new HAIL/START_CONNECTION
			return;
		}

		// Send CANCELPENDING and DISCONNECTED to Winlink (connection attempt timed out)
		if(role==COMMANDER && tcp_socket_control.get_status()==TCP_STATUS_ACCEPTED)
		{
			std::string str="CANCELPENDING\r";
			tcp_socket_control.message->length=str.length();
			for(int i=0;i<tcp_socket_control.message->length;i++)
			{
				tcp_socket_control.message->buffer[i]=str[i];
			}
			tcp_socket_control.transmit();

			str="DISCONNECTED\r";
			tcp_socket_control.message->length=str.length();
			for(int i=0;i<tcp_socket_control.message->length;i++)
			{
				tcp_socket_control.message->buffer[i]=str[i];
			}
			tcp_socket_control.transmit();
		}

		this->link_status=DROPPED;
		reset_session_state();
		reset_all_timers();
		connection_attempt_timer.stop();
		connection_attempt_timer.reset();

		fifo_buffer_tx.flush();
		fifo_buffer_backup.flush();
		fifo_buffer_rx.flush();

		// Reset messages_control so new CONNECT commands can work
		messages_control.status=FREE;

		// After failed connection attempt, always switch to RESPONDER/LISTENING
		// so we can receive incoming connections from the other side
		set_role(RESPONDER);
		link_status=LISTENING;
		connection_status=RECEIVING;
		load_configuration(init_configuration, FULL, YES);
		printf("Switching to RESPONDER mode after connection timeout\n");
	}

	// Check for max connection attempts
	if((link_status==CONNECTING || link_status==NEGOTIATING || link_status==CONNECTION_ACCEPTED) &&
	   connection_attempts >= max_connection_attempts)
	{
		std::cout<<"Maximum connection attempts ("<<max_connection_attempts<<") reached"<<std::endl;

		// Send CANCELPENDING and DISCONNECTED to Winlink (max connection attempts reached)
		if(role==COMMANDER && tcp_socket_control.get_status()==TCP_STATUS_ACCEPTED)
		{
			std::string str="CANCELPENDING\r";
			tcp_socket_control.message->length=str.length();
			for(int i=0;i<tcp_socket_control.message->length;i++)
			{
				tcp_socket_control.message->buffer[i]=str[i];
			}
			tcp_socket_control.transmit();

			str="DISCONNECTED\r";
			tcp_socket_control.message->length=str.length();
			for(int i=0;i<tcp_socket_control.message->length;i++)
			{
				tcp_socket_control.message->buffer[i]=str[i];
			}
			tcp_socket_control.transmit();
		}

		this->link_status=DROPPED;
		reset_session_state();
		reset_all_timers();
		connection_attempt_timer.stop();
		connection_attempt_timer.reset();

		fifo_buffer_tx.flush();
		fifo_buffer_backup.flush();
		fifo_buffer_rx.flush();

		// Reset messages_control so new CONNECT commands can work
		messages_control.status=FREE;

		// After failed connection attempts, always switch to RESPONDER/LISTENING
		// so we can receive incoming connections from the other side
		set_role(RESPONDER);
		link_status=LISTENING;
		connection_status=RECEIVING;
		load_configuration(init_configuration, FULL, YES);
		printf("Switching to RESPONDER mode after max connection attempts\n");
	}

	if(link_timer.get_elapsed_time_ms()>=link_timeout)
	{
		this->link_status=DROPPED;
		reset_session_state();
		reset_all_timers();

		fifo_buffer_tx.flush();
		fifo_buffer_backup.flush();
		fifo_buffer_rx.flush();

		// Notify Winlink of disconnect
		if(tcp_socket_control.get_status()==TCP_STATUS_ACCEPTED)
		{
			std::string str="DISCONNECTED\r";
			tcp_socket_control.message->length=str.length();
			for(int i=0;i<(int)tcp_socket_control.message->length;i++)
			{
				tcp_socket_control.message->buffer[i]=str[i];
			}
			tcp_socket_control.transmit();
		}

		if(this->role==COMMANDER)
		{
			// Stay as commander, retry connection at init config.
			// The responder side already dropped to LISTENING.
			// connection_attempt_timeout handler will give up after max retries.
			printf("[LINK-TIMEOUT] Commander retrying connection at init config\n");
			fflush(stdout);
			// Re-save NB preference (reset_session_state cleared it)
			commander_configured_nb = narrowband_enabled;
			load_configuration(init_configuration, FULL, YES);
			link_status=CONNECTING;
			connection_status=TRANSMITTING_CONTROL;

			// Reset turboshift for fresh probe on reconnect
			turboshift_active = true;
			turboshift_phase = TURBO_DONE;
			turboshift_last_good = -1;
			turbo_settle_pending = false;
			supershift_proven_ceiling = -1;
			turbo_snr_ack_enabled = false;
			turbo_received_snr = -99.0f;

			messages_control.status = FREE;
			connection_attempts = 0;
			connection_attempt_timer.reset();
			connection_attempt_timer.start();
			// Don't add_message_control here — CONNECTING flow in
			// process_messages_commander handles NB probe + START_CONNECTION
		}
		else if(this->role==RESPONDER)
		{
			link_status=LISTENING;
			connection_status=RECEIVING;
			load_configuration(init_configuration, FULL, YES);
		}
	}

	if(watchdog_timer.get_elapsed_time_ms()>= watchdog_timeout)
	{
		if(original_role==COMMANDER)
		{
			set_role(COMMANDER);
			link_status=CONNECTED;
			connection_status=TRANSMITTING_DATA;
			for(int i=0;i<nMessages;i++)
			{
				messages_tx[i].status=FREE;
			}

			char restore_buf[N_MAX/8 * 20];
			int total_restore = 0;
			int data_read_size;
			for(int i=0;i<get_nTotal_messages();i++)
			{
				data_read_size=fifo_buffer_backup.pop(restore_buf + total_restore,max_data_length+max_header_length);
				if(data_read_size!=0)
				{
					total_restore += data_read_size;
				}
				else
				{
					break;
				}
			}
			if(total_restore > 0)
				fifo_buffer_tx.push_front(restore_buf, total_restore);
			fifo_buffer_backup.flush();

		}
		else if(original_role==RESPONDER)
		{
			set_role(RESPONDER);
			link_status=CONNECTED;
			connection_status=RECEIVING;

			for(int i=0;i<nMessages;i++)
			{
				messages_rx[i].status=FREE;
			}
		}

		last_data_configuration=data_configuration;
		load_configuration(data_configuration,PHYSICAL_LAYER_ONLY,YES);

		gear_shift_blocked_for_nBlocks=0;  // Force cooldown: wait N blocks before shifting up again

		watchdog_timer.stop();
		watchdog_timer.reset();
		watchdog_timer.start();
		gear_shift_timer.stop();
		gear_shift_timer.reset();
		receiving_timer.stop();
		receiving_timer.reset();

	}

	// Fallback: if we're in COMMANDER mode and haven't received anything for 60+ seconds
	// while supposedly connected, force switch to RESPONDER mode to break infinite loops
	const int FORCED_ROLE_SWITCH_TIMEOUT = 180000;  // 180 seconds (6 BREAK recovery cycles at ~25s each)
	if(role==COMMANDER && link_status==CONNECTED &&
	   receiving_timer.get_elapsed_time_ms() >= FORCED_ROLE_SWITCH_TIMEOUT)
	{
		printf("Forced role switch: no RX for %d seconds, switching to RESPONDER\n", FORCED_ROLE_SWITCH_TIMEOUT/1000);

		reset_session_state();
		set_role(RESPONDER);
		link_status=LISTENING;
		connection_status=RECEIVING;
		load_configuration(init_configuration, FULL, YES);
		reset_all_timers();

		// Flush buffers
		fifo_buffer_tx.flush();
		fifo_buffer_backup.flush();
		fifo_buffer_rx.flush();
		messages_control.status=FREE;
	}

	if(gear_shift_on==YES && gear_shift_timer.get_elapsed_time_ms()>=gearshift_timeout)
	{
		gear_shift_timer.stop();
		gear_shift_timer.reset();

		if(gear_shift_algorithm==SNR_BASED)
		{
			messages_control_backup();
			load_configuration(init_configuration,PHYSICAL_LAYER_ONLY,YES);
			messages_control_restore();

			if(this->role==COMMANDER)
			{
				for(int i=0;i<nMessages;i++)
				{
					messages_tx[i].status=FREE;
				}

				char restore_buf[N_MAX/8 * 20];
				int total_restore = 0;
				int data_read_size;
				for(int i=0;i<get_nTotal_messages();i++)
				{
					data_read_size=fifo_buffer_backup.pop(restore_buf + total_restore,max_data_length+max_header_length);
					if(data_read_size!=0)
					{
						total_restore += data_read_size;
					}
					else
					{
						break;
					}
				}
				if(total_restore > 0)
					fifo_buffer_tx.push_front(restore_buf, total_restore);
				fifo_buffer_backup.flush();

				if (current_configuration!= last_data_configuration)
				{
					add_message_control(TEST_CONNECTION);
					gear_shift_timer.start();
					connection_status=TRANSMITTING_CONTROL;
				}
				else
				{
					connection_status=TRANSMITTING_DATA;
				}
			}
			else if(this->role==RESPONDER)
			{
				for(int i=0;i<nMessages;i++)
				{
					messages_rx[i].status=FREE;
				}

				connection_status=RECEIVING;
			}

		}
		else if(gear_shift_algorithm==SUCCESS_BASED_LADDER)
		{
			// During turboshift, retry/ceiling is handled in the control NAck
			// handler (arq_commander.cc). Skip the normal gearshift-down here.
			// During BREAK recovery, the SET_CONFIG for recovery is in-flight.
			// gearshift_timeout must not overwrite data_configuration or connection_status.
			if((turboshift_active || break_recovery_phase != 0 || emergency_break_active
				|| turboshift_phase != TURBO_DONE) && this->role==COMMANDER)
			{
				// SWITCH_ROLE timeout during turboshift: the other side likely
				// received SWITCH_ROLE and already became commander. Assume it
				// was received and become responder so we hear their frames.
				if(!turboshift_active && break_recovery_phase == 0
					&& !emergency_break_active && turboshift_phase != TURBO_DONE)
				{
					printf("[TURBO] SWITCH_ROLE timeout — assuming received, becoming responder\n");
					fflush(stdout);
					set_role(RESPONDER);
					link_status = CONNECTED;
					connection_status = RECEIVING;
					watchdog_timer.start();
					link_timer.start();
					telecom_system->data_container.frames_to_read =
						telecom_system->data_container.preamble_nSymb
						+ telecom_system->data_container.Nsymb;
					telecom_system->data_container.nUnder_processing_events = 0;
					telecom_system->receive_stats.mfsk_search_raw = 0;
					telecom_system->receive_stats.ofdm_search_raw = 0;
					telecom_system->receive_stats.ofdm_batch_active = false;
				}
				else
				{
					printf("[GEARSHIFT] Timeout during turboshift/break-recovery — skipping\n");
					fflush(stdout);
				}
				return;
			}

			messages_control_backup();
			data_configuration=config_ladder_down(current_configuration, robust_enabled);
			load_configuration(data_configuration,PHYSICAL_LAYER_ONLY,YES);
			messages_control_restore();

			if(this->role==COMMANDER)
			{
				gear_shift_blocked_for_nBlocks=0;

				for(int i=0;i<nMessages;i++)
				{
					messages_tx[i].status=FREE;
				}

				char restore_buf[N_MAX/8 * 20];
				int total_restore = 0;
				int data_read_size;
				for(int i=0;i<get_nTotal_messages();i++)
				{
					data_read_size=fifo_buffer_backup.pop(restore_buf + total_restore,max_data_length+max_header_length);
					if(data_read_size!=0)
					{
						total_restore += data_read_size;
					}
					else
					{
						break;
					}
				}
				if(total_restore > 0)
					fifo_buffer_tx.push_front(restore_buf, total_restore);
				fifo_buffer_backup.flush();

				connection_status=TRANSMITTING_DATA;
			}
			else if(this->role==RESPONDER)
			{
				for(int i=0;i<nMessages;i++)
				{
					messages_rx[i].status=FREE;
				}

				connection_status=RECEIVING;
			}
		}
	}

	if(!turboshift_active && switch_role_test_timer.get_elapsed_time_ms()>switch_role_test_timeout)
	{
		switch_role_test_timer.stop();
		switch_role_test_timer.reset();

		set_role(RESPONDER);
		this->link_status=CONNECTED;
		this->connection_status=RECEIVING;

		this->messages_control.ack_timeout=0;
		this->messages_control.id=0;
		this->messages_control.length=0;
		this->messages_control.nResends=0;
		this->messages_control.status=FREE;
		this->messages_control.type=NONE;

	}


	if(print_stats_timer.get_elapsed_time_ms()>(int)(1000.0/print_stats_frequency_hz))
	{
		print_stats_timer.start();
		print_stats();
	}

}

void cl_arq_controller::cleanup()
{

	if(messages_control.status==ACKED)
	{
		// SEND ACK TO USER
		this->messages_control.ack_timeout=0;
		this->messages_control.id=0;
		this->messages_control.length=0;
		this->messages_control.nResends=0;
		this->messages_control.status=FREE;
		this->messages_control.type=NONE;
	}
	else if(messages_control.status==FAILED_)
	{
		// SEND FAILED TO USER
		this->messages_control.ack_timeout=0;
		this->messages_control.id=0;
		this->messages_control.length=0;
		this->messages_control.nResends=0;
		this->messages_control.status=FREE;
		this->messages_control.type=NONE;
	}


	for(int i=0;i<this->nMessages;i++)
	{
		if(messages_tx[i].status==ACKED)
		{
			// SEND ACK TO USER
			this->messages_tx[i].ack_timeout=0;
			this->messages_tx[i].id=0;
			this->messages_tx[i].length=0;
			this->messages_tx[i].nResends=0;
			this->messages_tx[i].status=FREE;
			this->messages_tx[i].type=NONE;
		}
		else if(messages_tx[i].status==FAILED_)
		{
			// SEND FAILED TO USER
			this->messages_tx[i].ack_timeout=0;
			this->messages_tx[i].id=0;
			this->messages_tx[i].length=0;
			this->messages_tx[i].nResends=0;
			this->messages_tx[i].status=FREE;
			this->messages_tx[i].type=NONE;
		}
	}


}


void cl_arq_controller::pad_messages_batch_tx(int size)
{
	// Start from last unique message so it gets a duplicate first,
	// then wrap to 0. With 12 unique + 11 dups: IDs 0-9,11 get 2 copies,
	// only ID 10 has 1 copy (less critical than ID 0 header or last frame).
	int counter = (message_batch_counter_tx > 0) ? message_batch_counter_tx - 1 : 0;
	if(message_batch_counter_tx!=0 && message_batch_counter_tx<size)
	{
		for(int i=0;i<size-message_batch_counter_tx;i++)
		{
			int slot = i + message_batch_counter_tx;
			messages_batch_tx[slot]=messages_batch_tx[counter];
			// Assign unique ID so RSP stores each frame in a distinct slot
			// (duplicate data is fine, but colliding IDs break SACK bitmap).
			messages_batch_tx[slot].id = slot;
			counter++;
			if(counter>=message_batch_counter_tx)
			{
				counter=0;
			}
		}
		message_batch_counter_tx=size;
	}
}

void cl_arq_controller::process_main()
{
	std::string command="";

	if (tcp_socket_control.get_status()==TCP_STATUS_ACCEPTED)
	{
		// Mark that we had a control connection
		had_control_connection=YES;

		if(tcp_socket_control.timer.counting==0)
		{
			tcp_socket_control.timer.start();
		}
		int nBytes_received=tcp_socket_control.receive();
		if(nBytes_received>0)
		{
			tcp_socket_control.timer.start();

			for(int i=0;i<tcp_socket_control.message->length;i++)
			{
				user_command_buffer+=tcp_socket_control.message->buffer[i];
			}
		}
		else if(nBytes_received==0 || (tcp_socket_control.timer.get_elapsed_time_ms()>=tcp_socket_control.timeout_ms && tcp_socket_control.timeout_ms!=INFINITE_))
		{
			// Check if client disconnected cleanly (nBytes_received==0)
			if(nBytes_received==0 && exit_on_disconnect==YES && had_control_connection==YES)
			{
				std::cout<<std::endl;
				std::cout<<"Control connection closed by client - exiting as requested"<<std::endl;
				exit(0);
			}

			fifo_buffer_tx.flush();
			fifo_buffer_backup.flush();
			fifo_buffer_rx.flush();

			tcp_socket_control.check_incomming_connection();
			if (tcp_socket_control.get_status()==TCP_STATUS_ACCEPTED)
			{
				tcp_socket_control.timer.start();
			}

		}
		size_t pos=std::string::npos;
		do
		{
			// Strip any leading \n characters (from Windows \r\n line endings)
			while(!user_command_buffer.empty() && user_command_buffer[0]=='\n')
			{
				user_command_buffer=user_command_buffer.substr(1);
			}

			size_t pos=user_command_buffer.find('\r');
			if(pos!=std::string::npos)
			{
				command=user_command_buffer.substr(0, pos);
				process_user_command(command);
				user_command_buffer=user_command_buffer.substr(pos+1,std::string::npos);
			}
		}while(pos!=std::string::npos);

	}
	else
	{
		tcp_socket_control.check_incomming_connection();
		if (tcp_socket_control.get_status()==TCP_STATUS_ACCEPTED)
		{
			tcp_socket_control.timer.start();
		}
	}


	if (tcp_socket_data.get_status()==TCP_STATUS_ACCEPTED)
	{
		if(tcp_socket_data.timer.counting==0)
		{
			tcp_socket_data.timer.start();
		}
		// Only receive from TCP if FIFO has room for the max recv size.
		// Otherwise data pulled from the socket would be silently dropped
		// by push() (returns 0 when full). Leaving data in the TCP socket
		// buffer creates natural backpressure to the sender.
		if(fifo_buffer_tx.get_free_size() >= MAX_BUFFER_SIZE)
		{
			int nBytes_received=tcp_socket_data.receive();
			if(nBytes_received>0)
			{
				tcp_socket_data.timer.start();

				// B2F filter: parse outgoing stream, unroll LZHUF payloads
				if(b2f_handler.is_initialized())
				{
					char b2f_buf[MAX_BUFFER_SIZE * 4]; // plaintext can be larger than LZHUF
					int b2f_len = b2f_handler.filter_tx(
						tcp_socket_data.message->buffer,
						tcp_socket_data.message->length,
						b2f_buf, sizeof(b2f_buf));

					// Auto-arm compression when B2F SID detected (Winlink traffic)
					if(!compression_enabled && !force_compress &&
					   !b2f_compression_pending && b2f_handler.is_b2f_session())
					{
						bool both_support = (local_capability & CAP_COMPRESSION) &&
						                    (peer_capability & CAP_COMPRESSION);
						if(both_support)
						{
							b2f_compression_pending = true;
							printf("[COMPRESS] B2F detected — will arm on next ACK\n");
							fflush(stdout);
						}
					}

					if(b2f_len > 0)
						fifo_buffer_tx.push(b2f_buf, b2f_len);
					else if(!b2f_handler.is_b2f_session())
						fifo_buffer_tx.push(tcp_socket_data.message->buffer, tcp_socket_data.message->length);
					// else: B2F active, parser accumulating partial line -- skip raw push
				}
				else
				{
					fifo_buffer_tx.push(tcp_socket_data.message->buffer, tcp_socket_data.message->length);
				}

				std::string str="BUFFER ";
				str+=std::to_string(fifo_buffer_tx.get_size()-fifo_buffer_tx.get_free_size());
				str+='\r';
				for(long unsigned int i=0;i<str.length();i++)
				{
					tcp_socket_control.message->buffer[i]=str[i];
				}
				tcp_socket_control.message->length=str.length();
				tcp_socket_control.transmit();
			}
			else if(nBytes_received==0)
			{
				// TCP connection closed (FIN received) — flush and re-accept
				fifo_buffer_tx.flush();
				fifo_buffer_backup.flush();
				fifo_buffer_rx.flush();

				tcp_socket_data.check_incomming_connection();

				if (tcp_socket_data.get_status()==TCP_STATUS_ACCEPTED)
				{
					tcp_socket_data.timer.start();
				}
			}
			else if(role == COMMANDER &&
				tcp_socket_data.timer.get_elapsed_time_ms()>=tcp_socket_data.timeout_ms &&
				tcp_socket_data.timeout_ms!=INFINITE_)
			{
				// Commander only: timeout waiting for data from TCP client.
				// Responder skips this — its data client is read-only (never sends),
				// so recv() always returns EWOULDBLOCK. The old code treated this as
				// a disconnect, flushing buffers and dropping the connection every
				// 1 second. (Bug #61: responder data delivery)
				fifo_buffer_tx.flush();
				fifo_buffer_backup.flush();
				fifo_buffer_rx.flush();

				tcp_socket_data.check_incomming_connection();

				if (tcp_socket_data.get_status()==TCP_STATUS_ACCEPTED)
				{
					tcp_socket_data.timer.start();
				}
			}
		}

	}
	else
	{
		tcp_socket_data.check_incomming_connection();
		if (tcp_socket_data.get_status()==TCP_STATUS_ACCEPTED)
		{
			tcp_socket_data.timer.start();
		}
	}

	// Signal measurement when idle: measure_signal_only() uses FIR_rx_time_sync,
	// the same filter that receive_byte() uses for preamble detection. Running both
	// on the same iteration corrupts the FIR delay line state, making Schmidl-Cox
	// GI correlation fail (Bug #28). Only run when receive() is NOT called.
	// LISTENING has active receive() calls → signal strength comes from receive_byte().
	if(link_status == IDLE || link_status == DROPPED)
	{
		MUTEX_LOCK(&capture_prep_mutex);
		if(telecom_system->data_container.frames_to_read == 0)
		{
			int signal_period = telecom_system->data_container.Nofdm *
				telecom_system->data_container.buffer_Nsymb *
				telecom_system->data_container.interpolation_rate;

			int rwi = telecom_system->data_container.ring_write_index;
			memcpy(telecom_system->data_container.ready_to_process_passband_delayed_data,
				&telecom_system->data_container.passband_delayed_data[rwi],
				signal_period * sizeof(double));

			MUTEX_UNLOCK(&capture_prep_mutex);

#ifdef IDLE_GATE_TRACE
			// Step-0 instrumentation (IDLE_SCAN_CADENCE_RESEARCH.md §6 Step 0):
			// measure the raw-passband RMS of the buffer the Step-3 gate would
			// probe, and count how often this IDLE-loop FIR path runs. File-
			// static counters only (no class members — ODR hazard, see Plan B
			// §10). Whole #ifdef block is compiled out of the default build, so
			// the binary without -DIDLE_GATE_TRACE is byte-identical to baseline.
			{
				const double* idle_buf =
					telecom_system->data_container.ready_to_process_passband_delayed_data;
				double idle_sumsq = 0.0;
				for(int i = 0; i < signal_period; i++)
					idle_sumsq += idle_buf[i] * idle_buf[i];
				double idle_rms = std::sqrt(idle_sumsq / (double)signal_period);

				static long long idle_gate_fir_runs = 0;   // FIR (measure_signal_only) invocations
				static double    idle_gate_rms_sum  = 0.0;  // running RMS sum for mean
				static double    idle_gate_rms_min  = 1e30;
				static double    idle_gate_rms_max  = 0.0;
				static long long idle_gate_rms_n    = 0;    // RMS samples (== FIR runs here)
				static long long idle_gate_last_ms  = -1;

				idle_gate_fir_runs++;
				idle_gate_rms_n++;
				idle_gate_rms_sum += idle_rms;
				if(idle_rms < idle_gate_rms_min) idle_gate_rms_min = idle_rms;
				if(idle_rms > idle_gate_rms_max) idle_gate_rms_max = idle_rms;

				long long now_ms = mtl::now_ms();
				if(idle_gate_last_ms < 0) idle_gate_last_ms = now_ms;
				if(now_ms - idle_gate_last_ms >= 5000)
				{
					mtl::log_event_kv("idle_gate_trace",
						"fir_runs=%lld n=%lld rms_mean=%.6f rms_min=%.6f rms_max=%.6f rms_last=%.6f",
						idle_gate_fir_runs, idle_gate_rms_n,
						idle_gate_rms_n ? idle_gate_rms_sum / (double)idle_gate_rms_n : 0.0,
						idle_gate_rms_min, idle_gate_rms_max, idle_rms);
					idle_gate_last_ms = now_ms;
				}
			}
#endif

			measurements.signal_stregth_dbm = telecom_system->measure_signal_only(
				telecom_system->data_container.ready_to_process_passband_delayed_data);
		}
		else
		{
			MUTEX_UNLOCK(&capture_prep_mutex);
		}
	}

	process_messages();
	usleep(2000);
}

void cl_arq_controller::process_user_command(std::string command)
{

	if(command.substr(0,7)=="MYCALL ")
	{
		this->my_call_sign=command.substr(7);

		tcp_socket_control.message->buffer[0]='O';
		tcp_socket_control.message->buffer[1]='K';
		tcp_socket_control.message->buffer[2]='\r';
		tcp_socket_control.message->length=3;
	}
	else if(command.substr(0,8)=="CONNECT ")
	{
		command=command.substr(8,std::string::npos);
		this->my_call_sign=command.substr(0,command.find(" "));
		this->destination_call_sign=command.substr(my_call_sign.length()+1);
		commander_configured_nb=narrowband_enabled;
		local_capability = ((bandwidth_mode == BW_AUTO) ? CAP_WB_CAPABLE : 0) | CAP_COMPRESSION | CAP_B2F_UNROLL | CAP_STREAMING | CAP_SACK | ((encryption_mode != ENCRYPT_OFF) ? CAP_ENCRYPTION : 0);
		if(disable_sack) local_capability &= ~CAP_SACK;  // Phase-2 --no-sack
		if(enable_sack_v2) local_capability |= CAP_SACK_V2;  // Step 6 scaffolding
		peer_capability = 0;
		wb_upgrade_pending = false;
		compression_enabled = false;
		original_role=COMMANDER;
		set_role(COMMANDER);
		link_status=CONNECTING;
		reset_all_timers();

		// Reset messages_control so new connection can add START_CONNECTION
		messages_control.status=FREE;

		// Start connection attempt timer and reset counter
		connection_attempts=0;
		connection_attempt_timer.reset();
		connection_attempt_timer.start();

		// Send OK acknowledgement
		tcp_socket_control.message->buffer[0]='O';
		tcp_socket_control.message->buffer[1]='K';
		tcp_socket_control.message->buffer[2]='\r';
		tcp_socket_control.message->length=3;
		tcp_socket_control.transmit();

		// Send PENDING status to indicate connection attempt is starting
		std::string str="PENDING\r";
		tcp_socket_control.message->length=str.length();
		for(int i=0;i<tcp_socket_control.message->length;i++)
		{
			tcp_socket_control.message->buffer[i]=str[i];
		}
		// Note: transmit() will be called in process_main after all commands are processed
	}
	else if(command=="DISCONNECT")
	{
		disconnect_requested=YES;

		tcp_socket_control.message->buffer[0]='O';
		tcp_socket_control.message->buffer[1]='K';
		tcp_socket_control.message->buffer[2]='\r';
		tcp_socket_control.message->length=3;
	}
	else if(command=="ABORT")
	{
		// Abort connection attempt or active session - immediate teardown
		if(link_status==CONNECTING || link_status==NEGOTIATING || link_status==CONNECTION_ACCEPTED
			|| link_status==CONNECTED || link_status==DISCONNECTING)
		{
			printf("[ABORT] Aborting session (link_status=%d)\n", link_status);
			fflush(stdout);

			// Immediate teardown — no CLOSE_CONNECTION negotiation
			reset_session_state();

			set_role(RESPONDER);
			link_status=LISTENING;
			connection_status=RECEIVING;
			load_configuration(init_configuration, FULL, YES);

			// Clear buffers and reset timers
			fifo_buffer_tx.flush();
			fifo_buffer_backup.flush();
			fifo_buffer_rx.flush();
			reset_all_timers();

			// Reset messages_control so new CONNECT commands can work
			messages_control.status=FREE;

			// Send CANCELPENDING to cancel the connection attempt
			std::string str="CANCELPENDING\r";
			tcp_socket_control.message->length=str.length();
			for(int i=0;i<tcp_socket_control.message->length;i++)
			{
				tcp_socket_control.message->buffer[i]=str[i];
			}
			tcp_socket_control.transmit();

			// Send DISCONNECTED to fully clear Winlink's state and show we're free
			str="DISCONNECTED\r";
			tcp_socket_control.message->length=str.length();
			for(int i=0;i<tcp_socket_control.message->length;i++)
			{
				tcp_socket_control.message->buffer[i]=str[i];
			}
			tcp_socket_control.transmit();
		}

		// Send OK acknowledgement
		tcp_socket_control.message->buffer[0]='O';
		tcp_socket_control.message->buffer[1]='K';
		tcp_socket_control.message->buffer[2]='\r';
		tcp_socket_control.message->length=3;
	}
	else if(command=="LISTEN ON")
	{
		original_role=RESPONDER;
		set_role(RESPONDER);
		local_capability = ((bandwidth_mode == BW_AUTO) ? CAP_WB_CAPABLE : 0) | CAP_COMPRESSION | CAP_B2F_UNROLL | CAP_STREAMING | CAP_SACK | ((encryption_mode != ENCRYPT_OFF) ? CAP_ENCRYPTION : 0);
		if(disable_sack) local_capability &= ~CAP_SACK;  // Phase-2 --no-sack
		if(enable_sack_v2) local_capability |= CAP_SACK_V2;  // Step 6 scaffolding
		peer_capability = 0;
		wb_upgrade_pending = false;
		compression_enabled = false;
		link_status=LISTENING;
		connection_status=RECEIVING;
		reset_session_state();
		reset_all_timers();

		// Load init_configuration so we can hear incoming START_CONNECTION messages
		load_configuration(init_configuration, FULL, YES);

		tcp_socket_control.message->buffer[0]='O';
		tcp_socket_control.message->buffer[1]='K';
		tcp_socket_control.message->buffer[2]='\r';
		tcp_socket_control.message->length=3;
	}
	else if(command=="LISTEN OFF")
	{
		original_role=RESPONDER;
		set_role(RESPONDER);
		link_status=IDLE;
		connection_status=IDLE;
		reset_all_timers();

		tcp_socket_control.message->buffer[0]='O';
		tcp_socket_control.message->buffer[1]='K';
		tcp_socket_control.message->buffer[2]='\r';
		tcp_socket_control.message->length=3;
	}
	else if(command=="BW500")
	{
		// Narrowband only mode (500 Hz, Nc=10)
		printf("[BW] Setting NB only (500 Hz)\n");
		fflush(stdout);
		bandwidth_mode = BW_NB_ONLY;
		local_capability = CAP_COMPRESSION | CAP_B2F_UNROLL | CAP_STREAMING | CAP_SACK | ((encryption_mode != ENCRYPT_OFF) ? CAP_ENCRYPTION : 0);
		if(disable_sack) local_capability &= ~CAP_SACK;  // Phase-2 --no-sack
		if(enable_sack_v2) local_capability |= CAP_SACK_V2;  // Step 6 scaffolding
#ifdef MERCURY_GUI_ENABLED
		g_gui_state.bandwidth_mode.store(BW_NB_ONLY);
#endif
		if(narrowband_enabled != YES)
			switch_narrowband_mode(YES);

		tcp_socket_control.message->buffer[0]='O';
		tcp_socket_control.message->buffer[1]='K';
		tcp_socket_control.message->buffer[2]='\r';
		tcp_socket_control.message->length=3;
	}
	else if(command=="BW2300" || command=="BW2750")
	{
		// Auto mode (start NB, upgrade to WB if peer supports)
		printf("[BW] Setting auto mode (%s)\n", command.c_str());
		fflush(stdout);
		bandwidth_mode = BW_AUTO;
		local_capability = CAP_WB_CAPABLE | CAP_COMPRESSION | CAP_B2F_UNROLL | CAP_STREAMING | CAP_SACK | ((encryption_mode != ENCRYPT_OFF) ? CAP_ENCRYPTION : 0);
		if(disable_sack) local_capability &= ~CAP_SACK;  // Phase-2 --no-sack
		if(enable_sack_v2) local_capability |= CAP_SACK_V2;  // Step 6 scaffolding
#ifdef MERCURY_GUI_ENABLED
		g_gui_state.bandwidth_mode.store(BW_AUTO);
#endif
		// Start in NB (auto-negotiation will upgrade if peer supports WB)
		if(narrowband_enabled != YES)
			switch_narrowband_mode(YES);

		tcp_socket_control.message->buffer[0]='O';
		tcp_socket_control.message->buffer[1]='K';
		tcp_socket_control.message->buffer[2]='\r';
		tcp_socket_control.message->length=3;
	}
	else if(command=="BW2500")
	{
		// Legacy command — treat same as BW2300
		printf("[BW] Setting auto mode (BW2500, legacy)\n");
		fflush(stdout);
		bandwidth_mode = BW_AUTO;
		local_capability = CAP_WB_CAPABLE | CAP_COMPRESSION | CAP_B2F_UNROLL | CAP_STREAMING | CAP_SACK | ((encryption_mode != ENCRYPT_OFF) ? CAP_ENCRYPTION : 0);
		if(disable_sack) local_capability &= ~CAP_SACK;  // Phase-2 --no-sack
		if(enable_sack_v2) local_capability |= CAP_SACK_V2;  // Step 6 scaffolding
#ifdef MERCURY_GUI_ENABLED
		g_gui_state.bandwidth_mode.store(BW_AUTO);
#endif
		if(narrowband_enabled != YES)
			switch_narrowband_mode(YES);

		tcp_socket_control.message->buffer[0]='O';
		tcp_socket_control.message->buffer[1]='K';
		tcp_socket_control.message->buffer[2]='\r';
		tcp_socket_control.message->length=3;
	}
	else if(command=="VERSION")
	{
		std::string reply="VERSION Mercury " VERSION__ "\r";
		for(long unsigned int i=0;i<reply.length();i++)
		{
			tcp_socket_control.message->buffer[i]=reply[i];
		}
		tcp_socket_control.message->length=reply.length();
	}
	else if(command=="BUFFER TX")
	{
		std::string reply="BUFFER ";
		reply+=std::to_string(fifo_buffer_tx.get_size()-fifo_buffer_tx.get_free_size());
		reply+='\r';
		for(long unsigned int i=0;i<reply.length();i++)
		{
			tcp_socket_control.message->buffer[i]=reply[i];
		}
		tcp_socket_control.message->length=reply.length();
	}
	else if(command.substr(0,9)=="NOISESNR ")
	{
		// Dynamic noise injection control: "NOISESNR <db>" or "NOISESNR OFF"
		std::string arg=command.substr(9);
		if(arg=="OFF" || arg=="off") {
			noise_snr_db = 999.0;
			printf("[NOISE-Z] Noise OFF\n");
		} else {
			noise_snr_db = std::stod(arg);
			printf("[NOISE-Z] SNR set to %.1f dB\n", noise_snr_db);
		}
		tcp_socket_control.message->buffer[0]='O';
		tcp_socket_control.message->buffer[1]='K';
		tcp_socket_control.message->buffer[2]='\r';
		tcp_socket_control.message->length=3;
	}
	else if(command.substr(0,12)=="NOISESIGNAL ")
	{
		// Set expected wire signal level for noise calibration: "NOISESIGNAL <dBFS>"
		std::string arg=command.substr(12);
		noise_signal_dbfs = std::stod(arg);
		printf("[NOISE-Z] Signal level set to %.1f dBFS\n", noise_signal_dbfs);
		tcp_socket_control.message->buffer[0]='O';
		tcp_socket_control.message->buffer[1]='K';
		tcp_socket_control.message->buffer[2]='\r';
		tcp_socket_control.message->length=3;
	}
	else
	{
		tcp_socket_control.message->buffer[0]='O';
		tcp_socket_control.message->buffer[1]='K';
		tcp_socket_control.message->buffer[2]='\r';
		tcp_socket_control.message->length=3;
	}

	if (tcp_socket_control.get_status()==TCP_STATUS_ACCEPTED)
	{
		tcp_socket_control.transmit();
	}
}

void cl_arq_controller::ptt_on()
{
	if(passive_monitor) return;  // Never transmit in monitor mode
	std::string str="PTT ON\r";
	tcp_socket_control.message->length=str.length();

	for(int i=0;i<tcp_socket_control.message->length;i++)
	{
		tcp_socket_control.message->buffer[i]=str[i];
	}
	tcp_socket_control.transmit();
}
void cl_arq_controller::ptt_off()
{
	std::string str="PTT OFF\r";
	tcp_socket_control.message->length=str.length();

	for(int i=0;i<tcp_socket_control.message->length;i++)
	{
		tcp_socket_control.message->buffer[i]=str[i];
	}
	tcp_socket_control.transmit();
}



void cl_arq_controller::process_messages()
{
	this->update_status();
	if(this->role==COMMANDER)
	{
		process_messages_commander();
		process_buffer_data_commander();
	}
	else if(this->role==RESPONDER)
	{
		process_messages_responder();
		process_buffer_data_responder();
	}
}

void cl_arq_controller::reset_all_timers()
{
	link_timer.stop();
	link_timer.reset();
	watchdog_timer.stop();
	watchdog_timer.reset();
	gear_shift_timer.stop();
	gear_shift_timer.reset();
	receiving_timer.stop();
	receiving_timer.reset();
	switch_role_timer.stop();
	switch_role_timer.reset();
}

void cl_arq_controller::reset_session_state()
{
	// Config state — must match init() defaults
	negotiated_configuration = init_configuration;
	data_configuration = init_configuration;
	forward_configuration = CONFIG_NONE;
	reverse_configuration = CONFIG_NONE;
	ack_configuration = init_configuration;

	// Turboshift — fresh state for next connection
	turboshift_phase = TURBO_FORWARD;
	turboshift_active = true;
	turboshift_last_good = -1;
	turbo_settle_pending = false;
	turboshift_initiator = false;
	turboshift_retries = 1;
	supershift_proven_ceiling = -1;
	turbo_snr_ack_enabled = false;
	turbo_received_snr = -99.0f;
	turbo_switch_role_retries = 0;

	// BREAK / recovery
	emergency_nack_count = 0;
	emergency_break_active = 0;
	emergency_break_retries = 3;
	emergency_previous_config = init_configuration;
	break_drop_step = 1;
	break_recovery_phase = 0;
	break_recovery_retries = 0;
	ceiling_success_count = 0;
	break_detected = NO;
	hail_detected = NO;
	hail_sent = NO;

	// Compression — always deinit (safe if not initialized; handles deferred pre-init)
	compressor.deinit();
	compression_enabled = false;
	b2f_compression_pending = false;

	// B2F handler — reset state for next connection
	b2f_handler.reset();

	// Encryption — wipe all key material (volatile memset, compiler can't elide)
	cipher_suite.wipe();
	encryption_enabled = false;
#ifdef MERCURY_GUI_ENABLED
	g_gui_state.encryption_active.store(false);
	// Don't clear psk_mismatch here — let it persist so the GUI shows the error.
	// It gets cleared on next successful encryption activation.
#endif
	tx_batch_counter = 0;
	rx_batch_counter = 0;
	consecutive_auth_failures = 0;
	if (kx_data_buf) { free(kx_data_buf); kx_data_buf = NULL; }
	kx_data_len = 0;

	// Data exchange
	block_under_tx = NO;
	consecutive_data_acks = 0;
	frame_gearshift_just_applied = false;
	data_ack_received = NO;
	repeating_last_ack = NO;

	// Message tracking
	last_message_sent_type = NONE;
	last_message_sent_code = NONE;
	last_message_received_type = NONE;
	last_message_received_code = NONE;
	last_received_message_sequence = 255;

	// Return to NB after session ends — NB is the discovery/HAIL mode.
	// Skip when nb_probe_max==0: commander skips NB probing and hails in WB directly,
	// so responder must also stay in WB to detect WB HAIL patterns.
	if(nb_probe_max > 0 || bandwidth_mode == BW_NB_ONLY)
	{
		if(narrowband_enabled != YES)
		{
			printf("[NB-SWITCH] Restoring narrowband after session end\n");
			fflush(stdout);
		}
		narrowband_enabled = YES;
		telecom_system->narrowband_enabled = YES;
	}
	current_configuration = CONFIG_NONE;
	telecom_system->current_configuration = CONFIG_NONE;
	commander_configured_nb = -1;
	session_narrowband = false;
	peer_capability = 0;
	wb_upgrade_pending = false;

	// Connection
	connection_id = 0;
	assigned_connection_id = 0;
	connection_attempts = 0;
	disconnect_requested = NO;
}

void cl_arq_controller::switch_narrowband_mode(int nb_enabled)
{
	if(narrowband_enabled == nb_enabled)
		return;
	printf("[NB-SWITCH] Switching to %s mode (init_config=%d, cur_config=%d)\n",
		nb_enabled ? "narrowband" : "wideband", init_configuration, current_configuration);
	fflush(stdout);

	// Pause audio processing before the switch: set data_ready=0 so the
	// processing thread won't start a new receive() cycle, and stop the
	// capture_prep thread from accumulating nUnder during the transition.
	telecom_system->data_container.data_ready = 0;
	// Reset nUnder and anti-re-decode markers from the previous bandwidth
	// mode. Stale nUnder from large NB frames (Nsymb=80) would corrupt the
	// first frames_to_read calculation after switching to WB.
	telecom_system->data_container.nUnder_processing_events = 0;
	telecom_system->receive_stats.ofdm_search_raw = 0;
	telecom_system->receive_stats.mfsk_search_raw = 0;

	narrowband_enabled = nb_enabled;
	telecom_system->narrowband_enabled = nb_enabled;
	// Force reload by clearing current config on BOTH ARQ and PHY
	// (telecom_system skips load_configuration if config number matches,
	// even though narrowband_enabled changed the physical parameters)
	current_configuration = CONFIG_NONE;
	telecom_system->current_configuration = CONFIG_NONE;

	printf("[NB-SWITCH] Calling load_configuration(%d, FULL, YES)\n", init_configuration);
	fflush(stdout);
	load_configuration(init_configuration, FULL, YES);
	printf("[NB-SWITCH] load_configuration complete, cur_config=%d Nc=%d Nsymb=%d Nofdm=%d\n",
		current_configuration, telecom_system->ofdm.Nc,
		telecom_system->ofdm.Nsymb, telecom_system->data_container.Nofdm);
	fflush(stdout);
}


void cl_arq_controller::send(st_message* message, int message_location)
{
	printf("send()\n");

	int header_length=0;
	if(message->type==DATA_LONG)
	{
		// SACK Design A Steps 1+3 — DATA_LONG header growth + batch_seq_id plumbing.
		// v1 (default): 4 bytes [type, conn_id, seq(EOB bit7), id].
		// v2: 5 bytes [type, conn_id, seq(EOB bit7), batch_seq_id, id].
		// batch_seq_id source (v2 only): message->batch_seq_id field, masked mod 256.
		// Sentinel -1 (unset, defensive — should not happen for v2 DATA TX) → 0.
		message_TxRx_byte_buffer[0]=message->type;
		message_TxRx_byte_buffer[1]=connection_id;
		message_TxRx_byte_buffer[2]=message->sequence_number;
		if(sack_v2_enabled)
		{
			int bsi = message->batch_seq_id;
			if(bsi < 0) bsi = 0;  // defensive: caller should have assigned
			message_TxRx_byte_buffer[3]=(char)(bsi & 0xFF);
			message_TxRx_byte_buffer[4]=message->id;
		}
		else
		{
			message_TxRx_byte_buffer[3]=message->id;
		}
		header_length=effective_data_long_header_length(sack_v2_enabled);
	}
	else if (message->type==DATA_SHORT)
	{
		// SACK Design A Steps 2+3 — DATA_SHORT header growth + batch_seq_id plumbing.
		// v1 (default): 5 bytes [type, conn_id, seq(EOB bit7), id, length].
		// v2: 6 bytes [type, conn_id, seq(EOB bit7), batch_seq_id, id, length].
		message_TxRx_byte_buffer[0]=message->type;
		message_TxRx_byte_buffer[1]=connection_id;
		message_TxRx_byte_buffer[2]=message->sequence_number;
		if(sack_v2_enabled)
		{
			int bsi = message->batch_seq_id;
			if(bsi < 0) bsi = 0;
			message_TxRx_byte_buffer[3]=(char)(bsi & 0xFF);
			message_TxRx_byte_buffer[4]=message->id;
			message_TxRx_byte_buffer[5]=message->length;
		}
		else
		{
			message_TxRx_byte_buffer[3]=message->id;
			message_TxRx_byte_buffer[4]=message->length;
		}
		header_length=effective_data_short_header_length(sack_v2_enabled);
	}
	else if (message->type==ACK_RANGE || message->type==ACK_MULTI)
	{
		message_TxRx_byte_buffer[0]=message->type;
		message_TxRx_byte_buffer[1]=connection_id;
		message_TxRx_byte_buffer[2]=message->sequence_number;
		header_length=ACK_MULTI_ACK_RANGE_HEADER_LENGTH;
	}
	else if (message->type==SACK_RSP)
	{
		// SACK Design A Step 7 — OFDM SACK_RSP control frame. Same 3-byte
		// header as ACK_RANGE / ACK_MULTI / CONTROL: [type, conn_id, seq_num].
		// The payload (set by the caller in message->data, length tells how
		// many bytes total) is [batch_seq_id, bitmap..., CRC8]. We do NOT
		// touch the payload here — only the header bytes.
		message_TxRx_byte_buffer[0]=message->type;
		message_TxRx_byte_buffer[1]=connection_id;
		message_TxRx_byte_buffer[2]=message->sequence_number;
		header_length=ACK_MULTI_ACK_RANGE_HEADER_LENGTH;
	}
	else if (message->type==CONTROL || message->type==ACK_CONTROL)
	{
		message_TxRx_byte_buffer[0]=message->type;
		message_TxRx_byte_buffer[1]=connection_id;
		message_TxRx_byte_buffer[2]=message->sequence_number;
		header_length=CONTROL_ACK_CONTROL_HEADER_LENGTH;
	}

	for(int i=0;i<message->length;i++)
	{
		message_TxRx_byte_buffer[i+header_length]=message->data[i];
	}

	if(header_length>max_header_length)
	{
		std::cout<<"header size is too big, adjust the configuration parameters"<<std::endl;
		exit(0);
	}

	for(int i=0;i<(header_length+message->length);i++)
	{
		telecom_system->data_container.data_byte[i]=(int)(unsigned char)message_TxRx_byte_buffer[i];
	}

	// Bug #34 diagnostic: print TX frame bytes for CONTROL messages
	if(message->type == CONTROL || message->type == ACK_CONTROL)
	{
		int total = header_length + message->length;
		printf("[TX-CTRL] type=%d hdr=%d len=%d total=%d bytes:",
			message->type, header_length, message->length, total);
		for(int i = 0; i < total && i < 10; i++)
			printf(" %02x", telecom_system->data_container.data_byte[i] & 0xFF);
		printf("\n");
		fflush(stdout);
	}

	telecom_system->transmit_byte(telecom_system->data_container.data_byte,header_length+message->length,telecom_system->data_container.ready_to_transmit_passband_data_tx,message_location);

	{
		int active_nsymb = telecom_system->get_active_nsymb();
		tx_transfer(telecom_system->data_container.ready_to_transmit_passband_data_tx,
					telecom_system->data_container.Nofdm * telecom_system->data_container.interpolation_rate *
					(active_nsymb + telecom_system->data_container.preamble_nSymb));
	}

	while (size_buffer(playback_buffer) > 0)
		msleep(1);

	last_message_sent_type=message->type;
	if(message->type==CONTROL || message->type==ACK_CONTROL)
	{
		last_message_sent_code=message->data[0];
	}
	last_received_message_sequence=-1;

}

void cl_arq_controller::send_batch()
{
	if(passive_monitor) return;  // Never transmit in monitor mode
	// === DIAG: always print TX activity (remove after debug) ===
	printf("[CMD-TX] CONFIG_%d batch=%d type=%d pream=%d Nsymb=%d\n",
		current_configuration, message_batch_counter_tx,
		message_batch_counter_tx > 0 ? messages_batch_tx[0].type : -1,
		telecom_system->data_container.preamble_nSymb,
		telecom_system->data_container.Nsymb);
	fflush(stdout);
	if(g_verbose) {
		printf("[TX] send_batch() on CONFIG_%d, %d messages, first type=%d\n",
			current_configuration, message_batch_counter_tx,
			message_batch_counter_tx > 0 ? messages_batch_tx[0].type : -1);
		fflush(stdout);
	}

	// Flush capture buffer at the START of send_batch(), before TX begins.
	// On VB-Cable (and real radios), the responder decodes the frame and sends
	// its ACK pattern while the commander may still be draining playback or in
	// PTT-off delay. By flushing here, the buffer is clean BEFORE self-echo
	// starts, and the ACK pattern that arrives after the frame is preserved.
	// The order-aware ACK detector distinguishes ACK tones from OFDM self-echo.
	circular_buf_reset(capture_buffer);
	{
		int buf_samples = telecom_system->data_container.Nofdm * telecom_system->data_container.buffer_Nsymb * telecom_system->data_container.interpolation_rate;
		MUTEX_LOCK(&capture_prep_mutex);
		memset(telecom_system->data_container.passband_delayed_data, 0, 2 * buf_samples * sizeof(double));
		telecom_system->data_container.ring_write_index = 0;
		MUTEX_UNLOCK(&capture_prep_mutex);
	}
	telecom_system->data_container.nUnder_processing_events = 0;
	telecom_system->receive_stats.delay_of_last_decoded_message = -1;
	telecom_system->receive_stats.mfsk_search_raw = 0;
	telecom_system->receive_stats.ofdm_search_raw = 0;
	telecom_system->receive_stats.ofdm_batch_active = false;

	// rx_mute during TX: capture_prep_thread writes zeros to ring buffer,
	// so self-echo never enters. After playback drain, we unmute and flush
	// so the ACK arrives into a clean buffer. (Replaces Bug #38 post-TX
	// ring zero which destroyed early-arriving ACK audio.)
	telecom_system->data_container.rx_mute = 1;

	ptt_on();

	cl_timer ptt_on_delay, ptt_off_delay;
	ptt_on_delay.start();

	int active_nsymb = telecom_system->get_active_nsymb();
	int frame_output_size = telecom_system->data_container.Nofdm*telecom_system->data_container.interpolation_rate*(active_nsymb+telecom_system->data_container.preamble_nSymb);

	double *batch_frames_output_data=NULL;
	double *batch_frames_output_data_filtered1=NULL;
	double *batch_frames_output_data_filtered2=NULL;

	int batch_alloc_count = (message_batch_counter_tx+2)*frame_output_size;
	batch_frames_output_data=new double[batch_alloc_count];
	batch_frames_output_data_filtered1=new double[batch_alloc_count];
	batch_frames_output_data_filtered2=new double[batch_alloc_count];

	if (batch_frames_output_data==NULL)
	{
		exit(-31);
	}
	if (batch_frames_output_data_filtered1==NULL)
	{
		exit(-32);
	}
	if (batch_frames_output_data_filtered2==NULL)
	{
		exit(-33);
	}

	int header_length=0;
	for(int i=0;i<message_batch_counter_tx;i++)
	{
		// SACK retransmit: sequence_number already set to original position
		// with end-of-batch flag. Don't override.
		if(!sack_retransmit_active)
		{
			messages_batch_tx[i].sequence_number=i;
			// Mark last DATA frame in batch with bit 7 so responder knows actual batch size.
			// Only for data frames — control frames must not set this flag.
			if(i == message_batch_counter_tx - 1
				&& (messages_batch_tx[i].type == DATA_LONG || messages_batch_tx[i].type == DATA_SHORT))
				messages_batch_tx[i].sequence_number |= 0x80;
		}

		header_length=0;

		if(messages_batch_tx[i].type==DATA_LONG)
		{
			// SACK Design A Steps 1+3 — DATA_LONG header growth + batch_seq_id plumbing.
			// See cl_arq_controller::send() for full format documentation.
			message_TxRx_byte_buffer[0]=messages_batch_tx[i].type;
			message_TxRx_byte_buffer[1]=connection_id;
			message_TxRx_byte_buffer[2]=messages_batch_tx[i].sequence_number;
			if(sack_v2_enabled)
			{
				int bsi = messages_batch_tx[i].batch_seq_id;
				if(bsi < 0) bsi = 0;  // defensive
				message_TxRx_byte_buffer[3]=(char)(bsi & 0xFF);
				message_TxRx_byte_buffer[4]=messages_batch_tx[i].id;
			}
			else
			{
				message_TxRx_byte_buffer[3]=messages_batch_tx[i].id;
			}
			header_length=effective_data_long_header_length(sack_v2_enabled);
		}
		else if (messages_batch_tx[i].type==DATA_SHORT)
		{
			// SACK Design A Steps 2+3 — DATA_SHORT header growth + batch_seq_id plumbing.
			// See cl_arq_controller::send() for full format documentation.
			message_TxRx_byte_buffer[0]=messages_batch_tx[i].type;
			message_TxRx_byte_buffer[1]=connection_id;
			message_TxRx_byte_buffer[2]=messages_batch_tx[i].sequence_number;
			if(sack_v2_enabled)
			{
				int bsi = messages_batch_tx[i].batch_seq_id;
				if(bsi < 0) bsi = 0;
				message_TxRx_byte_buffer[3]=(char)(bsi & 0xFF);
				message_TxRx_byte_buffer[4]=messages_batch_tx[i].id;
				message_TxRx_byte_buffer[5]=messages_batch_tx[i].length;
			}
			else
			{
				message_TxRx_byte_buffer[3]=messages_batch_tx[i].id;
				message_TxRx_byte_buffer[4]=messages_batch_tx[i].length;
			}
			header_length=effective_data_short_header_length(sack_v2_enabled);
		}
		else if (messages_batch_tx[i].type==ACK_RANGE || messages_batch_tx[i].type==ACK_MULTI)
		{
			message_TxRx_byte_buffer[0]=messages_batch_tx[i].type;
			message_TxRx_byte_buffer[1]=connection_id;
			message_TxRx_byte_buffer[2]=messages_batch_tx[i].sequence_number;
			header_length=ACK_MULTI_ACK_RANGE_HEADER_LENGTH;
		}
		else if (messages_batch_tx[i].type==SACK_RSP)
		{
			// SACK Design A Step 7 — OFDM SACK_RSP. 3-byte standard header;
			// payload [batch_seq_id, bitmap..., CRC8] is carried in
			// messages_batch_tx[i].data[..] verbatim. Symmetric with the send()
			// branch above.
			message_TxRx_byte_buffer[0]=messages_batch_tx[i].type;
			message_TxRx_byte_buffer[1]=connection_id;
			message_TxRx_byte_buffer[2]=messages_batch_tx[i].sequence_number;
			header_length=ACK_MULTI_ACK_RANGE_HEADER_LENGTH;
		}
		else if (messages_batch_tx[i].type==CONTROL || messages_batch_tx[i].type==ACK_CONTROL)
		{
			message_TxRx_byte_buffer[0]=messages_batch_tx[i].type;
			message_TxRx_byte_buffer[1]=connection_id;
			message_TxRx_byte_buffer[2]=messages_batch_tx[i].sequence_number;
			header_length=CONTROL_ACK_CONTROL_HEADER_LENGTH;
		}

		for(int j=0;j<messages_batch_tx[i].length;j++)
		{
			message_TxRx_byte_buffer[j+header_length]=messages_batch_tx[i].data[j];
		}

		if(header_length>max_header_length)
		{
			std::cout<<"header size is too big, adjust the configuration parameters"<<std::endl;
			exit(0);
		}

		for(int j=0;j<(header_length+messages_batch_tx[i].length);j++)
		{
			telecom_system->data_container.data_byte[j]=(int)(unsigned char)message_TxRx_byte_buffer[j];
		}

		if(g_verbose) {
			int total = header_length + messages_batch_tx[i].length;
			printf("[TX-BYTES] frame=%d type=%d connid=%d hdr=%d len=%d bytes:",
				i, messages_batch_tx[i].type, (int)(unsigned char)connection_id,
				header_length, messages_batch_tx[i].length);
			for(int j=0; j<total && j<12; j++)
				printf(" %02x", (unsigned char)message_TxRx_byte_buffer[j]);
			printf("\n");
			fflush(stdout);
		}

		telecom_system->transmit_byte(telecom_system->data_container.data_byte,header_length+messages_batch_tx[i].length,&batch_frames_output_data[(i+1)*frame_output_size],NO_FILTER_MESSAGE);


		last_message_sent_type=messages_batch_tx[i].type;
		if(messages_batch_tx[i].type==CONTROL || messages_batch_tx[i].type==ACK_CONTROL)
		{
			last_message_sent_code=messages_batch_tx[i].data[0];
		}
		last_received_message_sequence=-1;

	}

	for(int i=0;i<frame_output_size;i++) //padding start and end to prepare for filtering
	{
		batch_frames_output_data[(0)*frame_output_size+i]=batch_frames_output_data[(0+1)*frame_output_size+i];
		batch_frames_output_data[(message_batch_counter_tx+1)*frame_output_size+i]=batch_frames_output_data[(message_batch_counter_tx)*frame_output_size+i];
	}

	{
		int total_fir_size = (message_batch_counter_tx+2)*frame_output_size;
		memset(batch_frames_output_data_filtered1, 0, total_fir_size * sizeof(double));
		memset(batch_frames_output_data_filtered2, 0, total_fir_size * sizeof(double));
		telecom_system->ofdm.FIR_tx1.apply(batch_frames_output_data,batch_frames_output_data_filtered1,total_fir_size);
		telecom_system->ofdm.FIR_tx2.apply(batch_frames_output_data_filtered1,batch_frames_output_data_filtered2,total_fir_size);

		// DIAG: TX peak amplitude after FIR filtering
		{
			double pk_pre = 0, pk_post = 0;
			for(int j = 0; j < total_fir_size; j++) {
				if(fabs(batch_frames_output_data[j]) > pk_pre) pk_pre = fabs(batch_frames_output_data[j]);
				if(fabs(batch_frames_output_data_filtered2[j]) > pk_post) pk_post = fabs(batch_frames_output_data_filtered2[j]);
			}
			printf("[TX-PEAK] pre_fir=%.4f post_fir=%.4f frames=%d size=%d cfg=%d\n",
				pk_pre, pk_post, message_batch_counter_tx, total_fir_size, current_configuration);
			fflush(stdout);
		}
	}

	// === TX SELF-TEST: verify matched filter template vs actual batch TX output ===
	// The first frame in the batch starts at offset frame_output_size in the filtered data
	// (position 0 is the padding copy). Preamble is at the start of the first frame.
	{
		static int batch_selftest_count = 0;
		static int batch_selftest_last_config = -1;
		if(batch_selftest_last_config != current_configuration) {
			batch_selftest_count = 0;
			batch_selftest_last_config = current_configuration;
		}
		if(batch_selftest_count < 1 && telecom_system->ofdm.ofdm_corr_template != NULL
			&& telecom_system->M != MOD_MFSK)
		{
			batch_selftest_count++;
			int interp = telecom_system->frequency_interpolation_rate;
			int Nofdm_l = telecom_system->data_container.Nofdm;
			int preamble_nsymb = telecom_system->data_container.preamble_nSymb;
			// First frame in batch is at offset frame_output_size (slot 1; slot 0 is padding)
			double* frame_pb = &batch_frames_output_data_filtered2[frame_output_size];
			int frame_len = frame_output_size;

			std::complex<double>* tx_bb = new std::complex<double>[frame_len];
			telecom_system->ofdm.passband_to_baseband(frame_pb, frame_len, tx_bb,
				telecom_system->sampling_frequency, telecom_system->carrier_frequency,
				telecom_system->carrier_amplitude, 1, &telecom_system->ofdm.FIR_rx_time_sync);

			int sym_interp = Nofdm_l * interp;
			printf("[TX-SELFTEST-BATCH] CONFIG_%d frames=%d frame_size=%d\n",
				current_configuration, message_batch_counter_tx, frame_output_size);
			double total_metric = 0;
			for(int k = 0; k < preamble_nsymb && k < telecom_system->ofdm.ofdm_corr_template_nsymb; k++)
			{
				int tmpl_off = k * Nofdm_l;
				int rx_off = k * sym_interp;
				double cr = 0, ci = 0, e_t = 0, e_r = 0;
				for(int n = 0; n < Nofdm_l; n++)
				{
					int rx_idx = rx_off + n * interp;
					if(rx_idx >= frame_len) break;
					std::complex<double> rx = tx_bb[rx_idx];
					double t_re = telecom_system->ofdm.ofdm_corr_template[tmpl_off + n].real();
					double t_im = telecom_system->ofdm.ofdm_corr_template[tmpl_off + n].imag();
					e_t += t_re*t_re + t_im*t_im;
					e_r += rx.real()*rx.real() + rx.imag()*rx.imag();
					cr += t_re*rx.real() + t_im*rx.imag();
					ci += t_im*rx.real() - t_re*rx.imag();
				}
				double cs = (e_t*e_r > 1e-30) ? (cr*cr + ci*ci) / (e_t*e_r) : 0;
				printf("  sym%d: cs=%.4f e_t=%.3f e_r=%.3f |corr|2=%.3f\n",
					k, cs, e_t, e_r, cr*cr+ci*ci);
				total_metric += cs;
			}
			printf("  total=%.4f (expect ~%.1f if template matches TX)\n",
				total_metric, (double)preamble_nsymb);
			printf("  tmpl[0..3]:");
			for(int n = 0; n < 4 && n < Nofdm_l; n++)
				printf(" (%.4f,%.4f)",
					telecom_system->ofdm.ofdm_corr_template[n].real(),
					telecom_system->ofdm.ofdm_corr_template[n].imag());
			printf("\n  tx_bb[0..3]:");
			for(int n = 0; n < 4; n++) {
				int idx = n * interp;
				if(idx < frame_len)
					printf(" (%.4f,%.4f)", tx_bb[idx].real(), tx_bb[idx].imag());
			}
			printf("\n");
			fflush(stdout);

			delete[] tx_bb;
		}
	}

	while(ptt_on_delay.get_elapsed_time_ms() < ptt_on_delay_ms)
		msleep(1);

	// Generate pilot tone if enabled (configurable frequency to warm up TX/amp)
	if(pilot_tone_ms > 0 && pilot_tone_hz > 0)
	{
		const double SAMPLE_RATE = 48000.0;
		const double PILOT_FREQ = (double)pilot_tone_hz;
		const double PI = 3.14159265358979323846;
		int pilot_samples = (int)(pilot_tone_ms * SAMPLE_RATE / 1000.0);
		double* pilot_buffer = new double[pilot_samples];

		for(int i = 0; i < pilot_samples; i++)
		{
			// Generate sine wave with soft ramp up/down to avoid clicks
			double t = (double)i / SAMPLE_RATE;
			double envelope = 1.0;
			int ramp_samples = (int)(SAMPLE_RATE * 0.005); // 5ms ramp
			if(i < ramp_samples)
				envelope = (double)i / ramp_samples;
			else if(i > pilot_samples - ramp_samples)
				envelope = (double)(pilot_samples - i) / ramp_samples;

			pilot_buffer[i] = envelope * 0.5 * sin(2.0 * PI * PILOT_FREQ * t);
		}

		tx_transfer(pilot_buffer, pilot_samples);
		delete[] pilot_buffer;
	}

	for(int i=0;i<message_batch_counter_tx;i++)
	{
		if(g_verbose) { printf("[TX] tx_transfer frame %d/%d, size=%d\n", i, message_batch_counter_tx, frame_output_size); fflush(stdout); }
		tx_transfer(&batch_frames_output_data_filtered2[(i+1)*frame_output_size], frame_output_size);
	}

	if(g_verbose) { printf("[TX] Waiting for playback buffer to drain...\n"); fflush(stdout); }
	// wait buffer to be played
	while (size_buffer(playback_buffer) > 0)
		msleep(1);

	// M1 (SACK turnaround trace): the true "last DATA audio sample left the
	// sound card" instant — the playback buffer just drained, and this is
	// BEFORE the capture ring reset. Anchors the CMD↔RSP overlap window.
	mtl::log_event("cmd_batch_last_sym_out");

	// Unmute + flush right after playback drain (before ptt_off_delay).
	// During TX, rx_mute=1 kept self-echo out of the ring.
	// Now unmute so the RSP's ACK can arrive during ptt_off_delay.
	// Safe: on real radio, still keyed during ptt_off_delay (no RX audio).
	// On VB-Cable, no self-echo (separate in/out cables).
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
	// M2 (SACK turnaround trace): post-TX capture ring reset complete, rx still
	// muted. The M2 -> cmd_post_tx_unmute delta is the CMD-side dead window
	// where freshly-captured RX audio would be clobbered by the ring reset.
	mtl::log_event("cmd_ring_reset_done");
	telecom_system->data_container.rx_mute = 0;
	telecom_system->data_container.rx_mute_samples = 0;
	mtl::log_event("cmd_post_tx_unmute");

	ptt_off_delay.start();
	while(ptt_off_delay.get_elapsed_time_ms() < ptt_off_delay_ms)
		msleep(1);

	ptt_off();
	mtl::log_event("cmd_ptt_off");

	if (batch_frames_output_data!=NULL)
	{
		delete[] batch_frames_output_data;
		batch_frames_output_data=NULL;
	}
	if (batch_frames_output_data_filtered1!=NULL)
	{
		delete[] batch_frames_output_data_filtered1;
		batch_frames_output_data_filtered1=NULL;
	}
	if (batch_frames_output_data_filtered2!=NULL)
	{
		delete[] batch_frames_output_data_filtered2;
		batch_frames_output_data_filtered2=NULL;
	}

	for(int i=0;i<message_batch_counter_tx;i++)
	{
		if(messages_batch_tx[i].type==DATA_LONG || messages_batch_tx[i].type==DATA_SHORT)
		{
			int id = (int)(unsigned char)messages_batch_tx[i].id;
			messages_tx[id].ack_timer.start();
			messages_tx[id].status=PENDING_ACK;
			// Ensure padded (duplicate) frames have valid timeout/resend
			// so they don't instantly expire (ack_timeout=0, nResends=0).
			if(messages_tx[id].ack_timeout == 0)
				messages_tx[id].ack_timeout = ack_timeout_data;
			if(messages_tx[id].nResends == 0)
				messages_tx[id].nResends = nResends;
		}
		if(messages_batch_tx[i].type==CONTROL)
		{
			messages_control.ack_timer.start();
			messages_control.status=PENDING_ACK;
		}

		this->messages_batch_tx[i].ack_timeout=0;
		this->messages_batch_tx[i].id=0;
		this->messages_batch_tx[i].length=0;
		this->messages_batch_tx[i].nResends=0;
		this->messages_batch_tx[i].status=FREE;
		this->messages_batch_tx[i].type=NONE;

	}
	message_batch_counter_tx=0;



	// Commander: after batch TX, set small ftr for quick ACK polling.
	// receive_ack_pattern() checks frames_to_read==0 before polling, so
	// a large ftr would delay the first ACK check by seconds, causing the
	// ACK pattern to scroll past the detection window.  preamble_nSymb (4)
	// gives a ~90ms initial delay, then receive_ack_pattern sets ftr=2.
	telecom_system->data_container.frames_to_read =
		telecom_system->data_container.preamble_nSymb;
	printf("[TX-END] frames_to_read=%d (ctrl=%d)\n", telecom_system->data_container.frames_to_read.load(), telecom_system->mfsk_ctrl_mode ? 1 : 0);
	fflush(stdout);
}

// Transmit short ACK tone pattern instead of LDPC-encoded ACK frame
void cl_arq_controller::send_ack_pattern()
{
	if(passive_monitor) return;
	cl_timer ack_turnaround_timer;
	ack_turnaround_timer.start();
	printf("[TX-ACK-PAT] Sending ACK pattern on CONFIG_%d at t=%dms\n", current_configuration, (int)ack_turnaround_timer.get_elapsed_time_ms()); fflush(stdout);
	mtl::log_event("rsp_ack_send_start");

	// Wait for the full OFDM frame to finish being received before
	// transmitting. With high-redundancy LDPC (e.g. CONFIG_0 rate 1/16),
	// the decoder converges before the frame is fully captured. Transmitting
	// prematurely on a half-duplex link collides with the incoming frame
	// and destroys subsequent audio via rx_mute + buffer flush.
	// Total commander TX: ptt_on_delay + pilot + OFDM frame + ptt_off_delay.
	// We see the preamble in audio (after ptt_on_delay + pilot), so remaining
	// channel time = remaining OFDM symbols + ptt_off_delay + ptt_on_delay.
	if(is_ofdm_config(current_configuration))
	{
		int interp = telecom_system->data_container.interpolation_rate;
		int sym_samples = telecom_system->data_container.Nofdm * interp;
		int frame_sym = telecom_system->data_container.preamble_nSymb
		              + telecom_system->data_container.Nsymb;
		int buf_sym = telecom_system->data_container.buffer_Nsymb.load();
		int delay_sym = (sym_samples > 0)
		              ? telecom_system->receive_stats.delay / sym_samples : 0;
		int frame_end_sym = delay_sym + frame_sym;
		int remaining_sym = frame_end_sym - buf_sym;
		if(remaining_sym < 0) remaining_sym = 0;

		// Remaining OFDM audio + commander's PTT tail + guard margin
		int wait_ms = (remaining_sym * telecom_system->data_container.Nofdm
		              * 1000 + 47999) / 48000;
		wait_ms += ptt_off_delay_ms + ptt_on_delay_ms;

		if(wait_ms > 0)
		{
			printf("[TX-ACK-PAT] Waiting %dms (remaining=%dsym delay=%dsym buf=%dsym frame=%dsym ptt_off=%d ptt_on=%d)\n",
				wait_ms, remaining_sym, delay_sym, buf_sym, frame_sym, ptt_off_delay_ms, ptt_on_delay_ms);
			fflush(stdout);
			msleep(wait_ms);
		}
	}
	else
	{
		// MFSK: frame is fully captured before decode, but the commander's
		// radio still needs PTT-off + TX→RX hardware switching time.
		// Without this, the ACK fires ~10ms after decode — before the
		// commander has switched to RX.
		int wait_ms = ptt_off_delay_ms + ptt_on_delay_ms;
		printf("[TX-ACK-PAT] MFSK guard %dms (ptt_off=%d, ptt_on=%d)\n",
			wait_ms, ptt_off_delay_ms, ptt_on_delay_ms);
		fflush(stdout);
		msleep(wait_ms);
	}

	printf("[TX-ACK-PAT] Guard done at t=%dms\n", (int)ack_turnaround_timer.get_elapsed_time_ms()); fflush(stdout);
	ptt_on();

	cl_timer ptt_on_delay_timer, ptt_off_delay_timer;
	ptt_on_delay_timer.start();

	int pattern_samples = telecom_system->ack_pattern_passband_samples;
	int symbol_period = telecom_system->data_container.Nofdm * telecom_system->data_container.interpolation_rate;

	// Allocate buffers: pattern + 1 symbol padding at each end for FIR filtering
	int padded_size = pattern_samples + 2 * symbol_period;
	double *raw_output = new double[padded_size];
	double *filtered1 = new double[padded_size];
	double *filtered2 = new double[padded_size];

	if(!raw_output || !filtered1 || !filtered2) exit(-34);

	memset(raw_output, 0, padded_size * sizeof(double));

	// Generate ACK pattern passband into the middle section
	telecom_system->generate_ack_pattern_passband(&raw_output[symbol_period]);

	// Pad start and end with copies of first/last symbol for FIR boundary
	memcpy(&raw_output[0], &raw_output[symbol_period], symbol_period * sizeof(double));
	memcpy(&raw_output[symbol_period + pattern_samples], &raw_output[pattern_samples], symbol_period * sizeof(double));

	// FIR filter chain (same as send_batch)
	memset(filtered1, 0, padded_size * sizeof(double));
	memset(filtered2, 0, padded_size * sizeof(double));
	telecom_system->ofdm.FIR_tx1.apply(raw_output, filtered1, padded_size);
	telecom_system->ofdm.FIR_tx2.apply(filtered1, filtered2, padded_size);

	// Wait PTT on delay
	while(ptt_on_delay_timer.get_elapsed_time_ms() < ptt_on_delay_ms)
		msleep(1);

	// Pilot tone (if enabled)
	if(pilot_tone_ms > 0 && pilot_tone_hz > 0)
	{
		const double SAMPLE_RATE = 48000.0;
		const double PILOT_FREQ = (double)pilot_tone_hz;
		const double PI = 3.14159265358979323846;
		int pilot_samples = (int)(pilot_tone_ms * SAMPLE_RATE / 1000.0);
		double* pilot_buffer = new double[pilot_samples];

		for(int i = 0; i < pilot_samples; i++)
		{
			double t = (double)i / SAMPLE_RATE;
			double envelope = 1.0;
			int ramp_samples = (int)(SAMPLE_RATE * 0.005);
			if(i < ramp_samples)
				envelope = (double)i / ramp_samples;
			else if(i > pilot_samples - ramp_samples)
				envelope = (double)(pilot_samples - i) / ramp_samples;
			pilot_buffer[i] = envelope * 0.5 * sin(2.0 * PI * PILOT_FREQ * t);
		}

		tx_transfer(pilot_buffer, pilot_samples);
		delete[] pilot_buffer;
	}

	// Transmit the filtered ACK pattern (skip padding at start)
	printf("[TX-ACK-PAT] Audio start at t=%dms (%d samples)\n", (int)ack_turnaround_timer.get_elapsed_time_ms(), pattern_samples); fflush(stdout);
	tx_transfer(&filtered2[symbol_period], pattern_samples);

	// Wait for playback to drain
	while(size_buffer(playback_buffer) > 0)
		msleep(1);

	printf("[TX-ACK-PAT] Audio done at t=%dms\n", (int)ack_turnaround_timer.get_elapsed_time_ms()); fflush(stdout);
	mtl::log_event("rsp_ack_audio_done");

	delete[] raw_output;
	delete[] filtered1;
	delete[] filtered2;

	// Flush capture buffer immediately after drain — BEFORE ptt_off_delay.
	// On VB-Cable (zero propagation delay), the commander detects the ACK
	// pattern mid-TX and starts its guard timer. If we wait for ptt_off_delay
	// (200ms) before flushing, the commander's data can arrive while the
	// responder is still muted, destroying seq=00's preamble.
	// Flushing now lets the capture thread receive clean audio during
	// ptt_off_delay, giving 200ms+ margin instead of potentially negative.
	telecom_system->data_container.rx_mute = 1;
	msleep(RX_MUTE_GUARD_MS);
	circular_buf_reset(capture_buffer);
	{
		int buf_samples = telecom_system->data_container.Nofdm * telecom_system->data_container.buffer_Nsymb * telecom_system->data_container.interpolation_rate;
		MUTEX_LOCK(&capture_prep_mutex);
		memset(telecom_system->data_container.passband_delayed_data, 0, 2 * buf_samples * sizeof(double));
		telecom_system->data_container.ring_write_index = 0;
		MUTEX_UNLOCK(&capture_prep_mutex);
	}
	telecom_system->data_container.rx_mute = 0;
	telecom_system->data_container.rx_mute_samples = 0;
	// Bug #41: Reset nUnder after flush. The flush destroyed all pre-flush
	// audio, so nUnder accumulated during ACK TX is stale — those captured
	// symbols no longer exist in the buffer. Without this reset, same-modulation
	// transitions (e.g. CONFIG_0→CONFIG_1) keep stale nUnder (~28 symbols),
	// the ftr calculation subtracts it, shrinking the capture window so the
	// commander's next frame arrives past upper_bound.
	telecom_system->data_container.nUnder_processing_events = 0;
	telecom_system->receive_stats.delay_of_last_decoded_message = -1;
	telecom_system->receive_stats.mfsk_search_raw = 0;
	telecom_system->receive_stats.ofdm_search_raw = 0;
	telecom_system->receive_stats.ofdm_batch_active = false;
	// ftr = exactly one frame. Turnaround waiting is handled by ptt_off_delay
	// below (200ms) plus the capture thread filling the buffer during that time.
	// ftr = one frame + 10 symbol margin. The margin covers PTT turnaround
	// settle time (~240ms) so the receiver doesn't start decoding noise
	// before the next TX batch arrives. Without it, early OFDM-FAILs cascade.
	{
		int rx_frame = telecom_system->data_container.preamble_nSymb
		             + telecom_system->data_container.Nsymb;
		telecom_system->data_container.frames_to_read = rx_frame + 10;
	}

	printf("[TX-ACK-PAT] Done at t=%dms, flushed capture buffer, nUnder reset, ftr=%d\n", (int)ack_turnaround_timer.get_elapsed_time_ms(), telecom_system->data_container.frames_to_read.load());
	mtl::log_event_kv("rsp_post_ack_flush_done", "ftr=%d", telecom_system->data_container.frames_to_read.load());
	fflush(stdout);

	// PTT off delay + release after flush. The capture thread is active
	// during this delay, receiving silence (VB-Cable) or post-TX settling
	// noise (real radio — rejected by preamble energy gate / metric threshold).
	ptt_off_delay_timer.start();
	while(ptt_off_delay_timer.get_elapsed_time_ms() < ptt_off_delay_ms)
		msleep(1);

	ptt_off();
}

// Transmit ACK + SNR suffix pattern (turboshift only)
void cl_arq_controller::send_ack_pattern_with_snr(float snr)
{
	if(passive_monitor) return;
	printf("[TX-ACK-SNR] Sending ACK+SNR pattern (SNR=%.1f dB, tone=%d) on CONFIG_%d\n",
		snr, telecom_system->ack_mfsk.snr_to_tone(snr), current_configuration);
	fflush(stdout);

	// Guard delay for MFSK modes (same as send_ack_pattern)
	if(is_robust_config(current_configuration))
	{
		int wait_ms = ptt_off_delay_ms + ptt_on_delay_ms;
		msleep(wait_ms);
	}

	ptt_on();

	cl_timer ptt_on_delay_timer, ptt_off_delay_timer;
	ptt_on_delay_timer.start();

	int pattern_samples = telecom_system->ack_snr_pattern_passband_samples;
	int symbol_period = telecom_system->data_container.Nofdm * telecom_system->data_container.interpolation_rate;

	int padded_size = pattern_samples + 2 * symbol_period;
	double *raw_output = new double[padded_size];
	double *filtered1 = new double[padded_size];
	double *filtered2 = new double[padded_size];

	if(!raw_output || !filtered1 || !filtered2) exit(-36);

	memset(raw_output, 0, padded_size * sizeof(double));

	telecom_system->generate_ack_snr_pattern_passband(&raw_output[symbol_period], snr);

	memcpy(&raw_output[0], &raw_output[symbol_period], symbol_period * sizeof(double));
	memcpy(&raw_output[symbol_period + pattern_samples], &raw_output[pattern_samples], symbol_period * sizeof(double));

	memset(filtered1, 0, padded_size * sizeof(double));
	memset(filtered2, 0, padded_size * sizeof(double));
	telecom_system->ofdm.FIR_tx1.apply(raw_output, filtered1, padded_size);
	telecom_system->ofdm.FIR_tx2.apply(filtered1, filtered2, padded_size);

	while(ptt_on_delay_timer.get_elapsed_time_ms() < ptt_on_delay_ms)
		msleep(1);

	if(pilot_tone_ms > 0 && pilot_tone_hz > 0)
	{
		const double SAMPLE_RATE = 48000.0;
		const double PILOT_FREQ = (double)pilot_tone_hz;
		const double PI = 3.14159265358979323846;
		int pilot_samples = (int)(pilot_tone_ms * SAMPLE_RATE / 1000.0);
		double* pilot_buffer = new double[pilot_samples];
		for(int i = 0; i < pilot_samples; i++)
		{
			double t = (double)i / SAMPLE_RATE;
			double envelope = 1.0;
			int ramp_samples = (int)(SAMPLE_RATE * 0.005);
			if(i < ramp_samples)
				envelope = (double)i / ramp_samples;
			else if(i > pilot_samples - ramp_samples)
				envelope = (double)(pilot_samples - i) / ramp_samples;
			pilot_buffer[i] = envelope * 0.5 * sin(2.0 * PI * PILOT_FREQ * t);
		}
		tx_transfer(pilot_buffer, pilot_samples);
		delete[] pilot_buffer;
	}

	tx_transfer(&filtered2[symbol_period], pattern_samples);

	while(size_buffer(playback_buffer) > 0)
		msleep(1);

	delete[] raw_output;
	delete[] filtered1;
	delete[] filtered2;

	// Same flush sequence as send_ack_pattern
	telecom_system->data_container.rx_mute = 1;
	msleep(RX_MUTE_GUARD_MS);
	circular_buf_reset(capture_buffer);
	{
		int buf_samples = telecom_system->data_container.Nofdm * telecom_system->data_container.buffer_Nsymb * telecom_system->data_container.interpolation_rate;
		MUTEX_LOCK(&capture_prep_mutex);
		memset(telecom_system->data_container.passband_delayed_data, 0, 2 * buf_samples * sizeof(double));
		telecom_system->data_container.ring_write_index = 0;
		MUTEX_UNLOCK(&capture_prep_mutex);
	}
	telecom_system->data_container.rx_mute = 0;
	telecom_system->data_container.rx_mute_samples = 0;
	telecom_system->data_container.nUnder_processing_events = 0;
	telecom_system->receive_stats.delay_of_last_decoded_message = -1;
	telecom_system->receive_stats.mfsk_search_raw = 0;
	telecom_system->receive_stats.ofdm_search_raw = 0;
	telecom_system->receive_stats.ofdm_batch_active = false;
	{
		int rx_frame = telecom_system->data_container.preamble_nSymb
		             + telecom_system->data_container.Nsymb;
		telecom_system->data_container.frames_to_read = rx_frame + 10;
	}

	printf("[TX-ACK-SNR] Done, flushed capture buffer, ftr=%d\n", telecom_system->data_container.frames_to_read.load());
	fflush(stdout);

	ptt_off_delay_timer.start();
	while(ptt_off_delay_timer.get_elapsed_time_ms() < ptt_off_delay_ms)
		msleep(1);

	ptt_off();
}

// Transmit SACK pattern with bitmap suffix (partial batch acknowledgement)
void cl_arq_controller::send_sack_pattern(const bool* received_bitmap, int nframes)
{
	if(passive_monitor) return;

	// Log which frames were received
	printf("[TX-SACK] Sending SACK pattern (batch=%d, received:", nframes);
	for(int i = 0; i < nframes; i++)
		printf(" %d", received_bitmap[i] ? 1 : 0);
	printf(") on CONFIG_%d\n", current_configuration);
	fflush(stdout);

	// Guard delay: same logic as send_ack_pattern
	cl_timer sack_turnaround_timer;
	sack_turnaround_timer.start();

	// M5 (SACK turnaround trace): RSP committed to SACK TX, guard about to run.
	// send_sack_pattern() was previously entirely uninstrumented — this is the
	// single most important TX in the SACK collision bug. Mirror of
	// rsp_ack_send_start. M6-M5 = the guard duration actually applied.
	mtl::log_event("rsp_sack_send_start");

	if(is_ofdm_config(current_configuration))
	{
		int interp = telecom_system->data_container.interpolation_rate;
		int sym_samples = telecom_system->data_container.Nofdm * interp;
		int frame_sym = telecom_system->data_container.preamble_nSymb
		              + telecom_system->data_container.Nsymb;
		int buf_sym = telecom_system->data_container.buffer_Nsymb.load();
		int delay_sym = (sym_samples > 0)
		              ? telecom_system->receive_stats.delay / sym_samples : 0;
		int frame_end_sym = delay_sym + frame_sym;
		int remaining_sym = frame_end_sym - buf_sym;
		if(remaining_sym < 0) remaining_sym = 0;

		int wait_ms = (remaining_sym * telecom_system->data_container.Nofdm
		              * 1000 + 47999) / 48000;
		wait_ms += ptt_off_delay_ms + ptt_on_delay_ms;

		// Extra guard for CMD's post-TX drain: CMD does playback drain
		// (~200ms) + ring buffer reset + ptt_off_delay (~200ms) after its
		// last frame. RSP may enter ACK-GATE while CMD is still draining.
		// Without this, SACK audio arrives while CMD is still muted.
		int drain_guard_ms = 500;
		wait_ms += drain_guard_ms;

		if(wait_ms > 0)
		{
			printf("[TX-SACK] OFDM guard: remaining_sym=%d delay_sym=%d frame_sym=%d buf_sym=%d wait=%dms (drain_guard=%d)\n",
				remaining_sym, delay_sym, frame_sym, buf_sym, wait_ms, drain_guard_ms);
			fflush(stdout);
			msleep(wait_ms);
		}
	}
	else
	{
		int wait_ms = ptt_off_delay_ms + ptt_on_delay_ms;
		printf("[TX-SACK] MFSK guard: wait=%dms\n", wait_ms);
		fflush(stdout);
		msleep(wait_ms);
	}

	ptt_on();

	// M6 (SACK turnaround trace): RSP SACK PTT-on instant — the root-cause
	// doc's quantity-of-interest "RSP SACK-ptt-on vs CMD last-DATA-symbol-out".
	mtl::log_event("rsp_sack_ptt_on");

	cl_timer ptt_on_delay_timer, ptt_off_delay_timer;
	ptt_on_delay_timer.start();

	int pattern_samples = telecom_system->sack_pattern_passband_samples(nframes);
	int symbol_period = telecom_system->data_container.Nofdm * telecom_system->data_container.interpolation_rate;

	int padded_size = pattern_samples + 2 * symbol_period;
	double *raw_output = new double[padded_size];
	double *filtered1 = new double[padded_size];
	double *filtered2 = new double[padded_size];

	if(!raw_output || !filtered1 || !filtered2) exit(-37);

	memset(raw_output, 0, padded_size * sizeof(double));

	telecom_system->generate_sack_bitmap_pattern_passband(&raw_output[symbol_period], received_bitmap, nframes);

	memcpy(&raw_output[0], &raw_output[symbol_period], symbol_period * sizeof(double));
	memcpy(&raw_output[symbol_period + pattern_samples], &raw_output[pattern_samples], symbol_period * sizeof(double));

	memset(filtered1, 0, padded_size * sizeof(double));
	memset(filtered2, 0, padded_size * sizeof(double));
	telecom_system->ofdm.FIR_tx1.apply(raw_output, filtered1, padded_size);
	telecom_system->ofdm.FIR_tx2.apply(filtered1, filtered2, padded_size);

	while(ptt_on_delay_timer.get_elapsed_time_ms() < ptt_on_delay_ms)
		msleep(1);

	if(pilot_tone_ms > 0 && pilot_tone_hz > 0)
	{
		const double SAMPLE_RATE = 48000.0;
		const double PILOT_FREQ = (double)pilot_tone_hz;
		const double PI = 3.14159265358979323846;
		int pilot_samples = (int)(pilot_tone_ms * SAMPLE_RATE / 1000.0);
		double* pilot_buffer = new double[pilot_samples];
		for(int i = 0; i < pilot_samples; i++)
		{
			double t = (double)i / SAMPLE_RATE;
			double envelope = 1.0;
			int ramp_samples = (int)(SAMPLE_RATE * 0.005);
			if(i < ramp_samples)
				envelope = (double)i / ramp_samples;
			else if(i > pilot_samples - ramp_samples)
				envelope = (double)(pilot_samples - i) / ramp_samples;
			pilot_buffer[i] = envelope * 0.5 * sin(2.0 * PI * PILOT_FREQ * t);
		}
		tx_transfer(pilot_buffer, pilot_samples);
		delete[] pilot_buffer;
	}

	printf("[TX-SACK] Audio start at t=%dms (%d samples = %dms)\n",
		(int)sack_turnaround_timer.get_elapsed_time_ms(),
		pattern_samples, (int)(pattern_samples * 1000.0 / 48000.0));
	fflush(stdout);
	// M7 (SACK turnaround trace): first SACK sample to the sound card — the
	// SACK audio (not PTT) is what must land in CMD's listening window.
	mtl::log_event_kv("rsp_sack_audio_start", "samples=%d", pattern_samples);
	tx_transfer(&filtered2[symbol_period], pattern_samples);

	while(size_buffer(playback_buffer) > 0)
		msleep(1);
	printf("[TX-SACK] Audio done at t=%dms\n",
		(int)sack_turnaround_timer.get_elapsed_time_ms());
	fflush(stdout);
	// M8 (SACK turnaround trace): SACK audio end. With M7 this defines the
	// full SACK occupancy window [M7,M8] compared against CMD's rx-usable
	// window. Mirror of rsp_ack_audio_done.
	mtl::log_event("rsp_sack_audio_done");

	delete[] raw_output;
	delete[] filtered1;
	delete[] filtered2;

	// Same flush sequence as send_ack_pattern
	telecom_system->data_container.rx_mute = 1;
	msleep(RX_MUTE_GUARD_MS);
	circular_buf_reset(capture_buffer);
	{
		int buf_samples = telecom_system->data_container.Nofdm * telecom_system->data_container.buffer_Nsymb * telecom_system->data_container.interpolation_rate;
		MUTEX_LOCK(&capture_prep_mutex);
		memset(telecom_system->data_container.passband_delayed_data, 0, 2 * buf_samples * sizeof(double));
		telecom_system->data_container.ring_write_index = 0;
		MUTEX_UNLOCK(&capture_prep_mutex);
	}
	telecom_system->data_container.rx_mute = 0;
	telecom_system->data_container.rx_mute_samples = 0;
	telecom_system->data_container.nUnder_processing_events = 0;
	telecom_system->receive_stats.delay_of_last_decoded_message = -1;
	telecom_system->receive_stats.mfsk_search_raw = 0;
	telecom_system->receive_stats.ofdm_search_raw = 0;
	telecom_system->receive_stats.ofdm_batch_active = false;
	{
		int rx_frame = telecom_system->data_container.preamble_nSymb
		             + telecom_system->data_container.Nsymb;
		telecom_system->data_container.frames_to_read = rx_frame + 10;
	}

	printf("[TX-SACK] Done in %dms, flushed capture buffer, ftr=%d\n",
		(int)sack_turnaround_timer.get_elapsed_time_ms(),
		telecom_system->data_container.frames_to_read.load());
	fflush(stdout);
	// M9 (SACK turnaround trace): RSP back to listening after the SACK TX.
	// Mirror of rsp_post_ack_flush_done — bounds the RSP-side post-SACK
	// window symmetrically with the ACK path.
	mtl::log_event_kv("rsp_post_sack_flush_done", "ftr=%d",
		telecom_system->data_container.frames_to_read.load());

	ptt_off_delay_timer.start();
	while(ptt_off_delay_timer.get_elapsed_time_ms() < ptt_off_delay_ms)
		msleep(1);

	ptt_off();
}

// Receive SACK pattern and decode bitmap from suffix
bool cl_arq_controller::receive_sack_pattern(bool* out_bitmap, int nframes)
{
	if(telecom_system->ack_pattern_passband_samples <= 0) return false;

	// Tight snapshot: capture only enough ring buffer to contain the SACK
	// pattern plus search margin. Same strategy as receive_ack_pattern().
	// A full-buffer snapshot (139 symbols) causes the coarse search to
	// find false matches in silence regions, beating the real signal.
	int base_nsymb = telecom_system->ack_mfsk.ack_pattern_nsymb;
	int bitmap_nsuffix = telecom_system->ack_mfsk.sack_bitmap_nsuffix(nframes, &telecom_system->sack_ldpc);
	int sack_total_nsymb = base_nsymb + bitmap_nsuffix;
	int tail_nsymb = sack_total_nsymb + sack_total_nsymb + 16;
	if(tail_nsymb > telecom_system->data_container.buffer_Nsymb)
		tail_nsymb = telecom_system->data_container.buffer_Nsymb;

	int sym_samples = telecom_system->data_container.Nofdm
	                * telecom_system->data_container.interpolation_rate;
	int signal_period = sym_samples * telecom_system->data_container.buffer_Nsymb;
	int tail_samples = tail_nsymb * sym_samples;
	if(tail_samples > signal_period)
		tail_samples = signal_period;
	int tail_offset = signal_period - tail_samples;

	MUTEX_LOCK(&capture_prep_mutex);

	int ftr_now = telecom_system->data_container.frames_to_read;
	if(ftr_now == 0)
	{
		// Snapshot the tail of the ring buffer (same approach as receive_ack_pattern)
		int rwi = telecom_system->data_container.ring_write_index;
		memcpy(telecom_system->data_container.ready_to_process_passband_delayed_data,
			&telecom_system->data_container.passband_delayed_data[rwi + tail_offset],
			tail_samples * sizeof(double));

		telecom_system->data_container.data_ready = 0;
		MUTEX_UNLOCK(&capture_prep_mutex);

		int suffix_tones[32]; // MAX_SACK_BITMAP_SYMBOLS (32 for LDPC, was 14)
		for(int i = 0; i < 32; i++) suffix_tones[i] = -1;

		int matched = 0;
		double metric = telecom_system->detect_sack_pattern_from_passband(
			telecom_system->data_container.ready_to_process_passband_delayed_data,
			tail_samples, &matched, nframes, suffix_tones);

		// Scan passband buffer for energy: divide into 8 segments
		double seg_e[8] = {};
		int seg_size = tail_samples / 8;
		double max_e = 0; int max_seg = -1;
		for(int seg = 0; seg < 8; seg++)
		{
			for(int i = seg * seg_size; i < (seg + 1) * seg_size; i++)
				seg_e[seg] += telecom_system->data_container.ready_to_process_passband_delayed_data[i] *
				              telecom_system->data_container.ready_to_process_passband_delayed_data[i];
			seg_e[seg] /= seg_size;
			if(seg_e[seg] > max_e) { max_e = seg_e[seg]; max_seg = seg; }
		}
		int rwi_snap = telecom_system->data_container.ring_write_index;

		// Diagnostic printf removed from hot polling loop — causes 175-875ms
		// cumulative latency on Windows console (same issue as ACK polling,
		// see comment at line ~4124).
		// Plan A1: lightweight tracker — record peak matched/metric across
		// the polling window so we can see what the detector saw even when
		// it never crosses threshold. No printf in hot loop; report on exit.
		if(matched > sack_diag_peak_matched)
		{
			sack_diag_peak_matched = matched;
			sack_diag_peak_metric = metric;
			sack_diag_peak_max_e = max_e;
			sack_diag_peak_max_seg = max_seg;
		}
		sack_diag_poll_count++;

		if(matched >= telecom_system->ack_mfsk.sack_match_threshold && metric >= 0.5)
		{
			// Cross-check: also run ACK detection on the same snapshot.
			int ack_matched = 0;
			telecom_system->detect_ack_pattern_from_passband(
				telecom_system->data_container.ready_to_process_passband_delayed_data,
				tail_samples, &ack_matched);
			if(ack_matched > matched + 4)
			{
				printf("[RX-SACK] Rejected: ACK cross-check matched=%d >> sack=%d (this is an ACK)\n",
					ack_matched, matched);
				fflush(stdout);
				// Throttle polling (same as receive_ack_pattern no-detect path)
				telecom_system->data_container.frames_to_read = 2;
				telecom_system->data_container.nUnder_processing_events = 0;
				return false;
			}

			// Try LDPC soft-decision decode first (WB M>=16 with sack_ldpc initialized)
			bool ldpc_ok = false;
			if (telecom_system->ack_mfsk.M >= 16 && telecom_system->sack_ldpc.N > 0)
			{
				ldpc_ok = telecom_system->decode_sack_bitmap_ldpc(
					telecom_system->data_container.ready_to_process_passband_delayed_data,
					tail_samples, nframes, out_bitmap);
			}

			// Fault-injection (CLI --test-sack-ldpc-fail): force the ldpc=NO
			// code path even on a healthy SACK reception, so the hard-fallback
			// behaviour can be observed deterministically in a loopback repro.
			// See fact-documents/SACK_LDPC_FALLBACK_INVESTIGATION.md §7 Step 1.
			if (force_sack_ldpc_fail && ldpc_ok)
			{
				printf("[RX-SACK-TESTFAIL] --test-sack-ldpc-fail: forcing ldpc_ok=false "
				       "(LDPC actually decoded OK)\n");
				fflush(stdout);
				ldpc_ok = false;
			}

			// Whether the LDPC wire format is the active SACK encoding for this
			// link. Mirrors the predicate that gated decode_sack_bitmap_ldpc()
			// above. For this format the TX puts an LDPC *codeword* in the
			// suffix tones (mfsk.cc:655-680); the legacy hard decoder
			// decode_sack_bitmap() is NOT its inverse.
			bool ldpc_wire_format =
				(telecom_system->ack_mfsk.M >= 16 && telecom_system->sack_ldpc.N > 0);

			if (!ldpc_ok)
			{
				if (ldpc_wire_format)
				{
					// Candidate A2 (SACK_LDPC_FALLBACK_INVESTIGATION.md §6/§7):
					// the LDPC soft-decision decoder is the ONLY correct decoder
					// for this wire format. On ldpc=NO the bitmap is
					// unrecoverable — the old hard fallback
					// (decode_sack_bitmap()) decoded the LDPC-coded tones with
					// the legacy un-coded codebook and fabricated a bitmap
					// structurally unrelated to what RSP sent, causing
					// wrong-frame retransmits AND a spurious data_ack_received.
					// Correct fail-safe: leave out_bitmap all-false (every frame
					// NACKed) -> CMD does a clean full-batch retransmit, and the
					// genuine "RSP responded / batch finished" signal is still
					// delivered via the return true below. This matches
					// decode_sack_bitmap_ldpc()'s documented "NACK everything on
					// failure" intent (telecom_system.cc:3268-3270).
					for(int i = 0; i < nframes; i++)
						out_bitmap[i] = false;

					printf("[RX-SACK] ldpc=NO -> full-batch retransmit "
					       "(hard fallback skipped for LDPC wire format)\n");
					fflush(stdout);
				}
				else
				{
					// Legacy NB / non-LDPC wire format (M<16 or sack_ldpc not
					// initialised): the TX *does* use the legacy un-coded
					// encode_sack_bitmap() format (mfsk.cc:683-712), so
					// decode_sack_bitmap() IS its correct inverse here. Keep it.
					int nsuffix = telecom_system->ack_mfsk.sack_bitmap_nsuffix(nframes, &telecom_system->sack_ldpc);

					printf("[RX-SACK-RAW] LDPC failed, hard fallback nsuffix=%d suffix_tones:", nsuffix);
					for(int i = 0; i < nsuffix; i++)
						printf(" %d", suffix_tones[i]);
					printf("\n"); fflush(stdout);

					telecom_system->ack_mfsk.decode_sack_bitmap(suffix_tones, nsuffix, nframes, out_bitmap);
				}
			}

			printf("[RX-SACK] Detected (matched=%d, metric=%.1f, ack_xcheck=%d, ldpc=%s), bitmap:",
				matched, metric, ack_matched, ldpc_ok ? "YES" : "NO");
			for(int i = 0; i < nframes; i++)
				printf(" %d", out_bitmap[i] ? 1 : 0);
			printf("\n");
			fflush(stdout);
			return true;
		}

		// Not detected — do NOT set frames_to_read here. The caller may still
		// need to run receive_ack_pattern() on the same poll cycle. That function
		// checks frames_to_read==0 and would skip if we throttled here.
		// Throttling is handled by receive_ack_pattern's failure path instead.
		return false;
	}

	MUTEX_UNLOCK(&capture_prep_mutex);
	return false;
}

// ============================================================================
// SACK Design A Step 7 — OFDM SACK_RSP control frame TX/RX helpers
// ============================================================================
// Replaces the ~1168 ms MFSK SACK pattern with a single OFDM LDPC control
// frame (~390 ms at WB_CFG10). Wire layout AFTER the standard 3-byte msg
// header [type=0x42, conn_id, seq_num=0]:
//
//   payload = [batch_seq_id : u8][bitmap : ceil(N/8) bytes][CRC8 : u8]
//
// where N = data_batch_size at TX time (both peers know N because
// data_batch_size is negotiated at TEST_CONNECTION and never moves within a
// Design A Step 7 session). CRC8 covers (batch_seq_id || bitmap_bytes); it
// does NOT cover the standard msg header (whose integrity is already
// guaranteed by the OFDM LDPC codeword's CRC16). Polynomial: POLY_CRC8
// (=0xF4 in datalink_defines.h), matching the existing CRC8_calc() helper.
//
// v1 MFSK SACK path (send_sack_pattern / receive_sack_pattern) is NOT
// modified by Step 7. Both code paths coexist; the v1 path is exercised when
// (sack_enabled && !sack_v2_enabled), the v2 path when sack_v2_enabled.

long long cl_arq_controller::send_sack_v2_frame(const bool* bitmap, int nframes,
                                                unsigned char batch_seq_id)
{
	if(passive_monitor) return 0;
	if(nframes <= 0 || nframes > MAX_SACK_BATCH_SIZE)
	{
		printf("[TX-SACK-V2] ERROR: nframes=%d out of range [1..%d]\n",
			nframes, MAX_SACK_BATCH_SIZE);
		fflush(stdout);
		return 0;
	}

	int bitmap_bytes = (nframes + 7) / 8;
	int payload_len  = 1 /*batch_seq_id*/ + bitmap_bytes + 1 /*CRC8*/;

	// Build the payload directly into a scratch buffer first so we can
	// CRC-cover the (batch_seq_id || bitmap) prefix before writing CRC8.
	unsigned char payload[1 + (MAX_SACK_BATCH_SIZE + 7) / 8 + 1];
	payload[0] = batch_seq_id;
	for(int b = 0; b < bitmap_bytes; b++) payload[1 + b] = 0;
	for(int i = 0; i < nframes; i++)
	{
		if(bitmap[i])
			payload[1 + (i / 8)] |= (unsigned char)(1u << (i % 8));
	}
	unsigned char crc = CRC8_calc((char*)payload, 1 + bitmap_bytes);

	// Optional CRC8 fault injection (CLI --test-rsp-sack-rsp-crc-corrupt).
	// One-shot: clears the armed flag after firing exactly once.
	if(test_rsp_sack_rsp_crc_corrupt_armed)
	{
		unsigned char corrupted = (unsigned char)(crc ^ 0xFFu);
		printf("[TX-SACK-V2-CRC-CORRUPT] frame: CRC8 0x%02x -> 0x%02x (synthetic fault injection)\n",
			(unsigned)crc, (unsigned)corrupted);
		fflush(stdout);
		crc = corrupted;
		test_rsp_sack_rsp_crc_corrupt_armed = false;
	}
	payload[1 + bitmap_bytes] = crc;

	// Log the ground-truth TX bitmap byte-for-byte so the loopback test can
	// assert byte-identity with the CMD-side decoded bitmap.
	{
		char hex[2 * sizeof(payload) + 1];
		for(int b = 0; b < payload_len; b++)
			snprintf(&hex[2*b], 3, "%02x", payload[b]);
		hex[2*payload_len] = '\0';
		printf("[TX-SACK-V2] batch_seq_id=%u nframes=%d bitmap_bytes=%d payload=%s crc8=0x%02x\n",
			(unsigned)batch_seq_id, nframes, bitmap_bytes, hex, (unsigned)crc);
		fflush(stdout);
	}

	// Stage as a single-frame OFDM batch via messages_batch_tx[0]. Use the
	// existing send_batch() path so the OFDM TX is bit-identical to any other
	// control-class frame's wire shape.
	message_batch_counter_tx = 0;
	messages_batch_tx[0].type = SACK_RSP;
	messages_batch_tx[0].sequence_number = 0;
	messages_batch_tx[0].id = 0;
	messages_batch_tx[0].length = payload_len;
	for(int b = 0; b < payload_len; b++)
		messages_batch_tx[0].data[b] = (char)payload[b];
	messages_batch_tx[0].status = ADDED_TO_BATCH_BUFFER;
	messages_batch_tx[0].batch_seq_id = batch_seq_id;  // diagnostic mirror
	message_batch_counter_tx = 1;

	// Full-length OFDM frame on the data configuration (NOT the MFSK ack
	// config — SACK_RSP carries real LDPC-coded payload).
	telecom_system->set_mfsk_ctrl_mode(false);
	pad_messages_batch_tx(control_batch_size);

	auto t_start = std::chrono::steady_clock::now();
	send_batch();
	auto t_end = std::chrono::steady_clock::now();
	long long elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
		t_end - t_start).count();

	rsp_sack_v2_tx_count++;
	printf("[TX-SACK-V2] send_batch() wire_ms=%lld ctrl_tx_time_ms=%d (legacy MFSK SACK ~1168 ms baseline)\n",
		elapsed_ms, ctrl_transmission_time_ms);
	fflush(stdout);
	return elapsed_ms;
}

bool cl_arq_controller::decode_sack_v2_frame(bool* out_bitmap, int nframes,
                                             unsigned char* out_batch_seq_id)
{
	// Caller has verified messages_rx_buffer.type == SACK_RSP and
	// messages_rx_buffer.status == RECEIVED. Payload bytes live in
	// messages_rx_buffer.data[0..]. Expected layout:
	//   [batch_seq_id : u8][bitmap : ceil(N/8) bytes][CRC8 : u8]
	if(nframes <= 0 || nframes > MAX_SACK_BATCH_SIZE)
	{
		printf("[CMD-SACK-V2-DECODE] ERROR: nframes=%d out of range\n", nframes);
		fflush(stdout);
		return false;
	}
	int bitmap_bytes = (nframes + 7) / 8;
	int payload_len  = 1 + bitmap_bytes + 1;

	unsigned char payload[1 + (MAX_SACK_BATCH_SIZE + 7) / 8 + 1];
	for(int b = 0; b < payload_len; b++)
		payload[b] = (unsigned char)messages_rx_buffer.data[b];

	unsigned char rx_crc       = payload[1 + bitmap_bytes];
	unsigned char computed_crc = CRC8_calc((char*)payload, 1 + bitmap_bytes);

	if(rx_crc != computed_crc)
	{
		cmd_sack_v2_crc_fail_count++;
		// Compose a hex dump for forensic analysis (no fabrication —
		// the bitmap is discarded; the caller falls back to existing
		// retransmit-timeout logic, exactly as if the OFDM frame had
		// been lost in the air per §9.4/A2).
		char hex[2 * sizeof(payload) + 1];
		for(int b = 0; b < payload_len; b++)
			snprintf(&hex[2*b], 3, "%02x", payload[b]);
		hex[2*payload_len] = '\0';
		printf("[CMD-SACK-V2-CRC-FAIL] rx_crc=0x%02x computed=0x%02x nframes=%d payload=%s fail_count=%lld (discarding bitmap; no fabrication per §9.4/A2)\n",
			(unsigned)rx_crc, (unsigned)computed_crc, nframes, hex,
			cmd_sack_v2_crc_fail_count);
		fflush(stdout);
		return false;
	}

	// CRC pass — write out the bitmap.
	*out_batch_seq_id = payload[0];
	for(int i = 0; i < nframes; i++)
	{
		out_bitmap[i] = (payload[1 + (i / 8)] & (1u << (i % 8))) != 0;
	}

	// Update CMD-side observability state (for tests).
	cmd_sack_v2_rx_count++;
	cmd_sack_v2_last_rx_batch_seq_id = (int)payload[0];
	cmd_sack_v2_last_rx_nbits = nframes;
	int copy_n = bitmap_bytes;
	if(copy_n > (int)sizeof(cmd_sack_v2_last_rx_bitmap))
		copy_n = (int)sizeof(cmd_sack_v2_last_rx_bitmap);
	for(int b = 0; b < copy_n; b++)
		cmd_sack_v2_last_rx_bitmap[b] = payload[1 + b];

	// Log decoded bitmap byte-for-byte for the v2<->v2 byte-identity test.
	char hex[2 * sizeof(payload) + 1];
	for(int b = 0; b < payload_len; b++)
		snprintf(&hex[2*b], 3, "%02x", payload[b]);
	hex[2*payload_len] = '\0';
	printf("[CMD-SACK-V2] batch_seq_id=%u nframes=%d bitmap_bytes=%d payload=%s crc8_ok=0x%02x rx_count=%lld\n",
		(unsigned)payload[0], nframes, bitmap_bytes, hex,
		(unsigned)rx_crc, cmd_sack_v2_rx_count);
	fflush(stdout);
	return true;
}

// Transmit BREAK tone pattern — emergency "drop to ROBUST_0" signal
void cl_arq_controller::send_break_pattern()
{
	if(passive_monitor) return;
	printf("[TX-BREAK] Sending BREAK pattern on CONFIG_%d\n", current_configuration);
	fflush(stdout);

	ptt_on();

	cl_timer ptt_on_delay_timer, ptt_off_delay_timer;
	ptt_on_delay_timer.start();

	int pattern_samples = telecom_system->ack_pattern_passband_samples;
	int symbol_period = telecom_system->data_container.Nofdm * telecom_system->data_container.interpolation_rate;

	int padded_size = pattern_samples + 2 * symbol_period;
	double *raw_output = new double[padded_size];
	double *filtered1 = new double[padded_size];
	double *filtered2 = new double[padded_size];

	if(!raw_output || !filtered1 || !filtered2) exit(-35);

	memset(raw_output, 0, padded_size * sizeof(double));

	// Generate BREAK pattern passband (different tones from ACK)
	telecom_system->generate_break_pattern_passband(&raw_output[symbol_period]);

	memcpy(&raw_output[0], &raw_output[symbol_period], symbol_period * sizeof(double));
	memcpy(&raw_output[symbol_period + pattern_samples], &raw_output[pattern_samples], symbol_period * sizeof(double));

	memset(filtered1, 0, padded_size * sizeof(double));
	memset(filtered2, 0, padded_size * sizeof(double));
	telecom_system->ofdm.FIR_tx1.apply(raw_output, filtered1, padded_size);
	telecom_system->ofdm.FIR_tx2.apply(filtered1, filtered2, padded_size);

	while(ptt_on_delay_timer.get_elapsed_time_ms() < ptt_on_delay_ms)
		msleep(1);

	if(pilot_tone_ms > 0 && pilot_tone_hz > 0)
	{
		const double SAMPLE_RATE = 48000.0;
		const double PILOT_FREQ = (double)pilot_tone_hz;
		const double PI = 3.14159265358979323846;
		int pilot_samples = (int)(pilot_tone_ms * SAMPLE_RATE / 1000.0);
		double* pilot_buffer = new double[pilot_samples];

		for(int i = 0; i < pilot_samples; i++)
		{
			double t = (double)i / SAMPLE_RATE;
			double envelope = 1.0;
			int ramp_samples = (int)(SAMPLE_RATE * 0.005);
			if(i < ramp_samples)
				envelope = (double)i / ramp_samples;
			else if(i > pilot_samples - ramp_samples)
				envelope = (double)(pilot_samples - i) / ramp_samples;
			pilot_buffer[i] = envelope * 0.5 * sin(2.0 * PI * PILOT_FREQ * t);
		}

		tx_transfer(pilot_buffer, pilot_samples);
		delete[] pilot_buffer;
	}

	tx_transfer(&filtered2[symbol_period], pattern_samples);

	while(size_buffer(playback_buffer) > 0)
		msleep(1);

	delete[] raw_output;
	delete[] filtered1;
	delete[] filtered2;

	// Flush before ptt_off_delay (same rationale as send_ack_pattern).
	telecom_system->data_container.rx_mute = 1;
	msleep(RX_MUTE_GUARD_MS);
	circular_buf_reset(capture_buffer);
	{
		int buf_samples = telecom_system->data_container.Nofdm * telecom_system->data_container.buffer_Nsymb * telecom_system->data_container.interpolation_rate;
		MUTEX_LOCK(&capture_prep_mutex);
		memset(telecom_system->data_container.passband_delayed_data, 0, 2 * buf_samples * sizeof(double));
		telecom_system->data_container.ring_write_index = 0;
		MUTEX_UNLOCK(&capture_prep_mutex);
	}
	telecom_system->data_container.rx_mute = 0;
	telecom_system->data_container.nUnder_processing_events = 0;
	telecom_system->receive_stats.delay_of_last_decoded_message = -1;
	telecom_system->receive_stats.mfsk_search_raw = 0;
	telecom_system->receive_stats.ofdm_search_raw = 0;
	telecom_system->receive_stats.ofdm_batch_active = false;
	{
		// One frame + 10 symbol margin for PTT turnaround settle.
		int frame_symb = telecom_system->data_container.preamble_nSymb + telecom_system->data_container.Nsymb;
		telecom_system->data_container.frames_to_read = frame_symb + 10;
	}

	printf("[TX-BREAK] Done, flushed capture buffer, ftr=%d\n", telecom_system->data_container.frames_to_read.load());
	fflush(stdout);

	ptt_off_delay_timer.start();
	while(ptt_off_delay_timer.get_elapsed_time_ms() < ptt_off_delay_ms)
		msleep(1);

	ptt_off();
}

// TX "I am Mercury" HAIL beacon — prefix + optional CRC suffix for directed hailing.
void cl_arq_controller::send_hail_pattern()
{
	if(passive_monitor) return;
	printf("[TX-HAIL] Sending HAIL beacon (%s, %d symbols)\n",
		telecom_system->ack_mfsk.hail_directed ? "directed" : "undirected",
		telecom_system->ack_mfsk.hail_detect_nsymb);
	fflush(stdout);

	ptt_on();

	cl_timer ptt_on_delay_timer, ptt_off_delay_timer;
	ptt_on_delay_timer.start();

	int hail_nsymb = telecom_system->ack_mfsk.hail_detect_nsymb;
	int sym_samples = telecom_system->data_container.Nofdm
	                * telecom_system->data_container.interpolation_rate;
	int pattern_samples = hail_nsymb * sym_samples;
	int symbol_period = telecom_system->data_container.Nofdm * telecom_system->data_container.interpolation_rate;

	int padded_size = pattern_samples + 2 * symbol_period;
	double *raw_output = new double[padded_size];
	double *filtered1 = new double[padded_size];
	double *filtered2 = new double[padded_size];

	if(!raw_output || !filtered1 || !filtered2) exit(-36);

	memset(raw_output, 0, padded_size * sizeof(double));

	telecom_system->generate_hail_pattern_passband(&raw_output[symbol_period]);

	memcpy(&raw_output[0], &raw_output[symbol_period], symbol_period * sizeof(double));
	memcpy(&raw_output[symbol_period + pattern_samples], &raw_output[pattern_samples], symbol_period * sizeof(double));

	memset(filtered1, 0, padded_size * sizeof(double));
	memset(filtered2, 0, padded_size * sizeof(double));
	telecom_system->ofdm.FIR_tx1.apply(raw_output, filtered1, padded_size);
	telecom_system->ofdm.FIR_tx2.apply(filtered1, filtered2, padded_size);

	while(ptt_on_delay_timer.get_elapsed_time_ms() < ptt_on_delay_ms)
		msleep(1);

	if(pilot_tone_ms > 0 && pilot_tone_hz > 0)
	{
		const double SAMPLE_RATE = 48000.0;
		const double PILOT_FREQ = (double)pilot_tone_hz;
		const double PI = 3.14159265358979323846;
		int pilot_samples = (int)(pilot_tone_ms * SAMPLE_RATE / 1000.0);
		double* pilot_buffer = new double[pilot_samples];

		for(int i = 0; i < pilot_samples; i++)
		{
			double t = (double)i / SAMPLE_RATE;
			double envelope = 1.0;
			int ramp_samples = (int)(SAMPLE_RATE * 0.005);
			if(i < ramp_samples)
				envelope = (double)i / ramp_samples;
			else if(i > pilot_samples - ramp_samples)
				envelope = (double)(pilot_samples - i) / ramp_samples;
			pilot_buffer[i] = envelope * 0.5 * sin(2.0 * PI * PILOT_FREQ * t);
		}

		tx_transfer(pilot_buffer, pilot_samples);
		delete[] pilot_buffer;
	}

	tx_transfer(&filtered2[symbol_period], pattern_samples);

	while(size_buffer(playback_buffer) > 0)
		msleep(1);

	delete[] raw_output;
	delete[] filtered1;
	delete[] filtered2;

	// Flush before ptt_off_delay (same rationale as send_ack_pattern).
	telecom_system->data_container.rx_mute = 1;
	msleep(RX_MUTE_GUARD_MS);
	circular_buf_reset(capture_buffer);
	{
		int buf_samples = telecom_system->data_container.Nofdm * telecom_system->data_container.buffer_Nsymb * telecom_system->data_container.interpolation_rate;
		MUTEX_LOCK(&capture_prep_mutex);
		memset(telecom_system->data_container.passband_delayed_data, 0, 2 * buf_samples * sizeof(double));
		telecom_system->data_container.ring_write_index = 0;
		MUTEX_UNLOCK(&capture_prep_mutex);
	}
	telecom_system->data_container.rx_mute = 0;
	telecom_system->data_container.nUnder_processing_events = 0;
	telecom_system->receive_stats.delay_of_last_decoded_message = -1;
	telecom_system->receive_stats.mfsk_search_raw = 0;
	telecom_system->receive_stats.ofdm_search_raw = 0;
	telecom_system->receive_stats.ofdm_batch_active = false;
	// Short ftr for fast HAIL response scanning (not a full LDPC frame).
	// The listen loop polls receive_hail_pattern() which needs ftr==0.
	telecom_system->data_container.frames_to_read = 2;

	printf("[TX-HAIL] Done, flushed capture buffer, ftr=%d\n", telecom_system->data_container.frames_to_read.load());
	fflush(stdout);

	ptt_off_delay_timer.start();
	while(ptt_off_delay_timer.get_elapsed_time_ms() < ptt_off_delay_ms)
		msleep(1);

	ptt_off();
}

// RX: Detect HAIL beacon in capture buffer tail (same mechanism as receive_ack_pattern).
bool cl_arq_controller::receive_hail_pattern()
{
	const int tail_nsymb = telecom_system->ack_mfsk.hail_detect_nsymb + 8 + 16;
	int sym_samples = telecom_system->data_container.Nofdm
	                * telecom_system->data_container.interpolation_rate;
	int signal_period = sym_samples * telecom_system->data_container.buffer_Nsymb;
	int tail_samples = tail_nsymb * sym_samples;
	if(tail_samples > signal_period)
		tail_samples = signal_period;
	int tail_offset = signal_period - tail_samples;

	MUTEX_LOCK(&capture_prep_mutex);

	if(telecom_system->data_container.frames_to_read == 0)
	{
		int rwi = telecom_system->data_container.ring_write_index;
		memcpy(telecom_system->data_container.ready_to_process_passband_delayed_data,
			&telecom_system->data_container.passband_delayed_data[rwi + tail_offset],
			tail_samples * sizeof(double));

		telecom_system->data_container.data_ready = 0;
		MUTEX_UNLOCK(&capture_prep_mutex);

		int matched_count = 0;
		int suffix_matched = 0;
		int suffix_start = telecom_system->ack_mfsk.hail_directed
		                  ? telecom_system->ack_mfsk.ack_pattern_nsymb : 0;
		double metric = telecom_system->detect_hail_pattern_from_passband(
			telecom_system->data_container.ready_to_process_passband_delayed_data,
			tail_samples, &matched_count, suffix_start, &suffix_matched);

		// Separate base pattern and suffix verification:
		// Base: enough Sidelnikov symbols match (same for all stations)
		// Suffix: callsign-derived tones must match independently (directed HAIL only)
		int base_matched = matched_count - suffix_matched;
		bool base_ok = base_matched >= telecom_system->ack_mfsk.hail_match_threshold;
		bool suffix_ok = !telecom_system->ack_mfsk.hail_directed
		              || suffix_matched >= (telecom_system->ack_mfsk.HAIL_SUFFIX_LEN - 1);
		// Per-match quality: noise gives metric/matched ≈ 2/Nc (0.2 NB, 0.04 WB).
		// Real signals give 0.5+. Gate at 0.3 to reject noise false alarms.
		double quality = (matched_count > 0) ? metric / matched_count : 0.0;
		if(base_ok && suffix_ok && metric >= 3.0 && quality >= 0.3)
		{
			printf("[HAIL] Detected: base=%d/%d suffix=%d/%d metric=%.1f quality=%.2f%s\n",
				base_matched, telecom_system->ack_mfsk.hail_match_threshold,
				suffix_matched, telecom_system->ack_mfsk.HAIL_SUFFIX_LEN,
				metric, quality,
				telecom_system->ack_mfsk.hail_directed ? " (directed)" : "");
			fflush(stdout);
#ifdef MERCURY_GUI_ENABLED
			gui_push_monitor_event("[HAIL detected]", false);
#endif
			MUTEX_LOCK(&capture_prep_mutex);
			telecom_system->data_container.frames_to_read =
				telecom_system->data_container.preamble_nSymb + telecom_system->data_container.Nsymb;
			telecom_system->data_container.nUnder_processing_events = 0;
			telecom_system->receive_stats.mfsk_search_raw = 0;
			telecom_system->receive_stats.ofdm_search_raw = 0;
			telecom_system->receive_stats.ofdm_batch_active = false;
			MUTEX_UNLOCK(&capture_prep_mutex);
			return true;
		}

		telecom_system->data_container.frames_to_read = 2;
		telecom_system->data_container.nUnder_processing_events = 0;
		return false;
	}

	telecom_system->data_container.data_ready = 0;
	MUTEX_UNLOCK(&capture_prep_mutex);
	return false;
}

// Receive and detect ACK tone pattern, returns true if detected.
// Scans the TAIL (newest symbols) of the capture buffer.
// Buffer was zeroed after TX, so the tail contains only fresh audio.
// Called frequently (every ~2 symbols / 45ms) to adapt to any round-trip latency.
// When turbo_snr_ack_enabled, also decodes 8 SNR suffix symbols and stores in turbo_received_snr.
bool cl_arq_controller::receive_ack_pattern()
{
	// Tail must cover the entire fresh audio region (= initial guard).
	// Tail = pattern length + margin + SNR suffix. Ensures ACKs arriving early are captured.
	// Tail must be large enough that even if the ACK is detected late in the buffer,
	// the suffix symbols still fit. Need: ack_pattern_nsymb for search range +
	// full pattern (with suffix) + margin.
	// WB: 16+20+16=52. NB M=8: 32+40+16=88. NB M=4: 48+56+16=120.
	int ack_nsymb = telecom_system->ack_mfsk.ack_pattern_nsymb;
	int pattern_len = turbo_snr_ack_enabled ?
		telecom_system->ack_mfsk.ack_snr_pattern_nsymb() :
		ack_nsymb;
	const int tail_nsymb = ack_nsymb + pattern_len + 16;
	int sym_samples = telecom_system->data_container.Nofdm
	                * telecom_system->data_container.interpolation_rate;
	int signal_period = sym_samples * telecom_system->data_container.buffer_Nsymb;
	int tail_samples = tail_nsymb * sym_samples;
	if(tail_samples > signal_period)
		tail_samples = signal_period;
	int tail_offset = signal_period - tail_samples;

	MUTEX_LOCK(&capture_prep_mutex);

	if(telecom_system->data_container.frames_to_read == 0)
	{
		// Snapshot only the tail (newest audio) — smaller copy, shorter mutex hold
		int rwi = telecom_system->data_container.ring_write_index;
		memcpy(telecom_system->data_container.ready_to_process_passband_delayed_data,
			&telecom_system->data_container.passband_delayed_data[rwi + tail_offset],
			tail_samples * sizeof(double));

		telecom_system->data_container.data_ready = 0;
		MUTEX_UNLOCK(&capture_prep_mutex);

		int matched_count = 0;

		// Energy gate (Opt 1): the ACK matched filter runs ~528 FFTs per call,
		// dominating CPU during idle polling. Skip the entire detection when
		// the recent audio tail has no signal. Floor measured at -73 dBFS
		// (≈0.0002 RMS); a single MFSK tone is at ≈0.02 RMS. Gate at 0.001
		// (≈14 dB above noise) — well below the smallest real ACK level but
		// well above silence/rx_mute noise.
		// Per-window reset of energy-log tracker (diagnostic).
		static bool energy_logged_this_window = false;
		if(ack_diag_poll_count == 0)
		{
			mtl::log_event("cmd_first_ack_poll");
			energy_logged_this_window = false;
		}
		const double ACK_ENERGY_GATE_RMS = 0.001;
		int probe_n = 8 * sym_samples;
		if(probe_n > tail_samples) probe_n = tail_samples;
		double* tail_ptr = telecom_system->data_container.ready_to_process_passband_delayed_data
			+ (tail_samples - probe_n);
		double sumsq = 0.0;
		for(int i = 0; i < probe_n; i++) sumsq += tail_ptr[i] * tail_ptr[i];
		double tail_rms = std::sqrt(sumsq / probe_n);
		if(!energy_logged_this_window && tail_rms > 0.005)
		{
			mtl::log_event_kv("cmd_ack_buffer_energy", "rms=%.4f", tail_rms);
			energy_logged_this_window = true;
		}
		if(tail_rms < ACK_ENERGY_GATE_RMS)
		{
			// Silent buffer — skip FFT-heavy ACK search this poll.
			ack_diag_poll_count++;
			MUTEX_LOCK(&capture_prep_mutex);
			telecom_system->data_container.frames_to_read = 2;
			telecom_system->data_container.nUnder_processing_events = 0;
			MUTEX_UNLOCK(&capture_prep_mutex);
			return false;
		}

		if(turbo_snr_ack_enabled)
		{
			// Turboshift mode: detect ACK and decode SNR suffix.
			// The suffix symbols arrive AFTER the ACK pattern. The ACK is
			// detected when 24+ of 32 symbols are in the buffer, but the
			// 8 suffix symbols may still be in flight (~200ms NB, ~100ms WB).
			// When ACK is found but suffix is unreadable, defer and keep
			// polling until the suffix arrives (up to 500ms timeout).
			bool snr_valid = false;
			float decoded_snr = telecom_system->detect_ack_snr_from_passband(
				telecom_system->data_container.ready_to_process_passband_delayed_data,
				tail_samples, &matched_count, &snr_valid);

			if(matched_count >= telecom_system->ack_mfsk.ack_match_threshold)
			{
				// SACK-before-ACK guard (turboshift path)
				if(sack_enabled)
				{
					int sack_matched = 0;
					telecom_system->detect_sack_pattern_from_passband(
						telecom_system->data_container.ready_to_process_passband_delayed_data,
						tail_samples, &sack_matched);
					if(sack_matched > matched_count)
					{
						printf("[ACK-SNR-GUARD] Suppressed: ACK=%d SACK=%d (SACK wins)\n",
							matched_count, sack_matched);
						fflush(stdout);
						telecom_system->data_container.frames_to_read = 2;
						telecom_system->data_container.nUnder_processing_events = 0;
						return false;
					}
				}

				if(snr_valid)
				{
					turbo_received_snr = decoded_snr;
					if(decoded_snr > turbo_best_snr)
						turbo_best_snr = decoded_snr;
					turbo_snr_defer_timer.reset();
					printf("[CMD-ACK-SNR] ACK detected with SNR=%.1f dB (matched=%d)\n",
						decoded_snr, matched_count);
					fflush(stdout);

#ifdef MERCURY_GUI_ENABLED
					gui_push_monitor_event("[ACK+SNR]", false);
#endif
					MUTEX_LOCK(&capture_prep_mutex);
					telecom_system->data_container.frames_to_read = 4;
					telecom_system->data_container.nUnder_processing_events = 0;
					telecom_system->receive_stats.mfsk_search_raw = 0;
					telecom_system->receive_stats.ofdm_search_raw = 0;
					telecom_system->receive_stats.ofdm_batch_active = false;
					MUTEX_UNLOCK(&capture_prep_mutex);
					return true;
				}
				else
				{
					// ACK detected but suffix not yet arrived. Start/check defer timer.
					if(turbo_snr_defer_timer.counting != COUNTING)
					{
						turbo_snr_defer_timer.start();
						printf("[CMD-ACK-SNR] ACK detected, waiting for suffix (matched=%d)\n",
							matched_count);
						fflush(stdout);
					}
					else if(turbo_snr_defer_timer.get_elapsed_time_ms() > 500)
					{
						// Timeout: accept ACK without SNR
						turbo_received_snr = -99.0f;
						turbo_snr_defer_timer.reset();
						printf("[CMD-ACK-SNR] ACK detected, suffix timeout (matched=%d)\n",
							matched_count);
						fflush(stdout);

#ifdef MERCURY_GUI_ENABLED
						gui_push_monitor_event("[ACK+SNR]", false);
#endif
						MUTEX_LOCK(&capture_prep_mutex);
						telecom_system->data_container.frames_to_read = 4;
						telecom_system->data_container.nUnder_processing_events = 0;
						telecom_system->receive_stats.mfsk_search_raw = 0;
						telecom_system->receive_stats.ofdm_search_raw = 0;
						telecom_system->receive_stats.ofdm_batch_active = false;
						MUTEX_UNLOCK(&capture_prep_mutex);
						return true;
					}
					// else: keep polling, suffix not yet in buffer
					return false;
				}
			}
			else
			{
				// No ACK detected at all — reset defer timer if it was running
				if(turbo_snr_defer_timer.counting == COUNTING &&
				   turbo_snr_defer_timer.get_elapsed_time_ms() > 500)
					turbo_snr_defer_timer.reset();

				// ACK poll diagnostic removed — printf/fflush in hot polling loop caused ~175-875ms cumulative latency
			}
		}
		else
		{
			// Normal mode: just detect ACK pattern
			uint32_t this_mask = 0;
			double metric = telecom_system->detect_ack_pattern_from_passband(
				telecom_system->data_container.ready_to_process_passband_delayed_data,
				tail_samples, &matched_count, &this_mask);

			// Track peak detection values for timeout diagnostic (no printf in hot loop)
			if(matched_count > ack_diag_peak_matched)
			{
				ack_diag_peak_matched = matched_count;
				ack_diag_peak_mask = this_mask;
			}
			if(metric > ack_diag_peak_metric) ack_diag_peak_metric = metric;
			ack_diag_poll_count++;

			// Metric threshold: For WB M=16, matched>=8/16 has P(false)~5.6e-5/pos.
			// Random noise has metric≈8/Nc=0.16 at 8 matches. metric>=0.5 rejects
			// noise while accepting marginal signals (was 3.0, caused ~50% timeouts).
			// Phase-2: --ack-metric-threshold=F overrides.
			if(matched_count >= telecom_system->ack_mfsk.ack_match_threshold && metric >= ack_metric_threshold)
			{
				// SACK-before-ACK guard: when SACK is enabled, cross-check against
				// SACK correlator on the same audio. If SACK matches MORE symbols
				// than ACK, this is actually a SACK being misdetected as ACK
				// (can happen due to partial tone overlap after hop expansion).
				if(sack_enabled)
				{
					int sack_matched = 0;
					telecom_system->detect_sack_pattern_from_passband(
						telecom_system->data_container.ready_to_process_passband_delayed_data,
						tail_samples, &sack_matched);
					if(sack_matched > matched_count)
					{
						printf("[ACK-GUARD] Suppressed: ACK=%d SACK=%d (SACK wins)\n",
							matched_count, sack_matched);
						fflush(stdout);
						// Don't accept as ACK — let receive_sack_pattern() handle it
						telecom_system->data_container.frames_to_read = 2;
						telecom_system->data_container.nUnder_processing_events = 0;
						return false;
					}
				}

#ifdef MERCURY_GUI_ENABLED
				gui_push_monitor_event("[ACK]", false);
#endif
				MUTEX_LOCK(&capture_prep_mutex);
				telecom_system->data_container.frames_to_read = 4;
				telecom_system->data_container.nUnder_processing_events = 0;
				telecom_system->receive_stats.mfsk_search_raw = 0;
				telecom_system->receive_stats.ofdm_search_raw = 0;
				telecom_system->receive_stats.ofdm_batch_active = false;
				MUTEX_UNLOCK(&capture_prep_mutex);
				return true;
			}
		}

		// Not detected — poll again in 2 symbols (~45ms).
		// This throttles regardless of sack_enabled. When SACK is enabled,
		// the caller runs receive_sack_pattern() first (which no longer throttles),
		// then receive_ack_pattern() second. Throttling here gates both checks.
		telecom_system->data_container.frames_to_read = 2;
		telecom_system->data_container.nUnder_processing_events = 0;
		return false;
	}

	telecom_system->data_container.data_ready = 0;
	MUTEX_UNLOCK(&capture_prep_mutex);
	return false;
}

void cl_arq_controller::receive()
{
	int signal_period = telecom_system->data_container.Nofdm * telecom_system->data_container.buffer_Nsymb * telecom_system->data_container.interpolation_rate; // in samples
	int symbol_period = telecom_system->data_container.Nofdm * telecom_system->data_container.interpolation_rate;

#if 0 // TODO:  do we need this?
	if(telecom_system->data_container.data_ready == 0)
	{
		msleep(1);
		return;
	}
#endif
	MUTEX_LOCK(&capture_prep_mutex);
	st_receive_stats received_message_stats;


	if(telecom_system->data_container.frames_to_read==0)
	{


		int rwi = telecom_system->data_container.ring_write_index;
		memcpy(telecom_system->data_container.ready_to_process_passband_delayed_data, &telecom_system->data_container.passband_delayed_data[rwi], signal_period * sizeof(double));

		// DIAG: ring buffer snapshot debug (verbose only — buffer scan is expensive)
		if(g_verbose)
		{
			double peak_head = 0, peak_mid = 0, peak_tail = 0;
			int sp = signal_period;
			for(int i = 0; i < sp/10 && i < sp; i++) {
				double v = fabs(telecom_system->data_container.ready_to_process_passband_delayed_data[i]);
				if(v > peak_head) peak_head = v;
			}
			for(int i = sp/2 - sp/20; i < sp/2 + sp/20 && i < sp; i++) {
				double v = fabs(telecom_system->data_container.ready_to_process_passband_delayed_data[i]);
				if(v > peak_mid) peak_mid = v;
			}
			for(int i = sp - sp/10; i < sp; i++) {
				double v = fabs(telecom_system->data_container.ready_to_process_passband_delayed_data[i]);
				if(v > peak_tail) peak_tail = v;
			}
			static int snap_count = 0;
			snap_count++;
			printf("[RING-SNAP] #%d rwi=%d sp=%d head=%.4f mid=%.4f tail=%.4f nUnder=%d M=%.0f\n",
				snap_count, rwi, sp, peak_head, peak_mid, peak_tail,
				telecom_system->data_container.nUnder_processing_events.load(),
				telecom_system->M);
			fflush(stdout);
		}

		// Previously: zero passband_delayed_data after copy to prevent stale
		// preamble re-detection from self-echo. Now handled by rx_mute (Bug #44):
		// capture thread zeros audio during TX, so self-echo never enters buffer.
		// Removing zeroing is critical for OFDM gearshift: after ACK TX + flush,
		// CMD turnaround takes ~1-2s. With zeroing + ftr=8 anti-spin, only 8
		// symbols accumulate before the next attempt zeros everything — signal
		// never builds up for a full 52-symbol frame. Without zeroing, signal
		// accumulates across attempts until a full frame is available.

		// Clear data_ready while we have the lock, before unlocking
		telecom_system->data_container.data_ready = 0;

		MUTEX_UNLOCK(&capture_prep_mutex);

		if(telecom_system->M != MOD_MFSK && g_verbose)
		{
			int sym_samples = telecom_system->data_container.Nofdm * telecom_system->data_container.interpolation_rate;
			int buf_nsymb = telecom_system->data_container.buffer_Nsymb;
			int chunk_symb = (buf_nsymb + 9) / 10;
			int chunk_samples = chunk_symb * sym_samples;
			printf("[BUF-ENERGY] nUnder=%d |", telecom_system->data_container.nUnder_processing_events.load());
			for(int c = 0; c < signal_period; c += chunk_samples)
			{
				double peak = 0.0;
				int end = (c + chunk_samples < signal_period) ? c + chunk_samples : signal_period;
				for(int s = c; s < end; s++)
				{
					double v = fabs(telecom_system->data_container.ready_to_process_passband_delayed_data[s]);
					if(v > peak) peak = v;
				}
				printf(" %.3f", peak);
			}
			printf("\n");
			fflush(stdout);
		}

#ifdef MERCURY_GUI_ENABLED
		// Apply live LDPC iteration limit from GUI
		int gui_ldpc_max = g_gui_state.ldpc_iterations_max.load();
		if (gui_ldpc_max >= 5 && gui_ldpc_max <= 50)
			telecom_system->ldpc.nIteration_max = gui_ldpc_max;
#endif

		auto proc_start = std::chrono::steady_clock::now();

		// Monitor mode with parallel decoders: try all 17 OFDM configs sequentially.
		// Falls through to single receive_byte() for MFSK modes or if decoders not ready.
		if(passive_monitor && monitor_decoders_ready && !is_robust_config(current_configuration))
		{
			int winning_config = parallel_monitor_decode(
				telecom_system->data_container.ready_to_process_passband_delayed_data,
				signal_period, received_message_stats);
			if(winning_config >= 0)
			{
				// Switch primary config FIRST (may deinit/reinit data_byte),
				// then copy decoded data from staging buffer.
				if(winning_config != current_configuration)
				{
					printf("[MONITOR] Parallel decode found CONFIG_%d (was CONFIG_%d)\n",
						winning_config, current_configuration);
					data_configuration = winning_config;
					load_configuration(winning_config, PHYSICAL_LAYER_ONLY, YES);
				}
				// Copy decoded data from staging buffer to primary's data_byte.
				// Safe now: load_configuration has finished reinit.
				memcpy(telecom_system->data_container.data_byte,
					monitor_decoded_data, monitor_decoded_len * sizeof(int));
			}
		}
		else
		{
			received_message_stats = telecom_system->receive_byte(
				telecom_system->data_container.ready_to_process_passband_delayed_data,
				telecom_system->data_container.data_byte);
		}

		auto proc_end = std::chrono::steady_clock::now();
		double proc_ms = std::chrono::duration<double, std::milli>(proc_end - proc_start).count();

		// Frame period = (preamble + data symbols) in wall clock time
		double frame_samples = (double)(telecom_system->data_container.Nofdm *
			(telecom_system->data_container.Nsymb + telecom_system->data_container.preamble_nSymb) *
			telecom_system->data_container.interpolation_rate);
		double frame_ms = (frame_samples / 48000.0) * 1000.0;
		float load = (frame_ms > 0) ? (float)(proc_ms / frame_ms) : 0.0f;

#ifdef MERCURY_GUI_ENABLED
		g_gui_state.processing_load.store(load);
		{
			size_t buf_used = size_buffer(capture_buffer);
			size_t buf_cap = circular_buf_capacity(capture_buffer);
			g_gui_state.buffer_fill_pct.store(buf_cap > 0 ? 100.0f * (float)buf_used / (float)buf_cap : 0.0f);
		}
#endif

		measurements.signal_stregth_dbm = received_message_stats.signal_stregth_dbm;

		// Ring buffer: zero the entire decoded frame (preamble + data)
		// after successful decode to prevent false Schmidl-Cox detections.
		// Without this, the data symbols of decoded frames contain OFDM
		// structure (pilots, subcarrier patterns) that produce false
		// autocorrelation peaks at metric 0.08-0.22, causing cascading
		// FAILs that waste 10-20 seconds per turnaround.
		// IMPORTANT: Only on OK decode. Zeroing on FAIL would destroy
		// real preambles during turnaround (data still arriving).
		if(telecom_system->M != MOD_MFSK
			&& received_message_stats.message_decoded == YES
			&& received_message_stats.delay > 0)
		{
			int sp = signal_period;
			int frame_syms = telecom_system->data_container.preamble_nSymb
				+ telecom_system->get_active_nsymb();
			int frame_samples = frame_syms * symbol_period;
			int frame_ring_start = (rwi + received_message_stats.delay) % sp;

			MUTEX_LOCK(&capture_prep_mutex);
			for(int k = 0; k < frame_samples; k++)
			{
				int pos = (frame_ring_start + k) % sp;
				telecom_system->data_container.passband_delayed_data[pos] = 0.0;
				telecom_system->data_container.passband_delayed_data[pos + sp] = 0.0;
			}
			MUTEX_UNLOCK(&capture_prep_mutex);
		}

		// Bug #35 diagnostic: track every decode attempt (verbose only)
		if(g_verbose)
		{
			static int decode_attempt = 0;
			decode_attempt++;
			if(received_message_stats.message_decoded == YES)
			{
				int nrd = telecom_system->data_container.nBits - telecom_system->ldpc.P;
				int fs = (nrd - 16) / 8;  // frame_size with CRC16
				printf("[RX-DECODE#%d] OK: delay=%d iters=%d ofdm_raw=%d ftr=%d nUnder=%d bytes: %02x %02x %02x %02x %02x %02x\n",
					decode_attempt, received_message_stats.delay, received_message_stats.iterations_done,
					telecom_system->receive_stats.ofdm_search_raw,
					telecom_system->data_container.frames_to_read.load(),
					telecom_system->data_container.nUnder_processing_events.load(),
					telecom_system->data_container.data_byte[0] & 0xFF,
					telecom_system->data_container.data_byte[1] & 0xFF,
					telecom_system->data_container.data_byte[2] & 0xFF,
					telecom_system->data_container.data_byte[3] & 0xFF,
					telecom_system->data_container.data_byte[4] & 0xFF,
					telecom_system->data_container.data_byte[5] & 0xFF);
			}
			else
			{
				if(received_message_stats.delay == -1)
					printf("[RX-DECODE#%d] NO-PREAMBLE: mfsk_raw=%d ofdm_raw=%d nUnder=%d link=%d\n",
						decode_attempt,
						telecom_system->receive_stats.mfsk_search_raw,
						telecom_system->receive_stats.ofdm_search_raw,
						telecom_system->data_container.nUnder_processing_events.load(),
						(int)link_status);
				else
					printf("[RX-DECODE#%d] FAIL: delay=%d metric=%.3f ofdm_raw=%d nUnder=%d link=%d conn=%d\n",
						decode_attempt, received_message_stats.delay,
						telecom_system->receive_stats.coarse_metric,
						telecom_system->receive_stats.ofdm_search_raw,
						telecom_system->data_container.nUnder_processing_events.load(),
						(int)link_status, (int)connection_status);
			}
			fflush(stdout);
		}

		if (received_message_stats.message_decoded==YES)
		{
			int rx_nsymb = telecom_system->get_active_nsymb();
			int rx_frame = rx_nsymb + telecom_system->data_container.preamble_nSymb;
			int end_of_current_message = received_message_stats.delay / symbol_period  + rx_frame;
			int frames_left_in_buffer = telecom_system->data_container.buffer_Nsymb - end_of_current_message;
			if(frames_left_in_buffer<0)
				frames_left_in_buffer=0;

			int nUnder_snapshot = telecom_system->data_container.nUnder_processing_events.load();
			// Only subtract nUnder when there's buffer margin to absorb it.
			// When frames_left <= nUnder, subtracting causes ftr < rx_frame,
			// which means the next frame drifts 1+ symbols higher per decode.
			// Eventually the preamble lands beyond upper_bound and can't decode.
			int nUnder_adj = (frames_left_in_buffer > nUnder_snapshot) ? nUnder_snapshot : 0;
			telecom_system->data_container.frames_to_read=rx_frame-frames_left_in_buffer-nUnder_adj;

			int ftr_clamped = 0;
			if(telecom_system->data_container.frames_to_read < 0)
			{
				// Buffer already has enough data for the next frame — decode immediately.
				// ofdm_search_raw / mfsk_search_raw will skip past the just-decoded frame.
				telecom_system->data_container.frames_to_read = 0;
				ftr_clamped = 1;
			}

			// Minimum shift to keep next frame within extraction bounds.
			// Must account for the turnaround gap: after this decode, the
			// other station receives the ACK then sends the next frame.
			// The gap is ~2s (~93 symbols at 22.67ms/sym).  Without this,
			// the next preamble lands past the buffer end because the ftr
			// was too small to make room.
			//
			// Safety margin (+4): without it, the next batch frame lands
			// exactly at upper_bound.  Any nUnder event (1-3 symbols of
			// capture-thread latency) pushes the preamble 1 symbol past
			// upper_bound → beyond-bounds FAIL → fast-forward → OK → repeat
			// (alternating 50% FAIL rate on VB-Cable batch runs).
			int upper_bound = telecom_system->data_container.buffer_Nsymb - rx_frame;
			int min_ftr = end_of_current_message - upper_bound + 4;
			if(min_ftr > 0 && telecom_system->data_container.frames_to_read < min_ftr)
			{
				telecom_system->data_container.frames_to_read = min_ftr;
				ftr_clamped = 3;
			}

			// Upper clamp: limit shift to end_of_current_message (flush
			// all decoded audio, keep only fresh buffer for next frame).
			// Old clamp was rx_frame which is too small when the turnaround
			// gap pushes the next preamble far into the buffer.
			if(telecom_system->data_container.frames_to_read > end_of_current_message)
			{
				telecom_system->data_container.frames_to_read = end_of_current_message;
				ftr_clamped = 2;
			}

			// === DIAG: success ftr trace (verbose only) ===
			if(g_verbose)
			{
				printf("[FTR-OK] CONFIG_%d ftr=%d delay_sym=%d end=%d left=%d nUnder=%d clamped=%d\n",
					current_configuration,
					telecom_system->data_container.frames_to_read.load(),
					received_message_stats.delay / symbol_period,
					end_of_current_message, frames_left_in_buffer, nUnder_snapshot, ftr_clamped);
				fflush(stdout);
			}
			if (g_verbose)
				printf("[RX-TIMING] OK: delay=%d delay_symb=%d rx_frame=%d end=%d left=%d nUnder=%d ftr=%d clamped=%d proc=%.0fms\n",
					received_message_stats.delay, received_message_stats.delay / symbol_period,
					rx_frame, end_of_current_message, frames_left_in_buffer, nUnder_snapshot,
					telecom_system->data_container.frames_to_read.load(), ftr_clamped, proc_ms);
			fflush(stdout);

			// MFSK anti-re-decode: after successful decode, record where the old
			// frame ends so the next time_sync_mfsk skips past it entirely.
			// Must skip the full frame (preamble + data), not just the preamble,
			// because MFSK data tones can create false preamble correlations.
			// mfsk_search_raw = frame_end_symb - frames_to_read (base value).
			// telecom_system subtracts nUnder at search time for the effective start.
			if(telecom_system->M == MOD_MFSK)
			{
				int frame_end_symb = received_message_stats.delay / symbol_period + rx_frame;
				telecom_system->receive_stats.mfsk_search_raw =
					frame_end_symb - telecom_system->data_container.frames_to_read;
			}
			else
			{
				// OFDM anti-re-decode: record frame end so next Schmidl-Cox
				// search skips past this decoded preamble.
				// Account for nUnder: during LDPC decode, the buffer shifted
				// nUnder_snapshot times. Without subtracting it here, search_raw
				// overshoots by nUnder symbols after the reset at line 3291,
				// causing the next batch frame to land BEFORE ofdm_skip and
				// become invisible to detection.
				// -1 margin catches frames 1 symbol below expected position.
				int frame_end_symb = received_message_stats.delay / symbol_period + rx_frame;
				telecom_system->receive_stats.ofdm_search_raw =
					frame_end_symb - telecom_system->data_container.frames_to_read - nUnder_snapshot - 1;
				if(telecom_system->receive_stats.ofdm_search_raw < 0)
					telecom_system->receive_stats.ofdm_search_raw = 0;
				// Clamp at upper_bound: search_raw can exceed buffer_Nsymb - rx_frame
				// when ftr is clamped to 0 (frame near buffer end). Without clamping,
				// next search starts past upper_bound → recovery forces unnecessary FAIL.
				int upper_clamp = telecom_system->data_container.buffer_Nsymb.load() - rx_frame;
				if(upper_clamp > 0 && telecom_system->receive_stats.ofdm_search_raw > upper_clamp)
					telecom_system->receive_stats.ofdm_search_raw = upper_clamp;
				telecom_system->receive_stats.ofdm_batch_active = true;

				// Opportunistic scan success: reset failure counter
				if(passive_monitor) monitor_consec_ofdm_fail = 0;

				if(g_verbose)
				{
					printf("[OFDM-SKIP] frame_end=%d ftr=%d search_raw=%d clamped=%d\n",
						frame_end_symb, telecom_system->data_container.frames_to_read.load(),
						telecom_system->receive_stats.ofdm_search_raw, ftr_clamped);
					fflush(stdout);
				}
			}

			telecom_system->receive_stats.delay_of_last_decoded_message += (rx_frame - (telecom_system->data_container.frames_to_read + telecom_system->data_container.nUnder_processing_events)) * symbol_period;

			telecom_system->data_container.nUnder_processing_events = 0;

			measurements.frequency_offset = received_message_stats.freq_offset;
			// Always update RX SNR from any decoded LDPC frame (regardless of role).
			// With pattern ACK, the commander never decodes LDPC during ACK detection,
			// so SNR_uplink only refreshes during SWITCH_ROLE when we receive data.
			measurements.SNR_uplink = received_message_stats.SNR;
			if(this->role == RESPONDER)
			{
				measurements.SNR_downlink = received_message_stats.SNR;
			}

			{
				int byte_copy_len = this->max_data_length + this->max_header_length;
				if(byte_copy_len > N_MAX/8) byte_copy_len = N_MAX/8;
				for(int i=0; i < byte_copy_len; i++)
				{
					message_TxRx_byte_buffer[i] = (char)telecom_system->data_container.data_byte[i];
				}
	
			}
			// In passive monitor mode, accept ALL connection_ids and adopt the session
			if(passive_monitor && this->connection_id == 0 && message_TxRx_byte_buffer[1] != BROADCAST_ID)
			{
				this->connection_id = message_TxRx_byte_buffer[1];
				printf("[MONITOR] Adopted connection_id=0x%02x\n",
					(unsigned char)this->connection_id);
				fflush(stdout);
			}
			if(passive_monitor || message_TxRx_byte_buffer[1] == this->connection_id || message_TxRx_byte_buffer[1] == BROADCAST_ID)
			{
				messages_rx_buffer.status=RECEIVED;
				messages_rx_buffer.type=message_TxRx_byte_buffer[0];
				// Bit 7 of sequence_number = end-of-batch flag from commander (data frames only)
				if((message_TxRx_byte_buffer[2] & 0x80)
					&& (messages_rx_buffer.type == DATA_LONG || messages_rx_buffer.type == DATA_SHORT))
					last_received_end_of_batch_seq = message_TxRx_byte_buffer[2] & 0x7F;
				messages_rx_buffer.sequence_number=message_TxRx_byte_buffer[2] & 0x7F;
				last_received_message_sequence=messages_rx_buffer.sequence_number;
				// Defensive clamp: never write more than alloc_size (N_MAX/8 = 200) bytes
				// into any .data buffer, regardless of max_data_length + max_header_length.
				const int alloc_size = N_MAX / 8;
				if(messages_rx_buffer.type==ACK_CONTROL  ||  messages_rx_buffer.type==CONTROL)
				{
					int copy_len = max_data_length+max_header_length-CONTROL_ACK_CONTROL_HEADER_LENGTH;
					if(copy_len > alloc_size) copy_len = alloc_size;
					for(int j=0;j<copy_len;j++)
					{
						messages_rx_buffer.data[j]=message_TxRx_byte_buffer[j+CONTROL_ACK_CONTROL_HEADER_LENGTH];
					}
				}
				if( messages_rx_buffer.type==ACK_MULTI || messages_rx_buffer.type==ACK_RANGE)
				{
					int copy_len = max_data_length+max_header_length-ACK_MULTI_ACK_RANGE_HEADER_LENGTH;
					if(copy_len > alloc_size) copy_len = alloc_size;
					for(int j=0;j<copy_len;j++)
					{
						messages_rx_buffer.data[j]=message_TxRx_byte_buffer[j+ACK_MULTI_ACK_RANGE_HEADER_LENGTH];
					}
				}
				else if(messages_rx_buffer.type==SACK_RSP)
				{
					// SACK Design A Step 7 — OFDM SACK_RSP. 3-byte header;
					// payload bytes [batch_seq_id, bitmap..., CRC8] live in
					// messages_rx_buffer.data[0..length-1]. Length is derived
					// by the caller (decode_sack_v2_frame) from the negotiated
					// data_batch_size. We DO NOT decode here — only copy.
					int copy_len = max_data_length+max_header_length-ACK_MULTI_ACK_RANGE_HEADER_LENGTH;
					if(copy_len > alloc_size) copy_len = alloc_size;
					for(int j=0;j<copy_len;j++)
					{
						messages_rx_buffer.data[j]=message_TxRx_byte_buffer[j+ACK_MULTI_ACK_RANGE_HEADER_LENGTH];
					}
					// length field on messages_rx_buffer is not set by the wire
					// for ACK-family frames (the LDPC codeword size implicitly
					// bounds it). The caller will validate using
					// data_batch_size + the CRC8 byte position.
				}
				else if(messages_rx_buffer.type==DATA_LONG)
				{
					// SACK Design A Steps 1+3 — DATA_LONG header parse + batch_seq_id store.
					// v1 (default): byte[3] = id. v2: byte[3] = batch_seq_id, byte[4] = id.
					// batch_seq_id is stored on messages_rx_buffer and (for diagnostics)
					// on last_received_batch_seq_id. NO routing/RX decision keys off it
					// yet — pure scaffolding per Step 3 (decision is Step 4+).
					int eff_hdr = effective_data_long_header_length(sack_v2_enabled);
					if(sack_v2_enabled)
					{
						int bsi = (unsigned char)message_TxRx_byte_buffer[3];
						messages_rx_buffer.batch_seq_id = bsi;
						last_received_batch_seq_id = bsi;
						messages_rx_buffer.id=message_TxRx_byte_buffer[4];
					}
					else
					{
						messages_rx_buffer.batch_seq_id = -1;  // v1: field not on wire
						messages_rx_buffer.id=message_TxRx_byte_buffer[3];
					}
					int copy_len = max_data_length+max_header_length-eff_hdr;
					if(copy_len > alloc_size) copy_len = alloc_size;
					messages_rx_buffer.length=copy_len;
					for(int j=0;j<copy_len;j++)
					{
						messages_rx_buffer.data[j]=message_TxRx_byte_buffer[j+eff_hdr];
					}
					if(sack_v2_enabled)
					{
						printf("[RX-BATCH-SEQ] type=DATA_LONG id=%d seq=%d batch_seq_id=%d (v2)\n",
							(unsigned char)messages_rx_buffer.id,
							(unsigned char)messages_rx_buffer.sequence_number & 0x7F,
							messages_rx_buffer.batch_seq_id);
						fflush(stdout);
					}

				}
				else if(messages_rx_buffer.type==DATA_SHORT)
				{
					// SACK Design A Steps 2+3 — DATA_SHORT header parse + batch_seq_id store.
					// v1 (default): byte[3] = id, byte[4] = length.
					// v2: byte[3] = batch_seq_id, byte[4] = id, byte[5] = length.
					// batch_seq_id is stored but no decision branches on it (Step 4+).
					int eff_hdr = effective_data_short_header_length(sack_v2_enabled);
					if(sack_v2_enabled)
					{
						int bsi = (unsigned char)message_TxRx_byte_buffer[3];
						messages_rx_buffer.batch_seq_id = bsi;
						last_received_batch_seq_id = bsi;
						messages_rx_buffer.id=message_TxRx_byte_buffer[4];
						messages_rx_buffer.length=(unsigned char)message_TxRx_byte_buffer[5];
					}
					else
					{
						messages_rx_buffer.batch_seq_id = -1;  // v1: field not on wire
						messages_rx_buffer.id=message_TxRx_byte_buffer[3];
						messages_rx_buffer.length=(unsigned char)message_TxRx_byte_buffer[4];
					}
					// Clamp length to buffer size — corrupted frames (e.g., from
					// noise) can have garbage length values that overflow the buffer.
					int max_short_len = max_data_length + max_header_length - eff_hdr;
					if(max_short_len < 0) max_short_len = 0;
					if(max_short_len > alloc_size) max_short_len = alloc_size;
					if(messages_rx_buffer.length > max_short_len)
						messages_rx_buffer.length = max_short_len;
					for(int j=0;j<messages_rx_buffer.length;j++)
					{
						messages_rx_buffer.data[j]=message_TxRx_byte_buffer[j+eff_hdr];
					}
					if(sack_v2_enabled)
					{
						printf("[RX-BATCH-SEQ] type=DATA_SHORT id=%d seq=%d batch_seq_id=%d (v2)\n",
							(unsigned char)messages_rx_buffer.id,
							(unsigned char)messages_rx_buffer.sequence_number & 0x7F,
							messages_rx_buffer.batch_seq_id);
						fflush(stdout);
					}

				}

				last_message_received_type=messages_rx_buffer.type;
				if(messages_rx_buffer.type==CONTROL || messages_rx_buffer.type==ACK_CONTROL)
				{
					last_message_received_code=messages_rx_buffer.data[0];
				}
			}
		}
		else
		{
			// MFSK frame completeness: if the frame extends beyond captured audio,
			// capture the remaining symbols instead of wasting a full recapture cycle.
			// The metric threshold in time_sync_mfsk already filters false preambles,
			// so any detected preamble with overflow is worth recapturing.
			// For NB ROBUST_0, frame (537) = capture window, so ALL real preambles
			// in the new-data half of the buffer have large overflow — must allow it.
			int frame_symb = telecom_system->data_container.preamble_nSymb +
			                 telecom_system->data_container.Nsymb;
			if(received_message_stats.frame_overflow_symbols > 0 &&
			   received_message_stats.frame_overflow_symbols < frame_symb)
			{
				int shift_symbols = received_message_stats.frame_overflow_symbols + 4;

				// Read nUnder BEFORE resetting — these shifts already happened to the
				// live buffer during processing and must be included in the total shift
				// when adjusting search cursor for the recaptured buffer.
				int nUnder_current = telecom_system->data_container.nUnder_processing_events.load();
				int total_shift = shift_symbols + nUnder_current;

				telecom_system->data_container.frames_to_read = shift_symbols;
				telecom_system->data_container.nUnder_processing_events = 0;

				// Adjust anti-re-decode cursor for buffer shift. Separate counters
				// for MFSK and OFDM — update whichever is active.
				int adjusted_search;
				if(telecom_system->M == MOD_MFSK)
				{
					adjusted_search = telecom_system->receive_stats.mfsk_search_raw - total_shift;
					if(adjusted_search < 0) adjusted_search = 0;
					telecom_system->receive_stats.mfsk_search_raw = adjusted_search;
				}
				else
				{
					adjusted_search = telecom_system->receive_stats.ofdm_search_raw - total_shift;
					if(adjusted_search < 0) adjusted_search = 0;
					telecom_system->receive_stats.ofdm_search_raw = adjusted_search;
				}

				if (g_verbose)
					printf("[RX-TIMING] INCOMPLETE: overflow=%d symbols, capturing %d more, nUnder=%d search_raw=%d mod=%d\n",
						received_message_stats.frame_overflow_symbols,
						shift_symbols, nUnder_current, adjusted_search,
						(int)telecom_system->M);
				fflush(stdout);
				return;
			}

			if (g_verbose)
				printf("[RX-TIMING] FAIL: nUnder=%d proc=%.0fms search_raw=%d delay_last=%d mod=%.0f\n",
					telecom_system->data_container.nUnder_processing_events.load(), proc_ms,
					telecom_system->receive_stats.mfsk_search_raw,
					telecom_system->receive_stats.delay_of_last_decoded_message,
					telecom_system->M);
			fflush(stdout);

			// BREAK pattern detection: after failed decode, check for emergency
			// "drop to ROBUST_0" signal from commander during turboshift.
			// Only active when: gearshift enabled + responder role.
			// Commander never needs to detect BREAK (it sends BREAK, not receives).
			// Without gearshift, BREAK has no purpose — disable to avoid false positives
			// (matched=8/16 threshold too easy to hit on random MFSK data).
			if(break_detected == NO && gear_shift_on && role == RESPONDER)
			{
				int matched = 0;
				double metric = telecom_system->detect_break_pattern_from_passband(
					telecom_system->data_container.ready_to_process_passband_delayed_data,
					signal_period, &matched);
				// Require break_match_threshold (WB:12/16, NB M=8:24/32, NB M=4:40/48)
				if(metric >= telecom_system->ack_pattern_detection_threshold
				   && matched >= telecom_system->ack_mfsk.break_match_threshold)
				{
					printf("[BREAK] Emergency pattern detected! metric=%.2f matched=%d/%d\n",
						metric, matched, telecom_system->ack_mfsk.ack_pattern_nsymb);
					fflush(stdout);
#ifdef MERCURY_GUI_ENABLED
					gui_push_monitor_event("[BREAK]", false);
#endif
					break_detected = YES;
				}
			}

			// HAIL detection: "I am Mercury" beacon from commander.
			// Active in LISTENING/CONNECTION_RECEIVED state (responder waiting for contact).
			if(hail_detected == NO && role == RESPONDER &&
				(link_status == LISTENING || link_status == CONNECTION_RECEIVED))
			{
				int matched = 0;
				double metric = telecom_system->detect_hail_pattern_from_passband(
					telecom_system->data_container.ready_to_process_passband_delayed_data,
					signal_period, &matched);
				double hail_quality = (matched > 0) ? metric / matched : 0.0;
				if(metric >= telecom_system->ack_pattern_detection_threshold
				   && matched >= telecom_system->ack_mfsk.hail_match_threshold
				   && hail_quality >= 0.3)
				{
					printf("[HAIL] 'I am Mercury' beacon detected! metric=%.2f matched=%d/%d quality=%.2f\n",
						metric, matched, telecom_system->ack_mfsk.ack_pattern_nsymb, hail_quality);
					fflush(stdout);
					hail_detected = YES;
				}
			}

			// MFSK FAIL anti-spin: without this, frames_to_read stays at 0
			// after FAIL (no overflow), causing a tight spin loop where each
			// iteration (~100-200ms) accumulates nUnder_processing_events.
			// Large nUnder skews the MFSK preamble search start position.
			// Small shift lets the buffer accumulate fresh audio.
			if(telecom_system->M == MOD_MFSK && telecom_system->data_container.frames_to_read == 0)
			{
				int mfsk_ftr = telecom_system->data_container.preamble_nSymb * 2;
				if(mfsk_ftr < 16) mfsk_ftr = 16;
				telecom_system->data_container.frames_to_read = mfsk_ftr;
				telecom_system->data_container.nUnder_processing_events = 0;
				telecom_system->receive_stats.mfsk_search_raw = 0;
			}

			// Prevent OFDM FAIL spin loop: pause to let the buffer accumulate
			// fresh audio instead of burning CPU on doomed LDPC decodes.
			// Smart fast-forward (Bug #33): after a config transition, the
			// buffer is half-empty and the preamble lands beyond upper_bound
			// (frame doesn't fit).  Instead of the default 8-symbol shift
			// (~181ms, needing ~17 iterations), calculate the exact overflow
			// and shift by that amount in one go.
			if(telecom_system->M != MOD_MFSK && telecom_system->data_container.frames_to_read == 0)
			{
				// Default anti-spin: NB frames are 5x longer (nc_scale=5),
				// so use 2x larger ftr to scroll past false detections faster.
				int ftr = (telecom_system->ofdm.Nc <= 10) ? 16 : 8;

				if(received_message_stats.delay > 0)
				{
					int sym_period = telecom_system->data_container.Nofdm
						* telecom_system->data_container.interpolation_rate;
					int pream_symb = received_message_stats.delay / sym_period;
					int frame_symb = telecom_system->data_container.Nsymb
						+ telecom_system->data_container.preamble_nSymb;
					int upper = telecom_system->data_container.buffer_Nsymb - frame_symb;

					if(received_message_stats.frame_data_missing)
					{
						// Preamble detected but data symbols are silence.
						// Zero the stale preamble in the ring to prevent
						// re-detection (shift_left would have slid it off).
						{
							int sp = signal_period;
							int pream_samples = telecom_system->data_container.preamble_nSymb * symbol_period;
							int pream_ring_start = (rwi + received_message_stats.delay) % sp;

							MUTEX_LOCK(&capture_prep_mutex);
							for(int k = 0; k < pream_samples; k++)
							{
								int pos = (pream_ring_start + k) % sp;
								telecom_system->data_container.passband_delayed_data[pos] = 0.0;
								telecom_system->data_container.passband_delayed_data[pos + sp] = 0.0;
							}
							MUTEX_UNLOCK(&capture_prep_mutex);
						}
						// Quick retry — 8 symbols (~49ms WB) is enough to skip past
						// the failed position without waiting a full frame.
						ftr = 8;
						telecom_system->receive_stats.ofdm_search_raw = 0;
						telecom_system->receive_stats.ofdm_batch_active = false;
						printf("[FTR-INCOMPLETE] pream=%d ftr=%d metric=%.3f\n",
							pream_symb, ftr,
							telecom_system->receive_stats.coarse_metric);
						fflush(stdout);
					}
					else if(pream_symb > upper)
					{
						if(telecom_system->receive_stats.coarse_metric >= 0.5)
						{
							// Beyond-bounds with meaningful metric: fast-forward
							// to bring the preamble within bounds in one shift.
							// +20 margin ensures the preamble lands well within
							// bounds even with timing jitter.
							ftr = pream_symb - upper + 20;
						}
						else
						{
							// Low-metric beyond-bounds: false detection in decoded
							// frame's data region (OFDM symbols create Schmidl-Cox
							// peaks at metric 0.08-0.11). Reset search_raw to 0 to
							// break the cascade — without this, search_raw -= ftr
							// below walks backwards ~8 syms/iter causing 20+ FAILs.
							// Fresh search from prescan gives only 1-2 FAILs.
							ftr = 8;
							telecom_system->receive_stats.ofdm_search_raw = 0;
							telecom_system->receive_stats.ofdm_batch_active = false;
						}
						if(telecom_system->receive_stats.ofdm_search_raw > 0)
						{
							// Track buffer shift: reduce search_raw by ftr so the
							// anti-re-decode skip stays aligned with frame positions.
							telecom_system->receive_stats.ofdm_search_raw -= ftr;
							if(telecom_system->receive_stats.ofdm_search_raw < 0)
							{
								telecom_system->receive_stats.ofdm_search_raw = 0;
								telecom_system->receive_stats.ofdm_batch_active = false;
							}
						}
						printf("[RX-TIMING] OFDM beyond-bounds: pream=%d upper=%d metric=%.3f shift=%d search_raw=%d\n",
							pream_symb, upper,
							telecom_system->receive_stats.coarse_metric,
							ftr,
							telecom_system->receive_stats.ofdm_search_raw);
						fflush(stdout);
					}
					else if(telecom_system->receive_stats.ofdm_search_raw > 0
						&& telecom_system->receive_stats.ofdm_batch_active)
					{
						if(telecom_system->receive_stats.coarse_metric < 0.5)
						{
							// Low-metric FAIL in batch → no more real frames.
							// Real preambles have metric ≥ 0.9; false GI peaks from
							// data symbols or silence give 0.16–0.24. Exit batch
							// but keep search_raw nonzero so next search skips past
							// decoded frames. nUnder decay naturally resets ofdm_skip
							// to 0 as the ring buffer rotates past old audio.
							// Resetting to 0 here causes false preamble detections
							// in the DATA region of already-decoded frames.
							telecom_system->receive_stats.ofdm_batch_active = false;
							ftr = 8;
						}
						else
						{
							// High-metric FAIL in batch (≥0.5) — likely a real
							// preamble that LDPC couldn't decode (rare on clean
							// cable, common at low SNR). Minimal shift to find
							// the next real preamble nearby.
							ftr = 2;
							telecom_system->receive_stats.ofdm_search_raw -= ftr;
							if(telecom_system->receive_stats.ofdm_search_raw < 0)
							{
								telecom_system->receive_stats.ofdm_search_raw = 0;
								telecom_system->receive_stats.ofdm_batch_active = false;
							}
						}
					}
				else if(telecom_system->receive_stats.coarse_metric >= 0.15
					&& telecom_system->receive_stats.coarse_metric < 0.5)
					{
						// Medium-metric within-bounds FAIL (not in batch): likely
						// false preamble from NB OFDM data autocorrelation.
						// NB (Nc=10) has higher Schmidl-Cox metric variance than
						// WB (Nc=50), producing false peaks at 0.15-0.45.
						// These waste LDPC decode time and push real preambles
						// past the buffer boundary, causing NAcks.
						// Zero the false detection region to prevent re-detection,
						// then retry quickly with minimal shift.
						// Sub-threshold detections (metric < detection_threshold)
						// are caught by early return in receive_byte() but now
						// preserve delay at the detected position (not -1), so
						// they CAN reach this path for zeroing when metric >= 0.15.
						{
							int sp = signal_period;
							int pream_samples = telecom_system->data_container.preamble_nSymb * symbol_period;
							int pream_ring_start = (rwi + received_message_stats.delay) % sp;

							MUTEX_LOCK(&capture_prep_mutex);
							for(int k = 0; k < pream_samples; k++)
							{
								int pos = (pream_ring_start + k) % sp;
								telecom_system->data_container.passband_delayed_data[pos] = 0.0;
								telecom_system->data_container.passband_delayed_data[pos + sp] = 0.0;
							}
							MUTEX_UNLOCK(&capture_prep_mutex);
						}
						ftr = 2;
						telecom_system->receive_stats.ofdm_search_raw = 0;
						telecom_system->receive_stats.ofdm_batch_active = false;
						printf("[FTR-FALSE] pream=%d metric=%.3f — zeroed, retry\n",
							pream_symb,
							telecom_system->receive_stats.coarse_metric);
						fflush(stdout);
					}
				// Default ftr (8 WB, 16 NB) applies for other cases
				// (no preamble, high metric without batch mode, etc.)
				}

				if(telecom_system->receive_stats.ofdm_search_raw > 0)
				{
					// Preserve effective search position: ofdm_search_raw - nUnder.
					// Resetting nUnder to 0 without adjusting search_raw would make
					// the effective position jump forward by nUnder symbols, skipping
					// valid preambles.
					int nUnder_snap = telecom_system->data_container.nUnder_processing_events.load();
					telecom_system->receive_stats.ofdm_search_raw -= nUnder_snap;
					if(telecom_system->receive_stats.ofdm_search_raw < 0)
					{
						telecom_system->receive_stats.ofdm_search_raw = 0;
						telecom_system->receive_stats.ofdm_batch_active = false;
					}
				}
				// Monitor mode: when parallel decoders are active, they handle
				// all 17 configs simultaneously — no sequential scan needed.
				// Only use opportunistic scan as fallback if decoders aren't ready.
				if(passive_monitor && !monitor_decoders_ready)
				{
					int frame_symb = telecom_system->data_container.preamble_nSymb
						+ telecom_system->data_container.Nsymb;
					int min_ftr = frame_symb * 2;
					if(ftr < min_ftr) ftr = min_ftr;

					monitor_consec_ofdm_fail++;
					if(monitor_consec_ofdm_fail >= 3 && is_ofdm_config(current_configuration))
					{
						int cur = current_configuration;
						int next = (cur + 1) % NUMBER_OF_CONFIGS;
						int cur_mod = modulation_for_ofdm_config(cur);
						int next_mod = modulation_for_ofdm_config(next);
						bool same_mod = (cur_mod == next_mod);

						printf("[MONITOR] Opportunistic: CONFIG_%d fail #%d → CONFIG_%d (%s)\n",
							cur, monitor_consec_ofdm_fail, next,
							same_mod ? "same-mod, instant" : "cross-mod, wait");
						fflush(stdout);

						data_configuration = next;
						forward_configuration = next;
						load_configuration(next, PHYSICAL_LAYER_ONLY, YES);

						if(same_mod)
						{
							telecom_system->receive_stats.ofdm_search_raw = 0;
							telecom_system->receive_stats.ofdm_batch_active = false;
							ftr = 0;
						}
						else
						{
							frame_symb = telecom_system->data_container.preamble_nSymb
								+ telecom_system->data_container.Nsymb;
							ftr = frame_symb * 2;
						}
					}
				}
				telecom_system->data_container.frames_to_read = ftr;
				telecom_system->data_container.nUnder_processing_events = 0;
				// === DIAG: OFDM anti-spin ftr trace ===
				// Suppress during rapid same-mod opportunistic scan (ftr==0)
				if(ftr > 0)
				{
					printf("[FTR-FAIL] CONFIG_%d ftr=%d delay=%d metric=%.3f batch=%d search_raw=%d consec_fail=%d\n",
						current_configuration, ftr,
						received_message_stats.delay,
						telecom_system->receive_stats.coarse_metric,
						telecom_system->receive_stats.ofdm_batch_active ? 1 : 0,
						telecom_system->receive_stats.ofdm_search_raw,
						passive_monitor ? monitor_consec_ofdm_fail : -1);
					fflush(stdout);
				}
			}

			// MFSK anti-spin (Bug #37): without this, MFSK NO-PREAMBLE leaves
			// frames_to_read=0 → capture thread never shifts the buffer →
			// receive_byte reprocesses the same content indefinitely.
			//
			// Signal-aware strategy (Bug #43):
			// - NO-PREAMBLE (delay==-1): buffer has noise/silence. Use a small
			//   shift (8 symbols ~181ms) to poll quickly for an incoming frame.
			//   This avoids a 12s penalty when the buffer simply has no signal.
			// - LDPC-FAIL (delay>=0): structured MFSK tones present but decode
			//   failed. Full-frame shift to skip past the stale/bad frame.
			//   Set mfsk_search_raw to search only the fresh region, preventing
			//   partial decode of incomplete frames (Bug #42).
			if(telecom_system->M == MOD_MFSK && telecom_system->data_container.frames_to_read == 0)
			{
				if(received_message_stats.delay >= 0)
				{
					// Structured signal present — full-frame shift
					int ftr = telecom_system->data_container.preamble_nSymb
						+ telecom_system->data_container.Nsymb;
					telecom_system->data_container.frames_to_read = ftr;
					telecom_system->data_container.nUnder_processing_events = 0;
					int buf_nsymb = telecom_system->data_container.buffer_Nsymb.load();
					telecom_system->receive_stats.mfsk_search_raw = buf_nsymb - ftr;
					if(telecom_system->receive_stats.mfsk_search_raw < 0)
						telecom_system->receive_stats.mfsk_search_raw = 0;
				}
				else
				{
					// No signal — small shift, poll again quickly.
					// Preserve effective mfsk_search_raw position.
					int nUnder_snap = telecom_system->data_container.nUnder_processing_events.load();
					telecom_system->receive_stats.mfsk_search_raw -= nUnder_snap;
					if(telecom_system->receive_stats.mfsk_search_raw < 0)
						telecom_system->receive_stats.mfsk_search_raw = 0;
					telecom_system->data_container.frames_to_read = 8;
					telecom_system->data_container.nUnder_processing_events = 0;
				}
			}

			// OFDM anti-spin: on FAIL with structured signal (preamble found but
			// LDPC failed), skip past the detected preamble to avoid re-finding it.
			// Same pattern as MFSK anti-spin above.
			if(telecom_system->M != MOD_MFSK
				&& telecom_system->data_container.frames_to_read == 0
				&& received_message_stats.delay >= 0)
			{
				int rx_frame = telecom_system->get_active_nsymb()
					+ telecom_system->data_container.preamble_nSymb;
				telecom_system->data_container.frames_to_read = rx_frame;
				telecom_system->data_container.nUnder_processing_events = 0;
				int buf_nsymb = telecom_system->data_container.buffer_Nsymb.load();
				telecom_system->receive_stats.ofdm_search_raw = buf_nsymb - rx_frame;
				if(telecom_system->receive_stats.ofdm_search_raw < 0)
				{
					telecom_system->receive_stats.ofdm_search_raw = 0;
					telecom_system->receive_stats.ofdm_batch_active = false;
				}
			}
			else if(telecom_system->M != MOD_MFSK && received_message_stats.delay >= 0)
			{
				// OFDM anti-spin SKIP: ftr != 0, so anti-spin doesn't apply.
				// Frame will be shifted out naturally via ftr countdown.
			}

			if(telecom_system->data_container.frames_to_read==0 && telecom_system->receive_stats.delay_of_last_decoded_message!=-1)
			{
				telecom_system->receive_stats.delay_of_last_decoded_message -= telecom_system->data_container.Nofdm*telecom_system->data_container.interpolation_rate;
				if(telecom_system->receive_stats.delay_of_last_decoded_message < 0)
				{
					telecom_system->receive_stats.delay_of_last_decoded_message = -1;
				}
			}
		}
		// Return here - we already unlocked the mutex at line 1929 and cleared data_ready
		return;
	}

	// frames_to_read != 0: just clear data_ready and unlock
	telecom_system->data_container.data_ready = 0;
	MUTEX_UNLOCK(&capture_prep_mutex);
}


void cl_arq_controller::copy_data_to_buffer()
{
	int copied = 0;
	int total_bytes = 0;

	if(compression_enabled)
	{
		// --- Batch-level decompression ---
		// Reassemble ACKED frames from current batch into one contiguous buffer,
		// then decompress as a single block.
		// IMPORTANT: Only iterate data_batch_size slots (not nMessages) to avoid
		// including stale ACKED data from previous batches in higher-numbered slots.
		char assembled[16384];
		int assembled_size = 0;

		for(int i=0;i<this->data_batch_size;i++)
		{
			if(messages_rx[i].status==ACKED)
			{
				if(assembled_size + messages_rx[i].length <= (int)sizeof(assembled))
				{
					memcpy(assembled + assembled_size,
						messages_rx[i].data, messages_rx[i].length);
					assembled_size += messages_rx[i].length;
				}
				messages_rx[i].status=FREE;
				copied++;
			}
			else
			{
				messages_rx[i].status=FREE;
			}
		}
		// Clear any stale slots beyond batch boundary
		for(int i=this->data_batch_size;i<this->nMessages;i++)
			messages_rx[i].status=FREE;

		if(assembled_size >= compressor.get_header_size())
		{
			// --- Decrypt batch (after reassembly, before decompression) ---
			char* comp_data = assembled;
			int comp_len = assembled_size;
			char decrypt_buf[16384];

			if(cipher_suite.is_active() && assembled_size > 0)
			{
				// Always use full 16-byte auth tag (matches TX side)
				int tag_size = AUTH_TAG_SIZE;
				uint32_t rx_direction = (original_role == COMMANDER)
					? DIRECTION_RSP_TO_CMD : DIRECTION_CMD_TO_RSP;
				printf("[CRYPTO-RX] Decrypting %d bytes, counter=%llu dir=%u tag=%d config=%d\n",
					assembled_size, (unsigned long long)rx_batch_counter,
					rx_direction, tag_size, (int)data_configuration);
				fflush(stdout);
				int plain_size = cipher_suite.decrypt(
					(const uint8_t*)assembled, assembled_size,
					(uint8_t*)decrypt_buf, sizeof(decrypt_buf),
					rx_batch_counter, rx_direction,
					tag_size);
				if(plain_size > 0)
				{
					comp_data = decrypt_buf;
					comp_len = plain_size;
					rx_batch_counter++;
					printf("[CRYPTO-RX] Decrypted: %d -> %d bytes OK\n",
						assembled_size, plain_size);
					fflush(stdout);
				}
				else
				{
					consecutive_auth_failures++;
					printf("[CRYPTO] Decrypt FAILED (batch %llu, fails=%d) — PSK MISMATCH\n",
						(unsigned long long)rx_batch_counter, consecutive_auth_failures);
					fflush(stdout);

					// Report error on control port
					const char* err_msg = "ENCRYPTION FAILURE\r";
					int elen = (int)strlen(err_msg);
					for(int e=0; e<elen; e++)
						tcp_socket_control.message->buffer[e] = err_msg[e];
					tcp_socket_control.message->length = elen;
					tcp_socket_control.transmit();

#ifdef MERCURY_GUI_ENABLED
					g_gui_state.encryption_psk_mismatch.store(true);
					gui_push_monitor_event("[PSK MISMATCH — authentication failed, disconnecting]", false);
#endif
					// Disconnect immediately — mismatched PSK is unrecoverable
					printf("[CRYPTO] Authentication failure — disconnecting\n");
					fflush(stdout);
					this->link_status = DROPPED;
					reset_session_state();
					goto copy_data_done;
				}
			}

			char decomp_buf[COMPRESS_WORKSPACE_SIZE];
			int dec_size = compressor.decompress_block(
				comp_data, comp_len,
				decomp_buf, (int)sizeof(decomp_buf));
			if(dec_size > 0)
			{
#ifdef MERCURY_GUI_ENABLED
				// Monitor tap: plaintext after decompression
				gui_push_monitor_text(decomp_buf, dec_size, false);
#endif
				// Headless monitor: output plaintext to stdout
				if(monitor_stdout)
				{
					fwrite(decomp_buf, 1, dec_size, stdout);
					fflush(stdout);
				}
				fifo_buffer_rx.push(decomp_buf, dec_size);
				total_bytes += dec_size;
				// Streaming: commit context (raw data = decompressed output)
				if(compressor.is_streaming())
					compressor.streaming_commit((unsigned char*)decomp_buf, dec_size);
				// Reset auth failure counter on success
				if(cipher_suite.is_active())
					consecutive_auth_failures = 0;
				// Update compression ratio on responder side (EMA)
				int comp_payload = comp_len - compressor.get_header_size();
				if(comp_payload > 0)
				{
					float measured = (float)dec_size / (float)comp_payload;
					compress_ratio_estimate = 0.7f * compress_ratio_estimate + 0.3f * measured;
				}
#ifdef MERCURY_GUI_ENABLED
				// Push algo to GUI from decompressed header (responder side)
				g_gui_state.compression_algo.store((int)(unsigned char)comp_data[0]);
#endif
			}
			else
			{
				// Decompression error — reset streaming and push raw as fallback
				if(compressor.is_streaming())
					compressor.streaming_reset();
				const unsigned char* ehdr = (const unsigned char*)comp_data;
				printf("[DECOMPRESS] Batch error (assembled %d bytes, hdr: algo=%d comp=%d orig=%d), pushing raw\n",
					comp_len, (int)ehdr[0],
					(int)(ehdr[1] | (ehdr[2] << 8)),
					(int)(ehdr[3] | (ehdr[4] << 8)));
				fflush(stdout);
				fifo_buffer_rx.push(comp_data, comp_len);
				total_bytes += comp_len;
			}
		}
		else if(assembled_size > 0)
		{
			// Too small for compression header — push raw
#ifdef MERCURY_GUI_ENABLED
			gui_push_monitor_text(assembled, assembled_size, false);
#endif
			if(monitor_stdout)
			{
				fwrite(assembled, 1, assembled_size, stdout);
				fflush(stdout);
			}
			fifo_buffer_rx.push(assembled, assembled_size);
			total_bytes += assembled_size;
		}
	}
	else
	{
		// --- No compression: original per-message push ---
		// Only iterate data_batch_size slots for current batch
		for(int i=0;i<this->data_batch_size;i++)
		{
			if(messages_rx[i].status==ACKED)
			{
#ifdef MERCURY_GUI_ENABLED
				gui_push_monitor_text(messages_rx[i].data, messages_rx[i].length, false);
#endif
				if(monitor_stdout)
				{
					fwrite(messages_rx[i].data, 1, messages_rx[i].length, stdout);
					fflush(stdout);
				}
				fifo_buffer_rx.push(messages_rx[i].data, messages_rx[i].length);
				total_bytes += messages_rx[i].length;
				messages_rx[i].status=FREE;
				copied++;
			}
			else if(messages_rx[i].status!=FREE)
			{
				messages_rx[i].status=FREE;
			}
		}
		// Clear stale slots beyond batch boundary
		for(int i=this->data_batch_size;i<this->nMessages;i++)
		{
			if(messages_rx[i].status!=FREE)
				messages_rx[i].status=FREE;
		}
	}
copy_data_done:
#ifdef MERCURY_GUI_ENABLED
	if(total_bytes > 0)
		gui_add_throughput_bytes_rx(total_bytes);
#endif
	block_ready=1;
}

void cl_arq_controller::restore_tx_from_compressed()
{
	// When streaming is active, messages_tx was compressed with streaming context
	// that has already advanced the PPMd model. We can't decompress it again.
	// Use the backup buffer (raw plaintext) and reset streaming context.
	if(compressor.is_streaming())
	{
		printf("[RESTORE_TX] Streaming active — resetting context, restoring from backup\n");
		fflush(stdout);
		compressor.streaming_reset();
		compressor.clear_pending();
		for(int i=0; i<nMessages; i++)
			messages_tx[i].status = FREE;
		restore_backup_buffer_data();
		return;
	}

	// When encryption is active, messages_tx contains encrypted+compressed data.
	// Decrypting requires the exact batch counter that was used, which is fragile.
	// The backup buffer always has raw plaintext, so use it directly.
	if(cipher_suite.is_active())
	{
		printf("[RESTORE_TX] Encryption active — restoring from backup buffer\n");
		fflush(stdout);
		for(int i=0; i<nMessages; i++)
			messages_tx[i].status = FREE;
		restore_backup_buffer_data();
		// Rewind TX counter so re-encrypted batch gets same counter
		if(tx_batch_counter > 0) tx_batch_counter--;
		return;
	}

	// Reassemble compressed chunks from messages_tx, decompress back to raw,
	// push raw data to fifo_buffer_tx for re-compression at new config.
	char assembled[16384];
	int assembled_size = 0;

	for(int i=0; i<nMessages; i++)
	{
		if(messages_tx[i].status != FREE && messages_tx[i].length > 0)
		{
			if(assembled_size + messages_tx[i].length <= (int)sizeof(assembled))
			{
				memcpy(assembled + assembled_size,
					messages_tx[i].data, messages_tx[i].length);
				assembled_size += messages_tx[i].length;
			}
		}
		messages_tx[i].status = FREE;
	}

	if(assembled_size >= COMPRESS_HEADER_SIZE_LEGACY)
	{
		char decomp_buf[COMPRESS_WORKSPACE_SIZE];
		int dec_size = compressor.decompress_block(
			assembled, assembled_size,
			decomp_buf, (int)sizeof(decomp_buf));
		if(dec_size > 0)
		{
			fifo_buffer_tx.push_front(decomp_buf, dec_size);
			printf("[RESTORE_TX] Decompressed %d -> %d bytes, pushed to FIFO\n",
				assembled_size, dec_size);
		}
		else
		{
			// Decompression failed — fall back to backup buffer
			printf("[RESTORE_TX] Decompress failed (%d bytes), restoring from backup\n",
				assembled_size);
			restore_backup_buffer_data();
			return;  // backup restore already handles fifo_buffer_backup
		}
	}
	else if(assembled_size > 0)
	{
		// Too small for header — push raw assembled data
		fifo_buffer_tx.push_front(assembled, assembled_size);
		printf("[RESTORE_TX] No header (%d bytes), pushed raw\n", assembled_size);
	}

	// Flush backup — no longer needed, we recovered from messages_tx
	fifo_buffer_backup.flush();
	fflush(stdout);
}

void cl_arq_controller::restore_backup_buffer_data()
{
	// SACK Design A Step 1 — effective DATA_LONG header length gates per-frame
	// payload capacity. In v1 (default) this is the legacy 4-byte value; in v2
	// it grows by 1 byte (matching the wire format the prior batch was sent in).
	int eff_long_hdr = effective_data_long_header_length(sack_v2_enabled);
	int nBackedup_bytes, data_read_size, nMessages;
	nBackedup_bytes=fifo_buffer_backup.get_size()-fifo_buffer_backup.get_free_size();
	if(nBackedup_bytes!=0 && (max_data_length+max_header_length-eff_long_hdr)!=0)
	{
		nMessages=nBackedup_bytes/(max_data_length+max_header_length-eff_long_hdr);

		char restore_buf[N_MAX/8 * 20];
		int total_restore = 0;
		for(int i=0;i<nMessages+1;i++)
		{
			data_read_size=fifo_buffer_backup.pop(restore_buf + total_restore,max_data_length+max_header_length-eff_long_hdr);
			if(data_read_size > 0)
				total_restore += data_read_size;
		}
		if(total_restore > 0)
			fifo_buffer_tx.push_front(restore_buf, total_restore);
	}
}

void cl_arq_controller::print_stats()
{
	printf("\033[2J");  // clean screen
	printf("\033[H");   // go to upper left corner

	if(this->current_configuration!=CONFIG_NONE)
	{
		printf("configuration:CONFIG_%d (%.1f bps)\n", (int)this->current_configuration, telecom_system->rbc);
	}
	else
	{
		printf("configuration: ERROR..( 0 bps)\n");
	}

	// Display audio devices
	extern char *input_dev;
	extern char *output_dev;
	printf("Audio_IN: %s\n", (input_dev ? input_dev : "default"));
	printf("Audio_OUT: %s\n", (output_dev ? output_dev : "default"));

	printf("\n");

	if(this->role==COMMANDER)
	{
		printf("Role:COM call sign= %s\n", this->my_call_sign.c_str());
	}
	else if (this->role==RESPONDER)
	{
		printf("Role:Res call sign= %s\n", this->my_call_sign.c_str());
	}

	if(this->link_status==DROPPED)
	{
		printf("link_status:Dropped\n");
#ifdef MERCURY_GUI_ENABLED
		gui_push_monitor_event("[DISCONNECTED]", true);
#endif
	}
	else if(this->link_status==IDLE)
	{
		printf("link_status:Idle\n");
	}
	else if (this->link_status==CONNECTING)
	{
		printf("link_status:Connecting to %s\n", this->destination_call_sign.c_str());
	}
	else if (this->link_status==CONNECTED)
	{
		printf("link_status:Connected to %s ID= %d\n", this->destination_call_sign.c_str(), (int)this->connection_id);
	}
	else if (this->link_status==DISCONNECTING)
	{
		printf("link_status:Disconnecting\n");
	}
	else if (this->link_status==LISTENING)
	{
		printf("link_status:Listening\n");
	}
	else if (this->link_status==CONNECTION_RECEIVED)
	{
		printf("link_status:Connection Received from %s\n", this->destination_call_sign.c_str());
	}
	else if (this->link_status==CONNECTION_ACCEPTED)
	{
		printf("link_status:Connection Accepted by %s\n", this->destination_call_sign.c_str());
	}
	else if (link_status==NEGOTIATING)
	{
		printf("link_status:Negotiating with %s\n", this->destination_call_sign.c_str());
	}

	if (this->connection_status==TRANSMITTING_DATA)
	{
		printf("connection_status:Transmitting data\n");
	}
	else if (this->connection_status==RECEIVING)
	{
		printf("connection_status:Receiving\n");
	}
	else if (this->connection_status==RECEIVING_ACKS_DATA)
	{
		printf("connection_status:Receiving data Ack\n");
	}
	else if(this->connection_status==ACKNOWLEDGING_DATA)
	{
		printf("connection_status:Acknowledging data\n");
	}
	else if (this->connection_status==TRANSMITTING_CONTROL)
	{
		printf("connection_status:Transmitting control\n");
	}
	else if (this->connection_status==RECEIVING_ACKS_CONTROL)
	{
		printf("connection_status:Receiving control Ack\n");
	}
	else if (this->connection_status==ACKNOWLEDGING_CONTROL)
	{
		printf("connection_status:Acknowledging control\n");
	}
	else if(this->connection_status==IDLE)
	{
		printf("connection_status:Idle\n");
	}

	printf("measurements.SNR_uplink= %.2f\n", measurements.SNR_uplink);
	printf("measurements.SNR_downlink= %.2f\n", measurements.SNR_downlink);
	printf("measurements.signal_stregth_dbm= %.2f\n", measurements.signal_stregth_dbm);
	printf("measurements.frequency_offset= %.2f\n", measurements.frequency_offset);

	printf("\n");

	printf("stats.nSent_data= %d\n", stats.nSent_data);
	printf("stats.nAcked_data= %d\n", stats.nAcked_data);
	printf("stats.nReceived_data= %d\n", stats.nReceived_data);
	printf("stats.nLost_data= %d\n", stats.nLost_data);
	printf("stats.nReSent_data= %d\n", stats.nReSent_data);
	printf("stats.nAcks_sent_data= %d\n", stats.nAcks_sent_data);
	printf("stats.nNAcked_data= %d\n", stats.nNAcked_data);
	printf("stats.ToSend_data:%d\n", this->get_nToSend_messages());

	printf("\n");

	printf("stats.nSent_control= %d\n", stats.nSent_control);
	printf("stats.nAcked_control= %d\n", stats.nAcked_control);
	printf("stats.nReceived_control= %d\n", stats.nReceived_control);
	printf("stats.nLost_control= %d\n", stats.nLost_control);
	printf("stats.nReSent_control= %d\n", stats.nReSent_control);
	printf("stats.nAcks_sent_control= %d\n", stats.nAcks_sent_control);
	printf("stats.nNAcked_control= %d\n", stats.nNAcked_control);

	printf("\n");
	printf("link_timer= %d\n", link_timer.get_elapsed_time_ms());
	printf("watchdog_timer= %d\n", watchdog_timer.get_elapsed_time_ms());
	printf("gear_shift_timer= %d\n", gear_shift_timer.get_elapsed_time_ms());
	printf("receiving_timer= %d\n", receiving_timer.get_elapsed_time_ms());

	printf("\n");
	printf("last_received_message_sequence= %d\n", (int)last_received_message_sequence);

	printf("last_transmission_block_success_rate= %d %%\n", (int)last_transmission_block_stats.success_rate_data);
	if(gear_shift_blocked_for_nBlocks<gear_shift_block_for_nBlocks_total)
	{
		printf("gear_shift_blocked_for_nBlocks= %d\n", (int)gear_shift_blocked_for_nBlocks);
	}
	else
	{
		printf("gear_shift_blocked_for_nBlocks=\n");
	}

	printf("\n");

	const char* msg_sent_str = "";
	if (this->last_message_sent_type==NONE)
	{
		msg_sent_str = "last_message_sent:";
	}
	else if (this->last_message_sent_type==DATA_LONG)
	{
		msg_sent_str = "last_message_sent:DATA:DATA_LONG";
	}
	else if (this->last_message_sent_type==DATA_SHORT)
	{
		msg_sent_str = "last_message_sent:DATA:DATA_SHORT";
	}
	else if (this->last_message_sent_type==ACK_MULTI)
	{
		msg_sent_str = "last_message_sent:DATA:ACK_MULTI";
	}
	else if (this->last_message_sent_type==ACK_RANGE)
	{
		msg_sent_str = "last_message_sent:DATA:ACK_RANGE";
	}
	else if (this->last_message_sent_type==CONTROL)
	{
		msg_sent_str = "last_message_sent:CONTROL:";
	}
	else if (this->last_message_sent_type==ACK_CONTROL)
	{
		msg_sent_str = "last_message_sent:ACK_CONTROL:";
	}

	const char* msg_sent_code_str = "";
	if(this->last_message_sent_type==CONTROL || this->last_message_sent_type==ACK_CONTROL)
	{
		if (this->last_message_sent_code==START_CONNECTION) msg_sent_code_str = "START_CONNECTION";
		else if (this->last_message_sent_code==TEST_CONNECTION) msg_sent_code_str = "TEST_CONNECTION";
		else if (this->last_message_sent_code==CLOSE_CONNECTION) msg_sent_code_str = "CLOSE_CONNECTION";
		else if (this->last_message_sent_code==KEEP_ALIVE) msg_sent_code_str = "KEEP_ALIVE";
		else if (this->last_message_sent_code==FILE_START) msg_sent_code_str = "FILE_START";
		else if (this->last_message_sent_code==FILE_END_) msg_sent_code_str = "FILE_END";
		else if (this->last_message_sent_code==PIPE_OPEN) msg_sent_code_str = "PIPE_OPEN";
		else if (this->last_message_sent_code==PIPE_CLOSE) msg_sent_code_str = "PIPE_CLOSE";
		else if (this->last_message_sent_code==SWITCH_ROLE) msg_sent_code_str = "SWITCH_ROLE";
		else if (this->last_message_sent_code==BLOCK_END) msg_sent_code_str = "BLOCK_END";
		else if (this->last_message_sent_code==SET_CONFIG) msg_sent_code_str = "SET_CONFIG";
		else if (this->last_message_sent_code==REPEAT_LAST_ACK) msg_sent_code_str = "REPEAT_LAST_ACK";
		else if (this->last_message_sent_code==SWITCH_BANDWIDTH) msg_sent_code_str = "SWITCH_BANDWIDTH";
	}
	printf("%s%s\n", msg_sent_str, msg_sent_code_str);

	const char* msg_recv_str = "";
	if (this->last_message_received_type==NONE)
	{
		msg_recv_str = "last_message_received:";
	}
	else if (this->last_message_received_type==DATA_LONG)
	{
		msg_recv_str = "last_message_received:DATA:DATA_LONG";
	}
	else if (this->last_message_received_type==DATA_SHORT)
	{
		msg_recv_str = "last_message_received:DATA:DATA_SHORT";
	}
	else if (this->last_message_received_type==ACK_MULTI)
	{
		msg_recv_str = "last_message_received:DATA:ACK_MULTI";
	}
	else if (this->last_message_received_type==ACK_RANGE)
	{
		msg_recv_str = "last_message_received:DATA:ACK_RANGE";
	}
	else if (this->last_message_received_type==CONTROL)
	{
		msg_recv_str = "last_message_received:CONTROL:";
	}
	else if (this->last_message_received_type==ACK_CONTROL)
	{
		msg_recv_str = "last_message_received:ACK_CONTROL:";
	}

	const char* msg_recv_code_str = "";
	if(this->last_message_received_type==CONTROL || this->last_message_received_type==ACK_CONTROL)
	{
		if (this->last_message_received_code==START_CONNECTION) msg_recv_code_str = "START_CONNECTION";
		else if (this->last_message_received_code==TEST_CONNECTION) msg_recv_code_str = "TEST_CONNECTION";
		else if (this->last_message_received_code==CLOSE_CONNECTION) msg_recv_code_str = "CLOSE_CONNECTION";
		else if (this->last_message_received_code==KEEP_ALIVE) msg_recv_code_str = "KEEP_ALIVE";
		else if (this->last_message_received_code==FILE_START) msg_recv_code_str = "FILE_START";
		else if (this->last_message_received_code==FILE_END_) msg_recv_code_str = "FILE_END";
		else if (this->last_message_received_code==PIPE_OPEN) msg_recv_code_str = "PIPE_OPEN";
		else if (this->last_message_received_code==PIPE_CLOSE) msg_recv_code_str = "PIPE_CLOSE";
		else if (this->last_message_received_code==SWITCH_ROLE) msg_recv_code_str = "SWITCH_ROLE";
		else if (this->last_message_received_code==BLOCK_END) msg_recv_code_str = "BLOCK_END";
		else if (this->last_message_received_code==SET_CONFIG) msg_recv_code_str = "SET_CONFIG";
		else if (this->last_message_received_code==REPEAT_LAST_ACK) msg_recv_code_str = "REPEAT_LAST_ACK";
		else if (this->last_message_received_code==SWITCH_BANDWIDTH) msg_recv_code_str = "SWITCH_BANDWIDTH";
	}
	printf("%s%s\n", msg_recv_str, msg_recv_code_str);

	printf("\n");
	printf("TX buffer occupancy= %.2f %%\n", (float)(fifo_buffer_tx.get_size()-fifo_buffer_tx.get_free_size())*100.0f/(float)fifo_buffer_tx.get_size());
	printf("RX buffer occupancy= %.2f %%\n", (float)(fifo_buffer_rx.get_size()-fifo_buffer_rx.get_free_size())*100.0f/(float)fifo_buffer_rx.get_size());
	printf("Backup buffer occupancy= %.2f %%\n", (float)(fifo_buffer_backup.get_size()-fifo_buffer_backup.get_free_size())*100.0f/(float)fifo_buffer_backup.get_size());
	fflush(stdout);
}

uint8_t cl_arq_controller::CRC8_calc(char* data_byte, int nItems)
{
	uint8_t crc = 0xff;
	for(int j=0; j < nItems; j++)
	{
		crc ^= data_byte[j];
		for (int i = 0; i < 8; i++)
		{
			if ((crc & 0x01) == 0x01)
			{
				crc = crc >> 1;
				crc ^= POLY_CRC8;
			}
			else
			{
				crc = crc >> 1;
			}
		}
	}
	return crc;
	//ref: MODBUS over serial line specification and implementation guide V1.02, Dec 20,2006, available at https://modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
}

