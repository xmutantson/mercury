// Recovery control-ACK capture exercise.
//
// A complete ROBUST_0 ACK is seated one ACK-window older than the newest tail,
// exactly as a late recovery reply appears after trailing silence has advanced
// the capture ring.  The production recovery poll is invoked without the
// caller's generic multi-window hint.  Bare monitor scans only the silent newest
// tail and returns false; the recovery-only re-phase port searches retained
// phases and accepts the same on-air ACK.

#include "datalink_layer/arq.h"
#include "datalink_layer/datalink_defines.h"
#include "physical_layer/telecom_system.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

int cl_arq_controller::test_recovery_ack_capture_exercise()
{
	cl_telecom_system* ts = new cl_telecom_system();
	cl_arq_controller* cmd = new cl_arq_controller();
	ts->operation_mode = ARQ_MODE;
	ts->narrowband_enabled = NO;
	cmd->telecom_system = ts;
	cmd->role = COMMANDER;
	cmd->narrowband_enabled = NO;
	cmd->current_configuration = CONFIG_NONE;
	cmd->load_configuration(ROBUST_0, FULL, NO);
	cmd->link_status = CONNECTED;
	cmd->connection_status = RECEIVING_ACKS_CONTROL;
	cmd->turbo_snr_ack_enabled = false;

	const int ack_nsymb = ts->ack_mfsk.ack_pattern_nsymb;
	const int ack_base_total = ts->ack_mfsk.ack_base_total_nsymb();
	const int pattern_len = ack_base_total;
	const int tail_nsymb = ack_base_total + pattern_len + 16;
	const int sym_samples = ts->data_container.Nofdm
	                      * ts->data_container.interpolation_rate;
	const int signal_period = sym_samples * ts->data_container.buffer_Nsymb.load();
	int tail_samples = tail_nsymb * sym_samples;
	if(tail_samples > signal_period) tail_samples = signal_period;
	const int tail_offset = signal_period - tail_samples;
	const int stride = pattern_len * sym_samples;
	const int older_offset = tail_offset - stride;
	const int ack_samples = ts->ack_pattern_passband_samples;

	int fails = 0;
	if(ack_nsymb != 16 || older_offset < 0 || ack_samples <= 0
	   || ack_samples > stride || older_offset + ack_samples > signal_period)
	{
		printf("[TEST-RECOV-CAP-EXERCISE] FAIL setup: ack_sym=%d ack_samples=%d "
		       "stride=%d older_off=%d signal_period=%d\n",
		       ack_nsymb, ack_samples, stride, older_offset, signal_period);
		delete cmd;
		delete ts;
		return 1;
	}

	std::vector<double> ack((size_t)ack_samples, 0.0);
	if(ts->generate_ack_pattern_passband(ack.data()) != ack_samples)
	{
		printf("[TEST-RECOV-CAP-EXERCISE] FAIL: ACK generation length mismatch\n");
		delete cmd;
		delete ts;
		return 1;
	}

	double* ring = ts->data_container.passband_delayed_data;
	memset(ring, 0, (size_t)(2 * signal_period) * sizeof(double));
	for(int i = 0; i < ack_samples; i++)
	{
		ring[older_offset + i] = ack[(size_t)i];
		ring[signal_period + older_offset + i] = ack[(size_t)i];
	}
	ts->data_container.ring_write_index = 0;
	ts->data_container.frames_to_read = 0;
	ts->data_container.data_ready = 1;

	setenv("MERCURY_RECOVERY_ACK_REPHASE", "1", 1);
	bool accepted = cmd->receive_ack_pattern(false, false);
	printf("[TEST-RECOV-CAP-EXERCISE] late recovery ACK accepted=%d "
	       "accepted_control_acks=%d/1 newest_tail=silent older_phase=%d\n",
	       accepted ? 1 : 0, accepted ? 1 : 0, older_offset);
	if(!accepted)
	{
		printf("[TEST-RECOV-CAP-EXERCISE] FAIL: recovery poll did not capture the "
		       "complete retained ACK (accepted_control_acks=0/1)\n");
		fails++;
	}

	// The additive scan must not manufacture an ACK from an empty history.
	memset(ring, 0, (size_t)(2 * signal_period) * sizeof(double));
	ts->data_container.frames_to_read = 0;
	ts->data_container.data_ready = 1;
	bool false_accept = cmd->receive_ack_pattern(false, false);
	printf("[TEST-RECOV-CAP-EXERCISE] silent history accepted=%d (want 0)\n",
	       false_accept ? 1 : 0);
	if(false_accept) fails++;

	unsetenv("MERCURY_RECOVERY_ACK_REPHASE");
	delete cmd;
	delete ts;
	printf("[TEST-RECOV-CAP-EXERCISE] %s\n", fails == 0 ? "PASS" : "FAIL");
	return fails == 0 ? 0 : 1;
}
