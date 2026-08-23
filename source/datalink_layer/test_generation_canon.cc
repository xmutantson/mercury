#include "datalink_layer/arq.h"

#include <cstdio>
#include <cstring>

// A2 decisive experiment: cumulative wire=N-1 carries a bitmap sourced from
// generation N while the commander's dispatch counter is already N+1.  Both N-1
// and N are retained so routing on the raw octet produces an observable wrong-
// generation requeue, not a benign miss.  This file was added and run against
// the unmodified production route before the canonicalization was implemented.
int cl_arq_controller::test_generation_canon()
{
	const int N = 7;
	const int NP1 = (N + 1) & 0xFF;
	const int wire = (N - 1) & 0xFF;
	const int span = 4;
	int fails = 0;
	int assertions = 0;
	auto check = [&](bool ok, const char* label)
	{
		assertions++;
		printf("[TEST-GEN-CANON] %s %s\n", ok ? "PASS" : "FAIL", label);
		if(!ok) fails++;
	};

	nMessages = 32;
	max_data_length = 170;
	max_header_length = 6;
	max_message_length = 200;
	deinit_messages_buffers();
	if(init_messages_buffers() != SUCCESSFUL)
	{
		printf("[TEST-GEN-CANON] FAIL buffer allocation\n");
		return 1;
	}
	sack_v2_enabled = true;
	cumulative_ack_enabled = true;
	data_batch_size = span;
	cmd_batch_seq_id = NP1;
	retransmit_count = 0;
	cmd_prev_retain_count = 0;

	unsigned char decoy[3] = {0x61, 0x62, 0x63};
	unsigned char wanted[3] = {0x71, 0x72, 0x73};
	cmd_prev_retain_capture(wire, 0, 3, DATA_LONG, 0, decoy);
	cmd_prev_retain_capture(N,    0, 3, DATA_LONG, 0, wanted);
	bool bitmap[span] = {false, true, true, true};

	// Both production transports feed the same resolve/validate gate after their
	// respective CRC checks.  The test labels both arms while exercising that
	// shared production decision.
	int route_target = -1;
	const char* transports[2] = {"MFSK", "OFDM"};
	for(const char* transport : transports)
	{
		st_resolved_generation_ack resolved = {};
		bool accepted = resolve_and_validate_generation_ack(
			(unsigned char)wire, span, /*integrity_ok=*/true,
			/*clean=*/false, /*bitmap_any=*/true, /*bitmap_all=*/false,
			&resolved);
		char label[160];
		snprintf(label, sizeof(label), "%s accepts owned partial after integrity", transport);
		check(accepted, label);
		snprintf(label, sizeof(label), "%s resolves wire N-1 to target N", transport);
		check(resolved.target == N && resolved.shadow_owned, label);
		route_target = resolved.target;
	}

	// Production must resolve once, before either R2c helper sees the key.
	bool routed = cmd_prev_resack_is_shadow_target(route_target);
	int rq = routed ? cmd_prev_retain_requeue(route_target, bitmap, span) : 0;
	int queued_bsi = rq > 0 ? retransmit_frame_batch_seq_ids[0] : -1;

	printf("[TEST-GEN-CANON] transport=synthetic cumulative=1 target=%d wire=%d "
	       "dispatch=%d route=%d rq=%d queued_bsi=%d\n",
		N, wire, NP1, route_target, rq, queued_bsi);
	check(wire == ((N - 1) & 0xFF), "wire is target-1");
	check(route_target == N, "resolve-before-route selects bitmap source generation N");
	check(routed && rq == 1, "exactly one retained missing slot is requeued");
	check(queued_bsi == N, "only generation N is queued; decoy N-1 is untouched");

	// Target zero has no sentinel exception: cumulative wire 255 resolves to 0.
	unsigned char wrap_payload[2] = {0x81, 0x82};
	cmd_prev_retain_capture(/*target=*/0, 0, 2, DATA_LONG, 0, wrap_payload);
	st_resolved_generation_ack wrap = {};
	bool wrap_ok = resolve_and_validate_generation_ack(
		/*wire=*/255, span, /*integrity_ok=*/true, /*clean=*/false,
		/*bitmap_any=*/true, /*bitmap_all=*/false, &wrap);
	check(wrap_ok, "wrap report is owned and valid");
	check(wrap.target == 0, "wire 255 resolves to target 0 without a sentinel");

	// Rejection is side-effect-free: bad integrity, span, class, empty bitmap,
	// and an unowned generation must not touch retention, retransmit, or ACK
	// tracker state.  These are the zero-mutation negative controls from §4.
	const int retain_before = cmd_prev_retain_count;
	const int retransmit_before = retransmit_count;
	const int partial_tracker_before = cmd_last_applied_sack_bsi;
	st_resolved_generation_ack rejected = {};
	check(!resolve_and_validate_generation_ack((unsigned char)wire, span,
		/*integrity_ok=*/false, false, true, false, &rejected),
		"integrity failure is rejected before resolution/routing");
	check(!resolve_and_validate_generation_ack((unsigned char)wire, span - 1,
		true, false, true, false, &rejected), "stored-span mismatch is rejected");
	check(!resolve_and_validate_generation_ack((unsigned char)wire, span,
		true, false, false, false, &rejected), "empty bitmap class is rejected");
	check(!resolve_and_validate_generation_ack((unsigned char)wire, span,
		true, false, true, true, &rejected), "all-ones bitmap cannot route as partial");
	check(!resolve_and_validate_generation_ack((unsigned char)90, span,
		true, false, true, false, &rejected), "unowned target is rejected");
	check(cmd_prev_retain_count == retain_before
		&& retransmit_count == retransmit_before
		&& cmd_last_applied_sack_bsi == partial_tracker_before,
		"all rejected reports produce zero queue/tracker mutation");

	deinit_messages_buffers();
	printf("[TEST-GEN-CANON] %s failures=%d assertions=%d\n",
		fails == 0 ? "PASS" : "FAIL", fails, assertions);
	return fails == 0 ? 0 : 1;
}
