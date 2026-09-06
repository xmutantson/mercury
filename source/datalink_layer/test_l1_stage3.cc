#include "datalink_layer/l1_block_ack.h"

#include <cstdio>
#include <cstdlib>
#include <vector>

#ifdef _WIN32
// MSVCRT never declares the POSIX env setters (same shim as arq_common.cc).
static int setenv(const char* name, const char* value, int overwrite) {
	if(!overwrite && getenv(name) != NULL) return 0;
	return _putenv_s(name, value);
}
static int unsetenv(const char* name) { return _putenv_s(name, ""); }
#endif

using l1_block::BlockAckRuntime;
using l1_block::DispatchDisposition;

namespace {

int failures = 0;

#define CHECK(expr, label) do {                                                \
	if(!(expr)) { std::printf("FAIL: %s\n", label); ++failures; }               \
	else std::printf("PASS: %s\n", label);                                     \
} while(0)

void set_gate(bool enabled, int batches = 4) {
	if(enabled) {
		setenv("MERCURY_L1_BLOCKACK", "1", 1);
		char text[16];
		std::snprintf(text, sizeof(text), "%d", batches);
		setenv("MERCURY_L1_BLOCKACK_N", text, 1);
	} else {
		unsetenv("MERCURY_L1_BLOCKACK");
		unsetenv("MERCURY_L1_BLOCKACK_N");
	}
}

std::vector<uint8_t> full_sack_wire(uint8_t connection, uint8_t start,
	uint8_t count, uint16_t span) {
	l1_block::BlockSack sack = {};
	sack.final = count < 4;
	sack.session_id = 0x11223344u;
	sack.epoch = 7;
	sack.negotiation_id = 0x4455;
	sack.block_serial = 1;
	sack.block_start_bsi = start;
	uint16_t offset = 0;
	for(uint8_t i = 0; i < count; ++i) {
		l1_block::BatchEntry entry = {
			(uint8_t)(start + i), span, offset
		};
		sack.batches.push_back(entry);
		offset = (uint16_t)(offset + span);
	}
	sack.bitmap_width_bits = offset;
	sack.bitmap.assign((offset + 7u) / 8u, 0xff);
	if(offset & 7u)
		sack.bitmap.back() &= (uint8_t)((1u << (offset & 7u)) - 1u);
	std::vector<uint8_t> body;
	CHECK(l1_block::encode_sack(sack, &body), "fixture BLOCK_SACK encodes");
	std::vector<uint8_t> wire;
	wire.push_back(BLOCK_SACK);
	wire.push_back(connection);
	wire.push_back(0);
	wire.insert(wire.end(), body.begin(), body.end());
	return wire;
}

void arm_commander(BlockAckRuntime* commander, uint8_t start, uint8_t count) {
	commander->begin_session(0x5a, 0x11223344u, 7, 0x4455);
	commander->complete_handshake(CAP_L1_BLOCKACK, CAP_L1_BLOCKACK, true);
	for(uint8_t i = 0; i < count; ++i)
		commander->note_transmitted_batch((uint8_t)(start + i));
}

void resign_block_body(std::vector<uint8_t>* wire) {
	const std::size_t body = l1_block::CONTROL_HEADER_BYTES;
	(*wire)[wire->size() - 1] = l1_block::control_crc8(
		wire->data() + body, wire->size() - body - 1);
}

void dispatch_acceptance_contract() {
	set_gate(true, 4);
	const uint8_t start = 80;

	// Build a real responder aggregate. This fixed wire vector is the valid-path
	// pre-hardening witness: semantic binding checks must not change one byte.
	BlockAckRuntime responder;
	responder.begin_session(0x5a, 0x11223344u, 7, 0x4455);
	responder.complete_handshake(CAP_L1_BLOCKACK, CAP_L1_BLOCKACK, true);
	std::vector<uint8_t> valid_wire;
	for(uint8_t i = 0; i < 4; ++i)
		responder.observe_received_batch((uint8_t)(start + i),
			std::vector<bool>{true, true}, false, &valid_wire);
	const uint8_t expected_bytes[] = {
		0x47,0x5a,0x00,0x01,0x00,0x00,0x2a,0x00,0x2c,0x11,0x22,0x33,
		0x44,0x00,0x00,0x00,0x07,0x44,0x55,0x00,0x01,0x50,0x04,0x00,
		0x08,0x00,0x00,0x02,0x00,0x00,0x01,0x00,0x02,0x00,0x02,0x02,
		0x00,0x02,0x00,0x04,0x03,0x00,0x02,0x00,0x06,0xff,0xad
	};
	CHECK(valid_wire == std::vector<uint8_t>(expected_bytes,
		expected_bytes + sizeof(expected_bytes)),
		"valid matched aggregate retains its byte-exact wire image");

	BlockAckRuntime valid_commander;
	arm_commander(&valid_commander, start, 4);
	l1_block::LegacyParserAckState valid_state = {FREE, NONE, 0, 0};
	CHECK(valid_commander.dispatch_received_frame(valid_wire.data(), valid_wire.size(),
		&valid_state, NULL) == DispatchDisposition::BLOCK_FRAME_ACCEPTED,
		"valid locally matched aggregate ACK is accepted");

	// Re-sign a structurally valid SACK after changing only its authored start.
	// The former dispatcher bound decode_sack() to this untrusted wire field,
	// making the check tautological instead of matching the pending TX group.
	std::vector<uint8_t> wrong_start = valid_wire;
	wrong_start[l1_block::CONTROL_HEADER_BYTES + 18] = (uint8_t)(start + 1);
	resign_block_body(&wrong_start);
	BlockAckRuntime start_commander;
	arm_commander(&start_commander, start, 4);
	l1_block::LegacyParserAckState start_state = {FREE, NONE, 0, 0};
	CHECK(start_commander.dispatch_received_frame(wrong_start.data(), wrong_start.size(),
		&start_state, NULL) == DispatchDisposition::INVALID,
		"SACK with wrong wire start is rejected by dispatch");

	// This is independently canonical and CRC-valid, but covers three batches
	// while the local commander has exactly four pending.
	std::vector<uint8_t> wrong_count = full_sack_wire(0x5a, start, 3, 2);
	BlockAckRuntime count_commander;
	arm_commander(&count_commander, start, 4);
	l1_block::LegacyParserAckState count_state = {FREE, NONE, 0, 0};
	CHECK(count_commander.dispatch_received_frame(wrong_count.data(), wrong_count.size(),
		&count_state, NULL) == DispatchDisposition::INVALID,
		"SACK with wrong pending batch count is rejected by dispatch");

	// A valid short-tail COMMIT is re-signed after clearing only block_mode.
	// Codec validity is preserved; the production dispatcher must reject it.
	BlockAckRuntime tail_responder;
	tail_responder.begin_session(0x5a, 0x11223344u, 7, 0x4455);
	tail_responder.complete_handshake(CAP_L1_BLOCKACK, CAP_L1_BLOCKACK, true);
	std::vector<uint8_t> ignored;
	tail_responder.observe_received_batch(90, std::vector<bool>{true}, false, &ignored);
	tail_responder.observe_received_batch(91, std::vector<bool>{true}, false, &ignored);
	BlockAckRuntime tail_commander;
	arm_commander(&tail_commander, 90, 2);
	std::vector<uint8_t> commit;
	CHECK(tail_commander.build_tail_commit(&commit),
		"fixture builds a valid short-tail COMMIT");
	commit[l1_block::CONTROL_HEADER_BYTES + 1] = 0;
	resign_block_body(&commit);
	l1_block::LegacyParserAckState commit_state = {FREE, NONE, 0, 0};
	CHECK(tail_responder.dispatch_received_frame(commit.data(), commit.size(),
		&commit_state, NULL) == DispatchDisposition::INVALID
		&& !tail_responder.take_flush_request(),
		"COMMIT with block_mode=false is rejected by dispatch");

	set_gate(false);
}

void legacy_rx_fence() {
	set_gate(true);
	BlockAckRuntime legacy;
	legacy.begin_session(0x5a, 0x11223344u, 7, 0x4455);
	legacy.complete_handshake(CAP_L1_BLOCKACK, 0, true);

	l1_block::LegacyParserAckState state = {FREE, NONE, 0x6c, 0x6d};
	const l1_block::LegacyParserAckState before = state;
	std::vector<uint8_t> wire = full_sack_wire(0x5a, 9, 1, 2);
	DispatchDisposition disposition = legacy.dispatch_received_frame(
		wire.data(), wire.size(), &state, NULL);
	CHECK(disposition == DispatchDisposition::FENCED,
		"legacy/mixed RX fences a BLOCK_SACK before dispatch");
	CHECK(state.rx_status == before.rx_status && state.rx_type == before.rx_type &&
		state.rx_sequence == before.rx_sequence &&
		state.last_received_sequence == before.last_received_sequence,
		"fenced frame changes zero legacy parser/ACK state");
}

void capability_negotiation() {
	set_gate(true);
	BlockAckRuntime both;
	const uint8_t base = CAP_WB_CAPABLE;
	CHECK(both.advertised_capability(base) == (uint8_t)(base | CAP_L1_BLOCKACK),
		"enabled gate advertises L1 block-ACK capability");
	both.begin_session(7, 0x11223344u, 7, 0x4455);
	both.complete_handshake((uint8_t)(base | CAP_L1_BLOCKACK),
		(uint8_t)(base | CAP_L1_BLOCKACK), true);
	CHECK(both.enabled(), "both authenticated peers negotiate block-ACK mode");

	BlockAckRuntime mixed;
	mixed.begin_session(7, 0x11223344u, 7, 0x4455);
	mixed.complete_handshake((uint8_t)(base | CAP_L1_BLOCKACK), base, true);
	CHECK(!mixed.enabled(), "mixed peer fails closed to legacy ACK mode");

	BlockAckRuntime bad_echo;
	bad_echo.begin_session(7, 0x11223344u, 7, 0x4455);
	bad_echo.complete_handshake((uint8_t)(base | CAP_L1_BLOCKACK),
		(uint8_t)(base | CAP_L1_BLOCKACK), false);
	CHECK(!bad_echo.enabled(), "unauthenticated/mismatched echo cannot enable mode");
}

void aggregation_and_atomic_settlement() {
	set_gate(true, 4);
	BlockAckRuntime responder;
	responder.begin_session(0x5a, 0x11223344u, 7, 0x4455);
	responder.complete_handshake(CAP_L1_BLOCKACK, CAP_L1_BLOCKACK, true);
	BlockAckRuntime burst_commander;
	burst_commander.begin_session(0x5a, 0x11223344u, 7, 0x4455);
	burst_commander.complete_handshake(CAP_L1_BLOCKACK, CAP_L1_BLOCKACK, true);
	std::vector<std::vector<uint8_t> > emitted;
	for(uint8_t bsi = 20; bsi < 29; ++bsi) {
		std::vector<uint8_t> frame;
		burst_commander.note_transmitted_batch(bsi);
		bool ready = responder.observe_received_batch(bsi,
			std::vector<bool>{true, true}, false, &frame);
		if(ready) {
			l1_block::LegacyParserAckState burst_state = {FREE, NONE, 0, 0};
			CHECK(burst_commander.dispatch_received_frame(frame.data(), frame.size(),
				&burst_state, NULL) == DispatchDisposition::BLOCK_FRAME_ACCEPTED,
				"production commander closes a complete retained group");
			emitted.push_back(frame);
		}
	}
	CHECK(burst_commander.intermediate_silence_expected(),
		"short tail remains journal-owned while waiting for an explicit flush");
	std::vector<uint8_t> commit;
	CHECK(burst_commander.build_tail_commit(&commit),
		"production commander builds a session-bound short-tail commit");
	l1_block::LegacyParserAckState commit_state = {FREE, NONE, 0, 0};
	CHECK(responder.dispatch_received_frame(commit.data(), commit.size(),
		&commit_state, NULL) == DispatchDisposition::BLOCK_FRAME_ACCEPTED
		&& responder.take_flush_request(),
		"production RX dispatch accepts the commit before legacy parsing");
	std::vector<uint8_t> tail_frame;
	CHECK(responder.flush_received_batches(&tail_frame),
		"tail commit emits the retained short aggregate");
	CHECK(burst_commander.dispatch_received_frame(tail_frame.data(), tail_frame.size(),
		&commit_state, NULL) == DispatchDisposition::BLOCK_FRAME_ACCEPTED,
		"production commander closes the committed short group");
	emitted.push_back(tail_frame);
	CHECK(emitted.size() == 3,
		"N=4 aggregation emits ceil(9/4)=3 reverse ACK frames");

	setenv("MERCURY_L1_JOURNAL", "1", 1);
	mercury::L1TerminalQueue terminal;
	mercury::L1TxJournal journal(&terminal);
	journal.begin_session(0x5a, 0x11223344u);
	for(uint8_t bsi = 20; bsi < 29; ++bsi) {
		std::vector<mercury::L1StageItem> items;
		for(uint16_t slot = 0; slot < 2; ++slot) {
			mercury::L1StageItem item;
			item.slot = slot;
			item.batch_index = (uint16_t)(bsi - 20);
			item.span = 2;
			item.plaintext.assign(1, (char)(bsi + slot));
			items.push_back(item);
		}
		CHECK(journal.stage_batch(bsi, items), "production journal stages retained batch");
		journal.mark_sent(bsi, 0);
		journal.mark_sent(bsi, 1);
	}
	CHECK(journal.size() == 18, "journal owns all nine unsettled batches");

	BlockAckRuntime commander;
	commander.begin_session(0x5a, 0x11223344u, 7, 0x4455);
	commander.complete_handshake(CAP_L1_BLOCKACK, CAP_L1_BLOCKACK, true);
	uint8_t next_bsi = 20;
	for(size_t i = 0; i < emitted.size(); ++i) {
		const uint8_t batch_count = i < 2 ? 4 : 1;
		for(uint8_t batch = 0; batch < batch_count; ++batch)
			commander.note_transmitted_batch(next_bsi++);
		l1_block::LegacyParserAckState state = {FREE, NONE, 0, 0};
		CHECK(commander.dispatch_received_frame(emitted[i].data(), emitted[i].size(),
			&state, &journal) == DispatchDisposition::BLOCK_ACK_APPLIED,
			"negotiated RX dispatch applies aggregate at journal chokepoint");
	}
	CHECK(journal.size() == 0,
		"three atomic aggregate settlements release all retained ownership");

	// The retained identity must survive the uint8 BSI rollover inside one
	// aggregate. Drive the same journal stage/send and wire dispatcher paths.
	mercury::L1TxJournal wrap_journal(&terminal);
	wrap_journal.begin_session(0x5a, 0x11223344u);
	BlockAckRuntime wrap_responder;
	wrap_responder.begin_session(0x5a, 0x11223344u, 7, 0x4455);
	wrap_responder.complete_handshake(CAP_L1_BLOCKACK, CAP_L1_BLOCKACK, true);
	std::vector<uint8_t> wrap_wire;
	for(int index = 0; index < 4; ++index) {
		uint8_t bsi = (uint8_t)(254 + index);
		mercury::L1StageItem item;
		item.slot = 0; item.batch_index = (uint16_t)index; item.span = 1;
		item.plaintext.assign(1, (char)bsi);
		CHECK(wrap_journal.stage_batch(bsi,
			std::vector<mercury::L1StageItem>(1, item)),
			"journal retains sequential batch across BSI rollover");
		wrap_journal.mark_sent(bsi, 0);
		wrap_responder.observe_received_batch(bsi,
			std::vector<bool>(1, true), index == 3, &wrap_wire);
	}
	BlockAckRuntime wrap_commander;
	wrap_commander.begin_session(0x5a, 0x11223344u, 7, 0x4455);
	wrap_commander.complete_handshake(CAP_L1_BLOCKACK, CAP_L1_BLOCKACK, true);
	for(int index = 0; index < 4; ++index)
		wrap_commander.note_transmitted_batch((uint8_t)(254 + index));
	l1_block::LegacyParserAckState wrap_state = {FREE, NONE, 0, 0};
	CHECK(wrap_commander.dispatch_received_frame(wrap_wire.data(), wrap_wire.size(),
		&wrap_state, &wrap_journal) == DispatchDisposition::BLOCK_ACK_APPLIED &&
		wrap_journal.size() == 0,
		"one atomic aggregate settles BSIs 254,255,0,1 without epoch loss");

	mercury::L1TxJournal partial_journal(&terminal);
	partial_journal.begin_session(0x5a, 0x11223344u);
	BlockAckRuntime partial_responder;
	partial_responder.begin_session(0x5a, 0x11223344u, 7, 0x4455);
	partial_responder.complete_handshake(CAP_L1_BLOCKACK, CAP_L1_BLOCKACK, true);
	std::vector<uint8_t> partial_wire;
	for(uint8_t bsi = 40; bsi < 42; ++bsi) {
		std::vector<mercury::L1StageItem> items;
		for(uint16_t slot = 0; slot < 2; ++slot) {
			mercury::L1StageItem item;
			item.slot = slot; item.batch_index = (uint16_t)(bsi - 40);
			item.span = 2; item.plaintext.assign(1, (char)(bsi + slot));
			items.push_back(item);
		}
		CHECK(partial_journal.stage_batch(bsi, items),
			"journal stages partially received aggregate member");
		partial_journal.mark_sent(bsi, 0);
		partial_journal.mark_sent(bsi, 1);
		partial_responder.observe_received_batch(bsi,
			std::vector<bool>{true, false}, bsi == 41, &partial_wire);
	}
	BlockAckRuntime partial_commander;
	partial_commander.begin_session(0x5a, 0x11223344u, 7, 0x4455);
	partial_commander.complete_handshake(CAP_L1_BLOCKACK, CAP_L1_BLOCKACK, true);
	partial_commander.note_transmitted_batch(40);
	partial_commander.note_transmitted_batch(41);
	l1_block::LegacyParserAckState partial_state = {FREE, NONE, 0, 0};
	CHECK(partial_commander.dispatch_received_frame(partial_wire.data(),
		partial_wire.size(), &partial_state, &partial_journal)
		== DispatchDisposition::BLOCK_ACK_APPLIED &&
		partial_journal.size() == 2 && !partial_commander.last_ack_all_received(),
		"partial aggregate atomically releases positives and retains both missing owners");
	CHECK(partial_journal.terminalize("stage3-test-partial-cleanup", {}),
		"partial test transfers retained misses to terminal owner without delivery claim");
	unsetenv("MERCURY_L1_JOURNAL");
}

// Site 1 classifier: aggregate_response_outstanding() must be the exact
// complement of intermediate_silence_expected() within an active block — false
// for the intermediate 1..N-1 window, true once the block is complete (Nth batch
// sent) or a short tail commit has been sent, and false again the instant the
// aggregate is applied. This is the predicate the commander BREAK-path hold keys
// on to distinguish a deferred/in-flight aggregate from a channel ACK-absence.
void aggregate_outstanding_classifier() {
	set_gate(true, 4);
	BlockAckRuntime commander;
	commander.begin_session(0x5a, 0x11223344u, 7, 0x4455);
	commander.complete_handshake(CAP_L1_BLOCKACK, CAP_L1_BLOCKACK, true);

	// Fresh session: no block active — neither window is open.
	CHECK(!commander.intermediate_silence_expected()
		&& !commander.aggregate_response_outstanding(),
		"fresh block session: neither silence window is open");

	// Intermediate 1..N-1 batches: silence expected, aggregate NOT yet outstanding.
	commander.note_transmitted_batch(20);
	commander.note_transmitted_batch(21);
	commander.note_transmitted_batch(22);
	CHECK(commander.intermediate_silence_expected()
		&& !commander.aggregate_response_outstanding(),
		"intermediate 1..N-1 window: silence expected, aggregate NOT outstanding");

	// Nth batch transmitted: block complete, the single aggregate is now DUE.
	commander.note_transmitted_batch(23);
	CHECK(!commander.intermediate_silence_expected()
		&& commander.aggregate_response_outstanding(),
		"complete block (Nth batch): aggregate outstanding, silence window closed");

	// Applying the aggregate clears the outstanding window (tx_pending_batches_=0).
	std::vector<uint8_t> agg = full_sack_wire(0x5a, 20, 4, 2);
	l1_block::LegacyParserAckState st = {FREE, NONE, 0, 0};
	CHECK(commander.dispatch_received_frame(agg.data(), agg.size(), &st, NULL)
		!= DispatchDisposition::INVALID,
		"commander applies the full aggregate");
	CHECK(!commander.intermediate_silence_expected()
		&& !commander.aggregate_response_outstanding(),
		"aggregate applied: outstanding window closes");

	// Short final group whose tail commit has been sent is ALSO outstanding.
	BlockAckRuntime tail_cmd;
	tail_cmd.begin_session(0x5a, 0x11223344u, 7, 0x4455);
	tail_cmd.complete_handshake(CAP_L1_BLOCKACK, CAP_L1_BLOCKACK, true);
	tail_cmd.note_transmitted_batch(30);
	tail_cmd.note_transmitted_batch(31);
	std::vector<uint8_t> commit;
	CHECK(tail_cmd.build_tail_commit(&commit),
		"short-tail commander builds a tail commit");
	CHECK(!tail_cmd.intermediate_silence_expected()
		&& tail_cmd.aggregate_response_outstanding(),
		"short tail after commit: aggregate outstanding, silence window closed");
}

void lost_aggregate_replay() {
	set_gate(true, 4);
	unsetenv("MERCURY_DEMOTE_TEMPORAL_HYSTERESIS");
	BlockAckRuntime off_responder;
	off_responder.begin_session(0x5a, 0x11223344u, 7, 0x4455);
	off_responder.complete_handshake(CAP_L1_BLOCKACK, CAP_L1_BLOCKACK, true);
	std::vector<uint8_t> off_dropped;
	for(uint8_t bsi = 20; bsi < 24; ++bsi) {
		std::vector<uint8_t> frame;
		if(off_responder.observe_received_batch(bsi,
			std::vector<bool>{true, true}, false, &frame)) off_dropped = frame;
	}
	std::vector<uint8_t> off_replay;
	CHECK(!off_dropped.empty() && !off_responder.observe_received_batch(23,
		std::vector<bool>{true, true}, false, &off_replay) && off_replay.empty(),
		"temporal gate OFF preserves duplicate-tail deferral with zero replay bytes");

	setenv("MERCURY_DEMOTE_TEMPORAL_HYSTERESIS", "1", 1);
	BlockAckRuntime responder;
	responder.begin_session(0x5a, 0x11223344u, 7, 0x4455);
	responder.complete_handshake(CAP_L1_BLOCKACK, CAP_L1_BLOCKACK, true);
	BlockAckRuntime commander;
	commander.begin_session(0x5a, 0x11223344u, 7, 0x4455);
	commander.complete_handshake(CAP_L1_BLOCKACK, CAP_L1_BLOCKACK, true);

	std::vector<uint8_t> dropped;
	for(uint8_t bsi = 20; bsi < 24; ++bsi) {
		commander.note_transmitted_batch(bsi);
		std::vector<uint8_t> frame;
		if(responder.observe_received_batch(bsi,
			std::vector<bool>{true, true}, false, &frame)) dropped = frame;
	}
	CHECK(!dropped.empty(), "fixture emits the full N=4 aggregate that is dropped");
	CHECK(dropped == off_dropped,
		"temporal gate leaves the first aggregate wire byte-identical");

	// The commander's retained one-batch cache contains only the aggregate tail.
	// If the aggregate response is lost it retransmits that tail BSI. The responder
	// must classify it as a duplicate of the just-flushed block and replay the exact
	// aggregate, not adopt it as batch 1 of a fresh block and defer forever.
	std::vector<uint8_t> replay;
	CHECK(responder.observe_received_batch(23,
		std::vector<bool>{true, true}, false, &replay) && replay == dropped,
		"lost aggregate + duplicate tail BSI replays the exact validated BLOCK_SACK");
	l1_block::LegacyParserAckState state = {FREE, NONE, 0, 0};
	CHECK(!replay.empty() && commander.dispatch_received_frame(replay.data(),
		replay.size(), &state, NULL) == DispatchDisposition::BLOCK_FRAME_ACCEPTED,
		"commander accepts replay under the still-outstanding aggregate identity");
	unsetenv("MERCURY_DEMOTE_TEMPORAL_HYSTERESIS");
}

void gate_off_byte_identity() {
	set_gate(false);
	BlockAckRuntime off;
	const uint8_t caps = (uint8_t)(CAP_WB_CAPABLE | CAP_ENCRYPTION);
	CHECK(off.advertised_capability(caps) == caps,
		"gate OFF leaves handshake capability byte identical");
	off.begin_session(0x5a, 0x11223344u, 7, 0x4455);
	off.complete_handshake(caps, caps, true);
	CHECK(!off.enabled(), "gate OFF cannot negotiate block-ACK");
	std::vector<uint8_t> frame;
	CHECK(!off.observe_received_batch(1, std::vector<bool>{true, true}, true, &frame)
		&& frame.empty(), "gate OFF emits zero block-ACK wire bytes");

	const uint8_t legacy[] = {SACK_RSP, 0x5a, 0x35};
	l1_block::LegacyParserAckState a = {FREE, NONE, 1, 2};
	l1_block::LegacyParserAckState b = a;
	bool legacy_seated = l1_block::seat_legacy_receive_prefix(legacy, sizeof(legacy), &a);
	DispatchDisposition disposition = off.dispatch_received_frame(
		legacy, sizeof(legacy), &b, NULL);
	CHECK(legacy_seated && disposition == DispatchDisposition::LEGACY_SEATED &&
		a.rx_status == b.rx_status && a.rx_type == b.rx_type &&
		a.rx_sequence == b.rx_sequence &&
		a.last_received_sequence == b.last_received_sequence,
		"gate OFF legacy RX state transition is byte/field identical");

	setenv("MERCURY_L1_BLOCKACK", "2", 1);
	BlockAckRuntime not_exact;
	mercury::L1TxJournal not_exact_journal;
	CHECK(not_exact.advertised_capability(caps) == caps
		&& !not_exact.gate_enabled() && !not_exact_journal.enabled(),
		"only the exact value MERCURY_L1_BLOCKACK=1 changes capability or ownership");
	set_gate(false);
}

// STAGE R pipeline gate (MERCURY_L1_BLOCKACK_PIPELINE) is LIVE + exact-value, and the
// knob is a COMMANDER receive-window policy that must not change any BlockAckRuntime
// wire/state output: the CMD->RSP aggregate is byte-identical with the knob off vs on.
void pipeline_gate_and_roundtrip_identity() {
	unsetenv("MERCURY_L1_BLOCKACK_PIPELINE");
	CHECK(!l1_block::pipeline_enabled(), "pipeline gate OFF when unset");
	setenv("MERCURY_L1_BLOCKACK_PIPELINE", "1", 1);
	CHECK(l1_block::pipeline_enabled(), "pipeline gate ON at exact value 1");
	setenv("MERCURY_L1_BLOCKACK_PIPELINE", "0", 1);
	CHECK(!l1_block::pipeline_enabled(), "pipeline gate OFF at 0");
	setenv("MERCURY_L1_BLOCKACK_PIPELINE", "2", 1);
	CHECK(!l1_block::pipeline_enabled(), "pipeline gate OFF at non-1 value");
	unsetenv("MERCURY_L1_BLOCKACK_PIPELINE");

	set_gate(true, 4);
	std::vector<std::vector<uint8_t> > off_frames, on_frames;
	for(int arm = 0; arm < 2; ++arm) {
		if(arm == 0) unsetenv("MERCURY_L1_BLOCKACK_PIPELINE");
		else setenv("MERCURY_L1_BLOCKACK_PIPELINE", "1", 1);
		BlockAckRuntime cmd, rsp;
		cmd.begin_session(0x5a, 0x11223344u, 7, 0x4455);
		cmd.complete_handshake(CAP_L1_BLOCKACK, CAP_L1_BLOCKACK, true);
		rsp.begin_session(0x5a, 0x11223344u, 7, 0x4455);
		rsp.complete_handshake(CAP_L1_BLOCKACK, CAP_L1_BLOCKACK, true);
		std::vector<std::vector<uint8_t> >& out = (arm == 0) ? off_frames : on_frames;
		for(uint8_t bsi = 20; bsi < 24; ++bsi) {
			cmd.note_transmitted_batch(bsi);
			CHECK(cmd.intermediate_silence_expected() != cmd.aggregate_response_outstanding(),
				"pipeline arm: intermediate/boundary are exact complements within the block");
			std::vector<uint8_t> frame;
			if(rsp.observe_received_batch(bsi, std::vector<bool>{true, true}, false, &frame))
				out.push_back(frame);
		}
	}
	CHECK(off_frames == on_frames && off_frames.size() == 1,
		"pipeline knob leaves the block-ACK aggregate wire byte-identical (RX unchanged)");
	unsetenv("MERCURY_L1_BLOCKACK_PIPELINE");
	set_gate(false);
}

// STAGE F gearshift feed (MERCURY_L1_BLOCKACK_GEARFEED). The gate is LIVE +
// exact-value, and dispatch_received_frame exposes the aggregate's per-batch
// breakdown (last_applied_batches) that the commander replays into the
// gearshift. The knob is COMMANDER-side and touches NO wire/RX state, so it does
// not appear in this l1 layer beyond the gate + the exposure accessor.
void gearfeed_gate_and_exposure() {
	unsetenv("MERCURY_L1_BLOCKACK_GEARFEED");
	CHECK(!l1_block::gearfeed_enabled(), "gearfeed gate OFF when unset");
	setenv("MERCURY_L1_BLOCKACK_GEARFEED", "1", 1);
	CHECK(l1_block::gearfeed_enabled(), "gearfeed gate ON at exact value 1");
	setenv("MERCURY_L1_BLOCKACK_GEARFEED", "0", 1);
	CHECK(!l1_block::gearfeed_enabled(), "gearfeed gate OFF at 0");
	setenv("MERCURY_L1_BLOCKACK_GEARFEED", "2", 1);
	CHECK(!l1_block::gearfeed_enabled(), "gearfeed gate OFF at non-1 value");
	unsetenv("MERCURY_L1_BLOCKACK_GEARFEED");

	set_gate(true, 4);
	setenv("MERCURY_L1_JOURNAL", "1", 1);
	mercury::L1TerminalQueue terminal;

	// A CLEAN N=4 aggregate: last_applied_batches() reports 4 batches, each with
	// acked_slots == span_slots (all delivered), and last_ack_all_received().
	{
		mercury::L1TxJournal journal(&terminal);
		journal.begin_session(0x5a, 0x11223344u);
		BlockAckRuntime responder;
		responder.begin_session(0x5a, 0x11223344u, 7, 0x4455);
		responder.complete_handshake(CAP_L1_BLOCKACK, CAP_L1_BLOCKACK, true);
		std::vector<uint8_t> wire;
		for(uint8_t bsi = 50; bsi < 54; ++bsi) {
			std::vector<mercury::L1StageItem> items;
			for(uint16_t slot = 0; slot < 2; ++slot) {
				mercury::L1StageItem item;
				item.slot = slot; item.batch_index = (uint16_t)(bsi - 50);
				item.span = 2; item.plaintext.assign(1, (char)(bsi + slot));
				items.push_back(item);
			}
			journal.stage_batch(bsi, items);
			journal.mark_sent(bsi, 0);
			journal.mark_sent(bsi, 1);
			responder.observe_received_batch(bsi, std::vector<bool>{true, true},
				bsi == 53, &wire);
		}
		BlockAckRuntime commander;
		commander.begin_session(0x5a, 0x11223344u, 7, 0x4455);
		commander.complete_handshake(CAP_L1_BLOCKACK, CAP_L1_BLOCKACK, true);
		for(uint8_t bsi = 50; bsi < 54; ++bsi)
			commander.note_transmitted_batch(bsi);
		l1_block::LegacyParserAckState state = {FREE, NONE, 0, 0};
		commander.dispatch_received_frame(wire.data(), wire.size(), &state, &journal);
		const std::vector<l1_block::PerBatchAck>& b = commander.last_applied_batches();
		bool ok = commander.last_ack_all_received()
			&& commander.last_ack_batch_count() == 4
			&& b.size() == 4;
		for(std::size_t i = 0; ok && i < b.size(); ++i)
			ok = ok && b[i].bsi == (uint8_t)(50 + i)
			     && b[i].span_slots == 2 && b[i].acked_slots == 2;
		CHECK(ok, "clean aggregate exposes 4 per-batch entries, each acked==span");
	}

	// A PARTIAL aggregate: one slot missing in the 2nd batch => that entry reports
	// acked_slots < span_slots and last_ack_all_received() is false (the commander
	// DROPs such an aggregate in production; the per-batch truth is still exact).
	{
		mercury::L1TxJournal journal(&terminal);
		journal.begin_session(0x5a, 0x11223344u);
		BlockAckRuntime responder;
		responder.begin_session(0x5a, 0x11223344u, 7, 0x4455);
		responder.complete_handshake(CAP_L1_BLOCKACK, CAP_L1_BLOCKACK, true);
		std::vector<uint8_t> wire;
		for(uint8_t bsi = 60; bsi < 62; ++bsi) {
			std::vector<mercury::L1StageItem> items;
			for(uint16_t slot = 0; slot < 2; ++slot) {
				mercury::L1StageItem item;
				item.slot = slot; item.batch_index = (uint16_t)(bsi - 60);
				item.span = 2; item.plaintext.assign(1, (char)(bsi + slot));
				items.push_back(item);
			}
			journal.stage_batch(bsi, items);
			journal.mark_sent(bsi, 0);
			journal.mark_sent(bsi, 1);
			std::vector<bool> got = (bsi == 61)
				? std::vector<bool>{true, false} : std::vector<bool>{true, true};
			responder.observe_received_batch(bsi, got, bsi == 61, &wire);
		}
		BlockAckRuntime commander;
		commander.begin_session(0x5a, 0x11223344u, 7, 0x4455);
		commander.complete_handshake(CAP_L1_BLOCKACK, CAP_L1_BLOCKACK, true);
		commander.note_transmitted_batch(60);
		commander.note_transmitted_batch(61);
		l1_block::LegacyParserAckState state = {FREE, NONE, 0, 0};
		commander.dispatch_received_frame(wire.data(), wire.size(), &state, &journal);
		const std::vector<l1_block::PerBatchAck>& b = commander.last_applied_batches();
		bool ok = !commander.last_ack_all_received() && b.size() == 2
			&& b[0].acked_slots == 2 && b[1].span_slots == 2 && b[1].acked_slots == 1;
		CHECK(ok, "partial aggregate exposes the under-delivered batch (acked<span, not all-received)");
	}

	unsetenv("MERCURY_L1_JOURNAL");
	set_gate(false);
}

}  // namespace

int main() {
	dispatch_acceptance_contract();
	legacy_rx_fence();
	capability_negotiation();
	aggregation_and_atomic_settlement();
	aggregate_outstanding_classifier();
	lost_aggregate_replay();
	gate_off_byte_identity();
	pipeline_gate_and_roundtrip_identity();
	gearfeed_gate_and_exposure();
	std::printf("L1 Stage-3: %s (%d failures)\n",
		failures ? "FAIL" : "PASS", failures);
	return failures ? 1 : 0;
}
