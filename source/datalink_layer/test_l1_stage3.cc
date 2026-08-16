#include "datalink_layer/l1_block_ack.h"

#include <cstdio>
#include <cstdlib>
#include <vector>

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
	sack.final = true;
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
	for(size_t i = 0; i < emitted.size(); ++i) {
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

}  // namespace

int main() {
	legacy_rx_fence();
	capability_negotiation();
	aggregation_and_atomic_settlement();
	gate_off_byte_identity();
	std::printf("L1 Stage-3: %s (%d failures)\n",
		failures ? "FAIL" : "PASS", failures);
	return failures ? 1 : 0;
}
