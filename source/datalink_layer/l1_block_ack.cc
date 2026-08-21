#include "datalink_layer/l1_block_ack.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <utility>

namespace l1_block {
namespace {

static const std::size_t MAX_BATCH_SLOTS = 96;

uint8_t configured_batches() {
	const char* value = std::getenv("MERCURY_L1_BLOCKACK_N");
	if(!value || !*value) return 4;
	char* end = NULL;
	long parsed = std::strtol(value, &end, 10);
	if(end == value || *end != '\0' || parsed < 2 || parsed > 8) return 4;
	return (uint8_t)parsed;
}

uint16_t get16(const uint8_t* p) {
	return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

bool bitmap_bit(const std::vector<uint8_t>& bitmap, uint16_t bit) {
	return (bitmap[bit / 8u] & (uint8_t)(1u << (bit & 7u))) != 0;
}

void append_bit(std::vector<uint8_t>* bitmap, uint16_t bit, bool value) {
	const std::size_t bytes = (std::size_t)(bit / 8u + 1u);
	if(bitmap->size() < bytes) bitmap->resize(bytes, 0);
	if(value) (*bitmap)[bit / 8u] |= (uint8_t)(1u << (bit & 7u));
}

}  // namespace

bool feature_gate_enabled() {
	const char* value = std::getenv("MERCURY_L1_BLOCKACK");
	return value && std::strcmp(value, "1") == 0;
}

bool temporal_hysteresis_enabled() {
	const char* value = std::getenv("MERCURY_DEMOTE_TEMPORAL_HYSTERESIS");
	return value && std::strcmp(value, "1") == 0;
}

uint8_t capability_advertise_bit() {
	return feature_gate_enabled() ? (uint8_t)CAP_L1_BLOCKACK : (uint8_t)0;
}

BlockAckRuntime::BlockAckRuntime()
	: gate_enabled_(feature_gate_enabled()), negotiated_(false),
	  aggregate_batches_(configured_batches()), connection_id_(0),
	  session_id_(0), epoch_(0), negotiation_id_(0), tx_block_serial_(0),
	  rx_block_serial_(0), pending_start_bsi_(0), pending_width_(0),
	  last_ack_all_received_(false), last_ack_batch_count_(0),
	  last_ack_slot_count_(0), tx_pending_start_bsi_(0),
	  tx_pending_batches_(0), tx_tail_commit_sent_(false),
	  flush_requested_(false), last_tx_ack_valid_(false),
	  last_tx_ack_tail_bsi_(0) {}

uint8_t BlockAckRuntime::advertised_capability(uint8_t legacy) const {
#ifdef L1_BLOCK_GATE_OFF_DEFEAT
	return (uint8_t)(legacy | CAP_L1_BLOCKACK);
#else
	return gate_enabled_ ? (uint8_t)(legacy | CAP_L1_BLOCKACK) : legacy;
#endif
}

void BlockAckRuntime::begin_session(uint8_t connection, uint32_t session,
	uint32_t epoch, uint16_t negotiation) {
	connection_id_ = connection;
	session_id_ = session;
	epoch_ = epoch;
	negotiation_id_ = negotiation;
	negotiated_ = false;
	tx_block_serial_ = 0;
	rx_block_serial_ = 0;
	pending_start_bsi_ = 0;
	pending_batches_.clear();
	pending_bitmap_.clear();
	pending_width_ = 0;
	last_ack_all_received_ = false;
	last_ack_batch_count_ = 0;
	last_ack_slot_count_ = 0;
	tx_pending_start_bsi_ = 0;
	tx_pending_batches_ = 0;
	tx_tail_commit_sent_ = false;
	flush_requested_ = false;
	last_tx_ack_valid_ = false;
	last_tx_ack_tail_bsi_ = 0;
	last_tx_ack_wire_.clear();
}

void BlockAckRuntime::note_transmitted_batch(uint8_t bsi) {
	if(!negotiated_) return;
	if(tx_pending_batches_ == 0) {
		tx_pending_start_bsi_ = bsi;
	} else if(bsi != (uint8_t)(tx_pending_start_bsi_ + tx_pending_batches_)) {
		// A non-contiguous generation cannot share one aggregate identity.
		// Force the caller into a full response wait; it must not silently
		// advance another batch under a malformed group.
		tx_pending_batches_ = aggregate_batches_;
		tx_tail_commit_sent_ = true;
		return;
	}
	if(tx_pending_batches_ < aggregate_batches_) ++tx_pending_batches_;
	tx_tail_commit_sent_ = false;
}

bool BlockAckRuntime::intermediate_silence_expected() const {
	return negotiated_ && tx_pending_batches_ > 0
		&& tx_pending_batches_ < aggregate_batches_ && !tx_tail_commit_sent_;
}

bool BlockAckRuntime::aggregate_response_outstanding() const {
	// The complement of intermediate_silence_expected() within an active block:
	// the block is COMPLETE (Nth batch transmitted) or a short final group has
	// had its tail commit sent, and the single aggregate BLOCK_SACK is the only
	// outstanding reverse frame. A timeout in this window is a DEFERRED (or lost)
	// aggregate, not a channel ACK-absence.
	return negotiated_ && tx_pending_batches_ > 0
		&& (tx_pending_batches_ >= aggregate_batches_ || tx_tail_commit_sent_);
}

bool BlockAckRuntime::build_tail_commit(std::vector<uint8_t>* wire) {
	if(wire) wire->clear();
	if(!negotiated_ || !wire || tx_pending_batches_ == 0
		|| tx_pending_batches_ >= aggregate_batches_) return false;
	BlockDesc commit = {};
	commit.block_mode = true;
	commit.session_id = session_id_;
	commit.epoch = epoch_;
	commit.negotiation_id = negotiation_id_;
	commit.block_serial = (uint16_t)(rx_block_serial_ + 1u);
	commit.block_start_bsi = tx_pending_start_bsi_;
	commit.batch_index = (uint8_t)(tx_pending_batches_ - 1u);
	commit.batch_count_limit = aggregate_batches_;
	commit.batch_span = tx_pending_batches_;
	std::vector<uint8_t> body;
	if(!encode_desc(commit, &body)) return false;
	wire->reserve(CONTROL_HEADER_BYTES + body.size());
	wire->push_back(BLOCK_COMMIT_TYPE);
	wire->push_back(connection_id_);
	wire->push_back(0);
	wire->insert(wire->end(), body.begin(), body.end());
	tx_tail_commit_sent_ = true;
	return true;
}

void BlockAckRuntime::reset_session() {
	begin_session(0, 0, 0, 0);
}

void BlockAckRuntime::complete_handshake(uint8_t local, uint8_t peer,
	bool authenticated_echo) {
	if(!gate_enabled_) {
		negotiated_ = false;
		return;
	}
#ifdef L1_BLOCK_CAPNEG_DEFEAT
	negotiated_ = false;
#elif defined(L1_BLOCK_MIXED_FAILOPEN_DEFEAT)
	negotiated_ = authenticated_echo && (local & CAP_L1_BLOCKACK)
		&& session_id_ != 0 && epoch_ != 0;
#else
	negotiated_ = gate_enabled_ && authenticated_echo
		&& (local & CAP_L1_BLOCKACK) && (peer & CAP_L1_BLOCKACK)
		&& session_id_ != 0 && epoch_ != 0;
#endif
	std::printf("[L1-BLOCKACK-NEG] %s local=0x%02x peer=0x%02x N=%u\n",
		negotiated_ ? "NEGOTIATED" : "legacy",
		(unsigned)local, (unsigned)peer, (unsigned)aggregate_batches_);
	std::fflush(stdout);
}

DispatchDisposition BlockAckRuntime::dispatch_received_frame(
	const uint8_t* wire, std::size_t size, LegacyParserAckState* legacy_state,
	mercury::L1TxJournal* journal) {
	if(!wire || size < CONTROL_HEADER_BYTES || !legacy_state)
		return DispatchDisposition::INVALID;
	if(!is_block_message_type(wire[0]))
		return seat_legacy_receive_prefix(wire, size, legacy_state)
			? DispatchDisposition::LEGACY_SEATED : DispatchDisposition::IGNORED;

	// Load-bearing compatibility fence: an unnegotiated block-family type is
	// consumed before the legacy parser can mutate ACK state.
	if(!negotiated_) {
#ifdef L1_BLOCK_RX_FENCE_DEFEAT
		legacy_state->rx_status = RECEIVED;
		legacy_state->rx_type = wire[0];
		legacy_state->rx_sequence = (uint8_t)(wire[2] & 0x7fu);
		legacy_state->last_received_sequence = legacy_state->rx_sequence;
		return DispatchDisposition::LEGACY_SEATED;
#else
		return DispatchDisposition::FENCED;
#endif
	}
	if(wire[1] != connection_id_)
		return DispatchDisposition::INVALID;
	const uint8_t* body = wire + CONTROL_HEADER_BYTES;
	std::size_t body_size = size - CONTROL_HEADER_BYTES;
	if(wire[0] == BLOCK_DESC_TYPE || wire[0] == BLOCK_COMMIT_TYPE) {
		if(body_size < 20) return DispatchDisposition::INVALID;
		DecodeResult<BlockDesc> decoded = decode_desc(body, 20, NULL);
		if(!decoded.valid || decoded.value.session_id != session_id_
			|| decoded.value.epoch != epoch_
			|| decoded.value.negotiation_id != negotiation_id_)
			return DispatchDisposition::INVALID;
		if(wire[0] == BLOCK_COMMIT_TYPE) {
			if(pending_batches_.empty()
				|| decoded.value.block_serial != (uint16_t)(tx_block_serial_ + 1u)
				|| decoded.value.block_start_bsi != pending_start_bsi_
				|| decoded.value.batch_index + 1u != pending_batches_.size())
				return DispatchDisposition::INVALID;
			flush_requested_ = true;
		}
		return DispatchDisposition::BLOCK_FRAME_ACCEPTED;
	}
	if(wire[0] != BLOCK_SACK_TYPE || body_size < 22)
		return DispatchDisposition::INVALID;
	const uint16_t encoded_size = get16(body + 4);
	if(encoded_size > body_size) return DispatchDisposition::INVALID;
	body_size = encoded_size;

	const uint16_t serial = get16(body + 16);
	const uint8_t start = body[18];
	Binding binding = {session_id_, epoch_, negotiation_id_, serial, start,
		aggregate_batches_, MAX_SLOT_CAP};
	DecodeOptions options = {binding};
	DecodeResult<BlockSack> decoded = decode_sack(body, body_size, options);
	if(!decoded.valid || serial != (uint16_t)(rx_block_serial_ + 1u))
		return DispatchDisposition::INVALID;

	std::vector<std::pair<uint8_t, uint16_t> > keys;
	for(std::size_t batch = 0; batch < decoded.value.batches.size(); ++batch) {
		const BatchEntry& entry = decoded.value.batches[batch];
		for(uint16_t slot = 0; slot < entry.span_slots; ++slot) {
			const uint16_t bit = (uint16_t)(entry.bitmap_offset_bits + slot);
			if(bitmap_bit(decoded.value.bitmap, bit))
				keys.push_back(std::make_pair(entry.bsi, slot));
		}
	}
	last_ack_all_received_ = keys.size() == decoded.value.bitmap_width_bits;
	last_ack_batch_count_ = decoded.value.batches.size();
	last_ack_slot_count_ = keys.size();
	if(journal && !journal->acknowledge_many(keys))
		return DispatchDisposition::INVALID;
	rx_block_serial_ = serial;
	tx_pending_batches_ = 0;
	tx_tail_commit_sent_ = false;
	std::printf("[L1-BLOCKACK-RX] applied block=%u start_bsi=%u batches=%zu acked_slots=%zu\n",
		(unsigned)serial, (unsigned)start, decoded.value.batches.size(), keys.size());
	std::fflush(stdout);
	return journal ? DispatchDisposition::BLOCK_ACK_APPLIED
		: DispatchDisposition::BLOCK_FRAME_ACCEPTED;
}

bool BlockAckRuntime::observe_received_batch(uint8_t bsi,
	const std::vector<bool>& received, bool final, std::vector<uint8_t>* wire) {
	if(wire) wire->clear();
	if(!negotiated_ || !wire || received.empty()
		|| received.size() > MAX_BATCH_SLOTS) return false;
	// A full aggregate response is transmitted once and the responder then clears
	// its pending window. If that response is lost, the commander can only
	// retransmit the just-flushed tail batch (earlier members are journal-only).
	// Replaying the exact prior aggregate for that exact tail BSI closes the still-
	// outstanding identity. Without this replay, the duplicate tail is adopted as
	// batch 1 of a new block and its deliberately deferred ACK manufactures
	// permanent absence. The existing commander miss budget remains authoritative
	// if every replay is also lost.
	if(temporal_hysteresis_enabled() && pending_batches_.empty()
		&& last_tx_ack_valid_ && bsi == last_tx_ack_tail_bsi_)
	{
		*wire = last_tx_ack_wire_;
		std::printf("[L1-BLOCKACK-REPLAY] duplicate tail bsi=%u; replaying prior aggregate\n",
			(unsigned)bsi);
		std::fflush(stdout);
		return true;
	}
	if(!pending_batches_.empty()) {
		const uint8_t expected = (uint8_t)(pending_start_bsi_
			+ (uint8_t)pending_batches_.size());
		if(bsi != expected) {
			pending_batches_.clear();
			pending_bitmap_.clear();
			pending_width_ = 0;
			return false;
		}
	}
	if((uint32_t)pending_width_ + received.size() > MAX_SLOT_CAP)
		return false;
	if(pending_batches_.empty()) pending_start_bsi_ = bsi;
	BatchEntry entry = {bsi, (uint16_t)received.size(), pending_width_};
	pending_batches_.push_back(entry);
	for(std::size_t slot = 0; slot < received.size(); ++slot)
		append_bit(&pending_bitmap_, (uint16_t)(pending_width_ + slot), received[slot]);
	pending_width_ = (uint16_t)(pending_width_ + received.size());
	std::size_t target = aggregate_batches_;
#ifdef L1_BLOCK_AGGREGATION_DEFEAT
	target = 1;
#endif
	if(!final && pending_batches_.size() < target) return false;
	return flush_received_batches(wire);
}

bool BlockAckRuntime::flush_received_batches(std::vector<uint8_t>* wire) {
	if(wire) wire->clear();
	if(!negotiated_ || !wire || pending_batches_.empty()
		|| pending_width_ == 0) return false;

	BlockSack sack = {};
	sack.final = pending_batches_.size() < aggregate_batches_;
	sack.session_id = session_id_;
	sack.epoch = epoch_;
	sack.negotiation_id = negotiation_id_;
	sack.block_serial = (uint16_t)(tx_block_serial_ + 1u);
	sack.block_start_bsi = pending_start_bsi_;
	sack.batches = pending_batches_;
	sack.bitmap_width_bits = pending_width_;
	sack.bitmap = pending_bitmap_;
	std::vector<uint8_t> body;
	if(!encode_sack(sack, &body)) return false;
	wire->reserve(CONTROL_HEADER_BYTES + body.size());
	wire->push_back(BLOCK_SACK_TYPE);
	wire->push_back(connection_id_);
	wire->push_back(0);
	wire->insert(wire->end(), body.begin(), body.end());
	tx_block_serial_ = sack.block_serial;
	std::printf("[L1-BLOCKACK-TX] block=%u start_bsi=%u batches=%zu slots=%u final=%d\n",
		(unsigned)sack.block_serial, (unsigned)sack.block_start_bsi,
		sack.batches.size(), (unsigned)sack.bitmap_width_bits, sack.final ? 1 : 0);
	std::fflush(stdout);
	if(temporal_hysteresis_enabled()) {
		last_tx_ack_valid_ = true;
		last_tx_ack_tail_bsi_ = pending_batches_.back().bsi;
		last_tx_ack_wire_ = *wire;
	}
	pending_batches_.clear();
	pending_bitmap_.clear();
	pending_width_ = 0;
	flush_requested_ = false;
	return true;
}

}  // namespace l1_block
