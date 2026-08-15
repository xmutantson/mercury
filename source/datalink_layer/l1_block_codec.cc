#include "datalink_layer/l1_block_codec.h"

#include <limits>

namespace l1_block {
namespace {

static const size_t DESC_BODY_BYTES = 19;
static const size_t SACK_FIXED_BYTES = 22;
static const size_t DIRECTORY_BYTES = 5;

void put16(std::vector<uint8_t>* out, uint16_t v) {
	out->push_back((uint8_t)(v >> 8)); out->push_back((uint8_t)v);
}
void put32(std::vector<uint8_t>* out, uint32_t v) {
	out->push_back((uint8_t)(v >> 24)); out->push_back((uint8_t)(v >> 16));
	out->push_back((uint8_t)(v >> 8)); out->push_back((uint8_t)v);
}
uint16_t get16(const uint8_t* p) { return (uint16_t)((uint16_t)p[0] << 8 | p[1]); }
uint32_t get32(const uint8_t* p) {
	return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}

template <typename T>
DecodeResult<T> invalid(InvalidReason reason) {
	DecodeResult<T> r = {}; r.valid = false; r.reason = reason; return r;
}

InvalidReason check_binding(uint32_t session, uint32_t epoch, uint16_t negotiation,
	uint16_t serial, uint8_t start, const Binding* b) {
	if(epoch == 0) return InvalidReason::EPOCH_ZERO;
	if(!b) return InvalidReason::VALID;
	if(session != b->session_id) return InvalidReason::SESSION_MISMATCH;
	if(epoch != b->epoch) return InvalidReason::EPOCH_MISMATCH;
	if(negotiation != b->negotiation_id) return InvalidReason::NEGOTIATION_MISMATCH;
	if(serial != b->block_serial) return InvalidReason::BLOCK_MISMATCH;
	if(start != b->block_start_bsi) return InvalidReason::START_BSI_MISMATCH;
	if(b->slot_cap == 0 || b->slot_cap > MAX_SLOT_CAP) return InvalidReason::SLOT_CAP;
	return InvalidReason::VALID;
}

}  // namespace

uint8_t control_crc8(const uint8_t* p, size_t n) {
	uint8_t crc = 0xff;
	for(size_t j = 0; j < n; ++j) {
		crc ^= p[j];
		for(int i = 0; i < 8; ++i)
			crc = (crc & 1u) ? (uint8_t)((crc >> 1) ^ POLY_CRC8)
			                 : (uint8_t)(crc >> 1);
	}
	return crc;
}

bool is_block_message_type(uint8_t type) {
	return type >= BLOCK_DESC_TYPE && type <= BLOCK_COMMIT_ACK_TYPE;
}

bool seat_legacy_receive_prefix(const uint8_t* wire, size_t size,
	LegacyParserAckState* state) {
	if(!wire || size < CONTROL_HEADER_BYTES || !state) return false;
#ifndef L1_BLOCK_TYPE_FENCE_DEFEAT
	if(is_block_message_type(wire[0])) return false;
#endif
	state->rx_status = RECEIVED;
	state->rx_type = wire[0];
	state->rx_sequence = (uint8_t)(wire[2] & 0x7fu);
	state->last_received_sequence = state->rx_sequence;
	return true;
}

InvalidReason validate_desc(const BlockDesc& v, const Binding* b) {
	InvalidReason r = check_binding(v.session_id, v.epoch, v.negotiation_id,
		v.block_serial, v.block_start_bsi, b);
	if(r != InvalidReason::VALID) return r;
	if(v.batch_count_limit == 0)
		return InvalidReason::BATCH_COUNT_LIMIT;
	if(b && v.batch_count_limit != b->max_batches)
		return InvalidReason::BATCH_COUNT_LIMIT;
	if(v.batch_index >= v.batch_count_limit) return InvalidReason::BATCH_INDEX;
	if(v.batch_span == 0) return InvalidReason::SPAN_ZERO;
	uint16_t cap = b ? b->slot_cap : MAX_SLOT_CAP;
	if(cap == 0 || cap > MAX_SLOT_CAP || v.batch_span > cap) return InvalidReason::SLOT_CAP;
	return InvalidReason::VALID;
}

bool encode_desc(const BlockDesc& v, std::vector<uint8_t>* wire, InvalidReason* reason) {
	InvalidReason r = validate_desc(v, NULL);
	if(reason) *reason = r;
	if(!wire || r != InvalidReason::VALID) return false;
	wire->clear(); wire->reserve(DESC_BODY_BYTES + INTEGRITY_TRAILER_BYTES);
	wire->push_back(VERSION); wire->push_back(v.block_mode ? 1 : 0);
	put32(wire, v.session_id); put32(wire, v.epoch); put16(wire, v.negotiation_id);
	put16(wire, v.block_serial); wire->push_back(v.block_start_bsi);
	wire->push_back(v.batch_index); wire->push_back(v.batch_count_limit); put16(wire, v.batch_span);
	wire->push_back(control_crc8(wire->data(), wire->size()));
	return true;
}

DecodeResult<BlockDesc> decode_desc(const uint8_t* p, size_t n, const Binding* b) {
	if(!p || n != DESC_BODY_BYTES + INTEGRITY_TRAILER_BYTES)
		return invalid<BlockDesc>(InvalidReason::TRUNCATED);
	if(p[DESC_BODY_BYTES] != control_crc8(p, DESC_BODY_BYTES))
		return invalid<BlockDesc>(InvalidReason::INTEGRITY_FAILURE);
	if(p[0] != VERSION) return invalid<BlockDesc>(InvalidReason::BAD_VERSION);
	if(p[1] & 0xfe) return invalid<BlockDesc>(InvalidReason::RESERVED_FLAGS);
	BlockDesc v = {};
	v.block_mode = (p[1] & 1) != 0; v.session_id = get32(p + 2); v.epoch = get32(p + 6);
	v.negotiation_id = get16(p + 10); v.block_serial = get16(p + 12);
	v.block_start_bsi = p[14]; v.batch_index = p[15]; v.batch_count_limit = p[16];
	v.batch_span = get16(p + 17);
	InvalidReason r = validate_desc(v, b);
	if(r != InvalidReason::VALID) return invalid<BlockDesc>(r);
	DecodeResult<BlockDesc> out = {}; out.valid = true; out.reason = InvalidReason::VALID; out.value = v; return out;
}

InvalidReason validate_sack(const BlockSack& v, const Binding* b) {
	InvalidReason r = check_binding(v.session_id, v.epoch, v.negotiation_id,
		v.block_serial, v.block_start_bsi, b);
	if(r != InvalidReason::VALID) return r;
	if(v.batches.empty() || v.batches.size() >= 256 || (b && v.batches.size() > b->max_batches))
		return InvalidReason::BATCH_COUNT;
	uint32_t sum = 0;
	bool seen[256] = {};
	for(size_t i = 0; i < v.batches.size(); ++i) {
		const BatchEntry& e = v.batches[i];
		uint8_t expected = (uint8_t)(v.block_start_bsi + (uint8_t)i);
		if(e.bsi != expected) return InvalidReason::BSI_DELTA;
		if(seen[e.bsi]) return InvalidReason::DUPLICATE_BSI;
		seen[e.bsi] = true;
		if(e.span_slots == 0) return InvalidReason::SPAN_ZERO;
		if(e.bitmap_offset_bits != sum) return InvalidReason::NONCONTIGUOUS_OFFSET;
		sum += e.span_slots;
		if(sum > std::numeric_limits<uint16_t>::max()) return InvalidReason::SPAN_SUM_OVERFLOW;
	}
	if(sum != v.bitmap_width_bits) return InvalidReason::BITMAP_WIDTH_MISMATCH;
	uint16_t cap = b ? b->slot_cap : MAX_SLOT_CAP;
	if(cap == 0 || cap > MAX_SLOT_CAP || sum > cap) return InvalidReason::SLOT_CAP;
	if(v.bitmap.size() != (sum + 7u) / 8u) return InvalidReason::BITMAP_WIDTH_MISMATCH;
	if((sum & 7u) && !v.bitmap.empty()) {
		uint8_t used = (uint8_t)((1u << (sum & 7u)) - 1u);
		if(v.bitmap.back() & (uint8_t)~used) return InvalidReason::NONZERO_PADDING;
	}
	return InvalidReason::VALID;
}

bool encode_sack(const BlockSack& v, std::vector<uint8_t>* wire, InvalidReason* reason) {
	InvalidReason r = validate_sack(v, NULL);
	if(reason) *reason = r;
	if(!wire || r != InvalidReason::VALID) return false;
	size_t header = SACK_FIXED_BYTES + DIRECTORY_BYTES * v.batches.size();
	size_t total = header + v.bitmap.size() + INTEGRITY_TRAILER_BYTES;
	if(header > 65535 || total > 65535) { if(reason) *reason = InvalidReason::BAD_MESSAGE_LENGTH; return false; }
	wire->clear(); wire->reserve(total);
	wire->push_back(VERSION); wire->push_back(v.final ? 1 : 0); put16(wire, (uint16_t)header);
	put16(wire, (uint16_t)total); put32(wire, v.session_id); put32(wire, v.epoch);
	put16(wire, v.negotiation_id); put16(wire, v.block_serial); wire->push_back(v.block_start_bsi);
	wire->push_back((uint8_t)v.batches.size()); put16(wire, v.bitmap_width_bits);
	for(size_t i = 0; i < v.batches.size(); ++i) {
		wire->push_back((uint8_t)i); put16(wire, v.batches[i].span_slots);
		put16(wire, v.batches[i].bitmap_offset_bits);
	}
	wire->insert(wire->end(), v.bitmap.begin(), v.bitmap.end());
	wire->push_back(control_crc8(wire->data(), wire->size()));
	return true;
}

DecodeResult<BlockSack> decode_sack(const uint8_t* p, size_t n, const DecodeOptions& o) {
	if(!p || n < SACK_FIXED_BYTES + INTEGRITY_TRAILER_BYTES)
		return invalid<BlockSack>(InvalidReason::TRUNCATED);
	if(p[0] != VERSION) return invalid<BlockSack>(InvalidReason::BAD_VERSION);
	if(p[1] & 0xfe) return invalid<BlockSack>(InvalidReason::RESERVED_FLAGS);
	uint16_t header = get16(p + 2), message = get16(p + 4);
	uint8_t count = p[19]; uint16_t width = get16(p + 20);
	if(count == 0 || count > o.binding.max_batches) return invalid<BlockSack>(InvalidReason::BATCH_COUNT);
	size_t expected_header = SACK_FIXED_BYTES + DIRECTORY_BYTES * count;
	if(header != expected_header) return invalid<BlockSack>(InvalidReason::BAD_HEADER_LENGTH);
	if(message != n) return invalid<BlockSack>(InvalidReason::BAD_MESSAGE_LENGTH);
	size_t bitmap_bytes = (width + 7u) / 8u;
	if((size_t)header + bitmap_bytes + INTEGRITY_TRAILER_BYTES != n)
		return invalid<BlockSack>(InvalidReason::BAD_MESSAGE_LENGTH);
	if(p[n - INTEGRITY_TRAILER_BYTES] != control_crc8(p, n - INTEGRITY_TRAILER_BYTES))
		return invalid<BlockSack>(InvalidReason::INTEGRITY_FAILURE);
	BlockSack v = {};
	v.final = (p[1] & 1) != 0; v.session_id = get32(p + 6); v.epoch = get32(p + 10);
	v.negotiation_id = get16(p + 14); v.block_serial = get16(p + 16);
	v.block_start_bsi = p[18]; v.bitmap_width_bits = width;
	uint32_t sum = 0; bool seen[256] = {};
	for(size_t i = 0; i < count; ++i) {
		const uint8_t* d = p + SACK_FIXED_BYTES + i * DIRECTORY_BYTES;
		if(d[0] != i) return invalid<BlockSack>(InvalidReason::BSI_DELTA);
		uint8_t bsi = (uint8_t)(v.block_start_bsi + d[0]);
		if(seen[bsi]) return invalid<BlockSack>(InvalidReason::DUPLICATE_BSI);
		seen[bsi] = true;
		uint16_t span = get16(d + 1), offset = get16(d + 3);
		if(span == 0) return invalid<BlockSack>(InvalidReason::SPAN_ZERO);
		if(offset != sum) return invalid<BlockSack>(InvalidReason::NONCONTIGUOUS_OFFSET);
		sum += span;
		if(sum > std::numeric_limits<uint16_t>::max()) return invalid<BlockSack>(InvalidReason::SPAN_SUM_OVERFLOW);
		BatchEntry e = {bsi, span, offset}; v.batches.push_back(e);
	}
	if(sum != width) return invalid<BlockSack>(InvalidReason::BITMAP_WIDTH_MISMATCH);
	if(sum > o.binding.slot_cap || o.binding.slot_cap == 0 || o.binding.slot_cap > MAX_SLOT_CAP)
		return invalid<BlockSack>(InvalidReason::SLOT_CAP);
	v.bitmap.assign(p + header, p + header + bitmap_bytes);
	if((width & 7u) && bitmap_bytes) {
		uint8_t used = (uint8_t)((1u << (width & 7u)) - 1u);
		if(v.bitmap.back() & (uint8_t)~used) return invalid<BlockSack>(InvalidReason::NONZERO_PADDING);
	}
	InvalidReason r = check_binding(v.session_id, v.epoch, v.negotiation_id,
		v.block_serial, v.block_start_bsi, &o.binding);
	if(r != InvalidReason::VALID) return invalid<BlockSack>(r);
	DecodeResult<BlockSack> out = {}; out.valid = true; out.reason = InvalidReason::VALID; out.value = v; return out;
}

bool decode_and_apply_sack(const uint8_t* p, size_t n, const DecodeOptions& o,
	AckApplier apply, void* apply_context, InvalidReason* reason) {
	DecodeResult<BlockSack> decoded = decode_sack(p, n, o);
	if(reason) *reason = decoded.reason;
	if(!decoded.valid || !apply) return false;
	return apply(decoded.value, apply_context);
}

}  // namespace l1_block
