#ifndef INC_DATALINK_LAYER_L1_BLOCK_CODEC_H_
#define INC_DATALINK_LAYER_L1_BLOCK_CODEC_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "datalink_layer/datalink_defines.h"

namespace l1_block {

enum : uint8_t {
	BLOCK_DESC_TYPE = BLOCK_DESC,
	BLOCK_SACK_TYPE = BLOCK_SACK,
	BLOCK_OFFER_TYPE = BLOCK_OFFER,
	BLOCK_ACCEPT_TYPE = BLOCK_ACCEPT,
	BLOCK_COMMIT_TYPE = BLOCK_COMMIT,
	BLOCK_COMMIT_ACK_TYPE = BLOCK_COMMIT_ACK
};

static const uint8_t VERSION = 1;
// Eight retained legacy-sized batches (96 slots each). Stage-3 extends the
// ownership horizon, not the per-batch ARQ slot limit.
static const uint16_t MAX_SLOT_CAP = 8 * 96;
// The codec owns the payload after the standard [type, connection, sequence]
// header. Every encoded body ends in the same CRC-8 used by SACK_RSP; the
// standard header remains protected by the enclosing data-frame CRC-16.
static const size_t CONTROL_HEADER_BYTES = 3;
static const size_t INTEGRITY_TRAILER_BYTES = 1;

enum class InvalidReason {
	VALID = 0,
	TRUNCATED,
	BAD_VERSION,
	RESERVED_FLAGS,
	BAD_HEADER_LENGTH,
	BAD_MESSAGE_LENGTH,
	SESSION_MISMATCH,
	EPOCH_ZERO,
	EPOCH_MISMATCH,
	NEGOTIATION_MISMATCH,
	BLOCK_MISMATCH,
	START_BSI_MISMATCH,
	BATCH_COUNT,
	BATCH_INDEX,
	BATCH_COUNT_LIMIT,
	SPAN_ZERO,
	SLOT_CAP,
	BSI_DELTA,
	DUPLICATE_BSI,
	NONCONTIGUOUS_OFFSET,
	SPAN_SUM_OVERFLOW,
	BITMAP_WIDTH_MISMATCH,
	NONZERO_PADDING,
	INTEGRITY_FAILURE
};

struct Binding {
	uint32_t session_id;
	uint32_t epoch;
	uint16_t negotiation_id;
	uint16_t block_serial;
	uint8_t block_start_bsi;
	uint8_t max_batches;
	uint16_t slot_cap;
};

struct BlockDesc {
	bool block_mode;
	uint32_t session_id;
	uint32_t epoch;
	uint16_t negotiation_id;
	uint16_t block_serial;
	uint8_t block_start_bsi;
	uint8_t batch_index;
	uint8_t batch_count_limit;
	uint16_t batch_span;
};

struct BatchEntry {
	uint8_t bsi;
	uint16_t span_slots;
	uint16_t bitmap_offset_bits;
};

struct BlockSack {
	bool final;
	uint32_t session_id;
	uint32_t epoch;
	uint16_t negotiation_id;
	uint16_t block_serial;
	uint8_t block_start_bsi;
	std::vector<BatchEntry> batches;
	uint16_t bitmap_width_bits;
	std::vector<uint8_t> bitmap;
};

struct DecodeOptions {
	Binding binding;
};

typedef bool (*AckApplier)(const BlockSack& value, void* context);

struct LegacyParserAckState {
	int rx_status;
	uint8_t rx_type;
	uint8_t rx_sequence;
	uint8_t last_received_sequence;
};

template <typename T>
struct DecodeResult {
	bool valid;
	InvalidReason reason;
	T value;
};

InvalidReason validate_desc(const BlockDesc& value, const Binding* binding);
uint8_t control_crc8(const uint8_t* covered, size_t covered_size);
bool is_block_message_type(uint8_t type);
bool seat_legacy_receive_prefix(const uint8_t* wire, size_t size,
	LegacyParserAckState* state);
bool encode_desc(const BlockDesc& value, std::vector<uint8_t>* wire,
	InvalidReason* reason = NULL);
DecodeResult<BlockDesc> decode_desc(const uint8_t* wire, size_t size,
	const Binding* binding = NULL);

InvalidReason validate_sack(const BlockSack& value, const Binding* binding);
bool encode_sack(const BlockSack& value, std::vector<uint8_t>* wire,
	InvalidReason* reason = NULL);
DecodeResult<BlockSack> decode_sack(const uint8_t* wire, size_t size,
	const DecodeOptions& options);
bool decode_and_apply_sack(const uint8_t* wire, size_t size,
	const DecodeOptions& options, AckApplier apply, void* apply_context,
	InvalidReason* reason = NULL);

}  // namespace l1_block

#endif
