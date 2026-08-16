#ifndef MERCURY_L1_BLOCK_ACK_H
#define MERCURY_L1_BLOCK_ACK_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include "datalink_layer/l1_block_codec.h"
#include "datalink_layer/l1_tx_journal.h"

namespace l1_block {

enum class DispatchDisposition {
	IGNORED = 0,
	FENCED,
	LEGACY_SEATED,
	BLOCK_FRAME_ACCEPTED,
	BLOCK_ACK_APPLIED,
	INVALID
};

// Session-scoped Stage-3 policy and wire dispatcher. All enablement is derived
// from the process gate plus authenticated handshake bytes; callers cannot
// force the negotiated state directly.
class BlockAckRuntime {
public:
	BlockAckRuntime();

	uint8_t advertised_capability(uint8_t legacy_capability) const;
	void begin_session(uint8_t connection_id, uint32_t session_id,
		uint32_t epoch, uint16_t negotiation_id);
	void reset_session();
	void complete_handshake(uint8_t local_capability, uint8_t peer_capability,
		bool authenticated_echo);

	bool gate_enabled() const { return gate_enabled_; }
	bool enabled() const { return negotiated_; }
	uint8_t aggregate_batches() const { return aggregate_batches_; }
	bool last_ack_all_received() const { return last_ack_all_received_; }
	std::size_t last_ack_batch_count() const { return last_ack_batch_count_; }
	std::size_t last_ack_slot_count() const { return last_ack_slot_count_; }

	// Commander-side burst ownership. A new batch remains journal-owned while
	// the short inter-batch receive window is silent. The Nth batch, or a tail
	// commit for a short final group, waits for the aggregate response.
	void note_transmitted_batch(uint8_t bsi);
	bool intermediate_silence_expected() const;
	bool build_tail_commit(std::vector<uint8_t>* wire);

	DispatchDisposition dispatch_received_frame(const uint8_t* wire,
		std::size_t size, LegacyParserAckState* legacy_state,
		mercury::L1TxJournal* journal);

	// Production responder chokepoint. Each call records the bitmap for one
	// completed forward batch. It emits one complete BLOCK_SACK wire frame on
	// the Nth batch (or on final=true), otherwise returns false and no bytes.
	bool observe_received_batch(uint8_t bsi, const std::vector<bool>& received,
		bool final, std::vector<uint8_t>* wire);
	bool flush_received_batches(std::vector<uint8_t>* wire);
	bool take_flush_request() {
		const bool requested = flush_requested_;
		flush_requested_ = false;
		return requested;
	}

private:
	bool gate_enabled_;
	bool negotiated_;
	uint8_t aggregate_batches_;
	uint8_t connection_id_;
	uint32_t session_id_;
	uint32_t epoch_;
	uint16_t negotiation_id_;
	uint16_t tx_block_serial_;
	uint16_t rx_block_serial_;
	uint8_t pending_start_bsi_;
	std::vector<BatchEntry> pending_batches_;
	std::vector<uint8_t> pending_bitmap_;
	uint16_t pending_width_;
	bool last_ack_all_received_;
	std::size_t last_ack_batch_count_;
	std::size_t last_ack_slot_count_;
	uint8_t tx_pending_start_bsi_;
	uint8_t tx_pending_batches_;
	bool tx_tail_commit_sent_;
	bool flush_requested_;
};

bool feature_gate_enabled();
uint8_t capability_advertise_bit();

}  // namespace l1_block

#endif
