#include "datalink_layer/l1_block_codec.h"

#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace l1_block;

static int failures;
static volatile uint64_t journal_bits;

#define CHECK(x, name) do { if(!(x)) { std::printf("FAIL: %s\n", name); ++failures; } } while(0)

static Binding binding(uint8_t start, uint8_t n = 8, uint16_t cap = 96) {
	Binding b = {0x11223344u, 0x01020304u, 0x5566u, 0x7788u, start, n, cap}; return b;
}

static BlockSack sample(uint8_t start, const std::vector<uint16_t>& spans, bool final) {
	BlockSack s = {}; s.final = final; s.session_id = 0x11223344u; s.epoch = 0x01020304u;
	s.negotiation_id = 0x5566u; s.block_serial = 0x7788u; s.block_start_bsi = start;
	uint16_t off = 0;
	for(size_t i = 0; i < spans.size(); ++i) {
		BatchEntry e = {(uint8_t)(start + (uint8_t)i), spans[i], off}; s.batches.push_back(e); off += spans[i];
	}
	s.bitmap_width_bits = off; s.bitmap.assign((off + 7u) / 8u, 0);
	for(uint16_t i = 0; i < off; ++i) if((i % 3) != 1) s.bitmap[i / 8] |= (uint8_t)(1u << (i & 7));
	return s;
}

static std::vector<uint8_t> encode_crc(const BlockSack& s) {
	std::vector<uint8_t> w; CHECK(encode_sack(s, &w), "encode sample"); return w;
}

static DecodeOptions options(const Binding& b) {
	DecodeOptions o = {b}; return o;
}

static bool apply_to_journal(const BlockSack& sack, void*) {
	uint64_t applied = 0;
	for(uint16_t bit = 0; bit < sack.bitmap_width_bits && bit < 64; ++bit)
		if(sack.bitmap[bit / 8] & (uint8_t)(1u << (bit & 7)))
			applied |= (uint64_t)1u << bit;
	journal_bits = applied;
	return true;
}

static void golden_vectors() {
	BlockDesc d = {true, 0x11223344u, 0x01020304u, 0x5566u, 0x7788u, 254, 2, 8, 9};
	const uint8_t expected_desc_body[] = {1,1,0x11,0x22,0x33,0x44,1,2,3,4,0x55,0x66,
		0x77,0x88,0xfe,2,8,0,9};
	std::vector<uint8_t> dw; CHECK(encode_desc(d, &dw), "BLOCK_DESC encode");
	CHECK(dw.size() == sizeof(expected_desc_body) + INTEGRITY_TRAILER_BYTES,
		"BLOCK_DESC golden length");
	CHECK(std::vector<uint8_t>(dw.begin(), dw.end() - 1) ==
		std::vector<uint8_t>(expected_desc_body,
			expected_desc_body + sizeof(expected_desc_body)), "BLOCK_DESC golden bytes");
	CHECK(dw.back() == control_crc8(expected_desc_body, sizeof(expected_desc_body)),
		"BLOCK_DESC mandatory CRC8 trailer");
	CHECK(decode_desc(dw.data(), dw.size(), NULL).valid, "BLOCK_DESC golden decode");
	std::vector<uint8_t> corrupt_desc = dw;
	corrupt_desc.back() ^= 1;
	DecodeResult<BlockDesc> bad_desc_crc = decode_desc(corrupt_desc.data(),
		corrupt_desc.size(), NULL);
	CHECK(!bad_desc_crc.valid && bad_desc_crc.reason == InvalidReason::INTEGRITY_FAILURE,
		"BLOCK_DESC rejects corrupt mandatory CRC8 trailer");
	Binding desc_binding = binding(254);
	CHECK(decode_desc(dw.data(), dw.size(), &desc_binding).valid,
		"BLOCK_DESC matches committed batch limit");
	dw[16] = 7;
	dw.back() = control_crc8(dw.data(), dw.size() - 1);
	DecodeResult<BlockDesc> bad_limit = decode_desc(dw.data(), dw.size(), &desc_binding);
	CHECK(!bad_limit.valid && bad_limit.reason == InvalidReason::BATCH_COUNT_LIMIT,
		"BLOCK_DESC rejects batch limit different from commitment");

	BlockSack one = sample(250, std::vector<uint16_t>(1, 5), true);
	std::vector<uint8_t> w1 = encode_crc(one);
	const uint8_t expected_one_prefix[] = {1,1,0,27,0,29,0x11,0x22,0x33,0x44,1,2,3,4,
		0x55,0x66,0x77,0x88,0xfa,1,0,5,0,0,5,0,0,0x0d};
	CHECK(w1.size() == sizeof(expected_one_prefix) + 1, "one-batch golden length");
	CHECK(std::vector<uint8_t>(w1.begin(), w1.end()-1) ==
		std::vector<uint8_t>(expected_one_prefix, expected_one_prefix + sizeof(expected_one_prefix)), "one-batch golden bytes");
	CHECK(w1.back() == control_crc8(expected_one_prefix, sizeof(expected_one_prefix)),
		"one-batch mandatory CRC8 trailer");
	CHECK(decode_sack(w1.data(), w1.size(), options(binding(250))).valid, "one-batch round trip");

	BlockSack two = sample(42, std::vector<uint16_t>{3, 7}, false);
	std::vector<uint8_t> w2 = encode_crc(two);
	CHECK(decode_sack(w2.data(), w2.size(), options(binding(42))).valid, "two variable-span batches");
	CHECK(w2[22] == 0 && w2[27] == 1 && w2[31] == 3, "two-batch canonical directory");
	journal_bits = 0;
	CHECK(decode_and_apply_sack(w2.data(), w2.size(), options(binding(42)),
		apply_to_journal, NULL), "valid ACK reaches journal apply seam");
	CHECK(journal_bits != 0, "valid ACK changes journal bits through apply seam");

	BlockSack eight = sample(254, std::vector<uint16_t>{12,12,12,12,12,12,12,12}, true);
	std::vector<uint8_t> w8 = encode_crc(eight);
	DecodeResult<BlockSack> r8 = decode_sack(w8.data(), w8.size(), options(binding(254)));
	CHECK(r8.valid && r8.value.batches.size() == 8 && r8.value.batches[2].bsi == 0,
		"N=8 max-cap BSI wrap FINAL");
	BlockSack partial = sample(80, std::vector<uint16_t>{25,25,11}, true);
	std::vector<uint8_t> wp = encode_crc(partial);
	CHECK(decode_sack(wp.data(), wp.size(), options(binding(80))).valid, "partial final batch");
}

static void expect_bad(std::vector<uint8_t> w, DecodeOptions o, InvalidReason why, const char* name) {
	uint64_t before = journal_bits;
	InvalidReason reason = InvalidReason::VALID;
	bool applied = decode_and_apply_sack(w.data(), w.size(), o,
		apply_to_journal, NULL, &reason);
	CHECK(!applied && reason == why, name);
	CHECK(journal_bits == before, "invalid changes zero journal bits at apply seam");
}
static void resign(std::vector<uint8_t>* w) {
	(*w)[w->size()-1] = control_crc8(w->data(), w->size()-1);
}

static void adversarial() {
	BlockSack s = sample(254, std::vector<uint16_t>{4,6}, false);
	std::vector<uint8_t> good = encode_crc(s); DecodeOptions o = options(binding(254));
	std::vector<uint8_t> w;
	w=good; w[0]=2; resign(&w); expect_bad(w,o,InvalidReason::BAD_VERSION,"bad version");
	w=good; w[1]=0x80; resign(&w); expect_bad(w,o,InvalidReason::RESERVED_FLAGS,"reserved flags");
	w=good; w[3]++; resign(&w); expect_bad(w,o,InvalidReason::BAD_HEADER_LENGTH,"header length mismatch");
	w=good; w[5]++; resign(&w); expect_bad(w,o,InvalidReason::BAD_MESSAGE_LENGTH,"message length mismatch");
	w=good; w[19]=0; resign(&w); expect_bad(w,o,InvalidReason::BATCH_COUNT,"zero batch count");
	w=good; w[27]=2; resign(&w); expect_bad(w,o,InvalidReason::BSI_DELTA,"bsi delta not index");
	w=good; w[30]=3; resign(&w); expect_bad(w,o,InvalidReason::NONCONTIGUOUS_OFFSET,"overlapping offset");
	w=good; w[30]=5; resign(&w); expect_bad(w,o,InvalidReason::NONCONTIGUOUS_OFFSET,"gapped offset");
	w=good; w[29]++; resign(&w); expect_bad(w,o,InvalidReason::BITMAP_WIDTH_MISMATCH,"bitmap width mismatch");
	BlockSack pad = sample(7, std::vector<uint16_t>{3}, false); w=encode_crc(pad); w[w.size()-2]|=0x80; resign(&w);
	expect_bad(w,options(binding(7)),InvalidReason::NONZERO_PADDING,"nonzero pad bits");
	w=good; w[23]=0xff; w[24]=0xff; w[28]=0; w[29]=1; w[30]=0xff; w[31]=0xff; resign(&w);
	expect_bad(w,o,InvalidReason::SPAN_SUM_OVERFLOW,"span sum overflow");
	for(size_t n = 0; n < good.size(); ++n) {
		DecodeResult<BlockSack> r = decode_sack(good.data(), n, o); CHECK(!r.valid, "truncation boundary rejected");
	}
	DecodeOptions mismatch=o; mismatch.binding.session_id++; expect_bad(good,mismatch,InvalidReason::SESSION_MISMATCH,"session mismatch");
	w=good; w[10]=w[11]=w[12]=w[13]=0; resign(&w); expect_bad(w,o,InvalidReason::EPOCH_ZERO,"epoch zero");
	mismatch=o; mismatch.binding.epoch++; expect_bad(good,mismatch,InvalidReason::EPOCH_MISMATCH,"epoch mismatch");
	mismatch=o; mismatch.binding.negotiation_id++; expect_bad(good,mismatch,InvalidReason::NEGOTIATION_MISMATCH,"negotiation mismatch");
	mismatch=o; mismatch.binding.block_serial++; expect_bad(good,mismatch,InvalidReason::BLOCK_MISMATCH,"block replay mismatch");
	mismatch=o; mismatch.binding.block_start_bsi++; expect_bad(good,mismatch,InvalidReason::START_BSI_MISMATCH,"start BSI mismatch");
	mismatch=o; mismatch.binding.slot_cap=9; expect_bad(good,mismatch,InvalidReason::SLOT_CAP,"committed slot cap exceeded");
	w=good; w.back()^=1; expect_bad(w,o,InvalidReason::INTEGRITY_FAILURE,"integrity trailer hook");
}

static uint32_t rng_state = 0x6d657263u;
static uint32_t rnd() { rng_state ^= rng_state << 13; rng_state ^= rng_state >> 17; rng_state ^= rng_state << 5; return rng_state; }

static void fuzz() {
	const int iterations = 120000;
	for(int it=0; it<iterations; ++it) {
		uint8_t count=(uint8_t)(1+rnd()%8); std::vector<uint16_t> spans; uint16_t left=96;
		for(uint8_t i=0;i<count;i++) { uint16_t max=(uint16_t)(left-(count-i-1)); uint16_t v=(uint16_t)(1+rnd()%max); spans.push_back(v); left-=v; }
		uint8_t start=(uint8_t)rnd(); BlockSack s=sample(start,spans,(rnd()&1)!=0); std::vector<uint8_t> w=encode_crc(s);
		int changes=1+(int)(rnd()%4); for(int j=0;j<changes;j++) w[rnd()%w.size()]^=(uint8_t)(1u<<(rnd()%8));
		DecodeOptions o=options(binding(start)); DecodeResult<BlockSack> r=decode_sack(w.data(),w.size(),o);
		if(r.valid) {
			std::vector<uint8_t> canon=encode_crc(r.value);
			CHECK(canon==w,"VALID mutation equals recomputed canonical form");
		}
	}
	std::printf("fuzz: %d deterministic structure-aware mutations passed (seed=0x6d657263)\n", iterations);
}

static void production_type_fence() {
	CHECK(BLOCK_DESC_TYPE == 0x46 && BLOCK_SACK_TYPE == 0x47 &&
		BLOCK_OFFER_TYPE == 0x48 && BLOCK_ACCEPT_TYPE == 0x49 &&
		BLOCK_COMMIT_TYPE == 0x4a && BLOCK_COMMIT_ACK_TYPE == 0x4b,
		"registered block message type assignments");

	const uint8_t block_prefix[] = {BLOCK_SACK_TYPE, 0x5a, VERSION};
	LegacyParserAckState state = {FREE, NONE, 0x6c, 0x6d};
	LegacyParserAckState before = state;
	bool seated = seat_legacy_receive_prefix(block_prefix, sizeof(block_prefix), &state);
	CHECK(!seated, "block-shaped prefix fenced before legacy receive parser");
	CHECK(state.rx_status == before.rx_status && state.rx_type == before.rx_type &&
		state.rx_sequence == before.rx_sequence &&
		state.last_received_sequence == before.last_received_sequence,
		"fenced block prefix changes zero legacy ACK state");

	const uint8_t legacy_prefix[] = {SACK_RSP, 0x5a, 0x35};
	seated = seat_legacy_receive_prefix(legacy_prefix, sizeof(legacy_prefix), &state);
	CHECK(seated && state.rx_status == RECEIVED && state.rx_type == SACK_RSP &&
		state.rx_sequence == 0x35 && state.last_received_sequence == 0x35,
		"legacy SACK prefix still seats production receive state");
}

int main() {
	journal_bits=0x0123456789abcdefULL;
	golden_vectors(); adversarial(); fuzz(); production_type_fence();
	std::printf("L1 block codec: %s (%d failures)\n", failures ? "FAIL" : "PASS", failures);
	return failures ? 1 : 0;
}
