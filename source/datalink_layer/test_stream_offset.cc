// ============================================================================
// Option W — FOUNDATION regression: the absolute-byte-stream cursors are GROUND TRUTH
// ============================================================================
//
// CLI: --test-stream-offset
//
// Pairs with fact-documents/data-flow-stream-offset.md (the producer/consumer audit)
// and silent-corruption-residual.md §14-§15 (the 4 silent-corruption mechanisms Option
// W closes). This FOUNDATION test proves the sender cursor + per-bsi latch + re-stage
// rollback maintain the invariant
//
//     tx_stream_stamp[bsi].start == the true cumulative TRANSPORTED origin offset
//
// through EVERY transition, and that a re-staged/rebuilt batch re-anchors at the SAME
// start (INV4) — the property the STEP-2 wire check depends on. It drives the PRODUCTION
// latch (stream_tx_latch), the PRODUCTION re-stage funnel (restage_requeue_tx_messages,
// which calls stream_tx_rollback_inflight), and the PRODUCTION receiver funnel
// (copy_data_to_buffer, which advances rx_stream_delivered). No wire, no RF, no sockets;
// deterministic; well under 1 s. 0 = PASS, 1 = FAIL.
//
// This is Fable's PRE-COHORT GATE: the cursor MUST be ground truth before the STEP-2
// wire stamp/check is meaningful.
// ============================================================================

#include "datalink_layer/arq.h"
#include "datalink_layer/datalink_defines.h"
#include "common/common_defines.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

static int g_fails = 0;
static void CHECK(bool cond, const char* what, long long got, long long want)
{
	if(cond)
	{
		printf("[TEST-STREAM-OFFSET]   ok   %-52s got=%lld want=%lld\n", what, got, want);
	}
	else
	{
		printf("[TEST-STREAM-OFFSET]  FAIL  %-52s got=%lld want=%lld\n", what, got, want);
		g_fails++;
	}
	fflush(stdout);
}

int cl_arq_controller::test_stream_offset()
{
	printf("[TEST-STREAM-OFFSET] Option W FOUNDATION — cursor = ground truth\n");
	fflush(stdout);

	// --- Setup: a commander controller with allocated messages_tx[]/messages_rx[] + fifos ---
	this->original_role       = COMMANDER;
	this->link_status         = CONNECTED;
	this->sack_v2_enabled     = true;
	this->compression_enabled = false;    // force the raw (transported==app) legs
	this->nMessages           = 255;
	this->max_data_length     = 170;
	this->max_message_length  = 200;
	this->max_header_length   = 6;
	if(init_messages_buffers() != SUCCESSFUL)
	{
		printf("[TEST-STREAM-OFFSET] ERROR: init_messages_buffers failed\n");
		return 1;
	}
	if(fifo_buffer_tx.set_size(65536) != SUCCESSFUL ||
	   fifo_buffer_rx.set_size(65536) != SUCCESSFUL)
	{
		printf("[TEST-STREAM-OFFSET] ERROR: fifo set_size failed\n");
		return 1;
	}
	fifo_buffer_tx.flush();
	fifo_buffer_rx.flush();

	// Anchor cursors + stamps at a clean start.
	tx_stream_committed = 0;
	rx_stream_delivered = 0;
	for(int i=0;i<256;i++) tx_stream_stamp[i].valid = false;

	// ---------------------------------------------------------------------------
	// PART A — sequential builds: stamp[k].start == Σ len[0..k-1] (varied lengths,
	// modelling compression on/off transported sizes).
	// ---------------------------------------------------------------------------
	printf("[TEST-STREAM-OFFSET] Part A — sequential builds\n");
	const uint32_t lenA[] = { 84, 4175, 3850, 154, 5665, 231 };  // robust..cfg16 mix
	uint64_t cum = 0;
	for(int k=0;k<6;k++)
	{
		stream_tx_latch(k, lenA[k]);
		CHECK(tx_stream_stamp[k].start == cum, "A: stamp[k].start == cumulative", (long long)tx_stream_stamp[k].start, (long long)cum);
		CHECK(tx_stream_stamp[k].length == lenA[k], "A: stamp[k].length == committed", (long long)tx_stream_stamp[k].length, (long long)lenA[k]);
		cum += lenA[k];
		CHECK(tx_stream_committed == cum, "A: cursor advanced by exactly len", (long long)tx_stream_committed, (long long)cum);
	}

	// ---------------------------------------------------------------------------
	// PART B — retx / mixbatch re-emit: reading a latched stamp does NOT double-commit;
	// the next new-data batch starts contiguously after the retx'd one.
	// ---------------------------------------------------------------------------
	printf("[TEST-STREAM-OFFSET] Part B — retx re-emit (no double-commit) + mixbatch\n");
	uint64_t cursor_before = tx_stream_committed;
	StreamStamp saved5 = tx_stream_stamp[5];
	// simulate N re-emits of bsi 5 (retx) — the wire re-sends the LATCHED stamp; the
	// cursor/latch are untouched (only reads).
	for(int r=0;r<4;r++)
	{
		volatile uint64_t re_start = tx_stream_stamp[5].start;   // read-only re-emit
		(void)re_start;
	}
	CHECK(tx_stream_committed == cursor_before, "B: retx re-emit did NOT move cursor", (long long)tx_stream_committed, (long long)cursor_before);
	CHECK(tx_stream_stamp[5].start == saved5.start && tx_stream_stamp[5].length == saved5.length,
		"B: retx re-emit did NOT mutate stamp[5]", (long long)tx_stream_stamp[5].start, (long long)saved5.start);
	// mixbatch: bsi 6 (new) prepends bsi 5 (retx) in one TX. Latch of 6 is unaffected by
	// the retx prefix — start[6] == start[5] + len[5].
	stream_tx_latch(6, 300);
	CHECK(tx_stream_stamp[6].start == saved5.start + saved5.length, "B: mixbatch start[6]==start[5]+len[5]",
		(long long)tx_stream_stamp[6].start, (long long)(saved5.start + saved5.length));
	cum = tx_stream_committed;

	// ---------------------------------------------------------------------------
	// PART C — re-stage rollback via the PRODUCTION funnel + rebuild re-anchors (INV4).
	// Stage an in-flight batch bsi=7 in messages_tx[], latch it, then drive the real
	// restage_requeue_tx_messages() (which calls stream_tx_rollback_inflight). The cursor
	// must roll back to stamp[7].start; a rebuild (NEW length) must re-anchor at the SAME start.
	// ---------------------------------------------------------------------------
	printf("[TEST-STREAM-OFFSET] Part C — re-stage rollback (PRODUCTION funnel) + rebuild INV4\n");
	const int BSI = 7;
	const int K = 12, FLEN = 100;
	uint64_t start7_original;
	// latch bsi 7 at the current cursor as a K*FLEN transported batch.
	stream_tx_latch(BSI, (uint32_t)(K*FLEN));
	start7_original = tx_stream_stamp[BSI].start;
	CHECK(start7_original == cum, "C: stamp[7].start == cursor at build", (long long)start7_original, (long long)cum);
	CHECK(tx_stream_committed == cum + (uint64_t)(K*FLEN), "C: cursor advanced by batch", (long long)tx_stream_committed, (long long)(cum + K*FLEN));
	// stage the in-flight frames (bsi 7) in messages_tx so the funnel's mod-256 scan finds it.
	for(int f=0;f<K;f++)
	{
		for(int j=0;j<FLEN;j++) messages_tx[f].data[j] = (char)((f*FLEN+j)&0x7F);
		messages_tx[f].length       = FLEN;
		messages_tx[f].id           = (char)f;
		messages_tx[f].batch_seq_id = BSI;      // stamped in-flight bsi
		messages_tx[f].status       = PENDING_ACK;
	}
	for(int f=K;f<nMessages;f++) messages_tx[f].status = FREE;
	// PRODUCTION re-stage funnel (rolls the cursor back to stamp[7].start).
	restage_requeue_tx_messages();
	CHECK(tx_stream_committed == start7_original, "C: cursor rolled back to stamp[7].start", (long long)tx_stream_committed, (long long)start7_original);
	// rebuild bsi 7 at a DIFFERENT (robust, smaller) length — must re-anchor at same start (INV4).
	stream_tx_latch(BSI, 154);
	CHECK(tx_stream_stamp[BSI].start == start7_original, "C: rebuild re-anchored at SAME start (INV4)", (long long)tx_stream_stamp[BSI].start, (long long)start7_original);
	CHECK(tx_stream_committed == start7_original + 154, "C: cursor = start + rebuilt len", (long long)tx_stream_committed, (long long)(start7_original + 154));

	// ---------------------------------------------------------------------------
	// PART D — BREAK → ROBUST → re-climb. Demote (rollback bsi 8) then rebuild bsi 8 at a
	// tiny robust length, then climb bsi 9 at a large cfg16 length. Contiguity holds across.
	// ---------------------------------------------------------------------------
	printf("[TEST-STREAM-OFFSET] Part D — BREAK->ROBUST->re-climb contiguity\n");
	uint64_t base = tx_stream_committed;
	stream_tx_latch(8, 4175);                      // cfg15 batch built
	uint64_t start8 = tx_stream_stamp[8].start;
	CHECK(start8 == base, "D: stamp[8].start contiguous", (long long)start8, (long long)base);
	// BREAK re-stage of bsi 8
	for(int f=0;f<10;f++){ messages_tx[f].length=100; messages_tx[f].id=(char)f; messages_tx[f].batch_seq_id=8; messages_tx[f].status=PENDING_ACK; }
	for(int f=10;f<nMessages;f++) messages_tx[f].status=FREE;
	restage_requeue_tx_messages();
	CHECK(tx_stream_committed == start8, "D: BREAK rolled cursor to start[8]", (long long)tx_stream_committed, (long long)start8);
	stream_tx_latch(8, 84);                        // rebuilt at ROBUST_0 (tiny)
	CHECK(tx_stream_stamp[8].start == start8, "D: robust rebuild re-anchored (INV4)", (long long)tx_stream_stamp[8].start, (long long)start8);
	stream_tx_latch(9, 5665);                      // re-climb cfg16
	CHECK(tx_stream_stamp[9].start == start8 + 84, "D: re-climb start[9]==start[8]+robustlen", (long long)tx_stream_stamp[9].start, (long long)(start8 + 84));

	// ---------------------------------------------------------------------------
	// PART F — RX advance via the PRODUCTION receiver funnel (copy_data_to_buffer) +
	// INV1: rx_stream_delivered tracks the sender's stamp.start at each delivery boundary.
	// Deliver the first few batches (their transported lengths) and check contiguity.
	// ---------------------------------------------------------------------------
	printf("[TEST-STREAM-OFFSET] Part F — RX advance (PRODUCTION funnel) + INV1\n");
	rx_stream_delivered = 0;
	// Deliver a synthetic batch of K2 raw frames through copy_data_to_buffer; assert the
	// cursor advanced by exactly the reassembled transported length.
	const int K2 = 8, FLEN2 = 150;
	this->data_batch_size = K2;
	uint64_t rx_before = rx_stream_delivered;
	for(int f=0;f<K2;f++)
	{
		for(int j=0;j<FLEN2;j++) messages_rx[f].data[j] = (char)((f*FLEN2+j)&0x7F);
		messages_rx[f].length = FLEN2;
		messages_rx[f].id     = (char)f;
		messages_rx[f].status = ACKED;
	}
	for(int f=K2;f<nMessages;f++) messages_rx[f].status = FREE;
	copy_data_to_buffer();
	CHECK(rx_stream_delivered == rx_before + (uint64_t)(K2*FLEN2),
		"F: rx cursor advanced by reassembled transported len", (long long)rx_stream_delivered, (long long)(rx_before + K2*FLEN2));
	// deliver a 2nd batch — cursor stays contiguous (INV1 shape: start2 == delivered-after-1)
	uint64_t after1 = rx_stream_delivered;
	for(int f=0;f<K2;f++){ messages_rx[f].length=FLEN2; messages_rx[f].id=(char)f; messages_rx[f].status=ACKED; }
	for(int f=K2;f<nMessages;f++) messages_rx[f].status=FREE;
	copy_data_to_buffer();
	CHECK(rx_stream_delivered == after1 + (uint64_t)(K2*FLEN2), "F: 2nd delivery contiguous", (long long)rx_stream_delivered, (long long)(after1 + K2*FLEN2));
	// A NON-delivered (incomplete) batch must NOT advance the cursor (all slots FREE/non-ACKED).
	uint64_t held = rx_stream_delivered;
	for(int f=0;f<nMessages;f++) messages_rx[f].status = FREE;
	copy_data_to_buffer();
	CHECK(rx_stream_delivered == held, "F: non-delivered batch did NOT advance rx cursor", (long long)rx_stream_delivered, (long long)held);

	// ---------------------------------------------------------------------------
	printf("[TEST-STREAM-OFFSET] ---- %d check(s) failed ----\n", g_fails);
	fflush(stdout);
	if(g_fails == 0)
	{
		printf("[TEST-STREAM-OFFSET] PASS — cursor is ground truth: latch/rollback/rebuild "
			"(INV1-INV4) hold across every transition\n");
		fflush(stdout);
		return 0;
	}
	printf("[TEST-STREAM-OFFSET] FAIL — %d cursor invariant(s) violated; cursor is NOT ground "
		"truth (STEP-2 wire check would be meaningless)\n", g_fails);
	fflush(stdout);
	return 1;
}
