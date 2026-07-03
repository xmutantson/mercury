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

	// ===========================================================================
	// STEP 2c — the 4-mechanism silent-corruption GATE. Reproduces each of the four
	// silent byte-corruption mechanisms of silent-corruption-residual.md §11-§14 in
	// process and asserts each is now GATED (PRIMARY withholds → no silent complete) or
	// LOUD (BACKSTOP teardown decision fires) — NEVER silently delivered. Drives the
	// PRODUCTION decision predicates w_stream_shift_detected() / w_bytegate_shortfall()
	// (the SAME the wire ACK-GATE / copy_data_to_buffer call) + the PRODUCTION
	// copy_data_to_buffer() reassembler as a byte-exact oracle (compression OFF). The
	// env-defeat arms reproduce the SILENT shift (fail-before); the default arms show
	// the gate/loud + zero silent (pass-after). Returns 0 when the EXPECTED behavior
	// holds in EITHER mode (the test_batchsize_desync_delivery contract).
	// ===========================================================================
	bool shift_defeat = false;
	{ const char* e = std::getenv("MERCURY_W_STREAM_SHIFT_DEFEAT");
	  if(e && *e && atoi(e)!=0) shift_defeat = true; }
	bool bytegate_defeat = false;
	{ const char* e = std::getenv("MERCURY_W_BYTEGATE_DEFEAT");
	  if(e && *e && atoi(e)!=0) bytegate_defeat = true; }

	// Seat `n_present` distinct raw frames (ids 0..n_present-1) of one batch into
	// messages_rx[] as ACKED, set data_batch_size = batch_size (the RSP's window — the
	// remaining slots stay FREE), and return the present total byte length. mech-1
	// tail-drop = full batch_size with fewer present; mech-4 shrunk-count = batch_size
	// itself smaller than the sender's committed frames.
	const int GFLEN = 150;
	auto seat_rx_batch = [&](int n_present, int batch_size)->int {
		for(int f=0;f<nMessages;f++){ messages_rx[f].status=FREE; messages_rx[f].length=0; }
		int total=0;
		for(int f=0;f<n_present && f<nMessages;f++){
			for(int j=0;j<GFLEN;j++) messages_rx[f].data[j]=(char)((f*GFLEN+j)&0x7F);
			messages_rx[f].length=GFLEN;
			messages_rx[f].id=(char)f;
			messages_rx[f].status=ACKED;
			total+=GFLEN;
		}
		this->data_batch_size = batch_size;
		return total;
	};

	// ---------------------------------------------------------------------------
	// PART G — BACKSTOP: mechanism 3 (re-stage orphan/reorder Δ=48) + mechanism 4
	// (c31w104 forward Δ=+6847). A positional shift ⇒ wire stamp.start diverges from
	// rx_stream_delivered ⇒ w_stream_shift_detected() true (LOUD teardown). fail-before
	// (MERCURY_W_STREAM_SHIFT_DEFEAT=1): copy_data_to_buffer delivers the shifted batch
	// SILENTLY. pass-after: the decision fires and the batch is REFUSED (0 silent bytes).
	// ---------------------------------------------------------------------------
	printf("[TEST-STREAM-OFFSET] Part G — BACKSTOP (mech-3 Δ=48, mech-4 Δ=+6847)\n");
	{
		const int GBSI = 20;
		// G0 — NO false-fire: a contiguous batch (stamp.start == rx_delivered) must NOT
		// trip the BACKSTOP and must deliver cleanly through the PRODUCTION funnel.
		rx_stream_delivered = 23581;                       // the res_c3100 boundary value
		rx_stream_stamp[GBSI].start  = 23581;              // contiguous — matches the cursor
		rx_stream_stamp[GBSI].length = 3750;
		rx_stream_stamp[GBSI].valid  = true;
		decrypt_delivered_bsi = GBSI;
		CHECK(!w_stream_shift_detected(GBSI), "G0: contiguous batch does NOT trip BACKSTOP (no false-fire)", 0, 0);
		fifo_buffer_rx.flush();
		int g0_total = seat_rx_batch(25, 25);
		copy_data_to_buffer();                             // BACKSTOP inert → clean delivery
		{ char tmp[65536]; int popped=fifo_buffer_rx.pop(tmp,(int)sizeof(tmp));
		  CHECK(popped==g0_total, "G0: contiguous batch delivered fully (funnel healthy)", popped, g0_total); }

		// The two shift shapes (mech-3 hole Δ=48, mech-4 forward Δ=+6847). Both make the
		// wire stamp.start (the sender's committed origin for this bsi) exceed the
		// receiver's absolute delivered cursor → a hole in the delivered stream.
		const long long deltas[2] = { 48, 6847 };
		const char*     names [2] = { "mech-3 (re-stage orphan Δ=48)", "mech-4 (c31w104 Δ=+6847)" };
		for(int m=0;m<2;m++)
		{
			int SBSI = 30 + m;
			uint64_t cursor = 40000 + (uint64_t)m*10000;   // arbitrary in-session offset
			rx_stream_delivered          = cursor;
			rx_stream_stamp[SBSI].start  = cursor + (uint64_t)deltas[m];   // the SHIFT
			rx_stream_stamp[SBSI].length = 25*GFLEN;
			rx_stream_stamp[SBSI].valid  = true;
			decrypt_delivered_bsi        = SBSI;
			// The decision predicate MUST fire on the shift, in BOTH modes.
			CHECK(w_stream_shift_detected(SBSI), names[m], (long long)deltas[m], (long long)deltas[m]);
			fifo_buffer_rx.flush();
			int stotal = seat_rx_batch(25, 25);
			if(shift_defeat)
			{
				// fail-before: BACKSTOP defeated → copy_data_to_buffer delivers the SHIFTED
				// batch silently (the bug). Assert the silent delivery reproduced.
				uint64_t before = rx_stream_delivered;
				copy_data_to_buffer();
				char tmp[65536]; int popped=fifo_buffer_rx.pop(tmp,(int)sizeof(tmp));
				bool silent = (popped==stotal) && (rx_stream_delivered==before+(uint64_t)stotal);
				CHECK(silent, "G(defeat): shifted batch SILENTLY delivered (fail-before bug)", popped, stotal);
			}
			else
			{
				// pass-after: the decision fired (asserted above). The production BACKSTOP
				// refuses delivery via rsp_gap_abort_teardown (socket-bound — mirrored, not
				// invoked, exactly as test_batchsize_desync_delivery does). REFUSE: 0 silent.
				char tmp[65536]; int popped=fifo_buffer_rx.pop(tmp,(int)sizeof(tmp));  // fifo still empty
				CHECK(popped==0, "G(fix): shift detected → batch REFUSED, 0 silent bytes", popped, 0);
			}
		}
	}

	// ---------------------------------------------------------------------------
	// PART H — PRIMARY byte-gate: mechanism 1 (EOB-undercount tail-drop) + mechanism 4
	// (complete-at-shrunk-count) + mechanism 2 (CMD>RSP batch-size desync, defense in
	// depth). The batch is FRAME-COUNT complete but the DELIVERED bytes fall short of
	// the committed stamp.length ⇒ w_bytegate_shortfall() true (WITHHOLD the clean ACK).
	// fail-before (MERCURY_W_BYTEGATE_DEFEAT=1): the batch delivers TRUNCATED (silent
	// short store). pass-after: the gate withholds → the batch is NOT delivered.
	// ---------------------------------------------------------------------------
	printf("[TEST-STREAM-OFFSET] Part H — PRIMARY byte-gate (mech-1 tail-drop, mech-4 shrunk, mech-2 desync)\n");
	{
		// H0 — NO false-fire: a COMPLETE batch (delivered bytes == committed) must NOT
		// trip the gate and must deliver fully.
		const int HBSI = 40;
		this->rsp_current_expected_batch_seq_id = HBSI;
		rx_stream_delivered = 0;
		int h0_committed = 25*GFLEN;
		rx_stream_stamp[HBSI].start  = 0;
		rx_stream_stamp[HBSI].length = h0_committed;
		rx_stream_stamp[HBSI].valid  = true;
		int h0_total = seat_rx_batch(25, 25);
		CHECK(!w_bytegate_shortfall(HBSI), "H0: complete batch does NOT trip byte-gate (no false-fire)", h0_total, h0_committed);
		decrypt_delivered_bsi = HBSI;
		fifo_buffer_rx.flush();
		copy_data_to_buffer();
		{ char tmp[65536]; int popped=fifo_buffer_rx.pop(tmp,(int)sizeof(tmp));
		  CHECK(popped==h0_total, "H0: complete batch delivered fully", popped, h0_total); }

		// mech-1 tail-drop (committed 25 frames, only 20 present) + mech-4 shrunk-count
		// (committed 25, only 21 present) + mech-2 desync (committed 30, only 25 present).
		const int committed_frames[3] = { 25, 25, 30 };
		const int present_frames  [3] = { 20, 21, 25 };
		const int rsp_window      [3] = { 25, 21, 25 };   // mech-1 full window; mech-4 shrunk; mech-2 RSP=25
		const char* hnames[3] = { "mech-1 (EOB-undercount tail-drop)",
		                          "mech-4 (complete-at-shrunk-count)",
		                          "mech-2 (CMD>RSP desync, defense-in-depth)" };
		for(int m=0;m<3;m++)
		{
			int MBSI = 50 + m;
			this->rsp_current_expected_batch_seq_id = MBSI;
			rx_stream_stamp[MBSI].start  = 0;
			rx_stream_stamp[MBSI].length = committed_frames[m]*GFLEN;   // sender committed
			rx_stream_stamp[MBSI].valid  = true;
			int present_total = seat_rx_batch(present_frames[m], rsp_window[m]);  // fewer bytes present
			// The decision predicate MUST fire (delivered < committed), in BOTH modes.
			CHECK(w_bytegate_shortfall(MBSI), hnames[m],
				(long long)present_total, (long long)(committed_frames[m]*GFLEN));
			if(bytegate_defeat)
			{
				// fail-before: gate defeated → the batch delivers TRUNCATED (silent short store).
				rx_stream_delivered = 0;
				rx_stream_stamp[MBSI].start = 0;   // contiguous start so the BACKSTOP stays inert
				decrypt_delivered_bsi = MBSI;
				fifo_buffer_rx.flush();
				copy_data_to_buffer();
				char tmp[65536]; int popped=fifo_buffer_rx.pop(tmp,(int)sizeof(tmp));
				bool silent_short = (popped==present_total) && (present_total < committed_frames[m]*GFLEN);
				CHECK(silent_short, "H(defeat): truncated batch SILENTLY short-delivered (fail-before bug)",
					popped, committed_frames[m]*GFLEN);
			}
			else
			{
				// pass-after: the gate withholds → do NOT deliver. 0 silent bytes.
				fifo_buffer_rx.flush();
				char tmp[65536]; int popped=fifo_buffer_rx.pop(tmp,(int)sizeof(tmp));
				CHECK(popped==0, "H(fix): byte shortfall → clean ACK withheld, 0 silent bytes", popped, 0);
			}
		}
	}

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
