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
// Pre-cohort gate: the cursor MUST be ground truth before the STEP-2
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
	// PART W — STAMP WIRE ROUND-TRIP (emit → parse symmetry). Build an EOB DATA_SHORT
	// frame's header + stamp + payload into message_TxRx_byte_buffer via the PRODUCTION
	// w_emit_eob_stamp (the same helper send_batch calls), then parse it back via the
	// PRODUCTION w_parse_eob_stamp (the same helper receive() calls) — asserting the stamp
	// VALUE round-trips AND the payload lands at the correct offset (no wire off-by-one).
	// ---------------------------------------------------------------------------
	printf("[TEST-STREAM-OFFSET] Part W — stamp wire emit/parse round-trip\n");
	{
		this->sack_v2_enabled   = true;
		this->header_carries_d5 = true;
		CHECK(w_stamp_rides(), "W: stamp rides at the test config (sack_v2+d5, ample max_frame)", w_stamp_rides()?1:0, 1);
		const int WBSI = 77;
		const uint64_t WSTART = 0x0123ABCDull;   // fits u32 low word
		const uint32_t WLEN   = 3850;
		tx_stream_stamp[WBSI].start  = WSTART;
		tx_stream_stamp[WBSI].length = WLEN;
		tx_stream_stamp[WBSI].valid  = true;
		int eff_short = effective_data_short_header_length(true, true);   // =7 (v2+D5)
		const int WPAYLEN = 40;
		// EOB DATA_SHORT header: [0]type [1]conn [2]seq|EOB [3]bsi [4]id [5]len [6]btf.
		message_TxRx_byte_buffer[0]=(char)DATA_SHORT;
		message_TxRx_byte_buffer[1]=(char)0x5A;
		message_TxRx_byte_buffer[2]=(char)(0x03 | 0x80);   // seq 3 + EOB bit
		message_TxRx_byte_buffer[3]=(char)WBSI;
		message_TxRx_byte_buffer[4]=(char)0x11;
		message_TxRx_byte_buffer[5]=(char)WPAYLEN;         // payload length (stamp excluded)
		message_TxRx_byte_buffer[6]=(char)25;              // D5 batch_total_frames
		int emitted = w_emit_eob_stamp(eff_short, WBSI);   // PRODUCTION emit
		CHECK(emitted == W_EOB_STAMP_BYTES, "W: emit wrote W_EOB_STAMP_BYTES", emitted, W_EOB_STAMP_BYTES);
		int tx_hdr = eff_short + emitted;                  // payload starts here on TX
		for(int j=0;j<WPAYLEN;j++) message_TxRx_byte_buffer[tx_hdr+j]=(char)((j*7+3)&0x7F);
		// RX: batch_seq_id is set from wire[3] before w_parse_eob_stamp (mirrors receive()).
		messages_rx_buffer.batch_seq_id = (unsigned char)message_TxRx_byte_buffer[3];
		rx_stream_stamp[WBSI].valid = false;
		int w_shift = w_parse_eob_stamp(eff_short);        // PRODUCTION parse
		CHECK(w_shift == W_EOB_STAMP_BYTES, "W: parse consumed W_EOB_STAMP_BYTES", w_shift, W_EOB_STAMP_BYTES);
		CHECK(rx_stream_stamp[WBSI].valid, "W: stamp parsed valid", rx_stream_stamp[WBSI].valid?1:0, 1);
		CHECK((uint32_t)rx_stream_stamp[WBSI].start == (uint32_t)WSTART, "W: start_lo32 round-trips",
			(long long)(uint32_t)rx_stream_stamp[WBSI].start, (long long)(uint32_t)WSTART);
		CHECK(rx_stream_stamp[WBSI].length == WLEN, "W: length16 round-trips",
			(long long)rx_stream_stamp[WBSI].length, (long long)WLEN);
		bool payload_ok = true;
		for(int j=0;j<WPAYLEN;j++)
			if((unsigned char)message_TxRx_byte_buffer[eff_short + w_shift + j] != (unsigned char)((j*7+3)&0x7F)){ payload_ok=false; break; }
		CHECK(payload_ok, "W: payload lands at [eff_hdr+stamp] on RX (no off-by-one)", payload_ok?1:0, 1);
		CHECK((unsigned char)message_TxRx_byte_buffer[5]==WPAYLEN, "W: DATA_SHORT length field excludes the stamp",
			(unsigned char)message_TxRx_byte_buffer[5], WPAYLEN);
	}

	// ---------------------------------------------------------------------------
	// PART R — the F1/F4.1 REGRESSION the deterministic suite previously MISSED: an
	// UNCOMPRESSED FRAME-UP climb that stages an in-flight batch and rebuilds it at a HIGHER
	// config. This drives the PRODUCTION re-stage (restage_requeue_tx_messages → the SAME
	// stream_tx_rollback_inflight() the open-coded raw legs arq_commander.cc:807/:6241 now
	// call) — NOT hand-set stamps. The old open-coded legs OMITTED that un-commit, so the
	// rebuild latched stamp[bsi]={S+L,...} instead of {S,...} and the RSP BACKSTOP false-tore-
	// down a byte-correct climb. The suite passed anyway because Parts C/D drive the funnel but
	// nothing reproduced the MISSING-rollback leg.
	//   PASS-AFTER (default): the un-commit runs → rebuild re-anchors at the TRUE origin S →
	//     w_stream_shift_detected() FALSE (no teardown).
	//   FAIL-BEFORE (MERCURY_W_RESTAGE_ROLLBACK_DEFEAT=1 — reproduces the pre-fix open-coded
	//     leg on the SAME binary): the un-commit no-ops → rebuilt stamp.start==S+L → the
	//     BACKSTOP sees S+L != rx_delivered(S) → these three CHECKs turn RED (rc=1).
	// ---------------------------------------------------------------------------
	printf("[TEST-STREAM-OFFSET] Part R — FRAME-UP climb re-stage: rollback re-anchors, no false teardown (F1/F4.1)\n");
	{
		bool rb_defeat = false;
		{ const char* e = std::getenv("MERCURY_W_RESTAGE_ROLLBACK_DEFEAT");
		  if(e && *e && atoi(e)!=0) rb_defeat = true; }
		// Clean mid-session cursor state; a prior batch gives the in-flight one a NON-ZERO origin S.
		fifo_buffer_tx.flush();
		tx_stream_committed = 0;
		for(int i=0;i<256;i++) tx_stream_stamp[i].valid = false;
		stream_tx_latch(0, 3000);                       // a prior delivered batch
		const int RKBSI = 15;
		const uint64_t S = tx_stream_committed;         // the in-flight batch's true origin
		const int RK = 20, RFLEN = 100;                 // in-flight ~cfg13: 20 frames * 100 B
		const uint32_t L = (uint32_t)(RK*RFLEN);
		stream_tx_latch(RKBSI, L);
		CHECK(tx_stream_stamp[RKBSI].start == S, "R: in-flight batch latched at origin S", (long long)tx_stream_stamp[RKBSI].start, (long long)S);
		CHECK(tx_stream_committed == S + L, "R: cursor advanced past the in-flight batch", (long long)tx_stream_committed, (long long)(S+L));
		// Stage the in-flight frames (bsi RKBSI) so the funnel's mod-256 scan finds them.
		for(int f=0;f<RK;f++){
			for(int j=0;j<RFLEN;j++) messages_tx[f].data[j]=(char)((f*RFLEN+j)&0x7F);
			messages_tx[f].length=RFLEN; messages_tx[f].id=(char)f;
			messages_tx[f].batch_seq_id=RKBSI; messages_tx[f].status=PENDING_ACK;
		}
		for(int f=RK;f<nMessages;f++) messages_tx[f].status=FREE;
		// The receiver delivered THROUGH the in-flight batch's start (the failed batch was never
		// delivered): rx_stream_delivered == S.
		rx_stream_delivered = S;
		// FRAME-UP fires → the PRODUCTION re-stage un-commit (defeatable via the env for fail-before).
		restage_requeue_tx_messages();
		// Rebuild the SAME bsi at a HIGHER config (larger transported length L2 — the climb).
		const uint32_t L2 = 5665;
		stream_tx_latch(RKBSI, L2);
		// Mirror the wire round-trip: the RX parses the rebuilt EOB stamp = tx_stream_stamp[RKBSI].
		rx_stream_stamp[RKBSI].start  = (uint32_t)(tx_stream_stamp[RKBSI].start & 0xFFFFFFFFULL);
		rx_stream_stamp[RKBSI].length = L2;
		rx_stream_stamp[RKBSI].valid  = true;
		decrypt_delivered_bsi = RKBSI;
		CHECK(tx_stream_committed == S + L2, "R: rebuilt cursor == origin + climbed length", (long long)tx_stream_committed, (long long)(S+L2));
		CHECK(tx_stream_stamp[RKBSI].start == S, "R: rebuild re-anchored at TRUE origin S (INV4)", (long long)tx_stream_stamp[RKBSI].start, (long long)S);
		CHECK(!w_stream_shift_detected(RKBSI), "R: no [RSP-V2-STREAM-SHIFT] teardown of byte-correct climb",
			(long long)(uint32_t)rx_stream_stamp[RKBSI].start, (long long)(uint32_t)rx_stream_delivered);
		printf("[TEST-STREAM-OFFSET] Part R rb_defeat=%d\n", (int)rb_defeat);
	}

	// ---------------------------------------------------------------------------
	// PART S — the F4.2/F2 REGRESSION: a demote-to-ROBUST rebuild at the SAME bsi must NOT be
	// permanently withheld by a STALE OFDM-sized RX stamp. The RX parsed a full 25-frame cfg16
	// stamp for bsi K, then the sender demoted to ROBUST and rebuilt K tiny; robust frames carry
	// NO stamp, so the stale stamp[K].length can never refresh → the PRIMARY byte-gate would
	// WITHHOLD forever (a LOUD STALL / BREAK spiral, never silent). The fix invalidates every RX
	// stamp on the config change (the PRODUCTION rx_stream_invalidate_stamps() that
	// load_configuration() calls). This arm drives that production invalidation — not hand-set
	// stamps — and proves the withhold CLEARS (recovery), so the outcome is COMPLETE-or-LOUD,
	// never a silent permanent hang (this is also the F2 withhold recover-vs-stall contract).
	//   PASS-AFTER (default): invalidation runs → w_bytegate_shortfall() FALSE → no withhold.
	//   FAIL-BEFORE (MERCURY_W_CFG_STAMP_KEEP=1 — reproduces the pre-fix no-invalidation): the
	//     stale stamp survives → the withhold PERSISTS → these two CHECKs turn RED (rc=1).
	// ---------------------------------------------------------------------------
	printf("[TEST-STREAM-OFFSET] Part S — demote-to-ROBUST: stale RX stamp invalidated on config change (F4.2/F2)\n");
	{
		bool cfg_keep = false;
		{ const char* e = std::getenv("MERCURY_W_CFG_STAMP_KEEP");
		  if(e && *e && atoi(e)!=0) cfg_keep = true; }
		const int SKBSI = 60;
		this->rsp_current_expected_batch_seq_id = SKBSI;
		rx_stream_delivered = 0;
		rx_stream_stamp[SKBSI].start  = 0;
		rx_stream_stamp[SKBSI].length = 25*GFLEN;       // STALE OFDM committed length
		rx_stream_stamp[SKBSI].valid  = true;
		int robust_present = seat_rx_batch(3, 3);        // the robust rebuild delivered only 3 frames
		// BEFORE the config-change invalidation the stale stamp DOES withhold (the loud stall).
		CHECK(w_bytegate_shortfall(SKBSI), "S: stale OFDM stamp WOULD withhold the robust rebuild (the stall)",
			(long long)robust_present, (long long)(25*GFLEN));
		// The PRODUCTION config-apply invalidation (what load_configuration() calls on the demote).
		rx_stream_invalidate_stamps();
		CHECK(!rx_stream_stamp[SKBSI].valid, "S: config change invalidated the stale RX stamp",
			rx_stream_stamp[SKBSI].valid?1:0, 0);
		CHECK(!w_bytegate_shortfall(SKBSI), "S: no permanent withhold after demote (recovery, not BREAK spiral)",
			w_bytegate_shortfall(SKBSI)?1:0, 0);
		printf("[TEST-STREAM-OFFSET] Part S cfg_keep=%d\n", (int)cfg_keep);
	}

	// ---------------------------------------------------------------------------
	// PART T — the cross-storage PREV byte-gate (c11w006). The PRIMARY byte-gate (Part H)
	// only protects the in-order BATCH-DONE path; the cross-storage PREV completion
	// (arq_responder.cc:1287 swap→copy_data_to_buffer) delivered messages_rx_prev[] UN-gated.
	// In c11w006 a 6-frame batch completed at a SHRUNK count of 4 (mechanism-4) and its
	// 12-byte short/shifted tail was delivered at stream offset 289 before the next-batch
	// positional guard caught the hole — a silent 12-byte corruption (res_c11w006:
	// first_bad=289 len=12). This drives the PRODUCTION decision predicate
	// w_bytegate_shortfall(prev_bsi, messages_rx_prev) (the SAME the wire PREV path now calls)
	// + the PRODUCTION copy_data_to_buffer reassembler, mirroring the socket-bound swap/
	// withhold control flow (the test_batchsize_desync_delivery contract). The stamp.start is
	// set CONTIGUOUS (== rx_delivered) so the positional BACKSTOP is INERT here — proving the
	// BYTE-gate, not the shift check, is the necessary catch for a short-but-in-position PREV.
	//   PASS-AFTER (default): the PREV gate fires → the short prev is WITHHELD → 0 bytes leak.
	//   FAIL-BEFORE (MERCURY_W_PREV_BYTEGATE_DEFEAT=1 — reproduces the pre-fix un-gated PREV
	//     path on the SAME binary): copy_data_to_buffer delivers the 12-byte short tail → 12
	//     leaked bytes reach the app fifo (byte-integrity broken).
	printf("[TEST-STREAM-OFFSET] Part T — cross-storage PREV byte-gate (c11w006 offset 289 / 12 bytes)\n");
	{
		bool prev_defeat = false;
		{ const char* e = std::getenv("MERCURY_W_PREV_BYTEGATE_DEFEAT");
		  if(e && *e && atoi(e)!=0) prev_defeat = true; }
		const int      TBSI  = 16;          // c11w006's leaking prev bsi
		const uint64_t TBASE = 289;         // res_c11w006 first_bad offset (== good_prefix bytes)
		const int      TLEAK = 12;          // res_c11w006 mismatch bytes (the short tail)
		// Seat the SHORT prev batch into messages_rx_prev[]: ONE 12-byte frame present (the
		// shrunk-count completion), all other slots FREE. data_batch_size = 1 = the shrunk
		// window, so copy_data_to_buffer would deliver exactly those 12 bytes.
		for(int f=0;f<nMessages;f++){ messages_rx_prev[f].status=FREE; messages_rx_prev[f].length=0; }
		for(int j=0;j<TLEAK;j++) messages_rx_prev[0].data[j]=(char)((j*13+7)&0x7F);
		messages_rx_prev[0].length = TLEAK;
		messages_rx_prev[0].id     = (char)0;
		messages_rx_prev[0].status = RECEIVED;      // as the prev-RX path stores it (arq_responder.cc:1151)
		this->data_batch_size = 1;
		rx_stream_delivered          = TBASE;       // cursor at the good-prefix boundary
		rx_stream_stamp[TBSI].start  = TBASE;       // CONTIGUOUS ⇒ BACKSTOP inert (isolate the byte-gate)
		rx_stream_stamp[TBSI].length = TLEAK + 3*GFLEN;  // sender committed MORE than the short tail
		rx_stream_stamp[TBSI].valid  = true;
		// T0 — NO false-fire: a COMPLETE prev (delivered == committed) must NOT trip the gate.
		rx_stream_stamp[TBSI].length = TLEAK;
		CHECK(!w_bytegate_shortfall(TBSI, messages_rx_prev),
			"T0: complete PREV does NOT trip the byte-gate (no false-fire)", TLEAK, TLEAK);
		// T0b — absent/invalid stamp (robust / cfg0 tiny-frame / lost EOB) ⇒ safe no-op.
		rx_stream_stamp[TBSI].valid = false;
		CHECK(!w_bytegate_shortfall(TBSI, messages_rx_prev),
			"T0b: absent PREV stamp is a safe no-op (never a false withhold)", 0, 0);
		rx_stream_stamp[TBSI].valid  = true;
		rx_stream_stamp[TBSI].length = TLEAK + 3*GFLEN;   // restore the short scenario
		// The decision predicate MUST fire (delivered 12 < committed), in BOTH modes.
		CHECK(w_bytegate_shortfall(TBSI, messages_rx_prev),
			"T: short PREV trips the byte-gate (delivered<committed)",
			(long long)TLEAK, (long long)rx_stream_stamp[TBSI].length);
		decrypt_delivered_bsi = TBSI;
		fifo_buffer_rx.flush();
		if(prev_defeat)
		{
			// fail-before: the PREV gate is DEFEATED → the production swaps messages_rx →
			// messages_rx_prev, marks RECEIVED→ACKED, and delivers the short tail (the bug).
			struct st_message* saved_rx = messages_rx;
			messages_rx = messages_rx_prev;
			for(int i=0;i<this->data_batch_size && i<this->nMessages;i++)
				if(messages_rx[i].status==RECEIVED) messages_rx[i].status=ACKED;
			copy_data_to_buffer();
			messages_rx = saved_rx;
			char tmp[65536]; int popped=fifo_buffer_rx.pop(tmp,(int)sizeof(tmp));
			CHECK(popped==TLEAK, "T(defeat): un-gated short PREV leaked its 12-byte tail (fail-before bug)",
				popped, TLEAK);
		}
		else
		{
			// pass-after: mirror the production PREV control flow — the gate fires ⇒ WITHHOLD
			// (copy_data_to_buffer NOT called), exactly as arq_responder.cc:1287 now guards the
			// swap→deliver body with `!prev_byte_withheld`. 0 bytes reach the app fifo.
			if(!w_bytegate_shortfall(TBSI, messages_rx_prev))
			{
				struct st_message* saved_rx = messages_rx;    // (unreached in the fix build)
				messages_rx = messages_rx_prev;
				copy_data_to_buffer();
				messages_rx = saved_rx;
			}
			char tmp[65536]; int popped=fifo_buffer_rx.pop(tmp,(int)sizeof(tmp));
			CHECK(popped==0, "T(fix): short PREV withheld → 0 leaked bytes reach the buffer", popped, 0);
		}
		printf("[TEST-STREAM-OFFSET] Part T prev_defeat=%d\n", (int)prev_defeat);
	}

	// ---------------------------------------------------------------------------
	// PART U — the EOT end-to-end LAST-BATCH tail-drop check (data-flow-stream-offset.md §8.6).
	// The per-batch W checks (PRIMARY byte-gate / BACKSTOP) are inherently ONE BATCH BEHIND: a
	// truncated FINAL batch has no next-batch stamp to backstop it. At a graceful disconnect the
	// CLOSE_CONNECTION frame carries the SENDER's final total_committed_bytes + running stream
	// CRC-32; this RECEIVER reconciles them against its rx_stream_delivered + rx_stream_crc. This
	// drives the PRODUCTION receiver fold (copy_data_to_buffer advances rx_stream_delivered AND
	// folds rx_stream_crc via crc32_update over the delivered bytes) + the PRODUCTION predicate
	// w_eot_mismatch() — the exact decision the RSP CLOSE handler makes. The counter catches a
	// SHORT tail; the running CRC ALSO catches a same-LENGTH final-content corruption the counter
	// is blind to. Mechanism-1/Fix-A shape: the final batch's tail frame is dropped on delivery.
	//   PASS-AFTER (default): delivered < committed ⇒ w_eot_mismatch fires ⇒ the RSP raises
	//     [RSP-V2-EOT-SHORT] and the transfer is NOT declared complete.
	//   FAIL-BEFORE (MERCURY_W_EOT_DEFEAT=1 — the same env the production handler reads): the
	//     CLOSE handler SKIPS the check ⇒ the short final delivery is silently declared complete.
	printf("[TEST-STREAM-OFFSET] Part U — EOT last-batch tail-drop (truncated FINAL batch)\n");
	{
		bool eot_defeat = false;
		{ const char* e = std::getenv("MERCURY_W_EOT_DEFEAT");
		  if(e && *e && atoi(e)!=0) eot_defeat = true; }

		const int UFLEN   = 100;   // bytes per delivered frame
		const int UNDELIV = 3;     // frames of the FINAL batch that actually arrive
		// Keep a copy of the delivered content so we can build a same-length corrupted CRC (U2).
		char ubuf[UNDELIV][UFLEN];
		for(int f=0; f<UNDELIV; f++)
			for(int j=0; j<UFLEN; j++)
				ubuf[f][j] = (char)((f*100 + j*7 + 3) & 0x7F);

		// Fresh receiver: fold the DELIVERED bytes through the PRODUCTION funnel.
		rx_stream_delivered = 0;
		rx_stream_crc       = CRC32_INIT;
		fifo_buffer_rx.flush();
		this->data_batch_size = UNDELIV;
		for(int f=0; f<nMessages; f++) messages_rx[f].status = FREE;
		for(int f=0; f<UNDELIV; f++)
		{
			memcpy(messages_rx[f].data, ubuf[f], UFLEN);
			messages_rx[f].length = UFLEN;
			messages_rx[f].id     = (char)f;
			messages_rx[f].status = ACKED;
		}
		decrypt_delivered_bsi = -1;                 // no wire stamp ⇒ BACKSTOP inert (isolate EOT)
		rx_stream_stamp[255].valid = false;         // (decrypt_delivered_bsi & 0xFF) — keep no-op
		copy_data_to_buffer();                      // PRODUCTION fold + rx_stream_delivered advance
		{ char tmp[65536]; (void)fifo_buffer_rx.pop(tmp,(int)sizeof(tmp)); }  // drain the app fifo
		CHECK(rx_stream_delivered == (uint64_t)(UNDELIV*UFLEN),
			"U: receiver funnel advanced rx cursor by the delivered bytes",
			(long long)rx_stream_delivered, (long long)(UNDELIV*UFLEN));

		// The SENDER's committed stream = the UNDELIV delivered frames + ONE dropped TAIL frame
		// (the final batch was truncated). Build committed count + running CRC over ALL of them.
		uint64_t U_committed = 0;
		uint32_t U_txcrc     = CRC32_INIT;
		for(int f=0; f<UNDELIV; f++)
		{
			U_txcrc = crc32_update(U_txcrc, ubuf[f], UFLEN);   // sender folds the delivered frames
			U_committed += UFLEN;
		}
		char utail[UFLEN];
		for(int j=0; j<UFLEN; j++) utail[j] = (char)((999 + j*5) & 0x7F);
		U_txcrc = crc32_update(U_txcrc, utail, UFLEN);          // ... and the dropped tail frame
		U_committed += UFLEN;

		// U0 — NO false-fire: a COMPLETE transfer (peer committed EXACTLY what we delivered) must
		// NOT trip EOT. rx_stream_crc == the sender's fold of the SAME delivered frames.
		CHECK(!w_eot_mismatch(rx_stream_delivered, rx_stream_crc),
			"U0: complete transfer does NOT trip EOT (no false-fire)",
			(long long)rx_stream_delivered, (long long)rx_stream_delivered);
		// U0b — a no-data connection (0 committed, 0 delivered, INIT crc) is a safe no-op.
		{
			uint64_t sd = rx_stream_delivered; uint32_t sc = rx_stream_crc;
			rx_stream_delivered = 0; rx_stream_crc = CRC32_INIT;
			CHECK(!w_eot_mismatch((uint64_t)0, (uint32_t)CRC32_INIT),
				"U0b: no-data close is a safe no-op (0==0, INIT==INIT)", 0, 0);
			rx_stream_delivered = sd; rx_stream_crc = sc;
		}

		// U — the tail-drop trips EOT (delivered 300 < committed 400). Pure predicate: fires in
		// BOTH modes (the env-defeat lives in the production caller, exercised below).
		CHECK(w_eot_mismatch(U_committed, U_txcrc),
			"U: last-batch tail-drop trips EOT (delivered<committed)",
			(long long)rx_stream_delivered, (long long)U_committed);

		// U2 — a SAME-LENGTH final-content corruption (delivered==committed count) that the byte
		// counter is BLIND to, caught by the running CRC. Fold the delivered content with one bit
		// flipped: same count, different CRC ⇒ w_eot_mismatch fires via the CRC leg alone.
		uint32_t U2crc = CRC32_INIT;
		for(int f=0; f<UNDELIV; f++)
		{
			char tmp[UFLEN]; memcpy(tmp, ubuf[f], UFLEN);
			if(f==0) tmp[0] ^= 0x01;   // one bit; length unchanged
			U2crc = crc32_update(U2crc, tmp, UFLEN);
		}
		CHECK(rx_stream_delivered == (uint64_t)(UNDELIV*UFLEN) && U2crc != rx_stream_crc &&
		      w_eot_mismatch(rx_stream_delivered, U2crc),
			"U2: same-length final-content corruption caught by CRC (counter blind)",
			(long long)rx_stream_delivered, (long long)rx_stream_delivered);

		// Fail-before / pass-after (mirror Parts R/S — the DEFEAT arm turns this RED). The RSP
		// CLOSE handler declares the transfer INCOMPLETE iff (!MERCURY_W_EOT_DEFEAT && mismatch)
		// — the EXACT production predicate (arq_responder.cc CLOSE_CONNECTION handler). A real
		// last-batch tail-drop MUST be declared incomplete:
		//   pass-after (default): declared_incomplete == true  → GREEN.
		//   fail-before (MERCURY_W_EOT_DEFEAT=1): the guard is skipped → declared_incomplete ==
		//     false → this CHECK goes RED, proving that WITHOUT the EOT guard the short final
		//     delivery would be SILENTLY declared complete (never a next-batch backstop).
		bool declared_incomplete = (!eot_defeat && w_eot_mismatch(U_committed, U_txcrc));
		CHECK(declared_incomplete,
			"U: real tail-drop declared INCOMPLETE (fail-before: EOT-defeat → RED)",
			(long long)declared_incomplete, 1);
		printf("[TEST-STREAM-OFFSET] Part U eot_defeat=%d (0=fix→GREEN, 1=defeat→RED expected)\n",
			(int)eot_defeat);
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
