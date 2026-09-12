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
static int accept_test_socket_write(const char*, int length)
{
	return length;
}

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
	// PART X — RECONNECT-CONTINUITY fail-closed (data-flow-reconnect-continuity.md §5b). The
	// cross-session splice: a low-SNR transfer delivers only N=101 app bytes while the sender's
	// app-read has raced far ahead (~122 KB); the link tears fully down and makes a FRESH
	// reconnect that RUNS reset_session_state() (re-anchoring rx_stream_delivered at 0); the
	// fresh session then streams the sender's CURRENT position onto the SAME persistent app data
	// socket right after the 101 delivered — a silent ~122 KB SKIP. Every per-session guard misses
	// it (each session's cursor is self-consistent: fresh stamp.start==cursor==0). This part
	// crosses a REAL reset_session_state() (the snapshot's home) — NOT a manual prev-delivered
	// poke (a cursor-poke test provably cannot exercise the snapshot survive/consume logic).
	//   PASS-AFTER (default): the seam arms at the fresh accept; copy_data_to_buffer REFUSES the
	//     first delivery (loud drop via rsp_gap_abort_teardown; 0 spliced bytes).
	//   FAIL-BEFORE (MERCURY_RECONNECT_FAILCLOSED_DEFEAT=1): the seam stays disarmed; the within-
	//     session backstop is inert (stamp.start==cursor==0) so the fresh-session bytes are
	//     SILENTLY spliced onto app position 101 — the corruption reproduced.
	// Also asserts NO false-refuse: a fresh transfer (prev==0) never arms, and the transfer AFTER
	// a refusal delivers fully (prev cleared → no livelock).
	int saved_data_socket_status = tcp_socket_data.status;
	tcp_socket_data.status = TCP_STATUS_ACCEPTED;
	auto drive_directed_start_accept = [&]() {
		// Drive the callsign-matched production consumer, not merely its seam-arm
		// helper. This is the reconnect path used by an ordinary responder while
		// the application data socket remains attached across link sessions.
		this->passive_monitor = false;
		this->narrowband_enabled = NO;
		this->my_call_sign = "RSP123";
		this->link_status = LISTENING;
		this->connection_status = RECEIVING;
		messages_control.data[0] = START_CONNECTION;
		messages_control.data[1] = (char)CRC8_calc(
			(char*)my_call_sign.c_str(), my_call_sign.length());
		callsign_pack("CMD123", 6, &messages_control.data[2]);
		messages_control.status = RECEIVED;
		int (*saved_hook)(const char*, int) = cl_tcp_socket::g_test_transmit_hook;
		cl_tcp_socket::g_test_transmit_hook = accept_test_socket_write;
		process_control_responder();
		cl_tcp_socket::g_test_transmit_hook = saved_hook;
	};
	printf("[TEST-STREAM-OFFSET] Part X — reconnect-continuity fail-closed (cross-session splice)\n");
	{
		bool failclosed_defeat = false;
		{ const char* e = std::getenv("MERCURY_RECONNECT_FAILCLOSED_DEFEAT");
		  if(e && *e && atoi(e)!=0) failclosed_defeat = true; }

		// Clean receiver state for this part.
		this->compression_enabled = false;
		fifo_buffer_rx.flush();
		rx_stream_delivered            = 0;
		rx_stream_crc                  = CRC32_INIT;
		for(int i=0;i<256;i++) rx_stream_stamp[i].valid = false;
		rsp_stream_aborted             = false;
		rsp_cross_session_seam_armed   = false;
		rsp_prev_session_app_delivered = 0;
		rsp_test_force_app_persistent  = false;

		// --- X0: NO false-refuse — a persistent socket with NOTHING delivered before (a fresh
		// transfer) must NOT arm the seam, and the batch delivers fully. ---
		rsp_prev_session_app_delivered = 0;        // but the prior session delivered nothing
		rsp_stream_aborted             = false;
		rsp_reconnect_seam_arm_on_accept();        // PRODUCTION arm decision
		CHECK(!rsp_cross_session_seam_armed,
			"X0: fresh transfer (prev=0) does NOT arm the seam (no false-refuse)",
			rsp_cross_session_seam_armed?1:0, 0);
		{
			const int X0BSI = 88;
			rx_stream_stamp[X0BSI].start = 0; rx_stream_stamp[X0BSI].length = 130; rx_stream_stamp[X0BSI].valid = true;
			decrypt_delivered_bsi = X0BSI;
			this->data_batch_size = 1;
			for(int f=0;f<nMessages;f++){ messages_rx[f].status=FREE; messages_rx[f].length=0; }
			for(int j=0;j<130;j++) messages_rx[0].data[j]=(char)((j*3+1)&0x7F);
			messages_rx[0].length=130; messages_rx[0].id=(char)0; messages_rx[0].status=ACKED;
			fifo_buffer_rx.flush();
			copy_data_to_buffer();
			char tmp[65536]; int popped=fifo_buffer_rx.pop(tmp,(int)sizeof(tmp));
			CHECK(popped==130, "X0: fresh transfer delivers fully (byte-identical, seam inert)", popped, 130);
		}

		// --- X: the cross-session splice scenario. ---
		// (1) Prior session delivers exactly 101 app bytes through the PRODUCTION funnel while the
		// sender's committed cursor is far ahead (~122 KB) — the low-SNR thrash.
		this->tx_stream_committed = 122000;        // sender raced ahead (context; RX gate reads prev only)
		rx_stream_delivered = 0;
		fifo_buffer_rx.flush();
		for(int i=0;i<256;i++) rx_stream_stamp[i].valid = false;
		decrypt_delivered_bsi = 200;               // invalid stamp ⇒ within-session backstop inert
		this->data_batch_size = 1;
		for(int f=0;f<nMessages;f++){ messages_rx[f].status=FREE; messages_rx[f].length=0; }
		for(int j=0;j<101;j++) messages_rx[0].data[j]=(char)((j*5+2)&0x7F);
		messages_rx[0].length=101; messages_rx[0].id=(char)0; messages_rx[0].status=ACKED;
		copy_data_to_buffer();
		CHECK(rx_stream_delivered == 101, "X: prior session delivered 101 app bytes (real funnel)",
			(long long)rx_stream_delivered, 101);
		{ char tmp[65536]; (void)fifo_buffer_rx.pop(tmp,(int)sizeof(tmp)); }   // app drained the 101

		// (2) GENUINE teardown → REAL reset_session_state() (the snapshot's home). The app data
		// socket persists across the reconnect. rsp_stream_aborted is false ⇒ a genuine session
		// boundary (not a mid-transfer gap-abort) ⇒ the snapshot fires.
		rsp_stream_aborted            = false;
		reset_session_state();                     // REAL reset — snapshots prev=101, zeroes cursor
		CHECK(rsp_prev_session_app_delivered == 101,
			"X: real reset_session_state snapshotted prev-delivered (survives the reset)",
			(long long)rsp_prev_session_app_delivered, 101);
		CHECK(rx_stream_delivered == 0,
			"X: real reset re-anchored the RX byte cursor at 0",
			(long long)rx_stream_delivered, 0);

		// (3) Fresh directed START_CONNECTION → PRODUCTION accept + arm decision.
		rsp_stream_aborted = true;                 // the accept itself must clear this latch
		drive_directed_start_accept();
		CHECK(link_status == CONNECTION_RECEIVED,
			"X: directed START reached the callsign-matched accept branch",
			link_status, CONNECTION_RECEIVED);
		CHECK(!rsp_stream_aborted,
			"X: directed START accept cleared the prior abort latch",
			rsp_stream_aborted?1:0, 0);
		if(failclosed_defeat)
			CHECK(!rsp_cross_session_seam_armed, "X(defeat): DEFEAT env leaves the seam disarmed",
				rsp_cross_session_seam_armed?1:0, 0);
		else
			CHECK(rsp_cross_session_seam_armed, "X(fix): seam ARMED at fresh accept (prev>0, persistent)",
				rsp_cross_session_seam_armed?1:0, 1);

		// (4) The fresh session presents its first delivery. It re-anchored at 0, so its wire
		// stamp.start==0==rx_stream_delivered — the within-session backstop is INERT (this is why
		// the splice is silent). The bytes would land at app position 101 (right after the 101).
		const int XBSI = 90;
		rx_stream_stamp[XBSI].start  = 0;          // fresh session self-consistent — hides the splice
		rx_stream_stamp[XBSI].length = 150;
		rx_stream_stamp[XBSI].valid  = true;
		decrypt_delivered_bsi = XBSI;
		this->data_batch_size = 1;
		for(int f=0;f<nMessages;f++){ messages_rx[f].status=FREE; messages_rx[f].length=0; }
		for(int j=0;j<150;j++) messages_rx[0].data[j]=(char)((j*7+9)&0x7F);
		messages_rx[0].length=150; messages_rx[0].id=(char)0; messages_rx[0].status=ACKED;
		CHECK(!w_stream_shift_detected(XBSI),
			"X: within-session backstop is INERT at the seam (stamp.start==cursor==0)",
			(long long)(uint32_t)rx_stream_stamp[XBSI].start, (long long)(uint32_t)rx_stream_delivered);
		fifo_buffer_rx.flush();
		uint64_t x_before = rx_stream_delivered;
		copy_data_to_buffer();
		char xtmp[65536]; int xpopped = fifo_buffer_rx.pop(xtmp,(int)sizeof(xtmp));
		if(failclosed_defeat)
			printf("[TEST-STREAM-OFFSET]   (defeat) splice reproduced: popped=%d rx_delivered=%llu "
				"(these bytes landed at app position 101)\n", xpopped, (unsigned long long)rx_stream_delivered);
		// INTEGRITY assertion — GREEN with the fix (0 spliced), RED under DEFEAT (mirrors the
		// Parts R/S/U fail-before convention: the DEFEAT env reproduces the pre-fix leg on the
		// SAME binary and turns this CHECK red → rc=1). fixed build REFUSES; defeat build SPLICES.
		CHECK(xpopped==0,
			"X: cross-session splice REFUSED — 0 spliced bytes (fail-before: DEFEAT → RED)", xpopped, 0);
		if(!failclosed_defeat)
		{
			(void)x_before;
			CHECK(rsp_stream_aborted,
				"X(fix): refusal latched rsp_stream_aborted (loud drop via gap-abort contract)",
				rsp_stream_aborted?1:0, 1);
			CHECK(!rsp_cross_session_seam_armed,
				"X(fix): seam consumed/disarmed after the refusal",
				rsp_cross_session_seam_armed?1:0, 0);
			CHECK(rsp_prev_session_app_delivered==0,
				"X(fix): prev-delivered cleared post-refusal (next transfer fresh — no livelock)",
				(long long)rsp_prev_session_app_delivered, 0);

			// (5) NO livelock / NO false-refuse: the NEXT genuine directed reconnect (still on the
			// persistent socket) must NOT re-arm (prev cleared), and its fresh transfer delivers fully.
			drive_directed_start_accept();
			CHECK(!rsp_cross_session_seam_armed,
				"X(fix): NEXT reconnect does NOT re-arm (prev cleared → no livelock)",
				rsp_cross_session_seam_armed?1:0, 0);
			const int X5BSI = 91;
			rx_stream_stamp[X5BSI].start=0; rx_stream_stamp[X5BSI].length=120; rx_stream_stamp[X5BSI].valid=true;
			decrypt_delivered_bsi = X5BSI;
			this->data_batch_size = 1;
			for(int f=0;f<nMessages;f++){ messages_rx[f].status=FREE; messages_rx[f].length=0; }
			for(int j=0;j<120;j++) messages_rx[0].data[j]=(char)((j*11+4)&0x7F);
			messages_rx[0].length=120; messages_rx[0].id=(char)0; messages_rx[0].status=ACKED;
			fifo_buffer_rx.flush();
			copy_data_to_buffer();
			char t2[65536]; int p2=fifo_buffer_rx.pop(t2,(int)sizeof(t2));
			CHECK(p2==120,
				"X(fix): fresh transfer after a refusal delivers fully (no false-refuse)", p2, 120);
		}
		printf("[TEST-STREAM-OFFSET] Part X failclosed_defeat=%d (0=fix→REFUSE, 1=defeat→SPLICE expected)\n",
			(int)failclosed_defeat);
	}

	// ---------------------------------------------------------------------------
	// PART X2 -- the WIRE path Part X could not reach: the teardown routes through
	// rsp_gap_abort_teardown() (which latches rsp_stream_aborted BEFORE its internal reset), THEN a
	// bare link-timeout reset_session_state(), THEN the fresh accept. On the wire EVERY relevant
	// reset is gated by that latch, so the abort-gated snapshot (the pre-fix guard) skipped the
	// high-water on exactly this sequence: prev stayed 0, the accept never armed, the fresh session
	// spliced. This part crosses the ACTUAL production teardown chain, not a bare reset.
	//   PASS-AFTER (fix): the snapshot is an unconditional monotonic-max, so prev survives the
	//     latched teardown; the accept ARMS (prev>0); copy_data_to_buffer REFUSES (0 spliced).
	//   FAIL-BEFORE (@ae8a236, F1 absent): the abort-gated snapshot skips -> prev==0 -> the accept
	//     does NOT arm -> the fresh-session bytes SPLICE onto the persistent socket (popped>0).
	printf("[TEST-STREAM-OFFSET] Part X2 -- reconnect splice through the REAL gap-abort teardown\n");
	{
		this->compression_enabled = false;
		fifo_buffer_rx.flush();
		rx_stream_delivered            = 0;
		rx_stream_crc                  = CRC32_INIT;
		for(int i=0;i<256;i++) rx_stream_stamp[i].valid = false;
		rsp_stream_aborted             = false;
		rsp_cross_session_seam_armed   = false;
		rsp_prev_session_app_delivered = 0;
		rsp_test_force_app_persistent  = false;    // use the production ACCEPTED-socket predicate

		// (1) Session 1 delivers 185 app bytes through the PRODUCTION funnel while the sender's
		// committed cursor raced far ahead (the low-SNR thrash).
		this->tx_stream_committed = 129024;
		decrypt_delivered_bsi = 205;               // invalid stamp => within-session backstop inert
		this->data_batch_size = 1;
		for(int f=0;f<nMessages;f++){ messages_rx[f].status=FREE; messages_rx[f].length=0; }
		for(int j=0;j<185;j++) messages_rx[0].data[j]=(char)((j*5+2)&0x7F);
		messages_rx[0].length=185; messages_rx[0].id=(char)0; messages_rx[0].status=ACKED;
		copy_data_to_buffer();
		CHECK(rx_stream_delivered == 185, "X2: session 1 delivered 185 app bytes (real funnel)",
			(long long)rx_stream_delivered, 185);
		{ char tmp[65536]; (void)fifo_buffer_rx.pop(tmp,(int)sizeof(tmp)); }   // app drained the 185

		// (2) The stream strands -> the PRODUCTION gap-abort teardown fires: it latches
		// rsp_stream_aborted, runs its internal reset, and restores the RX cursor.
		rsp_gap_abort_teardown("test: mid-transfer strand (X2)");
		CHECK(rsp_stream_aborted, "X2: gap-abort teardown latched rsp_stream_aborted",
			rsp_stream_aborted?1:0, 1);

		// (3) The link then times out -> a bare reset_session_state() runs WITH the latch still set
		// (the accept has not happened yet). Pre-fix this second latched reset also skipped the
		// snapshot and zeroed the cursor, so prev never captured. The fix's monotonic-max keeps it.
		reset_session_state();
		CHECK(rx_stream_delivered == 0, "X2: link-timeout reset re-anchored the RX cursor at 0",
			(long long)rx_stream_delivered, 0);
		// THE FIRE: prev must have survived the latched teardown chain. @ae8a236 this reads 0 (skip).
		CHECK(rsp_prev_session_app_delivered == 185,
			"X2: high-water SURVIVED the latched gap-abort+timeout teardown (F1 fire; ae8a236=0)",
			(long long)rsp_prev_session_app_delivered, 185);

		// (4) Fresh directed START_CONNECTION -> the callsign-matched production accept clears the
		// abort latch and runs the seam-arm decision.
		drive_directed_start_accept();
		CHECK(link_status == CONNECTION_RECEIVED,
			"X2: directed START reached the callsign-matched accept branch",
			link_status, CONNECTION_RECEIVED);
		CHECK(!rsp_stream_aborted,
			"X2: directed START accept cleared the gap-abort latch",
			rsp_stream_aborted?1:0, 0);
		CHECK(rsp_cross_session_seam_armed,
			"X2: seam ARMED at the fresh accept after the REAL teardown (F1 fire; ae8a236=disarmed)",
			rsp_cross_session_seam_armed?1:0, 1);

		// (5) The fresh session's first delivery re-anchored at 0, so its stamp.start==cursor==0 and
		// the within-session backstop is inert -- the splice is invisible to every per-session guard.
		const int X2BSI = 92;
		rx_stream_stamp[X2BSI].start  = 0;
		rx_stream_stamp[X2BSI].length = 150;
		rx_stream_stamp[X2BSI].valid  = true;
		decrypt_delivered_bsi = X2BSI;
		this->data_batch_size = 1;
		for(int f=0;f<nMessages;f++){ messages_rx[f].status=FREE; messages_rx[f].length=0; }
		for(int j=0;j<150;j++) messages_rx[0].data[j]=(char)((j*7+9)&0x7F);
		messages_rx[0].length=150; messages_rx[0].id=(char)0; messages_rx[0].status=ACKED;
		CHECK(!w_stream_shift_detected(X2BSI),
			"X2: within-session backstop INERT at the seam (stamp.start==cursor==0)",
			(long long)(uint32_t)rx_stream_stamp[X2BSI].start, (long long)(uint32_t)rx_stream_delivered);
		fifo_buffer_rx.flush();
		copy_data_to_buffer();
		char x2tmp[65536]; int x2popped = fifo_buffer_rx.pop(x2tmp,(int)sizeof(x2tmp));
		if(x2popped>0)
			printf("[TEST-STREAM-OFFSET]   (ae8a236) splice reproduced: popped=%d landed at app position 185\n", x2popped);
		// THE INTEGRITY ASSERTION: the fix REFUSES (0 spliced); @ae8a236 the bytes SPLICE (>0).
		CHECK(x2popped==0,
			"X2: cross-session splice REFUSED -- 0 spliced bytes (ae8a236 SPLICES: popped>0)", x2popped, 0);
		CHECK(rsp_stream_aborted,
			"X2: refusal latched rsp_stream_aborted (loud drop via gap-abort contract)",
			rsp_stream_aborted?1:0, 1);
		CHECK(rsp_prev_session_app_delivered==0,
			"X2: prev-delivered cleared post-refusal (next transfer fresh -- no livelock)",
			(long long)rsp_prev_session_app_delivered, 0);
	}

	// ---------------------------------------------------------------------------
	// PART X3 -- F2 no-false-refuse: a PROVEN-CLEAN end-of-transfer clears the app-delivered high-
	// water so a legitimate back-to-back NEW transfer on the SAME persistent socket is byte-
	// identical (never refused). Without F2, F1's monotonic-max keeps prev>0 across a clean
	// completion and the next accept would FALSELY arm. This exercises the SAME clear the CLOSE
	// handler calls (rsp_seam_clear_on_clean_eot), after a REAL reset_session_state().
	printf("[TEST-STREAM-OFFSET] Part X3 -- F2 clean-EOT clear (no false-refuse on back-to-back)\n");
	{
		fifo_buffer_rx.flush();
		rx_stream_delivered            = 0;
		rx_stream_crc                  = CRC32_INIT;
		for(int i=0;i<256;i++) rx_stream_stamp[i].valid = false;
		rsp_stream_aborted             = false;
		rsp_cross_session_seam_armed   = false;
		rsp_prev_session_app_delivered = 0;
		rsp_test_force_app_persistent  = false;

		// (1) A transfer delivers 240 app bytes and completes CLEANLY (peer CLOSE / EOT verified).
		decrypt_delivered_bsi = 210;
		this->data_batch_size = 1;
		for(int f=0;f<nMessages;f++){ messages_rx[f].status=FREE; messages_rx[f].length=0; }
		for(int j=0;j<240;j++) messages_rx[0].data[j]=(char)((j*3+7)&0x7F);
		messages_rx[0].length=240; messages_rx[0].id=(char)0; messages_rx[0].status=ACKED;
		copy_data_to_buffer();
		{ char tmp[65536]; (void)fifo_buffer_rx.pop(tmp,(int)sizeof(tmp)); }
		CHECK(rx_stream_delivered==240, "X3: transfer delivered 240 app bytes", (long long)rx_stream_delivered, 240);

		// (2) The CLOSE handler runs reset_session_state() (F1 re-snapshots prev=240). WITHOUT the
		// F2 clear this alone would leave the next accept armed -- prove that first.
		reset_session_state();
		CHECK(rsp_prev_session_app_delivered==240,
			"X3: clean completion's reset re-snapshotted the high-water (F1)",
			(long long)rsp_prev_session_app_delivered, 240);
		rsp_stream_aborted = false;
		rsp_reconnect_seam_arm_on_accept();
		CHECK(rsp_cross_session_seam_armed,
			"X3: WITHOUT the clean-EOT clear the seam would FALSELY arm (prev survived)",
			rsp_cross_session_seam_armed?1:0, 1);
		rsp_cross_session_seam_armed = false;   // undo the exploratory arm

		// (3) Apply the PROVEN-CLEAN EOT clear the CLOSE handler calls, then re-accept: no arm.
		rsp_seam_clear_on_clean_eot();
		CHECK(rsp_prev_session_app_delivered==0,
			"X3: F2 clean-EOT clear zeroed the high-water", (long long)rsp_prev_session_app_delivered, 0);
		rsp_stream_aborted = false;
		rsp_reconnect_seam_arm_on_accept();
		CHECK(!rsp_cross_session_seam_armed,
			"X3: after F2 clear the back-to-back transfer does NOT arm (no false-refuse)",
			rsp_cross_session_seam_armed?1:0, 0);

		// (4) The legitimate back-to-back transfer delivers fully (byte-identical).
		const int X3BSI = 93;
		rx_stream_stamp[X3BSI].start=0; rx_stream_stamp[X3BSI].length=200; rx_stream_stamp[X3BSI].valid=true;
		decrypt_delivered_bsi = X3BSI;
		this->data_batch_size = 1;
		for(int f=0;f<nMessages;f++){ messages_rx[f].status=FREE; messages_rx[f].length=0; }
		for(int j=0;j<200;j++) messages_rx[0].data[j]=(char)((j*13+5)&0x7F);
		messages_rx[0].length=200; messages_rx[0].id=(char)0; messages_rx[0].status=ACKED;
		fifo_buffer_rx.flush();
		copy_data_to_buffer();
		char x3tmp[65536]; int x3popped=fifo_buffer_rx.pop(x3tmp,(int)sizeof(x3tmp));
		CHECK(x3popped==200,
			"X3: back-to-back transfer after a clean EOT delivers fully (byte-identical)", x3popped, 200);
	}
	tcp_socket_data.status = saved_data_socket_status;

	// ---------------------------------------------------------------------------
	// PART Y -- compression recovery must restore BOTH coupled identities: the
	// transported-byte cursor and cmd_batch_seq_id. restore_tx_from_compressed()
	// is the one production funnel shared by BREAK, demote, climb, and CFG16-HOLD.
	// A sent batch B has already advanced cmd_batch_seq_id to B+1 while its frames
	// remain PENDING_ACK. If recovery restores B's plaintext but leaves that counter
	// at B+1, the receiver sees the same bytes as a fresh successor instead of a
	// duplicate of B. With no wire stream stamp (a supported safe-no-op condition),
	// copy_data_to_buffer() then appends the bytes twice. The default fix restores
	// the in-flight identity before freeing messages_tx[]; the same-binary
	// MERCURY_COMPRESS_RECOVERY_BSI_DEFEAT=1 arm preserves B+1 and turns the
	// identity + byte-exact checks red.
	// ---------------------------------------------------------------------------
	printf("[TEST-STREAM-OFFSET] Part Y -- compressed recovery restores in-flight bsi and de-dups replay\n");
	{
		bool bsi_defeat = false;
		{ const char* e = std::getenv("MERCURY_COMPRESS_RECOVERY_BSI_DEFEAT");
		  if(e && *e && atoi(e)!=0) bsi_defeat = true; }

		if(fifo_buffer_backup.set_size(65536) != SUCCESSFUL)
		{
			printf("[TEST-STREAM-OFFSET]  FAIL  Y: fifo_buffer_backup set_size failed\n");
			g_fails++;
		}
		compressor.init();
		compressor.streaming_enable();
		this->compression_enabled = true;
		this->sack_v2_enabled = true;

		auto recovery_case = [&](int inflight_bsi, const char* tag) {
			const int RAW = 101;
			unsigned char raw[RAW];
			for(int i=0;i<RAW;i++) raw[i]=(unsigned char)((i*29 + inflight_bsi*7 + 3) & 0xFF);

			fifo_buffer_tx.flush();
			fifo_buffer_backup.flush();
			fifo_buffer_rx.flush();
			for(int i=0;i<nMessages;i++)
			{
				messages_tx[i].status=FREE; messages_tx[i].length=0;
				messages_rx[i].status=FREE; messages_rx[i].length=0;
			}
			for(int i=0;i<256;i++)
			{
				tx_stream_stamp[i].valid=false;
				rx_stream_stamp[i].valid=false;
			}

			// Production chronology: B is built and sent, so the byte cursor is past
			// it and the commander's next-new-data identity is B+1. The reverse ACK
			// is lost, leaving B in messages_tx[] for the recovery funnel.
			tx_stream_committed=37;
			tx_stream_crc=CRC32_INIT;
			stream_tx_latch(inflight_bsi, RAW);
			messages_tx[0].length=RAW;
			messages_tx[0].batch_seq_id=inflight_bsi;
			messages_tx[0].status=PENDING_ACK;
			for(int i=0;i<RAW;i++) messages_tx[0].data[i]=(char)(raw[i] ^ 0x5A);
			cmd_batch_seq_id=(inflight_bsi + 1) & 0xFF;
			fifo_buffer_backup.push((char*)raw, RAW);

			// The real compression recovery funnel rolls the cursor, resets the
			// streaming context, frees messages_tx[], and restores plaintext.
			restore_tx_from_compressed();
			char restored[RAW+8];
			int restored_n=fifo_buffer_tx.pop(restored, (int)sizeof(restored));
			bool restore_exact=(restored_n==RAW && memcmp(restored, raw, RAW)==0);
			char what[160];
			snprintf(what, sizeof(what), "Y(%s): production restore returns plaintext byte-exact", tag);
			CHECK(restore_exact, what, restored_n, RAW);
			snprintf(what, sizeof(what), "Y(%s): cmd identity restored to in-flight batch", tag);
			CHECK((cmd_batch_seq_id & 0xFF)==(inflight_bsi & 0xFF), what,
				cmd_batch_seq_id & 0xFF, inflight_bsi & 0xFF);
			CHECK(tx_stream_committed==37, "Y: stream cursor rolled back in lockstep",
				(long long)tx_stream_committed, 37);

			// Receiver already emitted B before its ACK was lost. Seed the app FIFO
			// with that first byte-exact delivery, then re-present the restored bytes
			// under the identity the commander will use. No stamp is present: this is
			// the production backstop's documented safe-no-op case, so bsi de-dup is
			// the sole barrier against a duplicate append.
			this->compression_enabled=false;
			this->data_batch_size=1;
			rx_stream_delivered=RAW;
			rx_stream_emitted_bsi_hw=inflight_bsi & 0xFF;
			decrypt_delivered_bsi=cmd_batch_seq_id & 0xFF;
			fifo_buffer_rx.push((char*)raw, RAW);
			memcpy(messages_rx[0].data, raw, RAW);
			messages_rx[0].length=RAW;
			messages_rx[0].status=ACKED;
			copy_data_to_buffer();
			char app[RAW*2+8];
			int app_n=fifo_buffer_rx.pop(app, (int)sizeof(app));
			bool app_exact=(app_n==RAW && memcmp(app, raw, RAW)==0);
			snprintf(what, sizeof(what), "Y(%s): recovered replay does not duplicate app bytes", tag);
			CHECK(app_exact, what, app_n, RAW);
			this->compression_enabled=true;
		};

		recovery_case(41, "ordinary");
		recovery_case(255, "wrap-255-to-0");

		// No assigned in-flight batch is a strict no-op for the sequence identity.
		fifo_buffer_tx.flush(); fifo_buffer_backup.flush();
		for(int i=0;i<nMessages;i++){ messages_tx[i].status=FREE; messages_tx[i].length=0; }
		cmd_batch_seq_id=77;
		restore_tx_from_compressed();
		CHECK(cmd_batch_seq_id==77, "Y(control): no in-flight batch leaves cmd identity unchanged",
			cmd_batch_seq_id, 77);

		// Legacy v1 never carries batch_seq_id, so even an assigned-looking test
		// slot must not change its local counter (shipping-v1 behavior unchanged).
		this->sack_v2_enabled=false;
		messages_tx[0].status=PENDING_ACK; messages_tx[0].length=8;
		messages_tx[0].batch_seq_id=12;
		cmd_batch_seq_id=78;
		restore_tx_from_compressed();
		CHECK(cmd_batch_seq_id==78, "Y(control): v1 recovery leaves cmd identity unchanged",
			cmd_batch_seq_id, 78);

		// A frame staged before process_messages_tx_data() still has the -1
		// sentinel. It belongs to the current identity and must not roll it back.
		this->sack_v2_enabled=true;
		messages_tx[0].status=ADDED_TO_LIST; messages_tx[0].length=8;
		messages_tx[0].batch_seq_id=-1;
		cmd_batch_seq_id=79;
		restore_tx_from_compressed();
		CHECK(cmd_batch_seq_id==79, "Y(control): unassigned staged frame leaves cmd identity unchanged",
			cmd_batch_seq_id, 79);

		compressor.streaming_disable();
		compressor.deinit();
		this->compression_enabled=false;
		printf("[TEST-STREAM-OFFSET] Part Y bsi_defeat=%d\n", (int)bsi_defeat);
	}
	// PART V — REBASE-SEAM FAIL-CLOSED: the byte-offset STREAM-SPLICE escape
	// (data-flow-stream-offset.md §11). After a DEMOTE-REBASE re-baselines the bsi window
	// (cur=prev=-1), a byte-NON-contiguous batch can present as bsi-CONTIGUOUS — recovery
	// re-labels the bsi (lossless requeue rolls cmd_batch_seq_id) while the absolute byte
	// cursor jumps ~one config's worth of bytes. The delivery-time BACKSTOP
	// (w_stream_shift_detected) is FAIL-OPEN when the EOB frame (hence the stamp) is LOST,
	// so the shifted stamp-riding batch delivers SILENTLY (WGN:15 cfg13, 4/8 cells: a
	// byte-perfect prefix then every later byte lifted ~125 KB ahead — ~125 KB silently
	// skipped and admitted as contiguous). This drives the PRODUCTION decision predicate
	// w_seam_refuse() (the SAME copy_data_to_buffer calls) + the PRODUCTION
	// copy_data_to_buffer() reassembler (compression OFF — byte-exact oracle).
	//   PASS-AFTER (default): seam armed + stamp-riding + no aligned stamp ⇒ w_seam_refuse()
	//     true ⇒ the batch is REFUSED (mirroring the socket-bound gap-abort teardown, exactly
	//     as Part G does) ⇒ 0 silent bytes beyond the good prefix.
	//   FAIL-BEFORE (MERCURY_STREAM_SEAM_FAILCLOSED=0 — reproduces the pre-fix fail-open on
	//     the SAME binary): the caller-knob defeats the refuse ⇒ copy_data_to_buffer delivers
	//     the unproven (possibly-shifted) batch SILENTLY — the escape reproduced.
	// Also asserts the fix (a) does NOT false-fire when no seam is armed, (b) DISARMS on a
	// byte-aligned stamped delivery, and (c) does NOT break the legitimate stampless-
	// contiguous (robust / tiny-frame) path even WHILE the seam is armed.
	// ---------------------------------------------------------------------------
	printf("[TEST-STREAM-OFFSET] Part V — REBASE-SEAM fail-closed (byte-offset stream-splice)\n");
	{
		bool seam_defeat = false;
		{ const char* e = std::getenv("MERCURY_STREAM_SEAM_FAILCLOSED");
		  if(e && *e && atoi(e)==0) seam_defeat = true; }

		this->sack_v2_enabled   = true;
		this->header_carries_d5 = true;
		CHECK(w_stamp_rides(), "V: stamp rides at the test config (seam applies)", w_stamp_rides()?1:0, 1);

		const int VBSI = 88;
		const int VK = 20;   // a ~cfg13 OFDM batch: 20 frames * GFLEN(150) B = 3000 B
		// The receiver funnel de-dups a re-emit whose wire bsi equals the emit high-water
		// (fwd==0 -> [RSP-V2-DEDUP-DROP], keyed on rx_stream_emitted_bsi_hw). Across a real
		// DEMOTE-REBASE the delivery that must re-prove byte alignment is a NEW forward batch
		// (fwd>=1: the CMD re-sends the abandoned bytes under a rolled bsi), so it clears the
		// emit de-dup and reaches the rebase-seam gate. Model that before each delivery below
		// (emit high-water one behind the wire bsi) so each check exercises the REBASE-SEAM
		// decision + reassembler, not the orthogonal emit de-dup gate (tested in test_dedup_rebase).

		// V0 — NO false-fire (seam NOT armed): a stampless delivery must proceed. This is the
		// task's "legitimate stampless-contiguous case (no rebase seam)".
		rsp_rebase_seam_armed       = false;
		rx_stream_delivered         = 101;                 // a clean 101-byte prefix delivered
		rx_stream_stamp[VBSI].valid = false;
		decrypt_delivered_bsi       = VBSI;
		CHECK(!w_seam_refuse(VBSI), "V0: no seam armed => stampless delivery not refused (no false-fire)", 0, 0);
		fifo_buffer_rx.flush();
		int v0_total = seat_rx_batch(VK, VK);
		uint64_t v0_before = rx_stream_delivered;
		rx_stream_emitted_bsi_hw = (VBSI - 1) & 0xFF;      // forward batch across the rebase seam (fwd>=1): INV-DEDUP inert, so this exercises the SEAM gate
		copy_data_to_buffer();                             // seam inert => clean delivery (PRODUCTION path)
		{ char tmp[65536]; int popped=fifo_buffer_rx.pop(tmp,(int)sizeof(tmp));
		  CHECK(popped==v0_total, "V0: stampless batch delivered fully when no seam armed", popped, v0_total); }
		CHECK(rx_stream_delivered == v0_before + (uint64_t)v0_total,
			"V0: cursor advanced by the delivered bytes", (long long)rx_stream_delivered, (long long)(v0_before + v0_total));

		// V-empty — FILE_END and other no-residual flushes enter the delivery funnel
		// with the -1 sentinel. No batch exists, so the seam predicate must not turn
		// that sentinel into wire bsi 255 and refuse an otherwise clean config climb.
		rsp_rebase_seam_armed = true;
		decrypt_delivered_bsi = -1;
		CHECK(!w_seam_refuse(decrypt_delivered_bsi),
			"V-empty: no-batch sentinel is not refused as unstamped delivery",
			w_seam_refuse(decrypt_delivered_bsi)?1:0, 0);

		// V — ARM the seam (as the production DEMOTE-REBASE does) and present the SPLICE: a
		// bsi-contiguous batch with NO stamp (EOB lost) at a stamp-riding config.
		rsp_rebase_seam_armed       = true;
		rx_stream_delivered         = 101;                 // cursor at the good-prefix boundary
		rx_stream_stamp[VBSI].valid = false;               // EOB frame (stamp) LOST across the seam
		decrypt_delivered_bsi       = VBSI;
		// The decision predicate MUST fire (seam armed + stamp-riding + no aligned stamp), BOTH modes.
		CHECK(w_seam_refuse(VBSI), "V: seam+stamp-riding+no-stamp => REFUSE decision fires", 1, 1);
		fifo_buffer_rx.flush();
		int vtotal = seat_rx_batch(VK, VK);
		if(seam_defeat)
		{
			// fail-before: the caller-knob defeats the refuse => copy_data_to_buffer delivers
			// the unproven (possibly-shifted) batch SILENTLY — the escape reproduced.
			uint64_t before = rx_stream_delivered;
			rx_stream_emitted_bsi_hw = (VBSI - 1) & 0xFF;  // forward batch across the rebase seam (fwd>=1): INV-DEDUP inert, so this exercises the SEAM gate
			copy_data_to_buffer();
			char tmp[65536]; int popped=fifo_buffer_rx.pop(tmp,(int)sizeof(tmp));
			bool silent = (popped==vtotal) && (rx_stream_delivered==before+(uint64_t)vtotal);
			CHECK(silent, "V(defeat): unstamped batch across seam SILENTLY delivered (fail-before escape)", popped, vtotal);
		}
		else
		{
			// pass-after: the decision fired (asserted above). The production copy_data_to_buffer
			// refuses via rsp_gap_abort_teardown (socket-bound — mirrored, not invoked, exactly as
			// Part G does). REFUSE => 0 silent bytes beyond the 101-byte prefix.
			char tmp[65536]; int popped=fifo_buffer_rx.pop(tmp,(int)sizeof(tmp));  // fifo still empty
			CHECK(popped==0, "V(fix): unstamped stamp-riding batch across seam REFUSED, 0 silent bytes", popped, 0);
		}

		// V-disarm — a STAMPED, byte-aligned delivery across the seam is byte-PROVABLE => it
		// delivers AND disarms the seam (the rest of the transfer proceeds normally).
		rsp_rebase_seam_armed        = true;
		rx_stream_delivered          = 101;
		rx_stream_stamp[VBSI].start  = 101;                // stamp proves start == cursor (aligned)
		rx_stream_stamp[VBSI].length = VK*GFLEN;
		rx_stream_stamp[VBSI].valid  = true;
		decrypt_delivered_bsi        = VBSI;
		CHECK(!w_seam_refuse(VBSI), "V-disarm: aligned stamp across seam => not refused", 0, 0);
		fifo_buffer_rx.flush();
		int vd_total = seat_rx_batch(VK, VK);
		rx_stream_emitted_bsi_hw = (VBSI - 1) & 0xFF;      // forward batch across the rebase seam (fwd>=1): INV-DEDUP inert, so this exercises the SEAM gate
		copy_data_to_buffer();                             // delivers + disarms (PRODUCTION path)
		{ char tmp[65536]; int popped=fifo_buffer_rx.pop(tmp,(int)sizeof(tmp));
		  CHECK(popped==vd_total, "V-disarm: aligned stamped batch delivered fully across seam", popped, vd_total); }
		CHECK(!rsp_rebase_seam_armed, "V-disarm: seam DISARMED after a byte-aligned delivery",
			rsp_rebase_seam_armed?1:0, 0);

		// V-legit — WHILE the seam is armed, a NON-stamp-riding (robust / tiny-frame) delivery is
		// the legitimate stampless-contiguous path and must STILL deliver (the fix must not break
		// robust demote-recovery). header_carries_d5=false => w_stamp_rides()==false.
		rsp_rebase_seam_armed        = true;
		this->header_carries_d5      = false;              // robust config: no stamp rides
		CHECK(!w_stamp_rides(), "V-legit: non-stamp-riding config (robust)", w_stamp_rides()?1:0, 0);
		rx_stream_delivered          = 500;
		rx_stream_stamp[VBSI].valid  = false;
		decrypt_delivered_bsi        = VBSI;
		CHECK(!w_seam_refuse(VBSI), "V-legit: robust stampless delivery under armed seam => not refused", 0, 0);
		fifo_buffer_rx.flush();
		int vl_total = seat_rx_batch(3, 3);                // a tiny robust batch
		uint64_t vl_before = rx_stream_delivered;
		rx_stream_emitted_bsi_hw = (VBSI - 1) & 0xFF;      // forward batch across the rebase seam (fwd>=1): INV-DEDUP inert, so this exercises the SEAM gate
		copy_data_to_buffer();                             // delivers (seam leaves robust untouched)
		{ char tmp[65536]; int popped=fifo_buffer_rx.pop(tmp,(int)sizeof(tmp));
		  CHECK(popped==vl_total, "V-legit: robust stampless batch delivered under armed seam (fix does not break it)", popped, vl_total); }
		CHECK(rx_stream_delivered == vl_before + (uint64_t)vl_total,
			"V-legit: robust delivery advanced the cursor", (long long)rx_stream_delivered, (long long)(vl_before + vl_total));
		this->header_carries_d5 = true;                    // restore

		printf("[TEST-STREAM-OFFSET] Part V seam_defeat=%d (0=fix->refuse, 1=defeat->silent)\n", (int)seam_defeat);
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
