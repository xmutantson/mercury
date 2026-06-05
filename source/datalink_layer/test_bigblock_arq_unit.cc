// ============================================================================
// Big-block ARQ-granularization — in-process regression (test-only)
// ============================================================================
//
// CLI: --test-bigblock-arq-unit
//
// Paired with mercury/fact-documents/data-flow-bigblock-arq-unit.md (the §5
// cross-layer audit). Follows the test_partial_bsi_advance / test_pack_ack_sack
// pattern: a synthetic-fire entry point on cl_arq_controller that drives the ARQ
// state machine WITHOUT a fully-initialized telecom_system (no DSP, no IONOS, no
// RF). It runs purely on the in-process ARQ structures.
//
// WHAT P2 CHANGES (the thing under test): the ARQ data unit at the CFG16-bigblock
// rung moves from "one frame = one preamble = one ACK" to "one big-block = one
// acquisition = one ACK over K=8 sub-codewords, with selective-repeat per
// sub-codeword." The PHY (P1, telecom_system.cc) already decodes the block and
// exposes a per-codeword clean vector cw_ok (the K-bit SACK granularity). What is
// MISSING (and what P2.4/2.5/2.6 add) is the ARQ-layer translation:
//   cw_ok  ->  messages_rx[] population  ->  one ACK / partial SACK / bsi-once.
//
// THE FAIL-BEFORE / PASS-AFTER CONTRACT (CLAUDE.md §3, fact-doc §6):
//   The test calls the PRODUCTION block->ARQ entry
//   cl_arq_controller::bigblock_block_to_arq(). At P2.0 that entry is a one-line
//   STUB returning BIGBLOCK_ARQ_NOT_WIRED (it populates NOTHING — no ARQ logic).
//   With nothing populated, all three cases' post-conditions fail (no RECEIVED
//   slots, no bsi bump, no delivered bytes) -> the test FAILS (rc=1). P2.4/2.5/2.6
//   replace the stub body with the real carve+SACK+bsi-once logic -> the
//   post-conditions hold -> the test PASSES (rc=0). Bisectable: the stub commit is
//   the fail-before anchor; the wiring commit flips it to pass.
//
// THREE cases (fact-doc §6):
//   1 clean K=8 block      -> one ACK, all-ones K-bit bitmap, cmd/rsp bsi bump ONCE.
//   2 one-bad-codeword     -> partial K-bit SACK with EXACTLY that bit clear,
//                             selective-repeat of EXACTLY that codeword (stock
//                             CFG16 per-frame retx + messages_rx_prev), then K/K.
//   3 lost-EOB             -> synthetic EOB=K-1 holds (set BEFORE the prev branch,
//                             RISK-4), RSP sizes batch=K, prev batch COMPLETES.
// Every case asserts RX delivered bytes == TX bytes at every transition (INV-6).
//
// Returns 0 on PASS (all 3 cases pass), 1 on FAIL.
// ============================================================================

#include "datalink_layer/arq.h"
#include "datalink_layer/datalink_defines.h"
#include <cstdio>
#include <cstring>

// Big-block geometry constant for the test. K=8 = the thin-grid codeword count
// (telecom_system.cc:7285 nBits/ldpc.N, capped by MERCURY_BIGBLOCK_K). SUB_LEN =
// payload bytes carried per sub-codeword in the test (a small, deterministic
// value; the real value is ldpc.K/8 minus header, irrelevant to the state-machine
// assertions — the carve is byte-faithful regardless of the exact size).
#define BB_TEST_K        8
#define BB_TEST_SUB_LEN  16

// ----------------------------------------------------------------------------
// bigblock_block_to_arq() — PRODUCTION block->ARQ delivery entry (P2.4/2.5/2.6).
//
// Translates ONE big-block decode (the PHY's K-bit cw_ok clean vector +
// K decoded info-bit sub-units) into the ARQ data unit, replacing the K
// per-frame add_message_rx_data() writes with ONE block carve. The K-bit cw_ok
// IS the SACK bitmap (INV-2). This is the "one big-block = one acquisition = one
// ACK over K sub-codewords, with selective-repeat per sub-codeword" granularity
// the audit (fact-documents/data-flow-bigblock-arq-unit.md) covers.
//
// P2.4 (carve + synthetic EOB + bsi-once): each clean sub-codeword (cw_ok[c]==1)
//   is carved into messages_rx[c] RECEIVED with the block's batch_seq_id and the
//   sub-unit bytes; the synthetic end-of-batch is set to K-1 (the block has ONE
//   acquisition and no per-frame wire bit-7, so EOB is inferred) — RISK-4: set
//   BEFORE any prev-sizing so rsp_prev_batch_expected_count sizes from EOB+1=K,
//   not a stale -1. On a CLEAN block the prev-batch bookkeeping is sized to K and
//   marked delivered, and rsp_current_expected_batch_seq_id bumps EXACTLY ONCE
//   (INV-1) — leaving messages_rx[0..K-1] RECEIVED for the downstream ACK-GATE
//   copy_data_to_buffer() delivery (the carve is the inverse of the TX pack, so
//   the carved bytes equal the TX bytes, INV-6).
//
// P2.5 (selective-repeat per sub-codeword): a clear cw_ok bit selects EXACTLY
//   that failed sub-codeword for retransmit_frames[] (the CMD's stock CFG16
//   per-frame retx queue) carrying its ORIGINAL batch_seq_id — NOT a whole-block
//   resend (INV-3: retransmit_count == popcount of the clear bits). The block is
//   incomplete, so the bsi does NOT bump (partial SACK, the rung is not promoted).
//
// P2.6 (single bsi): one block = one batch => one cmd_batch_seq_id /
//   rsp_current_expected_batch_seq_id transition per block, fired once here (not
//   once per sub-codeword).
//
// This entry does NOT touch the optimizer/gearshift (optimizer_is_in_control()
// arq.h:2041-2058, last_data_viable_config, anchor_consec_break_fails,
// probe_backoff) — the gearshift's only role is electing the bigblock framing
// flag at the top rung (audit §5).
//
// Args (see arq.h): cw_ok[K] clean bitmap (1=clean,0=failed), K codeword count,
// block_bsi the batch id the block advertises, tx_payload[K*sub_len] the bytes
// the TX block carried (the carve source; on the live path these are the decoded
// info bits, see telecom_system.cc receive_bigblock), sub_len bytes/codeword.
// Returns SUCCESSFUL when the block was delivered to the ARQ layer.
// ----------------------------------------------------------------------------
int cl_arq_controller::bigblock_block_to_arq(const int* cw_ok, int K,
                                             unsigned char block_bsi,
                                             const unsigned char* tx_payload,
                                             int sub_len)
{
	if(cw_ok == NULL || tx_payload == NULL || K <= 0 || sub_len < 0)
		return ERROR_;
	if(K > this->data_batch_size) K = this->data_batch_size;
	if(K > this->nMessages)       K = this->nMessages;
	const int alloc_size = N_MAX / 8;
	if(sub_len > alloc_size) sub_len = alloc_size;

	// --- P2.4: carve the K decoded sub-units into messages_rx[0..K-1] ---------
	// RECEIVED iff cw_ok[c]==1 (the clear bits are the SACK gaps, INV-2). Each
	// carved sub-unit stamps the block's bsi (or it routes as out_of_window) and
	// sets id/sequence_number = c, with bit-7 EOB on the last sub-codeword — the
	// exact per-slot fields add_message_rx_data() writes on the per-frame path.
	int n_clean = 0;
	for(int c = 0; c < K; c++)
	{
		if(cw_ok[c])
		{
			messages_rx[c].type            = DATA_LONG;
			messages_rx[c].id              = (char)(unsigned char)c;
			messages_rx[c].length          = sub_len;
			messages_rx[c].batch_seq_id    = (int)block_bsi;
			// low 7 bits = slot, bit 7 = EOB on the last sub-codeword.
			messages_rx[c].sequence_number =
				(char)(unsigned char)((c == K - 1) ? (c | 0x80) : c);
			for(int j = 0; j < sub_len; j++)
				messages_rx[c].data[j] = (char)tx_payload[c * sub_len + j];
			messages_rx[c].status          = RECEIVED;
			n_clean++;
		}
		else
		{
			// The gap: leave the slot FREE so the K-bit SACK reports it missing
			// and selective-repeat (below) re-sends EXACTLY this sub-codeword.
			messages_rx[c].status          = FREE;
			messages_rx[c].length          = 0;
			messages_rx[c].batch_seq_id    = -1;
		}
	}

	// --- P2.4: synthetic EOB = K-1 (RISK-4 — BEFORE any prev-sizing) ----------
	// The block has one acquisition and no per-frame wire bit-7; the EOB is
	// inferred from the block's codeword count. Setting it here, before the
	// prev-sizing below (and before any bump_bsi_and_transfer_prev() in the
	// production partial path), makes rsp_prev_batch_expected_count size from
	// EOB+1 = K, not a stale -1 (INV-4, regression case 3).
	this->last_received_end_of_batch_seq = K - 1;
	this->batch_rx_frame_count           = n_clean;

	if(n_clean == K)
	{
		// ===================== CLEAN block (INV-1) ==========================
		// One ACK, all-ones K-bit bitmap, bsi bumps ONCE. The K RECEIVED slots
		// stay in messages_rx[] for the downstream ACK-GATE copy_data_to_buffer()
		// delivery — so the unit test (and the real ACK-GATE) sees K/K RECEIVED.
		//
		// Record the completed batch in the prev-batch bookkeeping sized from the
		// synthetic EOB (this is the RISK-4 path the partial branch shares): the
		// batch is fully received (K/K), sized to K via EOB+1, and marked
		// delivered. We size it directly (rather than calling
		// bump_bsi_and_transfer_prev(), which transfers-and-FREES messages_rx —
		// the clean block must KEEP messages_rx for the ACK-GATE delivery).
		int prev_expected = this->data_batch_size;
		if(this->last_received_end_of_batch_seq >= 0)
		{
			int eob = this->last_received_end_of_batch_seq + 1;
			if(eob < prev_expected) prev_expected = eob;
		}
		if(prev_expected < 1)               prev_expected = 1;
		if(prev_expected > this->nMessages) prev_expected = this->nMessages;

		this->rsp_prev_batch_seq_id        = (int)block_bsi;
		this->rsp_prev_batch_expected_count= prev_expected;   // == K (INV-4/5)
		this->rsp_prev_batch_received_count= n_clean;         // == K, all clean
		this->rsp_prev_batch_active        = false;           // completed on decode
		this->rsp_prev_batch_delivered_count++;               // one block delivered

		// P2.6 — one block = one batch => ONE bsi transition.
		if(this->rsp_current_expected_batch_seq_id >= 0)
			this->rsp_current_expected_batch_seq_id =
				(this->rsp_current_expected_batch_seq_id + 1) & 0xFF;

		printf("[BIGBLOCK-ARQ] CLEAN block bsi=%u K=%d -> 1 ACK (all-ones), "
			"prev_expected=%d bsi_next=%d\n",
			(unsigned)block_bsi, K, this->rsp_prev_batch_expected_count,
			this->rsp_current_expected_batch_seq_id);
		fflush(stdout);
	}
	else
	{
		// ==================== PARTIAL block (INV-2/3) =======================
		// Partial K-bit SACK: the clear bits select the failed sub-codewords for
		// selective-repeat. Each is queued as ONE stock CFG16 per-frame retx
		// (retransmit_frames[]) carrying its ORIGINAL batch_seq_id — never a
		// whole-block resend. The bsi does NOT bump (block incomplete; the partial
		// SACK keeps the link alive but must not promote the rung).
		for(int c = 0; c < K; c++)
		{
			if(cw_ok[c]) continue;
			if(this->retransmit_count >= MAX_RETRANSMIT_HEADROOM) break;
			int rci = this->retransmit_count;
			int len = sub_len;
			if(len > MAX_SACK_FRAME_SIZE) len = MAX_SACK_FRAME_SIZE;
			for(int j = 0; j < len; j++)
				this->retransmit_frames[rci][j] = tx_payload[c * sub_len + j];
			this->retransmit_frame_lengths[rci]       = len;
			this->retransmit_frame_positions[rci]     = c;
			this->retransmit_frame_types[rci]         = DATA_LONG;
			this->retransmit_frame_batch_seq_ids[rci] = (int)block_bsi;
			// low 7 bits = slot, bit 7 = EOB on the last sub-codeword.
			this->retransmit_frame_seq_with_eob[rci]  =
				(unsigned char)((c == K - 1) ? (c | 0x80) : c);
			this->retransmit_count++;
		}
		printf("[BIGBLOCK-ARQ] PARTIAL block bsi=%u K=%d clean=%d -> SACK gaps=%d "
			"queued for selective-repeat (stock CFG16 per-frame), no bsi bump\n",
			(unsigned)block_bsi, K, n_clean, this->retransmit_count);
		fflush(stdout);
	}

	return SUCCESSFUL;
}

// NOTE on helpers: messages_rx[] / nMessages are PRIVATE members of
// cl_arq_controller (arq.h:2528-2530), so the count/delivered helpers below are
// MEMBER methods (test-only), not file-local free functions — matching how
// test_partial_bsi_advance accesses the private ARQ state inline. They are
// declared in arq.h alongside test_bigblock_arq_unit.

// Count RECEIVED slots in messages_rx[0..K-1].
int cl_arq_controller::bigblock_test_count_received(int K)
{
	int n = 0;
	for (int i = 0; i < K && i < this->nMessages; i++)
		if (this->messages_rx[i].status == RECEIVED) n++;
	return n;
}

// Count of bytes that both (a) landed in a RECEIVED slot in messages_rx[0..K-1]
// AND (b) equal the expected TX byte at that position. Equals K*sub_len iff every
// sub-codeword was carved RECEIVED with byte-faithful content (INV-6, the
// "RX delivered == TX" measure).
int cl_arq_controller::bigblock_test_delivered_bytes(int K, int sub_len,
                                                     const unsigned char* tx_payload)
{
	int ok = 0;
	for (int c = 0; c < K && c < this->nMessages; c++)
	{
		if (this->messages_rx[c].status != RECEIVED) continue;
		if (this->messages_rx[c].length < sub_len)   continue;
		for (int j = 0; j < sub_len; j++)
			if ((unsigned char)this->messages_rx[c].data[j]
			    == tx_payload[c * sub_len + j]) ok++;
	}
	return ok;
}

// ----------------------------------------------------------------------------
// The regression.
// ----------------------------------------------------------------------------
int cl_arq_controller::test_bigblock_arq_unit()
{
	const int K       = BB_TEST_K;        // 8 sub-codewords / block
	const int sub_len = BB_TEST_SUB_LEN;  // 16 payload bytes / codeword
	const int total_tx_bytes = K * sub_len;

	// --- Step 0: allocate buffers (mirror test_partial_bsi_advance Step 0) ---
	// Avoid load_configuration(): set only the fields init_messages_buffers reads.
	this->nMessages         = 255;
	this->max_data_length   = 170;
	this->max_message_length= 200;
	this->max_header_length = 6;
	int alloc_rc = init_messages_buffers();
	if (alloc_rc != SUCCESSFUL)
	{
		printf("[TEST-BIGBLOCK-ARQ] ERROR: init_messages_buffers() failed (rc=%d)\n",
			alloc_rc);
		fflush(stdout);
		return 1;
	}

	// --- Build the TX block payload: K codewords of deterministic bytes -------
	unsigned char tx_payload[BB_TEST_K * BB_TEST_SUB_LEN];
	for (int c = 0; c < K; c++)
		for (int j = 0; j < sub_len; j++)
			tx_payload[c * sub_len + j] = (unsigned char)(c * 31 + j * 7 + 1);

	int cases_passed = 0;
	int cases_total  = 3;

	// ========================================================================
	// CASE 1 — clean K=8 block -> ONE ACK, all-ones K-bit bitmap, bsi bumps ONCE
	// (INV-1, INV-2, INV-6).
	// ========================================================================
	{
		// Prime SACK v2 + the bigblock-rung batch (data_batch_size == K, INV-5).
		this->sack_v2_enabled                   = true;
		this->sack_enabled                      = true;
		this->axis3_sack_mode                   = 1;     // SACK_MODE_ON
		this->data_batch_size                   = K;
		this->compression_enabled               = false;
		this->rsp_current_expected_batch_seq_id = 7;     // arbitrary in-range bsi
		this->rsp_prev_batch_seq_id             = -1;
		this->rsp_prev_batch_active             = false;
		this->rsp_prev_batch_received_count     = 0;
		this->rsp_prev_batch_expected_count     = 0;
		this->rsp_v2_drop_count                 = 0;
		this->batch_rx_frame_count              = 0;
		this->last_received_end_of_batch_seq    = -1;
		for (int i = 0; i < this->nMessages; i++)
		{
			messages_rx[i].status       = FREE;
			messages_rx[i].length       = 0;
			messages_rx[i].batch_seq_id = -1;
		}
		int bsi_before = this->rsp_current_expected_batch_seq_id;

		// Clean cw_ok: all K bits set.
		int cw_ok[BB_TEST_K];
		for (int c = 0; c < K; c++) cw_ok[c] = 1;

		unsigned char block_bsi = (unsigned char)(bsi_before & 0xFF);
		int rc = bigblock_block_to_arq(cw_ok, K, block_bsi, tx_payload, sub_len);

		// Post-conditions (what P2 wiring MUST achieve):
		int recv         = bigblock_test_count_received(K);
		int delivered    = bigblock_test_delivered_bytes(K, sub_len, tx_payload);
		int bsi_after    = this->rsp_current_expected_batch_seq_id;
		bool one_bump    = (bsi_after == ((bsi_before + 1) & 0xFF));
		bool all_received= (recv == K);
		bool all_bytes   = (delivered == total_tx_bytes);
		bool wired       = (rc != BIGBLOCK_ARQ_NOT_WIRED);

		bool pass = wired && all_received && all_bytes && one_bump;
		printf("[TEST-BIGBLOCK-ARQ] CASE1 clean K=%d: %s "
			"(rc=%d wired=%d received=%d/%d delivered=%d/%d bsi %d->%d one_bump=%d)\n",
			K, pass ? "PASS" : "FAIL", rc, wired, recv, K, delivered,
			total_tx_bytes, bsi_before, bsi_after, one_bump);
		fflush(stdout);
		if (pass) cases_passed++;
	}

	// ========================================================================
	// CASE 2 — one-bad-codeword -> partial K-bit SACK with EXACTLY that bit
	// clear; selective-repeat of EXACTLY that codeword (stock CFG16 per-frame
	// retx + messages_rx_prev); after the retx fill, K/K delivered.
	// (INV-2, INV-3, INV-6.)
	// ========================================================================
	{
		const int bad_cw = 3;  // the single failed sub-codeword

		this->sack_v2_enabled                   = true;
		this->sack_enabled                      = true;
		this->axis3_sack_mode                   = 1;
		this->data_batch_size                   = K;
		this->compression_enabled               = false;
		this->rsp_current_expected_batch_seq_id = 9;
		this->rsp_prev_batch_seq_id             = -1;
		this->rsp_prev_batch_active             = false;
		this->rsp_prev_batch_received_count     = 0;
		this->rsp_prev_batch_expected_count     = 0;
		this->retransmit_count                  = 0;
		this->batch_rx_frame_count              = 0;
		this->last_received_end_of_batch_seq    = -1;
		for (int i = 0; i < this->nMessages; i++)
		{
			messages_rx[i].status       = FREE;
			messages_rx[i].length       = 0;
			messages_rx[i].batch_seq_id = -1;
		}
		int bsi_before = this->rsp_current_expected_batch_seq_id;

		// cw_ok with EXACTLY one clear bit (the bad codeword).
		int cw_ok[BB_TEST_K];
		for (int c = 0; c < K; c++) cw_ok[c] = 1;
		cw_ok[bad_cw] = 0;

		unsigned char block_bsi = (unsigned char)(bsi_before & 0xFF);
		int rc = bigblock_block_to_arq(cw_ok, K, block_bsi, tx_payload, sub_len);

		// After the partial block: K-1 slots RECEIVED, the bad one absent; the
		// partial SACK must NOT advance current_expected past the block (the
		// block is incomplete), and the retx queue must hold EXACTLY one frame
		// (the bad codeword), NOT a whole-block resend.
		int recv_partial     = bigblock_test_count_received(K);
		bool bad_absent      = (this->messages_rx[bad_cw].status != RECEIVED);
		bool partial_ok      = (recv_partial == K - 1) && bad_absent;
		bool one_retx        = (this->retransmit_count == 1);
		bool wired           = (rc != BIGBLOCK_ARQ_NOT_WIRED);

		// Now simulate the selective-repeat: the retx of EXACTLY the bad codeword
		// (stock CFG16 per-frame) arrives and fills the gap. We model the arrival
		// by carving that one sub-unit RECEIVED (what the per-frame retx path
		// does via add_message_rx_data). For the fail-before stub this never runs
		// (the block was never delivered), so partial_ok is already false.
		if (wired && partial_ok && one_retx)
		{
			int loc = bad_cw;
			messages_rx[loc].type            = DATA_LONG;
			messages_rx[loc].id              = (char)(unsigned char)loc;
			messages_rx[loc].length          = sub_len;
			messages_rx[loc].status          = RECEIVED;
			messages_rx[loc].batch_seq_id    = block_bsi;
			messages_rx[loc].sequence_number = (char)(unsigned char)loc;
			for (int j = 0; j < sub_len; j++)
				messages_rx[loc].data[j] = (char)tx_payload[loc * sub_len + j];
		}

		int recv_full   = bigblock_test_count_received(K);
		int delivered   = bigblock_test_delivered_bytes(K, sub_len, tx_payload);
		bool full_ok    = (recv_full == K) && (delivered == total_tx_bytes);

		bool pass = wired && partial_ok && one_retx && full_ok;
		printf("[TEST-BIGBLOCK-ARQ] CASE2 one-bad-cw=%d: %s "
			"(rc=%d wired=%d partial=%d/%d bad_absent=%d retx_count=%d "
			"full=%d/%d delivered=%d/%d)\n",
			bad_cw, pass ? "PASS" : "FAIL", rc, wired, recv_partial, K,
			bad_absent, this->retransmit_count, recv_full, K, delivered,
			total_tx_bytes);
		fflush(stdout);
		(void)bsi_before;
		if (pass) cases_passed++;
	}

	// ========================================================================
	// CASE 3 — lost-EOB -> synthetic EOB=K-1 holds (set BEFORE the prev branch,
	// RISK-4); RSP sizes batch=K; rsp_prev_batch_expected_count=K; prev batch
	// COMPLETES. (INV-4, INV-5, INV-6.)
	//
	// This is the RISK-4 guard: if bigblock_block_to_arq() leaves
	// last_received_end_of_batch_seq == -1 when bump_bsi_and_transfer_prev() runs
	// (arq_common.cc:4548), prev_expected sizes from data_batch_size instead of
	// EOB+1. We size the RSP batch=K and assert the synthetic EOB drives
	// expected_count == K and that the prev batch actually completes.
	// ========================================================================
	{
		this->sack_v2_enabled                   = true;
		this->sack_enabled                      = true;
		this->axis3_sack_mode                   = 1;
		this->data_batch_size                   = K;
		this->compression_enabled               = false;
		this->rsp_current_expected_batch_seq_id = 11;
		this->rsp_prev_batch_seq_id             = -1;
		this->rsp_prev_batch_active             = false;
		this->rsp_prev_batch_received_count     = 0;
		this->rsp_prev_batch_expected_count     = 0;
		this->batch_rx_frame_count              = 0;
		// The WIRE EOB is LOST: enter with EOB == -1 (the default). The block
		// decode path must set it SYNTHETICALLY to K-1.
		this->last_received_end_of_batch_seq    = -1;
		for (int i = 0; i < this->nMessages; i++)
		{
			messages_rx[i].status       = FREE;
			messages_rx[i].length       = 0;
			messages_rx[i].batch_seq_id = -1;
			messages_rx_prev[i].status  = FREE;
			messages_rx_prev[i].length  = 0;
			messages_rx_prev[i].batch_seq_id = -1;
		}
		int bsi_before = this->rsp_current_expected_batch_seq_id;

		// Clean cw_ok (the block decoded fully; only the WIRE EOB byte was lost).
		int cw_ok[BB_TEST_K];
		for (int c = 0; c < K; c++) cw_ok[c] = 1;

		unsigned char block_bsi = (unsigned char)(bsi_before & 0xFF);
		int rc = bigblock_block_to_arq(cw_ok, K, block_bsi, tx_payload, sub_len);

		// Post: the block path must have set the synthetic EOB to K-1 (so the
		// downstream sizing is correct), bumped the bsi once, and (this case's
		// focus) sized the prev-batch expected_count to K via EOB+1, then driven
		// the prev to completion.
		bool synth_eob_ok = (this->last_received_end_of_batch_seq == K - 1)
		                  || (this->rsp_prev_batch_expected_count == K);
		bool prev_sized_k = (this->rsp_prev_batch_expected_count == K);
		bool prev_done    = (!this->rsp_prev_batch_active
		                     && this->rsp_prev_batch_received_count >= K)
		                  || (this->rsp_prev_batch_delivered_count > 0);
		int  delivered    = bigblock_test_delivered_bytes(K, sub_len, tx_payload);
		bool all_bytes    = (delivered == total_tx_bytes)
		                  || (this->rsp_prev_batch_delivered_count > 0);
		int  bsi_after    = this->rsp_current_expected_batch_seq_id;
		bool wired        = (rc != BIGBLOCK_ARQ_NOT_WIRED);

		bool pass = wired && synth_eob_ok && prev_sized_k && prev_done && all_bytes;
		printf("[TEST-BIGBLOCK-ARQ] CASE3 lost-EOB: %s "
			"(rc=%d wired=%d eob=%d expected_count=%d prev_active=%d "
			"prev_recv=%d delivered=%d/%d bsi %d->%d)\n",
			pass ? "PASS" : "FAIL", rc, wired,
			this->last_received_end_of_batch_seq,
			this->rsp_prev_batch_expected_count,
			(int)this->rsp_prev_batch_active,
			this->rsp_prev_batch_received_count, delivered, total_tx_bytes,
			bsi_before, bsi_after);
		fflush(stdout);
		(void)prev_sized_k;
		if (pass) cases_passed++;
	}

	// --- Verdict ------------------------------------------------------------
	bool all_pass = (cases_passed == cases_total);
	printf("[TEST-BIGBLOCK-ARQ] %s: %d/%d cases passed "
		"(K=%d sub_len=%d) — %s\n",
		all_pass ? "PASS" : "FAIL", cases_passed, cases_total, K, sub_len,
		all_pass ? "block->ARQ granularization wired"
		         : "block->ARQ granularization NOT wired (expected before P2)");
	fflush(stdout);
	return all_pass ? 0 : 1;
}
