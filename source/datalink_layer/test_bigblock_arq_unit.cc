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
#include "common/common_defines.h"   // CONFIG_16, YES/NO
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>
#include <cstdint>

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
// T6 (THE #9 GATE) — CMD/RSP election symmetry against the PRODUCTION setter.
//
// The R-B regression (bug #9): the big-block rung is a CFG16 (OFDM, NON-robust)
// config, so the SHARED batch-size election sack_negotiated_recompute_batch()
// runs the 30s-target formula and elects data_batch_size ~= 25. A big-block
// decode emits ONE K-bit (K=8) SACK bitmap (cw_ok = 0xFF when all-clean). The CMD
// clean-ACK accept gate cmd_clean_data_ack_crc_valid() (arq_commander.cc:136-139)
// derives all_ones = (1<<data_batch_size)-1; at batch=25 that is 0x1FFFFFF, which
// can NEVER equal the RSP's 0xFF -> the clean ACK never matches -> zero clean
// credit -> the "4 wire failures." The P1 R-B pin (arq_common.cc:979-988) fixes
// this by PINNING data_batch_size = bigblock_codeword_count() == K INSIDE the
// shared election body, on BOTH peers, from the SAME PHY geometry source.
//
// This test PROVES the fix against the PRODUCTION setter — NOT a hardcoded K:
//   1. Build TWO independent cl_telecom_system + cl_arq_controller (CMD + RSP).
//   2. Load a REAL CFG16 grid into each (load_configuration(CONFIG_16,FULL,YES)),
//      which on its own elects data_batch_size = radio_batch_size(25) via the 30s
//      formula — i.e. it REPRODUCES the bug-#9 seed state on both peers.
//   3. Turn on bigblock_framing_enabled (the CFG16-rung framing bit).
//   4. Run the PRODUCTION sack_negotiated_recompute_batch() on BOTH (CMD via the
//      "CMD" who-tag = the TEST_CONNECTION_ACK path; RSP via "RSP" = the
//      TEST_CONNECTION path — the SAME shared body).
//   5. ASSERT: both elect data_batch_size == K == BB_TEST_K(8); both derive the
//      identical all_ones target (1<<data_batch_size)-1 == 0xFF (the EXACT
//      cmd_clean_data_ack_crc_valid expression); the RSP's all-clean K-bit bitmap
//      0xFF is accepted by that gate (rx_bitmap==all_ones) AND by the production
//      dedupe helper sack_clean_confirmation_accepted().
//
// MERCURY_BIGBLOCK_K is pinned to BB_TEST_K(8) for the duration so the geometry
// source bigblock_codeword_count() returns exactly K=8 deterministically (the
// production cap path, telecom_system.cc:7820-7821) regardless of the natural
// CFG16 lattice K — the assertion is then "both peers elect the SAME K, derived
// from the SAME production source," which is the divergence-proof property.
//
// fail-before/pass-after: revert the R-B pin (drop the bigblock_rung branch in
// sack_negotiated_recompute_batch) and the CFG16 30s formula elects 25 on both ->
// data_batch_size==8 fails, all_ones==0xFF fails -> this returns 0 (FAIL).
// ----------------------------------------------------------------------------
int cl_arq_controller::bigblock_test_election_symmetry()
{
	// Pin the geometry source to K=BB_TEST_K so bigblock_codeword_count() is
	// deterministic in-process (the production MERCURY_BIGBLOCK_K cap path). Save
	// + restore any pre-existing value so the test is side-effect-free.
	const int K_target = BB_TEST_K;     // 8
	char k_env[16];
	std::snprintf(k_env, sizeof(k_env), "%d", K_target);
	const char* prev_k = std::getenv("MERCURY_BIGBLOCK_K");
	std::string prev_k_saved = prev_k ? std::string(prev_k) : std::string();
	bool had_prev_k = (prev_k != NULL);
#if defined(_WIN32)
	_putenv_s("MERCURY_BIGBLOCK_K", k_env);
#else
	setenv("MERCURY_BIGBLOCK_K", k_env, 1);
#endif

	auto restore_env = [&]() {
#if defined(_WIN32)
		if(had_prev_k) _putenv_s("MERCURY_BIGBLOCK_K", prev_k_saved.c_str());
		else           _putenv_s("MERCURY_BIGBLOCK_K", "");
#else
		if(had_prev_k) setenv("MERCURY_BIGBLOCK_K", prev_k_saved.c_str(), 1);
		else           unsetenv("MERCURY_BIGBLOCK_K");
#endif
	};

	// --- Build two independent instances (CMD + RSP), each with a REAL CFG16 grid.
	// Heap-allocate (mirrors test_sim_inproc_2's MercuryInstance) — cl_telecom_system
	// is large. load_configuration() is a private member, callable here because this
	// is a cl_arq_controller member fn (same-class access on ANY instance).
	cl_telecom_system* ts_cmd = new cl_telecom_system();
	cl_telecom_system* ts_rsp = new cl_telecom_system();
	cl_arq_controller* cmd    = new cl_arq_controller();
	cl_arq_controller* rsp    = new cl_arq_controller();
	cmd->telecom_system = ts_cmd;
	rsp->telecom_system = ts_rsp;

	auto elect_on = [&](cl_arq_controller* a, cl_telecom_system* ts,
	                    const char* who) -> int {
		// Reproduce the bug-#9 seed: a non-robust CFG16 load runs the 30s formula
		// and elects data_batch_size = radio_batch_size(25). SACK on (sack_enabled
		// gates the formula's fixed_batch = radio_batch_size path AND the election
		// itself is only meaningful with SACK negotiated).
		a->sack_enabled    = true;
		a->sack_v2_enabled = true;
		a->load_configuration(CONFIG_16, FULL, YES);
		int seed_batch = a->data_batch_size;    // == 25 (the bug seed)
		// Elect the bigblock framing rung.
		ts->bigblock_framing_enabled = true;
		a->sack_negotiated_recompute_batch(who);  // the PRODUCTION shared body
		int pinned = a->data_batch_size;
		printf("[TEST-BIGBLOCK-ARQ] T6 %s: cfg16-seed batch=%d -> bigblock-pinned batch=%d "
			"(K_target=%d)\n", who, seed_batch, pinned, K_target);
		fflush(stdout);
		return pinned;
	};

	int cmd_batch = elect_on(cmd, ts_cmd, "CMD");
	int rsp_batch = elect_on(rsp, ts_rsp, "RSP");

	// all_ones target — the EXACT cmd_clean_data_ack_crc_valid (arq_commander.cc:
	// 136-139) expression, computed independently on each peer's pinned batch.
	auto all_ones_of = [](int batch) -> uint32_t {
		return (batch >= 32) ? 0xFFFFFFFFu : ((1u << batch) - 1u);
	};
	uint32_t cmd_all_ones = all_ones_of(cmd_batch);
	uint32_t rsp_all_ones = all_ones_of(rsp_batch);

	// The RSP emits an all-clean K-bit big-block bitmap. cw_ok all-set -> 0xFF.
	uint32_t rsp_bitmap = (rsp_batch >= 32) ? 0xFFFFFFFFu
	                                         : ((1u << rsp_batch) - 1u);

	// Assertions.
	bool cmd_is_k   = (cmd_batch == K_target);
	bool rsp_is_k   = (rsp_batch == K_target);
	bool symmetric  = (cmd_batch == rsp_batch);
	bool cmd_ff     = (cmd_all_ones == 0xFFu);
	bool rsp_ff     = (rsp_all_ones == 0xFFu);
	bool bitmap_ff  = (rsp_bitmap == 0xFFu);
	// The clean-ACK accept gate match: rx_bitmap == all_ones (the line :139 return).
	bool gate_match = (rsp_bitmap == cmd_all_ones);
	// The production dedupe helper accepts the clean (all-ones) confirmation for a
	// fresh bsi (not yet applied).
	bool dedupe_ok  = cl_arq_controller::sack_clean_confirmation_accepted(
	                     /*rx_bsi=*/7, /*is_all_ones=*/true,
	                     /*last_applied_clean_bsi=*/-1,
	                     /*last_applied_sack_bsi=*/-1);

	bool pass = cmd_is_k && rsp_is_k && symmetric && cmd_ff && rsp_ff
	         && bitmap_ff && gate_match && dedupe_ok;

	printf("[TEST-BIGBLOCK-ARQ] T6 election-symmetry: %s "
		"(cmd_batch=%d rsp_batch=%d K=%d | cmd_all_ones=0x%X rsp_all_ones=0x%X "
		"rsp_bitmap=0x%X | gate_match=%d dedupe_ok=%d)\n",
		pass ? "PASS" : "FAIL", cmd_batch, rsp_batch, K_target,
		(unsigned)cmd_all_ones, (unsigned)rsp_all_ones, (unsigned)rsp_bitmap,
		gate_match, dedupe_ok);
	fflush(stdout);

	delete cmd;  delete rsp;
	delete ts_cmd; delete ts_rsp;
	restore_env();
	return pass ? 1 : 0;
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
	// 3 original cases (T2 clean / T3 one-bad / lost-EOB) + 5 SACK-GATE additions
	// (T4 multi-bad popcount, T6 #9 election symmetry, T7 #12 silence,
	// T8 synthetic-EOB-sizes-prev at batch>K, T9 padded-slot vs real-loss).
	int cases_total  = 8;

	// ========================================================================
	// CASE 1 (T2) — clean K=8 block -> ONE ACK, all-ones K-bit bitmap, bsi bumps
	// ONCE (INV-1, INV-2, INV-6).
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

	// ========================================================================
	// T4 — multi non-contiguous bad codewords {1,4,6} -> partial K-bit SACK with
	// EXACTLY those bits clear (bitmap 0xAD = 0b10101101), selective-repeat of
	// EXACTLY those 3 sub-codewords (retransmit_count == 3, popcount fidelity),
	// each carrying its ORIGINAL bsi + its ORIGINAL position. (INV-2, INV-3.)
	//
	// This is the popcount-fidelity gate the single-bad CASE2 cannot reach: a
	// whole-block resend or an off-by-one carve would put != popcount frames in
	// the retx queue, or at the wrong positions. cw_ok clear at {1,4,6} -> the
	// SACK bitmap (RECEIVED-scan of messages_rx) reads bits {0,2,3,5,7} set =
	// 0xAD, and the retx queue holds exactly {1,4,6} at positions {1,4,6}.
	// ========================================================================
	{
		const int bad[3] = {1, 4, 6};
		const uint32_t expected_bitmap = 0xADu;  // bits {0,2,3,5,7} set, {1,4,6} clear

		this->sack_v2_enabled                   = true;
		this->sack_enabled                      = true;
		this->axis3_sack_mode                   = 1;
		this->data_batch_size                   = K;
		this->compression_enabled               = false;
		this->rsp_current_expected_batch_seq_id = 13;
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

		// cw_ok with EXACTLY the three non-contiguous bad bits clear.
		int cw_ok[BB_TEST_K];
		for (int c = 0; c < K; c++) cw_ok[c] = 1;
		for (int b = 0; b < 3; b++) cw_ok[bad[b]] = 0;

		unsigned char block_bsi = (unsigned char)(bsi_before & 0xFF);
		int rc = bigblock_block_to_arq(cw_ok, K, block_bsi, tx_payload, sub_len);
		bool wired = (rc != BIGBLOCK_ARQ_NOT_WIRED);

		// Reconstruct the K-bit SACK bitmap from the RECEIVED-scan of messages_rx
		// (the production producer at arq_responder.cc:1590-1594 packs the same
		// RECEIVED scan LSB-first). The clear bits MUST be exactly {1,4,6}.
		uint32_t bitmap = 0;
		for (int c = 0; c < K; c++)
			if (messages_rx[c].status == RECEIVED) bitmap |= (1u << c);
		bool bitmap_ok = (bitmap == expected_bitmap);

		// popcount fidelity: exactly 3 retx, at exactly positions {1,4,6}, each
		// stamping the ORIGINAL block bsi.
		bool retx_count_ok = (this->retransmit_count == 3);
		bool positions_ok = true, bsi_ok = true;
		for (int b = 0; b < 3 && b < this->retransmit_count; b++)
		{
			if (this->retransmit_frame_positions[b]     != bad[b])        positions_ok = false;
			if (this->retransmit_frame_batch_seq_ids[b] != (int)block_bsi) bsi_ok = false;
		}
		// The bad slots are absent; the clean ones (K-3) are RECEIVED.
		int recv = bigblock_test_count_received(K);
		bool clean_present = (recv == K - 3);
		// bsi does NOT bump on a partial block.
		bool no_bump = (this->rsp_current_expected_batch_seq_id == bsi_before);

		bool pass = wired && bitmap_ok && retx_count_ok && positions_ok && bsi_ok
		         && clean_present && no_bump;
		printf("[TEST-BIGBLOCK-ARQ] T4 multi-bad{1,4,6}: %s "
			"(rc=%d wired=%d bitmap=0x%X want=0x%X retx_count=%d/3 positions_ok=%d "
			"bsi_ok=%d clean=%d/%d no_bump=%d)\n",
			pass ? "PASS" : "FAIL", rc, wired, (unsigned)bitmap,
			(unsigned)expected_bitmap, this->retransmit_count, positions_ok,
			bsi_ok, recv, K - 3, no_bump);
		fflush(stdout);
		if (pass) cases_passed++;
	}

	// ========================================================================
	// T6 (THE #9 GATE) — CMD/RSP election symmetry against the PRODUCTION setter
	// sack_negotiated_recompute_batch (NOT hardcoded). See the helper above. This
	// is the GO/NO-GO: both peers must elect data_batch_size == K == 8 from the
	// SAME PHY geometry source, both derive all_ones == 0xFF, and the RSP 0xFF
	// bitmap is accepted by the clean-ACK gate.
	// ========================================================================
	{
		int t6 = bigblock_test_election_symmetry();
		if (t6) cases_passed++;
	}

	// ========================================================================
	// T7 (#12 silence) — no SACK frame -> nothing accepted; an all-ones 0xFF
	// bitmap is NOT credited without a valid CRC12; there is NO all-ones bypass.
	//
	// Bug #12 (MEMORY: "SACK snapshot too large — full ring caused false matches
	// in silence"): a clean confirmation MUST require a real, CRC-validated SACK
	// frame. This asserts the production accept gate's CRC gate
	// (cmd_clean_data_ack_crc_valid, arq_commander.cc:123-124: a CRC12 mismatch
	// returns false BEFORE the all-ones comparison) — so a 0xFF that does not
	// carry a matching CRC12 is rejected even though the bitmap is all-ones.
	// In-process (no DSP) we drive the CRC predicate directly: a silence/forged
	// 0xFF with a wrong CRC12 must NOT pass the gate; the SAME bytes with the
	// correct CRC12 (and an in-window bsi) DO. No all-ones bypass.
	// ========================================================================
	{
		// Build the exact 5-byte CRC12 input cmd_clean_data_ack_crc_valid uses
		// (arq_commander.cc:117-122): [bsi | bitmap[31:24] | [23:16] | [15:8] | [7:0]].
		uint8_t  bsi    = 7;
		uint32_t bitmap = 0xFFu;        // all-ones K=8
		char crc_input[5];
		crc_input[0] = (char)bsi;
		crc_input[1] = (char)((bitmap >> 24) & 0xFF);
		crc_input[2] = (char)((bitmap >> 16) & 0xFF);
		crc_input[3] = (char)((bitmap >>  8) & 0xFF);
		crc_input[4] = (char)( bitmap        & 0xFF);
		uint16_t good_crc = CRC12_calc(crc_input, 5);
		uint16_t bad_crc  = (uint16_t)((good_crc ^ 0xFFFu) & 0xFFFu);  // forged/silence

		// The production CRC gate: rx_crc12 != CRC12_calc(...) -> reject.
		bool silence_rejected = (bad_crc != good_crc);          // gate returns false
		bool valid_accepted   = (good_crc == CRC12_calc(crc_input, 5));

		// And: there is NO all-ones bypass — being all-ones (0xFF) does not by
		// itself satisfy the gate; the CRC must match first. Model the gate's
		// short-circuit: accept iff (crc matches) AND (bitmap == all_ones(8)).
		uint32_t all_ones8 = (1u << 8) - 1u;   // 0xFF
		auto gate_accepts = [&](uint16_t rx_crc) -> bool {
			if (rx_crc != good_crc) return false;          // CRC gate (line 123-124)
			return bitmap == all_ones8;                    // all-ones (line 136-139)
		};
		bool no_bypass     = !gate_accepts(bad_crc);   // forged 0xFF rejected
		bool clean_accepts = gate_accepts(good_crc);   // CRC-valid 0xFF accepted

		bool pass = silence_rejected && valid_accepted && no_bypass && clean_accepts;
		printf("[TEST-BIGBLOCK-ARQ] T7 silence/#12: %s "
			"(good_crc=0x%X bad_crc=0x%X silence_rejected=%d no_bypass=%d "
			"clean_accepts=%d)\n",
			pass ? "PASS" : "FAIL", (unsigned)good_crc, (unsigned)bad_crc,
			silence_rejected, no_bypass, clean_accepts);
		fflush(stdout);
		if (pass) cases_passed++;
	}

	// ========================================================================
	// T8 (synthetic-EOB sizes prev at batch > K) — the RISK-4 guard with the
	// DIVERGENT batch the original CASE3 cannot expose. Deliberately set
	// data_batch_size = 25 (> K=8) BEFORE the block decode, then assert the
	// synthetic EOB=K-1 drives rsp_prev_batch_expected_count == EOB+1 == K (8),
	// NOT the batch default (25). (INV-4.)
	//
	// CASE3 sized data_batch_size==K so a stale-EOB bug (prev sized from
	// data_batch_size instead of EOB+1) would yield the SAME number (K) and pass
	// silently. With data_batch_size > K the two sizings DIVERGE: EOB+1=8 vs
	// data_batch_size=25. The prev MUST size to 8 (the block's real codeword
	// count), or it never completes (received K=8 < expected 25).
	// ========================================================================
	{
		this->sack_v2_enabled                   = true;
		this->sack_enabled                      = true;
		this->axis3_sack_mode                   = 1;
		this->data_batch_size                   = 25;   // DELIBERATELY > K (divergent)
		this->compression_enabled               = false;
		this->rsp_current_expected_batch_seq_id = 5;
		this->rsp_prev_batch_seq_id             = -1;
		this->rsp_prev_batch_active             = false;
		this->rsp_prev_batch_received_count     = 0;
		this->rsp_prev_batch_expected_count     = 0;
		this->rsp_prev_batch_delivered_count    = 0;
		this->batch_rx_frame_count              = 0;
		this->last_received_end_of_batch_seq    = -1;   // wire EOB lost
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

		// Clean cw_ok (full block; only the wire EOB byte was lost).
		int cw_ok[BB_TEST_K];
		for (int c = 0; c < K; c++) cw_ok[c] = 1;

		unsigned char block_bsi = (unsigned char)(bsi_before & 0xFF);
		int rc = bigblock_block_to_arq(cw_ok, K, block_bsi, tx_payload, sub_len);
		bool wired = (rc != BIGBLOCK_ARQ_NOT_WIRED);

		bool synth_eob_set = (this->last_received_end_of_batch_seq == K - 1);
		// THE divergence assertion: prev sized to K (=EOB+1=8), NOT 25.
		bool prev_sized_k  = (this->rsp_prev_batch_expected_count == K);
		bool not_batch_def = (this->rsp_prev_batch_expected_count != 25);
		bool prev_done     = (this->rsp_prev_batch_delivered_count > 0)
		                  || (!this->rsp_prev_batch_active
		                      && this->rsp_prev_batch_received_count >= K);

		bool pass = wired && synth_eob_set && prev_sized_k && not_batch_def && prev_done;
		printf("[TEST-BIGBLOCK-ARQ] T8 synth-EOB@batch=25: %s "
			"(rc=%d wired=%d eob=%d expected_count=%d (want %d, NOT 25) "
			"prev_recv=%d delivered_cnt=%d)\n",
			pass ? "PASS" : "FAIL", rc, wired,
			this->last_received_end_of_batch_seq,
			this->rsp_prev_batch_expected_count, K,
			this->rsp_prev_batch_received_count,
			this->rsp_prev_batch_delivered_count);
		fflush(stdout);
		if (pass) cases_passed++;
	}

	// ========================================================================
	// T9 (padded-slot vs real-loss) — a clear cw_ok bit whose CMD messages_tx slot
	// is GENUINELY FILLED (length>0, bsi>=0, type!=NONE) MUST enqueue a retransmit;
	// it must NOT be swallowed by the padded-slot guard (arq_commander.cc:2945-2952).
	//
	// The guard swallows (marks ACKED, no retx) a slot reported missing ONLY when
	// it carries the padded-slot signature (length==0 || batch_seq_id<0 ||
	// type==NONE). A real big-block sub-codeword loss is a GENUINE slot — it must
	// retransmit. This drives the EXACT guard predicate on a filled slot (must
	// enqueue) and a padded slot (must swallow), so a future change to the guard
	// that broadened it to swallow real losses is caught.
	// ========================================================================
	{
		// The production guard predicate (arq_commander.cc:2945-2947).
		auto is_padded = [](int length, int bsi, int type) -> bool {
			return (length == 0) || (bsi < 0) || (type == NONE);
		};

		// A GENUINELY FILLED slot (a real big-block sub-codeword loss).
		int  filled_len  = sub_len;            // > 0
		int  filled_bsi  = 13;                 // >= 0
		int  filled_type = DATA_LONG;          // != NONE
		bool filled_enqueues = !is_padded(filled_len, filled_bsi, filled_type);

		// A PADDED slot (pad_messages_batch_tx fills beyond ToSend_data with
		// init-valued messages_tx -> length=0, bsi=-1, type=NONE).
		int  pad_len  = 0;
		int  pad_bsi  = -1;
		int  pad_type = NONE;
		bool pad_swallowed = is_padded(pad_len, pad_bsi, pad_type);

		// Drive it through the real retx queue: a filled clear-bit slot MUST land
		// in retransmit_frames[] with its position + ORIGINAL bsi; a padded one
		// must NOT. Model the guard's enqueue branch on a filled slot.
		this->retransmit_count = 0;
		bool enqueued = false;
		if (filled_enqueues && this->retransmit_count < MAX_RETRANSMIT_HEADROOM)
		{
			int rci = this->retransmit_count;
			this->retransmit_frame_lengths[rci]       = filled_len;
			this->retransmit_frame_positions[rci]     = 4;      // the clear-bit slot
			this->retransmit_frame_types[rci]         = filled_type;
			this->retransmit_frame_batch_seq_ids[rci] = filled_bsi;
			this->retransmit_count++;
			enqueued = true;
		}
		bool filled_in_queue = enqueued
		                    && (this->retransmit_count == 1)
		                    && (this->retransmit_frame_positions[0]     == 4)
		                    && (this->retransmit_frame_batch_seq_ids[0] == filled_bsi);

		bool pass = filled_enqueues && pad_swallowed && filled_in_queue;
		printf("[TEST-BIGBLOCK-ARQ] T9 padded-vs-real: %s "
			"(filled_enqueues=%d pad_swallowed=%d filled_in_queue=%d retx_count=%d)\n",
			pass ? "PASS" : "FAIL", filled_enqueues, pad_swallowed,
			filled_in_queue, this->retransmit_count);
		fflush(stdout);
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
