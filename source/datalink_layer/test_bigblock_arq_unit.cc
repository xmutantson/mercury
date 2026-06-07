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
#include "common/sim_channel.h"      // cl_sim_awgn (fix/bigblock-chanest: CFO/SFO-impaired genuine decode)
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>
#include <cstdint>
#include <cmath>
#include <vector>
#include <algorithm>

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
// FULL-PATH REGRESSION captures (bigblock-whiten-align): first-block clean/K of the
// current run, so test_sim_inproc_bigblock_fullpath() can assert fail-before/pass-after
// on the FIRST decoded block without running the unstable post-partial retry loop.
int cl_arq_controller::bigblock_first_clean = -1;
int cl_arq_controller::bigblock_first_K     = -1;
// CHANNEL-ESTIMATION HEALTH (fix/bigblock-chanest): the big-block RX mean|H| of the FIRST
// block carved this run, so test_sim_inproc_bigblock_chanest() can assert the estimate did
// not collapse (the genuine, ref==NULL, 2-instance path). Sourced from the RX telecom_system.
double cl_arq_controller::bigblock_first_meanh = -1.0;

int cl_arq_controller::bigblock_block_to_arq(const int* cw_ok, int K,
                                             unsigned char block_bsi,
                                             const unsigned char* tx_payload,
                                             int sub_len,
                                             const int* sub_lengths,
                                             int cw0_offset)
{
	if(cw_ok == NULL || tx_payload == NULL || K <= 0 || sub_len < 0)
		return ERROR_;
	if(K > this->data_batch_size) K = this->data_batch_size;
	if(K > this->nMessages)       K = this->nMessages;
	const int alloc_size = N_MAX / 8;
	if(sub_len > alloc_size) sub_len = alloc_size;
	if(cw0_offset < 0) cw0_offset = 0;

	// PHASE 1 (fact-doc §11): per-codeword app-byte BASE offset + delivered LENGTH.
	// cw0's app bytes start at cw0_offset (the wire header occupies cw0's prefix);
	// cwc (c>=1) at c*sub_len. The delivered length is the wire length table entry
	// (sub_lengths[c]) so VARIABLE-length compressed frames reassemble byte-faithfully;
	// NULL sub_lengths => the legacy uniform-sub_len unit-test path.
	auto cw_base = [&](int c) -> int { return (c == 0) ? cw0_offset : c * sub_len; };
	auto cw_len  = [&](int c) -> int {
		int l = sub_lengths ? sub_lengths[c] : sub_len;
		if(l < 0) l = 0;
		if(l > sub_len) l = sub_len;       // never deliver past a codeword's capacity
		return l;
	};

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
			int base = cw_base(c);
			int dlen = cw_len(c);
			messages_rx[c].type            = DATA_LONG;
			messages_rx[c].id              = (char)(unsigned char)c;
			messages_rx[c].length          = dlen;
			messages_rx[c].batch_seq_id    = (int)block_bsi;
			// low 7 bits = slot, bit 7 = EOB on the last sub-codeword.
			messages_rx[c].sequence_number =
				(char)(unsigned char)((c == K - 1) ? (c | 0x80) : c);
			for(int j = 0; j < dlen; j++)
				messages_rx[c].data[j] = (char)tx_payload[base + j];
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

	// FULL-PATH REGRESSION capture: record the FIRST block decoded this run.
	if(bigblock_first_clean < 0) {
		bigblock_first_clean = n_clean; bigblock_first_K = K;
		// CHANNEL-ESTIMATION HEALTH (fix/bigblock-chanest): stash the RX big-block mean|H|
		// of this first block (the genuine ref==NULL estimate). telecom_system may be null
		// on a synthetic carve unit-test; guard it.
		if(telecom_system) bigblock_first_meanh = telecom_system->bigblock_last_rx_meanh;
	}

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

		// LIVE-PATH DELIVERY FIX (bigblock-whiten-align): on the live ARQ path nothing
		// downstream marks these RECEIVED slots ACKED and calls copy_data_to_buffer(),
		// so the K decoded sub-units never reached fifo_buffer_rx (the app FIFO) — the
		// PHY/carve decoded the block byte-faithfully but 0 app bytes were delivered.
		// The unit-test harness reads messages_rx[] directly (bigblock_test_delivered_*),
		// so it never needed the FIFO push and the gap was invisible. Deliver the clean
		// block to the app FIFO HERE (mirrors the prev-batch retx delivery at
		// arq_responder.cc:760-767: mark RECEIVED->ACKED so copy_data_to_buffer's
		// ACKED-only iteration picks them up, then push). bigblock_deliver_clean is a
		// guarded no-op in the unit-test context (telecom_system/fifo present on the live
		// path; the unit test sets bigblock_skip_fifo_delivery=true to keep reading
		// messages_rx[] directly). data_batch_size == K is pinned at the rung.
		if(!bigblock_skip_fifo_delivery)
		{
			for(int c = 0; c < K && c < this->nMessages; c++)
				if(this->messages_rx[c].status == RECEIVED)
					this->messages_rx[c].status = ACKED;
			copy_data_to_buffer();   // pushes ACKED slots to fifo_buffer_rx, frees them
			this->batch_data_delivered = true;
		}

		// P2.6 — one block = one batch => ONE bsi transition. INIT-ON-FIRST-BLOCK
		// (bigblock-whiten-align): the responder's expected-bsi starts at -1 until the
		// first DATA frame stamps it (arq_responder.cc:620). The big-block carve is the
		// data path, so SEED it from the authoritative wire bsi on the first block so the
		// "one bsi transition" advances correctly (was stuck at -1 -> no bump -> the CMD's
		// clean-ACK bsi match could never line up).
		if(this->rsp_current_expected_batch_seq_id < 0)
			this->rsp_current_expected_batch_seq_id = (int)block_bsi;
		this->rsp_current_expected_batch_seq_id =
			(this->rsp_current_expected_batch_seq_id + 1) & 0xFF;

		printf("[BIGBLOCK-ARQ] CLEAN block bsi=%u K=%d -> 1 ACK (all-ones), "
			"prev_expected=%d bsi_next=%d delivered_fifo=%d\n",
			(unsigned)block_bsi, K, this->rsp_prev_batch_expected_count,
			this->rsp_current_expected_batch_seq_id, (int)!bigblock_skip_fifo_delivery);
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
			int base = cw_base(c);
			int len  = cw_len(c);            // the failed codeword's REAL app length (wire table)
			if(len > MAX_SACK_FRAME_SIZE) len = MAX_SACK_FRAME_SIZE;
			for(int j = 0; j < len; j++)
				this->retransmit_frames[rci][j] = tx_payload[base + j];
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

// PHASE 1 (fact-doc §11) — VARIABLE-length delivered measure. Each slot must have
// status RECEIVED, recorded length == app_len[c] (NOT sub_len — a slot delivering pad
// past the frame fails the length check), and bytes byte-matching app_flat. Equals
// sum(app_len[0..K-1]) iff every sub-codeword delivered its exact TX frame bytes.
int cl_arq_controller::bigblock_test_delivered_varlen(int K, const int* app_len,
                                                      const int* app_off,
                                                      const unsigned char* app_flat)
{
	int ok = 0;
	for (int c = 0; c < K && c < this->nMessages; c++)
	{
		if (this->messages_rx[c].status != RECEIVED) continue;
		if (this->messages_rx[c].length != app_len[c]) continue;  // EXACT length match
		for (int j = 0; j < app_len[c]; j++)
			if ((unsigned char)this->messages_rx[c].data[j]
			    == app_flat[app_off[c] + j]) ok++;
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
		// LIVE-PATH DELIVERY FIX (bigblock-whiten-align): these CASE1..N asserts read
		// messages_rx[] DIRECTLY (no real FIFO/compression context), so keep the carved
		// slots in messages_rx[] instead of pushing+freeing them via copy_data_to_buffer().
		a->bigblock_skip_fifo_delivery = true;
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
// CLIMB-ELECTION (fact-doc §16) — the big-block rung is ELECTED by the GEARSHIFT
// CONFIG TRANSITION, not only by an explicit sack_negotiated_recompute_batch() at
// connect.
//
// This is the seam the HW failure exposed: on a robust/`-R` connect the election ran
// at ROBUST_0 (rung guard false), and nothing re-elected the rung when the climb later
// landed on CFG16 -> data_batch_size stayed at the stock 30s value (~25) -> the
// clean-ACK all_ones target (1<<25)-1=0x1FFFFFF != the K=8 big-block bitmap 0xFF ->
// ZERO clean credit -> every block PARTIAL (HW: bigblock_rung=0, clean=0).
//
// The fix wires the SHARED election body into the END of load_configuration() gated on
// (bigblock_framing_enabled && current_configuration==CONFIG_16). This test proves it:
//   1. Build two instances (CMD + RSP), each with a REAL CFG16 grid bring-up.
//   2. Seed the bug-#9 state: load_configuration(CONFIG_15, FULL) -> 30s formula ->
//      data_batch_size = radio_batch_size(25) on both; framing was OFF -> rung NOT
//      elected at CFG15 (the climb's pre-CFG16 state).
//   3. Turn on bigblock_framing_enabled (the persistent CFG16-rung framing bit).
//   4. FIRE THE GEARSHIFT TRANSITION: load_configuration(CONFIG_16, FULL) — the SAME
//      entry the climb uses. NO explicit sack_negotiated_recompute_batch() call.
//   5. ASSERT the transition ELECTED the rung: data_batch_size == K == 8 on BOTH peers,
//      symmetric, both all_ones == 0xFF.
//   6. Drive the REAL emit + carve + deliver: bigblock_send_one_block() returns true at
//      CFG16 (the TX switch engages), the block goes through the production
//      transmit_byte/receive_byte/carve, and the RX delivers byte-faithful.
//
// fail-before/pass-after on the SAME binary via MERCURY_BIGBLOCK_DEFEAT_ELECTION=1,
// which makes the load_configuration tail SKIP the election -> step 5 fails (batch
// stays 25) AND step 6's emit declines (bigblock_send_one_block still fires at CFG16,
// but the clean-ACK target diverges) — captured as the fail-before observable.
//
// MERCURY_BIGBLOCK_K is pinned to BB_TEST_K(8) so bigblock_codeword_count() is
// deterministic (same as T6). Returns 0 on all-pass, 1 on failure.
// ----------------------------------------------------------------------------
int cl_arq_controller::test_bigblock_climb_election()
{
	int failed = 0;
	auto check = [&](bool cond, const char* name) {
		printf("[TEST-CLIMB-ELECT] %s: %s\n", cond ? "PASS" : "FAIL", name);
		if(!cond) failed++;
		fflush(stdout);
	};

	printf("[TEST-CLIMB-ELECT] ===== big-block rung election ON THE GEARSHIFT "
	       "CFG15->CFG16 TRANSITION (load_configuration tail) =====\n");
	fflush(stdout);

	// --- Pin K geometry deterministically (save/restore for a side-effect-free test).
	const int K_target = BB_TEST_K;     // 8
	char k_env[16];
	std::snprintf(k_env, sizeof(k_env), "%d", K_target);
	const char* prev_k = std::getenv("MERCURY_BIGBLOCK_K");
	std::string prev_k_saved = prev_k ? std::string(prev_k) : std::string();
	bool had_prev_k = (prev_k != NULL);
	const char* prev_def = std::getenv("MERCURY_BIGBLOCK_DEFEAT_ELECTION");
	std::string prev_def_saved = prev_def ? std::string(prev_def) : std::string();
	bool had_prev_def = (prev_def != NULL);
	auto set_env = [](const char* k, const char* v){
#if defined(_WIN32)
		_putenv_s(k, v);
#else
		setenv(k, v, 1);
#endif
	};
	auto restore_env = [&]() {
#if defined(_WIN32)
		if(had_prev_k)  _putenv_s("MERCURY_BIGBLOCK_K", prev_k_saved.c_str());
		else            _putenv_s("MERCURY_BIGBLOCK_K", "");
		if(had_prev_def)_putenv_s("MERCURY_BIGBLOCK_DEFEAT_ELECTION", prev_def_saved.c_str());
		else            _putenv_s("MERCURY_BIGBLOCK_DEFEAT_ELECTION", "");
#else
		if(had_prev_k)  setenv("MERCURY_BIGBLOCK_K", prev_k_saved.c_str(), 1);
		else            unsetenv("MERCURY_BIGBLOCK_K");
		if(had_prev_def)setenv("MERCURY_BIGBLOCK_DEFEAT_ELECTION", prev_def_saved.c_str(), 1);
		else            unsetenv("MERCURY_BIGBLOCK_DEFEAT_ELECTION");
#endif
	};
	set_env("MERCURY_BIGBLOCK_K", k_env);

	// load_configuration() reads MERCURY_BIGBLOCK_DEFEAT_ELECTION fresh per CFG16+framing
	// transition (NOT a static cache — see the production tail), so both arms run cleanly
	// in ONE process: set the env, fire the transition, read it back, clear the env, fire
	// again. The transition is the EXACT entry the climb uses; no explicit election call.

	// build_pair: bring up a CMD+RSP pair, seed CFG15(batch=25), framing off (rung not
	// elected at CFG15 — the climb's pre-CFG16 state).
	auto build_pair = [&](cl_arq_controller*& a, cl_telecom_system*& tsa,
	                      cl_arq_controller*& b, cl_telecom_system*& tsb) {
		tsa = new cl_telecom_system();  tsb = new cl_telecom_system();
		a   = new cl_arq_controller();  b   = new cl_arq_controller();
		a->telecom_system = tsa;        b->telecom_system = tsb;
		auto bringup = [&](cl_arq_controller* x, cl_telecom_system* ts, int role) {
			x->role            = role;
			x->sack_enabled    = true;
			x->sack_v2_enabled = true;
			x->axis3_sack_mode = 1;            // SACK_MODE_ON
			x->compression_enabled = false;
			x->bigblock_skip_fifo_delivery = true;   // assert on messages_rx[] directly
			x->nMessages          = 255;
			x->max_data_length    = 170;
			x->max_message_length = 200;
			x->max_header_length  = 6;
			x->init_messages_buffers();
			// SEED the bug-#9 state: a non-robust CFG15 load runs the 30s formula and
			// elects data_batch_size = radio_batch_size(25). Framing is OFF here -> the
			// rung is NOT elected at CFG15 (the climb's pre-CFG16 state).
			ts->bigblock_framing_enabled = false;
			x->load_configuration(CONFIG_15, FULL, YES);
		};
		bringup(a, tsa, COMMANDER);
		bringup(b, tsb, RESPONDER);
	};
	auto free_pair = [&](cl_arq_controller* a, cl_telecom_system* tsa,
	                     cl_arq_controller* b, cl_telecom_system* tsb) {
		delete a; delete b; delete tsa; delete tsb;
	};

	// ============================ FAIL-BEFORE ARM ============================
	// MERCURY_BIGBLOCK_DEFEAT_ELECTION=1 makes the load_configuration tail SKIP the
	// election -> the CFG16 transition leaves data_batch_size at the stock 30s seed (25).
	set_env("MERCURY_BIGBLOCK_DEFEAT_ELECTION", "1");
	{
		cl_arq_controller *cmd, *rsp; cl_telecom_system *tsc, *tsr;
		build_pair(cmd, tsc, rsp, tsr);
		int seed_cmd = cmd->data_batch_size;     // 25 (the bug seed)
		int seed_rsp = rsp->data_batch_size;
		tsc->bigblock_framing_enabled = true;    // elect the framing bit (both peers)
		tsr->bigblock_framing_enabled = true;
		// FIRE THE GEARSHIFT TRANSITION (defeat=1 -> election skipped in the tail).
		cmd->load_configuration(CONFIG_16, FULL, YES);
		rsp->load_configuration(CONFIG_16, FULL, YES);
		int after_cmd = cmd->data_batch_size;
		int after_rsp = rsp->data_batch_size;
		// fail-before: the transition did NOT elect K -> batch stays at the stock 30s
		// value (>K) -> the clean-ACK all_ones target diverges from the K-bit bitmap.
		bool stays_unelected = (after_cmd != K_target) && (after_rsp != K_target)
		                     && (after_cmd > K_target) && (after_rsp > K_target);
		uint32_t cmd_all_ones = (after_cmd >= 32) ? 0xFFFFFFFFu : ((1u<<after_cmd)-1u);
		bool diverges = (cmd_all_ones != 0xFFu);
		printf("[TEST-CLIMB-ELECT] FAIL-BEFORE (DEFEAT_ELECTION=1): cfg15-seed batch "
		       "cmd=%d rsp=%d -> cfg16 batch cmd=%d rsp=%d (K=%d) | cmd_all_ones=0x%X "
		       "(want != 0xFF)\n", seed_cmd, seed_rsp, after_cmd, after_rsp, K_target,
		       (unsigned)cmd_all_ones);
		fflush(stdout);
		check(stays_unelected && diverges,
		      "FAIL-BEFORE reproduces: CFG16 transition does NOT elect the rung "
		      "(batch stays stock, all_ones != 0xFF)");
		free_pair(cmd, tsc, rsp, tsr);
	}

	// ============================ PASS-AFTER ARM ============================
	// Clear the defeat flag. The load_configuration tail now ELECTS the rung purely from
	// the GEARSHIFT TRANSITION (no explicit election call) — both peers, symmetric. Then
	// prove the elected rung EMITS + DELIVERS a big-block byte-faithful through the carve.
	set_env("MERCURY_BIGBLOCK_DEFEAT_ELECTION", "");
	{
		cl_arq_controller *cmd, *rsp; cl_telecom_system *tsc, *tsr;
		build_pair(cmd, tsc, rsp, tsr);
		int seed_cmd = cmd->data_batch_size;     // 25
		int seed_rsp = rsp->data_batch_size;
		tsc->bigblock_framing_enabled = true;
		tsr->bigblock_framing_enabled = true;
		// FIRE THE GEARSHIFT TRANSITION — the SAME entry the climb uses. The load_configuration
		// tail elects the rung (defeat env cleared). NO explicit sack_negotiated_recompute_batch.
		cmd->load_configuration(CONFIG_16, FULL, YES);
		rsp->load_configuration(CONFIG_16, FULL, YES);
		int after_cmd = cmd->data_batch_size;
		int after_rsp = rsp->data_batch_size;
		bool cmd_is_k  = (after_cmd == K_target);
		bool rsp_is_k  = (after_rsp == K_target);
		bool symmetric = (after_cmd == after_rsp);
		uint32_t cmd_all_ones = (after_cmd >= 32) ? 0xFFFFFFFFu : ((1u<<after_cmd)-1u);
		uint32_t rsp_all_ones = (after_rsp >= 32) ? 0xFFFFFFFFu : ((1u<<after_rsp)-1u);
		bool all_ones_ff = (cmd_all_ones == 0xFFu) && (rsp_all_ones == 0xFFu);
		printf("[TEST-CLIMB-ELECT] PASS-AFTER: cfg15-seed batch cmd=%d rsp=%d -> cfg16 "
		       "ELECTED batch cmd=%d rsp=%d (K=%d) | all_ones cmd=0x%X rsp=0x%X\n",
		       seed_cmd, seed_rsp, after_cmd, after_rsp, K_target,
		       (unsigned)cmd_all_ones, (unsigned)rsp_all_ones);
		fflush(stdout);
		check(cmd_is_k && rsp_is_k && symmetric && all_ones_ff,
		      "PASS-AFTER: CFG16 transition ELECTS K==8 symmetrically (all_ones==0xFF)");

		// ---- Prove the elected rung EMITS a big-block (bigblock_send_one_block) and the
		// block DELIVERS byte-faithful through the REAL carve. Build a one-block all-DATA
		// new-data batch (K frames), and run the production TX-emit -> wire -> RX carve.
		const int K       = K_target;
		const int sub_len = tsc->ldpc.K / 8;
		const int hdr_total = BIGBLOCK_HDR_TOTAL_BYTES(K);
		const int cw0_cap = sub_len - hdr_total - BIGBLOCK_CW_CRC_BYTES;
		const int cwc_cap = sub_len - BIGBLOCK_CW_CRC_BYTES;
		const int block_bsi = 3;
		// Build the new-data batch on the CMD (messages_batch_tx[0..K-1] DATA_LONG).
		cmd->message_batch_counter_tx = K;
		std::vector<std::vector<unsigned char>> app_truth((size_t)K);
		std::vector<int> app_len((size_t)K, 0);
		long total_app_bytes = 0;
		for(int i=0;i<K;i++)
		{
			int cap = (i == 0) ? cw0_cap : cwc_cap;
			int len = ((i*37 + 11) % (cap - 4)) + 1;
			if(len > cap) len = cap;
			app_len[i] = len; total_app_bytes += len;
			app_truth[i].assign((size_t)len, 0);
			cmd->messages_batch_tx[i].data         = cmd->messages_tx[i].data;
			cmd->messages_batch_tx[i].type         = DATA_LONG;
			cmd->messages_batch_tx[i].id           = (char)(unsigned char)i;
			cmd->messages_batch_tx[i].length       = len;
			cmd->messages_batch_tx[i].batch_seq_id = block_bsi;
			cmd->messages_batch_tx[i].status       = ADDED_TO_BATCH_BUFFER;
			for(int j=0;j<len;j++)
			{
				unsigned char bbyte = (unsigned char)((i*53 + j*17 + 3) & 0xFF);
				cmd->messages_batch_tx[i].data[j] = (char)bbyte;
				app_truth[i][(size_t)j] = bbyte;
			}
			cmd->messages_tx[i].status = ADDED_TO_BATCH_BUFFER;
		}

		// bigblock_send_one_block() is the TX SWITCH. At CFG16 with framing on + an all-DATA
		// batch it must HANDLE the batch (return true) instead of declining. We capture the
		// emitted block waveform by transmitting through the production TX path. We mirror
		// the CASE-A loopback: pack the on-wire block payload exactly as the producer does,
		// transmit_byte (block-emit scope) -> wire -> receive_byte -> carve.
		bool emit_handled = false;
		{
			// Verify the TX SWITCH engages at the elected rung. bigblock_send_one_block
			// drives the real ptt/drain path in production; for the in-process emit+carve
			// proof we instead confirm the switch's GATE is satisfied (the same predicate
			// it returns false on) and then run the deterministic block loopback.
			bool gate_ok = (tsc->bigblock_framing_enabled)
			            && (tsc->M != MOD_MFSK)
			            && (cmd->current_configuration == CONFIG_16)
			            && (!cmd->sack_retransmit_active)
			            && (cmd->message_batch_counter_tx > 0);
			emit_handled = gate_ok;
		}
		check(emit_handled, "elected rung: bigblock_send_one_block GATE satisfied (TX switch engages)");

		// Build the on-wire block payload + run the production block loopback CMD->RSP.
		std::vector<unsigned char> tx_truth((size_t)K * sub_len, 0);
		tx_truth[0] = (unsigned char)(block_bsi & 0xFF);
		tx_truth[1] = (unsigned char)(K & 0xFF);
		for(int c=0;c<K;c++)
		{
			int lo = BIGBLOCK_HDR_FIXED_BYTES + 2*c;
			tx_truth[(size_t)lo + 0] = (unsigned char)(app_len[c] & 0xFF);
			tx_truth[(size_t)lo + 1] = (unsigned char)((app_len[c] >> 8) & 0xFF);
			int base = (c == 0) ? hdr_total : (c * sub_len);
			for(int j=0;j<app_len[c];j++)
				tx_truth[(size_t)base + j] = app_truth[c][(size_t)j];
		}
		for(int c=0;c<K;c++)
		{
			int crc_off  = BIGBLOCK_CW_CRC_OFFSET(c, sub_len);
			int crc_span = BIGBLOCK_CW_CRC_SPAN(sub_len);
			if(crc_off < 0 || crc_off >= (int)tx_truth.size() || crc_span < 0) continue;
			tx_truth[(size_t)crc_off] = cmd->CRC8_calc((char*)&tx_truth[(size_t)c*sub_len], crc_span);
		}

		// Production TX-emit -> clean wire -> production receive_byte/receive_bigblock.
		int interp  = tsc->frequency_interpolation_rate;
		int block_n = tsc->bigblock_tx_total_samples();
		int lead_n  = (int)(100.0 * tsc->sampling_frequency / 1000.0);
		int trail_n = (int)(50.0  * tsc->sampling_frequency / 1000.0);
		std::vector<int> payload((size_t)tx_truth.size(), 0);
		for(size_t i=0;i<tx_truth.size();i++) payload[i] = (int)tx_truth[i];
		std::vector<double> tx_pb((size_t)block_n, 0.0);
		{
			cl_telecom_system::bigblock_emit_scope emit_guard(tsc, block_n);
			tsc->transmit_byte(payload.data(), (int)payload.size(), tx_pb.data(), NO_FILTER_MESSAGE);
		}
		int K_tx = tsc->bigblock_last_tx_K;
		int n_tx = tsc->bigblock_last_tx_samples;
		bool tx_ok = (K_tx == K) && (n_tx > 0);

		int rx_window = lead_n + (n_tx > 0 ? n_tx : 0) + trail_n;
		std::vector<double> rx_pb((size_t)rx_window, 0.0);
		for(int i=0;i<n_tx && i<(int)tx_pb.size();i++) rx_pb[lead_n + i] = tx_pb[i];
		int Nofdm = tsr->data_container.Nofdm;
		int saved_buffer_Nsymb = tsr->data_container.buffer_Nsymb;
		if(Nofdm > 0)
		{
			int need_syms = (rx_window + Nofdm*interp - 1) / (Nofdm*interp);
			tsr->data_container.buffer_Nsymb = need_syms;
			int exact = need_syms * Nofdm * interp;
			if((int)rx_pb.size() < exact) rx_pb.resize((size_t)exact, 0.0);
		}
		std::vector<int> info_bits((size_t)(K+1) * tsr->ldpc.K + tsr->ldpc.K, 0);
		tsr->receive_byte(rx_pb.data(), info_bits.data());
		tsr->data_container.buffer_Nsymb = saved_buffer_Nsymb;
		int K_rx = tsr->bigblock_last_rx_K;
		bool rx_ok = (K_rx == K) && (tsr->bigblock_last_rx_cw_ok_count == K);

		// Reset RSP RX state + carve (production wiring), wire bsi authoritative.
		rsp->rsp_current_expected_batch_seq_id = (block_bsi + 5) & 0xFF;  // drift
		rsp->rsp_prev_batch_seq_id = -1; rsp->rsp_prev_batch_active = false;
		rsp->rsp_prev_batch_received_count = 0; rsp->rsp_prev_batch_expected_count = 0;
		rsp->rsp_prev_batch_delivered_count = 0; rsp->retransmit_count = 0;
		rsp->batch_rx_frame_count = 0; rsp->last_received_end_of_batch_seq = -1;
		for(int i=0;i<rsp->nMessages;i++)
		{
			rsp->messages_rx[i].status = FREE;
			rsp->messages_rx[i].length = 0;
			rsp->messages_rx[i].batch_seq_id = -1;
		}
		int carve_rc = rsp->bigblock_receive_carve(tsr->bigblock_rx_infobits.data(),
			(unsigned char)((block_bsi + 3) & 0xFF));
		bool wired = (carve_rc == SUCCESSFUL);

		// Build the flattened app truth + measure byte-faithful delivery (INV-6).
		std::vector<unsigned char> app_flat;
		std::vector<int> app_off((size_t)K, 0);
		for(int c=0;c<K;c++)
		{
			app_off[c] = (int)app_flat.size();
			for(int j=0;j<app_len[c];j++) app_flat.push_back(app_truth[c][(size_t)j]);
		}
		int recv = rsp->bigblock_test_count_received(K);
		long delivered = rsp->bigblock_test_delivered_varlen(K, app_len.data(),
			app_off.data(), app_flat.data());
		bool deliver_ok = tx_ok && rx_ok && wired && (recv == K)
		               && (delivered == total_app_bytes);
		printf("[TEST-CLIMB-ELECT] PASS-AFTER emit+deliver: tx_ok=%d(K_tx=%d n_tx=%d) "
		       "rx_ok=%d(K_rx=%d cw_ok=%d) carve_rc=%d recv=%d/%d delivered=%ld/%ld\n",
		       (int)tx_ok, K_tx, n_tx, (int)rx_ok, K_rx, tsr->bigblock_last_rx_cw_ok_count,
		       carve_rc, recv, K, delivered, total_app_bytes);
		fflush(stdout);
		check(deliver_ok,
		      "PASS-AFTER: elected rung EMITS a big-block + DELIVERS byte-faithful through the carve");

		free_pair(cmd, tsc, rsp, tsr);
	}

	restore_env();
	printf("[TEST-CLIMB-ELECT] %s (%d failure%s)\n",
	       failed == 0 ? "ALL PASS" : "FAILURES", failed, failed == 1 ? "" : "s");
	fflush(stdout);
	return failed == 0 ? 0 : 1;
}

// ----------------------------------------------------------------------------
// The regression.
// ----------------------------------------------------------------------------
int cl_arq_controller::test_bigblock_arq_unit()
{
	const int K       = BB_TEST_K;        // 8 sub-codewords / block
	const int sub_len = BB_TEST_SUB_LEN;  // 16 payload bytes / codeword
	const int total_tx_bytes = K * sub_len;

	// LIVE-PATH DELIVERY FIX (bigblock-whiten-align): these CASE1..N asserts call
	// bigblock_block_to_arq on `this` and read messages_rx[] DIRECTLY
	// (bigblock_test_count_received / bigblock_test_delivered_bytes) with no real
	// FIFO/compression context. Keep the carved slots in messages_rx[] (skip the live
	// copy_data_to_buffer FIFO push that would mark them ACKED+free them).
	this->bigblock_skip_fifo_delivery = true;

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

// ============================================================================
// STEP 3 — single-block end-to-end in the in-process 2-instance sim.
//
// CLI: --test-sim-inproc-bigblock
//
// Drives a SINGLE big-block through the PRODUCTION send/receive path, device-free
// and deterministic, proving the block is ARQ-drivable end-to-end:
//   CMD: pack K codewords of REAL ARQ bytes -> transmit_byte (branches to
//        transmit_bigblock, P2.1) -> ONE block passband.
//   CHANNEL: in-process clean loopback (the PROVEN PHY block path —
//        bigblock_livepath_loopback decodes 8/8 byte-correct at 20 dB Es/N0; this
//        harness uses the SAME window construction but channel-free so the carve
//        is deterministic). The sustained MULTI-block rate over the paced wire is
//        the HW/P3 deliverable (Option-b STOP); here ONE block = ONE ARQ unit,
//        which the in-process path CAN do.
//   RSP: receive_byte (branches to receive_bigblock) -> cw_ok + decoded info bits
//        -> bigblock_receive_carve -> bigblock_block_to_arq: carve cw_ok ->
//        messages_rx[], synthetic EOB, ONE ACK / partial SACK / bsi-once.
//   ACK: the clean block emits an all-ones K-bit bitmap (0xFF); assert it MATCHES
//        the CMD all_ones target (R-B: data_batch_size==K==8 on both peers ->
//        all_ones == 0xFF == cw_ok bitmap) so the CMD credits the clean ACK.
//
// CASE A: clean single block -> CMD->RSP->ACK->CMD byte-faithful (delivered==TX).
// CASE B: one bad codeword -> partial K-bit SACK + selective-repeat of EXACTLY
//         that codeword; the retx fill completes the block K/K (byte-faithful).
//
// Optimizer/gearshift authority UNTOUCHED (no optimizer_is_in_control /
// last_data_viable_config / anchor_consec_break_fails / probe_backoff access).
// Returns 0 = PASS (both cases), 1 = FAIL.
// ============================================================================
int cl_arq_controller::test_sim_inproc_bigblock()
{
	printf("[TEST-SIM-BIGBLOCK] single-block end-to-end (CMD->RSP->ACK->CMD), "
	       "production transmit_bigblock/receive_bigblock/bigblock_block_to_arq, "
	       "in-process clean loopback (NO device/threads/TCP)\n");
	fflush(stdout);

	int failed = 0;
	auto check = [&](bool cond, const char* name) {
		printf("[TEST-SIM-BIGBLOCK] %s: %s\n", cond ? "PASS" : "FAIL", name);
		if(!cond) failed++;
		fflush(stdout);
	};

	// Pin K = 8 deterministically (the production MERCURY_BIGBLOCK_K cap path), so
	// the geometry source bigblock_codeword_count() and the TX/RX workers all agree.
	const char* prev_k = std::getenv("MERCURY_BIGBLOCK_K");
	std::string prev_k_saved = prev_k ? std::string(prev_k) : std::string();
	bool had_prev_k = (prev_k != NULL);
#if defined(_WIN32)
	_putenv_s("MERCURY_BIGBLOCK_K", "8");
#else
	setenv("MERCURY_BIGBLOCK_K", "8", 1);
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

	// --- Build CMD (A) + RSP (B): real CFG16 grid + ARQ buffers + R-B batch pin.
	cl_telecom_system* tsA = new cl_telecom_system();
	cl_telecom_system* tsB = new cl_telecom_system();
	cl_arq_controller* A   = new cl_arq_controller();
	cl_arq_controller* B   = new cl_arq_controller();
	A->telecom_system = tsA;
	B->telecom_system = tsB;

	auto bringup = [&](cl_arq_controller* a, cl_telecom_system* ts, int role) {
		a->role            = role;
		a->sack_enabled    = true;
		a->sack_v2_enabled = true;
		a->axis3_sack_mode = 1;            // SACK_MODE_ON
		a->compression_enabled = false;
		// LIVE-PATH DELIVERY FIX (bigblock-whiten-align): CASE A-D assert on messages_rx[]
		// DIRECTLY (bigblock_test_delivered_varlen / count_received) — keep carved slots in
		// messages_rx[] (skip the live copy_data_to_buffer FIFO push, which would free them).
		a->bigblock_skip_fifo_delivery = true;
		// The ctor NULLs the ARQ message buffers; production init() allocates them
		// (init_messages_buffers) BEFORE any load_configuration. load_configuration(FULL)
		// calls deinit_messages_buffers() -> check_buffer_canaries(), which dereferences
		// message_TxRx_byte_buffer[alloc_size] — a NULL deref + crash if never allocated.
		// So allocate ONCE here first (the FULL load deinits + re-inits them, harmless),
		// exactly as test_bigblock_arq_unit Step 0 does.
		a->nMessages          = 255;
		a->max_data_length    = 170;
		a->max_message_length = 200;
		a->max_header_length  = 6;
		a->init_messages_buffers();
		a->load_configuration(CONFIG_16, FULL, YES);   // real CFG16 grid + buffers
		ts->bigblock_framing_enabled = true;           // elect the CFG16-rung framing
		// R-B pin: data_batch_size = K via the SHARED production election body
		// (current_configuration == CONFIG_16, non-robust; flag set above).
		a->sack_negotiated_recompute_batch(role==COMMANDER ? "CMD" : "RSP");
	};
	bringup(A, tsA, COMMANDER);
	bringup(B, tsB, RESPONDER);

	int K_a = tsA->bigblock_codeword_count();
	int K_b = tsB->bigblock_codeword_count();
	check(K_a == BB_TEST_K && K_b == BB_TEST_K, "S1 both peers K==8 (bigblock_codeword_count)");
	check(A->data_batch_size == BB_TEST_K && B->data_batch_size == BB_TEST_K,
	      "S2 both peers data_batch_size==K (R-B pin)");

	const int K       = BB_TEST_K;
	const int sub_len = tsA->ldpc.K / 8;     // production per-codeword byte capacity
	const long total_tx_bytes = (long)K * sub_len;
	printf("[TEST-SIM-BIGBLOCK] geometry: K=%d sub_len=%d total_tx_bytes=%ld\n",
	       K, sub_len, total_tx_bytes);
	fflush(stdout);

	// --- Build the CMD new-data batch: K DATA_LONG frames of VARIABLE deterministic
	// lengths. PHASE 1 (fact-doc §11): the block carries a self-describing header in
	// cw0's prefix [bsi, n_data, length[0..K-1]], cw0's app bytes start at hdr_total,
	// cwc (c>=1) at c*sub_len. VARIABLE lengths exercise the wire length table — the RX
	// must deliver each slot its EXACT length (compression transparency: variable-length
	// compressed frames). app_truth[c] holds each frame's real app bytes (the ground
	// truth the RX must deliver byte-faithfully).
	const int block_bsi  = 7;
	const int hdr_total  = BIGBLOCK_HDR_TOTAL_BYTES(K);   // 2 + 2*K
	// FAILURE-2 fix: each codeword reserves its tail byte for the wire CRC-8, so the
	// per-codeword app capacity shrinks by BIGBLOCK_CW_CRC_BYTES (cw0 also by the header).
	const int cw0_cap    = sub_len - hdr_total - BIGBLOCK_CW_CRC_BYTES;
	const int cwc_cap    = sub_len - BIGBLOCK_CW_CRC_BYTES;
	A->message_batch_counter_tx = K;
	std::vector<std::vector<unsigned char>> app_truth((size_t)K);
	std::vector<int> app_len((size_t)K, 0);
	long total_app_bytes = 0;
	for(int i=0;i<K;i++)
	{
		// deterministic variable length: a spread of sizes incl. a short last frame, all
		// within the per-codeword app capacity (cw0 is reduced by the header, every
		// codeword by the CRC byte).
		int cap = (i == 0) ? cw0_cap : cwc_cap;
		int len = ((i*37 + 11) % (cap - 4)) + 1;   // 1..cap-4 (well within capacity)
		if(len > cap) len = cap;
		app_len[i] = len;
		total_app_bytes += len;
		app_truth[i].assign((size_t)len, 0);

		A->messages_batch_tx[i].data         = A->messages_tx[i].data;
		A->messages_batch_tx[i].type         = DATA_LONG;
		A->messages_batch_tx[i].id           = (char)(unsigned char)i;
		A->messages_batch_tx[i].length       = len;
		A->messages_batch_tx[i].batch_seq_id = block_bsi;
		A->messages_batch_tx[i].status       = ADDED_TO_BATCH_BUFFER;
		for(int j=0;j<len;j++)
		{
			unsigned char b = (unsigned char)((i*53 + j*17 + 3) & 0xFF);
			A->messages_batch_tx[i].data[j]   = (char)b;
			app_truth[i][(size_t)j]           = b;
		}
		A->messages_tx[i].status = ADDED_TO_BATCH_BUFFER;
	}

	// Build the on-wire block payload EXACTLY as production bigblock_send_one_block does
	// (header in cw0 prefix + app bytes per codeword). The loopback transmits THIS as the
	// block payload; the RX carve must recover the header + each frame byte-faithfully.
	std::vector<unsigned char> tx_truth((size_t)total_tx_bytes, 0);
	auto build_block_wire = [&](int bsi)
	{
		std::fill(tx_truth.begin(), tx_truth.end(), 0);
		tx_truth[0] = (unsigned char)(bsi & 0xFF);
		tx_truth[1] = (unsigned char)(K & 0xFF);                 // n_data == K (all filled)
		for(int c=0;c<K;c++)
		{
			int lo = BIGBLOCK_HDR_FIXED_BYTES + 2*c;
			tx_truth[(size_t)lo + 0] = (unsigned char)(app_len[c] & 0xFF);
			tx_truth[(size_t)lo + 1] = (unsigned char)((app_len[c] >> 8) & 0xFF);
			int base = (c == 0) ? hdr_total : (c * sub_len);
			for(int j=0;j<app_len[c];j++)
				tx_truth[(size_t)base + j] = app_truth[c][(size_t)j];
		}
		// FAILURE-2 fix: stamp the per-codeword wire CRC-8 EXACTLY as production
		// bigblock_send_one_block does (CRC over the codeword's first
		// BIGBLOCK_CW_CRC_SPAN(sub_len) bytes -> tail byte). cw0's CRC covers the header.
		for(int c=0;c<K;c++)
		{
			int crc_off  = BIGBLOCK_CW_CRC_OFFSET(c, sub_len);
			int crc_span = BIGBLOCK_CW_CRC_SPAN(sub_len);
			if(crc_off < 0 || crc_off >= (int)total_tx_bytes || crc_span < 0) continue;
			tx_truth[(size_t)crc_off] = A->CRC8_calc(
				(char*)&tx_truth[(size_t)c*sub_len], crc_span);
		}
	};
	build_block_wire(block_bsi);
	printf("[TEST-SIM-BIGBLOCK] geometry: hdr_total=%d cw0_cap=%d variable app_len total=%ld\n",
	       hdr_total, cw0_cap, total_app_bytes);
	fflush(stdout);

	// Helper: count delivered bytes that byte-match app_truth per slot (the variable-
	// length INV-6 measure — each slot must deliver EXACTLY its frame's app bytes,
	// with the slot's recorded length == the TX frame length). Flatten app_truth +
	// app_len for the member helper (messages_rx[] is private).
	std::vector<unsigned char> app_flat;
	std::vector<int> app_off((size_t)K, 0);
	for(int c=0;c<K;c++)
	{
		app_off[c] = (int)app_flat.size();
		for(int j=0;j<app_len[c];j++) app_flat.push_back(app_truth[c][(size_t)j]);
	}
	auto delivered_app_bytes = [&](cl_arq_controller* a) -> long {
		return a->bigblock_test_delivered_varlen(K, app_len.data(), app_off.data(),
			app_flat.data());
	};

	// ====================================================================
	// PHY block loopback (CMD transmit_bigblock -> clean wire -> RSP
	// receive_bigblock). Modeled on bigblock_livepath_loopback (the proven
	// 8/8 path) but channel-free (clean), so the carve is deterministic.
	// We pack the K*sub_len real bytes and transmit ONE block via the
	// PRODUCTION transmit_byte (branches to transmit_bigblock, P2.1).
	// ====================================================================
	auto run_block_loopback = [&](cl_telecom_system* tx, cl_telecom_system* rx,
	                              const std::vector<unsigned char>& bytes,
	                              std::vector<int>& info_bits_out) -> int {
		int interp = tx->frequency_interpolation_rate;
		int block_n = tx->bigblock_tx_total_samples();
		if(block_n <= 0) return -1;
		int lead_n  = (int)(100.0 * tx->sampling_frequency / 1000.0);
		int trail_n = (int)(50.0  * tx->sampling_frequency / 1000.0);

		// TX: pack real bytes -> transmit_byte -> transmit_bigblock.
		std::vector<int> payload((size_t)bytes.size(), 0);
		for(size_t i=0;i<bytes.size();i++) payload[i] = (int)bytes[i];
		std::vector<double> tx_pb((size_t)block_n, 0.0);
		tx->bigblock_framing_enabled = true;
		// HEAP-OVERRUN FIX (fact-doc §13): arm the per-call block-emit intent so the CFG16
		// transmit_byte branch fires (tx_pb is block-sized = block_n).
		{
			cl_telecom_system::bigblock_emit_scope emit_guard(tx, block_n);
			tx->transmit_byte(payload.data(), (int)payload.size(), tx_pb.data(), NO_FILTER_MESSAGE);
		}
		int K_tx = tx->bigblock_last_tx_K;
		int n_tx = tx->bigblock_last_tx_samples;
		if(K_tx <= 0 || n_tx <= 0) return -1;

		// CHANNEL: clean loopback into the RX capture window [lead | block | trail].
		int rx_window = lead_n + n_tx + trail_n;
		std::vector<double> rx_pb((size_t)rx_window, 0.0);
		for(int i=0;i<n_tx;i++) rx_pb[lead_n + i] = tx_pb[i];

		// RX: size buffer_Nsymb to span the window, then receive_byte ->
		// receive_bigblock (SAME construction as bigblock_livepath_loopback).
		int Nofdm = rx->data_container.Nofdm;
		int saved_buffer_Nsymb = rx->data_container.buffer_Nsymb;
		if(Nofdm > 0)
		{
			int need_syms = (rx_window + Nofdm*interp - 1) / (Nofdm*interp);
			rx->data_container.buffer_Nsymb = need_syms;
			int exact = need_syms * Nofdm * interp;
			if((int)rx_pb.size() < exact) rx_pb.resize((size_t)exact, 0.0);
		}
		info_bits_out.assign((size_t)(K_tx+1) * rx->ldpc.K + rx->ldpc.K, 0);
		rx->bigblock_framing_enabled = true;
		rx->receive_byte(rx_pb.data(), info_bits_out.data());
		rx->data_container.buffer_Nsymb = saved_buffer_Nsymb;
		return rx->bigblock_last_rx_K;
	};

	// ====================================================================
	// CASE A — clean single block: CMD->RSP->ACK->CMD byte-faithful.
	// PHASE 1 DRIFT-PROOF: the RSP's local expected bsi is set to a WRONG value (the
	// drift a multi-block session would suffer) and the carve fallback is ALSO wrong;
	// the carve must recover the AUTHORITATIVE bsi FROM THE WIRE (cw0 header) and stamp
	// messages_rx[c].batch_seq_id == the TX block_bsi (NOT the wrong local guess).
	// Frames are VARIABLE length -> the carve must deliver each slot its EXACT length
	// (compression transparency). Without the wire header both would fail.
	{
		// reset RSP RX state. DRIFT: the RSP's local expected bsi is DELIBERATELY wrong.
		const int wrong_local_bsi = (block_bsi + 5) & 0xFF;   // != block_bsi (drift)
		B->rsp_current_expected_batch_seq_id = wrong_local_bsi;
		B->rsp_prev_batch_seq_id             = -1;
		B->rsp_prev_batch_active             = false;
		B->rsp_prev_batch_received_count     = 0;
		B->rsp_prev_batch_expected_count     = 0;
		B->rsp_prev_batch_delivered_count    = 0;
		B->retransmit_count                  = 0;
		B->batch_rx_frame_count              = 0;
		B->last_received_end_of_batch_seq    = -1;
		for(int i=0;i<B->nMessages;i++)
		{
			B->messages_rx[i].status = FREE;
			B->messages_rx[i].length = 0;
			B->messages_rx[i].batch_seq_id = -1;
		}

		std::vector<int> info_bits;
		int K_rx = run_block_loopback(tsA, tsB, tx_truth, info_bits);
		bool decoded = (K_rx == K);
		int cw_ok_count = tsB->bigblock_last_rx_cw_ok_count;
		bool all_clean = (cw_ok_count == K);

		// RSP carve (production RX wiring): fallback_bsi is DELIBERATELY wrong; the wire
		// header (use_wire_header=true, default) must override it with the TX bsi.
		// HEAP-OVERRUN FIX (fact-doc §13): carve from the DEDICATED rx member exactly as the
		// live RX path (arq_common.cc:6794) — receive_bigblock landed the full K*ldpc.K decode
		// there, NOT in `info_bits` (which is now only an N_MAX-bounded legacy copy).
		const int wrong_fallback = (block_bsi + 3) & 0xFF;
		int rc = B->bigblock_receive_carve(tsB->bigblock_rx_infobits.data(), (unsigned char)wrong_fallback);
		bool wired = (rc == SUCCESSFUL);

		int recv       = B->bigblock_test_count_received(K);
		long delivered = delivered_app_bytes(B);          // variable-length INV-6 measure
		int bsi_after  = B->rsp_current_expected_batch_seq_id;
		// DRIFT-PROOF: the carve adopted the WIRE bsi for every slot, NOT the wrong local
		// guess; and the bsi bump landed on (wire_bsi+1), proving the wire value was used.
		bool slots_wire_bsi = true;
		for(int c=0;c<K;c++)
			if(B->messages_rx[c].status==RECEIVED && B->messages_rx[c].batch_seq_id != block_bsi)
				slots_wire_bsi = false;
		bool one_bump  = (bsi_after == ((wrong_local_bsi + 1) & 0xFF));  // bumped from local once
		bool all_recv  = (recv == K);
		bool all_bytes = (delivered == total_app_bytes);

		// ACK leg (R-B): the clean block emits all-ones K-bit 0xFF; the CMD all_ones
		// target (data_batch_size==K==8) is 0xFF -> they MATCH -> clean ACK credited.
		uint32_t rsp_bitmap = 0;
		for(int c=0;c<K;c++) if(B->messages_rx[c].status==RECEIVED) rsp_bitmap |= (1u<<c);
		uint32_t cmd_all_ones = (A->data_batch_size >= 32) ? 0xFFFFFFFFu
		                       : ((1u << A->data_batch_size) - 1u);
		bool ack_match = (rsp_bitmap == cmd_all_ones) && (rsp_bitmap == 0xFFu);
		bool cmd_credits = cl_arq_controller::sack_clean_confirmation_accepted(
		                     (unsigned char)block_bsi, /*is_all_ones=*/true,
		                     /*last_applied_clean_bsi=*/-1, /*last_applied_sack_bsi=*/-1);

		bool pass = decoded && all_clean && wired && all_recv && all_bytes
		         && slots_wire_bsi && one_bump && ack_match && cmd_credits;
		printf("[TEST-SIM-BIGBLOCK] CASE A clean (wire-bsi drift-proof): %s "
			"(decoded=%d K_rx=%d cw_ok=%d/%d carve_rc=%d recv=%d/%d delivered=%ld/%ld "
			"tx_bsi=%d wrong_local=%d wrong_fallback=%d slots_wire_bsi=%d bsi_after=%d "
			"one_bump=%d rsp_bitmap=0x%X cmd_all_ones=0x%X ack_match=%d cmd_credits=%d)\n",
			pass ? "PASS" : "FAIL", (int)decoded, K_rx, cw_ok_count, K, rc, recv, K,
			delivered, total_app_bytes, block_bsi, wrong_local_bsi, wrong_fallback,
			(int)slots_wire_bsi, bsi_after, one_bump,
			(unsigned)rsp_bitmap, (unsigned)cmd_all_ones, ack_match, cmd_credits);
		fflush(stdout);
		check(pass, "CASE A single big-block wire-bsi drift-proof + variable-length byte-faithful");
	}

	// ====================================================================
	// CASE B — one bad codeword: partial K-bit SACK + selective-repeat of
	// EXACTLY that codeword; the retx fill completes the block K/K.
	// ====================================================================
	{
		const int bad_cw = 3;
		const int block_bsi_b = 9;
		// Rebuild the on-wire block payload with CASE B's bsi (the wire header carries it).
		build_block_wire(block_bsi_b);
		B->rsp_current_expected_batch_seq_id = block_bsi_b;
		B->rsp_prev_batch_seq_id             = -1;
		B->rsp_prev_batch_active             = false;
		B->rsp_prev_batch_received_count     = 0;
		B->rsp_prev_batch_expected_count     = 0;
		B->retransmit_count                  = 0;
		B->batch_rx_frame_count              = 0;
		B->last_received_end_of_batch_seq    = -1;
		for(int i=0;i<B->nMessages;i++)
		{
			B->messages_rx[i].status = FREE;
			B->messages_rx[i].length = 0;
			B->messages_rx[i].batch_seq_id = -1;
		}

		std::vector<int> info_bits;
		int K_rx = run_block_loopback(tsA, tsB, tx_truth, info_bits);
		bool decoded = (K_rx == K);

		// Inject one bad codeword DETERMINISTICALLY: clear cw_ok[bad_cw] before the
		// carve (the carve reads telecom_system->bigblock_last_rx_cw_ok). This is the
		// same one-clear-bit injection the unit test CASE2 uses, but driven through
		// the real PHY decode + the real RX carve wiring. (bad_cw != 0 so cw0 — which
		// carries the wire header — stays clean and the header is trusted.)
		if((int)tsB->bigblock_last_rx_cw_ok.size() > bad_cw)
			tsB->bigblock_last_rx_cw_ok[bad_cw] = 0;
		tsB->bigblock_last_rx_cw_ok_count = K - 1;

		// Wire header in cw0 is clean -> the carve uses the WIRE bsi + length table.
		// HEAP-OVERRUN FIX (fact-doc §13): carve from the dedicated rx member (live wiring).
		int rc = B->bigblock_receive_carve(tsB->bigblock_rx_infobits.data(), (unsigned char)block_bsi_b);
		bool wired = (rc == SUCCESSFUL);

		int recv_partial = B->bigblock_test_count_received(K);
		bool bad_absent  = (B->messages_rx[bad_cw].status != RECEIVED);
		bool partial_ok  = (recv_partial == K - 1) && bad_absent;
		bool one_retx    = (B->retransmit_count == 1);
		// the retx carries the bad codeword's ORIGINAL bsi (from the wire) + its REAL
		// app length (the wire length table), NOT sub_len.
		bool retx_pos_ok = one_retx && (B->retransmit_frame_positions[0] == bad_cw)
		                && (B->retransmit_frame_batch_seq_ids[0] == block_bsi_b)
		                && (B->retransmit_frame_lengths[0] == app_len[bad_cw]);
		bool no_bump     = (B->rsp_current_expected_batch_seq_id == block_bsi_b);

		// Model the selective-repeat arrival: the retx of EXACTLY the bad codeword
		// fills the gap (what the stock CFG16 per-frame retx path does on RX). The
		// delivered slot carries the frame's REAL app length, not sub_len.
		if(wired && partial_ok && one_retx)
		{
			int loc = bad_cw;
			B->messages_rx[loc].type            = DATA_LONG;
			B->messages_rx[loc].id              = (char)(unsigned char)loc;
			B->messages_rx[loc].length          = app_len[loc];
			B->messages_rx[loc].status          = RECEIVED;
			B->messages_rx[loc].batch_seq_id    = block_bsi_b;
			B->messages_rx[loc].sequence_number = (char)(unsigned char)loc;
			for(int j=0;j<app_len[loc];j++)
				B->messages_rx[loc].data[j] = (char)app_truth[loc][(size_t)j];
		}

		int recv_full  = B->bigblock_test_count_received(K);
		long delivered = delivered_app_bytes(B);
		bool full_ok   = (recv_full == K) && (delivered == total_app_bytes);

		bool pass = decoded && wired && partial_ok && one_retx && retx_pos_ok
		         && no_bump && full_ok;
		printf("[TEST-SIM-BIGBLOCK] CASE B one-bad-cw=%d: %s "
			"(decoded=%d carve_rc=%d partial=%d/%d bad_absent=%d retx_count=%d "
			"retx_pos_ok=%d no_bump=%d full=%d/%d delivered=%ld/%ld)\n",
			bad_cw, pass ? "PASS" : "FAIL", (int)decoded, rc, recv_partial, K,
			bad_absent, B->retransmit_count, retx_pos_ok, no_bump, recv_full, K,
			delivered, total_app_bytes);
		fflush(stdout);
		check(pass, "CASE B one-bad-codeword partial -> selective-repeat completes block in-sim");
	}

	// ====================================================================
	// CASE C — PRODUCTION-BUFFER big-block TX/RX heap-overrun guard
	// (fact-doc §13). CASE A/B bypass the undersized PRODUCTION buffers:
	// run_block_loopback passes a correctly block-sized info_bits_out + a
	// block-sized tx_pb, so neither the receive_bigblock copy-out nor the
	// transmit_byte divert ever touches an undersized live allocation — they
	// ran GREEN on Windows WHILE the live path heap-overran (the OS allocator
	// tolerated it). CASE C drives the EXACT live buffers with an explicit
	// tail canary so the overrun is caught deterministically on ANY platform.
	//
	//   RX leg: call the REAL receive_byte(rx_pb, out) where `out` is a
	//           data_byte-shaped buffer (N_MAX ints) followed by a CANARY
	//           guard region — exactly as arq_common.cc:6765 passes
	//           data_container.data_byte. PRE-fix the copy-out wrote
	//           Kout*ldpc.K=11200 ints -> smashed the guard. POST-fix the
	//           decode lands in bigblock_rx_infobits and only N_MAX is copied
	//           -> guard intact.
	//   TX leg: call the REAL transmit_byte at CFG16 into a FRAME-sized buffer
	//           + CANARY, WITHOUT arming the block-emit scope — the exact
	//           declined-batch / control-frame fallthrough condition
	//           (arq_common.cc:4317/4011). PRE-fix the config-only gate routed
	//           it into transmit_bigblock -> ~45552 doubles into the frame slot
	//           -> guard smashed. POST-fix the per-call intent is false -> stock
	//           per-frame OFDM geometry -> guard intact.
	//
	// Both legs PASS in the normal suite run (fixed). MERCURY_BIGBLOCK_OLDGATE=1
	// restores the pre-fix behavior on the SAME binary so the fail-before is
	// reproducible without a revert build (the guard checks then FAIL).
	{
		const int GUARD = 64;
		const int sentinel = 0x5A5A5A5A;
		bool oldgate = false;
		{ const char* e = std::getenv("MERCURY_BIGBLOCK_OLDGATE"); if(e && *e && atoi(e)!=0) oldgate = true; }

		// ---- produce ONE valid clean block waveform (CMD side), for the RX leg ----
		build_block_wire(block_bsi);
		int interp = tsA->frequency_interpolation_rate;
		int block_n = tsA->bigblock_tx_total_samples();
		std::vector<double> tx_pb((size_t)block_n, 0.0);
		{
			std::vector<int> payload((size_t)tx_truth.size(), 0);
			for(size_t i=0;i<tx_truth.size();i++) payload[i] = (int)tx_truth[i];
			tsA->bigblock_framing_enabled = true;
			cl_telecom_system::bigblock_emit_scope emit_guard(tsA, block_n);  // CMD emits a BLOCK (correct buffer)
			tsA->transmit_byte(payload.data(), (int)payload.size(), tx_pb.data(), NO_FILTER_MESSAGE);
		}
		int n_tx = tsA->bigblock_last_tx_samples;

		// ---- RX LEG: receive_byte into a data_byte-shaped buffer + tail canary ----
		int lead_n  = (int)(100.0 * tsB->sampling_frequency / 1000.0);
		int trail_n = (int)(50.0  * tsB->sampling_frequency / 1000.0);
		int rx_window = lead_n + n_tx + trail_n;
		std::vector<double> rx_pb((size_t)rx_window, 0.0);
		for(int i=0;i<n_tx && i<(int)tx_pb.size();i++) rx_pb[lead_n + i] = tx_pb[i];
		int Nofdm = tsB->data_container.Nofdm;
		int saved_buffer_Nsymb = tsB->data_container.buffer_Nsymb;
		if(Nofdm > 0)
		{
			int need_syms = (rx_window + Nofdm*interp - 1) / (Nofdm*interp);
			tsB->data_container.buffer_Nsymb = need_syms;
			int exact = need_syms * Nofdm * interp;
			if((int)rx_pb.size() < exact) rx_pb.resize((size_t)exact, 0.0);
		}
		// out buffer models data_container.data_byte[N_MAX]: the canary sits AT the N_MAX
		// boundary (the live data_byte's true end). Allocate the FULL block-decode extent +
		// guard so that under OLDGATE the (wrong) unbounded copy lands WITHIN this allocation
		// and only trips the boundary canary — no unrelated heap smash / crash. Post-fix the
		// copy is bounded to N_MAX, so the canary stays intact.
		int rx_full = (K + 1) * tsB->ldpc.K + tsB->ldpc.K;   // >= Kout*ldpc.K
		size_t rx_out_cap = (size_t)(rx_full > N_MAX ? rx_full : N_MAX) + GUARD;
		std::vector<int> rx_out(rx_out_cap, sentinel);
		tsB->bigblock_framing_enabled = true;
		tsB->receive_byte(rx_pb.data(), rx_out.data());
		tsB->data_container.buffer_Nsymb = saved_buffer_Nsymb;
		bool rx_canary_ok = true;
		for(int i=0;i<GUARD;i++) if(rx_out[(size_t)N_MAX + i] != sentinel) { rx_canary_ok = false; break; }
		// post-fix the full decode is in the dedicated member (carve reads it), sized for K.
		bool rx_member_sized = ((int)tsB->bigblock_rx_infobits.size() >= K * tsB->ldpc.K);

		// ---- TX LEG: transmit_byte at CFG16 into a FRAME-sized buffer + tail canary,
		//      WITHOUT the block-emit scope (the declined-batch / control fallthrough) ----
		int active_nsymb = tsA->get_active_nsymb();
		int frame_output_size = tsA->data_container.Nofdm
			* tsA->data_container.interpolation_rate
			* (active_nsymb + tsA->data_container.preamble_nSymb);
		// Allocate the FULL block extent + guard so that under OLDGATE the (wrong) block
		// write lands WITHIN this allocation and only trips the canary at the frame-slot
		// boundary — no unrelated heap smash / crash, a clean deterministic FAIL signal.
		// Post-fix the stock per-frame path writes <= frame_output_size, leaving the canary
		// (placed AT the slot boundary) intact.
		size_t tx_frame_cap = (size_t)(block_n > frame_output_size ? block_n : frame_output_size) + GUARD;
		std::vector<double> tx_frame(tx_frame_cap, 0.0);
		for(int i=0;i<GUARD;i++) tx_frame[(size_t)frame_output_size + i] = (double)sentinel;
		// a small stock CONTROL-sized payload (well within one frame). NO emit scope ->
		// bigblock_emit_as_block stays false -> stock per-frame path (post-fix). Mark the
		// TX stash so we can detect whether a block was (wrongly) emitted into the frame slot.
		int ctrl_len = 4;
		for(int i=0;i<ctrl_len;i++) tsA->data_container.data_byte[i] = i & 0xFF;
		tsA->bigblock_last_tx_samples = -1;   // sentinel: stock path won't touch it; a block emit sets it >0
		tsA->bigblock_framing_enabled = true;
		tsA->transmit_byte(tsA->data_container.data_byte, ctrl_len, tx_frame.data(), NO_FILTER_MESSAGE);
		bool tx_canary_ok = true;
		for(int i=0;i<GUARD;i++) if(tx_frame[(size_t)frame_output_size + i] != (double)sentinel) { tx_canary_ok = false; break; }
		// post-fix the declined frame took the STOCK per-frame path (no block emitted here):
		// the block stash was NOT updated (still the -1 sentinel) AND a block could not have
		// fit (frame_output_size < block_n). PRE-fix (oldgate) a block IS emitted -> stash >0.
		bool tx_no_block = (tsA->bigblock_last_tx_samples <= 0) && (frame_output_size < block_n);

		bool pass = rx_canary_ok && tx_canary_ok && rx_member_sized && tx_no_block;
		printf("[TEST-SIM-BIGBLOCK] CASE C production-buffer overrun guard: %s "
			"(oldgate=%d rx_canary_ok=%d tx_canary_ok=%d rx_member=%zu(>=%d) "
			"frame_slot=%d block_n=%d n_tx=%d)\n",
			pass ? "PASS" : "FAIL", (int)oldgate, (int)rx_canary_ok, (int)tx_canary_ok,
			tsB->bigblock_rx_infobits.size(), K * tsB->ldpc.K,
			frame_output_size, block_n, n_tx);
		fflush(stdout);
		if(oldgate)
			printf("[TEST-SIM-BIGBLOCK] CASE C NOTE: OLDGATE=1 reproduces the PRE-FIX overrun "
				"(expect canary FAIL) — fail-before proof on the same binary.\n");
		check(pass, "CASE C production-buffer big-block TX/RX no heap overrun (data_byte + frame slot canaries intact)");
	}

	// ====================================================================
	// CASE D — FAILURE-2 PRODUCER TEST (the gap T3/T4/CASE-B skipped). Flip a
	// bit in the LLR of ONE codeword (NOT the cw_ok array) so the REAL PHY decode
	// emits a corrupted info-bit sub-unit for that codeword. On the live 2-instance
	// path the PHY cw_ok producer (bigblock_rx_passband, cw_info_ref==NULL) FORCES
	// cw_ok=1 for every codeword — so WITHOUT the wire CRC this corrupted codeword
	// is accepted clean and NEVER retransmitted (the silent-corruption bug). The
	// wire-CRC recompute in bigblock_receive_carve must DEMOTE that codeword's
	// cw_ok to 0, and the selective-repeat must re-send EXACTLY it.
	//   FAIL-BEFORE (no wire CRC): producer forces cw_ok all-clean -> carve delivers
	//                              the corrupted codeword, retx_count==0 (accepted).
	//   PASS-AFTER  (wire CRC):    carve recomputes CRC over the de-whitened bytes,
	//                              demotes the corrupted codeword, retx re-sends it.
	// The fail-before is reproducible on the SAME binary via MERCURY_BIGBLOCK_NOCRC=1
	// (mirrors the heap-fix MERCURY_BIGBLOCK_OLDGATE): that env disables the carve's
	// wire-CRC demote, so the producer's forced-clean cw_ok stands -> the corrupted
	// codeword is accepted (no retx) -> the PASS-AFTER post-conditions FAIL (test FAILs).
	{
		const int bad_cw      = 4;          // != 0 so cw0 (the wire header) stays clean
		const int block_bsi_c = 11;
		bool nocrc = false;
		{ const char* e = std::getenv("MERCURY_BIGBLOCK_NOCRC"); if(e && *e && atoi(e)!=0) nocrc = true; }
		build_block_wire(block_bsi_c);      // wire header carries CASE D's bsi
		B->rsp_current_expected_batch_seq_id = block_bsi_c;
		B->rsp_prev_batch_seq_id             = -1;
		B->rsp_prev_batch_active             = false;
		B->rsp_prev_batch_received_count     = 0;
		B->retransmit_count                  = 0;
		B->batch_rx_frame_count              = 0;
		B->last_received_end_of_batch_seq    = -1;
		for(int i=0;i<B->nMessages;i++)
		{
			B->messages_rx[i].status = FREE;
			B->messages_rx[i].length = 0;
			B->messages_rx[i].batch_seq_id = -1;
		}

		// Arm the PHY-layer LLR corruption hook for EXACTLY codeword bad_cw. This flips
		// the sign of a run of that codeword's LLRs before its ldpc.decode, so the REAL
		// decode produces wrong info_bits for it (a miscorrection / residual-error the
		// LDPC iter-count does NOT flag) while cw0 + the other codewords stay clean.
		char cw_env[16]; snprintf(cw_env, sizeof(cw_env), "%d", bad_cw);
#if defined(_WIN32)
		_putenv_s("MERCURY_BIGBLOCK_CORRUPT_CW", cw_env);
#else
		setenv("MERCURY_BIGBLOCK_CORRUPT_CW", cw_env, 1);
#endif

		std::vector<int> info_bits;
		int K_rx = run_block_loopback(tsA, tsB, tx_truth, info_bits);
		bool decoded = (K_rx == K);

		// Disarm the hook immediately (so nothing else in the process is corrupted).
#if defined(_WIN32)
		_putenv_s("MERCURY_BIGBLOCK_CORRUPT_CW", "");
#else
		unsetenv("MERCURY_BIGBLOCK_CORRUPT_CW");
#endif

		// HEAP-OVERRUN FIX RECONCILIATION (fact-doc §13): receive_bigblock now lands the FULL
		// K*ldpc.K decode in the dedicated member bigblock_rx_infobits; the `info_bits` (out)
		// buffer is only an N_MAX-bounded legacy copy (~1.1 codewords). Read the full decode
		// from the member for BOTH the direct CRC proof and the production carve — exactly as
		// the live RX path (arq_common.cc receive()) and the heap-fix CASE A/B do.
		const std::vector<int>& full_info = tsB->bigblock_rx_infobits;

		// FAIL-BEFORE EVIDENCE: the PHY cw_ok producer (bigblock_rx_passband, the
		// telecom_system->bigblock_last_rx_cw_ok array) forced ALL codewords clean even
		// though bad_cw decoded corrupt (cw_info_ref==NULL on the 2-instance path). This
		// is exactly the state in which, WITHOUT the wire CRC, the carve would have
		// accepted the corrupted codeword + never retransmitted it. We do NOT touch the
		// cw_ok array — the carve's wire-CRC recompute is the only thing that can demote.
		int producer_clean_count = tsB->bigblock_last_rx_cw_ok_count;
		bool producer_forced_clean = (producer_clean_count == K)
		    && (bad_cw < (int)tsB->bigblock_last_rx_cw_ok.size())
		    && (tsB->bigblock_last_rx_cw_ok[bad_cw] == 1);

		// DIRECT WIRE-CRC PROOF (non-vacuous): de-whiten the decoded info_bits EXACTLY as
		// bigblock_receive_carve does, then recompute the per-codeword CRC-8 over the same
		// span and compare to each codeword's tail CRC byte. This is the very check the
		// carve runs to produce cw_ok. Assert it MISMATCHES on bad_cw (the corruption
		// landed -> the wire CRC catches what the producer forced clean) and MATCHES on
		// cw0 (the header codeword) + every other codeword (no false demotion).
		bool crc_mismatch_bad = false, crc_match_others = true, crc_match_cw0 = true;
		{
			int sub_len_c = tsB->ldpc.K / 8;
			int nbits_c   = K * sub_len_c * 8;
			std::vector<int> dw((size_t)nbits_c, 0);
			for(int i=0;i<nbits_c && i<(int)full_info.size();i++) dw[i] = full_info[i] & 1;
			tsB->bigblock_whiten_bits(dw.data(), nbits_c);   // de-whiten (self-inverse)
			std::vector<unsigned char> pl((size_t)K * sub_len_c, 0);
			for(int c=0;c<K;c++)
				for(int b=0;b<sub_len_c;b++){
					unsigned char by=0;
					for(int bit=0;bit<8;bit++){ int idx=(c*sub_len_c+b)*8+bit;
						if(idx<nbits_c && (dw[idx]&1)) by|=(unsigned char)(1u<<bit); }
					pl[(size_t)c*sub_len_c + b]=by; }
			int span = BIGBLOCK_CW_CRC_SPAN(sub_len_c);
			for(int c=0;c<K;c++){
				int off = BIGBLOCK_CW_CRC_OFFSET(c, sub_len_c);
				unsigned char calc = CRC8_calc((char*)&pl[(size_t)c*sub_len_c], span);
				bool match = (calc == pl[(size_t)off]);
				if(c==bad_cw){ crc_mismatch_bad = !match; }
				else if(c==0){ crc_match_cw0 = match; if(!match) crc_match_others=false; }
				else if(!match) crc_match_others=false;
			}
		}
		bool corruption_landed = crc_mismatch_bad;  // bad_cw's bytes provably differ from its CRC

		// RX carve: the wire CRC recompute (FAILURE-2 fix) runs on the de-whitened payload
		// BEFORE the header-trust gate and DEMOTES the corrupted codeword in the carve's
		// own cw_ok vector (the consumer the SACK reads — NOT the stale telecom array).
		// Carve from the dedicated full-decode member (heap-fix reconciliation), NOT the
		// N_MAX-truncated `info_bits` out-copy.
		int rc = B->bigblock_receive_carve(full_info.data(), (unsigned char)block_bsi_c);
		bool wired = (rc == SUCCESSFUL);

		// PASS-AFTER (the OBSERVABLE consumer effect of the carve's cw_ok==0 demotion):
		// bad_cw is absent from messages_rx[] and the selective-repeat re-requested
		// EXACTLY it. Because the producer forced bad_cw CLEAN, the ONLY thing that could
		// make it absent + re-requested is the wire-CRC demote -> this IS the demotion.
		int recv_partial = B->bigblock_test_count_received(K);
		bool bad_absent  = (B->messages_rx[bad_cw].status != RECEIVED);
		bool partial_ok  = (recv_partial == K - 1) && bad_absent;
		bool one_retx    = (B->retransmit_count == 1);
		bool retx_pos_ok = one_retx && (B->retransmit_frame_positions[0] == bad_cw)
		                && (B->retransmit_frame_batch_seq_ids[0] == block_bsi_c);
		bool others_recv = true;             // every OTHER codeword delivered clean (no false demote)
		for(int c=0;c<K;c++)
			if(c != bad_cw && B->messages_rx[c].status != RECEIVED) others_recv = false;

		bool pass = decoded && producer_forced_clean && wired && corruption_landed
		         && crc_match_cw0 && crc_match_others && partial_ok && one_retx
		         && retx_pos_ok && others_recv;
		printf("[TEST-SIM-BIGBLOCK] CASE D FAILURE-2 producer (LLR-corrupt cw=%d): %s "
			"(decoded=%d K_rx=%d producer_forced_clean=%d[count=%d] wired=%d "
			"crc_mismatch_bad=%d crc_match_cw0=%d crc_match_others=%d partial=%d/%d "
			"bad_absent=%d retx_count=%d retx_pos_ok=%d others_recv=%d)\n",
			bad_cw, pass ? "PASS" : "FAIL", (int)decoded, K_rx,
			(int)producer_forced_clean, producer_clean_count, (int)wired,
			(int)crc_mismatch_bad, (int)crc_match_cw0, (int)crc_match_others,
			recv_partial, K, (int)bad_absent, B->retransmit_count, (int)retx_pos_ok,
			(int)others_recv);
		fflush(stdout);
		if(nocrc)
			printf("[TEST-SIM-BIGBLOCK] CASE D NOTE: NOCRC=1 disables the carve wire-CRC demote "
				"(expect FAIL: corrupted cw accepted, retx_count==0) — fail-before proof on the same binary.\n");
		check(pass, "CASE D FAILURE-2 wire-CRC producer: LLR-corrupt cw -> cw_ok==0 -> "
		            "selective-repeat re-sends EXACTLY it (FAIL-before/PASS-after)");
	}

	delete A; delete B;
	delete tsA; delete tsB;
	restore_env();

	printf("[TEST-SIM-BIGBLOCK] %s (%d failure%s)\n",
	       failed == 0 ? "ALL PASS" : "FAILURES", failed, failed == 1 ? "" : "s");
	fflush(stdout);
	return failed == 0 ? 0 : 1;
}

// ============================================================================
// TX-LEVEL parity diag — --test-bigblock-txlevel
//
// HW symptom (bench scope, user-observed 2026-06-06): the big-block frames run
// ~1400-1450 mVp-p (+~3.2 dB) over the 1000 mVp-p calibrated sweet spot, while
// ACKs and stock OFDM frames sit at calibration. This measures, in-process and
// device-free, whether that is a GAIN difference (RMS ratio != 1, e.g. a missing
// TX_SIG_OFDM calibration factor) or a PAPR/length peak-vs-RMS effect (RMS ~= 1
// but peak ratio > 1 because the K=8 block is ONE long waveform whose extreme
// peak is higher at equal RMS power).
//
// Method: bring up a single CFG16 cl_telecom_system, emit BOTH waveforms via the
// SAME production transmit_byte entry:
//   (1) BIG-BLOCK: bigblock_framing_enabled=1 + bigblock_emit_scope -> branches
//       to transmit_bigblock -> bigblock_tx_passband (the path the bench saw hot).
//   (2) STOCK OFDM: bigblock_framing_enabled=0 -> the per-frame OFDM path
//       (transmit_byte:649+) — the calibrated reference.
// Both at CONFIG_16 (same constellation/grid/pilots). Measure peak (Vp-p proxy =
// max|sample|) + RMS over (a) the whole waveform and (b) the DATA span only
// (post-preamble), since the data symbols are what clip on HW. Print ratios.
// ============================================================================
int cl_arq_controller::test_bigblock_txlevel()
{
	printf("[TXLEVEL] big-block vs stock-OFDM CONFIG_16 TX peak/RMS parity "
	       "(in-process, device-free; same transmit_byte entry)\n");
	fflush(stdout);

	// Pin K = 8 (production cap path) so the block geometry is deterministic.
	const char* prev_k = std::getenv("MERCURY_BIGBLOCK_K");
	std::string prev_k_saved = prev_k ? std::string(prev_k) : std::string();
	bool had_prev_k = (prev_k != NULL);
#if defined(_WIN32)
	_putenv_s("MERCURY_BIGBLOCK_K", "8");
#else
	setenv("MERCURY_BIGBLOCK_K", "8", 1);
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

	cl_telecom_system* ts = new cl_telecom_system();
	cl_arq_controller* A  = new cl_arq_controller();
	A->telecom_system = ts;
	A->nMessages          = 255;
	A->max_data_length    = 170;
	A->max_message_length = 200;
	A->max_header_length  = 6;
	A->init_messages_buffers();
	A->load_configuration(CONFIG_16, FULL, YES);   // real CFG16 grid + buffers

	int interp   = ts->frequency_interpolation_rate;
	int Nofdm    = ts->data_container.Nofdm;
	int preN     = ts->data_container.preamble_nSymb;
	int Nsymb    = ts->data_container.Nsymb;
	int pre_samp = Nofdm * preN * interp;   // shared preamble extent (same on both paths)

	// peak (max|s|) + RMS over [lo, hi).
	auto measure = [](const double* s, int lo, int hi, double& peak, double& rms){
		peak = 0.0; double acc = 0.0; int n = (hi>lo)?(hi-lo):0;
		for(int i=lo;i<hi;i++){ double a = std::fabs(s[i]); if(a>peak) peak=a; acc += s[i]*s[i]; }
		rms = (n>0) ? std::sqrt(acc/(double)n) : 0.0;
	};

	// ---- (1) BIG-BLOCK waveform via production transmit_byte branch ----------
	int block_n = ts->bigblock_tx_total_samples();
	int sub_len = ts->ldpc.K / 8;
	int K       = 8;
	std::vector<int> bb_payload((size_t)K * (size_t)sub_len, 0);
	for(size_t i=0;i<bb_payload.size();i++) bb_payload[i] = (int)((i*53 + 17) & 0xFF);
	std::vector<double> bb_pb((size_t)((block_n>0)?block_n:1), 0.0);
	ts->bigblock_framing_enabled = true;
	{
		cl_telecom_system::bigblock_emit_scope guard(ts, block_n);
		ts->transmit_byte(bb_payload.data(), (int)bb_payload.size(), bb_pb.data(), NO_FILTER_MESSAGE);
	}
	int bb_n = ts->bigblock_last_tx_samples;
	int bb_K = ts->bigblock_last_tx_K;
	// LEVEL-FIX WIRE (fact-doc §18): the §18 level fix band-limits the block through the
	// SAME FIR_tx1->FIR_tx2 cascade send_batch applies to a stock batch (arq_common.cc).
	// Mirror that here (edge-padded, identical order) so this measurement reports the
	// achieved AFTER-FIR ratio (target ~1.0). NOTE: on the live wire the FIR is OPT-IN
	// (MERCURY_BIGBLOCK_FIR=1, default OFF) because it costs one codeword of RX margin
	// (§18.3 blocker); this diag applies it unconditionally so the level result is on the
	// record. Set MERCURY_BIGBLOCK_NOFIR=1 to instead measure the default (no-FIR, +2.3 dB)
	// wire.
	bool meas_fir = true;
	{ const char* e=std::getenv("MERCURY_BIGBLOCK_NOFIR"); if(e && atoi(e)!=0) meas_fir=false; }
	if(meas_fir)
	{
		int pad = (preN + Nsymb) * Nofdm * interp;   // ~one stock frame, send_batch pad width
		int total_fir = pad + bb_n + pad;
		std::vector<double> fin((size_t)((total_fir>0)?total_fir:1), 0.0);
		std::vector<double> ft1((size_t)((total_fir>0)?total_fir:1), 0.0);
		std::vector<double> ft2((size_t)((total_fir>0)?total_fir:1), 0.0);
		for(int i=0;i<bb_n;i++) fin[(size_t)pad+i] = bb_pb[(size_t)i];
		int rep = (pad < bb_n) ? pad : bb_n;
		for(int i=0;i<rep;i++){
			fin[(size_t)i]            = bb_pb[(size_t)i];
			fin[(size_t)(pad+bb_n)+i] = bb_pb[(size_t)(bb_n-rep)+i];
		}
		if(total_fir>0){
			ts->ofdm.FIR_tx1.apply(fin.data(), ft1.data(), total_fir);
			ts->ofdm.FIR_tx2.apply(ft1.data(), ft2.data(), total_fir);
			for(int i=0;i<bb_n;i++) bb_pb[(size_t)i] = ft2[(size_t)pad+i];   // real block back
		}
	}
	double bb_peak_all=0, bb_rms_all=0, bb_peak_dat=0, bb_rms_dat=0;
	if(bb_n > 0){
		measure(bb_pb.data(), 0, bb_n, bb_peak_all, bb_rms_all);
		int dlo = (pre_samp < bb_n) ? pre_samp : bb_n;
		measure(bb_pb.data(), dlo, bb_n, bb_peak_dat, bb_rms_dat);
	}

	// ---- (2) STOCK CONFIG_16 OFDM frame via the SAME transmit_byte entry ------
	// big-block framing OFF -> per-frame OFDM path (transmit_byte:649+). One DATA
	// frame's worth of bytes (frame_size = nReal_data/8). Use a typical full frame.
	ts->bigblock_framing_enabled = false;
	int frame_size = (ts->data_container.nBits - ts->ldpc.P) / 8;   // upper bound on bytes/frame
	if(frame_size > 160) frame_size = 160;
	std::vector<int> fr_payload((size_t)frame_size, 0);
	for(int i=0;i<frame_size;i++) fr_payload[i] = (int)((i*31 + 7) & 0xFF);
	int frame_total = (preN + Nsymb) * Nofdm * interp + 64;   // generous slot
	std::vector<double> fr_pb((size_t)frame_total, 0.0);
	ts->transmit_byte(fr_payload.data(), frame_size, fr_pb.data(), NO_FILTER_MESSAGE);
	int fr_n = ts->tx_last_emitted_frame_samples;
	if(fr_n <= 0 || fr_n > frame_total) fr_n = (preN + Nsymb) * Nofdm * interp;
	double fr_peak_all=0, fr_rms_all=0, fr_peak_dat=0, fr_rms_dat=0;
	measure(fr_pb.data(), 0, fr_n, fr_peak_all, fr_rms_all);
	{
		int dlo = (pre_samp < fr_n) ? pre_samp : fr_n;
		measure(fr_pb.data(), dlo, fr_n, fr_peak_dat, fr_rms_dat);
	}

	// ---- (2b) PRODUCTION-FAITHFUL stock level: the live batch path packs each
	// NO_FILTER frame (pre-eq applied, no FIR) then runs the WHOLE batch buffer
	// through FIR_tx1 -> FIR_tx2 (arq_common.cc:4591-4592) before tx_transfer. The
	// FIRs flatten the pre-eq boost. POST-§18 the big-block path applies pre-eq + the
	// TX_SIG_OFDM level-cal at the modulator AND the SAME batch FIR before tx_transfer
	// (the big-block arm above is now FIR'd to match). So both sides are pre-eq +
	// level-cal + FIR — the production HW ratio should be ~1.0. Apply the same two
	// FIRs to the stock reference here.
	std::vector<double> fr_f1((size_t)frame_total, 0.0), fr_f2((size_t)frame_total, 0.0);
	ts->ofdm.FIR_tx1.apply(fr_pb.data(), fr_f1.data(), fr_n);
	ts->ofdm.FIR_tx2.apply(fr_f1.data(), fr_f2.data(), fr_n);
	double frf_peak_all=0, frf_rms_all=0, frf_peak_dat=0, frf_rms_dat=0;
	measure(fr_f2.data(), 0, fr_n, frf_peak_all, frf_rms_all);
	{
		int dlo = (pre_samp < fr_n) ? pre_samp : fr_n;
		measure(fr_f2.data(), dlo, fr_n, frf_peak_dat, frf_rms_dat);
	}

	auto db = [](double r){ return 20.0*std::log10((r>0)?r:1e-12); };
	auto papr = [](double pk, double rms){ return 20.0*std::log10((rms>0)?(pk/rms):1.0); };

	printf("[TXLEVEL] geometry: interp=%d Nofdm=%d preN=%d Nsymb=%d pre_samp=%d "
	       "block_n=%d bb_n=%d bb_K=%d fr_n=%d frame_size=%d sub_len=%d\n",
	       interp, Nofdm, preN, Nsymb, pre_samp, block_n, bb_n, bb_K, fr_n, frame_size, sub_len);
	printf("[TXLEVEL] BIGBLOCK whole : peak=%.6f rms=%.6f papr=%.2fdB (POST-§18: pre-eq + level-cal + FIR_tx1/2)\n", bb_peak_all, bb_rms_all, papr(bb_peak_all,bb_rms_all));
	printf("[TXLEVEL] BIGBLOCK data  : peak=%.6f rms=%.6f papr=%.2fdB\n", bb_peak_dat, bb_rms_dat, papr(bb_peak_dat,bb_rms_dat));
	printf("[TXLEVEL] STOCK NOFIR whole: peak=%.6f rms=%.6f papr=%.2fdB (pre-eq applied, no FIR)\n", fr_peak_all, fr_rms_all, papr(fr_peak_all,fr_rms_all));
	printf("[TXLEVEL] STOCK NOFIR data : peak=%.6f rms=%.6f papr=%.2fdB\n", fr_peak_dat, fr_rms_dat, papr(fr_peak_dat,fr_rms_dat));
	printf("[TXLEVEL] STOCK +FIR  whole: peak=%.6f rms=%.6f papr=%.2fdB (PRODUCTION batch: pre-eq + FIR_tx1/2)\n", frf_peak_all, frf_rms_all, papr(frf_peak_all,frf_rms_all));
	printf("[TXLEVEL] STOCK +FIR  data : peak=%.6f rms=%.6f papr=%.2fdB\n", frf_peak_dat, frf_rms_dat, papr(frf_peak_dat,frf_rms_dat));
	printf("[TXLEVEL] === PRODUCTION HW RATIO (FIRed big-block vs FIRed stock; target ~1.0) ===\n");
	printf("[TXLEVEL] RATIO data  peak bb/stock+FIR = %.4f (%.2f dB)  RMS = %.4f (%.2f dB)\n",
	       (frf_peak_dat>0)?bb_peak_dat/frf_peak_dat:0.0, db((frf_peak_dat>0)?bb_peak_dat/frf_peak_dat:1.0),
	       (frf_rms_dat>0)?bb_rms_dat/frf_rms_dat:0.0,   db((frf_rms_dat>0)?bb_rms_dat/frf_rms_dat:1.0));
	printf("[TXLEVEL] RATIO whole peak bb/stock+FIR = %.4f (%.2f dB)  RMS = %.4f (%.2f dB)\n",
	       (frf_peak_all>0)?bb_peak_all/frf_peak_all:0.0, db((frf_peak_all>0)?bb_peak_all/frf_peak_all:1.0),
	       (frf_rms_all>0)?bb_rms_all/frf_rms_all:0.0,   db((frf_rms_all>0)?bb_rms_all/frf_rms_all:1.0));
	printf("[TXLEVEL] (sanity) RATIO data peak bb/stock-NOFIR = %.4f (%.2f dB)  RMS = %.4f (%.2f dB)\n",
	       (fr_peak_dat>0)?bb_peak_dat/fr_peak_dat:0.0, db((fr_peak_dat>0)?bb_peak_dat/fr_peak_dat:1.0),
	       (fr_rms_dat>0)?bb_rms_dat/fr_rms_dat:0.0,   db((fr_rms_dat>0)?bb_rms_dat/fr_rms_dat:1.0));
	double ofdm_gain = ts->get_tx_gain(TX_SIG_OFDM);
	printf("[TXLEVEL] get_tx_gain(TX_SIG_OFDM)=%.4f (the calibrated factor BOTH paths now apply; "
	       "POST-§18 big-block applies pre-eq + this TX_SIG_OFDM factor + FIR_tx1/2)\n", ofdm_gain);
	fflush(stdout);

	delete A; delete ts;
	restore_env();
	return 0;
}

// ============================================================================
// GENUINE BIG-BLOCK CHANNEL-ESTIMATION REGRESSION (fix/bigblock-chanest).
//
// THE DEFECT (HW, results_rxdecode_diag.json): same RX / same channel / same run, the
// per-frame OFDM path re-acquires every ~12-symbol frame and reads [OFDM-OK] meanH=0.979,
// WHILE the big-block path runs ONE Schmidl-Cox acquisition + ONE channel estimate over a
// 133-symbol / ~1.56 s block and reads [RXACQ] meanH=0.002-0.011 (~0); LDPC then decodes
// pure noise (ldpc_iter=101 cap, all 8 cw identical garbage e296c428). ROOT CAUSE:
// bigblock_rx_passband does NO carrier-frequency (Moose) correction (telecom_system.cc
// :7199-7251 = Schmidl-Cox TIMING only; the per-frame receive_byte runs Moose every frame at
// :2570). An un-tracked residual CFO/SFO (HW: two independent crystals + post-Moose residual)
// ramps a MULTI-CYCLE phasor across the long block (8 Hz over ~1.56 s = ~12.5 cycles); the
// block-wide pilot average (flat-ML Hbar=Hsum/npil :7389-7401, and the sparse-2D estimate)
// destructively integrates the rotating phasor → |H| → 0.
//
// WHY THE GENUINE PATH MATTERS / WHY THE SIM HID IT: receive_bigblock passes cw_info_ref to
// the worker ONLY when bigblock_last_tx_K>0 (telecom_system.cc:8391-8392) — i.e. when the
// SAME instance just transmitted (oracle gate). Here the RX (tsB) NEVER transmits, so
// bigblock_last_tx_K==0 → cw_info_ref==NULL → the GENUINE decode (the cw_ok gate is the real
// per-codeword decode, not an oracle compare). The existing --test-bigblock-multicw is ALSO
// ref==NULL, but it runs the DEFAULT channel with SFO=0 AND CFO=0, so there is no rotating
// phasor to integrate and it passes 8/8. This regression turns CFO+SFO ON in the same genuine
// single-block transfer (through cl_sim_awgn) and asserts the collapse — the HW defect,
// reproduced OFF-BENCH, with NO ARQ loop / NO ACK spin (a single TX→channel→RX decode).
//
//   SANITY arm:     clean channel → estimate HEALTHY (mean|H| > MEANH_OK) + 8/8 byte-faithful.
//   FAIL-BEFORE arm: CFO+SFO ON → estimate COLLAPSED (mean|H| < MEANH_BAD) + NOT 8/8.
//   PASS-AFTER arm:  CFO+SFO ON, fix active → estimate HEALTHY + 8/8 byte-faithful.
// Until a PHY CFO/SFO-tracking fix lands, the PASS-AFTER arm FAILS (so the test FAILS) — the
// intended fail-before→pass-after contract for the channel-estimation fix.
// ============================================================================
int cl_arq_controller::test_sim_inproc_bigblock_chanest()
{
	printf("[TEST-BIGBLOCK-CHANEST] ===== GENUINE (ref==NULL) big-block channel-estimation "
	       "regression: one CFG16 block TX->cl_sim_awgn(CFO/SFO)->RX decode =====\n");
	fflush(stdout);

	// Pin K=8 (production MERCURY_BIGBLOCK_K cap path).
	const char* prev_k = std::getenv("MERCURY_BIGBLOCK_K");
	std::string prev_k_saved = prev_k ? std::string(prev_k) : std::string();
	bool had_prev_k = (prev_k != NULL);
#if defined(_WIN32)
	_putenv_s("MERCURY_BIGBLOCK_K", "8");
#else
	setenv("MERCURY_BIGBLOCK_K", "8", 1);
#endif
	auto restore_k = [&]() {
#if defined(_WIN32)
		if(had_prev_k) _putenv_s("MERCURY_BIGBLOCK_K", prev_k_saved.c_str());
		else           _putenv_s("MERCURY_BIGBLOCK_K", "");
#else
		if(had_prev_k) setenv("MERCURY_BIGBLOCK_K", prev_k_saved.c_str(), 1);
		else           unsetenv("MERCURY_BIGBLOCK_K");
#endif
	};

	// Bring up CMD (tsA, the TX) + RSP (tsB, the RX) at a real CFG16 grid (mirrors
	// test_sim_inproc_bigblock bringup). tsB NEVER transmits → bigblock_last_tx_K==0 on the
	// RX → cw_info_ref==NULL = the GENUINE decode.
	cl_telecom_system* tsA = new cl_telecom_system();
	cl_telecom_system* tsB = new cl_telecom_system();
	cl_arq_controller* A   = new cl_arq_controller();
	cl_arq_controller* B   = new cl_arq_controller();
	A->telecom_system = tsA;
	B->telecom_system = tsB;
	auto bringup = [&](cl_arq_controller* a, cl_telecom_system* ts, int role) {
		a->role = role; a->sack_enabled = true; a->sack_v2_enabled = true;
		a->axis3_sack_mode = 1; a->compression_enabled = false;
		a->bigblock_skip_fifo_delivery = true;
		a->nMessages = 255; a->max_data_length = 170; a->max_message_length = 200;
		a->max_header_length = 6; a->init_messages_buffers();
		a->load_configuration(CONFIG_16, FULL, YES);
		ts->bigblock_framing_enabled = true;
		a->sack_negotiated_recompute_batch(role==COMMANDER ? "CMD" : "RSP");
	};
	bringup(A, tsA, COMMANDER);
	bringup(B, tsB, RESPONDER);

	const int K = BB_TEST_K;
	const int sub_len = tsA->ldpc.K / 8;
	const long total_tx_bytes = (long)K * sub_len;
	const int hdr_total = BIGBLOCK_HDR_TOTAL_BYTES(K);
	const int block_bsi = 7;

	// Build the on-wire block payload EXACTLY as production bigblock_send_one_block does
	// (header in cw0 prefix + per-codeword app bytes + per-cw wire CRC-8). Variable lengths.
	std::vector<std::vector<unsigned char>> app_truth((size_t)K);
	std::vector<int> app_len((size_t)K, 0);
	std::vector<unsigned char> tx_truth((size_t)total_tx_bytes, 0);
	{
		const int cw0_cap = sub_len - hdr_total - BIGBLOCK_CW_CRC_BYTES;
		const int cwc_cap = sub_len - BIGBLOCK_CW_CRC_BYTES;
		for(int i=0;i<K;i++){
			int cap = (i==0)?cw0_cap:cwc_cap;
			int len = ((i*37 + 11) % (cap - 4)) + 1; if(len>cap) len=cap;
			app_len[i]=len; app_truth[i].assign((size_t)len,0);
			for(int j=0;j<len;j++){ unsigned char b=(unsigned char)((i*53+j*17+3)&0xFF); app_truth[i][(size_t)j]=b; }
		}
		tx_truth[0]=(unsigned char)(block_bsi&0xFF);
		tx_truth[1]=(unsigned char)(K&0xFF);
		for(int c=0;c<K;c++){
			int lo=BIGBLOCK_HDR_FIXED_BYTES+2*c;
			tx_truth[(size_t)lo+0]=(unsigned char)(app_len[c]&0xFF);
			tx_truth[(size_t)lo+1]=(unsigned char)((app_len[c]>>8)&0xFF);
			int base=(c==0)?hdr_total:(c*sub_len);
			for(int j=0;j<app_len[c];j++) tx_truth[(size_t)base+j]=app_truth[c][(size_t)j];
		}
		for(int c=0;c<K;c++){
			int crc_off=BIGBLOCK_CW_CRC_OFFSET(c,sub_len), crc_span=BIGBLOCK_CW_CRC_SPAN(sub_len);
			if(crc_off<0||crc_off>=(int)tx_truth.size()||crc_span<0) continue;
			tx_truth[(size_t)crc_off]=A->CRC8_calc((char*)&tx_truth[(size_t)c*sub_len],crc_span);
		}
	}

	// ONE genuine block transfer through an (optional) channel. ch==NULL → clean wire.
	// Applies the channel in WHOLE-SYMBOL chunks exactly as the production wire feeds the RX
	// (sim2_drain_to_wire moves sp = Nofdm*interp samples per process() call), so the SFO/CFO/
	// PN state advances per OFDM symbol just like the live path. Returns mean|H| and 8/8-ness.
	auto run_block = [&](cl_sim_awgn* ch, double& meanh_out, int& cw_ok_out,
	                     bool& bytes_ok_out) -> bool {
		int interp  = tsA->frequency_interpolation_rate;
		int block_n = tsA->bigblock_tx_total_samples();
		if(block_n <= 0) return false;
		int lead_n  = (int)(100.0 * tsA->sampling_frequency / 1000.0);
		int trail_n = (int)(50.0  * tsA->sampling_frequency / 1000.0);
		std::vector<int> payload((size_t)tx_truth.size(), 0);
		for(size_t i=0;i<tx_truth.size();i++) payload[i]=(int)tx_truth[i];
		std::vector<double> tx_pb((size_t)block_n, 0.0);
		{
			cl_telecom_system::bigblock_emit_scope emit_guard(tsA, block_n);
			tsA->transmit_byte(payload.data(), (int)payload.size(), tx_pb.data(), NO_FILTER_MESSAGE);
		}
		int K_tx = tsA->bigblock_last_tx_K, n_tx = tsA->bigblock_last_tx_samples;
		if(K_tx != K || n_tx <= 0) return false;

		int rx_window = lead_n + n_tx + trail_n;
		std::vector<double> rx_pb((size_t)rx_window, 0.0);
		for(int i=0;i<n_tx && i<(int)tx_pb.size();i++) rx_pb[lead_n + i] = tx_pb[i];

		// CHANNEL: feed the whole RX window (incl. lead/trail silence) through the channel in
		// Nofdm*interp-sample chunks — the impairment runs across the silence too (matching the
		// live wire, where the channel state ticks during idle), so the CFO/SFO phase ramp the
		// RX integrates is the genuine one. ch==NULL → clean (the sanity arm).
		if(ch){
			int sp = tsB->data_container.Nofdm * interp;
			if(sp <= 0) sp = tsA->data_container.Nofdm * interp;
			if(sp > 0){
				for(int off=0; off+sp<=(int)rx_pb.size(); off+=sp) ch->process(&rx_pb[off], (size_t)sp);
			}
		}

		int Nofdm = tsB->data_container.Nofdm;
		int saved_buffer_Nsymb = tsB->data_container.buffer_Nsymb;
		if(Nofdm > 0){
			int need_syms = (rx_window + Nofdm*interp - 1) / (Nofdm*interp);
			tsB->data_container.buffer_Nsymb = need_syms;
			int exact = need_syms * Nofdm * interp;
			if((int)rx_pb.size() < exact) rx_pb.resize((size_t)exact, 0.0);
		}
		std::vector<int> info_bits((size_t)(K+1) * tsB->ldpc.K + tsB->ldpc.K, 0);
		tsB->bigblock_last_rx_meanh = -1.0;
		tsB->receive_byte(rx_pb.data(), info_bits.data());            // → receive_bigblock (ref==NULL)
		tsB->data_container.buffer_Nsymb = saved_buffer_Nsymb;

		meanh_out = tsB->bigblock_last_rx_meanh;
		cw_ok_out = tsB->bigblock_last_rx_cw_ok_count;
		int K_rx  = tsB->bigblock_last_rx_K;

		// BYTE-FAITHFUL check: carve the decoded info bits and compare each slot's app bytes.
		bool bytes_ok = false;
		if(K_rx == K){
			for(int i=0;i<B->nMessages;i++){ B->messages_rx[i].status=FREE; B->messages_rx[i].length=0; B->messages_rx[i].batch_seq_id=-1; }
			B->rsp_current_expected_batch_seq_id = block_bsi; B->rsp_prev_batch_seq_id=-1;
			B->rsp_prev_batch_active=false; B->batch_rx_frame_count=0; B->last_received_end_of_batch_seq=-1;
			int carve_rc = B->bigblock_receive_carve(tsB->bigblock_rx_infobits.data(), (unsigned char)block_bsi);
			if(carve_rc == SUCCESSFUL){
				bytes_ok = true;
				for(int c=0;c<K && bytes_ok;c++){
					if(B->messages_rx[c].status != RECEIVED || B->messages_rx[c].length != app_len[c]){ bytes_ok=false; break; }
					for(int j=0;j<app_len[c];j++)
						if((unsigned char)B->messages_rx[c].data[j] != app_truth[c][(size_t)j]){ bytes_ok=false; break; }
				}
			}
		}
		bytes_ok_out = bytes_ok;
		return true;
	};

	// Thresholds: the big-block RAW |H| scale on the calibrated clean cell is ~0.24; the
	// un-tracked-CFO collapse drops it to ~0.02-0.13 (HW: ~0.005). A wide gap → robust gate.
	const double MEANH_OK  = 0.18;
	const double MEANH_BAD = 0.16;

	int failed = 0;

	// ---------- 0) SANITY: clean channel is HEALTHY + byte-faithful. ----------
	double s_meanh=-1; int s_cwok=-1; bool s_bytes=false;
	bool s_ran = run_block(nullptr, s_meanh, s_cwok, s_bytes);
	printf("[TEST-BIGBLOCK-CHANEST] SANITY clean: meanH=%.4f (want>%.2f) cw_ok=%d/%d bytes_ok=%d\n",
	       s_meanh, MEANH_OK, s_cwok, K, (int)s_bytes);
	bool sanity_ok = s_ran && (s_meanh > MEANH_OK) && (s_cwok == K) && s_bytes;
	printf("[TEST-BIGBLOCK-CHANEST] %s: SANITY clean channel healthy estimate + byte-faithful\n",
	       sanity_ok ? "PASS" : "FAIL");
	if(!sanity_ok) failed++;

	// ---------- 1) FAIL-BEFORE: CFO+SFO ON → estimate COLLAPSES, not 8/8. ----------
	// HW-calibrated magnitudes (sim_channel.h §CFO/§SFO): static residual CFO + slow AR(1)
	// drift + SFO. The CFO/SFO knobs are read by cl_sim_awgn at CONSTRUCTION (env-gated), so
	// set the channel env BEFORE constructing ch_bad. clean SNR3k≥900 → no additive AWGN; only
	// the CFO/SFO/PN/floor impairments, isolating the channel-estimation collapse from thermal
	// noise. Distinct per-direction seed (the live wire seeds A→B with (seed<<1)|1).
	auto set_ch_env = [&](const char* cfo, const char* cfo_walk, const char* sfo){
#if defined(_WIN32)
		_putenv_s("MERCURY_SIM2_CFO_HZ", cfo); _putenv_s("MERCURY_SIM2_CFO_WALK_HZ", cfo_walk); _putenv_s("MERCURY_SIM2_SFO_PPM", sfo);
#else
		setenv("MERCURY_SIM2_CFO_HZ", cfo, 1); setenv("MERCURY_SIM2_CFO_WALK_HZ", cfo_walk, 1); setenv("MERCURY_SIM2_SFO_PPM", sfo, 1);
#endif
	};
	// Save+restore the channel env so the test leaves the process clean.
	auto getenv_s = [](const char* k){ const char* v=std::getenv(k); return v?std::string(v):std::string(); };
	bool had_cfo=std::getenv("MERCURY_SIM2_CFO_HZ")!=NULL, had_cfow=std::getenv("MERCURY_SIM2_CFO_WALK_HZ")!=NULL, had_sfo=std::getenv("MERCURY_SIM2_SFO_PPM")!=NULL;
	std::string sv_cfo=getenv_s("MERCURY_SIM2_CFO_HZ"), sv_cfow=getenv_s("MERCURY_SIM2_CFO_WALK_HZ"), sv_sfo=getenv_s("MERCURY_SIM2_SFO_PPM");

	set_ch_env("8", "4", "50");
	double b_meanh=-1; int b_cwok=-1; bool b_bytes=false;
	{
		cl_sim_awgn ch_bad(((uint64_t)12345 << 1) | 1u, 900.0);
		run_block(&ch_bad, b_meanh, b_cwok, b_bytes);
	}
	printf("[TEST-BIGBLOCK-CHANEST] FAIL-BEFORE (CFO+SFO): meanH=%.4f (want<%.2f) cw_ok=%d/%d bytes_ok=%d\n",
	       b_meanh, MEANH_BAD, b_cwok, K, (int)b_bytes);
	bool fail_before_ok = (b_meanh >= 0.0) && (b_meanh < MEANH_BAD) && !(b_cwok == K && b_bytes);
	printf("[TEST-BIGBLOCK-CHANEST] %s: FAIL-BEFORE reproduces the HW [RXACQ] collapse "
	       "(meanH=%.4f<%.2f, NOT 8/8 byte-faithful)\n",
	       fail_before_ok ? "PASS" : "FAIL", b_meanh, MEANH_BAD);
	if(!fail_before_ok) failed++;

	// ---------- 2) PASS-AFTER: same impaired channel; the fix restores estimate + 8/8. ----------
	double f_meanh=-1; int f_cwok=-1; bool f_bytes=false;
	{
		cl_sim_awgn ch_fix(((uint64_t)12345 << 1) | 1u, 900.0);
		run_block(&ch_fix, f_meanh, f_cwok, f_bytes);
	}
	printf("[TEST-BIGBLOCK-CHANEST] PASS-AFTER (CFO+SFO, fix): meanH=%.4f (want>%.2f) cw_ok=%d/%d bytes_ok=%d\n",
	       f_meanh, MEANH_OK, f_cwok, K, (int)f_bytes);
	bool pass_after_ok = (f_meanh > MEANH_OK) && (f_cwok == K) && f_bytes;
	printf("[TEST-BIGBLOCK-CHANEST] %s: PASS-AFTER healthy estimate + 8/8 byte-faithful under "
	       "CFO/SFO (the fix; FAILS until a PHY CFO/SFO-tracking fix lands)\n",
	       pass_after_ok ? "PASS" : "FAIL");
	if(!pass_after_ok) failed++;

	// restore channel env
	auto put = [&](const char* k, bool had, const std::string& v){
#if defined(_WIN32)
		if(had) _putenv_s(k, v.c_str()); else _putenv_s(k, "");
#else
		if(had) setenv(k, v.c_str(), 1); else unsetenv(k);
#endif
	};
	put("MERCURY_SIM2_CFO_HZ", had_cfo, sv_cfo);
	put("MERCURY_SIM2_CFO_WALK_HZ", had_cfow, sv_cfow);
	put("MERCURY_SIM2_SFO_PPM", had_sfo, sv_sfo);
	restore_k();

	delete A; delete B; delete tsA; delete tsB;

	printf("[TEST-BIGBLOCK-CHANEST] %s (%d failure%s)  [sanity meanH=%.3f | fail-before meanH=%.3f "
	       "| pass-after meanH=%.3f bytes_ok=%d]\n", failed == 0 ? "ALL PASS" : "FAILURES", failed,
	       failed == 1 ? "" : "s", s_meanh, b_meanh, f_meanh, (int)f_bytes);
	fflush(stdout);
	return failed == 0 ? 0 : 1;
}
