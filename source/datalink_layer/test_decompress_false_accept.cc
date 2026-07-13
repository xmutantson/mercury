// ============================================================================
// Streaming decompress-failure SILENT-FALSE-ACCEPT regression (test-only)
// ============================================================================
//
// CLI: --test-decompress-false-accept
//
// Fact-doc: mercury/fact-documents/residual-silent-corruption-wgn25.md
//
// THE BUG (a THIRD distinct silent-false-accept trigger, reproduced ~20% of
// WGN:25 clean-load real-audio cells on baabacd5, deterministic onset offsets):
// with streaming PPMd compression + Winlink-dict priming active, the RX streaming
// model falls out of lockstep with the TX under the marginal-SNR OUT-OF-ORDER SACK
// / prev cross-storage delivery regime. A later batch then decodes to -1
// ("[DECOMPRESS] PPMd error: expected N, got -1") EVEN THOUGH every frame passed
// its per-frame CRC and the batch reassembled COMPLETE (16/16). The old
// copy_data_to_buffer() decompress-error fallback fifo_push_rx'd the RAW COMPRESSED
// blob straight to the app — delivering undecodable binary garbage as the
// "message". No [RSP-V2-GAP-ABORT] fires (the batch delivered contiguously); md5
// FALSE; the garbage cascades to the end of the stream. This is DISTINCT from the
// Fix A prev-orphan (messages_rx_prev[]) and the baabacd5 current-orphan
// (messages_rx[]) reassembly bugs — those are frame-count faults; this is a
// compression-layer decode fault, and it never touches the seal-orphan detector.
//
// THE FIX: a decompress failure means we DO NOT HAVE valid plaintext, so we must
// deliver NOTHING (never the compressed bytes). copy_data_to_buffer() now, on a
// streaming decompress failure, LOUD-detects ([RSP-DECOMPRESS-FALSE-ACCEPT-
// BLOCKED], rsp_decompress_false_accept_blocked++), delivers ZERO bytes, and
// streaming_reset()s for resync. MERCURY_DECOMPRESS_RAWPUSH_DEFEAT=1 restores the
// old silent raw-push (fail-before arm).
//
// FAITHFUL DETERMINISTIC DESYNC (no RF, no sockets): two REAL cl_compressor
// instances (a TX and the controller's own RX member) are dict-primed identically
// (v2), advanced ONE streaming batch in lockstep (proving they start SYNCED), then
// the RX alone is streaming_reset() — exactly the production lockstep break where
// the RX resets (drops to cold, dict_version_active=0) but the TX does not. The TX
// then compresses the next batch as a streaming continuation (dict_ver=2); routed
// through the REAL copy_data_to_buffer() the RX's decode fails deterministically
// (dict-version mismatch -> -1), exercising the exact fallback the fix guards. The
// controller's fifo_buffer_rx is the byte oracle.
//
// Returns 0 on PASS, 1 on FAIL.
// ============================================================================

#include "datalink_layer/arq.h"
#include "datalink_layer/datalink_defines.h"
#include "common/common_defines.h"
#include "compression/mercury_compress.h"
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cstdlib>

int cl_arq_controller::test_decompress_false_accept()
{
	bool defeat = false;
	{ const char* e = std::getenv("MERCURY_DECOMPRESS_RAWPUSH_DEFEAT");
	  if(e && *e && atoi(e)!=0) defeat = true; }

	// ── Step 0: allocate + a compression-viable OFDM config ──────────────────
	this->nMessages          = 120;
	this->max_data_length    = 170;
	this->max_message_length = 200;
	this->max_header_length  = 6;
	int alloc_rc = init_messages_buffers();
	if(alloc_rc != SUCCESSFUL)
	{
		printf("[TEST-DECOMP-FA] ERROR: init_messages_buffers() failed (rc=%d)\n", alloc_rc);
		fflush(stdout);
		return 1;
	}
	this->compression_enabled   = true;
	this->encryption_enabled    = false;
	this->sack_v2_enabled       = true;
	this->sack_enabled          = true;
	this->current_configuration = 0;     // OFDM (non-robust) -> compression viable
	this->header_carries_d5     = true;  // non-robust config carries the D5 wired count
	this->original_role         = COMMANDER;  // no app-socket drain on fifo_push_rx
	this->role                  = COMMANDER;
	this->data_batch_size       = 1;     // single-frame batch (the sole decode unit)

	if(!compression_viable_for_batch())
	{
		printf("[TEST-DECOMP-FA] ERROR: compression not viable for the test batch dims\n");
		fflush(stdout);
		return 1;
	}

	// ── Step 1: two REAL compressors, dict-primed identically (SYNCED) ───────
	cl_compressor txc;
	txc.init();
	txc.set_dict_priming(true);
	txc.streaming_enable();
	compressor.init();
	compressor.set_dict_priming(true);
	compressor.streaming_enable();

	if(txc.active_dict_version() <= 0 || compressor.active_dict_version() <= 0)
	{
		printf("[TEST-DECOMP-FA] ERROR: dict priming produced version tx=%d rx=%d (need >0 "
		       "for the deterministic dict-mismatch desync)\n",
			txc.active_dict_version(), compressor.active_dict_version());
		fflush(stdout);
		return 1;
	}

	// Advance BOTH by ONE streaming batch, in lockstep, to prove they start SYNCED
	// (a genuine MID-STREAM desync, not a cold-start artifact).
	const int MSGLEN = 256;
	char plain1[512];
	for(int i=0;i<MSGLEN;i++) plain1[i] = "EMERGENCY TRAFFIC "[i % 18];
	char f1[1024];
	int f1len = txc.compress_block(plain1, MSGLEN, f1, sizeof(f1));
	char o1[1024];
	int o1len = compressor.decompress_block(f1, f1len, o1, sizeof(o1));
	if(f1len <= 0 || o1len != MSGLEN || memcmp(o1, plain1, MSGLEN) != 0)
	{
		printf("[TEST-DECOMP-FA] ERROR(vacuous): in-sync batch did not round-trip "
		       "(f1len=%d o1len=%d expected=%d) — setup not synced\n",
			f1len, o1len, MSGLEN);
		fflush(stdout);
		return 1;
	}

	// ── Step 2: DESYNC — reset the RX only (TX stays warm/dict-primed) ───────
	// The production lockstep break: RX streaming_reset() drops to cold
	// (dict_version_active=0); the TX did NOT reset (still dict v2).
	compressor.streaming_reset();

	char plain2[512];
	for(int i=0;i<MSGLEN;i++) plain2[i] = "WELFARE CHECK GRID "[i % 19];
	char frame2[1024];
	int f2len = txc.compress_block(plain2, MSGLEN, frame2, sizeof(frame2));
	if(f2len <= 0 || f2len > this->max_data_length + this->max_header_length)
	{
		printf("[TEST-DECOMP-FA] ERROR(vacuous): frame2 size %d does not fit one slot\n", f2len);
		fflush(stdout);
		return 1;
	}
	// The frame must be a COMPRESSED (non-RAW) frame so its raw bytes are genuine
	// garbage vs the plaintext (a RAW frame would decode fine — no desync to test).
	int frame2_algo = (unsigned char)frame2[0] & COMPRESS_ALGO_MASK;
	if(frame2_algo == COMPRESS_ALGO_RAW)
	{
		printf("[TEST-DECOMP-FA] ERROR(vacuous): frame2 is RAW (algo=0) — not a "
		       "compressed frame; the desync would not fail decode\n");
		fflush(stdout);
		return 1;
	}

	// ── Step 3: route the desynced frame through the REAL reassembler ────────
	for(int i=0;i<this->nMessages;i++){ messages_rx[i].status=FREE; messages_rx[i].length=0; }
	memcpy(messages_rx[0].data, frame2, f2len);
	messages_rx[0].length = f2len;
	messages_rx[0].status = ACKED;

	fifo_buffer_rx.set_size(65536);
	fifo_buffer_rx.flush();
	auto occ_rx = [&]() -> int { return fifo_buffer_rx.get_size() - fifo_buffer_rx.get_free_size(); };

	long long blocked_before = rsp_decompress_false_accept_blocked;
	copy_data_to_buffer();
	int delivered = occ_rx();
	long long blocked_after = rsp_decompress_false_accept_blocked;

	printf("[TEST-DECOMP-FA] defeat=%d f2len=%d plaintext=%d delivered=%d blocked(+%lld)\n",
		defeat ? 1 : 0, f2len, MSGLEN, delivered, blocked_after - blocked_before);
	fflush(stdout);

	bool pass = true;

	if(defeat)
	{
		// FAIL-BEFORE: the OLD silent-false-accept pushed the raw compressed blob.
		// (a) the app FIFO received exactly the compressed blob length (garbage).
		if(delivered != f2len)
		{ printf("[TEST-DECOMP-FA] FAIL(defeat): delivered=%d expected raw blob len=%d\n",
			delivered, f2len); pass = false; }
		// (b) and the delivered bytes are the compressed blob, NOT the plaintext
		//     message (a genuine silent corruption).
		else
		{
			char got[1024];
			int n = fifo_buffer_rx.pop(got, f2len);
			if(n != f2len || memcmp(got, frame2, f2len) != 0)
			{ printf("[TEST-DECOMP-FA] FAIL(defeat): delivered bytes are not the raw "
				"compressed blob (n=%d)\n", n); pass = false; }
			if(f2len == MSGLEN && memcmp(frame2, plain2, MSGLEN) == 0)
			{ printf("[TEST-DECOMP-FA] FAIL(vacuous): compressed blob == plaintext — not garbage\n");
				pass = false; }
		}
		// (c) the loud detect did NOT fire (old silent path).
		if(blocked_after != blocked_before)
		{ printf("[TEST-DECOMP-FA] FAIL(defeat): blocked counter moved under the silent path\n");
			pass = false; }
	}
	else
	{
		// PASS-AFTER: never deliver undecodable garbage.
		// (a) ZERO bytes delivered for the failed batch (a detectable gap, not corruption).
		if(delivered != 0)
		{ printf("[TEST-DECOMP-FA] FAIL: delivered=%d — expected 0 (no garbage to the app)\n",
			delivered); pass = false; }
		// (b) the LOUD detect fired exactly once (proves the decompress-failure branch
		//     was reached — the vacuity guard: if the decode had NOT failed, this stays 0).
		if(blocked_after != blocked_before + 1)
		{ printf("[TEST-DECOMP-FA] FAIL(vacuous): blocked counter +%lld (expected +1); the "
			"decode may not have failed\n", blocked_after - blocked_before); pass = false; }
		// (c) streaming was reset (resync) — cold, so no cascade.
		if(compressor.active_dict_version() != 0)
		{ printf("[TEST-DECOMP-FA] FAIL: streaming not reset (dict_version=%d) after failure\n",
			compressor.active_dict_version()); pass = false; }
	}

	printf("[TEST-DECOMP-FA] %s (%s arm)\n",
		pass ? "PASS" : "FAIL", defeat ? "fail-before/defeat" : "pass-after/fix");
	fflush(stdout);
	return pass ? 0 : 1;
}
