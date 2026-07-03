// ============================================================================
// §12/§13 — RE-STAGE re-queue orphan/reorder: in-process regression (test-only)
// ============================================================================
//
// CLI: --test-restage-requeue-orphan
//
// Pairs with mercury/fact-documents/silent-corruption-residual.md §12/§13 (the
// ROOT of the residual WGN:25 SILENT byte-corruption confirmed in cohort-2). On a
// demote / BREAK / CFG16-HOLD re-stage the CMD must re-queue every in-flight
// (un-ACKed) messages_tx[] frame back into fifo_buffer_tx to re-frame at the new
// config. The 7 demote sites open-coded this as a FORWARD-iter fifo_buffer_tx.push()
// (append to the BACK) with the return IGNORED and the frame freed UNCONDITIONALLY.
// When newer app data is already queued in the (chronically-full) fifo, push()
// appends the in-flight block BEHIND it; the re-stage then rolls cmd_batch_seq_id
// back to the in-flight bsi and the next-built batch pops the FRONT (= newer
// source) under the OLD bsi -> a whole-tail positional SHIFT the period-256 ruler
// reads as 100% corrupt (mismatch == rx-first_bad). (When the fifo is FULL the same
// push() SILENTLY DROPS -> the block is ORPHANED -> the same shift.)
//
// THIS TEST drives the PRODUCTION helper restage_requeue_tx_messages() against a
// fifo that already holds newer app data, and asserts the re-queued in-flight block
// comes out CONTIGUOUS + IN-ORDER at the FRONT (ahead of the newer data), byte-exact,
// zero loss.
//
// FAIL-BEFORE / PASS-AFTER CONTRACT (CLAUDE.md §3):
//   PRE-FIX  (MERCURY_RESTAGE_ORPHAN_DEFEAT=1): forward-iter push()-to-BACK -> the
//            in-flight block lands BEHIND the newer data -> out-of-order -> FAIL
//            (the observed positional shift).
//   POST-FIX (default):                          reverse-iter push_front() -> the
//            in-flight block lands CONTIGUOUS at the front, in order -> PASS.
//
// Deterministic, no RF, no sockets, runs in well under 1 s. 0 = PASS, 1 = FAIL.
// ============================================================================

#include "datalink_layer/arq.h"
#include "datalink_layer/datalink_defines.h"
#include "common/common_defines.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <algorithm>

int cl_arq_controller::test_restage_requeue_orphan()
{
	const char* dfe = std::getenv("MERCURY_RESTAGE_ORPHAN_DEFEAT");
	bool defeat = (dfe && *dfe && atoi(dfe) != 0);
	printf("[TEST-RESTAGE] §12 re-stage re-queue orphan/reorder regression (defeat=%d)\n",
		(int)defeat);
	fflush(stdout);

	// --- Setup: a commander controller with an allocated messages_tx[] + fifos ---
	this->original_role       = COMMANDER;
	this->link_status         = CONNECTED;
	this->sack_v2_enabled     = true;
	this->compression_enabled = false;
	this->nMessages           = 255;
	this->max_data_length     = 170;
	this->max_message_length  = 200;
	this->max_header_length   = 6;
	int rc = init_messages_buffers();
	if(rc != SUCCESSFUL)
	{
		printf("[TEST-RESTAGE] ERROR: init_messages_buffers rc=%d\n", rc);
		fflush(stdout);
		return 1;
	}

	const int FIFO_SZ = 65536;
	if(this->fifo_buffer_tx.set_size(FIFO_SZ) != SUCCESSFUL)
	{
		printf("[TEST-RESTAGE] ERROR: fifo_buffer_tx.set_size failed\n");
		fflush(stdout);
		return 1;
	}
	this->fifo_buffer_tx.flush();

	// NEWER app data D already sitting in the fifo — the bytes the app fed AFTER
	// the in-flight batch was popped to build it (value range 0xC0..0xEF, disjoint
	// from B below so the two streams can never alias).
	const int DLEN = 2000;
	std::vector<char> D((size_t)DLEN);
	for(int i = 0; i < DLEN; i++)
		D[i] = (char)(0xC0 + (i % 0x30));
	this->fifo_buffer_tx.push(D.data(), DLEN);

	// IN-FLIGHT block B staged in messages_tx[0..K-1] — the un-ACKed frames a
	// re-stage must re-queue, in SLOT (== source) order. Values 0x00..0x9F, disjoint
	// from D. There is ROOM in the fifo for B (isolates the REORDER defect from the
	// separate fifo-full DROP defect, which the ingestion reserve prevents end-to-end).
	const int K    = 20;
	const int FLEN = 100;
	std::vector<char> B((size_t)K * FLEN);
	for(int f = 0; f < K; f++)
	{
		for(int j = 0; j < FLEN; j++)
		{
			char v = (char)(((f * FLEN) + j) % 0xA0);   // 0x00..0x9F
			B[(size_t)f * FLEN + j]      = v;
			this->messages_tx[f].data[j] = v;
		}
		this->messages_tx[f].length = FLEN;
		this->messages_tx[f].id     = (char)f;
		this->messages_tx[f].status = PENDING_ACK;      // in-flight (non-FREE)
	}
	for(int f = K; f < this->nMessages; f++)
		this->messages_tx[f].status = FREE;

	// --- Under test: the PRODUCTION re-stage re-queue ---
	restage_requeue_tx_messages();

	// Drain the entire fifo and reconstruct the re-queued stream.
	std::vector<char> out;
	char tmp[512];
	for(int guard = 0; guard < 1000000; guard++)
	{
		int got = this->fifo_buffer_tx.pop(tmp, (int)sizeof(tmp));
		if(got <= 0) break;
		for(int i = 0; i < got; i++) out.push_back(tmp[i]);
	}

	// Expected (post-fix): B (slot order) then D — the in-flight block CONTIGUOUS
	// at the FRONT, then the newer app data.
	std::vector<char> expect;
	expect.insert(expect.end(), B.begin(), B.end());
	expect.insert(expect.end(), D.begin(), D.end());

	bool len_ok   = (out.size() == expect.size());
	bool order_ok = true;
	int  first_bad = -1;
	int  cmp = (int)std::min(out.size(), expect.size());
	for(int i = 0; i < cmp; i++)
	{
		if(out[i] != expect[i]) { order_ok = false; first_bad = i; break; }
	}

	printf("[TEST-RESTAGE] requeued=%d expect=%d len_ok=%d order_ok=%d first_bad=%d\n",
		(int)out.size(), (int)expect.size(), (int)len_ok, (int)order_ok, first_bad);
	fflush(stdout);

	if(len_ok && order_ok)
	{
		printf("[TEST-RESTAGE] PASS — in-flight block re-queued CONTIGUOUS + IN-ORDER "
			"ahead of newer data, zero loss\n");
		fflush(stdout);
		return 0;
	}
	printf("[TEST-RESTAGE] FAIL — re-stage %s -> positional SHIFT (the WGN:25 silent "
		"byte-corruption, first_bad=%d)\n",
		!len_ok ? "DROPPED in-flight bytes" : "REORDERED the stream", first_bad);
	fflush(stdout);
	return 1;
}
