// ============================================================================
// FIX-6 — RX-delivery drain backpressure: in-process regression (test-only)
// ============================================================================
//
// CLI: --test-rx-drain-backpressure
//
// Pairs with bigblock_p3_hw/_wallb/fix6/STALL_ROOTCAUSE.md +
// fix6/FIX6_DESIGN.md (the §5 cross-layer audit of fifo_buffer_rx +
// tcp_socket_data). Captures the deterministic "61,621-byte" application-delivery
// stall observed on HW bench3: the responder pops bytes OUT of fifo_buffer_rx
// (arq_responder.cc) then calls tcp_socket_data.transmit() — a bare NON-BLOCKING
// send() — and pre-fix IGNORED the result. On a full OS send buffer send()
// returns short / would-block; the already-popped bytes were neither retried nor
// pushed back -> permanently LOST. The CMD had already ACKed the batch (no
// retransmit) -> the application stream plateaued forever while the link stayed
// healthy.
//
// WHY THE IN-PROCESS SIM CANNOT REPRO IT (STALL_ROOTCAUSE §3.4): the 2-instance
// SIM (arq_commander.cc) drains fifo_buffer_rx DIRECTLY into an in-memory rx_buf,
// bypassing the TCP data socket entirely — there is no send()/OS-send-buffer, so
// the loss can never fire. A faithful repro REQUIRES the socket path. This test
// supplies it WITHOUT real sockets via the build-time seam
// cl_tcp_socket::g_test_transmit_hook (tcp_socket.cc): the hook models a
// back-pressured app socket (accepts only a capped number of bytes per call, then
// would-blocks) and collects every byte the socket "accepted" so the test can
// reconstruct the delivered stream and assert byte-exact, in-order, no-loss.
//
// FAIL-BEFORE / PASS-AFTER CONTRACT:
//   PRE-FIX  (HEAD 62cb3dc): the drain discards the popped bytes on every
//            would-block -> the reconstructed stream is SHORT (a gap) -> FAIL.
//   POST-FIX (FIX-6):        the unsent tail is stashed in rx_deliver_pending and
//            re-sent in order next drain; once the stub raises its cap the ENTIRE
//            N-byte pattern is delivered IN ORDER, no gap, no duplication -> PASS.
//
// Deterministic, no RF, no real sockets, runs in well under 1 s.
// Returns 0 on PASS, 1 on FAIL.
// ============================================================================

#include "datalink_layer/arq.h"
#include "datalink_layer/datalink_defines.h"
#include "datalink_layer/tcp_socket.h"
#include "common/common_defines.h"
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <vector>

// ----------------------------------------------------------------------------
// Test-only back-pressured app-socket model (the cl_tcp_socket::g_test_transmit_hook
// target). Accepts up to g_cap bytes per transmit() call; the rest "would-block"
// (modelling a full OS send buffer / a slow app reader). Every accepted byte is
// appended to g_received so the test can reconstruct and verify the stream.
// ----------------------------------------------------------------------------
static std::vector<char> g_received;
static int g_cap = 0;          // bytes the stub will accept this call (0 => always would-block)

static int rx_drain_test_hook(const char* buf, int length)
{
	if(g_cap <= 0)
		return -1;             // would-block: nothing accepted (EWOULDBLOCK)
	int take = (length < g_cap) ? length : g_cap;
	for(int i = 0; i < take; i++)
		g_received.push_back(buf[i]);
	return take;               // short write if take < length
}

// ----------------------------------------------------------------------------
// --test-rx-drain-backpressure entry. Drives the PRODUCTION responder drain
// (process_buffer_data_responder) over a small fifo_buffer_rx against the
// back-pressured stub, then asserts the full pattern delivers in order.
// ----------------------------------------------------------------------------
int cl_arq_controller::test_rx_drain_backpressure()
{
	printf("[TEST-RX-DRAIN] FIX-6 RX-delivery backpressure regression\n");
	fflush(stdout);

	// --- Setup: a responder controller with a small RX FIFO + accepted socket ---
	this->original_role     = RESPONDER;
	this->link_status       = CONNECTED;
	this->sack_v2_enabled   = false;            // legacy pop budget (matches bench3)
	this->compression_enabled = false;
	this->nMessages         = 255;
	this->max_data_length   = 170;
	this->max_message_length= 200;
	this->max_header_length = 6;
	int alloc_rc = init_messages_buffers();
	if(alloc_rc != SUCCESSFUL)
	{
		printf("[TEST-RX-DRAIN] ERROR: init_messages_buffers rc=%d\n", alloc_rc);
		fflush(stdout);
		return 1;
	}

	// Small RX FIFO so we exercise the FULL/wrap path fast. Capacity = FIFO_SZ-1
	// (get_free_size reserves one slot). B2F off (is_initialized()==false) so the
	// drain copies rx_raw verbatim — the plain-text bench3 path.
	const int FIFO_SZ = 4096;
	if(this->fifo_buffer_rx.set_size(FIFO_SZ) != SUCCESSFUL)
	{
		printf("[TEST-RX-DRAIN] ERROR: fifo_buffer_rx.set_size failed\n");
		fflush(stdout);
		return 1;
	}

	// Mark the data socket ACCEPTED and install the back-pressured stub. status is
	// public; the ctor already allocated tcp_socket_data.message.
	this->tcp_socket_data.status = TCP_STATUS_ACCEPTED;
	int (*saved_hook)(const char*, int) = cl_tcp_socket::g_test_transmit_hook;
	cl_tcp_socket::g_test_transmit_hook = rx_drain_test_hook;
	g_received.clear();

	// --- The pattern: N bytes, N > FIFO capacity so the FIFO fills and the drain
	// must run repeatedly. Deterministic, byte-unique-ish so a gap is detectable. ---
	const int N = 20000;        // ~5x the FIFO; forces many drain passes
	std::vector<char> pattern(N);
	for(int i = 0; i < N; i++)
		pattern[i] = (char)((i * 37 + (i >> 8) * 11 + 3) & 0xFF);

	// Feed the pattern into the RX FIFO in batch-sized chunks and drain after each,
	// exactly as the responder does (copy_data_to_buffer push -> drain tick). We
	// push via the production fifo_push_rx() helper (Mouth B) and drain via the
	// production process_buffer_data_responder() (Mouth A) — the real code paths.
	//
	// The drain pops up to ~172 bytes/iteration then send()s. We THROTTLE the stub
	// (g_cap) so most sends are SHORT WRITES and some are full WOULD-BLOCKS — the
	// exact bench3 condition. The drain must stash the unsent tail (rx_deliver_pending)
	// and resume IN ORDER next tick. Pre-fix it discards the tail => loss.
	const int CHUNK = 170;      // ~one DATA frame payload
	int fed = 0;
	int ck  = 0;                // chunk counter (rotates the throttle pattern)

	while(fed < N)
	{
		int n = (N - fed < CHUNK) ? (N - fed) : CHUNK;
		// Mouth B push into the RX FIFO.
		fifo_push_rx(pattern.data() + fed, n);
		fed += n;
		ck++;

		// Rotate the socket throttle so the drain hits the FULL range of
		// back-pressure responses: a full would-block (cap 0), a short write
		// (cap 40, < the ~172 pop), and a moderate cap. This forces repeated
		// stash/break/resume cycles — the path that LOST bytes pre-fix.
		int phase = ck % 3;
		if     (phase == 0) g_cap = 0;    // would-block: stash whole payload, break
		else if(phase == 1) g_cap = 40;   // short write: stash the tail, break
		else                g_cap = 200;  // accepts a full pop, but not a big backlog

		// Drain this chunk to completion-as-far-as-the-socket-allows, THEN ensure the
		// FIFO is emptied before feeding the next chunk so Mouth B never has to drop
		// (this test isolates Mouth A, the socket-drain non-lossiness). We do this by
		// temporarily fully unblocking and draining the FIFO down before the next feed
		// ONLY when it is nearly full — but we still leave a pending tail across feeds
		// to exercise the cross-tick resume.
		process_buffer_data_responder();

		// If the FIFO is filling past half, relieve it under the SAME throttle by
		// running extra drain ticks (each re-sends the stash in order, then pops more).
		int relief = 0;
		while(this->fifo_buffer_rx.get_free_size() < (FIFO_SZ / 2) && relief < 10000)
		{
			// keep the throttle non-zero so it makes progress
			if(g_cap == 0) g_cap = 40;
			process_buffer_data_responder();
			relief++;
		}
	}

	// FINAL PHASE — socket UNBLOCKS (reader caught up): raise the cap and drain until
	// the FIFO + any stashed pending tail are fully flushed. Post-fix every held byte
	// delivers here, IN ORDER, with zero loss.
	g_cap = 65536;              // accept everything now
	for(int t = 0; t < 100000; t++)
	{
		process_buffer_data_responder();
		bool fifo_empty = (this->fifo_buffer_rx.get_size() == this->fifo_buffer_rx.get_free_size());
		if(fifo_empty && this->rx_deliver_pending_len == 0)
			break;
	}

	// --- Restore the hook that was active before this test ---
	cl_tcp_socket::g_test_transmit_hook = saved_hook;

	// --- ASSERTIONS ---------------------------------------------------------
	int got = (int)g_received.size();
	bool len_ok = (got == N);

	// Byte-exact, in-order: g_received must equal pattern[0..N).
	bool order_ok = true;
	int  first_mismatch = -1;
	int  cmp = (got < N) ? got : N;
	for(int i = 0; i < cmp; i++)
	{
		if(g_received[i] != pattern[i])
		{
			order_ok = false;
			first_mismatch = i;
			break;
		}
	}

	printf("[TEST-RX-DRAIN] fed=%d delivered=%d (expect %d) len_ok=%d order_ok=%d first_mismatch=%d\n",
		fed, got, N, (int)len_ok, (int)order_ok, first_mismatch);
	fflush(stdout);

	if(len_ok && order_ok)
	{
		printf("[TEST-RX-DRAIN] PASS — full %d-byte stream delivered in order, zero loss\n", N);
		fflush(stdout);
		return 0;
	}

	if(!len_ok)
		printf("[TEST-RX-DRAIN] FAIL — delivered %d of %d bytes (LOST %d) — the 61,621-class silent drop\n",
			got, N, N - got);
	if(!order_ok)
		printf("[TEST-RX-DRAIN] FAIL — stream out of order / corrupted at byte %d (reorder or duplicate)\n",
			first_mismatch);
	fflush(stdout);
	return 1;
}
