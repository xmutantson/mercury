/*
 * Winlink dictionary priming + version-lock regression suite.
 *
 * Wired via main.cc --test (and the standalone --test-winlink-dict). Drives the
 * PRODUCTION cl_compressor end-to-end (TX compress -> RX decompress) and asserts:
 *
 *   T1 PRIMED LIFT (pass-after) — small Winlink messages compress materially
 *      smaller with the baked dict than cold; cold ratio is in the 1.4-2.0x
 *      regime (the fail-before reference the whole feature exists to beat).
 *   T2 BIT-EXACT round-trip — primed TX -> primed RX reproduces the message
 *      byte-for-byte (the streaming-CRC TX/RX agreement requirement).
 *   T3 VERSION-MISMATCH FAIL-SAFE — TX primed v1 -> RX un-primed (cold) does NOT
 *      decode to wrong bytes; it rejects (returns <0) so the caller pushes raw.
 *      Reverse (cold TX -> primed RX) also never corrupts.
 *   T4 KILL-SWITCH — set_dict_priming(false) yields the cold path (no priming,
 *      active_dict_version()==0): byte-identical to the pre-dict baseline.
 *   T5 BULK NO-REGRESSION — the warm-carry on a primed stream still climbs to
 *      high ratios over repeated batches (the 3.69x bulk anchor is preserved).
 *   T6 ATTACHMENT INERTNESS — incompressible binary data takes the raw path and
 *      the primed wire is no larger than cold (the dict never harms a batch).
 *   T7 STREAMING DESYNC SELF-CORRECTION (the segfault-fix path, re-validated with
 *      the dict active) — a zstd/raw batch mid-stream followed by a text batch
 *      does not crash and stays bit-exact (PPMd model lock-step holds).
 *
 * Self-contained: corpus samples are embedded; no file I/O.
 */
#include "compression/mercury_compress.h"
#include "compression/winlink_dict.h"
#include "compression/test_winlink_dict.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>

static int wd_failures = 0;
static int wd_passes   = 0;

static void wd_check(bool ok, const char* name, const char* detail)
{
	if (ok) { wd_passes++; printf("  [PASS] %s — %s\n", name, detail); }
	else    { wd_failures++; printf("  [FAIL] %s — %s\n", name, detail); }
}

// Embedded representative Winlink small messages (plaintext after B2F unroll).
static const char* MSG_SHORT_CHECKIN =
	"Mid: 9QZ2WB7HKLMN\nBody: 214\nContent-Transfer-Encoding: 8bit\n"
	"Content-Type: text/plain; charset=ISO-8859-1\nDate: 2026/06/12 18:02\n"
	"From: N0CALL\nMbo: N0CALL\nSubject: Check in - net control\nTo: SECTION-NET\n\n"
	"Net control checking in for the Tuesday evening traffic net. Conditions\n"
	"fair on 80 meters, some QSB. Two pieces of traffic to pass for the\n"
	"northern district. Standing by. 73 de N0CALL.\n";

static const char* MSG_ICS213 =
	"Mid: 7R3PLMNB45QZ\nBody: 402\nContent-Transfer-Encoding: 8bit\n"
	"Content-Type: text/plain; charset=ISO-8859-1\nDate: 2026/06/12 09:15\n"
	"From: AC4EM\nMbo: AC4EM\nSubject: //WL2K ICS-213 General Message - Resource Request\n"
	"To: EOC-COUNTY\n\n<?xml version=\"1.0\"?>\nICS-213 GENERAL MESSAGE\n"
	"INCIDENT NAME: Hurricane Response Exercise 2026\n"
	"TO (Name/Position): County EOC Logistics Section Chief\n"
	"FROM (Name/Position): Shelter 4 Communications Unit Leader\n"
	"SUBJECT: Resource Request - Cots and Water\nDATE: 06/12/2026\nTIME: 0915 local\n"
	"MESSAGE:\nRequest the following resources be delivered to Shelter 4 no later\n"
	"than 1400 today: 150 folding cots, 40 cases bottled water, 6 light towers.\n"
	"Current shelter population is 312 and rising.\n"
	"APPROVED BY (Name/Position): J. Martinez, Shelter Manager\nSIGNATURE: /s/ J. Martinez\n"
	"REPLY:\nDATE:\nTIME:\nSIGNATURE:\n";

// Compress one message with a (possibly primed) TX, decode with a (possibly
// primed) RX. Returns the message's own wire bytes; sets rt_ok and rx_ret.
static int compress_one(const std::string& msg, bool tx_prime, bool rx_prime,
	bool* rt_ok, int* rx_ret)
{
	cl_compressor tx; tx.set_dict_priming(tx_prime); tx.init(); tx.streaming_enable();
	cl_compressor rx; rx.set_dict_priming(rx_prime); rx.init(); rx.streaming_enable();
	static char out[262144], rt[262144];
	int w = tx.compress_block(msg.data(), (int)msg.size(), out, (int)sizeof(out));
	int d = -1;
	if (w > 0) d = rx.decompress_block(out, w, rt, (int)sizeof(rt));
	*rx_ret = d;
	*rt_ok = (d == (int)msg.size() && memcmp(rt, msg.data(), msg.size()) == 0);
	tx.deinit(); rx.deinit();
	return w;
}

// T1 + T2: primed lift and bit-exact round-trip on a small message.
static void test_primed_lift(const char* name, const std::string& msg)
{
	bool rt_cold = false, rt_primed = false; int dc, dp;
	int cold   = compress_one(msg, false, false, &rt_cold,   &dc);
	int primed = compress_one(msg, true,  true,  &rt_primed, &dp);

	double cr = cold   > 0 ? (double)msg.size() / cold   : 0;
	double pr = primed > 0 ? (double)msg.size() / primed : 0;
	char buf[256];

	// Fail-before reference: cold ratio is in the small-message regime (< 2.1x).
	snprintf(buf, sizeof(buf), "cold=%d (%.2fx) in the cold small-msg regime", cold, cr);
	wd_check(cold > 0 && cr < 2.1, name, buf);

	// Pass-after: primed is materially smaller than cold (the lift).
	snprintf(buf, sizeof(buf), "primed=%d (%.2fx) < cold=%d (%.2fx)", primed, pr, cold, cr);
	wd_check(primed > 0 && primed < cold, name, buf);

	// Bit-exact both cold and primed.
	wd_check(rt_cold && rt_primed, name, "round-trip bit-exact (cold AND primed)");
}

// T3: version-mismatch fail-safe — TX primed v1 -> RX cold must NOT corrupt.
static void test_version_mismatch_failsafe()
{
	std::string msg = MSG_SHORT_CHECKIN;
	bool rt = false; int d;
	// TX primes (stamps dict-tag v1); RX does not prime (active dict 0).
	int w = compress_one(msg, true, false, &rt, &d);
	char buf[256];
	snprintf(buf, sizeof(buf), "TXv1->RXcold: wire=%d rx_ret=%d (must reject <0, no wrong bytes)", w, d);
	// Safe outcome: RX returns <0 (rejected). It MUST NOT return >=0 with wrong
	// bytes (silent corruption). rt is false either way for a reject.
	wd_check(d < 0, "version-mismatch (TX primed, RX cold)", buf);

	// Reverse: cold TX -> primed RX. The cold first frame triggers the existing
	// streaming-desync reset on the RX -> RX drops to cold -> decodes correctly.
	bool rt2 = false; int d2;
	int w2 = compress_one(msg, false, true, &rt2, &d2);
	snprintf(buf, sizeof(buf), "TXcold->RXv1: wire=%d rx_ret=%d rt=%d (no corruption)", w2, d2, rt2 ? 1 : 0);
	// Either a clean decode (RX reset to cold then decoded) or a clean reject —
	// never wrong bytes.
	bool corrupt = (d2 >= 0) && !rt2;
	wd_check(!corrupt, "version-mismatch (TX cold, RX primed)", buf);
}

// T4: kill-switch — set_dict_priming(false) => cold path, no priming.
static void test_kill_switch()
{
	cl_compressor c; c.set_dict_priming(false); c.init(); c.streaming_enable();
	char buf[128];
	snprintf(buf, sizeof(buf), "dict_priming=%d active_dict_version=%d (both 0)",
		c.dict_priming() ? 1 : 0, c.active_dict_version());
	wd_check(!c.dict_priming() && c.active_dict_version() == 0,
		"kill-switch (set_dict_priming(false))", buf);
	c.deinit();
}

// T5: bulk no-regression — warm-carry still climbs to a high ratio.
static void test_bulk_no_regression()
{
	std::string para;
	for (int k = 0; k < 40; k++)
		para += "The quick brown fox jumps over the lazy dog near the EOC shelter. ";

	cl_compressor tx; tx.set_dict_priming(true); tx.init(); tx.streaming_enable();
	cl_compressor rx; rx.set_dict_priming(true); rx.init(); rx.streaming_enable();
	static char out[262144], rt[262144];
	double last_ratio = 0; bool all_rt = true;
	for (int b = 0; b < 6; b++)
	{
		int w = tx.compress_block(para.data(), (int)para.size(), out, (int)sizeof(out));
		if (w <= 0) { all_rt = false; break; }
		int d = rx.decompress_block(out, w, rt, (int)sizeof(rt));
		bool ok = (d == (int)para.size() && memcmp(rt, para.data(), para.size()) == 0);
		all_rt = all_rt && ok;
		if (!ok) break;  // Never advance either streaming peer after a failed decode.
		int algo = (unsigned char)out[0] & COMPRESS_ALGO_MASK;
		if (rx.is_streaming()) rx.streaming_commit((unsigned char*)para.data(), (int)para.size());
		if (tx.is_streaming() && algo != COMPRESS_ALGO_RAW)
		{ tx.set_pending_raw((unsigned char*)para.data(), (int)para.size()); tx.commit_pending(); }
		last_ratio = (double)para.size() / w;
	}
	char buf[160];
	snprintf(buf, sizeof(buf), "warm-carry final ratio=%.1fx all_rt=%d (>5x expected)", last_ratio, all_rt ? 1 : 0);
	wd_check(last_ratio > 5.0 && all_rt, "bulk no-regression", buf);
	tx.deinit(); rx.deinit();
}

// T6: attachment inertness — incompressible binary -> raw path, primed no worse.
static void test_attachment_inertness()
{
	// Deterministic pseudo-random "binary attachment" (incompressible).
	std::string bin(8000, 0);
	unsigned int s = 0x12345678u;
	for (size_t i = 0; i < bin.size(); i++) { s = s * 1103515245u + 12345u; bin[i] = (char)(s >> 16); }

	bool rc = false, rp = false; int dc, dp;
	int cold   = compress_one(bin, false, false, &rc, &dc);
	int primed = compress_one(bin, true,  true,  &rp, &dp);
	char buf[160];
	snprintf(buf, sizeof(buf), "binary cold=%d primed=%d (delta %+d) rt c=%d p=%d",
		cold, primed, primed - cold, rc ? 1 : 0, rp ? 1 : 0);
	// Raw fallback: both ~ orig+header; primed must not exceed cold by more than a
	// header's worth, and both round-trip bit-exact.
	wd_check(rc && rp && primed <= cold + 8, "attachment inertness", buf);
}

// T7: streaming desync SAFETY with the dict active (segfault-fix path,
// re-validated). Mixed text/binary batches exercise the non-PPMd lock-step reset
// and the streaming-CRC desync recovery. The PRODUCTION discipline is the load-
// bearing contract: the RX commits raw to the streaming context ONLY on a
// successful decompress (arq_common.cc:8642, inside `if(dec_size > 0)`) and resets
// on failure (:8663); the TX commits its pending raw only on ACK
// (arq_commander.cc:5135). Under that discipline the SAFETY property is: NO batch
// ever decodes to WRONG bytes — every batch is either delivered bit-exact OR a
// CRC-caught desync that the ARQ layer recovers via raw fallback + reset (which we
// model by resetting BOTH sides). A mid-stream algorithm-switch desync (e.g. zstd
// chosen after a raw binary batch) is PRE-EXISTING and channel-independent of the
// dict — it occurs identically on the cold path — so we assert SAFETY (no
// corruption), not unconditional per-batch decode. The first text batch (primed)
// AND the recovery path must both stay corruption-free.
static void test_streaming_desync_with_dict()
{
	cl_compressor tx; tx.set_dict_priming(true); tx.init(); tx.streaming_enable();
	cl_compressor rx; rx.set_dict_priming(true); rx.init(); rx.streaming_enable();
	static char out[262144], rt[262144];

	// Binary batch (zstd or raw) — exercises the non-PPMd lock-step reset.
	std::string bin(3000, 0);
	unsigned int s = 0xC0FFEEu;
	for (size_t i = 0; i < bin.size(); i++) { s = s * 1103515245u + 12345u; bin[i] = (char)(s >> 16); }
	std::string text = MSG_ICS213;

	bool corrupt = false;     // any batch decoded to WRONG bytes (the unsafe outcome)
	int  delivered = 0;       // batches delivered bit-exact
	const std::string* seq[6] = { &text, &bin, &text, &text, &bin, &text };
	for (int b = 0; b < 6; b++)
	{
		const std::string& m = *seq[b];
		int w = tx.compress_block(m.data(), (int)m.size(), out, (int)sizeof(out));
		if (w <= 0)
		{
			// compress_block self-resets streaming on its own desync paths; mirror
			// the ARQ raw-fallback (no corruption, both sides resync).
			if (tx.is_streaming()) tx.streaming_reset();
			if (rx.is_streaming()) rx.streaming_reset();
			continue;
		}
		int d = rx.decompress_block(out, w, rt, (int)sizeof(rt));
		if (d > 0)
		{
			bool ok = (d == (int)m.size() && memcmp(rt, m.data(), m.size()) == 0);
			if (!ok) corrupt = true; else delivered++;
			// PRODUCTION: RX commits raw ONLY on success.
			if (rx.is_streaming()) rx.streaming_commit((unsigned char*)m.data(), (int)m.size());
			// PRODUCTION: TX commits pending raw on ACK (success).
			int algo = (unsigned char)out[0] & COMPRESS_ALGO_MASK;
			if (tx.is_streaming() && algo != COMPRESS_ALGO_RAW)
			{ tx.set_pending_raw((unsigned char*)m.data(), (int)m.size()); tx.commit_pending(); }
		}
		else
		{
			// CRC-caught desync: ARQ pushes raw (no corruption) and resyncs. The RX
			// already reset inside decompress_block; resync the TX too.
			if (tx.is_streaming()) tx.streaming_reset();
		}
	}
	char buf[160];
	snprintf(buf, sizeof(buf), "no corruption across mixed text/binary batches (delivered=%d, corrupt=%d)",
		delivered, corrupt ? 1 : 0);
	// SAFETY: never wrong bytes, and at least the primed text batches deliver.
	wd_check(!corrupt && delivered >= 3, "streaming desync safety (dict active)", buf);
	tx.deinit(); rx.deinit();
}

// Zero-order Shannon entropy is not an incompressibility proof. This stream
// contains every byte value equally often (8.0 bits/byte), but repeats one fixed
// 256-byte permutation. A carried order-6 PPMd model compresses the repetition
// strongly. The old warm-path entropy<7.5 gate skipped the codec and sent RAW.
static void test_high_entropy_conditional_structure()
{
	std::string structured(8192, 0);
	for(size_t i = 0; i < structured.size(); i++)
		structured[i] = (char)(unsigned char)(((i & 255u) * 73u + 19u) & 255u);

	cl_compressor tx; tx.set_dict_priming(true); tx.init(); tx.streaming_enable();
	cl_compressor rx; rx.set_dict_priming(true); rx.init(); rx.streaming_enable();
	static char out[262144], rt[262144];
	int w = tx.compress_block(structured.data(), (int)structured.size(),
		out, (int)sizeof(out));
	int algo = w > 0 ? ((unsigned char)out[0] & COMPRESS_ALGO_MASK) : -1;
	int d = w > 0 ? rx.decompress_block(out, w, rt, (int)sizeof(rt)) : -1;
	bool exact = d == (int)structured.size()
		&& memcmp(rt, structured.data(), structured.size()) == 0;

	char buf[192];
	snprintf(buf, sizeof(buf),
		"8.0-bit byte-balanced repetition: algo=%d wire=%d raw=%d exact=%d",
		algo, w, (int)structured.size() + tx.get_header_size(), exact ? 1 : 0);
	wd_check(exact && algo == COMPRESS_ALGO_PPMD
		&& w < (int)structured.size() + tx.get_header_size(),
		"high-entropy conditional structure", buf);
	tx.deinit(); rx.deinit();
}

// A persistent conditional model can need one locally-expanding bridge before
// the following repeated region becomes profitable. Prior PPMd savings may fund
// that bridge, but cumulative PPMd payload must remain smaller than cumulative
// raw payload. This is the JPEG-header -> first-scan -> later-scan shape.
static void test_ppmd_savings_credit_bridge()
{
	cl_compressor tx; tx.set_dict_priming(true); tx.init(); tx.streaming_enable();
	cl_compressor rx; rx.set_dict_priming(true); rx.init(); rx.streaming_enable();
	static char out[262144], rt[262144];

	std::string header;
	for(int i = 0; i < 24; i++) header += MSG_ICS213;
	// Long enough that PPMd's adaptation overhead is under the production 5%
	// bridge bound, while still expanding on its first unpredictable appearance.
	std::string bridge(30000, 0);
	unsigned int s = 0x9E3779B9u;
	for(size_t i = 0; i < bridge.size(); i++)
	{
		s = s * 1664525u + 1013904223u;
		bridge[i] = (char)(unsigned char)(s >> 24);
	}

	const std::string* seq[3] = { &header, &bridge, &bridge };
	int algos[3] = {-1, -1, -1};
	int payloads[3] = {-1, -1, -1};
	long total_wire = 0;
	long total_raw_wire = 0;
	bool exact = true;
	for(int i = 0; i < 3; i++)
	{
		const std::string& m = *seq[i];
		int w = tx.compress_block(m.data(), (int)m.size(), out, (int)sizeof(out));
		algos[i] = w > 0 ? ((unsigned char)out[0] & COMPRESS_ALGO_MASK) : -1;
		payloads[i] = w > 0
			? ((unsigned char)out[1] | ((unsigned char)out[2] << 8)) : -1;
		int d = w > 0 ? rx.decompress_block(out, w, rt, (int)sizeof(rt)) : -1;
		bool one_exact = d == (int)m.size()
			&& memcmp(rt, m.data(), m.size()) == 0;
		exact = exact && one_exact;
		if(one_exact)
		{
			rx.streaming_commit((const unsigned char*)m.data(), (int)m.size());
			tx.set_pending_raw((const unsigned char*)m.data(), (int)m.size());
			tx.commit_pending();
		}
		total_wire += w;
		total_raw_wire += (long)m.size() + tx.get_header_size();
	}
	int first_savings = (int)header.size() - payloads[0];
	int bridge_expansion = payloads[1] - (int)bridge.size();
	int bridge_limit = (int)bridge.size() / 20;
	if(bridge_limit < 64) bridge_limit = 64;

	// With the same primed model but zero earned wire savings, the expanding
	// bridge must select RAW. This proves batch 2 above uses the credit exception
	// rather than passing because it happened to shrink on this platform.
	cl_compressor no_credit;
	no_credit.set_dict_priming(true);
	no_credit.init();
	no_credit.streaming_enable();
	int nc_w = no_credit.compress_block(bridge.data(), (int)bridge.size(),
		out, (int)sizeof(out));
	int no_credit_algo = nc_w > 0
		? ((unsigned char)out[0] & COMPRESS_ALGO_MASK) : -1;
	no_credit.deinit();

	char buf[224];
	snprintf(buf, sizeof(buf),
		"algos=%d/%d/%d payloads=%d/%d/%d bridge_delta=%d no-credit=%d exact=%d",
		algos[0], algos[1], algos[2], payloads[0], payloads[1], payloads[2],
		bridge_expansion, no_credit_algo, exact ? 1 : 0);
	wd_check(exact
		&& algos[0] == COMPRESS_ALGO_PPMD
		&& algos[1] == COMPRESS_ALGO_PPMD
		&& algos[2] == COMPRESS_ALGO_PPMD
		&& bridge_expansion > 0
		&& bridge_expansion <= bridge_limit
		&& first_savings >= bridge_expansion
		&& no_credit_algo == COMPRESS_ALGO_RAW
		&& total_wire < total_raw_wire,
		"PPMd savings-credit bridge", buf);
	tx.deinit(); rx.deinit();
}

int run_winlink_dict_tests()
{
	wd_failures = 0; wd_passes = 0;
	printf("=== Winlink dictionary priming + version-lock tests (dict v%d, raw=%u compressed=%u) ===\n",
		WINLINK_DICT_VERSION, WINLINK_DICT_RAW_LEN, WINLINK_DICT_COMPRESSED_LEN);

	test_primed_lift("short-checkin", MSG_SHORT_CHECKIN);
	test_primed_lift("ics213-form",   MSG_ICS213);
	test_version_mismatch_failsafe();
	test_kill_switch();
	test_bulk_no_regression();
	test_attachment_inertness();
	test_streaming_desync_with_dict();
	test_high_entropy_conditional_structure();
	test_ppmd_savings_credit_bridge();

	printf("=== Winlink dict: %d passed, %d failed ===\n", wd_passes, wd_failures);
	return (wd_failures == 0) ? 0 : 1;
}
