/*
 * measure_xvara_prod — real-traffic x-VARA via the PRODUCTION priming path.
 *
 * Unlike _research/measure_primed.cc (which hand-seeds via set_pending_raw), this
 * tool exercises the ACTUAL firmware path: cl_compressor::set_dict_priming(true)
 * + streaming_enable() (which calls prime_with_dict() internally). It compresses
 * each corpus message as the FIRST real batch on a freshly-primed stream, decodes
 * it on an identically-primed RX (bit-exact assert), and compares the Mercury wire
 * to the VARA LZHUF reference (.lzh sibling). The dict bytes are NEVER on the wire.
 *
 * Usage: measure_xvara_prod <corpus_dir>
 * Prints per-sample + aggregate edge (VARA_wire / Mercury_wire); BEAT iff > 1.63.
 */
#include "compression/mercury_compress.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

static std::string slurp(const std::string& p)
{
	FILE* f = fopen(p.c_str(), "rb"); if (!f) return std::string();
	fseek(f, 0, SEEK_END); long s = ftell(f); rewind(f);
	std::string m((size_t)s, 0); size_t r = fread(&m[0], 1, s, f); (void)r; fclose(f);
	return m;
}

// Compress msg as the first real batch of a primed-or-cold stream; decode on an
// identically-configured RX. Returns wire bytes; sets rt_ok.
static int mercury_wire(const std::string& msg, bool prime, bool* rt_ok)
{
	cl_compressor tx; tx.set_dict_priming(prime); tx.init(); tx.streaming_enable();
	cl_compressor rx; rx.set_dict_priming(prime); rx.init(); rx.streaming_enable();
	static char out[262144], rt[262144];
	int w = tx.compress_block(msg.data(), (int)msg.size(), out, (int)sizeof(out));
	int d = (w > 0) ? rx.decompress_block(out, w, rt, (int)sizeof(rt)) : -1;
	*rt_ok = (d == (int)msg.size() && memcmp(rt, msg.data(), msg.size()) == 0);
	tx.deinit(); rx.deinit();
	return w;
}

int main(int argc, char** argv)
{
	std::string dir = argc > 1 ? argv[1]
		: "x:/Storage/Documents/hermes and mercury/_research/b2f_unroll_harness/corpus";
	const char* names[] = { "short1", "ics213", "email1", "batch_3msg" };
	const double THRESH = 1.63;

	long tot_orig = 0, tot_cold = 0, tot_primed = 0, tot_vara = 0;
	printf("%-12s %5s %7s %9s %5s %9s %11s %s\n",
		"sample", "orig", "M_cold", "M_primed", "VARA", "edge_cold", "edge_primed", "beat");
	int fails = 0;
	for (int i = 0; i < 4; i++)
	{
		std::string plain = slurp(dir + "/" + names[i] + ".plain");
		std::string lzh   = slurp(dir + "/" + names[i] + ".lzh");
		if (plain.empty() || lzh.empty()) { printf("SKIP %s\n", names[i]); continue; }
		bool rc = false, rp = false;
		int cold   = mercury_wire(plain, false, &rc);
		int primed = mercury_wire(plain, true,  &rp);
		int vara   = (int)lzh.size();
		if (!rc || !rp) { printf("  ** round-trip FAIL %s (cold=%d primed=%d) **\n", names[i], rc, rp); fails++; }
		double ec = (double)vara / cold, ep = (double)vara / primed;
		tot_orig += plain.size(); tot_cold += cold; tot_primed += primed; tot_vara += vara;
		printf("%-12s %5zu %7d %9d %5d %9.3f %11.3f %s\n",
			names[i], plain.size(), cold, primed, vara, ec, ep, ep > THRESH ? "YES" : "no");
	}
	double agg_cold = (double)tot_vara / tot_cold;
	double agg_primed = (double)tot_vara / tot_primed;
	printf("----------------------------------------------------------------------\n");
	printf("AGG edge_cold=%.3f (beat=%s)  edge_primed=%.3f (beat=%s)\n",
		agg_cold, agg_cold > THRESH ? "YES" : "no",
		agg_primed, agg_primed > THRESH ? "YES" : "no");
	printf("ratios: M_primed=%.3fx M_cold=%.3fx VARA=%.3fx\n",
		(double)tot_orig / tot_primed, (double)tot_orig / tot_cold, (double)tot_orig / tot_vara);
	return fails == 0 ? 0 : 1;
}
