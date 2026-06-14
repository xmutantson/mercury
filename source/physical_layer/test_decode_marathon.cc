/*
 * Mercury: A configurable open-source software-defined modem.
 *
 * LEVER C (feat/decode-marathon) §3 INTEGRITY REGRESSION TEST.
 * fact-documents/decode-marathon-C.md §8.
 *
 * The lever decodes the held-CFG16 big block's Kcw independent LDPC codewords
 * across a thread pool instead of serially on one core. The integrity risk is
 * SILENT CROSS-FRAME CORRUPTION: cl_ldpc holds the only per-decode mutable HEAP
 * workspace (R/Q/V_pos), so two codewords decoded concurrently through ONE
 * cl_ldpc race that workspace and corrupt each other. This test is the
 * fail-before / pass-after gate.
 *
 * IT PROVES, on a synthetic multi-codeword CFG16 batch with KNOWN info bits:
 *   (1) PASS-AFTER  — the PARALLEL decode (private cl_ldpc per worker) produces
 *       out_infobits + cw_ok BYTE-IDENTICAL to the SERIAL decode (exact memcmp,
 *       not a hash), for both clean and uncorrectable codewords;
 *   (2) IN-ORDER / NO CROSS-FRAME CORRUPTION — every codeword's K info bits land
 *       at the correct disjoint slice [c*K..) and equal the known info bits for
 *       the CLEAN codewords, and the deliberately-uncorrectable codewords are the
 *       ONLY ones flagged cw_ok=0 (no clean codeword silently flipped);
 *   (3) FAIL-BEFORE — with MERCURY_DECODE_POOL_DEFEAT_SHARE=1 (all workers share
 *       ONE cl_ldpc workspace, the bug the lever guards against) the result
 *       DIVERGES from the serial reference (corrupted bits and/or dropped cw_ok),
 *       and clearing the defeat RECOVERS the byte-identical pass-after.
 *
 * In-process, no IONOS / RF. One-shot via main.cc --test-decode-marathon.
 */

#include "physical_layer/telecom_system.h"
#include "common/common_defines.h"   // CONFIG_16, YES/NO
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <string>

#if defined(_WIN32)
  #include <stdlib.h>
  static void tdm_setenv(const char* k, const char* v)
  {
      if(v) _putenv_s(k, v); else _putenv_s(k, "");
  }
#else
  static void tdm_setenv(const char* k, const char* v)
  {
      if(v) setenv(k, v, 1); else unsetenv(k);
  }
#endif

namespace {

// Deterministic xorshift PRNG (no libc RNG => identical across platforms/threads).
struct tdm_rng { unsigned long long s; unsigned long long next(){ s^=s<<13; s^=s>>7; s^=s<<17; return s; } };

// Build the per-codeword KNOWN info bits, encode each to N coded bits, and map to
// strong +-LLR (sign convention: bit==1 -> negative LLR, matching decode_SPA's
// LLRbin = (LLRi<0) at ldpc_decoder_SPA.cc:225). Inject `nflip` bit-flips per
// codeword (as LLR sign flips) so the decoder actually iterates; codewords with a
// SMALL flip count are correctable (cw_ok=1 vs the known info), codewords with a
// LARGE flip count are uncorrectable (cw_ok=0). Returns the flat LLR buffer.
void build_batch(cl_telecom_system* ts, int Kcw, int N, int K,
                 std::vector<float>& clr,
                 std::vector<std::vector<int>>& cw_info,
                 const std::vector<int>& flip_per_cw)
{
	clr.assign((size_t)Kcw * N, 0.0f);
	cw_info.assign((size_t)Kcw, std::vector<int>((size_t)K, 0));
	std::vector<int> info((size_t)K, 0);
	std::vector<int> enc((size_t)N, 0);
	const float STRONG = 20.0f;
	for(int c=0;c<Kcw;c++)
	{
		tdm_rng rng{ 0x9E3779B97F4A7C15ULL ^ (0x100000001B3ULL * (unsigned long long)(c+1)) };
		for(int i=0;i<K;i++){ int b=(int)(rng.next() & 1ULL); info[(size_t)i]=b; cw_info[(size_t)c][(size_t)i]=b; }
		ts->ldpc.encode(info.data(), enc.data());
		for(int i=0;i<N;i++)
		{
			// bit 0 -> +STRONG, bit 1 -> -STRONG (decode_SPA: LLRi<0 => hard 1)
			clr[(size_t)c*N + i] = (enc[(size_t)i]==0) ? STRONG : -STRONG;
		}
		// inject deterministic flips at spread-out positions
		int nflip = flip_per_cw[(size_t)c];
		tdm_rng frng{ 0xDEADBEEFCAFEF00DULL ^ (0x100000001B3ULL * (unsigned long long)(c*7+3)) };
		for(int f=0; f<nflip; f++)
		{
			int pos = (int)(frng.next() % (unsigned long long)N);
			clr[(size_t)c*N + pos] = -clr[(size_t)c*N + pos];   // flip the sign
		}
	}
}

bool buffers_equal(const std::vector<int>& a, const std::vector<int>& b)
{
	if(a.size()!=b.size()) return false;
	return std::memcmp(a.data(), b.data(), a.size()*sizeof(int))==0;
}

// One serial-vs-parallel comparison for a given batch. Returns true on PASS.
bool run_one(cl_telecom_system* ts, int Kcw, const char* tag,
             const std::vector<int>& flip_per_cw)
{
	const int N = ts->ldpc.N;
	const int K = ts->ldpc.K;

	std::vector<float> clr;
	std::vector<std::vector<int>> cw_info;
	build_batch(ts, Kcw, N, K, clr, cw_info, flip_per_cw);

	// ---- SERIAL reference (MERCURY_LDPC_MULTICORE unset) ----
	tdm_setenv("MERCURY_LDPC_MULTICORE", nullptr);
	tdm_setenv("MERCURY_DECODE_POOL_DEFEAT_SHARE", nullptr);
	std::vector<int> out_serial((size_t)Kcw*K, -1);
	std::vector<int> cwok_serial;
	int ok_serial = ts->bigblock_decode_codewords(clr.data(), Kcw, out_serial.data(), cwok_serial, &cw_info);

	// ---- PARALLEL (MERCURY_LDPC_MULTICORE=4) ----
	tdm_setenv("MERCURY_LDPC_MULTICORE", "4");
	tdm_setenv("MERCURY_DECODE_POOL_DEFEAT_SHARE", nullptr);
	std::vector<int> out_par((size_t)Kcw*K, -2);
	std::vector<int> cwok_par;
	int ok_par = ts->bigblock_decode_codewords(clr.data(), Kcw, out_par.data(), cwok_par, &cw_info);

	bool bits_match = buffers_equal(out_serial, out_par);
	bool cwok_match = buffers_equal(cwok_serial, cwok_par);
	bool count_match = (ok_serial == ok_par);

	// (2) in-order + no cross-frame corruption: every clean codeword (flip below the
	// correction capability) must decode to its KNOWN info and be cw_ok=1; the
	// deliberately-uncorrectable ones (large flip) are the ONLY cw_ok=0 slots, and
	// no clean codeword's slice is silently wrong.
	int clean_expected_ok = 0;
	bool inorder_ok = true;
	for(int c=0;c<Kcw;c++)
	{
		bool slice_correct = true;
		for(int i=0;i<K;i++)
			if(out_par[(size_t)c*K + i] != cw_info[(size_t)c][(size_t)i]) { slice_correct=false; break; }
		bool is_clean_target = (flip_per_cw[(size_t)c] <= 8);   // small-flip => correctable
		if(is_clean_target)
		{
			clean_expected_ok++;
			if(!slice_correct || cwok_par[(size_t)c]!=1) inorder_ok=false;  // clean MUST be faithful+ok
		}
	}

	printf("[TEST-DECODE-MARATHON] %s Kcw=%d: ok_serial=%d ok_par=%d bits_match=%d cwok_match=%d "
	       "count_match=%d inorder_ok=%d (clean_targets=%d)\n",
	       tag, Kcw, ok_serial, ok_par, (int)bits_match, (int)cwok_match,
	       (int)count_match, (int)inorder_ok, clean_expected_ok);
	fflush(stdout);

	bool pass = bits_match && cwok_match && count_match && inorder_ok;
	if(!pass) printf("[TEST-DECODE-MARATHON] %s: FAIL (parallel != serial OR clean corrupted)\n", tag);
	return pass;
}

// FAIL-BEFORE: with the defeat hook the parallel decode shares ONE cl_ldpc across
// workers => races R/Q/V_pos => MUST diverge from the serial reference. Returns
// true when the defeat is correctly DETECTED (i.e. it diverged).
bool run_failbefore(cl_telecom_system* ts, int Kcw, const std::vector<int>& flip_per_cw)
{
	const int N = ts->ldpc.N;
	const int K = ts->ldpc.K;
	std::vector<float> clr;
	std::vector<std::vector<int>> cw_info;
	build_batch(ts, Kcw, N, K, clr, cw_info, flip_per_cw);

	tdm_setenv("MERCURY_LDPC_MULTICORE", nullptr);
	tdm_setenv("MERCURY_DECODE_POOL_DEFEAT_SHARE", nullptr);
	std::vector<int> out_serial((size_t)Kcw*K, -1);
	std::vector<int> cwok_serial;
	ts->bigblock_decode_codewords(clr.data(), Kcw, out_serial.data(), cwok_serial, &cw_info);

	// Defeat ON: shared workspace. Run several trials — a data race is
	// nondeterministic, so retry to expose the corruption robustly.
	tdm_setenv("MERCURY_LDPC_MULTICORE", "4");
	tdm_setenv("MERCURY_DECODE_POOL_DEFEAT_SHARE", "1");
	bool diverged = false;
	int trials = 0;
	for(trials=0; trials<64 && !diverged; trials++)
	{
		std::vector<int> out_def((size_t)Kcw*K, -3);
		std::vector<int> cwok_def;
		ts->bigblock_decode_codewords(clr.data(), Kcw, out_def.data(), cwok_def, &cw_info);
		// ANY divergence (decoded bits OR cw_ok flags) == the shared-workspace race
		// corrupted at least one codeword relative to the serial reference.
		if(!buffers_equal(out_serial, out_def) || !buffers_equal(cwok_serial, cwok_def))
			diverged = true;
	}

	// restore clean state
	tdm_setenv("MERCURY_DECODE_POOL_DEFEAT_SHARE", nullptr);
	tdm_setenv("MERCURY_LDPC_MULTICORE", nullptr);

	printf("[TEST-DECODE-MARATHON] FAIL-BEFORE Kcw=%d: defeat-share diverged_from_serial=%d "
	       "(after %d trial(s))\n", Kcw, (int)diverged, trials);
	fflush(stdout);
	return diverged;
}

} // namespace

// Exposed to main.cc (declared `extern int test_decode_marathon_run();` there,
// C++ linkage). Returns 0 on success.
int test_decode_marathon_run()
{
	printf("[TEST-DECODE-MARATHON] LEVER C parallel==serial integrity gate (CFG16 big-block decode)\n");
	fflush(stdout);

	cl_telecom_system* ts = new cl_telecom_system();
	ts->current_configuration = -999;       // force load_configuration to run
	ts->load_configuration(CONFIG_16);
	const int N = ts->ldpc.N;
	const int K = ts->ldpc.K;
	printf("[TEST-DECODE-MARATHON] CFG16 grid loaded: N=%d K=%d P=%d\n", N, K, ts->ldpc.P);
	fflush(stdout);
	if(N <= 0 || K <= 0 || K >= N)
	{
		printf("[TEST-DECODE-MARATHON] FAIL: CFG16 grid not loaded (N=%d K=%d)\n", N, K);
		delete ts; return 1;
	}

	bool all_pass = true;

	// Batch 1 — nominal big-block K=8: mostly clean, two uncorrectable.
	{
		std::vector<int> flips(8, 4);     // 4 flips => correctable (cw_ok=1)
		flips[3] = 600;                   // uncorrectable => cw_ok=0
		flips[6] = 600;                   // uncorrectable => cw_ok=0
		all_pass &= run_one(ts, 8, "PASS-AFTER-K8", flips);
		all_pass &= run_failbefore(ts, 8, flips);
	}

	// Batch 2 — held-CFG16 K=25 (the actual marathon target): mix.
	{
		std::vector<int> flips(25, 3);    // correctable
		for(int c=0;c<25;c+=5) flips[(size_t)c] = 600;   // every 5th uncorrectable
		all_pass &= run_one(ts, 25, "PASS-AFTER-K25", flips);
		all_pass &= run_failbefore(ts, 25, flips);
	}

	// Batch 3 — all-clean K=30 (held-CFG16 upper bound): every codeword cw_ok=1.
	{
		std::vector<int> flips(30, 2);    // all correctable
		all_pass &= run_one(ts, 30, "PASS-AFTER-K30-allclean", flips);
	}

	delete ts;

	printf("[TEST-DECODE-MARATHON] %s\n", all_pass ? "ALL PASS" : "FAILURES PRESENT");
	fflush(stdout);
	return all_pass ? 0 : 1;
}
