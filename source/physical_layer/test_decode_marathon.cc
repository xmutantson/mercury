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

bool buffers_equal(const std::vector<int>& a, const std::vector<int>& b);

struct tdm_env_saved
{
	const char* key;
	bool present;
	std::string value;
	explicit tdm_env_saved(const char* k) : key(k), present(false)
	{
		const char* v = std::getenv(k);
		if(v != nullptr) { present = true; value = v; }
	}
	void restore() const { tdm_setenv(key, present ? value.c_str() : nullptr); }
};

// Config-scoping boundary gate. This calls the production policy for the whole
// config domain under every documented lever state, then performs paired exact
// SPA-vs-scoped decodes on every robust config. The latter proves the policy is
// not merely a table claim: cl_ldpc::decode consumed SPA and produced identical
// output bytes and iteration counts for the same input.
bool run_decoder_policy_boundary(cl_telecom_system* ts)
{
	tdm_env_saved saved_minsum("MERCURY_LDPC_MINSUM");
	tdm_env_saved saved_fixed("MERCURY_LDPC_FIXEDPOINT");
	bool pass = true;
	int checks = 0;

	std::vector<int> configs;
	for(int cfg=CONFIG_0; cfg<=CONFIG_17; ++cfg) configs.push_back(cfg);
	configs.push_back(ROBUST_0);
	configs.push_back(ROBUST_1);
	configs.push_back(ROBUST_2);
	configs.push_back(ROBUST_3);
	configs.push_back(LOW48_ANCHOR_S20_R6);
	configs.push_back(CONFIG_NONE);
	configs.push_back(999);

	auto verify = [&](const char* tag, const char* lever, const char* fixed,
	                  bool global, bool scoped, ldpc_decoder_kind global_kind)
	{
		tdm_setenv("MERCURY_LDPC_MINSUM", lever);
		tdm_setenv("MERCURY_LDPC_FIXEDPOINT", fixed);
		for(int cfg : configs)
		{
			ldpc_decoder_kind want = LDPC_DECODER_SPA;
			if(global) want = global_kind;
			else if(scoped && cfg >= CONFIG_15 && cfg <= CONFIG_17)
				want = LDPC_DECODER_MINSUM_FIXED;
			ldpc_decoder_kind got = ldpc_decoder_policy_for_config(cfg);
			checks++;
			if(got != want)
			{
				printf("[TEST-LDPC-POLICY] FAIL %s cfg=%d got=%s want=%s\n",
				       tag, cfg, ldpc_decoder_kind_name(got),
				       ldpc_decoder_kind_name(want));
				pass = false;
			}
		}
	};

	// Lever law (default-ON): unset and empty both select the priced scoped
	// policy: cfg15/16/17 fixed-point min-sum, exact SPA everywhere else.
	verify("default-scoped", nullptr, nullptr, false, true, LDPC_DECODER_SPA);
	verify("empty-scoped", "", nullptr, false, true, LDPC_DECODER_SPA);
	verify("forced-SPA", "0", "1", false, false, LDPC_DECODER_SPA);
	verify("global-float", "1", nullptr, true, false, LDPC_DECODER_MINSUM);
	verify("global-fixed", "1", "1", true, false, LDPC_DECODER_MINSUM_FIXED);
	verify("scoped", "scoped", nullptr, false, true, LDPC_DECODER_SPA);
	verify("invalid-fail-closed", "yes", "1", false, false, LDPC_DECODER_SPA);

	// Paired byte proof for every robust/MFSK config through the real decode entry.
	const int robust_configs[] = { ROBUST_0, ROBUST_1, ROBUST_2, ROBUST_3 };
	for(int cfg : robust_configs)
	{
		ts->load_configuration(cfg);
		std::vector<int> info((size_t)ts->ldpc.K, 0);
		std::vector<int> encoded((size_t)ts->ldpc.N, 0);
		std::vector<float> llr((size_t)ts->ldpc.N, 0.0f);
		std::vector<int> spa((size_t)ts->ldpc.K, -1);
		std::vector<int> scoped_out((size_t)ts->ldpc.K, -2);
		for(int i=0; i<ts->ldpc.K; ++i) info[(size_t)i] = (i*17 + cfg) & 1;
		ts->ldpc.encode(info.data(), encoded.data());
		for(int i=0; i<ts->ldpc.N; ++i)
			llr[(size_t)i] = encoded[(size_t)i] ? -20.0f : 20.0f;

		tdm_setenv("MERCURY_LDPC_MINSUM", "0");
		int spa_iter = ts->ldpc.decode(llr.data(), spa.data());
		tdm_setenv("MERCURY_LDPC_MINSUM", "scoped");
		int scoped_iter = ts->ldpc.decode(llr.data(), scoped_out.data());
		bool bytes_equal = buffers_equal(spa, scoped_out);
		bool actual_spa = ts->ldpc.last_decoder_kind == LDPC_DECODER_SPA;
		bool this_pass = bytes_equal && actual_spa && spa_iter == scoped_iter;
		printf("[TEST-LDPC-POLICY] cfg=%d robust path=%s bytes_equal=%d "
		       "iter_equal=%d (%d/%d)\n", cfg,
		       ldpc_decoder_kind_name(ts->ldpc.last_decoder_kind),
		       (int)bytes_equal, (int)(spa_iter == scoped_iter), spa_iter, scoped_iter);
		pass &= this_pass;
	}

	// NB has no cfg15/16/17 aliases: the production loader clamps a cfg16
	// request to cfg14 before publishing ldpc.configuration. Exercise that exact
	// ordering and require the resulting decoder decision to be SPA.
	ts->narrowband_enabled = YES;
	ts->current_configuration = CONFIG_NONE;
	tdm_setenv("MERCURY_LDPC_MINSUM", "scoped");
	ts->load_configuration(CONFIG_16);
	ldpc_decoder_kind nb_kind = ldpc_decoder_policy_for_config(ts->ldpc.configuration);
	bool nb_pass = ts->current_configuration == CONFIG_14
		&& ts->ldpc.configuration == CONFIG_14
		&& nb_kind == LDPC_DECODER_SPA;
	printf("[TEST-LDPC-POLICY] NB cfg16 request -> active=%d ldpc.cfg=%d path=%s pass=%d\n",
	       ts->current_configuration, ts->ldpc.configuration,
	       ldpc_decoder_kind_name(nb_kind), (int)nb_pass);
	pass &= nb_pass;

	// Default-ON fire witness through the production decode entry: with the
	// lever UNSET a real cfg16 decode must select the scoped fixed-point
	// min-sum kernel and still return the exact info bits; with `0` the same
	// decode must run exact SPA. This pins the unset default itself, not just
	// the mapping table above.
	ts->narrowband_enabled = NO;
	ts->current_configuration = CONFIG_NONE;
	ts->load_configuration(CONFIG_16);
	{
		std::vector<int> info((size_t)ts->ldpc.K, 0);
		std::vector<int> encoded((size_t)ts->ldpc.N, 0);
		std::vector<float> llr((size_t)ts->ldpc.N, 0.0f);
		std::vector<int> out_default((size_t)ts->ldpc.K, -1);
		std::vector<int> out_off((size_t)ts->ldpc.K, -2);
		for(int i=0; i<ts->ldpc.K; ++i) info[(size_t)i] = (i*29 + 7) & 1;
		ts->ldpc.encode(info.data(), encoded.data());
		for(int i=0; i<ts->ldpc.N; ++i)
			llr[(size_t)i] = encoded[(size_t)i] ? -20.0f : 20.0f;

		tdm_setenv("MERCURY_LDPC_MINSUM", nullptr);
		ts->ldpc.decode(llr.data(), out_default.data());
		ldpc_decoder_kind kind_default = ts->ldpc.last_decoder_kind;
		bool default_faithful = buffers_equal(info, out_default);

		tdm_setenv("MERCURY_LDPC_MINSUM", "0");
		ts->ldpc.decode(llr.data(), out_off.data());
		ldpc_decoder_kind kind_off = ts->ldpc.last_decoder_kind;
		bool off_faithful = buffers_equal(info, out_off);

		bool fire_pass = kind_default == LDPC_DECODER_MINSUM_FIXED
			&& default_faithful
			&& kind_off == LDPC_DECODER_SPA
			&& off_faithful;
		printf("[TEST-LDPC-POLICY] cfg16 unset-default path=%s faithful=%d; "
		       "lever=0 path=%s faithful=%d; fire=%s\n",
		       ldpc_decoder_kind_name(kind_default), (int)default_faithful,
		       ldpc_decoder_kind_name(kind_off), (int)off_faithful,
		       fire_pass ? "PASS" : "FAIL");
		pass &= fire_pass;
	}

	// Restore the caller's A/B arm and the marathon's cfg16 geometry.
	saved_minsum.restore();
	saved_fixed.restore();
	ts->narrowband_enabled = NO;
	ts->current_configuration = CONFIG_NONE;
	ts->load_configuration(CONFIG_16);

	printf("[TEST-LDPC-POLICY] %s: %d mapping checks; cfg14/NB boundary=%s; "
	       "cfg15/16/17=%s under scoped\n",
	       pass ? "ALL PASS" : "FAILURES PRESENT", checks,
	       ldpc_decoder_kind_name(LDPC_DECODER_SPA),
	       ldpc_decoder_kind_name(LDPC_DECODER_MINSUM_FIXED));
	fflush(stdout);
	return pass;
}

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
	all_pass &= run_decoder_policy_boundary(ts);

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
