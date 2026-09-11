/*
 * Mercury: A configurable open-source software-defined modem.
 *
 * Exact-AVX2 FIR lever gate (--test-fir-lever).
 *
 * Lever law under test: MERCURY_FIR_AVX2_EXACT unset (or empty) means the
 * byte-exact AVX2 FIR kernels are the DEFAULT on capable x86-64 hosts; `0`
 * disables them and restores the scalar authority; any other spelling keeps
 * them on. On hosts without AVX2 (and on non-x86 builds) the lever is inert
 * and the scalar path is always selected.
 *
 * The gate also re-proves exactness at the production entry points: with the
 * kernels active, cl_FIR::apply (real and complex) and cl_FIR::apply_decimate
 * must produce byte-identical output to the lever-off scalar run. That is the
 * contract that makes the default-ON flip a pure implementation selection.
 */

#include "physical_layer/fir_filter.h"
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#if defined(_WIN32)
  #include <stdlib.h>
  static void tfl_setenv(const char* k, const char* v)
  {
      if(v) _putenv_s(k, v); else _putenv_s(k, "");
  }
#else
  static void tfl_setenv(const char* k, const char* v)
  {
      if(v) setenv(k, v, 1); else unsetenv(k);
  }
#endif

namespace {

struct tfl_env_saved
{
	const char* key;
	bool present;
	std::string value;
	explicit tfl_env_saved(const char* k) : key(k), present(false)
	{
		const char* v = std::getenv(k);
		if(v != nullptr) { present = true; value = v; }
	}
	void restore() const { tfl_setenv(key, present ? value.c_str() : nullptr); }
};

bool host_has_avx2()
{
#if (defined(__x86_64__) || defined(_M_X64)) && \
	(defined(__GNUC__) || defined(__clang__))
	__builtin_cpu_init();
	return __builtin_cpu_supports("avx2") != 0;
#else
	return false;
#endif
}

struct tfl_rng
{
	std::uint64_t s;
	std::uint64_t next() { s^=s<<13; s^=s>>7; s^=s<<17; return s; }
};

// One lever-state expectation; prints the observed state, true on PASS.
bool check_state(const char* tag, const char* lever, bool want_active)
{
	tfl_setenv("MERCURY_FIR_AVX2_EXACT", lever);
	bool got = fir_exact_avx2_active();
	bool pass = (got == want_active);
	printf("[TEST-FIR-LEVER] %s lever=%s active=%d want=%d %s\n",
	       tag, lever ? (*lever ? lever : "(empty)") : "(unset)",
	       (int)got, (int)want_active, pass ? "PASS" : "FAIL");
	return pass;
}

// Byte-exactness at the production entry points for one tap count.
bool check_exact(double transition)
{
	cl_FIR fir;
	fir.sampling_frequency = 48000.0;
	fir.filter_transition_bandwidth = transition;
	fir.lpf_filter_cut_frequency = 1800.0;
	fir.type = LPF;
	fir.filter_window = HAMMING;
	fir.design();

	const int n = 4096;
	const int M = 8;
	std::vector<double> in((std::size_t)n), out_on((std::size_t)n), out_off((std::size_t)n);
	std::vector<std::complex<double>> cin((std::size_t)n), cout_on((std::size_t)n), cout_off((std::size_t)n);
	std::vector<std::complex<double>> dec_on((std::size_t)n/M), dec_off((std::size_t)n/M);
	tfl_rng rng{ UINT64_C(0x51ed270693dfa10b) ^ (std::uint64_t)fir.filter_nTaps };
	for(int i=0;i<n;i++)
	{
		double v = ((double)(rng.next()>>11) * (1.0/9007199254740992.0))*2.0 - 1.0;
		in[(std::size_t)i] = v;
		cin[(std::size_t)i] = { v, (i&1) ? -v : v*0.5 };
	}

	tfl_setenv("MERCURY_FIR_AVX2_EXACT", nullptr);   // default: kernels active
	fir.apply(in.data(), out_on.data(), n);
	fir.apply(cin.data(), cout_on.data(), n);
	fir.apply_decimate(cin.data(), dec_on.data(), n, M);

	tfl_setenv("MERCURY_FIR_AVX2_EXACT", "0");       // scalar authority
	fir.apply(in.data(), out_off.data(), n);
	fir.apply(cin.data(), cout_off.data(), n);
	fir.apply_decimate(cin.data(), dec_off.data(), n, M);

	bool real_eq = std::memcmp(out_on.data(), out_off.data(), (std::size_t)n*sizeof(double)) == 0;
	bool cplx_eq = std::memcmp(cout_on.data(), cout_off.data(), (std::size_t)n*sizeof(cout_on[0])) == 0;
	bool dec_eq  = std::memcmp(dec_on.data(), dec_off.data(), dec_on.size()*sizeof(dec_on[0])) == 0;
	bool pass = real_eq && cplx_eq && dec_eq;
	printf("[TEST-FIR-LEVER] exactness nTaps=%d real=%d complex=%d decimate=%d %s\n",
	       fir.filter_nTaps, (int)real_eq, (int)cplx_eq, (int)dec_eq,
	       pass ? "PASS" : "FAIL");
	return pass;
}

} // namespace

// Exposed to main.cc (declared `extern int test_fir_lever_run();` there).
// Returns 0 on success.
int test_fir_lever_run()
{
	printf("[TEST-FIR-LEVER] exact-AVX2 FIR lever law + production-entry exactness gate\n");
	fflush(stdout);

	tfl_env_saved saved("MERCURY_FIR_AVX2_EXACT");
	const bool cpu = host_has_avx2();
	printf("[TEST-FIR-LEVER] host avx2=%d (the expected active state when the lever is not 0)\n",
	       (int)cpu);

	bool all_pass = true;
	all_pass &= check_state("unset-default", nullptr, cpu);
	all_pass &= check_state("empty-default", "", cpu);
	all_pass &= check_state("disabled", "0", false);
	all_pass &= check_state("enabled", "1", cpu);

	// Exactness across the tap-count family the design() table produces.
	all_pass &= check_exact(6000.0);   // 17 taps
	all_pass &= check_exact(3000.0);   // 33 taps
	all_pass &= check_exact(1000.0);   // 97 taps

	saved.restore();
	printf("[TEST-FIR-LEVER] %s\n", all_pass ? "ALL PASS" : "FAILURES PRESENT");
	fflush(stdout);
	return all_pass ? 0 : 1;
}
