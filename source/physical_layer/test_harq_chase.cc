/*
 * Mercury: A configurable open-source software-defined modem.
 *
 * HARQ CHASE COMBINING (feat/harq-chase-combining) — fail-before / pass-after gate.
 * fact-documents/data-flow-retx-queue.md §HARQ.
 *
 * Mercury retransmits FULL frames (identical coded bits), so a retransmit's channel
 * LLRs estimate the SAME a-posteriori bit reliabilities as a previously-FAILED
 * reception's LLRs. cl_telecom_system::harq_sum_and_decode() SUMS the two LLR
 * vectors before ldpc.decode — the ML-optimal equal-noise soft combine (Chase,
 * "Code Combining", IEEE Trans. Comm. 1985) — which doubles the effective SNR
 * (~3 dB) and recovers frames a single reception cannot decode.
 *
 * THIS TEST PROVES, on a synthetic AWGN-BPSK codeword with KNOWN info bits, driving
 * the EXACT production summation primitive (harq_sum_and_decode):
 *   (1) FAIL-BEFORE — at a chosen noise level a SINGLE reception's LLRs do NOT
 *       decode to the known info (residual bit errors survive LDPC);
 *   (2) PASS-AFTER  — SUMMING that same reception's LLRs with a second independent
 *       reception of the SAME codeword DECODES to the known info exactly;
 *   (3) WRONG-COMBINE SAFETY — summing reception-A's LLRs with a reception of a
 *       DIFFERENT codeword does NOT fabricate codeword A (the mis-pairing the
 *       production CRC gate rejects, harq_combine_rescue()).
 *
 * In-process, deterministic (xorshift + Box-Muller, no libc RNG), no IONOS / RF.
 * One-shot via main.cc --test-harq-chase.
 */

#include "physical_layer/telecom_system.h"
#include "common/common_defines.h"   // CONFIG_*, YES/NO
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>

namespace {

// Deterministic xorshift64 PRNG (identical across platforms).
struct hrng {
	unsigned long long s;
	unsigned long long next(){ s^=s<<13; s^=s>>7; s^=s<<17; return s; }
	double u01(){ return (double)((next()>>11) * (1.0/9007199254740992.0)); }
	// Box-Muller standard normal.
	double gauss(){
		double u1 = u01(); if(u1 < 1e-12) u1 = 1e-12;
		double u2 = u01();
		const double TWO_PI = 6.283185307179586476925286766559;
		return std::sqrt(-2.0*std::log(u1)) * std::cos(TWO_PI*u2);
	}
};

// Build a known info vector for codeword `seed`, encode it, and return the
// transmitted BPSK symbols x[i] = +1 (coded bit 0) / -1 (coded bit 1).
void make_codeword(cl_telecom_system* ts, int N, int K, unsigned long long seed,
                   std::vector<int>& info, std::vector<double>& x)
{
	info.assign((size_t)K, 0);
	std::vector<int> enc((size_t)N, 0);
	hrng rng{ 0x9E3779B97F4A7C15ULL ^ (0x100000001B3ULL * (seed+1)) };
	for(int i=0;i<K;i++) info[(size_t)i] = (int)(rng.next() & 1ULL);
	ts->ldpc.encode(info.data(), enc.data());
	x.assign((size_t)N, 0.0);
	for(int i=0;i<N;i++) x[(size_t)i] = (enc[(size_t)i]==0) ? +1.0 : -1.0;
}

// One AWGN reception of transmitted symbols x: y=x+n, n~N(0,sigma). Return the
// bit LLRs 2*y/sigma^2 (decode_SPA convention: LLR>0 => bit 0).
void receive(const std::vector<double>& x, double sigma, hrng& noise,
             std::vector<float>& llr)
{
	const double s2 = sigma*sigma;
	llr.assign(x.size(), 0.0f);
	for(size_t i=0;i<x.size();i++)
	{
		double y = x[i] + sigma*noise.gauss();
		llr[i] = (float)(2.0*y/s2);
	}
}

// True iff ldpc.decode(llr) reproduces `info` exactly over the K info bits.
bool decodes_to(cl_telecom_system* ts, const std::vector<float>& llr,
                const std::vector<int>& info, int K)
{
	std::vector<int> dec((size_t)ts->ldpc.N, 0);
	ts->ldpc.decode(llr.data(), dec.data());
	for(int i=0;i<K;i++) if(dec[(size_t)i] != info[(size_t)i]) return false;
	return true;
}

// True iff harq_sum_and_decode(a,b) reproduces `info` exactly.
bool combine_decodes_to(cl_telecom_system* ts,
                        const std::vector<float>& a, const std::vector<float>& b,
                        const std::vector<int>& info, int K)
{
	std::vector<int> dec((size_t)ts->ldpc.N, 0);
	ts->harq_sum_and_decode(a.data(), b.data(), ts->ldpc.N, dec.data());
	for(int i=0;i<K;i++) if(dec[(size_t)i] != info[(size_t)i]) return false;
	return true;
}

} // namespace

// Exposed to main.cc (extern "C++"). Returns 0 on success.
int test_harq_chase_run()
{
	printf("[TEST-HARQ-CHASE] RX-side soft-combining (Chase 1985) fail-before/pass-after gate\n");
	fflush(stdout);

	cl_telecom_system* ts = new cl_telecom_system();
	ts->current_configuration = -999;         // force load
	const int CFG = CONFIG_10;                 // moderate-rate OFDM config (SPA)
	ts->load_configuration(CFG);
	const int N = ts->ldpc.N;
	const int K = ts->ldpc.K;
	printf("[TEST-HARQ-CHASE] CFG%d grid: N=%d K=%d P=%d rate=%.3f\n",
	       CFG, N, K, ts->ldpc.P, N>0 ? (double)K/(double)N : 0.0);
	fflush(stdout);
	if(N <= 0 || K <= 0 || K >= N)
	{
		printf("[TEST-HARQ-CHASE] FAIL: config grid not loaded (N=%d K=%d)\n", N, K);
		delete ts; return 1;
	}

	// Sweep noise from clean to harsh over a batch of codewords. We look for the
	// CLIFF band where a SINGLE reception mostly FAILS but the two-reception COMBINE
	// mostly PASSES — the direct ~3 dB chase-combining signature. Deterministic
	// noise streams (fixed seeds) => reproducible verdict.
	const int NCW = 12;                        // codewords per noise level
	const double sigmas[] = { 0.70, 0.80, 0.90, 1.00, 1.10, 1.20, 1.30 };
	const int NS = (int)(sizeof(sigmas)/sizeof(sigmas[0]));

	bool cliff_found = false;
	double cliff_sigma = 0.0;
	int cliff_single_ok = 0, cliff_combine_ok = 0;

	for(int si=0; si<NS && !cliff_found; ++si)
	{
		double sigma = sigmas[si];
		int single_ok = 0, combine_ok = 0;
		for(int c=0;c<NCW;c++)
		{
			std::vector<int> info; std::vector<double> x;
			make_codeword(ts, N, K, (unsigned long long)(c*131+7), info, x);
			// two INDEPENDENT receptions of the SAME codeword (distinct noise seeds)
			hrng n1{ 0xABCDEF0123456789ULL ^ (0x100000001B3ULL*(unsigned long long)(c*2+1)) };
			hrng n2{ 0x0F0E0D0C0B0A0908ULL ^ (0x100000001B3ULL*(unsigned long long)(c*2+2)) };
			std::vector<float> llr1, llr2;
			receive(x, sigma, n1, llr1);
			receive(x, sigma, n2, llr2);
			if(decodes_to(ts, llr1, info, K))                  single_ok++;
			if(combine_decodes_to(ts, llr1, llr2, info, K))    combine_ok++;
		}
		printf("[TEST-HARQ-CHASE] sigma=%.2f  single_ok=%2d/%d  combine_ok=%2d/%d\n",
		       sigma, single_ok, NCW, combine_ok, NCW);
		fflush(stdout);
		// Cliff band: single reception is unreliable (majority fail) yet combining
		// recovers all of them.
		if(single_ok <= NCW/2 && combine_ok == NCW)
		{
			cliff_found  = true;
			cliff_sigma  = sigma;
			cliff_single_ok = single_ok;
			cliff_combine_ok = combine_ok;
		}
	}

	if(!cliff_found)
	{
		printf("[TEST-HARQ-CHASE] FAIL: no noise level exhibited single-FAIL + combine-PASS "
		       "(the ~3 dB combining gain did not manifest)\n");
		delete ts; return 1;
	}
	printf("[TEST-HARQ-CHASE] PASS-AFTER: at sigma=%.2f single_ok=%d/%d but combine_ok=%d/%d "
	       "(soft-combining recovered every failed frame)\n",
	       cliff_sigma, cliff_single_ok, NCW, cliff_combine_ok, NCW);
	fflush(stdout);

	// (3) WRONG-COMBINE SAFETY: at the cliff sigma, combining codeword A's LLRs with
	// a DIFFERENT codeword B's reception must NOT fabricate codeword A's info. (In
	// production, harq_combine_rescue additionally CRC-gates this — a mis-pairing is
	// discarded, never delivered.)
	{
		std::vector<int> infoA, infoB; std::vector<double> xA, xB;
		make_codeword(ts, N, K, 4242ULL, infoA, xA);
		make_codeword(ts, N, K, 9999ULL, infoB, xB);
		hrng na{ 0x1111111122222222ULL };
		hrng nb{ 0x3333333344444444ULL };
		std::vector<float> llrA, llrB;
		receive(xA, cliff_sigma, na, llrA);
		receive(xB, cliff_sigma, nb, llrB);
		bool false_fabricate = combine_decodes_to(ts, llrA, llrB, infoA, K);
		if(false_fabricate)
		{
			printf("[TEST-HARQ-CHASE] FAIL: combining MISMATCHED codewords fabricated codeword A "
			       "(wrong-combine safety violated)\n");
			delete ts; return 1;
		}
		printf("[TEST-HARQ-CHASE] WRONG-COMBINE SAFE: mismatched-codeword combine did NOT reproduce "
		       "codeword A (CRC gate rejects it in production)\n");
		fflush(stdout);
	}

	delete ts;
	printf("[TEST-HARQ-CHASE] ALL PASS\n");
	fflush(stdout);
	return 0;
}
