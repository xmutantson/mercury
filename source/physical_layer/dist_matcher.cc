/*
 * Mercury: A configurable open-source software-defined modem.
 *
 * cl_dist_matcher — CCDM distribution matcher (PAS / PCS). See
 * include/physical_layer/dist_matcher.h and
 * fact-documents/data-flow-pas-shaping.md for the construction + citations.
 */

#include "physical_layer/dist_matcher.h"
#include <cmath>
#include <algorithm>

// Exact multinomial of a remaining-count vector, in unsigned __int128.
// multinomial(counts) = (sum counts)! / prod(counts[i]!). Computed
// incrementally as a product of binomials to keep intermediates small:
//   C(n; k0,k1,..) = C(n, k0) * C(n-k0, k1) * ...
// Returns false on overflow (a count product would exceed 2^127); the caller
// rejects such a composition at configure() so shape/deshape never overflow.
typedef unsigned __int128 u128;

static bool mul_checked(u128 a, u128 b, u128* out)
{
	if(a == 0 || b == 0){ *out = 0; return true; }
	// overflow if a > UMAX/b
	u128 umax = (u128)-1;
	if(a > umax / b) return false;
	*out = a * b;
	return true;
}

// C(n, k) exact in u128, false on overflow. Uses the multiplicative formula
// with division at each step (always exact since the running product is a
// binomial coefficient times a falling factorial segment).
static bool binom_u128(int n, int k, u128* out)
{
	if(k < 0 || k > n){ *out = 0; return true; }
	if(k > n - k) k = n - k;   // symmetry, fewer iterations
	u128 r = 1;
	for(int i = 1; i <= k; ++i)
	{
		// r = r * (n - k + i) / i  — exact because r is always integral here.
		u128 num;
		if(!mul_checked(r, (u128)(unsigned)(n - k + i), &num)) return false;
		r = num / (u128)(unsigned)i;
	}
	*out = r;
	return true;
}

// multinomial over the remaining-count vector, exact u128, false on overflow.
static bool multinomial_u128(const int* counts, int n_levels, u128* out)
{
	int n = 0;
	for(int i = 0; i < n_levels; ++i) n += counts[i];
	u128 r = 1;
	int rem = n;
	for(int i = 0; i < n_levels; ++i)
	{
		u128 b;
		if(!binom_u128(rem, counts[i], &b)) return false;
		u128 prod;
		if(!mul_checked(r, b, &prod)) return false;
		r = prod;
		rem -= counts[i];
	}
	*out = r;
	return true;
}

cl_dist_matcher::cl_dist_matcher()
{
	n_levels = 0;
	blocklen = 0;
	for(int i = 0; i < DM_MAX_LEVELS; ++i) composition[i] = 0;
	k_bits = 0;
	ready = false;
}

cl_dist_matcher::~cl_dist_matcher()
{
}

bool cl_dist_matcher::configure(int _n_levels, const int* _composition, int block_len)
{
	ready = false;
	if(_n_levels <= 0 || _n_levels > DM_MAX_LEVELS || block_len <= 0) return false;
	int sum = 0;
	for(int i = 0; i < _n_levels; ++i)
	{
		if(_composition[i] < 0) return false;
		sum += _composition[i];
	}
	if(sum != block_len) return false;

	n_levels = _n_levels;
	blocklen = block_len;
	for(int i = 0; i < DM_MAX_LEVELS; ++i) composition[i] = (i < _n_levels) ? _composition[i] : 0;

	// Total number of constant-composition sequences = the multinomial. Must fit
	// u128 (mandatory for exact rank/unrank). The partial multinomials walked by
	// shape/deshape are always <= this total, so this single check is sufficient.
	u128 total;
	if(!multinomial_u128(composition, n_levels, &total)) return false;   // overflow -> reject
	if(total < 2) return false;   // need at least 1 info bit

	// k = floor(log2 total): largest k with 2^k <= total.
	int k = 0;
	u128 pow2 = 1;
	while(true)
	{
		u128 nxt;
		if(!mul_checked(pow2, (u128)2, &nxt)) break;   // pow2*2 would overflow -> stop
		if(nxt > total) break;
		pow2 = nxt;
		++k;
	}
	k_bits = k;
	ready  = (k_bits >= 1);
	return ready;
}

bool cl_dist_matcher::configure_maxwell_boltzmann(int _n_levels, const int* amplitudes,
                                                 double lambda, int block_len)
{
	if(_n_levels <= 0 || _n_levels > DM_MAX_LEVELS || block_len <= 0) return false;
	double w[DM_MAX_LEVELS], s = 0.0;
	for(int i = 0; i < _n_levels; ++i)
	{
		w[i] = std::exp(-lambda * (double)amplitudes[i] * (double)amplitudes[i]);
		s += w[i];
	}
	if(s <= 0.0) return false;

	// Largest-remainder rounding to an integer composition summing to block_len.
	int  base[DM_MAX_LEVELS];
	double frac[DM_MAX_LEVELS];
	int sum = 0;
	for(int i = 0; i < _n_levels; ++i)
	{
		double raw = (w[i] / s) * (double)block_len;
		base[i] = (int)std::floor(raw);
		frac[i] = raw - (double)base[i];
		sum += base[i];
	}
	int rem = block_len - sum;
	// distribute the remaining 'rem' to the largest fractional parts
	for(int r = 0; r < rem; ++r)
	{
		int best = -1; double bf = -1.0;
		for(int i = 0; i < _n_levels; ++i)
			if(frac[i] > bf){ bf = frac[i]; best = i; }
		if(best < 0) break;
		base[best] += 1;
		frac[best] = -1.0;   // don't pick twice
	}
	return configure(_n_levels, base, block_len);
}

int cl_dist_matcher::level_count(int lev) const
{
	if(lev < 0 || lev >= n_levels) return 0;
	return composition[lev];
}

// shape (unrank): map a k_bits-bit integer index to a constant-composition level
// sequence. Walk positions left->right; at each position pick the level whose
// remaining-multinomial bracket contains idx, then subtract and decrement.
bool cl_dist_matcher::shape(const int* in_bits, int* out_levels) const
{
	if(!ready || in_bits == nullptr || out_levels == nullptr) return false;

	// Assemble idx in [0, 2^k) from k_bits bits (MSB first — matched by deshape).
	u128 idx = 0;
	for(int b = 0; b < k_bits; ++b)
	{
		int bit = in_bits[b];
		if(bit != 0 && bit != 1) return false;
		idx = (idx << 1) | (u128)(unsigned)bit;
	}

	int rem_counts[DM_MAX_LEVELS];
	for(int i = 0; i < n_levels; ++i) rem_counts[i] = composition[i];

	for(int pos = 0; pos < blocklen; ++pos)
	{
		bool placed = false;
		for(int lev = 0; lev < n_levels; ++lev)
		{
			if(rem_counts[lev] == 0) continue;
			rem_counts[lev] -= 1;
			u128 cnt;
			if(!multinomial_u128(rem_counts, n_levels, &cnt)){ return false; }   // (cannot overflow: <= total)
			if(idx < cnt)
			{
				out_levels[pos] = lev;
				placed = true;
				break;
			}
			idx -= cnt;
			rem_counts[lev] += 1;   // restore, try next level
		}
		if(!placed) return false;   // idx out of range (only if input had >= 2^k value)
	}
	return true;
}

// deshape (rank): the exact inverse of shape — accumulate bracket offsets.
bool cl_dist_matcher::deshape(const int* in_levels, int* out_bits) const
{
	if(!ready || in_levels == nullptr || out_bits == nullptr) return false;

	int rem_counts[DM_MAX_LEVELS];
	for(int i = 0; i < n_levels; ++i) rem_counts[i] = composition[i];

	u128 idx = 0;
	for(int pos = 0; pos < blocklen; ++pos)
	{
		int chosen = in_levels[pos];
		if(chosen < 0 || chosen >= n_levels) return false;
		for(int lev = 0; lev < n_levels; ++lev)
		{
			if(rem_counts[lev] == 0) continue;
			if(lev == chosen)
			{
				rem_counts[lev] -= 1;
				break;
			}
			// skip past all sequences that would have placed 'lev' here
			rem_counts[lev] -= 1;
			u128 cnt;
			if(!multinomial_u128(rem_counts, n_levels, &cnt)) return false;
			idx += cnt;
			rem_counts[lev] += 1;
		}
	}

	// Emit k_bits bits MSB first (the inverse of shape's assembly).
	for(int b = 0; b < k_bits; ++b)
	{
		int shift = k_bits - 1 - b;
		out_bits[b] = (int)((idx >> shift) & (u128)1);
	}
	return true;
}

double cl_dist_matcher::logprior(int lev) const
{
	if(!ready || lev < 0 || lev >= n_levels) return 0.0;
	double p = (double)composition[lev] / (double)blocklen;
	if(p < 1e-12) p = 1e-12;
	return std::log(p);
}
