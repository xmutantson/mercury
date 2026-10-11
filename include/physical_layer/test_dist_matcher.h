/*
 * Mercury: A configurable open-source software-defined modem.
 *
 * cl_dist_matcher — CCDM distribution matcher for Probabilistic Amplitude
 * Shaping (PAS / PCS). Lever #2, branch feat/pcs.
 *
 * Prior art (see bigblock_p3_hw/_deepdsp/RESEARCH_pcs.md + fact-documents/
 * data-flow-pas-shaping.md):
 *   - Böcherer, Schulte, Steiner, "Bandwidth Efficient and Rate-Matched LDPC
 *     Coded Modulation," IEEE TCOM 63(12), 2015 (arXiv:1502.02733) — PAS.
 *   - Schulte, Böcherer, "Constant Composition Distribution Matching," IEEE
 *     T-IT 62(1), 2016 — CCDM via arithmetic coding (this module).
 *   - Gültekin et al., "Enumerative Sphere Shaping ...," IEEE TWC 2020 — the
 *     ESS upgrade noted as the follow-on (more gain at short blocklength).
 *
 * This is a fixed-to-fixed constant-composition matcher: a bijection between
 * the first floor(log2 multinomial) input bits and the level-sequences of one
 * fixed composition, realized by an EXACT-INTEGER arithmetic coder over the
 * multinomial. Exact integer arithmetic (unsigned __int128) is mandatory: a
 * precision-drift bug in shape/deshape silently corrupts the de-shaped payload
 * = the worst-class life-critical data-integrity failure (RESEARCH_pcs.md §8).
 * A deshape(shape(x))==x bijection test (tools/test_pas_shaping.py + the C++
 * --test-pas hook) guards it.
 *
 * RPi CPU budget: shape/deshape are O(blocklen * n_levels) 128-bit add/compare
 * ops over a small blocklen (<=34 levels/rail) — negligible vs the LDPC SPA
 * decode (the dominant per-frame cost). No per-call heap (fixed workspace).
 */

#ifndef INC_DIST_MATCHER_H_
#define INC_DIST_MATCHER_H_

#define DM_MAX_LEVELS 8     // supports up to 8 amplitude levels (4 used for 4-ASK)

class cl_dist_matcher
{
private:
	int  n_levels;                 // number of amplitude levels (4 for 4-ASK)
	int  blocklen;                 // levels per DM block (rail blocklength Lr)
	int  composition[DM_MAX_LEVELS]; // target counts, sum == blocklen
	int  k_bits;                   // info bits in/out per block = floor(log2 multinomial)
	bool ready;                    // configured AND counts fit __int128 exactly

public:
	cl_dist_matcher();
	~cl_dist_matcher();

	// Configure the matcher for n_levels amplitude levels with the given integer
	// composition (counts, summing to block_len). Computes k_bits and validates
	// the multinomial fits 128-bit. Returns true on success (ready==true).
	bool configure(int _n_levels, const int* _composition, int block_len);

	// Build a Maxwell-Boltzmann composition P(level) ~ exp(-lambda * amp(level)^2)
	// for the given amplitudes[] and block_len (largest-remainder rounding), then
	// configure(). Returns true on success.
	bool configure_maxwell_boltzmann(int _n_levels, const int* amplitudes,
	                                 double lambda, int block_len);

	int  info_bits()  const { return ready ? k_bits : 0; }   // k per block
	int  block_len()  const { return blocklen; }
	bool is_ready()   const { return ready; }
	int  level_count(int lev) const;                          // composition[lev]

	// shape:   in_bits[0..k_bits)  -> out_levels[0..blocklen)  (level indices 0..n_levels-1)
	// deshape: in_levels[0..blocklen) -> out_bits[0..k_bits)
	// Both return false on any out-of-range input / not-ready (defensive). The
	// bijection deshape(shape(x))==x holds for every valid k_bits-bit input.
	bool shape  (const int* in_bits,   int* out_levels) const;
	bool deshape(const int* in_levels, int* out_bits)   const;

	// ln P(level) under the configured composition (= ln(count/blocklen)). Used
	// by the PAS LLR de-shaper (cl_psk::demod_pas) to build the amplitude-bit
	// log-prior. Returns 0 if not ready / out of range.
	double logprior(int lev) const;
};

#endif
