/*
 * Mercury: in-process scalar-AWGN channel for the single-process 2-instance sim.
 * single-process-sim-refactor.md §10.6.
 *
 * Header-only, bit-reproducible port of sim_channel_relay.py's Xoshiro
 * (splitmix64 + Box-Muller with a stashed spare) + Channel AWGN math. Used ONLY
 * by the -m SIM_INPROC 2-instance stepper (one cl_sim_awgn per direction). NOT
 * on any production path. NO numpy, NO fade (scalar AWGN this increment; the full
 * Watterson port is the next increment per §6.1 fade cells).
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#ifndef INC_SIM_CHANNEL_H_
#define INC_SIM_CHANNEL_H_

#include <cstdint>
#include <cmath>
#include <cstddef>

// Portable Gaussian source: Box-Muller on a splitmix64 stream. Byte-for-byte
// equal to sim_channel_relay.py:Xoshiro (the misnomer is preserved from the
// relay; the actual generator is splitmix64). Seed map identical:
//   s = seed * 0x9E3779B97F4A7C15
// then each _u64() does the splitmix64 increment + finalizer mix. gauss()
// returns cos() first and stashes sin() as the spare (same call ordering as the
// relay so the per-sample noise stream matches exactly for a fixed seed).
class cl_sim_xoshiro
{
public:
	explicit cl_sim_xoshiro(uint64_t seed)
	{
		s_ = seed * 0x9E3779B97F4A7C15ULL;
		have_spare_ = false;
		spare_ = 0.0;
	}

	uint64_t u64()
	{
		s_ = s_ + 0x9E3779B97F4A7C15ULL;
		uint64_t z = s_;
		z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
		z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
		return z ^ (z >> 31);
	}

	double uniform()
	{
		return (double)(u64() >> 11) * (1.0 / 9007199254740992.0);
	}

	double gauss()
	{
		if (have_spare_)
		{
			have_spare_ = false;
			return spare_;
		}
		double u1 = uniform();
		if (u1 < 1e-15) u1 = 1e-15;
		double u2 = uniform();
		double mag = std::sqrt(-2.0 * std::log(u1));
		spare_ = mag * std::sin(2.0 * M_PI * u2);
		have_spare_ = true;
		return mag * std::cos(2.0 * M_PI * u2);
	}

private:
	uint64_t s_;
	bool     have_spare_;
	double   spare_;
};

// Scalar AWGN channel for ONE direction. Mirrors sim_channel_relay.py:Channel
// (AWGN-only path, fading=False): sticky TX-power tracking + the BER-harness
// noise stddev formula. process() adds noise IN PLACE to a passband block.
class cl_sim_awgn
{
public:
	// snr3k_db == channel SNR3k (the same axis the relay's --snr takes; the WGN
	// label maps via WGN_TO_SNR3K=2.4 elsewhere — the caller passes SNR3k here).
	// snr3k_db >= 900 → channel is transparent (no noise; clean cell).
	cl_sim_awgn(uint64_t seed, double snr3k_db)
		: rng_(seed)
	{
		clean_     = (snr3k_db >= 900.0);
		snr_lin_   = clean_ ? 0.0 : std::pow(10.0, snr3k_db / 10.0);
		peak_ms_   = 0.0;
		noise_std_ = 0.0;
	}

	// Add AWGN to a block of `n` passband doubles in place. Sticky power: only
	// chunks with real energy update P_sig (silent gaps don't drag the floor).
	void process(double* x, size_t n)
	{
		if (clean_ || n == 0) return;

		double ms = 0.0;
		for (size_t i = 0; i < n; i++) ms += x[i] * x[i];
		ms /= (double)n;
		if (ms > peak_ms_)
		{
			peak_ms_   = ms;
			noise_std_ = noise_std_from_psig(peak_ms_);
		}
		if (noise_std_ > 0.0)
			for (size_t i = 0; i < n; i++)
				x[i] += noise_std_ * rng_.gauss();
	}

private:
	// per-sample real-noise stddev matching the BER harness / relay:
	//   var = P_sig * F_NYQUIST / (SNR_lin * BW_noise)
	double noise_std_from_psig(double p_sig) const
	{
		const double F_NYQUIST = 24000.0;  // FS/2 (audioio.c f_nyquist)
		const double BW_NOISE  = 3000.0;   // SNR3k reference noise bandwidth
		double var = p_sig * F_NYQUIST / (snr_lin_ * BW_NOISE);
		if (var < 0.0) var = 0.0;
		return std::sqrt(var);
	}

	cl_sim_xoshiro rng_;
	bool   clean_;
	double snr_lin_;
	double peak_ms_;
	double noise_std_;
};

#endif // INC_SIM_CHANNEL_H_
