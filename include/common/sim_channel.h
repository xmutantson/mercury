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

// ---------------------------------------------------------------------------
// Band-limited Gaussian PHASE-NOISE process (the testbed PN ceiling).
//
// Phase-1 (HW-validated): the RPi/Fe-Pi/SGTL5000 testbed phase noise is a
// STATIONARY band-limited Gaussian phase process — NOT a free-running Wiener
// walk. Proven by 415 clean HW decodes showing BOUNDED post-EQ EVM variance
// (sd 0.09 dB) rather than the ever-growing variance a Wiener walk would
// produce. Baseband model r[n]=s[n]·exp(j·θ[n])+w[n], θ zero-mean stationary
// Gaussian, PSD band-limited.
//
// HW signature this model must reproduce (Phase-3 faithfulness gate):
//   • post-EQ EVM ceiling 14.5 dB on CFG15 (16QAM) AND CFG16 (32QAM)
//   • meanH droop 0.979  (the CPE amplitude-attenuation signature exp(-σ²/2))
//   • frames mostly DECODE (HW had 415 clean decodes incl. 32QAM CFG16)
//
// PHASE-3 CALIBRATED OUTCOME (this worktree, pinned clean snr3k≥900 cell):
//   PN_DEG=13.8° lands CFG15 (decode-reliable, 0 fails) at EVM 14.48 dB ≈ HW
//   14.5, meanH 0.989 (within the gate's ±0.01 of 0.979). CFG16 (32QAM) at the
//   same σ decodes only its clean tail (~20 %), so its survivor EVM reads ~15.5
//   (biased high) — there is NO single σ at which BOTH configs read 14.5 with
//   full decode, because 32QAM is ~1 dB more fragile to the per-symbol common
//   phase than 16QAM (HW's CFG16 decodes ~fully; the sim's does not). meanH
//   cannot be driven below ~0.989 without a large random ICI that craters the
//   decode (it is an AGC-normalized magnitude-mean — see the droop note in
//   rotate()). The EVM-ceiling mechanism is faithful; the CFG16 strict gate +
//   the last ~0.01 of meanH are honest residual limitations of a passband
//   phase-rotation model against this AGC+linear-CPE receiver.
//
// CPE / ICI DECOMPOSITION (the key to faithfulness — discovered Phase-3, with
// the model's mechanism cited inline):
//   Mercury's pilot CPE corrector (ofdm.cc:CPE_correction) estimates ONE linear
//   phase RATE across the whole frame and removes a ramp. So the phase error
//   the DECODER actually sees splits into:
//     (a) COMMON phase per OFDM symbol  φ_c[m]  — a constant rotation of all the
//         subcarriers in symbol m. This is benign for QAM (a pure rotation the
//         soft-demapper/LDPC tolerate) and is what lets 32QAM survive a 14.5 dB
//         EVM. The corrector removes only its linear-across-the-frame trend, so
//         the per-symbol residual (RMS σ_cpe) is what sets the measure_SNR
//         ceiling: EVM_dB = -20·log10(σ_cpe).
//     (b) ICI per sample  j[n]  — fast intra-symbol phase variation that
//         scatters energy off each subcarrier. This is what produces the |H|
//         amplitude droop meanH = exp(-σ_ici²/2) (Petrovic 2007 / Wu & Bar-Ness
//         2002: CPE attenuates the wanted bin by exp(-σ_ici²/2), the lost power
//         leaks to neighbours as ICI). A SMALL σ_ici both gives the 0.979 droop
//         and stays decoder-tolerant (too much ICI craters 32QAM — verified).
//   A SINGLE free-running band-limited θ[n] at f_3dB ~2.5 Hz canNOT reproduce
//   the HW signature: over a 77 ms (12-symbol) frame a 2.5 Hz drift is ~linear,
//   so CPE_correction removes essentially ALL of it, leaving a bursty non-linear
//   residual that is survivor-biased (some frames clean, most fail) — exactly
//   the failure observed. The faithful model therefore generates φ_c and j
//   SEPARATELY (this is the standard PLL-vs-free-running PN split, Petrovic §II):
//     • φ_c[m]: AR(1) at the SYMBOL rate (one draw per OFDM-symbol block), RMS
//       σ_cpe. Symbol-rate correlation ρ_c = exp(-2π·f_3dB/f_sym) where f_sym is
//       the OFDM symbol rate. At f_3dB ~2.5 Hz and f_sym ~155 Hz the symbols are
//       lightly correlated, so the frame-linear corrector removes only the trend
//       and the σ_cpe residual SURVIVES on EVERY frame ⇒ a STABLE 14.5 ceiling.
//     • j[n]: per-SAMPLE white-ish jitter, RMS σ_ici (≪ σ_cpe), giving the small
//       ICI ⇒ the 0.979 droop.
//   σ_cpe is the DOMINANT knob (sets EVM); σ_ici = ici_frac·σ_cpe is tuned once
//   so the droop lands at 0.979. Total RMS σ_φ = sqrt(σ_cpe² + σ_ici²).
//
// Application to a REAL 48 kHz PASSBAND block (carrier already baked in by
// baseband_to_passband ⇒ samples are real): we cannot naive-multiply a real
// array by exp(j·θ). Form the ANALYTIC signal a[n]=x[n]+j·Hilbert{x}[n] with a
// STATEFUL FIR Hilbert transformer (history carried across blocks, no per-block
// edge discontinuity), rotate it y[n]=Re{a[n]·exp(j·(φ_c+j[n]))} =
// x_d·cosθ − x_h·sinθ, x_d the group-delay-matched real path. AWGN is added on
// top (PN is the permanent ceiling — applied even on the clean snr3k≥900 cell).
//
// FIR Hilbert: odd-length Type-III, ideal h[k]=2/(πk) for odd k (0 even),
// Hamming-windowed (Oppenheim & Schafer §7; same construction as liquid-dsp's
// firhilbf). Real path delayed by group delay D=(L-1)/2 so x_d and x_h align.
//
// process() is called once per OFDM-symbol block (arq_commander.cc:9704
// sp = Nofdm·interpolation_rate = one symbol), so "one φ_c draw per rotate()"
// == one common phase per OFDM symbol exactly.
class cl_sim_phase_noise
{
public:
	// seed       : distinct from the AWGN seed (GATE-2 — never drawn from the
	//              AWGN rng_). sigma_phi in RADIANS = TOTAL RMS phase.
	// f3db_hz    : symbol-rate correlation bandwidth of the common phase.
	// fs_hz      : passband sample rate (48000).
	// ici_frac   : σ_ici / σ_cpe split. σ_cpe = σ_phi/sqrt(1+ici_frac²) carries
	//              the EVM; σ_ici = ici_frac·σ_cpe carries the droop.
	// The symbol rate f_sym = fs/n is derived LAZILY from the first block size n
	// (one rotate() call == one OFDM symbol), so the common-phase AR(1) pole is
	// set correctly for whatever config the pin selects without plumbing Nofdm
	// through the channel ctor.
	cl_sim_phase_noise(uint64_t seed, double sigma_phi_rad, double f3db_hz,
	                   double fs_hz, double ici_frac, double droop_sigma_rad,
	                   double resid_sigma_rad = 0.0, double slow_f3db_hz = 0.2)
		: rng_(seed), cpe_state_(0.0), ici_state_(0.0)
	{
		enabled_ = (sigma_phi_rad > 0.0);
		f3db_hz_ = f3db_hz;
		fs_hz_   = fs_hz;
		// Split total RMS into common (EVM) + ICI (droop) parts.
		double denom = std::sqrt(1.0 + ici_frac * ici_frac);
		sigma_cpe_ = sigma_phi_rad / denom;
		sigma_ici_ = ici_frac * sigma_cpe_;
		cpe_init_  = false;     // rho_cpe_/cpe_gain_/rho_ici_/ici_gain_ on 1st rotate()
		rho_cpe_   = 0.0;
		cpe_gain_  = 0.0;
		// ICI lowpass pole at a fixed INTRA-symbol cutoff (per-sample, fs known
		// now). The cutoff is set comparable to the subcarrier spacing band so the
		// jitter varies WITHIN the OFDM FFT window (→ a genuine |H| droop) instead
		// of averaging to ~0 the way per-sample WHITE jitter does. f_ici drives the
		// droop's spectral character; ici_frac drives its amplitude.
		rho_ici_   = std::exp(-2.0 * M_PI * F_ICI_HZ / fs_hz_);
		double ssi = 1.0 / std::sqrt(1.0 - rho_ici_ * rho_ici_);
		ici_gain_  = (ssi > 0.0) ? (sigma_ici_ / ssi) : 0.0;
		// Coherent ICI amplitude droop (the meanH signature): droop = exp(-σ_droop²/2)
		// (Petrovic 2007 §II — the wanted subcarrier's coherent attenuation). This
		// is DECOUPLED from the random ICI scatter above: in this receiver meanH =
		// mean(|H_pilot|) is a magnitude-mean (phase-rotation-insensitive), and the
		// only phase mechanism that droops it — large random ICI — craters 32QAM
		// decode (verified). The faithful, decode-tolerant carrier of the 0.979
		// droop is therefore this uniform coherent amplitude loss (HW's droop is a
		// per-subcarrier power loss, not random scatter). σ_droop is its own knob so
		// it sets meanH WITHOUT injecting decode-breaking scatter.
		droop_     = std::exp(-0.5 * droop_sigma_rad * droop_sigma_rad);
		// --- EXPERIMENTAL 2-component common-phase model (DIAGNOSIS VALIDATION,
		// fact-documents/sim2-cfg16-phase-noise-faithfulness.md §5). DEFAULT OFF:
		// when resid_sigma_rad==0 the path below is bypassed and the channel is
		// byte-identical to fa033eb. When MERCURY_SIM2_PN_RESID_DEG>0, the per-symbol
		// common phase becomes φ_c[m] = a·slow_AR1(ρ_slow) + N(0,σ_resid):
		//   • slow_AR1 at a sub-Hz pole (MERCURY_SIM2_PN_SLOW_F3DB, default 0.2 Hz) is
		//     ~purely LINEAR over a 9-12 symbol frame ⇒ FULLY absorbed by the linear
		//     CPE_correction (ofdm.cc:1928) ⇒ contributes ~0 to EVM. Its amplitude
		//     sigma_cpe_ is reused (the "drift" the corrector eats).
		//   • σ_resid = i.i.d. zero-mean per-symbol jitter ⇒ survives the ramp-only
		//     corrector ~undiminished on EVERY frame (sets the EVM ceiling) AND its
		//     per-frame |·|² mean over Nsymb is TIGHT (sd ~1/√Nsymb → HW 0.09), and it
		//     is a pure per-symbol ROTATION 32-QAM tolerates. This is the §5 model fix.
		resid_sigma_ = resid_sigma_rad;
		twocomp_     = (resid_sigma_ > 0.0);
		slow_f3db_   = slow_f3db_hz;
		slow_state_  = 0.0;
		rho_slow_    = 0.0;
		slow_gain_   = 0.0;
		// Stateful Hilbert FIR + matched real-path delay line.
		build_hilbert();
		hist_pos_ = 0;
		for (int i = 0; i < HIST_LEN; i++) hist_[i] = 0.0;
	}

	// Rotate ONE OFDM-symbol block of `n` real passband doubles in place:
	//   • draw one common phase φ_c for the whole block (per-symbol CPE)
	//   • add a small per-sample ICI jitter j[n]
	// No-op (and no rng draw) when disabled.
	void rotate(double* x, size_t n)
	{
		if (!enabled_ || n == 0) return;
		if (!cpe_init_)
		{
			// Symbol rate from the block size; set AR(1) pole + exact-RMS gain.
			double f_sym = fs_hz_ / (double)n;
			rho_cpe_  = std::exp(-2.0 * M_PI * f3db_hz_ / f_sym);
			double ss = 1.0 / std::sqrt(1.0 - rho_cpe_ * rho_cpe_);
			cpe_gain_ = (ss > 0.0) ? (sigma_cpe_ / ss) : 0.0;
			// Warm the common-phase AR(1) to its stationary distribution.
			for (int i = 0; i < 4000; i++)
				cpe_state_ = rho_cpe_ * cpe_state_ + rng_.gauss();
			// 2-component (experimental): set the SLOW drift AR(1) pole at the sub-Hz
			// cutoff so it is corrector-absorbed over a frame; warm it too.
			if (twocomp_) {
				rho_slow_  = std::exp(-2.0 * M_PI * slow_f3db_ / f_sym);
				double sss = 1.0 / std::sqrt(1.0 - rho_slow_ * rho_slow_);
				slow_gain_ = (sss > 0.0) ? (sigma_cpe_ / sss) : 0.0;
				for (int i = 0; i < 20000; i++)
					slow_state_ = rho_slow_ * slow_state_ + rng_.gauss();
			}
			cpe_init_ = true;
		}
		double phi_c;
		if (twocomp_) {
			// φ_c[m] = slow drift (corrector eats it) + i.i.d. per-symbol residual
			// (survives the ramp-only corrector, sets a STABLE per-frame EVM ceiling
			// with TIGHT sd; pure rotation 32-QAM tolerates). §5 model fix.
			slow_state_ = rho_slow_ * slow_state_ + rng_.gauss();
			phi_c = slow_gain_ * slow_state_ + resid_sigma_ * rng_.gauss();
		} else {
			// One common phase for the entire OFDM symbol (constant rotation = benign
			// CPE; this is the EVM-setting term that 32QAM can tolerate).
			cpe_state_ = rho_cpe_ * cpe_state_ + rng_.gauss();
			phi_c = cpe_gain_ * cpe_state_;
		}
		for (size_t i = 0; i < n; i++)
		{
			// Per-sample ICI jitter: band-limited AR(1) at F_ICI_HZ (RMS sigma_ici),
			// the intra-symbol component of θ[n]. Band-limited (not white) so it
			// varies coherently within the FFT window.
			ici_state_ = rho_ici_ * ici_state_ + rng_.gauss();
			double th = phi_c + ici_gain_ * ici_state_;
			// Optional coherent amplitude attenuation droop_ = exp(-σ_droop²/2)
			// (Petrovic 2007 §II — the ICI power loss off the wanted subcarrier).
			// DEFAULT OFF (droop_=1.0): Phase-3 testing found a UNIFORM amplitude
			// droop is REMOVED by the receiver AGC (automatic_gain_control,
			// telecom_system.cc:2447) before channel estimation, so it does NOT
			// move meanH here. meanH = mean(|H_pilot|) is a magnitude-mean — it is
			// insensitive to BOTH phase rotation (|e^{jθ}H|=|H|) AND uniform gain
			// (AGC-normalized); the only phase mechanism that droops it is large
			// random ICI, which craters 32QAM decode. The ~0.989 meanH the model
			// reaches comes naturally from the small ICI scatter. The knob is kept
			// for experimentation. (Comment matches behaviour: droop_ is inert at
			// the shipped default.)
			// Push current (optionally attenuated) sample; analytic pair at center.
			hist_[hist_pos_] = x[i] * droop_;
			double xd = hist_[(hist_pos_ - DELAY + 2 * HIST_LEN) % HIST_LEN]; // delay-matched real
			double xh = 0.0;                       // Hilbert (imag) at the same tap
			for (int k = 1; k <= DELAY; k += 2)    // Type-III: odd taps only
			{
				double c = htap_[k];               // = h[D-k] = -h[D+k]
				double a = hist_[(hist_pos_ - (DELAY - k) + 2 * HIST_LEN) % HIST_LEN];
				double b = hist_[(hist_pos_ - (DELAY + k) + 2 * HIST_LEN) % HIST_LEN];
				xh += c * (a - b);
			}
			hist_pos_ = (hist_pos_ + 1) % HIST_LEN;
			double ct = std::cos(th), st = std::sin(th);
			x[i] = xd * ct - xh * st;              // Re{ (xd + j·xh)·e^{jθ} }
		}
	}

private:
	static const int HILB_LEN = 65;            // odd FIR length (group delay 32)
	static const int DELAY    = (HILB_LEN - 1) / 2;
	static const int HIST_LEN = HILB_LEN + 1;  // ring length (> taps spanned)
	// Intra-symbol ICI cutoff. ~Subcarrier-spacing band (BW/Nc ≈ 2344/50 ≈ 47 Hz)
	// so the jitter is correlated across the FFT window and droops |H|, but stays
	// well below the signal bandwidth (genuine band-limited phase noise).
	static constexpr double F_ICI_HZ = 60.0;

	void build_hilbert()
	{
		// Ideal Type-III Hilbert h[k] = 2/(πk) for odd k, 0 for even; Hamming
		// window. Stored as htap_[k] = |h(center+k)| for k=1,3,5,… (odd), with
		// the antisymmetry (h[D-k] = -h[D+k]) applied analytically in rotate().
		for (int k = 0; k <= DELAY; k++) htap_[k] = 0.0;
		for (int k = 1; k <= DELAY; k += 2)
		{
			double ideal = 2.0 / (M_PI * (double)k);
			double w = 0.54 - 0.46 * std::cos(2.0 * M_PI * (double)(DELAY - k) /
			                                  (double)(HILB_LEN - 1));
			htap_[k] = ideal * w;
		}
	}

	cl_sim_xoshiro rng_;
	bool   enabled_;
	bool   cpe_init_;
	double f3db_hz_;
	double fs_hz_;
	double sigma_cpe_;
	double sigma_ici_;
	double rho_cpe_;
	double cpe_gain_;
	double cpe_state_;
	double rho_ici_;
	double ici_gain_;
	double ici_state_;
	double droop_;
	// EXPERIMENTAL 2-component common-phase state (§5; default OFF).
	bool   twocomp_;
	double resid_sigma_;   // i.i.d. per-symbol residual RMS (the EVM carrier)
	double slow_f3db_;     // sub-Hz drift pole (corrector-absorbed)
	double rho_slow_;
	double slow_gain_;
	double slow_state_;
	double htap_[DELAY + 1];
	double hist_[HIST_LEN];
	int    hist_pos_;
};

// ---------------------------------------------------------------------------
// DETERMINISTIC frequency-selective passband FIR — the HW EVM/meanH FLOOR.
//
// fact-documents/sim2-cfg16-phase-noise-faithfulness.md §5b/§9. The HW EVM
// ceiling (14.6 dB, per-frame sd 0.09) is DETERMINISTIC: a fixed, frame-
// repeating implementation residual, NOT a random phase process (a per-symbol
// common phase floors the per-frame sd at sqrt(2/Nsymb)=1.5-1.7 dB; only a
// fixed coherent distortion gives sd→0 AND 32-QAM decode AND CFG15/16
// co-location). The faithful realization is a fixed micro-multipath ripple in
// the analog/soundcard passband: a short STATEFUL FIR g[] = main tap + a few
// small "echo" taps. Its DTFT G(f) across the 2343.75 Hz OFDM band is a fixed
// per-subcarrier complex taper T(k)=G(f_k):
//   • a FREQUENCY-SELECTIVE residual the linear CPE corrector (one phase_rate
//     ramp, symbol-axis — ofdm.cc:1928) does NOT remove and that the Nc-point
//     DFT channel smoother (smooth_channel_estimate_dft, ofdm.cc:2127) cannot
//     fully represent ⇒ a FIXED pilot residual ⇒ a deterministic var= field ⇒
//     EVM_dB = -10log10(var), sd≈0 (frozen taps). The EVM echo gain sets the
//     residual magnitude; its delay sets the ripple's subcarrier-period.
//   • a frequency-selective AMPLITUDE ripple is NOT canceled by the AGC (which
//     normalizes the MEAN pilot amplitude, ofdm.cc:1978): DFT-smoothing of the
//     rippled complex H coherently shrinks |H_smooth| ⇒ mean|H| droops below 1
//     (the meanH 0.979 signature). A 2nd shorter-delay "tilt" echo carries the
//     droop with little extra EVM.
// 32-QAM tolerates it (a fixed per-subcarrier rotation+gain the soft-demap/LDPC
// absorb), so CFG15 and CFG16 BOTH sit at the same fixed EVM (co-locate).
//
// DETERMINISTIC: no rng, no seed — pure fixed taps. GATE-2 byte-identical and
// seed-independent. Applied BEFORE the AGC (i.e. first in process(), ahead of
// the small residual PN). Stateful history carried across OFDM-symbol blocks so
// there is no per-block edge discontinuity (mirrors the Hilbert FIR state).
class cl_sim_det_floor
{
public:
	// A single strong echo makes ONE deep amplitude null per ripple period, which
	// 32-QAM cannot tolerate (the MMSE erasure, ofdm.cc:2200, drops whole bands
	// and CFG16's tight min-distance dies on a cliff). The faithful floor (a real
	// soundcard's micro-multipath) is instead a SMALL COMB of echoes at staggered
	// delays: the combined G(f) carries the SAME RMS ripple (⇒ same fixed pilot
	// residual ⇒ same EVM) but SPREAD over several SHALLOW periods (no deep null),
	// which 32-QAM tolerates while CFG15/CFG16 still co-locate at the EVM floor.
	//
	//   echo_db   : MASTER EVM-ripple gain in dB rel. main tap (<0; 0 disables).
	//               The comb's per-tap amplitude derives from this and the count
	//               so the total ripple RMS == echo_amp (EVM stays db-controlled).
	//   echo_dly  : BASE delay (samples) of the first comb echo (sets ripple
	//               period); subsequent echoes step by `echo_step`.
	//   echo_n    : number of comb echoes (1 ⇒ single echo; >1 ⇒ shallow comb).
	//   echo_step : delay step between comb echoes (samples).
	//   tilt_db   : meanH-droop "tilt" echo gain in dB (<0). 0 ⇒ no tilt echo.
	//   tilt_dly  : the tilt echo's (shorter) delay (low-spatial-freq amplitude
	//               droop the smoother partly keeps ⇒ droops meanH).
	// PHASE-ONLY (all-pass) floor — the 32-QAM-FRIENDLY deterministic distortion.
	// A real echo makes AMPLITUDE nulls ⇒ the MMSE erasure (ofdm.cc:2200) drops
	// subcarriers ⇒ a minority of 32-QAM frames fail regardless of EVM level (the
	// comb's worst-case null alignment defeats CFG16 on ~10% of frames, multi-seed
	// verified). The faithful 32-QAM-tolerable floor is a fixed FREQUENCY-SELECTIVE
	// PHASE ripple with FLAT magnitude — a Schroeder all-pass section
	//   y[n] = -g·x[n] + x[n-D] + g·y[n-D]   (|H(e^jω)| ≡ 1 exactly, dispersive φ).
	// Flat |H| ⇒ NO amplitude nulls ⇒ NO MMSE erasure ⇒ 32-QAM tolerant; the
	// dispersive phase is a fixed per-subcarrier rotation the linear CPE corrector
	// (one ramp) cannot remove ⇒ a FIXED pilot residual ⇒ deterministic EVM, sd≈0.
	// meanH droops from the DFT-smoother coherently cancelling the phase ripple
	// (gentle, no erasure). ap_g sets the residual magnitude (EVM); ap_dly sets the
	// phase-ripple period. ap_g==0 ⇒ all-pass disabled (echo-comb path used).
	cl_sim_det_floor(double echo_db, int echo_dly, int echo_n, int echo_step,
	                 double tilt_db, int tilt_dly,
	                 double ap_g, int ap_dly, int ap_n)
	{
		// --- Schroeder all-pass cascade (phase-only floor). ---
		ap_g_ = ap_g;
		ap_dly_ = (ap_dly > 0 && ap_dly < RING - 1) ? ap_dly : 0;
		ap_n_ = (ap_n < 0) ? 0 : (ap_n > MAX_AP ? MAX_AP : ap_n);
		if (ap_g_ == 0.0 || ap_dly_ == 0) ap_n_ = 0;
		for (int s = 0; s < MAX_AP; s++) { ap_xpos_[s] = 0; ap_ypos_[s] = 0;
			for (int i = 0; i < AP_RING; i++) { ap_xh_[s][i] = 0.0; ap_yh_[s][i] = 0.0; } }

		double master = (echo_db < 0.0) ? std::pow(10.0, echo_db / 20.0) : 0.0;
		if (echo_n < 1) echo_n = 1;
		if (echo_n > MAX_ECHO) echo_n = MAX_ECHO;
		n_echo_ = 0;
		double e2 = 0.0;
		if (master > 0.0 && echo_dly > 0)
		{
			// Distribute the master RMS over echo_n taps: per-tap amp =
			// master/sqrt(n) so sqrt(Σ amp²) == master (total ripple RMS held).
			double per = master / std::sqrt((double)echo_n);
			for (int m = 0; m < echo_n; m++)
			{
				int d = echo_dly + m * echo_step;
				if (d <= 0 || d >= RING - 1) continue;
				// Alternate sign so successive echoes do not pile a single deep
				// null (a comb of ± echoes ⇒ several shallow nulls, 32-QAM-safe).
				echo_amp_[n_echo_] = (m & 1) ? -per : per;
				echo_dly_[n_echo_] = d;
				e2 += per * per;
				n_echo_++;
			}
		}
		tilt_amp_ = (tilt_db < 0.0) ? std::pow(10.0, tilt_db / 20.0) : 0.0;
		tilt_dly_ = (tilt_dly > 0 && tilt_dly < RING - 1) ? tilt_dly : 0;
		if (tilt_amp_ > 0.0 && tilt_dly_ > 0) e2 += tilt_amp_ * tilt_amp_;
		enabled_  = (n_echo_ > 0) || (tilt_amp_ > 0.0 && tilt_dly_ > 0) || (ap_n_ > 0);
		// Unit-energy normalize so |G(f)|² averages 1 ⇒ the SNR3k axis is exact.
		// (The all-pass is already unit-magnitude, so it does not enter e2.)
		norm_ = 1.0 / std::sqrt(1.0 + e2);
		pos_ = 0;
		for (int i = 0; i < RING; i++) hist_[i] = 0.0;
	}

	// One Schroeder all-pass section: y[n] = -g·x[n] + x[n-D] + g·y[n-D].
	double allpass_section(int s, double x)
	{
		double xD = ap_xh_[s][(ap_xpos_[s] - ap_dly_ + AP_RING) % AP_RING];
		double yD = ap_yh_[s][(ap_ypos_[s] - ap_dly_ + AP_RING) % AP_RING];
		double y  = -ap_g_ * x + xD + ap_g_ * yD;
		ap_xh_[s][ap_xpos_[s]] = x;  ap_xpos_[s] = (ap_xpos_[s] + 1) % AP_RING;
		ap_yh_[s][ap_ypos_[s]] = y;  ap_ypos_[s] = (ap_ypos_[s] + 1) % AP_RING;
		return y;
	}

	// Apply the fixed FIR to a passband block in place. No-op + no state change
	// when disabled (byte-identical to the pre-floor channel).
	void apply(double* x, size_t n)
	{
		if (!enabled_ || n == 0) return;
		for (size_t i = 0; i < n; i++)
		{
			double v = x[i];
			// Phase-only all-pass cascade FIRST (flat magnitude, dispersive phase).
			for (int s = 0; s < ap_n_; s++) v = allpass_section(s, v);
			// Then the (optional) amplitude echo-comb + tilt for any residual shaping.
			hist_[pos_] = v;
			double y = v;                                 // main tap (gain 1)
			for (int m = 0; m < n_echo_; m++)
				y += echo_amp_[m] * hist_[(pos_ - echo_dly_[m] + RING) % RING];
			if (tilt_dly_ > 0)
				y += tilt_amp_ * hist_[(pos_ - tilt_dly_ + RING) % RING];
			pos_ = (pos_ + 1) % RING;
			x[i] = y * norm_;
		}
	}

	bool enabled() const { return enabled_; }

private:
	static const int RING     = 1024;  // > max supported echo delay (comb span)
	static const int MAX_ECHO = 8;
	static const int MAX_AP   = 4;     // all-pass cascade depth
	static const int AP_RING  = 1024;  // > max all-pass delay
	bool   enabled_;
	int    n_echo_;
	double echo_amp_[MAX_ECHO];
	int    echo_dly_[MAX_ECHO];
	double tilt_amp_, norm_;
	int    tilt_dly_;
	double hist_[RING];
	int    pos_;
	// Schroeder all-pass cascade (phase-only floor) state.
	double ap_g_;
	int    ap_dly_, ap_n_;
	double ap_xh_[MAX_AP][AP_RING];
	double ap_yh_[MAX_AP][AP_RING];
	int    ap_xpos_[MAX_AP], ap_ypos_[MAX_AP];
};

// Scalar AWGN channel for ONE direction. Mirrors sim_channel_relay.py:Channel
// (AWGN-only path, fading=False): sticky TX-power tracking + the BER-harness
// noise stddev formula. process() applies the DETERMINISTIC frequency-selective
// floor (the dominant EVM/meanH source), then rotates by the SMALL residual
// band-limited phase noise (kept as a small perturbation so the floor is "near
// but not exactly noiseless"), then adds AWGN IN PLACE to a passband block.
class cl_sim_awgn
{
public:
	// snr3k_db == channel SNR3k (the same axis the relay's --snr takes; the WGN
	// label maps via WGN_TO_SNR3K=2.4 elsewhere — the caller passes SNR3k here).
	// snr3k_db >= 900 → AWGN is transparent (no additive noise; clean cell) —
	// but PHASE NOISE still applies (it is the testbed's permanent EVM ceiling,
	// independent of the AWGN SNR knob).
	cl_sim_awgn(uint64_t seed, double snr3k_db)
		: rng_(seed), pn_(pn_seed(seed), pn_sigma(), pn_f3db(), 48000.0, pn_ici(),
		                  pn_droop(), pn_resid(), pn_slow_f3db()),
		  det_(det_echo_db(), det_echo_dly(), det_echo_n(), det_echo_step(),
		       det_tilt_db(), det_tilt_dly(),
		       det_ap_g(), det_ap_dly(), det_ap_n())
	{
		clean_     = (snr3k_db >= 900.0);
		snr_lin_   = clean_ ? 0.0 : std::pow(10.0, snr3k_db / 10.0);
		peak_ms_   = 0.0;
		noise_std_ = 0.0;
	}

	// PN-rotate (always, incl. clean) then add AWGN (skipped when clean). Sticky
	// power: only chunks with real energy update P_sig (silent gaps don't drag
	// the floor). P_sig is measured AFTER PN rotation, which is power-preserving
	// to within the Hilbert filter's band edges, so the SNR axis is unchanged.
	void process(double* x, size_t n)
	{
		if (n == 0) return;

		det_.apply(x, n);   // DETERMINISTIC freq-selective floor (dominant EVM/meanH)
		pn_.rotate(x, n);   // small residual phase noise on top (near-but-not-zero sd)

		if (clean_) return; // clean cell: floor+PN only, no additive AWGN

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

	// Calibrated defaults (Phase-3 faithfulness gate, this worktree). PN_DEG sets
	// the EVM ceiling to HW 14.5 dB; PN_ICI sets the meanH droop to HW 0.979.
	// Phase-3 calibration (this worktree, pinned CFG15 clean cell):
	//   PN_DEG=13.8° lands the CFG15 (decode-reliable) post-EQ EVM ceiling at
	//   14.48 dB == HW 14.5; meanH naturally drops to ~0.989 (within the gate's
	//   ±0.01 of HW 0.979) from the per-symbol-common-phase residual.
	//   PN_DROOP defaults OFF: a UNIFORM coherent amplitude droop is removed by
	//   the receiver AGC (automatic_gain_control, telecom_system.cc:2447) before
	//   the channel estimate, so it does not move meanH — the knob is retained for
	//   experimentation but is ineffective by design here (see rotate()).
	// Attempt-2 (§9/§10): the DETERMINISTIC all-pass+echo floor is the dominant EVM
	// source. PN is kept ONLY as a SMALL residual perturbation (the floor is "near
	// but not exactly noiseless" — real HW has a little residual variation). PN_DEG
	// dropped 13.8 → 0.5° (alone ≈ 35 dB EVM, far above the 14.7 floor ⇒ a tiny sd
	// contribution). A LARGER PN re-introduces 32-QAM BP-iter-cap fragility on the
	// few borderline CFG16 frames (any per-symbol common-phase dither tips them over
	// the cap); 0.5° at the -20 dB EVM-echo margin keeps CFG16 robustly ≥0.95 across
	// seeds while preserving a genuine random residual (§10.3).
	static constexpr double PN_DEG_DEFAULT       = 0.5;
	static constexpr double PN_ICI_DEFAULT       = 0.15;
	static constexpr double PN_DROOP_DEG_DEFAULT = 0.0;

	// --- Phase-noise knobs (env-overridable; defaults = HW-faithful ceiling) ---
	//   MERCURY_SIM2_PN_DEG  : TOTAL sigma_phi in DEGREES (default calibrated so
	//                          the pinned CFG15/16 clean EVM ceiling == HW 14.5 dB;
	//                          set 0 to DISABLE PN for an A/B).
	//   MERCURY_SIM2_PN_F3DB : common-phase correlation bandwidth in Hz (default
	//                          2.5; held in the spec's 2-3 Hz band).
	//   MERCURY_SIM2_PN_ICI  : σ_ici/σ_cpe split (default calibrated so the meanH
	//                          droop lands at HW 0.979). Small (≪1).
	static double pn_sigma()
	{
		const char* e = std::getenv("MERCURY_SIM2_PN_DEG");
		double deg = (e && *e) ? atof(e) : PN_DEG_DEFAULT;
		if (deg < 0.0) deg = 0.0;
		return deg * M_PI / 180.0;
	}
	static double pn_f3db()
	{
		const char* e = std::getenv("MERCURY_SIM2_PN_F3DB");
		double f = (e && *e) ? atof(e) : 2.5;
		if (f <= 0.0) f = 2.5;
		return f;
	}
	static double pn_ici()
	{
		const char* e = std::getenv("MERCURY_SIM2_PN_ICI");
		double r = (e && *e) ? atof(e) : PN_ICI_DEFAULT;
		if (r < 0.0) r = 0.0;
		return r;
	}
	// MERCURY_SIM2_PN_DROOP : σ_droop in DEGREES — the coherent ICI amplitude
	// attenuation that sets meanH = exp(-σ_droop²/2). Default calibrated to HW
	// 0.979. Independent of PN_DEG so the droop is dialed without scatter.
	static double pn_droop()
	{
		const char* e = std::getenv("MERCURY_SIM2_PN_DROOP");
		double deg = (e && *e) ? atof(e) : PN_DROOP_DEG_DEFAULT;
		if (deg < 0.0) deg = 0.0;
		return deg * M_PI / 180.0;
	}
	// EXPERIMENTAL 2-component knobs (§5 diagnosis validation; default 0 = OFF).
	//   MERCURY_SIM2_PN_RESID_DEG  : i.i.d. per-symbol residual common-phase RMS in
	//                                DEGREES. >0 engages the 2-component model.
	//   MERCURY_SIM2_PN_SLOW_F3DB  : sub-Hz drift pole (Hz, default 0.2) — the
	//                                corrector-absorbed slow component.
	static double pn_resid()
	{
		const char* e = std::getenv("MERCURY_SIM2_PN_RESID_DEG");
		double deg = (e && *e) ? atof(e) : 0.0;
		if (deg < 0.0) deg = 0.0;
		return deg * M_PI / 180.0;
	}
	static double pn_slow_f3db()
	{
		const char* e = std::getenv("MERCURY_SIM2_PN_SLOW_F3DB");
		double f = (e && *e) ? atof(e) : 0.2;
		if (f <= 0.0) f = 0.2;
		return f;
	}
	// --- DETERMINISTIC floor knobs (env-overridable; defaults = HW-faithful
	// floor calibrated §9/§10). The deterministic FIR is the DOMINANT EVM/meanH
	// source; it is seed-INDEPENDENT (pure fixed taps ⇒ GATE-2 byte-identical).
	//   MERCURY_SIM2_DET_ECHO_DB  : EVM-ripple echo gain dB (<0; 0 disables the
	//                               whole floor). Sets the fixed pilot residual
	//                               ⇒ the EVM ceiling.
	//   MERCURY_SIM2_DET_ECHO_DLY : EVM echo delay (passband samples) ⇒ the
	//                               ripple's subcarrier-period (must exceed the
	//                               DFT-smoother window to survive as residual).
	//   MERCURY_SIM2_DET_TILT_DB  : meanH-droop tilt echo gain dB (<0).
	//   MERCURY_SIM2_DET_TILT_DLY : tilt echo delay (short ⇒ smooth amplitude
	//                               tilt the smoother partly keeps ⇒ droops meanH).
	//   MERCURY_SIM2_DET_ECHO_N    : comb echo count (>1 ⇒ shallow comb, 32-QAM-safe).
	//   MERCURY_SIM2_DET_ECHO_STEP : delay step between comb echoes (samples).
	//   MERCURY_SIM2_DET_AP_G      : Schroeder all-pass coeff (phase-only floor —
	//                                flat magnitude ⇒ no amplitude null ⇒ 32-QAM
	//                                tolerant; this is the SHIPPED EVM mechanism).
	//                                |g|<1; 0 ⇒ all-pass off (echo-comb path).
	//   MERCURY_SIM2_DET_AP_DLY    : all-pass delay D (samples) ⇒ phase-ripple period.
	//   MERCURY_SIM2_DET_AP_N      : all-pass cascade depth (1-4).
	// SHIPPED floor (§10): a STRONG phase-only Schroeder all-pass (g=0.50, D=16,
	// 3-section cascade) carries the bulk of the gentle, 32-QAM-tolerable residual,
	// + a SMALL amplitude echo comb (-18 dB) for the final EVM push. Calibrated on
	// the pinned clean cells so BOTH CFG15+CFG16 land EVM≈14.7 (in 14.6±0.3),
	// per-frame sd 0.15-0.6 (≪ the old random-PN 1.6; toward HW 0.09), CFG15/CFG16
	// EVM-CO-LOCATED, CFG16 32-QAM decoding (1.00 on most seeds; see §10 honesty
	// note). meanH lands ~0.983 jointly-with-decode (the §6/§10 open question: the
	// last ~0.004 to HW 0.979 is unreachable without an amplitude ripple that
	// breaks 32-QAM decode — re-derive the HW meanH from the clean-decode
	// population before treating 0.979 as a hard joint target).
	static constexpr double DET_ECHO_DB_DEFAULT   = -17.0; // small EVM-push echo (lands EVM ~14.85, robust CFG16 across seeds)
	static constexpr int    DET_ECHO_DLY_DEFAULT  = 80;
	static constexpr int    DET_ECHO_N_DEFAULT    = 4;
	static constexpr int    DET_ECHO_STEP_DEFAULT = 40;
	static constexpr double DET_TILT_DB_DEFAULT   = 0.0;   // tilt echo OFF (unreliable meanH lever)
	static constexpr int    DET_TILT_DLY_DEFAULT  = 48;
	static constexpr double DET_AP_G_DEFAULT      = 0.50;  // strong gentle phase floor
	static constexpr int    DET_AP_DLY_DEFAULT    = 16;
	static constexpr int    DET_AP_N_DEFAULT      = 3;
	static double det_echo_db()
	{
		const char* e = std::getenv("MERCURY_SIM2_DET_ECHO_DB");
		return (e && *e) ? atof(e) : DET_ECHO_DB_DEFAULT;
	}
	static int det_echo_dly()
	{
		const char* e = std::getenv("MERCURY_SIM2_DET_ECHO_DLY");
		return (e && *e) ? atoi(e) : DET_ECHO_DLY_DEFAULT;
	}
	static int det_echo_n()
	{
		const char* e = std::getenv("MERCURY_SIM2_DET_ECHO_N");
		return (e && *e) ? atoi(e) : DET_ECHO_N_DEFAULT;
	}
	static int det_echo_step()
	{
		const char* e = std::getenv("MERCURY_SIM2_DET_ECHO_STEP");
		return (e && *e) ? atoi(e) : DET_ECHO_STEP_DEFAULT;
	}
	static double det_tilt_db()
	{
		const char* e = std::getenv("MERCURY_SIM2_DET_TILT_DB");
		return (e && *e) ? atof(e) : DET_TILT_DB_DEFAULT;
	}
	static int det_tilt_dly()
	{
		const char* e = std::getenv("MERCURY_SIM2_DET_TILT_DLY");
		return (e && *e) ? atoi(e) : DET_TILT_DLY_DEFAULT;
	}
	static double det_ap_g()
	{
		const char* e = std::getenv("MERCURY_SIM2_DET_AP_G");
		return (e && *e) ? atof(e) : DET_AP_G_DEFAULT;
	}
	static int det_ap_dly()
	{
		const char* e = std::getenv("MERCURY_SIM2_DET_AP_DLY");
		return (e && *e) ? atoi(e) : DET_AP_DLY_DEFAULT;
	}
	static int det_ap_n()
	{
		const char* e = std::getenv("MERCURY_SIM2_DET_AP_N");
		return (e && *e) ? atoi(e) : DET_AP_N_DEFAULT;
	}
	// Deterministic, distinct PN seed: a fixed bijective transform of the AWGN
	// ctor seed. NEVER shares the AWGN rng_ stream and preserves the per-direction
	// seed separation (the A→B / B→A seeds differ in bit 0, so the transformed
	// PN seeds differ too). GATE-2: same ctor seed ⇒ identical PN stream.
	static uint64_t pn_seed(uint64_t s)
	{
		return (s ^ 0xD1B54A32D192ED03ULL) + 0x9E3779B97F4A7C15ULL;
	}

	cl_sim_xoshiro     rng_;
	cl_sim_phase_noise pn_;
	cl_sim_det_floor   det_;
	bool   clean_;
	double snr_lin_;
	double peak_ms_;
	double noise_std_;
};

#endif // INC_SIM_CHANNEL_H_
