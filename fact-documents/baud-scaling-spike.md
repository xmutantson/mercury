# P0 — Baud-scaling MAKE-OR-BREAK spike (sim) — the gate on frontier option (c)

**Status:** IN PROGRESS (2026-06-01). Worktree `sim/baud-scaling-spike` @ fef293f
(`C:/Users/kamer/mercury_wt/baud-scaling`). SIM ONLY — no bench (v13 + the ULTRA HW
verdict own the testbed). Gates `efficient-deep-modulation-frontier.md` §6 P0 / §9 [?].

## §0 The question to gate (frontier doc §9 [?])
Option (c) = retire repetition, get depth from **baud-scaling** (longer MFSK symbols →
~+2.6-3 dB per 2× slowdown, the FST4/Q65 time-bandwidth ladder, NO combining loss) +
one low-rate code. P0 asks: **does Mercury's OFDM-block PHY cleanly support 2×/4× longer
MFSK symbols, AND does the decode floor deepen ~+2.6-3 dB per 2× slowdown?** OR is the
baud knob PINNED to the OFDM clock (ultra §15 hinted it is)?

## §1 MECHANISM — how the MFSK symbol length is set (code-grounded)

**The MFSK symbol = exactly ONE OFDM-block FFT window. Confirmed end-to-end:**

1. **Symbol period anchor.** `data_container.cc:149`: `sym_time_ms = 1000 * Nofdm *
   frequency_interpolation_rate / 48000`, where `Nofdm = Nfft + Ngi`
   (`data_container.cc:103`, `ofdm.cc:560/705/891`) and `Ngi = Nfft*gi`
   (`ofdm.cc:162`). With Nfft=256, Ngi=36 (gi=36/256, the 3.0 ms default), interp=4 →
   **24.33 ms/symbol = (256+36)*4/48000.** This is the `connect-suffix-fec-research.md:72`
   / ultra §1 anchor.
2. **TX.** `telecom_system.cc:562` `mfsk.mod()` writes one one-hot subcarrier per stream
   per symbol into `ofdm_framed_data` (Nc-wide). `:668` `ofdm.symbol_mod(&framed[i*Nc],
   &modulated[i*Nofdm])` = `zero_padder`(Nc→Nfft) → `ifft(Nfft)` → `gi_adder`
   (`ofdm.cc:1023-1028`). **One Nfft IFFT per symbol, stride Nofdm.**
3. **RX.** `telecom_system.cc:2332` `ofdm.symbol_demod(&baseband[i*Nofdm+...],
   &demod[i*Nc])` = `gi_remover` → `fft(Nfft)` → `zero_depadder` (`ofdm.cc:1030-1035`);
   `:2340` `mfsk.demod(demod, ...)` reads per-tone energy from `fft_in[s*Nc +
   stream_offsets[st] + m]` (`mfsk.cc:1054`). **One Nfft FFT per symbol → the coherent
   integration window per MFSK symbol is exactly Nfft samples.**
4. **Tone grid.** M tones = M contiguous FFT bins inside the Nc grid (`mfsk.cc:118-124`
   `stream_offsets`); tone spacing = subcarrier spacing = (48000/interp)/Nfft =
   12000/256 = **46.875 Hz** WB. `zero_padder` (`ofdm.cc:331-352`) places Nc bins around
   DC in the Nfft grid.

**Is Nfft a per-config knob? NO — it is GLOBAL.** `load_configuration` (`telecom_system.cc
:5327-5330`) copies `ofdm.Nc/Nfft/gi/Nsymb` from `default_configurations_telecom_system`
(the ctor defaults, `physical_config.cc:35-37`: Nfft=256, gi) for **EVERY** config,
including all ROBUST/MFSK. The per-config table (`:5029-5168`) varies ONLY `_modulation`,
`_ldpc_rate`, preamble Nsymb, channel estimator. MFSK configs additionally differ in M
(32 vs 16) and nStreams — NOT in Nfft/gi.

**Is the engine hardwired to 256? NO — it is parameterized.** `cl_ofdm::init(Nfft, Nc,
Nsymb, gi)` (`ofdm.cc:143`); FFT plans cache by size (`get_fft_plan(n)`, `ofdm.cc:320`);
`data_container::set_size(..., Nfft, Nofdm, ...)` allocates every buffer from Nofdm
(`data_container.cc:95-194`); `bandwidth = 48000*Nc/Nfft/interp` recomputed at init
(`telecom_system.cc:4217`). So 256 is a *value pushed in*, not a structural constant.

**⇒ To make a longer COHERENT MFSK symbol you must enlarge the per-symbol FFT window =
enlarge Nfft.** There is no separate "symbol length" knob: the symbol length, the tone
spacing, the GI, the occupied bandwidth, and the buffer sizing are ALL functions of Nfft.
A genuine 2×/4× longer symbol = Nfft 256→512→1024 (with Nc/interp/GI co-managed). This is
the structural coupling the gate asked about.

## §2 MEASUREMENT VALIDITY — the trap (the load-bearing reasoning)

The cliff-sweep harness `passband_test_EsN0` (`telecom_system.cc:365`) defines the swept
x-axis for MFSK as **in-band channel SNR over `bandwidth`** (`:410-420`):
`sigma = sqrt(2*P_sig*f_nyquist / (10^(EsN0/10) * bandwidth))` ⇒
`EsN0 = P_sig / (P_noise in `bandwidth` Hz)`. This is the SNR3k convention (a fixed
reference bandwidth) — the CORRECT frame for baud-scaling gain, **IFF the reference
bandwidth is held fixed.**

**Derivation (AWGN, fixed TX power):** a length-Nfft FFT of a pure tone gives bin power
∝ Nfft²·a²; white noise gives bin power ∝ Nfft·(σ²/2). Post-FFT per-tone SNR ∝
Nfft·a²/(σ²/2) → **+3 dB per 2× Nfft (coherent processing gain).** BUT `bandwidth ∝
1/Nfft` (`:4217`), so the harness's `EsN0` axis ∝ P_sig/(σ²/2)·Nfft — it scales with
Nfft by the SAME factor. **If `bandwidth` is allowed to track Nfft, the cliff appears at
the SAME EsN0 for every K** — the real +3 dB/2× gain is exactly cancelled in the reported
number because the noise reference shrank with the signal. This is the measurement trap.

**Fix:** pin the noise-calibration reference `bandwidth` to the K=1 value (2343.75 Hz)
while Nfft scales. Then the SNR axis is a fixed 3 kHz-class reference and the cliff moves
by the TRUE baud-scaling gain. (Physically: this measures the FST4-period lever — the
signal genuinely narrows and lengthens as it slows; depth is referenced to a fixed
noise-PSD/fixed-bandwidth, exactly WSJT-X Table 7.)

## §3 THE SPIKE (flag-gated, byte-identical when off)

**Knob:** env var `MERCURY_BAUD_MULT=K` (1/2/4), read in `load_configuration`. When K>1
AND `is_robust_config` (MFSK), scale `ofdm.Nfft *= K` right after `:5328`. Everything
downstream (Ngi=Nfft·gi, Nofdm, buffers, FFT plan, tone grid on Nc bins) follows. Default
unset ⇒ K=1 ⇒ no change to any config (the load-bearing safety gate).

**Honest reference:** store the K=1 `bandwidth` and pin the MFSK noise calibration in
`passband_test_EsN0` to it (so the SNR axis is fixed across K).

**Measurement:** `mercury.exe -m PLOT_PASSBAND -s 100 -R` (ROBUST_0, MFSK M=32) at
K=1/2/4; the harness prints `EsN0;BER` from −25..+5 dB. The cliff = the SNR where BER
falls to ~0. Report **measured dB-per-2×-doubling** vs the predicted +2.6-3.

## §4 Cross-layer note
Spike is SIM-only and gated to `is_robust_config` + env-var-on, so it cannot perturb any
shipped config, the ARQ/SACK/optimizer layers, or non-MFSK PHY. The only state changed is
`ofdm.Nfft` (global, but only when the env var is set) and the BER-harness noise reference.
No production path reads `MERCURY_BAUD_MULT`. The flag is a measurement instrument, not a
shippable feature — the verdict decides whether to build the real rungs.

## §5 RESULTS (2026-06-01, worktree @ build o3, `mercury --test` 50/0)

AWGN cliff sweep, ROBUST_0 (config 100, MFSK M=32, rate-1/16 LDPC),
`-m PLOT_PASSBAND -s 100`, harness sweep −25..+5 dB / 1 dB step / 3 frames per
point, noise referenced to the PINNED K=1 bandwidth (2343.75 Hz, §2). Cliff =
deepest SNR with BER→0 (the point below it still fails). Init dims confirm the
mechanism: same Nsymb=320 (codeword symbol-count unchanged), Nofdm and occupied
BW scale with Nfft.

| K | Nfft | Nofdm | symbol len | occupied BW | **cliff (SNR3k)** | Δ vs K=1 |
|---|------|-------|-----------|-------------|-------------------|----------|
| 1 | 256  | 310   | 1× (24.3 ms-class) | 2344 Hz | **−13 dB** | — |
| 2 | 512  | 620   | 2×        | 1172 Hz | **−16 dB** | **+3.0 dB** |
| 4 | 1024 | 1240  | 4×        | 586 Hz  | **−20 dB** | **+7.0 dB** |

Reproduced on independent AWGN seeds: K=1 → −13, K=4 → −20 (stable).

**Measured dB-per-2×-doubling: +3.0 (1→2), +4.0 (2→4); mean +3.5 dB/doubling.**
The second step slightly exceeds the +2.6-3 prediction, but the 3-frame / 1 dB
grid gives ±1 dB cliff-location resolution, so +3.5 is within noise of +2.6-3.
**The floor deepens ≈ the predicted +2.6-3 dB per 2× baud-slowdown. The
prediction HOLDS.**

(Cosmetic: the `[BAUD-SPIKE]` banner prints "tone spacing inf Hz" because
`frequency_interpolation_rate` is assigned later in `load_configuration`
(`:5410`) than the banner; the Nfft scaling itself uses the literal default and
is correct — the +3/+4 dB results confirm it. Harmless to the measurement.)

## §6 VERDICT — GO (option (c) is viable as a re-tuning of the time-bandwidth lever)

**Mechanism:** The MFSK symbol = exactly one Nfft FFT window (§1). The baud knob
is therefore NOT an independent parameter — symbol length, tone spacing, GI, and
occupied bandwidth are all functions of `ofdm.Nfft`. Lengthening the coherent
symbol == enlarging Nfft. HOWEVER, Nfft is a fully PARAMETERIZED engine input
(FFT plans cache by size; `ofdm.init(Nfft,...)`, `data_container::set_size(...,
Nfft,...)`, and `bandwidth=f(Nfft)` all recompute), NOT a structural constant.
**So baud-scaling is achievable by a coordinated RE-TUNE of Nfft (+ co-managing
Nc/GI/occupied-BW + pinning the noise reference), not a from-scratch rebuild of
the symbol-gen path.** The spike implemented it in **48 lines across 2 files**
(scale Nfft for MFSK + pin the BER noise reference), and `--test` stays 50/0
(K=1 byte-identical).

The "pinned to the OFDM clock" hypothesis (ultra §15) is **half-true and not
fatal:** baud IS coupled to the OFDM clock (Nfft), but the OFDM clock is itself a
knob, so the coupling is a re-tune, not a wall. The work to build real rungs is:
(a) make Nfft (or an equivalent symbol-length multiplier) per-config in the
config table instead of global; (b) co-scale Nc / tone-grid / GI per the chosen
occupied-bandwidth policy (FST4-period "narrow the signal" — simplest, what the
spike measured — vs Q65 "constant band, narrower+more tones"); (c) the deep
rungs are slower symbols, so the §10/§11 ULTRA choreography-timer work (RSP
window + CMD connection_timeout scaled to the longer frame) is a HARD
prerequisite — the longer-symbol rungs hit the SAME timer-vs-airtime bug ULTRA
INCR-1/3 hit, only worse.

**Floor scaling:** measured +3.5 dB/doubling (mean) vs predicted +2.6-3 — HOLDS.
ROBUST_0's AWGN cliff moved −13 → −20 dB SNR3k across 1×→4× baud. This is the
real time-bandwidth gain, with NO noncoherent-combining loss (it is one longer
coherent FFT per symbol, not R squared-and-summed reps).

**⇒ GO: proceed to P1 (single low-rate code) + the baud-scaled rung family.** The
mechanism is a re-tune, the floor scales as predicted. The hidden coupling that
DOES need respecting before/with the rungs is the choreography-timer set (ULTRA
§10/§11), not the PHY symbol-gen.

**Artifacts:** worktree `sim/baud-scaling-spike` @ 79b7794 (`C:/Users/kamer/
mercury_wt/baud-scaling`); cliff CSVs `/tmp/baud_k1.csv`,`/baud_k2.csv`,
`/baud_k4.csv`; flag `MERCURY_BAUD_MULT={2,4,8}`; gate test = `mercury --test`
(50/0, incl. K=1 byte-identical) + the three PLOT_PASSBAND sweeps above.
