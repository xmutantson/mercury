# data-flow — PAPR cut (P1-A clipping ceiling lever, Phase 1)

Fact document for making the OFDM PAPR clip depth runtime-tunable and A/B-able.
Branch `feat/clipfix-papr-tune` off monitor `627c370`. All file:line cites from this
worktree (`C:/Users/kamer/mercury_wt/clipfix`). Facts from executing/reading code.

Motivation: `_research/DEEPLEVER_COMPLETENESS.md` levers #1 (RX clip-noise
cancellation) and #2 (PAPR re-tune). Phase 1 = the nearly-free re-tune that is the
failing-test-first baseline Phase 2 (RX cancellation) must beat.

---

## §1. The clip primitive

`cl_ofdm::peak_clip(double* in, int nItems, double papr)` (`source/physical_layer/ofdm.cc:2461`,
real-valued) and the complex overload (`ofdm.cc:2490`). Hard magnitude clip:
- measures average power over `nItems`,
- `peak_allowed = sqrt(avg_power * 10^(papr/10))` (real) / `avg_power*10^(papr/10)` then
  `sqrt` on assignment (complex),
- clamps each sample's magnitude to `peak_allowed`, preserving sign (real) / phase (complex).

`papr` is the **PAPR cut in dB** (peak-to-average ratio ceiling). Larger papr ⇒ higher
threshold ⇒ less clipping. `papr = 99` (the bare-constructor pre-init value,
`ofdm.cc:82-83`) ⇒ threshold ≈ 10^9.9 × avg ⇒ effectively **no clip** (OFDM peak PAPR is
~10-12 dB, never near 99 dB).

## §2. The two live parameters and their SINGLE assignment site

Member fields (`include/physical_layer/ofdm.h:291-292`):
```
double preamble_papr_cut;   // dB
double data_papr_cut;       // dB
```
Config-struct defaults (`include/physical_layer/physical_config.h:63-64`,
set in `source/physical_layer/physical_config.cc:115-116`):
```
ofdm_preamble_papr_cut = 7;   // dB
ofdm_data_papr_cut     = 10;  // dB
```
The ONLY place the config defaults are copied into the LIVE `ofdm` object is
`cl_telecom_system::load_configuration()` at **telecom_system.cc:10084-10085**:
```
ofdm.preamble_papr_cut = default_configurations_telecom_system.ofdm_preamble_papr_cut;
ofdm.data_papr_cut     = default_configurations_telecom_system.ofdm_data_papr_cut;
```
Verified by grep: NO other `ofdm.{data,preamble}_papr_cut = ...` assignment exists in
`source/` (only `ofdm.cc:82-83` bare-ctor pre-init, overridden here). So overriding the
live value at THIS site (after the copy) propagates to every consumer for the whole run.

## §3. Producers (writers) of the live values
1. `ofdm.cc:82-83` — bare constructor, `=99` (no-clip pre-init). Overridden before any TX.
2. `telecom_system.cc:10084-10085` — `load_configuration()`, copies config defaults.
   **THE override site.** Re-run on every `load_configuration(cfg)` (gearshift switch),
   so an env override here is re-applied on every config switch (intended).

## §4. Consumers (readers) of the live values — every peak_clip(... data/preamble) call
TX data clip (`ofdm.data_papr_cut`):
- `telecom_system.cc:887` — preamble+data SINGLE_MESSAGE TX (`transmit_bit`, the main path).
- `:3633,:3707,:3754` — ACK / ACK-SNR / ACK-SACK control patterns.
- `:4061` — ctrl-suffix pattern.
- `:4514` — ACK pattern (2nd).
- `:4584` — hail.
- `:7837` — block-TX data (`transmit_byte` SINGLE_MESSAGE block path).
- `:8666` — block-TX data (2nd block path).
- `mfsk_ctrl_codec_tests.cc:3749` — test harness only.

TX preamble clip (`ofdm.preamble_papr_cut`):
- `:886` — preamble (main).
- `:7836`, `:8665` — block-TX preamble.

RX correlation template replay (`ofdm.preamble_papr_cut`):
- **`:10402`** — `get_pre_equalization_channel()` rebuilds the preamble correlation
  template by replaying the full TX chain (`baseband_to_passband → peak_clip → FIR_tx1/2 →
  passband_to_baseband`). It reads the LIVE `ofdm.preamble_papr_cut`, so any override is
  automatically reflected — RX template matches TX preamble clip. This is the RX-match the
  research flagged (it cited the old line :9235-9240; in 627c370 it is :10402).

KEY: the **data** clip has no stored RX template — data symbols are demodulated via FFT/EQ
from whatever arrived. So data-clip RX-matching is automatic: RX just decodes the clipped
waveform. Only the PREAMBLE has a correlation template, and it already reads the live value.

## §5. Cross-layer audit (CLAUDE.md §"Cross-Layer", 5 questions)

The change adds an ENV OVERRIDE at the single producer site (§2). It does not change
the data-flow shape — it only changes the numeric value of an existing parameter, and only
when the env var is set.

1. **Producers**: §3. Override injected at producer #2 (the only one that matters).
2. **Consumers**: §4. ALL read the live `ofdm.{data,preamble}_papr_cut`. None cache a copy.
   (grep confirms no local copies of the value.)
3. **Valid states**: papr ∈ (0, 99] dB. Default 10 (data) / 7 (preamble). 99 = no-clip.
   BEFORE producer #2 runs the value is 99 (no-clip) — but no TX happens before
   `load_configuration`, so no consumer ever sees the pre-init 99 on a real path.
4. **Invariants consumers assume**: (a) the SAME papr is used at TX-data-clip and on the
   preamble template replay as was used at TX-preamble-clip — maintained, because both read
   the one live value, re-copied on every config switch. (b) RX demod does NOT need to know
   the data papr (no data template) — holds. (c) The clip is applied AFTER
   `baseband_to_passband` and BEFORE the TX FIR in every path (verified at :886-887 etc. and
   the template :10393→:10402→:10405) — unchanged by this fix (value-only change).
5. **What the fix changes**: only the numeric papr value, only when env set. Every consumer
   reads the new value identically. No consumer assumption is violated. When env unset →
   byte-identical (defaults unchanged).

VERDICT: value-only override at the single producer; no consumer breaks; default-off
byte-identical. Cross-layer-safe.

## §6. The harness gap (the reason a naive sweep would be a NULL result)

`sfo_grid_test()` (`telecom_system.cc:6325`) — the SFO-GRID coded-BER harness used for the
PCS/CFG17/turbo-EQ A/Bs — is **entirely frequency-domain**: it modulates symbols
(`psk.mod`), places them with `ofdm.framer` into a freq-domain `grid`, applies the channel
as per-subcarrier complex multiply, adds AWGN per-subcarrier (`:6878-6883`), then
estimates/equalizes/decodes. It NEVER calls `baseband_to_passband` or `peak_clip`
(grep-verified, function span 6325-...). Therefore the PAPR clip — a **time-domain**
operation on passband samples — NEVER RUNS in the SFO-GRID harness. Sweeping
`data_papr_cut` in the unmodified SFO-GRID would produce byte-identical BER for every cut.

### Fix: optional time-domain passband round-trip in the SFO-GRID harness
Add `MERCURY_SFO_GRID_PBLOOP=1` (default 0 = off = byte-identical). When on, after the grid
is modulated+framed (and BEFORE the freq-domain channel/AWGN), the harness:
1. IFFT/`baseband_to_passband` the clean grid to passband,
2. `peak_clip(pb, papr=ofdm.data_papr_cut)` — the lever under test,
3. (optional) FIR_tx1/tx2 to match the production TX spectral shaping,
4. add AWGN at passband scaled to the requested Es/N0,
5. `passband_to_baseband` + FFT back to a received grid,
6. feed that grid into the EXISTING estimate/decode/BER path.

This makes `data_papr_cut` a LIVE parameter the sweep moves, and measures the clip-distortion
BER cost + (later, Phase 2) the RX cancellation. Default-off preserves the freq-domain
harness used by every prior A/B.

NOTE on scope (from the task brief): the sim TX clips identically to HW (peak_clip runs),
so the sim CAN A/B the clip-distortion BER and the RX cancellation MECHANISM. The sim does
NOT model the HW analog/EVM floor (the ~28 dB HW-only gap). So a sim gain here is a
MECHANISM proof (clip distortion recovered / optimal cut found), NOT the HW ceiling number;
the bench validates the HW translation later.

## §7. Phase-1 deliverable
- Env tunable `MERCURY_DATA_PAPR_CUT` / `MERCURY_PREAMBLE_PAPR_CUT` (default-off byte-id).
- Passband-loopback SFO-GRID mode (`MERCURY_SFO_GRID_PBLOOP=1`) so the cut is a live param.
- CFG16 coded-BER sweep over cut ∈ {7,10,13,15,off} dB across an SNR ladder → optimal cut
  + dB shift vs 10 dB.

## §8. RESULTS — CFG16 (32-QAM r0.875) coded-BER sweep (AWGN, 8 seeds/cell)

Implementation: `MERCURY_SFO_GRID=1 _CODED=1 _PBLOOP=1 MERCURY_DATA_PAPR_CUT=<cut>
MERCURY_SFO_GRID_ESN0=<dB> -m PLOT_PASSBAND -s 16`. Data: `papr_sweep_cfg16.csv`.

frac_decoded (codewords decoded / 48 = 6 cw x 8 seeds):
```
esn0   off     15      13      10      7
14.0   0.0000  0.0000  0.0000  0.0000  0.0000
14.5   0.0000  0.0000  0.0000  0.0000  0.0000
15.0   0.3542  0.3542  0.3542  0.3542  0.2500   <- 7dB degrades
15.5   0.9375  0.9375  0.9375  0.9375  0.8542   <- 7dB degrades
16.0   1.0000  1.0000  1.0000  1.0000  1.0000
16.5   1.0000  1.0000  1.0000  1.0000  1.0000
```

### §8.1 FINDINGS
1. **Cuts {off, 15, 13, 10} are DECODE-IDENTICAL across the whole ladder** (byte-for-byte
   same frac_decoded). Only the aggressive **7 dB** cut degrades (knee 0.354→0.250 @15.0;
   0.938→0.854 @15.5; ~+0.3-0.5 dB effective-SNR LOSS, never a gain).
2. **The optimal cut at fixed peak is "10 dB or looser" — 10 dB is NOT mistuned (no shift).**
   No cut beats 10 dB; tightening below ~10 dB strictly loses. The effective-SNR optimum is
   a PLATEAU from ~10 dB upward, so 10 dB sits at the knee of the plateau (the looser edge of
   the optimum, the conservative choice).
3. **MECHANISM (why the plateau): the clip barely ENGAGES at >=10 dB on a CFG16 composite.**
   Clip-engagement estimate (frac of time samples whose |x|^2 exceeds the peak_allowed
   threshold, 2000 random 32-QAM OFDM symbols):
   ```
   PAPR_dB   frac_samples_clipped
     7        0.596%   <- the only depth that fires materially -> the only one that costs BER
    10        0.002%   <- ~inert; decode-equivalent to no-clip
    13/15/off 0.000%   <- never fires
   ```
   The realized PAPR of a ~50-carrier 32-QAM composite rarely exceeds 10 dB, so the default
   10 dB clip clips almost nothing in this sim. This is WHY 10 ≡ off in decode.

### §8.2 CONSEQUENCE FOR P1-A (honest scope)
- **Phase-1 verdict: the 10 dB default is well-chosen for CFG16 in AWGN — the re-tune buys
  nothing (no dB shift), and tightening costs.** The "10 dB may be sub-optimal" hypothesis
  (DEEPLEVER #2) is REJECTED for CFG16/AWGN at fixed peak: 10 dB is at/above the optimum.
- **Phase-2 (RX clip-noise cancellation, #1) has almost nothing to cancel at the default
  10 dB in THIS sim** — the clip is nearly inert there. The cancellation MECHANISM only has
  recoverable distortion to work on at AGGRESSIVE cuts (<=7 dB) OR in regimes where the
  realized PAPR is forced higher (more subcarriers; or — the real motivation — a HW PA that
  needs deeper backoff/tighter clipping for a fixed-peak transmit-SNR gain, the FreeDV
  "3-4 dB at fixed PEP" regime). The sim does not model the HW PA, so the sim CANNOT show
  the FreeDV fixed-PEP transmit-SNR gain; it can only show that the in-band distortion of a
  GIVEN clip depth is/ isn't recoverable. At 10 dB there is ~no in-band distortion to recover.
- **Therefore the failing-test-first baseline for Phase-2 is the 7 dB (or tighter) cut**, the
  only depth with a real, recoverable clip-distortion BER cost in the sim. Phase-2 must show
  the RX cancellation recovers the 7 dB (or tighter) penalty back toward the 10-dB/off curve.
- Translation to the HW 14.4 dB effective-SNR ceiling is BENCH-only (sim has no analog floor).

### §8.3 Knee-zoom (16 seeds, continuous metrics) — `papr_knee_cfg16.csv`
Captures mean post-FEC info BER + LDPC iter_mean per cut at {15.0, 15.25, 15.5} dB to
resolve sub-cliff differences the binary decode-count hides. Result: cuts
**{off, 15, 13, 12, 11} are BIT-FOR-BIT identical in ALL THREE metrics** (frac_decoded,
mean_BER, mean_itermean) at every SNR point — proof the clip produces the IDENTICAL channel
realization (it never fires at >=11 dB). At **10 dB** a vanishing engagement appears
(mean_BER 0.01399->0.01397 @15.0; 0.00368->0.00370 @15.25) with NO decode change — 10 dB
sits exactly at the onset of engagement, the conservative looser edge of the optimum
plateau. Engagement becomes material only at <=~8 dB (coarse: 7 dB = 0.596% clipped, the
only depth with a real decode penalty). The engagement boundary (~8-9 dB) is the
failing-test-first operating point for Phase-2.
