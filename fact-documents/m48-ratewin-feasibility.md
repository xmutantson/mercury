# WIN-CAMPAIGN INCR-1 — M48×1 single-stream "Mode B" rate-win make-or-break (sim BER)

**Status:** MEASUREMENT COMPLETE — **FAIL (double).** The −10 rate-win via single-stream
higher-order MFSK + a rate-½ code is **REFUTED at the PHY layer on two independent counts**:
(1) the literal spec (M48×1 + GF16-RA) is **architecturally impossible**; (2) the steelman
(M48×1 / M32×1 single-stream + LDPC r½) **fails BOTH gate criteria** — net bps 90.32
(< 143 bar) AND AWGN cliff −7.0 / −8.5 (> −10 bar).

**Date:** 2026-06-02. **Vehicle:** worktree `sim/m48-ratewin @` (this branch), off
`monitor @8fc1211` (the **inner** `mercury/` git repo; `mercury/` is gitignored in the
outer tools/docs repo — that is why `8fc1211` "did not exist" when queried from the outer
repo). **Build:** `bash build.sh o3`; **`mercury --test` = 50 passed / 0 failed** with the
SIM knob OFF (byte-identical regression guard). SIM-only on the dev host (NO bench — v13
Q-table cal owns the IONOS). No production config changed; all changes env-gated.
**Dispatch:** WIN CAMPAIGN INCR-1 (parent agent), the make-or-break −10 rate-win gate.
**Conventions:** numbered sections (§N), `file:line` citations, `[?]` for unknowns.

---

## §1. The gate (PASS/FAIL bar, from the dispatch)

> PASS = AWGN PHY cliff **≤ −10 dB SNR3k** at **≥ 143 net bps**. FAIL = cliff > −10 OR
> < 143 bps → STOP + audit which gap-term (A geometry / B code / C demap / D rate)
> underdelivered.

Fail-before baseline = **ROBUST_3** (M16×2, LDPC rate 8/16 = ½): measured cliff **−7.1 dB
SNR3k @ 140 bps net** (`robust3-mfsk-demap-feasibility.md:54,105`). The bet: switching the
M16×2 *diversity* geometry to M48×1 *single-stream* recovers ~2.5–3.5 dB (cliff −7.1 →
≤ −10) while preserving ~143–230 bps.

---

## §2. RESULT — double FAIL (decision-grade)

`mercury -m PLOT_PASSBAND -s 102 -R --ber-esn0=<S> --ber-frames=<N> -n`, SNR3k axis
(`bandwidth = 2343.75 Hz` held, see §4), SIM-only AWGN, points parallelized across cores.
Deepest SNR with BER→0 = the cliff. Fine pass = 0.5 dB / 60 frames; coarse = 1 dB / 30.

| Geometry | code rate | **net bps** | **cliff (SNR3k)** | ≥143 bps? | ≤−10 dB? |
|---|---|---:|---:|:---:|:---:|
| **M48×1** (literal spec) | ½ | **90.32** | **−7.0** | **NO (−37%)** | **NO** |
| **M32×1** (fair higher-order) | ½ | **90.32** | **−8.5** | **NO (−37%)** | **NO** |
| M16×2 r¼ (ROBUST_2, same-binary anchor) | ¼ | 68.82 | −8.0 | — | reconfirms `robust2-minus10-rate-win.md` |
| M16×2 r½ (ROBUST_3, doc anchor) | ½ | 140 | −7.1 | at bar | NO |

**Coarse waterfalls (1 dB / 30 frames):**

| SNR3k | −5 | −6 | −7 | −8 | −9 | −10 | −11 | −12 | −13 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| **M48×1 r½** BER | 0 | 0 | **0** | .079 | .178 | .237 | .285 | .330 | .370 |
| **M32×1 r½** BER | 0 | 0 | 0 | **0** | .027 | .165 | .235 | .284 | .332 |

**Fine cliff localization (0.5 dB / 60 frames):**
- M48×1 r½: −6.5→0, −7.0→0, **−7.5→0.0107**, −8.0→0.0686 ⇒ **cliff −7.0**.
- M32×1 r½: −7.5→0, −8.0→0, **−8.5→0**, −9.0→0.0183 ⇒ **cliff −8.5**.
- M16×2 r¼: −7→0, −8→0, **−8.5→0.0278**, −9→0.128 ⇒ cliff −8.0 (same-binary reconfirm).

**Verdict: FAIL on BOTH axes, both geometries.** The single-stream geometry neither
reaches the bps bar nor reaches the −10 cliff. The dB recovered vs the ROBUST_3 −7.1
baseline is only **+0.0 dB (M48×1)** / **+1.4 dB (M32×1)** — not the predicted +2.5–3.5,
and the rate is **0.65×** the baseline (90 vs 140 bps), not preserved.

---

## §3. Gap-term audit — which term underdelivered (dispatch requirement)

The dispatch's decomposition: A geometry, B code, C demap, D rate. **A and D both fail,
mechanistically, and they are coupled.**

### Gap-term D (rate / net bps) — FAILS FIRST, structurally, before any cliff
The net bps is set by **bits/symbol-time × symbol-rate × code-rate**, where
bits/symbol-time = `mfsk.bits_per_symbol() × nStreams` (`telecom_system.cc:567,2345`;
`rbc = rb·LDPC_real_CR`, `:3060`):
- **M16×2 (diversity):** 2 streams × log2(16)=4 = **8 bits/symbol-time** → 140 bps @ r½.
- **M48×1 (single-stream):** 1 stream × **5** bits/symbol-time → **90.32 bps @ r½**.

The killer: **M=48 is not a power of two.** `cl_mfsk::init` computes
`nBits = floor(log2 M)` via `while(temp>1) temp>>=1` (`mfsk.cc:98–104`), so M=48 →
**nBits=5, M_eff = 1<<5 = 32** (`telecom_system.cc:4406`). The one-hot
modulator/demodulator map only 5 bits onto 32 of the 48 tones; **tones 32..47 are never
used for data** (confirmed at runtime: `[PHY] MFSK: M=48 nStreams=1 bps=5`). So M48×1 is
**strictly dominated by M32×1** — same 5 bits/symbol, but M48 adds 16 noise-only argmax
competitors that *raise* the noncoherent error floor (which is exactly why its cliff is
1.5 dB *shallower* than M32×1's: −7.0 vs −8.5).

Going single-stream **throws away 3 of 8 bits/symbol-time.** To recover ≥143 bps at
single-stream r½ you need bits/symbol ≈ 8 → **M = 256**, which cannot fit the Nc=50 grid
(`stream_offsets`: 256 tones ≫ 50 bins, `mfsk.cc:120–124`; `bandwidth = 48000·Nc/Nfft/
interp`, `telecom_system.cc:4217`). **There is no single-stream M on the Nc=50 grid that
both fits and hits 143 net bps at rate ½.** D is unsatisfiable by construction.

### Gap-term A (geometry: diversity → single-stream) — FAILS the dB premise
The decomposition assumed M16×2→M48×1 *recovers* 2.5–3.5 dB. Measured: the single-stream
higher-order geometry buys **+1.4 dB at most** (M32×1 −8.5 vs ROBUST_3 M16×2 r½ −7.1),
and the literal M48×1 buys **0.0 dB** (−7.0 vs −7.1). The premise that diversity "spends
the budget on REACH" and single-stream redirects it to RATE is **inverted at this
operating point**: 2-stream noncoherent reception is a low-SNR *gain* (independent square-
law replicas), and higher modulation order at fixed rate gives diminishing Eb/N0 benefit
on the noncoherent square-law floor (the same flattening `robust2-minus10-rate-win.md §2`
measured for the rate lever: ~1.7 dB per rate-halving, not the ~3 dB capacity formula).
The +1.4 dB M32×1 *does* buy comes with the −0.65× rate hit (90 vs 140 bps) — a strictly
worse rate-vs-reach trade than the M16×2 baseline, not a "rate win."

### Gap-term B (the GF16-RA "near-capacity data code") — ARCHITECTURALLY IMPOSSIBLE
The dispatch names `mfsk_ctrl_codec.cc`'s GF16-RA as the data code at rate ~0.5. It cannot
serve that role on two independent, code-grounded counts:
1. **Hardwired to GF(16) / M=16.** `GF16RA_M = 16` (`mfsk_ctrl_codec.h:215`); the field,
   the Walsh–Hadamard fast-GF-convolution (`fwht16`, `mfsk_ctrl_codec.cc:384`), the
   per-tone intrinsic table `pix[N*16]`, and `g_gfexp[16]` are all **16-ary by
   construction**. A GF(16) RA code demands exactly 16 tones per symbol — **it cannot
   operate on an M=48 constellation.** "M48 + GF(16)-RA" is a contradiction (the code
   alphabet *is* the modulation order).
2. **Fixed 52-bit, CRC-gated, control-frame codec — not a variable-length data FEC.**
   `GF16RA_K = 13` info symbols (10 message + 3 CRC) = a **fixed 40-bit payload + 12-bit
   CRC** (`mfsk_ctrl_codec.h:217–219`; `msg_to_info`/`encode`, `.cc:528–561`).
   `configure(repfact)` only changes the repetition factor (`.cc:427–437`), never K.
   Codeword length is hard-capped at **N ≤ 64** (`GF16RA_MAX_N = 64`, `.h:220`).
   `soft_decode` returns a single `out_payload38` per call (`.cc:571,699`). It is the
   START_CONN / TEST_ACK / TEST_CONN ctrl-suffix codec (40 useful bits/codeword), not a
   data block code. Wiring it as the data FEC would be a multi-week generalization
   (generic K, generic GF order), not a SIM knob.

Therefore the **only coherent reading** of "Mode B" is single-stream higher-order MFSK +
the **LDPC** data code (the actual variable-length near-capacity code on the data path,
`ldpc.cc:127–261`, K∈{100..1400}=rates 1/16..14/16). That is what §2 measured. Gap-term C
(demap) was not the bottleneck and was not separately swept — A and D fail upstream of it,
and `robust3-mfsk-demap-feasibility.md §4–§5` already showed Bessel-I0/BP+OSD/iter-cap buy
≈0/negative on the M16×2 r½ cliff.

---

## §4. Method / Phase-1 evidence (executing-code, pinned before measuring)

- **Harness/axis** (`telecom_system.cc:365–422`, `passband_test_EsN0`): for MFSK the
  `--ber-esn0` value is **channel SNR (dB) referenced to `bandwidth`**; σ calibrated from
  measured TX power, `sigma = sqrt(2·P_sig·f_nyquist/(SNR_lin·bandwidth))` (`:420`).
- **No bandwidth pinning needed** (unlike the baud-scaling spike): `bandwidth =
  48000·Nc/Nfft/interp` (`:4217`) with **Nc=50 fixed for all WB configs**
  (`:4208`) regardless of how many of the 50 subcarriers MFSK fills. M48×1, M32×1, and
  M16×2 all run **`Bandwidth: 2343.750000 Hz`** (confirmed at runtime) ⇒ the channel-SNR
  axis IS SNR3k, directly comparable to the ROBUST_0 −13 / ROBUST_2 −8 / ROBUST_3 −7.1
  anchors.
- **net bps = `telecom_system.rbc = rb · LDPC_real_CR`** (`:3060`), the post-FEC user data
  rate; printed as `Bitrate:` (`main.cc:2617`). 90.32 bps reflects bps=5 + rate ½.
- **M/nStreams set per-config** at `:5449–5456` and `:5497–5504` (ROBUST_0 = M32×1; all
  other robust = M16×2). LDPC K = N·rate selected in `ldpc.init()` by K
  (`ldpc.cc:127–261`); K=800 (rate ½) and K=400 (rate ¼) matrices both exist on monitor.

### The SIM knob (env-gated, byte-identical when OFF; `mercury --test` 50/0)
Three edits in `load_configuration`, all behind `getenv("MERCURY_RATEWIN_M48")`:
- after the ROBUST config block (`:~5170`): `MERCURY_RATEWIN_RATE=<n>` → `_ldpc_rate=n/16`.
- at both `mfsk.init` sites (`:~5456`, `:~5504`): force `mfsk_M = atoi(MERCURY_RATEWIN_M)`
  (default 48), `nStreams = 1`, WB only.
Default unset ⇒ no config changed (the safety gate). M48 init is crash-safe: the generic
`else` preamble/tone branches handle non-{32,16,8,4} M (`mfsk.cc:195–201`); fixed arrays
(`preamble_tones[16]`, `ack_tones[48]`) are not M-indexed.

---

## §5. Recommendation (the honest answer to the gate)

**STOP — the −10 rate win on the noncoherent single-stream MFSK PHY is structurally
unreachable.** Both failing gap-terms are *constructive impossibilities*, not tuning gaps:
- **D:** no single-stream M on Nc=50 reaches 143 net bps at rate ½ (need M≈256, doesn't
  fit). The diversity geometry's 8 bits/symbol-time is *why* it hits 140 bps; abandoning it
  for single-stream forfeits the rate.
- **A:** single-stream higher-order buys at most +1.4 dB (M32×1), far short of the +2.5–3.5
  premise, and only by trading away 35% of the rate. The cliff stays at −7…−8.5, not −10.
- **B:** GF16-RA literally cannot be the data code (GF(16)/M=16-locked, fixed-52-bit
  ctrl codec). The premise rests on a codec that doesn't generalize.

This converges with the two sibling INCR-1 measurements: ROBUST_2 (M16×2 r¼) cliffs −8.0
(`robust2-minus10-rate-win.md`), ROBUST_3 (M16×2 r½) cliffs −7.1
(`robust3-mfsk-demap-feasibility.md`). **The entire noncoherent square-law M-FSK family —
any rate, any single/dual-stream order that fits Nc=50 — sits at a −7…−8.5 dB cliff and
~70–140 net bps.** Reaching −10 at ≥143 bps simultaneously requires *either* (a) a
**coherent** narrow tier (parity plan E-1 / Phase 4, SHELVED by user 2026-05-31 — the only
path to deeper reach at this rate class), *or* (b) accepting the time-bandwidth lever
(baud-scaling, `baud-scaling-spike.md`: +3 dB/2× but **halves the rate** → −10/~45 bps, a
re-tuned deep-reach rung, NOT a rate win). Both are architectural forks the dispatch's
"INCR-2/3/4 on PASS" path does not cover. **Surface to the user; do NOT greenlight INCR-2.**

---

## §6. Reproduction
```
# inner mercury repo (mercury/ is gitignored in the outer repo):
git worktree add -b sim/m48-ratewin <path> 8fc1211   # off monitor @8fc1211
cd <path> && bash build.sh o3 && ./mercury.exe --test   # 50/0 (knob OFF)
# M48x1 + LDPC r1/2 cliff:
for s in -6 -7 -7.5 -8 -9 -10; do MERCURY_RATEWIN_M48=1 MERCURY_RATEWIN_RATE=8 \
  ./mercury.exe -m PLOT_PASSBAND -s 102 -R --ber-esn0=$s --ber-frames=60 -n 2>/dev/null \
  | grep -E '^-?[0-9.]+;'; done            # cliff -7.0, banner net 90.32 bps, bps=5
# M32x1 (fair higher-order): add MERCURY_RATEWIN_M=32  -> cliff -8.5, 90.32 bps
# M16x2 r1/4 anchor: no env vars                        -> cliff -8.0, 68.82 bps
```
