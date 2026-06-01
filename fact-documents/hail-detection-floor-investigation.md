# HAIL beacon-detection floor — investigation (ROBUST_0 link establishment)

**Branch:** `sim/hail-detection-floor` (off inner monitor `01535f2`)
**Created:** 2026-05-31
**Status:** SIM + CODE-READ COMPLETE. No production change. HW validation is a
separate gated phase (per task scope — no IONOS bench used here).

**Question (from the tier-2-suffix-fec finding §11):** the ROBUST_0 link
*establishment* floor is gated by **HAIL beacon detection** (fires ~−10, dead by
−12/−14 on IONOS WGN), UPSTREAM of the ctrl-suffix work. Why does HAIL die ~4 dB
shallower than the structurally-identical ctrl base-pattern matched-count
detector (−14.68 dB)? Can HAIL reach ~−14 noncoherently?

Sibling docs: [[tier2-suffix-fec-design.md]] §11 (the finding), [[../.tmp_repsim/connect-ack-metric-gate.md]]
(the ctrl metric-gate relax — the closest prior art), [[mfsk-vara-parity-plan.md]]
(the Eb/N0 framing), MEMORY Bug #55 (NB-HAIL timing race, ddf049d).

---

## §1 The detection path (code-grounded)

HAIL detection routes through the SAME matched filter as every other MFSK
control pattern:

- **Pattern (undirected, WB):** `ack_pattern_nsymb = 16` symbols (8 Welch-Costas
  tones g=6, ×2 reps), `hail_match_threshold = 7` for M=32 / **8 for M=16**
  (`mfsk.cc:341-351`). For ROBUST_0 the ctrl-MFSK instance `ack_mfsk` is **M=16**
  (verified at runtime: `[PHY] ACK pattern: 16 symbols (M=16 …)`). Identical
  length to the ACK (`mfsk.cc:229`) and CONNECT base (`mfsk.cc:398`) patterns.
- **Detector:** `detect_hail_pattern_from_passband` (`telecom_system.cc:3722`)
  → `ofdm.detect_ack_pattern` (`ofdm.cc:3691`), the discrete FFT-bin-argmax
  matched filter. Per symbol it returns a **matched COUNT** (expected tone is the
  all-streams argmax peak, energy-gated, Bug #39 carrier-image recovery) and a
  soft **metric** = Σ over matched symbols of `e_target/e_total`, where
  `e_target` = expected-bin (+mirror) energy and `e_total` = energy across the
  Nc=50 occupied subcarriers (`ofdm.cc:3808-3823`). **This is the SAME function,
  same Nc=50 denominator, the ctrl base detector uses** — so the −14.68 dB
  ctrl-base floor IS HAIL's matched-count floor.

### §1.1 The −14.68 dB figure is matched-count ONLY
The base-pattern −14.68 dB floor (`.tmp_repsim/connect-suffix-fec-research.md`
§6.1) is measured as `P(matched ≥ threshold) = 0.5` — **no metric/quality gate**.
HAIL in production ANDs soft gates on top of that count (§2). That is the whole
gap.

---

## §2 The two HAIL detection sites — and the gate inconsistency (THE ROOT CAUSE)

| Site | File:line | Gate |
|---|---|---|
| **Fast LISTENING poll** (dominant — the path the IONOS finding observed) | `arq_common.cc:5375` | `base_matched>=hail_match_threshold && suffix_ok && metric >= 3.0 && quality >= 0.3` |
| receive() path (CONNECTION_RECEIVED) | `arq_common.cc:6373` | `metric >= ack_pattern_detection_threshold && matched>=hail_match_threshold && quality >= 0.3` |

`quality = metric / matched` (`arq_common.cc:5316`).

**Finding A — the fast-poll metric gate is a HARDCODED 3.0, inconsistent with the
codebase's own ROBUST_0 tuning.** The sibling site uses
`ack_pattern_detection_threshold`, which the config sets to **0.65 at ROBUST_0**
(`telecom_system.cc:5505-5506`, comment: "ROBUST_0 (−13 dB): low SNR, need
conservative threshold" — i.e. 0.65 IS the conservative value already). The
fast-poll path ignores that and applies 3.0 — **4.6× stricter than the same
codebase deems correct for this config**, and the identical `metric>=3.0` value
the ctrl metric-gate audit (`connect-ack-metric-gate.md` §1, §7) measured as the
acquisition-cliff limiter (~−11 dB) and relaxed to 2.0 for +2.5 dB on CONNECT/ACK.
**That relax explicitly did NOT touch HAIL** (`connect-ack-metric-gate.md` §3.4:
"HAIL / BREAK … do not route through the 4 sites").

**Finding B (the bigger one) — the `quality >= 0.3` gate DOMINATES the metric
gate at the floor.** `quality = metric/matched`. At the floor the count is high
(matched=16), so `quality >= 0.3 ⟹ metric >= 4.8` — **stricter than metric>=3.0**.
So relaxing the metric threshold *alone* (the ctrl §7 fix) buys HAIL **nothing**;
the quality gate still binds. Both soft gates must move together. (MEASURED, §4.)

The `metric>=3.0` and `quality>=0.3` constants carry no measurement basis in the
code (cf. CLAUDE.md §1 — unmeasured constants; the same class as the old
`metric>=3.0` ctrl gate the audit replaced).

---

## §3 Beacon combining — sent N, detected independently (NO integration)

- **TX:** CMD sends ONE HAIL beacon per `update_status` cycle
  (`arq_commander.cc:407` `send_hail_pattern()`), increments `connection_attempts`,
  listens for a response, returns and retries up to `max_connection_attempts`
  (`arq_commander.cc:403-429`). The responder, while LISTENING, polls
  `receive_hail_pattern()` every ~32 ms (`arq_responder.cc:111-138`).
- **RX:** `receive_hail_pattern` (`arq_common.cc:5276`) snapshots the capture-buffer
  tail and runs `detect_ack_pattern` ONCE per poll. `send_hail_pattern`
  **resets the capture ring** (`circular_buf_reset` at `:5247`) so each beacon's
  energy is discarded before the next. → **Each beacon must individually clear
  the gate. ZERO noncoherent integration across beacons today.**

The task hypothesised combining N ≈ +10·log10(N) dB would be the big cheap win.
§4 shows this is **only partly true** — see §5.

---

## §4 The sim — MEASURED (in-process, AWGN + CFO, no HW)

Test `test_hail_detection_cliff_sweep` in
`source/physical_layer/mfsk_ctrl_codec_tests.cc` §11 (wired into
`mercury.exe --test`; **31/31 pass**). Mirrors the ctrl metric-gate harness
(`connect-ack-metric-gate.md` §7): synthesizes the production HAIL beacon at
passband (`build_hail_template` → `generate_hail_pattern` + `baseband_to_passband`,
i.e. the real TX path), AWGN + ±12 Hz CFO, ROBUST_0 WB (M=16). Detection via a
fixed-offset energy-combining scorer that faithfully replicates
`detect_ack_pattern`'s bin mapping (R=1-vs-production fidelity check:
**scorer matched=16/metric=13.22 vs production matched=16/metric=13.23** — exact).
SNR3k = `sigma²·3000/(fs/2)` (same calibration as the −14.68 harness).

Noncoherent combining = sum the per-tone energy matrix E[s][m] across R aligned
reps BEFORE argmax/metric (square-law). Gate variants: G0 = CURRENT (m≥3.0,
q≥0.3), G1 = METRIC-RELAX (m≥2.0, q≥0.3), G2 = BOTH-RELAX (m≥0.65, q≥0.0 ≈
matched-count-only).

**Cliffs (P=0.5, SNR3k dB; more negative = deeper):**

| Statistic / gate | R=1 | R=2 | R=3 | R=5 |
|---|---|---|---|---|
| **Matched-COUNT only** (the −14.68 target) | **−13.25** | −15.05 | −15.05 | **−16.99** |
| G0 CURRENT (m≥3.0, q≥0.3) | **−4.95** | −4.95 | −4.95 | −4.95 |
| G1 METRIC-RELAX (m≥2.0, q≥0.3) | −4.95 | −4.95 | −4.95 | −4.95 |
| G2 BOTH-RELAX (m≥0.65, q≥0.0) | **−13.25** | −15.05 | −15.05 | **−16.99** |

**Headline deltas (R=1):**
- metric-thr 3.0→2.0 ALONE (quality kept) = **+0.00 dB** — the quality gate masks it (Finding B, MEASURED).
- BOTH soft gates relaxed = **+8.30 dB** (−4.95 → −13.25): recovers the entire matched-count floor.
- Combining on the matched-COUNT: R=2 **+1.80 dB**, R=5 **+3.74 dB** (vs 10log10 ideal +3.0/+7.0 — this is the realistic M=16 noncoherent gain at q≈0.1-0.24, the number the §10 open-question (b) asked for).
- Combining on the CURRENT gate: **+0.00 dB** — the energy-RATIO metric is scale-invariant to summing (the R cancels in e_target/e_total), so summing energies cannot help a normalized-ratio gate.
- Combining + both-relax together: R=5 reaches **−16.99 dB** (the levers stack: relax exposes the count floor, combining then deepens it).

**FAR (pure noise, 5000 trials, at the −15 dB deep-floor level):**

| gate | R=1 | R=2 | R=3 | R=5 |
|---|---|---|---|---|
| G0 CURRENT | 0/5000 | 0/5000 | 0/5000 | 0/5000 |
| G1 METRIC-RELAX | 0/5000 | 0/5000 | 0/5000 | 0/5000 |
| G2 BOTH-RELAX (count-only, 8/16) | 0/5000 | 1/5000 | 1/5000 | 0/5000 |

⇒ the **count gate (8/16) is the load-bearing FAR defense**; the soft metric/
quality gates contribute ≈ 0 FAR. Relaxing them is FAR-safe — same conclusion the
ctrl audit reached for the metric gate (`connect-ack-metric-gate.md` §6, 0/5000),
now confirmed for HAIL incl. the count-only case.

### §4.1 Caveats (absolute-dB vs the IONOS finding)
The sim's CURRENT-gate cliff (−4.95 dB SNR3k) is *shallower* than the IONOS
finding's "fires ~−10". Two reasons, both leaving the RELATIVE structure robust:
(1) the finding's "−10/−11" is the **IONOS WGN dial**, not calibrated SNR3k — the
testbed gain structure is not 1:1 with SNR3k. (2) the sim is **single-shot P=0.5,
AWGN-only**; production gets thousands of polls over the listen window, so its
"sometimes fires" floor is deeper than single-shot P=0.5. The
**calibration-independent finding** — the soft-gate cliff is ~8 dB shallower than
the matched-count floor, and the quality gate is the dominant limiter — is what
drives the recommendations and is invariant to the absolute axis.

---

## §5 WHY HAIL dies shallow (the answer)

NOT the pattern (16 sym, identical to ctrl base), NOT the threshold (count
7-8/16, identical), NOT the detector (same `detect_ack_pattern`), NOT hard-vs-soft
(it computes both). HAIL's **matched-count statistic reaches the same −13 dB
floor as the ctrl base detector** (§4). The ~8 dB gap is **the two soft gates ANDed
on top in the fast-poll path** (`arq_common.cc:5375`): `metric>=3.0` (the same
unmeasured constant the ctrl audit relaxed but left in HAIL) and — dominant —
`quality>=0.3`, which at high matched-count is the stricter `metric>=4.8`. The
ctrl base detector reaches −14.68 because it is gated on the COUNT alone; HAIL is
additionally gated on a normalized energy-RATIO that collapses to the 2/Nc noise
floor ~8 dB above the count floor.

**Does HAIL reach ~−14 noncoherently?** YES — but via gate relaxation (+8.3 dB to
the count floor at −13.25), NOT primarily via beacon-combining. Combining is real
on the count statistic (+1.8/+3.7 dB at R=2/5) and stacks on top once the ratio
gate is removed (R=5 → −16.99), but combining ALONE under the current gate buys
0 dB because the ratio metric is scale-invariant.

---

## §6 Ranked HAIL fix options (each: dB toward −14, scope, FAR, prior art)

All are RX-detection-only, throughput-neutral (no TX/wire/symbol-rate change),
except R3/R4 which add a CONNECT wire-length change. Present to user before any
production change; HW-validate on IONOS (gated phase).

1. **Align the fast-poll gate with the config-tuned threshold + drop/loosen the
   quality gate.** Replace the hardcoded `metric>=3.0` at `arq_common.cc:5375`
   with `ack_pattern_detection_threshold` (0.65 at ROBUST_0, already the
   "conservative" value per `telecom_system.cc:5505`) AND loosen `quality>=0.3`
   to e.g. `>=0.1` (or gate quality only when matched is LOW, where it actually
   defends — it's pointless when matched=16). **dB: ≈ +8 dB toward the count
   floor (−5 → ~−13).** Scope: 1-2 lines + a named constant, mirrors the ctrl
   metric-gate fix exactly. FAR: 0/5000 measured (§4). Prior art:
   `connect-ack-metric-gate.md` (same detector, 5000-trial FAR sweep, shipped
   sub-3.0 since 7076a4b); CLAUDE.md §1 (removing unmeasured constants). **THE
   cheap win — do this first.** *Cross-layer note:* the gate also fronts the
   directed-HAIL `suffix_ok` (`arq_common.cc:5312-5313`); relaxing the BASE soft
   gate does not touch the suffix-match logic, but a full audit of the directed
   path is required before ship (CLAUDE.md §5).

2. **Make the gate's metric the per-match QUALITY of matched symbols only, or
   replace the ratio gate with an absolute-energy gate** (so combining can help).
   Deeper redesign of the soft criterion; lets lever 3 add gain. dB: enables R3.
   Scope: medium (touch `detect_ack_pattern`'s metric semantics — cross-layer,
   shared by ACK/CONNECT/BREAK; full §5 audit required). FAR: re-measure.

3. **Noncoherent beacon energy-combining (R=2..5).** Accumulate E[s][m] across the
   N beacons the CMD already sends (today wasted, §3) before argmax. dB: **+1.8
   (R=2) / +3.7 (R=5)** on the COUNT statistic — but ONLY realizable on the link
   floor if lever 1/2 has removed the ratio gate (combining alone = 0 dB under the
   current gate, §4). Scope: larger — needs the RX to retain/sum per-beacon
   energy across polls instead of resetting the ring (`arq_common.cc:5247`), a
   CONNECT-only adaptive R, CAP-gated wire change if the beacon cadence changes;
   watch the Bug #55 NB-HAIL PTT/timing race. FAR: 0-1/5000 at R=2/3 (§4). Prior
   art: WSJT-X Q65 noncoherent integration (~2.6-3.0 dB/doubling — matches
   measured), `tier2-suffix-fec-design.md` §10 lever 3.

4. **Longer / repeated base pattern (more symbols per beacon).** Orthogonal to
   combining; raises per-beacon processing gain. dB: ~+3 dB per doubling but costs
   airtime + a wire change; LOWER priority than 1+3. (N=32 preamble was tried and
   REVERTED for data — MEMORY 738e049 — but that was the data preamble on a
   bps-limited floor; HAIL is establishment-only so the airtime tradeoff differs.)

**Recommended sequence:** lever 1 (free, +8 dB, reaches ~−13 ≈ the ctrl-base /
−14 target) → then lever 2+3 together if the residual to −14/−17 is wanted. Lever
1 alone likely moves the establishment floor PAST the IONOS finding's −10/−11 to
the ctrl base-detector floor — i.e. HAIL would no longer be the binding stage, the
ctrl-suffix work (tier2) becomes the next limiter as intended.

---

## §7 Open questions [?]
- [?] **Absolute-dB on HW.** The sim axis is SNR3k; the IONOS finding is a WGN
  dial. HW A/B (gated phase) must confirm lever 1 moves the *establishment* floor
  the predicted ~+8 dB and that FAR stays ~0 on real HF noise (watch for spurious
  `[HAIL] Detected` on noise/other-traffic).
- [?] **Directed-HAIL suffix interaction.** Lever 1 relaxes the BASE soft gate;
  the directed suffix (`suffix_matched`, FNV callsign tones) has its own
  `suffix_ok` check (`arq_common.cc:5312-5313`). Audit before ship (CLAUDE.md §5).
- [?] **Quality gate's real purpose.** It defends against the case matched is
  moderate but energy is diffuse. Gating quality ONLY when matched < (say) 12
  would keep that defense while removing the floor penalty — measure FAR for that
  variant.
- [?] **Multipath.** Sim is AWGN+CFO. Frequency-selective fading on real HF could
  depress the metric further (helping the relax case) or move the count floor.

## §8 Key file references
- HAIL pattern + thresholds: `mfsk.cc:332-378` (tones, `hail_match_threshold`),
  `:436-482` (`set_hail_target`/`clear_hail_target`, `hail_detect_*`),
  `:569-591` (`generate_hail_pattern`).
- Detector: `telecom_system.cc:3722` (`detect_hail_pattern_from_passband`),
  `ofdm.cc:3691` (`detect_ack_pattern` — the shared matched filter).
- HAIL gates: `arq_common.cc:5375` (fast-poll, hardcoded 3.0 + quality 0.3),
  `:6373` (receive() path, `ack_pattern_detection_threshold` + quality 0.3),
  `:5316` (`quality = metric/matched`).
- Config-tuned threshold: `telecom_system.cc:5505-5510`
  (`ack_pattern_detection_threshold` = 0.65 ROBUST_0).
- Beacon TX/RX loop: `arq_commander.cc:403-429` (CMD send+listen),
  `arq_responder.cc:111-138` (RSP poll), `arq_common.cc:5170-5273`
  (`send_hail_pattern`, ring reset at :5247), `:5276` (`receive_hail_pattern`).
- Sim: `mfsk_ctrl_codec_tests.cc` §11 (`build_hail_template`,
  `hail_score_combined`, `test_hail_detection_cliff_sweep`).
- Prior art: `.tmp_repsim/connect-ack-metric-gate.md` (the ctrl metric-gate relax
  + FAR sweep), `tier2-suffix-fec-design.md` §10-§11 (noncoherent levers + the
  finding).
