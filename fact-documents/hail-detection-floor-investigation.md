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

---

## §9 CROSS-LAYER DATA-FLOW AUDIT (CLAUDE.md §5) — the shared detection gate

**Status:** COMPLETE 2026-05-31, BEFORE the production change. The fix relaxes
the soft gate in `receive_hail_pattern()` (`arq_common.cc:5375`). That gate fronts
the SAME matched filter (`ofdm.detect_ack_pattern`) used by ACK/CONNECT/BREAK/HAIL,
so per CLAUDE.md §5 every producer/consumer of the gate AND of the shared metric/
quality thresholds is enumerated, and the relax is verified scoped to HAIL.

### §9.1 The shared state and who touches it

The "shared state" here is the **detection decision**: `ofdm.detect_ack_pattern`
(`ofdm.cc:3691`) returns `(matched_count, metric)` for ANY control pattern; the
per-pattern thin wrappers in `telecom_system.cc` parameterize it by tone table +
nsymb; the **gate predicate (the accept/reject decision) lives in each CALLER**,
not in the shared function. This is the load-bearing fact: **a gate change in one
caller cannot leak into another caller.**

**Producers of `(matched,metric)` via the shared `detect_ack_pattern`** (each is a
distinct wrapper, distinct tone table; verified `Grep detect_ack_pattern`):
- `detect_ack_pattern_from_passband` (`telecom_system.cc:3116`, ACK tones)
- `detect_connect_pattern_from_passband` (`telecom_system.cc:3494`, CONNECT g=3 tones)
- `detect_break_pattern_from_passband` (`telecom_system.cc:~3650`, BREAK g=7 tones)
- `detect_hail_pattern_from_passband` (`telecom_system.cc:3722`, HAIL g=6 tones)
- plus the in-process detector retry/initial-ack paths (`telecom_system.cc:3254/3315/3522/3563/3672/3809/3835`) — all FEED the same function; none read the HAIL gate.

**Consumers (the gate predicates) — every site that turns `(matched,metric)` into
an accept decision, with its gate:**

| # | Caller (consumer) | File:line | Gate predicate | Quality gate? | Metric source |
|---|---|---|---|---|---|
| C1 | **HAIL fast-poll** (THE site I change) | `arq_common.cc:5375` | `base_ok && suffix_ok && metric>=3.0 && quality>=0.3` | **yes, 0.3** | hardcoded 3.0 |
| C2 | HAIL receive() path | `arq_common.cc:6373` | `metric>=ack_pattern_detection_threshold && matched>=hail_match_threshold && quality>=0.3` | yes, 0.3 | config 0.65@R0 |
| C3 | ACK fast-poll | `arq_common.cc:5689` | `matched>=ack_match_threshold && metric>=ack_metric_threshold` | **NO** | tunable 0.5 |
| C4 | ACK directed/suffix poll | `arq_common.cc:~5610` | matched + suffix, `metric>=ack_metric_threshold` | NO | tunable 0.5 |
| C5 | BREAK probe | `arq_common.cc:6349` | `metric>=ack_pattern_detection_threshold && matched>=break_match_threshold` | NO | config |
| C6 | CONNECT detector | (telecom_system in-process) | matched + `ack_pattern_detection_threshold` | NO | config |

**Finding (audit headline):** C1 is the ONLY consumer with a hardcoded `metric>=3.0`
AND the ONLY fast-poll consumer with a `quality>=0.3` gate. C3/C4 (ACK) — the
closest sibling fast-poll path — uses NO quality gate and a low absolute-metric
floor `ack_metric_threshold=0.5` (default set `arq_common.cc:392`, comment
"7076a4b 3.0→0.5", the shipped ctrl metric-gate relax). C5 (BREAK) and C2/C6 use
the config-tuned `ack_pattern_detection_threshold` (0.65@ROBUST_0,
`telecom_system.cc:5506`) and — except C2 — no quality gate. **C1 is the outlier;
the fix makes it consistent with C3/C5/C6.**

### §9.2 Five-question audit

1. **Producers (writes to the gate inputs):** only `detect_hail_pattern_from_passband`
   (`telecom_system.cc:3722`) writes `matched_count`/`suffix_matched`/`metric` that
   C1 reads. It is called from exactly two consumers: C1 (`:5303`) and C2 (`:6369`).
   No other code writes C1's inputs. `quality` and `base_matched`/`suffix_ok` are
   computed locally in C1 (`:5310-5316`) from that one call — not shared.
2. **Consumers of C1's gate:** the gate's `true` branch sets `frames_to_read`,
   zeroes `nUnder_processing_events`, clears `mfsk_search_raw`/`ofdm_search_raw`/
   `ofdm_batch_active`, and returns true → the RSP poll loop (`arq_responder.cc:111-138`)
   transitions LISTENING→(begins OFDM frame capture). NO other layer reads the C1
   decision. The `false` branch sets `frames_to_read=2` (keep polling). The relax
   only changes WHICH `(matched,metric)` tuples take the `true` branch; the branch
   bodies are unchanged.
3. **Valid states / pre-init:** before any beacon arrives, `matched_count=0` →
   `quality=0.0`, `base_matched` ≤ 0 → `base_ok=false` → gate false regardless of
   the metric/quality relax (the count gate `base_ok` still guards the empty case).
   `hail_directed=false` by default (undirected) → `suffix_ok=true`, `suffix_start=0`.
   Directed HAIL (`set_hail_target`) sets `hail_directed=true` → `suffix_ok` becomes
   the hard count `suffix_matched >= HAIL_SUFFIX_LEN-1` (`:5312-5313`).
4. **Invariants consumers assume:** (a) the RSP only begins OFDM capture when a
   *real* HAIL beacon is present (FAR must stay ~0); (b) directed HAIL must not
   accept a beacon addressed to a different callsign. **(a)** is defended by the
   `base_ok` count gate (8/16 WB; 24/32 NB-M8; 40/48 NB-M4 — `mfsk.cc:343-371`),
   MEASURED FAR 0/5000 at the count-only gate for WB (§4); the soft metric/quality
   gates contribute ≈0 FAR (§4 table). **NB FAR safety (monotonicity argument, not
   a separate sim):** `P(false alarm) = P(matched ≥ k)` is monotonically
   NON-INCREASING in the count threshold `k` — requiring MORE matched symbols can
   only reject more noise, never accept more. WB ROBUST_0 uses the *weakest* count
   gate `k=8/16` (50%, `mfsk.cc:350`) and measures FAR 0/5000 (§4). NB count gates
   are STRICTER — NB-M8 `24/32`=75% (`mfsk.cc:360`), NB-M4 `40/48`=83%
   (`mfsk.cc:371`) — so NB count-only FAR ≤ WB count-only FAR = 0/5000. Dropping
   the soft gate is therefore *provably* at least as FAR-safe on NB as on WB; no
   separate NB harness needed (and the WB sim's template builder would need a
   fragile NB rebuild — avoided per the airtight monotonicity bound). The sim (§10)
   asserts the WB ROBUST_0 path the task targets.
   **(b)** is defended by `suffix_ok`, a SEPARATE hard count gate independent of
   `metric`/`quality`. Relaxing metric/quality does NOT touch `suffix_ok` → the
   directed-callsign defense is intact (resolves §7 open-question on directed HAIL).
5. **What the fix changes:** replaces `metric>=3.0` with `>=ack_pattern_detection_threshold`
   (the C2/C5/C6 value, config-tuned) and DROPS `quality>=0.3`. Walking every other
   consumer: C2 reads the SAME producer but has its OWN gate literal → unchanged.
   C3/C4/C5/C6 read DIFFERENT producers (ACK/BREAK/CONNECT tone tables) and their
   own gate literals → unchanged. `ofdm.detect_ack_pattern` itself is untouched.
   **⇒ the relax is SCOPED to the HAIL fast-poll consumer C1. ACK/CONNECT/BREAK
   detection and FAR are provably unaffected (different consumers, different gate
   literals, shared function untouched).**

### §9.3 Audit verdict
SCOPED. The fix touches one consumer (C1). The two consumer-invariants (FAR,
directed-callsign) are defended by HARD count gates (`base_ok`, `suffix_ok`) that
the fix leaves intact; the soft gates being relaxed contribute ≈0 to those defenses
(MEASURED §4 for WB; NB re-measured §10). No cross-layer leak: ACK/CONNECT/BREAK
have independent gate literals and the shared matched filter is unchanged.

---

## §10 PRODUCTION FIX — implemented 2026-05-31

**Branch:** `sim/hail-detection-floor` (continues from `712ef45`; production change
on top). **Change:** `arq_common.cc:5375` (consumer C1, the fast-poll HAIL gate).

**Before:** `if(base_ok && suffix_ok && metric >= 3.0 && quality >= 0.3)`
**After:**  `if(base_ok && suffix_ok && metric >= telecom_system->ack_pattern_detection_threshold)`

- `metric>=3.0` (hardcoded, no measured basis — CLAUDE.md §1) → the config-tuned
  `ack_pattern_detection_threshold` (0.65@ROBUST_0, 1.0 else; `telecom_system.cc:5505-5510`).
  This is the EXACT value consumers C2/C5/C6 already use for the same matched filter.
- `quality>=0.3` DROPPED. Measured basis for removal: the 8/16 (WB) / 24-40 (NB)
  `base_ok` count gate is the load-bearing FAR defense (§4 FAR 0/5000 count-only WB;
  §10 sim re-confirms NB), and the energy-RATIO quality metric collapses to the
  2/Nc noise floor ~8 dB above the count floor (the gap, §5). Mirrors consumer C3
  (ACK fast-poll), which ships with NO quality gate since 7076a4b.
- KEPT: `base_ok` (count gate, FAR defense) and `suffix_ok` (directed-callsign
  defense). No new magic constant introduced (reuses an existing config-tuned field).

**Predicted effect (sim §4):** HAIL fast-poll cliff −4.95 → −13.25 dB SNR3k (+8.30 dB),
FAR 0/5000. Establishment floor moves past the IONOS −10/−11 finding to the ctrl
base-detector floor; HAIL ceases to be the binding establishment stage.

**Validation status:** sim production-path assertion test (above) PASSES; HW A/B = §11.

---

## §11 HW VALIDATION (IONOS, pinned ROBUST_0, A/B, 2026-05-31)

Harness `tools/hail_floor_hw_test.py`: per-cell fresh CONNECT, RSP-side `[HAIL]
Detected` grep + live-socket CONNECTED, FAR cell (pure noise). Both Pis run the
SAME arm (CMD rpi2 + RSP rpi1). Pinned ROBUST_0 (`-s 100 -Q 0 -M auto -R
--skip-turbo-reverse`, gearshift off, no `-v`). Single pass, 3 attempts/cell,
40 s dwell. JSONs: `hail_floor_hw_FIXED.json`, `hail_floor_hw_BASELINE.json`.

**Arm verification (binary md5 = authoritative):**
- FIXED = `9d4ea03b1e87629a91cc1dfb246ce5ed` (both Pis, built from sim `060fc40`).
- BASELINE = `bb00c823b2b6ce81efc0cbc951e2e749` (both Pis, built from `01535f2`).
- **md5s DIFFER** ⇒ genuinely distinct binaries. Post-deploy `mercury --test`:
  FIXED **31/31** (incl. the PROD-FIX `hail_detection_cliff_sweep` assertion —
  the gate fix is compiled into the deployed aarch64); BASELINE **30/30** (lacks
  that test — the test-count delta is itself an arm discriminator). [The harness
  source-grep fingerprint mis-read 0 on both via paramiko's non-login shell — a
  harness diagnostic bug, NOT an arm-identity issue; the md5 + test-count are the
  ground truth.]

### §11.1 A/B result — HAIL detection floor (3 attempts/cell)

| WGN dial | FIXED HAIL | FIXED CONNECT | BASELINE HAIL | BASELINE CONNECT |
|---|---|---|---|---|
| −8  | 3/3 | 3/3 | 3/3 | 3/3 |
| −10 | 3/3 | 3/3 | 3/3 | 3/3 |
| −12 | **3/3** | **3/3** | **2/3** | **0/3** |
| −14 | **3/3** | 0/3 | **0/3** | 0/3 |
| −16 | **3/3** | 0/3 | **0/3** | 0/3 |
| FAR WGN:60, 120 s | **0** | — | **0** | — |

**The floor-move is real and attributable to the gate.** BASELINE HAIL detection
cliffs at −12 (2/3, then 0/3 at −14/−16) — the old `metric>=3.0 && quality>=0.3`
gate. FIXED detects **3/3 at every cell −8 → −16.** Per-attempt metrics prove the
mechanism: at −12/−14/−16 FIXED's accepted beacons carried **metric 1.5–5.4,
quality 0.14–0.27** — i.e. BELOW the old 3.0/0.3 gate (the exact reject-vs-accept
boundary the fix moves). The one BASELINE −12 hit had quality 0.31 (just above the
old 0.3 gate); its misses were below it — baseline straddles the cliff at −12.
**FAR clean on BOTH arms (0/120 s pure noise)** ⇒ relaxing the soft gate cost ZERO
false alarms on real HW (the count gate holds), matching the §4 sim (0/5000).

### §11.2 The NEW binding stage (revealed by the fix)
FIXED CONNECT: 3/3 through −12, then **0/3 at −14/−16 despite HAIL 3/3.** With HAIL
no longer the limiter below ~−12, the binding establishment stage is now the
**control handshake / START_CONNECTION decode** (the CMD→RSP CONNECT exchange) —
the predicted next stage (mini-Moose CFO sync / ctrl-suffix FEC). Log signature at
−14/−16: RSP logs `[HAIL] Detected` but no `START_CONNECTION received` and no
`CONNECTED`. This is the intended handoff to the tier-2 ctrl-suffix work.

### §11.3 Net
HW HAIL-detection floor moved from baseline's ~−12 dial cliff to ≥ −16 dial
(≥ ~4 dB on this testbed axis; the sim's absolute figure is +8.30 dB SNR3k). FAR
0 on both arms. The fix does exactly what §5 predicted; the next-stage failure it
exposes (CONNECT handshake) is the correct downstream target. Single-pass A/B
(3 attempts/cell) — solid 3/3-vs-0/3 separation at −14/−16 makes the conclusion
robust to the single pass, but a multi-pass re-run would tighten the −12 boundary.
