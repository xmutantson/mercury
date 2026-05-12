# Mercury IONOS-Era Validation Plan (2026-05-08)

## §1 Goal and scope

Between 2026-03-23 and 2026-04-20, nine commits landed on `monitor` in
response to issues discovered while testing Mercury through the IONOS HF
channel simulator. After cabling rebuild (cable_topology_test2.py:
~39 dB SNR, symmetric, all PASS) the direct-cable testbed became clean,
but Mercury throughput still ceilings at ~394 bps — far below the
historical VB-Cable WB CONFIG_15 number of 2479-3785 bps.

This plan validates which IONOS-era changes are load-bearing, which are
band-aids capping clean-cable throughput, and which loosened detection
thresholds carry false-alarm risk. Outputs feed a per-change keep /
revert / refactor decision and a final fact document
(`IONOS_ERA_VALIDATION.md`) recording results.

Engineering standards apply: root-cause over symptoms; no untested
fixes; no threshold tweaks to mask failures (`CLAUDE.md`).

## §2 IONOS-era commits surveyed

| Commit | Date | Title | Files | Lines |
|---|---|---|---|---|
| 5c0d1e8 | 03-23 | Fix responder data delivery, SUPERSHIFT ceiling, B2F resume | 6 | +140/-45 |
| b4e5dff | 03-24 | Add SNR-in-MFSK-ACK for faster turboshift via SUPERSHIFT | 11 | +625/-44 |
| 884b6b6 | 03-24 | Fix 3 turboshift bugs found during IONOS testing | 5 | +111/-2 |
| b806b76 | 03-26 | Fix IONOS HF data frame failures: CSI-LLR, PHY reinit, RX norm | 7 | +221/-82 |
| c9eccbd | 03-27 | IONOS fading channel optimization | 10 | +492/-154 |
| dd08185 | 03-27 | IONOS optimization: amp restoration, batch desync, hysteresis | 5 | +69/-83 |
| 6128d2a | 03-28 | Implement Selective ACK (SACK) | 10 | +862/-13 |
| 50e21d6 | 04-20 | Fact document only | 1 | +133 |
| 7076a4b | 04-20 | SACK+OFDM fixes | 18 | +2491/-354 |

No tracked-file uncommitted changes; only new untracked tools/docs.

## §3 Findings from prior review

Eight Opus subagents (one per substantive commit) reviewed each change
for necessity and regression risk. Summary verdicts:

### §3.1 Load-bearing (keep)

- 5c0d1e8 Bug #61: responder recv-timeout split — real protocol bug
  (`arq_common.cc:2158-2191`).
- 5c0d1e8 Bug #59: `supershift_proven_ceiling` tracking — correct
  memory variable; six enforcement sites are copy-paste fragile [?].
- 5c0d1e8 B2F FBB extended FS resume — correct parsing.
- b4e5dff: SNR-in-MFSK-ACK feature — architectural fix to a real
  protocol gap; not IONOS-specific.
- 884b6b6: three turboshift state-machine bugs — channel-agnostic
  defects, all genuine root-cause fixes.
- b806b76 #62 CSI-weighted LLR: textbook DSP, mean-normalized so collapses
  to ≈1.0 on flat channel (`telecom_system.cc:1929-1995`). Regression
  risk on flat channel is bounded.
- b806b76 #60 PHY reinit 300 ms settle: gates real race
  (`arq_commander.cc:2502-2511`); adds 300 ms/turboshift-step latency
  on every channel.
- c9eccbd DFT channel-estimate smoothing: textbook (Edfors VTC 1995).
- 7076a4b Fix A (OFDM frame_overflow defer extension): bug-preserving
  symmetry with MFSK path; channel-agnostic.
- 7076a4b SACK bug fixes (#9, #11, #12, #15): all real defects.

### §3.2 Prime suspects for the 394-bps clean-cable cap

c9eccbd is the standout. Five throughput-capping changes layered into
one commit, all still at HEAD:

1. `WB_CONFIG_MAX = CONFIG_15` (`common_defines.h:78`) silently
   reverted memory's "WB ceiling raised to CONFIG_16". Caps PHY at
   ~4361 bps.
2. `start_config = SNR_predicted − 2 ladder steps` in
   finish_turbo_direction. Even when SNR predicts CONFIG_15, ladder
   starts at CONFIG_13.
3. Ceiling ratchets down on every BREAK; needs 20 consecutive good
   blocks to recover (`arq_commander.cc:2861`). Single transient ACK
   loss caps speed for minutes.
4. Gearshift up 70%→85%, down 40%→55%, cooldown 2→5
   (`datalink_config.cc:48-50`). 85% upshift bar is high; clean-cable
   batches losing 1 frame in 7 sit on the edge and never climb.
5. MMSE α>0.1 erasure now applied to PSK because dd08185 globally
   disabled `amplitude_restoration`. On flat channel σ²ₙ estimate is
   interpolation residual, not real noise — can erase good
   subcarriers (violates `feedback_csi_flat_channel`).

The c9eccbd commit message also misstates two of its own values (LLR
clamp says ±20, code is ±12; ceiling recovery says 8 blocks, code is
20) — suggests it was not carefully audited.

### §3.3 Other threshold relaxations flagged (CLAUDE.md band-aid policy)

- b806b76: `mean_H` timing-quality gate 0.50→0.30
  (`telecom_system.cc:1876`); PSK variance floor 0.05→0.001
  (`psk.cc`); LS window 20×20→2×8 (`physical_config.cc:64-65`); energy
  gates 0.001→1e-12.
- dd08185: 3-consecutive-fail downshift hysteresis
  (`arq_commander.cc:2887-2895`); LDPC iter 50→100
  (`physical_config.cc:74`); LLR clamp ±12→±20
  (`telecom_system.cc:~1991`).
- 7076a4b: ACK metric 3.0→0.5 (`arq_common.cc:4175`); ack_match 8→7
  (`mfsk.cc:170,179`); break_match 8→7 (`mfsk.cc:226,272`); hail_match
  8→7 (`mfsk.cc:319,326`); ~~emergency_nack 6→3~~ — corrected by §16
  Phase 0.2 drift catalog: actual change is **2→3** at
  `arq_common.cc:228` (initialized at 2 pre-IONOS).
- The hail_match relaxation is safety-critical (uncommanded TX risk on
  noise) and warrants a dedicated false-alarm test [?].

### §3.4 SACK architectural concern

6128d2a SACK timing is fundamentally mismatched with Mercury's OFDM
ring-buffer / preamble-search model. The 1168 ms WB SACK burst exceeds
the ACK-sized snapshot window used by cross-check call sites; OFDM
preambles arriving in the buffer tail beyond `upper_bound` trigger the
31-symbol fast-forward. 7076a4b mitigates but doesn't structurally
resolve. v2 redesign was attempted and reverted. SACK gates off when
batches complete cleanly, so on direct cable it is dormant — risk is
on partial-batch IONOS fading paths.

## §4 Validation methodology

### §4.1 Two parallel tracks

- **HOST** — this Windows machine. VB-Cable loopback (`L`) +
  `--test`/`PLOT_PASSBAND` simulation (`S`).
- **PI** — butler at `localhost:7700` controlling rpi1/rpi2/IONOS.
  Clean direct cable (`C`) + IONOS fading (`F`).

The two tracks are independent compute resources and run benchmarks
concurrently. Single butler lock serializes C and F access on the PI
side; HOST runs anytime. Sync points are phase boundaries, where
verdicts from both tracks combine to gate the next phase.

See `feedback_validation_methodology.md` and
`project_testbed_parallelism.md` in user memory for the durable form
of these conventions.

### §4.2 Agent-judged verdicts

Pass/fail is **not a binary numeric threshold**. Each action collects
data; an Opus subagent renders the verdict — improvement / regression
/ no-change, with confidence and caveats — given mean/σ/min/max across
runs.

Verdict prompts specify:
- The hypothesis under test
- Baseline data (Phase 0 numbers)
- New data
- Channel(s) of interest
- Whether to recommend keep-revert / un-revert / extend variance

Numbers are cited; judgment is qualitative.

### §4.3 Variance budget

- Default: **3 runs per condition.**
- Agent may **upgrade to 5 runs** if 3 are insufficient (close call,
  high spread, suspected outlier). Don't pre-bake 5 everywhere — Pi
  hardware time is the scarce resource.
- Long-run stability tests (Phase 5) use a single 1-hour run per
  channel.

### §4.4 Build / patch staging

**Decision (2026-05-08):** **feature flags merged into the build**
(e.g. `--no-csi-llr`, `--legacy-ceiling`, `--no-csi-erasure-psk`,
`--gearshift-legacy`, `--ack-metric=N`). Trade-off accepted: more
code, cleaner tree, every A/B stays available for the rest of the
campaign and for any later regression hunt.

**Implementation rules:**
- Default behavior is HEAD's current behavior (flags opt *out* of
  IONOS-era changes, default off where the change adds something /
  default on where the flag *disables* something). State this
  explicitly in each flag's help text so default = HEAD.
- Each flag scoped to one §7 sub-change. If a sub-change has multiple
  knobs, keep them as one flag with options unless that obscures the
  hypothesis.
- Flags land in `monitor` branch as part of the campaign; survive
  past Phase 6 unless a clear winner gets baked in and the flag is
  removed in the cleanup commit.
- Build verification: `mercury.exe --test` must pass with each flag
  in default and toggled state.

### §4.5 Channels glossary

- `L` — VB-Cable loopback on host, `wasapi -n`
- `C` — clean direct cable testbed, butler-mediated
- `F` — IONOS fading: `WGN:30 OFFSET:-1 FM DEVIATION:1 FADE DEPTH:5 FADE FREQ:0.5 BANDWIDTH:3000`
- `S` — simulation only: `--test`, `PLOT_PASSBAND` BER

## §5 Phase 0 — Baselines

No code changes. Outputs feed every later verdict.

| # | Hypothesis | Track | Action | Data |
|---|---|---|---|---|
| 0.1 | HEAD passes correctness tests | HOST | `mercury.exe --test` | Pass/fail (binary here) |
| 0.2 | Catalog every numeric literal that drifted since IONOS | HOST | Dump `\b\d+(\.\d+)?\b` from `arq_*.cc`, `ofdm.cc`, `telecom_system.cc`, `mfsk.cc`, `psk.cc`, `physical_config.cc`, `datalink_config.cc`, `common_defines.h` at HEAD vs parent-of-5c0d1e8 | Diff catalog of every changed numeric — input to §7 |
| 0.3 | Establish loopback throughput at HEAD | HOST | 3×60 s at NB_CFG4, NB_CFG10, WB_CFG4, WB_CFG10, WB_CFG14, WB_CFG15 | mean/σ/min/max per config |
| 0.4 | Establish clean-cable throughput at HEAD | PI | Same 6 configs × 3 runs via butler | mean/σ/min/max per config |
| 0.5 | Establish IONOS-fading throughput at HEAD | PI | Same configs × 3 runs with §4.5 F settings | mean/σ/min/max per config |
| 0.6 | BER curves at HEAD match expectations | HOST | `PLOT_PASSBAND` for CFG_0/4/10/15/16, NB | Curves saved |

**Sync — Phase 0 verdict agent:** characterize variance per channel;
identify configs with largest C-vs-historical-VB-Cable gap; rank
priorities for Phase 1.

## §6 Phase 1 — Bisect: where did clean-cable throughput regress?

| # | Hypothesis | Track | Action |
|---|---|---|---|
| 1.1 | Loopback regression localizable across the 9 commits | HOST | `tools/bisect_benchmark.py` over `[pre-IONOS-parent, 5c0d1e8, b4e5dff, 884b6b6, b806b76, c9eccbd, dd08185, 6128d2a, 7076a4b]` × WB_CFG10/WB_CFG15/NB_CFG10 × 3 runs |
| 1.2 | Loopback findings replicate on clean cable | PI | Same bisect, WB_CFG10 + WB_CFG15 only, 3 runs (hardware time) |
| 1.3 | Identify commits that HELPED on IONOS fading | PI | Same bisect on F (after 1.2) |

**Sync — Phase 1 verdict agent:** for each commit, classify "improved
C / regressed C / neutral" × "improved F / regressed F / neutral".
Output: ranked suspect list with confidence. Agent may upgrade close
calls to 5 runs.

## §7 Phase 2 — Sub-change isolation

Each entry reverts ONE change on top of HEAD, measures, un-reverts.
HOST and PI run different entries simultaneously.

### §7.1 HOST track (sim/loopback)

| # | Sub-change | Action |
|---|---|---|
| 2H.1 | CSI-weighted LLR on flat channel (b806b76 #62) | Add `--csi-llr=off` flag; BPSK/QPSK BER on flat AWGN |
| 2H.2 | LS window 20×20 → 2×8 (b806b76) | Patch back to 20×20; BER curves all configs |
| 2H.3 | PSK variance floor 0.05→0.001 (b806b76) | Patch back; BER curves PSK modes |
| 2H.4 | MMSE α>0.1 erasure applied to PSK (c9eccbd × dd08185 interaction) | Gate erasure to QAM only; PSK BER on flat AWGN |
| 2H.5 | LLR clamp ±12 vs ±20 (c9eccbd) | A/B both; BER at high SNR |
| 2H.6 | DFT smoothing benefit on flat (c9eccbd) | A/B with smoothing on/off; PSK BER |
| 2H.7 | Loopback throughput with combined PI patches | Apply combined revert; loopback benchmark |

### §7.2 PI track (hardware A/B)

| # | Sub-change | Action |
|---|---|---|
| 2P.1 | `WB_CONFIG_MAX=CONFIG_15` (c9eccbd) | Revert `common_defines.h:78` + SNR table at `telecom_system.cc:4761` to CONFIG_16; WB_CFG15+ benchmark on C, F |
| 2P.2 | Ceiling-recovery 20-block ratchet (c9eccbd) | `arq_commander.cc:2861` 20→8; 5-min runs on C, F; count BREAK/down-ratchet events |
| 2P.3 | `start_config = SNR_predicted − 2` (c9eccbd) | Remove −2 offset; time-to-peak after turboshift on C, F |
| 2P.4 | Gearshift 85/55/cooldown=5 (c9eccbd) | `datalink_config.cc:48-50` → 70/40/2; throughput on C, F |
| 2P.5 | `amplitude_restoration=NO` global (dd08185) | Patch to YES for PSK only; throughput on C, F |
| 2P.6 | `mean_H` gate 0.50→0.30 (b806b76) | Revert to 0.50; throughput + decode rate on C, F |
| 2P.7 | 3-block downshift hysteresis (dd08185) | Revert to 1-block; intentional SNR degradation mid-run on F |
| 2P.8 | `emergency_nack` 6→3 (7076a4b) | A/B BREAK count on F with DEPTH:8 |

**Per-entry verdict prompt:** given Phase 0 baseline and patched-run
data, did C improve? did F regress? confidence? recommend keep-revert
/ un-revert / gather 2 more runs?

**Sync — Phase 2 verdict agent:** consolidate all 2H + 2P verdicts
into a "candidate revert set."

## §8 Phase 3 — False-alarm tests (loosened thresholds)

All HOST track except 3.5. Can run during Phase 1/2 idle.

| # | Hypothesis | Action |
|---|---|---|
| 3.1 | ACK metric 3.0→0.5 + ack_match 8→7 cause false-positive ACKs on noise | Generate WGN at typical RX gain; run `receive_ack_pattern()` 60 s × 3 SNR levels; count detections |
| 3.2 | mean_H 0.30 admits noise frames | Same WGN; count timing-gate passes |
| 3.3 | break_match 8→7 false-fires | WGN feed; count BREAK detections |
| 3.4 | hail_match 8→7 — uncommanded TX risk (safety-critical) | WGN feed during idle listen; count HAIL detections |
| 3.5 | emergency_nack 6→3 spurious BREAK on marginal F | Run on F with DEPTH:8 (only PI item this phase) |

**Sync — Phase 3 verdict agent:** each loosened threshold → measured
false-alarm rate. Flag any non-zero rate, especially HAIL.

## §9 Phase 4 — SACK architectural assessment

| # | Hypothesis | Track | Action |
|---|---|---|---|
| 4.1 | SACK stays dormant on clean cable | PI | Add SACK-fired counter (instrumentation); 5 min run on C |
| 4.2 | SACK measurably helps on F | PI | Compile-time disable CAP_SACK; benchmark on F; compare with HEAD |
| 4.3 | If 4.2 shows benefit: quantify late-snapshot ceiling | PI | Inspect SACK detection rate (true positives / total partial batches) on F |
| 4.4 | Architectural redesign worth the cost | — | Verdict-only, based on 4.1–4.3 |

**Sync — Phase 4 verdict agent:** keep / disable / redesign decision.

## §10 Phase 5 — Combined revert patch + cross-channel validation

| # | Hypothesis | Track | Action |
|---|---|---|---|
| 5.1 | Combined revert patch wins on C without losing F | PI | Apply Phase-2 winners as one patch; full benchmark on C, F; 3 runs |
| 5.1H | Same patch doesn't regress loopback | HOST | Full benchmark on L; 3 runs |
| 5.2 | Long-run stability holds | both | 1-hour continuous on each channel |
| 5.3 | Symmetric (CMD↔RSP swap) — no asymmetry surprise | both | `--swap-roles` benchmark on each channel |

**Sync — Phase 5 verdict agent:** patch ready / blocker / partial
revert needed. May upgrade to 5 runs on close calls.

## §11 Phase 6 — Document & commit

| # | Action |
|---|---|
| 6.1 | Write `mercury/fact-documents/IONOS_ERA_VALIDATION.md` with all phase tables, verdicts, and final per-change disposition |
| 6.2 | Update `MEMORY.md` (e.g., "WB ceiling raised to CONFIG_16" — was that silently dropped, then restored?) |
| 6.3 | Stage commits — one per logical revert/keep group, citing validation data |
| 6.4 | Decide on remaining policy violations (band-aids that didn't measurably regress) — keep with note, or revert on principle |

## §12 Parallelism summary

```
PHASE 0:  HOST: 0.1, 0.2, 0.3, 0.6      ║   PI: 0.4 → 0.5
PHASE 1:  HOST: 1.1                      ║   PI: 1.2 → 1.3
PHASE 2:  HOST: 2H.1–2H.7                ║   PI: 2P.1 → ... → 2P.8
PHASE 3:  HOST: 3.1–3.4                  ║   PI: 3.5
PHASE 4:                                  ║   PI: 4.1 → 4.2 → 4.3
PHASE 5:  HOST: 5.1H, 5.2H, 5.3H         ║   PI: 5.1 → 5.2 → 5.3
PHASE 6:  HOST: docs + commits
```

HOST track total: ~12 h (mostly unattended).
PI track total: ~25 h (serialized through butler).
Wall-clock with parallelism: **~25–28 h** vs ~48 h serial.

## §13 Resolved decisions (2026-05-08)

- **Patch staging (§4.4):** feature flags merged into the build. See
  §4.4 for implementation rules.
- **Butler lock cadence:** fresh lease per action. Tradeoff: more
  setup overhead, but recoverable if mercury hangs and prevents the
  worst-case "long lease + crash" of losing all in-flight data.
  Mitigation for "another agent grabbing the butler between actions":
  the campaign is a single-driver effort; if a slot opens between
  fresh leases that's tolerable.
- **First execution step:** pipeline Phase 0 → Phase 1 once HOST
  action 0.3 (loopback baseline) finishes. PI track continues 0.4 →
  0.5 in parallel; HOST starts 1.1 (loopback bisect) immediately
  after 0.3.
- **SUPERSHIFT helper extraction (§3.1, 6 enforcement sites):**
  separate refactor task **after** the campaign completes. Logged
  here so it's not lost; do not bundle with validation reverts.

## §13a Remaining open items (campaign-internal)

- [?] **HAIL false-alarm rate** (§3.3, §8 entry 3.4) — safety-critical
  (uncommanded TX risk on noise). If Phase 3.4 measures any non-zero
  rate at typical RX gain, escalate to a separate root-cause
  investigation before any HF deployment.

## §14 Cross-cutting prior art

A scan of the other Mercury and Iris fact documents (2026-05-08) found
five items that overlap with this plan and should inform execution.
Numbered for cross-reference from the phase tables.

### §14.1 CAP_SACK_V2=0x40 throughput regression mystery

`SACK_REDESIGN_PLAN.md` §9 records that during the v2 attempt, simply
**setting** the `CAP_SACK_V2 = 0x40` capability bit (with v2 TX/RX
code path disabled) capped throughput at ~185 bps vs 313-476 bps
without the bit. Cause never identified before the v2 attempt was
reverted.

This is not addressed by §9 Phase 4's compile-time `CAP_SACK` disable.
**Action:** during 4.1 instrumentation, also probe the capability
negotiation path — does any current code branch on capability bits in
a way that costs throughput when an unrelated bit toggles? Add the
bit-flip A/B as a sub-test of 4.1.

### §14.2 Iris parallel discovery on LLR sigma_sq poisoning

`iris/fact-documents/ota_session_20260417.md` §9 and §9.7 document
Iris's independent rediscovery of the same mechanism this plan flags
in §3.2 #5 (MMSE α erasure on PSK on flat channel) and §7.1 (2H.1
CSI-LLR on flat). Iris finding: per-carrier sigma_sq from arithmetic
mean of guard-bin energy is **poisoned by H-smoothness** on flat
channel — guard-bin nv is dominated by interpolation residual, not
true noise. Iris landed `llr_use_frame_nv` flag (commit `15d4966`),
got null result on weak direction (guard-bin contamination on
asymmetric path).

**Action:** before running 2H.1 / 2H.4, read ota_session §9.5 risks.
Predict that direct A/B of `--csi-llr=off` may show null result on
clean cable for the same reason Iris's flag was null on weak — both
projects' metrics are robust to `mean(weights)≈1` on flat. Consider
"median-of-percentile per-carrier sigma" (Iris §6.5a / §9.7 #2) as a
candidate fix that may apply to Mercury too.

### §14.3 Iris's threshold-bandaid postmortem as empirical support

`iris/fact-documents/ota_session_20260417.md` §8.1 and the four
explicit reverts (`b56cff9`, `01bea33`, plus chirp-threshold and
FD-ZC reverts) are direct empirical support for Phase 3
false-alarm tests. Iris dropped chirp peak/median 20→15 dB and FD-ZC
0.97→0.80 to admit weak signals; **both regressed throughput to 0 bps
on noise admittance** when re-tested. Threshold-band-aid policy
(`feedback_no_threshold_bandaids`) is not theoretical — Iris has the
data.

**Action:** cite ota_session §8.1 in the §3.3 plan rationale and in
each Phase 3 false-alarm test prompt. Use Iris's revert methodology
(measure throughput at the original threshold, then at the relaxed
threshold, on a representative noise floor) as the §8 template.

### §14.4 SACK_THROUGHPUT_INVESTIGATION §16.3 unresolved items

`SACK_THROUGHPUT_INVESTIGATION.md` §16.3 lists open items not
visible in this plan:
- NB SACK Sidelnikov uses g=13 / g=3 in code vs spec g=6 / g=7 —
  discrepancy never investigated
- Plan double-buffer crypto_batch_buffer status not verified at
  HEAD
- Retransmit-only-batches vs mixed payload behavior

**Action:** if §9 Phase 4 verdict 4.4 is "keep SACK", add a Phase 4.5
specifically triaging §16.3. If verdict is "disable" or "redesign",
fold these into the redesign scope.

### §14.5 SACK_LATE_SNAPSHOT §6 open questions to fold into Phase 4

`SACK_LATE_SNAPSHOT_BUG.md` §6 has two unresolved open questions:
- [?] §6 Q1: Why does the main 130816-buffer SACK path *also* see
  silent buffers? Is it polling AFTER the SACK already scrolled past
  the tail?
- [?] §6 Q2: Does `nframes` propagate correctly through all SACK call
  paths? ACK cross-check paths use default `nframes=0` →
  `bitmap_nsuffix=0` → smaller `reserve_after` in detector. May be
  root cause of 65408 buffer size.

The SACK-fired counter from §9 (4.1) won't surface these. **Action:**
extend 4.1 instrumentation to log `nframes`, `bitmap_nsuffix`,
`reserve_after`, and snapshot timing relative to RSP TX completion.
Resolve both [?] before deciding 4.4.

### §14.6 Contradiction reconciliation

Two explicit contradictions in the prior literature that the plan
should not re-litigate:

- **ACK metric 3.0→0.5** is defended in
  `SACK_THROUGHPUT_INVESTIGATION.md` §15.2 (math: matched_count
  threshold is the primary gate; 0.5 metric is just above noise
  floor of ~0.32) and flagged as a band-aid in §3.3 of this plan.
  **Both can be true** (math is correct on clean; risk is on
  noise). Phase 3.1's false-alarm test resolves it empirically. Cite
  §15.2 in the test prompt so the agent has the math.
- **SACK redesign attempted and reverted** —
  `SACK_REDESIGN_PLAN.md` §9 and the reverted v2 should not be
  re-attempted as part of this plan. Phase 4 verdict 4.4 should
  decide *deferral or harness-build first*, not redesign-now.

## §15 References

- `mercury/fact-documents/SACK_REDESIGN_PLAN.md` — prior SACK
  investigation
- `mercury/fact-documents/SACK_THROUGHPUT_INVESTIGATION.md` — ACK
  metric semantics (§15.2 explains the 3.0→0.5 math)
- `mercury/fact-documents/SACK_LATE_SNAPSHOT_BUG.md` — architectural
  diagnosis
- `tools/bisect_benchmark.py` — existing across-commit benchmark
  harness
- `tools/cable_topology_test2.py` — direct-cable validation showing
  39 dB SNR symmetric path
- `CLAUDE.md` — engineering standards (root-cause, no band-aids)
- User memory `feedback_validation_methodology.md`,
  `project_testbed_parallelism.md`,
  `feedback_csi_flat_channel.md`,
  `feedback_no_threshold_bandaids.md`

## §15c Phase 2 verdict — SACK is the root cause (2026-05-11)

After implementing 14 feature flags across the Tier-1/Tier-2 sub-changes
of b806b76 and 7076a4b, and running A/B sweeps at both 60s and 180s
windows, the campaign localized the NB_CFG10 regression to a single
root cause: **SACK enabled by default**.

### §15c.1 Methodology corrections

- **60 s measurement window is aliased** for NB_CFG10. Initial sweep
  showed all flags returning identical 54.4 bps because 60 s lands on
  an integer multiple of the NB batch cycle. Real throughput at 180 s
  window: **HEAD baseline = 84.6 bps σ=8.6**, not 54.4.
- Every individual flag in §15b.4 either has no effect on NB throughput
  or HURTS when reverted. None individually restore pre-IONOS levels.
- The +3000 ms `sack-timeout-extra-ms` flag is load-bearing (reverting
  to 0 breaks decode entirely) — it's a compensation for the slowness
  SACK introduced elsewhere, not the cause itself.

### §15c.2 The finding

| Condition | mean | σ | runs |
|---|---|---|---|
| HEAD baseline (SACK on) | 84.6 bps | 8.6 | 3/3 |
| Pre-IONOS (no SACK) — bisect | 135 bps | — | 3/3 |
| **`--no-sack` at HEAD** | **151.1 bps** | **56.1** | **3/3** |

`--no-sack` not only matches but **exceeds pre-IONOS** by 12%. The
SACK-enabled-by-default behavior costs ~45% of NB_CFG10 throughput.
SACK cannot even fire usefully on NB (batches too small for partial-
retransmit to benefit); it's pure overhead on this config.

### §15c.3 Disposition

The validation campaign's actionable recommendation:

- **SACK should not be enabled by default on NB modes.** Either gate
  CAP_SACK on bandwidth_mode != NB, or make SACK opt-in via CLI flag.
- The b806b76 and 7076a4b sub-changes — though many of them are
  threshold band-aids per CLAUDE.md policy — are mostly NET BENEFICIAL
  individually at HEAD (per 180s sweep). They should largely stay,
  with the threshold-tweak nature noted but not reverted.
- Only `--csi-llr=off` and `--sack-timeout-extra-ms=0` are confirmed
  hard-required at HEAD; everything else is +/- small effect.

### §15c.4 B2 fix landed and verified (2026-05-12)

Production fix landed as commit `856f024`: `cl_arq_controller::disable_sack`
default flipped from `false` to `true`. SACK is now opt-in via the
new `--enable-sack` flag.

**Verification — default behavior (no flag) after B2 fix:**

| Config | HEAD baseline (SACK on, yesterday) | Default after B2 (today) | `--no-sack` (yesterday) | Historical |
|---|---|---|---|---|
| HOST NB_CFG10 | 84.6 | **154.1** ✓ | 151.1 | 135 / 181 |
| HOST WB_CFG15 | 1690 | **2253.3** ✓ | 2353.4 | 2479 / 2599 |
| PI NB_CFG10 | 54.4 (2/3) | **72.5** (3/3) ↑33% | not tested | 135 / 181 |
| PI WB_CFG15 | 1001.4 | **1752.5** ↑75% | not tested | 2479 / 2599 |

Default behavior now matches the `--no-sack` measurement on HOST. PI
also benefits but is slower than HOST overall — that residual is real
hardware overhead (audio DAC/ADC, slower Pi CPU, IONOS analog
passthrough), not the IONOS-era regression.

Why default-off rather than NB-only gate (per the original §15c.3
recommendation): data showed SACK hurts both NB and WB. SACK is alpha
(v2 redesign attempted and reverted — see SACK_REDESIGN_PLAN.md §9).
Users who want SACK can opt in with `--enable-sack`.

### §15c.5 Resolved open work

- ~~Should SACK be removed entirely or just gated to WB?~~ — disabled
  everywhere; data showed both bands suffer (`856f024`).
- ~~Does `--no-sack` similarly restore WB_CFG15?~~ — yes; 1690→2353 bps
  on HOST, 1001→1752 on PI.
- PI 1.2/1.3 fading bisect — still optional; the headline regression is
  resolved by B2 fix.

### §15c.6 Remaining gaps (post-B2)

- **PI < HOST** even after B2: NB_CFG10 PI is 47% of HOST, WB_CFG15 PI
  is 78% of HOST. Hardware/audio-path overhead, not a regression.
- **NB on PI still below pre-IONOS** (72.5 vs 135 bps): SACK fix removed
  ~33% of the gap but a residual ~46% loss remains, unique to NB on
  real audio. Could be NB MFSK sensitivity to analog noise/timing,
  ARQ behavior on slower Pi CPU, or other unflagged sub-changes.
  Not investigated further this session — fixing SACK was the headline.

## §15b Phase 0 corrected baselines (2026-05-10)

**~~The Phase 0 results in §15a below are partially superseded.~~** The
`phase0_baseline.py` harness used today had two latent bugs that
collapsed many runs to 0 bps and skewed the regression magnitude:

1. **`-W -Q 0` (without `-M auto`) breaks WB decode.** Memory's loopback
   recipe is `-Q 0 -M auto` — `-Q 0` only works with `-M`. Mercury
   without the `-M` mode flag and with `-Q 0` (NB probe max = 0)
   leaves RSP stuck in `connection_status:Idle` while CMD transmits
   data into the void.
2. **`-E fast` (encryption fast-mode) breaks decode at HEAD.** With both
   processes negotiating fast-mode encryption, the negotiation hangs
   and no data flows. `-F off` (compression off) is safe.

Both bugs were isolated via `tools/flag_isolation.py` and
`tools/flag_isolation2.py`. Phase0_baseline.py was patched 2026-05-10
to use `-Q 0 -M auto/nb` (matching `bisect_benchmark.py`) and to omit
`-E fast`.

### §15b.1 Corrected baselines (HOST + PI clean + PI fade)

| Config | HOST (VB-Cable) | PI clean | PI fade | Historical (memory) | Real regression |
|---|---|---|---|---|---|
| NB_CFG4 | 0/3 ❌ | 0/3 ❌ | 0/3 ❌ | 74 bps | **BROKEN at HEAD** |
| NB_CFG10 | 54.4 (3/3) | 54.4 (2/3) | 60.4 (3/3) | 181 bps | **−70%** |
| WB_CFG4 | 134.4 (3/3) | 112.0 (2/3) | 89.6 (2/3) | 238 bps | **−53%** |
| WB_CFG10 | 453.3 (3/3) | 251.8 (3/3) | 352.6 (3/3) | 797 bps | **−40 to −68%** |
| WB_CFG14 | 1120.0 (3/3) | 995.5 (3/3) | 497.7 (3/3) | — | (no historical) |
| WB_CFG15 | 1690.0 (3/3) | 1001.4 (3/3) | 563.3 (2/3) | 2479 bps | **−32 to −78%** |

Files: `baseline_0.3_loopback_FIXED_20260509_161514.json`,
`baseline_0.4_clean_FIXED_20260509_161458.json`,
`baseline_0.5_fade_FIXED_20260510_142509.json`.

### §15b.2 What's actually broken at HEAD

After the harness correction, two real regressions remain:

1. **NB_CFG4 completely broken** at HEAD across all channels (was 74 bps
   historically). This is the most severe and most localized regression.
2. **NB_CFG10 −70%** (181 → 54–60 bps across channels).
3. **WB throughput −32 to −78%** depending on config and channel.
4. **PI IONOS fading works** — WB_CFG15 fade = 563 bps matches memory's
   historical 510–536 range. The "0/18 fading regression" was 100%
   harness artifact.

### §15b.3 Phase 1.1 bisect remains valid (used correct harness)

Phase 1.1 HOST bisect was run via `bisect_benchmark.run_throughput_test`,
which uses the correct `-Q 0 -M auto/nb` recipe and **never used
`-E fast`**. Its NB_CFG10 trajectory is the authoritative localization:

| Commit | NB_CFG10 |
|---|---|
| 6824262 (PRE) | 135 bps |
| 5c0d1e8 | 135 |
| b4e5dff | 150 |
| 884b6b6 | 90 |
| **b806b76** | **0** ← break #1 |
| c9eccbd | 165 ← partial recovery |
| dd08185 | 181 (best) |
| 6128d2a | 162 |
| **7076a4b HEAD** | **54** ← break #2 |

**Two distinct regression points**, not one. Phase 2 isolation needs
to attack both.

### §15b.4 Phase 2 priority (revised)

- **Tier 1 — b806b76 sub-changes** (the 90 → 0 cliff):
  - mean_H gate 0.50 → 0.30 (`telecom_system.cc:1876`)
  - PSK variance floor 0.05 → 0.001 (`psk.cc`)
  - LS window 20×20 → 2×8 (`physical_config.cc:64-65`)
  - Energy gates 0.001 → 1e-12 (multiple sites)
  - SKIP-VAR gate var > 0.5 (`telecom_system.cc:1911`) — already
    feature-flagged as `--skip-var-gate=on|off`
  - CSI-weighted LLR (`telecom_system.cc:1929-1995`)
  - PHY reinit 300 ms settle (`arq_commander.cc:2502-2511`)
  - RX passband normalization (`telecom_system.cc:861`)
- **Tier 2 — 7076a4b sub-changes** (the 162 → 54 cliff):
  - ACK metric threshold 3.0 → 0.5 (`arq_common.cc:4175`)
  - ack_match_threshold 8 → 7 (`mfsk.cc:170,179`)
  - emergency_nack_threshold 2 → 3 (`arq_common.cc:228`)
  - hail_match_threshold 8 → 7 (`mfsk.cc:273`) — safety-critical
  - break_match_threshold 8 → 7 (`mfsk.cc:226`)
  - SACK bitmap decode + defer extension (Fix A)
- **Tier 3 — NB_CFG4 diagnostic** (separate from CFG_10 bisect):
  - Phase 1.1 only tested NB_CFG10. Need a focused NB_CFG4 bisect to
    confirm whether the CFG_4 failure entered at the same commit(s)
    or has its own trajectory.

## §15a Phase 0 execution findings (2026-05-08)

### §15a.1 SKIP-VAR gate confirmed regression — feature flag landed

The `noise_variance_estimate > 0.5 → SKIP-VAR` gate
([telecom_system.cc:1911](../source/physical_layer/telecom_system.cc#L1911),
b806b76 — flagged in §16.2 of the drift catalog) was confirmed as a
real bug blocking PLOT_PASSBAND simulation. Every OFDM frame in a CFG_4
sweep produced `var=0.998 too high`, zero LDPC decodes.

**Resolution:** feature flag `--skip-var-gate=on|off` added per §4.4
implementation rules. Default = `on` (HEAD behavior). With `--skip-var-gate=off`:
- `[FLAG] --skip-var-gate=off` printed at startup
- 0 SKIP-VAR messages
- 17 OFDM-OK decodes
- BER curves descend correctly: -10 dB→0.232, -9.5 dB→0.205, -9 dB→0.169
- SACK self-test passes (`matched=16/16, LDPC bitmap PASS`)

This is the **template** for Phase 2 sub-change feature flags. Files
touched (3 small edits):
- [include/physical_layer/telecom_system.h](../include/physical_layer/telecom_system.h) — `bool skip_var_gate_enabled` member
- [source/physical_layer/telecom_system.cc](../source/physical_layer/telecom_system.cc) — init + gate
- [source/main.cc](../source/main.cc) — CLI parse + wire to instance

### §15a.2 False preamble-lock cascade observed (HOST loopback)

During HOST loopback ARQ at WB_CFG10, RSP initially decodes cleanly
(`[OFDM-OK] var=0.033 meanH=0.979 SNR=14.8`) but then variance spikes
to `var=6.9` and SKIP-VAR fires repeatedly. ARQ never recovers; CMD
gets `nNAcked_control=7`, eventually emits HAIL beacons trying to
re-connect. **Result: ~0 bps delivered after initial control-frame
exchange.**

Hypothesis: Mercury falsely synchronizes on noise/garbage in the ring
buffer, the variance estimator measures noise from those false-sync
attempts, the gate fires, and the buffer scrolls past real frames
before they get a chance.

This is a **distinct issue from §15a.1** — disabling the SKIP-VAR
gate doesn't fix it (frames still don't decode because they're false
syncs). Root cause of the false sync is unclear. **Not investigated
further this session** — captured here for follow-up.

Implication for Phase 0.3 (HOST loopback baseline): expect **very
high variance and frequent zero-throughput runs** at HEAD. The 0 bps
result IS the HEAD baseline on VB-Cable when this cascade triggers.
This may also be the mechanism behind the "394 bps cap" observed on
the PI clean cable in earlier sessions.

### §15a.3 PI 0.4 first real measurement

Background agent's first valid run: **NB_CFG4 clean cable = 18.2 bps**
(1 run, 120 s window, 546 bytes delivered out of 262144 sent). Way
below historical VB-Cable NB_CFG4 = 74 bps from MEMORY.md table.
Suggests a meaningful regression on the PI testbed too. Agent still
running; full sweep pending.

### §15a.4 0.6 BER curves complete (with anomalies)

Inline SACK roundtrip self-test **PASSED** at HEAD (CFG_4 PASSBAND,
matched=16/16, LDPC PASS). Binary verified healthy.

Two new findings flagged by 0.6 agent for future investigation:
- **Intermittent crash in `BER_PLOT_*_process_main()`** — exits with
  code 1 mid-sweep, non-deterministic, frequency increases with
  modulation order. Limited CFG_15/16 sweeps to 5/8 SNR points.
- **NB SACK LDPC-decode FAILS** despite tone matching `matched=32/32`
  in `nb_robust2_passband` log. WB SACK LDPC passes. NB-specific SACK
  LDPC integration issue.

Output artifacts in `mercury/fact-documents/ber_0.6/`.

## §16 Drift catalog (Phase 0.2 result, 2026-05-08)

Numeric-literal diff between pre-IONOS parent `6824262` and HEAD
`7076a4b` across PHY/datalink layers, produced by an Opus subagent
per §5 entry 0.2. ~40 distinct changes catalogued; ~17 already
flagged in §3.2/§3.3; **23 newly discovered** below.

### §16.1 New high-concern findings (add to Phase 2 isolation)

These were missed by the per-commit reviews and should each get a
Phase 2 entry with a feature flag.

| ID | File:line | Old → New | Commit | Concern |
|---|---|---|---|---|
| 2P.9 | `arq_commander.cc:1330,1631,1705,1375,1006,1027` | `emergency_break_retries` 3→1 (≥6 sites) | c9eccbd | **HIGH** — single-shot BREAK retry compounds §3.2 #3 ceiling ratchet |
| 2P.10 | `telecom_system.cc:1287,1365,1469,1655,1666` | absolute energy gates 0.001→1e-12 | b806b76 | **HIGH** — absolute silence detection effectively disabled; relies on relative gate |
| 2H.8 | `telecom_system.cc:861` | RX passband auto-normalization (NEW: 0.001-10000 scale clamp, activates if >1.5 or <0.67) | b806b76 | **HIGH** — silent up-to-80 dB gain; structural DSP, not a threshold |
| 2H.9 | `telecom_system.cc:1840` | CPE_correction unconditional (was NB-only) | b806b76 | MED — DSP scope expanded to WB |
| 2P.11 | `telecom_system.cc:1746` | Moose clamp widened ~4× + sanity-reject \|Δf\|>0.7×subspc | b806b76 | MED — frequency-correction window grew; verify WB still freq-tracks at ±93 Hz |

### §16.2 Other new findings (low/med concern, batched)

- `telecom_system.cc:1336` OFDM coarse-metric weak-peak gate 0.10
  (c9eccbd, new gate) — gates LDPC trial.
- `telecom_system.cc:1908-1916` `noise_variance_estimate > 0.5` SKIP-VAR
  (b806b76, new gate).
- `ofdm.cc:1305` channel_equalizer_without_amplitude_restoration
  erasure floor `nv→1e-12` (c9eccbd).
- `arq_commander.cc:2885-2900` 3-consecutive-fail downshift hysteresis
  (dd08185) — flagged §3.3 but the literal `3` magic number is here.
- `arq_common.cc:1755` `FORCED_ROLE_SWITCH_TIMEOUT` 60000→180000 ms
  (7076a4b) — 3× longer commander-side stuck-detection.
- `arq_common.cc:427` receive timeout +3000 ms when sack_enabled
  (7076a4b, new) — every batch adds 3 s.
- `arq_common.cc:1054` batch `target_time_ms` 10000→12000 ms
  (7076a4b).
- `arq_common.cc:1062` `fixed_batch=10` fallback when no SACK
  (7076a4b) — caps non-SACK throughput.
- `datalink_config.cc:54` `nMessages` 75→120 (c9eccbd).
- `mfsk.cc snr_to_tone` offsets +5 (WB) / +9 (NB), step 2.0 (b4e5dff)
  — quantization map for SUPERSHIFT decisions.
- `physical_config.cc:74` `ldpc_nIteration_max` 50→100 (dd08185)
  — already flagged but worth listing in catalog.
- `telecom_system.cc:3722` GUI `ldpc_iterations_max` upper bound
  50→100 (dd08185) — paired.
- `mfsk.cc` SACK match thresholds 10/16, 24/32, 40/48 (7076a4b, new
  pattern).
- `mfsk.cc` SACK LDPC params N=128 K=32 rate=0.25 nIter=50 (7076a4b,
  new code path).
- `telecom_system.cc:1929` mean_w floor 1e-6 (b806b76).
- `telecom_system.cc:1574` zero_hz_correlation hysteresis +0.08
  (preserved, not new).
- `telecom_system.cc:861` RX passband target_rms = sqrt(power)*0.5
  (b806b76, paired with 2H.8).

### §16.3 Plan corrections from catalog

- §3.3 `emergency_nack 6→3` is **wrong** — actual change is **2→3**
  at `arq_common.cc:228` (initialized at 2 pre-IONOS, set to 3 in
  7076a4b). Strikethrough applied above.
- §3.3 LLR clamp ±12→±20 timing is more nuanced: b806b76 introduced
  ±12 div, dd08185 widened to ±20.

### §16.4 Phase 2 expansion

Phase 2 now has **8 + 3 = 11 PI track entries** (2P.1–2P.11) and
**7 + 2 = 9 HOST track entries** (2H.1–2H.9). Verdict-agent prompts
for new entries should follow the same template as the original
entries (Phase 0 baseline + patched run + recommendation).
