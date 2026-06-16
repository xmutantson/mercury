# Recovery control-ACK robustness — the CFG15 BREAK-recovery crawl fix

**Branch:** `feat/recovery-ack-fix` (off monitor `e2cd1c5`, which carries A1+A2)
**Created:** 2026-06-15
**Status:** DESIGN + IMPLEMENTATION (env-gated `MERCURY_RECOVERY_ACK_ROBUST`,
default-off → byte-identical). Sim fail-before/pass-after in-process. HW A/B is
the serial confirm (NOT done here — sim/in-process only per task scope).
**Owner forensics this builds on:** [[cfg15_stall_root_cause]] (memory, 16 HW
logs), `bigblock_p3_hw/HW_A3_E2E/A3_E2E_VERDICT.md`.

---

## §1 The problem (forensic-established, not re-litigated here)

The CFG15 turnaround stall's SUSTAINING mechanism (100% of stalls, the
BREAK→ROBUST_0 crawl that pins the link at 66 bps) is a **marginal reverse
RECOVERY control-ACK**. When a BREAK fires the link drops to CONFIG_0, the CMD
sends `SET_CONFIG`, and waits for a control-ACK to climb back. On the CLEAN WGN
channel that ACK lands at **matched 6-7/16 vs `ack_match_threshold=7/16`**
(`mfsk.cc:265`) — so every recovery turnaround is a COIN FLIP; a miss logs
`[BREAK-RECOVERY] Phase 1 failed, re-sending BREAK` (`arq_commander.cc:2180`) and
re-BREAKs pinned at CONFIG_0.

A1/A2/A3 all targeted the **data** reverse-ACK (the SACK). The recovery
control-ACK is a DIFFERENT path that none touched — same MFSK pattern + correlator,
different ARQ state (`RECEIVING_ACKS_CONTROL` vs `RECEIVING_ACKS_DATA`).

---

## §2 The recovery control-ACK path — end to end (cited)

**TX (RSP side):** on receiving the recovery `SET_CONFIG` the responder calls
`send_ack_pattern()` (`arq_responder.cc:1158`, the `ack_pattern_time_ms>0` &&
not-turbo arm) → `cl_telecom_system::generate_ack_pattern_passband()`
(`telecom_system.cc:3640`) → `ack_mfsk.generate_ack_pattern()` (`mfsk.cc:546`):
**one** 16-symbol Welch-Costas block (M=16, nStreams=1).

**RX (CMD side):** in `RECEIVING_ACKS_CONTROL` the CMD polls
`receive_ack_pattern()` (`arq_commander.cc:1949`, gated `ack_pattern_time_ms>0 &&
!KEY_EXCHANGE_1 && !TEST_CONNECTION`). That calls
`detect_ack_pattern_from_passband()` (`telecom_system.cc:3644`) →
`ofdm.detect_ack_pattern(..., combine_reps=1)` (`ofdm.cc:3873`). Accept iff
`matched_count >= ack_match_threshold(7) && metric >= ack_metric_threshold(0.5)`
(`arq_common.cc:7266`).

**Same correlator** also serves the BREAK-ACK (`arq_commander.cc:209`), data-ACK
(`RECEIVING_ACKS_DATA`), HAIL, CONNECT. The recovery control-ACK is just one
caller; the correlator is `combine_reps=1` for ALL of them today except CONNECT.

---

## §3 WHY the real ACK peaks at only 6-7/16 on a CLEAN channel (root cause)

NOT thermal SNR. The matched-count floor of this exact detector is **−13.25 dB
SNR3k at R=1** (`hail-detection-floor-investigation.md §4`, measured) — the clean
WGN:40 recovery is ~50 dB above that. The marginality is **structural**:

- The per-symbol "match" is a **HARD peak-bin decision** — count this symbol only
  if the expected tone is the argmax peak among the M=16 bins for all streams
  (`ofdm.cc:3977-4019`). It is energy-gated and Bug-#39 carrier-image-aware, but
  it is still a hard 1/0.
- A TX→RX turnaround on real radio adds **timing jitter** (keyer/AGC/capture-flush
  + ±8.16 ppm SFO + ALSA buffering): the polled-tail FFT windows can straddle a
  symbol boundary, so that symbol's energy splits across two FFTs and its peak
  bin moves off the expected bin → that symbol drops from the count. One or two
  straddled symbols pulls matched from the clean ceiling (~14-16) down to **6-7**.
- **Independent evidence, same detector:** `PI_ACK_MISS_INVESTIGATION.md §2` —
  on a clean channel the Pi's DATA-ACK reaches `matched=6/16, metric=0.7` ("the
  audio contains a clear ACK candidate but **one symbol fails the peak-bin
  check**"); the Host reaches 16/16 on the SAME audio. metric=0.7 ≫ the 0.5 floor
  confirms strong energy is present — it is a hard-decision quantization cliff at
  the threshold, not a noise floor. The recovery turnaround is MORE jittery than a
  steady-state data turnaround (it follows a BREAK + CONFIG reload), so it sits
  even more reliably ON the 6-7 cliff.

### §3.1 The random-match floor (the false-accept bound for the chosen fix)
Per symbol in pure noise the expected bin is the argmax peak of M=16 bins with
P≈1/16; carrier-image acceptance (expected OR mirror, `ofdm.cc:4012-4017`) makes
~2 of 16 bins acceptable → **P(per-symbol false match) ≈ 2/16 = 0.125**. Over 16
symbols E[matched|noise] ≈ 2; the correlator takes the BEST over sliding
positions so the observed noise peak inflates above the mean but the code's
measured FAR at 7/16 is **2.4e-5/poll** (`mfsk.cc:265`) and the empirical FAR
sweeps are **0/5000** at the count gate (`hail §4`). So a REAL ACK at 6-7/16 is
WELL above the ~2/16 random mean but sits right ON the 7/16 accept bar.
**Conclusion: lowering the threshold to 6 would move it toward the noise mean and
inflate FAR (~16× per the FAR table) — a masking change, REJECTED (§5).**

---

## §4 The fix — noncoherent base-pattern repetition combining (the ROBUST option)

Make the REAL ACK clear 7/16 reliably WITHOUT moving the bar, by **repeating the
ACK base block R times and noncoherently combining** at the RX (sum per-symbol
FFT energy across the R aligned reps BEFORE the argmax/count). A symbol that
straddles in one rep is reinforced by the others → its true-tone bin re-wins the
peak → the matched count rises back to the ceiling. This is the SAME mechanism
CONNECT already ships (`connect_preamble_reps`, §20).

**Measured prior art (this exact detector, `ofdm.detect_ack_pattern` combine path):**
- `hail-detection-floor-investigation.md §4`: noncoherent combining on the
  matched-count statistic = **+1.8 dB (R=2) / +3.7 dB (R=5)**; FAR 0-1/5000.
- `mfsk.h:91-92` (§20): "+2.2-2.5 dB/doubling" on the base-pattern floor.
- External: WSJT-X Q65 noncoherent integration ~2.6-3.0 dB/doubling.
- It is **FAR-safe**: combining sums energy at the EXPECTED bins; noise energy is
  not coherent there, so the count gate's FAR is preserved (measured 0/4000 in
  the §20 CONNECT test `test_connect_preamble_combining_cliff_sweep`).

Why this beats the alternatives (task menu): (a) repeat/combine = proven, FAR-safe,
no bar change — CHOSEN; (b) longer single pattern = airtime + a wire change with
no per-rep alignment benefit; (c) soft/correlation metric = the metric is ALREADY
0.7 (above floor), the binding gate is the hard COUNT, and a soft count gate
re-opens FAR; (d) A2-style multi-window = recovers a LATE ACK (a timing miss),
but the recovery ACK is not late — it is PRESENT and energetic, just one symbol
short, so multi-window does not address the per-symbol straddle (different failure
than the data-ACK A2 fixed).

### §4.1 What changes (all gated `MERCURY_RECOVERY_ACK_ROBUST`, default-off)
- **mfsk**: `recovery_ack_reps` member (default 1); `generate_ack_pattern_reps()`
  emits the base block R times, per-rep-LOCAL hop (symbol s of every rep carries
  the same tone → RX can sum rep-r symbol s onto rep-0 symbol s), mirroring
  `generate_connect_pattern` (`mfsk.cc:886`). `ack_base_total_nsymb()` accessor.
- **telecom_system**: `set_recovery_ack_reps(R)` recomputes
  `ack_pattern_passband_samples` for R×16; `generate_ack_pattern_passband()`
  emits R×16 symbols when reps>1. `detect_ack_pattern_from_passband()` forwards
  `recovery_ack_reps` to `detect_ack_pattern(combine_reps=...)`.
- **TX (RSP)** `send_ack_pattern()` (`arq_common.cc`): when robust enabled, set
  reps = `RECOVERY_ACK_REPS` (4) before generating; restore after. Sizing
  (`pattern_samples`) re-read from the recomputed accessor.
- **RX (CMD)**: the detect path already forwards `recovery_ack_reps` via
  `detect_ack_pattern_from_passband` → byte-identical when reps==1.
- `RECOVERY_ACK_REPS = 4` (the §4 measured R=4 ≈ +3-5 dB ≈ +several matched
  symbols, ample to push 6-7 back over 7 with margin). Default member = 1.

---

## §5 Decision gate (CLAUDE.md §2 — root cause, not mask)

The chosen fix raises the REAL-ACK matched count (a robustness improvement) and
does NOT lower the 7/16 accept threshold — so the false-accept floor (FAR
2.4e-5/poll, empirical 0/5000) is UNCHANGED. It is therefore NOT a masking change
→ IMPLEMENT (env-gated for HW A/B), per the task DECISION GATE.

---

## §6 Cross-layer data-flow audit (CLAUDE.md §5)

**State touched:** the MFSK ACK pattern length on the wire (`ack_pattern_nsymb`
× reps) and the RX combine span. Producers/consumers of the ACK pattern:

1. **Producers (TX emit R×16):** `send_ack_pattern()` (RSP, the recovery /
   control-ACK and steady-state data-ACK turnaround) — the ONLY producer we
   change, and ONLY when robust enabled. `send_ack_pattern_with_snr()` (turbo)
   carries an SNR suffix and is NOT changed (reps=1 → byte-identical; turbo SNR
   climb is out of scope, recovery is the no-suffix arm).
2. **Consumers (RX match):** `receive_ack_pattern()` (CMD, both
   `RECEIVING_ACKS_CONTROL` recovery and `RECEIVING_ACKS_DATA` data arms call the
   SAME `detect_ack_pattern_from_passband`). With robust enabled the combine span
   = R×16; the RX energy-gate/tail window already searches the whole tail, so a
   longer pattern is covered (the tail snapshot `frames_to_read` for the ACK wait
   is the full ring under SIM and the 8-symbol pre-filter slides in production —
   §8019 comment). **INVARIANT to hold:** if TX emits R reps, RX MUST combine R
   reps, else a single-block RX correlator reads only the first 16 and is
   byte-identical-but-no-gain (safe, just no benefit). We gate BOTH on the same
   env flag so they move together.
3. **Valid states / default-init:** `recovery_ack_reps` ctor = 1 → every
   accessor returns the pre-change single-block value → BYTE-IDENTICAL. The
   accessor `ack_base_total_nsymb()` clamps reps to [1, MAX].
4. **Invariants consumers assume:**
   - The data-ACK SACK suffix (`send_ack_pattern_with_snr`, SACK_RSP) is a
     DIFFERENT frame (OFDM control) — untouched. The recovery ACK carries NO
     suffix, so repeating the base block cannot corrupt a suffix.
   - A **false-accept** of the recovery ACK = a premature "recovered" → CMD
     loads a config the RSP did not actually confirm → wrong-config DATA. The fix
     does NOT raise this risk (threshold unchanged, FAR unchanged); it LOWERS the
     real-miss rate. The downstream SET_CONFIG verification probe (Phase-2,
     `arq_commander.cc:4775`) is unchanged.
   - The half-duplex turnaround timing: R×16 is ~4× the ACK airtime
     (~389 ms → ~1.4 s at R=4). The CMD recovery-ACK listen window
     (`calculate_receiving_timeout`) must cover it. Audited: the recovery wait is
     a control-ACK wait whose timeout already includes generous frame_drain +
     guards; R=4 ACK (~1.4 s) fits the multi-second recovery window. (HW A/B must
     re-confirm the window covers the longer ACK — noted as the serial caveat.)
5. **What the fix changes:** only the matched-count REACHED on a present ACK
   (raises it). No ARQ-state, queue, batch, or compression state changes. No
   change when the flag is off.

### §6.3 SIBLING BUG the audit caught (the RX capture-tail span)

The §6.2 audit found a coupled defect BEFORE ship: `receive_ack_pattern()`
(arq_common.cc) sized its capture tail from `ack_pattern_nsymb` (16) —
`tail_nsymb = 16 + 16 + 16 = 48` symbols. But with reps=4 the RX combiner
(`detect_ack_pattern`) needs `combine_reps × 16 = 64` symbols and returns **0.0**
when `buffer_nsymb < total_needed`. So the original tail (48) would have made the
combined recovery ACK **NEVER detected** → the fix would have made HW recovery
WORSE, not better. FIXED: the tail now spans `ack_base_total_nsymb()` (R×16) on
the non-turbo path → `64 + 64 + 16 = 144` symbols, clamped to the ROBUST_0 ring
(804 sym, ample). reps=1 (default) → `ack_base_total_nsymb()==ack_pattern_nsymb`
→ tail unchanged → byte-identical. This is the canonical CLAUDE.md §5 sibling bug
(a one-layer fix exposing a consumer in the next layer). Guarded by the test's
`[INVARIANT]` assertion (R×16 fits tail and ring). The detector-direct test would
NOT have caught it (it passes a large buffer); only the production tail audit did.

### §6.4 Consistency: only the recovery TX repeats, only the recovery RX combines

The RSP repeats the ACK base block ONLY in the control-code/SET_CONFIG arm
(arq_responder.cc:1500) and the BREAK arm (:486). All other pattern-ACK control
responses are either LDPC (`TEST_CONNECTION_ACK` → CMD uses the LDPC RX path, not
`receive_ack_pattern`) or the turbo `send_ack_pattern_with_snr` (single block,
NOT repeated). The CMD combines R reps ONLY via the non-turbo
`detect_ack_pattern_from_passband` (the turbo branch uses
`detect_ack_snr_from_passband`, hardcoded combine_reps=1). So whenever reps=R is
active, the TX that produced the buffer DID emit R reps → TX/RX rep counts match
(the §6.2 invariant). The data-ACK turnaround clears reps back to 1.

**Regression test:** `test_recovery_ack_robust_marginal`
(`mfsk_ctrl_codec_tests.cc`) — fail-before/pass-after on the marginal-6-7/16
clean ACK + FAR-on-noise + the §6.3 tail-span invariant, wired into
`mercury.exe --test`. See §7.

---

## §7 Fail-before / pass-after (in-process, AWGN cliff) — MEASURED

`test_recovery_ack_robust_marginal` drives the production ACK TX
(`generate_ack_pattern_passband`, R=1 and R=4) + AWGN at the detector's
matched-COUNT cliff (the §3 / PI_ACK_MISS 6-7/16 regime; a half-symbol straddle
is added but the detector's fine-timing pass recovers it, so the cliff is
noise-driven — the faithful model). NOTE the modeling lesson: a *residual CFO*
biases every rep identically and combining cannot fix it — that is NOT the failure
combining addresses; the failure is per-rep NOISE on the hard peak-bin decision,
which combining averages out (hail §4 +1.8/+3.7 dB). MEASURED (cliff sigma 9×rms,
snr3k ≈ −10 dB):

| | R=1 (single, the old behavior) | R=4 (combined, the fix) |
|---|---|---|
| **P(matched ≥ 7)** | **0.67 (coin flip)** | **1.00 (reliable)** |
| mean matched | 11.6 | 16.0 |
| FAR (pure noise, count+metric gate, 4000) | 1034/4000 | **120/4000** |

- **FAIL-BEFORE / PASS-AFTER:** R=1 0.67 (coin flip) → R=4 1.00, gain +0.33
  (assert: R=4 ≥ 0.95 AND gain ≥ 0.25). On a binary where `combine_reps` is a
  no-op, R=4 == R=1 == 0.67 → the assert FAILS (the fail-before).
- **NO HIGHER FALSE-ACCEPT:** FAR(R=4) 120 < FAR(R=1) 1034 — combining TIGHTENS
  the FAR (noise is not coherent at the expected bins), so the 7/16 bar is not
  loosened (assert: FAR(R=4) ≤ FAR(R=1)+slack). The 1034/4000 R=1 number is the
  cliff operating point, NOT the clean-channel FAR (which is the documented
  2.4e-5/poll); the test drives to the cliff deliberately to make R=1 marginal.
- **BYTE-IDENTICAL-WHEN-OFF:** R=1 `generate_ack_pattern_reps` == the original
  `generate_ack_pattern` (max|diff| = 0.0).
- **§6.3 INVARIANT:** R=4 combine span 64 sym fits the production tail (144) and
  the ROBUST_0 ring (804).

---

## §8 Honest caveats
- Sim proves the MECHANISM (combining lifts a present-but-noise-marginal ACK over
  the bar, FAR-safe, +0.33 P at the cliff) + the §6.3 tail invariant + byte-
  identical-off. The **HW A/B is the serial confirm** of the wall-clock win
  (recovery coin-flip → reliable; cascade-EVENT-count → 0). The in-process model
  is single-shot AWGN at the cliff; real HW jitter is a distribution of timing +
  AGC + SFO transients — the combining gain is the same mechanism but the absolute
  HW operating point must be confirmed on the bench.
- R=4 ACK airtime ~1.4 s (vs ~389 ms). Confirm the CMD recovery/BREAK-ACK listen
  window covers it on HW (the window already covers multi-second recovery; flagged
  for the A/B). The §6.3 RX tail fix is the prerequisite that lets the longer ACK
  be detected at all.
- The fix is the *sustaining-crawl* lever. The §1 forward LDPC OFDM-FAIL TRIGGER
  (83%, the BP non-convergence that LIGHTS the BREAK) is a SEPARATE, harder
  decode-floor item ([[cfg16_decode_loss_is_ldpc_bp]] at CFG15) — making the miss
  CHEAP (this fix) stops the 100-365s crawl even when the trigger fires.
- Default-off; A1+A2 remain the banked turnaround win. This stacks on top (it
  fixes the residual crawl A1/A2 could not reach — they worked the data-ACK, this
  works the untouched recovery control-ACK).
- Validation run: `mercury.exe --test` = 58 passed / 0 failed (incl. the new test);
  `--test-climb-engine` = ALL PASS / 0 failures (the BREAK-recovery/turnaround state
  machine + the A1 default-off byte-identity W7d, flag off). Built o3, branch
  `feat/recovery-ack-fix` off monitor `e2cd1c5`.
