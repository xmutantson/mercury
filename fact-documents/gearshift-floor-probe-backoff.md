# FIX-B — Floor-probe back-off (stop the CONFIG_0↔ROBUST limit cycle)

**Status**: IMPLEMENTED 2026-06-03 on branch `fix/floor-probe-backoff`, based off
the FIX-A commit `8ba8b46` (branch `fix/robust-dwell-batch`). The result is the
**FIX-A + FIX-B FLOOR STACK** for the upcoming HW muething A/B. PRIMARY proof is
the in-process synthetic-fire `--test-probe-backoff` (PB1-PB5, PB1 FAIL-BEFORE →
PASS-AFTER proven). FTRT-sim integration validation is INTENTIONALLY NOT chased
for FIX-B (see §8); delivered-rate contribution is measured on HW.

**Plan source**: `GEARSHIFT_FIX_IMPLEMENTATION_PLAN.md` PHASE 1 / Increment 1.2.
The plan's line numbers were written against monitor `0c75cc2`; they SHIFTED on
this branch. §5 below records the AS-BUILT file:line. Where a plan line disagrees
with §5, §5 is authoritative.

**Companion docs**: `gearshift-start-and-recovery.md` (the clean-batch viability
gate, BREAK panic latch, anchor-raise producer this fix latches onto),
`gearshift-climb-engine.md` (the §15 over-climb clamps INV-1 must not regress, the
§10 anchor-DEMOTE downward escape INV-2 must preserve), `data-flow-robust-tier-arq-batch.md`
(FIX-A, the sibling floor fix in the same stack).

---

## §1 The bug (the limit cycle this fix breaks)

At the robust/OFDM boundary, with the data-viable anchor parked at the ROBUST
tier (ROBUST_0/1/2), the climb engine probes UP exactly one rung — CONFIG_0, the
lowest OFDM config (`config_ladder_up(ROBUST_2)==CONFIG_0`, ladder
`common_defines.h:153-158`). The anchor+1 clamp PERMITS this probe (CONFIG_0 index
3 == anchor ROBUST_2 index 2 + 1). At a deep-SNR floor (≈WGN:-10) CONFIG_0 cannot
pass OFDM data, so:

1. The FRAME-UP / LADDER-UP probe to CONFIG_0 is issued.
2. CONFIG_0 fails (SET_CONFIG ACK times out, or the first DATA batch NACKs / its
   MFSK-ACK-PAT never arrives).
3. The fail handler panic-recovers DOWN through BREAK to the robust floor.
4. A clean robust batch advances the anchor and resets the climb counters.
5. The climb re-probes CONFIG_0 **on the next eligible cycle** — GOTO 1.

This is a CONFIG_0↔ROBUST_0/2 limit cycle that burns the floor's scarce airtime
on a probe that is PROVEN to fail. HW evidence (WGN:-10 config_counts):
`ROBUST_0:6 ROBUST_2:4 CONFIG_0:4` — ~29% of the airtime spent on the doomed
CONFIG_0 probe + its BREAK recovery.

**Root cause**: nothing remembers that a specific up-rung was JUST proven to fail.
Each cycle re-probes on a FIXED cadence (the FRAME-UP threshold / block counter),
so a proven-failed rung is hammered every cycle.

**Fix (FIX-B)**: a PER-RUNG exponential back-off. The first up-probe failure
suppresses re-probing of THAT rung for `PROBE_BACKOFF_MS_INIT` (~8 s ≈ one robust
dwell cycle), doubling on each repeat fail up to `PROBE_BACKOFF_MS_CAP` (~120 s).
The instant ANY clean OFDM batch is delivered, the entire back-off resets (the
channel proved an OFDM rung recovered → no rung is "proven-failed" anymore).

This is **root-cause, not threshold-masking** (CLAUDE.md §2): it does not change
WHEN a rung is considered failed, nor relax any failure threshold, nor trim any
SACK/RSP/PTT margin. It only spaces out the re-probe of a rung the existing
machinery already decided is failing, on a cadence keyed to channel evidence.

---

## §2 Mechanism summary

A per-rung deadline array `probe_backoff_until_ms[FULL_CONFIG_LADDER_SIZE]`
(indexed by `config_ladder_index(cfg)` ∈ [0,20)) holds the `opt_now_ms()`
timestamp before which each rung must NOT be re-probed UP-ward (0 = no back-off).
A scalar `probe_backoff_ms` is the CURRENT exponential window. The predicate
`probe_rung_suppressed(cfg)` AND-s into the EXISTING up gate at all three CMD
up-decision sites; the producers `probe_backoff_arm(cfg)` (on up-probe fail) and
`probe_backoff_reset()` (on clean OFDM) drive the array.

---

## §3 The shared state (this fix's structure)

```cpp
// include/datalink_layer/arq.h:1956-1957
unsigned long long probe_backoff_until_ms[FULL_CONFIG_LADDER_SIZE];  // per-rung re-probe deadline (opt_now_ms units)
int probe_backoff_ms;                                                // current exponential window (ms)
```

- `FULL_CONFIG_LADDER_SIZE == 20` (`common_defines.h:159`). The array is indexed by
  `config_ladder_index(cfg)` which returns [0,20) for any ladder config and -1
  otherwise. **The plan's `FULL_CONFIG_LADDER_LEN` does not exist** — the real
  macro is `FULL_CONFIG_LADDER_SIZE`. Index -1 (off-ladder cfg) is handled
  defensively in all three accessors (never written, never read, never suppressed).
- CMD-ONLY state. The climb-control loop lives entirely on the commander
  (`arq_commander.cc`); the RSP has no climb decision and never reads/writes this.
  No wire transport, no RSP mirror needed (unlike FIX-A's batch, which the RSP must
  mirror). This is why FIX-B touches ZERO RSP code and adds NO new wire op.

### §3.1 Constants (`common_defines.h:424-425`)

```cpp
#define PROBE_BACKOFF_MS_INIT 8000     // first-fail window ≈ one robust dwell cycle
#define PROBE_BACKOFF_MS_CAP  120000   // worst-case re-probe latency bound
```

**OR-5 / [Q1]: these are STARTING values, swept-deferred.** They are NOT
magic-numbered final constants (CLAUDE.md §1). The rationale (INIT long enough to
do real floor work between probes / short enough to re-probe a recovering channel
promptly; CAP bounds the worst case when the channel improved but delivered no
clean OFDM batch to trigger the reset) is documented at the #define. They are to
be SWEPT in the FTRT sim / the HW floor-stack A/B before being treated as final.
See §6.

---

## §4 The three accessors (all PURE except the two producers' writes)

`arq.h:1966-1991`:

- `bool probe_rung_suppressed(int cfg) const` — PURE / const. `idx =
  config_ladder_index(cfg); if(idx<0) return false; return opt_now_ms() <
  probe_backoff_until_ms[idx];`. **INV-4: reads `opt_now_ms()` ONLY** (the virtual
  clock the FTRT sim drives, `arq.h:2239`), NEVER `cl_timer` (wall-clock). This is
  the clock the optimizer's bytes/time measurement also uses, so under `-x sim` the
  back-off measures CHANNEL time, not wall-clock time.
- `void probe_backoff_arm(int cfg)` — sets `probe_backoff_until_ms[idx] =
  opt_now_ms() + probe_backoff_ms`, then doubles `probe_backoff_ms` capped at
  `PROBE_BACKOFF_MS_CAP`. Off-ladder cfg ignored.
- `void probe_backoff_reset()` — zeroes the whole array, resets
  `probe_backoff_ms = PROBE_BACKOFF_MS_INIT`.

---

## §5 PRODUCER / CONSUMER / INVARIANT AUDIT (CLAUDE.md §5, the five questions)

### §5.1 Producers (every code path that WRITES this state)

| # | Site (file:line) | What | Which write |
|---|---|---|---|
| P0a | `arq_common.cc:308-309` | ctor seed | `probe_backoff_ms=INIT`; array=0 |
| P0b | `arq_common.cc:3197-3198` | `reset_session_state()` | `probe_backoff_ms=INIT`; array=0 (per-session reset; mirrors ctor) |
| P1 | `arq_commander.cc:2422` | `probe_backoff_arm(negotiated_configuration)` | ARM — SET_CONFIG-ACK-timeout fail (the up-probe SET_CONFIG NAcked) |
| P2 | `arq_commander.cc:3307` | `probe_backoff_arm(data_configuration)` | ARM — NACK data-fail ("FRAME UP DATA FAILED": just-applied up-rung can't pass DATA) |
| P3 | `arq_commander.cc:3493` | `probe_backoff_arm(data_configuration)` | ARM — pat data-fail ("FRAME UP DATA FAILED (pat)": same via MFSK-ACK-PAT path, after the §7.13.33 single retry already failed) |
| P4 | `arq_commander.cc:3734` | `probe_backoff_reset()` | RESET — clean OFDM batch (inside the `promotion_allowed_on_batch()` clean branch, after the anchor-raise, gated `is_ofdm_config(current_configuration)`) |
| Pt | `arq_commander.cc:6313/6347/6360/...` | unit-test driver | drives all of the above synthetically (no channel) |

**ARM-site config choice (the failed up-probe rung):**
- P1: `negotiated_configuration` is the rung the SET_CONFIG targeted (the UP
  probe). Armed BEFORE `working_config = config_ladder_down(negotiated_configuration)`
  overwrites `negotiated_configuration` (`arq_commander.cc:2429`).
- P2/P3: `data_configuration` is the just-applied FRAME-UP rung that failed to
  carry DATA. Armed BEFORE `working_config = config_ladder_down(data_configuration)`
  overwrites `data_configuration` (`arq_commander.cc:3314` / `:3500`).
- In all three, arming BEFORE the overwrite keys the back-off to the rung the
  climb ACTUALLY tried, not the recovered-down rung.

### §5.2 Consumers (every code path that READS this state)

All reads are `probe_rung_suppressed(proposed)` AND-ed into the EXISTING up gate.
There are exactly THREE production up-decision sites (the same three the §15
over-climb clamps and the Option-B +1 clamp gate):

| # | Site (file:line) | Gate variable | Up-decision path |
|---|---|---|---|
| C1 | `arq_commander.cc:3790` | `frame_ceiling_blocked = true` | FRAME-UP (frame-level gearshift, the fast climb) |
| C2 | `arq_commander.cc:5275` | `ceiling_blocked = true` | v2 `policy_evaluate_axis1()` LADDER-UP (sack_v2 sessions) |
| C3 | `arq_commander.cc:5112` | `ceiling_blocked = true` | legacy v1 inline LADDER-UP twin (`finalize_block_commander`, non-v2 sessions) |
| Ct | `arq_commander.cc:6324`… | predicate asserts | unit-test |

Each consumer was ALREADY computing a `*ceiling_blocked` boolean from the
proven-ceiling cap + the Option-B `last_data_viable_config+1` clamp; FIX-B appends
one more `if(probe_rung_suppressed(proposed)) *ceiling_blocked = true;` line
IMMEDIATELY AFTER the +1 clamp, matching the existing `= true` idiom. No new gate
variable, no restructuring.

### §5.3 Valid states (especially the default-init / before-any-producer state)

- **Before P0a (ctor)**: garbage (uninitialized array). P0a runs in the ctor before
  any consumer can read (consumers are on the connected data path). SAFE.
- **After ctor / after `reset_session_state`**: `probe_backoff_ms == INIT`, every
  `probe_backoff_until_ms[i] == 0`. `probe_rung_suppressed(cfg)` for any cfg →
  `opt_now_ms() < 0` → FALSE (opt_now_ms is unsigned and ≥0; 0 deadline means "never
  suppressed"). So a fresh session suppresses NOTHING — the climb behaves exactly as
  pre-FIX-B until the first up-probe fail. SAFE.
- **After an arm**: the armed rung's deadline = now+window (>0), every other rung
  still 0 → only the armed rung is suppressed. SAFE (per-rung isolation).
- **Off-ladder cfg** (`config_ladder_index<0`, e.g. CONFIG_NONE=-1 sentinel): all
  three accessors early-return (never suppressed, never armed). SAFE.
- **opt_now_ms wraparound**: unsigned 64-bit ms; wraps in ~5.8×10^8 years. N/A.

### §5.4 Invariants the consumers assume + that the producers maintain

- **INV-1 (cannot regress over-climb / monotone-OFF)**: `probe_rung_suppressed` is
  AND-ed into the EXISTING gate as `... || (already-blocked)`, and only ever SETS
  `*ceiling_blocked = true`. It can therefore only turn a PERMITTED probe OFF; it
  can NEVER turn a BLOCKED probe ON. The anchor/+1/ceiling clamps
  (`supershift_proven_ceiling`, `last_data_viable_config+1`, `max_config_override`,
  `config_is_at_top`) are UNTOUCHED — FIX-B adds NO code that lowers any of them and
  removes none. Proven in PB1: with CONFIG_0 suppressed the gate parks at ROBUST_2;
  with the back-off cleared (the conjunct reverted) the SAME gate promotes to
  CONFIG_0 — i.e., the back-off is purely additive suppression.
- **INV-2 (deep-SNR DOWNWARD escape intact)**: FIX-B adds NO `probe_backoff*`
  reference to ANY BREAK / demote / panic site. The downward escapes —
  `break_target_with_anchor()` panic-bypass (`arq.h`, panic at
  `breaks_since_last_data_success>=2`), the `break_drop_step=100` panic latch
  (`arq_commander.cc:3543`), and `anchor_demote_target()` (`arq.h:605`, the §10
  K-consecutive-fail demotion) — take NO back-off input and are unmodified. The
  back-off CANNOT trap the link above the floor: it only ever blocks UPWARD probes;
  every DOWNWARD path remains free. Proven in PB5: with a back-off armed, the panic
  floor still reaches ROBUST_0 and the anchor-DEMOTE still reaches ROBUST_0,
  byte-identical to the no-back-off case.
- **INV-4 (virtual-clock semantics)**: `probe_rung_suppressed` reads `opt_now_ms()`
  ONLY. Under `-x sim` that is the FTRT virtual clock (channel time); in production
  it is `CLOCK_MONOTONIC_RAW`. This matches the optimizer's own time base, so the
  back-off windows mean "channel seconds" consistently in sim and on HW. Using
  `cl_timer` instead would have made the sim's fast relay re-probe ~50× too soon.
  PB2 exercises this dependency directly (advances the sim clock via
  `sim_clock_add_samples` and observes suppression lift at the virtual deadline).
- **Per-session reset invariant**: a new connection must NOT inherit a prior
  session's suppression. P0b (`reset_session_state`) mirrors the ctor — verified it
  sits alongside the FIX-A robust-dwell reset and the climb-state resets
  (`last_data_viable_config`, `clean_batches_*`, `supershift_proven_ceiling`).

### §5.5 What the fix changes (which assumption each consumer's behavior alters)

The ONLY behavioral change: a proven-failed up-rung is temporarily un-probeable.
Walked each consumer (C1/C2/C3): each already declines to promote when its
`*ceiling_blocked` is true (it falls through to "hold / ceiling-recovery"); the
extra suppression simply makes that decline fire on a rung that the existing code
would otherwise re-attempt. No consumer assumes a probe WILL be attempted; all
three have a well-defined "blocked" branch. The ceiling-recovery counter
(`ceiling_success_count`) is on the BLOCKED branch and is independent of FIX-B
(FIX-B does not touch it). No consumer reads the back-off for any purpose other
than the gate, so there is no second consumer to break.

---

## §6 OR-5 sweep note (constants are starting values)

`PROBE_BACKOFF_MS_INIT=8000`, `PROBE_BACKOFF_MS_CAP=120000`. To be swept on:
1. the FTRT ARQ sim (cheap), and
2. the HW floor-stack muething A/B (the arbiter, where CONFIG_0 fails for a REAL
   RF reason — see §8).

Sweep axes: INIT ∈ {4s, 8s, 16s} (trade re-probe responsiveness vs airtime saved),
CAP ∈ {60s, 120s, 240s} (worst-case latency for a channel that improved without a
clean OFDM batch). Metric: deep-SNR delivered B/min ↑ with no clean/mid regression
and no increase in time-to-recover when the channel genuinely improves. The reset
on clean OFDM should make the link insensitive to CAP in practice (any real
improvement triggers a reset long before CAP), so CAP is mostly a safety bound.

---

## §7 Validation — unit tests (PRIMARY proof)

`--test-probe-backoff` → `test_probe_backoff()` (`arq_commander.cc:6276`),
mirrors `test_data_anchored_promote`. Enables the SIM virtual clock so
`opt_now_ms()` is deterministic and advanceable (PB2). All 20 assertions PASS:

- **PB1 (FAIL-BEFORE → PASS-AFTER)**:
  - PB1a: armed CONFIG_0 reports suppressed (true). PASS.
  - PB1b PASS-AFTER: with CONFIG_0 armed, the REAL `policy_evaluate_axis1()` UP gate
    parks at ROBUST_2 (does NOT promote). PASS.
  - PB1c: after `probe_backoff_reset()` CONFIG_0 is NOT suppressed. PASS.
  - PB1d FAIL-BEFORE: WITHOUT the back-off (== the conjunct reverted, i.e. pre-FIX-B
    code) the SAME gate DOES promote to CONFIG_0. PASS. **This is the explicit
    FAIL-BEFORE: it demonstrates that absent the `&& !probe_rung_suppressed`
    conjunct the gate exhibits the limit-cycle re-probe; the conjunct is the sole
    thing parking the link.** (A source-level revert reproduction is in §7.1.)
- **PB2 (virtual-clock elapse)**: armed CONFIG_0 suppressed → still suppressed at
  INIT-1000 ms of virtual time → suppression LIFTS after the window elapses. PASS.
- **PB3 (exponential + cap)**: window INIT=8000 → 16000 → 32000 after successive
  arms, clamps at CAP=120000 after 20 arms. PASS.
- **PB4 (reset)**: all per-rung deadlines zeroed, window back to INIT, no rung
  suppressed. PASS.
- **PB5 (INV-2)**: with a back-off armed, the panic floor and anchor-DEMOTE targets
  are byte-identical to the no-back-off case AND both reach ROBUST_0. PASS.

Result: `[TEST-PROBE-BACKOFF] ALL PASS (0 failures)` rc=0.

### §7.1 Source-level FAIL-BEFORE reproduction (recorded)

To rigorously confirm the conjunct is load-bearing, the v2 gate conjunct
(`arq_commander.cc:5275`) was temporarily removed, rebuilt, and `--test-probe-backoff`
re-run: PB1b FAILED (gate promoted to CONFIG_0 = 0, want ROBUST_2 = 102) while
PB1d still passed. Restoring the conjunct returns PB1b to PASS. This proves the
test's PB1b assertion FAILS-BEFORE the fix and PASSES-AFTER. (See §9 for the exact
result lines from the build run; the revert was reverted — branch HEAD has the
conjunct in place.)

### §7.2 Regression suites (all green on the FIX-A+FIX-B binary)

- `--test-climb-engine` (FIX-A Parts D'1-D'6 + the climb-engine Parts A-P): ALL PASS.
- `--test-sim-clock`: 7/7 (0 failed).
- `--test`: see §9.

---

## §8 Why FTRT-sim integration validation is NOT chased for FIX-B (KNOWN)

The FTRT sim's CONFIG_0 up-probe fails for a Q3-ARTIFACT reason — the OFDM
data-ACK turnaround coupling defect — NOT real SNR. So a sim thrash-reduction
would not cleanly attribute to FIX-B's mechanism. The unit tests are the
correctness proof; FIX-B's delivered-rate contribution is measured on HW (the
floor-stack muething A/B, where CONFIG_0 fails for a real RF reason). A quick sim
sanity run is acceptable but NOT a gate.

---

## §9 AS-BUILT record (branch / HEAD / results)

- Branch: `fix/floor-probe-backoff`, based off `8ba8b46` (FIX-A). HEAD: see commit.
- Files touched:
  - `include/common/common_defines.h:407-425` — the two constants + rationale.
  - `include/datalink_layer/arq.h:1943-1991` — members + 3 accessors; `:628` test decl.
  - `source/datalink_layer/arq_common.cc:305-309` (ctor), `:3195-3198` (reset_session_state) — seed/zero.
  - `source/datalink_layer/arq_commander.cc` — arm P1 `:2422`, P2 `:3307`, P3 `:3493`;
    gate C1 `:3790`, C2 `:5275`, C3 `:5112`; reset P4 `:3734`; `test_probe_backoff()` `:6276`.
  - `source/main.cc:354` (flag), `:835` (parse), `:1880` (dispatch).
- Build: `bash build.sh o3` → clean (only pre-existing warnings). Installed to
  `C:\Program Files\Mercury\mercury.exe`.
- Tests: `--test-probe-backoff` ALL PASS (20/20, rc=0); `--test-climb-engine` ALL
  PASS (rc=0); `--test-sim-clock` 7/7 (0 failed). The full `--test` is a heavy
  Monte-Carlo BER/FEC-reach characterization suite (minutes per run) that is
  ORTHOGONAL to this CMD-side gearshift control-loop change (FIX-B touches no
  PHY/BER path); the portion that ran emitted only `[ASSERT OK]` lines (no
  failures) — it was not run to completion as it is not a gate for this change.
- FAIL-BEFORE source reproduction (§7.1): with the v2 gate conjunct
  (`arq_commander.cc:5275`) commented out and rebuilt, `--test-probe-backoff` →
  `PB1b FAIL (got=0 want=102)`, rc=1 (the suppressed CONFIG_0 probe is NOT blocked
  and the gate promotes to CONFIG_0). Restoring the conjunct → PB1b PASS, rc=0.
  Binary byte-size identical before/after the demo (29754126) — the revert was
  reverted; branch HEAD has the conjunct in place.

## §10 Residual risk

- The HW floor-stack A/B must confirm the back-off actually reduces CONFIG_0
  airtime at WGN:-10 AND does NOT slow recovery when the channel improves (the
  clean-OFDM reset is the safety valve; if the channel improves but never delivers
  a clean OFDM batch — e.g. it improves only to a still-sub-CONFIG_0 SNR — the rung
  stays suppressed up to CAP, which is intended). Low risk: the reset fires on the
  first clean OFDM batch, which is exactly the evidence that the boundary improved.
- Constants are swept-deferred (§6) — do not treat 8s/120s as final.
- INV-1/INV-2/INV-4 are unit-proven; the HW A/B is the field confirmation.
