# Fact Document: Gearshift CLIMB Engine — off-ROBUST_0 multi-rung promotion

**Status**: Authoritative as of 2026-05-30 on `fix/climb-engine` (branched from
`fix/cfg16-nv-restore` @ 05012a9). Investigation of the "unpinned cascade can't
climb off ROBUST_0 into the OFDM tier" failure. Supersedes the per-single
analyses on `fix/climb-delivery-anchored` (C1), `fix/climb-batch1-robust` (C2),
`fix/climb-combined` (C3) — all three FAILED on the IONOS wire. **§10/§11
(2026-05-30, stacks on 57f938f)**: the deep-SNR down-hysteresis fix (climb
follow-up #2) — the WGN:-10 CONFIG_0↔ROBUST_0 thrash. The climb now works at
clean (Parts A-D verified) but thrashed at WGN:-10; §10 adds the missing anchor
DEMOTION producer + §11 the sustained-anchor RAISE gate. In-process verified
(Parts E/F); wire confirmation is the parent's pause-cal hardware test @WGN:-10.
**§12 (2026-05-30, stacks on b1ab550)**: the ADAPTIVE FRAME-UP threshold (climb
follow-up ③) — the climb is correct but SLOW at high SNR (uniform 3 clean
batches/rung). §12 restores the deleted forward-AARF half: fast-probe (threshold→1)
when delivery is PROVEN sustained-clean at the rung, conservative (the AARF-doubled
member) otherwise. Read-time only — does not touch the +1 clamp (fork ①) nor fight
the back-off. In-process verified (Part I); wire speedup is the parent's hardware
test with the fixed cascade-bench timing.
**§13 (2026-05-31, stacks on d12c042)**: the CONTROLLED ELEVATOR (fork ①,
the user-APPROVED fast-climb fix) — the ONE change that partially relaxes
af14a9e's anchor+1 clamp. At the SUPERSHIFT re-trigger ONLY, under a
HIGH-CONFIDENCE-SNR predicate, the modem JUMPS multiple rungs toward the
SNR-ideal config in one shot (capped by supershift_proven_ceiling AND the
WB/NB ceiling), instead of crawling +1/rung. PURE helper
`supershift_retrigger_target` (arq.h); FRAME-UP/LADDER-UP keep their strict +1
clamps; the jump never raises the anchor; provably INERT at deep/invalid SNR.
In-process verified (Part J, fail-before/pass-after); the over-climb safety
(@WGN:-10) + the fast-jump (@WGN:30) are the parent's HARDWARE verification.
**§14 (2026-05-31, NEW worktree `fix/fast-probe` off 540779c)**: the REAL
FAST-PROBE — (A1) repairs the CMD arm asymmetry that left `SNR_uplink` stuck at
−99.9 on the unpinned ladder (the RSP already suffixes the SNR; the CMD's
TURBO_DONE arm-gate diverged from the RSP's TURBO_FORWARD send-gate), and (B)
fires the §13 elevator from the data-anchored FRAME-UP path (which runs every
clean batch, unlike the dormant SUPERSHIFT re-trigger) via a SHARED
`elevator_target_from_snr()` extracted from §13's cap-chain. No wire change (A1);
no anchor raise / no turbo storm (B). In-process verified (Part J',
fail-before/pass-after); the wire proofs (SNR_uplink leaving −99.9; @WGN:-10
no-thrash; @WGN:30 multi-rung jump) are the parent's HARDWARE verification.
**§15 (2026-05-31, stacks on 94e80a6)**: §14 REGRESSED the af14a9e over-climb guard
at WGN:-10 (control-plane MFSK-suffix SNR over-reports OFDM-data viability → the
elevator jumped CFG_0→4→9 from a ROBUST anchor). §15 re-asserted the data-anchor
INSIDE `supershift_retrigger_target` (`is_ofdm_config(anchor)` on
`high_confidence_jump` + a `RETRIGGER_MAX_LEAP` bound). In-process verified (Part
J''); necessary-but-INSUFFICIENT — it gated ONLY the elevator helper.
**§16 (2026-05-31, stacks on edb8600)**: ANCHOR-TIER-CORRUPTION — the two siblings
§15's elevator-only gate did not cover. ROOT-1: the anchor-raise producer
(`arq_commander.cc:3617`) credited the LIVE `current_configuration`, which races
ahead of the delivered batch once a FRAME-UP/turbo SET_CONFIG is queued — a
ROBUST-tier clean ACK could poison the anchor into the OFDM tier, opening the §15
gate. Fix: a PURE `data_anchor_raise_target()` credits `clean_batches_config` (the
streak's HOME, the authoritative delivered config) and refuses a ROBUST→OFDM cross
whose live tier disagrees. ROOT-2: the turbo-forward ladder
(`arq_commander.cc:4602-4621`) had NO `last_data_viable_config` clamp (SNR-SUPERSHIFT
/ step-1 ratcheted CFG_4→9 unbounded); fix routes its target through the SAME shared
`supershift_retrigger_target` chokepoint (anchor+1 at a robust anchor, anchor+MAX_LEAP
at an OFDM anchor). PRE-EXISTING bug exposed by the ③ fast-probe (446887c climbed too
slowly to reach the rocket). In-process verified (Parts K/K'/K'', fail-before/
pass-after — incl. a temp-revert proving the production helper is load-bearing); the
wire proofs (WGN:-10 holds ROBUST_0 + no CFG_4→9 ratchet; WGN:30 keeps the fast ~3k
climb) are the parent's HARDWARE re-test.

This doc owns the producer/consumer/invariant facts for the CLIMB promotion
state shared CMD↔RSP: `last_data_viable_config`, `consecutive_data_acks`,
`gear_shift_blocked_for_nBlocks`, `last_batch_fully_acked`, `data_batch_size`
at robust configs, the SACK bsi window during promotion, and (§10/§11) the new
anchor-hysteresis counters `anchor_consec_break_fails`,
`clean_batches_at_current_config` / `clean_batches_config`.

---

## §1 The climb ladder + the promotion gates

Unified ladder (`common_defines.h:101`):
`ROBUST_0(idx0) ROBUST_1(1) ROBUST_2(2) CONFIG_0(3) ... CONFIG_16(19)`.

Two promotion engines run on the CMD for a `-R` SUCCESS_BASED_LADDER session:

- **FRAME-UP** (`arq_commander.cc:3473-3536`): after `frame_shift_threshold=3`
  (`arq_common.cc:337`) consecutive CLEAN data ACKs, advance exactly one rung.
  Gated by:
  - `promotion_allowed_on_batch(last_batch_fully_acked)` (`arq.h:621`, returns
    the bool) — only a CLEAN (all-ones) batch counts; a partial SACK does not
    advance `consecutive_data_acks` (`arq_commander.cc:3498`).
  - The **+1 anchor clamp** (`arq_commander.cc:3485-3487`): `frame_ceiling_blocked`
    if `config_ladder_index(proposed) > config_ladder_index(last_data_viable_config)+1`.
- **LADDER-UP** (`policy_evaluate_axis1`, `arq_commander.cc:4829-4971`; v1 twin
  inline at `:4667-4803`): per finalized block, if
  `success_rate_data_clean > gear_shift_up_success_rate_precentage` (85%) AND
  `gear_shift_blocked_for_nBlocks >= gear_shift_block_for_nBlocks_total`,
  advance one rung. Same +1 anchor clamp at `:4857-4859` / `:4700-4702`.

**The anchor is the gatekeeper.** `last_data_viable_config` is the highest rung
at which a CLEAN batch was confirmed delivered this session. Both engines refuse
to climb more than +1 past it. So the climb advances iff the anchor advances,
and the anchor advances iff a clean batch is credited at the current rung.

### §1.1 The sole anchor-raise producer

`arq_commander.cc:3437-3439`, inside the `data_ack_received==YES` branch, gated
by `promotion_allowed_on_batch(last_batch_fully_acked)`:
```cpp
if(config_ladder_index(current_configuration) > config_ladder_index(last_data_viable_config))
    last_data_viable_config = current_configuration;
```
Init = `init_configuration` (`arq_common.cc:352`, ctor; `:2910` reset_session_state).
This is the ONLY non-test writer. A failed/partial batch never reaches it.

### §1.2 `last_batch_fully_acked` (the clean-credit signal)

Reset FALSE at each batch-TX start (`arq_commander.cc:1246`, `:1742`), ctor,
reset_session_state. Set TRUE ONLY at the clean acceptance sites:
- MFSK all-ones suffix funnel `:2951` (entry: `v2_ack_pat_pre_detected`)
- bare-pattern data-ACK arm `:2951` (WB CRC-gated)
- LDPC ACK_RANGE `:3053`, ACK_MULTI `:3078`

---

## §2 Bug 3 (DORMANCY) — ROOT CAUSE: Axis-2 grows batch at the promoted robust rung

**Symptom (IONOS singles campaign, C3=8b1f1ce):** with C1+C2 the link climbed
exactly ONE rung (ROBUST_0→ROBUST_1), then only ONE ladder-hold evaluation fired
at config 101, then ~280 s of silence with no further promotion despite the
[Axis-2] ring resetting ~12 more times.

### §2.1 The chain (file:line)

1. **C3 dropped the Axis-2 robust guard.** The standalone C2 (`d012022`) added
   `if(is_robust_config(current_configuration)) return;` at the top of
   `policy_evaluate_axis2`. The COMBINED C3 (`8b1f1ce`) did NOT include it
   (verified: C3's `+is_robust_config` additions are only the two batch-floor
   gates + the prev-path; no `policy_evaluate_axis2` edit). So on the wire,
   Axis-2 was free to run at robust configs.

2. **Axis-2 is fed on every clean batch.** `arq_commander.cc:2990-2992`:
   `if(sack_v2_enabled) policy_evaluate_axis2(data_batch_size, data_batch_size);`
   — gated ONLY on `sack_v2_enabled`, NOT on config. At batch=1 a clean MFSK
   delivery gives `partial_rate = (1-1)/1 = 0` → `is_good` → `axis2_consecutive_good_batches++`
   (`:5124-5129`).

3. **FRAME-UP does NOT reset Axis-2.** Only LADDER-UP / LADDER-DOWN / BREAK call
   `policy_axis1_supremacy_on_move` (`:5008` resets `axis2_consecutive_good_batches=0`
   + sets `axis2_cooldown_batches`). The FRAME-UP block (`:3508-3533`) calls
   neither. So the Axis-2 good-run accumulated at ROBUST_0 carries into ROBUST_1
   with no cooldown.

4. **At ROBUST_1, Axis-2 grows the batch.** After `AXIS2_UP_GOOD_RUN=4`
   (`arq.h:1119`) good batches, `want_up` (`:5187`) fires:
   `data_batch_size(1) + AXIS2_STEP(5) = 6 <= AXIS2_BATCH_CEIL(32)`. CMD sets
   `data_batch_size=6` and sends SET_LINK_PARAMS to RSP.

5. **RSP clamps to the AXIS2 floor → MISMATCH.** RSP SET_LINK_PARAMS handler
   (`arq_responder.cc:2609-2615`) clamps `target` to
   `[AXIS2_BATCH_FLOOR=10, AXIS2_BATCH_CEIL=32]` → `target=max(6,10)=10`. Now
   **CMD batch=6, RSP batch=10** at ROBUST_1.

6. **Batch mismatch kills clean delivery.** At the MFSK cliff P(batch clean)=p^N;
   with mismatched N the ACK-GATE never PASSes a full batch → no all-ones clean
   ACK → `last_batch_fully_acked` never TRUE at ROBUST_1 → `last_data_viable_config`
   frozen at ROBUST_0.

7. **The +1 anchor clamp (correctly) holds.** With anchor=ROBUST_0(idx0),
   `proposed=ROBUST_2(idx2)`: `2 > 0+1` → blocked in BOTH FRAME-UP (`:3486`) and
   LADDER-UP (`:4858`). The engine is now DORMANT — exactly as observed. The
   clamp is not the bug; it is the messenger. The bug is that batch growth
   destroyed clean delivery one rung up.

The "~12 ring resets" in the symptom = the Axis-2 partial-rate ring being reset
each time Axis-2 attempted a SET_LINK_PARAMS move (each move fires the supremacy
hook on the OTHER side / clears the ring).

### §2.2 Why local 17/17 unit tests MISSED it

The existing `test_data_anchored_promote` (B1/B2/B3, `:5651`) **pre-sets
`last_data_viable_config` by hand** before each `policy_evaluate_axis1()` call.
It asserts the clamp math GIVEN an advancing anchor — it never drives the
end-to-end loop where a clean batch must RAISE the anchor. The dormancy lives
exactly in the un-tested gap: the anchor fails to advance because batch growth
broke clean delivery. No existing test exercises Axis-2 at a robust config, nor
a multi-rung climb where rung N+1's promotion depends on rung N being credited.

### §2.3 Fix

Restore the standalone-C2 robust guard at the top of `policy_evaluate_axis2`
(`arq_commander.cc`):
```cpp
if(is_robust_config(current_configuration)) return;
```
Rationale: "MFSK modes keep batch_size=1 for pattern-ACK optimization"
(`arq_common.cc:1259`); `load_configuration` pins batch=1 at robust
(`arq_common.cc:1229-1234`) and the OFDM batch-scaling at `:1269` is already
`!is_robust_config`-gated. Axis-2 has no business resizing batch at robust. This
makes batch stay 1 at every robust rung regardless of the missing FRAME-UP
supremacy call, so clean MFSK batches keep being credited and the anchor keeps
advancing. OFDM (CONFIG_0..16) is unchanged — Axis-2 runs there as before.

---

## §3 Bug 2 (CMD/RSP batch-predicate) — CONFIRMED in cfg16-nv-restore

The SACK batch-floor recompute in the TEST_CONNECTION handler runs
unconditionally once SACK negotiates (and SACK is default-on):
- CMD: `arq_commander.cc:3848-3863` (`set_data_batch_size(radio_batch_size)`,
  floor 5). NO `is_robust_config` guard in the base.
- RSP: `arq_responder.cc:2091-2106` (parallel). NO guard either.

At a `-R` connect (config = ROBUST_0) this clobbers the deliberate
`set_data_batch_size(1)` from `load_configuration` → batch≥5 at ROBUST_0. At
batch≥5 a clean all-ones MFSK ACK needs all 5 MFSK frames to survive first-pass
at the floor SNR — which essentially never happens — so the climb never gets a
single clean batch to start with. (This is the upstream block that prevents even
the FIRST rung; Bug 3 is the block on subsequent rungs.)

**Fix**: gate BOTH blocks on `!is_robust_config(...)`. ~~CMD gates on
`negotiated_configuration`, RSP on `current_configuration`; at TEST_CONNECTION
time both equal the established connect config, so they agree~~ **[WRONG — struck
2026-05-30, this was the 4th wire failure's root cause. See
`data-flow-batch-size.md` §4: on a fresh unpinned `-g -R` connect,
`negotiated_configuration` is its CTOR DEFAULT `CONFIG_0` (`arq_common.cc:281`),
NOT the connect config — the connect path never writes it (only teardown
`:443` / BREAK `:205,:297` / optimizer `:530` / turboshift `:3623` do). So
`is_robust_config(negotiated_configuration)=false` at ROBUST_0 connect → CMD
recompute RAN → CMD batch=5 while RSP (gated on `current_configuration=ROBUST_0`)
correctly stayed batch=1 → CMD/RSP mismatch → first ROBUST_0 block fails →
climb never starts. CORRECTED FIX: gate the CMD block on
`current_configuration` too (== ROBUST_0 at connect, the SAME var the RSP block
and the Axis-2 guard read), and enforce robust⇒batch=1 at the
`set_data_batch_size()` chokepoint (`arq_common.cc`) so no path can diverge.]**
(symmetry mandatory — an asymmetric override was historical Bug #9). This is the
standalone C2 batch-floor fix, verbatim.

Note: §2 (Axis-2 guard) and §3 (batch-floor guard) are the TWO halves of the
"keep batch=1 at robust" invariant. C3 shipped §3 but dropped §2 — that omission
IS the dormancy. The integrated fix ships BOTH.

---

## §4 Bug 1 (late completion-ACK on SACK-v2 prev path) — CONFIRMED, OFDM-tier only

On a multi-frame batch (OFDM, batch≥5) that loses a frame first-pass, the RSP
recovers it via the prev-storage path. Pre-fix, the prev-delivered completion
(`arq_responder.cc:721-773`, `[RSP-V2-PREV-DELIVERED]`) delivered to the app but
emitted NO ACK frame → CMD's `last_batch_fully_acked` stayed FALSE → no promotion
credit for a batch that actually got through.

CMD-side, even once the RSP emits a completion ACK, the MFSK ACK+SACK decode has
a **single dedupe tracker** `cmd_last_applied_sack_bsi` (`arq_commander.cc:2509`):
the PARTIAL SACK for bsi=B sets it (`:2545`); the later all-ones CLEAN
confirmation for the SAME bsi=B is then dropped as `duplicate` at `:2509-2511`
before it can reach the clean funnel (`:2516` all_ones → `v2_ack_pat_pre_detected`).
That is the "~1.3 s late, past the RX window, never credited" mechanism: even
in-window, the dedupe rejects it.

**Fix** (the standalone C1 design):
- RSP: after `[RSP-V2-PREV-DELIVERED]` emit a CLEAN all-ones MFSK ACK+SACK for
  the prev bsi (reuse `send_mfsk_ack_sack`; `send_ack_pattern` fallback). NOT
  WB-only — fires on the prev-delivered path whenever it runs (which is only at
  batch≥5, i.e. OFDM; robust is batch=1 and never uses the prev path).
- CMD: split the dedupe by event class. A CLEAN (all-ones) confirmation is a
  terminal transition that supersedes the partial; dedupe it ONLY against a new
  `cmd_last_applied_clean_bsi` (ctor-init -1). A repeated clean for the same bsi
  is still deduped (no double-count). The partial dedupe (`cmd_last_applied_sack_bsi`)
  is unchanged.

**Scope note**: Bug 1 only bites at batch≥5 (the prev path is dead at batch=1).
On the deep-SNR ROBUST_0/1/2 climb (batch=1), Bug 1 is inert. It matters once the
climb reaches the OFDM tier (CONFIG_0+) where batches are multi-frame and SACK
recovery is the norm. Shipping it now keeps the OFDM-tier climb honest and is
required for the "credited IN-window" assertion.

---

## §5 SNR-report interaction (38 dB → 14.6 dB on this base)

The cfg16-nv-restore commit (05012a9) corrected the LS-path SNR report from a
bogus ~38 dB to a sane ~14.6 dB. The gearshift consumes this via
`get_configuration(SNR)` (`telecom_system.cc:5536`), used ONLY in the turboshift
SNR-SUPERSHIFT targeting (`arq_commander.cc:3563`, `:4323`, `:4419`), always with
`- SUPERSHIFT_MARGIN_DB(6.0)`.

- Old bogus 38: `get_configuration(38-6=32)` → 32>13 → **CONFIG_16** (top). The
  SNR-probe would target the very top instantly — the over-climb/thrash that
  af14a9e's +1 anchor clamp was built to contain.
- New sane 14.6: `get_configuration(14.6-6=8.6)` → 8.6>6 → **CONFIG_13**.

**Conclusion**: the corrected report makes SNR-SUPERSHIFT strictly MORE
conservative (target drops 3 rungs at the same reported SNR) — it REDUCES
over-climb risk and does not introduce any new climb blockage. No success-based
climb gate (FRAME-UP / LADDER-UP) consults SNR; they use success rate + the
anchor. So the SNR correction is favorable-or-neutral to the climb, and the +1
anchor clamp remains the over-climb guard regardless of the SNR value. No gate
threshold needs adjustment for the SNR change. [verified — no SNR threshold in
the success-based path]

---

## §6 Cross-layer audit (CLAUDE.md §5) — the integrated 3-bug fix

Shared state changed: `data_batch_size` at robust (§2/§3), the SACK clean-confirm
dedupe + prev-path ACK (§4). Walk producers/consumers:

### 6.1 `data_batch_size` at robust configs

- **Producers**: `load_configuration` (`arq_common.cc:1231`, pins 1 at robust),
  OFDM batch-scaling (`:1269`, `!is_robust_config`-gated — untouched), the SACK
  TEST_CONNECTION recompute (CMD `:3858` / RSP `:2101` — NOW `!is_robust_config`-
  gated by §3 **on `current_configuration` for BOTH sides as of the 2026-05-30
  fix; the original §3 plan's `negotiated_configuration` for CMD was the 4th
  wire-failure bug — see `data-flow-batch-size.md` §4**), `policy_evaluate_axis2`
  (CMD `:5263` via SET_LINK_PARAMS — NOW `return`s early at robust by §2),
  **`set_data_batch_size()` itself (`arq_common.cc:561`) NOW the single chokepoint:
  clamps any robust write to 1**, RSP SET_LINK_PARAMS apply
  (`arq_responder.cc:2615`, driven only by CMD Axis-2; since CMD no longer moves
  batch at robust, RSP never receives a robust SET_LINK_PARAMS).
- **Consumers**: ACK-GATE expected-count (`arq_responder.cc:1373/1380`), the
  all-ones bitmap width (`arq_commander.cc:2513`, `arq_responder.cc:1656`),
  `messages_rx_prev` completion gate (`arq_responder.cc:550`, bound
  `i<data_batch_size`), `bump_bsi_and_transfer_prev` (`arq_common.cc:4085`).
- **Invariant restored**: at every robust rung, CMD batch == RSP batch == 1.
  With §2+§3, NO producer can move robust batch off 1. Both consumers that
  compute all-ones (`(1<<data_batch_size)-1`) therefore agree (both =0x1), and
  the ACK-GATE `data_batch_size>1` partial branch (`:1412`) is never taken at
  robust → clean PASS funnel always runs → clean credit fires. This is the
  invariant the dormancy violated.

### 6.2 SACK clean-confirm dedupe (§4)

- **Producer**: RSP prev-delivered clean ACK (`arq_responder.cc:~773`, new) and
  the existing clean-batch ACK funnel (`:1644`).
- **Consumers**: CMD MFSK ACK+SACK decode (`arq_commander.cc:2494-2576`). New
  field `cmd_last_applied_clean_bsi` (arq.h, ctor-init -1) splits clean-dedupe
  from partial-dedupe. The all-ones CLEAN branch (`:2516`) checks
  `rx_bsi != cmd_last_applied_clean_bsi` instead of inheriting the partial
  `duplicate`. Partial branch (`:2533`) is unchanged.
- **Invariant**: a CLEAN confirmation for a bsi whose PARTIAL was already applied
  is NOT dropped (the dormancy/no-credit at OFDM tier); a REPEATED clean for the
  same bsi IS dropped (no double-count of nBatches_fully_acked). Walk:
  first clean for B → `cmd_last_applied_clean_bsi != B` → accepted, set to B.
  second clean for B → `== B` → dropped. ✓

### 6.3 Why it will NOT re-thrash at deep SNR (the af14a9e/3b1726a guarantee)

- The **+1 anchor clamp** (`:3486`, `:4858`) is UNTOUCHED. Promotion past a rung
  still requires that rung to have produced a CLEAN delivery (anchor advance).
- The **clean-batch viability gate** (`promotion_allowed_on_batch`, §9) is
  UNTOUCHED — a partial SACK still does not promote.
- Bug-1's fix makes a genuinely-delivered (retransmit-completed) batch get
  CREDITED in-window; it does NOT relax the clean requirement. The completion ACK
  is all-ones = "every frame delivered", which is the definition of clean. A
  batch that never completes never emits the all-ones confirmation, so a
  non-viable rung still cannot promote.
- §2/§3 keep robust batch=1, which makes a single delivered MFSK frame == a
  clean all-ones batch. This is STRICTER than batch≥5 (one frame proves the rung),
  not looser — it cannot manufacture a false clean.
- The SNR correction (§5) only makes SNR-SUPERSHIFT more conservative.

Net: the fix supplies the missing INPUT the promotion gates were waiting for
(clean credit at each rung) without weakening any gate. At deep SNR where a rung
genuinely can't deliver, no clean ACK is produced, the anchor doesn't advance,
and the clamp holds — the over-climb protection is intact.

---

## §7 Regression tests (this doc's paired tests)

`--test-climb-engine` (`test_climb_engine`, `arq_commander.cc`). Parts A-D (the
original 3-bug climb fix) + Parts E-F (the §10/§11 deep-SNR down-hysteresis),
each fail-before / pass-after (Parts E/F detailed in §10.4 / §11.4):

- **(a) in-window clean credit (Bug 1)**: replay a PARTIAL SACK for bsi=B
  (sets `cmd_last_applied_sack_bsi=B`) then an all-ones CLEAN confirmation for
  bsi=B through the split-dedupe predicate. Assert the clean is ACCEPTED (not
  deduped) and a repeated clean for B is REJECTED. Pre-fix (single tracker): the
  clean for B is dropped as duplicate → FAIL.
- **(b) CMD==RSP batch at robust (Bug 2/3)**: drive the CMD batch-floor gate +
  `policy_evaluate_axis2` and the RSP batch-floor gate at ROBUST_0/1/2 and at
  CONFIG_10. Assert robust → batch stays 1 on BOTH sides after SACK-negotiation
  recompute AND after 6 consecutive good Axis-2 observations; CONFIG_10 → batch
  grows. Pre-fix: Axis-2 steps robust batch 1→6 → FAIL.
- **(c) MULTI-rung climb (Bug 3, THE key assertion the singles lacked)**: an
  end-to-end anchor-advance loop. Start anchor=ROBUST_0, current=ROBUST_0.
  Credit a clean batch (drives the real §1.1 producer) → assert anchor=ROBUST_0,
  FRAME-UP eligible. Promote to ROBUST_1 (real FRAME-UP path / proposed). Credit
  a clean batch at ROBUST_1 → assert anchor ADVANCES to ROBUST_1 (this is what
  dormancy blocked) → assert clamp now permits ROBUST_2. Repeat through ROBUST_2
  → CONFIG_0. Assert the climb reaches CONFIG_0 (≥3 rungs), NOT stuck at ROBUST_1.
  Pre-fix (with Axis-2 growing batch): anchor never advances past ROBUST_0 → FAIL.
- **(d) CONNECT-PATH CMD/RSP batch symmetry (the 4th wire failure; added
  2026-05-30, see `data-flow-batch-size.md` §6)**: set the two config members to
  the REAL unpinned-connect state that parts (a)–(c) never modeled —
  `current_configuration=ROBUST_0` (live PHY) but `negotiated_configuration=
  CONFIG_0` (its CTOR DEFAULT; the connect path never writes it). Drive the REAL
  production recompute `sack_negotiated_recompute_batch` (the single shared body
  both the CMD and RSP handlers now call) and assert CMD batch == RSP batch == 1
  (D1/D2/D3), the chokepoint clamps a direct robust over-request (D4), and OFDM
  at CONFIG_10 still scales to ≥5 (D5). **Fail-before** (86d39b4: CMD gate on
  `negotiated_configuration`=CONFIG_0 → recompute ran → batch=5≠1): D1/D3/D4
  FAIL — verified by reverting ONLY the helper predicate to
  `negotiated_configuration` + disabling the chokepoint (got=25 want=1).
  **Pass-after**: gate on `current_configuration`=ROBUST_0 → batch stays 1 →
  PASS. This is the assertion all four wire failures slipped past.

Local tests are NECESSARY but NOT SUFFICIENT — the singles passed local and
failed the wire. Part (d) closes the connect-path default-init gap specifically;
§8 flags the integration-path assumptions that still need hardware.

---

## §8 Open questions / integration-path assumptions needing the wire

- **[?]** Does FRAME-UP at ROBUST_1 actually receive its first clean ACK in-window
  on the IONOS, or does the post-SET_CONFIG rx-mute timing race
  (`arq_commander.cc:3290-3307`, the `frame_gearshift_just_applied` retry-once)
  bite? FRAME-UP does NOT set `frame_gearshift_just_applied` (only SUPERSHIFT at
  `:4464` does), so the retry-once protection does NOT cover FRAME-UP promotions.
  At robust ack==data PHY (no swap) so the race may not apply — but UNVERIFIED on
  hardware. If the first ROBUST_1 batch misses its ACK, emergency_nack handling
  may BREAK back. Watch `[GEARSHIFT] FRAME UP` followed by clean-ACK-miss at the
  new rung in the cmd log.
- **[?]** With batch pinned at 1 through ROBUST_2, does the climb reach CONFIG_0
  (the OFDM tier) and does the batch-floor recompute correctly take batch to ≥5
  at CONFIG_0? The TEST_CONNECTION recompute runs ONCE at connect (ROBUST_0); the
  CONFIG_0 batch comes from `load_configuration`'s OFDM scaling (`:1269`). Verify
  CMD/RSP agree on the CONFIG_0 batch after the ROBUST_2→CONFIG_0 promotion.
- **[?]** Bug-1's prev-path clean ACK adds an extra MFSK ACK TX on the OFDM-tier
  SACK-recovery path. Does it collide with the CMD's retransmit TX timing? The
  clean funnel's post-TX timeout math (`arq_responder.cc:1564`) is for the
  partial path; the prev-delivered emit reuses `send_mfsk_ack_sack` but the
  surrounding timeout state may differ. UNVERIFIED.
- **[?]** Does the corrected SNR (14.6) change which OFDM config the climb
  settles at once it reaches the tier (vs the old 38)? Expected yes (more
  conservative), but the parent's quick-cascade-check @clean will show it.

---

## §10 DEEP-SNR DOWN-HYSTERESIS — the missing anchor DEMOTION producer (climb follow-up #2)

**Status**: SHIPPED 2026-05-30 on `fix/climb-engine`, stacks on 57f938f. Fixes
the WGN:-10 `CONFIG_0 ↔ ROBUST_0` thrash that appeared AFTER the §1-§9 climb fix
made the clean-channel climb work. In-process verified (Part E); wire
confirmation is the parent's pause-cal hardware test @WGN:-10.

### §10.1 The confirmed root cause (do NOT re-investigate — this is the record)

The climb works at clean but thrashes at WGN:-10. Mechanism:

1. At CONFIG_0/-10, a batch completes ONLY after several SACK-retransmit rounds;
   the §4 prev-path then emits an all-ones CRC-valid completion ACK
   (`arq_responder.cc:~777-805`, `[RSP-V2-PREV-DELIVERED]` → `send_mfsk_ack_sack`).
2. CMD credits it clean → the anchor-raise producer (`arq_commander.cc:3503-3505`)
   sets `last_data_viable_config = CONFIG_0`, AND `breaks_since_last_data_success`
   resets to 0 (`arq_commander.cc:3496`).
3. CONFIG_0 data then fails → BREAK → `break_target_with_anchor()`
   (`arq_commander.cc:147-153`) clamps the recovery target UP to the anchor
   (CONFIG_0) unless `breaks_since_last_data_success >= 2` (panic bypass).
4. The panic escape NEVER latches: every slow retransmit-completion at CONFIG_0
   resets `breaks_since_last_data_success` to 0, so it oscillates 1→0→1→0, never
   reaching 2 → the anchor-floor is never bypassed → infinite CONFIG_0↔ROBUST_0
   thrash (15 BREAKs on hardware).
5. THE FLAW: there is NO demotion producer for `last_data_viable_config` — it only
   ever rises (anchor-raise + ctor/reset init). "Retransmit-completed ≠
   sustainably viable", but the anchor treated it as viable forever.

### §10.2 The fix (piece A) — anchor demotion

Add `anchor_consec_break_fails` (`arq.h`, ctor + reset_session_state init 0). It
increments at the BREAK trigger (`arq_commander.cc:~3390`, co-located with
`breaks_since_last_data_success++`) ONLY when
`current_configuration == last_data_viable_config` (the BREAK is firing AT the
anchor rung — i.e. the anchor rung itself is breaking). When it reaches
`K = ANCHOR_DEMOTE_BREAK_FAILS = 3` (matches the existing
`gear_shift_down_consecutive_fails < 3` house pattern at `:4793`/`:4954`), the
anchor is DEMOTED one rung via the PURE helper
`anchor_demote_target(anchor, fails, robust)` (`arq.h`,
`= config_ladder_down(anchor, robust)` once `fails >= K`), and the counter
resets. The reset producer is the clean-credit block (`arq_commander.cc:~3496`):
ANY clean confirmation clears `anchor_consec_break_fails` to 0, so the demotion
requires K *consecutive* anchor-rung BREAKs with no clean between.

Result: after K=3 anchor-rung BREAKs, `last_data_viable_config` drops CONFIG_0 →
ROBUST_2; `break_target_with_anchor` then floors only to ROBUST_2 (below CONFIG_0)
→ the next BREAK escapes toward ROBUST_0. If ROBUST_2 also can't carry data the
demotion walks the anchor down one rung per K failures until it reaches a viable
rung (ROBUST_0). This is the down-hysteresis. **K=3 and the per-tier sustained-N
(§11) are TUNABLE.**

`anchor_demote_target` ONLY ever LOWERS the anchor (or holds at the ladder floor)
— never raises — so the +1 up-clamp (§1) gets STRICTER, never looser → no
af14a9e/3b1726a over-climb regression (asserted by Part E5, a full-ladder sweep).

It is COMPLEMENTARY to the breaks>=2 panic-jump, not a replacement: if the panic
counter ever does reach 2 (two BREAKs with genuinely no clean between) it still
jumps straight to ROBUST_0 (Part E4). The demotion is the slower escape for when
slow completions keep resetting the panic counter — exactly the WGN:-10 case.

### §10.3 Producers / consumers of `anchor_consec_break_fails`

- **Producers**: `++` at the BREAK trigger (`arq_commander.cc:~3390`) gated on
  `current_configuration == last_data_viable_config`; reset to 0 at the demote
  site (after the decision) and on ANY clean confirmation in the credit block
  (`arq_commander.cc:~3496`); init 0 in ctor (`arq_common.cc:~352`) +
  reset_session_state (`arq_common.cc:~2987`).
- **Consumer**: the demote decision itself (`anchor_demote_target` at the BREAK
  trigger) — the SOLE consumer.
- **Valid states**: 0 (no anchor-rung BREAK streak — the steady state and the
  post-clean / post-demote state) through K-1 (demote pending). Never observed
  >= K (the demote+reset is atomic with reaching K).
- **Invariant the consumer assumes**: the count reflects CONSECUTIVE anchor-rung
  BREAKs with no intervening clean. Maintained: the clean-credit reset clears it,
  and the gate only counts BREAKs where current==anchor. A BREAK at a
  NON-anchor rung does not increment it (correct — only the anchor rung's own
  failure should demote the anchor).

### §10.4 Part E (`--test-climb-engine`) — fail-before / pass-after

Replays the REAL `:3390` demotion block (the SAME `anchor_demote_target` pure
helper production calls) + the REAL `break_target_with_anchor`:
- **E0**: the first K-1 anchor-rung BREAKs hold the anchor (sub-threshold).
- **E1/E1b**: the K-th consecutive anchor-rung BREAK DEMOTES CONFIG_0 → ROBUST_2
  (strictly below CONFIG_0 by ladder index).
- **E2/E2b**: `break_target_with_anchor(ROBUST_0)` now returns ROBUST_2 (sub-
  CONFIG_0) → the thrash escapes (breaks counter < 2, so this is the DEMOTION
  escape, not the panic bypass).
- **E3**: a clean confirmation resets the streak (demote needs K consecutive).
- **E4**: the panic-jump (breaks>=2) still bypasses the floor (complementary).
- **E5**: full-ladder sweep — `anchor_demote_target` NEVER raises the anchor.

**FAIL-BEFORE** (verified 2026-05-30 by temporarily reverting `anchor_demote_target`
to the pre-fix no-op `return anchor;`): E1/E1b/E2/E2b FAIL (anchor stuck at
CONFIG_0, break floor still CONFIG_0); E3/E4/E5 correctly stay PASS (they are
invariants independent of the demotion). **PASS-AFTER**: all PASS. The temp
revert was reverted; the shipped helper does the demotion.

---

## §11 SUSTAINED-ANCHOR GATE — close the leak at the source (piece B)

**Status**: SHIPPED 2026-05-30 with §10. Gates the anchor-RAISE so a single
retransmit-rescued batch cannot anchor a non-sustainable rung.

### §11.1 The leak

§1.1's anchor-raise (`arq_commander.cc:3503-3505`) credited ANY clean
confirmation, including the single §4 prev-path all-ones completion ACK a
SACK-retransmit-rescued OFDM batch emits. That single completion ACK is exactly
what raised the CONFIG_0 anchor in §10.1 step 2 and seeded the thrash.
"Retransmit-rescued ≠ sustainably viable."

### §11.2 The fix (piece B) — N consecutive cleans per rung

Add `clean_batches_at_current_config` + `clean_batches_config` (`arq.h`, ctor +
reset init 0 / CONFIG_NONE). In the clean-credit block (`arq_commander.cc:~3493`):
a clean at a config != `clean_batches_config` starts the streak fresh at 1 (new
rung); otherwise it extends. The anchor-RAISE now additionally requires
`clean_batches_at_current_config >= sustained_anchor_threshold(current_configuration)`
(PURE helper, `arq.h`): robust (batch=1) **N=1** (one clean MFSK frame is strong
proof — keep the off-ROBUST_0 climb FAST), OFDM **N=2** (a single SACK-rescued
batch can't anchor a non-sustainable rung). Reset to 0 on ANY failed block
(`arq_commander.cc:~3146` and `:~3276`, co-located with `consecutive_data_acks=0`;
the `:~3276` reset is upstream of the BREAK trigger so the demotion path also sees
a reset streak).

We do NOT neuter the prev-path completion ACK itself (`arq_responder.cc:~805`) —
it keeps the link alive on a genuinely-good-but-lossy channel (its purpose since
§4). "Retransmit-rescued ≠ viable" is encoded HERE in the CMD anchor gate, which
still lets a rung that REPEATEDLY delivers clean promote (the count crosses N on
the 2nd consecutive clean — Part F2). The per-rung `clean_batches_config` tracker
makes the streak self-reset on rung change without touching `load_configuration`
(which toggles `current_configuration` on every data/ack PHY swap — see §11.3).

### §11.3 Producers / consumers of `clean_batches_at_current_config` / `clean_batches_config`

- **Producers**: the clean-credit block (`arq_commander.cc:~3493`) — increments
  (or restarts at 1 on rung change); reset to 0 at the two failure sites
  (`:~3146`, `:~3276`); init 0 / CONFIG_NONE in ctor + reset_session_state.
- **Consumer**: the anchor-RAISE gate (`arq_commander.cc:~3505`) — SOLE consumer.
- **Valid states**: count 0 (no clean streak — fresh session / post-failure)
  through any positive run length; `clean_batches_config` = CONFIG_NONE (no streak
  yet) or the rung the streak belongs to.
- **Invariant the consumer assumes**: the count is the number of CONSECUTIVE
  cleans AT `clean_batches_config`, and `clean_batches_config == current_configuration`
  whenever the count > 0. Maintained: every increment first re-syncs
  `clean_batches_config` to `current_configuration` on a mismatch. **The data/ack
  PHY-swap subtlety**: `load_configuration(ack_configuration, ...)` per-batch
  (`arq_commander.cc:1023/1752`) toggles `current_configuration` to the ACK config
  and back, so `load_configuration`'s `current_configuration=` write (`arq_common.cc:1242`)
  is NOT a safe rung-change signal. We deliberately do NOT reset there; instead the
  credit block runs only when `data_ack_received==YES` with a data-batch result, at
  which point `current_configuration` is the DATA config (the swap has reverted).
  This is why the counter lives in the credit path, mirroring `consecutive_data_acks`.

### §11.4 Part F (`--test-climb-engine`) — fail-before / pass-after

Replays the REAL `:3493` clean-credit anchor-raise (the SAME
`sustained_anchor_threshold` helper + `clean_batches_*` update):
- **F0a/F0b**: thresholds are robust N=1 / OFDM N=2 (documents the TUNABLEs).
- **F1**: a SINGLE retransmit-rescued OFDM clean at CONFIG_10 does NOT raise the
  anchor (N=2 not met) — the exact thrash-seeding event is now inert.
- **F2**: a SECOND consecutive OFDM clean DOES raise it (repeatedly-clean promotes).
- **F3**: a SINGLE robust clean raises immediately (N=1 — climb stays fast).
- **F4**: a failed block between two OFDM cleans resets the streak (the post-
  failure clean leaves count=1 → anchor not raised; consecutiveness enforced).
- **F5**: a clean at a DIFFERENT rung restarts the streak at 1 (per-rung).

**FAIL-BEFORE** (verified 2026-05-30 by temporarily setting OFDM N=1, modelling
the pre-fix unconditional raise): F1/F4 FAIL (a single OFDM clean raises the
anchor; F0b also flips since the threshold changed). **PASS-AFTER**: all PASS.

### §11.5 Cross-layer audit (CLAUDE.md §5) — §10/§11 fix

Shared state changed: `last_data_viable_config` (NEW demotion producer) + the two
new hysteresis counters. `last_data_viable_config` + `breaks_since_last_data_success`
are CMD anchor state consumed by `break_target_with_anchor` (BOTH BREAK sites
`:180` / `:279`) and the +1-anchor up-clamp (FRAME-UP `:3501` / LADDER-UP `:4737`
inline + `:4894` policy_evaluate_axis1). Producers/consumers walked:

- **`last_data_viable_config`** — Producers: the §1.1 anchor-RAISE
  (`arq_commander.cc:~3505`, now ALSO gated on the §11 sustained-N), the NEW §10
  DEMOTE (`arq_commander.cc:~3420`, lowers one rung at K anchor-rung BREAKs), ctor
  + reset_session_state init. Consumers: `break_target_with_anchor` (both BREAK
  sites), the +1 up-clamp (FRAME-UP / both LADDER-UP twins). Unchanged consumers.
- **`anchor_consec_break_fails`** — see §10.3. **`clean_batches_*`** — see §11.3.

Three required invariants verified:

1. **The clean-SNR multi-rung climb is UNAFFECTED.** On clean: the anchor only
   ever RISES (no anchor-rung BREAKs fire, so `anchor_consec_break_fails` stays 0
   and never demotes); the sustained-N is met quickly (robust N=1 on the first
   clean MFSK frame; OFDM N=2 on the 2nd consecutive clean — a clean channel
   delivers consecutive cleans trivially). **Part C (the multi-rung clean climb,
   ROBUST_0→CONFIG_0) STILL PASSES** unchanged — confirmed in the PASS-after run
   (C1/C2/C3 all PASS alongside E/F). The robust rungs use N=1 so the climb off
   ROBUST_0 is exactly as fast as before; only the OFDM tier requires the 2nd
   clean, which a clean channel supplies immediately.
2. **The af14a9e/3b1726a over-climb protection is PRESERVED.** The demotion ONLY
   lowers the anchor (Part E5 full-ladder sweep), so the +1 up-clamp gets stricter,
   never looser. The §11 gate only makes the RAISE harder (more cleans required),
   never easier. Neither can let the climb leap past a rung that hasn't proven
   sustainably clean. The +1 clamp expressions themselves are UNTOUCHED.
3. **The panic-jump is not broken; the demotion is a COMPLEMENTARY escape.** The
   breaks>=2 bypass in `break_target_with_anchor` (`:149`) is UNTOUCHED (Part E4).
   The two escapes are independent: panic fires when breaks reaches 2 (fast path);
   demotion fires when K=3 consecutive anchor-rung BREAKs accumulate (the slow
   path for when slow completions keep resetting the panic counter). At the
   WGN:-10 cliff the panic path is defeated by the reset, so the demotion path
   carries the escape — exactly its design intent.

No consumer assumption is violated; the fix adds a missing INPUT (a way to LOWER
the anchor + a stricter RAISE) without weakening any existing gate.

---

## §12 ADAPTIVE FRAME-UP THRESHOLD — fast-probe when sustained-clean (Option 3, climb follow-up ③)

**Status**: SHIPPED 2026-05-30 on `fix/climb-engine`, stacks on b1ab550. Fixes
the "the climb is correct but SLOW at high SNR" problem: the unpinned
`-Q 0 -M auto -g -R` climb advances exactly +1 rung per `frame_shift_threshold=3`
CONSECUTIVE clean batches, UNIFORMLY at every SNR. At high SNR the ×3 multiplier ×
per-frame airtime dominates (~30–95 s/rung; ~300 s just to reach CONFIG_6). The
forward AARF half (probe-up-faster-when-winning) was deleted by af14a9e; only the
back-off half (`frame_shift_threshold *= 2` on FRAME-UP failure) survived. This
restores the forward half SAFELY. In-process verified (Part I); wire speedup is
the parent's hardware test (with the fixed cascade-bench timing).

**Fork ① (the +1-clamp relaxation) is now SHIPPED in §13 (2026-05-31).**
When §12 (③) shipped it was still the user's pending decision; §13 records
the approved fix. §12 itself remains UNRELATED to the clamp: the +1
anchor clamp (`arq_commander.cc:3617` FRAME-UP / `:4858` LADDER-UP), multi-rung
jumps, and the SUPERSHIFT re-trigger un-clamp are UNTOUCHED. Option 3 changes ONLY
how many clean batches trigger a +1 step — never how far a +1 reaches.

### §12.1 The fix — an EFFECTIVE (read-time) threshold, member untouched by the reduction

The reduction is a READ-TIME decision at the FRAME-UP comparison
(`arq_commander.cc:3637`), NOT a mutation of the `frame_shift_threshold` member.
Rationale (the AARF-compat constraint): the member is what the back-off half
DOUBLES (`:2302/:3180/:3359`, 3→6→12…) and what the `[GEARSHIFT]` logs print; if
the reduction mutated it, a `min(member, base)` cap would silently UNDO an
AARF-doubled value whenever delivery was merely non-clean (not the intent — AARF
doubling means "be EXTRA conservative here"). Instead, a PURE helper picks between
the FAST value and the (possibly AARF-doubled) member:

```cpp
// arq.h — PURE, no side effects (unit test replays it directly).
int effective_frame_shift_threshold(int base_threshold, int config,
                                     int clean_streak_at_config) const
{
  // FAST probe (== FRAME_SHIFT_FAST = 1) ONLY once the channel has PROVEN
  // sustained-clean delivery at THIS rung — the SAME viability bar §11 uses to
  // ANCHOR the rung (sustained_anchor_threshold). Below that bar the conservative,
  // possibly AARF-doubled `base_threshold` member stands UNCHANGED (does NOT fight
  // the back-off). config = current_configuration; clean_streak =
  // clean_batches_at_current_config.
  if(clean_streak_at_config >= fast_probe_clean_streak(config))
    return FRAME_SHIFT_FAST;            // 1 — step on the next clean batch
  return base_threshold;                // the member: base 3, or AARF-doubled 6/12/…
}

// fast_probe_clean_streak(config) = sustained_anchor_threshold(config):
//   robust  -> SUSTAINED_ANCHOR_N_ROBUST (1)  — one clean MFSK frame proves it
//   OFDM    -> SUSTAINED_ANCHOR_N_OFDM  (2)    — a single SACK-rescued batch can't
// Tying the fast-probe bar to the anchor bar guarantees fast-stepping can fire
// ONLY at a rung already proven viable enough to anchor — a marginal / cliff rung
// (where every failure resets clean_streak to 0) NEVER goes fast.
```

The FRAME-UP comparison (`arq_commander.cc:3637`) becomes:
```cpp
int eff_thresh = effective_frame_shift_threshold(frame_shift_threshold,
                    current_configuration, clean_batches_at_current_config);
if(consecutive_data_acks >= eff_thresh) { /* +1 step (unchanged body) */ }
```

**Constants** (`arq.h`, both TUNABLE): `FRAME_SHIFT_FAST = 1` (the fast-probe
target). `fast_probe_clean_streak` reuses `SUSTAINED_ANCHOR_N_ROBUST/OFDM`.

**Before / after** (clean channel, p≈1):
- Robust ROBUST_0: clean_streak hits N=1 on the 1st clean batch → eff_thresh=1 →
  steps after 1 clean (was 3). Off-ROBUST_0 climb ~3× faster.
- OFDM CONFIG_k: clean_streak hits N=2 on the 2nd clean batch → eff_thresh=1 →
  3rd clean batch steps (consecutive_data_acks=1 ≥ 1). Net ≈1 clean batch/rung in
  steady state once the rung is proven, vs 3.
- Marginal / cliff: a failure resets clean_streak to 0 (`:3172/:3307`); eff_thresh
  reverts to the (AARF-doubled) member → conservative, exactly as today.

### §12.2 Producers / consumers — what §12 changes

`frame_shift_threshold` (the member) — UNCHANGED producer/consumer set:
- **Producers**: base init `arq_common.cc:337` (=3) + reset_session_state init;
  the AARF back-off `*= 2` at `arq_commander.cc:2302` (turbo SET_CONFIG NAck),
  `:3180` (FRAME-UP DATA FAILED, no-ACK path), `:3359` (FRAME-UP DATA FAILED, pat
  path); test reset `:6364`. §12 adds NO producer of the member — the reduction is
  read-time only, so the member retains its AARF state verbatim.
- **Consumer (the ONLY one)**: the FRAME-UP comparison `arq_commander.cc:3637`,
  now reading it THROUGH `effective_frame_shift_threshold(...)`. No other site
  reads the member.

`clean_batches_at_current_config` (the gating signal) — §12 adds ONE consumer:
- **Producers** (unchanged, owned by §11): increment in the clean-credit block
  (`arq_commander.cc:3548`), restart at 1 on rung change (`:3544`), reset to 0 at
  the two failure sites (`:3172`, `:3307`), init 0 / CONFIG_NONE in ctor +
  reset_session_state.
- **Consumers**: the §11 anchor-RAISE gate (`:3567`) AND NOW the §12 fast-probe
  read (`:3637`, via the helper). Both are READS — §12 does not write the counter.
  Ordering: the credit block (`:3510-3575`) runs BEFORE the FRAME-UP block
  (`:3604-3667`) in the same `process_main()` pass, so the FRAME-UP read sees the
  current-pass-updated streak.

### §12.3 Cross-layer audit (CLAUDE.md §5) — the four required verifications

**(1) Deep-SNR / marginal delivery stays conservative → #2 (e3d818d) anti-thrash
INTACT (no WGN:-10 CONFIG_0↔ROBUST_0 thrash re-introduced).**
At the WGN:-10 cliff, batches FAIL → `clean_batches_at_current_config` is reset to
0 on EVERY failed block (`:3172`, `:3307`, both upstream of the BREAK trigger). A
streak of 0 < `fast_probe_clean_streak(config)` ⇒ `effective_frame_shift_threshold`
returns the conservative member (3, or AARF-doubled). So FRAME-UP CANNOT step fast
at the cliff — it requires the SAME `consecutive_data_acks >= 3` (or more) it does
today. Moreover §12 touches ONLY the step CADENCE; the +1 anchor clamp (`:3617`),
the `promotion_allowed_on_batch` clean gate, the §10 anchor DEMOTION, and the §11
sustained-anchor RAISE gate are ALL untouched. Even in the impossible event the
streak briefly armed fast-probe, a +1 step still reaches only anchor+1, and the
anchor is independently governed by §10/§11. The thrash mechanism (slow
retransmit-completion resetting the panic counter) is unrelated to FRAME-UP
cadence and is cured by §10/§11, which §12 does not modify. **No re-introduction.**

**(2) The AARF back-off on failure still RAISES the member (the reduction does not
fight the back-off).**
The reduction is read-time and never writes the member, so `*= 2` at
`:2302/:3180/:3359` is fully preserved (3→6→12…). Critically, the back-off and the
reduction are MUTUALLY EXCLUSIVE per `process_main()` pass: the `*= 2` sites are on
the `data_ack_received==NO` failure paths; the credit-block streak update + the
FRAME-UP read are on the `data_ack_received==YES` + clean path. They never run in
the same cycle. After a failure: AARF doubles the member AND
`clean_batches_at_current_config` is reset to 0 in the SAME failure handler ⇒ the
next FRAME-UP read sees streak=0 ⇒ returns the freshly-DOUBLED member (not FAST).
The fast-probe can only re-arm after a NEW sustained-clean streak rebuilds at the
rung — i.e. the channel actually recovered. This IS the missing forward-AARF
recovery half, and it is strictly gated behind proven re-cleanliness. ✓

**(3) Interaction with #2's anchor demotion + sustained-raise gate is benign.**
Both #2 mechanisms key off the SAME `clean_batches_at_current_config` /
`last_data_viable_config` / `anchor_consec_break_fails` state that §12 only READS
(§12 writes none of it). The fast-probe bar is set EQUAL to the anchor-raise bar
(`fast_probe_clean_streak == sustained_anchor_threshold`), so fast-stepping arms at
exactly the streak length that also (re-)arms the anchor at that rung — the two are
consistent by construction. After a FRAME-UP promotion `current_configuration`
changes; the credit block then sets streak=1 at the NEW rung (`:3544`), which is
< OFDM N=2 ⇒ fast-probe does NOT carry over a promotion — each rung must re-prove.
Anchor demotion only LOWERS the anchor (§10, Part E5), tightening the +1 clamp;
§12 cannot widen it. ✓

**(4) The consecutive-clean counter is not corrupted by retuning the threshold
mid-run.**
`consecutive_data_acks` and `clean_batches_at_current_config` are independent
members; §12 reads both and writes NEITHER from the helper. The FRAME-UP body's
existing `consecutive_data_acks = 0` on promotion (`:3643`) and the failure-site
resets (`:3166/:3302`) are unchanged. Changing the comparison RHS from a constant
to a read-time value does not touch the LHS counter or its lifecycle. A lower
eff_thresh simply makes the existing `>=` fire sooner; it cannot make
`consecutive_data_acks` skip, double-count, or go negative. ✓

**Net**: §12 supplies the missing forward-AARF input (fast-step when the rung is
PROVEN sustained-clean) by reusing the §11 clean-streak signal as a read-time gate,
with the fast-probe bar pinned to the anchor-viability bar. It weakens no gate,
mutates no shared state, and is provably inert at the deep-SNR cliff where #2's
anti-thrash lives.

### §12.4 Part I (`--test-climb-engine`) — fail-before / pass-after

Replays the REAL FRAME-UP cadence (the SAME `effective_frame_shift_threshold` +
`fast_probe_clean_streak` helpers + the real `consecutive_data_acks >= eff_thresh`
comparison) under sustained-clean vs marginal/post-failure delivery:
- **I0a/I0b**: documents the TUNABLEs — `FRAME_SHIFT_FAST == 1`;
  `fast_probe_clean_streak` == robust 1 / OFDM 2.
- **I1 (THE fast assertion)**: at a robust rung with a SUSTAINED-CLEAN streak,
  `effective_frame_shift_threshold(base=3, …)` returns 1 → FRAME-UP fires after
  ONE clean batch. **FAIL-BEFORE** (member fixed at 3, no helper): needs 3.
- **I2 (multi-rung speedup)**: end-to-end sustained-clean climb — drive the real
  FRAME-UP loop with the adaptive threshold and assert it reaches a high config in
  FAR fewer batches than 3/rung. Compared against the fixed-3 count (computed in
  the same test) to prove a strict reduction.
- **I3 (marginal stays conservative)**: with clean_streak BELOW the bar (the
  cliff / post-failure state), `effective_frame_shift_threshold` returns the base
  3 (NOT 1) — fast-probe does NOT arm. Anti-thrash preservation, asserted directly.
- **I4 (AARF-doubling compat)**: with an AARF-DOUBLED member (=12) AND
  clean_streak below the bar, the helper returns 12 (does NOT cap to base / 1) —
  the back-off is preserved. With a high streak it returns 1 (recovery). Proves the
  reduction never fights nor erases the doubling.
- **I5 (no carry-over across a promotion)**: immediately after a +1 step the new
  rung's streak is 1; at an OFDM rung (N=2) the helper returns the conservative
  base, so the rung must re-prove before fast-stepping again.

**FAIL-BEFORE** (on b1ab550: FRAME-UP reads the bare member, no helper): I1/I2
FAIL (1 clean batch does not step; the climb needs 3/rung). I0/I3/I4/I5 — the
helper does not exist pre-fix; the test ships WITH the helper so all parts compile,
and I verified fail-before by asserting the pre-fix arithmetic (`needs 3, not 1`)
directly. **PASS-AFTER**: all PASS. Parts A–H stay PASS (§12 is additive — the
FRAME-UP body, the clamp, and every #2 mechanism are unchanged).

## §13 CONTROLLED ELEVATOR — SNR-gated multi-rung jump (fork ① — SHIPPED)

**Status**: SHIPPED 2026-05-31 on `fix/climb-engine`, stacks on d12c042. This is
the user-APPROVED fast-climb fix and the ONE change that partially relaxes
af14a9e's anchor+1 clamp. It is precise, SNR-gated, and provably INERT at deep
SNR. In-process verified (Part J, fail-before/pass-after); the over-climb safety
and the fast-jump speedup are the parent's HARDWARE verification (WGN:-10
no-thrash + WGN:30 fast-jump). Resolves the "Out of scope (architectural fork ①)"
note in §12 (it is no longer pending — it ships here).

### §13.1 The problem (from investigation a56a652 — not re-investigated)

The unpinned `-Q 0 -M auto -g -R` climb walks EVERY rung (≤ anchor+1) because
af14a9e replaced the SUPERSHIFT elevator with a strict data-anchored +1 ladder.
③ (d12c042, §12) sped the per-rung cadence (fast-probe → threshold 1 when
sustained-clean), but the climb is still wall-clock-bound by per-rung big-batch
airtime — only SKIPPING rungs gives the dramatic speedup. The SUPERSHIFT
re-trigger (`arq_commander.cc:4578-4626`) ALREADY computes the SNR-ideal config
(`get_configuration(measurements.SNR_uplink - SUPERSHIFT_MARGIN_DB)`) but was
CLAMPED to anchor+1 (`config_ladder_up_n(last_data_viable_config, 1, …)`), making
it a no-op. OptA (446887c/b1ab550, §G/§H + data-flow-snr-measurements.md) now
populates `measurements.SNR_uplink` during the climb (on the turbo SET_CONFIG
ACK suffix), so the re-trigger finally has a REAL SNR to act on — the precondition
that makes this relaxation safe.

### §13.2 The fix — relax the clamp at the re-trigger ONLY, under a high-SNR predicate

A new PURE helper `supershift_retrigger_target()` (`arq.h`, after
`effective_frame_shift_threshold`) owns the clamp-vs-jump decision; production
calls it at the re-trigger (`arq_commander.cc:4611`) in place of the old inline
`if(!optimizer_is_in_control()){…}` clamp. The helper receives the
ALREADY-ceiling-capped `snr_ideal` (the caller applies the NB cap at `:4588-4589`
and the `supershift_proven_ceiling` cap at `:4591-4592` BEFORE the call, verbatim
as pre-①), and:

```cpp
int supershift_retrigger_target(int snr_ideal, double snr_uplink, int anchor,
                                bool optimizer_owns, bool robust_en,
                                bool narrowband) const
{
  if(optimizer_owns) return snr_ideal;          // Q-table owns the band — no clamp
  int anchor_cap = config_ladder_up_n(anchor, 1, robust_en, narrowband); // anchor+1
  bool high_confidence_jump = (snr_uplink > -90) &&
      config_ladder_index(snr_ideal) > config_ladder_index(anchor_cap);
  if(!high_confidence_jump &&                    // KEEP the conservative +1 clamp
     config_ladder_index(snr_ideal) > config_ladder_index(anchor_cap)) //  unless a
    snr_ideal = anchor_cap;                      //  high-SNR multi-rung jump exists
  return snr_ideal;
}
```

- **HIGH-CONFIDENCE-SNR predicate**: `measurements.SNR_uplink` is VALID (> -90 —
  actually populated by OptA, not the -99.9 ctor sentinel) AND high enough that
  the ceiling-capped `snr_ideal` lands MORE than +1 rung past the anchor (i.e.
  there IS a multi-rung jump to make). The enclosing re-trigger gate at `:4585`
  (`measurements.SNR_uplink > -90`) already guarantees the validity half; the
  helper re-states it so it is correct in isolation (and so Part J can drive the
  sentinel directly).
- **Before / after** (SNR=20 dB, anchor=CONFIG_4 ⇒ anchor_cap=CONFIG_5):
  `get_configuration(20-6=14)` → CONFIG_16. Pre-① clamp → CONFIG_5 (anchor+1,
  gap=… < SUPERSHIFT_RETRIGGER_CONFIGS=3 ⇒ re-trigger usually didn't even fire) →
  +1 crawl. Post-① → CONFIG_16 (the SNR-ideal), capped only by proven-ceiling /
  WB-NB ceiling ⇒ a multi-rung elevator jump in one SET_CONFIG.
- **Low/invalid SNR** (`snr_ideal ≤ anchor+1`): the predicate is FALSE; the inner
  `if(idx(snr_ideal) > idx(anchor_cap))` is ALSO false ⇒ no write ⇒ return
  `snr_ideal` unchanged — and since `snr_ideal ≤ anchor+1` already, this is
  BYTE-IDENTICAL to the pre-① clamp (which was likewise a no-op there).

### §13.3 §5 cross-layer audit — the over-climb-critical anchor clamp + `last_data_viable_config`

The anchor+1 clamp and `last_data_viable_config` are over-climb-critical shared
CMD state (the af14a9e/3b1726a protection). Enumerate the consumers of the clamp
and verify the five mandated invariants.

**Consumers of the "no upward move > anchor+1" clamp** (file:line, all read
`last_data_viable_config`):

1. **FRAME-UP** (`arq_commander.cc:3617`):
   `config_ladder_index(proposed_frame) > config_ladder_index(last_data_viable_config) + 1`
   ⇒ `frame_ceiling_blocked = true`. `proposed_frame = config_ladder_up(current, …)`
   — inherently +1.
2. **LADDER-UP v1 inline twin** (`arq_commander.cc:4869`): same predicate on
   `config_ladder_up(current, …)` — inherently +1.
3. **LADDER-UP `policy_evaluate_axis1`** (`arq_commander.cc:5026`): same predicate
   on `config_ladder_up(current, …)` — inherently +1.
4. **SUPERSHIFT re-trigger** (`arq_commander.cc:4611`, NOW via
   `supershift_retrigger_target`): the ONLY consumer that computes a MULTI-rung
   `snr_ideal` and could jump >+1 — and the ONLY one ① touches.
5. **`break_target_with_anchor`** (`arq_commander.cc:147-154`): floors a raw BREAK
   recovery target UP to `last_data_viable_config` (bypassed under
   `breaks_since_last_data_success >= 2` panic). The down-recovery backstop.

**(1) ONLY the re-trigger path is un-clamped; FRAME-UP / LADDER-UP byte-unchanged.**
Consumers 1/2/3 build their target with `config_ladder_up(current_configuration,
…)`, which is structurally +1 (`common_defines.h:116-128`) — they CANNOT express a
multi-rung jump regardless of the clamp, and ① does not edit their predicate text
at all. Verified by diff: `arq_commander.cc:3617`, `:4869`, `:5026` are
byte-identical to d12c042. Part J7a/J7b/J7c replay all three predicates at SNR=20
and assert (a) FRAME-UP still advances exactly +1, (b) `config_ladder_up` can
never reach the SNR-ideal CONFIG_16, (c) a stale-anchor 2-rung proposal is still
BLOCKED. ① is exclusive to consumer 4.

**(2) The jump never raises the anchor speculatively — the anchor follows
CONFIRMED delivery only.** `supershift_retrigger_target` READS `anchor`
(`last_data_viable_config`) to compute `anchor_cap` but NEVER writes it (it is a
`const` method returning an `int`; the caller assigns the result to `snr_ideal`,
not to `last_data_viable_config`). The sole anchor-RAISE producer remains §1.1
(`arq_commander.cc:3437-3439`, gated by `promotion_allowed_on_batch` + the §11
sustained-N) — a CONFIRMED clean batch at the landing config. The re-trigger
queues a SET_CONFIG to `snr_ideal` and sets `turboshift_phase = TURBO_FORWARD`
(`:4620`); the landing is SPECULATIVE. Part J6 snapshots `last_data_viable_config`
before the high-SNR decision and asserts it is unchanged (still CONFIG_4).

**(3) A failed/overshot jump → BREAK → recovery to the (still-low) anchor; §10
demotion backstops repeated overshoot.** Because (2) holds, after a speculative
jump to CONFIG_16 the anchor is still (say) CONFIG_4. If CONFIG_16 data fails,
BREAK fires; `break_target_with_anchor` floors recovery UP to CONFIG_4 (the
still-low anchor) — NOT up to CONFIG_16 — so the link drops back to proven ground.
If the overshoot REPEATS (the anchor rung itself keeps breaking), §10's
`anchor_consec_break_fails` reaches K=3 and `anchor_demote_target` LOWERS the
anchor one rung (Part E), and the breaks≥2 panic-jump (`:149`) is the independent
fast escape. ① adds NO new way for the anchor to rise, so it cannot defeat either
backstop. (Note: `supershift_proven_ceiling` is also lowered by BREAK at the
failed config — `:3486`/`:2268`/etc. — so a repeatedly-overshooting jump tightens
the proven-ceiling cap applied at `:4591-4592` on the NEXT re-trigger, a third
self-limiting effect. Part J4 asserts the proven-ceiling cap binds.)

**(4) ① is INERT at deep/invalid SNR (af14a9e behavior byte-identical when
`SNR_uplink` unpopulated).** Two independent guarantees:
  - *Invalid (sentinel −99.9)*: the enclosing re-trigger gate at
    `arq_commander.cc:4585` requires `measurements.SNR_uplink > -90`. At the ctor
    sentinel this is FALSE ⇒ the WHOLE re-trigger block (helper call included)
    never executes ⇒ byte-identical to d12c042. Part J2a asserts the gate is
    false at −99.9; J2b asserts the helper (if reached anyway) returns the
    pre-① result.
  - *Low-but-valid*: when `snr_ideal ≤ anchor+1`, `high_confidence_jump` is FALSE
    and the inner clamp `if(idx(snr_ideal) > idx(anchor_cap))` is also FALSE ⇒ no
    write ⇒ identical to the pre-① no-op. Part J3a/J3b assert the ① target equals
    the pre-① target and stays ≤ anchor+1 (no spurious jump). The Part J
    `adaptive=false` arm is the verbatim pre-① hard clamp (confirmed
    byte-identical via `git show d12c042` — the diff is in §13.4); J1d asserts it
    yields anchor+1 (no jump), J1e asserts ① differs ONLY when the predicate
    licenses the jump.

**(5) No SUPERSHIFT storm — the TURBO_DONE phase + `turbo_supershift_announce_pending`
guard still bound re-entry.** ① does not touch the re-entry guards. The re-trigger
gate still requires `turboshift_phase == TURBO_DONE` (`:4584`); the moment the
re-trigger FIRES it sets `turboshift_phase = TURBO_FORWARD` (`:4620`) and the
in-turbo jump sets `turbo_supershift_announce_pending` — so a SECOND re-trigger is
impossible until turbo finishes and the announce clears (Part G3/G3b, unchanged
and still PASS). ① only changes WHERE a single admitted re-trigger lands
(multi-rung vs +1), never HOW OFTEN it can re-enter. A higher landing means FEWER
total re-triggers to reach the SNR-ideal config, not more.

Net: ① supplies the missing OUTPUT (a multi-rung target the gates were already
computing but discarding) at the ONE site that can use it, gated on a high-SNR
predicate that is provably inert at the deep-SNR cliff, and adds no anchor-raise
and no re-entry. Every af14a9e/3b1726a over-climb protection (the +1 clamp on the
other three consumers, the confirmed-delivery anchor raise, the BREAK floor, the
§10 demotion, the §11 sustained-N, the proven-ceiling cap, the storm guards) is
intact.

### §13.4 Producers / consumers — what ① changes

- **`last_data_viable_config`** (the anchor): ① adds NO producer and NO new
  consumer. The re-trigger already read it (to clamp); it still reads it (inside
  the helper, to compute `anchor_cap`). Producers (unchanged): §1.1 anchor-RAISE
  (`:3437`), §10 DEMOTE (`:3454` region), ctor + reset_session_state init.
  Consumers (unchanged set): the four +1-clamp sites + `break_target_with_anchor`.
- **`snr_ideal`** (the re-trigger's local target): its post-clamp VALUE changes at
  clearly-high SNR (now the SNR-ideal instead of anchor+1). Consumed only by the
  `gap >= SUPERSHIFT_RETRIGGER_CONFIGS` test (`:4614`) and, if the re-trigger
  fires, by `negotiated_configuration = snr_ideal` (`:4628`) → the SET_CONFIG
  target. A higher `snr_ideal` makes the `gap` test PASS more readily (the
  re-trigger was previously self-suppressed because the clamped gap fell below 3)
  — this is the intended unblocking.
- **The pre-① inline clamp** (d12c042 `arq_commander.cc`, byte-identical to the
  Part J `adaptive=false` arm):
  ```cpp
  if(!optimizer_is_in_control()) {
    int anchor_cap = config_ladder_up_n(last_data_viable_config, 1, robust_enabled, narrowband_enabled == YES);
    if(config_ladder_index(snr_ideal) > config_ladder_index(anchor_cap))
      snr_ideal = anchor_cap;
  }
  ```
  is REPLACED by the single `snr_ideal = supershift_retrigger_target(…)` call. The
  `optimizer_is_in_control()` exemption is preserved (moved into the helper's
  `optimizer_owns` early-return) — when the Q-table owns the band, NO anchor clamp
  is applied either way (unchanged).

### §13.5 Part J (`--test-climb-engine`) — fail-before / pass-after

Part J replays the caller-side ceiling caps EXACTLY as production (`:4587-4592`),
then drives BOTH the REAL shipped helper (`adaptive=true` → PASS-AFTER) and the
verbatim pre-① hard clamp (`adaptive=false` → the d12c042 outcome, side-by-side):

- **J0**: `SUPERSHIFT_MARGIN_DB == 6.0` (a silent margin change shifts every
  assertion below — documents the TUNABLE).
- **J1 (THE multi-rung-jump assertion)**: SNR=20 dB, anchor=CONFIG_4. J1a sanity:
  `snr_ideal` (CONFIG_16) is multi-rung above anchor+1 (CONFIG_5). **J1b** (real
  helper): target == CONFIG_16. **J1c**: target STRICTLY above anchor+1. **J1d**:
  the pre-① clamp pins to CONFIG_5 (no jump). **J1e**: ① differs from pre-① (the
  fix bites).
- **J2 (DEEP-SNR INERT, sentinel)**: J2a the −99.9 sentinel fails the >-90 gate
  (block never runs); J2b the helper (if reached) returns the pre-① result.
- **J3 (DEEP-SNR INERT, low-but-valid)**: SNR=-3 → CONFIG_0 (≤ anchor+1); ① ==
  pre-① and target stays ≤ anchor+1 (no spurious jump).
- **J4 (proven-ceiling cap, SAFETY #2)**: SNR=20 but
  `supershift_proven_ceiling=CONFIG_10` → the jump is CAPPED at CONFIG_10 (never
  above proven-safe), still multi-rung.
- **J5 (WB/NB ceiling cap, SAFETY #2)**: NB + SNR=20 → CAPPED at
  NB_CONFIG_MAX (CONFIG_14).
- **J6 (anchor NOT raised, SAFETY #3)**: the high-SNR decision leaves
  `last_data_viable_config` unchanged (CONFIG_4).
- **J7 (FRAME-UP / LADDER-UP untouched, SAFETY #1)**: replay the REAL `:3617`
  FRAME-UP + `:4869`/`:5026` LADDER-UP predicates at SNR=20: J7a FRAME-UP still +1
  (CONFIG_4→CONFIG_5); J7b `config_ladder_up` can never reach CONFIG_16; J7c a
  2-rung proposal off a stale anchor is still BLOCKED.

**FAIL-BEFORE** (verified 2026-05-31 by temporarily reverting
`supershift_retrigger_target`'s body to the pre-① hard clamp, rebuilding):
**J1b/J1c/J1e + J4a + J5 FAIL** (got=CONFIG_5/anchor+1 instead of the jump);
J0/J1a/J1d/J2/J3/J6/J7 and Parts A–I correctly STAY PASS (they assert the
inert/off-path/no-raise properties, which are invariant). This proves the test
isolates exactly the ① behavior. **PASS-AFTER** (shipped helper restored): all
Parts A–J PASS (0 failures); `--test` 30/30. The temp revert was reverted; the
shipped helper does the SNR-gated jump.

**HONEST scope** (§8 applies): the in-process Part J proves the jump FIRES at high
SNR and is INERT at deep/invalid SNR, and that the over-climb guards are
structurally untouched. The over-climb SAFETY (WGN:-10 no-thrash) and the
fast-jump SPEEDUP (WGN:30 multi-rung in one shot) are the parent's HARDWARE
verification — the integration-path timing of a multi-rung SET_CONFIG landing
(post-jump rx-mute, the new config's first-batch ACK) is not exercised in-process.

---

## §14 REAL FAST-PROBE — (A1) CMD arm-asymmetry repair + (B) FRAME-UP-anchored elevator

**Status**: SHIPPED 2026-05-31 on `fix/fast-probe` (NEW worktree branched off
`fix/climb-engine` @ 540779c, the §13 fork-① HEAD). This is the "real fast-probe":
the §13 elevator only fires from the SUPERSHIFT re-trigger, which on the unpinned
`-Q 0 -M auto -g -R` ladder is dormant because (i) the CMD never decodes the
forward SNR so the re-trigger's `SNR_uplink > -90` gate never fires (the §14 (A1)
deadlock), and (ii) even with SNR populated the re-trigger's `gap >=
SUPERSHIFT_RETRIGGER_CONFIGS` self-suppression + the TURBO_DONE phase keep it
quiet on the steady-state +1 ladder. §14 supplies the missing INPUT (A1: arm the
CMD's SNR decode on gearshift SET_CONFIG ACKs) and the missing TRIGGER (B: fire
the §13 elevator from the data-anchored FRAME-UP path, which runs every clean
batch). In-process verified (Part J', fail-before/pass-after); the wire proofs
(WGN:-10 no-thrash; WGN:30 multi-rung jump; SNR_uplink leaving −99.9) are the
parent's HARDWARE verification.

### §14.1 (A1) The CMD arm asymmetry — ROOT CAUSE (diagnosed, not re-investigated)

The forward-link SNR is ALREADY on the wire. The RSP suffixes its measured SNR
onto SET_CONFIG ACKs whenever its SEND gate (`arq_responder.cc:1122-1124`) holds:
```
(turboshift_active || turboshift_phase != TURBO_DONE)
  && data[0] == SET_CONFIG && SNR_uplink > -90
```
On the unpinned data-anchored ladder the RSP's `turboshift_phase` stays at its
init `TURBO_FORWARD` (0 SWITCH_ROLE swaps occur), so the RSP KEEPS suffixing the
SNR on every gearshift SET_CONFIG ACK (14× in the logs, incl. a real 14.6 dB at
OFDM CONFIG_0/1/2/3).

The CMD never decodes it. The CMD's arm at `arq_commander.cc:1085` was
`turbo_snr_ack_enabled = turbo_snr_ack_expected_on_control(turboshift_active,
turboshift_phase, data[0])` (`arq.h:799-802`):
```
(turbo_active || phase != TURBO_DONE) && control_code == SET_CONFIG
```
The CMD's own `turboshift_phase` is `TURBO_DONE` in steady state and
`turboshift_active` is false, so BOTH disjuncts fail → the predicate returns
FALSE → `turbo_snr_ack_enabled` stays false → `receive_ack_pattern()` takes the
bare-ACK else branch (`arq_common.cc:5640`) instead of the SNR branch (`:5516`) →
the producer `arq_common.cc:5555` (`measurements.SNR_uplink =
snr_uplink_from_suffix(...)`) NEVER runs → `SNR_uplink` stays at the −99.9 ctor
sentinel. The send-gate and arm-gate DIVERGE on `turboshift_phase`. AND the §13
re-trigger (`arq_commander.cc:4584-4585`) is gated on `SNR_uplink > -90` (plus
TURBO_DONE, is_ofdm, gear_shift_on) — only the SNR term is unmet, so §13's helper
is never reached.

**The fix (no wire change — the RSP already sends):** widen the CMD arm with an
OR clause for the gearshift-ladder SET_CONFIG ACKs the RSP already suffixes. New
PURE helper `turbo_snr_ack_armed_for_gearshift(turbo_active, phase, control_code,
gear_shift_enabled, config_up)` (`arq.h:860`, immediately after the existing
helper):
```cpp
if(turbo_snr_ack_expected_on_control(turbo_active, phase, control_code))
  return true;                                   // the existing turbo arm
return gear_shift_enabled && control_code == SET_CONFIG && config_up;
```
The call site `arq_commander.cc:1085` (inside the `if(data[0]==SET_CONFIG)` block)
computes `config_up = config_ladder_index(negotiated_configuration) >
config_ladder_index(current_configuration)` and passes `gear_shift_on == YES`.

### §14.2 (B) The FRAME-UP-anchored controlled elevator

The §13 elevator (`supershift_retrigger_target` + the cap-chain) was extracted
VERBATIM into ONE shared non-const member `elevator_target_from_snr()`
(`arq_commander.cc:156-187`, declared `arq.h:759`), called by BOTH the SUPERSHIFT
re-trigger (`arq_commander.cc:4587`, formerly the inline cap-chain) AND the new
FRAME-UP site — eliminating the comment/code drift the DSP-commit anti-pattern
warns about. The body is byte-equivalent to the pre-extract re-trigger:
`get_configuration(SNR − SUPERSHIFT_MARGIN_DB)` → NB cap → `supershift_proven_ceiling`
cap → `supershift_retrigger_target(...)`.

At the FRAME-UP clean-batch-CONFIRMED branch (`arq_commander.cc:3702-3737`, inside
`data_ack_received==YES && promotion_allowed_on_batch(last_batch_fully_acked) &&
… && !frame_ceiling_blocked && !optimizer_owns_upward_frame`), the unconditional
`negotiated_configuration = proposed_frame;` (the +1) is replaced by an
ELEVATOR-OR-+1 decision:
```cpp
negotiated_configuration = proposed_frame;            // the +1 default
if(gear_shift_on==YES && is_ofdm_config(current_configuration) &&
   measurements.SNR_uplink > -90)
{
  int snr_ideal = elevator_target_from_snr();
  if(config_ladder_index(snr_ideal) > config_ladder_index(proposed_frame))
    negotiated_configuration = snr_ideal;             // multi-rung jump
}
```
The rest of the FRAME-UP block (FIFO restore, `add_message_control(SET_CONFIG)`,
the `TRANSMITTING_CONTROL` transition) is UNCHANGED — the elevator just sets a
higher `negotiated_configuration` before the SAME SET_CONFIG goes out.

### §14.3 §5 cross-layer audit (CLAUDE.md §5) — the shared state §14 touches

§14 touches three shared-state structures: `turbo_snr_ack_enabled` (the arm,
widened by A1), `measurements.SNR_uplink` (now populated per-rung as a consequence
of A1), and `last_data_viable_config` + the SACK suffix (the elevator reads/leaves
untouched). Enumerated per the five mandated questions:

#### §14.3.1 `turbo_snr_ack_enabled` (A1 widens the arm) — the NO-DATA-ACK-LEAK proof

- **Producers**: (1) the SET_CONFIG control-TX→wait arm `arq_commander.cc:1085`
  (NOW via `turbo_snr_ack_armed_for_gearshift`, widened); (2) the §13 re-trigger
  true-write `arq_commander.cc:4618`; (3) the turbo-finish false-writes
  `arq_commander.cc:2254` (NAck teardown) and `finish_turbo_direction()`
  `arq_commander.cc:3693` (unconditional clear before any TRANSMITTING_DATA); (4)
  ctor + reset_session_state init false.
- **Consumer (the ONLY one)**: `receive_ack_pattern()` (`arq_common.cc:5516`) — the
  `if(turbo_snr_ack_enabled)` branch routes the ACK suffix to the SNR decoder
  (`detect_ack_snr_from_passband` → the `:5555` producer); the else branch
  (`:5640`) treats it as a bare/SACK ACK.
- **Valid states**: false (steady state on a bare-ACK / data-ACK wait — the
  default, and the post-`finish_turbo_direction` state before data flows) ; true
  (a SET_CONFIG control-ACK wait during turbo OR — NEW — during an upward
  gearshift). Default-init: false (ctor/reset) — so before any producer writes, a
  spurious early ACK decode is impossible.
- **Invariant the consumer assumes**: the flag is true ONLY while awaiting a
  SET_CONFIG **control** ACK (never a DATA ACK), so the SNR decoder is never fed a
  SACK-suffixed data ACK. **§5 CRUX — preserved THREE structurally-independent
  ways:**
  1. **`control_code == SET_CONFIG` in BOTH disjuncts.** A DATA ACK is decoded by
     the SEPARATE function `process_messages_rx_acks_data()`; control ACKs are
     awaited in `process_messages_rx_acks_control()`. A data ACK's frame type
     (ACK_RANGE / ACK_MULTI / the SACK-v2 suffix) is NOT `SET_CONFIG`, so neither
     disjunct of `turbo_snr_ack_armed_for_gearshift` can be true for it. The
     widening adds `gear_shift_enabled && control_code == SET_CONFIG && config_up`
     — the `control_code == SET_CONFIG` conjunct is INSIDE the new disjunct, so the
     widening introduces NO non-SET_CONFIG arming path.
  2. **The arm is WRITTEN only inside the `if(messages_control.data[0]==SET_CONFIG)`
     block** (`arq_commander.cc:1050`) — a control-TX path. It is never written on
     a data-TX path.
  3. **`finish_turbo_direction()` (`:3693`) unconditionally clears it to false**
     before any `TRANSMITTING_DATA` transition, so even a stale true from a prior
     control wait cannot survive into a data-ACK wait.
  **Walk of the widening on a data ACK**: `turbo_snr_ack_armed_for_gearshift(*, *,
  ACK_RANGE, gear_shift_enabled=true, config_up=true)` → the inner
  `turbo_snr_ack_expected_on_control(*, *, ACK_RANGE)` is false (control_code !=
  SET_CONFIG) AND the new disjunct `… && control_code==SET_CONFIG && …` is false
  (control_code != SET_CONFIG) → returns false. **No leak.** (Part J' FP-J1c asserts
  this with the REAL helper; H2/H2b stay green — H2 drives the SAME ACK_RANGE
  through the *narrow* helper, which §14 does not modify.)
- **What my fix changes**: the arm is now ALSO true on an upward gearshift
  SET_CONFIG control wait (gear_shift_on==YES, config index rising). The SOLE
  consumer then routes THAT control ACK's suffix to the SNR decoder — which is
  exactly the RSP-suffixed SNR the producer needs. No data-ACK consumer is
  affected (the flag remains false on every data-ACK wait, by the three guards).
  H2/H2b/H3 (the narrow-helper assertions) are untouched because they exercise the
  narrow helper, which is byte-identical; FP-J1 adds the widened-helper assertions
  (TRUE on the ladder, FALSE on a data ACK / non-up / gearshift-off).

#### §14.3.2 `measurements.SNR_uplink` (now populated per-rung by the A1 unblock)

- **Producers**: the canonical all-roles producer `arq_common.cc:6051` (LDPC decode
  path — does NOT run on the CMD's MFSK-suffix climb); the §G/OptA suffix-decode
  producer `arq_common.cc:5555` (`snr_uplink_from_suffix`) — which, PRE-§14, was
  dead on the steady-state ladder because the arm was never set (the §14.1
  deadlock). §14 (A1) revives this producer on every upward gearshift SET_CONFIG
  ACK. NO new producer is added by §14 — it only ENABLES the existing `:5555` one.
- **Consumers**: the §13 re-trigger (`arq_commander.cc:4585` gate + `:4587`
  cap-chain via `elevator_target_from_snr`), and NOW the §14 (B) FRAME-UP elevator
  (`arq_commander.cc:3731` gate + `:3733` `elevator_target_from_snr`), plus
  diagnostics (`get_snr_uplink()`).
- **Valid states**: −99.9 (ctor sentinel, "no producer has run") through any
  decoded SNR. Default-init −99.9 — and BOTH elevator consumers gate on
  `SNR_uplink > -90`, so at the sentinel NEITHER fires (DEEP-SNR INERT). This is
  why §14 is byte-identical to the +1 ladder until the producer lifts SNR_uplink
  off the sentinel.
- **Invariant the consumers assume**: a value > −90 is a genuine, recent decode of
  the forward link. Maintained: the only producer that can write it on this path
  (`:5555`) runs ONLY inside the `turbo_snr_ack_enabled` branch, which A1 arms ONLY
  on a SET_CONFIG control-ACK wait — i.e. the value reflects the RSP's measurement
  of the CMD's signal at the rung just ACKed. (Staleness across rungs is a known
  [?] for the wire — see §8 — but it cannot be a SACK/data-ACK value, by §14.3.1.)
- **What my fix changes**: SNR_uplink transitions from "permanently −99.9 on the
  unpinned ladder" to "populated per upward-rung". This is the precondition §13
  was built for (§13.1: "OptA … now populates measurements.SNR_uplink during the
  climb … the precondition that makes this relaxation safe"). Both elevator
  consumers already existed; §14 (A1) just stops starving them, and §14 (B) adds
  the FRAME-UP consumer that fires far more often than the dormant re-trigger.

#### §14.3.3 `last_data_viable_config` (the anchor) — elevator READS, never RAISES

- **Producers** (UNCHANGED by §14): the §1.1 anchor-RAISE (`arq_commander.cc:3437`,
  gated by `promotion_allowed_on_batch` + the §11 sustained-N — a CONFIRMED clean
  batch at the landing config), the §10 DEMOTE (`anchor_demote_target` at the BREAK
  trigger), ctor + reset_session_state init. **§14 adds NO anchor producer.**
- **Consumers** (UNCHANGED set): the four +1-clamp sites (FRAME-UP `:3617`,
  LADDER-UP twin `:4869`, `policy_evaluate_axis1` `:5026`) + `break_target_with_anchor`
  (both BREAK sites). The §13 re-trigger and the §14 (B) FRAME-UP elevator both
  READ it (via `elevator_target_from_snr` → `supershift_retrigger_target`'s
  `anchor_cap` computation) but neither WRITES it — `elevator_target_from_snr()`
  returns an `int` assigned to `negotiated_configuration` / `snr_ideal`, never to
  `last_data_viable_config`.
- **Invariant the consumers assume**: the anchor reflects the highest rung with a
  CONFIRMED clean delivery this session; the +1 clamp and the BREAK floor depend on
  it not being raised speculatively. Maintained: §14 (B) fires the elevator INSIDE
  the clean-batch-confirmed FRAME-UP branch but does NOT touch the §1.1 producer —
  the anchor still rises only on confirmed delivery, and a speculative multi-rung
  landing leaves the anchor at its proven (lower) value. **Part J' FP-J4c asserts
  the FRAME-UP elevator leaves `last_data_viable_config` unchanged.**
- **What my fix changes**: nothing about the anchor's value lifecycle. The elevator
  sets a higher TARGET (`negotiated_configuration`); the anchor follows only when
  that target's first clean batch is confirmed. A failed/overshot jump → BREAK →
  `break_target_with_anchor` recovers UP to the still-low anchor (§10 demotion
  backstops repeated overshoot). This is identical to the §13 re-trigger's
  speculative-landing semantics (§13.3 SAFETY #2/#3), now also on the FRAME-UP path.

#### §14.3.4 The SACK suffix — UNTOUCHED (confirmed)

§14 does not touch the SACK suffix format, CRC, encode/decode, or the
`process_messages_rx_acks_data()` path. The (A1) arm-widening cannot route a SACK
suffix to the SNR decoder (§14.3.1, the three guards). The (B) elevator runs in the
FRAME-UP decision (post-ACK, control-TX side) and only changes
`negotiated_configuration`; the SACK bsi window, `cmd_last_applied_sack_bsi` /
`cmd_last_applied_clean_bsi` dedupe (§4), and the all-ones bitmap width are all
unchanged. **No SACK state is read or written by §14.**

### §14.4 The five safety guards (§13.3) — each PRESERVED by construction

1. **DEEP-SNR INERT (af14a9e):** the (B) elevator block is gated on
   `measurements.SNR_uplink > -90`, and `supershift_retrigger_target` (via the
   shared `elevator_target_from_snr`) keeps the conservative +1 unless its
   high-confidence predicate holds. At WGN:-10 the relayed SNR is low/sentinel →
   no jump → byte-identical to the +1 ladder (Part J' FP-J3a sentinel + FP-J3c
   marginal). The (A1) arm at deep SNR simply never receives a decodable suffix,
   so SNR_uplink stays at the sentinel and the gate stays shut — no behavior
   change at the cliff.
2. **Ceiling cap BEFORE the helper:** `elevator_target_from_snr()` applies the NB
   cap then `min(supershift_proven_ceiling, …)` BEFORE calling
   `supershift_retrigger_target` — VERBATIM from the §13 re-trigger. The FRAME-UP
   elevator inherits it (same method). Part J' FP-J4a/FP-J4b assert the
   proven-ceiling cap binds (target capped at CONFIG_2, not the higher SNR-ideal).
3. **Anchor NOT raised speculatively:** the elevator READS but never WRITES
   `last_data_viable_config` (§14.3.3); the §1.1 confirmed-delivery producer is the
   sole anchor-raise; the (B) elevator fires INSIDE the clean-batch-confirmed
   branch but the anchor still rises only on confirmed delivery. Part J' FP-J4c.
4. **#2 anchor-demotion backstop (e3d818d) UNTOUCHED:** a failed/overshot jump →
   BREAK → `break_target_with_anchor` floors UP to the still-low anchor; repeated
   overshoot → §10 `anchor_consec_break_fails` reaches K=3 → `anchor_demote_target`
   lowers the anchor (Parts E). §14 adds no anchor-raise, so it cannot defeat
   either backstop. (Parts E unchanged, still PASS.)
5. **NO SUPERSHIFT storm:** the (B) FRAME-UP path does NOT set `turboshift_active`
   nor re-enter turbo — it only sets `negotiated_configuration` and queues the
   SAME SET_CONFIG the +1 ladder would. This is CLEANER than the §13 re-trigger
   (which does re-enter turbo); §14 (B) adds zero turbo re-entry. The (A1) arm
   touches only the decode-enable flag, not the turbo state machine.

### §14.5 Part J' (`--test-climb-engine`) — fail-before / pass-after

Appended after the existing Part J (J0-J7, which test the §13 re-trigger helper in
isolation). Part J' drives the NEW §14 pieces; it reuses the Part J scaffolding
(real `get_configuration` / `config_ladder_*` / `SUPERSHIFT_MARGIN_DB`) and adds a
`frameup_target` lambda that replays the REAL FRAME-UP elevator-or-+1 decision body
calling the REAL shared `elevator_target_from_snr()`:

- **FP-J1 (A1 arm fix)**: FP-J1a FAIL-BEFORE proof — the narrow
  `turbo_snr_ack_expected_on_control` is FALSE on a TURBO_DONE gearshift SET_CONFIG
  (the bug). FP-J1b PASS-AFTER — the widened `turbo_snr_ack_armed_for_gearshift` is
  TRUE on a TURBO_DONE upward gearshift SET_CONFIG (gear_shift_on, config_up).
  FP-J1c §5 — FALSE for a DATA ACK (ACK_RANGE) even with gearshift up (SACK
  preserved, no leak). FP-J1d — FALSE for a non-upward SET_CONFIG. FP-J1e — FALSE
  when gear_shift_on==NO. FP-J1f — SUPERSET proof: the widened arm is still TRUE
  when turbo IS active (so H1/H2/H2b/H3/H4 all hold under it).
- **FP-J2 (B elevator multi-rung)**: current=CONFIG_0, SNR_uplink =
  `snr_uplink_from_suffix(14.6f)`, anchor=CONFIG_0, proposed_frame=CONFIG_1,
  optimizer disabled, no proven cap. FP-J2a sanity: snr_ideal
  (=get_configuration(14.6−6.0)=CONFIG_13) is multi-rung above CONFIG_1. FP-J2b
  PASS-AFTER: the FRAME-UP target == CONFIG_13 (multi-rung). FP-J2c FAIL-BEFORE: the
  unconditional +1 yields CONFIG_1. FP-J2d: the elevator differs from the +1 ladder.
- **FP-J3 (B deep-SNR inert)**: FP-J3a sentinel −99.9 → gate unmet → +1 (CONFIG_1).
  FP-J3b/FP-J3c marginal SNR=2.0 (get_configuration(−4.0)=CONFIG_5) modeled at
  current=CONFIG_4/proposed=CONFIG_5: snr_ideal == proposed → no jump → CONFIG_5.
  The WGN:-10 anti-thrash assertion in-process.
- **FP-J4 (B ceiling/anchor cap)**: proven_ceiling=CONFIG_2 + SNR=14.6 → target
  CAPPED at CONFIG_2 (FP-J4a/FP-J4b); the FRAME-UP elevator does NOT raise the
  anchor (FP-J4c, still CONFIG_0).

**FAIL-BEFORE evidence**: FP-J1a (narrow helper false on the ladder) and FP-J2c
(unconditional +1 yields CONFIG_1) ARE the pre-fix outcomes, asserted directly
side-by-side with the PASS-AFTER results (FP-J1b TRUE, FP-J2b CONFIG_13) — the same
side-by-side pattern Parts D/E/F/I/J use. The narrow helper and the
unconditional-+1 path STILL EXIST in the code (the narrow helper is used by the
*turbo* arm; the +1 is the elevator's default), so the fail-before arms are live,
not reverted. **PASS-AFTER**: all Part J' PASS; Parts A-I + the existing J0-J7 +
H2/H2b/H3 stay PASS (§14 is additive — A1 widens a helper into a superset, B adds
an elevator-OR-+1 max that only ever raises the target).

**HONEST scope** (§8 applies): in-process Part J' proves (A1) the widened arm fires
on the ladder case and stays off data ACKs, and (B) the FRAME-UP elevator jumps at
high SNR / is inert at deep SNR / is capped at proven-safe / does not raise the
anchor. The WIRE proofs — SNR_uplink actually leaving −99.9 on the IONOS, the
WGN:-10 no-thrash gate, and the WGN:30 multi-rung jump in one SET_CONFIG — are the
parent's HARDWARE verification. The integration-path timing of the A1-armed decode
(does the RSP suffix survive the CMD's post-TX flush at this rung?) and the
multi-rung SET_CONFIG landing (post-jump rx-mute) are not exercised in-process.

---

---

## §15 DEEP-SNR over-climb REGRESSION — re-assert the data-anchor at the chokepoint (fast-probe follow-up)

**SHIPPED on `fix/fast-probe` (this commit, stacked on 94e80a6).** The §13/§14
controlled elevator, as shipped on 94e80a6, REGRESSED the af14a9e over-climb guard
at the deep-SNR cliff. This section is the record of the root cause (DIAGNOSED, not
re-investigated) and the fix.

### §15.1 The regression (DIAGNOSED — the record)

At WGN:-10 the unpinned cascade (`-Q 0 -M auto -g -R`) over-climbed to CONFIG_9, 0
bytes, 0 BREAKs, where the #2 baseline (446887c) had SETTLED at ROBUST_0. Mechanism:

1. **A1 populates `measurements.SNR_uplink` from the CONTROL-plane MFSK ACK suffix.**
   The §14 (A1) arm-asymmetry repair (`turbo_snr_ack_armed_for_gearshift`,
   `arq_commander.cc`) arms the CMD's SNR-suffix decode on gearshift SET_CONFIG ACKs.
   The RSP measures the CMD's signal and suffixes the SNR; the MFSK control suffix
   decodes at ~1.0 dB **even when OFDM DATA frames at the same SNR cannot decode at
   all.** So `SNR_uplink` leaves the −99.9 ctor sentinel and reads ~1.0.
2. **The −99.9 sentinel used to lock out the re-trigger** (`arq_commander.cc:4667-4668`
   enclosing gate `… && measurements.SNR_uplink > -90`). Once A1 populates it to ~1.0,
   that gate is CLEARED.
3. **`supershift_retrigger_target()` (pre-§15, `arq.h:724-740`) then licensed a
   multi-rung jump** because `high_confidence_jump = (snr_uplink > -90) &&
   idx(snr_ideal) > idx(anchor_cap)` was TRUE: `snr_ideal =
   get_configuration(1.0 − SUPERSHIFT_MARGIN_DB=6.0) = get_configuration(−5.0) =
   CONFIG_4` (telecom_system.cc:5562; −5.0 is NOT > −5, falls to > −6 ⇒ CONFIG_4,
   idx 7), and the anchor was a ROBUST rung (anchor_cap = `config_ladder_up_n(ROBUST_2,
   1, …) = CONFIG_0`, idx 3). `idx7 > idx3` ⇒ the af14a9e +1 clamp was BYPASSED ⇒
   jump CFG_0 → CFG_4.
4. **Then CFG_4 → CFG_9 via the SNR-capped step-1 turbo branch**
   (`arq_commander.cc:4592-4598`) on PHANTOM ACK-pattern matches — the RSP never
   decoded the CFG_4-9 SET_CONFIG data probes (no OFDM data at WGN:-10), and the
   data-fail BREAK path NEVER engaged (the modem was cycling CONTROL probes, never in
   the data-TX-then-fail path).

**The false premise**: "`SNR_uplink > -90` ⇒ safe to jump." **The truth**: an MFSK
control-suffix SNR over-reports the OFDM-DATA-viable rate at deep SNR. The correct
discriminator is whether the channel has PROVEN it can carry OFDM DATA — i.e.
whether the data-viable anchor (`last_data_viable_config`) has reached the OFDM tier.

> **Why "max-leap-from-anchor" ALONE cannot fix this** (the investigation's first
> idea): the LEGIT high-SNR jump is ALSO a large leap from a low anchor (e.g.
> CFG_0 → CFG_13 at WGN:30, before the anchor has ratcheted). A pure leap-size bound
> cannot separate "ROBUST anchor at WGN:-10 leaping to CFG_4" from "OFDM anchor at
> WGN:30 leaping to CFG_13". The **OFDM-vs-ROBUST anchor distinction IS** the
> discriminator; the leap bound is only defense-in-depth on top of it.

### §15.2 The fix — at the SHARED chokepoint `supershift_retrigger_target()`

Both elevator sites (the re-trigger `arq_commander.cc:4682` AND the FRAME-UP
elevator `arq_commander.cc:3733`) reach the clamp via the shared
`elevator_target_from_snr()` → `supershift_retrigger_target()` (`arq.h`). Applying
the fix at the helper covers BOTH with no drift (the documented DSP-commit
anti-pattern).

**PRIMARY (required)** — add `is_ofdm_config(anchor)` to the `high_confidence_jump`
predicate (`arq.h`, `is_ofdm_config` from `common/common_defines.h:79`,
`config>=0 && config<=16`, so ROBUST 100-102 ⇒ false):

```diff
-    bool high_confidence_jump = (snr_uplink > -90) &&
-        config_ladder_index(snr_ideal) > config_ladder_index(anchor_cap);
+    bool high_confidence_jump = (snr_uplink > -90) &&
+        is_ofdm_config(anchor) &&
+        config_ladder_index(snr_ideal) > config_ladder_index(anchor_cap);
```

- **WGN:-10**: the anchor stays ROBUST (OFDM data never delivers, so §1.1's
  confirmed-delivery anchor-raise never fires past the robust rungs) ⇒
  `is_ofdm_config(anchor)=false` ⇒ `high_confidence_jump=false` ⇒ the inner
  `if(idx(snr_ideal) > idx(anchor_cap))` is TRUE ⇒ `snr_ideal = anchor_cap` (the
  af14a9e +1 clamp re-applies) ⇒ NO multi-rung jump ⇒ the modem stays at the
  OFDM-entry rung, the data-fail BREAK path re-engages, and it falls back to ROBUST_0
  (af14a9e behavior RESTORED).
- **WGN:30**: as #2's sustained-clean gate (§11) delivers clean OFDM batches, §1.1
  raises the anchor to CONFIG_0 (the first OFDM rung). `is_ofdm_config(CONFIG_0)=true`
  ⇒ the jump is permitted (the fast multi-rung climb PRESERVED). The fix adds at most
  a small confirm-at-CONFIG_0 delay before the big jump (the anchor must reach an
  OFDM rung first), which is acceptable.

**SECONDARY (defense-in-depth)** — bound a licensed jump to `anchor +
RETRIGGER_MAX_LEAP` (a new tunable, `common_defines.h`, `#define RETRIGGER_MAX_LEAP
13`):

```diff
+    if(high_confidence_jump)
+    {
+      int leap_cap = config_ladder_up_n(anchor, RETRIGGER_MAX_LEAP, robust_en, narrowband);
+      if(config_ladder_index(snr_ideal) > config_ladder_index(leap_cap))
+        snr_ideal = leap_cap;
+    }
```

Even once the anchor is OFDM, a marginal-OFDM channel (CONFIG_0 holds but CONFIG_13
does not) cannot overshoot the whole ladder in one shot — it leaps in bounded steps
as the anchor ratchets up, with the proven-ceiling cap (caller, `:4591-4592`) + the
§10 anchor-demotion backstopping any residual overshoot. The leap-cap NEVER lowers
`snr_ideal` below `anchor_cap` (it is still ≥ a +1 move).

**RETRIGGER_MAX_LEAP = 13 (TUNABLE) — chosen value & rationale.** 13 is the smallest
value that preserves the existing high-SNR multi-rung climb assertions so the WGN:30
fast climb is materially unchanged: a CONFIG_4 anchor still reaches CONFIG_16 in one
leap (idx 7 + 12 = idx 19, gap 12 ≤ 13 ⇒ uncapped — Part J's J1b), and a CONFIG_0
anchor still reaches CONFIG_13 (idx 3 + 13 = idx 16, gap 13 ≤ 13 ⇒ uncapped —
Part J's FP-J2b). For a realistic LOW OFDM anchor the bound still bites: a CONFIG_0
anchor with SNR mapping to CONFIG_16 (gap 16 > 13) is capped at CONFIG_13 (Part J''
high-SNR case) — a genuine bound, leaving the proven-ceiling cap + §10 demotion as
the PRIMARY overshoot backstops and MAX_LEAP as a coarse outer fence. The WGN:30
climb takes at most 2 bounded leaps from a low OFDM anchor (e.g. CONFIG_0 → CONFIG_13
→ CONFIG_16) instead of 1 giant jump — not material.

### §15.3 §5 cross-layer audit (CLAUDE.md §5)

The fix changes the `high_confidence_jump` predicate — over-climb-critical CMD
state. It READS `last_data_viable_config` (the anchor); it adds NO new write.

**Consumers of `high_confidence_jump`** (it is a LOCAL in
`supershift_retrigger_target`; its EFFECT — the multi-rung-vs-+1 target — flows to
the two elevator sites that call the helper through `elevator_target_from_snr()`):

1. **SUPERSHIFT re-trigger** (`arq_commander.cc:4682`, `int snr_ideal =
   elevator_target_from_snr();` then `gap >= SUPERSHIFT_RETRIGGER_CONFIGS` →
   SET_CONFIG to `snr_ideal`). After §15, at a ROBUST anchor `snr_ideal` is clamped
   to anchor+1 ⇒ `gap` from `current_configuration` is small ⇒ the re-trigger
   usually does not even fire (and if it does, it lands +1, not multi-rung).
2. **FRAME-UP elevator** (`arq_commander.cc:3733`, `int snr_ideal =
   elevator_target_from_snr();` then `if(idx(snr_ideal) > idx(proposed_frame))
   negotiated = snr_ideal;`). After §15, at a ROBUST anchor `snr_ideal ≤ anchor+1 ≤
   proposed_frame` (proposed_frame is the +1) ⇒ the `>` is false ⇒ `negotiated`
   stays the +1 (`proposed_frame`) ⇒ BYTE-IDENTICAL to the pre-elevator +1 ladder at
   the cliff.

Both consumers reach the helper through the ONE shared `elevator_target_from_snr()`
(`arq_commander.cc:174-186`), so the fix cannot drift between them.

**Producers / valid states of `last_data_viable_config` (the anchor — the READ
input):**
- **RAISE producer (sole)**: §1.1 `arq_commander.cc:3437-3439`, gated by
  `promotion_allowed_on_batch` + the §11 sustained-N — a CONFIRMED clean batch at
  the landing config. §15 adds NO raise. Valid states: ROBUST_0/1/2 (the robust
  rungs, where it sits until an OFDM batch is confirmed) or CONFIG_0..CONFIG_16.
- **DEMOTE producer**: §10 `anchor_demote_target` (`anchor_consec_break_fails` ≥
  K=3). §15 adds NO demote.
- **Default-init**: the anchor begins at the init/robust config. BEFORE any OFDM
  batch is confirmed it is a ROBUST rung — exactly the state the §15 primary gate
  keys on. This is the WGN:-10 state: the anchor never leaves ROBUST because OFDM
  never delivers, so `is_ofdm_config(anchor)` is false for the whole session ⇒ the
  jump is never licensed.

**The five required verifications:**

1. **WGN:-10 holds ROBUST_0 (af14a9e RESTORED).** Anchor stays ROBUST ⇒
   `high_confidence_jump=false` ⇒ +1 clamp ⇒ no jump ⇒ both elevator consumers keep
   the +1/dormant result ⇒ data-fail BREAK re-engages ⇒ `break_target_with_anchor`
   floors to the still-ROBUST anchor and the breaks≥2 panic-jump (`:149`) escapes to
   ROBUST_0. ✓ (Part J'' deep-SNR case asserts the helper returns ≤ proposed_frame.)
2. **WGN:30 jump PRESERVED.** Once §1.1 raises the anchor to CONFIG_0 (OFDM),
   `is_ofdm_config(anchor)=true` ⇒ the multi-rung jump fires (bounded by MAX_LEAP +
   proven-ceiling). ✓ (Part J'' high-SNR case asserts a capped multi-rung jump;
   existing J1b/FP-J2b assert the un-capped jumps at gaps ≤ 13.) **Verified premise**:
   does the WGN:30 anchor actually reach an OFDM config when the elevator should
   fire? YES — #2's sustained-clean gate (§11) raises it to CONFIG_0 after the first
   clean OFDM batches deliver; the fix therefore adds at most a confirm-at-CONFIG_0
   delay, not a permanent block.
3. **FRAME-UP / LADDER-UP +1 logic, the SACK path, and #2's anti-thrash are
   UNTOUCHED.** §15 edits ONLY `high_confidence_jump` inside
   `supershift_retrigger_target`. The other three +1-clamp consumers
   (`arq_commander.cc:3617` FRAME-UP ceiling, `:4869` / `:5026` LADDER-UP) build
   their target with `config_ladder_up` (structurally +1) and are byte-unchanged
   (§13.3 consumer list (1)). The §14 (A1) arm widening, the SACK suffix decode
   routing (H2/H2b/H3 invariants), and #2's §10 demotion / §11 sustained-N are
   untouched. ✓
4. **Moderate-SNR overshoot is bounded by MAX_LEAP + #2 demotion.** A marginal-OFDM
   channel (anchor reaches CONFIG_0 but CONFIG_13 cliffs) leaps at most +13 per
   re-trigger; if CONFIG_13 then breaks, BREAK lowers `supershift_proven_ceiling`
   (tightening the caller's cap on the NEXT jump) and, if the anchor rung itself
   keeps breaking, §10 demotes the anchor. ✓
5. **No new anchor raise / no new re-entry (SAFETY #3/#5 preserved).** The helper is
   `const`, returns an `int`; the caller assigns it to `snr_ideal`/`negotiated`, not
   to `last_data_viable_config`. The re-entry guards (`turboshift_phase==TURBO_DONE`,
   `turbo_supershift_announce_pending`) are untouched. ✓

**Net**: §15 supplies the MISSING half of the high-confidence premise (the anchor
must have PROVEN the OFDM tier) at the ONE chokepoint both elevators share, plus a
bounded outer fence. Every af14a9e/3b1726a over-climb protection is intact, and the
WGN:-10 over-climb is closed at its true root (the control-plane SNR over-report)
rather than by a threshold band-aid.

### §15.4 Producers / consumers — what §15 changes

- **`high_confidence_jump`** (local): §15 ADDS the `is_ofdm_config(anchor)` conjunct
  (PRIMARY) and the MAX_LEAP cap inside its TRUE branch (SECONDARY). No other code
  reads this local.
- **`last_data_viable_config`** (anchor): §15 adds NO producer; it adds a READ
  (`is_ofdm_config(anchor)`) — the same value `anchor_cap` already read. Unchanged
  producers: §1.1 raise, §10 demote.
- **`RETRIGGER_MAX_LEAP`** (new `#define`, common_defines.h): consumed ONLY by
  `supershift_retrigger_target`. Tunable.

### §15.5 Part J'' (`--test-climb-engine`) — fail-before / pass-after

Appended after Part J' (in `test_climb_engine`). Reuses the Part J scaffolding (real
`get_configuration` / `config_ladder_*` / `SUPERSHIFT_MARGIN_DB` /
`snr_uplink_from_suffix`). A `retrigger_target_v15` lambda models FAIL-BEFORE as the
VERBATIM pre-§15 helper body (the 94e80a6 `high_confidence_jump` WITHOUT
`is_ofdm_config` and WITHOUT the MAX_LEAP cap) and PASS-AFTER as the REAL shipped
`supershift_retrigger_target()`.

- **JJ1 — DEEP-SNR ROBUST-anchor (the regression)**: anchor=ROBUST_2 (a ROBUST
  config), SNR_uplink=`snr_uplink_from_suffix(1.0f)` (the control-plane SNR at
  WGN:-10), current=CONFIG_0, proposed_frame=CONFIG_0's +1 = CONFIG_1, proven=−1.
  `snr_ideal = get_configuration(1.0−6.0=−5.0) = CONFIG_4` (idx 7); anchor_cap =
  `config_ladder_up_n(ROBUST_2,1,robust,wb) = CONFIG_0` (idx 3).
  - **FAIL-BEFORE (pre-§15 body)**: `high_confidence_jump = (1>−90) && (idx7 > idx3)
    = true` ⇒ returns `snr_ideal = CONFIG_4` (the over-climb). Asserted directly.
  - **PASS-AFTER (real helper)**: `is_ofdm_config(ROBUST_2)=false` ⇒
    `high_confidence_jump=false` ⇒ inner clamp ⇒ returns `anchor_cap = CONFIG_0`
    (≤ proposed_frame, the +1 ladder; NO multi-rung jump).
  - Asserts: PASS-AFTER ≤ proposed_frame AND == anchor_cap (CONFIG_0); FAIL-BEFORE ==
    CONFIG_4; the two DIFFER (the fix bites).
- **JJ2 — HIGH-SNR OFDM-anchor (jump PRESERVED + MAX_LEAP cap)**: anchor=CONFIG_0
  (OFDM), SNR_uplink=`snr_uplink_from_suffix(20.0f)`, current=CONFIG_0,
  proposed_frame=CONFIG_1, proven=−1. `snr_ideal = get_configuration(20−6=14) =
  CONFIG_16` (idx 19). `is_ofdm_config(CONFIG_0)=true` ⇒ jump licensed; leap_cap =
  `config_ladder_up_n(CONFIG_0,13,…) = CONFIG_13` (idx 16) ⇒ capped to CONFIG_13.
  - **PASS-AFTER (real helper)**: returns CONFIG_13 — a multi-rung jump (> CONFIG_1)
    bounded by MAX_LEAP (< CONFIG_16). Asserts > proposed_frame AND == CONFIG_13 AND
    ≤ leap_cap.
  - **FAIL-BEFORE (pre-§15 body)**: also jumps, but UNCAPPED to CONFIG_16 — so this
    case ALSO asserts the fix did not BREAK the high-SNR jump (both jump; §15 only
    bounds it). Asserts FAIL-BEFORE == CONFIG_16 and PASS-AFTER == CONFIG_13.
- **JJ3 — sanity: a CONFIG_0 anchor with a small gap is UNCAPPED (climb not slowed)**:
  anchor=CONFIG_0, SNR_uplink=`snr_uplink_from_suffix(14.6f)` ⇒ snr_ideal=CONFIG_13
  (idx 16, gap 13 == MAX_LEAP) ⇒ NOT capped (leap_cap=CONFIG_13). PASS-AFTER ==
  CONFIG_13 (identical to FP-J2b's un-capped target) — documents that the WGN:30
  fast climb is materially unchanged at gaps ≤ 13.

**FAIL-BEFORE evidence**: JJ1 asserts the pre-§15 body returns CONFIG_4 (the
over-climb) side-by-side with the §15 helper returning CONFIG_0 (the +1 clamp) — the
same side-by-side pattern Parts D-J use. Reverting the §15 helper body to the
94e80a6 predicate makes JJ1's PASS-AFTER assertion (helper returns CONFIG_0) FAIL.

**PASS-AFTER**: JJ1/JJ2/JJ3 PASS; Parts A-I + the existing J0-J7 + Part J' (FP-J1..4)
+ H2/H2b/H3 stay PASS (§15 is additive — it only TIGHTENS the jump predicate at a
ROBUST anchor and BOUNDS it at an OFDM anchor; every existing high-SNR assertion uses
an OFDM anchor at a gap ≤ 13, so it is unaffected).

**HONEST scope** (§8 applies): in-process Part J'' proves the jump is BLOCKED at a
ROBUST anchor (regression closed) and PRESERVED-but-bounded at an OFDM anchor. The
WIRE proofs — WGN:-10 actually holding ROBUST_0 (anchor never leaves ROBUST on the
IONOS) and WGN:30 keeping the fast ~3k climb — are the parent's HARDWARE re-test.

---

## §16 ANCHOR-TIER-CORRUPTION — credit the delivered config + clamp the turbo ladder

**SHIPPED on `fix/fast-probe` (this commit, stacked on edb8600).** §15 (edb8600)
re-asserted the data-anchor INSIDE the elevator helper (`is_ofdm_config(anchor)` on
`high_confidence_jump`) — necessary, but it only gated the ONE elevator chokepoint.
It was NECESSARY-but-INSUFFICIENT: it moved the breach. Two siblings the §15
elevator-only gate did not cover:

1. **The anchor-raise producer itself could be POISONED into the OFDM tier on
   robust evidence** (ROOT-1). Once the anchor reaches an OFDM config,
   `is_ofdm_config(anchor)=true` and the §15 elevator gate OPENS, the SNR
   re-trigger fires (cmd.log:4829 `current 0`), and the turbo launches.
2. **The turbo-forward ladder has NO anchor clamp** (ROOT-2). The SNR-SUPERSHIFT /
   SNR-capped-step-1 / blind targets (`arq_commander.cc:4602-4621`) are bounded only
   by the proven-ceiling + WB/NB ceiling — NOT by `last_data_viable_config`. So a
   launched turbo ratchets CONFIG_4 → CONFIG_9 on phantom ACK matches with no anchor
   bound. The §15 fix never touched this path.

PRE-EXISTING: baseline 446887c also corrupted the anchor to CONFIG_0, but
`frame_shift_threshold=3` climbed slowly enough that BREAK/demote caught it
pre-rocket. The adaptive fast-probe (③, threshold→1) climbs past the catch point,
so the latent corruption became a live over-climb. **This is a PRE-EXISTING bug
exposed by the fast-probe, not a fast-probe bug.**

### §16.0 STEP 0 — the EXACT anchor-raise ordering (cited; the precise hole)

The diagnosis hypothesised a same-pass ordering bug ("the 3rd ROBUST ACK is credited
AFTER the FRAME-UP SET_CONFIG advanced the config to CONFIG_0"). Reading the code
PINS the exact ordering — and the literal same-pass hypothesis does NOT hold; the
real hole is subtler:

- **Anchor-raise (the credit)**: `arq_commander.cc:3617-3636`, inside the
  `data_ack_received==YES` branch (the DATA-ACK pass). PRE-§16 it set
  `last_data_viable_config = current_configuration` (the **live** config), gated by
  `clean_batches_at_current_config >= sustained_anchor_threshold(current_configuration)`.
- **FRAME-UP config-advance**: the FRAME-UP block (`arq_commander.cc:3654-3764`)
  sets `negotiated_configuration = proposed_frame` (`:3729`) — it does **NOT** write
  `current_configuration`. It queues a SET_CONFIG and `return`s (`:3763`).
- **`current_configuration` actually advances in a SEPARATE pass** at the
  SET_CONFIG-ACK apply site `arq_commander.cc:4456-4460`
  (`prev_configuration = current_configuration; load_configuration(data_configuration,…)`).
  AND `data_configuration` is advanced even earlier, at SET_CONFIG **TX** time
  (`arq_commander.cc:1082-1086`, `data_configuration = negotiated_configuration` when
  the promotion frame is queued) — BEFORE the RSP ACKs and before any data delivers
  at the new rung.

**Conclusion (cited)**: on the STEADY-STATE data path the credit at `:3617` sees the
delivered `current_configuration` — there is no literal same-pass mis-credit. The
hole is that the producer credited the **LIVE** `current_configuration`, which (a)
both `current_configuration` AND `data_configuration` race ahead of the delivered
batch once a FRAME-UP/turbo SET_CONFIG is queued/applied, and (b) `clean_batches_*`
(the only state that records WHERE the cleans actually landed) is the authoritative
"delivered config", not the live config. A robust-fragment clean credit that fires
while the live config has crossed to CONFIG_0 (a late/duplicate robust ACK, or two
phantom all-ones matches counted "at" CONFIG_0) seats the anchor at CONFIG_0 on
robust evidence. The fix is to credit `clean_batches_config` (the streak's HOME) and
to refuse a ROBUST→OFDM crossing whose live tier disagrees with the streak's claim —
i.e. "capture the delivered config before the advance," exactly as the diagnosis
prescribed, just expressed via the streak-home rather than a same-pass capture.

### §16.1 ROOT-1 — the anchor reflects PROVEN delivery PER TIER

**The PURE helper** `data_anchor_raise_target(streak_config, live_config,
current_anchor, clean_streak)` (`arq.h`, immediately after
`sustained_anchor_threshold`) is now the SOLE on-delivery anchor-raise decision:

- **Before (edb8600, `arq_commander.cc:3617-3620`)**:
  ```cpp
  if(clean_batches_at_current_config >= sustained_anchor_threshold(current_configuration) &&
     config_ladder_index(current_configuration) > config_ladder_index(last_data_viable_config))
      last_data_viable_config = current_configuration;     // credits the LIVE config
  ```
- **After (`arq_commander.cc:3634-3636`)**:
  ```cpp
  last_data_viable_config = data_anchor_raise_target(
      clean_batches_config, current_configuration,
      last_data_viable_config, clean_batches_at_current_config);
  ```

The helper: (Rule 2) requires `clean_streak >= sustained_anchor_threshold(streak_config)`
(§11 — robust N=1, OFDM N=2); (Rule 1) never lowers; (TIER GATE) credits
`streak_config` (the cleans' HOME — survives the live-config advance) and refuses a
`is_robust(current_anchor) && is_ofdm(streak_config)` crossing when
`!is_ofdm_config(live_config)` (the corruption signature: the streak claims OFDM but
the live tier is robust). Because a ROBUST-tier delivery has a ROBUST `streak_config`,
it can only ever seat a ROBUST anchor — a ROBUST fragment-ACK can NEVER push the
anchor past the top ROBUST rung. A within-ROBUST advance (ROBUST_1→ROBUST_2) is
UNAFFECTED (there `streak_config` is robust ⇒ the cross condition is false). A genuine
OFDM delivery (`streak_config`=CONFIG_0, N_OFDM=2 cleans AT CONFIG_0) raises the
anchor to CONFIG_0 unimpeded — WGN:30 preserved (§16.3 below).

On the steady-state data path `clean_batches_config == current_configuration` (§11
pins them on the same clean), so the change is a NO-OP there; it BITES only when the
live config has advanced ahead of the streak — the corruption scenario.

### §16.2 ROOT-2 — clamp the unanchored turbo-forward ladder

The turbo target (`negotiated_configuration` after the three SNR branches +
the WB/NB ceiling, `arq_commander.cc:4602-4629`) is now routed through the SAME
shared chokepoint the §13/§14/§15 elevator uses (`arq_commander.cc:4631-4635`):

```cpp
negotiated_configuration = supershift_retrigger_target(
    negotiated_configuration, effective_snr,
    last_data_viable_config, optimizer_is_in_control(),
    robust_enabled, narrowband_enabled == YES);
```

`supershift_retrigger_target` (the §15-hardened helper): at a ROBUST anchor
(`is_ofdm_config(anchor)=false`) it clamps the target to `anchor+1`; at an OFDM
anchor it bounds the jump to `anchor + RETRIGGER_MAX_LEAP (=13)`; when the optimizer
owns the band it returns the target unchanged (Q-table authority preserved). It only
ever LOWERS the target (never raises), READS but never RAISES the anchor.

- **WGN:-10** (anchor stays ROBUST by ROOT-1): a turbo launched at the OFDM-entry
  rung is clamped to ROBUST_2+1=CONFIG_0 — it CANNOT ratchet up the OFDM tier. The
  target falls to/below `current_configuration`, the data-fail BREAK re-engages, and
  the link falls back to ROBUST_0 (af14a9e restored on the turbo path too). The
  CONFIG_4→9 ratchet is dead.
- **WGN:30** (anchor reaches CONFIG_0 legitimately): the turbo from CONFIG_0 at high
  SNR is bounded to CONFIG_13 (anchor+MAX_LEAP) — a big multi-rung jump, PRESERVED.
  As the anchor ratchets CONFIG_13→CONFIG_16 the climb reaches the top in ≤2 bounded
  leaps (§15.2's documented behavior).

**With ROOT-1 keeping the anchor at ROBUST at WGN:-10, ROOT-2 clamps the turbo to
ROBUST → no OFDM ratchet.** The two fixes are complementary: ROOT-1 makes the anchor
honest; ROOT-2 bounds every turbo/elevator target by that honest anchor.

### §16.3 §5 cross-layer audit (CLAUDE.md §5)

Shared state touched: `last_data_viable_config` (the anchor — ROOT-1 changes the
RAISE producer's INPUT config; ROOT-2 adds a READ at the turbo chokepoint) and
`negotiated_configuration` (ROOT-2 lowers the turbo target). NO new anchor WRITER is
added — both fixes are read-time discipline.

**Producers of `last_data_viable_config`** (UNCHANGED set):
- RAISE (sole on-delivery): §1.1/§11 → NOW the `data_anchor_raise_target()` helper
  (`arq_commander.cc:3634`), keyed on `clean_batches_config`. Still the ONLY
  on-delivery raise; still gated by `promotion_allowed_on_batch` + §11 sustained-N.
- DEMOTE: §10 `anchor_demote_target` (`arq_commander.cc:3503`). UNTOUCHED.
- ctor + reset_session_state init. UNTOUCHED.

**Consumers of `last_data_viable_config`** (UNCHANGED set): the four +1-clamp sites
(FRAME-UP `:3667`, LADDER-UP twin `:4869`/`:5026`), `break_target_with_anchor` (both
BREAK sites), and the elevator chokepoint `supershift_retrigger_target` (re-trigger
`:4682` + FRAME-UP elevator `:3733`) — and NOW ALSO the ROOT-2 turbo-ladder clamp
(`:4631`, the SAME helper). ROOT-2 adds ONE consumer (the turbo clamp) that READS the
anchor; it adds no producer.

**The five required verifications:**

1. **WGN:30 PRESERVED (ROOT-1 does NOT delay/block a real OFDM delivery).** At high
   SNR the OFDM tier delivers REAL full batches, so the ROBUST→OFDM boundary
   qualifier is satisfied immediately: two consecutive clean CONFIG_0 batches
   (N_OFDM=2) seat `clean_batches_config=CONFIG_0, streak=2`; `data_anchor_raise_target`
   then returns CONFIG_0 (the cross is `is_ofdm(streak_config=CONFIG_0)=true`, and
   `live_config=CONFIG_0` is OFDM so the belt-and-suspenders guard does NOT fire).
   The anchor reaches CONFIG_0 LEGITIMATELY, `is_ofdm_config(anchor)=true`, and §15 +
   ROOT-2-capped turbo still do the multi-rung jump (bounded to anchor+MAX_LEAP).
   The fix adds at most a confirm-at-CONFIG_0 delay (the 2nd clean), NOT a permanent
   block. **Part K''2/K''3 assert exactly this** (2nd CONFIG_0 clean raises the
   anchor; the elevator then jumps multi-rung from the OFDM anchor). ✓
2. **#2 anti-thrash PRESERVED/STRENGTHENED.** ROOT-1 makes the anchor HARDER to raise
   (it must reflect the streak's home tier), which STRENGTHENS the BREAK-to-ROBUST_0
   path (`break_target_with_anchor` floors to a LOWER, honest anchor). ROOT-2 only
   LOWERS turbo targets (never raises), so no new thrash is introduced. The §10
   anchor-demotion and §11 sustained-raise are UNTOUCHED. `break_target_with_anchor`
   (`arq_commander.cc:147-154`) is byte-unchanged — it still floors to
   `last_data_viable_config` (now honest) and bypasses under the breaks≥2 panic. ✓
3. **No NEW producer of `last_data_viable_config`.** Both fixes are read-time
   clamps/qualifiers. ROOT-1 changes which config the EXISTING raise credits (a
   read-time choice between `clean_batches_config` and the live config); ROOT-2 adds a
   READ at the turbo clamp. Neither adds a write site. ✓
4. **The +1-clamp / LADDER-UP / SACK paths are UNTOUCHED.** ROOT-1 edits only the
   anchor-raise producer; ROOT-2 edits only the turbo-ladder target. The FRAME-UP +1
   clamp (`:3667`), the LADDER-UP twins, the SACK suffix decode, and the §14 (A1) arm
   are byte-unchanged. ✓
5. **optimizer (Q-table) authority PRESERVED.** ROOT-2's clamp passes
   `optimizer_is_in_control()` to `supershift_retrigger_target`, which returns the
   target unchanged when the Q-table owns the band — the same exemption the elevator
   and the +1 clamp already honor. ✓

### §16.4 Producers / consumers — what §16 changes

- **`last_data_viable_config`** (anchor): ROOT-1 changes the RAISE producer's input
  config (now `clean_batches_config`, the streak home) via the new helper; ROOT-2
  adds a READ at the turbo clamp. No new write site.
- **`negotiated_configuration`** (turbo target): ROOT-2 routes it through
  `supershift_retrigger_target` after the ceiling caps — the post-clamp value is
  LOWERED at a robust anchor / bounded at an OFDM anchor; unchanged when ≤ anchor+1
  or when the optimizer owns.
- **`data_anchor_raise_target`** (new PURE helper, `arq.h`): consumed ONLY by the
  anchor-raise producer (`:3634`) and Part K (unit test).

### §16.5 Part K / K' / K'' (`--test-climb-engine`) — fail-before / pass-after

Appended after Part J''. Drives the ACTUAL production decisions (the SAME helpers
production calls), not just the elevator helper in isolation — the gap that gave the
earlier Part J''/JJ1 FALSE confidence (elevator modeled in isolation, PASSED while
HW FAILED).

- **Part K (ROOT-1, FRAME-UP boundary)**: `anchor_raise_v16` replays the REAL
  `data_anchor_raise_target()` (PASS-AFTER, `adaptive=true`) side-by-side with the
  VERBATIM pre-§16 producer body (credit the LIVE config, FAIL-BEFORE,
  `adaptive=false`). **K1**: a robust streak (home ROBUST_2, streak=2) credited while
  live=CONFIG_0 → FAIL-BEFORE credits the LIVE CONFIG_0 (anchor poisoned into OFDM);
  PASS-AFTER credits the home ROBUST_2 (stays ROBUST, ≤ ROBUST_2); the two DIFFER
  (the fix bites). **K2**: the tier-gate belt-and-suspenders (streak claims OFDM but
  live tier robust) is REFUSED. **K3**: a within-ROBUST advance (ROBUST_1→ROBUST_2,
  N_ROBUST=1) is PRESERVED. **K4**: a single CONFIG_0 clean (N_OFDM=2 unmet) does NOT
  raise the anchor.
- **Part K' (ROOT-2, turbo-forward clamp)**: `turbo_clamp_v16` replays the EXACT
  production turbo-clamp call (`supershift_retrigger_target(neg_config, effective_snr,
  anchor, false, robust_en, nb)`, the call at `:4631`) — PASS-AFTER (`adaptive=true`)
  vs the pre-§16 unclamped path (`adaptive=false`). **K'1**: anchor=ROBUST_2, a turbo
  wanting CONFIG_9 at the control-plane SNR → FAIL-BEFORE ratchets to CONFIG_9
  (unbounded); PASS-AFTER clamps to anchor+1=CONFIG_0 (≤ `config_ladder_up_n(ROBUST_2,
  MAX_LEAP)`, strictly below CONFIG_9). **K'2**: OFDM anchor=CONFIG_0 at high SNR →
  PRESERVED but bounded to CONFIG_13. **K'3**: a step at anchor+1 is UNCHANGED (the
  clamp never raises).
- **Part K'' (WGN:30 preserved, end-to-end)**: drives the REAL
  `data_anchor_raise_target()` + the REAL `supershift_retrigger_target()`. **K''1/2**:
  one CONFIG_0 clean does NOT raise the anchor; the 2nd consecutive CONFIG_0 clean
  RAISES it to CONFIG_0 (genuine OFDM delivery). **K''3/4**: from the now-OFDM anchor
  the elevator jump STILL fires (multi-rung, bounded to CONFIG_13). Proves ROOT-1 does
  not block the legit OFDM climb.

**FAIL-BEFORE evidence**: (a) each Part has a live side-by-side `adaptive=false` arm
asserting the pre-§16 outcome (K1a credits CONFIG_0; K'1a ratchets to CONFIG_9) — the
same documented pattern as Parts D/E/F/I/J/J''; (b) ADDITIONALLY verified 2026-05-31
by temporarily reverting the REAL shipped `data_anchor_raise_target()` body to the
pre-§16 "credit the live config" form and the ROOT-2 turbo call to a no-op,
rebuilding: **K1b/K1c/K1d/K2 FAIL** (the production helper returns CONFIG_0 / the
poisoned OFDM anchor instead of ROBUST_2/ROBUST_1); K3/K4/K''1/K''2/K''3/K''4
correctly STAY PASS (within-tier + genuine-OFDM cases are invariant; K''3 calls the
un-reverted elevator helper). The temp revert was reverted; the shipped helpers do
the discipline. **PASS-AFTER**: all Part K/K'/K'' PASS; Parts A–J + J'/J'' +
H2/H2b/H3 stay PASS (§16 is additive — ROOT-1 changes only WHICH config the raise
credits, ROOT-2 only LOWERS the turbo target). `--test` 30/30.

**HONEST scope** (§8 applies): in-process Part K/K'/K'' proves the anchor HOLDS
ROBUST on robust deliveries (ROOT-1), the turbo target is CLAMPED at a robust anchor
(ROOT-2), and the WGN:30 multi-rung jump is PRESERVED from a genuine OFDM anchor. The
turbo-clamp test replays the production call's exact args (the SAME helper, the SAME
chokepoint) — but the full in-process drive of `process_messages_commander()`'s
turbo branch is NOT exercised (the §15.5 limitation). The WIRE proofs — WGN:-10
actually holding ROBUST_0 (anchor never leaves ROBUST + no CFG_4→9 ratchet on the
IONOS) and WGN:30 keeping the fast ~3k climb — are the parent's HARDWARE re-test.

---

## §9 Related fact documents

- `data-flow-messages_rx_prev.md` — the prev-storage state Bug 1 touches (RSP
  side). This doc's §4 RSP clean-confirm emit is a NEW producer of an ACK from
  that path; the prev-storage lifecycle itself is unchanged.
- Forward refs in code to `gearshift-start-and-recovery.md` (§6/§7/§9) describe
  the Option-B anchor + clean-batch-viability design; that doc lives on the C1/C3
  branches and is not present on this base. The relevant facts are restated here.
