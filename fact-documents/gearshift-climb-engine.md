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

**Out of scope (architectural fork ① — the user's pending decision):** the +1
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

---

## §9 Related fact documents

- `data-flow-messages_rx_prev.md` — the prev-storage state Bug 1 touches (RSP
  side). This doc's §4 RSP clean-confirm emit is a NEW producer of an ACK from
  that path; the prev-storage lifecycle itself is unchanged.
- Forward refs in code to `gearshift-start-and-recovery.md` (§6/§7/§9) describe
  the Option-B anchor + clean-batch-viability design; that doc lives on the C1/C3
  branches and is not present on this base. The relevant facts are restated here.
