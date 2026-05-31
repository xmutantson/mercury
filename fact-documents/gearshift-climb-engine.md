# Fact Document: Gearshift CLIMB Engine — off-ROBUST_0 multi-rung promotion

**Status**: Authoritative as of 2026-05-30 on `fix/climb-engine` (branched from
`fix/cfg16-nv-restore` @ 05012a9). Investigation of the "unpinned cascade can't
climb off ROBUST_0 into the OFDM tier" failure. Supersedes the per-single
analyses on `fix/climb-delivery-anchored` (C1), `fix/climb-batch1-robust` (C2),
`fix/climb-combined` (C3) — all three FAILED on the IONOS wire.

This doc owns the producer/consumer/invariant facts for the CLIMB promotion
state shared CMD↔RSP: `last_data_viable_config`, `consecutive_data_acks`,
`gear_shift_blocked_for_nBlocks`, `last_batch_fully_acked`, `data_batch_size`
at robust configs, and the SACK bsi window during promotion.

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

**Fix**: gate BOTH blocks on `!is_robust_config(...)`. CMD gates on
`negotiated_configuration`, RSP on `current_configuration`; at TEST_CONNECTION
time both equal the established connect config, so they agree (symmetry
mandatory — an asymmetric override was historical Bug #9). This is the standalone
C2 batch-floor fix, verbatim.

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
  gated by §3), `policy_evaluate_axis2` (CMD `:5263` via SET_LINK_PARAMS — NOW
  `return`s early at robust by §2), RSP SET_LINK_PARAMS apply
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

`--test-climb-engine` (`test_climb_engine`, `arq_commander.cc`). Three parts,
each fail-before / pass-after:

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

Local tests are NECESSARY but NOT SUFFICIENT — the singles passed local and
failed the wire. §8 flags the integration-path assumptions that still need
hardware.

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

## §9 Related fact documents

- `data-flow-messages_rx_prev.md` — the prev-storage state Bug 1 touches (RSP
  side). This doc's §4 RSP clean-confirm emit is a NEW producer of an ACK from
  that path; the prev-storage lifecycle itself is unchanged.
- Forward refs in code to `gearshift-start-and-recovery.md` (§6/§7/§9) describe
  the Option-B anchor + clean-batch-viability design; that doc lives on the C1/C3
  branches and is not present on this base. The relevant facts are restated here.
