# Track-C — D2 / D3 / D5 coupled-defect §5 data-flow audit + fix design (2026-06-11)

**Status: DESIGN-FIRST (CLAUDE.md §2 STOP-and-design + §5 cross-layer audit). NO code
written this session.** This is the mandatory design+audit gate for the three remaining
defects in the self-climbing CFG16-acquisition chain. Every file:line below was
**re-located against the monitor tip `627c370`** (worktree
`C:/Users/kamer/mercury_wt/trackc-d2d3d5`, branch `sim/trackc-d2d3d5-design`) and
verified by reading the live source. Where a cited line differs from a prior doc that
referenced the `cfg16-acq-d2d3` worktree (`05fb9b7`) or the `40b184f` tree, **this doc's
line numbers win** (they are the monitor-tip lines a merge would touch).

The companion fact document `fact-documents/data-flow-prev-bump.md` owns the prev cross-
storage in-order invariant and is updated in lockstep (its §5.6 / new §8 carry the D5
audit). This doc owns the **gearshift acq → delivery layer-boundary chain** (D2+D3+D5)
as one coupled unit, because the three cannot be merged independently without re-opening
a silent-wrong-bytes window (§6 MERGE ORDER).

---

## §0 BLUF — the coupled chain in one paragraph

The self-climbing ROBUST_0 → CFG16 acquisition is a **3-layer coupled defect** sitting on
the gearshift-acq → delivery boundary, surfaced by the D1 acq-fix (`climb_effective_snr_
margin_db`, already designed `_cfg16acq2/ROOTCAUSE_AND_FIX.md` §3, **not on monitor**)
that lets the climb COMPUTE a CFG16 target where it previously double-counted the 6 dB
fading margin and capped at CONFIG_13. With D1 in place: **D2** —
`supershift_retrigger_target` (arq.h:1028) RE-CLAMPS the CFG16 target to
`leap_cap = config_ladder_up_n(anchor, RETRIGGER_MAX_LEAP=13)` (arq.h:1063); from an
un-ratcheted CONFIG_0 anchor (ladder idx 3) that `leap_cap == CONFIG_13`. **D3** — when
the clamped target equals the current config (CONFIG_13), the turbo emits an
UNCONDITIONAL `add_message_control(SET_CONFIG)` no-op-to-self (arq_commander.cc:5447-5449
region in the d2d3 tree; the monitor turbo branch around arq_commander.cc:5413) and
re-enters TRANSMITTING_CONTROL — an infinite CONTROL spin (executed: 72 ctrl : 8 data,
~113 B) that STARVES data TX, so the delivery-gated anchor producer
`data_anchor_raise_target` (arq_commander.cc:4266) never fires, the anchor stays
CONFIG_0, and `leap_cap` stays CONFIG_13 forever → the wedge. **D5** — IF the climb
DOES reach CFG16 (e.g. via the D2/D3 fix or a rung-by-rung realization), the prev-bump
cross-storage delivery silently drops a frame: `bump_bsi_and_transfer_prev`
(arq_common.cc:6382-6389) derives `prev_expected = min(data_batch_size,
last_received_end_of_batch_seq + 1)`; when the LAST (EOB-bit-7) frame of a bumped batch
is LOST, `last_received_end_of_batch_seq` reflects a lower index, `expected_count` is set
SHORTER than the TX's true frame count, and the genuinely-missing tail frame in
`[expected_count, data_batch_size)` is dropped — a **one-frame ~155-B SILENT SKIP**, NO
`[RSP-V2-GAP-ABORT]`, md5_match FALSE (PREV_BUMP_VERDICT.md §2, executed). **All three
must land together (D5 is the integrity backstop for the regime D2/D3 unlocks).**

---

## §1 The shared state — declarations (monitor 627c370)

Three pieces of cross-layer state, each crossing ≥ 2 layers (gearshift/optimizer ↔ ARQ
control-plane ↔ delivery), so all three are §5-audit-mandatory.

### 1.1 `last_data_viable_config` (the climb anchor)
- `arq.h` member `int last_data_viable_config;` — the af14a9e data-viability anchor; the
  HIGHEST OFDM rung at which a DATA batch was CONFIRMED FULLY delivered. Init to the
  ROBUST_0 floor at session start (`arq_common.cc` session init; the §16 doc cites
  `arq_common.cc:722` for the ROBUST_0 floor).
- Crosses: **delivery layer** (raised only on a clean OFDM batch DELIVERY) → **gearshift
  layer** (gates the elevator/turbo leap_cap via D2's `supershift_retrigger_target`).

### 1.2 `supershift_retrigger_target()` output (the climb-target clamp)
- `arq.h:1028-1075` (signature :1028) — PURE policy. Returns the elevator/turbo target
  config, clamped to `config_ladder_up_n(anchor, RETRIGGER_MAX_LEAP)` (`leap_cap`,
  arq.h:1063) when the high-confidence jump is licensed (arq.h:1051), else to
  `anchor_cap = anchor+1` (arq.h:1034, applied :1073). `RETRIGGER_MAX_LEAP = 13`
  (common_defines.h:406).
- Consumes `last_data_viable_config` (the `anchor` argument). The leap_cap value is a
  PURE FUNCTION of the anchor → the wedge is data-coupled to §1.1 not advancing.

### 1.3 The TX-authoritative per-batch frame count (today: INFERRED, never wired)
- TX side: `message_batch_counter_tx` (the count of frames the CMD packed into the
  current radio batch) — the AUTHORITATIVE length. The EOB marker is the ONLY signal of
  it on the wire: `messages_batch_tx[i].sequence_number |= 0x80` on the LAST data frame
  (`arq_common.cc:5610-5614`, monitor; the line printed `Mark last DATA frame in batch
  with bit 7 so responder knows actual batch size`).
- RX side: `int last_received_end_of_batch_seq;` (`arq.h`, -1 = none) — set from the
  bit-7 EOB frame's seq (`arq_common.cc:8966-8974`, `rx_buffer_eob_seq = eob_seq` for v2,
  promoted to `last_received_end_of_batch_seq` on match-current at arq_responder.cc:887 in
  the 40b184f tree / the R038 promote block). It is the SOLE producer of `expected_count`
  (D5) AND the SACK-bitmap span (§4.3) AND the in-place ACK-GATE `effective_batch`.
- **The defect class: a single-frame-of-evidence length channel.** The batch length is
  carried by exactly ONE frame (the EOB-marked last frame). That frame's loss erases the
  length, and every downstream consumer silently substitutes the highest-seq-seen.

---

## §2 PRODUCERS (every writer, file:line, monitor 627c370)

### 2.1 `last_data_viable_config` producers
- **P-A (the gated raise)** `arq_commander.cc:4266-4267`:
  `last_data_viable_config = data_anchor_raise_target(clean_batches_config,
  current_configuration, last_data_viable_config, clean_batches_at_current_config);`
  Inside the `promotion_allowed_on_batch()` CLEAN branch, reached ONLY when
  `data_ack_received==YES` AND a batch FULLY delivered at this config
  (arq_commander.cc:3730-3766 region in 40b184f; the monitor block around :4230-4267).
  `data_anchor_raise_target` (arq.h:821-854) keeps the §11 sustained-N gate, only-ever-
  raises, and the §16 tier gate (ROBUST→OFDM cross only on an OFDM streak_config). **This
  is the SOLE on-delivery anchor-raise producer.** A failed probe / partial batch / pure
  CONTROL spin NEVER reaches here → the D3 starvation freezes the anchor.
- **P-B (demotion)** the §10 break/demote path lowers the anchor on recovery (not a raise;
  not in the D2/D3 wedge path).
- **P-C (init)** session init seats the anchor at the ROBUST_0 floor.

### 2.2 `supershift_retrigger_target` callers (the consumers of §1.1 that emit §1.2)
- **C-1 in-turbo SNR-SUPERSHIFT** `arq_commander.cc:5413-5419` (monitor):
  `negotiated_configuration = supershift_retrigger_target(negotiated_configuration,
  effective_snr, last_data_viable_config, optimizer_is_in_control(), robust_enabled,
  narrowband_enabled == YES);` — **THE binding wedge site.** The D1 acq-fix makes the
  SUPERSHIFT branch above compute `negotiated_configuration = CFG16`; this call clamps it
  to leap_cap. With anchor=CONFIG_0, clamp → CONFIG_13.
- **C-2 the elevator** `elevator_target_from_snr()` `arq_commander.cc:179-189` (calls
  `supershift_retrigger_target` at :187) — the FRAME-UP elevator + re-trigger sites
  (40b184f :4402 / :5565). Same clamp.

### 2.3 The EOB-length / `expected_count` / SACK-span producers (D5 core)
- **EOB-mark TX** `arq_common.cc:5610-5614` — bit-7 set on the last data frame only.
- **EOB-decode RX** `arq_common.cc:8966-8974` — `rx_buffer_eob_seq = eob_seq` (v2 stage),
  `last_received_end_of_batch_seq = eob_seq` (v1 direct).
- **prev `expected_count`** `arq_common.cc:6382-6389` (`bump_bsi_and_transfer_prev`):
  `int prev_expected = data_batch_size; if(last_received_end_of_batch_seq >= 0){ int eob =
  last_received_end_of_batch_seq + 1; if(eob < prev_expected) prev_expected = eob; }` →
  written to `rsp_prev_batch_expected_count` (arq_common.cc:6435 region). **D5 ROOT
  PRODUCER: a lost EOB frame makes `eob` smaller than the true frame count.**
- **in-place ACK-GATE `effective_batch`** `arq_responder.cc` (the 40b184f :895-900 block;
  monitor equivalent around the EOB-loss comment at arq_responder.cc:1087): same
  `eob = last_received_end_of_batch_seq + 1` derivation for the CURRENT-batch ACK gate.
- **SACK-bitmap `expected`** `arq_responder.cc:1639` (monitor):
  `expected = last_received_end_of_batch_seq + 1;` — the span the partial-SACK gate
  (`rx_received < expected`, arq_responder.cc:1672) compares against. **Critical: when the
  EOB frame is lost, `expected` = highest-seq-seen → `rx_received >= expected` → the gate
  at :1672 is FALSE → NO SACK is sent for the lost tail → the lost frame is NEVER
  retransmitted (the RX does not know it exists).** This is why D5 is unrecoverable by the
  existing retx machinery and why direction-1 (wire the count) is mandatory, not optional.

---

## §3 CONSUMERS (every reader, file:line, monitor 627c370)

### 3.1 `last_data_viable_config` consumers
- **D2 clamp** `supershift_retrigger_target` (arq.h:1034 anchor_cap, :1063 leap_cap) — the `anchor` →
  `anchor_cap`/`leap_cap`. (§2.2 callers.)
- **BREAK recovery** `break_target_with_anchor` (the §10 recovery target) — recovers to
  the anchor on a failed probe.
- **WALL-B FIX-7 CEILING settle** (arq.h:~1207 `is_ofdm_config(anchor)` gate) — settle-vs-
  break discriminator reads the anchor.
- **robust-dwell eligibility** `robust_dwell_batch_eligible_core` (arq.h:896-910, conjunct
  (c) `ladder_idx(current) <= ladder_idx(anchor)`).

### 3.2 `supershift_retrigger_target` output consumers
- **The turbo state machine** `arq_commander.cc:5413+` consumes `negotiated_configuration`:
  on monitor it falls through to `add_message_control(SET_CONFIG)` +
  `connection_status=TRANSMITTING_CONTROL` UNCONDITIONALLY (the D3 no-op spin — there is
  NO `supershift_clamp_yields_to_data` guard on monitor; confirmed `grep -c = 0`).

### 3.3 The EOB-length / `expected_count` consumers (D5)
- **prev completion GATE** `arq_responder.cc:787` — `... && rsp_prev_batch_received_count
  >= rsp_prev_batch_expected_count` (a COUNT test; even with the L1 SET helper it trusts
  `expected_count` as the universe — D5 is that universe being wrong).
- **prev delivery reassembler** `copy_data_to_buffer` (arq_common.cc, the 40b184f
  :9712-9729 / :9863-9883 legs) — concatenates ACKED slots in `[0, data_batch_size)`,
  silently skips FREE holes. The tail `[expected_count, data_batch_size)` is FREE on the
  lost-EOB path → dropped.
- **SACK-bitmap gate** `arq_responder.cc:1672` (`rx_received < expected`) — D5 suppresses
  the SACK (§2.3).
- **in-place ACK-GATE** `arq_responder.cc:1087` region — same EOB inference for the
  CURRENT batch; the comment AT :1087 already names the hazard: *"if the EOB frame itself
  is lost this branch never runs and Fix A's per-frame rx_timeout is the fallback."*

---

## §4 VALID STATES + the broken invariants

### 4.1 D2/D3 valid states of `last_data_viable_config`
| state | anchor | leap_cap (from anchor) | climb target after clamp | data flows? | anchor ratchets? |
|---|---|---|---|---|---|
| fresh ROBUST | ROBUST_0..2 | anchor+1 (ROBUST/CFG0) | anchor+1 | yes (robust batch) | yes (N_ROBUST=1) |
| OFDM-entry | CONFIG_0 (idx3) | `up_n(CFG0,13)`=**CONFIG_13** | min(CFG16, CFG13)=**CFG13** | first move CFG0→CFG13 ok | yes IF a CFG13 batch DELIVERS |
| **WEDGE** | **CONFIG_0**, current=**CONFIG_13** | **CONFIG_13** == current | clamp-to-self → **no-op SET_CONFIG spin** | **NO (control starves data)** | **NO (P-A never fires)** → leap_cap frozen at CFG13 |
| post-ratchet | CONFIG_13 (idx16) | `up_n(CFG13,13)`=**CONFIG_16** | CFG16 | yes | n/a (top) |

**INV-ASSUMED (D2):** the leap_cap bound is TEMPORARY — it lifts as the anchor ratchets
up. **INV-VIOLATED:** the ratchet (P-A) requires a clean DATA batch delivery, but the
clamp-to-self emits CONTROL not DATA, so the anchor cannot ratchet → the "temporary" bound
is PERMANENT. The two pieces of state (anchor, clamp output) are circularly dependent and
both stall at the OFDM-entry rung. This is the D2+D3 coupling.

### 4.2 D5 valid states of `expected_count` vs the true batch length
| state | TX frames | EOB frame | `last_received_end_of_batch_seq` | `expected_count` | gate | faithful? |
|---|---|---|---|---|---|---|
| full batch, EOB rx | N | received | N-1 | N | fires on `[0,N)` complete | ✓ |
| short batch (legit) | M<dbs | received (seq M-1) | M-1 | M | fires on `[0,M)` | ✓ (TX really sent M) |
| **EOB-lost tail** | N | **LOST** | < N-1 (highest other seq) | **< N** | fires on `[0,expected)` complete; tail `[expected,N)` FREE+dropped | **✗ SILENT SKIP** |

**INV-ASSUMED (D5, by the gate + reassembler + SACK + D3.1):** `expected_count` equals the
TX's true per-batch frame count. **INV-VIOLATED:** on the EOB-loss path `expected_count` is
the highest-seq-RECEIVED+1, strictly less than the true count; every consumer trusts it, so
the missing tail is declared "not part of the batch" and silently dropped — AND the SACK
gate (§2.3) cannot even ask for it. **The producer is wrong on the uncommon EOB-loss path;
all consumers are reasonable but blind** — the canonical CLAUDE.md §5 failure mode.

### 4.3 The SACK-recovery dead end (why D5 cannot self-heal)
A normal mid-batch hole (frame j < highest-seq lost) IS SACK-recoverable: `expected`
still spans it (a HIGHER seq was received, including the EOB), `rx_received < expected`,
the bitmap has a 0 at j, CMD retransmits j. The **EOB-frame loss is the unique unhealable
case**: losing the HIGHEST frame collapses `expected` to exclude it, so it is outside the
bitmap span and outside the gate. The single-frame-of-evidence length channel (§1.3) has no
redundancy for its own terminator.

---

## §5 THE DESIGN — three coupled fixes (root-cause, gated, byte-identical off-path)

### §5.1 D2 — let the forward target reach 14/15/16, do NOT permanently re-clamp to 13

**Root cause:** `RETRIGGER_MAX_LEAP=13` makes the FIRST leap off a CONFIG_0 anchor land at
CONFIG_13, and the anchor cannot ratchet past CONFIG_0 (D3) so the bound never lifts.

**The fix is NOT to raise RETRIGGER_MAX_LEAP** (that would re-open the §15 WGN:-10
over-climb the cap exists to bound, and the cap is anchor-gated defense-in-depth — see
arq.h:1056-1063). The MAX_LEAP cap is CORRECT in isolation; the defect is that the anchor
it leaps FROM is frozen (D3). **D2's real fix is D3** — once the anchor ratchets
CONFIG_0 → CONFIG_13, `leap_cap = up_n(CONFIG_13, 13) = CONFIG_16` and the SAME clamp
licenses CFG16 (verified: `_cfg16acq2/ROOTCAUSE_AND_FIX.md` §4b(a), the existing
`--test-climb-engine` Part NM6 asserts this exact site). So **D2 is a CONSEQUENCE of D3,
not an independent code change.** The design records this explicitly so a future agent does
not "fix D2" by bumping MAX_LEAP (the symptom-masking anti-pattern, CLAUDE.md §2).

**Decision:** D2 requires NO new clamp logic. The MAX_LEAP stays 13. The fix that makes
the clamp transparent is D3 (the anchor ratchet). The ONLY D2-local guard is the existing
`is_ofdm_config(anchor)` discipline (arq.h:1051-1053) — untouched, it correctly keeps the
WGN:-10 ROBUST-anchor inert. **Byte-identical: D2 ships zero new code.**

### §5.2 D3 — yield to DATA on a clamp-to-self so a CFG13+ batch delivers and ratchets

**Root cause:** when the fully-clamped turbo target lands AT-OR-BELOW the current config
there is no forward config change to announce, yet the turbo UNCONDITIONALLY emits
`add_message_control(SET_CONFIG)` (monitor arq_commander.cc, the
`turbo_supershift_announce_pending = true; add_message_control(SET_CONFIG);
this->connection_status=TRANSMITTING_CONTROL;` tail of the in-turbo SUPERSHIFT branch
~:5413+) — a no-op SET_CONFIG-to-self the peer ACKs, turbo re-triggers, the SAME
clamp-to-self fires, an infinite CONTROL spin that starves DATA.

**The fix (pure predicate + caller yield):** add a PURE helper

```
bool supershift_clamp_yields_to_data(int clamped_target, int current_config) const
  → config_ladder_index(clamped_target) <= config_ladder_index(current_config);
```

At the in-turbo SUPERSHIFT site, AFTER all caps (leap_cap + ceiling + cooldown +
WB/NB) are applied to `negotiated_configuration` and BEFORE the
`add_message_control(SET_CONFIG)` emit, branch on the helper: when TRUE, SETTLE the turbo
at the current rung and transition straight to DATA (mirror the CFG16-HOLD top-config
terminal state: `turboshift_active=false; turbo_supershift_announce_pending=false;
turboshift_phase=TURBO_DONE; turboshift_last_good=current; data_configuration=current;
negotiated_configuration=current; reverse_configuration=current;
connection_status=TRANSMITTING_DATA; return;`) **WITHOUT pinning
`supershift_proven_ceiling`** (the clamp is a TEMPORARY leap_cap bound that lifts once the
anchor ratchets — pinning the ceiling would forbid the later climb to CFG16; this is the
critical difference from the CFG16-HOLD branch, which legitimately pins because CFG16 is
the top).

**Why this ratchets the anchor:** with DATA flowing at CONFIG_13, a clean CONFIG_13 batch
DELIVERS → P-A (`data_anchor_raise_target`, arq_commander.cc:4266) fires →
`last_data_viable_config` ratchets CONFIG_0 → CONFIG_13 → the next SUPERSHIFT re-trigger
has `leap_cap = up_n(CONFIG_13, 13) = CONFIG_16` → the climb reaches CFG16, where the H1
reverse-ACK lever holds it (`feat/revack-geom`, already merged-to-monitor per memory).

**Byte-identical on a real upshift:** when the clamped target is STRICTLY ABOVE current (a
genuine config change — the normal rung-by-rung climb, and the SNR-capped-step-1 path
`negotiated = current+1`), the helper returns FALSE and the SET_CONFIG path is unchanged.
The branch fires ONLY on the genuine no-op clamp-to-self that is today a control spin.

**This design is ALREADY IMPLEMENTED + unit-proven** in `feat/cfg16-acq-d2d3 @ 05fb9b7`
(worktree `cfg16-acq-d2d3`): `supershift_clamp_yields_to_data` (arq.h there :1174-1184),
the yield branch (arq_commander.cc there :5470-5490), failing-test-first Part D2D3 in
`--test-climb-engine` (FAILBEFORE macro `CFG16ACQ2_D2D3_FAILBEFORE` neuters the helper →
D2D3-1 FAILS; PASS-AFTER all green), and the A/B (`CFG16_ACQ_D2D3_AB.md`: BASE wedges at
CONFIG_13 113 B, FIX reaches+holds CONFIG_16 42008 B). **Track-C's D3 deliverable is to
REBASE this implemented fix onto monitor `627c370` and re-run the failing-test-first +
A/B against a track-named binary.**

### §5.3 D5 — carry the TX-authoritative per-batch frame count on the WIRE

**Root cause (§4.2):** `expected_count` (and the SACK span, and the in-place
`effective_batch`) are INFERRED from `last_received_end_of_batch_seq`, a single-frame-of-
evidence channel whose terminator (the EOB frame) is exactly the frame whose loss erases
the length. The producer (`prev_expected` derivation, arq_common.cc:6382-6389) is wrong on
the EOB-loss path; every consumer trusts it; the SACK gate cannot even request the lost
tail (§4.3).

**Direction chosen: DIRECTION 1 (carry the count, do not infer it)** — the root-cause fix
from PREV_BUMP_VERDICT.md §4. Directions 2 (loud-abort every EOB-short bump) and 3 (hold
the bump until EOB confirmed) are REJECTED as the primary fix: 2 is blunt (aborts every
legitimately-short final batch and every transient EOB loss → a reliability/throughput
regression on a channel where partial loss is the NORM the SACK system exists to absorb);
3 changes the bump trigger timing and interacts with the already-strained half-duplex
turnaround budget. 2 is retained as the SAFETY NET for the residual case (§5.3.4).

**The wire-format change (the load-bearing decision — a real cross-layer change, NOT a
patch):** carry a per-batch frame count `batch_total_frames` (1..MAX_SACK_BATCH_SIZE=32,
6 bits) on EVERY DATA frame in a batch (so it survives loss of any single frame including
the EOB frame), under the EXISTING `sack_v2_enabled` capability gate (no new CLI flag; the
behaviour rides the already-negotiated SACK-v2 capability, byte-identical for v1/legacy/NB
peers).

Two sub-options for WHERE the 6 bits ride (to be settled with the user at the architecture
review — both are designed; option (a) is preferred):

- **(a) Reuse the EOB bit-7 semantics into a `batch_total_frames` field on a NEW v2
  header byte.** The SACK-v2 DATA_LONG/DATA_SHORT headers already grew by 1 byte for
  `batch_seq_id` (arq.h:264-271, `effective_data_*_header_length`). Grow the v2 header by
  ONE MORE byte carrying `batch_total_frames` (a full byte: 1..32 fits, 0 reserved =
  "unknown/legacy", future-proof to 128). On EVERY frame of the batch the CMD writes the
  same constant `message_batch_counter_tx`. The RX, on ANY received frame of the batch,
  learns the authoritative count. **The EOB bit-7 stays** as the "this is the last frame"
  fast-turnaround trigger (arq_responder.cc:1087 fast-SACK path) — it is now an
  OPTIMISATION, not the sole length authority. Cost: +1 byte/frame on WB-OFDM only (at
  CFG16 ~155 B/frame payload, +1 byte ≈ 0.6% — negligible; CFG16 is throughput-rich).
- **(b) Pack the 6-bit count into spare bits of an existing field.** `batch_seq_id` is a
  full 8-bit mod-256 wrap — NO spare bits. The seq field is 7-bit slot + bit-7 EOB — the
  slot needs all 7 bits for batch ≤ 128. So there is NO free field; (b) would require
  shrinking the bsi mod-space or the slot range — REJECTED as it re-opens aliasing
  (the `static_assert(MAX_SACK_BATCH_SIZE <= 128)` at arq.h:289 and the bsi mod-256
  wrap). **→ option (a) is the only clean realization.**

**The consumer changes (set `expected_count`/`expected`/`effective_batch` from the WIRE):**
1. **RX frame parse** (`arq_common.cc:8966-8974` region): when a DATA frame of the current
   bsi is stored, latch `rx_batch_total_frames = wire byte` (a new member, -1 = unknown).
   Keep `last_received_end_of_batch_seq` for the v1/legacy fallback ONLY.
2. **prev `expected_count`** (`arq_common.cc:6382-6389`): when `rx_batch_total_frames > 0`
   use it as `prev_expected` (clamped to `[1, data_batch_size]`), NOT the EOB inference.
   Fallback to the existing inference ONLY when the count is unknown (v1/legacy/no frame of
   the batch yet seen — but at least one frame is always seen before a bump, so v2 always
   has it).
3. **SACK-bitmap `expected`** (`arq_responder.cc:1639`): same — span the bitmap over the
   AUTHORITATIVE count. Now the lost EOB tail is INSIDE the bitmap → `rx_received <
   expected` is TRUE → the RX SACKs the missing tail → CMD retransmits it → faithful.
4. **in-place ACK-GATE `effective_batch`** (`arq_responder.cc:1087` region): same — the
   current-batch ACK gate spans the true count, so it waits for / SACKs the tail instead
   of ACKing early.

**The integrity composition (the WHOLE point):** with the count wired, a lost EOB frame
leaves `received_count < expected_count` → the L1 SET-gate `prev_batch_is_frame_complete()`
(designed in `data-flow-prev-bump.md` §5.2, **also not on monitor**) HOLDS the prev →
either (i) the SACK retransmit fills the held tail → faithful, OR (ii) the prev is
stale-discarded undelivered → the high-water stays un-advanced → the next current commit
trips the batch-level `delivery_step_is_gap` (D3.1) → LOUD `[RSP-V2-GAP-ABORT]`. **Either
faithful-after-retx OR loud-abort — never a silent skip.** D5 is the producer fix that
makes the L1 SET-gate's universe (`expected_count`) trustworthy; L1 without D5 still
trusts a corrupted universe (PREV_BUMP_VERDICT.md §3).

**§5.3.4 Safety net (Direction 2, retained):** in the ABSENCE of the wired count (a v2 peer
that somehow has `rx_batch_total_frames` unknown at bump time — should be impossible since
≥1 frame is always seen, but defense-in-depth), route an EOB-short prev-bump (bump fired
because a NEWER bsi arrived, NOT because the EOB was seen) through
`rsp_gap_abort_teardown()` rather than delivering at the inferred short length. This
converts the residual unknown-length case from silent-skip to loud-abort.

**Byte-identical guarantee:** `sack_v2_enabled == false` → no new header byte emitted, the
EOB inference is unchanged → byte-identical for ALL v1/legacy/NB sessions. For v2, when no
EOB frame is lost, `rx_batch_total_frames == last_received_end_of_batch_seq + 1` → the
derived `expected_count` is IDENTICAL → byte-identical on the faithful path. The change
BITES only on the EOB-loss path, exactly where today's behaviour is silent-wrong-bytes.

---

## §6 MERGE ORDER (why all three land together; CLAUDE.md robustness-over-speed)

- **D2 ships no code** (it is D3's consequence; §5.1).
- **D3 alone** (the clamp-to-self yield) UNLOCKS reaching CFG16. Landing it WITHOUT D5 puts
  the link into the prev-bump-under-climb regime where D5 silently drops frames
  (PREV_BUMP_VERDICT.md §5.4). **D3 must NOT land without D5** (do not ship a config that
  reaches CFG16 and silently mis-delivers).
- **D5 alone** (wire the count + SACK-span fix) is a STRICT integrity improvement and is
  SAFE to land independently — it only converts silent-skip → faithful/loud-abort on the
  EOB-loss path, and is byte-identical otherwise. It is INERT until the climb reaches the
  prev-bump-under-CFG16 regime, but harmless before then.
- **The D1 acq-fix** (`climb_effective_snr_margin_db`, `_cfg16acq2/ROOTCAUSE_AND_FIX.md`)
  is the prerequisite that lets D3 even compute a CFG16 target. It is a standalone
  byte-identical-off-SACK control-plane fix (already designed + unit-proven) and should
  land first; D3 sits on it.
- **The L1/L2 prev-gate fixes** (`data-flow-prev-bump.md` §5.2) are the SET-gate D5
  depends on (D5 makes the gate's universe correct; L1 makes the gate a SET not a count).
  L1/L2 land WITH D5.

**RECOMMENDED MERGE ORDER:** (1) D1 acq-fix (standalone, byte-identical-off-SACK). (2) D5
wire-the-count + L1/L2 SET-gate (standalone integrity gain, byte-identical-off-path). (3)
D3 clamp-to-self yield (the gear that reaches CFG16) — LANDS LAST, GATED behind D5 being in
tree. This ordering never has a tree state that reaches CFG16 and silently drops frames.

---

## §7 FAILING-TEST-FIRST plan (CLAUDE.md §3 — designed, to be implemented per fix)

All in-process `--test-<name>` entry points (no IONOS, no RF), each fails-before via a
`-D…_FAILBEFORE` macro or a `MERCURY_…_DEFEAT` env on the SAME binary (the codebase's
established pattern), passes-after.

- **D3** `--test-climb-engine` Part D2D3 (ALREADY EXISTS in the d2d3 tree): D2D3-0a/0b
  (byte-identical on a real upshift: clamped>current → no yield), D2D3-1 (clamp-to-self
  CONFIG_13 → YIELD; FAILBEFORE `CFG16ACQ2_D2D3_FAILBEFORE` → D2D3-1 FAILS), D2D3-2a/2b
  (the ratchet: after yield a CFG13 delivery raises the anchor → next leap_cap=CFG16),
  D2D3-3a/3b (no over-climb / no ceiling-pin regression). Re-run on the monitor rebase.
- **D5** a NEW `--test-eob-loss-batch-truncation`: TX a 30-frame batch, DROP the EOB
  (frame 29), bump-and-transfer. FAIL-BEFORE (`MERCURY_D5_INFER_DEFEAT=1` keeping the EOB
  inference): `expected_count=highest-seq+1 < 30`, the prev delivers a 29-frame batch
  (silent skip), md5 ≠ tx_prefix. PASS-AFTER (wired count): `expected_count=30`,
  `received_count=29 < 30` → SET-gate HOLDS → inject the frame-29 retransmit → faithful
  30-frame delivery; OR stale-discard → `[RSP-V2-GAP-ABORT]`. Also assert the SACK-span fix:
  the RX SACKs frame 29 (it is now inside `expected`).
- **D5 byte-identical guard**: a no-loss 30-frame v2 batch delivers byte-identical to today
  (wired count == EOB inference); a v1/legacy batch is byte-for-byte unchanged.
- **No-regression battery** (per the prev-bump precedent): `--test-prevbump-frame-hole`,
  `--test-climb-engine`, `--test-inorder-demote`, `--test-gap-abort`,
  `--test-partial-bsi-advance={mfsk,ofdm}`, `--test-batch-shrink-strands-prev`,
  `--test-eob-poison-prev-retx`, all rc=0.

**Sim hot-wash (per the user directive — sim first, bench is final arbiter):** the
2-process climb-sim (`tools/sim_arq_channel.py` + `tools/sim_channel_relay.py`) with
`--turnaround-drift` (ppm±8.16 jit300) + 64KB completion bound, against a track-named
binary copied off `C:/Program Files/Mercury/mercury.exe` IMMEDIATELY after each build
(BUILD-CONTENTION GUARD). The A/B: BASE (D5-infer-defeat) reaches CFG16 and md5_match
FALSE (155-B skip); FIX (wired count) reaches CFG16 and md5_match TRUE — the
integrity-sound self-climb the bench-9 ARM E assert needs.

---

## §8 Cross-layer audit checklist (future changes)

Before changing ANY of: `last_data_viable_config` (arq.h member; producer
arq_commander.cc:4266; D2 consumer arq.h:1034/:1063); `supershift_retrigger_target`
(arq.h:1028); the turbo SUPERSHIFT emit (arq_commander.cc:5413+); `bump_bsi_and_transfer_
prev` `prev_expected` (arq_common.cc:6382); the EOB-mark TX (arq_common.cc:5610); the EOB
decode RX (arq_common.cc:8966); the SACK-span `expected` (arq_responder.cc:1639); the
in-place `effective_batch` (arq_responder.cc:1087) — walk §2/§3/§4/§5 here AND update
`data-flow-prev-bump.md` + `gearshift-climb-engine.md` (§15/§16) + `data-flow-batch-size.md`.

**Special vigilance:** (i) NEVER "fix D2" by raising `RETRIGGER_MAX_LEAP` — it re-opens the
§15 WGN:-10 over-climb; D3 (anchor ratchet) is the fix. (ii) Any change that lets
`expected_count`/`expected`/`effective_batch` derive from `last_received_end_of_batch_seq`
re-opens D5 unless the wired count (§5.3) is the primary source. (iii) The wire-format byte
(§5.3a) MUST stay behind `sack_v2_enabled` for reversibility (the SACK_DESIGN_A §6
irreversibility mitigation).

---

## §9 Related documents
- `fact-documents/data-flow-prev-bump.md` — owns the prev cross-storage in-order invariant
  (L1/L2 + the D5 §5.6/§8 audit; updated in lockstep with this doc).
- `fact-documents/gearshift-climb-engine.md` §15/§16 — owns the anchor-tier-crossing
  discipline (the D2 `supershift_retrigger_target` clamp + the §16 anchor producer this
  audit's D3 ratchet feeds).
- `fact-documents/data-flow-messages_rx_prev.md` — per-slot storage lifecycle (the R7 loc
  bound; L1 is its delivery-time analogue).
- `fact-documents/data-flow-arq-recovery-cluster.md` — R035 reshrink / R038 EOB-poison.
- `bigblock_p3_hw/_cfg16acq2/ROOTCAUSE_AND_FIX.md` — the D1 acq-fix root cause (the
  prerequisite that surfaces D2/D3).
- `bigblock_p3_hw/_cfg16acqd2d3/CFG16_ACQ_D2D3_AB.md` — the IMPLEMENTED D3 fix's A/B
  (BASE wedge vs FIX reach16) — Track-C rebases this onto monitor.
- `bigblock_p3_hw/_prevbump/PREV_BUMP_VERDICT.md` — the executed D5 byte-attribution
  (155-B skip, md5 FALSE) that this design closes.
