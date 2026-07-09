# Data-Flow Audit: `data_batch_size`

**Status**: Authoritative as of 2026-05-30 on `fix/climb-engine` (after the
4th-wire-failure root fix, paired with this document and with the upgraded
`--test-climb-engine` connect-path assertions). Every change to `data_batch_size`
or to any predicate that gates a batch-size write MUST update this document.

**Context**: `data_batch_size` is the number of DATA frames the ARQ packs into
one radio TX (one "batch"). It is shared state across PHY ↔ ARQ ↔ SACK: the
commander builds/ACK-gates a batch of this size, the responder forms its RX
expectation from it, and BOTH sides independently compute the clean-batch
"all-ones" SACK target as `(1<<data_batch_size)-1`. If CMD and RSP ever hold
DIFFERENT values, the all-ones targets never match, no clean ACK is credited,
and (for the gearshift) the climb cannot start or advance.

This is the canonical owner of producer/consumer/invariant facts for
`data_batch_size`, per CLAUDE.md §"Cross-Layer Data-Flow Audits". It exists
because the climb-fix family failed on the IONOS wire **four** times in a row,
all the same class (CMD/RSP robust-batch mismatch), a *different producer each
time*. CLAUDE.md §5 mandates auditing the shared state instead of patching the
next individual path.

Declaration: `include/datalink_layer/arq.h` — `int data_batch_size;` member of
`cl_arq_controller`. Setter: `cl_arq_controller::set_data_batch_size(int)`
(`arq_common.cc:561`). Ctor default = 1 (`arq_common.cc:146`).

---

## §1 The invariant

> **At any robust/MFSK config (`is_robust_config(cfg)`, cfg ∈ {100,101,102}),
> CMD `data_batch_size` == RSP `data_batch_size` == 1.**

Definition: `is_robust_config(int config){ return config>=100 && config<=102; }`
(`include/common/common_defines.h:78`). `ROBUST_0=100` (`:74`), `CONFIG_0=0`
(`:54`).

Why batch MUST be 1 at robust (not merely "is" 1): robust modes use an
all-or-nothing pattern ACK. The clean-batch target is `(1<<data_batch_size)-1`
computed independently on each side. At batch=1 the target is `0x1` on both
sides and one delivered MFSK frame == a clean all-ones batch (this is STRICTER
than batch≥5, not looser — a single frame proves the rung). At batch=5 the CMD
target is `0x1F`; if the RSP held batch=1 it emits bitmap `0x1` → CMD never sees
a match → no clean credit → at the MFSK floor SNR the climb gets zero clean
batches and LINK-TIMEOUTs.

OFDM configs (CONFIG_0..16) are exempt: there both sides run the identical 30s
batch-scaling formula and SACK patches partial batches, so batch≥5 is correct
and symmetric.

---

## §2 Producers — every code path that writes `data_batch_size`

All production writes go through `set_data_batch_size()` EXCEPT the four
test-only direct-assigns in §2.7. The setter is now the single chokepoint (§5).

### 2.1 `load_configuration()` — the config-change pin (`arq_common.cc`)
- `:1223` (FULL only): `set_data_batch_size(default_configuration_ARQ.batch_size)`.
- `:1234` (robust branch, any level): `set_data_batch_size(1)`.
- `:1283` (OFDM batch-scaling, `!is_robust_config(configuration)` gated):
  `set_data_batch_size(fixed_batch)` where `fixed_batch = sack_enabled ?
  radio_batch_size(25) : 10`, clamped to a 30 000 ms / `message_transmission_time_ms`
  ceiling.

Reads `configuration` (the argument), and the **member `current_configuration`
is set to `configuration` at `:1168`, BEFORE this batch block** — so by the time
any `set_data_batch_size` runs inside `load_configuration`, the member already
reflects the new config. Robust branch (`:1232 if(is_robust_config(configuration))`)
pins 1; OFDM branch (`:1261 if(!is_robust_config(configuration) && …)`) scales.

### 2.2 CMD SACK recompute at TEST_CONNECTION_ACK (`arq_commander.cc:3883`)
Inside `process_control_commander()` (`:3671`), in the
`TEST_CONNECTION / TEST_CONNECTION_ACK` branch (`:3705-3707`), after SACK
negotiates (`sack_enabled` is default-on). Runs ONCE at connect.
- **Predicate (THIS FIX)**: `if(!is_robust_config(current_configuration))` →
  `set_data_batch_size(radio_batch_size)`, floor 5.
- **Predicate (PRE-FIX, the 4th-failure bug)**:
  `if(!is_robust_config(negotiated_configuration))` — see §4.

### 2.3 RSP SACK recompute at TEST_CONNECTION (`arq_responder.cc:2163`)
Parallel to §2.2, in the RSP TEST_CONNECTION handler. Runs ONCE at connect.
- **Predicate**: `if(!is_robust_config(current_configuration))` →
  `set_data_batch_size(radio_batch_size)`, floor 5. (Already correct pre-fix —
  it was always the CMD side that diverged.)

### 2.4 `policy_evaluate_axis2()` — Axis-2 batch controller (`arq_commander.cc:5316`)
Mid-session adaptive batch sizing for OFDM. After `AXIS2_UP_GOOD_RUN=4`
consecutive "good" batches it steps `data_batch_size += AXIS2_STEP(5)`
(clamped `[AXIS2_BATCH_FLOOR=10, AXIS2_BATCH_CEIL=32]`) and sends
SET_LINK_PARAMS to the RSP (§2.6).
- **Robust guard (86d39b4, KEEP)**: `if(is_robust_config(current_configuration))
  return;` at the top (`:5156`). Skips the whole controller at robust, so it
  never moves robust batch off 1. This is one of the two halves of the invariant
  (the other was the §2.2 predicate). Reads `current_configuration`.

### 2.5 CMD Axis-2 test fire (`arq_commander.cc:5316` via `test_fire_policy_axis2`)
`test_fire_policy_axis2()` (`:5400`) / `test_fire_policy_axis2_ceiling()` (`:5473`)
prime length members then call `policy_evaluate_axis2`. Same guard as §2.4.

### 2.6 RSP SET_LINK_PARAMS apply (`arq_responder.cc:2679`)
`process_messages_rx_acks_*` SET_LINK_PARAMS handler. On CRC-pass, clamps CMD's
requested batch to `[AXIS2_BATCH_FLOOR=10, AXIS2_BATCH_CEIL=32]` then
`set_data_batch_size(target)`. Driven ONLY by CMD Axis-2 (§2.4). Because §2.4
never fires at robust, the RSP never receives a robust SET_LINK_PARAMS — but
even if a corrupt frame proposed one, the §5 chokepoint backstops it.

### 2.7 Test-only DIRECT assigns (BYPASS the setter — not affected by the chokepoint)
- `arq_responder.cc:2895`: `this->data_batch_size = 25;` (SACK-v2 mixbatch test).
- `arq_responder.cc:3000`: `this->data_batch_size = 30;` (synthetic SET_LINK_PARAMS).
These deliberately bypass `set_data_batch_size()`'s `max_*_length` clamp in
SACK regression tests (OFDM-batch scenarios, `current_configuration` not robust).
They write the member directly, so the §5 chokepoint does not touch them. No
production path uses direct assignment.

---

## §3 Consumers — every code path that reads `data_batch_size`

### 3.1 Clean-batch "all-ones" SACK target (THE divergence-sensitive consumer)
- CMD `sack_clean_confirmation_accepted()` / data-ACK arm:
  `arq_commander.cc:131-133` and `:2518-2520`:
  `all_ones = (data_batch_size>=32) ? 0xFFFFFFFF : (1u<<data_batch_size)-1u;`
  then `is_clean_confirmation = (rx_bitmap == all_ones)`.
- RSP clean-ACK emit: `arq_responder.cc:801` and `:1711`:
  `bitmap_u32 = (1u<<data_batch_size)-1u;`
**If CMD batch≠RSP batch, CMD's `all_ones` ≠ the bitmap the RSP emits → no clean
confirmation → no promotion credit.** This is the mechanism of all four wire
failures.

> **★ 2026-07-03 — a FIFTH, WORSE consequence of the same desync: SILENT DATA
> CORRUPTION (not just a stall).** When the mismatch is CMD>RSP (CMD=30, RSP=25 — CMD
> Axis-2 stepped up but the RSP never durably APPLIED it; see §2.4/§2.6 + the unreliable
> SET_LINK_PARAMS round-trip), the RSP ACK-GATE (§3.3) does NOT stall — it DELIVERS the
> batch TRUNCATED to its own smaller size and treats it complete, ORPHANING the sender's
> surplus frames (their source bytes are delivered by NEITHER batch) → a permanent
> one-batch stream SHIFT → the whole tail is silently wrong. Captured as `res_c3100`
> (WGN:25, ~3%); full root-cause + fix design (confirmed-before-use step-up + an RSP
> loud desync-abort) in `fact-documents/silent-corruption-residual.md`. The §1 invariant
> is even more load-bearing than the stall framing implied: a CMD>RSP mismatch is a
> life-critical integrity fault, so the step-UP handshake must be RSP-confirmed-applied
> before the CMD builds at the new size (the round-trip ACK at `arq_commander.cc:7333`
> does NOT currently guarantee this).

### 3.2 CMD block-build / TX
`process_messages_tx_data()` (`arq_commander.cc:1138`) and the data-frame
producer iterate `i<data_batch_size` (e.g. `arq_common.cc:6654`, `:6805`).

### 3.3 RSP RX expectation / ACK-GATE
ACK-GATE expected-count and the `data_batch_size>1` partial branch
(`arq_responder.cc:~1373/1380/1512`). At robust batch=1 the partial branch is
never taken → the clean PASS funnel always runs.

### 3.4 `messages_rx_prev` completion + bsi transfer
`bump_bsi_and_transfer_prev()` (`arq_common.cc:4086`, `:4106`, `:4128`) bounds
loops by `data_batch_size`. The prev path is dead at batch=1 (single-frame
batches never use prev storage) — see `data-flow-messages_rx_prev.md`.

### 3.5 ACK-timeout math
`set_ack_timeout_data()` / `recalculate_ack_timeout_for_batch()` scale the
data-ACK timeout by `data_batch_size` (`arq_common.cc:657/674/678/1318/1359`).

### 3.6 Optimizer / nominal tracking
`nominal_batch_size` is set alongside `data_batch_size` at §2.2/§2.3/§2.4 sites.
The effective-rate optimizer reads neither directly for robust decisions
(robust pins config explicitly).

---

## §4 The connect-path config-variable values (why the 4th failure happened)

Three config members govern batch-gating predicates. Their REAL values at
**CMD TEST_CONNECTION_ACK** on a fresh unpinned headless `-g -R` connect
(`main.cc:2235` `mod_config=ROBUST_0`, `:2237` `robust_mode=1`, `:2320`
`ARQ.init(...,ROBUST_0)`):

| Member | Value at CMD TEST_CONNECTION_ACK | Why |
|---|---|---|
| `current_configuration` | **ROBUST_0 (100)** | `init()` calls `load_configuration(data_configuration=ROBUST_0)` (`arq_common.cc:860`), and every control-frame TX reloads `ack_configuration=ROBUST_0` (`arq_commander.cc:1023/1752`). `is_robust_config`=**true**. |
| `negotiated_configuration` | **CONFIG_0 (0) — ctor default** | Initialized `CONFIG_0` (`arq_common.cc:281`). `reset_session_state()` writes it `=init_configuration` only on the teardown branch (`arq_commander.cc:443`); the other writes are BREAK-recovery (`:205`,`:297`), OFDM-optimizer SET_CONFIG (`:530`), turboshift completion (`finish_turbo_direction` `:3623`), and post-connect gearshift (`:43xx/47xx`). **NONE run on the connect path before TEST_CONNECTION_ACK.** `is_robust_config`=**false**. |
| `init_configuration` | ROBUST_0 (100) | `arq_common.cc:842` from `initial_mode`. (Not used by the recompute predicate; listed for completeness.) |

So the PRE-FIX CMD gate `!is_robust_config(negotiated_configuration)` =
`!is_robust_config(CONFIG_0)` = **true** → recompute RAN → CMD batch=5. The RSP
gate `!is_robust_config(current_configuration)` = `!is_robust_config(ROBUST_0)`
= **false** → recompute SKIPPED → RSP batch stayed 1. **Divergence: CMD=5,
RSP=1** → §3.1 all-ones mismatch → first ROBUST_0 block fails → LINK-TIMEOUT →
climb never starts.

The pinned-robust case (`-s 100`, no `-g`) and the OFDM case (`-s 10`) did NOT
expose this because the local tests and some manual runs set both members
together; the bug is specific to the *unpinned* connect where
`negotiated_configuration` is left at its ctor default.

### §4.1 Correction to `gearshift-climb-engine.md`
That doc's §3 (lines 158-162) and §6.1 (line 239) asserted "CMD gates on
`negotiated_configuration`, RSP on `current_configuration` — equal at
TEST_CONNECTION time, so they agree." **That fact is WRONG for the unpinned
connect path** (the connect path never writes `negotiated_configuration`; it is
ctor-default CONFIG_0 there, not the connect config). The struck-through claim
is corrected in `gearshift-climb-engine.md` §3/§6.1; the fix makes BOTH sides
read `current_configuration`.

---

## §5 The fix — single chokepoint + symmetric predicate

Per CLAUDE.md §5 (enforce the invariant against ALL paths, not the one path):

1. **Chokepoint (structural guarantee)** — `set_data_batch_size()`
   (`arq_common.cc:561`): if `is_robust_config(current_configuration) &&
   requested != 1`, force `data_batch_size = 1` (log on a real clamp) and
   return. This is the SOLE setter for every production write (§2.1–§2.6), so
   NO current or future producer can put a robust config at batch≠1. Reads
   `current_configuration` — the live-PHY var, valid in every production path
   (set in `load_configuration` before its own batch sizing; the Axis-2 guard
   and the RSP recompute already key off it). Test direct-assigns (§2.7) bypass
   it intentionally.

   > **UPDATE — FIX-A (branch `fix/robust-dwell-batch`, 2026-06-03)**: the robust
   > chokepoint is now a RANGE clamp `[1..ROBUST_DWELL_BATCH_MAX(8)]`, NOT
   > force-to-1. The invariant is RELAXED from "robust ⇒ batch always 1" to
   > "robust ⇒ batch always within `[1..ROBUST_DWELL_BATCH_MAX]`, and CMD batch ==
   > RSP batch". batch=1 remains the DEFAULT (load_configuration seeds it; the
   > connect-path SACK recompute leaves it; only a PROVEN+PARKED robust dwell —
   > `robust_dwell_batch_eligible()`, CMD-decided — ever requests >1, mirrored to
   > the RSP via the DEDICATED `ROBUST_DWELL_BATCH_OP` (0x44), NOT SET_LINK_PARAMS
   > whose `[10,32]` clamp would break symmetry). The §1 CMD==RSP-symmetry
   > invariant is UNCHANGED (the load-bearing one). See
   > `data-flow-robust-tier-arq-batch.md` §10 for the full delta.

2. **Symmetric predicate (readability + correct connect-path behavior)** —
   change the CMD recompute (`arq_commander.cc:3883`, `:3897`) from
   `negotiated_configuration` → `current_configuration`. Now the CMD recompute,
   the RSP recompute (`arq_responder.cc:2155`), and the Axis-2 guard
   (`arq_commander.cc:5156`) read the IDENTICAL variable with the IDENTICAL
   predicate. CMD and RSP are textually the same code → cannot diverge.

3. **Keep** the 86d39b4 Axis-2 robust guard (§2.4) and the bug-1 split-dedupe /
   in-window clean-confirm (`sack_clean_confirmation_accepted`) — both GOOD,
   unchanged.

### §5.1 Why the chokepoint is safe at every transition
- **OFDM connect** (`-s 10`): `current_configuration=CONFIG_10` at recompute →
  not robust → batch scales to ≥5. ✓
- **robust→OFDM promotion** (ROBUST_2→CONFIG_0): `load_configuration(CONFIG_0)`
  sets `current_configuration=CONFIG_0` at `:1168` BEFORE the `:1283` scaling →
  not robust → batch≥5 allowed. ✓
- **OFDM→robust BREAK** (→ROBUST_0): `load_configuration(ROBUST_0)` sets
  `current_configuration=ROBUST_0` at `:1168` BEFORE `:1234` → batch pinned 1. ✓
- **BREAK recovery PHYS_ONLY** (`arq_commander.cc:202/294`,
  `load_configuration(robust_0, PHYSICAL_LAYER_ONLY)`): `:1232` robust branch is
  NOT level-gated → batch=1 even on PHYS_ONLY. ✓
- **Axis-2 / SET_LINK_PARAMS** mid-session: `current_configuration` always valid;
  robust → chokepoint clamps any stray request to 1 (and §2.4 already returns
  early). ✓

---

## §6 Paired regression test (CLAUDE.md §"Cross-layer regression tests")

`--test-climb-engine` (`test_climb_engine`, `arq_commander.cc`). The pre-existing
Parts A/B/C are KEPT. **NEW Part D models the REAL connect path** and is the
assertion the four wire failures slipped past:

- **D (connect-path CMD/RSP batch symmetry)**: set `current_configuration =
  ROBUST_0` (the live PHY at connect) and `negotiated_configuration = CONFIG_0`
  (its CTOR DEFAULT — exactly the unpinned-connect state the prior tests never
  modeled), then drive the ACTUAL CMD TEST_CONNECTION_ACK recompute logic and
  the RSP recompute logic. Assert CMD batch == RSP batch == 1 at the ROBUST_0
  connect, and that an OFDM connect (`current_configuration=CONFIG_10`) still
  scales both sides to ≥5.
  - **Fail-before** (86d39b4, `negotiated_configuration` predicate): the CMD
    side reads `is_robust_config(CONFIG_0)=false` → recompute runs → CMD batch=5
    while RSP=1 → assert FAILS.
  - **Pass-after** (`current_configuration` predicate + chokepoint): both read
    `is_robust_config(ROBUST_0)=true` → both stay 1 → assert PASSES.

The multi-rung climb assertion (Part C) and the Axis-2 robust guard assertion
(Part B) remain.

Local tests are NECESSARY but NOT SUFFICIENT — the prior singles passed local
and failed the wire because they hand-set state and never modeled the connect-
path default-init. Part D closes exactly that gap. Remaining hardware-only
assumptions are in `gearshift-climb-engine.md` §8.

---

## §8 Silent-corruption backstop (B) + step-up confirm (A) — audit + fix (2026-07-03)

Root-cause + full narrative in `silent-corruption-residual.md` (capture `res_c3100`,
CMD=30 / RSP=25 at bsi 11). This section is the SHARED-STATE audit for the NEW state the
fix touches: `rx_batch_total_frames` (the SENDER-declared per-batch frame count, D5) as
seen at the RSP ACK-GATE. Per CLAUDE.md §"Cross-Layer Data-Flow Audits":

### §8.1 The sender-declared count `rx_batch_total_frames` — producers
D5 (`TRACK_C_D2D3D5_DESIGN.md`) carries the TX-authoritative batch frame count on EVERY
data frame so a lost EOB frame cannot erase it. This IS the sender's `data_batch_size`
signal available to the RSP at the gate.
1. **TX (CMD)** `arq_common.cc:8892-8898,8916,8940`: `batch_total_frames_wire =
   message_batch_counter_tx` (frames the CMD packed into THIS radio batch = min(CMD
   `data_batch_size`, frames available)) written to DATA_LONG byte[5] / DATA_SHORT byte[6]
   when `header_carries_d5`. **Emits 0 (unknown) on `sack_retransmit_active`** (the retx
   queue is not the original batch) so the RX keeps the count already latched.
2. **RX decode (RSP)** `arq_common.cc:12960-12964` (DATA_LONG), `:13023-13027`
   (DATA_SHORT): `rx_buffer_batch_total_frames = (btf>0)?btf:-1` (staged per-frame).
3. **RX promote (RSP)** `arq_responder.cc:1458-1459`: `if(sack_v2_enabled &&
   rx_buffer_batch_total_frames>0) rx_batch_total_frames = rx_buffer_batch_total_frames;`
   — a NON-ZERO count on ANY surviving frame of the batch promotes the authoritative count.
4. **RX reset (RSP)** `arq_responder.cc:2441` (clean BATCH-DONE), `arq_common.cc:9856,9927`
   (session reset), `rsp_gap_abort_teardown` `arq_common.cc:9927` → `-1` so the next batch
   re-baselines. Robust configs do NOT carry the D5 byte (`arq_commander.cc:14805`,
   `arq_common.cc:2164`) → `rx_batch_total_frames` stays −1 there (the backstop is inert at
   robust, which is correct — robust batch is pinned symmetric by §1/§5).

### §8.2 Consumers of `rx_batch_total_frames` at delivery (the clamp = the truncation)
- **ACK-GATE expected-count** `arq_responder.cc:2111-2115`:
  `expected = rx_batch_total_frames; if(expected > data_batch_size) expected = data_batch_size;`
  **THE clamp is the silent-truncation point.** When the sender declared MORE frames than
  the RSP's `data_batch_size`, the clamp collapses `expected` to the RSP's smaller size, so
  `rx_received (25) >= expected (25)` → ACK-GATE PASS → `copy_data_to_buffer()` delivers 25
  frames and treats the 30-frame batch COMPLETE → orphans frames 25-29 → permanent shift.
- Twin clamps: current-batch gate `arq_responder.cc:1479-1483`, prev-batch
  `arq_common.cc:9791-9796` (both `min(rx_batch_total_frames, data_batch_size)`).

### §8.3 Valid states + the MISMATCH invariant the consumers assume
- **Invariant (the load-bearing one, §1):** CMD `data_batch_size` == RSP `data_batch_size`.
  When it holds, `rx_batch_total_frames <= data_batch_size` ALWAYS (the CMD cannot pack
  more frames than its own batch size, and its size == the RSP's). So
  `rx_batch_total_frames > data_batch_size` is a PRECISE, SUFFICIENT, false-positive-free
  detector of the CMD>RSP desync — the exact `res_c3100` fault. (Compression packs FEWER
  frames ⇒ count ≤ size; retx emits 0 ⇒ keeps latched; never a benign over-count.)
- MISMATCH states: **CMD>RSP** (`res_c3100`: CMD Axis-2 stepped up, RSP config-reset to 25
  and never applied 30) → silent truncation. **CMD<RSP** → RSP over-waits, SACKs the
  shortfall, safe (no corruption). **mid-batch change** → whichever side is larger at the
  gate governs; CMD>RSP is the dangerous one.

### §8.4 FIX (B) — LOUD BACKSTOP (SHIPPED, `arq_responder.cc` ACK-GATE)
New pure helper `batchsize_desync_detected(sender_total_frames, local_batch, sack_v2)` →
true iff `sack_v2 && sender_total_frames>0 && sender_total_frames>local_batch`. The ACK-GATE
(pattern-ACK branch, after `expected` derivation) calls it; on a detected desync (and
`!passive_monitor`, backstop not env-defeated) it prints `[RSP-V2-BATCHSIZE-DESYNC]` and
calls `rsp_gap_abort_teardown()` (link DROPPED, state cleared, no delivery) — the SAME
tested integrity teardown the D3.1 GAP-ABORT uses. Converts the silent stream-shift into a
LOUD, recoverable abort. `MERCURY_BATCHSIZE_DESYNC_DEFEAT=1` disables it (the fail-before
arm, mirrors `MERCURY_GAP_ABORT_DEFEAT` / `MERCURY_D5_INFER_DEFEAT`). Integrity D0: the RSP
NEVER silently delivers a batch truncated below the sender-declared size.

### §8.5 FIX (A) — confirmed-before-use step-up (see silent-corruption-residual.md §7-A)
The CMD must not BUILD at a stepped-UP `data_batch_size` until the RSP has verifiably
applied it (the round-trip at `arq_commander.cc:7069-7080` is unreliable — false-positive
control-ACK match OR lost SET_LINK_PARAMS). Step-DOWN is always safe (no wait). Status
recorded in this doc + silent-corruption-residual.md §10 as SHIPPED / designed-not-merged.

### §8.6 What each fix changes + consumer walk (uncommon paths)
- (B) adds ONE new consumer read of `rx_batch_total_frames` at the gate. It does NOT change
  any producer, so the clamp consumers (§8.2), the SACK bitmap, the climb credit, and every
  matched-batch path are BYTE-IDENTICAL when `rx_batch_total_frames <= data_batch_size`
  (the normal case) — the backstop is INERT unless CMD>RSP. BREAK / config-switch / session
  reset: `rx_batch_total_frames` is reset to −1 on all of them (§8.1.4) so a stale count
  cannot spuriously fire the backstop on the next batch. Robust (batch symmetric, no D5
  byte): inert. Monitor: `!passive_monitor` guard keeps monitors lenient (they legitimately
  accept partials). 1-lost-control-frame case (the capture): the CMD is at 30, RSP at 25 →
  the backstop fires LOUD instead of silent truncation — exactly the intended recovery.

## §7 Related fact documents
- `gearshift-climb-engine.md` — the climb promotion gates (anchor, +1 clamp,
  FRAME-UP/LADDER-UP) and Bugs 1/2/3. §3/§6.1 corrected by §4.1 above.
- `data-flow-messages_rx_prev.md` — the prev-storage state (dead at batch=1).
- `silent-corruption-residual.md` — the `res_c3100` root-cause + (A)/(B)/(C) design + test.
