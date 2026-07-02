# Data-Flow Audit — SUPER-ACK (responder-driven rate-UP skip signal)

Shared-state cross-layer audit for the SUPER-ACK feature (SUPERACK_DESIGN.md).
Built on `wip/superack` off `b8076112` (wip/trio-reverse-pin: reverse-pin +
turbo e5899443 + cheap-miss 2cf0c48d). Every claim cited `file:line`.

The SUPER-ACK is the rate-UP mirror of the already-shipped rate-NACK
(`MFSK_CTRL_NACK`). It reuses the CONFIG_TAG/NACK RM(1,4)+GF(16)-RA+CRC-12
substrate verbatim; NO new FEC, NO new DSP. The RSP maps its LDPC decode margin
(`receive_stats.iterations_done`) to an ABSOLUTE recommended WB target config and
signals it on the reverse robust ACK; the CMD leaps directly to it, bypassing the
dead `is_ofdm_config(current_configuration) && SNR_uplink>-90` elevator gate that
structurally cannot fire from ROBUST (SUPERACK_DESIGN.md §0).

---

## §1. What the change adds (the new shared state + code paths)

### Codec (PHY ctrl layer) — NO shared runtime state
- `MFSK_CTRL_SUPERACK = 6` — `mfsk_ctrl_codec.h:59` (was "codes 6,7 free").
- `pack_superack_payload` / `unpack_superack_payload` — `mfsk_ctrl_codec.cc`
  (37-bit payload: `skip_target_cfg:5 | ack_bsi_lsb:3 | skip_confidence:3 |
  epoch_parity:1 | reserved:25`), cloned bit-for-bit from `pack_nack_payload`.
- `superack_wrap_decode` — `mfsk_ctrl_codec.cc`, cloned from `nack_wrap_decode`:
  FWHT peak gate + GF(16)+CRC-12 type-6 decode + skip_target corroboration.
  Pure functions over caller-supplied energy buffers; touch NO member state.

### New member fields (arq.h)
- `cmd_last_applied_superack_key` (`arq.h`, init `-1`) — CMD dedup of a re-aired
  SUPER-ACK within one turnaround (mirror of `cmd_last_applied_nack_key`,
  `arq.h:4827`).
- `cmd_superack_seen_parity` (`arq.h`, init `0`) — CMD echo of the RSP epoch.
- `inband_rx_superack_parity` (`arq.h`, init `0`) — RSP rate-advice epoch, toggled
  on a CHANGED recommendation.
- `inband_rx_last_superack_target` (`arq.h`, init `-1`) — RSP last-recommended
  target, so the epoch toggles only on a real change.

### New member functions
- `superack_target_from_margin` (arq_common.cc) — PURE margin→ABSOLUTE-target map.
- `build_superack_tones` (arq_common.cc) — clone of `build_nack_tones`.
- `inband_emit_superack` (arq_common.cc, RSP) — clone of `inband_emit_nack`.
- `inband_decode_superack_from_capture` (arq_common.cc, CMD) — clone of
  `inband_decode_nack_from_capture`.
- `inband_handle_superack` (arq_common.cc, CMD) — the fail-safe leap; clones the
  supershift re-trigger block (`arq_commander.cc:8069-8081`) + the `[OPT]` bsi
  rollback (`arq_commander.cc:1067`).

### Mutated pre-existing shared state (the cross-layer surface)
On an accepted SUPER-ACK, `inband_handle_superack` writes:
`negotiated_configuration`, `turboshift_active/phase/initiator/last_good/retries`,
`turbo_snr_ack_enabled/received_snr/best_snr`, `messages_control` (via
`add_message_control(SET_CONFIG)`), `connection_status`, and the CMD bsi ring (via
`roll_back_cmd_bsi_to_inflight`). These are the SAME writes the SNR-driven
supershift re-trigger already performs (`arq_commander.cc:8069-8081`) — the
SUPER-ACK is a PARALLEL trigger, not a new state machine.

---

## §2. The five audit questions (CLAUDE.md §Cross-Layer Data-Flow Audits)

### Q1. Producers of `negotiated_configuration` (+ the turboshift_* leap state)
Every path that WRITES `negotiated_configuration`:
1. Gearshift frame-up elevator — `arq_commander.cc:653/747` (BREAK recovery),
   `:1061` (`[OPT]` optimizer), `:2320`, `:6061` (`inband_climb_target`).
2. SNR-driven supershift re-trigger — `arq_commander.cc:8077` (the block cloned).
3. Turbo terminal / ceiling — `arq_commander.cc:8013/8034/8013`.
4. **NEW: `inband_handle_superack`** (arq_common.cc) — the SUPER-ACK leap.

Producer (4) sets `negotiated_configuration = clamp(skip_target_cfg)` where the
clamp is the SAME never-raise chain elevator_target_from_snr uses
(`arq_commander.cc:462/459-460/471`) applied as UPPER bounds. It runs ONLY the
existing `turboshift_active=true; phase=TURBO_FORWARD; initiator=true;
last_good=current; retries=1; add_message_control(SET_CONFIG);
connection_status=TRANSMITTING_CONTROL` sequence — byte-equivalent to producer (2).

### Q2. Consumers of `negotiated_configuration` / turboshift_* / the SET_CONFIG op
- `process_control_commander` / the SET_CONFIG send path consumes
  `negotiated_configuration` → keys the CONFIG_TAG cross via
  `build_config_tag_tones` (`arq_common.cc:2790`, from `data_configuration`/
  `negotiated`), which the RSP adopts via `unpack_config_tag_payload` +
  `config_tag_wrap_decode` (`arq_responder.cc:4557`).
- `turboshift_phase/active` gate the turbo ladder walk + the reverse-ACK SNR-suffix
  arm (`arq.h:1485-1541`, `arq_responder.cc:1777`).
- `turboshift_last_good` is the fallback rung on a failed leap
  (`arq_commander.cc:3129`, TURBO CEILING settle).

Invariant every consumer assumes: `negotiated_configuration` is a VALID config id
and an UP-move relative to `current_configuration` is a real jump target the
CONFIG_TAG cross can announce. The SUPER-ACK leap satisfies this: `skip_target` is
validated `is_ofdm_config` in BOTH the decode
(`inband_decode_superack_from_capture` rejects a non-WB corroborated target) and
the handle (UP-only + WB gate), and the clamp keeps it ≤ proven ceiling.

### Q3. Valid states — especially BEFORE any producer writes
- `receive_stats.iterations_done` default/pre-decode = `-1`
  (`telecom_system.cc:91/1129`); FAIL sentinel = `> nIteration_max-1`
  (`telecom_system.cc:3315/3378`). `superack_target_from_margin` treats BOTH
  `<0` and `>niter-1` as FAIL → returns `-1` → NO SUPER-ACK emitted. So a stale /
  never-written margin can NEVER emit a spurious skip.
- `cmd_batch_seq_id` default at session start (before first batch) — the CMD bsi
  binding gate (`inband_handle_superack`) compares `ack_bsi_lsb` to
  `cmd_batch_seq_id&0x7` / `(cmd_batch_seq_id-1)&0x7`; a SUPER-ACK bound to any
  other bsi is discarded (fail toward slow).
- `cmd_last_applied_superack_key = -1` (no SUPER-ACK applied) — a first real
  SUPER-ACK (key ≥ 0) is never falsely deduped against `-1`.
- `inband_rx_last_superack_target = -1` (RSP has recommended nothing) — the FIRST
  emit toggles the epoch once (the change from `-1` to a real target).
- `supershift_proven_ceiling = -1` (no ceiling) — the clamp is skipped (no upper
  bound), matching `elevator_target_from_snr` (`arq_commander.cc:466`).

### Q4. Invariants the consumers assume — and that the producer maintains
1. **A SUPER-ACK never fires without a real ACK** (SUPERACK_DESIGN.md §2.4 gate 1).
   The CMD arm is placed INSIDE the CLEAN ACK/SACK confirmation handler
   (`arq_commander.cc:4358`, gated `bsi_in_window && bitmap_ok && !duplicate` +
   CRC-12-valid, `:4215/:4334`), so a missed SUPER-ACK simply lands as a normal
   clean ACK (+1) — the demote-amplifier is not fed.
2. **ABSOLUTE, not relative, skip_target** — a lost/dup SUPER-ACK cannot
   accumulate. The wire field is the target config itself; the CMD always leaps to
   the same value (`inband_handle_superack`, no `+=`). Dedup (§2 Q3) prevents a
   re-aired burst from re-leaping.
3. **UP-only** — `config_ladder_index(skip_target) > config_ladder_index(current)`
   both at decode-map (RSP, `arq_responder.cc`) and at handle (CMD). Never a DOWN
   move (that is the rate-NACK's job).
4. **Never above proven ceiling / NB cap / cooldown** — the SAME clamps as the
   elevator, UPPER-bound only (`inband_handle_superack`). A SUPER-ACK CANNOT defeat
   the batch-resize cooldown (`apply_bigblock_cooldown_cap`, `arq.h:1463`); if the
   cooldown is active the leap may clamp to a no-op (SAFE, §5.5).
5. **Not during an in-flight climb / BREAK / disconnect** — the handle bails if
   `turboshift_active || emergency_break_active || link_status != CONNECTED`
   (mirror of the `[OPT]` gate, `arq_commander.cc:1035-1037`). So the SUPER-ACK
   cannot race a live turboshift or a BREAK recovery.

### Q5. What the fix changes — walk every consumer
The fix adds a SECOND producer of the turboshift leap that is grounded in the
PEER's measured decode margin instead of the local (dead-from-ROBUST) SNR. It
does NOT alter the existing SNR gate (`arq_commander.cc:6052/8047-8048`), the
CONFIG_TAG cross, the RSP adopt path, or the reverse-NACK path. Consumers:
- CONFIG_TAG cross / RSP adopt: unchanged — receives a valid UP target exactly as
  from any other producer. ✔
- Turbo ladder walk: `turboshift_last_good=current` gives it a real fallback rung;
  the reverse-pin (b8076112) keeps the reverse ACK/NACK on ROBUST_1 so an over-leap
  is signalled by a reliable rate-NACK and backs off to `last_good`
  (SUPERACK_DESIGN.md §5.3). ✔
- SET_CONFIG / messages_control: `add_message_control(SET_CONFIG)` after
  `cleanup()` + `roll_back_cmd_bsi_to_inflight` — the identical ordering the
  `[OPT]` producer uses (`arq_commander.cc:1067-1072`). ✔
- SACK / batch-resize: the leap routes through `apply_bigblock_cooldown_cap`, so
  the reverse-SACK-death (65bb60bf) window is respected. The reverse SACK for the
  new config rides the pinned ROBUST carrier → a resize-induced miss is a cheap
  re-air, not a death (SUPERACK_DESIGN.md §5.5). ✔

---

## §3. Fail-safe summary (D0 safety property)

A mis-decoded / lost / duplicated SUPER-ACK degrades to a NORMAL clean ACK (+1) —
NEVER a spurious over-jump. Enforced by ALL of:
- gate 1: real base ACK confirmed (CLEAN handler placement);
- GF(16)+CRC-12 type-6 decode (`superack_wrap_decode`);
- FWHT R_peak ≥ CFG_TAG_PEAK_GATE (`mfsk_ctrl_codec.cc`);
- FWHT skip_target == CRC-field skip_target (corroboration);
- `ack_bsi_lsb` binds to the batch the CMD TX'd (`inband_handle_superack`);
- ABSOLUTE target (no accumulation) + turnaround dedup.
Any single failure → discard the skip, keep the +1. This is the STANAG-5066
"the sender can only reject requests to go faster" asymmetry (SUPERACK_DESIGN.md
§1.2 / §2.4).

---

## §4. Producer/consumer table (for maintenance — update on any change)

| State | Producers | Consumers |
|-------|-----------|-----------|
| `negotiated_configuration` | gearshift/BREAK/`[OPT]`/supershift `8077` / **SUPER-ACK** | SET_CONFIG send → CONFIG_TAG cross |
| `turboshift_*` (leap) | supershift `8069-8081` / **SUPER-ACK** | turbo ladder walk, reverse-SNR arm, `last_good` fallback |
| `receive_stats.iterations_done` | `ldpc.decode` `telecom_system.cc:3195/3275` | turbo surrogate `:3158`, **`superack_target_from_margin`** (RSP) |
| `inband_rx_superack_parity` | **`inband_emit_superack`** (RSP toggle-on-change) | CMD dedup via `epoch_parity` in the key |
| `cmd_last_applied_superack_key` | **`inband_handle_superack`** / cleared on batch advance (`arq_commander.cc:2687`) + ctor | **`inband_handle_superack`** dedup guard |

---

## §5. Regression-test plan (in-process, no IONOS/RF) — REQUIRED before merge

Drive RSP decode → SUPER-ACK emit → CMD decode → leap → CONFIG_TAG cross, asserting
RX config == TX target at each transition. Cases (SUPERACK_DESIGN.md §5.6):
1. **FAIL-sentinel → no-jump**: `iterations_done = niter_max` → margin map returns
   `-1` → no emit; CMD stays put.
2. **round-trip**: `iter=0` on ROBUST → emit skip_target=CONFIG_8 → CMD decodes,
   all gates pass → `negotiated_configuration == CONFIG_8`, turboshift armed.
3. **mis-decode → +1**: corrupt one energy chip → `superack_wrap_decode` rejects →
   `inband_decode_superack_from_capture` returns 0 → no leap (stays clean ACK).
4. **bsi mismatch → discard**: SUPER-ACK bound to a stale bsi → `inband_handle_
   superack` returns false → no leap.
5. **over-leap → clamp**: `supershift_proven_ceiling=CONFIG_4`, skip_target=CONFIG_8
   → clamp to CONFIG_4.
6. **cooldown-active → clamp-to-noop**: `bigblock_carve_cooldown_batches>0` →
   `apply_bigblock_cooldown_cap` pulls the leap ≤ current → `false` (safe no-op).
7. **dedup**: same SUPER-ACK re-aired within a turnaround → leaps ONCE.

Pure-map + payload round-trip cases (1,5,6,7) are driveable from a directed
`--test` member (`superack_target_from_margin` + `pack/unpack_superack_payload` +
`superack_wrap_decode` round-trip). Full RSP→CMD passband round-trip mirrors the
existing NACK directed test (`arq_responder.cc:6839` / `:6858`).

Status: pure-map + codec round-trip land in `mercury --test`; the full passband
round-trip test is the follow-on (same harness pattern as the NACK test).

---

## §7. Integration recalibration + leap smoke (combined build, integ/superack-cleanlock)

Combined build = b8076112 + superack(f799a604 + in-process regression tests) + trio
clean-lock(1804c8c8). Binary md5 `e0995acef8f904107e47e4d6b655ec10` (build.sh o3, box .11).
The merge touches ONLY the ctrl-suffix codec (mfsk_ctrl_codec.{cc,h}) + ARQ (arq_common /
arq_responder / arq_commander) + main.cc; **telecom_system.cc and ldpc*.cc are UNCHANGED**
(git diff b8076112..HEAD), so the ROBUST decode-margin axis is code-identical to base.

### §7.1 Curve A — ROBUST_2 (config 102) iterations_done vs harness SNR (re-measured)
`-m PLOT_PASSBAND -s 102 --ber-esn0=<E> --ber-frames=25`, reading the per-frame `iter=` :
| SNR dB | post-FEC BER | iterations_done | vs phase-1 (§3.2) |
|--------|--------------|-----------------|-------------------|
| <=-10  | >0 (FAIL)    | 201 (sentinel)  | 201 (match)       |
| -8     | 0            | 12..17 (near-floor) | 10            |
| -6     | 0            | 2..5            | 3                 |
| -4     | 0            | 1..3            | 2                 |
| -2     | 0            | 1               | 1                 |
| >=0    | 0            | 0               | 0..1              |
Strictly monotone, matches phase-1 within Monte-Carlo noise. Floor at -8 dB (first clean).

### §7.2 Curve B — WB decodable ceiling vs Es/N0 (re-measured on combined build)
`-m PLOT_PASSBAND -s <cfg> --ber-esn0=<E> --ber-frames=40`, ceiling = highest cfg BER==0:
| Es/N0 | ceiling (combined) | phase-1 (§3.2) |
|-------|--------------------|----------------|
| -8    | cfg3               | (—)            |
| -6    | cfg5               | cfg4           |
| -4    | cfg6               | (—)            |
| -2    | cfg9               | cfg8           |
| 0     | cfg10              | (—)            |
| +2    | cfg12              | cfg11          |
| +6    | cfg13              | cfg13 (match)  |
| +10   | cfg15              | cfg15 (match)  |
| +14   | cfg16              | cfg16 (match)  |
Re-measured ceilings are EQUAL-OR-HIGHER than phase-1 at the low/mid band, identical at top.

### §7.3 Join -> conservative skip map: CONFIRMED, no change
Baked `superack_target_from_margin` (iter>=10->cfg0, 3..9->cfg2, 2->cfg4, 1->cfg6, 0->cfg8)
vs re-measured ceilings at the matching channel: every target stays <= the re-measured
ceiling (decodable on arrival), with MORE headroom than phase-1 assumed (iter=0->cfg8 vs
re-measured ceiling cfg10+ = >=2 rungs). The single at-ceiling edge (iter=1 near -4 ->
cfg6, ceiling cfg6) is still decodable and reverse-pin-cheap. No map adjustment required.

### §7.4 Leap smoke (real audio, snd-aloop WGN:40, start ROBUST cfg100, MERCURY_INBAND_RATE=1)
RSP EMIT = YES (reproduced, 2 runs): `[RSP-SUPERACK] clean ROBUST decode (iters=0/max=200)
-> recommend CONFIG_8 (conf=5)` then `[INBAND-RX] SUPER-ACK emit skip_target=CONFIG_8
(ladder_idx=11) ... parity=1` — the iter=0->cfg8 map fires end-to-end on the wire.
CMD LEAP = NOT observed on real audio: the CMD fires its own robust intra-tier `[INBAND-TX]
UNILATERAL CONFIG 100 -> 101` at the SAME instant the RSP emits, and the config churn then
yields `[CMD-ACK-PAT] Timeout: no ACK detected` on the reverse path, so the single SUPER-ACK
suffix is never decoded (no `[CMD-SUPERACK]`). The CMD-leap LOGIC is proven in-process
(test_inband_superack C3a: `[CMD-SUPERACK] ACCEPT ... DIRECT LEAP`). FOLLOW-ON: the robust
+1 climb races/pre-empts the SUPER-ACK on the trio (the §6.2 "do not run bar=1 and SUPER-ACK
as competing from-ROBUST climbers" hazard, now observed live) — resolve by gating the robust
intra-tier +1 when a SUPER-ACK recommendation is pending, so the reverse ACK stays stable
long enough for the CMD to decode+leap.

Smoke3 full run (240 s): 2 RSP SUPER-ACK emits (T+87, T+154; both iter=0 -> CONFIG_8), 0 CMD decodes; the session crawls the ROBUST tier 100->101->102 and never loads any WB config -- the robust +1 climb, not the SUPER-ACK, is what moves the rate here.

---

## §8. Robust-+1 SEQUENCING GATE (the §7.4 keystone follow-on — implemented; exposes a SIBLING)

ROOT 1 (the competing climb) RESOLVED by the gate; the smoke then exposed a DISTINCT ROOT 2
(see §8.3) so the end-to-end leap still does not land. ROOT 1 (static-trace confirmed): the CMD's OWN robust intra-tier +1
climb and the from-ROBUST SUPER-ACK are TWO competing from-ROBUST rate levers that both fire on
the same clean-ROBUST-ACK turnaround. The +1 changes the live config (`inband_climb_target`
returns `proposed_frame`=`config_ladder_up` on the ROBUST tier because `allow_ofdm_elevator`
requires `is_ofdm_config`; `arq_common.cc:3640`), which is applied via the unilateral CONFIG_TAG
chokepoint (`[INBAND-TX] UNILATERAL CONFIG 100 -> 101`, `arq_common.cc:3458`). That config churn
times out the reverse ACK pattern (`[CMD-ACK-PAT] Timeout`), so the CMD's SUPER-ACK decode
(`inband_decode_superack_from_capture`, invoked in the CLEAN-ACK handler at `arq_commander.cc:4383`)
never gets a clean reverse ACK to pull the type-6 suffix from -> no `[CMD-SUPERACK]`, no leap.

### §8.1 The fix (file:line)
- NEW predicate `superack_plus1_hold_active()` — `arq_common.cc` (after `inband_handle_superack`);
  declared `arq.h` (after `cmd_superack_seen_parity`) with `#define SUPERACK_PLUS1_HOLD_ACKS 4`.
  Returns true (HOLD the robust +1) iff `inband_rate_feature_enabled()` AND `narrowband_enabled
  != YES` AND `is_robust_config(current_configuration)` AND `consecutive_data_acks <
  SUPERACK_PLUS1_HOLD_ACKS`.
- GATE the FRAME-UP +1 fire — `arq_commander.cc:~6436`: the climb-fire `if` now ANDs in
  `!superack_plus1_hold_active()` (via a local `superack_hold_plus1`); a `[GEARSHIFT] robust +1
  HELD for SUPER-ACK` diag prints when the +1 would have fired but for the hold. When held the
  streak is NOT reset (`consecutive_data_acks` keeps accruing) so the fallback fires promptly.
- Regression: `test_inband_superack` CASE 5 (C5a-C5f) — `arq_responder.cc`. Drives the predicate
  across all four conjuncts + the bounded-fallback boundary (`acks>=HOLD -> RELEASE`).

### §8.2 Cross-layer 5-question audit (robust-climb PRODUCER × SUPER-ACK-pending STATE × reverse-ACK CONSUMER)
1. **Producers of the robust +1** (the state being gated = `negotiated_configuration`←robust
   `config_ladder_up`): the FRAME-UP block (`arq_commander.cc:6513` via `inband_climb_target`) is
   the SOLE robust-tier +1 producer. `[OPT]` optimizer owns CONFIG_6+ (not robust); the SNR
   supershift re-trigger + turbo elevator are `is_ofdm_config`-gated (dead from ROBUST — the whole
   SUPER-ACK premise); BREAK is a downward demote, not a +1. Verified: the smoke crawl `100->101->102`
   is the FRAME-UP path. The unilateral chokepoint (`arq_commander.cc:1249`) merely APPLIES
   `negotiated_configuration` (`inband_target`, `:1196`), so gating FRAME-UP stops it at source.
2. **Consumers of the SUPER-ACK-pending window**: the CMD reverse-ACK decode
   (`inband_decode_superack_from_capture` @ `arq_commander.cc:4383`, inside the CLEAN-ACK handler
   `:4341`). It runs EARLIER in the poll than the FRAME-UP block (`:6436`), so with the config held
   STABLE it reads a clean reverse ACK and lands the leap on the first/second clean turnaround.
3. **Valid states before any producer writes**: `consecutive_data_acks` counts consecutive CLEAN
   data ACKs at the current config (`arq_commander.cc:6408`), reset to 0 on the +1 fire (`:6534`)
   and on a failed turnaround. Pre-first-ACK it is 0 -> hold engaged from the first ROBUST poll
   (correct: give the SUPER-ACK a chance before any +1). The RSP emit gate is the mirror image
   (`arq_responder.cc:2592`: `used_mfsk_path && inband_rate_feature_enabled() &&
   is_robust_config(current)`), so the hold is active in EXACTLY the states the RSP emits.
4. **Invariants the consumers assume** — and the gate maintains:
   - The leap path (`inband_handle_superack`) is UNCHANGED; the gate only DEFERS a competing
     producer, it does not touch `negotiated_configuration`/`turboshift_*`/the CONFIG_TAG cross.
   - No double-fire: after a leap, `add_message_control(SET_CONFIG)` sets `messages_control.status
     != FREE`, which the FRAME-UP block already requires FREE (`arq_commander.cc:6403`) -> the +1
     block is skipped that poll anyway; and the leap moves off ROBUST so the hold self-deactivates.
   - **CRITICAL (bounded fallback)**: a SUPER-ACK that NEVER arrives must not stall at ROBUST. After
     `SUPERACK_PLUS1_HOLD_ACKS` clean turnarounds the hold RELEASES and the normal +1 crawl resumes
     (C5c). A failed turnaround resets `consecutive_data_acks` -> restarts the (short) wait, which
     is correct (a non-clean link isn't emitting a SUPER-ACK anyway).
   - NB / off-feature / OFDM never hold (conjuncts 1-3) -> legacy + NB + OFDM-tier climbs are
     byte-identical (no new regression surface).
5. **What the fix changes**: adds one AND-term to the FRAME-UP fire on the ROBUST tier only. Walked
   consumers: the OFDM climb ladder (untouched — `is_robust_config` false), the SUPER-ACK decode
   (now reliably fed a stable reverse ACK), the demote/BREAK paths (downward, never consult the
   hold). No shared state is mutated by the predicate (pure read of existing members).

### §8.3 Leap smoke re-run (gated build md5 75803db78d8217f1967428b9fce6dbd8, WGN:40, cfg100, IR=1, pg84)
3 cells on free cards (dltax held Loopback..Loopback_5): FIX seed1/seed2 (gated) + OLD seed1
(pre-gate baseline e0995acef8f904107e47e4d6b655ec10), 240 s each. `mercury --test` RC=0, CASE 5
green (0 failures).

**The gate is PROVEN working (ROOT 1 removed):** FIX_s1 held the robust +1 5× (`[GEARSHIFT] robust
+1 HELD for SUPER-ACK`), pinning the config at 100 → **6** RSP SUPER-ACK emits (`[RSP-SUPERACK]
... recommend CONFIG_8`) + **6** clean CMD ACKs, vs OLD's **3** emits / 3 clean ACKs. FIX crawled
only 100→101 (hold slowed the +1) vs OLD 100→101→102. So the competing-climb churn is gone.

**BUT the leap still does NOT land — a DISTINCT ROOT 2 (reverse-ACK turnaround / suffix-emit
TIMING), decisively localized (FIX_s1, bsi 0):**
- T+85.44 RSP starts the base MFSK ACK/SACK audio (`[TX-MFSK-ACK-SACK] Audio start (79424 samples)`).
- T+86.398 CMD detects the ACK pattern EARLY (mid-burst, ~1 s before the RSP's ACK audio even
  finishes) → `[CMD-MFSK-ACK-SACK] CLEAN bsi=0` and runs `inband_decode_superack_from_capture`
  on its capture — which does NOT yet contain any SUPER-ACK burst (none emitted yet).
- T+86.603 CMD → `connection_status:Transmitting data` (turns around to send the next batch;
  STOPS capturing the reverse channel).
- T+87.099 RSP `[TX-MFSK-ACK-SACK] Audio done`; T+87.370 RSP `[INBAND-RX] SUPER-ACK emit ...
  burst_samples=75920` — the SUPER-ACK is a SEPARATE ~1.58 s burst emitted ~0.77 s AFTER the CMD
  already turned around. It plays into a channel the CMD is transmitting into → never captured.
- Result: **0** `[CMD-SUPERACK]` decodes across all clean ACKs (both FIX and OLD).

ROOT 2 characterization: `inband_emit_superack` (arq_common.cc:4402) is called AFTER
`send_mfsk_ack_sack` has already PLAYED the base ACK to completion (arq_responder.cc:2568 →
:2611), so the "type-6 suffix on the SAME robust carrier" (design §1/§2.4 intent) is in reality a
LATER separate transmission — and, worse, the CMD's base-ACK detector fires mid-burst and turns
the link around BEFORE even the base ACK audio ends, so appending the suffix to the ACK burst
alone would still miss the capture window. This is a turnaround/capture-window problem at the
PHY/ctrl-suffix layer, NOT the ARQ climb layer §8 fixed. FIX DIRECTION (needs its own design +
cross-layer audit + passband round-trip test, the §5 follow-on): either (a) the CMD, when on
ROBUST with the feature on, EXTENDS its reverse capture past base-ACK detection to include the
trailing suffix before turning around (adds bounded turnaround latency), or (b) the SUPER-ACK
rides as a PREFIX ahead of the base ACK so it lands inside the CMD's ACK-detection window
(risks the shipped base-ACK pattern timing — must be validated). The §8 gate is a NECESSARY
precondition (it stabilizes the config so a fixed suffix path has clean reverse ACKs to ride) and
SHIPS; ROOT 2 is the next keystone.
