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
