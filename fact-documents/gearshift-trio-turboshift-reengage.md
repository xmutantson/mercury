# Data-Flow / Design — TRIO TURBOSHIFT RE-ENGAGE (restore the SNR->ideal jump on the CONFIG_TAG carrier)

Branch: `wip/trio-turboshift-reengage`, off the trio HEAD `2cf0c48d` (wbcross-harden;
in-band redesign, cheap-miss re-air). Scope: the in-band rate-adapt trio
(`MERCURY_INBAND_RATE=1`) DROPPED the SNR->ideal fast climb ("turboshift/supershift")
when it replaced the legacy SET_CONFIG control handshake with the unilateral CONFIG_TAG.
This restores that jump on the OFDM ladder, paired with a first-post-jump batch cap so
the reverse data-SACK turnaround stays survivable.

Companions: `gearshift-climb-engine.md` (§13/§14 the legacy turboshift), `data-flow-snr-
measurements.md` (§9 the MFSK SNR estimator — the SNR is LIVE on the trio), `data-flow-
inband-frame0-rolling-partial.md` (§8 the 65bb60bf +1-suppression), `data-flow-inband-
tier-crossing.md` (§3 the reverse-ACK pin — the residual binding constraint, §7 below).

---

## §1 SYMPTOM & the owner hypothesis

From a CLEAN channel (WGN:40 -> SNR-ideal is cfg13-16) the trio CRAWLS one rung per
FRAME-UP (~220s post-connect) instead of LEAPING to the SNR-ideal config, and runs out of
clock before the WB ladder. The owner: "aren't we describing what turbo/supershift is
supposed to do?" — yes. RESTORE it on the trio, don't reinvent a promote-bar.

## §2 ROOT — the redesign disengaged EVERY turboshift entry (file:line proven)

Legacy reaches cfg16 from a clean channel via TWO SNR->ideal sites, BOTH riding the
SET_CONFIG control handshake:
  (A) the FRAME-UP SNR-elevator (`elevator_target_from_snr()`, arq_commander.cc) — the
      practical every-clean-batch multi-rung jump (legacy log: `FRAME UP config 0 -> 13`).
  (B) the SUPERSHIFT RE-TRIGGER (arq_commander.cc:~7945, the SET_CONFIG ACK-processing
      path) — the cfg13->16 top-up (`[TURBO] RE-TRIGGER: SNR=25.0 dB suggests config 16`).

The trio disengaged BOTH:
  1. Intra-tier config changes are intercepted under in-band at `add_message_control(SET_
     CONFIG)` (arq_common.cc:1189) and routed to the unilateral CONFIG_TAG, RETURNING at
     arq_common.cc:1258 with NO SET_CONFIG on the wire. So the SUPERSHIFT re-trigger (B),
     which fires only in the SET_CONFIG ACK-processing path, is NEVER reached.
  2. The FRAME-UP SNR-elevator (A) was HARD-SUPPRESSED to strict +1 for the in-band path
     by 65bb60bf (`inband_climb_target(..., inband_plus1_on=inband_rate_feature_enabled())`,
     arq_common.cc:3533). That suppression is LOAD-BEARING (§6).

Result: the in-band OFDM climb is a strict one-rung-at-a-time AARF ladder (no SNR->ideal
jump), so from a clean channel it crawls.

## §3 The SNR is ALREADY LIVE on the trio (fix B from the investigation is NOT needed)

The investigation proposed priming `measurements.SNR_uplink` on the trio. VERIFIED
UNNECESSARY by real-audio: the §9 MFSK SNR estimator + the canonical producer
(arq_common.cc:~13559, `= received_message_stats.SNR` on any decoded frame during
SWITCH_ROLE data receipt) already populate a REAL `SNR_uplink` on the trio CMD:
`/tmp/turbo_trio/arq_trio00.log` shows `[CMD] measurements.SNR_uplink= 55.99` from
T+57.5s (221 non-sentinel CMD reads). The FRAME-UP elevator's `SNR_uplink > -90` gate
(arq_commander.cc:6374) is therefore MET on the trio; the only blockers were §2's two
suppressions. So this fix is a pure RE-ENGAGE, no new SNR producer.

## §4 THE FIX — re-engage the FRAME-UP elevator on the in-band OFDM ladder

Chosen site: the FRAME-UP elevator (A), NOT the SUPERSHIFT re-trigger (B). (A) is the
every-clean-batch practical fast-jump and it already runs on the trio's climb path (it just
gets suppressed to +1); (B) is structurally a SET_CONFIG control probe that the trio's
CONFIG_TAG transport does not carry.

- `inband_climb_target(proposed_frame, snr_elevator, inband_plus1_on, allow_ofdm_elevator)`
  (arq_common.cc) — NEW 4th param (default false = byte-identical). When
  `allow_ofdm_elevator` is set the in-band suppression is OVERRIDDEN and the selector
  returns the multi-rung `snr_elevator` (elevator-OR-+1 max, only ever RAISES).
- The FRAME-UP caller (arq_commander.cc:~6416) computes
  `allow_ofdm_elevator = inband_turboshift_reengage_enabled() && is_ofdm_config(current)
   && snr_elevator >= 0 && ladder_idx(snr_elevator) > ladder_idx(proposed_frame)`.
  ROBUST rungs (is_ofdm_config false) stay strict +1 (the intra-ROBUST leg is a distinct
  lever, §7). Reuses the EXISTING `elevator_target_from_snr()` cap-chain VERBATIM (SNR-ideal
  -> NB cap -> supershift_proven_ceiling cap -> the high-confidence-SNR gate) — no new
  target machinery, no new promote-bar.
- `inband_turboshift_reengage_enabled()` (arq_common.cc) — DEFAULT-ON when in-band,
  env-defeatable `MERCURY_INBAND_TURBOSHIFT` (same cheap-miss DEFAULT-ON policy as the A3
  decouple). Off / legacy => byte-identical.

## §5 The 65bb60bf death, defused — the first-post-jump BATCH CAP

65bb60bf suppressed the elevator because a multi-rung jump aired a full-backlog batch at
the jumped rung whose reverse data-SACK turnaround died (`[TX-PEAK] frames=24 ... cfg=3` ->
nAcked_data stuck -> BREAK). The ROOT it NAMED was "keep each rung's batch small enough for
the reverse SACK to decode." This fix attacks THAT root directly instead of the blanket +1:
on a multi-rung elevator jump the FRAME-UP caller arms `inband_climb_jump_batch_cap =
INBAND_CLIMB_JUMP_BATCH_CAP` (1). `process_messages_tx_data()` consumes it as a local
`eff_data_batch_size` cap on the FIRST batch's frame count (retx prefix + new-data fill +
the v1 pad), then disarms — so exactly one batch is small (short forward airtime -> the
reverse data-SACK turnaround survives), and the batch grows back via the normal Axis-2
controller. The cap does NOT touch `data_batch_size` (SACK-bitmap / ACK-timeout sizing
unchanged; a capped batch just occupies fewer padded slots). sack_v2 (in-band) is
non-padding so the cap directly shortens the aired batch. Off/no-cap => byte-identical.

## §6 §5 CROSS-LAYER AUDIT (shared state: FRAME-UP `negotiated_configuration` + `data_batch_size`)

1. **Producers of `negotiated_configuration` on the FRAME-UP path**: ONLY the FRAME-UP site
   (via `inband_climb_target`). Turbo re-trigger / SNR_BASED SET_CONFIG / BREAK recovery are
   different paths; turbo is inactive on the in-band climb (0 markers). The elevator only
   RAISES (elevator-OR-+1 max), never below the +1 anchor clamp.
2. **Consumers of `negotiated_configuration`**: the `add_message_control(SET_CONFIG)`
   chokepoint, which under in-band intra-OFDM routes via the unilateral CONFIG_TAG
   (`inband_unilateral_config_change`, arq_common.cc:3348). That path RE-STAGES the TX FIFO
   (frees messages_tx[], re-pops fifo_buffer_backup -> re-encode at the new config,
   arq_common.cc:3399-3411), so a multi-rung jump's data is re-encoded at the jumped rung's
   sizes; the batch-cap (§5) then bounds the FRAME COUNT. The RSP follows the tag
   (down-window D=4) — a multi-rung UP jump is a forward self-identify; the cheap-miss
   re-air (2cf0c48d) + cumulative n_r self-heal cover a reverse miss during the jump.
3. **`data_batch_size` / `inband_climb_jump_batch_cap`**: the cap is one-shot per jump,
   consumed+cleared in `process_messages_tx_data`, reset in the ctor and `reset_session_
   state`. It is READ only as the local `eff_data_batch_size`; `data_batch_size` itself is
   untouched, so the SACK bitmap, ACK timeout, Axis-2 controller, and retx-runaway gate
   (`2*data_batch_size`) are all unchanged.
4. **Invariants**: the +1 anchor clamp still bounds the target (the elevator was the only
   thing that could outrun it, and it is cap-chained by `elevator_target_from_snr`); the
   floor / anti-thrash nets (AARF `frame_shift_threshold`, probe suppression, demote/BREAK)
   read the streak not the target — untouched. DEEP-SNR inert: at the WGN:-10 cliff the
   `SNR_uplink > -90` gate is unmet OR the helper keeps +1, so the elevator no-ops
   (byte-identical to the +1 ladder there).
5. **What the fix changes**: ONE assumption — "the in-band FRAME-UP elects EXACTLY +1
   (65bb60bf)" -> "the in-band FRAME-UP elects the SNR-ideal jump on an OFDM rung when a
   high-confidence SNR licenses it, with the first post-jump batch capped small." Every
   consumer re-verified. Legacy / feature-off byte-identical.

## §7 VERIFICATION + the RESIDUAL binding constraint (HONEST verdict)

- **Unit** (`--test-inband-plus1-climb`, in the `--test` battery): PASS. New cases F/G/H
  assert the reengage license fires the multi-rung jump (F: in-band + license CONFIG_4 ->
  CONFIG_8), only raises (G), and is safe with no elevator (H). Old A-E (suppressed default)
  still pass. Full `--test`: EXIT=0, 0 failures across the whole battery.
- **Legacy byte-identical**: the legacy arm (new binary, WGN:40) still fires the elevator
  `FRAME UP config 0 -> 13` at T+143 and DELIVERS FULL (16384 B, wall 226s). My change did
  not touch the legacy path.
- **Trio real-audio (WGN:40)**: the `[TURBOSHIFT]`/elevator prints do NOT yet fire on the
  live trio — NOT because the mechanism is wrong, but because the trio never reaches a CLEAN
  OFDM FRAME-UP for the elevator to launch from. TWO upstream constraints dominate:
  (i) the intra-ROBUST 100->101->102 crawl (~164s): is_ofdm_config()==false there so NO
      elevator fires on EITHER path; legacy is fast ONLY because its SET_CONFIG confirm-
      turnaround is quick, the trio's per-rung unilateral CONFIG_TAG waits the slow data-
      SACK (~68s/rung). A distinct lever (dwell/threshold on the CONFIG_TAG confirm), NOT
      turboshift.
  (ii) the cfg0 reverse-confirm fragility: at cfg0 the trio BREAK-thrashes (`[BREAK] ACK
      received! ... config 0 -> 0`) and `consecutive_data_acks` never accumulates, so the
      OFDM FRAME-UP never fires. Legacy sustains cfg0 because it PINS `reverse=101` (ROBUST_1)
      across the cross so its reverse data-SACK rides a rock-solid MFSK rung (legacy log:
      `forward=0 reverse=101`). This is the `data-flow-inband-tier-crossing.md §3` reverse-
      ACK pin gap on the trio's CONFIG_TAG cross — the ACTUAL binding constraint, upstream of
      turboshift.
  A/B control: the SAME trio with `MERCURY_INBAND_TURBOSHIFT=0` behaves identically at cfg0
  (both stuck at cfg0, ~24 B, BREAK-thrash, 0 elevator fires) — proving the cfg0 fragility
  is PRE-EXISTING and this fix is inert until a clean OFDM rung is reached (correct + safe).

**Conclusion**: the SNR->ideal turboshift jump is now RESTORED and correct on the trio OFDM
FRAME-UP path (unit-proven, legacy byte-identical, no regression). But it is NOT the trio's
binding constraint at WGN:40 — that is upstream (the ROBUST-crawl confirm latency + the cfg0
reverse-ACK pin). The elevator will engage once the trio sustains a clean OFDM rung; landing
that requires the §7(ii) reverse-pin work first. This fix is a necessary building block that
ships DEFAULT-ON (proven-correct, byte-identical off) and stops being dormant the moment the
upstream cfg0-confirm lands.
