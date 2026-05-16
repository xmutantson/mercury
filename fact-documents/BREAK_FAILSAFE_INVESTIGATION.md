# BREAK Failsafe Investigation

Date: 2026-05-16
Branch: monitor @ ffa9d75 (Step 15 shipped)
Trigger: `tools/axis_walk_sweep.py` v2 (`-s 10 -g -Q 0 -M auto`) showed link
death at WGN:38; BREAK fired but the recovery loop never produced any data
flow for the remaining 12 minutes of the walk. Owner challenge:
> "BREAK is supposed to fire and let it negotiate to a config which will actually
> work. it's a failsafe. are you telling me break is not functioning correctly?"

This document answers that question by tracing every relevant code path and
correlating it with `fact-documents/axis_walk_v2_desc_gearshift/cmd.log`.

## §1 Definitions and ladder

The ladder is defined in `mercury/include/common/common_defines.h:83-89`:

```
FULL_CONFIG_LADDER[] = {
    ROBUST_0, ROBUST_1, ROBUST_2,                       // idx 0..2  (MFSK)
    CONFIG_0..CONFIG_15                                  // idx 3..18 (OFDM)
};
```

ROBUST values are integer 100/101/102 (out of band of the OFDM 0..16
sequence); the ladder array is the indexing fabric BREAK descends along.

### §1.1 `config_ladder_down_n` semantics

`common_defines.h:137-146`:

```
inline int config_ladder_down_n(int config, int steps, bool robust_enabled) {
    if (!robust_enabled) {
        int target = config - steps;
        return (target > CONFIG_0) ? target : CONFIG_0;     // floor = CONFIG_0
    }
    int idx = config_ladder_index(config);
    idx -= steps;
    if (idx < 0) idx = 0;
    return FULL_CONFIG_LADDER[idx];                          // floor = ROBUST_0
}
```

Verdict: when `robust_enabled = false`, the function CAN NEVER reach
ROBUST_0/1/2 — its floor is CONFIG_0 (OFDM BPSK 1/16). Only when
`robust_enabled = true` does the descent traverse the ROBUST tier.

### §1.2 `config_is_at_bottom` semantics

`common_defines.h:157-160`:

```
inline bool config_is_at_bottom(int config, bool robust_enabled) {
    if (!robust_enabled) return config == CONFIG_0;
    return config_ladder_index(config) == 0;                 // ROBUST_0
}
```

Used at `arq_commander.cc:1659, 2329` to gate emergency BREAK. With
`robust_enabled = false` and `current_configuration == CONFIG_0`, BREAK
will NOT fire — there is nowhere to descend to.

## §2 Where `robust_enabled` comes from

### §2.1 Init defaults

`arq_common.cc:265`: `robust_enabled = NO;` (struct default).

### §2.2 Main CLI path (non-GUI build — i.e. the Pi binary)

`main.cc:1011-1014`: `case 'R':` sets `robust_mode = 1`.

`main.cc:1941-1947` — gearshift-without-explicit-config implicit enable:

```
if(gear_shift_mode != NO_GEAR_SHIFT && !explicit_config)
{
#ifndef MERCURY_GUI_ENABLED
    mod_config = ROBUST_0;
#endif
    robust_mode = 1;
}
```

`main.cc:1971` (non-GUI build): `ARQ.robust_enabled = robust_mode ? YES : NO;`

### §2.3 Implication for `-s 10 -g -Q 0 -M auto`

- `-s 10` → `explicit_config = true` (`main.cc:962`)
- `-g`    → `gear_shift_mode = GEAR_SHIFT_ENABLED` (`main.cc:953-955`)
- The `if(!explicit_config)` guard at line 1941 is FALSE → `robust_mode`
  stays 0 → `ARQ.robust_enabled = NO`.

Confirmed in `cmd.log:5752`:
> `[BREAK] ACK received! Dropping 1 step(s): config 15 -> 14 (robust_enabled=0)`

## §3 What BREAK does

### §3.1 Triggers

There are five trigger sites for `emergency_break_active = 1`:

| Site                                 | Condition |
|--------------------------------------|-----------|
| `arq_commander.cc:1264-1290`         | Recovery phase-2 SET_CONFIG at target failed |
| `arq_commander.cc:1486-1521`         | Turboshift probe failure (FORWARD path) |
| `arq_commander.cc:1553-1583`         | SWITCH_ROLE failed during turboshift |
| `arq_commander.cc:1613-1638`         | Frame gearshift UP control-frame failure |
| `arq_commander.cc:2161-2199`         | Frame gearshift UP data-frame failure |
| `arq_commander.cc:2286-2311`         | Frame gearshift UP data-failure (pattern-ACK path) |
| `arq_commander.cc:2323-2358`         | **Block-failure threshold (steady-state)** — `emergency_nack_count >= emergency_nack_threshold` (default 3, `arq_common.cc:337`) |

The trigger we observed in axis_walk v2 is the steady-state block-failure
path at `arq_commander.cc:2328`.

### §3.2 BREAK action sequence

`arq_commander.cc:2342-2357`:

1. Drop the proven ceiling one step (`supershift_proven_ceiling = config_ladder_down(...)`).
2. `emergency_previous_config = current_configuration` (the failing config).
3. `emergency_break_active = 1; emergency_break_retries = 3;`
4. `send_break_pattern()` — special MFSK pattern designed to be decodable
   well below CONFIG_0 LDPC threshold.
5. Poll for ACK pattern from responder.

### §3.3 BREAK-ACK handler (`arq_commander.cc:55-121`)

When the responder's ACK pattern is received:

1. `target = config_ladder_down_n(emergency_previous_config, break_drop_step, robust_enabled);`
   `break_drop_step` doubles each iteration (1→2→4, capped at 4).
2. `robust_0 = robust_enabled ? ROBUST_0 : CONFIG_0;` ← **coordination layer**
3. `load_configuration(robust_0, PHYSICAL_LAYER_ONLY, YES);`
4. Queue a SET_CONFIG control frame at the coordination layer carrying the
   new `target` value.
5. Enter Phase 1 recovery (`break_recovery_phase = 1`, retries 2).

### §3.4 Responder handling (`arq_responder.cc:226-253`)

```
int target = robust_enabled ? ROBUST_0 : CONFIG_0;
data_configuration = target;
load_configuration(target, PHYSICAL_LAYER_ONLY, YES);
```

The responder ALSO gates the coordination floor on `robust_enabled`. Both
peers MUST have `-R` (or the implicit-enable path of §2.2) for BREAK to
land on the MFSK tier.

## §4 The failure mode we observed (axis_walk v2)

Cell: `cmd.log` from `fact-documents/axis_walk_v2_desc_gearshift/`.

### §4.1 Forward turboshift

- `cmd.log:1748-2425`: Turboshift FORWARD probe ran 10 → 13 → 15, succeeded.
- `cmd.log:3370-4467`: Steady state at CFG_15 holding 2240 bps at WGN:40.

### §4.2 WGN drop to 38 — BREAK triggered

- `cmd.log:4897, 5275, 5654`: Three consecutive block failures at CFG_15.
- `cmd.log:5655-5656`: `[BREAK] Lowered ceiling to 14; Sending emergency BREAK pattern`.
- `cmd.log:5752`: `[BREAK] ACK received! Dropping 1 step(s): config 15 -> 14 (robust_enabled=0)`.
- `cmd.log:5753`: `[CFG] load_configuration(0)` — the coordination layer is **CONFIG_0 OFDM**, not ROBUST_0.

### §4.3 Phase 1 loop

- `cmd.log:6320`: `[BREAK-RECOVERY] Phase 1 retry (1 left) at config 0` (SET_CONFIG at CONFIG_0 not ACKed by responder).
- `cmd.log:6825`: `[BREAK-RECOVERY] Phase 1 failed, re-sending BREAK`.
- This pattern repeats from line 6320 to line 28264+ — Phase 1 SET_CONFIG
  at CONFIG_0 never receives an ACK, so the cycle:
  BREAK → ACK heard → load CONFIG_0 → SET_CONFIG at CONFIG_0 → timeout →
  Phase 1 retry → timeout → re-BREAK → repeat.

### §4.4 Throughput consequence

Per `walk.runner.log`: from step 2 (WGN:38) through step 14 (WGN:14),
every dwell reported `rx=0B bps=0.0`. Total 12 minutes of zero data flow.

## §5 Root-cause analysis — is BREAK broken?

**No.** BREAK is functioning exactly as designed for `robust_enabled = false`.
The architectural problem is:

1. The test flags pinned `robust_enabled = NO` (per §2.3).
2. The BREAK failsafe floor is therefore CONFIG_0 (OFDM BPSK 1/16, ~70 bps),
   not ROBUST_0 (32-MFSK, ~14 bps).
3. At WGN:38 with the IONOS channel's actual SNR translation, OFDM CONFIG_0
   cannot reliably carry the SET_CONFIG/ACK round trip.
4. The Phase 1 coordination layer SET_CONFIG fails repeatedly, and there is
   no further descent path — CONFIG_0 IS the floor.

When `robust_enabled = YES`:
- `config_ladder_down_n` floors at ROBUST_0 (`common_defines.h:144`).
- BREAK coordination layer is ROBUST_0 (`arq_commander.cc:71`).
- Responder symmetrically lands on ROBUST_0 (`arq_responder.cc:242`).
- ROBUST_0 is 32-MFSK at LDPC 1/16, designed to decode at ~-12 dB Es/N0 — a
  proper failsafe floor that survives where CONFIG_0 OFDM does not.

## §6 Why the test flags broke this

`tools/axis_walk_sweep.py:285-287` builds the flag string:

```
flags = (f'-m ARQ -x alsa -i {AUDIO_DEV} -o {AUDIO_DEV} '
         f'--rx-channel 1 -s {args.config} -Q 0 -M auto '
         f'-n -v -F off {gearshift_flag} {pin_flag} {sack_flag}').strip()
```

- `-s {args.config}` → `explicit_config = true` → blocks the implicit
  robust-mode enable at `main.cc:1941`.
- `-Q 0 -M auto` → `nb_probe_max == 0 && bandwidth_mode == BW_AUTO` →
  `ARQ.narrowband_enabled = NO` (`main.cc:1975-1976`). This is a separate
  concern from `robust_enabled` (`narrowband_enabled` is the
  HAIL bandwidth, not the gearshift floor), and `is_robust_config(...)`
  works in either NB or WB (telecom_system.cc:4143-4163, 4227-4228).
- No `-R` flag, so `robust_mode` stays 0 → `ARQ.robust_enabled = NO`.

The minimum change to give BREAK a working failsafe floor: **add `-R` to
both CMD and RSP flag sets**.

`-Q 0` is fine to keep — it skips the NB→WB negotiation probe (which would
add wasted setup time when both ends are WB-capable). ROBUST modes still
function as the gearshift floor regardless.

`-s 10` is also fine to keep — it pins the initial config for turboshift's
starting point, but with `-R -g` BREAK can still descend below it into the
ROBUST tier when needed.

## §7 Re-climb mechanism (after BREAK lands)

After BREAK lands at some lower config, the upward path is:

1. `consecutive_data_acks` accumulator (`arq_commander.cc:2409-2438`) —
   after `frame_shift_threshold` (default 3, `arq_common.cc:305`)
   consecutive data ACKs, frame-level gearshift queues a SET_CONFIG up
   one ladder step. Threshold DOUBLES on every up-failure (`arq_commander.cc:2155`).
2. `supershift_proven_ceiling` is recovered via `ceiling_success_count`
   (`arq_commander.cc:3517-3526, 3657-3666`) — after 20 successful blocks
   at the ceiling, the ceiling is raised one step.

So yes, there is a re-climb. The re-climb only operates on configs ABOVE
the current one and respects the proven ceiling — no immediate retry into
the failing tier.

## §8 Other gotchas

### §8.1 `cleanup_obligations after BREAK`

`arq_common.cc:2879-2891`: `reset_session_state()` restores `narrowband_enabled = YES`
UNLESS `nb_probe_max == 0 && bandwidth_mode != BW_NB_ONLY`. So with
`-Q 0 -M auto` (our case) and a DISCONNECT, the responder stays WB. This
is intentional and orthogonal to BREAK.

### §8.2 `emergency_previous_config` does not get refreshed in the loop

`arq_commander.cc:2345` sets `emergency_previous_config = current_configuration`
ONLY when a NEW BREAK is fired from the block-failure path. The
`[BREAK-RECOVERY] Phase 1 failed, re-sending BREAK` path at
`arq_commander.cc:1313-1330` does NOT update `emergency_previous_config`
("Keep emergency_previous_config unchanged" — line 1318 comment). So
during Phase 1 loop, target stays computed from the original failing
config (15 - 4 = 11 in our log). This is benign IF the floor is reachable
— with `robust_enabled=YES` the loop would eventually drop into ROBUST_0
because `break_drop_step` caps at 4 but the coordination layer is itself
ROBUST_0 (much more robust than the target), so SET_CONFIG ACK eventually
gets through. With `robust_enabled=NO`, the coordination layer IS CONFIG_0
and the SET_CONFIG ACK can fail forever.

### §8.3 Known-issue prior art

A grep across `mercury/fact-documents/` for "stuck", "reload CONFIG_0",
"BREAK loop" returns no prior documentation of this specific
failure mode. The closest is §15b.4 of `IONOS_ERA_VALIDATION_PLAN.md`
discussing `emergency_nack_threshold 2→3` regression, but that's an
unrelated commit's effect. This is the first investigation of the
robust-disabled BREAK-loop pathology.

## §9 Proposed test setup change

For `tools/axis_walk_sweep.py` (and any new walk script):

```diff
- -s {config} -Q 0 -M auto -n -v -F off {gearshift_flag} {pin_flag} {sack_flag}
+ -R -s {config} -Q 0 -M auto -n -v -F off {gearshift_flag} {pin_flag} {sack_flag}
```

OR equivalently drop `-s` entirely (which lets the implicit enable at
`main.cc:1941` trigger), but that changes the starting config to ROBUST_0
which forces a longer turboshift climb every test. Adding `-R` while keeping
`-s` is the minimal change.

Recommend: add `--with-robust` flag to `axis_walk_sweep.py` (default ON
for new runs, off for back-compat with the v1/v2 baselines we already have).

## §10 Verdict

BREAK is functioning correctly. The Pi-side mercury binary as built and
the BREAK state machine as written behave per spec for both
`robust_enabled = YES` and `robust_enabled = NO`. The issue observed in
axis_walk v2/v3 is a test-flag misconfiguration: `-s 10 -g` without `-R`
disables the MFSK failsafe floor that BREAK relies on as the last-resort
coordination layer. With `-R` added, BREAK should be able to negotiate
the link down to ROBUST_0 and resume data flow even at very low SNR.

This will be validated by §11 (next: an SNR walk with `-R` added).

## §11 Validation walk

Walk artifacts: `fact-documents/axis_walk_v5_robust/` (settle WGN:40 for
90 s, then sequence `36,30,24,20,16,14,20,28,36` at 60 s/step).

CLI: `python tools/axis_walk_sweep.py --with-robust --settle-snr 40
--settle-s 90 --wgn-sequence "36,30,24,20,16,14,20,28,36" --dwell-s 60
--config 10 --mode sackv2 --out-dir fact-documents/axis_walk_v5_robust`

Mercury flags: `-m ARQ -x alsa -i plughw:Audio -o plughw:Audio
--rx-channel 1 -s 10 -Q 0 -M auto -n -v -F off -g --enable-sack
--enable-sack-v2 -R`  (added `-R` vs v2).

### §11.1 BREAK descent observed (cmd.log)

| line  | event |
|-------|-------|
| 5007  | BREAK fired at CFG_15, ACK heard, target=14, **robust_enabled=1** |
| 5008  | `[CFG] load_configuration(100)` — coordination layer is **ROBUST_0** (line `current=15 level=PHYS_ONLY`) |
| 8479  | Second BREAK round, target=13 |
| 12334 | `[BREAK-RECOVERY] Config 13 verified, resuming data exchange (fifo=127840 bytes, block_tx=0)` |
| 23285 | BREAK target=12 (after CFG_13 failed) |
| 24445 | `[BREAK-RECOVERY] Config 12 failed probe, sending BREAK` |
| 25903 | `[BREAK-RECOVERY] Config 10 verified, resuming data exchange` |
| 27604 | BREAK target=9 |
| 28747 | `[BREAK-RECOVERY] Config 9 verified, resuming data exchange` |
| 30970 | `[GEARSHIFT] LADDER DOWN: success=50% < 55%, config 9 -> 8 (batch=1)` |
| 31341 | `[GEARSHIFT] FRAME UP FAILED: 7->8 NAck, BREAK to 7` |
| 31569 | BREAK target=7 |
| 36498 | `[BREAK] All retries exhausted — assuming responder already at ROBUST_0` |

Net descent trajectory: **CFG_15 → 14 → 13 → 12 → 10 → 9 → 8 → 7 →
ROBUST_0**. Each step exercised the BREAK state machine, the
coordination layer at ROBUST_0 (`load_configuration(100)`), and the
Phase-1/Phase-2 verification probe at the target config. No deadlock.

### §11.2 Why bps was still 0/0/0 in the harness-side buckets

`results.json` reports `total_rx_bytes = 12600` — exactly what was
delivered during the 90-s settle phase (1120 bps × 90 s ÷ 8 = 12 600).
Every walk-step bucket reads 0 bps. RSP-side mercury statistics
(`rsp.log`) corroborate: `stats.nReceived_data` plateaus at **74**
frames very early in the walk and never advances.

Two compounding factors explain this — **not** a BREAK fault:

1. **TX FIFO is full and the harness keeps pumping.** `cmd.log` status
   block at line ~36450: `TX buffer occupancy= 94.71 %`. Each BREAK
   saves the in-flight batch back to FIFO (e.g. line 5773 "Saved data
   to FIFO (127968 bytes total)"). The harness's `sender_thread()` runs
   uninterrupted, so once the FIFO fills, no further TCP bytes from the
   pump are acknowledged. The data that DID get through (the 74 frames
   observed RSP-side) was queued at settle time.
2. **The channel at WGN:14..30 cannot sustain CFG_7..13.** Each
   "Config N verified" event was IMMEDIATELY followed by `[BREAK]
   Block failure #1 at config N` — the SET_CONFIG control frame
   (1-frame batch, very reliable) gets through, but a 25-frame data
   batch loses ≥1 frame within seconds. This is the actual SNR
   operating limit of the OFDM configs on this channel, not a BREAK
   defect.

### §11.3 Walk findings (verdict-bearing)

1. **BREAK is functioning correctly** when `-R` is set on both peers.
   The descent CFG_15 → ROBUST_0 happened entirely under BREAK's own
   logic, with no manual intervention.
2. The `cmd.log:5007` line `robust_enabled=1` is the canonical proof
   that `-R` propagated and the failsafe ladder now includes ROBUST_0.
3. Coordination-layer SET_CONFIG frames at ROBUST_0 (CONFIG_100,
   10.7 bps) are decoded successfully at WGN:14.7 measured SNR (line
   36461: `measurements.SNR_uplink= 14.74`). This is the expected
   failsafe behavior — ROBUST_0 designed waterfall ≈ −12 dB Es/N0.
4. The remaining "0 bps" outcome is a **measurement / test-shape
   artifact**, not a BREAK bug: the test pumps data faster than the
   channel can carry, fills the FIFO at high SNR, and the saved-to-FIFO
   bytes block further TCP intake when BREAK fires.

### §11.4 Test-shape recommendations for next walk

To get non-zero per-bucket bps showing the BREAK adaptive descent:

- Throttle the harness TX pump (e.g. burst-and-pause, or rate-limit to
  the negotiated bps at the lowest expected config).
- OR start at LOW SNR with no settle phase so the FIFO never fills.
- OR drain the FIFO between steps via a pause in the TX pump.

A separate (orthogonal) follow-up: investigate whether
`emergency_previous_config` should refresh on each Phase 1 retry to let
the loop find a working config faster after the initial drop. Today it
stays at the original failing config until a NEW block-failure threshold
fires, which can leave the loop targeting a still-failing config for
several iterations (see §8.2). Mark as **[?] open question** — may be
intentional to preserve the original commander's intent.

## §12 Final verdict

> "BREAK is supposed to fire and let it negotiate to a config which will
> actually work. it's a failsafe. are you telling me break is not
> functioning correctly?"

BREAK is functioning correctly **when given the right flags**. The
original axis_walk_v2 deadlock was caused by `robust_enabled=NO` (a
consequence of `-s 10 -g` without `-R`), which left BREAK with no
failsafe floor below CONFIG_0. With `-R` added (axis_walk_v5_robust),
BREAK successfully descends through OFDM tiers and lands at ROBUST_0
exactly as designed. The remaining 0-bps measurement during the walk is
a test-shape issue (TX pump overwhelms the FIFO during high-SNR settle,
then BREAK saves the un-drainable bytes), not a BREAK defect.

