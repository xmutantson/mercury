# Post-BREAK Stuck Investigation

Date: 2026-05-16
Branch: monitor @ ffa9d75 (Step 15 shipped)
Trigger: Owner challenge after `BREAK_FAILSAFE_INVESTIGATION.md` (the prior
subagent's report concluded "BREAK works fine if you add `-R`" but punted
on **why** the SET_CONFIG at CFG_0 never lands when `-R` is missing).

> "at WGN:38 config 0 should work just fine, this is madness that it doesn't."

This document answers that, and shows that BREAK is **not** the only failure
mode: the **post-BREAK Phase-1 SET_CONFIG-at-CFG_0 round trip is broken under
a specific responder-side state condition** that has nothing to do with the
SNR margin of CFG_0 OFDM as a physical-layer mode.

References to source files use repo-relative paths and `file.cc:line` form.
References to logs are to the original v2 walk in
`fact-documents/axis_walk_v2_desc_gearshift/{cmd,rsp}.log` (the trigger run).

## §1 Background

The prior fact doc (`BREAK_FAILSAFE_INVESTIGATION.md`) established:

1. With `-s 10 -g` and no `-R`, `robust_enabled=NO` → BREAK ladder floors at
   CFG_0 (OFDM BPSK 1/16), not ROBUST_0 (32-MFSK 1/16).
2. After 3 block failures at CFG_15, CMD fires BREAK, RSP ACKs, both load
   CFG_0 (`arq_commander.cc:71-74`, `arq_responder.cc:241-244`).
3. CMD queues Phase-1 SET_CONFIG (target=14) at CFG_0
   (`arq_commander.cc:113-117`).
4. RSP receives the SET_CONFIG frame on CFG_0 but **never processes it**.
5. CMD times out, retries, eventually re-BREAKs, loop forever, 0 bps.

The prior conclusion was "add `-R` and BREAK lands on ROBUST_0 instead, problem
solved." But that explanation never answered the underlying question: **why
doesn't the SET_CONFIG-on-CFG_0 round trip work?** With or without `-R`, the
Phase-1 coordination layer SET_CONFIG should be perfectly carriable by CFG_0
at the SNRs in question. Task 1 (§3 below) proves CFG_0 OFDM is healthy at
WGN:38 in isolation.

## §2 The five anchor facts

| # | Anchor | Source |
|---|--------|--------|
| 1 | BREAK fires on CMD: 3 block-failures at CFG_15 → `send_break_pattern()` | `cmd.log:5654-5656` (axis_walk_v2) |
| 2 | RSP detects BREAK, ACKs, loads CFG_0 as coordination layer | `rsp.log:15487-15500` |
| 3 | CMD receives ACK, loads CFG_0, queues SET_CONFIG(target=14) | `cmd.log:5752-5775` |
| 4 | RSP decodes the SET_CONFIG frame on CFG_0 (logged) | `rsp.log:15676 [RX] CONTROL message received on CONFIG_0, code=59 seq=0/1` |
| 5 | RSP **never** processes it: no `[RX-CTRL] Processing` log, no `[GEARSHIFT] Received SET_CONFIG` log, no ACK back to CMD | `rsp.log:15676..17163` (1487 lines of silence) |

The key contrast: pre-BREAK SET_CONFIG-on-CFG_10 lines `rsp.log:2334-2335`:

```
[RX-CTRL] Processing control message: code=59 (...)
[GEARSHIFT] Received SET_CONFIG: forward=13 reverse=255
```

…both prints appear. **Post-BREAK** SET_CONFIG-on-CFG_0 lines `rsp.log:15676..15677`:

```
[RX] CONTROL message received on CONFIG_0, code=59 seq=0/1
[RX] Batch complete, processing control message immediately
```

…the immediately-following `[RX-CTRL] Processing` line is **absent**.

## §3 Task 1 — CFG_0 OFDM works at WGN:38 in isolation

CLI used (no gearshift, no robust, pinned config):

```
mercury -m ARQ -x alsa -i plughw:Audio -o plughw:Audio --rx-channel 1 \
  -s 0 -Q 0 -M auto -n -v -F off
```

WGN:38 programmed pre-mercury. Both Pis fresh-started. HAIL+START_CONNECTION
succeeded, sustained DATA flow on CFG_0 for 90 s.

Result: **rx=270 B in 90 s = 24 bps user-throughput** (out of CFG_0's ~70 bps
PHY ceiling — overhead is HAIL/control/ACK round trips and OFDM frame timing).
Artifacts in `mercury/fact-documents/task1_cfg0_wgn38/`.

Verdict: **CFG_0 OFDM is healthy at WGN:38.** The post-BREAK stuck state is
NOT a physical-layer SNR problem. It is a higher-layer state machine bug.

## §4 The exact code path that fails

`arq_responder.cc:255-298` — the receiver's CONTROL-frame handler:

```cc
if(messages_rx_buffer.status==RECEIVED)
{
    if(messages_rx_buffer.type==CONTROL)
    {
        printf("[RX] CONTROL message received on CONFIG_%d, code=%d seq=%d/%d\n",
            current_configuration, (int)messages_rx_buffer.data[0],
            messages_rx_buffer.sequence_number, control_batch_size);
        if(messages_control.status==FREE)              // ← gate ##A
        {
            messages_control.type=messages_rx_buffer.type;
            messages_control.id=0;
            messages_control.status=RECEIVED;
            messages_control.length=1;
            messages_control.sequence_number=messages_rx_buffer.sequence_number;
            { ... copy data ... }
            stats.nReceived_control++;
        }
        // [No else clause — incoming frame silently dropped if status!=FREE]
        if(messages_rx_buffer.sequence_number >= control_batch_size - 1)
        {
            printf("[RX] Batch complete, processing control message immediately\n");
            receiving_timer.stop();
            receiving_timer.reset();
            if(messages_control.status==RECEIVED)      // ← gate ##B
            {
                process_control_responder();
            }
        }
```

Two gates:
- **##A** (line 262): copy incoming frame into `messages_control` ONLY if
  `messages_control.status==FREE`. If non-FREE, the new frame's payload is
  discarded silently.
- **##B** (line 287): call `process_control_responder()` ONLY if the gate
  resulting state is RECEIVED. If gate ##A failed, the state is whatever it
  was before — for the v2 log, it's neither FREE nor RECEIVED.

The post-BREAK log signature (`[RX] CONTROL ... code=59` followed by
`[RX] Batch complete` and no `[RX-CTRL] Processing`) **proves** that gate ##A
failed AND the previous state was not RECEIVED. Process of elimination:
`messages_control.status` was in one of `{ACKED, PENDING_ACK, ADDED_TO_LIST,
ADDED_TO_BATCH_BUFFER, ACK_TIMED_OUT, FAILED_}` at the moment the post-BREAK
SET_CONFIG arrived.

## §5 How `messages_control.status` got stuck non-FREE

### §5.1 BREAK handler does not clear messages_control

`arq_responder.cc:226-252` (RSP BREAK handler):

```cc
if(break_detected == YES && link_status == CONNECTED)
{
    printf("[BREAK] %s, dropping to ROBUST_0\n", ...);
    break_detected = NO;
    send_ack_pattern();
    int target = robust_enabled ? ROBUST_0 : CONFIG_0;
    data_configuration = target;
    load_configuration(target, PHYSICAL_LAYER_ONLY, YES);
    calculate_receiving_timeout();
    receiving_timer.start();
    batch_rx_frame_count = 0;
    connection_status = RECEIVING;
    link_timer.start();
    return;
}
```

Notably **absent**: `messages_control.status = FREE`. Whatever state
`messages_control` was in when BREAK fired is preserved across the BREAK
transition.

### §5.2 `load_configuration` does not reset `messages_control.status`

`arq_common.cc:1051-1250` (read in full): rebuilds buffers, resets gearshift
counters, sets timing parameters. Touches nothing on the `messages_control`
struct's `status` field. `messages_control` is a stand-alone state slot
separate from the buffers that `load_configuration` clears.

### §5.3 Where it gets stuck

`process_messages_acknowledging_control()` at `arq_responder.cc:695-801`:

```cc
if(messages_control.status==RECEIVED)
{
    messages_control.type=ACK_CONTROL;
    messages_control.status=ACKED;                     // ← LINE 704
    ...
    send_ack_pattern();   // or send_ack_pattern_with_snr()
    ...
    if(data_configuration != current_configuration)
        load_configuration(data_configuration, PHYSICAL_LAYER_ONLY, YES);
    ...
    messages_control.status=FREE;                      // ← LINE 801
    ...
    connection_status=RECEIVING;
}
```

Between LINE 704 (set ACKED) and LINE 801 (set FREE) the responder performs
a multi-second ACK transmission and possibly a config switch. If a BREAK
fires DURING that window, the responder's BREAK handler returns at line 252
without ever reaching LINE 801.

In the v2 log, this is what happened. The CMD's third consecutive block
failure (`cmd.log:5654`) was triggered while RSP was likely still inside
`process_messages_acknowledging_control()` for one of the late control frames
(`rsp.log:3028 [ACK-CTRL] status=5 ... code=57` was the last ACK-CTRL fire
prior to BREAK at `rsp.log:15487`). Between line 3028 and 15487, multiple
data batches succeeded, so we cannot prove from logs alone that the SLOT was
in ACKED at BREAK time — the slot might have been freed during the data
phase. **However**, the negative evidence (post-BREAK SET_CONFIG never
processed) is conclusive proof that the slot was non-FREE at SET_CONFIG
arrival time. The only way for the slot to BE in a non-FREE state at that
moment is if some prior path left it that way and BREAK preserved the state.

### §5.4 [?] Open: which exact path leaves the slot stuck

Three candidate scenarios that would leave `messages_control.status` in a
non-FREE, non-RECEIVED state across BREAK:

1. **ACKED preservation**: Responder was mid-`process_messages_acknowledging_control()`
   when BREAK was detected. Status was set to ACKED at line 704 and the BREAK
   handler returned before reaching the FREE assignment at line 801.

2. **Late ACK-CTRL retransmit**: After ACK-CTRL is sent and slot is FREE'd
   (line 801), commander's retransmit of the same control frame arrives. The
   FREE-check at line 262 lets it set status=RECEIVED. Before processing
   completes, BREAK fires and returns without clearing.

3. **Some other state transition** in arq_common.cc or process paths invoked
   from `receive()` that touches messages_control.status without restoring
   it. (Not exhaustively searched — `grep -n "messages_control.status\s*="
   arq_common.cc` returns ~30 sites, many in CMD-only paths.)

A trace point printf in the `else` branch of gate ##A would distinguish
these by logging `messages_control.status` and `messages_control.data[0]`
(the old code) at the moment a frame is dropped. A diagnostic patch was
prepared (RSP-side rpi1 ARM build with the printf) but the reproduction
attempts at WGN:36 and WGN:30 with `-s 10 -g` (no `-R`) **did not trigger
the failure mode** — BREAK landed on CFG_14 cleanly each time:

| Reproducer | WGN drop | Result |
|------------|----------|--------|
| `task2_repro` (WGN:40→36) | settle 1792 bps → drop, single block failure each side, no BREAK | recovery, sustained 2240 bps |
| `task2_repro_v2` (WGN:40→30) | settle 1792 bps → drop, BREAK at line 10444, SET_CONFIG on CFG_0 ACK'd at line 10584, recovery at CFG_14 line 11106 | sustained 672 bps post-drop |

In both successful runs, `[RX-DIAG]` (the dropped-frame trace) is absent —
meaning `messages_control.status==FREE` at every CONTROL-frame arrival. The
v2 walk's failure is reproducible only with a specific timing alignment that
my reproducers did not hit. Reproducing it deterministically would require
either:
- Replaying the original audio captures with a controlled trigger
- Adding artificial delay in the ACK-CTRL send path on RSP to widen the
  race window
- Running long walks until the race naturally hits

The reproducers I built do confirm the OPPOSITE bound: when the race
doesn't happen, post-BREAK recovery works cleanly. So the bug is genuinely
intermittent and race-dependent — not a deterministic flaw in BREAK
descent at CFG_0.

## §6 Why the SECOND symptom is fatal

Even if §5 reproduces only intermittently, the impact is total. Once the
responder's `messages_control.status` is stuck non-FREE, **every subsequent
CONTROL frame is silently dropped** until something else FREEs it. Looking
at the v2 log:

| Source | Line range | Behavior |
|--------|-----------|----------|
| `rsp.log:15676..36487` | 21000 lines | 30 `[RX] CONTROL message received on CONFIG_0, code=59` events; ZERO `[RX-CTRL] Processing` events |
| `cmd.log:5773..36450` | 31000 lines | 16 BREAK round trips, all Phase-1 timeouts and re-BREAKs |
| `walk.runner.log` | all steps after step 1 | bps=0.0 |

The state survives every BREAK retry, every Phase-1 timeout, every
load_configuration() call. The only paths that set
`messages_control.status=FREE` (per §5.2 grep) are inside
process_control_responder(), process_messages_acknowledging_control(), and
a handful of session-reset paths. None of these fire because the
state-machine entry condition (status==RECEIVED) is never met.

## §7 Why it never recovers without DISCONNECT

`messages_control.status=FREE` is set at these RSP sites:

- `arq_responder.cc:801` — end of process_messages_acknowledging_control
- `arq_responder.cc:986` — end of HAIL/timeout reset path
- `arq_responder.cc:1227, 1291, 1458, 1526, 1731, 1758, 1901, 1915, 1981,
  2053, 2160, 2209, 2226` — various process_control_responder branches
  (which never fires post-BREAK because of gate ##A drop)

All other sites are gated behind reaching process_control_responder(), which
itself is gated on status==RECEIVED, which is gated on gate ##A succeeding,
which is gated on status==FREE. **The state is its own deadlock.**

The session can only exit the wedged state through:
- A `reset_session_state()` call from CLOSE_CONNECTION (commander hangs up)
- A watchdog-triggered DROPPED transition
- A process restart

None of these happen during a held-open ARQ session experiencing repeated
BREAK retries, which is exactly the v2 walk scenario.

## §8 Proposed fix (minimal, do not implement without owner approval)

### §8.1 RSP-side: BREAK handler must reset messages_control

`arq_responder.cc:226-252`, add ONE line after `break_detected = NO;`:

```cc
if(break_detected == YES && link_status == CONNECTED)
{
    printf("[BREAK] %s, dropping to ROBUST_0\n", ...);
    break_detected = NO;

    // Force-FREE the control slot. BREAK invalidates any in-flight control
    // exchange. Without this, a state stuck at ACKED/RECEIVED/etc. causes
    // every post-BREAK SET_CONFIG to be silently dropped at the gate in
    // process_messages_rx_data_control(). See POST_BREAK_STUCK_INVESTIGATION.md
    messages_control.status = FREE;

    send_ack_pattern();
    ...
```

Rationale: BREAK is a hard reset signal. Any in-flight control message is
moot — the commander is changing config and will re-issue. The slot must be
free for the post-BREAK SET_CONFIG to land. This is symmetric with the CMD
side, which DOES force-clear at `arq_commander.cc:113, 184` (post-BREAK ACK
handlers):

```cc
// Force-clear: cleanup() skips PENDING_ACK status
messages_control.status = FREE;
add_message_control(SET_CONFIG);
```

The CMD's comment ("cleanup() skips PENDING_ACK status") confirms that the
authors know this pattern of stuck control slot exists and have a precedent
for force-FREE'ing it at exactly the BREAK boundary on the CMD side. The
RSP side was missed.

### §8.2 Defensive: gate ##A should at least log when it drops

The silent-discard at line 262's else-branch is dangerous diagnostically.
Even after the fix in §8.1, add the diagnostic so any future stuck state is
visible immediately:

```cc
if(messages_control.status==FREE) { ... copy into slot ... }
else
{
    printf("[RX-CTRL-DROP] CONTROL frame dropped: messages_control.status=%d "
           "(prev code=%d), incoming code=%d seq=%d/%d cfg=%d\n",
        messages_control.status, (int)messages_control.data[0],
        (int)messages_rx_buffer.data[0], messages_rx_buffer.sequence_number,
        control_batch_size, current_configuration);
    fflush(stdout);
}
```

### §8.3 Test: add a unit/integration test for the BREAK boundary

Hard to write a deterministic unit test (the race is timing-dependent).
A minimum viable test would inject `messages_control.status = ACKED` before
calling the BREAK handler and verify that after `break_detected = YES` is
processed, status returns to FREE. This is an additive test of the §8.1 fix.

## §9 Why the prior subagent missed this

The `BREAK_FAILSAFE_INVESTIGATION.md` correctly proved that adding `-R`
makes BREAK land on ROBUST_0 instead of CFG_0, which the §11 validation
walk showed succeeds. The reason `-R` masks the §5 bug:

- ROBUST_0 is 32-MFSK at 1/16 LDPC, 17 dB more sensitive than CFG_0 OFDM.
- ROBUST_0 control batch_size=1 (`arq_common.cc:1157-1159`), so the
  one-frame-per-batch control sequence completes in a single round trip.
- ROBUST_0's longer frame time (~7 s vs ~1.3 s for CFG_0) gives much wider
  timing margin for the ACK-CTRL window in §5.3 to close before BREAK fires.

So `-R` is a robust workaround but the underlying bug is still latent. Any
future scenario that lands the BREAK floor on CFG_0 (or any OFDM config)
remains vulnerable. Even with `-R`, if a BREAK fires during a longer
ROBUST_0 frame transmission, the same race could in principle wedge.

The prior investigation also flagged §8.2 of its own report ("emergency_
previous_config does not get refreshed in the loop") as a benign concern.
With the §5 bug in place, that "benign" detail becomes load-bearing: the
Phase-1 retry loop targets the same failing config forever, BREAK retries
on the same emergency_previous_config forever, and no progress is made.
Once §5 is fixed, refreshing emergency_previous_config per retry becomes
optional polish.

## §10 Recommendation for parent

1. **Approve the §8.1 fix** (one-line addition in `arq_responder.cc:230`).
2. **Apply §8.2** as part of the same change (defensive logging).
3. **Re-run axis_walk v2 (no `-R`)** with the fix in place to confirm the
   post-BREAK stuck state is gone. Expected behavior: BREAK descends through
   CFG_15→14→13→…→0, each step taking a few BREAK round trips, **bps stays
   non-zero throughout the descent**.
4. Leave the `-R` recommendation from the prior fact doc in place — it is
   a strictly stronger failsafe (ROBUST tier survives at lower SNR than
   CFG_0). Both fixes are complementary.

## §11 Status

- §3 Task 1 verified: `mercury/fact-documents/task1_cfg0_wgn38/result.json`
- §5.4 Reproducer attempts: `mercury/fact-documents/task2_repro{,_v2}/`
- §8.1 fix: **proposed, not implemented** (per task instructions)
- §8.2 diagnostic: was patched onto rpi1 RSP for one reproduction attempt
  (built successfully, ran cleanly). The patch is still present on
  rpi1 source tree at `~/mercury-dev/source/datalink_layer/arq_responder.cc`
  line 279-ish — should be reverted before any production deploy.

[?] Open: deterministic reproducer for the race (would let us prove the
§8.1 fix with a failing-then-passing test).
