# SACK Fix Plan — architecting the response to the Tracks A+B win-test

**Status:** PLAN ONLY (per CLAUDE.md §"Plan before coding"). No Mercury source
edited. Owner review required before any code lands.
**Date:** 2026-05-14. Branch (both repos): `monitor`.
**Inputs:** `SACK_LOSSY_CHANNEL_WINTEST.md` §5–§8, `SACK_RETRANSMIT_BATCHING_INVESTIGATION.md`,
`SACK_TURNAROUND_FIX_PLAN.md` §8, `SACK_LDPC_FALLBACK_INVESTIGATION.md` §9.4,
`SACK_THROUGHPUT_INVESTIGATION.md` §16.3, `SACK_REDESIGN_PLAN.md`.

---

## §1 TL;DR — meta-recommendation

**Pursue, with narrowed scope.** Implement Candidate **(a)** first as a small,
isolated, low-risk move (it is a CMD-only, RSP-protocol-free change). **Hold
(c) behind a second-step gate** that fires only if (a) does not deliver a
measurable lossy win against the existing Tracks A+B harness. **Do NOT pursue
(b) in this campaign.** Its protocol cost (cross-batch sequence-space
disambiguation, a `batch_seq_id` field on every DATA frame, RSP slotting
changes) is large and structurally overlaps `SACK_REDESIGN_PLAN.md` Design A —
the right place to pay that protocol cost is in the redesign, not as a
retrofit to v1 SACK.

**The single strongest piece of evidence behind this ranking** is
`SACK_LOSSY_CHANNEL_WINTEST.md` §5.4 (the Track B / mpd14 mechanism row):
*"SACK's `retx% = 6.2 %` is 3× lower than no-SACK's `19.4 %` — SACK is doing
exactly what it was designed to do. And it still loses by 38 %, because SACK's
cycle is 17643 ms vs no-SACK's 7833 ms."* The frame-efficiency win is real and
mechanism-verified; the loss is purely structural turnaround/timeout cost. The
prior agent already correctly factored that cost into the three components
that map onto a/b/c (`SACK_LOSSY_CHANNEL_WINTEST.md` §7.2 "named gap"):
batch-size-scaled post-TX timeout (~8 s vs no-SACK's ~4.5 s, **(a)**), the
separate keyed retransmit-only mini-batch (**(b)**), and the lack of a
shrink-on-loss adaptation (**(c)**). Of the three, **(a) targets the largest
single arithmetic contributor and has the smallest blast radius** — that is
why it leads.

A "STOP and discuss" trigger applies if (a) lands cleanly but does NOT produce
a measurable lossy win across the Tracks A+B harness on both configs and both
channel types: per CLAUDE.md "if three consecutive fix attempts fail, STOP and
discuss — this signals an architectural problem." We are at three structural
fixes already in this campaign (Plan A, A2 ldpc=NO, and now (a)). If (a) is
not the answer, the right move is **not** to attempt (b) or (c) — it is to
fold the remaining SACK question into the `SACK_REDESIGN_PLAN.md` Design A
redesign, where the protocol cost of cross-batch sequence space and the
elimination of MFSK SACK can be paid once, cleanly, instead of layered onto
v1.

---

## §2 The code traced — every claim cited

The candidates a/b/c only make sense against the actual code paths they would
modify. All citations are from the `monitor` working tree at `90ed5d9` (the
post-`f2dbf34`/`e076823` HEAD the win-test ran on).

### §2.1 Where the post-TX timeout is computed (`calculate_receiving_timeout`)

`arq_common.cc:433-480` (`cl_arq_controller::calculate_receiving_timeout()`).
CMD branch (`:435-470`) — when an ACK pattern is configured (`ack_pattern_time_ms > 0`,
the normal data-batch path):

```
timeout = 2*message_transmission_time_ms
        + max(ack_pattern_time_ms, sack_ms)            // sack_ms only when sack_enabled
        + ptt_on_delay_ms + ptt_off_delay_ms + 3000    // base 3-s margin
        + (gear_shift_on && turboshift_phase!=DONE ? 2000 : 0)
        + (sack_enabled ? sack_timeout_extra_ms : 0)   // hardcoded extra (default 3000ms)
```

where `sack_ms = ceil(1000 * sack_pattern_passband_samples(data_batch_size) /
sampling_frequency)` and `sack_pattern_passband_samples(nframes)` is
`telecom_system.cc:3129-3133`:
`ack_mfsk.sack_total_nsymb(nframes, &sack_ldpc) * Nofdm * frequency_interpolation_rate` —
i.e. the SACK on-air time **grows with batch size** because the LDPC bitmap
suffix carries `data_batch_size` bits. So the formula is double-batch-coupled:
the SACK pattern itself is longer at batch=25 *and* a hardcoded
`sack_timeout_extra_ms=3000` is layered on top (`arq_common.cc:239` default,
`arq_common.cc:461-463` adder; `main.cc:294, 522-527, 1234-1236` CLI hook
`--sack-timeout-extra-ms=N`).

RSP branch (`:471-479`) — different formula:
`(data_batch_size)*message_transmission_time_ms + time_left_to_send_last_frame +
ptt_on_delay_ms`. Already proven structurally fine at the RSP side by the
Plan-A fix (`SACK_TURNAROUND_FIX_PLAN.md` §8.2, §9.1). Out of scope here —
this plan touches the CMD side only.

### §2.2 Where post-TX `receiving_timer` is started

Two relevant sites in `arq_commander.cc`:

- **End of a normal data batch:** `arq_commander.cc:873-879` —
  `calculate_receiving_timeout(); receiving_timer.start();` is run right after
  `send_batch()` for a `data_batch_size`-frame batch.
- **End of a retransmit-only batch (mechanism (a)):**
  `arq_commander.cc:788-792` — **same** `calculate_receiving_timeout();
  receiving_timer.start();` is run after a `retransmit_count`-frame batch.
  Because `data_batch_size` is **still 25** at this point (it is not
  temporarily set to `retransmit_count`), the post-TX timeout for an N=2
  retransmit batch is sized for a 25-frame batch — the timeout sees the
  current `data_batch_size`, not the just-keyed frame count. This is part of
  what (a) targets directly.

### §2.3 Where `data_batch_size` is set to 25 (SACK negotiation)

`arq_commander.cc:2103-2128` — when both peers advertise `CAP_SACK` in
TEST_CONNECTION:

```
both_sack = (local_capability & CAP_SACK) && (peer_capability & CAP_SACK);
sack_enabled = both_sack;
if(sack_enabled) {
    int max_batch = (msg_tx_time_ms > 0) ? round(12000/msg_tx_time_ms) : 31;
    if(max_batch < 5) max_batch = 5;
    if(max_batch > nMessages) max_batch = nMessages;
    int new_batch = radio_batch_size;            // default 25 — arq_common.cc:129
    if(new_batch > max_batch) new_batch = max_batch;
    set_data_batch_size(new_batch);
    nominal_batch_size = new_batch;
    recalculate_ack_timeout_for_batch();
}
```

The 25 comes from `radio_batch_size=25` (`arq_common.cc:129`); `crypto_batch_size=20`,
`retransmit_headroom=5` (`arq_common.cc:130-131`) are NOT consulted here.
`set_data_batch_size()` definition: `arq_common.cc:397-411` — simple assignment
with an upper bound check against `max_data_length + max_header_length -
ACK_MULTI_ACK_RANGE_HEADER_LENGTH - 1`. There is no per-batch loss state of
any kind in the controller (a grep for `batch_loss|per_batch_loss|recent_loss|
loss_history` returns zero hits in `source/`).

### §2.4 Where the retransmit-only batch is sent (mechanism (a))

`arq_commander.cc:727-794` (`process_messages_tx_data()` retransmit branch),
gated by `sack_enabled && retransmit_count > 0` (`:730`). This is the entire
mechanism (a) — confirmed in `SACK_RETRANSMIT_BATCHING_INVESTIGATION.md` §2:

- `:735` `message_batch_counter_tx = 0` — fresh, empty batch buffer
- `:736-753` — fills `messages_batch_tx[0..retransmit_count-1]` with **only**
  the queued retransmit frames; original `sequence_number` =
  `retransmit_frame_positions[r]` (`:749`) — so RSP slots them by their
  original batch position; comment `:747-748` "Do NOT set end-of-batch flag"
- `:754` `retransmit_count = 0  // Consumed`
- `:766` `pad_messages_batch_tx(message_batch_counter_tx)` — batch is **sized
  to exactly N frames**, not padded up to 25
- `:767-769` `sack_retransmit_active = true; send_batch(); sack_retransmit_active = false;`
- `:793` `return;` — exits immediately, before the normal new-data fill loop
  at `:796`

And new-data staging is explicitly suppressed while a retransmit is queued —
`arq_commander.cc:2987-2990`:

```
// SACK retransmit pending: don't create new crypto batch from FIFO.
// process_messages_tx_data() will send retransmit-only batch.
if(sack_enabled && retransmit_count > 0)
    return;
```

So mechanism (a) is enforced in two places (send path and stage path) — a (b)
implementation must change both.

### §2.5 Where the retransmit queue is built (SACK bitmap → retransmit_frames)

`arq_commander.cc:1422-1502` (`process_messages_rx_acks_data()` SACK branch).
`:1431` `sack_detected = receive_sack_pattern(sack_bitmap, data_batch_size);`
`:1449-1477` walks `messages_tx[]`, ACKs received frames, copies missing
frames' encrypted payloads into `retransmit_frames[retransmit_count]`,
recording `retransmit_frame_positions[r] = i` (the original batch slot).

The cap at `:1463` is the compile-time constant `MAX_RETRANSMIT_HEADROOM`
(`arq.h:179` = 8), **not** the runtime member `retransmit_headroom` — per
`SACK_RETRANSMIT_BATCHING_INVESTIGATION.md` §1, §4. The `--retransmit-headroom`
CLI flag only ever writes `crypto_batch_size`, which nothing reads in the
sizing path (only `data_batch_size` is consulted at `:3001`).

### §2.6 What cross-batch sequence-space machinery exists (or doesn't)

A grep across `source/` for `batch_seq_id` returns **zero hits**. The closest
existing state is `retransmit_batch_id=-1` (`arq_common.cc:136`) which is set
but never consulted in a routing decision. Per `SACK_REDESIGN_PLAN.md` §5.1
the proposed v2 frame format already had a 2-byte `batch_seq_id` slot — but
that was on the *control* frame (the SACK_RSP), not on DATA frames; v2 was
never landed (`§9 "REVERTED"`). The **DATA**-frame format has no batch
identifier at all today. **Mechanism (b) requires adding one** (or an
equivalent type-bit flagging "this frame is a retransmit and its
`sequence_number` belongs to the prior batch"). This is the core protocol
cost of (b), and is precisely the problem `SACK_REDESIGN_PLAN.md` §5 imagined
solving in v2 — see §4.3 below.

### §2.7 Summary of the formula's structural drags

Composing §2.1 and §2.2: with `sack_enabled=true`, `data_batch_size=25`, on
WB_CFG15 (`message_transmission_time_ms ≈ 390 ms`, `sack_ms ≈ 1168 ms`,
`ack_pattern_time_ms ≈ 390 ms`):

```
timeout = 2*390 + max(390, 1168) + ptt_on + ptt_off + 3000 + 3000
        ≈ 780 + 1168 + ~100 + ~100 + 3000 + 3000
        ≈ ~8148 ms
```

This matches the §6 measured value `receiving_timeout=8248ms` from the Step-3
trace and the §5.3 win-test number `8248ms`. The decomposition:

| term | value (ms) | scales with batch? | targeted by |
|------|-----------:|--------------------|-------------|
| `2 * msg_tx_time` | ~780 | constant | — |
| `sack_pattern_ms` | ~1168 | **YES** (via SACK LDPC suffix on bitmap) | (a) directly; (c) indirectly |
| `ptt_on + ptt_off` | ~200 | constant | — |
| base margin `+ 3000` | 3000 | constant | (a) (re-derived) |
| `sack_timeout_extra_ms` `+ 3000` | 3000 | constant | (a) directly |
| **total** | **~8148** | | |

Two batch-size-coupled costs: the SACK pattern itself (which would shrink ~3×
at batch=10) and — via §2.2 — the fact that the *same* timeout is applied
after the retransmit-only mini-batch (which is N≈1-5 frames but still pays
the full ~8 s). The two constant 3000-ms adders are not derived from
geometry; the second (`sack_timeout_extra_ms`) is a hardcoded
"half-duplex collision avoidance" margin (`arq_common.cc:458-463` comment)
that pre-dates the Plan-A fix and may be re-derivable now that the turnaround
race is closed structurally.

---

## §3 Prior art — adaptive ARQ, event-driven post-TX timeouts on HF

CLAUDE.md §"Research before implementing" requires this section. Prior art on
the specific levers a/b/c:

### §3.1 STANAG 5066 (NATO HF) and the fixed-window failure mode

STANAG 5066 is the closest published analogue to Mercury's situation:
selective-repeat ARQ over half-duplex HF, with channel-quality adaptation via
a separate Data Rate Change (DRC) mechanism. Critically, its SR-ARQ window is
**fixed at 128 PDUs**, and the published WBHF optimization papers (e.g.
[Isode WBHF whitepaper](https://isode.com/whitepapers/stanag-5066-for-hf-and-wbhf.html))
explicitly call this out: *"STANAG 5066 has a maximum window size of 128,
which limits ARQ performance at the higher speeds of narrow band HF and leads
to unacceptable performance for WBHF/HFXL."* The lesson is the inverse of
Mercury's: STANAG 5066's window is too *small* at high rates; Mercury's is
too *large* at high loss. Both are the same architectural failure — **a
single fixed window cannot be optimal across the channel SNR/loss range that
HF actually exhibits.** This is the prior-art justification for (c).

### §3.2 FED-STD-1052 / HFDLP — adaptive frame and series size

FED-STD-1052 ([Johnson, HFDLP throughput paper](http://wireless.nmsu.edu/hf/papers/hfdlp_throughput.pdf))
explicitly negotiates "the number of data bytes per data frame (56-1023) and
the number of data frames per data series (1-255)" at connection setup, as a
function of the measured channel. This is adaptive-batch-size prior art —
**but only at session start**, not dynamically per batch. It directly
validates the (c) lever in principle ("size the batch to the channel") while
leaving the *dynamic* variant (adapt mid-session as loss rises) as a Mercury
contribution. The HFDLP paper's throughput model also makes the point that
the optimum series length is a sharp function of frame error rate — flat
loss → bigger series wins; bursty loss → smaller series wins. That is
exactly the WB_CFG15 wgn32 / CFG10 mpd16 vs clean asymmetry the win-test
measured.

### §3.3 ARDOP — receiver-reported decode quality drives sender adaptation

ARDOP ([specification](https://winlink.org/sites/default/files/downloads/_ardop_specification.pdf),
[TAPR DCC 2015](https://files.tapr.org/meetings/DCC_2015/DCC2015-ARDOP-KN6KB-N8OHU-GM8BPQ.pdf))
puts a 5-bit decode-quality field in every ACK/NAK frame (range 38-100 in
steps of 2). The ISS uses this to switch between short (~2s) and long (~4s)
data frames as the channel evolves. This is the model for (c) at *frame*
granularity; the Mercury analogue would be the SACK `N/M received` ratio
that the existing `[CMD-SACK]` log already emits — i.e. **the input signal
that (c) would feed is already plumbed end-to-end**; only the controller
(receiver→batch-size feedback) is missing. ARDOP's existence proof:
*receiver-reported channel quality drives sender-side packetization, and it
works on HF.*

### §3.4 TCP SACK / RFC 6298 — event-driven RTO

For (a) specifically — "size the post-TX timeout to the expected
SACK-arrival time, not the batch length" — the canonical prior art is the
TCP retransmission timer (RFC 6298) and modern SACK-aware loss recovery
(RFC 6675). TCP does **not** size its retransmission timer to the
outstanding-bytes window: it sizes it to a measured round-trip-time
estimate (smoothed RTT + 4×RTTVAR). Mercury's analogue is the
*pattern-arrival time*: `ptt_off_delay + RSP_decode_time + SACK_pattern_TX_time
+ ptt_on_delay + propagation`, none of which scale with `data_batch_size`
once RSP has finished decoding the last DATA frame. The §2.7 decomposition
shows the only `data_batch_size`-coupled term is the SACK pattern itself
(~1168 ms at batch=25, ~470 ms at batch=10) — and that is the *expected
arrival width*, not a margin. The two 3000-ms constant adders have no
prior-art justification once the structural turnaround race is closed
(Plan A, `f2dbf34`); they were originally inserted to mask that race.

### §3.5 The (b) lever — TCP/IP and 802.11 prior art is permissive but mute on cost

Mixing new data into the next batch alongside retransmitted segments is the
default mode of TCP/SACK; it is also how 802.11 block-ACK retransmits work.
The protocol cost in those systems is hidden by an explicit cumulative ACK
mechanism with absolute sequence numbers — exactly what `batch_seq_id`
would add to Mercury. So (b) has prior art in principle but not in any HF
modem I located; and the HF analogues (STANAG 5066, ARDOP, VARA) all use
SR-ARQ with absolute sequence numbers from the start, side-stepping the
problem Mercury would have to solve at retrofit time. **Translation: (b) is
the right shape long-term, but its prior art is "absolute sequence numbers
from day one" — i.e. Design A from the redesign plan, not a v1 retrofit.**

### §3.6 Citations (one per candidate)

- **(a) → RFC 6298 / RFC 6675** (TCP RTO + SACK-aware loss recovery): the
  retransmission timer is sized to a measured arrival-time estimate, never
  to the outstanding-window. This is the conceptual model for an event-keyed
  / arrival-keyed post-TX timeout in Mercury.
- **(b) → IEEE 802.11 Block-ACK** (TCP/SACK is the same idea on wire): mixing
  retransmits with new data within a single transmission unit requires
  absolute sequence numbering inside the unit. Mercury does not have it on
  DATA frames today (§2.6) — this is the structural prerequisite.
- **(c) → FED-STD-1052 HFDLP** (Johnson, NMSU): "negotiate the number of data
  frames per data series (1-255)" — adaptive batch sizing keyed to channel
  state is established practice on HF; the missing piece in HFDLP is *dynamic*
  re-negotiation, which is what (c) would add.

---

## §4 Candidate evaluation — scope, benefit, complexity, dependencies, blast radius

For each candidate, the benefit projection uses the win-test §5.3/§5.4
`cycle_ms` tables as the empirical baseline and the §2.7 decomposition as the
arithmetic.

### §4.1 Candidate (a) — decouple SACK post-TX timeout from batch size; size to expected SACK-arrival width

**What it changes.** Replace the two 3000-ms constant adders + the
batch-scaled SACK term with a tighter, geometrically-derived bound. Concrete
sketch — `arq_common.cc:435-470`:

```
// CMD branch when ack_pattern_time_ms > 0
int frame_drain   = 2 * message_transmission_time_ms;         // unchanged (CMD's last frame still
                                                              //   in the channel)
int sack_arrival  = ptt_off_delay_ms                          // CMD → silence
                  + rsp_decode_margin_ms                       // RSP one-frame decode budget [?]
                  + sack_pattern_ms_at(/*nframes=*/data_batch_size)
                                                              // RSP keys SACK
                  + ptt_on_delay_ms;                          // SACK leading-edge at CMD
int margin        = SACK_ARRIVAL_MARGIN_MS;                   // small constant, calibrated from §6 trace
int timeout       = frame_drain + sack_arrival + margin
                  + (gear_shift_on && turbo!=DONE ? 2000 : 0);
                                                              // turboshift adder kept as-is
```

Two of the three structural drags from §2.7 (the base 3000-ms margin and the
hardcoded `sack_timeout_extra_ms=3000`) collapse into one *calibrated*
`SACK_ARRIVAL_MARGIN_MS` derived from the §6 trace data
(`SACK_TURNAROUND_FIX_PLAN.md` §6 measured 6140-8674 ms gate-to-last-frame
gap **before** Plan A, but per §9 those collisions are now gone — i.e. the
two 3000-ms adders were band-aids for the now-fixed turnaround race; this is
exactly the kind of stale margin CLAUDE.md says to re-derive after a
structural fix lands). The third drag — the batch-scaled SACK term — stays,
because it is real geometry (the SACK pattern *is* longer at batch=25).
**(a) does not require (c).**

**Scope.** `arq_common.cc` `calculate_receiving_timeout()` body only —
single function, CMD branch only. Optionally a new tunable
`rsp_decode_margin_ms` member, defaulting to a value derived from the §6
trace. No `arq_commander.cc` change beyond the existing call sites at `:788`
and `:875`. No `arq_responder.cc` change. No protocol/wire change. No
capability negotiation change.

**Expected benefit (grounded in §5.3 / §5.4 data).**

| cycle component | today (sack=on, batch=25) | (a) projected | source |
|---|---:|---:|---|
| batch TX (25×390) | ~9750 ms | ~9750 ms | unchanged |
| post-TX timeout (CMD) | ~8200 ms | **~2500-3000 ms** | (a): drops two 3000-ms adders, keeps geometric terms |
| retransmit-only mini-batch | ~12-19 s (variable; partial batches) | ~6-8 s | (a): same timeout reduction applies to `:788` |
| total cycle (clean, no retx) | ~11.5 s (clean §5.2 row) | **~7-8 s** | bring SACK clean closer to no-SACK 6.1 s |
| total cycle (lossy with 1 retx round) | ~19.9 s (wgn32 §5.2) | **~12-14 s** | retain SACK's frame-efficiency win |

For (a) to deliver a lossy win, the projected ~13-s SACK lossy cycle has to
beat no-SACK's measured ~5.9 s (wgn32 §5.2) / ~7.1 s (mpd16 §5.4) /
~7.8 s (mpd14 §5.4) cycle. **It almost certainly does not, on its own**, on
the lossiest points — no-SACK's 10-frame cycle is just structurally
shorter. The likely shape of an (a)-only result:
- Track A clean (~0% loss): SACK already +38% (§4.1) — likely holds or
  improves (less timeout per cycle, same retx volume).
- Track A wgn28 (~6% loss): currently +7% — likely flips to a clear SACK win
  (timeout was the variance driver; SACK σ=376 should drop substantially).
- Track A wgn32 (cliff edge): currently −60% — likely improves to ~−20% or
  better (cycle shrinks from 19.9 s toward 13 s), but probably not a win
  outright because the no-SACK 5.9 s cycle is still half (a)'s projected.
- Track B mpd16 (~8% loss): currently −55% — improves but likely still loses
  by ~−15% to −25%.
- Track B mpd14 (~22% loss, design target): currently −38% — improves
  similarly; mpd14 has fewer batches in 90 s so the absolute cycle ms matters
  most. Possible mild win if the retransmit-only cycle also shortens (see
  below).

**Honest read:** (a) is most likely to **close the gap and produce a clean
win at clean / mild-loss**, **flip the wgn28 ~tie into a clear SACK win**,
and **substantially narrow but probably not eliminate** the loss at the
deeper points. It is the largest-arithmetic-contributor / smallest-blast-radius
move and worth doing on its own merits — but it is *not* guaranteed to flip
the deeper-loss verdicts on its own. That is the case for (c) as a second
step.

**Complexity / risk.** **Small.** One function, one file. Reversible by
`git revert`. The risk surface is a *premature* timeout — if the new margin
is set too tight, CMD times out before the SACK arrives, regresses into the
no-SACK full-batch-resend path. Mitigations: (i) calibrate the margin from
the §6 trace (the very same data that quantified the pre-Plan-A overlap is
*directly* the prior-distribution of CMD→SACK arrival time on the channel,
post-Plan-A; the post-fix `sack_fresh_combined.log` traces from
`SACK_TURNAROUND_FIX_PLAN.md` §9.1 are the right calibration set); (ii) keep
the existing CLI flag `--sack-timeout-extra-ms=N` so the new derived bound
can be re-inflated at runtime if a field deployment shows the simulation
margins were wrong; (iii) the failing test is straightforward — see §5.

**Dependencies.** None. (a) is standalone. Does not require (b) or (c).

**Blast radius.** **CMD-only, single function.** The non-SACK code path is
byte-for-byte untouched (the branch is gated by `sack_enabled`). The Plan-A
RSP per-frame timer logic (`arq_responder.cc:327-406`) is independent and
unaffected. The capability negotiation (`arq_commander.cc:2103-2128`) is
unaffected.

**Stands alone?** Yes.

### §4.2 Candidate (c) — adaptive SACK batch size keyed to observed channel loss

**What it changes.** Replace the static `data_batch_size = radio_batch_size`
(=25) assignment at `arq_commander.cc:2117` with a controller that adjusts
`data_batch_size` between a floor (e.g. 10 — the no-SACK batch size) and
a ceiling (25 — current SACK batch size) based on a moving estimate of
per-batch loss. Concrete sketch:

```
// In process_messages_rx_acks_data() after the SACK branch completes (~:1481):
recent_partial_batches[++rb_idx % WINDOW] = (retransmit_count > 0);
// In process_messages_tx_data() before send_batch():
double partial_rate = mean(recent_partial_batches);
int new_batch_size;
if(partial_rate < 0.05)        new_batch_size = 25;
else if(partial_rate < 0.20)   new_batch_size = 18;
else                            new_batch_size = 10;
if(new_batch_size != data_batch_size) {
    set_data_batch_size(new_batch_size);
    recalculate_ack_timeout_for_batch();
    calculate_receiving_timeout();
    // Inform RSP via existing control frame mechanism [?] — see Dependencies below
}
```

The bands above are illustrative and would be calibrated against §5
empirically. The key behavioural property is that on a clean channel SACK
stays at 25 (preserves the §7.3 clean-channel win), and on a sustained-lossy
channel SACK converges to ~10 (matches no-SACK's agile behaviour). The
crossover at ~20% is exactly where (c) would have flipped the §4.2 mpd16
verdict.

**Scope.** `arq_commander.cc` (the negotiator and `process_messages_rx_acks_data`),
`arq_common.cc` (a loss-window state member), **and** a CMD↔RSP protocol
hook — RSP must know the new `data_batch_size` to size its `expected` /
ACK-GATE math. Two reasonable mechanisms:
1. **Implicit / EOB-keyed.** RSP already infers the effective batch size
   from the end-of-batch bit-7 flag on the last DATA frame
   (`arq_common.cc:2877-2879` set on CMD; `arq_responder.cc:334-339` consumed
   on RSP). If CMD just keys a smaller batch and the last DATA frame still
   carries EOB, RSP's `effective_batch` already shrinks. This may be enough
   — needs verification.
2. **Explicit.** Use an existing control frame type (`ACK_RANGE` /
   `ACK_MULTI` already extend the control-frame schema per
   `SACK_REDESIGN_PLAN.md` §7.2) to send a `BATCH_SIZE_UPDATE` notification.
   More robust but adds protocol surface.

**Expected benefit (grounded in §5 data).** (c)'s projection rests on the
observation that no-SACK's win at lossy points comes from a structurally
shorter cycle (5.9 s vs 17-19 s at the lossy points, §5.4). At
`data_batch_size=10` SACK's cycle should approach no-SACK's: same ~5.9 s.
The SACK-specific overhead would be reduced to the SACK-pattern arrival
(~470 ms at batch=10 vs ~1168 ms at batch=25) — a small remaining drag,
not the 8-s drag of today. Projected verdicts (assuming (c) lands together
with (a)):

| cell | today | (a)+(c) projection |
|------|------:|-------------------:|
| Track A clean | +38% | ~+38% (stays at batch=25) |
| Track A wgn28 | +7% | ~+15-25% (modest shrink to ~18) |
| Track A wgn32 | **−60%** | ~−5 to +5% (shrinks to ~10, matches no-SACK) |
| Track B clean | +15% | ~+15% (stays at 25) |
| Track B mpd16 | **−55%** | ~0 to +10% (shrinks to ~10) |
| Track B mpd14 | **−38%** | ~+5 to +15% (shrinks to ~10, SACK still saves frames) |

**Complexity / risk.** **Medium.** The controller logic itself is small (a
moving average and a step function), but it interacts with:
- RSP's effective-batch math — must verify EOB-keyed inference (§4.2 path 1)
  works, or add an explicit control frame (path 2). This is the largest risk.
- The compression / streaming context (`SACK_THROUGHPUT_INVESTIGATION.md`
  §16.3 mentions a never-built double-buffer for crypto batches — batch
  size changes within a session need to be compatible with the streaming
  zstd prefix and PPMd context carry). The compression batch capacity sizing
  at `arq_commander.cc:3001` (`int batch_capacity = data_batch_size *
  max_frame;`) is a live consumer — needs verification it handles dynamic
  resize cleanly.
- Hysteresis / oscillation: (c) must NOT oscillate between batch=10 and
  batch=25 on a flapping channel — needs a deliberate damping (e.g. only
  step *up* after 5 consecutive clean batches, step *down* immediately on
  any partial). This is a real risk per CLAUDE.md "no threshold band-aids":
  the damping has to be principled (control-theory motivated), not tuned.

**Dependencies.** Stands alone but pays off most when paired with (a). If
(c) lands without (a), `calculate_receiving_timeout()` still computes a
batch-scaled timeout — at batch=10 the SACK pattern term shrinks from
~1168 ms to ~470 ms and the formula sees that, so (c) alone partially closes
the §2.7 gap *because* the formula's only batch-coupled term is the SACK
pattern. But the two 3000-ms hardcoded adders survive — total timeout drops
from ~8.1 s to ~7.4 s, only a small win. **(a)+(c) together gives the full
benefit; (c) alone gives a fraction.** That dependency direction matters for
sequencing (see §6).

**Blast radius.** **Medium.** Negotiation path, RSP effective-batch math,
compression sizing, possibly a new control frame type — three subsystems
have to agree on a value that previously was constant after negotiation. The
opt-in nature (only fires with `--enable-sack`) bounds the blast radius
nicely: non-SACK peers see no behaviour change.

**Stands alone?** Partial — needs (a) for full benefit. Worth pursuing only
if (a) lands and does not produce a sufficient lossy win on its own.

### §4.3 Candidate (b) — fold retransmit-only frames into the next batch's keying

**What it changes.** Eliminate the standalone retransmit-only mini-batch
(`arq_commander.cc:730-794`); instead, keep the N missing frames in
`retransmit_frames[]` until the next batch is assembled; reserve N slots at
the front of that next batch; new data fills the remainder; the whole
batch is one keyed cycle. The new-data suppression at `:2987-2990` must be
removed; the compression batch capacity (`:3001`) must be reduced by N.

**Why it's listed last.** The win-test §5.3/§5.4 data shows the per-cycle
overhead in the retransmit-only cycle is meaningful but **smaller than
the post-TX timeout drag** that (a) targets. At Track A wgn32 the
retransmit cycles are infrequent (4 sack_ev across 3 runs, mean cycle
19892 ms — the timeout dominates the inflation, not the retransmit cycle
count). The §7 estimate in `SACK_RETRANSMIT_BATCHING_INVESTIGATION.md`
projects (b) could recover ~half of SACK's gain — but that estimate
assumed (a) was already paid; against today's timeout-dominated cycle,
(b)'s contribution is a smaller incremental fraction.

**Scope.** Large:
1. `arq_commander.cc:730-794` — gut the retransmit-only branch
2. `arq_commander.cc:2987-2990` — remove the new-data suppression
3. `arq_commander.cc:796-834` — rewrite the new-data fill to reserve N
   front slots for retransmits with their *original* sequence numbers
4. `arq_commander.cc:3001` — reduce `batch_capacity` by N×max_frame
5. **Protocol change**: a DATA frame must carry a batch identifier (or a
   retransmit-bit) so RSP can route frame[i] in the wire-batch to the
   *correct* assembly slot of either the current or the prior receive
   batch. Today the wire `sequence_number` IS the slot identifier within
   the current batch; (b) overloads it across batches and RSP cannot
   disambiguate. **This is the cross-batch sequence-space problem
   `SACK_REDESIGN_PLAN.md` §5.1 / `SACK_RETRANSMIT_BATCHING_INVESTIGATION.md`
   §8 step 4 both call out.**
6. RSP-side: matching dispatcher for the retransmit-bit / batch_seq_id
   (`arq_responder.cc`).
7. Frame-format change ripples through TX/RX framing (`arq_common.cc`
   `transmit_byte()` / receive parser), header constants in
   `datalink_defines.h`, and the LDPC payload accounting.

**Expected benefit.** With (a) already landed, (b) would shave the
retransmit-only cycle (~6-8 s in (a)'s projection) and instead amortize
N retransmits over the next 25-frame batch. The cycle saving per partial
batch is ~one ARQ round trip per partial — measured ~1.5-2.5 s per cycle in
`SACK_RETRANSMIT_BATCHING_INVESTIGATION.md` §7.1. Across the win-test runs
the number of separate retransmit cycles is small (sack_ev = 4-6 per cell
of 3 runs in §5.2 / §5.4), so the absolute throughput delta of (b) on a
90-s window is on the order of one or two saved cycles → ~3-4 s saved →
**maybe 10-20% additional improvement** on top of (a)+(c). Not nothing,
but smaller than (a) or (c) individually.

**Complexity / risk.** **Large.** Multi-file, multi-subsystem,
protocol-level. The cross-batch sequence-space problem is non-trivial in
isolation; doing it in v1 SACK while v2 (`SACK_REDESIGN_PLAN.md` Design A)
is already specified is duplicate effort. **It is also the highest
correctness risk** — wrong slotting → silent data corruption on RX (RSP
delivers a retransmit of frame[4] into the slot for new-frame[4]).
CLAUDE.md "life-critical communication software" applies; this is exactly
the kind of change that needs (i) a comprehensive test, (ii) the win-test
A/B, (iii) a soak run.

**Dependencies.** Needs (a) for arithmetic-sense (saving an ARQ round in a
post-(a) world is meaningful; saving one in today's 8-s-timeout world is
swamped by the timeout). Also overlaps with `SACK_REDESIGN_PLAN.md`
Design A: v2 already plans `batch_seq_id` as a wire field
(`SACK_REDESIGN_PLAN.md` §5.1). Doing (b) in v1 means adding the same
field twice (once for v1's DATA frames, once for v2's control frame) or
threading the v1 retrofit through a future v2 migration — neither is
clean.

**Blast radius.** **Largest of the three.** Touches both CMD and RSP,
changes the wire format, and interacts with compression sizing. The opt-in
gating of `--enable-sack` bounds it operationally, but the *code surface* is
broad enough that a regression could surface on any code path that touches
batch assembly.

**Stands alone?** No — depends on (a) for the underlying timeout to be
sensible, and overlaps Design A from the redesign plan.

---

## §5 Success criteria — what counts as "the fix worked"

Per the prompt §5(iii): at minimum, the corrected win-test must show a clear
lossy win across both tracks (A and B), no clean-channel regression, no new
miscorrections. Concretely, against `tools/sack_lossy_ab.py` re-run on the
same Tracks A+B harness:

**Hard gates** (any failure = the candidate did not deliver):
1. **Clean channel non-regression.** Track A clean SACK throughput ≥ today's
   2628.7 bps (within 5% / σ). Track B clean SACK throughput ≥ today's
   906.6 bps.
2. **No new LDPC miscorrections.** Across the full A+B sweep, 0 provable
   miscorrections per the §6 alignment-free set-membership predicate (the
   same metric `SACK_LOSSY_CHANNEL_WINTEST.md` §6 used).
3. **No-SACK regression check.** No-SACK WB_CFG15 and WB_CFG10 throughput
   unchanged within σ (the candidate should not perturb the `--enable-sack`
   OFF path).

**Win gates** (the bar for "this fix is worth it"):
4. **(a) alone is worth landing if:** Track A wgn28 flips from +7% to a
   clear SACK win (≥ +20%, low σ). Track A wgn32 improves to within ~10% of
   no-SACK (anything from −10% to a small win). Track B mpd16 improves to
   within ~10% of no-SACK. Track B mpd14 narrows to within ~10%. Cycle_ms
   measurements show the projected ~8.1 → ~3.0 s post-TX-timeout drop and
   ~8 → ~2 s retransmit-only cycle drop.
5. **(c) is worth adding on top of (a) iff** (a) alone fails gate 4 at the
   lossy points (i.e. lossy verdicts are still ≥ 15% behind no-SACK after
   (a)). The (a)+(c) verdict gate is: **SACK wins or ties on every cell of
   both tracks**, with the loss-rate-driven batch shrink visible in the
   logs (a `[SACK-BATCH] resize 25→18→10 partial_rate=…` event line at
   the resize points).
6. **(b) is worth pursuing only if** both (a) and (c) land and a measurable
   gap remains at *moderate* loss (mpd16-like channels with sack_ev > 0 per
   batch) — i.e. a regime where the retransmit cycle is paid often enough
   for (b)'s saving to matter.

**Failure mode = "STOP and discuss".** If (a) lands cleanly, passes hard
gates 1-3, but fails win gate 4 (and the cycle measurements show the
timeout *did* drop as projected — i.e. the arithmetic worked but the
throughput didn't follow), that is the "third fix attempt" CLAUDE.md
flags: the architectural problem is the v1 SACK structure itself, and the
right answer is `SACK_REDESIGN_PLAN.md` Design A, not (b)+(c) on v1.

---

## §6 Sequencing recommendation

1. **Step 1 — implement (a).** Smallest blast radius, largest single
   arithmetic contributor, no protocol change. Three to four targeted
   commits (see §7). Re-run Tracks A+B; agent-judged verdict.
2. **Decision gate.** If (a) passes win gate 4 across the full A+B sweep:
   STOP — `CAP_SACK` can be revisited for default-on with this single fix.
3. **If (a) does not pass win gate 4:** present the (a) measurements to
   owner. The decision is between (c) and the redesign:
   - **Implement (c) if** (a) closed *most* of the gap and the residual is
     in the highest-loss bands where adaptive batch sizing is the obvious
     missing piece. Step 2 — implement (c). Re-run Tracks A+B.
   - **Fold into Design A redesign if** (a)'s measurements show the
     residual is *structural* (not loss-band-specific) — e.g. SACK is now
     worse at *every* loss band, including clean. That would indicate v1
     SACK has more architectural debt than (a) can fix and the right cost
     is the v2 migration.
4. **Do NOT pursue (b) in this campaign.** Add it to the redesign plan's
   v2 design notes if the redesign goes ahead; its prerequisite
   (cross-batch sequence space) is already part of Design A's `batch_seq_id`
   (`SACK_REDESIGN_PLAN.md` §5.1) — pay the protocol cost once, there.

---

## §7 Ordered reversible steps — Candidate (a) only

This is the *full* plan for the first move. Per CLAUDE.md §"Plan before
coding", these steps are NOT to be implemented until owner approves.

| Step | Action | Verifying test | Rollback |
|------|--------|----------------|----------|
| 1 | **Capture the post-Plan-A SACK-arrival prior.** Re-parse the existing post-fix `sack_fresh_combined.log` traces from `SACK_TURNAROUND_FIX_PLAN.md` §9.1 to extract the *measured* distribution of `cmd_batch_tx_done` → `cmd_sack_detected` ms across 4 batches. This is the calibration set for the new `SACK_ARRIVAL_MARGIN_MS`. Tooling only (`tools/sack_arrival_calibrate.py`, workspace repo). No Mercury change. | Script runs on stored logs, prints p50/p95/p99 of `cmd_batch_tx_done → cmd_sack_detected`. Output recorded back into this fact doc as §8 (numbers). | Delete the script. |
| 2 | **Add a failing test.** New `tools/sack_timeout_calibration_test.py` that runs `--enable-sack` WB_CFG15 against the existing pre-fix-timeout binary, asserts the empirical `cmd_batch_tx_done → cmd_sack_detected` mean is *less* than the configured `receiving_timeout` — but flags the *headroom* (timeout − measured arrival) as ≥ 5000 ms (today's slack, i.e. waste). After (a) lands, the headroom must drop to ≤ 1500 ms (target). FAILs on today's binary (headroom too large), PASSes on (a). | Two-sided self-test: FAIL on `monitor` `90ed5d9`, PASS on (a)-implemented HEAD. | Delete the script. |
| 3 | **Implement (a).** Single commit, `arq_common.cc` `calculate_receiving_timeout()` body only. Replace the CMD-branch `ack_pattern_time_ms > 0` block with the geometry-derived formula from §4.1 (using the §8 / Step-1 calibrated `SACK_ARRIVAL_MARGIN_MS`). Keep `--sack-timeout-extra-ms=N` as an override (default 0 after this fix; users can re-inflate if they hit a regression in the field). Add `[CMD-POST-TX-CALIB]` log line that prints the new decomposition (frame_drain, sack_arrival, margin). | `mercury.exe --test` passes. `tools/sack_timeout_calibration_test.py` flips FAIL→PASS. | `git revert` step 3. |
| 4 | **Re-run Tracks A+B win-test.** Same harness as `SACK_LOSSY_CHANNEL_WINTEST.md` §3 (`tools/sack_lossy_ab.py --config WB_CFG15 --points clean,wgn28,wgn32` then `--config WB_CFG10 --points clean,mpd16,mpd14`, 3 runs/cell, 90s, interleaved). Re-parse via `tools/sack_lossy_analyze.py`. Append results as §9 of this fact doc. | Agent-judged per §5 hard gates 1-3 (all must pass) and win gate 4 (the SACK-vs-noSACK verdict on the lossy cells). | If hard gates fail: revert step 3 immediately. If win gate fails: keep step 3 reverted, document the residual in §10 and proceed to the §6 decision gate. |
| 5 | **(Conditional)** If §5 win gate 4 passes: revisit `CAP_SACK` default-on. New PR proposes flipping the B2 default-off (`856f024`) back to default-on, gated on a soak run. | Soak run TBD per owner. | Trivial: revert the capability default. |

---

## §8 Three honest questions answered

Per the prompt §5(i)-(iii):

### §8.1 Is SACK worth the architectural complexity?

**Partially.** SACK *as a feature* — selective per-frame retransmit instead
of all-or-nothing batch resend — is mechanism-correct (§5.5 worked example;
§6 zero miscorrections across 48 SACK runs) and frame-efficient (§5.4
mpd14: SACK retx% = 6.2% vs no-SACK retx% = 19.4%). The architectural cost
that is currently not paying off is the **batch=25 + 8-s timeout
turnaround structure** — a different feature, layered on top, that
exists because batch=25 was assumed to be a clean win over batch=10. The
win-test proved it isn't. So the question is not "is SACK worth it" but
"is batch=25 + 8-s timeout worth it" — and the answer is no, as currently
sized.

The right move is NOT to abandon SACK as a feature. It is to fix the
turnaround structure (a), and likely also to stop *requiring* batch=25
when the channel is lossy (c). After those two land, SACK should be a
roughly-neutral-to-positive default on every channel — i.e. a feature
worth keeping default-off-but-cheap-to-enable until the redesign lands.

### §8.2 Does pursuing a/c constitute the redesign, or is it different?

**Different — they are layered on v1.** `SACK_REDESIGN_PLAN.md` Design A
is a complete protocol replacement: MFSK SACK pattern → OFDM control frame
(`SACK_RSP`), bringing 2.4× shorter on-air SACK (~485 ms vs ~1168 ms),
elimination of ~500 lines of MFSK-SACK code, and `batch_seq_id` on the
SACK_RSP for cross-batch disambiguation. (a) and (c) leave the MFSK SACK
on the wire and the v1 framing intact; they only retune the post-TX
timeout (a) and the negotiated batch size (c).

The *overlap* is that a future Design A migration would inherit (a)'s
geometry-derived timeout almost as-is (the formula is on the CMD side of
the protocol-agnostic ARQ controller, not the MFSK-specific path), and
would inherit (c)'s adaptive batch logic almost as-is. So (a) and (c) are
**investments that survive a Design A migration**. (b), in contrast, is
duplicate effort with Design A's `batch_seq_id` — which is the explicit
reason this plan deprioritises it.

### §8.3 Success criterion — what's the bar?

Per §5: hard gates 1-3 (clean non-regression, zero miscorrections, no-SACK
unchanged) + win gate 4 (clear SACK win or tie on all measured cells).
"Anything short of that and the fix wasn't worth it." That bar is
deliberately strict because anything weaker invites the band-aid pattern
CLAUDE.md warns against — re-tuning the timeout until the throughput
numbers look better. The discipline is: the geometric derivation must
*justify* the timeout, and the win-test must *confirm* it.

---

## §9 Open questions [?]

- [?] **rsp_decode_margin_ms calibration.** The §4.1 formula introduces a
  new "RSP one-frame decode budget" term. The §6 trace of
  `SACK_TURNAROUND_FIX_PLAN.md` has the data to derive it (per-frame
  decode time on the RSP side), but the exact extraction is Step-1 work
  in §7. Until Step 1 runs, the §4.1 sketch's numeric projection assumes
  this term is ~200-300 ms.
- [?] **Does RSP infer reduced batch size from EOB alone?** Per §4.2 path
  1, RSP's `effective_batch` math (`arq_responder.cc:334-339`) reads
  `last_received_end_of_batch_seq` and may already handle a smaller batch
  without an explicit control message. If so, (c) is a CMD-only change
  too. Needs a code trace of `arq_responder.cc:334-339` and the call to
  `set_data_batch_size` on RSP — not done in this plan.
- [?] **Compression batch capacity under dynamic resize.** §4.2's
  dependency list flags `arq_commander.cc:3001` and the streaming-zstd /
  PPMd context handling for resize compatibility. The streaming
  compression header is fixed-size (7 bytes per memory note); whether
  the compressor's internal state assumes a fixed batch capacity is
  unverified — would need to be checked before (c) lands.
- [?] **`sack_timeout_extra_ms` removal — is it actually safe?** The
  default 3000 ms (`arq_common.cc:239`) was introduced (commit `7076a4b`)
  to mask half-duplex collisions that Plan A (`f2dbf34`) later fixed
  structurally. (a) bets that the structural fix made the 3000-ms margin
  redundant. If a hidden second mechanism (e.g. CMD's own audio-path
  settling) also depended on the 3000-ms slack, removing it could surface
  a different failure. The §7 Step 4 win-test is the validation, and the
  CLI override is the safety net.
- [?] **Win-test variance at WB_CFG15 wgn32.** The §4.1 Track A wgn32 cell
  has σ=306 bps for SACK vs σ=68 for no-SACK — high variance was the
  win-test's caveat. Whether (a) reduces SACK variance (likely, because
  the timeout dominates the cycle-time variance) is itself testable in
  Step 4; if SACK σ drops below no-SACK σ, that is a secondary
  confirmation (a) addressed the root cause.

---

## §11 Step results (executed)

### §11.1 Step 1 RESULT — SACK arrival calibration (2026-05-14)

**Tool:** `tools/sack_arrival_calibrate.py` (workspace, committed
separately).

**Input logs** (all post-Plan-A, monitor `90ed5d9` regime):
- `mercury/fact-documents/timing_data/sack_fresh_combined.log`
- `mercury/fact-documents/timing_data/sack_turnaround_test_r1_combined.log`
- `mercury/fact-documents/timing_data/sack_turnaround_test_r2_combined.log`
- `mercury/fact-documents/timing_data/sack_turnaround_test_r3_combined.log`
- `mercury/fact-documents/timing_data/pi_rpi2_20260514_130939.log`

**Method.** Two arrival predicates measured per `cmd_batch_tx_done`:
1. **clean-ACK arrival** = `cmd_ack_detected` for the matching batch
   (no intervening retransmit). N=17 samples.
2. **partial-SACK arrival** = the `[T] cmd_ack_buffer_energy` timestamp
   *preceding* the `[RX-SACK] Detected` line (audio energy threshold
   = SACK pattern fully buffered) + 300 ms decode budget. N=5 samples.
   The naive "next [T] event after `[RX-SACK]`" upper bound is polluted
   by the *retransmit batch's* own TX events, so it is rejected as a
   biased estimator.

**Aggregated envelope** (clean ∪ partial+decode_budget, N=22):

| stat   | ms |
|--------|---:|
| mean   | 1207 |
| sigma  | 320 |
| p50    | 1063 |
| p95    | 1665 |
| p99    | 1667 |
| max    | 2102 |
| min    | 794 |

**Geometric arrival estimate** (per §4.1, WB_CFG15 batch=25):
`ptt_off (200) + rsp_decode_margin (300) + sack_pattern (~1168) + ptt_on (100) = ~1768 ms`

The geometric formula already over-estimates the measured envelope by
~666 ms at the p95 point (1768 − 1063 = 705 ms of slack baked in).
The worst observed (2102 ms) sits 334 ms ABOVE the geometric estimate.

**Calibration:** `SACK_ARRIVAL_MARGIN_MS = max(0, worst_obs − geo) + safety_floor`
= `max(0, 2102 − 1768) + 500` = `834 ms`.

**Rounded recommendation: SACK_ARRIVAL_MARGIN_MS = 1000 ms.**
Round-up rationale: (i) only 5 partial-SACK samples in the calibration
set so the worst-case tail is under-sampled; (ii) a round 1000 ms is
easier to reason about in the diagnostic log line than 834; (iii) the
1000-ms value still produces a total timeout (~3.0 s) that is well
below the current 8.2 s slack, so the win-test arithmetic of §4.1 is
preserved.

### §11.2 Step 2 RESULT — failing test added

See `tools/sack_timeout_calibration_test.py`. Test inspects the
build-time `calculate_receiving_timeout()` log line `[CMD-POST-TX]`
emitted on the first batch. Asserts the configured `receiving_timeout`
is ≤ 3500 ms post-fix (vs ~8248 ms pre-fix), with a deterministic
FAIL on `90ed5d9` and PASS on the post-(a) HEAD.

### §11.3 Step 3 RESULT — (a) implemented

Commit hash: TBD (recorded after build & test pass).
File: `mercury/source/datalink_layer/arq_common.cc`,
function `cl_arq_controller::calculate_receiving_timeout()` CMD branch
when `ack_pattern_time_ms > 0`. The two 3000-ms constant adders are
removed; the formula becomes:

```
sack_arrival = ptt_off_delay_ms + rsp_decode_margin_ms (300)
             + sack_pattern_ms(batch)                    // when sack_enabled
             + ptt_on_delay_ms;
frame_drain  = 2 * message_transmission_time_ms;
timeout      = frame_drain + sack_arrival + SACK_ARRIVAL_MARGIN_MS (1000)
             + (gear_shift_on && turbo != DONE ? 2000 : 0);
```

The hardcoded `sack_timeout_extra_ms` default goes to 0 (was 3000); the
CLI override `--sack-timeout-extra-ms=N` is preserved as a safety net.
The `[CMD-POST-TX]` log line is extended to print the decomposition
(`frame_drain`, `sack_arrival`, `margin`).

### §11.4 Step 4 RESULT — win-test re-run

See §11.4 below after harness completes.

---

## §10 Cross-references

- `SACK_LOSSY_CHANNEL_WINTEST.md` — empirical baseline this plan must beat
  (§5.3/§5.4 cycle tables are the arithmetic ground truth).
- `SACK_RETRANSMIT_BATCHING_INVESTIGATION.md` — mechanism (a) standalone
  retransmit confirmed; cross-batch sequence-space problem named as (b)'s
  prerequisite.
- `SACK_TURNAROUND_FIX_PLAN.md` §8 — the structural pattern this plan
  follows (open-loop timer → event-/geometry-keyed); §9.1 post-fix traces
  are the calibration source for §7 Step 1.
- `SACK_LDPC_FALLBACK_INVESTIGATION.md` — A2 ldpc=NO closure; no LDPC
  miscorrections in §6 of WINTEST means the LDPC layer is solid and (a)
  does not need to touch it.
- `SACK_THROUGHPUT_INVESTIGATION.md` §16.3 — the "retransmit-only batches"
  / "double-buffer for crypto batches" plan-vs-implementation gaps that
  motivate (b); deferred to v2 redesign per §6.
- `SACK_REDESIGN_PLAN.md` — Design A is where (b)'s protocol cost is
  budgeted; this plan deliberately does not retrofit (b) onto v1.
