# SACK Design A — Multi-Axis Adaptive Gearshift + Protocol Redesign

**Status:** PLAN ONLY (per CLAUDE.md §"Plan before coding"). No Mercury source
edited. Owner review required before any code lands.
**Date:** 2026-05-14. Branch (both repos): `monitor` (mercury HEAD `bdb6e39`,
post-(a)).
**Relationship to prior docs:** EXTENDS `SACK_REDESIGN_PLAN.md` (which remains
the canonical record of v2-SACK-protocol intent). Supersedes that doc's §5–§6
scope only — its §5.1 wire format for `SACK_RSP` is preserved as written; what
this doc adds is the **DATA-frame `batch_seq_id`** and the **three-axis
adaptive controller** that the prior doc did not contemplate. SACK_FIX_PLAN.md
§11.7 named the architectural problem; this doc is the architectural answer.

---

## §1 Why we are here (one paragraph)

Plan-(a) in `SACK_FIX_PLAN.md` shrank CMD's post-TX timeout 8248→3548 ms
(`SACK_FIX_PLAN.md` §11.3) and *did* close two of three lossy verdicts (Track A
wgn32 -59.6 % → -12.5 %, Track B mpd14 -37.5 % → **+25 % SACK win**). It also
**failed the hard win-gate at Track B mpd16** (-39.7 %, only 16 pp closure;
`SACK_FIX_PLAN.md` §11.6/§11.7). That residual is not a timeout-sizing problem
— the timeout *did* drop as projected (cycle 18313→18313 ms is misleading;
post-(a) cycle dropped ~30-35 % yet no-SACK's batch=10 cycle is still 2.5×
shorter at mpd16: 6901 ms vs 18313 ms `SACK_FIX_PLAN.md` §11.6). Per CLAUDE.md
"three consecutive structural fixes signals an architectural problem" we have
hit the trigger (Plan A, A2 ldpc=NO, (a)); per `SACK_FIX_PLAN.md` §11.8 the
right move is **NOT (b) or (c) layered on v1**, but to fold them into the
redesign so the protocol cost (the missing `batch_seq_id`) is paid once.

The empirical lesson, from `SACK_LOSSY_CHANNEL_WINTEST.md` §7.3 read across
both tracks: **SACK's wins are loss-band-conditional and the policy axes are
wrong.** A single 1-D modulation gearshift (today's turboshift /
SUCCESS_BASED_LADDER, `arq_commander.cc:2864-2967`) cannot reach the optimum
on a 3-D throughput surface where loss-band and SACK-reliability are also free
variables. Design A makes the policy axes right.

---

## §2 Code traced — current state and the structural gap

All citations against `monitor` HEAD `bdb6e39` (post-(a)).

### §2.1 Axis-1 today — turboshift / SUCCESS_BASED_LADDER

The only adaptive axis the modem has.

- **Algorithm selector:** `arq_common.cc:203` `gear_shift_algorithm =
  SUCCESS_BASED_LADDER`; `arq_common.cc:205-208` set thresholds
  `gear_shift_up_success_rate_precentage=85`, `gear_shift_down_=55`,
  `gear_shift_block_for_nBlocks_total=5`,
  `gear_shift_down_consecutive_fails=0` (memory: ">55 % failure rate triggers
  gearshift down").
- **Decision site:** `arq_commander.cc:2864-2967` — block-end ladder
  evaluation. Up-shift requires success_rate > 85 % AND
  `gear_shift_blocked_for_nBlocks >= 5` (`:2870`). Down-shift requires success
  rate < 55 % AND three consecutive bad blocks (`:2918-2932`).
- **Observable:** `last_transmission_block_stats.success_rate_data`. With
  ACK-pattern: `100 * nBatches_acked / nBatches_sent` (`:2829-2834`); without:
  `100 * (1 - nReSent/nSent)` (`:2837`). Updated at block boundary.
- **Update cadence:** Once per BLOCK (a fixed set of N batches per
  `BLOCK_END`), so the axis updates **slowly** (≥ 5 blocks before any
  up-shift; ≥ 3 bad blocks before any down-shift).
- **Turboshift state:** `turboshift_phase` ∈ `{TURBO_FORWARD, TURBO_REVERSE,
  TURBO_DONE}` (`arq_common.cc:214-218`). Turboshift is the *initial* probe
  walk that climbs the ladder once at link-up; gearshift then takes over
  during data. `supershift_proven_ceiling` (`arq_commander.cc:2875-2876`)
  remembers the worst attempted config so a future re-up does not blindly
  revisit a known-failing config (the fix for bug #59 per memory).
- **Safe-state invariants today:**
  - Three-bad-block down-shift hysteresis: down-moves are *not* immediate,
    require sustained failure (`:2924`).
  - Block-count gating on up-shift (cooldown ≥ 5 blocks).
  - `supershift_proven_ceiling` prevents re-trying a config that just failed.
  - All config moves are negotiated via `SET_CONFIG` control frame; RSP and
    CMD never diverge silently.

### §2.2 Axis-2 today — batch size (essentially static)

There is no Axis 2 today. Batch size is set ONCE at TEST_CONNECTION
negotiation and never moves:

- `arq_commander.cc:2103-2128` — capability handshake. If both peers advertise
  `CAP_SACK`, `data_batch_size` is set to
  `min(radio_batch_size=25, 12000/msg_tx_ms, nMessages)` via
  `set_data_batch_size(new_batch)` at `:2117`. Without SACK, batch defaults to
  10. Memory: "BATCH-ADAPT is disabled" (the prior CMD/RSP-desync bug;
  `SACK_THROUGHPUT_INVESTIGATION.md` §16.1).
- The runtime `crypto_batch_size` and `retransmit_headroom` members exist
  (`arq_common.cc:129-131`) but are **inert** for batch sizing
  (`SACK_RETRANSMIT_BATCHING_INVESTIGATION.md` §4; only `data_batch_size` is
  consulted at the actual TX site `arq_commander.cc:3001`).
- No per-batch loss observable feeds any controller. A grep across `source/`
  for `batch_loss|recent_loss|loss_history` returns zero hits
  (`SACK_FIX_PLAN.md` §2.3).
- `set_data_batch_size()` body — `arq_common.cc:397-411` — is a plain
  assignment with an upper bound check against
  `max_data_length + max_header_length - ACK_MULTI_ACK_RANGE_HEADER_LENGTH - 1`.
  Safe to call dynamically *if* both peers agree on the new value (the BUG
  the previous BATCH-ADAPT effort tripped on; `SACK_THROUGHPUT_INVESTIGATION.md`
  §16.1).

### §2.3 Axis-3 today — SACK on/off (mode-level only)

- Capability bit `CAP_SACK = 0x20` (`datalink_defines.h:96`). Negotiated by
  TEST_CONNECTION at `arq_commander.cc:2103-2107`: `both_sack = (local & 0x20) &&
  (peer & 0x20); sack_enabled = both_sack;`. Once set at link-up, **never
  changes for the session.**
- Default is `--enable-sack` opt-in (memory: B2 fix `856f024`, after
  `SACK_LOSSY_CHANNEL_WINTEST.md` §8 recommended keep opt-in).
- No SACK-reliability observable. The `[CMD-SACK]` log emits `N/M received`
  on every decode (`SACK_LOSSY_CHANNEL_WINTEST.md` §3.3 ARDOP analogue —
  "the input signal that (c) would feed is already plumbed end-to-end; only
  the controller is missing"), but nothing reads it.

### §2.4 The post-TX timeout site (where (a) installed)

`arq_common.cc:437-506` — `calculate_receiving_timeout()`. CMD branch
(`:441-491`) post-(a):

```
pattern_time = max(ack_pattern_time_ms,
                   ceil(1000 * sack_pattern_passband_samples(data_batch_size)
                              / sampling_frequency))
                   // sack_pattern_passband_samples = ack_mfsk.sack_total_nsymb(nframes)
                   //   * Nofdm * frequency_interpolation_rate
                   //   — telecom_system.cc:3129-3133
frame_drain  = 2 * message_transmission_time_ms
sack_arrival = ptt_off_delay_ms + RSP_DECODE_MARGIN_MS
             + pattern_time + ptt_on_delay_ms
timeout      = frame_drain + sack_arrival + SACK_ARRIVAL_MARGIN_MS
             + (turboshift_active ? 2000 : 0)
             + (sack_enabled ? sack_timeout_extra_ms : 0)   // default 0
```

This is exactly the geometry-derived shape `SACK_TURNAROUND_FIX_PLAN.md` §8.1
specified for the RSP-side per-frame timer (and the same one
`SACK_FIX_PLAN.md` §11.3 ported to the CMD side). Design A inherits it
unmodified.

### §2.5 SACK pattern generation and reception

- **Encode:** `mfsk.cc:610, 650-680` — `encode_sack_bitmap(received, nframes,
  out_tones, sack_ldpc)` — packs the bitmap as LDPC-coded MFSK tones for
  WB M≥16, legacy un-coded otherwise.
- **TX:** `arq_common.cc:3556` — `send_sack_pattern(received_bitmap, nframes)`
  → `telecom_system.cc:3136` →
  `generate_sack_bitmap_pattern_passband(out, received, nframes)`. On-air
  length = `ack_mfsk.sack_total_nsymb(nframes, &sack_ldpc) * Nofdm *
  interp_rate` (`telecom_system.cc:3129-3133`,
  `mfsk.h:92 sack_total_nsymb = ack_pattern_nsymb + sack_bitmap_nsuffix`).
  At WB M=16, `ack_pattern_nsymb=16` + `bitmap_nsuffix=32` = 48 symbols
  ≈ **1168 ms** on-air (`SACK_LATE_SNAPSHOT_BUG.md:89-90`,
  `SACK_REDESIGN_PLAN.md` §1.1).
- **RX:** `arq_common.cc:3738 receive_sack_pattern(out_bitmap, nframes)` —
  ring-buffer poll + LDPC decode with the A2 ldpc=NO fallback
  (`SACK_LDPC_FALLBACK_INVESTIGATION.md` §10.2 — `arq_common.cc:3812-3899`
  region post-fix).
- **CMD consumer:** `arq_commander.cc:1422-1502` — SACK branch of
  `process_messages_rx_acks_data()`; `:1431` calls `receive_sack_pattern()`;
  `:1449-1480` builds the retransmit queue capped at `MAX_RETRANSMIT_HEADROOM=8`
  (`arq.h:179`); `:1483 data_ack_received = YES`.
- **RSP producer:** `arq_responder.cc:843-879` — in the ACK-GATE partial-batch
  branch, when `sack_enabled && rx_received > 0`, build `sack_bitmap[i] =
  (messages_rx[i].status == RECEIVED)` and call `send_sack_pattern(...)`.

### §2.6 The missing protocol piece — no DATA `batch_seq_id`

The wire DATA frame header (`arq_common.cc:2953-2969` for DATA_LONG /
DATA_SHORT, `datalink_defines.h:122-123`):

```
DATA_LONG  header  = [type, connection_id, sequence_number(EOB bit-7), id]  // 4B
DATA_SHORT header  = [type, connection_id, sequence_number(EOB bit-7), id, length]  // 5B
```

`sequence_number` bits 0–6 are the intra-batch slot (0..127); bit 7 is the
End-Of-Batch marker (`arq_common.cc:2948`). **There is no batch identifier.**
Verified by grep across `source/` (`SACK_FIX_PLAN.md` §2.6): `batch_seq_id` =
zero hits; `retransmit_batch_id = -1` exists at `arq_common.cc:136` but is
written-only and never read in a routing decision.

Operational consequence (the structural prerequisite for `SACK_REDESIGN_PLAN.md`
mechanism (b) — folding retransmits into the next new-data batch): RSP slots
a frame by its `sequence_number` within the *current* batch in flight; there
is no way for RSP to disambiguate "this frame is a retransmit of slot 4 of
the *prior* batch" from "this frame is slot 4 of the *new* batch" because
the wire carries no such distinction. **This is the unpaid cost (b) hit.
Design A pays it (§4.2 below).**

### §2.7 Frame format budget — does a new field fit?

`SACK_REDESIGN_PLAN.md` §7.1 already established: at CONFIG_10 WB the smallest
LDPC codeword K = 1000 bits = 125 bytes; control header is 3 bytes
(`datalink_defines.h:121`), leaving 122 bytes payload — easily enough for a
2-byte `batch_seq_id` plus the 8-byte SACK_RSP payload. The cost on DATA
frames is +1 byte per frame (the smallest viable batch_seq_id is 1 byte ≡
256 batches mod-wraparound, more than enough since at most one batch is
outstanding at any time in this half-duplex protocol). 1 byte × 25 frames =
25 bytes / batch ≈ 0.2 % overhead on a 25-frame CONFIG_15 batch (~12000
bytes). **Cost is negligible.**

---

## §3 Prior art (citations, one per design choice)

Per CLAUDE.md "Research before implementing": research-then-cite, not guess.

### §3.1 The three-axis architecture pattern

The closest published analogue to Design A's multi-axis decomposition is the
**inner-loop / outer-loop link adaptation** (ILLA + OLLA) pattern used in
LTE/5G NR cellular link adaptation. ILLA selects MCS from estimated SNR
(fast, no feedback); OLLA adjusts an offset on the SNR estimate based on
observed BLER (slow, feedback-driven). The
[Wikipedia / Sionna treatment](https://nvlabs.github.io/sionna/sys/tutorials/LinkAdaptation.html)
of OLLA: "OLLA basically modifies the measured SNR by an offset, according to
whether data packets are received correctly or not, in order to adjust the
average block error rate (aBLER) to a target." And the
[Springer eOLLA paper](https://link.springer.com/article/10.1186/s13638-016-0518-3)
documents the standard pattern: a *fast* inner loop tracks instantaneous
channel state (∆SNR-driven), a *slow* outer loop trims a bias on the inner
loop's decision based on aggregated outcome statistics (BLER over a window).
**This is exactly the asymmetric-responsiveness pattern Design A needs.**
Mapped to Mercury:

- **Inner loop / fast** = Axis 1 modulation (turboshift). Observable:
  per-batch success rate. Cadence: per-block.
- **Outer loop / slow** = Axis 2 batch size + Axis 3 SACK on/off. Observable:
  windowed loss history (Axis 2) + windowed SACK-decode reliability (Axis 3).
  Cadence: per-N-blocks.

The hysteresis discipline this prior art establishes —
[US 2013/0310091 patent](https://patents.google.com/patent/US20130310091)
"relative throughput variation is compared to a hysteresis threshold" — is
the standard control-theoretic damping; we apply it on every axis (§4.3).

### §3.2 The SACK_RSP frame format

`SACK_REDESIGN_PLAN.md` §5.1 already specified an 8-byte SACK_RSP control
frame (type byte + 2-byte `batch_seq_id` + 4-byte bitmap-25 + 1-byte CRC8).
The wire-shape model is **IEEE 802.11n/ac Compressed Block ACK**: a single
control frame carrying a 64-bit bitmap acknowledging up to 64 outstanding
MPDUs, anchored to a Starting Sequence Number
([Wikipedia / Hitch-Hiker's Guide](http://www.hitchhikersguidetolearning.com/2017/09/17/block-ack-frame-formats-block-ack-request/),
[Wireless On The Go](https://wirelessonthego.postach.io/post/802-11n-block-acknowledgement)).
Each bit of the bitmap = one MPDU; SN(bit i) = SSN + i. The 802.11 design
also defines a separate **Block ACK Request (BAR)** that an initiator can use
to poll the responder for a missing BA — the analogue of
`SACK_REDESIGN_PLAN.md` §3.3 Design C (CMD-requested SACK). Adopted *parts*:
the absolute-sequence-number anchor (our `batch_seq_id`); the compressed
single-bit-per-frame bitmap; the explicit type byte. Rejected: the request/
response polling (adds an extra half-duplex round trip per partial batch;
`SACK_REDESIGN_PLAN.md` §3.3 measured it at ~25 % throughput hit).

### §3.3 The DATA `batch_seq_id` field

**IEEE 802.11 sequence numbering inside a TID** is the cleanest prior art:
every MPDU carries an absolute sequence number, so the Block ACK Request
anchors a bitmap against an absolute Starting Sequence Number and there is
no cross-batch slot ambiguity. **TCP/SACK** is the same idea on TCP — each
segment carries its own absolute byte sequence; SACK blocks reference those
absolute ranges (RFC 6675,
[RFC 6675 § state machine](https://datatracker.ietf.org/doc/html/rfc6675)).
The lesson: every transport that does selective retransmit AND wants to mix
retransmits into a new transmission unit needs absolute sequence numbering;
none of the systems that succeed at this do it with intra-batch-only slots.
**Mercury's intra-batch-only slot is the design debt; Design A pays it down
by adding a 1-byte `batch_seq_id` to DATA headers.**

### §3.4 Axis-1 observable (modulation)

Per-batch success rate is what Mercury uses today (`arq_commander.cc:2829-2842`).
The cellular equivalent is **post-decode BLER as the OLLA driver** — the
[telecomhall OLLA writeup](https://www.telecomhall.net/t/working-of-outer-loop-link-adaptation-olla-based-on-bler-in-5g/22420)
and [Springer eOLLA paper](https://link.springer.com/article/10.1186/s13638-016-0518-3)
both establish post-decode BLER as the right outer-loop signal (not the raw
CSI / SNR estimate, which is noisy). The Mercury `nBatches_acked /
nBatches_sent` ratio is the exact analogue. **Adopted as-is.** The single
caveat — `SACK_LOSSY_CHANNEL_WINTEST.md` §5.3 shows that the
all-or-nothing pattern-ACK pathology makes the *no-SACK* batch ACK ratio
binary per batch — does not apply once Design A is in place, because
post-A.1 every batch produces a SACK_RSP control frame whose bitmap gives a
true partial-batch loss readout.

### §3.5 Axis-2 observable (batch size)

The HF-specific prior art is
[FED-STD-1052 / HFDLP (Johnson, NMSU)](http://wireless.nmsu.edu/hf/papers/hfdlp_throughput.pdf):
"the number of data bytes per data frame (56-1023) and the number of data
frames per data series (1-255)" negotiated at connection setup as a function
of measured channel quality. The "missing piece" the SACK_FIX_PLAN §3.2
identifies — *dynamic* re-negotiation mid-session as channel evolves —
is what Design A's Axis 2 adds. Validation that this is the right shape:
the [Isode WBHF whitepaper](https://isode.com/whitepapers/stanag-5066-for-hf-and-wbhf.html)
explicitly calls out STANAG 5066's fixed-128 window as a *failure* at WBHF
rates — i.e. the negative case for static windows on adaptive HF.

The cellular analogue: **HARQ-CBG / TBG (Code Block Group / Transport Block
Group) sizing** as in 5G NR; batch sizing is adapted per UE per TTI based on
recent error history
([Adaptive HARQ A-HARQ MDPI](https://www.mdpi.com/2079-9292/12/19/4127)).

### §3.6 Axis-3 observable (SACK on/off and reliability)

The closest analogue is **ARDOP's 5-bit decode-quality field in every ACK/NAK**
([ARDOP specification](https://winlink.org/sites/default/files/downloads/_ardop_specification.pdf),
[TAPR DCC 2015](https://files.tapr.org/meetings/DCC_2015/DCC2015-ARDOP-KN6KB-N8OHU-GM8BPQ.pdf)).
ARDOP uses the receiver-reported decode quality (range 38-100 in steps of 2)
to drive sender-side frame-length adaptation — receiver-quality-driven
sender adaptation works on HF. The Mercury analogue is: count successful
`[CMD-SACK] Detected (ldpc=YES, metric=X)` decodes vs missed/`ldpc=NO`/stale
decodes over a sliding window. SACK is *kept on* while the SACK control
frame itself is decoding reliably; SACK is *backed off* when control-frame
reliability drops below a threshold (because at that point Axis 1 should
take over and Axis 2 should shrink batch).

### §3.7 Hysteresis / no-oscillation discipline

[Wikipedia link-adaptation](https://en.wikipedia.org/wiki/Link_adaptation) +
the [US 2013/0310091 patent](https://patents.google.com/patent/US20130310091)
on outer-loop hysteresis establish the standard control-theoretic pattern:
**different thresholds for up-moves vs down-moves; counted-event gating
before any move; cooldown after a move.** Mercury's existing
SUCCESS_BASED_LADDER already does this for Axis 1 (different thresholds 85 %
up / 55 % down, 5-block up-cooldown, 3-bad-block down-gate;
`arq_commander.cc:2864-2967`). Design A applies the same discipline to
Axes 2 and 3, with axis-specific time constants (§4.3 below).

### §3.8 Initiator-controls-flow

Memory `feedback_initiator_control` and CLAUDE.md "RSP never acts
independently". The architectural analogue: in 802.11 Block ACK the
Originator (transmitter) drives the BA agreement and the Recipient never
unilaterally changes parameters. Design A keeps Mercury aligned with this:
**every axis move is a CMD decision communicated to RSP via existing control
frames (`SET_CONFIG` for Axis 1; new `SET_BATCH_SIZE` for Axis 2; new
`SET_SACK_MODE` for Axis 3 — or fold both new operations into a single
`SET_LINK_PARAMS` per §4.4)**. RSP observes and applies; never decides
unilaterally.

---

## §4 Design A — two intertwined pieces

### §4.1 Naming

To avoid colliding with `SACK_REDESIGN_PLAN.md` §3.1 (which named *that*
doc's protocol-replacement plan "Design A"), this doc uses:

- **A.1** = SACK protocol redesign (preserves `SACK_REDESIGN_PLAN.md` §5.1
  SACK_RSP frame + adds DATA `batch_seq_id` per §2.6 above).
- **A.2** = Multi-axis adaptive gearshift architecture (new in this doc).

A.1 is the protocol cost; A.2 is the policy that exploits it. The two are
intertwined: A.2 cannot run without A.1's batch_seq_id (because Axis 2's
batch shrinks would otherwise create cross-batch sequence collisions), and
A.1's OFDM SACK_RSP would not pay off without A.2's adaptive batch size
(because the wins in `SACK_FIX_PLAN.md` §11 are loss-band-conditional).

### §4.2 A.1 — SACK protocol redesign

#### §4.2.1 DATA frame format change

Add 1 byte `batch_seq_id` to **DATA_LONG** and **DATA_SHORT** headers. The
header lengths in `datalink_defines.h:122-123` change:

```
OLD: DATA_LONG_HEADER_LENGTH  = 4   [type, conn_id, seq_num(EOB bit7), id]
NEW: DATA_LONG_HEADER_LENGTH  = 5   [type, conn_id, seq_num(EOB bit7), batch_seq_id, id]

OLD: DATA_SHORT_HEADER_LENGTH = 5   [type, conn_id, seq_num(EOB bit7), id, length]
NEW: DATA_SHORT_HEADER_LENGTH = 6   [type, conn_id, seq_num(EOB bit7), batch_seq_id, id, length]
```

`batch_seq_id` is a 1-byte counter, mod 256, incremented by CMD at the start
of every new-data batch. It is **never** incremented for retransmit-only
content (retransmit frames carry the *original* `batch_seq_id` they were
first sent under, regardless of which batch they ride in this time).

Cost: +1 byte/frame on the wire = 0.2 % at CONFIG_15 batch=25. Verified to
fit in single OFDM LDPC codeword at every config per `SACK_REDESIGN_PLAN.md`
§7.1.

#### §4.2.2 SACK_RSP control frame format (unchanged from SACK_REDESIGN_PLAN.md §5.1)

```
+--------+----------------+----------------+--------+
| type   | batch_seq_id   | bitmap (N b)   | CRC8   |
| 1 byte | 1 byte         | ceil(N/8) byte | 1 byte |
+--------+----------------+----------------+--------+
```

Adjustments vs `SACK_REDESIGN_PLAN.md` §5.1:
- `batch_seq_id` is **1 byte** (was 2) — mod-256 wraparound is sufficient
  because only one batch is outstanding on a half-duplex link.
- bitmap is `ceil(N/8)` bytes where N = `data_batch_size` at TX time (was
  hardcoded 4 bytes = 25 bits). Allows Axis 2 to shrink batch to 10 (2
  bytes), to grow to 50 (7 bytes), with the same frame format. **Self-
  describing length: the SACK_RSP frame's payload length tells RX exactly
  how many bits the bitmap carries.** (RX must already verify against the
  expected `batch_seq_id`'s outstanding `data_batch_size`.)
- type byte: pick the next free constant in `datalink_defines.h:61-89`;
  proposed `SACK_RSP = 0x42` (after `KEY_ACTIVATE = 0x41`).
- CRC8 polynomial: reuse the existing `POLY_CRC8 = 0xF4` at
  `datalink_defines.h:131`.

Total payload at batch=25: 1+1+4+1 = **7 bytes**. At batch=10: 1+1+2+1 = **5
bytes**. Compared to today's 1168-ms MFSK SACK pattern (48 symbols at WB M=16),
the OFDM control-frame TX time is the smallest LDPC codeword's airtime —
~ 390 ms at WB_CFG10 / 16, per `SACK_REDESIGN_PLAN.md` §3.1 ("2.4× shorter on
the wire"). **Direct on-air time saving per partial batch: ~700 ms.**

#### §4.2.3 RSP slotting in mixed-content batches (the cross-batch sequence-space fix)

`arq_responder.cc` slots an incoming frame by its `sequence_number` within
the *current* batch in flight. With `batch_seq_id` on the wire, RSP changes:

```
on RX of DATA frame F:
  if F.batch_seq_id == current_batch.batch_seq_id:
      slot[F.sequence_number] = F  // current-batch new-data slot
  elif F.batch_seq_id == prev_batch.batch_seq_id and prev_batch.outstanding:
      prev_batch.slot[F.sequence_number] = F  // retransmit into prior batch
  else:
      discard with [RSP-RETX-STALE batch_seq=X expected=Y] log
```

This is the routing rule that lets retransmits ride a *new* keyed batch
(mechanism (b) from `SACK_RETRANSMIT_BATCHING_INVESTIGATION.md`) without
mis-slotting. **It is the protocol prerequisite (b) needed and that v1 did
not provide.**

#### §4.2.4 Capability negotiation

Add new capability bit:

```
#define CAP_SACK_V2 0x40  // OFDM SACK_RSP + DATA batch_seq_id + multi-axis gearshift
```

Both `CAP_SACK = 0x20` (v1, MFSK SACK pattern, no batch_seq_id) and
`CAP_SACK_V2 = 0x40` (Design A) coexist for a migration window. Negotiation
in `arq_commander.cc:2103-2128`:

```
both_v2 = (local & CAP_SACK_V2) && (peer & CAP_SACK_V2);
both_v1 = (local & CAP_SACK)    && (peer & CAP_SACK);
if(both_v2) {
    sack_v2_enabled = true; sack_enabled = true;        // both paths share sack_enabled
    // adopt new DATA header lengths; A.2 controllers active
}
else if(both_v1) {
    sack_v2_enabled = false; sack_enabled = true;       // legacy MFSK SACK path
}
else {
    sack_v2_enabled = false; sack_enabled = false;      // pure pattern-ACK
}
```

**Critical lesson from `SACK_REDESIGN_PLAN.md` §9 (the failed v2 attempt).**
The 0x40 capability bit was previously observed to regress throughput even
when its gated code path was dead. Hypothesis: byte-level interaction with
the control-message length / LDPC interleaving. **Mitigations Design A must
take:**
1. Capability byte must remain a single byte. 0x40 + 0x20 = 0x60 leaves three
   spare bits — fine.
2. The control frame carrying capabilities (TEST_CONNECTION) must be
   bit-identical between v1-only peers and v1+v2 peers EXCEPT the literal
   byte value. No length change, no header change.
3. The deterministic-harness fix recommended by `SACK_REDESIGN_PLAN.md`
   §9-next-steps (a WAV-based loopback A/B with no Pi-testbed variance) is
   **a prerequisite** for landing this — not an afterthought. See §7 step 0.

#### §4.2.5 What gets deleted (legacy MFSK SACK)

Per `SACK_REDESIGN_PLAN.md` §5.4 — deferred until v2 is field-validated:
- `mfsk.cc:610, 650-680` `encode_sack_bitmap()` and its helpers
- `arq_common.cc:3556` `send_sack_pattern()`
- `arq_common.cc:3738` `receive_sack_pattern()` and the entire `:3812-3899`
  fallback region
- `telecom_system.cc:3129-3133, 3136, 3264` SACK pattern TX/RX helpers and
  `sack_ldpc` instance
- `include/physical_layer/mercury_sack_{2,4}_16.{h,cc}` LDPC codes

Migration: keep v1 paths active behind `!sack_v2_enabled && sack_enabled`
for one release; delete once `CAP_SACK_V2` is universal.

### §4.3 A.2 — Multi-axis adaptive gearshift architecture

#### §4.3.1 State-space

```
LinkPolicy {
    int    config;          // Axis 1: CONFIG_0..16 / ROBUST_0..2  — already exists
    int    batch_size;      // Axis 2: range [10, 50] step 5    — Design A
    enum   sack_mode;       // Axis 3: SACK_OFF | SACK_ON | SACK_PROBE  — Design A
}

// Observables (rolling windows; all CMD-side)
LinkObservations {
    Ring<float, 8>  recent_batch_success;     // for Axis 1, per-block
    Ring<float, 5>  recent_partial_rate;      // for Axis 2, per-batch (fraction of frames lost)
    Ring<bool, 10>  recent_sack_ok;           // for Axis 3, per-SACK-event (decoded vs missed/stale)
    int             consecutive_sack_misses;  // Axis 3 fast-out
}
```

State-space size: 17 (configs) × 9 (batch sizes 10..50) × 3 (sack modes) =
**459 states.** Compared to today's 17. The architectural risk is the
state-space *exploration* on a transient channel — addressed by the
hysteresis discipline in §4.3.3.

#### §4.3.2 Per-axis controller specification

| Axis | Observable | Update cadence | Action | Hysteresis |
|------|------------|----------------|--------|------------|
| **1 — modulation** | `block_success_rate = nBatches_acked/nBatches_sent` (existing, `arq_commander.cc:2829-2834`) | per **block** (every N-batch group, where N is the existing block size) | `config_ladder_{up,down}()` (existing) | up: `>85 %` AND `>=5 blocks` cooldown; down: `<55 %` AND `>=3 consecutive bad blocks` (existing) |
| **2 — batch size** | `mean(recent_partial_rate)` over rolling window of 5 batches; partial_rate per batch = `frames_lost / batch_size` extracted from SACK_RSP bitmap | per **batch** (every SACK_RSP receipt, post-A.1) | step batch_size up/down by 5 (clamped [10, 50]) | up: mean partial_rate `< 0.05` AND `>=8 consecutive good batches`; down: mean partial_rate `> 0.20` AND `>=3 consecutive bad batches`; reset counters on any move |
| **3 — SACK mode** | `sack_ok_rate = mean(recent_sack_ok)` over 10 most recent SACK events; AND `consecutive_sack_misses` | per **SACK event** (every time CMD attempts a SACK_RSP decode at the expected window) | three-state: ON ↔ PROBE ↔ OFF | ON→PROBE: `consecutive_sack_misses >= 3`; PROBE→ON: `next_sack_ok == true`; PROBE→OFF: `consecutive_sack_misses >= 5`; OFF→PROBE: every 20 batches; PROBE→ON: any successful SACK in PROBE state |

The "axis-specific time constants" map directly to the inner/outer-loop
pattern (§3.1): Axis 1 is per-block (slowest); Axis 2 is per-batch (medium);
Axis 3 is per-SACK-event (fastest decisions inside SACK_OFF/SACK_PROBE
flips, but the windowed observable for ON↔PROBE is medium-cadence). All
three axes are CMD-decided per the initiator-controls-flow rule (§3.8).

#### §4.3.3 Hysteresis discipline (no oscillation)

For each axis, the move trigger is **counted-event** (not single-event) and
the up/down thresholds are **non-overlapping**. Specifically:

- **Axis 2 up-threshold 0.05, down-threshold 0.20** — a 4× ratio between
  the two ensures a batch-size move never reverses itself within one or two
  cycles. A channel sitting at 10 % partial loss does not toggle.
- **Axis 3 ON→PROBE on 3 consecutive misses, but PROBE→OFF only on 5 total
  misses** — fast detection of a SACK decode collapse, slower full-off
  (because the cost of a bad SACK_OFF is an ARQ stall, not just a wasted
  cycle).
- **A move on a "fast" axis resets counters on a "slow" axis.** If Axis 1
  moves config down (channel got worse), Axis 2's partial-rate window is
  invalidated (the loss pattern changes when config changes), so Axis 2
  enters a 3-batch cooldown before its next decision. Implementation:
  `axis2_cooldown_batches = 3` after any Axis 1 move; ignore observations
  inside the cooldown.

#### §4.3.4 Safe-state invariants (life-critical comms)

Per CLAUDE.md "life-critical communication software." Design A must remain
*correct* under every transient. Invariants:

1. **One outstanding batch.** A retransmit can only ride a new batch if at
   most one prior batch is in flight; if two outstanding batches existed,
   `batch_seq_id` would not disambiguate. Invariant: CMD must `WAIT` for
   the SACK_RSP or full-ACK of batch N before keying batch N+1. Enforced
   today by `data_ack_received` gating (`arq_commander.cc:1408`); preserved
   in Design A.
2. **batch_seq_id monotonicity.** `batch_seq_id` increments by 1 mod 256
   per new-data batch. Retransmits carry their *original* `batch_seq_id`.
   On any `set_data_batch_size()` move, `batch_seq_id` is **NOT** reset
   (the field is independent of batch size). Tested by
   `tools/batch_seq_id_test.py` (§5).
3. **No silent corruption.** RSP discards any DATA frame with a
   `batch_seq_id` it has never seen as "outstanding" or "prior", with a
   `[RSP-RETX-STALE]` log line. CMD discards any SACK_RSP whose
   `batch_seq_id` is not the one it just transmitted.
4. **Bounded recovery time on single-axis failure.** If Axis 2 thinks the
   batch should be 50 but RSP doesn't agree (control-frame loss during a
   `SET_BATCH_SIZE` exchange), the EOB bit-7 on the last DATA frame is the
   ground truth: RSP infers the actual transmitted batch size from the EOB
   (`arq_responder.cc:811-815`). So a missed `SET_BATCH_SIZE` causes at
   most one batch's worth of "wrong-batch-size-on-RSP", self-correcting on
   the next batch's EOB. This is the safety net that the BATCH-ADAPT
   desync bug (`SACK_THROUGHPUT_INVESTIGATION.md` §16.1) lacked.
5. **Reversibility of any single policy move.** Every move emits a
   `[POLICY-MOVE axis=X from=Y to=Z reason=R]` log line. Every move is
   observed in the existing block-boundary BLOCK_END statistics. No move
   is silent.
6. **Axis 1 supremacy.** If Axis 1 decides "drop to ROBUST_0 / BREAK", that
   decision overrides any in-flight Axis 2 / Axis 3 move. Implementation:
   on `gear_shift_algorithm` config change at `arq_commander.cc:2950`,
   immediately reset batch_size to a known-safe `radio_batch_size_floor =
   10` and SACK mode to PROBE. Sequencing matters: BREAK should not
   simultaneously trigger an Axis 2 grow.
7. **`supershift_proven_ceiling` analogue for Axis 2.** Mirror the
   `arq_commander.cc:2875-2876` ceiling discipline: if batch_size = K just
   produced a `>20 %` partial rate, that K becomes
   `batch_size_proven_ceiling` for some recovery period (e.g., 20
   subsequent batches), preventing the batch_size from immediately
   re-climbing into the same failure regime.

#### §4.3.5 Correctness budget

The "correctness budget" Design A must respect, in line with
`SACK_LOSSY_CHANNEL_WINTEST.md` §6 (zero LDPC miscorrections across 48 SACK
runs is the standard to keep):

- **0 silent corruption events.** Every wrong-slot routing must be detected
  by `batch_seq_id` check and logged. (Verified by `tools/batch_seq_id_test.py`
  with simulated frame loss and reordering.)
- **0 permanent stalls.** Any single policy move + single subsequent
  control-frame loss must recover within 1 block (≤ `block_size`-batch
  cycles). Verified by the existing link-timeout safety net and the
  EOB-derived RSP batch-size inference (§4.3.4 invariant 4).
- **Bounded recovery on Axis-1 BREAK.** Today's BREAK falls back to
  ROBUST_0 within ~3 NAck cycles. Design A must preserve this (Axis 1
  supremacy invariant §4.3.4 #6).
- **Clean-channel non-regression.** Design A must not regress the
  `SACK_LOSSY_CHANNEL_WINTEST.md` §5.2 clean SACK throughputs (Track A:
  2628 bps, Track B: 906 bps). Validated by §5 below.

### §4.4 The new control-frame surface

To keep the wire change minimal, Design A piggybacks every policy move onto
**existing** control-frame types where possible:

- **Axis 1 move** — `SET_CONFIG = 0x3B` (existing). No change.
- **Axis 2 + Axis 3 move** — proposed new control:
  `SET_LINK_PARAMS = 0x43`. Payload (3 bytes):
  ```
  +--------+--------+--------+
  | batch  | sack   | CRC8   |
  | 1 byte | 1 byte | 1 byte |
  +--------+--------+--------+
  ```
  `batch` is the new `data_batch_size` (clamped to [10, 50]). `sack`
  encodes the new Axis 3 mode (0=OFF, 1=ON, 2=PROBE). CRC8 over the two
  bytes using `POLY_CRC8`.

Alternative considered: bake Axis 2 / Axis 3 into the *next* TEST_CONNECTION
exchange (peers re-negotiate every N blocks). Rejected: TEST_CONNECTION is
heavyweight (full re-handshake, CAP negotiation; `arq_commander.cc:2103-2128`).
A lightweight `SET_LINK_PARAMS` keeps mid-session adaptation cheap.

**RSP-side response on receipt of SET_LINK_PARAMS:** apply the change
**immediately** (atomically — `set_data_batch_size()` is safe to call live
per §2.2), ACK with the existing control-ACK mechanism, and emit
`[RSP-POLICY-APPLIED batch=X sack=Y]` log line for diagnosis. On CRC8
failure, ignore the control frame and let CMD time out → re-send. No half-
applied state.

---

## §5 Validation framework — success criteria *before* designing

Per the prompt §5: define success *before* designing the implementation. The
A/B win-test harness from `SACK_LOSSY_CHANNEL_WINTEST.md` §3
(`tools/sack_lossy_ab.py` at workspace `626c3f9`, analyzer `91f24e4`) is the
reference. The full grid:

| Track | Config | Channel points | Per-cell runs | Duration |
|-------|--------|----------------|---------------|----------|
| A   | WB_CFG15 | clean, wgn32, wgn28 | 3 (5 if σ > 15 %) | 90 s |
| B   | WB_CFG10 | clean, mpd16, mpd14 | 3 (5 if σ > 15 %) | 90 s |
| C (NEW) | WB_CFG10 | mpd16+CFO-50Hz, mpd14+CFO-50Hz | 3 | 90 s |

Track C is a new "freq-drift + multipath" cell to confirm the multi-axis
policy is robust against the kind of channel evolution that triggers Axis 1
+ Axis 2 moves in the same window (the §4.3.3 reset-counters-on-faster-axis
invariant test).

### §5.1 Hard gates (any failure = Design A does NOT ship)

1. **Clean-channel non-regression.** Track A clean SACK_V2 ≥ 2628.7 bps
   (within 5 %); Track B clean SACK_V2 ≥ 906.6 bps (within 5 %). Floors are
   from `SACK_LOSSY_CHANNEL_WINTEST.md` §4.1, §4.2.
2. **0 LDPC miscorrections.** Across all 36+ runs, 0 provable miscorrections
   per `SACK_LOSSY_CHANNEL_WINTEST.md` §6 alignment-free set-membership
   predicate. **Plus** 0 batch_seq_id mis-slottings (a new class — verified
   by `tools/batch_seq_id_test.py`).
3. **0 permanent stalls.** No run exhibits a > 30 s gap between any two
   successful batch deliveries (loose 4× of today's worst observed).
4. **No-SACK regression check.** No-SACK WB_CFG15 and WB_CFG10 throughput
   unchanged within σ. (Design A's wire changes must not perturb the
   `!sack_enabled` path. Validation: a peer that does not advertise
   `CAP_SACK_V2` must see byte-identical behavior, including the unchanged
   4-byte DATA_LONG header.)
5. **CAP_SACK_V2 bit-set non-regression** (the §4.2.4 lesson from
   `SACK_REDESIGN_PLAN.md` §9). When `local & CAP_SACK_V2` is set but the
   peer does not advertise it, throughput must match no-v2 within σ.

### §5.2 Win gates (the bar for "ship it")

6. **Track A wgn32 (cliff-edge AWGN).** Design A's SACK_V2 ≥ no-SACK
   throughput. The post-(a) result was -12.5 %; A.2's adaptive batch
   shrink should converge SACK toward batch=10 at this cell, matching
   no-SACK's agile mode. Bar: **SACK_V2 ≥ no-SACK − 5 %**.
7. **Track B mpd16 (moderate multipath).** Design A's SACK_V2 ≥ no-SACK.
   Post-(a) result: -39.7 %. With A.2 shrinking batch and A.1's shorter
   OFDM SACK_RSP, the cycle should drop from 18313 ms toward no-SACK's
   6901 ms. Bar: **SACK_V2 ≥ no-SACK − 5 %**.
8. **Track B mpd14 (deep multipath, SACK design target).** Design A's
   SACK_V2 ≥ no-SACK + 10 %. Post-(a) result already wins by +25 %
   (`SACK_FIX_PLAN.md` §11.6); Design A should hold this and improve
   slightly (shorter SACK_RSP airtime, plus retransmit-into-next-batch
   eliminates the standalone retransmit cycle).
9. **No clean-channel back-pressure.** Track A clean SACK_V2 ≥ 2628.7 bps
   (matches gate #1); Track B clean ≥ 906.6 bps (matches gate #1). On a
   clean channel, A.2 keeps batch_size at its ceiling, A.3 keeps SACK ON,
   so the *idle* policy is identical to today's `--enable-sack` —
   regression here would indicate a coding bug, not a policy issue.

### §5.3 Observability gates

10. Every policy move is logged via `[POLICY-MOVE axis=X from=Y to=Z
    reason=R]`. The validator counts moves per run and asserts no axis
    oscillates more than 5×/90s. Oscillation = a move followed by its
    inverse within < 2× the axis cooldown.
11. The win-test analyzer emits a `[POLICY-SUMMARY]` per-cell with
    fraction-of-batches at each (config, batch_size, sack_mode) tuple.
    Acceptance: at clean cells, > 95 % at the policy "ceiling" tuple
    (highest config, batch_size=50, sack=ON); at mpd14, the policy
    converges to (CFG10, batch_size=10..15, sack=PROBE-or-OFF) > 50 % of
    the run.

### §5.4 STOP-and-discuss trigger

If hard gates 1-5 fail: **revert immediately, return to pre-Design-A
HEAD.** If hard gates pass but win gates 6-9 fail (the same shape as
`SACK_FIX_PLAN.md` §11.7), that is the **fourth** consecutive structural
fix in this campaign — STOP per CLAUDE.md. The implication is that v1
SACK's batch-25-was-always-too-big assumption is even more wrong than
suspected, and the right next conversation is whether to abandon SACK
entirely (keep the codepath compiled, default off, document as
experimental).

---

## §6 Reversible-step implementation sketch — ordered, smallest-first

DO NOT implement until owner approves. Each step has action / verifying
test / rollback. All steps land on Mercury `monitor`; workspace harness
changes land on workspace `monitor`. Steps that cannot be made reversible
are flagged.

| # | Action | Verifying test | Rollback |
|---|--------|----------------|----------|
| 0 | **Deterministic loopback harness** (workspace). Build a WAV-based A/B harness where each run is byte-deterministic — no IONOS/Pi variance. Per `SACK_REDESIGN_PLAN.md` §9 next-steps: "Without this, the redesign cannot be debugged — every test takes 5 min and has 2-3× variance." Tool: `tools/sack_redesign_wav_ab.py`. | Harness runs twice on the same input, byte-identical output. | Delete the tool. |
| 1 | **Add `batch_seq_id` to DATA frame layout — sender side only, in-batch hardcoded 0.** Bump `DATA_LONG_HEADER_LENGTH` 4→5, `DATA_SHORT_HEADER_LENGTH` 5→6. Insert `batch_seq_id = 0` byte at the new offset in `arq_common.cc:2953-2969`. RSP not yet aware. **NOT REVERSIBLE in deployed state — wire change.** | `mercury.exe --test`: ensure non-SACK + SACK_v1 paths still pass; loopback test: an old-binary RSP can no longer interop with the new-binary CMD → expected (the protocol diverged). | `git revert` step 1. Both peers must run pre-step-1 binary. |
| 2 | **Mirror RSP-side parsing of `batch_seq_id`** (`arq_common.cc:4825-4830` region + `arq_responder.cc:319-340`). Initially just log the parsed value; no routing decision yet. | `mercury.exe --test` passes; loopback CMD↔RSP both new-binary: every DATA frame logs the same batch_seq_id (0). | `git revert` step 2 (and step 1 must also revert). |
| 3 | **Wire `batch_seq_id` to actual batch counter on CMD.** Increment in `process_messages_tx_data()` at the start of every new-data batch (not retransmits). New member `cl_arq_controller::cmd_batch_seq_id` starting at 0. | Loopback: each successive batch logs a `batch_seq_id` 0,1,2,3,... | `git revert` step 3. |
| 4 | **RSP-side cross-batch routing rule.** Implement §4.2.3 routing — accept retransmits keyed to the prior batch's `batch_seq_id`. New `cl_arq_controller::prev_batch_seq_id`, freed on prior-batch full-ACK or 2-batch-old expiry. | `tools/batch_seq_id_test.py` (new, workspace): simulated frame loss + retransmit, verify RSP slots retransmits into prior batch (not current). FAIL on step-3 binary, PASS on step-4. | `git revert` step 4. |
| 5 | **Define `SET_LINK_PARAMS` control frame** (`datalink_defines.h:89-ish`: `#define SET_LINK_PARAMS 0x43`). Encode/decode in `arq_common.cc` control-frame TX/RX path; RSP-side handler emits `[RSP-POLICY-APPLIED]`. No CMD-side caller yet. | Manual control-frame round-trip test via a `--test-link-params` CLI flag (one-shot send). | `git revert` step 5. |
| 6 | **Define `CAP_SACK_V2 = 0x40`.** Negotiate per §4.2.4. When `sack_v2_enabled` is true, **but A.2 policy not yet active** — i.e., gated on a stub controller that always returns "keep current state". | `mercury.exe --test`; loopback: with `--enable-sack-v2` both sides, log line `[SACK-V2] enabled`. Hard gate #5 (CAP_SACK_V2 bit-set non-regression on peers without v2) verified. | `git revert` step 6. |
| 7 | **Define SACK_RSP OFDM control frame type 0x42.** Implement the §4.2.2 encode/decode helpers. RSP-side caller in the `arq_responder.cc:843-879` SACK trigger block, gated on `sack_v2_enabled`. CMD-side dispatch case in `arq_commander.cc:~1560` (per `SACK_REDESIGN_PLAN.md` §7.2). | Loopback `--enable-sack-v2` + simulated frame loss: SACK_RSP arrives; CMD parses; retransmit queue built. **The mfsk SACK path is also still active for `sack_enabled && !sack_v2_enabled` peers.** | `git revert` step 7. |
| 8 | **Remove the standalone retransmit-only batch on `sack_v2_enabled`.** Replace `arq_commander.cc:727-794` retransmit-only `send_batch()` with the §4.2.3 "fold retransmits into next batch" pattern, gated on `sack_v2_enabled`. The v1 path remains for v1-only peers. | Loopback: with `sack_v2_enabled`, retransmits appear *inside* the next 25-frame batch (verified by inspecting `send_batch()` log). RSP correctly slots them into the prior batch's slot. | `git revert` step 8. |
| 9 | **Implement Axis 1 in the new policy framework.** New `cl_arq_controller::policy_evaluate_axis1()` — wraps existing `SUCCESS_BASED_LADDER` logic from `arq_commander.cc:2864-2967`. No behavior change, just refactor. | `mercury.exe --test`; no-regression on existing turboshift / gearshift behavior. | `git revert` step 9. |
| 10 | **Implement Axis 2 controller.** Per-batch loss rate from SACK_RSP bitmap (post step 7) → `recent_partial_rate` ring → step_up / step_down decision → `SET_LINK_PARAMS` TX. | `tools/sack_redesign_wav_ab.py` step 0 harness: simulated 25 %-loss channel; verify batch_size drops from 25 to 10 within 5 batches; on simulated 0 %-loss channel, verify batch_size grows from 10 back to 50 over 8 batches. | `git revert` step 10. |
| 11 | **Implement Axis 3 controller.** SACK-decode tracking → `recent_sack_ok` ring → ON↔PROBE↔OFF state machine → `SET_LINK_PARAMS` TX. | Simulated SACK_RSP-LDPC-fail channel (using `--test-sack-ldpc-fail`): verify Axis 3 transitions to PROBE after 3 misses, OFF after 5. | `git revert` step 11. |
| 12 | **Cross-axis safe-state invariants** (§4.3.4 #6 + axis2_cooldown_batches). | Hand-crafted unit test: Axis-1 down-move while Axis-2 was about to up-move; assert Axis-2 cooldown engaged. | `git revert` step 12. |
| 13 | **Re-run the Track A + Track B + Track C win-test grid** (§5). Append results as §8 of this fact doc. | All §5 hard gates 1-5 PASS; win gates 6-9 PASS (or partial — agent-judged per §5.4). | If hard gates fail: revert 6-12; if win gates fail: STOP per §5.4. |
| 14 | **(Conditional)** If gates pass, propose flipping `CAP_SACK_V2` default-on. Separate PR; soak run on Pi testbed. | Soak run TBD. | Trivial — flip default back. |
| 15 | **(Conditional, future)** Delete legacy MFSK SACK code per §4.2.5. Done only after `CAP_SACK_V2` is default-on for one release. | `--test` + loopback regression: v1-only peer interop no longer needed; all paths exercise v2. | `git revert` — but be aware step 1's wire change is the irreversible boundary. |

**Steps that are NOT reversible in deployed state:**
- **Step 1** is the wire change (DATA header layout). Once shipped, peers
  running pre-step-1 binaries cannot interop on the SACK_V2 path. Mitigation:
  the new header is gated on `CAP_SACK_V2`; if either peer does not
  advertise it, both fall back to the legacy header. This means **the
  header growth must be conditional on `sack_v2_enabled`, not unconditional**.
  Sub-amendment: refine step 1 to "add header growth gated on
  `sack_v2_enabled`" — which sequences step 6 BEFORE step 1's wire activation.
  Revised order: 0, 5, 6 (negotiation only), 1+2+3 together (gated on
  sack_v2_enabled), 4, 7, 8, 9-13, 14, 15.

### §7.0 RESULT — Step 0 (2026-05-15)

Tool committed: workspace `monitor` `593aaf1` —
`tools/sack_redesign_wav_ab.py` (691 LOC, single file).

Reuse:
- `tools/sack_lossy_ab.py:263` `parse_mercury_logs(cmd_text, rsp_text)` —
  imported verbatim. Pure-text regex parser; deterministic given identical
  inputs. This is what makes the harness's grading layer byte-stable.
- `mercury/tools/mercury_benchmark.py:457` `MercurySession` — composed into
  the optional `--live` mode for WASAPI+VB-Cable runs (acknowledged
  non-deterministic at the audio layer; grading still deterministic).
- `tools/analyze_sack_wav.py:14` `read_wav` pattern — re-implemented via
  Python's stdlib `wave` module to drop the `numpy` dependency for the
  hashing path (`tools/sack_redesign_wav_ab.py:181`).

Verifying test (the Step 0 §7 row): PASS.

```
$ python tools/sack_redesign_wav_ab.py --self-test
[STEP0-SELFTEST] pass 1: building result from synthetic fixtures...
[STEP0-SELFTEST] pass 2: rebuilding result from same fixtures...
[STEP0-SELFTEST] pass 1 sha256: 2a9366a1166182ba57e66fa4174989675059b750652023ae63bc6764cfbb0a84
[STEP0-SELFTEST] pass 2 sha256: 2a9366a1166182ba57e66fa4174989675059b750652023ae63bc6764cfbb0a84
[STEP0-SELFTEST] pass 1 bytes : 4323
[STEP0-SELFTEST] pass 2 bytes : 4323
[STEP0-SELFTEST] PASS — same input → byte-identical output (and different input → different output, as expected).
```

Also verified PASS:
1. Same harness run twice in two SEPARATE Python invocations (different
   PIDs, different `PYTHONHASHSEED`) — identical exit codes, identical
   internal hash.
2. Replay-mode with bundled fixtures written to disk and consumed via
   `--a-cmd-log/--a-rsp-log` flags across two separate `python` invocations
   — sha256 of `--out` JSON identical (`957bfd9e…73c386e5`, 8897 bytes).

Non-determinism caveats (mercury source, read-only audit):
1. `source/datalink_layer/arq_responder.cc:1218`
   `messages_control.data[1]=1+rand()%0xfe;` — connection_id is random per
   session. Observable in logs (different conn_id byte per run). Affects
   **mercury runs**, NOT the harness's grading of stored logs. Documented
   as a caveat. Mitigation if later needed: add a `--connection-id <byte>`
   CLI flag to mercury (Step 1+ territory; out of scope for Step 0).
2. `source/physical_layer/awgn.cc:33-39` `cl_awgn::set_seed(long)` — AWGN
   is seeded by `telecom_system.cc:4700` `awgn_channel.set_seed(rand())`.
   The outer `rand()` is never explicitly `srand()`-ed in mercury source.
   On both Linux and Windows libc, the implicit seed is 1, so it is
   reproducible **per-process** — but this is a libc implementation detail
   and not a guarantee. Affects only the `-Z` noise-injection path (PHY
   tests); the ARQ-level grading harness operates on logs, not on raw
   PHY samples, so this is not load-bearing for Step 0.
3. `source/physical_layer/telecom_system.cc:3829, 3919; ofdm.cc:913` use
   `__srandom()` (vendored glibc random from `os_interop.cc:379`) for
   bit-energy-dispersal and OFDM scrambling, seeded with a fixed
   `bit_energy_dispersal_seed`. **Deterministic.** No caveat.
4. Real-time audio path (WASAPI shared-mode) adds buffer-scheduling jitter
   (~5–15 ms/buffer), PTT-delay timers are clock-based, and TCP-data-port
   sends are Nagle-batched at variable boundaries. These break byte-level
   determinism on **any** live mercury run, which is why Step 0 separates
   the "live run" (acknowledged non-deterministic) from the "grading"
   (proven byte-deterministic by `--self-test`). The next 15 steps depend
   on the **grading** being deterministic; they do not depend on the live
   run being deterministic.

Mercury source was **not modified** for Step 0, per the plan's "Step 0 only,
workspace tooling only" constraint.

Plan doc commit (this entry): mercury `monitor` (next commit on this branch).

Step 1 is NOT started.

---

## §7 Open questions [?]

- [?] **`SET_LINK_PARAMS` ACK latency.** A new control frame is needed any
  time Axis 2 / Axis 3 moves. If the move happens between batches the
  half-duplex cost is one extra control round-trip (~ 1.5 s at WB_CFG10).
  At Axis 2's projected step cadence (1 move per 5-batch window) that is
  ~5 % overhead. Alternative: piggyback `SET_LINK_PARAMS` as a side-channel
  in the SACK_RSP frame (bitmap field + 3 bytes more = 8 bytes total, still
  fits any LDPC codeword). Recommended in implementation but flagged here
  because it raises the SACK_RSP from a pure "ACK bitmap" to a "ACK +
  policy update" frame — protocol coupling.
- [?] **Axis 3 PROBE state behavior on RSP.** When CMD enters SACK_PROBE,
  does RSP still generate SACK_RSP on partial batches, or wait for a SACK
  request? Recommended: RSP continues to generate SACK_RSP unconditionally
  while `sack_v2_enabled`; PROBE state only changes how CMD *interprets*
  observed SACK arrivals (i.e., it is purely a CMD-side counter regime).
  Simpler than the ARDOP-style request/response. Needs validation on the
  step-7 loopback.
- [?] **Multi-axis interaction with turboshift forward/reverse probe.**
  Turboshift is a one-shot ladder climb at link-up (`arq_common.cc:214-218`,
  `arq_commander.cc:1079-1276`). Design A's Axis 1 is the *post-turboshift*
  gearshift. During turboshift, Axis 2 / Axis 3 should be **frozen** at
  their negotiated defaults (batch=25 if sack_v2; sack_mode=ON). Released
  to Design A once `turboshift_phase == TURBO_DONE`. Implementation: gate
  `policy_evaluate_axis{2,3}()` on `turboshift_phase == TURBO_DONE`.
- [?] **Encryption-batch coupling.** `arq_common.cc:130` `crypto_batch_size = 20`
  + `arq_commander.cc:3001` `batch_capacity = data_batch_size * max_frame`
  imply compression / streaming context assumes a specific batch size.
  Per `SACK_FIX_PLAN.md` §9: dynamic batch resize compatibility with
  streaming-zstd / PPMd context is **unverified.** Step 10 must include a
  test that resizes batch in the middle of a streaming crypto session and
  asserts the receiver still decrypts/decompresses correctly. If this
  fails, A.2's range may need to be tied to the crypto-batch boundary
  (e.g., only allow resizes at crypto-batch reflushes).
- [?] **Should `batch_seq_id` be 1 byte or 2?** 1 byte ≡ 256 mod wraparound;
  on a half-duplex link with at most one outstanding batch, 256 is far more
  than enough. But if a future "pipelined SACK" optimization wanted to
  allow 2 outstanding batches, 1 byte is still 128× the headroom. **1 byte
  recommended.**
- [?] **`recent_sack_ok` ring depth.** 10 samples = a slow signal (≥ 10
  SACK events ≈ 10+ batches). Faster Axis-3 reaction would be 5 samples.
  Trade-off: fast Axis-3 reacts to transient bad SACKs (e.g., a single
  fade); slow Axis-3 misses brief SACK collapses. The cellular OLLA
  literature uses 10 samples by default. **Start at 10; tune from win-test
  data in step 13.**

---

## §8 Cross-references

- `SACK_REDESIGN_PLAN.md` — this doc EXTENDS it. §5.1 (SACK_RSP frame),
  §5.3 (CMD-side dispatch), §5.4 (cleanup), §7 (open questions resolved),
  §9 (the failed v2 attempt — §4.2.4 critical mitigations) all flow
  through unchanged.
- `SACK_FIX_PLAN.md` — Plan-(a) is the post-TX-timeout fix that Design A
  inherits unmodified (§2.4 trace). §11.7 named the architectural problem
  this doc answers. The CMD-only timeout-decoupling (`arq_common.cc:437-506`)
  is the calibration source for the geometry-derived `sack_arrival` term;
  Design A's new SACK_RSP frame replaces the `pattern_time` calculation
  with the OFDM control-frame airtime.
- `SACK_LOSSY_CHANNEL_WINTEST.md` — §5.2/§5.4 cycle_ms baselines are the
  arithmetic floor Design A must beat. §6 (LDPC miscorrection predicate)
  is the correctness check Design A inherits. §7.3 (the no-loss-band-where-
  SACK-wins finding) is the design motivation.
- `SACK_RETRANSMIT_BATCHING_INVESTIGATION.md` — §8's "Sketch of a fix
  toward (b)" steps 1-3 are implemented as Design A steps 7-8; step 4
  (the cross-batch sequence-space caveat) is paid by Design A's
  `batch_seq_id` (§4.2.1-§4.2.3). §6.2 — (a) was never (b), since first
  SACK commit — explains why this redesign is not "fixing a regression"
  but "paying a cost the original plan deferred."
- `SACK_TURNAROUND_FIX_PLAN.md` §8.1 — the geometry-derived per-frame
  timer pattern on RSP (`arq_responder.cc:327-406`) is the model that
  Plan-(a) ported to CMD (§2.4). Design A keeps both as-is.
- `SACK_LDPC_FALLBACK_INVESTIGATION.md` — A2 ldpc=NO fallback is the
  correctness fix for the legacy MFSK SACK path. Design A's SACK_RSP
  OFDM frame uses the existing data-path LDPC (no separate `sack_ldpc`
  needed) — A2 becomes moot once §4.2.5 cleanup runs.
- `SACK_THROUGHPUT_INVESTIGATION.md` §16.3 — flags the never-built
  `crypto_batch_buffer` double-buffer that would have enabled (b);
  Design A pays that cost (§4.2.3, §7 open question on encryption-batch
  coupling).
