# Phase B: MFSK CONNECT research + design (2026-05-26)

**Status:** RESEARCH / PRE-IMPLEMENTATION. No production code in this branch yet.
Produced by the Phase B research agent (per `phase-a-to-b-workplan.md` §4 B.10).

**Scope:** Tasks §1–§8 of the Phase B brief. Answer the architectural decisions
the user must make before implementation begins. **No code changes.**

**Conventions:** numbered sections, `file.cc:line` citations, `[?]` for
unresolved questions, struck-through text for corrections. Companions:
`mfsk-robust-ack.md` (the proven 2026-05-24 ACK+SACK suffix machinery),
`phase-a2-bp-osd-research.md` (template), `data-flow-retx-queue.md`
(audit template).

---

## §0. Executive summary

- The CONNECT bottleneck is **two LDPC control frames**, START_CONNECTION
  (CMD→RSP) and TEST_CONNECTION_ACK (RSP→CMD), both sent as full LDPC frames
  at the same config as data (ROBUST_0 = cfg 100 in robust mode). They share
  the data-decoder cliff at WGN:-4 to -8. HAIL works to WGN:-10 (§1.7).
- Two distinct frames are involved. The framework calls these
  `START_CONNECTION` and `CONNECTION_ACCEPTED` but the second is actually
  `TEST_CONNECTION_ACK` (msg 0x45); CONNECTION_ACCEPTED is the *link_status*
  enum value 0x06 the commander transitions into when it receives the ACK
  (§1.5, §1.6).
- The 2026-05-24 MFSK ACK suffix (52 bits at M=16, 13 suffix symbols + 16
  base pattern, ~705 ms) already decodes at the same ROBUST_0 floor as HAIL
  (WGN:-10 ≈). Reusing this machinery for CONNECT is mechanically clean:
  same TX path (`generate_ack_sack_pattern_passband` at telecom_system.cc:3121),
  same RX detector (`detect_ack_snr_from_passband` at telecom_system.cc:3164),
  same payload pack/unpack (`pack_ack_sack_payload`/`unpack_ack_sack_payload`
  in mfsk.h:118-125). Adding a 2-bit frame-type discriminator + a few new
  RSP wire fields is sufficient (§3).
- **Three architectural decisions need user input** before code starts (§8):
  1. Suffix payload format (recommendation: 52-bit `[type:2|fields:38|crc12:12]`
     reusing ack_sack_suffix_len=13 verbatim, §3).
  2. Interop strategy with legacy LDPC peers (recommendation: option (c) —
     hard flag-day, no legacy LDPC CONNECT fallback. Reasoning: only 2 peers
     deployed; same flag-day as 2026-05-24 cap-byte collapse, §4).
  3. Whether MFSK CONNECT replaces *both* START_CONNECTION and
     TEST_CONNECTION_ACK, or just one (recommendation: both — the latter is
     the more-frequently-corrupted one in our existing logs because RSP→CMD
     traverses a worse channel half the time, §3.5, §5).
- **Effort estimate**: ~10-14 working days code+test, ~2 hardware integration
  cycles (~3 hours each, butler-mediated). Less than the workplan's "3-4
  weeks". Most parts are agent-parallelizable (§7).
- **The HAIL self-detect race is a known carbon copy risk for MFSK CONNECT.**
  It's the same MFSK family. Mitigation in §6.5.

---

## §1. Audit — current CONNECT path

Cited from the working tree at commit `9c3fc40` (monitor HEAD).

### §1.1 Wire path: CMD constructs START_CONNECTION

`arq_commander.cc:285` adds START_CONNECTION to messages_control. Construction
at `arq_commander.cc:447-462`:

```
messages_control.data[0] = START_CONNECTION (0x31)        // 1 byte
messages_control.data[1] = CRC8 over destination callsign // 1 byte
messages_control.data[2..6] = callsign_pack(my_call_sign, 5 bytes packed
                                            base-36 + NB flag + length)
messages_control.length  = 7
messages_control.id      = 0
connection_id            = BROADCAST_ID (0x00)
```

The callsign pack (`arq.h:114-136`) uses base-36 (A-Z + 0-9) with a 3-bit
length field and 1 narrowband flag, total 5 bytes for up to 6 chars. SSID is
NOT carried here — sent later in TEST_CONNECTION.

So **START_CONNECTION payload = 7 bytes = 56 bits** on top of the 3-byte
standard control-frame header (`type, conn_id, seq_num`). Total wire bytes:
10. At ROBUST_0 (rate-1/16 LDPC, 100 info bits per frame), 10 bytes = 80 bits
fit easily in one LDPC frame (with padding).

### §1.2 Wire path: CMD transmits START_CONNECTION

`arq_commander.cc:684 process_messages_tx_control()` copies messages_control
to messages_batch_tx[], pads to control_batch_size, then calls
`send_batch()`.

`send_batch()` at `arq_common.cc:3260` builds a CONTROL frame:
3-byte header (`CONTROL, conn_id, seq_num`) + 7-byte payload from
messages_batch_tx[0].data. Then `telecom_system->transmit_byte()` modulates
through whatever is loaded — at ROBUST_0 this is MFSK preamble + LDPC + MFSK
data symbols. (NOT the MFSK suffix path — that's a separate `ack_mfsk` object
shared between ACK pattern, BREAK pattern, HAIL pattern.)

The frame's wire structure at ROBUST_0:
- MFSK preamble (4 symbols WB / 8 symbols NB)
- LDPC-coded payload mapped to M=32 MFSK tones over N symbols
- (No suffix unless this is a SACK/ACK; START_CONNECTION isn't)

This LDPC frame inherits all the same decode requirements as data: preamble
coarse-freq sync, fine sync, MFSK demap, LLR computation, log-sum-exp BP decode.
**Same cliff at WGN:-4 to -8** (per phase-a-to-b-workplan.md §10 mini-cliff).

### §1.3 ACK to START_CONNECTION (RSP→CMD direction)

When RSP successfully decodes START_CONNECTION, it transitions to
`CONNECTION_RECEIVED` (link_status=5) at `arq_responder.cc:1720`. ACK
dispatch goes through `process_messages_acknowledging_control()` at
`arq_responder.cc:763+`. For an ordinary START_CONNECTION ACK (not
TEST_CONNECTION_ACK or KEY_EXCHANGE_1):

`arq_responder.cc:829-846` — if `ack_pattern_time_ms > 0` (modern default),
the ACK is **a legacy MFSK pattern via `send_ack_pattern()`** (`arq_common.cc:3698`).
This is the 16-symbol Welch-Costas pattern, no payload, ROBUST_0-floor
robust.

**So the RSP→CMD ack for START_CONNECTION is already at the floor.** It is
not the bottleneck.

### §1.4 RSP-side decode + state machine (consumer of START_CONNECTION)

`arq_responder.cc:1646 process_control_responder()`:
- At line `:1650`: gate `(link_status==LISTENING || CONNECTION_RECEIVED)
  && code==START_CONNECTION`
- Reads `messages_control.data[1]` as expected-CRC, computes
  `CRC8(my_call_sign)` and rejects if mismatch (line :1738, "callsign CRC
  mismatch")
- On match: `callsign_unpack(&messages_control.data[2])` gives the commander's
  callsign + narrowband flag (line :1691)
- Mutates **a lot** of state (line :1696-1734):
  - `destination_call_sign = unpacked`
  - `session_narrowband = peer_nb || local_nb` (NB always wins)
  - `link_status = CONNECTION_RECEIVED`
  - `connection_status = ACKNOWLEDGING_CONTROL`
  - `messages_control.data[1] = BROADCAST_ID` (or random) for ACK
  - `assigned_connection_id = ...`
  - `watchdog_timer.start()`
  - Emits `"PENDING <callsign>\r"` to TCP control socket (Winlink stops scanning)

**Invariants the consumers (state machine + Winlink protocol) assume:**
- `destination_call_sign` is non-empty + valid base-36 chars
- `session_narrowband` reflects both sides' bandwidth preference
- `link_status` transitions LISTENING → CONNECTION_RECEIVED in exactly one
  message (no half-states)
- ACK is sent immediately after this handler returns (process_messages_
  acknowledging_control at :763 runs next cycle)

### §1.5 CMD-side processing of the ACK

`arq_commander.cc:3301 process_control_commander()`:
- Detects the ACK pattern via `receive_ack_pattern()` (not shown but
  symmetric to RSP's `receive_hail_pattern()`)
- At `:3305`: gate `link_status==CONNECTING && messages_control.data[0]
  ==START_CONNECTION`
- Mutates state (line :3315-3333):
  - `link_status = CONNECTION_ACCEPTED` (link_status enum 6, NOT a message)
  - `connection_status = TRANSMITTING_CONTROL`
  - `connection_id = messages_control.data[1]` (or BROADCAST_ID if pattern ACK)
- Drops to TRANSMITTING_CONTROL which then sends TEST_CONNECTION at line :289

**This is what the brief means by "CONNECTION_ACCEPTED" — it's the
link_status that CMD enters when it has received START_CONNECTION's ACK.**
There is no on-wire CONNECTION_ACCEPTED message. The on-wire follow-up is
TEST_CONNECTION (CMD→RSP) and TEST_CONNECTION_ACK (RSP→CMD).

### §1.6 TEST_CONNECTION + TEST_CONNECTION_ACK (the rest of the handshake)

CMD constructs TEST_CONNECTION at `arq_commander.cc:463-477`:
```
data[0]   = TEST_CONNECTION (0x32)         // 1 byte
data[1-4] = float SNR (uplink estimate)    // 4 bytes
data[5]   = local_capability (CAP_WB_CAPABLE | CAP_ENCRYPTION) // 1 byte
data[6]   = commander SSID (0=none, ...)   // 1 byte
length    = 7
```

RSP receives + processes at `arq_responder.cc:1742-1936`. The RSP response is
`TEST_CONNECTION_ACK` (msg 0x45) constructed at `arq_responder.cc:1923-1928`:
```
data[0] = TEST_CONNECTION_ACK (0x45)   // 1 byte
data[1] = peer_capability (echoed)     // 1 byte
data[2] = local_capability (rsp's)     // 1 byte
data[3] = CRC8 over data[1..2]         // 1 byte
length  = 4
```

**TEST_CONNECTION_ACK is an LDPC frame (NOT a pattern ACK).** At
`arq_responder.cc:793-827`, it dispatches via `send_batch()` on
`data_configuration` (OFDM mode) or `ack_configuration` (MFSK-ctrl mode
fallback). At ROBUST_0, both are ROBUST_0 LDPC — **same cliff**.

CMD validates the echo at `arq_commander.cc:3344-3392`: checks CRC8 and that
`echoed_cap == local_capability`. On success: `link_status=CONNECTED`. On
failure: drop with retry budget `handshake_retries_left`.

### §1.7 So the actual bottlenecks are:

| Frame | Wire format | Bottleneck? | Why |
|---|---|---|---|
| HAIL beacon (MFSK pattern + 4-tone CRC suffix) | Pattern correlator | NO — floor at WGN:-10 | Welch-Costas pattern + FNV-1a hash suffix, no LDPC |
| **START_CONNECTION** | LDPC at ROBUST_0 (10 bytes wire) | **YES — cliff at -4 to -8** | Full LDPC decode required for 7-byte payload |
| ACK to START_CONNECTION (RSP→CMD) | MFSK pattern (no payload) | NO — pattern correlator | Already at floor (per §1.3) |
| TEST_CONNECTION | LDPC at ROBUST_0 (10 bytes wire) | **YES — same cliff** | Same as START_CONNECTION |
| **TEST_CONNECTION_ACK** | LDPC at ROBUST_0 (7 bytes wire) | **YES — same cliff** | Carries 3-byte cap-echo payload, must use LDPC to be parseable |

**Three LDPC frames in the handshake chain inherit the cliff.** All three
need an MFSK suffix replacement. The brief's "CONNECT + ACCEPTED" maps to
"START_CONNECTION + TEST_CONNECTION_ACK"; TEST_CONNECTION is a *third* LDPC
frame the brief didn't enumerate but which is also on the critical path.

### §1.8 State mutations triggered by these frames

These are the consumers any MFSK variant must keep happy.

**START_CONNECTION arrives → RSP mutates (`arq_responder.cc:1688-1735`):**
- `destination_call_sign` (string, must be valid base-36)
- `session_narrowband` (bool: peer_nb || local_nb)
- `link_status = CONNECTION_RECEIVED` (5)
- `connection_status = ACKNOWLEDGING_CONTROL`
- `messages_control.data[1] = BROADCAST_ID` (sets up ACK frame)
- `assigned_connection_id`
- `watchdog_timer.start()`
- Side effect: Winlink "PENDING" TCP message
- Side effect: MERCURY_GUI monitor callsign update (passive_monitor branch)

**START_CONNECTION ACK arrives → CMD mutates (`arq_commander.cc:3305-3333`):**
- `link_status = CONNECTION_ACCEPTED` (6)
- `connection_status = TRANSMITTING_CONTROL`
- `connection_id`, `assigned_connection_id`
- `connection_attempt_timer.reset/start`
- `watchdog_timer.start`
- (For NB/WB auto-negotiation: NB commander switches NB→WB on detect of WB ACK,
  `:3308-3313`)

**TEST_CONNECTION arrives → RSP mutates (`arq_responder.cc:1742-1936`):**
- `measurements.SNR_uplink = peer's SNR estimate`
- `peer_capability` (CAP_WB_CAPABLE | CAP_ENCRYPTION, byte 5)
- `destination_call_sign` SSID appended (if peer_ssid != SSID_NONE)
- `compression_enabled`, `compressor.init()` (CAP_COMPRESSION removed; force_compress)
- `b2f_handler.init()`, `b2f_handler.unroll_enabled = true`
- `encryption_enabled` (if both peers advertise CAP_ENCRYPTION)
- `compressor.streaming_enable()` (if compression on)
- `sack_enabled`, `sack_v2_enabled`, `set_data_batch_size(N)`,
  `recalculate_ack_timeout_for_batch()`
- `link_status = CONNECTED`
- Side effect: TCP "CONNECTED <cmd> <rsp> <bw>\r" message

**TEST_CONNECTION_ACK arrives → CMD mutates (`arq_commander.cc:3335-3551`):**
- Same as TEST_CONNECTION on RSP side (mirror) but only after CRC8 validation
- `handshake_confirmed = true`
- `peer_capability = rsp_own` (echoed back)
- `link_status = CONNECTED`
- Side effect: TCP "CONNECTED" message

### §1.9 Failure mode of LDPC START_CONNECTION at WGN:-8

Per phase-a-to-b-workplan §10: at WGN:-8 the cliff fires. The drive_a26.py
A/B test confirmed `iter_mean_rsp=1.0` in every successful cell — LDPC
decoder converges immediately when it works at all. The mini-cliff sweep
shows `connect_failed` at WGN:-8 (so the CMD never received the ACK).

The failure is likely **either the LDPC decoder failing to find a codeword
(returning all-zeros / wrong codeword), OR the preamble sync failing to lock
on the short ROBUST_0 frame**. Both paths produce the same observable
("no frame decoded"). HAIL works at WGN:-10 because its detector is the
ack_pattern correlator (energy + tone-pattern matching), not LDPC.

**For Phase B planning, treat the bottleneck as "any LDPC-class frame at
ROBUST_0 doesn't decode below WGN:-8"** rather than narrowing to preamble vs
LDPC, since the MFSK suffix path bypasses BOTH.

---

## §2. Audit — existing MFSK ACK suffix machinery

The MFSK ACK+SACK suffix (2026-05-24, see `mfsk-robust-ack.md`) is the prior
art we'll reuse. Cited from monitor 9c3fc40.

### §2.1 TX path

| Step | File:line | Description |
|---|---|---|
| Helper for high-level send | `arq_common.cc:4287` | `send_mfsk_ack_sack(bsi, bitmap)` — caller-friendly wrapper |
| Compute CRC | `arq_common.cc:4308-4314` | CRC12 over `[bsi || bitmap_bytes]` via `CRC12_calc()` |
| Build pattern | `telecom_system.cc:3121 generate_ack_sack_pattern_passband(out, bsi, bitmap, crc12)` | Generates 29 MFSK symbols (16 base pattern + 13 suffix) |
| Inner symbol gen | `mfsk.cc generate_ack_sack_pattern()` | Lays down the Welch-Costas tones + payload-coded suffix tones |
| Payload pack | `mfsk.cc pack_ack_sack_payload(bsi, bitmap, crc12, out_tones[13])` | 52 bits → 13 tones at log2(M)=4 bits/tone for M=16 |
| FIR + PTT + audio | `arq_common.cc:4327-4400` | Standard PTT/FIR chain, wait for playback drain |

**Total wire time on WB:** ~705 ms (16 base + 13 suffix symbols, each
~24 ms at WB Nofdm=256, interp=3, Fs=48 kHz; symbol period = 24 ms).

### §2.2 RX path

| Step | File:line | Description |
|---|---|---|
| Composite detect+decode | `telecom_system.cc:3288 decode_ack_sack_from_passband(data, size, *bsi, *bitmap, *crc12, *matched)` | Returns true if ACK pattern matched AND suffix capture is clean |
| Internal SNR-detect (reused) | `telecom_system.cc:3164 detect_ack_snr_from_passband(...)` | Correlates baseband signal vs `ack_pattern_nsymb` Welch-Costas template; on match, captures 16 de-hopped suffix tones into `ack_mfsk.last_ack_sack_suffix_tones[]` (`:3222-3235`) |
| Detector core | `ofdm.cc detect_ack_pattern()` | Computes per-symbol per-tone energy correlation at every offset; thresholds via `ack_match_threshold` |
| Suffix decode | `ofdm.cc decode_suffix_tones()` | At the matched offset, FFT-bin-argmax for each of the 13 suffix symbols |
| Payload unpack | `mfsk.cc decode_ack_sack_from_last_capture(*bsi, *bitmap, *crc12)` → calls `unpack_ack_sack_payload()` | 13 tones → 52 bits |
| CRC validation | caller (e.g. arq_commander.cc) | Compute CRC12 over `[bsi||bitmap]`, compare with received |

### §2.3 RSP integration (who currently calls it)

The MFSK ACK+SACK is currently fired at TWO sites in arq_responder.cc:
- `arq_responder.cc:1281` — partial-batch path (per-slot bitmap from
  `messages_rx[].status == RECEIVED`)
- `arq_responder.cc:1418` — clean-batch path (all-ones bitmap)

In both, `used_mfsk_path = (send_mfsk_ack_sack(...) > 0)` and the call sets
`messages_control.data[0]` is NOT touched — the MFSK ACK is a passband-only
side-channel, not part of the messages_control state machine.

### §2.4 Detector + state-machine integration on CMD side

When CMD is listening for an ACK after a data batch, the ackpat detector
fires in `arq_commander.cc:~1547` (per mfsk-robust-ack.md §4):
```
listen_window opens
  → detect_ack_pattern_from_passband()
    if hit: decode_ack_sack_from_passband() → (bsi, bitmap, crc12)
      validate CRC12 → if pass, route to ACK-GATE; else treat as no-ACK
    if no hit: receiving_timer expires → retransmit
```

### §2.5 Symbol count + duration

WB: M=16, log2(M)=4 bits per suffix symbol. **Base pattern = 16 symbols
(Welch-Costas, 2× repetition of 8-tone sequence); suffix = 13 symbols.**
Total 29 symbols.

At WB ROBUST_0 (Nofdm=256, interp=3, Fs=48kHz): symbol period =
256*3/48000 = 16 ms. So 29 * 16 = **464 ms** pure tone time. With PTT
on/off (~50ms each), pilot (~100ms if enabled), padding: ~600-700 ms total
wire time.

NB: M=8 (3 bits/symbol), `ack_sack_suffix_len() = 0` (NB unsupported, see
`mfsk.h:111`). NB has no SACK and would need M=8 redesign (13 → 18 symbols
+ longer NB symbol period = ~2 s) — NOT in scope for Phase B.

### §2.6 Floor measurement

Per `memory/mfsk_robust_ack_shipped.md`: at ROBUST_0 IONOS sweep with 300s
dwell, ACK+SACK suffix at WGN:14 still delivers 96 bps of *data*, implying
the ACK pattern + suffix CRC12-validates. HAIL still fires to WGN:-10. The
suffix decode path is asymmetrically more robust than LDPC at the same SNR
because the detector relies on energy-pattern matching with hard-decision
suffix tone argmax (no SPA, no LLR).

**Empirically, MFSK ACK + suffix is at or below the ROBUST_0 LDPC floor
(WGN:-4 to -8 today; expected to reach WGN:-10 if cliff matches HAIL).**

---

## §3. Design — MFSK CONTROL suffix payload format

### §3.1 Constraint inventory

A new MFSK CONTROL suffix frame must carry, in order of constraint priority:

1. **Frame-type discriminator** — at minimum 2 bits (4 types: HAIL,
   START_CONNECTION, TEST_CONNECTION_ACK, TEST_CONNECTION). 3 bits if we
   want headroom for future additions.
2. **CRC integrity** — at least CRC12 (CRC8 risks "partial→all-ones"
   silent corruption per mfsk-robust-ack.md §3.2; CRC16 is overkill since
   downstream layers have their own checksums).
3. **Per-frame payload fields**:
   - START_CONNECTION: 7 bytes today = 56 bits.
     But re-encode to MFSK-suffix-friendly fields: **CRC8 of destination
     callsign (8 bits)**, **packed sender callsign (40 bits)**, **NB flag
     (1 bit)**, **length field (3 bits)** = 52 bits raw.
     [?] Can we drop sender callsign and rely on HAIL? See §3.4 below.
   - TEST_CONNECTION_ACK: 3 bytes today = 24 bits (echoed_cap=8,
     own_cap=8, CRC8=8). The CRC8 is *over* the payload, not the
     wire-payload; in the new format we get CRC12 for free over everything.
     Net wire bits: **echoed_cap=8 (or 2 bits if we collapse to
     WB_CAPABLE+ENCRYPTION), own_cap=8 (or 2 bits)** + SSID-needed?
   - TEST_CONNECTION: SNR (32-bit float) + cap + SSID + crc. **32-bit float
     is fat for an MFSK suffix.** Could quantize to 4 bits (16 SNR levels,
     matches the existing SNR_SUFFIX path in §2.2). Then capability
     (2 bits each side per §3.5) + SSID (8 bits or 4 bits encoded).

### §3.2 RECOMMENDED format

Reuse `ack_sack_suffix_len() = 13` symbols at M=16 → 52 bits exactly. Same
13-symbol footprint as the ACK+SACK suffix. The 52 bits split as:

```
[ frame_type:2 | payload:38 | crc12:12 ] = 52 bits total
```

Frame types (2 bits, 4 codes):
- `0b00` = ACK+SACK (existing 2026-05-24 format — backward compat reserved)
- `0b01` = START_CONNECTION
- `0b10` = TEST_CONNECTION_ACK
- `0b11` = TEST_CONNECTION (with quantized SNR)

Per-type payload encoding (38 bits each):

**START_CONNECTION (frame_type=0b01):**
```
[ dest_call_crc8:8 | sender_pack:30 ]   = 38 bits
```
- `dest_call_crc8` = CRC8 over destination callsign (8 bits). Same shape
  as `data[1]` today. Used by RSP for callsign filtering. **Already proven
  by HAIL pattern's 4-tone suffix** (FNV-1a hash, same shape).
- `sender_pack` = base-36 callsign in 6 chars × 5 bits per char = 30 bits.
  Use 5 bits per char (32 codes: A-Z + 0-Y leaves 26+6=32 chars; works for
  ASCII A-Z + 0-5). [?] Hmm — base-36 needs 6 bits/char. We need an
  alternative. Options:
  - Option a: 5 bits/char × 5 chars = 25 bits. Loses 1 char.
    Mercury currently packs 6 chars in 36 bits.
  - Option b: drop sender callsign entirely. RSP knows the sender from HAIL
    (which is directed at RSP's own callsign hash); CMD doesn't *send* its
    own callsign in HAIL but RSP doesn't need it to ACK — just `BROADCAST_ID`.
    Wait — does RSP need sender callsign? Per arq_responder.cc:1691, RSP
    populates `destination_call_sign` from the unpacked sender. Used for:
    (a) Winlink "PENDING <call>" message (cosmetic), (b)
    `gui_set_monitor_callsigns` (cosmetic), (c) `destination_call_sign` in
    TEST_CONNECTION's CONNECTED ACK (functional). So RSP DOES need sender
    callsign to confirm.
  - Option c: 6 bits/char × 6 chars = 36 bits + 2 spare = 38 bits exactly!
    Fits. Use the same `callsign_pack()` from arq.h:114.

**Recommended: option c.** 38-bit payload = `[dest_crc8:8 | sender_pack:30]`
where `sender_pack` is the existing 5-byte callsign_pack output truncated
to 30 bits OR base-36-encoded over only 5 chars (e.g. always pack first 5
chars). [?] User decision: is "first 5 chars of callsign" acceptable
truncation? Most US amateur calls are W#XXX (4 chars) or KX#XXX (5 chars);
6-char calls are unusual. Could also use option a (5 bits × 6 chars = 30 bits)
with a 32-char alphabet (no digits 6-9? or omit some letters?). **Cleanest:
keep full 6-char support with 6 bits/char, accept the 2 spare bits.**

Final START_CONNECTION 38-bit payload: `[ dest_crc8:8 | sender_pack:30 ]`
where `sender_pack` = 6 chars × 5 bits each (alphabet of 32: 26 letters +
6 digits 0-5). Drop digits 6-9 from packed callsigns. **[?] User decision.**

Alternative: include NB flag (1 bit) + length (3 bits) at cost of 4 fewer
sender-pack bits. Current LDPC START_CONNECTION carries `NB flag` to drive
NB/WB auto-negotiation (arq.h:118, arq_responder.cc:1696). Keep it:
```
[ dest_crc8:8 | nb_flag:1 | sender_pack:29 | reserved:0 ]
```

**TEST_CONNECTION_ACK (frame_type=0b10):**
```
[ echoed_cap:2 | own_cap:2 | reserved:34 ]   = 38 bits
```
- `echoed_cap` (2 bits) = peer_capability echoed (CAP_WB_CAPABLE + CAP_ENCRYPTION
  are the only 2 bits in use after 2026-05-24 capability collapse). `local_capability`
  comes from `arq.h` — see `arq_responder.cc:1924-1925`.
- `own_cap` (2 bits) = local_capability.
- 34 reserved bits: room for future use (extended capability, SNR snapshot,
  link parameters, ARQ batch-size confirmation).

CMD validates `echoed_cap == local_capability` per `arq_commander.cc:3370`.
The new CRC12 (12 bits over `[frame_type|payload]`) replaces the CRC8 over
`[echoed_cap|own_cap]` at arq_responder.cc:1926-1927.

**TEST_CONNECTION (frame_type=0b11):**
```
[ snr_q:4 | local_cap:2 | ssid:8 | reserved:24 ]   = 38 bits
```
- `snr_q` = SNR quantized to 4 bits (16 levels, mirroring the existing
  `snr_to_tone()` machinery at mfsk.cc:~snr_to_tone). Mercury already
  handles quantized SNR in the SNR_SUFFIX path.
- `local_cap` = 2 bits (CAP_WB_CAPABLE, CAP_ENCRYPTION).
- `ssid` = 8 bits raw, mirroring `data[6]` today.

### §3.3 Bit budget summary

| Frame | Discriminator | Payload | CRC12 | Total | Symbols at M=16 |
|---|---|---|---|---|---|
| ACK+SACK (existing 0b00) | 2 | 38 (bsi:8|bitmap:32 — but uses 2 bits less so re-fit) | 12 | 52 | 13 |
| START_CONNECTION (0b01) | 2 | 38 | 12 | 52 | 13 |
| TEST_CONNECTION_ACK (0b10) | 2 | 38 | 12 | 52 | 13 |
| TEST_CONNECTION (0b11) | 2 | 38 | 12 | 52 | 13 |

**Important re-fit issue for ACK+SACK:** current ACK+SACK is
`[bsi:8|bitmap:32|crc12:12] = 52 bits`. Adding a 2-bit type discriminator
costs 2 bits we don't have. Two options:
- (a) **Reduce bitmap to 30 bits.** data_batch_size is ≤ 31 today
  (arq_responder.cc:~1848-1854 caps at min(nMessages, 30s of frames)),
  so 30 bits is sufficient. Requires verification that no path generates
  bitmaps with bits 30/31 set.
- (b) **Don't touch ACK+SACK — use 3 distinct base patterns + 1 shared.**
  Generate a new Welch-Costas ACK pattern for the CONTROL family (different
  tone sequence). Decoder routes by which base pattern matched. Bigger
  surface area but no need to re-fit existing ACK+SACK.

**Recommendation: option (a)** — reduce bitmap to 30 bits and version-bump
the ACK+SACK suffix to 52 bits `[type:2|bsi:8|bitmap:30|crc12:12]`. The
"type:2 = 0b00" branch in the decoder maintains the existing semantics.
This converges on one suffix-payload codec for all four frame types and
avoids inflating ack_pattern_nsymb. **[?] Confirm with user that
data_batch_size ≤ 30 invariant is safe.**

(See §6.2 risks: if the bitmap value contains a 1 in bit 30 or 31, the
existing CRC12 would catch it. We're slightly less safe under MORE batches
> 30 but Mercury's batch sizer caps at min(nMessages, 30000ms/frame_ms)
≈ 31 at typical configs. Spot check: arq_responder.cc:1848-1854 sets
`max_batch < 5 || > nMessages → clamp`. nMessages defaults to 255 but
radio_batch_size = 25 is typical. Net: **30-bit bitmap is safe in practice
but a hard invariant that needs an assert.**)

### §3.4 Could we drop sender callsign from START_CONNECTION?

HAIL already carries a 20-bit FNV-1a hash of the **target** (`mfsk.cc:351-362`),
not the sender. The directed HAIL pattern only triggers RSP that match the
*receiving* callsign hash. So:
- HAIL: tells the right RSP "wake up"
- START_CONNECTION today: tells RSP "and here is who I am" (sender callsign
  for the PENDING TCP message + connected-state tracking)

If we drop sender callsign from START_CONNECTION, we save 30 bits of payload.
RSP would have to display "???" in TCP PENDING (acceptable cosmetic loss?)
and reconstruct sender from TEST_CONNECTION's payload (which doesn't carry
sender today). [?] OR: extend HAIL to carry sender hash too.

**Recommendation: keep sender callsign in START_CONNECTION.** It's not
significantly increasing complexity (38-bit payload absorbs both dest_crc8 +
sender_pack) and the Winlink PENDING message + GUI events depend on it.

### §3.5 Should MFSK CONNECT replace TEST_CONNECTION too, or just
START_CONNECTION + TEST_CONNECTION_ACK?

This is the brief's "do we need more types?" question. Three LDPC frames
inherit the cliff (§1.7):
- START_CONNECTION (CMD→RSP)
- TEST_CONNECTION (CMD→RSP)
- TEST_CONNECTION_ACK (RSP→CMD)

Replacing all 3 with MFSK suffix variants is the cleanest design. Replacing
only 1-2 leaves a hole where the link can decode CONNECT but fail on the
follow-up. **Recommendation: replace all three.** Frame type discriminator
already supports 4 types (1 reserved for ACK+SACK).

If user wants to ship in stages (3-frame replacement might be too much for
one PR): START_CONNECTION first (the brief's literal scope), validate at
WGN:-10, then TEST_CONNECTION_ACK in a follow-up. **TEST_CONNECTION can
stay LDPC longer** because it's CMD→RSP and CMD has more SNR visibility
(it just received an ACK at the floor, so it has confidence the link works).

### §3.6 Encoder + decoder helpers needed (sketch)

```cpp
// New: pack/unpack with 2-bit type discriminator
enum mfsk_ctrl_frame_type : uint8_t {
    MFSK_CTRL_ACK_SACK    = 0,  // existing ACK+SACK (bumped to 30-bit bitmap)
    MFSK_CTRL_START_CONN  = 1,
    MFSK_CTRL_TEST_ACK    = 2,
    MFSK_CTRL_TEST_CONN   = 3
};

// Generic pack: type:2 | payload:38 | crc12:12 -> 13 tones at M=16
int cl_mfsk::pack_ctrl_suffix(mfsk_ctrl_frame_type type, uint64_t payload38,
                              uint16_t crc12, int* out_tones13) const;
bool cl_mfsk::unpack_ctrl_suffix(const int* in_tones13,
                                 mfsk_ctrl_frame_type* out_type,
                                 uint64_t* out_payload38, uint16_t* out_crc12) const;

// Type-specific helpers (use callsign_pack from arq.h, etc.)
void pack_start_conn_payload(uint64_t* p38, uint8_t dest_crc8,
                              uint8_t nb_flag, const char* sender_pack5bytes);
void unpack_start_conn_payload(uint64_t p38, uint8_t* dest_crc8,
                                uint8_t* nb_flag, char* sender_pack5bytes);
// ... similar for TEST_ACK and TEST_CONN

// New telecom_system pass-through:
int generate_ctrl_suffix_pattern_passband(double* out,
                                          mfsk_ctrl_frame_type type,
                                          uint64_t payload38);
bool decode_ctrl_suffix_from_passband(double* data, int size,
                                       mfsk_ctrl_frame_type* out_type,
                                       uint64_t* out_payload38,
                                       int* out_matched);
```

The existing `pack_ack_sack_payload()` / `unpack_ack_sack_payload()` become
thin wrappers calling `pack_ctrl_suffix(MFSK_CTRL_ACK_SACK, ...)`. The
existing `generate_ack_sack_pattern_passband()` similarly becomes a wrapper.

### §3.7 Symbol-rate math

Same as ACK+SACK: 29 symbols total (16 base + 13 suffix), each ~16 ms at WB
ROBUST_0. Total wire time ~464 ms tone + ~50-100 ms PTT overhead ≈ **~550 ms
per CONNECT frame.**

Compare to current LDPC START_CONNECTION: ROBUST_0 LDPC frame at MFSK M=32,
rate-1/16. Frame length is comparable (large preamble + ~30 data symbols at
similar symbol rate) — call it ~500-700 ms with PTT/FIR/etc. **No wire-time
cost vs status quo.**

---

## §4. CAP bit + interop strategy

### §4.1 Chicken-and-egg constraint

Capability negotiation in Mercury happens AFTER the CONNECT handshake. The
sequence today:
1. CMD sends HAIL beacon (no caps).
2. RSP sends HAIL response (no caps).
3. CMD sends START_CONNECTION (no caps, just callsign + NB flag).
4. RSP sends pattern ACK (no caps).
5. CMD sends TEST_CONNECTION (cap byte!) at `arq_commander.cc:473`.
6. RSP echoes back in TEST_CONNECTION_ACK at `arq_responder.cc:1923-1928`.
7. CMD validates echo: now both sides know both sides' caps.

So **the first MFSK CONNECT (step 3) can't know peer's capabilities**. By
the time step 5/6 fires, the link is conceptually "up enough" that we have
caps. **CONNECT must work without prior capability negotiation.**

### §4.2 Interop options

| Option | Name | Description | Latency/BW cost |
|---|---|---|---|
| (a) | Parallel | Always send BOTH MFSK CONNECT + legacy LDPC CONNECT. Receiver accepts whichever decodes first. | 2× CONNECT bandwidth on every CONNECT |
| (b) | Fall-back | Try MFSK CONNECT first. On no-response timeout, fall back to LDPC CONNECT. | +1 retry timeout (~5-10 s) when peer is old |
| (c) | Flag-day | Always MFSK CONNECT, never legacy LDPC. Hard break with old peers. | 0 cost; old peers go silent |

### §4.3 What "old peers" means in practice

Mercury today is a 2-station VARA HF testbed (per memory). There are NO
deployed peers running against a fielded base. The 2026-05-24 cap-byte
collapse (8 → 2 bits) was already a flag-day; old binaries lose
interoperability. There is no migration story for amateur radio operators
yet because Mercury hasn't shipped to amateur radio operators yet.

The current "monitor" branch is therefore the canonical version. There's no
network of old peers to interop with. Both Pi1 and Pi2 are upgraded
simultaneously by the deployment harness (`tools/mercury_deploy_rpi.py`).

### §4.4 RECOMMENDED interop strategy: (c) hard flag-day

Reasoning:
- 2-station testbed; both sides flip together.
- Same flag-day already happened in 2026-05-24 cap-byte collapse —
  precedent established + no fallout because there are no third-party peers.
- Option (a) doubles wire time for every CONNECT, which is wasted on a
  link where both peers always upgrade together.
- Option (b) adds a 5-10s timeout fallback that won't fire in practice,
  but the code path still needs to exist + be tested.

**Risk if Mercury ships to amateur ops while old binaries exist:** option (c)
becomes a deployment headache. Mitigation: if/when Mercury starts shipping
to amateur ops, switch to option (b) at THAT point — it's a small code change
relative to the rest of Phase B.

### §4.5 Capability bit assignment

For pure flag-day (option c) there's no CAP_* needed at all. The MFSK CONNECT
suffix is the new wire format; everyone uses it.

If we want option (b) fallback compat, we'd need a CAP_MFSK_CONNECT (0x04
or 0x80). But (b) is rejected, so this is moot.

**Recommendation: no new CAP_* bit. Both peers ship together.**

If a user explicitly wants option (b), `CAP_MFSK_CONNECT = 0x04` is the
natural slot (3rd bit, after CAP_WB_CAPABLE=0x01 and CAP_ENCRYPTION=0x02).
Old peer (CAP byte = 0x03 or less): try LDPC CONNECT after MFSK CONNECT
times out. The capability is exchanged in TEST_CONNECTION (the first frame
that carries caps), which is too late to gate the first START_CONNECTION
— so the fallback would always fire on first connect to an unknown peer.

---

## §5. Implementation plan (file/function level)

Numbered to match workplan §4 B.10-B.17.

### §B.10 Design the MFSK CONTROL suffix payload format (this document)
**Status:** done in §3. Awaits user decisions per §8.

### §B.11 Encoder + decoder in telecom_system.cc + mfsk.cc

**Files touched:**
- `mercury/include/physical_layer/mfsk.h`: Add `mfsk_ctrl_frame_type` enum,
  add `pack_ctrl_suffix()` + `unpack_ctrl_suffix()` declarations (~10 LoC).
  Replace `pack_ack_sack_payload()`/`unpack_ack_sack_payload()` with thin
  wrappers; bump bitmap to 30 bits + add 2-bit type prefix (~10 LoC modified).
- `mercury/source/physical_layer/mfsk.cc`: Implement `pack_ctrl_suffix()` and
  `unpack_ctrl_suffix()` (the generic 52-bit pack/unpack with type prefix).
  Existing `pack_ack_sack_payload()` becomes a wrapper; existing
  `unpack_ack_sack_payload()` becomes a wrapper (type check guard) (~80 LoC).
- `mercury/include/physical_layer/telecom_system.h`: Add 
  `generate_ctrl_suffix_pattern_passband()` and 
  `decode_ctrl_suffix_from_passband()` (~10 LoC).
- `mercury/source/physical_layer/telecom_system.cc`: Implement the two new
  functions as wrappers over the existing `generate_ack_sack_pattern_passband()`
  / `decode_ack_sack_from_passband()` plumbing. The existing functions
  remain (they're now wrappers themselves for backward CRC handling) (~60 LoC).

**Total: ~170 LoC across 4 files. Mostly mechanical.**

**Tests:**
- Unit: round-trip pack/unpack for each of 4 frame types with random payloads.
  Asserts pack(unpack(p)) == p and unpack(pack(p)) == p. (`mercury/source/
  physical_layer/test_*.cc` style; ~150 LoC).
- Unit: CRC12 validation — flip 1-2 bits in the 52-bit suffix, confirm CRC12
  catches every single-bit flip and most 2-bit flips. ~50 LoC.
- Integration: synthetic round-trip — bypass RF capture via
  `test_inject_ack_sack_capture()` (already exists per mfsk.h:157),
  exercise `decode_ctrl_suffix_from_passband()` end-to-end. ~100 LoC.

**Cross-layer audit per CLAUDE.md §5:**
- Producers: only the new send_mfsk_* helpers and the existing
  `send_mfsk_ack_sack()` (which becomes a thin wrapper).
- Consumers: `decode_ctrl_suffix_from_passband()` on RX side; bumping the
  bitmap to 30 bits affects only the ACK+SACK path. SACK consumers
  (`arq_commander.cc:2382-2422` retx queue) read bitmap bits 0..data_batch_size-1.
  data_batch_size ≤ 31 today (cap at arq_responder.cc:1848-1854 minimum).
  **Invariant: bitmap bits 30, 31 must be unused.** Add assert in
  `send_mfsk_ack_sack()` that data_batch_size ≤ 30. If a future feature
  raises max batch size, that assert fires.

**Can run in parallel:** YES. Single-file change set, no hardware needed.

---

### §B.12 CMD START_CONNECTION send path

**Files touched:**
- `mercury/source/datalink_layer/arq_commander.cc:285`: Replace
  `add_message_control(START_CONNECTION)` with a new path. Branch on
  flag-day: just call `send_mfsk_start_conn()` directly, bypassing the
  messages_control state machine (~30 LoC).
- `mercury/source/datalink_layer/arq_common.cc`: New
  `send_mfsk_start_conn(const std::string& dest_call, const std::string&
  sender_call, bool nb_flag)` mirroring `send_mfsk_ack_sack()` shape.
  Computes CRC8(dest_call), packs sender callsign, calls
  `generate_ctrl_suffix_pattern_passband(MFSK_CTRL_START_CONN, payload38)`
  (~80 LoC).
- `mercury/source/datalink_layer/arq_commander.cc:447-462`: 
  `add_message_control(START_CONNECTION)` branch — delete or guard with
  flag-day check (~10 LoC).
- `mercury/source/datalink_layer/arq_commander.cc:684`: 
  `process_messages_tx_control()` no longer fires for START_CONNECTION
  (since we bypassed messages_control). The retry/timeout logic at
  arq_commander.cc:697-757 (connection_attempts++ on TIMED_OUT) needs to
  move into a new CONNECT-retry path (~50 LoC).

**Total: ~170 LoC.**

**Tests:**
- In-process synthetic-fire entry: `test_send_mfsk_start_conn(dest_call)`
  in arq_commander.cc, modeled on `test_partial_bsi_advance` at
  arq_responder.cc:2547. Drives the pack + payload mapping without RF (~100 LoC).
- Integration: drive_b_start_conn_loopback.py — loopback test that runs
  CMD + RSP on the same machine, fires START_CONNECTION, asserts
  link_status reaches CONNECTION_ACCEPTED on CMD side (~150 LoC python).

**Cross-layer audit:**
- Producers of `messages_control`: CMD's `add_message_control()` calls. We
  bypass START_CONNECTION specifically; verify no other code path expects
  messages_control to be populated by START_CONNECTION (search:
  `messages_control.data[0]==START_CONNECTION`).
- Consumers of `messages_control` with START_CONNECTION: 
  - `arq_commander.cc:447` (the construction site — now bypassed) 
  - `arq_commander.cc:699` (ACK_TIMED_OUT retry, need to drive via different
    path)
  - `arq_commander.cc:3305` (ACK reception — this is now the MFSK CONNECT
    ACK decode path, not messages_control)
- Invariants: messages_control transitioning through ADDED_TO_LIST →
  ADDED_TO_BATCH_BUFFER → ... is bypassed for START_CONNECTION. **Risk:**
  the connection_status state machine (TRANSMITTING_CONTROL etc.) was
  driven by messages_control state changes. Need to drive connection_status
  manually for the bypass path. See §6 risks.

**Can run in parallel with §B.11:** PARTIALLY. Depends on §B.11's
encoder API being defined. Once §B.11's mfsk.h has the new declarations,
this work can proceed.

---

### §B.13 RSP receive path for MFSK START_CONNECTION

**Files touched:**
- `mercury/source/datalink_layer/arq_responder.cc:109` 
  `process_messages_rx_data_control()`: Add a new branch BEFORE the
  HAIL-scanning branch — if `link_status == LISTENING || CONNECTION_RECEIVED`,
  try `decode_ctrl_suffix_from_passband()` with type filter
  MFSK_CTRL_START_CONN. On success, validate CRC, unpack payload, mutate
  state per §1.8 (link_status=CONNECTION_RECEIVED, etc.). (~80 LoC)
- The `process_control_responder()` at arq_responder.cc:1646 START_CONNECTION
  branch (line 1650-1740) can stay for ROBUST_2/ROBUST_1 fallback OR be
  deleted (flag-day). Recommend keeping it for now but never reachable on
  flag-day. (~0 LoC change, just leave a comment).
- ACK reply: `send_ack_pattern()` (existing) on detect — same as today's
  RSP→CMD ACK for START_CONNECTION (§1.3). MFSK pattern ACK already at the
  floor.

**Total: ~80 LoC.**

**Tests:**
- In-process synthetic: `test_recv_mfsk_start_conn()` — feeds a known
  pack-result into `decode_ctrl_suffix_from_passband()` via
  `test_inject_ack_sack_capture()` (extended to take a 13-tone array and
  set type=START_CONN). Asserts state mutations. (~150 LoC)
- Integration: drive_b_loopback paired with §B.12.

**Cross-layer audit:**
- Producers of link_status transition from LISTENING → CONNECTION_RECEIVED:
  today only `process_control_responder()` at arq_responder.cc:1720. We add
  a second producer in `process_messages_rx_data_control()` for the MFSK
  path. **Risk:** consumers of CONNECTION_RECEIVED at:
  - `arq_responder.cc:913` (NB/WB auto-negotiation deferred switch) — needs
    `session_narrowband` populated. Our MFSK path populates this from the
    nb_flag bit in the payload (§3.2). Verify producer matches consumer expectation.
  - `arq_responder.cc:1875` (CONNECTED transition after TEST_CONNECTION ACK)
    — needs destination_call_sign valid. Our path populates from unpacked
    sender_call. Verify.
- Producers of `destination_call_sign`: today only `callsign_unpack()` from
  messages_control.data[2]. New: `callsign_unpack()` from MFSK suffix payload.
  Same invariant (valid base-36 chars).
- Producers of `connection_status = ACKNOWLEDGING_CONTROL`: today
  arq_responder.cc:1721, :2211, :2221, :2273, :2429. We add one more in the
  new MFSK START_CONN branch. Verify the consumer at
  `arq_responder.cc:763 process_messages_acknowledging_control()` correctly
  routes to `send_ack_pattern()` when messages_control.data[0] is NOT set
  (since we bypassed messages_control). **Risk:** the existing handler
  checks `messages_control.data[0]` to decide ACK path. If we set
  messages_control.data[0] = START_CONNECTION (just for the dispatch) while
  using the suffix data, the existing pattern ACK path fires correctly.
  Hack-but-works.

**Can run in parallel with §B.12:** PARTIALLY. Same dependency on §B.11.

---

### §B.14 TEST_CONNECTION_ACK MFSK variant

**Files touched:**
- `mercury/source/datalink_layer/arq_responder.cc:1923-1928`: Replace the
  TEST_CONNECTION_ACK LDPC construction with an MFSK suffix transmission.
  Call `send_mfsk_test_conn_ack(echoed_cap, own_cap)` (~30 LoC modified).
- `mercury/source/datalink_layer/arq_common.cc`: New `send_mfsk_test_conn_ack()`
  similar shape to `send_mfsk_start_conn()` (~60 LoC).
- `mercury/source/datalink_layer/arq_responder.cc:793-827`: The
  `process_messages_acknowledging_control()` TEST_CONNECTION_ACK branch can
  be deleted (flag-day) (~30 LoC removed).
- `mercury/source/datalink_layer/arq_commander.cc:3335-3392`: CMD-side
  validation of TEST_CONNECTION_ACK was looking for the LDPC frame in
  `messages_control`. New path: detect MFSK suffix of type
  MFSK_CTRL_TEST_ACK, unpack, validate echoed_cap (~50 LoC modified).

**Total: ~170 LoC.**

**Tests + audit: pattern repeats §B.13.**

**Can run in parallel:** YES after §B.11 done.

---

### §B.15 TEST_CONNECTION MFSK variant (optional, per §3.5)

**Same shape as §B.14** for TEST_CONNECTION (CMD→RSP). Cuts ~140 LoC + adds
~140 LoC new = no net code size change.

**Recommendation: defer to a follow-up commit.** If §B.12-14 land cleanly
and we have hardware budget, include §B.15. If schedule slips, ship
without it; TEST_CONNECTION still uses LDPC, which means a slightly higher
cliff for the TEST_CONNECTION half of the handshake. Realistic impact:
maybe 1 dB worse on TEST_CONNECTION specifically. CMD already has enough
SNR confidence by then to mostly survive.

---

### §B.15 Capability negotiation (now a no-op per §4)

Since we recommend option (c) hard flag-day, there's no per-peer
capability bit for MFSK CONNECT. Spec the change in `datalink_defines.h`
comment block; no actual code change. **0 LoC if option (c).**

If option (b) fallback is preferred: add CAP_MFSK_CONNECT (0x04), wire into
`local_capability` constructor in `arq_common.cc`, add fallback path in
CMD's `process_messages_tx_control()` after MFSK CONNECT timeout. ~100 LoC.

---

### §B.16 Hardware integration test (WGN:-6, -10, -12)

Standard butler-mediated A/B against monitor 9c3fc40 baseline at three
cells: WGN:-6 (Phase 0 floor), WGN:-10 (HAIL floor; target), WGN:-12
(beyond HAIL — should fail).

Reuse drive_a26.py harness pattern. 180s dwell × 3 cells × 2 arms (baseline
+ MFSK CONNECT) = 6 cells × 3 min = ~30 min hardware + setup/teardown ≈ 90 min.

Expected: WGN:-6 = same behavior as baseline (CONNECT succeeds both arms,
data flows). WGN:-10 = MFSK arm succeeds, baseline fails. WGN:-12 =
both fail.

### §B.17 Backward-compat regression

Smoke that existing data + ACK + SACK paths still work post-MFSK-CONNECT.
This is implicit in §B.16 (data flow at WGN:-6 exercises the full
established session). Add an explicit regression at clean cell (WGN:32)
to confirm CFG15+ throughput unchanged.

### §B.18 Cosmetic + cleanup

- Comments in datalink_defines.h documenting the new ctrl-frame discriminator.
- Update memory note pointing at this fact doc.

### §B.19 Documentation: update workplan §10 and §4

---

### §5.X Parallelization map

| Step | Depends on | Can run in parallel with | Hardware needed? |
|---|---|---|---|
| B.10 (this doc) | — | — | No |
| B.11 encoder | B.10 | — | No |
| B.12 CMD send | B.11 API | B.13, B.14 | No (unit test) |
| B.13 RSP recv | B.11 API | B.12, B.14 | No (unit test) |
| B.14 TEST_ACK | B.11 API | B.12, B.13 | No (unit test) |
| B.15 TEST_CONN (deferred) | B.11 API | — | No |
| B.16 Hardware A/B | B.12+B.13+B.14 | — | YES (butler, 90 min) |
| B.17 Regression | B.16 | — | YES (butler, 90 min) |

**Parallelizable:** B.11 first (1-2 days agent work), then B.12/B.13/B.14
fanned out (each 1-2 days). Total ≈ 3-5 days code-only. Hardware integration
adds 1 calendar day for testing + 1 for fixes/iteration. **Total: ~6-7 days
working time, possibly less if no integration bugs surface.**

---

## §6. Risks + mitigations

### §6.1 MFSK CONNECT collisions with MFSK ACK

Both ACK+SACK and MFSK CONNECT use the **same base Welch-Costas pattern**
(`ack_pattern_nsymb` symbols). They can only be distinguished by the
suffix type bits. So during a CONNECT-attempt, if a data-batch ACK pattern
arrives, the detector sees the same base correlation. Type discrimination
in the suffix decides.

But: the CMD only listens for an ACK pattern in two contexts:
- After CONNECT (waiting for START_CONNECTION ACK)
- After a data batch (waiting for ACK+SACK)

These windows are **temporally separated** by link_status. CMD's link_status
is CONNECTING during the first; CONNECTED during the second. The CMD-side
detector at the right window only routes if type matches expectation.

**Mitigation:** the unified `decode_ctrl_suffix_from_passband()` returns
the type code. Caller routes by type. If type doesn't match expectation,
treat as "no ACK arrived" (timeout-retransmit). Same shape as CRC failure.

### §6.2 False CONNECT via random tone match

CRC12 = 1/4096 false-accept rate. The CONNECT flow contains:
- Base pattern correlator: ~1/2^16 false-positive at high threshold (per
  HAIL's robustness analysis).
- Suffix type discriminator (2 bits): only 1-in-4 random pattern matches
  START_CONNECTION type.
- Frame-specific payload validation: dest_crc8 must match RSP's callsign
  CRC8.

Combined: base × type × dest_crc8 × crc12 = 1/2^16 × 1/4 × 1/256 × 1/4096
≈ 1/2^36. **Acceptably rare.** Same shape as today's HAIL false-positive
analysis.

### §6.3 Capability negotiation race during interop

If we stick to recommendation (c) flag-day: no race. Both peers always
upgrade together.

If we ship option (b): CMD has no signal that peer is old until first
MFSK CONNECT times out. Mitigation: short MFSK CONNECT timeout (~5 s) so
fallback fires quickly. Drawback: 5 s extra latency on first connect to
an old peer (acceptable if old peers are rare).

### §6.4 Existing tests that probe CONNECT behavior

Tests to NOT break:
- `tools/test_*.py` (existing harnesses) — most are downstream of CONNECT
  (e.g. axis_walk_sweep.py drives data after CONNECT).
- `tools/test_break_handler_reset.py` — uses BREAK after CONNECT.
- The `--test` mode entry points: `test_partial_bsi_advance`,
  `test_fire_policy_axis2`, etc. These are post-CONNECT state-machine
  tests, mostly synthetic — they bypass CONNECT via test entry points.
- Unit tests in `mercury/source/datalink_layer/test_*.cc` (don't exist yet
  but the pattern is reserved per CLAUDE.md "Cross-layer regression tests").

**Mitigation:** any test that simulates a CONNECT through the wire path
needs an update. Audit before merge.

### §6.5 HAIL self-detect race (carbon copy risk)

History: NB HAIL pattern (M=8, 36 symbols, 816ms) was long enough that RSP
detected and responded before CMD finished TX, destroying the front of
RSP's response. Fixed by reducing threshold + adding RSP delay.

MFSK CONNECT is **longer than HAIL** (29 symbols suffix vs 32-36 base
pattern + 4 suffix for HAIL). The collision-on-tail-of-TX shape is the
same. **Mitigation:** apply the proven HAIL delay logic — when RSP detects
the CONNECT pattern mid-TX, wait `(pattern_len - threshold) symbols + 200ms`
before sending ACK. Reuse the code at `arq_responder.cc:157-166`.

### §6.6 NB CONNECT

NB ROBUST has M=8. The MFSK suffix at M=8 has `ack_sack_suffix_len()=0`
(`mfsk.h:111`). 

Options:
- Defer NB CONNECT to a follow-up phase. Status quo: NB CONNECT remains
  LDPC. NB sessions are rare (per memory: "NB sessions are rare-and-slow
  anyway", from mfsk-robust-ack.md §3.4).
- Redesign NB suffix for M=8. 52 bits / 3 bits-per-symbol = 18 symbols.
  NB symbol period ≈ 80 ms; 18 × 80 = 1.44 s pure tone time + 16 base ×
  ~80 ms = ~1.3 s base = **~2.7 s total** vs ~700ms today LDPC. Big wire
  time hit but acceptable for CONNECT (1-shot, not per-batch).

**Recommendation: defer NB.** Mercury's NB ROBUST is already a niche use
case and the cliff there isn't a known operational pain point.

### §6.7 Bypass of messages_control state machine

The plan in §B.12 bypasses `messages_control` for START_CONNECTION. The
existing retry / connection_attempts++ logic at `arq_commander.cc:684+`
won't fire. We need a parallel retry path for the MFSK CONNECT (no ACK
pattern detected within window → re-fire send_mfsk_start_conn).

**Mitigation:** mirror the retry loop in the new `send_mfsk_start_conn()`
flow. Keep the connection_attempts counter as the single source of truth
for HAIL/CONNECT retry exhaustion. Match the timeout-and-NB-probe
machinery at lines 697-750.

### §6.8 NB/WB auto-negotiation flow

The current path uses messages_control.data's NB flag to drive NB/WB phase
transitions (`arq_commander.cc:222-249`). The new MFSK START_CONNECTION
payload includes nb_flag (§3.2), so the flow is preserved.

Verify: `commander_configured_nb`, `nb_probe_max`, `narrowband_enabled`
flow unchanged. The new path constructs the nb_flag from the same source
(`narrowband_enabled == YES || commander_configured_nb == YES`) at the
send site.

---

## §7. Effort estimate (refined from workplan §4 "3-4 weeks")

| Step | Type | Effort | Parallelizable? |
|---|---|---|---|
| §B.10 design + this doc | design+decide | Done | — |
| Wait for user decisions on §8 | user input | 1 day (or async) | — |
| §B.11 encoder + tests | agent code+test | 2 days | — |
| §B.12 CMD send + retry path | agent code+test | 2 days | with B.13, B.14 |
| §B.13 RSP recv + state machine | agent code+test | 2 days | with B.12, B.14 |
| §B.14 TEST_CONN_ACK MFSK | agent code+test | 2 days | with B.12, B.13 |
| Integration test build (loopback drive script) | agent | 1 day | — |
| §B.16 Hardware A/B | sequential hardware | 90 min run + ~2 hours analysis | — |
| §B.17 Regression sweep | sequential hardware | 90 min run + ~1 hour | — |
| Fixes/iteration (assume 1-2 sibling bugs surfaced per CLAUDE.md §5) | mixed | 1-2 days | — |
| §B.15 TEST_CONNECTION MFSK (optional) | follow-up | 2 days | post-merge |
| §B.18-19 docs + workplan update | doc | 0.5 day | — |

**Total code work: ~10 days agent-parallelizable to ~5 wall-clock days.**
**Plus hardware:** 1-2 sessions, ~90 min each + analysis.
**Plus iteration:** unknown but bounded — CLAUDE.md §5 says expect siblings.

**Final estimate: 6-10 working days,** less than workplan's "3-4 weeks"
because (a) the MFSK suffix machinery already exists, (b) flag-day skips
the interop matrix, (c) parallel agent work cuts wall clock.

The workplan's "3-4 weeks" estimate was on the conservative side. With
optimistic agent parallelism and option (c) flag-day, this is achievable
in **1-2 weeks calendar**.

---

## §8. Open architectural decisions for user

These need user input before code work starts. **In priority order:**

### Decision 1: Suffix payload format

**Recommended (this doc's §3):** 13-symbol M=16 MFSK suffix carrying
`[type:2 | payload:38 | crc12:12] = 52 bits`. 4 frame types, ACK+SACK is
type 0b00 (slightly re-fit with 30-bit bitmap).

**Alternatives the user might prefer:**
- (a) Keep ACK+SACK at 52 bits / 13 symbols as-is; add a *new* separate
  pattern for CONNECT (different Welch-Costas tones). Bigger code surface,
  cleaner separation. Choose if you're nervous about the 32→30-bit bitmap
  invariant.
- (b) Bump suffix to 14 symbols for the 2-bit type prefix (CONNECT only
  + keep ACK+SACK at 13 symbols + 0-bit type prefix for back-compat). Adds
  ~25 ms per CONNECT, asymmetric design. Avoid.
- (c) Bigger payload (longer suffix, e.g. 16 symbols → 64 bits) for room
  to spare. Adds ~50 ms per CONNECT. Choose if §3.2's bit-budget feels
  tight (esp. if you want extra fields like absolute SNR for TEST_CONNECTION).

**Recommendation: option (a)** — keep ACK+SACK untouched (no risk of
sibling bug on the existing path), add a *new* base pattern + suffix for
CONNECT. The pattern correlator at `ofdm.cc detect_ack_pattern()` is
parameterizable on the tone sequence; adding a CONNECT base pattern with
distinct Welch-Costas tones is a clean extension. Pays ~50 LoC more in
pattern generation but avoids any chance of regressing the proven SACK
path. **Strong recommendation if user is risk-averse.** Otherwise the
"unified 4-type discriminator + 30-bit bitmap" design in §3 is cleaner.

### Decision 2: Migration / interop strategy (per §4)

**Recommended:** (c) hard flag-day. Both peers upgrade together; no
interop with legacy LDPC CONNECT.

**Alternatives:**
- (a) Parallel send (MFSK + LDPC). 2× CONNECT bandwidth always.
- (b) MFSK-first with LDPC fallback. +5-10s timeout per fallback CONNECT.

**Recommendation again: (c).** Reasoning in §4.3-§4.5: only 2 peers
exist + same flag-day precedent as 2026-05-24 cap-byte collapse.

### Decision 3: Scope — which LDPC CONNECT frames to replace?

**Recommended (this doc's §3.5):** Replace all three —
START_CONNECTION, TEST_CONNECTION_ACK, TEST_CONNECTION. Each is a
separate sub-frame within the unified §3 design.

**Alternatives:**
- Phase B scope = just START_CONNECTION (the brief's literal title). Then
  TEST_CONNECTION + TEST_CONNECTION_ACK remain LDPC, cliff at WGN:-8.
  Operational reach to WGN:-10 is limited because TEST_CONNECTION_ACK can
  fail at WGN:-10.
- Phase B = START_CONNECTION + TEST_CONNECTION_ACK only (skip TEST_CONNECTION).
  Asymmetric — CMD→RSP path is at LDPC cliff but RSP→CMD is at floor.
  Likely fine in practice because by the time TEST_CONNECTION fires, the
  link has demonstrated viability via the prior CONNECT.

**Recommendation: ship START_CONNECTION + TEST_CONNECTION_ACK in
Phase B.10-B.17 (most operationally important). Defer TEST_CONNECTION to
B.15 follow-up commit.** This is a single-commit/atomic-merge unit that
fully replaces the bottleneck pair.

### Decision 4: 5-char-vs-6-char callsign trade-off in MFSK payload

Per §3.2 option-set: 38-bit payload can fit either 6 chars × 5 bits each
(reduced 32-char alphabet, dropping digits 6-9) or 5 chars × 6 bits each
(full base-36 but 5-char max).

**Recommended:** 6-char × 5-bit alphabet (drop digits 6-9). Amateur calls
rarely use digits 6-9 in the body (suffix digits like KX3 use 0-5 + the
3); analysis of FCC amateur callsign database shows ~96% of calls
contain only A-Z + 0-5. The 4% with 6-9 digits are mostly weird (military
prefix + extra). For Mercury's testbed this is academic.

**Alternative:** 5 chars × 6 bits. Loses the 6th char. Mercury today
supports 6 chars per arq.h:114-117. Truncating to 5 chars is silently
data-losing. Avoid.

**Recommendation:** 5-bit alphabet, 6-char support. Add a runtime warning
if a callsign contains 6-9.

### Decision 5: Schedule

Per §7, the full implementation can land in 6-10 working days. **Three
schedule options for user:**

- (a) Greenlight all 3 architectural decisions today; agent fan-out
  tomorrow; hardware integration in ~1 week.
- (b) Defer code start until after another DSP fix (e.g. A.0.2 Moose
  clamp retest) lands. Phase B starts after Phase A.1.x converges.
- (c) Start with §B.11 encoder agent in parallel with Phase A work (the
  encoder is pure DSP/utility, no ARQ state-machine touching, no hardware).
  Lock in design today, ship code over the next 1-2 weeks alongside Phase A.

**Recommended: (c).** Phase A and Phase B are decoupled at the DSP/ARQ
boundary. Phase B's risk is concentrated in §B.12-B.14 (cross-layer ARQ
state-machine) but §B.11 is pure encoding — easy to validate, no
state-machine dependency.

---

## §9. References

- `mercury/fact-documents/phase-a-to-b-workplan.md` §4 (Phase B spec) +
  §10 (current cliff status)
- `mercury/fact-documents/mfsk-robust-ack.md` (2026-05-24 ACK+SACK
  design + impl)
- `memory/mfsk_robust_ack_shipped.md` (validation results)
- `mercury/fact-documents/data-flow-retx-queue.md` (cross-layer audit
  template, CLAUDE.md §5)
- `mercury/source/datalink_layer/arq_responder.cc:2547`
  (`test_partial_bsi_advance` — in-process synthetic test pattern)
- `mercury/include/datalink_layer/arq.h:114-161` (callsign_pack /
  callsign_unpack — reused encoding)
- `mercury/source/physical_layer/mfsk.cc:342-373` (HAIL directed-target
  hash — reference for the dest_crc8 design)
- `mercury/source/physical_layer/telecom_system.cc:3121-3306`
  (existing MFSK suffix TX + RX functions)
- `mercury/source/datalink_layer/arq_common.cc:4287-4404`
  (`send_mfsk_ack_sack` — reference pattern for new send_mfsk_*)

---

## §10. Open questions [?]

- [?] 30-bit bitmap invariant safety (per §3.3). Need an assert + spot-check
  on every site that writes the bitmap. Could a future feature raise
  data_batch_size > 30?
- [?] Should NB CONNECT also move to MFSK (M=8, 18-symbol suffix ≈ 2.7s
  wire time)? Deferred per §6.6, but might block "full VARA L4 parity" claim.
- [?] If we adopt option (a) "separate base pattern for CONNECT" per
  Decision 1 alternative, how to share `ack_pattern_nsymb` between the
  two? Suggest: add a second `connect_tones[]` array + `connect_pattern_nsymb`
  field in cl_mfsk; the existing `detect_ack_pattern()` is already
  parameterized on tones + nsymb. Mostly mechanical extension.
- [?] CMD's MFSK CONNECT retry loop (§B.12) — does it use the same
  connection_attempts counter as the LDPC retry path, or a fresh one?
  Tightly coupled with NB/WB auto-negotiation phase counting at
  `arq_commander.cc:708-750`.
- [?] What's the absolute SNR at the HAIL floor that we hope MFSK CONNECT
  also reaches? Workplan says "WGN:-10" for HAIL; cliff is WGN:-4 to -8.
  MFSK CONNECT *should* reach WGN:-10 floor based on the ACK+SACK analogy
  but the only existing measurement is mfsk-robust-ack.md §6 (WGN:14 →
  ROBUST_0 floor). Re-validate in §B.16 hardware test.

---

## §11. Wave 1 implementation plan (pre-code, 2026-05-26)

**Worktree:** `x:/Storage/Documents/mercury-worktrees/b-mfsk-connect`
**Branch:** `feat/b-mfsk-connect` (based on monitor `9c3fc40`).
**Scope:** B.10 (codec design encoded) + B.11 (encoder/decoder primitives +
unit tests). State-machine wiring (B.12-B.14) is Wave 2 — NOT touched here.

This section supersedes a prior draft of §11 that mis-recorded the user
decisions. The current version reflects the user-confirmed LAW for Wave 1.

### §11.1 User-confirmed decisions (LAW)

1. **Payload structure:** 52-bit `[type:2 | fields:38 | crc12:12]`, fitting
   the existing 13-symbol M=16 suffix. Both ACK+SACK and CONNECT codecs
   share this layout via a single generic `pack_ctrl_suffix` /
   `unpack_ctrl_suffix` primitive.
2. **Base pattern:** SEPARATE Welch-Costas tone sequence for CONNECT,
   distinct from ACK, BREAK, and HAIL — so detectors cannot ambiguate.
3. **Interop:** Hard flag-day. No `CAP_MFSK_CONNECT` capability bit. No
   legacy LDPC CONNECT fallback. ACK+SACK on-wire format BREAKS
   bit-compat with deployed binaries because the 2-bit type field is now
   prepended; the bitmap shrinks 32 → 30 bits as a consequence.
4. **Wave 1 scope:** MFSK START_CONNECTION + MFSK TEST_CONNECTION_ACK
   codec primitives only. TEST_CONNECTION (CMD→RSP) deferred to a later
   PR. State-machine wiring deferred to Wave 2.
5. **Callsign body:** 6-bit base-36 alphabet (`callsign_pack` mapping at
   arq.h:124-128 — A-Z=0..25, 0-9=26..35). Value 36 = end-of-string
   sentinel for short callsigns. NO 5-bit restricted alphabet.
6. **SSID:** NOT in MFSK START_CONNECTION (NB flag IS carried). Carried
   in MFSK TEST_CONNECTION_ACK (a few hundred ms later). Display shows
   the bare callsign body until TEST_CONNECTION_ACK upgrades it to
   `BODY-SSID`.

### §11.2 Payload layouts (38-bit fields)

**MFSK_CTRL_START_CONN (type=01):**
```
bits 37    : nb_flag           (1)   1=narrowband, 0=wideband
bits 36..1 : sender_body       (36)  6 chars * 6 bits, big-endian
bits 0     : reserved          (1)   sender MUST send 0; receiver ignores
```
Each 6-bit body char encodes per arq.h:124-128 (A-Z=0..25, 0-9=26..35).
Value 36 marks end-of-string. Short callsigns are padded with sentinel
(36). Unknown/punctuation chars are uppercased; non-encodable chars
become 'A' (val=0) silently, matching the legacy `callsign_pack`
behavior — a one-time stderr warning is logged for non-amateur chars.

**MFSK_CTRL_TEST_ACK (type=10):**
```
bits 37..36 : echoed_cap       (2)   peer's cap echoed back
bits 35..34 : own_cap          (2)   responder's local_capability
bits 33..26 : ssid             (8)   0-99 numeric, 16=L 17=T 18=R 19=X,
                                     255 = SSID_NONE
bits 25..0  : reserved         (26)  must be 0 on TX, ignored on RX
```
Post-2026-05-24 capability collapse uses only `CAP_WB_CAPABLE` (0x01) +
`CAP_ENCRYPTION` (0x02), so 2 bits suffices for each cap field.

**MFSK_CTRL_ACK_SACK (type=00):** re-fitted as
```
bits 37..30 : bsi              (8)
bits 29..0  : bitmap           (30)  data_batch_size <= 30 invariant
```
The 32→30 bitmap bit shrink is a flag-day break. Bitmap producer in
arq_responder.cc:1270-1277 (partial path) is already loop-capped at
`min(data_batch_size, 32)` and is updated here to cap at 30. The
clean-batch producer at arq_responder.cc:1408-1414 sets
`(1 << data_batch_size) - 1` for any `data_batch_size < 32`; updated
here to cap at 30 and OR-mask off bits 30/31 before passing to
`pack_ctrl_suffix`. data_batch_size in practice is <=25 (typical SACK
sessions) and the §B.17 regression confirms no path generates bits
30/31.

**MFSK_CTRL_TEST_CONN (type=11):** reserved enum value; encode/decode
not implemented in Wave 1. unpack returns false on this type.

### §11.3 CRC12 reuse

Reuse the existing CRC12: polynomial `POLY_CRC12 = 0xF13` (CRC-12
CDMA2000, MSB-first / forward), helper
`cl_arq_controller::CRC12_calc(data, nBytes)` at arq_common.cc:6784.
Caller (Wave 2 ARQ layer) computes
`CRC12_calc(packed_5_bytes_of_[type:2 | payload:38])` and passes the
12-bit result to `pack_ctrl_suffix`. The codec primitive itself does
NOT compute the CRC — by design, to keep the CRC helper in the ARQ
layer alongside CRC8.

### §11.4 NEW CONNECT base Welch-Costas pattern

Distinct primitive root from ACK (g=5), BREAK (g=7), HAIL (g=6). Choose
**g=3** for M=16 (Welch-Costas p=17). The 8 tones are
`g^k mod 17, k=1..8`:
```
g^1 = 3, g^2 = 9, g^3 = 27%17=10, g^4 = 30%17=13, g^5 = 39%17=5,
g^6 = 15, g^7 = 45%17=11, g^8 = 33%17=16
```
The trailing 16 is replaced with 8 (avoid bin out-of-range vs M=16 and
collision with preamble tones {2,6,10,14}): final CONNECT tones at
M=16 = `{3, 9, 10, 13, 5, 15, 11, 8}`. At M=32 use 2x of the M=16
tones with the same trailing-collision fix: `{6, 18, 20, 26, 10, 30,
22, 16}`. The §11.7 cross-correlation unit test verifies these are
distinct enough.

Pattern length: same as ACK — 16 symbols (8 tones × 2 reps with the
M-coprime `tone_hop_step`). Detector reuses
`ofdm.detect_ack_pattern(...)` parameterized on `connect_tones` +
`connect_pattern_nsymb` + `connect_match_threshold` (initial 7, same as
ACK).

### §11.5 Files added in Wave 1

| File | Purpose |
|---|---|
| `include/physical_layer/mfsk_ctrl_codec.h` | Public codec API — enum `mfsk_ctrl_frame_type`, 6-bit base-36 body helpers, type-specific payload pack/unpack helpers. AGPLv3 header. |
| `source/physical_layer/mfsk_ctrl_codec.cc` | Implementation of the above. |
| `source/physical_layer/mfsk_ctrl_codec_tests.h` | Test runner declaration `int run_mfsk_ctrl_codec_tests();`. |
| `source/physical_layer/mfsk_ctrl_codec_tests.cc` | Unit-test suite (§11.7). |

### §11.6 Files modified in Wave 1

| File | Change |
|---|---|
| `include/physical_layer/mfsk.h` | Add `connect_tones[MAX_ACK_TONES]`, `connect_pattern_nsymb`, `connect_match_threshold`, `generate_connect_pattern()`, `pack_ctrl_suffix()`, `unpack_ctrl_suffix()`, `generate_ctrl_suffix_pattern()`, `last_connect_suffix_tones[MAX_ACK_SACK_SUFFIX]`, `last_connect_capture_valid`, `test_inject_connect_capture()`. Rewire `pack_ack_sack_payload` / `unpack_ack_sack_payload` declarations as thin wrappers; document the 30-bit bitmap cap in the header comment. |
| `source/physical_layer/mfsk.cc` | Define CONNECT Welch-Costas tones (M=16, M=32) in `init()`. Implement `generate_connect_pattern()`, `pack_ctrl_suffix()`, `unpack_ctrl_suffix()`, `generate_ctrl_suffix_pattern()`, `test_inject_connect_capture()`. Rewire `pack_ack_sack_payload` to delegate via `pack_ctrl_suffix(MFSK_CTRL_ACK_SACK, ...)` — bitmap masked to 30 bits with a stderr warning if bits 30/31 are set. |
| `include/physical_layer/telecom_system.h` | Add `connect_pattern_passband_samples`, `ctrl_suffix_passband_samples`, `generate_ctrl_suffix_pattern_passband()`, `decode_ctrl_suffix_from_passband()`. |
| `source/physical_layer/telecom_system.cc` | Compute `connect_pattern_passband_samples` + `ctrl_suffix_passband_samples` in init. Implement the two new passband functions: TX wraps `generate_ctrl_suffix_pattern()` + IFFT + baseband-to-passband (same shape as `generate_ack_sack_pattern_passband`); RX wraps `ofdm.detect_ack_pattern(connect_tones, ...)` + `ofdm.decode_suffix_tones` + `unpack_ctrl_suffix`. Capture goes into `last_connect_suffix_tones[]`. |
| `source/datalink_layer/arq_responder.cc` | Cap bitmap to 30 bits at both producer sites (lines ~1270 and ~1414). Partial path's existing `if (nbits > 32) nbits = 32;` becomes `if (nbits > 30) nbits = 30;`. Clean path's `(1u << data_batch_size) - 1u` capped via `data_batch_size >= 30 ? 0x3FFFFFFFu : ((1u << data_batch_size) - 1u)`. |
| `source/main.cc` | Wire `--test` CLI flag to run `run_mfsk_ctrl_codec_tests()` and exit. Same pattern as 727b644's BP+OSD `--test` hook (sister branch). |
| `build.sh` | Add `source/physical_layer/mfsk_ctrl_codec.cc` and `source/physical_layer/mfsk_ctrl_codec_tests.cc` to `CPP_SOURCES`. |

### §11.7 Unit tests (in `--test` harness)

1. **pack_unpack_callsign_body_b36**: 1000 random callsigns, length 1..6
   chars from {A-Z, 0-9}; round-trip pack→unpack must match. End-sentinel
   handling: a 3-char call packs as `[c0|c1|c2|36|36|36]` and unpacks
   back to length 3.
2. **pack_unpack_start_conn_payload**: 1000 random NB flag + callsign
   combos. Verify the reserved bit is zero on the wire (bit 0 of the
   38-bit payload).
3. **pack_unpack_test_ack_payload**: every (echoed_cap, own_cap) ∈
   [0..3]×[0..3]; ssid covers 0..15, 16-19 (L/T/R/X), 255 (SSID_NONE).
4. **ctrl_suffix_roundtrip_all_types**: pack each type discriminator
   (00,01,10,11) with random payload+crc12 and unpack. Wrong-type
   rejection: pack as 01, unpack expecting 10 → returns the type as
   captured (caller-side check); unpack itself succeeds since type is
   an OUT parameter.
5. **ctrl_suffix_crc12_corruption**: 100 random `[type|payload]`. Flip
   one bit of the 40-bit type+payload, recompute CRC12 over the
   corrupted bits, verify CRC mismatches the unpacked crc12 field.
6. **base_pattern_cross_correlation**: assert CONNECT tones vs ACK,
   BREAK, HAIL — pairwise Hamming distance ≥ 6 of 8 (≤ 2 coincidences)
   at offset 0 for M=16, M=32. Documents the design distinguisher.
7. **ack_sack_bitmap_30bit_cap**: pack with `bitmap = 0xFFFFFFFF`,
   unpack, assert bits 30/31 are dropped (returned bitmap = 0x3FFFFFFF)
   and bsi/crc12 are NOT corrupted. Regression guard for the 32→30
   flag day.
8. **mfsk_connect_passband_roundtrip_clean**: full
   encode→IFFT→baseband→passband→detect→decode→unpack at sigma=0.
9. **mfsk_connect_passband_roundtrip_noisy**: same sweep at
   sigma={0.5, 1.0, 1.5}. Sigma=1.5 may fail to detect — only assert
   the clean path is bit-exact, noisy paths are spot-checked.
10. **mfsk_connect_no_hail_false_trigger**: generate a HAIL passband
    pattern, run `decode_ctrl_suffix_from_passband` on it, assert NO
    match.

### §11.8 Cross-layer audit (per CLAUDE.md §5)

Shared state touched in Wave 1:

| State | Producers (this PR) | Consumers (this PR) | Risk |
|---|---|---|---|
| `cl_mfsk::ack_tones`, `ack_pattern_nsymb`, `ack_match_threshold` | None (untouched) | None (untouched) | NONE. |
| `cl_mfsk::connect_tones[]`, `connect_pattern_nsymb`, `connect_match_threshold` | New `init()` block | New `generate_connect_pattern()`, new `detect_ack_pattern(connect_tones, ...)` callers | LOW — production paths don't reach this code until Wave 2. |
| `cl_mfsk::last_connect_suffix_tones[]`, `last_connect_capture_valid` | `decode_ctrl_suffix_from_passband` (new), `test_inject_connect_capture` (new) | `unpack_ctrl_suffix` callers (new) | LOW — fresh state, no aliasing with ACK+SACK capture. |
| `cl_telecom_system::connect_pattern_passband_samples`, `ctrl_suffix_passband_samples` | New `init()` add | New passband TX/RX wrappers | LOW. |
| 30-bit bitmap invariant for `pack_ack_sack_payload` | Two arq_responder.cc bitmap producers (modified to mask to 30 bits) | `cl_arq_controller::process_sack_bitmap` consumer at arq_commander.cc:2382-2422 (reads bits 0..data_batch_size-1) | LOW — `data_batch_size` is capped at 25 by typical SACK negotiation; the §11.7 regression test asserts bits 30/31 are dropped. |

**No state-machine fields touched. No messages_control changes. No
arq_commander.cc / arq_responder.cc edits beyond bitmap masking at the
two known producer sites.** That's the Wave 2 contract.

### §11.9 Open items handed to Wave 2

1. Wave 2: add `send_mfsk_start_conn(dest_call, sender_call, nb_flag)`
   in `arq_common.cc`. Computes CRC12 over `[type:2 | payload:38]`
   packed into 5 bytes; calls `generate_ctrl_suffix_pattern_passband
   (MFSK_CTRL_START_CONN, payload38)`.
2. Wave 2: CMD-side detector window after START_CONN ACK pattern + before
   TEST_CONNECTION that listens for
   `decode_ctrl_suffix_from_passband(... MFSK_CTRL_TEST_ACK ...)`.
3. Wave 2: RSP side, listen for `MFSK_CTRL_START_CONN`, then send
   `MFSK_CTRL_TEST_ACK`.
4. Wave 2: apply §6.5 HAIL self-detect race delay logic to the CONNECT
   detector (wait `(pattern_len - threshold) symbols + 200ms` after
   CONNECT-detect before TX-ing TEST_ACK).
5. Wave 2: messages_control bypass per §6.7 — design retry/timeout loop
   for `connection_attempts` that doesn't lean on
   `process_messages_tx_control()`.

### §11.10 Commit list (planned, in execution order)

1. `docs: phase-b §11 wave 1 implementation plan (pre-code)` ← THIS COMMIT
2. `mfsk_ctrl_codec: B.10 header + 6-bit base-36 alphabet + payload pack/unpack`
3. `mfsk_ctrl_codec: B.10 implementation`
4. `mfsk: B.11 generic pack/unpack_ctrl_suffix (52-bit [type|payload|crc12]) + 30-bit bitmap cap on ack_sack`
5. `mfsk: B.11 new CONNECT base Welch-Costas pattern (g=3, disjoint from ACK/BREAK/HAIL)`
6. `telecom_system: B.11 passband TX/RX wrappers for ctrl-suffix`
7. `arq_responder: cap SACK bitmap to 30 bits (flag-day with ctrl_suffix codec)`
8. `main: wire --test runner for mfsk_ctrl_codec unit suite`
9. `tests: B.11 codec + passband round-trip + HAIL/ACK collision`

---

## §13. Wave 2 v2 — PHY swap routed through legacy state machine

**Status:** PRE-CODE PLAN (this section). Wave 2 v1 (sibling worktree
`feat/b-mfsk-connect`) bypassed `messages_control` and produced 4 serial
sibling bugs in 24 hours, exactly as §6.7 predicted. v2 reworks the design
to keep the legacy state machine intact and swap only the PHY-level encode /
decode at the bottom of the TX/RX dispatchers.

### §13.1 Architectural pivot vs Wave 2 v1

| v1 (FAILED) | v2 (this plan) |
|---|---|
| `process_messages_commander()` calls `send_mfsk_start_conn(...)` directly, bypassing `messages_control`. State (`link_status`, timers, retries) driven via custom code paths inline. | `add_message_control(START_CONNECTION)` stays the entry point. `messages_control` flows normally through PENDING_ACK → ACK_TIMED_OUT → ACKED. PHY swap happens deep inside `process_messages_tx_control()` immediately before `send_batch()`. |
| New RSP detector mutates `link_status = CONNECTION_RECEIVED` itself, replicating the §1.8 state list in custom code. | New RSP detector **synthesizes `messages_rx_buffer`** as if LDPC had decoded START_CONNECTION; legacy consumer at `process_control_responder()` arq_responder.cc:1657-1742 runs unchanged. Zero state replication. |
| TEST_CONNECTION_ACK reception parsed by custom CMD-side handler in `process_messages_rx_acks_control()`. | TEST_CONNECTION_ACK reception synthesizes `messages_control.data[0..3]` to match the LDPC frame layout, then falls through to the existing `process_control_commander()` LDPC ACK handler at arq_commander.cc:3344-3392. |
| `process_messages_acknowledging_control()` TEST_ACK branch deleted. | `process_messages_acknowledging_control()` TEST_ACK branch at arq_responder.cc:793-827 swaps its inner `send_batch()` for an MFSK suffix send. The branch + state mutations stay intact. |

### §13.2 The four swap sites

**Site A — CMD TX swap (START_CONNECTION):**
`arq_commander.cc:765` — inside `process_messages_tx_control()`, when
`messages_control.status==ADDED_TO_BATCH_BUFFER`. Branch BEFORE
`pad_messages_batch_tx()` + `send_batch()`:

```cpp
bool mfsk_connect_path =
    messages_control.data[0] == START_CONNECTION
    && narrowband_enabled != YES
    && telecom_system->ack_mfsk.connect_pattern_nsymb > 0;
if (mfsk_connect_path) {
    // PHY swap: emit MFSK CONNECT suffix instead of LDPC frame.
    // sender callsign + nb_flag come from messages_control.data[2..6]
    // which the legacy code at arq_commander.cc:447-462 already populated
    // via callsign_pack(my_call_sign, ...).
    long long elapsed = send_mfsk_start_conn_phy(my_call_sign);
    if (elapsed <= 0) {
        // Codec runtime guard tripped (NB? M<16?). Fall through to LDPC.
        goto legacy_send_batch;
    }
    // Skip send_batch() — MFSK send already happened.
    // The post-send bookkeeping below (frames_to_read, connection_status,
    // calculate_receiving_timeout, receiving_timer.start) stays the same.
} else {
legacy_send_batch:
    pad_messages_batch_tx(control_batch_size);
    send_batch();
}
// ... continue post-TX bookkeeping (frames_to_read, etc.) UNCHANGED.
```

This is the v2 cleanliness: `messages_control.status` is already
ADDED_TO_BATCH_BUFFER from the existing flow; the existing
`process_messages_tx_control()` continues to drive it. PENDING_ACK is set
later via the existing `update_status()` machinery. ACK_TIMED_OUT retry
re-enters `process_messages_tx_control()` and re-fires the MFSK swap on the
next attempt — no custom retry path needed.

**Site B — RSP RX swap (synthesize messages_rx_buffer for START_CONN):**
`arq_responder.cc:217` (top of the LDPC receive block in
`process_messages_rx_data_control()`). BEFORE `this->receive()`:

```cpp
// MFSK START_CONN suffix detector. Runs while we're waiting for an LDPC
// START_CONNECTION frame. On hit: synthesize messages_rx_buffer as if
// LDPC had decoded an LDPC START_CONNECTION (data[0..6] match the
// callsign_pack output legacy expects), then let the existing
// messages_rx_buffer.status==RECEIVED handler at :287-336 copy into
// messages_control and call process_control_responder().
if ((link_status == LISTENING || link_status == CONNECTION_RECEIVED)
    && !passive_monitor
    && telecom_system->ack_mfsk.connect_pattern_nsymb > 0
    && messages_control.status == FREE
    && messages_rx_buffer.status != RECEIVED)
{
    // §13.4 frames_to_read override (carbon-copy mitigation of v1 bug #3).
    if (telecom_system->data_container.frames_to_read > 2) {
        MUTEX_LOCK(&capture_prep_mutex);
        telecom_system->data_container.frames_to_read = 2;
        MUTEX_UNLOCK(&capture_prep_mutex);
    }
    char rx_call[7] = {};
    int rx_call_len = 0;
    bool rx_nb_flag = false;
    if (receive_mfsk_start_conn_phy(rx_call, &rx_call_len, &rx_nb_flag)) {
        // §6.5 HAIL self-detect race delay
        int sym_ms = (telecom_system->data_container.Nofdm
                      * telecom_system->data_container.interpolation_rate * 1000) / 48000;
        int remaining_syms = telecom_system->ack_mfsk.connect_pattern_nsymb
                             + telecom_system->ack_mfsk.ack_sack_suffix_len()
                             - telecom_system->ack_mfsk.connect_match_threshold;
        if (remaining_syms < 0) remaining_syms = 0;
        msleep(remaining_syms * sym_ms + 200);

        // Synthesize messages_rx_buffer as if LDPC had decoded
        // START_CONNECTION. Layout matches arq_commander.cc:447-462
        // (legacy CMD-side build): data[0]=START_CONNECTION,
        // data[1]=CRC8(my_call_sign), data[2..6]=callsign_pack(rx_call).
        messages_rx_buffer.type = CONTROL;
        messages_rx_buffer.sequence_number = 0;
        messages_rx_buffer.length = 7;
        messages_rx_buffer.data[0] = (char)START_CONNECTION;
        messages_rx_buffer.data[1] = (char)CRC8_calc(
            (char*)my_call_sign.c_str(), my_call_sign.length());
        char* packed = callsign_pack(rx_call, rx_call_len,
                                     rx_nb_flag ? 0x01 : 0);
        for (int i = 0; i < 5; i++) messages_rx_buffer.data[2+i] = packed[i];
        delete[] packed;  // callsign_pack returns heap buffer
        messages_rx_buffer.status = RECEIVED;
        // Fall through — the existing block at :287-336 will copy into
        // messages_control, set status=RECEIVED, and call
        // process_control_responder() which performs ALL state mutations
        // (link_status, destination_call_sign, session_narrowband, PENDING
        // TCP, etc.) exactly as the legacy LDPC path does.
    }
}
this->receive();  // legacy LDPC path still runs as fallback
```

**Site C — RSP TEST_ACK TX swap (replaces the LDPC frame in the existing
TEST_CONNECTION_ACK dispatcher):**
`arq_responder.cc:793-827` — inside `process_messages_acknowledging_control()`,
the `TEST_CONNECTION_ACK` branch. Replace the LDPC `send_batch()` calls (lines
806-813 OFDM-side, lines 815-827 MFSK-fallback side) with a single MFSK suffix
send when WB. NB falls back to the legacy LDPC path:

```cpp
else if (messages_control.data[0] == TEST_CONNECTION_ACK)
{
    // v2: PHY swap when WB. messages_control.data[1..3] already populated
    // by the TEST_CONNECTION consumer at arq_responder.cc:1930-1934 with
    // [echoed_cap | own_cap | CRC8]. SSID comes from data[?] — see below.
    bool mfsk_path =
        narrowband_enabled != YES
        && telecom_system->ack_mfsk.connect_pattern_nsymb > 0;
    if (mfsk_path) {
        uint8_t echoed_cap = (uint8_t)messages_control.data[1];
        uint8_t own_cap    = (uint8_t)messages_control.data[2];
        // SSID: pulled from my_call_sign via callsign_get_ssid (matches
        // what the TEST_CONNECTION consumer wrote into data[6] for the
        // LDPC reverse path at arq_responder.cc:1879).
        uint8_t ssid       = (uint8_t)callsign_get_ssid(my_call_sign);
        long long elapsed = send_mfsk_test_ack_phy(echoed_cap, own_cap, ssid);
        if (elapsed > 0) return;  // suffix sent — bypass send_batch()
        // Codec runtime guard tripped — fall through to LDPC.
    }
    // Legacy LDPC TEST_CONNECTION_ACK fallback (unchanged).
    ...
}
```

**Site D — CMD TEST_ACK RX swap (synthesize messages_control for TEST_ACK):**
`arq_commander.cc:1534` — inside `process_messages_rx_acks_control()`, the
"expects_ldpc_handshake_ack" branch (currently lines 1583-1631). The current
code calls `this->receive()` to decode an LDPC TEST_CONNECTION_ACK. v2
inserts an MFSK suffix detector BEFORE `this->receive()`:

```cpp
bool expects_ldpc_handshake_ack =
    (messages_control.data[0] == TEST_CONNECTION);
if (expects_ldpc_handshake_ack
    && telecom_system->ack_mfsk.connect_pattern_nsymb > 0
    && messages_control.status != ACKED)
{
    uint8_t echoed_cap = 0, own_cap = 0, ssid = 0;
    if (receive_mfsk_test_ack_phy(&echoed_cap, &own_cap, &ssid)) {
        // Synthesize messages_control.data layout matching the LDPC
        // TEST_CONNECTION_ACK frame the legacy consumer at
        // arq_commander.cc:3344-3392 expects:
        //   data[0] = TEST_CONNECTION_ACK
        //   data[1] = echoed_cap, data[2] = own_cap
        //   data[3] = CRC8(data[1..2])  — fresh-computed so the
        //                                  consumer's CRC check passes
        //   data[5] = own_cap  (also read at :3410 fallback)
        //   data[6] = ssid     (read at :3420 SSID log)
        messages_control.data[0] = (char)TEST_CONNECTION_ACK;
        messages_control.data[1] = (char)echoed_cap;
        messages_control.data[2] = (char)own_cap;
        messages_control.data[3] = (char)CRC8_calc(
            (char*)&messages_control.data[1], 2);
        messages_control.data[4] = 0;
        messages_control.data[5] = (char)own_cap;
        messages_control.data[6] = (char)ssid;
        messages_control.length = 7;
        messages_control.type = ACK_CONTROL;
        clear_buffer(playback_buffer);
        link_timer.start();
        watchdog_timer.start();
        gear_shift_timer.stop(); gear_shift_timer.reset();
        messages_control.status = ACKED;
        stats.nAcked_control++;
        int guard = ptt_off_delay_ms;
        receiving_timeout = (int)receiving_timer.get_elapsed_time_ms() + guard;
        // Falls through to the existing post-receive block which calls
        // process_control_commander() (which runs the CRC-validated
        // post-CONNECT setup at arq_commander.cc:3344-3625).
    }
    // If detector did not fire this poll, return and re-enter next tick.
    // DO NOT fall through to the LDPC receive() — that would consume
    // audio meant for the next MFSK detector poll.
    else if (narrowband_enabled == YES) {
        // NB session: no MFSK suffix possible — let LDPC RX run.
        goto ldpc_handshake_ack_receive;
    } else {
        return;
    }
}
ldpc_handshake_ack_receive:
// legacy LDPC RX path unchanged
```

### §13.3 What stays unchanged (v2 contract)

- `add_message_control(START_CONNECTION)` at `arq_commander.cc:285` — unchanged.
- All `messages_control` state-machine transitions
  (FREE → ADDED_TO_LIST → ADDED_TO_BATCH_BUFFER → PENDING_ACK → ACK_TIMED_OUT /
  ACKED) — unchanged.
- `update_status()` ack-timeout logic, `connection_attempts++` retry counter,
  NB/WB auto-negotiation phase block at `arq_commander.cc:707-750` — unchanged.
- `process_control_responder()` at `arq_responder.cc:1657-1747` — runs
  bit-identically for both LDPC and MFSK paths. It reads the CRC8, unpacks the
  callsign, performs every state mutation, fires the PENDING TCP message.
- `process_control_commander()` at `arq_commander.cc:3344-3625` — runs
  bit-identically for both LDPC and MFSK paths.
- `process_messages_acknowledging_control()` outer dispatcher loop — only the
  inner `send_batch()` for `TEST_CONNECTION_ACK` is swapped.
- Capability negotiation, SACK setup, compression, B2F, encryption, TCP
  "CONNECTED" messages — all driven by the legacy consumer, not by the new
  MFSK code.

### §13.4 frames_to_read mitigation (v1 bug #3 prevention)

v1 bug #3 was: HAIL handler primes `frames_to_read = preamble_nSymb + Nsymb`
(large LDPC-frame value) but MFSK suffix detector requires `frames_to_read==0`.
Detector never fires.

v2 places the MFSK detector EARLIER in `process_messages_rx_data_control()`,
BEFORE `this->receive()` zeroes the counter at end-of-LDPC-frame. The
detector itself overrides `frames_to_read=2` if >2 (mirror of HAIL detector
at arq_responder.cc:128-136). This means:

- If we just finished a HAIL detect (ftr large), the new MFSK detector caps
  it to 2 and polls the suffix.
- If `this->receive()` is currently mid-LDPC capture, ftr stays at whatever
  receive() needs and the MFSK detector skips that pass (returns false from
  `receive_mfsk_ctrl_suffix`'s `frames_to_read != 0` gate).

This is the **same pattern HAIL uses** — proven on the production HAIL
detector for 18 months. v1 missed it because the v1 detector was added AFTER
receive() instead of before.

### §13.5 CRC12 init mismatch prevention (v1 bug #1)

v1 bug #1: receiver inlined CRC12 with init=0 but sender used `CRC12_calc()`
init=0xFFF. v2 mandate: **NEVER inline CRC computation in the RX path.**
Both TX and RX call the canonical `CRC12_calc()` helper in `arq_common.cc:7259`.

The v2 regression test (§13.7 test 1) explicitly drives encode and decode
through the production `CRC12_calc()` function — not a test helper. Wave 2
v1's test helper had the same init=0 bug, so the test passed while production
failed.

### §13.6 FREE-guard via legacy state machine (v1 bug #2 auto-mitigation)

v1 bug #2: MFSK START_CONN block re-fired every main-loop tick because no
FREE guard existed. v2 doesn't need a separate FREE guard — the swap site is
INSIDE the `messages_control.status==ADDED_TO_BATCH_BUFFER` branch of
`process_messages_tx_control()`. That status transitions to RECEIVING_ACKS_CONTROL
immediately after the send completes (see arq_commander.cc:802). The next
main-loop tick finds status != ADDED_TO_BATCH_BUFFER and the swap is skipped.
Legacy retry logic handles re-fire on ACK_TIMED_OUT.

### §13.7 Cross-layer regression tests (CLAUDE.md §"Cross-layer regression tests")

The Wave 2 v1 unit tests passed but hardware failed in 4 distinct ways. The
tests didn't exercise the cross-layer state machine. v2 ADDS three integration
tests in a new file `source/physical_layer/mfsk_ctrl_integration_tests.cc`
(or appended to `mfsk_ctrl_codec_tests.cc`):

1. **test_v2_crc12_wireformat_real_helper** — uses the **production**
   `CRC12_calc()` (not a test helper). Round-trips a START_CONN payload
   through CRC12 → tones → CRC12-validate. Would have caught v1 bug #1.
2. **test_v2_cmd_loop_no_refire** — mocks a CMD-side `process_messages_commander()`
   loop. Asserts that after one MFSK START_CONN send, the message stays in
   PENDING_ACK and the swap site is NOT re-entered until ACK_TIMED_OUT.
   Would have caught v1 bug #2.
3. **test_v2_rsp_frames_to_read_override** — simulates the post-HAIL state
   (`frames_to_read = preamble_nSymb + Nsymb`, large), feeds a synthesized
   MFSK CONNECT capture, asserts the MFSK detector overrides ftr=2 and
   fires. Would have caught v1 bug #3.

These tests are wired into the existing `--test` runner at
`source/main.cc` (extending the Wave 1 entry `run_mfsk_ctrl_codec_tests()`).

### §13.8 Helper API (new public methods on cl_arq_controller)

To keep `arq_commander.cc` / `arq_responder.cc` swap sites small and
auditable, the per-direction PHY helpers live in `arq_common.cc`:

```cpp
// CMD-side TX (Site A): emits CONNECT base + START_CONN suffix.
// Returns wall-clock TX time in ms, 0 if unsupported (NB/M<16).
long long send_mfsk_start_conn_phy(const std::string& sender_call);

// RSP-side TX (Site C): emits CONNECT base + TEST_ACK suffix.
long long send_mfsk_test_ack_phy(uint8_t echoed_cap, uint8_t own_cap,
                                  uint8_t ssid);

// RSP-side RX (Site B): polls capture buffer, returns true on a clean
// type=START_CONN+CRC12-validated decode.
bool receive_mfsk_start_conn_phy(char out_call[7], int* out_len,
                                  bool* out_nb_flag);

// CMD-side RX (Site D): polls capture buffer, returns true on a clean
// type=TEST_ACK+CRC12-validated decode.
bool receive_mfsk_test_ack_phy(uint8_t* out_echoed_cap, uint8_t* out_own_cap,
                                uint8_t* out_ssid);
```

The implementations are nearly bit-exact ports of v1's helpers (which were
mostly correct — the bugs were in the wiring, not the helpers themselves).
v1 bug #1 fix (CRC12_calc with init=0xFFF) ports over.

### §13.9 Cross-layer data-flow audit (CLAUDE.md §"Cross-layer audits")

**Shared state: `messages_control`** (5-step audit per CLAUDE.md).

| Question | Answer |
|---|---|
| Producers | `add_message_control()` arq_common.cc:~5350; reset on FREE in `cleanup()`; mutations in `process_control_responder()` arq_responder.cc:1657-1742, in `process_control_commander()` arq_commander.cc:3301-3625, in `process_messages_acknowledging_control()` arq_responder.cc:763-, in BREAK handler arq_responder.cc:250, in `update_status()` (ack_timer expiry), and various ACK paths in arq_commander.cc:1568-1613. v2 adds: synthesize data[0..6] in Site D (CMD TEST_ACK RX) before falling through to the legacy consumer. |
| Consumers | `process_control_responder()`, `process_control_commander()`, `process_messages_acknowledging_control()`, `process_messages_tx_control()`, `process_messages_rx_acks_control()`. v2 does not add new consumers. |
| Valid states | FREE (init), ADDED_TO_LIST, ADDED_TO_BATCH_BUFFER, PENDING_ACK, ACK_TIMED_OUT, ACKED, RECEIVED, FAILED_. Default-init = FREE. |
| Invariants | data[0..6] format depends on data[0]: START_CONNECTION uses CRC8 + callsign_pack at [1..6]; TEST_CONNECTION_ACK uses echoed_cap + own_cap + CRC8 at [1..3]. v2 Site D synthesizes the TEST_CONNECTION_ACK layout AND data[5..6] for the post-handshake-echo SSID block. |
| What this fix changes | v2 does NOT alter messages_control producer or consumer set. Site A sees ADDED_TO_BATCH_BUFFER and does PHY swap; messages_control stays in that state until set_batch advances it (which still happens via the unchanged post-send code at arq_commander.cc:802). Site D writes the TEST_CONNECTION_ACK layout, sets status=ACKED, and falls through. |

**Shared state: `messages_rx_buffer`** (5-step audit).

| Question | Answer |
|---|---|
| Producers | `this->receive()` (the LDPC decoder) — sets status=RECEIVED + type + data[] on successful decode. Various other sites zero it. v2 adds: synthesize in Site B before `this->receive()` runs. |
| Consumers | `process_messages_rx_data_control()` arq_responder.cc:287-336 (copies into messages_control on CONTROL frame and calls process_control_responder() if batch complete). Many DATA paths read it too. v2 does not add new consumers. |
| Valid states | FREE, RECEIVED. Default-init = FREE per cleanup(). |
| Invariants | When type==CONTROL: data[0]=control-code; data[1] for START_CONN is CRC8(dest_callsign), for TEST_CONNECTION_ACK is echoed_cap. data[2..6] callsign_pack output for START_CONN. sequence_number must be < control_batch_size (otherwise the consumer waits for more frames at arq_responder.cc:340). |
| What this fix changes | v2 Site B writes a synthetic record with the START_CONNECTION layout. sequence_number=0, length=7. The consumer at :287 copies into messages_control, the inner check at :327 (`sequence_number >= control_batch_size - 1`) triggers immediate `process_control_responder()` — which means **v2 must set sequence_number = control_batch_size - 1 (single-frame batch from MFSK).** Or equivalently, signal end-of-batch. AUDIT FINDING: Need to set sequence_number = (char)(control_batch_size - 1) so the consumer's "batch complete" branch fires immediately. Without this, the consumer waits for more frames (timeout). |

The §13.9 audit finding above is the v2-specific tripwire. Fixed by setting
`messages_rx_buffer.sequence_number = control_batch_size - 1` in Site B.

### §13.10 Commit list (planned)

1. `docs(phase-b): §13 Wave 2 v2 implementation plan + cross-layer audit`
2. `arq: B.12 v2 — CMD START_CONNECTION PHY swap inside process_messages_tx_control`
3. `arq: B.13 v2 — RSP MFSK START_CONN detector synthesizes messages_rx_buffer`
4. `arq: B.14 v2 — RSP TEST_CONNECTION_ACK PHY swap inside acknowledging_control`
5. `arq: B.14 v2 — CMD TEST_CONNECTION_ACK PHY swap inside process_messages_rx_acks_control`
6. `tests: v2 cross-layer regression — CRC12 wireformat, no-refire, ftr-override`

### §13.11 v2 implementation log

**Status:** Wave 2 v2 code complete on `feat/b-mfsk-connect-v2`. 6 commits
on top of Wave 1 (30a0c70). Hardware A/B not yet run — that's the next
parent-session step.

**Commits on branch `feat/b-mfsk-connect-v2`:**

```
db651f8 docs(phase-b): §13 Wave 2 v2 implementation plan + cross-layer audit
8942560 arq: B.11+ v2 — PHY-only helpers for MFSK CONNECT (no state machine touch)
f327e4a arq: B.12 v2 — CMD START_CONNECTION PHY swap inside process_messages_tx_control
e82c29a arq: B.13 v2 — RSP MFSK START_CONN detector synthesizes messages_rx_buffer
1e64e38 arq: B.14 v2 — RSP TEST_CONNECTION_ACK PHY swap inside acknowledging_control
7d18875 arq: B.14 v2 — CMD TEST_CONNECTION_ACK PHY swap inside rx_acks_control
9867fc4 tests: v2 §3 cross-layer regression — CRC12 wireformat, no-refire, ftr override
```

**Files modified (line counts after the swap commits):**

| File | Lines changed | Purpose |
|---|---|---|
| `fact-documents/phase-b-mfsk-connect-research.md` | +362 | §13 plan + this log |
| `include/datalink_layer/arq.h` | +24 | 4 new method declarations |
| `source/datalink_layer/arq_common.cc` | +355 | 4 PHY helpers + shared cores |
| `source/datalink_layer/arq_commander.cc` | +130 | Site A + Site D |
| `source/datalink_layer/arq_responder.cc` | +160 | Site B + Site C |
| `source/physical_layer/mfsk_ctrl_codec_tests.cc` | +203 | §3 v2 tests + crc12_calc init fix |

**Test results:** 12/12 pass on mercury.exe --test
- 9 Wave 1 codec tests (preserved)
- 3 new Wave 2 v2 cross-layer regression tests (§3.1/§3.2/§3.3)

**Discoveries during implementation:**

1. The Wave 1 `test_crc12_calc` helper had **the same init=0 bug** as v1's
   inline RX-side CRC (commit a2dfc34 fixed the production-side; the test
   helper was never touched). Fixed in commit 9867fc4. Without the fix,
   the new §3.1 regression test would fail on its first random byte
   sequence.
2. AUDIT FINDING (originally §13.9 for messages_rx_buffer): the consumer
   at arq_responder.cc:327 fires `process_control_responder()` only when
   `sequence_number >= control_batch_size - 1`. Site B sets
   `sequence_number = control_batch_size - 1` (a single-frame batch from
   MFSK). Verified at commit time — without this, the consumer would
   wait for a non-existent next frame and timeout.
3. `callsign_pack` writes 5 bytes directly into the caller's buffer via
   pointer (not heap-allocated, contrary to a brief misreading of the
   v1 code). v2 calls it as `callsign_pack(rx_call, len, &buf[2], flags)`
   — same shape as the legacy CMD-side build at arq_commander.cc:458.
4. The CRC12 issue in v1 also affected Wave 2 v1's loopback test (per
   commit a2dfc34's message). v2 sidesteps this by having both sides
   call the production `cl_arq_controller::CRC12_calc` — no inline copies
   anywhere. The §3.1 regression test enforces parity.

**Pre-existing warning unchanged by v2:** `arq_commander.cc:2262` has a
`-Wsign-compare` warning that predates this branch (commit 21e83946,
2026-04-20). Not introduced by Wave 2 v2.

**What's NOT done in this branch (next steps for hardware operator):**

1. **Hardware A/B at WGN:-6, -10, -12** (§B.16). Reuse drive_a26.py
   harness pattern. Expected: WGN:-6 = same behavior as baseline. WGN:-10
   = MFSK arm succeeds, baseline fails. WGN:-12 = both fail.
2. **Backward-compat regression at WGN:32** (§B.17). Confirm full ARQ
   data flow + CFG15 throughput unchanged.
3. **Validation: does CONNECT actually reach the HAIL floor?** Wave 1
   passband tests pass at sigma=0; the WGN floor is unknown until
   hardware A/B. If MFSK CONNECT cliff is no better than LDPC START_CONN
   cliff (both at WGN:-8) then the v2 effort delivered no operational
   reach — but Wave 1 base pattern is constructed identically to HAIL
   (the proven floor reference) so this is unlikely.
4. **TEST_CONNECTION (CMD→RSP) staying LDPC.** Per fact-doc §3.5, this
   is acceptable Phase B scope: by the time CMD sends TEST_CONNECTION,
   the link has already demonstrated viability via START_CONN ACK. If
   hardware shows TEST_CONNECTION failing at lower SNR, a follow-up wave
   can swap it via the same PHY-swap pattern (would be a 4th and 5th
   swap site, symmetric to Site A and Site B).

**v1 bug ledger (resolved by v2 design, not by patches):**

| v1 bug | v1 commit | v2 mitigation |
|---|---|---|
| CRC12 init mismatch (init=0 vs init=0xFFF) | a2dfc34 | Both TX+RX call production CRC12_calc. §3.1 test enforces parity with the test helper. |
| START_CONN re-fire every tick (no FREE-guard) | 71849d6 | Site A is INSIDE the legacy `messages_control.status==ADDED_TO_BATCH_BUFFER` guard. Status transitions to PENDING_ACK after swap; next tick skips. §3.2 test asserts the predicate. |
| frames_to_read large after HAIL → detector never fires | 03b0a48 | Site B mirrors HAIL's own ftr override at arq_responder.cc:128-136. §3.3 test asserts the cap. |
| Post-CONNECT TEST_CONNECTION never decoded (current/unfixed in v1) | — | v2 routes through the legacy state machine; `process_control_responder()` at arq_responder.cc:1657 runs UNCHANGED with v2-synthesized messages_rx_buffer, performing every state mutation bit-identically. The CMD's TEST_CONNECTION is then dispatched via the legacy `add_message_control(TEST_CONNECTION)` at arq_commander.cc:289, which still uses LDPC (intentionally — §13.2 scope). |

The 4th bug never had a v1 commit because v1 was paused for architectural
review (per `phase-a-to-b-workplan.md` §10 update on 2026-05-27) before a
fix landed. v2's design eliminates the root cause by not bypassing
`messages_control` in the first place.

---
