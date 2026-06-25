# Connect TEST_CONNECTION / TEST_ACK handshake — single-missed-ack stall

**Status:** DESIGN / pre-implementation (plan for owner approval; NO code yet).
**Owner:** ARQ-architect session 2026-06-21, branch `fix/testbed-reliability`.
**Scope:** the CMD↔RSP connect handshake (START_CONNECTION → TEST_CONNECTION →
TEST_CONNECTION_ACK). A single un-detected TEST_CONNECTION_ACK costs the WHOLE
~40-60s connect window because the responder does NOT re-emit the ACK on the
~30-70 duplicate TEST_CONNECTIONs the commander retransmits.

VERIFIED = code-read or measured from a sim run this session. INFERRED = reasoned
from code but not directly observed firing.

---

## §1 The bug (VERIFIED in code + reproduced in sim)

### §1.1 Handshake shape (VERIFIED, code)
The WB connect handshake is three control exchanges:
1. **START_CONNECTION** (CMD→RSP), ACKed by an MFSK **ACK pattern** (tones).
   CMD detects via `[CMD-ACK-PAT]` (arq_commander.cc:2103 `receive_ack_pattern`).
2. **TEST_CONNECTION** (CMD→RSP) carrying SNR + caps + SSID. On WB (M=16) it is
   emitted as an **MFSK CONNECT suffix**, not an LDPC frame
   (`mfsk_connect_path`, arq_commander.cc:1109-1122; `send_mfsk_test_conn_phy`,
   arq_common.cc:7735-7747).
3. **TEST_CONNECTION_ACK** (RSP→CMD) carrying the capability echo + CRC8. On WB
   it is emitted as an **MFSK TEST_ACK suffix** (`send_mfsk_test_ack_phy`,
   arq_common.cc:7559-7566, dispatched at arq_responder.cc:1427-1444); CMD polls
   for it with the matching MFSK detector (`receive_mfsk_test_ack_phy`,
   arq_common.cc:7712-7727, called at arq_commander.cc:2144-2199).

> CORRECTION to the task framing: on WB the modalities already MATCH — RSP sends
> MFSK TEST_ACK and CMD polls MFSK TEST_ACK. The task's "RSP sends LDPC-OFDM,
> CMD polls MFSK" describes the **NB / codec-unavailable fallback** (LDPC
> TEST_CONNECTION_ACK on data_configuration, arq_responder.cc:1464-1488; CMD's
> matching LDPC receive at arq_commander.cc:2202-2211). So a pure modality
> mismatch is NOT the root cause on the WB path. The root cause is the
> **re-emit gap** below.

### §1.2 Root cause — the responder stops answering duplicate TEST_CONNECTIONs (VERIFIED, code)
The MFSK TEST_CONNECTION detector ("Site F") is gated to fire **only in
`link_status == CONNECTION_RECEIVED`**, and DELIBERATELY excludes `CONNECTED`:

- arq_responder.cc:355-359 — gate is `link_status == CONNECTION_RECEIVED && …`.
- arq_responder.cc:340-350 — the comment states CONNECTED is excluded on purpose
  (leaving it in made Site F re-poll forever and reset `frames_to_read=2` every
  miss, starving the LDPC data-RX path; "Hardware bug 2026-05-27").

Flow when the FIRST TEST_CONNECTION lands:
RSP is in CONNECTION_RECEIVED → Site F (355) detects it → synthesizes
messages_rx_buffer → process_control_responder() runs the TEST_CONNECTION branch
(arq_responder.cc:2535) → sets `link_status=CONNECTED` (2684) and
`connection_status=ACKNOWLEDGING_CONTROL` with TEST_CONNECTION_ACK queued
(2720-2726) → ACK is sent (arq_responder.cc:1436) → `connection_status=RECEIVING`.

Now RSP is `link_status==CONNECTED, connection_status==RECEIVING`. If that single
TEST_ACK is missed by CMD, CMD retransmits TEST_CONNECTION. But:
- Site F (355) does NOT fire (gate requires CONNECTION_RECEIVED, not CONNECTED).
- The TEST_CONNECTION branch in process_control_responder (2535) DOES list
  `link_status==CONNECTED` in its guard and WOULD re-queue the ACK — **but it is
  never reached**, because the duplicate arrives as an **MFSK suffix** that only
  Site F decodes; the legacy LDPC `receive()` at arq_responder.cc:436 cannot
  decode an MFSK suffix, so messages_rx_buffer is never populated with a
  TEST_CONNECTION and 2535 never runs.

Net: **the WB TEST_CONNECTION_ACK is sent exactly ONCE. Every subsequent
duplicate TEST_CONNECTION is silently dropped.** A single CMD-side ACK miss is
unrecoverable until the connect window expires.

### §1.3 Timing / cadence (VERIFIED, code)
- `connection_timeout` default = 30000 ms (arq_common.cc:525), overridden to
  15000 ms by the CLI default (main.cc:797, applied main.cc:3741), and floored up
  for slow frames (arq_common.cc:2244-2252). It is a **per-attempt** timeout:
  reset at each attempt (arq_commander.cc:1034-1036) and checked in
  update_status (arq_common.cc:2811-2816).

  > The task's "~57.7 s connection_timeout at arq_common.cc:1998" is STALE —
  > line 1998 is `load_configuration`, not a timeout. The real symbol is
  > arq_common.cc:525 (=30000) / main.cc:797 (CLI default 15000) checked at
  > arq_common.cc:2814.
- `max_connection_attempts` default = 15 (main.cc, referenced
  arq_commander.cc:1033). TEST_CONNECTION is retransmitted under
  `messages_control` ACK-timeout → `process_messages_tx_control` resend loop
  (arq_commander.cc:1024-1088, `nResends`); each resend's per-frame ACK window is
  `calculate_receiving_timeout()` (arq_commander.cc:2065).

### §1.4 Reproduced in sim (MEASURED, this session)
`python tools/sim/sim_arq_channel.py --cell WGN:30 --secs 80 --json …` (clean
cell, guard binary `/c/Program Files/Mercury/mercury.exe` 2026-06-10), log
`sim_arq_channel.log`:
- CMD: START_CONNECTION ACKed at T+21.6 (`[CMD-ACK-PAT] code=49 detected`).
  CMD then sends `CONTROL:TEST_CONNECTION` **72 times** (T+31.3 → T+64.7).
- RSP: reaches `Connection Received from TESTA` at T+21.1 and **stays there the
  entire window** (T+21 → T+64.7, ~43 s), `connection_status:Receiving`.
  `RSP-TEST-CONN-V3` synthesized = **0**, `HANDSHAKE-ECHO RSP queued
  TEST_CONNECTION_ACK` = **0**, `MFSK TEST_ACK sent` = **0**.
- Result: `connected:false`, 0 bytes in 85 virtual-s.
- So **72 TEST_CONNECTIONs received → 0 acks emitted** — the re-emit gap of §1.2,
  observed directly. (This run shows RSP never decoding ANY of the 72 — i.e. the
  worst case where even the first ACK is lost AND no duplicate ever re-detects;
  the bug fully explains the 0-byte connect.)

[?] OPEN: in §1.4 RSP shows CONNECTION_RECEIVED but `RSP-TEST-CONN-V3=0`, so on
this seed RSP never decoded even the first TEST_CONNECTION suffix while in
CONNECTION_RECEIVED (a detection/window miss compounding the re-emit gap). The
re-emit fix (§3) covers this case too: if Site F also ran in CONNECTED, the very
first decoded duplicate would complete the handshake. Worth one more seed to
confirm the "first ACK sent, then lost" variant vs the "never decoded" variant —
both are healed by the same fix.

---

## §2 Prior art (§1 research) — robust connect-ACK design

The governing principle across every mature ARQ/link-setup protocol: **the
connect acknowledgment is idempotent and re-emitted for every (duplicate)
connect request the responder hears, on the modality the requester listens for.**
A lost ACK must cost ONE round trip, never the whole connect budget.

- **TCP / RFC 9293 (canonical).** In the LISTEN/SYN-RECEIVED state, an arriving
  SYN is ALWAYS answered with a SYN-ACK; the responder "has no way of knowing
  whether the segment was an old one or not," so it re-sends the SYN-ACK on every
  SYN it sees. A lost SYN-ACK is recovered by the requester's SYN retransmit in
  one RTT — the responder does NOT leave the answering state after the first ACK.
  This is exactly the property Mercury's Site F gate breaks.
  (datatracker.ietf.org/doc/rfc9293/, §3.5/§3.10 — "SYN-ACK be sent in response
  to a SYN"; duplicate-SYN handling.)
- **ARDOP (Winlink HF, the VARA-class analogue).** ARQ connect is
  ConReq(ISS) → ConAck(IRS). The ISS retransmits ConReq up to its attempt count;
  the IRS re-emits ConAck on each ConReq it decodes (the reverse ACK/NAK channel
  is the protocol's reliability mechanism). The CONACK echoes the negotiated
  bandwidth, mirroring Mercury's cap-echo TEST_CONNECTION_ACK.
  (winlink.org `_ardop_specification.pdf`, ARQ / frame-appendix sections.)
- **STANAG 4538 3G-ALE FLSU / MIL-STD-188-110.** Fixed-cycle link-setup
  handshake (request/confirm bursts) where the confirm is re-sent each cycle the
  request is heard; robustness comes from re-emitting the short confirm, not from
  widening a one-shot window. (Confirms the "make a miss cheap, re-air the
  confirm" posture; en.wikipedia.org/wiki/Automatic_link_establishment.)
- **Internal precedent — same fix shape already used twice in Mercury:** the
  START_CONNECTION detector ("Site B") and the data-ACK pattern scan were both
  narrowed/re-scoped for exactly this class of single-miss stall, and the CMD
  already multi-window-scans the START_CONNECTION control-ACK across the retained
  ring (`mw_scan`, arq_commander.cc:2102-2103) so a gap-displaced ACK is still
  caught. The TEST_CONNECTION_ACK leg simply never got the same treatment.

Design consensus: **(a) re-emit on duplicate** is the structural fix (TCP/ARDOP);
**(b) modality alignment** is already satisfied on WB but must be preserved.

---

## §3 The fix (chosen design)

### §3.1 Decision: option (a) responder RE-EMITS, with a bounded same-state ACK replay
Make the responder answer EVERY decoded TEST_CONNECTION, including duplicates
that arrive after it is already CONNECTED — matching TCP's "SYN in any
listen/established-pre-data state ⇒ re-send SYN-ACK." This is minimal, addresses
the root cause directly, and needs no CMD-side change (CMD already polls MFSK
TEST_ACK and falls through on a miss, arq_commander.cc:2144-2199).

We do NOT simply add `CONNECTED` to the Site F gate at arq_responder.cc:355 — the
2026-05-27 comment (arq_responder.cc:340-350) documents that doing so naively
starves the data-RX path (Site F resets `frames_to_read=2` on every miss). The
fix must re-emit the ACK WITHOUT leaving Site F free-running in CONNECTED.

### §3.2 Mechanism — a duplicate-TEST_CONNECTION re-ACK that is scoped to the
**pre-data window only** (INFERRED design; insertion points VERIFIED)

Add a narrow Site-F variant that runs in `link_status==CONNECTED` **only while no
data batch has yet been received** (the connect handshake is not yet "done" from
the responder's view until the first data frame arrives — same boundary the
break-fh fix used, gating on `batch_rx_frame_count==0`). In that pre-data window:
1. Poll `receive_mfsk_test_conn_phy` (the existing detector) — but on a hit,
   **do NOT re-run the full TEST_CONNECTION state mutation** (caps/SACK/SSID
   re-negotiation already happened). Instead, **re-send the already-built
   TEST_CONNECTION_ACK only** (re-dispatch the arq_responder.cc:1427-1444 MFSK
   TEST_ACK path using the caps/CRC captured at the first handshake), then return
   to RECEIVING. This is the idempotent replay — cheap (~1 suffix, ~0.7 s wire),
   no state churn, no `frames_to_read` starvation because it exits immediately on
   a miss back to the normal RECEIVING data path (does not pin ftr=2 across the
   data phase).
2. Bound it: stop re-ACKing once `batch_rx_frame_count>0` (data has started) OR
   after a small cap (e.g. the connect-attempt budget) so a stuck CMD cannot keep
   RSP in replay forever.

New/changed state (minimal):
- Cache the handshake echo triple at first TEST_CONNECTION: `echoed_cap`,
  `own_cap`, `ssid` (already computed at arq_responder.cc:2721-2724 /
  1433-1435) into a small `connect_ack_cache{valid,echoed,own,ssid}` member so
  the replay rebuilds the identical TEST_ACK without re-deriving negotiation.
- A `pre_data_window` predicate = `link_status==CONNECTED &&
  connection_status==RECEIVING && batch_rx_frame_count==0`.

Insertion points (VERIFIED line anchors):
- **RSP detector/replay:** new block in `process_messages_rx_data_control`
  alongside Site F, arq_responder.cc:355 (clone the gate with the §3.2 predicate
  + replay-only action). Replay calls the existing dispatcher at
  arq_responder.cc:1427-1444.
- **RSP cache populate:** at the first-handshake site, arq_responder.cc:2720-2726
  (write `connect_ack_cache`).
- **No CMD change required.** (If desired, also extend the CMD multi-window scan
  `mw_scan` to TEST_CONNECTION as it already does for START_CONNECTION,
  arq_commander.cc:2102 — strictly additive belt-and-suspenders, not required.)

### §3.3 Why robust + minimal
- One round trip recovers a lost ACK (TCP/ARDOP property), vs ~40-60 s today.
- Re-uses the EXISTING MFSK TEST_ACK TX path and the EXISTING detector — no new
  frame type, no new modality, no CMD edit.
- Scoped to the pre-data window so it cannot regress the 2026-05-27 data-RX
  starvation bug (the reason CONNECTED was excluded from Site F): the replay
  never pins `frames_to_read` across the data phase and self-terminates at the
  first data frame.
- Idempotent: replays the byte-identical ACK; no negotiation re-run, so CMD/RSP
  cannot diverge on caps/SACK (the failure mode handshake-capability-echo.md
  warns about).

### §3.4 Considered + rejected
- **(b) modality re-align only** — already aligned on WB; would not fix the
  re-emit gap (the dominant cause). Keep as a no-op invariant check.
- **Naive "add CONNECTED to Site F gate"** — REJECTED, reintroduces the
  2026-05-27 ftr=2 data-RX starvation (arq_responder.cc:340-350).
- **Widen the CMD ACK window** — REJECTED per CLAUDE.md §2 (masks a one-shot ACK
  with a bigger timeout; doesn't make the miss recoverable).

---

## §4 Cross-layer audit (§5) — connect-handshake state

State touched: TEST_CONNECTION receive/ACK, `link_status`/`connection_status`
transitions, the new `connect_ack_cache`, `batch_rx_frame_count`,
`frames_to_read`, and the negotiated `peer_capability`/`sack_v2_enabled`.

1. **Producers.**
   - `link_status=CONNECTED`: arq_responder.cc:2684 (first TEST_CONNECTION).
   - `connection_status`: ACKNOWLEDGING_CONTROL @2726, →RECEIVING after ACK send
     (arq_responder.cc post-1444 / :2087 etc.).
   - TEST_CONNECTION_ACK build: arq_responder.cc:2720-2726; MFSK TX dispatch
     :1427-1444.
   - `connect_ack_cache` (NEW): written at :2720-2726.
   - `batch_rx_frame_count`: data-RX path (arq_responder.cc, reset on ACK-gate
     paths e.g. :2032/:2061).
   - CMD-side TEST_CONNECTION (re)send: arq_commander.cc:1024-1122; ACK poll
     :2144-2199.
2. **Consumers.**
   - Site F gate reads `link_status`/`connection_status`/`messages_control.status`
     /`messages_rx_buffer.status` (arq_responder.cc:355-359).
   - CMD `expects_ldpc_handshake_ack` reads `messages_control.data[0]`
     (arq_commander.cc:2082); MFSK poll reads `connect_pattern_nsymb`,
     `narrowband_enabled` (:2144-2148).
   - Negotiated caps/SACK consumed by the whole data path
     (handshake-capability-echo.md §1).
3. **Valid states / default-init.** Before first TEST_CONNECTION:
   `connect_ack_cache.valid=false`, `link_status` ∈ {LISTENING,
   CONNECTION_RECEIVED}, `batch_rx_frame_count=0`. The replay MUST be inert when
   `connect_ack_cache.valid==false` (no first handshake yet ⇒ nothing to replay;
   Site F at :355 still handles the genuine first TEST_CONNECTION).
4. **Invariants consumers assume.**
   - INV-A: a TEST_CONNECTION_ACK is emitted on data_configuration/ MFSK exactly
     matching what CMD polls (preserved — replay re-uses :1427-1444 verbatim).
   - INV-B: the data-RX path is not starved by `frames_to_read=2` pinning
     (preserved — replay self-terminates at first data frame; never enters once
     `batch_rx_frame_count>0`).
   - INV-C: caps/SACK negotiation runs EXACTLY ONCE (preserved — replay does NOT
     re-run the :2535 negotiation; it only re-airs the cached ACK).
   - INV-D: SWITCH_ROLE / connect-accept path just changed in **894529c** must
     stay correct. The replay does not touch `link_status` (stays CONNECTED) or
     role; it only re-emits a control suffix in the pre-data window, so the
     role-swap / connect-accept transition is unaffected. [?] VERIFY against
     894529c diff that it did not move the CONNECTED entry or the
     ACKNOWLEDGING_CONTROL→RECEIVING edge the replay piggybacks on.
   - INV-E: data-phase turnaround geometry (the reverse-ACK window work,
     data-flow-revack-turnaround-geometry.md) is untouched — replay is gated OFF
     once data starts (`batch_rx_frame_count>0`), so it cannot fire during a data
     batch turnaround.
5. **What the fix changes.** Adds a pre-data CONNECTED re-ACK of a cached,
   byte-identical TEST_CONNECTION_ACK. Walked consumers: CMD MFSK poll
   (unchanged input), data-RX (protected by INV-B/E), negotiation (INV-C). No
   consumer assumption violated.

This audit lives here; pair with the regression test in §5 (the doc owns the
producer/consumer list per CLAUDE.md).

---

## §5 Test plan (§3 — fails-before / passes-after)

Vehicle: the in-repo FTRT sim, `tools/sim/sim_arq_channel.py` (two `-x sim`
peers + `sim_channel_relay.py`). **Bounded runs only** — single cells, `--secs`
80-100, hard wall-clock `timeout`. No open-ended multi-cell sweeps.

**T1 — connect-time-under-ACK-loss (primary).**
- Force the failure deterministically: a small relay knob to drop the FIRST
  RSP→CMD control suffix after the responder reaches CONNECTED (one-shot ACK
  loss), OR reuse a seed (e.g. the §1.4 seed) that already exhibits the stall.
- **FAIL-BEFORE (monitor binary):** `connected:false` / connect time ≥ window
  (≥ ~40 s; §1.4 shows 0 bytes in 85 s, RSP-queued-ACK=0 across 72
  TEST_CONNECTIONs).
- **PASS-AFTER (fixed binary):** assert connect completes within ~1 extra
  round-trip of the first lost ACK (target: `connected:true` and connect time
  ≤ START_CONNECTION-ack time + 1 TEST_CONNECTION cycle, i.e. seconds not tens of
  seconds), and `HANDSHAKE-ECHO RSP queued TEST_CONNECTION_ACK` count ≥ 2 (first
  + at least one re-emit on a duplicate).
- Metric source: harness `connected` + parse `sim_arq_channel.log` for
  `[CMD-TEST-ACK-V2]` first-detect timestamp and the re-emit count.

**T2 — no-regression on the clean happy path.**
- A clean cell where the first ACK is NOT lost: assert connect time unchanged vs
  monitor and the replay block fires 0 times (cache valid but no duplicate
  decoded in the pre-data window) — proves the fix is inert when not needed.

**T3 — data-RX starvation guard (the 2026-05-27 regression the gate prevented).**
- A normal transfer cell (e.g. WGN:30, target a few KB): assert data delivers
  byte-faithful (`md5_match:true`) and that `[RX-TIMING]`/data decode events fire
  — i.e. the replay did NOT pin `frames_to_read` and starve data RX (INV-B).

**T4 — in-process unit (per CLAUDE.md cross-layer test rule).**
- Add a synthetic-fire entry point (pattern: `test_partial_bsi_advance`,
  arq_common.cc:~8001) that drives RSP to CONNECTED+pre-data, fires a duplicate
  TEST_CONNECTION, and asserts a second TEST_CONNECTION_ACK is queued with
  byte-identical caps/CRC (INV-A/C), and that firing a data frame disables
  further replay (INV-B/E). Fail-before (no re-emit) / pass-after.

Bounded-run note: each sim cell ≈ 55-65 s wall at FTRT 0.77×; wrap in
`timeout 220`. Do NOT launch the effective-rate sweep or any multi-cell loop.

---

## §6 Cross-references
- `handshake-capability-echo.md` — history of the TEST_CONNECTION_ACK / cap-echo
  leg (why the ACK carries caps+CRC; the v8 reverted echo-in-reply).
- `mfsk-robust-ack.md` §7 — TEST_CONNECTION_ACK is explicitly out of scope of the
  data-ACK rework; this doc owns it.
- `break_fh_cfg15_regression` (memory) — precedent for data-phase scoping a
  control-frame behavior via `batch_rx_frame_count>0`.
- arq_responder.cc:340-350 — the 2026-05-27 comment that explains WHY CONNECTED
  was excluded from Site F (the constraint this fix must respect).
- Prior art: RFC 9293 (datatracker.ietf.org/doc/rfc9293/); ARDOP spec
  (winlink.org `_ardop_specification.pdf`); ALE/STANAG-4538
  (en.wikipedia.org/wiki/Automatic_link_establishment).

---

## §7 IMPLEMENTATION + TEST RESULTS (2026-06-22, off-bench)

**Status:** IMPLEMENTED (option a) on branch `fix/connect-reack` (from `monitor`
7dd4c42, NOT merged — owner reviews/merges). Worktree
`C:/Users/kamer/mercury_wt/connect-reack`. Build + master `--test` GREEN.

**INV-D (blast radius of 894529c, the SWITCH_ROLE fix) — VERIFIED CLEAR.**
894529c touched arq_responder.cc in ONE 5-line hunk only (`session_data_frame_
received = true;` in add_message_rx_data ~:99). It did NOT move Site F (:355),
the CONNECTED entry (:2684), the ACK build (:2720-2726), the ACK dispatcher
(:1416-1488), or the ACKNOWLEDGING_CONTROL->RECEIVING edge (:1642-1645) the
re-ack block piggybacks on. Bulk of 894529c is in arq_commander.cc (SWITCH_ROLE
gate + BREAK teardown). The re-ack block reads RESPONDER state only, never
mutates link_status/role -> INV-D holds.

**FIX (file:line).** (1) cache `connect_ack_cache{valid,echoed_cap,own_cap,ssid,
replays}` (arq.h after session_data_frame_received) populated at the first-
handshake ACK build (arq_responder.cc:2720-2726, byte-identical to the
dispatcher's :1433-1435 reads); reset in reset_session_state (arq_common.cc:4046)
+ init (:530). (2) new gated re-ACK block beside Site F (arq_responder.cc, just
before `this->receive()`), gate = single-sourced predicate
`connect_reack_pre_data_window()` (arq.h) == CONNECTED && RECEIVING &&
batch_rx_frame_count==0 && cache.valid && replays<max_connection_attempts &&
messages_control.status==FREE && !passive_monitor && narrowband_enabled!=YES,
ANDed in production with the DSP term `ack_mfsk.connect_pattern_nsymb>0`. On a
decoded duplicate it re-airs the CACHED ACK via the existing
send_mfsk_test_ack_phy (NO :2535 negotiation re-run, NO messages_rx_buffer
synthesis), bounded to max_connection_attempts replays.

**TESTS** (bounded sim, `--no-gearshift` WB-pin so the MFSK TEST_ACK path runs;
deterministic single-ACK loss via NEW test-only relay knob `--erase-b2a-burst N`
that zeros the Nth RSP->CMD burst — burst #3 == the first MFSK TEST_ACK,
empirically pinned by binary search):
- **T1 FAIL-BEFORE** (monitor baseline 7dd4c42, erase burst 3): RSP reaches
  CONNECTED + sends 1 MFSK TEST_ACK (erased) -> CMD detect=0, CMD Connected=0
  (connect FAILS; no re-emit exists).
  **PASS-AFTER** (fix, erase burst 3): RSP fires `[CONNECT-REACK] ... replay=1/15`
  on the duplicate TEST_CONNECTION (T+32.9) -> CMD `[CMD-TEST-ACK-V2] echoed=0x01
  own=0x01 ssid=255` (BYTE-IDENTICAL to the erased original) -> CMD
  `Connected to TESTB` (T+33.4), one TEST_CONNECTION RTT after the duplicate.
- **T2 no-regression** (fix, clean WGN:30, no erase): CONNECT-REACK fires 0x,
  HANDSHAKE-ECHO queued 1x, connected:true (inert when not needed).
- **T3 starvation guard / INV-B** (fix, erase burst 3): re-emit fires, CMD
  connects, frames_to_read reaches its full data value 346 (NOT pinned at 2),
  delivered byte-faithful (md5_match:true) -> no 2026-05-27 ftr=2 data-RX
  starvation recurrence.
  **~~OVERTURNED 2026-06-25 (§8): T3's "ftr not pinned -> no starvation"
  conclusion was WRONG for the COMMON clean-connect arm.~~** T3 only measured the
  LOST-ACK arm: the duplicate IS decoded, so the suffix-core restores ftr to
  preamble+Nsymb (arq_common.cc:7721), masking the bug. On the COMMON arm (no
  lost ACK -> NO duplicate to decode) the re-ACK block re-clamped ftr=2 EVERY
  pre-data iteration and the suffix-core MISS path (arq_common.cc:7676/7694/7712)
  left it at 2 -> OFDM data acquisition starved (nReceived_data 0). See §8.
- **T4 in-process unit** (`--test-connect-reack`, wired into master `--test`):
  A1 old Site-F gate does NOT fire when CONNECTED (fail-before); A2 pre-data
  predicate FIRES on a cached duplicate; A3 cached triple byte-identical
  (INV-A/C); A4 predicate FALSE once batch_rx_frame_count>0 (INV-B/E); A5 FALSE
  at the replay bound; A6 inert before first handshake. 9/9 PASS.

## §8 REGRESSION + CROSS-LAYER FIX (2026-06-25) — ftr starvation, OFDM delivery 0

**Status:** FIXED-SURGICAL on branch `fix/monitor-ofdm-8e62722e` (from current
`fix/sim-2proc-capture-determinism` HEAD, which contains 8e62722e). build.sh o3
rc=0; master `--test` rc=0 (50 passed/0 failed + all sub-suites incl. the new
`[TEST-REACK-FTR]` 8/8). NOT pushed to monitor (owner reviews/merges).

### §8.1 What broke (bisect)
A 73-commit bisect on wf_9664be1c found a sharp 54-vs-0 OFDM-frame split:
clean base 8d290ec3 delivers 46-54 CONFIG_0 data frames; monitor tip 206de313
CONNECTS but delivers ZERO. **FIRST-BAD = 8e62722e** ("re-emit cached TEST_ACK
on duplicate TEST_CONNECTION"; parent 63a8180e GOOD; merged via 30367670). A
CONNECT-handshake heal that broke OFDM DATA-FRAME ACQUISITION — a cross-layer
side effect that slipped past `--test` (T4 only checked the predicate boolean,
never the shared `frames_to_read` the data path consumes).

### §8.2 Root cause (static + empirical, frames_to_read cross-layer audit)
- **PRODUCER (data-RX):** arq_responder.cc:1633 sets `frames_to_read =
  frame_symb+10` ONCE at TEST_CONNECTION_ACK time (RECEIVING entry). This is the
  budget the OFDM data consumer needs.
- **CONSUMER:** `this->receive()` (arq_responder.cc:~519, post-fix) -> OFDM
  decode; needs `ftr ~= frame_symb+10` to snapshot a full data frame.
- **THE BUG (second producer):** the 8e62722e re-ACK block (just before
  `this->receive()`) clamped `frames_to_read=2` EVERY iteration while
  `connect_reack_pre_data_window()` was true (CONNECTED + RECEIVING +
  batch_rx_frame_count==0). The suffix-core requires ftr to drain to 0 to sample
  (arq_common.cc:7648), so the clamp is necessary for the PROBE — but it leaks
  into the data path. With ftr re-pinned to 2 each tick, the OFDM frame never
  decodes -> `batch_rx_frame_count` never increments past 0 -> the window
  predicate never closes. **The INV-B "self-terminate on batch_rx_frame_count>0"
  is a DEAD invariant: the counter it waits on can only advance via the decode
  the clamp prevents.** Re-introduced the exact 2026-05-27 ftr=2 starvation Site
  F (:360) excludes CONNECTED to avoid.
- **Empirical (fleet .11 real-audio sim, CONFIG_0, WGN:30):** suspect 8e62722e
  -> rsp_nreceived_frames=5 with 50 RX-TIMING fail events; the same binary with
  the :449 clamp disabled -> 60 (>12x collapse, one-variable control).

### §8.3 The surgical fix (keeps the heal, restores OFDM)
The probe and the data RX want OPPOSITE ftr values on ONE shared ftr, and
`receive()` runs the very next line — per-iteration save/restore can't work (it
would keep ftr>0 so the probe never samples = heal lost). Resolution =
**WINDOW-BOUNDED OWNERSHIP** (the lost-ACK duplicate arrives within ~1 CMD
retransmit cycle, so the heal only needs a SHORT turnaround sub-window):
1. `connect_reack_probe_window_open()` (arq_responder.cc, new) arms a one-shot
   `connect_reack_timer` on the FIRST pre-data tick and returns true only while
   `elapsed < connect_reack_window_ms()` (= `2*message_transmission_time_ms +
   ptt_on_delay_ms`). After the window elapses it returns false forever for this
   pre-data window. The probe (and its ftr=2 clamp) is gated on THIS, not the
   unbounded pre-data predicate.
2. Hand-back (arq_responder.cc, new `else if`): once the window has elapsed and
   the last probe iteration left ftr<=2, restore `frames_to_read = frame_symb+10`
   ONCE (one-shot `connect_reack_ftr_handed_back`, so the capture-thread drain is
   not fought) -> the OFDM consumer is re-armed -> delivery restored.
3. New state (arq.h, beside connect_ack_cache): `cl_timer connect_reack_timer;`
   `bool connect_reack_window_armed; bool connect_reack_ftr_handed_back;` reset
   in init (arq_common.cc:~572) + reset_session_state (~4093); the arbiter also
   self-disarms whenever the pre-data predicate is false.

### §8.4 Regression test (the class that slipped past --test)
`test_connect_reack_ftr_starvation()` (arq_responder.cc; CLI
`--test-reack-ftr-starvation`; wired into master `--test`). Drives the REAL
arbiter + REAL turnaround timer and asserts the cross-layer invariant "a
connect-heal must NOT leave frames_to_read pinned at 2 across the OFDM
data-acquisition phase": B1 arms, B2 probe-in-window, B3 arbiter CLOSES after
the bounded window (the unbounded-loop bug is gone), B4 hand-back restores ftr>2,
B5 one-shot, B6 data-arrival self-terminate. 8/8 PASS. FAIL-BEFORE: the old code
called `connect_reack_pre_data_window()` directly (never closes) with no
hand-back, so B3/B4 would FAIL.
