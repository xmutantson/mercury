# Data-flow audit — `frames_to_read` across the CONNECT-REACK pre-data window

**Status:** FIX SHIPPED on `feat/inband-a3-decouple`. Cross-layer audit for the
8e62722e-class OFDM-RX-acquisition regression that the redesign branch carried.
**Scope:** the shared PHY field `telecom_system->data_container.frames_to_read`
("ftr") as it is contended between the ARQ connect-heal (duplicate-TEST_CONNECTION
re-ACK) and the OFDM data consumer (`this->receive()`), in the responder's
`process_messages_rx_data_control()` loop.

VERIFIED = code-read or measured this session. The fix mirrors monitor commit
`2ce38ec` ("bound connect-reack probe window so it stops starving OFDM data RX")
which fixed the identical bug on the monitor lineage; that fix was NOT present on
this branch (the branch carried the unbounded pre-data block, the bug).

---

## §1 The bug (VERIFIED, code + fleet real-audio sim)

The redesign branch's CONNECT-REACK block (arq_responder.cc, the
duplicate-TEST_CONNECTION re-ACK before `this->receive()`) gated on the raw
predicate `connect_reack_pre_data_window()` (arq.h: CONNECTED + RECEIVING +
`batch_rx_frame_count==0` + cached ACK valid + replay budget + FREE control +
not-monitor + not-NB). Inside it, ftr was clamped to 2 EVERY tick the predicate
held (so the MFSK suffix detector core can sample). But the OFDM data consumer on
the very next line (`this->receive()`) needs `ftr = preamble_nSymb + Nsymb + 10`
to stage a fresh Schmidl-Cox preamble search + capture a full CONFIG_0 frame.

Because `connect_reack_pre_data_window()` stays true until
`batch_rx_frame_count>0`, and `batch_rx_frame_count` ONLY advances when
`this->receive()` decodes a frame, and `receive()` can only decode when ftr is
large — the clamp is **self-perpetuating**: ftr pinned at 2 → no OFDM window
staged → no decode → `batch_rx_frame_count` stays 0 → predicate stays true → ftr
re-pinned to 2. A DEAD invariant. nReceived_data == 0 forever, even though CONNECT
succeeds.

**Measured (fleet real-audio sim, .11, this session):** pinned
`--start-cfg 0 --no-gearshift --passthrough --snr 30 --payload 2048`,
MERCURY_INBAND_RATE unset, N=4. BEFORE (branch tip 27e1241, bin
md5 3902d4b6): `connected=True` but `rsp_nreceived=0` on 4/4 cells. This is the
"1.2 vs legacy 54 PINNED" delivery the redesign showed.

---

## §2 Producers of `frames_to_read` (VERIFIED, file:line on this branch)

1. LISTENING HAIL scan clamp → 2 (arq_responder.cc:141).
2. CONNECTION_RECEIVED Site-F suffix probe clamp → 2 (arq_responder.cc:268, :370).
3. RECEIVING entry / turnaround → `preamble_nSymb+Nsymb(+10)`
   (arq_responder.cc:254, :1879, :2103, :2720) — the OFDM data budget.
4. **CONNECT-REACK probe clamp → 2** (arq_responder.cc, inside the re-ACK block) —
   the producer this fix bounds.
5. **CONNECT-REACK hand-back → `frame_symb+10`** (NEW, this fix) — restores #3's
   budget once the bounded probe window elapses.
6. capture thread drains ftr toward 0 as audio is consumed (background).

## §3 Consumers (VERIFIED)

1. `this->receive()` (arq_responder.cc, immediately after the CONNECT-REACK block)
   — needs ftr large to stage a fresh OFDM window + decode a CONFIG_0 frame.
2. MFSK suffix detector core (`receive_mfsk_ctrl_suffix_phy_core`) — needs ftr
   small (drains to ~0) to sample the suffix.

These two consumers want OPPOSITE ftr magnitudes from one shared field, one line
apart. That is the cross-layer hazard.

## §4 Valid states / invariants

- Default-init: ftr per the active config; `connect_reack_window_armed=false`,
  `connect_reack_ftr_handed_back=false` (ctor + reset_session_state, arq_common.cc).
- INV-B (consumer #1): across the OFDM data-acquisition phase, ftr MUST be allowed
  to reach `frame_symb+10` at least once so `receive()` can decode the first frame
  (which advances `batch_rx_frame_count` and closes the heal). The unbounded clamp
  VIOLATED this — the fix restores it.

## §5 What the fix changes (the bounded-ownership mechanism)

`connect_reack_probe_window_open()` (NEW) arms a one-shot `connect_reack_timer` on
the first pre-data tick and returns true only while
`elapsed < connect_reack_window_ms()` (= `2*message_transmission_time_ms +
ptt_on_delay_ms`, ~1 CMD retransmit turnaround). The CONNECT-REACK block now gates
on THIS instead of the raw predicate, so the ftr=2 clamp owns ftr for only that
short turnaround sub-window (long enough to catch the lost-ACK duplicate). A
hand-back `else if` then restores `ftr=frame_symb+10` ONCE (one-shot
`connect_reack_ftr_handed_back`, so the capture-thread drain is not fought) →
OFDM consumer re-armed. The heal (re-air cached TEST_ACK on a decoded duplicate)
is preserved; only its ftr ownership is time-bounded.

**Measured AFTER (bin md5 7cd7d1a5):** same N=4 pinned CONFIG_0 cells →
`rsp_nreceived=50` on 4/4 (mean 50, min 50, max 50; in the healthy ~42-54 band).
Handback log fires at T+~20s: `[CONNECT-REACK] probe window elapsed (2632 ms) ->
handed frames_to_read=62 back to OFDM data path`.

## §6 Regression test

`test_connect_reack_ftr_starvation()` (arq_responder.cc; CLI
`--test-reack-ftr-starvation`; wired into master `--test`). Drives the REAL
arbiter + REAL turnaround timer. B1 arm, B2 in-window clamp, B3 arbiter closes
after the bounded window (PASS-AFTER; fail-before: the old unbounded predicate
never closes), B4 hand-back restores ftr>2, B5 one-shot, B6 data-arrival
self-terminate. 8/8 PASS on the fixed binary.
