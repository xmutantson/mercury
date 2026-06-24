# Investigation: "inband-specific CONNECT deafness" — PREMISE NOT REPRODUCIBLE

Status: CLOSED 2026-06-23 (keystone HEAD 960df86, feat/inband-a3-decouple).
Verdict: the claimed **redesign-specific** connect-handshake deafness is **NOT an inband
logic bug**. It is a **marginal realtime-sim acquisition that is wall-clock-LOAD sensitive**,
confounded by **A/B run-order** (the second arm in a session is systematically penalized).
A secondary, real but smaller effect: under inband the post-connect climb is slower.
No code change shipped (per CLAUDE.md §2 "never tune a timeout to mask a failure" + the
MEMORY bench-claim guard "never state a timing/load cause as a structural fact without
re-deriving"). This doc records the falsification so the next session does not re-chase it.

## §1 The claim under test (from the paired diag `_msab/diag/`)
- `legacy_3003`: connected, climbed to CONFIG_16, rx=8109B. RSP acquired TEST_CONNECTION at
  internal T+31.576 (`legacy_3003_modem.log:4720`). FTRT **0.71x** (real 286 / virtual 405).
- `redesign_3003` (MERCURY_INBAND_RATE=1): rx=0B, deep stall. RSP acquired TEST_CONNECTION
  only at internal T+63.741 (`redesign_3003_modem.log:10937`). FTRT **0.73x**.
- Claim: inband INDUCES a ~32s RSP deafness to TEST_CONNECTION → no data phase → null.

## §2 What is byte-identical between the two arms (VERIFIED from the two modem logs)
1. **Same binary** — both `Mercury Version 0.4.2 (build 2d40a23-dirty)`; only env differs.
2. **CMD wire identical** — `[CMD-TX] CONFIG_100 batch=1 type=48 pream=8 Nsymb=533`,
   START_CONNECTION sends T+11.17/T+21.56, TEST_CONNECTION `last_message_sent` T+31.0,
   `[TX-PEAK] size=1895664` — all within ~30ms BOTH arms. No CONFIG_TAG suffix on connect
   frames (the inband gate at arq_commander.cc:834 is SET_CONFIG-only; the START/TEST builders
   at :744/:760 have NO inband gate). So the "robust-suffix geometry-confusion" hypothesis is
   FALSE — the connect frames are the same length on the wire.
3. **RSP staged audio identical** — `[CAP-PEAK] pk=0.276 sp=1168` same cadence (T+10.7, 13.7,
   17.2, 20.7, 23.8, 27.3, 30.8) in BOTH arms.
4. **RSP PHY geometry identical** during connect — `Nofdm=292 buffer_Nsymb=1301 Nc=10` and
   `ftr=551` after the START_CONNECTION ACK in BOTH arms.
5. **ZERO `[INBAND` markers fire on the RSP** during the deafness window (all inband RSP
   helpers gate on link_status==CONNECTED; the connect phase is CONNECTION_RECEIVED).
   So no inband code path runs on the deaf RSP — the receive/decode code executed is
   byte-identical to legacy.

The only visible divergence: legacy's RSP decodes the connect frame (`[RX-CTRL] code=50`) right
when the CMD finishes airing it (cmd_ptt_off T+31.0 → decode T+31.572); the redesign RSP's
snapshot phase (`P2B-DIAG` at T+30.974 then a 1.3s gap to T+32.285) missed the narrow decodable
window and had to wait for the CMD's NEXT connect-frame retransmit — gated by the
`[CMD-POST-TX-CALIB] timeout=30619ms` window (frame_drain=26330 at robust cfg 100,
arq_common.cc:1352). That 30.6s retransmit-wait AMPLIFIES one missed snapshot into ~32s.

## §3 The falsification (VERIFIED, fresh paired runs this session)
Re-ran seed 3003, snr 40, identical params, on THIS machine:
- **REPRO1** (concurrent: build + ON arm running while OFF queued):
  ON rx=55B (stall), **OFF rx=0B (stall)** — FTRT ~1.01. **Legacy ALSO went deaf.**
- **REPRO2** (idle machine, OFF ran FIRST, ON SECOND):
  **OFF rx=12284B CONFIG_16** (connected+climbed), ON rx=6257B CONFIG_0 — FTRT 1.01.

Two facts kill the "redesign-specific deafness" premise:
1. **Legacy (OFF) went deaf at rx=0 on the exact seed the diag claimed legacy always clears**
   (REPRO1). "Legacy clears every one" is FALSE — it is load-dependent.
2. The arm that STALLS is the one that runs SECOND in the session (REPRO1 OFF-second stalled;
   REPRO2 ON-second crawled), i.e. **run-ORDER**, not arm-TYPE, is the dominant predictor.
   The diag ran the two arms at different wall-clock times → asymmetric load → the apparent
   "inband bug" is the slower-arm/higher-load realization, not an inband code path.

Root sensitivity: the robust connect-frame (config 100, ~9s on the wire, decoded ONE snapshot
at a time with ~1.3–3.5s gaps between RSP receive() snapshots) is a **marginal single-shot
acquisition**. When the host runs faster-than-realtime (diag legacy FTRT 0.71) the snapshot
lands in the decodable window; at/above realtime (FTRT ~1.0) the snapshot phase drifts and the
acquisition misses, costing one ~30.6s CMD retransmit cycle — for EITHER arm.

## §4 Secondary REAL effect (not the deafness)
Even deconfounded for connect, the inband ON arm climbs the post-connect ladder noticeably
slower (REPRO2: OFF→CONFIG_16/12.3KB vs ON→CONFIG_0/6.3KB in the same budget). That is a
genuine climb-rate gap and is the legitimate next target — but it is DOWNSTREAM of connect and
is NOT a "deafness." It belongs to the climb investigation, not here.

## §5 Why no fix shipped
- The "deafness" is not inband-caused, so an inband-scoped fix is unjustified and untestable
  (no clean fail-before that is inband-specific — legacy fails too).
- Shortening the 30.6s connect retransmit window WOULD reduce the amplifier, but (a) it helps
  legacy and inband equally (not an inband fix), and (b) it is a timeout change masking a
  marginal acquisition — CLAUDE.md §2 forbids that as the "fix." The correct durable fix, if
  this marginal connect acquisition is worth hardening, is a SIM-FIDELITY / acquisition-margin
  task (multi-phase connect-snapshot retry, or a realtime-pacing FTRT floor in the harness so
  the A/B is run faster-than-realtime symmetrically), pursued under the sim-fidelity campaign —
  NOT a connect-path band-aid on the keystone branch.

## §6 Methodology fix for the A/B harness (the actual actionable item)
The `multi_sample_ab.py` paired A/B runs arms SEQUENTIALLY in one process, so the second arm
inherits residual machine load + a warmer relay → systematic order bias at FTRT≈1. At a
marginal channel this manufactures a fake "arm B is worse" signal. The driver's own output even
warns: "OVERLAP at small N => inconclusive ... move to a NON-marginal SNR where the trajectory
is deterministic." Recommended: (a) run the two arms truly concurrently on disjoint ports, OR
(b) alternate arm order across samples, OR (c) require FTRT < ~0.8 (faster-than-realtime
headroom) before trusting a connect-timing delta. Until then, connect-timing deltas from this
harness at snr 40 are NOT reliable evidence of an arm-specific bug.
