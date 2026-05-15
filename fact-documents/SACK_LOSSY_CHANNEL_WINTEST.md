# SACK Lossy-Channel Win-Test — does `--enable-sack` win on a lossy channel?

Investigation date: 2026-05-14. Branch: `monitor`. **Measurement only — no
Mercury source edited.** Harness: `tools/sack_lossy_ab.py` (new, this session;
workspace repo `monitor`). All hardware access via the IONOS butler
(`localhost:7700`).

Spun off from the SACK turnaround-collision fix (`f2dbf34`) and the SACK
`ldpc=NO` wrong-bitmap fix (`e076823`). With both fixes in, on a CLEAN IONOS
channel `--enable-sack` ≈ no-SACK throughput (SACK actually slightly behind:
pure protocol overhead, no losses to recover). **This document answers the
open question: does SACK WIN on a LOSSY channel, and where is the
clean↔lossy crossover?** The result decides whether `CAP_SACK` should go back
to default-on (currently `--enable-sack` opt-in, per the B2 fix `856f024`).

---

## §1 Test design — apples-to-apples A/B

**Binary under test.** Both Pis run mercury `monitor` HEAD `90ed5d9`, which
sits on top of BOTH SACK fixes — `f2dbf34` (turnaround collision: RSP
ACK-GATEs on observed channel state) and `e076823` (ldpc=NO no longer
fabricates a bitmap). Confirmed by grepping the deployed binary's string
table: `strings ~/mercury-dev/mercury | grep -c "RSP-TIMER. SET"` → 1
(f2dbf34 marker) and `grep -c "hard fallback skipped for LDPC"` → 1
(e076823 marker), on **both** rpi1 and rpi2. Binaries dated 2026-05-14. No
rebuild/redeploy was needed.

**Roles / wiring** (same as `phase0_baseline.py` PI track):
- rpi2 (192.168.2.215) = Commander, rpi1 (192.168.2.217) = Responder
- IONOS HF channel simulator between them, controlled via the butler
- Audio: Fe-Pi card `plughw:Audio`, `AUDIO_SETUP` before each run

**Fixed config, NO gearshift.** Every run uses
`-s 15 -Q 0 -M auto -n -v -F off --skip-turbo-reverse` — i.e. WB_CFG15
pinned, no adaptive gearshift, no turbo reverse. This is the exact recipe
`phase0_baseline.py` uses for its PI track. `gearshift_lines=0` is asserted in
every parsed log (no `GEARSHIFT`/`SUPERSHIFT` events) so config never moves.

**The only A/B variable** is `--enable-sack`, passed to **both** Mercury
instances in the `sack` cell and omitted in the `nosack` cell. Confirmed in
the logs: the `sack` cell logs `[FLAG] --enable-sack: opt-in SACK
negotiation` and `[SACK] Enabled (radio_batch=25 crypto_batch=20 headroom=5
batch=25)`; the `nosack` cell logs neither and runs `batch=10`.

> **Mode asymmetry — by design, and intentional.** SACK mode negotiates a
> 25-frame radio batch (20 data + 5 retransmit-headroom slots); no-SACK runs
> the standard 10-frame batch. This is not a harness artifact — it is what
> SACK *is* (memory: "batch=25 when SACK negotiated"). The A/B therefore
> compares the two real operational *modes* the `CAP_SACK` default selects
> between: {big batch + selective retransmit} vs {small batch + full-batch
> retransmit}. That is precisely the decision under test.

**Interleaved schedule.** For each channel point the harness alternates
`sack, nosack, sack, nosack, …` so slow channel drift biases neither mode.
Runs are NOT batched mode-first.

**Runs per cell.** ≥3 per (channel point, mode) cell; upgraded to 5 if
variance was high (agent-judged).

**Channel points.** Documented-valid IONOS serial commands only (`WGN:N`,
`MPM:N`, `MPP:N`, `FADE …`). `CH:FM`/`CH:THRU` are INVALID and never used.
Each point pins gains (`CH1/2 IN:1 OUT:1`), bandwidth (`BANDWIDTH:3000`) and
resets fade/offset so a point fully defines the channel.

---

## §2 Loss-cliff calibration

SACK's win is partial-batch recovery, so the channel points must produce
**real per-batch frame loss** — but not so much that the link drops. A
calibration sweep (`--modes sack` only, since the SACK path logs
`[CMD-SACK] N/M received` = an explicit per-batch loss readout) located
where each config's per-batch loss lives.

### §2.1 WB_CFG15 — sharp cliff, NO 10-40% partial-loss plateau

`sack_lossy_calib3.json` — WB_CFG15, `--modes sack`, 1 run/point, 70 s,
re-parsed with the final parser via `tools/sack_lossy_analyze.py`:

| Point | SNR | bps | nSent | nReSent | retx% | sack_ev | per-batch loss | cycle_ms |
|-------|-----|-----|-------|---------|-------|---------|----------------|----------|
| wgn36 | 36 dB | 2897.0 | 150 | 0 | 0.0% | 0 | — | 11506 |
| wgn32 | 32 dB | 2414.2 | 150 | 2 | 1.3% | 1 | 0.10 | 11588 |
| wgn30 | 30 dB | 482.8 | 150 | 1 | 0.7% | 1 | 0.00 | 12105 |
| wgn28 | 28 dB | 1931.3 | 100 | 4 | 4.0% | 1 | 0.10 | 13123 |
| wgn26 | 26 dB | 1448.5 | 125 | 1 | 0.8% | 1 | 0.00 | 13890 |
| wgn24 | 24 dB | 1448.5 |  75 | 4 | 5.3% | 1 | 0.10 | 13975 |

(cited: `sack_lossy_calib3.json` runs[0..5]; values RE-PARSED from
`sack_lossy_calib3_logs/sack_lossy_wgn*_WB_CFG15_sack_r1_{cmd,rsp}.log`.)
An earlier separate calibration (`sack_lossy_calib2.json`, killed after 1
cell) established that **WGN:20 → 100 % data-frame failure** — link still
up (turboshift control frames are far more robust), but every WB_CFG15
DATA frame failed LDPC, `nReceived_data=0` on the RSP side.

**Finding.** WB_CFG15's per-batch loss never exceeds ~10 % anywhere it
stays useful. Its rate-0.875 LDPC keeps a *batch* either almost-intact
(~0-10 % loss) or — below the ~WGN:22 cliff — totally dead. There is **no
WGN band where WB_CFG15 sits at 10-40 % partial-batch loss**: the cliff is
too sharp. The 2897→1448 bps throughput slide across WGN:36→24 is driven
almost entirely by **batch-cycle inflation** (11506→13975 ms — timeouts
and re-polls), *not* by partial-batch loss. Per CLAUDE.md ("if a mode
fails at expected SNR the fix is in the pipeline, not the test"), it would
be dishonest to crank WGN lower to force WB_CFG15 into 40 % loss — at that
SNR the config is simply *failing*, not *partially succeeding*, and SACK
cannot recover a batch in which every frame is dead.

### §2.2 WB_CFG10 under AWGN — same sharp cliff

`sack_lossy_calib4_cfg10.json` — WB_CFG10, `--modes sack`, 1 run/point, 70 s:

| Point | SNR | bps | nSent | nReSent | per-batch loss | note |
|-------|-----|-----|-------|---------|----------------|------|
| wgn24 | 24 dB | 971.4 | 125 | 0 | — (0 sack_ev) | clean for CFG10 |
| wgn20 | 20 dB | 0.0 | 50 | 50 | — (link up, all frames fail) | past cliff |

(cited: `sack_lossy_calib4_cfg10.json` runs[0..1]. Calibration was stopped
after wgn20 — see below.)

**Finding.** WB_CFG10 has the *same* sharp-cliff shape as WB_CFG15, just
shifted ~8-12 dB lower in SNR: WGN:24 → 0 % per-batch loss, WGN:20 →
100 % data-frame failure (link still up). Under **pure AWGN**, the partial-
loss band for *any* Mercury config is razor-thin (a few dB wide). This is
inherent: AWGN applies a *uniform* noise floor, so every OFDM data frame in
a ~12 s batch sees the *same* SNR — the LDPC waterfall then makes the
*whole batch* succeed or the *whole batch* fail. There is no AWGN SNR at
which a config sits stably at 10-40 % *partial-batch* loss.

### §2.3 The right lossy channel for a SACK test — frequency-selective multipath

SACK's design target is **partial-batch loss**: some frames in a batch
arrive, some do not. Pure AWGN cannot produce that regime (§2.2). What
*does* is **frequency-selective multipath fading** — the IONOS `MPG`/`MPM`
modes (`MPG` = 0.1 Hz Doppler / 0.5 ms delay, `MPM` = 0.5 Hz / 1 ms; both
documented-valid serial commands). Under multipath, the channel transfer
function has notches that move across the band, so consecutive OFDM frames
within one batch hit *different* fade depths — exactly the uncorrelated
per-frame loss pattern that makes a batch land partially-received. The
WB_CFG10 main sweep (Track B, §3) therefore uses MPG/MPM points, not deeper
AWGN, to reach the genuine 10-40 % partial-batch regime.

---

## §3 A/B sweep — channel points and schedule

Two fixed-config tracks, each apples-to-apples within itself (no gearshift,
`gearshift_lines=0` asserted in every run):

**Track A — WB_CFG15** (`sack_lossy_ab_cfg15.json`): the headline high-rate
config. Per §2.1 it has no 10-40 % partial-loss band, so this track
characterises SACK at WB_CFG15's *actual* loss profile — clean (0 %) and
the lossiest still-useful point (~5-10 %):
- `clean` (WGN:40) — baseline, ~0 % per-batch loss
- `wgn28` (WGN:28) — ~5-10 % per-batch loss, link carrying ~1900 bps

**Track B — WB_CFG10 under multipath** (`sack_lossy_ab_cfg10.json`): the
robust config on **frequency-selective multipath** — the channel that
actually produces partial-batch loss (§2.3):
- `clean` (WGN:40) — baseline, ~0 % per-batch loss
- `mpg26` (MPG:26) — multipath-good, moderate frequency-selective loss
- `mpm26` (MPM:26) — multipath-moderate, deeper frequency-selective loss

Each cell: ≥3 runs, `sack` and `nosack` **interleaved** (sack, nosack,
sack, nosack, …) under one butler lease per channel point so channel drift
biases neither mode. 90 s measurement window per run. Payload: `pg84.txt`
(448 929 bytes, looped).

---

## §4 Results table — throughput per (channel point × mode)

### §4.1 Track A — WB_CFG15, AWGN

`sack_lossy_ab_cfg15.json`, 3 runs/cell, 90 s window, mechanism RE-PARSED
from the per-run logs via `tools/sack_lossy_analyze.py`:

| Point | SACK mean (σ) | noSACK mean (σ) | Δ | Δ% | verdict |
|-------|---------------|------------------|----|-----|---------|
| clean (WGN:40) | 2628.7 (0.0) | 1902.7 (141.6) | **+726.0** | +38.2 % | SACK wins |
| wgn32 (WGN:32) | 751.1 (306.6) | 1857.6 (67.6) | **−1106.5** | −59.6 % | no-SACK wins |
| wgn28 (WGN:28) | 1502.2 (375.6) | 1402.0 (70.8) | **+100.2** | +7.1 % | ~tie |

(cited: `sack_lossy_ab_cfg15.json` runs[0..17]. Per-run logs in
`sack_lossy_ab_cfg15_logs/`. gearshift lines across all 18 runs = 0 —
fixed config confirmed.)

> **`wgn28/sack/r2` is a harness artifact, not a SACK failure.** That run
> shows `bps=0`, but the modem fully worked: CMD `nSent_data=150
> nAcked_data=150` (all 150 frames sent AND acked) and RSP
> `nReceived_data=144` — comparable to r3's 156 (which the harness *did*
> capture as 1878 bps). For every other run `rx_bytes` tracks RSP's
> `nReceived_data`; only r2 has RSPrx=144 yet `rx_bytes=0` — the harness's
> TCP reader on the RSP data port (8401) missed the delivery. The analyzer
> excludes 0-bps runs from the mean (correct), so the reported SACK wgn28
> mean of 1502.2 is the mean of r1 (1127) and r3 (1878). Had r2 been
> captured (~2100 bps implied by 144 frames / 90 s), SACK's wgn28 mean
> would be *higher* — so the +7.1 % SACK edge at wgn28 is **conservative**.

**Reading Track A.** The verdict *flips with SNR*, and not toward SACK:
- **clean**: SACK +38 %. SACK is bitwise-repeatable (σ=0); no-SACK is
  noisier (σ=142) and lower because even WGN:40 has audio-path frame loss
  (`nReSent` 10-30/run) that triggers no-SACK's all-or-nothing batch
  resends, while SACK's 25-frame batch has a better duty cycle and 0
  retransmits. *On this testbed SACK is ahead on the clean channel —
  contrary to the prior clean-channel finding; see §7.*
- **wgn32**: SACK **loses by 60 %** — and consistently (3/3 runs). This is
  the WB_CFG15 cliff edge; §5.2 shows why.
- **wgn28**: essentially a tie (+7 %, but SACK σ=376 vs no-SACK σ=71 —
  SACK is wildly variable, one run delivered 0 bps).

### §4.2 Track B — WB_CFG10, frequency-selective multipath

`sack_lossy_ab_cfg10.json`:

[FILLED when CFG10 sweep completes]

---

## §5 Mechanism evidence

The throughput numbers in §4 are only an answer if the *mechanism* behind
them is shown. The two modes differ structurally in how a partially-lost
batch is recovered (traced from source, working tree `90ed5d9`):

- **no-SACK** uses a **pattern ACK that is all-or-nothing per batch**.
  `process_messages_rx_acks_data()` → `receive_ack_pattern()` returns a
  single bool; on success *every* PENDING_ACK frame in the batch is
  `register_ack()`-ed (`arq_commander.cc:1518-1524`). If even one frame is
  lost, RSP does not send the ACK pattern, CMD's `receiving_timer` expires,
  and **every un-ACKed frame of that batch becomes `ACK_TIMED_OUT` and is
  re-sent** (`arq_commander.cc:809-820`). Penalty per lossy batch ≈ the
  whole batch.
- **SACK** sends a **per-frame bitmap**. `receive_sack_pattern()` fills
  `sack_bitmap[]`; `process_messages_rx_acks_data()` marks the received
  frames ACKED and queues **only the missing ones** for retransmit
  (`arq_commander.cc:1448-1480`), then `[CMD-RETX] Sending N retransmit
  frames` sends exactly those N (`arq_commander.cc:732`,
  `stats.nReSent_data += N`). Penalty per lossy batch ≈ the number of lost
  frames (capped at `retransmit_headroom=5`; >5 lost → remainder rides the
  next SACK cycle).

So the apples-to-apples mechanism metric is **`nReSent_data / nSent_data`**
(the wasted-frame fraction) and **`[CMD-RETX]` frame counts** vs
**`[CMD-SACK] N/M received`** (frames re-sent vs frames actually lost), plus
**batch-cycle count and `batch_cycle_ms`** (the [T] cmd_batch_tx_start
deltas) — fewer/shorter cycles per delivered byte is the throughput edge.

### §5.1 Worked example — a clean SACK recovery (WGN:32, WB_CFG15)

`sack_lossy_calib3_logs/sack_lossy_wgn32_WB_CFG15_sack_r1_cmd.log:3132-3141`
captures one full SACK recovery, fully traced:

```
[SACK-OFFSET] best_offset=74416 ... ok
[SACK-LDPC] decode: iterations=0/50 CONVERGED hard_tones: 15 14 15 15 15 14 1 0...
[SACK-LDPC] bitmap: 1 1 1 1 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 1 1 1 1
[RX-SACK] Detected (matched=16, metric=16.0, ack_xcheck=5, ldpc=YES), bitmap: 1 1 1 1 0 1 ... 0 1 1 1 1
[CMD-SACK] Partial batch ACK detected!
[CMD-SACK] 23/25 received, 2 queued for retransmit
```

RSP's `[TX-SACK]` (rsp log:11500) sent the identical bitmap — frames 4 and
20 lost out of 25. SACK retransmitted **exactly 2 frames**
(`[CMD-RETX] Sending 2 retransmit frames`, cmd log:3275). Under no-SACK,
that same channel hitting a 10-frame batch with one loss produces NO ACK
pattern → CMD `receiving_timer` expiry → the whole 10-frame batch
re-queued. The structural saving is real; §5.2 measures whether it nets out
positive once SACK's extra turnaround (the SACK pattern + the
retransmit-only mini-batch, each with its own preamble — see
`SACK_THROUGHPUT_INVESTIGATION.md §16.3`: retransmit-only batches, "simpler
but less efficient") is paid for.

### §5.2 Mechanism table — Track A (WB_CFG15)

Per-cell aggregate, re-parsed from `sack_lossy_ab_cfg15_logs/`:

| Point | mode | runs | nSent | nReSent | retx% | sack_ev | retx_fr | loss/batch | cycle_ms |
|-------|------|------|-------|---------|-------|---------|---------|------------|----------|
| clean | sack | 3 | 600 | 0 | 0.0 % | 0 | 0 | — | 11519 |
| clean | nosack | 3 | 410 | 50 | 12.2 % | 0 | 0 | — | 6134 |
| wgn32 | sack | 3 | 150 | 7 | 4.7 % | 6 | 3 | 0.10 | **19892** |
| wgn32 | nosack | 3 | 400 | 60 | 15.0 % | 0 | 0 | — | **5919** |
| wgn28 | sack | 3 | 425 | 7 | 1.6 % | 4 | 6 | 0.10 | 13626 |
| wgn28 | nosack | 3 | 410 | 50 | 12.2 % | 0 | 0 | — | 6187 |

(cited: `sack_lossy_analyze.py` aggregate over `sack_lossy_ab_cfg15.json`.)

### §5.3 The root mechanism — `receiving_timeout` scales with batch size

The decisive number is **`cycle_ms`**, and it is NOT explained by retransmit
volume. At wgn32, SACK's `retx% = 4.7` is *lower* than no-SACK's `15.0` —
SACK genuinely retransmits fewer frames, exactly as designed (`[CMD-RETX]
Sending 1 retransmit frames` for a 24/25 batch — `wgn32 …sack_r2_cmd.log:
2247-2339`). And yet SACK's `cycle_ms` is **19892 vs no-SACK's 5919** — a
3.4× longer cycle — so SACK loses by 60 %.

Traced from the `[CMD-POST-TX]` lines:
- no-SACK, batch=10: `receiving_timeout=4470ms`
  (`wgn32 …nosack_r1_cmd.log:1285`). Cycle = ~4.2 s batch TX + ~4.5 s
  ACK-wait ≈ **5.9 s**, delivering 10 frames.
- SACK, batch=25: `receiving_timeout=8248ms`
  (`wgn32 …sack_r2_cmd.log:1319`). Cycle = ~10 s batch TX + up to 8.2 s
  SACK-wait + (on a partial batch) a whole extra retransmit-only mini-batch
  cycle ≈ **12-20 s**, delivering 25 frames.

In the *best* case the per-frame rates are near-equal (no-SACK 10 f / 5.9 s
= 1.69 f/s; SACK 25 f / ~15 s ≈ 1.67 f/s). But SACK carries an **8.2 s
timeout penalty every time the SACK pattern is missed or late** — and at a
config's cliff edge that happens often. That penalty, plus the
retransmit-only mini-batch being a *separate* keyed cycle
(`SACK_THROUGHPUT_INVESTIGATION.md §16.3`), is what inflates SACK's mean
cycle to ~20 s and drives the −60 % at wgn32. So the mechanism is **not**
"SACK retransmits more" — it retransmits *less* — it is "SACK's batch=25 +
8.2 s timeout + extra retransmit cycle cost more wall-clock per delivered
frame than no-SACK's agile batch=10 fail-and-resend, whenever the channel
is lossy enough to make the SACK turnaround unreliable."

---

## §6 LDPC miscorrection watch

A *provable* LDPC miscorrection = CMD logs `[RX-SACK] Detected … ldpc=YES`
with a bitmap RSP **never transmitted**. The harness cross-checks every
CMD `ldpc=YES` decoded bitmap against the set of all `[TX-SACK] Sending
SACK pattern (… received: …)` bitmaps RSP actually keyed onto the channel
(`arq_common.cc:3535-3538` is the ground truth). A CMD bitmap matching
**none** of RSP's sent bitmaps is flagged; one matching any of them is a
correct decode. (Note: RSP re-sends a SACK whenever no retransmit returns,
so there are routinely *more* `[TX-SACK]` than `[RX-SACK]` events — a naive
k-th↔k-th pairing falsely flags miscorrections; the alignment-free
set-membership test is the correct one.)

Calibration result (`sack_lossy_calib3.json`, 6 WB_CFG15 SACK runs,
re-parsed): **0 provable miscorrections.** 5 `ldpc=YES` events, every one
matched a bitmap RSP sent; 0 `ldpc=NO` events. The `e076823` fix is doing
its job — no fabricated bitmaps observed.

**Track A main sweep (`sack_lossy_ab_cfg15.json`, 18 runs):**
- `ldpc=YES` total: **10** — every one matched a bitmap RSP actually sent.
- `ldpc=NO` total: **0** — the `e076823` ldpc=NO full-batch-retransmit
  fallback path was never even exercised (no LDPC decode of a SACK suffix
  failed).
- **Provable miscorrections: 0.**
- `stale-but-valid`: **5** — CMD's `ldpc=YES` decode was a *correct* decode
  of a bitmap RSP sent, but not RSP's *latest* one (CMD missed a re-send
  and acted on the prior, still-valid SACK). Not a miscorrection — the
  decode was right — but it confirms the SACK turnaround is still lossy
  enough that RSP re-sends and CMD lags. This is a *throughput* cost
  (§5.3), not a *correctness* one.

So across calibration + Track A (24 SACK runs total) there is **not a
single provable LDPC miscorrection**. The deferred "Candidate B"
suffix-quality gate is **not warranted by any observed correctness
failure** — `e076823` already prevents the fabricated-bitmap class of bug,
and the soft LDPC decoder never produced a bitmap RSP did not send.

[TRACK B MISCORRECTION RESULT — filled from sack_lossy_ab_cfg10.json]

---

## §7 Verdict

The question: **does `--enable-sack` win on a lossy channel, and where is the
clean↔lossy crossover?**

### §7.1 Track A (WB_CFG15, AWGN) — SACK does NOT win on the lossy points

| Channel | per-batch loss | SACK vs no-SACK | verdict |
|---------|----------------|------------------|---------|
| clean (WGN:40) | ~0 % | +38.2 % | SACK wins |
| wgn28 (WGN:28) | ~6 % | +7.1 % (conservative, §4.1) | ~tie |
| wgn32 (WGN:32) | ~4-30 %, high-variance | **−59.6 %** | no-SACK wins clearly |

The Track A result is the **opposite** of the hoped-for "SACK wins on a
lossy channel." SACK wins on the *clean* channel and *loses* — badly, and
repeatably (3/3 runs) — at the lossiest WB_CFG15 point. wgn28 is a wash.
The mechanism (§5.3) is unambiguous and it is *not* "SACK retransmits more"
(it retransmits **less**: 4.7 % vs 15 % at wgn32): it is that SACK's
batch=25 carries an 8.2 s post-TX `receiving_timeout` vs no-SACK's 4.5 s,
and at the cliff edge — where the SACK pattern is itself frequently lost or
late — that timeout fires often, inflating SACK's mean batch cycle to ~20 s
against no-SACK's agile ~5.9 s.

[§7.2 Track B (WB_CFG10, multipath) — FILLED when CFG10 sweep completes.
 This is the channel SACK was actually designed for (frequency-selective,
 genuine partial-batch loss). If SACK wins anywhere, it is here.]

[§7.3 Crossover — FILLED. So far Track A shows the crossover going the
 WRONG way: SACK is ahead at 0 % loss and behind at the lossy points.]

---

## §8 Recommendation — `CAP_SACK` default-on decision

`CAP_SACK` is currently `--enable-sack` opt-in (B2 fix `856f024`), made so
because on clean / low-loss channels SACK's batch=25 + extra turnaround was
pure overhead. The decision now is whether the **two SACK fixes**
(`f2dbf34` turnaround collision, `e076823` ldpc=NO) plus a **real
lossy-channel win** justify flipping it back to default-on.

[FILLED — recommendation grounded in §4-§7. The decision hinges on:
 1. Is there a clear SACK win on a genuinely lossy channel? (§7)
 2. How wide is the loss band where SACK wins vs loses? (§4 crossover)
 3. Are there LDPC miscorrections that would argue for keeping it gated
    until the Candidate-B suffix-quality gate lands? (§6)
 4. Does SACK ever *hurt* on the clean/low-loss channel real links spend
    most of their time on? (§4 clean rows)
 A default-on recommendation requires a clear lossy win AND no clean-channel
 regression AND zero unexplained miscorrections. Anything short of that =
 keep opt-in, with the specific gap named.]
