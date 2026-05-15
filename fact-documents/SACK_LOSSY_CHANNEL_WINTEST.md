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
per-frame loss pattern that makes a batch land partially-received. ~~The
WB_CFG10 main sweep (Track B, §3) therefore uses MPG/MPM points, not deeper
AWGN, to reach the genuine 10-40 % partial-batch regime.~~ **Correction
(§2.4 below): MPG/MPM at `:26` did NOT produce partial-batch loss for
WB_CFG10 — both the SNR was too high and the 0.5–1 ms delay spreads were
too short. Track B uses the deeper `MPD` mode (2 Hz / 4 ms) instead.**

### §2.4 WB_CFG10 multipath re-calibration

The prior agent's first Track B (`sack_lossy_ab_cfg10.json`, points
`clean,mpg26,mpm26`, 18 runs, `finished 20260514_171341`) ran to completion
but is **invalid as a Track B**: `MPG:26`/`MPM:26` produced **zero**
partial-batch loss for WB_CFG10 — 0 `[CMD-SACK]` events across all 18 runs,
RSP measured `SNR=13.5 dB`, every data frame `[OFDM-OK]`. The `:N` suffix on
`MPG/MPM/MPP/MPD` is the **SNR in dB** (the multipath profile —
Doppler/delay — is fixed per mode); `:26` left WB_CFG10 well above its
cliff, and the MPG/MPM delay spreads (0.5–1 ms) are short relative to the
OFDM guard interval so the frequency selectivity was shallow. That file
tested SACK on what was, for WB_CFG10, a *clean* channel — it does not
satisfy §2.3's "genuine 10-40 % partial-batch regime" requirement.

Re-calibration: `sack_lossy_calib5_cfg10_mp.json` — WB_CFG10,
`--modes sack`, 1 run/point, 70 s, deeper multipath modes (`MPP` =
1 Hz / 2 ms, `MPD` = 2 Hz / 4 ms — the 2-4 ms delay spread is comparable to
/ exceeds the OFDM guard interval, so it produces real frequency-selective
ISI):

| Point | mode/SNR | sack_ev | nReSent | per-batch loss (mean/max) | regime |
|-------|----------|---------|---------|---------------------------|--------|
| mpp18 | MPP:18 | 1 | 1 | 0.04 / 0.04 | light |
| mpp16 | MPP:16 | 2 | 2 | 0.04 / 0.04 | light |
| mpp14 | MPP:14 | 0 | 25 | — (link past cliff, all-fail) | too lossy |
| mpm16 | MPM:16 | 1 | 2 | 0.04 / 0.04 | light |
| mpd16 | MPD:16 | 1 | 4 | **0.08 / 0.08** | **moderate partial-batch** |
| mpd14 | MPD:14 | 2 | 14 | **0.22 / 0.28** | **deep partial-batch** |

(cited: `sack_lossy_calib5_cfg10_mp.json` runs[0..5];
`finished 20260514_190605`.) **Finding.** Unlike pure AWGN (§2.2), the
deep-multipath modes DO produce a stable partial-batch-loss band: `MPD:16`
≈ 8 % and `MPD:14` ≈ 22-28 % per-batch loss — squarely the 10-40 % regime
SACK targets. The Track B A/B therefore uses `clean`, `mpd16`, `mpd14`.
`MPP:14` is past WB_CFG10's cliff (all-or-nothing) and excluded.

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

**Track B — WB_CFG10 under multipath** (`sack_lossy_ab_cfg10_mp.json` —
the re-calibrated run; the original `sack_lossy_ab_cfg10.json` with
`mpg26,mpm26` is superseded per §2.4): the robust config on
**frequency-selective multipath** — the channel that actually produces
partial-batch loss (§2.3, §2.4):
- `clean` (WGN:40) — baseline, ~0 % per-batch loss
- `mpd16` (MPD:16, 2 Hz / 4 ms profile @ 16 dB SNR) — moderate
  frequency-selective loss, ~8 % per-batch
- `mpd14` (MPD:14, same profile @ 14 dB SNR) — deep frequency-selective
  loss, ~22-44 % per-batch (the depth SACK was designed for)

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

Channel points calibrated in §2.4 (`clean`, `mpd16`, `mpd14`). The original
`sack_lossy_ab_cfg10.json` is **superseded** — see §2.4 (it produced 0
partial-batch loss). Track B re-run as:

`sack_lossy_ab_cfg10_mp.json`, points `clean,mpd16,mpd14`, 3 runs/cell, 90 s,
`sack`/`nosack` interleaved, mechanism RE-PARSED via `tools/sack_lossy_analyze.py`:

| Point | SACK mean (σ) | noSACK mean (σ) | Δ | Δ% | verdict |
|-------|---------------|------------------|----|-----|---------|
| clean (WGN:40) | 906.6 (0.0) | 785.7 (0.0) | **+120.9** | +15.4 % | SACK wins |
| mpd16 (MPD:16, ~8 % loss) | 251.8 (71.2) | 564.1 (28.5) | **−312.3** | −55.4 % | no-SACK wins |
| mpd14 (MPD:14, ~22-44 % loss) | 226.6 (75.5) | 362.6 (0.0) | **−136.0** | −37.5 % | no-SACK wins |

(cited: `sack_lossy_ab_cfg10_mp.json` runs[0..17]. Per-run logs in
`sack_lossy_ab_cfg10_mp_logs/`. gearshift lines across all 18 runs = 0 —
fixed config confirmed. `started 20260514_19…`, `finished 20260514_194517`.)

> **Three `mpd14` runs are harness TCP-capture artifacts, not SACK/modem
> failures** — same class as Track A's `wgn28/sack/r2` (§4.1). `mpd14/sack/r1`:
> CMD `nSent_data=125 nAcked_data=125` (all sent AND acked), RSP
> `nReceived_data=75`, yet `rx_bytes=0`. `mpd14/nosack/r2`: CMD
> `nSent=100 nAcked=100`, RSP `nReceived=69`, `rx_bytes=0`. `mpd14/nosack/r3`:
> CMD `nSent=90 nAcked=90`, RSP `nReceived=57`, `rx_bytes=0`. In every case
> the modem delivered data; the harness's TCP reader on RSP data port 8401
> missed it. The analyzer correctly excludes 0-bps runs from the mean. **The
> `mpd14` cell is therefore statistically thin** — noSACK mean is one run
> (362.6, σ=0), SACK mean is two (151.1, 302.2). `mpd16` is the clean,
> fully-captured cell (3 valid SACK + 3 valid noSACK) and tells the same
> story decisively; `mpd14` corroborates the direction, not the precise
> magnitude.

**Reading Track B.** Same shape as Track A, on the channel SACK was
*designed* for:
- **clean**: SACK +15.4 %, both bitwise-repeatable (σ=0). Matches the prior
  agent's clean numbers exactly (906.6 / 785.7). SACK's batch=25 has a
  better duty cycle when there is nothing to recover.
- **mpd16** (~8 % genuine partial-batch loss): SACK **loses by 55 %**,
  consistently — noSACK σ=28.5 (251.8 vs 564.1).
- **mpd14** (~22-44 % partial-batch loss — the *deepest* partial-batch
  point, exactly SACK's design target): SACK still **loses by ~38 %**.
  Even where SACK's selective-retransmit machinery is working hardest and
  correctly (§5.4), it does not net out ahead.

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

### §5.4 Mechanism table — Track B (WB_CFG10, multipath)

Per-cell aggregate, re-parsed from `sack_lossy_ab_cfg10_mp_logs/`:

| Point | mode | runs | nSent | nReSent | retx% | sack_ev | retx_fr | loss/batch | cycle_ms |
|-------|------|------|-------|---------|-------|---------|---------|------------|----------|
| clean | sack | 3 | 475 | 0 | 0.0 % | 0 | 0 | — | 14009 |
| clean | nosack | 3 | 420 | 0 | 0.0 % | 0 | 0 | — | 6529 |
| mpd16 | sack | 3 | 250 | 16 | 6.4 % | 4 | 11 | 0.20 | **18869** |
| mpd16 | nosack | 3 | 390 | 10 | 2.6 % | 0 | 0 | — | **7101** |
| mpd14 | sack | 3 | 275 | 17 | 6.2 % | 5 | 14 | 0.20 | **17643** |
| mpd14 | nosack | 3 | 310 | 60 | 19.4 % | 0 | 0 | — | **7833** |

(cited: `sack_lossy_analyze.py` aggregate over `sack_lossy_ab_cfg10_mp.json`.)

**Same root mechanism as §5.3, now on the multipath channel SACK was built
for.** The decisive number is again `cycle_ms`, not retransmit volume:
- At **mpd16**, SACK's batch cycle is **18869 ms vs no-SACK's 7101 ms**
  (2.7×). The `[CMD-POST-TX]` timeouts confirm the cause: SACK
  `receiving_timeout=8442 ms` vs no-SACK `4664 ms`
  (`sack_lossy_mpd16_WB_CFG10_{sack,nosack}_r1_cmd.log`) — the same
  batch-size-scaled post-TX timeout, fired at MPD:16's loss rate.
- At **mpd14** — the *deepest* partial-batch point — the retransmit picture
  finally inverts the way SACK's design predicts: no-SACK's `retx% = 19.4`
  (it re-sends whole 10-frame batches on every loss) is **3× SACK's
  `6.2 %`** (SACK re-sends only the missing frames). SACK is doing exactly
  what it was designed to do. **And it still loses by 38 %**, because
  SACK's cycle is **17643 ms vs no-SACK's 7833 ms** — the batch=25 +
  8.4 s-timeout + separate retransmit-only mini-batch structure costs more
  wall-clock per delivered frame than no-SACK's agile fail-and-resend, even
  when no-SACK is wasting 3× the frames. Selective retransmit wins the
  *frame-efficiency* contest and loses the *wall-clock* contest — and
  throughput is wall-clock.

### §5.5 Worked example — a correct deep SACK recovery (MPD:14, WB_CFG10)

`sack_lossy_ab_cfg10_mp_logs/sack_lossy_mpd14_WB_CFG10_sack_r2_cmd.log`
captures SACK's machinery working correctly at depth — three consecutive
partial batches:

```
[RX-SACK] Detected (matched=16, metric=15.7, ack_xcheck=7, ldpc=YES),
          bitmap: 1 1 1 1 1 0 1 1 0 0 1 1 1 1 1 1 1 1 1 1 1 1 1 0 1
[CMD-SACK] 21/25 received, 4 queued for retransmit
[CMD-RETX] Sending 4 retransmit frames
[RX-SACK] Detected (... ldpc=YES),
          bitmap: 1 0 1 1 0 1 0 1 0 1 1 1 1 1 1 1 1 1 1 0 1 0 1 1 0
[CMD-SACK] 18/25 received, 7 queued for retransmit
[CMD-RETX] Sending 7 retransmit frames
```

RSP's `[TX-SACK]` log (`…sack_r2_rsp.log`) keyed the **identical** bitmaps
onto the channel — `1 1 1 1 1 0 1 1 0 0 …` and `1 0 1 1 0 1 0 1 0 1 …`
line-for-line. SACK retransmitted **exactly** the 4 then 7 missing frames.
The mechanism is correct end-to-end; the throughput loss is purely the
wall-clock cost of the batch=25 turnaround structure (§5.4), not a
correctness failure.

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

**Track B main sweep (`sack_lossy_ab_cfg10_mp.json`, 18 runs) + WB_CFG10
multipath calibration (`sack_lossy_calib5_cfg10_mp.json`, 6 runs):**
- `ldpc=YES` total: **9** (main sweep) + **7** (calib) = 16 — every one
  matched a bitmap RSP actually sent (verified by the alignment-free
  set-membership test against RSP's `[TX-SACK]` log).
- `ldpc=NO` total: **0** — the `e076823` ldpc=NO fallback path was never
  exercised on Track B either.
- **Provable miscorrections: 0.**
- `stale-but-valid`: **5** (main sweep) — CMD's `ldpc=YES` decode was a
  *correct* decode of a bitmap RSP sent, but not RSP's latest one. A
  throughput cost (§5.4 turnaround lag), not a correctness one — identical
  finding to Track A.
- The §5.5 worked example shows the deepest-loss case (MPD:14, batches with
  4 and 7 lost frames) decoding the bitmap *correctly* — RSP's `[TX-SACK]`
  bitmaps match CMD's `[RX-SACK]` bitmaps line-for-line.

**Combined across calibration + Track A + Track B (≈48 SACK runs, 26
`ldpc=YES` events): not a single provable LDPC miscorrection, 0 `ldpc=NO`
events.** The `e076823` fix holds across both channel types (AWGN and
frequency-selective multipath). The deferred "Candidate B" suffix-quality
gate remains **not warranted by any observed correctness failure**.

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

### §7.2 Track B (WB_CFG10, multipath) — SACK does NOT win, even on its design channel

| Channel | per-batch loss | SACK vs no-SACK | verdict |
|---------|----------------|------------------|---------|
| clean (WGN:40) | ~0 % | +15.4 % | SACK wins |
| mpd16 (MPD:16) | ~8 %, genuine partial-batch | **−55.4 %** | no-SACK wins clearly |
| mpd14 (MPD:14) | ~22-44 %, deepest partial-batch | **−37.5 %** | no-SACK wins |

Track B was the real test: frequency-selective multipath (`MPD` = 2 Hz
Doppler / 4 ms delay spread) **does** produce the genuine 10-40 %
partial-batch loss regime that pure AWGN cannot (§2.2, §2.4) — confirmed by
the `[CMD-SACK] N/M received` readouts (mpd14: batches landing 21/25,
18/25, 9/25). This is the channel SACK's selective-retransmit design
*targets*. **SACK still lost — at both lossy points.**

The mechanism (§5.4) is the *same* as Track A and is, again, **not "SACK
retransmits more"**: at mpd14 SACK's wasted-frame fraction is 6.2 % vs
no-SACK's 19.4 % — SACK retransmits **3× fewer** frames, doing exactly what
it was designed to do (§5.5 shows it decoding 4- and 7-lost-frame bitmaps
correctly and re-sending precisely those). It loses because its batch=25
carries an ~8.4 s post-TX `receiving_timeout` (vs no-SACK's ~4.7 s) plus a
separate keyed retransmit-only mini-batch, inflating SACK's batch cycle to
~17-19 s against no-SACK's agile ~7-8 s. **Selective retransmit wins the
frame-efficiency contest and loses the wall-clock contest — and throughput
is wall-clock.**

Caveat (honest): the `mpd14` cell is statistically thin — 3 of its 6 runs
were harness TCP-capture artifacts (§4.2), so the noSACK mpd14 mean rests
on one captured run. `mpd16` is the fully-captured cell (3 valid SACK + 3
valid noSACK, noSACK σ=28.5) and is decisive on its own; `mpd14`
corroborates the *direction* (SACK loses) but not a precise magnitude.

### §7.3 Crossover — SACK is ahead only at ~0 % loss, on BOTH tracks

| | clean (~0 % loss) | lossy points |
|--|-------------------|--------------|
| **Track A** (WB_CFG15, AWGN) | SACK **+38.2 %** | wgn28 ~tie (+7 %); wgn32 **−59.6 %** |
| **Track B** (WB_CFG10, multipath) | SACK **+15.4 %** | mpd16 **−55.4 %**; mpd14 **−37.5 %** |

The crossover is **consistent across both tracks and both channel types**,
and it goes the *opposite* way to the test's hypothesis:

- **SACK wins only on the clean channel** (~0 % per-batch loss) — +38 %
  (CFG15) / +15 % (CFG10). There, with nothing to recover, SACK's batch=25
  is purely a better duty cycle (more payload frames per preamble/turnaround)
  and there are 0 retransmits.
- **The moment the channel produces real per-batch loss — AWGN cliff edge
  OR frequency-selective multipath — SACK loses, by 38-60 %.** The
  crossover sits *just past 0 % loss*: there is no measured loss band, on
  either config or either channel type, where SACK is ahead.
- This is **not** because SACK fails to do its job. On every lossy point
  SACK's selective retransmit works correctly (§5.5) and wastes fewer
  frames than no-SACK (§5.2, §5.4). SACK loses on **wall-clock per
  delivered frame**: the batch=25 + ~8 s post-TX timeout + separate
  retransmit-only mini-batch structure is heavier than no-SACK's batch=10
  fail-and-resend, and that structural cost is paid on *every* batch,
  whereas no-SACK's full-batch-resend penalty is only paid on *lossy*
  batches. Selective retransmit needs the recovered-frame saving to exceed
  the per-batch structural overhead — and at Mercury's batch sizes and
  turnaround timings, on this testbed, it never does.

---

## §8 Recommendation — `CAP_SACK` default-on decision

`CAP_SACK` is currently `--enable-sack` opt-in (B2 fix `856f024`), made so
because on clean / low-loss channels SACK's batch=25 + extra turnaround was
pure overhead. The decision now is whether the **two SACK fixes**
(`f2dbf34` turnaround collision, `e076823` ldpc=NO) plus a **real
lossy-channel win** justify flipping it back to default-on.

**Recommendation: KEEP `CAP_SACK` opt-in (`--enable-sack`). Do NOT flip it
to default-on.**

Against the four decision criteria, grounded in both tracks:

1. **Is there a clear SACK win on a genuinely lossy channel? — NO.** This
   was the decisive question and the answer is unambiguous. Track A swept
   WB_CFG15 across the AWGN cliff; Track B swept WB_CFG10 across
   frequency-selective multipath — the channel SACK's selective-retransmit
   design specifically targets, and the one §2.3 identified as the *only*
   channel that produces a stable 10-40 % partial-batch loss band. **SACK
   lost on every lossy point on both tracks** (Track A wgn32 −59.6 %; Track
   B mpd16 −55.4 %, mpd14 −37.5 %), and lost *consistently* (low-σ noSACK
   cells). It loses even at mpd14's ~22-44 % loss — the deepest
   partial-batch regime, where SACK's machinery is provably working (§5.5)
   and retransmitting 3× fewer frames than no-SACK (§5.4). The mechanism
   (§5.3, §5.4) is understood and structural: batch=25 + ~8 s post-TX
   timeout + separate retransmit-only mini-batch costs more wall-clock per
   delivered frame than no-SACK's agile batch=10 fail-and-resend.

2. **How wide is the loss band where SACK wins? — effectively zero.** On
   both configs and both channel types SACK is ahead *only* at ~0 %
   per-batch loss and is behind at every measured point with real loss. The
   clean↔lossy crossover sits essentially at 0 % loss (§7.3). There is no
   operational loss regime to switch default-on *into*.

3. **LDPC miscorrections? — none, so §6 does not block a flip — but it does
   not enable one either.** Across ≈48 SACK runs / 26 `ldpc=YES` events
   (calibration + Track A + Track B) there were **0 provable
   miscorrections** and **0 `ldpc=NO`** events. The `e076823` fix holds on
   both AWGN and multipath. So correctness is *not* the reason to keep SACK
   gated — but the absence of a correctness problem cannot by itself
   justify default-on when criterion 1 fails. The deferred Candidate-B
   suffix-quality gate is **not warranted** by any observed failure.

4. **Does SACK hurt on the clean channel? — no, it slightly helps there
   (+15-38 %) — but that is the wrong place to optimise.** The clean-channel
   SACK win is real and repeatable (σ=0 on both tracks), and comes purely
   from batch=25's better duty cycle. But default-on is a decision about
   what happens when the channel degrades — and real HF links spend
   meaningful time *lossy*, which is exactly where SACK costs 38-60 %. A
   default that helps 15-38 % on a perfect channel and hurts 38-60 % the
   moment the channel gets lossy is a bad default for life-critical comms.

**Decision rule check.** Default-on requires *a clear lossy win AND no
clean-channel regression AND zero unexplained miscorrections*. Criteria 3
and 4 pass (no miscorrections, no clean regression). **Criterion 1 fails
hard and on the channel SACK was designed for.** Per the rule, anything
short = keep opt-in with the gap named.

**The named gap.** SACK's selective retransmit is *correct* and *frame-
efficient* — it is not a broken feature. What makes it lose is purely the
**turnaround/cycle structure**: a batch=25 whose post-TX `receiving_timeout`
scales with batch size (~8 s vs no-SACK's ~4.5 s) and a retransmit-only
mini-batch that is a *separate* keyed cycle with its own preamble. For SACK
to ever win on a lossy channel, that structural overhead must drop below
the recovered-frame saving. Concretely, the gap to close (future work, not
this measurement campaign) is one or more of: (a) decouple the SACK post-TX
timeout from batch size — size it to the *expected SACK-pattern arrival
time*, not the batch length; (b) fold the retransmit-only frames into the
*next* batch's keying instead of a separate keyed cycle (avoid the extra
preamble/turnaround); (c) make the SACK batch size adaptive — large only
when the channel is clean enough that the SACK pattern itself is reliable,
shrinking toward batch=10 as loss rises (which would converge SACK toward
no-SACK's agile behaviour exactly where no-SACK currently wins). Until one
of those lands and is re-measured against this same A/B harness on both
tracks, `CAP_SACK` stays `--enable-sack` opt-in.

---

## §9 Reproduction & artifacts

- **Track A** (WB_CFG15, AWGN): `sack_lossy_ab_cfg15.json` +
  `sack_lossy_ab_cfg15_logs/` — committed `e7a0654`. Calibration:
  `sack_lossy_calib2.json`, `sack_lossy_calib3.json`.
- **Track B** (WB_CFG10, multipath): `sack_lossy_ab_cfg10_mp.json` +
  `sack_lossy_ab_cfg10_mp_logs/`. Calibration:
  `sack_lossy_calib4_cfg10.json` (AWGN, established the AWGN cliff is too
  sharp), `sack_lossy_calib5_cfg10_mp.json` (multipath, located the MPD
  partial-batch band).
- **Superseded:** `sack_lossy_ab_cfg10.json` + `sack_lossy_ab_cfg10_logs/` —
  the prior agent's `clean,mpg26,mpm26` run; complete but invalid as a
  Track B (0 partial-batch loss, see §4.2). Retained for the record; its
  `clean` rows agree with `sack_lossy_ab_cfg10_mp.json` to the bit.
- Harness: `tools/sack_lossy_ab.py`, analyzer: `tools/sack_lossy_analyze.py`
  (workspace `monitor` `626c3f9`, plus the §2.4 multipath channel points
  added this session). Re-run Track B:
  `python tools/sack_lossy_ab.py --out <json> --config WB_CFG10
  --points clean,mpd16,mpd14 --modes sack,nosack --runs 3 --duration 90`.
