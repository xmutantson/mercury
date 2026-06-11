# tools/sim/ — device-free FTRT ARQ channel simulator

These drivers run a REAL two-process Mercury ARQ session through a software
channel (Watterson 2-tap fading + SNR3k AWGN) with NO audio hardware, faster
than real time (FTRT). They were migrated here from the outer workspace `tools/`
(2026-06-09, branch `fix/sim-drivers-migration`) so the sim that gates
chase-combine's I7 delivered-bps A/B lives inside the mercury repo it validates.

## Files

| file | role |
|------|------|
| `sim_channel_relay.py` | the software channel. Sits between two `-x sim` mercury peers; applies Watterson fading + CFO + AWGN at a calibrated SNR3k, cross-wires the two directions. Self-contained (stdlib + numpy). |
| `sim_arq_channel.py` | the FTRT harness. Launches the relay + two `mercury.exe -x sim` peers, drives an ARQ transfer, prints/JSON-emits the config timeline, **delivered-byte md5**, and **retx/chase counters** (I7 instrumentation). |
| `effective_rate_calibrate.py` | the IONOS **hardware** Q-table sweep (imports `sack_lossy_ab`). Not the FTRT sim — the HW calibration driver that produces `effective_rate_table.json`. |
| `sack_lossy_ab.py` | HW butler/IONOS run machinery used by the calibrator. |
| `payload_incompressible_64k.bin` | fixed 64 KiB high-entropy payload (md5 `d991876b…`) for the delivered-bps A/B. Regenerate with `gen_incompressible_payload.py`. |
| `gen_incompressible_payload.py` | deterministic regenerator for the payload above. |

## Quick start (FTRT sim — needs a guard-built binary)

```
# clean over-climb cell:
python tools/sim/sim_arq_channel.py --cell WGN:30 --secs 60 --json /tmp/wgn.json
# moderate fading cell:
python tools/sim/sim_arq_channel.py --cell WGN:12 --profile mpm --secs 60 --json /tmp/mpm.json
```

The harness `require_guard_binary()` refuses any `mercury.exe` that lacks the
compiled-in `[SIM-AUDIO-GUARD]` marker (GUARD 2). That marker is emitted by the
`-x sim` branch of `source/audioio/audioio.c` (GUARD 1) and the device threads
abort-before-render if ever entered under sim. Build with `bash build.sh o3`
from a tree containing GUARD 1 and point `--bin` at the result.

## I7 result fields (sim_arq_channel.py --json)

* `payload`, `payload_bytes`, `payload_md5` — the fixed incompressible TX file.
* `delivered_bytes` — bytes the responder data socket delivered.
* `tx_md5` — md5 of the full TX byte stream.
* `recv_md5` — md5 of the delivered byte stream.
* `recv_md5_ref` — md5 of the same-length prefix of the looped TX stream.
* `md5_match` — `recv_md5 == recv_md5_ref` (byte-faithful delivery).
* `n_cmd_retx`, `cmd_retx_frames`, `n_cmd_retx_v2`, `n_sack_retx_lines`,
  `sack_retx_frames`, `nresent_data_max` — generic ARQ retransmission counts
  (parsed from the CMD log).
* `n_chase_fire` — count of `[CHASE]` markers (0 against a monitor binary that
  lacks the chase-combine feature; non-zero once feat/chase-combine ships its
  `[CHASE]` print).

## Inter-peer sample-clock drift + PTT turnaround model (FIX9, OPT-IN)

The conservative-PDES barrier bounds the a2b/b2a virtual-clock split to
`K*1024` samples *by design*, so the default sim is a single-clock, barrier-
locked channel. That **structurally masks** the HW CFG16 climb-collapse, whose
root cause (`bigblock_p3_hw/_fix9/FIX9_ROOTCAUSE.md`) is the two RPis'
**independent soundcard sample-clock drift** ~~(`[CLK-TX] -670.7 ppm`,
`[CLK-RX] -193.0 ppm`)~~ de-aligning the half-duplex turnaround over the longest
(25-frame CONFIG_16) batches until the CMD's reverse MFSK-ACK/SACK correlator
sees silence and BREAKs. These OPT-IN flags add that physical skew back so the
FIX9 D2/D3 turnaround fixes have a failing-first off-bench vehicle.

> **CORRECTION (C4, 2026-06-10):** the ~~`[CLK-TX] -670.7 ppm` / `[CLK-RX]
> -193.0 ppm`~~ figures struck above were a **measurement artifact**, NOT the
> real crystal skew. Those tags (now renamed `[TX-PUSH-RATE]` /
> `[RX-DELIVER-RATE]` in `audioio.c`) report a PRODUCER-PUSH-RATE / 10 s-window
> *quantization* metric that includes zero-fill silence — it swings ±2000 ppm
> in 10 s on a static pair, which no crystal can do (see the audioio.c comment
> + SIMFIDELITY_ROOTCAUSE.md §1). The **true inter-Pi sample-clock skew is
> ±8.16 ppm** (tone method, CLOCK_VERDICT.md §2). The `-670` repro value in the
> table below is therefore an ~80× over-statement; use `±8` for a physically
> faithful skew. The drift de-alignment mechanism is real; only the magnitude
> was wrong.

| flag | default | effect |
|------|---------|--------|
| `--drift-ppm-a2b N` | `0` (off) | re-time the A→B forwarded stream by a sample-clock skew of `N` ppm (`rate_out/rate_in = 1+N/1e6`). Applied AFTER the calibrated channel (noise/taps unchanged). FIX9 repro: `-670`. |
| `--drift-ppm-b2a N` | `0` (off) | same on B→A, set independently (the two soundcards drift independently). |
| `--ptt-latency-ms M` | `0` (off) | inject `M` ms of channel-noise silence at each TX onset (silent→signal edge) per direction — the radio PTT/AGC/capture-flush keying delay the sim idealizes to zero. |
| `--ptt-latency-jitter-ms J` | `0` | ± uniform per-onset jitter on the PTT latency (seeded). Requires `--ptt-latency-ms>0`. |
| `--turnaround-batch-accrual` | `off` | **bench-9 BATCH-LENGTH-DEPENDENT turnaround accrual** (rides on `--turnaround-drift`). Adds a *one-sided* late-shift to the reverse-ACK proportional to how long the forward batch held the channel. Default rate `30.0 ms/s` (`--turnaround-accrual-ms-per-s`). Env mirror: `MERCURY_SIM_BATCH_ACCRUAL=1`. **OFF == legacy symmetric-jitter model, byte-identical.** |

### Why the batch-length accrual (bench-9 calibration)

The `--turnaround-jitter-ms` term above is **zero-mean symmetric**: its accumulated
offset is a random walk whose magnitude grows only as `~sqrt(n_keyups)`. So a long
CFG16 batch (25-30 frames, one reverse-ACK turnaround) misses its window only
`~sqrt(3)=1.7×` more than a short CFG15 batch — **not** the bench-9 STEP. The HW
ground truth (`bigblock_p3_hw/HW_BENCH9/ARMA_CALIBRATION.json`) is a step, not a
ratio:

* **held-CFG16** (`-s 16`, no `-g`, WGN:40 clean, real ±8 ppm + PTT/capture jitter):
  reverse-ACK `matched=0/7` on **14/16** turnarounds = **87.5 % full window miss**,
  link active **18.9 %** of wall, whole-window wire **597.6 bps** (in-burst 3057);
  40 D3 reverse-ACK-starvation demotes over the transfer.
* **held-CFG15** short batches under the SAME drift (bench-8): reverse-ACK lands in
  the window on essentially every batch — sustained **3060 bps** (NOT halved),
  `d2_reverse_ack_fires=0`, 100 % high-rung.

The physical cause: a **longer forward batch holds the half-duplex channel longer**,
so keyer/AGC/capture-flush/scheduling turnaround latency **accumulates within the
batch** and pushes the single end-of-batch reverse-ACK **systematically late** in
proportion to the batch length. That is a one-sided, batch-length-dependent offset —
which a zero-mean symmetric walk cannot produce. The legacy faithful model therefore
gave held-CFG16 **~1992 bps** (too optimistic, link ~50 %+ active); the accrual term
drops it toward the HW **~600 bps** by making the long-batch reverse-ACK miss its
window ~80 %+ of the time while the short CFG15 batch still lands.

**CROSS-DIRECTION application (v2 — `SIMTURNCAL_VERDICT.json`).** The forward
(`a2b` OFDM) and reverse (`b2a` MFSK ACK) directions are **separate**
`TurnaroundDrift` instances. The accrual amount is keyed to the **forward** batch
airtime, but the late silence must be inserted into the **reverse** turnaround GAP
(delaying the `b2a` ACK onset) — **not** into the forward signal. The v1 model
added the late accrual to each direction's *own* just-ended burst and spent it
inserting silence **ahead of the FORWARD signal onset**, which de-aligned the
forward OFDM decode (relay `ins=90313 samp`/286 ms into the forward signal;
`FTR-FAIL ×414`, metric collapse, modem `proc_died`; held-CFG16 stuck ~1897 bps).
The v2 fix couples the two directions through a shared **`TurnaroundCoupler`**:
each direction **publishes** its just-ended forward-burst airtime on its
signal→silence falling edge and **consumes** the *other* direction's pending
forward airtime at its own ACK onset. So the long forward `a2b` OFDM batch pushes
the short `b2a` ACK late (the bench-9 collapse) while the forward OFDM samples stay
**bit-exact** (the forward instance only publishes; it consumes only the tiny
reverse-ACK airtime, realized harmlessly in its own turnaround gap).

Calibration: `30.0 ms/s` × a ~28-frame CFG16 forward batch (~4.78 s airtime) =
~143 ms (~6881 samp) reverse-ACK late-shift `>` the ~90 ms (4320 samp) CMD
reverse-ACK window half-width → MISS; × a ~6-frame CFG15 forward batch (~1.02 s) =
~31 ms (~1475 samp) `<` the window → LANDS. Validated by
`tools/sim/test_sim_relay_turnaround_xdir.py` (the v2 contracts: forward OFDM
bit-exact with accrual ON — the v1 regression; reverse-ACK onset delayed by
`30 ms/s × forward-batch-airtime`; CFG16 forward batch → reverse-ACK MISS /
CFG15 → LAND; CFG16 miss-fraction ≥ 0.80, CFG15 ≤ 0.05; accrual-OFF
byte-identical to the legacy model). `tools/sim/test_sim_relay_turnaround_batchlen.py`
(the v1 self-accrual test) is **superseded** and now delegates to the v2 test.

**Default OFF == byte-identical** to the pre-change (monitor) relay: `ppm==0`
is a strict identity pass-through (no float reconstruction, no state) and
`latency==0` injects nothing. Validated by
`tools/sim/test_sim_relay_drift.py` (model contracts) and the forwarded-wire
md5 A/B in `bigblock_p3_hw/_fix9/driftsim/byte_identity_check.py` (the PDES
determinism contract). Drift CANNOT be combined with `--idle-bigstep>1`
(coalescing silence breaks the continuous resample stream + onset detection);
the relay errors out on that combination.

> The drift is injected on the FORWARDED audio only — `ch.process()` (Watterson
> + AWGN + CFO) runs per inbound chunk exactly as before, so the noise PSD /
> fade realization / SNR3k calibration are untouched. The resampler re-times the
> already-impaired stream, which (under the modem's local-ADD sim clock,
> `audioio.c rx_transfer`) skews the receiving peer's virtual clock against the
> transmitting peer's batch boundaries — the HW de-alignment mechanism.

Repro recipe (highest-config cell + measured HW skew):
```
python tools/sim/sim_arq_channel.py --snr 35 --profile wgn --phase-noise-deg 0 \
    --start-cfg 0 --secs 180 --drift-ppm-a2b -670 --drift-ppm-b2a -193 \
    --json /tmp/fix9_repro.json
```

## Channel calibration (do NOT re-derive without re-validating)

* AWGN matches the BER harness (`telecom_system.cc` f_nyquist / `awgn.cc`); the
  per-sample noise std mirrors it so a sim cell at SNR3k reproduces the harness
  noise PSD. Validated by `tools/test_sim_relay_noise.py` (in the outer tree).
* `--cell WGN:N` ⇒ SNR3k = N + 2.4 dB (testbed WGN-label mapping; true −10 dB =
  WGN:−12).
* Watterson ITU-R MPG/MPM/MPP (fd 0.1/0.5/1.0 Hz, dtau 0.5/1.0/2.0 ms), unit
  mean tap power, Gaussian-Doppler IIR taps.
