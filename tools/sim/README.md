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
**independent soundcard sample-clock drift** (`[CLK-TX] -670.7 ppm`,
`[CLK-RX] -193.0 ppm`) de-aligning the half-duplex turnaround over the longest
(25-frame CONFIG_16) batches until the CMD's reverse MFSK-ACK/SACK correlator
sees silence and BREAKs. These OPT-IN flags add that physical skew back so the
FIX9 D2/D3 turnaround fixes have a failing-first off-bench vehicle.

| flag | default | effect |
|------|---------|--------|
| `--drift-ppm-a2b N` | `0` (off) | re-time the A→B forwarded stream by a sample-clock skew of `N` ppm (`rate_out/rate_in = 1+N/1e6`). Applied AFTER the calibrated channel (noise/taps unchanged). FIX9 repro: `-670`. |
| `--drift-ppm-b2a N` | `0` (off) | same on B→A, set independently (the two soundcards drift independently). |
| `--ptt-latency-ms M` | `0` (off) | inject `M` ms of channel-noise silence at each TX onset (silent→signal edge) per direction — the radio PTT/AGC/capture-flush keying delay the sim idealizes to zero. |
| `--ptt-latency-jitter-ms J` | `0` | ± uniform per-onset jitter on the PTT latency (seeded). Requires `--ptt-latency-ms>0`. |

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
