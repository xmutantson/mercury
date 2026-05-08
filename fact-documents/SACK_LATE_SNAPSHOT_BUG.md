# SACK Late-Snapshot Bug — Investigation (2026-04-20)

## §1 Problem statement

SACK never successfully decodes in live RPi1↔RPi2 direct-cable testing.
Throughput stalls at 400-600 bps at WB CONFIG_10; base rate is ~800 bps.
Partial batches trigger SACK at RSP but CMD cannot decode the SACK bitmap,
so CMD retransmits the entire batch → wasted airtime.

## §2 Evidence

### §2.1 Self-test passes (algorithm is not broken)
`mercury -m PLOT_PASSBAND -s 10` runs `sack_pattern_detection_test()` which
generates SACK in-memory and detects it immediately. Result: matched=16/16,
LDPC bitmap decode PASS.
Source: `mercury/source/physical_layer/telecom_system.cc:3058`.

### §2.2 Audio path is clean
Butler-driven RSP→CMD cable test with 1500 Hz tone: peak 0.52 at RX, 4 dB
loss. No channel issue.
Source: `tools/audio_path_test.py`.

### §2.3 RSP TX audio is correct
Instrumented `generate_sack_bitmap_pattern_passband` to dump the generated
SACK burst to WAV. Analysis of `/tmp/sack_tx_00.wav` (RSP-side):
- 56064 samples = 1168 ms = 48 OFDM symbols (16 base + 32 suffix)
- Per-symbol peak matches the expected Welch-Costas WB M=16 base sequence
  {0, 9, 6, 14, 8, 7, 8, 11, 8, 1, 14, 6, 0, 15, 0, 3} for symbols 0-15
  followed by 32 LDPC-encoded suffix tones.
- All 16 base-pattern tones land at their expected passband frequencies in
  the 1125-1875 Hz range (46.875 Hz spacing, reversed mapping due to
  Mercury's downshift TX — see §3.3 below).

### §2.4 CMD RX audio is MISSING the front of the SACK burst
Instrumented `detect_sack_pattern_from_passband` to dump the passband
snapshot passed to the detector. Analysis of captured RX WAVs:

| WAV                | Samples | Burst position in buffer     | Captured portion  |
|--------------------|---------|------------------------------|-------------------|
| cmd_rx_00.wav      | 93440   | 1050 ms - 1900 ms (of 1947)  | ~850 ms (73% of 1168 ms) |
| cmd_rx_04.wav      | 56064   | 778 ms - 1168 ms (of 1168)   | ~390 ms (33%)     |
| cmd_rx_05.wav      | 56064   | similar to 04                | similar           |
| cmd_rx_06..11      | 130816  | no burst (silent buffer)     | 0%                |

In cmd_rx_04, per-symbol FFT shows symbols 0-31 are pure silence. Symbols
32-47 contain the burst, matching the *last 16* TX symbols (suffix symbols
16-31 of 32). The base pattern (first 16 TX symbols) is gone.

### §2.5 Detector finds only 5-8 / 16 matches
With BASE pattern missing from buffer, detector locks onto coincidental
matches between base-pattern expected tones and suffix tone hops. Reports
`matched=6/16 metric=6.0` at best, below threshold=10 → SACK decode
rejected → CMD falls back to full-batch retransmit.
Source: `mercury/source/physical_layer/telecom_system.cc:2727` [SACK-PER-SYM].

## §3 Diagnostic instrumentation added (uncommitted)

All in `mercury/source/physical_layer/telecom_system.cc`:
- **§3.1** `[SACK-FREQ]` at line 2708 — logs `carrier`, `coarse_off`,
  `effective_carrier`, `M`, `Nc`, `stream_offsets[0]`, buffer `size`.
  **Finding:** coarse_off always 0.000. Ruled out stale frequency offset.
- **§3.2** `[SACK-LEVEL]` after passband_to_baseband — logs `pb_max`,
  `bb_max`, size, interp, Nofdm. **Finding:** for 56064/65408/93440
  buffers, pb_max≈0.35 and bb_max≈0.16 (signal present). For 130816
  "polling" buffers, pb_max≈0.0006 (silent — polled between bursts).
- **§3.3** `[SACK-PER-SYM]` enhanced — prints per-tone energy at expected
  bin, mirror bin, and global peak bin. **Finding:** the M=16 detector
  counts "matched" via `peak_bin == expected_bin OR peak_bin == mirror_bin`
  ([ofdm.cc:3379](../source/physical_layer/ofdm.cc#L3379)); for WB M=16,
  mirror bins happen to ALL fall within the 16-bin M-set (due to symmetric
  layout around DC), so mirror-match inflates the count.
- **§3.4** `[SACK-DUMP]` — first 12 RX snapshots dumped to
  `/tmp/sack_dump_NN.wav`.
- **§3.5** `[SACK-TX-DUMP]` — first 3 RSP TX passband bursts dumped to
  `/tmp/sack_tx_NN.wav`.

Python analysis tools (all in `tools/`):
- `mercury_sack_detector.py` — Python port of `detect_ack_pattern`;
  reproduces Mercury's 5-8/16 result on captured WAVs (proves algorithm
  port is faithful).
- `analyze_sack_wav.py` / `analyze_sack_wav2.py` — spectrum analysis.
- `compare_tx_rx.py` — cross-correlation between TX and RX WAVs.

## §4 Root cause

### §4.1 Buffer sizing is adequate in theory
`receive_sack_pattern` at `arq_common.cc:3619-3633` computes:
```
sack_total_nsymb = ack_pattern_nsymb + bitmap_nsuffix = 16 + 32 = 48
tail_nsymb = sack_total_nsymb + sack_total_nsymb + 16 = 112
tail_samples = 112 × 1168 = 130816 (2726 ms)
```
This is 2.33x the SACK duration (1168 ms) — should have margin.

### §4.2 Other call paths use smaller buffers
Call sites `arq_common.cc:4079` and `:4184` (SACK-before-ACK cross-checks)
call `detect_sack_pattern_from_passband` passing a `tail_samples` sized
for the ACK pattern, NOT the SACK pattern. That produces the 56064 /
65408 / 93440 buffers seen in logs — too small to hold the 48-symbol
SACK burst + timing margin.

### §4.3 Polling timing is early
Multiple SACK polls fire during the RSP SACK TX window, before the full
burst has arrived. Each small-buffer snapshot catches only the current
tail (last 33-73% of burst). The first 16 symbols — the BASE pattern —
are never in frame.

## §5 Proposed fixes (to be tested in order)

> **STATUS (2026-05-08):** §5.1–§5.3 were **superseded** by the v2
> redesign attempt documented in `SACK_REDESIGN_PLAN.md` §3.1
> (Design A: OFDM-framed SACK). The v2 attempt was implemented
> 2026-04-25 and **reverted entirely** — see SACK_REDESIGN_PLAN §9.
> HEAD is at `7076a4b` "Fix A" only, which addresses unrelated frame
> overflow but not the late-snapshot architecture. The §6 [?] open
> questions below are folded into the IONOS-era validation plan
> (Phase 4.1 instrumentation) as cited in
> `IONOS_ERA_VALIDATION_PLAN.md` §14.5.

### §5.1 Option 1 — enlarge small-buffer call paths
Change the ACK cross-check call sites to use a SACK-sized tail_samples
when SACK is enabled. Smallest code change; most conservative. Testable
by re-running the throughput test and checking whether base pattern
appears in the SACK decoder snapshots.

### §5.2 Option 2 — delay the poll until SACK would be complete
Add a "wait for full SACK window" state: after CMD TX completes, wait
for RSP_response_delay + SACK_duration + margin before snapshotting.
Larger code change; timing-sensitive.

### §5.3 Option 3 — accumulate across multiple snapshots
Keep the last N SACK snapshots and run detection on each until one
succeeds. Most complex; only makes sense if §5.1/§5.2 fail.

## §6 Open questions [?]

- [?] Why does `receive_sack_pattern` at :3653 (main SACK path, 130816
  buffer) ALSO often see silent buffers in the logs? Is it polling
  AFTER the SACK already scrolled past the tail? If so, same bug in
  reverse — need both §5.1 AND §5.2.
- [?] Does `nframes` propagate correctly through all SACK call paths?
  ACK cross-check paths use default `nframes=0` → `bitmap_nsuffix=0` →
  smaller reserve_after in detector. May be root cause of 65408 size.
