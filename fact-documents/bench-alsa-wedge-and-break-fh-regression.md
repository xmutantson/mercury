# Bench unreliability (ALSA kill-9 wedge) + the break-fh CFG15 regression

Investigation record, 2026-06-19/20. The "RPi bench is randomly unreliable" saga
resolved to TWO software bugs (one testbed, one modem), not the PHY/decode/channel
causes chased earlier. Every claim cited `file:line`. Corrections to prior reads are
kept visible (CLAUDE.md fact-doc rules).

## §1 Symptom
The CFG15 health gate intermittently delivered 0 bytes (`ofdm_ok=0`), apparently at
random: intermittent, load-sensitive (looked thermal), survived every hardware reset,
invisible in sim, and **binary-independent** (627c370 ≡ a9f8511, bit-identical clean
BER). That profile is the signature of a *race*, not a decode regression.

## §2 ROOT CAUSE A (testbed) — ALSA Fe-Pi capture-substream wedge from `kill -9 mercury`
Mercury had **no SIGTERM/SIGINT handler** (only the internal `shutdown_` atomic flag,
audioio.c:73). The harness/butler reaped it with `pkill -9` (SIGKILL, uncatchable), so
mercury died WITHOUT closing the Fe-Pi (sgtl5000 I2S) capture PCM → the substream
**leaks**: `arecord -l` then shows `Subdevices: 0/1` with NO process holder, and the next
`snd_pcm_open` returns **-16 EBUSY** (ffaudio alsa.c) → `radio_capture_thread exit` at
startup (audioio.c:1347) → `ofdm_ok=0` → false health-fail → TESTBED_BLOCKED. Transient
form = an EBUSY race (next launch loses to the kernel releasing the fd); persistent form
= a kernel-wedged substream that only a power-cycle clears. This single mechanism is the
WHOLE §1 profile.

### §2.1 Fixes (branch `fix/bench-moose-combined`, landed on monitor)
- **FIX-B** (the enabling win, testbed): `wait_capture_free` + `kill_capture_holders` +
  relaunch-3x in the CFG15 health gate (bigblock_p3_hw/run_bb_hw_A.py, run_bench8.py).
  Bench went 0% → reliable connect.
- **FIX-A** (mercury, monitor `a203cd3`): retry `snd_pcm_open` on -16 EBUSY / -11 EAGAIN
  (20×150ms) before giving up — a real deployment robustness hole (a transiently-busy
  sound card should not kill the modem). Single chokepoint in ffaudio alsa.c, covers
  capture + playback.
- **FIX-C** (mercury, monitor `ed1f727`): async-signal-safe SIGTERM/SIGINT handler →
  `shutdown_=true` → the audio threads `snd_pcm_close` cleanly before exit (the wind-down
  via `audioio_deinit` join was already correct; only the handler was missing). main.cc.
- **Harness graceful kills** (outer repo `67a998d`): all `-9` mercury/arecord/`fuser -k`
  reaps → SIGTERM-then-`sleep 3`-then-SIGKILL (ionos_butler `_reap_mercury`, run_bench8
  `recover`, run_bb_hw_A `kill_capture_holders`). SIGKILL is uncatchable, so FIX-C only
  helps if the kill is a SIGTERM — these two MUST ship together.

## §3 ROOT CAUSE B (modem) — the break-fh CFG15-delivery regression
With the bench finally honest (§2), a clean HW A/B exposed a real regression: monitor
627c370 (bench-8) transfers CFG15 health (5000 B, ~10.8 kbps, byte-faithful) but
a9f8511 delivers **0 bytes** (CMD stuck re-sending SET_CONFIG, control-ACK never lands →
`[BREAK] retries exhausted` ×12 → slides CFG15→CFG0). The regressor (isolated by
env-disable, NOT marker-reading which gave a wrong first answer): the **break-fh
forward-health gate** (default-ON since 2026-06-18, commit 34cba89). Its latch
`last_forward_ofdm_decode_frame` (arq_common.cc:9391) is set on ANY decoded OFDM frame
**including SET_CONFIG control frames** — before the frame type is parsed (:9439) — and
the gate has no data-phase scoping (only `role==RESPONDER && CONNECTED`, :9716-9720). So
during the SET_CONFIG handshake (`batch_rx_frame_count==0`, no data batch)
`break_fh_suppress()` (:4747) + `break_kofn_corroborate()` (:4760) wrongly engage and the
handshake never consummates. M6 lossless-requeue and A1/A2 turnaround-rephase were both
disproven by the same env-disable test.

### §3.1 Fix (monitor `963222c`)
Re-scope, do NOT revert: gate both predicates on the DATA phase. `break_fh_suppress()`:
`if(batch_rx_frame_count <= 0) return false;`; `break_kofn_corroborate()`:
`if(batch_rx_frame_count <= 0) return probe_matched;`. `batch_rx_frame_count` (arq.h:2322,
incremented arq_responder.cc:1190) is 0 exactly during the control handshake, >0 only
mid-data-batch — so break-fh stays default-ON and keeps its data-phase phantom-BREAK
suppression benefit (turnaround TIER-1), while the control handshake is never gated.
Test: `--test-break-fh` (fail-before/pass-after). Bench: 4/6 clean byte-faithful CFG15
passes (2 independent EBUSY/weak-acq flakes, both still held CFG15). break-fh is part of
the turnaround design — a real fix with a control-path scoping bug.

## §4 Refuted hypotheses (kept visible — all were chased and disproven this arc)
- ~~"Physical audio-chain attenuation / check the cables"~~ — REFUTED: modem log SNR was
  the normal 14.4 dB; not a channel/cabling problem.
- ~~"HW-only decode-margin / nv-collapse → LDPC non-convergence"~~ — REFUTED: it was the
  §2 ALSA wedge, not the PHY decode margin.
- ~~"−244 ppm clock drift"~~ — MISREAD: `[TX-PUSH-RATE]` is a 10s producer-push
  quantization metric (mean +55.7 ppm, ±1000 swing), not a crystal slip (audioio.c
  self-documents it).
- ~~"Moose CFO dead-zone fix"~~ — a misdiagnosis-driven REGRESSION (the 1× reject ceiling
  over-rejected valid ~−70 Hz reverse-ACK CFOs the known-good 2× clamp decodes); built,
  then REVERTED on HW.
- ~~"Build flags (-mcpu=native / -fno-math-errno)"~~ — REFUTED earlier by bit-identical BER.

## §5 Bench operational notes
- Bug-zapper (Tasmota .67) power-cycles **BOTH** Pis (empirically: simultaneous reboot;
  the earlier "rpi1 separate plug" claim is contradicted). Owner-authorized.
- AUDIO_SETUP levels are already calibrated to max — no software headroom.
- DIAGNOSE A BLOCK by grepping the RSP modem log for `snd_pcm_open: (-16)` /
  `radio_capture_thread exit` and `arecord -l` `Subdevices x/1` BEFORE blaming decode.

## §6 Monitor state after this arc
`ed1f727` (monitor) = a9f8511 + break-fh data-phase fix (963222c) + watchdog (0dfe2d2) +
FIX-A (a203cd3) + FIX-C (ed1f727). Build + full `--test` + `--test-break-fh` green.

## §7 Open items
- [?] FIX-C live-modem exit validation (SIGTERM a running mercury, confirm clean release)
  — was only `--test`-validated + observed clean across the 4/6 bench run.
- Health-gate `bytes=` readout race (final bytes reads 0 even when `done=True/meas=5000`)
  — judge by meas/done/bps/ofdm_ok, not the final bytes field.
- No-progress watchdog for run_inband_ab.py:555 (the ~16.5 min deadline hang on 0-byte).
