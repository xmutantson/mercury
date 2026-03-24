# Turboshift Speed Investigation — Fact Document

**Created**: 2025-03-23
**Updated**: 2026-03-24
**Status**: Phase 2 IMPLEMENTED — SNR-in-MFSK-ACK working on IONOS
**Bug**: Turboshift takes 60-90+ seconds; user requirement: <20 seconds

---

## §1 Problem Statement

Turboshift (bidirectional rate negotiation) currently takes 60-90+ seconds before data transfer begins. Observed through IONOS HF simulator at ~10.5 dB SNR:

- TURBO_FORWARD: ~28s (7 steps × ~4s each, CONFIG_0→3→6→9→12→15→16)
- SWITCH_ROLE: ~5s
- TURBO_REVERSE: ~28s (same linear crawl)
- CONFIG_16 ceiling/retry: ~14s extra (32QAM fails at 10.5 dB)
- **Total: ~75s** before first data byte transferred

User requirement: "turboshift should be faster than 90 seconds" — practically wants <20s.

## §2 Root Cause Analysis

### §2.1 Why SUPERSHIFT Can't Fire During TURBO_FORWARD

SUPERSHIFT requires `measurements.SNR_uplink > -90 dB` (`arq_commander.cc:1025`). During TURBO_FORWARD:
- Commander sends OFDM data frames to responder
- Responder replies with **MFSK ACK patterns** (not OFDM frames)
- MFSK patterns don't carry SNR measurements
- Commander's `measurements.SNR_uplink` stays at **-99.90 dB** (init value)
- SUPERSHIFT condition fails → falls back to `config_ladder_up_n(current, 3, ...)` (linear step-3)

This is the fundamental bottleneck. SNR information exists at the responder (from decoding the OFDM probe frame) but has **no path back to the commander**.

### §2.2 Linear Step-3 Crawl Arithmetic

Config ladder (WB): CONFIG_0→1→2→3→4→5→6→7→8→9→10→11→12→13→14→15→16 (17 configs)
Step-3 jumps: 0→3→6→9→12→15→16 = **7 probes per direction**

Each probe requires:
1. SET_CONFIG sent (~frame TX time)
2. SET_CONFIG ACK received (~propagation + processing)
3. Data/control frame at new config (~frame TX time)
4. ACK pattern received (~propagation)
≈ **4 seconds per probe** (OFDM frame ~1s + turnaround + ACK)

7 probes × 4s = **28s per direction**, 56s for both directions + 5s SWITCH_ROLE = **61s minimum**.

## §3 Implementation — SNR-in-MFSK-ACK (Phase 2)

### §3.1 Design

During turboshift, responder appends 8 MFSK suffix symbols to the ACK pattern, all encoding the same quantized SNR tone. Commander decodes suffix via FFT peak detection + majority vote.

**SNR encoding** (NB M=8): tone 0-7 → SNR = tone×2 - 9 dB (range -9 to +5 dB, 2 dB step)
**SNR encoding** (WB M=16): tone 0-15 → SNR = tone×2 - 5 dB (range -5 to +25 dB, 2 dB step)

**Timing**: NB suffix adds 8 × 24.3ms = ~195ms to 779ms ACK pattern. Total ~974ms.
Only used during turboshift; normal data exchange uses short ACK.

### §3.2 Files Changed

- `mfsk.h/cc`: SNR encoding/decoding, ACK+SNR pattern generation
- `ofdm.h/cc`: `decode_suffix_tones()`, `reserve_after` parameter for `detect_ack_pattern()`
- `telecom_system.h/cc`: `generate_ack_snr_pattern_passband()`, `detect_ack_snr_from_passband()`
- `arq.h`: `turbo_snr_ack_enabled`, `turbo_received_snr`, `turbo_snr_defer_timer`
- `arq_common.cc`: `send_ack_pattern_with_snr()`, modified `receive_ack_pattern()` with defer logic
- `arq_responder.cc`: sends SNR ACK during turboshift SET_CONFIG
- `arq_commander.cc`: uses `turbo_received_snr` for SNR-SUPERSHIFT jumps

### §3.3 Bugs Found and Fixed During Implementation

1. **Buffer too small**: Suffix symbols fell off the end of capture buffer. Fix: `reserve_after` parameter limits coarse search so suffix always fits.
2. **Timing race**: ACK detected before suffix transmitted (~390ms gap). Fix: defer timer in `receive_ack_pattern()` — keep polling up to 500ms for suffix to arrive.
3. **Carrier image ambiguity**: Bug #39's mirror-bin recovery made tone pairs indistinguishable (e.g., tone 0 ↔ tone 7 share bin 252/4). Fix: use primary bin only for suffix decoding — complex baseband has proper image rejection.

### §3.4 Test Results (IONOS, NB mode, 2026-03-24)

**Before** (step-3 crawl):
- CONFIG_0→3→6→9→12 in 33s (TURBO_FORWARD only)
- Final config: CONFIG_12

**After** (SNR-SUPERSHIFT):
- CONFIG_0→3→**8**→10→13 in 33s (SNR-SUPERSHIFT jumps 3→8 and 8→10)
- CONFIG_0→3→**8**→11→14 in 45s (CONFIG_14 reached, previously unreachable in turbo)
- Suffix decode: 8/8 agreement on most probes, robust majority vote on noisy probes

**Improvement**: SUPERSHIFT jumps save 1-2 probe rounds (~8-12s) per direction. CONFIG_13-14 reachable where CONFIG_12 was previous ceiling.

## §4 Open Questions

- [?] WB mode (M=16) not yet tested on IONOS — should work better (more tones, larger margin)
- [?] TURBO_REVERSE still uses linear crawl — Phase 1 (skip reverse) would further halve time
- [?] Real HF (non-IONOS) may have lower SNR → suffix decode less reliable

## §5 Prior Art

(See original research in git history — PACTOR, ARDOP, VARA, SoftRate approaches)
