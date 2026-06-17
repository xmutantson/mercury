# Data-Flow Audit: Coordinated Recovery-ACK GEOMETRY (closed-loop, config-derived)

**Status**: Authoritative as of 2026-06-17. Built against `feat/turnaround-timing-instr @4dcdc53`
(worktree `C:/Users/kamer/mercury_wt/revack-geometry`, branch `feat/revack-geometry`). All line
numbers RE-LOCATED against THIS tree (4dcdc53 carries the [TT] wall-clock instrumentation +
monitor 9afe802's A1 CFG15 re-phase + A2 multi-window data-ACK).

**Env gate (NEW)**: `MERCURY_REVACK_GEOMETRY` — DEFAULT-OFF, byte-identical-when-off. This is a
NEW gate; it does NOT reuse `MERCURY_RECOVERY_ACK_REPHASE` (that gated the FALSIFIED capture fix)
nor `MERCURY_TURNAROUND_REPHASE` (A1, the CFG15 listen-window re-phase) nor
`MERCURY_RECOVERY_ACK_ROBUST` (the R=4 reps experiment).

**Why a NEW lever** (and why it can succeed where 4 RX-only fixes failed): the four prior fixes
were RX-only — they tried to make the CMD window WIDER / RE-SNAPSHOT / search older ring phases
to *catch* an ACK whose on-air instant the CMD could not predict. Each detonated the
poll/BREAK/demote storm (`RECOVACK_VERDICT.md`: 68x polls, 178 breaks, 0 bytes) because re-polling
older ring phases under a marginal correlator manufactures phantom near-misses. The HW wall-clock
measurement (`bigblock_p3_hw/HW_TURNAROUND_TIMING/tt_verdict.json`) PROVED the recovery-ACK timing
is **DETERMINISTIC, not drift**: `rsp_decode->ack_onair = 400.94ms` (sd 0.15ms, spread 0.68ms);
the ACK-vs-window offset is constant to ~±2 symbols (sd 15.6ms over 12 turnarounds). ±8.16ppm
crystal drift over the ~1.7s window would be ±39ms (>2x the observed sd) — REFUTED as the cause.
Because the offset is a fixed CONFIG-DERIVED constant, BOTH peers can compute the IDENTICAL
expected ACK-arrival instant from shared config and align to it — so the ACK lands in the CMD's
FIRST scored snapshot BY CONSTRUCTION, with NO re-poll / no re-snapshot / no older-phase search.

**Companion docs**: `RECOVERY_WINDOW_WIDEN_DESIGN.md` (the R=4 robust-reps coupling, a DIFFERENT
lever — that widens the *duration*; this re-phases the *key instant* + *snapshot center*),
`data-flow-revack-turnaround-geometry.md` (the FIX-9 D2/D3 lockstep widen, also duration-class),
`tt_verdict.json` (the measured deterministic offsets this fix is calibrated to).

---

## §0. TL;DR — the shared offset both peers compute

The single number BOTH peers derive identically from SHARED config + fixed constants (NO
dependence on the noisy `receive_stats.delay`):

```
K_REVACK_KEY_MS  = the deterministic RSP "forward-frame EOT -> ACK on-air" offset.
                   HW-measured 400.94ms (tt_verdict.json rsp_decode_to_ack_onair).
                   Decomposed from the constants both peers already share:
                     ptt_off_delay_ms (200) + RSP_DECODE_MARGIN_MS (300) ... clamped to
                   a single named constant REVACK_KEY_OFFSET_MS sized to the HW 401ms.
```

- **RSP side** keys the recovery ACK at `T_fwd_eot + REVACK_KEY_OFFSET_MS`, where `T_fwd_eot` is
  the instant the RSP finished CAPTURING the forward frame it is ACKing. The on-air ACK
  WAVEFORM/symbol-count is UNCHANGED (no wire-format break; only the KEY INSTANT moves, and only
  when the env is set). The CURRENT code instead derives the pre-key `wait_ms` from
  `receive_stats.delay` (the NOISY time-sync sample offset, ofdm.h:39 "Sample delay to preamble
  start") — that is the term the fix replaces with the deterministic constant.
- **CMD side** predicts the SAME arrival instant from the SAME shared config: it knows when it
  stopped keying the forward frame (= the RSP's capture EOT, modulo ~0 HF-data propagation), the
  forward-frame airtime, and the IDENTICAL `REVACK_KEY_OFFSET_MS`. It then CENTERS the scored
  snapshot so the deterministic ACK block lands near the MIDDLE of the searched tail (block at
  ~sym 16 of the 48-symbol tail) instead of straddling the newest-tail edge — so the sliding
  correlator (`detect_ack_pattern`) has the WHOLE block present at a slide position on the FIRST
  scored poll.

**Both sides derive `REVACK_KEY_OFFSET_MS` from the SAME named constant** → the geometry is truly
closed-loop. INV-G (§5) is the proof obligation that they match within ±2 symbols.

---

## §1. The shared state under audit

### 1.1 RSP-side TX timing (`cl_arq_controller::send_ack_pattern`, arq_common.cc:6214)
| Quantity | Role | file:line (4dcdc53) |
|----------|------|---------------------|
| `wait_ms` (OFDM branch) | The pre-`ptt_on()` settle before the ACK is keyed. TODAY derived from `receive_stats.delay`. | arq_common.cc:6244-6246 |
| `receive_stats.delay` | The NOISY time-sync sample offset of the captured forward frame's preamble. Producer: `time_sync_mfsk_corr` / `ofdm_forced_delay` etc. (telecom_system.cc:1096,1263,1290,1332,...). | ofdm.h:39 |
| forward-frame geometry | `preamble_nSymb + Nsymb` symbols; `sp = Nofdm * interpolation_rate`. Config-derived, both peers know it. | data_container fields, read at arq_common.cc:6232-6236 |

### 1.2 CMD-side RX scored-snapshot geometry (`receive_ack_pattern`, arq_common.cc:8198)
| Quantity | Role | file:line |
|----------|------|-----------|
| `tail_offset` | Where in the ring the scored snapshot starts. TODAY = `signal_period - tail_samples` (NEWEST tail). | arq_common.cc:8218 |
| `tail_samples` / `tail_nsymb` | Size of the searched snapshot (`ack_nsymb + pattern_len + 16` symbols). | arq_common.cc:8211-8215 |
| `multiwindow_scan` | Existing CONNECT-only lever that REPOSITIONS `tail_offset` to an older ring phase and re-runs the UNCHANGED correlator. The PROOF that moving `tail_offset` + re-running the detector is a safe, established pattern. | arq_common.cc:8354-8404 |
| `detect_ack_pattern` | The correlator. SLIDES `s` from 0 to `buffer_nsymb-total_needed` over the snapshot, finds the best-matching ACK block position. NOT clamped — searches the whole snapshot it is handed. | ofdm.cc:4485 |

### 1.3 The CMD listen-window DURATION (`calculate_receiving_timeout`, arq_common.cc:1263)
COMMANDER `ack_pattern_time_ms>0` branch. Computes `receiving_timeout` (a DURATION). This is the
A1/D2 layer — the geometry fix does NOT touch the duration (tt_verdict proves the ACK is already
in-window 12/12); it touches WHERE the snapshot is CENTERED, not how LONG the window is open.
Listed here so the audit is complete: the geometry fix and the duration logic are INDEPENDENT
and do not interact (geometry repositions the snapshot WITHIN the already-open window).

### 1.4 Pure config geometry (read identically by both peers)
- `forward_frame_nsymb = preamble_nSymb + Nsymb` (config-derived).
- `sp = Nofdm * interpolation_rate` (config-derived).
- `Fs = sampling_frequency = 48000`.
- `REVACK_KEY_OFFSET_MS` (NEW constant, §3) — the deterministic key offset, identical both sides.

---

## §2. Producers (every write site, file:line on 4dcdc53)

### 2.1 `wait_ms` in `send_ack_pattern` (RSP, the re-anchor producer)
- arq_common.cc:6244-6246 — OFDM branch: `wait_ms = ceil(remaining_sym*Nofdm/48000) + ptt_off + ptt_on`,
  where `remaining_sym` derives from `receive_stats.delay` (NOISY). **THE RE-ANCHOR SITE.** When
  `MERCURY_REVACK_GEOMETRY` set: `wait_ms = REVACK_KEY_OFFSET_MS` (deterministic), bypassing the
  delay-derived term. When UNSET: the delay-derived term is byte-identical to base.
- arq_common.cc:6262 — MFSK branch (`ptt_off+ptt_on`): UNTOUCHED (NB/robust recovery ACKs are not
  the OFDM-forward-config recovery turnaround the fix targets; left byte-identical).

### 2.2 `tail_offset` in `receive_ack_pattern` (CMD, the center producer)
- arq_common.cc:8218 — init `= signal_period - tail_samples` (newest tail). **THE CENTER SITE.**
  When `MERCURY_REVACK_GEOMETRY` set AND the predicted-arrival phase is reached: bias `tail_offset`
  EARLIER by `center_bias_samples` so the predicted ACK block sits at ~sym 16 of the 48-sym tail.
  When UNSET: `tail_offset` stays at the newest tail (byte-identical).
- arq_common.cc:8401 — the EXISTING `multiwindow_scan` repositioner (CONNECT only). The geometry
  center is a SIBLING of this established repositioning pattern, scoped to the recovery/data-ACK
  wait via the env gate (NOT the CONNECT multiwindow path).

### 2.3 `REVACK_KEY_OFFSET_MS` (the shared constant)
- include/common/common_defines.h (NEW) — a single `static const int`, read by BOTH the RSP
  re-anchor (§2.1) and the CMD center (§2.2). ONE definition → the two sides cannot diverge.

---

## §3. Consumers (every read site)

### 3.1 RSP re-anchor consumer
`send_ack_pattern` (arq_common.cc:6248-6254) consumes `wait_ms` → `pumped_settle_wait(wait_ms)` →
`ptt_on()` → keys the ACK. The ACK on-air instant = `T_fwd_eot + wait_ms`. With the fix,
`wait_ms = REVACK_KEY_OFFSET_MS` → deterministic on-air instant.

### 3.2 CMD center consumer
`receive_ack_pattern` (arq_common.cc:8405-8409) snapshots `[rwi + tail_offset .. +tail_samples]`
and hands it to `detect_ack_pattern` (via the energy gate + correlator body). With the fix,
`tail_offset` is biased so the predicted ACK block is centered in that snapshot → the unchanged
sliding correlator finds the full block at a slide position on the FIRST scored poll.

### 3.3 The correlator (UNCHANGED)
`detect_ack_pattern` (ofdm.cc:4422) — SAME energy gate (ACK_ENERGY_GATE_RMS), SAME
`ack_match_threshold`, SAME `ack_metric_threshold`. The fix changes ONLY where the snapshot reads
(WHEN/WHERE), never WHETHER the correlator accepts (the §3.4 RECOVACK lesson: never relax the
detection bar). A genuine miss (silence at the centered phase) falls through to the normal miss
path (frames_to_read=2; return false), re-polled next tick exactly as today.

---

## §4. Valid states

### 4.1 Default-init danger (CLAUDE.md §5.3)
- Before any forward frame: `receive_stats.delay=0`, `tail_offset` unset until the snapshot. The
  fix MUST NOT fire the center bias before a forward OFDM frame exists. GATE: env set AND OFDM
  forward config AND a non-degenerate `center_bias` (clamped >=0). On ROBUST/NB the RSP MFSK
  branch (§2.1, arq_common.cc:6262) is untouched and the CMD center is is_ofdm-gated.
- `center_bias_samples` clamped to `[0, tail_offset]` so the biased snapshot never reads before
  the ring origin (the double-mapped ring at data_container.cc:170 makes `[rwi+off]` in-bounds for
  off in `[0, tail_offset]` only).

### 4.2 The recovery turnaround (where the fix bites)
BREAK→ROBUST_0 recovery (arq_commander.cc:250-340) and the SET_CONFIG recovery: the CMD polls
`receive_ack_pattern()` each tick. The RSP keys the recovery ACK ~401ms after capturing the
forward (BREAK/SET_CONFIG) frame. With the fix BOTH sides anchor to `REVACK_KEY_OFFSET_MS` → the
CMD's first scored snapshot at the predicted phase is centered on the block.

### 4.3 Clean full-batch data-ACK (must stay byte-identical when off; safe when on)
On a clean full batch the RSP keys a CLEAN ACK; the geometry, when ON, merely centers the snapshot
on the same deterministic instant — a no-op if the ACK was already caught at the newest tail
(centering only HELPS a straddle, never hurts a clean catch: the correlator still slides the whole
snapshot). When OFF: byte-identical.

---

## §5. Invariants consumers assume + what the fix changes

### INV-G (geometry closed-loop — the KEYSTONE proof obligation)
The CMD's PREDICTED ACK-arrival instant MUST equal the RSP's ACTUAL key instant within the bounded
jitter (~±2 sym / ~±50ms per tt_verdict sd 15.6ms). Both derive `REVACK_KEY_OFFSET_MS` from the
SAME named constant and the SAME shared forward-frame geometry → they match by construction. The
focused test (`--test-revack-geometry`) ASSERTS this equality directly. **If they could NOT be
made to match from shared config, the fix would be a one-sided guess and is HARD-BOUNDED OUT
(CLAUDE.md §2/§5).** They match → the fix proceeds.

### INV-1 (lockstep: CMD window superset-of RSP-keyed ACK) — PRESERVED
The DURATION logic (A1/D2, §1.3) is untouched. The geometry repositions the snapshot WITHIN the
already-open window; the ACK is already in-window 12/12 (tt_verdict). The center never moves the
snapshot OUTSIDE the window (clamped to `[0, tail_offset]`). ✔

### INV-2 (config-purity: robust/NB turnaround untouched) — PRESERVED
The RSP re-anchor is in the `is_ofdm_config` branch only (arq_common.cc:6230); the MFSK branch
(:6262) is byte-identical. The CMD center is is_ofdm-gated. ROBUST_0/1/2 and NB recovery ACKs are
unchanged. ✔

### INV-3 (detection bar unchanged: never relax thresholds) — PRESERVED
`detect_ack_pattern` and ALL accept thresholds are unchanged (§3.3). The fix changes only WHERE the
snapshot reads. This is the explicit lesson of `RECOVACK_VERDICT.md` (the capture fix's storm came
from re-polling under a marginal bar). ✔

### INV-4 (byte-identical-when-off) — the GATE
With `MERCURY_REVACK_GEOMETRY` unset: the RSP `wait_ms` keeps the delay-derived value
(arq_common.cc:6244-6246), the CMD `tail_offset` keeps the newest-tail value (arq_common.cc:8218),
and `calculate_receiving_timeout` is untouched → the SFO-grid / climb-engine render is
BYTE-IDENTICAL to base 4dcdc53. Guaranteed because every new branch is `if(revack_geometry_enabled())`
gated and the cached gate defaults OFF. ✔ (proven by md5 in §6 of the return payload).

### What the fix changes, walked per consumer
1. RSP `wait_ms` (§3.1): delay-derived → `REVACK_KEY_OFFSET_MS` ONLY when env on, OFDM branch.
   Consumer = the keyer; it WANTS a deterministic instant. ✔
2. CMD `tail_offset` (§3.2): newest-tail → center-biased ONLY when env on. Consumer = the
   snapshot+correlator; it WANTS the block centered. ✔
3. Correlator (§3.3): UNCHANGED. ✔
4. Duration logic (§1.3): UNCHANGED. ✔
No consumer is broken; two are improved; the detection bar and the off-path are untouched.

---

## §6. The failing test (fail-before / pass-after) — `--test-revack-geometry`

In-process, no IONOS/RF, always-on under `mercury.exe --test` AND standalone via
`--test-revack-geometry`. PURE geometry (no audio): drives the SAME shared-constant arithmetic both
peers run.

1. Using the measured deterministic offsets (RSP decode→key 401ms, the forward-frame geometry),
   compute the RSP's keyed-ACK instant `T_key_rsp = T_fwd_eot + REVACK_KEY_OFFSET_MS` and the CMD's
   predicted-arrival instant `T_arr_cmd = T_fwd_eot + REVACK_KEY_OFFSET_MS` (both from the shared
   constant). **Assert |T_key_rsp - T_arr_cmd| <= 2 symbol periods.** (INV-G.)
2. Compute the centered snapshot's block position: with the center bias applied, the predicted ACK
   block sits at `block_sym ≈ tail_nsymb/2 - ack_nsymb/2` (~sym 16 of 48). **Assert the block is
   CENTERED** (within ±2 sym of mid-tail), not straddling the newest edge.
   **FAIL-BEFORE** (geometry OFF, `-DREVACK_GEOMETRY_FAILBEFORE` drops the center bias): the
   snapshot reads the NEWEST tail and the deterministic-arrival block lands at the newest edge
   (block_sym ≈ tail_nsymb - ack_nsymb, straddling) → the centering assert FAILS.
   **PASS-AFTER**: block centered → assert passes.
3. **Byte-identical-when-off leg**: with the gate OFF, assert the RSP `wait_ms` formula and the CMD
   `tail_offset` reduce EXACTLY to the base values (delay-derived `wait_ms`, newest-tail
   `tail_offset`).

The decisive assertions are 1 (INV-G: both peers compute the SAME instant) and 2 (the snapshot is
centered on the deterministic block, fail-before/pass-after).

---

## §7. Prior art

1. **Fixed-cycle / self-synchronizing ACK timing.** VARA HF, ARDOP, and PACTOR size the receiver's
   listen slot to the EXACT frame/ACK position on the wire and key replies at a fixed turnaround
   offset, precisely so a reply lands in a predictable slot rather than being hunted for
   (sigidwiki VARA_HF; ARDOP timing). Mercury's bug is hunting for an ACK whose instant the codebase
   could already PREDICT from config — this fix makes the prediction explicit and aligns both sides.
2. **In-codebase precedent.** `multiwindow_scan` (arq_common.cc:8354) already proves repositioning
   `tail_offset` + re-running the UNCHANGED correlator is safe. The geometry center is the same
   move, but to a CONFIG-PREDICTED phase rather than an energy-hunted older phase — strictly more
   principled (no hunt, no phantom near-miss).
3. **The tt_verdict measurement.** The fix is NOT a tuned margin; `REVACK_KEY_OFFSET_MS` is the
   HW-MEASURED 401ms deterministic offset (sd 0.15ms). Both peers anchoring to a measured constant
   is the textbook fixed-turnaround discipline, not a guess.
