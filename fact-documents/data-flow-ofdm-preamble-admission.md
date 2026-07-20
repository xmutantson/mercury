# Data-Flow: OFDM Preamble Admission (sub-peak / reverse-MFSK false-lock gate)

> **CURRENT-TRUNK PORT NOTE:** the harvested implementation retains only the
> default-off preamble `bb/pb >= 0.74` discriminator. The experiment's secondary
> `preamble_detect_threshold = 0.65` wall and `SUBPEAK_METRIC_FLOOR_WB` constant
> are intentionally not ported: later in-band captures contain valid
> CFO-impaired wideband preambles with coarse metrics 0.426-0.609. All references
> below to the 0.65 secondary floor describe the historical experiment, not the
> current harvest.

Branch: `feat/ofdm-subpeak-gate` (off monitor `9afe802`).
Env gate: `MERCURY_SUBPEAK_METRIC_GATE` (default-OFF, byte-identical when unset).
Status: HELD (no merge/push/HW). Built + sim/loopback validated only.

This document owns the producer/consumer list for the **OFDM forward-preamble
admission decision** — the point where a Schmidl-Cox candidate is accepted as a
real forward CFG15 OFDM data frame and handed to Moose CFO → channel-est →
pilot-residual-var → LDPC. It is paired with the loopback regression
`--test-subpeak-gate` (telecom_system / climb-engine).

---

## §1 Root cause (HW-verified, firsthand from `bigblock_p3_hw/HW_TURNAROUND_TIMING/tt_cmd.log`)

During the half-duplex CFG15 climb-back the forward-OFDM Schmidl-Cox detector
FALSE-LOCKS onto the **reverse-direction MFSK control/BREAK burst**. The WB OFDM
admission floor is only 0.15 (`telecom_system.cc:1769-1771`, `narrowband_enabled
? 0.30 : 0.15` in the default `!sack_cross_check_mode` path), so the detector
admits a sub-peak at `coarse_metric≈0.25-0.555` that is NOT a forward OFDM frame.

### §1.1 The three RX populations (extracted from tt_cmd.log, n classified by outcome)

| Population | metric | **pream bb/pb** | data bb/pb | pilot-var |
|---|---|---|---|---|
| **REAL fwd CFG15 (OK + real-preamble incomplete)** n=43 | **0.999-1.000** | **0.565-0.584** | 0.152-0.154 | 0.036 |
| **CATASTROPHIC false-lock (SKIP-VAR var>10)** n=70 | **0.250-0.555** | **0.904-0.907** | 0.226-0.227 | **47-86** |
| weak-noise sub-peak (SKIP-VAR var≈2) n=75 | 0.151-0.662 | 0.364-0.469 | 0.089-0.118 | 1.7-2.8 |

`pream`/`data` pb=passband (`((double*)data)[]`), bb=baseband-decimated
(`baseband_data_decimated[]`), computed at the ENERGY-DIAG site
`telecom_system.cc:2109-2134`.

### §1.2 The decisive discriminator is the PREAMBLE bb/pb ratio, NOT the data ratio

- **pream bb/pb: REAL ≤ 0.584, false-lock ≥ 0.904 → GAP 0.32** (clean).
- metric: REAL ≥ 0.999, false-lock ≤ 0.555 → GAP 0.44 (clean, but see §1.3).
- **data bb/pb does NOT separate**: REAL ≈ 0.152, false-lock ≈ 0.227 — only
  0.075 apart, and BOTH far below 1.0. A data-ratio gate at ~0.5 (as a naive
  reading of "false≈0.23 vs real≈1.0" would suggest) would **reject every real
  forward frame** (0.15 < 0.5) = the catastrophic acquisition regression
  the experiment's hard-bound review warns against. **Corrected**: the "real≈1.0" model
  is wrong for this codebase's ENERGY-DIAG (pb and bb are different scales /
  domains); the load-bearing fact is the *preamble* ratio.

**Physics**: the reverse MFSK burst is a *narrowband single-tone* — nearly all
its passband energy survives the anti-alias decimation filter, so pream bb/pb
≈ 0.9. A real WB OFDM preamble spreads energy across the full 2.3 kHz band; the
decimation filter + scaling drop pream bb/pb to ≈0.57. This is the
in-band-vs-passband discriminator the design called for, read at the preamble
(where the SNR is highest) instead of the data region.

### §1.3 Why a metric floor ALONE is unsafe (the §7.13.31 hazard)

`sack_cross_check_mode` once raised the WB floor to 0.65, but it was DELIBERATELY
removed from the v2-OFDM dispatch path (`arq_commander.cc:2965-2977`,
§7.13.31): the 0.65 floor rejected legitimate low-metric CONTROL frames
(SET_LINK_PARAMS) and produced "CMD sending SET_LINK_PARAMS 10x with 0 ACKs".
So the default active floor is 0.15. A blanket raise to 0.65 would re-introduce
that control-frame regression. Therefore the metric floor is **secondary /
belt-and-suspenders, gated narrowly** (forward OFDM DATA admission only,
behind the env), and the **energy-ratio gate is primary**.

### §1.4 Downstream detonation chain (one false-lock → three symptoms)

`tt_cmd.log:33927-33946` + `:35573-35577` (firsthand):
1. `[OFDM-SYNC] WB INIT ... metric=0.5016` admitted (floor 0.15).
2. `[ENERGY-DIAG] pream pb=1.36e-1 bb=1.23e-1` (ratio 0.905) ... `data pb=5.42e-1 bb=1.23e-1`.
3. `[WB-FREQ] Moose=-69.9998 Hz` — spurious CFO from garbage (Moose path is
   CORRECT for real frames; do not touch it).
4. `var=49.3538 too high (>0.50)` ×3 → `3 consecutive SKIP-VAR — abort`
   (`telecom_system.cc:2903-2921`).
5. The MFSK tones alias onto the break-tone bins, and the false-lock's
   `coarse_metric=0.25 < 0.30` passes the OFDM-alias gate
   (`arq_common.cc:9700`) → `[BREAK-PROBE] matched=11/16` →
   `[BREAK] Emergency pattern detected!` → drop to ROBUST_0
   (`arq_common.cc:9725-9737`). This is what `feat/break-fh-ofdm-presence`
   treated at the symptom; preventing the false-lock at admission **subsumes**
   that veto (no false-lock → no SKIP-VAR churn → no aliased BREAK probe).

### §1.5 Trigger

A *real* preamble lands BODY-UNCAPTURED (frame-incomplete, metric≈1.0):
the `incomplete` population has metric 0.999-1.000 but the body has not arrived
(`telecom_system.cc:1732-1748` ofdm_defer_overflow / `arq_common.cc:9604-9641`
INCOMPLETE recapture). The deferral already exists; the fix ensures the
sub-peak is rejected so the next receive re-captures the *real* preamble rather
than latching the earlier 0.25 sub-peak.

---

## §2 The fix (all gated `MERCURY_SUBPEAK_METRIC_GATE`, default-OFF byte-identical)

### §2.1 PRIMARY — preamble in-band-vs-passband energy-ratio gate
Site: the existing ENERGY-DIAG block, `telecom_system.cc` ~2135 (immediately
after the diag print, before the data-energy completeness gate). The values
`pb_pream`, `bb_pream` are ALREADY computed in scope. When the gate is enabled
AND `M != MOD_MFSK`, REJECT (set `energy_ok=false`, `frame_data_missing=true`)
when `bb_pream / pb_pream >= SUBPEAK_PREAM_RATIO_REJECT` (0.74 — safely between
REAL ≤0.584 and false-lock ≥0.904; ~0.16 margin below false, ~0.16 above real).
This is metric-independent and runs BEFORE Moose/var/LDPC, so the spurious CFO,
the SKIP-VAR churn, and the aliased BREAK are ALL prevented.

### §2.2 SECONDARY — coarse_metric admission floor (belt-and-suspenders)
Site: `telecom_system.cc:1769-1771`. When the env gate is on, raise the
**WB** floor for forward OFDM data admission to `SUBPEAK_METRIC_FLOOR` (0.65,
safely between false-lock max 0.555 and REAL min 0.999). NB floor unchanged
(narrowband control path; §1.3 control-frame hazard). The energy-ratio gate is
primary; this is a second wall in case a future channel shifts the ratio.

### §2.3 TRIGGER handling — deferral carry already covers it
The body-uncaptured real preamble is already deferred (§1.5). No new code is
required for re-capture; the energy-ratio gate ensures the *earlier* sub-peak is
not admitted in the interim. (No change to the overflow path → no risk to the
INCOMPLETE-recapture invariant.)

---

## §3 Cross-layer audit (five questions)

**State changed**: the boolean OFDM forward-preamble *admission* decision
(`energy_ok`) and its downstream `receive_stats.coarse_metric` /
`frame_data_missing` / `frame_skip_var_aborted`.

1. **Producers of the admission decision**:
   - `telecom_system.cc:1709` coarse Schmidl-Cox `WB INIT` sets
     `receive_stats.coarse_metric` + `delay`.
   - `telecom_system.cc:1769-1771` `preamble_detect_threshold` (admission floor).
   - `telecom_system.cc:2065-2072` silence-skip recovery re-admits.
   - `telecom_system.cc:2082-2148` data-energy completeness gate (sets
     `energy_ok=false` + `frame_data_missing` when data silent). **My primary
     gate is added here, same mechanism, additive.**
   - `telecom_system.cc:2835-2843` SUBPEAK-REJECT (metric≥0.97 AND mean_H<0.5)
     — does NOT catch this false-lock (metric 0.25-0.555 « 0.97).

2. **Consumers of the admission decision**:
   - Moose CFO `telecom_system.cc` WB-FREQ (`ofdm.cc:~617`) — reads admitted
     frame; spurious -22/-70 Hz on a false-lock. Prevented by rejecting earlier.
   - `telecom_system.cc:2903` SKIP-VAR gate (var>ceiling). The false-lock's
     var=47-86 is what aborts. Prevented.
   - `arq_common.cc:9697-9737` BREAK probe — gated on `coarse_metric<0.30`.
     A rejected false-lock never sets the sub-peak coarse_metric into the path
     that the BREAK probe reads on the *failed-decode* branch, AND the SKIP-VAR
     abort that produced the failed-decode no longer fires. Prevented.
   - `arq_common.cc:9604-9641` INCOMPLETE recapture — reads
     `frame_overflow_symbols`. **Unchanged by the fix** (I do not touch the
     overflow path); the real body-uncaptured preamble still defers + recaptures.

3. **Valid states / default-init**: before any producer, `energy_ok` starts
   true only after a coarse PASS; `frame_data_missing=false`,
   `coarse_metric` from the coarse search. When the env is UNSET, none of the
   new branches execute → admission decision is byte-identical.

4. **Invariants consumers assume**:
   - INV-A (LDPC/Moose): an admitted frame is a forward OFDM frame with in-band
     data energy. The false-lock VIOLATES this (out-of-band MFSK). The primary
     gate RESTORES INV-A by rejecting out-of-band pream-ratio candidates.
   - INV-B (INCOMPLETE recapture): a deferred real preamble is re-captured on a
     later receive. **Preserved** — the fix does not alter overflow/defer; it
     only rejects the competing sub-peak so the recapture is not pre-empted.
   - INV-C (CONTROL frames): low-metric (<0.65) legitimate CONTROL frames must
     still be admitted (§7.13.31). **Preserved** — the metric floor is WB-only,
     behind the env, and the *energy-ratio* gate keys on the pream ratio
     (≈0.57 for real WB control too, NOT the 0.9 MFSK-burst signature), so a
     real low-metric WB control frame (in-band OFDM) passes the energy gate.
     [?] CONTROL frames in this trace are OFDM (post §7.13.30 pure-OFDM), so
     they share the ≈0.57 pream ratio of data; verified in §4 below.

5. **What the fix changes**: it adds a rejection on `bb_pream/pb_pream≥0.74`
   (env-on). Walked each consumer: Moose/var/BREAK all benefit (false-lock
   removed); INCOMPLETE-recapture untouched; CONTROL admission preserved
   because real WB OFDM (data OR control) has pream ratio ≈0.57 « 0.74.

---

## §4 Validation (MEASURED — `--test-subpeak-gate`, build off 9afe802)

`--test-subpeak-gate` drives the production `passband_to_baseband_decimated`
path (`cl_ofdm`) on a real CFG15 OFDM preamble vs a narrowband MFSK-burst proxy
and computes the pream bb/pb ratio exactly as the ENERGY-DIAG site.

- **(1) REAL WB OFDM preamble**: synthesized CFG15 preamble → pream bb/pb =
  **0.6498** < 0.74 → **ADMITTED**. (Clean-harness 0.65 sits slightly above the
  HW 0.57 because the HW path adds channel/AGC out-of-band leakage that decimates
  the ratio LOWER — i.e. the HW is even safer than the harness; the gate is
  conservative.)
- **(2) narrowband MFSK-burst proxy**: pream bb/pb = **0.9063** ≥ 0.74 →
  **REJECTED**. Matches the HW false-lock 0.904-0.907 within 0.0007.
- **(3) NO-REGRESSION amplitude sweep** (the ship-blocker): the SAME real
  preamble scaled −6 … −40 dB → worst ratio **0.6498** (amplitude-invariant:
  bb and pb scale together) → **ALL ADMIT**. A real low-SNR preamble is never
  rejected.
- **(4) HW-population decision**: REAL {0.5649,0.5763,0.5843} all admit;
  false-lock {0.9042,0.9059,0.9065} all reject → **SEPARATES_OK**.
- **(5) FAIL-BEFORE/PASS-AFTER at the production gate-site condition**:
  env-OFF → false-lock `admitted (default-off-bug-exposed)`, real `admitted`;
  env-ON → false-lock `REJECTED (fix)`, real `admitted`. The real preamble is
  admitted in BOTH states (no-regression invariant).
- **`--test-subpeak-gate`**: ALL PASS (0 failures) both env states.
- **`--test`**: green (exit 0). **`--test-climb-engine`**: ALL PASS both states.
- **byte-identical-OFF**: 27-cell coded SFO-GRID render
  (`-m PLOT_PASSBAND -s 16`, `MERCURY_SFO_GRID_CODED=1 NSYMB=600`), env unset,
  version build-stamp line excluded:
  branch md5 = **`ba71664f7f1935969001ed54af182664`** ==
  freshly-built 9afe802 anchor md5 = `ba71664f7f1935969001ed54af182664`.
  (The only raw-output diff is the `9afe802` vs `9afe802-dirty` build stamp,
  an uncommitted-tree artifact that clears on commit — not a behavioral diff.)

## §5 SUBSUMES the break-fh veto
`feat/break-fh-ofdm-presence` treated the SYMPTOM (the aliased emergency-BREAK,
§1.4 step 5) by suppressing the probe when a forward OFDM frame recently
decoded. This acquisition gate is the ROOT: a rejected false-lock never sets the
sub-peak `coarse_metric`, never runs Moose, never produces the SKIP-VAR abort
that drives the failed-decode → BREAK-probe path, and never aliases its MFSK
tones onto the break bins. With the false-lock prevented at admission, the
false BREAK trip cannot fire — so the break-fh veto becomes unnecessary. That
branch is untouched.
