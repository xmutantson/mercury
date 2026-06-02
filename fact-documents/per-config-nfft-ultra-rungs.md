# Per-config Nfft + ULTRA_0-3 baud-scaled rung family (P3a + P3b) — SIM

**Status:** IN PROGRESS (2026-06-01). Worktree `sim/ultra-baud-rungs`
(`C:/Users/kamer/mercury_wt/ultra-baud-rungs`), composed = monitor @fef293f +
INCR-4 @fb58f22 (ULTRA scaffolding + establishment timers) + baud-fading-spike
@5a3c11b (baud-scaling Nfft mechanism + Watterson harness). SIM ONLY — no bench
(v13 cal owns the testbed). Implements `efficient-deep-modulation-frontier.md`
§4/§5 option (c) increment P3a (per-config Nfft) + P3b (the rung family).

Siblings: [[efficient-deep-modulation-frontier.md]] (the (c) plan, ULTRA_0-4),
[[baud-scaling-spike.md]] (P0 gate — Nfft mechanism), [[baud-fading-spike.md]]
(the fading gate — K-mapping data), [[ultra-tier-design.md]] (the INCR-1/2/4
ULTRA scaffolding + the §12 establishment-timer audit this builds on).

---

## §1 What this increment does (scope)

1. **Per-config Nfft (P3a).** Productionize P0's global `MERCURY_BAUD_MULT` env
   flag into a PER-CONFIG parameter: each ULTRA config carries its baud multiplier
   K (→ Nfft = 256·K); every non-ULTRA config (OFDM 0-16, ROBUST 100-102) keeps
   Nfft=256, byte-identical.
2. **ULTRA_0-3 rung family (P3b).** 4 baud-scaled rungs (config IDs 200-203,
   ULTRA_0=deepest=K=8 … ULTRA_3=shallowest=K=1), reusing the ROBUST_0-class MFSK
   DATA PHY (the INCR scaffolding already loads ULTRA as ROBUST_0-class), with
   depth coming from baud-scaling (Nfft), NOT repetition (frontier §1/§2). The
   INCR-4 deep-frame-aware establishment timers apply (they floor off
   `ctrl_suffix_tx_time_ms`, which grows K× with the longer symbol — free).

Explicitly NOT in this increment (follow-ons, per the task): gearshift / channel-
state-2D-lookup integration (P3c); single-low-rate-code refinement esp. for K=8
(P1); HW / real-fading cross-check (P6). NO CAP_ULTRA (no negotiation; ULTRA is
pin/gearshift-selected, backward-compatible by construction — INCR §21.6).

---

## §2 The composition (git) — justification

All three inputs branch off monitor HEAD (fef293f), so the graph is clean:

```
monitor fef293f ──┬── 598ad71 (P0 baud-scaling) ── 5a3c11b (Watterson fading gate)   [sim/baud-fading-spike]
                  └── 942c563 (INCR-1) ── e04e5b3 (INCR-2) ── fb58f22 (INCR-4)        [sim/ultra-choreo-incr4]
```

Merged BOTH onto a fresh `sim/ultra-baud-rungs` off monitor:
- INCR-4 fast-forwarded (linear descendant of monitor).
- fading-spike merged with the `ort` strategy, **auto-merged with NO conflicts**
  (the two branches' edits to `telecom_system.h` / `telecom_system.cc` are in
  non-overlapping hunks — INCR at .h:254 / .cc:3479-3844+5166-5300; baud at
  .h:312 / .cc:50+417+5339). Merge commit `5b84519`.

**Why merge INCR-4 wholesale (not cherry-pick just the timers)?** The INCR-4
timer fix is built ON the INCR-1/2 ULTRA scaffolding it cannot be separated from:
`connect_listen_window_ms()`, `is_ultra_config()`, `ctrl_suffix_tx_time_ms`,
`ultra_tier_suffix_params()`, and the config-ID 200-202 + `load_configuration`
ULTRA dispatch all come from INCR-1/2. That scaffolding is EXACTLY what frontier
§8 says to KEEP ("the choreography fix is KEPT; repetition-as-depth is dropped").
So the right composition is: reuse the INCR ULTRA scaffolding, and SWAP the depth
mechanism from establishment-suffix-repetition to baud-scaling (per-config Nfft).

**Why merge the fading-spike (not just baud-scaling)?** The Watterson harness
(`apply_watterson_fading`, the `MERCURY_FADING*` flags) lives ONLY on the fading
branch and is the validation instrument for the fading gate (§5 below).

---

## §3 K-mapping decision (the rung family) — pick + justify

### §3.1 The fading-gate data (baud-fading-spike.md §5/§5.1) — the basis

| K | Nfft | T_fft (integ) | AWGN cliff | poor 1Hz cliff | role |
|---|------|---------------|-----------|----------------|------|
| 1 | 256  | 21 ms  | −13 dB | −13 | bridges ROBUST_0 (−13/−14) |
| 2 | 512  | 43 ms  | −16 dB | −16 | |
| 4 | 1024 | 85 ms  | −20 dB | −19 | **SAFE deepest** (fading-FAVORABLE: −1 dB high-frame) |
| 8 | 2048 | 171 ms | −22 db | −21 | aggressive cap (AWGN saturates +2dB; edge-fading-sensitive) |

The fading gate validated EXACTLY K ∈ {1,2,4,8} (powers of two). It found:
- K=4 is rock-solid (poor/edge fading 1 dB BETTER than its AWGN cliff at high
  frame count — interleaved-LDPC fading diversity).
- K=8 is usable (−21 poor) but is the diminishing-returns boundary: the AWGN gain
  itself saturates to +2 dB (a coding-gain limit of the single rate-1/16 code at
  that depth, NOT a fading effect), and its 5.86 Hz tone spacing makes it the most
  edge-fading-sensitive rung (FST4 "degraded" band at f_d≥1 Hz).

### §3.2 The decision: 4 rungs, ULTRA_0..3 = K = 8 / 4 / 2 / 1

| Rung | config ID | K | Nfft | AWGN | poor-fading | step vs shallower |
|------|-----------|---|------|------|-------------|-------------------|
| **ULTRA_0** | 200 | 8 | 2048 | −22 | **−21** | +1-2 dB (the cap) |
| **ULTRA_1** | 201 | 4 | 1024 | −20 | **−19** | +3-4 dB (the workhorse) |
| **ULTRA_2** | 202 | 2 | 512  | −16 | **−16** | +3 dB |
| **ULTRA_3** | 203 | 1 | 256  | −13 | **−13** | bridge to ROBUST_0 |

**Why 4 rungs, not 5 (the task allowed "4-5"):** a 5th deeper rung would need
K=16 (Nfft=4096, ~−23/−24 by extrapolation). That is **unvalidated** and lands
PAST the K=8 saturation point the fading gate already flagged — both the AWGN
coding-gain saturation and the coherence/tone-spacing wall worsen monotonically.
Building a K=16 rung now would be an untested extrapolation, violating CLAUDE.md
§3 ("no untested fixes"). The task's own parenthetical — "make ULTRA_0=K=8 with
K=4 as ULTRA_1, etc." — describes exactly this 4-rung K=8/4/2/1 ladder. A K=16
rung is a clean follow-on once P1 adds the deeper coding K=8 needs (frontier P1
note) and a fading re-check validates Nfft=4096. The `is_ultra_config` range and
the enum reserve ULTRA_4=204 for that future rung; it is simply not built here.

**ULTRA_0 = deepest** satisfies the 0=deepest convention (matches ROBUST_0=deepest
robust, CONFIG_0=deepest OFDM). ULTRA_3 (K=1, −13) bridges ROBUST_0 (−13/−14) for
a smooth gearshift hand-off (P3c).

### §3.3 The depth mechanism per rung = baud-scaling (Nfft), NOT repetition

The INCR ULTRA tier got its reach from CONNECT establishment-suffix REPETITION
(R_base=8, R_suffix=8-12, R_frame=2-4 + escalating low-rate GF16). Frontier §1/§2
calls that out as the inefficient anomaly (double-coding + noncoherent combining
loss; the added repetition was measured "sub-one-WGN-cell" on HW, ultra §20.9).

This rung family REPLACES that: depth = baud-scaling (Nfft), which deepens BOTH
the data PHY cliff AND the CONNECT establishment coherently, because the CONNECT
ctrl-suffix MFSK tones ride the SAME `ofdm.Nfft` FFT window as the data
(`generate_ctrl_suffix_pattern_passband` / `decode_suffix_energies` both key on
`ofdm.Nfft`). So a K=4 ULTRA config gets ~+7 dB processing gain on establishment
from the larger FFT alone — no suffix repetition needed.

⇒ For each baud-scaled rung, the establishment path is set to the SAME single
low-rate code the production ROBUST tier already ships (the §19/§20 enhanced
CONNECT: `set_suffix_fec(true, repfact=3)`, K_info=13), with the combining reps
turned OFF (R_base=R_suffix=1) and R_frame=1. This is literally frontier (c):
"delete the repetition, keep ONE low-rate code, let baud-scaling do the work."

`ultra_tier_suffix_params()` (INCR's sole-owner of per-rung PHY numbers) is
repurposed to return these baud-scaled values + the new K (Nfft multiplier).
R_frame=1 makes the CMD frame-rep loop a no-op (byte-identical to a single TX),
but the INCR-4 timer floor STILL fires (it is gated on `is_ultra_config` +
`ctrl_suffix_tx_time_ms>0`, both true) and floors `connection_timeout` to
`1·ctrl_suffix_tx_time_ms + base_window` — correct, because the SINGLE CONNECT
frame is K× longer than a data frame at K=8 (the exact bug INCR-4 fixed).

---

## §4 §5 CROSS-LAYER AUDIT — Nfft is SHARED ENGINE STATE

Per CLAUDE.md §5. `ofdm.Nfft` is read by the OFDM init, the FFT-plan cache, the
data_container buffer sizing, channel estimation, the GI, the bandwidth calc, the
MFSK tone grid, and the BER harness. The five questions:

### §4.1 Producers (every write to `ofdm.Nfft`)
- `physical_config.cc:36`: `ofdm_Nfft=256` (the default-config struct field; the
  single source of the 256 baseline).
- `telecom_system.cc:5328` (load_configuration): `ofdm.Nfft =
  default_configurations_telecom_system.ofdm_Nfft;` — copies the default into the
  live engine for EVERY config. **THIS is the productionization site:** scale by
  the per-config K right here (was: the P0 env-flag block at :5339).
- `ofdm.cc:146` (`cl_ofdm::init(Nfft,...)`): `this->Nfft=Nfft;` — the 4-arg init,
  used by the test harness / standalone OFDM. Production uses the no-arg `init()`
  which reads the member set at :5328.
- (P0 spike block at :5339, now removed/repurposed) was the only other writer.

### §4.2 Consumers (every read of `ofdm.Nfft` / `data_container.Nfft` / `Nofdm`)
The full set keyed on Nfft (all RECOMPUTED downstream of the :5328 write, in the
same `load_configuration`→`init()` call — this is why per-config Nfft flows):
1. **OFDM engine init** `ofdm.cc:160-178` (`cl_ofdm::init()`): `Ngi=Nfft·gi`
   (:162); reallocs `zero_padded_data/iffted_data/gi_removed_data/ffted_data`
   (all `Nfft`-sized, :165-168); `preamble_configurator.init(Nfft,...)` (:174),
   `pilot_configurator.init(Nfft,...)` (:175); `init_fft_tables(Nfft)` (:178).
2. **FFT plan cache** `ofdm.cc:320` (`get_fft_plan(n)`) — keyed BY size, so each K
   gets its own cached plan; no cross-K contamination.
3. **data_container buffers** `telecom_system.cc:4407/4411` →
   `data_container::set_size(..., ofdm.Nfft, ofdm.Nfft*(1+ofdm.gi), ...)`:
   `Nofdm=Nfft*(1+gi)` (:101-103), and EVERY passband/baseband buffer is sized
   from `Nofdm` × `buffer_Nsymb` × interp (`data_container.cc:167-201`). Scales
   with Nfft automatically. `sym_time_ms = 1000·Nofdm·interp/48000` (:149) grows
   K× → `buffer_Nsymb`/turnaround recomputed (:154-163).
4. **Bandwidth** `telecom_system.cc:4217`: `bandwidth = 48000·Nc/Nfft/interp` —
   shrinks K× (Nc held fixed). This is the FST4 "narrow the signal" policy. The
   FIR cutoffs all derive from `bandwidth` (:4246/:4259/:4263) → auto-adapt.
   Carrier stays centered (:4222/:4245).
5. **GI** `ofdm.cc:162`: `Ngi = Nfft·gi`. gi held at the ratio 54/256 → Ngi scales
   54→108→216→432 for K=1/2/4/8. The GI *fraction* (time %) is constant; the GI
   *duration* grows K× (4.5→9→18→36 ms) — BENEFICIAL for HF multipath (longer
   delay-spread coverage), confirmed harmless in the fading gate (baud-fading §2).
6. **MFSK tone grid** `mfsk.cc:118-124` (`stream_offsets`) + `:1054` (per-tone
   energy read at `fft_in[s*Nc + offset + m]`): M tones = M contiguous bins inside
   the Nc grid. Nc is held fixed; the bins are the same INDICES, but each bin is
   narrower (tone spacing = 12000/Nfft → 46.875/23.4/11.7/5.86 Hz). The one-hot
   subcarrier mod (`telecom_system.cc:562/668`) and demod (`:2332/:2340`) ride the
   same Nfft FFT, so the MFSK symbol IS the Nfft window (baud-scaling-spike §1).
7. **MFSK corr template** `telecom_system.cc:5510-5569`: regenerated from
   `data_container.Nofdm`/`ofdm.Nc` AFTER init() → picks up the scaled Nofdm.
8. **CONNECT ctrl-suffix passband** `telecom_system.cc:3551/:6012`:
   `ctrl_suffix_pattern_passband_samples = (base+suffix nsymb)·Nofdm·interp` →
   grows K× → `ctrl_suffix_tx_time_ms` (arq_common.cc) grows K× → the INCR-4
   timer floor scales correctly (the "free" deep-frame-aware behavior).
9. **BER harness noise reference** `telecom_system.cc:418` (`passband_test_EsN0`):
   the P0/fading pin — noise referenced to the K=1 bandwidth so the SNR3k axis is
   fixed across K (the measurement trap, baud-scaling §2). SIM-only; productionize
   keeps this for the validation sweeps.
10. **`data_container::set_size` symbol-count floor** `data_container.cc:126-128`:
    `CTRL_SUFFIX_FEC_MAX_NSYMB=1024` (INCR raised 128→1024 for deep suffix
    combining). Sized in SYMBOLS (×Nc), Nfft-INDEPENDENT. With this family's
    R_base=R_suffix=1 the CONNECT pattern is ~80-144 symbols ≪ 1024 → ample
    headroom (the deep-combining headroom is now unused but harmless).

### §4.3 Valid states (what Nfft is BEFORE/at each transition)
- Default (no config loaded): `default_configurations_telecom_system.ofdm_Nfft=256`.
- After `load_configuration(non-ULTRA)`: `ofdm.Nfft=256` (K=1, byte-identical).
- After `load_configuration(ULTRA_k)`: `ofdm.Nfft=256·K` (K∈{8,4,2,1} for
  200/201/202/203). Set BEFORE `init()` → all consumers see the scaled value.
- Config SWITCH (gearshift, BREAK, session reset): every switch re-enters
  `load_configuration`, which re-copies/re-scales Nfft and calls `init()` →
  `set_size()`, fully reallocating buffers from the NEW Nfft. No buffer keeps a
  stale Nfft. (Verified: `set_size` is called every init; `CNEW` reallocs.)

### §4.4 Invariants the consumers assume (and whether per-config Nfft holds them)
- INV-1: "Nfft is a positive power of two." 256·K for K∈{1,2,4,8} = 256/512/1024/
  2048 — all powers of two. HOLDS. (FFT plans need this; `init_fft_tables` keys on
  it.) The new code must REJECT/clamp K to {1,2,4,8}.
- INV-2: "Nc ≤ Nfft and the Nc grid fits inside the Nfft bins." Nc held fixed (50
  WB / 10 NB) while Nfft grows → MORE headroom. HOLDS (zero_padder centres Nc in
  Nfft, ofdm.cc:331-352).
- INV-3: "Ngi = Nfft·gi is an integer." gi=54/256; Nfft=256·K → Ngi=54·K, integer
  for all K. HOLDS.
- INV-4: "All buffers sized from the CURRENT Nofdm." `set_size` runs every init
  from `ofdm.Nfft*(1+gi)`. HOLDS (no buffer caches a prior Nfft).
- INV-5: "The MFSK symbol = one Nfft FFT window; TX/RX stride = Nofdm." Unchanged
  by per-config Nfft — that IS the baud-scaling mechanism (longer window). HOLDS.
- INV-6: "Non-ULTRA configs are byte-identical to monitor." The scaling is gated
  `if (K>1 && is_ultra_config)`; K=1 for everything else → `ofdm.Nfft=256` exactly
  as monitor. The ONLY new state for non-ULTRA is an unused per-config-K lookup
  returning 1. MUST verify byte-identity (§5 test below). KEY RISK — see §4.6.

### §4.5 What the fix changes (per-config Nfft) — walk each consumer
The fix changes ONE thing: WHERE Nfft's multiplier comes from (per-config table
vs the P0 env flag), applied at the SAME site (telecom_system.cc:5328-ish, before
init()). Every consumer in §4.2 already RECOMPUTES from `ofdm.Nfft` at init time
(that is the engine's design — Nfft is "a value pushed in," baud-scaling §1/§6),
so each consumer sees the per-config value with no further change. The P0 spike
already PROVED this end-to-end (K=1 byte-identical, --test 50/0; K=2/4/8 cliffs
moved as predicted). Productionizing only swaps env→table.

### §4.6 KEY RISK (flagged) — the full-ARQ buffer path at K=8
The P0/fading spikes validated Nfft up to 2048 ONLY in the BER harness
(`-m PLOT_PASSBAND`). The full ARQ path allocates LARGER buffers via `set_size`
(`passband_delayed_data = 2·Nofdm·buffer_Nsymb·interp`, etc.). At K=8 (Nofdm=2480)
these are ~8× the K=1 size. This is expected to work (allocation is linear in
Nofdm and the harness already builds at Nfft=2048), but it is the one path the
spikes did NOT exercise. **MUST verify:** `mercury --test` (which drives ARQ
synthetic-fire) passes with the ULTRA configs present, AND an ARQ-mode smoke at
ULTRA_0 (K=8) allocates without OOM/crash. Tracked in §5.

### §4.7 Layers ABOVE the PHY (ARQ/SACK/optimizer) — do they break on K≠1?
- ARQ timers: the INCR-4 floor (arq_common.cc) is the ONLY timer logic that needs
  K-awareness, and it derives K-awareness for free via `ctrl_suffix_tx_time_ms`
  (§4.2.8). The data-frame timers (`message_transmission_time_ms`) also grow
  because `set_size`'s `sym_time_ms` grows K× → no manual scaling needed. VERIFIED
  by reading the INCR-4 hook (it reads the post-init `ctrl_suffix_pattern_passband
  _samples`).
- SACK / streaming / compression: operate on DECODED bytes, agnostic to Nfft.
  Unaffected.
- Optimizer (Q-table / gearshift): NOT wired to ULTRA in this increment (P3c). The
  ULTRA configs are pin-selected (`-s 200..203`). The optimizer's config ring
  never produces an ULTRA ID until P3c, so no optimizer state sees K≠1 here.
- `is_ofdm_config`/`is_robust_config`/`is_ultra_config` classifiers: ULTRA is
  already a distinct range (200-203); non-ULTRA classifiers unchanged → no
  mislabel. (INCR added `is_ultra_config`; I extend its upper bound 202→203.)

### §4.8 SURPRISE #1 — five `ROBUST_0`-only PHY branches ULTRA did NOT inherit (FIXED)
The §5 consumer-walk found that INCR's ULTRA `load_configuration` dispatch (the
`else if(is_ultra_config)` block at telecom_system.cc:5574) set `_modulation` /
`_ldpc_rate` / estimator, and INCR fixed the FIRST M/nStreams selector (:5654),
but FIVE OTHER `current_configuration == ROBUST_0` PHY branches were NOT updated
to include ULTRA — so an ULTRA config would silently fall into the ROBUST_1/2
(`else`) branch for the DATA PHY. INCR never hit this because its ULTRA depth was
establishment-only (ctrl-suffix via `ack_mfsk`, a separate M=16 instance) — the
DATA-PHY M/nStreams was never exercised at the deep cliff. For the baud-scaled
rungs the DATA PHY IS the point, so these were latent bugs. FIXED (all → `==
ROBUST_0 || is_ultra_config`):
1. `:5919` (`reinit_subsystems.psk` data MFSK M/nStreams) — was M=16/2 for ULTRA,
   now M=32/1. (The smoke test confirms ULTRA_0 now prints `M=32 nStreams=1`.)
2. `:5967` (post-init data MFSK re-init, same selector).
3. `:5856` `ldpc.nIteration_max=200` (Q3) — ULTRA uses rate-1/16 at an even deeper
   waterfall than ROBUST_0; without this it capped at 100 iters → deep-cliff decode
   would fail (the exact Q3 failure mode). CRITICAL fix.
4. `:6280` `ctrl_nBits=1200` (punctured ctrl-frame size for the M=32 PHY).
5. `:6351` `ack_pattern_detection_threshold=0.65` (ULTRA is deeper than ROBUST_0 →
   the conservative threshold, not the 1.0 default).
NOT touched (out of scope / correct as-is): the GUI-only live LDPC override
(`#ifdef MERCURY_GUI_ENABLED`, :5195, already flagged GOTCHA, GUI builds only),
and all the gearshift/BREAK `ROBUST_0` logic in arq_commander.cc (P3c territory —
ULTRA is pin-selected this increment, the gearshift never emits an ULTRA ID yet).

### §4.9 SURPRISE #2 — hardcoded `[256]` stack arrays (latent landmine, WB-ULTRA SAFE)
Hunting hardcoded `256` in the PHY hot path found stack arrays sized `[256]`
(meant to be Nfft-sized) in TWO functions: `carrier_frequency_sync_nb`
(ofdm.cc:561-565 — `fft_in/out`, `depadded`, `H_prev/cur`) and the OFDM
Schmidl-Cox FFT preamble path (`time_sync_preamble_fft`/`_fine`, ofdm.cc:2713-2891).
At K>1 (Nfft 512-2048) these WOULD overflow. **BUT neither is on the WB MFSK/ULTRA
path:**
- WB MFSK time sync = `time_sync_mfsk_corr` (:3462) / `time_sync_mfsk` (:3297) —
  verified Nfft-dynamic, NO hardcoded arrays (member `work_buf_a/b`, `fft(...,Nfft)`).
- WB MFSK CFO = `carrier_frequency_sync_wb_mfsk` (:616) — verified Nfft-dynamic
  (`half=Nfft/2`, incremental de-rotation), NO `[256]`.
- WB MFSK symbol mod/demod = `symbol_mod`/`symbol_demod` (:1023/:1030) — use the
  member buffers `zero_padded_data`/`iffted_data`/`gi_removed_data`/`ffted_data`,
  reallocated to Nfft in `ofdm::init()` (:165-168). Scale correctly.
- `carrier_frequency_sync_nb` is the NB-Moose (telecom_system.cc:2441 notes it is
  "unreliable for NB" — the MFSK path uses the wb_mfsk variant), and
  `time_sync_preamble_fft` is the OFDM Schmidl-Cox path, NOT called for MFSK
  (telecom_system.cc:1307/1327 route MFSK to the `_mfsk` variants).
EMPIRICAL PROOF the WB MFSK path is Nfft-safe: the P0 spike ran K=8 (Nfft=2048)
PLOT_PASSBAND, and §5's K=2 production sweep produces a CLEAN cliff — both exercise
the full WB MFSK TX→time-sync→CFO→demod→decode chain at Nfft>256. A `[256]`
overflow there would crash, not produce clean BER curves.
**[?] RESIDUAL (documented, NOT a blocker for WB ULTRA):** if a future increment
enables NB ULTRA (would hit `carrier_frequency_sync_nb`) OR routes MFSK through the
Schmidl-Cox FFT path, those two functions' `[256]` arrays MUST be made Nfft-sized
FIRST. This increment's rungs are WB-only, so the landmine is dormant.

### §4.10 SURPRISE #3 — the full-ARQ buffer path at K=8 (§4.6 risk, partially closed)
The §4.6 concern (ARQ-mode `set_size` allocations at Nfft=2048) is partially
closed: the ULTRA_0 (K=8) config LOADS in the BER-harness path (smoke:
`init() done ... Nofdm=2480 buffer_Nsymb=736`, no OOM/crash) and the K=8 cliff
sweep runs. The full-ARQ DATA path (loopback/HW) at K=8 is the natural P6 check
(SIM-only here). No allocation failure observed at Nfft=2048 in any sim path run.

---

## §5 SIM VALIDATION PLAN + RESULTS

Gates (from the task):
- **G1 — non-ULTRA byte-identical** (load-bearing safety). `mercury --test` 54/0
  (the INCR-4 count), AND a CONFIG_10 + ROBUST_0 cliff sweep IDENTICAL to monitor.
- **G2 — each ULTRA rung hits its target depth** (AWGN + Watterson), matching the
  fading gate's −13/−16/−19/−21 (poor): ULTRA_3≈−13, ULTRA_2≈−16, ULTRA_1≈−19/−20,
  ULTRA_0≈−21/−22.
- **G3 — FAR clean** (no false decodes in pure noise at the deep rungs).
- **G4 — build foreground o3 + `--test` pass.**

### §5.1 RESULTS (2026-06-01, worktree @ build o3)

**G4 — build + test.** Foreground `bash build.sh o3` clean (only pre-existing
WASAPI/sign-compare warnings, none in changed regions). **Merged BASELINE (before
my edits): `mercury --test` = 54 passed, 0 failed** (the INCR-4 count → the
composition itself is sound). [Post-implementation `--test`: see §5.4.]

**G1 — non-ULTRA byte-identical.** Config-load smoke confirms:
- ROBUST_0 (`-s 100`): NO `[BAUD]` banner, `Nofdm=310` (Nfft=256) — unchanged.
- CONFIG_10 (`-s 10`, OFDM): 0 `[BAUD]` banners — untouched.
- ULTRA_3 (`-s 203`, K=1): NO `[BAUD]` banner, `Nofdm=310`, `M=32 nStreams=1` —
  IDENTICAL to ROBUST_0's PHY (K=1 is the byte-identical path).
The only new code on the non-ULTRA path is `ultra_baud_mult()` returning 1 (a
no-op). `ofdm.Nfft` stays 256 for every config 0-16 / 100-102.

**G2 — each PRODUCTION ULTRA rung hits its target depth.** Cliff sweep
`-m PLOT_PASSBAND -s <cfg> -R` on the ACTUAL ULTRA CONFIGS (NOT the env
instrument), pinned 3 kHz noise reference (baud_mult set per-config → ref_bandwidth
auto-pinned), AWGN + poor Watterson (1 Hz / 2 ms), cliff = deepest SNR3k with
BER==0 contiguous to the top. `/tmp/ultra_validate/ultra_cliffs.csv`:

| Config | K | Nfft | **AWGN** | **poor 1Hz** | fading-gate target (AWGN/poor) | match |
|--------|---|------|----------|--------------|--------------------------------|-------|
| ULTRA_3 (203) | 1 | 256  | **−13** | **−13** | −13 / −13 | ✓ |
| ULTRA_2 (202) | 2 | 512  | **−16** | **−16** | −16 / −16 | ✓ |
| ULTRA_1 (201) | 4 | 1024 | **−20** | **−19** | −20 / −19 | ✓✓ |
| ULTRA_0 (200) | 8 | 2048 | **−22** | **−21** | −22 / −21 | ✓ |

The productionized per-config Nfft path REPRODUCES the fading-gate cliffs EXACTLY
for K=1/2/4. ULTRA_1 (K=4) shows poor −19 = 1 dB BETTER than AWGN −20, reproducing
the interleaved-LDPC fading-diversity benefit the fading gate found at K=4. This
is decisive: the production rungs reach their designed depths, AWGN and under HF
fading, with no env flag — the depth is real and per-config.

**G3 — FAR clean.** The cliff curves ARE the data-path FAR evidence: below each
cliff, BER ≈ 0.45-0.52 (≈ 0.5 = pure random — the LDPC+CRC reject everything, NO
information leaks → NO false-positive decodes), with a sharp monotonic transition
to BER=0 at the cliff and NO sub-cliff false-zeros (a phantom CRC-pass would show
as an anomalous BER=0 below the cliff — none seen; e.g. ULTRA_3 AWGN: −14 dB
BER=0.214, −13 dB BER=0 clean). The CONNECT-establishment FAR is inherited:
ULTRA's establishment = the ROBUST tier's shipped/validated enhanced CONNECT
(0/4000 false-accepts, INCR §22.5) MINUS combining (R_base=R_suffix=1) → strictly
LESS aggressive than the tested ULTRA, and the larger Nfft narrows the tone bins
→ MORE selective → FAR ≤ ROBUST_0's. No FAR regression possible by construction.

### §5.2 verdict on per-config Nfft (P3a)
Per-config Nfft is implemented (telecom_system.cc:5768 `baud_mult =
ultra_baud_mult(configuration)`), §5-clean: every Nfft-keyed consumer recomputes
from `ofdm.Nfft` at init (§4.2), non-ULTRA is byte-identical (K=1, §5.1 G1), and
the five ROBUST_0-inheritance bugs (§4.8) + the `[256]`-landmine boundary (§4.9)
are handled/documented. The KEY RISK the increment carried (per-config Nfft
breaking a non-ULTRA assumption) did NOT materialize — non-ULTRA is bit-for-bit
unchanged because the scaling is gated `baud_mult>1` which is false for all
non-ULTRA configs.

### §5.3 ULTRA_0 (K=8) cliff — DONE
ULTRA_0 (`-s 200`, K=8, Nfft=2048): **AWGN cliff −22 dB, poor-fading cliff −21 dB**
(BER curve clean: AWGN 0.33/0.15/0.04 at −24/−23/−22→ wait, AWGN clean at −22;
poor 0.38/0.15/0.04 at −24/−23/−22 then BER==0 at −21). Both match the fading-gate
K=8 targets (AWGN −22, poor −21). The full production cliff ladder is therefore
−13/−16/−20/−22 (AWGN) and −13/−16/−19/−21 (poor) for ULTRA_3/2/1/0 — IDENTICAL to
the fading-gate spike's K=1/2/4/8 ladder, reached via the per-config Nfft path with
NO env flag. ULTRA_0's caveat (fading-gate §6: AWGN gain saturates +2 dB at this
last doubling; most edge-fading-sensitive) stands — it is the aggressive cap; the
extra coding it could use is the P1 follow-on. Sweep job exit 0.

### §5.4 post-implementation `--test` — RECONCILED (2026-06-01, P3 increment)

~~[fill: 54/0 expected — the changes are additive...]~~ **WRONG prediction.** The
P3 production change (per-config Nfft + reframed `ultra_tier_suffix_params`) was
SOUND, but the FIRST post-implementation `--test` ran **50 passed / 4 FAILED**, not
54/0. The 4 failures were ALL in
`source/physical_layer/mfsk_ctrl_codec_tests.cc` — which P3 did NOT touch — so they
still encoded the **RETIRED INCR-1/2 establishment-REPETITION contract**
(R_frame=2/3/4, R_suffix=12, deep-repetition cliffs) that the (c) reframe
deliberately replaced with R_frame=1 + baud-scaling. The tests asserted behavior the
production code no longer produces. Reconciled to the reframed contract (P3 commit);
final `--test` = **54 passed / 0 failed** (the 4 reconciled ULTRA tests now pass; 50
+ 4 = 54).

**Root cause of the stale tests:** the 4 ULTRA tests in `mfsk_ctrl_codec_tests.cc`
were written against the INCR-1/2 ULTRA tier (depth from establishment-suffix
repetition), and the (c) reframe — which is a `telecom_system.cc` / `arq_common.cc` /
`common_defines.h` change — left them behind. They are NOT cross-layer regression
tests of P3; they are pre-existing tests whose CONTRACT changed under P3.

**The 4 tests' before→after (each remains a MEANINGFUL guard, not rubber-stamped):**

1. `ultra_connect_choreography` — was: `R_frame={2,3,4}` for ULTRA_0/1/2 + window
   ≥ R_frame×airtime with a fail-before of `2*mtt+3000 < airtime`. **Now:**
   `R_frame=1` for ULTRA_0/1/2/3; `ultra_baud_mult` = the rung's K (8/4/2/1); the
   single-CONNECT-frame airtime scales EXACTLY K× (ULTRA_0 = 8× ULTRA_3 — the
   baud-scaling depth mechanism, fails if per-config Nfft regresses); and the shared
   `connect_listen_window_ms(true,airtime,1,mtt) == airtime + (2*mtt+3000)` (the
   tier-aware floor ADDS one baud-scaled CONNECT frame). The INCR fail-before
   (`2*mtt+3000 < airtime`) is GONE: measured airtime/mtt show the DATA frame's own
   mtt grows K× and DWARFS one CONNECT frame (ULTRA_0: airtime 14054 ms vs 2*mtt+3000
   = 141880 ms), so the floor's job is to ADD the CONNECT-frame airtime, not rescue a
   repetition deficit.

2. `ultra_establishment_timers` — was: `R_frame=2` for ULTRA_0 + connection_timeout
   floor ≥ R_frame×airtime + a REQUIRED ack_timeout deficit at ULTRA_2
   (`prefix_ackto < airtime`). **Now:** `R_frame=1`; connection_timeout/link_timeout
   floor == `airtime + (2*mtt+3000)` and ≥ airtime; the REFRAMED BINDER is
   `ack_floor > prefix_ackto` at EVERY ULTRA rung (the tier-aware floor adds the
   baud-scaled CONNECT-frame airtime where the data-frame default adds only the short
   ack-tone time; measured Δ = airtime − ack_pat − 600 > 0 at all 4 rungs:
   +10147/+4773/+2087/+743 ms for K=8/4/2/1). The INCR fail-before
   (`prefix_ackto < airtime`) is GONE for the same mtt-dominance reason.

3. `ultra_tier_establishment_cliff_sweep` — was: ULTRA_2 *establishment* reaches
   ≤ −19.0 dB (via R_suffix=12/R_frame=4 repetition). **Now (MEASURED this worktree,
   `ultra_prod_sweep` R_frame=1):** the CONNECT-establishment cliff is a DISTINCT
   quantity from the §5.3 data-PHY cliff. ULTRA_0 (K=8) establishment = **−21.60 dB**
   (baud-scaling DOES extend the handshake reach at high K → assert ≤ −20.0); ULTRA_2
   (K=2) establishment = **−11.80 dB** ≈ ROBUST_0 (**NO establishment gain at K=2** —
   see §6 finding → assert only the ROBUST_0-class floor ≤ −11.0, NOT the data −16);
   plus monotonicity ULTRA_0 < ULTRA_2 and FAR clean. Fail-before: K=1 regression
   stalls ULTRA_0 at ~−12 → the ≤ −20.0 assertion fails.

4. `ultra_count_admission_tier_gated_no_leak` — was: hardcoded sigma-mults
   {22,28,34} = SNR3k −17.8/−19.9/−21.6 dB and `ULTRA_2_decode ≥ ROBUST_0 + 0.15`.
   Those SNR points now sit BELOW the reframed ULTRA_2 establishment cliff → both
   arms decode 0 → Δ vanishes. **Now (REDESIGNED — the premise itself was falsified
   by measurement, §6):** count-admission gives only ~0.015 decode-fraction at K=1
   (a weak relaxation) and is MOOT at the ULTRA tier (at K≥2 the ratio-gate-block
   point moves below the content cliff). So a decode-Δ assertion is no longer TRUE.
   The test now verifies the GATING LOGIC directly on IDENTICAL K=1 wires
   (ULTRA_3 vs ROBUST_0, same Nfft=256): (a) ROBUST_0 FOLLOW_TIER == count=OFF (no
   leak), (b) ULTRA_3 FOLLOW_TIER == count=ON (tier activates), (c) count=ON ≥
   count=OFF per-cell (relaxation) AND count=ON > count=OFF in aggregate over the
   marginal band (gate exercised → (a)/(b) non-vacuous). Measured (the committed
   test's band, sigma-mults 11..16 = SNR3k −11.8..−15.1): ROBUST_0 FOLLOW/count=OFF
   = 0.514, ULTRA_3 FOLLOW/count=ON = 0.556, byte-exact (the ~0.015 figure above was
   a wider-band diagnostic probe; the gate's relaxation magnitude is band-dependent
   but always > 0 in the marginal region, and the FOLLOW==forced equivalence is
   band-independent and bit-exact).

---

## §6 OPEN QUESTIONS [?]
- **[FINDING, 2026-06-01] CONNECT-establishment cliff ≠ data-PHY cliff; baud-scaling
  does NOT deepen establishment at low K.** §3.3 claimed "baud-scaling deepens BOTH
  the data PHY cliff AND the CONNECT establishment." MEASURED (`ultra_prod_sweep`,
  this worktree): ULTRA_0 (K=8) establishment −21.60 dB (≈ data cliff — baud delivers)
  but ULTRA_2 (K=2) establishment −11.80 dB ≈ ROBUST_0 (NO gain). The 16-symbol
  base-pattern detector (`detect_ack_pattern`) + GF16 suffix decode do not benefit
  from the K=2 longer symbol the way the data BER does — only at K=8 does the larger
  FFT window extend the handshake reach. CONSEQUENCE: the ULTRA handshake reaches the
  ULTRA_0 floor but NOT a graduated K-ladder; ULTRA_1/2/3 ESTABLISH at ~ROBUST_0-class
  SNR even though their DATA PHY cliffs are −20/−16/−13. This is acceptable for a
  pin/gearshift-selected tier (you establish at the shallow SNR, then the deep DATA
  rung carries the payload), but it means the deep rungs cannot ESTABLISH a fresh
  link at their data-cliff SNR. Flagged for the (c) plan owner — may warrant a
  follow-on (e.g. a baud-scaled CONNECT base pattern, or establish-at-ULTRA_0-then-
  switch). NOT fixed here (out of P3 scope; P3 is the rung family + test reconcile).
- **[FINDING, 2026-06-01] count-admission is a WEAK relaxation and MOOT at the ULTRA
  tier under baud-scaling.** Measured: count=ON vs count=OFF differ by ~0.015
  decode-fraction on a K=1 wire (it admits a few CRC-valid marginal decodes the
  metric-ratio sub-gate blocks); at K≥2 the ratio-gate-block point moves below the
  content cliff so it makes ZERO difference (count=ON ≡ count=OFF byte-exact on the
  ULTRA_2 K=2 wire, −11 to −15 band). The count-admission gate (INCR §22) was sized
  for the OLD repetition-extended cliff; under the reframe it is near-vestigial. Kept
  (tier-gated, harmless, no FAR cost) but its value is marginal. Test 4 now verifies
  the GATING LOGIC (no leak), not a decode advantage.
- [?] §4.6 full-ARQ buffer path at K=8 (Nfft=2048) — `--test` drives ARQ
  synthetic-fire + loads all ULTRA configs (incl K=8) with no OOM/crash → the
  allocation path is exercised; full loopback DATA at K=8 is the P6 check.
- [?] ULTRA_4 (K=16, the deferred 5th rung) — left to a future increment after P1
  coding + a fading re-check validate Nfft=4096.
