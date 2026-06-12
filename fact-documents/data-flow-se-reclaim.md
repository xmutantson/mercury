# Data-Flow: SE-Reclaim Grid (CP / pilot-cadence / Nsymb) as per-direction wire-negotiated state within CFG15

Fact document for the productionization of the sim-proven SE-reclaim clean-front beat.
Pairs with the regression tests `--test-se-reclaim` (wire + materialization, Stages 1-2) and
`--test-se-reclaim-transition` (the mandatory cross-layer transition test, Stage 5).

Every claim is cited `file:line`. Facts come from executing/reading code, not comments.
Open questions are marked `[?]`. Corrections are struck through, never deleted.

---

## §0 SUMMARY / STATUS

**The proven beat.** On the clean/good front, OFDM overhead (cyclic prefix + time-pilots) can
be reclaimed on the *reliable* CFG15 rung (16-QAM r0.875) to beat VARA HF. Sim verdict
`_research/SE_RECLAIM_ARMA_VERDICT.json` (arm_a_sweep_GOOD_e14):

| Grid | Ngi | Dy | Nsymb | nData | rbc (bps) | xVARA | decode (CCIR-GOOD @EsN0 14) |
|------|-----|----|-------|-------|-----------|-------|------------------------------|
| FULL (stock) | 54 | 3 | 12 | 400 | 3348.4 | 0.947 | 100% |
| **RECLAIM-PILOTS** | 54 | 5 | 10 | 400 | **3826.7** | **1.082** | 100% (full CP → zero sync risk) |
| RECLAIM-FULL | 18 | 5 | 10 | 400 | 4329.5 | 1.224 | 100% (reduced CP → sync-gated) |

VARA clean effective = 13048 bps; 1x wire = 3536; conservative target wire = 3825
(`SE_RECLAIM_ARMA_VERDICT.json` calibration_anchors). The `× 3.69` is the text-compression
multiplier; the rbc figures are the **per-frame PHY ceiling at 100% decode** — the end-to-end
number additionally pays ARQ/turnaround overhead (verdict honest_bounding); a pinned-CFG15
A/B + compress-ON on the bench is the final confirmation before any HW claim.

**The shared state introduced.** Today the entire OFDM grid (Ngi, Dy, Nsymb, nData, buffer
geometry) is derived 100% RX-side from the **config index** alone (`load_configuration(int)`,
telecom_system.cc:9034-9048). This document introduces a *new* shared state object: a
per-direction **GridSelector** `{forward_grid, reverse_grid}` (enum `{GRID_FULL=0,
GRID_RECLAIM=1}`) that pairs with `(forward_configuration, reverse_configuration)`, so CFG15
maps to TWO grids instead of one. The selector rides the existing CRC16+LDPC+ACK-confirmed
SET_CONFIG handshake.

**Default-safe posture.** Default and startup = `GRID_FULL` both directions. A legacy peer that
never sends the selector bytes transmits them as **0 = GRID_FULL** automatically (transmit_byte
zero-pads the codeword tail, telecom_system.cc:770-777) → byte-identical, back-compatible.
RECLAIM is opt-in only on a confidently-clean FORWARD link, slow-promote / instant-demote.

**Status: GO (conditional)** per verdict production_go_no_go. The wire-framing is minimal and
low-risk. The binding risk is GATE RELIABILITY against a SHARP knee (a mislabel applies the
reduced grid to a fade and loses traffic) — a bench question, not a sim question. Stage 6
(optimizer election) is HELD behind a default-OFF feature flag until the bench A/B confirms the
gate. Lowest-risk ship target = RECLAIM-PILOTS (1.08x, full CP, zero sync risk).

---

## §1 THE SHARED STATE

`GridSelector`: a `(forward_grid, reverse_grid)` pair of `se_grid_t` enum values, defined in
`include/common/common_defines.h` alongside the config defines:

```
enum se_grid_t { GRID_FULL = 0, GRID_RECLAIM = 1 };
```

It is paired with `(forward_configuration, reverse_configuration)` (arq members, set at
arq_commander.cc:705-706, swapped on role-reversal at arq_responder.cc:1322-1324).

The grid PARAMS each selector expands to (for CONFIG_15 only; every other rung is FULL by
construction):

| selector | Ngi | Dy | Nsymb | nData | rbc |
|----------|-----|----|-------|-------|-----|
| GRID_FULL | 54 | 3 | 12 | 400 | 3348.4 |
| GRID_RECLAIM (PILOTS, shippable) | 54 | 5 | 10 | 400 | 3826.7 |
| GRID_RECLAIM (FULL, sync-gated) | 18 | 5 | 10 | 400 | 4329.5 |

**Hard constraints on the params (verified):**

- **nData = N/log2M = 400 is FIXED** for CFG15 (LDPC N=1600 bits → 400 16-QAM data REs;
  verdict fixed_codeword_constraint; telecom_system.cc:275 `nVirtual_data = ldpc.N - nBits`
  goes NEGATIVE and overruns if nData > 400). Reclaim is cashed in by SHRINKING Nsymb
  (12→10) and Ngi (54→18), NEVER by growing nData.
- **Dy ≤ 5.** Dy ≥ 6 is DEGENERATE: the production LS estimator FLOORS (decode_rate=0) even at
  EsN0=30 (verdict LS_estimator_Dy_ceiling). Dy is clamped ≤ 5 by construction in the
  materializer.

---

## §2 PRODUCERS (every code path that WRITES this state; file:line)

1. **SET_CONFIG selector producer (TX wire):** `arq_commander.cc:705-707` writes
   `data[1]=forward_configuration`, `data[2]=reverse_configuration`, `length=3`. Extended to
   also write `data[3]=forward_grid`, `data[4]=reverse_grid`, `length=5`.
2. **Grid election producer:** `arq_commander.cc:5180-5212` — today OBSERVATION-ONLY
   (`get_correlator_snr_proxy()` + `get_channel_selectivity()` + `channel_lookup.lookup()`;
   proposal never acted on). The new forward-link gate writes `forward_grid` here.
3. **Grid-param materializer:** `telecom_system.cc:9034-9048` copies the stock grid
   (Nc/Nfft/gi/Nsymb/Dx/Dy) from `default_configurations_telecom_system` — same for every OFDM
   config. The SIM experiment overrides via env at telecom_system.cc:9068-9100
   (`MERCURY_SE_NGI`/`SE_DY`/`SE_NSYMB`, gated `configuration==CONFIG_15`). Production replaces
   this with a deterministic `(config,grid)→{Ngi,Dy,Nsymb}` table.
4. **Forward-selectivity producer (RSP-side):** `telecom_system.cc:2862-2897`
   `last_channel_selectivity = std(|H|)/mean(|H|)` over DATA subcarriers — the fade signal the
   gate consumes.
5. **GI producer:** `ofdm.cc:148` (`this->gi=gi`), `ofdm.cc:162` (`Ngi=Nfft*gi`) — ONE shared
   Ngi for preamble AND data.
6. **data_container geometry producer:** `telecom_system.cc:4881`/`4885`
   `set_size(..., ofdm.Nfft*(1+ofdm.gi), ...)`; `data_container.cc:95-104` sets
   `this->Nofdm=Nofdm`, `this->Ngi=Nofdm-Nfft`; `data_container.cc:194` total_frame_size uses
   the single Nofdm for both preamble and data regions.
7. **mean_H / SKIP-H producers:** `telecom_system.cc:2844-2861` (`receive_stats.mean_H`).
8. **rbc / Tf producer:** `telecom_system.cc:3509-3514` (`rbc = rb*LDPC_real_CR`,
   `Tf=Ts*(Nsymb+preamble_nSymb)`, `Ts=Tu*(1+gi)`).

---

## §3 CONSUMERS (every code path that READS this state; file:line)

1. **SET_CONFIG selector consumer (RX wire):** `arq_responder.cc:2507-2508` reads
   `forward_configuration=data[1]`, `reverse_configuration=data[2]`. Extended to read
   `data[3]`/`data[4]` → grids. **NOTE (verified, INV-5):** the RX `messages_control.length` is
   hardcoded to 1 (arq_responder.cc:511) — it does NOT carry the wire length — so a `length<5`
   guard is impossible. Backward-compat is instead guaranteed by transmit_byte zero-padding
   (§5 INV-5).
2. **Two-phase grid installer:** non-monitor path `arq_responder.cc:2573-2581`
   (saves `data_configuration`, loads after ACK at arq_responder.cc:1393); monitor path
   `arq_responder.cc:2526-2551` (immediate `load_configuration` + capture flush);
   CMD side `arq_commander.cc:1152-1154`/2095 load on data/ack config.
3. **Universal grid installer:** `load_configuration(int)` telecom_system.cc:9034-9048 — must
   learn the grid via a member (`pending_grid_selector`) so RX and TX derive IDENTICAL geometry.
4. **Demod-lattice consumers:** `ofdm.cc:2173-2209` channel_equalizer (MMSE
   `alpha=|H|^2/(|H|^2+nv)`); psk.cc LLR demod reads `noise_variance_estimate` (nv-scaled).
5. **Estimator consumers:** LS estimator `ofdm.cc:1649-1772` (window LS_window_hight=9, span
   i±4); cross-pilot nv `ofdm.cc:1475-1518` (pairs when `(i-prev_row)==Dy`); DFT smoother
   `ofdm.cc:2127-2171` (`window_taps=(int)(gi*Nc+0.5)+2`, AUTO-TRACKS gi, guards :2140-2141);
   linear interpolation `interpolator.cc:163-252` bridges the Dy-row pilot gap.
6. **Gate constants (calibrated on the Dy3/Ngi54 lattice):** SKIP-H mean_H 0.30
   (`telecom_system.cc:2929`), sub-peak reject mean_H<0.5 (`telecom_system.cc:2912`),
   selectivity bins (`channel_state_lookup.cc`), estimator sel≥0.15 split
   (`telecom_system.cc:7520-7521`).
7. **Sync / stride consumers:** live Schmidl-Cox `ofdm.cc:2606-2710` (core metric L=Nfft/nIS
   GI-INDEPENDENT at :2625; per-symbol stride `base=d+sym*Nofdm`, Nofdm Ngi-dependent at :2653);
   batch-predict `telecom_system.cc:1487`/1511/1517/1521 (`predicted_pos=ofdm_skip*sym_samples
   +drift`, verify window ±2*gi_interp); fine-timing search `telecom_system.cc:7446`/7457;
   **TX FIR tap budget** `telecom_system.cc:4700-4713` (`filter_nTaps ≤ Ngi*interp` or
   Schmidl-Cox GI-copy property is destroyed; TX FIR=97 taps, RX time-sync FIR=33 taps; at
   Ngi=18 the budget is 72 → 97 VIOLATES it).
8. **Optimizer consumers:** `rate_optimizer.cc:462-513` identify_channel_label (the gate that
   MUST NOT drive the grid decision — AWGN-only, noisy, 50-batch-lagged, reverse-direction);
   opt window `arq.h:2749`/2808 (must `opt_reset_window()` on grid switch).
9. **RX buffer consumers:** `arq_common.cc:1216`/1254/1843 (Nofdm/Nsymb-sized capture buffers).
10. **use_last_good timing consumers:** `telecom_system.cc:2313`/2620 read
    `delay_/freq_offset_of_last_decoded_message` (grid-relative sample units; defaults YES,
    physical_config.cc:86-87).

---

## §4 VALID STATES (enumerate; especially the pre-write state)

- **Default-init (startup, pre-any-producer):** both grids = `GRID_FULL` (0). This is the
  state the data sits in before any election; the modem behaves EXACTLY as today.
- **Legacy peer (never sends data[3]/data[4]):** transmit_byte zero-pads → data[3]=data[4]=0
  decoded RX-side as `GRID_FULL/GRID_FULL` (back-compat; §5 INV-5).
- **Mid-switch transient (CMD elected RECLAIM, ACK not yet confirmed):** both ends MUST still be
  on the LAST-AGREED grid. The two-phase SET_CONFIG+ACK (arq_responder.cc:1393, 2573-2581)
  guarantees neither side transmits DATA on the new grid until the peer has the selector.
- **FORBIDDEN: RX on FULL while TX on RECLAIM (or vice-versa)** → preamble still syncs (grid-
  agnostic, §5 INV-2) but every data symbol demods on the wrong Ngi/Dy/Nsymb lattice → whole-
  batch decode collapse to ~0 (§5 INV-3). This is the dominant hazard.
- **FORBIDDEN: Dy ≥ 6** (degenerate; §1).
- **FORBIDDEN: grid change mid-batch** — only at a batch/SET_CONFIG boundary (§5 INV-3).

---

## §5 INVARIANTS (consolidated from the 4 audit lenses, deduped)

- **INV-1 (codeword fits the lattice):** nData = N/log2M = 400 fixed; reclaim cashed by
  SHRINKING Nsymb (+ Ngi), never growing nData (telecom_system.cc:275 guard; verdict
  fixed_codeword_constraint). The materializer hard-codes Nsymb per (Dy,Ngi) so RX and TX
  derive the SAME nData=400. FATAL if Nsymb is not grid-keyed.
- **INV-2 (GRID == CONFIG-INDEX today; preamble grid-agnostic):** RX derives the entire grid
  from the config index (telecom_system.cc:9034-9048); the preamble is identical across all
  OFDM configs (ofdm.cc:1341-1406 depends only on Nc/Nfft/start_shift) so RX cannot infer the
  grid from the air. The change makes CFG15 map to TWO grids — every "index fully determines
  grid" consumer must read index + selector. The Schmidl-Cox CORE metric is GI-independent
  (ofdm.cc:2625), so the preamble still DETECTS under a short data GI; but the preamble's own
  GI (shared field) and every stride DO scale with Ngi.
- **INV-3 (DATA frame carries NO grid):** DATA/DATA_SHORT headers are
  [type,conn_id,seq,(bsi),id,length] (arq_common.cc:4486-4527) — zero grid bytes. RX demods on
  whatever grid `load_configuration` last installed. Therefore the selector MUST be
  ACK-confirmed BEFORE any data frame is sent on the new grid, and the grid changes ONLY at a
  batch boundary.
- **INV-4 (SET_CONFIG is CRC16+LDPC + two-phase ACK):** the config index rides inside a full
  control codeword (CRC16 physical_config.cc + LDPC) and the switch is two-phase (CMD sends on
  OLD cfg, RSP ACKs on OLD cfg, then both load NEW; arq_responder.cc:1393, 2573-2581). A
  bit-flipped index fails CRC → frame dropped → retransmit, never silently mis-applied. The
  selector inherits this exact channel (same frame, same CRC, same ACK).
- **INV-5 (length-5 wire-safe + back-compat, REVISED from the plan):** the RX always copies the
  FULL control codeword payload into `messages_rx_buffer.data[]`
  (`copy_len = max_data_length+max_header_length-CONTROL_ACK_CONTROL_HEADER_LENGTH`,
  arq_common.cc:7779-7784) and into `messages_control.data[]` (arq_responder.cc:514-518) →
  data[3]/data[4] are ALWAYS populated from the wire. **The plan's "if length<5 ⇒ FULL" guard
  is NOT implementable** because the RX hardcodes `messages_control.length=1`
  (arq_responder.cc:511) — the wire length is lost. Back-compat is instead guaranteed by the TX
  path: `transmit_byte(data, nBytes=header+length, ...)` byte-aligns only `length` payload
  bytes and ZERO-PADS the rest of the frame to frame_size (telecom_system.cc:770-777). So a
  legacy length-3 SET_CONFIG transmits data[3]=0, data[4]=0 = `GRID_FULL/GRID_FULL`. The
  responder therefore reads data[3]/data[4] UNCONDITIONALLY; legacy zeros decode as FULL.
- **INV-6 (LS window catches ≥2 pilots OR interpolation bridges):** Dy ≤ 5 SAFE (Dy=5 → single-
  pilot LS + linear interp bridges the 5-row gap within the 16-QAM margin on a slow channel),
  Dy ≥ 6 FLOORS (verdict). Clamp Dy ≤ 5.
- **INV-7 (DFT smoother window auto-tracks gi):** `window_taps=(int)(gi*Nc+0.5)+2`
  (ofdm.cc:2139) narrows appropriately for the reduced CP (Ngi=54→~12 taps, Ngi=18→~5 taps,
  Nc=50). NOT broken by the gi change; verify guards (taps ≥ 3, < Nc/2) at ofdm.cc:2140-2141.
- **INV-8 (gate constants calibrated on the stock grid):** mean_H 0.30, sub-peak 0.5,
  selectivity bins were calibrated on the Dy3/Ngi54 lattice; the reduced grid changes the
  MEASURED-bin count and |H| spread → they read DIFFERENTLY. Re-measure for the reclaim grid OR
  anchor the DEMOTE decision on a periodic FULL-grid selectivity probe, never reduced-grid
  self-measurement (the selectivity self-consistency trap: a sparse lattice under-resolves the
  very fade it must detect).
- **INV-9 (TX FIR ≤ Ngi*interp for Schmidl-Cox):** telecom_system.cc:4700-4713. 97 taps > 72 at
  Ngi=18 → the short-GI preamble (RECLAIM-FULL) risks Schmidl-Cox degradation. RECLAIM-PILOTS
  keeps Ngi=54 (budget 216) → no violation. RECLAIM-FULL is gated OFF until a loopback sync-
  margin check (Stage 4) passes OR a GI-safe TX FIR / per-region GI is built.
- **INV-10 (use_last_good timing is grid-relative):** `delay_of_last_decoded_message` is a
  passband-sample offset tied to Nofdm=Nfft*(1+gi) (defaults YES, physical_config.cc:86-87).
  Reusing an Ngi=54-learned delay as the fallback on an Ngi=18 frame is a unit mismatch → fine-
  timing lands off → mean_H collapses → SKIP-H (looks like a fade). MUST invalidate on grid
  switch.
- **INV-11 (single Nofdm across the whole frame):** data_container.cc:194 total_frame_size uses
  ONE Nofdm for preamble + data → per-region GI (long-GI preamble / short-GI data) is a
  substantial structural change, DEFERRED. Stage-1 RECLAIM-FULL (if elected) is whole-frame
  short-GI.

---

## §6 WHAT THE FIX CHANGES (per-consumer walk)

The change introduces the GridSelector and makes the materializer/installer grid-aware. Per INV:

- **INV-1/INV-2:** materializer (telecom_system.cc:9034-9048 region) reads a member
  `pending_grid_selector` and applies a deterministic `(CONFIG_15, GRID_RECLAIM)→{Ngi,Dy,Nsymb}`
  table → both ends derive IDENTICAL geometry from negotiated state, no env. Every other config
  / GRID_FULL → stock path unchanged (byte-identical).
- **INV-3/INV-4:** the selector rides SET_CONFIG (producer arq_commander.cc:705-707, consumer
  arq_responder.cc:2507-2508) → ACK-confirmed before any data on the new grid; grid change only
  at the SET_CONFIG/batch boundary.
- **INV-5:** TX writes data[3]/data[4] + length=5; RX reads them unconditionally; legacy zeros =
  FULL.
- **INV-8 (gate):** gate on FORWARD selectivity (RSP-reported on the ACK suffix), NOT the CMD
  reverse-link value, NOT the rate_optimizer label. DEMOTE anchored on a conservative predicate;
  do not trust reduced-grid self-measurement for the demote.
- **INV-10:** invalidate `delay_/freq_offset_of_last_decoded_message` on every grid transition.
- **Capture-flush / opt-reset:** route the grid switch through the SAME
  `load_configuration(PHYSICAL_LAYER_ONLY)` reinit + capture-flush path the config switch uses
  (arq_responder.cc:2539-2547), EXTENDED to fire on a 15→15 GRID change (config index
  unchanged), with `opt_reset_window()` on every transition (else the 50-batch window blends two
  geometries and mislabels).

---

## §7 TRANSITION HAZARDS (the regression-test charter for `--test-se-reclaim-transition`)

H1. **RX/TX grid disagreement** (dominant): TX-RECLAIM / RX-FULL → preamble syncs, data collapses
to ~0. Test: a length-5 SET_CONFIG round-trips and the grid is adopted only after ACK; the
in-flight batch completes on its STARTING grid.
H2. **15→15 capture-buffer stale-grid:** a FULL→RECLAIM switch within CONFIG_15 changes Ngi
(frame length in samples) but the OLD config-switch flush gate (`forward_configuration !=
current_configuration`, arq_responder.cc:2526/2573) does NOT fire (15==15) → stale FULL-grid
samples demod on the RECLAIM lattice. Test: the flush fires on GRID change too.
H3. **Buffer re-size race:** Nofdm/Nsymb change between grids → RX data_container buffers must be
re-derived under capture_prep_mutex via the load_configuration reinit path, never a partial
in-place Ngi poke. Test: post-switch buffer geometry matches the new grid.
H4. **Mislabel / fade-onset (the safety test):** drive the gate clean to promote, then inject a
fade onset (selectivity spike / FER>0) → INSTANT demote to FULL, at-most-ONE-batch loss, self-
heal, and the demote does NOT depend on a successful decode (no-progress/timeout fallback).
H5. **Asymmetric role-reversal:** the (forward_grid,reverse_grid) pair swaps in lockstep with
(forward_configuration,reverse_configuration) at arq_responder.cc:1322-1324 → the forward grid is
never applied to the reverse direction.
H6. **opt_window poisoning:** `opt_reset_window()` on every grid transition.
H7. **use_last_good timing carryover (INV-10):** invalidated on every transition.
H8. **Sync margin under shrunk CP (RECLAIM-FULL only, INV-9):** Schmidl-Cox lock + first-frame
mean_H on the Ngi=18 whole-frame grid within the acceptance band of Ngi=54 (Stage 4 loopback).
If it regresses → keep the shippable target at RECLAIM-PILOTS (Ngi=54, zero sync risk).

---

## §8 OPEN QUESTIONS [?]

- [?] Does the 97-tap TX FIR degrade Schmidl-Cox at Ngi=18 enough to matter on the clean front?
  (Stage-4 loopback; if yes → RECLAIM-FULL stays gated OFF pending per-region GI.)
- [?] Do mean_H / selectivity gate constants shift materially on the Dy5/Ngi18 lattice? (Stage-4
  estimator-constant sanity / A-B.)
- [?] EsN0 ↔ SNR3k mapping for the gate thresholds (SEL_RECLAIM_MAX, SNR margin) — bench-only.
- [?] Is the pilots-only Dy5/Ngi54 1.08x cell sufficient to ship first (no sync risk at all)? —
  current answer: YES, it is the lowest-risk ship target and the Stage-3 gate elects it.
- [?] Realized e2e wire after ARQ/turnaround overhead vs the 3826 per-frame rbc — bench A/B.

---

## §9 PRIOR-ART / CITATIONS

- Qualcomm US9485678B2 — "Effective utilization of cyclic prefix ... under benign channel
  conditions" (reduced CP under benign channels).
- IEEE doc 8301857 — variable-CP coded OFDM throughput.
- Watterson (1970) / CCIR-520 / CCIR-GOOD — HF channel model used in the verdict sweep.
- ARDOP / PACTOR negotiated-config handshake — analog for the two-phase wire negotiation.
- Verdict: `_research/SE_RECLAIM_ARMA_VERDICT.json` (agent aeda12d0). Experiment branch:
  `sim/se-reclaim-cp-pilots @ d364fcf` (env-gated ARM-A hooks, default-off byte-identical).

---

## §10 IMPLEMENTATION STATUS (Stages 0-5 built; Stage 6 held)

Built on `sim/se-reclaim-cp-pilots` on top of `d364fcf`. Default-off, byte-identical
to the baseline; Stage 6 (optimizer election) HELD behind the default-off gate flag.

**What shipped:**
- §1 enum `se_grid_t {GRID_FULL=0, GRID_RECLAIM=1}` (common_defines.h), ARQ pair
  `forward_grid`/`reverse_grid` + tracker `current_grid` (arq.h).
- Stage 1 wire: SET_CONFIG `data[3]`/`data[4]` + length 3→5 (arq_commander.cc producer,
  arq_responder.cc consumer), role-reversal lockstep swap, legacy zero → FULL.
- Stage 2 materializer: `pending_grid` member (telecom_system.h) → deterministic
  `(CONFIG_15,GRID_RECLAIM)→{Ngi,Dy,Nsymb}` (telecom_system.cc); ARQ pushes the
  direction-keyed grid at the single load_configuration chokepoint (arq_common.cc).
  RECLAIM-PILOTS = 54/5/10 (rbc 3826.7); RECLAIM-FULL = 18/5/10 (rbc 4329.5,
  `MERCURY_SE_RECLAIM_FULL`-gated).
- Stage 3 gate: `cl_se_reclaim_gate` (se_reclaim_gate.h), default-OFF flag
  `se_reclaim_gate_enabled` (`MERCURY_SE_RECLAIM_GATE`). Slow-promote CONFIRM_N=10 /
  instant-demote / no-progress fail-safe; fed FORWARD selectivity+SNR+FER. The
  gate-feed is NOT called from the CMD reverse-link point (INV-4); the RSP-forward
  ACK-suffix transport is the held Stage-6 wiring.
- Stage 4 sync verify: loopback Schmidl-Cox lock + mean_H, Ngi=18 vs Ngi=54.
- Stage 5 transition test (`--test-se-reclaim-transition`) + the 15→15 fix below.

**Key finding (Stage 5, caught by the transition test — H2/H3):** the same-config
early-return EXISTS AT TWO LAYERS — the ARQ wrapper (`cl_arq_controller::load_configuration`,
arq_common.cc) AND the PHY (`cl_telecom_system::load_configuration`, telecom_system.cc:8712).
A 15→15 FULL↔RECLAIM grid switch was SILENTLY DROPPED by BOTH, because the config index
is unchanged and the modulation/preamble/ldpc-rate reinit triggers don't fire on a grid-only
change. FIX (both layers): a `current_grid` tracker per layer + the early-return falls through
when `effective_grid != current_grid`; the PHY additionally FORCES
`reinit_subsystems.{telecom_system,data_container,ofdm,psk,pre_equalization_channel}=YES`
on a grid change so the new Ngi/Dy/Nsymb lattice + buffer geometry actually install. INV-10
(use_last_good timing invalidation) is satisfied for free because the now-reached tail of
load_configuration already resets `delay_/freq_offset_of_last_decoded_message`
(telecom_system.cc:9591-9596). This is the canonical §7 H2/H3 hazard — predicted by the audit,
caught fail-before by the paired test, fixed at root cause.

**Test evidence:** `--test-se-reclaim` (6 suite tests: wire round-trip/legacy, materializer
FULL/PILOTS/FULL-ab/nData-400, gate safety×3, sync-margin), `--test-se-reclaim-wire`
(producer + role-reversal), `--test-se-reclaim-transition` (H0/H2-H3/H5/INV-5/H6/gate).
Fail-before/pass-after confirmed at every stage; full `mercury --test` green (56/0); default-off
CFG15 BER bit-identical to `d364fcf` (`6;0.0474205 8;0.00607659 10;0` both).

**Sync-margin result (Stage 4, honest):** at 20 dB SNR3k clean AWGN loopback, Ngi=18
RECLAIM-FULL locks 12/12 (= Ngi=54) with mean_H 0.978 vs 0.998 — WITHIN the acceptance band,
so the 97-tap TX FIR (> 72-sample Ngi=18 GI budget, INV-9) does not degrade clean-front lock.
SHIP TARGET stays RECLAIM-PILOTS (1.08x, Ngi=54, zero sync risk); RECLAIM-FULL (1.224x) is
loopback-sync-safe but held behind the flag + bench (the verdict knee collapses Ngi=18 on a
moderate fade).

**Still open / held [?]:** Stage-6 optimizer rate-table election + the RSP-forward ACK-suffix
transport (both behind the default-off gate, bench-gated on the EsN0↔SNR3k mapping); the
end-to-end pinned-CFG15 RECLAIM-vs-FULL A/B with compress-ON (the realized e2e wire vs 3826
after ARQ/turnaround overhead).
