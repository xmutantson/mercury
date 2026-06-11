# data-flow — CONFIG_17 shaped-64-QAM gear (composition + §5 audit)

Branch `feat/cfg17` off `monitor` (10729d4, which carries feat/pcs PAS machinery
8e260d5 TINTERP-seed + 49e204a PCS). Worktree `C:/Users/kamer/mercury_wt/cfg17`.
Dossier `bigblock_p3_hw/_deepdsp/RESEARCH_cfg17-64qam.md`; PCS verdict
`bigblock_p3_hw/_pcs/PCS_VERDICT.md`. Every claim cites file:line from the
executed worktree tree.

## §0 What this build IS (and is NOT)

It COMPOSES three already-merged deep levers into ONE shaped-64-QAM decode gear,
and adds a CONFIG_17 production config SKELETON that is **DEFAULT-NOT-SELECTED**
(the optimizer/gearshift never auto-elects it). It does NOT invent DSP.

The three composed levers, all already in the tree:
1. **PAS** (Probabilistic Amplitude Shaping): `cl_dist_matcher`
   (`source/physical_layer/dist_matcher.cc`), `cl_psk::demod_pas` +
   `cl_psk::rescale_shaped_power` (`source/physical_layer/psk.cc`). Wired in the
   SFO-GRID harness `pas_on` path (`telecom_system.cc:6539`, decode 7156-7182).
2. **TINTERP-seed turbo estimator**: `LS_channel_estimator_tinterp` (it=0 seed) +
   `data_aided_channel_estimator` + `dd_seed_floor` + nv-warm-anchor. Wired in the
   harness via `MERCURY_SFO_GRID_TURBO_SEED=tinterp` →
   `turbo_seed_tinterp`/`ofdm.dd_seed_floor` (`telecom_system.cc:6341-6343,7224`).
3. **ratio-nvfix**: `demap_variance = (nv < measure_var/K) ? measure_var : nv`,
   K=8 — held on `fix/cfg16-nvfix@a0e22c8`, in production at
   `telecom_system.cc:2992-2999`. **The one piece NOT yet wired into the harness
   decode path** (the harness used `cvar = noise_variance_estimate` raw at 7179).

## §1 The CFG17 config skeleton

`CONFIG_17` = MOD_64QAM, LDPC rate-14/16 (0.875), preamble Nsymb=4,
LEAST_SQUARE estimator — identical to CFG15/CFG16 except `_modulation=MOD_64QAM`
(`telecom_system.cc:9809-9815` CONFIG_16 is the template). Reuses the existing
rate-14/16 QC-LDPC matrix (no new matrix → no ABI/build risk). The OFDM
dimensioning for MOD_64QAM ALREADY EXISTS (`telecom_system.cc:4809,4818` Nsymb=8;
:4829 Dx=1; :4841,:4850 Dy=3) and the 64-QAM constellation ALREADY EXISTS
(`psk.cc:159-225`, `MOD_64QAM=64` `psk.h:34`). Production follow-on noted in code:
rate-2/3 QC-LDPC PAS (canonical), HELD until after the CFG16-acquisition fix.

### §1.1 Why WB_CONFIG_MAX is NOT bumped (the default-not-selected mechanism)

The TASK requires CFG17 be reachable ONLY via the harness/env, NOT auto-elected.
The gearshift/optimizer ceiling is `WB_CONFIG_MAX` (`common_defines.h:150`), read
by `config_ladder_up/up_n/is_at_top` (`common_defines.h:181-238`) and the ARQ
ceiling/turbo-cap (`arq_commander.cc`). **Leaving `WB_CONFIG_MAX=CONFIG_16`
guarantees the climb engine never steps to CFG17**: `config_ladder_up(16,…)`
returns 16 (`:183` `config<ceiling` is false at 16). So no D3 demote-gate change,
no optimizer registration, no `FULL_CONFIG_LADDER` append — all HELD for after the
CFG16-acquisition fix (avoids concurrent gearshift changes, per the dossier §4.5).

### §1.2 The ONE table-size guard that MUST change, and why it is safe

`load_configuration` rejects `configuration >= NUMBER_OF_CONFIGS`
(`telecom_system.cc:9677`). With `NUMBER_OF_CONFIGS=17`, `-s 17` is rejected
before the config branch. To make CFG17 reachable AT ALL (via `-s 17` and the
SFO-GRID harness), `NUMBER_OF_CONFIGS` must become 18.

**Side-effect audit of NUMBER_OF_CONFIGS (grep, 22 sites):**
- `arq.h:3214` `monitor_decoders[NUMBER_OF_CONFIGS]` + `arq_common.cc:1660-1731`
  parallel-decode loops `for(cfg=0;cfg<NUMBER_OF_CONFIGS;cfg++)`: these run ONLY
  in MONITOR_MODE (a separate operation mode, `main.cc` MONITOR_MODE=9). Growing
  the array to 18 makes MONITOR init one extra (CFG17) decoder. This is the
  ONLY behavioral change from the bump, and it is **confined to MONITOR_MODE**,
  which is not the ARQ production path and not exercised by `-m PLOT_PASSBAND`
  (the BER/SFO-GRID vehicle) nor by `-m ARQ` loopback. The CFG0-16 production
  decode/encode paths are byte-identical.
- `main.cc:2129,2140` config-table walk + range guard: walking to 17 prints the
  CFG17 row; the range guard `mod_config>=NUMBER_OF_CONFIGS` now admits `-s 17`
  (the INTENDED reachability). No CFG0-16 change.
- `arq_common.cc:9507` `(cur+1)%NUMBER_OF_CONFIGS`: a monitor-decoder rotation;
  MONITOR-only.
- `telecom_system.cc:9677`: the guard being relaxed — intended.

Net: the bump's only non-CFG17 effect is the MONITOR_MODE decoder array. The
ARQ/PLOT_PASSBAND/SFO-GRID production paths are byte-identical with CFG17 unused.
Documented as the accepted, scoped cost of making CFG17 *reachable* — it does NOT
make CFG17 *elected* (that is WB_CONFIG_MAX, untouched).

## §2 The ratio-nvfix composition into the harness decode path

The harness coded decode (`telecom_system.cc`) demaps with
`cvar = ofdm.noise_variance_estimate` raw (the old :7179) and again on the turbo
feedback pass (old :7315). The production ratio-nvfix
(`telecom_system.cc:2992-2999`) gates `demap_variance = measure_var` only when
`nv < measure_var/K` (K=8). `measure_variance(rx.data())` is ALREADY computed in
the harness as `variance` (:7102) — exactly the `measure_var` the gate needs.

**Composition (default-OFF, env `MERCURY_SFO_GRID_NVFIX`):** when set, the harness
applies the SAME ratio-gate to `cvar` (and `cvar2` on the turbo pass) using the
already-computed `variance`. Default unset → `cvar` is the raw nv, byte-identical
to base. The gate is a NO-OP whenever `nv >= variance/8` (true in-sim, where the
~1000× HW collapse never reproduces — PCS_VERDICT.md, nvfix commit a0e22c8) — so
even ENABLED it is a sim NO-OP UNLESS a cell is constructed with a collapsed nv.
The failing-test cell (§3.3) constructs exactly that collapsed-nv regime.

## §3 The failing-first composition self-test (`--test-cfg17`)

A new in-process self-test (`main.cc::run_cfg17_selftest`) drives the SFO-GRID
harness via `setenv` for two decisive cells and asserts the COMPOSED stack
decodes where a BARE arm fails. Each cell runs `sfo_grid_test()` twice
(bare-arm then composed-arm) and parses `codewords_decoded=k/K`.

- **Cell A — clean @ ~15.3 dB (the PAS lever):** uniform-64-QAM (`_M64=1`, no
  `_PCS`) at EsN0=15.5 dB clean (CHAN=0) decodes < K (the uniform-64 waterfall is
  ~17.6 dB, PCS_VERDICT.md), while PAS-shaped-64 (`_PCS=1`) decodes K/K (the PAS
  waterfall ~15.3 dB). This is the §1.1 PCS gain crossing the LDPC waterfall —
  the decisive clean-front cell.
- **Cell B — det-floor via the estimator (the TINTERP-seed lever):** a
  frequency-selective det-floor cell where LS-only (`TURBO_SEED` unset) floors
  (0/K) but the TINTERP-seed turbo estimator
  (`MERCURY_SFO_GRID_TURBO_SEED=tinterp`, `TURBO_ITERS>1`) crosses (k>0). The
  estimator is what closes the estimate→genie gap that PAS alone cannot
  (PCS_VERDICT.md (2): PAS does NOT cross the estimator-limited det-floor).

The test PASSES only if BOTH the bare arm fails AND the composed arm decodes —
failing-first proven in one run. The nvfix is exercised in a THIRD assertion
(Cell C, §3.3) on a collapsed-nv cell so all three composed levers are covered.

### §3.3 The nvfix cell (collapsed-nv regime)

The in-sim nv never collapses, so to exercise the ratio-gate the test forces a
collapsed nv via `MERCURY_SFO_GRID_NV_FORCE` (a harness env that, when set,
overrides `ofdm.noise_variance_estimate` to the given tiny value before the coded
demap — additive, default-unset → byte-identical). With a forced 1e-6 nv the bare
demap (`NVFIX` unset) over-confidently flips inner 64-QAM bits → BP iter-cap →
0/K; with `MERCURY_SFO_GRID_NVFIX=1` the ratio-gate substitutes `measure_var` →
decode recovers. This reproduces the HW collapse mechanism in sim so the gate's
correctness is testable (the gate's *benefit* on real HW remains bench-only, per
PCS_VERDICT/nvfix-a0e22c8).

## §4 Default-off byte-identity proof

CFG0-16 production paths untouched. The only edits that compile into the default
path are: (a) `NUMBER_OF_CONFIGS 17→18` (§1.2 — MONITOR-only behavioral delta,
ARQ/PLOT/SFO byte-identical); (b) the CONFIG_17 `else if` branch
(`telecom_system.cc`, only reached when `configuration==17`); (c) enum/string/
mod-order CFG17 cases (only reached for config==17); (d) the harness `_NVFIX` /
`_NV_FORCE` env gates (default-unset → raw nv, byte-identical). Verified by
`md5sum` of a `-s 16` and `-s 15` PLOT_PASSBAND BER render vs the monitor-base
binary (§6 results).

## §5 Cross-layer audit (CLAUDE.md §5)

Shared state touched: the config/constellation/estimation triple.

### 5.1 Producers (who WRITES the config/constellation/estimation state)
- `cl_telecom_system::load_configuration` (`telecom_system.cc:9670`) writes
  `_modulation`, `_ldpc_rate`, estimator, then `M`, `ldpc.rate`, `current/last_
  configuration`. **CFG17 adds one producer branch** (`configuration==CONFIG_17`).
- `config_to_mod_order` (`common_defines.h:280`) — pure map config→{2,4,8,16,32,64}.
  CFG17 adds `return 64`.
- The SFO-GRID harness M64 override (`telecom_system.cc:6362-6366`) writes
  `M=MOD_64QAM` + rebuilds the constellation, restoring at teardown (:7372). This
  producer is UNCHANGED — CFG17 reuses it.
- `psk.set_predefined_constellation` / `rescale_shaped_power` — constellation
  producers, UNCHANGED.
- `ofdm.noise_variance_estimate` producers: `LS_channel_estimator`,
  `data_aided_channel_estimator` (estimation state). The nvfix does NOT write
  `noise_variance_estimate`; it derives a LOCAL `demap_variance`/`cvar` (scope
  preserved exactly as the production a0e22c8 fix: MMSE-ZF erasure + SKIP-VAR sync
  gate read the untouched `noise_variance_estimate`).

### 5.2 Consumers (who READS it)
- The PHY mod/demod loops (`psk.cc:260-334`) — constellation-size-generic, key off
  `M`. CFG17 → M=64 drives them with no edit (dossier §4.2/4.3).
- The OFDM dimensioning (`telecom_system.cc:4625-4684`) reads `M`; MOD_64QAM
  branches present. No edit.
- The optimizer/Q-table (`rate_optimizer.cc`) reads the config ladder + the
  effective-rate table. **CFG17 is NOT registered** (WB_CONFIG_MAX unchanged) →
  the optimizer's candidate set is byte-identical; it never sees CFG17.
- The ARQ ceiling/turbo-cap/D3 demote (`arq_commander.cc:570,4827,4860,4962,5741`)
  read `WB_CONFIG_MAX`/`==CONFIG_16`. UNCHANGED → no CFG17 in any ARQ decision.
  (The `==CONFIG_16 → ||==CONFIG_17` D3 extension is the HELD follow-on.)
- `config_to_string*` / `config_to_short_string*` — display only.
- MONITOR parallel decoders (`arq_common.cc:1676`) read `NUMBER_OF_CONFIGS` — the
  one consumer whose behavior changes (§1.2), MONITOR-only.

### 5.3 Valid states / default-init
- Before any producer writes, `current_configuration=CONFIG_NONE(-1)`; the
  load_configuration `>=NUMBER_OF_CONFIGS` guard (:9677) rejects out-of-range. With
  the bump, 17 is admitted; 18+ still rejected. CFG17 reachable only via explicit
  `-s 17` or the harness M64 override — never via a default/optimizer path
  (WB_CONFIG_MAX bounds every climb).
- The NB clamp (`telecom_system.cc:9683`) clamps `is_ofdm_config(config) &&
  config>NB_CONFIG_MAX` to NB max. **CFG17 is NOT made is_ofdm_config** (the
  `<=16` bound at `common_defines.h:98` is LEFT UNCHANGED) → CFG17 is WB-only by
  construction; an `-N -s 17` would NOT be clamped by the is_ofdm path. Mitigation:
  CFG17 is only ever invoked via the WB SFO-GRID harness / `-s 17` WB; the NB path
  never reaches it (the optimizer can't elect it, and NB caps at CONFIG_14). This
  is the ONE place where not-bumping `is_ofdm_config` is a deliberate scoping
  choice: keeping `is_ofdm_config<=16` means none of the 18+ ladder/break/optimizer
  sites that gate on `is_ofdm_config` (≈30 sites) treat CFG17 as a ladder rung →
  reinforces default-not-selected. (When CFG17 is promoted to a real gear post-
  acquisition-fix, `is_ofdm_config` AND WB_CONFIG_MAX bump together — both held.)

### 5.4 Invariants consumers assume — verified
- "config in [0,WB_CONFIG_MAX] is a climbable OFDM rung": MAINTAINED — CFG17 is
  outside [0,16] for both the ladder (FULL_CONFIG_LADDER unchanged, CFG17 absent)
  and is_ofdm_config (<=16), so no ladder/optimizer consumer treats it as a rung.
- "M ∈ constellation table": MAINTAINED — MOD_64QAM exists and is normalized.
- "demap_variance feeds only the psk.demod LLR calls": MAINTAINED — the harness
  nvfix derives a local cvar exactly as production a0e22c8 (noise_variance_estimate
  untouched → MMSE-ZF + sync gate unchanged).
- "NUMBER_OF_CONFIGS = monitor decoder count = table-walk bound": the bump grows
  both consistently; the table walk and the monitor array stay in sync (both 18).

### 5.5 What the fix changes / per-consumer walk
The ONLY production-path assumption altered is the table-size bound
(NUMBER_OF_CONFIGS), which admits config==17. Every consumer of that bound walked
in §5.2/§1.2: MONITOR grows its array (scoped, MONITOR-only); the `-s` range guard
admits 17 (intended); load_configuration admits 17 (intended); the optimizer/ARQ
ceiling DO NOT read NUMBER_OF_CONFIGS (they read WB_CONFIG_MAX, unchanged) → no
gearshift change. No consumer's invariant is violated on the CFG0-16 paths.

## §6 Results (build o3, monitor base 10729d4)

- **build o3**: clean (73 files, only pre-existing winsock/WASAPI warnings). mercury.exe 34.78 MB.
- **`--test-cfg17` ALL PASS (0 cell failures)** — failing-first proven, each composed lever live:
  - CELL-A PAS clean@16dB CHAN0: uniform-64 **0/7** (bare fails) → PAS-64 **7/7** (composed decodes).
    The PAS shaping gain (~2.3 dB vs uniform-64) crosses the LDPC waterfall.
  - CELL-B estimator det-floor@18dB CHAN1: LS-only **0/7** (bare floors) → TINTERP-seed turbo
    **7/7** (composed crosses). GENIE decodes 7/7 here → the 64-QAM det-floor is
    estimator-limited (not modulation-limited); the TINTERP-seed stack reaches genie-class
    CSI. Reproducible over 4 seeds (LS 0/7, turbo 7/7; post-FEC BER 0.458→0).
  - CELL-C nvfix nv-collapse@16dB: raw-nv **2/7** (bare fails under the forced 1e-6 collapse) →
    ratio-nvfix **7/7** (composed recovers). The ratio-gate substitutes measure_var on the
    catastrophic collapse the HW exhibits and the sim only reproduces via NV_FORCE.
- **`--test-pas` ALL PASS** (6 DM configs, 200k×6 bijection + constant-composition + histogram).
- **`--test-climb-engine` ALL PASS** (0 failures) — gearshift regression intact; CFG17 does NOT
  perturb the climb engine (W0a/W1/W2x all green, the CFG16 D2/D3 turnaround logic unchanged).
- **default-off byte-identical** (grep'd deterministic output md5, cfg17 vs monitor-base 10729d4):
  - `-s 16` BER render: base==cfg17 (17a08c3b…). `-s 15`: base==cfg17 (a4baaebd…). CFG16≠CFG15
    md5 (proves the grep captured real config-distinct output).
  - SFO-GRID default coded path (no M64/PCS/NVFIX): base==cfg17 (edd7eee4…).
- **`-s 17` reachability**: `[PHY] Config 17 active: M=64 LDPC_rate=0.875 Nsymb=8 nBits=1596`,
  per-frame wire 4451.61 bps. `-s 18` correctly rejected (table-size guard at 18).
- **gearshift ceiling unchanged**: WB_CONFIG_MAX=CONFIG_16, is_ofdm_config≤16,
  FULL_CONFIG_LADDER_SIZE=20 (CFG17 absent, 0 occurrences), BREAK_DROP_STEP_MAX=16 →
  CFG17 is structurally default-not-selected (the optimizer/D3 cannot reach it).

### §6.1 Honest scoping note (Phase-3 empirical finding)
On the M64-PCS **full-grid-LS** harness, the plain `LS_channel_estimator` (global LS over the
Nc×Nsymb window) is near-genie on slow/fast Watterson, and the `LS_channel_estimator_tinterp`
seed by itself does NOT beat it there (it can regress). The TINTERP-seed COMPOSED win is on the
**freq-selective det-floor (CHAN=1)** where the cold flat-ML/LS H cannot represent the dispersive
per-subcarrier phase — that is the cell where LS floors 0/7 and the TINTERP-seed turbo crosses
7/7 (CELL-B). This matches PCS_VERDICT (2): the 64-QAM det-floor is estimator-limited (genie
decodes), and the estimator lever's job is to reach genie-class CSI — which CELL-B demonstrates.

## §7 Fleet validation sweep (vs genie, across channels) — bigblock_p3_hw/_cfg17/CFG17_VALIDATE.json

Fleet (.31/.21/.11, 56-core each) rebuilt feat/cfg17 @ 26ae773, 1032 cells, 6 seeds/cell,
arms {uniform-64 LS-only / pasls (PAS+plain-LS) / stack (PAS+TINTERP-seed-turbo, nvfix OFF) /
stacknvfix (diag) / genie-PAS}, EsN0 sweeps. Waterfall = lowest EsN0 with ≥99% decode-fraction.

| channel    | uniform | pasls | **stack** | stacknvfix | genie |
|------------|---------|-------|-----------|------------|-------|
| clean      | 19      | 17    | 19        | —          | 16    |
| detfloor   | none    | none  | **18**    | none       | 16    |
| watt-good  | none    | none  | **19**    | —          | 17    |
| watt-mod   | none    | none  | none(0.95@22) | —      | 19    |
| watt-poor  | none    | none  | none(0.29@24) | —      | 23    |

### §7.1 KEY FINDING — the ratio-nvfix MUST be OFF in sim (it MISFIRES on dispersive channels)
On the det-floor the estimator nv is legitimately LOW (0.0093 @18 dB) while the post-EQ
measured `variance` is LARGE (dispersive EVM). The ratio-gate `nv < measure_var/8` therefore
TRIPS and over-softens the LLRs → decode dies. MEASURED (detfloor @18 dB seed12345 TINTERP-
turbo): **NVFIX=0 → 7/7, NVFIX=1 → 0/7**; the `stacknvfix` arm floors 0.0 across ALL 14-22 dB,
all 6 seeds. nvfix is a **HW-ONLY gated lever** (the in-sim nv never collapses); its CORRECTNESS
is proven only via NV_FORCE (`--test-cfg17` CELL-C). Engaging it on a real dispersive sim channel
ACTIVELY HARMS. Matches MEMORY.md "NVFIX SETBACK" + PCS_VERDICT bench-only scoping. **CORRECTION
to the §0/§2 composed-stack default: the deployable SIM stack is nvfix-OFF; nvfix engages only
on the HW post-EQ-EVM collapse (ratio-gate K=8, validated by bench, not sim).**

### §7.2 KEY FINDING — the TINTERP seed is channel-gated (regresses on clean)
clean waterfall: pasls (PAS+plain-LS) = **17 dB** vs stack (PAS+TINTERP-turbo) = **19 dB** — the
TINTERP seed REGRESSES 2 dB on clean. detfloor: pasls = NONE vs stack = **18 dB**. The estimator
lever differentiates ONLY where cold-LS cannot represent the channel (det-floor, fades). The
deployable CFG17 estimator must be channel-appropriate (plain LS on clean/flat where near-genie;
TINTERP-seed turbo on dispersive/fading). Confirms §6.1.

### §7.3 Decisive answers
- **clean**: CFG17 PAS-64 (pasls) full-decodes at **17 dB** (genie 16, CFG16 uniform-32 16 in the
  SAME vehicle) → CFG17 decodes ~1 dB above CFG16's working point carrying +20% payload (the
  PCS 15.3 dB figure is the genie-CSI PAS waterfall; the real-LS-estimator working point is 17).
- **DET-FLOOR (the nv-collapse domain, decisive)**: ONLY the stack decodes (**18 dB**, tracking
  genie 16 within ~2 dB); uniform/pasls/stacknvfix all FLOOR 0/7 across 14-22 dB. **The TINTERP-
  seed turbo CLOSES the 64-QAM det-floor gap to genie where PCS-alone floored** — YES.
- **GOOD fade**: stack full-decodes **19 dB** (genie 17); uniform/pasls floor → YES.
- **MOD fade**: stack reaches 0.95 frac @22 dB but never full-99% in range (genie 19) → PARTIAL,
  ~3 dB estimator gap. **POOR fade**: stack tops at 0.29 @24 dB (genie 23) → the estimator does
  NOT close the gap; the time-varying deep fade outpaces it (the known fade-tier non-beat carries
  to 64-QAM, exactly as hypothesized).

### §7.4 Wire / VARA-beat math
CFG17 = M=64 (6 b/sym) vs CFG16 M=32 (5 b/sym), same Nc=50/BW=2344Hz/rate0.875. Data-symbol
payload edge = **1.197× (+20%)**; per-frame-with-preamble = 1.153× (CFG17 Nsymb=8 < CFG16 9
dilutes vs the fixed 4-sym preamble). CFG17 per-frame wire = 4451.61 bps. × R=3.69 compression:
- sustained conservative (1.153× on bench-8 CFG16 3060) = 3527 × 3.69 = **13,016 eff = 0.998× VARA**
- sustained +20% (1.197×) = 3663 × 3.69 = **13,516 eff = 1.036× VARA (beat)**
- per-frame PHY ceiling 4451.61 × 3.69 = **16,426 eff = 1.26× VARA** (held-at-per-frame)

**HEADLINE: the composed CFG17 stack DECODES across clean/det-floor/GOOD where CFG16 works,
carrying +20% wire — with two load-bearing caveats: (1) nvfix OFF in sim (HW-only gated), (2)
the TINTERP seed channel-gated (clean uses plain-LS PAS). MOD partial / POOR floors (estimator
gap, the carried-over fade-tier non-beat). Sim PASS is necessary-not-sufficient: the det-floor
nv-collapse benefit is bench-only-confirmable (the gated follow-on).**
