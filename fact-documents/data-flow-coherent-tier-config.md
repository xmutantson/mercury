# data-flow — Coherent QPSK low-rate tier (faded-front lever) + §5 audit

Branch `feat/coherent-tier` off `monitor` (627c370). Worktree
`C:/Users/kamer/mercury_wt/coherent-tier`. Research motivation:
`_research/COHERENT_TIER.md` (CONDITIONAL-GO; FreeDV DATAC3/DATAC4 existence
proofs). Every claim cites `file:line` from the executed worktree tree, or a
MEASURED SFO-GRID coded-BER cell run on `mercury_coherent.exe` (o3 build of
627c370, source byte-identical to monitor at the time the cells were grounded).

## §0 What this build IS (and is NOT)

**HEADLINE: the coherent-QPSK low-rate tier ALREADY EXISTS as production
CONFIG_7/8/9.** This work does NOT invent a constellation, an LDPC code, a demod
path, or a CONFIG row. It REGISTERS the existing tier for the faded front and adds
the mandatory paired regression test. The genuinely new artifacts are:
1. `--test-coherent-tier` (in-process fail-before/pass-after, cells A–D).
2. `effective_rate_table.coherent_faded.json` — a synthetic faded-front optimizer
   table the bench/sim loads via `MERCURY_RATE_TABLE` (the env-gated registration
   vehicle). It is NOT the production table; the production
   `effective_rate_table.json` is UNCHANGED → the live optimizer never auto-elects
   the QPSK tier on a real faded channel until faded-front bench validation lands.
3. This fact-document.

**No DSP invented. No production default behavior changed.** Verified
byte-identical: `git diff 627c370 HEAD` over `source/` + `include/` is empty for
all PHY/ARQ files (the only committed changes are the new test in `main.cc`, the
new JSON, and this doc).

## §1 The tier already exists (CONFIG_7/8/9), MEASURED

`load_configuration(int)` (`source/physical_layer/telecom_system.cc:9692`) already
defines, in production:
- **CONFIG_7** = `MOD_QPSK`, `_ldpc_rate=5/16` (=0.3125) — the DATAC4-class deep
  forward tier (`telecom_system.cc:9768-9774`).
- **CONFIG_8** = `MOD_QPSK`, `_ldpc_rate=6/16` (`telecom_system.cc:9775-9781`).
- **CONFIG_9** = `MOD_QPSK`, `_ldpc_rate=8/16` (=1/2) — the DATAC3-class forward
  tier (`telecom_system.cc:9782-9788`).

LDPC matrix selection is by `K` (`source/physical_layer/ldpc.cc:187-216`):
`K=500 → mercury_normal_QCmatrixC_5_16` (CONFIG_7, N=1600, r5/16);
`K=800 → mercury_normal_QCmatrixC_8_16[800][12]` (CONFIG_9, N=1600, r1/2). Both
matrices are #included in `include/physical_layer/mercury_ldpc.h:27-36` and are
prior art inherited from the upstream Fadi Jerji design (N=1600, lifting 100) — no
new code. The N=1600 codeword divides the QPSK harness grid (nData=2000, log2M=2,
nBits=4000) into exactly **Kcw=2** codewords (`telecom_system.cc:6522`,
`Kcw = nBits/ldpc.N`), MEASURED.

PHY params, MEASURED from the PHY-active print on CONFIG_9:
`M=4 LDPC_rate=0.500 BW=2344Hz Nc=50 Nsymb=24 nBits=1600`. SFO-GRID grid print:
`cfg=9 M=4 Nsymb=60 Nc=50 Dx=1 Dy=3 nData=2000 nBits=4000 pilots=1000
(33.33% of grid)`. Pilots Dx=1/Dy=3 (`telecom_system.cc:4824-4837`) → 33% pilot
density, DENSER than FreeDV 700D's 1-in-8. GI=54/256 (physical_config.cc) covers
both CCIR delay profiles (2 ms MPP, 4 ms MPD).

## §2 The faded-front lever, GROUNDED (failing-test-first)

SFO-GRID coded-BER harness: `MERCURY_SFO_GRID=1 _CODED=1 _CHAN=3` (Watterson
2-path Gaussian-Doppler, `telecom_system.cc:6714-6800`) via `-m PLOT_PASSBAND -s`.
Channel knobs `WATT_DEPTH_DB / WATT_FD_HZ / WATT_DLY` (passband samples;
2 ms→96, 4 ms→192 at Fs=48 kHz). Profiles: MPP-canon = depth6/fd1/dly96;
MPD = depth6/fd2/dly192. Success signal: `[SFO-GRID-CODED] codewords_decoded=X/Kcw`.

### §2.1 MPP-canon @ EsN0=6 dB, 6 seeds (12345..67890) — UNANIMOUS

| config | modulation/rate | codewords decoded | verdict |
|---|---|---|---|
| CFG15 | 16-QAM r0.875 | **0/5** every seed | FAIL |
| CFG16 | 32-QAM r0.875 | **0/6** every seed | FAIL |
| CONFIG_9 | QPSK r1/2 | **2/2 BER0** every seed | PASS |

### §2.2 GENIE bound @ EsN0=6 dB MPP-canon (exact channel handed to EQ)
CFG15 = 0/5 (genie BER 0.168), CFG16 = 0/6 (genie BER 0.225) — STILL FAIL with
PERFECT CSI. CONFIG_9 = 2/2 with the PRODUCTION LS estimator (no genie needed).
→ the CFG15/16 failure is FUNDAMENTAL to the constellation+rate on this fade, NOT
an estimator deficiency. This is the §2(b)+(c) mechanism of COHERENT_TIER.md:
lower QAM order + lower code rate = fade-riding. The lever's gain is the CODE/
CONSTELLATION, not a new estimator.

### §2.3 CONFIG_9 EsN0 waterfall, MPP-canon, 6-seed aggregate cw
EsN0=-2 → 0/12; 0 → 0/12; 2 → 2/12; 4 → 9/12; **6 → 12/12**. PER~0.10 crossing
(DATAC3-equivalent 90/100) ≈ EsN0 4–5 dB.

### §2.4 Deep tier CONFIG_7 (QPSK r5/16, DATAC4-class)
MPP-canon EsN0 sweep, 6-seed aggregate: -4→0/12; -2→0/12; 0→3/12; **2→12/12**;
4→12/12. CONFIG_7 crosses ~2–4 dB BELOW CONFIG_9 (lower code rate buys deeper
fade) — confirms INV-2 (rate order = index order: CONFIG_7 r5/16 < CONFIG_9 r1/2).
MPD (4 ms/2 Hz) @ EsN0=4, seed 12345: CFG15 0/5, CFG16 0/6 (fail), CONFIG_9 1/2
(partial), **CONFIG_7 2/2** (deep tier rides the harder fade).

### §2.5 Clean no-regression
CONFIG_9 on flat channel (CHAN=0) decodes 2/2 at EsN0 = 2/4/6 — does NOT regress
on the clean front.

### §2.6 EsN0 ↔ SNR3k mapping (OPEN, honest caveat)
The harness EsN0 is per-subcarrier Es/N0 with Es=1 (`telecom_system.cc:6872-6883`),
NOT SNR(3 kHz). The DATAC3 "~0 dB MPP" / DATAC4 "~-4 dB" anchors are SNR(3 kHz).
First-order: SNR(3k) = EsN0 + 10·log10(BW_occ/3000) = EsN0 + 10·log10(2344/3000)
≈ EsN0 − 1.07 dB (bandwidth scaling only; pilot/CP overhead not folded in). The
ABSOLUTE-dB DATAC parity claim therefore carries this ±~1–2 dB calibration
uncertainty and is stated as APPROXIMATE. The failing-test-first DELTA (§2.1/§2.2)
is calibration-INDEPENDENT: at identical EsN0 on identical MPP, CFG15/16 floor 0/N
while QPSK decodes — that is the lever's existence proof and it does not depend on
the SNR3k mapping.

## §3 §5 cross-layer data-flow audit

**Shared state touched:** the set of configs the optimizer may ELECT on a faded
channel label (via an env-gated alternate `MERCURY_RATE_TABLE`). NO new
default-selected config, NO new struct fields, NO production-table edit.

### §3.1 Producers
- `load_configuration(int)` (`telecom_system.cc:9692`) — sole PHY config producer;
  CONFIG_7/8/9 paths already exist (`:9768-9788`). UNCHANGED.
- `effective_rate_table.json` (production, loaded `arq_common.cc:4050-4051`) —
  UNCHANGED. The synthetic faded table is loaded ONLY via the `MERCURY_RATE_TABLE`
  env override path (`arq_common.cc:4046-4048`), which precedes the production
  paths. So the bench/sim opts in explicitly; the live link never sees it.
- `arq_commander.cc` climb/demote via `config_ladder_up/down_n`
  (`common_defines.h:198-246`). UNCHANGED. CONFIG_7/8/9 are already in
  `FULL_CONFIG_LADDER` (`common_defines.h:172-177`, indices 10–12) and already
  ≤ `WB_CONFIG_MAX=CONFIG_16` (`:168`) → already reachable by the auto-climb,
  same as today.

### §3.2 Consumers
- receive-PHY demod/decode (`telecom_system.cc:2914-3004`) — config-agnostic;
  CONFIG_9 QPSK already flows here (MEASURED, §2). UNCHANGED.
- optimizer rate comparison (`rate_optimizer.cc:632-652`,
  `get_current_effective_rate_bps`). See §3.4.
- gearshift (`common_defines.h:198-246`). UNCHANGED.
- D3 demote-gate (`arq_commander.cc:3716`, `if(current_configuration==CONFIG_16)`).
  See INV-1.

### §3.3 Valid / pre-write states
Before any faded calibration, the production table has NO MPP/MPD rows and no
CONFIG_7 row at all (`configs_tested=[6,9,11,12,13,14,15,16]`,
`channels_tested=[clean,wgn30..wgn16]`, MEASURED from
`effective_rate_table.json`). The optimizer SKIPS missing (config,label) cells:
`get_cell()` returns NULL → `cand_cell` invalid → `continue`
(`rate_optimizer.cc:640-641`). So with the production table the QPSK tier is never
auto-elected — additive and SAFE. Default-init is the "no faded row" state.

### §3.4 Invariants + verification
- **INV-1 (D3 gate is CFG16-specific):** the D3 demote keys on
  `current_configuration==CONFIG_16` (`arq_commander.cc:3716`) and demotes FROM
  CFG16 TO CFG15. CONFIG_7/9 are LOW gears D3 demotes TOWARD, not FROM → electing
  them does NOT touch the gate. Unlike CFG17 (a TOP gear that needed the gate
  extended), **NO `||==CONFIG_x` extension is needed.** VERIFIED.
- **INV-2 (ladder index ~ rate order):** CONFIG_7 r5/16 < CONFIG_8 r6/16 <
  CONFIG_9 r1/2 in both ladder index and code rate; monotone preserved. VERIFIED
  by code (`:9768-9788`) and by the EsN0-crossing ordering (§2.4).
- **INV-3 (table cell = MEASURED effective rate):** the synthetic faded table is
  derived from the §2 SFO-GRID measurements (decode/no-decode at EsN0), NOT
  fabricated. It is a SIM/BENCH vehicle, explicitly labelled synthetic, and the
  production calibrator path (faithful Watterson bench calibration) remains the
  source of any row that would ever enter the production table. The bench MUST
  re-measure on hardware before a production row is written.
- **INV-4 (estimator seed gate must match optimizer channel label):** the
  TINTERP-seed (`dd_seed_floor`) REGRESSES on clean and helps on dispersive
  (CFG17 finding, confirmed for QPSK in §2.2 — QPSK fully decodes MPP WITHOUT the
  seed). For the SIM vehicle the seed is OFF (the QPSK tier decodes the faded front
  on the plain LS estimator). If a future build gates the seed ON for the faded
  label, it MUST consume the SAME channel label the optimizer uses, not an
  independent SNR threshold. NOTED for the follow-on; NOT changed here.

### §3.5 What this change alters, consumer walk
Only the OPTIONAL env-gated table the optimizer reads when `MERCURY_RATE_TABLE`
points at the synthetic faded file. receive-PHY unaffected (config-agnostic, live);
optimizer gains a faded preference ONLY under the env opt-in (additive, INV-3);
gearshift topology unaffected (INV-2); D3 unaffected (INV-1); the estimator seed is
the one cross-layer edge and is held OFF (INV-4). No consumer assumption is
violated.

## §4 Paired regression test
`--test-coherent-tier` (`source/main.cc`, modeled on `--test-cfg17` `:386-524`):
runs the SFO-GRID harness in-process for cells A–D and asserts the QPSK tier
decodes where the QAM gears fail. Fail-before/pass-after: gate behind
`MERCURY_COHERENT_TIER_FAILBEFORE` (forces a too-low EsN0 so the QPSK arm cannot
decode → the assertion fails before the lever is "armed"). Plus `--test-climb-engine`
must stay ALL PASS (the tier registration does not perturb the gearshift, since
CONFIG_7/9 were already in the ladder).

## §5 Ready-for-bench
The faded-front bench validation (reproduce DATAC3/DATAC4 on the IONOS Watterson
emulator, confirm the optimizer elects the QPSK tier on a real MPP/MPD label, and
measure the throughput-vs-MFSK win) queues later and is owned by a separate agent
with the bench lease. This branch is sim-proven and default-not-selected: safe to
hold and merge independently.
