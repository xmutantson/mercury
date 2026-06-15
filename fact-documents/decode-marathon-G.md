# Decode marathon — LEVER G: fixed-point (int16) min-sum — fact document

Branch `feat/decode-marathon` (stacks on levers E `@d26de02` + D `@abf3992` +
C `@4ad4f70`). Worktree `C:/Users/kamer/mercury_wt/decode-marathon`.
Status: **IMPLEMENTED + default-off byte-identical (env-off SPA render md5
`4575011ba38e23575ef3f3bbce0703b2` == base @d26de02). `--test` +
`--test-climb-engine` + `--test-decode-marathon` green BOTH states. See §6.**

## §0 The lever

LEVER G quantizes the **min-sum** decode STATE — the messages `R`/`Q` and the
a-posteriori APP — to **int16** with a fixed LLR scale (Q-format) + saturation, so
the check-node / var-node / syndrome run in saturating integer arithmetic instead
of double. It engages ONLY on the min-sum path (lever E): the min-sum check node
(`ms_check_row`) uses NO transcendentals — only compare / min / add / sign — so it
quantizes cleanly to integer. The SPA tanh/atanh kernel does NOT (it needs float),
so fixed-point is gated `minsum && MERCURY_LDPC_FIXEDPOINT`.

Motivation = SIMD width on the Pi5/A76: int16 packs **8 lanes per 128-bit NEON
register vs 4 for float**, ~2× the decode throughput on the iteration-bound LDPC
hot loop. The Windows build here proves the QUANTIZATION CORRECTNESS (BER parity);
the actual NEON speedup is Pi-only.

Standard result (cited, not invented): a well-scaled fixed-point (int16, even
int8) normalized/offset min-sum is within **~0.1 dB** of float min-sum.

### References
- T. Zhang, Z. Wang, K. K. Parhi, "On finite precision implementation of low
  density parity check codes decoder," IEEE ISCAS 2001, vol.4 pp.202-205 —
  finite-precision BP quantization, the few-bit-LLR result.
- J. Chen, A. Dholakia, E. Eleftheriou, M. P. C. Fossorier, X.-Y. Hu,
  "Reduced-complexity decoding of LDPC codes," IEEE Trans. Commun.
  53(8):1288-1299, 2005 — quantized normalized/offset min-sum, the scale +
  uniform-quantizer treatment.
- A. Inan (xdsopl), https://github.com/xdsopl/LDPC — open-source saturating
  fixed-point reference (`code_type = int8_t`, `FACTOR` scale; testbench.hh): the
  channel LLR is the float LLR × a constant FACTOR, clamped to the integer range,
  and NMS/OMS/SCMS run in saturating integer arithmetic.
- AFF3CT (aff3ct.github.io) — production quantized min-sum BP with a fixed-point
  Q-format channel LLR + saturating message arithmetic.

## §1 SCALE / Q-FORMAT — the quantization design (the heart of the lever)

The float SPA/MS path caps the per-edge check→var message magnitude at the SPA
saturation ceiling `2·atanh(0.9999999) ≈ 16.6355` (`ms_check_row`'s `MS_MAG_MAX`),
so the natural LLR dynamic range of interest is `≈[−16.64, 16.64]` plus the raw
channel LLR (which can spike larger early).

| knob | env | default | meaning |
|---|---|---|---|
| scale **S** | `MERCURY_LDPC_FIXEDPOINT_SCALE` | **64** (Q9.6, 6 frac bits, res 1/64 ≈ 0.0156) | float LLR × S → int16 |
| sat cap | `MERCURY_LDPC_FIXEDPOINT_SAT` | **4096** fixed units (= ±64.0 LLR) | every quantized value clamped to ±cap |

- **Why S=64.** `16.6355 × 64 ≈ 1065 ≪ 32767`, so int16 has ample headroom for
  both messages AND the var-node APP sum. Resolution 1/64 ≈ 0.0156 LLR is fine
  for a min-sum decoder (literature 4–6-bit LLRs suffice; we are generous).
- **Why cap=4096.** It (a) bounds the var-node int32 accumulator: the running sum
  over ≤ dc(46) messages is ≤ 46·4096 ≈ 188k, never overflowing int32 before being
  clamped back to ±cap; (b) keeps a degenerate degree-1 / unset-min2 row from
  injecting a runaway value, exactly like the float `MS_MAG_MAX` guard. The int
  min-sum magnitude ceiling is `fp_mag_cap = min(cap, round(S·16.6355))` so the
  int kernel shares the float kernel's dynamic range.
- **Where the float↔int16 conversion sits.** float→int16 ONCE at decode entry
  (`LLR16q[v] = fp_quantize(LLRi[v], S, cap)`, round-half-away-from-zero); the
  whole BP loop is integer; int16→float ONCE per iteration into `LLRtmp = L16/S`
  so the shared epilogue (`LLRo`, `app_llr`) is uniform across all kernels.

Empirically (cliff cell chan0/seed12345/esn0=15, §6): float MS 17/62, **int16
default S=64 15/62** (within ~2 cw on the worst cell = the ~0.1 dB result),
coarse S=2/cap=8 collapses to 3/62 (proves the scale knob is live + correctly
wired). Fleet-wide int16 is at parity (NMS 412 vs float 413; SCMS 429 vs 428).

## §2 DESIGN — the int16 kernel

`ms_check_row_i16(q16[], nv, alpha_q, variant, S, ms_mag_cap, rout16[])`
(ldpc_decoder_SPA.cc, the integer analogue of `ms_check_row`):
- Two-smallest-magnitude trick in pure int: `min1`/`min2`/`imin1` + `neg_parity`,
  leave-one-out magnitude = `min1` except the owner of `min1` gets `min2`,
  leave-one-out sign = `neg_parity ⊕ sign(q16[k])`.
- **NMS**: `rout = round( (alpha_q · mag) / S )` with `alpha_q = round(alpha·S)`
  the Q-format NMS scale (one int multiply + a divide-by-S round-to-nearest).
- **OMS**: `rout = sign · max(mag − alpha_q, 0)` (`alpha_q` reused as the offset
  beta in fixed units).
- result clamped to `±ms_mag_cap`.

**Var-node + APP (int32 accumulate, saturate on store):**
- FLOODING: `app = LLR16q[v] + Σ_j R16[v][j]` in int32, `fp_sat` to ±cap → `L16`;
  next-iteration `Q16 = fp_sat(L16 − R16)`.
- LAYERED (composes with D): incremental fold `L16[v] = fp_sat(L16[v] + Rnew −
  Rold)`; extrinsic `q = fp_sat(L16[v] − R16[v][vi])`.
- Hard decision = `L16[v] < 0`; syndrome on that hard decision (kernel-agnostic).

**SCMS (composes with D, layered only):** reuses `Q16` as the prior-message store
(the layered kernel never re-reads it for the message itself), seeded to `LLR16q`,
erasing a var→check message whose sign flipped — the int16 mirror of the float
SCMS in lever E §2. Under FLOODING, SCMS degrades to NMS (same constraint as E).

**Memory:** `R16`/`Q16` are `std::vector<short>` sized `N·VWidthMax` at runtime,
allocated ONLY when the lever is engaged → **zero allocation on the default-off
path**. `L16`/`LLR16q` are `vector<short>` of N. Per-edge scratch `fb_q16` /
`fb_rout16` are `short[CW_SCRATCH=64]` on the stack.

## §3 PRODUCERS / CONSUMERS — the state lever G touches

Lever G lives ENTIRELY inside `decode_SPA`. It reads the SAME inputs the float
min-sum path reads (`LLRi`, `C`/`V`/`V_pos`/`d`) and writes the SAME outputs
(`LLRo`, optional `app_llr`, the return iteration count / FAIL sentinel). Its int16
state (`R16`/`Q16`/`L16`/`LLR16q`) is FUNCTION-LOCAL — no new cross-call /
cross-thread shared state. The float `R`/`Q` heap workspace is zeroed at entry but
otherwise UNUSED on the fixed-point path (the int16 mirrors carry the messages).

## §4 COMPOSITION — how lever G stacks with E / D / #3 / B / C

- **E (min-sum):** REQUIRED. Gate is `minsum && MERCURY_LDPC_FIXEDPOINT`. Off the
  min-sum path, fixed-point is a no-op (SPA stays float, bit-for-bit).
- **D (layered):** the int16 path branches on `layered` exactly like the float
  path — there is a fixed-point flooding kernel AND a fixed-point layered kernel
  (incremental APP fold). SCMS pairs with layered, same as E.
- **#3 (syndrome early-term):** `spa_nonconverge_detect` reads the int16 APP hard
  decision + `nOnes`, computed after the int check-node update. The early-term
  sentinel (`return nIteration_max+1`) + `out_early_term_iter` are kernel-agnostic.
- **B (iter-cap):** the loop / `nIteration_max` are outside the kernel — untouched.
- **C (decode pool):** the pool runs whole `decode_SPA` calls on private `cl_ldpc`
  workspaces; lever G is a pure in-function change with no new cross-call state
  (the int16 buffers are function-local). `--test-decode-marathon` passes
  byte-identical parallel==serial WITH fixed-point enabled (§6).

## §5 LIMITS / not-yet-done

- Fixed-point is **lossy by construction** (quantization on top of the min-sum
  approximation). The default S=64/cap=4096 is at-parity-or-better on the measured
  fleet; a FULL fleet BER A/B + a scale/cap sweep across the campaign grid (and the
  NEON-speed measurement on the Pi) is the follow-on quality gate before any
  default-on consideration. Ships **env-gated, default-off**, per the standing
  per-fix-proof authority.
- The ~2× NEON-lane speedup is **Pi-only-confirmable** — Windows proves the BER
  correctness, not the SIMD throughput (no vectorized int16 inner loop is written
  yet; that is the Pi-build follow-on, the int16 state is the prerequisite).
- SCMS-under-flooding degrades to NMS (§2) — inherited from lever E, acceptable.

## §6 §3 proof (CLAUDE.md §3, fail-before / pass-after)

Vehicle = the same 27-cell coded SFO-GRID fleet as levers E/D/C
(`-m PLOT_PASSBAND -s 16`, `MERCURY_SFO_GRID_CODED=1 NSYMB=600`,
CHAN{0,1,2}×SEED{12345,999,77}×ESN0{15,16,17}). Driver
`tools/test_decode_marathon_fixedpoint.py`.

- **TEST-1 default-off byte-identical**: env-off 27-cell render md5
  `4575011ba38e23575ef3f3bbce0703b2` == base @d26de02 (== documented base
  @4ad4f70/@943a083). The SPA tanh/atanh path is bit-for-bit. PASS.
- **TEST-2 int16 decodes, no clean-cell regression vs FLOAT min-sum**: across the
  cells where float min-sum fully converges, the int16 decode keeps full decode —
  NMS-flooding 0 cells regressed (fleet 412 vs float 413), SCMS-layered 0 cells
  regressed (fleet **429 vs float 428** — int16 +1). PASS.
- **TEST-3 quantization parity (the ~0.1 dB claim)**: int16 fleet total within the
  small slack of float min-sum — NMS 412 ≥ 413−10, SCMS 429 ≥ 428−10. SCMS-layered
  int16 EXCEEDS float (quantization acting as mild message regularization). PASS.
- **Scale-knob sensitivity (negative control)**: cliff cell — default S=64 15/62
  ≈ float 17/62, but coarse S=2/cap=8 collapses to 3/62 ⇒ the scale is live and
  correctly wired (under-quantizing destroys decode, as the literature predicts).
- **Composition gates** (BOTH the default-off AND the
  `MERCURY_LDPC_MINSUM=1 MERCURY_LDPC_FIXEDPOINT=1` state, and the
  `… MERCURY_LDPC_LAYERED=1 MERCURY_LDPC_MINSUM_VARIANT=scms` state):
  - `--test` exit 0 (57 passed / 0 failed, all suites).
  - `--test-climb-engine` exit 0.
  - `--test-decode-marathon` (lever-C pool integrity) ALL PASS — parallel==serial
    byte-identical with fixed-point enabled, fail-before defeat still diverges ⇒
    fixed-point composes with the pool.
