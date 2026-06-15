# Decode marathon — LEVER E: min-sum check-node — fact document

Branch `feat/decode-marathon` (stacks on levers D `@abf3992` + C `@4ad4f70`).
Worktree `C:/Users/kamer/mercury_wt/decode-marathon`.
Status: **IMPLEMENTED + default-off byte-identical (env-off SPA render md5
`4575011ba38e23575ef3f3bbce0703b2` == base @4ad4f70). `--test` + `--test-climb-engine`
green both states. See §6.**

## §0 The lever

LEVER E replaces the SPA/BP check-node update — the only place in the LDPC
decoder that calls libm transcendentals — with the **min-sum approximation**.
It is the ONLY lossy lever in the marathon, but it is also a **decode-QUALITY**
candidate: memory `cfg16_decode_loss_is_ldpc_bp` pins the clean ~21% CFG16
decode loss to **BP non-convergence** of Mercury's "naive saturating SPA (NO
damping/normalization/OSD)" — and normalization/self-correction is exactly the
missing ingredient that min-sum adds.

The SPA check node (`ldpc_decoder_SPA.cc`, default path) is
```
R_out[k] = 2*atanh( prod_{m != k} tanh(0.5*Q[m]) )            (SPA, two libm/edge)
```
Min-sum replaces the transcendental product with the magnitude minimum × the
sign product:
```
R_out[k] = ( prod_{m != k} sign(Q[m]) ) * f( min_{m != k} |Q[m]| )   (min-sum)
```
computed in **O(dc)** via the two-smallest-magnitude trick (min1 = smallest |Q|,
min2 = second-smallest, total sign-parity): the leave-one-out minimum is `min1`
for every edge except the edge that OWNS `min1` (gets `min2`); the leave-one-out
sign is `total_parity XOR sign(Q[k])`. No libm; NEON-friendly; ~1 compare + 1
multiply per edge vs the tanh/atanh pair (~3-10× cheaper per iteration).

The normalization `f()` is what restores SPA-like convergence (plain min-sum
OVER-estimates check reliability and converges to a worse floor):

| variant (env `MERCURY_LDPC_MINSUM_VARIANT`) | `f(x)` | note |
|---|---|---|
| `nms` (DEFAULT, value 0) | `alpha * x` | Normalized MS, alpha ~ 0.75-0.875 |
| `oms` (value 1) | `max(x - alpha, 0)` | Offset MS, alpha reused as the offset beta |
| `scms` (value 2) | NMS check node + var-node erasure | Self-Corrected MS |

`alpha` via `MERCURY_LDPC_MS_ALPHA` (default **0.8**, clamped (0,4]). NMS at
alpha≈0.8 is the canonical "keeps SPA convergence" setting (Chen-Fossorier).

### Variant choice + justification
- **NMS is the default.** Density evolution (Chen-Fossorier IEEE Comm. Lett.
  6(5) 2002) shows normalized-BP with alpha≈0.8 sits within a small fraction of
  a dB of SPA on regular/irregular codes, while plain MS loses ~0.5-1 dB and
  converges slower. Default-on behaviour must not regress, so NMS.
- **SCMS is offered as the convergence-RECOVERY variant** (the one aimed at the
  CFG16 BP-non-convergence target). Savin (ISIT 2008) erases unreliable
  (sign-flipping) var→check messages, which makes the check-node messages
  symmetric-Gaussian and recovers **near-SPA** performance at MS complexity,
  independent of noise-variance estimation error. SCMS is a VAR-node
  modification, not a check-node one — see §2 composition with the schedule.

### References (cited, not invented)
- M. P. C. Fossorier, M. Mihaljević, H. Imai, "Reduced complexity iterative
  decoding of LDPC codes based on belief propagation," IEEE Trans. Commun.
  47(5):673-680, 1999 — the min-sum / two-min check node.
- J. Chen and M. P. C. Fossorier, "Near optimum universal belief propagation
  based decoding of LDPC codes," IEEE Trans. Commun. 50(3):406-414, 2002; and
  "Density evolution for two improved BP-based decoding algorithms over AWGN,"
  IEEE Comm. Lett. 6(5):208-210, 2002 — NMS (normalized-BP) / OMS (offset-BP),
  the alpha≈0.8 / beta scaling that recovers near-SPA.
- V. Savin, "Self-corrected min-sum decoding of LDPC codes," IEEE ISIT 2008,
  pp. 146-150 (doi:10.1109/ISIT.2008.4594965, arXiv:0803.1090) — SCMS: erase
  sign-flipping var→check messages ⇒ near-SPA at MS complexity.

## §1 PRODUCERS / CONSUMERS — the state lever E touches

Lever E lives ENTIRELY inside `decode_SPA` (`ldpc_decoder_SPA.cc`). It reads the
same inputs the SPA path reads and writes the same outputs:

**Reads (per check row):**
- FLOODING path: the materialized var→check messages `Q[vj*VWidthMax+vi]`
  (produced at the bottom of the prior iteration, `Q = LLRtmp - R`,
  ldpc_decoder_SPA.cc:~735). Min-sum gathers the RAW `Q` (the SPA path gathers
  `tanh(0.5*Q)`).
- LAYERED path (lever D): the on-the-fly extrinsic `q = L[vj] - R[vj][vi]`
  (ldpc_decoder_SPA.cc:482). Min-sum stores the RAW `q`.

**Writes (per check row):**
- FLOODING: `R[j*VWidthMax+V_pos[...]]` — the SAME check→var cells the SPA path
  writes (ldpc_decoder_SPA.cc:642).
- LAYERED: `R[vj*VWidthMax+vi]` + the incremental APP fold `L[vj] += Rnew - Rold`
  (ldpc_decoder_SPA.cc:520-524) — identical to lever D's SPA write-back.

**SCMS prior-message store (LAYERED only):** SCMS needs the PRIOR iteration's
var→check message to detect a sign flip. In the layered path the `Q[]` workspace
is initialized once (to `LLRi`, ldpc_decoder_SPA.cc:~430) and **never read again**
by the SPA layered kernel, so lever E reuses `Q[vj*VWidthMax+vi]` to stash the
raw prior message (ldpc_decoder_SPA.cc:494-499). This is the only NEW use of
shared state, and it is confined to the SCMS-layered combination.

## §2 COMPOSITION — how lever E stacks with D, #3, B, C

- **D (layered min-sum):** min-sum slots into BOTH check-node kernels. In the
  LAYERED branch it replaces the leave-one-out tanh product, fed the
  layered-fresh `q = L - R`; the incremental APP fold is unchanged. This is the
  intended pairing — "layered min-sum" is a single combined algorithm.
- **#3 (syndrome early-term):** the non-convergence detector
  (`spa_nonconverge_detect`) reads the hard decision of the SAME APP
  (`L`/`LLRtmp`) and the syndrome weight `nOnes`, computed AFTER the check-node
  update regardless of kernel. Min-sum changes the LLR magnitudes but not the
  syndrome-weight semantics, so #3 fires on the min-sum syndrome unchanged. The
  early-term sentinel (`return nIteration_max+1`) and `out_early_term_iter`
  measurement are kernel-agnostic.
- **B (iter-cap):** the iteration loop and `nIteration_max` are outside the
  check-node kernel — untouched.
- **C (decode pool):** the pool runs whole `decode_SPA` calls on private
  `cl_ldpc` workspaces; min-sum is a pure in-function change with no new
  cross-call/cross-thread state (the SCMS `Q` stash is per-`cl_ldpc`, re-zeroed
  at every `decode_SPA` entry, ldpc_decoder_SPA.cc:~222). The lever-C integrity
  test (`--test-decode-marathon`) passes byte-identical parallel==serial WITH
  min-sum enabled — see §6.

### Composition constraint: SCMS requires layered
SCMS is a var-node modification and needs the prior-iteration message store. The
FLOODING path's `Q[]` is actively used (it IS the materialized var→check message
for the next iteration), so there is no free per-edge store without a second
full-size buffer. Rather than allocate one, **SCMS under FLOODING degrades to
NMS** (the check node is identical; only the var-node erasure is unavailable).
SCMS pairs naturally with LAYERED — the convergence-quality variant with the
convergence-acceleration schedule. Documented and enforced at
ldpc_decoder_SPA.cc:635-636 (`eff_variant = (ms_variant==MS_SCMS)?MS_NMS:...`).

## §3 SATURATION / DYNAMIC-RANGE PARITY (robustness)

The SPA path clamps the leave-one-out tanh product to ±0.9999999, so its output
magnitude tops out at `2*atanh(0.9999999) ≈ 16.6355`. `ms_check_row` clamps every
input magnitude AND the unset-`min2` sentinel to the SAME ceiling
(`MS_MAG_MAX = 16.63553233343869`, ldpc_decoder_SPA.cc:~187), so:
1. the two kernels share a dynamic range (no LLR-scale mismatch when a downstream
   consumer compares), and
2. a degenerate degree-1 check row (or any row where `min2` is never set) can
   NEVER inject a runaway `1e300` LLR — it caps at the SPA ceiling. (Mercury's
   matrices have dc ≥ 2, but this is life-critical software; the guard is cheap.)

## §4 WHAT THE FIX CHANGES + default-off proof

With `MERCURY_LDPC_MINSUM` unset/0, `minsum==false`, and the control flow is
`if(minsum){...} else if(!fwdback){ORIGINAL SPA} else {fwd-back SPA}` in BOTH the
flooding and layered branches. The default-off path is the ORIGINAL O(dc²) SPA
tanh/atanh kernel, bit-for-bit. PROVEN: env-off 27-cell SFO-GRID-CODED render
md5 == base @4ad4f70 (§6 TEST-1).

## §5 LIMITS / not-yet-done

- Min-sum is **lossy by construction** (it is an approximation of SPA). The
  default variant (NMS, alpha 0.8) is chosen to minimize the loss; on the
  measured cells it is at-parity-or-better and faster, but a FULL fleet BER A/B
  vs SPA across the campaign grid + the channel-gated alpha sweep is the
  follow-on quality gate before any default-on consideration. This lever ships
  **env-gated, default-off**, per the standing per-fix-proof authority.
- The decode-QUALITY claim (recovering CFG16 BP-non-convergers) is sim-evidenced
  on the cliff cell (SCMS layered 20/62 vs SPA 18/62, §6) but the HW post-EQ-EVM
  nv-collapse interaction is bench-only-confirmable (same caveat as the sibling
  decoder levers).
- SCMS-under-flooding degrades to NMS (§2) — acceptable, SCMS is meant to pair
  with layered.

## §6 §3 proof (CLAUDE.md §3, fail-before / pass-after)

Vehicle = the same 27-cell coded SFO-GRID fleet as levers D/C
(`-m PLOT_PASSBAND -s 16`, `MERCURY_SFO_GRID_CODED=1 NSYMB=600`,
CHAN{0,1,2}×SEED{12345,999,77}×ESN0{15,16,17}).

- **TEST-1 default-off byte-identical**: env-off 27-cell render md5
  `4575011ba38e23575ef3f3bbce0703b2` == base @4ad4f70 (== documented base
  @943a083). The SPA tanh/atanh path is bit-for-bit. PASS.
- **TEST-2 min-sum decodes + no clean regression**: on the converging cells all
  variants keep full decode (clean chan0/seed12345/esn0=17: SPA 62/62 → NMS
  62/62 → SCMS 62/62; mid chan0/seed999/esn0=16: 62/62 across) AND converge in
  FEWER iterations (clean iter_mean SPA 2.58 → NMS 1.68; mid SPA 5.10 → NMS
  3.11; the layered+min-sum speedup compounds). PASS.
- **TEST-3 cliff non-converger recovery (the decode-quality target)**: cliff
  chan0/seed12345/esn0=15 — SPA 18/62, NMS 18/62, **SCMS layered 20/62** (+2
  recovered BP-non-convergers, fail 44→42), consistent with the
  cfg16_decode_loss_is_ldpc_bp prediction that normalization/self-correction
  recovers the naive-SPA floor. PASS (illustrative; full-fleet A/B is the §5
  follow-on).
- **Composition gates**:
  - `--test` exit 0 BOTH states (default-off AND
    `MERCURY_LDPC_MINSUM=1 MERCURY_LDPC_LAYERED=1 MERCURY_LDPC_MINSUM_VARIANT=scms`).
  - `--test-climb-engine` exit 0 BOTH states.
  - `--test-decode-marathon` (lever-C pool integrity) ALL PASS BOTH states —
    parallel==serial byte-identical with min-sum enabled, fail-before defeat
    still diverges ⇒ min-sum composes with the pool.
