# LDPC decode-acceleration triple — fact document

Branch `fix/ldpc-decode-accel` off `fix/break-fh-gate @465956e`.
Worktree `C:/Users/kamer/mercury_wt/ldpc-accel`.

## §0 Problem (MEASURED, firsthand HW [TIMING] logs)

Held-CFG16 reverse-ACK turnaround miss is a **DECODE-LATENCY spike**, not a
window-centering miss. A non-converging CFG16 frame burns the full
`nIteration_max=100` SPA iterations (`iter=101` = failed) at 1481-3261 ms LDPC
time each; each failing OTA frame triggers ~4.5 full 101-iter decodes (sub-peak
timing probe) = ~6-7 s injected. The SACK then fires up to 3480 ms after the
last frame, past the 3524 ms CMD window. OK frames are FAST (median iter=3,
p95=8, zero hit 101) and early-terminate on the syndrome check
(`ldpc_decoder_SPA.cc` `nOnes==0` break).

Two independent levers + one build lever attack this:
- **A** makes EVERY iteration cheaper (O(dc^2)->O(dc) check-node update).
- **B** bounds the FAIL spike (cap the wasted iterations 31..100 on a frame that
  will fail anyway).
- **F** tunes the on-Pi build for the actual Cortex-A76 uarch.

## §1 SOLUTION A — forward-backward check-node update (env MERCURY_LDPC_FWDBACK)

`ldpc_decoder_SPA.cc`. The original check-node update (the gated default loop,
`:172-204`) recomputes the full leave-one-out tanh product SEPARATELY for every
outgoing edge of a check node => O(dc^2) tanh per check. dc=46 for CFG16
rate-14/16 (`mercury_normal_14_16.cc:28`), P=200 => ~414k tanh per BP iteration.

The forward-backward form (`:206-251`, gated ON by `MERCURY_LDPC_FWDBACK`):
gather the valid edges' `tanh(0.5*Q)` into `fb_t[]`, compute exclusive forward
prefix products `fb_pref[k]=prod(fb_t[0..k-1])` and a single backward suffix
sweep, then `out[k]=fb_pref[k]*suffix` = the leave-one-out product in O(dc).

### Why mathematically identical
- The original excludes the current edge by VARIABLE-NODE index (`i1 != j`).
  Each Mercury LDPC check row holds DISTINCT, strictly-increasing variable-node
  indices terminated by -1 (verified `mercury_normal_14_16.cc:31+`, IRA
  structure), so "exclude varnode j" == "exclude this slot". The forward-backward
  excludes by slot position k => SAME product set.
- The `temp==±1` saturation clamp is applied to the SAME leave-one-out product
  value before `2*atanh`, exactly as the original.
- Only the multiply ORDERING changes => possible last-ULP FP reassociation.

### Identity proof (STRONGER than BER-identity: full decode-trajectory identity)
Coded SFO-GRID harness (`-m PLOT_PASSBAND -s 16`, `MERCURY_SFO_GRID=1
MERCURY_SFO_GRID_CODED=1`), 27 cells = 3 channels {flat, det-floor, two-ray} x
3 seeds {12345,999,77} x 3 EsN0 {15,16,17}, K=62 codewords each.
- md5 of all `[SFO-GRID-CODED]` lines: **`96bf314cb26af20553971223da10428f`**
  with FWDBACK off === with FWDBACK on === BASE binary @465956e.
- Identical fields: `codewords_decoded`, `fail`, `post_FEC_info_BER`, AND
  `iter_mean/iter_min/iter_max`. The iter counts matching => the forward-backward
  produces the EXACT same per-iteration syndrome trajectory: not one decode
  decision nor one convergence iteration changed on this fleet. So on the tested
  matrices/channels the result is BIT-IDENTICAL, not merely BER-identical.

The O(dc) restructure is the speedup; the Pi-side wall-clock drop (ldpc-ms per
iteration) is Pi-only-confirmable (§5).

## §2 SOLUTION B — OFDM iteration cap (env MERCURY_LDPC_ITERCAP=<N>)

`telecom_system.cc` per-config setup (right after the ROBUST `nIteration_max=200`
override at `:10114-10115`). When `MERCURY_LDPC_ITERCAP=N` is set AND the config
`is_ofdm_config && !is_robust_config`, and `1 <= N < ldpc.nIteration_max`, sets
`ldpc.nIteration_max = N`. Default suggest 30 (the cap the campaign should run).

This bounds the fail spike: a frame that does not converge by N fails at N+1
instead of 101. OK frames that converge at iter<=N are UNAFFECTED.

### Cross-layer audit of `ldpc.nIteration_max` (shared state)
**Producers**: `telecom_system.cc:10110` (default 100), `:10115` (ROBUST 200),
`:5308` (GUI live override, clamped [5,100]), NEW `:10117-10134` (this cap).
**Consumers**:
- `:3147` fail-classify: `iterations_done > (nIteration_max-1)` -> message_decoded=NO.
- `:3172` sub-peak-recovery gate: same `> nIteration_max-1` predicate.
- `ldpc_decoder_SPA.cc:164` decode loop bound `iteration<=nIteration_max`.
- harness `:7278` `capped = iters > (nIteration_max-1)`.
**Invariant**: the fail-classify threshold is DERIVED from `nIteration_max`, so it
moves WITH the cap. A capped frame (`iterations_done == N+1 > N-1`) is still
classified FAIL exactly as before; an OK frame (`iterations_done <= N`) is still
OK. The producer ordering is correct: the cap runs AFTER the ROBUST override,
and only narrows (never widens) for OFDM configs, so ROBUST stays 200 and the
GUI override (OFDM-only, [5,100]) composes (whichever is lower bounds the fail
burn — both safe).
**What the fix changes**: only the OFDM `nIteration_max` value when the env is
set; every consumer reads the same field and its FAIL predicate tracks it. No
consumer assumes a fixed 100.

### Cap sweep (coded SFO-GRID, clean WGN)
Steep-waterfall band (EsN0 {14,15,16}, 6 seeds, K=62 = 1116 codewords):
| cap | decoded | OK-frame loss vs no-cap | global iter_max |
|-----|---------|-------------------------|-----------------|
| none(100) | 470/1116 | -- | 101 |
| 50  | 466/1116 | 4/470  = 0.85% | 51 |
| 30  | 458/1116 | 12/470 = 2.6%  | 31 |
| 20  | 443/1116 | 27/470 = 5.7%  | 21 |

The fail-frame decode count is PROVABLY bounded to cap+1 (`global_iter_max`),
never 101. The 2.6% at cap=30 is the steep-cliff worst case.

HW-like operating point (above the cliff = where bench-9 CFG16 clean-front
actually runs; `iter_mean=3-5, iter_max=6-22`, matching the HW "median 3 / p95 8
/ p99 16" distribution), EsN0 {16, 16.5, 17}, K=125 each:
**cap=30 loses ZERO codewords (125/125 -> 125/125 at all three points)** while the
fail-frame burn drops 101 -> 31 (~3.3x). => cap=30 is the right default: 0%
OK-loss at the deployed SNR, fail spike bounded.

## §3 SOLUTION F — ARM CPU tuning (build.sh, aarch64 native build only)

`build.sh` Linux non-cross-build branch. On an aarch64/arm64 host (`uname -m`),
add `-mcpu=native` (auto-detects the Pi 5 Cortex-A76 on the on-Pi `build.sh o3`)
with a compile-probe fallback to `-mcpu=cortex-a76`, plus `-fno-math-errno`.
Pi 5 Cortex-A76 confirmed via `tools/mercury_deploy_rpi.py:467/695` ("both Pi 5
aarch64"). NO `-ffast-math` (it would drop the NaN/Inf guards the SPA temp-clamp
and the OFDM variance/LLR clamps rely on); `-fno-math-errno` is safe (libm errno
never read). Cross-build path is untouched (a cross host may mistune). Windows /
macOS render untouched => §1 byte-identity holds on the Windows render.

## §4 Default-off byte-identical

With BOTH `MERCURY_LDPC_FWDBACK` and `MERCURY_LDPC_ITERCAP` unset: SPA runs the
original loop bit-for-bit; `iter_cap` stays 100 (verified unset/0/200 all leave
100). Render md5 `96bf314c...` === base @465956e (§1). F is a build flag with no
behavior change on the existing render.

## §5 What only the Pi can confirm
- The ACTUAL ldpc-ms-per-iteration drop from A (O(dc) restructure) and F
  (-mcpu=native) on the Cortex-A76 — Windows is a different uarch and the
  harness measures decode CORRECTNESS, not Pi wall-clock.
- Whether the bounded fail burn (B) + cheaper iterations (A) actually pull the
  SACK back inside the CMD window and close the turnaround stall on HW (the
  end-to-end bench-9 metric: reverse-ACK miss -> 0, active-fraction up).
- A is bit-identical on the tested fleet; a pathological channel could in
  principle expose an FP last-ULP decode flip. The Pi BER run is the final gate
  (expected identical; if not, A stays BER-identity-gated per the FACTS caveat).
