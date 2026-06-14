# Decode marathon — LEVER D: layered / row-by-row BP — fact document

Branch `feat/decode-marathon` off `feat/turnaround-eff @943a083`.
Worktree `C:/Users/kamer/mercury_wt/decode-marathon`.
First of the marathon levers (D); later levers (C/E/G/H-I) stack on this branch.

## §0 Problem & lever (the convergence-speed half of the decode marathon)

The SPA decoder (`ldpc_decoder_SPA.cc`) uses **FLOODING** scheduling: within one
BP iteration every check node consumes the SAME old var->check message snapshot
(`Q`), and the var nodes are only refreshed AFTER all checks have produced their
new `R`. Information from a check updated early in an iteration does not reach the
other checks until the NEXT iteration.

**LEVER D = LAYERED / horizontal-shuffled / row-layered BP.** Process the P
parity-check rows ONE AT A TIME and immediately fold each updated check->var
message into a running a-posteriori sum `L[v]`, so a LATER row in the SAME
iteration reads var->check extrinsics already refreshed by the EARLIER rows of
that iteration. "The most recent information is disseminated." On the QC/IRA
matrices of `mercury_normal_*.cc` this is the natural row-layered schedule.

### Citations (CLAUDE.md §1 research-first)
- D. Hocevar, "A reduced complexity decoder architecture via layered decoding of
  LDPC codes," IEEE Workshop on Signal Processing Systems (SIPS), 2004,
  pp. 107-112, doi:10.1109/SIPS.2004.1363033 — the canonical layered-BP result:
  ~2x faster convergence (about half the iterations) + ~50% logic for equal
  throughput/BER.
- M. M. Mansour, N. R. Shanbhag, "High-throughput LDPC decoders," IEEE Trans.
  VLSI Syst. 11(6):976-996, 2003 — turbo-decoding-message-passing (layered)
  schedule.
- E. Sharon, S. Litsyn, J. Goldberger, "Efficient serial message-passing
  schedules for LDPC decoding," IEEE Trans. Inf. Theory 53(11):4076-4091, 2007 —
  serial-C schedule, same ~2x acceleration, BER comparable-or-better.

## §1 Implementation (`ldpc_decoder_SPA.cc`)

Gate `ldpc_layered_enabled()` reads env `MERCURY_LDPC_LAYERED` once (static
cache). OFF (unset/0) => the existing flooding loop (the O(dc^2) default AND the
`MERCURY_LDPC_FWDBACK` O(dc) path) runs **bit-for-bit** in the `else` branch.

The layered branch (gated `if(layered)`) maintains the running APP
`L[v] = LLRi[v] + sum_all R[v][slot]`. R is all-zero at entry (the function zeros
R/Q at `:179-188`), so `L == LLRi` at iteration 1 — the layered schedule is a
pure reschedule of the SAME message updates, no semantic change to the LLRs.

Per iteration, one sweep over the P rows. For each row `iindex`:
1. Gather valid edges: for each edge to var `vj` at V-slot `vi`, the extrinsic
   `Q = L[vj] - R[vj][vi]` uses the LATEST `L` (already refreshed by earlier rows
   of THIS iteration = the layered win). Store `tanh(0.5*Q)` and the (vj, vi).
2. Leave-one-out product over the OTHER edges (same kernel as flooding: O(dc^2)
   by default, or the lever-A O(dc) forward-backward when `MERCURY_LDPC_FWDBACK`
   is set), with the SAME +-1 saturation clamp -> `Rnew = 2*atanh(product)`.
3. Incremental APP write-back: `L[vj] += Rnew - R[vj][vi]; R[vj][vi] = Rnew`.

After the sweep: hard-decision `LLRbin[v]=(L[v]<0)`, syndrome `nOnes`, success
break on `nOnes==0`, and the #3 shared non-convergence early-term on the layered
`nOnes` (composes with #3). `LLRtmp` is kept synced to `L` each iteration so the
function epilogue (`LLRo[0..K)` and `app_llr[0..N)`) and the return sentinels
(`iteration` / `nIteration_max+1` early-term / `-iteration` abort) are byte-for-
byte the SAME contract as the flooding path.

### Composition
- **A (env MERCURY_LDPC_FWDBACK):** layered reuses the same forward-backward
  leave-one-out kernel — layered+fwdback produces the identical layered result.
- **#3 (env MERCURY_SYND_EARLYTERM):** the syndrome non-convergence detector
  operates on the layered `nOnes` unchanged; it still fires and bounds doomed
  decodes (measured: chan=1 esn0=16 trips 62/62 at et_iter_mean=12, as on
  flooding).

## §2 Cross-layer data-flow audit (CLAUDE.md §5)

State touched: the per-call scratch arrays `R[]`/`Q[]` (cl_ldpc members allocated
once at `load()`, ldpc.cc:140-185) and the local APP `L[]`/`LLRtmp`.

1. **Producers of R/Q**: `decode_SPA` entry zeroes R and Q (`:179-188`) every
   call; the flooding init then sets `Q[i][j]=LLRi[i]`. The layered branch never
   reads Q (it derives the extrinsic on the fly from `L - R`) and writes only the
   `R[vj][vi]` slots it just recomputed.
2. **Consumers of R/Q**: ONLY `decode_SPA` itself — `R`/`Q` are never read
   outside the decoder (grep ldpc.cc: only alloc + NULL-init). So the layered
   path's different R/Q usage cannot leak across calls or layers.
3. **Valid states**: R/Q are re-zeroed at every call entry => no cross-call
   carry. `L==LLRi` at entry depends on R==0, which the entry zeroing guarantees.
   Only `V_pos` slots `< VWidth` are ever touched (matches the zeroing extent).
4. **Invariants consumers assume**: `cl_ldpc::decode` reads the RETURN value
   (iterations / sentinel) and `last_early_term_iter`. Both are produced
   identically by the layered branch. `LLRo`/`app_llr` are written from
   `LLRtmp`, kept == `L`.
5. **What the fix changes**: only the SCHEDULE (when Q is formed). With the env
   OFF nothing changes (byte-identical). With it ON the LLR semantics are
   preserved; only convergence speed and the (improved) non-converger floor
   change.

## §3 Proof (CLAUDE.md §3, `tools/test_decode_marathon.py`)

27-cell coded SFO-GRID fleet: CHANS{0,1,2} x SEEDS{12345,999,77} x ESN0S{15,16,17},
`-m PLOT_PASSBAND -s 16`, `MERCURY_SFO_GRID_CODED=1 NSYMB=600` (K=62 codewords/cell,
1674 codewords total), same vehicle as ldpc-decode-accel/turnaround-eff.

- **TEST-1 byte-identical**: OFF render md5 `4575011ba38e23575ef3f3bbce0703b2`
  == base @943a083 `4575011ba38e23575ef3f3bbce0703b2`. PASS.
- **TEST-2 BER comparable / non-converger not regressed**: fleet
  codewords_decoded flooding=416/1674, layered=421/1674 (layered decodes 5 MORE;
  non-converger count 1258 -> 1253, IMPROVED). No cell regressed (per-cell
  layered_ok >= flooding_ok AND layered_ber <= flooding_ber). PASS.
- **TEST-3 iter ~2x drop**: over the fully-converging cells, layered/flooding
  iter_mean ratio = 0.621 (~1.61x faster). Representative cells:
  (0,12345,16) 5.68->3.53; (0,999,16) 5.10->2.98; (0,12345,17) 2.58->1.66;
  (0,999,17) 2.48->1.60. iter_max also drops (esn0=16: 25->17). PASS.

Note: at the cliff (esn0=15) the iter_mean is dominated by the cap-pinned
non-convergers (101 each), masking the per-converger speedup in the raw mean; the
clean ~1.6x figure is read off the cells whose codewords fully converge. The
~2x literature figure is the per-converger iteration count; the realised fleet
mean win is bounded by the cap-mass on cliff cells.

- `--test` exit 0 both states (default-off AND MERCURY_LDPC_LAYERED=1).
- `--test-climb-engine` exit 0 both states.
