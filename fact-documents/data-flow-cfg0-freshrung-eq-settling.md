# data-flow / design: cfg0 fresh-rung EQ settling — the SKIP-VAR seam

Built on `staging/inband-cfg0-stack` @ e554f66e (branch `wip/cfg0-eq-settling`).
Pairs with `_research/cfg0_eq/RESEARCH.md` (root cause + prior art) and the
existing `data-flow-noise_variance_estimate.md` (nv producer/consumer registry).

All file:line refs are against this tree. Facts from reading executing code.

---

## §1 SYMPTOM (verified, from the task + source)

The first 1-2 OFDM frames of a FRESH CONFIG_0 batch trip the SKIP-VAR gate
(`telecom_system.cc:3039`, ceiling `skip_var_nv_ceiling(CONFIG_0)=1.60`,
`:437`) at `noise_variance_estimate` ≈ 1.9-3.4, while settled frames sit at
≈ 0.0357. cfg0 reaches the rung and the bulk decodes clean, but the SEAM
frames fail → the batch never completes clean → `wb_configs_seen` never
includes 0.

## §2 ROOT (source-verified — NOT the threshold)

cfg0 is WB ⇒ `channel_estimator == LEAST_SQUARE` (`telecom_system.cc:10700`;
ZERO_FORCE is forced ONLY for narrowband, `:11002-11004`). So the gate input
for WB cfg0 is the **LS pilot-residual** estimate
(`LS_channel_estimator`, `ofdm.cc:1904-1933`):
`nv = mean over pilots |Y - H_smoothed·X_pilot|²`.

The var spike is RESIDUAL-CFO/SFO PHASE, not channel noise:

1. A FRESH-batch frame-0 has NO refined CFO to reuse. With `rx_eff_preamble>=2`
   it runs the one-shot Moose estimate (`telecom_system.cc:2704`,
   `carrier_sampling_frequency_sync`). At cfg0 that estimate is marginal
   (FTR≈0.08-0.12, frame0-doc §1) → leaves a residual CFO/SFO.
2. `CPE_correction` (`ofdm.cc:2367`, run at `telecom_system.cc:2880` BEFORE the
   estimator) removes only ONE global linear phase ramp
   (`phase_rate = arg(dH_sum)/Dy`, applied `e^{-j·phase_rate·i}`). On the cold
   frame that slope is itself noisy (few pilot rows, marginal lock) and is a
   first-order model; SFO adds a phase that is ALSO per-subcarrier
   (`≈2π·ε·n·k/N`, RESEARCH §3.2) which a single global ramp cannot remove.
3. The leftover per-symbol deterministic phase rotates `H_smoothed·X` away from
   `Y`, so the LS residual `|Y - H·X|²` inflates to O(1). The cross-pilot helper
   `estimate_noise_from_pilot_pairs` (`ofdm.cc:1563`, the ZF/NB path + the LS
   A/B toggle) has the SAME flaw: it differences pilots in TIME and books the
   per-symbol phase increment as noise.
4. Frames 1+ enter demod pre-de-rotated by the REFINED
   `freq_offset_of_last_decoded_message` (latched at `telecom_system.cc:3520`
   after a successful decode; reused at `:2667/:2693`). Their residual is tiny →
   nv collapses to the true AWGN floor (0.0357). THAT is why only the seam
   frames spike (the decisive discriminator) and why legacy SET_CONFIG settles.

CONCLUSION: nv is being inflated by COHERENT residual-CFO/SFO phase. Raising
1.60 would admit genuinely-mis-equalized frames (the gate's job). The fix is to
(A) SETTLE the seam frame's per-symbol phase before the estimate, and (C) make
the estimator not count coherent residual rotation as noise.

## §3 THE FIX (inband-gated, legacy byte-identical when gate OFF)

Two coordinated pieces, both = "make frame-0 converge before the gate", neither
raises the 1.60 ceiling. Gated behind a single runtime flag
`cfg0_freshrung_settle_enabled` (default OFF ⇒ byte-identical), forced ON by the
in-band path. The flag covers BOTH pieces so the trio is one default-ON unit.

### §3.1 Option A — TIME-DOMAIN second-pass CFO refine (the corrected mechanism)

⚠️ CORRECTION (ICI diagnosis, --test-cfg0-eq-settle sweep): the var spike is the
INTER-CARRIER INTERFERENCE of a residual CARRIER frequency offset, not a removable
per-symbol/per-subcarrier PHASE. A FREQUENCY-DOMAIN correction (the global CPE ramp,
a per-symbol common phase θ_n, OR a per-pilot-pair de-rotation) CANNOT reduce it —
ICI is energy the FFT already mislocated across bins (sweep-proven: freq-domain
ratio off/on = 1.00). The initial per-symbol-CPE design (OpenOFDM Eq.9-10 in the
frequency domain) was therefore replaced.

The cure (US6996194 / STANAG iterative re-demod): `cl_ofdm::freshrung_cfo_refine`
measures the residual per-symbol common phase rate from the just-demodulated pilots
(the same dH_sum aggregate CPE_correction uses), converts it to a per-time-sample
frequency (phase_per_symbol / Nofdm), RE-MIXES the TIME-DOMAIN frame by e^{-jΔφ·t},
and RE-DEMODs (symbol_demod). The re-FFT sees the CFO removed → the ICI collapses →
the honest nv drops to the AWGN floor AND the data bins decode. Runs BEFORE AGC/CPE
under the gate; on a settled frame phase_rate≈0 → byte-identical no-op.

LIMIT (sweep-proven cliff): works only within the pilot-pair UNAMBIGUOUS range
(phase/sym < π/Dy; cliff at omega≈0.85 for Dy=3). A residual beyond that wraps and
is unrecoverable from Dy-spaced pilots — acquisition must own the coarse CFO. This
is the binding NEXT-ROOT on the real bench (FTR≈0.2 marginal lock; see EQ_VERIFY.md).

Cost: ONE pilot walk + ONE re-mix + ONE re-demod. Bounded, only when the gate is ON.

### §3.2 Option C — residual-CFO/SFO-robust noise estimators (honest gate input)

`estimate_noise_from_pilot_pairs` (ZF/NB + LS A/B toggle) and the LS
pilot-residual walk (`ofdm.cc:1904-1933`) both gain a residual-robust mode under
the gate:

- cross-pilot helper: before squaring `delta = H_raw(i) - prev_H[j]`, de-rotate
  `prev_H[j]` by the COMMON per-pair phase increment `arg(dH_sum)` (the SAME
  aggregate `CPE_correction` computes at `ofdm.cc:2404`). `delta` then carries
  only the (incoherent) noise difference, not the coherent rotation. This floors
  AT the true AWGN σ²/|X|² — it removes a coherent term, it does NOT subtract
  noise, so nv cannot collapse below the thermal floor (the cfg16-nv-collapse
  failure class, fact-doc §3.1/§3.2, is structurally impossible here).
- LS residual walk: after building `H_smoothed`, estimate the common per-symbol
  phase `θ_n` (same θ as A) and remove it from `Y` before forming the residual
  `|Y·e^{-jθ_n} - H·X|²`. Equivalent to measuring the residual AFTER the
  per-symbol settle. Default path (gate OFF) keeps the exact existing residual.

C makes the gate INPUT correct (the measurement was wrong, not the threshold).
A makes the DECODE succeed. Together: the seam frame's honest nv drops < 1.60,
the gate passes it, and it actually decodes.

## §4 §5 CROSS-LAYER AUDIT (per CLAUDE.md §5)

State touched: `noise_variance_estimate` (+ the per-symbol rotation of the live
`ofdm_symbol_demodulated_data`), and indirectly `freq_offset_of_last_decoded_message`
(unchanged — A/C do not write it).

### 1. Producers of nv (writers)
- `ZF_channel_estimator` `ofdm.cc:1686` via `estimate_noise_from_pilot_pairs`.
- `LS_channel_estimator` residual walk `ofdm.cc:1904-1933` (production cfg0 path).
- `LS_channel_estimator_tinterp` (FADE tier) — floors at the cross-pilot helper.
- A/B toggle `ls_use_crosspilot_nv` `ofdm.cc:1942-1943` (default false).
Fix changes ONLY the helper + the LS residual walk, both gated. TINTERP's floor
call goes through the (gated) helper → when the gate is ON its FLOOR also becomes
residual-robust, which is strictly MORE honest (lower coherent inflation) and
still floored 1e-6 — verified safe (it only floors, never raises tinterp nv).

### 2. Consumers of nv (readers) — from data-flow-noise_variance_estimate.md §3
- §3.1 MMSE erasure (`ofdm.cc:2173-2209`): `alpha=|H|²/(|H|²+nv)`. MOST sensitive.
  Fix LOWERS nv on a residual-CFO frame toward the TRUE floor → alpha rises
  toward the value a clean frame would have → erasure becomes MORE correct
  (it was over-erasing because coherent phase faked high noise). Cannot collapse
  (floored at thermal σ², not below). SAFE; net-positive on the seam frame.
- §3.2 psk.demod LLR scale: lower-but-honest nv → LLRs closer to the true SNR →
  better decode. Cannot over-confident-collapse (floor preserved). SAFE.
- §3.6 SKIP-VAR gate: the intended beneficiary — nv now honest, passes the
  decodable seam frame, still rejects a genuinely-noisy/mis-timed frame (those
  have INCOHERENT residual the de-rotation does NOT remove). SAFE.
- §3.3/§3.4/§3.5 (CSI weight, mean_H, selectivity): read H ONLY, not nv. A
  rotates the input `in` per-symbol BEFORE the estimator, so H is built from the
  de-rotated `in` — its MAGNITUDE is unchanged (rotation is unit-modulus), only
  its phase is cleaned. mean_H (magnitude) and selectivity (|H| spread)
  UNCHANGED. CSI weight |H_k|² UNCHANGED. SAFE.
- §3.9 ARQ diagnostic: read-only log. SAFE.

### 3. Valid states before any producer writes
Default-init nv=0.01, H status UNKNOWN (`ofdm.cc:88`). An estimator always runs
first in the decode loop. The per-symbol refine (A) runs on a real acquired
frame only (gated + inside the OFDM branch after CPE_correction). Degenerate
frame (Nsymb<=0 / Nc<=0 / <2 pilot pairs): A and the C-de-rotation early-return
(mirror CPE_correction's `dH_count<2` guard) → falls back to the existing
estimate → byte-identical to gate-OFF. No NaN/Inf path.

### 4. Invariants consumers assume, and that the fix preserves
- nv is a TRUE per-config noise estimate (SKIP-VAR §3.6): preserved — C removes
  only a COHERENT term, leaving the incoherent thermal floor; a real-noise frame
  (incoherent residual) is NOT helped by de-rotation → still rejected.
- nv never collapses below thermal (MMSE/LLR §3.1/§3.2, the cfg16 class):
  preserved — de-rotation subtracts a phase, the magnitude floor 1e-6 stays,
  and the residual after removing coherent phase is still ≥ the AWGN difference.
- H magnitude unchanged (mean_H/CSI/selectivity): preserved — A/C rotate by
  unit-modulus factors only.
- `freq_offset_of_last_decoded_message` latch/scrub (`:3520/:3677-3706`): NOT
  written by A/C. The poison-scrub still fires on a sustained fail run. SAFE.

### 5. What the fix changes, walked per consumer
Lowers the COHERENT-phase inflation in nv on residual-CFO frames (mainly the
cold seam frame). Every nv consumer above either improves (MMSE/LLR/SKIP-VAR)
or is untouched (H-only readers). No consumer regresses on a clean settled frame
because there the coherent residual is already ~0 (frames 1+ are pre-de-rotated)
→ A/C are ~no-ops (θ_n ≈ 0, de-rotation ≈ identity) → nv ≈ unchanged.

## §5 CFG16 RE-VALIDATION REQUIREMENT (RESEARCH §5 caveat)

Production cfg16 uses the LS RESIDUAL estimator (NOT the cross-pilot helper;
`ls_use_crosspilot_nv` default false). The fix's LS-residual change is GATED
(`cfg0_freshrung_settle_enabled`), and the in-band path drives cfg0; cfg16 is a
high rung. To be safe we still re-run cfg16 clean BER with the gate in its
production setting and confirm no regression vs legacy (a settled cfg16 frame
has ~0 coherent residual, so the per-symbol θ_n ≈ 0 ⇒ expect byte-identical).

## §6 FAIL-BEFORE / PASS-AFTER

- Unit (`--test` M=0): a synthetic cfg0 frame demod'd with an injected residual
  CFO must read nv > 1.60 with the gate OFF (FAIL) and nv < 1.60 with the gate ON
  (PASS), and the existing clean-frame nv must be unchanged with the gate ON.
- Faithful real-audio: the in-band cfg0 path crosses (clean BATCH-DONE,
  `wb_configs_seen=[0]`, bytes approaching legacy) with the gate ON and does not
  with the gate OFF.
