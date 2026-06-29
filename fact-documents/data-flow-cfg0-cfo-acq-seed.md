# data-flow / §5 audit: cfg0 fresh-acquisition CFO SEED (ROBUST→cfg0 cross)

Branch `wip/cfg0-eq-settling` (built on 3c293e1c, the consolidated 7-root stack +
the time-domain EQ-refine df844683). Pairs with
`data-flow-cfg0-freshrung-eq-settling.md` (the EQ-refine; this doc is the
COMPLEMENTARY upstream acquisition fix it names as the binding NEXT-ROOT in its
§3.1 LIMIT note) and `data-flow-noise_variance_estimate.md` (nv registry).

All file:line refs are against this tree. Facts from reading executing code.

---

## §1 SYMPTOM + the binding next-root (verified, from the task + the EQ-settling doc §3.1)

The EQ-refine (`freshrung_cfo_refine`, ofdm.cc:2522) cleans a residual CFO ONLY
within the pilot-pair unambiguous range (phase/sym < π/Dy, sweep cliff omega≈0.85).
Its own LIMIT note (eq-settling §3.1): a residual BEYOND that wraps and is
unrecoverable from Dy-spaced pilots — "acquisition must own the coarse CFO. This is
the binding NEXT-ROOT on the real bench (FTR≈0.2 marginal lock)."

The cfg0 fresh frame-0 locks COLD: a fresh WB OFDM acquisition has NO CFO seed, so
the fine Moose (telecom_system.cc:2704) measures the full carrier offset from a
marginal preamble correlation (FTR≈0.2) and leaves a large residual CFO → frame-0
nv 1.9-3.4 → SKIP-VAR (telecom_system.cc:3039, ceiling 1.60) → rolling-partial → no
BATCH-DONE → wb_configs_seen=[] → ~210-804 B vs legacy ~28-113 KB.

## §2 ROOT (source-verified — the CFO scrub at config-change is OVER-BROAD)

The carrier-frequency offset (crystal/clock mismatch + sound-card drift) is a
CONFIG-INDEPENDENT physical property of the link. But `load_configuration`
(telecom_system.cc:11549-11551) scrubs it on EVERY config change:

```
receive_stats.delay_of_last_decoded_message = -1;        // timing: legit reset
receive_stats.freq_offset_of_last_decoded_message = 0;   // CFO: OVER-BROAD reset
consecutive_ofdm_decode_fails = 0;
```

The comment justifies the reset by "frame timing differs between configs
(preamble_nSymb varies 4 OFDM / 8 NB MFSK / 16 WB MFSK)". That is TRUE FOR TIMING
(`delay`) — the preamble length changes, so `delay_of_last_decoded_message` is
correctly invalidated. It is NOT true for the CARRIER OFFSET: a 11 Hz crystal
mismatch is 11 Hz at ROBUST_2 and 11 Hz at CONFIG_0. Bundling the CFO scrub with the
timing scrub throws away a still-valid physical estimate, forcing the ROBUST→cfg0
cross to re-acquire the carrier COLD — exactly the marginal lock §1 describes.

The settled CFO EXISTS at the cross but is discarded:
1. The ROBUST modes (MFSK 100/101/102) that just decoded the CONNECT/robust traffic
   measure the residual carrier offset EVERY frame via the purpose-built,
   confidence-gated `carrier_frequency_sync_wb_mfsk` (mini-Moose, telecom_system.cc:
   2728; ofdm.cc:705, capture ±46.875 Hz, returns 0 below |C|/E<0.05). The net
   carrier the robust frame demodulated at = `carrier_frequency + coarse_freq_offset
   - freq_offset_measured` (the re-mix at telecom_system.cc:2803). The net CFO =
   `coarse_freq_offset - freq_offset_measured`.
2. That MFSK CFO is NEVER latched: the latch at telecom_system.cc:3538-3540 gates on
   `M != MOD_MFSK` (an OLD distrust of the legacy MFSK-on-OFDM-preamble Moose, which
   is NOT the new confidence-gated wb_mfsk estimator). So `freq_offset_of_last_
   decoded_message` is whatever a prior OFDM decode left — and `load_configuration`
   zeroed even that at the cross. Result: COLD.

## §3 THE FIX (carry the settled CFO as an acquisition SEED; in-band-gated, legacy byte-identical)

A new persistent member `cfo_acq_seed_hz` (cl_telecom_system) holds the settled net
carrier offset to seed the next fresh acquisition. Three coordinated pieces, ALL
gated behind `ofdm.cfg0_freshrung_settle_enabled` (the existing in-band gate, set ON
for CONFIG_0..6 under MERCURY_INBAND_RATE at telecom_system.cc:11040; default OFF ⇒
byte-identical):

- §3.1 PRODUCER (latch): on a successful decode that measured a confident CFO, store
  `cfo_acq_seed_hz = coarse_freq_offset - freq_offset_measured` (the net carrier the
  frame demodulated at). Done for BOTH the OFDM success path AND the ROBUST/MFSK
  success path (the latter is the cross seed). Gated; legacy never writes it.
  For MFSK the latch only fires when the wb_mfsk estimator returned a CONFIDENT
  (non-zero) residual — a 0 return (low |C|/E) means "no estimate", so we keep the
  prior seed rather than latch a 0.

- §3.2 CARRY (the cross): `cfo_acq_seed_hz` is NOT zeroed by `load_configuration`
  (telecom_system.cc:11550). The timing scrub (delay) stays; the CFO seed survives.
  This is the carry-site — the crystal offset crosses ROBUST→cfg0 intact.

- §3.3 CONSUMER (seed the cold acquisition): for a FRESH WB OFDM acquisition (cold
  trial-0, `coarse_freq_offset==0`, not the per-call coarse-search trial, OFDM
  config, not NB, not BER-forced), when the gate is ON and `cfo_acq_seed_hz` is
  VALID (non-zero, |seed|≤ a sane carrier-drift bound), initialize
  `coarse_freq_offset = cfo_acq_seed_hz` BEFORE frame extraction. The extraction
  (telecom_system.cc:2613/2652) then mixes at `carrier_frequency + seed`, and the
  fine Moose (:2704) measures only the SMALL residual → STRONG, unambiguous lock →
  small residual CFO → frame-0 nv drops below 1.60 AND the data decodes. The EQ-refine
  (df844683) is complementary: it cleans whatever residual remains after the seed.

The seed bound: ±1 subcarrier spacing (47 Hz WB), matching the existing Moose clamp
(telecom_system.cc:2764). A seed outside that is implausible for a crystal/clock
offset (the robust rung could not have decoded with it) → treated as invalid →
fall back to the cold path (byte-identical to today). The per-call coarse search
(:2353) still runs on trial 1 if the seeded trial 0 fails, so the seed never REMOVES
acquisition capability — it only WARM-STARTS it.

## §4 §5 CROSS-LAYER AUDIT (shared state: the acquisition CFO seed)

State touched: the new `cfo_acq_seed_hz`, and the per-call `coarse_freq_offset`
(seeded on the cold fresh-OFDM trial-0). NOT touched: `freq_offset_of_last_decoded_
message`, `last_coarse_freq_offset`, `delay_of_last_decoded_message`,
`consecutive_ofdm_decode_fails` (the poison-scrub state) — all unchanged.

### 1. Producers of the CFO seed (writers)
- NEW: telecom_system.cc OFDM success path (~:3540 area), gated. Writes
  `coarse_freq_offset - freq_offset_measured`.
- NEW: telecom_system.cc ROBUST/MFSK success path (~:3540 area), gated, only on a
  confident wb_mfsk estimate. Writes `coarse_freq_offset - freq_offset_measured`.
- `init()` / ctor: `cfo_acq_seed_hz = 0.0` (the default no-seed state).
- NOT written by `load_configuration` (the carry: it survives the config change).

### 2. Consumers of the CFO seed (readers)
- NEW: the cold fresh-OFDM trial-0 seed of `coarse_freq_offset` (~:1153 area),
  gated. The ONLY reader. Downstream, `coarse_freq_offset` then feeds:
  - frame extraction mix (:2613/:2652) — now warm-started.
  - the fine Moose input (:2704) — now measures a small residual.
  - the fine-sync slice mix (:2485) — warm-started (bit-exact w.r.t. the seed value).
  - `last_coarse_freq_offset` is ONLY assigned from the coarse SEARCH (:2417), which
    does not run on trial 0 — so the ACK/SACK detectors (which read
    `last_coarse_freq_offset`, :3931/:4229/...) are UNAFFECTED by the trial-0 seed.
    (If the seeded trial-0 succeeds, `last_coarse_freq_offset` keeps its prior value,
    same as today's cold trial-0 success.)

### 3. Valid states before any producer writes
Default `cfo_acq_seed_hz = 0.0` (ctor/init). The consumer treats 0.0 as
"no seed" (the cold path, byte-identical). A confident robust/OFDM decode is the
only thing that makes it non-zero. Degenerate inputs (no robust decode yet, low
wb_mfsk confidence) leave it 0 → cold path. No NaN/Inf: the bound check
(`|seed| ≤ 47 Hz`) rejects any wild value and falls back to cold.

### 4. Invariants consumers assume, and that the fix preserves
- INV-A (acquisition completeness): the per-call coarse search (:2353, trial 1) and
  all later trials are UNCHANGED. The seed only WARM-STARTS trial 0; if it is wrong
  the existing trial-1 coarse search + Moose recover exactly as today. The fix can
  never REDUCE acquisition reach. PRESERVED.
- INV-B (Moose sanity clamp, :2754/:2764): unchanged. With a good seed the Moose
  residual is SMALL (well inside ±47 Hz), so the clamp/reject path is reached LESS
  often, never more. A bad seed makes the residual LARGER, but then the MOOSE-REJECT
  (:2754) advances to trial 1 (coarse search) — the existing recovery. PRESERVED.
- INV-C (the poison-CFO scrub, :3719-3728): reads `consecutive_ofdm_decode_fails`
  and scrubs `freq_offset_of_last_decoded_message` + `last_coarse_freq_offset`. The
  seed is a DISTINCT member; on a sustained-fail run the scrub should ALSO drop the
  seed (a stale seed must not poison forever). The fix scrubs `cfo_acq_seed_hz` at
  the SAME site (:3726-3727) so the cure scope is identical. PRESERVED + extended.
- INV-D (config-change timing reset, :11549): `delay_of_last_decoded_message=-1` is
  UNCHANGED — timing is still correctly reset per config. Only the CFO seed survives.
  PRESERVED.
- INV-D2 (ORDERING — load_configuration calls init() on a config REINIT, :11319):
  CORRECTION to the first draft. `init()` is NOT a fresh-session-only reset — it is
  called by `load_configuration` (gated `reinit_subsystems.telecom_system`) on a
  config change, AFTER the gate block sets `inband_cfo_seed_active` (:11120). So
  resetting `cfo_acq_seed_hz`/`inband_cfo_seed_active` in `init()` would (a) WIPE the
  settled carrier on the ROBUST->cfg0 cross (defeating the carry) and (b) clobber the
  just-set in-band flag. Both are therefore reset ONLY in the ctor (cold default) +
  re-set per-config by `load_configuration` (the flag) + dropped by the poison-CFO
  scrub (:3726). The producer must latch at the ROBUST rung (config 100..102), which
  is OUTSIDE the cfg0..6 `cfg0_freshrung_settle_enabled` scope, so the producer is
  gated on the config-INDEPENDENT `inband_cfo_seed_active`, not `cfg0_freshrung_settle_
  enabled`. (Bench-verified: with the producer gated on the cfg0..6 flag, the ROBUST
  seed never latched — MERCURY_CFOSEED_DBG showed every LATCH at cfg=0, never robust.)
- INV-E (legacy / gate-off byte-identical): every producer + consumer is behind
  `ofdm.cfg0_freshrung_settle_enabled`; off-flag the seed is never written and never
  read → `coarse_freq_offset` stays 0 on trial 0 exactly as today, and the CFO scrub
  at :11550 still zeroes `freq_offset_of_last_decoded_message`. PRESERVED.

### 5. What the fix changes, walked per consumer
Trial-0 of a fresh WB OFDM acquisition starts at `coarse_freq_offset = settled-CFO`
instead of 0, under the in-band gate. Every downstream reader of `coarse_freq_offset`
(extraction mix, Moose, slice mix) sees a warm carrier instead of a cold one → the
Moose lock is strong, the residual small, the frame-0 nv low, the SKIP-VAR passes,
the frame decodes. No consumer regresses: the coarse-search and trial recovery are
intact (INV-A/B), the ACK/SACK detector carrier is untouched (it reads
`last_coarse_freq_offset`, not the trial-0 seed), and off-flag everything is
byte-identical (INV-E).

## §5 CFG16 RE-VALIDATION REQUIREMENT

CONFIG_16 is OUTSIDE the gate scope (gate is CONFIG_0..6, telecom_system.cc:11041),
so cfg16 acquisition is byte-identical by construction. But because an acquisition
change can ripple, we still re-run cfg16 clean BER and confirm no regression vs
legacy (expect byte-identical — the gate is OFF for cfg16, and the seed member is
never read there).

## §6 FAIL-BEFORE / PASS-AFTER

- Unit (`--test`, M=0): a synthetic fresh cfg0 acquisition with an injected carrier
  offset + a valid settled seed reads a LARGE Moose residual / high nv with the seed
  DISABLED (FAIL: cold), and a SMALL residual / nv < 1.60 with the seed ENABLED
  (PASS: warm). A 0/invalid seed → byte-identical cold path. Legacy gate-off → cold.
- Faithful real-audio: the in-band cfg0 path crosses (clean BATCH-DONE,
  wb_configs_seen=[0], bytes approaching legacy) with the gate ON; the fresh cfg0
  lock metric rises (0.197 → high) and frame-0 var drops below 1.60.
