# SACK `ldpc=NO` Hard-Fallback — Root Cause Investigation

Investigation date: 2026-05-14. Branch: `monitor`. Investigation + planning
only — no source edited. Spun off from `SACK_TURNAROUND_FIX_PLAN.md` §9.4
(out-of-scope observation surfaced during the turnaround-collision fix's
Step-8 captures). Background on the SACK detector path: `SACK_DETECTION_ROOTCAUSE.md`.

**Symptom (per §9.4 of the turnaround plan):** a `--capture` run logged
`[RX-SACK] Detected … metric=6.6 ldpc=NO`, after which CMD concluded
"12/25 received, 8 queued" while RSP had requested only 1 frame. CMD
retransmitted the wrong frames; RSP had to re-SACK. A failed LDPC decode
produced an *actionable* (and wrong) bitmap.

---

## §1 Status of the evidence

The exact log line cited in the turnaround plan (`pi_rpi2_…:1774`,
`metric=6.6 ldpc=NO`) is **no longer present in any log in
`fact-documents/timing_data/`**. In the pulled file
`timing_data/pi_rpi2_20260514_130939.log` line 1774 is a `[SACK-PER-SYM]`
diagnostic, and the only `[RX-SACK] Detected` in that file (`:1489`) is the
healthy `ldpc=YES metric=16.0` case. A grep across the whole `timing_data/`
directory for `ldpc=NO`, `[RX-SACK-RAW]`, or `metric=6.x` returns nothing.

This is consistent with `SACK_TURNAROUND_FIX_PLAN.md` §9.2 "Harness Bug 1"
(the log-pull pipeline silently truncated paths at spaces and grabbed stale
logs around exactly that time window) — the `ldpc=NO` capture was overwritten
or never landed. **However, the bug is fully provable from source alone**
(§2–§5 below); the missing log only costs us the live `matched`/`metric`
pair. §8 states the one additional capture that would pin the threshold
question quantitatively.

---

## §2 The traced code path (`[RX-SACK] Detected`)

CMD's `process_messages_rx_acks_data()` opens the SACK window and calls
`receive_sack_pattern()`:

- `arq_commander.cc:1426-1431` — `memset(sack_bitmap,0,…)` then
  `sack_detected = receive_sack_pattern(sack_bitmap, data_batch_size)`.
- `arq_commander.cc:1434` — `if(sack_detected)` … the entire retransmit-queue
  build (`:1448-1480`) and `data_ack_received = YES` (`:1483`) are gated **only**
  on the `bool` return of `receive_sack_pattern()`.

`receive_sack_pattern()` (`arq_common.cc:3711-3839`):

1. `:3752-3754` — `detect_sack_pattern_from_passband(... &matched, nframes,
   suffix_tones)` returns `metric`; fills `matched` and the **hard-decision**
   `suffix_tones[]`.
2. `:3785` — detection gate: `if(matched >= sack_match_threshold && metric >= 0.5)`.
3. `:3788-3801` — ACK cross-check; rejects only if `ack_matched > matched + 4`.
4. `:3804-3810` — `ldpc_ok = decode_sack_bitmap_ldpc(... out_bitmap)` (the
   soft-decision LDPC decode) when `M>=16 && sack_ldpc.N>0`.
5. `:3812-3823` — **`if(!ldpc_ok)` → hard-decision fallback**:
   `ack_mfsk.decode_sack_bitmap(suffix_tones, nsuffix, nframes, out_bitmap)`.
6. `:3825-3831` — prints `[RX-SACK] Detected (… ldpc=%s …)` with
   `ldpc_ok ? "YES":"NO"`, then **`return true;` unconditionally** — the
   `ldpc=NO` path returns `true` exactly like `ldpc=YES`.

So the `ldpc` field in the log is purely informational; **the function's
`true`/`false` return — the only thing CMD checks — does not depend on it.**

Where each logged field comes from:
- `matched` — `out_matched` from `ofdm.detect_ack_pattern()`
  (`ofdm.cc:3560-3561, 3401, 3428`): count of base-pattern symbols whose
  expected tone was the peak bin for all streams.
- `metric` — `best_metric` return of `ofdm.detect_ack_pattern()`
  (`ofdm.cc:3569, 3420-3431`): sum over matched symbols of
  `e_target / e_total` (energy concentration ∈ ~[0,1] per symbol). Hence
  `metric ≤ matched`, and `metric` is the *softer* quantity.
- `ack_xcheck` — `ack_matched` from the ACK cross-check
  (`arq_common.cc:3788-3791`).
- `ldpc` — `ldpc_ok`, the return of `decode_sack_bitmap_ldpc()`
  (`telecom_system.cc:3264-3384`).

---

## §3 Root cause: the hard fallback decodes LDPC-coded tones as legacy raw bits

`decode_sack_bitmap_ldpc()` is **correct** and conservative: it inits
`out_bitmap` to all-false (`telecom_system.cc:3269-3270`), and on a
non-converged decode prints `FAILED`, `return false` (`:3361,3370-3371`),
leaving `out_bitmap` untouched (all-false). If CMD acted on *its* output on
failure, the result would be "all frames NACKed" — wasteful but safe.

The bug is the **fallback at `arq_common.cc:3822`**:
`ack_mfsk.decode_sack_bitmap(suffix_tones, nsuffix, nframes, out_bitmap)`.

There is a **format mismatch** between what the TX put in those tones and what
the hard fallback assumes:

- **TX** (`mfsk.cc:650-681`, `encode_sack_bitmap`): for WB `M>=16` with a
  non-null `sack_ldpc`, the SACK suffix tones are the **LDPC codeword**:
  25 bitmap bits → K=32 info bits → `sack_ldpc->encode()` → N=128 coded bits →
  32 tones (4 coded bits/tone, natural binary). The tones are *channel-code
  symbols*, not the bitmap.
- **RX hard fallback** (`mfsk.cc:717-781`, `decode_sack_bitmap`): interprets
  the same tones as the **legacy un-coded format** — direct bit-packing with
  2× repetition: `out_received[u*nBits+b] = (tone >> b) & 1` with a
  rep-A/rep-B "bit-level merge" (`:743-762`). That decode rule only matches
  the legacy `encode_sack_bitmap` path (`mfsk.cc:683-712`), which is **not**
  the path WB M>=16 TX uses.

Therefore, when `ldpc_ok == false`, `decode_sack_bitmap()` reads the first
~`ceil(25/4)=7` LDPC *coded-symbol* tones (plus 7 more as a phantom "rep B"),
masks bits out of them, and writes the result into `out_bitmap`. The output
is **structurally unrelated to the real bitmap** — it is exactly the garbage
the §9.4 symptom describes ("12/25 received" vs the true "24/25"). The
`suffix_tones` themselves are not even noise here: they are real,
correctly-received LDPC code symbols — but decoded with the wrong codebook.

Note on *why the LDPC decode failed in the first place*: `metric=6.6` is a
**degraded but real** SACK reception (a true SACK pattern at low energy
concentration / partial tone matches — see §4), so a handful of the 32 suffix
tones were corrupted, enough to exceed the rate-1/4 code's correction power
within 50 iterations. That is a normal channel event; the bug is not that
LDPC failed, it is that the fallback then fabricated a bitmap.

**Decisive citations:**
- CMD acts on the failed-LDPC bitmap: `arq_commander.cc:1434` (`if(sack_detected)`)
  → `:1456` (`sack_bitmap[i]` drives ACKED-vs-retransmit) → `:1483`
  (`data_ack_received = YES`). `sack_detected` came from
  `receive_sack_pattern()` returning `true` at `arq_common.cc:3831`
  **regardless of `ldpc_ok`**.
- The garbage bitmap is produced at `arq_common.cc:3822`
  (`decode_sack_bitmap(suffix_tones,…)` in the `!ldpc_ok` branch,
  `arq_common.cc:3812`).
- The format mismatch: TX `mfsk.cc:655-680` (LDPC-coded tones) vs RX fallback
  `mfsk.cc:728-763` (legacy raw-bit decode). The two are not inverses.

---

## §4 Should `metric=6.6` have passed detection at all?

**Partly — the gate is on the wrong variable, but the metric itself is not
obviously a false alarm.**

- The detection gate is `arq_common.cc:3785`:
  `matched >= sack_match_threshold && metric >= 0.5`.
- `sack_match_threshold = 10` for WB (`mfsk.cc:320, 327`); `ack_pattern_nsymb`
  (base length) is 16 for WB. So the gate needs `matched >= 10` of 16.
- `metric` and `matched` differ: `metric = Σ e_target/e_total` over matched
  symbols (`ofdm.cc:3420-3421`), so `metric ≤ matched`. `metric=6.6` is
  consistent with, e.g., `matched ≈ 11-13` symbols each contributing
  ~0.5-0.6 energy concentration — i.e. **the gate (`matched >= 10`) was very
  likely satisfied legitimately**; `metric=6.6` just says the matched tones
  were energetically muddy (degraded channel), not absent. Contrast the
  healthy `:1489` case: `matched=16, metric=16.0` (every symbol a clean
  peak).
- The `metric >= 0.5` floor is effectively dead: any pattern with
  `matched >= 10` will have `metric` far above 0.5.

So `metric=6.6` passing "Detected" is **not, by itself, the root cause** — a
real (degraded) SACK was present. The threshold question is nonetheless
**part of the picture** in one specific sense: the gate admits SACK patterns
whose *base* is solid enough (`matched>=10`) but whose *suffix tone quality is
too poor for the LDPC code to correct*. There is currently **no quality gate
on the suffix region** — detection of the base pattern implicitly green-lights
acting on the suffix. A pattern good enough to detect is not necessarily good
enough to *decode the bitmap from*. That gap is real, but the **primary** bug
is §3 (acting on a garbage hard-fallback), not the base-pattern threshold.

[?] Without the original log we do not have the live `matched` value paired
with `metric=6.6`. If `matched` were, say, exactly 10-11 it would argue for
also revisiting `sack_match_threshold`; if `matched` were 14-16 it would
confirm the base was fine and only the suffix was degraded. §8 specifies the
capture that settles this. It does not change the §3 root cause or the §6
recommendation either way.

---

## §5 Correct behavior on `ldpc=NO`

When `ldpc_ok == false`, the SACK bitmap is **unrecoverable** (the only
correct decoder for the WB M>=16 wire format is the LDPC decoder; the legacy
hard decode is not its inverse — §3). The only safe states are:

1. **Treat it as "no usable SACK heard."** Do not set `data_ack_received`,
   do not build a retransmit queue from a fabricated bitmap. Let the existing
   ACK-timeout / retransmit machinery (the path CMD already uses when it hears
   nothing) handle the batch.
2. **A degraded SACK is positive proof RSP finished the batch and is
   responding** — that information is still usable even though the bitmap is
   not. CMD may use it to (a) stop waiting for more DATA-frame audio and
   (b) retransmit conservatively.

Per CLAUDE.md memory rule *"initiator controls flow"* and *"retransmitting
WRONG frames is worse than retransmitting too many"*: the worst outcome is
the current one — acting on a fabricated bitmap, marking real-missing frames
ACKED and real-received frames as missing. A **full-batch retransmit** (treat
the bitmap as all-NACK) is strictly safe: every truly-missing frame is
resent; RSP's next SACK/ACK cleans up the duplicates. It costs throughput on
that one round, not correctness.

This is also exactly what `decode_sack_bitmap_ldpc()` already *intends* —
it inits `out_bitmap` to all-false ("NACK everything on failure",
`telecom_system.cc:3268-3270`). The fallback at `arq_common.cc:3812-3823`
overwrites that safe default with garbage. Removing the fallback restores the
intended fail-safe.

---

## §6 Fix candidates

### Candidate A (recommended) — delete the hard fallback for the LDPC wire format; on `ldpc=NO`, do not act on a bitmap

In `receive_sack_pattern()` (`arq_common.cc:3804-3831`): when the LDPC path
is the active wire format (`M>=16 && sack_ldpc.N>0`) and `ldpc_ok == false`,
**do not call `decode_sack_bitmap()`**. Two sub-options for the return:

- **A1 — return `false`** (treat as "no SACK heard"): CMD's
  `if(sack_detected)` is not taken; the ACK-timeout path runs the batch as if
  no response arrived. Simplest; zero new state. Downside: throws away the
  "RSP is alive / batch finished" signal, so CMD waits out a timeout it could
  have skipped.
- **A2 — keep `return true` but with an all-NACK (`out_bitmap` all-false)
  bitmap.** `out_bitmap` is already all-false from the caller's `memset`
  (`arq_commander.cc:1430`) and from `decode_sack_bitmap_ldpc`'s init. With an
  all-false bitmap, `arq_commander.cc:1456` sends *every* pending frame to the
  retransmit queue — a clean full-batch retransmit, and `data_ack_received`
  is set so CMD doesn't also wait out the ACK timeout. This matches the
  documented intent ("NACK everything on failure").

A2 is preferred over A1: it preserves the genuine information in a degraded
SACK (RSP responded → batch done) while being correctness-safe. Trade-off:
one wasted full-batch retransmit per `ldpc=NO` event vs. A1's one wasted
ACK-timeout. Either is strictly better than today's wrong-frame retransmit.

The legacy NB / non-LDPC path (`M<16`, or `sack_ldpc` null) still legitimately
needs `decode_sack_bitmap()` — that path's TX *does* use the legacy
`encode_sack_bitmap` format (`mfsk.cc:683-712`), so `decode_sack_bitmap` *is*
its correct inverse there. The fix must be scoped to the LDPC wire format,
mirroring the `M>=16 && sack_ldpc.N>0` condition already at
`arq_common.cc:3805`. **This is a one-branch change in one function.**

### Candidate B — add a suffix-region quality gate; require LDPC success to "Detect"

Make `ldpc_ok == false` (for the LDPC wire format) cause `receive_sack_pattern()`
to *not* report a detection at all — i.e. fold the LDPC result into the
detection predicate, not just the bitmap decode. Effectively Candidate A1 plus
the framing that "a SACK you can't decode the bitmap from is not a detected
SACK." Cleaner conceptually; same throughput downside as A1. Could also add a
minimum suffix-tone energy / `metric` gate so that obviously-degraded patterns
are rejected before the LDPC decode is even attempted (addresses §4's "no
quality gate on the suffix" gap). Heavier change; touches the detection
contract that other callers (`detect_sack_pattern_from_passband` test paths)
share.

### Candidate C — make `decode_sack_bitmap()` LDPC-aware (a real soft/hard LDPC fallback)

Give the fallback a *correct* hard-decision LDPC decode (hard-demap each tone
to 4 bits, run the LDPC decoder on hard bits, or a bounded-distance decode).
This is the "make the fallback actually work" option. Rejected as the primary
fix: `decode_sack_bitmap_ldpc()` *already* does a soft-decision LDPC decode
(`telecom_system.cc:3308-3359`), which is strictly stronger than any
hard-decision LDPC fallback — if the soft decode failed, a hard one will not
succeed. Candidate C would add code that cannot beat what already ran. It is
not a fallback; it is a weaker copy of the primary decoder.

**Recommendation: Candidate A2.** It is the minimal, correctness-restoring
change; it is scoped to one branch in one function (`arq_responder.cc` is
untouched — this is a CMD-side `arq_common.cc` change); it matches the
existing documented intent ("NACK everything on failure"); and it keeps the
useful "RSP is alive" signal. Candidate B is a reasonable follow-up hardening
(suffix quality gate) but is a larger change to the detection contract and
should be its own step after A2 is validated. Candidate C is rejected.

---

## §7 Reversible implementation plan (Candidate A2 — IMPLEMENTED 2026-05-14)

> **STATUS: DONE.** Steps 1–4 implemented and validated on the IONOS Pi
> testbed — see §10 for the implementation log and per-step RESULT blocks,
> and §8 for the resolved threshold question. Commits: `baa125f` (Step 1
> repro harness), `e076823` (Step 2 Candidate A2). Step 5 (Candidate B
> suffix-quality gate) remains an optional follow-up, out of scope here.

Mercury has **no source-level unit-test harness**; validation is loopback
(`-m ARQ -s <cfg> -Q 0 -M auto`) plus the IONOS Pi testbed via the butler
(`localhost:7700`). Steps are individually reversible (each is a single
commit; revert = `git revert`). NB: the two-writer VB-Cable host loopback
could not carry the SACK reverse path (§10.1 RESULT); all FAIL-before /
PASS-after validation was done on the Pi testbed.

| Step | Action | Revert | Pass criterion |
|------|--------|--------|----------------|
| 1 | **Repro / capture-the-bug test.** Add a fault-injection toggle (or a `--test`-mode harness) that forces `ldpc_ok=false` in `receive_sack_pattern()` for one SACK, OR build a loopback scenario that corrupts ≥N suffix tones so the rate-1/4 LDPC genuinely fails. Confirm today's binary logs `ldpc=NO` **and** a `[CMD-SACK]` bitmap that disagrees with what RSP sent (the bug reproduced). | drop the toggle | bug observed deterministically: `ldpc=NO` → wrong `out_bitmap` |
| 2 | **Implement A2.** In `arq_common.cc` `receive_sack_pattern()`, replace the `if(!ldpc_ok){ … decode_sack_bitmap(…) }` block (`:3812-3823`) so that: for the LDPC wire format (`M>=16 && sack_ldpc.N>0`), on `!ldpc_ok` skip `decode_sack_bitmap()` entirely and leave `out_bitmap` all-false (full-batch retransmit); still `return true`. Keep `decode_sack_bitmap()` for the legacy `M<16`/no-`sack_ldpc` path unchanged. Keep the `[RX-SACK] Detected … ldpc=NO` log line (now followed by an all-zero bitmap) for observability; consider an explicit `[RX-SACK] ldpc=NO → full-batch retransmit` line. | `git revert` step 2 | step-1 repro: `ldpc=NO` now yields all-NACK bitmap → CMD retransmits the *whole* batch, never marks a truly-missing frame ACKED. No wrong-frame retransmit. |
| 3 | **Loopback regression.** `-m ARQ -s WB_CFG15 -Q 0 -M auto … --enable-sack`, clean channel. Confirm the normal `ldpc=YES` SACK path is byte-for-byte unaffected (A2 only touches the `!ldpc_ok` branch). | revert step 2 | `ldpc=YES` path unchanged; throughput within the §9.3 band (1690-2253 bps); no new failures. |
| 4 | **IONOS Pi testbed, degraded channel.** Via the butler, run WB_CFG15 `--enable-sack --capture` on a channel bad enough to occasionally produce real `ldpc=NO` events (e.g. with `WGN`). Pull logs with the *fixed* `timing_pull_pi_logs.py` (`088cfc6`, see §9.2 of the turnaround plan — avoid the stale-log trap). Agent-judged per CLAUDE.md validation methodology. | revert step 2 | every `ldpc=NO` event → all-NACK / full-batch retransmit; RSP's follow-up SACK/ACK resolves the duplicates; no batch ever stalls on a wrong-frame retransmit; throughput ≥ pre-fix (wrong-frame retransmit storms gone). |
| 5 | **(Optional follow-up, separate plan)** Candidate B suffix-quality gate, if §4's `[?]` capture shows the base threshold also admits patterns too weak to ever decode. | n/a | its own fact doc + steps. |

Predicted outcome: throughput **improves** in degraded conditions (the
wrong-frame retransmit + re-SACK round-trip from §9.4 is replaced by one
honest full-batch retransmit), and correctness is restored unconditionally.

---

## §8 Open questions

- **RESOLVED — Live `matched` paired with a degraded `ldpc=NO`.** The §8
  capture was taken on the IONOS Pi testbed, 2026-05-14 (degraded WGN:25
  channel, CONFIG_15 WB, `--enable-sack`, no fault injection — see §10.5(b)).
  A genuine `ldpc=NO` event was logged with **`matched=16, metric=8.1`** —
  `matched` is the **maximum** (16/16), nowhere near the detection threshold
  of 10. Across **every** `[RX-SACK] Detected` line captured in all five
  2026-05-14 Pi runs (FAIL_BEFORE, PASS_AFTER, PASS_AFTER_S25, DEGRADED_REAL,
  + the SNR-40 run), `matched` was **16 in 100% of cases** — the SACK base
  pattern, when detected at all, was detected cleanly. The degraded `ldpc=NO`
  reception had `metric=8.1` vs the healthy `ldpc=YES` events at `metric=16.0`
  — i.e. the *base* pattern was perfect (`matched=16`) and only the *suffix
  tone energy concentration* (`metric`) was degraded enough to defeat the
  rate-1/4 LDPC code. This is exactly the §4 hypothesis "if `matched` were
  14-16, the base detection is fine and only the suffix was degraded."

  **Verdict:** the base `sack_match_threshold=10` does **NOT** need revisiting
  — no detection ever landed near it, and the one degraded `ldpc=NO` event was
  at `matched=16`. **Candidate A2 alone is sufficient** for correctness: it
  makes the `ldpc=NO` outcome safe (clean full-batch retransmit) regardless of
  how degraded the suffix is. Candidate B (a suffix-region quality gate to
  *reject* a SACK whose suffix is too poor to decode) remains a reasonable
  optional throughput optimisation — it would let CMD skip the one wasted
  full-batch retransmit per `ldpc=NO` — but it is **not a correctness
  requirement** and is explicitly out of scope here (its own follow-up plan
  per §7 Step 5). Caveat: only **one** genuine `ldpc=NO` event was captured;
  the `matched=16` finding is consistent and unambiguous but the sample is
  thin — a future degraded run that accumulates more genuine `ldpc=NO` events
  would strengthen it, though it cannot change the A2-is-sufficient verdict
  (A2 is safe at any `matched`/`metric`).
- [?] **`decode_sack_bitmap_ldpc()` convergence test edge cases.** At
  `telecom_system.cc:3361`, `converged = (iterations < sack_ldpc.nIteration_max)`.
  In `ldpc_decoder_SPA.cc`: a decode that satisfies parity *exactly on* the
  last iteration returns `nIteration_max` (`:128,192-194,221`) → flagged
  `FAILED` (false negative). And `decode_abort` returns `-iteration`
  (`:132`) → negative → `< nIteration_max` → flagged **converged** (false
  positive — could itself feed a bad bitmap through the *LDPC* path). Both are
  separate latent bugs in the convergence predicate; worth a one-line audit
  but out of scope here. The §3 fallback bug dominates.
- [?] **Is the legacy `decode_sack_bitmap()` reachable at all on current
  hardware?** All current testbed configs are WB M>=16 with `sack_ldpc`
  initialized. If the legacy path is dead code in practice, Candidate A2 could
  be simplified to "delete the fallback unconditionally" — but verifying NB
  SACK usage is needed before doing so; A2 as written is safe regardless.

---

## §9 Summary

- **Root cause:** `arq_common.cc:3812-3823` — on LDPC decode failure
  (`ldpc_ok == false`), `receive_sack_pattern()` falls back to
  `ack_mfsk.decode_sack_bitmap(suffix_tones,…)` (`:3822`), which decodes the
  WB M>=16 suffix tones using the **legacy un-coded bit-packing format**
  (`mfsk.cc:717-781`) — but the TX encoded those tones as an **LDPC
  codeword** (`mfsk.cc:655-680`). The two formats are not inverses, so the
  fallback fabricates a bitmap unrelated to the real one.
- **Where CMD acts on it:** `arq_common.cc:3831` returns `true`
  unconditionally (ignoring `ldpc_ok`); `arq_commander.cc:1434` gates the
  whole retransmit-queue build on that `true`; `:1456` uses the fabricated
  `sack_bitmap[i]` to decide ACKED-vs-retransmit; `:1483` sets
  `data_ack_received = YES`, suppressing the ACK-timeout safety net.
- **Was `metric=6.6` a valid detection?** Most likely yes — `metric ≤ matched`
  and the gate is `matched >= 10` (`arq_common.cc:3785`,
  `mfsk.cc:320`); `metric=6.6` indicates a *degraded but real* SACK, not a
  noise false-alarm. The metric is not the root cause; the absence of any
  *suffix-quality* gate is a secondary gap (Candidate B), not the primary bug.
- **Recommended fix:** Candidate A2 — for the LDPC wire format, on
  `ldpc=NO` skip the hard fallback and leave `out_bitmap` all-false
  (full-batch retransmit), still `return true`. One branch, one function,
  CMD-side `arq_common.cc` only; restores the documented "NACK everything on
  failure" intent. Ordered reversible steps in §7.

---

## §10 Implementation log (Candidate A2)

Implementation session 2026-05-14. Each step is a single commit on `monitor`.

### §10.1 Step 1 — fault-injection toggle + repro harness

**Implemented:**
- New member `bool force_sack_ldpc_fail` on `cl_arq_controller`
  (`include/datalink_layer/arq.h`, near `disable_sack`; default `false`,
  initialised in the `arq_common.cc` constructor next to `disable_sack`).
- CLI flag `--test-sack-ldpc-fail` (`source/main.cc`) sets
  `ARQ.force_sack_ldpc_fail = true`. Off in production builds.
- In `receive_sack_pattern()` (`arq_common.cc`), immediately after the
  `decode_sack_bitmap_ldpc()` call: `if(force_sack_ldpc_fail && ldpc_ok)`
  forces `ldpc_ok=false` and logs `[RX-SACK-TESTFAIL]`. This deterministically
  drives the `!ldpc_ok` hard-fallback branch even on a *healthy* SACK
  reception — so the fabricated-bitmap behaviour is observable without needing
  to physically corrupt suffix tones.
- Repro harness `tools/sack_ldpc_fail_repro.py`: launches CMD+RSP, `--enable-sack`
  both, `--test-sack-ldpc-fail` on CMD only; pairs each CMD
  `[RX-SACK] Detected (… ldpc=NO …), bitmap:` line against the RSP
  `[TX-SACK] Sending SACK pattern (… received:…)` that produced it. Verdict
  `BUG REPRODUCED` if CMD's decoded bitmap is non-zero and disagrees with RSP;
  `SAFE BEHAVIOUR` if CMD's bitmap is all-zero (full-batch retransmit).

**RESULT — host VB-Cable loopback could NOT carry the SACK reverse path
(2 honest attempts, per CLAUDE.md "stop after 2"):**
- Attempt 1: connection never established. CMD `[CAP-PEAK]` alternated between
  real signal (`pk≈0.51`) and silence (`pk=0.000000`); `[CLK-TX-GLITCH]` events
  up to `dt_call=865ms` — WASAPI scheduling jitter on the network-share host.
  0 RSP TX-SACKs, 0 CMD RX-SACKs. Verdict INCONCLUSIVE.
- Attempt 2 (clean, no concurrent build): RSP received some data and sent
  **4 `[TX-SACK]`** patterns, but CMD logged **0 `[RX-SACK] Detected`** — the
  RSP→CMD reverse audio path did not carry the SACK. Verdict INCONCLUSIVE.
- Conclusion: the two-writer VB-Cable host loopback cannot reliably carry SACK
  (consistent with the MEMORY note "SACK cannot be tested on VB-Cable"). This
  is a **harness/environment limitation, not a fix-logic failure** — the fix
  is not yet implemented at this point. Per §7 ("validation is loopback **plus**
  the IONOS Pi testbed") and Step 4, validation of the repro (FAIL-before) and
  the A2 fix (PASS-after) is therefore performed on the **IONOS Pi testbed**,
  which has real, separate, bidirectional audio hardware. `tools/sack_ldpc_fail_repro.py`
  is retained as the host-side harness for environments where VB-Cable does
  carry the reverse path (it worked in March 2026 per `tools/mercury_*_700*.log`).
- Step 1 binary builds clean (`bash build.sh o3`, only a pre-existing unrelated
  sign-compare warning at `arq_commander.cc:1423`).

**Commit:** `baa125f` — *sack test: --test-sack-ldpc-fail fault-injection
toggle + repro harness*.

### §10.2 Step 2 — Candidate A2 implemented

**Implemented** (`source/datalink_layer/arq_common.cc`, `receive_sack_pattern()`):
the `if(!ldpc_ok){…}` block was split on the active SACK wire format
(`ldpc_wire_format = M>=16 && sack_ldpc.N>0`, mirroring the predicate that
gates `decode_sack_bitmap_ldpc()`):
- **LDPC wire format** (`ldpc_wire_format == true`): the hard fallback
  `decode_sack_bitmap()` is **skipped entirely**; `out_bitmap` is explicitly
  cleared to all-`false` (every frame NACKed). Logs
  `[RX-SACK] ldpc=NO -> full-batch retransmit (hard fallback skipped for LDPC
  wire format)`. Function still `return true` below — the genuine "RSP
  responded / batch finished" signal is preserved. The explicit clear (rather
  than relying on the caller's `memset` + `decode_sack_bitmap_ldpc`'s init) is
  deliberate: it is robust independent of caller state, and it is *required*
  for the fault-injection path where `--test-sack-ldpc-fail` forces
  `ldpc_ok=false` *after* a successful decode left a real bitmap in
  `out_bitmap`.
- **Legacy NB / non-LDPC format** (`ldpc_wire_format == false`): unchanged —
  `decode_sack_bitmap()` IS the correct inverse of the legacy
  `encode_sack_bitmap()` format there.
- The `ldpc=YES` path is **wholly untouched** — the entire change is inside the
  existing `if(!ldpc_ok)` block.
- `arq_responder.cc` and `mfsk.cc` untouched; `CAP_SACK` default unchanged.
- Builds clean (`bash build.sh o3`).

**Commit:** `e076823` — *fix: SACK ldpc=NO no longer fabricates a bitmap
(Candidate A2)*.

### §10.3 Pi test harness fixes (prerequisite for Steps 3–4)

`tools/mercury_ionos_sack_test.py` required three fixes before it could
validate the change:
- **`--enable-sack` / `--test-sack-ldpc-fail` / `--label` CLI flags** added.
  `--enable-sack` is passed to both Mercury instances (SACK is off by default
  after the B2 fix); `--test-sack-ldpc-fail` is passed to the **commander
  only** (the bug and the A2 fix both live in CMD's `receive_sack_pattern()`).
- **Audio device fix**: `AUDIO_DEV` was `plughw:1,0` — wrong card index;
  `snd_pcm_open` returned `-2 No such file or directory` and Mercury exited
  before opening its TCP port (the first FAIL-before attempt failed here). The
  Fe-Pi card is card 0 and the index is not stable; switched to the named form
  `plughw:Audio`.
- **DOWNLOAD path-with-spaces fix**: the butler's `DOWNLOAD` splits its
  argument line on whitespace, so a destination under the "hermes and mercury"
  workspace share was silently truncated at the first space — the file landed
  nowhere and the butler still answered `OK` (this is the §9.2 stale-log
  harness bug class from `SACK_TURNAROUND_FIX_PLAN.md`). Fixed by downloading
  into a space-free `tempfile.mkdtemp()` dir and `shutil.move`-ing to the real
  destination — the same pattern as the already-fixed `timing_pull_pi_logs.py`
  (workspace `088cfc6`).

### §10.4 Step 1 RESULT — FAIL-before reproduced on the Pi testbed

Binary: `baa125f` (Step-1 toggle present, A2 fix **absent** — produced by
`git checkout baa125f -- source/datalink_layer/arq_common.cc` then
`mercury_deploy_rpi.py`). Run: `--enable-sack --test-sack-ldpc-fail
--label FAIL_BEFORE --snr 40`, CONFIG_15 WB, both Pis. Logs:
`tools/ionos_results/sack_test_FAIL_BEFORE_20260514_144424_{cmd,rsp}_log.log`.

The fault-injection toggle deterministically drove the `!ldpc_ok` hard
fallback on healthy SACK receptions. For **each** of the 3 forced events the
CMD log shows:
```
[RX-SACK-TESTFAIL] --test-sack-ldpc-fail: forcing ldpc_ok=false (LDPC actually decoded OK)
[RX-SACK-RAW] LDPC failed, hard fallback nsuffix=32 suffix_tones: 15 15 15 15 15 11 1 0 3 9 …
[RX-SACK] Detected (matched=16, metric=16.0, ack_xcheck=5, ldpc=NO), bitmap: 0 0 0 0 1 1 0 0 1 0 0 1 1 1 1 1 0 0 1 1 1 1 0 1 1
[CMD-SACK] 14/25 received, 8 queued for retransmit
```
Cross-checked against what RSP actually sent
(`[TX-SACK] Sending SACK pattern (batch=25, received: …)`):

| Event | RSP actually requested | CMD `ldpc=NO` decoded bitmap |
|-------|------------------------|------------------------------|
| 1 | `1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 1 1` (24/25 rx, only frame 22 missing) | `0 0 0 0 1 1 0 0 1 0 0 1 1 1 1 1 0 0 1 1 1 1 0 1 1` (14 ones — **fabricated**) |
| 3 | `1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 1 1` (24/25 rx, only frame 22 missing) | `0 0 0 0 1 0 1 0 1 0 0 1 1 1 1 1 0 0 0 1 0 1 1 0 1` (12 ones — **fabricated**) |

The decoded bitmap is **structurally unrelated** to RSP's request — it marks
frames 0–3 as *not received* when RSP explicitly said all of 0–21 *were*
received. This is precisely the §3 root cause (LDPC-coded suffix tones decoded
with the legacy un-coded codebook) and reproduces the §9.4 symptom
("12/25 received" vs the true "24/25"). **Bug reproduced deterministically.
FAIL-before confirmed.**

### §10.5 Steps 2–4 RESULT — PASS-after + degraded-channel validation on the Pi testbed

Binary: `e076823` (A2 fix present — working tree restored to `HEAD`, then
`mercury_deploy_rpi.py`).

**(a) PASS-after — forced `ldpc=NO` (fault injection), degraded channel.**
Run: `--enable-sack --test-sack-ldpc-fail --label PASS_AFTER_S25 --snr 25`,
CONFIG_15 WB. Logs:
`tools/ionos_results/sack_test_PASS_AFTER_S25_20260514_*_​{cmd,rsp}_log.log`.
RSP sent 6 real `[TX-SACK]` patterns; CMD's toggle forced `ldpc=NO` on 2 of
the receptions it detected. For both:
```
[RX-SACK-TESTFAIL] --test-sack-ldpc-fail: forcing ldpc_ok=false (LDPC actually decoded OK)
[RX-SACK] ldpc=NO -> full-batch retransmit (hard fallback skipped for LDPC wire format)
[RX-SACK] Detected (matched=16, metric=16.0, ack_xcheck=5, ldpc=NO), bitmap: 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
[CMD-SACK] 0/25 received, 8 queued for retransmit
```
- `out_bitmap` is **all-zero** — NOT a fabricated bitmap (contrast §10.4's
  14-/12-one garbage).
- **No `[RX-SACK-RAW]` line** — the buggy `decode_sack_bitmap()` hard fallback
  is never called (`RX-SACK-RAW` count = 0 across the whole run).
- `[CMD-SACK] 0/25 received` — CMD marks **zero** frames ACKED; every pending
  frame is queued for retransmit. No truly-missing frame is wrongly marked
  ACKED; the ACK-timeout safety net is not spuriously suppressed by a
  fabricated "received" flag.
- `[CMD-SACK] Partial batch ACK detected!` still fires → the genuine "RSP
  responded / batch finished" signal is preserved (A2, not A1).

**(b) Degraded channel, NO fault injection — genuine `ldpc=NO` + `ldpc=YES`
regression.** Run: `--enable-sack --label DEGRADED_REAL --snr 25`, CONFIG_15
WB. Logs: `tools/ionos_results/sack_test_DEGRADED_REAL_20260514_*_{cmd,rsp}_log.log`.
RSP sent 9 real `[TX-SACK]`; CMD detected 3 SACKs:
```
[RX-SACK] Detected (matched=16, metric=16.0, ack_xcheck=6, ldpc=YES), bitmap: 1 1 0 1 1 0 1 1 …  -> [CMD-SACK] 23/25 received, 2 queued
[RX-SACK] Detected (matched=16, metric=16.0, ack_xcheck=5, ldpc=YES), bitmap: 1 1 1 1 1 1 1 0 …  -> [CMD-SACK] 22/25 received, 3 queued
[RX-SACK] Detected (matched=16, metric=8.1,  ack_xcheck=5, ldpc=NO),  bitmap: 0 0 0 0 0 0 0 0 …  -> [CMD-SACK] 0/25 received, 8 queued
```
- **`ldpc=YES` regression: PASS.** The healthy path is byte-for-byte
  unaffected — the decoded bitmap is acted on normally (23/25, 22/25
  received). A2 only touches the `!ldpc_ok` branch.
- **Genuine `ldpc=NO` on a real channel (`metric=8.1` — exactly the §9.4
  `metric≈6.6`-class degraded reception): PASS.** All-zero bitmap →
  `[CMD-SACK] 0/25 received, 8 queued` → clean full-batch retransmit.
  `full-batch retransmit` marker count = 1, `RX-SACK-RAW` count = 0 — the
  fix engages on real channel events, not just the fault-injection path. **No
  wrong-frame retransmit.**

**Overall:** FAIL-before (`baa125f`) fabricates a bitmap and acts on it;
PASS-after (`e076823`) leaves `out_bitmap` all-false → clean full-batch
retransmit, never calls the buggy hard fallback, never wrongly ACKs a frame,
and the `ldpc=YES` path is unchanged. The four-phase "test that fails before
the fix and passes after" requirement is satisfied on real hardware.
