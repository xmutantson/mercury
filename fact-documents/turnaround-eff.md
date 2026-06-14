# Turnaround-efficiency: sub-peak/trial-multiplier kill + syndrome early-term — fact document

Branch `feat/turnaround-eff` off `fix/ldpc-decode-accel @65eb2be`
(which is off `fix/break-fh-gate @465956e`; carries A=forward-backward CN,
B=OFDM iter-cap, F=-mcpu). Worktree `C:/Users/kamer/mercury_wt/turnaround-eff`.

## §0 Problem (MEASURED, firsthand HW [TIMING] logs — inherited from ldpc-decode-accel.md §0)

Held-CFG16 reverse-ACK turnaround miss is a **DECODE-LATENCY spike**. On clean
WGN the CFG16 fail is **LDPC NON-CONVERGENCE** (`cfg16_decode_loss_is_ldpc_bp`
memory: 117/117 fail-frames iter=101, meanH~0.98, no CFO), NOT a timing miss. A
non-converging frame is decoded MULTIPLE times at different timing positions:

1. The trial loop (`telecom_system.cc:2178` `while sync_trials<=effective_trials_max`)
   re-runs sync→CE→demod→**full SPA** per trial. Pre-SPA gates (SKIP-H `:2826`,
   SUBPEAK-REJECT `:2816`, SKIP-VAR) already reject low-`mean_H` positions, so the
   wasted SPA runs are the positions that pass the gates but still don't converge.
2. The ±1-OFDM-symbol sub-peak goto (`:3169-3196`, `subpeak_recover_phase<2`):
   on an iter-cap FAIL with `coarse_metric>=0.97 && mean_H>=0.5`, it re-runs the
   data-FIR→demod→**full SPA** at `delay+sym` then `delay-sym`, up to 2 extra
   full decodes — UNCONDITIONALLY on the metric gate, NOT gated on whether the
   shift actually IMPROVES anything.

So a single clean-WGN non-converging frame pays ~4.5 full 100-iter (now
iter-capped-30) SPA decodes = ~6-7 s injected, pushing the SACK past the 3524 ms
CMD window. Lever B (iter-cap) bounds EACH decode; this branch attacks the
**MULTIPLIER** (how many full decodes a doomed frame pays) and adds a
**per-decode non-convergence early-term** so even the in-budget decodes quit at
~iter 8-12 instead of running to the cap.

Two PHY-local, lossless, env-gated levers + ONE shared detector:
- **#1** SUB-PEAK / TRIAL MULTIPLIER KILL (env `MERCURY_SUBPEAK_KILL`).
- **#3** SYNDROME-WEIGHT EARLY-TERM (env `MERCURY_SYND_EARLYTERM`).
- **Shared detector**: the non-convergence detector implemented ONCE in
  `decode_SPA`; both #1(c) and #3 route through it.

## §1 Prior art (CLAUDE.md §1 research-first)

Non-convergence / early-stop for BP LDPC — the citable principle:

- **Check-Sum Variation (CSV)**: D. Li, X. Huang et al., "A Unified Early
  Stopping Criterion for Binary and Nonbinary LDPC Codes Based on Check-Sum
  Variation Patterns," IEEE Communications Letters 14(11):1053-1055, 2010.
  KEY FINDING: *for decodable blocks the syndrome weight (check-sum)
  monotonically decreases as iteration count grows; for undecodable blocks it
  fluctuates/plateaus in a range of magnitudes.* Detect non-convergence by the
  ABSENCE of a decreasing trend.
- **CMM** (Convergence of Mean Magnitude), Shin/Heo/Kim and Kienle/Wehn: mean
  |LLR| stagnation as an alternative non-convergence signal.
- Shin et al., "New stopping criteria for iterative decoding of LDPC codes in
  H-ARQ systems," Int. J. Commun. Syst. 26, 2013: undecodable-frame detection.

Mercury already computes the syndrome weight `nOnes` every SPA iteration and
breaks on `nOnes==0` (`ldpc_decoder_SPA.cc:264-281`) — that is the SUCCESS half
of CSV. We add the FAIL half: a non-convergence detector on the `nOnes`
trajectory. We use CSV (syndrome-weight trend), NOT CMM, because `nOnes` is
ALREADY computed every iteration at zero added cost; mean-|LLR| would add a
full-N reduction per iteration.

## §2 SHARED DETECTOR — `spa_nonconverge_detect()` (ONE implementation)

In `ldpc_decoder_SPA.cc`. Pure function over the syndrome-weight history; no I/O,
no globals. Signature (file-local):

    // returns true => declare this frame a non-converger, quit now.
    static inline bool spa_nonconverge_detect(
        int iteration, int nOnes, int& min_nones, int& best_synd_iter,
        int warmup, int confirm)

State carried by the caller across iterations: `min_nones` (best=lowest
syndrome weight seen so far, init INT_MAX), `best_synd_iter` (iteration that set
it, init 0).

Logic (CSV, Li/Huang 2010 + the FLOOR correction below):
1. Update the running minimum: if `nOnes < min_nones` then `min_nones=nOnes`,
   `best_synd_iter=iteration` (a NEW minimum = the trajectory is still
   descending).
2. **Guard 1 (syndrome-depth >= 5)**: do nothing until `iteration >= warmup`
   (warmup >= 5).
3. **Guard 2 (FLOOR = P/2)**: only trip when `min_nones > floor` — the running
   minimum is STILL above half the check count, i.e. the frame never got remotely
   close to a codeword. (This is the design correction — see "FLOOR" below.)
4. **Guard 3 (confirm window)**: declare non-convergence when
   `iteration - best_synd_iter >= confirm` — no new minimum for `confirm` iters.

### ~~Original (running-minimum-only) design~~ — REFUTED, false-early-stop
~~The first design was Guards 1+confirm only (no floor): "trip when no new
syndrome minimum for `confirm` iters after warmup>=5, warmup=8/confirm=4".~~
MEASURED FALSE on the 27-cell grid: it killed 1-7 real convergers per
(warmup,confirm) (e.g. chan=0 EsN0=16 dropped 62/62 -> 59/62). ROOT CAUSE
(syndrome-trace, MERCURY_SYND_TRACE): **a real CFG16 converger is NOT monotone
per-iteration in flooding BP.** Trace `conv@45` = `50 54 41 43 47 48 46 33 36 38
33 45 44 40 37 ... 8 0` — it oscillates in the 30s-50s for 40 iterations, then
converges at iter 45; `conv@62` finds min=2 at iter ~19, RISES back to 62, then
finally converges at 62. These are INDISTINGUISHABLE from a true non-converger
by any "no-new-minimum" window. The CSV paper's "monotone decrease" is a LOOSE,
EVENTUAL property, not per-iteration — so the running-minimum trend alone is not
a lossless discriminator on this code/channel.

### FLOOR = P/2 (the lossless discriminator)
The discriminator that IS lossless: a real converger ALWAYS dips its running
minimum BELOW P/2 (= 55 for CFG16, P=111 checks) on the way down — even
`conv@45`/`conv@62` reach single digits. A truly-stuck frame (the clean-WGN
non-convergence §0 targets) never gets close: its running minimum stays HIGH
(the det-floor non-convergers are PINNED at nOnes=111=P, never moving). So we
trip ONLY when `min_nones > P/2`.

SWEEP (27-cell grid, 1674 codewords, 416 convergers / 1258 failers):
| floor | conv LOST | failers caught | mean catch-iter |
|-------|-----------|----------------|-----------------|
| P/8 (~13) | 3-7 (LOSSY) | ~1250 | ~12-30 |
| P/4 (~27) | 0-2        | ~1206 | ~12-30 |
| **P/2 (~55)** | **0 at every (warmup,confirm)** | **1117 (88.8%)** | **~12** |
floor=P/2 loses ZERO convergers at warmup∈{12,16,20,30} × confirm∈{8,12,16}
while catching 89% of failers at mean iter ~12. PROVEN lossless: the 27-cell
`[SFO-GRID-CODED]` render with MERCURY_SYND_EARLYTERM=1 is BYTE-IDENTICAL to OFF
(every converger decodes, every BER unchanged, every fail still FAIL), while the
det-floor/cliff cells trip the detector at iter 12 (et_iter_max=12).

### Aggressiveness modes (warmup, confirm) — floor=P/2 for both
- **Mode 1** (`MERCURY_SYND_EARLYTERM`, #3, every OFDM decode): `warmup=12`,
  `confirm=8`, `floor=P/2`. Swept LOSSLESS; quits a doomed frame at ~iter 12.
- **Mode 2** (#1(c) speculative wrong-position trial): `warmup=8`, `confirm=6`,
  `floor=P/2`. More eager (a wrong-position decode is the PINNED-high case);
  still depth>=5, still floor=P/2 => lossless (a genuinely-helpful sub-peak that
  decodes dips below P/2 and is never killed).
- Overridable via `MERCURY_SYND_WARMUP` / `MERCURY_SYND_CONFIRM` /
  `MERCURY_SYND_FLOOR` (percent of P) for the campaign sweep; mode-2 fixed.

### Return-value correctness (the critical invariant)
When the detector trips, `decode_SPA` returns **`nIteration_max + 1`** — the
EXACT value a natural cap-out returns (loop exits with `iteration ==
nIteration_max+1`). Every consumer's FAIL predicate `iterations_done >
(nIteration_max-1)` (`:3147`, `:3172`, harness `:7278`) therefore fires
IDENTICALLY. An early-terminated non-converger is classified FAIL exactly like a
cap-out — never spuriously OK. The CRC16 self-check (`:3140-3143`, runs on ALL
non-zero frames) is the second safety net (garbage hard bits → CRC!=0 → FAIL).
The ACTUAL early-term iteration (for measurement) is exposed separately via
`cl_ldpc::last_early_term_iter` (a member, default -1) so the test can prove the
wasted iterations were skipped without perturbing the production return value.

## §3 #3 SYNDROME-WEIGHT EARLY-TERM (env MERCURY_SYND_EARLYTERM)

`decode_SPA`: after the per-iteration syndrome weight `nOnes` is computed
(`:264-276`) and BEFORE the `nOnes==0` break is checked, call the shared detector
in mode 1 when `MERCURY_SYND_EARLYTERM` is set. On trip: record the real iter in
`g_last_early_term_iter`, return `nIteration_max+1`. Read the env ONCE (static).

Gate: env unset/0 => detector never called => the loop runs to the cap exactly
as before (byte-identical). The `min_nones`/`best_synd_iter` state vars are
declared but the detector early-returns on warmup so they are pure scratch when
off — and the env-off path skips the call entirely.

## §4 #1 SUB-PEAK / TRIAL MULTIPLIER KILL (env MERCURY_SUBPEAK_KILL)

`telecom_system.cc:3169-3196` (the ±1-sym sub-peak goto) + the speculative-trial
early-out. When `MERCURY_SUBPEAK_KILL` is set:

- **(b) Gate the ±1-sym goto on a coarse-metric IMPROVEMENT and cap to 1.**
  Today the goto fires up to 2 times (`subpeak_recover_phase<2`) on the metric
  gate alone. With the kill: cap to ONE probe (`subpeak_recover_phase<1`), and
  only take it if the candidate shift is plausible (delay in range — already
  checked). We pick the SINGLE best direction using the already-computed
  `coarse_metric`/`mean_H` rather than blindly trying +sym then -sym. (a) The
  ranking input is `mean_H` (a real timing sub-peak that helps lands ~1 sym off
  and leaves enough preamble energy for a healthy `mean_H`; a hopeless shift
  does not). We keep the +sym probe (the empirically-dominant sub-peak side per
  the existing comment `:3163` "Sub-peaks ... land at OFDM-symbol-aligned
  offsets") but DROP the second (-sym) probe — halving the multiplier — UNLESS
  the first probe improved `mean_H` (then the architecture already exits via the
  OK path). PRESERVES the aligned/correctable case: a real sub-peak that DOES
  help is still tried once (+sym), and if it decodes it exits OK exactly as
  before.
- **(c) Early-out a wrong-position decode via the SHARED detector.** The
  sub-peak probe (and, when `MERCURY_SUBPEAK_KILL` set, the speculative extra
  trials) run `ldpc.decode` with `cl_ldpc::early_term_speculative=true` so the
  decoder applies the shared detector in MODE 2 (eager) — a wrong-position
  decode bails ~iter 8 instead of burning to the cap. The flag is set right
  before the speculative decode and cleared right after, so the PRIMARY
  (trial-0 / aligned) decode is NEVER early-termed by #1 (only by #3 if THAT
  env is set). This is where #1(c) and #3 SHARE the one detector.

Gate: env unset/0 => `subpeak_recover_phase<2` (2 probes), no speculative flag
=> byte-identical to base. The metric-improvement / cap-to-1 / speculative-flag
all live behind `subpeak_kill`.

### Why this preserves decode correctness on the aligned/correctable frame
- The PRIMARY decode (trial 0, aligned position) is unchanged: no speculative
  flag, no early-term unless `MERCURY_SYND_EARLYTERM` is independently set, and
  even then mode-1 only quits a frame whose syndrome has stalled (a correctable
  frame converges → 0 and breaks first).
- A genuine timing sub-peak that DOES help is still tried (+sym probe) and still
  decodes (the eager mode-2 detector only quits the WRONG-position decodes,
  which by construction don't converge; the right shift converges → break).
- The -sym probe we drop was the SECOND, rarer side; dropping it costs at most
  the frames whose ONLY good shift is -sym (covered by the SACK-retx path, which
  the §0 comment already names as the fallback). Measured cost in §6.

## §5 Cross-layer data-flow audit (CLAUDE.md §5)

Shared state touched: `ldpc.nIteration_max` (read-only here),
`receive_stats.iterations_done`, `subpeak_recover_phase`, and the NEW
`cl_ldpc::{early_term_speculative,last_early_term_iter}`.

1. **Producers of `iterations_done`**: `ldpc.decode` return (`:3047` primary,
   `:3114` turbo refine). NEW: on detector trip, `decode_SPA` returns
   `nIteration_max+1` (= natural cap-out value). Producer invariant PRESERVED
   (same sentinel). (Line numbers as of this branch's working tree.)
2. **Consumers of `iterations_done`**: fail-classify `:3167`, sub-peak gate
   `:3195`, harness `capped` `:7333`, turbo gate `:3065` (`>0` —
   `nIteration_max+1>0` ✓), `:3080` (monotone surrogate — a fail value is the
   worst case, monotone-safe). ALL read the `> nIteration_max-1` FAIL predicate
   or `>0`; the early-term sentinel satisfies every one identically.
3. **Valid states**: `iterations_done` ∈ {-1 (pre-init), [1,nIteration_max]
   (converged), nIteration_max+1 (fail/cap), -iteration (parallel abort)}. The
   detector adds NO new state value (reuses nIteration_max+1).
4. **Invariants**: (INV-A) FAIL ⇔ `iterations_done > nIteration_max-1`. The
   detector returns nIteration_max+1 ⇒ FAIL ⇒ INV-A held. (INV-B) the CRC16
   self-check independently catches wrong-codeword convergence ⇒ even if a
   future consumer ignored the iter count, the byte-path is protected.
   (INV-C) `early_term_speculative` is a transient flag: set immediately before a
   speculative decode, cleared immediately after; no other code path reads it
   between. Default false. (INV-D) `last_early_term_iter` is measurement-only;
   no control-flow consumer.
5. **What the fix changes**: ONLY when an env is set. #3 makes some
   primary-decode fails return at iter 8-12 instead of the cap (same FAIL
   classification, fewer iterations). #1 caps the sub-peak multiplier to 1 probe
   and early-terms the speculative decodes. Walked every `iterations_done`
   consumer above: all safe.

## §6 Tests (fail-before / pass-after; CLAUDE.md §3)

`tools/test_turnaround_eff.py` drives the coded SFO-GRID harness (the SAME
vehicle ldpc-decode-accel.md §1 used).

- **TEST-1 (byte-identity, default-off)**: 27-cell grid (3 chan × 3 seed × 3
  EsN0, NSYMB=600 => K=62 codewords/cell), md5 of all `[SFO-GRID-CODED]` lines.
  MEASURED: OFF render md5 = **`4575011ba38e23575ef3f3bbce0703b2`** == base
  @65eb2be render (verified by building the base binary in
  `C:/Users/kamer/mercury_wt/ldpc-accel` and `diff`-ing the renders — BYTE-
  IDENTICAL, zero-line diff). (NB: the ldpc-decode-accel.md §1 md5
  `96bf314c...` was on the OLDER base @465956e at a DIFFERENT NSYMB; the correct
  anchor for THIS branch's grid is `4575011b...`.) PROVES default-off
  byte-identical: with both `MERCURY_SYND_EARLYTERM` and `MERCURY_SUBPEAK_KILL`
  unset the decode path is bit-for-bit the base.
- **TEST-3b (no false-early-stop / lossless)**: with `MERCURY_SYND_EARLYTERM=1`
  the 27-cell `[SFO-GRID-CODED]` render is **byte-IDENTICAL to OFF**
  (md5 `4575011b...`) — every correctable codeword still decodes, every
  `post_FEC_info_BER` unchanged, every fail still classified FAIL. This is the
  §3 correctness gate. FAIL-BEFORE (the refuted running-min design) dropped
  62/62 -> 59/62 on chan=0 EsN0=16; PASS-AFTER (floor=P/2) loses zero.
- **TEST-3a (early-term fires + bounds the doomed decode)**: with
  `MERCURY_SYND_EARLYTERM=1` the det-floor (chan=1) and cliff (chan=0/EsN0=15)
  cells trip the shared detector — `[SFO-GRID-EARLYTERM] tripped=N/62
  et_iter_max=12` — bounding the doomed-frame decode from up-to-101 down to
  iter 12 (~8.4x fewer iterations). FAIL-BEFORE: OFF emits NO `[SFO-GRID-
  EARLYTERM]` line (the detector never runs). The decode still RETURNS
  nIteration_max+1 so the FAIL classification is byte-identical.
- **#1 multiplier**: the SFO-GRID harness runs ONE grid and does NOT exercise the
  RX trial loop / sub-peak goto, so #1's cap-to-1 + speculative-flag are
  control-flow changes proven by (a) code inspection, (b) TEST-1k: `MERCURY_
  SUBPEAK_KILL=1` leaves the coded render byte-identical (the env is inert on the
  decode path — it only changes the trial-loop goto), and (c) the speculative
  mode-2 detector routing through the SAME shared `spa_nonconverge_detect` that
  TEST-3 proved lossless+effective (mode 2 = warmup 8/confirm 6/floor P/2, within
  the lossless envelope). The actual full-SPA-call drop (~4.5 -> ~1.5 on a doomed
  frame: 2 probes -> 1, each bounded by mode-2) and the wall-clock are
  Pi-only-confirmable (§7).

`mercury.exe --test` (exit 0) and `--test-climb-engine` (ALL PASS, 0 failures)
green in BOTH env states.

## §8 LEVER #2 — SPECULATIVE / PROMPT SACK (env MERCURY_SPEC_SACK)

Levers #1 (sub-peak/trial multiplier kill) and #3 (syndrome early-term) BOUND the
per-doomed-frame decode cost. Lever #2 attacks the same turnaround miss from the
ARQ side: it fires the reverse-ACK (SACK) on a **window-fraction DEADLINE timer**
rather than waiting for the whole batch — including any still-decoding /
non-converging frame — to finish the serial `receive()` pump. Frames that have not
yet reached `messages_rx[].status==RECEIVED` at the deadline are reported **bit-0**
in the SACK bitmap and recovered through the EXISTING idempotent partial-SACK +
CMD-retx path. Env `MERCURY_SPEC_SACK`; default-off ⇒ the SACK fires after the
serial loop exactly as today (byte-identical).

### §8.0 Prior art (CLAUDE.md §1)
Standard selective-repeat ARQ semantics: the receiver SACKs which frames it HAS and
HAS-NOT; a not-yet-decoded frame is reported not-received, retransmitted, and merged
(Selective Repeat ARQ, Wikipedia; Baeldung "Selective Repeat Protocol"). Lever #2
applies those semantics but fires the SACK on a RECEIVER-SIDE deadline (a "prompt
ACK"/"ACK timer" in the SR-ARQ literature) instead of waiting for the full data unit
to finish decoding. This is the same discipline as VARA/ARDOP/STANAG fixed-cycle /
self-sync ACK (MEMORY turnaround_solution_design): make a MISS CHEAP — a
still-decoding frame costs ONE retransmit (~one frame-time), not the
~3524ms-window-miss → ~133s D3 crawl. The recovery path itself is unchanged and
already bench-validated + idempotent (arq_commander.cc:3098-3164, applies the bitmap
by SLOT INDEX to messages_tx[]).

### §8.1 The single-threaded pump and the deadline-check site
The RSP receive path is ONE thread: `process_messages_rx_data_control()`
(arq_responder.cc:109) calls `this->receive()` (arq_responder.cc:432), which decodes
ONE frame from the captured audio. `receive()` is BLOCKING — while a clean-WGN
non-converging CFG16 frame is in the SPA trial-loop (~6-7s, §0), the thread is INSIDE
`receive()` and cannot poll any timer. So the deadline cannot be a wall-clock
interrupt; it is checked at the LOOP BOUNDARY — the FIRST line after `receive()`
returns (whether it stored a frame, FAILed, or found nothing). This covers all three
cases. Today the SACK only fires when `receiving_timer.get_elapsed_time_ms() >=
receiving_timeout` (the `else` at arq_responder.cc:1142 → `connection_status =
ACKNOWLEDGING_DATA` → `process_messages_acknowledging_data()` at :41/:1600). Lever #2
adds an EARLIER trigger: force that SAME transition when the window-fraction deadline
has elapsed and the batch is still incomplete.

### §8.2 The deadline (window-fraction, NOT EOB-arrival-only)
The deadline is a FRACTION of the RSP receiving window — NOT "fire on EOB arrival".
A slow / lost / non-converging EOB frame must STILL trigger the SACK, so the gate
cannot key on EOB. The RSP receiving window is sized at
`data_batch_size*message_transmission_time_ms + time_left_to_send_last_frame +
ptt_on_delay_ms` (arq_common.cc:1416) and dynamically re-armed per decoded frame
(arq_responder.cc:1044-1131). Lever #2 fires when
`receiving_timer.get_elapsed_time_ms() >=
SPEC_SACK_WINDOW_NUM/SPEC_SACK_WINDOW_DEN * receiving_timeout` AND the batch is
incomplete (rx_received < expected). Default fraction = 3/4 (env-overridable
MERCURY_SPEC_SACK_NUM / MERCURY_SPEC_SACK_DEN). The 3/4 point is past the bulk of
the legitimate batch airtime (so a clean batch that is merely slow but progressing
is NOT cut short — it will have re-armed the timer forward on each frame) yet early
enough to leave the SACK inside the CMD listen window even when the last frame(s)
are doomed. Because each decoded frame re-arms `receiving_timeout` to "the whole
rest of the batch" (Fix A, arq_responder.cc:1109-1124), a healthy in-progress batch
keeps pushing the deadline forward; the gate only bites once the channel/decode has
genuinely stalled relative to the remaining-batch estimate. The gate ALSO requires
`batch_rx_frame_count >= 1` (at least one frame landed — never SACK an empty batch;
an all-empty batch is the normal "nothing arrived" path that re-arms via :1163-1181)
and `expected >= 2` / `data_batch_size > 1` (the existing SACK-meaningful gate;
single-frame batches use the all-or-nothing EOB receipt, §8.5 below).

### §8.3 How the gate fires (mechanism)
At the deadline, the gate sets `connection_status = ACKNOWLEDGING_DATA` and returns.
The NEXT pump tick enters `process_messages_acknowledging_data()` (the existing
ACK-GATE handler, arq_responder.cc:1600), which builds the SACK bitmap from
`messages_rx[i].status == RECEIVED` AS-IS (arq_responder.cc:1689-1691) — exactly the
post-decode-loop snapshot it builds today. Still-decoding / non-converged frames are
`FREE` (never reached `add_message_rx_data`, arq_responder.cc:103) ⇒ bit-0 ⇒
retransmitted. We do NOT duplicate the ACK-GATE logic; we only move the moment it
runs EARLIER. The transition is identical to the natural timer-expiry transition, so
EVERY downstream consumer (rx_received counter, expected-count inference, bsi-bump,
prev-transfer, SACK_RSP TX) runs byte-for-byte the same code on the same state — the
ONLY difference is the wall-clock instant.

### §8.4 CROSS-LAYER DATA-FLOW AUDIT (CLAUDE.md §5) — the messages_rx[].status chain
Shared state touched: NONE written by lever #2 except `connection_status` (advanced
to a value the natural path ALSO sets, one tick earlier). The chain audited:
`messages_rx[].status` → SACK bitmap (arq_responder.cc:1690-1691) → CMD partial-SACK
retx (arq_commander.cc:3098-3164) → bsi / last_received_end_of_batch_seq / the D5
prev-bump chain (bump_bsi_and_transfer_prev, arq_common.cc:6512-6598).

1. **Producers of `messages_rx[].status`**:
   - `add_message_rx_data` (arq_responder.cc:103) sets `RECEIVED` on a
     successfully-decoded+stored DATA frame. THIS is the only path that flips a slot
     to RECEIVED in a data batch.
   - `init_messages_buffers` / per-batch reset / `bump_bsi_and_transfer_prev`
     (arq_common.cc:6575) set `FREE`.
   - A still-decoding or non-converging frame NEVER calls add_message_rx_data ⇒ its
     slot stays `FREE`. INVARIANT: at the deadline, RECEIVED ⇔ the frame's bytes are
     already in messages_rx[i].data. Lever #2 does NOT change this producer; it only
     READS the snapshot earlier.
2. **Consumers of `messages_rx[].status`** (the SACK bitmap build):
   - arq_responder.cc:1637-1638 rx_received counter; :1690-1691 sack_bitmap[i] =
     (status==RECEIVED); :1042-1043 EOB promote (match-current only). All READ-ONLY
     of status. Firing earlier feeds them a SNAPSHOT with possibly-fewer RECEIVED
     slots — which is EXACTLY a partial batch, the case these consumers already
     handle every day (a lost frame is indistinguishable from a still-decoding one).
3. **bitmap → CMD retx (arq_commander.cc:3098-3164)**: for each i, sack_bitmap[i]
   true ⇒ messages_tx[i] marked ACKED (delivered); false ⇒ enqueued for retransmit
   (idempotent, append-mode for v2). A speculatively-bit-0 frame ⇒ enqueued ⇒
   retransmitted ⇒ on re-receive RSP's add_message_rx_data stores it RECEIVED in the
   SAME slot. INVARIANT (no double-delivery): the byte-path delivers a slot's bytes
   to the app exactly once — see §8.6.
4. **bsi / last_received_end_of_batch_seq / D5 prev-bump**: see §8.5 — the load-bearing
   audit, because firing the SACK earlier is firing the bsi-bump+prev-transfer
   earlier.
5. **What lever #2 changes**: ONLY the wall-clock instant of the
   RECEIVING→ACKNOWLEDGING_DATA transition (and only when MERCURY_SPEC_SACK is set).
   It alters NO producer of messages_rx[].status, NO field of any frame, NO bsi
   arithmetic. Walked every consumer above: each already tolerates a partial
   snapshot (that is the entire reason the SACK path exists). Default-off ⇒ the
   transition fires at the SAME instant as today ⇒ byte-identical.

### §8.5 D5 prev-bump / EOB-inference under MORE-FREQUENT partial SACKs (the known risk)
The known D5 defect (PREV_BUMP_VERDICT §4, MEMORY): `bump_bsi_and_transfer_prev()`
(arq_common.cc:6539-6543) sizes the SEALED batch's `prev_expected = min(
data_batch_size, last_received_end_of_batch_seq+1)`. If the EOB frame of a bumped
batch is the LOST one, `last_received_end_of_batch_seq` is stale/shorter than the
TX's true frame count ⇒ the genuinely-missing TAIL frame in
`[prev_expected, data_batch_size)` is silently dropped (the SET-gate
`prev_batch_is_frame_complete` checks only `[0,prev_expected)`). Does lever #2 make
this WORSE? Audited per case:

- **Case A — EOB already decoded, a NON-EOB frame k still decoding at the deadline.**
  `last_received_end_of_batch_seq` was promoted when the EOB landed
  (arq_responder.cc:1042-1043), so `prev_expected = data_batch_size` (the true,
  full count). Frame k is in `[0, prev_expected)` ⇒ reported bit-0 ⇒ retransmitted.
  NO truncation, NO D5. Lever #2 is SAFE here, and this is the COMMON case the lever
  targets (a doomed interior frame holding up the whole batch).
- **Case B — the EOB frame itself is the still-decoding/lost one at the deadline.**
  Then `last_received_end_of_batch_seq` was NEVER set this batch (stays −1 from the
  per-batch reset / arq_common.cc:876), so `prev_expected = data_batch_size` (the
  FULL batch, NOT a truncated EOB+1). The tail (including the EOB slot) sits in
  `[0, data_batch_size)` ⇒ reported bit-0 ⇒ retransmitted. NO truncation. This is
  STRICTLY the pre-existing behavior: the EOB-inference only shortens `prev_expected`
  when the EOB WAS decoded; a missing EOB leaves the full size. Lever #2 firing
  earlier does not change which path runs.
- **Case C — D5 PROPER (stale shorter EOB from a PRIOR batch).** D5 requires
  `last_received_end_of_batch_seq` to hold a SHORT value at bump time AND the real
  batch to be longer with a missing tail. That stale value can ONLY come from a
  PRIOR decoded EOB that was not reset — a pre-existing concern INDEPENDENT of WHEN
  the SACK fires. Lever #2 does not write `last_received_end_of_batch_seq` and does
  not change the reset cadence; it cannot manufacture a stale-short EOB. Firing the
  bump earlier with the SAME (correct, full or −1) `last_received_end_of_batch_seq`
  reproduces the pre-existing (sound) behavior. CONCLUSION: lever #2 does NOT
  introduce or amplify D5. (D5 PROPER remains the separate, gated cfg16-acq-d2d3
  concern; lever #2 is orthogonal and is the OFF-default, so the integrity-gated D5
  fix is unaffected.)
- **Single-frame batch (data_batch_size==1)**: the gate's `data_batch_size > 1`
  guard (mirrors arq_responder.cc:1679 / :1721) suppresses the speculative SACK —
  single-frame batches use the all-or-nothing EOB receipt (the lone DATA frame's
  bit-7), so there is nothing to selectively SACK. No D5 surface.

### §8.6 NO double-delivery, NO silent loss (the integrity proof)
- **No double-delivery of a speculatively-bit-0 frame that LATER decodes.** When
  frame k is reported bit-0, RSP did NOT deliver k (its slot is FREE; delivery copies
  only RECEIVED slots, copy_data_to_buffer). CMD retransmits k. RSP re-receives it
  via add_message_rx_data → slot k flips FREE→RECEIVED → delivered ONCE at the
  batch's eventual completion. The ORIGINAL doomed decode of k, if it ever DID
  converge after the deadline, lands in the SAME slot k (add_message_rx_data writes
  by `id`, arq_responder.cc:53) — it cannot create a second app delivery because
  delivery is per-SLOT, not per-decode-event. There is exactly one slot k and it is
  delivered exactly once. (In the actual clean-WGN target, the doomed frame NEVER
  converges — it is pure non-convergence, §0 — so the "later decodes" race is the
  conservative worst case, and even it is single-delivery.)
- **No silent loss.** A speculatively-bit-0 frame is ENQUEUED for retransmit by the
  CMD (arq_commander.cc:3111-3160) — never marked ACKED. The batch is NOT promoted
  (last_batch_fully_acked=false, arq_commander.cc:3188), so CMD keeps the in-flight
  batch outstanding and the missing slot is re-sent until RECEIVED. The full payload
  is delivered in-order after the retx (the RSP delivers a batch only when
  rx_received >= expected). If the speculative SACK itself is lost on the wire, the
  CMD's ACK-timeout drives a full-batch retransmit (the existing fallback,
  arq_responder.cc:1697-1719). Either way the missing bytes are recovered.

### §8.7 Implementation (env-gated, default-off byte-identical)
ONE gate block inserted in `process_messages_rx_data_control()` immediately after the
BREAK handling and before the `if(messages_rx_buffer.status==RECEIVED)` consumer
(arq_responder.cc, after the BREAK block at :498, before :500). The gate:
- reads `MERCURY_SPEC_SACK` ONCE (static, like turnaround_rephase_env at
  arq_common.cc:1378); unset/0 ⇒ the entire block is skipped ⇒ byte-identical.
- fires only when: spec_sack_on && link_status==CONNECTED &&
  connection_status==RECEIVING && sack_enabled && data_batch_size>1 &&
  batch_rx_frame_count>=1 && rx_received<expected &&
  receiving_timer.get_elapsed_time_ms() >= deadline (window-fraction).
- on fire: log `[RSP-SPEC-SACK]`, `receiving_timer.stop()/reset()`, set
  `connection_status=ACKNOWLEDGING_DATA`, `return;`. The existing ACK-GATE handler
  builds + sends the SACK next tick.
The `expected` count is computed identically to the existing ACK-GATE
(arq_responder.cc:1640-1669): default data_batch_size, lowered to
last_received_end_of_batch_seq+1 when an EOB was seen, else the compression-header
fallback. Reusing the SAME formula keeps the gate's "incomplete?" decision consistent
with the handler's.

## §9 LEVER #2 cross-layer regression test (test_partial_bsi_advance pattern)

`mercury.exe --test-spec-sack` (in-process synthetic-fire, arq_responder.cc, modelled
on `test_partial_bsi_advance` arq_responder.cc:3339). It drives the PRODUCTION
helpers + the PRODUCTION routing/bitmap logic on directly-primed private ARQ state —
no IONOS, no RF, no DSP. The scenario (the test pattern named in the task):

1. Prime a CFG16-shaped batch (data_batch_size=K, sack_v2 on). Mark slots
   `[0,K)\{k}` RECEIVED and frame-k STILL-DECODING (slot k left FREE) at the deadline
   — i.e. frame-k is non-converging (the clean-WGN §0 case).
2. **fail-before** (`MERCURY_SPEC_SACK` UNSET): assert the deadline does NOT advance
   the state early (the production gate is compiled but env-off) — the SACK would
   only fire after the whole serial loop (here: never, since frame-k never
   converges) → rx_received stays K−1 < expected (the stall the lever fixes).
3. **pass-after** (`MERCURY_SPEC_SACK=1`): drive the gate predicate at the
   window-fraction deadline → assert it fires IN-WINDOW (elapsed < CMD window) with
   the SACK bitmap built from messages_rx[].status==RECEIVED ⇒ bit_k==0, all other
   bits 1.
4. Run the PRODUCTION CMD partial-SACK retx consumer logic on that bitmap ⇒ assert
   frame-k is ENQUEUED for retransmit (not ACKED), every other slot ACKED.
5. Synthesize the retransmit of frame-k ⇒ add_message_rx_data into slot k ⇒ assert
   slot k flips FREE→RECEIVED and the byte payload is BIT-FAITHFUL to the TX bytes.
6. Assert NO double-delivery (slot k delivered exactly once: a second decode of the
   same slot does not add a second app-copy) and NO silent loss (final rx_received==K,
   full payload in-order).
The test asserts the §8.6 integrity properties directly and is the §5 cross-layer
regression. `tools/test_turnaround_eff.py` gains a TEST-SS wrapper that runs
`--test-spec-sack` and checks rc==0.

## §10 What only the Pi confirms (HONEST) — lever #2

The wall-clock half: whether firing the SACK at the 3/4-window deadline actually
pulls the reverse-ACK landing back INSIDE the CMD listen window on HW (bench-9
metric: reverse-ACK miss → 0, active-fraction up), and whether 3/4 is the right
fraction (a slow-but-healthy batch must not be cut short — sim WGN is timing-clean
so the only frames not-yet-RECEIVED at the deadline are genuinely doomed; on HW the
keyer/AGC/capture latency could make a healthy-but-late frame look not-yet-received).
The fraction is env-tunable (MERCURY_SPEC_SACK_NUM/DEN) for the bench sweep. The
in-process test proves the STATE-MACHINE + integrity (in-window fire, bit_k=0, retx,
byte-faithful re-receive, single-delivery, no silent loss); only the latency payoff
and the fraction calibration await the Pi.

## §7 What only the Pi confirms (HONEST) — levers #1 / #3
- The actual WALL-CLOCK turnaround improvement: whether the bounded multiplier
  (#1) + earlier per-decode termination (#3) pull the reverse-ACK SACK back
  inside the CMD window on HW (the bench-9 end-to-end metric: reverse-ACK miss
  -> 0). Windows measures decode CORRECTNESS and iteration COUNTS, not the
  Cortex-A76 ldpc-ms-per-iteration nor the keyer/AGC/capture latency budget.
- The real-channel rate of frames whose ONLY good timing shift is -sym (the
  probe #1 drops). Sim WGN is timing-clean; the -sym-only rate is an OTA
  quantity. If it is non-negligible, #1(b) is the lever to relax (allow 1 probe
  in the better-metric direction rather than fixed +sym).
