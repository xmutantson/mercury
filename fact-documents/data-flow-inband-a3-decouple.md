# data-flow-inband-a3-decouple.md

Owner-gated A3 demote-decouple for the in-band rate-adapt redesign.
Branch `feat/inband-a3-decouple` off `ab/redesign-v3` (5d9633f). Env gate
`MERCURY_INBAND_A3_DECOUPLE`, DEFAULT-OFF (OFF = current redesign-v3 behavior,
byte-identical). NOT enabled, NOT merged — owner-ratification-gated.

## §0 The defect (HW-diagnosed, established)

On `ab/redesign-v3` the CMD connect-liveness guard
(`inband_connect_liveness_guard`, arq_commander.cc:3172) fires whenever the link
is CONNECTED, ~N=200 data-less control-plane polls elapse with `stats.nAcked_data`
FLAT, and a forward DATA batch is in flight. At the stall threshold
(arq_commander.cc:3268-3286) it routes a **forward-healthy reverse-ACK miss** to
`inband_route_failure_demote()` — i.e. it DEMOTES ONE RUNG per confirmed stall,
unconditionally (the only gate is `cmd_has_inflight_data_batch() && !config_is_at_bottom`;
there is NO demote-rate cap on this path, and it does NOT consume
`INBAND_LIVENESS_MAX_BREAKS`).

A forward-healthy reverse-ACK miss is NOT a decode failure: the RX is decoding the
forward DATA fine, but its EOB SACK lands outside the CMD listen window, so
`nAcked_data` stays flat. Demoting on this trigger walked CFG 15 -> 6 on HW
(9/10 demotes labelled `forward_healthy_revack_miss`). The RSP down-ladder window
is D=4 (cap INBAND_DOWN_D_MAX=8, arq.h:3349; arq_common.cc:3920-3927); once the CMD
config drops below the RSP's down-window the RSP can no longer decode the tag,
cannot advance its anchor, the two sides desync (self-reinforcing) ->
SESSION_DEAD -> TERMINAL BREAK -> stall at 11090.

ROOT CAUSE: the demote is triggered by a LIVENESS/ACK-MISS alone, not by a real
decode-rate failure. A missed reverse-ACK is recoverable for free on the next
turnaround IF the SACK is cumulative (STANAG-5066 acknowledged-through). Without
that, "make the miss cheap" by demoting instead converts crawl -> config-walk -> dead.

## §1 The fix (strict-sequenced)

1. **PREREQUISITE — port A3 cumulative-n_r ACK** (from `feat/a3-cumulative-ack`
   1d2f77f, D5-aware port of reference e538d56). Reshapes the EOB SACK 8-bit bsi
   field into a cumulative high-water n_r = `rsp_last_delivered_batch_seq_id` when
   `CAP_CUMULATIVE_ACK` is negotiated, so a dropped reverse-ACK is SUPERSEDED by the
   next turn's report. Without this, decoupling the demote turns crawl -> dead-link.
   Ported verbatim, capability-negotiated, env-gated `MERCURY_CUMULATIVE_ACK`,
   DEFAULT-OFF byte-identical. (1d2f77f is a clean port; redesign-v3 carries NO A3
   infra — verified `git grep -l cumulative_ack ab/redesign-v3` = empty.)

2. **RE-TARGET the demote** (arq_commander.cc:3268-3286). Under
   `MERCURY_INBAND_A3_DECOUPLE` ON: on a forward-healthy reverse-ACK miss, RE-AIR
   the SAME config (rely on the A3 cumulative-ACK recovery to self-heal the missed
   ACK on the next turnaround), and DO NOT demote. Demotion happens ONLY via the
   real decode-rate-failure paths (`frame_gearshift_data_failed_*`, arq_commander.cc
   :4236/:4458). OFF: byte-identical current behaviour (demote on the miss).

   The decouple gate (`inband_a3_decouple_enabled()`) requires BOTH
   `MERCURY_INBAND_A3_DECOUPLE` AND `cumulative_ack_enabled` (the negotiated A3
   capability) — without the self-heal spine the decouple is unsafe (it would
   crawl/dead), so the gate refuses to decouple if A3 was not negotiated.

## §5 CROSS-LAYER AUDIT (CLAUDE.md §5)

### State touched

**`cmd_inband_liveness_*`** (arq.h:3512-3514: last_acked / no_progress_polls / breaks)
- Producers: arq_commander.cc:3196-3201 (advance reset), :3225/:3290-3291 (re-arm),
  :3306 (breaks++), :3268-3286 (demote path re-arm).
- Consumers: the guard itself (loop at :3220-3306).
- Valid states: streak in [0, stall_polls], breaks in [0, INBAND_LIVENESS_MAX_BREAKS=3].
- Invariant: a forward-DATA delivery (`nAcked_data` advance) resets streak AND breaks.
- **Fix change:** on the decoupled re-air, we re-arm `no_progress_polls=0` (same as the
  existing demote path :3290) so the next stall re-accumulates from 1. We do NOT tick
  `breaks` (a re-air is not a BREAK — same as the existing demote path). VERIFIED: no
  consumer reads a stale streak; the budget invariant is preserved.

**`cmd_batch_seq_id` epoch** (producers: :3272 via demote's bsi-rollback :3055/:4236/
:4458 via demote, :4856 D3). The CURRENT demote path rolls `cmd_batch_seq_id` BACK to
the earliest in-flight bsi (`inband_route_failure_demote` :3014-3058) so the re-sent
batch is contiguous with the RSP high-water.
- **Fix change:** the decoupled RE-AIR does NOT call `inband_route_failure_demote`, so it
  does NOT roll the epoch and does NOT change the config. It re-presents the SAME
  in-flight batch at the SAME config — `cmd_batch_seq_id` is UNCHANGED, the in-flight
  batch already carries its bsi in `messages_tx[]`, and the next `send_batch` re-airs it.
  AUDIT: because we do NOT free `messages_tx[]` (no demote re-stage), the in-flight
  batch and its bsi are intact; no GAP-ABORT can arise (the bsi did not move). The RSP
  high-water (n_r) supersedes the missed report on the next turnaround under A3.
  CONSTRAINT: the re-air must NOT advance `cmd_batch_seq_id` (it would orphan the
  in-flight batch into a gap). VERIFIED the re-air leaves the epoch untouched.

**RSP D-window anchor** (`rsp_current_expected_batch_seq_id` /
`rsp_last_delivered_batch_seq_id`, arq_responder.cc:908-930; down-window
arq_common.cc:3920-3927, D=4 cap 8).
- The defect: the CMD config walking BELOW the RSP down-window (cur_idx - D) strands the
  anchor. The decouple KEEPS the CMD at the SAME config, so the CMD config never leaves
  the RSP down-window — the desync mechanism is removed at the source. VERIFIED: with no
  config change the RSP always decodes the re-aired batch's tag (same config), advances
  `rsp_last_delivered_batch_seq_id`, and the next cumulative n_r retires the miss.

**Retx queue** (`retransmit_frames[]`, `messages_tx[]`).
- The current demote path frees `messages_tx[]` + clears the retx queue + re-stages from
  `fifo_buffer_tx`/compressed. The decoupled re-air does NONE of this: it leaves the
  in-flight batch in `messages_tx[]` for the normal retx/re-air machinery to re-present.
  AUDIT: no retx queue is stranded because we don't touch it — the existing per-frame
  retx (nResends) owns the re-air, exactly as a normal in-flight batch awaiting its ACK.

### §5 verdict
The decouple is SAFER than the demote on every axis BECAUSE it changes LESS state: it
neither moves the config (so the RSP D-window is never escaped), nor rolls the bsi epoch
(so no GAP-ABORT), nor frees `messages_tx[]` (so no retx-queue churn). Its only risk —
a genuinely-dead reverse channel never self-heals and crawls forever — is closed by the
EXISTING genuine-death nets, which the decouple leaves intact:
- `INBAND_LIVENESS_MAX_BREAKS=3` hard reset (:3294-3304) — unchanged, still backstops a
  link no amount of re-airing rescues (the decouple still increments toward it on the
  genuine path).
- The real decode-failure demote (`frame_gearshift_data_failed_*`) still fires on actual
  decode loss — the decouple ONLY suppresses the demote on the liveness/ACK-MISS trigger.
- A3's INV-T2-CONTIG: a frozen high-water can never cover an undelivered batch, so a
  truly dead reverse channel is NOT papered over by a stale cumulative n_r
  (proven by `--test-a3-decouple-safety` T5b).

## §6 Test (fails-before / passes-after, bounded)

Extend `--test-inband-deliver` (arq_responder.cc:7635) with a PART F driving >=6
consecutive forward-healthy reverse-ACK misses through the production guard:
- FAIL-BEFORE (decouple OFF): reproduce the config-walk (CFG_FROM -> CFG_FROM-6) +
  the CMD leaving the RSP D=4 window -> desync signature.
- PASS-AFTER (decouple ON + A3 negotiated): the CMD stays at CFG_FROM across all 6
  misses (within the RSP D-window), the missed ACKs self-heal via the cumulative n_r,
  the in-flight batch is preserved and delivers.
Plus the ported `--test-cumulative-ack` and `--test-a3-decouple-safety` from the A3
commits (the A3 predicate + decouple-safety proofs).

## §7 VERIFIED vs INFERRED
- VERIFIED (read on ab/redesign-v3): defect site routes to `inband_route_failure_demote`
  not `send_break_pattern`; redesign-v3 has NO A3 infra; the guard/route/test bodies;
  the RSP D-window math.
- VERIFIED (sim, --test-inband-deliver PART G, 2026-06-22): the config-walk count under
  6 misses is CONFIG_10 (idx 13) -> CONFIG_4 (idx 7) = 6 rungs (> the RSP D=4 window, so the
  desync signature is reproduced), 6 demotes fired (decouple OFF); and the decoupled re-air
  leaves `cmd_batch_seq_id=7` UNCHANGED, the in-flight batch queued, NO break, no dead-batch
  tick, across all 6 misses (decouple ON + cumulative_ack negotiated). Both arms run in ONE
  process via a runtime `MERCURY_INBAND_A3_DECOUPLE` toggle (no recompile) — stronger than a
  `-D` compile-flag arm. PART A (the OFF/default path) is unchanged (CONFIG_10->CONFIG_9
  demote, bsi rollback) = default-off byte-identical.

## §8 STATUS (2026-06-22)
- IMPLEMENTED on `feat/inband-a3-decouple`, DEFAULT-OFF, owner-ratification-gated.
  - Gate: `inband_a3_decouple_enabled()` (arq_common.cc) — env `MERCURY_INBAND_A3_DECOUPLE`
    AND `cumulative_ack_enabled` (strict sequencing: refuses to decouple without the A3
    self-heal spine). Decl arq.h; env cache `inband_a3_decouple_env` ctor-init, NOT
    session-reset (env-keyed).
  - Re-target: arq_commander.cc:3299 — on a forward-healthy reverse-ACK miss with the gate ON,
    RE-AIR the same config (no `inband_route_failure_demote`, no bsi roll, no `messages_tx[]`
    free); re-arm the stall window; do NOT tick `cmd_inband_liveness_breaks`. Genuine-death
    nets (real decode-failure demote `frame_gearshift_data_failed_*`, INBAND_LIVENESS_MAX_BREAKS,
    A3 INV-T2-CONTIG) intact.
  - Prerequisite A3 cumulative-n_r ACK (env `MERCURY_CUMULATIVE_ACK`, negotiated
    `cumulative_ack_enabled`, CAP_CUMULATIVE_ACK) already ported + green
    (--test-cumulative-ack, --test-a3-decouple-safety §2 CHECKPOINT GREEN), both now wired
    into the `--test` battery (main.cc).
  - Build (bash build.sh o3) clean; `--test` GREEN (229 PASS, 0 fail, exit 0).
- NOT enabled by default, NOT merged. Owner/me-gated per the task.
