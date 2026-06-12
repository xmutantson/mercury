# data-flow-cheap-ack-retry.md — Conservative cheap reverse-ACK-miss recovery

**Owner of the shared state:** `emergency_nack_count`, `cfg16_revack_starve_fails`,
`breaks_since_last_data_success`, `anchor_consec_break_fails`,
`last_data_viable_config`, and the new `recent_forward_decode_frac` /
`cheap_ack_retries_used` on the `data_ack_received == NO` (reverse-ACK-miss)
decision path.

**Author:** xmutantson
**Branch:** `feat/cheap-ack-retry` off `sim/trackc-d5-coupled-chain @630a240`
(carries D5 TX-authoritative `batch_total_frames` + D2/D3, off monitor `627c370`).
**Design:** `_research/ACTIVE_FRACTION_OVERHAUL_DESIGN.md` (Section-5, lever 2,
CONSERVATIVE arm). USER-RATIFIED: build the conservative arm only.
**Status:** implemented, env-gated `MERCURY_CHEAP_ACK_RETRY`, default-off
byte-identical. Unit-proven via `--test-cheap-ack-retry` (T1–T5).

All line cites are against the `feat/cheap-ack-retry` worktree
(`C:/Users/kamer/mercury_wt/cheap-ack`).

---

## §0 — The defect (taken as given from the design, not re-derived)

On a reverse-ACK miss the CMD enters the `data_ack_received == NO` branch
(`arq_commander.cc:3608`), flips all `PENDING_ACK` slots to `ACK_TIMED_OUT` for
resend (`:3576-3580`, UPSTREAM of the miss-eval), increments
`emergency_nack_count` (`:3715`), and at threshold (default 3,
`arq_common.cc` `emergency_nack_threshold`) fires `send_break_pattern()`
(`:4139`) → RSP drops to ROBUST_0/CONFIG_0 (~66 bps) + ~30 s robust timeout.
Panic accelerant: `breaks_since_last_data_success >= 2` forces
`break_drop_step = 100` (`:4058-4065`) → direct ROBUST_0 jump.

`data_ack_received == NO` is OVERLOADED: it fires identically for "RSP got the
batch, ACK lost" (transient; forward channel healthy) and "RSP got nothing"
(genuine forward collapse). The demote is CORRECT for genuine loss; the defect is
it fires on the transient case too. The conservative arm defers the BREAK ONLY
when there is POSITIVE recent evidence the RSP is still decoding our forward CFG16
data, and ONLY within a bounded budget; otherwise the existing BREAK path fires
bit-for-bit (the safety guarantee).

---

## §1 — The cheap-retry decision (the new pure helper)

`cheap_ack_retry_allowed(...)` — `include/common/common_defines.h`. PURE inline,
mirrors the `cfg16_revack_starve_fallback_target` idiom (`common_defines.h:482`).
Returns `true` iff ALL of (the CONSERVATIVE, AND-of-evidence discriminator):

- `cheap_retry_enabled` — the env gate `MERCURY_CHEAP_ACK_RETRY` (read once at the
  call site, passed in; the helper stays pure). Default false → byte-identical.
- `current_config == CONFIG_16` (D-scope; CFG16 only, the audited turnaround-limited rung).
- `is_ofdm_config(current_config)` (corroborating scope guard).
- `link_connected && turbo_done && gear_shift_on` (same outer guards the BREAK uses).
- `retries_used < CHEAP_RETRY_BUDGET` (D-b bounded; genuine loss costs only
  `CHEAP_RETRY_BUDGET` extra ~3.5 s CFG16 turnarounds, then the existing demote).
- `recent_forward_decode_frac >= CHEAP_FWD_HEALTH_FRAC` (D-a; the STRONGEST,
  POSITIVE proof — a recent SACK_RSP whose bitmap showed the RSP decoded
  ≥ `CHEAP_FWD_HEALTH_FRAC` of our forward frames). This is the CONSERVATIVE
  arm's load-bearing gate: genuine forward collapse NEVER has this evidence
  (no recent SACK_RSP ⇒ frac stale/0), so the cheap-retry CANNOT arm on genuine
  loss ⇒ zero genuine-loss-recovery regression.

FAIL-BEFORE: `-DCHEAP_ACK_RETRY_FAILBEFORE` compiles the helper to `return false`
(no cheap-retry → the BREAK cascade runs) — mirrors `-DFIX9_D3_FAILBEFORE`.

Constants (`common_defines.h`, all TUNABLE, default-off via env so inert):
- `CHEAP_RETRY_BUDGET = 3` — extra CFG16 turnarounds before the BREAK is allowed.
- `CHEAP_FWD_HEALTH_FRAC = 0.50` — a batch with ≥50% frames decoded proves health
  (in-burst CFG16 ≈ CFG15, design §2.4 / audit §0).

NOTE — why D-a alone (not D-c `ack_diag_peak_metric`): the design offers a composite
`D-a OR D-c`. The CONSERVATIVE arm uses D-a ONLY, because D-a is the discriminator
that "genuine loss never has" (positive forward-decode proof). D-c (ACK-present-but-
garbled correlator hit) is the AGGRESSIVE arm's cold-start lever and is HELD per the
user-ratified staging — it leans on the phantom-ACK content gate and only helps the
first-batch-no-history case, which the staging defers until HW false-positive data
exists. The conservative arm therefore "fails safe to BREAK" on the cold first batch
(no history) exactly as today.

---

## §2 — Producers / Consumers / Invariants (CLAUDE.md §5 audit)

### `recent_forward_decode_frac` (NEW; `arq.h` member, `{0.0}` init)
- **Producers (write):** the SACK_RSP partial-batch handler
  (`arq_commander.cc:3146`, after `rx_count/data_batch_size` is computed) rolls
  `recent_forward_decode_frac = rx_count / data_batch_size`. The clean full-batch
  ACK path (all frames delivered) sets it to 1.0. This is REVERSE EVIDENCE OF
  FORWARD HEALTH recorded on a PRIOR turnaround — it survives a LATER turnaround
  whose ACK is lost (that is the whole point: it is the freshest proof the RSP was
  decoding our forward data).
- **Reset:** ctor + both session-reset paths (R3 parity with
  `cfg16_revack_starve_fails`) → `0.0` ("no proof yet" → conservative fail-to-BREAK).
- **Consumer:** `cheap_ack_retry_allowed()` (D-a gate). One consumer.
- **Valid states:** `[0.0, 1.0]`. BEFORE any producer (cold first batch): `0.0`
  ⇒ `< CHEAP_FWD_HEALTH_FRAC` ⇒ cheap-retry never arms ⇒ BREAK fires as today
  (the deliberate conservative cold-start fail-safe).
- **Invariant the consumer assumes:** a value ≥ `CHEAP_FWD_HEALTH_FRAC` means a
  RECENT SACK_RSP proved the RSP decoded that fraction of forward frames. Held by
  the producer: the value is only written from a real SACK_RSP `rx_count` or a real
  clean-full ACK; it is never fabricated. (It is intentionally NOT decayed per-miss
  — within a bounded budget of ~3 turnarounds a single fresh proof is the design's
  recency basis; the budget IS the recency bound, §1 D-b.)

### `cheap_ack_retries_used` (NEW; `arq.h` member, `{0}` init)
- **Producer (increment):** the cheap-retry path itself (`arq_commander.cc`, the
  new block at the top of the miss-eval), each time a cheap-retry is taken.
- **Producer (reset to 0):** ANY `data_ack_received == YES` (the `else` branch,
  `arq_commander.cc:4152` neighbourhood — symmetric with `emergency_nack_count`),
  i.e. any delivery resets the budget; and on config change off CFG16 (the budget
  is CFG16-scoped, mirrors `cfg16_revack_starve_fails`). Ctor + session resets → 0.
- **Consumer:** `cheap_ack_retry_allowed()` (D-b bound). One consumer.
- **Invariant:** counts CONSECUTIVE cheap-retries with NO delivery between. When it
  reaches `CHEAP_RETRY_BUDGET`, the gate goes false and the EXISTING path
  (D3 demote / BREAK) runs — bounding the worst case. A delivery zeroes it.

### `emergency_nack_count` (the BREAK trigger; `arq.h`)
- **Producers:** `arq_commander.cc:2622` (control-failure path — UNTOUCHED),
  `:3715` (data-ACK-miss — the path we GUARD, but do NOT modify the ++ itself).
- **Change:** the cheap-retry block `return`s BEFORE `:3715`, so on a cheap-retry
  `emergency_nack_count` does NOT advance. When the cheap-retry gate is false
  (budget exhausted OR forward unhealthy), control falls through to `:3715`
  exactly as today — the increment, the D3 check, and the BREAK threshold check
  are byte-identical on that path.
- **Consumers:** the two BREAK gates (`:2627` control, `:4039` data). The data
  gate is now reached only when forward-unhealthy OR budget-exhausted. The control
  gate is UNAFFECTED (separate branch, not on `data_ack_received==NO`).
- **Invariant REFINED (design §1.4):** "N consecutive misses with NO positive
  evidence the RSP is still decoding forward data ⇒ demote." A transient miss WITH
  fresh D-a evidence and budget remaining does not advance the counter to threshold.

### `cfg16_revack_starve_fails` (FIX-9 D3 starvation streak; `arq.h:3312`)
- **Producer:** `:3728-3731` (`++` at CFG16, reset off-CFG16) — UPSTREAM of the D3
  check at `:3900`. **The cheap-retry block returns BEFORE `:3728`**, so a
  cheap-retry does NOT advance the starve streak either. This is correct: a
  transient miss the cheap-retry absorbs must not count toward the D3 demote
  deadline (otherwise the first post-budget miss would instantly trip D3). When the
  budget is exhausted, the fall-through resumes advancing it from its prior value,
  so D3 still fires after `CHEAP_RETRY_BUDGET + CFG16_REVACK_STARVE_FAILS`
  un-absorbed misses — strictly later, never earlier, never suppressed.
- **Consumer:** `cfg16_revack_starve_fallback_target` (`:3900`). UNAFFECTED in
  logic; only reached fewer times (good — fewer spurious CFG16→CFG15 demotes of a
  healthy rung whose ACK was merely transiently lost).

### Panic + anchor (`breaks_since_last_data_success` `:4058`;
`anchor_consec_break_fails` `:4086`; `last_data_viable_config` `:4099`)
- **Change:** the cheap-retry returns before the BREAK block, so on a transient
  miss these do NOT increment. Strictly FEWER spurious panic/anchor increments.
- **Consumer — `break_target_with_anchor` / anchor-demote / `test_climb_engine`
  Part E (`:7948-8068`):** the deep-SNR anti-thrash escape relies on K consecutive
  anchor-rung BREAKs. **A GENUINE deep-SNR cliff is forward-UNHEALTHY** (no recent
  SACK_RSP ⇒ `recent_forward_decode_frac` low ⇒ D-a fails ⇒ cheap-retry never
  arms), so real BREAKs still accumulate and the escape still works. The
  discriminator's NEGATIVE branch is what protects Part E. **This is the
  highest-risk interaction — T3 asserts it.**

### Config ladder / gearshift (`current_configuration`, `negotiated_configuration`,
`supershift_proven_ceiling`)
- **Change:** a cheap-retry changes NO config (both ends stay CFG16). Strictly
  less perturbation than a demote+re-climb. No CMD/RSP config desync risk (no
  SET_CONFIG round-trip to lose) — the cheap-retry is the LEAST-perturbing option
  on the miss path (less than D3's CFG16→CFG15 SET_CONFIG, far less than BREAK).

### Optimizer (design §4-(3))
- The cheap-retry block records the batch `failed=true` via `opt_record_batch`
  (it delivered 0 bytes this turn — honest) and `opt_pending_switch_cfg = -1`
  (cancel stale switch), MIRRORING `:3624-3635`, then returns. It does NOT run
  `evaluate()` (no mid-recovery switch), same as the existing miss path.

### Retx queue (design §4-(5)) / in-order bsi (design §4-(6))
- The cheap-retry rides the EXISTING `PENDING_ACK → ACK_TIMED_OUT` resend
  (`:3576-3580`, already done before our block). No new retx producer. NO BREAK ⇒
  the RSP does NOT reset its bsi window (`arq_responder.cc` BREAK reset only) ⇒ the
  in-order window stays continuous (BETTER for in-order delivery). The resend
  dedups by `batch_seq_id` exactly as a sub-threshold retry does today.

---

## §3 — D5 sibling-bug interaction (design §4-(7), CLAUDE.md §5)

D5 (lost-EOB silent-tail-drop on the PREV-BUMP path) is FIXED in this base
(`630a240` carries `99588ef` — TX-authoritative `batch_total_frames` on the v2
wire; `data-flow-d5-eob-batch-truncation.md`). The cheap-retry does NOT change EOB
inference or the bump path — it only DEFERS a BREAK and re-sends via the existing
`ACK_TIMED_OUT` path. But it CHANGES the FREQUENCY of CFG16 retransmit cycles,
which exercises the EOB/retx path MORE. Guard: the validation asserts
byte-faithful (md5) delivery across cheap-retry cycles (the integrity battery:
partial-bsi / gap-abort / inorder-demote) so any D5 escalation surfaces loudly,
not as silent-wrong-bytes. Because the base has the D5 fix, the lost-EOB tail
leaves received < expected ⇒ the SET-gate HOLDS ⇒ faithful-after-retx — the extra
cheap retransmits are SAFE for the D5 class.

---

## §4 — The RSP symmetric-guard open question (design §2.3 / §5.4 SECOND FORK)

CMD-only fix. The cheap-retry keeps BOTH ends at CFG16 and re-sends the batch; the
RSP is in its normal CFG16 `RECEIVING` window (it never saw a BREAK) and dedups by
`batch_seq_id`. OPEN [?]: whether the RSP's own `receiving_timeout`
(`arq_common.cc`, robust cap path) can expire and self-demote during the CMD's
cheap-retry turnaround. The unit test cannot exercise cross-end timing; this is
flagged for the sim/bench arm (design §5.2/§5.3). If a residual RSP-initiated
demote is observed on the bench, the both-ends symmetric guard is the follow-on.

---

## §5 — Failing-test-first (the PRIMARY gate)

`--test-cheap-ack-retry` (`arq_commander.cc`, pattern of `test_climb_engine`
Part E `:7948`). Drives the PURE helper + the real member-field discipline.
FAIL-BEFORE: `-DCHEAP_ACK_RETRY_FAILBEFORE` → helper returns false.

- **T1 TRANSIENT-MISS-ON-HEALTHY** — CFG16, `recent_forward_decode_frac=0.9`,
  budget remaining ⇒ helper TRUE; replay the miss-eval ⇒ stays CFG16, batch
  re-queued (`ACK_TIMED_OUT`), `emergency_break_active==0`,
  `breaks_since_last_data_success==0`. FAIL-BEFORE: helper false ⇒ after threshold
  the BREAK path stages a demote ⇒ FAIL.
- **T2 GENUINE-LOSS-STILL-DEMOTES (safety)** — `recent_forward_decode_frac=0.0`
  (silent line) ⇒ helper FALSE both before AND after the fix ⇒ BREAK fires at the
  SAME threshold. No-regression assertion.
- **T3 DEEP-SNR ANTI-THRASH INTACT (safety)** — forward-UNHEALTHY at the anchor
  rung ⇒ helper FALSE ⇒ the K-consecutive anchor-rung BREAK escape (Part-E
  `anchor_demote_target`) still reaches `ANCHOR_DEMOTE_BREAK_FAILS` and demotes
  the anchor. The cheap-retry's negative branch preserves Part E.
- **T4 BUDGET BOUND** — healthy but ACK-deaf: after `CHEAP_RETRY_BUDGET`
  cheap-retries the helper goes FALSE (`retries_used >= budget`) even with
  `recent_forward_decode_frac=0.9` ⇒ the BREAK eventually fires (finite re-listen).
- **T5 BYTE-IDENTICAL DEFAULT-OFF (safety)** — with `cheap_retry_enabled=false`
  (env unset) the helper returns FALSE for the T1-healthy inputs ⇒ identical to the
  pre-fix BREAK path; `--test-climb-engine` ALL PASS (gearshift/D2/D3 unperturbed);
  the default-render md5 equals the base.

T2/T3/T5 are the SAFETY gates — all must be green or the fix is not shippable.
