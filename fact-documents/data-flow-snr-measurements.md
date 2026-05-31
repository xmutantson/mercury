# Data-Flow Audit: `measurements.SNR_uplink` / `measurements.SNR_downlink`

**Status**: Authoritative as of 2026-05-30, written BEFORE the SUPERSHIFT
SNR-sentinel fix (climb follow-up #1, "Option A") lands on
`fix/climb-engine` (stacks on `e3d818d`). Every future change that writes
or reads either field MUST update this document.

**Driving work item**: climb follow-up #1 — slow ladder climb because the
SNR sentinel is never populated during the CMD's forward MFSK-ACK climb.
`measurements.SNR_uplink` is the value the SUPERSHIFT re-trigger gate
consumes (`arq_commander.cc` re-trigger block, `... && measurements.SNR_uplink > -90`).
During a forward pattern-ACK climb the CMD never decodes an LDPC data frame,
so the only `SNR_uplink` producer that runs on the CMD (the LDPC-decode
producer, §1.2) never fires; the field stays at its ctor sentinel `-99.9`;
the SNR-elevator can never engage; the modem crawls one rung at a time.

**The fix (Option A, minimal)**: at the SNR-suffix decode site (where the
RSP's measured-SNR suffix is decoded and stored in `turbo_received_snr`),
ALSO write `measurements.SNR_uplink` with the SAME decoded value. This
makes a real SNR available to the re-trigger gate mid-climb. Known accepted
tradeoff: the value goes stale after turbo (it is not refreshed by a steady-
state ACK suffix — that would be Option B, explicitly NOT done here).

**Field semantics (decided by §1.2, the canonical producer)**:
- `SNR_uplink` — the canonical producer (`arq_common.cc:6051`) writes it
  **UNCONDITIONALLY for every role** from a decoded LDPC frame's SNR. It is
  "this node's measured SNR of the most recent frame it decoded". On the CMD
  (the only role that runs the suffix-decode path) this is the CMD's view of
  the link the SUPERSHIFT gate consumes. **The fix writes THIS field.**
- `SNR_downlink` — the canonical producer writes it **only when
  `role == RESPONDER`** (`arq_common.cc:6052-6055`). The suffix-decode path is
  CMD-only (§4), so matching §1.2's field semantics exactly means the fix
  does **NOT** write `SNR_downlink`. (If the CMD wrote `SNR_downlink` here it
  would diverge from the canonical producer, which never writes it on the CMD.)

**Cross-layer mandate**: CLAUDE.md §5. The retx-queue and Phase-B-v1 chains
show that silently changing one branch's value of shared ARQ state ships a
sibling bug within hours. This audit walks every producer and consumer
BEFORE the fix lands, and gives each consumer a per-consumer verdict.

`measurements` is a member struct of `cl_arq_controller` (persists for the
session). `SNR_uplink` / `SNR_downlink` are `double`. Ctor-init `-99.9`
(§1.1). A reset path restores `-99.9` on session teardown (§1.6).

---

## §1 Producers — code paths that write `SNR_uplink` / `SNR_downlink`

### §1.1 Ctor / session init — the sentinel
- **File:line**: `arq_common.cc:141-142`
- **Write**: `measurements.SNR_uplink=-99.9;` `measurements.SNR_downlink=-99.9;`
- **Reach**: object construction and every full session re-init (this is the
  body of `init()`-class setup). This is the **sentinel state**: "no SNR
  measurement has been taken this session".
- **Touched by fix?** NO. The sentinel value is unchanged. The fix only
  changes WHEN a real value first overwrites it (now: mid-climb, on the first
  SNR-suffix decode; before: not until an LDPC data frame decodes).

### §1.2 Canonical producer — any decoded LDPC data frame (BOTH fields)
- **File:line**: `arq_common.cc:6051` (`SNR_uplink`),
  `arq_common.cc:6052-6055` (`SNR_downlink`, RESPONDER-only)
- **Write**: `measurements.SNR_uplink = received_message_stats.SNR;`
  then `if(this->role == RESPONDER) measurements.SNR_downlink = received_message_stats.SNR;`
- **Reach**: every successful LDPC data-frame decode, regardless of role.
- **Note (the bug)**: the in-code comment at `:6048-6050` already records
  the gap: *"With pattern ACK, the commander never decodes LDPC during ACK
  detection, so SNR_uplink only refreshes during SWITCH_ROLE when we receive
  data."* The forward MFSK-ACK climb decodes NO LDPC data on the CMD →
  this producer does not run on the CMD's climb → sentinel persists.
- **This is the producer whose field semantics the fix matches** (writes
  `SNR_uplink` unconditionally; writes `SNR_downlink` only as RESPONDER —
  hence the fix, running on the CMD, writes only `SNR_uplink`).
- **Touched by fix?** NO (unchanged). The fix ADDS a second producer (§1.5)
  that writes the SAME field (`SNR_uplink`) on a path this one cannot reach.

### §1.3 RESPONDER: TEST_CONNECTION decode → `SNR_uplink`
- **File:line**: `arq_responder.cc:2046`
- **Write**: `measurements.SNR_uplink=(double)tmp_SNR.f_SNR;` (the CMD's
  reported SNR, parsed from the decoded TEST_CONNECTION LDPC frame).
- **Reach**: RESPONDER, on TEST_CONNECTION during the handshake (pre-climb).
- **Touched by fix?** NO. Different role (RSP), different path (handshake).

### §1.4 CMD: SET_CONFIG SNR_BASED branch → `SNR_downlink`
- **File:line**: `arq_commander.cc:3869` (`measurements.SNR_downlink=tmp_SNR.f_SNR;`)
- **Reach**: a SET_CONFIG-handling path. (Sibling of the §2.4 consumer.)
- **Touched by fix?** NO.

### §1.5 NEW PRODUCER (the fix) — CMD SNR-suffix decode → `SNR_uplink`
- **File:line**: `arq_common.cc` SNR-suffix decode site, alongside the
  existing `turbo_received_snr = decoded_snr;` write (the
  `[CMD-ACK-SNR] ACK detected with SNR=...` branch).
- **Write**: `measurements.SNR_uplink = (double)decoded_snr;` — the SAME
  decoded value the suffix carries (the RSP's measured SNR of the CMD's
  forward signal). Wrapped in the shared pure helper `snr_uplink_from_suffix()`
  (header, static) so the in-process test replays the IDENTICAL expression.
- **Reach**: CMD only (the suffix-decode path runs only inside
  `receive_ack_pattern`, which is CMD-only — see §4), and only when an ACK
  is detected with a valid SNR suffix (`snr_valid==true`).
- **Field choice**: writes `SNR_uplink` ONLY, matching §1.2's CMD-role
  semantics (which writes `SNR_uplink` unconditionally, `SNR_downlink`
  never on the CMD). Does NOT write `SNR_downlink`.
- **Semantic consistency**: the in-turbo SUPERSHIFT jump (§2.5) already
  treats `turbo_received_snr` as the authoritative forward-climb SNR and
  uses `SNR_uplink` as its non-REVERSE fallback (comment `:4442-4444`:
  *"measurements.SNR_uplink is the FORWARD path SNR"*). Writing the decoded
  forward-link SNR into `SNR_uplink` is therefore consistent with the
  existing field meaning.

### §1.6 Session-reset / teardown (sentinel restore)
- The §1.1 init body is re-run on session re-init (CLOSE_CONNECTION →
  `reset_session_state()` → load init config). `SNR_uplink` /
  `SNR_downlink` return to `-99.9`. The fix does not add or remove any
  reset; a new session starts at the sentinel exactly as before.

---

## §2 Consumers — code paths that read `SNR_uplink` / `SNR_downlink`

For each: does populating `SNR_uplink` mid-climb (where it was `-99.9`)
change behavior, and is the new behavior correct? **The fix never makes
`SNR_downlink` change relative to today** (the fix does not write it), so
`SNR_downlink`-only consumers are listed for completeness with verdict
"unaffected".

### §2.1 SUPERSHIFT RE-TRIGGER (the INTENDED consumer) — reads `SNR_uplink`
- **File:line**: `arq_commander.cc` re-trigger block (the `turboshift_phase ==
  TURBO_DONE && gear_shift_on == YES && is_ofdm_config(...) &&
  measurements.SNR_uplink > -90` gate; the body computes
  `snr_ideal = get_configuration(measurements.SNR_uplink - SUPERSHIFT_MARGIN_DB)`
  and re-enters turbo if `gap >= SUPERSHIFT_RETRIGGER_CONFIGS`).
- **Behavior change**: YES, and **this is the point of the fix**. Pre-fix
  `SNR_uplink == -99.9` → gate `> -90` is false → re-trigger can NEVER fire
  on a CMD that climbed via pattern-ACK → slow one-rung-at-a-time climb.
  Post-fix `SNR_uplink` holds the real decoded SNR → gate is ELIGIBLE → the
  SNR-elevator can engage and jump to the SNR-appropriate config.
- **Correct?** YES — this is the designed behavior. NOTE: this gate fires
  only when `turboshift_phase == TURBO_DONE` (turbo finished), AND
  `turbo_received_snr` has reset to `-99.0` at TURBO_DONE
  (`finish_turbo_direction`, `arq_commander.cc:3660`), so after turbo the
  re-trigger has ONLY `SNR_uplink` to consult — which is precisely why the
  sentinel-stuck bug starved it. The Option-B "data-anchored" `anchor_cap`
  (caps `snr_ideal` to `last_data_viable_config+1` unless the optimizer owns
  the band) STILL bounds how far a fresh-but-optimistic SNR can leap — the
  fix does not remove that guard.
- **Anti-storm (the §5 risk)**: when the re-trigger fires it sets
  `turboshift_active = true; turboshift_phase = TURBO_FORWARD`. A SECOND
  re-trigger is structurally impossible until turbo finishes again (the gate
  requires `TURBO_DONE`). The in-turbo SUPERSHIFT jump (§2.5) is itself
  bounded by `turbo_supershift_announce_pending` (set true on each
  `add_message_control(SET_CONFIG)` jump, `arq_commander.cc:4512`; cleared
  only when the announced config is confirmed/aborted, `:2071/:2157/:2233/
  :3656/:4073/:4226/:4334`). So a now-live `SNR_uplink` does NOT enable an
  unbounded SUPERSHIFT storm: re-entry is gated on phase==TURBO_DONE and the
  announce-pending one-jump-in-flight guard. (Verified by test G3.)

### §2.2 In-turbo SUPERSHIFT jump — fallback read of `SNR_uplink`
- **File:line**: `arq_commander.cc:4446-4451`:
  `if(turbo_received_snr > -90) effective_snr = turbo_received_snr;`
  `else if(turboshift_phase != TURBO_REVERSE) effective_snr = measurements.SNR_uplink;`
  `else effective_snr = -99.0;`
- **Behavior change**: MARGINAL. During a forward climb the suffix-decode
  ALSO writes `turbo_received_snr` (the first branch), so `effective_snr`
  takes `turbo_received_snr` and the `SNR_uplink` fallback is usually NOT
  reached. It is reached only in the window where an ACK was accepted without
  a valid suffix (timeout path, `turbo_received_snr` reset to `-99.0`) yet a
  prior suffix had populated `SNR_uplink`. Pre-fix that fallback yielded
  `-99.9` → "force incremental probing"; post-fix it yields a real value →
  the jump can size correctly.
- **Correct?** YES. A real forward-path SNR is strictly better guidance than
  `-99.9` here, and the comment's REVERSE caveat is respected (this branch is
  guarded by `turboshift_phase != TURBO_REVERSE`; during REVERSE the code
  still forces `-99.0`, and the fix only writes during the CMD's forward ACK
  decode, never relabeling the REVERSE meaning). Downstream of `effective_snr`
  the SNR-target is still clamped by `supershift_proven_ceiling`,
  `max_config_override`, the WB/NB ceiling, and the optimizer-handoff cap
  (`:4456-4464`) — the fix loosens none of those.

### §2.3 `finish_turbo_direction` — fallback read of `SNR_uplink`
- **File:line**: `arq_commander.cc:3671`:
  `float effective_snr = (turbo_best_snr > -90) ? turbo_best_snr : measurements.SNR_uplink;`
- **Behavior change**: MARGINAL. `turbo_best_snr` is the running max of every
  suffix-decoded SNR (updated at the same suffix site, `arq_common.cc:5539-5540`),
  so during a climb with any valid suffix decode it is already `> -90` and the
  `SNR_uplink` fallback is not taken. The fallback matters only if turbo
  finished with NO valid suffix ever (then both `turbo_best_snr` and, pre-fix,
  `SNR_uplink` were sentinels). Post-fix `SNR_uplink` may now carry a value
  from an earlier suffix.
- **Correct?** YES. Used only to pick a sane finish/start config; a real value
  beats `-99.9`. No storm risk (one-shot at turbo finish).

### §2.4 CMD SET_CONFIG config selection — reads BOTH (SNR_BASED only)
- **File:line**: `arq_commander.cc:643-644`:
  `forward_configuration = get_configuration(measurements.SNR_downlink);`
  `reverse_configuration = get_configuration(measurements.SNR_uplink);`
- **GATE**: inside `if(gear_shift_algorithm==SNR_BASED)`. The SUCCESS_BASED_LADDER
  else-branch ignores both fields (uses `negotiated_configuration`).
- **Default**: `gear_shift_algorithm` is `SUCCESS_BASED_LADDER` everywhere
  (`arq_common.cc:328`, `datalink_config.cc:46`, and every runtime initializer
  in `arq_commander.cc`: `:5574,:5673,:5747,:5847,:6271`). SNR_BASED is not the
  shipped default.
- **Behavior change (SNR_BASED only)**: in a hypothetical SNR_BASED session,
  pre-fix `reverse_configuration = get_configuration(-99.9)` = lowest config;
  post-fix it = the real-SNR config. This is MORE correct (the whole point of
  SNR_BASED is to pick config from SNR), not a regression. In the shipped
  SUCCESS_BASED_LADDER default this branch is dead → **zero behavior change.**
- **Correct?** YES (improvement under SNR_BASED; no-op under the default).

### §2.5 CMD TEST_CONNECTION frame pack — reads `SNR_uplink`
- **File:line**: `arq_commander.cc:581`: `tmp_SNR.f_SNR=(float)measurements.SNR_uplink;`
- **Reach**: building the TEST_CONNECTION control frame during the handshake,
  BEFORE the climb starts. At that point no suffix decode has happened yet, so
  `SNR_uplink` is still the sentinel both pre- and post-fix → **no change at
  the handshake.** (Were a TEST_CONNECTION ever re-sent after a climb, the
  packed value would be a real SNR instead of `-99.9` — strictly more
  informative for the peer, never a regression.)
- **Correct?** YES.

### §2.6 link_status report — reads BOTH (`get_configuration`, telemetry)
- **File:line**: `arq_commander.cc:643-644` (the §2.4 SNR_BASED block) and the
  `[GEARSHIFT]`/link telemetry print at `:660-662`. Also the diagnostic dump
  `arq_common.cc:7141-7142` (`printf("measurements.SNR_uplink= %.2f ...")`).
- **Behavior change**: telemetry/logging only — prints a real value instead of
  `-99.9` during the climb. No control decision keys off the print.
- **Correct?** YES (more accurate logs).

### §2.7 RESPONDER turbo ACK+SNR gate — reads `SNR_uplink`
- **File:line**: `arq_responder.cc:1124`:
  `... && measurements.SNR_uplink > -90;` gating `send_ack_pattern_with_snr(...)`.
- **Role**: RESPONDER. The fix writes on the CMD only (§4). On the RSP,
  `SNR_uplink` is produced by §1.2 (LDPC data decode, RESPONDER does decode
  data) and §1.3 (TEST_CONNECTION). **The fix never executes on the RSP**, so
  this gate's input is byte-identical to today.
- **Correct?** YES — unaffected (CMD-only fix). This is the producer of the
  suffix the CMD decodes; the loop is RSP-measures → suffix → CMD-stores. The
  fix only adds the CMD-store step on a path that had been dropping the value.

### §2.8 RESPONDER reverse-SNR pack — reads `SNR_downlink`
- **File:line**: `arq_responder.cc:2167`: `tmp_SNR.f_SNR=(float)measurements.SNR_downlink;`
- **Behavior change**: NONE — the fix does not write `SNR_downlink`, and this
  is the RSP role besides. Listed for completeness.
- **Correct?** YES — unaffected.

---

## §3 Valid states (before any producer writes)

- Both fields sit at `-99.9` (§1.1) from session init until the first
  producer fires. Every consumer treats `> -90` as "have a real measurement"
  and `<= -90` (i.e. `-99.9` / `-99.0`) as "no measurement → fall back to
  incremental/blind behavior". The fix preserves this contract: it only moves
  the moment `SNR_uplink` FIRST crosses above `-90` on the CMD earlier (to the
  first valid SNR-suffix decode of the forward climb), and only to a genuine
  measured value.

---

## §4 Invariant: the suffix-decode path is CMD-only

`receive_ack_pattern` (which contains the SNR-suffix decode site) is called
ONLY from `arq_commander.cc` (`:172`, `:1785`, `:2946`). There is no caller in
`arq_responder.cc`. Therefore the new producer (§1.5) executes only when this
node is the COMMANDER decoding the RESPONDER's ACK. This is what makes the
field-semantics choice unambiguous: matching §1.2's CMD-role behavior means
writing `SNR_uplink` only.

---

## §5 What the fix changes (summary of the audit)

The fix adds ONE producer (§1.5) that writes `SNR_uplink` (a field §1.2
already writes for all roles) on the CMD forward-climb path that §1.2 cannot
reach. It does NOT write `SNR_downlink` (matching §1.2's CMD-role semantics).

- The INTENDED consumer (§2.1 SUPERSHIFT re-trigger) gains eligibility — the
  designed behavior. Anti-storm bounds (`TURBO_DONE` phase gate +
  `turbo_supershift_announce_pending`) still hold; a live `SNR_uplink` does
  not enable unbounded re-entry.
- The fallback consumers (§2.2, §2.3) improve from "blind incremental" to
  "real-SNR-sized" only in narrow windows; both already prefer
  `turbo_received_snr` / `turbo_best_snr`, which the same suffix site writes.
- The config-selection consumer (§2.4) is dead under the shipped
  SUCCESS_BASED_LADDER default; under SNR_BASED it improves.
- All other reads (§2.5–§2.8) are telemetry, handshake-time (still sentinel),
  or RSP-role (the fix never runs there) → no behavior change.

No consumer's assumption is violated; no consumer needs constraining. The
accepted tradeoff is staleness after turbo (§ driving-item note), which is the
Option-A scope boundary.

---

## §6 Cross-layer regression test (paired with this doc)

`--test-climb-engine` **Part G** (`arq_commander.cc::test_climb_engine`)
replays the REAL §1.5 write expression via the shared helper
`snr_uplink_from_suffix()` and the REAL §2.1 gate predicate:

- **G1** — after a simulated SNR-suffix decode, `measurements.SNR_uplink > -90`
  (FAIL-BEFORE on `e3d818d`: the helper does not exist / no producer writes it
  on the climb → stays `-99.9`).
- **G2** — with `SNR_uplink` populated, the §2.1 re-trigger eligibility predicate
  flips from false → true (the gate that was unreachable is now reachable).
- **G3** — anti-storm: repeated suffix decodes (each writing `SNR_uplink`) do NOT
  let the re-trigger re-enter unboundedly — the `TURBO_DONE`-phase gate +
  `turbo_supershift_announce_pending` admit at most one re-entry until turbo
  finishes again.

**Honest scope**: G1–G3 prove ELIGIBILITY and the ANTI-STORM bound in-process.
They do NOT prove the climb-SPEED win — that is hardware-confirmable only
(does the CMD reach e.g. CONFIG_6 via SUPERSHIFT jumps faster than the slow
one-rung ladder at clean/moderate SNR?). The parent tests that on the wire.
