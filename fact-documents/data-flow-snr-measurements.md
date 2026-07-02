# Data-Flow Audit: `measurements.SNR_uplink` / `measurements.SNR_downlink`

**Status**: Authoritative as of 2026-05-31. §1–§6 cover the SUPERSHIFT
SNR-sentinel PRODUCER fix (climb follow-up #1, "Option A", shipped on
`fix/climb-engine` @ `446887c`). **§1.7 + §7 cover the ENABLEMENT fix
(climb follow-up #1b, "Option 1") that lands ON TOP of `446887c`** — it
breaks the bootstrap deadlock that kept the §1.5 producer from EVER running
on the CMD's forward pattern-ACK climb. **§8 (2026-05-31, READ+PLAN ONLY, NO
code shipped) audits the PROPOSED "suffix the TRUE OFDM SNR" climb-throttle fix
and finds the literal swap INFEASIBLE — see §8 verdict.** Every future change
that writes or reads either field — OR that touches `turbo_snr_ack_enabled` (the
decode-branch selector that gates the §1.5 producer) — MUST update this document.

> **ANCHOR REFRESH 2026-07-01 (registry content VALID; only line numbers drifted —
> verified against HEAD).** The §1-§7 citations are heavily pre-drift; the §8 block
> (written 2026-05-31) used a fresher-but-also-now-drifted set. Current HEAD sites:
> | Symbol (doc §) | doc anchor(s) | **current HEAD** |
> |---|---|---|
> | §1.2 canonical producer `SNR_uplink = received_message_stats.SNR` | `arq_common.cc:6051` (§8: `:6087`) | **`arq_common.cc:12846`** (SNR_downlink `:12849`) |
> | §1.5 CMD suffix producer `SNR_uplink = snr_uplink_from_suffix(...)` | `arq_common.cc:5555` (§8: `:5576`) | **`arq_common.cc:11875`** |
> | §1.5/§7 `if(turbo_snr_ack_enabled)` branch in `receive_ack_pattern()` | `arq_common.cc:5516` (§8: `:5537`) | **`arq_common.cc:11836`** |
> | §7.0 re-trigger setter `turbo_snr_ack_enabled = true` | `arq_commander.cc:4596` | **`arq_commander.cc:7594`** |
> | §1.7 enablement `turbo_snr_ack_enabled = turbo_snr_ack_expected_on_control(...)` | `arq_commander.cc:1050` | **`arq_commander.cc:11658`** |
>
> Treat every other `file:line` in §1-§8 below as approximate; grep the symbol name to relocate.

**The bootstrap deadlock (follow-up #1b, the §1.5 producer never ran)**: the
§1.5 producer at `arq_common.cc:5555` runs ONLY inside
`receive_ack_pattern()`'s `if(turbo_snr_ack_enabled)` branch
(`arq_common.cc:5516`). On the CMD, `turbo_snr_ack_enabled` was set TRUE in
EXACTLY ONE place — the SUPERSHIFT re-trigger (`arq_commander.cc:4596`,
was `:4579` pre-edit) — itself gated by `measurements.SNR_uplink > -90`
(`arq_commander.cc:4548-4549`). DEADLOCK: the §1.5 producer is the only thing
that lifts `SNR_uplink` off the `-99.9` sentinel on the CMD during a forward
pattern-ACK climb, but it cannot run until `SNR_uplink > -90`, which only it
provides. Proof (hardware): 0× `[CMD-ACK-SNR]`, 0× `[TURBO]` in cmd.log;
`SNR_uplink` stayed `-99.9`; SUPERSHIFT never armed. **So Option A shipped a
producer that, on the CMD's forward climb, was unreachable.** §1.7 adds the
missing enablement.

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

### §1.7 NEW ENABLEMENT (follow-up #1b) — CMD arms the §1.5 producer's branch
- **File:line**: `arq_commander.cc:1050` (inside the
  `if(messages_control.data[0]==SET_CONFIG)` block in
  `process_messages_tx_control()`, at the control-TX → `RECEIVING_ACKS_CONTROL`
  transition).
- **Write**: `turbo_snr_ack_enabled = turbo_snr_ack_expected_on_control(
  turboshift_active, turboshift_phase, messages_control.data[0]);`
- **The pure helper** (`arq.h`, static, no side effects so Part H replays the
  identical expression):
  `(turbo_active || phase != TURBO_DONE) && control_code == SET_CONFIG`.
- **What it changes**: `turbo_snr_ack_enabled` is the BRANCH SELECTOR consumed
  by `receive_ack_pattern()` (`arq_common.cc:5452/5516`) — true ⇒ the SNR-suffix
  branch (`detect_ack_snr_from_passband`, which runs the §1.5 producer); false ⇒
  the normal-mode ACK branch (`detect_ack_pattern_from_passband`, no SNR decode).
  Pre-#1b the CMD had no producer of `turbo_snr_ack_enabled=true` on the forward
  climb (the only setter, the re-trigger `:4596`, was deadlocked — see the
  status block). This write ARMS the decode at every turbo SET_CONFIG-ACK wait,
  so the §1.5 producer finally runs mid-climb and lifts `SNR_uplink` off `-99.9`.
- **Symmetry**: this is the EXACT counterpart of the RSP's SNR-suffix SEND gate
  (`arq_responder.cc:1122-1124`:
  `(turboshift_active || turboshift_phase != TURBO_DONE) && data[0]==SET_CONFIG
  && SNR_uplink > -90`), MINUS the `SNR_uplink > -90` conjunct. That conjunct is
  the RSP's "do I HAVE a measured SNR to encode" check (RSP gets `SNR_uplink`
  from decoding the SET_CONFIG LDPC frame, `arq_responder.cc:2046`). The CMD is
  the opposite end of the loop — it only needs the DECODER armed to RECEIVE the
  suffix; gating the CMD on `SNR_uplink > -90` would re-create the very deadlock
  (the CMD has no `SNR_uplink` yet — that is what the suffix is FOR).
- **Field choice**: does NOT touch `SNR_uplink` / `SNR_downlink` directly — it
  enables the BRANCH whose producer (§1.5) writes `SNR_uplink`. Lifecycle of the
  flag is bounded by turbo: set here on each turbo SET_CONFIG; cleared by
  `finish_turbo_direction()` (`arq_commander.cc:3657`) before any
  `TRANSMITTING_DATA` transition (§7.3). So the flag is provably FALSE on every
  DATA-ACK wait — the SACK-suffix path is never routed to the SNR decoder (§7.1).
- **Touched by #1b?** YES — this IS the #1b fix. The §1.5 producer line is
  UNCHANGED (it shipped at `446887c`); #1b only makes its branch reachable.

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

---

## §7 ENABLEMENT cross-layer audit (CLAUDE.md §5) — follow-up #1b

The §1.7 enablement writes `turbo_snr_ack_enabled` — the BRANCH SELECTOR that
gates the §1.5 producer AND chooses which detector `receive_ack_pattern()` runs.
`turbo_snr_ack_enabled` is shared across the PHY-decode layer (which detector)
and the ARQ layer (which producer). Walk every producer/consumer.

### §7.0 Producers / consumers of `turbo_snr_ack_enabled`

- **Producers (CMD)**: ctor init false (`arq_common.cc:366`); reset_session_state
  init false (`arq_common.cc:2044`, `:3000`); SUPERSHIFT re-trigger sets TRUE
  (`arq_commander.cc:4596`); `finish_turbo_direction()` clears FALSE
  (`arq_commander.cc:3657`); SWITCH_ROLE-BREAK clears FALSE
  (`arq_commander.cc:2234`); **NEW §1.7: set to the helper value at the turbo
  SET_CONFIG control-TX (`arq_commander.cc:1050`)**.
- **Producers (RSP)**: ctor/reset false; SWITCH_ROLE reverse-probe sets TRUE
  (`arq_responder.cc:1315`); skip-reverse / role-return clear FALSE
  (`arq_responder.cc:1302/1334`). **The §1.7 write is in `process_messages_tx_control`,
  a CMD function (the RSP TX path is `acknowledging_*`), so #1b never executes on
  the RSP** — the RSP's flag lifecycle is byte-identical to today.
- **Consumers**: `receive_ack_pattern()` tail-window sizing + branch selection
  (`arq_common.cc:5400/5452/5516`); `zero_mfsk_ack_audio_tail()` tail-window
  (`arq_common.cc:5400`). Both are CMD-only (`receive_ack_pattern` is CMD-only,
  §4). No ARQ control decision reads the flag directly except through these.

### §7.1 CHECK 2 (the crux) — NO SACK-vs-SNR suffix collision

**Verdict: collision is STRUCTURALLY IMPOSSIBLE; the SACK path is untouched.**

Two distinct ACK suffixes exist:
- **SNR suffix** — on SET_CONFIG ACKs during turbo. Decoded by
  `detect_ack_snr_from_passband` (`telecom_system.cc:3236`), reached ONLY via
  `receive_ack_pattern()`'s `if(turbo_snr_ack_enabled)` branch (`arq_common.cc:5516`).
- **SACK suffix** — on DATA ACKs. Decoded by the SEPARATE SACK-v2 cross-check
  (`decode_sack_v2_frame` / `decode_suffix_tones`, `arq_commander.cc:1789-1840`,
  `ofdm.cc:3986`) in the DATA-ACK path `process_messages_rx_acks_data()`
  (`arq_commander.cc:2378`, callsite `:2946`).

Three independent reasons they cannot cross:

1. **Different functions, selected by `connection_status`.** SET_CONFIG ACKs are
   awaited in `process_messages_rx_acks_control()` (`:1763`, callsite `:1785`),
   entered at `RECEIVING_ACKS_CONTROL`. DATA ACKs are awaited in
   `process_messages_rx_acks_data()` (`:2378`, callsite `:2946`), entered at
   `RECEIVING_ACKS_DATA`. They never run in the same tick.

2. **The enablement is gated `control_code == SET_CONFIG`** (§1.7 helper). A DATA
   ACK is not a control frame at all — the helper is never even evaluated on the
   data path (it lives in the SET_CONFIG control-TX block). Part H2/H2b/H3 assert
   a DATA ACK (`ACK_RANGE`), a `SWITCH_ROLE`, and a non-turbo `SET_CONFIG` all
   return FALSE from the helper.

3. **The flag is provably FALSE on every DATA-ACK wait** (§7.3): turbo always
   ends (the flag cleared) before `TRANSMITTING_DATA`. So even the
   `receive_ack_pattern()` call inside `process_messages_rx_acks_data` (`:2946`)
   takes the `else` (normal-mode) branch (`arq_common.cc:5640`) — it never calls
   `detect_ack_snr_from_passband`. The data ACK's SACK suffix is fed to the SACK
   decoder only, exactly as before #1b.

The test campaign confirmed the SACK path was working (9× CLEAN data ACKs);
#1b touches neither the SACK decoder nor the data-ACK branch — it only flips a
flag that is already false on that path.

### §7.2 CHECK 1 — the CMD still correctly DETECTS the ACK pattern

**Verdict: ACK detection is UNCHANGED on the armed branch.**
`detect_ack_snr_from_passband` (`telecom_system.cc:3236`) detects the base ACK
with the SAME `ofdm.detect_ack_pattern()` call, the SAME `ack_tones` /
`ack_pattern_len` / `ack_match_threshold`, and the SAME `metric >= 3.0` gate
(`:3264`) as the normal-mode `detect_ack_pattern_from_passband`. The ONLY delta
is that it additionally reserves + decodes `SNR_SUFFIX_LEN` symbols. ACK
ACCEPTANCE keys off `matched_count >= ack_match_threshold` (`arq_common.cc:5529`),
NOT off suffix validity: if the suffix is garbled, the `else` defer path
(`:5586`) waits ≤500 ms then accepts the ACK anyway (`:5595-5624`, returns true).
So enabling the SNR branch for a SET_CONFIG ACK cannot cause a MISSED control
ACK. (This is the SAME branch the re-trigger `:4596` already used post-turbo;
#1b only makes it reachable on the forward climb too.) The ≤500 ms suffix-wait
is bounded and is the existing, intended turbo SET_CONFIG-ACK behavior, not a
new latency path.

### §7.3 CHECK 3 — SUPERSHIFT still bounded; no storm; flag never leaks to data

**Verdict: storm guards intact; flag lifecycle bounded by turbo.**

- The §1.7 write does NOT touch the re-trigger's own guards. The re-trigger
  (`arq_commander.cc:4548-4596`) still requires `turboshift_phase == TURBO_DONE`,
  still applies `supershift_proven_ceiling`, the `anchor_cap`
  (`last_data_viable_config + 1` unless the optimizer owns the band), and the
  `gap >= SUPERSHIFT_RETRIGGER_CONFIGS` gate. #1b only PRIMES the `SNR_uplink`
  input the gate reads (via §1.5) — Part G3/G3b already prove a live `SNR_uplink`
  admits exactly ONE re-entry (the `TURBO_DONE`-phase gate +
  `turbo_supershift_announce_pending` one-jump-in-flight guard).
- **Flag never leaks to a data-ACK wait** (the §7.1 reason 3 proof): every path
  to `TRANSMITTING_DATA` either runs `finish_turbo_direction()` first (which sets
  `turbo_snr_ack_enabled=false` at `arq_commander.cc:3657` BEFORE any of its
  branches, including its own `:3763` data fallback) or occurs when turbo was
  already `TURBO_DONE` + inactive (e.g. break-recovery `:4430`, where the flag
  was already cleared). Walked exits of the turbo SET_CONFIG-ACK handler
  (`arq_commander.cc:4434+`): each either re-queues a SET_CONFIG (control path,
  flag re-affirmed true — no leak) or calls `finish_turbo_direction()` (cleared).
- **No clobber of the re-trigger.** The re-trigger sets the flag TRUE then queues
  a SET_CONFIG; on the next tick `process_messages_tx_control` reaches `:1050`
  with `turboshift_active==true` (set at `:4576`) → the helper returns true →
  re-affirms, never clobbers.
- **For a NON-turbo SET_CONFIG** the helper returns FALSE, so #1b sets the flag
  false — which is CORRECT and SYMMETRIC: the RSP also sends a bare ACK (no SNR
  suffix) there (its send gate is also false), so the CMD must decode in
  normal mode. This also prevents a stale TRUE from a prior turbo leaking into a
  later non-turbo SET_CONFIG wait. (Part H3.)

### §7.4 Part H (`--test-climb-engine`) — fail-before / pass-after

Replays the REAL §1.7 helper `turbo_snr_ack_expected_on_control()` (the SAME
expression the production `:1050` assignment uses) + the REAL §1.5 producer
helper + the REAL §2.1 gate:

- **H0** — climb-entry state: decode DISARMED + `SNR_uplink` at `-99.9` (the
  deadlock, exactly as `reset_session_state` `:2038-2044` leaves it).
- **H1 / H1b** — a turbo SET_CONFIG control-TX ARMS the decode WITHOUT first
  requiring `SNR_uplink > -90` (the deadlock break: the enable is independent of
  the value the producer would supply).
- **H1c** — full chain: armed decode → §1.5 producer primes `SNR_uplink` → §2.1
  re-trigger becomes eligible. None of this could happen pre-#1b.
- **H2 / H2b** — the §7.1 collision guard: a DATA ACK (`ACK_RANGE`) and a
  `SWITCH_ROLE` do NOT arm the SNR decode (SACK suffix never routed to it).
- **H3** — a NON-turbo SET_CONFIG does NOT arm (scoped to turbo; matches the RSP
  bare-ACK).
- **H4** — a mid-direction-switch SET_CONFIG (`phase != TURBO_DONE`,
  `active==false`) STILL arms via the disjunct (loop stays symmetric across the
  role swap).

**FAIL-BEFORE** (verified 2026-05-30 by temporarily reverting the helper body to
`return false;` — modelling `446887c`, where nothing armed the decode on the
forward climb): **H1 / H1b / H1c / H4 FAIL** (flag stays false → producer never
runs → `SNR_uplink` never leaves `-99.9` → re-trigger never eligible);
**H0 / H2 / H2b / H3 stay PASS** (they assert the disarmed-entry state and the
collision guard, which hold even in the no-op — confirming Part H is not
trivially all-or-nothing). **PASS-AFTER**: all PASS; the temp revert was
reverted; the shipped helper does the enablement.

**Honest scope**: H proves the DEADLOCK IS BROKEN (the producer's branch is now
reachable on the forward climb) and the SACK path is PRESERVED — in-process.
It does NOT prove the climb-SPEED win. That is hardware-only: the parent will
high-SNR (WGN:30/40) test that `[CMD-ACK-SNR]` now appears, `SNR_uplink` leaves
`-99.9`, `[TURBO]` re-trigger fires, and the CMD elevator-jumps instead of
crawling one rung at a time.

### §7.5 Related fact documents

- `gearshift-climb-engine.md` — the climb engine #1b unblocks (the SUPERSHIFT
  elevator is the fast path off the one-rung FRAME-UP/LADDER-UP ladder).
- `data-flow-batch-size.md` §4/§6 — the connect-path default-init trap (the 4th
  wire failure); a reminder that "the connect-path state is not what the steady-
  state code assumes" — checked here for `turbo_snr_ack_enabled` (ctor/reset
  init false; §1.7 is the first forward-climb producer of true on the CMD).

---

## §8 PROPOSED "suffix the TRUE OFDM SNR" fix — AUDIT + FEASIBILITY (2026-05-31, read+plan only, NO code shipped)

**Driving work item**: the unpinned climb is throttled because the elevator
(`supershift_retrigger_target` via the re-trigger gate, `arq_commander.cc:4722-4757`)
at CONFIG_0 computes `get_configuration(measurements.SNR_uplink − SUPERSHIFT_MARGIN_DB)`,
and on a *fixed* channel `SNR_uplink` reads only **~1.0 dB at CONFIG_0** but **~15.0 dB
at CONFIG_4**. The proposed fix (as handed to this audit): make the RSP suffix the TRUE
OFDM data-frame SNR (`received_message_stats.SNR`, the canonical producer
`arq_common.cc:6087`) when it has just decoded OFDM data, so SNR_uplink reads ~15 at
CONFIG_0 and the elevator jumps CONFIG_0→CONFIG_13 (bounded by RETRIGGER_MAX_LEAP=13).

**THIS SECTION'S VERDICT (up front): the literal "swap the suffix source" fix is
INFEASIBLE at the one boundary that matters (ROBUST→CONFIG_0), because at that boundary
there is NO OFDM SNR for the RSP to suffix. The 1.0-vs-15.0 split is NOT a
control-vs-OFDM *estimator* choice made at one site — it is a TRANSITION-ORDERING
artifact: the SET_CONFIG that PROMOTES to CONFIG_0 is decoded by the RSP while it is
still on the OLD (ROBUST = MFSK) config, whose SNR estimator is a hardcoded `0.0`. The
real fix is larger than a suffix-source swap; §8.5 describes what it actually requires.**

### §8.1 EXACT production of `measurements.SNR_uplink` at the elevator (answer to task #1)

The elevator consumer is the SUPERSHIFT re-trigger, `arq_commander.cc:4722-4723`:
`turboshift_phase == TURBO_DONE && gear_shift_on == YES && is_ofdm_config(current_configuration)
&& measurements.SNR_uplink > -90`, body `int snr_ideal = elevator_target_from_snr();`
(`:4737`). It runs at the TAIL of `process_messages_rx_acks_data()` (the data-ACK pass).

**On the CMD, `measurements.SNR_uplink` has exactly ONE live producer on the climb:**
the SNR-suffix decode site `arq_common.cc:5576`
(`measurements.SNR_uplink = snr_uplink_from_suffix(decoded_snr);`), inside
`receive_ack_pattern()`'s `if(turbo_snr_ack_enabled)` branch (`:5537`). The canonical
LDPC-decode producer `arq_common.cc:6087` does NOT run on the CMD during the climb (the
CMD decodes no LDPC data while climbing — §1.2 note). So whatever the CMD reads at the
elevator is the value `decoded_snr` carried by the LAST SET_CONFIG-ACK suffix it decoded.

**`decoded_snr` = the RSP's `measurements.SNR_uplink` at the moment the RSP built that
suffix** (`arq_responder.cc:1130`, `send_ack_pattern_with_snr((float)measurements.SNR_uplink)`),
**quantized** through the MFSK suffix codec (round-trip `snr_to_tone`→`tone_to_snr`,
`mfsk.cc:594-611`). And the RSP's `SNR_uplink` is set by the SAME canonical producer
`arq_common.cc:6087` (`measurements.SNR_uplink = received_message_stats.SNR;`) — which on
the RSP DOES run, on EVERY decoded frame (the block is gated only by
`received_message_stats.message_decoded==YES`, `arq_common.cc:5959` — NOT by data-vs-control
nor by modulation). So the RSP's `SNR_uplink` = `received_message_stats.SNR` of **the last
frame the RSP decoded**.

**`received_message_stats.SNR` is set in `telecom_system.cc` and is a GENUINELY DIFFERENT
estimator per modulation** (answer (a), confirmed by code — NOT a propagation lag (b)):
- **`telecom_system.cc:2730`**: `if(M == MOD_MFSK) receive_stats.SNR = 0.0;` — a HARDCODED
  PLACEHOLDER with the comment `// TODO: estimate SNR from peak tone energy vs noise
  energy`. **MFSK has NO real SNR estimator.** Round-tripped through the WB suffix codec,
  `0.0` → `snr_to_tone((0+5)/2+0.5)=3` → `tone_to_snr(3)=3*2−5 = 1.0`. **THAT is the 1.0.**
- **`telecom_system.cc:2738` (LS) / `:2766`/`:2770` (ZF, via `ofdm.measure_SNR`,
  `ofdm.cc:2111-2124`, `SNR=−10·log10(EVM_variance)`)**: the REAL OFDM data-frame SNR
  (~15 at a clean/moderate channel). THAT is the 15.0.
- ROBUST_0/1/2 are MFSK (`telecom_system.cc:4876-4877` set `new_mfsk_M`=32/16;
  `is_robust_config` ⇒ MFSK), so a robust-config decode ALWAYS yields `SNR=0.0`.

**WHY 1.0 at CONFIG_0 but 15.0 at CONFIG_4 on a FIXED channel — the transition-ordering
artifact (the crux)**: the gearshift SET_CONFIG is sent by the CMD on the OLD
(`current_configuration`) config — the legacy LDPC control-TX path
`arq_commander.cc:1022-1028` (`set_mfsk_ctrl_mode(false); send_batch();`) emits a full
frame on the CURRENT config (SET_CONFIG is NOT in the MFSK-suffix PHY-swap list, which
covers only START_CONNECTION `:925` and TEST_CONNECTION `:935`). The RSP decodes that
SET_CONFIG on the OLD config and ACKs on the OLD config (it loads the NEW
`data_configuration` only AFTER sending the ACK — `arq_responder.cc:1140-1143`, and the
SET_CONFIG handler defers the load to `acknowledging_control`, `:2503-2507`). Therefore:
- The promotion **ROBUST_2 → CONFIG_0** SET_CONFIG is decoded by the RSP while on
  **ROBUST_2 (MFSK)** ⇒ `received_message_stats.SNR = 0.0` ⇒ RSP `SNR_uplink = 0.0` ⇒
  suffix `0.0` ⇒ round-trips to **~1.0** on the CMD. The CMD then runs the elevator at
  `current_configuration == CONFIG_0` reading **SNR_uplink ≈ 1.0**. ← the throttle.
- The promotion **CONFIG_0 → CONFIG_1** (any OFDM→OFDM) SET_CONFIG is decoded by the RSP
  while on **CONFIG_0 (OFDM)** ⇒ `received_message_stats.SNR ≈ 15` ⇒ suffix ~15 ⇒ CMD
  reads ~15 and the elevator at CONFIG_1+ sees the true SNR. ← "15 at CONFIG_4".

So `SNR_uplink` reads the SNR of the config the modem was on **one rung ago**, and at the
ROBUST→OFDM boundary "one rung ago" is MFSK (estimator = 0.0). It is BOTH a different
estimator (a) AND an ordering effect — the estimator difference only bites because the
promotion frame is carried on the old (robust/MFSK) rung.

**Does `received_message_stats.SNR` (the OFDM value) feed `SNR_uplink`?** Yes, but only
via the canonical producer `:6087`, which on the CMD does not run on the climb, and on the
RSP runs for the LAST decode (which at the critical boundary is the MFSK SET_CONFIG, not an
OFDM frame). The OFDM data-frame SNR the RSP measures while decoding a CONFIG_0 DATA batch
is NOT relayed to the CMD: the RSP's DATA-ACK send path (`arq_responder.cc:1864-1868`) emits
a PLAIN MFSK ACK (`set_mfsk_ctrl_mode(true); send_batch();`) with NO SNR suffix
(`send_ack_pattern_with_snr` is called ONLY on the control SET_CONFIG-ACK path `:1130`,
never on the data-ACK path). Routing the data-ACK through the SNR decoder was tried (the
A1 widening) and CAUSED A BUG — §18 / Part N: the widened arm leaked onto the data-ACK
wait, writing a bogus `SNR_uplink` from the data-ACK tail (`arq_common.cc:5576`); the fix
`clear_snr_arm_for_data_ack_wait()` deliberately CLOSED that path. So today the OFDM
data-frame SNR has NO channel back to the CMD's `SNR_uplink` at all.

### §8.2 §5 audit — the FIVE questions for the PROPOSED fix

**Q1 Producers of `measurements.SNR_uplink`** (extends §1; unchanged today):
- `arq_common.cc:141` ctor sentinel `-99.9` (§1.1).
- `arq_common.cc:6087` canonical LDPC-decode producer, ALL roles (§1.2). On the RSP this
  is ALSO the producer of the value the RSP suffixes (it sets RSP `SNR_uplink`).
- `arq_common.cc:5576` CMD SNR-suffix decode (§1.5) — the only CMD climb producer.
- `arq_responder.cc:2046` RSP TEST_CONNECTION decode (§1.3); `arq_commander.cc:3869`
  CMD SET_CONFIG SNR_BASED (§1.4, `SNR_downlink`).
- **The proposed fix adds NO new producer of `SNR_uplink`. It changes the ARGUMENT to the
  RSP's existing suffix SEND** (`arq_responder.cc:1130`) — i.e. it changes what
  `decoded_snr` the CMD's §1.5 producer receives. The producer SITE `:5576` is unchanged.

**Q2 Consumers of `SNR_uplink`** (the fix raises the CONFIG_0 value from ~1.0 to ~15 —
walk each; consumers enumerated in §2, plus the §15/§16 sites that post-date §2):
1. **Elevator re-trigger gate** `arq_commander.cc:4722-4757` (the INTENDED consumer; §2.1)
   — eligibility gate `SNR_uplink > -90` already passes at 1.0; the value feeds
   `elevator_target_from_snr()` → `get_configuration(SNR_uplink−6)`. Fix EFFECT: target
   rises from `get_configuration(1−6)=CONFIG_4`-but-clamped to `get_configuration(15−6=9)≈CONFIG_13`,
   so `gap ≥ SUPERSHIFT_RETRIGGER_CONFIGS` and the elevator JUMPS. **This is the goal.**
2. **`elevator_target_from_snr()` + `supershift_retrigger_target()`**
   (`arq_commander.cc:174-186`, `arq.h:819-866`) — the §15/§16 chokepoint both elevator
   sites share. Reads `SNR_uplink` as `snr_uplink`. **CRITICAL: the §15 `is_ofdm_config(anchor)`
   conjunct (`arq.h:842-844`) gates the multi-rung jump on the ANCHOR being OFDM, NOT on the
   SNR value.** A higher CONFIG_0 SNR does NOT bypass this — see §8.3.
3. **FRAME-UP elevator** `arq_commander.cc:3733` (`elevator_target_from_snr()`; §15.3
   consumer 2) — same chokepoint, same `is_ofdm_config(anchor)` gate.
4. **In-turbo SUPERSHIFT jump fallback** `arq_commander.cc:4598` (§2.2) — reached only when
   `turbo_received_snr ≤ -90`; uses `SNR_uplink` for `effective_snr`, then routed through
   the §16 ROOT-2 clamp (`arq_commander.cc:4631-4635`, the SAME `supershift_retrigger_target`).
5. **`finish_turbo_direction` finish/start pick** `arq_commander.cc:3820` (§2.3) — fallback
   after `turbo_best_snr`; one-shot, picks a sane finish config. Higher value = better pick.
6. **CMD SET_CONFIG SNR_BASED config selection** `arq_commander.cc:676` (§2.4) — DEAD under
   the shipped `SUCCESS_BASED_LADDER` default (`arq_common.cc:328`); improvement under
   SNR_BASED.
7. **CMD TEST_CONNECTION frame pack** `arq_commander.cc:613` (§2.5) — handshake-time, before
   any suffix; still sentinel → no change.
8. **link_status / GEARSHIFT telemetry** `arq_commander.cc:694`, diag dump
   `arq_common.cc:7177` (§2.6) — logging only.
9. **RSP turbo ACK+SNR SEND gate** `arq_responder.cc:1122-1124` (§2.7) — this is the SEND
   gate the fix would MODIFY (see §8.4). It currently reads RSP `SNR_uplink > -90` to decide
   whether to suffix at all, and passes `(float)SNR_uplink` as the value.
10. **RSP reverse-SNR pack** `arq_responder.cc:2167` (`SNR_downlink`; §2.8) — unaffected.

**Q3 Valid states (esp. BEFORE the first OFDM decode)**:
- `-99.9` ctor sentinel (§1.1) / `-99.0` post-turbo reset of `turbo_received_snr` (NOT
  `SNR_uplink`; `arq_commander.cc:3660`-class). Both `≤ -90` ⇒ "no measurement".
- `~1.0` — the round-trip of the MFSK placeholder `0.0`. **This is a VALID state the consumers
  currently see at CONFIG_0** and is the value the elevator is (correctly, per §15) NOT
  allowed to jump on from a ROBUST anchor. **THE KEY HAZARD STATE: a value that LOOKS like a
  real measurement (`> -90`) but encodes "MFSK, no real estimate".** `0.0`/`1.0` is
  indistinguishable from a genuine 0–1 dB OFDM channel by value alone.
- `~15` — a real OFDM measurement at CONFIG_0+.

**Q4 Invariants each consumer assumes**:
- Elevator/chokepoint (consumers 1-4): "`SNR_uplink > -90` ⇒ a usable forward-link SNR,"
  AND (post-§15) "a multi-rung jump additionally requires `is_ofdm_config(anchor)`." The
  §15 conjunct exists PRECISELY because the value-only invariant was FALSE at deep SNR (the
  1.0 over-report). **The fix must not weaken the anchor conjunct.**
- RSP SEND gate (consumer 9): "I only suffix a value I actually measured (`SNR_uplink > -90`)."

**Q5 What the fix changes for each consumer**: the fix raises the value the CMD reads at
CONFIG_0 from ~1.0 to ~15 *when the underlying frame was genuinely OFDM*. For consumer 1/2/3
this FLIPS `get_configuration(SNR−6)` from a low config to ~CONFIG_13, which — **only if
`is_ofdm_config(anchor)` is already true** — licenses the bounded multi-rung jump (the goal).
For 4/5 it improves a fallback pick. For 6 it improves a dead branch. 7/8/10 unaffected. The
DANGER consumer is 1/2/3 at a ROBUST anchor — covered in §8.3.

### §8.3 SAFETY — the §15 WGN:-10 over-climb must NOT reopen (answer to task #3)

**VERDICT: a correctly-scoped fix is SAFE — the §15 guard is anchor-gated, not value-gated,
so raising the CONFIG_0 SNR value cannot reopen the over-climb PROVIDED the fix never makes
the RSP suffix a high SNR while the decoded frame was a robust/MFSK frame.**

Mechanism, from code:
- The §15 over-climb is blocked by `is_ofdm_config(anchor)` in `high_confidence_jump`
  (`arq.h:842-844`), where `anchor == last_data_viable_config`. At WGN:-10 the anchor stays
  ROBUST (OFDM data never DELIVERS, so §16's `data_anchor_raise_target` `arq.h:740-741` REFUSES
  the ROBUST→OFDM anchor crossing). With `is_ofdm_config(anchor)==false`,
  `high_confidence_jump` is FALSE **regardless of the SNR value** — the af14a9e +1 clamp
  re-applies (`arq.h:862-864`). **A higher SNR value cannot bypass this; the gate ignores the
  value once the anchor is robust.** (Verified by JJ1, `arq_commander.cc:7779-7809`: even with
  `snr_uplink_from_suffix(1.0)`→CONFIG_4-ideal, a ROBUST_2 anchor clamps to CONFIG_0.)
- **Is the fix inert at deep SNR?** At ROBUST_0/1/2 the RSP decodes the SET_CONFIG as MFSK
  (`telecom_system.cc:2730` ⇒ `received_message_stats.SNR = 0.0`). A correctly-scoped fix
  substitutes the OFDM SNR ONLY when the last decode was genuinely OFDM at an OFDM config — at
  ROBUST that condition is FALSE, so the suffix is UNCHANGED (still 0.0→1.0). **The fix must be
  gated `M != MOD_MFSK && message_decoded==YES` at the RSP suffix-build (or, equivalently,
  `is_ofdm_config(current_configuration)` at the moment of the decode being relayed).** With
  that gate, §15's behavior at the cliff is byte-identical.
- **Can the fix EVER make the suffix report a high SNR while the anchor is still robust?**
  Only if the RSP decodes a genuine OFDM frame (real SNR ~15) at a moment when the CMD's
  `last_data_viable_config` is still robust. That window EXISTS (the first OFDM batch at
  CONFIG_0 decodes, raising the RSP-relayed SNR, before the CMD's anchor has ratcheted to
  CONFIG_0). **But that high SNR alone cannot over-climb**: the elevator STILL requires
  `is_ofdm_config(anchor)` (anchor = CMD's `last_data_viable_config`), which is still robust
  until §16's `data_anchor_raise_target` credits a CONFIG_0 OFDM delivery. So a high suffix +
  robust anchor ⇒ `high_confidence_jump=false` ⇒ +1 clamp ⇒ NO jump. The §16 anchor-tier gate
  is the backstop that makes the value-raise safe. **The fix does NOT touch
  `is_ofdm_config(anchor)`, RETRIGGER_MAX_LEAP, `supershift_proven_ceiling`, or the §16 ROOT-2
  ladder clamp — all four high-SNR bounds remain.** (RETRIGGER_MAX_LEAP=13 + proven-ceiling
  still bound the CONFIG_0→CONFIG_13 jump; `arq.h:854-856`, JJ2 `arq_commander.cc:7816-7846`.)
- **Distinction from §15**: §15 SUPPRESSED climbing on the under-reporting 1.0 by gating on the
  anchor. The proposed fix CORRECTS the value (~15) at OFDM configs so the elevator works as
  designed AFTER the anchor reaches OFDM. The two are complementary and BOTH gate on the same
  `is_ofdm_config(anchor)`; the value-correction is inert until the anchor is OFDM.

### §8.4 Minimal implementation plan (answer to task #4) — and why it is NOT a one-line swap

**The literal "swap the suffix source" (RSP suffixes `received_message_stats.SNR` instead of
`measurements.SNR_uplink`) does NOT solve the throttle**, for the reason in §8.1: at the
ROBUST_2→CONFIG_0 promotion, the RSP's most recent decode is the SET_CONFIG **on ROBUST_2
(MFSK)**, so `received_message_stats.SNR == 0.0` there too — there is NO OFDM SNR at that
instant. Both the current source (`measurements.SNR_uplink`) and the proposed source
(`received_message_stats.SNR`) equal `0.0`→`1.0` at the boundary. The estimator is identical
because the FRAME is the same MFSK SET_CONFIG. Swapping the source is a no-op at the exact
boundary the task wants to fix.

**What is actually required** (smallest CORRECT change; touches ≥2 layers ⇒ this audit + a
plan + approval per CLAUDE.md §4 before any code):

The OFDM data-frame SNR the RSP measures at CONFIG_0 must reach the CMD's `SNR_uplink` on a
path that is (a) NOT the under-reporting MFSK control decode, and (b) is *gated OFDM* so §15
stays shut. Two candidate shapes, both larger than a suffix-source swap:

- **Plan A — relay the last OFDM data-frame SNR via the SET_CONFIG-ACK suffix.** Add a RSP
  member `last_ofdm_data_snr` (init `-99.9`) written at the canonical producer
  `arq_common.cc:6087` ONLY when `telecom_system->M != MOD_MFSK` (i.e. an OFDM data decode).
  At the RSP suffix SEND (`arq_responder.cc:1122-1130`), when the SESSION has a valid OFDM
  data SNR (`last_ofdm_data_snr > -90`) suffix THAT instead of `measurements.SNR_uplink`
  (which is the MFSK SET_CONFIG's 0.0). Consistency across CONFIG_0..16: the suffix carries the
  most recent OFDM data SNR uniformly at every config; at ROBUST (no OFDM data decoded yet)
  `last_ofdm_data_snr` is still `-99.9` ⇒ falls back to the current behavior ⇒ §15 inert.
  - **Layer reach**: PHY/decode (write `last_ofdm_data_snr` at the OFDM-only producer) → ARQ
    (RSP suffix send) → ARQ (CMD §1.5 decode, unchanged) → elevator. The CMD side
    (`arq_common.cc:5576`) is UNCHANGED; only the VALUE it receives changes.
  - **Quantization caveat**: the WB suffix codec saturates at +25 dB (`mfsk.cc:597`,
    `(SNR+5)/2`, M=16 ⇒ max tone 15 ⇒ `tone_to_snr(15)=25`). A ~15 dB OFDM SNR encodes cleanly
    (tone 10). No new range work needed for the CONFIG_0..13 climb.
  - **Timing caveat (the real subtlety)**: at the FIRST ROBUST_2→CONFIG_0 promotion the RSP
    has not yet decoded ANY OFDM data (the promotion ACK precedes the first CONFIG_0 batch),
    so `last_ofdm_data_snr` is still `-99.9` and the elevator does NOT fire on the very first
    CONFIG_0 entry. It fires on the NEXT SET_CONFIG-ACK after the first clean CONFIG_0 batch —
    which is ALSO exactly when §16 has credited the OFDM anchor, so `is_ofdm_config(anchor)`
    is true and the jump is licensed. This is the CORRECT ordering (jump only after OFDM is
    proven), and it means the win is "one rung then a big jump," not "instant jump from
    ROBUST." Whether that satisfies the throughput goal is a HARDWARE question (§8.6).

- **Plan B — re-enable the data-ACK SNR suffix, OFDM-gated.** Reverse the §18 closure narrowly:
  let the RSP DATA-ACK carry the OFDM SNR suffix and the CMD decode it on the data-ACK wait.
  **REJECTED for now**: §18 / Part N shows this is exactly the path that leaked a bogus
  `SNR_uplink` and required `clear_snr_arm_for_data_ack_wait()`. Re-opening it re-introduces the
  SACK-vs-SNR suffix collision risk on the data-ACK (the §7.1 crux). Plan A keeps the SNR on the
  CONTROL ACK (where the SACK suffix never rides) and is strictly safer.

**Side effects of the higher CONFIG_0 value on OTHER consumers**: §8.2 consumers 4/5 (turbo
fallbacks) would size correctly instead of blindly — desirable. Consumer 6 (SNR_BASED) is dead.
No consumer is harmed by a TRUE ~15 at CONFIG_0; the only consumer that could be harmed by a
SPURIOUS high value (the elevator) is protected by the unchanged `is_ofdm_config(anchor)` gate
(§8.3). **NO threshold/margin is touched (CLAUDE.md §2).**

### §8.5 If the fix is more invasive than "swap the suffix source" — IT IS (explicit, task #4)

Stated plainly per the task's instruction: **the fix is NOT "swap the suffix source." The RSP
has no clean OFDM SNR at the ROBUST→CONFIG_0 suffix point** (the SET_CONFIG it just decoded was
MFSK, SNR=0.0). The minimal CORRECT fix (Plan A) requires: (1) a NEW RSP member
`last_ofdm_data_snr`; (2) a write at the OFDM-only branch of the canonical producer
(`arq_common.cc:6087`, gated `M != MOD_MFSK`); (3) a change to the RSP suffix SEND value
selection (`arq_responder.cc:1122-1130`) to prefer `last_ofdm_data_snr` when valid; (4) a
reset of the new member on session teardown (mirror §1.6). The CMD §1.5 producer and the
elevator are UNCHANGED. This is a cross-layer change (PHY-decode → ARQ-send → ARQ-decode →
elevator) and per CLAUDE.md §4 needs the plan approved before code.

### §8.6 Regression-test design (answer to task #5; in-process, the Part G/J''/N idiom)

Add to `--test-climb-engine` (`arq_commander.cc::test_climb_engine`), replaying REAL helpers
(`snr_uplink_from_suffix`, `supershift_retrigger_target`, `is_ofdm_config`,
`config_ladder_*`) and the REAL `:4722` gate predicate. Tests assert at MEMBER granularity
(the Part L/N idiom) so they exercise the real selection logic, not a paraphrase.

- **(a) FIX-WORKS — elevator jumps from CONFIG_0 on a TRUE OFDM SNR.** Setup: OFDM anchor
  (`last_data_viable_config = CONFIG_0`, `is_ofdm_config==true`), `current_configuration =
  CONFIG_0`, `turboshift_phase = TURBO_DONE`, `gear_shift_on = YES`, optimizer off,
  `supershift_proven_ceiling = -1`. Drive `measurements.SNR_uplink =
  snr_uplink_from_suffix(15.0f)` (the relayed OFDM value). ASSERT: the `:4722` eligibility
  predicate is TRUE; `elevator_target_from_snr()` returns `min(get_configuration(15−6=9),
  CONFIG_0+RETRIGGER_MAX_LEAP)` = CONFIG_13 (idx 16); `gap = idx(CONFIG_13) − idx(CONFIG_0) =
  13 ≥ SUPERSHIFT_RETRIGGER_CONFIGS` ⇒ the re-trigger WOULD fire (multi-rung jump).
  FAIL-BEFORE proxy: with `snr_uplink_from_suffix(1.0f)` (today's CONFIG_0 value) the target
  is the +1-clamped CONFIG_0/CONFIG_1 (gap<3) ⇒ no jump — proving the throttle and that the
  value is the lever. (This mirrors JJ2's PASS arm but asserts the FULL re-trigger fire
  condition, not just the helper return.)
- **(b) OVER-CLIMB STAYS SHUT — a high suffix at a ROBUST anchor does NOT jump.** Setup:
  ROBUST anchor (`last_data_viable_config = ROBUST_2`, `is_ofdm_config==false`),
  `current_configuration = CONFIG_0`, same flags. Drive `measurements.SNR_uplink =
  snr_uplink_from_suffix(15.0f)` (model the WINDOW where an OFDM frame relayed a high SNR but
  the anchor is still robust — the §8.3 hazard). ASSERT: `supershift_retrigger_target(...)`
  returns `anchor_cap` (the +1 clamp) because `is_ofdm_config(ROBUST_2)==false` ⇒
  `high_confidence_jump==false` (`arq.h:842-844`); the elevator target ≤ +1 ⇒ NO multi-rung
  jump. This is the SAME assertion as JJ1c but driven with a HIGH (not 1.0) SNR, proving the
  guard is value-INDEPENDENT and the value-correction fix cannot reopen §15.
- **(c) SUFFIX-SOURCE INERT AT ROBUST/MFSK — no spurious high SNR is ever produced.** Drive
  the Plan-A SEND-side selection helper (to be added, pure): with a robust/MFSK last decode
  (`last_ofdm_data_snr = -99.9`, `received_message_stats.SNR = 0.0`) ASSERT the suffix value
  selected == the MFSK value (0.0/round-trip 1.0), NOT a high SNR; with a valid OFDM last
  decode (`last_ofdm_data_snr = 15.0`) ASSERT the suffix value == 15.0. This pins the §8.3
  "fix only substitutes when the last decode was genuinely OFDM" invariant at the producer.
- **(d) SACK PRESERVATION (regression guard).** Re-assert (Part N idiom) that
  `clear_snr_arm_for_data_ack_wait()` still forces `turbo_snr_ack_enabled=false` on every
  data-ACK wait, so Plan A (control-ACK only) does NOT route any suffix to the SNR decoder on
  the data path. (Plan A adds no data-ACK suffix; this guards against drift if someone later
  reaches for Plan B.)

**Honest scope**: (a)-(d) prove the ELEVATOR LEVER, the ANTI-OVER-CLIMB, the PRODUCER-INERTNESS,
and SACK preservation in-process. They do NOT prove the throughput win — that is hardware-only
(WGN:30 IONOS: does the CMD reach ~CONFIG_13 via the elevator after the first clean CONFIG_0
batch, and does total bps rise toward the ~3k goal?), and they do NOT prove the §8.4 timing
caveat is acceptable (the "one rung then jump" ordering). The parent must wire-test on IONOS.

### §8.7 Related fact documents

- `gearshift-climb-engine.md` §15 (the over-climb guard this fix must not reopen — anchor-gated,
  `arq.h:842-844`), §16 (the anchor-tier gate that makes the value-raise safe,
  `arq.h:740-741`), §18 / Part N (the data-ACK SNR-leak bug — why Plan B is rejected).
- This doc §1.5/§1.7/§7 (the existing CMD-side §1.5 producer + §1.7 enablement the fix builds
  on; the fix changes only the VALUE that arrives at `arq_common.cc:5576`, not the site).
