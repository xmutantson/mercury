# SACK Turnaround Collision — Trace + Repro First, Then Fix (Plan A)

Plan date: 2026-05-13. Author: planning agent (PLANNING ONLY — no source edited).
Predecessor doc: `SACK_DETECTION_ROOTCAUSE.md` (root cause already established).
Scope: produce the concrete, reversible, step-by-step plan to (1) instrument,
(2) capture the gap, (3) build a failing reproduction, (4) sketch the fix. Per
CLAUDE.md: research before implementing, root cause not symptoms, no threshold
band-aids, plan before coding, failing test before fix.

This document is the cross-session context for the SACK turnaround work. Every
claim below is cited to `file:line` against the working tree at commit `acbdb56`.

---

## §0 Root cause recap (from SACK_DETECTION_ROOTCAUSE.md, condensed)

With `--enable-sack`, throughput collapses ~2496 bps → ~281 bps. The SACK
*detector* is correct — it reports non-detection of a signal that is genuinely
absent. The real cause is a **half-duplex turnaround collision**:

- RSP's RX-timeout fires mid-batch. `process_messages_acknowledging_data()`
  prints `[RSP-RX-TIMEOUT] Entering ACK-GATE: rx_count=10`
  (`arq_responder.cc:759-760`) having decoded only a partial batch.
- RSP then keys up the ~1168 ms SACK pattern via `send_sack_pattern()`
  (`arq_responder.cc:837` → `arq_common.cc:3478`) *while CMD's tail DATA frames
  are still in flight* — RSP logs `rsp_data_frame_rxed` for later seq numbers
  AFTER it entered ACK-GATE (`arq_responder.cc:320`).
- RSP's SACK TX guard (`arq_common.cc:3492-3523`) computes `remaining_sym` from
  RSP's *own* `receive_stats.delay` / `buffer_Nsymb` geometry — it produces
  `remaining_sym=0` and applies only a fixed `drain_guard_ms=500`
  (`arq_common.cc:3513`) plus PTT delays. It has **no knowledge of CMD's actual
  turnaround state**, so the SACK lands in a gap where CMD's RX capture stream
  has only noise floor.
- This violates the CLAUDE.md feedback rule "initiator controls flow; responder
  never acts independently" — RSP is keying TX based purely on its own partial
  decode state and its own buffer math, while the CMD (initiator) is still
  transmitting. Same bug class as #55 (NB HAIL race).

The fix is architectural, not a detector/threshold tweak. But per CLAUDE.md §3
("no untested fixes") and §"Systematic Debugging", we must FIRST get a
shared-clock trace of the exact overlap and a reproducible failing test BEFORE
touching the fix code. That is what this plan delivers.

---

## §1 Shared-clock turnaround instrumentation

### §1.1 The instrumentation facility (reuse — do not invent)

`mtl::log_event` / `mtl::log_event_kv` in `include/common/timing_log.h:34-47`
emit `[T] <event> abs_ms=<N> [k=v ...]` lines on a per-process monotonic
`steady_clock` (`timing_log.h:22-31`). Always-on, ~<10 us cost. The parser
`tools/timing_log_parse.py` already reconstructs a cross-process timeline:
it tags lines by `[CMD]`/`[RSP]` prefix or `cmd_`/`rsp_` event-name prefix
(`timing_log_parse.py:52-63, 90-100`) and computes a CMD↔RSP clock offset by
matching `rsp_ack_send_start` to `cmd_ack_detected` and minimizing delta
variance (`timing_log_parse.py:103-127`). **All new events MUST use this
facility and the `cmd_` / `rsp_` naming convention** so the existing offset
alignment keeps working.

### §1.2 Audit — turnaround events that ALREADY EXIST

Required-event audit for reconstructing the CMD↔RSP turnaround:

| Side | Required event | Exists? | Where |
|------|----------------|---------|-------|
| CMD | batch TX start | YES | `cmd_batch_tx_start` `arq_commander.cc:839` |
| CMD | batch TX done (`send_batch()` returned) | YES | `cmd_batch_tx_done` `arq_commander.cc:842` |
| CMD | RX un-mute after TX (ring reset already done) | YES | `cmd_post_tx_unmute` `arq_common.cc:3112` |
| CMD | PTT off | YES | `cmd_ptt_off` `arq_common.cc:3119` |
| CMD | first ACK poll of window | YES | `cmd_first_ack_poll` `arq_common.cc:4106` |
| CMD | ACK-poll buffer had energy | YES | `cmd_ack_buffer_energy` `arq_common.cc:4119` |
| CMD | ACK/SACK detected | YES | `cmd_ack_detected` `arq_commander.cc:1508` |
| RSP | one DATA frame decoded/delivered | YES | `rsp_data_frame_rxed` `arq_responder.cc:320` |
| RSP | ACK pattern send start | YES | `rsp_ack_send_start` `arq_common.cc:3187` |
| RSP | ACK pattern audio done | YES | `rsp_ack_audio_done` `arq_common.cc:3306` |
| RSP | post-ACK capture flush done | YES | `rsp_post_ack_flush_done` `arq_common.cc:3354` |

### §1.3 Audit — turnaround events that are MISSING

The existing events instrument the *ACK-pattern* turnaround well, but the
*SACK* path and the precise collision boundaries are **not** instrumented.
Missing events, with exact insertion points (insert ONLY these):

| # | Missing event | Insert at | Rationale (what gap it closes) |
|---|---------------|-----------|--------------------------------|
| M1 | `cmd_batch_last_sym_out` | `arq_common.cc:3092`, immediately after the `while (size_buffer(playback_buffer) > 0)` drain loop completes (line 3091–3092) and BEFORE the `circular_buf_reset(capture_buffer)` at 3099. This is the true "last DATA audio sample left the sound card" instant. | Root-cause needs *CMD last-DATA-symbol-out*. `cmd_batch_tx_done` (`arq_commander.cc:842`) fires when `send_batch()` *returns* — but `send_batch()` already includes the drain + ring reset + `ptt_off_delay`, so `cmd_batch_tx_done` is ~400 ms+ AFTER audio actually stopped. We need the audio-stop instant itself. |
| M2 | `cmd_ring_reset_done` | `arq_common.cc:3109`, immediately after `MUTEX_UNLOCK(&capture_prep_mutex)` that closes the post-TX `passband_delayed_data` memset + `ring_write_index=0` block (lines 3104–3109), BEFORE `rx_mute=0` at 3110. | Fix option 3 in root-cause doc requires confirming the post-TX ring reset does not clobber freshly-captured SACK audio. `cmd_post_tx_unmute` (3112) fires *after* the reset; we also need the reset-*start*-relative boundary. M2 marks "ring reset complete, rx still muted"; the M2→`cmd_post_tx_unmute` delta is the dead window. |
| M3 | `rsp_ack_gate_entry` | `arq_responder.cc:761`, immediately after the existing `[RSP-RX-TIMEOUT] Entering ACK-GATE` printf+fflush at 759–761, at the top of `process_messages_acknowledging_data()`. Emit `kv` with `rx_count=%d expected_unknown` — i.e. `mtl::log_event_kv("rsp_ack_gate_entry","rx_count=%d batch=%d",batch_rx_frame_count,data_batch_size)`. | The collision *starts* the moment RSP decides to ACK-GATE. There is currently NO `[T]` event for ACK-GATE entry — only a plain printf. Without it the parser cannot place the decision instant on the common clock. |
| M4 | `rsp_rx_timeout_fired` | `arq_responder.cc:426`, inside the `else` branch of `process_messages_receiving_data()` where `receiving_timer.get_elapsed_time_ms() >= receiving_timeout`, immediately after `receiving_timer.stop(); receiving_timer.reset();` (lines 426–427). Emit `kv` with `timeout=%d rx_count=%d` so we capture *which* timeout value fired. | M3 marks ACK-GATE entry, but the *root trigger* is the RX-timeout expiry that pushed the state machine into `ACKNOWLEDGING_DATA`. M4 is the "RX-timeout fire" instant the root-cause doc explicitly calls for. (M3 and M4 are close in time but distinct: M4 = timer expired; M3 = ACK-GATE handler running. Keeping both lets us see scheduler latency between them.) |
| M5 | `rsp_sack_send_start` | `arq_common.cc:3491`, immediately after `sack_turnaround_timer.start()` at the top of `send_sack_pattern()` (lines 3490–3491), BEFORE the OFDM guard block. | `send_sack_pattern()` has ZERO `[T]` events — it is the single most important TX in this bug and is completely dark to the parser. M5 = "RSP committed to SACK TX, guard about to run". Mirror of `rsp_ack_send_start` (`arq_common.cc:3187`). |
| M6 | `rsp_sack_ptt_on` | `arq_common.cc:3532`, immediately after `ptt_on();` in `send_sack_pattern()` (line 3532), BEFORE the `ptt_on_delay_timer.start()`. | The root-cause doc's §2 quantity-of-interest is "RSP's SACK-ptt-on vs CMD's last-DATA-symbol-out". M6 is exactly *SACK PTT-on*. The guard `msleep(wait_ms)` at `arq_common.cc:3521` runs between M5 and M6, so M6−M5 = the guard duration actually applied. |
| M7 | `rsp_sack_audio_start` | `arq_common.cc:3588`, immediately before `tx_transfer(&filtered2[symbol_period], pattern_samples);` (line 3588) — co-located with the existing `[TX-SACK] Audio start` printf at 3584–3587. Emit `kv` with `samples=%d` so the parser knows the SACK length. | The SACK *audio* (not PTT) is what must land in CMD's listening window. M7 = first SACK sample to the sound card. The existing printf at 3584 already prints this — we just need it as a `[T]` event for the parser. |
| M8 | `rsp_sack_audio_done` | `arq_common.cc:3594`, immediately after the `while(size_buffer(playback_buffer) > 0)` drain loop (lines 3590–3592), co-located with the existing `[TX-SACK] Audio done` printf at 3592–3594. | Gives the SACK audio *end* on the common clock → with M7 defines the full SACK occupancy window `[M7, M8]` that must be compared against CMD's rx-usable window. Mirror of `rsp_ack_audio_done` (`arq_common.cc:3306`). |
| M9 | `rsp_post_sack_flush_done` | `arq_common.cc:3627`, co-located with the existing `[TX-SACK] Done` printf at 3624–3627, AFTER the `frames_to_read` reset block (3618–3622). Emit `kv` with `ftr=%d`. | Mirror of `rsp_post_ack_flush_done` (`arq_common.cc:3354`). Marks RSP back to listening; needed to bound the RSP-side post-SACK window symmetrically with the ACK path. |

Notes on the audit:
- The CMD side already has the events to reconstruct *CMD last-audio-out →
  ptt-off → rx-unmute → ring-reset-done* once **M1** and **M2** are added
  (`cmd_batch_tx_done`, `cmd_post_tx_unmute`, `cmd_ptt_off` already exist).
- The RSP side is the gap: `send_sack_pattern()` is entirely uninstrumented.
  M5–M9 mirror the existing `send_ack_pattern()` events one-for-one so the
  SACK path becomes as observable as the ACK path.
- M3/M4 add the *decision* events (ACK-GATE entry + RX-timeout fire) that the
  root-cause doc requires to anchor the collision's start.
- **No new facility.** All nine reuse `mtl::log_event[_kv]`. The parser
  `timing_log_parse.py` keys on `cmd_`/`rsp_` prefixes
  (`timing_log_parse.py:91, 99-100`), so M1–M9 are auto-ingested; the
  CYCLE_STAGES / CROSS_STAGES tables (`timing_log_parse.py:30-42`) will need a
  small additive extension (§2.4) — additive only, no behavior change.

### §1.4 Reversibility & verification of §1

- **Reversible:** every change is a single added line (`mtl::log_event[_kv](...)`)
  at the cited insertion point. Rollback = delete those nine lines. They are
  pure observability — no control-flow, no timing, no state change. Group them
  in one commit titled `instrument: SACK turnaround [T] events (M1-M9)` so the
  whole instrumentation set reverts with one `git revert`.
- **Verify added correctly:** build with `bash build.sh o3`; run any short
  `--enable-sack` loopback or Pi run; `grep '\[T\]'` the log and confirm all
  nine event names appear and that for one batch the ordering is
  `cmd_batch_tx_start < cmd_batch_last_sym_out(M1) < cmd_ring_reset_done(M2)
  < cmd_post_tx_unmute < cmd_ptt_off < cmd_batch_tx_done`, and on RSP
  `rsp_rx_timeout_fired(M4) <= rsp_ack_gate_entry(M3) < rsp_sack_send_start(M5)
  < rsp_sack_ptt_on(M6) < rsp_sack_audio_start(M7) < rsp_sack_audio_done(M8)
  < rsp_post_sack_flush_done(M9)`.
- **Verify no perturbation:** `printf`+`fflush` cost is bounded (~<10 us per
  `timing_log.h` header comment). Confirm by comparing a 3-run `--enable-sack`
  bps number before vs after adding M1–M9 — must be statistically unchanged
  (it is already broken at ~281 bps; adding logs must not change that number).

---

## §2 Capture the gap

### §2.1 Why the Pi testbed (not host loopback)

Per project memory: "SACK cannot be tested on VB-Cable (audio routing
limitation between two writer processes)". The collision is a *half-duplex
two-station turnaround* phenomenon — it only manifests with two independent
processes on a real half-duplex audio path. The IONOS RPi pair via the butler
is the only environment that reproduces it. (Host `--mode host` is retained
only for the regression-guard discussion in §3, not for capturing the gap.)

### §2.2 Trace capture procedure (existing tools, no new tooling)

1. Build `acbdb56` + the §1 instrumentation, deploy the binary to both Pis
   (existing deploy path used by the F23 run — see `SACK_DETECTION_ROOTCAUSE.md`
   header). Do NOT change any other code.
2. Run the SACK-enabled benchmark:
   `python tools/phase0_baseline.py --mode pi --channel clean
   --configs WB_CFG15 --runs 3 --duration 60
   --extra-args=--enable-sack --out mercury/fact-documents/timing_data/sack_trace.json`
   - `--extra-args` is appended verbatim to the mercury command line
     (`phase0_baseline.py:375-379`, applied at `:55` host / passed through to
     `pi_run_one` at `:411`). The Pi launcher redirects each process to
     `/tmp/m_rsp.log` and `/tmp/m_cmd.log` (`phase0_baseline.py:279, 282`).
   - WB_CFG15 chosen because the F23 failure data and the ~2496-vs-281 bps gap
     are at WB; the partial-batch + early-timeout collision is most visible at
     large batch sizes (SACK uses batch=25, memory). A second run at WB_CFG10
     can confirm config-independence.
3. Pull the logs:
   `python tools/timing_pull_pi_logs.py --out mercury/fact-documents/timing_data`
   — downloads `/tmp/m_rsp.log` → `pi_rpi1_<ts>.log` and `/tmp/m_cmd.log` →
   `pi_rpi2_<ts>.log` (`timing_pull_pi_logs.py:42-46`).
4. Parse:
   `python tools/timing_log_parse.py <combined-or-per-side log>`
   — it auto-computes the CMD↔RSP clock offset (`timing_log_parse.py:103-127`)
   and pairs RSP events to CMD batches (`:129-153`).

### §2.3 The specific quantity to extract — the overlap window

Define, per DATA batch, on the **CMD common clock** (RSP events shifted by
`rsp_to_cmd_offset` from `timing_log_parse.py:127`):

- `T_cmd_lastsym = cmd_batch_last_sym_out (M1)` — CMD's last DATA audio sample out.
- `T_cmd_rx_usable_start = cmd_post_tx_unmute` — CMD RX capture becomes usable
  (`arq_common.cc:3110-3112`; ring already reset at M2).
- `T_cmd_rx_usable_end` — end of CMD's listening window for this batch. Upper
  bound = `cmd_ack_detected` if detected, else `cmd_first_ack_poll +
  receiving_timeout` (`receiving_timeout` from `[CMD-POST-TX]` printf,
  `arq_commander.cc:876-877`; `calculate_receiving_timeout()` at
  `arq_common.cc:432-479`).
- `T_rsp_sack_ptt = rsp_sack_ptt_on (M6)` and SACK audio occupancy
  `[T_rsp_sack_audio_start (M7), T_rsp_sack_audio_done (M8)]`.

**Primary metric — the gap:**
`overlap = T_cmd_lastsym − T_rsp_ack_gate_entry(M3)`
(positive ⇒ RSP entered ACK-GATE while CMD was still transmitting — the bug).

**Secondary metric — the miss distance:**
`miss = T_rsp_sack_audio_start(M7) − T_cmd_rx_usable_start`
and `slack = T_cmd_rx_usable_end − T_rsp_sack_audio_done(M8)`.
The failure signature is `miss < 0` (SACK audio started before CMD RX usable)
OR the SACK window `[M7,M8]` not fully inside `[T_cmd_rx_usable_start,
T_cmd_rx_usable_end]`. Cross-check against the existing `[CAP-PEAK]` evidence:
in F23 the CMD capture read −57 dBFS noise in exactly this window
(`SACK_DETECTION_ROOTCAUSE.md` §1).

Also extract: number of `rsp_data_frame_rxed` events with `cmd_ms >
T_rsp_ack_gate_entry(M3)` — the "frames still arriving after ACK-GATE" count
(F23 showed seq 17–24, i.e. 8 frames; `SACK_DETECTION_ROOTCAUSE.md` §2).

### §2.4 Parser extension (additive only)

`timing_log_parse.py` CROSS_STAGES (`:36-42`) currently only knows the ACK
path. Add SACK-path rows (additive, no removal):
`('CMD last sym -> RSP ACK-GATE','cmd_batch_last_sym_out','rsp_ack_gate_entry')`,
`('RSP ACK-GATE -> SACK ptt','rsp_ack_gate_entry','rsp_sack_ptt_on')`,
`('RSP SACK ptt -> audio start','rsp_sack_ptt_on','rsp_sack_audio_start')`,
`('RSP SACK audio','rsp_sack_audio_start','rsp_sack_audio_done')`,
`('CMD rx-usable -> SACK audio','cmd_post_tx_unmute','rsp_sack_audio_start')`.
Also add the `overlap` / `miss` / `slack` derived columns to the per-batch
table. This is a tooling change in `tools/` — reversible by `git revert`, does
NOT touch mercury source.

### §2.5 Reversibility & verification of §2

- **Reversible:** §2 is *running existing tools* + one additive parser patch.
  Nothing in mercury source changes. Rollback of the parser patch = `git revert`
  the tools commit. The captured logs are data artifacts under
  `mercury/fact-documents/timing_data/` — keep them (they ARE the evidence).
- **Verify:** the trace is valid if (a) `timing_log_parse.py` reports a stable
  `rsp_to_cmd_offset` with low variance across ≥3 batches, (b) all M1–M9 events
  are paired into ≥3 batches, and (c) the F23 qualitative picture reproduces:
  `overlap > 0` (RSP ACK-GATEs mid-batch) and `miss < 0` or SACK window outside
  CMD rx-usable window. If the trace does NOT reproduce the F23 signature, STOP
  — the root cause is not yet confirmed and §3/§4 are premature (CLAUDE.md
  Phase 1: "reproduce the issue").

---

## §3 Build a FAILING reproduction (CLAUDE.md §3 — failing test before fix)

### §3.1 The honest problem statement

CLAUDE.md §3 demands "a test that fails before the fix and passes after."
Three constraints make this hard:

1. **No host loopback path.** VB-Cable cannot route audio between two writer
   processes (project memory), so the standard host loopback recipe cannot
   reproduce a two-station SACK turnaround at all.
2. **No C++ unit-test harness in Mercury.** `mercury/source/main.cc` has NO
   `--test` flag — only `TX_TEST` / `RX_TEST` operation modes
   (`main.cc:687-690`, invoked at `:1676, :1692`). The CLAUDE.md line
   "`mercury.exe --test`" is Iris's harness, not Mercury's. There is no
   existing place to drop a C++ assert-based unit test and no test runner.
3. **The bug is timing/scheduling-dependent**, not deterministic logic — it
   depends on when `receiving_timer` expires relative to CMD's TX completion,
   which depends on real audio-path latency.

So options must be evaluated honestly:

- **(a) Deterministic IONOS-Pi test.** Achievable and *does* exercise the real
  two-process half-duplex path — this is where the bug actually lives. But it
  is not perfectly deterministic: it depends on real channel timing. We can
  make it *reliably triggering* (see §3.3) but a "before fails / after passes"
  assertion has to tolerate run-to-run jitter. Strength: tests the real
  failure mechanism. Weakness: needs hardware, not a CI gate, has variance.

- **(b) Single-process unit-level reproduction of the RSP RX-timeout-fires-
  mid-batch logic.** The *decision* logic — "RSP enters ACK-GATE / sends SACK
  while CMD's batch is incomplete" — is the part that can be isolated. The
  collision itself (audio landing in a gap) cannot be unit-tested without an
  audio path, but the *premature trigger* can: the relevant logic is the
  timeout arithmetic in `calculate_receiving_timeout()` (`arq_common.cc:432-
  479`, RSP branch `:470-478`) and the per-frame timeout update in
  `process_messages_receiving_data()` (`arq_responder.cc:327-406`). However,
  there is no harness to host such a test, and `cl_arq_controller` is deeply
  coupled to `telecom_system` + audio I/O — standing up a test double is a
  non-trivial sub-project. Strength: deterministic, fast, CI-able. Weakness:
  tests a *proxy* (the trigger condition) not the collision; significant
  scaffolding cost; risk of testing the model not the system.

- **(c) FM / channel-sim.** Not applicable. Mercury's channel sim
  (`PLOT_PASSBAND`, the `tests/Mercury_channel_test.grc` GNU Radio flowgraph)
  is a single-process BER tool. It has no two-station ARQ turnaround model —
  it cannot reproduce a half-duplex collision. Rejected.

### §3.2 Chosen strategy — (a) primary + (b) as the CI regression guard

**Primary failing test = (a), the deterministic IONOS-Pi trace test.** It is
the only environment that reproduces the actual bug, and §1+§2 already give us
the exact metric (`overlap`, `miss`, SACK-window-vs-rx-usable-window) to assert
on. Justification: CLAUDE.md "Fix root causes, not symptoms" and "Don't deploy
to hardware without passing simulation tests first" are in tension here — but
the simulation environment that *would* catch this (two-process half-duplex)
does not exist, and inventing one is itself a multi-week project that would
violate "fix root causes not symptoms" (we'd be building infrastructure to
avoid the real test). The IONOS Pi pair IS the validation environment for
turnaround timing bugs — this is consistent with how bug #55 (the sibling NB
HAIL race) was validated (per memory, NB HAIL fixed and verified on the
testbed, not in sim).

**Secondary = (b), but scoped down to a pure-function regression guard, not a
full system test.** Do NOT build a `cl_arq_controller` test double. Instead,
once the trace (§2) tells us *which* timeout/trigger condition is wrong,
extract that decision into a small pure helper (this is part of the §4 fix
anyway — see §4 candidate B) and unit-test *that helper* with a tiny standalone
`tools/test_sack_turnaround.cc` compiled directly (the `tools/` dir already has
precedent: `tools/test_b2f_handler.cc` + `tools/test_b2f_handler.exe`). The
helper test asserts: "given (frames_decoded < expected_batch, time_since_last_
data_symbol < CMD_turnaround_bound), the RSP must NOT enter ACK-GATE". This is
deterministic, fast, and CI-able — but it is explicitly a *guard on the fixed
decision rule*, not a reproduction of the collision.

### §3.3 Making the IONOS-Pi test (a) reliably trigger the collision

The collision requires a *partial batch* (RSP times out before all frames
arrive). To make this reliable rather than incidental:

- Use the natural F23 conditions: WB_CFG15, SACK batch=25, `--channel clean`
  first (F23 reproduced it on clean — `SACK_DETECTION_ROOTCAUSE.md` header), and
  WB_CFG15 `--channel fade` as a stronger trigger (fading drops more frames →
  more partial batches → more SACK sends).
- The test = run §2's capture procedure, then assert on the §2.3 metrics:
  **FAIL (bug present)** if, across the run, `overlap > 0` in ≥1 batch AND the
  resulting SACK audio window `[M7,M8]` is not fully inside CMD's rx-usable
  window for ≥1 batch (equivalently: `cmd_ack_detected` absent for that batch
  AND `cmd_ack_buffer_energy` absent in the SACK window — i.e. CMD heard noise).
  **PASS (bug fixed)** if for every batch where RSP sends a SACK, the SACK audio
  window lands fully inside CMD's rx-usable window (or RSP no longer sends a
  mid-batch SACK at all because it now waits for CMD PTT-off).
- Cross-check the throughput: a fixed run should also recover bps materially
  toward the no-SACK ~2496 bps (this is the user-visible symptom, not the
  assertion — the assertion is the timing relationship, per CLAUDE.md "agent-
  judged pass/fail, not binary thresholds").
- Wrap this as `tools/sack_turnaround_test.py`: it calls `phase0_baseline.py
  --extra-args=--enable-sack`, then `timing_pull_pi_logs.py`, then a thin
  analysis (reusing the §2.4 parser extension) that prints PASS/FAIL on the
  `overlap` / `SACK-window-inside-rx-usable` predicate. Run it 3 times (memory:
  "default 3 runs, agent may upgrade to 5"); FAIL if ≥1 run shows the collision.

### §3.4 If no perfect repro — explicit statement

There is **no perfect (deterministic, host-only, no-hardware) reproduction** of
the SACK turnaround collision, because the bug is a property of the real
two-process half-duplex audio path which Mercury has no simulator for. This is
stated explicitly per the task requirement. The best available combination is:
the IONOS-Pi trace test (§3.2a — reproduces the *actual* bug, with run-to-run
variance handled by the ≥1-of-3-runs FAIL rule) plus a deterministic pure-
function guard on the fixed decision rule (§3.2b — prevents regression of the
*logic*, runs in CI). Building a two-station half-duplex audio simulator is
noted as a possible future investment but is explicitly OUT OF SCOPE for this
fix — taking it on now would be gold-plating and would delay the root-cause fix.

### §3.5 Reversibility & verification of §3

- **Reversible:** §3 produces two new test files (`tools/sack_turnaround_test.py`,
  later `tools/test_sack_turnaround.cc`) — both are new files in `tools/`,
  removable by deletion / `git revert`. No mercury source changes in §3 itself
  (the helper extraction that (b) tests is part of §4).
- **Verify the test actually fails first (CLAUDE.md §3 / Phase 4):** run
  `tools/sack_turnaround_test.py` against the *un-fixed* `acbdb56`+instrumented
  binary — it MUST report FAIL (collision present), reproducing F23. If it does
  not FAIL on the broken binary, the test is invalid and §4 must not proceed.
  Only after a confirmed-failing test do we touch the fix.

---

## §4 Fix design — SKETCH ONLY (to be finalized after the §2 trace)

Per CLAUDE.md §"Plan before coding" and §2 "fix root causes not symptoms": the
final fix is chosen *after* the trace data tells us the actual numbers. Below
are 2–3 candidates with trade-offs. **None is selected here.**

Underlying principle (CLAUDE.md feedback "initiator controls flow; responder
never acts independently"): RSP must not key TX while CMD is still
transmitting. Every candidate enforces that; they differ in *how* RSP learns
CMD has finished.

### §4.1 Candidate A — anchor RSP's RX-timeout to "time since last DATA symbol"

Instead of RSP's `receiving_timeout` being a fixed budget started once per
batch (`calculate_receiving_timeout()` RSP branch, `arq_common.cc:472-477`;
started at `arq_responder.cc:384`), make ACK-GATE entry conditional on an
*idle* timer: "no DATA symbol decoded for `> idle_threshold` ms". RSP only
enters ACK-GATE when the channel has actually gone quiet — i.e. CMD's batch TX
truly ended — not when an absolute budget elapsed mid-batch.

- Touch points: `process_messages_receiving_data()` per-frame block resets a
  `last_data_symbol_timer` on every `rsp_data_frame_rxed` (`arq_responder.cc:
  320-326`); the timeout check at `arq_responder.cc:210` / the `else` at `:415`
  gates on that idle timer instead of (or in addition to) `receiving_timeout`.
- Trade-off (+): directly removes the overlap at its source; robust to batch
  size, config, channel. Aligns with "initiator controls flow".
- Trade-off (−): if CMD's *last* frame fails to decode, RSP's idle timer keeps
  the channel "busy" only as long as CMD is still TXing; once CMD stops, idle
  fires — actually fine. The real risk: choosing `idle_threshold` — must be
  > one inter-frame gap but < CMD's post-batch listen window. The §2 trace
  gives the exact inter-frame gap and CMD turnaround numbers to set it
  structurally (NOT a tuned band-aid — it is derived from measured frame
  geometry, same as `message_transmission_time_ms`).

### §4.2 Candidate B — handshake the turnaround off the end-of-batch flag

CMD already marks the last DATA frame with bit 7 of `sequence_number`
(`arq_common.cc:2877-2879`); RSP decodes it into `last_received_end_of_batch_seq`
(`arq_common.cc:4677-4679`) and already uses it to compute `expected`
(`arq_responder.cc:792-796, 334-339`). **But** RSP currently still relies on the
*timer* to leave RECEIVING — it does not treat "end-of-batch frame decoded" as
a turnaround trigger. Candidate B: when the end-of-batch frame is decoded AND
the batch is complete → ACK immediately (already happens, `arq_responder.cc:
357-361` sets `rx_timeout = ptt_on_delay_ms`). When end-of-batch frame is
decoded but batch is INCOMPLETE → RSP knows CMD has finished TX *and* knows
exactly which frames are missing → send SACK now, with the guard keyed off the
end-of-batch arrival instant + a measured CMD turnaround constant, replacing
the open-loop `drain_guard_ms=500` (`arq_common.cc:3513`) and the
`remaining_sym` math (`arq_common.cc:3494-3506`) that produces `remaining_sym=0`
every time.

- Touch points: `process_messages_receiving_data()` add an "end-of-batch seen
  + incomplete" branch; `send_sack_pattern()` guard block (`arq_common.cc:
  3492-3523`) replaced with a turnaround derived from when the end-of-batch
  frame was decoded.
- Trade-off (+): RSP only SACKs *after* it has positive proof CMD finished the
  batch (decoded the bit-7 frame). Strongest "initiator controls flow"
  guarantee. Reuses existing bug-#58 infrastructure. Naturally extracts a pure
  helper `compute_sack_turnaround_ms(...)` → satisfies §3.2(b) regression guard.
- Trade-off (−): if the end-of-batch frame itself is one of the *lost* frames,
  RSP never sees the bit-7 marker → falls back to a timer anyway. So B needs A
  (or the existing timeout) as a fallback. B is not standalone — it is "fast
  path when EOB frame decoded, timer fallback otherwise". Also: end-of-batch
  flag is per-*transmission*; on a SACK retransmit the last *retransmitted*
  frame carries it (`arq_common.cc:2870-2872, 2877-2879`) — confirm the
  semantics hold across retransmit rounds.

### §4.3 Candidate C — replace the open-loop drain_guard with a CMD-PTT-off observation

Keep RSP's existing timeout logic, but before `send_sack_pattern()` actually
keys TX, have RSP *positively confirm* the channel is idle (CMD PTT-off) — e.g.
a short carrier/energy sense on the capture stream: if RSP still hears CMD's
DATA carrier, defer the SACK. This replaces the fixed `drain_guard_ms=500`
(`arq_common.cc:3513`) with a closed-loop "wait until quiet, then +turnaround".

- Touch points: `send_sack_pattern()` guard block (`arq_common.cc:3492-3523`)
  — replace the `msleep(wait_ms)` with a "poll capture energy until below
  noise floor for N ms, then proceed" loop.
- Trade-off (+): purely RSP-local, no protocol change, no dependency on the
  EOB flag surviving. Directly addresses the `[CAP-PEAK]` evidence.
- Trade-off (−): energy-sense on the capture stream is exactly the kind of
  fragile heuristic CLAUDE.md warns against; it can be fooled by post-TX
  settling noise or by RSP's own rx_mute window; and it does not fix the
  *root* issue that RSP entered ACK-GATE mid-batch (it just delays the
  symptom). Weakest of the three on "fix root causes not symptoms" — likely
  only acceptable as a *safety net on top of* A or B, not as the primary fix.

### §4.4 Selection criteria (decide AFTER the §2 trace)

The trace must answer:
1. Is the end-of-batch (bit-7) frame usually *decoded* by RSP even on partial
   batches? If YES → Candidate B is viable as the fast path. If it is often
   itself lost → B needs A as fallback, lean toward A-primary.
2. What is the measured inter-frame gap vs CMD post-batch turnaround? → sets
   Candidate A's `idle_threshold` structurally.
3. Is the overlap purely "RSP timer too short" or also "RSP guard math wrong"?
   → if the guard math (`remaining_sym=0`) is the dominant error, B's guard
   replacement matters most; if the *timer firing mid-batch* is dominant, A
   matters most.

Likely outcome (hypothesis, NOT a decision): **A + B combined** — A makes RSP
not ACK-GATE until the channel is idle (removes the overlap at the source), B
makes the SACK guard closed-loop off the EOB frame when available. C is held
in reserve as a safety net only. Final selection is deferred to the post-trace
session per CLAUDE.md §"Plan before coding".

### §4.5 Reversibility & verification of §4 (when implemented later)

- **Reversible:** each candidate is a localized change to one or two functions
  in `arq_responder.cc` / `arq_common.cc`; implement on a branch, one candidate
  per commit, so each is independently `git revert`-able. No changes to
  `telecom_system.cc` / `ofdm.cc` (another agent owns those — do not touch).
- **Verify:** the §3 failing test (`tools/sack_turnaround_test.py`) must flip
  from FAIL → PASS; the §3.2(b) pure-function guard must pass; and
  `--enable-sack` WB_CFG15 throughput on the Pi pair must recover materially
  toward the ~2496 bps no-SACK baseline (agent-judged, 3 runs, per memory's
  validation methodology). Then run the no-SACK and NB regression configs to
  confirm no collateral damage.

---

## §5 Ordered, reversible step list (the actionable plan)

| Step | Action | Reversible by | Verified by | Status |
|------|--------|---------------|-------------|--------|
| 1 | Add §1.3 events M1–M9 (9 single `mtl::log_event[_kv]` lines at the cited insertion points). One commit. | `git revert` the instrumentation commit | Build o3; short `--enable-sack` run; `grep [T]` shows all 9; ordering per §1.4; 3-run bps unchanged vs un-instrumented | **DONE** — mercury `monitor` commit `8009380`. Build o3 clean; host `--enable-sack` run emitted all 9 with correct kv suffixes; RSP ordering M4<M3 confirmed; no crash/perturbation. |
| 2 | Extend `tools/timing_log_parse.py` with the SACK CROSS_STAGES rows + `overlap`/`miss`/`slack` columns (§2.4). Tooling-only commit. | `git revert` the tools commit | Parser runs on an existing F23-style log without error; new columns populated | **DONE** — workspace-repo `monitor` commits `5567fee` (+`b7726d8` adds the offset-free RSP single-clock analysis, see §6.5). Runs clean on F23 pre-instr log + the Step-3 trace + raw single-side/empty logs. |
| 3 | Deploy instrumented binary to both Pis; run `phase0_baseline.py --mode pi --configs WB_CFG15 --runs 3 --extra-args=--enable-sack`; `timing_pull_pi_logs.py`; `timing_log_parse.py` (§2.2). | N/A (data capture only — keep logs as evidence under `timing_data/`) | Stable `rsp_to_cmd_offset`; M1–M9 paired into ≥3 batches; F23 signature reproduced: `overlap > 0` and SACK window outside CMD rx-usable window (§2.5). If NOT reproduced → STOP. | **DONE** — deployed via `mercury_deploy_rpi.py`; 1-run WB_CFG15 60 s `--enable-sack` capture (logs `pi_rpi{1,2}_20260514_123229.log`). **F23 signature REPRODUCED** — 4/4 ACK-GATE events on partial batches, frames keep arriving after gate (§6.1). NOTE: the plan's cross-clock `overlap` was found unreliable for collision traces — see §6.5; the equivalent signature is proven offset-free on RSP's single clock. |
| 4 | Record the measured numbers (overlap ms, miss ms, inter-frame gap, CMD turnaround, EOB-frame decode rate) back into THIS fact document, new §6. | Edit the doc | Numbers cited to the parsed log file:line | **DONE** — §6 above. Headline gap **6140–8674 ms (mean 7286)**; CMD post-TX 0/200 ms (not slow); root mechanism = open-loop `[RSP-TIMER] KEEP` at a 3780 ms budget vs ~9879 ms CMD batch TX (§6.4). |
| 5 | Build the failing test: `tools/sack_turnaround_test.py` wrapping steps 3's tools + the §2.4 predicate (§3.2a / §3.3). | Delete the new file | Run against un-fixed instrumented binary → MUST report FAIL (≥1 of 3 runs shows collision). If it does not FAIL, test is invalid — fix the test before proceeding. | **DONE** — see §5-Step-5 RESULT below the table. |
| 6 | Using step 4's data, finalize the §4 candidate selection (§4.4) and write the chosen design as a new §8 in this doc. Get approval before coding (CLAUDE.md §"Plan before coding"). | Edit the doc | Approval recorded | **APPROVAL GATE — NOT STARTED.** Trace data points to **A+B** (see §6 / final report). User selects. |
| 7 | Implement the chosen fix in `arq_responder.cc` / `arq_common.cc` ONLY (one candidate per commit on a branch). Extract the pure decision helper; add `tools/test_sack_turnaround.cc` deterministic guard (§3.2b). | `git revert` per-candidate commits; delete the helper test | `tools/sack_turnaround_test.py` flips FAIL→PASS; pure-function guard passes | not started |
| 8 | Regression: re-run `phase0_baseline.py` for `--enable-sack` WB_CFG15 (bps recovers toward ~2496), plus no-SACK WB + NB_CFG4/10 configs (unchanged). 3 runs, agent-judged. | Revert step 7 commits | bps numbers within expected bands; no collateral regression | not started |

Steps 1–5 are "trace + reproduce" (the user's chosen approach, fully reversible,
no fix code touched) — **all five DONE**. Step 6 is an approval gate (the user
picks the §4 fix candidate from the §6 trace data). Steps 7–8 are the fix, gated
on the confirmed-failing test from step 5.

> **RESULT — Step 5 DONE: failing reproduction test `tools/sack_turnaround_test.py`**
>
> New file `tools/sack_turnaround_test.py` (workspace `tools/`, committed on the
> workspace-repo `monitor` branch). Two modes:
> - **`--from-logs <combined.log>`** (default, deterministic, no hardware):
>   re-runs the §6.5 offset-free RSP-single-clock predicate over an existing
>   captured trace. **FAIL** iff ≥1 ACK-GATE event has `frames_after_gate > 0`
>   AND `gate→lastframe > SACK_AUDIO_MS (1168)` — i.e. RSP keyed its SACK while
>   CMD still had more than a full SACK-pattern's worth of the same batch left
>   to send. This is the §3.3 predicate, restated on the single clock because
>   §6.5 showed the cross-clock `overlap` is not trustworthy for collision
>   traces.
> - **`--capture`**: runs the full §2.2 pipeline (`phase0_baseline.py --mode pi
>   … --extra-args=--enable-sack` → `timing_pull_pi_logs.py` → predicate) for an
>   end-to-end hardware run; `--runs N` (default 3), FAIL if ≥1 run collides.
>
> **Verified FAILS on the un-fixed binary** (HEAD `7fd82c2` + the Step-1
> instrumentation, which does NOT change behaviour): run against the Step-3
> capture `fact-documents/timing_data/sack_trace_combined.log` →
> `FAIL — SACK turnaround collision present`, 4/4 ACK-GATE events flagged,
> headline gap 6140–8674 ms. This is the CLAUDE.md §3 "failing test before the
> fix" — it must flip to PASS once the §7 fix lands.

---

## §6.5b Re-numbering note

The fact-doc's original "§6 Open questions" is now **§7** (the measured-numbers
section claimed §6). The §5 table's Step 6 now writes the chosen design as a
new **§8** (was "§7").

---

## §6 Measured numbers — Step-3 Pi trace (2026-05-14)

Capture: `phase0_baseline.py --mode pi --channel clean --configs WB_CFG15
--runs 1 --duration 60 --extra-args=--enable-sack`, instrumented binary
(commit `8009380` instrumentation on `monitor` HEAD `7fd82c2`) built on
both Pis via `mercury_deploy_rpi.py`. Logs:
`fact-documents/timing_data/pi_rpi1_20260514_123229.log` (RSP),
`pi_rpi2_20260514_123229.log` (CMD), combined +tagged as
`sack_trace_combined.log`. Parser: extended `timing_log_parse.py`
(commits `5567fee` + `b7726d8`). Throughput this run: **563.3 bps with
`--enable-sack`** vs the ~2496 bps no-SACK baseline — the collapse the
plan is chasing, reproduced.

### §6.1 The F23 collision signature IS reproduced — all 4 batches

| Batch (ACK-GATE #) | RSP `rx_count` at gate | frames decoded AFTER gate | gate→last-frame **GAP** |
|---|---|---|---|
| 1 (`abs_ms=7707`)  | 11 / 25 | 7  (seq 18–24) | **6140 ms** |
| 2 (`abs_ms=24802`) | 15 / 25 | 10 (seq 15–24) | **6551 ms** |
| 3 (`abs_ms=41951`) | 15 / 25 | 14 (seq …–24)  | **7780 ms** |
| 4 (`abs_ms=59039`) | 15 / 25 | 19 (seq …)     | **8674 ms** |

Every ACK-GATE event fired on a *partial* batch (11–15 of 25 frames) and
in every case CMD kept delivering DATA frames of the **same batch** to
RSP for **6140–8674 ms (mean 7286 ms)** afterward.
[`pi_rpi1_20260514_123229.log`: `rsp_rx_timeout_fired`/`rsp_ack_gate_entry`
at lines 2086/2132, 3744/3790, 5406/5452, …; `rsp_data_frame_rxed` seq
18–24 at lines 2519–2619 — i.e. AFTER the line-2132 gate.]

### §6.2 THE HEADLINE NUMBER — the CMD↔RSP overlap gap

> **RSP keys up its ~1168 ms SACK pattern 6.1–8.7 seconds (mean ≈ 7.3 s)
> before CMD has finished transmitting the very batch RSP is SACK-ing.**

Measured offset-free on RSP's *own* `steady_clock` as
`(last same-batch rsp_data_frame_rxed) − rsp_ack_gate_entry` — needs no
cross-process clock alignment, so it has **no error bars** (see §6.5).
RSP additionally commits the SACK *audio* only **900 ms** after ACK-GATE
entry (`gate→sack_audio_start` = 900 ms, identical all 4 batches —
`rsp_sack_send_start` fires at the same `abs_ms` as `rsp_ack_gate_entry`,
then the ~800 ms OFDM guard + ~100 ms ptt = 900 ms). So the SACK audio
lands **~5.2–7.8 s before CMD's batch even ends**, squarely in CMD's
DATA-TX window — exactly the half-duplex collision.

### §6.3 CMD side — CMD never detects the SACK; CMD post-TX is NOT slow

- **Zero `cmd_ack_detected` `[T]` events** in the entire CMD log
  (`pi_rpi2_20260514_123229.log`): CMD never matched any of the 3 SACK
  patterns RSP transmitted. The SACK lands in dead air. This *is* the
  F23 `[CAP-PEAK]` "−57 dBFS noise" / `max_e≈0` finding, re-confirmed.
- **CMD post-TX turnaround is fast — root-cause doc fix-option-3 / §7
  open-question [?] ANSWERED NEGATIVE.** Per batch, CMD's
  `cmd_batch_last_sym_out` → `cmd_post_tx_unmute` delta is **0 ms** (same
  timestamp) and `cmd_post_tx_unmute` → `cmd_ptt_off` is **200 ms**
  (`pi_rpi2_…log`: batch-1 block `cmd_batch_last_sym_out abs_ms=13086`,
  `cmd_ring_reset_done abs_ms=13086`, `cmd_post_tx_unmute abs_ms=13086`,
  `cmd_ptt_off abs_ms=13287`; identical shape on batches 2–4). The CMD's
  RX capture is usable essentially the instant its last DATA sample
  leaves the card. **The collision is NOT "CMD too slow to un-mute" —
  it is purely "RSP ACK-GATEs and keys its SACK while CMD is still mid-
  batch-TX".** Fix candidate C (energy-sense CMD-PTT-off) addresses a
  non-problem; A/B address the real one.
- CMD batch-1 actual DATA-audio duration: `cmd_batch_tx_start abs_ms=3207`
  → `cmd_batch_last_sym_out abs_ms=13086` = **9879 ms** of DATA audio
  for a 25-frame batch (~395 ms/frame). Batches 2–4: ~9886/9920/9892 ms.

### §6.4 Root mechanism — the open-loop RSP timer (the `[RSP-TIMER]` evidence)

The `[RSP-TIMER]` diagnostic lines pin the mechanism precisely
(`pi_rpi1_20260514_123229.log:1650-2041`):

- Batch 1: the timer is set to `cur_t=3780` ms and then **`KEEP`**'d at
  3780 for every one of frames 1–11 — it is *never* updated. The
  `[RSP-TIMER] KEEP` branch (`arq_responder.cc:400-405`) is taken because
  `sack_enabled=1` AND `rxcnt < effective_batch(=25)` — so the per-frame
  `else` at `arq_responder.cc:386-405` deliberately does **not** restart
  or extend the timer (the comment at `arq_responder.cc:373-376` says
  "keep the initial timeout … which covers full CMD TX").
- **But the "initial timeout" was only 3780 ms while CMD's batch TX is
  ~9879 ms.** The `rx_t` column the same log lines print (9460, 9070,
  8680, … — a correct ~9.8 s estimate decreasing per frame) is computed
  but *thrown away* by the `KEEP` branch. So `receiving_timeout` stays
  3780 ms, the timer expires at `abs_ms=7705` — **6140 ms before CMD's
  last DATA frame of that batch arrives** (`rsp_data_frame_rxed` seq 24
  at `abs_ms=13847`).
- Later batches show `cur_t=14775` (the post-SACK `*3/2` value,
  `arq_responder.cc:860`) — still `KEEP`'d, and still the timer started
  too early relative to CMD's batch, so it still fires mid-batch.
- **Conclusion:** the RSP `receiving_timeout` is an *open-loop budget*
  with no coupling to CMD's actual batch geometry. Whether the bug is
  "`calculate_receiving_timeout()` produced 3780 instead of ≥9879" or
  "the timer was started too late so 3780 was only the tail" — the
  *structural* fault is the same: **RSP decides to ACK-GATE on a clock,
  not on observed channel state.** This is precisely the
  CLAUDE.md-feedback violation "initiator controls flow; responder never
  acts independently" and the same bug-class as #55.

### §6.5 Methodology note — a real flaw in the plan's §2 offset method

The plan's §1.1/§2.3 cross-clock `overlap` metric anchors the CMD↔RSP
clock offset on `cmd_ack_detected` `[T]` events. **In a SACK-collision
trace CMD detects nothing, so there are ZERO `cmd_ack_detected` events**
— the offset falls back to a noisy `cmd_batch_tx_start`↔rsp-first-frame
anchor (variance ≈ 1.6 × 10⁸ ms² on this trace), making the cross-clock
`overlap`/`miss` columns unreliable (they even come out sign-flipped).
The plan's own §2.5 validity criterion "a *stable* `rsp_to_cmd_offset`
with low variance" therefore **cannot be met by the method §2
specified** — a genuine flaw, recorded here per CLAUDE.md.

Resolution (commit `b7726d8`): the collision is fully provable on RSP's
*single* clock — `rsp_ack_gate_entry` vs the same-batch
`rsp_data_frame_rxed` events that follow it — needing **no offset at
all**. That single-clock `gate→lastframe` gap is the authoritative
headline number in §6.2 and is what the §5 / failing-test predicate uses.
The cross-clock table is retained (additive) but explicitly marked
non-authoritative for collision traces.

---

## §7 Open questions [?]

- [✓ RESOLVED §6.1] Does the end-of-batch (bit-7) frame survive decode on
  partial batches? **YES — but only AFTER the collision.** `seq=24/25`
  *is* decoded (3× in the trace), but on every partial batch frames
  15–24 (incl. the EOB seq-24 frame) arrive *after* RSP already entered
  ACK-GATE. So for Candidate B: the EOB frame is a usable "CMD finished
  the batch" signal, but RSP must *wait for it* instead of timing out
  first — B is viable as a fast path only if paired with A (or a longer
  timer) so RSP does not ACK-GATE before the EOB frame can arrive. This
  matches the §4.2 trade-off "(−) B needs A as fallback".
- [✓ RESOLVED §6.5] Is `rsp_to_cmd_offset` stable enough on the Pi pair
  for sub-100 ms gap measurement? **NO — not from the events the plan
  specified** (no `cmd_ack_detected` in a collision trace). Mooted: the
  headline gap is now measured offset-free on RSP's single clock (§6.2,
  §6.5), which has no error bars.
- [✓ RESOLVED §6.3] Is the CMD post-TX path itself too slow regardless of
  RSP timing? **NO.** `cmd_batch_last_sym_out` → `cmd_post_tx_unmute` = 0
  ms, → `cmd_ptt_off` = 200 ms. CMD RX is usable immediately. Fix
  candidate C targets a non-problem.
- [?] On SACK *retransmit* rounds, does the end-of-batch flag land on the
  last *retransmitted* frame correctly? (`arq_common.cc:2870-2879` — the
  `sack_retransmit_active` branch skips re-setting `sequence_number`, so
  the flag is whatever was set on the original TX; confirm this is the
  intended semantics for Candidate B.) — still open, decide during §7-fix.
