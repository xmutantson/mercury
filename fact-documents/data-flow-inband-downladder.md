# Data-Flow Audit — In-Band Down-Ladder Delivery (decode-but-no-deliver)

Branch: `fix/inband-downladder-delivery` (off `feat/inband-rate-adapt` @ 1b1ee01).
Scope: the `MERCURY_INBAND_RATE=1` (opt-in) RX down-ladder path. The default-off
legacy path is byte-identical (every production change below is entry-gated on
`inband_rate_feature_enabled()` or added inside an already-feature-gated block).

This document owns the producer/consumer list for the shared batch state the fix
touches: `messages_rx_prev[]`, `rsp_current_expected_batch_seq_id`, the
`rsp_prev_batch_*` reshrink state, and `data_batch_size`.

---

## §0. The bug (HW A/B + saved-log confirmed)

Under `MERCURY_INBAND_RATE=1` on a demoting channel, the RX decoded frames on the
PRIMARY path (`rsp_ofdm_ok=56`, `[RSP-V2-ADOPT]`, `[RX-DATA]` seq to 17/25) yet
delivered **0 BYTES** — a false-BREAK batch-reshrink storm. Legacy OFF (no
down-ladder) delivered 98999 B over the same channel. THREE coupled defects:

1. **Snapshot extraction (root, §1).** `inband_try_down_ladder_on_decode_fail`
   read `passband_delayed_data[rwi]` FORWARD from the ring write head — the region
   the capture thread is about to overwrite, NOT the frame body. Every trial decode
   got mostly zero-pad: `[PRESCAN] no signal found, max_peak=0.000000` → 0/174
   decodes during live CFG15 traffic. A weak head-of-ring energy transient
   (`pk=0.086 > 0.05`) passed the gate while the decode buffer was empty
   (gate-region ≠ decode-region).
2. **Too-eager firing (§2).** The entry gate (`messages_rx_buffer.status !=
   RECEIVED`) fired on EVERY benign inter-frame `receive()` pass that stored no
   frame, accumulating false "total-loss" ticks → TERMINAL BREAK.
3. **BREAK orphans the in-flight batch (cross-layer, §3).** TERMINAL-BREAK →
   `load_configuration(ROBUST_0)` reseeds `data_batch_size` (→ `set_data_batch_size`
   → `rescan_prev_on_batch_shrink`) → `[RSP-V2-PREV-RESHRINK] 25->6
   orphaned_received=13` → prev incomplete → `[RSP-V2-PREV-STALE] discarding` → 0
   in-order deliveries → 0 bytes.

---

## §1. FIX #1 — snapshot extraction + per-decoder prescan
`arq_common.cc` `inband_try_down_ladder_on_decode_fail()` and
`inband_down_ladder_resync()`.

The PRIMARY decode path copies the head-aligned frame body into the STAGED member
buffer and decodes from THERE:
```
arq_common.cc:10434-10435   rwi = ring_write_index;
                            memcpy(ready_to_process_passband_delayed_data,
                                   &passband_delayed_data[rwi], signal_period);
```
The PRE-FRAME tag-follow and parallel-monitor both read the SAME staged buffer:
```
arq_common.cc:10652-10654   inband_detect_follow_from_snapshot(
                                ready_to_process_passband_delayed_data, signal_period, …)
arq_common.cc:10668         parallel_monitor_decode(
                                ready_to_process_passband_delayed_data, signal_period, …)
```
**FIX:** the down-ladder now snapshots `ready_to_process_passband_delayed_data`
over the PRIMARY `signal_period` (= `Nofdm*buffer_Nsymb*interpolation_rate`), NOT
`passband_delayed_data[rwi]`. The window-level energy gate is computed over the
SAME snapshot (gate-region ≡ decode-region). Plus a PER-DECODER prescan in the
resync loop: a silent per-decoder copy is skipped WITHOUT incrementing
`inband_down_decode_attempts`, and the caller treats `attempts==0` (all silent) as
a no-signal pass that does NOT NACK and does NOT tick the dead-batch streak.
`MERCURY_INBAND_DOWN_DEFEAT_SNAPFIX=1` restores the old wrong-buffer read (A/B).

The prior 1b1ee01 attempt grew the snapshot LENGTH but kept the wrong source
buffer; that is why it never recovered. The new code reads the buffer the primary
already validated (it acquired + demodulated frame 0 from it this pass).

---

## §2. FIX #2 — firing condition
`arq_responder.cc:529-536` (the `inband_try_down_ladder_on_decode_fail()` call gate).

Added term: `&& rsp_current_expected_batch_seq_id >= 0` (an IN-FLIGHT active batch,
i.e. `bsi_lsb != 255`). A lost CONFIG_TAG can only happen WITHIN a batch we are
tracking; a no-active-batch / pre-adopt / idle pass is not a lost tag and must
never tick the streak. The term is added inside the block that already requires
`inband_rate_feature_enabled()`, so the legacy path is untouched.

### §2.1 FIX #2b — the FRESH-WINDOW firing term (the HW ~8KB throughput cap)
`arq_responder.cc:535-542` (same call gate) + `arq_common.cc receive()`.

**The bug (HW-VERIFIED, `ib_on_c4_rsp.log`):** the redesign capped at ~8KB because
the RX down-ladder fired **3127×** logging `no trial decoder saw signal (all silent)`
(`arq_common.cc:~4244`) — burning the ~500 Hz receive loop on benign inter-frame passes
instead of decoding forward DATA. ROOT CAUSE (trace, 2026-06-21): the call gate's
`messages_rx_buffer.status != RECEIVED` term is TRUE on **every** no-frame pass during an
active batch (status only goes RECEIVED on a decode, `arq_common.cc:11420`); the ARQ loop
runs `receive()` blocking-paced; and `receive()` only re-stages a fresh capture window +
attempts a decode on the `frames_to_read==0` branch (`arq_common.cc:10624`), taking the
`frames_to_read!=0` **early-exit** (`arq_common.cc:12158`) on inter-frame passes WITHOUT
re-staging. So the staged buffer (`ready_to_process_passband_delayed_data`, the snapshot
source) holds the LAST decoded frame — **stale-but-loud** (energy ≥ 0.05) — and the §1
window energy gate PASSES on stale energy, running the bank every benign pass.
`frames_to_read` is NOT a usable gate: a genuine fresh-window decode-FAIL re-arms it
non-zero (anti-spin, `arq_common.cc:11767/12055`) **before** the down-ladder runs.

**FIX:** new one-shot member `rx_fresh_window_decoded_this_pass`
(`include/datalink_layer/arq.h`). `receive()` clears it at the top of every pass and sets
it TRUE only inside the `frames_to_read==0` staging+decode branch (`arq_common.cc`). The
call gate adds `&& (rx_fresh_window_decoded_this_pass || inband_freshwin_gate_defeat())`.
A STALE inter-frame pass (no fresh window) is now a cheap no-op: the down-ladder is not
even entered (no bank, no log, no streak tick). A GENUINE lost-tag frame DOES stage a
fresh window (`frames_to_read==0` → flag TRUE) then FAILs to decode, so the resync still
fires — this term **cannot** suppress a signal-present loss. ALWAYS-maintained but read
ONLY inside the `inband_rate_feature_enabled()` block (C++ short-circuit) ⇒ legacy
byte-identical. `MERCURY_INBAND_FRESHWIN_DEFEAT=1` (`inband_freshwin_gate_defeat()`,
`arq_common.cc`) restores the pre-fix unconditional firing (one-binary A/B fail-before).

**Why this is the right discriminator (not a threshold band-aid):** `frames_to_read==0`
is the modem's OWN "a fresh frame window is ready to process" signal (the same condition
that stages the buffer the primary decodes from). We reuse it, not a magic number, and we
do not touch the energy thresholds. The §1 window + per-decoder energy prescans REMAIN as
the signal-present-but-silent-snapshot backstop; this term removes the upstream churn.

**Regression** (`test_inband_downladder` PART C/C4, in `--test`): C0 asserts every gate
term except the flag is satisfied on a stale pass (the flag is the sole discriminator —
the old gate fired); C1 stale pass → gate FALSE (no fire); C2 fresh-window FAIL → gate
TRUE (resync preserved); C3 the flag alone toggles the gate; C4 fail-before
(`DEFEAT=1`) → the old gate FIRES on a stale pass (reproduces the bug). All PASS.

---

## §3. FIX #3 — decouple BREAK from delivery
New `arq_common.cc deliver_complete_inflight_before_break()`, called from BOTH
BREAK→ROBUST_0 handlers BEFORE the reseed:
- `arq_responder.cc` legacy `break_detected` handler (before the bsi reset + the
  `load_configuration(ROBUST_0)`).
- `arq_responder.cc` inband TERMINAL-BREAK handler (same placement).

The helper delivers a COMPLETE in-flight prev batch (`received >= expected`) via
the EXACT frame-driven prev-deliver discipline — pointer-swap `messages_rx ←
messages_rx_prev`, mark RECEIVED→ACKED, `copy_data_to_buffer()`, restore, FREE
prev, `advance_last_delivered()` — and clears `rsp_prev_batch_active` so the
subsequent ROBUST_0 `rescan_prev_on_batch_shrink` is a clean no-op. It honors the
delivery-time gap gate (`delivery_step_is_gap`): a gapped prev is NOT pushed
(delivering a hole would corrupt the app stream). A genuinely PARTIAL prev (real
hole) is intentionally left undelivered. Feature-gated (returns 0 in legacy) and
`MERCURY_PREBREAK_DELIVER_DEFEAT=1` defeats it (A/B fail-before).

With §1+§2, a decoding channel no longer REACHES TERMINAL BREAK (the streak only
advances on real signal-present primary-decode-fails). §3 is the hardening backstop
so a LEGITIMATE BREAK can never zero already-decoded, deliverable bytes.

---

## §4. Producers / Consumers (CLAUDE.md §5)

### 4.1 `messages_rx_prev[]` (and `.status`)
PRODUCERS:
- `init_messages_buffers()` — all slots FREE.
- `bump_bsi_and_transfer_prev()` (arq_common.cc:~8307+) — transfers current→prev on
  bsi bump; sets RECEIVED/FREE per slot.
- `receive_v2_data_frame()` prev-route (arq_responder.cc:~1000) — writes RECEIVED +
  payload for a prev-bsi frame; bumps `rsp_prev_batch_received_count`.
- frame-driven prev-deliver (arq_responder.cc:~1147-1160) — RECEIVED→ACKED for the
  copy, then FREE after `copy_data_to_buffer`.
- `rescan_prev_on_batch_shrink()` (arq_common.cc:1086) — does NOT free slots; it
  RE-COUNTS within the new bound (orphan accounting). **The orphan site (§3).**
- HINGE re-baseline (`inband_adopt_resynced_config`, arq_common.cc:~3980) — all FREE.
- **NEW** `deliver_complete_inflight_before_break()` — RECEIVED→ACKED, deliver, FREE.
CONSUMERS:
- frame-driven prev-deliver gate (arq_responder.cc:1045-1152) — reads `.status` to
  deliver a complete prev via the pointer swap + `copy_data_to_buffer`.
- `rescan_prev_on_batch_shrink()` (arq_common.cc:1096-1100) — counts RECEIVED in
  `[0,new_batch)` and orphans in `[new_batch,old_batch)`.
- `bigblock_partial_block_crc_ok()` — reads prev `.length`/`.data` for the K-cw image.
- **NEW** `deliver_complete_inflight_before_break()` — reads `.status` to deliver.

### 4.2 `rsp_current_expected_batch_seq_id`
PRODUCERS: `init()` (-1); first-frame adopt (arq_responder.cc:~797);
`bump_bsi_and_transfer_prev()` (+1 mod 256); HINGE re-baseline (-1); BREAK reset
(arq_responder.cc:476/562) (-1); gap-abort teardown (-1).
CONSUMERS: bsi route decision (current vs prev, arq_responder.cc:~748/805); the
down-ladder `bsi_lsb` derivation (arq_common.cc:4044/4064); re-adopt predicate;
**NEW** the FIX #2 firing gate (>= 0 means an in-flight batch); the prev-deliver
gate. Default-init: **-1 (no active batch)** — the state BEFORE any adopt; FIX #2
relies on exactly this (-1 ⇒ down-ladder does not fire).

### 4.3 `rsp_prev_batch_active` / `_seq_id` / `_received_count` / `_expected_count`
PRODUCERS: `init()`/`deinit_messages_buffers()` (false/-1/0/0);
`bump_bsi_and_transfer_prev()` (active=true, counts set); `receive_v2_data_frame()`
prev-route (received_count++, D5 expected re-derive); `rescan_prev_on_batch_shrink()`
(recompute received+expected within the new bound); frame-driven prev-deliver
(active=false, counts=0 after deliver/reject); HINGE re-baseline (active=false);
**NEW** `deliver_complete_inflight_before_break()` (active=false, counts=0 after
deliver).
CONSUMERS: `rescan_prev_on_batch_shrink()` (active gate); the prev-deliver gate
(`active && received >= expected`); `bump_bsi_and_transfer_prev()` stale-detect;
**NEW** the FIX #3 helper (active + received>=expected gate). Default-init:
**active=false** — the FIX #3 helper early-returns, so a BREAK with no in-flight
prev is a no-op.

### 4.4 `data_batch_size`
PRODUCERS: `load_configuration(FULL)` → `set_data_batch_size(default batch)`;
`load_configuration()` robust pin → `set_data_batch_size(1)` (arq_common.cc:2175 —
runs even on `PHYSICAL_LAYER_ONLY`, the BREAK→ROBUST_0 reseed reshrink trigger);
RSP CONFIG_TAG / ROBUST_DWELL_BATCH_OP apply; test direct-assigns.
CONSUMERS: batch-loop bounds; `bump_bsi_and_transfer_prev()` expected default;
`rescan_prev_on_batch_shrink()` clamp; `copy_data_to_buffer()` per-slot iteration
(`i < data_batch_size`); SACK expected-count; timeout scheduling.
**Key interaction:** the ROBUST_0 reseed shrinks `data_batch_size` (e.g. 25→1)
while `rsp_prev_batch_active` is still true ⇒ `rescan_prev_on_batch_shrink` orphans
RECEIVED prev slots. FIX #3 delivers + clears the prev BEFORE this shrink, so the
reshrink sees `active==false` and early-returns (no orphan).

---

## §5. Invariants the fix changes (and the consumers re-checked)

§5.1 INV-SNAP: "the down-ladder trial decoders read the same frame body the primary
read." BEFORE: violated (read the live ring head). AFTER: holds (reads
`ready_to_process_passband_delayed_data`, the staged buffer the primary +
tag-follow + monitor all read). Consumers re-checked: the resync's per-decoder
`memcpy`+zero-pad (arq_common.cc:3860-3865) unchanged; the energy gate now reads
the same snapshot. No legacy consumer affected (gated).

§5.2 INV-STREAK: "`inband_session_dead_batches` advances ONLY on a real
signal-present primary-decode-fail." BEFORE: violated (benign/silent passes ticked
it). AFTER: holds — FIX #2 requires an in-flight batch to fire, and the
window/per-decoder prescan + the `attempts==0` guard suppress the silent tick.
Consumer re-checked: the TERMINAL-BREAK arming (`>= SESSION_DEAD_BATCHES`,
arq_common.cc:4114) — now only reachable from real losses.

§5.3 INV-DELIVER-BEFORE-BREAK (NEW): "a BREAK→ROBUST_0 reseed never orphans a
COMPLETE-but-undelivered prev batch." Producer constrained:
`deliver_complete_inflight_before_break()` runs before the reseed reshrink.
Consumers re-checked for the THREE paths:
- **Legacy path (no down-ladder):** helper is feature-gated → returns 0 → BREAK
  reshrink behaves EXACTLY as before. Byte-identical.
- **Normal config-shrink reshrink (Axis-2 down-move 15→10, robust-dwell 8→1):** NOT
  a BREAK; `deliver_complete_inflight_before_break` is NOT on those paths;
  `rescan_prev_on_batch_shrink` is unchanged — its existing R035 streaming-reset +
  orphan accounting still runs. Unaffected.
- **Genuine-loss BREAK:** the helper only delivers a COMPLETE + contiguous (gap-gate
  passed) prev. A genuinely incomplete prev (real hole) is left for the standard
  stale-discard (correct — a hole can't be delivered). The gap gate
  (`delivery_step_is_gap`) prevents pushing a non-contiguous prev to the app FIFO.

§5.4 INV-GAP (preserved): the prev-deliver gap gate is reused verbatim, so the
high-water (`rsp_last_delivered_batch_seq_id`) advances monotonic-with-wrap; a late
older prev cannot regress it (`advance_last_delivered`).

---

## §6. Regression — `--test-inband-downladder` (+ wired into `--test`)
`arq_responder.cc test_inband_downladder()`, synthetic-fire (no IONOS/RF), in `--test`.
- PART A (defect #3), both arms in one process:
  - FAIL-BEFORE (`MERCURY_PREBREAK_DELIVER_DEFEAT=1`): the production reshrink
    (`set_data_batch_size(1)` → `rescan_prev_on_batch_shrink`) ORPHANS the complete
    25-frame prev (`[RSP-V2-PREV-RESHRINK] 25->1 … orphaned_received=24`) → 0 app
    bytes. **VERIFIED: A2-FAILBEFORE got=0 want=0.**
  - PASS-AFTER: `deliver_complete_inflight_before_break()` flushes all 25 frames
    (400 B) to `fifo_buffer_rx` BEFORE the reshrink → reshrink is a no-op.
    **VERIFIED: A2-PASSAFTER got=400 want=400; A3 prev cleared.**
- PART B (defect #1/#2): a minimal RX with a ZEROED staged buffer drives the
  production `inband_try_down_ladder_on_decode_fail()`; the dead-batch streak stays
  0 and no TERMINAL BREAK is armed. **VERIFIED: B1/B2 PASS.**
Result: `[TEST-INBAND-DOWNLADDER] ALL PASS (failed=0)`; full `--test` EXIT=0.

---

## §7. Open questions / honest limits [?]
- The fix is proven IN-PROCESS (decode→deliver coupling on synthetic ARQ state +
  the production reshrink/deliver primitives). The end-to-end live-channel
  decode-AND-deliver under a real demote is NOT re-validated on the bench here
  (per task scope: off-bench). Ready for an A/B re-validation (env-on vs env-off,
  and the two `*_DEFEAT` arms) — see §6.
- §1 reads the primary `signal_period` of the staged buffer. A down-ladder target
  whose frame is LONGER than the primary window (deep robust rung) is fed a
  head-truncated + zero-padded copy by the resync (arq_common.cc:3860-3865); the
  preamble + head codewords are present (the primary acquired from it), but a very
  deep robust resync from a tiny high-OFDM primary window is the same length
  constraint the prior `inband_seat_robust_ring_floor` seat addresses. For the
  observed defect (CFG15 traffic, OFDM-to-OFDM demote) the staged window is
  sufficient. [?] confirm on the bench whether a multi-rung robust resync needs the
  ring-floor seat in addition to the staged-buffer source.

## §8. FIX #4 — the per-pass PHY-rebuild leak CLASS (ON-arm 0-deliver)
The HW ON arm decoded 0 OFDM frames / delivered 0 bytes where legacy OFF decoded
810 — not because the down-ladder logic was wrong, but because the inband RX path
ran a FULL throwaway-`cl_telecom_system` + `load_configuration()` PHY init **per
receive pass** on the hot capture thread, starving the OFDM decode PHY. A fresh
`cl_telecom_system` has `current_configuration == CONFIG_NONE`, so the
`load_configuration` dedup (telecom_system.cc:~10052) is bypassed → a full reinit
every call. This is a CLASS (the throwaway-tmp + `load_configuration` size-probe
idiom), not a single site:

- **Instance #1 (c7b4aca)**: `inband_robust_floor_buffer_nsymb()`
  (arq_common.cc:3660) via `inband_seat_robust_ring_floor()` (arq_common.cc:3712),
  called EVERY CONNECTED+RECEIVING pass (arq_responder.cc:520). HW: 2793 rebuilds.
  Fixed: memo keyed by `narrowband_enabled` + an idempotent fast-path that returns
  before any probe once the ring is seated. `inband_down_window_buffer_nsymb()`
  (arq_common.cc:3619) got the same memo (keyed by `lo_idx + nb`) — though it has
  no live production caller today (the snapshot is sized from the primary
  `signal_period` at arq_common.cc:4134, not this helper).
- **Instance #2 (this fix)**: `inband_ensure_down_decoders()` (arq_common.cc:3779)
  probed the bank's common `buffer_Nsymb` with a throwaway tmp + `load_configuration`
  EVERY call (arq_common.cc:3787-3799 pre-fix). The bank SLOTS were always reused
  (the `inband_down_decoder_cfg[i] == cfg` `continue`, arq_common.cc:3833), but the
  size PROBE leaked. This runs on EVERY down-ladder fire (a degraded RX pass that
  decode-failed with an active in-flight batch — arq_responder.cc:535-553); v6
  cycle1 ON saw 44 CONFIG-11 PHY rebuilds. Fixed: memo `want_buffer_nsymb` keyed by
  (capped `lo_idx`, `nb`); skip the tmp construction when unchanged.

PRODUCERS of the three memo keys: `current_configuration` (config switch /
down-ladder adopt / BREAK→ROBUST_0 reseed → `lo_idx` changes → invalidates) and
`narrowband_enabled` (NB/WB switch → `nb` changes → invalidates). Both already
force a bank rebuild via the `want_buffer_nsymb != inband_down_buffer_nsymb`
free-all at arq_common.cc:3824, so the memo can never serve a stale size to the
bank. CONSUMERS: the bank build loop (3828) reads `inband_down_buffer_nsymb`; the
seat (3734) reads the floor cache. CORRECTNESS asserted in `--test-inband-deliver`
PART E/F: the memoized Nsymb == a fresh uncached probe for every site.

SWEEP RESULT (all `load_configuration(` + throwaway `cl_telecom_system` in
arq_common.cc / arq_responder.cc / arq_commander.cc):
- arq_common.cc:1767, 1822 — `init_monitor_decoders` throwaway probes: **one-time**
  (init), not on the RX pass.
- arq_common.cc:3643/3677 (memoized #1), 3814 (memoized #2) — the three leak sites,
  ALL now cached.
- arq_common.cc:3822, 3845 — the GENUINE reused decoder banks (`continue`-gated by
  config id) — NOT touched; zero rebuilds on a steady same-window loop.
- All other `load_configuration(` in the three files are **event-driven** production
  config switches (SET_CONFIG / demote / BREAK / NB-switch / session init) or live
  inside `test_*` functions — none run per RX pass.

VERDICT: after FIX #4, ZERO per-pass throwaway-PHY `load_configuration` calls remain
on the inband steady RX path. The only remaining down-ladder work is its FUNCTIONAL
0-recovery thrash (the snapshot/firing/BREAK-decouple FIXes #1–#3), handled
separately above.

## §9. Regression — `--test-inband-deliver` PART E/F
PART E (FIX #1) and PART F (FIX #4 instance #2) each drive the production hot-path
helper in a steady loop and count throwaway PHY probes via `inband_floor_probe_count`:
- **PART E** (`inband_seat_robust_ring_floor` × 50): fail-before
  (`-DINBAND_DELIVER_FAILBEFORE`) = 50 probes; pass-after = 0. Plus E1 (cached floor
  == fresh probe), E2 (genuine first seat still grows the ring).
- **PART F** (`inband_ensure_down_decoders` × 41): fail-before = 1 (first build) + 40
  (per-call); pass-after = 1 (first build) + 0 (steady). Plus F2 (cached bank Nsymb
  == fresh probe), F1 (bank still builds).
Both arms in one binary; `./mercury.exe --test` green; legacy byte-identical
(inband-gated; default-off render md5 unchanged).

## §10. FIX #5 — ROBUST-TIER lost-tag follow (the OFDM-only gate generalization)

### §10.0 The bug (faithful realtime sim, MEASURED — `_fixrun/fb.*`)
Redesign-on (`MERCURY_INBAND_RATE=1`), faithful realtime relay (`MERCURY_SIM_REALTIME=1`,
FTRT 1.03× = wall≈airtime), WGN:18, start ROBUST_0, 12 KB target → **15 bytes delivered,
stuck at ROBUST_0, then a true-loss BREAK**. Legacy (inband-off) delivers ~4259 B on the
same channel. Root cause (clean single root): the RX config-tag-follow recovery is
**OFDM-only**. The rate-adapt legitimately probes **ROBUST_0 → ROBUST_1** on the degraded
channel (`is_ofdm_config(ROBUST_*)==false`, common_defines.h:125-130/186-191:
ROBUST_0/1/2 = FULL_CONFIG_LADDER idx 0/1/2). Measured trajectory (`_fixrun/fb.modem.log`):
- CMD `[INBAND-TX] CONFIG_TAG passband emit cfg=101 (ladder_idx=1)` × **3** (the up-probe).
- RSP `CONFIG_TAG follow` count = **0** — the RX never follows.
- RSP `[RX-TIMEOUT] No frames decoded. cfg=100` × **10** (~60 s stuck at ROBUST_0).
- CMD `[INBAND-LIVENESS] … firing the retained true-loss BREAK (#1/3)`.

Why neither recovery fires at robust:
1. The **W2 adopt-follow** (arq_responder.cc:974, inside `[RSP-V2-ADOPT]`) only runs after a
   SUCCESSFUL DATA decode — impossible across the ROBUST_0/ROBUST_1 MFSK PHY mismatch.
2. The **Stage-4 OFDM down-ladder** (arq_responder.cc:588-596) is GATED
   `is_ofdm_config(current_configuration)` (FALSE at robust). Even if ungated, its
   `inband_down_ladder_resync` searches the window `FULL_CONFIG_LADDER[lo..hi=cur_idx]`
   **DOWNWARD** (arq_common.cc:3967-3981) — it structurally cannot reach the UP-probe
   target idx 1 from cur idx 0.
The liveness BREAK is downstream fallout, not a 2nd root.

### §10.1 The fix (arq_responder.cc:576-622, the block ADDED before the OFDM down-ladder)
Add a robust-tier sibling to the OFDM down-ladder, gated on the **complement**
`!is_ofdm_config(current_configuration)` (all other firing terms identical: feature-on,
CONNECTED+RECEIVING, !passive_monitor, fresh-window, `messages_rx_buffer.status!=RECEIVED`,
`rsp_current_expected_batch_seq_id>=0`). It calls the SHARED direction-agnostic real-tag
follow `inband_detect_follow_from_capture(0xFF,0xFF,&fc)` (arq_common.cc:3434), which:
- sizes a capture-tail snapshot from the **current robust PHY geometry** and reads the live
  ring tail under `capture_prep_mutex` (arq_common.cc:3468-3482) — this IS the robust-ring
  snapshot. The **robust-ring floor seat** (arq_responder.cc:567-574 →
  `inband_seat_robust_ring_floor`) guarantees the ring physically holds a full robust frame
  whose tail carries the tag (`emit_config_tag_passband` fires for ROBUST_0/1 — the tag rides
  the M=16 robust suffix, arq_common.cc:2803-2805);
- runs `detect_and_follow_config_tag` → `FULL_CONFIG_LADDER[r.cfg_index]`, which follows
  **UP or DOWN** (arq_common.cc:3008-3056), recovering the ROBUST_0→ROBUST_1 up-probe;
- on a CRC/LDPC-bound follow runs the HINGE via `inband_adopt_resynced_config`; the caller
  re-arms the receiving timer. No tag in the tail (steady state / silence) → cheap false,
  no action, no streak tick, no BREAK.
The OFDM down-ladder `if` block is left **byte-identical** (still `is_ofdm_config && …`).

### §10.2 Producers / Consumers (CLAUDE.md §5)
- **Producer**: CMD CONFIG_TAG (`emit_config_tag_passband`, arq_common.cc:2796; tagged from
  `inband_route_failure_demote`/`inband_unilateral_config_change`). May tag ANY ladder config
  INCLUDING robust (ladder_idx 0..31, no OFDM gate) — VERIFIED on the wire (cfg=100 ×2,
  cfg=101 ×3 in `_fixrun/fb.modem.log`).
- **Consumer (changed)**: the RX lost-tag recovery. BEFORE: only the OFDM down-ladder, gated
  off at robust → robust tags unfollowable. AFTER: robust tags are followed via the real-tag
  path; OFDM tags continue via the down-ladder (unchanged).
- **Robust-ring snapshot geometry**: `inband_detect_follow_from_capture` reads the SAME ring
  the production tag-decode (`decode_config_tag_from_passband`) and the OFDM follow read; the
  tail length is computed from `data_container.Nofdm/interpolation_rate/buffer_Nsymb` at the
  CURRENT (robust) config — correct for a robust resync. The floor-seat (§8 FIX #4 instance #1)
  keeps `buffer_Nsymb_min` ≥ the robust floor so the tail is never truncated.

### §10.3 Invariants the fix changes (consumers re-checked)
- **INV-OFDM-FOLLOW (preserved)**: the OFDM down-ladder block is unchanged (separate `if`,
  `is_ofdm_config` retained). The two robust/OFDM blocks are mutually exclusive on
  `is_ofdm_config(current_configuration)` → never both fire in one pass → no double-adopt.
- **INV-ADOPT (preserved)**: the `[RSP-V2-ADOPT]` adopt-path follow (:906/:974) is untouched;
  it still gates on `rsp_current_expected_batch_seq_id < 0` and a successful DATA frame. The
  robust follow runs the SAME `inband_adopt_resynced_config` HINGE (capture-flush + D3.1
  re-baseline back to bsi=-1) so the next robust frame re-adopts through the gap-gate exactly
  as the OFDM follow does — no divergence.
- **INV-NO-BLIND-GUESS (preserved)**: the robust follow adopts ONLY on a CRC-12 + cfg_index
  corroborated tag (FAR ~1e-8); a silent / no-tag tail self-rejects. Unlike the OFDM blind
  down-ladder it does not even attempt a speculative decode, so it cannot tick the
  terminal-BREAK streak — the robust path is strictly less aggressive (follow-only).

### §10.4 Evidence (faithful realtime sim, WGN:18, 12 KB, inband-on)
- FAIL-BEFORE (HEAD 61e5eaa, `_fixrun/fb.*`): rx=**15 B**, final ROBUST_0, RSP follow=0,
  10× RX-TIMEOUT, INBAND-LIVENESS BREAK, md5_match=false.
- PASS-AFTER (`_fixrun/pa.*`): see commit message / §10.4 below — the RX follows cfg=101,
  delivers e2e comparable to legacy, no ~60 s stall, no liveness BREAK.
- Default-off (`MERCURY_INBAND_RATE` unset): the whole block is feature-gated → legacy
  byte-identical.
