# Data-Flow Audit — robust→OFDM in-band adopt capture-ring flush (HINGE-1)

Branch: `feat/inband-a3-decouple`. Scope: the in-band rate-adapt UNILATERAL adopt path
only (`MERCURY_INBAND_RATE=1`). Legacy SET_CONFIG behavior is byte-identical (untouched).

This audit covers the shared state mutated by HINGE-1 inside
`cl_arq_controller::inband_adopt_resynced_config` (arq_common.cc:4071) — the capture ring
(`passband_delayed_data`, `ring_write_index`) and the OFDM acquisition cursors
(`ofdm_search_raw`, `ofdm_batch_active`).

---

## §1 The bug (root cause — VERIFIED by code read)

The in-band redesign drops to a robust config on degradation and, on recovery, the RX
re-acquires the OFDM config by either (a) decoding the CONFIG_TAG burst
(`detect_and_follow_config_tag`, arq_common.cc:2941) or (b) the blind down-ladder
(`inband_down_ladder_resync`, arq_common.cc:3935 → :4049). BOTH funnel through
`inband_adopt_resynced_config(cfg)` (arq_common.cc:4049, :3053), which runs HINGE-1
(arq_common.cc:4087-4102):

```
circular_buf_reset(capture_buffer);
memset(passband_delayed_data, 0, 2*buf_samples*sizeof(double));   // WIPE the ring
ring_write_index            = 0;                                  // reset the write head
receive_stats.ofdm_search_raw   = 0;
receive_stats.ofdm_batch_active = false;
```

The capture path is a DOUBLE-MAPPED ring (audioio.c:1430-1453): the capture-prep thread
writes incoming passband samples at `passband_delayed_data[ring_write_index]` (+ mirror at
`+sp`) and advances `ring_write_index = (wi + symbol_period) % sp`. The OFDM acquisition
reads the contiguous `signal_period`-sample window at `[ring_write_index + offset]`
(arq_common.cc:3305-3308, :3478-3481; the coarse Schmidl-Cox / FTR search in
telecom_system.cc:1387+ reads `baseband_data_decimated`, decimated from that window).

On the robust→OFDM crossing (e.g. 102→CONFIG_0) the CMD's CONFIG_0 OFDM batch is ALREADY
airing — it overlaps the adopt by seconds. The down-ladder/tag decode only consumed frame 0
out of a snapshot; the REST of the burst is still streaming into the primary ring. HINGE-1's
`memset` ZEROES the in-flight preamble samples already captured, and `ring_write_index=0`
re-anchors the read window away from them. For the remaining ~5s the coarse search sees a
zeroed / partial ring → `coarse_metric≈0`, `pream_symb_loc < lower_bound` → SKIP
(telecom_system.cc:1695-1710, :1783) → never acquires one OFDM frame → "no trial decoder saw
signal" → 3 total-loss → TERMINAL BREAK → ROBUST_0 spiral → 53B vs legacy 5645B.

LEGACY avoids this: the coordinated SET_CONFIG handshake flushes+re-acquires in LOCKSTEP on
both sides (the CMD does not start the new-config burst until after its own
`send_batch` flush, arq_common.cc:7671-7683, post-ACK-turnaround). The redesign's UNILATERAL
drop has no such barrier — the adopt fires WHILE the burst is mid-flight.

The flush's LEGITIMATE purpose is to kill STALE preambles from the OLD config (same
Schmidl-Cox preamble structure across OFDM configs → false-lock). That hazard applies to a
COLD adopt (no live burst — inter-frame, robust-tier idle). It does NOT apply to a LIVE
current-config burst already mid-capture, which is exactly the samples we need to KEEP.

The codebase ALREADY recognizes "flush during an active OFDM batch is unsafe":
`inband_seat_robust_ring_floor` is gated `!receive_stats.ofdm_batch_active` with the comment
"between frames: flush is safe" (arq_responder.cc:567-573). HINGE-1 has no such guard.

---

## §2 Producers (writers) of the capture ring + OFDM cursors

`passband_delayed_data` / `ring_write_index`:
- audioio.c:1430-1453 — the capture-prep thread (the ONLY continuous producer; double-map
  write + `ring_write_index=(wi+symbol_period)%sp`).
- data_container.cc:76,192,264 — init (`ring_write_index=0`, buffer alloc/zero).
- The MANY HINGE/flush sites: arq_common.cc:4093-4097 (this fix), :7671-7683 (send_batch TX),
  :7714-7724 (big-block TX), :8169+/:8413+/:8538+/:9141+/:9349+/:9531+/:9876+ (SET_CONFIG +
  ACK-turnaround flush family), arq_responder.cc:194-204 (HAIL monitor), :3251-3259
  (SET_CONFIG monitor).

`ofdm_search_raw` / `ofdm_batch_active`:
- telecom_system.cc:95-96,108-109,5246-5247,10951-10952 — init / reset.
- telecom_system.cc:1707 — sub-threshold coarse → `ofdm_batch_active=false`.
- arq_common.cc:2316-2321 — turboshift stale-position reset.
- the same HINGE/flush sites above set both to 0/false.

## §3 Consumers (readers) of the capture ring + OFDM cursors

- telecom_system.cc:1387-1471 — the OFDM coarse/BATCH-predict acquisition reads
  `baseband_data_decimated` (decimated from the ring window) and keys off
  `ofdm_batch_active && ofdm_search_raw>0` for predict-verify vs full search.
- telecom_system.cc:1390,1799 — `ofdm_skip = ofdm_search_raw - nUnder_processing_events`
  (anti-re-decode position).
- arq_common.cc:3305-3308 / :3478-3481 — `read passband_delayed_data[ring_write_index+off]`
  to pull the CONFIG_TAG / NACK suffix tail (snapshot for tag-follow).
- arq_common.cc:4239-4256 — the down-ladder snapshot read (`ready_to_process_…` staged copy).
- audioio.c:1431 — the producer also READS `ring_write_index` to place the next write.

## §4 Valid states (the cursor BEFORE any producer writes)

- Cold init: ring all-zero, `ring_write_index=0`, `ofdm_search_raw=0`,
  `ofdm_batch_active=false`. (data_container.cc, telecom_system.cc init.)
- Robust-tier idle (cold adopt target): the RX is in MFSK; `ofdm_batch_active=false`; the
  ring holds MFSK/idle/old-config audio (STALE w.r.t. the OFDM target) — flush IS correct.
- LIVE in-flight OFDM burst (the bug case): the ring holds the CMD's current-config OFDM
  preamble+data already captured; energy peak ≫ noise; `ofdm_batch_active` may be true (mid
  OFDM batch) OR false (the very first crossing from MFSK, before the first OFDM frame locks).

## §5 Invariants the consumers assume

I1. The acquisition reads a window starting at `ring_write_index`; that window must contain
    the LIVE current-config preamble for the coarse search to lock. (telecom_system.cc:1387+.)
I2. `ofdm_search_raw`/`ofdm_batch_active` describe positions WITHIN the current ring contents;
    after a real config change the per-frame geometry differs, so stale OFDM cursors must NOT
    be carried across a change (they'd point at the wrong frame stride). → reset to 0/false.
I3. STALE old-config preambles in the ring must be removed before acquiring the new config
    (Schmidl-Cox false-lock). The ONLY producer that can leave stale old-config samples is the
    pre-adopt history; a LIVE new-config burst is NOT stale — it is exactly I1's target.

## §6 What the fix changes + per-consumer walk

The fix scopes HINGE-1's RING-WIPE (memset + ring_write_index=0 + circular_buf_reset) to the
COLD case. When adopting INTO an OFDM config AND a live in-flight OFDM burst is present on the
ring (energy peak ≥ LIVE_BURST_PEAK over `passband_delayed_data`, OR `ofdm_batch_active`), the
ring contents AND `ring_write_index` are PRESERVED; only the OFDM cursors
`ofdm_search_raw=0`/`ofdm_batch_active=false` are reset (a fresh ANCHOR search from the
preserved write head — re-locating frame 0 of the live burst with a FULL preamble search,
which is correct for a new geometry). The COLD case (no live energy, not active) keeps the
full destructive flush verbatim (kills stale old-config preambles).

Per-consumer:
- I1 (acquisition window) — PRESERVED contents keep the live preamble in the read window
  starting at the UNCHANGED `ring_write_index`; the coarse FULL search re-anchors frame 0
  (ofdm_search_raw=0, ofdm_batch_active=false → the predict-verify is bypassed, a full search
  runs, telecom_system.cc:1407 gate is false). VERIFIED the FTR coarse search reads the ring
  window and locks on the preserved energy. ✓ (the regression test asserts search_raw>0.)
- I2 (cursor geometry) — STILL reset to 0/false on EVERY adopt (both branches). The fresh
  anchor search re-derives positions for the new geometry. ✓ unchanged.
- I3 (stale-preamble kill) — COLD adopt (no live burst): the full destructive flush is
  UNCHANGED → stale old-config preambles still removed. ✓. LIVE adopt: the only ring contents
  are the live NEW-config burst (the very burst we just decoded frame 0 of), which is NOT
  stale — preserving it is correct, not a hazard.
- HINGE-2 (D3.1 bsi re-baseline, arq_common.cc:4106-4116) — UNCHANGED in both branches
  (independent of the ring).
- MFSK-tier follow (robust→robust, the ROBUST_0→ROBUST_1 lost-tag case, arq_responder.cc:576+)
  — `is_ofdm_config(followed_config)` is FALSE for a robust target, so the fix's live-preserve
  branch is NOT taken; the full flush runs verbatim (correct — a robust MFSK target wants a
  clean ring for the MFSK preamble search, and there is no "live OFDM burst" to preserve). ✓.
- `nUnder_processing_events` — NOT touched by HINGE-1 (neither before nor after the fix);
  unchanged. The anti-re-decode `ofdm_skip` math (telecom_system.cc:1390) sees
  ofdm_search_raw=0 → ofdm_skip≈0 → a full-window search, which is what a fresh anchor wants.

## §7 Cold/stale-adopt + MFSK-follow safety — VERDICT

PRESERVED. The destructive flush is retained verbatim for: (a) a cold robust-idle adopt (no
live OFDM energy, ofdm_batch_active=false) and (b) ANY robust-target adopt (is_ofdm_config
false) — i.e. the MFSK-tier follow. Only the (is_ofdm_config target) + (live-burst energy)
combination takes the preserve branch. Legacy/default (`!inband_rate_feature_enabled()`) never
calls `inband_adopt_resynced_config` at all → byte-identical.

## §8 Regression test (in-process, fails-before / passes-after)

`test_inband_adopt_preserve_live_burst()` (arq_responder.cc, run from `--test` and CLI
`--test-inband-adopt-preserve`): builds a production RX at CONFIG_1, writes a synthetic LIVE
OFDM preamble into `passband_delayed_data` (energy peak ≫ 0.05) at a known `ring_write_index`,
then drives `inband_adopt_resynced_config(CONFIG_0)`. ASSERT the ring still holds the burst
energy and `ring_write_index` is preserved post-adopt (PASS-AFTER); a defeat arm
(`MERCURY_ADOPT_FLUSH_DEFEAT=1`) restores the unconditional wipe → asserts the ring is zeroed
(FAIL-BEFORE signature). A COLD arm (silent ring) asserts the full flush still runs. The PHY
acquisition consequence is asserted via a coarse Schmidl-Cox search over the post-adopt ring:
`search_raw>0` (lock) preserve vs `=0` (no lock) on the wiped/defeat arm.
