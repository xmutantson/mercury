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
- `nUnder_processing_events` / `frames_to_read` — ~~NOT touched by HINGE-1 … which is what a
  fresh anchor wants.~~ **FALSIFIED by the decisive A/B (see §9).** Fix #1's preserve DID land
  (1 OFDM acquisition vs 0) but the RX still STALLED at ~101B: after the crossing the
  re-anchored coarse search reported `FTR-FAIL CONFIG_0 metric=0.13-0.24 search_raw=0` and never
  re-locked. The reason: leaving `frames_to_read==0` makes the very next rx_transfer snapshot the
  ring IMMEDIATELY (telecom_system.cc:5402) before a full new-geometry frame is captured, and a
  STALE `nUnder_processing_events` scrolls `ofdm_skip` (telecom_system.cc:1390) off the live
  preamble. **Fix #1b** re-arms BOTH (the legacy receiver re-init) — see §9.

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

---

## §9 Fix #1b — OFDM-receiver RE-INIT (the second half of the robust→OFDM class)

### §9.1 The residual stall (VERIFIED by the decisive A/B + code read)

Fix #1 (§1–§8) defeated the ring-WIPE: the in-flight preamble survives the adopt (the A/B
measured 1 OFDM acquisition vs 0). But the redesign still STALLED at ~101B. The RX's
re-anchored coarse/anchor search could NOT RE-LOCK after the preserved crossing:
`FTR-FAIL CONFIG_0 metric=0.13-0.24 search_raw=0` ("all silent") for the rest of the burst —
never decoded one CONFIG_0 frame → down-ladder → stall. LEGACY's coordinated SET_CONFIG path
re-locks fine.

ROOT (VERIFIED): `inband_adopt_resynced_config` preserved the SAMPLES but did NOT re-INITIALIZE
the OFDM receiver's FTR/anti-scroll state for the new geometry. Every LEGACY config-change
receiver re-init does TWO things the adopt was MISSING:
1. `nUnder_processing_events = 0`
2. `frames_to_read = frame_symb + 10` (a POSITIVE countdown; `frame_symb = preamble_nSymb + Nsymb`)

Cited legacy sites (the proven recipe — REUSED verbatim, not reinvented):
- SET_CONFIG post-ACK turnaround: arq_responder.cc:1818 (`frames_to_read = frame_symb + 10`),
  :1819 (`nUnder_processing_events = 0`).
- BREAK re-init: arq_common.cc:9425 (`nUnder=0`), :9433 (`frames_to_read = frame_symb + 10`).

What the adopt did instead: `load_configuration` set `frames_to_read=0` (arq_common.cc:2084) —
and on a cross-MODULATION switch (robust MFSK → OFDM) the PHY reinit `set_size` reseeds it to
`preamble_nSymb+Nsymb` (data_container.cc:189), but on an OFDM→OFDM adopt NOTHING re-armed it,
and `nUnder_processing_events` carried a stale value from the robust-tier dwell.

Mechanism: with `frames_to_read==0` the next rx_transfer (telecom_system.cc:5402) snapshots the
ring `&passband_delayed_data[rwi]` IMMEDIATELY — before the capture thread (audioio.c:1455
`frames_to_read--`) re-fills a WHOLE fresh CONFIG_0 frame at the new geometry — and a stale
`nUnder` scrolls the anti-re-decode skip `ofdm_skip = ofdm_search_raw - nUnder`
(telecom_system.cc:1390) off the live preamble. Net: a partial/misaligned coarse window →
metric 0.13-0.24 → never PASS → stall.

### §9.2 The fix (arq_common.cc:4169+, inband-scoped)

After the OFDM-cursor reset, when adopting INTO an OFDM config (`is_ofdm_config(followed_config)`),
re-arm `nUnder_processing_events = 0` and `frames_to_read = preamble_nSymb + Nsymb + 10` (the
legacy formula verbatim), under the same `capture_prep_mutex`. The capture thread then accrues
one full new-geometry frame into the PRESERVED ring BEFORE the next snapshot, so the fresh anchor
search sees a complete CONFIG_0 preamble and locks (`search_raw>0`). A robust/MFSK target keeps
the destructive flush + its own MFSK FTR/search machinery (untouched). Off path
(`!inband_rate_feature_enabled()`) never runs → legacy byte-identical.

### §9.3 Preserve + re-init COMPOSE (the §5 audit — VERDICT: YES)

The re-arm does NOT scroll the preserved preamble out of the ring: the ring size is
`buffer_Nsymb = frame_symb + turnaround_symb + frame_symb + margin` (data_container.cc:159) ≫
`frame_symb + 10`, so during the FTR fill the most-recent `buffer_Nsymb` symbols (the
double-mapped read window) still contain the preserved preamble. While `frames_to_read > 0` the
overrun counter `nUnder++` (audioio.c:1425, gated on `frames_to_read <= 0`) does NOT fire, so
`nUnder` stays 0 and `ofdm_skip` stays at the fresh-anchor 0. The two fixes compose: fix #1 keeps
the samples in the ring, fix #1b gives the capture thread time to complete the frame at the new
geometry and positions the snapshot. On the cross-MODULATION crossing (where load_configuration
reallocates+zeroes the ring, data_container.cc:248/170) the preserve is moot but the FTR re-arm
is still the load-bearing step — `frame_symb+10` is a strict superset of the PHY reinit's
`preamble_nSymb+Nsymb` reseed (+10 margin, +nUnder reset).

### §9.4 Regression (PART D of test_inband_adopt_preserve_live_burst)

PART D pre-stages the stall (`frames_to_read=0`, `nUnder=99`), drives the OFDM-target adopt, and
asserts PASS-AFTER `frames_to_read == frame_symb+10` (>0) + `nUnder==0`; FAIL-BEFORE
(`MERCURY_ADOPT_FTR_REARM_DEFEAT=1`, same binary) leaves `frames_to_read==0` (the ~101B stall
signature). Wired into `--test` + `--test-inband-adopt-preserve`.

---

## §10 Fix #1c — RESTORE THE NATURAL OFDM ACQUISITION GEOMETRY (the last half of the class)

### §10.1 The residual blocker (VERIFIED by the fair A/B `_fairgo/arm1_redesign.arqlog`)

Fixes #1 + #1b STILL did not lock the robust→OFDM crossing. After the preserve + FTR re-arm
(arm1, line 17585) the RX logged for the whole burst (11 s, T+151→162):

```
[RX-TIMING] OFDM beyond-bounds: pream=1279 upper=1239 metric=0.120 shift=8 search_raw=0
[FTR-FAIL]  CONFIG_0 ftr=8 metric=0.120 batch=0 search_raw=0
```

The preamble was found at snapshot symbol **1279 > upper_bound 1239** with metric **0.12** (a false
GI sub-peak — a REAL preamble is 0.999, cf. the working RSP lock `pream_symb=1230 metric=0.999 PASS`
in `arm2_defeat.arqlog:17899`). The preamble slid 1279→1272→1266 then JUMPED back to 1276 — a
NEW re-aired burst at the tail. It never reached `≤upper`.

### §10.2 ROOT (VERIFIED — code read + the A/B geometry)

The in-band redesign seats `buffer_Nsymb_min` to the ROBUST floor (`inband_seat_robust_ring_floor`,
arq_common.cc:3781) so the blind down-ladder can read a full slow MFSK ROBUST_0 frame. That makes the
**CONFIG_0 OFDM capture ring `buffer_Nsymb=1291`** (run-specific; the test build = 804). The NATURAL
CONFIG_0 ring is **217** — the size LEGACY uses (legacy has NO robust-floor seat: `arm3_legacy.arqlog`
shows `bufNsymb=217` EXCLUSIVELY, never 1291) and which locks 43× in a flat baseline.

The snapshot reads `[ring_write_index, +signal_period)` (telecom_system.cc:5594); index 0 = oldest,
index `buffer_Nsymb-1` = newest (the write head). The CMD RE-AIRS the CONFIG_0 burst continuously
(`[CMD-TX] CONFIG_0 batch=6`, `[INBAND-TX] re-tag #1`), so the **freshest preamble always lands at
the TAIL** of the oversized window — `pream_symb ≈ buffer_Nsymb (1279) > upper_bound (1239)` — with its
frame DATA off-buffer (`1279+frame_symb 52 = 1331 > 1291`). **No `frames_to_read` value fixes this**:
the re-air races the slide so the tail preamble never settles below `upper`. LEGACY never hits it —
its 217 ring puts the tail preamble at `≈upper=165`, which FITS. **The robust-floor oversize is the
inband-specific amplifier legacy lacks**, and it is the SHARED-crossing reliability lever (the natural
geometry locks; the oversized one never does).

This FALSIFIES §9.3's claim that "preserve + re-init COMPOSE": they do not, because the §9.3 audit
assumed the read window placed the preamble in-bounds. In the robust-floor-OVERSIZED ring it places it
at the tail, beyond `upper`.

### §10.3 The fix (arq_common.cc:4257+, inband-scoped)

On an adopt INTO an OFDM config, if the live ring is LARGER than that config's NATURAL size
(`inband_natural_ofdm_buffer_nsymb(cfg)`, arq_common.cc:3738), UN-seat the floor and re-allocate the
ring at the natural OFDM size (`cl_telecom_system::force_set_capture_ring_natural`,
telecom_system.cc:11111 — mirrors `force_resize_capture_ring` but un-seats). Re-arm the FTR after the
realloc (set_size zeroed the ring + reset `frames_to_read`). `inband_ofdm_acq_ring_shrunk` (new, arq.h)
GATES the per-pass robust-floor re-seat (arq_common.cc:3809) so it cannot immediately re-grow the ring;
the flag CLEARS on a demote to a robust config (arq_common.cc:3811-3812, :4149) so the next down-ladder
re-seats the floor BEFORE it reads a robust frame. This restores LEGACY's exact natural geometry for
OFDM acquisition — re-using the proven `set_size` path, not a new acquisition routine. Off path
(`!inband_rate_feature_enabled`) never runs → legacy byte-identical. FAIL-BEFORE A/B:
`MERCURY_ADOPT_RING_SHRINK_DEFEAT=1` keeps the oversized ring on the SAME binary.

### §10.4 §5 AUDIT — the ring-geometry / coarse-search-window / seat composition (VERDICT: YES)

- **Producers of `buffer_Nsymb`/`buffer_Nsymb_min`**: `data_container.set_size` (the only writer of
  `buffer_Nsymb`, honoring `buffer_Nsymb_min` as a floor, data_container.cc:147-163);
  `inband_seat_robust_ring_floor`/`force_resize_capture_ring` (raise the floor + grow);
  `force_set_capture_ring_natural` (NEW — un-seat to 0 + shrink to natural). All hold
  `capture_prep_mutex` across the realloc.
- **Consumers of the geometry**: the coarse search `upper_bound = buffer_Nsymb-(Nsymb+rx_eff_preamble)`
  (telecom_system.cc:1766) and `pream_symb_loc = delay/(Nofdm*interp)` (`:1721`); the ARQ
  beyond-bounds gate `upper = buffer_Nsymb-frame_symb` (arq_common.cc:12054); the snapshot
  `memcpy(&passband_delayed_data[rwi], signal_period)` (telecom_system.cc:5594). After the shrink ALL
  read the natural size coherently (one `set_size` re-derives every dependent buffer), so the preserved/
  re-aired preamble at the tail now sits at `≈upper_natural=165`, WITHIN `[lower=4, upper]`, frame fits.
- **Invariant restored**: the down-ladder still needs the big ring to READ a robust frame — preserved
  because the shrink is only kept while `is_ofdm_config(current_configuration)`; a demote clears the
  flag and the next `inband_seat_robust_ring_floor` re-grows BEFORE the down-ladder reads (cold/stale/
  MFSK-target adopt paths still flush + re-seat verbatim — `adopt_into_ofdm` false → no shrink).
- **Uncommon paths**: cross-MODULATION crossing (set_size already reallocs) — the shrink is a no-op if
  the ring is already ≤ natural; a robust-target adopt clears the flag (arq_common.cc:4149) so the floor
  re-seats. The seat's idempotent fast-path cache (`inband_robust_floor_nsymb_cached`) is unaffected (the
  shrink un-seats `buffer_Nsymb_min`, not the cache).

### §10.5 Regression (PART E of test_inband_adopt_preserve_live_burst)

PART E seats the robust floor (grows the live ring), drives the OFDM-target adopt, and asserts
PASS-AFTER `buffer_Nsymb == natural_CONFIG_0` + `inband_ofdm_acq_ring_shrunk` set + a subsequent
`inband_seat_robust_ring_floor()` does NOT re-grow it; FAIL-BEFORE (`MERCURY_ADOPT_RING_SHRINK_DEFEAT=1`,
same binary) leaves the ring OVERSIZED (the permanent-beyond-bounds signature). Wired into `--test` +
`--test-inband-adopt-preserve`. Measured (test build): floor=804, natural=212; shrink 804→212 PASS,
defeat stays 804.

## §11 Fix #1d — DURABILITY of the fresh OFDM lock through a transient ROBUST probe (the residual collapse)

### §11.1 The residual blocker (VERIFIED by the diagnosis + code read)

Fixes #1/#1b/#1c make the robust→OFDM crossing LOCK (metric ~0.998, bufNsymb 217, ACK-SACKs a data
batch). But ~5s later a TRANSIENT robust-tier reload collapsed the lock (metric 0.998 → 0.289 → 0.181)
→ down-ladder total-loss → TERMINAL BREAK → ROBUST_0 → 53B. ROOT (two coupled defects):

- **(A) The transient robust ADOPT.** The down-ladder fires on a single inter-frame decode-FAIL pass
  even with a healthy lock (gate arq_responder.cc:642-650). Its window `[cur-D .. cur]` reaches the
  ROBUST tier (CONFIG_0 idx 3, D≥3 → ROBUST_0 idx 0). A trial ROBUST_0 decoder spuriously CRC/LDPC-
  passes on residual energy; `inband_down_ladder_resync` then `inband_adopt_resynced_config(ROBUST_0)`
  (arq_common.cc:4076→4109) reloads ROBUST_0 on the PRIMARY → `current_configuration` flips non-OFDM →
  the next `inband_seat_robust_ring_floor` CLEARS the shrink flag (:3811) and RE-GROWS the ring
  217→804 (:3849) → the live lock collapses. The down-ladder's trial decode uses THROWAWAY decoders
  (arq_common.cc:4011, never the primary) — so the trial is safe; the ADOPT that follows is the nuke.
  NB: the `[PHY] Loading configuration 100/101/102` lines in the failing log are the benign THROWAWAY
  probes (`inband_robust_floor_buffer_nsymb` tmp :3731, the down-decoder bank :3956, `inband_natural_
  ofdm_buffer_nsymb` tmp :3751) — `force_resize_capture_ring`/`force_set_capture_ring_natural` call
  `set_size` DIRECTLY and never `load_configuration`. The actual primary reload is the down-ladder adopt.
- **(B) The shrink flag did not fire reliably (the secondary nondeterminism).** Fix #1c set
  `inband_ofdm_acq_ring_shrunk` only inside the `cur_nsymb > natural_nsymb` shrink branch. If the
  robust-floor seat had not yet grown the ring at the adopt instant (`cur_nsymb == natural_nsymb`,
  adopt-ordering-dependent) the branch no-op'd and the flag was LEFT UNSET → a LATER seat re-grew the
  ring (the same collapse, deferred).

### §11.2 The fix (inband-scoped, two parts)

- **(A) GUARD the transient robust adopt (arq_common.cc, inband_down_ladder_resync trial loop).** While
  holding a fresh OFDM lock (`inband_ofdm_acq_ring_shrunk && is_ofdm_config(current_configuration)`),
  SKIP a robust trial config (`is_robust_config(cfg)`) — no `receive_byte`, no adopt, no attempt counted.
  The loop still trials any OFDM rung below (a legit in-OFDM rate-down re-shrinks correctly, :4274); if
  NOTHING decodes the caller ticks the dead-batch streak, so a REAL sustained loss STILL reaches the
  genuine §7 TERMINAL BREAK demote (which re-grows for robust capture via `load_configuration(ROBUST_0)`
  directly). FAIL-BEFORE: `MERCURY_ADOPT_RING_DURABILITY_DEFEAT=1` disables the guard.
- **(B) Set the flag reliably (arq_common.cc:4274+).** Set `inband_ofdm_acq_ring_shrunk = true` whenever
  adopt_into_ofdm AND the ring is now ≤ the config's natural size — NOT only when a physical shrink ran.
  The flag's true meaning is "we hold a natural-geometry OFDM lock; do not let a robust-floor seat
  re-grow it" — an invariant independent of adopt-instant ordering. Still gated by `ring_shrink_defeat`
  (the §10 FAIL-BEFORE arm keeps the oversized ring → flag stays meaningfully-off).

### §11.3 §5 AUDIT — `inband_ofdm_acq_ring_shrunk` durability (VERDICT: YES)

- **Producers of the flag**: set TRUE on adopt-into-OFDM @ natural geometry (arq_common.cc:4274+, now
  ordering-independent); set FALSE on adopt into a robust config (:4149) and in the seat when flag-set +
  non-OFDM (:3811-3812). **Consumer**: the seat's early-return suppress (:3809).
- **Lock SURVIVES a transient robust probe**: the guard skips the robust trial → no adopt → the flag
  stays set + current stays OFDM → the seat re-grow stays suppressed. VERIFIED (PART F PASS-AFTER:
  winner=-1, current stays CONFIG_1, flag set, re-seat no-grows; FAIL-BEFORE: robust decodes+adopts→100,
  flag cleared).
- **Down-ladder STILL recovers a genuine loss**: a robust adopt is suppressed ONLY while a fresh OFDM
  lock is held (flag set). A genuine CMD demote manifests as the OFDM lock dying across MANY passes →
  the dead-batch streak reaches SESSION_DEAD_BATCHES → §7 TERMINAL BREAK → `load_configuration(ROBUST_0)`
  (NOT this path) → flag clears → re-grow. The LEGITIMATE direct robust adopt
  (`test_inband_down_resync` PART 2b: from CONFIG_1 with flag=false) is UNAFFECTED — guard inactive →
  `CONFIG_100 DECODED → adopting` still fires (VERIFIED). The A3-DECOUPLE T5b3 GENUINE-DEATH test
  (sustained loss → BREAK) still PASSES.
- **Off path** (`!inband_rate_feature_enabled`): `inband_down_ladder_resync` returns -1 at :3985 before
  the guard; the flag-set is inside the inband adopt → legacy byte-identical.

### §11.4 Regression (PART F of test_inband_adopt_preserve_live_burst)

PART F seats the robust floor (grows the ring to 804 = the vulnerable oversized-but-flagged window),
sets the flag (models fix #1c's lock state), and drives `inband_down_ladder_resync` directly with a real
ROBUST_0 frame. PASS-AFTER (guard active): the robust trial is SKIPPED (winner≠ROBUST_0, current stays
OFDM, flag still set, a subsequent re-seat does not further re-grow). FAIL-BEFORE
(`MERCURY_ADOPT_RING_DURABILITY_DEFEAT=1`, same binary): the robust trial DECODES + ADOPTS (winner=100,
current flips non-OFDM, flag cleared — the collapse gate opens). Wired into `--test` +
`--test-inband-adopt-preserve`. Measured (test build): floor=804, natural=212; PASS-AFTER winner=-1,
FAIL-BEFORE winner=100.
