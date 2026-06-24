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

## §12 Fix — the HYBRID SET_CONFIG cross reachability gap (the SECOND cross-path miss)

### §12.1 The bug (VERIFIED by code read + workflow wf_3bc471e6-829)

Fixes #1/#1b/#1c/#1d live ONLY inside `inband_adopt_resynced_config` — the redesign's UNILATERAL
CONFIG_TAG-follow adopt. But at HEAD d28f02d the robust→OFDM tier-cross routes through the HYBRID
legacy SET_CONFIG path: the responder's data-config adopt at `arq_responder.cc:1723/1751/1764` is a
PLAIN `load_configuration(data_configuration, PHYSICAL_LAYER_ONLY, YES)`. That re-runs
`telecom_system::set_size` HONORING the still-raised `buffer_Nsymb_min` (the robust-floor seat,
`inband_seat_robust_ring_floor`) → the CONFIG_0 ring is RE-ALLOCATED at the ROBUST floor (~1291 sym,
or ~804 in the unit harness) instead of the natural ~217. Neither the shrink (`force_set_capture_ring_
natural`) nor the gate flag `inband_ofdm_acq_ring_shrunk` runs. Result: every re-aired CONFIG_0 preamble
lands at the TAIL of the oversized window beyond `upper_bound` (`OFDM beyond-bounds pream=1278 upper=
1239`) → ZERO forward frames decode → the responder sends no reverse ACK → the COMMANDER mislabels the
deaf-responder FORWARD failure as a CONFIG_0 REVERSE degradation and false-demotes via `emergency_nack_
threshold=3` → 36× loss to legacy at WGN:40 (REDESIGN flat 53B vs LEGACY climb to 5867B, seed s2000).

### §12.2 The fix (factor the OFDM-entry setup into a SHARED helper)

Extract the post-load OFDM-entry ring setup (HINGE-1 flush/preserve, OFDM cursor re-anchor, FTR re-init
#1b, natural ring SHRINK + gate flag #1c/#1d) out of `inband_adopt_resynced_config` into a shared helper
`inband_finalize_ofdm_adopt_ring(int adopted_config)` (arq_common.cc). `inband_adopt_resynced_config`
now calls `load_configuration` → the helper → HINGE-2 bsi re-baseline (behaviorally identical). Each of
the three SET_CONFIG cross sites (arq_responder.cc:1723/1751/1764) calls the helper right after its own
`load_configuration(data_configuration, …)`, gated `inband_rate_feature_enabled() && is_ofdm_config
(data_configuration)`. The cross can never again strand the shrink. FAIL-BEFORE: `MERCURY_ADOPT_RING_
SHRINK_DEFEAT=1` (the same #1c knob) keeps the oversized ring on the cross.

### §12.3 §1.2 SYSTEMIC AUDIT — which OTHER adopt-setup steps were stranded on the cross? (VERDICT)

Enumerated every step `inband_adopt_resynced_config` runs and checked the cross:
- **Ring SHRINK + flag (#1c/#1d)**: STRANDED — the load honors `buffer_Nsymb_min` → oversized. **THE bug. Folded into the shared helper.**
- **OFDM cursors** (`ofdm_search_raw=0`, `ofdm_batch_active=false`, `delay_of_last_decoded_message=-1`):
  NOT stranded — `telecom_system::load_configuration` resets them (telecom_system.cc:5244-5247).
- **FTR/anti-scroll re-arm** (`frames_to_read=frame_symb+10`, `nUnder=0`): NOT stranded — the responder's
  post-adopt FTR block re-arms it (arq_responder.cc:1818-1819) after all three cross sites.
- **HINGE-2 bsi re-baseline** + **NACK throttle re-arm** (`inband_nack_emitted_for_dead_streak=false`):
  UNILATERAL-adopt-specific. The coordinated SET_CONFIG cross owns its own bsi/session machinery; folding
  the unilateral re-baseline in would double-reset. DELIBERATELY left out of the shared helper.

### §12.4 §5 AUDIT — does the cross shrink break the down-ladder robust-floor need? (VERDICT: NO)

The capture-ring geometry feeds BOTH the OFDM preamble search AND the down-ladder trial-decoder (which
needs the robust floor to hold a ~336-sym ROBUST_0 frame). The cross shrink mirrors the unilateral path
EXACTLY: `inband_ofdm_acq_ring_shrunk` is set so `inband_seat_robust_ring_floor` does NOT re-grow the
ring while at an OFDM config (:3970); on a demote to a robust config the flag clears (:3972-3973 / :4346)
so the floor re-seats BEFORE the down-ladder reads a robust frame. Asserted by Part E2-5 (re-seat
suppressed while shrunk@OFDM after the cross) and the unchanged Part F durability tests. Everything is
gated on `inband_rate_feature_enabled()`; the legacy path never calls the helper → byte-identical.

### §12.5 Regression test (Part E2, arq_responder.cc test_inband_adopt_preserve_live_burst)

Drives the cross VERBATIM: build at ROBUST_0 → seat the robust floor (ring 804) → the cross's own
`load_configuration(CONFIG_0, PHYSICAL_LAYER_ONLY, YES)` (E2-2 asserts it leaves the ring 804, the
stranded state) → the shared helper. PASS-AFTER (arm 0): E2-3 ring shrinks 804→212, E2-4 flag set, E2-5
re-seat suppressed. FAIL-BEFORE (arm 1, `MERCURY_ADOPT_RING_SHRINK_DEFEAT=1`): E2-3 ring stays 804.

## §13 Fix — the post-demote robust/OFDM ceiling deadlock (belt-and-suspenders)

### §13.1 The structural deadlock (VERIFIED by code read)

If a (false) demote already landed, `inband_route_failure_demote` pins `supershift_proven_ceiling =
demote_target = ROBUST_2` (arq_commander.cc:3141). The §16 TIER GATE (`data_anchor_raise_target`,
arq.h:938-941) refuses to raise the ANCHOR (`last_data_viable_config`) across the robust→OFDM boundary on
robust evidence — a ROBUST clean ACK has a robust `streak_config`. So the anchor stays at robust-top,
`inband_ceiling_raise_target` caps the ceiling AT that robust-top anchor (Rule 3), the FRAME-UP gate
(arq_commander.cc:5460: `index(proposed) > index(ceiling)` ⇒ blocked) walls the +1 probe to CONFIG_0, and
the link can NEVER re-attempt the cross even once the channel recovers. (The PRIMARY §12 fix makes the
false-demote not happen; §13 covers a demote that already landed.)

### §13.2 The fix (robust/OFDM boundary tier-cross exemption — bounded, inband-only)

`inband_ceiling_raise_target` (arq.h): when the re-proven anchor sits AT the top robust rung
(`is_robust_config(anchor)` and the next ladder rung is OFDM — the natural CONFIG_0 entry), permit the
ceiling to reach EXACTLY that one OFDM-entry rung above robust-top — never higher. The existing +1
FRAME-UP anchor clamp (arq_commander.cc:5468: `proposed ≤ anchor+1`) already permits CONFIG_0 above a
robust-top anchor, so this only lifts the CEILING wall; it probes ONE rung above proven ground. If the
CONFIG_0 probe fails, the gearshift decode-failure demote re-pins the ceiling — no over-climb opened (the
+1 clamp remains the sole over-climb bound). The sole caller is inband-gated (arq_commander.cc:5406), so
legacy ceiling discipline is byte-identical. Tests X6/X6b/X6c (--test-climb-engine Part X): X6 lifts the
ceiling robust-top→CONFIG_0 on re-proof; X6b tops out AT CONFIG_0 (no leap); X6c stays sustained-gated.

## §14 Fix #1e — the cross ring re-grow (the 212-vs-217 flag-set skew, the CONFIG_0↔ROBUST_2 oscillation root)

### §14.1 The residual (VERIFIED — diagnosis run `_probe/lwp2001.arqlog`, FINALIZE-PROBE + the seat re-grow log)

After §12 (the cross runs `inband_finalize_ofdm_adopt_ring`) the redesign STILL stalled at WGN:40:
the RSP crossed to CONFIG_0, the ring shrank 1291→217, then ~2 ms LATER `inband_seat_robust_ring_
floor` RE-GREW the ring 217→804 — `OFDM beyond-bounds pream~200 upper=165` for the rest of the
CONFIG_0 visit → 0 forward decode → the commander false-demoted CONFIG_0→ROBUST_2. The link
OSCILLATED CONFIG_0↔ROBUST_2 every ~25 s, delivering ~53–268 B vs LEGACY's single clean cross +
climb to CONFIG_12–16 / 8000+ B (seed 2000-2004, WGN:40, secs 360).

ROOT (VERIFIED, the FINALIZE-PROBE line): `OFDM-RING SHRINK: CONFIG_0 ring 1291 -> natural 212` then
the live ring is **217**, and the #1d flag-set gate was `buffer_Nsymb.load() <= natural_nsymb`
= `217 <= 212` == **FALSE** → `inband_ofdm_acq_ring_shrunk` stayed 0 → the seat-gate (arq_common.cc:
3970) did not suppress the per-pass re-seat → it re-grew the ring. The 212-vs-217 gap is a 5-symbol
disagreement between `inband_natural_ofdm_buffer_nsymb()` (a THROWAWAY `cl_telecom_system`, fresh
load ⇒ 212, arq_common.cc:3891) and `force_set_capture_ring_natural()` (the LIVE `set_size` ⇒ 217,
telecom_system.cc:11111). The §11 #1d patch tried to widen the gate to `<=` but kept comparing the
LIVE ring against the THROWAWAY number — the skew defeated it.

### §14.2 The fix (arq_common.cc:4504-4561, inband-scoped)

Gate BOTH the shrink trigger AND the flag-set on the AUTHORITATIVE robust-floor state, not the
throwaway-vs-live size compare:
- SHRINK trigger: `buffer_Nsymb_min > 0` (a raised robust floor is oversizing the ring) instead of
  `cur_nsymb > natural_nsymb`. Fires exactly when a floor is seated; idempotent once un-seated.
- FLAG set: `inband_ofdm_acq_ring_shrunk = true` iff `buffer_Nsymb_min == 0` (the floor is un-seated,
  i.e. the live ring holds the NATURAL OFDM geometry — exactly what `force_set_capture_ring_natural()`
  guarantees, telecom_system.cc:11116). Immune to the 212/217 skew and to adopt-instant ordering.

`buffer_Nsymb_min == 0` IS the flag's true meaning ("robust floor un-seated, natural OFDM ring held").
`inband_seat_robust_ring_floor()` re-raises `buffer_Nsymb_min` on a real robust re-seat, and the flag
clears on a demote to a robust config (arq_common.cc:3972-3973 / :4376), so the down-ladder re-grows
the floor THEN, before it reads a robust frame — the §12.4 down-ladder contract is preserved.

### §14.3 §5 AUDIT — VERDICT: down-ladder + legacy preserved (YES)

- Down-ladder (consumer needing the robust floor): unchanged. While shrunk@OFDM the seat is suppressed
  (ring stays natural); on a demote to robust the flag clears → seat re-grows the floor (F-tests pass).
- Legacy: the whole block runs only under `inband_rate_feature_enabled()` → byte-identical off.
- FAIL-BEFORE: `MERCURY_ADOPT_RING_SHRINK_DEFEAT=1` skips the block → `buffer_Nsymb_min` stays raised →
  ring stays oversized (E2-3-DEFEAT got=804). Knob preserved.

### §14.4 Regression + sim (fails-before / passes-after)

- `--test`: 58/0 pass. E2-3/E2-4/E2-5 (cross shrink + flag set + re-seat suppressed) and F3 (flag
  durable through a transient probe) all PASS. (In the fresh unit harness live==throwaway==212, so the
  OLD gate `212<=212` also passed there — the skew is sim-only, where the live system carried robust/
  MFSK geometry; the new authoritative gate covers both.)
- Single-seed sim (seed 2001, WGN:40): BEFORE the ring re-grew 217→804 every cross (`bounds=[4,752]`,
  oscillation). AFTER: `OFDM-RING flag … ring=217 buf_min=0 shrunk=1`, ring HELD at 217 (`bounds=[4,165]`,
  0× `[4,752]` re-grow), OFDM-OK frames decode, CONFIG_0 lock HELD ~63 s (no oscillation) vs the prior
  ~25 s oscillation cycle.

### §14.5 NOT the last residual (HONEST — OPEN [?])

#1e eliminates the ring re-grow + the CONFIG_0↔ROBUST_2 oscillation, but the redesign at WGN:40 still
HOLDS CONFIG_0 then eventually demotes to ROBUST_2 without climbing — far below legacy's CONFIG_12-16.
The CONFIG_0 forward-decode RATE is too low to sustain/climb (seed 2001: 6 OFDM-OK over a 63 s hold).
This is a SEPARATE layer (CONFIG_0 decode rate / climb-from-CONFIG_0), NOT the capture-ring geometry.
[?] Next: characterize why CONFIG_0 OFDM-OK rate stays low after a clean coarse lock (FTR/snapshot
timing for the live re-aired burst, or the SNR margin at WGN:40 CONFIG_0).

~~[?] OPEN as of §14.5~~ — ~~RESOLVED in §15: the low CONFIG_0 OFDM-OK rate was the ring-shrink
**Nofdm drift**, NOT FTR/snapshot timing or SNR margin. The §14 fix held buffer_Nsymb (ring SIZE);
§15 holds Nofdm (per-symbol GEOMETRY), the remaining skew from the SAME throwaway-vs-live `ofdm.gi`
disagreement.~~ **WRONG — see §16: §15's "live Nofdm drift" was a mis-read of a THROWAWAY instance's
log line; the LIVE ring is 217/292 (correct). The residual is a CLEAN-LOCK decode-bit mismatch, OPEN.**

## §15 Fix — the ring-shrink Nofdm/GI drift (the CONFIG_0 under-decode root, diagnosis a468b2fc)

### §15.1 The bug (VERIFIED — diagnosis run `_probe/lwp2001.arqlog`)

After §14 (#1e fixes the ring SIZE re-grow), the redesign HELD CONFIG_0 but under-decoded (~53 B, all
OFDM-FAIL `iter=0` with `meanH=0.98` + preamble metric `0.999`, garbage CRC `0xC7C3`) — the §14.5
residual. ROOT: `force_set_capture_ring_natural()` (telecom_system.cc) — the HINGE-1 ring-shrink — RE-
DERIVED the per-symbol OFDM geometry `Nofdm = ofdm.Nfft*(1+ofdm.gi)` from the LIVE `ofdm` object,
which can be STALE at the cross. The just-completed `load_configuration` had installed the CORRECT
`data_container.Nofdm` (CONFIG_0 with the startup 3.0 ms GI: `Ngi=36` → `Nofdm=292`; `main.cc:4103`
writes `default_configurations.ofdm_gi = 36/256`), but the live `ofdm.gi` sat at the
`physical_config.cc:37` default `54/256` → `Nofdm=310`. The recompute OVERWROTE 292 with 310.

DIAGNOSIS LOG (the smoking gun — RSP, one CONFIG_0 visit):
```
T+0174.758 [RSP] init() done (... Nsymb=48 Nofdm=292 buffer_Nsymb=1291)   ← the LOAD: correct 292
T+0174.994 [RSP] init() done (... Nsymb=48 Nofdm=310 buffer_Nsymb=212)    ← the SHRINK: drifted to 310
T+0175.245 [CMD] init() done (... Nsymb=48 Nofdm=292 ...)                  ← CMD/TX stays 292
```
TX + the legacy demod run at Nofdm=292; the redesign RSP now demods at 310 → an **18-sample/symbol**
(310−292) FFT-window drift accrues across the 48-symbol CONFIG_0 frame → degenerate LLRs → LDPC
`iter=0` → garbage CRC. Legacy NEVER calls the shrink (keeps 292, decodes first try, climbs 0→16).

This is the SAME root as §14's 212/217 size skew: the live `ofdm.gi` (54/256) disagreeing with the
loaded geometry (36/256). §14 fixed the SIZE symptom (gating on `buffer_Nsymb_min`); §15 fixes the
GEOMETRY symptom (Nofdm), the actual decode-killer. The shrink producing `buffer_Nsymb=212` (the
throwaway number) instead of the correct 217 was the visible tell that its Nofdm arg was also wrong.

### §15.2 The fix (Approach A — telecom_system.cc force_set_capture_ring_natural + force_resize_capture_ring)

Drive the `Nofdm` argument to `set_size` from the AUTHORITATIVE, already-correct `data_container.Nofdm`
(the value the load installed) instead of `ofdm.Nfft*(1+ofdm.gi)`, so the symbol geometry is INVARIANT
across the ring resize — ONLY the ring SIZE (buffer_Nsymb) changes, which is the shrink's sole intent.
Applied to BOTH ring-resize producers that share the recompute pattern:
- `force_set_capture_ring_natural()` (the natural shrink, sole caller arq_common.cc:4520).
- `force_resize_capture_ring()` (the robust-floor GROW, sole caller `inband_seat_robust_ring_floor`,
  arq_common.cc:4010) — SIBLING-fixed for completeness (the floor dominates its buffer_Nsymb, but its
  Nofdm field would drift identically if `ofdm.gi` were ever stale at a re-seat).
FAIL-BEFORE / A-B knob `MERCURY_ADOPT_NOFDM_PRESERVE_DEFEAT=1` restores the legacy recompute on the
SAME binary (reproduces 292→310). Production never sets it. With the fix, the live geometry now yields
the CORRECT `buffer_Nsymb=217` (matching legacy / fix #1c's natural ring), not the throwaway's 212.

### §15.3 Regression test (in-process, fails-before / passes-after)

`cl_arq_controller::test_inband_adopt_nofdm_invariant()` (arq_responder.cc, CLI
`--test-inband-adopt-nofdm-invariant`, also in the master `--test`). Installs the production 3.0 ms GI
(`Ngi=36` → load `Nofdm=292`, mirroring main.cc:4103), seats the robust floor, runs the cross's bare
`load_configuration(CONFIG_0, PHYSICAL_LAYER_ONLY, YES)`, PERTURBS the live `ofdm.gi` back to 54/256
(modelling the exact production staleness — the finalize helper does NOT re-load, so it survives to the
shrink), then runs `inband_finalize_ofdm_adopt_ring(CONFIG_0)` and asserts `data_container.Nofdm`
INVARIANT. arm 0 (fix): 292 held. arm 1 (`MERCURY_ADOPT_NOFDM_PRESERVE_DEFEAT=1`): 292→310 (the bug).
Also asserts the ring STILL shrank to the natural OFDM size (217 < 400) — geometry-preserve must not
defeat the §14 size shrink. Master `--test` exit 0.

### §15.4 §5 AUDIT — VERDICT: size-shrink + down-ladder re-seat + legacy preserved (YES)

- (a) Ring-SIZE shrink (#1c/#1e, the 217 natural ring for preamble bounds): UNCHANGED. The fix touches
  only the Nofdm `set_size` arg; `buffer_Nsymb` is computed from `buffer_Nsymb_min=0` (un-seated) →
  natural sizing. The fix makes it yield the CORRECT 217 (was 212 under the drift). Adopt-preserve PART
  E2/E4 still pass.
- (b) Down-ladder robust-floor re-seat on demote: UNCHANGED. `inband_seat_robust_ring_floor` /
  `force_resize_capture_ring` GROW the ring to the floor (floor dominates buffer_Nsymb); the sibling
  Nofdm-preserve only holds the geometry invariant, never blocks the grow. F-tests + down-ladder test pass.
- (c) Legacy: both functions are reached ONLY via `inband_rate_feature_enabled()`-gated paths; off →
  neither runs → byte-identical. On the inband path, when `ofdm.gi` is NOT stale the preserved value
  EQUALS the old recompute → no behavior change except in the stale window the bug lived in.

## §16 CORRECTION — §15's "live Nofdm drift" theory is FALSIFIED; the residual is a CLEAN-LOCK decode-bit mismatch (NOT geometry)

### §16.1 What §15 mis-read (VERIFIED — `_probe/fix2001.arqlog`, the §15 binary's own diagnosis run)

§15.1's "smoking gun" log mis-attributed a THROWAWAY instance's numbers to the LIVE RSP shrink. The
second `[PHY] Loading configuration 0 (was -1)` at the cross is NOT a live re-load: it is the throwaway
`cl_telecom_system tmp` constructed in `cl_arq_controller::inband_natural_ofdm_buffer_nsymb()`
(arq_common.cc:3895) — called from `inband_finalize_ofdm_adopt_ring` (arq_common.cc:4504) purely to
compute `natural_nsymb` for a LOG string. `tmp` starts at `current_configuration=-1` (ctor,
telecom_system.cc:159) → its load logs "(was -1)" → its `init()` recomputes `tmp.data_container.Nofdm`
and prints the throwaway's own `[PHY-SWITCH] init() done (... Nofdm=310 buffer_Nsymb=212)`. `tmp`
DESTRUCTS at arq_common.cc:3898 — it never touches the LIVE `data_container`.

PROOF the live ring is CORRECT: in the SAME `_probe/fix2001.arqlog` the live shrink flag log reads
`[INBAND-RX] HINGE-1 OFDM-RING flag: CONFIG_0 ring=217 buf_min=0 shrunk=1` (L23581 / T+0194.112). The
live ring is **217**, not the throwaway's 212; the live decode runs at **Nofdm=292**, not 310. Every
live `[OFDM-SYNC] coarse:` line shows `count=292 bufNsymb=217 Nsymb=48` — the geometry §15 claimed was
"drifted to 310" is in fact PRISTINE on the live path. §15 is therefore a **NO-OP** (confirmed: lwp2001
pre-§15 and fix2001 post-§15 both deliver IDENTICAL 53 B / final ROBUST_2 / deep_stall=true).

### §16.2 The REAL residual (VERIFIED — working-vs-broken, same `_probe/` runs)

The CONFIG_0 under-decode is a **clean-lock decode-bit mismatch**, not geometry. Side-by-side at WGN:40
CONFIG_0, IDENTICAL PHY params (`M=2 LDPC_rate=0.062 Nc=50 Nsymb=48 nBits=1600 Nofdm=292 ring=217`):

| metric | LEGACY (`_abrun/legacy.arqlog`) | REDESIGN (`_probe/fix2001.arqlog`) |
|---|---|---|
| coarse metric | 0.998 | 0.997 (also reaches 0.999) |
| noise_variance_estimate (var) | 0.024–0.037 | 0.018–0.037 (≥ as good) |
| mean_H | 0.98 | 0.98–0.99 |
| LDPC iter | 0 | 0 |
| outcome | **13× OFDM-OK, 0 FAIL** | **0 OK, 198× OFDM-FAIL crc=0xC7C3** |

The redesign achieves a PRISTINE lock (tight constellation var 0.018–0.037, mean_H 0.98–0.99, coarse
0.997) yet LDPC converges at `iter=0` to a self-consistent codeword whose CRC16 is a CONSTANT 0xC7C3 for
every one of 198 frames — across 7+ DIFFERENT anchor delays (13255, 22599, 30775, 41287, 47127, 75159…).
A constant CRC across different delays RULES OUT a timing/FFT-window offset (that yields delay-varying
garbage). Legacy decodes the IDENTICAL transmitted burst byte-correct. The `var=81.11` SKIP-VAR frames
(75 of them) are a SEPARATE set (mis-anchored slides while the re-aired burst slides); the load-bearing
failure is the 198 CLEAN-lock CRC fails that SHOULD have decoded.

RULED OUT as the cause (each checked): geometry/Nofdm (live = 292/217, identical to legacy); PHY config
params (identical active line); descrambler `bit_energy_dispersal_sequence` (re-seeded deterministically
per-load at telecom_system.cc:5218–5222, identical CMD vs RSP; throwaway shares the process-global RNG
(`rng_own_=false`, :56/:65) but the live RSP does NOT regenerate its sequence after the throwaway runs).

### §16.3 Direction for the real fix (NOT YET IMPLEMENTED — [?])

A clean, tight, well-equalized constellation that LDPC-converges (iter=0) to a CRC-failing codeword on a
CLEAN channel, with a delay-INVARIANT constant CRC, points to a FIXED bit transform that differs between
the redesign's tag-follow demod and legacy: a candidate is a **constellation phase-reference / BPSK
polarity** resolved differently by the redesign's fresh FULL-anchor re-search (predict-verify bypassed,
telecom_system.cc:1407) vs legacy's locked predict-verify, or a data-carrier interleaver/deframe phase
that pilots tolerate (periodic) but data carriers do not. The §15/§14/#1c-#1e geometry work is COMPLETE
and CORRECT (the live ring IS 217/292); building MORE geometry symmetry would be a 5th ghost-chase.

### §16.4 §5 audit note

No code changed in §16 (correction + re-direction only). The §15 commit 17fe801 telecom_system.cc
Nofdm-preserve is a harmless no-op on the live path (it preserves a value that was already correct) and
is byte-identical off the inband path; it can stay or be reverted — it neither helps nor hurts the
residual. The fix-before/fix-after must target the DECODE-BIT mismatch on the live tag-follow path, not
the (already-correct) capture-ring geometry.

## §17 ROOT CAUSE FOUND + FIXED — the CONFIG_0 clean-lock CRC-fail is a DESCRAMBLER wipe (VERIFIED)

§16 falsified the geometry theory and re-directed to a "clean-lock decode-bit mismatch." §17 is the
VERIFIED root cause and the fix.

### §17.1 The discriminator (VERIFIED — bit-level instrumentation, runs `_residual/bd3..bd5`)

A `MERCURY_BITDIAG`-gated probe dumped, at each clean CONFIG_0 lock, the RX pre-LDPC LLRs vs candidate
transforms and the TX-truth message bytes. Findings, all EXACT-MATCH across every frame:
- POLARITY (global LLR sign flip) decode → iter=101, crc≠0 ⇒ **NOT a polarity/phase flip** (falsified).
- `RX_decoded_message == TX_message XOR descrambler_sequence(seed=0)` — EXACT for all frames, all
  batch seqs (the seq counter in byte 3/5 tracks through). The XOR mask `3D 6B D0 78 5C 2F 95 B8 …`
  equals the seed-0 `bit_energy_dispersal_sequence` byte-for-byte.
- Direct dump of the RSP's LIVE `bit_energy_dispersal_sequence` at CONFIG_0 decode = **ALL ZEROS**
  (`00 00 …`), while the CMD's TX sequence = the correct `3D 6B D0 …`. ⇒ the RSP descrambles with an
  all-zero sequence (a no-op) → recovers `M XOR S` → CRC fails with the constant 0xC7C3, iter=0.

### §17.2 The mechanism (VERIFIED by code read + the all-zero dump)

`init()` generates the descrambler AFTER its `data_container.set_size` (the `ts_srandom(seed)`+draw
loop). The inband HINGE-1 ring-shrink `force_set_capture_ring_natural()` (and its sibling
`force_resize_capture_ring()`) call `data_container.set_size()` DIRECTLY to resize the capture ring.
`set_size` CDELETE+reallocates `bit_energy_dispersal_sequence` (data_container.cc:243→145, `new int[N_MAX]`
= fresh ZEROED pages) — and NEITHER helper regenerated it. So after the shrink the RSP's descrambler is
all-zero. LEGACY never calls the shrink (inband-only), so its descrambler (from init) survives → decodes
+ climbs 0→16. This is the SAME throwaway-vs-live disease family as §14/§15 (a direct set_size that
skips an init side-effect), but it wipes the DESCRAMBLER, which §15's Nofdm-preserve never addressed —
why §15 was a no-op (§16).

### §17.3 The fix

Factored the init descrambler regen into `cl_telecom_system::regenerate_bit_energy_dispersal_sequence()`
and call it after the `set_size` in BOTH ring-resize helpers (and from init, unchanged behavior). Now any
`data_container.set_size` is followed by a descrambler regen. FAIL-BEFORE knob
`MERCURY_DESCRAMBLER_REGEN_DEFEAT=1` skips the regen (reproduces the all-zero bug on the same binary).

### §17.4 LIVE-PATH test (fail-before / passes-after, NOT an isolation unit)

`test_inband_descrambler_survives_ring_shrink()` (`--test-inband-descrambler-survives-shrink`, in master
`--test`) drives the LIVE load(ROBUST_0)→seat-floor→load(CONFIG_0)→`inband_finalize_ofdm_adopt_ring`
(=force_set_capture_ring_natural) path, then does a REAL scramble(M)^descramble round-trip with the RSP's
post-shrink sequence vs the TX reference. arm 0 (fix): descrambler NON-ZERO, EQUALS the TX seq, round-trip
== M. arm 1 (DEFEAT): all-zero, round-trip == M^S (the 0xC7C3 floor). Master `--test` exit 0.
LIVE SIM (`_residual/fix.arqlog`, MERCURY_INBAND_RATE=1 WGN:40): RSP descr_seq now `3D 6B D0 …`,
**CONFIG_0 OFDM-OK=5 (was 0), OFDM-FAIL=0 (was 198)**, DATA_LONG frames RX-BATCH-SEQ deliver.

### §17.5 §5 AUDIT — descrambler vs every set_size producer

Producers of `bit_energy_dispersal_sequence`: init() (after its set_size) + NOW both ring-resize helpers.
Invalidator: ANY `data_container.set_size` (reallocs the array zeroed). Audited ALL 6 `data_container.set_size`
callers in telecom_system.cc — 2 in init() (regen follows), 2 in force_resize_capture_ring (regen added),
2 in force_set_capture_ring_natural (regen added). No other producer-invalidator. Consumers (TX scramble
728, RX descramble, turbo scoring, ZF-SNR re-encode) all read the now-valid sequence. Legacy path never
calls the helpers → byte-identical.

### §17.6 RESIDUAL (honest)

The descrambler fix makes CONFIG_0 DECODE + DELIVER (OFDM-OK + RX-BATCH-SEQ). The seed-2001 realtime run
still showed final ROBUST_2/53 B because CONFIG_0 was reached LATE (~T+167 of a 240 s run) with no climb
runway; this is a SEPARATE climb-latency/timing layer, not the decode bug. A WGN:40 A/B with adequate
runway is needed to confirm climb-to-legacy. [?] Next: the climb behaviour now that the forward decode works.
