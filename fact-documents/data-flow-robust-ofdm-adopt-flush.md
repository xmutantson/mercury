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

### §17.6 RESIDUAL (honest) — a SEPARATE climb/reliability layer remains

The descrambler fix makes CONFIG_0 DECODE + DELIVER (VERIFIED: OFDM-OK cfg0=5 vs 0, OFDM-FAIL=0 vs 198,
RXSEQ=3D6BD0.., RX-BATCH-SEQ DATA_LONG frames). It is NECESSARY but NOT SUFFICIENT for the climb. A
WGN:40 redesign run WITH full runway (secs=400) STILL ended ROBUST_2/53 B: with the decode now working,
the session still accrues `DOWN-LADDER reached SESSION_DEAD_BATCHES=3 -> TERMINAL BREAK -> ROBUST_0`
(arq_responder/down-ladder). So CONFIG_0 decodes SOME batches but not reliably enough to avoid the
3-consecutive-total-loss dead-streak -> terminal BREAK -> demote. This is a DISTINCT layer (forward
acquisition reliability / the re-aired-burst sliding-window miss of the fix-#1c family, or the dead-batch
streak threshold), NOT the descrambler decode bug this section fixed. [?] Next: characterize why, with
the descrambler fixed, the CONFIG_0 batch DECODE RATE still trips SESSION_DEAD_BATCHES (the climb layer).


## §18 The climb-killer: PARTIAL-batch SACK-turnaround feeds a FALSE dead-batch BREAK (VERIFIED root; clean fix NOT YET found — REPORT+STOP)

§17 fixed the descrambler so CONFIG_0 DECODES; §17.6 flagged the session still BREAKs to ROBUST_2
before climbing. §18 is the VERIFIED root of that BREAK. A first fix attempt was FALSIFIED (§18.3); the
clean fix is still open — reported per CLAUDE.md §2 / the coordinator's "stop if the root isn't clean."

### §18.1 The VERIFIED sequence (`_residual/v18.arqlog`, descrambler-fixed redesign, seed 2001)

1. CONFIG_0 batch_seq_id=3 receives **5 of its 6 frames** (id 1–5, seq 1–5; frame 0/seq 0 was missed) —
   all clean OFDM-OK at ring=217/Nofdm=292, var~0.036 (T+0163.1 … T+0165.5).
2. T+0165.741 `[ACK-GATE] SACK: received 5/6 (expected 6)` → the RSP dispatches a PARTIAL SACK
   (`bitmap=0x3e`, frame 0 missing) and TRANSMITS it as an MFSK reverse-ACK (`TX-MFSK-ACK-SACK`, ~1.1 s
   of audio, T+0165.7 … T+0166.6).
3. During that partial-SACK TX TURNAROUND the RSP is not capturing the forward CONFIG_0 burst; when it
   resumes, the down-ladder bank builds (`Loading configuration 100/101/102/0 (was -1)` trial decoders)
   and the forward read finds the burst INCOMPLETE/absent (mid re-air).
4. The inband down-ladder fires 3× in ~0.45 s (`DOWN-LADDER: no config in window [0..3] decoded
   (1 attempts)` → `total-loss batch 1/3 → 2/3 → 3/3`) → `SESSION_DEAD_BATCHES=3 → TERMINAL BREAK →
   ROBUST_0` (arq_common.cc:4783/4789). The session demotes ROBUST_2/53 B and never climbs.

So the dead-batch streak is fed by the **PARTIAL-batch → reverse-SACK turnaround → cannot-re-acquire**
window, NOT a clean total-loss. A CONFIG_0 batch is ~12 s; 3 ticks in <0.5 s are turnaround RX-loop
passes mis-counted as 3 dead BATCHES. Legacy never runs the inband down-ladder, so it never false-ticks
→ it climbs 0→16 (geomfix2 A/B: legacy reaches CONFIG_16/7647 B; all 5 REDESIGN samples
peak=CONFIG_0/53–77 B/ROBUST_2, rc=0 bounded=virtual_secs — a STABLE result, not a harness artifact).

### §18.2 §5 / channel / harness notes (VERIFIED)

- geomfix2 ran `--snr 40` (the sim's --snr IS SNR3k); a very clean channel where legacy reaches
  CONFIG_16. The prior `_abrun/legacy.json` 5645 B baseline was `--snr 12` — a DIFFERENT channel, NOT
  the comparable. The right comparable for "does the redesign climb" is the SAME --snr 40, both arms.
- LEGACY-ARM HARNESS NOTE [?]: geomfix2 LEGACY shows `bounded=proc_died` (LEGACY 1: 7647 B/CONFIG_16 —
  climbed the FULL ladder [0,4,10..16], md5 ok — then a process exited cleanly, which the monitor
  LABELS proc_died at sim_arq_channel.py:863–866) and one `final=None`/0 B early death. So the legacy
  "missing from aggregates" is mostly a COSMETIC monitor-labeling issue (a clean CMD exit counted as a
  death) plus genuine occasional early death — worth a separate harness fix so both-arms aggregates are
  clean. The legacy CLIMB capability itself is confirmed.

### §18.3 ~~Fix attempt A (frame_data_missing gate) — FALSIFIED~~

~~Clear rx_fresh_window_decoded_this_pass on a receive_stats.frame_data_missing pass.~~ FALSIFIED on the
live path (`_residual/v18.arqlog`): the validation run STILL ended ROBUST_2/53 B with the same 3-tick →
BREAK. The gate-firing pass's PRIMARY decode does NOT carry frame_data_missing — during the partial-SACK
turnaround the staged forward window is empty/noise (a genuine no-frame pass), and the "frame incomplete"
lines near the tick are the DOWN-LADDER's own CONFIG_0 trial (count=310, the wrong-geometry trial bank),
not the primary. So gating the primary on frame_data_missing is a no-op for this trigger.

### §18.4 Direction for the real fix (OPEN — needs design, not a point patch)

The down-ladder dead-batch tick must not count turnaround passes that follow a JUST-SENT reverse-SACK on
a PARTIAL-but-progressing batch. Candidate directions (each needs a live-path fail-before/passes-after):
(a) suppress the down-ladder / dead-batch tick for a bounded window AFTER the RSP transmits a partial
SACK (the turnaround is expected, not a loss); (b) only tick the dead-batch streak when the forward batch
made ZERO progress since the last tick (a 5/6 batch that is awaiting one retransmit is not "dead"); (c)
require the dead-batch ticks to span ≥1 real batch PERIOD (rate-limit), so 3 sub-second turnaround passes
cannot reach SESSION_DEAD_BATCHES. (b)/(c) look closest to the true invariant ("dead" = no forward
progress across real batch periods), but each touches the down-ladder ↔ SACK ↔ dead-batch cross-layer
state and must be designed + audited, not patched. REPORTED + STOPPED here per §2.

## §19 DESIGNED FIX — tie SESSION_DEAD_BATCHES to REAL batch periods + ZERO-PROGRESS (the climb-killer fix)

§18 VERIFIED the climb-killer (a 5/6 batch + partial-SACK turnaround false-counts 3 sub-second RX-loop
passes as 3 dead batches → BREAK) and FALSIFIED the frame_data_missing point-patch. §19 is the designed
cross-layer fix.

### §19.1 The fix — a production CLASSIFIER (`cl_arq_controller::inband_deadbatch_classify`)

The unconditional `inband_session_dead_batches++` at the down-ladder no-decode site is replaced by a
classifier that decides PROGRESS_RESET / RATE_LIMITED / TICK and applies the streak side-effects:
- **(1) FORWARD PROGRESS** — a forward DATA frame decoded since the last classify
  (`inband_total_data_frames_rx`, a NEW session-monotonic counter ++'d at the same confirmed-storage
  point `batch_rx_frame_count` advances, arq_responder.cc) → the link is ALIVE → RESET the streak +
  arm the batch-period clock (so the immediately-following turnaround re-fires are rate-limited, not
  ticked at streak 0). The 5/6-batch-awaiting-one-retransmit case.
- **(2) TIME RATE-LIMIT** — zero progress but `< one REAL BATCH PERIOD` of VIRTUAL time
  (`receiving_timeout`, floored 3000 ms / capped 20000 ms) elapsed since the last classify → a
  sub-second partial-SACK-turnaround re-fire → do NOT tick. The rate-limit is TIME-based, NOT bsi-based:
  a real total loss decodes NO frame so the bsi never advances; a bsi rate-limit would suppress the
  LEGITIMATE BREAK (the §5 audit catch). A `cl_timer` advances regardless of decode.
- **(3) TICK** — zero progress AND ≥ one batch period (or the first ever pass) → a genuine dead batch;
  advance the streak (the caller checks SESSION_DEAD_BATCHES → BREAK, mechanism unchanged).
Net: the streak counts CONSECUTIVE ZERO-PROGRESS REAL BATCH PERIODS, not RX-loop passes. FAIL-BEFORE
knob MERCURY_INBAND_DEADBATCH_RATELIMIT_DEFEAT=1 forces TICK every pass (the pre-§19 false BREAK).

### §19.2 §5 CROSS-LAYER AUDIT — session-level BREAK state (recovery MUST survive)

Producer: the down-ladder no-decode classify (→ tick). Consumer: TERMINAL BREAK (arq_common.cc:4789).
New state (`inband_total_data_frames_rx`, `inband_dead_tick_frames_snap`, `inband_dead_tick_timer`,
`inband_dead_tick_timer_armed`) reset at: session start, terminal BREAK, resync success. Walk:
- (a) REAL deep-SNR total loss (zero progress over successive real periods): first pass TICKs, re-fires
  rate-limited, after each real batch period a fresh TICK → reaches SESSION_DEAD_BATCHES → BREAK.
  **Recovery PRESERVED** (regression test CASE B asserts ticks==limit → BREAK).
- (b) partial-then-recovering 5/6 batch: progress → RESET, no false BREAK; the retransmit completes it.
- (c) partial-SACK turnaround sub-second passes: rate-limited (time) and/or progress-reset → no tick.

### §19.3 LIVE-PATH test (fail-before/passes-after + recovery-preserved)

`test_inband_deadbatch_progress` (`--test-inband-deadbatch-progress`, in master `--test`) drives the
PRODUCTION classifier `inband_deadbatch_classify()` (the EXACT decision the down-ladder caller makes)
with the EXACT production state — NOT a fake, and not the synthetic-audio-fragile full down-ladder.
CASE A (5/6 batch + 3 back-to-back turnaround passes): FIX → progress resets + re-fires rate-limited →
NO false BREAK; DEFEAT → 3 unconditional ticks → BREAK (the 53 B floor). CASE B (true total loss): a
sub-second re-fire is rate-limited (live cl_timer exercised), and zero progress across `limit` real
batch periods STILL ticks once per period → BREAK (recovery NOT regressed). Master `--test` exit 0.

### §19.4 END-TO-END (VERIFIED, _residual/v19.arqlog, seed 2001 --snr 40)

The §19 fix WORKS for its target: **ZERO RX-side TERMINAL BREAK** (was the climb-killer), CONFIG_0 now
SUSTAINS, rx **189 B** (was 53 B), with 125 rate-limited turnaround re-fires and 0 false total-loss ticks.
BUT a SEPARATE **CMD-side** demote still blocks the climb: the CMD accrues `[CMD] [BREAK] Block failure
#1/#2/#3 at config 0 (threshold=3)` and `INBAND-NOBREAK Class-A degradation: demoting 0 -> 102` (CONFIG_0
-> ROBUST_2 via the CONFIG_TAG). So the redesign reaches final ROBUST_2 / peak CONFIG_0 still — but via a
DIFFERENT, CMD-side path (the reverse-SACK round-trips not confirming clean batches), NOT the RX dead-batch
BREAK §19 fixed. This is the SYMMETRIC SIBLING of §18/§19 on the COMMANDER side (the same partial-batch +
turnaround pattern counted as `cmd_inband_session_dead_batches` / block-failures). §19 is a correct, tested
improvement (RX climb-killer removed, 189 vs 53 B); the CMD-side block-failure demote is the NEXT layer.
[?] Next: the CMD-side block-failure / `cmd_inband_session_dead_batches` analog of the §19 zero-progress
rate-limit (arq_commander.cc:3186, `[CMD] [BREAK] Block failure ... at config 0`).

## §20 CMD-SIDE block-failure demote — NOT the §18/§19 sibling; it is a GENUINE marginal-link response (DO NOT mirror §19 — REPORT+STOP)

After §19 removed the RX-side false BREAK, the redesign still ends ROBUST_2 via a CMD-side demote
(`[CMD] [BREAK] Block failure #1/#2/#3 at config 0 (threshold=3)` → `Class-A degradation: demoting
0 -> 102`, arq_commander.cc emergency_nack_count :4720 / threshold :4740). The hypothesis was that this
is the SYMMETRIC commander-side sibling of §18/§19 (a false-count of partial-SACK-turnaround re-fires).
VERIFIED FALSE.

### §20.1 The block failures are REAL per-batch-period, not sub-second turnaround re-fires (VERIFIED `_residual/v19.arqlog`)

The config-0 block-failure timestamps that reached the threshold: #1 T+0332.051, #2 T+0345.187, #3
T+0358.401 — **~13 s apart = ONE REAL CONFIG_0 BATCH PERIOD each** (vs §19's 3 sub-second re-fires).
emergency_nack_count ALREADY resets on any data-ACK (forward progress, arq_commander.cc:2451 etc.) — and
it DID reset between an earlier #1 (T+0305) and the threshold streak (nAcked_data advanced 37→43). So the
CMD-side counter is ALREADY zero-progress + real-period gated by construction; a §19-style TIME rate-limit
would change NOTHING (3 genuine zero-progress real periods → the §19 classifier itself would tick 3× and
demote).

### §20.2 The real cause — the reverse confirmation genuinely does not arrive (VERIFIED)

Every threshold block failure is preceded by `[CMD] [CMD-ACK-PAT] Timeout: no ACK detected,
peak_matched=4-5/7 peak_metric=0.3-0.5` — the reverse MFSK ACK/SACK correlator matching only 4-5 of 7
(sub-threshold = NOISE, no real ACK). In the window before #3 the RSP was `connection_status:Receiving`
with `receiving_timer` climbing 6643→8176 ms and sent NO `TX-MFSK-ACK-SACK` — i.e. NO batch completed at
the RSP to ACK, so no SACK was sent, so the CMD's reverse poll timed out. CONFIG_0 batches complete
INTERMITTENTLY (some periods deliver — nAcked 37→43 — others stall with the RSP waiting), and after 3
consecutive real periods with no completed/confirmed batch the CMD demotes. That is a GENUINE marginal-
link response, not a false count.

### §20.3 Verdict — DO NOT mirror §19 here (REPORT + STOP per §2)

Mirroring §19's classifier onto emergency_nack_count would be a NO-OP (the failures are already real-
period-paced) OR, if forced to suppress them, would MASK a genuine reverse-ACK-decode / forward-batch-
completion failure — the "tune thresholds to make failing modes not fail in logs" anti-pattern CLAUDE.md
forbids. The §19 RX-side fix STANDS (189 B vs 53 B, 0 RX BREAK, CONFIG_0 sustains). The remaining climb
blocker is a DEEPER layer: the CONFIG_0 forward-batch-completion / reverse-MFSK-ACK reliability, NOT a
false-count. No code change in §20.

### §20.4 Honest is-this-the-last read

NO — and the next blocker is NOT a quick point-fix. The chain so far: §17 descrambler (CONFIG_0 decodes)
→ §19 dead-batch (CONFIG_0 sustains, no false RX BREAK) → §20 the CMD-side demote is a GENUINE marginal-
link symptom (intermittent CONFIG_0 batch completion + sub-threshold reverse-MFSK-ACK). The diagnosis-
flagged ROBUST_DWELL liveness false-fire and the slow ~100 s hailing climb up-ladder are plausibly part
of THIS same forward-acquisition / reverse-confirmation reliability layer (the RSP's `receiving_timer`
growing to 8 s without a batch completing is a dwell/liveness signature). The right next step is a
DESIGNED investigation of the CONFIG_0 forward-batch-completion + reverse-ACK reliability (why a batch
intermittently fails to complete at the RSP / the reverse MFSK ACK matches only 4-5/7), NOT another
counter-guard. [?]

## §21 CONFIG_0-START robust-floor over-seat — the uncovered sibling of FIX #1e (oversized OFDM ring, 0 forward DATA decode)

VERIFIED ROOT (false-break hunt, localized — all adopt-path siblings #1a–#1e exonerated): on a session that
STARTS at CONFIG_0 (already OFDM-locked, no prior robust→OFDM adopt), the responder's
`inband_seat_robust_ring_floor()` (arq_common.cc:4205) grows the CONFIG_0 OFDM capture ring from its NATURAL
217 symbols to the ROBUST_0 floor (~804). The suppression guard at arq_common.cc:4216
(`if(inband_ofdm_acq_ring_shrunk && is_ofdm_config(current_configuration)) return;`) never fires because its
flag `inband_ofdm_acq_ring_shrunk` has a SINGLE setter — arq_common.cc:4803, inside
`inband_finalize_ofdm_adopt_ring` (the robust→OFDM ADOPT branch). A CONFIG_0-start session never runs an
adopt, never latches the flag → the floor seat oversizes the ring → every CONFIG_0 OFDM preamble lands at
the ring TAIL beyond `upper_bound = buffer_Nsymb - frame_symb` → `[RX-TIMING] OFDM beyond-bounds` → 0 forward
DATA decode. Evidence: redesign (MERCURY_INBAND_RATE=1) rx_bytes=0 / nReceived_data=0; legacy (unset) ring
stays 217 and delivers 42–54 CONFIG_0 OFDM frames. This is the EXACT failure FIX #1e fixed — but only on the
adopt path; the flag-latch was the adopt path's, so the CONFIG_0-start path was uncovered.

### §21.1 The fix — a NARROW geometry guard scoped to the LOWEST OFDM config (CONFIG_0)

Principle (per CLAUDE.md §5 "constrain the producer"): only ENLARGE the ring when actually below OFDM
geometry that NEEDS the larger physical window. Add a guard at arq_common.cc (the seat, §4216) that SKIPS
the floor seat when `current_configuration == CONFIG_0` AND `buffer_Nsymb_min == 0` AND the live ring
`buffer_Nsymb` is at/below the natural OFDM size for CONFIG_0 (a small +8-symbol slack absorbs the documented
212-vs-217 throwaway-vs-live skew, §14; gating primarily on `buffer_Nsymb_min == 0` makes it skew-immune).

CRITICAL SCOPE NARROWING (corrected during implementation — an initial `is_ofdm_config(current_configuration)`
guard was TOO BROAD and regressed the sibling ADOPT-PRESERVE/DELIVER tests that seat the floor at CONFIG_1/
CONFIG_10): the floor grow EXISTS so the OFDM-tier down-ladder can read a frame from a LOWER (more-robust,
larger-frame) rung whose decode needs more PHYSICAL ring than `current_configuration`'s natural ring — the
down-ladder copies the primary snapshot into each trial decoder and ZERO-PADS only the LENGTH
(arq_common.cc:4503), it cannot reconstruct samples a too-small primary ring never captured. So:
- At a HIGH OFDM config (e.g. CONFIG_10, natural ~128) reaching DOWN to CONFIG_0 (natural ~217), the grow is
  GENUINELY needed → KEEP it.
- CONFIG_0 is the LOWEST OFDM config (FULL_CONFIG_LADDER idx 3; below it are only ROBUST rungs). A fresh OFDM
  lock SKIPS robust trials entirely (FIX #1d, arq_common.cc:4481), so at CONFIG_0 the grow serves NO
  down-ladder purpose and only breaks CONFIG_0's own continuously-re-aired acquisition (the CMD re-airs
  CONFIG_0, pinning the freshest preamble at the oversized-ring tail beyond upper_bound — §10).
Hence the skip is scoped to `current_configuration == CONFIG_0` only. The adopt-path `inband_ofdm_acq_ring_
shrunk` flag-guard above is unchanged and complementary (it suppresses the per-pass re-seat at ANY OFDM rung
once the adopt has shrunk the ring); the new guard covers the CONFIG_0-START case the flag never latched.

### §21.2 §5 CROSS-LAYER AUDIT — capture ring buffer_Nsymb / buffer_Nsymb_min (the shared state)

1. PRODUCERS of buffer_Nsymb / buffer_Nsymb_min:
   - `inband_seat_robust_ring_floor()` (arq_common.cc:4237 set min; :4256 force_resize_capture_ring) — THE seat being constrained.
   - `inband_finalize_ofdm_adopt_ring()` (arq_common.cc:4769 force_set_capture_ring_natural -> min=0; :4803 flag set) — adopt-path shrink (#1c/#1e).
   - `init_monitor_decoders()` (arq_common.cc:1825/1873) — passive-monitor primary seat (passive_monitor only; the responder seat is gated !passive_monitor at arq_responder.cc:570).
   - `load_configuration` -> `data_container::set_size` (data_container.cc:147-163) — natural sizing, honors buffer_Nsymb_min.
2. CONSUMERS of the ring geometry:
   - OFDM preamble anchor search upper_bound (telecom_system.cc; `upper = buffer_Nsymb - frame_symb`) — the BROKEN consumer (oversized ring -> tail preamble beyond upper -> 0 decode).
   - OFDM-tier down-ladder `inband_try_down_ladder_on_decode_fail` (arq_common.cc:4974): reads the STAGED buffer for `signal_period = Nofdm*buffer_Nsymb*interp`, and `inband_down_ladder_resync` ZERO-PADS each trial decoder up to the robust-rung buffer (:5003 comment). FIRED ONLY when `is_ofdm_config(current_configuration)` (arq_responder.cc:649). => does NOT need the ring physically grown to 804; natural 217 + zero-pad is correct.
   - ROBUST-tier follow `inband_detect_follow_from_capture` (arq_common.cc:3755): reads the CAPTURED robust-frame tail. FIRED ONLY when `!is_ofdm_config(current_configuration)` (arq_responder.cc:608). => DOES need the ring grown to hold a full slow MFSK frame (can't be zero-pad-reconstructed; the samples must be physically captured). This is the LEGITIMATE consumer of the grow.
3. VALID STATES of buffer_Nsymb_min: 0 (natural OFDM sizing, the CONFIG_0-start default) | >0 (robust floor seated). Default-init on a CONFIG_0-start session = 0 (load_configuration(CONFIG_0) sizes 217, never raises min).
4. INVARIANT consumers assume: at an OFDM config the ring is the NATURAL OFDM size so the preamble search upper_bound contains a tail preamble. The robust-floor seat VIOLATED it on the CONFIG_0-start path.
5. WHAT THE FIX CHANGES (NARROWED to current==CONFIG_0): the seat is a NO-OP only when current==CONFIG_0 AND min==0 AND ring<=natural+slack — the CONFIG_0-START state the adopt flag never latched. Walk each consumer:
   - preamble search (at CONFIG_0): ring stays 217 -> the continuously-re-aired CONFIG_0 tail preamble fits below upper -> LOCKS (the fix's intent). OK.
   - OFDM-tier down-ladder AT CONFIG_0: window below CONFIG_0 is ROBUST-only, SKIPPED while holding a shrunk OFDM lock (FIX #1d) -> the grow served no purpose there -> natural ring sufficient. OK.
   - OFDM-tier down-ladder AT A HIGHER OFDM config (CONFIG_1..16): the fix does NOT skip (current!=CONFIG_0) -> ring still grows to reach LOWER OFDM rungs whose natural ring exceeds current's -> physical samples present. OK (UNCHANGED — this is why the scope had to narrow to CONFIG_0).
   - ROBUST-tier follow: fires only at `!is_ofdm_config` where the fix does NOT skip -> ring still grows for robust capture. OK (unchanged).
   - passive-monitor primary seat: gated !passive_monitor on the seat caller; unaffected. OK.
   The fix removes ONLY the spurious grow at CONFIG_0 (the lowest OFDM rung, where the grow has no consumer); every path that needs the grow (higher-OFDM down-ladder reach + robust-tier follow) is left untouched. No consumer invariant is broken. VERIFIED: the sibling ADOPT-PRESERVE (seats at CONFIG_1) + DELIVER (seats at CONFIG_10) tests PASS unchanged after the narrowing.

### §21.3 LIVE-PATH test (fail-before / passes-after) — `--test-inband-config0-start-ring`

Drives a RESPONDER built directly at CONFIG_0 (no adopt) under MERCURY_INBAND_RATE=1, then calls the
PRODUCTION `inband_seat_robust_ring_floor()` on the receive path (the exact arq_responder.cc:573 caller),
and asserts the ring `buffer_Nsymb` stays at the natural OFDM size (<= natural+slack) AND that a painted
CONFIG_0 preamble still lands within `upper_bound` (the consumer the oversize broke). FAIL-BEFORE arm:
`MERCURY_CONFIG0_RING_GUARD_DEFEAT=1` restores the pre-fix unconditional grow on the SAME binary, so the ring
balloons 217->804 and the preamble lands beyond upper_bound (reproduces the 0-forward-decode geometry).


## §22 — ROOT CAUSE of the WB-cfg0 under-decode under MERCURY_INBAND_RATE: fresh in-band decoder instances never inherit the primary's startup-patched ofdm_gi

### §22.0 The symptom and why §15/§17 did not fix it
With `MERCURY_INBAND_RATE=1` (the trio) a real-audio session at WGN:40 stays stuck at the ROBUST floor:
configs ever seen = {100,101,102}, ~189-253 B delivered, `block_success` 0% at WB CONFIG_0; legacy (flags OFF,
eac0c876) on the SAME audio climbs past cfg0 (configs {0,13,14,...}, ~28 KB). The §15 NOFDM-preserve
(`force_set_capture_ring_natural`) and §17 descrambler-regen fixes are CORRECT and engage on the **primary**
telecom_system (instrumented: `[GIDIAG-PRIMARY] default_gi=0.14062 preserved_Nofdm=292`), but they target a
STALE-LIVE-gi disagreement inside the primary's ring-shrink. They never touch a **second instance class** — the
freshly-constructed in-band decoder objects — which is where the real corruption lives.

### §22.1 ROOT CAUSE (verified)
The primary's PHY geometry comes from `default_configurations_telecom_system`, which `main.cc` patches at
startup: `ofdm_gi` <- `Ngi/256` (main.cc:4896; production 3.0 ms GI => Ngi=36 => gi=36/256=0.14062), plus
`ofdm_Nfft` (3651), `ldpc_nIteration_max` (4825), FIR cutoffs (3547). `load_configuration` copies
`ofdm.gi = default_configurations_telecom_system.ofdm_gi` (telecom_system.cc:11037) and `init()` derives
`Nofdm = Nfft + round(gi*Nfft)` (telecom_system.cc:7206-7208). A fresh `new cl_telecom_system()` /
stack `cl_telecom_system tmp` carries the **constructor default** `ofdm_gi = 54/256 = 0.21094`
(physical_config.cc:37). So an un-inherited CONFIG_0 decoder builds at `Nofdm = 256 + 54 = 310`, not the
production `256 + 36 = 292`. The 18-sample/symbol FFT-window stride error accrues across the frame => post-EQ
variance 4..65 (SKIP-VAR threshold 1.60, telecom_system.cc) => LDPC skipped (`iter=-1`) => 0% block_success at
WB CONFIG_0 => the robust->OFDM cross never sustains => no climb past the ROBUST floor.

Instrumented A/B proof: `[GIDIAG] init() default_gi=0.21094 -> Nofdm=310 (cfg=0)` (fresh in-band instance)
vs the clean primary `Nofdm=292`. Legacy never spawns these in-band decoders, so its decode always runs on the
gi-patched primary => 292 => var=0.0357, iter=0, climbs.

### §22.2 The six (eight call-site) fresh-instance sites — all the same bug class
1. `inband_ensure_down_decoders` — the LOAD-BEARING decode bank: `new cl_telecom_system()` (arq_common.cc:4579)
   + the bank-size probe `cl_telecom_system tmp` (arq_common.cc:4552).
2. `inband_down_window_buffer_nsymb` — window-size probe `tmp` (arq_common.cc:4154).
3. `inband_robust_floor_buffer_nsymb` — robust-floor-size probe `tmp` (arq_common.cc:4189).
4. `inband_natural_ofdm_buffer_nsymb` — natural-OFDM-size probe `tmp` (arq_common.cc:4210).
5. `init_monitor_decoders` — passive-monitor bank: size probe `tmp` (1864) + per-cfg `new` (1876).
6. `reinit_monitor_decoders` — NB/WB-switch rebuild: size probe `tmp` (1923) + per-cfg `new` (1935).
The probes (2-4, and the size probes in 1/5/6) read `buffer_Nsymb`, which itself depends on Nofdm — so a wrong
gi mis-sizes the bank buffer too. The decode sites (1 `new`, 5/6 `new`) demod at the wrong stride. Monitor
decoders are a passive-monitor-only role (a separate code path), but they share the identical root and are
fixed for completeness.

### §22.3 THE FIX — single chokepoint
`cl_arq_controller::inband_inherit_phy_defaults(cl_telecom_system* fresh)` (arq_common.cc:4503) copies the
PRIMARY's whole `default_configurations_telecom_system` struct into the fresh instance BEFORE its
`load_configuration`. Copying the whole struct (trivially copyable aside from one std::string) captures every
startup patch (gi, Nfft, FIR, ldpc iters, carrier_freq) in one place so no future fresh-instance site can
silently drift to the ctor default. Called at all eight sites above. No-op when `telecom_system==NULL` (the
standalone --test path that builds its own primary already pre-sets gi). The next `load_configuration`
re-derives the per-config modulation/rate from the inherited defaults.
FAIL-BEFORE knob: `MERCURY_INBAND_GI_INHERIT_DEFEAT=1` makes the helper a no-op on the same binary (restores
the ctor gi => Nofdm=310). Production never sets it.

### §22.4 §5 CROSS-LAYER AUDIT — `default_configurations_telecom_system` on fresh decoder instances
1. PRODUCERS (writes to a fresh instance's `default_configurations_telecom_system`):
   - ctor `cl_configuration_telecom_system::cl_configuration_telecom_system()` (physical_config.cc:33-121) —
     installs the raw defaults incl. `ofdm_gi=54/256`. This is the value the bug rode in on.
   - `inband_inherit_phy_defaults` (arq_common.cc:4503) — THE NEW producer; overwrites with the primary's
     patched struct. Runs once per fresh instance, immediately before load_configuration.
   - (no other writer touches a fresh in-band instance's default struct before its load.)
2. CONSUMERS (reads of `default_configurations_telecom_system` on these instances):
   - `load_configuration(int)` (telecom_system.cc:11035-11068) copies the OFDM/pilot/FIR/preamble geometry
     into the live `ofdm`/filter objects; `init()` then derives Nofdm/buffer_Nsymb from `ofdm.gi`,`ofdm.Nfft`.
     This is the ONLY consumer on the in-band path; everything downstream reads the DERIVED live geometry.
3. VALID STATES: ctor-default struct (gi=54/256) — the state BEFORE any producer on a fresh instance; or
   primary-inherited struct (gi=36/256 in production) — after the fix's producer. The bug was the consumer
   running on the ctor-default state.
4. INVARIANT consumers assume: a decoder that must demod the SAME OTA frames the primary TX/RX produced uses
   the SAME per-symbol geometry (Nofdm) as the primary. Violated by the ctor-default gi => 310 vs 292.
5. WHAT THE FIX CHANGES: it makes every fresh in-band instance's default struct EQUAL the primary's before
   load_configuration. Walk each consumer:
   - load_configuration/init on the down-ladder decode bank (the load-bearing path): now builds CONFIG_0 at
     292 => correct stride => LDPC runs => the WB cross sustains. FIXED.
   - the size probes (window/floor/natural/bank): now compute buffer_Nsymb at the production geometry, so the
     bank buffer the decoders share matches the real frames. Consistent with the decode sites. OK.
   - monitor decoders (passive-monitor role): now build at the production geometry too; passive-monitor was
     a separate role and not on the trio's failing path, but the change is strictly geometry-correcting. OK.
   No consumer outside load_configuration/init reads the default struct on these instances, so no other
   invariant is touched. The PRIMARY's struct is never modified (the helper writes only the fresh copy), so
   the §15/§17 primary-path fixes are untouched and legacy (feature OFF) is byte-identical (the in-band
   sites are reached only under MERCURY_INBAND_RATE / passive_monitor).

### §22.5 LIVE-PATH test (fail-before / passes-after) — `--test-inband-down-decoder-gi-inherit`
Builds a RESPONDER whose PRIMARY is patched to PROD_GI (36/256) exactly as main.cc does, then drives the
PRODUCTION builder `inband_ensure_down_decoders(idx(CONFIG_0), idx(CONFIG_0))` and reads the CONFIG_0 rung's
installed `data_container.Nofdm` and live `ofdm.gi` — WITHOUT pre-patching the decoder's gi (only the primary
is patched). PASS-AFTER: the rung inherits => Nofdm=292, Ngi=36. FAIL-BEFORE
(`MERCURY_INBAND_GI_INHERIT_DEFEAT=1`, same binary): ctor gi => Nofdm=310. This is the first test that exercises
the un-inherited default; the existing 56 inband units all PRE-SET
`ts_rx->default_configurations_telecom_system.ofdm_gi = PROD_GI` on their throwaway instances (e.g.
arq_responder.cc:8843/8999/9180), which MASKED the production bug. Verified: both arms PASS; full --test
suite reports 68 passed, 0 failed, exit 0.

---

## §VAR-FIX — cfg0 SKIP-VAR storm at the in-band turnaround (investigation 2026-06-28)

### Symptom (faithful real-audio, snd-aloop, WGN:40 seed2001, card0, native ELF)
- **Legacy (inband-off):** rx_bytes=**8192 (delivered_full)**, configs_seen=[0,100,101,102], wb=[0],
  **SKIP-VAR=8**, cfg0 coarse metric 0.998×72 / 0.556×2 → clean cross + full delivery.
- **Redesign (inband-on, +demote-safety):** rx_bytes=**193**, configs_seen=[100,101,102], wb=[],
  **SKIP-VAR=607** vs **OFDM-OK=56**, cfg0 coarse 0.500-plateau ×many interleaved with 0.998 reals.
  Replicated across obsB(161B), dbg(129B), Rc0(193B); identical with the demote-safety DEFEATED (no diff).

### Root: the storm is SPECIFIC to the in-band RX path, NOT inherent to cfg0
The in-band RX runs the full-buffer forward-preamble search DURING the reverse-ACK turnaround gap (when the
CMD is NOT airing a forward burst). The GI/data self-correlation of the idle/echo region peaks at the
Schmidl-Cox half-correlation **metric≈0.500-0.504** (telecom_system.cc s5_coarse early-exit=0.5 lands
exactly here; final accept threshold 0.15 WB admits it). The FFT/pilot window then anchors on a data region →
`noise_variance_estimate`=63-81 (a few 6-7) → SKIP-VAR (ceiling 1.60). The protect-the-lock guard
(arq_common.cc:4797) holds the shrunk-ring cfg0 pin and SKIPS every ROBUST down-ladder trial (129-147 skips),
so the storm cannot escape and the real forward bursts (nv≈0.033, OFDM-OK) are starved/zeroed-advanced.
Legacy avoids ALL of this via the lockstep SET_CONFIG handshake: its RX only searches inside forward-burst
windows, so it sees 0.998 reals and ~zero plateau.

### REFUTED: a coarse-metric structural floor (the obvious Fix A)
Hypothesis: reject the 0.50 plateau with a WB-OFDM coarse floor (~0.55), since "real preamble≈0.99, plateau≈0.50".
**Refuted by deterministic --test evidence** (test_ofdm_fine_timing_magnitude_cfo_cliff, FTR_DEBUG): valid WB
CONFIG_0 preambles at SNR3k=8 dB with residual CFO=30 Hz produce coarse metric **0.426-0.609** WITH CORRECT
TIMING (delay err ≤2) and survivable mean_H (0.307-0.335). A real CFO-degraded preamble is INDISTINGUISHABLE
from a GI-plateau by coarse metric alone — a 0.55 floor dropped coarse_ok from 10/10 to 2/10 (a genuine
weak-signal regression). The metric is NOT cleanly bimodal once CFO is present; the SKIP-VAR/mean_H gates
downstream ALREADY (correctly) reject the plateau (nv 63-81) while admitting reals (nv 0.033). The problem is
the storm's COST + the pin starving the real bursts, NOT a missing coarse gate. **Do NOT ship a coarse floor.**

### The real next layer (NOT a localized variance fix)
Gate the in-band RX forward-preamble search to forward-burst windows (the lockstep discipline legacy has) so it
never searches the RSP's own turnaround gap. This crosses RX/turnaround/PHY layers (the in-band data-plane), is
the known ROBUST→WB climb binding constraint (memory faithful_beat_vara_climb_binding), and needs the full §5
cross-layer audit before coding — it is NOT the cfg0 "live-decode variance" the original diagnosis framed.

### §VAR-FIX-DEMOTE — the shipped no-regress backstop (demote-safety watchdog)
`inband_deliver_stall_watchdog()` (arq_common.cc): on each down-ladder decode-fail pass, evaluated ONCE per
real batch period BEFORE the no-attempt guard. While holding a shrunk OFDM pin, if the DELIVERED batch id
(rsp_last_delivered_batch_seq_id) does not advance across `limit` (default 3) consecutive batch periods, it
RELEASES `inband_ofdm_acq_ring_shrunk` so the down-ladder can demote a config that cannot DELIVER.
- **Producers of the pin:** inband_finalize_ofdm_adopt_ring (set); inband_seat_robust_ring_floor (clear on
  leaving OFDM tier); BREAK→ROBUST_0; **this watchdog (clear on sustained no-delivery)**.
- **Consumers:** the protect-the-lock guard (arq_common.cc:4797) + inband_seat_robust_ring_floor (:4487).
- **Invariant preserved:** the down-ladder still requires a real CRC/LDPC pass to ADOPT (INV-S4-1) — releasing
  the pin cannot adopt a guess; it only stops the guard SKIPPING robust trials.
- **Why it does NOT fire on the WGN:40 storm:** the storm is a slow TRICKLE, not a hard stall — the delivered
  id keeps advancing (2→3→4→5), so the watchdog correctly re-arms and never releases. It is the backstop for a
  TRUE hard pin (zero delivery ≥3 periods), proven by test_inband_deliver_stall_releases_pin (FIX/DEFEAT/
  PRODUCTIVE arms). Additive, inband-scoped, legacy byte-identical. Full --test 68/0.

---

## §VAR-FIX-2 — RX FORWARD-SEARCH GATE: suppress the in-band coarse search during the reverse-ACK turnaround gap (implemented 2026-06-28)

§VAR-FIX VERIFIED the root (the in-band RX storms the forward-preamble coarse search during the RSP's own
reverse-ACK turnaround gap) and REFUTED a coarse-metric floor. §VAR-FIX-2 is the implemented gate: the RX
forward search is now suppressed for the turnaround gap and resumes at the forward-burst window — the
lockstep discipline legacy has.

### §VF2.1 The mechanism the storm rides (VERIFIED — code read)

`frames_to_read` (ftr) IS the forward-burst-window gate. The capture thread (audioio.c:1455) decrements ftr
by 1 per captured symbol_period; `cl_arq_controller::receive()` (arq_common.cc:12386) stages a FRESH snapshot
and runs the coarse search ONLY when `ftr == 0` (when `ftr != 0` it early-exits at :13950 — no snapshot, no
search, no SKIP-VAR). So a positive ftr SUPPRESSES the search for ftr symbols; ftr==0 OPENS the window.

The bug is two coupled re-arms that make ftr==0 dwell across the multi-second turnaround:
1. The post-ACK re-arm (`send_ack_pattern` arq_common.cc:9827, `send_mfsk_ack_sack` :10618) sets
   `frames_to_read = rx_frame + 10` (~58 sym ≈ 0.35 s) — far SHORTER than the CMD turnaround gap (CMD
   detect+process+PTT+re-air = seconds at the marginal cfg0 rung).
2. After that short window expires (ftr→0), the OFDM FAIL anti-spin (the single write site at
   arq_common.cc:13846-13847, `if(ftr>0) ftr=bigblock_block_ftr_or(ftr); frames_to_read = ftr;`) re-arms
   ftr to the 8/2-symbol quick-retry on EVERY plateau fail → a fresh coarse search every ~8 sym (~49 ms)
   → ~hundreds of plateau searches over the gap (the measured 607 SKIP-VAR / 56 OFDM-OK).

Legacy NEVER hits this: its coordinated SET_CONFIG handshake re-acquires in LOCKSTEP, so the RX only searches
inside forward-burst windows (the 8-sym anti-spin only fires there). The UNILATERAL in-band reverse-ACK has
no such barrier — the RSP's own turnaround gap is mis-treated as a forward-burst window.

### §VF2.2 The fix (inband-scoped, two coordinated parts in arq_common.cc)

- **(A) Latch a turnaround-suppress deadline when the in-band reverse ACK/SACK is sent.** New members
  `cl_timer inband_revack_turnaround_timer` + `int inband_revack_turnaround_budget_ms`. Set in
  `send_ack_pattern`, `send_mfsk_ack_sack`, `send_mfsk_compact_confirm` AFTER the existing post-ACK flush/
  ftr re-arm, GATED `inband_rate_feature_enabled()`. Budget = the CMD turnaround estimate (CMD decode +
  process + PTT before its next forward preamble can land):
  `RSP_DECODE_MARGIN_MS + ptt_on_delay_ms + ptt_off_delay_ms + message_transmission_time_ms`. This is a
  CONSERVATIVE gap-before-burst (NOT the whole receiving_timeout, which includes the burst airtime).
- **(B) Gate the OFDM FAIL anti-spin re-arm to the turnaround gap.** At the single ftr write site
  (arq_common.cc:13846), when `inband_rate_feature_enabled()` AND the suppress deadline is still in effect
  (`inband_revack_turnaround_timer.get_elapsed_time_ms() < budget`) AND this pass found NO real forward
  frame (`received_message_stats.message_decoded != YES` — a plateau/no-preamble fail, NOT a decoded frame),
  RAISE ftr to span the REMAINING gap in symbols (`remaining_ms * 48000 / 1000 / symbol_period`) instead of
  8/2. CAP ftr at `buffer_Nsymb - frame_symb` so the real burst CANNOT scroll off the ring during the wait.
  The search is thus suppressed once for the gap remainder and re-opens at the forward-burst window.

Why this does NOT regress acquisition of the REAL burst (the load-bearing constraint):
- A real forward burst that arrives mid-gap and DECODES never reaches the anti-spin (the OK path re-arms ftr
  itself at :13037/:13129); the gate is on the FAIL path only.
- A real preamble-first frame (`frame_data_missing`, the in-flight SACK-dispatch race) keeps its existing
  8-sym quick retry — the gate keys on `message_decoded != YES` but the §VF2.4 test confirms a real burst
  preamble inside the window still acquires (ftr is capped so the burst stays in-ring and the NEXT ftr==0
  pass — at the gap close — snapshots the full preamble+frame).
- It does NOT gate on coarse_metric (the REFUTED Fix A): a real CFO-degraded preamble (metric 0.43-0.61) is
  unaffected. The signal is purely the TURNAROUND-GAP TIMING, paced by the same audio clock the burst uses.

### §VF2.3 §5 CROSS-LAYER AUDIT — the shared state (RX search ↔ turnaround state ↔ lockstep window)

1. **PRODUCERS of `frames_to_read`** (the gate): the capture thread decrement (audioio.c:1455); the post-ACK
   re-arms (`send_ack_pattern`:9827, `send_mfsk_ack_sack`:10618, `send_mfsk_compact_confirm`); the OFDM OK
   re-arm (receive():13037/per-frame); the OFDM FAIL anti-spin (:13847, THE site §VF2.2-B constrains); the
   MFSK anti-spin (:13883/:13898); the HAIL/CONNECT scan overrides (arq_responder.cc:160/292); SET_CONFIG /
   BREAK re-inits. **PRODUCERS of the suppress deadline**: the three reverse-ACK send sites (set); cleared
   implicitly by elapsed-time (a one-shot window) — no explicit clear needed, the timer simply ages out.
2. **CONSUMERS**: `receive()` (:12386 ftr==0 gate → snapshot+coarse search); the FAIL anti-spin (:13846
   reads the deadline). The deadline has exactly ONE consumer (the anti-spin gate), so it cannot leak.
3. **VALID STATES of the deadline**: UNSET (timer never started / ctor) → `get_elapsed_time_ms()` returns a
   large value vs a 0 budget → the gate is INACTIVE (legacy/non-inband and the steady forward-burst state).
   ACTIVE (within budget after a reverse-ACK) → the gate raises ftr on a no-frame fail. The budget is 0 when
   `inband_rate_feature_enabled()` is false (never set) → INACTIVE → byte-identical.
4. **INVARIANTS the consumers assume**:
   - INV-1 (forward burst MUST still acquire): preserved — the cap `ftr ≤ buffer_Nsymb - frame_symb` keeps
     the burst in the ring; the OK path bypasses the gate; the gate only lengthens a doomed plateau retry.
   - INV-2 (down-ladder still recovers a GENUINE loss): preserved — the gate only DELAYS the next forward
     search within ONE turnaround budget; a genuine sustained loss spans many budgets, each of which re-opens
     the window and re-fails, so the dead-batch streak / TERMINAL BREAK (§7/§19) still reaches its floor.
     The §VAR-FIX-DEMOTE watchdog is unaffected (it keys on delivered-id advance, not ftr).
   - INV-3 (legacy byte-identical): the whole gate is `inband_rate_feature_enabled()`-scoped; off → the
     deadline is never set, the anti-spin keeps its 8/2 re-arm verbatim. VERIFIED by diff scope.
5. **WHAT THE FIX CHANGES**: on the in-band FAIL anti-spin path, within a turnaround budget, ftr is raised
   (search suppressed) instead of 8/2. Walk each consumer: `receive()` snapshots LATER (at the gap close) —
   correct (no forward burst was airing); the OK path / SACK-dispatch race path / BREAK / HAIL paths are
   untouched (the gate keys on inband + active-deadline + no-frame-fail, none of which those paths satisfy).

### §VF2.4 Regression test (fails-before / passes-after) — `--test-inband-revack-rxgate`

`cl_arq_controller::test_inband_revack_rxgate()` (in master `--test`). Drives the PRODUCTION ftr-gate decision
(`inband_revack_rxgate_ftr()`, the exact helper the anti-spin calls) with the production state:
- ARM the suppress deadline (model a just-sent reverse ACK), then feed N back-to-back PLATEAU fails
  (message_decoded=NO). PASS-AFTER: ftr is raised to span the gap (the search is suppressed → the storm count
  drops to ~0 over the gap); FAIL-BEFORE (`MERCURY_INBAND_REVACK_RXGATE_DEFEAT=1`, same binary): ftr stays
  8 every fail → N searches (the storm signature).
- ACQUIRE arm: with the deadline active, a pass with `message_decoded=YES` (a real burst) bypasses the gate
  (ftr re-armed by the OK path), proving the gate never starves the real forward burst.
- COLD arm: no deadline (legacy/inband-off) → ftr stays 8 (byte-identical).
Asserts the SKIP-VAR/search count during the gap drops to ~0 (PASS) vs N (DEFEAT) while the real-burst arm
still acquires. Master `--test` exit 0 (M=0).

### §VF2.5 CMD-SIDE sibling — the DOMINANT storm (VERIFIED by the fleet A/B, the RSP gate alone was insufficient)

The first faithful real-audio A/B (fleet .11, WGN:40 seed2001, FIX binary, redesign env) showed the RSP-side
gate (§VF2.2) FIRED (armed 20×, suppressed) yet rx_bytes stayed 177 B and the storm persisted at **408
[CMD] FTR-FAIL CONFIG_0** — and the DEFEAT arm (same binary) was identical (456 SKIP-VAR / 177 B). The storm
log lines were ALL `[CMD]` (commander), not `[RSP]`. ROOT (VERIFIED by the log + code read): the storm is on
the COMMANDER, not the responder. After a forward batch TX the CMD waits in RECEIVING_ACKS_DATA for the
reverse ACK. The MFSK detector runs first each poll; when it does NOT fire (the whole turnaround gap before
the MFSK SACK arrives), the CMD's v2 SACK dispatch (arq_commander.cc, `if(!mfsk_handled_this_poll)`) FORCES
`frames_to_read = 0` and runs a fresh OFDM forward-preamble `receive()` EVERY poll — looking for an OFDM
SACK_RSP that, at a WB rung (reverse ACK is MFSK), the RSP never sends. That OFDM search false-locks the
~0.50 GI plateau → SKIP-VAR every poll. The RSP-side ftr gate is irrelevant here because this site
re-forces `ftr=0` each poll, bypassing the anti-spin.

FIX: arm the SAME turnaround suppress window on the CMD at its data-ACK wait entry
(`calculate_receiving_timeout` COMMANDER `ack_pattern_time_ms>0` branch — every RECEIVING_ACKS_DATA
transition routes through it, in lockstep t0 with `receiving_timer`), and SKIP the forced-ftr0 OFDM dispatch
while `inband_cmd_suppress_ofdm_ack_dispatch()` is true (window active AND
`reverse_ack_uses_robust_geometry(current_configuration)` — the reverse ACK is MFSK/robust-geometry, true
for every non-robust OFDM config). The MFSK detector (which runs FIRST each poll) carries the wait; the OFDM
dispatch resumes when the window ages out (a late OFDM SACK_RSP, if any, lands then), so a NB session (OFDM
SACK_RSP, reverse_ack_uses_robust_geometry handles NB via the suffix early-return) and a genuine loss are
preserved (window ages out → dispatch resumes → timeout → demote). Inband + window + reverse-MFSK gated; off
→ the dispatch runs verbatim → legacy byte-identical. FAIL-BEFORE: the same
`MERCURY_INBAND_REVACK_RXGATE_DEFEAT=1` knob disables both the RSP and CMD gates.

§5 audit delta (CMD turnaround state ↔ CMD OFDM SACK dispatch ↔ MFSK ACK detector): the suppressed dispatch
is the CMD's forward OFDM search; its only legitimate consumer at a WB rung's turnaround is an OFDM SACK_RSP
the RSP does not send there (it sends MFSK). The MFSK detector is UNGATED (runs before the suppress), so the
reverse ACK is never missed. The window is the same one-shot timer as §VF2.2 (one consumer per side); the CMD
and RSP arm it independently at their own turnaround entries.

### §VF2.6 SLOW-POLL refinement + HONEST FLEET A/B VERDICT (replicated — gate is NECESSARY-NOT-SUFFICIENT)

A fixed ~1.8s "gap-before-budget" under-covered the ~13s cfg0 batch period, so the storm ran in the long
inter-arm gaps. The refinement: the suppress window spans the WHOLE `receiving_timeout`, and the gated ftr is
capped at ONE FRAME PERIOD (not the ring depth). The forward search thus SLOW-POLLS once per frame across the
turnaround — robust to the gap length, and the real burst is still caught within 1 frame (it decodes on the
OK path, which bypasses the gate). 

FLEET A/B (faithful real-audio snd-aloop, WGN:40 seed2001, FIX binary; defeat = same binary
`MERCURY_INBAND_REVACK_RXGATE_DEFEAT=1`; legacy = inband off). Decisive clean run (run4):

| metric | legacy | FIX (gate) | DEFEAT (storm) |
|---|---|---|---|
| total SKIP-VAR | 12 | **51** | **517** |
| CMD SKIP-VAR | 12 | **0** | 297 |
| RSP SKIP-VAR | 0 | 51 | 220 |
| rx_bytes | 8192 (full) | 161 | 177 |
| configs_seen | [0,100,101,102] | [100,101,102] | [100,101,102] |

VERDICT — the gate is a PROVEN, CORRECT, TESTED fix for the SKIP-VAR storm: **~10× total SKIP-VAR reduction
(517→51), CMD storm 297→0**, clean fails-before/passes-after on the SAME binary (the defeat knob), unit test
green, full `--test` 68/0 (M=0), legacy byte-identical. BUT it is **NECESSARY-NOT-SUFFICIENT for delivery**:
fix 161 B ≈ defeat 177 B, both ≫ below legacy's 8192 B, and NEITHER crosses to cfg0 (no `wb=[0]`; the
redesign holds robust 100–102). The cfg0 SKIP-VAR storm was a real symptom this gate eliminates, but the
binding constraint on delivery is the redesign's ROBUST→cfg0 CROSS/CLIMB failure — a SEPARATE, deeper layer
(the "faithful_beat_vara_climb_binding" constraint), NOT the turnaround search storm. Run-to-run variance is
real (one run had both arms 0 B / no-connect — a connect flake, fix==defeat so not the gate). Honest scope:
this fix removes the storm and its wasted RX cycles + the pin-starvation pressure; it does not by itself make
the redesign deliver at cfg0. The climb/cross layer is the next investigation, NOT another search-gate.

### §VF2.7 CONNECT-PHASE EXCLUSION — fix the rxgate CONNECT regression (the §VF2.6 "connect flake" was the gate)

ROOT (code-verified): `inband_arm_revack_turnaround_window()` (arq_common.cc — the SINGLE producer of
`inband_revack_turnaround_budget_ms` / `inband_revack_turnaround_timer`) is reached from 5 sites. TWO fire
during the pre-link CONNECT / CONTROL-ACK turnaround, not a data-batch turnaround:
- RSP `process_messages_acknowledging_control()` (connection_status==ACKNOWLEDGING_CONTROL, dispatched at
  arq_responder.cc:35) → `send_ack_pattern_with_snr()` (arq_responder.cc:1785) and
  `send_ack_pattern(control_ack=true)` (arq_responder.cc:1795). This is the TEST_CONNECTION_ACK handshake
  echo + the SET_CONFIG / KEY_EXCHANGE control ACKs. NOTE: `link_status` is already CONNECTED here
  (set at arq_responder.cc:3069 BEFORE the echo is queued), so a link-state guard ALONE is insufficient.
- CMD `calculate_receiving_timeout()` COMMANDER `ack_pattern_time_ms>0` branch (arq_common.cc:1563) is also
  reached from the `RECEIVING_ACKS_CONTROL` control-ACK wait (connection_status set at arq_commander.cc:1730,
  arm at calculate_receiving_timeout:1743).

CONSEQUENCE (the §VF2.6 "connect flake", now explained): after the control ACK the side's NEXT RX is a
forward burst it must ACQUIRE at full search cadence (the commander's TEST_CONNECTION, or the first forward
DATA batch at the new config). The armed window instead SLOW-POLLS / suppresses that forward search →
acquisition is starved → CONNECT fails. Measured: conn 6/12 (FIX) vs 11/12 (legacy). The gate's suppress
window — meant for the inter-BATCH turnaround gap — was bleeding into link establishment.

FIX (single choke point at the producer): arm ONLY in the DATA phase — `link_status==CONNECTED` AND
`connection_status` NOT a control-ACK state (`!= ACKNOWLEDGING_CONTROL && != RECEIVING_ACKS_CONTROL`). This
excludes EVERY pre-link state (LISTENING/CONNECTION_RECEIVED/CONNECTING/NEGOTIATING/CONNECTION_ACCEPTED →
not CONNECTED) AND the CONNECTED-but-control turnarounds (handshake echo, SET_CONFIG/KEY_EXCHANGE ACK, CMD
control-ACK wait). Data turnarounds STILL arm: RSP `ACKNOWLEDGING_DATA` (process_messages_acknowledging_data,
sites 2334/2558/2568/2587/2594), CMD `RECEIVING_ACKS_DATA` (calculate_receiving_timeout data branch), and the
RSP `RECEIVING` prev-batch redelivery ACK (process_messages_rx_data_control sites 532/1410/1393). connection_
status is authoritative at every arm site (the responder dispatcher keys the RSP sites on it; the CMD wait
sites set it before calculate_receiving_timeout). Inband-gated already; pre-link/control → budget stays 0 →
the consumers (inband_revack_rxgate_ftr, inband_cmd_suppress_ofdm_ack_dispatch) fall back to the stock
anti-spin / verbatim OFDM dispatch → CONNECT byte-identical to legacy. Same RXGATE_DEFEAT A/B knob.

§5 audit delta (producer/consumer of the turnaround window vs the CONNECT/control state machine):
- Producer: `inband_arm_revack_turnaround_window()` (the only writer). Now reads link_status +
  connection_status and refuses to arm outside the data phase. Setting budget=0 on a control turnaround
  cannot clobber a live data window — control turnarounds only occur at link setup / SET_CONFIG, between
  data batches, never mid-data-turnaround.
- Consumers UNCHANGED: both already treat budget<=0 as "inactive" → legacy fallback. No consumer assumption
  altered; the fix only narrows WHEN the producer arms.
- Test: `test_inband_revack_rxgate()` (arq_responder.cc) gains CONNECT (a)..(d): (a) pre-link
  CONNECTION_RECEIVED → budget==0; (b) CONNECTED+ACKNOWLEDGING_CONTROL (handshake echo) → budget==0;
  (c) CONNECTED+RECEIVING_ACKS_CONTROL (CMD control wait) → budget==0; (d) NEGATIVE: CONNECTED+
  ACKNOWLEDGING_DATA → budget>0 (data turnaround NOT over-excluded). FAIL-BEFORE: drop the guard → (a)(b)(c)
  arm → starve the CONNECT/forward search.

---

## §CROSS — the ROBUST→cfg0 CROSS death-spiral: a FUTILE NACK_DECODE_FAIL on an ACTIVELY-PROGRESSING batch starves forward capture (VERIFIED root; fix implemented 2026-06-28)

§VAR-FIX-2 left the binding constraint as "the redesign's ROBUST→cfg0 CROSS/CLIMB failure." §CROSS is the
VERIFIED root of that failure (instrumented real-audio A/B, mercury_crossbase @ fa8b8e92, WGN:40 seed2001,
snd-aloop native ELF, .31).

### §CROSS.1 The cross DECODES — the collapse is NOT a decode/variance bug (VERIFIED)
At the cfg0 cross the RSP decodes cfg0 cleanly: **38 [OFDM-OK] cfg=0, 0 [OFDM-FAIL]**, var 0.0357, meanH
0.979, iter=0. The §17 descrambler + §22 gi-inherit + §VAR-FIX rxgate fixes ALL engage (SKIP-VAR down 607→56;
the storm is gated). The original task framing ("cfg0 DATA frames SKIP-VAR after a clean lock = bad
equalizer/channel state") is FALSIFIED for the residual: clean cfg0 frames decode byte-correct. Legacy on the
SAME audio crosses + delivers 11339 B (configs_seen [0,100,101,102], wb=[0]); redesign delivers 147 B,
wb=[], breaks=1.

### §CROSS.2 The death-spiral (VERIFIED — the per-batch SACK bitmaps DEGRADE)
The forward batch never COMPLETES, and progressively LOSES frames each period (ACK-GATE-DIAG):

| batch_seq_id | received seqs | bitmap | rx/exp |
|---|---|---|---|
| 3 | 1 2 3 4 5 | 0x3e | 5/6 |
| 4 | 2 3 4 | 0x1c | 3/6 |
| 5..8 | 2 3 | 0x0c | 2/6 |

Frame **seq 0 is ALWAYS lost** (the acquisition-seam frame that bears the fresh OFDM anchor + the CONFIG_TAG
burst, main.cc:1117); then frames 1 and 5 also drop, converging on only the middle seqs 2,3. The session
delivers ~147 B and never sustains the climb.

### §CROSS.3 ROOT — the down-ladder fires + emits a FUTILE NACK_DECODE_FAIL on a HEALTHY progressing batch
During the inter-frame gaps of an actively-receiving cfg0 batch (frame 6 not yet arrived, or seq-0 retransmit
pending), the RSP firing gate (arq_responder.cc:626-634: fresh-window-decoded + status!=RECEIVED + OFDM +
bsi>=0) fires `inband_try_down_ladder_on_decode_fail`. The down-ladder correctly skips all ROBUST trials
(holding the cfg0 lock) → "no config in window decoded" → and on the first such pass per dead-streak segment
**emits a NACK_DECODE_FAIL** (arq_common.cc:5674-5679). That NACK is:
- **Semantically FALSE**: it tells the CMD "I cannot follow / I am stuck below your announced config" — but the
  RSP has decoded 1..5 frames of THIS batch at cfg0 (`batch_rx_frame_count >= 1`), PROVING the CONFIG_TAG
  arrived and the RSP IS at cfg0. The CMD correctly reads it as "not a climb-miss (announced idx=-1) → no
  accelerated demote" (arq_common.cc:3932-3941) — i.e. the NACK accomplishes NOTHING.
- **Actively HARMFUL**: each NACK is a ~75920-sample (~1.5 s) reverse passband burst (arq_common.cc:2821
  tx_transfer). While the RSP keys that reverse burst it CANNOT capture the forward cfg0 burst → it misses the
  seq-0 retransmit AND the early frames of the NEXT batch → the bitmap degrades 5/6→3/6→2/6. A 1-frame
  acquisition-seam miss is amplified into a death spiral. (This is the campaign reverse-ACK-transmit-time
  binding constraint, memory feedback_reverse_ack_transmit_time_first: a reverse-ACK MISS must be CHEAP.)
- **Re-decode storm on the CMD**: the NACK burst sits in the CMD capture ring for the whole turnaround; the CMD
  re-decodes the SAME NACK (bsi_lsb=3 parity=0) ~12× in 0.2 s (arq_commander.cc:4124, no dedup, unlike the
  SACK dedup at :4179) → 69 [CMD-NACK] in the run → burns the CMD forward-TX RX-loop passes.

§19 (dead-batch classifier) correctly suppresses the false BREAK (PROGRESS_RESET / RATE_LIMITED → breaks=1,
not many) — but §19 runs AFTER the NACK emit (line 5703 vs 5674) and does NOT touch the NACK. So the climb-
killer that REMAINS after §17/§19/§20/§21/§22/§VAR-FIX is this futile-NACK reverse-airtime starvation.

### §CROSS.4 THE FIX — never assert "cannot-follow" on a progressing batch (chokepoint at the NACK emit)
The down-ladder lost-tag NACK_DECODE_FAIL is suppressed when `batch_rx_frame_count >= 1`. Discriminator
rationale: a lost CONFIG_TAG can only strand a batch whose FIRST frame at the current config failed (a config
change starts a NEW batch / NEW bsi, so its batch_rx_frame_count is 0 — arq_responder.cc:542/682 reset). Once
≥1 DATA frame of the current batch has decoded at the current config (batch_rx_frame_count++ at
arq_responder.cc:1454, the confirmed-storage point), the tag is PROVABLY present and any missing frames are a
PARTIAL batch the SACK/retx path already owns — never a cannot-follow. The blind down-ladder DECODE, the §19
classifier, and the §VAR-FIX-DEMOTE watchdog all STILL run (no-regress: the genuine-stall demote/BREAK paths
are untouched) — only the harmful reverse-airtime NACK is gated. FAIL-BEFORE knob
`MERCURY_INBAND_PROGRESSING_NACK_DEFEAT=1` restores the futile NACK on the same binary.
Defense-in-depth: the CMD dedups a re-decoded NACK by (bsi,reason,parity) so a single genuine NACK is handled
ONCE per turnaround, not ~12× (arq_commander.cc, mirrors the SACK dedup).

### §CROSS.5 §5 CROSS-LAYER AUDIT — the NACK_DECODE_FAIL emit decision (shared state: down-ladder ↔ SACK/retx ↔ NACK ↔ batch-progress ↔ reverse-turnaround)
1. PRODUCERS of the cannot-follow NACK: ONLY `inband_try_down_ladder_on_decode_fail` (arq_common.cc:5674) via
   `inband_emit_nack(NACK_DECODE_FAIL)`; and `inband_emit_nack(NACK_UNFOLLOWABLE_CLIMB)` at arq_common.cc:3145
   (an un-adoptable tag — a DIFFERENT trigger, NOT gated by this fix). The production firing gate is the single
   site arq_responder.cc:655.
2. CONSUMERS of the NACK: the CMD `inband_handle_nack` (arq_common.cc:3899) → accelerated-demote ONLY when the
   RX is BELOW the announced config (climb-miss) or UNFOLLOWABLE; otherwise a no-op log. So a NACK on a
   progressing batch was ALREADY a CMD no-op — suppressing its EMISSION changes no CMD decision, only removes
   the reverse-airtime + the re-decode storm.
3. VALID STATES of batch_rx_frame_count: 0 (no frame of the current batch stored yet — the lost-tag
   precondition AND the genuine total-loss precondition) | ≥1 (≥1 frame stored at the current config — the tag
   is present). Reset to 0 at: session start, batch boundary (542), TERMINAL BREAK (682). Default-init 0.
4. INVARIANTS consumers assume: (INV-E1) a NACK_DECODE_FAIL means "the RX genuinely could not follow." The
   futile firing VIOLATED INV-E1 (the RX WAS following — 5/6 frames). The fix restores INV-E1: NACK only when
   batch_rx_frame_count==0 (no frame followed). (INV-E3 stale-epoch / mutual-exclusion unchanged.)
5. WHAT THE FIX CHANGES + per-consumer walk:
   - CMD inband_handle_nack: previously got a NACK it logged as "no accelerated demote." Now gets no NACK on a
     progressing batch → identical net decision (no demote) but no re-decode storm. A GENUINE lost-tag
     (batch_rx_frame_count==0) still NACKs → the climb-miss accelerated-demote path is preserved. OK.
   - SACK/retx (the partial-batch owner): unchanged — the partial SACK (bitmap 0x3e) still goes out and the CMD
     retransmits the missing seq-0; with the reverse channel no longer congested by the 1.5 s futile NACK the
     retransmit can actually be captured. OK (this is the intended win).
   - §19 dead-batch classifier (arq_common.cc:5703) + §VAR-FIX-DEMOTE watchdog (:5649): both still run on every
     real-signal down-ladder-fail pass → the genuine zero-progress BREAK + the hard-stall pin-release are
     untouched. A progressing batch hits PROGRESS_RESET anyway, so the suppressed NACK never co-occurs with a
     genuine demote. OK (no-regress).
   - down-ladder ADOPT (INV-S4-1, a CRC/LDPC pass required to adopt): unchanged — the fix gates only the NACK,
     not the decode/adopt. A legit in-OFDM rate-down adopt still works. OK.
   - reverse-turnaround / rxgate (§VAR-FIX-2): fewer reverse bursts → the RX forward-search gate has fewer
     turnarounds to span; strictly complementary. OK.
   Legacy (inband off): inband_emit_nack early-returns (feature gate) → byte-identical. OK.

### §CROSS.6 RESULT + the NEXT root (HONEST — the fix is NECESSARY-NOT-SUFFICIENT)
The §CROSS NACK fix WORKS for its target (instrumented A/B, mercury_crossfix, WGN:40 .11, N=3 inband):
- the futile NACK storm is ELIMINATED: **NACK emit 0** on the progressing cells (33 "SUPPRESS futile"
  per cell), vs the pre-fix 69-NACK / 12×-redecode storm.
- the RSP now decodes MORE cfg0 frames: **65 [OFDM-OK] cfg=0** (was 38), 0 OFDM-FAIL.
- delivery improves marginally (inband rx 211/211/1280 B, 1/3 cells load WB) vs the bug arm @fa8b8e92
  (187 B mean, 0/2 WB-load). Legacy crosses+delivers 11339 B (3/3) on the same audio.

But the cross is STILL NOT restored — the deeper binding root, now PINNED:

**NEXT ROOT — the post-cross RSP prev-batch cross-storage LAGS one batch (VERIFIED, the
[RSP-V2-PREV-BUMP] / [RSP-V2-PREV-RX] trail, arq_diag_inband / cx11_inband_sd1).**
At the FIRST multi-frame cfg0 batch (batch_seq_id=3, 6 frames) the batch reaches only 5/6 (the
acquisition-seam frame-0 / partial), never completes, and when the CMD starts batch 4 the RSP runs
`[RSP-V2-PREV-BUMP] prev_batch_seq_id=3 next_expected=4 ... received_on_transfer=5/6 (cross-storage
routing armed)`. From then on EVERY batch is incomplete and PREV-BUMPed; `received_on_transfer` locks at
**2/4** and stays there for the rest of the session. The smoking gun: after the cross, **every decoded
forward frame is routed to `[RSP-V2-PREV-RX]` (19 of them) and `current store (RX-DATA) == 0`** — i.e.
the RSP is PERMANENTLY one batch behind the CMD: the CMD is on batch N+1, the RSP is still trying to
complete batch N (now "prev"), so frame 0 of every new batch is mis-counted, neither batch ever completes,
and `rsp_wb_data_frames` stays 0 / wb_configs_seen stays [] / 0 durable delivery. seq=0 IS decoded (13×) —
this is NOT an acquisition loss but a **bsi-advance / prev-batch cross-storage one-batch-lag deadlock**
(shared ARQ state: rsp_current_expected_batch_seq_id, rsp_prev_batch_seq_id, the V2 prev-bump transfer).

This is a DISTINCT root in the batch-completion / bsi-routing layer (NOT the NACK this section fixed, NOT
the §17/§19/§21/§22/§VAR-FIX layers). It needs its own §5 audit of the V2 prev-batch cross-storage state
machine at the ROBUST→cfg0 cross: why the first cfg0 batch never completes (the acquisition-seam frame-0
that legacy avoids via the SET_CONFIG lockstep handshake — the CMD knows the RSP is ready before airing
frame 0), and why a single incomplete batch puts the RSP into a permanent one-batch lag instead of
catching up to the CMD's current bsi. Candidate directions (each needs a live fail-before/passes-after):
(a) on the cross, the CMD must not advance past a batch the RSP has not started (a lockstep-lite on the
FIRST cfg0 batch only); (b) the RSP prev-bump must CATCH UP to the CMD's current bsi when it falls >1
behind (re-baseline current_expected to the freshest seen bsi) rather than perpetually draining a stale
prev; (c) make the first cfg0 batch's frame-0 acquisition robust (a longer settle / a re-aired frame-0).
(b) looks closest to the true invariant. VERDICT for §CROSS: the NACK fix SHIPS (proven necessary,
eliminates a real amplifier, no-regress, --test green); the bsi-lag is the NEXT root, NOT this one.
