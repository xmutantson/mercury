# Data-Flow Audit — in-band capture-ring ROBUST-floor OVER-SEAT at a climbed OFDM rung

Branch: `staging/inband-frameloss` (off `monitor` @027cc927). Scope: the RSP primary
OFDM capture-ring sizing under `MERCURY_INBAND_RATE=1` — specifically
`inband_seat_robust_ring_floor()` (`arq_common.cc:4336`). Legacy (`!inband_rate_feature_enabled()`)
is byte-identical (the seat early-returns off-flag).

Sibling of `data-flow-robust-ofdm-adopt-flush.md` (which fixed the SAME over-seat for the
robust->OFDM **adopt** path, §10/§14/§21) and `data-flow-inband-frame0-rolling-partial.md`
(the gearshift consequence of a lead-frame partial). This doc owns the THIRD, uncovered
sibling: a session that **climbs ROBUST -> ... -> CONFIG_n via the in-band CONFIG_TAG + the
legacy SET_CONFIG tier-cross** (never through `inband_finalize_ofdm_adopt_ring`) and PINS at an
OFDM rung. There the `inband_ofdm_acq_ring_shrunk` latch is never set, so the per-pass seat
over-grows the ring and breaks the current rung's own OFDM acquisition.

---

## §1 The bug (root cause — VERIFIED by a fair `-x sim` A/B, this session)

A/B: two `mercury.exe` instances through `tools/sim/sim_arq_channel.py` (the device-free
AWGN software channel), WGN:40, `--start-cfg 8 --robust --compress off`, 120s. ON arm =
`MERCURY_INBAND_RATE=1`; OFF arm = unset. Both climb ROBUST->CONFIG_8 and PIN there.

| metric | OFF (legacy) | ON (in-band, pre-fix) |
|---|---|---|
| `[BREAK] Block failure` | **0** | 1+ |
| `SKIP-VAR` events | 4 (noise) | **282** |
| `DOWN-LADDER lost-tag resync` | **0** | 19 |
| partial SACKs (`CMD-MFSK-ACK-SACK PARTIAL`) | **0** | 4 |
| forward delivery | clean, contiguous `seq=2..28/30` | rolling frame-0 loss every batch |

MECHANISM (RSP, ON arm — `/tmp/ab_on.modem.log`):
1. The link climbs to CONFIG_8. Before any seat, the OFDM coarse acquisition is healthy:
   `coarse: ... bounds=[4,343] metric=0.998 bufNsymb=467 Nsymb=120 PASS` — ring = **467**
   (CONFIG_8 natural).
2. `[INBAND-RX] seating robust ring floor: buffer_Nsymb_min=1291 (was ring 467)` — the
   per-pass seat GROWS the ring **467 -> 1291** (the ROBUST_0 floor).
3. From then on, EVERY batch's frame-0 fails the pre-LDPC SKIP-VAR gate at a garbage variance
   (`SKIP-VAR: var=66.1171 too high (>0.50 cfg=8)`). `[ENERGY-DIAG]` shows `data: pb=1.92` vs
   `pream: pb=0.48` — the freshest preamble lands at the TAIL of the oversized 1291-symbol ring,
   beyond `upper_bound = buffer_Nsymb - (Nsymb + preamble_nSymb)`, with its frame DATA
   off-buffer. IDENTICAL `OFDM beyond-bounds` mechanism `data-flow-robust-ofdm-adopt-flush.md
   §10` diagnosed for CONFIG_0 — here at CONFIG_8.
4. Frame-0 lost every batch -> RSP reports `24/25` (`bitmap=0x01fffffe`, bit0 clear) -> CMD
   mixbatch-retx -> recovers one batch late, but the rolling lag + `DOWN-LADDER lost-tag resync`
   churn drives `data_ack_received==NO` cycles -> `[BREAK] Block failure #1 at config 8`.

OFF (legacy) NEVER seats (the seat is `inband_rate_feature_enabled()`-gated), so the CONFIG_8
ring STAYS natural (467) -> clean contiguous delivery, 0 breaks. The over-seat is the
in-band-specific amplifier.

## §2 Why the existing guards miss it

`inband_seat_robust_ring_floor()` already had TWO guards that prevent the over-seat:
- `inband_ofdm_acq_ring_shrunk` (`arq_common.cc:4347`) — set ONLY by
  `inband_finalize_ofdm_adopt_ring` (`:4979`), i.e. ONLY on the robust->OFDM **adopt** path.
- the CONFIG_0-only natural-ring guard (`:4382`) — scoped `current_configuration == CONFIG_0`.

A session that climbs to CONFIG_8 via the in-band CONFIG_TAG (unilateral) + the SET_CONFIG
tier-cross reaches CONFIG_8 WITHOUT calling `inband_finalize_ofdm_adopt_ring`, so the shrunk
latch is never set; and CONFIG_8 != CONFIG_0 so the second guard does not apply. Both guards
are point fixes for two earlier siblings (CONFIG_0-adopt = #1e; CONFIG_0-start = §21). This is
the general case they each special-cased.

## §3 The fix (root cause — keep the natural ring at EVERY healthy OFDM rung)

FIRST ATTEMPT (REJECTED — refuted by the A/B): seat to `inband_down_window_buffer_nsymb()` (the
down-decoder bank's own snapshot size, smaller than the robust floor) instead of the robust
floor. This SHRANK the over-seat (CONFIG_8 1291 -> 697 in the live sim) but did NOT fix it: 697
is STILL larger than CONFIG_8's natural ring (467), and the re-run A/B showed the SKIP-VAR storm
and the BREAK PERSISTED (`bounds=[4,669]` still puts the preamble at the tail, delivery stuck at
1675 B). VERIFIED: it is the GROW ITSELF — any ring larger than the CURRENT rung's natural size
— that breaks that rung's coarse acquisition (preamble beyond `upper_bound`), not the grow's
magnitude. So the down-ladder's deep blind-decode reach (a SECONDARY follow path) and the
CURRENT rung's acquisition (the BINDING need) CANNOT coexist on one primary ring.

THE FIX (matches the robust->OFDM adopt path's already-shipped tradeoff): at EVERY healthy OFDM
rung holding its natural ring, SUPPRESS the seat — keep the ring natural so acquisition works.
The robust->OFDM adopt path already does exactly this (`force_set_capture_ring_natural()` +
latch `inband_ofdm_acq_ring_shrunk` to suppress the re-seat, §10/§14). The §21 fix did it for
the CONFIG_0-START case via a CONFIG_0-only natural-ring guard. We GENERALIZE that guard to
EVERY OFDM config (`is_ofdm_config(current_configuration)` instead of `== CONFIG_0`,
`arq_common.cc:4382`). On a genuine demote BACK to a ROBUST config `is_ofdm_config()` is false ->
the guard does NOT fire -> the seat runs and grows the ring for the robust-tier down-ladder
(UNCHANGED). The down-ladder loses its deep OFDM->lower-OFDM blind-decode reach, but a CMD
down-demote is still followed via the robust-suffix CONFIG_TAG
(`inband_detect_follow_from_capture`, `arq_responder.cc:663`), which does NOT need an oversized
primary ring — the SAME tradeoff the adopt path ships.

## §4 §5 CROSS-LAYER AUDIT — `data_container.buffer_Nsymb_min` / the primary capture ring

Shared PHY<->ARQ state: the ARQ layer (`inband_seat_robust_ring_floor`) raises the PHY ring
floor; the PHY layer (OFDM coarse acquisition + the down-decoder bank) consume it.

1. **Producers** of `buffer_Nsymb_min` / the ring size:
   - `inband_seat_robust_ring_floor()` (`arq_common.cc:4413`) — the seat. THE FIX SITE (the
     guard above it now suppresses the seat at any healthy OFDM rung).
   - `inband_finalize_ofdm_adopt_ring()` -> `force_set_capture_ring_natural()` (`:4945`) —
     un-seats to natural on a robust->OFDM adopt. UNTOUCHED.
   - `load_configuration` / `data_container.set_size` — honors `buffer_Nsymb_min`. UNTOUCHED.
   - ctor / reset — `buffer_Nsymb_min = 0`. UNTOUCHED.
2. **Consumers**:
   - OFDM coarse acquisition (`telecom_system.cc`, `bounds=[4, buffer_Nsymb-frame_symb]`) —
     the VICTIM of the over-seat (preamble beyond upper_bound). The fix keeps the ring natural
     at OFDM rungs -> healthy `bounds`.
   - the down-decoder bank snapshot (`inband_down_ladder_resync` reads `buffer_Nsymb *
     sym_samples`, `arq_responder.cc:4831`) — at an OFDM rung the ring is now natural, so the
     bank can read frames AT or ABOVE the current rung but not a deeper (larger-frame) rung.
     This is the ACCEPTED tradeoff (the CMD down-demote is followed via the CONFIG_TAG, not the
     blind down-ladder). At a ROBUST rung the seat still fires -> the bank still reaches the
     robust floor. PRESERVED where it matters.
   - `skip_var_nv_ceiling` gate (`telecom_system.cc:2973`) — reads the mis-acquired frame's
     `noise_variance_estimate`; with the natural ring frame-0 acquires correctly -> the gate
     passes.
3. **Valid states / invariant**: before any seat, `buffer_Nsymb_min == 0` (natural ring). The
   invariant the BINDING consumer (acquisition) needs: **the current rung's freshest preamble
   sits <= upper_bound**, which holds iff the ring is at the rung's natural size. The old seat
   VIOLATED it at OFDM rungs (1291 >> CONFIG_8 geometry). The fix keeps the natural ring at OFDM
   rungs (invariant restored) and only seats at ROBUST rungs (where the rung's own frame is that
   large, so the preamble fits there too).
4. **Uncommon paths verified**:
   - ROBUST config: `is_ofdm_config` false -> guard does NOT fire -> seat grows the ring to the
     robust floor (byte-identical to before). PRESERVED.
   - CONFIG_0 (lowest OFDM): the old CONFIG_0-only guard already suppressed the seat; the
     generalized guard is a strict superset -> identical at CONFIG_0. SUBSUMED.
   - robust->OFDM adopt: `inband_ofdm_acq_ring_shrunk` already early-returns the seat at an OFDM
     config (`:4347`); the new guard is downstream and redundant there. UNTOUCHED.
   - flag-off / legacy: the seat early-returns at `:4338`. BYTE-IDENTICAL.
5. **What the fix changes**: ONE assumption — "the primary ring must hold the deepest ladder
   rung even while pinned at an OFDM rung" -> "while pinned at a HEALTHY OFDM rung the ring stays
   natural (acquisition first); the robust-floor grow runs only at a ROBUST rung". Every consumer
   above re-verified.

## §5 Tests (fail-before / pass-after)

Directed in-process unit `test_inband_ring_floor_overseat` (`--test` + `--test-inband-ring-floor`):
drives `inband_seat_robust_ring_floor()` at CONFIG_8 (a climbed OFDM rung holding its natural
ring, no prior adopt) and ROBUST_0.
- A1 PASS-AFTER: at CONFIG_8 the seat is SUPPRESSED (`buffer_Nsymb_min` stays 0, the live ring
  is UNCHANGED at its natural size).
- A1-FB FAIL-BEFORE (`MERCURY_CONFIG0_RING_GUARD_DEFEAT=1`, the EXISTING §21 knob — it now
  disables the GENERALIZED guard on the SAME binary): the seat over-grows the CONFIG_8 ring to
  the ROBUST floor (the bug reproduced as an explicit assert).
- A2 ROBUST_0: the seat STILL grows the ring to the robust floor (`is_ofdm_config` false ->
  guard does not fire -> robust-tier down-ladder ring preserved).
- A3 flag-OFF: the seat is a no-op (legacy byte-identical).

E2E confirmation: re-run the `-x sim` A/B with the fixed binary; the ON arm's CONFIG_8
`SKIP-VAR` storm, `DOWN-LADDER lost-tag resync` churn and the `24/25` rolling partial drop to
the legacy-OFF baseline and the link delivers contiguously.

PART E of `test_inband_deliver` (the per-pass PHY-rebuild-leak regression) was moved from
CONFIG_10 to ROBUST_0 — its seat now legitimately fires there (the OFDM guard suppresses it at
OFDM rungs), so the leak + genuine-seat behaviors it owns are exercised on the path that seats.

## §6 HONEST VERDICT — the over-seat is ONE of TWO contributors (residual flagged)

The `-x sim` A/B with the fix (`/tmp/abv2.modem.log`, WGN:40, CONFIG_8) confirms the over-seat
is GONE (`seating robust ring floor` count = 0; coarse acq holds the natural ring 467 at
`bounds=[4,343] metric=0.998`) and delivery improves materially: **311 B (broken ON) -> 4489 B
(fixed ON)**, `repro_deep_stall=False`, `repro_over_climb_collapse=False`. The constant
`var=66.1171` per-batch storm is eliminated.

BUT it does NOT fully close the gap to legacy OFF (which gets 5 clean Data-ACK-PATs, 0 partials
at CONFIG_8). The fixed ON arm STILL shows CONFIG_8 partials (24/25 on bsi=0, then 17/25, 16/25).
ROOT of the RESIDUAL (correlated, this session): a SECOND, distinct in-band contributor — the
CONFIG_TAG passband burst keyed BEFORE frame-0 (`emit_config_tag_passband`, `arq_common.cc:9120`,
75920 samples). The first CONFIG_8 batch's `var=60.4372` frame-0 SKIP-VAR storm starts at
T+0022.4 IMMEDIATELY after the `CONFIG_TAG passband emit cfg=8` at T+0021.2 — the burst
contaminates frame-0's acquisition seam (the fact-doc-noted "tag contamination", §1 of
`data-flow-inband-frame0-rolling-partial.md`). Some later partials (between tags, T+47/T+60) are
the genuinely-marginal CONFIG_8 acquisition at WGN:40, recovered by mixbatch retx.

So this fix SHIPS as a real, independently-correct improvement (the over-seat was a hard
acquisition-breaker; removing it is unconditionally right and the directed test gates it). The
CONFIG_TAG-before-frame-0 contamination is a SEPARATE fix (candidates: key the tag AFTER the
batch, or gap-protect frame-0's preamble from the burst tail) tracked as residual work — NOT
folded in here to keep this change minimal and its test honest.
