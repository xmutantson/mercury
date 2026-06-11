# data-flow-data-ack-sack-correlator.md

Cross-layer data-flow audit for the **steady DATA-ACK / SACK-suffix correlator**
and its multi-window extension (`MERCURY_DATA_ACK_MULTIWINDOW`).

Track A Step 2 (mwcorr worktree, branch `feat/data-ack-multiwindow`, off monitor
`627c370`). Companion regression test: `cl_arq_controller::test_data_ack_multiwindow()`
(CLI `--test-data-ack-multiwindow`). All `file:line` cites are against the mwcorr
worktree tree at branch creation (627c370) unless a fix moves the line, in which
case the post-fix line is given.

---

## §1 The bug (root cause, execution-cited)

The steady DATA-ACK + SACK-suffix correlator in `process_messages_rx_acks_data()`
correlates ONLY the **newest `tail_samples`** of the capture ring. The snapshot is
the inline memcpy:

- `int tail_offset = signal_period - tail_samples;` — `arq_commander.cc:2730`
- `int rwi_mfsk = ring_write_index;` + memcpy from
  `passband_delayed_data[rwi_mfsk + tail_offset]` — `arq_commander.cc:2737-2740`
- decoded by `decode_ack_sack_from_passband(...)` — `arq_commander.cc:2747-2749`,
  which delegates to `detect_ack_snr_from_passband` (`telecom_system.cc:3964`) —
  the SAME base-pattern correlator over the newest tail.

This snapshot is taken every poll while `receiving_timer < receiving_timeout`
(`arq_commander.cc:2663`). The newest-tail assumption is correct ONLY under the
production ~500 Hz capture-prep thread, which CONTINUOUSLY refreshes the ring so
whichever poll the trailing ACK sample reaches the demod sees it. It BREAKS when
the ACK+SACK burst arrives ONCE, late and mis-phased, then trailing idle silence
scrolls it out of the newest `tail_samples` before a poll's snapshot fires →
`detect_ack_snr_from_passband` returns `matched=0/7`, `peak_metric=0.00`
PURE-SILENT (the bench-9 `HW_BENCH9/BENCH9_VERDICT.json` `matched=0/7` locus).

**Root enabler (not fixed here — orthogonal):** the CMD-branch
`calculate_receiving_timeout` builds its listen window from per-frame / fixed
terms with NO term ∝ forward batch airtime (`arq_common.cc:1065` `frame_drain =
2 * message_transmission_time_ms`; `:1069` `timeout = frame_drain + sack_arrival
+ margin`). On a long held-CFG16 batch (~4.78s) the keyer/AGC/capture-flush
latency + ±8.16 ppm accrue one-sided and push the single EOB SACK late relative
to the fixed newest-tail window.

**The burst is NOT lost.** The ring is double-mapped and retains
`~buffer_Nsymb` (~1301) symbols (`data_container.cc:170-171`), far more than the
ACK round-trip — the ACK sits at an OLDER phase, recoverable by re-snapshotting
back through the retained history.

---

## §2 Producers (writers of the shared state)

SHARED STATE = the capture ring `passband_delayed_data` (double-mapped,
`data_container.cc:170-171`, size `2*Nofdm*buffer_Nsymb*freq_interp`), its
`ring_write_index`, the `frames_to_read` snapshot gate, and the derived match
outcome (`*out_matched`, `data_ack_received`, `sack_detected`, the SACK bitmap,
`emergency_nack_count`).

- **Capture/prep thread** `audioio.c:1407-1428`: writes each captured symbol to
  BOTH mirror halves (`passband_delayed_data[wi]` and `[wi+sp]`), advances
  `ring_write_index = (wi + symbol_period) % sp` (`:1427-1428`), decrements
  `frames_to_read` (`:1431-1433`), sets `data_ready=1` (`:1435`). Sole live audio
  writer; source of the double-mapped contiguity invariant.
- **Ring/index resets** (post-TX drain, session reset, config switch):
  `arq_common.cc:4631, 4677, 5075, 5316, 5441, 5913, 6114, 6299, 6641` (each
  `ring_write_index = 0` + memset of `2*buf_samples`). Also `arq_responder.cc:189,
  2740` (memset of `passband_delayed_data` on reset).
- **SIM_INPROC pump** `pumped_settle_wait()` (`arq_common.cc:6966`) — drives the
  shared virtual clock so the prep-thread-equivalent fill happens in-thread (sim
  only).
- The MFSK SACK probe itself does **NOT** write the ring or `frames_to_read`:
  its snapshot is a read-only peek (`arq_commander.cc:2732-2741` comment + memcpy;
  no `frames_to_read` mutation). The OFDM dispatch BELOW it owns audio-advance
  (`arq_commander.cc:2918-2920` sets `frames_to_read = 0`).

---

## §3 Consumers (readers of the tail snapshot + match outcome)

- **DATA-ACK+SACK suffix probe** (this file's primary consumer):
  `arq_commander.cc:2737-2741` (snapshot) → `:2747` `decode_ack_sack_from_passband`
  → CRC12 gate `:2755-2774` → bsi-window / bitmap!=0 / dedupe sanity `:2777-2873`
  → CLEAN sets `v2_ack_pat_pre_detected=true` (`:2819`) / PARTIAL sets
  `sack_detected=true` + populates `sack_bitmap[]` (`:2840-2843`).
- **SACK bitmap → retransmit queue**: the downstream `if(sack_detected)` block
  consumes `sack_bitmap[]` to build the retx list (`[CMD-V2-MIXBATCH-RETX]`).
- **data_ack_received / emergency_nack_count → control flow**:
  `data_ack_received==NO` drives the timeout/BREAK escalation
  `emergency_nack_count++` → BREAK/D3 demote when `>= emergency_nack_threshold`
  (`arq_commander.cc:2610-2650`); reset to 0 on a real ACK.
- **Detector internals**: `detect_ack_snr_from_passband` (`telecom_system.cc:3515+`)
  captures suffix tones into `ack_mfsk.last_ack_sack_suffix_tones[]` /
  `last_ack_sack_capture_valid`, consumed one-shot by
  `decode_ack_sack_from_last_capture` (`telecom_system.cc:3967-3970`). Each call
  re-runs the detector and re-captures, so running the decode twice (helper +
  body) is safe — the second run re-captures from the re-snapshotted phase.

---

## §4 Valid states (especially pre-write defaults)

- Ring before any producer writes: zero-filled (`memset` at `data_container.cc:171`
  and every reset site) → all snapshots read silence (tail RMS < the energy gate)
  → energy gate skips the FFT.
- `ring_write_index = 0` default; `frames_to_read` default lets the prep thread
  fill; `data_ack_received` default NO; `sack_detected` default false.
- `MERCURY_DATA_ACK_MULTIWINDOW` unset (default) → the look-back loop is entirely
  skipped (gated `if(mw_data_ack_multiwindow_enabled() && ...)`), so the snapshot
  is the unchanged newest-tail memcpy ⇒ byte-identical to 627c370.

---

## §5 Invariants the consumers assume + WHAT THE FIX CHANGES

(I1) **Double-map contiguity**: `[rwi + off]` for `off ∈ [0, signal_period]` is a
contiguous chronological window — MAINTAINED by `audioio.c:1407-1425` writing both
halves. The MW look-back reads `[rwi_mfsk + off]` for `off < tail_offset`, strictly
inside `[0, signal_period]` ⇒ I1 holds for every probed phase. (load-bearing.)

(I2) **The ACK lives in the NEWEST `tail_samples` at the instant the snapshot
fires** — TRUE only under continuous prep-thread refresh; VIOLATED on a single-shot
late burst (the §1 bug). **THIS IS THE ONLY INVARIANT THE FIX ALTERS.** The fix
relaxes I2 from "newest tail only" to "newest tail OR an older energetic phase
whose decode passes CRC12". Walk of consumers under the relaxed I2:

  (a) **SACK bitmap → retx queue**: now possibly fed a bitmap recovered from an
  older phase. SAFE because the bitmap is gated by the UNCHANGED CRC12
  (`arq_commander.cc:2755-2774`) AND the UNCHANGED bsi-window check (`:2782-2785`,
  `rx_bsi == cmd_bsi || prev_bsi`). A wrong-phase frame for the wrong batch is
  rejected by the bsi-window; a noise phase cannot pass CRC12. The dedupe
  (`sack_clean_confirmation_accepted`, `cmd_last_applied_clean_bsi` /
  `cmd_last_applied_sack_bsi`, `:2805-2807`) prevents double-applying a recovered
  ACK. No retx-queue assumption is violated.

  (b) **data_ack_received / emergency_nack_count / D3-demote**: recovering a REAL
  late ACK correctly RESETS the escalation (the intended behavior) instead of
  spuriously incrementing `emergency_nack_count` toward a false TX-BREAK / D3
  demote. A genuine all-silence miss leaves the newest-tail behavior verbatim and
  still escalates. The fix REMOVES false escalation; it adds no new suppression of
  real misses.

  (c) **ring_write_index**: read under `capture_prep_mutex` for the whole scan
  (one lock held across all phases), exactly as the existing snapshot does
  (`arq_commander.cc:2736-2741`). No producer is constrained.

(I3) **tail RMS over the probe span ≥ gate ⇒ signal present** — the MW loop uses
the SAME `ACK_ENERGY_GATE_RMS` (0.001) as a CHEAP pre-gate per phase so only
energetic phases pay the FFT. Phases below the gate are silence ⇒ skipped (same
semantics as the body's energy gate).

(I4) **A matched-count failure means "no ACK this poll" → re-poll / escalate** —
the fix makes I4 fire LESS often (only when EVERY energetic older phase also
fails CRC12), correcting the §1 false-fire. A genuine miss still falls through to
the unchanged miss path.

(I5) **The detector body, energy gate, accept thresholds, CRC12 + bsi-window +
bitmap!=0 + dedupe sanity are UNCHANGED** — the fix only MOVES where the snapshot
reads (`tail_offset` reassigned to the chosen older phase). The body at
`arq_commander.cc:2742-2873` re-snapshots + re-decodes that phase verbatim. No
threshold is relaxed (CLAUDE.md §2). The CRC12 gate inside the helper IS the
discriminator that makes an older-phase accept safe — a false phase cannot pass
CRC12.

### Decision per CLAUDE.md §5

The only altered invariant (I2) is constrained by the UNCHANGED CRC12 + bsi-window
+ dedupe gates downstream, which are exactly the gates that make a recovered
older-phase ACK safe. No consumer assumption is violated; one consumer assumption
(I2) is corrected. The fix is per-call-site scoped (only the steady DATA-ACK/SACK
wait arms it) and env-gated (`MERCURY_DATA_ACK_MULTIWINDOW`, default OFF →
byte-identical), so the merged binary's production wire is unchanged until the
bench arbitrates.

---

## §6 The fix (mechanism)

Extends the CONNECT sub-mode-B multi-window template (`feat/connect-race` HEAD
`579ea8d`, `receive_ack_pattern(..., multiwindow_scan)`) to the steady DATA-ACK /
SACK-suffix probe.

A new private helper `cl_arq_controller::mw_find_ack_sack_phase(rwi, tail_offset,
tail_samples, sym_samples, pattern_len, &chosen_off)`:
- gated by `mw_data_ack_multiwindow_enabled()` (reads `MERCURY_DATA_ACK_MULTIWINDOW`,
  cached on first call — no getenv in the hot loop);
- `stride = pattern_len * sym_samples` (one ACK-pattern length per phase so
  consecutive phases overlap by the full search range — no ACK can fall entirely
  between two phases);
- `MW_MAX_PHASES` derived from the held-CFG16 batch airtime
  (`data_batch_size * message_transmission_time_ms`, the quantity the late shift
  is ∝ to), clamped to the retained ring depth (`buffer_Nsymb` symbols of
  look-back) and a hard `MW_DATA_ACK_MAX_PHASES` ceiling;
- loops `ph = 1..MW_MAX_PHASES`, `off = tail_offset - ph*stride`, breaks if `off<0`;
- cheap energy pre-gate per phase (`continue` if tail RMS < `ACK_ENERGY_GATE_RMS`);
- on energetic phases, snapshots into `ready_to_process_passband_delayed_data` and
  runs `decode_ack_sack_from_passband` + the SAME CRC12 check; accepts the FIRST
  phase that decodes AND passes CRC12; sets `chosen_off`; breaks.

Call-site (`arq_commander.cc`, the MFSK SACK probe): after the newest-tail decode,
if `!decoded` (or CRC12 failed) AND the helper finds a phase, set
`tail_offset = chosen_off`, re-snapshot, re-run the EXACT existing decode body +
sanity. A genuine all-silence miss leaves `tail_offset` at the newest tail and
falls through verbatim.

---

## §7 The paired regression test

`cl_arq_controller::test_data_ack_multiwindow()` (CLI `--test-data-ack-multiwindow`),
self-contained, in-process, no IONOS/RF:
1. `load_configuration(WB config)` so the MFSK tone tables + data_container ring
   exist; assert `ack_sack_suffix_len() > 0`.
2. `generate_ack_sack_pattern_passband(burst, bsi, bitmap, crc12)` — a real
   ACK+SACK burst.
3. Place the burst at an OLDER phase in `passband_delayed_data`; fill the newest
   `tail_samples` with silence; set `ring_write_index` so the newest tail is silent.
4. **fail-before**: newest-tail decode → `decoded==false` / `matched < threshold`
   (the §1 miss).
5. **pass-after**: `mw_find_ack_sack_phase` (env ON) recovers the SAME bsi/bitmap
   with CRC12 pass at the older phase.
6. **no false-accept**: a pure-silence ring returns no phase in BOTH modes.

Returns 0=PASS, 1=FAIL. Default builds never call this.

---

## §8 Open questions

- [?] The HW whole-window parity of the climb-sim (597/3060) is not yet collapsed
  (MEMORY `_simturncal` open_note); the in-process climb-sim hot-wash here reads
  link-active rising above bench-9's 18.9%, but the absolute throughput beat is
  arbitrated by the physical bench (a separate agent owns the lease). This fix is
  NECESSARY (recovers the late reverse-ACK so the link stops stalling 81%) but the
  end-to-end VARA-beat remains conditional on the held-CFG16 acquisition chain
  (MEMORY D2/D3) which is orthogonal to this correlator fix.
