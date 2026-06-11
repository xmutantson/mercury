# data-flow: sim RX capture ring & virtual-time scroll (Track B sim fidelity)

Shared-state structure: the `-x sim` RX **capture ring** (`passband_delayed_data` +
`ring_write_index`) and the **sim virtual clock** (`sim_clock` sample counter), and
the coupling between them and the modem's reverse-ACK decode-poll cadence.

Owner of this doc: the sim-fidelity (Track B) reverse-ACK-starvation parity work.
Branch `sim/simfidelity` (worktree `C:/Users/kamer/mercury_wt/simfidelity`), base
monitor `627c370`.

> Engineering directive (user): "if you ever catch yourself saying the sim does
> NOT, then the sim NEEDS to." A sim that does not reproduce a HW behavior is a
> **fidelity bug to fix**, not a documented limitation.

---

## §1 The two anchors (HW ground truth)

Bench-9 held-CFG16 HW calibration (`bigblock_p3_hw/HW_BENCH9/ARMA_CALIBRATION.json`,
`BENCH9_VERDICT.json`):

| anchor | whole-window bps | link-active fraction | reverse-ACK | bytes |
|--------|------------------|----------------------|-------------|-------|
| **held-CFG16** (under ±8ppm drift + PTT/capture jitter) | ~597.6 | ~18.9% | matched **0/7** per batch | byte-faithful in-order (loud gap-aborts, no silent-wrong) |
| **held-CFG15** | ~3060 | ~100% | lands every turnaround | byte-faithful |

TRUE PARITY = the sim reproduces **both** rows. CFG15 already reproduces in sim
(short forward burst → reverse ACK lands). CFG16 did NOT (sim caught the late ACK,
block-success ~100%, whole-window ~2x HW).

---

## §2 The capture ring & virtual clock — producers / consumers

### §2.1 Capture ring (`passband_delayed_data`, `ring_write_index`)

**Producers (writes to `ring_write_index` / ring contents):**
- `radio_capture_prep_thread` main ring write — `audioio.c:1507-1528`. Once per
  prep iteration: writes ONE `symbol_period` chunk into the double-mapped ring at
  `ring_write_index` and advances `ring_write_index = (wi + symbol_period) % sp`.
  This is the SOLE production producer of `ring_write_index`.
- v1 reverse-ACK-starve extra scroll — `audioio.c:1549-1576` (env-gated, the
  heuristic falling-edge mechanism this work REPLACES; default-off).
- Ring reset on config switch / post-TX — `arq_common.cc` reset paths
  (`reset_radio_capture_*`, e.g. `:5076`, `:5519`, `:5704`): zero the ring and
  `ring_write_index=0`. Per `data-flow-sim2-ofdm-delivery-cadence.md §10`, the CMD's
  ring is reset at unmute, so reverse-direction turnaround audio is the only thing
  in the CMD ring during the listen window.

**Consumers (reads of `ring_write_index` / ring contents):**
- v2 OFDM decode dispatch throttle — `arq_commander.cc:2897-2945`. Reads
  `ring_write_index`, computes `advance = (rwi - last_rwi) % sp`, and only calls
  `receive()` when `advance >= min_advance_syms` (default 1 symbol). **THE poll
  cadence gate.**
- MFSK reverse-ACK/SACK tail probe — `arq_commander.cc:2723-2749`. Reads a
  FIXED-LENGTH tail `tail_samples = mfsk_tail_nsymb * sym_samples` at
  `tail_offset = signal_period - tail_samples` relative to the CURRENT
  `ring_write_index` (`:2730`, `:2737-2740`), then `decode_ack_sack_from_passband`.
  **A reverse ACK decodes ONLY if it sits in this fixed tail window at the instant
  of the snapshot.**
- OFDM frame carve / geometry — `process_main` / receive path (reads
  `frames_to_read==0` falling edge, `data-flow-sim2-ofdm-delivery-cadence.md §6`).

### §2.2 Sim virtual clock (`sim_clock` sample counter, `include/common/sim_clock.h`)

**Producer (the ONLY one):**
- `rx_transfer` — `audioio.c:1900-1901`: `if(sim_clock_enabled()) sim_clock_add_samples(len)`
  where `len == symbol_period`, called once per prep iteration (`audioio.c:1422`).
  So **virtual time advances by exactly `symbol_period` per prep iteration** — the
  SAME quantum, at the SAME cadence, as `ring_write_index`.

**Consumers:** every decision-path clock — `cl_timer` (timer.cc), `opt_now_ms()`
(arq.h), and crucially the CMD post-batch **reverse-ACK listen-window timer**
(`calculate_receiving_timeout`, arq_common.cc). All funnel through
`sim_clock_now_ns()`.

### §2.3 Relay delivery (the free-running virtual-time source on HW)

- The relay (`tools/sim/sim_channel_relay.py`) ALWAYS forwards a noise floor, even
  in inter-frame gaps (`:65-67`): "band noise does not vanish when a station stops
  keying." The TX bridge floods silence on TX gaps (`audioio.c:1811-1816`), so the
  relay receives + forwards chunks to BOTH peers continuously.
- The relay forwards in **conservative-PDES lockstep** (`sim_channel_relay.py:1373-1399`,
  BARRIER_K, default K=1): the a2b/b2a per-direction chunk counters never split by
  more than K chunks. Relaxed to K>=4 ONLY in `--turnaround-drift` mode (`:1413-1414`).
- The `--turnaround-batch-accrual` model (`:150-167`) inserts the one-sided LATE
  reverse-ACK shift into the REVERSE (b2a) turnaround GAP via the cross-direction
  TurnaroundCoupler, keeping the forward OFDM bit-exact. **This already delivers a
  LATE reverse ACK** — the missing piece is the sim modem catching it anyway.

---

## §3 Root cause (the HW↔sim fidelity gap) — from EXECUTING code

On **HW** the capture DMA scrolls `ring_write_index` in REAL TIME at the 48 kHz
audio rate, **decoupled** from the modem decode-poll cadence (the modem polls the
MFSK tail every ~1 symbol of *processing*, i.e. ~24 ms, but the DMA has written
many symbols of audio into the ring between two polls). A reverse ACK that arrives
LATE (drift/keying-latency pushes it ~143 ms past the window center per
`MEMORY.md` / `ARMA_CALIBRATION.json`) lands between two polls; by the time the
modem probes the fixed-length tail, the DMA has scrolled that audio out of the
tail window → `peak_metric≈0.0`, matched=0/7, the bench-9 597 bps collapse.

In **sim** two mechanisms force the ring scroll and the modem poll into LOCKSTEP,
so the late ACK is never displaced:

1. **Local-ADD virtual clock** (`audioio.c:1900-1901`): virtual time advances by
   `symbol_period` ONLY when the prep thread consumes a symbol — the SAME event that
   advances `ring_write_index` by `symbol_period` (`audioio.c:1527`). So the
   listen-window TIMER and the ring WRITE-INDEX advance at an IDENTICAL rate. The
   reverse ACK written into the ring at virtual-time T is still in the tail window
   when the listen-window timer (also at virtual-time T) does the probe.
2. **Conservative-PDES barrier** (`sim_channel_relay.py:1373-1399`): the relay
   delivers chunks to each peer in K-bounded lockstep, so the prep thread can never
   get "ahead" idle silence to scroll into the ring FASTER than the modem polls it.
   The inter-poll ring scroll equals exactly `min_advance_syms` (the throttle waits
   for that much advance, `arq_commander.cc:2909`) — the ring NEVER scrolls more
   than one poll's worth between polls, so a late ACK cannot be buried.

`data-flow-sim2-ofdm-delivery-cadence.md:47-53` states the same structurally:
"In PRODUCTION the capture-prep thread feeds ONE symbol per sim_paced_wait
CONCURRENTLY ... The sim collapsed that concurrency into a [serial cadence]."

**Net:** in sim, `ring_write_index` advance rate == modem poll rate. The late
reverse ACK always lands in the fixed tail snapshot → block-success ~100% →
held-CFG16 whole-window ~2x the HW 597 anchor. THIS is the fidelity bug.

### §3.1 The discriminator is the CONFIG, NOT the batch length (measured)

The v1 mechanism (and §4 v1 below) gated arming on the forward-batch LENGTH
(`muted_run >= 600 symbols`). **MEASURED FALSE** (debug build, falling-edge
muted-run histogram, `MERCURY_SIM_STARVE_DEBUG=1`):

| forward config | per-batch mute run (symbols) |
|----------------|------------------------------|
| **CFG16** (32-QAM) | ~293-299 |
| **CFG15** (16-QAM) | ~374-376 |

A fixed payload at CFG15's lower bits/symbol takes MORE symbols → the CFG15 batch
is LONGER than CFG16. So a batch-length gate (a) never fires at the v1 `>=600`
threshold (neither config reaches 600 — v1's mechanism NEVER ARMED on a real CFG16
batch), and (b) cannot discriminate CFG16 from CFG15 (the order is inverted). The
correct discriminator is the **constellation/config** (`telecom_ptr->
current_configuration == CONFIG_16`): the bench-9 anchor shows CFG16's denser
32-QAM is reverse-ACK-fragile to the turnaround de-alignment where CFG15's 16-QAM
tolerates it. The §4 mechanism arms on `current_configuration >= CONFIG_16`.

### §3.2 The relay batch-accrual is the STABLE acquisition companion (measured)

Held-CFG16 cold-acquisition under `--turnaround-drift` ALONE is ~25% reliable
(6/8 probe seeds delivered 0 bytes; only seeds 11, 42 acquired) — the documented
CFG16-acq instability (`MEMORY.md`: "cold start-cfg-16+gearshift = 0 bytes"). This
swamps any per-seed A/B. **BUT** adding the relay's `--turnaround-batch-accrual`
makes acquisition DETERMINISTIC and stable (4/4 seeds → 11625 B, ~3535-3608 bps):
the cross-direction coupler keeps the forward OFDM bit-exact so CONNECT survives,
and it delivers a LATE reverse ACK that the modem then CATCHES (the §3 gap — wire
~3535, not collapsed to 597). So the trustworthy BASE for the ring A/B is
`-s 16 --no-gearshift --turnaround-drift --turnaround-batch-accrual` (stable ~3535).

---

## §4 The fix — free-running virtual-time ring scroll (env-gated, default-off)

Per the v1 fix direction (this doc, prior §9.4/§9.5; carried forward): make the
sim RX capture ring **scroll in virtual time INDEPENDENTLY of modem consumption**,
driven from the free-running relay-delivery sample count, so trailing idle silence
scrolls a late reverse ACK OUT of the fixed tail before the next decode poll —
exactly the HW DMA mechanism.

### §4.1 Mechanism (as implemented, env `MERCURY_SIM_STARVE`, default-off)

`audioio.c radio_capture_prep_thread`, gated `starve_on` (env `MERCURY_SIM_STARVE`,
legacy alias `MERCURY_SIM_REVACK_STARVE`). ARM on the rx_mute falling edge iff
`current_configuration >= CONFIG_16` (§3.1 discriminator) AND the mute run was a
real forward batch (`>= starve_min_run`). While armed AND in the listen window
(rx un-muted), a ONE-SHOT budget (`starve_credit`, default `starve_window_syms=120`,
≤ `starve_max_syms=8`/iter) of REAL relay-delivered backlog symbols is read AHEAD
via `rx_transfer` (consumed ONCE — advancing the virtual clock by the true delivered
amount) and written into the ring, racing `ring_write_index` past the modem's
1-symbol-per-poll throttle so the trailing gap-silence that FOLLOWS the reverse ACK
scrolls the ACK back out of the fixed MFSK tail (`arq_commander.cc:2723-2740`)
before the next poll. CFG15 / control turnarounds do not arm (config gate) → the
reverse ACK lands → byte-faithful. Default-off: every new statement is under
`if(starve_on)` → byte-identical (verified: only side-effect-free `getenv` + local
var decls are unconditional; no malloc, no unconditional ring/clock write).

### §4.2 Three mechanisms tried — only this one displaces, and it is non-deterministic

1. **FIFO drain inside the capture mutex (attempt 1)** — consumed extra symbols but
   the early variant also re-`rx_transfer`'d on the same iteration paths; on BOTH
   peers it perturbed forward OFDM frame accumulation → chaotic/0-byte. Patch saved
   `_simfid2/attempt1_fifodrain.patch`.
2. **Silence-AHEAD scroll (attempt 2)** — fabricated silence into the ring ahead of
   the main write. **Does NOT displace**: the main path writes the real ACK AFTER
   the fabricated silence, so the ACK is always the NEWEST tail content → caught.
   Measured: env-ON INCREASED throughput (seed 11: 3655 vs 2604 base) — the opposite
   of starvation. Refuted.
3. **Config-gated one-shot FIFO read-ahead (attempt 3, current code)** — consumes
   the trailing gap-silence (after the ACK) into the ring, scrolling the ACK back.
   This is the only mechanism that DISPLACES the ACK. BUT it is a **thread race**:
   the prep-thread read-ahead races the arq_commander decode-poll thread for the
   capture_buffer backlog; which wins varies per run.

### §4.3 Cross-layer audit (CLAUDE.md §5)

1. **Producers** of `ring_write_index`/clock: §2.1/§2.2. The mechanism calls the
   SAME `rx_transfer` + double-mapped ring write the production path uses; consumes
   each capture_buffer symbol exactly ONCE (no double-consume).
2. **Consumers**: the throttle (`arq_commander.cc:2909`) sees a larger `advance`
   (intended); the MFSK tail probe (`:2736`) reads the post-read-ahead tail (intended
   displacement). The OFDM carve consumer is NOT reached on the CMD in the reverse-
   listen window (CMD ring reset at unmute, no forward OFDM in the CMD ring). The
   config gate keeps the RSP (forward-OFDM receiver) from ever arming.
3. **Valid states / invariants**: armed only on CFG16 falling edge + listen window +
   available backlog. Default-off → byte-identical. Clock advanced by true delivered
   count (faithful).
4. **What it changes**: the order/timing in which the CMD reads its reverse-
   turnaround audio. No fabricated signal; forward OFDM untouched.

---

## §5 Results & VERDICT — parity NOT closed (honest)

### §5.1 What the mechanism achieves (measured, WGN:40, batch-accrual stable base)

- **CFG15 PRESERVED (anchor 2 holds)**: env-ON, `-s 15`, seeds 11/42 →
  **12600 B delivered, md5 True, 2763-3042 bps** ≈ env-OFF (12600 B, md5 True,
  2784-2807 bps). The config gate leaves CFG15 byte-faithful. ✅
- **DEFAULT-OFF BYTE-IDENTICAL**: all new code under `if(starve_on)`; env-OFF
  executes zero new statements (§4.1). ✅
- **`--test-climb-engine` ALL PASS** (rc=0) — no ARQ-layer regression. ✅
- **CFG16 decouple DIRECTION lands**: env-ON does drive the held-CFG16 whole-window
  toward collapse (whole-window → 0 on the runs where the read-ahead wins the race),
  vs the stable BASE ~3535 bps. The structural lever (ring scrolls ahead of poll →
  reverse ACK misses) is real. ⚠️ (see §5.2)

### §5.2 RESIDUAL 1 — the mechanism is BISTABLE and NON-DETERMINISTIC (the blocker)

Anchor 1 (CFG16 whole-window ~597 / active ~19% / matched ~0) is **NOT reproduced**.
Two compounding reasons, both measured:

- **BISTABLE, no partial**: a window/max-syms sweep (seed 11, batch-accrual) gives
  ONLY two outcomes — total starvation (whole-window → 0) when the cumulative read-
  ahead exceeds the reverse-ACK tail span, or no effect (~2660-3535, ≈ base) below
  it. **There is NO intermediate knob landing ~597.** The HW 19%-survival GRADIENT
  does not exist in the sim because the relay's `--turnaround-batch-accrual` applies
  a FIXED per-batch late-shift (no ACK-arrival JITTER distribution) — so the modem
  either consistently catches or consistently misses.
- **NON-DETERMINISTIC**: SAME seed (42), SAME config, 3 repeats → 2731 / 2494 / 0
  bps. The read-ahead racing the decode-poll thread for the capture_buffer backlog
  makes the per-run outcome scheduling-dependent. A fidelity model MUST be
  reproducible; this is not.

### §5.3 ROOT of the residual — the fix belongs in the RELAY, not the modem ring

The 19% survival gradient is a property of the **reverse-ACK ARRIVAL-TIME
distribution** (drift + per-key-up jitter spread → ~81% of turnarounds land outside
the CMD window, ~19% inside). That distribution lives in the RELAY's turnaround
model, NOT the modem ring. The modem-side ring decouple can only answer the binary
"is the (single, fixed-time) ACK in the tail at poll time?" — it has no continuous
arrival-time distribution to convert into a 19% survival rate, and (local-ADD clock,
§2.2) no independent free-running source to drive a deterministic scroll without
racing the poll thread. **Conclusion (CLAUDE.md §2, after 3 mechanism attempts):
STOP — the parity-closing fix is to give the relay's batch-accrual a per-turnaround
ACK-arrival JITTER (e.g. Gaussian about the late-shift, σ calibrated so ~19% land
inside the CMD window) so a FRACTION of turnarounds miss DETERMINISTICALLY per
seed, yielding ~597 whole-window. The modem ring is faithful enough once the
ACK-arrival distribution is correct: a missed ACK at the relay never lands in the
tail regardless of ring timing.** The current modem mechanism is retained
env-gated + default-off as a DIAGNOSTIC that the ring CAN displace (direction
proof), explicitly NOT a parity model.

### §5.4 Held-CFG16 acquisition instability (RESIDUAL 2, pre-existing, orthogonal)

Held-CFG16 cold-acq under drift-only is ~25% reliable (§3.2). It is masked by the
batch-accrual stable base for THIS A/B but remains the documented blocker on the
natural-climb path (`MEMORY.md` CFG16-ACQ2). Orthogonal to the ring decouple.

---

## §6 Validation log

- muted-run histogram (§3.1): `_simfid2/dbg3.json` run, CFG16 ~293-299, CFG15 ~374-376.
- acq-reliability probe (§3.2): seeds 1/2/3/7/13/17 → 0 B; 11 → 11625; 42 → 3875.
- batch-accrual stable base: seeds 11/23/31/42 env-OFF → 11625 B, 3535-3608 bps.
- attempt-2 silence-ahead refutation: env-ON 3655 > base 2604 (seed 11).
- attempt-3 read-ahead: CFG16 env-ON whole-window → 0 on race-win runs; CFG15
  env-ON 12600 B md5 True (preserved).
- window sweep (bistable): win 4/8 → 0; 16/24 → ~2660; 120 → 0 (race-dependent).
- non-determinism: seed 42 env-ON ×3 → 2731 / 2494 / 0 bps.
- `--test-climb-engine` ALL PASS (rc=0) on `mercury_simfid2.exe`.
- Raw JSON + sweep outputs under `_simfid2/`.

---

## §9 v1 residuals (carried, for context)

§9.4 (v1): magnitude was knob+CONNECT-variance sensitive; the exact 597/19% was
not hit (v1 reached ~1279 whole-window, active ~45-65%). §9.5 (v1): the 19% active
fraction is partly a retx-backoff POLICY axis, not purely a ring axis; and the
held-CFG16 BASE sim is run-to-run unstable (CONNECT/acquisition flakiness),
complicating a clean A/B. This v2 work targets the ring-axis structural decouple;
the policy-axis active fraction and the CONNECT-stability residual are tracked
separately if they remain after the ring fix.
