---
status: research complete — verdict delivered, plan included
created: 2026-05-14
author: idle-scan-cadence research agent
relates_to: TIMESYNC_POLYPHASE_PLAN.md (Plan B, §7 Step-6 RPi1 idle-CPU profile)
---

# Idle-Scan Cadence — Research & Feasibility

Research-and-feasibility investigation into whether to pursue the idle-scan
**cadence** change that Plan B's §7 follow-up `[?]` flagged: Plan B made the
`FIR_rx_time_sync` filter ~M× cheaper, RPi1 idle CPU went 91 % → ~73 % (not the
~12-15 % originally projected), and the residual is floored by the fixed
`usleep(2000)` in `cl_arq_controller::process_main()`.

**PLANNING / RESEARCH ONLY.** No source file was edited. The single artifact
created is this document.

---

## §0. VERDICT — worth it at **priority 3 (low — opportunistic)**

**Pursue it, but it is not urgent and not high-value.** The change is
*architecturally clean, near-zero-risk, and costs ZERO detection latency* (§5),
which is the only reason it clears the bar at all. But the harm it removes is
**cosmetic, not functional** (§3): 73 % idle CPU on a Pi 5 causes no measured
thermal throttling, no ALSA xruns, and — critically — does **not** touch the
RX-decode burst, because the expensive idle FIR runs in the **`IDLE` state
only** and is structurally mutually-exclusive with both the `LISTENING`
call-detection path and the `CONNECTED` decode path (§1, §2).

**The two decisive facts:**

1. **(item 3 — is 73 % harmful)** The 91 %→73 % profile was captured with
   `link_status == IDLE` and **no connection**
   (`TIMESYNC_POLYPHASE_PLAN.md` §7 Step-6 profile block: "*ARQ, `-s 15`, no
   connection → `link_status == IDLE` → the `measure_signal_only` loop*"). In
   that state the modem is **not detecting calls and not decoding** — it is
   only updating a signal-strength meter. `measure_signal_only`'s entire output
   is one `double`, `measurements.signal_stregth_dbm`, consumed only by the GUI
   VU/SNR display (`telecom_system.cc:2745-2752`, `arq_common.cc:2240`,
   `:5591`). There is **no evidence of harm** in any RPi1 perf profile, timing
   log, `PI_CPU_OPTIMIZATION_REPORT.md`, or `PI_ACK_MISS_INVESTIGATION.md`: the
   ACK-miss campaign's frame corruption is a *sustained-TX* CPU-contention
   problem (`PI_CPU_OPTIMIZATION_REPORT.md:8-12`), a different state from
   `IDLE`. All RPi1 idle perf captures show `Total Lost Samples: 0`
   (`perf_rpi1_1778778847.txt:5`) — no audio drops while the idle FIR was at
   91 %.

2. **(item 5 — realistic floor + true latency cost)** The promising approach —
   a cheap raw-passband RMS energy gate every 2 ms, running the expensive FIR
   only when the gate trips — costs **0 ms detection latency** (the 2 ms loop
   cadence is untouched; only the *work inside* a quiet loop is skipped) and
   realistically reaches **~5-10 % idle CPU%** on a quiet channel (down from
   73 %). A naive longer/adaptive sleep would reach a similar floor but at a
   **direct 1:1 latency cost** (a 20 ms idle sleep adds up to 20 ms before the
   modem even looks at the channel) — and is therefore the wrong approach.

**Why priority 3 and not higher:** the win is real CPU headroom on the
RSP/decoder Pi while *idle between sessions*, which is genuinely nice for a
battery/solar-powered field deployment and for thermal margin on a fanless
enclosure — but it is preventive, not corrective. Nothing is broken at 73 %.
The ACK-miss / throughput campaign (`PI_ACK_MISS_INVESTIGATION.md`) and the
MFSK timesync polyphase work are corrective and should stay ahead of this.

**Why pursue it at all rather than "not worth it":** the energy-gate approach
is the *same prior-art pattern already proven in this codebase* (Opt 1, the
ACK-detector energy gate, `arq_common.cc:4099-4124`), it is ~30 lines, it has a
clean dual-state rollback, and it costs no latency. A near-free, in-codebase-
precedented, zero-latency change that reclaims ~65 points of idle CPU on the
field-deployed Pi is worth doing when the corrective queue is clear.

---

## §1. What `measure_signal_only()` actually does — and what it does NOT do

`cl_telecom_system::measure_signal_only(double* data)` — `telecom_system.cc:2714`.

- **It is an RMS/energy measurement, NOT a preamble correlation.** The body
  (`telecom_system.cc:2733-2751`) is exactly two steps:
  1. `ofdm.passband_to_baseband_decimated(...)` (`telecom_system.cc:2735`) —
     mix the full passband buffer to baseband and run the `FIR_rx_time_sync`
     anti-alias LPF through the polyphase `apply_decimate` primitive.
  2. `ofdm.measure_signal_stregth(...)` (`telecom_system.cc:2741`) — a plain
     `sum(re²+im²)/N` → dBm (`ofdm.cc:1641-1657`). No FFT, no correlation, no
     Schmidl-Cox, no tone matching.
- **It does NOT detect preambles or HAILs.** Comment at
  `telecom_system.cc:2716-2717`: "*No preamble detection or decoding.*" Its
  only output is the scalar `signal_stregth_dbm`
  (`telecom_system.cc:2745`, returned to `arq_common.cc:2240`).
- **The FIR runs unconditionally every 2 ms — there is NO cheap pre-gate.**
  `process_main()` (`arq_common.cc:2224-2247`) checks only
  `link_status == IDLE || link_status == DROPPED` and
  `frames_to_read == 0`, then memcpy's the whole buffer and calls
  `measure_signal_only` straight into `passband_to_baseband_decimated` →
  `apply_decimate`. **No RMS probe, no squelch, no "is there even signal"
  check** gates the FIR. Contrast the ACK detector, which *does* have such a
  gate (§4).
- **It processes the WHOLE buffer.** `ms_full_size = Nofdm * buffer_Nsymb *
  frequency_interpolation_rate` (`telecom_system.cc:2734`) — the entire ring,
  not a tail. The RPi1 call-graph profile confirms the cost:
  `measure_signal_only` = 95.29 % children, `cl_FIR::apply_decimate` = 85.97 %
  self, `passband_to_baseband_decimated` = 8.37 %
  (`perf_rpi1_1778778847.txt:14-16`).

### §1.1 The IDLE path vs the actual call-detection path — they are DIFFERENT functions

This is the structural fact that makes the energy gate latency-free.

| | IDLE / DROPPED path | LISTENING path (real call detection) |
|---|---|---|
| Trigger | `link_status == IDLE \|\| DROPPED` (`arq_common.cc:2224`) | `link_status == LISTENING` (`arq_responder.cc:107`) |
| Function | `measure_signal_only()` (`telecom_system.cc:2714`) | `receive_hail_pattern()` (`arq_common.cc:3982`) |
| Work | full-buffer mix + FIR + RMS | **tail-only** mix + FIR + MFSK matched filter (`detect_hail_pattern_from_passband`, `arq_common.cc:4009`) |
| Buffer span | whole ring, `Nofdm*buffer_Nsymb*M` (`telecom_system.cc:2734`) | tail of `hail_detect_nsymb + 24` symbols only (`arq_common.cc:3984`) |
| Detects a call? | **NO** — only a dBm meter | **YES** — this is the call-setup detector |
| Comment in code | "*No preamble detection or decoding*" (`telecom_system.cc:2717`) | "*Fast HAIL scanning while LISTENING … instead of slow receive()*" (`arq_responder.cc:104-105`) |

`link_status` enum: `DROPPED -1`, `IDLE 0`, `CONNECTING 1`, `CONNECTED 2`,
`DISCONNECTING 3`, `LISTENING 4` (`datalink_defines.h:38-43`). The `IDLE` and
`LISTENING` branches in `process_main()` / `process_messages` are **mutually
exclusive** — a process in `IDLE` is *not* running `receive_hail_pattern()`,
and a process in `LISTENING` *is* (and skips the `measure_signal_only` block
because `link_status != IDLE`, `arq_common.cc:2224`; the comment at
`:2222-2223` says exactly this: "*LISTENING has active receive() calls →
signal strength comes from receive_byte()*").

**Consequence:** optimizing the `IDLE` path's cadence **cannot delay call
detection**, because call detection does not happen in the `IDLE` state. A
field receiver armed to take calls is in `LISTENING` (entered via TCP
`LISTEN ON`, `arq_common.cc:2365-2380`, or `--monitor` auto-start,
`main.cc:1436-1448`). `IDLE` is the state of a modem that has **not** been
told to listen (`LISTEN OFF` → `link_status=IDLE`, `arq_common.cc:2387-2392`;
also the power-on default `arq_common.cc:49` before any `LISTEN ON`).
`DROPPED` is the post-session transient before the responder re-arms to
`LISTENING` (`arq_commander.cc:1133`, `:2078`, `:2271`, `:2334`; the responder
side re-arms to `LISTENING` at `arq_common.cc:1701-1706`).

**[?] Open question:** how long does a real deployment actually sit in `IDLE`
vs `LISTENING`? If an operator runs `LISTEN ON` and leaves the station armed
(the expected "waiting for a call" posture), it is in `LISTENING` and the
`measure_signal_only` hot path **is not even reached** — the 73 % figure would
not apply to that station at all. The 73 % is specifically the
"connected-to-GUI / TCP-up / not-listening / not-connected" posture. This
materially shrinks the real-world scope of the win and is the strongest
argument for the *low* priority. Worth one Pi measurement: `ps` %CPU in
`LISTENING` (idle, no incoming HAIL) — predicted to already be far below 73 %
because `receive_hail_pattern` only filters a short tail.

---

## §2. Detection-latency budget — the hard ceiling on any idle-sleep increase

This section sizes the latency ceiling **for the LISTENING path** (the one
that matters) and shows why the energy-gate approach never touches it.

### §2.1 HAIL beacon length

- HAIL detect pattern length: `hail_detect_nsymb = ack_pattern_nsymb +
  HAIL_SUFFIX_LEN` (`mfsk.cc:404`). `ack_pattern_nsymb` is mode-dependent:
  **WB M=32: 16 symbols** (`mfsk.cc:170,179`), **NB M=8: 32**
  (`mfsk.cc:189`), **NB M=4: 48** (`mfsk.cc:201`).
- The transmitted HAIL beacon is `hail_detect_nsymb` MFSK symbols long
  (`send_hail_pattern`, `arq_common.cc:3889-3892`) — i.e. WB ≈ 16 + suffix,
  NB ≈ 32-48 + suffix symbols.
- MFSK symbol period = `Nofdm * interpolation_rate` samples at 48 kHz
  (`arq_common.cc:3985-3986`, `:4085-4086`). The HAIL responder uses this to
  compute its TX-overlap wait: `sym_ms = Nofdm*interpolation_rate*1000/48000`
  (`arq_responder.cc:151-152`). Memory records WB HAIL ≈ short, NB HAIL ≈
  816 ms for the M=8 36-symbol pattern (MEMORY.md "NB HAIL Reliability").
- The responder scans for the HAIL while it is still in flight and only needs
  `hail_match_threshold` of the symbols to match — WB 7/16 (`mfsk.cc:274`),
  NB M=8 24/32 (`mfsk.cc:291`), NB M=4 40/48 (`mfsk.cc:302`). The commander
  *repeats* HAIL beacons across its connection-attempt window
  (`arq_common.cc:2285-2287` connection-attempt timer; CONNECTING re-sends per
  `arq_common.cc:1556`).

### §2.2 How much idle-loop latency the modem can absorb

For the `LISTENING` path: `receive_hail_pattern()` polls the buffer tail every
`process_main()` iteration. The audio capture thread fills the ring
continuously and independently (`arq_responder.cc:122-123`: "*The audio
callback continuously fills the buffer regardless, so audio is always
fresh*"). So a single missed poll does **not** lose audio — it only delays
*recognition*. The hard ceiling on any per-loop sleep increase is set by:
**the poll interval must stay short enough that, within the HAIL beacon
on-air time, enough polls land to catch `hail_match_threshold` symbols before
the commander gives up the attempt.** A safe rule: keep poll interval ≤ a few
symbol periods (the current 2 ms is ≈ 1/10th of even a WB symbol). The HAIL is
also *repeated*, so the true ceiling is the commander's whole
connection-attempt window (seconds), not one beacon — but designing to one
beacon is the robust choice.

### §2.3 The ceiling does not bind the recommended approach

The recommended approach (§5) **does not increase any sleep** — neither in
`IDLE` nor in `LISTENING`. It keeps the 2 ms cadence and only skips *work*
inside a loop iteration when the channel is quiet. Therefore §2.2's ceiling is
**not engaged at all**. The ceiling only matters as the reason to *reject* the
naive longer-sleep alternative.

---

## §3. Is 73 % idle CPU HARMFUL on the Pi 5, or cosmetic? — COSMETIC

Decisive for the verdict. Evidence searched: all `perf_rpi1_*.txt`, the
`pi_rpi1_*.log` timing logs, `PI_CPU_OPTIMIZATION_REPORT.md`,
`PI_ACK_MISS_INVESTIGATION.md`, project memory.

1. **No audio drops at even 91 % idle CPU.** Every RPi1 idle perf capture
   reports `Total Lost Samples: 0` (`perf_rpi1_1778778847.txt:5`,
   `perf_rpi1_1778778318.txt:5`, `perf_rpi1_1778777920.txt:5`). The 91 % pre-
   Plan-B idle profiles show ALSA functions (`ffalsa_read`, `snd_pcm_*`,
   `radio_capture_thread`) all present and at <0.1 % — i.e. the audio threads
   were getting scheduled fine even with the FIR pegged. No xrun/underrun
   symbol appears in any profile.
2. **No thermal-throttle evidence.** No `perf` capture, timing log, or fact
   doc mentions `thermal`, throttling, or frequency capping on RPi1. The Pi 5
   is a 4-core A76; the idle FIR is **single-threaded** on `process_main()`
   (profile: `99.85 % cl_arq_controller::process_main()`, all of it on `main`
   — `perf_rpi1_1778778847.txt:13-14`). One core at 73 % leaves three cores
   idle and ample thermal headroom on a Pi 5.
3. **The RX-decode burst is NOT CPU-starved by this.** The decode burst runs
   in `CONNECTED` / `RECEIVING`, where `link_status != IDLE`, so
   `measure_signal_only` **does not run** (`arq_common.cc:2224` gate). The
   idle FIR and the decode burst are temporally disjoint. The frame-corruption
   issue in `PI_ACK_MISS_INVESTIGATION.md` is explicitly a *sustained-TX*
   contention problem (`PI_CPU_OPTIMIZATION_REPORT.md:8-12`,
   `PI_ACK_MISS_INVESTIGATION.md:2-3`) and was addressed by the
   `PI_CPU_OPTIMIZATION_REPORT.md` campaign (FFT cache, FIR rewrite, ACK
   energy gate) — *those* opts touched the active-session hot path, not the
   idle loop. `PI_ACK_MISS_INVESTIGATION.md` §11-§12 conclude the residual
   ACK-miss variance is audio-path SNR and run-order setup, **not idle CPU**.
4. **The only "harm" is the wasted cycles themselves.** 73 % of one core spent
   computing a dBm number that updates a GUI meter. On mains power with a fan,
   purely cosmetic. On a solar/battery field node in a fanless box, it is
   wasted energy and thermal margin — a real but *preventive* concern, not a
   correctness or reliability bug.

**Conclusion: 73 % idle CPU is cosmetic/efficiency, not a functional hazard.**
This is the single fact that caps the priority at 3. If a Pi field-power
budget later becomes a hard constraint, re-rank upward.

---

## §4. Prior art

### §4.1 In-codebase precedent — the strongest citation (Mercury Opt 1)

Mercury **already implemented exactly this pattern** for the ACK detector.
`PI_CPU_OPTIMIZATION_REPORT.md:32-37` "Opt 1 — Energy gate on ACK detector":
"*Before each `detect_ack_pattern_from_passband` call (which runs ~528 FFTs),
do an O(N) RMS probe of the tail's last 8 symbols. If RMS < 0.001 … skip the
FFT search entirely.*" The code is `arq_common.cc:4099-4124`:

```
const double ACK_ENERGY_GATE_RMS = 0.001;
int probe_n = 8 * sym_samples;
... double sumsq = 0.0; for(...) sumsq += tail_ptr[i]*tail_ptr[i];
double tail_rms = std::sqrt(sumsq / probe_n);
if(tail_rms < ACK_ENERGY_GATE_RMS) { ... return false; }   // skip FFT search
```

It was measured to *raise* throughput (`PI_CPU_OPTIMIZATION_REPORT.md:66`:
"F13 (+Opt 1 energy gate) … +23 % vs F12") because the freed CPU stabilised
the audio path. The threshold rationale — noise floor ≈ -73 dBFS ≈ 0.0002 RMS,
a real MFSK tone ≈ 0.02 RMS, gate at 0.001 (≈14 dB above noise, well below the
smallest real signal) — is documented at `arq_common.cc:4100-4107` and is
**directly reusable** for the idle gate (same passband units, same buffer).
This is two-stage detection (cheap energy gate → expensive correlation) and it
is already shipping in Mercury.

### §4.2 External prior art — two-stage / squelch-gated detection

- **Squelch / VOX-gated processing** is the universal HF/VHF practice: a cheap
  power/energy estimate gates the expensive demod. ARDOP (`Rick Muething,
  KN6KB — "ARDOP Specification"`) uses a leading-edge energy detector to arm
  the 4FSK/PSK frame search rather than correlating continuously. VARA
  similarly only engages its OFDM acquisition after channel-busy/energy
  detection (behaviourally observable; closed source).
- **codec2 / FreeDV** — in this workspace at `hermes-modem/` (codec2-based).
  FreeDV's modems separate a cheap timing/energy estimate from the expensive
  per-frame demod; `freedv_api` callers gate `freedv_rx()` work behind sync
  state. The relevant pattern: `cohpsk`/`ofdm` modems track a `sync` flag and
  only do full per-carrier equalisation/derotation when sync is plausible,
  running a lighter timing-recovery estimate otherwise (`codec2/src/ofdm.c`
  `ofdm_sync_search` vs the full `ofdm_demod` — search is the cheap stage).
- **GNU Radio** — the canonical block is `blocks.threshold_ff` /
  `analog.pwr_squelch_cc` ahead of a correlator
  (`digital.corr_est_cc` / `digital.correlate_access_code_*`): the squelch
  zeroes/gates the stream so the correlator's downstream work collapses when
  the channel is quiet. GNU Radio's `gr-digital` preamble/access-code
  detection is explicitly designed as "cheap gate → expensive correlate"
  (GNU Radio docs, `gr-digital` correlation estimator).
- **Adaptive duty-cycling** (longer sleep when quiet, shorter when active) is
  used by low-power beacon/APRS trackers, but it is the *latency-costly*
  option and is the one to **avoid** here (§5.2).

**Citations summary:** the in-codebase Opt 1 (`PI_CPU_OPTIMIZATION_REPORT.md`,
`arq_common.cc:4099-4124`) is the primary, already-validated source. ARDOP
spec, FreeDV/codec2 `ofdm.c` sync-search-vs-demod split, and GNU Radio
`pwr_squelch` + `corr_est` are the external confirmations that two-stage
energy-gated detection is standard practice.

---

## §5. The recommended approach and its REAL latency cost

### §5.1 Recommended: cheap raw-passband RMS energy gate in the IDLE path

Mirror Opt 1 (§4.1) into the `process_main()` IDLE block
(`arq_common.cc:2224-2247`). Before the `measure_signal_only()` call, do an
O(N) raw-passband RMS probe of the just-memcpy'd buffer (it is already in
`ready_to_process_passband_delayed_data`, `arq_common.cc:2234-2236` —
**no extra copy**, no FIR). If RMS is below a quiet-channel gate, skip
`measure_signal_only()` entirely this loop and just publish the last
`signal_stregth_dbm` (or a floor value).

- **Latency cost: ZERO.** The `process_main()` loop still runs every
  `usleep(2000)` = 2 ms. The 2 ms cadence — the thing that governs
  responsiveness — is **untouched**. Only the *work inside a quiet loop* is
  skipped. And critically (§1.1, §2.3) this path does **not detect calls**
  anyway, so even a hypothetical latency hit here could not delay call setup.
  The energy gate is pure dead-weight removal.
- **Realistic idle-CPU floor: ~5-10 %.** When the channel is quiet (the common
  idle case), the per-loop work collapses to: one memcpy + one O(N) RMS sum +
  `usleep(2000)` + `process_messages()`. The RMS sum over the buffer is
  cheap relative to the FIR (the FIR is ~94 % of `measure_signal_only`,
  `perf_rpi1_1778778847.txt:15-16`; an RMS pass is comparable to the
  `measure_signal_stregth` step which is only 0.87 % self). With the FIR gone
  on quiet loops, CPU% is dominated by the 2 ms sleep → the ratio falls toward
  the `process_messages` + memcpy + RMS floor, realistically **~5-10 %**.
  When the channel is *busy* it climbs back toward 73 % — correct behaviour:
  spend CPU only when there is signal to measure.
- **The probe should be raw passband, not baseband.** Opt 1 probes the raw
  passband tail (`arq_common.cc:4111-4117`) precisely to avoid the FIR. The
  idle gate must do the same: RMS of `ready_to_process_passband_delayed_data`
  directly, *before* any `passband_to_baseband*` call. (Threshold note: the
  raw passband RMS floor differs from the post-FIR baseband floor, so the gate
  constant must be measured fresh on the Pi, not copied verbatim from Opt 1's
  `0.001` — see Step 2 below. This is a *calibration*, not a band-aid: the
  gate must sit comfortably below the weakest real signal and above the
  rx-mute/silence floor, exactly as Opt 1's comment derives it.)

### §5.2 Rejected alternative: naive longer / adaptive idle sleep

Increase `usleep(2000)` to e.g. `usleep(20000)` when `link_status == IDLE`, or
ramp it adaptively.

- **Latency cost: DIRECT 1:1.** A 20 ms idle sleep adds up to 20 ms before the
  loop runs at all. In `IDLE` that only delays the dBm meter — harmless. *But*
  the same loop also calls `process_messages()` and the TCP `check_incomming_
  connection()` paths (`arq_common.cc:2104-2109`, `:2249`); slowing the whole
  loop slows control-socket responsiveness and any IDLE→CONNECTING/LISTENING
  transition. And it sets a fragile precedent: if anyone later routes a
  detection path through the IDLE loop, the latency is silently there.
- **Realistic floor:** a 20 ms sleep gives `T/(T+20)` — with the post-Plan-B
  ~7 ms FIR that is ~26 %; to reach ~10 % needs a ~60 ms sleep. The floor is
  *worse* than the energy gate AND it costs latency. Strictly dominated.
- **Verdict:** reject. This is the "tune a timeout to mask the symptom" anti-
  pattern CLAUDE.md §2 / `feedback_no_threshold_bandaids` warns against — the
  root issue is "the FIR runs when there is nothing to measure", and the
  energy gate fixes *that*, structurally.

### §5.3 Optional refinement (defer): also tail-scope the IDLE FIR

`measure_signal_only` processes the **whole** buffer
(`telecom_system.cc:2734`), whereas `receive_hail_pattern` /
`receive_ack_pattern` process only a tail. For a *signal-strength meter* a
tail (e.g. last 8-16 symbols, like Opt 1's probe) is entirely sufficient and
would shrink the *non-quiet* FIR cost ~`buffer_Nsymb/16`× as well. This is a
clean additional win but is **out of scope** for the cadence question and
should be its own follow-up — the energy gate alone delivers the headline
result. Listed here so it is not lost.

---

## §6. Reversible-action implementation plan

Mirrors the step format of `TIMESYNC_POLYPHASE_PLAN.md` §7. Each step:
**Action / Test / Rollback.** Steps are ordered so the diagnostic comes first
and the behaviour change is a single isolated edit. **No source file is edited
by this document — this is the plan to be approved before implementation.**

`interpolation_rate == frequency_interpolation_rate == M`, M = 4 in current
builds (`TIMESYNC_POLYPHASE_PLAN.md` §10).

### Step 0 — Instrument: log idle-loop FIR-run rate and raw-passband RMS

- **Action:** In the `process_main()` IDLE block (`arq_common.cc:2224-2247`),
  behind a compile guard (`#ifdef IDLE_GATE_TRACE`, file-static counters — see
  the ODR hazard note in `TIMESYNC_POLYPHASE_PLAN.md` §10, do **not** add
  members), compute the raw-passband RMS of
  `ready_to_process_passband_delayed_data` each loop and log, every ~5 s, the
  RMS distribution and the count of loops. Also add the same `#ifdef` env hook
  to `build.sh` as Plan B's `TIMESYNC_TRACE`.
- **Test:** Build with the define is the only build that changes; build
  *without* the define must be byte-identical to baseline (the Plan B Step-0
  byte-identical check). Deploy to RPi1, run `ARQ -s 15` with no connection
  (`link_status == IDLE`) for 60 s; confirm the trace prints a stable
  quiet-channel RMS and that it is well-separated from a deliberately-injected
  test tone's RMS (use `tools/` audio injection or the IONOS butler `WGN` /
  tone playback).
- **Rollback:** Remove the `#ifdef` block + the `build.sh` hook. Zero risk —
  diagnostic only.
- **Purpose:** turns the gate threshold from a guess into a *measured* value
  (CLAUDE.md §3 "no untested fixes"; §2 "calibrate, don't band-aid").

### Step 1 — Add a failing test that captures the bug

- **Action:** Add a unit/integration check to the `mercury.exe --test` suite
  (or a `tools/` harness): drive `process_main()` (or `measure_signal_only`'s
  caller path) with a **silent** passband buffer for N iterations in
  `link_status == IDLE`, and assert that the expensive FIR
  (`passband_to_baseband_decimated`) is **not** invoked — e.g. via a
  call-counter hook or by timing the loop against a known-quiet vs known-
  signal buffer. **Before the fix this test FAILS** (the FIR runs every loop).
- **Test:** Confirm the new test fails on the current `HEAD` binary.
- **Rollback:** Remove the test. Self-contained.

### Step 2 — Pick the gate threshold from Step-0 data

- **Action:** From the Step-0 RPi1 trace, set `IDLE_ENERGY_GATE_RMS` to sit
  ≥ ~10-14 dB above the measured quiet-channel raw-passband RMS floor AND
  comfortably below the weakest real on-air signal RMS (cross-check against
  the Opt 1 derivation at `arq_common.cc:4100-4107`, adjusted for raw-passband
  vs post-FIR-baseband units). Document the derivation in a code comment, as
  Opt 1 does.
- **Test:** No code-behavior change yet — this step only fixes a constant
  value to be used in Step 3. Sanity-check: the chosen gate must classify the
  Step-0 quiet captures as "quiet" and the injected-tone captures as "signal"
  with margin.
- **Rollback:** N/A (no edit yet — this is the value decision).

### Step 3 — Add the energy gate to the IDLE path (the behaviour change)

- **Action:** In `process_main()` IDLE block, between the buffer memcpy
  (`arq_common.cc:2234-2236`, after `MUTEX_UNLOCK`) and the
  `measure_signal_only()` call (`arq_common.cc:2240`): compute the raw-
  passband RMS of `ready_to_process_passband_delayed_data` over the buffer
  (or a tail — see §5.3, but keep this step minimal: whole-buffer RMS is still
  ~FIR/15 cheap). If RMS `< IDLE_ENERGY_GATE_RMS`, **skip**
  `measure_signal_only()` and instead publish a floor/last-value
  `signal_stregth_dbm` (decide: hold last value, or report a noise-floor dBm —
  holding last value matches GUI expectations better). Otherwise call
  `measure_signal_only()` as today. ~25-30 lines, single function, mirrors
  `arq_common.cc:4099-4124` structurally.
- **Test:**
  1. The Step-1 test now **PASSES** (FIR skipped on silent buffer).
  2. `mercury.exe --test` full suite passes.
  3. Host loopback smoke (`-m ARQ -s 10 -M auto …`) — connection setup,
     data transfer, disconnect all still work (the gate is in the IDLE path
     which a loopback session passes through at start/end).
  4. **RPi1 idle-CPU re-measure** with the exact methodology of
     `TIMESYNC_POLYPHASE_PLAN.md` §7 Step-6 (`ps` %CPU, 6 samples,
     `ARQ -s 15`, no connection): expect **~5-10 %** on a quiet bench channel
     vs the current ~73 %. Re-profile with `perf` to confirm
     `apply_decimate` / `passband_to_baseband_decimated` are GONE from the
     quiet-channel idle profile.
  5. **Functional non-regression — the meter still works:** with a signal
     present on the channel, confirm `signal_stregth_dbm` still tracks (GUI VU
     responds) — i.e. the gate trips and `measure_signal_only` runs when there
     IS signal.
- **Rollback:** Revert the single `process_main()` IDLE-block edit — the gate
  is one self-contained `if`. The modem returns to running the FIR every loop.
  Independent of Steps 0/1 (diagnostics) which can stay or go separately.

### Step 4 — (optional, defer) tail-scope `measure_signal_only`

- **Action:** Per §5.3, change `measure_signal_only` (or add a tail-scoped
  variant) to process only a tail of the buffer for the meter. **Separate
  decision** — do not bundle into the cadence change.
- **Test:** meter value vs whole-buffer value within tolerance on a steady
  signal; idle profile shows the *non-quiet* FIR cost dropped too.
- **Rollback:** revert the `measure_signal_only` body — independent of Step 3.

### Step 5 — Remove instrumentation

- **Action:** Delete the Step-0 `#ifdef IDLE_GATE_TRACE` blocks, the file-
  static counters, and the `build.sh` hook.
- **Test:** Clean build; final loopback + idle-CPU smoke. Binary should be
  byte-identical to the Step-3 production build (proof the trace was fully
  `#ifdef`-excluded — same check Plan B Step 7 used).
- **Rollback:** Trivial — re-add the diagnostic block.

---

## §6.E. Execution log (implementation — branch `monitor`, from HEAD `2022256`)

### Step 0 — Instrument idle-loop FIR-run rate + raw-passband RMS — DONE
- **Commit:** _(see below — committed immediately after this block)_
- **What landed:**
  - `build.sh`: added `IDLE_GATE_TRACE` env hook (`IDLE_GATE_TRACE=1 bash
    build.sh o3` → `-DIDLE_GATE_TRACE`), mirroring the removed Plan-B
    `TIMESYNC_TRACE` hook.
  - `arq_common.cc` `process_main()` IDLE block: `#ifdef IDLE_GATE_TRACE`
    block computes the raw-passband RMS of
    `ready_to_process_passband_delayed_data` over the whole `signal_period`
    buffer (the exact buffer the Step-3 gate will probe), before
    `measure_signal_only()`. File-static counters only (`idle_gate_fir_runs`,
    running RMS sum/min/max/n, `idle_gate_last_ms`) — no class members (ODR
    hazard per Plan B §10). Emits `[T] idle_gate_trace ...
    fir_runs=.. n=.. rms_mean=.. rms_min=.. rms_max=.. rms_last=..` every ≥5 s.
- **Test — byte-identical-without-define:** PASS.
  - `IDLE_GATE_TRACE=1 bash build.sh o3` compiles clean.
  - Clean build *without* the define vs clean build of HEAD `2022256`:
    full `.exe` differs by 2.64 M bytes — **but that is entirely DWARF
    debug-line-number tables** (the `#ifdef` block shifts ~40 source lines, and
    o3 mode builds with `-g`). After `strip`, the two binaries are **0 bytes
    different / byte-identical** (5,403,662 bytes each). Control: two clean
    HEAD builds differ by only 2 bytes (PE-header timestamp at 0x88/0xd8),
    confirming the toolchain is otherwise deterministic. So the Step-0
    instrumentation is provably compiled out of the default build (machine
    code identical; only debug line tables move). This matches Plan B's
    Step-0 "byte-identical" contract (code-identical modulo debug info).
- **Rollback:** revert the `build.sh` hook + the `#ifdef IDLE_GATE_TRACE` block.

### Step 1 — Add a failing test that captures the bug — DONE
- **Commit:** _(see below — committed immediately after this block)_
- **Note on test methodology — RECONCILED WITH CODEBASE REALITY:** Step 1 as
  written ("add a unit/integration check to the `mercury.exe --test` suite")
  rests on a `--test` harness that **does not exist** — CLAUDE.md / MEMORY.md
  reference `mercury.exe --test` but `main.cc` has no such mode, and
  `MFSK_TIMESYNC_POLYPHASE_PLAN.md` §7 states plainly "Mercury has no
  source-level unit-test harness." The in-codebase precedent for a unit test
  is a standalone compiled `tools/*.cc` linked against the production source
  (`tools/test_b2f_handler.cc`). `process_main()` itself is not unit-testable
  standalone (it pulls the whole telecom_system + ofdm + FIR + sockets graph,
  and there is no file-input audio mode — audio is always live device I/O).
- **What landed:** `tools/test_idle_energy_gate.cc` — a standalone test of the
  production gate predicate `idle_energy_gate_open(buf, n, gate_rms)` that
  Step 3 will introduce in `include/datalink_layer/idle_energy_gate.h` and that
  `process_main()` will call to gate the FIR. The test asserts the bug-capturing
  contract: a silent / sub-threshold buffer keeps the gate CLOSED (FIR skipped),
  a signal-present buffer OPENs it (FIR runs), and the decision is monotonic in
  RMS at the threshold.
- **Test — RED on HEAD `2022256`:** PASS (it fails as required).
  `g++ -O2 -std=c++14 -I./include -o tools/test_idle_energy_gate.exe
  tools/test_idle_energy_gate.cc` → `fatal error: datalink_layer/idle_energy_gate.h:
  No such file or directory`. On HEAD there is **no gate predicate and no gate**
  — the FIR runs on every silent IDLE loop. The test references the production
  API the fix must introduce; it cannot build until Step 3 adds it, and once it
  does, the test exercises the *same* predicate `process_main()` uses. This is a
  genuine red→green: red = won't build (no gate exists), green = builds + all
  cases pass (Step 3).
- **Integration proof** that the FIR is actually skipped *inside the live
  `process_main()` loop* (not just that the predicate is correct) is the
  `IDLE_GATE_TRACE` `fir_runs` counter measured on RPi1 — Step 0 / Step 3.
- **Rollback:** delete `tools/test_idle_energy_gate.cc`. Self-contained.

### Step 0/2 — Pi RMS measurement + threshold calibration
- _(in progress / see RESULT block appended below once Pi data is captured)_

---

## §7. Open questions [?]

- **[?]** How long does a real deployment actually sit in `IDLE` vs
  `LISTENING`? A station armed with `LISTEN ON` is in `LISTENING` and never
  reaches the `measure_signal_only` hot path — so the 73 % only applies to the
  "TCP up, not listening, not connected" posture. One Pi `ps` measurement in
  `LISTENING` (idle, no HAIL) would size the *real* scope of this win. (This
  is the main reason for priority 3 rather than 2.)
- **[?]** Should the gated-out loop *hold* the last `signal_stregth_dbm` or
  report a noise-floor value? Holding the last value matches GUI VU
  expectations but can show a stale "signal present" reading after a signal
  ends. Reporting a floor dBm is more truthful but makes the meter twitch.
  Decide during Step 3 with the GUI behaviour in view.
- **[?]** Is the raw-passband RMS floor on RPi1 stable enough across
  rx-mute / no-rx-mute and across the Fe-Pi gain settings that one constant
  works? Step 0 answers this; if the floor moves with gain, the gate may need
  to be relative to a tracked floor (as Opt 1's comment hints the noise floor
  is "measured").
- **[?]** Does `DROPPED` ever persist long enough to matter? It is documented
  as a post-session transient (`arq_commander.cc:1133` etc.) before re-arming
  to `LISTENING`/`IDLE`. If it is always sub-second the `DROPPED` half of the
  `arq_common.cc:2224` gate is irrelevant to the CPU question; the gate covers
  it for free regardless.

---

## §8. Files referenced

- `mercury/source/datalink_layer/arq_common.cc` — `process_main()` idle loop
  `:2039-2251` (IDLE block `:2224-2250`, `usleep(2000)` `:2250`);
  `receive_hail_pattern()` `:3982-4053`; `receive_ack_pattern()` energy gate
  (Opt 1 prior art) `:4060-4124`; `send_hail_pattern()` `:3876-3979`;
  `LISTEN ON/OFF` `:2365-2399`; power-on `link_status=IDLE` `:49`.
- `mercury/source/physical_layer/telecom_system.cc` — `measure_signal_only()`
  `:2714-2752`.
- `mercury/source/physical_layer/ofdm.cc` — `measure_signal_stregth()`
  `:1641-1657`; `passband_to_baseband_decimated()` `:3917-3952`.
- `mercury/source/physical_layer/fir_filter.cc` — `apply_decimate()`
  `:233-295`.
- `mercury/source/physical_layer/mfsk.cc` — `ack_pattern_nsymb` /
  `hail_match_threshold` / `hail_detect_nsymb` `:170-201,274-302,404-405`.
- `mercury/source/datalink_layer/arq_responder.cc` —
  `process_messages_rx_data_control()` HAIL scan `:102-208` (LISTENING vs IDLE
  distinction `:104-107,122-123,151-152`).
- `mercury/include/datalink_layer/datalink_defines.h` — link-status enum
  `:38-43`.
- `mercury/source/main.cc` — `--monitor` LISTENING auto-start `:1436-1448`.
- `mercury/fact-documents/TIMESYNC_POLYPHASE_PLAN.md` — Plan B; §7 Step-6
  RPi1 idle-CPU profile + the "~12-15 % was an over-projection" correction
  `:823-863`; §10 ODR hazard / M=4 facts.
- `mercury/fact-documents/PI_CPU_OPTIMIZATION_REPORT.md` — Opt 1 energy-gate
  prior art `:32-37,66`; sustained-TX (not idle) motivation `:8-12`.
- `mercury/fact-documents/PI_ACK_MISS_INVESTIGATION.md` — ACK-miss is
  sustained-TX / audio-path SNR, not idle CPU `:2-3,§11-§12`.
- `mercury/fact-documents/timing_data/perf_rpi1_1778778847.txt` — post-Plan-B
  idle profile (`measure_signal_only` 95.29 %, `apply_decimate` 85.97 %,
  `Total Lost Samples: 0`).
- `mercury/fact-documents/timing_data/perf_rpi1_1778778318.txt`,
  `perf_rpi1_1778777920.txt` — pre-Plan-B idle profiles (`cl_FIR::apply`
  ~92 %, `Total Lost Samples: 0`).
