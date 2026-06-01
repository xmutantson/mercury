# Fact Document: CONNECT/ACK detection metric-gate relaxation

**Branch:** `wt/connect-metric` (off inner monitor `01535f2`)
**Created:** 2026-05-31
**Goal:** Relax the hardcoded `metric >= 3.0` soft energy-ratio sub-gate on
CONNECT/ACK MFSK detection to deepen acquisition (the weak-signal floor is
acquisition-limited by the CONNECT handshake, not the data PHY), WITHOUT
hurting data throughput. RX-detection-only change; no wire/TX/symbol-rate
change.

This document is the CLAUDE.md §5 cross-layer data-flow audit for the metric
gate. It owns the producer/consumer map for the gate and the
THROUGHPUT-SAFETY verdict.

---

## §1 What the metric is (the gated quantity)

`cl_ofdm::detect_ack_pattern` (`source/physical_layer/ofdm.cc:3691`) is the
discrete FFT-bin-argmax matched filter shared by ALL MFSK control patterns
(CONNECT, ACK, HAIL, BREAK, SACK suffix). For each candidate symbol-grid
position it computes, per symbol `p`:

- **match** (a count): the expected tone is the argmax peak bin for ALL
  `nStreams` streams (energy-gated so silence cannot 0==0 match) —
  `ofdm.cc:3796-3803`. `matched` ∈ [0, ack_nsymb].
- **metric** (a soft ratio): `metric += e_target / e_total` accumulated ONLY
  over matched symbols, where `e_target` = energy in the expected tone bin
  (+ its carrier image), `e_total` = energy across all `Nc` occupied
  subcarriers — `ofdm.cc:3808-3823`. `metric` ∈ [0, ack_nsymb].

Returned via `*out_matched` (count) and the function return value (metric).

**The gate** (at all 4 sites) is the AND of three conditions:
```
matched >= match_threshold   (the HARD count gate, e.g. 7/16 — strong FAR defense)
&& metric >= 3.0             (the SOFT energy-ratio sub-gate — THIS is what we relax)
&& best_offset >= 0
```
`match_threshold` is `ack_match_threshold` / `connect_match_threshold`,
both **7** for WB M=16 (`mfsk.cc:230,399`), with documented per-poll false
probability P(false|M=16) ≈ 2.4e-5 to 2.5e-7 from the count gate alone.

**Why metric is the binding gate at the cliff:** the acquisition-cliff
diagnosis measured that at the WGN:-10 floor the discrete tone-match count is
~14/16 (≫ the count threshold of 7) — frames are rejected PURELY on the soft
ratio. Random noise produces metric ≈ matched/Nc per the existing in-code
comment (`arq_common.cc:5685`: "Random noise has metric≈8/Nc=0.16 at 8
matches", Nc=50 WB). A real weak preamble sits well above noise's 0.16 but can
dip under 3.0 at low SNR → the 3.0 gate, not the count, is the floor.

---

## §2 The four hardcoded `metric >= 3.0` sites (the producers of the gate decision)

All in `source/physical_layer/telecom_system.cc`:

| # | Line | Function | Path | Form |
|---|------|----------|------|------|
| 1 | 3264 | `detect_ack_snr_from_passband` | ACK base detect (pass 1) | `metric < 3.0` → return -99 |
| 2 | 3324 | `detect_ack_snr_from_passband` | ACK base re-detect after mini-Moose CFO re-mix (pass 2) | `remetric >= 3.0` → accept refined offset |
| 3 | 3535 | `decode_ctrl_suffix_from_passband` | CONNECT base detect (pass 1) | `metric < 3.0` → return false |
| 4 | 3573 | `decode_ctrl_suffix_from_passband` | CONNECT base re-detect after mini-Moose CFO re-mix (pass 2) | `remetric >= 3.0` → accept refined offset |

Sites 2 & 4 are the second-pass acceptance of a CFO-corrected re-detection;
they gate only whether the *refined* offset replaces the original — on failure
the original offset/baseband is still used downstream. They must use the SAME
threshold as their pass-1 sibling (1 & 3 respectively) or the re-detect could
be rejected while the pass-1 detect passed, leaving a worse-aligned offset.
**→ all 4 sites move together to one named constant.**

### §2.1 The raw (un-gated) detector — `detect_ack_pattern_from_passband`
`telecom_system.cc:3116` returns the raw metric with NO 3.0 gate. Its only
consumer that gates on metric is the **normal-mode ACK path**
(`arq_common.cc:5670,5689`), which applies the SEPARATE, CLI-overridable
`ack_metric_threshold` — **already lowered from 3.0 → 0.5** in commit 7076a4b
(`arq_common.cc:392`, comment: "3.0 caused ~50% timeouts"). This path is NOT
one of the 4 hardcoded sites and is NOT changed here; it is prior art proving
a sub-3.0 ACK metric gate has shipped without a FAR regression.

---

## §3 Consumer map (every code path that consumes a gated detection)

### §3.1 CONNECT — `decode_ctrl_suffix_from_passband` (sites 3,4)
Single ARQ consumer chain:
- `arq_common.cc:5024` `receive_mfsk_ctrl_suffix_phy_core()` calls it, then applies
  **downstream guards** before any ARQ state mutation:
  1. **Type discriminator** — `rx_type != expected_type` → reject
     (`arq_common.cc:5038`). Random noise yields a ~uniform 2-bit type; P(match) ≤ 1/4.
  2. **CRC12 validation** — `rx_crc12 != CRC12_calc(typed_bytes,5)` → reject
     (`arq_common.cc:5058-5059`). 12-bit CRC over the typed-40 payload; P(random
     pass) ≈ 2^-12 ≈ 2.4e-4.
  3. **Payload unpack** — `unpack_start_conn_payload` must succeed and yield a
     callsign the handshake state machine accepts (`arq_common.cc:5097`,
     `receive_mfsk_start_conn_phy`).
- Only after ALL three does it set `*out_p38` and flush the audio window
  (`arq_common.cc:5071-5081`). State touched: handshake `messages_rx_buffer`
  synthesis at the START_CONN site — i.e. it can only *start a connection*, and
  only if a valid callsign survives type+CRC. A false CONNECT cannot corrupt an
  in-flight data batch (no batch exists pre-CONNECT).

### §3.2 ACK turbo-SNR — `detect_ack_snr_from_passband` (sites 1,2)
- `arq_common.cc:5546` (CMD-only, `turbo_snr_ack_enabled` branch of
  `receive_ack_pattern`). **Acceptance is keyed on the COUNT gate**
  `matched_count >= ack_match_threshold` (`arq_common.cc:5550`), NOT on the
  metric. `*out_matched` is written by `detect_ack_pattern` *before* the
  metric early-out (`telecom_system.cc:3261` then 3264), so `matched_count`
  carries the TRUE count even when the function returns -99 for metric<3.0.
  - Effect of the 3.0 gate here today: when count≥7 but metric<3.0, the
    function returns -99 with `snr_valid=false` → the ACK is still ACCEPTED
    via the "suffix timeout" path after ≤500 ms (`arq_common.cc:5616-5645`),
    just without an SNR readout. So at this site the 3.0 gate mostly **denies/
    delays the SNR suffix**, it does not block ACK acceptance. Relaxing it lets
    the SNR read succeed sooner (removes the 500 ms defer) — a latency/turbo
    improvement, not an ACK-admission change.

### §3.3 ACK+SACK — `decode_ack_sack_from_passband` (inherits sites 1,2)
`decode_ack_sack_from_passband` (`telecom_system.cc:3426`) calls
`detect_ack_snr_from_passband` internally (`:3437`) → inherits the same 3.0
gate, then requires `last_ack_sack_capture_valid` + a clean 13-symbol suffix
decode. Two ARQ consumers, each with FOUR independent downstream guards:
- `arq_commander.cc:105` (clean-batch ACK arm) and `arq_commander.cc:2548`
  (SACK window):
  1. `decode_ack_sack_from_passband` false unless count+metric+capture pass.
  2. **CRC12** over [bsi||bitmap] (`arq_commander.cc:118`, `:2565`). P≈2^-12.
  3. **BSI sanity** — bsi must equal current or just-prior batch mod 256
     (`arq_commander.cc:125`, `:2583`).
  4. **Bitmap shape** — clean-batch arm requires all-ones
     (`arq_commander.cc:131-134`); SACK arm validates against the live batch.

### §3.4 Consumers NOT affected
- Normal-mode ACK (`arq_common.cc:5689`) — uses `ack_metric_threshold` (0.5),
  not a hardcoded 3.0. Untouched.
- HAIL / BREAK detection — separate detectors with their own count thresholds;
  do not route through the 4 sites.
- All DATA-PHY decode (OFDM symbol demap, LDPC, equalizer, channel estimation)
  — the 4 sites are in the MFSK control-frame RX path ONLY. No site touches
  symbol rate, constellation, FEC, GI, or any data-bearing decode.

---

## §4 The §5 five-question audit

**1. Producers (writers of the gate decision):** the 4 sites in §2. They emit a
boolean "this is a valid CONNECT/ACK detection" by combining count+metric+offset.

**2. Consumers (readers of a gated detection):** §3.1–§3.3 — exactly the CONNECT
handshake-start path and the ACK / ACK+SACK acceptance paths in
arq_common/arq_commander. No other reader.

**3. Valid states / pre-producer state:** before any producer fires, ARQ sits in
its handshake or data-wait state; a *false* detection is the only way the gate
can hand a consumer a frame that was not actually sent. The relaxation widens
the set of buffers the producer calls "detected", so the central question is
purely: **does relaxing metric admit false detections that reach a state
mutation?**

**4. Invariants the consumers assume:** every consumer assumes "a detection that
reaches my state mutation corresponds to a real transmitted frame." This
invariant is NOT enforced by the metric gate — it is enforced by the
**downstream CRC12 + type + BSI + bitmap guards** (§3.1–§3.3). The metric gate
is a cheap pre-filter to avoid running the suffix decode on obvious noise; it is
NOT the false-alarm defense of record. The count gate (7/16, P≈2.4e-5..2.5e-7)
plus the 12-bit CRC (P≈2.4e-4) are the load-bearing defenses, and they are
UNCHANGED by this fix.

**5. What the fix changes:** metric threshold 3.0 → a lower named constant.
Walking each consumer:
- CONNECT (§3.1): a relaxed gate can only let more *candidate* CONNECT buffers
  reach the type+CRC12+unpack guards. To corrupt state, noise must pass count≥7
  AND metric≥(new) AND a clean suffix AND a valid 2-bit type AND a 12-bit CRC AND
  a valid callsign. The metric relaxation moves only the second term; the
  compound FAR is dominated by CRC12·type ≈ 2^-12·(1/4) regardless. ✔ safe.
- ACK turbo-SNR (§3.2): acceptance is on COUNT, not metric — relaxing metric does
  not change which ACKs are admitted; it only lets the SNR suffix read succeed
  without the 500 ms defer. ✔ safe (in fact a small latency win).
- ACK+SACK (§3.3): four downstream guards (CRC12+BSI+bitmap) dominate the FAR.
  Metric relaxation only changes how many candidates reach them. ✔ safe.

**No consumer's invariant is violated** because no consumer relies on the metric
gate for correctness — they rely on CRC12/type/BSI/bitmap, all unchanged.

---

## §5 THROUGHPUT-SAFETY VERDICT (the user's hard gate)

**Verdict: relaxing the metric gate from 3.0 to the chosen named constant
cannot hurt data throughput.** Three independent reasons:

1. **No data-PHY surface is touched.** The 4 sites are in the MFSK
   control-frame RX correlator path. Nothing about the OFDM data symbol rate,
   constellation, LDPC, equalizer, GI, or framing changes. A clean/good-SNR
   data batch decodes bit-for-bit identically (verified by the
   throughput-neutrality test — the data path code is byte-identical to
   baseline). This is the dominant argument.

2. **The only throughput risk is false alarms, and it is bounded to ~0 by
   layers the fix does NOT change.** A noise false-alarm that reached an ARQ
   state mutation would waste airtime / corrupt ARQ state. But every consumer
   gates the *acceptance* on CRC12 (P≈2^-12) + type/BSI/bitmap, NOT on the
   metric. The metric is a pre-filter. Even a metric gate of 0.0 would leave
   the CRC12 defense fully intact. The FAR sweep (§6) additionally confirms the
   *detection-stage* FAR stays ~0 at the chosen threshold over ≥5000
   noise-only trials, so the suffix decode is not even run on noise in
   practice.

3. **Prior art already shipped a sub-3.0 ACK metric gate (0.5) with no FAR
   regression** (commit 7076a4b, `arq_common.cc:392`), and the in-code comment
   explicitly notes the 3.0 value *cost* throughput ("caused ~50% timeouts").
   The hardcoded 3.0 survivors in §2 are if anything a latent
   throughput/latency drag at moderate SNR (extra 500 ms ACK-SNR defers,
   §3.2), which relaxation removes.

**Residual risk:** the ONLY way this could regress throughput is if the chosen
threshold admitted enough detection-stage false alarms that the RX wasted CPU
running suffix decodes on noise (CRC would still reject them, so no ARQ
corruption — pure CPU). The FAR sweep bounds this to ~0; the regression test
locks it in permanently. Flagged for HW validation: confirm on real HF/IONOS
that the deepened acquisition does not interact with the gearshift
down-cascade differently (orthogonal to this gate, but the floor moves).

---

## §6 FAR sweep + chosen threshold (MEASURED)

Test `connect_ack_metric_far_sweep` (mfsk_ctrl_codec_tests.cc §9.0).
**5000 noise-only trials per pattern**, pure passband AWGN at sigma = 2× clean
RMS, both ACK and CONNECT tone sets, full production gate (matched>=7/16 AND
metric>=T):

| pattern | thr=3.0 | thr=2.5 | thr=2.0 | thr=1.5 |
|---------|---------|---------|---------|---------|
| ACK     | 0       | 0       | 0       | 0       |
| CONNECT | 0       | 0       | 0       | 0       |

Worst metric observed among noise buffers that passed the COUNT gate = **1.207**.

**Interpretation:** the count gate (7/16, P(false)≈2.4e-5/poll) is so strong
that pure noise essentially never reaches count>=7; on the rare buffer that
does, the energy-ratio metric tops out ≈ 1.2. So EVERY candidate threshold
(3.0…1.5) yields 0 false detections. The margin above the worst noise metric:
- 2.0 → **0.79 absolute margin** (chosen)
- 1.5 → 0.29 margin

**Chosen: CTRL_DETECT_METRIC_MIN = 2.0.** It delivers the full predicted
acquisition gain (§7, +2.5 dB) while keeping a comfortable ~0.8 FAR margin.
1.5 buys negligible extra gain (the metric distribution at the cliff clusters
2.0–3.0, §7 probe) for half the margin — not worth a FAR risk that could cost
throughput, per the user constraint. This matches the task's "lean SAFE, favor
2.0" guidance and is corroborated by the already-shipped normal-mode ACK gate
at 0.5 (commit 7076a4b) proving sub-3.0 is FAR-safe in production.

The diagnosis measured FAR exploding only at 0.5 (359/500); 2.0/1.5 ≈ 0 FAR.
Our 5000-trial/pattern sweep confirms 0 FAR at both, with a wider margin at 2.0.

## §7 Acquisition-gain measurement (MEASURED)

Test `connect_metric_acquisition_gain` (mfsk_ctrl_codec_tests.cc §9.2).
Synthesized CONNECT/ACK base pattern + AWGN + **realistic channel impairment**
(12 Hz CFO within Moose range + up-to-half-symbol off-grid timing jitter),
swept noise UP in 0.5 dB steps, 40 trials/step, contiguous majority-pass
(>=60%) cliff:

| pattern | noise cliff @3.0 | noise cliff @2.0 | gain |
|---------|------------------|------------------|------|
| ACK     | 17.0 dB-noise    | 19.5 dB-noise    | **+2.50 dB** |
| CONNECT | 17.0 dB-noise    | 19.5 dB-noise    | **+2.50 dB** |

**Mechanism (confirmed by a metric-regime probe, telecom log this branch):** at
the cliff the COUNT gate is fully satisfied (matched>=7/16, ~60/60 trials) but
the soft metric dips into [2.0, 3.0). Probe at CONNECT thr=7, impaired:
- sigma=7×RMS: metric mean 3.31 → pass@3.0=49/60, pass@2.0=60/60
- sigma=8×RMS: metric mean 2.76 → pass@3.0=**10/60**, pass@2.0=**58/60**
- sigma=9×RMS: metric mean 2.42 → pass@3.0= 2/60, pass@2.0=50/60

So 3.0 rejects the bulk of detections that 2.0 admits, while the count is
strong — EXACTLY the diagnosis's finding ("at the −10 cliff the count is ~14/16
≫ 7; frames are rejected PURELY on this ratio"). Relaxing 3.0→2.0 recovers
them. The +2.5 dB lands in the diagnosis's predicted +2..+3.6 dB range. (The
clean-AWGN-only arm — no CFO/jitter — also shows ~+1.9 dB, because the
energy-ratio degrades with noise alone; impairment just brings the cliff in
sooner. The metric-binding regime does NOT exist at good SNR — see §8.)

## §8 Throughput-neutrality measurement (MEASURED)

The user's hard gate. Three independent confirmations:

**(a) Code identity (the dominant argument).** `git diff 01535f2` touches
exactly 3 files: `include/physical_layer/mfsk.h` (+24, the named constant +
doc), `source/physical_layer/mfsk_ctrl_codec_tests.cc` (+406, tests only), and
`source/physical_layer/telecom_system.cc` (12 lines = the 4 control-frame RX
gate sites, each `3.0 → cl_mfsk::CTRL_DETECT_METRIC_MIN`, nothing else).
**ZERO** changes to any data-PHY file: no ofdm.cc, no ldpc*.cc, no
mercury_*_16.cc (modcods), no arq_*.cc, no equalizer/demap/framing. The OFDM
data demodulation, LDPC decode, channel estimation, and ARQ data loop are
byte-for-byte identical to baseline. The data path cannot behave differently.

**(b) Clean-channel no-op (test `connect_ack_metric_throughput_neutral`, §9.3).**
Over the good-SNR band (bit-clean .. 4× RMS AWGN, no CFO/jitter — the regime
where data flows), 40 trials × 5 noise levels × 2 patterns:
**0 gate-decision divergences** between the old 3.0 gate and the new 2.0 gate;
min metric over count-passers = **7.05** (≫ 3.0). Every detection admitted by
2.0 is also admitted by 3.0 at good SNR, so ACK/CONNECT acceptance — and thus
the ARQ loop that gates throughput — behaves identically. The relaxation is a
verified no-op where data flows.

**(c) Existing data-PHY regression suite passes unchanged.** All §5/§6 data-PHY
roundtrip + high-SNR-no-regression tests pass identically
(mfsk_data_preamble_passband_roundtrip_clean, _argmax_clean/_cliff/_high_snr_
no_regression, mfsk_connect_passband_roundtrip_clean, v3_test_conn_passband_
roundtrip_clean). Full suite: **34 passed, 0 failed**.

**VERDICT: clean-channel / good-SNR data throughput is UNCHANGED.** The only
behavioral delta is which CONNECT/ACK detections pass the soft gate, and ONLY
in the marginal [2.0,3.0) band that appears below the good-SNR operating point.
There, the relaxation ADMITS more real control frames (the acquisition win),
and false alarms remain bounded to ~0 by the unchanged count gate + the
downstream CRC12/type/BSI/bitmap guards (§3) — so even the false-alarm path
cannot perturb ARQ state at the relaxed threshold.

### Residual risk flagged for HW validation
1. The +2.5 dB is a SIM figure under a 12 Hz-CFO/half-symbol-jitter model. Real
   HF adds multipath + deeper CFO that further depress the metric — the gain
   could be larger OR the cliff regime could shift; confirm on IONOS/HF that
   the deepened CONNECT/ACK acquisition translates to a lower link-establish
   SNR without a FAR uptick (watch for [RX-MFSK-CTRL-*] CRC12-fail spam, which
   would indicate noise reaching the suffix decode — harmless to ARQ state but
   wasted CPU).
2. The data PHY cliffs separately near the new acquisition floor (per MEMORY:
   data preamble reaches ~-14.6 dB). Deepening CONNECT/ACK acquisition does not
   move the data cliff — expect the link to ESTABLISH lower but not necessarily
   carry more bps at the very floor until the data-side floor is addressed too.
   This is orthogonal to throughput at good SNR (unchanged) and is the expected
   next bottleneck, not a regression.
