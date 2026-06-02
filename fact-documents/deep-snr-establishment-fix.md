# Deep-SNR ESTABLISHMENT fix — the shared wall (ULTRA + the −10 MFSK floor)

**Status:** DESIGN pass (2026-06-02). NO code this pass (CLAUDE.md §4). SIM/analysis only;
the bench is running v13 (do NOT touch). Decision-grade: root cause (file:line + HW-log
evidence), approaches (cited), the chosen approach, the increment plan, and the
reconciliation with the prior "#2 doorbell" user choice.

**Reads-on:** [[ultra-tier-design.md]] §9/§10/§11/§12 (the three establishment-timer fixes,
SIM); [[connect-suffix-fec-research.md]] (the `p^13` suffix decode cliff + Tier-1 soft
decode, MEASURED); [[phase-b-mfsk-connect-research.md]] §1/§6.7 (the MFSK-CONNECT state
machine + the v1 messages_control-bypass disaster); [[efficient-deep-modulation-frontier.md]]
(the baud-scaled-rungs = ULTRA framing, option (c)); MEMORY
[[ofdm_skipvar_gate_minus10_path]] (the 2026-06-02 RF convergence verdict),
[[testbed_wgn_snr3k_mapping]] (channel SNR3k = WGN_label + 2.4 dB).

---

## §1. The symptom (HW-confirmed, 2026-06-02)

Two failures that LOOK different but share one wall:

1. **ULTRA_0 (config 200, K=8 baud-scaled MFSK) NEVER establishes on RF**, even at
   WGN:−8 (= **−5.6 dB SNR3k**, ~15 dB ABOVE its ~−21 dB PHY design floor). HAIL detects
   (base 8/8, suffix 4/4), RSP responds, then the link never reaches CONNECTED — endless
   PTT cycling (`OK PENDING PTT ON PTT OFF ×11` in `ultra0_refocus.json`).
2. **The −10 MFSK path (ROBUST_0) has MARGINAL establishment at WGN:−12** (= −10 dB
   SNR3k): CONNECT intermittent (5 PTT → CANCELPENDING).

(SNR axis: channel SNR3k = WGN_label + 2.4 dB, slope 1.0; [[testbed_wgn_snr3k_mapping]].)

---

## §2. ROOT CAUSE — the establishment failure is FRAME-LENGTH-bound, not SNR-bound, and not PHY-decode-bound

### §2.1 The decisive controlled comparison (same binary, same SNR, only frame length differs)

The RF "refocus" runs (agent a979910e, 2026-06-02, binary head `4dd8ffb` — which CARRIES
the [[ultra-tier-design.md]] §12/INCR-4 timer fix: CMD log line
`[ULTRA-INCR4] establishment timers floored: cfg=200 ... connection_timeout=266632ms`):

| config | K | ctrl_suffix_tx | CONNECTED @ WGN:−8 (−5.6 dB) | CONNECTED @ WGN:−12 (−10 dB) | RSP decode |
|---|---|---|---|---|---|
| **ULTRA_3** (203) | 1 (≡ROBUST_0) | **1655 ms** (~2 s frame) | **True** (2/2) | **True** (2/2) | `[RX-MFSK-CTRL-CONNECT-START] sender='TESTA'` → `START_CONNECTION received` → `Connected` |
| ULTRA_2 (202) | 2 | ~6 s | establishes (per MEMORY) | establishes, 0 delivered | — |
| **ULTRA_0** (200) | 8 | **13238 ms** (~13 s frame) | **False** (0/2) | **False** (0/2) | NO `matched=`, NO `RX-MFSK-CTRL`, NO `START_CONNECTION received` |

Source files: `ultra3_refocus.json` / `ultra3_refocus_logs/*wgn-8*r1_rsp.log` (the
working short frame), `ultra0_refocus.json` / `ultra0_refocus_logs/*wgn-8*r1_{cmd,rsp}.log`
(the failing long frame).

**The discriminator is unambiguous:** on the IDENTICAL binary, at the IDENTICAL (and DEEP,
−10 dB) SNR, the **short** ULTRA_3 CONNECT frame decodes and establishes, while the **long**
ULTRA_0 CONNECT frame is never decoded by the RSP (`matched=` absent). A PHY-decode SNR
failure would fail the short frame too (and ULTRA_0's PHY floor is −21 dB — 15 dB below the
−5.6 dB test cell). A pure timer failure was already FIXED (INCR-4 present, window = 266 s).
⇒ **The binder is the long CONNECT FRAME's interaction with the two-radio handshake — the
choreography, not the content code, not the timers (now), not detection.** This matches the
[[ultra-tier-design.md]] §1.5 HW lesson ("the limiter kept moving one stage downstream") and
the §9/§11 finding that establishment is choreography-bound.

### §2.2 The handshake on `monitor` today (the path both failures share), file:line

Phase-B MFSK-CONNECT shipped (@41ac734); START_CONNECTION is an **MFSK ctrl-suffix**, not
LDPC. The establishment is a 3-frame round trip on the ctrl-suffix PHY:

1. CMD → RSP: **START_CONNECTION** (`send_mfsk_start_conn_phy`, blocking TX of the whole
   base+suffix frame). For ULTRA_0 this is ~13.2 s of audio; for ROBUST_0 ~2 s.
2. RSP → CMD: **TEST_CONNECTION_ACK / SWITCH** (`[HANDSHAKE-ECHO]`).
3. CMD → RSP: **TEST_CONNECTION** → RSP `Connected`.

The RX side that fails for the long frame:
- **RSP listen window:** `arq_responder.cc:202-205` `hail_timeout = 2*message_transmission_time_ms
  + 3000` (sized from the DATA frame on `monitor`; ULTRA-scaled to ~266 s by INCR-4 in the
  worktree). The wait loops at `:217` (`receiving_timer < receiving_timeout`); on expiry it
  prints `[HAIL] Timeout waiting for START_CONNECTION, resuming HAIL scan` (`:1008`).
- **RSP capture/decode** of START_CONN: the suffix is decoded by hard argmax
  (`ofdm.cc:3986-4070 decode_suffix_tones`), CRC12-gated (`arq_common.cc:5056-5069`). The
  RSP snapshots the buffer TAIL (`receive_mfsk_ctrl_suffix_phy_core`, ~`:5132`) sized to
  `connect_base_total_nsymb() + ctrl_suffix_len() + 16`.
- **CMD abort timer:** `connection_attempt_timer` vs `connection_timeout`
  (`arq_common.cc:1959-1962`, `Connection attempt timeout after <N> ms`); `connection_timeout`
  floored from the DATA-frame `min_ct` (`:1448-1456`) on `monitor`, ULTRA-scaled by INCR-4.

### §2.3 Why the LONG frame fails the RSP decode even with a wide window — the residual mechanism [?]

INCR-4 made the RSP WAIT long enough (266 s window) and the CMD not ABORT (266 s timer), yet
the RSP still logs no `matched=` for the 13.2 s ULTRA_0 frame. Candidate mechanisms, in
descending probability (to be pinned by Phase-1 diagnostic, §5):

- **[?] (H1) Capture-ring `buffer_Nsymb` too small for the long frame.** The RSP tail-snapshot
  reads `base_total + suffix + 16` symbols from a ring of `buffer_Nsymb`. [[ultra-tier-design.md]]
  §10.3 invariant **I-E** explicitly DEFERRED to HW: "*confirm the ring `buffer_Nsymb` ≥ one
  ULTRA_2 frame's base+suffix+margin on HW*." ULTRA_0 ≈ 544 ctrl symbols (13238 ms / 24.33 ms);
  if the ring (sized for a DATA frame) is shorter, the tail snapshot cannot contain a complete
  base+suffix → argmax never aligns → `matched=` never fires. **This is the single most likely
  cause and the cheapest to confirm/fix.**
- **[?] (H2) PTT / capture-flush / RX-mute over a 13 s key-down.** The post-TX flush +
  `RX_MUTE_GUARD_MS` and the Bug #55 RSP-respond-delay discipline were tuned for sub-second
  control frames; a 13 s blocking TX may overrun a capture buffer or land the RSP's snapshot
  in muted/flushed audio. ([[ultra-tier-design.md]] §4.4 flagged "Bug #55 is MORE acute at
  ULTRA frame lengths.")
- **[?] (H3) Mini-Moose CFO / coarse-sync drift across the long base+suffix.** The detector's
  mini-Moose was validated on short frames; CFO estimate may not hold across ~13 s. Lower
  probability — the base-pattern detection (HAIL) already reaches −22 dB and the short ULTRA_3
  suffix decodes at −10 dB.

The ARCHITECTURAL conclusion does NOT depend on which of H1/H2/H3 binds: **a CONNECT frame
that is materially longer than a normal control frame is fragile in the two-radio handshake,
and the prior art never makes the establishment frame longer (§3).** The fix is to STOP
establishing on the long frame.

### §2.4 The ROBUST_0 −10/−12 marginality is the SAME wall, one notch up

ROBUST_0's CONNECT frame is short (≈ ULTRA_3's 2 s), so it ESTABLISHES at −10 dB (ULTRA_3
2/2 at WGN:−12). Its marginality at WGN:−12 is the suffix `p^13` hard-decision decode cliff
([[connect-suffix-fec-research.md]] §1: hard suffix 0.5-crossing ≈ −7.3 dB SNR3k in that
harness; near-misses below that). I.e. ROBUST_0 establishment is bound by the SAME ctrl-suffix
decode that ULTRA rides — just at the shallow-frame end of it. **One fix that hardens the
ctrl-suffix establishment decode helps BOTH** (ULTRA via "establish on the short frame";
ROBUST_0 via "decode the short frame deeper").

---

## §3. Prior art — how JS8/FT8/FST4/Q65/VARA establish deep (CITED)

**The universal pattern: a FIXED-format, FIXED-symbol-count establishment frame with the
SAME sync across all depths; depth comes ONLY from baud/integration scaling — never from a
longer mode-specific handshake.**

- **FST4 / Q65 / FT8 / FT4 / JT65 / WSPR (WSJT-X 2.7 User Guide, K1JT):** "*they use nearly
  identical message [formats]*" — a fixed 77-bit message + fixed Costas sync. FST4 reaches
  −20.7 dB (15 s) to −43.2 dB (1800 s) **purely by lengthening the T/R period (lowering the
  baud)** of the SAME (240,101) LDPC frame; the message length and sync pattern do NOT change
  with depth. (wsjt.sourceforge.io/wsjtx-doc/wsjtx-main-2.7.0.html, §6.6 FST4, Table 7.)
- **The coherence ceiling (same guide, §6.5, verbatim):** "*successful operation requires …
  channel Doppler spread smaller than [the keying rate]*" — deep FST4 (900/1800 s) is
  documented as an **LF/MF** mode because HF Doppler + Tx/Rx frequency drift break coherence.
  ⇒ a longer deep frame is HARDER to deliver intact on HF, not easier. (Already cited in
  [[ultra-tier-design.md]] §3.2.)
- **Q65 (Nico Palermo IV3NWV qracodes / K1JT):** noncoherent M=64 FSK + GF(64) RA + 12-bit
  CRC; "message-averaging" sums repeated FIXED transmissions to decode several dB below the
  single-shot threshold. Depth = integrating MORE of the SAME fixed frame, not a bigger frame.
  This is literally the codec family Mercury's GF16 ctrl-suffix is ported from.
- **JS8 (FT8 PHY, speed submodes Slow/Normal/Fast/Turbo):** the directed-message + sync
  structure is FIXED; the submodes scale the symbol rate. The "Slow" (deepest) mode uses the
  same acquisition pattern as the faster ones. (js8call.com / OARC wiki.)
- **VARA HF (Rosseló EA5HVK):** establishes its connection in its most-robust modulation and
  then negotiates UP to the data speed the channel supports — establishment SNR is decoupled
  from data-rate SNR. (Architectural pattern; primary release notes 403/404 to the fetcher —
  flagged [?], but the "robust handshake then speed-negotiate" behavior is well-documented in
  the VARA user community and is the textbook ARQ design.)

**Implication for Mercury:** ULTRA_0's K=8 CONNECT frame (13.2 s, 544 symbols) is the
ANTI-pattern. Every cited deep mode keeps the establishment frame SHORT and FIXED. Mercury
should establish on a short frame and reserve "deep" for the DATA phase (or for *integration*
of a short frame, not a *longer* frame).

---

## §4. Approaches evaluated, and the choice

The task lists four candidate approaches. Evaluated against "serves BOTH ULTRA establishment
AND −10 ROBUST_0 reliability, lowest R&D risk," with the §2 root cause (long CONNECT frame is
the binder) decisive:

### (b) **Establish-shallow-then-switch** — CHOSEN (primary). = the user's prior "#2 doorbell."
**Establish the link on a FIXED shallow doorbell config (ROBUST_0 / ULTRA_3-K=1, the short
~2 s CONNECT frame that HW-PROVED establishes at −10 dB, §2.1), then SWITCH to the deep data
PHY (ULTRA_0/1/2) after CONNECTED via the existing turboshift/SWITCH machinery.** This
DECOUPLES establishment SNR from data SNR — exactly the VARA/JS8 pattern (§3).
- **Why it serves ULTRA:** ULTRA_0 establishment becomes ULTRA_3/ROBUST_0 establishment,
  which is HW-confirmed to work at −10 dB (the deepest tested cell). The 13.2 s frame and its
  H1/H2/H3 fragility (§2.3) are removed from the handshake entirely. The deep K=8 DATA PHY
  then runs on an ALREADY-ESTABLISHED link (a config-SWITCH, not a cold establishment).
- **Why it serves the −10 ROBUST_0 path:** ROBUST_0 ALREADY uses this short frame — so "the
  doorbell" IS the ROBUST_0 establishment; hardening it (via (c)) is shared.
- **R&D risk: LOW-MEDIUM.** The config-SWITCH-after-CONNECT path EXISTS (turboshift /
  SWITCH_BANDWIDTH / the gearshift down-cascade). The new work is: (i) make the doorbell a
  FIXED shallow config for ULTRA sessions (don't COLD-establish at 200), and (ii) a post-CONNECT
  switch DOWN to the deep ULTRA data rung. **CRITICAL guardrail:** keep the [[phase-b-mfsk-connect-research.md]]
  §6.7 lesson — the v1 "messages_control bypass" surfaced 4 sibling bugs in 24 h; v2's
  **PHY-swap pattern (state-machine flow UNCHANGED, only PHY parameters swapped)** is the
  proven approach. Establish-shallow-then-switch MUST reuse the existing SWITCH state flow,
  NOT a new control bypass.

### (c) **Give the CONNECT ctrl-suffix the discrete-match / soft-decode reach treatment** — CHOSEN (complementary, FREE, ship first).
[[connect-suffix-fec-research.md]] §6 already IMPLEMENTED + MEASURED Tier-1 soft list-decode
of the ctrl-suffix: **+1.34 dB acquisition on BOTH CONNECT and ACK, ZERO airtime, 0.25% FAR,
byte-identical when off**, 36/36 tests. This hardens the SHORT-frame establishment decode that
BOTH the doorbell (ULTRA) and ROBUST_0 −10 rely on. It does NOT lengthen any frame (so it
cannot reintroduce the §2 binder). Tier-2 (parity symbols) is **CONNECT-only** and the §2.3
fragility warns AGAINST growing the frame — so Tier-2's airtime growth should be applied as
INTEGRATION/repetition of the short frame, not a longer single frame (see (a)).
- **Why both ULTRA and −10:** it lowers the ctrl-suffix decode cliff that gates establishment
  on the short doorbell frame in BOTH lines.
- **R&D risk: VERY LOW.** Already built + measured in `wt/connect-suffix-fec`; the remaining
  work is wiring it into the production ARQ consumers behind the flag (the §6.6 ship rec).

### (a) **Baud-scaled / Costas-array CONNECT base pattern matched to the deep modes** — PARTIAL adopt (as INTEGRATION, not a longer frame).
The base-pattern is ALREADY baud-scaled and combining-capable ([[ultra-tier-design.md]] §1.3,
`detect_ack_pattern combine_reps`; HAIL alive to −22 dB at R=4). The legit sub-lever here is
**noncoherent FRAME-level repetition** (Q65 message-averaging, §3): send the SHORT doorbell
CONNECT frame R_frame× and combine — depth WITHOUT a longer single frame, which sidesteps
§2.3. This is [[ultra-tier-design.md]] §2.5 Lever D (Change B), but applied to the SHORT
doorbell frame, not the K=8 frame. Adopt only IF the doorbell establishment needs to go below
where the short frame + (c) reaches.
- **R&D risk: LOW** (the combining primitive exists; repetition is the §10.2 Change B).

### (d) **Fix the handshake timing for long frames** — REJECTED as the primary path.
This is what INCR-2/INCR-4 already did (scaled the RSP window + the CMD abort timer to the
long frame). HW (§2.1) proves it is INSUFFICIENT: with a 266 s window the long frame STILL
does not decode (`matched=` absent). Timer-scaling is necessary hygiene but does NOT fix the
binder, because the binder is the long frame's decode/capture fragility (§2.3), not the timer.
Pursuing (d) further = "tune the timeout to mask a failure," which CLAUDE.md §2 forbids. The
root-cause fix is to NOT establish on the long frame (b), not to keep widening windows.

**CHOSEN = (b) establish-shallow-then-switch [primary] + (c) soft-decode the short CONNECT
suffix [complementary, free, ship-first], with (a) frame-repetition held in reserve.** This
trio attacks the root cause (remove the long frame from the handshake), hardens the shared
short-frame decode at zero cost, and keeps a depth lever that does not reintroduce a long frame.

---

## §5. Implementation plan (one-change-one-test, SIM before HW; bench owns v13)

Ordered so the FREE shared win ships first and the architectural change is de-risked in sim.

**INCR-A — Phase-1 diagnostic: pin §2.3 (H1/H2/H3) [SIM/instrument, no behavior change].**
Confirm WHY the 13.2 s ULTRA_0 frame is not decoded by the RSP even with the 266 s window.
Cheapest first: instrument the RSP capture path — log `buffer_Nsymb` vs the required
`connect_base_total_nsymb()+ctrl_suffix_len()+16` for ULTRA_0 (test H1). If H1, the fix is to
size the ctrl capture ring to the ULTRA frame (the §10.3 I-E deferred item) — a buffer-sizing
fix, not a protocol change, and it may ALONE let the long frame decode (informing whether (b)
is even strictly required for ULTRA, vs a nice-to-have). **This is the FIRST increment because
it determines how much of (b) is load-bearing.** Test: an in-process assertion that the ctrl
capture ring ≥ one ULTRA_0/1/2 frame; a two-buffer sim harness driving a 544-symbol ctrl
frame through the RSP snapshot+decode and asserting `matched=` fires (fails-before if the ring
is short).

**INCR-B — wire Tier-1 soft ctrl-suffix decode into production (approach (c)) [SIM].**
Promote `wt/connect-suffix-fec` §6 Tier-1 (already built + 36/36) into the production ARQ
CONNECT/ACK consumers behind `suffix_fec_mode=1`, as a FALLBACK after the hard decode misses
(clean-channel behavior byte-identical). Test: the existing cliff-sweep asserts soft cliff ≤
hard cliff (+1.34 dB) and FAR ≤ 0.25%; the existing baseline-identical + NB-unchanged gates.
This hardens the SHORT-frame establishment for BOTH ULTRA-doorbell and ROBUST_0 −10 at ZERO
airtime. **Independently shippable; no dependency on (b).**

**INCR-C — establish-shallow-then-switch for ULTRA (approach (b)) [SIM].** Make an ULTRA
session COLD-ESTABLISH on a FIXED shallow doorbell (ROBUST_0-class short CONNECT frame), then
SWITCH to the deep ULTRA data rung post-CONNECT via the EXISTING SWITCH/turboshift state flow
(PHY-swap pattern, [[phase-b-mfsk-connect-research.md]] §6.7 — NO messages_control bypass).
Requires the §5 cross-layer data-flow audit (the doorbell config ↔ the negotiated data config
↔ the gearshift down-cascade ↔ the sticky-deep hysteresis from [[ultra-tier-design.md]] §4.5).
Test: the in-process CONNECT→SWITCH-to-ULTRA→DATA-unit regression (the CLAUDE.md datalink
pattern) asserting RX state == TX state across the establish-on-doorbell → switch-to-deep
transition; assert the COLD CONNECT frame is the SHORT one (send-count / frame-airtime check),
not the 13.2 s K=8 frame.

**INCR-D — HW re-test [bench-gated, AFTER v13 frees].** Deploy INCR-A+B(+C) to both Pis;
re-run `tools/ultra_establish_hw_test.py` at the deep cells (WGN:−8/−10/−12 = −5.6/−7.9/−10 dB
SNR3k), AWGN + MPx fading. GATE: (i) ULTRA establishes deep (doorbell CONNECT + switch to deep
data), (ii) ROBUST_0 −10/−12 establishment reliability improves with the soft-decode (INCR-B),
(iii) FAR clean on pure noise. Honest report: the ULTRA DATA-phase floor is then measurable
for the FIRST time (it has never run on RF — §9/§11).

**First increment to execute: INCR-A** (the H1/H2/H3 diagnostic). It is pure instrumentation
(no behavior change, no bench), it determines how much of the architectural (b) work is
load-bearing vs whether a buffer-sizing fix alone unblocks the long frame, and it satisfies
CLAUDE.md §1 Phase-1 (pin the root cause with executing-code evidence) before any structural
change. INCR-B can proceed in parallel (independent, free, already-measured).

---

## §6. Reconciliation with the prior "#2 doorbell" choice

The user previously chose **"#2 = ULTRA_0 universal-deep-doorbell"** for the establishment
cliff (MEMORY [[ofdm_skipvar_gate_minus10_path]] names it the "ULTRA #2-doorbell"). The
2026-06-02 RF run (agent a979910e) tested it and found it **NO-GO as built** — but the reason
is exactly §2: the "doorbell" was implemented as ULTRA_0 itself (a LONG K=8 frame), so the
cold establishment used the 13.2 s frame and hit the choreography wall.

**The chosen design IS the #2-doorbell, corrected:** "one deep doorbell config" = establish on
a FIXED SHORT doorbell (the ULTRA_3-K=1 / ROBUST_0 short frame that HW-PROVED establishes at
−10 dB), then SWITCH to the deep ULTRA data PHY. This is "establish-shallow-then-switch" and
"one deep doorbell" reconciled: the DOORBELL (establishment) is SHALLOW/short and universal;
the DEEP-ness moves to the DATA phase. The RF evidence (ULTRA_3 establishes, ULTRA_0 does not,
same binary/SNR — §2.1) is precisely WHY the doorbell must be the short frame, not the K=8 one.
No contradiction with the prior choice; it refines "#2" from "establish AT the deep config" to
"establish at a shallow doorbell, then go deep" — which is also the VARA/JS8 textbook pattern
(§3).

---

## §7. Open questions [?]
- [?] **H1 vs H2 vs H3 (§2.3)** — INCR-A pins it. H1 (ring too small) is the leading
  hypothesis and the cheapest fix; may alone unblock the long frame.
- [?] **Does a buffer-sizing fix alone (H1) make (b) optional for ULTRA?** If the long frame
  decodes once the ring fits it, ULTRA could COLD-establish at 200 after all — but §3 prior art
  + the §2.3 H2/H3 fragility argue the doorbell-then-switch is the robust design regardless.
- [?] **VARA establishment primary source** — release-notes URLs 403/404'd; the "robust
  handshake then speed-negotiate" pattern is community-documented + textbook ARQ, not a
  K1JT-grade primary citation. Confirm against a VARA technical writeup if it becomes
  load-bearing (it is corroborative, not the sole basis — FST4/Q65/JS8 already establish the
  fixed-short-establishment pattern from primary sources).
- [?] **Sticky-deep gearshift hysteresis** at the doorbell→deep switch (the WGN:−10
  unpinned-thrash bug, [[ultra-tier-design.md]] §4.5) — INCR-C design item.

## §9. INCR-A — H1/H2/H3 PINNED (2026-06-02, SIM/instrument, executing-code evidence)

**Verdict: H1 BINDS — but the mechanism is "ring < FULL FRAME", a sharper finding than
the §2.3 framing of "ring < tail-snapshot".** A ring-resize alone does NOT unblock the
long frame (it cannot, see §9.4); the doorbell-switch (INCR-C) is load-bearing.

### §9.1 The decisive arithmetic (production accessors, NOT hand-estimates)
The ULTRA CONNECT frame on the wire is `full_frame_nsymb = connect_base_total_nsymb() +
ctrl_suffix_total_nsymb()` with the BASE pattern FRONT-loaded `[0, base_total)` and the
R_suffix suffix-codeword reps BACK-loaded. The RSP capture ring is
`buffer_Nsymb` OFDM symbols (`data_container.cc:155-171`), sized from the DATA frame
(turnaround+2×frame+margin), NOT from the CONNECT frame. From the production sources:

MEASURED (executing accessors, `test_incr_a_ultra_ring_capacity_h1`, 2026-06-02):

| config | base_total | suffix_one | suffix_total(wire) | **FULL FRAME** | ring buffer_Nsymb | overflow | clean decode |
|---|---|---|---|---|---|---|---|
| ULTRA_0 (200) | 128 | 56 | 448 | **576 sym (14.9 s)** | 564 sym (14.6 s) | +12 (2%) | **MISS (matched=0)** |
| ULTRA_1 (201) | 128 | 48 | 480 | **608 sym (15.7 s)** | 564 sym | +44 (7%) | **MISS (matched=0)** |
| ULTRA_2 (202) | 128 | 45 | 540 | **668 sym (17.3 s)** | 564 sym | +104 (16%) | **MISS (matched=0)** |
| ROBUST_0 (100) doorbell | 16 | 13 | 13 | **29 sym (0.7 s)** | 804 sym (20.8 s) | fits | **DECODED (matched=16)** |

(The ULTRA ring is **564 symbols** — the loaded ROBUST_0-class M=32 data PHY, NOT the
~212 a crude `Nsymb=48` estimate gives; the exact value is load-bearing and is why this was
run through the real accessors. ROBUST_0's own ring is 804 because its NB/data framing
differs. Note ULTRA_0 overflows by only **2%** yet STILL misses — the front base block need
only lose its head + fall outside the 200-sym tail window for `detect_ack_pattern` to fail.)

Sources: `telecom_system.cc:3517-3519` (ULTRA R_base/K/repfact/R_suffix), `mfsk.cc:868`
(`connect_base_total_nsymb()=reps*connect_pattern_nsymb`), `mfsk.h:104-134`
(`connect_base_total_nsymb`/`ctrl_suffix_total_nsymb`), `mfsk_ctrl_codec.cc:442-443`
(`g_N=K+repfact*K`), `mfsk.cc:899-937` (TX layout: base front, R_suffix codewords back),
`mfsk.cc:155-171` data_container `buffer_Nsymb` formula.

### §9.2 Why this is H1 (capture-ring), and why it fails at ANY SNR
`receive_mfsk_ctrl_suffix_phy_core` (arq_common.cc:5288-5298 in the ULTRA worktree)
reads a TAIL of `tail_nsymb = connect_base_total_nsymb()+ctrl_suffix_len()+16` (≈200 sym
for ULTRA_0 — base + ONE codeword + 16), **clamped to `signal_period = Nofdm*interp*
buffer_Nsymb`** (the ring). Because the FULL frame (576 sym) is **larger than the
ring (564 sym)**, by the time the frame finishes filling the ring the FRONT of the base
block `[0,128)` has already SCROLLED OUT — and the tail-snapshot reads only the LAST 200
sym, which are back-loaded *suffix* audio with no complete base block at a findable offset. The decoder's first gate is `detect_ack_pattern(...,
connect_pattern_nsymb base, connect_tones, ...)` (telecom_system.cc:3588) — looking for the
base pattern that is **no longer in the ring**. So `matched` never reaches
`connect_match_threshold` → the decode returns false **independent of SNR**. This is
EXACTLY the §2.1 HW signature (no `matched=`, no `[RX-MFSK-CTRL]`, no `START_CONNECTION
received` — even at WGN:−8 = −5.6 dB, 15 dB above the PHY floor).

### §9.3 The two-buffer in-process proof (`test_incr_a_ultra_ring_capacity_h1`)
Added to `mfsk_ctrl_codec_tests.cc` in the ultra-incr4 worktree (`C:/Users/kamer/
mercury_wt/ultra-incr4`, branch `sim/ultra-choreo-incr4`; INSTRUMENT-ONLY, not committed
to the deep-snr branch). The test loads ULTRA_0/1/2 + ROBUST_0 via the PRODUCTION
`load_configuration` + `ultra_apply_tier_params`, reads the REAL accessors, renders the
full CONNECT frame with the PRODUCTION `generate_ctrl_suffix_pattern_passband`, lays its
TAIL into a `buffer_Nsymb`-sized ring exactly as the production tail-snapshot reads it,
and runs the PRODUCTION `decode_ctrl_suffix_from_passband` at **sigma=0 (clean)**.
- **RESULT (PASS, `mercury --test` = 55 passed / 0 failed)**: `[INCR-A VERDICT] H1 binds on
  3 config(s); clean decode MISSED on 3 overflow config(s); short-frame control DECODED on 1
  fitting config(s).` ULTRA_0/1/2 each render the full frame, lose the front base block to the
  ring, and the production `decode_ctrl_suffix_from_passband` returns MISS (matched=0) AT
  sigma=0 (proving the failure is GEOMETRIC, not SNR). ROBUST_0 (29 sym ≪ 804-sym ring)
  DECODES clean (matched=16). This in-process result reproduces the §2.1 HW signature
  (no `matched=`, even at −5.6 dB) from first principles.

### §9.4 Would a ring-resize ALONE unblock the long frame? — NO (informs INCR-C)
A naïve "make `buffer_Nsymb` ≥ full ULTRA frame" would need ≥668 symbols for ULTRA_2 (vs
the current 564 — ~+18%; +12 for ULTRA_0). But `buffer_Nsymb` is the STEADY-STATE ARQ
capture ring: it is sized to
the DATA frame + turnaround so the OFDM data receiver and the gearshift see the right
window every batch (`data_container.cc:155-171`; consumed by every OFDM `receive()` and
the SACK/ACK snapshot). Tripling it to fit a once-per-session CONNECT frame would (a)
balloon every per-batch snapshot/flush, (b) change the OFDM coarse-sync search window, and
(c) still leave H2/H3 (the 13–17 s key-down PTT/flush + CFO drift across the long frame)
untested. **The §3 prior art is decisive regardless**: every deep mode (FST4/Q65/JS8)
keeps the establishment frame SHORT and FIXED; none grows it. So even though H1 is "just"
a sizing mismatch, the correct fix is NOT to grow the ring — it is to **stop establishing
on the long frame** (INCR-C: doorbell = the short ROBUST_0-class CONNECT, then SWITCH to
deep ULTRA data PHY). H1 confirms the long-frame establishment is structurally fragile;
the ring is one of (at least) three reasons it can't work, and the cheapest to see.

### §9.5 H2/H3 status
- **H2 (PTT/flush over 13–17 s key-down)**: NOT separately reproduced in sim (needs the
  two-radio PTT/flush timing). H1 is SUFFICIENT to explain the total establishment failure
  (the decode can't even see the base pattern), so H2 is currently MASKED by H1 and cannot
  be evaluated until the frame is short enough to fit the ring. Carried to INCR-D (HW).
- **H3 (mini-Moose CFO drift across the long base+suffix)**: likewise masked by H1; the
  short-frame ULTRA_3/ROBUST_0 base detection + suffix decode already work at −10 dB
  (§2.1), so CFO is not the binder at the SHORT frame. Carried to INCR-D.

**INCR-A conclusion for INCR-C**: the doorbell-switch is REQUIRED (a ring-resize alone is
both insufficient — H2/H3 lurk — and architecturally wrong — §3/§9.4). H1 being the
*visible* binder means INCR-C's "establish on the short frame" will at minimum restore the
base-pattern-in-window invariant that ULTRA_0 violates today.

## §10. INCR-B — Tier-1 soft ctrl-suffix decode WIRED into production (2026-06-02, SIM)

Approach (c), the FREE shared win, shipped onto the worktree (`sim/deep-snr-establishment`
off monitor@8fc1211; `C:/Users/kamer/mercury_wt/deep-snr-establishment`). The Tier-1
soft list-decode functions (`decode_ctrl_suffix_from_passband_soft`,
`decode_ack_sack_from_passband_soft`) already existed on monitor (merged via `177dc31`)
but were exercised ONLY by `--test`. INCR-B wires them as a PRODUCTION fallback.

### §10.1 What changed
A `suffix_fec_mode==1` soft fallback added in THREE production consumers, AFTER the hard
decode misses (clean-channel byte-identical):
- **CONNECT (RSP)**: `arq_common.cc:receive_mfsk_ctrl_suffix_phy_core` — after the hard
  `decode_ctrl_suffix_from_passband` miss, retry `decode_ctrl_suffix_from_passband_soft`
  on the SAME tail; synthesize `rx_type=expected_type` + recompute `rx_crc12` so the outer
  CRC re-check passes by construction.
- **ACK (CMD) ×2**: `arq_commander.cc:~105` (bare-ACK arm) and `~2548` (SACK window arm) —
  after the hard `decode_ack_sack_from_passband` miss, retry
  `decode_ack_sack_from_passband_soft`; recompute `rx_crc12` over the soft `[bsi|bitmap]`.
  A file-local `arq_ctrl_crc12_cb` (mirror of the arq_common static, NEVER inlined) was
  added since the arq_common copy is `static` (separate TU).
- **NOT touched**: `detect_ack_snr_from_passband` (arq_common.cc:5669, the turboshift
  SNR-vote path). Per connect-suffix-fec §5 I1 it reads the HARD `out_tones`; the soft
  path is an ADDITIONAL output and must not perturb the SNR vote. Left byte-identical.

### §10.2 Enable path (gating)
- New CLI `--suffix-fec-soft=on|off` (default off) → `telecom_system.suffix_fec_soft_fallback`
  + sets the initial `suffix_fec_mode=1`. `set_suffix_fec(false)` now restores mode to the
  baseline (`suffix_fec_soft_fallback ? 1 : 0`) instead of hardcoding 0, so a config reload
  does not silently disable the fallback. The GF(16) Tier-2 path (`set_suffix_fec(true)` →
  mode 3) is untouched and orthogonal (it has its OWN soft decode inside
  `decode_ctrl_suffix_from_passband`; mode 1 only fires when `suffix_fec_coded==false`).

### §10.3 §5 CROSS-LAYER DATA-FLOW AUDIT (the ctrl-suffix RX decode)
State changed: the RX decode of the ctrl-suffix (NOT the wire — zero airtime). Extends
connect-suffix-fec-research.md §5 (which already audited the soft codec primitives).
1. **Producers** (write the suffix tones on the wire): UNCHANGED. TX
   `generate_ctrl_suffix_pattern` / `generate_ack_sack_pattern` (mfsk.cc) — the wire is
   byte-identical (Tier-1 adds NO symbols). Verified: `ctrl_suffix_len()`/
   `ack_sack_suffix_len()` untouched ⇒ all passband sample counts unchanged.
2. **Consumers** (read the suffix): the 3 sites above + the SNR-vote path. The 3 wired
   sites now try hard-then-soft; the SNR-vote path is untouched (I1 preserved). Both peers:
   CONNECT decode runs on the RSP (START_CONN) AND CMD (TEST_ACK/TEST_CONN via the same
   `receive_mfsk_ctrl_suffix_phy_core`); ACK decode runs on the CMD. The matched-pair holds:
   a soft-enabled RX decodes BOTH a hard (legacy) AND a soft-rescued frame, because the soft
   path is a strict SUPERSET (candidate[0] == hard argmax, proven by the existing
   `suffix_soft_candidate0_equals_hard` test) gated by the SAME CRC12+type accept (I2).
3. **Valid states**: the soft path inherits the hard path's pre-write states — `out_tones=-1`
   past-buffer ⇒ erasure/abort (never a spurious accept; connect-suffix-fec §5.3). NB
   (`ack_sack_suffix_len()==0`) ⇒ both hard and soft early-return false (the soft fns guard
   `ack_sack_suffix_len()<=0`). Default `suffix_fec_mode=0` ⇒ the fallback block is INERT.
4. **Invariants**: I1 (SNR vote reads hard tones) — preserved (not touched). I2 (accept iff
   CRC12+type) — preserved (the soft decode is CRC12+type-gated; FAR rises by ≤ trials·2^−12,
   bounded by `max_flips=1` ⇒ measured 0.25%). I3 (separate ACK/CONNECT capture buffers) —
   preserved (soft fns use the same buffers). I4 (buffer sizing) — unchanged (no wire growth).
5. **What the fix changes, per consumer**: only the 3 wired RX consumers gain a CRC-gated
   retry on miss; every other reader of the suffix is byte-identical. The ONE new risk —
   FAR from extra CRC trials — is bounded + measured (§6.2: 0.25% @flips=1) and backstopped
   by the type field + ARQ retransmit (same as today).

### §10.4 Tests (the gate)
- `mercury --test` green (the existing 36 suffix-FEC tests + all others) — the soft codec
  was already covered; INCR-B adds the production wiring, which the byte-identical-when-off
  tests (`suffix_soft_candidate0_equals_hard`, `connect_suffix_byte_identical_when_off`)
  guard. [TO RUN on the deep-snr build.]
- Byte-identical OFF: with `--suffix-fec-soft=off` (default) the 3 consumer blocks are inert
  (`suffix_fec_mode==0`) ⇒ decode behavior is bit-for-bit the legacy hard path. [TO VERIFY.]
- Cliff gain (+1.34 dB, FAR ≤ 0.25%): the EXISTING measured prototype numbers
  (connect-suffix-fec §6.3) apply unchanged — INCR-B does not alter the soft codec, only
  its call site. The cliff-sweep test that produced §6.3 lives in `mfsk_ctrl_codec_tests.cc`
  and is re-run by `--test`.

## §8. Status log
- 2026-06-02: doc created (DESIGN pass). Root cause pinned to FRAME-LENGTH-bound choreography
  via the §2.1 controlled RF comparison (ULTRA_3 short establishes @−10 dB, ULTRA_0 long fails
  @−5.6 dB, same binary `4dd8ffb` WITH INCR-4 timers). Chosen: (b) establish-shallow-then-switch
  [primary, = corrected #2-doorbell] + (c) Tier-1 soft ctrl-suffix decode [free, ship-first].
  (d) timer-widening REJECTED as primary (HW-proven insufficient; CLAUDE.md §2). Prior art
  (FST4/Q65/JS8/VARA: fixed-short establishment, depth by baud/integration) confirms the long
  CONNECT frame is the anti-pattern. NO code this pass.
- 2026-06-02: INCR-A executed (SIM/instrument). **H1 PINNED via executing-code arithmetic +
  a two-buffer production-decode diagnostic**: the ULTRA CONNECT frame (576/608/668 sym) is
  2.7–3.2× the capture ring (~212 sym), so the FRONT base pattern scrolls out before the
  frame ends → `detect_ack_pattern` finds no base → decode misses at ANY SNR (matches the
  §2.1 HW signature). Sharper than §2.3's "ring < tail": it's "ring < FULL FRAME". A
  ring-resize ALONE is insufficient (H2/H3 masked) AND architecturally wrong (§3/§9.4) →
  INCR-C doorbell-switch is load-bearing. INCR-B executed: Tier-1 soft fallback wired into
  the 3 production ctrl-suffix consumers behind `--suffix-fec-soft` (default off,
  byte-identical), §5 audit in §10.3.
