# ROBUST_3 coherent weak-signal tier — PHASE 1 implementation plan + §5 audit

**Status:** PLAN → IMPLEMENT. Phase 1 of the multi-phase build scoped in
[phase4-coherent-tier-design.md](phase4-coherent-tier-design.md). SOFTWARE/SIM ONLY.
**Worktree:** `C:/Users/kamer/mercury_wt/robust3`, branch `wt/robust3`, off monitor `01535f2`.
**Prototype that validated the decode:** `C:/Users/kamer/mercury_wt/coherent-proto`
(`PROTO_P1_PLAN.md`) — Nc=5/QPSK/rate-1/4/6 ms-CP/16-sym preamble decoded to
**-7.5 dB SNR(3k) AWGN**; acquisition (16-sym Schmidl-Cox) cliffed ~4 dB higher (~-3.5 dB),
which Phase 2 inherits.

This document is the §4 plan-before-coding deliverable and owns the §5 cross-layer
data-flow audit for the frame-geometry change.

---

## §1. Phase-1 scope (what ships, what is deferred)

**Ships (Phase 1 = "a runnable, production-grade ROBUST_3 config"):**
1. Frame-geometry fix: per-config `Nc` + `Nsymb` plumbed so `nBits == ldpc.N` for
   ARBITRARY Nc (the N_MAX=1600 wall, prototype blocker #1). Production-grade,
   replacing the prototype's env-var hack.
2. ROBUST_3 config (ID 103): Nc=5, QPSK, rate-1/4 LDPC (existing 4/16 N=1600/K=400
   matrix), 6 ms CP, dense 1-in-3 pilots, 16-symbol preamble, LS estimator + MMSE-ZF.
   Placed between ROBUST_2 (102) and CONFIG_0 in the ladder.
3. `CAP_COHERENT_TIER` capability bit + negotiation gating (LDPC TEST_CONNECTION
   byte 5 path AND MFSK CONNECT handshake 2→3-bit cap field) + gearshift old-peer guard.
4. Build + `--test` (full suite) + BER sanity (~-7.5 dB AWGN) + a frame-geometry
   round-trip regression test.

**Deferred (later phases — leave clean seams):**
- Phase 2: acquisition — close the ~-3.5 dB detect vs ~-7.5 dB decode gap (pre+post-amble
  time diversity, the dominant risk; design §3.3/§7.2). 16-sym preamble is the Phase-2 seam.
- Phase 3: 2D-channel-state-lookup entry + Q-table re-cal (design §5.2). The rung is in
  the ladder array; calibration is a separate workstream.
- Nc=4 (DATAC4-exact, deeper floor): a one-line per-config Nc change once acquisition
  is solved — see §3 justification for why Phase 1 uses the validated Nc=5.
- IONOS / hardware validation (this build is SIM-ONLY).

---

## §2. The frame-geometry wall — root-cause analysis (Phase-1 fix #1)

### §2.1. The shortening scheme (NOT "nBits must == N")

`cl_telecom_system::transmit_bit` (telecom_system.cc:517-542) implements LDPC
**shortening-by-repetition**, which DOES support `nBits != ldpc.N`:

```
nVirtual_data = ldpc.N - nBits      // repeated (shortened) info bits, NOT transmitted
nReal_data    = nBits - ldpc.P      // real payload bits
// encoder input = [nReal_data real][nVirtual_data repeated] = (nBits-P)+(N-nBits) = N-P = K bits
ldpc.encode(...)                    // -> N-length codeword
// repack (:537-540): transmit [nReal_data info][P parity] = nBits bits; virtual bits dropped
```

RX inverse (telecom_system.cc:317-328): reconstructs the dropped virtual-bit LLRs by the
same repetition rule, then decodes the full N-length vector. **Round-trips for any
`ldpc.P <= nBits <= min(ldpc.N, N_MAX)`.**

So the wall is NOT "nBits must equal N". It is two real constraints + one quality goal:

- **Hard A (overflow):** `nBits <= N_MAX (=1600)`. `data_bit`, `encoded_data`,
  `bit_interleaved_data`, `demodulated_data`, `deinterleaved_data`, `hd_decoded_*`,
  `bit_energy_dispersal_sequence` are all `CNEW(…, N_MAX, …)` (data_container.cc:106-132).
  `nBits > 1600` heap-overflows → segfault (prototype's "Nc=8 segfault").
- **Hard B (encoder input):** `nBits <= ldpc.N`. If `nBits > N`, `nVirtual_data < 0`
  (the repeat loop is a no-op) AND `nReal_data = nBits-P > K`, so the info-copy
  `data_bit[0..nReal_data)` reads past the K-length encoder input → corrupt codeword.
- **Quality goal (floor reproduction):** for the BER floor to MATCH the prototype's
  validated -7.5 dB, the production config should use the **full N=1600 codeword**
  (nBits == N, nVirtual_data == 0). A shortened code (nBits < N) is a *different* code
  with a *different* (worse, shorter-block) waterfall — it would NOT reproduce the
  validated floor, so the Phase-1 BER sanity would have no apples-to-apples reference.
  → Phase 1 targets `nBits == ldpc.N` exactly (full codeword).

### §2.2. Why AUTO sizing only lands nBits==N when Nc divides 50

`init()` (telecom_system.cc:3924-3952) computes `nc_scale = 50/ofdm.Nc` (INTEGER div)
and `Nsymb = 24*nc_scale` for QPSK/HIGH_DENSITY. `nData = Nc*Nsymb - nPilots - nConfig`
(pilot grid, ofdm.cc:1132-1220). `nBits = nData*log2(M)` (data_container.cc:98).
For QPSK, `nBits == 1600` requires `nData == 800`. Empirically (probe replicating
the exact pilot_configurator math, /tmp/nc_probe.cc), with Dy=3:

| Nc | AUTO Nsymb (24·⌊50/Nc⌋) | AUTO nData | **explicit Nsymb for nData=800** | BW |
|----|----|----|----|----|
| 4  | 288 | 768 ✗ | **300** | 187.5 Hz |
| 5  | 240 | **800 ✓** | 240 | 234.4 Hz |
| 6  | 192 | 768 ✗ | **200** | 281.2 Hz |
| 8  | 144 | 768 ✗ | **150** | 375.0 Hz |
| 10 | 120 | **800 ✓** | 120 | 468.8 Hz |

`⌊50/Nc⌋` truncates for Nc∉{1,2,5,10,25,50} → AUTO under-sizes (nData=768 → nBits=1536 →
shortened code). **Fix = per-config explicit `Nsymb` that yields nData=800 for the
chosen Nc**, sized once at config-load (not the AUTO formula).

### §2.3. The fix (production-grade)

Add two per-config fields resolved in `load_configuration`:
- `ofdm.Nc` override (already a member; today only AUTO→50/10 at :3850). Set a real
  per-config value for ROBUST_3; leave AUTO for all existing configs (byte-identical).
- `ofdm.Nsymb` explicit override (already a member; today AUTO at :3926). Set the
  value from §2.2 for ROBUST_3.
- `ofdm.gi` per-config (already configurable via `--gi`; set 72/256 = 6 ms for ROBUST_3).

These slot into the SAME machinery the AUTO path feeds — `init()` sees a non-AUTO Nc
(skips the :3850 branch) and a non-AUTO Nsymb (skips the :3926 branch), then
`calculate_parameters` → `data_container.set_size` compute nBits/nData/bandwidth/FIRs
from the resolved values. No new algorithms; parameter values + the existing reinit path.

**Production-grade guard (NOT in prototype):** a runtime invariant check after init that
`data_container.nBits == ldpc.N` for the coherent tier (and a clear fatal log if not),
so a future Nc/Dy/Nsymb edit that breaks the geometry fails loudly at load, not silently
into a shortened code. This is the production replacement for the prototype's
`[PROTO-FRAME] … FRAME-SHORT` printf.

---

## §3. ROBUST_3 config parameters + the Nc choice

| Param | Value | Basis |
|---|---|---|
| Config ID | **103** | ROBUST namespace (100-102 used); design §5.1 "COHERENT_0 = ID ~103" |
| Modulation | **QPSK** | design §7.3; all DATAC = QPSK; prototype validated QPSK |
| LDPC | **rate-1/4** (4/16, N=1600, K=400) | design §7.3; matrix EXISTS (ldpc.cc:175 K==400); DATAC4=0.30 |
| Carriers Nc | **5** | see justification below |
| Nsymb | **240** (explicit) | §2.2: Nc=5 → nData=800 → nBits=1600=N (full codeword) |
| BW | **234.4 Hz** | 48000·5/256/4; ≈ DATAC4 250 Hz |
| CP / GI | **6 ms** (72/256 samp) | design §7.3; DATAC4 tcp=6 ms |
| Pilots | Dx=1, **Dy=3** (1-in-3, HIGH_DENSITY) | design §7.3 "1-in-3..1-in-5"; Mercury default Dy=3 is DENSER than DATAC4's 1-in-5 (good for weak signal) |
| Preamble | **16 OFDM symbols** | design §7.3; §2.2/§3.3 16-sym ≈ +4 dB acq reach. Phase-2 seam. |
| Channel est | **LEAST_SQUARE** + DFT smooth | design §2.1/§3.3; CONFIG_0 uses LS |
| Equalizer | **MMSE-ZF** (existing) | shared with all OFDM configs |
| Frame duration | ~6.56 s data (+0.44 s preamble) | Nsymb·(Tu+CP) = 240·27.33 ms |

### §3.1. Nc choice: **Nc=5, NOT Nc=4** — justification

The directive: "target the deepest the geometry cleanly allows — Nc=4 like DATAC4 if
feasible, else the validated Nc=5; justify." With the §2 explicit-Nsymb plumbing, **Nc=4
IS geometrically feasible** (Nsymb=300 → nData=800). So feasibility is not the
discriminator. The discriminator is **validation + the Phase-1 success criterion**:

1. **The prototype validated Nc=5 at -7.5 dB AWGN. It never validated Nc=4** — Nc=4
   segfaulted on the geometry wall (PROTO_P1_PLAN results: "Nc=4/8 do NOT divide 50 ⇒
   FRAME-SHORT/segfault"). Nc=4's floor is *extrapolated* (~2 dB/halving trend ⇒ ~-8.5 dB),
   not measured.
2. **Phase-1's BER success criterion is defined as "~-7.5 dB AWGN, matching the
   prototype."** That number IS the Nc=5 point. Choosing Nc=4 would leave the Phase-1
   BER sanity with no validated reference to match — violating CLAUDE.md §3 (no untested
   fixes; validate in simulation against a known-good reference).
3. **Cost of Nc=4 now:** +1.6 s/frame latency (8.64 s vs 7.00 s with preamble), a
   narrower 187.5 Hz occupied BW that stresses the GI-safe FIR design and Schmidl-Cox
   detection *further* (design §4.1 NB ZF-fallback lesson at Nc<10), for ~1 dB of
   *unvalidated* extra floor — while acquisition (cliffing ~4 dB ABOVE decode) is the
   actual Phase-2 bottleneck, so the extra decode dB is not even reachable yet.
4. **Nc=5 ≈ DATAC4 in the dimension that matters:** 234.4 Hz vs 250 Hz occupied BW,
   +10 dB/carrier vs Nc=50. The "4 carriers" of DATAC4 is a means to the narrow-BW /
   high-per-carrier-energy end, which Nc=5 achieves.

**Decision: Nc=5 for Phase 1.** Per-config Nc leaves Nc=4 a one-line change (`ofdm.Nc=4;
ofdm.Nsymb=300;`) for a future phase once acquisition is solved and an Nc=4 floor can be
*measured* on the testbed. The seam is clean.

### §3.2. Ladder placement

`FULL_CONFIG_LADDER` (common_defines.h:101-107) currently:
`{ROBUST_0,1,2, CONFIG_0..16}` (size 20). Insert ROBUST_3 between ROBUST_2 and CONFIG_0:
`{ROBUST_0,1,2, ROBUST_3, CONFIG_0..16}` (size 21). This makes `config_ladder_up`
from ROBUST_2 land on ROBUST_3, then CONFIG_0 — matching design §5.1's
"deeper than CONFIG_0, ~equal/better throughput than ROBUST, hands off to CONFIG_0/1".
`is_robust_config()` extended to `<=103` so ROBUST_3 inherits the MFSK-CONNECT
handshake classification, the 200-iteration LDPC budget (telecom_system.cc:5030), and
the BREAK/anchor "below-the-WB-ladder" treatment.

---

## §4. CAP_COHERENT_TIER negotiation + gearshift old-peer guard

### §4.1. Wire-format surfaces (BOTH must carry the new bit)

`peer_capability` is set from TWO wire paths (audit in §5.B):
1. **LDPC TEST_CONNECTION / TEST_CONNECTION_ACK byte 5** — full `u8`
   (datalink_defines.h:122-127; commander.cc:4025, responder.cc:2051). Room for 0x04.
2. **MFSK CONNECT handshake** (Phase B, the DEFAULT establishment path) — cap packed
   in a **2-bit field** `& 0x3` (mfsk_ctrl_codec.cc:150-151,184,196; .h:100-101,123-124).
   This path CANNOT carry 0x04 without widening the field. **This is the binding
   constraint** the design doc under-specified (§5.3 said "rides TEST_CONNECTION" but the
   MFSK handshake is what actually establishes the link post-Phase-B).

**Fix:** widen the MFSK cap fields from 2 bits to **3 bits**, consuming one reserved bit
each (TEST_ACK has 26 reserved bits, TEST_CONN has 24 — ample). New bit `CAP_COHERENT_TIER
= 0x04`. Mask becomes `& 0x7`. Backward-compat: an OLD peer sends the bit as 0 (it was a
reserved-must-be-0 bit), so it reads as "no coherent tier" — exactly the desired
old-peer behavior. A NEW peer talking to OLD: old peer ignores the (now-meaningful)
bit position as reserved on its RX, and never sets it on TX → new peer sees peer-cap
without 0x04 → won't propose ROBUST_3. **Additive, safe both directions.**

Field re-layout (preserve snr_q/ssid/echoed positions; steal 1 LSB-adjacent reserved bit):
- TEST_ACK (38b): `echoed_cap` 3b @ [37:35], `own_cap` 3b @ [34:32], `ssid` 8b @ [31:24],
  reserved 24b @ [23:0]. (was echoed@[37:36], own@[35:34], ssid@[33:26].)
- TEST_CONN (38b): `snr_q` 4b @ [37:34], `local_cap` 3b @ [33:31], `ssid` 8b @ [30:23],
  reserved 23b @ [22:0]. (was local_cap@[33:32], ssid@[31:24].)

The pack/unpack regression tests (`test_pack_unpack_test_ack_payload`,
`test_pack_unpack_test_conn_payload`) must be extended to round-trip a cap value with the
0x04 bit set — this is the failing-before/passing-after test for the wire change.

### §4.2. Gearshift old-peer guard

`config_ladder_up/up_n` (common_defines.h:116-144) are PURE functions walking
`FULL_CONFIG_LADDER` — they have no peer-cap access, and adding ROBUST_3 to the array
means an unguarded `config_ladder_up(ROBUST_2)` would route to ROBUST_3 against ANY peer.

**Fix (constrain the producer, not the pure helper):** add a parameter
`bool coherent_tier_ok` to `config_ladder_up`/`config_ladder_up_n` (defaulted false), and
**skip the ROBUST_3 rung when `!coherent_tier_ok`** (treat it like an out-of-ceiling
config: return the current config / clamp). Callers in the gearshift
(arq_commander.cc — the climb/SUPERSHIFT/FRAME-UP sites, §5.C) pass
`(peer_capability & CAP_COHERENT_TIER) && (local_capability & CAP_COHERENT_TIER)`.
Down-cascade (`config_ladder_down`) is NOT gated — if somehow seated at ROBUST_3 we must
always be able to descend; and BREAK/anchor recovery flooring to ROBUST_0 still works
(ROBUST_3 is just another rung below CONFIG_0).

Rationale for guarding in the helper (vs each call site): there are ~15
`config_ladder_up*` call sites in the gearshift; a single defaulted parameter that
defaults to the SAFE (no-coherent) behavior means any site not explicitly opted-in is
automatically old-peer-safe. This is the "constrain the producer" choice from CLAUDE.md
§5 — don't depend on every consumer remembering to check the cap.

---

## §5. Cross-layer data-flow audit (CLAUDE.md §5 — MANDATORY before shared-state change)

The shared state changed: **frame geometry** (`nBits`, `nData`, `Nsymb`, `Nc`),
the **gearshift rung array** (`FULL_CONFIG_LADDER`), and the **capability bytes**
(`peer_capability`/`local_capability` + the MFSK cap fields). Three sub-audits.

### §5.A. Frame geometry (`nBits`, `nData`, `Nsymb`, `Nc`)

**1. Producers (who writes these):**
- `init()` telecom_system.cc:3850-3852 — `ofdm.Nc` from AUTO (50/10). **My change:**
  per-config override set in load_configuration BEFORE init (so AUTO branch is skipped).
- `init()` :3924-3952 — `ofdm.Nsymb` from AUTO (`24·⌊50/Nc⌋` etc.). **My change:**
  per-config explicit Nsymb set BEFORE init.
- `cl_pilot_configurator::configure()` ofdm.cc:1132-1220 — computes `nPilots`, `nConfig`,
  `nData = Nc·Nsymb - nPilots - nConfig`. Unchanged (reads Nc/Nsymb/Dx/Dy).
- `data_container.set_size()` data_container.cc:95-98 — `nData = arg`, `nBits =
  nData·log2(M)`. Unchanged (reads pilot_configurator.nData + M).

**2. Consumers (who reads these):**
- `transmit_bit` :519-542 — `nVirtual=N-nBits`, `nReal=nBits-P`, interleaver(nBits/nData).
  Assumes `P <= nBits <= N`. ✓ Holds for ROBUST_3 (nBits=1600=N, nVirtual=0).
- `receive_byte`/decode :310-328 — deinterleave(nData/nBits), demod(nBits), virtual-bit
  reconstruct (assumes nBits<=N), decode(N-length). ✓ Holds (nBits=N).
- ARQ payload sizing: `get_max_data_length()` :467, `get_active_nbits()` :3067 —
  `(nBits - P - outer_code_reserved_bits)/8`. ROBUST_3: (1600-1200-16)/8 = 48 B/frame.
  ✓ A valid positive frame size; ARQ batch sizing (data-flow-batch-size.md) consumes a
  positive per-frame byte count — no assumption broken.
- All `N_MAX`-sized arrays (data_container.cc:106-132). ✓ nBits=1600=N_MAX exactly — the
  arrays are sized for precisely this; no overflow. (This is WHY N_MAX=1600 and the full
  codeword fits — the existing CONFIG_0 BPSK Nsymb=48 also lands nBits=1600.)
- `ofdm.framer/deframer`, `psk.mod/demod` — operate on nData complex symbols / nBits.
  ✓ Sized from data_container; consistent.
- GUI constellation plot :332-335 — loops `pilot_configurator.nData`. ✓ Reads the
  resolved nData; no fixed assumption.

**3. Valid states + default-init:** before load_configuration, `ofdm.Nc=AUTO_SELLECT(-1)`,
`ofdm.Nsymb=AUTO_SELLECT`, `data_container.nBits=0`/`nData=0` (data_container.cc:31,
ctor). The per-config override runs AFTER the default-reset block
(telecom_system.cc:5062-5064 restores Nc/gi/Nsymb from defaults) — I must inject the
ROBUST_3 override AFTER that reset (same placement the prototype used), else the reset
clobbers it. **Critical ordering fact.**

**4. Invariants consumers assume + verification:**
- INV-1: `P <= nBits <= min(N, N_MAX)`. ROBUST_3: P=1200, nBits=1600, N=1600,
  N_MAX=1600 → `1200 <= 1600 <= 1600` ✓.
- INV-2: `nData == pilot_configurator.nData` (set_size arg). ✓ set_size is called with
  `ofdm.pilot_configurator.nData` (telecom_system.cc:4055).
- INV-3: `nBits == nData·log2(M)`. ✓ by construction in set_size.
- INV-4 (NEW, my guard): coherent tier requires `nBits == ldpc.N` (full codeword, floor
  reproduction). Enforced by the per-config explicit Nsymb + a runtime check.
- Uncommon paths: NB (narrowband_enabled) — ROBUST_3 is **WB-only** (gated
  `!narrowband_enabled`, like the prototype); NB clamps to NB_CONFIG_MAX and never
  reaches ROBUST_3. Config-switch/reinit — the `_ldpc_rate != ldpc.rate` and
  `_modulation != M` reinit triggers (telecom_system.cc:4851-4891) fire correctly when
  switching INTO ROBUST_3 (rate 4/16, QPSK) from ROBUST_2 (MFSK) or CONFIG_0 (BPSK):
  modulation changes → full data_container/ofdm/psk reinit → set_size recomputes nBits.
  ✓ The reinit matrix already handles per-config M/rate/Nsymb changes.

**5. What my fix changes:** introduces a config where Nc≠{AUTO,50,10} and Nsymb is
explicit. The only consumer assumption this touches is "Nsymb came from the AUTO formula"
— but NO consumer reads the AUTO formula; they all read the resolved `ofdm.Nsymb` /
`data_container.Nsymb`. The geometry is internally consistent because set_size derives
everything from the resolved nData. ✓ No consumer broken.

### §5.B. Capability bytes (`peer_capability`, `local_capability`, MFSK cap fields)

**1. Producers:** `local_capability` built in main.cc / arq init (CAP_WB_CAPABLE | …);
**my change:** OR-in CAP_COHERENT_TIER (WB-capable builds). `peer_capability` set at:
commander.cc:4008 (MFSK echo `rsp_own`), :4025 (LDPC byte 5); responder.cc:2051 (LDPC
byte 5). MFSK pack: mfsk_ctrl_codec.cc:145-199.
**2. Consumers:** encryption gate (commander.cc:4063, responder.cc:2096 — `& CAP_ENCRYPTION`);
WB-upgrade (`& CAP_WB_CAPABLE`); GUI mirror (main.cc:2435); **NEW:** gearshift ROBUST_3
guard (`& CAP_COHERENT_TIER`).
**3. Valid states:** `peer_capability=0` at session start (arq_common.cc:304,2696,2793,3100)
→ no coherent tier until negotiated. ✓ Safe default (gearshift won't propose ROBUST_3).
**4. Invariants:** consumers mask the specific bit they care about (`& CAP_X`); adding a
new bit doesn't disturb existing masks. ✓ The 2→3-bit MFSK widening preserves
snr_q/ssid/echoed bit positions (§4.1 re-layout) → existing TEST_ACK/TEST_CONN consumers
(SNR reconstruction commander.cc, SSID) read the same values. **Must verify with the
extended pack/unpack round-trip tests.**
**5. What my fix changes:** old peer ⇒ bit reads 0 ⇒ no proposal (safe). New↔new ⇒ both
set it ⇒ ROBUST_3 selectable. The widening is the only structural change; reserved bits
absorb it.

### §5.C. Gearshift rung array (`FULL_CONFIG_LADDER`)

**1. Producers:** static const array (common_defines.h:101). **My change:** insert
ROBUST_3 at index 3, bump size 20→21.
**2. Consumers:** `config_ladder_index/up/up_n/down/down_n/is_at_top/is_at_bottom`
(common_defines.h:109-178); all gearshift sites in arq_commander.cc (§ search: ~25
`config_ladder_*` calls). `session_floor_anchor` returns `FULL_CONFIG_LADDER[0]` (still
ROBUST_0 ✓). `SUSTAINED_ANCHOR_N_ROBUST` gating via `is_robust_config` (arq.h:678) —
extend `is_robust_config` to include 103 so ROBUST_3 gets robust-tier anchor treatment.
**3. Valid states:** index of every existing config SHIFTS by +1 for CONFIG_0..16 (they
move from indices 3..19 to 4..20). Any code that hardcodes a ladder INDEX (not a config
ID) would break. **Audit result:** all consumers use config IDs via `config_ladder_index`
(ID→index lookup) — none hardcode indices. ✓ The shift is transparent.
**4. Invariants:** `config_ladder_up` monotonic in ID-order within OFDM range (the
`is_ofdm_config(next) && next>ceiling` guard at :126 still works — ROBUST_3 is
is_robust, not is_ofdm, so it's never ceiling-clamped, but IS now guarded by the new
coherent_tier_ok parameter §4.2). `config_is_at_bottom` = index 0 = ROBUST_0 ✓.
**5. What my fix changes:** adds a rung between 102 and 0 + a per-call opt-in to reach it.
Walked correctly; old-peer-safe via the §4.2 guard; index-shift transparent.

### §5.D. Cross-layer regression test (CLAUDE.md mandate)

Two tests, both in-process, no IONOS/RF:
1. **Geometry round-trip** (`test_robust3_frame_geometry_roundtrip`, in
   mfsk_ctrl_codec_tests.cc suite): load ROBUST_3, assert `nBits==ldpc.N==1600`,
   `nData==800`, `Nc==5`, `M==MOD_QPSK`, `ldpc.rate==0.25`, `preamble_nSymb==16`,
   `gi≈72/256`. Then encode→interleave→(noiseless mod/demod or direct)
   →deinterleave→decode a random K-bit payload and assert exact bit recovery
   (the shortening/repack/virtual-bit path round-trips at the new Nc). Fails before
   the config exists / if geometry mis-packs; passes after.
2. **Cap wire round-trip** (extend existing `test_pack_unpack_test_ack_payload` +
   `test_pack_unpack_test_conn_payload`): round-trip `own_cap = CAP_WB_CAPABLE |
   CAP_COHERENT_TIER` (0x05) through the widened 3-bit fields; assert it survives and
   that snr_q/ssid/echoed are unperturbed. Fails before widening (0x04 bit truncated by
   `&0x3`); passes after.

The full ROBUST_0→ROBUST_3→CONFIG_0 in-session transition test (driving the ARQ state
machine) is deferred to Phase 3 (gearshift integration) — Phase 1 ships the rung +
geometry + negotiation, not the live cascade tuning. The geometry + cap round-trips are
the Phase-1-scoped cross-layer assertions.

---

## §6. BER sanity (Phase-1 success gate)

Reproduce the prototype's validated point with the PRODUCTION config (no env-var hacks):
- `mercury.exe -m PLOT_PASSBAND -s 103` (or the single-point `--ber-esn0` path) through
  the AWGN injector, in-band-SNR calibration, ~100+ frames/point.
- **Success = coded-PER (FER) cliff at SNR(3k) ≤ -4 dB AWGN; expected ~-7.5 dB matching
  the prototype** (prototype: FER 0.00@-7.07 dB, 0.46@-8.07, 1.0@-9.07).
- If it diverges (e.g. cliffs at ~0 dB), the production geometry differs from the
  prototype's — diagnose (most likely a Nsymb/pilot/gi mismatch) before claiming Phase-1
  done. The geometry round-trip test (§5.D) catches gross mis-packs; the BER catches
  subtler floor divergence.

Note: the prototype reused CONFIG_0's branch + env vars; the production version must
reproduce the SAME effective waveform (Nc=5, QPSK, 4/16, 6 ms CP, Dy=3, 16-sym preamble)
through the ROBUST_3 config branch. The in-band-SNR harness instrumentation
(`proto_inband_snr` etc.) is throwaway and lives only in coherent-proto; for Phase-1 BER
I will use the production single-point BER path and the standard Es/N0→SNR(3k)
conversion, OR port the minimal in-band-SNR calibration as a clean `--ber-inband` flag if
needed to make the axis directly comparable. (Decide at BER-run time; prefer the existing
path if the cliff is already legible without it.)

---

## §7. What Phase 2 inherits (the acquisition gap)

- **Decode floor: PROVEN ~-7.5 dB AWGN** (prototype + Phase-1 BER sanity). The coherent
  demod (LS + MMSE-ZF + rate-1/4 SPA) is NOT the limiter.
- **Acquisition cliff: ~-3.5 dB** (16-sym Schmidl-Cox time-sync, prototype). This is ~4 dB
  ABOVE the decode floor — Phase 2's job is to close it. The 16-sym preamble is the seam;
  Phase 2 adds pre+post-amble time-diversity (P(miss)² detection, codec2 technique,
  design §3.3) and/or the discrete-bin matched detector already shipped for MFSK
  (data-preamble-port-research.md). Until then, ROBUST_3's *usable* floor is acquisition-
  bound at ~-3.5 dB, not the -7.5 dB decode floor.
- **Phase 3 inherits:** the rung is in the ladder + cap-gated, but NOT yet in the
  2D-channel-state-lookup or the Q-table. Re-cal needed before unpinned selection.
- **No hardware validation yet** — SIM-ONLY. IONOS Watterson/fading behavior at the floor
  is unmeasured (the harness has only static 2-ray freq-selectivity).

---

## §8. Implementation order (this session)

1. ✅ Plan + audit (this doc).
2. Frame-geometry plumbing (§2.3): per-config Nc/Nsymb/gi override + runtime invariant
   check. (telecom_system.cc load_configuration + the post-reset block.)
3. ROBUST_3 config branch (§3): config-table entry, ID 103 define, ladder insert,
   is_robust_config extension, MFSK M/nStreams N/A (QPSK not MFSK — verify the
   MFSK-reinit block at :4873 is correctly skipped for ROBUST_3).
4. CAP_COHERENT_TIER (§4): define bit, widen MFSK cap fields 2→3b, OR into
   local_capability, gearshift guard parameter + call-site opt-in.
5. Tests (§5.D): geometry round-trip + cap round-trip extensions.
6. build.sh o3 + --test (full suite).
7. BER sanity (§6).
8. Commit (no attribution).
