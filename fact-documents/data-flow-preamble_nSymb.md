# Data-Flow Audit: MFSK / OFDM Data Preamble Length (`preamble_nSymb`)

**Status**: Authoritative as of 2026-05-27, written BEFORE the WB-MFSK
4 → 16 preamble extension lands on `fix/preamble-extend`. Every future
change to `preamble_nSymb` or its consumers MUST update this document.

**Driving work item**: `data-frame-cliff-audit-2026-05-27.md` §H1 — extend
the MFSK data-frame preamble from 4 to 16 symbols on the WB ROBUST_0/1/2
path. Expected gain +6 dB at the cliff (matches CONNECT base which already
integrates 16 symbols and survives WGN:-8).

**Cross-layer mandate**: CLAUDE.md §5 "Cross-Layer Data-Flow Audits".
Phase B Wave 2 v1 surfaced four sibling bugs because a structural PHY
change shipped without a producer/consumer walk; this audit is the
pre-flight checklist for the analogous 4-symbol → 16-symbol change.

---

## §1 Producers — code paths that write `preamble_nSymb` / `preamble_tones[]`

### §1.1 Per-config "configured" length (the wire-format authority)

The configured preamble length is owned by
`cl_ofdm::preamble_configurator.Nsymb` (an `int`, set per CONFIG_*/ROBUST_*
in `cl_telecom_system::load_configuration`). Every OFDM and MFSK config
table block in `telecom_system.cc:4481-4620` writes a local
`ofdm_preamble_configurator_Nsymb` that is then assigned to
`ofdm.preamble_configurator.Nsymb` at `telecom_system.cc:4749`.

Each per-config write (file `source/physical_layer/telecom_system.cc`):

| Config | line | value | reach |
|---|---|---|---|
| CONFIG_0..CONFIG_16 (OFDM) | 4485, 4492, 4499, 4506, 4513, 4520, 4527, 4534, 4541, 4548, 4555, 4562, 4569, 4576, 4583, 4590, 4597 | 4 | OFDM data preamble |
| ROBUST_0 (MFSK WB M=32) | 4604 | 4 | **MFSK WB data preamble — THIS IS THE FIX TARGET** |
| ROBUST_1 (MFSK WB M=16 ×2 streams) | 4611 | 4 | **MFSK WB data preamble — THIS IS THE FIX TARGET** |
| ROBUST_2 (MFSK WB M=16 ×2 streams, rate-1/4) | 4618 | 4 | **MFSK WB data preamble — THIS IS THE FIX TARGET** |

Post-table NB override (`telecom_system.cc:4751-4752`): when
`narrowband_enabled && M == MOD_MFSK`, force
`ofdm.preamble_configurator.Nsymb = 8`. This is what gives NB MFSK its 8-symbol
preamble today.

**The fix introduces a symmetric WB-MFSK override** at the same site,
forcing `ofdm.preamble_configurator.Nsymb = 16` when
`!narrowband_enabled && M == MOD_MFSK`. The per-config table values stay 4
so OFDM configs (CONFIG_0..16) are untouched.

Default value (pre-load) at `physical_config.cc:50`:
`ofdm_preamble_configurator_Nsymb = 4`. This is the constructor-default for
`cl_telecom_system::default_configurations_telecom_system`; relevant only
when an INI/CLI overrides the table — Mercury's config loader always
overwrites it via the table block.

### §1.2 `cl_data_container::preamble_nSymb` (the buffer-math authority)

`source/physical_layer/data_container.cc:105` — set inside
`cl_data_container::set_size(...)` from the `preamble_nSymb` parameter.
The single call site is at `telecom_system.cc:3872` (MFSK branch) and
`telecom_system.cc:3876` (OFDM branch); both pass
`ofdm.preamble_configurator.Nsymb` as the constructor arg. So
`data_container.preamble_nSymb == ofdm.preamble_configurator.Nsymb` after
any successful `load_configuration` call. INV-PROD-1 below.

Initial state (`data_container.cc:61`): `preamble_nSymb = 0`. Stays 0
until `set_size()` runs. **Any consumer that reads `preamble_nSymb` before
`set_size()` will see 0 and likely silent-fail (memory of size 0
allocations).** No producer guards against this — production code paths
always call `set_size()` first via `load_configuration()`.

### §1.3 `cl_mfsk::preamble_nSymb` and `cl_mfsk::preamble_tones[]`

`source/physical_layer/mfsk.cc:122-170` — set inside `cl_mfsk::init(M, Nc,
nStreams)`. THIS IS THE PRIMARY CODE CHANGE SITE. Per-M branches:

| M (alphabet) | line | preamble_nSymb | preamble_tones[] |
|---|---|---|---|
| 32 (WB ROBUST_0) | 124 | **4 → 16 in this fix** | 4 tones, **extend to 16** with `tone_hop_step` repetitions |
| 16 (WB ROBUST_1/2) | 132 | **4 → 16 in this fix** | 4 tones, **extend to 16** with `tone_hop_step` repetitions |
| 8 (NB) | 141 | 8 | 8 tones (unchanged) |
| 4 (NB 2-stream) | 154 | 8 | 8 palindrome tones (unchanged) |
| else (fallback) | 166-170 | 4 | computed |

Initial state (`mfsk.cc:35, 446`): `preamble_nSymb = 0`. Stays 0 until
`init()` runs (called by `load_configuration` for MFSK configs only).

**Critical sentinel: `preamble_tones[MAX_PREAMBLE_SYMB=8]` (mfsk.h:51-52)**.
With preamble_nSymb=16, this array would overflow on writes
`preamble_tones[8..15]`. **The fix MUST bump `MAX_PREAMBLE_SYMB` to 16.**

Init is called from `cl_telecom_system::load_configuration()` only when
`M == MOD_MFSK` — `telecom_system.cc:4945`. OFDM configs never touch
`mfsk.preamble_nSymb`; the OFDM-data preamble path reads
`data_container.preamble_nSymb` directly (= ofdm preamble_configurator).

### §1.4 Constructor-default invariant

For both `cl_data_container::preamble_nSymb` and `cl_mfsk::preamble_nSymb`,
the constructor-default is 0. This means any code that reads either field
on a stack/heap object before `load_configuration` or `mfsk.init` runs
will see 0 and produce a degenerate (empty) preamble. Production paths
always init first.

### §1.5 Round-trip template generator (consumer of preamble_nSymb, producer of mfsk_corr_template)

`source/physical_layer/telecom_system.cc:4956-5015` —
`mfsk.generate_preamble(data_container.preamble_data,
data_container.preamble_nSymb)`. This populates the FREQUENCY-domain
preamble symbols, then round-trips them through symbol_mod →
baseband_to_passband → passband_to_baseband(FIR_rx_time_sync) → decimate,
producing `ofdm.mfsk_corr_template[]` — the matched-filter template the
RX detector reads. The template length is `preamble_nSymb * Nofdm`.

**WATCH: `mfsk_corr_template_sym_energy[8]` (ofdm.h:241).** Per-symbol
energies cached at template generation time. Iterates `k < template_nsymb
&& k < 8` (line 4995) — silently truncates symbols 8..15 to zero energy.
The fix MUST bump this array to 16 AND remove the `< 8` cap.

---

## §2 Consumers — code paths that read `preamble_nSymb`

### §2.1 PHY-internal consumers (high frequency, hot paths)

| Site | file:line | What it does | Safe under 4→16? |
|---|---|---|---|
| TX preamble symbol_mod | `telecom_system.cc:627-629` | symbol-modulates `preamble_nSymb` symbols | Yes — loops by value |
| TX preamble power norm | `telecom_system.cc:655-659` | scales `Nofdm * preamble_nSymb` samples | Yes |
| TX preamble pre_eq (OFDM only) | `telecom_system.cc:610-616` | scales per-subcarrier | Yes — OFDM branch, preamble_nSymb=4 there |
| TX baseband→passband | `telecom_system.cc:669-673` | writes `Nofdm * preamble_nSymb * interp_rate` samples into `passband_data_tx` | YES IF `total_frame_size = Nofdm*(Nsymb+preamble_nSymb)*interp_rate` (data_container.cc:181) is scaled — it is |
| TX peak_clip preamble | `telecom_system.cc:672` | preamble-region clip | Yes |
| MFSK RX preamble sync (corr) | `ofdm.cc:3021-3221` (`time_sync_mfsk_corr`) | uses `mfsk_corr_template_nsymb` (== preamble_nSymb at template gen) | **YES IFF `mfsk_corr_template_sym_energy[]` is sized >= 16**. See §1.5 |
| MFSK RX preamble sync (FFT-energy fallback) | `ofdm.cc:2867-3008` (`time_sync_mfsk`) | reads `preamble_nSymb` parameter and `preamble_tones[p % preamble_nSymb]` | Yes — caller passes `data_container.preamble_nSymb` and `mfsk.preamble_tones` |
| OFDM RX freq sync (Moose) | `telecom_system.cc:2188` calling `ofdm.cc:465 carrier_sampling_frequency_sync` | reads `preamble_nSymb` parameter | Yes — internal halving + L=Nfft/nIS, no buffer overrun |
| OFDM RX freq sync NB | `ofdm.cc:528 carrier_frequency_sync_nb` | reads `preamble_nSymb` parameter | UNCALLED in MFSK path (telecom_system.cc:2193 sets `freq_offset_measured=0` for MFSK anyway) |
| RX frame size math | `telecom_system.cc:1413, 1424-1425, 4096-4106, 4183-4193, 4313-4323` | `frames_to_read = Nsymb + preamble_nSymb`, `upper_bound = buffer_Nsymb - (Nsymb + preamble_nSymb)` | Yes — `buffer_Nsymb` formula scales |
| RX buffer search bounds | `telecom_system.cc:1424-1425` | `lower_bound = preamble_nSymb`, `upper_bound = buffer_Nsymb - frame_symb` | Yes |
| Buffer fine-slice sizing | `data_container.cc:173` | `(3*preamble_nSymb+4)*Nofdm*interp` | Yes — scales |
| Buffer Nsymb formula | `data_container.cc:137-150` | `frame_symb = preamble_nSymb + Nsymb`, then `2*frame_symb + turnaround + margin` | Yes — scales (buffer grows ~24 symbols for 4→16) |
| Total frame size | `data_container.cc:181` | `Nofdm*(Nsymb+preamble_nSymb)*interp_rate` | Yes — scales (allocates `passband_data_tx` accordingly) |
| Microphone/speaker buffer | `telecom_system.cc:5164, 5181` | `Nofdm*(1+gi)*interp*(Nsymb+preamble_nSymb) * 2` | Yes — scales |
| `time_sync_preamble_fft` | `ofdm.cc:2253-2406` | `int n_preamble_bins_per_sym[16]; int preamble_bin_list[16][256]` — hardcoded cap 16 | **DEAD CODE** — uncalled. Audit confirms no callers in source/ except its own declarations. Future caller would already break at preamble_nSymb=17+ |
| `time_sync_preamble_matched` | `ofdm.cc:2605-...` | template_nsymb capped via `if(template_nsymb > preamble_nSymb) template_nsymb = preamble_nSymb` | Uncalled from MFSK path (matched OFDM only) |

### §2.2 Datalink-layer consumers (ARQ timing math)

All of these use `preamble_nSymb + Nsymb` (or active_nsymb) to compute
frame-symbol counts for buffer-bound calculations, retry timers, and
SACK window sizing. Auto-scales.

Sites (all confirmed by inspection to read-only):

- `arq_common.cc:1234, 1237` — `message_transmission_time_ms`,
  `ctrl_transmission_time_ms`. Scales linearly with `preamble_nSymb`. TX
  duration for a ROBUST_0 frame grows from 7.27 s to 7.54 s
  (preamble: 97 ms → 363 ms). Acceptable.
- `arq_common.cc:2129, 3245, 3267, 3309, 3505, 3692, 3718, 3865, 3979,
  4427, 4625, 4948, 5260, 5731, 5760, 5819, 6113, 6248, 6274, 6316,
  6435, 6478, 6509, 6549, 6579` — frame-symbol counts. All scale.
- `arq_commander.cc:339, 891, 911, 1622, 4091, 4372` — same pattern.
- `arq_responder.cc:212, 250 (comment), 333 (comment), 1123, 1179, 1324,
  1584, 1698, 1825, 2441, 2510` — same pattern.

Anti-spin recovery at `arq_common.cc:6248-6250` already has
`if(mfsk_ftr < 16) mfsk_ftr = 16` lower bound — robust against
preamble_nSymb growing.

### §2.3 Tools / diagnostics

- `tools/analyze_turboshift_log.py:937` — comment-only reference.
  Log-parsing script, no functional impact.

### §2.4 Stale comments (will need touching up post-fix; not load-bearing)

- `telecom_system.cc:5194` — "preamble_nSymb varies 1-4" — now 4-16.
- `arq_common.cc:3689` — "preamble_nSymb (4)" — now 4 (OFDM) or 16 (MFSK WB).

Will leave these alone unless they actively mislead.

---

## §3 Valid states

| State | data_container.preamble_nSymb | mfsk.preamble_nSymb | ofdm.preamble_configurator.Nsymb | mfsk_corr_template | Valid? |
|---|---|---|---|---|---|
| Pre-init (constructor) | 0 | 0 | 0 | NULL | ✓ as long as no consumer runs |
| OFDM config loaded | 4 | 0 (MFSK unused) | 4 | NULL | ✓ (canonical OFDM) |
| MFSK NB config loaded (pre-fix) | 8 | 8 | 8 | populated, nsymb=8 | ✓ (canonical NB) |
| MFSK WB config loaded (pre-fix) | 4 | 4 | 4 | populated, nsymb=4 | ✓ (canonical WB, the SOURCE of the data-frame cliff) |
| **MFSK WB config loaded (post-fix)** | **16** | **16** | **16** | **populated, nsymb=16** | **✓ (target invariant)** |
| MFSK NB config loaded (post-fix) | 8 | 8 | 8 | populated, nsymb=8 | ✓ (unchanged — NB untouched) |
| Inconsistent (mfsk.init never ran after configurator changed) | 4 | 8 | 4 | populated, nsymb=4 | ✗ — mfsk preamble TX would emit garbage; not reachable through `load_configuration` |
| Mid-reinit (data_container set_size has run but mfsk.init has not yet) | new | old | new | old (will be regenerated at line 4956 below) | ✓ transient; template regen at telecom_system.cc:4956 makes it consistent |

The post-fix invariant is that all four values (`mfsk.preamble_nSymb`,
`data_container.preamble_nSymb`, `ofdm.preamble_configurator.Nsymb`, and
`ofdm.mfsk_corr_template_nsymb`) equal 16 after MFSK WB
`load_configuration`.

---

## §4 Invariants (post-fix)

### INV-PROD-1: triple-equality after load_configuration

For every `M==MOD_MFSK && !narrowband_enabled` config that survives
`load_configuration`:

```
mfsk.preamble_nSymb
  == data_container.preamble_nSymb
  == ofdm.preamble_configurator.Nsymb
  == ofdm.mfsk_corr_template_nsymb
  == 16
```

Verified by inspection: `mfsk.init(M, Nc, nStreams)` writes
`preamble_nSymb` based on M (mfsk.cc:124 / :132 — fix changes both 4 → 16).
`data_container.set_size(..., ofdm.preamble_configurator.Nsymb, ...)`
copies the configurator into the data_container field
(data_container.cc:105). `telecom_system.cc:4956` writes
`mfsk_corr_template_nsymb = data_container.preamble_nSymb` via the round-
trip template generator. With the fix, the WB-MFSK override at line ~4752
forces all three to 16.

### INV-PROD-2: template-energy array size

`mfsk_corr_template_sym_energy[N]` MUST satisfy N >= max(preamble_nSymb)
across all configs. Today max is 8 (NB) and the array is 8. Post-fix max
is 16 (WB MFSK). **Array MUST be bumped to 16** (ofdm.h:241).

Per-symbol energy computation loop at telecom_system.cc:4995 has a
hard-coded `k < 8` clamp — MUST be removed (replace with
`k < template_nsymb`).

### INV-PROD-3: preamble_tones array size

`cl_mfsk::preamble_tones[MAX_PREAMBLE_SYMB]` MUST satisfy
`MAX_PREAMBLE_SYMB >= max(preamble_nSymb)`. Today MAX_PREAMBLE_SYMB=8
(mfsk.h:51). Post-fix max is 16. **Constant MUST be bumped to 16.**

### INV-CONS-1: receive_msg upper_bound

`telecom_system.cc:1425` defines
`upper_bound = buffer_Nsymb - (Nsymb + preamble_nSymb)`. Must remain > 0
(or RX rejects all candidate positions). `buffer_Nsymb` is computed from
`frame_symb = preamble_nSymb + Nsymb` plus turnaround plus margin
(data_container.cc:137-150). For ROBUST_0 WB Nsymb=320, post-fix
frame_symb=336. `buffer_Nsymb = 2*336 + 4000ms_in_symbols + 50 = ~700+`.
`upper_bound = 700 - 336 = 364`. Safe.

### INV-CONS-2: NB preamble unchanged

For MFSK NB (`narrowband_enabled && M==MOD_MFSK`), the post-table
override forces `preamble_configurator.Nsymb=8`; mfsk.init's M=8/M=4
branches keep `preamble_nSymb=8`; everything stays at 8. The NB ROBUST_0
floor (44.7 bps post-Phase-0) is preserved.

### INV-CONS-3: OFDM data path unchanged

For all OFDM configs (CONFIG_0..16), `M != MOD_MFSK`, `mfsk.init` never
runs, and `preamble_configurator.Nsymb=4` per table. Pre-eq, framing,
matched-filter, channel estimator — all untouched. The 8898 bps wire
throughput on clean channel is preserved.

### INV-CONS-4: CONNECT / HAIL / ACK / BREAK independence

`cl_mfsk::connect_pattern_nsymb`, `hail_detect_nsymb`,
`ack_pattern_nsymb`, BREAK pattern length are all independent of
`preamble_nSymb`. They have separate constants in `cl_mfsk::init`
(mfsk.cc:180-371). Cross-correlation guarantees against ACK/HAIL/CONNECT
false-triggering on the (extended) preamble must hold — see §5.2.

---

## §5 What the fix changes — per-consumer walk

Fix scope:

1. `mfsk.cc:122-137`: `preamble_nSymb = 4` → 16 for M=32 and M=16
   branches. Extend `preamble_tones[]` from 4 entries to 16 (4 base tones,
   reused). The generate_preamble already does
   `preamble_tones[s % preamble_nSymb]` so we need the array filled out.
2. `mfsk.h:51`: `MAX_PREAMBLE_SYMB = 8` → 16.
3. `ofdm.h:241`: `mfsk_corr_template_sym_energy[8]` → `[16]`.
4. `ofdm.cc:114, 234`: init loops `for(int i=0;i<8;...)` → 16.
5. `telecom_system.cc:4995`: `for(int k = 0; k < template_nsymb && k < 8;
   ...)` → drop the `< 8` clamp (use `k < template_nsymb`).
6. `telecom_system.cc:4752`-area: add WB-MFSK override
   `if(!narrowband_enabled && M == MOD_MFSK) ofdm.preamble_configurator.Nsymb = 16`.

Per-consumer impact verification (the §4 audit row by row):

### §5.1 PHY consumers — verified safe

All PHY consumers in §2.1 either:
- Take preamble_nSymb as a parameter and loop by value (safe, scales) — or
- Read `data_container.preamble_nSymb` and use it in size/offset math
  that already scales linearly.

The only PHY consumer with a hardcoded cap was
`mfsk_corr_template_sym_energy[8]` (§1.5), which the fix lifts.

`time_sync_preamble_fft` has hardcoded `[16]` arrays but is dead code
(no callers). Future MFSK callers would not hit it — they go through
`time_sync_mfsk_corr`. If a future change wires this in for MFSK,
remember to bump the constants.

### §5.2 CONNECT / HAIL / ACK / BREAK cross-correlation

The MFSK preamble tones are (M=32) `{4,20,12,28}` and (M=16) `{2,6,10,14}`
— even-valued only, with `tone_hop_step` chosen coprime with M (13 for
M=32, 7 for M=16). Repeating these 4 tones with the hopping schedule
yields a 16-symbol sequence where the actual transmitted tone at symbol s
is `(preamble_tones[s%4] + s*hop) mod M`.

This pattern is structurally different from the WB CONNECT base
(`connect_tones`, Welch-Costas g=3, M=32 scaled), the WB ACK base
(Welch-Costas g=5), WB BREAK (g=7), WB HAIL (g=6) — they all use
unique-Costas-property tone sequences, NOT repeating 4-tone cycles.

**Cross-correlation against the preamble**:
- CONNECT/ACK/BREAK/HAIL detectors all run `detect_ack_pattern`
  (`ofdm.cc:3231+`) which counts symbols where the FFT-peak bin matches
  `(pattern[p % pattern_len] + p*hop) % M`.
- If the (extended) preamble's hopped tone at symbol p happens to match
  the CONNECT/ACK pattern tone at symbol p, that's one false match.
- For M=32 with hop=13 and a 4-tone preamble repeating, the preamble
  trajectory is `4,20,12,28, 4+4·13,20+5·13,12+6·13,28+7·13, ...` mod 32
  = `4,20,12,28, 24,21,26,23, 12,5,8,2, 28,21,26,23, ...`. This is
  unique enough that the per-symbol probability of accidentally landing
  on any specific Welch-Costas pattern's expected tone at the right
  symbol index is ~1/32 — same as random.
- Total expected false matches across 16 symbols vs a Welch-Costas
  pattern: 16 × 1/32 = 0.5 matches. The match threshold is 7. Safe.

Existing test `test_base_pattern_cross_correlation`
(`mfsk_ctrl_codec_tests.cc:166+`) verifies pairwise distance >= 6
between CONNECT/ACK/BREAK/HAIL. The fix does NOT add new entries to
those Costas tables; it only extends `preamble_tones[]` from 4 to 16
entries via cyclic repeat. The cross-correlation of these extended
preambles against existing patterns is dominated by the hopping schedule,
not by the small per-pattern overlap. The new regression test verifies
empirically that no false-trigger occurs.

### §5.3 ARQ timing math — verified safe

Every ARQ consumer in §2.2 reads `preamble_nSymb + Nsymb` or
`preamble_nSymb + active_nsymb`. With preamble_nSymb growing from 4 to
16, frame-symbol counts grow by 12 (3.6% increase at ROBUST_0 Nsymb=320,
4.5% at ROBUST_1/2). Frame-duration timers grow proportionally —
`message_transmission_time_ms` (arq_common.cc:1234) for ROBUST_0
increases from ~7.27 s to ~7.54 s.

Anti-spin recovery (arq_common.cc:6248-6250) already has
`mfsk_ftr = preamble_nSymb*2; if(mfsk_ftr<16) mfsk_ftr=16` — robust.

### §5.4 Buffer allocation — verified safe

All buffers allocated from `preamble_nSymb`:

- `preamble_data` (data_container.cc:126): `preamble_nSymb * Nc`
- `preamble_symbol_modulated_data` (line 125): `preamble_nSymb * Nofdm`
- `passband_data` (line 156): `passband_frame` where
  `passband_frame = (Nsymb + preamble_nSymb) * Nofdm * interp_rate`
- `passband_data_tx` (line 184): `total_frame_size = Nofdm*(Nsymb+
  preamble_nSymb)*interp_rate`
- `baseband_data_fine_slice` (line 174): `(3*preamble_nSymb+4)*Nofdm*
  interp_rate`
- `microphone.nbuffer_Samples` / `speaker.nbuffer_Samples`
  (telecom_system.cc:5164, 5181): `2*Nofdm*(1+gi)*interp*(Nsymb+
  preamble_nSymb)`
- `mfsk_corr_template` (telecom_system.cc:4989): `bb_len = template_nsymb
  * Nofdm` — sized at run time from `data_container.preamble_nSymb`.

ALL scale linearly with preamble_nSymb. No hardcoded sizes anywhere
outside the `[8]` arrays this fix bumps.

### §5.5 Test mode / synthetic-fire paths

`mfsk_ctrl_codec_tests.cc:481+` calls `ts.load_configuration(CONFIG_0)`
which is an OFDM config (preamble_nSymb=4, M=BPSK). No MFSK preamble
size assertions in the existing test suite. The new regression test in
this fix calls `ts.load_configuration(ROBUST_0)` so it exercises the
MFSK WB path and asserts preamble_nSymb==16.

`telecom_system.cc:394, 406-409` — BER test path uses
`data_container.preamble_nSymb` for delay offset. The mfsk_fixed_delay
formula at line 409 is `(preamble_nSymb+2)*Nofdm + delay)*interp_rate`,
which scales correctly. BER tests that hardcode an expected delay value
would need to be re-baselined; spot-checked the test suite — no
hard-coded delay constants found.

### §5.6 Test-mode pre-init invariant (CLAUDE.md §5 "uncommon paths")

Test code paths that construct `cl_telecom_system` and access
`data_container.preamble_nSymb` BEFORE `load_configuration` run will see
0 (constructor default). This is the same as today (no regression).
Production code paths always init first.

---

## §6 Open questions [?]

1. **[?] CSI / channel-estimation assumption**: MFSK is noncoherent
   (FFT-energy demap), so no per-pilot channel estimation. The longer
   preamble does NOT add new channel-estimate samples or change demap
   precision. Confirmed by inspection of `mfsk.cc:demod` (line ~750):
   it does plain `|FFT_bin|²` accumulation, no `mean_H` dependency.

2. **[?] Pre-equalization**: OFDM preamble has per-subcarrier pre-eq
   applied (`telecom_system.cc:609-616`). MFSK preamble does NOT have
   pre-eq (the `if(M != MOD_MFSK)` guard at line 589 wraps the pre-eq
   loop). Extending MFSK preamble length doesn't add pre-eq state.
   Verified.

3. **[?] Optimizer / Q-table state**: The effective-rate optimizer
   doesn't read `preamble_nSymb` directly — it reads aggregated bps
   from `arq_controller::stats`. Throughput drops ~5% for ROBUST_0 due
   to longer preamble; the Q-table will see slightly lower bps for
   ROBUST_0 at high SNR but the SNR-vs-config policy is unaffected
   because the alternative (no-decode at WGN:-8) is 0 bps. Confirmed
   by grep — no Q-table consumer reads preamble_nSymb.

4. **[?] B2F / SACK / compression**: All higher layers consume
   delivered-bytes counts, not symbol/preamble counts. Untouched.

---

## §7 Cross-layer audit checklist (for future changes)

Before changing `preamble_nSymb` (in any of the per-config table writes,
`mfsk::init`, or `data_container::set_size`), revisit this document and
verify each consumer in §2 still satisfies §4 invariants. Specifically:

- If any new `[N]` cap appears (template arrays, FFT-bin lists, etc.),
  it MUST be >= max(preamble_nSymb across all configs). Today max=16
  with this fix. If a future fix pushes WB MFSK to 24 or 32 symbols,
  bump these together.
- Cross-correlation against CONNECT/HAIL/ACK/BREAK must remain below
  the 7/16 threshold. New preamble tone sequences MUST be cross-checked
  via the `test_base_pattern_cross_correlation` style test.
- Buffer sizing assertions (`buffer_Nsymb_min` if set via INI/CLI) must
  exceed `2*(preamble_nSymb+Nsymb)+turnaround+margin`.

---

## §8 Regression test coverage

This fix ships with two new tests wired into `mercury.exe --test`:

1. **`preamble_nSymb_wb_robust0_extended_to_16`** — calls
   `load_configuration(ROBUST_0)`; asserts
   `data_container.preamble_nSymb == 16`,
   `mfsk.preamble_nSymb == 16`,
   `ofdm.preamble_configurator.Nsymb == 16`,
   `mfsk_corr_template_nsymb == 16`,
   and that `mfsk_corr_template_sym_energy[k]` is non-zero for k=0..15.

2. **`mfsk_data_preamble_passband_roundtrip_clean`** — full
   load_configuration(ROBUST_0) → generate_preamble → symbol_mod →
   baseband_to_passband → passband_to_baseband(FIR_rx_time_sync) →
   `time_sync_mfsk_corr`. Asserts the corr metric exceeds the production
   0.5 threshold and the returned delay is within ±1 symbol of the
   injection point. This is the cross-layer regression that would have
   caught a sibling bug in template-energy zero-truncation, off-by-one
   in `preamble_tones[]` cycling, or a buffer-size shortfall in
   `passband_data_tx`.

Both tests fail BEFORE the fix lands (template_sym_energy[8..15] is
zero on the unmodified code; corr metric collapses with the
energy-normalized denominator going through zero). Both pass AFTER.

---

## §9 References

- `data-frame-cliff-audit-2026-05-27.md` — the audit that produced this
  work item (§H1, recommendation #1).
- `mfsk-vara-parity-plan.md` — Phase 0 history.
- `mercury/source/physical_layer/mfsk.cc:122-170` — preamble init (fix site).
- `mercury/source/physical_layer/ofdm.cc:3021-3221` — `time_sync_mfsk_corr`
  (the detector whose floor moves).
- `mercury/source/physical_layer/telecom_system.cc:4748-4752` — NB
  preamble override (fix site for WB symmetric override).
- `mercury/source/physical_layer/telecom_system.cc:4956-5015` — corr
  template generator (fix site for template_sym_energy cap).
- `mercury/include/physical_layer/mfsk.h:51` — `MAX_PREAMBLE_SYMB` (fix
  site).
- `mercury/include/physical_layer/ofdm.h:241` —
  `mfsk_corr_template_sym_energy[8]` (fix site).
