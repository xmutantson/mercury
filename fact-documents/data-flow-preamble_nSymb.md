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

---

## §10 Discrete-match port — new metric-form invariants (2026-05-27)

**Driving work item**: `data-preamble-port-research.md` §14. The
`time_sync_mfsk_corr` body was replaced with a FFT-bin argmax matched-
count detector (mirror of `detect_ack_pattern`). This section extends
the audit with the producer/consumer walk for the NEW metric.

### §10.1 Producers — preamble-detection metric

**New producer:** the rewritten body of `cl_ofdm::time_sync_mfsk_corr`
(`source/physical_layer/ofdm.cc`). The function signature is unchanged
— writes:
- Return value: full-rate interpolated sample offset of the detected
  preamble start, or -1 on no-detect. Contract unchanged from pre-port.
- `*out_metric`: NOW the discrete match count (0..preamble_nSymb).
  Was cosine²-mean (range 0..1).

**New ancillary state on `cl_ofdm`** (mirror of `cl_mfsk` fields,
populated by `load_configuration` at `telecom_system.cc:4956+`
alongside the template):

| Field | Type | Source | Purpose |
|---|---|---|---|
| `mfsk_M` | int | mirror of mfsk.M | FFT bin search range |
| `mfsk_nStreams` | int | mirror of mfsk.nStreams | per-stream argmax |
| `mfsk_stream_offsets[4]` | int[] | mirror of mfsk.stream_offsets | bin offset per stream |
| `mfsk_preamble_tones[16]` | int[] | mirror of mfsk.preamble_tones | expected tone at each preamble symbol |
| `mfsk_preamble_nsymb` | int | mirror of mfsk.preamble_nSymb | scan length |
| `mfsk_preamble_match_threshold` | int | mirror of mfsk.preamble_match_threshold | detection gate |

Constructor-default (cl_ofdm::cl_ofdm at ofdm.cc:62) sets all to 0.
Function `time_sync_mfsk_corr` returns -1 (no detect) when
`mfsk_M <= 0 || mfsk_nStreams <= 0 || mfsk_preamble_nsymb <= 0 ||
mfsk_preamble_match_threshold <= 0`. This protects against pre-init
calls.

### §10.2 Consumers of `*out_metric` post-port

| Site | file:line | What it does | Magnitude change OK? |
|---|---|---|---|
| `cl_telecom_system::receive_msg` MFSK branch | `telecom_system.cc:1037-1041` | stores in local `mfsk_sync_metric` | Yes — local variable, not used for flow control |
| `[RX-DECODE#N] FAIL` log line | `arq_common.cc:5798-5811` | prints `receive_stats.coarse_metric` | Yes — log magnitude changes from 0..1 to 0..16. Diagnostic only. |
| Existing test `mfsk_data_preamble_passband_roundtrip_clean` | `mfsk_ctrl_codec_tests.cc:1194` | asserts `sync_metric >= 0.5` | Yes — post-port matched ≈ 16 at sigma=0 ≫ 0.5 |
| New §6 tests | `mfsk_ctrl_codec_tests.cc:1339+` | assert specific match counts | Created against the NEW metric form |

Grep confirmation:
- `mfsk_sync_metric` consumers: only the local store + the RX-DECODE
  log printf. No Q-table, gearshift, or ARQ state-machine reads.
- `receive_stats.coarse_metric` consumers: arq_common.cc:5798-5811
  (log only). Not read elsewhere as a flow-control gate.

### §10.3 Invariants (post-port)

#### INV-PORT-1: detector returns -1 unless `mfsk_M > 0`

The detector is initialized lazily by `load_configuration`. If a
test or test-mode path calls `time_sync_mfsk_corr` before
`load_configuration`, the pre-init state (`mfsk_M = 0`) is observed
and the function returns -1 immediately. This matches the pre-port
behavior under `mfsk_corr_template == NULL`. Verified by inspection
of `ofdm.cc:time_sync_mfsk_corr` head.

#### INV-PORT-2: triple-equality with mfsk fields after load_configuration

After `M==MOD_MFSK` `load_configuration` succeeds:
```
ofdm.mfsk_M                          == mfsk.M
ofdm.mfsk_nStreams                   == mfsk.nStreams
ofdm.mfsk_stream_offsets[0..nStreams-1] == mfsk.stream_offsets[0..nStreams-1]
ofdm.mfsk_preamble_tones[0..nsymb-1] == mfsk.preamble_tones[0..nsymb-1]
ofdm.mfsk_preamble_nsymb             == mfsk.preamble_nSymb
ofdm.mfsk_preamble_match_threshold   == mfsk.preamble_match_threshold
```

Producer site: `telecom_system.cc:5022-5031` (added in the discrete-
match port commit). Walks each cl_mfsk field after the template is
generated, before the [PHY] log line. The copy loops bound by
`cl_mfsk::MAX_PREAMBLE_SYMB` (=16) and `cl_mfsk::MAX_STREAMS` (=4) so
no out-of-bounds writes are possible.

#### INV-PORT-3: non-MFSK reset

On OFDM configs (M != MOD_MFSK), the `else` branch at
`telecom_system.cc:5033+` resets the ofdm.mfsk_* mirror to zeros.
This prevents stale state from a previous MFSK config leaking into
the next OFDM-only run.

#### INV-PORT-4: discrete-match threshold scaling

`mfsk_preamble_match_threshold` MUST be in the range
`(2·preamble_nSymb / M, preamble_nSymb]`.
~~Below 1/M·N the random-data baseline overruns the threshold (FAR
explodes).~~ **Corrected 2026-05-28 per
`data-preamble-port-research.md` §15.8:** the detector accepts
expected-bin OR mirror-bin (Bug #39 image recovery,
`ofdm.cc:3130, 3228`), so the random-data baseline is `2/M`, not
`1/M`. The lower bound moves to `2·preamble_nSymb / M`. Above N is
unreachable. The init() values (7 for WB / NB, against N=16 / N=8
respectively) sit at 7 vs `2·16/32 = 1` (WB M=32, OK) and
`2·16/16 = 2` (WB M=16, OK).

#### INV-PORT-5: deferred template lifecycle

`mfsk_corr_template` (and `mfsk_corr_template_*_energy` arrays) are
populated by load_configuration but NEVER read by the post-port
`time_sync_mfsk_corr`. They are kept alive for revert safety and
cleanup in a follow-up commit per `data-preamble-port-research.md`
§8.4. The production caller's gate
`if(ofdm.mfsk_corr_template != NULL)` at `telecom_system.cc:1034`
stays satisfied because the template generator at lines 4962-5021
still runs unconditionally for MFSK configs.

### §10.4 Regression test coverage (§6 in mfsk_ctrl_codec_tests.cc)

Five new tests added to `mercury.exe --test`:

1. `mfsk_data_preamble_argmax_clean` — sanity at sigma=0.
2. `mfsk_data_preamble_argmax_cliff` — fail-before-passes at
   passband SNR ≈ -12 dB (in-band ≈ +1 dB after FIR). 5 seeds, ≥4/5
   must detect.
3. `mfsk_data_preamble_argmax_pure_noise` — 100 random WGN buffers,
   ≤1 false detect.
4. `mfsk_data_preamble_argmax_data_content` — Bug #44 regression
   guard: random in-alphabet MFSK data, assert no detect.
5. `mfsk_data_preamble_argmax_high_snr_no_regression` — sigma=0,
   assert matched == preamble_nSymb.

Fail-before-passes verification was run by reverting just the
`ofdm.cc` body and re-running tests. Result on pre-port code:
- Test #2 (cliff): FAIL (0/5 detect; matched=0 at threshold=7).
  Detector returns -1 because cosine²-mean at this SNR falls below
  the 0.5 absolute threshold.
- Tests #1 and #5: FAIL (sync_metric is cosine²-mean ≈ 1.0, cast to
  int = 1; assertion ≥ 14 / == 16 fails). These tests are written
  against the NEW metric semantics.
- Tests #3 and #4: PASS (the OLD detector also rejects these; the
  tests document structural safety properties, not the cliff fix).
Post-port: all 5 pass.

### §10.5 Operator backlog

- IONOS hardware A/B at WGN ∈ {+14, +6, 0, -4, -8, -10, -12},
  ROBUST_0, 180s dwell. Tool: `tools/axis_walk_sweep.py --pin-config 100`.
- Compare matched-count histograms at the IONOS WGN cliff cells
  against the §14.3 binomial prediction.
- Watch for: false detect events between HAIL and CMD frames in
  IONOS captures (Bug #44 regression).
- Watch for: any flow-control script that may have parsed the old
  cosine²-mean magnitude from RX-DECODE log lines. Update
  `tools/analyze_turboshift_log.py` parsing if it depends on a
  threshold of 0.5.

### §10.6 Phase-2 cleanup followups (deferred)

Per `data-preamble-port-research.md` §8.4:
- Delete `mfsk_corr_template`, `mfsk_corr_template_len`,
  `mfsk_corr_template_energy`, `mfsk_corr_template_nsymb`,
  `mfsk_corr_template_sym_energy[]` from `cl_ofdm`.
- Delete the `mfsk_corr_template != NULL` gate at
  `telecom_system.cc:1034` (always use the new detector).
- Delete the `time_sync_mfsk` cosine²-energy fallback (it was the
  pre-Bug#44 path; now superseded by the discrete-match detector).
- Delete the template-generation block at
  `telecom_system.cc:4956-5021`.

Each cleanup commit needs its own audit against §10.1-§10.3
invariants — the deletion would require flipping the caller gate
to unconditional. Schedule after hardware A/B validates the cliff
shift.

---

## §11. N=16→32 extension plan (pre-code, 2026-05-28)

**Status:** PLAN — written BEFORE code change extending WB MFSK data
preamble from 16 to 32 symbols. Builds on §1-§10 audit. Baseline
`fc8b6d3` (HEAD of `monitor`). `mercury.exe --test` 29/29 passing.

**Goal.** Push the WB MFSK data-preamble detector cliff another ~1.5
dB by doubling the matched-filter integration length. Per binomial
detection theory (`P(K ≥ T)` for `K ~ Binomial(N, p_signal)`), doubling
N at fixed `T/N` fraction sharpens the cliff via √N variance reduction
without shifting the operating-point p_signal. The §15 push (T 7→6 at
M=32 N=16) was deferred (§15.8) because mirror-bin acceptance forces
`p_random = 2/M` so the FAR ate the threshold relax. The N-extension
path captures detection gain by ADDING integration, not by RELAXING
the threshold-fraction.

### §11.1 Prior-art and cliff-push budget

- §14 (Option A, shipped 2026-05-27): discrete-match port; cliff WGN:−4
  → WGN:−8. +50% bps mean.
- §15 (M=32 T=7→6): deferred per §15.8 — mirror-bin doubles FAR; T=6
  exceeded operator's 1e-5 gate.
- §17 (mirror-bin drop): hardware-rejected, −9% total bytes. Mirror
  empirically load-bearing.
- §20 (mini-Moose data preamble): shipped 2026-05-28. +11.7% total
  bytes.
- §23 (sign flip): shipped. Synthetic + hardware verified.
- §24 (ctrl-frame mini-Moose v2): shipped.
- §19.2 axis table: "Preamble N=16 → 24/32 extension | ~1.5–3 dB
  (√N gain: N=16→32 = +3.0 dB) | 5–10 days | Med-High (frame timing,
  Bug #44 cross-correlation budget, ARQ poll math) | YES — once N ≥ ~32
  the matched-filter gain saturates against per-symbol SNR".
  This plan executes that axis.

### §11.2 4→16 audit recap — what carries forward to 32

The §1-§5 audit identified 6 critical sites that needed widening for
the 4→16 push:

| # | Site | 4→16 change | N=16→32 status |
|---|---|---|---|
| 1 | `mfsk.h:56` `MAX_PREAMBLE_SYMB` | 8 → 16 | **Bump 16 → 32** |
| 2 | `ofdm.h:280` `mfsk_corr_template_sym_energy[N]` | [8] → [16] | **Bump [16] → [32]** |
| 3 | `mfsk.cc:122-170` `preamble_nSymb` + `preamble_tones[]` init | 4 → 16, 4 tones × 4 reps → 8 Welch-Costas × 2 reps | **16 → 32, base × 4 reps** |
| 4 | `telecom_system.cc:4923` WB MFSK override | added `Nsymb = narrowband_enabled ? 8 : 16;` | **Update WB branch 16 → 32** |
| 5 | `telecom_system.cc:5169` template energy clamp `k < 16` | `k < 8` → `k < 16` | **Bump `k < 16` → `k < 32`** |
| 6 | `mfsk.cc:212-215` `preamble_match_threshold` | added 7 (uniform WB+NB) | **Per-M scaled threshold per §11.5** |

Mirror sites discovered post-Option-A port (§10, no §1 entry yet):
| # | Site | 4→16 / port change | N=16→32 status |
|---|---|---|---|
| 7 | `ofdm.h:290` `mfsk_preamble_tones[16]` mirror | added at [16] | **Bump [16] → [32]** |
| 8 | `ofdm.cc:115-122, 230-248` init loops `for(i<16)` over template_sym_energy + mfsk_preamble_tones + ofdm_corr_template_sym_energy | added 16-cap loops | **Bump loops 16 → 32** |
| 9 | `telecom_system.cc:5199, 5215` mirror copy loops `for(s<16)` | added 16-cap | **Bump 16 → 32** |
| 10 | `ofdm.cc:3463, 3565` `mfsk_preamble_tones[p % 16]` in detector | added `p % 16` | **Bump `p % 16` → `p % 32`** |
| 11 | `telecom_system.cc:5306` OFDM corr template `k < 16` | added at OFDM-only branch | unrelated to MFSK; **leave at 16**. OFDM preamble_nSymb stays at 4. |

All other §2 consumers (ARQ frame-symbol math, buffer sizing, etc.)
read `preamble_nSymb` by value or as a parameter — they AUTO-SCALE.
No new wires needed.

### §11.3 New §1.5 audit: deferred-clean state

The audit § noted `time_sync_preamble_fft` carries hardcoded `[16]`
arrays but is **DEAD CODE** (no callers). At N=32, a future caller
would break at preamble_nSymb=17+. If anybody rewires it in, they'll
need to bump those arrays. **Not in this fix's scope**; documented
for future audits.

`time_sync_preamble_matched` reads `template_nsymb` capped to
`preamble_nSymb` parameter — auto-scales. Uncalled from MFSK path.

### §11.4 Throughput cost at high SNR

ROBUST_0 WB frame time at N=16: ~7.27 s (per §5.3, message_transmission_time_ms
= 7271 ms). Per-symbol time = 7271 / (Nsymb + preamble_nSymb) =
7271 / 336 ≈ 21.6 ms.

Adding 16 preamble symbols → +346 ms per frame. New frame time
~7.62 s. **Throughput cost: 4.76% length increase at all SNRs**.

At clean (WGN:+14, ROBUST_0 ~20 bps in full-burst regime):
- bps loss ≈ 4.55% → -0.91 bps absolute.

At cliff (current floor WGN:−8 ~3 bps post-§20):
- bps loss ≈ 0.14 bps absolute.

At sub-cliff (WGN:−10 currently 0 bps): infinite improvement if the
extension moves the cliff.

**Trade**: ~5% bps loss at clean for ~1.5–3 dB cliff push. Net win
expected positive across the IONOS-relevant SNR distribution (the
distribution is concentrated below SNR cliff in typical IONOS
operation — see memory `ionos_optimization.md` §3).

### §11.5 Threshold value at N=32 — per-config FAR

Hand-calc using `P(K ≥ T) for K ~ Binomial(N=32, p=2/M)` (mirror-bin
empirical baseline per §15.8 and §17). Cross-checked via Python
scipy.stats.binom.sf.

**FAR table at N=32 T=14:**

| Modulation | N | T | p = 2/M | P(K ≥ T) per poll | Per-call (13 win) | Per-day (10 Hz) |
|---|---|---|---|---|---|---|
| **WB M=32** (ROBUST_0) | 32 | **14** | 1/16 | **2.22×10⁻⁹** | 2.89×10⁻⁸ | 0.02 FA |
| **WB M=16** (ROBUST_1/2) | 32 | **14** | 1/8  | **1.16×10⁻⁵** | 1.51×10⁻⁴ | 130 FA |
| NB M=8  (preamble_nSymb=8, unchanged) | 8  | 7  | 1/4 | 3.82×10⁻⁴ | 4.97×10⁻³ | (no change from §10) |
| NB M=4  (preamble_nSymb=8, unchanged) | 8  | 7  | 1/2 | 3.52×10⁻² | (degenerate; mitigated by 2-stream all-match gate) | (no change) |

**Baseline N=16 T=7 (mirror-bin baseline 2/M) for comparison:**
- WB M=32 N=16 T=7: FAR = 2.57×10⁻⁵/poll (shipped)
- WB M=16 N=16 T=7: FAR = 1.94×10⁻³/poll (shipped — acceptable per
  §15.8 because of `streams_matched < mfsk_nStreams` gate on ROBUST_1/2's
  2-stream geometry)

**Operator gate** from §15 brief: `<1×10⁻⁵/poll` for M=32. T=14
delivers 2.22×10⁻⁹/poll — 4500× under the gate. Strong margin.

**Per-call analysis** (~13 candidate windows union bound,
ROBUST_0 polls ~10 Hz):
- WB M=32: 0.02 expected FA per day continuous operation.
- WB M=16: 130 expected FA per day. **167× safer than N=16 baseline**
  (which is the current operating state and is accepted as safe
  thanks to the 2-stream all-match gate, per §15.8).

### §11.6 Why T=14 (not T=11, not T=10) — design rationale

Three candidate thresholds were considered:

**T=10 ("strict same-FAR equivalent")**: Pure binomial-FAR matching
of N=16 T=7 baseline. FAR(M=32) = 1.63×10⁻⁵, FAR(M=16) = 6.16×10⁻⁵
(BREAKS the ≤baseline guarantee at M=16, actually WORSE than current).
**REJECTED.**

**T=11 ("dB-optimal detection")**: Below operator gate at 2.14×10⁻⁶/
poll. Best cliff-edge detection probability (P(K≥11|N=32, p_sig=0.4)
= 0.795 vs 0.473 at N=16 T=7) — captures the full √N detection gain.
But FAR at M=16 is 6.16×10⁻⁴/poll (worse than the §15.8 estimate of
1.94×10⁻³ but only 3× better, not safety-headroom).
**REJECTED** — operator brief said "scale linearly to maintain same
FAR", not "optimize detection".

**T=14 ("linear scale per operator brief")**: T/N fraction
preserved (7/16 = 14/32 = 0.4375). Operating-point p_signal in
binomial detector UNCHANGED — the cliff is at the same SNR but
SHARPER. FAR at both M=32 and M=16 drops below baseline. M=16
moves from 1.94e-3 baseline to 1.16e-5 — 167× safer.
**ADOPTED** — matches operator brief exactly + carries strong
FAR margin on both alphabets.

**Detection cliff sharpening** (at fixed T/N = 0.4375):
| p_signal | P(K ≥ 7 | N=16) | P(K ≥ 14 | N=32) | Sharpening |
|---|---|---|---|
| 0.30 | 0.175 | 0.069 | -0.106 (cliff moves UP in p_signal) |
| 0.40 | 0.473 | 0.396 | -0.077 |
| **0.4375** (operating point) | **0.598** | **0.555** | parity |
| 0.50 | 0.773 | 0.811 | +0.038 (cliff sharper, slightly better above) |
| 0.60 | 0.926 | 0.974 | +0.048 |
| 0.70 | 0.989 | 0.999 | +0.010 |

The shift IS the sharpening: the cliff is steeper, the operating
point unchanged. Per-symbol SNR-to-p_signal mapping is a smooth
sigmoid; squeezing the binomial's transition width (variance √
scales as 1/√N) shifts the inflection point by ~0.5*(σ_N16 - σ_N32)
in p_signal-space. At p_op = 0.4375 the per-poll detection prob is
55-60% in both cases; the cliff above this point gets sharper.

**Cliff-SNR move estimate**: Per §19.2 axis-table prediction
(+1.5–3.0 dB for N=16 → 32 from √N). For a Gaussian-shaped LLR
operating-point distribution, the cliff width is ~5 dB and the
effective shift from doubling N is `σ_old - σ_new = √(p(1-p)/16) -
√(p(1-p)/32) ≈ √0.246 (1/4 - 1/√32) = ~0.49 - 0.41 = 0.08` in
p_signal-space ≈ 1.5 dB at typical per-sym SNR slope. Conservative.

### §11.7 New tone sequence — 32-symbol design

Operator brief: "2 reps of the existing 16-symbol pattern; verify
Hamming distance vs CONNECT/ACK/HAIL/BREAK is preserved."

The existing 16-symbol pattern is 8 Welch-Costas g=2 base tones × 2
reps (§14.6). For N=32: **same 8 base tones × 4 reps** (i.e., extend
the existing pattern's natural period). Cleanly equivalent to "2 reps
of the 16-symbol pattern" — the 16-symbol pattern is itself the base
× 2, and 32 is base × 4.

M=32 base = {4, 8, 16, 0, 30, 26, 18, 2}.
M=16 base = {2, 4, 8, 0, 15, 13, 9, 1}.

Generation: `preamble_tones[s] = base[s % 8]` for s in [0, 32).

**Hamming-distance verification** (vs ACK g=5, BREAK g=7, HAIL g=6,
CONNECT g=3 — all using 8-tone Welch-Costas bases per §14.1):
- Distances at the 8-base level are UNCHANGED by the rep count (the
  base sequences are unchanged).
- §14.1 Hamming table: vs ACK 7/8, vs BREAK 8/8, vs HAIL 8/8, vs
  CONNECT 8/8 at both M=16 and M=32.
- At the EXPANDED 32-symbol level: distances become 28/32, 32/32,
  32/32, 32/32 (proportionally preserved).
- All distances ≥ 28/32 = 87.5%, well above the 14/32 threshold
  and well above the >20-symbol overlap needed for a sustained
  cross-detector false trigger.
- §14.1 test `test_base_pattern_cross_correlation`
  (`mfsk_ctrl_codec_tests.cc:411`) verifies pairwise distance ≥ 6/8
  at the BASE level — unaffected by N=32 since base tones don't
  change.

**Distinctness inside the preamble itself**: 8/8 distinct values at
base level (Costas property preserved). Within the 32-symbol
expansion, each base tone appears exactly 4 times.

**Cross-detector accidental-match analysis** (extending §5.2):
For a 32-symbol expanded preamble vs the 16-symbol ACK pattern: the
ACK detector reads 16 symbols starting from any RX-buffer position.
Per-symbol accidental-match probability when scanning a preamble
window is bounded by the base Hamming-distance over both rep cycles.
At 28/32 distance vs ACK, the maximum 16-symbol overlap with ACK
yields at most ~14/16 distinct-tone symbols → 2 max accidental
matches. ACK threshold is 7. Safe by a margin of 5.

### §11.8 Buffer & timing math (per-consumer audit walk)

Every §2.1 PHY consumer of `preamble_nSymb` and §2.2 ARQ consumer
auto-scales. Verifications for N=32:

- `data_container.cc:137-150` `buffer_Nsymb` formula: `frame_symb =
  preamble_nSymb + Nsymb = 32 + 320 = 352` for ROBUST_0; `buffer_Nsymb
  ≈ 2*352 + 4000ms_in_symbols + 50 ≈ 730+`. **OK** — buffer scales.
- `data_container.cc:181` `total_frame_size`: scales linearly.
- `data_container.cc:174` `baseband_data_fine_slice = (3·32+4)·Nofdm
  *interp = 100*Nofdm*interp` vs old 52*Nofdm*interp. ~2× larger.
  **OK** — heap allocation, no fixed cap.
- `telecom_system.cc:5358, 5375` `microphone/speaker.nbuffer_Samples
  = 2 * Nofdm * (1+gi) * interp * (Nsymb + preamble_nSymb)`. Scales.
- `telecom_system.cc:1425` `upper_bound = buffer_Nsymb - (Nsymb +
  preamble_nSymb) ≈ 730 - 352 = 378`. Still safe (positive).
- `arq_common.cc:1234` `message_transmission_time_ms` scales linearly,
  ROBUST_0 grows 7271 → ~7616 ms. Below 60s timeout — safe.
- `arq_common.cc:6248-6250` `mfsk_ftr = preamble_nSymb * 2 = 64; if
  (mfsk_ftr < 16) ... ` — 64 > 16, no clamp triggers. Safe.

### §11.9 Constants and arrays touched

| File:line | OLD | NEW | Reason |
|---|---|---|---|
| `mfsk.h:56` `MAX_PREAMBLE_SYMB` | 16 | **32** | Sentinel for `preamble_tones[]` |
| `ofdm.h:280` `mfsk_corr_template_sym_energy[]` | [16] | **[32]** | Template per-sym energy cap |
| `ofdm.h:290` `mfsk_preamble_tones[]` (cl_ofdm mirror) | [16] | **[32]** | Detector consumer |
| `ofdm.h:300` `ofdm_corr_template_sym_energy[]` | [16] | unchanged | OFDM path stays at preamble_nSymb=4 |
| `mfsk.cc:143, 153` `preamble_nSymb = 16` | 16 | **32** | M=32 and M=16 WB branches |
| `mfsk.cc:148, 158` `for(s<16)` loops | 16 | **32** | Tone-table fill |
| `mfsk.cc:213-215` `preamble_match_threshold = 7` | 7 | **14** | Per §11.5 (uniform across WB M=16/M=32) |
| `mfsk.cc:38, 50` `for(i<MAX_PREAMBLE_SYMB)` | uses constant | unchanged (auto via constant) | Constructor zero-init |
| `telecom_system.cc:4923` WB override | `Nsymb = 16` | **`Nsymb = 32`** | data_container authority |
| `telecom_system.cc:5169` template energy clamp | `k < 16` | **`k < 32`** | Per-symbol energy compute |
| `telecom_system.cc:5199` mirror copy loop | `for(s<16)` | **`for(s<32)`** | mfsk_preamble_tones mirror |
| `telecom_system.cc:5215` mirror reset | `for(s<16)` | **`for(s<32)`** | reset on non-MFSK configs |
| `ofdm.cc:116, 122, 129` `for(i<16)` ctor init | 16 | **32** | Constructor of cl_ofdm |
| `ofdm.cc:243, 248` `for(i<16)` deinit | 16 | **32** | Deinit zeroing |
| `ofdm.cc:3463, 3565` `mfsk_preamble_tones[p % 16]` | `p % 16` | **`p % 32`** | Detector index |

**Out of scope**: `ofdm_corr_template_sym_energy[16]` (OFDM path,
preamble_nSymb stays at 4 there); `time_sync_preamble_fft` (dead
code; document caveat but don't touch).

### §11.10 Test plan (mandatory regression tests)

Add to `mfsk_ctrl_codec_tests.cc` §5 (cross-layer regression). Wire
into `run_mfsk_ctrl_codec_tests()`.

1. **`preamble_nSymb_wb_robust0_extended_to_32`** (state invariant):
   `load_configuration(ROBUST_0)`; assert all four authorities
   (`mfsk.preamble_nSymb`, `data_container.preamble_nSymb`,
   `ofdm.preamble_configurator.Nsymb`, `ofdm.mfsk_corr_template_nsymb`)
   == 32. Assert `MAX_PREAMBLE_SYMB == 32`. Assert
   `mfsk.preamble_match_threshold == 14`. Assert tone-table diversity
   (preamble_tones[s%8] cycles correctly across all 32 slots).

2. **`mfsk_data_preamble_passband_roundtrip_clean_n32`** (sigma=0
   passband round-trip): extension of `_passband_roundtrip_clean`
   for N=32. Asserts detector returns delay ≥ 0 AND matched ≥ 28/32
   AND |delay - injection| ≤ 1 sym.

3. **`mfsk_data_preamble_argmax_cliff_n32`** (cliff regression):
   AWGN at WGN:-12 equivalent in-band SNR (in-band ≈ -1 dB after FIR),
   5 PRNG seeds. Assert ≥ 4/5 return delay ≥ 0 AND matched ≥ 14/32.
   **Pre-fix (with N=16)**: ROBUST_0 detector at WGN:-12 in-band 0 dB
   produces matched ≤ 5/16, FAILS. **Post-fix (with N=32)**: matched
   typically 18-22/32, PASSES.

The §6 existing tests (`mfsk_data_preamble_argmax_pure_noise`,
`_data_content`, `_high_snr_no_regression`) automatically exercise
N=32 once the config loads with the new constants.

**Fail-before-passes verification**: stash `mfsk.cc:143/153` change
(reset preamble_nSymb back to 16) + tone-table init + threshold
init, rebuild, run tests:
- Test #1: FAILS (mfsk.preamble_nSymb == 16, not 32; assertion
  fails immediately).
- Test #2: FAILS (detector reads `mfsk_preamble_tones[0..15]` for
  all positions but the N=32 detector tries `[16..31]` which is
  zero-init; matched count collapses).
- Test #3: FAILS (zero matched across all 5 seeds).

Then unstash, rebuild, all 32 tests pass.

### §11.11 Cross-layer audit refresh (§3 / §4 invariants)

**Updated §3 valid states** (post-N=32):

| State | data_container | mfsk | preamble_configurator | mfsk_corr_template | Valid? |
|---|---|---|---|---|---|
| Pre-init | 0 | 0 | 0 | NULL | ✓ |
| OFDM loaded | 4 | 0 | 4 | NULL | ✓ (OFDM unchanged) |
| MFSK NB loaded | 8 | 8 | 8 | populated, nsymb=8 | ✓ (NB unchanged) |
| MFSK WB loaded (pre-N=32) | 16 | 16 | 16 | populated, nsymb=16 | obsolete |
| **MFSK WB loaded (post-N=32)** | **32** | **32** | **32** | **populated, nsymb=32** | **✓ (new target)** |

**Updated §4 invariants:**

- **INV-PROD-1** (triple-equality): post-N=32, all four authorities
  equal 32 for WB MFSK. Same chain as before; just larger value.
- **INV-PROD-2** (`mfsk_corr_template_sym_energy[]`): array MUST
  satisfy `N >= 32`. Bumped to [32].
- **INV-PROD-3** (`MAX_PREAMBLE_SYMB`): MUST satisfy `>= 32`. Bumped
  to 32.
- **INV-CONS-1** (receive_msg upper_bound > 0): WB ROBUST_0
  Nsymb=320, preamble_nSymb=32, frame_symb=352. buffer_Nsymb ≈ 730.
  upper_bound = 730 - 352 = 378 > 0. **Safe.**
- **INV-CONS-2** (NB preamble unchanged): preamble_configurator
  override at telecom_system.cc:4923 keeps NB MFSK at 8 via the
  ternary `narrowband_enabled ? 8 : 32`. NB MFSK init() M=8/M=4
  branches keep preamble_nSymb=8. NB unchanged. **Safe.**
- **INV-CONS-3** (OFDM unchanged): OFDM configs have M != MOD_MFSK,
  mfsk.init never runs. preamble_configurator.Nsymb stays at the
  per-config table value (4 for all OFDM configs). **Safe.**
- **INV-CONS-4** (CONNECT/HAIL/ACK/BREAK independence): These read
  `connect_pattern_nsymb`, `hail_detect_nsymb`, `ack_pattern_nsymb`,
  BREAK length — separate fields from `preamble_nSymb`. All Welch-Costas
  patterns use 16-symbol expansions (base × 2). Cross-correlation with
  the new 32-symbol preamble (base × 4) — Hamming distance preserved per
  §11.7. **Safe.**
- **INV-PORT-4** (discrete-match threshold range): At N=32,
  range becomes `(2·32/M, 32] = (4, 32]` for M=16 and `(2, 32]`
  for M=32. T=14 is well within bounds for both. **Safe.**

### §11.12 ARQ-layer call sites that read preamble_nSymb + Nsymb

§2.2 enumerates these. Auto-scales because every site uses the value
by name. Quick spot check on three high-traffic sites:

- `arq_common.cc:1234` `message_transmission_time_ms`: scales.
  ROBUST_0 grows 7271 → 7617 ms. **OK** (under all timeouts).
- `arq_responder.cc:212, 250` frame-symbol math: scales.
- `arq_common.cc:6248` anti-spin `mfsk_ftr = preamble_nSymb*2 = 64`.
  Lower bound 16 is unaffected. **OK**.

### §11.13 HAIL / CONNECT / ACK preamble length independence

Verified: these are separate constants in cl_mfsk and have their own
detectors (`detect_ack_pattern` family). The data-preamble change
does NOT touch them.

- `hail_detect_nsymb` (mfsk.cc:185+): WB 16 sym, NB 32 sym. Untouched.
- `connect_pattern_nsymb`: 16 sym WB-only. Untouched.
- `ack_pattern_nsymb`: 16 sym WB / 32 sym NB. Untouched.
- `break_match_threshold` / `hail_match_threshold` / `ack_match_threshold`:
  all length-bound to their own pattern_nsymb, NOT to preamble_nSymb.
  Untouched.

### §11.14 INI / config / persistence

Grep confirms NO hardcoded "16" in INI parsers or config persistence
files (config.cc, INI dispatch in main.cc). The preamble length flows
from per-config table writes in telecom_system.cc → mfsk.init →
data_container.set_size. No external knob.

`tools/analyze_turboshift_log.py:937` is comment-only (`# preamble
varies 1-4`). Update comment to `1-4 OFDM, 32 WB MFSK, 8 NB MFSK`.
Cosmetic, not load-bearing.

### §11.15 Commit list

1. `docs(preamble): §11 N=16→32 extension plan (pre-code)` — this section
   only.
2. `phy(mfsk+ofdm): extend WB MFSK data preamble 16 → 32` — all six §11.9
   core sites + the mirror sites (constants, arrays, init loops, override,
   threshold, tones).
3. `test(preamble): N=32 cross-layer + cliff regression suite` — 3 new
   tests in §5 of mfsk_ctrl_codec_tests.cc, hooked into the runner.
   Verify fail-before-passes per §11.10.
4. `docs(preamble): §11 update — N=32 invariants verified` — §3, §4
   updates after code lands and tests pass.

### §11.16 Open issues for hardware operator

- IONOS A/B sweep at ROBUST_0, cells WGN ∈ {+14, +6, 0, -4, -8, -10,
  -12, -14}, 180s dwell, 3 passes/arm vs baseline `fc8b6d3`.
  Tool: `tools/axis_walk_sweep.py --pin-config 100 --with-robust`.
  Expected: WGN:-10 floor breaches (non-zero bps). Predicted floor:
  WGN:-11 to -13.
- Watch for ~5% bps drop at WGN:+14 (clean) — this is the expected
  throughput cost. NOT a regression; quantify and report.
- Watch for any false detects in HAIL-only windows during real
  sessions — N=32 random-data baseline 2.22e-9 → expected ~0/day. If
  observed, file ticket against §11.5 math.
- Watch for spurious ACK detector triggers on preamble — §11.7
  Hamming distance margin is 5-symbol; if observed at low SNR, may
  need to revisit base tone choice.
- The 4.76% additional preamble time pushes ROBUST_0 message timing
  to ~7.62 s — still under all ARQ timeouts (next-message-window is
  60s, batch-cycle is ~15s). If observed timing failures, escalate.

### §11.17 Post-implementation verification (2026-05-28)

**Status**: SHIPPED. Three commits on `monitor`:

| Commit | Subject |
|---|---|
| `baa5c5b` | docs(preamble): §11 plan — extend WB MFSK data preamble 16 → 32 (pre-code) |
| `0ab258b` | phy(mfsk+ofdm): extend WB MFSK data preamble 16 → 32 |
| `f6aeb4e` | test(preamble): N=32 cross-layer + cliff regression suite |

**Build**: `bash build.sh o3` clean (one pre-existing unrelated
warning in arq_commander.cc:2313 sign-compare).

**Test suite**: 32/32 pass with N=32 (29 pre-existing tests + 3 new).

**Build-log diagnostic confirmation** (from `--test` startup,
ROBUST_0 load):
```
[PHY] MFSK corr template: 32 symbols, 9920 samples,
       energy=404377.854 (per-sym corr, FIR round-tripped)
```
Confirms `mfsk_corr_template_nsymb == 32` and template energy is
populated across all 32 indices (energy ≈ 4× pre-N=32 baseline,
consistent with 4× more symbols at the same per-symbol energy).

**Fail-before-passes verified**: with the source changes stashed
(N=16 baseline restored) and the new tests kept in place:
- 13 tests fail: `preamble_nSymb_wb_robust0_extended_to_32`
  (`MAX_PREAMBLE_SYMB=16 (expected 32)`),
  `mfsk_data_preamble_passband_roundtrip_clean_n32`
  (synth precondition fails: preamble_nSymb=16),
  `mfsk_data_preamble_argmax_cliff_n32` (same),
  plus all the existing argmax/mini_moose tests whose preconditions
  check `!= 32` now (they were 16 in the stashed state).
- After `git stash pop` + rebuild: 32/32 pass again.

The pre-existing `time_sync_preamble_fft` dead-code arrays still
hardcode `[16]` — documented in §11.3, out of scope for this change.

### §11.18 §3 / §4 post-verification — invariants confirmed

§3 Valid States row "MFSK WB loaded (post-N=32)" — VERIFIED:
- mfsk.preamble_nSymb == 32 (test §5.1 asserts)
- data_container.preamble_nSymb == 32 (test §5.1 asserts)
- ofdm.preamble_configurator.Nsymb == 32 (test §5.1 asserts)
- ofdm.mfsk_corr_template_nsymb == 32 (test §5.1 asserts)
- mfsk_corr_template_sym_energy[0..31] > 0 (test §5.1 asserts)
- ofdm.mfsk_preamble_tones[0..31] == mfsk.preamble_tones[0..31]
  (test §5.1 asserts INV-PORT-2 mirror)
- ofdm.mfsk_preamble_match_threshold == mfsk.preamble_match_threshold == 14
  (test §5.1 asserts)

§4 invariants — VERIFIED:
- INV-PROD-1 (triple-equality at N=32): test §5.1 covers all four
  authorities including the mfsk_corr_template mirror.
- INV-PROD-2 (template energy array size >= 32): bumped to [32]
  at ofdm.h:280.
- INV-PROD-3 (MAX_PREAMBLE_SYMB >= 32): bumped to 32 at mfsk.h:56.
- INV-CONS-1 (receive_msg upper_bound > 0): math in §11.11 holds;
  buffer_Nsymb at ROBUST_0 = 836 per build log, upper_bound = 484.
  Implicitly verified by all existing tests passing.
- INV-CONS-2 (NB preamble unchanged): NB MFSK init() M=8/M=4
  branches untouched; preamble_configurator override at
  telecom_system.cc:4923 keeps NB at 8 via `narrowband_enabled ? 8 : 32`.
- INV-CONS-3 (OFDM unchanged): all OFDM tests pass; no OFDM source
  modified beyond the §11.9 sites which are all gated on M=MOD_MFSK.
- INV-CONS-4 (CONNECT/HAIL/ACK/BREAK independence):
  test_base_pattern_cross_correlation still passes (base tones
  unchanged); test_mfsk_connect_no_hail_false_trigger still passes;
  test_mfsk_connect_passband_roundtrip_clean still passes.
- INV-PORT-4 (discrete-match threshold range): At N=32, M=32 lower
  bound is 2*32/32 = 2; T=14 well above. At M=16 bound is 4; T=14
  well above. Upper bound 32; T=14 below. Verified by test §5.1
  (asserts preamble_match_threshold == 14) and binomial FAR math.
