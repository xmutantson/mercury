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
- ~~`receive_stats.coarse_metric` consumers: arq_common.cc:5798-5811
  (log only). Not read elsewhere as a flow-control gate.~~
  **CORRECTION (factdoc-refresh, 2026-06-07 — stale, flagged by race audit
  R057/R079):** FALSE. `receive_stats.coarse_metric` IS read as a flow-control
  gate in multiple ARQ and PHY sites, not "log only". The grep above missed the
  full-word `coarse_metric` reads in the RESPONDER recovery/BREAK block of
  `arq_common.cc` and in `telecom_system.cc`. Live flow-control consumers:
  - `arq_common.cc:6575` — `coarse_metric < 0.30` GATES the BREAK-pattern probe
    (RESPONDER only, `link_status==CONNECTED`). This is a true branch: it decides
    whether `detect_break_pattern_from_passband()` is even called.
  - `arq_common.cc:6696` — `&& coarse_metric >= 0.5` is a term in a
    recovery-acceptance predicate.
  - `arq_common.cc:6737` — `if(coarse_metric >= 0.5)` branch.
  - `arq_common.cc:6778` — `if(coarse_metric < 0.5)` branch.
  - `arq_common.cc:6806-6807` — `coarse_metric >= 0.15 && coarse_metric < 0.5`
    band gate (mid-confidence recovery handling).
  - `telecom_system.cc:1714` — `if(energy_ok && coarse_metric < 0.10)` decode-path
    branch.
  - `telecom_system.cc:2574` — `if(coarse_metric >= 0.97 && mean_H < 0.5)` branch.
  - `telecom_system.cc:2822` — `coarse_metric >= 0.97` term in a decode-decision.
  The original "(log only)" characterization referred to the `[RX-DECODE#N] FAIL`
  printf (arq_common.cc:5798-5811), which IS log-only — but that printf is NOT the
  only consumer, so the magnitude-change-safety conclusion of §10.2 must be
  re-evaluated against the gates above (a 0..1 → 0..16 metric rescale would
  break the `< 0.30`, `< 0.5`, `>= 0.5`, `< 0.10`, `>= 0.97` thresholds).
  Note: the BER harness forces `coarse_metric = 10.0` (telecom_system.cc:1184),
  which is ≥ all these thresholds, so the gates do NOT fire in sim — only on the
  live path. This is why the magnitude-change risk was not caught in sim.

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
`(preamble_nSymb / M, preamble_nSymb]`.
~~Below 1/M·N the random-data baseline overruns the threshold (FAR
explodes).~~ ~~**Corrected 2026-05-28 per
`data-preamble-port-research.md` §15.8:** the detector accepts
expected-bin OR mirror-bin (Bug #39 image recovery,
`ofdm.cc:3130, 3228`), so the random-data baseline is `2/M`, not
`1/M`. The lower bound moves to `2·preamble_nSymb / M`.~~

**Re-corrected 2026-05-28 per `data-preamble-port-research.md`
§16**: mirror-bin acceptance was dropped from `time_sync_mfsk_corr`
because the DATA preamble runs post-Moose-lock — mirror bin carries
only noise. The random-data baseline reverts to `1/M`. The lower
bound is `preamble_nSymb / M`. Above N is unreachable.

Current init values (`mfsk.cc:212-217`):
- M=32 WB ROBUST_0: T=6 vs `16/32 = 0.5` (OK; FAR 5.69e-6/poll).
- M=16 WB ROBUST_1/2: T=7 vs `16/16 = 1.0` (OK; FAR 2.57e-5/poll).
- NB M=8: T=7 vs `8/8 = 1.0` (OK; goes through detect_ack_pattern
  which still has mirror-bin acceptance, so effective p=2/M there;
  FAR 3.40e-6/poll).
- NB M=4: T=7 (NB ctrl path; same caveat).

The mirror-drop ONLY applies to the DATA preamble path
(`time_sync_mfsk_corr`). `detect_ack_pattern` (CONNECT/HAIL/ACK/
BREAK) is unchanged because those detectors run pre-Moose-lock and
NB needs mirror-bin to suppress in-alphabet image collisions
(`ofdm.cc:3341-3375`, Bug #39 origin).

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
