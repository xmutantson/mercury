# data-flow-revsack-data-sack.md — REVSACK: D0-robust reverse data-SACK + cheap-miss

Living fact document for the REVSACK fix (revsack/design.json). The reverse data-SACK at
a climbed OFDM rung CRC-fails on a slow half-duplex turnaround, stalling nAcked_data ->
CMD-ACK-PAT timeout -> emergency_nack -> BREAK -> demote to ROBUST_0 (the whole climb
craters). FIX = (A) FEC-protect the shared robust-MFSK ACK suffix at the climbed rungs so
the confirm survives one turnaround; (B) make a residual data-SACK miss CHEAP (re-air the
same config + cumulative-n_r self-heal, not emergency_nack++). PLUS (P2) purge the spurious
no-op in-band SET_CONFIG.

Every claim is `file:line` cited against the wip/tiercross-impl tree at 243c6e9b (the base)
+ this commit's edits.

## §1 Root cause (re-grounded from source, not the design summary)

§1.1  The reverse data-ACK is ONE MFSK burst = [16-sym ACK base pattern] + [SACK suffix].
The base is a long energy correlation (threshold 7/16, robust to timing/CFO); the suffix
carries [bsi:8 | bitmap:30 | crc12:12] via per-symbol FFT argmax (mfsk.cc:706-711, the
uncoded branch of pack_ctrl_suffix). CRC12 is DETECT-ONLY (no FEC). ANY single per-symbol
argmax error fails CRC12 -> decode discarded (arq_commander.cc:4103-4123).

§1.2  On a slow turnaround at a climbed rung, inter-Pi clock drift + CFO de-aligns the
suffix symbol window -> per-symbol argmax errors while the base still matches 7/7. The
uncoded suffix cannot absorb them -> the confirm (and the n_r riding the bsi field) dies.

§1.3  The GF(16)-RA suffix FEC that absorbs 2-8 such errors ALREADY EXISTS (gf16ra::encode/
soft_decode; the SAME engine CONNECT + CONFIG_TAG ride) but was (a) master-disabled
(ARQ_ACK_SUFFIX_FEC_ENABLE=0, common_defines.h:66) and (b) scoped robust-tier-ONLY
(ack_suffix_fec_eligible() == is_robust_config, arq.h:4317). It NEVER covered the climbed
OFDM rungs where the failure lives.

§1.4  DECISIVE: even if the flag were flipped, the coded ACK was NEVER WIRED on the wire —
generate_ack_sack_pattern looped over ack_sack_suffix_len()==13 (mfsk.cc:878), and the ACK
passband sizing (ack_sack_pattern_passband_samples, telecom_system.cc) used the uncoded 13.
So a coded TX would emit only the first 13 of the N=52 codeword tones. This is the "ACK
coded-window sizing work" tier2-suffix-fec-design.md §21.3 named as the held-off
prerequisite. THIS commit builds it.

§1.5  And the existing cheap-miss (A3-decouple re-air, arq_commander.cc:3829) intercepts
ONLY the inband_connect_liveness_guard demote (the data-less livelock). The DATA-SACK miss
escalates via a SEPARATE path: data_ack_received==NO -> emergency_nack_count++ (:5278) ->
BREAK-gate demote (:5596 inband_route_failure_demote 'emergency_nack_threshold'). No
cheap-miss check there. Part B adds it.

## §2 The fix (what changed)

### Part A — D0-robust suffix FEC at the climbed rungs
- `ack_sack_coded_suffix_len()` (mfsk.h) — coded N when ack_suffix_fec_coded, else 13.
  `ack_sack_pattern_nsymb()` now uses it (was hardcoded +13).
- `generate_ack_sack_pattern` (mfsk.cc) loops over the coded length (was 13).
- `set_ack_suffix_fec(on, crc_fn, ctx)` (telecom_system.cc) — configures gf16ra(repfact 3,
  N=52, == CONNECT's value), sets ack_suffix_fec_coded, stores the WIRE-form CRC callback,
  RE-DERIVES ack_sack_pattern_passband_samples. Mirror of set_suffix_fec.
- `decode_ack_sack_coded_trybooth` (telecom_system.cc) — base-detect + CFO front-half
  (mirrors decode_ack_sack_from_passband_soft) then TRY-BOTH: uncoded-13-first (systematic
  prefix == hard pack), then the full N-tone GF(16) RA BP with its CRC12+type accept gate.
  Called internally by `decode_ack_sack_from_passband` when ack_suffix_fec_coded -> NO
  caller signature change (all 6 callers keep working). gf16ra global SAVED/RESTORED.
- `generate_ack_sack_pattern_passband` (telecom_system.cc) wraps the encode with
  save/restore-to-repfact-3 (the build_config_tag_tones discipline).
- `ack_suffix_fec_eligible()` (arq.h) WIDENED: robust tier OR inband_rate_feature_enabled()
  (the climbed OFDM rungs under the in-band stack). Legacy/non-inband byte-identical.
- `ack_suffix_fec_master_enabled()` (arq_common.cc) — compile-time macro OR the in-band
  default-on (inband_cheapmiss_resolve, explicit MERCURY_ACK_SUFFIX_FEC escape-hatch).
- `load_configuration` (arq_common.cc) ACK FEC enable hook: set_ack_suffix_fec(master &&
  eligible, arq_ack_sack_crc12_cb, this). Re-applied every config switch.
- `arq_ack_sack_crc12_cb` (arq_common.cc) — the WIRE-form ([bsi||bitmap]) CRC the GF(16)
  gate validates (matches send_mfsk_ack_sack's TX CRC + the uncoded RX re-check).
- RX capture-window tail sizing (arq_commander.cc:110, :4036) uses the coded length so the
  snapshot holds the whole codeword.
- send_mfsk_ack_sack no longer per-call sets/clears ack_suffix_fec_coded (hook-owned now).
- sim2_activate reconciles gf16ra to 3 when EITHER CONNECT or ACK FEC is coded.

### Part B — data-SACK cheap-miss decouple
- arq_commander.cc :5278 (right before emergency_nack_count++): IF inband_a3_decouple_
  enabled() (in-band default-on AND cumulative_ack_enabled) AND cmd_has_inflight_data_batch()
  AND !config_is_at_bottom -> RE-AIR the same config (load_configuration PHYSICAL_LAYER_ONLY,
  re-arm receiving, no ++; the in-flight batch is re-presented by the normal retx path that
  already marked the frames ACK_TIMED_OUT at :5106-5110). FAIL-BEFORE -DREVSACK_CHEAPMISS_
  FAILBEFORE.

### P2 — SET_CONFIG purge
- arq_commander.cc SET_CONFIG builder no-op/off-ladder fall-through: under inband, a
  non-tier-crossing no-op EARLY-RETURNS (drop slot, arm inband_unilateral_armed, no wire
  frame) instead of building a legacy SET_CONFIG. The tier-crossing-degenerate edge keeps
  the legacy fall-through. Flag-off byte-identical.

## §3 The CRC convention (the load-bearing cross-layer detail)
The data-ACK CRC is the WIRE form CRC12 over the 5-byte [bsi || bitmap32]
(send_mfsk_ack_sack:10806-10812; uncoded RX re-check arq_commander.cc:4105-4111). It is NOT
the typed40 [type:2|payload38] form the generic ctrl-suffix uses. gf16ra::soft_decode hands
its CRC callback the typed40 bytes it builds internally (pack_ctrl_typed40_msb). So the ACK
FEC gate uses arq_ack_sack_crc12_cb, which CONVERTS typed40 -> wire form before CRC. This
keeps: (1) the systematic prefix legacy-uncoded-decodable; (2) the caller's outer wire-form
re-check passing by construction; (3) the GF(16) gate validating the EXACT CRC the TX
embedded. A type!=0 suffix yields a different CRC -> rejected (type discriminator enforced
both by soft_decode's expected_type and the CRC).

## §5 CROSS-LAYER DATA-FLOW AUDIT (the required audit)

### Shared state #1 — ack_suffix_fec_coded + the ACK passband sizing
- Producers: set_ack_suffix_fec() (the ONLY producer now; sets the flag AND re-derives
  ack_sack_pattern_passband_samples TOGETHER — they can never desync). mfsk.cc:70 ctor
  default false. (REMOVED the old per-call set/clear in send_mfsk_ack_sack — that desynced
  the flag from the sizing, the §21.3 hazard.)
- Consumers: ack_sack_coded_suffix_len()/ack_sack_pattern_nsymb() (TX sizing); pack_ack_sack
  _payload->pack_ctrl_suffix(fec) (TX encode); generate_ack_sack_pattern loop bound (TX);
  ack_sack_pattern_passband_samples (TX passband length); decode_ack_sack_from_passband
  coded gate (RX); the RX capture-window tail sizing (arq_commander.cc:110/:4036).
- Valid states: false (uncoded 13-tone, legacy/non-inband/non-eligible default, byte-
  identical) | true (coded N=52, in-band + master-on + eligible rung). Default false.
- Invariant: TX and RX agree on coded-vs-uncoded per ACK. Maintained because BOTH peers run
  load_configuration -> the SAME enable hook on the SAME (master, eligible) inputs; and the
  RX try-both decodes either form (length-discriminated, NOT negotiated — the systematic
  codeword makes it backward-compatible BY CONSTRUCTION, tier2 §21.6). A peer mismatch ->
  coded gate fails CRC -> treated as no-ACK -> Part B re-airs (safe).
- Capture-ceiling hazard: RESOLVED in-tree — MAX_ACK_SACK_SUFFIX=64 >= N=52 (mfsk.h:375),
  GF16RA_MAX_N=64, last_ack_sack_suffix_tones[64], suffix_tones[MAX_ACK_SACK_SUFFIX],
  payload_tones[MAX_ACK_SACK_SUFFIX] all fit. (Paid by the CONNECT integration.)

### Shared state #2 — gf16ra process-global repfact
- The codec is process-global with current_repfact(). CONNECT uses configure(3) (N=52),
  CONFIG_TAG uses configure(2) (N=39) with save/restore (build_config_tag_tones:2728-2734).
- WHAT THE FIX ADDS: the ACK FEC uses repfact 3 (== CONNECT). The ACK TX
  (generate_ack_sack_pattern_passband) and RX (decode_ack_sack_coded_trybooth) BOTH
  save/restore the global around their encode/BP (belt-and-suspenders). The enable hook
  configure(3) is idempotent with CONNECT. sim2_activate reconciles to 3 on EITHER flag.
- INVARIANT preserved: no consumer at a different repfact is corrupted (every temporary
  configure is restored).

### Shared state #3 — emergency_nack_count
- Producers: arq_commander.cc:5278 (++ on data_ack_received==NO — the site Part B
  intercepts); reset to 0 on any clean/partial delivery (UNGATED) + the BREAK/demote
  re-arms; the control-failure path :3400 (separate, untouched).
- Consumers: :5596 BREAK-gate; the CFG16-HOLD FIX-4 carve-viability + FIX-9 D3 revack-
  starve deadlines (they key on the streak reaching threshold).
- WHAT THE FIX CHANGES: Part B SKIPS the ++ on a forward-healthy reverse-only miss
  (re-air + return BEFORE the ++). RESOLUTION (design §5): Part B SUPERSEDES FIX-4/FIX-9 for
  the forward-healthy case (a re-air is cheaper than a CFG16->CFG15 demote) — by returning
  before the ++ those deadlines never reach threshold on a forward-healthy miss (CORRECT,
  they were band-aids for exactly this miss). They STILL fire for the genuine-CFG16-
  starvation case (no in-flight batch / cumulative_ack off / truly silent) where Part B
  bails to the fall-through.

### Shared state #4 — cmd_batch_seq_id epoch
- The Part B re-air does NOT advance cmd_batch_seq_id (would orphan the in-flight batch into
  a gap -> GAP-ABORT). It touches NEITHER the epoch NOR the config (a3-decouple §5 verified
  pattern). SAFE.

### Shared state #5 — cumulative n_r (the self-heal carrier)
- Part A is what makes n_r actually ARRIVE (the suffix carrying it now decodes). Part B
  makes the residual gaps non-load-bearing. The two are mutually enabling: Part A without B
  still demotes on residual misses; Part B without A self-heals via an n_r that never decodes
  -> crawls. The existing apply-gate (CRC12 + bsi-window + dedupe, arq_commander.cc:4187+)
  validates the FEC-decoded n_r verbatim (no new consumer).

### D0 no-false-confirm / no-silent-loss
- Part A confirm rides the config-DISCRIMINATING bsi+bitmap+crc12 (NOT the retired config-
  invariant base pattern, arq_commander.cc:4139 inband_retag_confirm_from_base_pattern still
  a no-op). The GF(16) BP emits ONE codeword gated by recompute-CRC12 + 2-bit type (FAR
  6.1e-5). NO bare/short-detect accept.
- Part B gated on cmd_has_inflight_data_batch() (forward-healthy proof) + cumulative_ack_
  enabled (inside inband_a3_decouple_enabled — a dead reverse channel never advances n_r ->
  INV-T2-CONTIG keeps the batch uncovered -> genuine demote/BREAK fires). A forward DATA
  decode failure is a SEPARATE path (frame_gearshift_data_failed_*) that still demotes.

## §6 Must-not-regress
- Legacy / non-inband / non-eligible: master enable off -> set_ack_suffix_fec(false) ->
  uncoded 13-tone + no Part-B re-air -> BYTE-IDENTICAL.
- The genuine-death nets (frame_gearshift_data_failed_*, INBAND_LIVENESS_MAX_BREAKS,
  INV-T2-CONTIG, SESSION_DEAD_BATCHES floor) intact — Part B suppresses only the forward-
  healthy reverse-only miss.

## §7 Tests (fail-before / pass-after)
- Part A: test_revsack_suffix_fec_coded_decode (mfsk_ctrl_codec_tests.cc) — at CONFIG_0,
  inject 3 suffix symbol errors; FEC-OFF uncoded CRC fails (fail-before), FEC-ON coded BP
  recovers bsi/bitmap + CRC (pass-after); + byte-identical-when-off.
- Part B: test_revsack_data_sack_cheapmiss (arq_commander.cc, --test) — drives the :5278
  discriminator truth table; spine-live+inflight+not-bottom -> re-air; no-inflight /
  cumulative-off / at-bottom -> NO re-air (D0). FAIL-BEFORE -DREVSACK_CHEAPMISS_FAILBEFORE.
