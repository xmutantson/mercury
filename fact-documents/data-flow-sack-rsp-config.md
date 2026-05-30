# Data-flow audit: SACK_RSP transport configuration (qtable-Q3)

Status: LIVING. Created 2026-05-30 on branch `fix/qtable-sack-robust`
(worktree off `monitor` HEAD 3b1726a). Pairs with the regression test
`test_sack_rsp_robust_config_*` registered in
`source/physical_layer/mfsk_ctrl_codec_tests.cc` and run by `mercury.exe --test`.

This document covers the shared state changed by the qtable-Q3 fix: the OFDM
**configuration** under which the partial-batch `SACK_RSP` frame is
transmitted (RSP) and decoded (CMD). It is a CLAUDE.md §5 cross-layer audit
(PHY ⇄ ARQ).

---

## §1 The bug (root cause)

The reverse partial-batch SACK frame `SACK_RSP` is an LDPC-coded OFDM frame
produced by `cl_arq_controller::send_sack_v2_frame()`
(`source/datalink_layer/arq_common.cc:4144`). Before this fix it was
transmitted on the **current data configuration** via `send_batch()`
(`arq_common.cc:4247` `set_mfsk_ctrl_mode(false)` → `arq_common.cc:4264`
`send_batch()`).

On a high data config (e.g. CONFIG_16, 32-QAM, LDPC rate 14/16) the forward
DATA frames are themselves near the decode cliff. The reverse `SACK_RSP`
inherits that *same* fragile PHY, so even when the commander decoded ~78% of
the forward batch, the commander cannot decode the reverse `SACK_RSP`
(`n_cmd_sack_events=0`). The batch is never confirmed → retransmit → timeout.

The in-code comment names the fix already:
`arq_common.cc:4258-4259` — *"If single-shot proves unreliable on lossy
channels we can revisit with a robust-config SACK_RSP TX (CFG10/CFG4) rather
than redundancy."*

### §1.1 When does `send_sack_v2_frame` actually fire?

`send_sack_v2_frame()` has exactly ONE live caller:
`source/datalink_layer/arq_responder.cc:1542`. It is the **fallback** for the
partial-batch path; the *preferred* partial path is the config-independent
robust MFSK ACK+SACK suffix `send_mfsk_ack_sack()`
(`arq_responder.cc:1526`), taken when
`MFSK_ACK_SACK_ENABLED && ack_mfsk.ack_sack_suffix_len() > 0`
(`arq_responder.cc:1507-1508`).

- **WB sessions**: `ack_mfsk.M == 16` ⇒ `ack_sack_suffix_len() == 13`
  (`include/physical_layer/mfsk.h:136`) ⇒ MFSK suffix path normally fires;
  the OFDM `send_sack_v2_frame` fallback is reached only when
  `send_mfsk_ack_sack()` returns 0 at runtime (`arq_responder.cc:1534-1538`).
  THIS is the CFG16 symptom in the SPEC.
- **NB sessions**: `ack_mfsk.M == 8` ⇒ `ack_sack_suffix_len() == 0`
  ⇒ the OFDM `send_sack_v2_frame` fallback is the ONLY partial-SACK return
  path, transmitted on the NB data config.

So this fix hardens the fallback path used by (a) WB when the MFSK suffix
misfires and (b) every NB partial batch.

---

## §2 The fix

Transmit / decode the fallback `SACK_RSP` on a fixed **robust OFDM
configuration** rather than the (possibly fragile) data configuration.

Chosen config: `SACK_RSP_FALLBACK_CONFIG = CONFIG_4`
(`include/common/common_defines.h`). CONFIG_4 is BPSK, LDPC rate 5/16
(`telecom_system.cc:4669-4675`) — robust BPSK like CONFIG_0 but with a much
shorter frame, so the reverse frame stays inside the receive ring window (the
scroll-off failure mode documented at `arq_common.cc:4248-4259`). Using an
MFSK `ROBUST_*` (100-102) config was rejected: `send_sack_v2_frame` builds an
LDPC-coded OFDM frame via `send_batch()`; routing it through the MFSK
modulator is a larger, separate change. A BPSK OFDM config keeps the exact
same `send_batch()`/`receive()` decode path on both ends.

The switch uses the **existing proven idiom** already used for the legacy
LDPC-ACK fallback at `arq_responder.cc:1100-1106`:
`load_configuration(robust, PHYSICAL_LAYER_ONLY, YES)` → `send_batch()` →
`load_configuration(data, PHYSICAL_LAYER_ONLY, YES)`. `PHYSICAL_LAYER_ONLY`
does **not** call `deinit_messages_buffers()` (only `FULL` does,
`arq_common.cc:1135-1142`), so message-queue state is preserved across the
switch.

Gating: the switch only happens when the live config is *more fragile* than
the fallback, i.e. `is_ofdm_config(cfg) && cfg > SACK_RSP_FALLBACK_CONFIG`.
If the session is already at/below CONFIG_4 (or on an MFSK/ROBUST config,
where `send_batch` already runs the robust MFSK PHY) the code path is
unchanged — current behavior preserved bit-for-bit. Helper:
`sack_rsp_needs_robust_downshift(int cfg)` in `common_defines.h`.

### §2.1 Cross-layer agreement (the load-bearing invariant)

`SACK_RSP` is decoded on the CMD by `cl_arq_controller::receive()`
(`arq_common.cc:5611`) using the **primary** `telecom_system` at its
`current_configuration` (the live ARQ path is single-config; parallel
decode at `arq_common.cc:5715` is `passive_monitor`-only). Therefore the CMD
must decode `SACK_RSP` at the **same** config the RSP transmitted it on.

The CMD OFDM SACK dispatch is `process_messages_rx_acks_data()`
(`arq_commander.cc:2378`), specifically the `receive()` call at
`arq_commander.cc:2636` and the `SACK_RSP` decode at
`arq_commander.cc:2656-2693`. The fix wraps **only** that `receive()` call in
a tight save/switch/restore to `SACK_RSP_FALLBACK_CONFIG`, gated on the same
`sack_rsp_needs_robust_downshift(current_configuration)` predicate so RSP-TX
and CMD-RX use the identical constant.

The single source of truth is the shared constant + predicate in
`common_defines.h`; the regression test asserts both sides compile against
the same value (§5 invariant test).

---

## §3 §5 audit — producer/consumer of the SACK_RSP transport config

The "shared state" is **which OFDM config carries SACK_RSP**. It is not a
stored variable; it is an implicit contract between two code sites that must
agree. Enumerated:

### §3.1 Producers (who sets the config under which SACK_RSP goes on the wire)

1. RSP TX: `send_sack_v2_frame()` `arq_common.cc:4247`+`:4264` — *before fix*
   it used whatever `current_configuration` held (the data config). *After
   fix* it loads `SACK_RSP_FALLBACK_CONFIG` around `send_batch()` when
   `sack_rsp_needs_robust_downshift(current_configuration)`.

### §3.2 Consumers (who decodes SACK_RSP / depends on its config)

1. CMD RX OFDM dispatch: `process_messages_rx_acks_data()`
   `arq_commander.cc:2636` `this->receive()` then `:2661`
   `decode_sack_v2_frame()`. *After fix* it switches the primary decoder to
   `SACK_RSP_FALLBACK_CONFIG` for that one `receive()` call under the same
   predicate.
2. `decode_sack_v2_frame()` `arq_common.cc:4467` — pure **byte-level** codec
   (CRC8 + bitmap over `messages_rx_buffer.data[]`); **config-independent**.
   The config only governs the PHY carriage, not the payload bytes. So the
   payload round-trip is unchanged by this fix; only the PHY layer that
   delivers those bytes changes.

### §3.3 Valid states / default-init

- Before any batch: CMD `current_configuration` = data config; RSP idem.
- During `RECEIVING_ACKS_DATA` (CMD SACK window): the ONLY OFDM frame the CMD
  decodes is `SACK_RSP`. `SET_LINK_PARAMS` is CMD→RSP (sent by CMD, handled by
  RSP at `arq_responder.cc:2556`), so the CMD never needs to OFDM-decode a
  data-config control frame in its own SACK window. ⇒ switching the CMD OFDM
  decode config to CONFIG_4 for that window cannot starve any other inbound
  OFDM frame.
- ACK detection in the same window is the dedicated `ack_mfsk` (M=16)
  matched-filter (`telecom_system.cc:3063`, "always use dedicated ack_mfsk
  — config-independent"). It does NOT depend on `current_configuration`, so
  the temporary OFDM config switch does not perturb ACK detection.

### §3.4 Invariants the consumers assume, and verification

- **I1**: CMD decodes SACK_RSP on the same config RSP TX'd it. *Maintained*:
  both sides keyed off the single predicate
  `sack_rsp_needs_robust_downshift()` on their respective live config; both
  resolve to `SACK_RSP_FALLBACK_CONFIG` (CONFIG_4) exactly when the data
  config is fragile, else neither switches.
- **I2**: message-queue state (`messages_tx[]`, `messages_rx[]`,
  `messages_control`, retx queue) survives the config switch. *Maintained*:
  `PHYSICAL_LAYER_ONLY` does not deinit buffers (`arq_common.cc:1135-1142`);
  no producer writes those queues inside the switch scope.
- **I3**: the ring buffer (`passband_delayed_data`) is large enough for the
  CONFIG_4 frame. *Maintained*: CONFIG_4 is a SMALLER OFDM frame than any
  config > 4; the ring is sized for the live (larger) config, so reading the
  shorter CONFIG_4 `signal_period` from it is safe (same direction as the
  monitor-decoder `buffer_Nsymb_min` reasoning at `arq_common.cc:910-924`).
- **I4**: on a partial batch the CMD will retransmit, which reloads the data
  config anyway. *Maintained*: the CMD restores the data config at the end of
  the scoped switch, BEFORE the retransmit-staging path runs; the retransmit
  path's own `load_configuration` is unaffected.
- **I5**: RSP restores the data config after SACK_RSP TX so the next inbound
  retransmit DATA batch decodes at the data config. *Maintained*: explicit
  restore after `send_batch()`, mirroring `arq_responder.cc:1106`.
- **I6** (found during audit): `last_data_configuration` must survive the RSP
  round-trip untouched — it drives BREAK recovery / config-toggle
  (`arq_common.cc:1397-1405`). `load_configuration(..., backup=YES)` stores the
  *old* `current_configuration` into `last_data_configuration`
  (`arq_common.cc:1270-1272`), so a `YES`/`YES` switch+restore would leave
  `last_data_configuration == SACK_RSP_FALLBACK_CONFIG` (corrupted). *Fix*: the
  RSP switch AND restore both pass `backup=NO`; `current_configuration` is
  restored explicitly to the saved data config, so `NO` costs nothing and keeps
  `last_data_configuration` exactly as it was. The legacy LDPC-ACK idiom
  (`arq_responder.cc:1100/1106`) uses `NO`/`YES`, which DOES perturb
  `last_data_configuration` to `ack_configuration` — acceptable there because
  it runs in the RESPONDER ACK context where the toggle target is already the
  data config; this fix is stricter (`NO`/`NO`) to be side-effect-free.

### §3.5 Uncommon paths checked

- **NB**: `send_sack_v2_frame` is the sole partial path. NB data configs are
  also OFDM (0-16 with NB carrier set), so the downshift to CONFIG_4-NB is a
  valid OFDM config in NB mode (the `narrowband_enabled` flag is unchanged by
  `load_configuration(cfg)`; only the modulation/rate change). The fallback
  still helps when the NB data config is > CONFIG_4.
- **BREAK / emergency**: the BREAK path (`arq_commander.cc:2344`,
  `send_break_pattern`) does not call `send_sack_v2_frame` and is reached on a
  different branch; unaffected.
- **Config-switch mid-window**: the scoped CMD switch is restored on the same
  poll, before any state machine transition; no `return` path inside the
  scope.
- **Already-robust data config**: predicate false ⇒ no switch ⇒ behavior
  identical to pre-fix. This is the "do no harm" guarantee for the common
  clean-channel CFG10/CFG4 case.

---

## §4 What this fix does NOT cover (local-test limits)

- End-to-end "the SACK loop now closes at CFG16 on a lossy link" can only be
  shown on the IONOS testbed (RF / fading). Local validation here is limited
  to: clean build, `mercury.exe --test` (30→32), and the SACK_RSP codec +
  config-agreement round-trip regression test. Per the qtable-Q3 SPEC this is
  the expected local scope; full validation is on the user's hardware run.
- The WB fallback only fires when the robust MFSK suffix `send_mfsk_ack_sack`
  misfires; this fix does not change how often that misfire happens (a
  separate concern). It only ensures that *when* the OFDM fallback is used,
  the reverse frame is robust.

---

## §5 Regression test

`test_sack_rsp_robust_config_roundtrip()` and
`test_sack_rsp_config_agreement_invariant()` in
`source/physical_layer/mfsk_ctrl_codec_tests.cc`, registered in
`run_mfsk_ctrl_codec_tests()` and run by `mercury.exe --test`:

1. **Round-trip**: pack a SACK_RSP payload `[bsi | bitmap | CRC8]` with the
   exact byte layout of `send_sack_v2_frame` (`arq_common.cc:4165-4204`),
   then decode it with the exact CRC8-verify + bitmap-unpack of
   `decode_sack_v2_frame` (`arq_common.cc:4480-4513`); assert byte-identity
   of the recovered bitmap and bsi, and that a 1-bit CRC corruption is
   rejected. (Guards the payload contract the PHY must deliver.)
2. **Config-agreement invariant**: assert
   `sack_rsp_needs_robust_downshift(CONFIG_16) == true`,
   `... (CONFIG_4) == false`, `... (CONFIG_0) == false`,
   `... (ROBUST_0) == false`, and that the resolved fallback constant is a
   valid robust OFDM config (`is_ofdm_config(SACK_RSP_FALLBACK_CONFIG) &&
   SACK_RSP_FALLBACK_CONFIG <= CONFIG_4`). This is the cross-layer "both sides
   agree" guard: RSP-TX and CMD-RX both call this predicate, so testing it
   pins the contract.
