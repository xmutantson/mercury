# SIM_INPROC 2-instance bigblock de-whiten faithfulness — finding (2026-06-05)

Ref: integ/bigblock-final-2026-06-05 @c746064 (worktree C:/Users/kamer/mercury_wt/bigblock-final)

## VERDICT
SIM_INPROC (MERCURY_SIM_2INST=1 -m SIM_INPROC) with MERCURY_BIGBLOCK_FRAMING=1 at
CFG16 is a FAITHFUL reproducer. It drives the SAME real receive_bigblock decode
AND the SAME real arq_common carve de-whiten that HW uses. It does NOT bypass
either. It WOULD reproduce the 0-byte delivery.

## Call chain (sim drives REAL process_main on each instance)
- main.cc:1782 -> cl_arq_controller::test_sim_inproc_2() (arq_commander.cc:10032)
- Loop drives REAL A->arq.process_main() (TX) and B->arq.process_main() (RX),
  arq_commander.cc:10271 / 10292. Payload staged into A's REAL fifo_buffer_tx
  (:10216); delivered bytes drained from B's REAL fifo_buffer_rx (:10321-10327).
- TX (commander A): process_main -> process_messages (arq_common.cc:3009) ->
  process_messages_commander -> send_batch (arq_common.cc:4132). send_batch:4189
  calls bigblock_send_one_block() -> telecom_system->transmit_byte ->
  transmit_bigblock (telecom_system.cc:8099). TX whiten ONCE at
  transmit_bigblock telecom_system.cc:8161 (bigblock_whiten_payload_bits).
- RX (responder B): process_main -> process_messages (3009) ->
  process_messages_responder (arq_responder.cc:30) -> process_messages_rx_data_control
  (:45, fires when connection_status==RECEIVING) -> receive() (:431).
- receive() (arq_common.cc:6736): calls receive_byte (:6864) AND bigblock_receive_carve
  (:6901). receive_byte branches to receive_bigblock (telecom_system.cc:1024-1026)
  on the SAME gate (bigblock_framing_enabled && M!=MFSK && cfg==CONFIG_16).
- carve gate at arq_common.cc:6888 is identical: bigblock_framing_enabled &&
  M!=MFSK && current_configuration==CONFIG_16 && bigblock_last_rx_K>0.

## Whitening accounting (CORRECTS the task's "double de-whiten" hypothesis)
- TX whiten: EXACTLY ONCE, transmit_bigblock telecom_system.cc:8161
  (bigblock_whiten_payload_bits over payload_bits_len = Kpack*ldpc.K, after
  byte_to_bit, BEFORE bigblock_tx_passband LDPC-encode). Seed BIGBLOCK_WHITEN_SEED
  =0x5A3C96E1 (telecom_system.cc:6982).
- receive_bigblock (telecom_system.cc:8188-8280) does NOT de-whiten. It decodes
  via bigblock_rx_passband (:8243) into bigblock_rx_infobits and stashes the bits
  RAW (still whitened). No bigblock_whiten_bits call anywhere in receive_bigblock.
- carve de-whiten: EXACTLY ONCE, bigblock_receive_carve arq_common.cc:3892
  (telecom_system->bigblock_whiten_bits over nbits=K*sub_len*8). Same seed.
- => LIVE path = ONE whiten (TX) + ONE de-whiten (carve). NOT a double de-whiten.
  The "receive_bigblock de-whitens once + carve de-whitens again" hypothesis is
  REFUTED by the code: receive_bigblock does not de-whiten.

## Why the BIGBLOCK_LIVE validator passes 8/8 yet HW delivers 0
- bigblock_livepath_loopback (telecom_system.cc:8389-8408) compares the RAW decoded
  info bits bigblock_rx_infobits (STILL whitened) against bigblock_last_tx_cw_info
  (the WHITENED systematic info bits cw_info captured at TX, telecom_system.cc:8163/
  8174). Both sides whitened -> BER=0. The validator NEVER de-whitens and NEVER
  exercises the byte-level de-whiten -> it proves only that the PHY/FEC recovers the
  whitened codeword bits, NOT that the carve de-whiten reproduces the original bytes.
- So validator-pass does NOT vindicate the de-whiten alignment, and is consistent
  with a carve-side de-whiten bug delivering 0 app bytes.

## No SIM_INPROC special-casing in the bigblock RX/TX/whiten path
- SIM_INPROC hooks (arq_common.cc:105-226) ONLY gate TCP polls and step-pump the
  TX spin-waits (ptt_busy_wait/drain_playback_wait). They are transport/timing only.
- There is NO SIM_INPROC / sim2 branch in receive(), receive_byte, receive_bigblock,
  transmit_bigblock, bigblock_send_one_block, or bigblock_receive_carve.
- The carve comment itself names "the live 2-instance path (ref==NULL)"
  (arq_common.cc:3911) — the 2-instance sim and HW are the SAME "live path"
  (no per-instance TX cw_info ref, unlike the single-instance oracle loopback).

## Contrast: unit tests are NOT a pure synthetic bypass for whitening
- --test-bigblock-arq-unit CASE A (test_bigblock_arq_unit.cc:1246-1316):
  run_block_loopback (:1191) DOES run the REAL transmit_byte/transmit_bigblock
  (TX whiten) + REAL receive_byte/receive_bigblock (PHY decode), then calls the
  REAL bigblock_receive_carve (:1277, de-whiten). So CASE A DOES exercise the real
  TX whiten + real carve de-whiten — through a CLEAN, jitter-free, directly-wired
  loopback (rx_pb[lead+i]=tx_pb[i], :1218) rather than via ARQ send_batch ->
  tx_transfer -> audio rings -> capture-prep -> receive() consumer.
  CASE A PASSES byte-faithful => with one-whiten+one-de-whiten and a clean wire,
  the de-whiten IS self-consistent. (CASE A's only "synthetic" part is the cw_ok
  bit-flip injection in CASE B-D and the controlled local/fallback BSI; the
  whiten/de-whiten and PHY are real.)

## Implication for the fix agent
- SIM_INPROC at CFG16 with MERCURY_BIGBLOCK_FRAMING=1 (pin via MERCURY_SIM2_PIN=1
  MERCURY_SIM2_CFG=16, payload via MERCURY_SIM2_PAYLOAD_BYTES) routes through the
  real carve+de-whiten and IS a faithful off-bench reproducer of the live delivery.
- BUT: since the whiten is single, not double, the 0-byte cause is more likely
  (a) a TX-span / RX-span MISMATCH in the whitened region (TX whitens
      payload_bits_len = Kpack*ldpc.K; carve de-whitens nbits = K*sub_len*8 =
      K*ldpc.K — these match ONLY if Kpack==K AND sub_len==ldpc.K/8 AND the bit
      ORDER/packing matches), or
  (b) a bit-offset/packing convention mismatch between TX byte_to_bit + whiten
      and the carve's per-(c,b,bit) idx packing (arq_common.cc:3893-3905), or
  (c) the carve reading a different K/sub_len than TX used.
  The "TX-span vs full K*ldpc.K" mismatch the measure agent flagged is the live
  suspect — NOT a double de-whiten. The CLEAN-wire CASE A passing while the
  jittered live ARQ path fails also keeps an acquisition/over-advance interaction
  in scope, but the carve gate fires identically in SIM_INPROC, so SIM_INPROC
  should reproduce whatever the carve-level mismatch is.
