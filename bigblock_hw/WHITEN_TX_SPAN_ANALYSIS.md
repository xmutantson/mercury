# Big-block TX whitening span/seed/offset — read-only analysis (integ/bigblock-final-2026-06-05 @c746064)

## TX whiten (transmit_bigblock, nBytes>0 real-payload branch)
- File: source/physical_layer/telecom_system.cc
- 8146-8147: `Kpack = nBits/ldpc.N`, capped by env `MERCURY_BIGBLOCK_K`.
- 8148: `payload_bits_len = Kpack * ldpc.K`  (CFG16: ldpc.K=1400, N=1600 → sub_len=175).
- 8154: `byte_to_bit(data, payload, use_bytes)` — LSB-first (misc.cc:93-105), packs block bytes
  contiguously: codeword c, info bit j → payload index `c*ldpc.K + j`.
- 8161: `bigblock_whiten_payload_bits(payload.data(), payload_bits_len, BIGBLOCK_WHITEN_SEED)`.
  - SPAN: full `payload_bits_len = Kpack*ldpc.K` bits (the WHOLE block, INCLUDING zero-pad).
  - SEED: `BIGBLOCK_WHITEN_SEED = 0x5A3C96E1` (telecom_system.cc:6982).
  - OFFSET: bit 0; contiguous whole-block stream (NOT per-codeword reseed).
  - PRBS: LCG, self-inverse XOR, bit i = `((s>>33)&1)` where s advances per bit (6969-6978).
  - ORDER: applied to systematic info bits BEFORE LDPC encode (encode is at 8163
    bigblock_tx_passband → bigblock_build_tx_bits:7004 ldpc.encode).
- Block bytes assembled by ARQ producer arq_common.cc:3699-3785 (cw0 header [bsi,n_data,len[]],
  app bytes, per-codeword CRC-8 tail) — NOT whitened there; whitening is the single 8161 call.

## RX de-whiten (live ARQ carve)
- arq_common.cc:3889-3892: `nbits = K*sub_len*8 = K*ldpc.K`;
  `bigblock_whiten_bits(dw, nbits)` → same helper, same seed 0x5A3C96E1, offset 0, AFTER decode.
- Decoded info bits land at out_infobits[c*ldpc.K + i] (telecom_system.cc:7446); K_out=Kcw (7455)
  = same nBits/ldpc.N(capped) as TX Kpack. sub_len=ldpc.K/8=175; sub_len*8==ldpc.K (1400 div by 8).

## Hypotheses
- DOUBLE de-whiten: REFUTED. Only 3 whiten call sites in telecom_system.cc (helper def 6969,
  wrapper def 8092, TX call 8161). receive_bigblock (8188-8280) and bigblock_rx_passband
  (7128-7460) call ZERO whiten. The only de-whiten on the live path is the single carve call
  arq_common.cc:3892. One whiten + one de-whiten = self-inverse, cancels.
- SPAN mismatch (n_data vs K*ldpc.K): REFUTED at CFG16. TX span = Kpack*ldpc.K; carve span =
  K*ldpc.K; ldpc.K=1400 divisible by 8 so K*sub_len*8 == K*ldpc.K exactly; same per-cw index
  map c*ldpc.K+i; Kpack==K_out (both nBits/ldpc.N capped by same MERCURY_BIGBLOCK_K env).
- Standalone BIGBLOCK_LIVE validator passes 8/8 NOT because its de-whiten aligns, but because it
  exercises the nBytes==0 seeded-PRBS TX branch (8168, NO whitening) and compares decoded bits
  against bigblock_last_tx_cw_info (the un-whitened encoded info), bypassing whiten entirely
  (validator compare loop arq... telecom_system.cc:8396-8400). Whitening is NEVER tested there.

## Implication
Whitening TX/RX are symmetric → whitening is NOT the live-path corruption asymmetry. The
NOCRC=1 `wire_bsi=159 / bsi_next=-1` garbage must originate where TX and RX K (or the decoded
bits themselves) actually diverge on the 2-instance path — i.e. the LDPC decode is producing
wrong info bits (de-whiten faithfully un-scrambles garbage → garbage), OR Kpack(TX)!=K_out(RX),
NOT a whiten seed/offset/span/double-XOR error.
