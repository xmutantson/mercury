# L2 — LIVE RX carve de-whiten span/seed/offset (read-only analysis)

Ref: integ/bigblock-final-2026-06-05 worktree C:/Users/kamer/mercury_wt/bigblock-final

## RX carve de-whiten — EXACT parameters (arq_common.cc:3886-3905)
- function: `cl_arq_controller::bigblock_receive_carve` (arq_common.cc:3857)
- source data: `info_bits` = `telecom_system->bigblock_rx_infobits.data()` (passed at the
  call site arq_common.cc:6901). These are the RAW LDPC-decoded systematic info bits,
  STILL WHITENED (the RX PHY does not de-whiten — see below).
- span (nbits): `nbits = K * sub_len * 8` (arq_common.cc:3889)
    - `K = telecom_system->bigblock_last_rx_K` (= Kout from receive_bigblock) (:3861)
    - `sub_len = telecom_system->ldpc.K / 8` (:3863). At CFG16 ldpc.K=1400 -> sub_len=175
      -> sub_len*8=1400=ldpc.K exactly (1400 % 8 == 0). So nbits = K*ldpc.K.
- bits used: ALL of them, contiguous, index 0..nbits-1 (no offset). dw[i]=info_bits[i]&1 (:3890-3891)
- de-whiten call: `telecom_system->bigblock_whiten_bits(dw.data(), nbits)` (:3892)
    -> `bigblock_whiten_payload_bits(bits, nbits, BIGBLOCK_WHITEN_SEED)` (telecom_system.cc:8092-8096)
- SEED: `BIGBLOCK_WHITEN_SEED = 0x5A3C96E1u` (telecom_system.cc:6982)
- bit-offset/start: 0 (PRBS starts at the same warmup the TX uses; see PRBS below)
- PRBS (telecom_system.cc:6969-6978): LCG, self-inverse XOR
    s0 = seed*6364136223846793005 + 1442695040888963407   (one warmup mult before the loop)
    per bit: s = s*6364136223846793005 + 1442695040888963407; w = (s>>33)&1; bit ^= w
- packing after de-whiten (:3893-3905): byte[(c*sub_len)+b] from bits LSB-first
    idx=(c*sub_len+b)*8+bit; byte|=(1<<bit). MATCHES misc.cc:93 byte_to_bit (LSB-first).

## TX whiten — EXACT parameters (telecom_system.cc:8140-8164)
- producer: bigblock_block_to_arq (arq_common.cc:3717-3822) assembles the FULL byte image
  block_payload[K*sub_len] = [cw0: bsi,n_data,len-table,app,pad,CRC | cw1..: app,pad,CRC].
  Header written :3739-3752, app :3755-3766, per-codeword CRC-8 at codeword tail :3777-3785.
- hands block_payload (K*sub_len byte-valued ints) to transmit_byte->transmit_bigblock,
  nBytes=K*sub_len (arq_common.cc:3820).
- transmit_bigblock: byte_to_bit(data, payload, use_bytes) LSB-first (:8154), THEN
  bigblock_whiten_payload_bits(payload, payload_bits_len, BIGBLOCK_WHITEN_SEED) (:8161).
    - payload_bits_len = Kpack*ldpc.K, Kpack = nBits/ldpc.N (:8146-8148).
- whitened bits become the codeword info bits in bigblock_build_tx_bits (:7002-7004):
  cw_info[c] = (the WHITENED bits), ldpc.encode(cw_info[c]).

## VERDICT on the stated hypotheses
- "DOUBLE de-whiten": **REFUTED.** The live RX PHY de-whitens ZERO times:
    - bigblock_rx_passband writes raw decoded info bits straight to out_infobits with no
      whiten (telecom_system.cc:7445-7446).
    - receive_bigblock copies info_bits->out verbatim, no whiten (telecom_system.cc:8254-8263).
  The carve at arq_common.cc:3892 is the ONLY de-whiten on the live path, applied ONCE.
  Span/seed/bit-order/offset all MATCH the TX whiten (seed 0x5A3C96E1, offset 0, LSB-first,
  span K*ldpc.K == Kpack*ldpc.K when Kout==Kpack and ldpc.K%8==0, which holds at CFG16/1400).
  => the carve de-whiten is, IN ISOLATION, correct.

- "TX-span mismatch (n_data vs full K*ldpc.K)": TX whitens the FULL payload_bits_len
  (Kpack*ldpc.K, incl pad+CRC), arq_common.cc:8161 — NOT n_data. RX whitens K*sub_len*8.
  These are equal iff Kout==Kpack. This is the ONE remaining alignment dependency to verify
  on HW (does bigblock_last_rx_K == the TX Kpack for the live block?).

## WHY the standalone BIGBLOCK_LIVE validator passes but the live ARQ carve fails
- bigblock_livepath_loopback (telecom_system.cc:8393-8404) compares bigblock_rx_infobits
  (whitened) against bigblock_last_tx_cw_info (ALSO whitened — set from the post-whiten
  payload at telecom_system.cc:7003 via the payload passed at :8163). It NEVER calls
  de-whiten. It is a WHITENED-vs-WHITENED bit round-trip check. It can NOT exercise or
  catch a de-whiten error. So "validator passes 8/8" does NOT certify the de-whiten path —
  it only certifies the LDPC bit round-trip. The de-whiten is exercised ONLY by the ARQ
  carve, which the validator bypasses.

## Where the real misalignment must be (for the measure agent to confirm)
Given the carve de-whiten is byte/seed/offset-aligned with TX, the garbage (wire_bsi=159,
all-byte corruption) points to ONE of:
  (a) Kout (bigblock_last_rx_K) != Kpack (TX) on the live block -> PRBS phase/span shifts ->
      every byte XORed against the wrong PRBS bit. CHECK the [BIGBLOCK-RX] K= vs [BIGBLOCK-TX] K=.
  (b) the decoded info bits in bigblock_rx_infobits are NOT the TX info bits at all on the
      live 2-instance path (the validator's whitened-vs-whitened compare is single-instance
      with cw_info_ref!=NULL; the live path runs cw_info_ref==NULL so cw_ok is FORCED clean
      and never bit-compared) -> the codewords may be MISCORRECTED yet "clean", so de-whiten
      of wrong bits = garbage. This is consistent with NOCRC=1 giving 74 "CLEAN" blocks that
      still de-whiten to bsi=159.
Either way the de-whiten PRIMITIVE is correct; the input bits or their COUNT are wrong.
