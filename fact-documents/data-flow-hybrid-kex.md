# data-flow-hybrid-kex.md — Hybrid X25519 + ML-KEM-768 key exchange

Cross-layer fact + data-flow audit for the hybrid post-quantum handshake wired
on `staging/mlkem` (off `monitor` 913ec8b5). Pairs with the regression suite
`source/crypto/test_mlkem_hybrid.cc` (`mercury.exe --test` /
`--test-mlkem-hybrid`). Design source: `_research/auto/MLKEM_HYBRID_PLAN.md`.

Scope: this commit is BUILD + UNIT-TEST. The combiner (§1) and the KX chunk
codec (§2) are exercised end-to-end by the unit suite. The live multi-frame
turn-taking choreography over real audio (§3) is wired structurally and compiles
clean; faithful real-audio integration verify is the explicit later step.

--------------------------------------------------------------------------------
## §1 The combiner (mercury_crypto.cc derive_session_key)
--------------------------------------------------------------------------------

Two load-bearing correctness points, both cited to X25519MLKEM768
(draft-ietf-tls-ecdhe-mlkem-05) + X-Wing (Barbosa et al.):

1. **IKM order = ML-KEM FIRST.** `ikm = mlkem_shared(32) || x25519_shared(32)`
   (mercury_crypto.cc derive_session_key, hybrid branch). §4.3 / SP 800-56Cr2:
   the first shared secret in an HKDF-over-two-secrets combiner must come from a
   FIPS-approved scheme; ML-KEM (FIPS 203) is that scheme in this pairing.
   FIX: the pre-existing code built `x25519 || mlkem` (BACKWARDS) — swapped.
   Classical fallback (`mlkem_done=false`) keeps `ikm = x25519_shared` — BYTE
   IDENTICAL to the pre-hybrid build.

2. **Transcript binding.** The naive concat combiner is NOT generically IND-CCA
   unless the KEM ciphertext + public keys are absorbed (draft §6: the analysis
   "relies crucially on the TLS 1.3 message transcript"; Mercury has NONE). We
   fold a 32-byte digest into the HKDF salt on the hybrid path only:
     `bind = Blake2b("mercury-hybrid-v1" || mlkem_ct(1088) || mlkem_pk(1184)
                     || x25519_pk_cmd(32) || x25519_pk_rsp(32))`
   X-Wing template (`ss||ss||ct||pk||epk`) adapted to HKDF-Blake2b: SS stay raw
   in IKM, only the PUBLIC material is pre-digested (keeps salt within
   `salt[256]` / Blake2b's key slot). Both peers absorb the SAME order
   (`ct||pk||pk_cmd||pk_rsp`); a tamper/bit-error that slips past CRC yields
   divergent session keys -> KEY_ACTIVATE confirm-tag mismatch -> clean
   disconnect. The confirm tag is the protocol-specific substitute for the TLS
   Finished MAC.

Producers of session_key: `derive_session_key` (classical at KX1; hybrid at
KX3-complete). Consumers: `compute_key_confirmation` (confirm tag),
`encrypt`/`decrypt` (AEAD). The IKM-order swap + bind only alter the HYBRID
branch — every classical consumer is unchanged.

Peer-pubkey caching: `compute_x25519_shared` now stores `x25519_peer_pk`; the
handshake assembles `bind` via `get_x25519_pubkey()` (own) +
`get_x25519_peer_pubkey()` (peer), ordered commander-then-responder by role.

--------------------------------------------------------------------------------
## §2 KX chunk transport (mercury_crypto.cc kx_chunk_*; arq_common.cc kx_*)
--------------------------------------------------------------------------------

The 1184B encaps key (KX2) + 1088B ciphertext (KX3) exceed a single frame
(`max_data_length < 256`, set_max_buffer_length guard), so they are chunked.

Wire (one chunk per KEY_EXCHANGE_2/3 control frame, MLKEM_HYBRID_PLAN.md §4):
  byte0 kind(0x3F/0x40) | byte1 idx | byte2 count | byte3 CRC8(POLY 0xF4 over 0..2)
  | bytes4.. chunk payload (<= kx_chunk_payload_capacity())

`kx_chunk_payload_capacity()` mirrors the X25519 KX read geometry
(`max_data_length + max_header_length - CONTROL_ACK_CONTROL_HEADER_LENGTH`) minus
the 4-byte header, clamped to `[1, N_MAX/8 - 4]` so a chunk can never overrun
`messages_control.data` (200B).

Reassembly state (arq.h, per-controller; reset by `kx_chunk_state_reset()` on
session reset + at phase start): `kx_mlkem_pk/ct`, `kx_rx_got[256]` bitmap,
`kx_rx_count/received/total`, reassembled into `kx_data_buf` (lazily alloc'd
MLKEM_PK_SIZE). Codec invariants (kx_chunk_decode): kind match, header CRC8,
index<count, non-final chunks carry FULL capacity (so total length is
unambiguous), and `offset+payload <= reasm_cap` (no overflow). A rejected chunk
returns -1 -> ARQ resends (a KX chunk is a frame carrying a reserved first byte;
loss/CRC-fail are handled by the EXISTING control ACK/retransmit, no new
reliability code).

--------------------------------------------------------------------------------
## §3 Handshake state machine (STRICT / SNDL-safe; -E/-K path only)
--------------------------------------------------------------------------------

Gated on `encryption_mode == ENCRYPT_STRICT`; the classical (`ENCRYPT_FAST`/
legacy) and non-encrypted paths are UNTOUCHED (the new branches key off
ENCRYPT_STRICT + the KEY_EXCHANGE_2/3 codes, which never fire otherwise).

  CMD                                              RSP
  KX1 X25519 (control) ----------------------->  gen X25519, derive classical,
                              <----------------  KX1 ACK (pub + classical confirm)
  X25519 confirm OK -> generate_mlkem_keypair()
  KX2 encaps-key chunks (control req) -------->  reassemble -> encapsulate_mlkem()
       [CMD set_kx_phase MLKEM_PK_SENT]            -> ct + ML-KEM ss; derive HYBRID
                                                   (mlkem_done=true, bind);
                              <----------------  KX3 ciphertext chunks (control ACK)
  reassemble ct; decapsulate_mlkem();              [RSP set_kx_phase HYBRID_DONE]
  derive HYBRID (mlkem_done=true, bind)
       [CMD set_kx_phase HYBRID_DONE]
  KEY_ACTIVATE (hybrid confirm tag) ---------->  confirm == own hybrid tag
                              <----------------  KEY_ACTIVATE (hybrid confirm)
  activate(); pq_active=true [KX_ACTIVE]          activate(); pq_active=true
  ---- subsequent data batches AEAD under the HYBRID key ----

Transport asymmetry (the reason §4 of the plan preferred the data plane): CMD
streams KX2 via the control-REQUEST path (`add_message_control` builder cases
KEY_EXCHANGE_2/3 encode kx_tx_* -> messages_control); RSP streams KX3 via the
control-ACK REPLY path (the same LDPC ACK on `data_configuration` that returns
the KX1 pubkey; arq_responder.cc ACK builder now also sends KEY_EXCHANGE_3).
`kx_send_next_chunk()` is role-aware. mlkem_done flips true at both derives;
pq_active follows; a SECOND KEY_ACTIVATE confirms the hybrid key.

Failure handling (MLKEM_HYBRID_PLAN.md §6): encaps-key check is internal to
crypto_kem_enc (rc!=0 -> DROP); FIPS-203 implicit rejection means a bad ct
decaps to a pseudorandom ss -> the mismatch surfaces at the KEY_ACTIVATE confirm
tag -> clean disconnect (the designed integrity gate). KX chunk loss/CRC-fail ->
existing ARQ resend. Generate/encaps/decaps failures -> DROPPED + reset.

--------------------------------------------------------------------------------
## §4 Tests (source/crypto/test_mlkem_hybrid.cc; --test wired main.cc)
--------------------------------------------------------------------------------

17 cases, all PASS (full `--test` exits 0):
  A: hybrid CMD/RSP derive IDENTICAL session key; pq_active both ends.
  B: hybrid key DIFFERS from classical-only key (ML-KEM ss genuinely entered).
  C: flipping one byte of mlkem_ct / mlkem_pk / x25519_pk_cmd / x25519_pk_rsp
     each DIVERGES the keys (proves the bind covers every transcript element).
  D: KX2(1184B)/KX3(1088B) chunk round-trip at caps {196,100,49,53,7}; CRC8
     rejects a corrupted header; wrong kind rejected; reasm overflow rejected.

Fail-before: pre-fix there was no hybrid path (A/B), no transcript bind (C would
have produced identical keys despite flipped public material), no chunk codec
(D). All four properties are now enforced.
