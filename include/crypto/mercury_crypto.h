/*
 * Mercury post-quantum hybrid encryption suite.
 *
 * Hybrid key exchange: X25519 (classical EC-DH) + ML-KEM-768 (post-quantum KEM).
 * Symmetric encryption: ChaCha20-Poly1305 (IETF, 256-bit key, 96-bit nonce).
 * Key derivation: HKDF-Blake2b (monocypher).
 *
 * An attacker must break BOTH X25519 AND ML-KEM to recover the session key.
 * This defends against passive "store now, decrypt later" (SNDL) attacks
 * by future quantum computers, while maintaining classical security via X25519.
 *
 * See ENCRYPTION_PLAN.md for full design rationale and security invariants.
 */

#ifndef MERCURY_CRYPTO_H
#define MERCURY_CRYPTO_H

#include <cstdint>
#include <cstddef>

// Key material sizes (ML-KEM-768)
#define MLKEM_PK_SIZE    1184   // ML-KEM-768 encapsulation key (public)
#define MLKEM_SK_SIZE    2400   // ML-KEM-768 decapsulation key (secret)
#define MLKEM_CT_SIZE    1088   // ML-KEM-768 ciphertext
#define MLKEM_SS_SIZE    32     // ML-KEM-768 shared secret

#define X25519_KEY_SIZE  32     // X25519 public/secret key
#define SESSION_KEY_SIZE 32     // ChaCha20-Poly1305 key
#define AUTH_TAG_SIZE    16     // Poly1305 authentication tag
#define AUTH_TAG_ROBUST  4      // Truncated tag for ROBUST modes
#define NONCE_SIZE       12     // IETF ChaCha20-Poly1305 nonce

// Direction tags for nonce derivation
#define DIRECTION_CMD_TO_RSP  0x00000000
#define DIRECTION_RSP_TO_CMD  0x00000001

// Encryption modes (CLI -E flag)
#define ENCRYPT_OFF       0    // No encryption
#define ENCRYPT_STRICT    1    // SNDL-safe: hold data until full PQ key exchange
#define ENCRYPT_FAST      2    // Classical-first: X25519 immediate, PQ upgrade later

// Encryption negotiation outcome — computed identically by the commander and the
// responder from (encryption_mode, local_capability, peer_capability) so the
// fail-closed policy has ONE source of truth (decide_encryption_negotiation()).
//
// FAIL-CLOSED (mandatory): once the operator has OPTED IN (-E, encryption_mode !=
// ENCRYPT_OFF), a peer that does not advertise CAP_ENCRYPTION — whether it truly
// lacks the capability OR a MITM stripped the cap bit pre-KX to force a plaintext
// downgrade — yields ENC_NEG_REFUSE for BOTH STRICT and FAST. Encryption must
// never opportunistically fall back to plaintext under an -E opt-in.
//
// DEFAULT-OFF (Part-97, non-negotiable): PLAINTEXT_OK is returned ONLY when the
// operator did NOT opt in (encryption_mode == ENCRYPT_OFF). Legal default-off
// plaintext operation is unchanged.
enum enc_negotiation_outcome_t {
    ENC_NEG_PLAINTEXT_OK = 0,  // encryption_mode == ENCRYPT_OFF: legal default-off plaintext, unchanged
    ENC_NEG_ENABLED      = 1,  // opted in AND peer advertises CAP_ENCRYPTION: encrypt
    ENC_NEG_REFUSE       = 2   // opted in but peer lacks CAP_ENCRYPTION (unsupported or MITM-stripped): fail-closed drop
};

// Key exchange phases
#define KX_IDLE           0    // No key exchange in progress
#define KX_X25519_SENT    1    // X25519 pubkey sent, awaiting peer's
#define KX_X25519_DONE    2    // X25519 shared secret computed
#define KX_MLKEM_PK_SENT  3    // ML-KEM encaps key sent (commander only)
#define KX_MLKEM_CT_SENT  4    // ML-KEM ciphertext sent (responder only)
#define KX_HYBRID_DONE    5    // Both shared secrets ready, session key derived
#define KX_ACTIVE         6    // Encryption active (KEY_ACTIVATE exchanged)

// Per-batch encryption overhead
// encrypt() adds auth tag; decrypt() removes it
// Caller must account for this in batch capacity calculations
// Full tag (16 bytes) for CONFIG_0+, truncated (4 bytes) for ROBUST

class cl_cipher_suite {
public:
    cl_cipher_suite();
    ~cl_cipher_suite();

    // --- Key Exchange ---

    // Phase 1: X25519 (classical Diffie-Hellman)
    // Generate ephemeral X25519 keypair, write 32-byte pubkey to out.
    // Returns 0 on success, -1 on RNG failure.
    int generate_x25519_keypair(uint8_t pubkey_out[X25519_KEY_SIZE]);

    // Compute X25519 shared secret from peer's public key.
    // Must call generate_x25519_keypair() first.
    // Returns 0 on success.
    int compute_x25519_shared(const uint8_t peer_pubkey[X25519_KEY_SIZE]);

    // Length-checked variant: rejects (returns -1) a NULL or short peer pubkey
    // BEFORE reading X25519_KEY_SIZE bytes, so a truncated/corrupted KEY_EXCHANGE
    // frame is rejected, not run over into stale/OOB tail bytes. Prefer this at
    // wire-facing call sites. See data-flow-aead-nonce.md §KX.
    int compute_x25519_shared_checked(const uint8_t* peer_pubkey,
                                      int peer_pubkey_len);

    // Phase 2: ML-KEM-768 (post-quantum KEM)
    // Generate ephemeral ML-KEM keypair, write 1184-byte encaps key to out.
    // Commander calls this. Returns 0 on success, -1 on RNG failure.
    int generate_mlkem_keypair(uint8_t encaps_key_out[MLKEM_PK_SIZE]);

    // Encapsulate: generate shared secret using peer's encaps key.
    // Writes 1088-byte ciphertext and 32-byte shared secret.
    // Responder calls this. Returns 0 on success.
    int encapsulate_mlkem(const uint8_t encaps_key[MLKEM_PK_SIZE],
                          uint8_t ciphertext_out[MLKEM_CT_SIZE]);

    // Decapsulate: recover shared secret from ciphertext.
    // Commander calls this (has the secret key from generate_mlkem_keypair).
    // Returns 0 on success.
    int decapsulate_mlkem(const uint8_t ciphertext[MLKEM_CT_SIZE]);

    // --- Key Derivation ---

    // Derive session key from X25519 + ML-KEM shared secrets via HKDF-Blake2b.
    // If psk is NULL, no PSK is mixed in (unauthenticated mode).
    // If mlkem_done is false, derives from X25519 only (classical-first mode).
    //
    // HYBRID COMBINER (X25519MLKEM768 / draft-ietf-tls-ecdhe-mlkem-05 §4.3 +
    // X-Wing, Barbosa et al.). Two load-bearing correctness points (see
    // MLKEM_HYBRID_PLAN.md §2-§3 / fact-documents/data-flow-hybrid-kex.md):
    //
    //  (1) SHARED-SECRET ORDER is ML-KEM FIRST: ikm = mlkem_ss || x25519_ss.
    //      Per SP 800-56Cr2 the first shared secret in an HKDF-over-two-secrets
    //      combiner must come from a FIPS-approved scheme; ML-KEM (FIPS 203) is
    //      that scheme in this pairing, so it leads. (The classical-only fallback
    //      keeps ikm = x25519_ss.)
    //
    //  (2) TRANSCRIPT BINDING. The naive concat combiner is NOT generically
    //      IND-CCA unless the KEM ciphertext + public keys are absorbed — TLS
    //      gets this from its transcript hash; draft §6 warns the analysis
    //      "relies crucially on the TLS 1.3 message transcript". Mercury has NO
    //      transcript, so we MUST bind explicitly: when mlkem material is passed
    //      (hybrid path) we fold a 32-byte digest
    //          bind = Blake2b("mercury-hybrid-v1" || mlkem_ct || mlkem_pk
    //                         || x25519_pk_cmd || x25519_pk_rsp)
    //      into the HKDF salt (X-Wing's ss||ss||ct||pk||epk template, adapted to
    //      HKDF-Blake2b). A tampered/erroneous ct or pk yields a different
    //      session key -> KEY_ACTIVATE confirm-tag mismatch -> clean disconnect.
    //
    // Transcript args (hybrid path only; pass NULL on the classical-only
    // fallback so the salt is byte-identical to the pre-hybrid build):
    //   mlkem_ct      : 1088-byte ML-KEM ciphertext (or NULL)
    //   mlkem_pk      : 1184-byte ML-KEM encaps key (or NULL)
    //   x25519_pk_cmd : commander's 32-byte X25519 pub (or NULL)
    //   x25519_pk_rsp : responder's 32-byte X25519 pub (or NULL)
    // BOTH peers MUST assemble bind over the SAME byte order (ct||pk||cmd||rsp);
    // the commander holds mlkem_pk (it generated the keypair) + learns mlkem_ct
    // from KX3, the responder holds mlkem_ct (it encapsulated) + learns mlkem_pk
    // from KX2, and both already hold both X25519 pubs from KX1.
    void derive_session_key(const char* commander_call,
                            const char* responder_call,
                            const uint8_t* psk, int psk_len,
                            bool mlkem_done,
                            const uint8_t* mlkem_ct = nullptr,
                            const uint8_t* mlkem_pk = nullptr,
                            const uint8_t* x25519_pk_cmd = nullptr,
                            const uint8_t* x25519_pk_rsp = nullptr);

    // Re-derive with both shared secrets after ML-KEM completes (PQ upgrade).
    // Call derive_session_key again with mlkem_done=true.

    // Read-only access to this suite's own X25519 public key (cached at
    // generate_x25519_keypair time). Needed by the handshake to assemble the
    // transcript-binding `bind` input in the correct commander/responder order.
    // Returns NULL until generate_x25519_keypair() has run.
    const uint8_t* get_x25519_pubkey() const { return x25519_ready ? x25519_pk : nullptr; }

    // Read-only access to the cached PEER X25519 public key (captured at
    // compute_x25519_shared time). Returns NULL until the shared secret is set.
    const uint8_t* get_x25519_peer_pubkey() const { return x25519_ready ? x25519_peer_pk : nullptr; }

    // --- Per-Batch Encrypt/Decrypt ---

    // Encrypt plaintext batch. Writes ciphertext + auth tag to out.
    // Returns total bytes written (in_len + tag_size), or -1 on error.
    // tag_size is AUTH_TAG_SIZE (16) for CONFIG, AUTH_TAG_ROBUST (4) for ROBUST.
    //
    // batch_index MUST be the UNWRAPPED wire batch index (epoch<<8 | wire_bsi),
    // NOT a local encrypt-call counter. Both peers derive the SAME nonce from
    // the SAME wire batch_seq_id, so the nonce survives SACK reorder/retx. The
    // index is strictly increasing per session+direction; the caller MUST
    // re-key / tear down before it can wrap into a reused (key,nonce) pair.
    // See NONCE_DESIGN.md / fact-documents/data-flow-aead-nonce.md.
    int encrypt(const uint8_t* in, int in_len,
                uint8_t* out, int out_capacity,
                uint64_t batch_index, uint32_t direction,
                int tag_size);

    // Decrypt ciphertext batch. Writes plaintext to out.
    // Returns plaintext bytes (in_len - tag_size), or -1 on auth failure.
    // batch_index: see encrypt() — the UNWRAPPED wire batch index for the
    // batch being delivered (reconstructed from the wire batch_seq_id, NOT a
    // local delivery-order counter).
    int decrypt(const uint8_t* in, int in_len,
                uint8_t* out, int out_capacity,
                uint64_t batch_index, uint32_t direction,
                int tag_size);

    // --- Nonce sequence unwrap (shared TX/RX math) ---
    // Reconstruct a strictly-monotone 64-bit batch index from the 8-bit wire
    // batch_seq_id stream using RFC-1982 forward-step arithmetic (a step in
    // [1,128] mod 256 is "forward"; crossing the 0xFF->0x00 boundary bumps the
    // epoch). Both peers observe the SAME committed wire-bsi order, so both
    // reconstruct the SAME index for a given batch. State (epoch,last_bsi) is
    // held per direction by the caller; pass last_bsi = -1 for the first batch.
    // Updates *epoch and *last_bsi in place; returns the unwrapped index.
    // wire_bsi in [0,255]. See NONCE_DESIGN.md §3.3.
    static uint64_t unwrap_batch_index(int wire_bsi,
                                       uint64_t* epoch, int* last_bsi);

    // Re-seal generation stride (data-flow-aead-nonce.md §11). The AEAD nonce
    // index is fold_gen_index(gen, unwrap_index) = gen*NONCE_GEN_STRIDE +
    // unwrap_index. NONCE_GEN_STRIDE (2^48) is larger than any reachable
    // unwrap_index within ONE generation: the per-session re-key floor caps a
    // session FAR below 2^32 batches (RFC 9001 §6.6), so the unwrap index
    // (epoch<<8 | bsi) stays < 2^40 << 2^48 — distinct generations occupy
    // DISJOINT, strictly-ordered index bands, so a higher gen always yields a
    // strictly-greater index than any index of a lower gen. Combined with the
    // TX seal high-water guard, this makes a re-seal land at a fresh nonce
    // EVEN AT THE SAME WIRE BSI (the recovery rolls the wire bsi back for RSP
    // delivery contiguity, so the gen — not the bsi — is what advances).
    static const uint64_t NONCE_GEN_STRIDE = (uint64_t)1 << 48;
    static uint64_t fold_gen_index(uint64_t gen, uint64_t unwrap_index)
    {
        return gen * NONCE_GEN_STRIDE + unwrap_index;
    }

    // --- KX chunk transport (KEY_EXCHANGE_2 / KEY_EXCHANGE_3) ---
    //
    // The 1184-byte ML-KEM encaps key (KX2) and 1088-byte ciphertext (KX3) are
    // too large for a single Mercury frame (max_data_length < 256, arq_common.cc
    // set_max_buffer_length guard), so they are CHUNKED. Each chunk carries a
    // 4-byte header so the receiver can reassemble before any user data flows
    // (MLKEM_HYBRID_PLAN.md §4):
    //   byte 0 : kind        (KEY_EXCHANGE_2 0x3F / KEY_EXCHANGE_3 0x40)
    //   byte 1 : chunk_index (0..count-1)
    //   byte 2 : chunk_count (n)
    //   byte 3 : CRC8(POLY_CRC8=0xF4) over bytes 0..2 (matches CRC8_calc())
    //   bytes 4.. : chunk payload (<= chunk_payload_capacity bytes of key/ct)
    // Loss of a chunk is covered by the EXISTING ARQ retransmit (a KX chunk is a
    // frame carrying a reserved first byte); CRC8 fail -> drop -> ARQ resend.
    static const int KX_CHUNK_HEADER_LEN = 4;

    // The wire `kind` byte must equal these so a stale frame can't be mistaken
    // for a KX chunk; they mirror datalink_defines.h KEY_EXCHANGE_2/3.
    static const uint8_t KX_KIND_PK = 0x3F;   // KEY_EXCHANGE_2 (encaps key)
    static const uint8_t KX_KIND_CT = 0x40;   // KEY_EXCHANGE_3 (ciphertext)

    // Number of chunks needed to carry `total` payload bytes given a per-frame
    // chunk payload capacity. capacity must be >= 1.
    static int kx_chunk_count(int total_len, int chunk_payload_capacity)
    {
        if (chunk_payload_capacity < 1) return -1;
        if (total_len <= 0) return 0;
        return (total_len + chunk_payload_capacity - 1) / chunk_payload_capacity;
    }

    // Encode chunk `index` of `src` (length src_len) into `out` (capacity
    // out_cap). Writes the 4-byte header + this chunk's payload slice. Returns
    // total bytes written (header + payload), or -1 on bad args / overflow.
    static int kx_chunk_encode(uint8_t kind, const uint8_t* src, int src_len,
                               int index, int chunk_payload_capacity,
                               uint8_t* out, int out_cap);

    // Validate + decode a received chunk. On success writes the chunk payload
    // into reasm[offset..] (offset = index*chunk_payload_capacity) and returns
    // the payload byte count (>=0); sets *out_index/*out_count from the header.
    // Returns -1 on CRC8 mismatch, kind mismatch, header/length inconsistency,
    // or a write that would exceed reasm_cap. Does NOT assume contiguous arrival
    // — the caller tracks which indices have arrived via a bitmap.
    static int kx_chunk_decode(uint8_t expect_kind,
                               const uint8_t* in, int in_len,
                               int chunk_payload_capacity,
                               uint8_t* reasm, int reasm_cap,
                               int* out_index, int* out_count);

    // CRC8 over bytes 0..2 of a KX chunk header (POLY_CRC8=0xF4, init 0xFF,
    // reflected — identical to cl_arq_controller::CRC8_calc).
    static uint8_t kx_crc8(const uint8_t* data, int n);

    // --- State Queries ---
    bool is_active() const { return encryption_active; }
    bool is_pq_upgraded() const { return pq_active; }
    int  get_kx_phase() const { return kx_phase; }
    // Advance the KX phase from the handshake state machine (KX_* constants).
    // Pure bookkeeping for the hybrid ML-KEM upgrade — does NOT touch keys.
    void set_kx_phase(int phase) { kx_phase = phase; }
    int  get_tag_size(bool is_robust) const { return is_robust ? AUTH_TAG_ROBUST : AUTH_TAG_SIZE; }

    // Display-only: write a short hex session fingerprint (the same Blake2b-keyed
    // derivation logged in derive_session_key) into out for out-of-band voice
    // verification. Writes at most cap-1 chars + NUL. Does NOT mutate cipher
    // state and never exposes raw key bytes. Safe to call only after a session
    // key has been derived; no-op (out="") otherwise.
    void get_fingerprint_hex(char* out, int cap) const;

    // --- Key Confirmation ---
    // Compute 8-byte confirmation tag from session key.
    // Both sides compute this; mismatch = PSK wrong.
    void compute_key_confirmation(uint8_t tag_out[8]);

    // TEST-ONLY: copy the 32-byte derived session key out for in-process unit
    // tests (test_mlkem_hybrid.cc IKM-order regression — it must compare the
    // production combiner output byte-for-byte against an independently
    // recomputed reference for BOTH the production mlkem||x25519 order AND the
    // legacy x25519||mlkem order, to catch a future order revert that the
    // black-box confirm tag alone cannot distinguish). The key never leaves the
    // process; this is the standard KAT seam and does NOT weaken the wire
    // posture (no key material is ever transmitted or logged in the clear).
    void copy_session_key_for_test(uint8_t out[SESSION_KEY_SIZE]) const;

    // TEST-ONLY: derive the session key using the LEGACY (pre-fix, WRONG) IKM
    // order x25519_ss || mlkem_ss — salt / transcript-bind otherwise IDENTICAL
    // to the production hybrid path. The IKM-order regression asserts that
    // production (mlkem-first) produces a DIFFERENT key than this legacy order
    // from the SAME two shared secrets, so a future revert of the order swap is
    // caught (the black-box confirm tag alone cannot distinguish order). Run a
    // hybrid exchange first so mlkem_shared/x25519_shared are populated; this
    // overwrites session_key with the legacy-order result.
    void derive_session_key_legacy_order_for_test(
            const char* commander_call, const char* responder_call,
            const uint8_t* psk, int psk_len,
            const uint8_t* mlkem_ct, const uint8_t* mlkem_pk,
            const uint8_t* x25519_pk_cmd, const uint8_t* x25519_pk_rsp);

    // --- Activation ---
    void activate();     // Set encryption_active = true after KEY_ACTIVATE ACK

    // --- Cleanup ---
    // Securely wipe all key material. Called on disconnect.
    void wipe();

private:
    // X25519 state
    uint8_t x25519_sk[X25519_KEY_SIZE];
    uint8_t x25519_pk[X25519_KEY_SIZE];
    uint8_t x25519_peer_pk[X25519_KEY_SIZE];  // cached peer pub (transcript bind)
    uint8_t x25519_shared[X25519_KEY_SIZE];
    bool x25519_ready;

    // ML-KEM state
    uint8_t* mlkem_sk;    // 2400 bytes (heap — too large for stack)
    uint8_t  mlkem_shared[MLKEM_SS_SIZE];
    bool mlkem_ready;

    // Session key
    uint8_t session_key[SESSION_KEY_SIZE];
    bool encryption_active;
    bool pq_active;
    int  kx_phase;
};

#endif
