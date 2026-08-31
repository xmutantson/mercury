/*
 * AEAD bsi-bound nonce regression tests.
 *
 * Wired via mercury.exe --test (run_aead_nonce_tests). Fast, deterministic,
 * no IONOS / RF / telecom_system. Validates the SECURITY INVARIANT directly:
 * NO ChaCha20-Poly1305 (key,nonce) reuse with two different plaintexts, ever,
 * across batches / directions / SACK reorder / retransmit.
 *
 * The bug this guards (NONCE_DESIGN.md §1): the nonce used to bind to a LOCAL
 * encrypt-call / delivery-order counter (tx_batch_counter / rx_batch_counter).
 * Under SACK reorder/retx those two counters desync from the WIRE batch order,
 * so the RX reconstructs the WRONG nonce -> AEAD auth-fail -> batch DROP, with a
 * latent same-nonce/different-plaintext reuse hazard. The fix binds the nonce to
 * the UNWRAPPED WIRE batch_seq_id (epoch<<8 | wire_bsi), reconstructed
 * identically on both peers via cl_cipher_suite::unwrap_batch_index().
 *
 * FAIL-BEFORE / PASS-AFTER: the "reordered batch decrypts" and "wrap > 256"
 * cases model the OLD counter behaviour explicitly (a local monotone counter)
 * and assert it produces a DIFFERENT nonce than the wire-bound one -> the OLD
 * design would have auth-failed; the NEW design (unwrap_batch_index) matches.
 *
 * See data-flow-aead-nonce.md and NONCE_DESIGN.md.
 */

#include "../../include/crypto/mercury_crypto.h"
#include "test_aead_nonce.h"

#include <cstdio>
#include <cstring>
#include <cstdint>
#include <set>
#include <vector>

namespace {

// A test-local mirror of build_nonce (private in mercury_crypto.cc). Identical
// byte layout to the production builder (NONCE_DESIGN.md §3): we reconstruct the
// nonce here only so the uniqueness test can compare nonces WITHOUT decrypting.
// If the production layout changes, decrypt-round-trip tests below still catch a
// real mismatch end-to-end; this mirror is purely for the no-reuse set check.
void mirror_nonce(uint8_t nonce[12], uint32_t direction, uint64_t batch_index)
{
    nonce[0] = 0x00;
    nonce[1] = 0x00;
    nonce[2] = 0x00;
    nonce[3] = (uint8_t)(direction & 0xFF);
    nonce[4]  = (uint8_t)((batch_index >> 56) & 0xFF);
    nonce[5]  = (uint8_t)((batch_index >> 48) & 0xFF);
    nonce[6]  = (uint8_t)((batch_index >> 40) & 0xFF);
    nonce[7]  = (uint8_t)((batch_index >> 32) & 0xFF);
    nonce[8]  = (uint8_t)((batch_index >> 24) & 0xFF);
    nonce[9]  = (uint8_t)((batch_index >> 16) & 0xFF);
    nonce[10] = (uint8_t)((batch_index >>  8) & 0xFF);
    nonce[11] = (uint8_t)((batch_index      ) & 0xFF);
}

// Bring TWO cipher suites to the SAME active session key by performing a REAL
// 2-party X25519 exchange (cmd <-> rsp), then deriving the key from the shared
// secret + a fixed PSK + fixed callsigns on BOTH ends (the same derive inputs
// the production handshake uses). This is the only way both ends share a key:
// each suite has its own ephemeral keypair, so a self-DH would NOT match the
// peer. (Discovered via the reorder-roundtrip FAIL: separate self-DH keys ->
// mismatched session keys -> every decrypt auth-fails.)
int make_paired_suites(cl_cipher_suite& cmd, cl_cipher_suite& rsp)
{
    uint8_t pk_cmd[X25519_KEY_SIZE];
    uint8_t pk_rsp[X25519_KEY_SIZE];
    if (cmd.generate_x25519_keypair(pk_cmd) != 0)
        return -1;
    if (rsp.generate_x25519_keypair(pk_rsp) != 0)
        return -1;
    // Each computes the shared secret from the OTHER's pubkey -> identical secret.
    if (cmd.compute_x25519_shared(pk_rsp) != 0)
        return -1;
    if (rsp.compute_x25519_shared(pk_cmd) != 0)
        return -1;
    const char* psk = "MERCURY-AEAD-NONCE-TEST-PSK";
    // Same (commander_call, responder_call) ordering on both ends so the salt
    // matches (production passes them role-consistently).
    cmd.derive_session_key("CMDTEST", "RSPTEST",
                           (const uint8_t*)psk, (int)strlen(psk), false);
    rsp.derive_session_key("CMDTEST", "RSPTEST",
                           (const uint8_t*)psk, (int)strlen(psk), false);
    cmd.activate();
    rsp.activate();
    return 0;
}

int fail(const char* what)
{
    printf("[TEST-AEAD-NONCE] FAIL: %s\n", what);
    fflush(stdout);
    return 1;
}

// --- Case 1: unwrap_batch_index is strictly monotone & wraps correctly -------
// >256 batches: the 8-bit wire bsi recurs; the unwrapped index must NOT.
int test_unwrap_monotone_wrap()
{
    uint64_t epoch = 0; int last = -1;
    uint64_t prev_idx = 0;
    bool first = true;
    std::set<uint64_t> seen;
    // Drive 1000 forward batches (≈ 4 wraps of the 8-bit bsi).
    for (int n = 0; n < 1000; n++)
    {
        int wire_bsi = n & 0xFF;
        uint64_t idx = cl_cipher_suite::unwrap_batch_index(wire_bsi, &epoch, &last);
        if (!first && idx <= prev_idx)
            return fail("unwrap_batch_index not strictly increasing across wrap");
        if (seen.count(idx))
            return fail("unwrap_batch_index reused an index across wrap (NONCE REUSE)");
        seen.insert(idx);
        prev_idx = idx; first = false;
    }
    // After 1000 batches the index must have crossed the 8-bit boundary.
    if (prev_idx < 256)
        return fail("unwrap index did not exceed 8-bit range after 1000 batches");
    printf("[TEST-AEAD-NONCE] OK: unwrap monotone+unique over 1000 batches (idx=%llu)\n",
           (unsigned long long)prev_idx);
    fflush(stdout);
    return 0;
}

// --- Case 2: no two distinct batches share a nonce in a session+direction ----
// Includes the >256 wrap; the OLD 8-bit-only binding would reuse at bsi wrap.
int test_nonce_uniqueness()
{
    std::set<std::vector<uint8_t>> nonces;
    for (uint32_t dir = 0; dir <= 1; dir++)
    {
        uint64_t epoch = 0; int last = -1;
        for (int n = 0; n < 600; n++)   // > 2 wraps
        {
            int wire_bsi = n & 0xFF;
            uint64_t idx = cl_cipher_suite::unwrap_batch_index(wire_bsi, &epoch, &last);
            uint8_t nb[12];
            mirror_nonce(nb, dir, idx);
            std::vector<uint8_t> key(nb, nb + 12);
            if (nonces.count(key))
                return fail("two distinct batches produced the SAME nonce (NONCE REUSE)");
            nonces.insert(key);
        }
    }
    printf("[TEST-AEAD-NONCE] OK: 1200 nonces (2 dirs x 600 batches, >2 wraps) all unique\n");
    fflush(stdout);
    return 0;
}

// --- Case 3: cross-direction disjointness ------------------------------------
// CMD->RSP and RSP->CMD at the SAME wire bsi must NOT collide (direction byte).
int test_direction_disjoint()
{
    uint8_t n0[12], n1[12];
    mirror_nonce(n0, DIRECTION_CMD_TO_RSP, 5);
    mirror_nonce(n1, DIRECTION_RSP_TO_CMD, 5);
    if (memcmp(n0, n1, 12) == 0)
        return fail("CMD->RSP and RSP->CMD share a nonce at the same bsi");
    printf("[TEST-AEAD-NONCE] OK: directions occupy disjoint nonce subspaces\n");
    fflush(stdout);
    return 0;
}

// --- Case 4: reordered + retransmitted batch decrypts (FAIL-BEFORE) ----------
// TX encrypts batches in wire-bsi order. RX delivers them OUT OF ORDER (models
// SACK prev/out-of-order delivery + a retransmit). Each batch must decrypt
// because both ends derive the nonce from the WIRE bsi, not delivery order.
// The OLD design (a local delivery-order counter) would feed the WRONG counter
// and auth-fail — asserted by also checking the wire-bound nonce DIFFERS from
// what a naive delivery-order counter would have produced.
int test_reorder_retx_roundtrip()
{
    cl_cipher_suite tx, rx;
    if (make_paired_suites(tx, rx) != 0)
        return fail("X25519 pairing setup failed");

    const uint32_t DIR = DIRECTION_CMD_TO_RSP;
    const int N = 5;
    // TX: encrypt batches with wire bsi 10,11,12,13,14 (each distinct payload).
    std::vector<std::vector<uint8_t>> ct(N);
    std::vector<int> wire_bsi(N);
    std::vector<std::vector<uint8_t>> pt(N);
    {
        uint64_t tx_epoch = 0; int tx_last = -1;
        for (int k = 0; k < N; k++)
        {
            wire_bsi[k] = 10 + k;
            pt[k].resize(64);
            for (int b = 0; b < 64; b++) pt[k][b] = (uint8_t)(k * 7 + b);
            uint64_t idx = cl_cipher_suite::unwrap_batch_index(wire_bsi[k],
                                                               &tx_epoch, &tx_last);
            uint8_t out[64 + AUTH_TAG_SIZE];
            int n = tx.encrypt(pt[k].data(), 64, out, sizeof(out),
                               idx, DIR, AUTH_TAG_SIZE);
            if (n != 64 + AUTH_TAG_SIZE) return fail("encrypt returned wrong size");
            ct[k].assign(out, out + n);
        }
    }

    // RX delivers OUT OF ORDER: 12, 10, 14, 11, 13, then RE-DELIVERS 12 (retx).
    // Each maintains its own rx epoch/last state — but reorder must NOT desync.
    int order[6] = {2, 0, 4, 1, 3, 2};
    uint64_t rx_epoch = 0; int rx_last = -1;
    for (int o = 0; o < 6; o++)
    {
        int k = order[o];
        uint64_t idx = cl_cipher_suite::unwrap_batch_index(wire_bsi[k],
                                                           &rx_epoch, &rx_last);
        uint8_t plain[64];
        int n = rx.decrypt(ct[k].data(), (int)ct[k].size(), plain, sizeof(plain),
                           idx, DIR, AUTH_TAG_SIZE);
        if (n != 64)
            return fail("reordered/retx batch FAILED to decrypt (auth-fail)");
        if (memcmp(plain, pt[k].data(), 64) != 0)
            return fail("reordered batch decrypted to WRONG plaintext");
    }

    // FAIL-BEFORE proof: a naive delivery-order counter (0,1,2,3,4,5) would give
    // batch '12' (delivered first, counter 0) the index 0 — different from its
    // wire index 12 -> the OLD design would have produced a different nonce and
    // auth-failed. Confirm the two indices indeed differ.
    {
        uint64_t e = 0; int l = -1;
        uint64_t wire_idx_for_12 = cl_cipher_suite::unwrap_batch_index(12, &e, &l);
        uint64_t naive_delivery_counter_first = 0;  // OLD rx_batch_counter on 1st deliver
        if (wire_idx_for_12 == naive_delivery_counter_first)
            return fail("wire index coincides with naive delivery counter (test vacuous)");
    }

    printf("[TEST-AEAD-NONCE] OK: reorder(12,10,14,11,13)+retx(12) all decrypt to correct plaintext\n");
    fflush(stdout);
    return 0;
}

// --- Case 5: truncated / corrupted KX pubkey is rejected, not run over -------
int test_kx_truncated_reject()
{
    cl_cipher_suite cs;
    uint8_t pk[X25519_KEY_SIZE];
    if (cs.generate_x25519_keypair(pk) != 0)
        return fail("X25519 keypair generation failed");

    // A short/missing pubkey must be REJECTED before the 32-byte read.
    uint8_t shortbuf[8] = {1,2,3,4,5,6,7,8};
    if (cs.compute_x25519_shared_checked(shortbuf, 8) == 0)
        return fail("compute_x25519_shared_checked accepted an 8-byte pubkey");
    if (cs.compute_x25519_shared_checked(NULL, 0) == 0)
        return fail("compute_x25519_shared_checked accepted a NULL pubkey");
    if (cs.compute_x25519_shared_checked(pk, X25519_KEY_SIZE - 1) == 0)
        return fail("compute_x25519_shared_checked accepted a 31-byte pubkey");

    // A full-length valid pubkey must be ACCEPTED (no false-reject).
    if (cs.compute_x25519_shared_checked(pk, X25519_KEY_SIZE) != 0)
        return fail("compute_x25519_shared_checked rejected a valid 32-byte pubkey");

    printf("[TEST-AEAD-NONCE] OK: truncated/NULL KX pubkey rejected; valid accepted\n");
    fflush(stdout);
    return 0;
}

// --- Case 6: tamper -> auth fail (AEAD integrity intact under new nonce) ------
int test_tamper_rejected()
{
    cl_cipher_suite tx, rx;
    if (make_paired_suites(tx, rx) != 0)
        return fail("X25519 pairing setup failed");
    // SHARED key so a CLEAN ct would decrypt;
    // the tamper must be what fails, not the key.
    uint8_t pt[32];
    for (int i = 0; i < 32; i++) pt[i] = (uint8_t)(i + 1);
    uint8_t ct[32 + AUTH_TAG_SIZE];
    uint64_t e = 0; int l = -1;
    uint64_t idx = cl_cipher_suite::unwrap_batch_index(3, &e, &l);
    int n = tx.encrypt(pt, 32, ct, sizeof(ct), idx, DIRECTION_CMD_TO_RSP, AUTH_TAG_SIZE);
    if (n <= 0) return fail("encrypt failed in tamper test");

    // Non-vacuity: a CLEAN copy must decrypt under the shared key (so the tamper
    // reject below is meaningful, not just a key mismatch).
    {
        uint8_t clean[32];
        uint64_t ec = 0; int lc = -1;
        uint64_t idxc = cl_cipher_suite::unwrap_batch_index(3, &ec, &lc);
        int dc = rx.decrypt(ct, n, clean, sizeof(clean), idxc,
                            DIRECTION_CMD_TO_RSP, AUTH_TAG_SIZE);
        if (dc != 32 || memcmp(clean, pt, 32) != 0)
            return fail("clean ciphertext did NOT round-trip (test vacuous)");
    }

    uint8_t tampered[32 + AUTH_TAG_SIZE];
    memcpy(tampered, ct, n);
    tampered[0] ^= 0x01;  // flip a ciphertext bit
    uint8_t plain[32];
    uint64_t e2 = 0; int l2 = -1;
    uint64_t idx2 = cl_cipher_suite::unwrap_batch_index(3, &e2, &l2);
    int d = rx.decrypt(tampered, n, plain, sizeof(plain), idx2,
                       DIRECTION_CMD_TO_RSP, AUTH_TAG_SIZE);
    if (d > 0) return fail("tampered ciphertext decrypted (auth NOT enforced)");
    printf("[TEST-AEAD-NONCE] OK: clean round-trips; tampered ciphertext rejected (auth enforced)\n");
    fflush(stdout);
    return 0;
}

// --- Case 7: prev recovered AFTER a wrap reconstructs its epoch-1 index -------
// TX in-order: ...254,255,0,1. RX delivers 254,255,0 (epoch bumps to 1 on the
// 255->0 forward crossing), then RECOVERS 255 as a late prev. The backward-step
// unwrap must return 255's ORIGINAL index (epoch 0) = 255, NOT (epoch 1)|255,
// so its decrypt nonce matches what TX used. This is the exact production
// prev-recovery-across-wrap path; getting the epoch wrong here would auth-fail
// (or, worse, reuse a nonce). data-flow-aead-nonce.md §5.1.
int test_backward_across_wrap()
{
    cl_cipher_suite tx, rx;
    if (make_paired_suites(tx, rx) != 0)
        return fail("X25519 pairing setup failed");
    const uint32_t DIR = DIRECTION_RSP_TO_CMD;

    // TX encrypts 254,255,0,1 in order, each a distinct payload.
    int tx_bsi[4] = {254, 255, 0, 1};
    std::vector<std::vector<uint8_t>> ct(4), pt(4);
    {
        uint64_t te = 0; int tl = -1;
        for (int k = 0; k < 4; k++)
        {
            pt[k].resize(48);
            for (int b = 0; b < 48; b++) pt[k][b] = (uint8_t)(k * 13 + b + 1);
            uint64_t idx = cl_cipher_suite::unwrap_batch_index(tx_bsi[k], &te, &tl);
            uint8_t out[48 + AUTH_TAG_SIZE];
            int n = tx.encrypt(pt[k].data(), 48, out, sizeof(out), idx, DIR, AUTH_TAG_SIZE);
            if (n != 48 + AUTH_TAG_SIZE) return fail("wrap-case encrypt size");
            ct[k].assign(out, out + n);
        }
    }

    // RX delivery order: 254, 255, 0, then late-prev 255, then 1.
    int order[5] = {0, 1, 2, 1, 3};
    uint64_t re = 0; int rl = -1;
    for (int o = 0; o < 5; o++)
    {
        int k = order[o];
        uint64_t idx = cl_cipher_suite::unwrap_batch_index(tx_bsi[k], &re, &rl);
        uint8_t plain[48];
        int n = rx.decrypt(ct[k].data(), (int)ct[k].size(), plain, sizeof(plain),
                           idx, DIR, AUTH_TAG_SIZE);
        if (n != 48)
            return fail("late-prev-across-wrap FAILED to decrypt (epoch mismatch)");
        if (memcmp(plain, pt[k].data(), 48) != 0)
            return fail("late-prev-across-wrap decrypted WRONG plaintext");
    }
    printf("[TEST-AEAD-NONCE] OK: late prev 255 recovered after wrap decrypts at epoch-0 index\n");
    fflush(stdout);
    return 0;
}

// --- Case 8: encrypted-batch frame-span truncation -> auth-fail ---------------
// Guards the 2nd encryption bug (data-flow-encrypted-batch-size.md §2/§4): the
// whole-batch AEAD MAC covers the EXACT ciphertext + its LENGTH (RFC 8439). The
// RX reassembles per-frame DATA chunks into one ciphertext, then decrypts it as
// ONE atomic unit. If the batch completion gate ever delivers ONE FRAME SHORT
// (e.g. a lost-EOB prev batch whose expected count collapsed by 1), the
// reassembled ciphertext is truncated -> MAC over a shorter buffer -> auth-fail
// -> false PSK-mismatch disconnect.
//
// This case reproduces the mechanism at the crypto layer: encrypt a multi-frame
// AEAD unit, split the ciphertext into per-frame chunks exactly as the TX frame-
// split does, then
//   (a) reassemble ALL chunks -> decrypt MUST succeed (the post-fix steady
//       state: the unit is bounded to crypto_batch_size frames with
//       retransmit_headroom free, so the full set ALWAYS lands and reassembles
//       byte-exact);
//   (b) reassemble all-but-the-last chunk -> decrypt MUST auth-FAIL (the
//       FAIL-BEFORE short-delivery the fix prevents from ever reaching here).
// The TX fix is what guarantees (a) on the wire (the batch reliably completes
// inside the negotiated unit); this case proves the crypto contract both arms
// depend on.
int test_batch_span_truncation_authfail()
{
    cl_cipher_suite tx, rx;
    if (make_paired_suites(tx, rx) != 0)
        return fail("X25519 pairing setup failed");
    // SHARED key so only TRUNCATION fails, not key

    const uint32_t DIR = DIRECTION_CMD_TO_RSP;
    // Model a CFG-class encrypted batch: crypto_batch_size(20) frames at a
    // representative WB max_frame payload. Use 20 frames so the unit matches the
    // negotiated crypto_batch_size the TX fix bounds to.
    const int max_frame   = 140;          // representative OFDM DATA_LONG payload
    const int n_frames    = 20;           // == crypto_batch_size (the bounded span)
    const int plain_len   = n_frames * max_frame - AUTH_TAG_SIZE;  // pad-to-frames

    std::vector<uint8_t> pt(plain_len);
    for (int i = 0; i < plain_len; i++) pt[i] = (uint8_t)((i * 31 + 7) & 0xFF);

    uint64_t e = 0; int l = -1;
    uint64_t idx = cl_cipher_suite::unwrap_batch_index(7, &e, &l);

    std::vector<uint8_t> ct(plain_len + AUTH_TAG_SIZE);
    int enc = tx.encrypt(pt.data(), plain_len, ct.data(), (int)ct.size(),
                         idx, DIR, AUTH_TAG_SIZE);
    if (enc != plain_len + AUTH_TAG_SIZE)
        return fail("batch-span encrypt returned wrong size");

    // The sealed ciphertext spans exactly n_frames frames (the last carries the
    // 16-byte tag). The frame-split is just chunking ct into <=max_frame pieces;
    // reassembly is the concatenation back. Total length is what the MAC binds.
    const int ct_len = enc;
    const int last_frame_len =
        ct_len - (ct_len / max_frame) * max_frame; // bytes in the final chunk
    (void)last_frame_len;

    // (a) FULL reassembly -> decrypt OK + correct plaintext.
    {
        uint64_t re = 0; int rl = -1;
        uint64_t ridx = cl_cipher_suite::unwrap_batch_index(7, &re, &rl);
        std::vector<uint8_t> out(plain_len);
        int n = rx.decrypt(ct.data(), ct_len, out.data(), (int)out.size(),
                           ridx, DIR, AUTH_TAG_SIZE);
        if (n != plain_len)
            return fail("FULL-batch reassembly FAILED to decrypt (auth-fail on the byte-exact unit)");
        if (memcmp(out.data(), pt.data(), plain_len) != 0)
            return fail("FULL-batch reassembly decrypted to WRONG plaintext");
    }

    // (b) ONE-FRAME-SHORT reassembly -> decrypt MUST auth-FAIL.
    // Drop the final frame's worth of ciphertext (the lost-EOB tail). The MAC
    // input length + bytes change -> crypto_aead_read must reject.
    {
        int short_len = ct_len - max_frame;       // drop the last full frame
        if (short_len <= AUTH_TAG_SIZE)           // keep the test meaningful
            short_len = ct_len - 1;
        uint64_t re = 0; int rl = -1;
        uint64_t ridx = cl_cipher_suite::unwrap_batch_index(7, &re, &rl);
        std::vector<uint8_t> out(plain_len);
        int n = rx.decrypt(ct.data(), short_len, out.data(), (int)out.size(),
                           ridx, DIR, AUTH_TAG_SIZE);
        if (n > 0)
            return fail("ONE-FRAME-SHORT reassembly DECRYPTED (truncation not rejected — the bug)");
    }

    printf("[TEST-AEAD-NONCE] OK: full %d-frame AEAD unit decrypts; one-frame-short reassembly auth-fails (batch-span fix contract)\n",
           n_frames);
    fflush(stdout);
    return 0;
}

// ============================================================================
// §11 RE-SEAL NONCE-REUSE GUARANTEE — BREAK-rebuild / SACK-reorder / wrap
// (data-flow-aead-nonce.md §11). These model the PRODUCTION re-seal logic at the
// crypto layer: a TX "seal" that bumps the GENERATION on every recovery re-queue
// (restore_tx_from_compressed) and applies the encrypt-site HIGH-WATER guard; a
// RX "decrypt" that bumps the generation on every config-transition re-adopt it
// processes. The folded index = fold_gen_index(gen, unwrap_index). The
// SECURITY INVARIANT asserted: across BREAK-rebuild (bsi re-stamp), SACK
// reorder/retx, and >255 batch wrap, NO (dir,index)/nonce is EVER reused with
// two different plaintexts, and every batch RX needs decrypts.
// ============================================================================

// A tiny TX-side seal model mirroring arq_commander.cc encrypt site + the
// restore_tx_from_compressed gen bump. Tracks (epoch,last_bsi,gen,high_water).
struct tx_seal_model {
    cl_cipher_suite cs;
    uint64_t epoch = 0; int last_bsi = -1;
    uint64_t gen = 0;
    uint64_t high_water = UINT64_MAX;     // unset

    // Seal one batch at wire bsi over plaintext pt; returns 0 on success,
    // -1 if encryption fails, and writes ciphertext to ct.
    // Mirrors the production: unwrap -> fold gen -> high-water guard -> encrypt.
    int seal(int wire_bsi, const std::vector<uint8_t>& pt,
             std::vector<uint8_t>& ct, uint64_t* out_idx)
    {
        uint64_t uw = cl_cipher_suite::unwrap_batch_index(wire_bsi, &epoch, &last_bsi);
        uint64_t idx = cl_cipher_suite::fold_gen_index(gen, uw);
        if (high_water != UINT64_MAX && idx <= high_water) {
            uint64_t band = high_water / cl_cipher_suite::NONCE_GEN_STRIDE;
            gen = band + 1;
            idx = cl_cipher_suite::fold_gen_index(gen, uw);
        }
        high_water = idx;
        ct.resize(pt.size() + AUTH_TAG_SIZE);
        int n = cs.encrypt(pt.data(), (int)pt.size(), ct.data(), (int)ct.size(),
                           idx, DIRECTION_CMD_TO_RSP, AUTH_TAG_SIZE);
        if (n < 0) {
            ct.clear();
            if (out_idx) *out_idx = idx;
            return -1;
        }
        ct.resize(n);
        if (out_idx) *out_idx = idx;
        return 0;
    }
    // restore_tx_from_compressed recovery: bump the generation.
    void recovery() { gen++; }
};

// RX-side decrypt model mirroring arq_common.cc copy_data_to_buffer + the
// arq_responder.cc re-adopt gen bump.
struct rx_decrypt_model {
    cl_cipher_suite cs;
    uint64_t epoch = 0; int last_bsi = -1;
    uint64_t gen = 0;
    bool adopted_once = false;

    // A config-transition re-adopt: bump gen iff already adopted once.
    void transition_readopt() { if (adopted_once) gen++; adopted_once = true; }

    int decrypt(int wire_bsi, const std::vector<uint8_t>& ct,
                std::vector<uint8_t>& out)
    {
        uint64_t uw = cl_cipher_suite::unwrap_batch_index(wire_bsi, &epoch, &last_bsi);
        uint64_t idx = cl_cipher_suite::fold_gen_index(gen, uw);
        out.assign(ct.size() > AUTH_TAG_SIZE ? ct.size() - AUTH_TAG_SIZE : 0, 0);
        return cs.decrypt(ct.data(), (int)ct.size(), out.data(), (int)out.size(),
                          idx, DIRECTION_CMD_TO_RSP, AUTH_TAG_SIZE);
    }
};

std::vector<uint8_t> mk_pt(int seed, int len)
{
    std::vector<uint8_t> v(len);
    for (int i = 0; i < len; i++) v[i] = (uint8_t)((seed * 131 + i * 7 + 1) & 0xFF);
    return v;
}

// --- Case 9: BREAK-rebuild bsi RE-STAMP -> NO nonce reuse, RX decrypts --------
// The ROOT bug (§10.3 P3): TX seals bsi=7, then a BREAK FREEs+re-queues the
// plaintext WITHOUT advancing the bsi and re-seals the SAME bsi=7 over DIFFERENT
// (re-compressed) plaintext. Old design: same (key,nonce), 2 plaintexts = REUSE.
// New design: the recovery bumps the gen, so the re-seal lands at a strictly-
// higher index; the high-water guard backstops it. RX observed the transition
// (re-adopt) so it decrypts the re-seal. We assert: (1) the two seals used
// DISTINCT indices; (2) NO index emitted twice; (3) RX decrypts the re-seal to
// the NEW plaintext.
int test_break_rebuild_no_reuse()
{
    tx_seal_model tx; rx_decrypt_model rx;
    if (make_paired_suites(tx.cs, rx.cs) != 0)
        return fail("X25519 pairing setup failed");

    std::set<uint64_t> emitted;

    // Steady-state: TX seals bsi 0..6 and RX delivers them (RX adopts at bsi 0).
    rx.transition_readopt();   // first-ever adopt (gen stays 0)
    for (int b = 0; b <= 6; b++) {
        uint64_t idx; auto pt = mk_pt(b, 80);
        std::vector<uint8_t> ct;
        if (tx.seal(b, pt, ct, &idx) != 0)
            return fail("pre-BREAK seal failed");
        if (!emitted.insert(idx).second) return fail("pre-BREAK index reused");
        std::vector<uint8_t> out;
        if (rx.decrypt(b, ct, out) != (int)pt.size() || out != pt)
            return fail("pre-BREAK batch failed to decrypt");
    }

    // TX seals bsi=7 (aborted batch — never delivered to RX).
    uint64_t idx_a; auto pt_a = mk_pt(700, 90);
    std::vector<uint8_t> ct_a;
    if (tx.seal(7, pt_a, ct_a, &idx_a) != 0)
        return fail("aborted seal failed");
    if (!emitted.insert(idx_a).second) return fail("aborted seal index reused");
    // (RX never sees ct_a — the batch was freed mid-flight by the BREAK.)

    // BREAK: recovery re-queues plaintext; TX bumps gen. RX processes the
    // transition (SET_CONFIG/CONFIG_TAG re-adopt) and bumps its gen too.
    tx.recovery();
    rx.transition_readopt();

    // The re-build re-stamps the SAME wire bsi=7 (rolled back for delivery
    // contiguity) over DIFFERENT plaintext (re-compressed at the demoted cfg).
    uint64_t idx_b; auto pt_b = mk_pt(701, 50);   // different content + length
    std::vector<uint8_t> ct_b;
    if (tx.seal(7, pt_b, ct_b, &idx_b) != 0)
        return fail("BREAK re-seal failed");

    // (1) the two seals at bsi=7 MUST have used DISTINCT indices.
    if (idx_a == idx_b)
        return fail("BREAK re-stamp at bsi=7 reused the SAME nonce index (KEYSTREAM REUSE)");
    // (2) no index emitted twice across the whole run.
    if (!emitted.insert(idx_b).second)
        return fail("BREAK re-seal index collides with an earlier emitted index (REUSE)");
    // (3) RX decrypts the re-seal to the NEW plaintext (gen aligned via re-adopt).
    std::vector<uint8_t> out_b;
    if (rx.decrypt(7, ct_b, out_b) != (int)pt_b.size() || out_b != pt_b)
        return fail("BREAK re-sealed batch failed to decrypt on RX (gen desync)");

    printf("[TEST-AEAD-NONCE] OK: BREAK-rebuild bsi=7 re-stamp -> distinct nonces "
           "(idx %llu vs %llu), 0 reuse, RX decrypts re-seal\n",
           (unsigned long long)idx_a, (unsigned long long)idx_b);
    fflush(stdout);
    return 0;
}

// --- Case 10: harsh SACK reorder/retx WITHIN a generation -> no reuse ---------
// No config transition (gen fixed): TX seals 20..29, RX delivers OUT OF ORDER
// with a retransmit replaying STORED ciphertext (verbatim, same nonce+plaintext).
// Assert every nonce distinct across new seals, retx is a verbatim replay (NOT a
// re-encrypt), and every delivery (incl. reorder + retx) decrypts correctly.
int test_sack_reorder_retx_no_reuse()
{
    tx_seal_model tx; rx_decrypt_model rx;
    if (make_paired_suites(tx.cs, rx.cs) != 0)
        return fail("X25519 pairing setup failed");

    const int BASE = 20, N = 10;
    std::set<uint64_t> emitted;
    std::vector<std::vector<uint8_t>> ct(N), pt(N);
    std::vector<uint64_t> idx(N);
    for (int k = 0; k < N; k++) {
        pt[k] = mk_pt(2000 + k, 70 + k);
        if (tx.seal(BASE + k, pt[k], ct[k], &idx[k]) != 0)
            return fail("SACK seal failed");
        if (!emitted.insert(idx[k]).second)
            return fail("SACK new-seal nonce reused");
    }
    // RX delivers reordered: 22,20,29,21,25,...,then RETX 22 (verbatim replay).
    int order[] = {2, 0, 9, 1, 5, 3, 4, 6, 7, 8, 2 /*retx*/};
    rx.transition_readopt();   // first adopt at bsi=22 (gen 0)
    for (int o = 0; o < (int)(sizeof(order)/sizeof(order[0])); o++) {
        int k = order[o];
        std::vector<uint8_t> out;
        if (rx.decrypt(BASE + k, ct[k], out) != (int)pt[k].size() || out != pt[k])
            return fail("SACK reorder/retx batch failed to decrypt");
    }
    // Retx is a VERBATIM replay: same nonce index, same ciphertext -> SAFE by
    // construction (never a second different plaintext under that nonce).
    printf("[TEST-AEAD-NONCE] OK: SACK reorder(10 batches)+retx all decrypt; "
           "%zu distinct nonces, 0 reuse\n", emitted.size());
    fflush(stdout);
    return 0;
}

// --- Case 11: batch-wrap >255 WITH interleaved BREAK recoveries -> no reuse ----
// Drive >255 batches (so the 8-bit wire bsi recurs) AND inject a BREAK recovery
// (bsi re-stamp) at a few points. Assert NO folded index is ever emitted twice
// and a sampled re-seal still decrypts. This is the union of the wrap-epoch and
// the gen mechanisms — the two must not collide (disjoint index bands).
int test_wrap_with_recoveries_no_reuse()
{
    tx_seal_model tx; rx_decrypt_model rx;
    if (make_paired_suites(tx.cs, rx.cs) != 0)
        return fail("X25519 pairing setup failed");

    std::set<uint64_t> emitted;
    rx.transition_readopt();   // first adopt
    int wire = 0;
    int last_restamp_wire = -1;
    std::vector<uint8_t> last_pt; std::vector<uint8_t> last_ct;
    uint64_t last_idx = 0; bool have_restamp = false;

    for (int n = 0; n < 700; n++) {
        // Every 137 batches, simulate a BREAK: recovery + RE-STAMP the SAME wire
        // bsi over different plaintext (the §11 hazard), then resume forward.
        if (n > 0 && n % 137 == 0) {
            tx.recovery();
            rx.transition_readopt();
            // re-stamp the SAME wire bsi (no forward advance) over new plaintext
            auto rpt = mk_pt(90000 + n, 60);
            uint64_t ridx;
            std::vector<uint8_t> rct;
            if (tx.seal(wire, rpt, rct, &ridx) != 0)
                return fail("wrap+recovery: seal failed");
            if (!emitted.insert(ridx).second)
                return fail("wrap+recovery: re-stamp folded index REUSED (NONCE REUSE)");
            last_restamp_wire = wire; last_pt = rpt; last_ct = rct;
            last_idx = ridx; have_restamp = true;
            // RX decrypts the re-stamp (it observed the transition)
            std::vector<uint8_t> rout;
            if (rx.decrypt(wire, rct, rout) != (int)rpt.size() || rout != rpt)
                return fail("wrap+recovery: re-stamp failed to decrypt");
            // The re-stamped batch consumed wire bsi W; the NEXT new-data batch
            // advances to W+1 (production: cmd_batch_seq_id +1 after the completed
            // re-sent batch). Both peers keep the bumped generation for the rest of
            // this generation, so subsequent forward batches stay gen-aligned.
            wire = (wire + 1) & 0xFF;
            continue;
        }
        auto pt = mk_pt(n, 64);
        uint64_t idx;
        std::vector<uint8_t> ct;
        if (tx.seal(wire, pt, ct, &idx) != 0)
            return fail("wrap+recovery: seal failed");
        if (!emitted.insert(idx).second)
            return fail("wrap+recovery: forward folded index REUSED across wrap");
        std::vector<uint8_t> out;
        if (rx.decrypt(wire, ct, out) != (int)pt.size() || out != pt)
            return fail("wrap+recovery: forward batch failed to decrypt across wrap");
        wire = (wire + 1) & 0xFF;
    }
    if (!have_restamp) return fail("wrap+recovery test never exercised a re-stamp (vacuous)");
    (void)last_restamp_wire; (void)last_pt; (void)last_ct; (void)last_idx;
    printf("[TEST-AEAD-NONCE] OK: 700 batches across >2 wraps + 5 BREAK re-stamps, "
           "%zu folded indices ALL UNIQUE (0 reuse), re-stamps decrypt\n", emitted.size());
    fflush(stdout);
    return 0;
}

// --- Case 12: HIGH-WATER GUARD backstop (gen bookkeeping DELIBERATELY broken) --
// Safety must NOT depend on the gen counters being correct (data-flow-aead-nonce.md
// §11.3). Re-seal the SAME wire bsi over DIFFERENT plaintext WITHOUT calling
// recovery() (i.e. tx_nonce_gen is NOT bumped — modelling a gen-bookkeeping bug).
// The encrypt-site high-water guard MUST still force a distinct, strictly-higher
// index so no (key,nonce) is reused. (RX would then auth-fail on the un-bumped
// gen — a SAFE drop, asserted — never a reuse.)
int test_high_water_guard_backstop()
{
    tx_seal_model tx; rx_decrypt_model rx;
    if (make_paired_suites(tx.cs, rx.cs) != 0)
        return fail("X25519 pairing setup failed");

    // Seal bsi=3 (gen 0).
    uint64_t idx0; auto pt0 = mk_pt(300, 64);
    std::vector<uint8_t> ct0;
    if (tx.seal(3, pt0, ct0, &idx0) != 0)
        return fail("initial seal failed");

    // Re-seal the SAME bsi=3 over DIFFERENT plaintext WITHOUT bumping the gen
    // (tx.recovery() intentionally NOT called). unwrap(3) with last_bsi already 3
    // returns the SAME unwrap index; gen is still 0 -> candidate index == idx0
    // <= high_water -> the GUARD must fire and bump the gen internally.
    uint64_t idx1; auto pt1 = mk_pt(301, 40);
    std::vector<uint8_t> ct1;
    if (tx.seal(3, pt1, ct1, &idx1) != 0)
        return fail("guarded re-seal failed");

    if (idx1 == idx0)
        return fail("high-water guard did NOT prevent reuse (same index, diff plaintext)");
    if (idx1 <= idx0)
        return fail("high-water guard produced a non-increasing index");
    if (ct0 == ct1)
        return fail("two seals of the same bsi produced identical ciphertext (reuse)");

    // The guard kept TX safe even though the gen was never bumped. RX (gen 0,
    // no re-adopt — modelling the SAME broken bookkeeping) reconstructs the OLD
    // index for bsi=3: it decrypts the FIRST seal fine, but the guarded re-seal
    // (a higher gen band on TX) auth-FAILS on RX — a SAFE drop, NOT a reuse.
    {
        std::vector<uint8_t> o0;
        if (rx.decrypt(3, ct0, o0) != (int)pt0.size() || o0 != pt0)
            return fail("guard test: original seal must still decrypt on aligned gen");
    }
    {
        // NOTE: rx is at gen 0 (transition_readopt never called) — decrypting the
        // guarded re-seal at the wrong gen MUST auth-fail, never reuse/leak.
        std::vector<uint8_t> o1;
        if (rx.decrypt(3, ct1, o1) > 0)
            return fail("guarded re-seal decrypted at the WRONG gen (should auth-fail safely)");
    }

    printf("[TEST-AEAD-NONCE] OK: high-water guard backstops a BROKEN gen "
           "(idx %llu -> %llu, no reuse; wrong-gen RX safely auth-fails)\n",
           (unsigned long long)idx0, (unsigned long long)idx1);
    fflush(stdout);
    return 0;
}

} // namespace

int run_aead_nonce_tests()
{
    printf("[TEST-AEAD-NONCE] === AEAD bsi-bound nonce regression suite ===\n");
    fflush(stdout);
    int failed = 0;
    failed += test_unwrap_monotone_wrap();
    failed += test_nonce_uniqueness();
    failed += test_direction_disjoint();
    failed += test_reorder_retx_roundtrip();
    failed += test_kx_truncated_reject();
    failed += test_tamper_rejected();
    failed += test_backward_across_wrap();
    failed += test_batch_span_truncation_authfail();
    failed += test_break_rebuild_no_reuse();
    failed += test_sack_reorder_retx_no_reuse();
    failed += test_wrap_with_recoveries_no_reuse();
    failed += test_high_water_guard_backstop();
    if (failed == 0)
        printf("[TEST-AEAD-NONCE] === ALL PASS ===\n");
    else
        printf("[TEST-AEAD-NONCE] === %d FAILED ===\n", failed);
    fflush(stdout);
    return failed;
}
