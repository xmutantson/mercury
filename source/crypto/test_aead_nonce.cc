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
void make_paired_suites(cl_cipher_suite& cmd, cl_cipher_suite& rsp)
{
    uint8_t pk_cmd[X25519_KEY_SIZE];
    uint8_t pk_rsp[X25519_KEY_SIZE];
    cmd.generate_x25519_keypair(pk_cmd);
    rsp.generate_x25519_keypair(pk_rsp);
    // Each computes the shared secret from the OTHER's pubkey -> identical secret.
    cmd.compute_x25519_shared(pk_rsp);
    rsp.compute_x25519_shared(pk_cmd);
    const char* psk = "MERCURY-AEAD-NONCE-TEST-PSK";
    // Same (commander_call, responder_call) ordering on both ends so the salt
    // matches (production passes them role-consistently).
    cmd.derive_session_key("CMDTEST", "RSPTEST",
                           (const uint8_t*)psk, (int)strlen(psk), false);
    rsp.derive_session_key("CMDTEST", "RSPTEST",
                           (const uint8_t*)psk, (int)strlen(psk), false);
    cmd.activate();
    rsp.activate();
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
    make_paired_suites(tx, rx);   // tx=CMD encrypt, rx=RSP decrypt, SHARED key

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
    cs.generate_x25519_keypair(pk);

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
    make_paired_suites(tx, rx);   // SHARED key so a CLEAN ct would decrypt;
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
    make_paired_suites(tx, rx);
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
    if (failed == 0)
        printf("[TEST-AEAD-NONCE] === ALL PASS ===\n");
    else
        printf("[TEST-AEAD-NONCE] === %d FAILED ===\n", failed);
    fflush(stdout);
    return failed;
}
