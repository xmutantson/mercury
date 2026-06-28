/*
 * ML-KEM-768 hybrid KEX regression tests.
 *
 * Wired via mercury.exe --test (run_mlkem_hybrid_tests). Fast, deterministic,
 * no IONOS / RF / telecom_system. Validates the two load-bearing correctness
 * points of the X25519 + ML-KEM-768 hybrid combiner (MLKEM_HYBRID_PLAN.md
 * §2-§3, fact-documents/data-flow-hybrid-kex.md) plus the KX chunk transport:
 *
 *  (A) COMBINER SYMMETRY: a full CMD/RSP hybrid exchange (X25519 both ways +
 *      ML-KEM keypair/encaps/decaps) derives the SAME session key on both ends
 *      (proxied by compute_key_confirmation, the protocol's own integrity gate).
 *
 *  (B) SHARED-SECRET ORDER (ML-KEM FIRST, §4.3): the hybrid key MUST differ
 *      from a key derived with the secrets in the legacy (x25519||mlkem) order.
 *      Fail-before: the pre-fix build concatenated x25519||mlkem; this asserts
 *      the new mlkem||x25519 order is what's used.
 *
 *  (C) TRANSCRIPT BINDING (§6 caveat / X-Wing): flipping ONE byte of mlkem_ct,
 *      mlkem_pk, or either X25519 pub in the bind input changes the session key
 *      -> a tamper/bit-error that slips past CRC yields a confirm-tag mismatch
 *      -> clean disconnect. Fail-before: a combiner that ignored the transcript
 *      would produce identical keys despite the flipped public material.
 *
 *  (D) KX CHUNK ROUND-TRIP: encode/reassemble a 1184B (KX2) and 1088B (KX3)
 *      buffer at several chunk capacities; assert byte-identical reassembly and
 *      that CRC8 / kind / index checks reject a corrupted or mislabeled chunk.
 */

#include "../../include/crypto/mercury_crypto.h"
#include "test_mlkem_hybrid.h"

#include <cstdio>
#include <cstring>
#include <cstdint>

namespace {

static int g_failed = 0;
static void check(bool cond, const char* name)
{
    if (cond) { printf("[MLKEM-TEST] PASS  %s\n", name); }
    else      { printf("[MLKEM-TEST] FAIL  %s\n", name); g_failed++; }
    fflush(stdout);
}

// Compare two 8-byte key-confirmation tags. Equal tags <=> equal session keys
// (the tag is Blake2b-keyed over the session key — the same gate the live
// KEY_ACTIVATE round-trip uses).
static bool tags_equal(cl_cipher_suite& a, cl_cipher_suite& b)
{
    uint8_t ta[8], tb[8];
    a.compute_key_confirmation(ta);
    b.compute_key_confirmation(tb);
    return memcmp(ta, tb, 8) == 0;
}

// Run a full hybrid exchange into cmd + rsp, optionally tampering one byte of a
// chosen transcript element on the RSP side only (to test binding). Returns
// after both sides have derived their hybrid session keys.
//   tamper: 0 = none, 1 = mlkem_ct, 2 = mlkem_pk, 3 = pk_cmd, 4 = pk_rsp
static void run_hybrid(cl_cipher_suite& cmd, cl_cipher_suite& rsp, int tamper)
{
    // --- KX1: X25519 both ways ---
    uint8_t cmd_pub[X25519_KEY_SIZE], rsp_pub[X25519_KEY_SIZE];
    cmd.generate_x25519_keypair(cmd_pub);
    rsp.generate_x25519_keypair(rsp_pub);
    cmd.compute_x25519_shared(rsp_pub);   // CMD: peer = RSP pub
    rsp.compute_x25519_shared(cmd_pub);   // RSP: peer = CMD pub

    // --- KX2: CMD generates ML-KEM keypair, publishes encaps key ---
    uint8_t mlkem_pk[MLKEM_PK_SIZE];
    cmd.generate_mlkem_keypair(mlkem_pk);

    // --- KX3: RSP encapsulates against CMD's encaps key -> ciphertext ---
    uint8_t mlkem_ct[MLKEM_CT_SIZE];
    rsp.encapsulate_mlkem(mlkem_pk, mlkem_ct);

    // CMD decapsulates the ciphertext to recover the same ML-KEM ss
    cmd.decapsulate_mlkem(mlkem_ct);

    // Transcript-binding material. CMD and RSP must assemble IDENTICAL bytes;
    // pk order is ALWAYS commander-then-responder regardless of role.
    // The CMD side sees the true (untampered) transcript; the RSP side may see a
    // single-byte-flipped copy (modeling a MITM / undetected bit error).
    uint8_t ct_c[MLKEM_CT_SIZE], pk_c[MLKEM_PK_SIZE];
    uint8_t pkcmd_c[X25519_KEY_SIZE], pkrsp_c[X25519_KEY_SIZE];
    memcpy(ct_c, mlkem_ct, MLKEM_CT_SIZE);
    memcpy(pk_c, mlkem_pk, MLKEM_PK_SIZE);
    memcpy(pkcmd_c, cmd_pub, X25519_KEY_SIZE);
    memcpy(pkrsp_c, rsp_pub, X25519_KEY_SIZE);

    uint8_t ct_r[MLKEM_CT_SIZE], pk_r[MLKEM_PK_SIZE];
    uint8_t pkcmd_r[X25519_KEY_SIZE], pkrsp_r[X25519_KEY_SIZE];
    memcpy(ct_r, mlkem_ct, MLKEM_CT_SIZE);
    memcpy(pk_r, mlkem_pk, MLKEM_PK_SIZE);
    memcpy(pkcmd_r, cmd_pub, X25519_KEY_SIZE);
    memcpy(pkrsp_r, rsp_pub, X25519_KEY_SIZE);

    if (tamper == 1) ct_r[7]    ^= 0x01;
    if (tamper == 2) pk_r[11]   ^= 0x01;
    if (tamper == 3) pkcmd_r[3] ^= 0x01;
    if (tamper == 4) pkrsp_r[5] ^= 0x01;

    cmd.derive_session_key("CMDCALL", "RSPCALL", nullptr, 0, /*mlkem_done=*/true,
                           ct_c, pk_c, pkcmd_c, pkrsp_c);
    rsp.derive_session_key("CMDCALL", "RSPCALL", nullptr, 0, /*mlkem_done=*/true,
                           ct_r, pk_r, pkcmd_r, pkrsp_r);
}

// --- (A) symmetry + (B) order + (C) binding ---
void test_combiner()
{
    {
        cl_cipher_suite cmd, rsp;
        run_hybrid(cmd, rsp, 0);
        check(tags_equal(cmd, rsp),
              "A: hybrid CMD/RSP derive identical session key");
        check(cmd.is_pq_upgraded() && rsp.is_pq_upgraded(),
              "A: pq_active set on both ends after hybrid derive");
    }

    // (B) Order regression: derive with ML-KEM ss first (production) must differ
    // from a hand-built key with the LEGACY x25519||mlkem order. We can't reach
    // the private session_key, but we CAN prove the order matters: build two
    // suites that share the SAME two shared secrets and feed them in opposite
    // order via a controlled construction. Simplest faithful check: a hybrid
    // exchange's key MUST differ from the SAME parties' classical-only key
    // (which uses x25519 alone) — the ML-KEM secret genuinely entered the mix
    // in a position the classical path never occupies.
    {
        cl_cipher_suite cmd, rsp;
        run_hybrid(cmd, rsp, 0);          // hybrid key on cmd
        uint8_t hybrid_tag[8];
        cmd.compute_key_confirmation(hybrid_tag);

        // Re-derive classical-only on the SAME cmd suite (same x25519 ss, no
        // ML-KEM, no bind) — this is the byte-identical-to-legacy path.
        cmd.derive_session_key("CMDCALL", "RSPCALL", nullptr, 0,
                               /*mlkem_done=*/false);
        uint8_t classical_tag[8];
        cmd.compute_key_confirmation(classical_tag);
        check(memcmp(hybrid_tag, classical_tag, 8) != 0,
              "B: hybrid key differs from classical-only key (ML-KEM ss entered)");
    }

    // (B2) TRUE IKM-ORDER regression (the prompt's "regression vs the old
    // backwards order"). The B check above only proves hybrid != classical — it
    // would NOT catch a revert to the legacy x25519||mlkem order, because both
    // orders still differ from the classical-only key. Here we derive from the
    // SAME two shared secrets twice: production (mlkem||x25519, via the normal
    // derive captured by copy_session_key_for_test) and the legacy order (via
    // derive_session_key_legacy_order_for_test) with byte-identical salt/bind.
    // If they ever match, the order swap has been reverted / is a no-op.
    {
        cl_cipher_suite cmd, rsp;
        // Inline a hybrid exchange so we keep the exact transcript bytes.
        uint8_t cmd_pub[X25519_KEY_SIZE], rsp_pub[X25519_KEY_SIZE];
        cmd.generate_x25519_keypair(cmd_pub);
        rsp.generate_x25519_keypair(rsp_pub);
        cmd.compute_x25519_shared(rsp_pub);
        rsp.compute_x25519_shared(cmd_pub);
        uint8_t mlkem_pk[MLKEM_PK_SIZE];
        cmd.generate_mlkem_keypair(mlkem_pk);
        uint8_t mlkem_ct[MLKEM_CT_SIZE];
        rsp.encapsulate_mlkem(mlkem_pk, mlkem_ct);
        cmd.decapsulate_mlkem(mlkem_ct);

        // Production order (mlkem||x25519) on cmd:
        cmd.derive_session_key("CMDCALL", "RSPCALL", nullptr, 0, /*mlkem_done=*/true,
                               mlkem_ct, mlkem_pk, cmd_pub, rsp_pub);
        uint8_t prod_key[SESSION_KEY_SIZE];
        cmd.copy_session_key_for_test(prod_key);

        // Legacy order (x25519||mlkem) on the SAME cmd suite (same two ss,
        // same salt/bind) — overwrites session_key:
        cmd.derive_session_key_legacy_order_for_test("CMDCALL", "RSPCALL", nullptr, 0,
                                                     mlkem_ct, mlkem_pk, cmd_pub, rsp_pub);
        uint8_t legacy_key[SESSION_KEY_SIZE];
        cmd.copy_session_key_for_test(legacy_key);

        check(memcmp(prod_key, legacy_key, SESSION_KEY_SIZE) != 0,
              "B2: mlkem-first IKM differs from legacy x25519-first (order regression)");

        // Re-derive production order and confirm it reproduces prod_key exactly
        // (the legacy call above must not have perturbed any shared-secret state).
        cmd.derive_session_key("CMDCALL", "RSPCALL", nullptr, 0, /*mlkem_done=*/true,
                               mlkem_ct, mlkem_pk, cmd_pub, rsp_pub);
        uint8_t prod_key2[SESSION_KEY_SIZE];
        cmd.copy_session_key_for_test(prod_key2);
        check(memcmp(prod_key, prod_key2, SESSION_KEY_SIZE) == 0,
              "B2: production derive is deterministic (mlkem-first reproduces)");
    }

    // (C) Transcript binding: each tamper position must break key agreement.
    const char* names[4] = {
        "C: flip mlkem_ct byte -> keys diverge (bind covers ct)",
        "C: flip mlkem_pk byte -> keys diverge (bind covers pk)",
        "C: flip x25519 pk_cmd byte -> keys diverge (bind covers pk_cmd)",
        "C: flip x25519 pk_rsp byte -> keys diverge (bind covers pk_rsp)",
    };
    for (int t = 1; t <= 4; t++)
    {
        cl_cipher_suite cmd, rsp;
        run_hybrid(cmd, rsp, t);
        check(!tags_equal(cmd, rsp), names[t - 1]);
    }
}

// --- (D) KX chunk round-trip + CRC/kind/index rejection ---
void test_chunking()
{
    struct Case { uint8_t kind; int total; int cap; const char* label; };
    Case cases[] = {
        { cl_cipher_suite::KX_KIND_PK, MLKEM_PK_SIZE, 196, "D: KX2 1184B @cap196" },
        { cl_cipher_suite::KX_KIND_PK, MLKEM_PK_SIZE, 100, "D: KX2 1184B @cap100" },
        { cl_cipher_suite::KX_KIND_PK, MLKEM_PK_SIZE,  49, "D: KX2 1184B @cap49" },
        { cl_cipher_suite::KX_KIND_CT, MLKEM_CT_SIZE, 196, "D: KX3 1088B @cap196" },
        { cl_cipher_suite::KX_KIND_CT, MLKEM_CT_SIZE,  53, "D: KX3 1088B @cap53" },
        { cl_cipher_suite::KX_KIND_CT, MLKEM_CT_SIZE,   7, "D: KX3 1088B @cap7" },
    };

    for (auto& c : cases)
    {
        // Source payload: a deterministic ramp.
        uint8_t src[MLKEM_PK_SIZE];
        for (int i = 0; i < c.total; i++) src[i] = (uint8_t)((i * 31 + 7) & 0xFF);

        int count = cl_cipher_suite::kx_chunk_count(c.total, c.cap);
        bool ok = (count >= 1);

        uint8_t reasm[MLKEM_PK_SIZE];
        memset(reasm, 0, sizeof(reasm));
        bool got[256] = { false };
        int got_count = 0;

        for (int idx = 0; ok && idx < count; idx++)
        {
            uint8_t frame[4 + MLKEM_PK_SIZE];
            int flen = cl_cipher_suite::kx_chunk_encode(c.kind, src, c.total,
                                                        idx, c.cap,
                                                        frame, sizeof(frame));
            if (flen < 0) { ok = false; break; }

            int oi = -1, oc = -1;
            int got_payload = cl_cipher_suite::kx_chunk_decode(
                c.kind, frame, flen, c.cap, reasm, c.total, &oi, &oc);
            if (got_payload < 0 || oi != idx || oc != count) { ok = false; break; }
            if (!got[idx]) { got[idx] = true; got_count++; }
        }

        ok = ok && (got_count == count) && (memcmp(reasm, src, c.total) == 0);
        check(ok, c.label);
    }

    // (D2) REORDER + DROP-then-RETRANSMIT reassembly (the prompt's "incl a
    // dropped/reordered chunk -> retransmit"). The decode API explicitly does
    // NOT assume contiguous arrival; the receiver tracks a bitmap and is done
    // only when every index has arrived. We deliver chunks out of order, DROP
    // one mid-stream (as ARQ loss would), keep going, then RETRANSMIT the
    // dropped chunk last — and assert byte-identical reassembly. A duplicate
    // re-delivery (ARQ can resend an already-acked chunk) must be idempotent.
    {
        const uint8_t kind = cl_cipher_suite::KX_KIND_CT;
        const int total = MLKEM_CT_SIZE;   // 1088B (KX3)
        const int cap   = 100;             // -> 11 chunks
        uint8_t src[MLKEM_CT_SIZE];
        for (int i = 0; i < total; i++) src[i] = (uint8_t)((i * 17 + 3) & 0xFF);

        int count = cl_cipher_suite::kx_chunk_count(total, cap);

        // Pre-encode every chunk frame (the sender's output). static to keep
        // this ~70KB buffer off the stack.
        static uint8_t frames[64][4 + MLKEM_CT_SIZE];
        int flens[64];
        bool enc_ok = (count >= 1 && count <= 64);
        for (int idx = 0; enc_ok && idx < count; idx++)
        {
            flens[idx] = cl_cipher_suite::kx_chunk_encode(
                kind, src, total, idx, cap, frames[idx], sizeof(frames[idx]));
            if (flens[idx] < 0) enc_ok = false;
        }

        // Receiver state.
        uint8_t reasm[MLKEM_CT_SIZE];
        memset(reasm, 0, sizeof(reasm));
        bool got[64] = { false };
        int got_count = 0;
        bool decode_ok = enc_ok;

        // A scrambled delivery order that (a) is non-contiguous, (b) omits the
        // last index initially (the "dropped" chunk), and (c) repeats an index
        // (a duplicate ARQ resend). Indices >= count are simply skipped.
        // Delivery list for count==11: out-of-order, drops index 4, dups 2.
        const int order[] = { 3, 0, 7, 2, 9, 2, 8, 1, 6, 10, 5 /* idx 4 dropped */ };
        for (size_t k = 0; decode_ok && k < sizeof(order)/sizeof(order[0]); k++)
        {
            int idx = order[k];
            if (idx >= count) continue;
            int oi = -1, oc = -1;
            int p = cl_cipher_suite::kx_chunk_decode(
                kind, frames[idx], flens[idx], cap, reasm, total, &oi, &oc);
            if (p < 0 || oi != idx || oc != count) { decode_ok = false; break; }
            if (!got[idx]) { got[idx] = true; got_count++; }   // dup is idempotent
        }

        // Mid-stream we are NOT done — the dropped index 4 (and any tail index
        // not in `order`) is still missing.
        bool incomplete_midstream = decode_ok && (got_count < count);

        // RETRANSMIT every still-missing index (ARQ resend), out of order.
        for (int idx = count - 1; decode_ok && idx >= 0; idx--)
        {
            if (got[idx]) continue;
            int oi = -1, oc = -1;
            int p = cl_cipher_suite::kx_chunk_decode(
                kind, frames[idx], flens[idx], cap, reasm, total, &oi, &oc);
            if (p < 0 || oi != idx) { decode_ok = false; break; }
            got[idx] = true; got_count++;
        }

        bool reorder_ok = decode_ok && incomplete_midstream &&
                          (got_count == count) &&
                          (memcmp(reasm, src, total) == 0);
        check(reorder_ok,
              "D2: reorder + drop-then-retransmit (+dup) reassembles byte-identical");
    }

    // CRC8 / kind / index rejection on a single KX2 chunk.
    {
        uint8_t src[MLKEM_PK_SIZE];
        for (int i = 0; i < MLKEM_PK_SIZE; i++) src[i] = (uint8_t)(i & 0xFF);
        uint8_t frame[4 + MLKEM_PK_SIZE];
        int flen = cl_cipher_suite::kx_chunk_encode(
            cl_cipher_suite::KX_KIND_PK, src, MLKEM_PK_SIZE, 0, 196,
            frame, sizeof(frame));
        uint8_t reasm[MLKEM_PK_SIZE];
        int oi, oc;

        // baseline: accepts
        bool base_ok = cl_cipher_suite::kx_chunk_decode(
            cl_cipher_suite::KX_KIND_PK, frame, flen, 196,
            reasm, MLKEM_PK_SIZE, &oi, &oc) >= 0;
        check(base_ok, "D: clean KX2 chunk accepted");

        // corrupt a payload byte but leave header CRC intact -> the chunk's
        // header still validates (CRC only covers the 3 header bytes), so the
        // codec ACCEPTS it; the corruption is caught later by the whole-key
        // KEY_ACTIVATE confirm-tag. Corrupt the HEADER instead to exercise CRC8.
        uint8_t bad = frame[2]; frame[2] ^= 0xFF;   // flip chunk_count
        bool crc_reject = cl_cipher_suite::kx_chunk_decode(
            cl_cipher_suite::KX_KIND_PK, frame, flen, 196,
            reasm, MLKEM_PK_SIZE, &oi, &oc) < 0;
        check(crc_reject, "D: corrupted KX header -> CRC8 rejects");
        frame[2] = bad;   // restore

        // wrong expected kind -> reject
        bool kind_reject = cl_cipher_suite::kx_chunk_decode(
            cl_cipher_suite::KX_KIND_CT, frame, flen, 196,
            reasm, MLKEM_PK_SIZE, &oi, &oc) < 0;
        check(kind_reject, "D: wrong kind -> rejected");

        // reasm too small -> reject (never overflow)
        bool ovf_reject = cl_cipher_suite::kx_chunk_decode(
            cl_cipher_suite::KX_KIND_PK, frame, flen, 196,
            reasm, 10, &oi, &oc) < 0;
        check(ovf_reject, "D: reasm capacity exceeded -> rejected (no overflow)");
    }
}

} // namespace

int run_mlkem_hybrid_tests()
{
    g_failed = 0;
    printf("[MLKEM-TEST] === ML-KEM-768 hybrid KEX suite ===\n");
    fflush(stdout);
    test_combiner();
    test_chunking();
    printf("[MLKEM-TEST] === %d failure(s) ===\n", g_failed);
    fflush(stdout);
    return g_failed;
}
