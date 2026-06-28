# Data-Flow Audit — Encrypted batch size vs negotiated crypto_batch_size

Shared state: the encrypted AEAD unit (whole-batch ciphertext) and the frame
count that bounds it. Crosses 3 layers: ARQ batch sizing (PHY/ARQ) ↔ batch
completion gate (SACK) ↔ AEAD whole-batch MAC (Compression/crypto).

Built 2026-06-28 for the TRACK-A enc-batchsize fix (the 2nd encryption bug,
sibling of the nonce-vs-wire-bsi fix 85daa0be). CLAUDE.md §5 audit.

---

## §1 Symptom (verified live, prompt)

Encrypted session under harsh SACK loss (WGN:15/20): `-E` delivers 0/8.
On a lossy batch the RX prev-completion gate fires one frame short
→ `copy_data_to_buffer()` reassembles N-1 frames → the whole-batch AEAD MAC
over a truncated ciphertext fails → `[CRYPTO] Decrypt FAILED … PSK MISMATCH`
→ `link_status = DROPPED` (arq_common.cc:13777). The session disconnects.

## §2 The AEAD whole-batch MAC invariant (crypto/mercury_crypto.cc)

`cl_cipher_suite::encrypt()` (mercury_crypto.cc:527) seals ONE contiguous
plaintext into ONE ciphertext + 16-byte Poly1305 tag. The MAC
(`aead_compute_mac`, :493) covers `pad16(ct) || aad_len(0) || ct_len`
(RFC 8439 §2.8) — **the ciphertext LENGTH is a MAC input**. Therefore the
ciphertext handed to `decrypt()` MUST be byte-for-byte and length-for-length
identical to what `encrypt()` produced. Any truncation (one missing frame)
changes both the bytes AND `ct_len` → guaranteed `crypto_aead_read` rc!=0
→ auth-fail. There is no partial-decrypt: the AEAD unit is atomic.

INVARIANT-MAC: the set of DATA frames reassembled at RX before `decrypt()`
== the exact set of DATA frames whose payload `encrypt()` sealed at TX.

## §3 Root cause — TX over-sizes the AEAD unit; crypto_batch_size is dead

`crypto_batch_size` (default 20) is set in `reset()` (arq_common.cc:677) and
once from CLI (main.cc:4509/4517) as `radio_batch_size - retransmit_headroom`
(25 - 5). It is the NEGOTIATED size of the AEAD unit: the count of frames the
encrypted batch may span, leaving `retransmit_headroom` (5) frames of the
radio batch (`radio_batch_size`, 25) free for SACK retransmissions.

BUG: `crypto_batch_size` is never consumed by the sizing path (grep: only a
printf at arq_common.cc:1620 + the main.cc set). The TX instead sizes the
encrypted batch by `data_batch_size` (the live radio batch, 25 at OFDM):

- arq_commander.cc:19098 — `batch_capacity = data_batch_size * max_frame`
  (the compression target). Compression fills toward this, so `comp_size`
  can approach `data_batch_size * max_frame - AUTH_TAG_SIZE`.
- arq_commander.cc:19277 — `target_comp = data_batch_size * max_frame - tag`
  (the post-compress zero-pad target). Pads the plaintext UP to fill all
  `data_batch_size` frames.

Result: the AEAD plaintext spans up to `data_batch_size` (25, ~24 after tag)
frames — PAST the negotiated `crypto_batch_size` (20). The radio batch is then
fully consumed by the AEAD unit; there is no retransmit headroom, and the
whole AEAD unit must arrive intact or the MAC fails.

## §4 Why loss → short delivery → auth-fail (the prev-batch path)

RX reassembly + decrypt: `copy_data_to_buffer()` (arq_common.cc:13670)
reassembles ACKED slots `[0, data_batch_size)` into `assembled`, then
`decrypt()`s the whole buffer (:13741). The prev-batch (SACK-recovery)
completion gate fires at arq_responder.cc:1186-1187 when
`rsp_prev_batch_received_count >= rsp_prev_batch_expected_count`.

`rsp_prev_batch_expected_count` is derived (arq_common.cc:9790-9804) from
`rx_batch_total_frames` (the WIRED D5 count, authoritative) when available,
else `last_received_end_of_batch_seq + 1` (EOB inference, SHORT on EOB-loss).
When the AEAD unit spans 24 frames but the gate's `expected` collapses to 23,
the gate fires with 23 frames → reassembly truncates the ciphertext by one
frame (~92 B) → §2 INVARIANT-MAC violated → auth-fail → disconnect.

The fragility scales with batch size: a 25-frame AEAD unit needs all 25 to
land before SACK headroom can even help, and any expected-count miss is fatal
to the whole batch (not just the lost frame). Bounding the AEAD unit to the
negotiated 20 restores 5 frames of headroom so SACK retx ALWAYS has room and
the batch reliably completes inside it.

## §5 Cross-layer audit (CLAUDE.md §5)

State changed: the AEAD unit size (frame span of the encrypted batch).

### Producers (writers of the AEAD unit / its frame count)
- arq_commander.cc:19098 — `batch_capacity` (compression target). PRIMARY.
- arq_commander.cc:19277 — `target_comp` (zero-pad-up target). PRIMARY.
- arq_commander.cc:19306 — `cipher_suite.encrypt()` produces the sealed unit.
- arq_commander.cc:19336-19357 — frame split: ceil(comp_size/max_frame) frames.
- arq_common.cc:8892-8898 — D5 `batch_total_frames_wire = message_batch_counter_tx`
  (the TRUE count of frames packed = the AEAD unit's frame span after fix).

### Consumers (readers that must agree on the AEAD unit)
- arq_common.cc:13670 `copy_data_to_buffer()` — reassembles `[0,data_batch_size)`
  ACKED slots → `assembled`, then `decrypt()` (:13741). Reassembles WHATEVER
  is ACKED; correctness depends on the gate delivering the COMPLETE set.
- arq_responder.cc:1186 — prev-batch completion gate (received >= expected).
- arq_responder.cc:2151/2577 — current-batch SACK-partial gate / BATCH-DONE
  delivery (only decrypts on full receipt).
- arq_common.cc:9790 / arq_responder.cc:1144 / :2111 — `expected` derivation
  from wired D5 (`rx_batch_total_frames`) or EOB inference.
- arq_common.cc:12962/13025 — RX parse of the D5 byte → `rx_buffer_batch_total_frames`.

### Valid states (before any producer writes)
- `data_batch_size` ∈ [1, ~hundreds]; runtime values: 1 (robust pin),
  10 (Axis-2 floor), 25 (radio default), K (big-block), 30 (test).
- `crypto_batch_size` = 20 CONSTANT (never mutated at runtime; symmetric on
  both peers from identical defaults / identical CLI).
- `cipher_suite.is_active()` true only post-turboshift+KX (OFDM rungs); robust
  (batch=1) never encrypts a multi-frame batch.

### Invariants consumers assume
- INVARIANT-MAC (§2): RX-reassembled frame set == TX-sealed frame set, exactly.
- The gate's `expected` == the true AEAD-unit frame span, so a complete gate
  fire reassembles the full ciphertext.
- D5 wired count (`message_batch_counter_tx`) == the AEAD-unit frame span.

### What the fix changes
- Bound the AEAD-unit frame span to `min(data_batch_size, crypto_batch_size)`
  when encryption is active. Concretely: at arq_commander.cc:19098 cap
  `batch_capacity` to `crypto_frames * max_frame` (which also bounds :19277
  since `target_comp <= batch_capacity`). The split then produces
  `<= crypto_frames` frames; D5 wired count = that span; `retransmit_headroom`
  slots stay free for SACK retx.
- Walk each consumer:
  - `copy_data_to_buffer` reassembles `[0, data_batch_size)` — still correct;
    it just sees `<= crypto_frames` ACKED slots (the rest FREE).
  - prev/current gates: `expected` = wired D5 count = the bounded span; SACK
    retx now ALWAYS has headroom, so the batch completes inside it; reassembly
    is byte-exact → MAC passes.
  - D5 parse: unchanged; the wired count is just smaller (≤20).
- No RX change required: the fix is purely TX-side bounding of the producer.
  Both peers hold the same `crypto_batch_size` constant, so no wire negotiation
  is added.

### Nonce-uniqueness (the SECURITY invariant — MUST hold)
- Unchanged. The nonce binds to the WIRE batch_seq_id via
  `unwrap_batch_index` (mercury_crypto.cc:443); one (dir,index) per batch.
  Bounding the batch FRAME span does not change the per-batch bsi or the
  nonce derivation — each batch still gets exactly one fresh (dir,index).
  A SACK retx replays STORED ciphertext (does not re-enter encrypt). A
  config-change rebuild re-queues as fresh new-data with a NEW bsi. No
  same-nonce/different-plaintext path is introduced.

## §5.1 Residual found in sim → fix (2) added (consumer fix)

The TX bound (fix 1) alone left a residual: on a lossy encrypted batch whose
EVERY D5-bearing frame was lost, `rx_batch_total_frames` stays -1, so the
completion gate / receiving-window timer fell back to the EOB inference
(`last_received_end_of_batch_seq + 1`), which collapses `expected` BELOW the
true (bounded) span. Sim (WGN:24/26 over -x sim, -E): prev bsi=3 sealed at
20 frames but `expected`=16 → gate fired at 16/16 → decrypt of 16 frames
(2672 B) of a 20-frame (3324 B) AEAD unit → 1 auth-fail + disconnect per run
(link recovered, but the disconnect is the bug).

Fix (2) — constrain the consumer (CLAUDE.md §5 "fix the consumer OR the
producer"; here BOTH, because the EOB inference is structurally unsafe for an
encrypted unit): when `cipher_suite.is_active()` AND the wired D5 count is
unavailable, size `expected` / `effective_batch` / `prev_expected` to the
DETERMINISTIC crypto span `min(data_batch_size, crypto_batch_size)` (the exact
TX seal bound, computed identically on both peers from shared constants) —
NEVER the short EOB inference. The gate then HOLDS (received < expected) and
SACK recovers the missing tail until the full decryptable set lands. Three
sites, all guarded `cipher_suite.is_active()`, all keeping the non-encrypted
path byte-identical:
- arq_common.cc bump_bsi_and_transfer_prev() — `prev_expected`.
- arq_responder.cc process_messages_acknowledging_data() — pattern-ACK gate
  `expected`.
- arq_responder.cc receiving-window timer — `effective_batch` (must not be
  LOWERED by EOB for an encrypted batch).
(The default-off MERCURY_SPEC_SACK speculative gate at arq_responder.cc:708
already pins to data_batch_size on missing EOB; untouched.)

Sim result after fix (2): 8/8 harsh-SACK encrypted runs (WGN:24/26 × 4 seeds)
deliver byte-exact (md5_match=True), 0 AEAD auth-fail, 0 disconnect; clean -E
(WGN:38) no-regress (md5_match=True); NONCE-TRACE shows all ENC (dir,idx)
unique under SACK retx + out-of-order prev recovery (no reuse).

## §6 Regression test

`test_enc_batch_size.cc` (crypto layer): drives `encrypt()` over a 24-frame-
equivalent plaintext, then `decrypt()` over the FULL ciphertext (passes) and
over a one-frame-TRUNCATED ciphertext (must auth-FAIL, capturing the §2/§4
mechanism). Plus the ARQ-level invariant: with the fix, the frame split of an
encryption-active batch yields <= crypto_batch_size frames. Nonce-uniqueness:
asserts distinct (dir,index) across batches (reuses test_aead_nonce harness).
