# Data-flow: AEAD nonce (bsi-bound) — producer/consumer audit

Status: LIVE. Built 2026-06-28 alongside the bsi-bound nonce fix (branch
`staging/enc-bsi-nonce`), re-seal-generation fix added on `staging/enc-noncefix`
(§11). Pairs with `source/crypto/test_aead_nonce.cc` (`--test-aead-nonce`, also in
the master `--test` suite). Design gate: `_research/enc/NONCE_DESIGN.md`. Crypto
prior art: RFC 7905 (TLS ChaCha20-Poly1305), RFC 9001 §5.3/§5.4/§6.6 (QUIC),
RFC 7634 / RFC 4303 ESN (IPsec-ESP).

SECURITY INVARIANT (the reason this doc exists): NO ChaCha20-Poly1305
(key,nonce) pair is ever used with two different plaintexts — across batches,
sessions, directions, SACK reorder, retransmit, OR recovery re-seal. Mercury AEAD
is **IETF ChaCha20-Poly1305, 12-byte (96-bit) nonce** (`mercury_crypto.h:31`
NONCE_SIZE 12; `mercury_crypto.cc` `crypto_aead_init_ietf`). NOT XChaCha20/24-byte
— so the nonce MUST be a deterministic reconstructible function of wire-agreed
state (the RFC-7905/QUIC/ESP regime), not a 192-bit random.

--------------------------------------------------------------------------------
## §1. The shared state

Nonce layout (`build_nonce`, `mercury_crypto.cc`):
```
nonce[0..2] = 0x000000             reserved
nonce[3]    = direction byte       0x00 CMD->RSP, 0x01 RSP->CMD
nonce[4..11]= batch_index (BE u64) = fold_gen_index(gen, (epoch<<8)|wire_bsi)
```
`batch_index` is `fold_gen_index(gen, unwrap(wire_bsi))` (§11). The unwrap math is
`cl_cipher_suite::unwrap_batch_index(wire_bsi, &epoch, &last_bsi)`, an RFC-1982
forward-step (mirrors `advance_last_delivered`, `arq_common.cc`).

Per-controller state (`arq.h`):
- `tx_nonce_epoch` (u64), `tx_nonce_last_bsi` (int, -1 unset) — TX direction.
- `rx_nonce_epoch` (u64), `rx_nonce_last_bsi` (int, -1 unset) — RX direction.
- `tx_nonce_gen` / `rx_nonce_gen` (u64) — re-seal generation (§11).
- `tx_nonce_sealed_high_water` (u64, UINT64_MAX=unset) — highest sealed index (§11).
- `rx_nonce_adopted_once` (bool) — RX adopted an expected-bsi this session (§11).
- `decrypt_delivered_bsi` (int) — the wire bsi of the batch being delivered in
  `copy_data_to_buffer()`; set immediately before each call (§4).

--------------------------------------------------------------------------------
## §2. The bug this fixes (root cause)

The OLD nonce bound to LOCAL counters: `tx_batch_counter` (encrypt order) /
`rx_batch_counter` (delivery order), each `++` per call, with a rewind hack.
These count encrypt/delivery order, NOT wire order. SACK-v2 whole-batch ARQ
delivers out-of-order (prev/late) and retransmits, so the RX delivery-order
counter desyncs from the TX wire order → RX reconstructs the WRONG nonce → AEAD
tag fails → batch DROPPED → DISCONNECT. Symptom: ENCRYPTION 0/4 WGN:20 under SACK
partial-retx. LATENT HAZARD: if the two local counters ever realign onto the SAME
(key,nonce) with DIFFERENT plaintext → catastrophic keystream-XOR + Poly1305-key
leak. Overturns ENCRYPTION_PLAN.md:440-445.

--------------------------------------------------------------------------------
## §3. PRODUCERS (who writes the nonce state / supplies the bsi)

### TX (encrypt)
- `arq_commander.cc` process_buffer_data_commander, encrypt site: reads
  `cmd_batch_seq_id & 0xFF` (the CURRENT new-data wire bsi, pre-increment),
  feeds `unwrap_batch_index(...,&tx_nonce_epoch,&tx_nonce_last_bsi)`, folds
  `tx_nonce_gen` + applies the high-water guard (§11) → encrypt.
- SACK retx does NOT re-encrypt: `retransmit_frames[]` stores the ALREADY-
  ENCRYPTED frame bytes + original bsi; replayed verbatim. So a retx never
  re-enters the encrypt path → same bsi, same stored ciphertext, same nonce.
- Config-change/BREAK/demote rebuild: `restore_tx_from_compressed()`
  (`arq_common.cc`) re-queues PLAINTEXT and **bumps `tx_nonce_gen`** (§11) so the
  re-seal lands at a fresh, strictly-higher nonce index even at the same wire bsi.

### RX (decrypt) — `decrypt_delivered_bsi` set immediately before each
`copy_data_to_buffer()` call (the §4 source table). Inside the decrypt feeds
`fold_gen_index(rx_nonce_gen, unwrap_batch_index(decrypt_delivered_bsi,...))`.

### State init / reset (all epoch/last_bsi/gen/high_water/adopted_once)
- constructor/init: `arq_common.cc` (~:876)
- session reset `reset_session_state()`: `arq_common.cc` (~:7058)
- activate (CMD): `arq_commander.cc` KEY_ACTIVATE-ACKed (~:7010)
- activate (RSP): `arq_responder.cc` KEY_ACTIVATE-confirmed (~:3203)

--------------------------------------------------------------------------------
## §4. CONSUMERS — copy_data_to_buffer() call sites & the bsi each delivers

THE ORDERING TRAP: at the in-order BATCH-DONE site,
`rsp_current_expected_batch_seq_id` is ADVANCED BEFORE `copy_data_to_buffer()`.
So the DELIVERED batch's bsi is in `rsp_prev_batch_seq_id`, NOT current.

| copy_data_to_buffer() call site                  | delivered batch wire bsi (decrypt_delivered_bsi) |
|--------------------------------------------------|--------------------------------------------------|
| arq_responder.cc:2617 (in-order BATCH-DONE)      | rsp_prev_batch_seq_id (cur already advanced)     |
| arq_responder.cc:1296 (prev out-of-order)        | rsp_prev_batch_seq_id (prev being flushed)       |
| arq_common.cc:5055 (deliver complete prev pre-BREAK)| rsp_prev_batch_seq_id                          |
| arq_responder.cc:3445 (FILE_END_ flush)          | rsp_current_expected_batch_seq_id (NOT advanced) |
| arq_responder.cc:3457 (SWITCH_ROLE flush)        | rsp_current_expected_batch_seq_id (NOT advanced) |
| arq_responder.cc:10563, :12709 (test rigs)       | mirror production (inert: rigs don't activate enc)|

Decrypt consumer: `cl_arq_controller::copy_data_to_buffer()` (`arq_common.cc`),
decrypt call at the `cipher_suite.is_active() && assembled_size>0` branch.

--------------------------------------------------------------------------------
## §5. INVARIANTS the consumers assume, and why each holds

1. **Both peers compute the SAME index for a given wire bsi + generation.** Both
   observe the SAME committed wire-bsi order; `unwrap_batch_index` is a pure
   function of that stream; the generation is bumped on the SAME wire-observable
   config transition by both peers (§11). ✓
2. **Index strictly increasing over new-batch transmission** → distinct nonces.
   WRAP absorbed by epoch; recovery re-seal absorbed by the generation band (§11). ✓
3. **Direction byte disjoint** → CMD/RSP never collide at the same bsi. ✓
4. **Re-encrypt ⇒ NEW nonce** — ~~"rebuild re-queues fresh ⇒ NEW bsi ⇒ NEW
   nonce"~~ **✗ FALSIFIED (§10), NOW CLOSED by §11**: a recovery re-stamps the
   SAME wire bsi (must, for delivery contiguity), so uniqueness comes from the
   GENERATION + the TX high-water guard, not a fresh bsi. Re-seal of bsi=N lands
   in a strictly-higher disjoint index band; the guard enforces no-reuse
   unconditionally. Retx still replays stored ciphertext verbatim. ✓ (§7 cases 9/10/11)
5. **Re-key floor.** ChaCha20-Poly1305 IETF safe limit ≈ 2^32 msgs/key. HF batch
   rates make 2^32 unreachable; epoch+8-bit index + gen band give headroom. [?] A
   hard teardown at index==2^32 is recommended but not yet wired (unreachable in
   practice; add if a long-session re-key is introduced).

--------------------------------------------------------------------------------
## §6. copy_len LOW clamp (the run-over fix) — arq_common.cc DATA_LONG leg

`int copy_len = max_data_length+max_header_length-eff_hdr;` had only an UPPER
clamp. On a degenerate config eff_hdr can exceed it, making copy_len NEGATIVE →
corrupts the decrypt ciphertext length → guaranteed AEAD auth-fail + potential
OOB. FIX: `if(copy_len < 0) copy_len = 0;`. CONTROL/ACK legs UNCHANGED.

KX hardening: `compute_x25519_shared_checked(pubkey,len)` rejects NULL/short
(len < 32) BEFORE the 32-byte read.

--------------------------------------------------------------------------------
## §7. Tests (source/crypto/test_aead_nonce.cc; --test-aead-nonce)

1. unwrap monotone + unique over 1000 batches (>3 wraps).
2. 1200 nonces (2 dirs × 600 batches) all unique.
3. direction disjointness at the same bsi.
4. reorder(12,10,14,11,13)+retx(12) all decrypt — FAIL-BEFORE/PASS-AFTER.
5. truncated/NULL KX pubkey rejected; valid accepted.
6. clean round-trips AND tampered ciphertext rejected.
7. late prev 255 recovered after wrap decrypts at epoch-0 index.
8. encrypted-batch frame-span truncation → auth-fail (batch-span fix contract).
9. **BREAK-rebuild bsi RE-STAMP → distinct nonces, 0 reuse, RX decrypts re-seal** (§11).
10. **harsh SACK reorder(10)+retx → all decrypt, 0 reuse** (§11).
11. **>255 batch wrap WITH 5 interleaved BREAK re-stamps → all folded indices
    unique, 0 reuse, re-stamps decrypt** (§11).

--------------------------------------------------------------------------------
## §8. OPEN ITEMS
- [?] Re-key/teardown at index==2^32 not wired (unreachable at HF rates; §5.5).
- [DONE] bsi-nonce desync verified on fleet under WGN+SACK partial-retx (§9).
- [PENDING] faithful-fleet confirm of the §11 re-seal-generation fix under harsh
  BREAK churn + the §9 crypto-batch-size fix (both needed for `-E` delivery).

--------------------------------------------------------------------------------
## §9. FAITHFUL FLEET VERIFICATION (2026-06-28) — bsi-nonce fix CORRECT, but a
##      SEPARATE crypto-batch-size bug still blocks -E delivery under SACK

The bsi-nonce fix PROVED nonce desync ELIMINATED + SECURITY INVARIANT HOLDS (55
ENC emissions, 0 duplicate (dir,index)); no clean-channel throughput regression;
`--test-aead-nonce` exit 0. Remaining auth-fails were a SEPARATE pre-existing bug:
TX packs the encrypted batch to ~`data_batch_size` frames, overflowing the
negotiated `crypto_batch_size`; on a LOSSY batch the prev-completion gate delivers
ONE FRAME SHORT → whole-batch AEAD MAC over truncated ciphertext → auth-fail. The
§9 batch-size work (`staging/enc-batchsize`) addresses that; THIS §11 fix is
orthogonal (nonce correctness) and a prerequisite.

--------------------------------------------------------------------------------
## §10. CROSS-LAYER AUDIT — EVERY wire_bsi-stamping path vs the AEAD nonce
##      (2026-06-28, branch staging/enc-batchsize @6801b9cc; CLAUDE.md §1-5)
##      ⚠️ ROOT CLASS: the OLD AEAD nonce derived PURELY from `cmd_batch_seq_id`
##      via unwrap. ANY path that re-stamps an already-consumed bsi onto a NEW
##      plaintext WITHOUT advancing the unwrap state = same-nonce/diff-plaintext.

### §10.1 The single encrypt producer
Only ONE encrypt site exists: `arq_commander.cc` inside
`process_buffer_data_commander()`, reachable ONLY on the
`compression_viable_for_batch()` branch — ENCRYPTION IS COMPRESSION-PATH-ONLY.
The responder NEVER encrypts; only the CMD→RSP direction is sealed (the COMMANDER
decrypt path, rx_direction RSP_TO_CMD, is dormant).

### §10.2 The build↔send↔increment split (the structural root)
- SEAL/encrypt happens at BUILD time under the CURRENT `cmd_batch_seq_id`.
- `cmd_batch_seq_id` INCREMENTS only AFTER a completed `send_batch()`
  (`process_messages_tx_data`, :2464) AND only `if(batch_includes_new_data)`.
- A recovery between build and the increment that FREEs messages_tx[], re-queues
  plaintext, sets `block_under_tx=NO` WITHOUT advancing `cmd_batch_seq_id` re-opens
  the build to re-seal the SAME bsi over RE-COMPRESSED plaintext.

### §10.3 ENUMERATION — every bsi (re-)stamping path, REUSE verdict (PRE-§11)
| # | path / site | bsi action | reuse (pre-§11)? |
|---|-------------|------------|------------------|
| P1 | normal TX seal — :19331; incr :2464 | new bsi, +1 after send | NO — unique ✓ |
| P2 | SACK retx — `retransmit_frames[]` replay | replays STORED ciphertext | NO — verbatim ✓ |
| P3 | legacy BREAK ACK-recovery — restore_tx_from_compressed @ :574/:662 | re-seal SAME bsi over re-compressed plaintext | **YES — ⚠️ ROOT** |
| P4 | inband NOBREAK demote — :3344, rollback gated `!compression_enabled` | re-seal SAME bsi | **YES** |
| P5 | FIX-9 D3 reverse-ACK demote — :5318 | re-seal SAME bsi | **YES** |
| P6 | runaway-retx forced BREAK — :2086 | compression restore leg | **YES** |
| P7 | M6 BREAK lossless requeue — :5564 | gated, never fires on enc path | NO direct |
| P8 | climb-UP churn promote — :6130 | gated `!compression_enabled` | NO direct |
| P9 | per-frame in-flight rollback (!compression leg) | rolls bsi BACKWARD | LATENT (enc is compression-only) |
| P10 | config-rebuild / turboshift demote (SET_CONFIG :736) | routes through P3/P4/P5 | inherits **YES** |
| P11 | session reset — reset_session_state :7058 | full reset + fresh KEY | NO ✓ |
| P12 | KX / re-key — activate sites zero state | fresh key + zeroed state | NO ✓ |
| P13 | test-mode pre-init — readopt/decrypt rigs | hard-set | INERT (no enc) ✓ |

ALL of P3/P4/P5/P6/P10 are CLOSED by §11 (the generation bump in
restore_tx_from_compressed + the TX high-water guard). P9 stays LATENT-safe
(enc is compression-only; if enc ever moves to the per-frame path the guard still
prevents reuse, but a rollback-to-consumed-bsi would auth-fail until re-adopted).

### §10.4 The bind: WHY this is a genuine cross-layer trap (the DUAL)
The per-frame paths FIX RSP-V2-GAP-ABORT by ROLLING `cmd_batch_seq_id` BACK to
the in-flight bsi; that rollback on the encryption path would be an INSTANT
keystream reuse, so it is gated `!compression_enabled` — which means the
encryption path re-seals the UN-rolled (still-consumed) bsi ⇒ reuse anyway.
GAP-ABORT and nonce-reuse are DUAL: the wire bsi MUST stay at last_delivered+1 for
delivery contiguity (`delivery_step_is_gap` fires on a forward step ≥2), so you
cannot make the nonce unique by moving the bsi. §11 resolves it by decoupling the
nonce index from the wire bsi (a re-seal generation), keeping the wire bsi free to
roll back for delivery.

### §10.5 PRODUCER list delta to §3 (corrections)
- §3 "SACK retx does NOT re-encrypt" — CONFIRMED (P2). ✓
- §3 "Config-change rebuild re-queues FRESH new-data under a NEW
  cmd_batch_seq_id … NEW nonce" — WRONG (P3/P4/P5/P6); CLOSED by §11.
- §3 init/reset — CONFIRMED (P11/P12). ✓

--------------------------------------------------------------------------------
## §11. THE FIX — re-seal GENERATION + TX high-water guard (branch
##      staging/enc-noncefix, off staging/enc-batchsize @6801b9cc)
##      Closes the §10.3 P3/P4/P5/P6/P10 KEYSTREAM-REUSE class. CLAUDE.md §1-5.

### §11.1 Why the wire bsi alone cannot carry uniqueness (the dual, restated)
A recovery (BREAK-rebuild / config-rebuild / demote) re-seals over DIFFERENT
plaintext at a wire bsi that **must stay equal to the in-flight value** for RSP
delivery contiguity: the RSP delivery-time gate `delivery_step_is_gap(bsi,last)`
(arq.h:1648) tears the session down on a forward step ≥2, so the re-sent batch
has to land at `last_delivered+1` = the SAME bsi the aborted seal used. Therefore
the nonce index **cannot be a pure function of the wire bsi** — at one wire bsi we
need TWO distinct nonces (aborted seal vs re-seal). The OLD design (`unwrap(bsi)`
only) returned the SAME index → REUSE. Advancing the bsi instead would gap-abort
delivery (the §10.4 dual). RX reconstructs nonce from the wire bsi it receives, so
the extra discriminator must be reconstructable by RX from a wire-OBSERVABLE event.

### §11.2 The mechanism — a per-direction GENERATION folded into the index
Nonce index = `fold_gen_index(gen, unwrap_index) = gen*NONCE_GEN_STRIDE +
unwrap_index`, `NONCE_GEN_STRIDE = 2^48` (mercury_crypto.h). The per-session
re-key floor caps a session FAR below 2^32 batches (§5.5 / RFC 9001 §6.6), so the
unwrap index stays < 2^40 ≪ 2^48 → distinct generations occupy DISJOINT,
strictly-ordered index bands; a higher gen always yields a strictly-greater index.

- **TX bump** (producer): `restore_tx_from_compressed()` (arq_common.cc) bumps
  `tx_nonce_gen` whenever encryption is active — in BOTH the streaming and the
  non-streaming early-return branches. This is the single helper every recovery
  re-queue (P3/P4/P5/P6/P10) routes through, and it is 1:1 with the SET_CONFIG /
  config-transition emitted right after (BREAK retries re-send the pattern without
  re-calling restore, so a lost SET_CONFIG does NOT double-bump).
- **RX bump** (consumer-side producer): the FIX-8 re-adopt-from −1 site
  (arq_responder.cc, `rsp_current_expected_batch_seq_id = bsi`) bumps
  `rx_nonce_gen` iff `rx_nonce_adopted_once` (a genuine RE-adopt after a config
  transition, not the first-ever adopt). The transition that wiped expected to −1
  (BREAK / SET_CONFIG / CONFIG_TAG follow) is the SAME one that drove the TX
  restore → 1:1 across BREAK retries and multi-demote. RX re-adopts BEFORE the
  re-sent batch reaches `copy_data_to_buffer` (same RX tick path), so the gen is
  aligned at decrypt time.
- **Decrypt fold** (consumer): `copy_data_to_buffer()` (arq_common.cc) folds
  `rx_nonce_gen` into the decrypt index the same way the encrypt site folds
  `tx_nonce_gen`.

### §11.3 The HARD BACKSTOP — safety does NOT depend on gen alignment
At the encrypt site (arq_commander.cc), after folding the candidate index: if
`tx_nonce_sealed_high_water != UINT64_MAX && idx <= high_water` (the reuse
condition — this index was already sealed), bump `tx_nonce_gen` to one band past
the high-water and re-derive BEFORE sealing; then set `high_water = idx`.
**This makes no-reuse a TX-LOCAL invariant** that holds even if `rx_nonce_gen`
ever drifts. A gen drift then only makes RX reconstruct a wrong index → AEAD
auth-fail (a safe drop / session reset; the CMD re-sends under a fresh session,
non-lossy at the system level) — it can NEVER produce a same-nonce/different-
plaintext. RFC-4303-ESN / RFC-9001-§5.4 regime: implicit high-order sequence bits
the receiver reconstructs; a wrong guess fails the tag, it never weakens the
cipher. The §10.4 dual is resolved: the wire bsi is free to roll back for delivery
(gap-gate happy) while the gen advances for crypto.

### §11.4 Retx is still verbatim replay (unchanged, P2). New plaintext NEVER
reuses (P3/P4/P5/P6/P10 now bump the gen + the high-water guard). Reset/KX
(P11/P12) zero gen/high-water/adopted_once at all 3 sites (constructor,
reset_session_state, both activate sites). Test rigs (P13) stay inert.

### §11.5 §5 invariant-4 — NOW CLOSED (see §5.4 above).

### §11.6 §5 consumer re-walk (CLAUDE.md §5 — every consumer still reconstructs)
- In-order BATCH-DONE decrypt (arq_responder.cc:2617 → copy_data_to_buffer):
  reads `rx_nonce_gen` set by the same-tick re-adopt; aligned. ✓
- prev out-of-order / pre-BREAK prev flush (arq_responder.cc:1296,
  arq_common.cc:5055): a recovered prev is ALWAYS same-generation as current — a
  config transition FREEs+clears all in-flight + retx state (restore +
  clear_retx_queue), so no cross-gen prev survives. The pre-BREAK prev flush runs
  BEFORE the reseed/re-adopt, so it decrypts at the OLD (correct) gen. ✓
- FILE_END / SWITCH_ROLE flush (arq_responder.cc:3445/3457): end-of-transfer
  current batch, no transition, same gen. ✓
- COMMANDER-side decrypt (rx_direction RSP_TO_CMD) is DORMANT: encryption is
  emitted only at process_buffer_data_commander (§10.1) so only CMD→RSP is sealed;
  the RESPONDER decrypt path (where the gen bump lives) is the only live consumer. ✓

### §11.7 OPEN / residual (honest)
- Liveness (NOT safety) under a pathological transition mis-count: if a config
  transition ever bumps TX gen but RX never re-adopts (or vice-versa), the
  re-sealed batch auth-fails → session reset (recoverable, non-lossy). Safety is
  unaffected (high-water guard). The 1:1 argument (§11.2) makes this unreachable on
  the enumerated paths; flagged for the faithful-fleet confirm (§8).
- `-E` remains gated on the SEPARATE §9 crypto-batch-size short-deliver work; this
  fix is orthogonal (nonce correctness) and a prerequisite for it.
