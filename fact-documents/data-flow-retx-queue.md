# Data-Flow Audit: TX Retx Queue + HARQ Chase-Combining LLR Buffer

**Status**: Authoritative as of 2026-07-01. Built on `monitor` (720d4bcc) for the
`feat/harq-chase-combining` change. This document is the CLAUDE.md §"Cross-Layer
Data-Flow Audits" gate for the HARQ RX-side LLR buffer (§4 = the mandatory 5-question
audit). It also serves as the canonical producers/consumers/invariants record for the
TX retransmit queue (§2), which the companion `data-flow-arq-recovery-cluster.md`
already cross-references as a stub. Every change to the structures in §2 or §3 MUST
update this document.

**Line-number note**: all `file:line` below were located against the live worktree
(monitor 720d4bcc + this branch's WIP). `data-flow-arq-recovery-cluster.md` cites some
of the same TX arrays under DIFFERENT line numbers (`arq.h:1388` there vs `arq.h:3105`
here) because that doc was built against an older tree (2026-06-06). Where they differ,
verify against the current tree; the arrays and semantics are identical.

---

## §1 Why these two structures are ONE audit

Chase combining (Chase, "Code Combining", IEEE Trans. Comm. COM-33, 1985; = Type-I HARQ
with soft/packet combining) sums the channel LLRs of two receptions of the SAME
transmitted codeword before LDPC decode. Summation is the ML-optimal equal-noise combine
and doubles effective SNR (~3 dB, more near the waterfall cliff).

The combine is only mathematically valid if the two receptions carry **bit-identical
coded symbols**. Mercury has no rate-compatible/incremental-redundancy machinery — it
retransmits **whole frames**. Whether a retransmit reproduces the original coded bits is
therefore decided ENTIRELY by the TX retransmit queue (§2). The RX LLR buffer (§3) is a
new physical-layer structure that *exploits* that identity. The two never share memory,
but the RX buffer's correctness depends on the TX queue's identity guarantee — hence one
audit.

---

## §2 TX retransmit queue — the identity source (producers / consumers / invariants)

### 2.1 Declarations — `cl_arq_controller` (`include/datalink_layer/arq.h`)
```
arq.h:3103  int retransmit_count;                                       // # frames queued
arq.h:3105  unsigned char retransmit_frames[MAX_RETRANSMIT_HEADROOM][MAX_SACK_FRAME_SIZE];
arq.h:3123  int retransmit_frame_batch_seq_ids[MAX_RETRANSMIT_HEADROOM]; // orig bsi (SACK-A Step 3)
arq.h:3135  unsigned char retransmit_frame_seq_with_eob[MAX_RETRANSMIT_HEADROOM]; // orig seq|EOB
            // + parallel: retransmit_frame_lengths / _positions / _types
```

### 2.2 Producer — enqueue a missed frame (`source/datalink_layer/arq_commander.cc`)
```
arq_commander.cc:4489  // "Frame missing — save encrypted payload for retransmit."
arq_commander.cc:4518  memcpy(retransmit_frames[retransmit_count], messages_tx[i].data, len);
arq_commander.cc:4527  retransmit_frame_batch_seq_ids[retransmit_count] = messages_tx[i].batch_seq_id;
arq_commander.cc:4534  retransmit_frame_seq_with_eob[retransmit_count]  = messages_tx[i].sequence_number;
```
The retx is a **verbatim `memcpy` of the stored payload** with the **ORIGINAL**
`batch_seq_id` and `sequence_number|EOB` resurrected. The stored payload is the
**already-encrypted** ciphertext (arq_commander.cc:4489 "save encrypted payload";
:2198 "the retx queue's contents (encrypted under the old config's ...)"). So the retx
re-serializes byte-identical header+payload ⇒ identical LDPC-coded bits, **including when
encryption is ON** (Mercury does NOT re-encrypt with a fresh nonce on retx — it resends
the ciphertext). This is the load-bearing premise for §3.

Empty/padding frames are ACKed-and-skipped, never enqueued (arq_commander.cc:~4508:
`length==0 || batch_seq_id<0 || type==NONE`), so no zero-length pseudo-frame ever enters
the queue.

### 2.3 Consumer / invalidator — `clear_retx_queue()`
Called on recovery + config-DEMOTE paths (arq_commander.cc:677, 765, 3490, 4975, 5313).
Recovery **re-queues PLAINTEXT** ("drop stale retx"). This is the ONE case where a later
reception of a "retx" is a DIFFERENT codeword than a buffered failure (plaintext vs the
old ciphertext, and/or a new config). It is CRC-safe by construction (§4 Q4-c) and the
config-change subset is also explicitly invalidated at the PHY layer (§3, `harq_reset`).

### 2.4 Invariant this whole feature relies on
> A frame that is retransmitted at the SAME config produces coded symbols bit-identical
> to its first transmission.
Verified: §2.2 (verbatim ciphertext + original identity bytes). Broken only by a config
change (different generator/constellation/interleave) or a recovery plaintext re-queue —
both handled below.

---

## §3 HARQ chase-combining LLR buffer — the new RX-PHY structure

### 3.1 Declarations — `cl_telecom_system` (`include/physical_layer/telecom_system.h`)
```
telecom_system.h:914  struct st_harq_entry { int cfg; int N; unsigned long long age; std::vector<float> llr; };
telecom_system.h:920  std::vector<st_harq_entry> harq_buf;       // bounded ring (HARQ_BUF_MAX=6)
telecom_system.h:922  unsigned long long harq_rx_counter;        // monotonic receive_byte index (age)
telecom_system.h:926  std::vector<float> harq_snap_llr;          // per-reception best-failure snapshot
telecom_system.h:934  int harq_chase_enabled = -2;               // env gate cache (default ON)
```
Keyed by **(cfg, N)**, NOT `(batch_seq_id, frame-index)` as the task brief proposed. The
ARQ header (`batch_seq_id`/`sequence_number`) lives INSIDE the LDPC-protected payload and
is parsed only AFTER a clean decode — on a CRC-FAIL those bytes are corrupt and the
`(bsi, idx)` key is **unreadable**. `(cfg, N)` is the strongest identity available on a
corrupt frame (it fixes modulation/interleave/dispersal/codeword geometry, the summation-
validity guard); CRC then ARBITRATES which buffered copy is the same codeword. This
realizes the task's `(bsi, idx)` *intent* via the association available at fail-time.

### 3.2 Producers (writers)
```
telecom_system.cc:1125  receive_byte top — reset harq_snap_valid/metric; ++harq_rx_counter (EVERY call)
telecom_system.cc:3350  fail branch — capture harq_snap_llr = deinterleaved_data[0..N) of the best
                        (highest coarse_metric) REAL-looking OFDM failure (gate: feature_on && !MFSK
                        && all_zeros==NO && mean_H>=0.5 && N>0)
telecom_system.cc:3949  harq_bank_snapshot() — push snapshot into harq_buf (age-evict >24, drop-oldest at 6)
telecom_system.cc:3834  harq_reset() — clear harq_buf + snapshot
telecom_system.cc:10906 load_configuration() — harq_reset() on any real config change
```
The captured vector is `deinterleaved_data[0..ldpc.N)` taken AFTER the parity/virtual
dispersal fixup (telecom_system.cc:3364-3374) — i.e. the EXACT vector fed to the failed
`ldpc.decode` (telecom_system.cc:~3210). Both banked and fresh snapshots share this
post-dispersal layout, so element-wise summation is valid.

### 3.3 Consumers (readers)
```
telecom_system.cc:3763  post-loop — if(message_decoded != YES && snap valid) harq_combine_rescue(...) else bank
telecom_system.cc:3866  harq_combine_rescue() — loop compatible harq_buf entries (capped, §4 Q5),
                        harq_sum_and_decode() then production CRC self-check; on CRC-clean publish to `out`+rs
telecom_system.cc:3847  harq_sum_and_decode() — clip-sum(±40, matches :3133 demod rail) + ldpc.decode(float*)
telecom_system.cc:3818  harq_feature_on() — cached MERCURY_HARQ_CHASE (default ON; 0/n/f = off)
```

---

## §4 The mandatory 5-question cross-layer audit (CLAUDE.md gate)

**Q1 — Producers.** §3.2. All are inside `cl_telecom_system` (physical layer). None
writes to `retransmit_frames[]`/`messages_tx`/`messages_rx`. The only shared coupling with
§2 is *semantic* (§2 makes the coded bits identical; §3 assumes it), never memory.

**Q2 — Consumers.** §3.3, plus the DOWNSTREAM datalink consumer of `out`/`receive_stats`:
`process_...` in the ARQ layer reads the delivered bytes into `messages_rx`. It is
oblivious to HOW a frame decoded — `harq_combine_rescue` fills `out` and every
`receive_stats` field EXACTLY like the loop success path (telecom_system.cc:3866-3945:
`message_decoded=YES`, `all_zeros=NO`, `crc=0`, `iterations_done`, `SNR`, `delay*`,
`freq_offset*`), so `messages_rx` sees a normal delivery.

**Q3 — Valid states.**
- `harq_buf` = ∅ : before any failure, after `harq_reset` (config/session change), after a
  delivered combine erases its entry (telecom_system.cc:~3944), after age/size eviction.
- `harq_buf` = 1..6 entries, each `{cfg, N, age, llr(size==N)}`.
- Per-reception snapshot: `harq_snap_valid` true only between the fail-branch capture and
  the post-loop consume; reset to false at each receive_byte top (:1125).
- Default-init (before any producer): `harq_buf` empty, `harq_snap_valid=false`,
  `harq_rx_counter=0`, `harq_chase_enabled=-2` (unread). All benign — no consumer reads
  `harq_buf`/`harq_snap_*` until a producer has written them (guarded by `harq_snap_valid`
  and the `size()`/`(cfg,N)` checks).

**Q4 — Invariants consumers assume, and why each holds.**
- (a) `entry.llr.size() == entry.N` and `snap.size() >= N`: enforced at bank
  (telecom_system.cc:3949 assigns exactly N) and re-checked in the rescue loop
  (`(int)e.llr.size()!=N → continue`, `snap_llr.size()<N → return`).
- (b) Summation is only valid for the SAME codeword geometry: enforced by the `(cfg,N)`
  key match before any combine; `harq_reset()` on config change (telecom_system.cc:10906)
  guarantees no cross-config entry survives.
- (c) A pairing is NEVER trusted — CRC16 arbitrates. The rescue mirrors the production
  post-decode self-check (bit_energy_dispersal → bit_to_byte → all_zeros → CRC16, the same
  logic at telecom_system.cc:~3279) and rejects a non-zero CRC (`continue`). This is the
  SAME wrong-codeword backstop the normal decode already relies on
  (telecom_system.cc:~3302 "Check on ALL frames ... to catch wrong-codeword convergence").
  Mis-association can therefore never deliver corrupt data — proven by
  `--test-harq-chase` WRONG-COMBINE case.
- (d) MONOTONE: `harq_combine_rescue` is reached ONLY when `message_decoded != YES`
  (telecom_system.cc:3760). A clean first-pass decode never enters it ⇒ the success path is
  byte-identical; the feature can only PROMOTE a CRC-fail to a CRC-verified success. Gated
  OFF (`MERCURY_HARQ_CHASE=0`) ⇒ the whole block is skipped and output is byte-identical.

**Q5 — What the fix changes, walked per consumer.**
The fix adds RX-PHY state and promotes some CRC-fail receptions to CRC-verified successes.
- `messages_rx` / datalink ARQ: unaffected — sees a normal delivery (Q2). A promoted frame
  ACKs like any decoded frame, retiring its `messages_tx` slot and its `retransmit_frames`
  entry via the normal SACK path; no new write to §2 state.
- Rate adaptation (SNR-driven climb): `harq_combine_rescue` deliberately reports the
  **conservative single-frame SNR** `10log10(1/variance)` (telecom_system.cc:~3928), NOT
  the ~3 dB-higher combined SNR, so the optimizer sees the TRUE marginal channel and does
  not over-climb on a frame that only decoded because of combining. (Avoids a demote-
  amplifier of a different flavor.)
- STALE-CFO scrub: on a combine success we reset `consecutive_ofdm_decode_fails=0`
  (telecom_system.cc:~3767) to mirror the normal success branch, keeping the scrub armed.
- **NEW cross-layer risk (timing layer):** each compatible candidate costs one FULL
  `ldpc.decode`, and a WRONG pair does not converge ⇒ runs to `nIteration_max` (the most
  expensive decode). This is on the RX turnaround path; on the Pi that budget is tight, so
  an unbounded loop over a full ring could blow reverse-ACK timing and manufacture the very
  DEMOTE cascade HARQ exists to prevent. **Constrained the producer** (CLAUDE.md
  resolution): (1) buffer bounded to 6 with age-evict; (2) `harq_reset` on config change
  keeps the compatible set small; (3) combine attempts CAPPED
  (`MERCURY_HARQ_MAX_ATTEMPTS`, default 3) at telecom_system.cc:~3874 — a full-frame retx
  almost always pairs with the most-recent failure, and the cap is monotone-safe (fewer
  attempts can only DECLINE a rescue, never fabricate — CRC still gates every accept).

**Resolution:** every consumer's assumption holds; the one new interaction (timing) is
constrained at the producer. No consumer requires a change.

---

## §5 Test coverage

`--test-harq-chase` (source/physical_layer/test_harq_chase.cc, main.cc `--test-harq-chase`)
drives the EXACT production primitive `harq_sum_and_decode` on synthetic AWGN-BPSK
codewords with KNOWN info bits (deterministic xorshift+Box-Muller, no IONOS/RF):
- FAIL-BEFORE: at the CFG10 cliff (sigma=1.10) a single reception decodes only 3/12.
- PASS-AFTER: summing two independent receptions of the same codeword decodes 12/12
  (the ~3 dB combine gain).
- WRONG-COMBINE SAFE: summing codeword A with a DIFFERENT codeword B does not fabricate A.
Full `mercury.exe --test` = 47 OK / 0 FAIL with the feature default-ON (no regression;
HARQ never fires in the unit suite since it has no failed-then-retx OFDM receptions).

## §6 Open questions / limitations [?]

- [?] **HW/full-ARQ confirmation pending.** Proven at the LDPC primitive level + monotone-
  safe by construction, but not yet exercised through a 2-process ARQ climb on the fleet
  (CONNECT gap) or on HW. Default-ON is justified by construction (only promotes fails,
  CRC-gated); `MERCURY_HARQ_CHASE=0` is the A/B kill switch.
- [?] **Recovery plaintext re-queue (§2.3).** A buffered encrypted-payload failure will not
  combine with a plaintext recovery re-queue of the "same" frame (different bytes) — it is
  CRC-safely discarded, i.e. HARQ simply does not HELP there (no harm). Not currently
  special-cased beyond CRC arbitration; acceptable.
- [?] **Cross-reception LLR independence.** The ~3 dB assumes the two receptions' noise is
  independent. Deep correlated fades (same notch on both copies) yield less than 3 dB; the
  combine still never HURTS (CRC-gated), it just may not rescue. A decorrelation gate
  (reject combining two copies whose channel signatures are near-identical) is a possible
  future refinement (cf. `feat/chase-combine` I4's F4 gate).
