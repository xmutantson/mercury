# Data flow: RX delivery/commit funnel and authenticated-origin adoption

Cross-layer data-flow audit of the responder (RX) ordered-delivery and byte-commit
funnel, and the change needed to make the offset-0 head-skip delivery defect
unrepresentable by binding the delivered-stream origin to an authenticated stream
descriptor via the shared delivery-correctness core (`mercury::fade`, `fade_core.h`).

Base: monitor tip `0c57fc1a1a` (equivalently branch `feat/fadecore-rx-adopt`, whose
only additions vs base are the shared-core files + their test/dispatch wiring — the
RX delivery/KX code audited here is byte-identical to base). Risk tier: **A** — this
touches shared cross-layer delivery state and the integrity/commit path, and changes
a correctness-behavior default.

Cite convention: file:line verified by inspection at the base above. Header-decl and
predicate cites in `arq.h` were opened directly; `.cc` definition-body line numbers
were localized during this audit and MUST be re-confirmed with `tools/cite_check.sh`
before any edit lands (they drift with unrelated commits). Open items marked `[?]`.

---

## The defect (what this audit exists to close)

On a completed transfer the receiver can deliver a batch's payload at delivered-stream
offset 0 that does not belong there, producing a fixed-length head-skip: the first
N bytes of the application stream are missing and every later byte is shifted earlier
by N. Observed signature: a K=24-byte head-skip (K = batch-0 payload at the CONFIG_0
crawl, `data_batch_size=6`, 4 B/frame), byte-identical across repeated runs on the
same seed. Two distinct protocol-race paths reach the identical corruption:

- **PATH-A** — the origin batch (batch 0) IS adopted but is sealed into the "prev"
  slot with a hole (partial receipt), and the next batch (batch 1) completes and is
  delivered first. The delivery-step gate treats batch 1 as a legal first delivery at
  offset 0 because nothing has been delivered yet.
- **PATH-B** — the origin batch (batch 0) is wholly lost on air and never adopted; the
  receiver first-adopts batch 1, and again delivers it at offset 0 as a legal first
  delivery. (A prior narrow fix that latched the origin to the *first adopted* batch
  index does not close this path — it binds the origin to batch 1, the wrong batch.)

Both paths exploit the SAME invariant defect: **"the first delivery is always legal at
offset 0, whatever batch it is."** That is only true if the first delivered batch is
the stream origin. The correct invariant is: a first delivery is legal at offset 0
only if the batch is the stream's authenticated origin (the batch that genuinely
belongs at offset 0).

loss_class: **STRUCTURAL** — reproduces deterministically in-vehicle on a clean
decision path (a state-machine ordering defect), not a channel error. Per the
loss-class rule, the fix is PREVENTION by construction (refuse the mis-ordered commit),
not recovery/retransmission.

### Diagnostic discipline (cleared before booking the root)
- **Determinism test:** the corruption is byte-identical across repeated same-seed runs
  (K=24 every time). A race at the commit instant would predict run-to-run variance;
  N byte-identical repetitions instead indicate a STEADY-STATE decision defect — the
  gate deterministically returns "legal" for the wrong first batch. This matches the
  proposed root exactly.
- **Steady-state vs transition-instant:** the mis-ordered first delivery is a
  steady-state property of `delivery_step_is_gap` when `last_delivered_bsi < 0`; it
  does not require re-triggering any adopt/cross transition. Adopt-instant race
  mechanisms are therefore excluded as the root.
- **Sample-fate trace:** the lost unit is batch 0's payload. It is never committed to
  the app FIFO before batch 1 is; batch 1's bytes are pushed at cursor 0
  (`fifo_push_rx`, the append), so the delivered stream starts at batch 1. Batch 0,
  when/if it later completes, is dropped as a late/older prev. The bytes die at the
  commit chokepoint (`copy_data_to_buffer`), not upstream in decode/capture.

---

## Q1 — Producers (writers of the delivery-ordering / origin / commit state)

- `rsp_current_expected_batch_seq_id` (the batch under delivery) — first-adopt latch,
  `arq_responder.cc:1241` (binds it to the first adopted batch seq id; nothing on the
  wire proves that batch is the origin). Also advanced by the batch-done / bump path.
- `rsp_last_delivered_batch_seq_id` — set by `advance_last_delivered(int)`
  (`arq_common.cc:15675`; first-set `:15680`, forward-only `:15687-15688`). Callers:
  `arq_responder.cc:2618`, `arq_responder.cc:1580`, `arq_common.cc:6921`.
- `rx_stream_delivered` — the absolute app-FIFO append cursor; advanced at
  copy-data-done inside `copy_data_to_buffer()` (`arq_common.cc:22055` funnel; byte
  push `fifo_push_rx` `[?]~22503`). Re-anchored to 0 on KEY_ACTIVATE
  (`arq_common.cc:10098-10146`).
- `rx_stream_stamp[256]` positional descriptors — parsed by `w_parse_eob_stamp(int)`
  (`arq_common.cc:21898`; `.start/.length/.valid` set `:21916-21918`). `StreamStamp`
  decl `arq.h:4440`; array `arq.h:4470`. `.start` is the ABSOLUTE delivered-stream
  byte offset the batch's payload belongs at; `.length` its size. `.crc` is DECLARED
  but never populated or checked by the parser — the stamp is protected by the frame
  CRC only, NOT by the AEAD MAC, and the parse fails OPEN on an absent stamp.
- Per-DATA-frame identity on the wire: 8-bit `batch_seq_id`
  (`message_TxRx_byte_buffer[2]` low 7 bits) + EOB flag (bit 7). PLAINTEXT header;
  used only to build the AEAD nonce index, NOT as AEAD associated-data.
- Session keys / identity: `derive_session_key(...)` (`arq_common.cc:10274` responder,
  `:10312` commander; decl `mercury_crypto.h:153`) from the hybrid KX
  (X25519 + ML-KEM + callsigns); `set_kx_phase(KX_HYBRID_DONE)`; KEY_ACTIVATE 8-byte
  confirm-tag gate (`arq_common.cc:10585` refuse / `:10661` match). No session id is
  transmitted; the authenticated connection identity IS the derived key + callsigns.
- AEAD nonce fold: `batch_index = fold_gen_index(rx_nonce_gen, unwrap_batch_index(...))`
  (`arq_common.cc:22428-22437`); `rx_nonce_gen` is LOCAL, never transmitted. The AEAD
  authenticates only the reassembled ciphertext payload bound to
  `(batch_index, direction, tag_size)`; the origin/offset is NOT in the AD.

## Q2 — Consumers (readers of the delivery-ordering gate)

- `delivery_step_is_gap(int bsi, int last_delivered_bsi)` — PURE/static predicate,
  `arq.h:2376`. The hole/contiguity ruler. Read at the batch-done gate
  `arq_responder.cc:3263-3266`, and at `arq_responder.cc:1401`, `:1630`,
  `arq_common.cc:6847`.
- `gap_is_recoverable_prev_hole(int cur, int last, bool prev_active, int prev_bsi,
  int prev_received_count)` — PURE/static, `arq.h:2395`; recoverable-hold classifier
  read at `arq_responder.cc:3290`.
- Batch-done gate: `process_messages_acknowledging_data()` `arq_responder.cc:2665`;
  gate at `:3263-3266`; recoverable-hold at `:3290`; commit call at `:3377`.
- Commit primitive: `rsp_commit_cur_batch_delivery()` (decl `arq.h:1037`;
  def `arq_responder.cc:2576`) — sets `decrypt_delivered_bsi=delivered_bsi` `[?]~2626`,
  rolls prev/cur, calls `copy_data_to_buffer()`. Also the prev/held-cur commit at
  `arq_responder.cc:1644`.
- The single byte-commit funnel: `copy_data_to_buffer()` (decl `arq.h:4164`;
  def `arq_common.cc:22055`). Comments confirm it is THE single delivery funnel
  ("the ONE receiver funnel", `arq.h:4458`; "The single byte-delivery funnel
  copy_data_to_buffer() refuses to [deliver non-contiguous]", `arq.h:4596`). It already
  hosts contiguity machinery keyed on `decrypt_delivered_bsi` BEFORE `fifo_push_rx`:
  INV-DEDUP `[?]~22086`, PREV span-gate `[?]~22150`, cross-session seam `[?]~22197`,
  `w_stream_shift_detected` `[?]~22237`, rebase-seam `[?]~22290`. Existing intent:
  "byte-contiguity is UNPROVABLE ⇒ REFUSE (copy_data_to_buffer teardown)"
  (`arq.h:4554`).

All the above are LIVE production RX paths (dispatch `process_messages_acknowledging_data`
`arq_responder.cc:50`, `process_messages_rx_data_control` `:54`). Test-only references
live at `arq_responder.cc:>=12694`, `arq_commander.cc:>=15580`, and the
`test_stream_offset.cc` / `test_compress_reassembly_bounds.cc` / `test_bigblock_arq_unit.cc`
harnesses — DEAD for production.

## Q3 — Valid states of the ordering variables

- `rsp_last_delivered_batch_seq_id == -1`: NOTHING delivered yet (session start /
  post-KEY_ACTIVATE re-anchor). This is the ONLY state in which the defect fires — the
  `last < 0` branch of both predicates.
- `>= 0`: the seq id (mod 256) of the last committed batch; forward-only advance.
- `rx_stream_delivered`: monotonic app-FIFO append cursor; 0 at session start.
- `StreamStamp.valid`: true when a stamp was parsed for a batch; the stamp's `.start`
  gives the batch's absolute intended offset. `.valid == false` (absent stamp) today
  fails OPEN — the origin decision cannot rely on the stamp being present unless that
  is tightened.

## Q4 — Invariants the funnel currently assumes (and which is wrong)

- INV-1 (contiguity): a batch may only commit if it is the +1 successor of the last
  delivered batch (mod-256 forward distance 1). Enforced correctly for `last >= 0`.
- INV-2 (dedup): re-delivery of the same seq id is a no-op. Correct.
- **INV-3 (BROKEN): "the first delivery is legal at offset 0, whatever batch it is."**
  Verbatim, `arq.h:2378`: `if(last_delivered_bsi < 0) return false;` with the comment
  "session-start first delivery is always legal", and the same early-out in
  `gap_is_recoverable_prev_hole` (`arq.h:~2400`). This is the exploited invariant.
  It silently assumes the first batch the receiver commits IS the stream origin. When
  the origin is sealed-with-a-hole (PATH-A) or wholly lost (PATH-B), the first
  committed batch is NOT the origin, and its bytes land at offset 0 anyway.

The correct invariant is **INV-3': a first delivery is legal at offset 0 only if the
batch is the stream's authenticated origin** — the batch whose descriptor places it at
delivered-stream offset 0. Everything else about ordered delivery (INV-1/INV-2) is
unchanged.

## Q5 — What the origin-binding change does

Replace INV-3 with INV-3' at the one commit chokepoint (`copy_data_to_buffer`, and the
predicate `delivery_step_is_gap`'s `last < 0` branch), by binding the delivered-stream
origin to an authenticated stream descriptor rather than to first-seen batch order.
The bytes reach the app FIFO through the shared-core ordered-commit funnel: a batch
whose generation is not the descriptor's first_generation cannot commit while nothing
has been delivered — it is HELD (the existing GAP-HOLD / recoverable-hold path)
instead of committed at offset 0. Both PATH-A and PATH-B then become unrepresentable:
batch 1 can never be the offset-0 delivery, because the origin is fixed to the
authenticated descriptor, not to whichever batch the receiver happened to complete or
adopt first.

The shared core (`mercury::fade::Receiver`) enforces exactly INV-3'/INV-1: `publish_once`
seeds `next_generation_` from `descriptor.first_generation()`; `prepare_commit` refuses
`identity.generation != next_generation_`; `commit` appends through the `OrderedSink`
(which performs the existing `fifo_push_rx` append) and advances the cursor. `admit`
consumes a single-use `WireAttestation` minted at the receiver's one authentication
boundary, refuses a null receipt before any storage mutation, and refuses any record
whose bytes changed since attestation. This is the delivery-correctness invariant the
core was built to hold; adopting it here retires the ad-hoc `last < 0` early-out.

### Two adoption shapes (the origin descriptor's protection is the fork)

The shared core supports two protection modes (`fade_core.h`): `AEAD_COVERED`
(identity bytes already AEAD-authenticated) and `AUTHENTICATED_FRAME` ("a non-AEAD
carrier whose existing protected frame covers the same bytes"). Which one applies is a
wire-format question:

- **SHAPE (a) — no wire change (recommended for the accidental defect class).** Build
  the stream descriptor from the KX-derived session identity (Q1: `derive_session_key`,
  frame-CRC + confirm-tag gated) plus the existing frame-CRC-protected positional
  descriptor `StreamStamp.start` (Q1: `w_parse_eob_stamp`). The origin is the batch
  whose `StreamStamp.start == 0`; `first_generation` is that batch's seq id. Admit each
  completed batch with `Protection::AUTHENTICATED_FRAME` / `Verification::FRAME_VERIFIED`
  and gate the commit so a batch may reach offset 0 only when its `StreamStamp.start`
  equals the current `rx_stream_delivered` cursor (i.e. `== 0` for the first delivery).
  This closes both accidental paths without adding any wire bytes and without an
  AEAD-AD change. It leverages the funnel's EXISTING contiguity machinery (`arq.h:4554`,
  the PREV span-gate, `w_stream_shift_detected`) but tightens the origin decision to
  FAIL-CLOSED: an absent/`invalid` stamp on a candidate first delivery HOLDS instead of
  committing at offset 0. Frame-CRC coverage is sufficient against ACCIDENTAL
  corruption (protocol races, bit flips), which is the defect class here; it is NOT
  adversary-resistant (a forged stamp is out of scope for this class).

- **SHAPE (b) — full authenticated binding (attacker-proof; OWNER-DECISION).** Fold the
  origin/first_generation token and `StreamStamp.start` into the AEAD associated-data,
  and add a new wire origin/generation token so the receiver can cryptographically
  verify the origin. Admit with `Protection::AEAD_COVERED` / `Verification::AEAD_VERIFIED`,
  attesting over `canonical_identity_bytes(identity)`. This satisfies the shared core's
  full obligation set and resists a forged origin, but it is a WIRE-FORMAT change (new
  token bytes per stream + AEAD-AD layout change) with backward-compatibility impact,
  and is therefore an owner-decision, surfaced not taken here. Wire cost estimate: `[?]`
  (per-stream origin token + first_generation field; quantify bytes/frame and
  old/new-peer compatibility before proposing).

### The three shared-core integration obligations (bind whichever shape ships)
1. **KX-derived session/keys.** `Receiver.expected_session` and the wire-authenticator
   material derive from the per-connection KX (KEY_EXCHANGE_1..3 / KEY_ACTIVATE,
   `arq_common.cc:9840`/`:9845`, `derive_session_key` `:10274`/`:10312`), never from
   static configuration; one `Receiver` per (session, direction). This keeps a
   prior-lifetime record from re-admitting under a new session.
2. **Single attestation mint site** at the one authentication boundary — the frame/AEAD
   check inside the RX funnel (`cipher_suite.decrypt` `[?]arq_common.cc:22447`,
   gated by `is_active()` `[?]:22415`) — attesting only over the bytes that check
   accepted; one wire event = one admission attempt.
3. **AAD = canonical identity bytes.** The attestation binds
   `canonical_identity_bytes(identity)` (SHAPE b) / the frame-CRC-covered identity
   incl. `StreamStamp` (SHAPE a). No field may change between authentication and
   admission.

### Wire-activation witness (the change must be seen to FIRE)
The commit gate must emit a production log token when it HOLDS a non-origin batch that
would otherwise have committed at offset 0 (e.g. an origin-hold at
`rsp_last_delivered_batch_seq_id < 0` with a non-zero `StreamStamp.start`), and when the
authenticated origin commits first. This is the wire-activation witness that a live run
must show moving before the fix is claimed effective — a prior narrow fix passed its
unit gates but its session-start hold never fired on the wire, so it was inert on the
lost-origin path. The witness must be a production counter/log line on the real funnel,
not a test-only signal.

---

## Open questions [?]
- Re-anchor all `.cc` line numbers with `tools/cite_check.sh` before editing (they were
  localized during this audit; only the `arq.h` predicate/decl and `StreamStamp` cites
  were opened directly).
- Absent-stamp handling: confirm every completed-batch first-delivery path reaches
  `w_parse_eob_stamp` with a valid stamp, or define the fail-closed hold precisely for
  the `.valid == false` case (SHAPE a).
- SHAPE (b) wire cost: exact bytes/frame for the origin+first_generation token and the
  AEAD-AD layout change, plus mixed old/new-peer compatibility.
- Confirm `rx_stream_delivered` is the correct absolute-offset comparand for
  `StreamStamp.start` in all re-anchor/rebase-seam cases (`arq_common.cc:10098-10146`,
  the cross-session/rebase seams in the funnel).
