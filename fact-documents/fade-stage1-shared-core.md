# FADE Stage-1 shared correctness core

The Stage-1 implementation is `include/datalink_layer/fade_core.h` plus
`source/datalink_layer/fade_core.cc`. It intentionally chooses neither Stage-2
delivery architecture and changes no existing ARQ framing, payload capacity,
batch sizing, timed waveform packing, or runtime default. Both Stage-2 designs
must use this core at the same ownership boundaries.

## Required integration order

1. The directional sender creates one `StreamDescriptor` with
   `StreamDescriptor::create_local`. The descriptor is immutable through the
   public API and contains the full endpoint/connection/stream/direction address,
   session, 256-bit CSPRNG origin, sender instance, and first 64-bit generation.
2. The sender gives source and transported bytes to `SourceRetainer::stage`
   before transmission. The resulting `DataRecord` owns the complete identity.
3. The selected wire design encodes that identity. If DATA is already AEAD
   protected, `canonical_identity_bytes` is existing AEAD AAD. Otherwise the
   existing authenticated frame covers those bytes. The binding also carries
   the sender's application length and digest. Do not add another MAC.
4. The receiver constructs `Receiver` with the full expected address and the
   current authenticated session identifier. It authenticates/decrypts the
   frame and calls `Receiver::admit`
   with the corresponding verification result before parsing or mutating any
   batch, big-block, ACK-accounting, BSI, or connection state.
5. A successful admission produces RECEIPT feedback. RECEIPT can suppress an
   unnecessary retransmission but cannot release the retained source.
6. Existing reassembly, decrypt, and decompress work completes. Only then does
   the delivery funnel obtain a `CommitTicket` for the exact next generation and
   call `Receiver::commit` with the final application bytes. Length and digest
   must match the sender commitment before the sole sink is invoked.
7. Sink acceptance produces distinct COMMIT feedback. Only a matching,
   authenticated COMMIT with the same full identity, transported digest, and
   application length releases source ownership.
8. Teardown calls `Receiver::revoke`. A replacement session uses a new Receiver
   and a new locally created descriptor. Retained source may cross that explicit
   transition only through `SourceRetainer::rebind`, which replaces origin,
   instance, and generations while retaining bytes.

## Contract mapping

| Requirement | Enforced by |
|---|---|
| Fresh directional origin | OS CSPRNG `OriginFactory`; all-zero and process-local repeats refused; descriptor fields have no public mutators; a receiver is pre-bound to the current authenticated session and refuses an old session's first offer. |
| Sender-bound ownership | `SourceRetainer` requires the local endpoint to equal the descriptor sender; DATA, stored records, tickets, and feedback retain the whole descriptor, generation, application length, and application digest. |
| Unconditional admission | `Receiver::admit` checks publication, absolute deadline, complete stream identity, verification mode, full generation window, and digest before insertion or other mutation. |
| One revocable ordered sink | One const sink is constructor-bound. Admission, ticketing, append, cursor advance, and revoke share one mutex; the shared revocation token invalidates every outstanding ticket, including after destruction. |
| Retain until COMMIT | Source bytes remain in `SourceRetainer` after RECEIPT and are erased only by a fully matching COMMIT. |
| Receipt differs from commit | Separate feedback kinds and canonical encodings; COMMIT exists only after the application sink returns success. |
| Active non-sliding deadline | `steady_clock` absolute deadline plus a dedicated condition-variable worker; duplicate DATA/control and progress never write the deadline. |
| Address preservation | Canonical identity includes both endpoint IDs, connection, stream, direction, session, origin, instance, and full generation; equality gates storage and feedback release. |
| Structural production isolation | Production header/source have no test/fault switch, environment lookup, defeat macro, or test-build branch. Build-time audit fails if one is introduced. |

Generation exhaustion fails closed rather than wrapping. Pending receiver storage
is bounded by a constructor limit. Conflicting duplicate DATA revokes the stream;
exact duplicates are idempotent and produce RECEIPT only.

## Permanent gates

`mercury --test-fade-core` covers all five predecessor refutation categories,
the missing-first-generation 24-byte deletion case, conflicting and relabelled
identity, reconnect feedback replay, source retention, commit backpressure,
concurrent revocation, active deadline expiry, and generation exhaustion. The
same test is part of full `mercury --test` and is also built and run during every
normal build.

The WGN gate replays the frozen six-run clean cohort at the core boundary: six
262144-byte exact transfers, 18 timed RF batches per transfer, and the frozen
319.033 / 349.56232966449915 forward-occupancy ledger (91.266%). It protects the
batch ownership boundary without claiming a new RF observation or touching the
live fleet.
