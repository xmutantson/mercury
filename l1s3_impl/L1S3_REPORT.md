# L1 Stage-3 implementation report

Date: 2026-08-15  
Base: `293181d` (`293181de5` in the brief)

## Result

L1 Stage-3 is implemented behind the exact, default-off gate
`MERCURY_L1_BLOCKACK=1`. It adds fail-closed handshake negotiation, a
pre-parser RX fence/dispatcher, real N-batch clean-ACK deferral, bounded
multi-batch TX ownership, tail flushing, and atomic settlement through the L1
journal.

The clean production flow is:

1. Both CRC-checked handshake capability bytes must carry `CAP_L1_BLOCKACK`.
2. Each transmitted batch is staged and marked SENT in the journal before the
   legacy single-batch cache may be released.
3. The responder's existing ACK gate records the received bitmap but emits no
   reverse frame for clean intermediate batches.
4. The Nth batch emits one `BLOCK_SACK`. A short final group is closed by a
   session-bound `BLOCK_COMMIT`, then emits one tail `BLOCK_SACK`.
5. The commander validates session, epoch, negotiation id, block serial, BSI
   sequence, directory, bitmap bounds, and CRC before applying all positive
   `(transmitted_bsi, slot)` keys through `L1TxJournal::acknowledge_many()`.

Thus a clean B-batch transfer emits `ceil(B/N)` reverse ACK frames. Suppressing
only the commander listener was not used: the responder really remains silent,
and the journal retains every unsettled batch until a validated aggregate
arrives.

## Compatibility and negotiation

- Allocated capability bit `0x20`; `CAP_NEGOTIABLE_MASK` is now `0x3f`.
- Reused previously reserved TEST_CONN bit 20 and TEST_ACK bits 19/18 for the
  local/echoed capability. Both payloads remain exactly 38 bits.
- A legacy/bare ACK can never infer the block-ACK bit.
- Enablement requires the exact environment value, both advertised bits, the
  checked capability echo, and nonzero session/epoch binding.
- Even after negotiation, a PHY configuration whose information word cannot
  hold the worst-case `N`-batch aggregate fails closed to the legacy per-batch
  ACK path; an oversized control frame is never truncated or emitted.
- Unnegotiated block-family types are consumed before the legacy parser can
  alter `messages_rx_buffer`, sequence state, or ACK state.
- With the gate unset, `0`, `2`, or any value other than the exact string `1`,
  the advertised byte, legacy receive transition, journal enablement, timers,
  state-machine route, and emitted wire bytes remain on the old path.

## Ownership and loss behavior

- The journal retains up to eight legacy-sized batches (8 x 96 slots) while
  preserving the 96-slot per-batch bound.
- Sequential BSI rollover `254,255,0,1` remains in one retained epoch and is
  settled by one aggregate.
- Positive bits in a partial aggregate settle atomically; misses remain owned.
- This Stage-3 version fails loudly on any partial block: after emitting the
  partial aggregate, both production endpoints abort the session, and the
  commander transfers all unresolved journal entries to `L1TerminalQueue` with
  `delivery_claimed=false`. It never advances across a hole and never fabricates
  retransmission ownership. Selective recovery across a multi-batch window is a
  possible later extension, not silently approximated here.

## Test-first evidence

The production-path test was created before `l1_block_ack.h/.cc`. Its initial
RED compile failed with:

`fatal error: datalink_layer/l1_block_ack.h: No such file or directory`

The final test has compile-time defeat arms which remove one required behavior
at a time. Each arm exits nonzero against the same production methods:

| Arm | Expected RED result |
| --- | ---: |
| `L1_BLOCK_RX_FENCE_DEFEAT` | 2 failures |
| `L1_BLOCK_CAPNEG_DEFEAT` | 11 failures (negotiation plus dependent paths) |
| `L1_BLOCK_MIXED_FAILOPEN_DEFEAT` | 2 failures |
| `L1_BLOCK_AGGREGATION_DEFEAT` | 9 failures |
| `L1_BLOCK_GATE_OFF_DEFEAT` | 2 failures |

The tests call production runtime, codec, RX dispatch, and journal methods; they
do not assign private negotiated/dispatch/journal state.

Pass-after coverage includes:

- legacy/mixed block-frame fence with zero parser/ACK mutation;
- authenticated both-peer enable and mixed/invalid-echo fallback;
- exact environment gate and off-path field/wire identity;
- N=4 over 9 batches: two full groups plus a committed one-batch tail, exactly
  3 reverse ACKs (`ceil(9/4)`);
- all 18 journal slots for those 9 batches settled only at the aggregate
  chokepoint;
- atomic rollover settlement for BSIs `254,255,0,1`;
- partial aggregate: positives released, two misses retained, then transferred
  to the terminal owner without a delivery claim.

## Verification

- `bash build.sh o3`: PASS.
  - L1 block codec deterministic structure-aware fuzz: 120,000 mutations PASS.
  - L1 Stage-3 production-path test: PASS, 0 failures.
  - L1 ownership-journal contract test: all 30 rows PASS.
  - Native Mercury link: PASS.
- Focused ASan/UBSan run: PASS, 0 failures (`detect_leaks=0`; LeakSanitizer is
  unsupported under this traced execution environment).
- Shipped `./mercury --test`: NONZERO. The changed TEST_CONN and TEST_ACK
  six-bit round-trip cases reported `[OK]`, and the L1S3-focused tests above are
  green. The exhaustive existing harness nevertheless reported seven failures
  in three non-L1S3/default-off groups: `TEST-INBAND-DELIVER` (2),
  `TEST-CMPRETRY` (1), and `TEST-MEASTIMERS` (4). No global-suite pass is
  claimed; these broad-harness failures remain unresolved.
- `git diff --check`: PASS.

## Wire-activation witness

Full N=4 emission:

`[L1-BLOCKACK-TX] block=1 start_bsi=20 batches=4 slots=8 final=0`

Short-tail commit and emission:

`[L1-BLOCKACK-TX] block=3 start_bsi=28 batches=1 slots=2 final=1`

Atomic commander settlement:

`[L1-BLOCKACK-RX] applied block=1 start_bsi=20 batches=4 acked_slots=8`

The live ARQ hooks additionally emit `[L1-BLOCKACK-RSP]` for actual responder
deferral/tail flush and `[L1-BLOCKACK-ARQ]` when validated settlement enters the
existing `data_ack_received=YES` funnel.

## Scope and follow-up

- No +10-12% duty claim is made from the structural test. That figure remains
  the oracle's measured ceiling; a sustained RF/audio loopback benchmark is the
  appropriate follow-up performance measurement.
- The sandbox does not grant write access to
  `/mnt/c/mercury_codex_stage/l1s3_impl/`. The approved workspace fallback is
  `l1s3_impl/`; this report and `l1s3.diff` are delivered there.
- No commit was created.
