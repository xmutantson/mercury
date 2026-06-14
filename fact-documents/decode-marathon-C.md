# Decode marathon — LEVER C: multi-core batch decode — fact document

Branch `feat/decode-marathon` (stacks on lever D `@abf3992`).
Worktree `C:/Users/kamer/mercury_wt/decode-marathon`.
Status: **IMPLEMENTED + §3 integrity-gated + default-off byte-identical (md5 == base
@abf3992). See §9.** (The audit/design in §0-§8 below was written BEFORE code; §9
records the shipped implementation + test results.)

## §0 The lever and the EXACT target loop

LEVER C = decode the held-CFG16 block's `Kcw` codewords (held-CFG16: up to
25-30; nominal big-block K=8) ACROSS A THREAD POOL instead of serially on one
core. The Pi 5 has 4 cores; capture-prep owns one, the main RX thread one, so
2-3 cores sit idle during a batch decode.

**THE TARGET LOOP IS NOT the per-frame `receive_byte` RX path** (one acquisition
= one frame, decoded serially, each immediately carved into ARQ state — not a
batch in memory). The held-CFG16 batch is decoded in ONE acquisition via the
**big-block** path: `receive_byte` → `receive_bigblock` (telecom_system.cc:1038)
→ `bigblock_rx_passband` (telecom_system.cc:7958). Inside it, after the WHOLE
block has been acquired, demodulated, equalized, CSI-weighted and the LLRs for
ALL `Kcw` codewords are sitting in one `clr[nBits]` vector, the codewords are
decoded by:

```
// telecom_system.cc:8633-8651
cw_ok_out.assign(Kcw, 0);
int cw_ok=0;
std::vector<float> cwllr(ldpc.N); std::vector<int> dec(ldpc.N);   // SHARED scratch
for(int c=0;c<Kcw;c++)
{
    for(int i=0;i<ldpc.N;i++) cwllr[i]=clr[(size_t)c*ldpc.N+i];   // slice in
    ldpc.decode(cwllr.data(), dec.data());                        // SERIAL decode
    for(int i=0;i<ldpc.K;i++)
        out_infobits[(size_t)c*ldpc.K + i] = dec[i];              // disjoint slice out
    int ierr=0;
    if(cw_info_ref && (int)cw_info_ref->size() > c)
        for(int i=0;i<ldpc.K;i++) if(dec[i]!=(*cw_info_ref)[c][i]){ ierr++; }
    cw_ok_out[c] = (ierr==0) ? 1 : 0;                              // disjoint slot out
    if(ierr==0) cw_ok++;
}
K_out = Kcw;
return cw_ok;
```

This loop is the lever-C site. It is **embarrassingly parallel by
construction**: every per-`c` input (`clr[c*N..]`) is read-only and already in
memory; every per-`c` output (`out_infobits[c*K..]`, `cw_ok_out[c]`) is a
DISJOINT slice indexed by `c`. The only obstacles are (a) the single shared
`cl_ldpc ldpc` instance and (b) the shared `cwllr`/`dec` scratch vectors and the
shared `cw_ok` accumulator. None crosses an ARQ-state line — the carve into
`messages_rx[]` is a SEPARATE serial step that runs AFTER this function returns
(§3 below).

(For symmetry the per-frame `receive_byte` LDPC site at telecom_system.cc:3047
and the `--test`/SFO-GRID harness sites at :7316, :8639, :9155 also call
`ldpc.decode`; lever C touches ONLY the `bigblock_rx_passband` :8639 site. The
per-frame :3047 path decodes ONE codeword per call and is out of scope.)

---

## §1 PRODUCERS — every path that WRITES the per-decode mutable state

### 1.1 `cl_ldpc` instance workspace (the landmine the task named)

`cl_ldpc` (include/physical_layer/ldpc.h) owns mutable decode workspace as
INSTANCE members. One `cl_ldpc` cannot decode two codewords at once:

| Member | ldpc.h | Allocated | Written every decode |
|---|---|---|---|
| `double* R` | :43 | `update_code_parameters()` ldpc.cc:140 (per rate) | `decode_SPA` zeroes + fills R[N*Vwidth] every call (ldpc_decoder_SPA.cc:222-224, :360-366, :460, :508) |
| `double* Q` | :44 | ldpc.cc:141 | zeroed + filled (:223, :285, :579) |
| `int* V_pos` | :45 | ldpc.cc:271 (`P*Cwidth`) | rebuilt inside decode_SPA (:248-271) when `nOnes!=0` |
| `std::atomic<bool>* decode_abort` | :101 | nullptr default | READ in decode_SPA (:326,:428); written by the MONITOR parallel-config path only |
| `bool early_term_speculative` | :111 | false | set TRUE by the per-frame sub-peak probe (telecom_system.cc:3219), cleared :3053 — **NOT touched on the bigblock path** |
| `int last_early_term_iter` | :119 | -1 | written by decode() (ldpc.cc:307) + decode_SPA (:200,:415,:556) |
| `int N,P,K`, `Cwidth/Vwidth/dwidth`, the `QCmatrix*` pointers, `rate/framesize/nIteration_max/decoding_algorithm_val` | ldpc.h:36-73 | `init()`/`update_code_parameters()` | **read-only during a decode** (set once at load_configuration); shareable IF every worker is the SAME config |

> CORRECTION-NOTE: the task brief lists `R/Q/V_pos/L/LLRtmp/Cout/LLRbin` as if all
> live in `cl_ldpc`. **Only `R`, `Q`, `V_pos` are `cl_ldpc` instance members.**
> `L`, `LLRtmp`, `Cout`, `LLRbin` (and the fwd-back scratch `fb_t/fb_pref/fb_slot/
> fb_vslot`) are **STACK-LOCAL arrays inside `decode_SPA`** (ldpc_decoder_SPA.cc:
> 191-195 `Cout[N_MAX]`,`LLRbin[N_MAX]`,`LLRtmp[N_MAX]`; :320 `L[N_MAX]`; :300-306
> the fb_* scratch). `N_MAX=1600` (physical_defines.h:31) so each is ~12.8 KB of
> stack — fine for a worker thread (default thread stack ≥ 1 MB on glibc/Pi). Being
> stack-local, they are **already thread-private** per `decode_SPA` invocation:
> they are NOT a replication concern. The ONLY heap workspace that must be private
> is `R`/`Q`/`V_pos`, and the cleanest way to get private `R`/`Q`/`V_pos` is a
> private `cl_ldpc` per worker.

### 1.2 `bigblock_rx_passband` loop-local shared scratch

- `std::vector<float> cwllr(ldpc.N)` (telecom_system.cc:8635) — reused every `c`.
- `std::vector<int> dec(ldpc.N)` (:8635) — reused every `c`.
- `int cw_ok` (:8634) accumulator — incremented per `c`.

These are function-locals; trivially replicated per worker (each worker gets its
own `cwllr`/`dec`, and `cw_ok` becomes a reduction over `cw_ok_out`).

### 1.3 the per-frame `data_container` buffers (NOT on the bigblock decode loop)

`cl_telecom_system` owns `data_container`, `psk`, `mfsk`, `ofdm`, `ldpc`,
`receive_stats` as VALUE members (telecom_system.h:126-305). The per-frame
`receive_byte` decode chain writes a long list of single per-instance buffers
(`deinterleaved_data`, `hd_decoded_data_bit/byte`, `equalized_data`,
`demodulated_data`, `ofdm_deframed_data`, …, data_container.h:40-77). **The
bigblock per-codeword loop does NOT use those** — it works out of its own local
vectors (`clr`, `cwllr`, `dec`, `out_infobits`) and the shared `cl_ldpc`.
`out_infobits` is the caller's `bigblock_rx_infobits` member
(telecom_system.cc:9566), written in disjoint `c*ldpc.K` slices.

### 1.4 `messages_rx[]` — the ARQ producer (DOWNSTREAM, stays serial)

`add_message_rx_data(type,id,length,data)` (arq_responder.cc:50-106) writes
`messages_rx[loc]` (`.type/.length/.data[]/.status=RECEIVED`, bumps
`stats.nReceived_data`), indexed by frame `id`. On the bigblock path this is
bypassed — the carve writes `messages_rx[]` directly:
`bigblock_receive_carve` → `bigblock_block_to_arq` carves `cw_ok[c]==1 →
messages_rx[c].status=RECEIVED` + synthetic EOB
(data-flow-bigblock-arq-unit.md §1.2, arq_common.cc:8798). **This runs on the
main thread AFTER `receive_bigblock` returns** (arq_common.cc:8785-8807) — it is
NOT inside the decode loop and lever C does not move it.

---

## §2 CONSUMERS — every path that READS the state lever C parallelizes

### 2.1 in-loop consumers of `R`/`Q`/`V_pos`
Only `decode_SPA` itself (ldpc_decoder_SPA.cc), via the `R`/`Q`/`V_pos` pointers
passed from `cl_ldpc::decode` (ldpc.cc:308). No other thread or function reads
these between codewords.

### 2.2 consumers of the decode OUTPUT (`out_infobits`, `cw_ok_out`)
- `bigblock_rx_passband` returns `cw_ok` (count) and fills `cw_ok_out`,
  `out_infobits`, `K_out` (telecom_system.cc:8648-8652).
- `receive_bigblock` stashes them: `bigblock_last_rx_cw_ok = cw_ok`
  (telecom_system.cc:9605), `bigblock_last_rx_K = Kout` (:9606),
  `bigblock_rx_infobits` holds the bits (:9565-9566), copies a bounded prefix to
  `out` (:9602).
- **ARQ carve** reads them: `bigblock_receive_carve(bigblock_rx_infobits.data(),
  …)` (arq_common.cc:8798) → `messages_rx[c]` (cw_ok→RECEIVED).

### 2.3 the SACK bitmap (reads `messages_rx[].status`)
- RSP build: `sack_bitmap[i] = (messages_rx[i].status == RECEIVED)` for
  `i<data_batch_size` (arq_responder.cc:1818-1820; also :3949-3951 in the unit
  test) → `send_sack_v2_frame` (:1959).
- On the bigblock path the K-bit `cw_ok` IS the bitmap source
  (data-flow-bigblock-arq-unit.md §1.7).
- CMD side: `sack_bitmap[i]` from the decoded SACK_RSP
  (arq_commander.cc:2694-2866), drives the retransmit set (:3095-3103).

### 2.4 in-order delivery — `copy_data_to_buffer()` (arq_common.cc:9881)
Iterates `messages_rx[i]` IN SLOT ORDER `for i=0..data_batch_size`
(arq_common.cc:9903), reassembles ACKED slots into `assembled[]` (:9909-9911),
then runs the SINGLE-BLOCK decrypt + **streaming decompress**
(`compressor.decompress_block`, :9985) which carries cross-frame PPMd/zstd
state, then `fifo_push_rx` (:10000). **In-order delivery is structural: it reads
the array in index order.** Lever C does not touch this; the carve produces the
same `messages_rx[]` it always did.

### 2.5 `last_received_end_of_batch_seq` / `effective_batch` / ACK timing
The per-frame storage block reads `last_received_end_of_batch_seq` to size
`effective_batch` and choose the ACK timeout (arq_responder.cc:1171-1209). On
the bigblock path the carve sets the synthetic EOB. Single-threaded; untouched.

---

## §3 VALID STATES (esp. pre-producer defaults)

- `cl_ldpc` before `init()`: all NULL/0 (ldpc.cc:26-56). After
  `load_configuration` (`PHYSICAL_LAYER_ONLY`), `R`/`Q`/`V_pos` allocated, the
  code params frozen for the active config. **A worker `cl_telecom_system` must
  have `load_configuration(CONFIG_16)` run on it before use** — same as the
  monitor decoders (arq_common.cc:1771).
- `messages_rx[i].status` defaults FREE (arq.h); the carve sets RECEIVED only for
  `cw_ok[c]==1`. The decode loop NEVER writes `messages_rx[]`.
- `decode_abort==nullptr` (ldpc.h:101) ⇒ decode_SPA's abort checks are no-ops
  (:326,:428). The bigblock loop leaves it nullptr ⇒ no atomic contention.
- `early_term_speculative==false` on the bigblock path (only the per-frame
  sub-peak path sets it). ⇒ `et_mode=0` ⇒ the non-converge detector is never
  called ⇒ decode runs to the cap exactly as serial (ldpc_decoder_SPA.cc:204).
- `Kcw` valid range: `nBits/ldpc.N`, optionally capped by `MERCURY_BIGBLOCK_K`
  (telecom_system.cc:8601-8602). `cw_ok_out` is sized `Kcw` (:8633).

---

## §4 INVARIANTS the consumers assume + how lever C must preserve them

1. **INV-BITEXACT**: every codeword's decoded bits must be IDENTICAL to the
   serial decode. Holds iff each worker has a PRIVATE, freshly-zeroed
   `R`/`Q`/`V_pos` and the SAME config params. `decode_SPA` re-zeros `R`/`Q` at
   entry (ldpc_decoder_SPA.cc:222-224) and rebuilds `V_pos` (:248-271), so a
   private `cl_ldpc` per worker yields the same numbers a serial decode of that
   codeword would. There is NO cross-codeword state in `decode_SPA` (no static
   mutable, no carry); each call is a pure function of (LLRi, matrices, params).
   **Determinism**: the layered/fwd-back gates (`MERCURY_LDPC_LAYERED`,
   `MERCURY_LDPC_FWDBACK`) read env via `static` once-per-process (ldpc_decoder_
   SPA.cc:43,:82) — same value in all threads; the static-init race is benign
   (idempotent, same result). No RNG in decode. ⇒ bit-exact regardless of which
   worker decodes which `c`.

2. **INV-DISJOINT-OUT**: `out_infobits[c*K..]` and `cw_ok_out[c]` are written
   only by codeword `c`. TRUE by indexing (telecom_system.cc:8642,8648). No
   false sharing concern for correctness; `cw_ok` accumulator becomes a
   post-loop reduction over `cw_ok_out` (no shared counter).

3. **INV-SLOT-ORDER (ARQ)**: `messages_rx[]` must be written/read in a way that
   `copy_data_to_buffer` delivers byte-faithful in-order to the streaming
   decompressor. PRESERVED because lever C does NOT move the carve — it stays a
   serial step on the main thread after `receive_bigblock` returns, reading the
   FULLY-POPULATED `bigblock_rx_infobits` + `bigblock_last_rx_cw_ok`. The decode
   loop must therefore COMPLETE ALL `Kcw` codewords (join) before returning.

4. **INV-SINGLE-WRITER (ARQ state)**: `messages_rx[]`, the SACK build,
   `last_received_end_of_batch_seq`, `rsp_current_expected_batch_seq_id`,
   `stats.nReceived_data`, `copy_data_to_buffer`, the streaming PPMd/zstd
   contexts — ALL stay on the main thread, untouched by the worker threads. The
   workers touch ONLY their private `cl_ldpc` + private scratch + disjoint output
   slices.

5. **INV-CONFIG-FROZEN**: a worker decodes with `cl_ldpc` params for CONFIG_16.
   The block is a single-config block (held-CFG16). `bigblock_restore_stock_
   config()` (telecom_system.cc:9611) reloads the PRIMARY after the block; it must
   NOT run while workers are mid-decode (they hold their OWN cl_ldpc, but the
   thin-grid rebuild on the primary must be ordered before/after the join). Lever
   C joins all workers BEFORE `receive_bigblock` does its restore.

---

## §5 WHAT THE FIX CHANGES + per-consumer walk

**The change**: replace the serial `for(c)` loop (telecom_system.cc:8636-8650)
with an enqueue-to-pool + join. Each worker owns a PRIVATE decode context
(a private `cl_ldpc` already loaded for CONFIG_16, OR a replicated `R/Q/V_pos`
workspace) and PRIVATE `cwllr`/`dec`. The main thread:
1. extracts all `Kcw` LLR slices (or workers slice from the shared read-only
   `clr`),
2. ENQUEUES `c=0..Kcw-1` decode jobs to the pool,
3. JOINS (barrier) — all `out_infobits`/`cw_ok_out` slices filled,
4. reduces `cw_ok = sum(cw_ok_out)`,
5. returns to `receive_bigblock` → carve → SACK → copy_data_to_buffer, ALL
   serial on the main thread, byte-identical.

Per-consumer verification:
- §2.1 decode_SPA: each worker calls its OWN `cl_ldpc::decode` ⇒ private R/Q/V_pos
  ⇒ INV-BITEXACT holds.
- §2.2 output stash: filled before join returns ⇒ `receive_bigblock` sees a
  complete `bigblock_rx_infobits`/`cw_ok` exactly as serial.
- §2.3 SACK / §2.4 copy_data_to_buffer / §2.5 EOB: read the SAME serially-carved
  `messages_rx[]` ⇒ unchanged.

**WHAT MUST BE PRIVATE per worker** (§4 INV-BITEXACT): a `cl_ldpc` instance with
its own `R`/`Q`/`V_pos` (ldpc.h:43-45) loaded for the active config, plus its own
`cwllr`/`dec` scratch. `decode_SPA`'s `Cout/LLRbin/LLRtmp/L/fb_*` are already
stack-private. The replication unit precedent is `monitor_decoders[cfg]` — an
array of separate `cl_telecom_system` instances, each with a private
`data_container` + `cl_ldpc`, each fed a private audio copy, results written to a
STAGING buffer (`monitor_decoded_data`), never the live `messages_rx[]`
(arq_common.cc:1764-1775, :1915-1939, :8572). Lever C reuses exactly this
isolation model but at codeword granularity within ONE config.

**WHAT STAYS SERIAL / SINGLE-WRITER**: the carve into `messages_rx[]`
(arq_common.cc:8798), the SACK build (arq_responder.cc:1818), the ARQ batch state
(`last_received_end_of_batch_seq`, `rsp_current_expected_batch_seq_id`,
`stats.*`), `copy_data_to_buffer` + the streaming PPMd/zstd decompressor
(arq_common.cc:9881-10000), and `bigblock_restore_stock_config` — all on the main
thread, before/after the parallel region, NEVER concurrent with it. The pool
collects results in slot order (`out_infobits[c]`, `cw_ok_out[c]`) so the
post-join carve sees the EXACT serial layout.

---

## §6 RX / KEYER / TURNAROUND CONTENTION (must not steal the turnaround core)

- Decode runs on the MAIN RX thread, under `capture_prep_mutex`
  (arq_common.cc:8355). The dedicated `radio_capture_prep_thread`
  (audioio.c:1354) continuously fills the ring on its OWN core; the
  RX-capture/playback threads are separate (audioio.c:1821-1829). So the
  capture pipeline is ALREADY off the main thread.
- The keyer/PTT/ACK/turnaround state machine runs on the MAIN thread, AFTER
  `receive()` returns (responder.cc:432 `this->receive()` then the
  BREAK/ACK-gate logic). **A long serial batch decode BLOCKS the main thread
  from reaching the turnaround logic** — this is precisely the wall the marathon
  attacks: speeding the decode tail shortens the time the main thread is busy and
  GETS IT TO THE ACK SOONER, it does not contend with the keyer.
- CONTENTION RISK to control: the worker pool is ACTIVE only inside the
  `bigblock_rx_passband` codeword loop — a bounded window BEFORE the carve/ACK.
  The workers must JOIN before the main thread proceeds, so they cannot overlap
  the keyer or the next-frame capture. They DO transiently use the 2-3 idle cores
  during that window; on a 4-core Pi 5 that is exactly the design intent (idle
  cores during a decode burst). The capture-prep thread keeps running on its core
  (it does not need a worker core), so the next-frame RX capture is NOT starved.
- MITIGATION the design must specify: cap the pool at `min(Kcw, hw_concurrency-2)`
  (reserve one core for capture-prep, one for the main/keyer thread), so workers
  never oversubscribe the cores the RX/keyer path needs during turnaround. The
  pool is created once and reused (no per-block thread spawn cost). Default-OFF
  env gate (e.g. `MERCURY_DECODE_POOL`=0) ⇒ falls back to the serial loop ⇒
  byte-identical render, per the marathon's default-off discipline.

---

## §7 RISKS

- **R1 — config-frozen workers vs `bigblock_restore_stock_config`**: the restore
  reloads the PRIMARY telecom_system (telecom_system.cc:9611). Workers must hold
  their OWN cl_ldpc (loaded once for CONFIG_16) so a primary reload cannot free
  their `R/Q/V_pos` mid-decode. Join BEFORE restore. (Mirrors the monitor path,
  which keeps per-decoder cl_ldpc independent of the primary's reload.)
- **R2 — stack size**: each worker's `decode_SPA` uses ~5×`N_MAX`×8B ≈ 64 KB of
  stack (Cout/LLRbin int + LLRtmp/L double + fb_* small). Well under a default
  ≥1 MB pthread stack. Confirm the pool sets a sane stack (default is fine).
- **R3 — false sharing on `cw_ok_out[c]`/`out_infobits`**: adjacent `c` write
  adjacent cache lines. Correctness is unaffected (disjoint indices); only a
  micro perf concern. The `int K` (≥1400) info-bit stride per `c` means
  `out_infobits` slices are far apart; `cw_ok_out` (1 int per c) can share a line
  but writes are rare (once per codeword) ⇒ negligible.
- **R4 — env-gate static-init race**: `ldpc_layered_enabled()`/`ldpc_fwdback_
  enabled()`/`spa_earlyterm_params` cache env via `static` (ldpc_decoder_SPA.cc:
  43,82,152). First concurrent call races the init, but C++11 guarantees
  thread-safe function-local static init AND the value is identical in all threads
  ⇒ benign. (Pre-warm by one serial decode before the pool if paranoid.)
- **R5 — determinism of `cw_ok` reduction**: must be a post-join sum over
  `cw_ok_out[]` (order-independent for a count), NOT a shared `++` ⇒ no data race,
  same total as serial.
- **R6 — the WHOLE-BLOCK fields** (`bigblock_last_rx_cw_ok_count`,
  `bigblock_last_rx_K`, `K_out`, `acq_metric`) are written by the MAIN thread
  after the join (telecom_system.cc:8651, 9606-9607) ⇒ no worker writes them.
- **R7 — must not regress the per-frame path**: lever C touches ONLY the
  `bigblock_rx_passband` :8639 site. The per-frame `receive_byte` :3047 decode
  (one codeword/call) is unchanged ⇒ ROBUST/CFG0-15 and all non-bigblock traffic
  byte-identical.

## §8 §3 regression-test plan (fail-before / pass-after)

In-process, no IONOS/RF. Drive `bigblock_rx_passband` (or `receive_bigblock`) on
a known CFG16 K=N block (the existing loopback harness, `cw_info_ref` supplied so
the per-codeword gate is byte-exact — telecom_system.cc:9579-9583). Assert the
PARALLEL `out_infobits` + `cw_ok` are BIT-IDENTICAL to a serial reference decode
of the same captured passband (md5 of `bigblock_rx_infobits`). Fail-before: a
deliberately-shared single `cl_ldpc` across "workers" (or a defeat env) corrupts
≥1 codeword ⇒ md5 mismatch / cw_ok drop. Pass-after: private-context workers ⇒
identical md5 + identical `cw_ok`. Pair with the existing
`test_bigblock_arq_unit.cc` carve/SACK/copy_data_to_buffer assertions to prove
the downstream in-order delivery is unchanged.

---

## §9 IMPLEMENTATION + RESULTS (shipped)

### §9.1 Files
- `include/physical_layer/ldpc_decode_pool.h` + `source/physical_layer/ldpc_decode_pool.cc`
  — `cl_ldpc_decode_pool`: a PERSISTENT thread pool (created once, reused). Each
  worker owns a PRIVATE `cl_ldpc` (its own R/Q/V_pos) cloned from the primary's
  PUBLIC config scalars (`standard/framesize/rate/decoding_algorithm/GBF_eta/
  nIteration_max/print_nIteration`) + `init()`; the QCmatrix* tables it points at
  are read-only process globals (mercury_normal_*), safe to share. Workers claim
  codewords via an atomic counter (work-stealing), decode into private cwllr/dec,
  write DISJOINT out_infobits[c*K..]/cw_ok_out[c], and a condvar barrier (`decode_batch`)
  blocks the caller until every codeword is done. `ensure()` rebuilds only on a
  config/worker-count/defeat change.
- `source/physical_layer/telecom_system.cc`:
  - `bigblock_decode_codewords()` (new) = the single replacement for the serial
    codeword loop. Reads `MERCURY_LDPC_MULTICORE`: 0/1/unset => the ORIGINAL serial
    loop runs verbatim (byte-identical); >=2 => ensure+decode across the pool,
    clamped to `min(req, max(1,hw-2), Kcw)` (reserve a core for capture-prep + the
    main/keyer thread, §6). The serial loop body inside `bigblock_rx_passband` was
    replaced by a call to this method.
  - dtor frees the lazily-constructed pool (default-off leaves it nullptr).
- `include/physical_layer/telecom_system.h`: forward-declares `cl_ldpc_decode_pool`,
  adds the `ldpc_decode_pool*` member + `bigblock_decode_codewords()` decl.
- `source/physical_layer/test_decode_marathon.cc` + main.cc `--test-decode-marathon`:
  the §3 integrity gate (§8).
- `build.sh`: the two new .cc added to CPP_SOURCES.

### §9.2 §3 integrity gate — fail-before / pass-after (`--test-decode-marathon`)
Synthetic CFG16 batches (K8 mixed, K25 mixed, K30 all-clean) with KNOWN per-codeword
info bits (encode -> +-strong LLR -> deterministic flips: small=correctable,
600=uncorrectable). For each batch:
- PASS-AFTER: `bigblock_decode_codewords` SERIAL (env unset) vs PARALLEL
  (MERCURY_LDPC_MULTICORE=4) => `out_infobits` + `cw_ok` EXACT memcmp-equal
  (bits_match=1 cwok_match=1 count_match=1), every clean codeword faithful + in its
  correct disjoint slice + cw_ok=1 (inorder_ok=1).
- FAIL-BEFORE: MERCURY_DECODE_POOL_DEFEAT_SHARE=1 makes all workers share ctx[0]'s
  ONE cl_ldpc workspace (the exact bug) => DIVERGES from the serial reference on the
  FIRST trial (diverged_from_serial=1). Clearing the defeat recovers the pass-after.

Result: **ALL PASS, rc=0.** K8 ok_serial=ok_par=6, K25=20, K30=30. Both defeat arms
diverged after 1 trial.

### §9.3 default-off byte-identical
27-cell coded SFO-GRID render (`-m PLOT_PASSBAND -s 16`, MERCURY_SFO_GRID_CODED,
NSYMB=600), MERCURY_LDPC_MULTICORE UNSET:
- lever-C binary OFF render md5 = `4575011ba38e23575ef3f3bbce0703b2`
- freshly-built base @abf3992 OFF render md5 = `4575011ba38e23575ef3f3bbce0703b2`
=> **BYTE-IDENTICAL.** (The lever touches ONLY the bigblock decode loop; the
SFO-GRID render uses the per-frame :3047 decode path, untouched.)

### §9.4 harness green both states
`--test` and `--test-climb-engine` both exit 0 with MERCURY_LDPC_MULTICORE unset AND
=4. Local o3 build clean (pre-existing -Wmisleading-indentation warnings only).

### §9.5 scope / limits
- Cross-build (aarch64/Pi) uses the SAME build.sh CPP_SOURCES => the new files
  compile there too; the pool uses only std::thread/mutex/condvar/atomic (already
  `-pthread`/`-lpthread`). NOT yet built on the cross toolchain in this session
  (see the run summary).
- The pool's THROUGHPUT win (the marathon's point) is HW/Pi-confirmable; sim/loopback
  proves CORRECTNESS (parallel==serial, no corruption, default-off byte-identical),
  which is the §3 gate. The wall-time win on the held-CFG16 tail is a bench measurement.
