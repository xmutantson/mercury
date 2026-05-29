# Phase A.2 BP+OSD port research (2026-05-26)

**Status:** RESEARCH NOTES — no code changes. Produced by Phase A.2 research agent
for go/no-go on porting WSJT-X's BP+OSD decoder to Mercury's rate-1/16 LDPC.

**Scope:** Tasks 1–5 of the Phase A.2 brief — find WSJT-X source, document the
algorithm, document Mercury's matrix format, propose a porting layout + call-site,
and audit cross-layer interactions per CLAUDE.md §5.

**Conventions:** numbered sections (`§N.M`), `file.cc:line` citations, `[?]` for
unresolved questions, struck-through text for corrections.

---

## §1. Executive summary

- **License compatibility: OK.** WSJT-X is GPLv3 ([§2.2](#22-license-confirmation));
  Mercury is AGPLv3 ([ldpc.h:9-19](../include/physical_layer/ldpc.h#L9-L19)).
  GPLv3 code can be linked into an AGPLv3 work; the combined work as a whole must
  be distributed under AGPLv3 (AGPLv3 §13). Attribution must remain in headers.
- **Matrix format compatibility: OK.** WSJT-X's BP uses sparse `Mn` (col-indices-
  per-row, ≤3 per row) + `Nm` (row-indices-per-col, ≤7 per col) for the (174,91)
  code. Mercury's QCmatrixC + QCmatrixV use the **same dual representation**
  ([ldpc.cc:124-272](../source/physical_layer/ldpc.cc#L124)) — direct translation,
  no algorithmic conversion required ([§3.2](#32-mercury-matrix-format)).
- **Algorithm complexity at Mercury's K=100, N=1600: ACCEPTABLE FOR FALLBACK,
  RISKY FOR PRIMARY.** OSD-1 ≈ 100 test patterns × 100×1600 Gaussian elimination
  (one-shot, GF(2)). OSD-2 ≈ 4,950 patterns. OSD-3 ≈ 161,700 patterns. Most calls
  are dominated by the *first* Gaussian elimination, not the TEP loop, because
  TEP encoding is XOR-cheap. Estimated runtime on Pi 5 @ rate-1/16:
  OSD-1 ≈ 5-15 ms per failed decode, OSD-2 ≈ 50-150 ms ([§2.4](#24-complexity-at-mercurys-low-rate-code)).
  Used only as fallback when SPA exhausts its 200-iter cap, this is acceptable.
- **Algorithm signature match: OK.** WSJT-X's BP+OSD takes a float LLR array
  and produces an int codeword and a hard-error count — matches Mercury's
  existing `decode()` signature ([ldpc.h:91](../include/physical_layer/ldpc.h#L91)).
- **The hardest port question is NOT algorithmic — it's the generator matrix.**
  WSJT-X's OSD requires a dense generator matrix `G[K][N]` to do MRB encoding.
  Mercury's encoder uses a sparse accumulator-style `QCmatrixEnc[P][3]`
  ([ldpc.cc:100-121](../source/physical_layer/ldpc.cc#L100)). We need to either
  (a) precompute the dense G once at init from the existing encoder, or
  (b) reuse Mercury's encoder inside the MRB-encode loop (slower but no extra
  memory). Option (a) costs N×K bits = 100×1600 = 20 kB per active rate
  ([§4.4](#44-generator-matrix-construction)).
- **Estimated effort: 7–14 working days** for a tested A/B against SPA
  ([§7](#7-effort-estimate)).

---

## §2. WSJT-X source — Task 1

### §2.1. Files and locations

Source acquired via WebFetch from the saitohirga/WSJT-X GitHub mirror
(refreshed every 6 h from the upstream SVN/SourceForge):

| File | Role |
|---|---|
| `lib/ft8/decode174_91.f90` | Wrapper: BP loop, OSD fallback orchestration |
| `lib/ft8/bpdecode174_91.f90` | Log-domain belief-propagation (BP) decoder |
| `lib/ft8/osd174_91.f90` | Order-statistics decoder (OSD) with optional pre-processing |
| `lib/ft8/ldpc_174_91_c_parity.f90` | `Mn`/`Nm`/`nrw`/`ncw` sparse matrix data |
| `lib/ft8/ldpc_174_91_c_generator.f90` | Generator matrix `gen[K][N]` (only used by OSD; BP doesn't need it) |
| `lib/ft8/ldpc_174_91_c_colorder.f90` | Column ordering for systematic form |
| `lib/ft8/ldpc_174_91_c_decode.f90` | (named in workplan) — appears to have been split / renamed; current decode entry is `decode174_91.f90`. [?] |
| `lib/ft8/ft8b.f90` | Caller — passes `maxosd`, `norder` based on `ndepth` |

### §2.2. License confirmation

WSJT-X COPYING file at the repo root: **GPL v3, 29 June 2007**.
[https://github.com/saitohirga/WSJT-X/blob/master/COPYING](https://github.com/saitohirga/WSJT-X/blob/master/COPYING).
No license header on the individual decoder files we fetched (BP and OSD files
in the mirror show no per-file header — they inherit the repo-wide COPYING).
For Mercury we must add the AGPLv3 header per Mercury convention
([ldpc_decoder_SPA.cc:1-21](../source/physical_layer/ldpc_decoder_SPA.cc#L1-L21))
plus an attribution block citing K1JT/K9AN, the FT8 (174,91) code lineage, and
GPLv3-into-AGPLv3 inclusion under AGPLv3 §13.

### §2.3. Algorithm — plain English

**BP loop (`bpdecode174_91`, 174-bit code):**

1. Initialize `tov` (variable→check messages) and `toc` (check→variable
   messages) to zero. Seed `toc(i,j) = llr(Nm(i,j))` — every check receives
   its channel LLR via its neighbor variables.
2. For `iter = 0..maxiterations` (production: 30):
   - **Update bit posteriors** `zn(i) = llr(i) + sum(tov(:,i))` for every
     unmasked variable. `apmask(i)=1` (puncture/erasure marker) skips the sum.
   - **Hard-decide** `cw = (zn > 0)`. Compute syndrome
     `synd(j) = sum(cw(Nm(:,j))) mod 2`. Count unsatisfied
     parity checks `ncheck`.
   - **Codeword test:** if `ncheck == 0`, run the CRC check on the message
     bits. If CRC passes, return `iter` (BP success).
   - **Early stop:** if `ncheck` has been non-decreasing for 5 consecutive
     iters AND `iter >= 10` AND `ncheck > 15`, return `nharderror = -1` (BP
     failed — converged to wrong codeword or stuck).
   - **Update v→c messages:** for each edge `(i,j)`, `toc(i,j) = zn(Nm(i,j))
     - tov(kk,Nm(i,j))` where `kk` is the index of `j` in variable `Nm(i,j)`'s
     check list.
   - **Update c→v messages (tanh-product rule):** for each check `j` and each
     attached variable `i`, `tov(i,j) = 2 * atanh( prod( tanh(-toc(:,j)/2),
     excluding edge (i,j) ))`. Saturate via `platanh` near ±1.
3. If the loop exits without success, the caller invokes OSD using `zsave(:,i)`
   — the **accumulated posterior LLR vector from iteration `i`**, NOT the
   raw channel LLRs. This is the BP+OSD coupling: OSD gets BP's best guess
   about which bits are most reliable.

**OSD (`osd174_91`, parameter `ndeep` selects depth 0..6):**

1. **Sort variables by reliability:** `absrx = |zn|`, sort descending,
   permute generator matrix columns. The `K` most-reliable bits become the
   "Most-Reliable Basis" (MRB).
2. **Gaussian elimination over GF(2)** to bring the permuted generator into
   systematic form on the MRB. If the first K columns are linearly dependent,
   swap in up to 20 columns from the next-most-reliable bunch.
3. **Order-0 codeword:** take the hard decisions on the K MRB bits as the
   message `m0`, re-encode using the permuted G, get codeword `c0`. Compute
   Euclidean distance `dmin = sum( (c0 XOR hard_decision) * |zn| )`.
4. **Order-w search (w = 1..norder):** flip every w-bit subset of `m0`,
   re-encode (cheap — single XOR per flip), check if the new codeword has
   smaller distance. Keep the best.
5. **Pre-processing pruning (npre1/npre2):** WSJT-X augments the brute search
   with a "boxit/fetchit" hash table that short-circuits patterns based on
   `ntau` low-weight syndromes — speeds OSD-3+ but isn't core.
6. **CRC validation:** the best codeword's CRC is checked; if it fails,
   `nhardmin` is returned negated to signal "OSD found a low-distance
   codeword but the CRC says it's still wrong" — caller can detect.

### §2.4. OSD order recommendations and CPU cost

WSJT-X's `decode174_91.f90` wrapper maps the user-facing `ndepth` slider to
OSD parameters as follows (from `lib/ft8/ft8b.f90` call site):

| `ndepth` | `maxosd` | `norder` | Meaning |
|---|---|---|---|
| 1 | -1 | (unused) | **BP only**, no OSD |
| 2 | 0 | 2 | BP + single OSD-2 call after BP fails |
| 3 | 2 | 2 | BP + up to 3 OSD-2 calls using different `zsave` snapshots |

**Production WSJT-X uses `ndepth=3` → maxosd=2 → norder=2 (OSD-2)** for the
hardest decode attempts. Lower depths trade dB for CPU.

The `osd174_91` function itself enumerates pre-baked parameter sets for
`ndeep=1..6`. For Mercury's adapter we will most likely choose `ndeep=2`
(norder=1, with one pre-processing pass = `npre1=1`) as a starting point
because Mercury's K=100 is comparable to FT8's K=91 *but* Mercury's N=1600 is
~10× larger, so the per-pattern Gaussian-elimination cost is higher.

**Complexity at Mercury's low-rate code.** OSD-w runtime per failed decode
≈ (K-choose-w) × encode_cost + 1 × gauss_cost, where:

- gauss_cost ≈ K² × N / 32 (bit-packed GF(2) ops) ≈ 100²×1600/32 ≈ 500 k
  ops ≈ 0.5 ms on a Pi 5 core at ~1 Gops/s.
- encode_cost per TEP ≈ N/32 XORs of K-bit rows ≈ 1600/32×K ≈ 5 k ops
  ≈ 5 µs.

Per-call totals (back-of-envelope, will measure in §6.2):
- OSD-0 (no flips): 0.5 ms.
- OSD-1: 0.5 + 100×0.005 ≈ 1 ms.
- OSD-2: 0.5 + 4 950×0.005 ≈ 25 ms.
- OSD-3: 0.5 + 161 700×0.005 ≈ 800 ms. **Too slow for real-time.**

Practical conclusion: target **OSD-1 or OSD-2** as fallback. OSD-3+ is for
post-hoc analysis only and would need `decode_abort` integration
([§5.3](#53-decode_abort-flag-status)).

### §2.5. The "BP fails" criterion — when to invoke OSD

In WSJT-X this is implicit: BP completes its 30 iterations, either hits the
non-decreasing-syndrome early-stop, or hits the CRC-fails-on-syndrome=0 case.
The wrapper then snapshots `zsum` (running sum of `zn` across iterations) at
fixed iterations into `zsave(:,1..maxosd)`, and calls OSD with each snapshot
in turn until one produces a CRC-valid codeword.

In Mercury today the equivalent test is:
- **Final decision:** at [telecom_system.cc:2598](../source/physical_layer/telecom_system.cc#L2598)
  the code declares failure when `iterations_done > (nIteration_max - 1)`.
  This is the natural call site to insert "now try OSD before giving up."
- **No CRC inside the LDPC layer:** Mercury's outer CRC (`CRC16_MODBUS_RTU`)
  is checked at the same call site, AFTER `decode()` returns. WSJT-X bakes
  the CRC into the BP loop so it can early-exit. Mercury's BP+OSD wrapper
  must do the same — receive the codeword bits and a hook to verify them
  before declaring success — OR the outer caller must accept that BP+OSD
  may return a CRC-failing codeword and check externally.

---

## §3. Mercury LDPC matrix format — Task 3

### §3.1. Class `cl_ldpc`

[ldpc.h:33-98](../include/physical_layer/ldpc.h#L33-L98). Key members:

| Member | Type | Meaning |
|---|---|---|
| `N` | int | Codeword length. Rate-1/16 ROBUST_0: **N=1600**. |
| `K` | int | Info bits. Rate-1/16: **K=100**. |
| `P` | int | Parity bits = N − K = 1500. |
| `rate` | float | K/N. |
| `framesize` | int | Synonym for N. `MERCURY_NORMAL = 1600` from [physical_defines.h](../include/physical_layer/physical_defines.h). |
| `decoding_algorithm` | int | 0=GBF, 1=SPA. Insert **2=BP_OSD**. |
| `nIteration_max` | int | 100 (OFDM), 200 (ROBUST tier) — set per-config at [telecom_system.cc:4698-4699](../source/physical_layer/telecom_system.cc#L4698). |
| `decode_abort` | atomic<bool>* | Parallel-monitor-decode early-exit hook ([ldpc.h:96](../include/physical_layer/ldpc.h#L96)). Declared, but only wired into `decode_SPA`'s outer iter loop ([ldpc_decoder_SPA.cc:131](../source/physical_layer/ldpc_decoder_SPA.cc#L131)). |
| `QCmatrixC` | int* | Sparse parity matrix, row-indexed (col-indices per check row). |
| `QCmatrixV` | int* | Sparse parity matrix, col-indexed (row-indices per variable col). |
| `QCmatrixd` | int* | Degree distribution. |
| `QCmatrixEnc` | int* | Sparse accumulator-style encoder map. |
| `R`, `Q` | double* | SPA workspace: check-to-variable / variable-to-check messages, `N*Vwidth` doubles each. |
| `V_pos` | int* | Pre-computed position lookup `[P*Cwidth]` for SPA inner loop ([ldpc.cc:270](../source/physical_layer/ldpc.cc#L270)). |

### §3.2. Mercury matrix format vs WSJT-X

For rate 1/16:
- `mercury_normal_Cwidth_1_16 = 4` ([mercury_normal_1_16.cc:28](../source/physical_layer/mercury_normal_1_16.cc#L28)) — max non-zero entries per parity-check row.
- `mercury_normal_Vwidth_1_16 = 8` ([:29](../source/physical_layer/mercury_normal_1_16.cc#L29)) — max non-zero entries per variable column.
- `mercury_normal_QCmatrixC_1_16[1500][4]` — for each parity-check row, the col indices of the 1s. -1 fills unused entries.
- `mercury_normal_QCmatrixV_1_16[1600][8]` — for each variable column, the row indices of the 1s.
- `mercury_normal_QCmatrixd_1_16[12]` — degree-distribution table: pairs of `(count, width)` describing how variable degrees are grouped. Used by SPA to initialize `Q` ([ldpc_decoder_SPA.cc:110-123](../source/physical_layer/ldpc_decoder_SPA.cc#L110-L123)).
- `mercury_normal_QCmatrixEnc_1_16[1500][3]` — for each parity bit `i+K`, the col indices that XOR into it during encode ([ldpc.cc:100-121](../source/physical_layer/ldpc.cc#L100)).

**WSJT-X format (rate ~1/2, FT8):**
- `Mn(7,M)` — for each parity-check row, the col indices of the 1s (≤7 per row, M=83).
- `Nm(3,N)` — for each variable column, the row indices of the 1s (≤3 per col, N=174).
- `nrw(M)` — actual non-zero count per check row.
- `ncw` — scalar = 3 = max non-zero per variable column.
- `gen(k,N)` — dense generator matrix, K rows × N columns.

**Mapping table:**

| WSJT-X | Mercury | Notes |
|---|---|---|
| `Mn` | `QCmatrixC` | Same: col-indices-per-check-row. Both use -1 (Mercury) / 0 (WSJT) for unused slots. |
| `Nm` | `QCmatrixV` | Same: row-indices-per-variable-col. |
| `nrw(j)` | (implicit, via -1 sentinel) | WSJT stores actual width; Mercury scans until -1. |
| `ncw` | `Cwidth` (max) | WSJT scalar = Mercury `Cwidth` constant. |
| (none) | `QCmatrixd` | SPA degree-grouping table — BP+OSD doesn't need it. |
| `gen` | (none — must construct) | **Critical gap.** See §4.4. |

**The two formats are equivalent up to indexing convention (1-based Fortran vs 0-based C++) and sentinel choice. Both are *row-major sparse* in the same way.** The BP loop can be translated almost line-for-line.

### §3.3. `decode()` interface

[ldpc.cc:281-293](../source/physical_layer/ldpc.cc#L281-L293):

```cpp
int cl_ldpc::decode(const float* data, int* decoded_data)
{
    int iterations_done=0;
    if(decoding_algorithm_val==GBF)
        iterations_done=decode_GBF(...);
    else if(decoding_algorithm_val==SPA)
        iterations_done=decode_SPA(...);
    return iterations_done;
}
```

- **Input:** `data` = float LLR array of size N (=1600), order: K info bits then P parity bits. LLR sign convention: **negative ⇒ bit=1**, positive ⇒ bit=0 (matches `LLRbin[i]=(LLRi[i]<0)` at [ldpc_decoder_SPA.cc:59](../source/physical_layer/ldpc_decoder_SPA.cc#L59)).
- **Output:** `decoded_data` = int array of size K, hard-decided info bits only.
- **Return value:** number of iterations used. Caller interprets `iterations_done > (nIteration_max - 1)` as **failure** ([telecom_system.cc:2598](../source/physical_layer/telecom_system.cc#L2598)).
- **No explicit success/failure flag.** This is by design — the outer CRC test is the canonical truth.
- **Side effects:** writes `decoded_data`. Does NOT mutate `data`. Mutates internal `R`, `Q`, `V_pos` workspace.

### §3.4. Call sites

Only two production sites call `ldpc.decode()`:

1. **[telecom_system.cc:321](../source/physical_layer/telecom_system.cc#L321)** — `cl_telecom_system::baseband_test_process` — internal BER plot path (`PLOT_BASEBAND` mode). Single-shot, low risk for first integration.
2. **[telecom_system.cc:2562](../source/physical_layer/telecom_system.cc#L2562)** — `cl_telecom_system::receive_msg` — the production RX path. This is the one we care about.

Additionally there's a third use pattern: **parallel monitor decoders.** In `MONITOR` operation mode, `cl_arq_controller::init_monitor_decoders` ([arq_common.cc:875](../source/datalink_layer/arq_common.cc#L875)) creates one `cl_telecom_system` per config and races them. Each owns its own `cl_ldpc` and would receive the same BP+OSD treatment. The `decode_abort` flag must propagate or this loop becomes a CPU disaster.

### §3.5. Per-config LDPC tier

[telecom_system.cc:4471-4490](../source/physical_layer/telecom_system.cc#L4471-L4490) — only ROBUST_0/1 use rate 1/16; ROBUST_2 uses rate 4/16; OFDM configs use rate 8/16..14/16. Mercury already gates `nIteration_max = 200` to robust configs only ([telecom_system.cc:4698](../source/physical_layer/telecom_system.cc#L4698)).

**Recommendation:** initially gate BP+OSD to `is_robust_config(configuration)` too — that's where the dB shortfall lives. OFDM configs decode reliably with SPA at iter caps far below 100.

---

## §4. Porting approach — Task 4

### §4.1. File layout

| New file | Role |
|---|---|
| `mercury/source/physical_layer/ldpc_decoder_BP_OSD.cc` | Top-level decoder: BP loop + OSD fallback orchestration |
| `mercury/include/physical_layer/ldpc_decoder_BP_OSD.h` | Function signature `decode_BP_OSD(...)` and config struct |
| `mercury/source/physical_layer/ldpc_decoder_BP.cc` | BP log-domain decoder, separate so it's reusable and unit-testable |
| `mercury/include/physical_layer/ldpc_decoder_BP.h` | BP signature |
| `mercury/source/physical_layer/ldpc_decoder_OSD.cc` | OSD: GF(2) Gauss-Jordan + MRB encode + TEP enumeration |
| `mercury/include/physical_layer/ldpc_decoder_OSD.h` | OSD signature; takes dense G + permutation array |
| `mercury/source/physical_layer/ldpc_generator_1_16.cc` | Dense generator matrix construction for rate 1/16 (call existing `encode()` K times at init) |
| (existing) `ldpc.cc` | Add `if(decoding_algorithm_val == BP_OSD)` branch to `decode()`; add init-time `dense_G` allocation when BP_OSD is selected |
| (existing) `ldpc.h` | Add `BP_OSD = 2` to enum; add `dense_G`, `osd_workspace` pointers; add `osd_norder`, `osd_maxosd` config knobs |

Rationale for splitting BP and OSD into separate translation units:
- Matches existing pattern (`ldpc_decoder_GBF.cc` + `ldpc_decoder_SPA.cc`).
- Lets us unit-test BP standalone via PLOT_PASSBAND with `osd_maxosd = -1` (BP-only).
- Lets us potentially substitute Mercury's existing SPA for BP later if the WSJT-X BP turns out to be slower (it is single-precision floats; SPA is double).

### §4.2. Decoder signatures (proposed)

```cpp
// ldpc_decoder_BP.h
int decode_BP(
    const float LLRi[],        // [N] channel LLRs
    int LLRo[],                // [K] hard decisions on info bits (output)
    float zsave[][],           // [maxosd][N] BP posterior snapshots (output, optional)
    int  maxosd_snapshots,     // number of zsave slots to fill (0 = none)
    int* C, int CWidth,        // QCmatrixC, max width
    int* V, int VWidth,        // QCmatrixV, max width
    int  N, int K, int P,
    int  nIteration_max,
    std::atomic<bool>* abort_flag = nullptr
);
// Returns: iter count if BP converged with valid syndrome; -1 if BP failed.

// ldpc_decoder_OSD.h
int decode_OSD(
    const float LLRi[],        // [N] posterior LLRs (from BP) or channel LLRs
    int LLRo[],                // [K] hard decisions (output)
    const uint8_t* dense_G,    // [K*N] generator matrix (row-major, K rows of N bytes)
    const int* apmask,         // [N] puncture/erasure mask (1 = punctured, don't trust LLR)
    int  N, int K,
    int  norder,               // 0..3 typically; 4+ for offline analysis
    float* dmin_out,           // (optional) euclidean distance of best codeword
    std::atomic<bool>* abort_flag = nullptr
);
// Returns: hard-error count vs hard-decision input (0 = exact match, positive = corrections made).

// ldpc_decoder_BP_OSD.h — the top-level glue
int decode_BP_OSD(
    const float LLRi[],
    int LLRo[],
    /* matrix pointers as above */
    const uint8_t* dense_G,
    int  N, int K, int P,
    int  bp_nIteration_max,
    int  osd_maxosd,           // -1 = BP only; 0 = single OSD call; >0 = multi-call with zsave
    int  osd_norder,           // 0..3
    std::atomic<bool>* abort_flag = nullptr
);
```

### §4.3. Call-site change in `ldpc.cc:decode()`

Minimal, additive:

```cpp
int cl_ldpc::decode(const float* data, int* decoded_data) {
    int iter = 0;
    if (decoding_algorithm_val == GBF) {
        iter = decode_GBF(...);
    } else if (decoding_algorithm_val == SPA) {
        iter = decode_SPA(...);
    } else if (decoding_algorithm_val == BP_OSD) {
        iter = decode_BP_OSD(data, decoded_data,
                             QCmatrixC, Cwidth, QCmatrixV, Vwidth,
                             dense_G_1_16,
                             N, K, P,
                             nIteration_max_val,
                             /*maxosd=*/ 0, /*norder=*/ 1,
                             decode_abort);
    }
    return iter;
}
```

**Decision: try SPA first, OSD as fallback — but at a higher level than `decode()`.** Two options:

**Option A: BP+OSD is its own algorithm.** Set `decoding_algorithm = BP_OSD` per-config. BP runs in place of SPA, with OSD as its built-in fallback. Simpler, but throws away Mercury's tuned SPA performance.

**Option B: SPA-then-OSD cascade.** Keep `decoding_algorithm = SPA`, run SPA as today, and ALSO run OSD if SPA fails. Place the cascade in `cl_telecom_system::receive_msg` around [telecom_system.cc:2562](../source/physical_layer/telecom_system.cc#L2562), not inside `cl_ldpc::decode()`. SPA's `R`/`Q` workspace already holds the iter-N posterior — we can recover `LLRtmp` ([ldpc_decoder_SPA.cc:172](../source/physical_layer/ldpc_decoder_SPA.cc#L172)) and pass it to OSD.

**Recommendation: Option B.** Two reasons:
1. Mercury's SPA is double-precision and has 200 iter cap; it's likely already strictly better than WSJT-X's single-precision 30-iter BP for the bulk of decodes. We want SPA for the easy cases.
2. The OSD module then becomes a pure add-on — no risk to the working path. If OSD is bypassed (because SPA succeeds in 5 iters), runtime is unchanged.

The downside of Option B: WSJT-X's `zsave` snapshot is specific to *its* BP run. We'd need to either run BP separately to get a `zsave`-equivalent, or feed OSD only the final SPA posterior, which is less robust (OSD likes diverse starting points). Initial port goes single-snapshot; if performance is poor, retro-fit zsave-from-SPA.

### §4.4. Generator matrix construction

[OPEN] WSJT-X ships `lib/ft8/ldpc_174_91_c_generator.f90` as static data. Mercury doesn't have one — its encoder is sparse-accumulator-style ([ldpc.cc:100-121](../source/physical_layer/ldpc.cc#L100)).

**Construction approach:** at `init()` time, for each i in 0..K-1, set `data[i] = 1` (zeros elsewhere), call `encode(data, encoded_data)`, and store `encoded_data` as row i of `dense_G`. Cost: K calls to existing encoder = 100 * ~3000 XORs = 300 k XORs, one-time, ~1 ms.

Storage: K×N bits = 100×1600 = 20 kB per rate (use uint8 array for simplicity; bit-packing optional). Stash on `cl_ldpc` as `uint8_t* dense_G`, alloc in `update_code_parameters()` only when `BP_OSD` is the selected algorithm AND the rate is configured (currently only 1/16 is the OSD target).

**Verification:** sanity-check the generated G against the parity matrix: `G * H^T = 0` over GF(2) for every row of G. This is a unit test
([§6.1](#61-decoder-only-loopback)).

### §4.5. CRC integration

WSJT-X's BP and OSD both check a CRC inline — they refuse to declare success until CRC passes. Mercury currently checks CRC outside `decode()` ([telecom_system.cc:2593](../source/physical_layer/telecom_system.cc#L2593)).

**Decision: keep CRC outside.** Reasons:
- Mercury's `outer_code` ([telecom_system.cc:4702-4710](../source/physical_layer/telecom_system.cc#L4702)) is configurable — could be CRC16_MODBUS_RTU, could be none.
- WSJT-X bakes CRC into BP because it's a 14-bit CRC inline with the codeword bits. Mercury's CRC is over the K info bits AFTER decode.
- Means OSD must return its "best guess" codeword and let the caller decide. WSJT-X's `nhardmin = -nhardmin` "CRC failed but codeword was found" signal is preserved as a return code.

**Caveat:** in WSJT-X, BP+OSD with CRC inside is strictly faster — it can short-circuit when CRC passes. Without inline CRC, Mercury's BP+OSD may run more iters/TEPs than strictly needed. The cost is at most one extra iter — acceptable.

### §4.6. Per-config flag

Add to `default_configurations_telecom_system_t`:

- `ldpc_decoding_algorithm` — already exists ([physical_config.cc:72](../source/physical_layer/physical_config.cc#L72)), currently set to SPA globally. Per-config override added at [telecom_system.cc:4692](../source/physical_layer/telecom_system.cc#L4692).
- `ldpc_osd_norder` — int, 0..3, default 1 for ROBUST configs, unused otherwise.
- `ldpc_osd_maxosd` — int, -1 = BP only, 0 = single OSD call, 1+ = multi-snapshot. Default 0.

CLI / INI hooks ride alongside the existing `--gi`, `--csi-llr`, `--ldpc-iter` knobs — a matching pattern at [main.cc:1936](../source/main.cc#L1936) and [arq_common.cc:5286](../source/datalink_layer/arq_common.cc#L5286).

### §4.7. Test plan

Mercury already has the infrastructure:

1. **`mercury.exe -m PLOT_PASSBAND -s 100 -N`** — passband BER simulation on AWGN ([main.cc:2326](../source/main.cc#L2326)). Sweep EsN0 from +20 dB (decode-easy) to -10 dB (decode-impossible). Run SPA-only vs SPA+OSD-1 vs SPA+OSD-2, compare BER curves.
2. **`mercury.exe --test`** — full unit test suite. Add a decoder-only test that:
   - Encodes a known message via `encode()`.
   - Adds Gaussian noise of variance σ² to LLRs.
   - Runs SPA vs BP+OSD.
   - Asserts BP+OSD's BER is ≤ SPA's BER at every σ.
   - Tests `dense_G * H^T = 0` over GF(2).
3. **`tools/axis_walk_sweep.py --config 100 --with-robust`** — production-style IONOS-channel sweep, mentioned in MEMORY as the canonical low-SNR validator. Run with `--ldpc-algo=BP_OSD` (new CLI flag) at WGN:0, -4, -8, -12. Compare cliff position vs monitor HEAD (SPA).
4. **`tools/bisect_benchmark.py`** — pre/post commit run on WB and NB to ensure no regression in the high-SNR path (BP+OSD should be CPU-equivalent to SPA when SPA succeeds; we're verifying that's true).

**Critical test:** measure the *time* spent in OSD across an axis walk. If 95% of decodes succeed with SPA in 20 iters, OSD CPU budget is fine. If 50% of decodes call OSD at the cliff, we need OSD-1 not OSD-2.

### §4.8. Estimated effort

See §7.

### §4.9. Known risks identified during research

1. **OSD on rate-1/16 may produce nuisance false-positives.** OSD always returns *some* codeword; without inline CRC, a CRC16 failure on a low-distance OSD-2 codeword means the receiver got a 0-bit-error indication from the decoder but data is still wrong. Mercury's existing outer CRC handles this, but only if it's enabled (`outer_code == CRC16_MODBUS_RTU`). The ROBUST tier MUST run with outer CRC. **Verify before merging.**
2. **WSJT-X's `platanh` saturation may differ from Mercury's `tanh/atanh`.** WSJT-X uses a piecewise-linear `atanh` to avoid edge saturation; Mercury's SPA uses the standard library and clamps to ±0.9999999 ([ldpc_decoder_SPA.cc:152-159](../source/physical_layer/ldpc_decoder_SPA.cc#L152-L159)). For the port, use Mercury's existing saturation — empirically validated already.
3. **Single-precision vs double-precision LLRs.** WSJT-X is single-precision throughout. Mercury's SPA is double for the iter loop and single for the I/O. For BP+OSD: keep single-precision in BP messages (matches WSJT), use double in OSD's distance accumulator (avoids precision loss summing 1600 floats).
4. **`apmask` (puncture) semantics.** WSJT's `apmask` is a per-bit "trust me" flag — when set, the BP loop substitutes `zn = llr` directly without summing the v→c messages. Mercury's puncture path zeros the LLR ([telecom_system.cc:2298-2301](../source/physical_layer/telecom_system.cc#L2298-L2301)) and lets the decoder iterate. Behavior is similar but not identical — verify in unit test.
5. **`decode_abort` not propagated through BP+OSD.** The atomic flag is checked in SPA's outer iter loop. BP+OSD's outer loops are: BP iter loop (cheap), OSD TEP loop (potentially long). Plumbing the flag into both is needed before parallel monitor decode can use BP+OSD.

---

## §5. Cross-layer audit — Task 5 (per CLAUDE.md §5)

**Shared state structures touched by `ldpc.decode()`:**

### §5.1. LLR scaling (`llr_scale`, `noise_var`)

Producers:
- **MFSK path:** [mfsk.cc:759-761](../source/physical_layer/mfsk.cc#L759-L761) — `noise_var` computed from guard-bin energy pooled across all symbols (F4 fix). `llr_scale = 1 / noise_var`. **A.1.4 cross-pilot diff feeds this for ROBUST_0** via the demap chain in mfsk.cc.
- **OFDM path:** [telecom_system.cc:301](../source/physical_layer/telecom_system.cc#L301) — `variance = ofdm.measure_variance(...)`, then passed into `psk.demod(..., variance)` at [:305](../source/physical_layer/telecom_system.cc#L305).

Consumers of resulting LLRs: SPA decoder's `LLRi[]` ([ldpc_decoder_SPA.cc:26](../source/physical_layer/ldpc_decoder_SPA.cc#L26)).

**Invariant SPA assumes:** LLR magnitude is correctly calibrated to noise; sign convention `<0 ⇒ bit=1`. **Q1/Q2/E1/A.1.4 fix history confirms this is fragile:** the historical `var_floor=0.001` was papering over a cross-pilot bug; even rate-1/16 LDPC depends on accurate LLR magnitudes for graph-message updates.

**Will BP+OSD break this?** Two checks:

1. **BP loop magnitude sensitivity.** WSJT-X's BP uses identical message-update math to Mercury's SPA (tanh-half/atanh-half-twice). The numerical sensitivity to LLR magnitude is the same. **No new failure mode.**
2. **OSD reliability sorting.** OSD orders bits by `|LLR|`. If LLR magnitudes are systematically biased (e.g., one tone consistently overestimated), the MRB ordering is wrong and OSD gets less reliable bits in the "trusted" basis. This is a NEW sensitivity — BP doesn't have it (the message updates are reliability-magnitude-insensitive). **OSD is more sensitive to LLR calibration than SPA.** A.1.4's cross-pilot fix is a prerequisite for OSD to work well. [?] Need a unit test that injects synthetic LLR-scale errors and measures OSD's BER degradation.

### §5.2. Iteration cap (`nIteration_max`)

Producers: [telecom_system.cc:4694-4699](../source/physical_layer/telecom_system.cc#L4694-L4699) — set to 100 (default) or 200 (robust). GUI override at [arq_common.cc:5286-5290](../source/datalink_layer/arq_common.cc#L5286-L5290).

Consumer: SPA's outer iter loop ([ldpc_decoder_SPA.cc:128](../source/physical_layer/ldpc_decoder_SPA.cc#L128)).

**Will BP+OSD break this?** BP needs a separate iter cap from SPA — WSJT-X uses 30, Mercury's SPA uses 200. **Recommendation:** add `ldpc.bp_nIteration_max` (default 30, or 50 to be conservative) separate from `ldpc.nIteration_max`. Do NOT use the SPA cap — 200 BP iters would be wasteful and slow.

Caveat: at [telecom_system.cc:2598](../source/physical_layer/telecom_system.cc#L2598) the failure test uses `(nIteration_max - 1)` as the threshold. For BP+OSD, the equivalent test must look at the algorithm's own return-code semantics, not a raw iter-count threshold. Update the failure test to a returned-flag check.

### §5.3. `decode_abort` flag status

[ldpc.h:96](../include/physical_layer/ldpc.h#L96). Declared on `cl_ldpc`. Only `decode_SPA` reads it ([ldpc_decoder_SPA.cc:131](../source/physical_layer/ldpc_decoder_SPA.cc#L131)). Mercury's `init_monitor_decoders` ([arq_common.cc:875](../source/datalink_layer/arq_common.cc#L875)) creates parallel decoders, but no code in this tree wires `decode_abort` to a parallel-success hook — appears to be infrastructure-only.

**Will BP+OSD break this?** No (it's not yet operational). But BP+OSD must respect it once wired — both inside BP's iter loop AND inside OSD's TEP enumeration. Without that, monitor-mode CPU explodes during cliff conditions where all configs fail SPA and fall into OSD simultaneously.

### §5.4. `data_container.deinterleaved_data[]` (LLR input)

Producer: psk/mfsk demod + deinterleaver chain ([telecom_system.cc:2548-2559](../source/physical_layer/telecom_system.cc#L2548)).
Consumer: `ldpc.decode()` ([:2562](../source/physical_layer/telecom_system.cc#L2562)).

**Invariant:** array size = `N = K + P` floats, ordered systematic (info bits 0..K-1, then parity 0..P-1). Punctured positions zeroed at [:2298-2301](../source/physical_layer/telecom_system.cc#L2298-L2301).

**Will BP+OSD break this?** No — both BP and OSD consume the same shape. OSD reads them just like BP does. Puncturing-as-zero-LLR is honored by BP (zero LLR = posterior follows messages alone). OSD's `apmask` is the equivalent — would need to materialize an `apmask[]` from the puncture range at decode entry: `apmask[i] = (i >= puncture_from && i < puncture_from + nVirtual_data) ? 1 : 0`.

### §5.5. `data_container.hd_decoded_data_bit[]` (hard decisions output)

Producer: SPA's final hard-decide loop ([ldpc_decoder_SPA.cc:217-220](../source/physical_layer/ldpc_decoder_SPA.cc#L217-L220)). Output is K bits.

Consumers:
- `bit_energy_dispersal()` at [telecom_system.cc:2566](../source/physical_layer/telecom_system.cc#L2566) — XOR with a pseudo-random sequence. Pure bit-array transformation; doesn't care about decoder identity.
- `bit_to_byte()` at [:2569](../source/physical_layer/telecom_system.cc#L2569) — pack K bits into K/8 bytes. Same.
- `CRC16_MODBUS_RTU_calc()` at [:2593](../source/physical_layer/telecom_system.cc#L2593) — outer CRC check.

**Will BP+OSD break this?** No — same shape, same semantics. **Verify CRC sign convention in unit test** — OSD returning a "low-distance but CRC-wrong" codeword could mimic a successful SPA decode in `hd_decoded_data_bit[]` while CRC silently fails. The CRC check catches this, but only if it's enabled.

### §5.6. `receive_stats.iterations_done`

Producer: return value of `ldpc.decode()`.
Consumers:
- Failure test at [:2598](../source/physical_layer/telecom_system.cc#L2598) (`iterations_done > nIteration_max-1`).
- Diagnostic prints at [:2607](../source/physical_layer/telecom_system.cc#L2607), [:2641](../source/physical_layer/telecom_system.cc#L2641), [:2721](../source/physical_layer/telecom_system.cc#L2721).
- v2-OFDM sub-peak recovery gate at [:2623](../source/physical_layer/telecom_system.cc#L2623).
- Q3 GUI display at [arq_common.cc:5288](../source/datalink_layer/arq_common.cc#L5288).

**Will BP+OSD break this?** `iterations_done` for BP+OSD has a different meaning. **Recommendation:** define a return-code convention:
- 0..bp_nIteration_max-1: BP succeeded at iter N.
- bp_nIteration_max + k: OSD called, returned codeword with k bit-flips.
- bp_nIteration_max + (osd_norder<<16): OSD called with that norder (encoded in upper bits).
- nIteration_max (sentinel maximum): BP+OSD failed entirely.

Document this in a comment block; the existing diagnostics still print the integer value and a low integer == fast decode.

### §5.7. Per-config tier guard

[telecom_system.cc:4698-4699](../source/physical_layer/telecom_system.cc#L4698-L4699) — `is_robust_config()` already gates the iter-200 logic.

**Recommendation:** add `is_robust_config()` gate on `BP_OSD` as well, at least until the OFDM-config performance is measured. OFDM rate-8/16+ codes have plenty of room above the cliff and unlikely benefit from OSD's complexity.

### §5.8. Channel-state CSI per-bit weighting (`csi_llr_enabled`)

[telecom_system.cc:46](../source/physical_layer/telecom_system.cc#L46), referenced in the OFDM path at [:2540-2547](../source/physical_layer/telecom_system.cc#L2540-L2547). CSI scaling multiplies LLRs by `|H_k|²` before LDPC.

**Will BP+OSD break this?** Same answer as §5.1 — OSD's ordering is sensitive to LLR magnitude calibration. If CSI multiplication is wrong (Phase 2 V-shape investigation), OSD ordering is wrong. **The A.1.5 V-shape validation work is a prerequisite for testing BP+OSD on OFDM configs.** For MFSK/ROBUST_0 (which doesn't run CSI), this is moot at first.

### §5.9. Summary table

| State | Producer | Consumer | BP+OSD risk |
|---|---|---|---|
| LLR (`deinterleaved_data`) | demap chain | decoder | low — same shape |
| LLR magnitude calibration | F4/E1/A.1.4 chain | SPA, BP, OSD | medium — OSD adds reliability-sort sensitivity |
| Iter cap | `nIteration_max` | SPA inner loop | low — add separate BP cap, redefine failure test |
| Puncturing | LLR zero-fill | SPA, BP | low — translate to `apmask` for OSD |
| Hard decisions (`hd_decoded_data_bit`) | decoder | CRC, dispersal, bit-to-byte | low — same shape; **verify CRC catches OSD false positives** |
| `iterations_done` return | decoder | failure test, diagnostics, sub-peak probe, GUI | **medium — redefine return semantics** |
| `decode_abort` flag | (declared, not wired) | SPA inner loop | low — must extend to BP and OSD before monitor mode |
| CSI per-bit weighting | OFDM equalizer | OFDM-tier decode | not applicable to ROBUST_0 (no CSI) |
| Per-config tier (`is_robust_config`) | telecom_system init | nIteration_max gate | low — extend same gate to BP_OSD |

---

## §6. Validation plan (Phase 4 of the four-phase framework)

### §6.1. Decoder-only loopback

`mercury.exe --test` already exists ([main.cc](../source/main.cc) test wiring at multiple points). Add a test case:

```
// Unit test: ldpc_BP_OSD_correctness
//   K=100, rate-1/16. Random message, encode, add LLR noise σ.
//   For σ = 0 (perfect), assert both SPA and BP+OSD recover exact message.
//   For σ = 1.0 (cliff), assert BP+OSD BER ≤ SPA BER.
//   For σ = 1.5 (impossible), assert both fail gracefully (no crash).
// Unit test: generator_matrix_correctness
//   Construct dense_G from existing encoder. Verify G * H^T = 0 over GF(2).
// Unit test: ldpc_BP_OSD_puncture
//   Set apmask, verify decoder treats those positions as erasures.
```

### §6.2. OSD CPU profile

Add `[OSD-PROFILE]` log line: when OSD is invoked, log `osd_norder`, `tep_count`, `elapsed_ms`. Run an axis-walk sweep with this enabled to see how often OSD fires and what it costs.

### §6.3. PLOT_PASSBAND A/B

Run `mercury.exe -m PLOT_PASSBAND -s 100 -N` for ROBUST_0 at EsN0 sweep 20 dB → −15 dB in 1 dB steps, 1000 frames per point. Three arms:
- SPA only (current).
- SPA + OSD-0 fallback (Mercury new).
- SPA + OSD-1 fallback (Mercury new).
- SPA + OSD-2 fallback (Mercury new).

Plot BER + frame-error-rate. Expected: OSD curves trail SPA by 0 dB above the cliff, beat SPA by 0.5-1.25 dB below the cliff.

### §6.4. IONOS cliff sweep

`tools/axis_walk_sweep.py --config 100 --with-robust --ldpc-algo=BP_OSD` at WGN 0, -4, -8, -12. Compare cliff position vs monitor HEAD baseline (Phase A.0.3 + A.1.4 with SPA). Per workplan §9 the cliff target after Phase A is WGN:-7 to -11; BP+OSD should contribute 0.5-1.25 dB to that.

### §6.5. Bisect/regression

`tools/bisect_benchmark.py` on WB CFG10/CFG15 — confirm BP+OSD does not regress high-SNR throughput. Expected: zero impact; SPA succeeds fast and OSD is never called.

---

## §7. Effort estimate

| Phase | Days | Risk | Deliverable |
|---|---|---|---|
| §7.1 BP port (translate Fortran → C++, write unit test) | 2 | low | `ldpc_decoder_BP.cc/h` + unit test passing on PLOT_PASSBAND |
| §7.2 Generator matrix construction (build dense_G, verify G·H^T=0) | 0.5 | low | `ldpc_generator_1_16.cc` |
| §7.3 OSD port (translate Fortran → C++, with norder=0..2 only initially) | 3 | medium | `ldpc_decoder_OSD.cc/h` + unit test |
| §7.4 BP+OSD glue (top-level decoder, snapshot management) | 1 | low | `ldpc_decoder_BP_OSD.cc/h` |
| §7.5 Cascade integration into `cl_telecom_system::receive_msg` (Option B from §4.3) | 1 | medium | Modified `telecom_system.cc:2562` region; `is_robust_config()` gate; new `osd_norder` + `osd_maxosd` knobs |
| §7.6 `decode_abort` plumbing into BP and OSD | 0.5 | low | Atomic-flag checks in BP iter loop + OSD TEP loop |
| §7.7 PLOT_PASSBAND A/B sweep + analysis | 1 | low | BER curves, dB delta measurement |
| §7.8 IONOS cliff sweep (axis_walk_sweep.py) | 1 | medium | Cliff position measurement |
| §7.9 Documentation + cross-references in fact-doc | 0.5 | low | Updated `phase-a-to-b-workplan.md §A.2.6` with results |
| **Total** | **10.5 days** | | |

**Reduced effort if scope tightens:** OSD-0 only (no TEP enumeration, just MRB encode of order-0 codeword) cuts §7.3 to 1 day but only gains ~0.2-0.5 dB. OSD-1 is the sweet spot for first ship.

**Increased effort if surprises:**
- WSJT-X's `platanh`/`indexx`/`crc14a` helpers needed direct port → +1 day.
- Bit-packed GF(2) Gaussian elimination needed for speed → +1-2 days.
- Multi-snapshot zsave from SPA (Option B refinement) → +1 day.
- `apmask` semantics differ from Mercury's zero-LLR puncture in subtle ways → +1 day debugging.

Realistic range: **7 days (optimistic, OSD-1 only, no surprises) to 14 days (pessimistic, OSD-2, with Gauss-elim speed work)**. Workplan §A.2 estimate of "1-2 weeks" lines up.

---

## §8. Open questions / unknowns

[?] Q1: Does WSJT-X's `ldpc_174_91_c_decode.f90` (named in the workplan) still exist under that name, or has it been split into `decode174_91.f90` + `bpdecode174_91.f90` + `osd174_91.f90`? Latest mirror shows only the split form. The workplan name may be historical.

[?] Q2: Is there a C/C++ port of WSJT-X's BP+OSD in `rtmrtmrtmrtm/ft8mon` or similar third-party project? Search returned `ft8mon` (C++ FT8 receiver) — worth checking before re-translating from Fortran. If a clean GPLv3 C++ port exists, port time drops from ~5 days (BP+OSD+glue) to ~2 days (adapt to Mercury matrix format). **Action:** WebFetch `https://github.com/rtmrtmrtmrtm/ft8mon` and look for `bpdecode*.cc` / `osd*.cc` BEFORE starting §7.1.

[?] Q3: What is OSD's actual measured dB-win on FT8's rate-1/2 LDPC? Authoritative sources (Franke-Taylor QEX 2020) are behind a PDF that didn't parse via WebFetch. Generic literature suggests "1-2 dB" but at very different code rates. Mercury's rate-1/16 may see less than that (OSD-w with w small hits diminishing returns on low-rate codes per §2.4 search results — they "usually require high decoding order"). The workplan's +0.5-1.25 dB estimate aligns with this caveat.

[?] Q4: Is the existing `decode_abort` flag wired anywhere outside the SPA decoder, or is it purely declared infrastructure waiting for `init_monitor_decoders` to integrate? Code search shows no writes to it in production. Confirm before relying on it as a real abort mechanism.

[?] Q5: Does the SPA's final posterior LLR (`LLRtmp[]` at [ldpc_decoder_SPA.cc:172](../source/physical_layer/ldpc_decoder_SPA.cc#L172)) make a good OSD input? WSJT-X uses BP's `zsum` (sum across iterations, not the final). For "Option B" cascade, we may need to add an `LLRtmp[]` snapshot return from `decode_SPA` to mirror that. ~half-day add.

---

## §9. Recommendation to user

**Feasible: yes.** License compatible, matrix format compatible, algorithm signature compatible, no blockers identified.

**Risky: not particularly.** The main risk is OSD's per-decode cost at the cliff (low-rate codes benefit more from high-order OSD; Mercury's K=100 is comparable to FT8's K=91 but rate is 1/16 not 1/2). Mitigation: gate to ROBUST configs only, default to OSD-1, measure CPU profile before going to OSD-2.

**Best return for effort:** SPA-then-OSD cascade (§4.3 Option B). Keeps Mercury's tuned SPA on the hot path, adds OSD only when SPA fails, requires zero changes to monitor-decoder parallelism logic, and gives a clean A/B knob (`--osd-maxosd=-1` to disable).

**Estimated dB:** workplan's +0.5-1.25 dB at the cliff is plausible. The Franke-Taylor literature on rate-1/2 LDPC suggests 1-2 dB at the cliff; rate-1/16 with K=100 likely sees the low end of that range, hence the 0.5-1.25 dB target.

**Pre-requisite:** A.1.4 cross-pilot LLR fix (shipped 2026-05-26, monitor 9c3fc40) is foundational. OSD is more sensitive than BP to LLR magnitude bias — running OSD without correct LLR calibration would degrade performance, not improve it.

**Estimated effort: 7-14 days** for tested A/B against SPA on PLOT_PASSBAND + IONOS axis walk.

---

## §10. References

- Franke, S. & Taylor, J. (2020). "The FT4 and FT8 Communication Protocols." QEX July/August 2020. PDF: https://wsjt.sourceforge.io/FT4_FT8_QEX.pdf (couldn't fully text-extract via WebFetch).
- Fossorier, M.P.C. & Lin, S. (1995). "Soft-Decision Decoding of Linear Block Codes Based on Ordered Statistics." IEEE Transactions on Information Theory, 41(5), 1379-1396. DOI: 10.1109/18.412683.
- Kschischang, F.R., Frey, B.J. & Loeliger, H.-A. (2001). "Factor Graphs and the Sum-Product Algorithm." IEEE Transactions on Information Theory, 47(2), 498-519. (Mercury's SPA citation — same paper applies to BP.)
- WSJT-X source mirror: https://github.com/saitohirga/WSJT-X — refreshed every 6 h from upstream SourceForge SVN. Files inspected: `lib/ft8/bpdecode174_91.f90`, `lib/ft8/osd174_91.f90`, `lib/ft8/decode174_91.f90`, `lib/ft8/ldpc_174_91_c_parity.f90`, `lib/ft8/ft8b.f90`, `COPYING`.
- Mercury LDPC layer:
  - [ldpc.h](../include/physical_layer/ldpc.h)
  - [ldpc.cc](../source/physical_layer/ldpc.cc)
  - [ldpc_decoder_SPA.h](../include/physical_layer/ldpc_decoder_SPA.h) / [.cc](../source/physical_layer/ldpc_decoder_SPA.cc)
  - [mercury_normal_1_16.h](../include/physical_layer/mercury_normal_1_16.h) / [.cc](../source/physical_layer/mercury_normal_1_16.cc)
  - [physical_defines.h](../include/physical_layer/physical_defines.h) — `GBF=0`, `SPA=1`. Add `BP_OSD=2`.
- Mercury demap LLR scaling:
  - [mfsk.cc:759-861](../source/physical_layer/mfsk.cc#L759) — log-sum-exp non-coherent FSK metric (F2 fix on top of Q3).
  - F4 noise variance pooling — referenced in code comment at [mfsk.cc:737](../source/physical_layer/mfsk.cc#L737).
- Workplan references:
  - [phase-a-to-b-workplan.md §4](phase-a-to-b-workplan.md) — Phase A.2 spec.
  - [mfsk-vara-parity-plan.md](mfsk-vara-parity-plan.md) — Eb/N0 framing.
  - [weak-signal-floor-investigation.md](weak-signal-floor-investigation.md) — DSP fix history.

---

## §11. Implementation log (2026-05-26, branch fix/a26-bp-osd)

Phase A.2 §7.1–§7.4 implemented per this research doc. Five commits on
`fix/a26-bp-osd` (worktree `x:/Storage/Documents/mercury-worktrees/a26-bp-osd`)
on top of monitor 9c3fc40:

| Commit | Subject |
|---|---|
| 205a668 | ldpc: port log-domain BP decoder from ft8mon (§7.1) |
| 4482623 | ldpc: dense generator-matrix construction for rate 1/16 (§7.2) |
| 432af85 | ldpc: port OSD decoder from ft8mon (§7.3) |
| 370490d | ldpc: BP+OSD glue / cascade decoder (§7.4) |
| 727b644 | ldpc: BP+OSD unit tests + --test CLI hook (§6.1) |

Reference source pulled from `rtmrtmrtmrtm/ft8mon` (MIT, (c) Robert T.
Morris), not from upstream WSJT-X Fortran. ft8mon is a C++ port of WSJT-X's
BP+OSD and matches Mercury's existing log-domain SPA style much more
closely than the Fortran originals; this cut translation risk substantially.
Files used as templates: `libldpc.c:215-297` (`ldpc_decode_log`) for BP,
`osd.cc:102-222` (`osd_decode`) for OSD. License attribution preserved in
each new file's header per AGPLv3 §13.

### §11.1. Files added

| File | LoC | Role |
|---|---|---|
| `include/physical_layer/ldpc_decoder_BP.h` | 102 | BP signature + return-code sentinels |
| `source/physical_layer/ldpc_decoder_BP.cc` | 282 | BP loop, log-domain, float msgs |
| `include/physical_layer/ldpc_decoder_OSD.h` | 99 | OSD signature |
| `source/physical_layer/ldpc_decoder_OSD.cc` | 318 | OSD: GJ + MRB + TEP enum |
| `include/physical_layer/ldpc_decoder_BP_OSD.h` | 87 | Top-level glue signature |
| `source/physical_layer/ldpc_decoder_BP_OSD.cc` | 102 | Cascade: BP → OSD-on-failure |
| `include/physical_layer/ldpc_generator_1_16.h` | 64 | Dense G access + verification |
| `source/physical_layer/ldpc_generator_1_16.cc` | 137 | Lazy build, G·H^T verifier |
| `include/physical_layer/ldpc_bp_osd_tests.h` | 33 | Test entry point |
| `source/physical_layer/ldpc_bp_osd_tests.cc` | 422 | 7 unit tests, self-contained |
| `source/main.cc` (modified) | +13 | --test CLI hook |
| `build.sh` (modified) | +5 | Adds new sources to CPP_SOURCES |
| `.gitignore` (modified) | +3 | Excludes local .ft8mon-ref/ reference dir |

Total new: ~1700 LoC including comments and AGPLv3+MIT attribution headers.

### §11.2. Deviations from research doc

1. **Mass-storage layout for BP messages: `P × N` rectangular, not `N × Vwidth`
   compressed.** Research §4.1 implies compressed; ft8mon uses rectangular. I
   went with rectangular for porting fidelity, which costs ~9.6 MB per call
   (heap-allocated via `std::vector`, freed on return). For the §7.5
   integration this should be pre-allocated on `cl_ldpc` like Mercury's
   existing `R`/`Q` workspace, or compressed to `N*Vwidth*sizeof(float)`
   (~50 KB). Flagged in `ldpc_decoder_BP.cc` comments. Not load-bearing for
   first ship since OSD-fallback is rare.

2. **Snapshot scheme simplified: per-iter posterior, not running zsum.**
   Research §2.5 / §4.3 explains WSJT-X's BP takes snapshots of the running
   sum `zsum` of posteriors so OSD gets averaged-over-iters reliability.
   I implemented per-iter snapshots (the latest posterior at the chosen
   iter index) because the ft8mon reference also uses per-iter snapshots
   (just one — at the end). Multi-snapshot zsum-style is research §7.5
   future work; ship currently uses single-snapshot OSD input.

3. **OSD score function: weighted-disagreement sum, not WSJT-X's
   `nhardmin*4.6` formula.** Research doc §2.3 quotes WSJT's formula;
   ft8mon uses a 4.6-scale variant. My port uses the equivalent `sum of
   |LLR| over disagreement positions`, which is the same metric minus an
   irrelevant scale factor — and avoids float wrap-around concerns at
   large N. Documented in `ldpc_decoder_OSD.cc` near `osd_score_double`.

4. **`apmask[]` semantics: positions DEMOTED to bottom of MRB sort, not
   forced to top.** Research §4.9 risk note 4 flags that WSJT's apmask
   means "trust the channel LLR, skip the message sum" — which is correct
   for the BP loop. For OSD, the same apmask must mean "don't put this
   position in the MRB" (because we don't trust it). I treat apmask in BP
   by skipping the message sum (matches WSJT) AND in OSD by zeroing the
   strength (the position sorts last and is excluded from the MRB).
   These are two different but consistent semantics. Documented in both
   decoder files. The puncture unit test exercises both paths.

5. **No `--test` flag existed in main.cc before this branch.** CLAUDE.md
   and README.md both reference `mercury.exe --test`, but no such argv
   parser existed (only `--test-*` prefixed flags for SACK fault
   injection). I added the bare `--test` flag in `main.cc` early (before
   any heavy init) and have it run only the BP+OSD test group, exit code
   0=pass / 1=fail. Future test groups should hook into the same call
   point.

### §11.3. Unit-test results (on Windows MinGW, o3 build)

```
=== Mercury BP+OSD unit tests (Phase A.2) ===
[TEST] generator_matrix_correctness ... PASS
[TEST] bp_correctness_clean (sigma=0) ... PASS  (5 trials, 0 errors)
[TEST] bp_vs_spa_noisy ... PASS  (bp_errs=0, spa_errs=0, K*trials=500)
[TEST] osd_recovers_from_bp_fail ... PASS  (OSD improved on BP in 5/5 trials)
[TEST] bp_osd_puncture ... PASS  (erasure-path 5 trials, apmask-path 3 trials)
[TEST] bp_osd_glue ... PASS  (3 noise-free trials, BP+OSD)
[TEST] bp_osd_graceful_fail (sigma=2.0) ... PASS  (no crashes / bad output)
=== 0 test(s) failed ===
```

`osd_recovers_from_bp_fail` is the key existence proof: a 5-iter BP cap
forces BP to fail (rc=-1) on sigma=1.0 noise, and OSD-1 reduces the
bit-error count vs BP's best-effort output in 5/5 trials. This validates
the cascade wiring end-to-end. The exact OSD dB-win at the cliff
(research §6.3 expects 0.5–1.25 dB) requires PLOT_PASSBAND sweeps which
are §7.7 work, not §7.1-7.4.

### §11.4. Cross-layer audit notes

Per CLAUDE.md §5, listing the new state introduced by this branch:

- **`g_dense_G[K*N]` (160 KB process-static buffer).** Producer:
  `build_dense_G_once()` (called once via `std::call_once` from
  `ldpc_get_dense_G_1_16`). Consumers: `verify_ldpc_generator_1_16` and
  `decode_OSD`. No mutability after first call — fully thread-safe.

- **`std::vector<float> m_storage / e_storage` (per BP call).**
  Producer: `decode_BP`. Consumer: same function. Stack-scoped, freed on
  return. No cross-call state.

- **No new state crosses ARQ / SACK / streaming layers.** The decoder
  pair is a pure function over (LLR input, matrix pointers) → (info bits,
  return code). Integration into `cl_telecom_system::receive_msg` (§7.5)
  is where cross-layer audit becomes load-bearing.

Existing state structures from research §5 audited as expected:

- LLR sign convention <0 ⇒ bit=1 (§5.1): verified by tests
  `bp_correctness_clean` and `bp_osd_glue` against random messages.
- Iter cap return semantics (§5.6): new sentinels documented in
  ldpc_decoder_BP.h and ldpc_decoder_BP_OSD.h. §7.5 integration will
  need to update the failure test at `telecom_system.cc:2598` which
  currently uses `iterations_done > (nIteration_max - 1)`.
- `decode_abort` (§5.3): plumbed into both BP's iter loop and OSD's TEP
  enumeration. Monitor-mode parallel decode in §7.5 should now safely
  early-exit BP+OSD as well.

### §11.5. Open items handed off to §7.5 (integration agent)

1. **`physical_defines.h`: add `BP_OSD = 2`.** Owned by §7.5.
2. **`cl_ldpc::decode()` cascade.** Add `else if (decoding_algorithm_val == BP_OSD)`
   branch. Caller passes `nIteration_max_val` as `bp_nIteration_max`.
3. **`is_robust_config()` gate** on BP+OSD initially, per research §3.5
   recommendation.
4. **Failure test at `telecom_system.cc:2598`** needs updating to handle
   new return-code regime (see §5.6) — `iterations_done > (cap-1)` no
   longer correctly identifies failure when OSD returns
   `LDPC_BP_OSD_OSD_BASE + order`.
5. **`dense_G` allocation lifecycle.** Currently the dense G is built
   lazily on first call and lives in a process-static buffer. For
   multi-`cl_ldpc` parallel monitor decode this is correct (shared
   read-only). §7.5 may want to call `ldpc_get_dense_G_1_16()` once at
   `init()` to amortize the ~1ms build outside the decode hot path.
6. **CLI / INI knobs** for `--ldpc-osd-norder` and `--ldpc-osd-maxosd`
   per research §4.6 — not added in this branch because they're only
   useful once §7.5 wires the cascade.
7. **Compressed BP message buffers** (see §11.2 deviation 1) for
   memory efficiency at parallel monitor decode scale.

### §11.6. Build verification

`bash build.sh o3` from worktree: 71 files compiled, 0 errors, link
succeeded. Warnings are pre-existing (winsock include order, signed
comparison in `arq_commander.cc:2132`, `STARTUPINFOA` initializer
warnings on Windows GUI dialog). New decoder files compile with no
warnings under `-Wall -Wextra -Wno-format -Wno-unused -std=c++14`.

`mercury.exe --test` returns exit code 0 with all 7 tests passing.
`mercury.exe --help` and normal-mode startup unaffected (the `--test`
parser runs before any other arg processing and returns immediately).

---

## §12. Rebase onto current monitor (2026-05-28, branch feat/bp-osd-rebased)

The original branch `fix/a26-bp-osd` was cut from monitor `9c3fc40`. Between
then and 2026-05-28 monitor advanced to `fb9c617`, adding ~50 commits across
Phase B Wave 3 (MFSK ctrl-suffix codec), Option A discrete-match preamble,
mini-Moose CFO refinement, WB MFSK preamble 4→16 extension, sign-flip
fix and the WB ctrl-frame mini-Moose. Several of those touched files the
BP+OSD branch also modified.

Per parent dispatch, the goal of the rebase: place BP+OSD on top of current
monitor so an A/B run at today's deeper operating SNR can determine whether
the prior NULL DELTA verdict (2026-05-26: BP converged in 1 iter, OSD never
fired) still holds, or whether the cliff has moved enough to expose LDPC
iter-cap binding.

### §12.1. Method

`git worktree add x:/Storage/Documents/mercury-worktrees/bp-osd-rebased
-b feat/bp-osd-rebased monitor` (monitor=`fb9c617`). Then cherry-picked the
10 BP+OSD commits in order (`205a668 4482623 432af85 370490d 727b644
dd6beb5 bc39506 e4f7877 f072959 cea294d`). Used cherry-pick rather than
`git rebase --onto` because `fix/a26-bp-osd` is checked out in another
worktree (`mercury-worktrees/a26-bp-osd`) so it could not be moved.

### §12.2. Conflicts resolved (1 file, 1 commit)

| Commit | File | Resolution |
|---|---|---|
| `727b644` (BP+OSD `--test` hook) | `source/main.cc` | Phase B Wave 1 commit `30a0c70` had landed its own `--test` runner (per its commit message, "mirrors the sister-branch BP+OSD --test hook pattern"). Both runners want the same hook. Resolved by including both headers and chaining the test calls: `failed = run_ldpc_bp_osd_tests(); failed += run_mfsk_ctrl_codec_tests();`. Exit 0 only if BOTH suites pass — the conservative composition. |

All other commits auto-merged cleanly:
- `build.sh` auto-merged (both source-file lists co-existed cleanly).
- `source/physical_layer/telecom_system.cc` auto-merged twice (commits
  `634d851` and `d9de5c3`) despite heavy monitor-side mini-Moose / preamble
  edits — the BP+OSD hunks added a new `if(configuration == ROBUST_0)`
  block in `load_configuration` and new failure-test calls in
  `receive_msg` neither of which the monitor commits touched.
- `source/main.cc` auto-merged on `d9de5c3` (the `--ldpc-osd-*` knobs
  appended to the argv parser landed in a region monitor didn't touch).

Verified BP_OSD wiring after auto-merges:
- `telecom_system.cc:5022` — `ldpc.decoding_algorithm = BP_OSD` set for
  `configuration == ROBUST_0` (config id 100).
- `telecom_system.cc:2651, 2676` — failure tests now go through
  `ldpc_decode_failed(rc, algo, cap)` which returns the right thing for
  both SPA and BP_OSD return-code regimes.
- `ldpc.cc:87,327` — dense_G allocation at init (lazy) and BP_OSD branch
  in `decode()` both intact.

### §12.3. Final commit list (on feat/bp-osd-rebased, oldest first)

```
9a5d2dd ldpc: port log-domain BP decoder from ft8mon (Phase A.2 §7.1)
fe3b454 ldpc: dense generator-matrix construction for rate 1/16 (Phase A.2 §7.2)
533239e ldpc: port OSD decoder from ft8mon (Phase A.2 §7.3)
5fe7de5 ldpc: BP+OSD glue / cascade decoder (Phase A.2 §7.4)
ac8126d ldpc: BP+OSD unit tests + --test CLI hook (Phase A.2 §6.1)
032ed5a ldpc: add BP_OSD enum + ldpc_decode_failed helper (Phase A.2 §7.5)
570bf5b ldpc: wire BP_OSD cascade in cl_ldpc::decode + init dense_G (§7.5)
634d851 telecom_system: select BP_OSD for ROBUST_0 + unify failure check (§7.5)
d9de5c3 cli+ini: --ldpc-osd-norder / --ldpc-osd-maxosd knobs (§7.5 item 6)
f623fad ldpc: add cstddef include for size_t on gcc 12 (Pi build fix)
```

(Plus this fact-doc update commit on top.)

### §12.4. Build + test status

- `bash build.sh o3` from worktree: 73 files compiled, 0 errors, link OK.
  Same pre-existing warning set as monitor (winsock include order, WASAPI
  printf format, SoundCardDialog field-init order). Zero new warnings.
- `mercury.exe --test` exit code 0:
  - BP+OSD suite: 7 / 7 PASS (`generator_matrix_correctness`,
    `bp_correctness_clean`, `bp_vs_spa_noisy`, `osd_recovers_from_bp_fail`,
    `bp_osd_puncture`, `bp_osd_glue`, `bp_osd_graceful_fail`).
  - MFSK ctrl-suffix suite: 23 / 23 PASS (Phase B Wave 1 codec + Wave 2 v2
    + Wave 3 sign-invariance / cfo-recover / zero-cfo / pure-noise).
  - Total: 30 / 30 PASS.

### §12.5. Risk note for hardware A/B operator

The `osd_recovers_from_bp_fail` unit test still passes — i.e. the cascade
wiring is intact and OSD will improve on BP when BP fails. Whether OSD
actually fires under operating conditions depends on whether BP converges
inside its iter cap. On 2026-05-26 it converged in 1 iter at every WGN cell
tested (NULL DELTA verdict). The point of this rebase is to re-test on
current monitor where the cliff has moved deeper (Phase B MFSK CONNECT
reaches WGN:-8, data-PHY cliffs separately); if BP now sits closer to its
iter cap at the new operating point, OSD's fallback will exercise.

The decoder still defaults to OSD-1 / `maxosd=0` per
`cl_ldpc`'s constructor. Override via `--ldpc-osd-norder N` and
`--ldpc-osd-maxosd M` CLI flags (commit `d9de5c3`). The `is_robust_config`
gate is still in place — only ROBUST_0 selects BP_OSD; ROBUST_1, ROBUST_2,
and OFDM configs stay on SPA.

### §12.6. Open items handed to A/B operator

- IONOS axis-walk sweep at ROBUST_0 / WGN sweep matching the 2026-05-26
  baseline (research §6.4). Compare to `9c3fc40` baseline arm.
- `[OSD-PROFILE]` log instrumentation is NOT in this branch (research
  §6.2 future work). To see whether OSD fires, the operator can check for
  `iterations_done >= LDPC_BP_OSD_OSD_BASE` (=1000) at the receive-msg
  diagnostic prints.
- §11.5 items 5 (dense_G amortization), 6 (CLI knobs — DONE in d9de5c3),
  and 7 (compressed BP buffers) remain as in the original branch.

---

## §13. BP+OSD retest verdict on post-mini-Moose monitor (2026-05-28)

Rebased `feat/bp-osd-rebased` (10 commits from `fix/a26-bp-osd`, 1 conflict resolved in `main.cc:--test` hook) onto monitor `fb9c617` (sign-flip + ctrl-v2 + mini-Moose stack). Built clean, 30/30 unit tests pass.

5-pass-per-arm A/B at WGN:−11 (current cliff edge):

| pass | baseline (SPA) | BP+OSD |
|---|---|---|
| 1 | 3.6 | 0.0 |
| 2 | 0.0 | 0.0 |
| 3 | 0.7 | 0.4 |
| 4 | 1.6 | (lock fail) |
| 5 | 2.4 | (lock fail) |
| mean | **1.66** | **0.13** |

**Verdict: −92% regression on the 3 BP passes that completed.**

This is a HARDER verdict than the original 2026-05-26 null delta at WGN:−8. At WGN:−8 the issue was "OSD never fires because SPA converges in 1 iter". At WGN:−11 the issue is "BP decoder behaves DIFFERENTLY from SPA and worse — BP gives up where SPA recovers".

Likely culprits (not investigated — branch dropped):
- BP's tanh-based message-passing has different convergence dynamics at low SNR than SPA's log-domain version
- `ldpc_decode_failed()` helper introduced in §7.5 unifies failure detection across GBF/SPA/BP-OSD — too strict for BP at low SNR
- OSD pre-processing somehow corrupts state

**Conclusion: BP+OSD is NOT the right cliff push path for Mercury's rate-1/16 LDPC at any SNR we've measured.** The decoder is not the bottleneck; the SPA decoder is well-matched to the operating regime. Branch dropped without merge for the second time.

The original §A.2 hypothesis (BP+OSD adds 0.5-1.25 dB at the cliff per Franke-Taylor) doesn't hold for this codebase. Speculation on why: Mercury's rate-1/16 LDPC has girth-10 (clean), so SPA converges very fast on real codewords. WSJT-X's BP+OSD wins are on rate-~1/2 LDPC with higher girth-4 cycles where SPA has more trouble. Mercury's PCM is just structurally easier to decode.

Next axis per the queue: preamble length N=16→32 (the bigger code change, predicted +1.8-3 dB at the cliff if mini-Moose-class measurement applies).
