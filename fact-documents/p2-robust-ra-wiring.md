# Data-flow audit — P2 ROBUST_RA PHY production wiring (TX encode + RX decode + frame geometry)

**Status:** WIN CAMPAIGN — DRIVE increment 1 of the 71-rate-win build. Wire the GF(16)-RA
ROBUST_RA mode (cfg103) into the PRODUCTION PHY path: TX-side RA encode in `transmit_byte`,
RX-side `decode_robust_ra_data` spliced into `receive_byte`, and frame-geometry reconciliation
so a `-s 103` session round-trips real RA-coded BYTES TX→RX→CRC. **SIM/loopback only**
(no bench, no HW). Increment 2 (ARQ batch≥2+SACK chokepoint) and rate-opt and HW are LATER.

**Branch:** `win/incr1-p2-robust-ra-wiring` off the inner-mercury **monitor @0c75cc2** (after the
combiner merge a41b1e41). Worktree `C:/Users/kamer/mercury_wt/incr1-p2-robust-ra-wiring`.
**Date:** 2026-06-02. **Conventions:** §N sections, `file:line` citations, `[?]` unknowns.
Citations are against this worktree's tree unless noted.

This is the CLAUDE.md §5 cross-layer shared-state audit REQUIRED before the structural change.
It builds on the e2e PHY-proof audit (`data-flow-robust-ra-e2e.md`, reference branch
`sim/robust-ra-e2e @8306e2c`) which already enumerated producers/consumers/invariants for the
LOWER layers the RA mode touches (gearshift ladder §3, `load_configuration` PHY params §4,
`cl_mfsk` geometry + ofdm mirror §5, `freq_offset_measured` §6, the `gf16ra::g_k_*` codec graph
§7). That doc explicitly deferred the surface THIS increment wires (its §11 "Explicit P2
follow-on"): **frame-geometry reconciliation + TX encode + RX byte/CRC convergence for a real
ARQ session.** This audit owns those two NEW shared-state surfaces.

---

## §0 The wiring gap on the monitor base (verified by grep, not assumed)

Confirmed on `win/incr1-p2-robust-ra-wiring @0c75cc2` (= monitor):
- `ROBUST_RA`, `configure_k`, `encode_k`, `soft_decode_k`, `decode_robust_ra_data` appear ONLY
  in COMMENTS (ofdm.cc:3779 "future ROBUST_RA/ULTRA"; mfsk_ctrl_codec_tests.cc:1666/1678/1925/1934
  reference the M16×2 geometry as "ROBUST_RA geometry"). **No define, no codec functions, no
  decode method exist on monitor.** (Reference branch `8306e2c` HAS them; this increment ports
  them onto the monitor base + adds the new TX/RX/geometry wiring.)
- `is_robust_config` = 56 call sites across 9 files (audited in the e2e doc §8.2 — widening to
  include 103 is purely additive; no predicate result changes for configs 0..102).
- `NUMBER_OF_ROBUST_CONFIGS` = define-only, **0 consumers in source/** (the e2e doc §8.1
  verified; re-confirmed here). Left at 3 = "ladder robust count".
- The task brief cited "telecom_system.h:255 header comment claiming receive_msg calls
  decode_robust_ra_data" as comment/code drift to confirm. On the monitor base, h:255 is
  `set_connect_preamble_reps` — there is NO decode_robust_ra_data decl to be drifted. The drift
  note describes the REFERENCE branch's header (where the method was declared but never called
  from receive_msg). This increment adds the declaration AND the call, so the header comment I
  write will be TRUE (no drift).

**CONFIRMED:** `decode_robust_ra_data` is DEFINED-but-not-CALLED only on the reference branch;
on monitor it does not exist at all. Either way the production RX path decodes cfg103 with the
binary-LDPC chain today (cfg103 is not a real config until this increment). This is the
fail-before condition the gate test will pin.

---

## §1 The geometry decision (the load-bearing choice for this increment)

The cleanest, least-invasive reconciliation — and the one that keeps every non-RA path and the
whole ARQ frame-size contract BYTE-IDENTICAL — is:

> **cfg103 reuses ROBUST_2's M16×2 frame geometry VERBATIM (same Nsymb, same `data_container.nBits`,
> same LDPC matrix `ldpc.P` from rate 1/16). The RA codec internally re-codes the SAME
> `nReal_data` payload bits that the binary-LDPC path would carry — replacing `mfsk.demod`+LDPC
> (RX) and the LDPC encode (TX) with `gf16ra::{encode_k,soft_decode_k}` — without changing ANY
> buffer size, frame count, or the ARQ-facing `get_frame_size_bytes()`.**

### §1.1 Why this works — the geometry numbers (derived from code, then to be measured)

For a WB MFSK config the geometry authority is (telecom_system.cc):
- `ofdm.Nsymb = N_MAX / mfsk.bits_per_symbol()` (`:4288`); `N_MAX=1600`
  (physical_defines.h:31). M16×2 ⇒ `bits_per_symbol = nBits(=4)·nStreams(=2) = 8`
  ⇒ **Nsymb = 1600/8 = 200 symbol periods.**
- `data_container.set_size(nData=Nsymb, ..., M_eff=1<<bits_per_symbol=256, ..., Nsymb=200, ...)`
  (`:4406-4407`); set_size sets `nBits = nData·log2(M_eff) = 200·8 = 1600` (data_container.cc:98).
- LDPC: `ldpc.framesize = MERCURY_NORMAL = 1600` (mercury_ldpc.h:43; telecom_system.cc:5378),
  `ldpc.init()` sets `N=framesize=1600`, `K=N·rate`, `P=N−K` (ldpc.cc:66-68). For ROBUST_RA's
  `_ldpc_rate=1/16` (the e2e §4 INV-C1 choice): **K=100, P=1500.**
- ⇒ `nReal_data = nBits − P = 1600 − 1500 = 100` bits; `nVirtual_data = N − nBits = 0`
  (telecom_system.cc:532-533/840-841).
- ARQ frame size: `get_frame_size_bytes() = (nBits − ldpc.P − outer_code_reserved_bits)/8`
  (telecom_system.cc:478-480). With `outer_code=CRC16_MODBUS_RTU` (16 reserved bits, see §3
  INV-RX2), the per-frame **payload+header = (100−0)/8 = 12 bytes of which 16 bits are the
  appended CRC ⇒ ~10 usable bytes.** This is exactly ROBUST_0/1's tiny robust frame.
  **[VERIFY by instrumenting cfg103 at load — confirm Nsymb=200, nBits=1600, ldpc.P=1500,
  nReal_data=100. These drive the RA codec K below.]**

### §1.2 RA codeword sizing to MATCH `nReal_data`

The RA codec is symbol-native over GF(16): K_info info symbols × 4 bits each. To carry the SAME
`nReal_data = 100` payload bits the LDPC frame would: **K_info = nReal_data/4 = 100/4 = 25 GF(16)
info symbols.** At repfact=3 (R¼, the §7/§10 proven rate): N = K_info·(1+repfact) = 25·4 = **100
codeword symbols = 100 OFDM symbol periods.** That fits inside Nsymb=200 with 100 periods to
spare (the unused tail periods are simply not transmitted/decoded — `get_active_nsymb()` returns
N=100 for cfg103, see §2 INV-TX3 / §3 INV-RX4).

`100 = nReal_data` happens to equal N here only because rate=1/16 and repfact=3 coincide
(100 info bits → 25 sym → ×4 = 100 codeword sym). This is a numerical coincidence, NOT an
invariant — the impl computes K_info = `nReal_data/4` and N = `configure_k(K_info,3)` from the
live geometry, never hardcoded. **The CRC16 is computed over the byte payload BEFORE RA-encode
on TX and re-checked AFTER RA-decode on RX, exactly as the LDPC path does (§3 INV-RX2).**

### §1.3 Why NOT change `data_container.nBits` / Nsymb for cfg103

Changing the geometry (e.g. to make N==Nsymb exactly, or to grow the payload) would ripple into
every `data_container` buffer size, `buffer_Nsymb`, `message_transmission_time_ms`
(arq_common.cc:1348), the ARQ `set_max_buffer_length` (arq_common.cc:1317), and the SACK/batch
arrays — the exact multi-layer surface CLAUDE.md §5 warns against. Reusing ROBUST_2's geometry
verbatim means **every one of those consumers sees cfg103 as an ordinary M16×2 robust frame**;
only the inner encode/decode primitive differs. The unused tail symbol periods (Nsymb−N=100) are
inert: TX doesn't emit them, RX doesn't read them, and the frame-completeness/buffer math keys on
`get_active_nsymb()` (§2/§3) which returns N.

---

## §2 Shared state #A — the TX path (`transmit_byte` → `transmit` → passband), cfg103 branch

`cl_telecom_system::transmit_byte` (telecom_system.cc:488) then `transmit(...)` (`:528`).

### §2.1 Producers (who WRITES the TX data buffers on the cfg103 path)
- `transmit_byte` (`:488-525`): zero-pads `nBytes`→`frame_size` bytes, appends CRC16 over
  `[frame_size bytes]` when `outer_code==CRC16_MODBUS_RTU` (`:509-519`), `byte_to_bit`s into
  `data_container.data_bit[0..nReal_data)`, then calls `transmit(...,SINGLE_MESSAGE)`.
- `transmit` (`:528+`): TODAY does `ldpc.encode(data_bit→encoded_data)` (`:547-553`),
  `interleaver(encoded_data→bit_interleaved_data, nBits, bit_interleaver_block_size)` (`:555`),
  then for `M==MOD_MFSK`: `mfsk.mod(bit_interleaved_data, active_nbits, ofdm_framed_data)`
  (`:561-562`), preamble prepend, `symbol_mod`, `baseband_to_passband`, `peak_clip`
  (`:664-705`).

### §2.2 Consumers (who READS what TX produces)
- The RX chain (`receive_byte`) consumes the passband over the channel. The on-wire contract is:
  N preamble symbols (Welch-Costas) + `active_nsymb` data symbol periods, each data period a tone
  `(code_tone+hop)%M` replicated across both stream bands (the diversity geometry, e2e §5 INV-G2).
- `mfsk.mod` reads `mfsk.M/nBits/nStreams/stream_offsets/tone_hop_step` (e2e §5).

### §2.3 Valid states / default-init
- `data_container.data_bit[0..nReal_data)` holds the payload+CRC bits; positions `[nReal_data..)`
  are virtual/parity scratch. On cfg103 the RA path uses only `[0..nReal_data)` (= 100 bits = 25
  GF16 symbols) and produces N=100 codeword tones — it does NOT touch the LDPC parity region.
- `ofdm_framed_data` is sized `alloc_Nsymb·Nc` with `alloc_Nsymb=max(Nsymb,48,CTRL_SUFFIX_FEC_MAX
  _NSYMB)` (data_container.cc:127-129) ⇒ ≥200·Nc, so writing N=100 symbol slots is in-bounds.

### §2.4 Invariants the TX consumers assume
- **INV-TX1 (geometry match TX↔RX):** the tone placed for codeword symbol p MUST be
  `(code_tones[p] + p·tone_hop_step) % M` in BOTH stream bands (data-relative hop, p 0-based) —
  identical to the e2e TX builder (`build_robust_ra_e2e_audio`, mfsk_ctrl_codec_tests.cc:4524-4531)
  and to what `decode_robust_ra_data` de-hops (`(t − s·tone_hop_step)%M`, ref telecom_system.cc RA
  decode). Using `mfsk.mod` for the RA path would pack BITS→Gray→tone, NOT the symbol-native
  `code_tone` order ⇒ TX/RX mismatch. **⇒ the cfg103 TX path must place RA codeword tones DIRECTLY
  (bypassing `mfsk.mod`'s bit→tone mapping AND the bit-interleaver), mirroring the e2e builder.**
- **INV-TX2 (preamble identity):** the N preamble symbols are the loaded config's
  `mfsk.preamble_tones` (Welch-Costas g=2, 16-symb WB) — UNCHANGED from ROBUST_1/2. The RA path
  reuses the existing preamble synthesis (`:664-693`); only the DATA symbol fill differs.
- **INV-TX3 (active symbol count):** the number of data symbol periods TX emits = `get_active_nsymb()`
  (`:664`). For cfg103 this must be **N (=100)**, the RA codeword length — NOT Nsymb (200). ⇒
  `get_active_nsymb()` must return N for cfg103 (see §4 the helper change), so the
  `baseband_to_passband`/`peak_clip` spans (`:702-705`) and `total_frame_size`-based TX buffering
  cover exactly N data periods + preamble. (This mirrors the existing `mfsk_ctrl_mode` mechanism
  which already makes `get_active_nsymb()` return a REDUCED `ctrl_nsymb` for punctured ctrl frames
  — the RA path uses the SAME reduction hook, so the TX framing/`message_transmission_time_ms`
  consumers already handle active<Nsymb.)
- **INV-TX4 (power/gain):** RA data symbols use the SAME `TX_SIG_MFSK_2S` gain + power
  normalization as ROBUST_2 (the e2e builder uses `get_tx_gain(TX_SIG_MFSK_2S)`,
  mfsk_ctrl_codec_tests.cc:4538). No new gain entry.

### §2.5 What the fix changes (TX)
- Add a `current_configuration==ROBUST_RA` branch in `transmit` (at the `M==MOD_MFSK` site
  `:557-562`): instead of `ldpc.encode`+`interleaver`+`mfsk.mod`, (a) read the `nReal_data/4`
  payload symbols from `data_container.data_bit` (4 bits/symbol, MSB-first — the symmetric
  convention with the RA decode), (b) `gf16ra::configure_k(K_info,3)` + `gf16ra::encode_k` → N
  codeword tones, (c) place each tone `(code+hop)%M` in both stream bands of `ofdm_framed_data`
  (INV-TX1). Everything downstream (preamble, symbol_mod, passband, clip) is UNCHANGED and runs on
  the N active symbols via `get_active_nsymb()`.
- `transmit_byte`'s CRC16 append (`:509-519`) is UNCHANGED — the RA path codes the SAME
  CRC-suffixed payload bits (INV-RX2 symmetry). The bit-interleaver + LDPC encode are SKIPPED for
  cfg103 ONLY; all other configs keep `:547-562` byte-identical.

---

## §3 Shared state #B — the RX byte-fill + decode contract (`receive_byte`), cfg103 branch

`cl_telecom_system::receive_byte` (telecom_system.cc:835). The splice point is the
`M==MOD_MFSK` block AFTER the per-symbol FFT loop fills `ofdm_symbol_demodulated_data`
(`:2330-2333`) and AFTER mini-Moose re-mix (`:2244-2327`).

### §3.1 Producers (who WRITES the RX decode/stats state on the MFSK path)
- Sync front-end: `time_sync_mfsk_corr` → `receive_stats.delay` (`:1069`); `carrier_frequency_
  sync_wb_mfsk` → `freq_offset_measured` (`:2244`); re-mix (`:2307-2324`).
- `symbol_demod` loop (`:2330-2333`) → `data_container.ofdm_symbol_demodulated_data[i·Nc..]` for
  `i∈[0, get_active_nsymb())`. **This buffer is exactly the `sym_fft` contract `decode_robust_ra_
  data` consumes** (e2e §2 step 5 / RX-5; the e2e test fed the SAME `symbol_demod` output).
- TODAY: `mfsk.demod` (`:2340`) → LLRs `demodulated_data`; deinterleave (`:2634`); `ldpc.decode`
  (`:2647`) → `hd_decoded_data_bit`; `bit_to_byte` (`:2654`) → `hd_decoded_data_byte`; copy to
  `*out` (`:2667-2670`); CRC16 self-check (`:2675-2679`); `all_zeros`/`message_decoded`
  (`:2657-2788`).

### §3.2 Consumers (who READS the RX decode/stats — the ARQ-facing contract)
- The ARQ layer (`arq_responder.cc` / `arq_common.cc`) consumes the OUTPUT of `receive_byte`:
  - `*out` (= `data_container.hd_decoded_data_byte[0..nReal_data/8)`): the decoded frame bytes.
  - `receive_stats.message_decoded` (YES/NO): whether the frame is accepted.
  - `receive_stats.crc` (0 = pass): the CRC self-check result.
  - `receive_stats.all_zeros`, `.iterations_done`, `.SNR`, `.delay`, etc. (diagnostics / gating).
- The ARQ frame-size expectation = `get_frame_size_bytes()` (arq_common.cc:1313 derives
  `nBytes_data` from `nBits − ldpc.P − reserved`). cfg103 keeps ROBUST_2 geometry ⇒ this is
  UNCHANGED (§1).

### §3.3 Valid states / default-init (the bite to guard)
- `receive_stats` is zero/`-99.9`-initialized at function entry (`:843-852`). If the RA branch
  returns WITHOUT setting `message_decoded`/`crc`/`*out`, the ARQ layer sees `message_decoded=NO`
  (safe reject) — but a CORRECT RA decode MUST set them exactly as the LDPC path does, or a good
  frame is silently dropped.
- `hd_decoded_data_byte` is the buffer `*out` is copied FROM (`:2667-2669`); the RA branch must
  fill `hd_decoded_data_byte[0..nReal_data/8)` from the decoded info bits BEFORE the shared
  CRC/copy logic runs (so the existing `:2657-2788` block works verbatim).

### §3.4 Invariants the RX consumers assume
- **INV-RX1 (accept gate = CRC, not LDPC iters, for CRC16 configs):** the accept/reject branch
  (`:2681-2683`) is `all_zeros==YES || (CRC16 && crc!=0) || (!CRC16 && iters>cap)`. ROBUST configs
  set `outer_code=CRC16_MODBUS_RTU` (verified §3.5), so the gate is **purely the CRC** — the
  `iterations_done>cap` clause does NOT apply. ⇒ the RA branch must produce `hd_decoded_data_byte`
  such that `CRC16_MODBUS_RTU_calc(hd_decoded_data_byte, nReal_data/8)==0` for a correct frame.
  `receive_stats.iterations_done` is set to the BP iteration count from `soft_decode_k` (a
  diagnostic; does not gate acceptance for CRC16 configs) — and we must NOT let a `-1` (RA size
  error) masquerade as a decoded frame (guard: on `decode_robust_ra_data` returning <0, set
  `message_decoded=NO` and fall through to the failure path).
- **INV-RX2 (CRC symmetry):** TX appends CRC16 over the `frame_size`-byte payload BEFORE coding
  (§2, `:509-519`); RX must reproduce those SAME `nReal_data/8` bytes (payload+CRC) so the
  self-check `CRC16([payload||CRC])==0` holds. The RA decode yields K_info·4 = nReal_data info
  bits → `bit_to_byte` → the `nReal_data/8` bytes. **The bit→byte order and the info-bit MSB-first
  packing MUST match TX (§2.5(a)).** ⇒ identical pack/unpack convention on both sides; the existing
  `bit_to_byte`/`byte_to_bit` (MSB-first within a byte) is reused, and the RA symbol↔nibble mapping
  is MSB-first per GF(16) symbol (matches the ref decode's `(info[k]>>(3−b))&1` unpack).
- **INV-RX3 (sym_fft offset):** `decode_robust_ra_data` reads `ofdm_symbol_demodulated_data`
  indexed `[s·Nc + stream_offset + tone]` for s∈[0,N) — the SAME layout `symbol_demod` writes
  (`:2332`, `&...[i·Nc]`). The data symbols start AFTER the preamble; `symbol_demod` is already
  called at `baseband_data[i·Nofdm + Nofdm·preamble_nSymb]` (`:2332`), i.e. the i-th DATA symbol.
  ⇒ feeding `ofdm_symbol_demodulated_data` directly to the RA decode is offset-correct (the e2e
  test confirmed 200/200 clean via this exact buffer, e2e §10.1).
- **INV-RX4 (active symbol count):** the FFT loop runs `i∈[0, get_active_nsymb())` (`:2329-2333`).
  For cfg103 `get_active_nsymb()` returns N (=100), so exactly N symbol FFTs are produced — the
  count `decode_robust_ra_data` expects. (Same helper as TX INV-TX3; one change serves both.)
- **INV-RX5 (frame-completeness math):** the overflow/bounds checks (`:1441-1453`,
  `:1456-1457`) use `get_active_nsymb()` ⇒ cfg103's N-symbol frame is bounded correctly, no
  buffer-tail false-forward. UNCHANGED logic; correct because the helper returns N.
- **INV-RX6 (non-RA byte-identity):** the RA branch is gated `current_configuration==ROBUST_RA`
  (equivalently `M==MOD_MFSK && current_configuration==ROBUST_RA`). For EVERY other config the
  `:2336 if(M==MOD_MFSK)` / `:2389 else` flow and the `:2634-2788` LDPC+CRC flow are
  byte-identical (the RA branch is an early `if` that fills `hd_decoded_data_byte` then jumps to
  the shared `all_zeros`/CRC/`message_decoded` logic, OR a self-contained block that sets the
  same `receive_stats` fields and `*out`).

### §3.5 [VERIFY] outer_code == CRC16_MODBUS_RTU for robust configs
The accept gate (INV-RX1) and CRC symmetry (INV-RX2) depend on `outer_code==CRC16_MODBUS_RTU`
being set for ROBUST_RA. **[VERIFY at impl: confirm where `outer_code` is set for robust/MFSK
configs and that cfg103 inherits it. If robust configs do NOT use CRC16, the accept gate falls to
the `iterations_done>cap` clause — then the RA branch MUST set `iterations_done` to a PASS value
(< cap) on success and a FAIL value (≥ cap, or message_decoded=NO directly) on RA size error /
CRC fail, and run its OWN CRC check.]** The impl will set `message_decoded` explicitly from an
internal CRC check to be robust to either configuration.

### §3.6 What the fix changes (RX)
- Add a `current_configuration==ROBUST_RA` branch inside the `M==MOD_MFSK` block (right after the
  `:2330-2333` FFT loop, BEFORE the `:2340 mfsk.demod`): call `decode_robust_ra_data(
  ofdm_symbol_demodulated_data, N, K_info, 3, info_bits)`; on success `bit_to_byte` the
  nReal_data info bits → `hd_decoded_data_byte`, then either (a) jump into the shared
  `:2657-2788` accept/CRC/`message_decoded` logic, or (b) run the identical accept logic inline.
  On `decode_robust_ra_data<0` set `message_decoded=NO`. NO change to the `else`/OFDM path or to
  any other MFSK config (INV-RX6).

---

## §4 Shared state #C — the active-symbol/active-bits helpers (`get_active_nsymb/nbits`)

- **Producer:** `get_active_nsymb()` (`:3073-3076`) returns `mfsk_ctrl_mode && ctrl_nsymb>0 ?
  ctrl_nsymb : data_container.Nsymb`; `get_active_nbits()` (`:3078-3081`) analogous with
  `ctrl_nBits`/`nBits`.
- **Consumers:** TX framing (`:664` active_nsymb, `:561` active_nbits), RX FFT loop (`:2329`),
  frame-completeness (`:1444`), `mfsk.demod` length (`:2339`).
- **Valid states:** today these return the FULL Nsymb/nBits unless `mfsk_ctrl_mode` is on (a
  CONNECT/handshake puncture). For cfg103 DATA frames they must return **N / N·bits_per_symbol**
  (the RA codeword length), so TX emits and RX reads exactly N symbol periods.
- **What the fix changes:** make `get_active_nsymb()` return **N (the RA codeword symbol count)
  for cfg103** and `get_active_nbits()` return the matching bit count. Implemented by storing the
  RA codeword length when cfg103 is loaded (a member, e.g. `robust_ra_nsymb = configure_k(K_info,3)`
  computed in `load_configuration`'s ROBUST_RA arm AFTER geometry is finalized), and returning it
  from the helpers when `current_configuration==ROBUST_RA`. This is the SAME reduction mechanism
  `mfsk_ctrl_mode`/`ctrl_nsymb` already uses (INV-TX3) ⇒ all consumers already tolerate
  active<Nsymb. **Non-cfg103: helpers byte-identical (the RA clause is an added `if` returning
  early only for 103).**
  - **[VERIFY]** `get_active_nsymb()` is `const`; computing N requires K_info from the live
    geometry. Store N in a member at config-load (not computed in the const getter). Confirm the
    member is reset/!RA-safe for non-RA configs (default 0 / unused).

---

## §5 Cross-references to the e2e audit (lower layers — NOT re-audited here, verified unchanged)

The following shared state is owned by `data-flow-robust-ra-e2e.md`; this increment does NOT
change any of it beyond what that doc already shipped/specified. Re-verified present-or-portable:
- **§3 gearshift ladder:** ROBUST_RA off-ladder; `FULL_CONFIG_LADDER`/SIZE UNCHANGED; gearshift +
  Q-table byte-identical. (Port the `is_robust_config` widening + `ROBUST_RA` define.)
- **§4 load_configuration PHY arm:** port the `else if(configuration==ROBUST_RA)` arm
  (`_modulation=MOD_MFSK; _ldpc_rate=1/16; Nsymb-preamble=4→16 via :5311; estimator=LEAST_SQUARE`).
  The ROBUST_0!= guards route 103→M16×2 (INV-C2). **This increment ADDS to that arm:** compute +
  store `robust_ra_nsymb` (=N) after geometry is finalized, and `configure_k(K_info,3)` so the
  codec graph is ready before the first encode/decode (e2e §7 INV-K2).
- **§5 cl_mfsk geometry + ofdm mirror:** UNCHANGED (cfg103 reuses M16×2 verbatim). INV-G1/G2/G3
  (diversity geometry, one symbol/period both streams) govern the TX placement (§2 INV-TX1) and
  RX de-hop (§3 INV-RX3).
- **§6 freq_offset_measured (mini-Moose):** UNCHANGED; cfg103 data frames flow through it
  identically to ROBUST_1/2 (config-agnostic, `M==MOD_MFSK && !narrowband` gated).
- **§7 gf16ra::g_k_* codec graph:** port the `configure_k/encode_k/soft_decode_k` impls
  (additive; independent of the K=13 ctrl graph, e2e §7 INV-K1). This increment adds TWO new
  consumers: the production TX encode (§2) and RX decode (§3). INV-K2 (configure before use) is
  satisfied by configuring at load (§4) — and BOTH TX and RX re-`configure_k(K_info,3)` defensively
  at the call site is acceptable since the rebuild is idempotent and K_info is fixed for cfg103
  (single-threaded per frame, INV-K3). **[VERIFY]** TX and RX use the SAME K_info — both derive it
  from `nReal_data/4`, so they cannot diverge.

---

## §6 The ofdm.cc revert on the reference branch is NOT ported (monitor is ahead)

The reference `8306e2c`'s `ofdm.cc` diff REVERTS the §13 FAR-cleanup detector decision
(`decision_matched = nStreams>=2 ? best_matched : fine_best_matched` → back to `fine_best_matched`).
**Monitor @0c75cc2 already has the §13 cleanup (merged via dfd74cd/9cecc8f).** Porting the
reference's ofdm.cc would be a regression. ⇒ **ofdm.cc is left at monitor.** The combiner detector
(gated nStreams≥2) is already on monitor and serves cfg103's M16×2 detection. (The e2e gate
result −12.28 was measured on the reference's older detector; on monitor's stricter-FAR detector
the cliff may differ slightly, but increment 1's gate is a CLEAN-channel byte round-trip, not a
cliff measurement — detector reach is an HW/cliff concern for later increments.)

---

## §7 The fail-before / pass-after gate test for THIS increment

**Test:** `test_robust_ra_production_roundtrip` (new, mfsk_ctrl_codec_tests.cc), in-process:
1. `cl_telecom_system ts; ts.operation_mode=ARQ_MODE; ts.load_configuration(ROBUST_RA);`
2. Assert geometry: M==16, nStreams==2, and the derived N/K_info/nReal_data (record them).
3. Build a random payload of `get_frame_size_bytes()` bytes; `transmit_byte(payload, n, passband,
   SINGLE_MESSAGE)` (the PRODUCTION TX path with the RA encode).
4. Feed `passband` straight into `receive_byte(passband, out)` on a CLEAN channel (loopback, no
   AWGN) — the PRODUCTION RX path with the RA decode.
5. **ASSERT:** `receive_stats.message_decoded==YES`, `receive_stats.crc==0`, and `out[0..n)` ==
   the transmitted payload bytes (real RA-coded bytes round-tripped TX→RX→CRC).

**Fail-before:** on the monitor base (no RA wiring) cfg103 either does not exist (load fails /
no define) — so the test does not COMPILE without the port — or, with ONLY the define+arm but the
binary-LDPC encode/decode still in the path, the TX LDPC-encodes 100 info bits at rate 1/16 and
the RX LDPC-decodes, which is a DIFFERENT code than the RA codeword the test's geometry implies;
more concretely, once the RA decode is gated in but the TX still LDPC-encodes (or vice-versa),
the byte round-trip FAILS the CRC (mismatched codec) — the concrete fail-before. **Pass-after:**
TX RA-encode + RX RA-decode agree, CRC==0, bytes match.

**This is the §5 cross-layer regression test** (drives TX byte → PHY → RX byte → CRC, asserting
RX byte-state == TX byte-state). It lives in `mfsk_ctrl_codec_tests.cc` alongside the e2e gate and
runs in the default `mercury --test` suite.

---

## §8 Open items to VERIFY during implementation (don't ship assuming)

1. **Geometry numbers** [§1.1]: instrument cfg103 at load → confirm Nsymb=200, nBits=1600,
   ldpc.P=1500, nReal_data=100, get_frame_size_bytes()=… . K_info = nReal_data/4, N =
   configure_k(K_info,3). If nReal_data is NOT divisible by 4, pad the info-symbol packing
   (last symbol's high bits zero) and account for it symmetrically TX/RX.
2. **outer_code for robust** [§3.5]: confirm cfg103 sets `outer_code=CRC16_MODBUS_RTU`; if not,
   the RA branch runs its own CRC and sets message_decoded explicitly.
3. **get_active_nsymb const + member** [§4]: store N in a member at load; confirm const getter
   reads it; confirm non-RA path unaffected.
4. **TX active span** [§2 INV-TX3]: confirm `baseband_to_passband`/`peak_clip`/`total_frame_size`
   cover exactly preamble+N data periods for cfg103 (no read/write past N).
5. **Byte-identical proof** [INV-RX6/TX byte-identity]: `git diff monitor` shows ONLY: common_
   defines.h (+define, widened predicate), telecom_system.{h,cc} (+RA arm, +decode method, +TX
   branch, +RX branch, +helper clause, +member), mfsk_ctrl_codec.{cc,h} (+codec primitives),
   mfsk_ctrl_codec_tests.cc (+roundtrip test [+ port of e2e harness if needed]), +this fact-doc.
   NO edits to FULL_CONFIG_LADDER, arq_*, the Q-table, ofdm.cc, psk. A targeted diff of the OFDM
   and non-RA MFSK code paths shows no semantic change. `mercury --test` green (prior count + the
   new test).
6. **batch stays 1** [increment-2 boundary]: confirm `set_data_batch_size`/`load_configuration`
   keep cfg103 at batch=1 (is_robust_config gate, arq_common.cc:595/1332-1337). Increment 1 must
   NOT touch the batch chokepoint (arq_common.cc:595) — that's increment 2.

---

## §9 RESULTS (measured 2026-06-02, dev host, SIM/in-process)

**Build:** `bash build.sh o3` clean (only pre-existing winsock2/wasapi warnings). **Test:**
`./mercury.exe --test` (in-process; main.cc:251 runs `run_mfsk_ctrl_codec_tests`, returns 0 iff
0 failures). NO bench, NO HW.

### §9.1 Geometry (measured — confirms §1.1 derivation exactly)
cfg103 load prints `[ROBUST_RA] geometry: nReal_data=100 K_info=25 repfact=3 N=100 Nsymb=200
(active data periods=100)`. PHY: `M=16 nStreams=2 Nofdm=310 interp=4 preamble=16 | nBits=1600
ldpc.P=1500 nReal=100 K_info=25 N=100 Nsymb=200 frame_size=10B reserved=16`. Every number
matches the audit's prediction (Nsymb=200, nBits=1600, ldpc.P=1500, nReal_data=100, K_info=25,
N=100, frame_size=10 bytes). N=100 ≤ Nsymb=200 (no geometry overflow; the clamp guard never trips).

### §9.2 PASS-AFTER (the gate) — GREEN
`robust_ra_production_roundtrip`: `[ASSERT OK] cfg103 round-tripped 10 real RA-coded bytes
TX->RX->CRC clean (message_decoded=YES, crc=0, BP iters=0)`. The production TX (transmit_byte
CRC16 append → transmit_bit RA encode → M16×2 diversity tone placement) and production RX
(receive_byte symbol_demod → decode_robust_ra_data → hd_decoded_data_byte → shared CRC16 gate)
agree on a clean channel: **message_decoded=YES, crc=0, decoded bytes == TX payload (byte-exact).**
BP iters=0 = the Q-ary BP converged on the first iteration (clean channel). Full suite:
**53 passed, 0 failed, exit=0** (verified across repeated deterministic runs).

### §9.3 FAIL-BEFORE — confirmed empirically
With the TX RA-encode branch temporarily disabled (`if(false && … ROBUST_RA)` — so cfg103 TX
takes the binary-LDPC `mfsk.mod` path while RX RA-decodes), the SAME test FAILS:
`[FAIL] robust_ra_production_roundtrip: message_decoded=NO (crc=0x0000 iters=-1 all_zeros=1) —
RA byte round-trip failed` (52 passed, 1 failed). Only this one test flips — no collateral. This
proves the gate genuinely discriminates the RA wiring (TX↔RX RA codec agreement), not just config
loadability. (Probe reverted; final tree has the TX RA branch active.) The structural fail-before
(monitor base: cfg103 unusable → geometry assert) is additionally guaranteed by the
`is_robust_config`/load-arm/codec being absent on monitor.

### §9.4 Non-cfg103 byte-identity — PROVEN
`git diff -w monitor -- source/ include/` genuinely-removed-line set (ignoring whitespace) =
ONLY the two `if(M == MOD_MFSK)` → `else if(M == MOD_MFSK)` lines (RA-gated branch added before
each; original body preserved verbatim) + the `is_robust_config … <=102` → `… <=103` predicate
widening. The deinterleaver/LDPC/energy-dispersal block is byte-identical (re-indented one level
inside `if(!ra_data_path)`; `ra_data_path` is always false for non-103 ⇒ the block always runs).
NO edits to FULL_CONFIG_LADDER, arq_*, the Q-table, ofdm.cc, or psk (not in the diffstat).
Diffstat: common_defines.h (+22/-…), mfsk_ctrl_codec.{h,cc} (pure append), telecom_system.{h,cc}
(+RA arm/method/branches/members/helper clauses), mfsk_ctrl_codec_tests.cc (+the §22 gate test
+registration), + this fact-doc. **Non-103 execution path is identical.**

### §9.5 §8 open items — resolved
1. Geometry numbers: measured = predicted (§9.1). ✓
2. outer_code: CRC16_MODBUS_RTU set for all configs incl 103 (physical_config.cc:77,
   telecom_system.cc load_configuration); reserved=16. The accept gate is the CRC (INV-RX1). ✓
3. get_active_nsymb const + member: robust_ra_N/K_info stored at load (init() geometry block);
   const getters read them; non-RA returns 0/full. ✓
4. TX active span: TX emits preamble+N data periods (get_active_nsymb=N); clean round-trip
   confirms no read/write past N. ✓
5. Byte-identical: proven (§9.4). ✓
6. batch=1: is_robust_config(103)=true ⇒ arq_common.cc:595/1332-1337 pin batch=1; the ARQ batch
   chokepoint (increment 2) is UNTOUCHED. ✓

### §9.6 Branch + commit
Branch `win/incr1-p2-robust-ra-wiring` off monitor @0c75cc2. Worktree
`C:/Users/kamer/mercury_wt/incr1-p2-robust-ra-wiring`. Commit hash: [filled at commit]. NOT
pushed; monitor untouched. Binary NOT installed to Program Files (SIM-only; --test runs
in-process; avoided clobbering a possibly-concurrent bench binary).

---

## §10 WIN-CAMPAIGN Front-A increment 1 — LONGER FRAMES + the airtime nStreams/active bug

**Status:** SIM/in-process, 2026-06-02, branch `win/incr1-longer-frame-airtime` off the
integration `win/integ-incr1-incr2 @fd5c698` (worktree
`C:/Users/kamer/mercury_wt/win-integ-incr1-incr2`). NO bench, NO HW. batch stays 1
(incr2's batch≥2 RF-refuted; this increment keeps `robust_dwell_batch=1`).

### §10.1 The problem (Front-A ac0546c5, code-grounded)
Per-frame cycle @ −10 = 7891 ms for ~5 user bytes: frame on-air 2946 ms (37%), **ACK
turnaround 4539 ms (57.5%)**, tails ~400 ms. The lever is LONGER FRAMES: efficiency =
T_air/(T_air + ~4.9 s fixed). The shipped cfg103 (§9.1) is rate 1/16 →
nReal_data=100 → K_info=25 → **N=100 codeword periods in a Nsymb=200 frame (HALF EMPTY,
only 100 of 200 periods on air)**, so the fixed ACK turnaround + the fixed DATA header
(6 B, §10.4) are amortized over only ~4 user bytes.

### §10.2 The change (ONE structural change + the prerequisite airtime fix)
**(a) Longer frame — `telecom_system.cc:5366` cfg103 `_ldpc_rate` 1/16 → 2/16.** The RA
data path bypasses the binary LDPC; `ldpc.P` is ONLY the payload-size knob
(`nReal_data = nBits − ldpc.P`, `:4578`). Rate 2/16 ⇒ P=1400 ⇒ nReal_data=200 ⇒
K_info=50 ⇒ **N = configure_k(50,3) = 200**, which FILLS the existing Nsymb=200 frame
exactly. The RA codec stays **repfact=3 (R¼)** — `ROBUST_RA_REPFACT` is unchanged — so the
~5.3 dB coding gain that puts cfg103 at the −14 dB floor
(`tier2-suffix-fec-gf16-spike.md` §8.4) is INTACT; the LDPC rate sizes the payload, it is
NOT the data FEC. frame_size 10 B → **23 B**. N=200 ≤ Nsymb=200 ⇒ the §4 overflow clamp
never trips. K is NOT a blocker: `gf16ra::configure_k/encode_k/soft_decode_k` are the
K-GENERALIZED data variant (`mfsk_ctrl_codec.h:295-316`, heap-allocated, K unbounded by
GF16RA_MAX_N); the K=13 lock is only the *ctrl-suffix* `configure/encode/soft_decode`
(no `_k`), which this path does not use.

**(b) Airtime bug (prerequisite) — `arq_common.cc:1397` `data_container.Nsymb` →
`get_active_nsymb()`, cfg103-gated.** `message_transmission_time_ms` is the DATA-frame
on-air time. The TX emits `get_active_nsymb()` data periods (`telecom_system.cc:712` loop
bound), and the RX bounds use `get_active_nsymb()` (`:1490`), but the formula used the
ALLOCATED `data_container.Nsymb`. For cfg103 `get_active_nsymb()` returns `robust_ra_N`
(`:3168`) = N < Nsymb, so the formula **over-estimated cfg103 airtime by Nsymb/N** (1.86×
at N=100; the formula computed 5580 ms vs the true ~2997 ms on-air — matches Front-A's
~2946 ms measured). The task framed this as "Nsymb=200 but cl_mfsk emits 100 because
nStreams=2 packs 2/period". That `Nsymb/nStreams` heuristic gives the right number ONLY
while N=Nsymb/2 (the rate-1/16 coincidence); the ROOT CAUSE is **active≠allocated**, and
once the longer frame fills N to 200 the `nStreams`-divide would compute HALF the true
airtime (catastrophically short timeouts). **`get_active_nsymb()` is the correct,
frame-size-robust fix.** Gated `current_configuration==ROBUST_RA` so the non-cfg103 value
is provably byte-identical (and independent of the transient `mfsk_ctrl_mode` puncture,
which is never active at config-load anyway). Ordering: ARQ `load_configuration`
(`arq_common.cc:1325`) calls `telecom_system->load_configuration()` (which sets
`robust_ra_N`, `:4580`) BEFORE the formula (`:1397`) — so `get_active_nsymb()` is valid.

### §10.3 Why NOT 4× (the N_MAX ceiling — the bounded follow-on)
4× payload (K_info≈100) needs N=400 > Nsymb=200. Nsymb is `N_MAX/bits_per_symbol`
(`telecom_system.cc:4439`; N_MAX=1600, M16×2 ⇒ 200). Growing it would require raising the
global `#define N_MAX` (`physical_defines.h:31`), which sizes `data_bit`, `encoded_data`,
`bit_interleaved_data`, `hd_decoded_data_*` etc. (`data_container.cc:106-145`) for EVERY
config AND the LDPC `framesize` (`MERCURY_NORMAL`) — the exact multi-layer ripple §1.3 +
CLAUDE.md §5 warn against. Lever A (fill the existing frame) is the **largest frame
achievable within the existing buffer geometry** and is fully contained. The N_MAX-growth
4× frame is flagged as the bounded follow-on increment (a per-cfg103 Nsymb override + the
N_MAX-sized buffers audited for the bigger frame).

### §10.4 §5 cross-layer audit — the frame-size + airtime CONSUMERS

**The shared state changed: (i) the cfg103 frame geometry (frame_size, N, nReal_data) via
ldpc.P; (ii) `message_transmission_time_ms` via the airtime formula.** PHY (frame
geometry/buffers) × ARQ (windows/caps). The decisive property that keeps this contained:
**Nsymb stays 200** — only the *fill fraction* (N: 100→200) and the *payload* (P: 1500→1400)
change. Every buffer sized on Nsymb is therefore UNCHANGED.

**(1) Producers of the geometry:** `load_configuration` (`telecom_system.cc:5366`
`_ldpc_rate`), `ldpc.init()` (`ldpc.cc:67-68` K=N·rate, P=N−K — algorithmic IRA, every k/16
rate valid, NOT table-driven), the §1/§4 RA-geometry block (`:4576-4599` derives K_info, N
from nReal_data). **Producer of the airtime:** `arq_common.cc:1397` (the one site;
`ctrl_transmission_time_ms` `:1398-1400` keys on the separate `ctrl_nsymb` and is
unchanged).

**(2) Consumers and how each stays safe for the N=100→200 (active) + P=1500→1400 (payload)
change:**

| Consumer | file:line | Keys on | Safe because |
|---|---|---|---|
| RX overflow guard | telecom_system.cc:1490-1499 | `get_active_nsymb()` (=N) + `buffer_Nsymb` | `buffer_Nsymb` sized on `preamble+Nsymb=200` (`data_container.cc:150`); N≤200 ⇒ `frame_end_samples ≤ buffer_samples` exactly as a full robust frame |
| OFDM preamble bound | telecom_system.cc:1503 | full `Nsymb` (=200) | Nsymb UNCHANGED |
| TX symbol_mod loop | telecom_system.cc:712 | `get_active_nsymb()` (=N) | emits N periods into `ofdm_symbol_modulated_data` sized `Nofdm·alloc_Nsymb` (alloc=max(Nsymb,48,128)=200); N=200 fits |
| TX passband span | telecom_system.cc:739 | `Nofdm·active_nsymb` | within `passband_data` sized `(Nsymb+preamble)·Nofdm·interp` (`data_container.cc:167`) |
| RX FFT loop | telecom_system.cc:2329-2333 | `get_active_nsymb()` (=N) | writes `ofdm_symbol_demodulated_data` sized `Nsymb·Nc=200·Nc`; N≤200 |
| RA decode periods | telecom_system.cc:2399 | `robust_ra_N` (=N) | reads N≤200 periods from the same Nsymb-sized buffer |
| `ofdm_framed_data` | data_container.cc:129 | `alloc_Nsymb·Nc`=200·Nc | TX RA places N=200 tones (`:579+`), in-bounds |
| ARQ frame_size | arq_common.cc:1355 | `nBits − ldpc.P − reserved` | grows 10→23 B BY DESIGN; the ARQ buffers (`set_max_buffer_length`) re-size to the new frame_size at load |
| `message_transmission_time_ms` | arq_common.cc:1397 (FIXED) | `get_active_nsymb()` for cfg103 | now the TRUE on-air time (5580 ms at N=200; was wrongly 5580 at N=100) |
| CMD post-TX frame_drain | arq_common.cc:703 (`2*msg_time`) | msg_time | was 1.86× inflated at N=100 → now correct; at N=200 the true bigger frame is correctly accounted |
| RSP rx_timeout | arq_common.cc:732 (`batch*msg_time`) | msg_time | same — was inflated, now tracks active N |
| data/ctrl ACK windows | arq_common.cc:749/753/1467/1468/1472/1473 | msg_time | same |
| RSP monitor_timeout | arq_responder.cc:1787/2492/2562 (`batch*msg_time`) | msg_time | same |
| OFDM batch sizing | arq_common.cc:1421-1424, :782 | msg_time, **gated `!is_robust_config`** | cfg103 IS robust ⇒ batch sizing SKIPS it (batch stays `robust_dwell_batch`=1) ⇒ the airtime change cannot perturb cfg103's batch |
| `buffer_Nsymb` | data_container.cc:150 | full `Nsymb` (=200) | Nsymb UNCHANGED ⇒ buffer_Nsymb UNCHANGED (already holds the N=200 active frame) |

**(3) Valid states / default-init:** before the geometry block runs, `robust_ra_N=0`
(`:87`); `get_active_nsymb()` then returns `Nsymb` (the `robust_ra_N>0` guard fails) — so a
mis-ordered call is fail-safe (full Nsymb, never a too-short airtime). After the load arm,
`robust_ra_N=200` for cfg103, 0 for every other config.

**(4) Invariants:** (INV-A) `N ≤ Nsymb` (200≤200 — clamp guard `:4581` is the backstop).
(INV-B) `buffer_Nsymb ≥ preamble + N` (564 ≥ 216 — the RX ring holds the active frame with
turnaround+margin headroom). (INV-C) airtime byte-identical for active==Nsymb (every
non-cfg103 config — proven by the cfg103 gate). (INV-D) repfact unchanged ⇒ R¼ coding gain
unchanged (the bigger frame is NOT weaker FEC, it is MORE coded symbols at the same rate).

**(5) What the fix changes vs every consumer:** only (i) cfg103's payload/N (fills the
already-allocated frame) and (ii) cfg103's airtime (corrects an over-estimate). No consumer
that keys on full `Nsymb`, `buffer_Nsymb`, or any N_MAX-sized buffer sees any change. The
ARQ frame_size grows by design and the ARQ buffers track it at load. batch stays 1.

### §10.5 Failing-test-first (CLAUDE.md §3) — `test_robust_ra_longer_frame_throughput`
New in-process gate (`mfsk_ctrl_codec_tests.cc` §23, runs in `mercury --test`). Measures
delivered **USER-bps** = `(frame_size − 6) · 8 / (T_air + T_fixed)` (T_air ∝ N+preamble
periods) for the COMPILED cfg103 frame vs the documented small-frame reference (10 B, N=100,
§9.1), and asserts ≥ 1.8× at the PESSIMISTIC T_fixed=0 (header + frame-fill amortization
only; the realistic T_fixed only raises the ratio). It also round-trips the compiled frame
**clean (4/4) + a lossy AWGN cell** (targeted SNR3k, well within the R¼ waterfall) so the
throughput reflects a frame that actually delivers.
- **FAIL-BEFORE** (rate 1/16): live geometry == reference (10 B/100) ⇒ ratio **1.00× < 1.8×
  ⇒ FAIL** (53 passed, 1 failed). clean 4/4 + lossy 8/8 STILL pass at 1/16 — proving the
  gate discriminates FRAME SIZE, not decode capability. (Measured 2026-06-02.)
- **PASS-AFTER** (rate 2/16): [filled from the final run — see §10.6].

### §10.6 RESULTS (measured 2026-06-02, dev host, SIM/in-process)
**Build:** `bash build.sh o3` clean (only pre-existing winsock2/wasapi warnings). **Test:**
`./mercury.exe --test` (in-process; NO bench, NO HW; binary NOT installed — avoided
clobbering a possibly-concurrent v13 Q-table bench, per §9.6).

- **Geometry (PASS-AFTER, rate 2/16):** `[ROBUST_RA] geometry: nReal_data=200 K_info=50
  repfact=3 N=200 Nsymb=200 (active data periods=200)`; PHY `nBits=1600 ldpc.P=1400 nReal=200
  K_info=50 N=200 Nsymb=200 frame_size=23B reserved=16`. The frame is FILLED (N=200 of 200);
  repfact=3 (R¼ coding gain intact). frame_size 10→**23 B**.
- **§22 existing roundtrip STILL GREEN:** cfg103 round-tripped **23** real RA-coded bytes
  TX→RX→CRC clean (message_decoded=YES, crc=0, BP iters=0) — the bigger frame still
  byte-round-trips through the production PHY.
- **§23 throughput gate (PASS-AFTER):** `LIVE frame=23B N=200` vs `REF frame=10B N=100`:
  **delivered USER-bps ratio = 2.28× (T_fixed=0, pessimistic) / 3.21× (T_fixed=4945 ms,
  realistic)** — both ≥ 1.8×. Delivery: **clean 4/4 + lossy (SNR3k=−5.0 dB) 8/8 byte-exact**
  (the longer frame decodes reliably ~9 dB above the R¼ cliff — the throughput is real,
  not fictional). sym=25.83 ms (matches `1000·Nofdm/((BW/Nc)·Nfft)` = 1000·310/12000).
- **FAIL-BEFORE (rate 1/16, measured):** LIVE==REF (10B/100) → ratio **1.00× < 1.8× → FAIL**
  (53 passed, 1 failed); clean 4/4 + lossy 8/8 STILL passed — the gate discriminates FRAME
  SIZE, not decode. Only this one test flips → no collateral.
- **Full suite:** **54 passed, 0 failed** (was 53; +1 new gate). Repeatable across runs
  (deterministic seeds).
- **Non-cfg103 byte-identity — PROVEN.** `git diff -w fd5c698 -- source/ include/`: (i)
  telecom_system.cc = ONLY the cfg103 `_ldpc_rate` 1/16→2/16 inside the `ROBUST_RA` arm
  (+comment); CONFIG_1's existing 2/16 untouched. (ii) arq_common.cc = the airtime formula
  body is byte-identical for the `: data_container.Nsymb` branch (the original expression
  verbatim); only cfg103 takes `get_active_nsymb()`. (iii) mfsk_ctrl_codec_tests.cc = pure
  append (new helper + test + registration). NO edits to FULL_CONFIG_LADDER, the Q-table,
  ofdm.cc, psk, the LDPC matrices, N_MAX, or buffer_Nsymb. Diffstat: arq_common.cc +20/-1,
  telecom_system.cc cfg103-arm only, mfsk_ctrl_codec_tests.cc +196 (test). batch stays 1
  (`robust_dwell_batch=1` default, is_robust_config(103)=true).

### §10.7 Projected delivery efficiency at the new frame size
on-air = (N+preamble)·sym_ms = (200+16)·25.83 = **5580 ms** (was 2997 ms at N=100). With
the Front-A ~4945 ms fixed turnaround: efficiency T_air/(T_air+fixed) = 5580/10525 =
**53%** (was 2997/7942 = 38%). Delivered USER-bps @ −10: ref ~4 B/cycle vs live ~17
B/cycle → the realistic ratio 3.21× (the per-frame ACK turnaround is now amortized over
4.25× more user payload + the airtime fix removes the 1.86× timeout inflation that was
itself stretching the cycle). The 4× N_MAX-growth frame (§10.3) is the next lever; the
Front B' "raw" lever (PHY-rate) combines orthogonally with this efficiency win at HW.

### §10.8 Branch + commit
Branch `win/incr1-longer-frame-airtime` off the integration `win/integ-incr1-incr2 @fd5c698`
(worktree `C:/Users/kamer/mercury_wt/win-integ-incr1-incr2`). Commit hash: **dc029fb**
(code + this §10). NOT pushed; integration branch otherwise untouched. SIM-only.
