/*
 * Mercury: A configurable open-source software-defined modem.
 * Copyright (C) 2022-2024 Fadi Jerji
 * Author: Fadi Jerji
 * Email: fadi.jerji@  <gmail.com, caisresearch.com, ieee.org>
 * ORCID: 0000-0002-2076-5831
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, version 3 of the
 * License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#ifndef INC_MFSK_CTRL_CODEC_H_
#define INC_MFSK_CTRL_CODEC_H_

#include <cstdint>

// MFSK CONTROL frame discriminator (2 bits, MSB-first within the suffix
// 52-bit field [type:2|payload:38|crc12:12]). See fact-documents/
// phase-b-mfsk-connect-research.md §11 for the design rationale.
//
// Wave 1 implements pack/unpack primitives for all four type codes but only
// START_CONN + TEST_ACK are wired into production code paths in Wave 2.
// MFSK_CTRL_TEST_CONN is reserved (encode/decode helpers are NOT provided in
// Wave 1) — callers requesting it from unpack will get out_type=3 and must
// treat the payload as opaque.
enum mfsk_ctrl_frame_type : uint8_t {
	MFSK_CTRL_ACK_SACK   = 0,  // existing ACK+SACK (re-fitted to 30-bit bitmap)
	MFSK_CTRL_START_CONN = 1,  // CMD → RSP: callsign + NB-flag
	MFSK_CTRL_TEST_ACK   = 2,  // RSP → CMD: cap echo + own cap + SSID
	MFSK_CTRL_TEST_CONN  = 3,  // reserved (Wave 2+ — quantized SNR + cap)
};

// =============================================================================
// 6-bit base-36 callsign body codec
// =============================================================================
//
// Alphabet mapping (matches arq.h:124-128 / `callsign_pack`):
//   'A'..'Z' → 0..25
//   '0'..'9' → 26..35
//   value 36 = end-of-string sentinel (padding for callsigns shorter than 6
//             chars). The legacy `callsign_pack` uses value 0 ('A') as padding
//             which collides with real-call body chars. The sentinel makes
//             short calls round-trip lossless.
//
// Six chars × 6 bits = 36 bits. Returns the number of body chars packed
// (0..6). Chars beyond the supported alphabet are uppercased; if still
// non-encodable (e.g. '/', '-', '6'..'9' for grandfathered weird calls
// when len > 6) they become 'A' (val=0) silently with a one-time stderr
// warning.
//
// Wave 1 NB-flag is carried separately in `pack_start_conn_payload`; this
// helper is callsign-body-only.

// Pack a callsign body (up to 6 chars, A-Z/0-9) into the low 36 bits of
// *out36. Char N occupies bits 35-6N..30-6N (MSB-first; char 0 is bits
// 35..30). Returns the number of body chars consumed (0..6). Pads
// remainder with the end-of-string sentinel (36).
int pack_callsign_body_b36(const char* call, int call_len, uint64_t* out36);

// Unpack 36 bits into a 7-byte buffer (6 chars + NUL terminator). Stops
// at the end-of-string sentinel. Returns char count (0..6).
int unpack_callsign_body_b36(uint64_t in36, char out[7]);

// =============================================================================
// MFSK_CTRL_START_CONN payload (38 bits)
// =============================================================================
//
//   bits 37    : nb_flag        (1)   1=narrowband, 0=wideband
//   bits 36..1 : sender_body    (36)  6 chars × 6 bits, char 0 at bits 36..31
//   bits 0     : reserved       (1)   sender MUST send 0; receiver ignores
//
// `call` is the bare callsign (no SSID). If call_len > 6 the body is
// truncated to the first 6 chars. The function uppercases A-Z and pads
// short calls with the end-of-string sentinel via pack_callsign_body_b36.
void pack_start_conn_payload(uint64_t* p38, bool nb_flag,
                              const char* call, int call_len);

// Returns true if the unpack succeeds. out_call receives the unpacked
// callsign body as a NUL-terminated string of up to 6 chars; out_len
// receives the number of body chars decoded (0..6). The reserved bit is
// ignored on RX.
bool unpack_start_conn_payload(uint64_t p38, bool* out_nb_flag,
                                char out_call[7], int* out_len);

// =============================================================================
// MFSK_CTRL_TEST_ACK payload (38 bits)
// =============================================================================
//
//   bits 37..36 : echoed_cap[1:0] (2)   peer's cap echoed back (low 2 bits)
//   bits 35..34 : own_cap[1:0]    (2)   responder's local_capability (low 2 bits)
//   bits 33..26 : ssid            (8)   0-15 numeric, 16=L, 17=T, 18=R, 19=X,
//                                        255 = SSID_NONE (matches
//                                        `arq.h:60 #define SSID_NONE 0xFF`)
//   bit  25     : echoed_cap[2]   (1)   CAP_SUFFIX_FEC of the echoed peer cap
//   bit  24     : own_cap[2]      (1)   CAP_SUFFIX_FEC of the responder's own cap
//   bits 23..0  : reserved        (24)  must be 0 on TX, ignored on RX
//
// echoed_cap / own_cap are the 3 negotiable bits (CAP_NEGOTIABLE_MASK=0x07):
// CAP_WB_CAPABLE (0x01), CAP_ENCRYPTION (0x02), CAP_SUFFIX_FEC (0x04, §21). The
// 3rd bit lives in former-reserved space (bits 25/24) so a legacy peer packs it
// 0 and ignores it on RX → CAP_SUFFIX_FEC negotiates OFF on a mixed pair. Bits
// above 0x07 are masked off.
void pack_test_ack_payload(uint64_t* p38, uint8_t echoed_cap,
                            uint8_t own_cap, uint8_t ssid);

bool unpack_test_ack_payload(uint64_t p38, uint8_t* echoed_cap,
                              uint8_t* own_cap, uint8_t* ssid);

// =============================================================================
// MFSK_CTRL_TEST_CONN payload (38 bits)  — Wave 3 (§14)
// =============================================================================
//
//   bits 37..34 : snr_q          (4)   SNR quantized via cl_mfsk::snr_to_tone
//                                       at M=16 (0..15, 2 dB step, range
//                                       -5..+25 dB per mfsk.cc:549-559)
//   bits 33..32 : local_cap[1:0] (2)   sender's local_capability (low 2 bits)
//                                       (CAP_WB_CAPABLE=0x01, CAP_ENCRYPTION=0x02)
//   bits 31..24 : ssid           (8)   0-15 numeric, 16=L, 17=T, 18=R, 19=X,
//                                       255 = SSID_NONE (matches
//                                       `arq.h:60 #define SSID_NONE 0xFF`).
//                                       Identical encoding to TEST_ACK.
//   bit  23     : local_cap[2]   (1)   CAP_SUFFIX_FEC of sender's own cap (§21)
//   bits 22..0  : reserved       (23)  must be 0 on TX, ignored on RX
//
// Site F (RSP RX) reconstructs the legacy float SNR via
// cl_mfsk::tone_to_snr(snr_q). 2 dB quantization step is documented in the
// v3_test_conn_snr_quantization_roundtrip regression test.
void pack_test_conn_payload(uint64_t* p38, uint8_t snr_q,
                             uint8_t local_cap, uint8_t ssid);

bool unpack_test_conn_payload(uint64_t p38, uint8_t* snr_q,
                               uint8_t* local_cap, uint8_t* ssid);

// =============================================================================
// CRC-aided soft list decode of the 13-symbol ctrl-suffix (Tier 1, ZERO airtime)
// =============================================================================
//
// connect-suffix-fec-research.md §3. The uncoded suffix decode passes only if
// ALL n symbol argmax decisions are correct (P ~= (1-q)^n — the cliff). This
// decoder uses the per-symbol top-K candidate tones (ranked by energy, from
// cl_ofdm::decode_suffix_candidates) and the existing 12-bit CRC to CORRECT a
// few wrong symbols by bounded best-first search: it enumerates one-tone-per-
// symbol assignments in increasing total soft-cost order and accepts the first
// whose recomputed CRC12 matches the embedded crc12 AND whose type matches
// `expected_type`. No wire-format change; the all-best assignment (tried first)
// reproduces the hard decode exactly.
//
// CRC is NOT inlined here (v1 bug #1, fact-doc §13.5): the caller passes
// `crc12_fn` wrapping the production cl_arq_controller::CRC12_calc over the
// 5-byte MSB-first [type:2|payload38:38] field.
//
//   cand[s*K + k] / cost[s*K + k] : k-th candidate tone (0..M-1, -1 invalid)
//                                   and its soft cost (>=0) for symbol s.
//   n          : suffix symbol count (= ack_sack_suffix_len(), 13 at M=16)
//   bits_per_tone : log2(M) (4 at M=16)
//   expected_type : required type discriminator (drop other types)
//   max_trials : hard cap on CRC checks (bounds runtime; upper-bounds FAR)
//   max_flips  : reject any assignment that deviates from the per-symbol argmax
//                in more than this many symbols (Hamming radius). This is the
//                PRIMARY false-accept-rate lever: it limits the search to a
//                small ball around the hard decode so a pure-noise input can
//                only reach a tiny number of candidate codewords. 0 = hard only
//                (baseline); a small value (1-3) captures the cliff regime
//                (where only a few symbols are wrong) while keeping FAR low.
//                <0 = unlimited (bounded only by max_trials).
//   crc12_fn / crc12_ctx : production CRC-12 over (data,5) bytes.
//
// On success writes *out_payload38 and *out_flips (number of symbols that
// differ from the hard argmax — 0 means the hard decode would have passed) and
// returns true. Returns false if no CRC-valid type-matched assignment found
// within the (max_trials, max_flips) budget.
typedef uint16_t (*ctrl_crc12_fn)(void* ctx, const unsigned char* data, int n);

// Pack [type:2|payload38:38] into 5 bytes MSB-first (CRC input). Mirrors
// arq_common.cc:pack_ctrl_typed40_msb_v2 — exposed so the soft decoder and the
// ARQ layer can share one definition / cross-check.
void pack_ctrl_typed40_msb(unsigned char out_bytes[5], uint8_t type,
                           uint64_t payload38);

bool soft_list_decode_ctrl_suffix(const int* cand, const double* cost,
                                  int n, int K, int bits_per_tone,
                                  uint8_t expected_type, int max_trials,
                                  int max_flips,
                                  ctrl_crc12_fn crc12_fn, void* crc12_ctx,
                                  uint64_t* out_payload38, int* out_flips);

// =============================================================================
// Tier-2 candidate A: soft GF(16) rate-~1/2 RA code (Q65/QRA lineage)
// =============================================================================
//
// fact-documents/tier2-suffix-fec-gf16-spike.md. A symbol-matched (GF(16))
// repeat-accumulate FEC for the M=16 noncoherent-FSK control suffix, decoded by
// Q-ary belief propagation directly from the per-tone ENERGIES (Bessel-I0
// intrinsic metric) — no hard argmax, no bit-LLR marginalization loss. Ported
// from the qracodes algorithm (IV3NWV, GPLv3→AGPLv3): WHT check-node
// convolution + permutation weights + EXIT-chart convergence. Q65 carries the
// CRC as PROTECTED info symbols (QRATYPE_CRC), which this code mirrors so the
// CRC accept-gate is FEC-reliable at the floor.
//
// Code: GF(16), N=20 symbols = K=13 info (10 message + 3 CRC) + 7 RA parity,
// rate 13/20. The 40-bit message [type:2|payload:38] occupies the first 10 info
// symbols (4 bits/symbol, MSB-first — identical bit order to pack_ctrl_suffix);
// the production CRC12 occupies the next 3 info symbols. SIM SPIKE only:
// measurement-only, gated behind cl_telecom_system::suffix_fec_mode==3.

namespace gf16ra {

static const int GF16RA_M       = 16;  // symbol alphabet (M-FSK order)
static const int GF16RA_m       = 4;   // bits/symbol = log2(M)
static const int GF16RA_K_MSG   = 10;  // DEFAULT info symbols carrying the 40-bit message
static const int GF16RA_K_CRC   = 3;   // info symbols carrying the 12-bit CRC (FIXED)
static const int GF16RA_K       = 13;  // DEFAULT total info symbols (message + CRC)
// ULTRA spike (lever A + C): the codeword length N and the info-symbol count K
// are now RUNTIME knobs (configure(repfact,K)). MAX_N raised 64->128 so the
// lowest rates fit: at K=13, repfact=8 -> N = K(1+8) = 117 <= 128 (R 1/9);
// at K=8, repfact=8 -> N = 72; etc. GF16RA_MAX_K caps the info stack arrays /
// the runtime K (the production default K=13 is the maximum useful message).
static const int GF16RA_MAX_K   = 13;  // ceiling for the runtime info-symbol count
static const int GF16RA_MAX_N   = 128; // buffer ceiling for out_tones / energies (lever A: was 64)

// TRUE Q-ary RA structure (matches qracodes / Q65): the parity is a length-NC
// GF(16) accumulator chain, each stage folding in EXACTLY ONE interleaved info
// replica (check degree 3: prev-parity, this-parity, one info edge). NC = total
// info replicas = repfact * K. The transmitted codeword is systematic
// [13 info | NC parity], so N = K + NC and rate R = K/N. repfact is the only
// knob: repfact=1 -> N=26 (R=0.50), repfact=2 -> N=39 (R=0.33),
// repfact=3 -> N=52 (R=0.25, the Q65 operating class). A degree-3 sparse graph
// is the ONLY structure where Q-ary BP delivers multi-symbol correction (a
// high-rate/few-parity code collapses to ~1-symbol correction — measured, see
// tier2-suffix-fec-gf16-spike.md §8). The CRC is carried as PROTECTED info
// (Q65 QRATYPE_CRC) so the recompute-and-compare accept gate is FEC-reliable.

// Configure the code repeat factor (must be called before init()/the first
// encode/decode, or after a reconfigure). Default repfact = 2 (N=39, R=1/3),
// K = GF16RA_K = 13 info symbols. Returns the resulting codeword length N
// (= K + repfact*K). Capped so N <= GF16RA_MAX_N (repfact backs off if needed).
int  configure(int repfact);

// ULTRA spike (lever C): configure with an explicit total info-symbol count
// K_total (message + 3 CRC), 4 <= K_total <= GF16RA_MAX_K(13). The message
// occupies (K_total - 3) systematic symbols = 4*(K_total-3) bits; the CRC stays
// 3 symbols (12 bits, the FAR gate). Fewer info symbols => fewer info bits to
// recover in the same airtime => deeper FEC reach (the +3 dB/halving lever).
// Returns N (= K_total + repfact*K_total). Default configure(repfact) ==
// configure_k(repfact, 13).
int  configure_k(int repfact, int K_total);

int  codeword_len();   // current N (= K + NC); valid after configure()/init()
int  parity_len();     // current NC (= repfact*K)
int  info_len();       // current K (total info symbols, message + 3 CRC)
int  msg_bits();       // current message-bit capacity = 4*(K-3)

// One-time construction of the GF(16) field tables and the RA graph for the
// current repfact (interleaver, accumulator weights). Idempotent until the next
// configure(). Returns true.
bool init();

// Encode: 40-bit message [type:2|payload38:38] + 12-bit crc12 -> N GF(16)
// codeword tones (0..15). out_tones must have room for codeword_len() (<=
// GF16RA_MAX_N). The 10 message symbols + 3 CRC symbols are systematic (appear
// verbatim as the first 13 tones); the remaining NC are RA parity. Bit order in
// the message symbols is MSB-first, matching cl_mfsk::pack_ctrl_suffix.
void encode(uint8_t type, uint64_t payload38, uint16_t crc12, int* out_tones);

// Soft decode from the per-tone ENERGY matrix.
//
//   energies : N*M row-major (N = codeword_len()); energies[s*M + t] = received
//              energy of tone t in codeword symbol s. These are exactly the
//              per-tone energies cl_ofdm::decode_suffix_energies emits (same FFT
//              + de-hop math as decode_suffix_candidates, but the full vector).
//   maxiter  : BP iteration cap (50 is ample).
//   esno_metric : assumed Es/No for the Bessel intrinsic (qra_mfskbesselmetric;
//                 a fixed design point since true Es/No is unknown at ~20 sym).
//   expected_type : required 2-bit type discriminator (reject others).
//   crc12_fn / crc12_ctx : production CRC-12 over the 5-byte [type:2|payload38]
//                 field (NEVER inline — same callback convention as Tier-1).
//
// Decodes the 13 info symbols by Q-ary BP + MAP argmax, reassembles
// [type|payload38] + the decoded 12-bit CRC, recomputes CRC12, and accepts iff
// (a) the recomputed CRC matches the decoded CRC AND (b) type == expected_type.
// On success writes *out_payload38 and (if non-null) *out_iters (BP iterations
// used; -1 if it ran to the cap) and returns true.
bool soft_decode(const double* energies, int maxiter, double esno_metric,
                 uint8_t expected_type,
                 ctrl_crc12_fn crc12_fn, void* crc12_ctx,
                 uint64_t* out_payload38, int* out_iters);

} // namespace gf16ra

#endif // INC_MFSK_CTRL_CODEC_H_
