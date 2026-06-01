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
//   bits 37..36 : echoed_cap     (2)   peer's cap echoed back
//   bits 35..34 : own_cap        (2)   responder's local_capability
//   bits 33..26 : ssid           (8)   0-15 numeric, 16=L, 17=T, 18=R, 19=X,
//                                       255 = SSID_NONE (matches
//                                       `arq.h:60 #define SSID_NONE 0xFF`)
//   bits 25..0  : reserved       (26)  must be 0 on TX, ignored on RX
//
// echoed_cap / own_cap are the 2 valid bits after the 2026-05-24 cap-byte
// collapse: CAP_WB_CAPABLE (0x01), CAP_ENCRYPTION (0x02). Higher bits are
// masked off.
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
//   bits 33..32 : local_cap      (2)   sender's local_capability
//                                       (CAP_WB_CAPABLE=0x01, CAP_ENCRYPTION=0x02)
//   bits 31..24 : ssid           (8)   0-15 numeric, 16=L, 17=T, 18=R, 19=X,
//                                       255 = SSID_NONE (matches
//                                       `arq.h:60 #define SSID_NONE 0xFF`).
//                                       Identical encoding to TEST_ACK.
//   bits 23..0  : reserved       (24)  must be 0 on TX, ignored on RX
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

#endif // INC_MFSK_CTRL_CODEC_H_
