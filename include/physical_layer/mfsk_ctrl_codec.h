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
//   bits 37..35 : echoed_cap     (3)   peer's cap echoed back
//   bits 34..32 : own_cap        (3)   responder's local_capability
//   bits 31..24 : ssid           (8)   0-15 numeric, 16=L, 17=T, 18=R, 19=X,
//                                       255 = SSID_NONE (matches
//                                       `arq.h:60 #define SSID_NONE 0xFF`)
//   bits 23..0  : reserved       (24)  must be 0 on TX, ignored on RX
//
// echoed_cap / own_cap carry the 3 valid cap bits: CAP_WB_CAPABLE (0x01),
// CAP_ENCRYPTION (0x02), CAP_COHERENT_TIER (0x04). The fields were WIDENED 2->3
// bits in Phase 4 (phase4-coherent-tier-design.md) to carry CAP_COHERENT_TIER,
// each stealing one bit from the formerly-26-bit reserved field. Higher bits are
// masked off (& 0x7). Old peers send 0x04's position as reserved-must-be-0 ->
// additive, old<->new safe.
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
//   bits 33..31 : local_cap      (3)   sender's local_capability (CAP_WB_CAPABLE
//                                       =0x01, CAP_ENCRYPTION=0x02,
//                                       CAP_COHERENT_TIER=0x04). WIDENED 2->3
//                                       bits in Phase 4 (steals one reserved bit).
//   bits 30..23 : ssid           (8)   0-15 numeric, 16=L, 17=T, 18=R, 19=X,
//                                       255 = SSID_NONE (matches
//                                       `arq.h:60 #define SSID_NONE 0xFF`).
//                                       Identical encoding to TEST_ACK.
//   bits 22..0  : reserved       (23)  must be 0 on TX, ignored on RX
//
// Site F (RSP RX) reconstructs the legacy float SNR via
// cl_mfsk::tone_to_snr(snr_q). 2 dB quantization step is documented in the
// v3_test_conn_snr_quantization_roundtrip regression test.
void pack_test_conn_payload(uint64_t* p38, uint8_t snr_q,
                             uint8_t local_cap, uint8_t ssid);

bool unpack_test_conn_payload(uint64_t p38, uint8_t* snr_q,
                               uint8_t* local_cap, uint8_t* ssid);

#endif // INC_MFSK_CTRL_CODEC_H_
