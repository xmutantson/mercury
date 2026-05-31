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

#include "physical_layer/mfsk_ctrl_codec.h"

#include <cstdio>
#include <cstring>

// =============================================================================
// 6-bit base-36 callsign body codec
// =============================================================================

static const int CALLSIGN_BODY_SENTINEL = 36;  // end-of-string marker

// Translate one char to its 6-bit codepoint. Returns the codepoint
// (0..35) for supported chars, or 0 with a one-time stderr warning for
// non-encodable chars (matching the legacy callsign_pack behavior at
// arq.h:124-128).
static int encode_callsign_char(char c, bool* warned)
{
	// Uppercase a-z
	if (c >= 'a' && c <= 'z') c -= 32;
	if (c >= 'A' && c <= 'Z') return (int)(c - 'A');         // 0..25
	if (c >= '0' && c <= '9') return (int)(c - '0') + 26;    // 26..35
	if (warned && !*warned) {
		fprintf(stderr,
			"[MFSK-CTRL-CODEC] callsign char '%c' (0x%02x) not in "
			"[A-Z0-9]; substituting 'A'\n",
			(c >= 32 && c < 127) ? c : '?',
			(unsigned)(unsigned char)c);
		*warned = true;
	}
	return 0;  // silently substitute 'A' (matches legacy callsign_pack)
}

int pack_callsign_body_b36(const char* call, int call_len, uint64_t* out36)
{
	if (!out36) return 0;
	*out36 = 0;
	if (!call) call_len = 0;
	if (call_len < 0) call_len = 0;
	if (call_len > 6) call_len = 6;

	bool warned = false;
	int packed = 0;
	uint64_t body = 0;
	for (int i = 0; i < 6; i++) {
		int val;
		if (i < call_len) {
			val = encode_callsign_char(call[i], &warned);
			packed = i + 1;
		} else {
			val = CALLSIGN_BODY_SENTINEL;
		}
		// Char i occupies bits 35-6i..30-6i (MSB-first; char 0 high).
		body |= ((uint64_t)(val & 0x3F)) << (30 - 6 * i);
	}
	*out36 = body;
	return packed;
}

int unpack_callsign_body_b36(uint64_t in36, char out[7])
{
	if (!out) return 0;
	int n = 0;
	for (int i = 0; i < 6; i++) {
		int val = (int)((in36 >> (30 - 6 * i)) & 0x3F);
		if (val == CALLSIGN_BODY_SENTINEL) break;
		if (val < 26) {
			out[n++] = (char)('A' + val);
		} else if (val < 36) {
			out[n++] = (char)('0' + (val - 26));
		} else {
			// Out-of-alphabet code (37..63). Treat as end-of-string.
			break;
		}
	}
	out[n] = '\0';
	return n;
}

// =============================================================================
// MFSK_CTRL_START_CONN (type=01) — 38-bit payload
// =============================================================================
//
//   bits 37    : nb_flag           (1)
//   bits 36..1 : sender_body       (36)   6 chars * 6 bits
//   bits 0     : reserved          (1)
//

void pack_start_conn_payload(uint64_t* p38, bool nb_flag,
                              const char* call, int call_len)
{
	if (!p38) return;
	uint64_t body36 = 0;
	(void)pack_callsign_body_b36(call, call_len, &body36);
	// body36 occupies bits 35..0; shift to bits 36..1 in the 38-bit field.
	uint64_t v = 0;
	v |= ((uint64_t)(nb_flag ? 1u : 0u)) << 37;
	v |= (body36 & ((1ULL << 36) - 1ULL)) << 1;
	// reserved bit (bit 0) MUST be zero on TX
	*p38 = v & ((1ULL << 38) - 1ULL);
}

bool unpack_start_conn_payload(uint64_t p38, bool* out_nb_flag,
                                char out_call[7], int* out_len)
{
	if (!out_nb_flag || !out_call || !out_len) return false;
	uint64_t v = p38 & ((1ULL << 38) - 1ULL);
	*out_nb_flag = ((v >> 37) & 0x1ULL) != 0;
	uint64_t body36 = (v >> 1) & ((1ULL << 36) - 1ULL);
	*out_len = unpack_callsign_body_b36(body36, out_call);
	return true;
}

// =============================================================================
// MFSK_CTRL_TEST_ACK (type=10) — 38-bit payload
// =============================================================================
//
//   bits 37..35 : echoed_cap     (3)   WIDENED 2->3 for CAP_COHERENT_TIER (0x04)
//   bits 34..32 : own_cap        (3)   WIDENED 2->3 for CAP_COHERENT_TIER (0x04)
//   bits 31..24 : ssid           (8)
//   bits 23..0  : reserved       (24)
//
// Phase 4 (phase4-coherent-tier-design.md / robust3-phase1-plan.md §4.1): the
// cap fields were widened from 2 to 3 bits, each stealing one bit from the
// (formerly 26-bit) reserved field. CAP_COHERENT_TIER=0x04 needs the 3rd bit.
// Backward-compat: an OLD peer left bit 0x04's position as reserved-must-be-0,
// so it reads as "no coherent tier" on a new RX, and never sets it on TX — the
// widening is additive and old<->new safe in both directions.

void pack_test_ack_payload(uint64_t* p38, uint8_t echoed_cap,
                            uint8_t own_cap, uint8_t ssid)
{
	if (!p38) return;
	uint64_t v = 0;
	v |= ((uint64_t)(echoed_cap & 0x7)) << 35;
	v |= ((uint64_t)(own_cap    & 0x7)) << 32;
	v |= ((uint64_t)(ssid       & 0xFF)) << 24;
	// reserved (bits 23..0) MUST be zero on TX
	*p38 = v & ((1ULL << 38) - 1ULL);
}

bool unpack_test_ack_payload(uint64_t p38, uint8_t* echoed_cap,
                              uint8_t* own_cap, uint8_t* ssid)
{
	if (!echoed_cap || !own_cap || !ssid) return false;
	uint64_t v = p38 & ((1ULL << 38) - 1ULL);
	*echoed_cap = (uint8_t)((v >> 35) & 0x7);
	*own_cap    = (uint8_t)((v >> 32) & 0x7);
	*ssid       = (uint8_t)((v >> 24) & 0xFF);
	return true;
}

// =============================================================================
// MFSK_CTRL_TEST_CONN (type=11) — 38-bit payload  (Wave 3, §14)
// =============================================================================
//
//   bits 37..34 : snr_q          (4)
//   bits 33..31 : local_cap      (3)   WIDENED 2->3 for CAP_COHERENT_TIER (0x04)
//   bits 30..23 : ssid           (8)
//   bits 22..0  : reserved       (23)
//
// Phase 4: local_cap widened 2->3 bits (steals one reserved bit) for
// CAP_COHERENT_TIER=0x04. snr_q [37:34] is UNCHANGED. See TEST_ACK note above
// and robust3-phase1-plan.md §4.1 for the backward-compat reasoning.

void pack_test_conn_payload(uint64_t* p38, uint8_t snr_q,
                             uint8_t local_cap, uint8_t ssid)
{
	if (!p38) return;
	uint64_t v = 0;
	v |= ((uint64_t)(snr_q     & 0xF))  << 34;
	v |= ((uint64_t)(local_cap & 0x7))  << 31;
	v |= ((uint64_t)(ssid      & 0xFF)) << 23;
	// reserved (bits 22..0) MUST be zero on TX
	*p38 = v & ((1ULL << 38) - 1ULL);
}

bool unpack_test_conn_payload(uint64_t p38, uint8_t* snr_q,
                               uint8_t* local_cap, uint8_t* ssid)
{
	if (!snr_q || !local_cap || !ssid) return false;
	uint64_t v = p38 & ((1ULL << 38) - 1ULL);
	*snr_q     = (uint8_t)((v >> 34) & 0xF);
	*local_cap = (uint8_t)((v >> 31) & 0x7);
	*ssid      = (uint8_t)((v >> 23) & 0xFF);
	return true;
}
