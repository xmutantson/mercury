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

#ifndef INC_DATALINK_LAYER_DATALINK_DEFINES_H_
#define INC_DATALINK_LAYER_DATALINK_DEFINES_H_


//Message status
#define FAILED_ -2
#define ACK_TIMED_OUT -1
#define FREE 0
#define ADDED_TO_LIST 1
#define ADDED_TO_BATCH_BUFFER 2
#define PENDING_ACK 3
#define ACKED 4
#define RECEIVED 5

// link status
#define DROPPED -1
#define IDLE 0
#define CONNECTING 1
#define CONNECTED 2
#define DISCONNECTING 3
#define LISTENING 4
#define CONNECTION_RECEIVED 5
#define CONNECTION_ACCEPTED 6
#define NEGOTIATING 7

//connection status
#define IDLE 0
#define TRANSMITTING_DATA 1
#define RECEIVING 2
#define RECEIVING_ACKS_DATA 3
#define ACKNOWLEDGING_DATA 4
#define TRANSMITTING_CONTROL 5
#define RECEIVING_ACKS_CONTROL 6
#define ACKNOWLEDGING_CONTROL 7

//Connection ID
#define BROADCAST_ID 0x00

//Message type
#define NONE 0x00
#define DATA_LONG 0x10
#define DATA_SHORT 0x11
#define ACK_CONTROL 0x20
#define ACK_RANGE 0x21
#define ACK_MULTI 0x22

#define CONTROL 0x30 //multi-commands or especial commands

//Control commands
#define START_CONNECTION 0x31
#define TEST_CONNECTION 0x32
#define CLOSE_CONNECTION 0x33
#define KEEP_ALIVE 0x34
#define FILE_START 0x35
#define FILE_END_ 0x36
#define PIPE_OPEN 0x37
#define PIPE_CLOSE 0x38
#define SWITCH_ROLE 0x39
#define BLOCK_END 0x3A
#define SET_CONFIG 0x3B
#define REPEAT_LAST_ACK 0x3C
#define SWITCH_BANDWIDTH 0x3D
#define KEY_EXCHANGE_1   0x3E   // X25519 public key (32 bytes)
#define KEY_EXCHANGE_2   0x3F   // ML-KEM encaps key (sent as data, 1184 bytes)
#define KEY_EXCHANGE_3   0x40   // ML-KEM ciphertext (sent as data, 1088 bytes)
#define KEY_ACTIVATE     0x41   // Encryption activated (both sides switch to encrypted data)
#define SACK_RSP         0x42   // SACK Design A §4.2.2 — OFDM control frame carrying a
                                // partial-batch ACK bitmap. Replaces the legacy MFSK SACK
                                // pattern (~1168 ms on wire at WB M=16) with a single OFDM
                                // LDPC control frame (~390 ms at WB_CFG10) on
                                // sack_v2_enabled sessions. Wire payload (after the
                                // standard 3-byte msg header [type, conn_id, seq_num]):
                                //   [batch_seq_id : u8][bitmap : ceil(N/8) bytes][CRC8 : u8]
                                // where N = data_batch_size at TX time (CMD and RSP agree
                                // on N because data_batch_size is negotiated at TEST_CONNECTION
                                // and never moves within a Design A Step 7 session). CRC8
                                // covers batch_seq_id || bitmap_bytes (NOT the standard
                                // msg header). Polynomial: POLY_CRC8 (=0xF4), matching the
                                // existing CRC8_calc() helper. v1 MFSK SACK path is
                                // untouched; it remains operational on
                                // sack_enabled && !sack_v2_enabled peers.
#define SET_LINK_PARAMS  0x43   // SACK Design A Axes 2+3 policy update (batch, sack mode)
                                // Phase: SCAFFOLDING — message type defined, RSP-side no-op
                                // stub logs receipt only; no CMD-side sender; no state mutation.
                                // Gated behavior arrives in later Design A steps.
#define TEST_CONNECTION_ACK 0x45  // Handshake echo. RSP-initiated LDPC reply
                                // to CMD's TEST_CONNECTION, always sent in
                                // place of the legacy ACK pattern. Wire
                                // payload after the 3-byte msg header:
                                //   [echoed_peer_cap : u8]
                                //   [own_capability : u8]
                                //   [CRC8 : u8]
                                // CRC8 over [echoed_peer_cap, own_cap], POLY=0xF4.
                                // CMD validates echoed_peer_cap == local_capability
                                // before transitioning out of CONNECTION_ACCEPTED.
#define ROBUST_DWELL_BATCH_OP 0x44  // FIX-A: ROBUST-tier dwell-batch decouple
                                // (data-flow-robust-tier-arq-batch.md §5.2). CMD-decided,
                                // RSP-mirrored batch size for a PROVEN+PARKED robust dwell.
                                // Distinct from SET_LINK_PARAMS (0x43) ON PURPOSE: that op's
                                // RSP handler clamps batch to [AXIS2_BATCH_FLOOR=10,32]
                                // (the OFDM Axis-2 contract), which would force a robust
                                // 4-8 batch UP to 10 on the RSP only → CMD≠RSP all-ones
                                // target mismatch → the literal Bug-3 4-wire-failure
                                // (OR-2 / landmine L4). This op applies the value straight
                                // through the relaxed set_data_batch_size() chokepoint
                                // (clamped only to the robust [1..ROBUST_DWELL_BATCH_MAX]
                                // range), so CMD and RSP converge on the SAME value.
                                // Wire payload after the 3-byte msg header:
                                //   [batch : u8][CRC8 : u8]   (length=3 incl. data[0]=op)
                                // CRC8 over data[1] only, POLY_CRC8=0xF4 (matches SACK_RSP /
                                // SET_LINK_PARAMS coverage rule). batch ∈ [1..ROBUST_DWELL_BATCH_MAX].
                                // (Reuses the 0x44 slot freed when OFDM_ACK_CLEAN was
                                // removed 2026-05-24 — the MFSK ACK+SACK pattern carrying
                                // [bsi:8|bitmap:32|crc12:12] superseded that frame type;
                                // see mercury/fact-documents/mfsk-robust-ack.md.)

// Capability flags (embedded in TEST_CONNECTION byte 5).
// Down to two bits after the 2026-05-24 capability cleanup: compression /
// streaming / B2F unroll / SACK / SACK_v2 / handshake-echo were always-on or
// CLI-only and are now unconditional in the codebase.
#define CAP_WB_CAPABLE   0x01   // Supports wideband upgrade after NB connection
#define CAP_ENCRYPTION   0x02   // Supports hybrid PQ encryption (X25519 + ML-KEM-768)
// The enhanced ctrl-suffix (GF(16) RA FEC + base-pattern combining) on the MFSK
// CONNECT handshake (tier2-suffix-fec-design.md §21) is NOT capability-negotiated:
// Mercury shipped no version, so there are no legacy peers, and the GF(16) RA
// codeword is systematic (backward-compatible by construction). It is the
// unconditional default at the robust tier, triggered by the gearshift config
// (is_robust_config). The former CAP_SUFFIX_FEC (0x04) negotiation bit was
// removed in cleanup/drop-suffix-fec-cap.
// Bits 0..1 are the negotiable cap bits carried in the 2-bit MFSK ctrl-suffix cap
// fields (TEST_ACK echoed_cap/own_cap, TEST_CONN local_cap). Packers/unpackers
// mask to this; higher bits are not on the MFSK wire.
#define CAP_NEGOTIABLE_MASK 0x03

// Bandwidth mode (persisted in INI, controls NB/WB negotiation)
enum BandwidthMode { BW_AUTO = 0, BW_NB_ONLY = 1 };

//Error control
#define MESSAGE_ID_ERROR -4
#define MEMORY_ERROR -3
#define MESSAGE_LENGTH_ERROR -2
#define ERROR_ -1
#define SUCCESSFUL 0

// P2 big-block ARQ re-granularization stub sentinel
// (fact-documents/data-flow-bigblock-arq-unit.md §6): returned by
// bigblock_block_to_arq() while the block->ARQ logic is NOT yet wired (P2.0). The
// --test-bigblock-arq-unit regression treats this as "block not delivered" so the
// test FAILS before P2 wiring and PASSES after (the stub body is replaced in
// P2.4/2.5/2.6). Distinct from any success/error code above so a future real
// implementation can never collide with it.
#define BIGBLOCK_ARQ_NOT_WIRED -77

// PHASE 1 (P3 prereq, fact-documents/data-flow-bigblock-arq-unit.md §11): the
// big-block carries a SELF-DESCRIBING header ON THE WIRE so a SUSTAINED multi-block
// session cannot drift the CMD/RSP block bsi (the block has ONE acquisition + no
// per-frame wire bit-7 to carry it) AND so the RX delivers each sub-codeword its
// EXACT frame length (REQUIRED for compression transparency: variable-length
// compressed frames must reassemble byte-faithfully — §11.4 / INV-10).
//
// The header rides as a PREFIX of codeword 0's systematic info bits ⇒ LDPC-protected,
// decoded with the block. Layout (LSB-first bytes, hdr_total = BB_HDR_FIXED + 2*K):
//   [0] = block_bsi (low 8 bits)            <- the authoritative wire bsi
//   [1] = n_data    (0..K filled codewords)
//   [BB_HDR_FIXED + 2*c + 0] = length[c] low  byte   } per-codeword app length (uint16 LE),
//   [BB_HDR_FIXED + 2*c + 1] = length[c] high byte   }  c = 0..K-1
// cw0's app bytes start at offset hdr_total (cw0 capacity = sub_len - hdr_total);
// cwc (c>=1) app bytes occupy [c*sub_len .. c*sub_len + length[c]) (codeword-aligned
// so the K-bit cw_ok SACK granularity / selective-repeat stays frame == codeword).
#define BIGBLOCK_HDR_FIXED_BYTES 2                            // bsi + n_data
#define BIGBLOCK_HDR_TOTAL_BYTES(K) (BIGBLOCK_HDR_FIXED_BYTES + 2*(K))  // + uint16 length table

//Node role
#define COMMANDER 0
#define RESPONDER 1

//Gear shift
#define GEAR_SHIFT 255

//Gear shift algorithms
#define SNR_BASED 0
#define SUCCESS_BASED_LADDER 1

//Header length
#define ACK_MULTI_ACK_RANGE_HEADER_LENGTH 3
#define CONTROL_ACK_CONTROL_HEADER_LENGTH 3
#define DATA_LONG_HEADER_LENGTH 4
#define DATA_SHORT_HEADER_LENGTH 5

// SACK Design A Step 1 — gated DATA_LONG header growth.
// When sack_v2_enabled is false (v1 path / default), DATA_LONG header is 4
// bytes [type, conn_id, seq_num(EOB bit7), id] — identical to pre-Step-1
// wire format. When sack_v2_enabled is true, DATA_LONG header grows to 5
// bytes [type, conn_id, seq_num(EOB bit7), batch_seq_id, id]. The
// batch_seq_id byte is a placeholder (0) until Step 3 plumbs the real
// counter. Header growth is the ONLY irreversible wire change in this
// step; v1↔v1 traffic remains byte-identical. See SACK_DESIGN_A_PLAN.md
// §4.2.1 and §7 (revised order).
#define DATA_LONG_HEADER_LENGTH_V2 5
// SACK Design A Step 2 — gated DATA_SHORT header growth.
// When sack_v2_enabled is false (v1 path / default), DATA_SHORT header is
// 5 bytes [type, conn_id, seq_num(EOB bit7), id, length] — identical to
// pre-Step-2 wire format. When sack_v2_enabled is true, DATA_SHORT header
// grows to 6 bytes [type, conn_id, seq_num(EOB bit7), batch_seq_id, id,
// length]. Same placeholder-then-plumb semantics as DATA_LONG_HEADER_LENGTH_V2.
#define DATA_SHORT_HEADER_LENGTH_V2 6

//Load config level
#define FULL 0
#define PHYSICAL_LAYER_ONLY 1

#define INFINITE_ -1

#define POLY_CRC8 0xF4
// CRC-12-CDMA2000 forward polynomial (x^12 + x^11 + x^10 + x^9 + x^8 + x^4 + x + 1).
// Used by CRC12_calc() over the 40-bit MFSK ACK+SACK payload — see
// mercury/fact-documents/mfsk-robust-ack.md §3.2.
#define POLY_CRC12 0xF13

#endif
