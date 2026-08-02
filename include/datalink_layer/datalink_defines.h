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
                                // RSP handler clamps batch to the negotiated Axis-2 range
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
                                // [bsi:8|bitmap:30|crc12:12] superseded that frame type;
                                // see mercury/fact-documents/mfsk-robust-ack.md.)

// Capability flags (embedded in TEST_CONNECTION byte 5).
// Down to two bits after the 2026-05-24 capability cleanup: compression /
// streaming / B2F unroll / SACK / SACK_v2 / handshake-echo were always-on or
// CLI-only and are now unconditional in the codebase.
#define CAP_WB_CAPABLE   0x01   // Supports wideband upgrade after NB connection
#define CAP_ENCRYPTION   0x02   // Supports hybrid PQ encryption (X25519 + ML-KEM-768)
// CAP_CUMULATIVE_ACK (FORGIVING-ACK Tier 2 — fact-documents/data-flow-forgiving-ack.md
// §T2.1): the peer interprets the SACK's 8-bit bsi field as a CUMULATIVE n_r
// (= rsp_last_delivered_batch_seq_id, the contiguous delivery high-water) instead
// of a per-batch bsi. SEMANTICS-only — no wire-width change. Engages ONLY when
// BOTH ends advertise it (the both_support pattern, like CAP_ENCRYPTION) AND the
// env opt-in MERCURY_CUMULATIVE_ACK gates the local advertise, so default-off ≡
// byte-identical + interop-safe with any non-Tier-2 peer (which never sets bit 2 →
// both_support false → per-batch fallback). This reclaims the 0x04 slot the former
// CAP_SUFFIX_FEC used before it was removed in cleanup/drop-suffix-fec-cap.
#define CAP_CUMULATIVE_ACK 0x04 // Supports cumulative-n_r (high-water) SACK semantics
// Scalable mode and both peers must enable this before D5 bit 7 may mark the
// physical last frame of a proven selective-retry turn. Legitimate D5 spans are bounded by
// MAX_SACK_BATCH_SIZE (96), so bits 0..6 retain the complete span. An older
// peer leaves bit 3 clear and keeps the conservative previous-ACK defer.
#define CAP_RETX_TURN_TAIL 0x08 // Supports the D5 selective-retry turn-tail marker
// NB robust-preamble capability (robust-preamble rollout step 1+2): this
// peer's RX acquisition detector carries the NB sidelnikov robust-DATA
// preamble tables (32-symbol M=8 / 48-symbol M=4) IN ADDITION to the legacy
// 8-symbol sequences (detect-both), so a peer that sees this bit may emit the
// sidelnikov preamble at the NB robust tier. Symmetric decision rule on both
// ends: sidelnikov-TX <=> local advertises AND peer advertised. Absent bit
// (old build, lost byte, 4-bit ctrl-suffix path) => legacy preamble both ways
// — the interop floor. Advertising is a promise about RX capability only.
// 0x20 is reserved for the WB robust-preamble analog (not shipped: the WB
// sidelnikov preamble measured a wash at operating SNRs).
#define CAP_ROBUST_PREAMBLE_NB 0x10 // RX can acquire the NB sidelnikov robust preamble
// The enhanced ctrl-suffix (GF(16) RA FEC + base-pattern combining) on the MFSK
// CONNECT handshake (tier2-suffix-fec-design.md §21) is NOT capability-negotiated:
// Mercury shipped no version, so there are no legacy peers, and the GF(16) RA
// codeword is systematic (backward-compatible by construction). It is the
// unconditional default at the robust tier, triggered by the gearshift config
// (is_robust_config). (The §21 3rd-bit was removed in cleanup/drop-suffix-fec-cap;
// the slot is now CAP_CUMULATIVE_ACK above.)
// Bits 0..4 are the negotiable cap bits carried in the MFSK ctrl-suffix cap
// fields (TEST_ACK echoed_cap/own_cap, TEST_CONN local_cap), which use the formerly
// reserved payload bits for bits 2, 3 and 4 (the §21 precedent; no payload-width
// change — an older peer transmits 0 there and ignores them on RX), and
// the full LDPC TEST_CONNECTION/ACK capability byte. Packers/unpackers mask to this.
#define CAP_NEGOTIABLE_MASK 0x1F

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
// BLOCK-CRC (D2_BLOCKCRC, fix/bigblock-d3-carve): a WHOLE-BLOCK CRC-32 over the
// assembled de-whitened K*sub_len payload. It is the block-level integrity anchor
// STACKED ON TOP of the K per-codeword CRC-8s: the HW NO-GO (WINRUN_FINAL_VERDICT.json)
// showed all 8 per-cw CRC-8 FALSE-PASSING a corrupt K=8 block (the LDPC miscorrected
// each codeword to a valid-but-wrong word whose recomputed CRC-8 still matched),
// delivering 1374 wrong bytes. An 8-bit per-cw gate cannot rule that out (2^-8 residual
// per cw, and the failure was self-consistent ACROSS all 8). CRC-32 (poly 0x04C11DB7,
// reflected 0xEDB88320 — the IEEE 802.3 CRC the in-tree ffbase crc32 family uses) gives
// a ~2^-32 block-level residual with Hamming distance >= 4 well past the ~11200-bit
// block, at a 4-byte/block (=0.29% of 1374B) cost. CRC-16's 2^-16 residual is too weak
// for a life-critical "silent wrong-byte must be impossible" guarantee.
//
// PLACEMENT: the 4 block-CRC bytes sit in the LAST codeword (cw K-1), immediately BEFORE
// that codeword's own per-cw CRC-8 tail byte. This deliberately does NOT touch cw0's
// header (BIGBLOCK_HDR_TOTAL_BYTES is unchanged at 2+2*K) so cw0's app capacity — which
// must hold the first ARQ frame — is NOT reduced (a 4-byte cw0 shrink dropped it below the
// ~155-byte frames and tripped the bigblock decline guard, stalling the block). cw K-1
// (app cap sub_len-1) loses only 4 of its ~174 bytes, far above any frame. The offset is
// FIXED (geometry-derived) so the RX can locate it WITHOUT trusting the length table.
//
// COVERAGE / SELF-REFERENCE: the CRC-32 covers the ENTIRE K*sub_len payload with TWO sets
// of bytes treated as ZERO — its own 4 block-CRC bytes AND all K per-codeword CRC-8 tail
// bytes — so the two CRC layers are decoupled (neither's tail bytes feed the other) and TX
// and RX (which de-whitens first) compute over an identical image with no circular
// dependency. Checked at RX in bigblock_receive_carve BEFORE bigblock_block_to_arq, ONLY
// when the per-cw layer reports the block fully clean (n_clean==K); on mismatch ALL cw_ok
// are cleared -> the block routes to the EXISTING PARTIAL/SACK gap path (re-send) and is
// NEVER delivered. Does NOT weaken the per-cw CRC-8 (both are kept): both run, the CRC-8
// still picks the per-codeword SACK granularity when the block is partial.
#define BIGBLOCK_BLOCK_CRC_BYTES 4                            // 1 CRC-32 byte-quad / block
// byte offset of the block CRC-32 within the K*sub_len payload: the 4 bytes just before
// cw (K-1)'s per-cw CRC-8 tail byte. uint32 little-endian. Requires sub_len large enough
// to hold both (sub_len > BIGBLOCK_CW_CRC_BYTES + BIGBLOCK_BLOCK_CRC_BYTES); the CFG16 thin
// grid (sub_len=175) has ample room. The RX/TX guard the bound before using it.
#define BIGBLOCK_BLOCK_CRC_OFFSET(K, sub_len) \
	((long)(K)*(long)(sub_len) - BIGBLOCK_CW_CRC_BYTES - BIGBLOCK_BLOCK_CRC_BYTES)

// FAILURE-2 fix (per-codeword CRC-8 ON THE WIRE). The LDPC iter count does NOT
// detect a MISCORRECTION (the decoder converges to a valid-but-wrong codeword on a
// noisy channel), and the only per-codeword "clean" check on the live 2-instance
// path was an ORACLE compare against the TX instance's own info bits — valid ONLY in
// single-instance loopback (telecom_system.cc bigblock_rx_passband cw_info_ref). On
// the live path that ref is NULL -> cw_ok was FORCED CLEAN -> a corrupted codeword was
// delivered clean + NEVER retransmitted (silent corruption). The fix reserves ONE
// CRC-8 byte per sub-codeword (mirrors the stock per-frame CRC; CRC8_calc + POLY_CRC8),
// at a FIXED offset the RX can locate WITHOUT trusting the (possibly-corrupt) length
// table: the LAST byte of each codeword's sub_len systematic-byte region. The CRC
// covers that codeword's first (sub_len - 1) bytes — for cw0 that INCLUDES the wire
// header, so a corrupted header is caught (§5: a CRC-failed cw0 forces the fallback-bsi
// path and is NOT length-table-parsed). The RX recomputes the CRC over the de-whitened
// payload and DEMOTES cw_ok[c] on mismatch (can only demote, never promote a genuine
// bit-mismatch to clean), feeding the existing SACK selective-repeat.
#define BIGBLOCK_CW_CRC_BYTES 1                               // 1 CRC-8 byte / sub-codeword
// byte offset of codeword c's CRC within the K*sub_len block payload, and the span the
// CRC covers (the codeword's systematic bytes EXCLUDING its own CRC byte).
#define BIGBLOCK_CW_CRC_OFFSET(c, sub_len) ((c)*(sub_len) + (sub_len) - BIGBLOCK_CW_CRC_BYTES)
#define BIGBLOCK_CW_CRC_SPAN(sub_len)      ((sub_len) - BIGBLOCK_CW_CRC_BYTES)

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
// wire format. When sack_v2_enabled is true, DATA_LONG header grows to 6
// bytes [type, conn_id, seq_num(EOB bit7), batch_seq_id, id, batch_total_frames].
// The batch_seq_id byte (Step 3) and batch_total_frames byte (D5) are both
// v2-only. Header growth is the ONLY irreversible wire change here; v1↔v1
// traffic remains byte-identical. See SACK_DESIGN_A_PLAN.md §4.2.1 / §7 and
// TRACK_C_D2D3D5_DESIGN.md §5.3 (the D5 batch_total_frames carrier).
//
// D5 (EOB-inference batch truncation): batch_total_frames is the TX-authoritative
// per-batch frame count (message_batch_counter_tx, 1..MAX_SACK_BATCH_SIZE),
// written IDENTICALLY on EVERY data frame of a batch so it survives the loss of
// any single frame — including the EOB-bit-7-marked last frame, whose loss
// previously erased the batch length and silently truncated the delivered batch
// (PREV_BUMP_VERDICT.md §2). 0 on the wire = unknown/legacy (RX falls back to the
// EOB inference). The byte is the highest-offset header field so the v1/v2 parse
// of every PRIOR field (type/conn_id/seq/batch_seq_id/id/length) is byte-identical.
#define DATA_LONG_HEADER_LENGTH_V2 6
// D5 base (without the batch_total_frames byte) — the v2 DATA_LONG header BEFORE
// D5. Robust / batch=1 configs use this (D5 is meaningless at batch=1 and the
// extra byte would steal the scarce ROBUST_0 payload, re-opening the streaming-
// compression deadlock floor max_frame >= COMPRESS_HEADER_SIZE; see
// test_robust0_compress_deadlock C0). OFDM multi-frame configs use the +D5 value.
#define DATA_LONG_HEADER_LENGTH_V2_NO_D5 5
// SACK Design A Step 2 — gated DATA_SHORT header growth.
// When sack_v2_enabled is false (v1 path / default), DATA_SHORT header is
// 5 bytes [type, conn_id, seq_num(EOB bit7), id, length] — identical to
// pre-Step-2 wire format. When sack_v2_enabled is true, DATA_SHORT header
// grows to 7 bytes [type, conn_id, seq_num(EOB bit7), batch_seq_id, id,
// length, batch_total_frames]. Same placeholder-then-plumb semantics as
// DATA_LONG_HEADER_LENGTH_V2; the batch_total_frames byte (D5) is appended last.
#define DATA_SHORT_HEADER_LENGTH_V2 7
// D5 base (without the batch_total_frames byte). Same rationale as
// DATA_LONG_HEADER_LENGTH_V2_NO_D5 — robust / batch=1 keeps the pre-D5 length.
#define DATA_SHORT_HEADER_LENGTH_V2_NO_D5 6

// Option W (data-flow-stream-offset.md §8) — the absolute-byte-stream stamp carried
// on the EOB-bearing DATA frame ONLY, at OFDM (header_carries_d5) configs where
// max_frame is large enough (w_stamp_rides()). Layout appended AFTER the D5
// batch_total_frames byte on the EOB frame: [ start_lo32 : u32 LE ][ length16 : u16 LE ].
//   start_lo32 = tx_stream_stamp[bsi].start & 0xFFFFFFFF (wrapping; 4 GB session window)
//   length16   = tx_stream_stamp[bsi].length (a batch's transported bytes <= ~16 KB)
#define W_EOB_STAMP_BYTES 6
// Payload the build MUST reserve on the EOB (last) frame so header+payload+stamp fits
// the LDPC codeword C (= max_frame + 6): the 6 stamp bytes PLUS 1 for the DATA_SHORT
// header being one byte wider than DATA_LONG (the reserved frame is always < max_frame
// => DATA_SHORT). Capping the last frame's payload at (max_frame - W_EOB_RESERVE) makes
// short_hdr(7) + payload + stamp(6) == C exactly. Derivation: data-flow-stream-offset.md §8.1.
#define W_EOB_RESERVE 7
// Minimum max_frame for the stamp to ride. Below this, reserving W_EOB_RESERVE would
// starve payload, so the stamp is skipped (deterministic on both peers). The 4 captured
// silent-shift mechanisms all occur at cfg13-16 where max_frame is 100+ bytes.
#define W_STAMP_MIN_MAXFRAME (W_EOB_RESERVE + 8)

// Option W STEP 3 (data-flow-stream-offset.md §8.6) — the end-to-end EOT exchange.
// The running stream CRC-32 init (standard reflected IEEE 802.3 register seed). Both
// peers seed identically and compare the running registers (no final XOR needed).
#define CRC32_INIT 0xFFFFFFFFu
// The EOT payload rides the EXISTING CLOSE_CONNECTION (0x33) control frame, appended after
// data[0]=code (no new handshake): [ committed:u64 LE ][ crc32:u32 LE ][ crc8:u8 ] = 13 B,
// so messages_control.length = 1 + 13 = 14 (well within control-frame capacity; KEY_EXCHANGE_1
// carries 33). crc8 (POLY_CRC8) over data[1..12] lets the RX reject a short/legacy/garbled
// frame as ABSENT-EOT (safe no-op) rather than false-fire. Sent once per graceful disconnect.
#define W_EOT_PAYLOAD_BYTES 13
#define W_EOT_FRAME_LENGTH  (1 + W_EOT_PAYLOAD_BYTES)

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
