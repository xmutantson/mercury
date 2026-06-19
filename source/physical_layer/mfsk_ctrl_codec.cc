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
#include <cstdint>
#include <cmath>
#include <queue>
#include <vector>

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
//   bits 37..36 : echoed_cap[1:0] (2)
//   bits 35..34 : own_cap[1:0]    (2)
//   bits 33..26 : ssid            (8)
//   bits 25..0  : reserved        (26)
//
// The cap fields carry the 2 negotiable MFSK-wire bits (CAP_WB_CAPABLE,
// CAP_ENCRYPTION). (The former §21 3rd-bit widening that carried CAP_SUFFIX_FEC
// in bits 25/24 was removed in cleanup/drop-suffix-fec-cap: Mercury shipped no
// version, so there were no legacy peers to negotiate against — the enhanced
// ctrl-suffix is the unconditional default at the robust tier, gated on the
// gearshift config, not on a negotiated bit.)

void pack_test_ack_payload(uint64_t* p38, uint8_t echoed_cap,
                            uint8_t own_cap, uint8_t ssid)
{
	if (!p38) return;
	uint64_t v = 0;
	v |= ((uint64_t)(echoed_cap & 0x3)) << 36;
	v |= ((uint64_t)(own_cap    & 0x3)) << 34;
	v |= ((uint64_t)(ssid       & 0xFF)) << 26;
	// reserved (bits 25..0) MUST be zero on TX
	*p38 = v & ((1ULL << 38) - 1ULL);
}

bool unpack_test_ack_payload(uint64_t p38, uint8_t* echoed_cap,
                              uint8_t* own_cap, uint8_t* ssid)
{
	if (!echoed_cap || !own_cap || !ssid) return false;
	uint64_t v = p38 & ((1ULL << 38) - 1ULL);
	*echoed_cap = (uint8_t)((v >> 36) & 0x3);
	*own_cap    = (uint8_t)((v >> 34) & 0x3);
	*ssid       = (uint8_t)((v >> 26) & 0xFF);
	return true;
}

// =============================================================================
// MFSK_CTRL_TEST_CONN (type=11) — 38-bit payload  (Wave 3, §14)
// =============================================================================
//
//   bits 37..34 : snr_q          (4)
//   bits 33..32 : local_cap[1:0] (2)
//   bits 31..24 : ssid           (8)
//   bits 23..0  : reserved       (24)
//
// The cap field carries the 2 negotiable MFSK-wire bits (CAP_WB_CAPABLE,
// CAP_ENCRYPTION). (The former §21 3rd-bit widening that carried CAP_SUFFIX_FEC
// in bit 23 was removed in cleanup/drop-suffix-fec-cap — see pack_test_ack_payload.)

void pack_test_conn_payload(uint64_t* p38, uint8_t snr_q,
                             uint8_t local_cap, uint8_t ssid)
{
	if (!p38) return;
	uint64_t v = 0;
	v |= ((uint64_t)(snr_q     & 0xF))  << 34;
	v |= ((uint64_t)(local_cap & 0x3))  << 32;
	v |= ((uint64_t)(ssid      & 0xFF)) << 24;
	// reserved (bits 23..0) MUST be zero on TX
	*p38 = v & ((1ULL << 38) - 1ULL);
}

bool unpack_test_conn_payload(uint64_t p38, uint8_t* snr_q,
                               uint8_t* local_cap, uint8_t* ssid)
{
	if (!snr_q || !local_cap || !ssid) return false;
	uint64_t v = p38 & ((1ULL << 38) - 1ULL);
	*snr_q     = (uint8_t)((v >> 34) & 0xF);
	*local_cap = (uint8_t)((v >> 32) & 0x3);
	*ssid      = (uint8_t)((v >> 24) & 0xFF);
	return true;
}

// =============================================================================
// CRC-aided soft list decode (connect-suffix-fec-research.md §3 Tier 1)
// =============================================================================

void pack_ctrl_typed40_msb(unsigned char out_bytes[5], uint8_t type,
                           uint64_t payload38)
{
	uint64_t typed40 = ((uint64_t)(type & 0x3) << 38)
	                 | (payload38 & ((1ULL << 38) - 1ULL));
	for (int b = 0; b < 5; b++)
		out_bytes[b] = (unsigned char)((typed40 >> (8 * (4 - b))) & 0xFF);
}

// Reconstruct (type, payload38, crc12) from a full per-symbol tone assignment
// `tones[0..n-1]`. Bit-identical to cl_mfsk::unpack_ctrl_suffix (mfsk.cc:646):
// shift in bits_per_tone bits per tone MSB-first to build the 52-bit field
// [type:2|payload38:38|crc12:12].
static inline void unpack_assignment(const int* tones, int n, int bits_per_tone,
                                     uint8_t* out_type, uint64_t* out_payload38,
                                     uint16_t* out_crc12)
{
	uint64_t payload = 0;
	int mask = (1 << bits_per_tone) - 1;
	for (int g = 0; g < n; g++) {
		uint64_t t = (uint64_t)(tones[g] & mask);
		payload = (payload << bits_per_tone) | t;
	}
	*out_crc12     = (uint16_t)(payload & 0x0FFF);
	*out_payload38 = (uint64_t)((payload >> 12) & ((1ULL << 38) - 1ULL));
	*out_type      = (uint8_t)((payload >> 50) & 0x3);
}

// Best-first lattice search node: symbols 0..depth-1 are fixed to chosen[],
// remaining symbols default to their k=0 (argmax) candidate. `cost` is the
// cumulative soft cost of the deviations chosen so far. We expand by advancing
// the next undecided symbol through its K candidates. Exploring lowest-cost
// nodes first means the all-argmax assignment (cost 0) is checked first, so a
// clean channel reproduces the hard decode on the very first trial.
namespace {
struct LatticeNode {
	double cost;
	int depth;              // number of symbols already pinned in `choice`
	int flips;              // #symbols pinned to k>0 so far (Hamming dist)
	uint8_t choice[16];     // candidate index k per pinned symbol (n<=16)
	bool operator>(const LatticeNode& o) const { return cost > o.cost; }
};
}

bool soft_list_decode_ctrl_suffix(const int* cand, const double* cost,
                                  int n, int K, int bits_per_tone,
                                  uint8_t expected_type, int max_trials,
                                  int max_flips,
                                  ctrl_crc12_fn crc12_fn, void* crc12_ctx,
                                  uint64_t* out_payload38, int* out_flips)
{
	if (!cand || !cost || !crc12_fn || !out_payload38) return false;
	if (n <= 0 || n > 16 || K < 1) return false;

	// Every symbol must have at least its argmax candidate (k=0) valid; a
	// symbol that ran past the buffer end (cand=-1) makes the suffix
	// undecodable — bail rather than risk a spurious CRC hit on garbage.
	for (int s = 0; s < n; s++)
		if (cand[s * K + 0] < 0) return false;

	std::priority_queue<LatticeNode, std::vector<LatticeNode>,
	                    std::greater<LatticeNode> > pq;
	LatticeNode root; root.cost = 0.0; root.depth = 0; root.flips = 0;
	pq.push(root);

	int trials = 0;
	int tones[16];
	while (!pq.empty() && trials < max_trials)
	{
		LatticeNode node = pq.top(); pq.pop();

		if (node.depth == n) {
			// Full assignment — build tone vector, unpack, CRC-check.
			int flips = 0;
			for (int s = 0; s < n; s++) {
				int k = node.choice[s];
				tones[s] = cand[s * K + k];
				if (k != 0) flips++;
			}
			uint8_t type; uint64_t p38; uint16_t embedded_crc;
			unpack_assignment(tones, n, bits_per_tone, &type, &p38, &embedded_crc);
			trials++;
			if (type != expected_type) continue;  // wrong frame type
			unsigned char typed[5];
			pack_ctrl_typed40_msb(typed, type, p38);
			uint16_t calc = crc12_fn(crc12_ctx, typed, 5) & 0x0FFF;
			if (calc == embedded_crc) {
				*out_payload38 = p38;
				if (out_flips) *out_flips = flips;
				return true;
			}
			continue;
		}

		// Expand: pin symbol `depth` to each of its valid candidates.
		int s = node.depth;
		for (int k = 0; k < K; k++) {
			int tone = cand[s * K + k];
			if (tone < 0) break;                 // no more valid candidates
			double c = cost[s * K + k];
			if (c >= 1.0e299) break;             // invalid-slot sentinel
			int new_flips = node.flips + (k > 0 ? 1 : 0);
			if (max_flips >= 0 && new_flips > max_flips)
				continue;                        // outside the Hamming ball — prune
			LatticeNode child = node;
			child.cost  = node.cost + c;
			child.flips = new_flips;
			child.choice[s] = (uint8_t)k;
			child.depth = node.depth + 1;
			pq.push(child);
		}
	}
	return false;
}



// =============================================================================
// Tier-2 candidate A: soft GF(16) RA code (true degree-3 Q-ary RA, Q65 lineage)
// (fact-documents/tier2-suffix-fec-gf16-spike.md)
//
// Algorithm ported from qracodes (Nico Palermo IV3NWV, GPLv3 -> AGPLv3), and
// corrected to the TRUE RA structure after the first (degree-9) attempt was
// measured to correct < 1 symbol (§8): the real Q65 code (qra15_65_64_irr_e23)
// has MAXCDEG = 3 — every code check is degree 3 (prev parity, this parity, ONE
// interleaved info edge). The parity is a length-NC GF(16) ACCUMULATOR CHAIN,
// NC = repfact*K stages, each folding in one info replica chosen by an
// interleaver. Only this sparse degree-3 graph lets Q-ary BP correct several
// symbol errors.
//   - GF(16), additive group = XOR (the property the WHT check-conv needs).
//   - encode: y[K+j] = y[K+j-1] XOR alpha^wlog[j] * x[acc_idx[j]]  (RA chain).
//   - intrinsic: Bessel-I0 of the per-tone amplitudes (noncoherent-FSK-optimal).
//   - decode: symbol-domain BP, check node = IWHT(prod WHT(.)) (Walsh-Hadamard
//     fast GF convolution); GF weight = permutation of the pd.
//   - CRC carried as PROTECTED info (Q65 QRATYPE_CRC): recompute + compare =
//     FEC-reliable accept gate.
// =============================================================================

namespace gf16ra {

// ---- GF(16) field, primitive poly x^4 + x + 1 (0x13), alpha = 2 -------------
static int  g_gfexp[31];   // g_gfexp[i] = alpha^i (period 15)
static int  g_gflog[16];   // g_gflog[v] = log_alpha(v) (g_gflog[0] unused)
static bool g_field_inited = false;

// ---- RA graph (rebuilt by configure()) --------------------------------------
static int  g_repfact = 2;            // replicas per info symbol (default)
static int  g_NC = GF16RA_K * 2;      // parity symbols = repfact*K
static int  g_N  = GF16RA_K * 3;      // codeword length = K + NC
static bool g_graph_inited = false;
static int  g_built_repfact = -1;

// Accumulator interleaver + weights: stage j (0..NC-1) folds info symbol
// g_acc_idx[j] with GF weight alpha^g_acc_wlog[j]. Built so each of the K info
// symbols appears exactly g_repfact times, spread across the NC stages.
static int  g_acc_idx[GF16RA_MAX_N];
static int  g_acc_wlog[GF16RA_MAX_N];

// Code factor j (j=0..NC-1) incident variables, for the BP decoder:
//   var (K+j)          weight 1      [this parity]
//   var (K+j-1)        weight 1      [prev parity, absent for j=0]
//   var g_acc_idx[j]   weight alpha^g_acc_wlog[j]   [the one info edge]
struct FactorEdge { int var; int wlog; };
static std::vector<FactorEdge> g_factor_edges[GF16RA_MAX_N];

// ---- Walsh-Hadamard transform over 16 points (in place) ---------------------
static inline void fwht16(double* a)
{
	for (int len = 1; len < 16; len <<= 1)
		for (int i = 0; i < 16; i += (len << 1))
			for (int j = 0; j < len; j++) {
				double u = a[i + j], v = a[i + j + len];
				a[i + j] = u + v; a[i + j + len] = u - v;
			}
}

static inline void pd_normalize(double* p)
{
	double s = 0.0; for (int i = 0; i < 16; i++) s += p[i];
	if (s <= 0.0) { for (int i = 0; i < 16; i++) p[i] = 1.0 / 16.0; return; }
	double inv = 1.0 / s; for (int i = 0; i < 16; i++) p[i] *= inv;
}

// Forward weight: out[ alpha^wlog * x ] = in[x].
static inline void pd_mul_perm(double* out, const double* in, int wlog)
{
	if (wlog == 0) { for (int x = 0; x < 16; x++) out[x] = in[x]; return; }
	out[0] = in[0];
	for (int x = 1; x < 16; x++) out[g_gfexp[(g_gflog[x] + wlog) % 15]] = in[x];
}
// Backward weight: out[x] = in[ alpha^wlog * x ].
static inline void pd_div_perm(double* out, const double* in, int wlog)
{
	if (wlog == 0) { for (int x = 0; x < 16; x++) out[x] = in[x]; return; }
	out[0] = in[0];
	for (int x = 1; x < 16; x++) out[x] = in[g_gfexp[(g_gflog[x] + wlog) % 15]];
}

static void build_field()
{
	if (g_field_inited) return;
	int x = 1;
	for (int i = 0; i < 15; i++) { g_gfexp[i] = x; x <<= 1; if (x & 0x10) x ^= 0x13; }
	for (int i = 15; i < 31; i++) g_gfexp[i] = g_gfexp[i - 15];
	g_gflog[0] = 0;
	for (int i = 0; i < 15; i++) g_gflog[g_gfexp[i]] = i;
	g_field_inited = true;
}

int configure(int repfact)
{
	if (repfact < 1) repfact = 1;
	// cap so K + repfact*K <= GF16RA_MAX_N
	while (GF16RA_K + repfact * GF16RA_K > GF16RA_MAX_N && repfact > 1) repfact--;
	g_repfact = repfact;
	g_NC = repfact * GF16RA_K;
	g_N  = GF16RA_K + g_NC;
	g_graph_inited = false;   // force rebuild
	return g_N;
}

int codeword_len() { return g_N; }
int parity_len()   { return g_NC; }
int current_repfact() { return g_repfact; }

bool init()
{
	build_field();
	if (g_graph_inited && g_built_repfact == g_repfact) return true;

	// ---- Build the accumulator interleaver + weights ----
	// Stage assignment: lay the K*repfact replicas across the NC stages by a
	// spread interleaver. Replica r of info i goes to stage
	// (i + r*K) mod NC after a coprime scramble, guaranteeing each info appears
	// exactly repfact times and consecutive stages rarely repeat an info symbol
	// (good Tanner-graph girth). Each stage gets EXACTLY ONE info edge (degree-3
	// check). Weights cycle through alpha^1..alpha^14 (always nonzero); for each
	// info symbol force its replica weights to XOR-sum to 0 so the accumulator
	// terminates to 0 (the EXIT-convergence aid; the real CRC gate is the
	// correctness check).
	const int K = GF16RA_K;
	// position each replica deterministically; the stage list is then a
	// permutation of [0,NC). Use stride coprime with NC where possible.
	int stride = 1;
	for (int s = K; s >= 2; s--) {            // pick a stride coprime with NC
		int a = s, b = g_NC; while (b) { int t = a % b; a = b; b = t; }
		if (a == 1) { stride = s; break; }
	}
	// assign: enumerate (info, r) in a scrambled order and drop into stages
	// sequentially through the coprime stride so every stage is hit once.
	int order_pos = 0;
	int tmp_idx[GF16RA_MAX_N];
	int tmp_info_wlog[GF16RA_MAX_N];   // weight-log per stage
	for (int j = 0; j < g_NC; j++) { tmp_idx[j] = -1; }
	// per-info running weight accumulator to close XOR-to-0
	int info_wvals[GF16RA_MAX_N];      // last partial value per info (indexed 0..K-1)
	int info_count[GF16RA_K];
	for (int i = 0; i < K; i++) { info_count[i] = 0; info_wvals[i] = 0; }

	for (int i = 0; i < K; i++) {
		for (int r = 0; r < g_repfact; r++) {
			// find next free stage via the coprime stride walk
			int stage = (order_pos * stride) % g_NC;
			int guard = 0;
			while (tmp_idx[stage] != -1 && guard < g_NC) { order_pos++; stage = (order_pos * stride) % g_NC; guard++; }
			order_pos++;
			tmp_idx[stage] = i;
			// weight VALUE for this replica
			int wval;
			if (r < g_repfact - 1) {
				int wlog = 1 + ((i * g_repfact + r) % 14);   // 1..14, nonzero
				wval = g_gfexp[wlog];
				info_wvals[i] ^= wval;
			} else {
				// close: last weight = XOR of the others so the sum over the
				// info's replicas is 0 (accumulator termination). Avoid 0.
				wval = info_wvals[i];
				if (wval == 0) wval = 1;
			}
			tmp_info_wlog[stage] = g_gflog[wval];
			info_count[i]++;
		}
	}
	// Any stage that somehow stayed unassigned (shouldn't happen) -> give it a
	// benign self-consistent info edge (info 0, weight 1) to keep degree 3.
	for (int j = 0; j < g_NC; j++) {
		if (tmp_idx[j] == -1) { tmp_idx[j] = 0; tmp_info_wlog[j] = 0; }
		g_acc_idx[j]  = tmp_idx[j];
		g_acc_wlog[j] = tmp_info_wlog[j];
	}

	// ---- Compile into BP code-factor edge lists (degree 3) ----
	for (int j = 0; j < g_NC; j++) {
		g_factor_edges[j].clear();
		FactorEdge pe; pe.var = K + j; pe.wlog = 0; g_factor_edges[j].push_back(pe);   // this parity
		if (j > 0) { FactorEdge ppe; ppe.var = K + j - 1; ppe.wlog = 0; g_factor_edges[j].push_back(ppe); } // prev
		FactorEdge ie; ie.var = g_acc_idx[j]; ie.wlog = g_acc_wlog[j]; g_factor_edges[j].push_back(ie);     // info
	}

	g_graph_inited = true;
	g_built_repfact = g_repfact;
	return true;
}

static inline int gf_mul(int a, int b)
{
	if (a == 0 || b == 0) return 0;
	return g_gfexp[(g_gflog[a] + g_gflog[b]) % 15];
}

// 40-bit message + 12-bit CRC -> 13 systematic GF(16) info symbols (MSB-first).
static inline void msg_to_info(uint8_t type, uint64_t payload38, uint16_t crc12, int* info)
{
	uint64_t field40 = ((uint64_t)(type & 0x3) << 38) | (payload38 & ((1ULL << 38) - 1ULL));
	for (int s = 0; s < GF16RA_K_MSG; s++) info[s] = (int)((field40 >> (40 - 4 * (s + 1))) & 0xF);
	uint16_t c = (uint16_t)(crc12 & 0x0FFF);
	for (int s = 0; s < GF16RA_K_CRC; s++) info[GF16RA_K_MSG + s] = (int)((c >> (12 - 4 * (s + 1))) & 0xF);
}

static inline void info_to_msg(const int* info, uint8_t* type, uint64_t* payload38, uint16_t* crc12)
{
	uint64_t field40 = 0;
	for (int s = 0; s < GF16RA_K_MSG; s++) field40 = (field40 << 4) | (uint64_t)(info[s] & 0xF);
	*type = (uint8_t)((field40 >> 38) & 0x3);
	*payload38 = field40 & ((1ULL << 38) - 1ULL);
	uint16_t c = 0;
	for (int s = 0; s < GF16RA_K_CRC; s++) c = (uint16_t)((c << 4) | (info[GF16RA_K_MSG + s] & 0xF));
	*crc12 = (uint16_t)(c & 0x0FFF);
}

void encode(uint8_t type, uint64_t payload38, uint16_t crc12, int* out_tones)
{
	init();
	int info[GF16RA_K];
	msg_to_info(type, payload38, crc12, info);
	for (int s = 0; s < GF16RA_K; s++) out_tones[s] = info[s] & 0xF;
	// RA accumulator chain: one info edge folded per stage.
	int prev = 0;
	for (int j = 0; j < g_NC; j++) {
		int w = g_gfexp[g_acc_wlog[j]];
		int acc = prev ^ gf_mul(w, info[g_acc_idx[j]]);
		out_tones[GF16RA_K + j] = acc & 0xF;
		prev = acc;
	}
}

// log(I0(v)) rational approximation (qra_ioapprox).
static inline double log_i0_approx(double v)
{
	double vsq = v * v;
	double r = vsq * (v + 0.039) / (vsq * 0.9931 + v * 2.6936 + 0.5185);
	return (r > 80.0) ? 80.0 : r;
}

// Shared Q-ary BP core: decode the per-tone ENERGY matrix to the K info symbols.
// Identical math to the original soft_decode body (extracted verbatim so the
// legacy decoder stays bit-for-bit unchanged); the ONLY thing the two public
// entries do differently is the message reassemble + CRC/type gate (2-bit type
// for soft_decode vs 3-bit type for soft_decode_config_tag).
static void gf16ra_bp_to_info(const double* energies, int maxiter,
                              double esno_metric, int* info_out, int* out_iters)
{
	const int N = g_N, M = GF16RA_M, K = GF16RA_K, nfac = g_NC;

	// ---- intrinsic (channel) probabilities pix[s][16] (Bessel metric) ----
	std::vector<double> pix((size_t)N * M);
	{
		double rsum = 0.0;
		for (int i = 0; i < N * M; i++) rsum += energies[i];
		rsum /= (double)(N * M);
		double sigmaest = std::sqrt(rsum / (1.0 + esno_metric / M) / 2.0);
		if (!(sigmaest > 0.0)) sigmaest = 1e-6;
		double cmetric = std::sqrt(2.0 * esno_metric) / sigmaest;
		for (int s = 0; s < N; s++) {
			double* p = &pix[(size_t)s * M];
			for (int t = 0; t < M; t++)
				p[t] = std::exp(log_i0_approx(std::sqrt(energies[(size_t)s * M + t]) * cmetric));
			pd_normalize(p);
		}
	}

	// ---- BP message arrays (per factor-edge) ----
	std::vector<int> fac_off(nfac + 1, 0);
	for (int j = 0; j < nfac; j++) fac_off[j + 1] = fac_off[j] + (int)g_factor_edges[j].size();
	int total_edges = fac_off[nfac];
	std::vector<double> v2c((size_t)total_edges * M);
	std::vector<double> c2v((size_t)total_edges * M);

	for (int j = 0; j < nfac; j++)
		for (size_t k = 0; k < g_factor_edges[j].size(); k++) {
			int e = fac_off[j] + (int)k, v = g_factor_edges[j][k].var;
			for (int t = 0; t < M; t++) v2c[(size_t)e * M + t] = pix[(size_t)v * M + t];
		}

	double tmp[16], prod[16], permd[16];
	int rc = -1;
	std::vector<double> vprod((size_t)N * M);

	for (int nit = 0; nit < maxiter; nit++) {
		// ---- check -> variable ----
		for (int j = 0; j < nfac; j++) {
			int deg = (int)g_factor_edges[j].size();
			double wht[3 * 16];   // deg <= 3
			for (int k = 0; k < deg; k++) {
				int e = fac_off[j] + k;
				pd_mul_perm(tmp, &v2c[(size_t)e * M], g_factor_edges[j][k].wlog);
				for (int t = 0; t < M; t++) wht[k * M + t] = tmp[t];
				fwht16(&wht[k * M]);
			}
			for (int k = 0; k < deg; k++) {
				for (int t = 0; t < M; t++) prod[t] = 1.0;
				for (int kk = 0; kk < deg; kk++) if (kk != k)
					for (int t = 0; t < M; t++) prod[t] *= wht[kk * M + t];
				prod[0] += 1e-12;
				fwht16(prod);
				int e = fac_off[j] + k;
				pd_div_perm(permd, prod, g_factor_edges[j][k].wlog);
				pd_normalize(permd);
				for (int t = 0; t < M; t++) c2v[(size_t)e * M + t] = permd[t];
			}
		}

		// ---- variable -> check (and per-variable product) ----
		for (int v = 0; v < N; v++)
			for (int t = 0; t < M; t++) vprod[(size_t)v * M + t] = pix[(size_t)v * M + t];
		for (int j = 0; j < nfac; j++)
			for (size_t k = 0; k < g_factor_edges[j].size(); k++) {
				int e = fac_off[j] + (int)k, v = g_factor_edges[j][k].var;
				const double* m = &c2v[(size_t)e * M];
				double* vp = &vprod[(size_t)v * M];
				for (int t = 0; t < M; t++) vp[t] *= m[t];
			}
		for (int j = 0; j < nfac; j++)
			for (size_t k = 0; k < g_factor_edges[j].size(); k++) {
				int e = fac_off[j] + (int)k, v = g_factor_edges[j][k].var;
				const double* vp = &vprod[(size_t)v * M];
				const double* m  = &c2v[(size_t)e * M];
				double* out = &v2c[(size_t)e * M];
				for (int t = 0; t < M; t++) { double d = m[t]; out[t] = (d > 1e-300) ? vp[t] / d : vp[t]; }
				pd_normalize(out);
			}

		// ---- EXIT-chart convergence: sum of per-symbol max-marginal ----
		double totmax = 0.0;
		for (int v = 0; v < N; v++) {
			const double* vp = &vprod[(size_t)v * M];
			double mx = 0.0, s = 0.0;
			for (int t = 0; t < M; t++) { if (vp[t] > mx) mx = vp[t]; s += vp[t]; }
			if (s > 0.0) mx /= s;
			totmax += mx;
		}
		if (totmax > (double)N - 0.02) { rc = nit; break; }
	}
	if (out_iters) *out_iters = rc;

	// ---- MAP decode: APP = intrinsic * product(c->v) ----
	std::vector<double> app((size_t)N * M);
	for (int v = 0; v < N; v++)
		for (int t = 0; t < M; t++) app[(size_t)v * M + t] = pix[(size_t)v * M + t];
	for (int j = 0; j < nfac; j++)
		for (size_t k = 0; k < g_factor_edges[j].size(); k++) {
			int e = fac_off[j] + (int)k, v = g_factor_edges[j][k].var;
			const double* m = &c2v[(size_t)e * M];
			double* ap = &app[(size_t)v * M];
			for (int t = 0; t < M; t++) ap[t] *= m[t];
		}
	for (int s = 0; s < K; s++) {
		const double* ap = &app[(size_t)s * M];
		int best = 0; double bv = -1.0;
		for (int t = 0; t < M; t++) if (ap[t] > bv) { bv = ap[t]; best = t; }
		info_out[s] = best;
	}
}

bool soft_decode(const double* energies, int maxiter, double esno_metric,
                 uint8_t expected_type,
                 ctrl_crc12_fn crc12_fn, void* crc12_ctx,
                 uint64_t* out_payload38, int* out_iters)
{
	if (!energies || !crc12_fn || !out_payload38) return false;
	init();
	if (out_iters) *out_iters = -1;
	int info[GF16RA_K];
	gf16ra_bp_to_info(energies, maxiter, esno_metric, info, out_iters);

	// ---- reassemble + CRC/type accept gate (2-bit legacy type) ----
	uint8_t type; uint64_t p38; uint16_t dec_crc;
	info_to_msg(info, &type, &p38, &dec_crc);
	if (type != expected_type) return false;
	unsigned char typed[5];
	pack_ctrl_typed40_msb(typed, type, p38);
	uint16_t calc = crc12_fn(crc12_ctx, typed, 5) & 0x0FFF;
	if (calc != dec_crc) return false;
	*out_payload38 = p38;
	return true;
}

// -----------------------------------------------------------------------------
// CONFIG_TAG variant: 3-bit type / 37-bit payload message split (OD-1 widen
// scoped to the tag). The 40-bit field is [type:3|payload:37]; otherwise the
// RA codeword, the BP, and the CRC-as-protected-info are IDENTICAL to the legacy
// pair (encode/soft_decode) above.
static inline void config_tag_msg_to_info(uint8_t type, uint64_t payload37,
                                          uint16_t crc12, int* info)
{
	uint64_t field40 = ((uint64_t)(type & 0x7) << 37) | (payload37 & ((1ULL << 37) - 1ULL));
	for (int s = 0; s < GF16RA_K_MSG; s++) info[s] = (int)((field40 >> (40 - 4 * (s + 1))) & 0xF);
	uint16_t c = (uint16_t)(crc12 & 0x0FFF);
	for (int s = 0; s < GF16RA_K_CRC; s++) info[GF16RA_K_MSG + s] = (int)((c >> (12 - 4 * (s + 1))) & 0xF);
}

static inline void config_tag_info_to_msg(const int* info, uint8_t* type,
                                          uint64_t* payload37, uint16_t* crc12)
{
	uint64_t field40 = 0;
	for (int s = 0; s < GF16RA_K_MSG; s++) field40 = (field40 << 4) | (uint64_t)(info[s] & 0xF);
	*type = (uint8_t)((field40 >> 37) & 0x7);
	*payload37 = field40 & ((1ULL << 37) - 1ULL);
	uint16_t c = 0;
	for (int s = 0; s < GF16RA_K_CRC; s++) c = (uint16_t)((c << 4) | (info[GF16RA_K_MSG + s] & 0xF));
	*crc12 = (uint16_t)(c & 0x0FFF);
}

void encode_config_tag(uint8_t type, uint64_t payload37, uint16_t crc12, int* out_tones)
{
	init();
	int info[GF16RA_K];
	config_tag_msg_to_info(type, payload37, crc12, info);
	for (int s = 0; s < GF16RA_K; s++) out_tones[s] = info[s] & 0xF;
	// RA accumulator chain: one info edge folded per stage (identical to encode()).
	int prev = 0;
	for (int j = 0; j < g_NC; j++) {
		int w = g_gfexp[g_acc_wlog[j]];
		int acc = prev ^ gf_mul(w, info[g_acc_idx[j]]);
		out_tones[GF16RA_K + j] = acc & 0xF;
		prev = acc;
	}
}

bool soft_decode_config_tag(const double* energies, int maxiter, double esno_metric,
                            uint8_t expected_type,
                            ctrl_crc12_fn crc12_fn, void* crc12_ctx,
                            uint64_t* out_payload37, int* out_iters)
{
	if (!energies || !crc12_fn || !out_payload37) return false;
	init();
	if (out_iters) *out_iters = -1;
	int info[GF16RA_K];
	gf16ra_bp_to_info(energies, maxiter, esno_metric, info, out_iters);

	// ---- reassemble + CRC/type accept gate (3-bit config-tag type) ----
	uint8_t type; uint64_t p37; uint16_t dec_crc;
	config_tag_info_to_msg(info, &type, &p37, &dec_crc);
	if (type != expected_type) return false;
	unsigned char typed[5];
	pack_config_tag_typed40_msb(typed, type, p37);
	uint16_t calc = crc12_fn(crc12_ctx, typed, 5) & 0x0FFF;
	if (calc != dec_crc) return false;
	*out_payload37 = p37;
	return true;
}

} // namespace gf16ra

// =============================================================================
// CONFIG_TAG codec (OD-1/OD-2) — RM(1,4) Walsh codeword + WRAP detector
// =============================================================================
//
// DESIGN DECISION (TRACKED — fact-documents/ is gitignored, so the rationale
// lives here so it travels with the source). tag-codeword-design.md §1.3.
//
// RM-chip <-> M=16-tone front-end = OPTION (a), the tone-PERMUTATION realization
// (the design note's RECOMMENDED choice), NOT the original Stage-1 option (b)
// 2-tone {0,8} binary-FSK overlay.
//
//   Option (b) (REPLACED): every one of the 16 RM chips rode the SAME two tones
//   (CFG_TAG_TONE_PLUS=0 / CFG_TAG_TONE_MINUS=8); the soft chip was e[i][0]-e[i][8].
//   That throws away the M=16 non-coherent gain (note §1.3): a frequency-selective
//   null or single-tone interferer on tone 0 or 8 corrupts ALL 16 chips at once,
//   so the FWHT floors exactly where the 32-MFSK robust ACK still decodes. Because
//   the WRAP requires fwht_passed && crc_passed && bind_agree, a floored FWHT CAPS
//   the tag's survival SNR below the robust substrate it is supposed to out-live.
//
//   Option (a) (THIS IMPL): chip i occupies its own M=16 symbol on an antipodal
//   tone PAIR { perm[i], perm[i]^0xF } that is UNIQUE to position i (CFG_TAG_TONE_PERM
//   is a permutation of {0..15}). A clean codeword and its bi-orthogonal complement
//   together exercise the FULL 16-tone alphabet. A per-tone fade now damages only
//   the few chips that ride the faded tones; the 16-pt FWHT (d_min=8) integrates the
//   surviving chips and still recovers cfg_index. The soft chip is the SAME-symbol
//   antipodal energy difference e[i][perm[i]]-e[i][perm[i]^0xF], read from the
//   per-tone energy matrix the gf16ra RA decoder also consumes (so the FWHT and the
//   CRC-field substrate corroborate over the same robust energies). The DECODER is
//   unchanged: one 16-pt FWHT over the 16 soft chips, argmax|bin| -> row, sign ->
//   complement. Only the chip<->tone mapping moved from a fixed pair to a permutation.
//   The §24 sweep (test T4) MEASURES (a) vs (b) detection-vs-Es/N0 on AWGN and on a
//   per-tone (frequency-selective) fade and is the decisive evidence for this choice.
//
// OD-1 SCOPED-WIDEN rationale (unchanged): the 2->3-bit type widen that admits
// CONFIG_TAG=4 is scoped to this tag's OWN pack/unpack helpers
// (pack_config_tag_typed40_msb + the gf16ra config-tag layout) rather than globally
// repartitioning the shared 52-bit suffix [type:2|payload:38]. A global repartition
// would drop payload bit 37 (load-bearing for every legacy type and asserted by green
// unit tests) and silently change the production ACK wire. The four legacy types stay
// byte-identical; the tag is the ONLY codec that reads/writes a 3-bit type. See
// fact-documents/data-flow-config-tag-codec.md §2 (the cost-of-the-global-repartition
// measurement). [?] OD-1 owner ratification carries to Stage-2 (the ARQ adopt wiring).

void pack_config_tag_typed40_msb(unsigned char out_bytes[5], uint8_t type,
                                 uint64_t payload37)
{
	uint64_t typed40 = ((uint64_t)(type & 0x7) << 37)
	                 | (payload37 & ((1ULL << 37) - 1ULL));
	for (int b = 0; b < 5; b++)
		out_bytes[b] = (unsigned char)((typed40 >> (8 * (4 - b))) & 0xFF);
}

void pack_config_tag_payload(uint64_t* p37, uint8_t cfg_index,
                             uint8_t batch_seq_lsb, uint8_t epoch_parity)
{
	if (!p37) return;
	uint64_t v = 0;
	v |= ((uint64_t)(cfg_index     & 0x1F)) << 32;  // bits 36..32
	v |= ((uint64_t)(batch_seq_lsb & 0x07)) << 29;  // bits 31..29
	v |= ((uint64_t)(epoch_parity  & 0x01)) << 28;  // bit  28
	// reserved (bits 27..0) MUST be zero on TX
	*p37 = v & ((1ULL << 37) - 1ULL);
}

bool unpack_config_tag_payload(uint64_t p37, uint8_t* cfg_index,
                               uint8_t* batch_seq_lsb, uint8_t* epoch_parity)
{
	if (!cfg_index || !batch_seq_lsb || !epoch_parity) return false;
	uint64_t v = p37 & ((1ULL << 37) - 1ULL);
	*cfg_index     = (uint8_t)((v >> 32) & 0x1F);
	*batch_seq_lsb = (uint8_t)((v >> 29) & 0x07);
	*epoch_parity  = (uint8_t)((v >> 28) & 0x01);
	return true;
}

// -----------------------------------------------------------------------------
// RM(1,4) = (16,5,8) bi-orthogonal Walsh/Hadamard codeword for cfg_index.
//
// The 16 Sylvester-Hadamard rows are H[r][i] = (-1)^popcount(r & i) (i,r in
// [0,16)). cfg_index (5 bits) maps to (row:4, complement:1): row = cfg_index&0xF,
// complement = (cfg_index>>4)&1. The 16 chips are H[row][.] negated iff
// complement. Decode: a 16-pt FWHT of the soft chips yields, at bin = row, a
// value whose magnitude is the correlation peak and whose sign is the complement
// bit (+ for non-complemented, - for complemented). The Sylvester-Hadamard
// transform IS the FWHT (same butterfly the gf16ra fwht16 uses), so the row that
// produced the chips lights up exactly one bin.
// -----------------------------------------------------------------------------

static inline int rm_popcount4(int x) {
	x &= 0xF; return (x & 1) + ((x >> 1) & 1) + ((x >> 2) & 1) + ((x >> 3) & 1);
}

bool cfg_tag_rm_encode(int cfg_index, int* out_chips)
{
	if (!out_chips || cfg_index < 0 || cfg_index >= 32) return false;
	int row = cfg_index & 0xF;
	int comp = (cfg_index >> 4) & 0x1;
	for (int i = 0; i < 16; i++) {
		int h = (rm_popcount4(row & i) & 1) ? -1 : +1;  // Hadamard row entry
		out_chips[i] = comp ? -h : h;
	}
	return true;
}

int cfg_tag_rm_fwht_decode(const double* soft_chips, double* out_rpeak)
{
#ifdef STAGE1_FAILBEFORE
	// FAIL-BEFORE stub: the real FWHT decode does not exist yet. Return a fixed
	// WRONG cfg_index with a saturated peak ratio (so the peak-margin gate passes
	// but the index is wrong → T1 roundtrip mismatch, T2 no recovery, and the
	// WRAP corroboration cannot agree). This is the genuine "decode absent" state
	// the Stage-1 fail-before must demonstrate.
	(void)soft_chips;
	if (out_rpeak) *out_rpeak = 1e9;
	return 7;  // fixed wrong index
#else
	// Walsh-Hadamard transform (natural order) of the 16 soft chips. The
	// Sylvester-Hadamard rows are the WHT basis, so a clean codeword puts all
	// energy in bin = row, signed by the complement bit.
	double a[16];
	for (int i = 0; i < 16; i++) a[i] = soft_chips ? soft_chips[i] : 0.0;
	for (int len = 1; len < 16; len <<= 1)
		for (int i = 0; i < 16; i += (len << 1))
			for (int j = 0; j < len; j++) {
				double u = a[i + j], v = a[i + j + len];
				a[i + j] = u + v; a[i + j + len] = u - v;
			}
	int best = 0; double bestmag = -1.0, secondmag = 0.0;
	for (int r = 0; r < 16; r++) {
		double m = std::fabs(a[r]);
		if (m > bestmag) { secondmag = bestmag; bestmag = m; best = r; }
		else if (m > secondmag) { secondmag = m; }
	}
	int comp = (a[best] < 0.0) ? 1 : 0;
	if (out_rpeak) *out_rpeak = (secondmag > 1e-12) ? (bestmag / secondmag) : 1e9;
	return (comp << 4) | best;
#endif
}

// OPTION (a): each chip's soft value is the antipodal energy difference of THAT
// symbol's UNIQUE tone pair { perm[i], perm[i]^0xF } (vs option (b)'s fixed {0,8}).
// Reads the same per-tone energy matrix the gf16ra RA decoder consumes.
void cfg_tag_softchips_from_energies(const double* energies16x16, double* out_chips)
{
	if (!energies16x16 || !out_chips) return;
	for (int i = 0; i < 16; i++) {
		int tplus  = CFG_TAG_TONE_PERM[i] & 0xF;
		int tminus = tplus ^ 0xF;
		out_chips[i] = energies16x16[i * 16 + tplus]
		             - energies16x16[i * 16 + tminus];
	}
}

// OPTION (a): per-symbol one-hot on the antipodal tone chip i selects
// (perm[i] for +1, perm[i]^0xF for -1). Across the 16 chips a codeword and its
// complement span the full 16-tone alphabet (per-tone / frequency diversity).
void cfg_tag_energies_from_cfg(int cfg_index, double hi, double lo, double* out_e16x16)
{
	if (!out_e16x16) return;
	int chips[16];
	cfg_tag_rm_encode(cfg_index, chips);
	for (int i = 0; i < 16; i++) {
		for (int t = 0; t < 16; t++) out_e16x16[i * 16 + t] = lo;
		int tplus = CFG_TAG_TONE_PERM[i] & 0xF;
		int tone  = (chips[i] > 0) ? tplus : (tplus ^ 0xF);
		out_e16x16[i * 16 + tone] = hi;
	}
}

// -----------------------------------------------------------------------------
// WRAP detector (FWHT primary + GF(16)/CRC-12 secondary + binding gates).
// -----------------------------------------------------------------------------

// BP iteration cap + assumed Es/No for the Bessel intrinsic (a fixed design
// point, matching the test-harness constants for the legacy gf16ra path).
static const int    CFG_TAG_BP_MAXITER = 50;
static const double CFG_TAG_ESNO_METRIC = 4.0;  // ~6 dB

bool config_tag_wrap_decode(const double* energies,
                            const double* chip_soft,
                            double peak_ratio_gate,
                            uint8_t expect_bsi_lsb, uint8_t expect_parity,
                            ctrl_crc12_fn crc12_fn, void* crc12_ctx,
                            config_tag_decode_result* out)
{
	config_tag_decode_result r;
	r.cfg_index = 0; r.batch_seq_lsb = 0; r.epoch_parity = 0;
	r.fwht_rpeak = 0.0; r.fwht_passed = false; r.crc_passed = false; r.bind_agree = false;
	if (out) *out = r;
	if (!energies || !chip_soft || !crc12_fn) return false;

	// Gate-1: FWHT correlator over the 16 soft chips.
	double rpeak = 0.0;
	int fwht_cfg = cfg_tag_rm_fwht_decode(chip_soft, &rpeak);
	r.fwht_rpeak = rpeak;
	r.fwht_passed = (rpeak >= peak_ratio_gate);

	// Gate-2: GF(16) RA + CRC-12 over the 3-bit-typed field. The CONFIG_TAG rides
	// the configure(2)=N=39 substrate; ensure the process-global gf16ra graph is at
	// repfact=2 for THIS decode and restore the prior value (the legacy CONNECT FEC
	// runs at configure(3)=N=52 — this decode must neither depend on nor corrupt it;
	// CLAUDE.md §5 cross-layer guard). Self-contained so any caller is safe.
	int saved_repfact = gf16ra::current_repfact();
	if(saved_repfact != 2) { gf16ra::configure(2); gf16ra::init(); }
	uint64_t p37 = 0; int iters = -2;
	bool gf_ok = gf16ra::soft_decode_config_tag(energies, CFG_TAG_BP_MAXITER,
		CFG_TAG_ESNO_METRIC, (uint8_t)MFSK_CTRL_CONFIG_TAG,
		crc12_fn, crc12_ctx, &p37, &iters);
	if(saved_repfact != 2) { gf16ra::configure(saved_repfact); gf16ra::init(); }
	r.crc_passed = gf_ok;

	uint8_t crc_cfg = 0, bsi_lsb = 0, parity = 0;
	if (gf_ok) {
		unpack_config_tag_payload(p37, &crc_cfg, &bsi_lsb, &parity);
		r.cfg_index = crc_cfg; r.batch_seq_lsb = bsi_lsb; r.epoch_parity = parity;
	} else {
		// Without the CRC field we still surface the FWHT estimate for logging.
		r.cfg_index = (uint8_t)fwht_cfg;
	}

	// Gate-3/4/5: cfg_index corroboration (FWHT == CRC field) + bsi + parity.
	// expect_bsi_lsb == 0xFF and expect_parity == 0xFF are the "do not bind on this
	// field" sentinels (Stage 3d §15): the PRE-FRAME tag detect runs BEFORE frame 0
	// decodes, so the RX does not yet know the batch bsi to bind against — it relies on
	// the FWHT peak + GF(16)+CRC-12 + cfg_index corroboration (the ~1e-8 FAR gates).
	// No existing caller passes 0xFF for bsi (they pass a real 0..7), so the sentinel is
	// purely additive (byte-identical for them).
	bool corroborate = gf_ok && (fwht_cfg == (int)crc_cfg);
	bool bsi_ok      = gf_ok && (expect_bsi_lsb == 0xFF || bsi_lsb == (uint8_t)(expect_bsi_lsb & 0x7));
	bool parity_ok   = gf_ok && (expect_parity == 0xFF || parity == (uint8_t)(expect_parity & 0x1));
	r.bind_agree = corroborate && bsi_ok && parity_ok;

	// WRAP: accept ONLY on ALL gates.
	bool accept = r.fwht_passed && r.crc_passed && r.bind_agree;
	if (out) *out = r;
	return accept;
}

// =============================================================================
// Stage 4e (D2 NACK first-class) — NACK payload codec + WRAP decode
// =============================================================================
//
// The NACK rides the SAME RM(1,4)+gf16ra+CRC-12 substrate as the CONFIG_TAG (only the
// type discriminator [5 vs 4] and the payload field layout differ), so encode/decode
// reuse gf16ra::encode_config_tag / soft_decode_config_tag verbatim (type-parameterized).

void pack_nack_payload(uint64_t* p37, uint8_t rx_cfg_index,
                       uint8_t rx_expected_bsi_lsb, uint8_t reason,
                       uint8_t epoch_parity)
{
	if (!p37) return;
	uint64_t v = 0;
	v |= ((uint64_t)(rx_cfg_index        & 0x1F)) << 32;  // bits 36..32
	v |= ((uint64_t)(rx_expected_bsi_lsb & 0x07)) << 29;  // bits 31..29
	v |= ((uint64_t)(reason              & 0x03)) << 27;  // bits 28..27
	v |= ((uint64_t)(epoch_parity        & 0x01)) << 26;  // bit  26
	// reserved (bits 25..0) MUST be zero on TX
	*p37 = v & ((1ULL << 37) - 1ULL);
}

bool unpack_nack_payload(uint64_t p37, uint8_t* rx_cfg_index,
                         uint8_t* rx_expected_bsi_lsb, uint8_t* reason,
                         uint8_t* epoch_parity)
{
	if (!rx_cfg_index || !rx_expected_bsi_lsb || !reason || !epoch_parity) return false;
	uint64_t v = p37 & ((1ULL << 37) - 1ULL);
	*rx_cfg_index        = (uint8_t)((v >> 32) & 0x1F);
	*rx_expected_bsi_lsb = (uint8_t)((v >> 29) & 0x07);
	*reason              = (uint8_t)((v >> 27) & 0x03);
	*epoch_parity        = (uint8_t)((v >> 26) & 0x01);
	return true;
}

bool nack_wrap_decode(const double* energies,
                      const double* chip_soft,
                      double peak_ratio_gate,
                      ctrl_crc12_fn crc12_fn, void* crc12_ctx,
                      nack_decode_result* out)
{
	nack_decode_result r;
	r.rx_cfg_index = 0; r.rx_expected_bsi_lsb = 0; r.reason = 0; r.epoch_parity = 0;
	r.fwht_rpeak = 0.0; r.fwht_passed = false; r.crc_passed = false; r.cfg_corroborate = false;
	if (out) *out = r;
	if (!energies || !chip_soft || !crc12_fn) return false;

	// Gate-1: FWHT correlator over the 16 RM soft chips (the rx_cfg_index codeword).
	double rpeak = 0.0;
	int fwht_cfg = cfg_tag_rm_fwht_decode(chip_soft, &rpeak);
	r.fwht_rpeak = rpeak;
	r.fwht_passed = (rpeak >= peak_ratio_gate);

	// Gate-2: GF(16) RA + CRC-12 over the 3-bit-typed field, type=MFSK_CTRL_NACK.
	// Same configure(2)=N=39 substrate + save/restore as config_tag_wrap_decode.
	int saved_repfact = gf16ra::current_repfact();
	if (saved_repfact != 2) { gf16ra::configure(2); gf16ra::init(); }
	uint64_t p37 = 0; int iters = -2;
	bool gf_ok = gf16ra::soft_decode_config_tag(energies, CFG_TAG_BP_MAXITER,
		CFG_TAG_ESNO_METRIC, (uint8_t)MFSK_CTRL_NACK,
		crc12_fn, crc12_ctx, &p37, &iters);
	if (saved_repfact != 2) { gf16ra::configure(saved_repfact); gf16ra::init(); }
	r.crc_passed = gf_ok;

	uint8_t crc_cfg = 0, bsi_lsb = 0, reason = 0, parity = 0;
	if (gf_ok) {
		unpack_nack_payload(p37, &crc_cfg, &bsi_lsb, &reason, &parity);
		r.rx_cfg_index = crc_cfg; r.rx_expected_bsi_lsb = bsi_lsb;
		r.reason = reason; r.epoch_parity = parity;
	} else {
		r.rx_cfg_index = (uint8_t)fwht_cfg;   // surface the FWHT estimate for logging
	}

	// Gate-3: cfg_index corroboration (FWHT == CRC field). The sender applies its own
	// parity/in-window policy in inband_handle_nack — kept out of the codec.
	r.cfg_corroborate = gf_ok && (fwht_cfg == (int)crc_cfg);

	bool accept = r.fwht_passed && r.crc_passed && r.cfg_corroborate;
	if (out) *out = r;
	return accept;
}
