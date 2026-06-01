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
//   bits 37..36 : echoed_cap     (2)
//   bits 35..34 : own_cap        (2)
//   bits 33..26 : ssid           (8)
//   bits 25..0  : reserved       (26)
//

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
//   bits 33..32 : local_cap      (2)
//   bits 31..24 : ssid           (8)
//   bits 23..0  : reserved       (24)
//

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

bool soft_decode(const double* energies, int maxiter, double esno_metric,
                 uint8_t expected_type,
                 ctrl_crc12_fn crc12_fn, void* crc12_ctx,
                 uint64_t* out_payload38, int* out_iters)
{
	if (!energies || !crc12_fn || !out_payload38) return false;
	init();
	if (out_iters) *out_iters = -1;
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
	int info[GF16RA_K];
	for (int s = 0; s < K; s++) {
		const double* ap = &app[(size_t)s * M];
		int best = 0; double bv = -1.0;
		for (int t = 0; t < M; t++) if (ap[t] > bv) { bv = ap[t]; best = t; }
		info[s] = best;
	}

	// ---- reassemble + CRC/type accept gate ----
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

} // namespace gf16ra
