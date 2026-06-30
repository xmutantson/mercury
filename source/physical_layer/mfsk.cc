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

#include "physical_layer/mfsk.h"
#include "physical_layer/ldpc.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>   // Stage 3c: getenv/atoi for MERCURY_INBAND_TAG_SYNC_REPS

cl_mfsk::cl_mfsk()
{
	M = 0;
	nBits = 0;
	Nc = 0;
	nStreams = 0;
	tone_hop_step = 0;
	preamble_nSymb = 0;
	for (int i = 0; i < MAX_STREAMS; i++)
		stream_offsets[i] = 0;
	for (int i = 0; i < MAX_PREAMBLE_SYMB; i++)
		preamble_tones[i] = 0;
	for (int i = 0; i < MAX_ACK_TONES; i++)
		ack_tones[i] = 0;
	for (int i = 0; i < MAX_ACK_TONES; i++)
		break_tones[i] = 0;
	for (int i = 0; i < MAX_ACK_TONES; i++)
		connect_tones[i] = 0;
	connect_pattern_nsymb = 0;
	connect_match_threshold = 0;
	for (int i = 0; i < HAIL_SUFFIX_LEN; i++)
		hail_suffix[i] = 0;
	for (int i = 0; i < MAX_ACK_TONES + HAIL_SUFFIX_LEN; i++)
		hail_detect_tones[i] = 0;
	hail_directed = false;
	hail_detect_nsymb = 0;
	hail_detect_threshold = 0;
	ack_pattern_len = 0;
	ack_pattern_nsymb = 0;
	ack_match_threshold = 0;
	break_match_threshold = 0;
	hail_match_threshold = 0;
	wb_match_threshold_bias = 0;  // Phase-2 flag default = HEAD
	for (int i = 0; i < MAX_ACK_SACK_SUFFIX; i++)
		last_ack_sack_suffix_tones[i] = -1;
	last_ack_sack_capture_valid = false;
	for (int i = 0; i < MAX_ACK_SACK_SUFFIX; i++)
		last_connect_suffix_tones[i] = -1;
	last_connect_capture_valid = false;
	last_demod_snr_db = -99.0;  // "no measurement" sentinel until first demod()
	suffix_fec_coded = false;  // Tier-2 FEC off by default (§19) — CONNECT suffix
	ack_suffix_fec_coded = false; // §21: ACK-suffix FEC off by default (separate
	                              // from the CONNECT flag; held off this increment)
	ack_suffix_fec_N = 0;         // REVSACK Part A: captured coded N (0 = off / uncoded 13)
	connect_preamble_reps = 1; // Tier-2 base-pattern combining off by default (§20)
	recovery_ack_reps = 1;     // RECOVERY-ACK robustness off by default
	                           // (recovery-ack-robustness.md §4) → byte-identical
}

cl_mfsk::~cl_mfsk()
{
	deinit();
}

void cl_mfsk::init(int _M, int _Nc, int _nStreams)
{
	M = _M;
	Nc = _Nc;
	nStreams = _nStreams;
	if (nStreams < 1) nStreams = 1;
	if (nStreams > MAX_STREAMS) nStreams = MAX_STREAMS;

	for (int i = 0; i < MAX_ACK_SACK_SUFFIX; i++)
		last_ack_sack_suffix_tones[i] = -1;
	last_ack_sack_capture_valid = false;
	for (int i = 0; i < MAX_ACK_SACK_SUFFIX; i++)
		last_connect_suffix_tones[i] = -1;
	last_connect_capture_valid = false;
	// NOTE: suffix_fec_coded is NOT reset here — it is owned by the telecom
	// layer (set after gf16ra::configure(3)+init() when FEC is enabled) and
	// init() is called once at load_configuration before that. Resetting it
	// here would clobber a FEC-enable that ran first. Constructor sets it false.

	// Calculate log2(M)
	nBits = 0;
	int temp = M;
	while (temp > 1)
	{
		nBits++;
		temp >>= 1;
	}

	// Tone hop step: must be coprime with M for full-period cycling
	if (M == 32)
		tone_hop_step = 13;  // 13 is prime, coprime with 32
	else if (M == 16)
		tone_hop_step = 7;   // 7 is prime, coprime with 16
	else if (M == 8)
		tone_hop_step = 3;   // 3 is prime, coprime with 8 (narrowband)
	else if (M == 4)
		tone_hop_step = 1;   // coprime with 4 (narrowband 2-stream)
	else
		tone_hop_step = 1;

	// Stream frequency allocation: center all streams within Nc subcarriers
	// Each stream gets M contiguous bins, streams are adjacent
	int total_bins = nStreams * M;
	int global_offset = (Nc - total_bins) / 2;
	if (global_offset < 0) global_offset = 0;
	for (int k = 0; k < nStreams; k++)
		stream_offsets[k] = global_offset + k * M;

	// MFSK preamble: known tone sequence spread across each stream's band.
	// Same tone index used in all streams simultaneously.
	// NB (M<=8): 8-symbol preamble for cross-correlation detection.
	// WB (M>=16): 16-symbol preamble (raised from 4 on 2026-05-27 per
	// data-frame-cliff-audit-2026-05-27.md §H1) for +6 dB matched-filter
	// integration gain at the WGN:-8 cliff.
	//
	// 2026-05-27 (data-preamble-port-research.md §14): WB preamble tones
	// are now a Welch-Costas sequence with primitive root g=2 (mod 17),
	// distinct from ACK (g=5), BREAK (g=7), HAIL (g=6), CONNECT (g=3).
	// 8 base tones × 2 reps fill the 16-symbol preamble. The data-preamble
	// TX path (cl_mfsk::generate_preamble, mfsk.cc:467) reads
	// preamble_tones[s % preamble_nSymb] DIRECTLY with no tone-hopping at
	// emit time, so the array stores the full 16-symbol sequence here.
	// The new RX detector (time_sync_mfsk_corr discrete-match port,
	// ofdm.cc) reads the same array — symmetric.
	//
	// Cross-correlation against existing Welch-Costas patterns
	// (8-base-tone Hamming distance): vs ACK 7/8, vs BREAK 8/8, vs HAIL
	// 8/8, vs CONNECT 8/8 at both M=16 and M=32. All ≥ 6/8, the bar
	// enforced by test_base_pattern_cross_correlation
	// (mfsk_ctrl_codec_tests.cc:409). Expanded 16-symbol sequences
	// ≥ 14/16 — well above the 7/16 detector threshold.
	if (M == 32)
	{
		preamble_nSymb = 16;
		// 2× scaled M=16 Welch-Costas g=2 sequence:
		// (2^k mod 17 for k=1..8) × 2 mod 32, with the trailing 16→0
		// substitution carried through: {4, 8, 16, 0, 30, 26, 18, 2}.
		const int base[8] = {4, 8, 16, 0, 30, 26, 18, 2};
		for (int s = 0; s < 16; s++)
			preamble_tones[s] = base[s % 8];
	}
	else if (M == 16)
	{
		preamble_nSymb = 16;
		// Welch-Costas (p=17, g=2): 2^k mod 17, k=1..8 = {2,4,8,16,15,13,9,1}.
		// Trailing 16 is out-of-range for M=16; substituted to 0 (free of
		// CONNECT/ACK/BREAK/HAIL base-tone collisions at any index).
		const int base[8] = {2, 4, 8, 0, 15, 13, 9, 1};
		for (int s = 0; s < 16; s++)
			preamble_tones[s] = base[s % 8];
	}
	else if (M == 8)
	{
		// Narrowband: 8 symbols, all tones used once
		preamble_nSymb = 8;
		preamble_tones[0] = 1;
		preamble_tones[1] = 5;
		preamble_tones[2] = 3;
		preamble_tones[3] = 7;
		preamble_tones[4] = 0;
		preamble_tones[5] = 6;
		preamble_tones[6] = 2;
		preamble_tones[7] = 4;
	}
	else if (M == 4)
	{
		// Narrowband 2-stream: 8 symbols, palindrome for symmetry
		preamble_nSymb = 8;
		preamble_tones[0] = 0;
		preamble_tones[1] = 2;
		preamble_tones[2] = 1;
		preamble_tones[3] = 3;
		preamble_tones[4] = 3;
		preamble_tones[5] = 1;
		preamble_tones[6] = 2;
		preamble_tones[7] = 0;
	}
	else
	{
		preamble_nSymb = 4;
		// Generic: spread evenly
		for (int i = 0; i < preamble_nSymb && i < MAX_PREAMBLE_SYMB; i++)
			preamble_tones[i] = (i * M / preamble_nSymb + M / (2 * preamble_nSymb)) % M;
	}

	// Discrete-match preamble detection threshold (post-2026-05-27 port,
	// fact-documents/data-preamble-port-research.md §14). Required count
	// of per-symbol FFT-bin-argmax matches for the discrete-tone-match
	// detector in cl_ofdm::time_sync_mfsk_corr to declare detection.
	//
	// Detector accepts expected OR mirror (Bug #39 carrier-image recovery)
	// so the random-data per-symbol false-match baseline is p≈2/M, NOT 1/M.
	//
	// THRESHOLD IS GATED ON nStreams to match the detector's two decision
	// paths (ofdm.cc time_sync_mfsk_corr, productionized 2026-06-02):
	//
	//  nStreams == 1 (M32×1 ROBUST_0 / M8×1 NB-ROBUST_0): the legacy
	//    per-stream argmax + bin-mirror accept path. T=7. Binomial(N,2/M)
	//    FAR model (unchanged, byte-identical to the pre-combiner detector):
	//      WB M=32 N=16 → T=7 : FAR = 2.57e-5/poll
	//      NB M=8  N=8  → T=7 : FAR = 3.82e-4/poll
	//
	//  nStreams >= 2 (M16×2 ROBUST_1/2 / M4×2 NB): STREAM-ENERGY COMBINING
	//    replaces the old 2-stream AND-gate (which had been the FAR
	//    mitigation for the degenerate-mirror cases). Combining recovers the
	//    redundant per-stream preamble tone for +6 dB acquisition reach
	//    (M16×2 production data-frame cliff −9.03 → −15.05 dB SNR3k @T=7;
	//    P3 data-frame-detector-deepening-p3.md §11) but raises FAR: the
	//    Binomial(N,2/M) model no longer applies (combining + the Phase-2
	//    fine-offset max over sub-positions inflate it). MEASURED on the
	//    production detector (P3 sweep, 4000 pure-noise polls):
	//      WB M16×2 : T=7 FAR 9.3e-2 (cliff −15.05) | T=8 FAR 1.8e-2 (cliff −13.89)
	//    T=8 is chosen for nStreams>=2: it keeps the FULL end-to-end win
	//    (the data FEC binds at −10.84 dB, ~3 dB ABOVE the −13.89 T=8
	//    detector cliff, so −15.05→−13.89 costs nothing end-to-end) while
	//    cutting FAR ~5×. The residual 1.8e-2 is CRC16-backstopped (a
	//    spurious preamble → LDPC on noise → CRC16 reject → ~30 ms wasted,
	//    not corruption). [OPEN] the fine-pass FAR inflation is a separate
	//    architectural item (gate the detect decision on the COARSE matched
	//    count, refine offset only) — tracked in the P3 fact-doc §11/§13.
	if (nStreams >= 2)
		preamble_match_threshold = 8;
	else
		preamble_match_threshold = 7;

	// ACK pattern tones.
	// WB (M>=16): Welch-Costas array (p=17, g=5), 8 base tones × 2 reps = 16 symbols.
	//   Costas property: all pairwise (dt, df) difference vectors are unique.
	//   Tone hopping provides additional frequency diversity.
	// NB (M<=8): Sidelnikov sequences — longer patterns (32/48 symbols) with optimal
	//   Hamming autocorrelation. No repetition, no tone hopping (diversity is intrinsic).
	//   Needed because small M makes short Costas arrays vulnerable to false detection
	//   (P(false/poll) = 2.7% for M=8, 50% for M=4 with 16-symbol patterns).
	if (M == 32)
	{
		// 2x scaled M=16 Costas values. Avoids preamble {4,20,12,28}.
		ack_pattern_len = 8;
		ack_pattern_nsymb = 16; // 8 × 2 reps
		ack_match_threshold = 7; // 7/16: P(false)=2.5e-7/poll. SACK-before-ACK guard catches cross-pattern false positives.
		const int tones[] = {8, 14, 10, 24, 26, 2, 18, 30};
		for (int i = 0; i < 8; i++) ack_tones[i] = tones[i];
	}
	else if (M == 16)
	{
		// Welch-Costas (p=17, g=5). Avoids preamble {2,6,10,14}.
		ack_pattern_len = 8;
		ack_pattern_nsymb = 16;
		ack_match_threshold = 7; // 7/16: P(false|M=16)=2.4e-5/poll. Lowered from 8: IONOS ACK detection showed peak_matched=7/8.
		const int tones[] = {4, 7, 5, 12, 13, 1, 9, 15};
		for (int i = 0; i < 8; i++) ack_tones[i] = tones[i];
	}
	else if (M == 8)
	{
		// Sidelnikov (p=37, g=2, offset=3). 32 symbols, no repetition.
		// Hamming autocorrelation: max 3 coincidences at any shift (L-G bound).
		ack_pattern_len = 32;
		ack_pattern_nsymb = 32;
		ack_match_threshold = 24; // 75% — P(false|p=0.25) ≈ 10^-11
		const int tones[] = {
			1, 3, 6, 5, 3, 7, 6, 5, 2, 5, 3, 6, 4, 1, 3, 7,
			7, 7, 6, 4, 1, 2, 4, 0, 1, 2, 5, 2, 4, 1, 3, 6
		};
		for (int i = 0; i < 32; i++) ack_tones[i] = tones[i];
	}
	else if (M == 4)
	{
		// Sidelnikov (p=53, g=2, offset=3). 48 symbols, no repetition.
		ack_pattern_len = 48;
		ack_pattern_nsymb = 48;
		ack_match_threshold = 40; // 83% — P(false|p=0.5) ≈ 10^-9
		const int tones[] = {
			0, 1, 2, 0, 1, 3, 2, 1, 2, 1, 2, 0, 1, 2, 0, 0,
			0, 1, 3, 3, 2, 0, 1, 3, 3, 3, 3, 2, 1, 3, 2, 0,
			1, 2, 1, 2, 1, 3, 2, 1, 3, 3, 3, 2, 0, 0, 1, 3
		};
		for (int i = 0; i < 48; i++) ack_tones[i] = tones[i];
	}
	else
	{
		ack_pattern_len = 8;
		ack_pattern_nsymb = 16;
		ack_match_threshold = 8;
		for (int i = 0; i < ack_pattern_len; i++)
			ack_tones[i] = (i * M / ack_pattern_len + 1) % M;
	}

	// BREAK pattern tones.
	// WB: Welch-Costas (p=17, g=7) — different generator from ACK (g=5).
	// NB: Sidelnikov with different primitive root (g=5 vs ACK g=2).
	// Cross-correlation at shift 0: M=8: 5/32, M=4: 13/48 (near random baseline).
	if (M == 32)
	{
		const int tones[] = {12, 28, 4, 6, 20, 16, 22, 30};
		for (int i = 0; i < 8; i++) break_tones[i] = tones[i];
		break_match_threshold = 7;  // 7/16: P(false|M=32)=2.5e-7/poll.
	}
	else if (M == 16)
	{
		const int tones[] = {6, 14, 2, 3, 10, 8, 11, 15};
		for (int i = 0; i < 8; i++) break_tones[i] = tones[i];
		// 10/16 (was 12). gearshift_v10 BREAK-PROBE distribution measured
		// matched values clustering at 6 and 11 with rare 16 — and a
		// natural gap at 9-10 — so the old threshold of 12 sat in the
		// gap and missed BREAK ~95% of the time. Real BREAK reliably
		// hits 11. Set at 10 for extra robustness in marginal audio
		// conditions (a future quiet channel or low TX-BREAK boost may
		// shave a tone off, making the threshold-11 case fragile).
		// P(false|coarse_metric<0.30 gate active) ≈ 4.5e-9/poll which
		// is bounded — the coarse_metric gate already excludes OFDM
		// signal that aliases the tone bins.
		break_match_threshold = 10;
	}
	else if (M == 8)
	{
		// Sidelnikov (p=37, g=5, offset=3). 32 symbols.
		const int tones[] = {
			3, 7, 3, 2, 3, 3, 1, 6, 0, 2, 2, 6, 6, 7, 4, 7,
			6, 2, 4, 0, 4, 5, 4, 4, 6, 1, 7, 5, 5, 1, 1, 0
		};
		for (int i = 0; i < 32; i++) break_tones[i] = tones[i];
		break_match_threshold = 24;
	}
	else if (M == 4)
	{
		// Sidelnikov (p=53, g=5, offset=3). 48 symbols.
		const int tones[] = {
			1, 3, 3, 3, 0, 1, 1, 0, 1, 3, 1, 0, 3, 0, 0, 0,
			2, 1, 2, 2, 2, 2, 1, 3, 3, 2, 2, 0, 0, 0, 3, 2,
			2, 3, 2, 0, 2, 3, 0, 3, 3, 3, 1, 2, 1, 1, 1, 1
		};
		for (int i = 0; i < 48; i++) break_tones[i] = tones[i];
		break_match_threshold = 40;
	}
	else
	{
		break_match_threshold = 8;
		for (int i = 0; i < ack_pattern_len; i++)
			break_tones[i] = (ack_tones[i] + M / 2) % M;
	}

	// HAIL pattern tones: "I am Mercury" beacon.
	// WB: Welch-Costas (p=17, g=6) — different generator from ACK (g=5) and BREAK (g=7).
	//   Cross-correlation: vs ACK 1/8, vs BREAK 2/8 (near random).
	// NB: Sidelnikov with different primitive roots:
	//   M=8: (p=37, g=24, offset=3) — vs ACK 8/32, vs BREAK 6/32.
	//   M=4: (p=53, g=13, offset=3) — vs ACK 10/48, vs BREAK 11/48.
	if (M == 32)
	{
		// 2x scaled M=16 Welch-Costas (g=6)
		const int tones[] = {0, 10, 2, 22, 6, 12, 14, 26};
		for (int i = 0; i < 8; i++) hail_tones[i] = tones[i];
		hail_match_threshold = 7;  // 7/16: P(false|M=32)=2.5e-7/poll.
	}
	else if (M == 16)
	{
		// Welch-Costas (p=17, g=6)
		const int tones[] = {0, 5, 1, 11, 3, 6, 7, 13};
		for (int i = 0; i < 8; i++) hail_tones[i] = tones[i];
		hail_match_threshold = 8;
	}
	else if (M == 8)
	{
		// Sidelnikov (p=37, g=24, offset=3). 32 symbols.
		const int tones[] = {
			4, 3, 0, 2, 5, 5, 6, 0, 4, 2, 7, 1, 5, 5, 4, 3,
			2, 7, 7, 0, 3, 1, 6, 6, 5, 3, 7, 1, 4, 2, 6, 6
		};
		for (int i = 0; i < 32; i++) hail_tones[i] = tones[i];
		hail_match_threshold = 24; // 75% — matches ACK/BREAK. Was 16: P(false)≈10^-4/pos on HF noise
	}
	else if (M == 4)
	{
		// Sidelnikov (p=53, g=13, offset=3). 48 symbols.
		const int tones[] = {
			0, 0, 1, 3, 2, 3, 1, 2, 3, 3, 1, 3, 0, 0, 0, 1,
			3, 2, 3, 1, 2, 3, 3, 1, 3, 0, 0, 0, 1, 3, 2, 3,
			1, 2, 3, 3, 1, 3, 0, 0, 0, 1, 3, 2, 3, 1, 2, 3
		};
		for (int i = 0; i < 48; i++) hail_tones[i] = tones[i];
		hail_match_threshold = 40;
	}
	else
	{
		hail_match_threshold = 8;
		for (int i = 0; i < ack_pattern_len; i++)
			hail_tones[i] = (ack_tones[i] + M / 4) % M;
	}

	// CONNECT base pattern (Phase B Wave 1):
	// Welch-Costas with primitive root g=3 — distinct from ACK (g=5),
	// BREAK (g=7), HAIL (g=6) so the detectors can't ambiguate. Generated
	// tones for M=16 are g^k mod 17, k=1..8 = {3,9,10,13,5,15,11,16}. The
	// trailing 16 is out-of-range for M=16, so we substitute 8 (which is
	// not used by ACK, BREAK, or HAIL at index 7 of any pattern). Result:
	// {3, 9, 10, 13, 5, 15, 11, 8} — pairwise tone-Hamming distance vs
	// ACK ≥ 6, vs BREAK ≥ 6, vs HAIL ≥ 6 (see
	// fact-documents/phase-b-mfsk-connect-research.md §11.4; verified by
	// the base_pattern_cross_correlation unit test).
	//
	// WB-only — NB CONNECT remains LDPC (per §6.6 of the research doc).
	if (M == 32)
	{
		// 2× scaled M=16 CONNECT tones with the same 16→trailing-16-fix
		// substitution: 2·{3,9,10,13,5,15,11,16} → {6,18,20,26,10,30,22,16}.
		// The trailing 16 is fine at M=32 (no out-of-range). It's also
		// distinct from ACK[6]=18, BREAK[6]=22, HAIL[6]=14 at this index.
		connect_pattern_nsymb = 16;
		connect_match_threshold = 7;
		const int tones[] = {6, 18, 20, 26, 10, 30, 22, 16};
		for (int i = 0; i < 8; i++) connect_tones[i] = tones[i];
	}
	else if (M == 16)
	{
		connect_pattern_nsymb = 16;
		connect_match_threshold = 7;
		const int tones[] = {3, 9, 10, 13, 5, 15, 11, 8};
		for (int i = 0; i < 8; i++) connect_tones[i] = tones[i];
	}
	else
	{
		// NB: no CONNECT MFSK suffix (deferred). Detector will see
		// connect_pattern_nsymb=0 and skip.
		connect_pattern_nsymb = 0;
		connect_match_threshold = 0;
	}

	// Step 15: legacy MFSK SACK tone tables removed — partial-batch SACK is
	// now exclusively the OFDM SACK_RSP control frame (arq_common.cc Step 7).

	// Initialize directed HAIL detect arrays (undirected by default)
	clear_hail_target();

	// Phase-2 validation: --wb-match-threshold-bias=N adds N to ack/break/hail
	// match thresholds for the WB cases (M=16, M=32). Pass +1 to revert
	// b806b76+7076a4b's 8→7 reductions on those thresholds.
	if(wb_match_threshold_bias != 0 && (M == 16 || M == 32))
	{
		ack_match_threshold   += wb_match_threshold_bias;
		break_match_threshold += wb_match_threshold_bias;
		hail_match_threshold  += wb_match_threshold_bias;
		connect_match_threshold += wb_match_threshold_bias;
	}
}

void cl_mfsk::set_hail_target(const char* callsign, int len)
{
	if (!callsign || len <= 0 || M == 0)
	{
		clear_hail_target();
		return;
	}

	// FNV-1a 32-bit hash of uppercase callsign (case-insensitive matching)
	uint32_t hash = 2166136261u;
	for (int i = 0; i < len; i++)
	{
		char c = callsign[i];
		if (c >= 'a' && c <= 'z') c -= 32;  // uppercase
		hash ^= (uint8_t)c;
		hash *= 16777619u;
	}

	// Derive 4 suffix tones from hash
	for (int i = 0; i < HAIL_SUFFIX_LEN; i++)
		hail_suffix[i] = (int)((hash >> (i * 5)) & 0x1F) % M;

	hail_directed = true;

	// Build flat detect array: [expanded hail tones (no modulo)] + [suffix]
	for (int s = 0; s < ack_pattern_nsymb; s++)
		hail_detect_tones[s] = hail_tones[s % ack_pattern_len];
	for (int s = 0; s < HAIL_SUFFIX_LEN; s++)
		hail_detect_tones[ack_pattern_nsymb + s] = hail_suffix[s];

	hail_detect_nsymb = ack_pattern_nsymb + HAIL_SUFFIX_LEN;
	hail_detect_threshold = hail_match_threshold + HAIL_SUFFIX_LEN;
}

void cl_mfsk::clear_hail_target()
{
	hail_directed = false;
	for (int i = 0; i < HAIL_SUFFIX_LEN; i++)
		hail_suffix[i] = 0;

	// Undirected: flat array of just the base hail tones
	for (int s = 0; s < ack_pattern_nsymb; s++)
		hail_detect_tones[s] = hail_tones[s % ack_pattern_len];

	hail_detect_nsymb = ack_pattern_nsymb;
	hail_detect_threshold = hail_match_threshold;
}

void cl_mfsk::deinit()
{
	M = 0;
	nBits = 0;
	Nc = 0;
	nStreams = 0;
	tone_hop_step = 0;
	preamble_nSymb = 0;
}

// Generate MFSK preamble: known tones in all streams simultaneously
void cl_mfsk::generate_preamble(std::complex<double>* preamble_out, int nSymb)
{
	if (M == 0 || Nc == 0 || nStreams == 0) return;

	// Amplitude: total power = Nc, split across nStreams tones
	double amp = sqrt((double)Nc / nStreams);

	for (int s = 0; s < nSymb; s++)
	{
		// Zero all subcarriers
		for (int k = 0; k < Nc; k++)
		{
			preamble_out[s * Nc + k] = std::complex<double>(0.0, 0.0);
		}
		// Place known tone in each stream's band
		int tone = preamble_tones[s % preamble_nSymb];
		for (int st = 0; st < nStreams; st++)
		{
			preamble_out[s * Nc + stream_offsets[st] + tone] = std::complex<double>(amp, 0.0);
		}
	}
}

// Generate ACK pattern: ack_pattern_nsymb symbols of known tones.
// WB: 8 base tones with hopping, repeated 2x. NB: full Sidelnikov sequence, no hopping.
void cl_mfsk::generate_ack_pattern(std::complex<double>* pattern_out)
{
	if (M == 0 || Nc == 0 || nStreams == 0) return;

	double amp = sqrt((double)Nc / nStreams);

	for (int s = 0; s < ack_pattern_nsymb; s++)
	{
		for (int k = 0; k < Nc; k++)
		{
			pattern_out[s * Nc + k] = std::complex<double>(0.0, 0.0);
		}

		int tone_base = ack_tones[s % ack_pattern_len];
		int actual_tone = (tone_base + s * tone_hop_step) % M;

		for (int st = 0; st < nStreams; st++)
		{
			pattern_out[s * Nc + stream_offsets[st] + actual_tone] = std::complex<double>(amp, 0.0);
		}
	}
}

// RECOVERY-ACK robustness (recovery-ack-robustness.md §4): emit the ACK base
// block ack_base_total_nsymb() = recovery_ack_reps * ack_pattern_nsymb symbols.
// Each rep is IDENTICAL — symbol s of every rep carries the SAME tone (per-rep-
// LOCAL hop index s, NOT a continued abs index) so the RX detector can sum the
// energy of rep-r symbol s onto rep-0 symbol s (same expected bin). reps=1 →
// exactly the generate_ack_pattern single 16-symbol block (byte-identical).
// Mirrors generate_connect_pattern's §20 layout.
void cl_mfsk::generate_ack_pattern_reps(std::complex<double>* pattern_out)
{
	// TERNACK2: the default ACK variant is just generate_ack_pattern_reps_tones
	// with the ACK (g=5) table — byte-identical to the prior body.
	generate_ack_pattern_reps_tones(pattern_out, ack_tones);
}

// TERNACK2 (data-flow-ternack2-variant-confirm.md §2): R-rep base block over a
// caller-supplied 8-tone table. Layout/hop/streams IDENTICAL to the ACK path so
// detect_ack_pattern(tones,...) matched-filters each variant verbatim. The ONLY
// difference from generate_ack_pattern_reps is which 8-tone table feeds tone_base.
void cl_mfsk::generate_ack_pattern_reps_tones(std::complex<double>* pattern_out,
                                              const int* tones)
{
	if (M == 0 || Nc == 0 || nStreams == 0) return;
	if (ack_pattern_nsymb <= 0) return;
	if (tones == nullptr) tones = ack_tones;

	double amp = sqrt((double)Nc / nStreams);

	int total_base = ack_base_total_nsymb();   // reps * ack_pattern_nsymb (clamped)
	for (int abs_b = 0; abs_b < total_base; abs_b++)
	{
		int s = abs_b % ack_pattern_nsymb;   // index WITHIN the base block
		for (int k = 0; k < Nc; k++)
			pattern_out[abs_b * Nc + k] = std::complex<double>(0.0, 0.0);

		int tone_base = tones[s % ack_pattern_len];
		int actual_tone = (tone_base + s * tone_hop_step) % M;

		for (int st = 0; st < nStreams; st++)
			pattern_out[abs_b * Nc + stream_offsets[st] + actual_tone] =
				std::complex<double>(amp, 0.0);
	}
}

// Generate BREAK pattern: identical structure to ACK but with break_tones
void cl_mfsk::generate_break_pattern(std::complex<double>* pattern_out)
{
	if (M == 0 || Nc == 0 || nStreams == 0) return;

	double amp = sqrt((double)Nc / nStreams);

	for (int s = 0; s < ack_pattern_nsymb; s++)
	{
		for (int k = 0; k < Nc; k++)
		{
			pattern_out[s * Nc + k] = std::complex<double>(0.0, 0.0);
		}

		int tone_base = break_tones[s % ack_pattern_len];
		int actual_tone = (tone_base + s * tone_hop_step) % M;

		for (int st = 0; st < nStreams; st++)
		{
			pattern_out[s * Nc + stream_offsets[st] + actual_tone] = std::complex<double>(amp, 0.0);
		}
	}
}

// Generate HAIL pattern: "I am Mercury" prefix + optional CRC suffix for directed hailing.
// When hail_directed is true, appends HAIL_SUFFIX_LEN symbols derived from target callsign.
void cl_mfsk::generate_hail_pattern(std::complex<double>* pattern_out)
{
	if (M == 0 || Nc == 0 || nStreams == 0) return;

	double amp = sqrt((double)Nc / nStreams);

	// Generate all symbols (prefix + suffix) from the flat detect array
	for (int s = 0; s < hail_detect_nsymb; s++)
	{
		for (int k = 0; k < Nc; k++)
		{
			pattern_out[s * Nc + k] = std::complex<double>(0.0, 0.0);
		}

		int tone_base = hail_detect_tones[s];
		int actual_tone = (tone_base + s * tone_hop_step) % M;

		for (int st = 0; st < nStreams; st++)
		{
			pattern_out[s * Nc + stream_offsets[st] + actual_tone] = std::complex<double>(amp, 0.0);
		}
	}
}

// SNR quantization: map float SNR to MFSK tone index
int cl_mfsk::snr_to_tone(float snr) const
{
	if (M == 0) return 0;
	// WB (M>=16): tone = (SNR + 5) / 2, range -5..+25 dB
	// NB (M<=8):  tone = (SNR + 9) / 2, range -9..+5 dB
	float offset = (M >= 16) ? 5.0f : 9.0f;
	int tone = (int)((snr + offset) / 2.0f + 0.5f);
	if (tone < 0) tone = 0;
	if (tone >= M) tone = M - 1;
	return tone;
}

float cl_mfsk::tone_to_snr(int tone) const
{
	if (M == 0) return -99.0f;
	float offset = (M >= 16) ? 5.0f : 9.0f;
	return (float)tone * 2.0f - offset;
}

// =============================================================================
// Generic MFSK control-suffix codec
// =============================================================================
//
// 52-bit field [type:2 | payload:38 | crc12:12], MSB-first. At M=16 the
// 13-symbol suffix carries 4 bits/symbol. The 2-bit type field is the
// flag-day break with pre-2026-05-26 deployed peers — see
// fact-documents/phase-b-mfsk-connect-research.md §11.1. Returns number of
// tones written (= ack_sack_suffix_len()).
int cl_mfsk::pack_ctrl_suffix(mfsk_ctrl_frame_type type, uint64_t payload38,
                              uint16_t crc12, int* out_tones, bool fec) const
{
	int n = ack_sack_suffix_len();
	if (n == 0 || M < 16) return 0;
	// Tier-2 FEC (§19): emit the GF(16) RA codeword (N=codeword_len() tones,
	// each 0..15) instead of the 13-symbol hard bit-pack. The codeword carries
	// [type:2|payload38:38] in 10 systematic GF(16) info symbols + the 12-bit
	// CRC in 3 protected info symbols + RA parity; the tone values are 0..M-1
	// exactly like the hard pack, so the one-hot mapping downstream is identical.
	// gf16ra is configured (repfact)/inited once at FEC enable. §21: `fec` is the
	// EXPLICIT per-call decision (CONNECT passes suffix_fec_coded, ACK passes
	// ack_suffix_fec_coded) — pack_ctrl_suffix no longer reads any global.
	if (fec) {
		gf16ra::encode(type, payload38, crc12, out_tones);
		return gf16ra::codeword_len();
	}
	int bits_per_tone = 0;
	for (int m = M; m > 1; m >>= 1) bits_per_tone++;  // log2(M); 4 for M=16
	uint64_t payload = ((uint64_t)(type & 0x3) << 50)
	                 | ((payload38 & ((1ULL << 38) - 1ULL)) << 12)
	                 | ((uint64_t)(crc12 & 0x0FFF));
	int total_bits = 2 + 38 + 12;  // 52
	int mask = M - 1;
	for (int g = 0; g < n; g++) {
		int shift = total_bits - bits_per_tone * (g + 1);
		if (shift < 0) shift = 0;
		out_tones[g] = (int)((payload >> shift) & mask);
	}
	return n;
}

// Inverse of pack_ctrl_suffix. Reconstruct (type, payload38, crc12) from N
// tones. Caller is responsible for verifying crc12 against a freshly-computed
// CRC12 over [type:2|payload:38] packed as 5 bytes — this function does the
// bit-level unpack only.
bool cl_mfsk::unpack_ctrl_suffix(const int* in_tones,
                                 mfsk_ctrl_frame_type* out_type,
                                 uint64_t* out_payload38,
                                 uint16_t* out_crc12) const
{
	if (!in_tones || !out_type || !out_payload38 || !out_crc12) return false;
	int n = ack_sack_suffix_len();
	if (n == 0 || M < 16) return false;
	int bits_per_tone = 0;
	for (int m = M; m > 1; m >>= 1) bits_per_tone++;
	uint64_t payload = 0;
	for (int g = 0; g < n; g++) {
		uint64_t t = (uint64_t)(in_tones[g] & (M - 1));
		payload = (payload << bits_per_tone) | t;
	}
	// payload now holds 52 bits in its low bits: [type:2|payload38:38|crc12:12].
	*out_crc12     = (uint16_t)(payload & 0x0FFF);
	*out_payload38 = (uint64_t)((payload >> 12) & ((1ULL << 38) - 1ULL));
	*out_type      = (mfsk_ctrl_frame_type)((payload >> 50) & 0x3);
	return true;
}

// =============================================================================
// Backward-named ACK+SACK wrappers around pack/unpack_ctrl_suffix
// =============================================================================
//
// payload38 = [bsi:8 | bitmap:30], total 38 bits. The bitmap shrank from 32
// to 30 bits as part of the Phase B Wave 1 flag-day (the 2-bit type prefix
// stole the high two bits). Bits 30/31 of an input bitmap are dropped with a
// stderr warning — the producer in arq_responder.cc:1270/1408 caps at 30
// before calling this helper (invariant: data_batch_size <= 30).
int cl_mfsk::pack_ack_sack_payload(uint8_t bsi, uint32_t bitmap, uint16_t crc12,
                                   int* out_tones) const
{
	if ((bitmap & 0xC0000000u) != 0) {
		static bool warned = false;
		if (!warned) {
			fprintf(stderr,
				"[MFSK] pack_ack_sack_payload: bitmap=0x%08x has bits "
				"30/31 set — dropping (30-bit cap, fact-doc §11.2)\n",
				(unsigned)bitmap);
			warned = true;
		}
	}
	uint32_t bitmap30 = bitmap & 0x3FFFFFFFu;
	uint64_t payload38 = ((uint64_t)bsi << 30) | (uint64_t)bitmap30;
	// §21: ACK uses its OWN fec flag (ack_suffix_fec_coded), NEVER the CONNECT
	// suffix_fec_coded — so an FEC-on CONNECT session cannot garble the data ACK
	// (the §21.1 fix). Default false → byte-identical 13-tone ACK suffix.
	return pack_ctrl_suffix(MFSK_CTRL_ACK_SACK, payload38, crc12, out_tones,
	                        ack_suffix_fec_coded);
}

bool cl_mfsk::unpack_ack_sack_payload(const int* in_tones,
                                      uint8_t* out_bsi, uint32_t* out_bitmap,
                                      uint16_t* out_crc12) const
{
	if (!out_bsi || !out_bitmap || !out_crc12) return false;
	mfsk_ctrl_frame_type type = MFSK_CTRL_ACK_SACK;
	uint64_t payload38 = 0;
	if (!unpack_ctrl_suffix(in_tones, &type, &payload38, out_crc12))
		return false;
	// Type-discriminator mismatch is a CRC-equivalent failure — the caller
	// already treats CRC mismatch as "no ACK arrived" so returning here
	// would change behavior; pass the field through and let the caller
	// re-check the type after CRC12 succeeds (Wave 2 producer path will
	// pre-check it explicitly).
	*out_bitmap = (uint32_t)(payload38 & 0x3FFFFFFFu);          // bits 29..0
	*out_bsi    = (uint8_t)((payload38 >> 30) & 0xFFu);         // bits 37..30
	(void)type;
	return true;
}

// Decode the (bsi, bitmap) pair from the most recent capture written into
// last_ack_sack_suffix_tones[] by the detector hook (in telecom_system.cc:
// detect_ack_snr_from_passband). Returns false if M doesn't support SACK
// (NB) or if no fresh capture is available.
//
// The captured tones are already de-hopped by ofdm.decode_suffix_tones —
// that helper inverts the (payload + abs_s*tone_hop_step) % M mapping the
// transmitter applies in generate_ack_sack_pattern, so we can feed them
// directly into unpack_ack_sack_payload. Conceptually:
//   payload_tone[g] = (raw_argmax_bin[g] - (ack_pattern_nsymb+g)*tone_hop_step) mod M
// — that inversion happens inside decode_suffix_tones; this function only
// runs the bit-level unpack on the result.
bool cl_mfsk::decode_ack_sack_from_last_capture(uint8_t* out_bsi,
                                                uint32_t* out_bitmap,
                                                uint16_t* out_crc12)
{
	if (ack_sack_suffix_len() == 0) return false;  // NB or unsupported
	if (!last_ack_sack_capture_valid) return false;
	if (out_bsi == nullptr || out_bitmap == nullptr || out_crc12 == nullptr)
		return false;
	return unpack_ack_sack_payload(last_ack_sack_suffix_tones,
	                               out_bsi, out_bitmap, out_crc12);
}

// Test-only injection point: stuff de-hopped payload tones directly into
// last_ack_sack_suffix_tones[] and flag the capture as valid, bypassing
// the RF path. The symbol-domain round-trip test uses this so it can
// validate the decode primitive without setting up FFT plumbing.
void cl_mfsk::test_inject_ack_sack_capture(const int* tones, int count)
{
	int n = ack_sack_suffix_len();
	if (n == 0 || tones == nullptr) {
		last_ack_sack_capture_valid = false;
		return;
	}
	if (count > n) count = n;
	if (count > MAX_ACK_SACK_SUFFIX) count = MAX_ACK_SACK_SUFFIX;
	for (int i = 0; i < count; i++)
		last_ack_sack_suffix_tones[i] = tones[i] & (M - 1);
	for (int i = count; i < MAX_ACK_SACK_SUFFIX; i++)
		last_ack_sack_suffix_tones[i] = -1;
	last_ack_sack_capture_valid = (count == n);
}

// Decode the most-recent CONNECT-suffix capture into (type, payload, crc12).
// Mirror of decode_ack_sack_from_last_capture, but draws from
// last_connect_suffix_tones[] (populated by the CONNECT-detector hook in
// telecom_system.cc).
bool cl_mfsk::decode_ctrl_suffix_from_last_capture(
	mfsk_ctrl_frame_type* out_type, uint64_t* out_payload38,
	uint16_t* out_crc12)
{
	if (ack_sack_suffix_len() == 0) return false;
	if (!last_connect_capture_valid) return false;
	if (!out_type || !out_payload38 || !out_crc12) return false;
	return unpack_ctrl_suffix(last_connect_suffix_tones,
	                          out_type, out_payload38, out_crc12);
}

// Test-only: stuff CONNECT-suffix payload tones directly into the capture
// buffer (bypasses RF). Mirror of test_inject_ack_sack_capture.
void cl_mfsk::test_inject_connect_capture(const int* tones, int count)
{
	int n = ack_sack_suffix_len();
	if (n == 0 || tones == nullptr) {
		last_connect_capture_valid = false;
		return;
	}
	if (count > n) count = n;
	if (count > MAX_ACK_SACK_SUFFIX) count = MAX_ACK_SACK_SUFFIX;
	for (int i = 0; i < count; i++)
		last_connect_suffix_tones[i] = tones[i] & (M - 1);
	for (int i = count; i < MAX_ACK_SACK_SUFFIX; i++)
		last_connect_suffix_tones[i] = -1;
	last_connect_capture_valid = (count == n);
}

// Generate ACK base + ack_sack_suffix_len() ACK+SACK suffix symbols.
// Suffix layout mirrors generate_ack_snr_pattern: same tone-hopping formula.
// Caller computes crc12 over the packed [type:2|bsi:8|bitmap:30] (5 bytes
// MSB-aligned) and passes it; we don't compute it here because cl_mfsk has
// no access to the ARQ-layer CRC12_calc helper. See fact-doc §11.3.
void cl_mfsk::generate_ack_sack_pattern(std::complex<double>* pattern_out,
                                        uint8_t bsi, uint32_t bitmap,
                                        uint16_t crc12)
{
	if (M == 0 || Nc == 0 || nStreams == 0) return;
	// REVSACK Part A: loop over the CODED suffix length (coded N when ack_suffix_fec_
	// coded, 13 uncoded) so the GF(16) RA codeword is fully laid down. Was
	// ack_sack_suffix_len() (always 13) — with FEC on that emitted only the first 13
	// of N tones (the systematic prefix), the wire bug §21.3 flags as the missing
	// "coded-window sizing work". FEC-off path is byte-identical (coded len == 13).
	int suffix_len = ack_sack_coded_suffix_len();
	if (suffix_len == 0) return;  // NB unsupported for now

	// First: generate the standard ACK base pattern (16 symbols WB)
	generate_ack_pattern(pattern_out);

	// Pack [type:2|bsi:8|bitmap:30|crc12:12] = 52 bits into per-symbol tones.
	// pack_ack_sack_payload emits the GF(16) RA codeword (N tones) when
	// ack_suffix_fec_coded is set, else the 13-symbol hard pack — matching suffix_len.
	int payload_tones[MAX_ACK_SACK_SUFFIX];
	pack_ack_sack_payload(bsi, bitmap, crc12, payload_tones);

	double amp = sqrt((double)Nc / nStreams);
	for (int s = 0; s < suffix_len; s++) {
		int abs_s = ack_pattern_nsymb + s;  // absolute symbol index
		for (int k = 0; k < Nc; k++)
			pattern_out[abs_s * Nc + k] = std::complex<double>(0.0, 0.0);

		// Tone hopping consistent with ACK pattern + SNR suffix (same formula).
		int actual_tone = (payload_tones[s] + abs_s * tone_hop_step) % M;
		for (int st = 0; st < nStreams; st++)
			pattern_out[abs_s * Nc + stream_offsets[st] + actual_tone] =
				std::complex<double>(amp, 0.0);
	}
}

// Option B compact confirm: ACK base + the K=5 GF(16)-RA compact codeword
// (N=10 sym) carrying [bsi:8|crc12:12]. Mirrors generate_ack_sack_pattern's
// tone-hopping (abs_s = ack_pattern_nsymb + g) so the RX uses the SAME base
// detector + de-hop math. crc12 = caller-supplied CRC12 over [bsi]. The base
// pattern is the ACK base (ack_tones) so the detector routes it as an ACK; the
// commander's CRC12-gated compact decode is what distinguishes it from a 13-sym
// SACK/clean-data-ACK suffix (data-flow-compact-confirm.md §4).
void cl_mfsk::generate_compact_confirm_pattern(std::complex<double>* pattern_out,
                                               uint8_t bsi, uint16_t crc12)
{
	if (M == 0 || Nc == 0 || nStreams == 0) return;
	int suffix_len = compact_confirm_suffix_len();
	if (suffix_len == 0) return;  // NB unsupported

	// ACK base pattern (16 sym WB) — identical to the ACK+SACK path.
	generate_ack_pattern(pattern_out);

	// K=5 GF(16)-RA compact codeword tones (N = compact_codeword_len() = 10).
	int code_tones[MAX_ACK_SACK_SUFFIX];
	gf16ra::encode_compact(bsi, crc12, code_tones);

	double amp = sqrt((double)Nc / nStreams);
	for (int s = 0; s < suffix_len; s++) {
		int abs_s = ack_pattern_nsymb + s;  // absolute symbol index (continues the hop)
		for (int k = 0; k < Nc; k++)
			pattern_out[abs_s * Nc + k] = std::complex<double>(0.0, 0.0);
		int actual_tone = (code_tones[s] + abs_s * tone_hop_step) % M;
		for (int st = 0; st < nStreams; st++)
			pattern_out[abs_s * Nc + stream_offsets[st] + actual_tone] =
				std::complex<double>(amp, 0.0);
	}
}

// Generate CONNECT base pattern: 16 symbols (WB) hopping over connect_tones.
// Mirror of generate_ack_pattern but uses connect_tones instead.
void cl_mfsk::generate_connect_pattern(std::complex<double>* pattern_out)
{
	if (M == 0 || Nc == 0 || nStreams == 0) return;
	if (connect_pattern_nsymb <= 0) return;  // NB unsupported

	double amp = sqrt((double)Nc / nStreams);

	// CONNECT base uses an 8-tone base sequence × 2 reps = 16 symbols (WB),
	// same shape as ACK; the base sequence is connect_tones[0..7].
	const int base_len = 8;
	// §20: noncoherent base-pattern combining. Emit the connect_pattern_nsymb
	// base block connect_preamble_reps times. Each rep is IDENTICAL — symbol s
	// of every rep carries the SAME tone (per-rep-LOCAL hop index s, NOT a
	// continued abs index), so the RX detector can sum the energy of rep-r
	// symbol s onto rep-0 symbol s (same expected bin). reps=1 → exactly the
	// pre-§20 single 16-symbol block (byte-identical).
	int total_base = connect_base_total_nsymb();   // reps * connect_pattern_nsymb
	for (int abs_b = 0; abs_b < total_base; abs_b++)
	{
		int s = abs_b % connect_pattern_nsymb;   // index WITHIN the base block
		for (int k = 0; k < Nc; k++)
			pattern_out[abs_b * Nc + k] = std::complex<double>(0.0, 0.0);

		int tone_base = connect_tones[s % base_len];
		int actual_tone = (tone_base + s * tone_hop_step) % M;

		for (int st = 0; st < nStreams; st++)
			pattern_out[abs_b * Nc + stream_offsets[st] + actual_tone] =
				std::complex<double>(amp, 0.0);
	}
}

// Generate CONNECT base + 13-symbol ctrl-suffix carrying (type, payload, crc12).
// Mirror of generate_ack_sack_pattern but uses the CONNECT base pattern so the
// detector can route by base correlation. See fact-doc §11.4.
void cl_mfsk::generate_ctrl_suffix_pattern(std::complex<double>* pattern_out,
                                            mfsk_ctrl_frame_type type,
                                            uint64_t payload38, uint16_t crc12)
{
	if (M == 0 || Nc == 0 || nStreams == 0) return;
	// §19: coded length (52 with FEC, 13 uncoded). pack_ctrl_suffix below
	// returns the same count; we loop over it for the one-hot tone placement.
	int suffix_len = ctrl_suffix_len();
	if (suffix_len == 0) return;  // NB unsupported
	if (connect_pattern_nsymb <= 0) return;

	// First: CONNECT base pattern (R×16 symbols WB when combining, NOT ack_tones).
	generate_connect_pattern(pattern_out);

	int payload_tones[MAX_ACK_SACK_SUFFIX];
	// §21: CONNECT passes its own suffix_fec_coded (the loop bound ctrl_suffix_len()
	// reads the same flag → consistent N). pack emits 13 (uncoded) or N (FEC).
	pack_ctrl_suffix(type, payload38, crc12, payload_tones, suffix_fec_coded);

	// §20: the suffix follows ALL R base reps (combining is on the base, not the
	// suffix). connect_base_total_nsymb() = R*connect_pattern_nsymb (=16 when
	// reps=1, byte-identical). The hop index abs_s uses this same base total so
	// TX and the RX suffix-decode offset (which is passed connect_base_total_nsymb)
	// stay phase-consistent (§20.3 C5).
	int base_total = connect_base_total_nsymb();
	double amp = sqrt((double)Nc / nStreams);
	for (int s = 0; s < suffix_len; s++) {
		int abs_s = base_total + s;  // suffix index after ALL base reps
		for (int k = 0; k < Nc; k++)
			pattern_out[abs_s * Nc + k] = std::complex<double>(0.0, 0.0);

		int actual_tone = (payload_tones[s] + abs_s * tone_hop_step) % M;
		for (int st = 0; st < nStreams; st++)
			pattern_out[abs_s * Nc + stream_offsets[st] + actual_tone] =
				std::complex<double>(amp, 0.0);
	}
}

// In-band rate adaptation (Stage 3a): CONNECT base pattern + the CONFIG_TAG
// GF(16) RA codeword as the suffix. MIRROR of generate_ctrl_suffix_pattern; the
// ONLY difference is the suffix tones come from `tones[]` (the
// gf16ra::encode_config_tag codeword the ARQ layer already built) instead of
// pack_ctrl_suffix. Same base, same one-hot per-symbol tone placement, same hop
// formula and base_total offset — so the SAME detect_ack_pattern correlator
// locates it and the SAME decode_suffix_energies extracts the per-tone energy
// matrix the config_tag_wrap_decode consumes. n_suffix MUST be
// gf16ra::codeword_len().
void cl_mfsk::generate_config_tag_mfsk_pattern(std::complex<double>* pattern_out,
                                               const int* tones, int n_suffix)
{
	if (M == 0 || Nc == 0 || nStreams == 0) return;
	if (n_suffix <= 0 || tones == NULL) return;
	if (connect_pattern_nsymb <= 0) return;

	// Stage 3c: the base is the TRIMMED acquisition sync (config_tag_sync_nsymb()
	// symbols, default a swept minimum < the full 16) — a prefix of the connect
	// base sequence with NO rep-combining. The deterministic offset (the tag rides
	// right after frame-0) replaces blind-acquire rep-integration. The same
	// detect_ack_pattern correlator (on connect_tones, combine_reps=1) finds it.
	int base_total = config_tag_sync_nsymb();
	generate_config_tag_base(pattern_out, base_total);

	// The suffix follows the (trimmed) base. abs_s uses base_total so TX and the RX
	// suffix-decode offset (passed config_tag_sync_nsymb()) stay phase-consistent —
	// the SAME hop discipline the ctrl-suffix uses, just at a shorter base offset.
	double amp = sqrt((double)Nc / nStreams);
	for (int s = 0; s < n_suffix; s++) {
		int abs_s = base_total + s;  // suffix index after the trimmed base
		for (int k = 0; k < Nc; k++)
			pattern_out[abs_s * Nc + k] = std::complex<double>(0.0, 0.0);

		int tone = tones[s];
		if (tone < 0 || tone >= M) tone = 0;  // defensive clamp
		int actual_tone = (tone + abs_s * tone_hop_step) % M;
		for (int st = 0; st < nStreams; st++)
			pattern_out[abs_s * Nc + stream_offsets[st] + actual_tone] =
				std::complex<double>(amp, 0.0);
	}
}

// Stage 3c: number of base-sync symbols the CONFIG_TAG burst emits/searches.
// Env MERCURY_INBAND_TAG_SYNC_REPS overrides (cached once); clamped to
// [1, connect_pattern_nsymb]. Default CFG_TAG_SYNC_NSYMB_DEFAULT (the swept
// minimum keeping detection >=99% at the op SNR with timing jitter + FAR held).
int cl_mfsk::config_tag_sync_nsymb() const
{
	// Test-only override takes precedence (the Stage-3c sweep varies this per
	// iteration; the env cache below is process-static so it cannot be re-read).
	if (tag_sync_nsymb_override > 0) {
		int v = tag_sync_nsymb_override;
		int hi = (connect_pattern_nsymb > 0) ? connect_pattern_nsymb : 16;
		if (v < 1) v = 1;
		if (v > hi) v = hi;
		return v;
	}
	static int cached = -1;   // -1 = unresolved
	if (cached < 0) {
		int v = CFG_TAG_SYNC_NSYMB_DEFAULT;
		const char* e = std::getenv("MERCURY_INBAND_TAG_SYNC_REPS");
		if (e != nullptr && *e) {
			int parsed = atoi(e);
			if (parsed > 0) v = parsed;
		}
		cached = v;
	}
	int v = cached;
	int hi = (connect_pattern_nsymb > 0) ? connect_pattern_nsymb : 16;
	if (v < 1) v = 1;
	if (v > hi) v = hi;
	return v;
}

// Stage 3c: the matched-count gate for the trimmed tag base. Scale
// connect_match_threshold by the base-length ratio (ceil) so the detection
// stringency-per-symbol is preserved, floored at CFG_TAG_SYNC_MATCH_MIN (FAR
// backstop on very short bases). At the full base this equals
// connect_match_threshold (byte-identical to the CONNECT gate).
int cl_mfsk::config_tag_sync_match_threshold() const
{
	int n   = config_tag_sync_nsymb();
	int full = (connect_pattern_nsymb > 0) ? connect_pattern_nsymb : 16;
	int base_thr = (connect_match_threshold > 0) ? connect_match_threshold : 7;
	// ceil(base_thr * n / full)
	int thr = (base_thr * n + full - 1) / full;
	if (thr < CFG_TAG_SYNC_MATCH_MIN) thr = CFG_TAG_SYNC_MATCH_MIN;
	if (thr > n) thr = n;   // can't match more symbols than exist
	return thr;
}

// Stage 3c: emit `nsymb` base-sync symbols (a prefix of the connect base
// sequence, NO rep-combining) at the front of pattern_out. The per-symbol-LOCAL
// hop index is `s` (== detect_ack_pattern's `p`), so the RX correlator searching
// connect_tones with combine_reps=1 aligns symbol-for-symbol. This is the
// reps=1 single-block path of generate_connect_pattern, truncated to nsymb
// symbols — DECOUPLED from connect_preamble_reps (CONNECT keeps its full base).
void cl_mfsk::generate_config_tag_base(std::complex<double>* pattern_out, int nsymb)
{
	if (M == 0 || Nc == 0 || nStreams == 0) return;
	if (connect_pattern_nsymb <= 0) return;
	if (nsymb < 1) nsymb = 1;
	if (nsymb > connect_pattern_nsymb) nsymb = connect_pattern_nsymb;

	const int base_len = 8;
	double amp = sqrt((double)Nc / nStreams);
	for (int s = 0; s < nsymb; s++) {
		for (int k = 0; k < Nc; k++)
			pattern_out[s * Nc + k] = std::complex<double>(0.0, 0.0);

		int tone_base = connect_tones[s % base_len];
		int actual_tone = (tone_base + s * tone_hop_step) % M;
		for (int st = 0; st < nStreams; st++)
			pattern_out[s * Nc + stream_offsets[st] + actual_tone] =
				std::complex<double>(amp, 0.0);
	}
}

// Generate ACK pattern + 4 SNR suffix symbols
void cl_mfsk::generate_ack_snr_pattern(std::complex<double>* pattern_out, float snr)
{
	if (M == 0 || Nc == 0 || nStreams == 0) return;

	// First: generate normal ACK pattern
	generate_ack_pattern(pattern_out);

	// Then: append SNR suffix symbols
	int snr_tone = snr_to_tone(snr);
	double amp = sqrt((double)Nc / nStreams);

	for (int s = 0; s < SNR_SUFFIX_LEN; s++)
	{
		int abs_s = ack_pattern_nsymb + s;  // absolute symbol index
		for (int k = 0; k < Nc; k++)
			pattern_out[abs_s * Nc + k] = std::complex<double>(0.0, 0.0);

		// Apply tone hopping consistent with ACK pattern (same formula)
		int actual_tone = (snr_tone + abs_s * tone_hop_step) % M;

		for (int st = 0; st < nStreams; st++)
			pattern_out[abs_s * Nc + stream_offsets[st] + actual_tone] = std::complex<double>(amp, 0.0);
	}
}

// TX: Map groups of bits to one-hot subcarrier vectors across all streams
// Each symbol period consumes nStreams * nBits input bits
void cl_mfsk::mod(const int* bits_in, int total_bits,
                  std::complex<double>* symbols_out)
{
	if (M == 0 || nBits == 0 || Nc == 0 || nStreams == 0) return;

	int bps = nBits * nStreams; // bits per symbol period
	int nSymbols = total_bits / bps;

	// Amplitude: total power = Nc, split across nStreams active tones
	double amp = sqrt((double)Nc / nStreams);

	for (int s = 0; s < nSymbols; s++)
	{
		// Zero all subcarriers for this symbol
		for (int k = 0; k < Nc; k++)
		{
			symbols_out[s * Nc + k] = std::complex<double>(0.0, 0.0);
		}

		// Process each stream
		for (int st = 0; st < nStreams; st++)
		{
			int bit_offset = s * bps + st * nBits;

			// Convert nBits bits to tone index (Gray code mapping)
			int tone_index = 0;
			for (int b = 0; b < nBits; b++)
			{
				if (bits_in[bit_offset + b])
				{
					tone_index |= (1 << (nBits - 1 - b));
				}
			}

			// Gray to binary conversion for better bit-error properties
			int binary_index = tone_index;
			for (int shift = 1; shift < nBits; shift++)
			{
				binary_index ^= (tone_index >> shift);
			}
			tone_index = binary_index;

			if (tone_index >= M) tone_index = M - 1;

			// Apply tone hopping for frequency diversity
			int actual_tone = (tone_index + s * tone_hop_step) % M;

			// Place in this stream's band
			symbols_out[s * Nc + stream_offsets[st] + actual_tone] = std::complex<double>(amp, 0.0);
		}
	}
}

// RX: Non-coherent energy detection across all streams with soft LLR output
void cl_mfsk::demod(const std::complex<double>* fft_in, int total_bits,
                    float* llr_out)
{
	if (M == 0 || nBits == 0 || Nc == 0 || nStreams == 0) return;

	int bps = nBits * nStreams; // bits per symbol period
	int nSymbols = total_bits / bps;

	// Codeword-wide noise variance: average guard-bin energy over ALL symbols
	// in this codeword instead of per-symbol. With ~18 guard bins/symbol
	// (Nc=50 minus nStreams*M used by signal), per-symbol estimates carry
	// ~33% RMS error which scrambles LLR magnitudes; rate-1/16 LDPC at
	// ROBUST_0 is acutely sensitive to LLR-magnitude jitter. Pooling across
	// ~240 symbols × ~18 bins drops the error below 2% RMS. See
	// fact-documents/weak-signal-floor-investigation.md §4 (F4).
	int band_start = stream_offsets[0];
	int band_end = stream_offsets[nStreams - 1] + M;
	double noise_sum_all = 0.0;
	int noise_bins_all = 0;
	for (int s = 0; s < nSymbols; s++)
	{
		for (int k = 0; k < Nc; k++)
		{
			if (k < band_start || k >= band_end)
			{
				std::complex<double> val = fft_in[s * Nc + k];
				double e = val.real() * val.real() + val.imag() * val.imag();
				if (std::isfinite(e)) {
					noise_sum_all += e;
					noise_bins_all++;
				}
			}
		}
	}
	double noise_var = (noise_bins_all > 0) ? noise_sum_all / noise_bins_all : 1e-30;
	if (noise_var < 1e-30) noise_var = 1e-30;
	double llr_scale = 1.0 / noise_var;

	// Codeword SNR estimate (the long-standing MFSK SNR TODO): accumulate the
	// PEAK tone energy per symbol-stream below. The peak bin holds the
	// transmitted tone (signal + noise); pooled `noise_var` is one noise bin's
	// energy. After the loop:
	//   SNR_dB = 10*log10( max(mean(E_peak) - noise_var, eps) / noise_var ).
	// Pooled over ~nSymbols*nStreams peaks → the same low-variance estimate the
	// LLR scale relies on. Noncoherent FSK, Proakis 5th ed §4.5.4.
	double peak_sum_all = 0.0;
	int    peak_count_all = 0;

	for (int s = 0; s < nSymbols; s++)
	{
		// Process each stream independently
		for (int st = 0; st < nStreams; st++)
		{
			// Measure energy in this stream's tone bins
			double E_raw[64]; // M <= 64
			double E_peak = 0.0;
			for (int m = 0; m < M; m++)
			{
				std::complex<double> val = fft_in[s * Nc + stream_offsets[st] + m];
				E_raw[m] = val.real() * val.real() + val.imag() * val.imag();
				if (!std::isfinite(E_raw[m])) { E_raw[m] = 0.0; }
				if (E_raw[m] > E_peak) E_peak = E_raw[m];
			}
			// The de-hopped winning tone is the per-symbol signal observation;
			// its energy == max over the M raw bins (hopping only permutes them).
			peak_sum_all += E_peak;
			peak_count_all++;

			// Reverse tone hopping: E[data_tone] = E_raw[actual_tone]
			double E[64];
			int hop = (s * tone_hop_step) % M;
			for (int m = 0; m < M; m++)
			{
				int actual = (m + hop) % M;
				E[m] = E_raw[actual];
			}

			// Compute LLRs for this stream's bits — log-sum-exp noncoherent FSK
			// metric (F2 retry on top of Q3). Per Proakis 5th ed §4.5.4 and
			// Stark, IEEE TCOM 1985, the true bit LLR for noncoherent
			// orthogonal FSK with Gray mapping is
			//   LLR_k = log(sum_{m in S_0} exp(E_m/sigma^2))
			//         - log(sum_{m in S_1} exp(E_m/sigma^2))
			// max-log is the high-SNR limit (the largest exp dominates). At
			// rate-1/16 LDPC the per-tone curvature near the cliff matters,
			// so we use the full LSE form with the standard max-subtraction
			// for numerical stability.
			//
			// First F2 attempt 2026-05-25 (reverted, see fact-document
			// weak-signal-floor-investigation.md §4 (F2)): regressed the
			// floor from WGN:0 to WGN:6. Hypothesis was that LSE produces
			// smaller LLR magnitudes which need more SPA iterations to
			// converge, and the global 100-iter cap was binding. Q3 (commit
			// cb779d1) raised the ROBUST-tier iter cap to 200; retry on top.
			int llr_offset = s * bps + st * nBits;
			for (int k = 0; k < nBits; k++)
			{
				int mask = 1 << (nBits - 1 - k);
				double max_E1 = -1e30;
				double max_E0 = -1e30;
				for (int m = 0; m < M; m++)
				{
					int gray_m = m ^ (m >> 1);
					if (gray_m & mask)
					{
						if (E[m] > max_E1) max_E1 = E[m];
					}
					else
					{
						if (E[m] > max_E0) max_E0 = E[m];
					}
				}

				// Numerically-stable log-sum-exp over S_0 and S_1
				// separately, using the per-set max as the pivot. Each set
				// includes its own max (the exp(0)=1 term).
				double sum0 = 0.0;
				double sum1 = 0.0;
				for (int m = 0; m < M; m++)
				{
					int gray_m = m ^ (m >> 1);
					if (gray_m & mask)
					{
						sum1 += std::exp((E[m] - max_E1) * llr_scale);
					}
					else
					{
						sum0 += std::exp((E[m] - max_E0) * llr_scale);
					}
				}
				// LSE(S_0) - LSE(S_1) where LSE is in LLR-domain (already
				// scaled by 1/sigma^2). Note: max_E and llr_scale combine to
				// (max_E0 - max_E1) * llr_scale as the dominant term, plus
				// the log() correction that reduces to 0 at high SNR.
				double llr = (max_E0 - max_E1) * llr_scale
				           + std::log(sum0) - std::log(sum1);
				if (!std::isfinite(llr)) llr = 0.0;
				// LLR cap removed 2026-05-24. The previous ±5 clip was
				// arbitrary defensive code with no rationale in the
				// original commit (28d8204, "Add MFSK ROBUST modes"). At
				// rate-1/16 LDPC (ROBUST_0), the SPA decoder needs the
				// full magnitude of high-confidence tone observations to
				// flip the ~94% parity-dominated codeword bits — clipping
				// to ±5 kept the floor ~15 dB above the original commit's
				// "-13 dB SNR" design target. The isfinite() guard above
				// handles infinities (only possible if noise_variance==0);
				// SPA's tanh/atanh internals naturally saturate, so
				// unbounded float LLRs are safe.
				llr_out[llr_offset + k] = (float)llr;
			}
		}
	}

	// Codeword SNR estimate (replaces the hardcoded MFSK SNR=0.0 placeholder at
	// the telecom-layer MFSK-decode SNR site). mean(E_peak) ≈ S + noise_var, so
	// the de-biased signal power is mean(E_peak) - noise_var; SNR = that / one
	// noise bin's energy. Clamp the post-subtraction signal to a small positive
	// floor so deep-noise (peak ≈ noise) reports a low — not NaN/-inf — SNR, and
	// clamp the final dB to a sane reporting range. Leaves the -99.0 sentinel
	// when the codeword carried no measurable symbol (peak_count_all==0).
	if (peak_count_all > 0)
	{
		double mean_peak = peak_sum_all / peak_count_all;
		double sig = mean_peak - noise_var;
		if (sig < 1e-30) sig = 1e-30;             // deep noise → very low SNR, not NaN
		double snr_db = 10.0 * std::log10(sig / noise_var);
		if (!std::isfinite(snr_db)) snr_db = -99.0;
		if (snr_db < -30.0) snr_db = -30.0;       // reporting floor
		if (snr_db >  60.0) snr_db =  60.0;       // reporting ceiling
		last_demod_snr_db = snr_db;
	}
}
