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

#ifndef INC_MFSK_H_
#define INC_MFSK_H_

#include <complex>
#include <cmath>
#include <cstdint>

#include "physical_layer/mfsk_ctrl_codec.h"  // mfsk_ctrl_frame_type enum

class cl_ldpc;

#define MOD_MFSK 200

class cl_mfsk
{
private:

public:
	int M;           // Number of tones per stream (e.g., 16 or 32)
	int nBits;       // log2(M) = bits per stream per symbol
	int Nc;          // Total subcarriers in OFDM frame (typically 50)
	int nStreams;    // Parallel MFSK streams (1=ROBUST_0, 2=ROBUST_1/ROBUST_2)
	int tone_hop_step; // Tone hopping step for frequency diversity (coprime with M)

	static const int MAX_STREAMS = 4;
	int stream_offsets[MAX_STREAMS]; // Starting subcarrier bin for each stream

	// MFSK preamble: known tone indices for time sync.
	// WB data preamble extended 4 -> 16 symbols on 2026-05-27 (data-flow-
	// preamble_nSymb.md §H1) for +6 dB matched-filter integration gain at
	// the WGN:-8 cliff. NB preamble (M=8 / M=4) stays at 8 symbols.
	// MAX_PREAMBLE_SYMB must be >= max(preamble_nSymb across all configs)
	// and >= mfsk_corr_template_sym_energy[] size in ofdm.h.
	static const int MAX_PREAMBLE_SYMB = 16;
	int preamble_tones[MAX_PREAMBLE_SYMB]; // Known tone indices per preamble symbol
	int preamble_nSymb;                     // Number of preamble symbols used
	// Length-scaled detection threshold for the discrete-match preamble
	// detector (`time_sync_mfsk_corr` ofdm.cc, post-2026-05-27 port per
	// fact-documents/data-preamble-port-research.md §14). Number of
	// per-symbol FFT-bin-argmax matches required to declare detection.
	// Mirrors ack_match_threshold / connect_match_threshold.
	// FAR ≈ 2.5e-7/poll at M=32 (7/16); ≈ 2.4e-5/poll at M=16.
	int preamble_match_threshold;

	// ACK/BREAK/HAIL/SACK pattern: known tone sequences for pattern-based signaling.
	// WB (M>=16): 8 Welch-Costas tones × 2 reps = 16 symbols, with tone hopping.
	// NB (M<=8): 32/48-element Sidelnikov sequences, no repetition, no hopping.
	// ACK = data acknowledged, BREAK = emergency downshift, HAIL = "I am Mercury" beacon.
	// Step 15: legacy SACK pattern (selective-ACK with bitmap suffix) removed —
	// OFDM SACK_RSP is the only SACK transport now.
	static const int MAX_ACK_TONES = 48;  // Max for M=4 NB (48 symbols)
	int ack_tones[MAX_ACK_TONES];
	int break_tones[MAX_ACK_TONES];
	int hail_tones[MAX_ACK_TONES];
	// CONNECT base pattern (Phase B Wave 1): Welch-Costas with primitive root
	// g=3 (distinct from ACK g=5, BREAK g=7, HAIL g=6). 8 base tones × 2
	// reps = 16 symbols (WB-only — NB CONNECT remains LDPC). The detector
	// reuses `ofdm.detect_ack_pattern(connect_tones, ...)`; tone hopping +
	// match threshold mirror ack_tones for shared decoder plumbing. See
	// fact-documents/phase-b-mfsk-connect-research.md §11.4.
	int connect_tones[MAX_ACK_TONES];
	int connect_pattern_nsymb;
	int connect_match_threshold;
	// Tier-2 base-pattern noncoherent COMBINING (tier2-suffix-fec-design.md §20,
	// INCREMENT 2). When connect_preamble_reps>1 the CONNECT handshake emits the
	// connect_pattern_nsymb base block R times (identical, per-rep-LOCAL hop) and
	// the RX detector (ofdm.detect_ack_pattern, combine_reps param) noncoherently
	// sums the per-symbol FFT energy across the R aligned reps BEFORE the
	// argmax/matched-count — deepening the base-pattern detection floor
	// ~+2.2-2.5 dB/doubling (measured: hail-detection-floor-investigation.md §4,
	// repetition sim §14). The suffix is NOT repeated (combining is on the
	// PREAMBLE/base, §14). connect_preamble_reps=1 (default) → R*16=16 → every
	// consumer byte-identical to pre-§20. CONNECT-only (ACK/BREAK/HAIL pass
	// combine_reps=1 and never read this). The base now occupies
	// connect_base_total_nsymb()=R*connect_pattern_nsymb symbols on the wire; the
	// suffix follows at that offset (the I4 length accessor for the base, §20.3).
	static const int MAX_CONNECT_PREAMBLE_REPS = 4;
	int connect_preamble_reps;   // default 1 (set in init())
	int connect_base_total_nsymb() const {
		int r = connect_preamble_reps;
		if (r < 1) r = 1;
		if (r > MAX_CONNECT_PREAMBLE_REPS) r = MAX_CONNECT_PREAMBLE_REPS;
		return r * connect_pattern_nsymb;
	}
	// Soft energy-ratio sub-gate for CONNECT-handshake / ACK-SNR MFSK suffix
	// decode (telecom_system.cc decode_ctrl_suffix_from_passband +
	// detect_ack_snr_from_passband). ofdm.detect_ack_pattern returns
	// metric = Σ(e_target/e_total) over matched symbols ∈ [0, pattern_nsymb];
	// this is the MINIMUM that metric must reach (in addition to the HARD
	// count gate matched>=*_match_threshold) for a detection to be admitted to
	// the ctrl-suffix decode.
	//
	// The HARD count gate (7/16, FAR≈2.4e-5..2.5e-7/poll, mfsk.cc:230-406) is
	// the load-bearing false-alarm defense; the downstream CRC12 (P≈2^-12) +
	// 2-bit type discriminator (×¼) on the CONNECT path, and the SNR-suffix
	// 3/8 majority + 2-vote-margin gate on the ACK-SNR path
	// (telecom_system.cc:3408), are the correctness defenses. This metric is
	// only a cheap pre-filter to skip the suffix decode on obvious noise.
	//
	// Was a hardcoded 3.0 at 4 sites. The metric falls monotonically with SNR
	// and crosses 3.0 at the −8.65 dB SNR3k cliff while decode_suffix_energies
	// (and the CRC-aided soft list) deliver P≈1.0 down to −14 — i.e. the gate,
	// NOT the content, was the SOLE ctrl-suffix masker (isolation sim, agent
	// a1fe962c, fact-documents/tier2-suffix-fec-design.md §16). The 3.0 gate
	// masks 59%/97%/100% of perfectly-decodable suffixes at −9.8/−10.8/−11.8.
	//
	// §16 cliff table (FEC-reach with the gate relaxed; base detector floor
	// −14.68): 2.0 reaches only −11.75; ~1.0–1.5 is needed to feed the floor.
	// Set to 1.2 = mid of the §16 [1.0,1.5] window with ~0.2 of FAR headroom
	// over the count gate's worst measured noise metric (1.207 past the 7/16
	// count gate, connect-ack-metric-gate.md §6). FAR-safe: §16 measured the
	// gate FULLY OFF = 0/4000 pure-noise false-accepts on the CRC-backstopped
	// path (Q65/FT8 precedent — the CRC, not a pre-decode energy threshold, is
	// the floor's FAR gate; Franke-Taylor QEX 2020), so 1.2 (> off) is strictly
	// safer than the measured-clean OFF case. §5 audit: ctrl-suffix-LOCAL — the
	// data OFDM demod uses a SEPARATE Schmidl-Cox/coarse detector
	// (ofdm.cc time_sync_preamble*, receive_stats.coarse_metric), so relaxing
	// this cannot affect data DEMOD; and in the good-SNR band where data flows
	// the metric stays ≥7 → gate decision identical to the old 3.0 (0
	// divergences) → THROUGHPUT-NEUTRAL. See §17.
	static constexpr double CTRL_DETECT_METRIC_MIN = 1.2;
	int ack_pattern_len;    // Base tone sequence length (8 for WB, 32/48 for NB)
	int ack_pattern_nsymb;  // Total symbols transmitted (16 for WB, 32/48 for NB)
	int ack_match_threshold;   // Min matched symbols for ACK detection

	// RECOVERY-ACK robustness (recovery-ack-robustness.md §4). The BREAK-recovery
	// reverse control-ACK lands at a marginal 6-7/16 on a CLEAN channel because a
	// turnaround timing straddle knocks 1-2 symbols off the HARD per-symbol
	// peak-bin decision (root cause §3). recovery_ack_reps>1 emits the
	// ack_pattern_nsymb base block R times (per-rep-LOCAL hop, IDENTICAL to
	// generate_connect_pattern's §20 layout) so the RX detector
	// (ofdm.detect_ack_pattern, combine_reps param) noncoherently sums per-symbol
	// FFT energy across the R aligned reps BEFORE the argmax/count — lifting a
	// straddled symbol's true-tone bin back over the peak (+2.2-2.5 dB/doubling,
	// hail §4). recovery_ack_reps=1 (default, ctor) → R*16=16 → BYTE-IDENTICAL to
	// pre-change. Gated at the call sites by MERCURY_RECOVERY_ACK_ROBUST.
	static const int MAX_RECOVERY_ACK_REPS = 4;
	int recovery_ack_reps;   // default 1 (set in init())
	int ack_base_total_nsymb() const {
		int r = recovery_ack_reps;
		if (r < 1) r = 1;
		if (r > MAX_RECOVERY_ACK_REPS) r = MAX_RECOVERY_ACK_REPS;
		return r * ack_pattern_nsymb;
	}
	int break_match_threshold; // Min matched symbols for BREAK detection
	// Min CORRELATION metric for BREAK detonation (WB M=16 only; 0 = inert for the
	// NB/other-M cases, which fall back to ack_pattern_detection_threshold). The BREAK
	// correlator metric scales to ~ack_pattern_nsymb for a perfect match: a genuine
	// BREAK burst lands metric~10-16, while a marginal-decode OFDM DATA frame whose
	// subcarrier argmax aliases the 8 WB break_tones lands matched>=10 but metric~1.0.
	// The shared ack_pattern_detection_threshold (1.0 on WB OFDM configs) does NOT
	// separate them; this dedicated floor sits in the ~10x gap so the alias is rejected
	// while a real BREAK is still accepted. See the BREAK OFDM-alias data-flow audit.
	double break_metric_threshold;
	int hail_match_threshold;  // Min matched symbols for undirected HAIL detection
	// Phase-2 validation: --wb-match-threshold-bias=N added to ack/break/hail
	// match thresholds for M=16 and M=32 (the WB cases). Default 0 = HEAD.
	// Pass +1 to revert b806b76+7076a4b's 8→7 reductions. Applied at end of
	// cl_mfsk::init() so it stacks with the computed defaults.
	int wb_match_threshold_bias;

	// Directed HAIL: 4-tone CRC suffix appended after the "I am Mercury" prefix.
	// Derived from FNV-1a hash of the target callsign (including SSID).
	// Only stations matching the suffix respond, preventing multi-station collisions.
	static const int HAIL_SUFFIX_LEN = 4;
	int hail_suffix[HAIL_SUFFIX_LEN];          // CRC-derived suffix tones
	bool hail_directed;                          // true when suffix is active
	int hail_detect_tones[MAX_ACK_TONES + HAIL_SUFFIX_LEN]; // flat expanded array for detection
	int hail_detect_nsymb;                       // total symbols (base + suffix when directed)
	int hail_detect_threshold;                   // adjusted threshold

	void set_hail_target(const char* callsign, int len);
	void clear_hail_target();

	// Step 15: legacy SACK bitmap-suffix helpers (sack_bitmap_nsuffix,
	// sack_total_nsymb, encode_sack_bitmap, decode_sack_bitmap,
	// generate_sack_pattern, generate_sack_bitmap_pattern, MAX_SACK_BITMAP_SYMBOLS)
	// removed — OFDM SACK_RSP replaces this entire path.

	// SNR suffix for turboshift ACK: 8 extra symbols encoding quantized SNR.
	// WB (M=16): tone 0-15 → SNR = tone*2 - 5 dB (range -5 to +25 dB, 2 dB step)
	// NB (M=8):  tone 0-7  → SNR = tone*2 - 9 dB (range -9 to +5 dB, 2 dB step)
	// All 8 symbols carry the same tone (majority vote on decode, need 3/8).
	static const int SNR_SUFFIX_LEN = 8;
	int snr_to_tone(float snr) const;
	float tone_to_snr(int tone) const;
	void generate_ack_snr_pattern(std::complex<double>* pattern_out, float snr);
	// Total symbols when SNR suffix is active
	int ack_snr_pattern_nsymb() const { return ack_pattern_nsymb + SNR_SUFFIX_LEN; }

	// MFSK control suffix: 13-symbol suffix at M=16 carrying
	//   [type:2 | payload:38 | crc12:12] = 52 bits.
	// Phase B Wave 1 (fact-doc §11) added the 2-bit type field — flag-day
	// break with pre-2026-05-26 deployed peers. Type values per
	// `mfsk_ctrl_frame_type`. CRC12 protects against false-accept after
	// pattern correlator lock (mercury/fact-documents/mfsk-robust-ack.md §3.2).
	//
	// Suffix length: NB M=8 returns 0 (deferred). WB M=16 → 4 bits/symbol
	// → 13 symbols for 52 bits. Total ACK pattern wall-clock: 16 base +
	// 13 suffix = 29 symbols ≈ 705 ms (WB).
	int ack_sack_suffix_len() const { return (M >= 16) ? 13 : 0; }  // 0 = unsupported
	int ack_sack_pattern_nsymb() const { return ack_pattern_nsymb + ack_sack_suffix_len(); }

	// Tier-2 suffix FEC (tier2-suffix-fec-design.md §19, INCREMENT 1). When
	// suffix_fec_coded is set (by the telecom layer after gf16ra::configure(3)+
	// init()), the CONNECT ctrl-suffix is encoded with the GF(16) RA code:
	// pack_ctrl_suffix emits gf16ra::codeword_len() (N, default 52 @ R=1/4)
	// tones instead of the 13-symbol hard pack, and RX decodes the per-tone
	// ENERGY matrix via gf16ra::soft_decode (decode-from-passband path only).
	// ctrl_suffix_len() is THE coded symbol count — the I4 length accessor every
	// CONNECT TX/RX site routes through (§19.4). suffix_fec_coded=false (the
	// default) → returns the uncoded 13 → byte-identical-when-off. NB (M<16)
	// always returns 0 (FEC deferred there). gf16ra owns the active N (it is
	// configured once at FEC enable); ctrl_suffix_len() never configures.
	bool suffix_fec_coded;   // default false (set in init()) — the CONNECT-suffix
	                          // FEC enable (CONNECT-path-LOCAL: ctrl_suffix_len() +
	                          // generate_ctrl_suffix_pattern + the CONNECT RX decode).
	// §21: the ACK-suffix FEC enable is SEPARATE from the CONNECT one so the data
	// ACK can NEVER inherit the CONNECT FEC state (the §21.1 bug — pack_ctrl_suffix
	// used to read the single global suffix_fec_coded, so an FEC-on CONNECT session
	// silently coded the data ACK to 52 tones while the ACK generator emitted only
	// 13 → garbled ACK at every OFDM SNR). Default false = byte-identical ACK. The
	// ARQ layer sets it per-batch ONLY when ack_suffix_fec_eligible() (robust tier
	// — a throughput gate, not a negotiation); held off (§21.3) → always 13-tone uncoded.
	bool ack_suffix_fec_coded;   // default false (set in init())
	int ctrl_suffix_len() const {
		if (ack_sack_suffix_len() <= 0) return 0;          // NB: unsupported either way
		return suffix_fec_coded ? gf16ra::codeword_len() : ack_sack_suffix_len();
	}
	// Generic ctrl-suffix codec (52-bit [type:2|payload:38|crc12:12]). §21: `fec`
	// is an EXPLICIT per-call argument (no longer the global suffix_fec_coded) so
	// each caller decides independently — CONNECT TX passes suffix_fec_coded, the
	// ACK packer passes ack_suffix_fec_coded. fec=true emits the GF(16) RA codeword
	// (gf16ra::codeword_len() tones); fec=false emits the 13-symbol hard bit-pack.
	int pack_ctrl_suffix(mfsk_ctrl_frame_type type, uint64_t payload38,
	                     uint16_t crc12, int* out_tones, bool fec) const;
	bool unpack_ctrl_suffix(const int* in_tones,
	                        mfsk_ctrl_frame_type* out_type,
	                        uint64_t* out_payload38,
	                        uint16_t* out_crc12) const;
	// Backward-named ACK+SACK wrappers — delegate to pack/unpack_ctrl_suffix
	// with type=MFSK_CTRL_ACK_SACK. The bitmap field is now 30 bits (down
	// from 32 in pre-2026-05-26 deployments); bits 30/31 are silently
	// dropped with a stderr warning. data_batch_size <= 30 is the new
	// invariant — verified at the producer side in arq_responder.cc.
	int pack_ack_sack_payload(uint8_t bsi, uint32_t bitmap, uint16_t crc12,
	                          int* out_tones) const;
	bool unpack_ack_sack_payload(const int* in_tones, uint8_t* out_bsi,
	                             uint32_t* out_bitmap, uint16_t* out_crc12) const;
	// Generate ACK pattern + ack_sack_suffix_len() suffix symbols.
	// Caller passes a pre-computed crc12 (12-bit CRC over [type|bsi||bitmap]
	// packed as 5 bytes [type<<6|bsi>>2, (bsi<<6)|(bitmap>>24), ...]; or
	// equivalently CRC12 over the full 40-bit [type:2|bsi:8|bitmap:30]
	// big-endian MSB-justified). For compatibility with the existing
	// Wave-1 ARQ callers (which compute CRC12 over [bsi||bitmap] = 5 bytes),
	// see mercury/fact-documents/phase-b-mfsk-connect-research.md §11.3.
	void generate_ack_sack_pattern(std::complex<double>* pattern_out,
	                               uint8_t bsi, uint32_t bitmap,
	                               uint16_t crc12);
	// Option B compact confirm: ACK base (ack_pattern_nsymb) + the K=5 GF(16)-RA
	// compact codeword (gf16ra::compact_codeword_len() = 10 sym) carrying
	// [bsi:8|crc12:12]. Same tone-hop continuation as generate_ack_sack_pattern
	// (abs_s = ack_pattern_nsymb + g). crc12 = caller-supplied CRC12 over [bsi].
	// Length: compact_confirm_pattern_nsymb() = ack_pattern_nsymb + 10.
	int  compact_confirm_suffix_len() const {
		return (M >= 16) ? gf16ra::compact_codeword_len() : 0;   // 0 = NB unsupported
	}
	int  compact_confirm_pattern_nsymb() const {
		return ack_pattern_nsymb + compact_confirm_suffix_len();
	}
	void generate_compact_confirm_pattern(std::complex<double>* pattern_out,
	                                      uint8_t bsi, uint16_t crc12);
	// Exact compact-confirm prefix followed by a second K=5 codeword carrying
	// [report:8|crc12(report):12]. Old peers consume only the unchanged prefix.
	void generate_topgear_confirm_pattern(std::complex<double>* pattern_out,
	                                      uint8_t bsi, uint16_t crc12,
	                                      uint8_t report, uint16_t report_crc12);
	// Generate CONNECT base pattern + 13-symbol ctrl-suffix carrying
	// (type, payload, crc12). The base pattern uses connect_tones (NOT
	// ack_tones) so the detector can distinguish CONNECT from ACK+SACK.
	// Caller supplies crc12 (computed over the packed [type:2|payload:38]).
	void generate_ctrl_suffix_pattern(std::complex<double>* pattern_out,
	                                  mfsk_ctrl_frame_type type,
	                                  uint64_t payload38, uint16_t crc12);

	// In-band rate adaptation (Stage 3a) — CONFIG_TAG keying.
	// MIRROR of generate_ctrl_suffix_pattern: same CONNECT base pattern + a
	// one-tone-per-symbol suffix, BUT the suffix tones come from the
	// gf16ra::encode_config_tag codeword (N = gf16ra::codeword_len() symbols,
	// the configure(2)=39 R=1/3 substrate) rather than pack_ctrl_suffix. The
	// CONFIG_TAG uses the [type:3|payload:37] split (so MFSK_CTRL_CONFIG_TAG=4
	// survives the type field), which pack_ctrl_suffix does NOT understand — hence
	// a dedicated keyer. `n_suffix` is gf16ra::codeword_len(); `tones` is that
	// many GF(16) tones (0..M-1). The base is identical to the ctrl-suffix base so
	// the SAME detect_ack_pattern correlator locates it. Stage 3a is a SELF-
	// CONTAINED robust burst; the in-line append into send_batch is Stage 3b.
	void generate_config_tag_mfsk_pattern(std::complex<double>* pattern_out,
	                                       const int* tones, int n_suffix);

	// In-band rate adaptation (Stage 3c) — CONFIG_TAG ACQUISITION-SYNC TRIM.
	// =====================================================================
	// The CONFIG_TAG burst (generate_config_tag_mfsk_pattern) prepends a base
	// acquisition-sync pattern so the RX base-correlator (detect_ack_pattern on
	// connect_tones) can LOCATE the burst. A free-standing CONNECT burst must be
	// blind-located, so it carries a FULL connect_pattern_nsymb (=16) base block.
	// But after Stage 3b the tag rides at a DETERMINISTIC OFFSET — keyed right
	// after the first OFDM DATA frame of a batch — so the RX
	// (decode_config_tag_from_passband, reading the capture-tail) already knows
	// approximately WHERE the burst is. A known-offset acquire needs FAR fewer
	// base-sync symbols than a cold blind acquire, so the acquisition sync can be
	// TRIMMED. THIS IS THE ONLY PART THAT SHRINKS — the 55-symbol payload suffix
	// (RM(1,4) FWHT cfg_index + GF(16) RA binding/FEC + CRC-12) is UNTOUCHED.
	//
	// config_tag_sync_nsymb() returns the number of base-sync symbols the tag
	// burst emits (TX) and the RX correlator searches for, default
	// CFG_TAG_SYNC_NSYMB_DEFAULT (the swept minimum, see Stage-3c verdict), clamped
	// to [1, connect_pattern_nsymb]. Env MERCURY_INBAND_TAG_SYNC_REPS overrides it
	// (cached once; named "REPS" historically — the base block was reps×16 — but it
	// now directly sets the base-sync symbol COUNT, the real airtime lever). The
	// tag base uses combine_reps=1 (no noncoherent rep-combining — the deterministic
	// offset replaces rep-integration as the acquisition aid), so it is DECOUPLED
	// from connect_preamble_reps: the CONNECT handshake's hardware-validated
	// reps×16 acquisition is UNCHANGED.
	//
	// CROSS-LAYER: this is tag-path-LOCAL. connect_pattern_nsymb,
	// connect_preamble_reps, connect_match_threshold, and connect_base_total_nsymb()
	// are NOT modified — only the CONFIG_TAG TX keyer + RX decoder consult
	// config_tag_sync_nsymb()/config_tag_sync_match_threshold(). The CONNECT
	// ctrl-suffix path, ACK/SACK, BREAK, HAIL all keep their full base.
	int config_tag_sync_nsymb() const;
	// The matched-count gate for the tag base, scaled from connect_match_threshold
	// by the base-length ratio (ceil), floored at CFG_TAG_SYNC_MATCH_MIN so a 1- or
	// 2-symbol base can't be trivially false-triggered. At the full 16-symbol base
	// this returns connect_match_threshold (= the CONNECT gate, byte-identical).
	int config_tag_sync_match_threshold() const;
	// Emit `nsymb` base-sync symbols (a prefix of the connect base sequence, NO
	// rep-combining) at the FRONT of pattern_out. The per-symbol-LOCAL hop index is
	// `s` (matching detect_ack_pattern's `p` indexing), so the RX correlator —
	// searching the SAME connect_tones with combine_reps=1 — aligns symbol-for-symbol.
	void generate_config_tag_base(std::complex<double>* pattern_out, int nsymb);
	// The minimum base length that holds detection on the cliff (Stage-3c sweep,
	// test_config_tag_sync_trim_sweep). The sweep MEASURED, at the op Es/N0 (6 dB)
	// with +/-50% symbol timing jitter: len=16/12/10 all hold >=99% detect+accept,
	// len=8 drops to ~98%/~95%, len<=6 falls off. 10 is the swept minimum that holds
	// >=99% on BOTH detect and accept (8.5% burst-size reduction vs the full 16; the
	// 55-symbol payload suffix dominates the burst, so the sync trim is bounded).
	static const int CFG_TAG_SYNC_NSYMB_DEFAULT = 10;  // swept minimum (>=99% @ op SNR + jitter)
	static const int CFG_TAG_SYNC_MATCH_MIN     = 4;   // count-gate floor (FAR backstop)
	// Test-only override of config_tag_sync_nsymb() (the env cache is process-static,
	// so the Stage-3c sweep — which must vary the base length across iterations in one
	// process — sets this instead). -1 = unset (use env/default). NEVER set in
	// production; the sweep restores it to -1 when done.
	int tag_sync_nsymb_override = -1;

	// RX-side capture buffer populated by the ACK detector hook
	// (cl_telecom_system::detect_ack_snr_from_passband). Each entry is the
	// de-hopped payload tone (0..M-1) for the corresponding SACK suffix
	// symbol — i.e. the inverse of the (payload+abs_s*hop)%M mapping the
	// transmitter applies in generate_ack_sack_pattern(). When the detector
	// declares an ACK match it writes ack_sack_suffix_len() entries here
	// (10 for WB M=16) and sets last_ack_sack_capture_valid=true.
	// Sized to the Tier-2 FEC ceiling (gf16ra::GF16RA_MAX_N = 64 ≥ the R=1/4
	// coded length 52, tier2-suffix-fec-design.md §19.4 C7) so the
	// last_*_suffix_tones[] / suffix_tones[] / payload_tones[] buffers that
	// derive their size from this constant are safe whether the uncoded (13)
	// or coded (52) ctrl-suffix path runs. Was 16 (uncoded-only).
	static const int MAX_ACK_SACK_SUFFIX = 64;
	int  last_ack_sack_suffix_tones[MAX_ACK_SACK_SUFFIX];
	bool last_ack_sack_capture_valid;

	// CONNECT-suffix capture (separate from ACK+SACK so the two detector
	// windows can coexist without aliasing).
	int  last_connect_suffix_tones[MAX_ACK_SACK_SUFFIX];
	bool last_connect_capture_valid;

	// Codeword SNR estimate (dB) from the most recent demod() call —
	// noncoherent-FSK "peak tone energy vs noise energy" measurement (the
	// long-standing TODO at telecom_system.cc MFSK-decode SNR site). demod()
	// already pools the guard-bin noise variance across the whole codeword
	// (mfsk.cc, `noise_var`) to scale its LLRs; this member additionally
	// accumulates the per-symbol-per-stream PEAK tone energy and reports
	//   SNR_dB = 10*log10( max(mean(E_peak) - noise_var, eps) / noise_var ).
	// The peak bin carries (signal + noise), so subtracting one noise-bin's
	// energy de-biases the signal estimate (Proakis 5th ed §4.5.4 noncoherent
	// FSK; matches the LSE metric demod() already uses). Init -99.0 (the
	// "no measurement" sentinel the ARQ SNR consumers treat as `<= -90`);
	// set to a real value on every MFSK codeword decode. Read by
	// cl_telecom_system::receive_byte's MFSK branch into receive_stats.SNR,
	// which flows up to measurements.SNR_uplink/_downlink (the connect-plane
	// SNR + the SUPERSHIFT elevator input). See
	// fact-documents/data-flow-snr-measurements.md §9.
	double last_demod_snr_db;

	// Decode the most-recent CONNECT-suffix capture into (type, payload,
	// crc12). Returns true on success — requires WB (M>=16) and a prior
	// CONNECT-pattern hit that populated last_connect_suffix_tones[].
	// Caller verifies crc12 separately by recomputing CRC12 over
	// [type:2|payload:38] packed as 5 bytes.
	bool decode_ctrl_suffix_from_last_capture(mfsk_ctrl_frame_type* out_type,
	                                          uint64_t* out_payload38,
	                                          uint16_t* out_crc12);

	// Test-only: stuff CONNECT-suffix payload tones directly into the
	// capture buffer (bypasses RF). Mirror of test_inject_ack_sack_capture.
	void test_inject_connect_capture(const int* tones, int count);

	// Decode the most recently captured SACK suffix into (bsi, bitmap, crc12).
	// Returns true on success — requires WB (M>=16) and a prior detector
	// hit that populated last_ack_sack_suffix_tones[]. Caller is expected
	// to clear last_ack_sack_capture_valid when consumed AND to verify
	// that the returned crc12 matches a freshly-computed CRC12 over
	// [bsi || bitmap]; this function performs the bit-level unpack only.
	bool decode_ack_sack_from_last_capture(uint8_t* out_bsi, uint32_t* out_bitmap,
	                                       uint16_t* out_crc12);

	// Test-only: stuff payload tones directly into the capture buffer
	// (bypasses the RF capture path). Used by symbol-domain round-trip
	// tests so we can exercise decode_ack_sack_from_last_capture() without
	// running passband_to_baseband + FFT. Not for production code paths.
	void test_inject_ack_sack_capture(const int* tones, int count);

	cl_mfsk();
	~cl_mfsk();

	// PRECOOK M3 (BUNDLE_FIELD_CHECKLIST PART 3): cl_mfsk has NO owning pointers
	// (all scalars + fixed arrays), so a memberwise copy is correct. Provided for
	// M3 uniformity with the other geometry classes.
	void copy_from(const cl_mfsk& s) { *this = s; }

	void init(int _M, int _Nc, int _nStreams = 1);
	void deinit();

	// Effective bits per symbol period (nBits * nStreams)
	int bits_per_symbol() const { return nBits * nStreams; }

	// Generate MFSK preamble data (tones in all streams simultaneously)
	// preamble_out: nSymb * Nc complex values
	void generate_preamble(std::complex<double>* preamble_out, int nSymb);

	// Generate ACK pattern: ack_pattern_nsymb symbols of known tones
	// pattern_out: ack_pattern_nsymb * Nc complex values
	void generate_ack_pattern(std::complex<double>* pattern_out);

	// RECOVERY-ACK robustness (recovery-ack-robustness.md §4): emit the ACK base
	// block ack_base_total_nsymb() (= recovery_ack_reps * ack_pattern_nsymb)
	// symbols. Each rep is IDENTICAL — symbol s of every rep carries the SAME
	// tone (per-rep-LOCAL hop index s, NOT a continued abs index) so the RX
	// detector can sum rep-r symbol s onto rep-0 symbol s (same expected bin).
	// reps=1 → exactly generate_ack_pattern's single 16-symbol block
	// (byte-identical). pattern_out: ack_base_total_nsymb() * Nc complex values.
	void generate_ack_pattern_reps(std::complex<double>* pattern_out);

	// Generate BREAK pattern: same structure as ACK but with break_tones
	void generate_break_pattern(std::complex<double>* pattern_out);

	// Generate HAIL pattern: "I am Mercury" beacon, same structure as ACK but with hail_tones
	void generate_hail_pattern(std::complex<double>* pattern_out);

	// Generate CONNECT base pattern: same structure as ACK but with
	// connect_tones (Phase B Wave 1). Only the base 16 symbols are
	// written — the per-frame 13-tone ctrl-suffix is written by
	// generate_ctrl_suffix_pattern() above.
	void generate_connect_pattern(std::complex<double>* pattern_out);

	// TX: Map bits to one-hot subcarrier vectors across all streams
	// Consumes bits_per_symbol() bits per symbol period
	void mod(const int* bits_in, int total_bits,
	         std::complex<double>* symbols_out);

	// RX: Non-coherent energy detection across all streams -> soft LLRs
	// Produces bits_per_symbol() LLRs per symbol period
	void demod(const std::complex<double>* fft_in, int total_bits,
	           float* llr_out);
};

#endif
