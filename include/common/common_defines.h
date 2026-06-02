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

#ifndef INC_COMMON_DEFINES_H_
#define INC_COMMON_DEFINES_H_

#define VERSION__ "0.4.2"

// Compile-time gate for the MFSK ACK+SACK signaling (WB-only). RSP sends
// ACK/SACK via the MFSK pattern + 52-bit suffix [bsi:8 | bitmap:32 |
// crc12:12] and CMD listens for it. When 0, falls back to the legacy
// MFSK ACK pattern (no SACK; receiver implicitly treats any hit as a
// clean batch ACK). Both peers MUST agree (deployed together — no
// runtime negotiation). See mercury/fact-documents/mfsk-robust-ack.md.
#ifndef MFSK_ACK_SACK_ENABLED
#define MFSK_ACK_SACK_ENABLED 1
#endif

// §21 (tier2-suffix-fec-design.md §21.3): MASTER ENABLE for the
// robust-tier ACK FEC. The per-batch ACK enhanced-suffix is gated on
// ack_suffix_fec_eligible() (robust tier — a throughput gate, not a capability
// negotiation; CAP_SUFFIX_FEC was removed in cleanup/drop-suffix-fec-cap), but the
// actual TX/RX ENABLE is held OFF here so the data ACK is byte-identical in 100%
// of cases (the hard throughput-neutrality constraint), not just at CONFIG_6+.
// Rationale: (a) the §20.9 HW result shows the ACK was never the establishment
// limiter, so robust-tier ACK FEC buys nothing on the validated bottleneck yet;
// (b) enabling it requires the ACK TX/RX window to be sized for the 52-tone coded
// suffix (today decode_ack_sack_from_passband reads 13 — §21.4 audit defers this
// ACK-sizing work). Flip to 1 ONLY after the ACK coded-window sizing lands AND a
// dedicated HW A/B. The gate predicate + plumbing are wired + tested so this is a
// one-flag follow-on.
#ifndef ARQ_ACK_SUFFIX_FEC_ENABLE
#define ARQ_ACK_SUFFIX_FEC_ENABLE 0
#endif

// Verbose debug output (0=quiet, 1=debug prints enabled). Set via -v flag.
extern int g_verbose;

#define BER_PLOT_baseband 0
#define BER_PLOT_passband 1
#define TX_RAND 2
#define RX_RAND 3
#define TX_TEST 4
#define RX_TEST 5
#define TX_SHM 6
#define RX_SHM 7
#define ARQ_MODE 8
#define MONITOR_MODE 9

#define NUMBER_OF_CONFIGS 17
#define CONFIG_NONE -1
#define CONFIG_0 0
#define CONFIG_1 1
#define CONFIG_2 2
#define CONFIG_3 3
#define CONFIG_4 4
#define CONFIG_5 5
#define CONFIG_6 6
#define CONFIG_7 7
#define CONFIG_8 8
#define CONFIG_9 9
#define CONFIG_10 10
#define CONFIG_11 11
#define CONFIG_12 12
#define CONFIG_13 13
#define CONFIG_14 14
#define CONFIG_15 15
#define CONFIG_16 16

// ROBUST (MFSK) configurations - values 100+ to avoid collision with OFDM configs
#define NUMBER_OF_ROBUST_CONFIGS 3
#define ROBUST_0 100  // 32-MFSK, LDPC rate 1/16, ~14 bps (hailing mode)
#define ROBUST_1 101  // 16-MFSK x2, LDPC rate 1/16, ~22 bps
#define ROBUST_2 102  // 16-MFSK x2, LDPC rate 1/4,  ~87 bps

inline bool is_robust_config(int config) { return config >= 100 && config <= 102; }

// ULTRA (deep-SNR "survival") configurations - values 200+ (ULTRA tier, the
// road DOWN from ROBUST is more noncoherent). The ULTRA tier reuses the
// ROBUST_0-class MFSK data PHY (M=32 WB / M=8 NB, single stream, LDPC 1/16) but
// reaches deeper via BAUD-SCALING: each rung carries a baud multiplier K that
// scales Nfft (= 256*K) → a K* longer coherent MFSK symbol (the FST4/Q65
// time-bandwidth lever, +~3 dB/2x, NO noncoherent-combining loss). This REPLACES
// the INCR-1/2 establishment-suffix REPETITION (frontier §1/§2: repetition on an
// already-coded PHY is the inefficient anomaly). Depth = Nfft; the establishment
// path stays the single low-rate code the ROBUST tier ships (repfact=3/K_info=13),
// combining OFF (R_base=R_suffix=R_frame=1). Baud-scaling deepens BOTH the data
// PHY cliff AND the CONNECT establishment (same Nfft FFT window).
//
// K-mapping (baud-fading-spike.md §5, validated K in {1,2,4,8}; ULTRA_0=deepest
// per the 0=deepest convention):
//   ULTRA_0 200  K=8  Nfft=2048  ~-21 dB SNR3k (poor fading)  aggressive cap
//   ULTRA_1 201  K=4  Nfft=1024  ~-19/-20 dB                  SAFE deepest workhorse
//   ULTRA_2 202  K=2  Nfft=512   ~-16 dB
//   ULTRA_3 203  K=1  Nfft=256   ~-13 dB                      bridges ROBUST_0 (-13/-14)
//   ULTRA_4 204  K=16 Nfft=4096  (RESERVED, not built — unvalidated past the K=8
//                                 coherence/coding-saturation wall; P1 follow-on)
// SELECTABLE this increment via -s 200/201/202/203 (pin); gearshift entry +
// sticky-hysteresis is a later increment (P3c). NO CAP_ULTRA / no negotiation
// (backward-compatible by construction). The per-rung params (K, repfact, K_info,
// R_base, R_suffix, R_frame) live in cl_telecom_system::ultra_tier_suffix_params
// (the sole owner of the ULTRA PHY numbers). See fact-documents/
// per-config-nfft-ultra-rungs.md for the full design + the §5 Nfft audit.
#define NUMBER_OF_ULTRA_CONFIGS 4
#define ULTRA_0 200   // K=8 (Nfft=2048) -> data+establishment cliff ~-21 dB SNR3k (poor fading)
#define ULTRA_1 201   // K=4 (Nfft=1024) -> ~-19/-20 dB (SAFE deepest; fading-favorable)
#define ULTRA_2 202   // K=2 (Nfft=512)  -> ~-16 dB
#define ULTRA_3 203   // K=1 (Nfft=256)  -> ~-13 dB (bridges ROBUST_0)
#define ULTRA_4 204   // K=16 RESERVED (not built this increment)

inline bool is_ultra_config(int config) { return config >= 200 && config <= 203; }
inline bool is_ofdm_config(int config) { return config >= 0 && config <= 16; }

// §21 (tier2-suffix-fec-design.md): the base-pattern noncoherent combining factor
// for the PRODUCTION enhanced CONNECT suffix at the robust tier. R=4 is the §20
// sim/HW-validated operating point (= cl_mfsk::MAX_CONNECT_PREAMBLE_REPS); R=1
// would be combining-off. CONNECT is once/session so the +26.8% airtime of R=4 is
// negligible (§4). Only applied when is_robust_config(current) — OFDM stays R=1.
#define CONNECT_PREAMBLE_REPS_PROD 4

// NB mode cap — CONFIG_14 (8PSK, LDPC 14/16) is the highest feasible NB config.
// 16QAM/32QAM (CONFIG_15+) require accurate amplitude equalization that NB's
// sparse pilot grid (Nc=10, Dy=3) cannot provide with sufficient accuracy.
#define NB_CONFIG_MAX CONFIG_14

// WB mode cap — CONFIG_16 (32QAM) re-enabled with SACK Design A.
// History: CONFIG_16 was disabled because control frames decode at much lower
// SNR than data frames, causing false turboshift ceilings. Data frames at
// CONFIG_16 showed <12% success even at 20 dB SNR on real channels.
// SACK Design A (partial-batch SACK_RSP recovery) directly addresses this:
// even at 12% per-frame success, the bitmap-driven retransmit recovers the
// failed frames at one retransmit slot each instead of dropping the whole
// config. The cliff effect that made CONFIG_16 "fragile" no longer applies
// — the protocol is now resilient to per-frame loss. CONFIG_16 PHY 5665 bps
// is +30% over CONFIG_15's 4361 bps; with text compression, application
// throughput ceiling ≈ 13 kbps decompressed.
#define WB_CONFIG_MAX CONFIG_16

// Unified config ladder for gearshift (ROBUST → OFDM)
// CONFIG_16 re-added (see WB_CONFIG_MAX comment above).
static const int FULL_CONFIG_LADDER[] = {
	ROBUST_0, ROBUST_1, ROBUST_2,
	CONFIG_0, CONFIG_1, CONFIG_2, CONFIG_3, CONFIG_4, CONFIG_5, CONFIG_6,
	CONFIG_7, CONFIG_8, CONFIG_9, CONFIG_10, CONFIG_11, CONFIG_12,
	CONFIG_13, CONFIG_14, CONFIG_15, CONFIG_16
};
static const int FULL_CONFIG_LADDER_SIZE = 20;

inline int config_ladder_index(int config) {
	for (int i = 0; i < FULL_CONFIG_LADDER_SIZE; i++) {
		if (FULL_CONFIG_LADDER[i] == config) return i;
	}
	return -1;
}

inline int config_ladder_up(int config, bool robust_enabled, bool narrowband = false) {
	int ceiling = narrowband ? NB_CONFIG_MAX : WB_CONFIG_MAX;
	if (!robust_enabled) {
		return (config < ceiling) ? config + 1 : config;
	}
	int idx = config_ladder_index(config);
	if (idx < 0) return config;
	int next_idx = idx + 1;
	if (next_idx >= FULL_CONFIG_LADDER_SIZE) return config;
	int next = FULL_CONFIG_LADDER[next_idx];
	if (is_ofdm_config(next) && next > ceiling) return config;
	return next;
}

inline int config_ladder_up_n(int config, int steps, bool robust_enabled, bool narrowband = false) {
	int ceiling = narrowband ? NB_CONFIG_MAX : WB_CONFIG_MAX;
	if (!robust_enabled) {
		int target = config + steps;
		return (target < ceiling) ? target : ceiling;
	}
	int idx = config_ladder_index(config);
	if (idx < 0) return config;
	int next_idx = idx + steps;
	if (next_idx >= FULL_CONFIG_LADDER_SIZE) next_idx = FULL_CONFIG_LADDER_SIZE - 1;
	int next = FULL_CONFIG_LADDER[next_idx];
	if (is_ofdm_config(next) && next > ceiling)
		return ceiling;
	return next;
}

inline int config_ladder_down(int config, bool robust_enabled) {
	if (!robust_enabled) {
		return (config > CONFIG_0) ? config - 1 : config;
	}
	int idx = config_ladder_index(config);
	if (idx > 0) return FULL_CONFIG_LADDER[idx - 1];
	return config;
}

inline int config_ladder_down_n(int config, int steps, bool robust_enabled) {
	if (!robust_enabled) {
		int target = config - steps;
		return (target > CONFIG_0) ? target : CONFIG_0;
	}
	int idx = config_ladder_index(config);
	idx -= steps;
	if (idx < 0) idx = 0;
	return FULL_CONFIG_LADDER[idx];
}

inline bool config_is_at_top(int config, bool robust_enabled, bool narrowband = false) {
	if (is_ofdm_config(config)) {
		int ceiling = narrowband ? NB_CONFIG_MAX : WB_CONFIG_MAX;
		return config >= ceiling;
	}
	if (!robust_enabled) return config == WB_CONFIG_MAX;
	return config_ladder_index(config) == FULL_CONFIG_LADDER_SIZE - 1;
}

inline bool config_is_at_bottom(int config, bool robust_enabled) {
	if (!robust_enabled) return config == CONFIG_0;
	return config_ladder_index(config) == 0;
}

// SESSION-START value of the data-viability anchor (last_data_viable_config).
// gearshift-climb-engine.md §17 — the anchor means "highest config with CONFIRMED
// data delivery"; at session start NOTHING above the floor is proven, so it must
// init to the session's actual FLOOR/start config, NOT the stale
// init_configuration-defaults-to-CONFIG_0 value (which is CONFIG_0 at ctor time —
// before init() resolves init_configuration — and on a GUI build whose
// g_settings.initial_config is not robust). Computing the floor from robust_enabled
// (NOT from init_configuration) closes BOTH poison mechanisms (the ctor-ordering
// hole AND the GUI-build init_configuration==CONFIG_0 path):
//   - robust/-R session  -> the ladder FLOOR (ROBUST_0 = FULL_CONFIG_LADDER[0]).
//     is_ofdm_config(ROBUST_0)=false, so the §15 SUPERSHIFT re-trigger gate is
//     CLOSED at t=0 (no multi-rung jump until an OFDM batch is PROVEN) and
//     break_target_with_anchor can floor recovery all the way to ROBUST_0.
//   - non-robust session -> the operating/start config (init_configuration:
//     CONFIG_0 for a normal start, or a pinned CONFIG_N). is_ofdm_config(CONFIG_N)
//     is true — correct: a pinned/normal OFDM session legitimately starts in the
//     OFDM tier and BREAK must floor there, not below it.
// PURE; no side effects. Production calls it at the three init sites (ctor,
// init(), reset_session_state — arq_common.cc); the unit test (Part M) replays it
// directly.
inline int session_floor_anchor(bool robust_enabled, int start_config) {
	if (robust_enabled) return FULL_CONFIG_LADDER[0];  // ROBUST_0 — ladder floor
	return start_config;
}

// Returns the modulation type for an OFDM config (MOD_BPSK=2, MOD_QPSK=4, etc.)
// Used by monitor opportunistic decoder to detect same-modulation config switches
// (which preserve the audio buffer) vs cross-modulation switches (which destroy it).
inline int modulation_for_ofdm_config(int config) {
	if (config >= CONFIG_0  && config <= CONFIG_6)  return 2;  // MOD_BPSK
	if (config >= CONFIG_7  && config <= CONFIG_9)  return 4;  // MOD_QPSK
	if (config >= CONFIG_10 && config <= CONFIG_11) return 8;  // MOD_8PSK
	if (config == CONFIG_12)                        return 4;  // MOD_QPSK
	if (config == CONFIG_13)                        return 8;  // MOD_8PSK (12/16)
	if (config == CONFIG_14)                        return 8;  // MOD_8PSK (14/16)
	if (config == CONFIG_15)                        return 16; // MOD_16QAM
	if (config == CONFIG_16)                        return 32; // MOD_32QAM
	return -1;  // Not an OFDM config
}

/*
 * Config	CODE	Mode	EsN0(FER<0,1)
0	BPSK 	1/16	BPSK 1/16	-10
1	BPSK 	2/16	BPSK 2/16	-7,5
2	BPSK 	3/16	BPSK 3/16	-6
3	BPSK 	4/16	BPSK 4/16	-4,5
4	BPSK 	5/16	BPSK 5/16	-3,5
5	BPSK 	6/16	BPSK 6/16	-2,5
6	BPSK 	8/16	BPSK 8/16	-1,5
7	QPSK 	5/16	QPSK 5/16	-0,5
8	QPSK 	6/16	QPSK 6/16	0,5
9	QPSK 	8/16	QPSK 8/16	1,5
10	8PSK 	6/16	8PSK 6/16	3
11	8PSK 	8/16	8PSK 8/16	4
12	QPSK 	14/16	QPSK 14/16	6,5
13	16QAM 	8/16	16QAM 8/16	7,5
14	8PSK 	14/16	8PSK 14/16	9
15	16QAM 	14/16	16QAM 14/16	12,5
16	32QAM 	14/16	32QAM 14/16	13,5


HIGH_DENSITY PILOTS

CONFIG_0 (71.3 bps).
CONFIG_1 (156.1 bps).
CONFIG_2 (241.0 bps).
CONFIG_3 (325.8 bps).
CONFIG_4 (410.6 bps).
CONFIG_5 (495.5 bps).
CONFIG_6 (665.2 bps).
CONFIG_7 (762.6 bps).
CONFIG_8 (920.2 bps).
CONFIG_9 (1235.3 bps).
CONFIG_10 (1353.7 bps).
CONFIG_11 (1818.1 bps).
CONFIG_12 (2261.4 bps).
CONFIG_13 (2470.6 bps).
CONFIG_14 (3389.7 bps).
CONFIG_15 (4361.3 bps).
CONFIG_16 (5664.7 bps).


LOW DENSITY PILOTS

CONFIG_0 (84.2 bps).
CONFIG_1 (184.5 bps).
CONFIG_2 (284.8 bps).
CONFIG_3 (385.0 bps).
CONFIG_4 (485.3 bps).
CONFIG_5 (585.6 bps).
CONFIG_6 (786.1 bps).
CONFIG_7 (889.7 bps).
CONFIG_8 (1073.5 bps).
CONFIG_9 (1441.2 bps).
CONFIG_10 (1353.7 bps).
CONFIG_11 (1818.1 bps).
CONFIG_12 (2654.7 bps).
CONFIG_13 (2882.4 bps).
CONFIG_14 (3389.7 bps).
CONFIG_15 (5088.2 bps).
CONFIG_16 (5664.7 bps).


 *
 */


// messages definition
#define FIRST_MESSAGE 0
#define MIDDLE_MESSAGE 1
#define FLUSH_MESSAGE 2
#define SINGLE_MESSAGE 3
#define NO_FILTER_MESSAGE 4

// supported radios
#define RADIO_SBITX 0
#define RADIO_STOCKHF 1

// {TX,RX}_SHM shared memory interface
#define SHM_PAYLOAD_BUFFER_SIZE 131072
#define SHM_PAYLOAD_NAME "/mercury-comm"

// audio buffers shared memory interface
// 1536000 * 8
#define AUDIO_PAYLOAD_BUFFER_SIZE 12288000
#define AUDIO_CAPT_PAYLOAD_NAME "/audio-capt"
#define AUDIO_PLAY_PAYLOAD_NAME "/audio-play"


// Gear shifting modes
#define NO_GEAR_SHIFT 0
#define GEAR_SHIFT_ENABLED 1
// #define NO_GEAR_SHIFT_LADDER 2
// #define NO_GEAR_SHIFT_SNR 3

// SNR-based supershift margin: subtract this from measured SNR before config lookup.
// Lands ~2-3 configs below the edge so the effective-rate optimizer (Q-table)
// has room to ratchet UP to the actual best config from a known-working start.
// Was 3.0 dB (lands at the cliff edge) — caused SUPERSHIFT to plant us at
// CFG16 in conditions where CFG16 reliably cliffs (e.g., wgn18-22), triggering
// BREAK and a fall-back-then-climb cycle. The optimizer now corrects the
// conservatism upward on the first batch close. Raised to 6.0 dB.
#define SUPERSHIFT_MARGIN_DB 6.0

// Re-trigger supershift if measured SNR suggests we're this many configs below optimal.
// Checked after each ladder gearshift SET_CONFIG success (fresh OFDM SNR available).
#define SUPERSHIFT_RETRIGGER_CONFIGS 3

// Controlled-elevator multi-rung jump BOUND (gearshift-climb-engine.md §15, the
// DEEP-SNR over-climb regression fix). Even once the data-viable anchor has PROVEN
// the OFDM tier (the §15 primary gate `is_ofdm_config(anchor)`), a single SNR-driven
// re-trigger jump is capped at `config_ladder_up_n(anchor, RETRIGGER_MAX_LEAP, …)` so
// a marginal-OFDM channel (CONFIG_0 holds but CONFIG_13 does not) cannot overshoot the
// whole ladder in one shot — it leaps in bounded steps as the anchor ratchets up, with
// the proven-ceiling cap (arq_commander.cc:4591-4592 / elevator_target_from_snr) and
// the §10 anchor-demotion backstopping any residual overshoot. TUNABLE: chosen as the
// smallest value that preserves the existing high-SNR multi-rung climb (a CONFIG_4
// anchor still reaches CONFIG_16 in one leap = gap 12; a CONFIG_0 anchor still reaches
// CONFIG_13 = gap 13), so the WGN:30 fast climb is materially unchanged (≤2 bounded
// leaps from a low OFDM anchor) while a pathological jump is bounded.
#define RETRIGGER_MAX_LEAP 13

#define YES 1
#define NO 0

// Config-to-string conversion for GUI display
// Bitrates are approximate (low-density pilots, default config)
inline const char* config_to_string(int config) {
	switch (config) {
		case ROBUST_0: return "ROBUST 0 (32-MFSK, ~14 bps)";
		case ROBUST_1: return "ROBUST 1 (16-MFSK x2, ~22 bps)";
		case ROBUST_2: return "ROBUST 2 (16-MFSK x2, ~87 bps)";
		case CONFIG_0:  return "CONFIG 0 (BPSK 1/16, ~84 bps)";
		case CONFIG_1:  return "CONFIG 1 (BPSK 2/16, ~185 bps)";
		case CONFIG_2:  return "CONFIG 2 (BPSK 3/16, ~285 bps)";
		case CONFIG_3:  return "CONFIG 3 (BPSK 4/16, ~385 bps)";
		case CONFIG_4:  return "CONFIG 4 (BPSK 5/16, ~485 bps)";
		case CONFIG_5:  return "CONFIG 5 (BPSK 6/16, ~586 bps)";
		case CONFIG_6:  return "CONFIG 6 (BPSK 8/16, ~786 bps)";
		case CONFIG_7:  return "CONFIG 7 (QPSK 5/16, ~890 bps)";
		case CONFIG_8:  return "CONFIG 8 (QPSK 6/16, ~1074 bps)";
		case CONFIG_9:  return "CONFIG 9 (QPSK 8/16, ~1441 bps)";
		case CONFIG_10: return "CONFIG 10 (8PSK 6/16, ~1354 bps)";
		case CONFIG_11: return "CONFIG 11 (8PSK 8/16, ~1818 bps)";
		case CONFIG_12: return "CONFIG 12 (QPSK 14/16, ~2655 bps)";
		case CONFIG_13: return "CONFIG 13 (8PSK 12/16, ~2882 bps)";
		case CONFIG_14: return "CONFIG 14 (8PSK 14/16, ~3390 bps)";
		case CONFIG_15: return "CONFIG 15 (16QAM 14/16, ~5088 bps)";
		case CONFIG_16: return "CONFIG 16 (32QAM 14/16, ~5665 bps)";
		default: return "UNKNOWN";
	}
}

// Long config name with narrowband awareness
inline const char* config_to_string_nb(int config, bool narrowband) {
	if (!narrowband) return config_to_string(config);
	switch (config) {
		case ROBUST_0: return "ROBUST 0 NB (8-MFSK, ~28 bps)";
		case ROBUST_1: return "ROBUST 1 NB (4-MFSK x2, ~37 bps)";
		case ROBUST_2: return "ROBUST 2 NB (4-MFSK x2, ~149 bps)";
		case CONFIG_0:  return "CONFIG 0 NB (BPSK 1/16, ~61 bps)";
		case CONFIG_15: return "CONFIG 15 NB (16QAM 14/16, ~1018 bps)";
		case CONFIG_16: return "CONFIG 16 NB (32QAM 14/16, ~1133 bps)";
		default: return config_to_string(config); // fallback to wideband string
	}
}

// Short config label for status bar
inline const char* config_to_short_string(int config) {
	switch (config) {
		case ROBUST_0: return "ROBUST 0";
		case ROBUST_1: return "ROBUST 1";
		case ROBUST_2: return "ROBUST 2";
		case CONFIG_0:  return "CFG 0 BPSK";
		case CONFIG_1:  return "CFG 1 BPSK";
		case CONFIG_2:  return "CFG 2 BPSK";
		case CONFIG_3:  return "CFG 3 BPSK";
		case CONFIG_4:  return "CFG 4 BPSK";
		case CONFIG_5:  return "CFG 5 BPSK";
		case CONFIG_6:  return "CFG 6 BPSK";
		case CONFIG_7:  return "CFG 7 QPSK";
		case CONFIG_8:  return "CFG 8 QPSK";
		case CONFIG_9:  return "CFG 9 QPSK";
		case CONFIG_10: return "CFG 10 8PSK";
		case CONFIG_11: return "CFG 11 8PSK";
		case CONFIG_12: return "CFG 12 QPSK";
		case CONFIG_13: return "CFG 13 8PSK";
		case CONFIG_14: return "CFG 14 8PSK";
		case CONFIG_15: return "CFG 15 16QAM";
		case CONFIG_16: return "CFG 16 32QAM";
		default: return "???";
	}
}

// Short config label with narrowband prefix
inline const char* config_to_short_string_nb(int config, bool narrowband) {
	if (!narrowband) return config_to_short_string(config);
	switch (config) {
		case ROBUST_0: return "NB ROB 0";
		case ROBUST_1: return "NB ROB 1";
		case ROBUST_2: return "NB ROB 2";
		case CONFIG_0:  return "NB C0 BPSK";
		case CONFIG_1:  return "NB C1 BPSK";
		case CONFIG_2:  return "NB C2 BPSK";
		case CONFIG_3:  return "NB C3 BPSK";
		case CONFIG_4:  return "NB C4 BPSK";
		case CONFIG_5:  return "NB C5 BPSK";
		case CONFIG_6:  return "NB C6 BPSK";
		case CONFIG_7:  return "NB C7 QPSK";
		case CONFIG_8:  return "NB C8 QPSK";
		case CONFIG_9:  return "NB C9 QPSK";
		case CONFIG_10: return "NB C10 8PSK";
		case CONFIG_11: return "NB C11 8PSK";
		case CONFIG_12: return "NB C12 QPSK";
		case CONFIG_13: return "NB C13 8PSK";
		case CONFIG_14: return "NB C14 8PSK";
		case CONFIG_15: return "NB C15 16QAM";
		case CONFIG_16: return "NB C16 32QAM";
		default: return "NB ???";
	}
}

#endif // INC_COMMON_DEFINES_H_
