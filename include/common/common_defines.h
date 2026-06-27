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

// MERCURY_BUILD_ID — git short-rev (+ "-dirty") of the BUILT source, baked
// deterministically by build.sh into the generated include/common/build_id.h
// (same source -> same id -> same md5 on every Pi). The header is .gitignored
// and may be absent on a from-scratch compile that skipped build.sh, so guard
// with a fallback here. VERSION__ stays a plain string literal: arq_common.cc's
// "VERSION Mercury " VERSION__ "\r" relies on string-literal concatenation, so
// the build-id is a SEPARATE macro, never folded into VERSION__.
#if __has_include("common/build_id.h")
#include "common/build_id.h"
#endif
#ifndef MERCURY_BUILD_ID
#define MERCURY_BUILD_ID "unknown"
#endif

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

// Option B — compact coded reverse-confirm (data-flow-compact-confirm.md). When
// 1, the responder emits the K=5 GF(16)-RA compact confirm (16+10 sym, ~70ms
// shorter AND ~4 dB more robust than the 13-uncoded ACK suffix) for a CLEAN
// (all-ones) batch, and the commander tries cmd_compact_confirm_crc_valid()
// first. The CODEC is sim-PROVEN (cliff -4.22 dB deeper, FAR 1e-4, full-path
// round-trip + no-cross-validate all PASS in --test). This is the temporary,
// DEFAULT-ON (unconditional, pre-release): the WB M=16 compact coded reverse
// confirm (Option B). Sim-proven codec (+4.22 dB deeper cliff, ~70 ms shorter on
// the wire than the MFSK ACK+SACK suffix) and the faithful real-audio re-verify
// (6bcc3964) cleared both central invariants on a batch>1-WB-SACK channel,
// replicated on two boxes: (a) reverse airtime SHORTER and the compact confirm is
// ACCEPTED inside the SACK window (4-7/clean cell, was 0 at f1da9cbe — the 4.7x
// batch>1-WB delivery regression is GONE, e1>=e0 clean), and (b) 0 false-confirm
// on an asymmetric reverse-WORSE channel (CRC12-over-[bsi] is the guard) with the
// partial-SACK retx path intact (the compact decode REJECTS a SACK partial frame,
// different field layout). The no-cross-validate invariant holds: the compact
// CRC12-over-[bsi] cannot validate a 13-uncoded [bsi||bitmap] suffix and vice-versa.
// NOT combinable with the FORGIVING-ACK cumulative path (the compact field carries
// plain bsi, no n_r reshape) — the responder gates compact OFF when
// cumulative_ack_enabled. WB-only (M=16); NB / ROBUST M<8 return-path is out of
// scope (compact_confirm_suffix_len()<=0 => no-op there).
#ifndef ARQ_COMPACT_CONFIRM_ENABLE
#define ARQ_COMPACT_CONFIRM_ENABLE 1
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
#define TX_WAV 10  // ULTRA audio render: one frame -> S16LE WAV, no audio device (sim instrument)
#define SIM_INPROC 11  // single-process in-process self-loopback feasibility prototype (no device/TCP/threads)
#define DECODE_RXCTRL 12  // PROBE X: offline replay of a dumped control-frame passband buffer through receive_byte (no device/relay/threads) — cross-platform decode arbiter

// NUMBER_OF_CONFIGS = 18 to make CONFIG_17 (shaped-64-QAM gear, default-NOT-
// selected) REACHABLE via -s 17 / the SFO-GRID harness. This is the table-size /
// monitor-decoder-array bound (arq.h:3214 monitor_decoders[], arq_common.cc
// parallel-decode loops, telecom_system.cc:9677 load_configuration range guard) —
// NOT the gearshift ceiling. The gearshift/optimizer ceiling is WB_CONFIG_MAX,
// LEFT at CONFIG_16 (below), so the climb engine NEVER auto-elects CFG17. The only
// behavioral delta from this bump is MONITOR_MODE initializing one extra (CFG17)
// parallel decoder; ARQ / PLOT_PASSBAND / SFO-GRID production paths are
// byte-identical with CFG17 unused. See fact-documents/data-flow-cfg17-shaped-64qam.md §1.2.
#define NUMBER_OF_CONFIGS 18
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
// CONFIG_17 = shaped-64-QAM top-gear SKELETON (LDPC rate-14/16). DEFAULT-NOT-
// SELECTED: WB_CONFIG_MAX stays CONFIG_16, FULL_CONFIG_LADDER does NOT list it,
// is_ofdm_config caps at 16 — so no climb/optimizer/D3 path elects it. Reachable
// ONLY via explicit -s 17 or the SFO-GRID harness (the composition vehicle for
// PAS + TINTERP-seed turbo + ratio-nvfix). The D3 demote-gate (==CONFIG_16 ->
// ||==CONFIG_17) + optimizer/effective-rate-table registration + the WB_CONFIG_MAX
// /is_ofdm_config/ladder bumps are HELD for after the CFG16-acquisition fix to
// avoid concurrent gearshift changes (RESEARCH_cfg17-64qam.md §4.5).
#define CONFIG_17 17

// ROBUST (MFSK) configurations - values 100+ to avoid collision with OFDM configs
#define NUMBER_OF_ROBUST_CONFIGS 3
#define ROBUST_0 100  // 32-MFSK, LDPC rate 1/16, ~14 bps (hailing mode)
#define ROBUST_1 101  // 16-MFSK x2, LDPC rate 1/16, ~22 bps
#define ROBUST_2 102  // 16-MFSK x2, LDPC rate 1/4,  ~87 bps

inline bool is_robust_config(int config) { return config >= 100 && config <= 102; }
inline bool is_ofdm_config(int config) { return config >= 0 && config <= 16; }

// §21 (tier2-suffix-fec-design.md): the base-pattern noncoherent combining factor
// for the PRODUCTION enhanced CONNECT suffix at the robust tier. R=4 is the §20
// sim/HW-validated operating point (= cl_mfsk::MAX_CONNECT_PREAMBLE_REPS); R=1
// would be combining-off. CONNECT is once/session so the +26.8% airtime of R=4 is
// negligible (§4). Only applied when is_robust_config(current) — OFDM stays R=1.
#define CONNECT_PREAMBLE_REPS_PROD 4

// FIX-A: ROBUST-tier dwell-batch decouple (data-flow-robust-tier-arq-batch.md).
// At a ROBUST config the data batch is normally pinned to 1 (one MFSK frame per
// ACK turnaround — >90% dead-time on the deep-SNR floor) because at the MFSK
// cliff P(batch clean)=p^N and only batch=1 makes the strict all-ones clean
// target achievable while the climb is still earning the rung. FIX-A lifts the
// pin to a multi-frame batch ONLY once the link is PROVEN + PARKED on a robust
// rung (robust_dwell_batch_eligible(), arq.h): the rung has already delivered
// clean batches (anchor reached it) and the climb is not actively probing a
// higher rung, so the p^N penalty is acceptable and the M=16 MFSK SACK suffix
// patches any partial. Then the whole payload streams 4-8 frames per turnaround
// instead of one, amortizing the fixed ACK dead-time.
//
// ROBUST_DWELL_BATCH_MAX — the hard ceiling the relaxed set_data_batch_size()
//   chokepoint clamps a robust batch into ([1..MAX]). 8 keeps the all-ones SACK
//   target (1<<batch)-1 = 0xFF well under both the CMD 32-bit and RSP 30-bit
//   bitmap caps (data-flow-robust-tier-arq-batch.md §3.1), and ≤ the M=16 SACK
//   suffix's 32-frame bitmap (mfsk-robust-ack.md).
// ROBUST_DWELL_BATCH — the value the CMD requests on a proven+parked robust
//   dwell (the operating point inside [1..MAX]).
// ROBUST_DWELL_PROOF_BATCHES — consecutive clean batches AT THE ROBUST RUNG
//   required before the raise fires (proves PARKED, not transient).
// All three are SWEPT in the FTRT sim, not magic-numbered (CLAUDE.md §1 / OR-5).
// Starting values below; the sweep result is recorded in the fact doc §8.
#define ROBUST_DWELL_BATCH_MAX     8
#define ROBUST_DWELL_BATCH         4
#define ROBUST_DWELL_PROOF_BATCHES 2

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

// ROBUST-CONFIG-AWARE ladder navigation (gearshift-climb-engine.md §19): the
// OFDM-only fast-path is taken ONLY when the session is not robust-enabled AND
// the config argument is not itself a robust-tier config. A robust LIVE config
// (current_configuration in [ROBUST_0..ROBUST_2]) implies the full ladder
// regardless of the intent flag `robust_enabled` (which the GUI per-loop sync at
// main.cc:2430 can clobber to NO, and an explicit `-s 100` without `-R` leaves
// NO). Without this, config_ladder_up(ROBUST_0, robust_enabled=NO) returns
// ROBUST_0 UNCHANGED (100 >= WB ceiling 16) → FRAME-UP logs "config 100 -> 100"
// → the link never climbs off ROBUST_0 at any SNR. When robust_enabled==YES the
// added clause is a no-op (the full-ladder branch already ran), so the proven
// climb/anti-thrash paths are byte-identical. See §19.
inline int config_ladder_up(int config, bool robust_enabled, bool narrowband = false) {
	int ceiling = narrowband ? NB_CONFIG_MAX : WB_CONFIG_MAX;
	if (!robust_enabled && !is_robust_config(config)) {
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
	if (!robust_enabled && !is_robust_config(config)) {   // §19: robust LIVE config ⇒ full ladder
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
	if (!robust_enabled && !is_robust_config(config)) {   // §19: robust LIVE config ⇒ full ladder
		return (config > CONFIG_0) ? config - 1 : config;
	}
	int idx = config_ladder_index(config);
	if (idx > 0) return FULL_CONFIG_LADDER[idx - 1];
	return config;
}

inline int config_ladder_down_n(int config, int steps, bool robust_enabled) {
	if (!robust_enabled && !is_robust_config(config)) {   // §19: robust LIVE config ⇒ full ladder
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
	// §19: a robust LIVE config is never "at the OFDM ceiling"; rank it on the
	// full ladder regardless of robust_enabled (which the GUI loop can clear).
	if (!robust_enabled && !is_robust_config(config)) return config == WB_CONFIG_MAX;
	return config_ladder_index(config) == FULL_CONFIG_LADDER_SIZE - 1;
}

inline bool config_is_at_bottom(int config, bool robust_enabled) {
	if (!robust_enabled && !is_robust_config(config)) return config == CONFIG_0;   // §19
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

// WALL-B FIX-2 (WALLB_DIAGNOSIS.md §7 FIX-2; long-run-degradation latch #2): the
// BREAK-down aggression step `break_drop_step` is a session AARF that doubles on
// every un-ACKed BREAK recovery (arq_commander.cc:227/:327) but — on the CFG16
// big-block carve path — has NO decrease (its only reset is the clean-fully-
// delivered-batch :3708, which never fires when 0 blocks carve). Uncapped, it
// marches 2->4->8->16->32... and config_ladder_down_n(emergency_prev=16, step)
// walks 16->14->12->8->0->ROBUST_0 in four deaf-peer cycles, stranding a clean
// channel at ROBUST_0 with 0 bytes delivered.
//
// CAP = 16 = the OFDM span. FULL_CONFIG_LADDER index math: ROBUST_0/1/2 = idx 0..2,
// CONFIG_0 = idx 3 ... CONFIG_16 = idx 19. The OFDM rungs span CONFIG_16(19)..
// CONFIG_0(3) = 16 ladder steps. With the step capped at 16, ONE EXHAUSTED BREAK
// recovery from CFG16 lands at idx 19-16=3 = CONFIG_0 (the LOWEST OFDM rung) — it
// can never skip the entire OFDM span straight into the ROBUST tier in a single
// recovery cycle. This BOUNDS THE RATE OF DESCENT (one OFDM-span per cycle), it
// does NOT forbid reaching ROBUST tiers: from any rung below CFG16 a step of 16
// still clamps the ladder index to 0 = ROBUST_0 (config_ladder_down_n floors at 0),
// so a genuinely-cratered channel still escapes downward over successive cycles.
// The panic single-shot crater (break_drop_step = 100 at arq_commander.cc:3608) is
// a deliberate SET applied at a SEPARATE site and is consumed for the recovery
// target BEFORE any doubling, so the cap (applied only at the *=2 sites) never
// blocks the intended panic jump to ROBUST_0.
static const int BREAK_DROP_STEP_MAX = 16;   // OFDM ladder span (CONFIG_16 -> CONFIG_0)

// IDLE-SWITCHROLE-RACE recovery bound (idle-switchrole-race.md §3/§6). Number of
// consecutive NO-PROGRESS BREAK-EXHAUSTED re-arms (no RX data this session AND tx
// FIFO empty AND a zero-byte re-queue) tolerated before conceding the link is dead
// and routing to the FORCED-fallback teardown (arq_common.cc:3053-3065). This is a
// structural escalation count, NOT a tuned margin: the watchdog never disconnects
// (arq_common.cc:2985 re-arms itself) so without this a never-fed link spins
// forever; the only legacy escape is the 180 s FORCED_ROLE_SWITCH_TIMEOUT keyed off
// receiving_timer. K=4 gives ~one FORCED cycle of headroom (4 BREAK cycles ~ 25s
// each) before tearing down. A live BREAK with ANY progress resets the counter to 0
// (negative control), so this is a no-op on every healthy or recovering link.
static const int BREAK_NOPROGRESS_TEARDOWN_K = 4;

// PURE kernel of the BREAK no-progress escalation (idle-switchrole-race.md §3).
// On a DEAD cycle (no_progress==true: no RX data this session AND empty TX AND a
// zero-byte re-queue) increment `cycles` and return true ONLY once it reaches the
// teardown bound K — the caller then routes to the FORCED-fallback teardown. On
// ANY progress (no_progress==false) RESET `cycles` to 0 and return false (the
// negative control: a live BREAK is never torn down). Production calls this at the
// EXHAUSTED re-arm site; the unit test (test_break_noprogress_teardown) replays it
// directly, same as break_drop_step_after_double's "test replays it" idiom.
inline bool break_noprogress_step(int& cycles, bool no_progress) {
	if(!no_progress) { cycles = 0; return false; }
	cycles++;
	return cycles >= BREAK_NOPROGRESS_TEARDOWN_K;
}

// Multiplicative-INCREASE half of the break_drop_step AARF, CAPPED. Production
// calls this at BOTH doubling sites (arq_commander.cc:227/:327) instead of a bare
// `break_drop_step *= 2`. PURE; the unit test (Part R) replays it directly.
inline int break_drop_step_after_double(int step) {
	int doubled = step * 2;
	return (doubled > BREAK_DROP_STEP_MAX) ? BREAK_DROP_STEP_MAX : doubled;
}

// Multiplicative-DECREASE half (the missing AARF decay, mirrors the
// frame_shift_threshold decay at arq_commander.cc:3737). A successful BREAK-RECOVERY
// handshake (a recovery SET_CONFIG ACKed at the Phase-2 site, arq_commander.cc:4795)
// is forward progress, so HALVE the step back toward its base floor (2). This is a
// single-notch DECAY, NOT a reset to 2: a full reset here was deliberately removed
// (arq_commander.cc:4798-4811 — it defeated the escalation ladder on the WGN:14->0
// deep-SNR sweep, crawling one config/BREAK). Halving lets the ladder keep escalating
// across consecutive UN-ACKed BREAKs while relaxing one notch when a handshake
// actually completes. Floors at the base aggression 2 (never below). PURE; Part R
// replays it directly.
inline int break_drop_step_after_handshake_decay(int step) {
	int decayed = step / 2;
	return (decayed < 2) ? 2 : decayed;
}

// WALL-B FIX-4 (WALLB_DIAGNOSIS.md §7 FIX-4): carve-viability deadline. On a CLEAN
// channel the link holds CFG16 on SACK-trust (the C0-a CFG16-HOLD skip,
// arq_commander.cc:5006), but the big-block carve fires 0x (wall-C) so the RSP never
// sends a data-ACK. After K=emergency_nack_threshold (3) consecutive block-failures
// with ZERO carves, the EXISTING code enters the BREAK cascade — which, on the CFG16
// big-block rung, is the wall-B free-flow collapse (a deaf-peer cascade to ROBUST_0,
// 0 bytes on long transfers). emergency_nack_count RESETS to 0 on ANY data-ACK (clean
// OR partial, arq_commander.cc:3699), so reaching the threshold is exactly "no carve /
// data-ACK landed within the deadline" — the natural trigger the diagnosis names.
//
// This pure decision picks the FALLBACK: when the failing rung is the CFG16 BIG-BLOCK
// rung specifically and the deadline is reached, fall back to the HIGHEST per-frame
// OFDM rung (config_ladder_down(CFG16) == CONFIG_15) and run PER-FRAME there, INSTEAD
// of cascading toward ROBUST_0. Per-frame CFG15 is NOT carve-gated (the RX carve gate
// and the TX bigblock_send_one_block BOTH require current_configuration == CONFIG_16),
// so a CFG16-deaf RSP CAN follow this demote: the CMD steps it via the EXISTING,
// decodable SET_CONFIG control exchange (the SET_CONFIG is sent on the live CFG16 PHY
// and re-decoded by the RSP on the STOCK per-frame path via the GAP-3 carve-gate
// fallback, arq_common.cc:8186-8211). This is FIX-1's policy expressed at the
// block-failure decision point, WITHOUT touching FIX-1's demote targeting
// (config_ladder_down_n / emergency_previous_config) or FIX-3's RSP receive routing.
//
// Returns the per-frame fallback config (CONFIG_15) when the deadline is reached on the
// CFG16 big-block rung; returns -1 otherwise (caller proceeds to the EXISTING BREAK
// path unchanged — the generic fade-down BREAK on non-bigblock rungs is byte-identical,
// and the deep-SNR escape for a genuinely-cratered channel is preserved). PURE; the
// unit test (Part S) replays it directly.
inline int bigblock_carve_fallback_target(int current_config, bool bigblock_rung_live,
		int nack_count, int nack_threshold, bool robust_enabled) {
#ifdef WALLB_FIX4_FAILBEFORE
	(void)current_config; (void)bigblock_rung_live; (void)nack_count;
	(void)nack_threshold; (void)robust_enabled;
	return -1;   // FAIL-BEFORE stub: no carve-viability fallback exists -> BREAK cascade.
#else
	// ONLY the CFG16 big-block rung has a carve that can be non-viable. A non-CFG16
	// config (no carve) or a non-bigblock rung -> no fallback (let the BREAK path run).
	if (!bigblock_rung_live) return -1;
	if (current_config != CONFIG_16) return -1;
	// Deadline: K consecutive block-failures with zero carves/data-ACKs.
	if (nack_count < nack_threshold) return -1;
	// The highest per-frame OFDM rung directly below CFG16. config_ladder_down on the
	// full ladder maps CFG16(idx 19) -> CFG15(idx 18); on a non-robust live config it
	// maps CFG16 -> CFG15 too. Guard the result is a real per-frame OFDM rung (never
	// returns a robust/MFSK target — that would re-enter the deaf-peer regime).
	int fallback = config_ladder_down(current_config, robust_enabled);
	if (!is_ofdm_config(fallback) || is_robust_config(fallback)) return -1;
	if (fallback >= current_config) return -1;   // must be a genuine demote
	return fallback;
#endif
}

// WALL-B FIX-5 (WALLB_DIAGNOSIS.md §1.6 limit cycle; fix5/FIX5_DESIGN.md §4): CFG16
// big-block carve COOLDOWN. After a FIX-4 carve-viability demote fires
// (arq_commander.cc:3669, supershift_proven_ceiling=CFG15), the CFG16 big-block rung is
// PROVEN non-viable on THIS channel. But FIX-4's pin has a lifetime of ~one cycle:
// finish_turbo_direction() UNCONDITIONALLY resets supershift_proven_ceiling to the probe
// top (=turboshift_last_good=CFG16) on every ROBUST->CFG16 turbo re-climb
// (arq_commander.cc:4147). So on the pg84 long stream the loop THRASHES:
// climb->CFG16 carve-park->3 carve-fails->FIX-4 demote->CFG15->gearshift RE-climbs to
// CFG16 (clean channel, SNR says go)->re-elects the carve-dead rung->re-parks->repeat.
// Nothing carries cross-cycle "this rung is carve-dead" memory because
// supershift_proven_ceiling is owned/reset by the turbo state machine.
//
// FIX-5 = a SEPARATE CMD-side field bigblock_carve_cooldown_batches that the turbo state
// machine does NOT touch (so it survives the :4147 reset), armed at the FIX-4 deadline,
// and consulted as an additional index-cap (->CFG15) at every climb hook. It is an AARF:
// the 1st demote arms BASE batches; each RE-demote WHILE the cooldown is still active (the
// carve is STILL dead) DOUBLES the span, capped at MAX (mirrors BREAK_DROP_STEP_MAX's 16x
// span idiom). A real CFG16 carve clears it; the batch count expires the window for an
// optimistic re-probe.

// BASE = 1st demote window in BATCHES (~3x the measured ~8-batch elect->deadline distance,
// so one clean CFG15 run amortizes the prior storm). MAX = 16x BASE (mirroring
// BREAK_DROP_STEP_MAX's 16x ladder-span idiom). Unit is BATCHES not wall-time: the thrash
// cadence is dominated by variable deaf-peer BREAK/watchdog retry storms (minutes per
// cycle), so a wall clock would either expire mid-storm (useless) or need worst-case
// length (over-long on a fast channel); a batch count self-scales with link speed and is
// consulted exactly at the re-election decision.
static const int BB_CARVE_COOLDOWN_BASE = 24;
static const int BB_CARVE_COOLDOWN_MAX  = 384;

// WALL-B FIX-5: pick the NEXT cooldown span (AARF multiplicative-increase, capped). 1st
// demote (prev_span<=0) = base; each repeat WHILE the cooldown is still active (re-demote
// before it expired = the carve is STILL dead) doubles, capped at cap. PURE; the unit test
// (Part T) replays it directly. FAIL-BEFORE: returns 0 (no cooldown) -> the re-climb limit
// cycle (0 bytes on pg84) is NOT broken.
inline int bigblock_carve_cooldown_next_span(int prev_span, int base, int cap) {
#ifdef WALLB_FIX5_FAILBEFORE
	(void)prev_span; (void)base; (void)cap;
	return 0;   // FAIL-BEFORE: no cooldown -> the re-climb limit cycle persists.
#else
	int next = (prev_span <= 0) ? base : prev_span * 2;
	return (next > cap) ? cap : next;
#endif
}

// WALL-B FIX-5: the climb-cap CEILING. Returns the cooldown ceiling (CONFIG_15 == the
// highest per-frame OFDM rung, config_ladder_down(CONFIG_16)) when a CFG16 big-block carve
// cooldown is active (batches remaining > 0), else -1 (no cap — the no-op normal case, so
// non-bigblock and clean-CFG16 operation is byte-identical). PURE; Part T replays it.
// FAIL-BEFORE: returns -1 always (no cap exists) -> CFG16 re-election is never refused.
inline int bigblock_carve_cooldown_ceiling(int cooldown_batches_remaining, bool robust_enabled) {
#ifdef WALLB_FIX5_FAILBEFORE
	(void)cooldown_batches_remaining; (void)robust_enabled;
	return -1;
#else
	if (cooldown_batches_remaining <= 0) return -1;
	return config_ladder_down(CONFIG_16, robust_enabled);   // CFG15: highest per-frame rung
#endif
}

// WALL-B FIX-9 D3 (bigblock_p3_hw/_fix9/FIX9_ROOTCAUSE.md §1/§2 Rank-1, FIX9_D3_DESIGN.md):
// the CFG16 reverse-ACK STARVATION discriminator. On a CLEAN channel the link climbs to CONFIG_16
// and SUSTAINS the 25-frame batch; inter-Pi sample-clock drift de-aligns the half-duplex turnaround
// at the DENSEST constellation; the reverse MFSK ACK+SACK correlator fails to decode (the timeout
// fires — on HW the window goes pure-silent peak_metric=0.0; in the drift-sim it degrades to a
// SUB-THRESHOLD partial peak_matched 5/7, peak_metric~0.5 — BOTH are "the reverse ACK did not
// decode = reverse-ACK loss"). N such consecutive block-failures BREAK off CFG16 -> the ROBUST
// cascade (0 bytes). D3 demotes CFG16->CFG15 (one rung down, NOT a 2+ rung BREAK toward ROBUST) and
// holds it, instead of cascading.
//
// Returns the fallback config (CONFIG_15) when ALL of:
//   (D3-1) current_config == CONFIG_16 (the densest 32-QAM rung; FIX9_ROOTCAUSE "Why CFG16": CFG16
//          is the intersection of densest-constellation x longest-sustained-batch — it de-aligns
//          first; CFG15 holds the turnaround it lacks, proven by the drift-OFF control).
//   (D3-2) OFDM IS PROVEN ON THIS CHANNEL: ofdm_proven == true (last_data_viable_config is an OFDM
//          config — the link delivered DATA at an OFDM rung this session). This is the discriminator
//          vs a GENUINE deep-SNR collapse: there the anchor is a ROBUST/MFSK rung (OFDM never proved
//          out), so D3-2 is FALSE and the BREAK->ROBUST escape runs UNTOUCHED. When OFDM IS proven,
//          a CFG16-specific failure should step DOWN WITHIN OFDM (to CFG15), not abandon OFDM
//          entirely. NOTE: this is INTENTIONALLY broader than "anchor==CFG16" — the HW repro raised
//          the anchor into the high OFDM tier before the reverse-ACK starved, but the drift-sim
//          elects CFG16 speculatively (anchor at the OFDM floor) and BOTH directions de-align; the
//          shared, faithful signal across both is "OFDM works, CFG16 specifically doesn't".
//   (D3-3/D3-4) SUSTAINED reverse-ACK loss: starve_fails >= starve_threshold, where starve_fails
//          counts CONSECUTIVE block-failures AT CFG16 (the ACK timed out, data_ack_received==NO).
//          A SINGLE transient CFG16 ACK miss does NOT abandon the rung.
//   (D3-5) the PER-FRAME path (bigblock_rung_live == false): mutually exclusive with FIX-4, which
//          owns the big-block carve path.
// Else returns -1 (caller proceeds to the EXISTING BREAK path unchanged — deep-SNR escape intact).
// This is NOT a generic ACK-timeout rebrand: D3-1 restricts it to CFG16 alone, and D3-2 restricts
// it to the case where OFDM is proven viable (so the move is a within-OFDM step-down, never an
// OFDM-abandoning BREAK). A timeout at any other rung, or a CFG16 timeout with no OFDM proven,
// takes the unchanged BREAK path.
// PURE; the unit test (Part V) replays it directly. FAIL-BEFORE: -DFIX9_D3_FAILBEFORE -> -1 always
// (no D3 fallback -> the reverse-ACK-starved link BREAKs to ROBUST_0, 0 bytes).
inline int cfg16_revack_starve_fallback_target(int current_config, bool ofdm_proven,
		bool bigblock_rung_live, int starve_fails, int starve_threshold, bool robust_enabled) {
#ifdef FIX9_D3_FAILBEFORE
	(void)current_config; (void)ofdm_proven; (void)bigblock_rung_live;
	(void)starve_fails; (void)starve_threshold; (void)robust_enabled;
	return -1;   // FAIL-BEFORE stub: no D3 fallback -> the reverse-ACK-starved BREAK cascade runs.
#else
	if (bigblock_rung_live) return -1;          // (D3-5) FIX-4 owns the big-block carve path.
	if (current_config != CONFIG_16) return -1; // (D3-1) only CFG16 has this turnaround fragility.
	if (!ofdm_proven) return -1;                // (D3-2) deep-SNR collapse (anchor robust) -> BREAK.
	if (starve_fails < starve_threshold) return -1; // (D3-3/D3-4) the sustained-loss deadline.
	// (D3) target = the highest per-frame OFDM rung directly below CFG16 (config_ladder_down maps
	// CFG16 -> CFG15). Guard it is a real per-frame OFDM rung (never robust/MFSK -> never re-enters
	// the de-aligned-peer regime), exactly like FIX-4.
	int fallback = config_ladder_down(current_config, robust_enabled);
	if (!is_ofdm_config(fallback) || is_robust_config(fallback)) return -1;
	if (fallback >= current_config) return -1;  // must be a genuine demote.
	return fallback;
#endif
}

// WALL-B FIX-9 D3: the sustained-reverse-ACK-silence deadline (BATCHES of consecutive pure-silence
// block-failures at CFG16 before the per-frame demote). =2 per the root-cause Q4 D3 spec ("2
// consecutive ACK-silence block-failures with healthy forward SACK history"). TUNABLE.
static const int CFG16_REVACK_STARVE_FAILS = 2;

// WALL-B FIX-9 D2 (bigblock_p3_hw/_fix9/FIX9_ROOTCAUSE.md §4 D2, FIX9_D2_DESIGN.md): the
// reverse-data-ACK turnaround-geometry predicate. The reverse MFSK ACK+SACK PHY (the M=16 WB
// Welch-Costas pattern, send_mfsk_ack_sack) is config-INDEPENDENT in TONE SET — the same correlator
// pattern fires at every config — but it is EMITTED inside the LIVE config's PTT/turnaround
// geometry. At the robust tier the RSP keys the ACK with a pre-TX pumped_settle_wait guard
// (arq_common.cc:6581, the fatter turnaround the OFDM cadence lacks), and the CMD's listen-window
// budget covers it. At OFDM forward configs that guard is SKIPPED, so under inter-Pi sample-clock
// drift the ppm-slipped reverse ACK lands OUTSIDE the CMD's tight OFDM-cadence window and the
// correlator goes pure-silent (the D3 collapse). D2 attacks the ROOT: emit the reverse data-ACK on
// the SAME robust turnaround geometry (the pre-TX settle + a CMD window widen, in lockstep) at ALL
// OFDM forward configs, so the slipped ACK STILL lands in window and CFG16 itself HOLDS — the
// stronger outcome than D3's after-the-fact CFG15 settle (D2 removes D3's trigger; the two compose
// — D2 = prevention, D3 = recovery if the drift defeats even the robust geometry).
//
// Returns true iff the LIVE forward data config is an OFDM rung (where the OFDM-cadence ACK is at
// risk of the drift-driven turnaround de-alignment). FALSE at the robust tier (the EXISTING
// is_robust_config guard already fires the settle there — robust path byte-identical) and FALSE for
// NB (handled by send_mfsk_ack_sack's ack_sack_suffix_len()<=0 early-return). The CMD's
// calculate_receiving_timeout() and the RSP's send_mfsk_ack_sack() BOTH call this on
// current_configuration, so the two sides cannot diverge (the FIX9_ROOTCAUSE §5 lockstep invariant).
// PURE; the unit test (Part W) replays it directly. FAIL-BEFORE: -DFIX9_D2_FAILBEFORE -> false
// always (no robust geometry at OFDM -> the reverse-ACK-starved link collapses, the D3 path runs).
inline bool reverse_ack_uses_robust_geometry(int forward_config) {
#ifdef FIX9_D2_FAILBEFORE
	(void)forward_config;
	return false;   // FAIL-BEFORE stub: OFDM keeps the tight cadence -> the drift collapse.
#else
	return is_ofdm_config(forward_config) && !is_robust_config(forward_config);
#endif
}

// WALL-B FIX-9 D2: the extra CMD listen-window margin (ms) added when the reverse data-ACK is keyed
// on the robust turnaround geometry. Covers the accumulated per-batch clock-drift arrival jitter
// the stock per-frame frame_drain (=2*message_transmission_time_ms) did not absorb. The HW evidence
// showed the correlator going PURE-SILENT (the window CLOSED before the late ACK) rather than a few-
// ms near-miss, so this is dominated by the settle-flush re-arm gap, not the ~6 ms/batch raw slip.
// Same order as SACK_ARRIVAL_MARGIN_MS (1000). TUNABLE; gate (b) confirms it suffices (repro HOLDS
// CFG16). The RSP's matching pre-TX settle (ptt_off+ptt_on) is added SEPARATELY in lockstep.
static const int ROBUST_ACK_DRIFT_MARGIN_MS = 600;

// WALL-B FIX-9 H1 (bigblock_p3_hw/_revackgeom/data-flow-revack-turnaround-geometry.md §5.1): the
// smallest forward batch (frames) at which the RSP can answer with a PARTIAL SACK (and therefore
// arm its pre-TX settle, arq_responder.cc:1789). The D2-REFINE producer (arq_commander.cc:1954)
// arms data_ack_retx_turnaround ONLY on a v2-mixed/degrading batch, which is FALSE on the FIRST
// partial of a FRESH OFDM batch — the RSP settles that partial ACK while the CMD listen window is
// still narrow (the §5.1 INV-1 lockstep break -> the partial SACK lands outside -> the silent
// reverse-ACK miss that drives the D3 starvation deadline). H1 PRE-ARMS the CMD widen for any
// fresh OFDM batch large enough to go partial, restoring lockstep BEFORE the partial is known. A
// single-frame batch (data_batch_size==1) can never go partial (one frame is all-or-nothing), so
// the threshold is the smallest MULTI-frame batch =2. Robust/NB are excluded by the is_ofdm_config
// gate on the disjunct (this constant only sizes the OFDM arm). TUNABLE.
static const int BATCH_MAY_BE_PARTIAL_THRESHOLD = 2;

// TURNAROUND BATCH-AIRTIME RE-PHASE (bench-9; bigblock_p3_hw/_turnaroundfix/TURNAROUND_FIX_DESIGN.md §5.1)
// ----------------------------------------------------------------------------------------------------
// ROOT CAUSE (HW-confirmed, BENCH9_VERDICT.json): the CMD reverse-ACK listen window
// (calculate_receiving_timeout, arq_common.cc CMD branch) is built ENTIRELY from PER-FRAME / fixed
// terms — frame_drain = 2*message_transmission_time_ms + sack_arrival(fixed) + margin. There is NO
// term proportional to FORWARD BATCH AIRTIME. But on a held-CFG16 25-30-frame batch the half-duplex
// channel is occupied ~4.78s, so keyer/AGC/capture-flush/scheduling latency + the ±8.16ppm crystal
// slip accumulate WITHIN the batch and push the single end-of-batch reverse SACK systematically LATE
// in proportion to batch airtime. The CMD window opens at the right phase for a ~2-frame batch and
// ~143ms too EARLY for a ~28-frame batch -> the SACK slides fully past the window (matched=0/7, 81%
// stall on bench-9). This is a ONE-SIDED, batch-length-proportional MIS-PHASE, NOT a too-narrow
// window: H1 (a fixed +600ms widen, ROBUST_ACK_DRIFT_MARGIN_MS) was ACTIVE in the bench-9 binary
// 627c370 and the pinned held-CFG16 STILL stalled 81% — widening a window whose CENTER is ~143ms off
// past its ~90ms half-width cannot recapture a one-sided slide.
//
// FIX: add a batch-airtime-keyed late-shift (re-CENTER the window LATER by the accrued amount so the
// SACK lands back in the middle). Calibrated to the SAME 30 ms/s the relay sim uses
// (sim/turnaround-calibrate @8606389, TurnaroundDrift.accrual_ms_per_s default 30.0,
// late_offset = accrual_ms_per_s * burst_airtime_s) so HW and sim share ONE locked constant. This is
// the ARDOP "adjust for keying-offset" discipline in static-calibrated form and the structural analog
// of PACTOR's fixed turnaround idle scaled to batch length (prior art: TURNAROUND_FIX_DESIGN.md §3).
//
// DEFAULT-OFF: gated by MERCURY_TURNAROUND_REPHASE (see turnaround_rephase_enabled_common,
// arq_common.cc). With the env UNSET production is BYTE-IDENTICAL to monitor tip 627c370. Even when
// ENABLED the term only bites on OFDM multi-frame batches (is_ofdm_config && data_batch_size >=
// BATCH_MAY_BE_PARTIAL_THRESHOLD); CFG15-short / robust / NB / single-frame degenerate to ~0 accrual.
static const int TURNAROUND_ACCRUAL_MS_PER_S = 30; // relay-locked (8606389); ms late-shift per s of fwd batch airtime
// Small fixed guard added with the accrual to cover the residual spread of the SACK arrival around the
// re-centered phase (the accrual cancels the MEAN late-shift; the guard covers the variance). Bounded.
static const int ACCRUAL_PHASE_GUARD_MS    = 150;

// ===========================================================================
// FORGIVING-ACK TIER 2 — cumulative-n_r status report (self-healing spine)
// fact-documents/data-flow-forgiving-ack.md TIER 2.
// ===========================================================================
// A missed reverse-ACK is SUPERSEDED by the next one: the SACK's 8-bit bsi field
// is reinterpreted (when the CAP_CUMULATIVE_ACK capability is negotiated on BOTH
// ends) as n_r = the cumulative contiguous delivery high-water
// (rsp_last_delivered_batch_seq_id). A later report's n_r covers everything every
// earlier report would have, so a residual miss is recovered FOR FREE on the next
// turnaround (STANAG-5066 acknowledged-through semantics). SEMANTICS-only — no
// wire-width change; the 30/32-bit selective bitmap is unchanged (it describes the
// batch ABOVE n_r). Both helpers are PURE so --test-cumulative-ack drives the EXACT
// production predicates; -DCUMULATIVE_ACK_FAILBEFORE pins the apply to the per-batch
// fallback for the fail-before arm.

// CUMULATIVE_ACK_WINDOW: how many batches BACKWARD from n_r the CMD will accept as
// cumulatively-acknowledged. Mercury is batch-level stop-and-wait (cmd_batch_seq_id
// advances per SENT batch, one batch outstanding), so the outstanding batch is at
// most a couple back; W=8 recovers up to 8 superseded reports (matched to the Tier-1
// re-air bound) while staying FAR below the mod-256 half-window (128) so the forward/
// backward wrap direction is never ambiguous. TUNABLE.
static const int CUMULATIVE_ACK_WINDOW = 8;

// SEND helper (RSP, producer): choose the value written into the SACK bsi field.
// When the cumulative cap is negotiated AND a high-water exists, send n_r (the
// contiguous high-water, which by INV-T2-CONTIG can never point past an undelivered
// batch); otherwise send the legacy per-batch bsi (byte-identical default-off). A
// high_water < 0 (nothing delivered yet) falls back to per_batch_bsi so a fresh
// session never emits a 0xFF/garbage n_r. Returns a value already masked to 8 bits.
inline unsigned char cumulative_ack_bsi_field(unsigned char per_batch_bsi,
		int high_water, bool cap_on) {
	if (cap_on && high_water >= 0)
		return (unsigned char)(high_water & 0xFF);
	return per_batch_bsi;
}

// APPLY helper (CMD, consumer): does a received report with high-water n_r address
// `target_bsi` (the batch the CMD is waiting on)?
//   cap_on  == false : pure per-batch fallback -> return per_batch_in_window
//                      (the legacy rx_bsi==cmd_bsi||rx_bsi==prev_bsi test the caller
//                      already computed). This is the interop-safe path: a Tier-2 CMD
//                      that did NOT negotiate the cap with this RSP never reinterprets
//                      the RSP's per-batch bsi as an n_r.
//   cap_on  == true  : cumulative -> target_bsi is addressed iff it lies in the
//                      window [n_r - W .. n_r + 1] (mod 256). Two sub-ranges, each
//                      grounded in the wire semantics:
//                       (a) [n_r - W .. n_r]: target is at-or-below the contiguous
//                           delivery high-water ⇒ CONFIRMED DELIVERED. This is the
//                           cumulative ACK / SELF-HEAL (a later n_r retires a batch
//                           whose own earlier report was missed). By INV-T2-CONTIG
//                           n_r never points past an undelivered batch, so "≤ n_r =
//                           delivered" is a TRUE statement, never optimistic.
//                       (b) n_r + 1 (forward overhang of EXACTLY 1): the in-flight
//                           PARTIAL batch the 30-bit selective bitmap describes (the
//                           contiguous successor of the high-water; Mercury is batch-
//                           level stop-and-wait so the only batch above n_r is n_r+1).
//                           The overhang is BOUNDED to 1 — never an arbitrary forward
//                           reach — so a corrupt n_r still cannot ACK a far-future /
//                           unsent batch (§T2.4 safety preserved).
//                      A STALE (older) n_r leaves a newer outstanding batch outside
//                      [n_r-W .. n_r+1] ⇒ UNaddressed ⇒ the CMD keeps waiting / re-airs.
// -DCUMULATIVE_ACK_FAILBEFORE pins this to the per-batch fallback even when cap_on,
// so the self-heal assertion fails before / passes after in the same binary.
inline bool cumulative_ack_covers(int rx_n_r, int target_bsi, bool cap_on,
		bool per_batch_in_window) {
#ifdef CUMULATIVE_ACK_FAILBEFORE
	(void)rx_n_r; (void)target_bsi; (void)cap_on;
	return per_batch_in_window;   // FAIL-BEFORE: no cumulative recovery (per-batch only).
#else
	if (!cap_on) return per_batch_in_window;
	unsigned nr   = (unsigned)(rx_n_r     & 0xFF);
	unsigned tgt  = (unsigned)(target_bsi & 0xFF);
	unsigned succ = (nr + 1u) & 0xFFu;            // the selective-bitmap batch n_r+1
	if (tgt == succ) return true;                 // (b) in-flight partial batch (overhang 1)
	unsigned back = (nr - tgt) & 0xFFu;           // forward distance target->n_r (mod 256)
	return back <= (unsigned)CUMULATIVE_ACK_WINDOW;  // (a) at-or-below the high-water
#endif
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

// FIX-B — FLOOR-PROBE BACK-OFF (gearshift-floor-probe-backoff.md). At the
// robust/OFDM boundary a failed CONFIG_0 up-probe panic-collapses the link to
// the ROBUST tier; the climb then re-probes that SAME rung on a FIXED cadence,
// re-fails, and the link burns the deep-SNR floor's airtime in a
// CONFIG_0↔ROBUST_0/2 limit cycle (HW WGN:-10 config_counts ROBUST_0:6
// ROBUST_2:4 CONFIG_0:4). These bound a PER-RUNG exponential back-off so a
// PROVEN-FAILED up-probe is not hammered every cycle: the first failure
// suppresses re-probing of that rung for PROBE_BACKOFF_MS_INIT, doubling on each
// repeat fail up to PROBE_BACKOFF_MS_CAP, and the whole back-off is reset the
// instant ANY clean OFDM batch is delivered (the channel proved it recovered).
// OR-5 / [Q1]: these are STARTING values, to be SWEPT in the FTRT sim / HW
// floor-stack A/B (NOT magic-numbered final constants — see CLAUDE.md §1 and the
// fact-doc §6 sweep note). INIT ~8 s ≈ one robust-tier dwell cycle (long enough
// to do real floor work between probes, short enough that a recovering channel
// re-probes promptly); CAP ~120 s bounds the worst-case re-probe latency on a
// channel that genuinely improved but delivered no clean OFDM batch to trigger
// the reset.
#define PROBE_BACKOFF_MS_INIT 8000
#define PROBE_BACKOFF_MS_CAP  120000

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
		case CONFIG_17: return "CONFIG 17 (64QAM-PAS 14/16, ~6546 bps)";
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
		case CONFIG_17: return "CFG 17 64QAM";
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
