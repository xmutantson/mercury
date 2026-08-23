// ============================================================================
// Option B compact coded reverse-confirm — LIVE RX-PATH regression (test-only)
// ============================================================================
//
// CLI: --test-compact-confirm-rx   (also runs inside `mercury.exe --test`)
//
// Pairs with fact-documents/data-flow-compact-confirm.md §3/§4/§9 and
// _research/coded_confirm/ (the K=5 GF(16)-RA codec feasibility).
//
// THE BUG THIS CAPTURES (root cause, w3mxyyze8):
//   The commander accept gate (arq_commander.cc, the "Data ACK pattern detected"
//   else-if) wires the compact-confirm decode AFTER the bare-pattern presence
//   gate:
//       !sack_window_open && receive_ack_pattern()
//                         && ( ... || cmd_compact_confirm_crc_valid() || ...)
//   receive_ack_pattern() is a CRC-LESS 7/16 base-pattern presence gate that
//   exists ONLY to protect the legacy bare ACK. The K=5 compact confirm is a
//   SELF-VALIDATING frame (its own 16-sym base detect + GF(16) soft-decode +
//   CRC12 inside cmd_compact_confirm_crc_valid()). Gating the self-validating
//   compact confirm behind the bare presence gate is wrong: when the bare gate
//   short-circuits FALSE, cmd_compact_confirm_crc_valid() is NEVER reached and
//   the confirm is missed -> BREAK / false-confirm storm.
//
//   The pre-existing in-process test (test_compact_confirm_passband_roundtrip_clean)
//   PASSED only because it called decode_compact_confirm_from_passband() DIRECTLY,
//   bypassing the live bare-pattern gate. This test drives the FULL LIVE RX
//   path: passband -> commander capture ring -> the production accept expression.
//
// THE FIX (decouple, not band-aid): the self-validating compact confirm gets its
// OWN capture + decode + CRC on its OWN geometry, NOT behind the bare 7/16
// presence gate. The bare-pattern gate for the legacy bare ACK is UNCHANGED; the
// CRC12 (over [bsi]) provides the false-confirm protection that the bare pattern
// lacks, so decoupling does NOT re-open false-confirm (verified below: a noise
// tail and a wrong-bsi frame are both REJECTED, and a 13-uncoded ACK+SACK frame
// is NOT cross-accepted as a compact confirm).
//
// FAIL-BEFORE / PASS-AFTER CONTRACT:
//   PRE-FIX  (MERCURY_COMPACT_RX_FAILBEFORE=1, same binary): the accept path is
//            the OLD `receive_ack_pattern() && ...` chain -> the compact frame is
//            REJECTED (bare gate peaks < 7/16 on the 26-sym frame) -> FAIL.
//   POST-FIX (default): the compact confirm is accepted via its own CRC-gated
//            decode on the live ring -> PASS. The ACK+SACK frame still accepts via
//            the legacy bare+CRC arm; noise + wrong-bsi + cross-frame all reject.
//
// Deterministic, no RF, no IONOS. Returns 0 on PASS, 1 on FAIL.
// ============================================================================

#include "datalink_layer/arq.h"
#include "datalink_layer/datalink_defines.h"
#include "physical_layer/telecom_system.h"
#include "common/common_defines.h"
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cstdlib>
#include <vector>
#include <cmath>

#if MFSK_ACK_SACK_ENABLED

namespace {

struct cc_rx_ctx {
	int fails = 0;
	void check(bool ok, const char* what) {
		printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
		if(!ok) fails++;
		fflush(stdout);
	}
};

// Place `frame` (active_samples valid samples) into the commander's capture ring
// so the frame's NEWEST sample ends `trailing_silence` samples before signal_period
// (i.e. the frame is followed by trailing_silence samples of post-frame silence,
// then the tail-window boundary). trailing_silence=0 is the freshly-arrived-at-the-
// tail case; trailing_silence>0 models the live ~500 Hz capture-prep thread / PTT
// drain having scrolled some post-frame idle silence in behind the frame before the
// frames_to_read==0 snapshot fires (inter-Pi clock drift + turnaround timing put the
// frame at an arbitrary phase in the tail window). ring_write_index=0; ftr=0.
static void seat_frame_at_phase(cl_telecom_system* ts,
                                const double* frame, int active_samples,
                                int trailing_silence)
{
	int sym = ts->data_container.Nofdm * ts->data_container.interpolation_rate;
	int signal_period = sym * ts->data_container.buffer_Nsymb;
	double* ring = ts->data_container.passband_delayed_data;
	memset(ring, 0, (size_t)2 * signal_period * sizeof(double));
	if(trailing_silence < 0) trailing_silence = 0;
	int start = signal_period - active_samples - trailing_silence;
	if(start < 0) start = 0;
	int n = active_samples;
	if(start + n > signal_period) n = signal_period - start;
	memcpy(ring + start, frame, (size_t)n * sizeof(double));
	memcpy(ring + signal_period, ring, (size_t)signal_period * sizeof(double));
	ts->data_container.ring_write_index = 0;
	ts->data_container.frames_to_read = 0;
	ts->data_container.data_ready = 1;
}

// The freshly-arrived-at-the-tail placement (no trailing silence).
static void seat_frame_at_live_tail(cl_telecom_system* ts,
                                    const double* frame, int active_samples)
{
	seat_frame_at_phase(ts, frame, active_samples, 0);
}

// Sweep the placement phase and return the WORST (minimum) bare-pattern peak the
// correlator scores across the post-frame-silence window. This is the live defect's
// search space: if the compact frame's worst-case bare peak drops below the 7/16
// gate at phases where the ACK+SACK frame does NOT, the bare gate is geometry-
// fragile on the shorter frame (the task's hypothesis). The sweep strides whole
// symbols across one tail-window's worth of post-frame silence.
static int worst_bare_peak_over_phase(cl_arq_controller* cmd, cl_telecom_system* ts,
                                      const double* frame, int active_samples,
                                      int* out_worst_trailing)
{
	int sym = ts->data_container.Nofdm * ts->data_container.interpolation_rate;
	int worst = 9999; int worst_trail = 0;
	// Up to ~30 symbols of trailing silence, one-symbol strides.
	for(int t = 0; t <= 30; t++)
	{
		int trail = t * sym;
		seat_frame_at_phase(ts, frame, active_samples, trail);
		cmd->ack_diag_peak_matched = 0; cmd->ack_diag_peak_metric = 0.0;
		cmd->ack_diag_poll_count = 0;
		cmd->receive_ack_pattern();
		int pk = cmd->ack_diag_peak_matched;
		if(pk < worst) { worst = pk; worst_trail = trail / sym; }
	}
	if(out_worst_trailing) *out_worst_trailing = worst_trail;
	return worst;
}

// Over the SAME phase sweep, count how many phases the bare presence gate
// (receive_ack_pattern) ACCEPTS vs how many the self-validating compact decode
// (cmd_compact_confirm_crc_valid) ACCEPTS. The compact decode has NO 8-symbol
// energy pre-gate (it snapshots the full tail and the correlator searches every
// start position), so it should accept across the whole window where the bare gate
// only accepts when the frame sits in the last ~8 symbols. This QUANTIFIES the
// decoupling win and is the load-bearing evidence for the fix.
static void sweep_bare_vs_compact(cl_arq_controller* cmd, cl_telecom_system* ts,
                                  const double* frame, int active_samples,
                                  int* out_bare_accepts, int* out_compact_accepts,
                                  int* out_total)
{
	int sym = ts->data_container.Nofdm * ts->data_container.interpolation_rate;
	int bare_ok = 0, compact_ok = 0, total = 0;
	for(int t = 0; t <= 30; t++)
	{
		int trail = t * sym;
		seat_frame_at_phase(ts, frame, active_samples, trail);
		bool bare = cmd->receive_ack_pattern();
		seat_frame_at_phase(ts, frame, active_samples, trail);
		bool compact = cmd->cmd_compact_confirm_crc_valid();
		if(bare) bare_ok++;
		if(compact) compact_ok++;
		total++;
	}
	if(out_bare_accepts) *out_bare_accepts = bare_ok;
	if(out_compact_accepts) *out_compact_accepts = compact_ok;
	if(out_total) *out_total = total;
}

} // namespace

// The production accept predicate, isolated so the test drives EXACTLY the wire
// condition (and the FAIL-BEFORE = the OLD chain). Returns true iff the commander
// would set data_ack_received=YES for a CLEAN data ACK this poll.
//
// This MIRRORS the arq_commander.cc condition verbatim:
//   data_ack_received==NO && (v2_ack_pat_pre_detected
//       || (!sack_window_open && <ACK arm> ))
// with v2_ack_pat_pre_detected=0 and sack_window_open=0 (the bare-arm path the
// compact confirm rides). The ONLY thing the fix changes is the <ACK arm>, so the
// test calls the SAME helper the production code calls (cmd_compact_confirm_live_accept)
// — see arq_commander.cc. FAIL-BEFORE compiles the OLD chain.
int cl_arq_controller::test_compact_confirm_live_rx_path()
{
	const char* TAG = "compact_confirm_live_rx_path";
	printf("[TEST] %s starting\n", TAG);
	fflush(stdout);

#if !MFSK_ACK_SACK_ENABLED
	printf("  [SKIP] MFSK_ACK_SACK not compiled in\n");
	return 0;
#else
	bool failbefore = false;
	{
		const char* e = std::getenv("MERCURY_COMPACT_RX_FAILBEFORE");
		failbefore = (e && *e && *e != '0');
	}

	cc_rx_ctx ctx;
	const int CFG = CONFIG_0;   // deepest WB rung (compact confirm is WB-only)

	// Build a fresh COMMANDER controller + telecom_system at a WB config. Inlined
	// (not a free helper) so load_configuration() — a private member — is reachable
	// (a member fn may touch any same-class instance's privates). The compact confirm
	// is WB-only (M>=16); CONFIG_0 is the deepest WB rung (matches the codec tests).
	cl_telecom_system* ts  = new cl_telecom_system();
	cl_arq_controller* cmd = new cl_arq_controller();
	ts->operation_mode = ARQ_MODE;
	ts->narrowband_enabled = NO;     // WB → ack_mfsk M=16 (compact confirm capable)
	cmd->telecom_system = ts;
	cmd->role = COMMANDER;
	cmd->narrowband_enabled = NO;
	// The controller ctor seeds current_configuration=CONFIG_0; loading CONFIG_0
	// would early-return WITHOUT initializing ack_mfsk / the capture ring. Force the
	// CONFIG_NONE sentinel first so load_configuration actually runs the telecom load.
	cmd->current_configuration = CONFIG_NONE;
	cmd->load_configuration(CFG, FULL, NO);
	cmd->link_status = CONNECTED;
	cmd->connection_status = RECEIVING_ACKS_DATA;

	if(ts->ack_mfsk.compact_confirm_suffix_len() <= 0) {
		printf("  [SKIP] compact confirm unsupported on this WB config\n");
		delete cmd; delete ts;
		return 0;
	}

	int sym = ts->data_container.Nofdm * ts->frequency_interpolation_rate;

	// Scratch frame buffers (sized to the larger ACK+SACK pattern + guard).
	int max_samples = (ts->ack_mfsk.ack_sack_pattern_nsymb() + 4) * sym;
	std::vector<double> compact_buf((size_t)max_samples, 0.0);
	std::vector<double> acksack_buf((size_t)max_samples, 0.0);

	// The bsi the commander is waiting on (compact confirms a CLEAN, all-ones batch).
	const uint8_t bsi = 0x2A;
	cmd->cmd_batch_seq_id = bsi;

	// Pin the batch size so the legacy clean-data-ACK content gate
	// (cmd_clean_data_ack_crc_valid: rx_bitmap == (1<<data_batch_size)-1) has a
	// deterministic all-ones target that matches the ACK+SACK frame we synthesize.
	// 30 is the WB <=30-bit invariant ceiling.
	cmd->data_batch_size = 30;
	cmd->nMessages = cmd->data_batch_size;
	cmd->max_data_length = 170;
	cmd->max_header_length = 6;
	cmd->max_message_length = 200;
	ctx.check(cmd->init_messages_buffers() == SUCCESSFUL,
		"A2 fixture stages an owned generation for compact validation");
	for(int i=0; i<cmd->data_batch_size; i++) {
		cmd->messages_tx[i].status = PENDING_ACK;
		cmd->messages_tx[i].length = 1;
		cmd->messages_tx[i].batch_seq_id = bsi;
	}
	const uint32_t clean_bitmap = mfsk_sack_mask_for_frames(cmd->data_batch_size);

	// --- Generate the compact-confirm passband (16 base + 10 suffix = 26 sym) ---
	int compact_samples = 0;
	{
		unsigned char bb[1]; bb[0] = (unsigned char)bsi;
		uint16_t crc12 = (uint16_t)(cmd->CRC12_calc((const char*)bb, 1) & 0x0FFF);
		compact_samples = ts->generate_compact_confirm_passband(compact_buf.data(), bsi, crc12);
	}
	ctx.check(compact_samples > 0, "compact confirm passband generated");

	// --- Generate the 13-uncoded ACK+SACK passband (16 base + 13 = 29 sym) ---
	int acksack_samples = 0;
	{
		uint32_t bitmap = clean_bitmap;  // all-ones for data_batch_size (clean)
		char crc_in[5];
		crc_in[0] = (char)bsi;
		crc_in[1] = (char)((bitmap >> 24) & 0xFF);
		crc_in[2] = (char)((bitmap >> 16) & 0xFF);
		crc_in[3] = (char)((bitmap >>  8) & 0xFF);
		crc_in[4] = (char)( bitmap        & 0xFF);
		uint16_t crc12 = (uint16_t)(cmd->CRC12_calc(crc_in, 5) & 0x0FFF);
		acksack_samples = ts->generate_ack_sack_pattern_passband(
			acksack_buf.data(), bsi, bitmap, crc12);
	}
	ctx.check(acksack_samples > 0, "ACK+SACK passband generated");

	// ====================================================================
	// DIAGNOSTIC — measure the bare-pattern correlator IN-WINDOW. Establishes (NOT
	// assumes) that the in-window correlator peak is 16/16 for BOTH the 26-sym
	// compact AND the 29-sym ACK+SACK frame — the 16-sym base is byte-identical
	// (generate_compact_confirm_pattern and generate_ack_sack_pattern both call
	// generate_ack_pattern). So the live miss is NOT a correlator peak deficiency on
	// the shorter frame; the phase sweep below isolates the true mechanism (the bare
	// gate's 8-symbol energy pre-gate phase fragility).
	// ====================================================================
	int compact_bare_matched = 0;
	{
		seat_frame_at_live_tail(ts, compact_buf.data(), compact_samples);
		cmd->ack_diag_peak_matched = 0; cmd->ack_diag_peak_metric = 0.0;
		cmd->ack_diag_poll_count = 0;
		bool bare = cmd->receive_ack_pattern();   // CRC-less bare presence gate
		compact_bare_matched = cmd->ack_diag_peak_matched;
		printf("  [DIAG] compact frame (%d sym): receive_ack_pattern()=%d "
		       "peak_matched=%d/%d metric=%.2f\n",
		       compact_samples / sym, bare ? 1 : 0, compact_bare_matched,
		       ts->ack_mfsk.ack_match_threshold, cmd->ack_diag_peak_metric);
		fflush(stdout);
	}
	int acksack_bare_matched = 0;
	{
		seat_frame_at_live_tail(ts, acksack_buf.data(), acksack_samples);
		cmd->ack_diag_peak_matched = 0; cmd->ack_diag_peak_metric = 0.0;
		cmd->ack_diag_poll_count = 0;
		bool bare = cmd->receive_ack_pattern();
		acksack_bare_matched = cmd->ack_diag_peak_matched;
		printf("  [DIAG] ACK+SACK frame (%d sym): receive_ack_pattern()=%d "
		       "peak_matched=%d/%d metric=%.2f\n",
		       acksack_samples / sym, bare ? 1 : 0, acksack_bare_matched,
		       ts->ack_mfsk.ack_match_threshold, cmd->ack_diag_peak_metric);
		fflush(stdout);
	}

	// PHASE SWEEP — the LIVE defect's search space. The live capture places the
	// frame at an arbitrary phase in the tail window (inter-Pi drift + turnaround
	// timing). Find the WORST-case bare peak over post-frame-silence phases for
	// EACH frame. If the compact frame's worst peak drops below the 7/16 gate at a
	// phase where the ACK+SACK frame's does NOT, the bare gate is geometry-fragile
	// on the shorter frame (the task's hypothesis); if BOTH stay >=7 the bare gate
	// is robust and the live miss has a different mechanism (the bench-claim guard).
	{
		int ct = -1, at = -1;
		int compact_worst = worst_bare_peak_over_phase(cmd, ts, compact_buf.data(),
		                                               compact_samples, &ct);
		int acksack_worst = worst_bare_peak_over_phase(cmd, ts, acksack_buf.data(),
		                                               acksack_samples, &at);
		printf("  [DIAG] PHASE-SWEEP worst bare peak: compact=%d/%d (@%d sym trail) "
		       "ACK+SACK=%d/%d (@%d sym trail)\n",
		       compact_worst, ts->ack_mfsk.ack_match_threshold, ct,
		       acksack_worst, ts->ack_mfsk.ack_match_threshold, at);
		fflush(stdout);

		// Quantify the decoupling win on the compact frame: bare-gate accepts vs
		// compact-decode accepts over the phase window.
		int bare_ok = 0, compact_ok = 0, tot = 0;
		sweep_bare_vs_compact(cmd, ts, compact_buf.data(), compact_samples,
		                      &bare_ok, &compact_ok, &tot);
		printf("  [DIAG] PHASE-SWEEP accepts over %d phases (compact frame): "
		       "bare-gate=%d  compact-decode=%d\n", tot, bare_ok, compact_ok);
		fflush(stdout);
	}

	// ====================================================================
	// PRIMARY ASSERTION — the LIVE accept path at the DEFECT PHASE.
	//
	// ROOT CAUSE (empirically established by the sweep above, NOT assumed): the bare
	// presence gate receive_ack_pattern() has an 8-symbol ENERGY PRE-GATE (it probes
	// only the LAST 8 symbols of the tail for energy before running the FFT
	// correlator; arq_common.cc, probe_n=8*sym in production). When the reverse
	// confirm lands EARLIER in the tail window — which the live capture-prep timing
	// does (inter-Pi clock drift + PTT/turnaround drain scroll post-frame idle
	// silence in behind the frame before the frames_to_read==0 snapshot fires) — that
	// 8-symbol probe reads SILENCE and SKIPS the correlator, so the bare gate returns
	// FALSE even though the frame is fully present. The self-validating compact decode
	// (cmd_compact_confirm_crc_valid -> decode_compact_confirm_from_passband) has NO
	// such pre-gate: it snapshots the full 42-symbol tail and the correlator searches
	// every start position, so it finds the frame across far more phases. The sweep
	// above quantifies it (bare-gate ~8/31 phases vs compact-decode ~17/31). This is
	// NOT a correlator peak deficiency on the shorter frame (the in-window peak is
	// 16/16 for BOTH frames) — it is the bare gate's 8-symbol pre-gate phase fragility.
	//
	// The DEFECT phase: enough trailing silence that the 8-symbol probe reads silence
	// (trail > 8 sym) but the frame still sits inside the compact decode's tail
	// window (26-sym frame + trail <= 42-sym window).
	//   FAIL-BEFORE arm: gating the compact decode behind the bare gate -> the bare
	//                    gate's pre-gate misses -> compact confirm REJECTED (the bug).
	//   PASS-AFTER  arm: the decoupled compact decode finds + CRC-validates it -> ACCEPT.
	// ====================================================================
	const int DEFECT_TRAIL = 12;   // > 8-sym pre-gate, < (42-26) decode window margin

	// (0) ISOLATION PROBE at the defect phase: the bare gate misses while the compact
	//     decode succeeds. Establishes the mechanism (no assert — diagnostic).
	{
		seat_frame_at_phase(ts, compact_buf.data(), compact_samples, DEFECT_TRAIL * sym);
		bool bare = cmd->receive_ack_pattern();
		seat_frame_at_phase(ts, compact_buf.data(), compact_samples, DEFECT_TRAIL * sym);
		bool decode = cmd->cmd_compact_confirm_crc_valid();
		printf("  [DIAG] DEFECT phase (trail=%d sym): bare-gate=%d compact-decode=%d\n",
		       DEFECT_TRAIL, bare ? 1 : 0, decode ? 1 : 0);
		fflush(stdout);
		// Sanity: this IS a defect phase (bare misses, decode finds it). If not, the
		// in-window pre-gate assumption changed — surface it rather than silently
		// testing a non-defect phase.
		ctx.check(!bare && decode,
		  "DEFECT phase reproduces the mechanism: bare gate misses, compact decode finds it");
	}

	// (1) Compact confirm at the DEFECT phase — the live miss. SAME assertion in both
	//     arms (the textbook fail-before/pass-after contract): the compact confirm
	//     MUST be accepted at the defect phase. PASS-AFTER (production) -> ACCEPT ->
	//     PASS (rc=0). FAIL-BEFORE (use_legacy_chain) -> the bare pre-gate misses ->
	//     REJECT -> this check FAILS (rc=1), demonstrating the bug the fix removes.
	{
		seat_frame_at_phase(ts, compact_buf.data(), compact_samples, DEFECT_TRAIL * sym);
		bool accepted = cmd->cmd_compact_confirm_live_accept(/*sack_window_open=*/false, /*compact_enabled=*/true, /*use_legacy_chain=*/failbefore);
		ctx.check(accepted,
		  "compact confirm ACCEPTED at the live DEFECT phase (decoupled CRC-gated decode)");
		if(failbefore && !accepted)
			printf("    (expected under FAIL-BEFORE: the bare pre-gate misses the early-phase "
			       "frame so the old compact-behind-bare-gate chain rejects it)\n");
	}

	// (1b) Compact confirm at the CLEAN tail (frame in last symbols) MUST accept in
	//      BOTH arms — the bare gate's pre-gate sees it there, so the legacy chain
	//      works too. Proves the fix did not REGRESS the in-window case.
	{
		seat_frame_at_live_tail(ts, compact_buf.data(), compact_samples);
		bool accepted = cmd->cmd_compact_confirm_live_accept(/*sack_window_open=*/false, /*compact_enabled=*/true, /*use_legacy_chain=*/failbefore);
		ctx.check(accepted,
		  "compact confirm at the in-window (clean-tail) phase ACCEPTED in both arms");
	}

	// (2) The 13-uncoded ACK+SACK clean data ACK at the clean tail MUST still accept
	//     in BOTH arms (the legacy bare-pattern + CRC arm is untouched).
	{
		seat_frame_at_live_tail(ts, acksack_buf.data(), acksack_samples);
		bool accepted = cmd->cmd_compact_confirm_live_accept(/*sack_window_open=*/false, /*compact_enabled=*/true, /*use_legacy_chain=*/failbefore);
		ctx.check(accepted,
		  "legacy 13-uncoded ACK+SACK clean ACK still ACCEPTED (legacy arm preserved)");
	}

	// (3) FALSE-CONFIRM INVARIANT #1: a pure-silence / structured-noise tail MUST
	//     be REJECTED in the PASS-AFTER arm (the CRC12 protects the decoupled path).
	if(!failbefore) {
		std::vector<double> noise((size_t)compact_samples, 0.0);
		// Low-level structured noise (the phantom class) — well below a real frame.
		uint32_t r = 0x12345678u;
		for(int i = 0; i < compact_samples; i++) {
			r = r * 1664525u + 1013904223u;
			noise[i] = 1e-4 * (((double)(r >> 8) / (double)0xFFFFFF) - 0.5);
		}
		seat_frame_at_live_tail(ts, noise.data(), compact_samples);
		bool accepted = cmd->cmd_compact_confirm_live_accept(/*sack_window_open=*/false, /*compact_enabled=*/true, /*use_legacy_chain=*/false);
		ctx.check(!accepted,
		  "false-confirm invariant: noise tail REJECTED (no compact confirm)");
	}

	// (4) FALSE-CONFIRM INVARIANT #2: a compact confirm for a bsi OUTSIDE the
	//     commander's window (not cmd_bsi or prev) MUST be REJECTED even though its
	//     CRC is internally valid (the bsi-in-window gate). PASS-AFTER arm.
	if(!failbefore) {
		const uint8_t far_bsi = (uint8_t)((bsi + 50) & 0xFF);
		std::vector<double> far_buf((size_t)max_samples, 0.0);
		unsigned char bb[1]; bb[0] = (unsigned char)far_bsi;
		uint16_t crc12 = (uint16_t)(cmd->CRC12_calc((const char*)bb, 1) & 0x0FFF);
		int far_samples = ts->generate_compact_confirm_passband(far_buf.data(), far_bsi, crc12);
		seat_frame_at_live_tail(ts, far_buf.data(), far_samples);
		bool accepted = cmd->cmd_compact_confirm_live_accept(/*sack_window_open=*/false, /*compact_enabled=*/true, /*use_legacy_chain=*/false);
		ctx.check(!accepted,
		  "false-confirm invariant: out-of-window bsi REJECTED (bsi-in-window gate holds)");
	}

	// (5) FALSE-CONFIRM INVARIANT #3 (no cross-validate): the 13-uncoded ACK+SACK
	//     frame MUST NOT decode as a compact confirm (different CRC fields). Drive
	//     cmd_compact_confirm_crc_valid() DIRECTLY on the ACK+SACK tail.
	if(!failbefore) {
		seat_frame_at_live_tail(ts, acksack_buf.data(), acksack_samples);
		bool as_compact = cmd->cmd_compact_confirm_crc_valid();
		// It is allowed to detect the base, but the compact GF(16)+CRC12 over [bsi]
		// must NOT validate the 13-uncoded suffix.
		ctx.check(!as_compact,
		  "no cross-validate: 13-uncoded ACK+SACK does NOT validate as a compact confirm");
	}

	delete cmd; delete ts;

	printf("[TEST] %s %s (%d failures)\n", TAG,
	       ctx.fails == 0 ? "PASS" : "FAIL", ctx.fails);
	fflush(stdout);
	return ctx.fails == 0 ? 0 : 1;
#endif // MFSK_ACK_SACK_ENABLED
}

// ============================================================================
// Option B fix-b — LIVE RX-PATH regression for the SACK-WINDOW-OPEN case
// ============================================================================
//
// CLI: --test-compact-confirm-rx (runs both this and the SACK-closed test above).
//
// THE BUG THIS CAPTURES (data-flow-compact-confirm.md §10.4/§10.5, the SECOND
// live-land defect): for a batch>1, SACK-on, clean, WB session the RSP emits the
// COMPACT confirm (arq_responder.cc:2576). But the commander's Branch-2 SACK-window
// probe (process_messages_rx_acks_data) historically decoded ONLY the 13-uncoded
// ACK+SACK (decode_ack_sack_from_passband, CRC12 over the 5-byte [bsi||bitmap]). The
// compact tail (CRC12 over the single [bsi] byte) FAILS that 5-byte CRC12 and is
// dropped; the SACK-closed compact arm (cmd_compact_confirm_live_accept) HARD-BAILS
// at if(sack_window_open). Result: the confirm is lost, the commander waits out
// receiving_timeout and retransmits -> the 4.7x batch>1-WB delivery regression at
// ARQ_COMPACT_CONFIRM_ENABLE=1.
//
// THE FIX (root cause): cmd_compact_confirm_sack_window_accept() tries the
// SELF-VALIDATING compact decode FIRST, inside the window, BEFORE the 13-uncoded
// decode. The CRC12-over-[bsi] (not the SACK window) is the false-confirm guard.
//
// FAIL-BEFORE / PASS-AFTER CONTRACT:
//   PRE-FIX  (MERCURY_COMPACT_SACK_WINDOW_FAILBEFORE=1, same binary): the in-window
//            accept is the OLD Branch-2 — only decode_ack_sack_from_passband runs on
//            the compact tail -> NO CRC-valid clean ACK -> the confirm is REJECTED.
//   POST-FIX (default): the decoupled compact decode accepts it (returns true and
//            sets v2_ack_pat_pre_detected) -> ACCEPT. A corrupted-suffix compact is
//            rejected; a 13-uncoded ACK+SACK is NOT cross-accepted as compact;
//            out-of-window bsi rejected; a duplicate is consumed but not re-credited.
//
// Deterministic, no RF, no IONOS. Returns 0 on PASS, 1 on FAIL.
// (This function lives inside the file-level #if MFSK_ACK_SACK_ENABLED region that
//  also defines cc_rx_ctx / seat_frame_at_live_tail; the !MFSK_ACK_SACK stub is in
//  the file-level #else below.)
// ============================================================================
int cl_arq_controller::test_compact_confirm_sack_window_rx_path()
{
	const char* TAG = "compact_confirm_sack_window_rx_path";
	printf("[TEST] %s starting\n", TAG);
	fflush(stdout);

	bool failbefore = false;
	{
		const char* e = std::getenv("MERCURY_COMPACT_SACK_WINDOW_FAILBEFORE");
		failbefore = (e && *e && *e != '0');
	}

	cc_rx_ctx ctx;
	const int CFG = CONFIG_0;   // deepest WB rung (compact confirm is WB-only)

	cl_telecom_system* ts  = new cl_telecom_system();
	cl_arq_controller* cmd = new cl_arq_controller();
	ts->operation_mode = ARQ_MODE;
	ts->narrowband_enabled = NO;
	cmd->telecom_system = ts;
	cmd->role = COMMANDER;
	cmd->narrowband_enabled = NO;
	cmd->current_configuration = CONFIG_NONE;
	cmd->load_configuration(CFG, FULL, NO);
	cmd->link_status = CONNECTED;
	cmd->connection_status = RECEIVING_ACKS_DATA;

	if(ts->ack_mfsk.compact_confirm_suffix_len() <= 0) {
		printf("  [SKIP] compact confirm unsupported on this WB config\n");
		delete cmd; delete ts;
		return 0;
	}

	int sym = ts->data_container.Nofdm * ts->frequency_interpolation_rate;
	int max_samples = (ts->ack_mfsk.ack_sack_pattern_nsymb() + 4) * sym;
	std::vector<double> compact_buf((size_t)max_samples, 0.0);
	std::vector<double> acksack_buf((size_t)max_samples, 0.0);

	// THE batch>1 scenario: a multi-frame batch the commander is waiting on.
	const uint8_t bsi = 0x2A;
	cmd->cmd_batch_seq_id = bsi;
	cmd->data_batch_size = 8;                 // batch>1 (the regression scope)
	cmd->nMessages = cmd->data_batch_size;
	cmd->max_data_length = 170;
	cmd->max_header_length = 6;
	cmd->max_message_length = 200;
	ctx.check(cmd->init_messages_buffers() == SUCCESSFUL,
		"A2 fixture stages an owned generation for compact validation");
	for(int i=0; i<cmd->data_batch_size; i++) {
		cmd->messages_tx[i].status = PENDING_ACK;
		cmd->messages_tx[i].length = 1;
		cmd->messages_tx[i].batch_seq_id = bsi;
	}
	cmd->cmd_last_applied_clean_bsi = -1;     // nothing applied yet
	cmd->cmd_last_applied_sack_bsi  = -1;
	const uint32_t clean_bitmap = mfsk_sack_mask_for_frames(cmd->data_batch_size);

	// --- Generate the compact-confirm passband (16 base + 10 suffix) ---
	int compact_samples = 0;
	{
		unsigned char bb[1]; bb[0] = (unsigned char)bsi;
		uint16_t crc12 = (uint16_t)(cmd->CRC12_calc((const char*)bb, 1) & 0x0FFF);
		compact_samples = ts->generate_compact_confirm_passband(compact_buf.data(), bsi, crc12);
	}
	ctx.check(compact_samples > 0, "compact confirm passband generated");

	// --- Generate a clean 13-uncoded ACK+SACK passband (the no-cross-validate probe) ---
	int acksack_samples = 0;
	{
		uint32_t bitmap = clean_bitmap;
		char crc_in[5];
		crc_in[0] = (char)bsi;
		crc_in[1] = (char)((bitmap >> 24) & 0xFF);
		crc_in[2] = (char)((bitmap >> 16) & 0xFF);
		crc_in[3] = (char)((bitmap >>  8) & 0xFF);
		crc_in[4] = (char)( bitmap        & 0xFF);
		uint16_t crc12 = (uint16_t)(cmd->CRC12_calc(crc_in, 5) & 0x0FFF);
		acksack_samples = ts->generate_ack_sack_pattern_passband(
			acksack_buf.data(), bsi, bitmap, crc12);
	}
	ctx.check(acksack_samples > 0, "ACK+SACK passband generated");

	// Helper: simulate the PRE-FIX Branch-2 in-window accept — ONLY the 13-uncoded
	// decode ran on whatever tail is seated. Returns true iff a CRC-valid, in-window,
	// CLEAN (all-ones) 13-uncoded ACK was decoded (i.e. the OLD code would have set
	// v2_ack_pat_pre_detected). On the compact tail this MUST be false (the bug).
	auto old_branch2_accepts = [&](void)->bool {
		uint8_t  rx_bsi = 0; uint32_t rx_bitmap = 0; uint16_t rx_crc12 = 0; int m = 0;
		// Same tail window the production Branch-2 snapshots (16 + max(snr,sack) + 16).
		int ack_nsymb = ts->ack_mfsk.ack_pattern_nsymb;
		int pattern_len = ts->ack_mfsk.ack_snr_pattern_nsymb();
		int sack_suffix_len = ts->ack_mfsk.ack_sack_suffix_len();
		if(sack_suffix_len > pattern_len - ack_nsymb)
			pattern_len = ack_nsymb + sack_suffix_len;
		int tail_n = ack_nsymb + pattern_len + 16;
		int sp = sym * ts->data_container.buffer_Nsymb;
		int tail_samples = tail_n * sym; if(tail_samples > sp) tail_samples = sp;
		int tail_off = sp - tail_samples;
		memcpy(ts->data_container.ready_to_process_passband_delayed_data,
			&ts->data_container.passband_delayed_data[
				ts->data_container.ring_write_index + tail_off],
			(size_t)tail_samples * sizeof(double));
		bool dec = ts->decode_ack_sack_from_passband(
			ts->data_container.ready_to_process_passband_delayed_data,
			tail_samples, &rx_bsi, &rx_bitmap, &rx_crc12, &m);
		if(!dec) return false;
		char ci[5];
		ci[0]=(char)rx_bsi;
		ci[1]=(char)((rx_bitmap>>24)&0xFF); ci[2]=(char)((rx_bitmap>>16)&0xFF);
		ci[3]=(char)((rx_bitmap>>8)&0xFF);  ci[4]=(char)(rx_bitmap&0xFF);
		if(rx_crc12 != (uint16_t)(cmd->CRC12_calc(ci,5)&0x0FFF)) return false;
		unsigned cb=(unsigned)(cmd->cmd_batch_seq_id&0xFF), pb=(cb-1u)&0xFFu;
		if(!((unsigned)rx_bsi==cb||(unsigned)rx_bsi==pb)) return false;
		return mfsk_sack_bitmap_is_clean(rx_bitmap, cmd->data_batch_size);
	};

	// ====================================================================
	// PRIMARY ASSERTION — the in-SACK-window compact accept (the live miss).
	// Both arms assert the SAME thing: the compact confirm MUST be accepted in the
	// SACK window. PASS-AFTER routes through the decoupled compact decode -> ACCEPT.
	// FAIL-BEFORE replays the OLD Branch-2 (13-uncoded decode only) -> the compact
	// tail does NOT yield a CRC-valid clean ACK -> REJECT -> this check FAILS (rc=1).
	// ====================================================================
	{
		seat_frame_at_live_tail(ts, compact_buf.data(), compact_samples);
		bool pre = false;
		bool accepted;
		if(failbefore)
			accepted = old_branch2_accepts();              // OLD path: 13-uncoded only
		else
			accepted = cmd->cmd_compact_confirm_sack_window_accept(
				/*compact_enabled=*/true, &pre);           // NEW path: decoupled compact
		ctx.check(accepted,
		  "compact confirm ACCEPTED inside the SACK window (batch>1, clean, WB)");
		if(!failbefore)
			ctx.check(pre,
			  "fresh CLEAN compact sets v2_ack_pat_pre_detected (routes to clean funnel)");
		if(failbefore && !accepted)
			printf("    (expected under FAIL-BEFORE: the pre-fix Branch-2 decoded only the "
			       "13-uncoded ACK+SACK on the compact tail -> 5-byte CRC12 fail -> dropped)\n");
	}

	// PASS-AFTER-only invariants (the fix's safety contract).
	if(!failbefore) {
		// (a) DEDUPE: a SECOND compact for the SAME bsi is consumed (return true) but
		//     does NOT re-set the pre-detect flag (no double-credit of nBatches_fully_acked).
		//     cmd_last_applied_clean_bsi was set to bsi by the accept above.
		{
			seat_frame_at_live_tail(ts, compact_buf.data(), compact_samples);
			bool pre = false;
			bool accepted = cmd->cmd_compact_confirm_sack_window_accept(true, &pre);
			ctx.check(accepted && !pre,
			  "duplicate compact for an already-applied clean bsi: consumed, NOT re-credited");
		}
		// Reset the dedupe tracker for the remaining single-shot probes.
		cmd->cmd_last_applied_clean_bsi = -1;

		// (b) NO-CROSS-VALIDATE: a 13-uncoded clean ACK+SACK frame MUST NOT be accepted
		//     by the compact path (different CRC fields) — it falls through to the legacy
		//     13-uncoded decode. cmd_compact_confirm_sack_window_accept returns false.
		{
			seat_frame_at_live_tail(ts, acksack_buf.data(), acksack_samples);
			bool pre = false;
			bool accepted = cmd->cmd_compact_confirm_sack_window_accept(true, &pre);
			ctx.check(!accepted && !pre,
			  "no cross-validate: 13-uncoded ACK+SACK NOT accepted as compact (falls to legacy)");
			// Sanity: that SAME frame DOES decode via the legacy 13-uncoded path (the
			// partial-SACK/clean path is intact and would handle it).
			ctx.check(old_branch2_accepts(),
			  "legacy 13-uncoded clean ACK still decodes via the unchanged Branch-2 path");
		}

		// (c) FALSE-CONFIRM #1: a corrupted-suffix compact (valid base, garbled codeword)
		//     MUST be rejected (the GF(16) soft-decode + CRC12-over-[bsi] guard).
		{
			std::vector<double> bad((size_t)max_samples, 0.0);
			memcpy(bad.data(), compact_buf.data(), (size_t)compact_samples * sizeof(double));
			// Corrupt the suffix region (everything after the 16-sym base) with noise.
			int base_samples = ts->ack_mfsk.ack_pattern_nsymb * sym;
			uint32_t r = 0xC0FFEEu;
			for(int i = base_samples; i < compact_samples; i++) {
				r = r * 1664525u + 1013904223u;
				bad[i] = ((double)(r >> 8) / (double)0xFFFFFF) - 0.5;  // full-scale noise
			}
			seat_frame_at_live_tail(ts, bad.data(), compact_samples);
			bool pre = false;
			bool accepted = cmd->cmd_compact_confirm_sack_window_accept(true, &pre);
			ctx.check(!accepted && !pre,
			  "false-confirm: corrupted-suffix compact REJECTED (GF16+CRC12 guard)");
		}

		// (d) FALSE-CONFIRM #2: an out-of-window bsi compact (CRC internally valid) MUST
		//     be rejected (the bsi-in-window gate).
		{
			const uint8_t far_bsi = (uint8_t)((bsi + 50) & 0xFF);
			std::vector<double> far_buf((size_t)max_samples, 0.0);
			unsigned char bb[1]; bb[0] = (unsigned char)far_bsi;
			uint16_t crc12 = (uint16_t)(cmd->CRC12_calc((const char*)bb, 1) & 0x0FFF);
			int far_samples = ts->generate_compact_confirm_passband(far_buf.data(), far_bsi, crc12);
			seat_frame_at_live_tail(ts, far_buf.data(), far_samples);
			bool pre = false;
			bool accepted = cmd->cmd_compact_confirm_sack_window_accept(true, &pre);
			ctx.check(!accepted && !pre,
			  "false-confirm: out-of-window bsi compact REJECTED (bsi-in-window gate)");
		}

		// (e) GATE-OFF byte-identity: with compact_enabled=false the predicate returns
		//     false immediately even on a perfect compact tail (the held-off default ->
		//     byte-identical to the pre-fix Branch-2).
		{
			seat_frame_at_live_tail(ts, compact_buf.data(), compact_samples);
			bool pre = false;
			bool accepted = cmd->cmd_compact_confirm_sack_window_accept(
				/*compact_enabled=*/false, &pre);
			ctx.check(!accepted && !pre,
			  "gate-off: compact_enabled=false returns false (byte-identical pre-fix path)");
		}
	}

	delete cmd; delete ts;

	printf("[TEST] %s %s (%d failures)\n", TAG,
	       ctx.fails == 0 ? "PASS" : "FAIL", ctx.fails);
	fflush(stdout);
	return ctx.fails == 0 ? 0 : 1;
}

#else  // !MFSK_ACK_SACK_ENABLED

int cl_arq_controller::test_compact_confirm_live_rx_path()
{
	printf("[TEST] compact_confirm_live_rx_path SKIPPED (MFSK_ACK_SACK disabled)\n");
	return 0;
}

int cl_arq_controller::test_compact_confirm_sack_window_rx_path()
{
	printf("[TEST] compact_confirm_sack_window_rx_path SKIPPED (MFSK_ACK_SACK disabled)\n");
	return 0;
}

#endif
