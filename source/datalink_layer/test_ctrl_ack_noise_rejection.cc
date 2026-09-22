// Control-ACK noise-rejection regression (cross-layer control-ACK acceptance audit).
//
// A CLOSE/control reverse-ACK is a bare, content-free MFSK base tone pattern
// (generate_ack_pattern_passband). The COMMANDER accepts it whenever
// receive_ack_pattern() returns true and, in DISCONNECTING, completes a full link
// teardown. Before this change the control-ACK accept used the lax DATA-ACK metric
// floor (ack_metric_threshold = 0.5). The metric is an in-band ENERGY-CONCENTRATION
// score (sum over matched symbols of target-tone-energy / total-passband-energy): a
// real tone burst concentrates energy in its tones (high score); white noise spreads
// it across all bins (low score). Every OTHER control-frame detector
// (detect_ack_snr_from_passband, decode_ctrl_suffix_from_passband, CONNECT) gates on
// the CONTROL floor CTRL_DETECT_METRIC_MIN (1.2); the CLOSE control-ACK was wrongly on
// the DATA floor (0.5), so a low-energy noise correlation that cleared matched>=7 &&
// metric>=0.5 could be promoted to an ACKed CLOSE and tear the session down. The fix
// brings the CLOSE control-ACK accept to the control-plane floor (parity, not a nudge).
//
// This test drives the PRODUCTION receive_ack_pattern() on the COMMANDER control-ACK
// path and verifies:
//   B  a strong clean ACK is accepted (harness self-check; records real-ACK concentration);
//   C  a weak-but-valid ACK that clears the control floor is STILL accepted (no over-tighten);
//   D  a PURE-WGN buffer whose detection lands in the [0.5, 1.2) gray band is ACCEPTED by
//      the old data floor and REJECTED by the control floor — the direct old-accepts/
//      new-rejects fail-before/pass-after on real noise. Under CTRL_ACK_FLOOR_FAILBEFORE
//      the control path reverts to the data floor and this SAME noise buffer is accepted
//      => FAILBEFORE-DEMONSTRATED (the spurious-teardown admission path);
//   F  every control code (not only CLOSE) routes through the strict gate: the same
//      gray-band noise buffer is REJECTED for SET_CONFIG / SWITCH_BANDWIDTH /
//      SET_LINK_PARAMS / START_CONNECTION with the gate on, and ACCEPTED for SET_CONFIG
//      with MERCURY_CTRL_ACK_STRICT=0 semantics (fail-before). A strong clean ACK is
//      still accepted for every code. The commander's production control-ACK wait
//      (process_messages_rx_acks_control) is driven once on the noise buffer for
//      SET_CONFIG and must NOT mark the control frame ACKED.
//   R  the production control-ACK wait (process_messages_rx_acks_control) per code on a
//      pure-noise buffer the data floor accepts, no override knob: gate on = no code is
//      ACKED on noise, the clean ACK is ACKED; MERCURY_CTRL_ACK_STRICT=0 = the previous
//      behaviour (noise ACKED for every code except CLOSE).
//   S  the same wait on the SNR-suffix branch (turbo_snr_ack_enabled, the upward
//      SET_CONFIG path): noise must not accept, including the 500 ms suffix-timeout
//      accept; a clean bare ACK (suffix-timeout) and a clean ACK+SNR both accept.
//   K  the emergency-BREAK ACK poll (break_ack_poll): noise refused, clean ACK accepted.
//   Built with CTRL_ACK_BASE_TREE against the previous tree, R/S/K FAIL (fail-before).
//   E  a large-n pure-WGN margin measurement: P(matched>=7 && metric>=1.2) and the noise
//      metric tail + the energy-concentration separation (noise vs a real ACK), to
//      quantify the floor margin rather than assert it.

#include "datalink_layer/arq.h"
#include "datalink_layer/datalink_defines.h"
#include "physical_layer/telecom_system.h"
#include "physical_layer/mfsk.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <random>
#include <vector>
#include <algorithm>
#include "common/sim_clock.h"

int cl_arq_controller::test_ctrl_ack_noise_rejection()
{
	const char* NM = "TEST-CTRL-ACK-NOISE";
	const double CTRL_FLOOR = (double)cl_mfsk::CTRL_DETECT_METRIC_MIN; // 1.2
	const double DATA_FLOOR = 0.5;
#ifdef CTRL_ACK_BASE_TREE
	const double CONC_FLOOR = 0.35;   // previous CLOSE floor
#endif
	const unsigned SEED_BASE = 0xA5A50000u;
	int fails = 0;

	cl_telecom_system* ts = new cl_telecom_system();
	cl_arq_controller* cmd = new cl_arq_controller();
	ts->operation_mode = ARQ_MODE;
	ts->narrowband_enabled = NO;
	cmd->telecom_system = ts;
	cmd->role = COMMANDER;
	cmd->narrowband_enabled = NO;
	cmd->current_configuration = CONFIG_NONE;
	cmd->load_configuration(ROBUST_0, FULL, NO);
	cmd->link_status = CONNECTED;
	cmd->connection_status = RECEIVING_ACKS_CONTROL;
	cmd->turbo_snr_ack_enabled = false;
	cmd->messages_control.data[0] = CLOSE_CONNECTION; // the code the call site flags strict for

	const int ack_base_total = ts->ack_mfsk.ack_base_total_nsymb();
	const int pattern_len = ack_base_total;
	const int tail_nsymb = ack_base_total + pattern_len + 16;
	const int sym_samples = ts->data_container.Nofdm * ts->data_container.interpolation_rate;
	const int signal_period = sym_samples * ts->data_container.buffer_Nsymb.load();
	int tail_samples = tail_nsymb * sym_samples;
	if (tail_samples > signal_period) tail_samples = signal_period;
	const int ack_samples = ts->ack_pattern_passband_samples;
	const int match_thr = ts->ack_mfsk.ack_match_threshold;
#ifndef CTRL_ACK_BASE_TREE
	const double CONC_FLOOR = ts->ack_mfsk.ack_conc_floor;   // derived per geometry (cl_mfsk::ack_conc_floor_for)
#endif

	if (ack_samples <= 0 || ack_samples > tail_samples
	    || ts->data_container.passband_delayed_data == NULL) {
		printf("[%s] FAIL setup: ack_samples=%d tail_samples=%d ring=%p\n",
		       NM, ack_samples, tail_samples, (void*)ts->data_container.passband_delayed_data);
		delete cmd; delete ts; return 1;
	}
	printf("[%s] ctrl_floor(CTRL_DETECT_METRIC_MIN)=%.2f data_floor(ack_metric_threshold)=%.3f match_thr=%d\n",
	       NM, CTRL_FLOOR, cmd->ack_metric_threshold, match_thr);

	std::vector<double> ack((size_t)ack_samples, 0.0);
	if (ts->generate_ack_pattern_passband(ack.data()) != ack_samples) {
		printf("[%s] FAIL: ACK generation length mismatch\n", NM);
		delete cmd; delete ts; return 1;
	}

	// Stage: Gaussian noise over the full ring, optional clean ACK (scaled) at the
	// newest tail. Mirrors both halves of the 2*signal_period ring.
	auto stage = [&](double ack_scale, double noise_amp, std::mt19937& rng) {
		double* ring = ts->data_container.passband_delayed_data;
		std::normal_distribution<double> nd(0.0, 1.0);
		for (int i = 0; i < 2 * signal_period; i++) ring[i] = noise_amp * nd(rng);
		if (ack_scale != 0.0) {
			const int off = signal_period - ack_samples;
			for (int i = 0; i < ack_samples; i++) {
				ring[off + i] += ack_scale * ack[(size_t)i];
				ring[signal_period + off + i] += ack_scale * ack[(size_t)i];
			}
		}
		ts->data_container.ring_write_index = 0;
		ts->data_container.frames_to_read = 0;
		ts->data_container.data_ready = 1;
	};

	// Production control-ACK accept (control_ack_strict = CLOSE path); report peak matched/metric.
	auto accept = [&](bool strict, int* out_m, double* out_met) -> bool {
		cmd->ack_diag_peak_matched = 0;
		cmd->ack_diag_peak_metric = 0.0;
		cmd->ack_diag_poll_count = 0;
		ts->data_container.frames_to_read = 0;
		ts->data_container.data_ready = 1;
		bool acc = cmd->receive_ack_pattern(false, false, -1, strict);
		if (out_m) *out_m = cmd->ack_diag_peak_matched;
		if (out_met) *out_met = cmd->ack_diag_peak_metric;
		return acc;
	};

	// ---- B: strong clean ACK accepted; record real-ACK energy concentration. ----
	double real_conc = -1.0;
	{
		std::mt19937 rb(0xB0u);
		int m = 0; double met = 0.0;
		stage(1.0, 0.0008, rb);
		bool acc = accept(/*strict=*/true, &m, &met);
		if (m > 0) real_conc = met / (double)m;
		printf("[%s] B strong-ACK: accepted=%d matched=%d metric=%.3f concentration(met/matched)=%.3f (want accept)\n",
		       NM, acc ? 1 : 0, m, met, real_conc);
		if (!acc) { printf("[%s] FAIL B: strict floor rejected a strong clean ACK\n", NM); fails++; }
	}

	// ---- C: a weak-but-valid ACK whose energy clears the control floor stays accepted. ----
	{
		double noise_amp = 0.02;
		double weak_metric = -1.0; int weak_m = 0; double weak_scale = 0.0; bool weak_acc = false;
		for (double scale = 1.0; scale >= 0.08; scale -= 0.04) {
			std::mt19937 r2(0x5EEDu);
			int m = 0; double met = 0.0;
			stage(scale, noise_amp, r2);
			bool acc = accept(/*strict=*/true, &m, &met);
#ifdef CTRL_ACK_BASE_TREE
			const bool clears = (met >= CTRL_FLOOR);
#else
			const bool clears = (m >= match_thr && m > 0 && met / m >= CONC_FLOOR);   // derived gate
#endif
			if (clears) { weak_metric = met; weak_m = m; weak_scale = scale; weak_acc = acc; }
		}
		printf("[%s] C weakest-valid-ACK: scale=%.2f matched=%d metric=%.3f conc=%.3f accepted=%d (want accept; conc_floor=%.4f)\n",
		       NM, weak_scale, weak_m, weak_metric, weak_m > 0 ? weak_metric / weak_m : 0.0, weak_acc ? 1 : 0, CONC_FLOOR);
		if (weak_metric < 0.0) {
			printf("[%s] WARN C: no ACK scale produced metric>=control_floor in sweep (harness-regime)\n", NM);
		} else if (!weak_acc) {
			printf("[%s] FAIL C: strict floor rejected a weak ACK that clears the control floor\n", NM);
			fails++;
		}
	}

	// ---- D: PURE-WGN gray-band witness — old floor ACCEPTS, control floor REJECTS. ----
	// Search noise seeds for a buffer whose production detection has matched>=match_thr
	// and metric in [DATA_FLOOR, CTRL_FLOOR): the old data floor admits it (false-ACK),
	// the control floor rejects it. This is the direct fail-before/pass-after on real noise.
	bool gray_found = false;
	unsigned gray_seed = 0;
	{
		const double noise_amp = 0.05;
		bool found = false;
		unsigned wseed = 0;
		int band_m = 0; double band_met = 0.0;
		for (unsigned seed = 1; seed <= 4000u && !found; seed++) {
			std::mt19937 rs(SEED_BASE + seed);
			int m = 0; double met = 0.0;
			stage(0.0, noise_amp, rs);
			accept(/*strict=*/true, &m, &met); // read the production peak matched/metric
			if (m >= match_thr && met >= DATA_FLOOR && met < CTRL_FLOOR) {
				found = true; wseed = seed; band_m = m; band_met = met;
				gray_found = true; gray_seed = seed;
			}
		}
		if (!found) {
			printf("[%s] WARN D: no pure-WGN buffer produced matched>=%d with metric in [%.2f,%.2f) in 4000 seeds\n",
			       NM, match_thr, DATA_FLOOR, CTRL_FLOOR);
		} else {
			int m0 = 0, m1 = 0; double met0 = 0.0, met1 = 0.0;
			std::mt19937 rA(SEED_BASE + wseed); stage(0.0, noise_amp, rA);
			bool acc_data = accept(/*strict=*/false, &m0, &met0); // OLD data floor
			std::mt19937 rB(SEED_BASE + wseed); stage(0.0, noise_amp, rB);
			bool acc_ctrl = accept(/*strict=*/true, &m1, &met1);  // control floor (reverted under FAILBEFORE)
			printf("[%s] D gray-band pure-WGN seed=%u matched=%d metric=%.3f | data_floor_accept=%d ctrl_floor_accept=%d\n",
			       NM, wseed, band_m, band_met, acc_data ? 1 : 0, acc_ctrl ? 1 : 0);
			if (!acc_data) {
				printf("[%s] FAIL D: a [%.2f,%.2f) noise buffer was NOT accepted by the data floor "
				       "(cannot reproduce the pre-fix accept)\n", NM, DATA_FLOOR, CTRL_FLOOR);
				fails++;
			}
#ifdef CTRL_ACK_FLOOR_FAILBEFORE
			if (acc_ctrl) {
				printf("[%s] FAILBEFORE-DEMONSTRATED (control-ACK reverted to data floor: a pure-WGN "
				       "[%.2f,%.2f) correlation was accepted -> spurious teardown admission path)\n",
				       NM, DATA_FLOOR, CTRL_FLOOR);
			} else {
				printf("[%s] FAIL D(defeat): expected the reverted data floor to accept the noise buffer\n", NM);
			}
			fails++; // defeat build returns nonzero DISTINCTLY
#else
			if (acc_ctrl) {
				printf("[%s] FAIL D: control floor accepted a pure-WGN [%.2f,%.2f) correlation "
				       "(gate change did not fire)\n", NM, DATA_FLOOR, CTRL_FLOOR);
				fails++;
			}
#endif
		}
	}

	// ---- F: every control code routes through the strict gate. ----
#if !defined(CTRL_ACK_FLOOR_FAILBEFORE) && !defined(CTRL_ACK_BASE_TREE)
	if (!gray_found) {
		printf("[%s] WARN F: no gray-band noise buffer from D; F skipped\n", NM);
	} else {
		const double noise_amp = 0.05;
		const unsigned char codes[] = { SET_CONFIG, SWITCH_BANDWIDTH, SET_LINK_PARAMS,
		                                START_CONNECTION, CLOSE_CONNECTION };
		for (int force = 1; force >= 0; force--) {
			cmd->ctrl_ack_strict_force = force;
			for (unsigned char code : codes) {
				const bool strict = cmd->control_ack_strict_for(code);
				int m = 0; double met = 0.0;
				std::mt19937 rn(SEED_BASE + gray_seed); stage(0.0, noise_amp, rn);
				const bool acc_noise = accept(strict, &m, &met);
				std::mt19937 rs(0xB0u); stage(1.0, 0.0008, rs);
				const bool acc_real = accept(strict, nullptr, nullptr);
				const bool want_strict = (force == 1) || (code == CLOSE_CONNECTION);
				printf("[%s] F gate=%s code=0x%02X strict=%d noise(matched=%d metric=%.3f) accept=%d real_accept=%d\n",
				       NM, force ? "on" : "off", (unsigned)code, strict ? 1 : 0, m, met,
				       acc_noise ? 1 : 0, acc_real ? 1 : 0);
				if (strict != want_strict) {
					printf("[%s] FAIL F: code=0x%02X strict=%d want %d\n", NM, (unsigned)code, strict ? 1 : 0, want_strict ? 1 : 0);
					fails++;
				}
				if (want_strict && acc_noise) {
					printf("[%s] FAIL F: code=0x%02X accepted a pure-WGN gray-band buffer with the strict gate\n", NM, (unsigned)code);
					fails++;
				}
				if (!acc_real) {
					printf("[%s] FAIL F: code=0x%02X rejected a strong clean ACK\n", NM, (unsigned)code);
					fails++;
				}
				if (force == 0 && code == SET_CONFIG) {
					if (acc_noise)
						printf("[%s] F FAILBEFORE-DEMONSTRATED: gate off, SET_CONFIG accepted the pure-WGN buffer\n", NM);
					else {
						printf("[%s] FAIL F: gate off did not reproduce the SET_CONFIG noise accept\n", NM);
						fails++;
					}
				}
			}
		}
		// Production control-ACK wait, gate on: a SET_CONFIG wait polled on the noise
		// buffer must not mark the control frame ACKED.
		cmd->ctrl_ack_strict_force = 1;
		cmd->messages_control.data[0] = SET_CONFIG;
		cmd->messages_control.status = PENDING_ACK;
		cmd->ack_pattern_time_ms = 400;
		cmd->receiving_timeout = 60000;
		cmd->receiving_timer.reset();
		cmd->receiving_timer.start();
		{
			std::mt19937 rn(SEED_BASE + gray_seed); stage(0.0, noise_amp, rn);
			cmd->ack_diag_peak_matched = 0; cmd->ack_diag_peak_metric = 0.0; cmd->ack_diag_poll_count = 0;
			cmd->process_messages_rx_acks_control();
			const bool acked = (cmd->messages_control.status == ACKED);
			printf("[%s] F production SET_CONFIG wait on noise: peak matched=%d metric=%.3f acked=%d (want 0)\n",
			       NM, cmd->ack_diag_peak_matched, cmd->ack_diag_peak_metric, acked ? 1 : 0);
			if (acked) { printf("[%s] FAIL F: production SET_CONFIG wait ACKED on noise\n", NM); fails++; }
		}
		cmd->ctrl_ack_strict_force = -1;
		cmd->messages_control.data[0] = CLOSE_CONNECTION;
	}
#endif

	// ---- R, S, K: the PRODUCTION control-ACK consumers on a pure-noise buffer. ----
	// No override knob: the gate state comes from MERCURY_CTRL_ACK_STRICT exactly as
	// in a live run. gate_on (env unset) = every control wait must refuse the noise
	// buffer and accept the clean ACK; MERCURY_CTRL_ACK_STRICT=0 = the previous
	// behaviour must come back (the noise buffer is ACKED for every code except CLOSE),
	// which is the fail-before on this binary. Built on the previous tree
	// (CTRL_ACK_BASE_TREE) the gate_on expectations FAIL: the base-tree fail-before.
	{
		const char* senv = std::getenv("MERCURY_CTRL_ACK_STRICT");
		const bool gate_on = !(senv && *senv == '0');
		const double noise_amp = 0.05;
		static uint8_t r_play_mem[65536];
		const bool r_own_play = (playback_buffer == NULL);
		if (r_own_play) playback_buffer = circular_buf_init(r_play_mem, sizeof(r_play_mem));
		auto arm_wait = [&](unsigned char code) {
			cmd->messages_control.data[0] = code;
			cmd->messages_control.status = PENDING_ACK;
			cmd->link_status = CONNECTED;
			cmd->connection_status = RECEIVING_ACKS_CONTROL;
			cmd->ack_pattern_time_ms = 400;
			cmd->receiving_timeout = 60000;
			cmd->receiving_timer.reset();
			cmd->receiving_timer.start();
			cmd->ack_diag_peak_matched = 0; cmd->ack_diag_peak_metric = 0.0; cmd->ack_diag_poll_count = 0;
		};

		// R: plain branch. Gray-band seed = a noise buffer the data floor accepts.
		unsigned rseed = 0; int rm = 0; double rmet = 0.0;
		for (unsigned seed = 1; seed <= 4000u && rseed == 0u; seed++) {
			std::mt19937 rs(SEED_BASE + seed);
			int m = 0; double met = 0.0;
			stage(0.0, noise_amp, rs);
			accept(false, &m, &met);
			if (m >= match_thr && met >= DATA_FLOOR) { rseed = seed; rm = m; rmet = met; }
		}
		if (rseed == 0u) {
			printf("[%s] FAIL R: no data-floor noise buffer in 4000 seeds\n", NM); fails++;
		} else {
			printf("[%s] R noise seed=%u matched=%d metric=%.3f conc=%.3f (data floor accepts)\n",
			       NM, rseed, rm, rmet, rm > 0 ? rmet / rm : 0.0);
			const unsigned char rcodes[] = { SET_CONFIG, SWITCH_BANDWIDTH, SET_LINK_PARAMS, CLOSE_CONNECTION };
			for (unsigned char code : rcodes) {
				for (int kind = 0; kind < 2; kind++) {
					std::mt19937 rn(kind == 0 ? SEED_BASE + rseed : 0xB0u);
					if (kind == 0) stage(0.0, noise_amp, rn); else stage(1.0, 0.0008, rn);
					arm_wait(code);
					cmd->process_messages_rx_acks_control();
					const bool acked = (cmd->messages_control.status == ACKED);
					const bool want = (kind == 1) || (!gate_on && code != CLOSE_CONNECTION);
					printf("[%s] R gate=%s code=0x%02X input=%s acked=%d want=%d\n", NM,
					       gate_on ? "on" : "off", (unsigned)code, kind == 0 ? "noise" : "clean-ACK",
					       acked ? 1 : 0, want ? 1 : 0);
					if (acked != want) {
						printf("[%s] FAIL R: code=0x%02X %s acked=%d want %d\n", NM, (unsigned)code,
						       kind == 0 ? "noise" : "clean-ACK", acked ? 1 : 0, want ? 1 : 0);
						fails++;
					} else if (kind == 0 && !gate_on && acked) {
						printf("[%s] R FAILBEFORE-DEMONSTRATED: gate off, code=0x%02X ACKED the noise buffer\n",
						       NM, (unsigned)code);
					}
				}
			}
		}

		// S: SNR-suffix branch (turbo_snr_ack_enabled; armed for SET_CONFIG on an upward
		// gearshift or a turboshift). Previously it accepted on the match count alone and,
		// with no suffix, accepted after the 500 ms suffix wait with no metric floor.
		// The virtual clock drives the 500 ms deterministically.
		{
			const int prior_sim = sim_clock_enabled();
			sim_clock_set_enabled(1);
			const int s_tail_nsymb = ts->ack_mfsk.ack_pattern_nsymb + ts->ack_mfsk.ack_snr_pattern_nsymb() + 16;
			int s_tail = s_tail_nsymb * sym_samples;
			if (s_tail > signal_period) s_tail = signal_period;
			std::vector<double> sbuf((size_t)s_tail, 0.0);
			unsigned sseed = 0; int sm = 0;
			for (unsigned seed = 1; seed <= 4000u && sseed == 0u; seed++) {
				std::mt19937 rs(SEED_BASE + 0x10000u + seed);
				stage(0.0, noise_amp, rs);
				const double* ring = ts->data_container.passband_delayed_data;
				for (int i = 0; i < s_tail; i++) sbuf[(size_t)i] = ring[signal_period - s_tail + i];
				int m = 0; bool valid = false;
				ts->detect_ack_snr_from_passband(sbuf.data(), s_tail, &m, &valid);
				if (m >= match_thr) { sseed = seed; sm = m; }
			}
			const int snr_samples = ts->ack_snr_pattern_passband_samples;
			std::vector<double> snr_ack((size_t)(snr_samples > 0 ? snr_samples : 1), 0.0);
			const bool snr_gen = snr_samples > 0 && snr_samples <= s_tail
				&& ts->generate_ack_snr_pattern_passband(snr_ack.data(), 12.0f) == snr_samples;
			auto stage_snr_ack = [&](std::mt19937& rng) {
				double* ring = ts->data_container.passband_delayed_data;
				std::normal_distribution<double> nd(0.0, 1.0);
				for (int i = 0; i < 2 * signal_period; i++) ring[i] = 0.0008 * nd(rng);
				const int off = signal_period - snr_samples;
				for (int i = 0; i < snr_samples; i++) {
					ring[off + i] += snr_ack[(size_t)i];
					ring[signal_period + off + i] += snr_ack[(size_t)i];
				}
				ts->data_container.ring_write_index = 0;
				ts->data_container.frames_to_read = 0;
				ts->data_container.data_ready = 1;
			};
			// Two polls 600 ms apart on the SAME input (a stationary buffer): poll 1 may
			// start the suffix wait, poll 2 completes the suffix-timeout accept.
			auto run_suffix_wait = [&](int input) -> bool {
				arm_wait(SET_CONFIG);
				cmd->turbo_snr_ack_enabled = true;
				cmd->turbo_snr_defer_timer.stop();
				cmd->turbo_snr_defer_timer.reset();
				for (int poll = 0; poll < 2 && cmd->messages_control.status != ACKED; poll++) {
					if (input == 0) { std::mt19937 rn(SEED_BASE + 0x10000u + sseed); stage(0.0, noise_amp, rn); }
					else if (input == 1) {
						// Bare ACK followed by suffix-length silence + margin, as the ring
						// holds it once the tone has fully arrived (the suffix detector
						// reserves SNR_SUFFIX_LEN symbols after the pattern).
						// Noise above the 8-symbol energy pre-gate (ACK_ENERGY_GATE_RMS
						// 0.001) so the poll reaches the detector with the tone mid-tail.
						std::mt19937 rn(0xB0u);
						stage(0.0, 0.004, rn);
						double* ring = ts->data_container.passband_delayed_data;
						const int off = signal_period - ack_samples - (cl_mfsk::SNR_SUFFIX_LEN + 4) * sym_samples;
						for (int i = 0; i < ack_samples; i++) {
							ring[off + i] += ack[(size_t)i];
							ring[signal_period + off + i] += ack[(size_t)i];
						}
					}
					else { std::mt19937 rn(0xB1u); stage_snr_ack(rn); }
					cmd->process_messages_rx_acks_control();
					sim_clock_add_samples(48ull * 600ull);
				}
				cmd->turbo_snr_ack_enabled = false;
				cmd->turbo_snr_defer_timer.stop();
				cmd->turbo_snr_defer_timer.reset();
				return cmd->messages_control.status == ACKED;
			};
			if (sseed == 0u) {
				printf("[%s] FAIL S: no noise buffer reaching the match threshold on the suffix detector\n", NM); fails++;
			} else {
				const bool n_acked = run_suffix_wait(0);
				const bool want_n = !gate_on;
				printf("[%s] S gate=%s SET_CONFIG suffix-branch noise seed=%u matched=%d acked=%d want=%d\n",
				       NM, gate_on ? "on" : "off", sseed, sm, n_acked ? 1 : 0, want_n ? 1 : 0);
				if (n_acked != want_n) {
					printf("[%s] FAIL S: suffix-branch SET_CONFIG wait on noise acked=%d want %d\n", NM, n_acked ? 1 : 0, want_n ? 1 : 0);
					fails++;
				} else if (!gate_on) {
					printf("[%s] S FAILBEFORE-DEMONSTRATED: gate off, the suffix branch ACKED the noise buffer\n", NM);
				}
			}
			const bool bare_acked = run_suffix_wait(1);   // clean ACK without the SNR suffix: suffix-timeout accept
			printf("[%s] S clean bare ACK (no suffix) acked=%d want=1\n", NM, bare_acked ? 1 : 0);
			if (!bare_acked) { printf("[%s] FAIL S: clean bare ACK not accepted on the suffix branch\n", NM); fails++; }
			if (!snr_gen) {
				printf("[%s] FAIL S: ACK+SNR pattern generation\n", NM); fails++;
			} else {
				const bool snr_acked = run_suffix_wait(2);
				printf("[%s] S clean ACK+SNR suffix acked=%d want=1\n", NM, snr_acked ? 1 : 0);
				if (!snr_acked) { printf("[%s] FAIL S: clean ACK+SNR not accepted\n", NM); fails++; }
			}
			sim_clock_set_enabled(prior_sim);
		}

		// K: the BREAK-ACK poll of the emergency-BREAK state machine.
		if (rseed != 0u) {
			for (int kind = 0; kind < 2; kind++) {
				std::mt19937 rn(kind == 0 ? SEED_BASE + rseed : 0xB0u);
				if (kind == 0) stage(0.0, noise_amp, rn); else stage(1.0, 0.0008, rn);
				cmd->emergency_break_active = 1;
				cmd->connection_status = RECEIVING_ACKS_CONTROL;
#ifdef CTRL_ACK_BASE_TREE
				const bool acc = cmd->receive_ack_pattern();   // the previous production call
#else
				const bool acc = cmd->break_ack_poll();
#endif
				cmd->emergency_break_active = 0;
				const bool want = (kind == 1) || !gate_on;
				printf("[%s] K gate=%s BREAK-ACK poll input=%s accept=%d want=%d\n", NM,
				       gate_on ? "on" : "off", kind == 0 ? "noise" : "clean-ACK", acc ? 1 : 0, want ? 1 : 0);
				if (acc != want) {
					printf("[%s] FAIL K: BREAK-ACK poll %s accept=%d want %d\n", NM,
					       kind == 0 ? "noise" : "clean-ACK", acc ? 1 : 0, want ? 1 : 0);
					fails++;
				} else if (kind == 0 && !gate_on) {
					printf("[%s] K FAILBEFORE-DEMONSTRATED: gate off, the BREAK-ACK poll accepted the noise buffer\n", NM);
				}
			}
		}

		if (r_own_play) playback_buffer = NULL;
		cmd->messages_control.data[0] = CLOSE_CONNECTION;
		cmd->messages_control.status = PENDING_ACK;
		cmd->link_status = CONNECTED;
		cmd->connection_status = RECEIVING_ACKS_CONTROL;
	}

	// ---- E: large-n pure-WGN margin measurement (energy-concentration separation). ----
	// Use the detector directly on independent noise windows (same metric receive_ack_pattern
	// computes). Report the FAR numerators at both floors and the concentration contrast.
	{
		const int N = 4000;
		const double noise_amp = 0.05;
		std::vector<double> buf((size_t)tail_samples, 0.0);
		std::mt19937 re(0xE0FFu);
		std::normal_distribution<double> nd(0.0, 1.0);
		int nmatch = 0, n_ge_data = 0, n_ge_ctrl = 0, n_full = 0;
		double mx = 0.0, conc_sum = 0.0, conc_max = 0.0; int conc_n = 0;
		for (int t = 0; t < N; t++) {
			for (int i = 0; i < tail_samples; i++) buf[(size_t)i] = noise_amp * nd(re);
			int m = 0; uint32_t mask = 0;
			double met = ts->detect_ack_pattern_from_passband(buf.data(), tail_samples, &m, &mask);
			if (met > mx) mx = met;
			if (m >= match_thr) {
				double c = met / (double)m;
				nmatch++;
				if (met >= DATA_FLOOR) n_ge_data++;                          // OLD data floor
				if (met >= CTRL_FLOOR) n_ge_ctrl++;                          // control-floor-only
#ifdef CTRL_ACK_BASE_TREE
				if (met >= CTRL_FLOOR && c >= CONC_FLOOR) n_full++;          // previous CLOSE gate
#else
				if (c >= CONC_FLOOR) n_full++;                               // derived control gate
#endif
				if (c > conc_max) conc_max = c;
				conc_sum += c; conc_n++;
			}
		}
		double noise_conc = (conc_n > 0) ? conc_sum / conc_n : 0.0;
		printf("[%s] E pure-WGN N=%d: matched>=%d in %d; OLD-data-floor(>=%.2f) false-accepts %d; "
		       "metric>=%.2f %d; control gate (conc>=%.4f) false-accepts %d; max_metric=%.3f\n",
		       NM, N, match_thr, nmatch, DATA_FLOOR, n_ge_data, CTRL_FLOOR, n_ge_ctrl, CONC_FLOOR, n_full, mx);
		printf("[%s] E concentration(met/matched): noise_mean=%.3f noise_max=%.3f real_ACK=%.3f conc_floor=%.4f "
		       "(structural separation; noise cannot forge tone concentration)\n",
		       NM, noise_conc, conc_max, real_conc, CONC_FLOOR);
		printf("[%s] E margin: per-poll false-accept OLD=%d/%d ctrl-only=%d/%d FULL=%d/%d\n",
		       NM, n_ge_data, N, n_ge_ctrl, N, n_full, N);
	}

	printf("[%s] %s\n", NM, fails == 0 ? "ALL PASS" : "FAILED");
	delete cmd; delete ts;
	return fails == 0 ? 0 : 1;
}

#ifndef CTRL_ACK_BASE_TREE
// Offline measurement behind the control-ACK concentration floors
// (cl_mfsk::ack_conc_floor_for). --test with MERCURY_CTRL_ACK_MC=wb|nb runs only this.
//   noise:   N independent pure-WGN poll buffers of the production tail length, through
//            the production detectors (plain: detect_ack_pattern_from_passband; suffix:
//            detect_ack_snr_from_passband). Per matched count m, a histogram of
//            conc = metric/m in 0.0025 bins.
//   genuine: G trials per ACK SNR (ACK power over noise in 3 kHz, dB) with the complete
//            tone at a random offset inside the tail, same histograms.
// Knobs (measurement only): MERCURY_CTRL_ACK_MC_N, _SEED, _GEN, _CFG, _SNR_LO/_HI.
// The reduction (floor = lowest conc whose noise-only per-poll false-accept upper
// bound meets the pattern-length budget) is done offline over the printed histograms.
int cl_arq_controller::measure_ctrl_ack_gate(const char* geometry)
{
	const bool nb = geometry && std::strcmp(geometry, "nb") == 0;
	auto envl = [](const char* k, long d) -> long {
		const char* e = std::getenv(k);
		return (e && *e) ? std::atol(e) : d;
	};
	const long N = envl("MERCURY_CTRL_ACK_MC_N", 20000);
	const unsigned seed = (unsigned)envl("MERCURY_CTRL_ACK_MC_SEED", 1);
	const long G = envl("MERCURY_CTRL_ACK_MC_GEN", 0);
	const int cfg = (int)envl("MERCURY_CTRL_ACK_MC_CFG", CONFIG_0);
	const int snr_lo = (int)envl("MERCURY_CTRL_ACK_MC_SNR_LO", -12);
	const int snr_hi = (int)envl("MERCURY_CTRL_ACK_MC_SNR_HI", 14);

	cl_telecom_system* ts = new cl_telecom_system();
	cl_arq_controller* cmd = new cl_arq_controller();
	ts->operation_mode = ARQ_MODE;
	ts->narrowband_enabled = nb ? YES : NO;
	cmd->telecom_system = ts;
	cmd->role = COMMANDER;
	cmd->narrowband_enabled = nb ? YES : NO;
	cmd->current_configuration = CONFIG_NONE;
	cmd->load_configuration(cfg, FULL, NO);

	const cl_mfsk& am = ts->ack_mfsk;
	const int nsymb = am.ack_pattern_nsymb;
	const int thr = am.ack_match_threshold;
	const int sym = ts->data_container.Nofdm * ts->data_container.interpolation_rate;
	const int period = sym * ts->data_container.buffer_Nsymb.load();
	int tailP = (2 * am.ack_base_total_nsymb() + 16) * sym;
	if (tailP > period) tailP = period;
	int tailS = (nsymb + am.ack_snr_pattern_nsymb() + 16) * sym;
	if (tailS > period) tailS = period;
	const double fs = ts->sampling_frequency;
	printf("[CTRL-ACK-MC] geom=%s cfg=%d M=%d Nc=%d nsymb=%d thr=%d floor=%.4f fs=%.0f sym=%d tailP=%d tailS=%d N=%ld seed=%u G=%ld\n",
	       nb ? "nb" : "wb", cmd->current_configuration, am.M, am.Nc, nsymb, thr, am.ack_conc_floor,
	       fs, sym, tailP, tailS, N, seed, G);
	fflush(stdout);

	const int NBIN = 400;   // conc bins of 0.0025 over [0,1)
	const int MMAX = 64;
	std::vector<long> hist((size_t)(MMAX + 1) * NBIN, 0);
	auto clear_hist = [&]() { std::fill(hist.begin(), hist.end(), 0L); };
	auto add = [&](int m, double met) {
		if (m < 0) m = 0;
		if (m > MMAX) m = MMAX;
		double c = (m > 0) ? met / (double)m : 0.0;
		int b = (int)(c / 0.0025);
		if (b < 0) b = 0;
		if (b >= NBIN) b = NBIN - 1;
		hist[(size_t)m * NBIN + b]++;
	};
	auto dump = [&](const char* kind, const char* path, int snr_db, long n) {
		printf("[CTRL-ACK-MC] %s path=%s snr=%d n=%ld\n", kind, path, snr_db, n);
		for (int m = 0; m <= MMAX; m++) {
			long tot = 0;
			for (int b = 0; b < NBIN; b++) tot += hist[(size_t)m * NBIN + b];
			if (tot == 0) continue;
			printf("[CTRL-ACK-MC] %s path=%s snr=%d m=%d tot=%ld bins=", kind, path, snr_db, m, tot);
			if (m >= thr - 3) {
				bool first = true;
				for (int b = 0; b < NBIN; b++) {
					long v = hist[(size_t)m * NBIN + b];
					if (!v) continue;
					printf("%s%d:%ld", first ? "" : ",", b, v);
					first = false;
				}
			}
			printf("\n");
		}
		fflush(stdout);
	};

	std::vector<double> buf((size_t)(tailP > tailS ? tailP : tailS), 0.0);
	std::normal_distribution<double> nd(0.0, 1.0);

	// ---- noise-only ----
	for (int path = 0; path < 2; path++) {
		const int tail = path == 0 ? tailP : tailS;
		std::mt19937 rng(seed * 2654435761u + (unsigned)path * 97u + 1u);
		clear_hist();
		for (long t = 0; t < N; t++) {
			for (int i = 0; i < tail; i++) buf[(size_t)i] = nd(rng);
			int m = 0; double met = 0.0;
			if (path == 0) {
				uint32_t mask = 0;
				met = ts->detect_ack_pattern_from_passband(buf.data(), tail, &m, &mask);
			} else {
				bool valid = false;
				ts->detect_ack_snr_from_passband(buf.data(), tail, &m, &valid, &met);
			}
			add(m, met);
		}
		dump("noise", path == 0 ? "plain" : "suffix", 0, N);
	}

	// ---- genuine tone ----
	if (G > 0) {
		std::vector<double> ackP((size_t)ts->ack_pattern_passband_samples, 0.0);
		std::vector<double> ackS((size_t)ts->ack_snr_pattern_passband_samples, 0.0);
		const int LP = ts->generate_ack_pattern_passband(ackP.data());
		const int LS = ts->generate_ack_snr_pattern_passband(ackS.data(), 12.0f);
		double pP = 0.0, pS = 0.0;
		for (int i = 0; i < LP; i++) pP += ackP[(size_t)i] * ackP[(size_t)i];
		for (int i = 0; i < LS; i++) pS += ackS[(size_t)i] * ackS[(size_t)i];
		pP /= (LP > 0 ? LP : 1);
		pS /= (LS > 0 ? LS : 1);
		const double n3k = 1.0 * 3000.0 / (fs / 2.0);   // unit-variance white noise in 3 kHz
		printf("[CTRL-ACK-MC] genuine LP=%d LS=%d powP=%.6g powS=%.6g n3k=%.6g\n", LP, LS, pP, pS, n3k);
		for (int path = 0; path < 2; path++) {
			const int tail = path == 0 ? tailP : tailS;
			const int L = path == 0 ? LP : LS;
			const double pw = path == 0 ? pP : pS;
			const std::vector<double>& a = path == 0 ? ackP : ackS;
			if (L <= 0 || L > tail || pw <= 0.0) continue;
			for (int s = snr_lo; s <= snr_hi; s++) {
				const double scale = std::sqrt(std::pow(10.0, s / 10.0) * n3k / pw);
				std::mt19937 rng(seed * 2654435761u + (unsigned)(s + 1000) * 131u + (unsigned)path * 7u);
				std::uniform_int_distribution<int> off_d(0, tail - L);
				clear_hist();
				for (long t = 0; t < G; t++) {
					for (int i = 0; i < tail; i++) buf[(size_t)i] = nd(rng);
					const int off = off_d(rng);
					for (int i = 0; i < L; i++) buf[(size_t)(off + i)] += scale * a[(size_t)i];
					int m = 0; double met = 0.0;
					if (path == 0) {
						uint32_t mask = 0;
						met = ts->detect_ack_pattern_from_passband(buf.data(), tail, &m, &mask);
					} else {
						bool valid = false;
						ts->detect_ack_snr_from_passband(buf.data(), tail, &m, &valid, &met);
					}
					add(m, met);
				}
				dump("genuine", path == 0 ? "plain" : "suffix", s, G);
			}
		}
	}
	printf("[CTRL-ACK-MC] done\n");
	fflush(stdout);
	delete cmd; delete ts;
	return 0;
}
#endif
