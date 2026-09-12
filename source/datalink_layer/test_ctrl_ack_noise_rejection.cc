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

int cl_arq_controller::test_ctrl_ack_noise_rejection()
{
	const char* NM = "TEST-CTRL-ACK-NOISE";
	const double CTRL_FLOOR = (double)cl_mfsk::CTRL_DETECT_METRIC_MIN; // 1.2
	const double DATA_FLOOR = 0.5;
	const double CONC_FLOOR = 0.35; // must match CTRL_ACK_CONCENTRATION_MIN in arq_common.cc
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
			if (met >= CTRL_FLOOR) { weak_metric = met; weak_m = m; weak_scale = scale; weak_acc = acc; }
		}
		printf("[%s] C weakest-valid-ACK: scale=%.2f matched=%d metric=%.3f accepted=%d (want accept, metric>=%.2f)\n",
		       NM, weak_scale, weak_m, weak_metric, weak_acc ? 1 : 0, CTRL_FLOOR);
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
				if (met >= CTRL_FLOOR && c >= CONC_FLOOR) n_full++;          // FULL new gate
				if (c > conc_max) conc_max = c;
				conc_sum += c; conc_n++;
			}
		}
		double noise_conc = (conc_n > 0) ? conc_sum / conc_n : 0.0;
		printf("[%s] E pure-WGN N=%d: matched>=%d in %d; OLD-data-floor(>=%.2f) false-accepts %d; "
		       "ctrl-floor-only(>=%.2f) false-accepts %d; FULL-gate(>=%.2f && conc>=%.2f) false-accepts %d; max_metric=%.3f\n",
		       NM, N, match_thr, nmatch, DATA_FLOOR, n_ge_data, CTRL_FLOOR, n_ge_ctrl, CTRL_FLOOR, CONC_FLOOR, n_full, mx);
		printf("[%s] E concentration(met/matched): noise_mean=%.3f noise_max=%.3f real_ACK=%.3f conc_floor=%.2f "
		       "(structural separation; noise cannot forge tone concentration)\n",
		       NM, noise_conc, conc_max, real_conc, CONC_FLOOR);
		printf("[%s] E margin: per-poll false-accept OLD=%d/%d ctrl-only=%d/%d FULL=%d/%d\n",
		       NM, n_ge_data, N, n_ge_ctrl, N, n_full, N);
	}

	printf("[%s] %s\n", NM, fails == 0 ? "ALL PASS" : "FAILED");
	delete cmd; delete ts;
	return fails == 0 ? 0 : 1;
}
