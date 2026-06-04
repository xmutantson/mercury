/*
 * finetiming_validator.cc — SHADOW-ONLY ground-truth validator for the
 * OFDM fine-timing peak-selection investigation (wgn0 good-timing collapse).
 *
 * NO PRODUCTION BEHAVIOR CHANGE. This file is reached ONLY via the
 * --validate-finetiming CLI flag, dispatched at the very top of main()
 * (before any audio/GUI/threading/ARQ init). It constructs a private
 * cl_telecom_system, modulates a real CONFIG_0 WB OFDM frame, places it at
 * a KNOWN true sample-delay in an AWGN buffer, and runs ALL THREE fine
 * detectors in shadow (logging only). It opens no audio device.
 *
 * Detectors compared (ofdm.cc):
 *   (1) PRODUCTION self-autocorr  : time_sync_preamble_with_metric (telecom_system.cc:2123 site)
 *   (2) coherent FFT-fine         : time_sync_preamble_fft_fine     (ofdm.cc:2839)
 *   (3) coherent matched-filter   : time_sync_preamble_matched      (ofdm.cc:3036)  [PREFERRED]
 *
 * The matched detector requires ofdm.ofdm_corr_template, whose production
 * build is #if 0 disabled in telecom_system.cc:5825. The validator builds an
 * equivalent template IN-PROCESS (build_ofdm_corr_template_shadow below) so
 * the matched detector has its template — this build is logging/shadow only
 * and never touches the production binary's init path.
 *
 * Output: per-Es/N0 files in <repo>/../ofdm_finetiming_ab/ with the
 * delay-error distribution of every detector, plus a summary file.
 */

#include "physical_layer/telecom_system.h"
#include "common/common_defines.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <complex>
#include <vector>
#include <algorithm>
#include <string>

// ----------------------------------------------------------------------------
// Shadow OFDM matched-filter template build.
// Faithful port of the (production-disabled) telecom_system.cc:5825 block:
//   preamble * pre_eq -> symbol_mod -> boost -> b2p -> peak_clip -> FIR_tx1
//   -> FIR_tx2 -> p2b(FIR_rx_time_sync) -> decimate-by-interp.
// Writes ts.ofdm.ofdm_corr_template{,_len,_nsymb,_sym_energy[],_energy}.
// ----------------------------------------------------------------------------
static void build_ofdm_corr_template_shadow(cl_telecom_system& ts)
{
	cl_ofdm& ofdm = ts.ofdm;
	if(ofdm.ofdm_corr_template != NULL) { delete[] ofdm.ofdm_corr_template; ofdm.ofdm_corr_template = NULL; }

	int template_nsymb = ts.data_container.preamble_nSymb;
	int Nofdm = ts.data_container.Nofdm;
	int bb_len = template_nsymb * Nofdm;
	std::complex<double>* bb_template = new std::complex<double>[bb_len];

	// preamble subcarrier values WITH pre-equalization (matches transmit_bit)
	std::complex<double> preamble_sc[256];
	for(int i = 0; i < template_nsymb; i++)
	{
		for(int k = 0; k < ofdm.Nc; k++)
			preamble_sc[k] = ofdm.ofdm_preamble[i * ofdm.Nc + k].value
				* ts.pre_equalization_channel[k].value;
		ofdm.symbol_mod(preamble_sc, &bb_template[i * Nofdm]);
	}

	double power_normalization = sqrt((double)(ofdm.Nfft * ts.frequency_interpolation_rate));
	double preamble_boost = ofdm.preamble_configurator.boost;
	double ofdm_tx_gain = ts.get_tx_gain(TX_SIG_OFDM);
	for(int i = 0; i < bb_len; i++)
		bb_template[i] = bb_template[i] / power_normalization * sqrt(ts.output_power_Watt) * preamble_boost * ofdm_tx_gain;

	int interp = ts.frequency_interpolation_rate;
	int pb_len = bb_len * interp;
	double* pb_data  = new double[pb_len];
	double* pb_fir1  = new double[pb_len];
	double* pb_fir2  = new double[pb_len];

	long unsigned saved_pss = ofdm.passband_start_sample;
	ofdm.passband_start_sample = 0;
	ofdm.baseband_to_passband(bb_template, bb_len, pb_data,
		ts.sampling_frequency, ts.carrier_frequency, ts.carrier_amplitude, interp);
	ofdm.passband_start_sample = saved_pss;

	ofdm.peak_clip(pb_data, pb_len, ofdm.preamble_papr_cut);
	ofdm.FIR_tx1.apply(pb_data, pb_fir1, pb_len);
	ofdm.FIR_tx2.apply(pb_fir1, pb_fir2, pb_len);

	std::complex<double>* filtered = new std::complex<double>[pb_len];
	ofdm.passband_to_baseband(pb_fir2, pb_len, filtered,
		ts.sampling_frequency, ts.carrier_frequency, ts.carrier_amplitude, 1, &ofdm.FIR_rx_time_sync);

	ofdm.ofdm_corr_template_len = bb_len;
	ofdm.ofdm_corr_template_nsymb = template_nsymb;
	ofdm.ofdm_corr_template = new std::complex<double>[bb_len];
	for(int i = 0; i < bb_len; i++)
		ofdm.ofdm_corr_template[i] = filtered[i * interp];

	ofdm.ofdm_corr_template_energy = 0.0;
	for(int k = 0; k < template_nsymb && k < 16; k++)
	{
		double sym_energy = 0.0;
		for(int n = 0; n < Nofdm; n++)
		{
			int idx = k * Nofdm + n;
			sym_energy +=
				ofdm.ofdm_corr_template[idx].real() * ofdm.ofdm_corr_template[idx].real() +
				ofdm.ofdm_corr_template[idx].imag() * ofdm.ofdm_corr_template[idx].imag();
		}
		ofdm.ofdm_corr_template_sym_energy[k] = sym_energy;
		ofdm.ofdm_corr_template_energy += sym_energy;
	}

	delete[] pb_data; delete[] pb_fir1; delete[] pb_fir2; delete[] filtered; delete[] bb_template;

	printf("[VALIDATOR] OFDM corr template built (shadow): %d symbols, %d samples, energy=%.3f\n",
		template_nsymb, bb_len, ofdm.ofdm_corr_template_energy);
	fflush(stdout);
}

// ----------------------------------------------------------------------------
// Measure the pilot-residual noise_variance + mean_|H| the production decode
// path would compute IF the FFT window were placed at `full_rate_delay`.
// Faithful minimal port of receive_byte's OFDM branch (telecom_system.cc:
// extract decimated frame from delay -> symbol_demod (skip preamble) -> AGC ->
// CPE -> ZF/LS estimator -> read ofdm.noise_variance_estimate + mean|H|).
// This ties delay-error to decode rejection (the documented nv/ mean_H signature).
// ----------------------------------------------------------------------------
struct NvResult { double nv; double mean_H; int h_count; };

static NvResult measure_nv_mean_h(cl_telecom_system& ts,
                                  std::complex<double>* bb_full, int buf_samp,
                                  int full_rate_delay,
                                  std::vector<std::complex<double>>& dec_scratch)
{
	cl_ofdm& ofdm = ts.ofdm;
	int interp     = ts.frequency_interpolation_rate;
	int Nofdm      = ts.data_container.Nofdm;
	int Nsymb      = ts.data_container.Nsymb;
	int pre_nsymb  = ts.data_container.preamble_nSymb;
	int frame_full = (pre_nsymb + Nsymb) * Nofdm * interp;

	NvResult r; r.nv = -1.0; r.mean_H = -1.0; r.h_count = 0;

	if(full_rate_delay < 0) full_rate_delay = 0;
	if(full_rate_delay + frame_full > buf_samp)
		full_rate_delay = buf_samp - frame_full;
	if(full_rate_delay < 0) return r;   // buffer too small for this delay

	// Decimate the full-rate baseband [delay, delay+frame_full) by interp into
	// base-rate baseband (Nofdm*(pre+Nsymb) samples). Mirrors production's
	// rational_resampler(baseband_data_interpolated -> baseband_data).
	int base_len = (pre_nsymb + Nsymb) * Nofdm;
	if((int)dec_scratch.size() < base_len) dec_scratch.resize(base_len);
	ofdm.rational_resampler(&bb_full[full_rate_delay], frame_full,
		dec_scratch.data(), interp, DECIMATION);

	// Demodulate DATA symbols (skip the preamble_nSymb symbols), as production.
	std::vector<std::complex<double>> demod((size_t)Nsymb * ofdm.Nc);
	for(int i = 0; i < Nsymb; i++)
		ofdm.symbol_demod(&dec_scratch[(size_t)i * Nofdm + (size_t)pre_nsymb * Nofdm],
		                  &demod[(size_t)i * ofdm.Nc]);

	ofdm.automatic_gain_control(demod.data());
	ofdm.CPE_correction(demod.data());

	if(ofdm.channel_estimator == LEAST_SQUARE)
		ofdm.LS_channel_estimator(demod.data());
	else
		ofdm.ZF_channel_estimator(demod.data());

	r.nv = ofdm.noise_variance_estimate;

	double h_sum = 0; int h_count = 0;
	for(int ci = 0; ci < ofdm.Nsymb * ofdm.Nc; ci++)
	{
		if(ofdm.estimated_channel[ci].status == MEASURED)
		{
			h_sum += std::abs(ofdm.estimated_channel[ci].value);
			h_count++;
		}
	}
	if(h_count > 0) r.mean_H = h_sum / h_count;
	r.h_count = h_count;
	return r;
}

// Percentile helper (data sorted ascending)
static double pct(std::vector<double>& v, double p)
{
	if(v.empty()) return 0.0;
	int idx = (int)(p * (v.size() - 1) + 0.5);
	if(idx < 0) idx = 0;
	if(idx >= (int)v.size()) idx = (int)v.size() - 1;
	return v[idx];
}

int run_finetiming_validator(const char* out_dir_arg)
{
	printf("=== OFDM FINE-TIMING GROUND-TRUTH VALIDATOR (shadow, no audio) ===\n");
	fflush(stdout);

	// --- 1. Build a CONFIG_0 WB OFDM telecom_system (no -R: real OFDM PHY) ---
	cl_telecom_system ts;
	ts.operation_mode = BER_PLOT_passband;     // matches the production BER setup path
	ts.narrowband_enabled = NO;                // wideband
	ts.load_configuration(CONFIG_0);           // sets sampling_freq, carrier, interp, bw, FIRs, preamble, pilots
	ts.output_power_Watt = 1;                  // same as BER_PLOT_passband_process_main

	int M_mod      = ts.data_container.M;
	int Nofdm      = ts.data_container.Nofdm;
	int Nsymb      = ts.data_container.Nsymb;
	int pre_nsymb  = ts.data_container.preamble_nSymb;
	int interp     = ts.frequency_interpolation_rate;
	int sym_samp   = Nofdm * interp;                       // full-rate samples per OFDM symbol
	int frame_samp = (pre_nsymb + Nsymb) * sym_samp;       // = data_container.total_frame_size

	printf("[VALIDATOR] CONFIG_0: M=%d Nfft=%d Ngi=%d Nofdm=%d Nsymb=%d preamble_nSymb=%d interp=%d\n",
		M_mod, ts.ofdm.Nfft, ts.data_container.Ngi, Nofdm, Nsymb, pre_nsymb, interp);
	printf("[VALIDATOR] sampling_freq=%.1f carrier=%.1f bandwidth=%.1f sym_samp=%d frame_samp=%d\n",
		ts.sampling_frequency, ts.carrier_frequency, ts.bandwidth, sym_samp, frame_samp);
	fflush(stdout);

	// Build the matched-filter template (shadow) so detector (3) is functional.
	build_ofdm_corr_template_shadow(ts);

	// --- 2. Generate ONE clean TX passband CONFIG_0 frame (preamble+pilots+data) ---
	int nReal_data = ts.data_container.nBits - ts.ldpc.P;
	int frame_size_bytes = (nReal_data - ts.outer_code_reserved_bits) / 8;
	if(frame_size_bytes < 1) frame_size_bytes = 1;
	std::vector<int> data_bytes(frame_size_bytes);
	for(int i = 0; i < frame_size_bytes; i++) data_bytes[i] = (i * 37 + 11) & 0xFF;

	std::vector<double> tx_frame(frame_samp, 0.0);
	ts.transmit_byte(data_bytes.data(), frame_size_bytes, tx_frame.data(), SINGLE_MESSAGE);

	// Measure clean signal power for Es/N0 -> sigma calibration (matches BER path:
	// sigma = sqrt(2 * P_sig * f_nyquist / (10^(EsN0/10) * bandwidth)))
	double P_sig = 0.0;
	for(int i = 0; i < frame_samp; i++) P_sig += tx_frame[i] * tx_frame[i];
	P_sig /= frame_samp;
	double f_nyquist = ts.sampling_frequency / 2.0;

	// --- 3. Buffer geometry: place frame at a known true delay inside a wider
	//        AWGN buffer (mirrors the idle-scan: preamble embedded in noise). ---
	// Buffer holds several symbols of noise before + after the frame.
	int lead_symbols  = 4;   // noise symbols before the frame
	int trail_symbols = 4;   // noise symbols after the frame
	int true_delay = lead_symbols * sym_samp;                 // GROUND TRUTH (full-rate samples)
	int buf_samp   = (lead_symbols + trail_symbols) * sym_samp + frame_samp;

	// pream_symb_loc the coarse stage would report (true symbol index of preamble start)
	int true_pream_symb_loc = lead_symbols;

	// Detector search window mirrors production site-8:
	//   window starts at (pream_symb_loc-1)*Nofdm*interp, length (preamble_nSymb+4)*Nofdm*interp.
	// We seed the coarse position with the TRUE symbol boundary +/- jitter; the
	// fine detectors then must resolve sub-symbol AND wrong-adjacent-boundary.
	int win_start = (true_pream_symb_loc - 1) * sym_samp;
	if(win_start < 0) win_start = 0;
	int win_len   = (pre_nsymb + 4) * sym_samp;
	if(win_start + win_len > buf_samp) win_len = buf_samp - win_start;

	// Es/N0 sweep (mandate: -4,-6,-8,-10 spanning the marginal/mis-place region)
	double esn0_points[] = { -4.0, -6.0, -8.0, -10.0 };
	int n_esn0 = (int)(sizeof(esn0_points)/sizeof(esn0_points[0]));
	int n_trials = 240;   // >= 200 independent noise realizations per Es/N0

	// half-symbol threshold in full-rate samples for the "mis-place" criterion
	double half_sym = 0.5 * sym_samp;
	double one_sym  = 1.0 * sym_samp;

	// Output dir
	std::string out_dir = out_dir_arg ? out_dir_arg
		: "x:/Storage/Documents/hermes and mercury/ofdm_finetiming_ab";

	// Scratch buffers
	std::vector<double> rx_pb(buf_samp);
	std::vector<std::complex<double>> bb(buf_samp);   // full-rate baseband for fine detectors
	std::vector<std::complex<double>> bb_dec(buf_samp / interp + 4); // decimated for coarse halfsym

	// Summary accumulators
	std::string summary;
	char line[1024];
	snprintf(line, sizeof(line),
		"# OFDM fine-timing detector A/B — CONFIG_0 WB, true_delay=%d samp (symbol %d), sym_samp=%d, half_sym=%.1f\n"
		"# detectors: PROD=self-autocorr(with_metric loc0), FFTFINE=fft_fine, MATCHED=matched-filter\n"
		"# columns: esn0_dB n_trials | <det>_misplace_>1sym_frac <det>_misplace_>0.5sym_frac <det>_median_abs_err <det>_p90_abs_err <det>_mean_nv_at_delay\n",
		true_delay, true_pream_symb_loc, sym_samp, half_sym);
	summary += line;

	// global seed for reproducibility
	srand(12345);

	// KEY conditional metric: on frames where PROD mis-places (>1 sym), what
	// fraction does MATCHED land within +-0.5 sym of true?
	struct KeyAgg { int prod_misplace=0; int matched_resolves_on_prod_fail=0;
	                int fftfine_resolves_on_prod_fail=0; };

	for(int e = 0; e < n_esn0; e++)
	{
		double EsN0 = esn0_points[e];
		float sigma = (float)sqrt(2.0 * P_sig * f_nyquist / (pow(10.0, EsN0 / 10.0) * ts.bandwidth));

		std::vector<double> err_prod, err_fft, err_matched;     // signed delay error (samp)
		std::vector<double> nv_prod;                            // pilot residual nv at PROD delay (marginal-region calibration)
		std::vector<double> nv_at_prod_v, nv_at_true_v;         // nv at prod-chosen vs true delay
		std::vector<double> mh_at_prod_v, mh_at_true_v;         // mean|H| at prod-chosen vs true delay
		int prod_miss_1   = 0, fft_miss_1   = 0, matched_miss_1   = 0;   // >1 sym
		int prod_miss_05  = 0, fft_miss_05  = 0, matched_miss_05  = 0;   // >0.5 sym
		KeyAgg key;

		FILE* f = NULL;
		{
			char path[1200];
			snprintf(path, sizeof(path), "%s/esn0_%+03d.csv", out_dir.c_str(), (int)EsN0);
			f = fopen(path, "w");
			if(f)
				fprintf(f, "trial,prod_delay,prod_err,prod_corr,fftfine_delay,fftfine_err,fftfine_corr,matched_delay,matched_err,matched_corr,nv_at_prod,meanH_at_prod,nv_at_true,meanH_at_true\n");
		}

		for(int t = 0; t < n_trials; t++)
		{
			// (a) fill whole buffer with pure AWGN (passband), real-valued.
			float ampl_val = sigma; // apply() uses ampl/sqrt2 for I and Q; passband is real -> use sigma directly /sqrt2 conv:
			// Match BER passband noise variance: each real sample gets sigma/sqrt(2)*N(0,1)*sqrt(2)?  The double
			// apply_with_delay adds (sigma/sqrt2)*awgn_value_generator() per real sample. Reproduce that exactly.
			float per_samp = (float)(sigma / sqrtf(2.0f));
			for(int i = 0; i < buf_samp; i++)
				rx_pb[i] = (double)(per_samp * ts.awgn_channel.awgn_value_generator());
			(void)ampl_val;

			// (b) add the clean frame on top at the known true delay (signal + noise)
			for(int i = 0; i < frame_samp; i++)
				rx_pb[true_delay + i] += tx_frame[i];

			// (c) mix passband -> full-rate baseband with FIR_rx_time_sync (decimation_rate=1),
			//     exactly as production site-8 prepares the fine-search input.
			ts.ofdm.passband_to_baseband(rx_pb.data(), buf_samp, bb.data(),
				ts.sampling_frequency, ts.carrier_frequency, ts.carrier_amplitude,
				1, &ts.ofdm.FIR_rx_time_sync);

			// ---- DETECTOR 1: PRODUCTION self-autocorr (site-8 semantics) ----
			// Run with_metric over the same window [win_start, win_start+win_len)
			// at step=1, location_to_return=0 (first-trial == GLOBAL MAX peak).
			TimeSyncResult prod = ts.ofdm.time_sync_preamble_with_metric(
				&bb[win_start], win_len, interp, /*location_to_return=*/0,
				/*step=*/1, /*nTrials_max=*/ts.time_sync_trials_max);
			int prod_delay = win_start + prod.delay;
			double prod_err = (double)(prod_delay - true_delay);

			// ---- DETECTOR 2: coherent FFT-fine ----
			// coarse_pos = true symbol boundary (what the coarse stage reports);
			// search_half_window = +-1 symbol so wrong-adjacent-boundary is reachable.
			TimeSyncResult fft = ts.ofdm.time_sync_preamble_fft_fine(
				bb.data(), buf_samp, interp, pre_nsymb,
				/*coarse_pos=*/true_delay, /*search_half_window=*/sym_samp);
			int fft_delay = fft.delay;
			double fft_err = (double)(fft_delay - true_delay);

			// ---- DETECTOR 3: coherent matched-filter [PREFERRED] ----
			// matched runs its own coarse(GI-stride)+fine(sample-stride) over the
			// whole buffer; amplitude-independent, no external coarse seed needed.
			TimeSyncResult mf = ts.ofdm.time_sync_preamble_matched(
				bb.data(), buf_samp, interp, pre_nsymb);
			int mf_delay = mf.delay;
			double mf_err = (double)(mf_delay - true_delay);

			// ---- pilot-residual nv + mean_|H| the decode path would see ----
			// At the PROD-chosen delay (the decisive timing) AND at the TRUE delay
			// (the reference). Reproduces the documented signature: wrong window ->
			// nv blows up (1.76-7.4) + mean_H collapses (~0.29) -> SKIP-H rejects;
			// right window -> nv ~0.70-1.17 + mean_H ~0.84-0.86 -> LDPC decodes.
			NvResult nv_prod_r = measure_nv_mean_h(ts, bb.data(), buf_samp, prod_delay, bb_dec);
			NvResult nv_true_r = measure_nv_mean_h(ts, bb.data(), buf_samp, true_delay, bb_dec);
			double nv_at_prod = nv_prod_r.nv;

			err_prod.push_back(prod_err);
			err_fft.push_back(fft_err);
			err_matched.push_back(mf_err);
			nv_prod.push_back(nv_at_prod);
			if(nv_prod_r.nv >= 0)  nv_at_prod_v.push_back(nv_prod_r.nv);
			if(nv_true_r.nv >= 0)  nv_at_true_v.push_back(nv_true_r.nv);
			if(nv_prod_r.mean_H >= 0) mh_at_prod_v.push_back(nv_prod_r.mean_H);
			if(nv_true_r.mean_H >= 0) mh_at_true_v.push_back(nv_true_r.mean_H);

			if(fabs(prod_err) > one_sym)  prod_miss_1++;
			if(fabs(fft_err)  > one_sym)  fft_miss_1++;
			if(fabs(mf_err)   > one_sym)  matched_miss_1++;
			if(fabs(prod_err) > half_sym) prod_miss_05++;
			if(fabs(fft_err)  > half_sym) fft_miss_05++;
			if(fabs(mf_err)   > half_sym) matched_miss_05++;

			// KEY conditional: on PROD >1-sym mis-place, does MATCHED / FFTFINE
			// land within +-0.5 sym of true?
			if(fabs(prod_err) > one_sym)
			{
				key.prod_misplace++;
				if(fabs(mf_err)  <= half_sym) key.matched_resolves_on_prod_fail++;
				if(fabs(fft_err) <= half_sym) key.fftfine_resolves_on_prod_fail++;
			}

			if(f)
				fprintf(f, "%d,%d,%.1f,%.4f,%d,%.1f,%.4f,%d,%.1f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
					t, prod_delay, prod_err, prod.correlation,
					fft_delay, fft_err, fft.correlation,
					mf_delay, mf_err, mf.correlation,
					nv_prod_r.nv, nv_prod_r.mean_H, nv_true_r.nv, nv_true_r.mean_H);
		}
		if(f) fclose(f);

		// stats
		auto abs_sorted = [](std::vector<double> v){
			for(double& x : v) x = fabs(x);
			std::sort(v.begin(), v.end()); return v; };
		std::vector<double> ap = abs_sorted(err_prod);
		std::vector<double> af = abs_sorted(err_fft);
		std::vector<double> am = abs_sorted(err_matched);

		double prod_med = pct(ap, 0.5), prod_p90 = pct(ap, 0.9);
		double fft_med  = pct(af, 0.5), fft_p90  = pct(af, 0.9);
		double mf_med   = pct(am, 0.5), mf_p90   = pct(am, 0.9);

		double key_matched_frac = key.prod_misplace > 0
			? (double)key.matched_resolves_on_prod_fail / key.prod_misplace : -1.0;
		double key_fft_frac = key.prod_misplace > 0
			? (double)key.fftfine_resolves_on_prod_fail / key.prod_misplace : -1.0;

		auto med_of = [&](std::vector<double> v){ std::sort(v.begin(), v.end()); return pct(v, 0.5); };
		double nv_prod_med = med_of(nv_at_prod_v);
		double nv_true_med = med_of(nv_at_true_v);
		double mh_prod_med = med_of(mh_at_prod_v);
		double mh_true_med = med_of(mh_at_true_v);

		snprintf(line, sizeof(line),
			"EsN0=%+.1f n=%d | "
			"PROD: >1sym=%.3f >0.5sym=%.3f med=%.0f p90=%.0f | "
			"FFTFINE: >1sym=%.3f >0.5sym=%.3f med=%.0f p90=%.0f | "
			"MATCHED: >1sym=%.3f >0.5sym=%.3f med=%.0f p90=%.0f | "
			"KEY[on PROD>1sym fails(%d)]: MATCHED_resolves<=0.5sym=%.3f FFTFINE_resolves<=0.5sym=%.3f | "
			"nv_med[prod-delay=%.3f true-delay=%.3f] meanH_med[prod-delay=%.3f true-delay=%.3f]\n",
			EsN0, n_trials,
			(double)prod_miss_1/n_trials, (double)prod_miss_05/n_trials, prod_med, prod_p90,
			(double)fft_miss_1/n_trials,  (double)fft_miss_05/n_trials,  fft_med,  fft_p90,
			(double)matched_miss_1/n_trials, (double)matched_miss_05/n_trials, mf_med, mf_p90,
			key.prod_misplace, key_matched_frac, key_fft_frac,
			nv_prod_med, nv_true_med, mh_prod_med, mh_true_med);
		summary += line;
		printf("%s", line);
		fflush(stdout);
	}

	// write summary
	{
		char path[1200];
		snprintf(path, sizeof(path), "%s/SUMMARY.txt", out_dir.c_str());
		FILE* fs = fopen(path, "w");
		if(fs) { fputs(summary.c_str(), fs); fclose(fs);
			printf("[VALIDATOR] wrote %s\n", path); }
		else   { printf("[VALIDATOR] WARNING: could not open %s for write\n", path); }
		fflush(stdout);
	}

	return 0;
}
