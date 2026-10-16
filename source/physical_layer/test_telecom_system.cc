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


#include "physical_layer/telecom_system.h"
#include "common/snr_decision_grid.h"
#include "common/sim_channel.h"
#include "physical_layer/dist_matcher.h"
#include "debug/canary_guard.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

cl_error_rate cl_telecom_system::baseband_test_EsN0(float EsN0,int max_frame_no)
{
	cl_error_rate lerror_rate;
	float power_normalization=sqrt((double)ofdm.Nfft);
	float sigma=1.0/sqrt(pow(10,(EsN0/10)));
	float variance=1.0/(pow(10,(EsN0/10)));
	int nVirtual_data;
	int nReal_data;
	int delay=0;
	nVirtual_data=ldpc.N-data_container.nBits;
	nReal_data=data_container.nBits-ldpc.P;

	int constellation_plot_counter=0;
	int constellation_plot_nFrames=10;
	float contellation[ofdm.pilot_configurator.nData*constellation_plot_nFrames][2]={0};

	while(lerror_rate.Frames_total<max_frame_no)
	{
		for(int i=0;i<nReal_data;i++)
		{
			data_container.data_bit[i]=ts_random()%2;   // §10.1 per-instance when opted in
		}
		for(int i=0;i<nVirtual_data;i++)
		{
			data_container.data_bit[nReal_data+i]=data_container.data_bit[i];
		}

		ldpc.encode(data_container.data_bit,data_container.encoded_data);

		for(int i=0;i<ldpc.P;i++)
		{
			data_container.encoded_data[nReal_data+i]=data_container.encoded_data[i+ldpc.K];
		}

		interleaver(data_container.encoded_data,data_container.bit_interleaved_data,data_container.nBits,bit_interleaver_block_size);

		psk.mod(data_container.bit_interleaved_data,data_container.nBits,data_container.modulated_data);
		interleaver(data_container.modulated_data, data_container.ofdm_time_freq_interleaved_data, data_container.nData, time_freq_interleaver_block_size);
		ofdm.framer(data_container.ofdm_time_freq_interleaved_data,data_container.ofdm_framed_data);

		for(int i=0;i<data_container.Nsymb;i++)
		{
			ofdm.symbol_mod(&data_container.ofdm_framed_data[i*data_container.Nc],&data_container.ofdm_symbol_modulated_data[i*data_container.Nofdm]);
		}

		for(int j=0;j<data_container.Nofdm*data_container.Nsymb;j++)
		{
			data_container.ofdm_symbol_modulated_data[j]/=power_normalization;
		}


		awgn_channel.apply_with_delay(data_container.ofdm_symbol_modulated_data,data_container.baseband_data,sigma,data_container.Nofdm*data_container.Nsymb,0);


		for(int j=0;j<data_container.Nofdm*data_container.Nsymb;j++)
		{
			data_container.baseband_data[j]*=power_normalization;
		}

		for(int i=0;i<data_container.Nsymb;i++)
		{
			ofdm.symbol_demod(&data_container.baseband_data[delay*0+i*data_container.Nofdm],&data_container.ofdm_symbol_demodulated_data[i*data_container.Nc]);
		}

		if(ofdm.channel_estimator==ZERO_FORCE)
		{
			ofdm.ZF_channel_estimator(data_container.ofdm_symbol_demodulated_data);
		}
		else if (ofdm.channel_estimator==LEAST_SQUARE)
		{
			ofdm.LS_channel_estimator(data_container.ofdm_symbol_demodulated_data);
		}

		if(ofdm.channel_estimator_amplitude_restoration==YES)
		{
			ofdm.restore_channel_amplitude();
			ofdm.channel_equalizer_without_amplitude_restoration(data_container.ofdm_symbol_demodulated_data,data_container.equalized_data_without_amplitude_restoration);
			ofdm.deframer(data_container.equalized_data_without_amplitude_restoration,data_container.ofdm_deframed_data_without_amplitude_restoration);
		}

		ofdm.channel_equalizer(data_container.ofdm_symbol_demodulated_data,data_container.equalized_data);

		variance=ofdm.measure_variance(data_container.ofdm_symbol_demodulated_data);

		ofdm.deframer(data_container.equalized_data,data_container.ofdm_deframed_data);
		deinterleaver(data_container.ofdm_deframed_data, data_container.ofdm_time_freq_deinterleaved_data, data_container.nData, time_freq_interleaver_block_size);
		psk.demod(data_container.ofdm_time_freq_deinterleaved_data,data_container.nBits,data_container.demodulated_data,variance);

		deinterleaver(data_container.demodulated_data,data_container.deinterleaved_data,data_container.nBits,bit_interleaver_block_size);


		for(int i=ldpc.P-1;i>=0;i--)
		{
			data_container.deinterleaved_data[i+nReal_data+nVirtual_data]=data_container.deinterleaved_data[i+nReal_data];
		}

		for(int i=0;i<nVirtual_data;i++)
		{
			data_container.deinterleaved_data[nReal_data+i]=data_container.deinterleaved_data[i];
		}


		ldpc.decode(data_container.deinterleaved_data,data_container.hd_decoded_data_bit);

		// Always use fully-equalized data for visualization (tight clusters).
		// Amplitude restoration still helps the decoder via the separate path.
		for(int i=0;i<ofdm.pilot_configurator.nData;i++)
		{
			contellation[constellation_plot_counter*ofdm.pilot_configurator.nData+i][0]=data_container.ofdm_deframed_data[i].real();
			contellation[constellation_plot_counter*ofdm.pilot_configurator.nData+i][1]=data_container.ofdm_deframed_data[i].imag();
		}

		constellation_plot_counter++;

		if(constellation_plot_counter==constellation_plot_nFrames)
		{
			constellation_plot_counter=0;
			constellation_plot.plot_constellation(&contellation[0][0],ofdm.pilot_configurator.nData*constellation_plot_nFrames);
		}


		lerror_rate.check(data_container.data_bit,data_container.hd_decoded_data_bit,nReal_data);
	}
	return lerror_rate;
}

// P1 ACQ BAND-EXCLUSION directed regression (runs in --test). Drives the REAL
// production predicate acq_band_excl_hit() (the same call receive_byte() makes at
// the sub-peak reject site), so the exclusion logic the storm fix relies on is
// exercised, not simulated. Synthetic-fire house pattern (test_batch_coast /
// test_subpeak_rescue). The end-to-end fire proof (a stormed cell COMPLETING) is
// the stored-seed cohort A/B — this test proves the DECISION fires correctly.
//   FAIL-BEFORE (no exclusion memory): the first reject at the plateau delay is a
//     MISS (returns false) — base behavior, the wrong point is restored & re-locked.
//   PASS-AFTER (exclusion memory): a SECOND reject in the same band is a HIT
//     (returns true) → the caller forces a clean re-acquire, breaking the loop.
//   0=PASS, 1=FAIL.
int cl_telecom_system::test_acq_band_excl()
{
	int failed = 0;
	auto check = [&](bool cond, const char* name) {
		if(cond) printf("[TEST-ACQ-EXCL] PASS: %s\n", name);
		else   { printf("[TEST-ACQ-EXCL] FAIL: %s\n", name); failed++; }
		fflush(stdout);
	};

	// Real cfg15 geometry so the derived (radius<=0) default is a real symbol span.
	operation_mode = BER_PLOT_passband;
	load_configuration(CONFIG_15);
	int sym_full = data_container.Nofdm * frequency_interpolation_rate;
	int radius = 2 * sym_full;               // explicit radius (matches the default)
	printf("[TEST-ACQ-EXCL] cfg15 sym_full=%d radius=%d\n", sym_full, radius);
	fflush(stdout);
	check(sym_full > 0, "cfg15 geometry loaded (sym_full>0)");

	// Fresh capture/batch epoch.
	acq_band_excl_begin_epoch();

	// The field storm plateau band (~141k-147k full-rate samples, ~21 offsets).
	const int PLATEAU = 142309;              // refuter-observed pinned delay
	int mc = -1;

	// FAIL-BEFORE: first reject at the plateau is a MISS (records the band).
	bool first = acq_band_excl_hit(PLATEAU, radius, &mc);
	check(!first, "FAIL-BEFORE: first plateau reject is a MISS (base restores wrong point)");
	check(acq_excl_count == 1, "band recorded (count 0->1)");

	// PASS-AFTER: a repeat reject at the SAME delay is a HIT → force re-acquire.
	mc = -1;
	bool repeat = acq_band_excl_hit(PLATEAU, radius, &mc);
	check(repeat, "PASS-AFTER: repeat plateau reject is a HIT (forces clean re-acquire)");
	check(mc == PLATEAU, "HIT reports the matched band center");

	// A nearby delay within the band (same wrong-lock, ±half symbol) also HITs.
	mc = -1;
	bool near_hit = acq_band_excl_hit(PLATEAU + sym_full, radius, &mc);   // within 2*sym_full
	check(near_hit, "in-band neighbor (+1 symbol) is a HIT (BAND, not a single delay)");

	// A delay well outside the band is a MISS and opens a NEW band (not one delay).
	mc = -1;
	int far_delay = PLATEAU + 40 * sym_full;
	bool far_hit = acq_band_excl_hit(far_delay, radius, &mc);
	check(!far_hit, "far delay (out of band) is a MISS => new band, not a global block");
	check(acq_excl_count == 2, "second band opened (count 1->2)");

	// Directed stale-ring regression: this relative delay now contains a valid
	// preamble in a NEW batch. Starting that epoch must discard the former band's
	// exclusion, so the production predicate cannot suppress the new candidate.
	// FAIL-BEFORE: the stale ring reports HIT. PASS-AFTER: the epoch reset makes it
	// a MISS (eligible for the normal preamble/decode path).
	acq_band_excl_begin_epoch();
	mc = -1;
	bool after_reset = acq_band_excl_hit(PLATEAU, radius, &mc);
	check(!after_reset, "new batch accepts valid preamble at formerly-excluded relative delay");
	check(acq_excl_count == 1, "new epoch records no stale centers (candidate opens a fresh band)");

	printf("[TEST-ACQ-EXCL] %s (%d failure%s)\n", failed==0?"ALL PASS":"FAILURES", failed, failed==1?"":"s");
	fflush(stdout);
	// This acquisition entry is already invoked by the aggregate test dispatcher.
	cl_telecom_system contracts;
	failed += contracts.test_rx_contracts();
	failed += contracts.test_agc_receive_entry();
	return failed == 0 ? 0 : 1;
}

int cl_telecom_system::test_rx_shm_output_overflow()
{
	static const char *ENV = "MERCURY_RX_SHM_FAIL_CLOSED";
	const char *old_env = std::getenv(ENV);
	const bool had_old_env = old_env != NULL;
	const std::string saved_env = had_old_env ? old_env : "";
	auto set_flag = [&](const char *value) { setenv(ENV, value, 1); };
	auto restore_flag = [&]() {
		if(had_old_env) setenv(ENV, saved_env.c_str(), 1);
		else unsetenv(ENV);
	};

	uint8_t storage[8] = {};
	uint8_t fill[sizeof(storage)] = {};
	uint8_t decoded = 0x5a;
	cbuf_handle_t output = circular_buf_init(storage, sizeof(storage));
	int failures = 0;
	if(write_buffer(output, fill, sizeof(fill)) != 0 || !circular_buf_full(output))
	{
		printf("[TEST-RX-SHM-OVERFLOW] FAIL: could not fill output fixture\n");
		circular_buf_free(output);
		restore_flag();
		return 1;
	}

	// Fail-before arm: default-off retains the historical stdout-only signal.
	set_flag("0");
	cl_telecom_system legacy;
	const bool legacy_delivered = legacy.RX_SHM_deliver_decoded_frame(
		output, &decoded, sizeof(decoded));
	const uint64_t legacy_drops = legacy.RX_SHM_output_drop_count();
	if(legacy_delivered || legacy_drops != 0 || !circular_buf_full(output))
	{
		printf("[TEST-RX-SHM-OVERFLOW] FAIL-BEFORE reproduction failed "
		       "(delivered=%d drops=%llu full=%d)\n",
		       legacy_delivered ? 1 : 0,
		       (unsigned long long)legacy_drops,
		       circular_buf_full(output) ? 1 : 0);
		failures++;
	}

	// Pass-after arm: the same full-buffer decode is now machine-observable.
	set_flag("1");
	cl_telecom_system fixed;
	const bool fixed_delivered = fixed.RX_SHM_deliver_decoded_frame(
		output, &decoded, sizeof(decoded));
	const uint64_t fixed_drops = fixed.RX_SHM_output_drop_count();
	if(fixed_delivered || fixed_drops != 1 || !circular_buf_full(output))
	{
		printf("[TEST-RX-SHM-OVERFLOW] PASS-AFTER failed "
		       "(delivered=%d drops=%llu full=%d)\n",
		       fixed_delivered ? 1 : 0,
		       (unsigned long long)fixed_drops,
		       circular_buf_full(output) ? 1 : 0);
		failures++;
	}

	// Capacity available remains a successful delivery and cannot raise the
	// error counter.
	circular_buf_reset(output);
	if(!fixed.RX_SHM_deliver_decoded_frame(output, &decoded, sizeof(decoded))
	   || fixed.RX_SHM_output_drop_count() != 1 || size_buffer(output) != 1)
	{
		printf("[TEST-RX-SHM-OVERFLOW] FAIL: successful-delivery control\n");
		failures++;
	}

	circular_buf_free(output);
	restore_flag();
	printf("[TEST-RX-SHM-OVERFLOW] %s (legacy drops=%llu, enabled drops=%llu)\n",
	       failures == 0 ? "PASS" : "FAIL",
	       (unsigned long long)legacy_drops,
	       (unsigned long long)fixed.RX_SHM_output_drop_count());
	return failures;
}

int run_snr_decision_grid_selftest()
{
	int fails = 0;
	int threshold_cases = 0;
	auto check = [&](bool condition, const char* what) {
		if(!condition)
		{
			fails++;
			printf("[TEST-SNR-GRID] FAIL: %s\n", what);
		}
	};
	auto aligned = [](double value) {
		return std::isfinite(value)
			&& std::fabs(value * 100.0 - std::round(value * 100.0)) < 1e-9;
	};
	auto strict_pair = [&](const char* consumer, double threshold) {
		const double epsilon = 1e-12;
		const double below = threshold - epsilon;
		const double above = threshold + epsilon;
		char msg[192];
		snprintf(msg, sizeof(msg), "%s threshold is centi-dB aligned", consumer);
		check(aligned(threshold), msg);
		snprintf(msg, sizeof(msg), "%s raw perturbation straddles strict threshold", consumer);
		check((below > threshold) != (above > threshold), msg);
		snprintf(msg, sizeof(msg), "%s quantized perturbation has one decision", consumer);
		check((quantize_snr_decision_db(below) > threshold)
			== (quantize_snr_decision_db(above) > threshold), msg);
		threshold_cases++;
	};

	// Complete strict-threshold inventory from the accepted-SNR consumer audit.
	const double gear[] = {-9, -8, -7, -6, -5, -4, -3, -2, 0, 1, 3, 4, 6, 9, 11, 13};
	for(double threshold : gear) strict_pair("gear-map", threshold);
	strict_pair("sentinel-validity", -90.0);
	strict_pair("cfg16-margin", CFG16_MIN_SNR_DB);

	const double rung_floor[] = {10, 12, 14, 16, 22, 24};
	const double rung_bump[] = {0, RUNG_FLOOR_BUMP_STEP_DB, 2 * RUNG_FLOOR_BUMP_STEP_DB,
		3 * RUNG_FLOOR_BUMP_STEP_DB, RUNG_FLOOR_BUMP_MAX_DB};
	for(double floor_db : rung_floor)
		for(double bump_db : rung_bump)
			strict_pair("rung-floor-derived", floor_db + bump_db);

	for(double threshold : gear)
		strict_pair("elevator-margin-derived", threshold + SUPERSHIFT_MARGIN_DB);
	for(double threshold : gear)
		strict_pair("connect-seed-margin-derived", threshold + CONNECT_SEED_MARGIN_DB);
	strict_pair("topgear-margin-derived", 13.0 + TOPGEAR_ELECT_MARGIN_DB);
	strict_pair("topgear-floor", 24.0);
	strict_pair("turbo-best/config", 13.0 + SUPERSHIFT_MARGIN_DB);

	// The report/tone encoders change bins only at odd integer-dB boundaries.
	for(int q = 0; q <= 15; q++)
		strict_pair("topgear/turbo-2dB-bin", -5.0 + 2.0 * q);

	check(quantize_snr_decision_db(0.004999999999) == 0.0,
		"positive value below half-centi rounds to zero");
	check(quantize_snr_decision_db(0.005) == 0.01,
		"positive half-centi rounds away from zero");
	check(quantize_snr_decision_db(0.005000000001) == 0.01,
		"positive value above half-centi rounds up");
	check(quantize_snr_decision_db(-0.004999999999) == 0.0,
		"negative value above half-centi rounds to zero");
	check(quantize_snr_decision_db(-0.005) == -0.01,
		"negative half-centi rounds away from zero");
	check(quantize_snr_decision_db(-0.005000000001) == -0.01,
		"negative value below half-centi rounds down");
	check(quantize_snr_decision_db(-99.9) == -99.9,
		"unmeasured sentinel is preserved");
	check(std::isnan(quantize_snr_decision_db(NAN)), "NaN is preserved");
	check(quantize_snr_decision_db(INFINITY) == INFINITY, "infinity is preserved");

	// Derived seams: integer margins and half-dB bump accumulation remain on
	// the grid; max/hold selects an existing cell rather than creating a value.
	const double cell = quantize_snr_decision_db(13.371);
	check(aligned(cell - SUPERSHIFT_MARGIN_DB), "elevator margin preserves grid");
	check(aligned(cell - CONNECT_SEED_MARGIN_DB), "connect margin preserves grid");
	check(aligned(12.0 + 3 * RUNG_FLOOR_BUMP_STEP_DB), "rung bump sum preserves grid");
	const double held = std::max(quantize_snr_decision_db(12.999999999999),
		quantize_snr_decision_db(13.000000000001));
	check(held == 13.0 && aligned(held), "held maximum preserves grid cell");

	// A float32 legacy-wire round trip is normalized again at decode ingress.
	const double wire_source = quantize_snr_decision_db(13.371);
	const double wire_decode = quantize_snr_decision_db((double)(float)wire_source);
	check(wire_decode == wire_source, "legacy float32 decode returns to source grid cell");

	printf("[TEST-SNR-GRID] %s: %d strict cases, %d failure%s\n",
		fails == 0 ? "PASS" : "FAILED", threshold_cases, fails, fails == 1 ? "" : "s");
	fflush(stdout);
	return fails;
}

int cl_telecom_system::test_subpeak_gate()
{
	printf("[TEST-SUBPEAK] reverse-MFSK false-lock admission gate (pream in-band/passband ratio)\n");
	int fails = 0;

	// --- helper: mean per-sample passband + decimated-baseband energy over the
	// preamble region, computed EXACTLY as the ENERGY-DIAG block. ---
	auto pream_ratio = [&](double* passband, int total_pb_samples) -> double {
		int de_M        = data_container.interpolation_rate;
		int pream_len   = data_container.preamble_nSymb * data_container.Nofdm * frequency_interpolation_rate;
		int pream_len_dec = data_container.preamble_nSymb * data_container.Nofdm; // de_sym_dec = Nofdm
		if(pream_len > total_pb_samples) pream_len = total_pb_samples;
		// passband mean energy over preamble (matches pb_pream)
		double pb = 0.0;
		for(int i = 0; i < pream_len; i++) { double v = passband[i]; pb += v*v; }
		pb /= pream_len;
		// production decimation into baseband_data_decimated (matches bb_pream source)
		int dec_len = total_pb_samples / de_M;
		ofdm.passband_to_baseband_decimated(passband, total_pb_samples,
			data_container.baseband_data_decimated, sampling_frequency,
			carrier_frequency, carrier_amplitude, de_M, &ofdm.FIR_rx_time_sync);
		double bb = 0.0;
		int cnt = 0;
		for(int i = 0; i < pream_len_dec && i < dec_len; i++) {
			double re = data_container.baseband_data_decimated[i].real();
			double im = data_container.baseband_data_decimated[i].imag();
			bb += re*re + im*im; cnt++;
		}
		bb = (cnt > 0) ? bb / cnt : 0.0;
		return (pb > 0.0) ? bb / pb : -1.0;
	};

	int pream_pb_samples = data_container.preamble_nSymb * data_container.Nofdm * frequency_interpolation_rate;
	// Generous scratch buffer for both signals (preamble length + slack).
	int buf_n = pream_pb_samples + 4 * data_container.Nofdm * frequency_interpolation_rate;
	double* sig = new double[buf_n];

	// (1) REAL WB OFDM preamble: synthesize the preamble passband exactly as
	// transmit_byte does (baseband_to_passband of the modulated preamble symbols).
	{
		for(int i = 0; i < buf_n; i++) sig[i] = 0.0;
		// Build the OFDM preamble baseband symbols (mirrors transmit_byte:782-840).
		for(int i = 0; i < data_container.preamble_nSymb * ofdm.Nc; i++)
			data_container.preamble_data[i] = ofdm.ofdm_preamble[i].value;
		for(int i = 0; i < data_container.preamble_nSymb; i++)
			ofdm.symbol_mod(&data_container.preamble_data[i*data_container.Nc],
			                &data_container.preamble_symbol_modulated_data[i*data_container.Nofdm]);
		ofdm.baseband_to_passband(data_container.preamble_symbol_modulated_data,
			data_container.Nofdm*data_container.preamble_nSymb, sig,
			sampling_frequency, carrier_frequency, carrier_amplitude, frequency_interpolation_rate);
		double r_real = pream_ratio(sig, pream_pb_samples);
		bool admit = !subpeak_reject_out_of_band(/*bb*/r_real, /*pb*/1.0); // ratio passed via bb,pb=1
		printf("[TEST-SUBPEAK]   (1) REAL WB OFDM preamble: pream bb/pb=%.4f thr=%.2f -> %s (expect ADMIT, ratio<thr)\n",
			r_real, SUBPEAK_PREAM_RATIO_REJECT, admit ? "ADMIT_OK" : "REJECTED_REGRESSION");
		if(!(r_real >= 0.0 && r_real < SUBPEAK_PREAM_RATIO_REJECT && admit)) fails++;

		// (3) NO-REGRESSION sweep: drive the SAME real preamble progressively
		// QUIETER (cumulative scale-down to ~1e-7 of full scale, far below the
		// CFG15 working point). The pream bb/pb ratio is amplitude-INVARIANT (bb
		// and pb scale together), so a real low-SNR preamble must STILL admit at
		// every level. This is the catastrophic-acquisition-regression ship-blocker.
		bool sweep_ok = true;
		double worst = -1.0;
		const double scales[] = {0.5, 0.5, 0.4, 0.5, 0.4, 0.5}; // cumulative ~0.5 .. ~5e-3
		for(double s : scales) {
			for(int i = 0; i < pream_pb_samples; i++) sig[i] *= s; // cumulative scale-down
			double rr = pream_ratio(sig, pream_pb_samples);
			if(rr > worst) worst = rr;
			bool a = (rr >= 0.0 && rr < SUBPEAK_PREAM_RATIO_REJECT);
			if(!a) sweep_ok = false;
		}
		printf("[TEST-SUBPEAK]   (3) NO-REGRESSION amplitude sweep (real preamble quieted to ~5e-3 FS, worst ratio=%.4f): %s\n",
			worst, sweep_ok ? "ALL-ADMIT_OK" : "REJECTED_LOW_SNR_REGRESSION");
		if(!sweep_ok) fails++;
	}

	// (2) Reverse-MFSK FALSE-LOCK proxy: a narrowband single tone at the carrier
	// (the MFSK burst is a concentrated single-tone signal). Its in-band energy
	// survives the decimation nearly intact -> HIGH bb/pb.
	{
		for(int i = 0; i < buf_n; i++) sig[i] = 0.0;
		// MFSK preamble tone sits at carrier_frequency; emit a pure tone there.
		double w = 2.0 * M_PI * carrier_frequency / sampling_frequency;
		for(int i = 0; i < pream_pb_samples; i++)
			sig[i] = carrier_amplitude * cos(w * i);
		double r_false = pream_ratio(sig, pream_pb_samples);
		bool reject = subpeak_reject_out_of_band(/*bb*/r_false, /*pb*/1.0);
		printf("[TEST-SUBPEAK]   (2) narrowband MFSK-burst proxy: pream bb/pb=%.4f thr=%.2f -> %s (expect REJECT, ratio>=thr)\n",
			r_false, SUBPEAK_PREAM_RATIO_REJECT, reject ? "REJECT_OK" : "ADMITTED_BUG");
		if(!(r_false >= SUBPEAK_PREAM_RATIO_REJECT && reject)) fails++;
	}

	// (4) Decision against the exact HW-measured populations (tt_cmd.log, §1.1):
	// REAL pream ratio 0.565-0.584 must ADMIT; false-lock 0.904-0.907 must REJECT.
	{
		const double real_hw[]  = {0.5649, 0.5763, 0.5843};
		const double false_hw[] = {0.9042, 0.9059, 0.9065};
		bool ok = true;
		for(double v : real_hw)  if(subpeak_reject_out_of_band(v, 1.0)) ok = false; // must NOT reject
		for(double v : false_hw) if(!subpeak_reject_out_of_band(v, 1.0)) ok = false; // must reject
		printf("[TEST-SUBPEAK]   (4) HW-population decision (REAL 0.565-0.584 admit, false 0.904-0.907 reject): %s\n",
			ok ? "SEPARATES_OK" : "MISCLASSIFY");
		if(!ok) fails++;
	}

	// (5) FAIL-BEFORE/PASS-AFTER at the EXACT production gate-site condition
	// (`subpeak_admission_reject(bb,pb)`). With the HW false-lock pream ratio
	// (0.9059): env-OFF the site is bypassed => the burst is ADMITTED as a
	// forward OFDM frame (=> Moose garbage / SKIP-VAR / aliased BREAK = the bug);
	// env-ON the site fires => the burst is REJECTED pre-Moose (=> the fix).
	{
		const double bb_false = 0.9059, pb_false = 1.0;  // HW false-lock signature
		const double bb_real  = 0.5763, pb_real  = 1.0;  // HW real-preamble signature
		bool gate_on = subpeak_metric_gate_enabled();
		bool site_rejects_false = subpeak_admission_reject(bb_false, pb_false);
		bool site_rejects_real  = subpeak_admission_reject(bb_real,  pb_real);
		// Invariant in BOTH env states: a REAL preamble is NEVER rejected by the site.
		bool ok = (!site_rejects_real)
		          && (gate_on ? site_rejects_false : !site_rejects_false);
		printf("[TEST-SUBPEAK]   (5) production gate-site env-%s: false-lock %s, real-preamble %s -> %s\n",
			gate_on ? "ON" : "OFF",
			site_rejects_false ? "REJECTED(fix)" : "admitted(default-off-bug-exposed)",
			site_rejects_real  ? "REJECTED(REGRESSION)" : "admitted(ok)",
			ok ? (gate_on ? "PASS-AFTER" : "FAIL-BEFORE(expected default-off)") : "UNEXPECTED");
		if(!ok) fails++;
	}

	// (6) The ratio gate must not recreate the discarded 0.65 coarse-metric
	// floor. Current captures include valid CFO-impaired WB preambles at
	// coarse_metric 0.426-0.609; their real-preamble ratios must remain admitted.
	{
		const double coarse_metrics[] = {0.426, 0.500, 0.609};
		const double real_ratios[] = {0.5649, 0.5763, 0.5843};
		bool ok = true;
		for(int i = 0; i < 3; ++i) {
			bool rejected = subpeak_admission_reject(real_ratios[i], 1.0);
			printf("[TEST-SUBPEAK]   (6) valid CFO-impaired WB: coarse=%.3f bb/pb=%.4f -> %s\n",
				coarse_metrics[i], real_ratios[i], rejected ? "REJECTED_REGRESSION" : "ADMIT_OK");
			if(rejected) ok = false;
		}
		if(!ok) fails++;
	}

	delete[] sig;
	printf("[TEST-SUBPEAK] %s (%d failure%s)\n", fails==0?"ALL PASS":"FAILED", fails, fails==1?"":"s");
	return fails==0 ? 0 : 1;
}

// Receiver contracts use production publication/acquisition/detector paths.
// Run MINI and MINI0 in separate processes: their receiver selector is cached.
int cl_telecom_system::test_rx_contracts()
{
	int failed = 0;
	auto check = [&](bool ok, const char* name) {
		printf("[TEST-RX-CONTRACTS] %s: %s\n", ok ? "PASS" : "FAIL", name);
		fflush(stdout);
		if(!ok) ++failed;
	};
	auto seed = [](cl_telecom_system& ts) {
		ts.receive_stats.ofdm_search_raw = 7;
		ts.receive_stats.mfsk_search_raw = 3;
		ts.receive_stats.ofdm_batch_active = true;
		ts.receive_stats.ofdm_drift_per_frame = 2.5;
		ts.receive_stats.delay_of_last_decoded_message = 1234;
	};
	auto reset = [](const cl_telecom_system& ts) {
		const auto& r = ts.receive_stats;
		return r.ofdm_search_raw == 0 && r.mfsk_search_raw == 0
			&& !r.ofdm_batch_active && r.ofdm_drift_per_frame == 0.0
			&& r.delay_of_last_decoded_message == -1;
	};
	auto seeded = [](const cl_telecom_system& ts) {
		const auto& r = ts.receive_stats;
		return r.ofdm_search_raw == 7 && r.mfsk_search_raw == 3
			&& r.ofdm_batch_active && r.ofdm_drift_per_frame == 2.5
			&& r.delay_of_last_decoded_message == 1234;
	};
	auto next_full = [&](cl_telecom_system& ts) {
		int pb = ts.data_container.Nofdm * ts.frequency_interpolation_rate
			* ts.data_container.buffer_Nsymb;
		std::vector<double> silence((size_t)pb, 0.0);
		std::vector<int> out((size_t)N_MAX, 0);
		auto r = ts.receive_byte(silence.data(), out.data());
		check(r.message_decoded != YES && reset(ts)
			&& ts.receive_stats.last_eff_preamble_nsymb == ts.data_container.preamble_nSymb,
			"next receive after actual reset searches FULL without stale grid");
	};

	// Both branches must come from real initialization, including storage/capacity.
	const char* old_defeat = std::getenv("MERCURY_PRECOOK_DEFEAT");
	const bool had_defeat = old_defeat != nullptr;
	const std::string saved_defeat = old_defeat ? old_defeat : "";
	for(int pinned = 0; pinned <= 1; ++pinned)
	{
		setenv("MERCURY_PRECOOK_DEFEAT", pinned ? "0" : "1", 1);
		cl_telecom_system ts;
		ts.operation_mode = BER_PLOT_passband;
		check(ts.load_configuration(CONFIG_15) == 0, "publication fixture loads cfg15");
		ts.precook_pin_shared_ring();
		check(ts.data_container.precook_ring_pinned == (pinned != 0),
			"requested pinned/unpinned publication branch is installed");
		if(ts.data_container.precook_ring_pinned != (pinned != 0)) continue;
		const int natural = ts.data_container.buffer_Nsymb;
		const int floor = natural + 8;
		printf("[TEST-RX-CONTRACTS] ROUTE publication pinned=%d natural=%d floor=%d cap=%d\n",
			pinned, natural, floor, ts.data_container.pinned_capacity_buffer_Nsymb);
		seed(ts);
		ts.data_container.ring_write_index = 2;
		ts.data_container.nUnder_processing_events = 4;
		ts.force_resize_capture_ring(floor);
		check(ts.data_container.buffer_Nsymb == floor
			&& ts.data_container.buffer_Nsymb_min == floor
			&& ts.data_container.ring_write_index == 0,
			"seat witness: active window/floor raised and ring cursor reset");
		check(reset(ts), "seat resets both cursors, active batch, drift and decoded delay");
		next_full(ts);

		// A repeat seat must return before publication, preserving all coordinates.
		seed(ts);
		ts.data_container.ring_write_index = 2;
		ts.data_container.nUnder_processing_events = 4;
		ts.data_container.passband_delayed_data[0] = 0.125;
		ts.data_container.passband_delayed_data[1] = -0.375;
		ts.force_resize_capture_ring(floor);
		check(seeded(ts) && ts.data_container.ring_write_index == 2
			&& ts.data_container.nUnder_processing_events == 4
			&& ts.data_container.buffer_Nsymb == floor
			&& ts.data_container.passband_delayed_data[0] == 0.125
			&& ts.data_container.passband_delayed_data[1] == -0.375,
			"already-seated no-op preserves grid, ring cursor, samples and window");

		seed(ts);
		ts.force_set_capture_ring_natural();
		check(ts.data_container.buffer_Nsymb == natural
			&& ts.data_container.buffer_Nsymb_min == 0
			&& ts.data_container.ring_write_index == 0,
			"unseat witness: natural active window and reset ring cursor");
		check(reset(ts), "unseat resets all five receiver-grid fields");
		next_full(ts);

		seed(ts);
		check(ts.load_configuration(CONFIG_15) == 0 && seeded(ts),
			"same-configuration load is a grid-preserving no-op");
		check(ts.load_configuration(CONFIG_0) == 0 && reset(ts),
			"actual configuration transition resets all five receiver-grid fields");
		next_full(ts);
	}
	if(had_defeat) setenv("MERCURY_PRECOOK_DEFEAT", saved_defeat.c_str(), 1);
	else unsetenv("MERCURY_PRECOOK_DEFEAT");

	operation_mode = BER_PLOT_passband;
	check(load_configuration(CONFIG_15) == 0, "receive fixture loads cfg15");
	const int sym = data_container.Nofdm * frequency_interpolation_rate;
	const int pb = sym * data_container.buffer_Nsymb;
	if(sym <= 0 || pb <= 0) return 1;
	std::vector<int> out((size_t)N_MAX, 0);
	std::vector<double> win((size_t)pb, 0.0);
	for(int i = 0; i < pb; ++i) win[(size_t)i] = 1e-3 * sin(0.37 * i);
	win[(size_t)(pb / 2)] = 5.0;
	const std::vector<double> original(win);
	receive_stats.ofdm_batch_active = false;
	receive_stats.ofdm_search_raw = 0;
	receive_byte(win.data(), out.data());
	check(memcmp(win.data(), original.data(), sizeof(double) * (size_t)pb) == 0,
		"caller snapshot remains byte-exact after scaling and impulse clipping");

	// Use the edge-defer route for MINI; silence alone can re-anchor to FULL.
	preamble_amortization_enabled = true;
	ofdm_defer_overflow_enabled = true;
	std::fill(win.begin(), win.end(), 0.0);
	receive_stats.ofdm_batch_active = true;
	receive_stats.ofdm_search_raw = data_container.buffer_Nsymb - 1;
	receive_stats.ofdm_drift_per_frame = 0.0;
	data_container.nUnder_processing_events = 0;
	const char* mini_env = std::getenv("MERCURY_KEYDOWN_MINI0");
	const int expected = (!mini_env || !*mini_env || atoi(mini_env) != 0) ? 0 : 1;
	// MINI0 bypasses verification; use a complete predicted span so its silence
	// rejection is reached. MINI requires a frame extending beyond the snapshot.
	if(expected == 0) receive_stats.ofdm_search_raw = data_container.preamble_nSymb + 6;
	auto rejected = receive_byte(win.data(), out.data());
	printf("[TEST-RX-CONTRACTS] ROUTE reject mini=%d overflow=%d decoded=%d reported=%d\n",
		expected, rejected.frame_overflow_symbols, rejected.message_decoded,
		receive_stats.last_eff_preamble_nsymb);
	check(rejected.message_decoded != YES && (expected == 0 || rejected.frame_overflow_symbols > 0),
		"MINI early-defer or MINI0 silence-rejection route witnessed");
	check(receive_stats.last_eff_preamble_nsymb == expected,
		"early rejection publishes the actual MINI/MINI0 searched geometry");
	load_configuration(CONFIG_0);
	check(reset(*this), "config reset clears prior reject grid");
	next_full(*this);

	// Real control emitters are positive controls; pristine/post-receive parity
	// compares the same detector state, including its CFO correction.
	last_coarse_freq_offset = 0.0;
	const int ctrl_pb = data_container.Nofdm * frequency_interpolation_rate * data_container.buffer_Nsymb;
	for(int hail = 0; hail <= 1; ++hail)
	{
		std::vector<double> control((size_t)ctrl_pb, 0.0);
		int emitted = hail ? generate_hail_pattern_passband(control.data())
			: generate_break_pattern_passband(control.data());
		check(emitted > 0 && emitted <= ctrl_pb, "real control emitter fits detector snapshot");
		if(emitted <= 0 || emitted > ctrl_pb) continue;
		const std::vector<double> pristine(control);
		int before_count = 0, after_count = 0;
		double before = hail ? detect_hail_pattern_from_passband(control.data(), ctrl_pb, &before_count)
			: detect_break_pattern_from_passband(control.data(), ctrl_pb, &before_count);
		receive_stats.ofdm_batch_active = false;
		receive_stats.ofdm_search_raw = 0;
		receive_byte(control.data(), out.data());
		last_coarse_freq_offset = 0.0;
		double after = hail ? detect_hail_pattern_from_passband(control.data(), ctrl_pb, &after_count)
			: detect_break_pattern_from_passband(control.data(), ctrl_pb, &after_count);
		printf("[TEST-RX-CONTRACTS] ROUTE detector hail=%d emitted=%d matches=%d/%d metric=%.17g/%.17g\n",
			hail, emitted, before_count, after_count, before, after);
		check(before_count > 0 && before > 0.0, "emitted control is detected by production detector");
		check(before_count == after_count && before == after
			&& memcmp(control.data(), pristine.data(), sizeof(double) * (size_t)ctrl_pb) == 0,
			"BREAK/HAIL detector parity and byte-exact snapshot after actual receive");
	}

	// The alternate NB preamble must be selected by the production correlator.
	{
		cl_telecom_system tx, rx;
		tx.operation_mode = rx.operation_mode = BER_PLOT_passband;
		tx.narrowband_enabled = rx.narrowband_enabled = YES;
		check(tx.load_configuration(ROBUST_0) == 0 && rx.load_configuration(ROBUST_0) == 0,
			"NB matched-preamble fixtures initialize");
		tx.set_robust_preamble_negotiated(true);
		const int local = rx.mfsk.preamble_nSymb;
		const int alternate = tx.mfsk.preamble_nSymb;
		check(alternate != local && rx.ofdm.mfsk_corr_template != nullptr,
			"alternate preamble differs from local active template");
		const int msym = tx.data_container.Nofdm * tx.frequency_interpolation_rate;
		const int frame_samples = (tx.data_container.Nsymb + alternate) * msym;
		const int rx_samples = rx.data_container.buffer_Nsymb * msym;
		std::vector<double> frame((size_t)frame_samples, 0.0), window((size_t)rx_samples, 0.0);
		const int bytes = (tx.data_container.nBits - tx.ldpc.P - tx.outer_code_reserved_bits) / 8;
		std::vector<int> payload((size_t)std::max(bytes, 1), 0);
		for(int i = 0; i < bytes; ++i) payload[(size_t)i] = (i * 37 + 11) & 255;
		tx.transmit_byte(payload.data(), bytes, frame.data(), SINGLE_MESSAGE);
		const int onset = rx_samples - (alternate + 2) * msym;
		check(onset > 0, "alternate preamble fits at incomplete-frame onset");
		if(onset > 0)
		{
			std::copy(frame.begin(), frame.begin() + std::min(frame_samples, rx_samples - onset),
				window.begin() + onset);
			rx.mfsk_fixed_delay = -1;
			auto r = rx.receive_byte(window.data(), out.data());
			printf("[TEST-RX-CONTRACTS] ROUTE matched local=%d alternate=%d actual=%d delay=%d overflow=%d\n",
				local, alternate, rx.ofdm.mfsk_matched_preamble_nsymb, (int)r.delay, r.frame_overflow_symbols);
			check(rx.ofdm.mfsk_matched_preamble_nsymb == alternate && r.delay >= 0
				&& r.message_decoded != YES, "real correlator selects alternate on an incomplete frame");
			check(rx.receive_stats.last_eff_preamble_nsymb == alternate,
				"matched MFSK rejection publishes alternate searched geometry");
			rx.load_configuration(CONFIG_0);
			check(reset(rx), "config change clears matched MFSK reject state");
			next_full(rx);
		}
	}
	printf("[TEST-RX-CONTRACTS] %s (%d failures)\n", failed ? "FAILURES" : "ALL PASS", failed);
	fflush(stdout);
	return failed ? 1 : 0;
}

// Ticket 95: real acquisition followed by explicit post-demodulation state injection.
// Each row owns its counters and payload; no receiver-state reset between calls.
int cl_telecom_system::test_agc_receive_entry()
{
	int failed = 0;
	auto check = [&](bool ok, const char* name) {
		printf("[TEST-AGC-ENTRY] %s: %s\n", ok ? "PASS" : "FAIL", name);
		if(!ok) ++failed;
	};
	cl_telecom_system tx, rx;
	tx.operation_mode = rx.operation_mode = BER_PLOT_passband;
	tx.narrowband_enabled = rx.narrowband_enabled = YES;
	const bool initialized = tx.load_configuration(CONFIG_0) == 0
		&& rx.load_configuration(CONFIG_0) == 0;
	check(initialized, "fresh NB CONFIG_0 initializes both real instances");
	if(!initialized) return 1;
	const int sym = rx.data_container.Nofdm * rx.frequency_interpolation_rate;
	const int frame_size = (tx.data_container.Nsymb + tx.data_container.preamble_nSymb) * sym;
	const int window_size = rx.data_container.buffer_Nsymb * sym;
	const int bytes = (tx.data_container.nBits - tx.ldpc.P - tx.outer_code_reserved_bits) / 8;
	const int onset = sym;
	const bool geometry = rx.M != MOD_MFSK && rx.ofdm.channel_estimator == ZERO_FORCE
		&& rx.ofdm.Nsymb == rx.data_container.Nsymb && rx.ofdm.Nc == rx.data_container.Nc
		&& rx.ofdm.Nsymb > 0 && rx.ofdm.Nc > 0 && rx.ofdm.pilot_configurator.nPilots > 0
		&& bytes > 0 && bytes <= N_MAX && frame_size > 0 && onset > 0
		&& frame_size + onset <= window_size && rx.ofdm_forced_delay < 0 && rx.mfsk_fixed_delay < 0
		&& !rx.preamble_amortization_enabled && !tx.preamble_amortization_enabled;
	check(geometry, "initialized OFDM/ZF/pilots/full-anchor geometry and unforced acquisition");
	if(!geometry) return 1;
	std::vector<int> payload((size_t)bytes), output((size_t)N_MAX, -777);
	std::vector<double> frame((size_t)frame_size), window((size_t)window_size);
	// Ordinary control, observed control, isolated invalid, recovery, repeated
	// invalids, recovery, ordinary control. Every call uses the same RX instance.
	for(int row_id = 0; row_id < 8; ++row_id) {
		const bool invalid = row_id == 2 || row_id == 4 || row_id == 5;
		const bool observed = row_id != 0 && row_id != 7;
		for(int i = 0; i < bytes; ++i) payload[(size_t)i] = (11 + row_id * 53 + i * 37) & 255;
		std::fill(frame.begin(), frame.end(), 0.0);
		std::fill(window.begin(), window.end(), 0.0);
		std::fill(output.begin(), output.end(), -777);
		tx.transmit_byte(payload.data(), bytes, frame.data(), SINGLE_MESSAGE);
		std::copy(frame.begin(), frame.end(), window.begin() + onset);
		const std::vector<double> original(window);
		agc_receive_observation obs;
		obs.inject_zero = invalid;
		st_receive_stats result;
		if(observed) {
			agc_receive_scope scope(rx, obs);
			result = rx.receive_byte(window.data(), output.data());
		} else result = rx.receive_byte(window.data(), output.data());
		const bool keyed = std::equal(payload.begin(), payload.end(), output.begin());
		const bool untouched = std::all_of(output.begin(), output.end(), [](int v) { return v == -777; });
		printf("[AGC-ENTRY-v1] row=%d sequence=%d inject=%d observed=%d calls=%u agc=%u zf=%u equalizer=%u "
			"nsymb=%d nc=%d modulation=%d estimator=%d cells=%llu,%llu,%llu nonfinite=%llu,%llu,%llu "
			"nonzero=%llu,%llu,%llu subpeak=%u skip_h=%u writes=%u accepted=%u decoded=%d full=%d "
			"delay=%d cursor=%d overflow=%d forced=%d,%d bytes=%d keyed=%d untouched=%d expected=",
			row_id, row_id, invalid, observed, obs.calls, obs.agc, obs.zf, obs.equalizer,
			obs.nsymb, obs.nc, obs.modulation, obs.estimator,
			(unsigned long long)obs.cells[0], (unsigned long long)obs.cells[1], (unsigned long long)obs.cells[2],
			(unsigned long long)obs.nonfinite[0], (unsigned long long)obs.nonfinite[1], (unsigned long long)obs.nonfinite[2],
			(unsigned long long)obs.nonzero[0], (unsigned long long)obs.nonzero[1], (unsigned long long)obs.nonzero[2],
			obs.subpeak_reject, obs.skip_h_reject, obs.output_writes, obs.accepted, result.message_decoded,
			rx.receive_stats.last_eff_preamble_nsymb, (int)result.delay, rx.receive_stats.ofdm_search_raw,
			result.frame_overflow_symbols, rx.ofdm_forced_delay, rx.mfsk_fixed_delay, bytes, keyed, untouched);
		for(int value : payload) printf("%02x", value);
		printf(" actual="); for(int i = 0; i < bytes; ++i) printf("%03x", output[(size_t)i] & 0xfff);
		printf("\n");
		check(rx.agc_receive_test_ == nullptr, "call scope disarms after receive");
		check(window == original, "receive preserves caller snapshot bytes");
		check(rx.ofdm_forced_delay < 0 && rx.mfsk_fixed_delay < 0
			&& rx.receive_stats.last_eff_preamble_nsymb == rx.data_container.preamble_nSymb,
			"call retains unforced full-anchor acquisition");
		if(observed) {
			const uint64_t cells = (uint64_t)rx.ofdm.Nsymb * rx.ofdm.Nc;
			check(obs.calls == 1 && obs.agc > 0 && obs.zf > 0
				&& obs.cells[0] == cells * obs.agc && obs.cells[1] == cells * obs.zf,
				"row independently reaches actual AGC and initialized ZF");
			check(obs.nonfinite[0] == 0 && obs.nonfinite[1] == 0 && obs.nonfinite[2] == 0,
				"observed AGC, ZF and reached equalizer outputs are finite");
		}
		if(invalid) {
			check(obs.nonzero[0] == 0 && obs.nonzero[1] == 0,
				"whole initialized matrix stays exact zero through guarded AGC and ZF");
			check(result.message_decoded != YES && obs.accepted == 0 && obs.output_writes == 0
				&& untouched && obs.subpeak_reject + obs.skip_h_reject > 0,
				"invalid row follows real rejection with no decoded or output publication");
		} else {
			check(result.message_decoded == YES && keyed, "valid or recovery payload is byte-exact for its row key");
			if(observed) check(obs.equalizer > 0 && obs.accepted == 1
				&& obs.cells[2] == (uint64_t)rx.ofdm.Nsymb * rx.ofdm.Nc * obs.equalizer,
				"valid or recovery row independently reaches actual equalizer and acceptance");
		}
	}
	printf("[TEST-AGC-ENTRY] %s (%d failures; rows=8 invalid=3 observed_valid=3 disabled_valid=2)\n",
		failed ? "FAILURES" : "ALL PASS", failed);
	fflush(stdout);
	return failed ? 1 : 0;
}
