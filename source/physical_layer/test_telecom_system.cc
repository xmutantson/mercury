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
