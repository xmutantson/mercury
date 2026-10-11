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

// ============================================================================
// TIMING-ACQUISITION-UNDER-SFO HARNESS  (F1 fix — the durable infra investment)
// ============================================================================
// PURPOSE. The two existing decode paths CANNOT exhibit timing-acquisition
// failures, because both PIN the FFT window:
//   • the standard BER sweep (passband_test_EsN0) sets ofdm_forced_delay to a
//     KNOWN nominal position and the RX then bypasses Schmidl-Cox AND Moose
//     (telecom_system.cc:504 sets it; :1010/:2137/:2462 bypass on it);
//   • the in-process 2-instance pump hands the decoder a frame-aligned window
//     at a deterministic ring offset (arq_commander.cc:~9822). Injecting
//     cl_sim_sfo there leaves ARM A == ARM B byte-identical to SFO-off, because
//     the pinned window cannot move (fact-doc §9/F1).
// So SFO has ZERO effect in either path. This harness instead makes the RX
// ACQUIRE timing from a LONG, SFO-DRIFTED, CONTINUOUS block (a real OFDM batch
// is back-to-back frames on ONE continuous TX clock), so SFO matters:
//   1. TX N back-to-back CFG OFDM frames into one contiguous passband buffer
//      (the SAME transmit_byte the production TX uses; per-frame preamble length
//      governed by LEVER P's tx override / MERCURY_SIM2_MINI_NSYM).
//   2. Pass the WHOLE buffer through ONE cl_sim_sfo instance — a stateful,
//      phase-continuous fractional resampler (sim_channel.h). The integer part
//      of the accumulated SFO phase skips/repeats whole input samples, so the
//      start-of-frame CREEPS across the block; the fractional part sub-sample-
//      misaligns the Schmidl-Cox half-symbol repeat. This is the impairment the
//      pinned paths structurally cannot show.
//   3. For each frame, present a buffer_Nsymb window to the REAL receive_byte
//      with ofdm_forced_delay = -1 (so it runs Schmidl-Cox time sync + Moose
//      carrier sync + channel est + equalizer + LDPC), and compare the decoded
//      bits against the known TX bits. Record the detected (drifting) preamble
//      position + coarse metric per frame to expose the timing creep.
//
// Two arms (env-selected, NOT hard-coded here):
//   • FULL : preamble_amortization_enabled = false  ⇒ every frame 4-sym preamble
//            (long Schmidl-Cox integration L; should HOLD under SFO).
//   • MINI : preamble_amortization_enabled = true, MERCURY_SIM2_MINI_NSYM = 1
//            ⇒ tail frames carry a 1-sym preamble (L 4× shorter ⇒ metric
//            variance 4× ⇒ peak-pick jitters under SFO; should FAIL on the tail).
//
// Knobs (all default to a sensible value; NONE change the production BER sweep —
// that path is gated out above by MERCURY_SFO_BLOCK_TEST):
//   MERCURY_SFO_BLOCK_NFRAMES : frames in the block (default 60 — the make-or-
//                               break TEST 1 length for big-block framing).
//   MERCURY_SFO_BLOCK_ESN0    : channel Es/N0 in dB (default 900 = clean cell;
//                               isolates the SFO timing effect from AWGN).
//   MERCURY_SFO_BLOCK_SEED    : TX-data + AWGN RNG seed (default 12345).
//   MERCURY_SIM2_SFO_PPM / _SFO_WALK_PPM / _SFO_MAX_PPM / _SFO_SEED : the SFO
//                               itself (cl_sim_sfo knobs; default OFF ⇒ a clean
//                               control run that MUST decode the whole block).
//   MERCURY_SIM2_MINI_NSYM    : MINI-arm per-frame preamble length (LEVER P).
//
// Determinism: TX data + AWGN seeded from MERCURY_SFO_BLOCK_SEED; the SFO stage
// has its OWN cl_sim_xoshiro (sfo_seed). Same env ⇒ bit-reproducible.
void cl_telecom_system::sfo_block_test()
{
	if(M == MOD_MFSK)
	{
		std::cout << "[SFO-BLOCK] MFSK not supported (OFDM timing-acquisition harness); use -s 15/16." << std::endl;
		return;
	}

	auto env_i = [](const char* k, int def) { const char* e = std::getenv(k); return (e && *e) ? atoi(e) : def; };
	auto env_f = [](const char* k, double def) { const char* e = std::getenv(k); return (e && *e) ? atof(e) : def; };

	int    nFrames = env_i("MERCURY_SFO_BLOCK_NFRAMES", 60);
	if(nFrames < 1) nFrames = 1;
	double esn0_db = env_f("MERCURY_SFO_BLOCK_ESN0", 900.0);
	uint64_t seed  = (uint64_t)env_i("MERCURY_SFO_BLOCK_SEED", 12345);

	// LEVER P arm selection: MINI engages preamble amortization (the per-frame
	// MINI preamble length is read from MERCURY_SIM2_MINI_NSYM inside
	// preamble_sched_nsymb). FULL forces every frame to the configured 4-sym
	// preamble. The arm is chosen from preamble_amortization_enabled, which the
	// in-process pump path sets via the CLI (arq_commander.cc:10150). Standalone
	// (this PLOT_PASSBAND harness has no ARQ pump to set it), select the arm with
	// MERCURY_SFO_BLOCK_ARM=MINI|FULL (default = the member flag = FULL).
	{
		const char* arm = std::getenv("MERCURY_SFO_BLOCK_ARM");
		if(arm && (*arm=='M' || *arm=='m')) preamble_amortization_enabled = true;
		else if(arm && (*arm=='F' || *arm=='f')) preamble_amortization_enabled = false;
	}
	bool mini_arm = preamble_amortization_enabled;
	int  full_pre = data_container.preamble_nSymb;

	// Geometry (one frame, FULL preamble = the allocation maximum).
	int interp        = frequency_interpolation_rate;
	int sym_samples   = data_container.Nofdm * interp;            // one OFDM symbol, passband
	int data_nsymb    = data_container.Nsymb;                     // data symbols / frame
	int full_frame    = (data_nsymb + full_pre) * sym_samples;    // FULL frame, passband samples
	int buf_interp    = data_container.Nofdm * data_container.buffer_Nsymb * interp; // RX window

	int nReal_data    = data_container.nBits - ldpc.P;
	int payload_bytes = (nReal_data - outer_code_reserved_bits) / 8;
	int payload_bits  = payload_bytes * 8;   // byte-aligned: matches transmit_byte(payload_bytes)

	std::cout << "[SFO-BLOCK] cfg=" << current_configuration
	          << " M=" << M << " Nsymb=" << data_nsymb
	          << " full_pre=" << full_pre
	          << " arm=" << (mini_arm ? "MINI" : "FULL")
	          << " mini_nsym=" << preamble_sched_nsymb(1, false, full_pre)
	          << " nFrames=" << nFrames
	          << " EsN0=" << esn0_db
	          << " SFO_ppm=" << env_f("MERCURY_SIM2_SFO_PPM", 0.0)
	          << " SFO_walk_ppm=" << env_f("MERCURY_SIM2_SFO_WALK_PPM", 0.0)
	          << " seed=" << seed << std::endl;

	// --- TX: build N back-to-back frames into one contiguous passband buffer. ---
	// Capacity: N FULL frames + headroom; emitted MINI frames are shorter so this
	// is a safe upper bound regardless of arm.
	std::vector<double> tx_block((size_t)nFrames * (size_t)full_frame + (size_t)sym_samples, 0.0);
	// Per-frame: TX bit pattern (for BER) + the emitted-frame sample offset/length.
	std::vector<std::vector<int>> frame_bits(nFrames);
	std::vector<long>             frame_off(nFrames, 0);
	std::vector<int>              frame_len(nFrames, 0);
	std::vector<int>              frame_pre(nFrames, 0);   // preamble symbols this frame emitted

	output_power_Watt = 1;
	ts_srandom((unsigned int)seed);

	long write_pos = 0;
	for(int f = 0; f < nFrames; f++)
	{
		// Per-frame known random payload (reproducible from the block seed).
		frame_bits[f].resize(payload_bits);
		for(int i = 0; i < payload_bits; i++)
			frame_bits[f][i] = (int)(ts_random() % 2);
		bit_to_byte(frame_bits[f].data(), data_container.data_byte, payload_bits);

		// LEVER P: drive the per-frame TX preamble length exactly like the ARQ
		// batch assembler does (arq_common.cc:3918). Frame 0 = anchor (FULL);
		// tail frames = MINI when the MINI arm is on. preamble_sched_nsymb is the
		// SAME pure schedule the production TX/RX both derive, so this faithfully
		// reproduces LEVER P's emitted-frame geometry.
		int emit_pre = full_pre;
		if(mini_arm)
		{
			tx_preamble_nsymb_override = preamble_sched_nsymb(f, /*force_full=*/false, full_pre);
			emit_pre = tx_preamble_nsymb_override;
		}
		else
		{
			tx_preamble_nsymb_override = -1; // FULL every frame
		}

		// Emit one frame into passband_data; transmit_byte populates it and sets
		// tx_last_emitted_frame_samples to the ACTUAL emitted length (preamble+data).
		// Clear the status first so an early-returning TX cannot inherit a plausible
		// length from this harness and silently continue with stale waveform data.
		tx_last_emitted_frame_samples = 0;
		this->transmit_byte(data_container.data_byte, payload_bytes,
		                    data_container.passband_data, SINGLE_MESSAGE);
		// Test-only fault seam for the --test death test. Unset by default and scoped
		// to this already opt-in SFO harness; it models a failed/overrunning producer.
		const char* forced_emitted = std::getenv("MERCURY_TEST_SFO_BLOCK_EMITTED_SAMPLES");
		if(forced_emitted && *forced_emitted)
			tx_last_emitted_frame_samples = atoi(forced_emitted);
		int emitted = tx_last_emitted_frame_samples;
		if(emitted <= 0 || emitted > full_frame)
		{
			std::cerr << "[SFO-BLOCK] transmit_byte reported invalid frame length "
			          << emitted << " (capacity " << full_frame << ", frame " << f
			          << "); aborting" << std::endl;
			tx_preamble_nsymb_override = -1;
			std::abort();
		}

		frame_off[f] = write_pos;
		frame_len[f] = emitted;
		frame_pre[f] = emit_pre;
		for(int i = 0; i < emitted; i++)
			tx_block[(size_t)write_pos + i] = data_container.passband_data[i];
		write_pos += emitted;
	}
	tx_preamble_nsymb_override = -1; // defensive reset
	long block_len = write_pos;

	// --- CHANNEL: drift the WHOLE contiguous block through one cl_sim_sfo. ---
	// One stateful instance ⇒ the fractional accumulator + integer SOF-creep carry
	// across the entire block (frame boundaries are NOT realigned — exactly the HW
	// continuous-clock drift). cl_sim_sfo is n-in/n-out exact; the read pointer's
	// accumulated drift across block_len samples is the cumulative timing error the
	// per-frame acquisition must track. SFO knobs come from the env (default OFF).
	{
		cl_sim_sfo sfo(/*seed=*/ (seed ^ 0x2545F4914F6CDD1DULL) + 0xC2B2AE3D27D4EB4FULL,
		               env_f("MERCURY_SIM2_SFO_PPM", 0.0),
		               env_f("MERCURY_SIM2_SFO_WALK_PPM", 0.0),
		               env_f("MERCURY_SIM2_SFO_MAX_PPM", 90.0),
		               48000.0);
		// Process in one shot over the whole block (phase-continuous; equivalent to
		// streaming since the resampler carries state, but one call is simplest and
		// has no inter-block edge since there are no edges).
		sfo.process(tx_block.data(), (size_t)block_len);
		std::cout << "[SFO-BLOCK] SFO stage " << (sfo.enabled() ? "ON" : "OFF (control run)")
		          << " block_samples=" << block_len << std::endl;
	}

	// AWGN sigma (Es/N0), measured from the actual block power (same calibration as
	// passband_test_EsN0). Clean cell (esn0>=900) ⇒ no additive noise.
	double sigma = 0.0;
	bool clean = (esn0_db >= 900.0);
	if(!clean)
		sigma = 1.0 / sqrt(pow(10.0, esn0_db / 10.0));

	// --- RX: per-frame REAL acquisition over a buffer_Nsymb window. ---
	// Window layout: [lead margin of zeros | frame's drifted samples | trailing
	// zeros], placed in passband_delayed_data. The lead margin (a couple symbols)
	// gives the Schmidl-Cox search room and lets the SFO-creep move the preamble
	// AWAY from a fixed sample so timing acquisition is genuinely exercised. We map
	// frame f to its TX offset; the SFO has already drifted the content so the
	// actual preamble sits at frame_off[f] +/- accumulated creep, which the RX must
	// re-find. ofdm_forced_delay = -1 ⇒ full Schmidl-Cox + Moose.
	// Lead margin: place the preamble at symbol (full_pre+2) so it clears the RX
	// coarse-bounds gate (lower_bound = preamble_nSymb; the gate requires
	// pream_symb_loc > preamble_nSymb — telecom_system.cc:1701,1730). This mirrors
	// the standard BER path's forced-delay convention
	// ((preamble_nSymb+2)*Nofdm+delay). The SFO-creep then moves the actual
	// preamble away from this nominal sample, which the RX must re-acquire.
	int lead = (full_pre + 2) * sym_samples;            // preamble lands at symbol full_pre+2
	int win_frame_max = full_frame + 4 * sym_samples;   // FULL frame + slack
	if(lead + win_frame_max > buf_interp)
	{
		// Should not happen for WB CFG15/16 (buffer_Nsymb >> frame), but clamp.
		lead = 0;
		if(win_frame_max > buf_interp) win_frame_max = buf_interp;
	}

	int    decoded_ok = 0;
	long   total_bit_errors = 0;
	long   total_bits = 0;
	int    frames_zero_ber = 0;
	double metric_sum_full = 0.0, metric_sum_mini = 0.0;
	int    n_full = 0, n_mini = 0;

	// AWGN seeded once (separate from TX data stream) for reproducibility.
	awgn_channel.set_seed((long)(seed | 1));

	// ---- BIG-BLOCK ONE-ACQUISITION MODE (MERCURY_SFO_BIGBLOCK) ----------------
	// The task's "big block" = ONE timing acquisition on the HEAD preamble, then
	// decode the whole contiguous K-frame run WITHOUT per-frame re-acquisition (the
	// anti-LEVER-P model: VARA-style single-acquire-and-hold). The per-frame loop
	// above RE-ACQUIRES every frame (Schmidl-Cox each time) and so structurally
	// cannot show a tail timing failure — the very limitation §11 flagged.
	//
	// ARCHITECTURE NOTE (code-verified, fact-doc §12): a SINGLE LDPC codeword is
	// fixed at N=MERCURY_NORMAL=1600 bits (ldpc.cc:66, physical_defines.h:31), and
	// CFG16's Nsymb=9 is DERIVED so nData*log2(M)==1600 (telecom_system.cc:4598).
	// A literal "one preamble + 60 DATA symbols carrying one codeword" is therefore
	// IMPOSSIBLE without a new ~10k-bit LDPC code. The faithful realization of a
	// 60-data-symbol block is K=ceil(60/Nsymb) back-to-back codeword-frames under
	// ONE head acquisition: each frame is a valid 1600-bit codeword (real LDPC
	// block-decode metric), the SFO drifts the WHOLE run on one clock, and the RX
	// anchors all frames to the head-acquired alignment + NOMINAL frame stride
	// (no Schmidl-Cox after frame 0). The SFO creep then walks the nominal window
	// off the true (drifted) content across the block — the tail must HOLD (the
	// per-frame pilot estimate + equalizer absorb it) or FAIL (the GO/NO-GO).
	bool bigblock = (env_i("MERCURY_SFO_BIGBLOCK", 0) != 0);
	// NEGATIVE CONTROL (faithfulness): MERCURY_SFO_BIGBLOCK_NOTRACK forces the RX
	// to anchor every tail frame to the head alignment WITHOUT advancing by the
	// per-frame SFO-creep that a real tracker/re-acquisition would apply. In the
	// default bigblock mode the nominal stride is exact for SFO=0 and only drifts
	// under SFO — i.e. the per-frame pilot estimate + equalizer are the ONLY thing
	// holding the tail. To PROVE the harness is not too lenient, NOTRACK injects a
	// fixed extra misalignment per frame (a few samples) that NO per-frame pilot
	// estimate can absorb, so the tail MUST fail. If even the default bigblock tail
	// holds at 50ppm but NOTRACK fails, the "holds" result is real discrimination.
	bool notrack_ctrl = (env_i("MERCURY_SFO_BIGBLOCK_NOTRACK", 0) != 0);
	// Per-frame extra misalignment (interp samples) injected in the NOTRACK control.
	int  notrack_skew = env_i("MERCURY_SFO_BIGBLOCK_NOTRACK_SKEW", 8);

	long   head_delay_interp = -1;   // acquired ONCE on frame 0 (bigblock mode)

	for(int f = 0; f < nFrames; f++)
	{
		st_receive_stats st;
		double metric;
		long   det;
		rx_preamble_nsymb_override = frame_pre[f];

		if(bigblock)
		{
			// HELD-PATH RING-RELATIVE RE-SLIDE (harness-hygiene fix).
			// The RX decode window (passband_delayed_data) is hard-capped at
			// `buf_interp` by the production frame-completeness gate
			// (telecom_system.cc:1745) and bounds gate (:1766); for WB CFG15/16
			// `buf_interp` only spans ~6-9 frames. A 60-frame test block does NOT
			// fit, so we CANNOT lay the whole block down once and march
			// ofdm_forced_delay forward (the old code did exactly that — frame 0
			// copied only `buf_interp-lead` samples and f>0 read ZEROS past frame
			// ~9, manufacturing an SFO-INDEPENDENT decode cliff that was a buffer
			// artifact, not a timing wall).
			//
			// Fix: keep ONE acquisition on the head (frame 0) but RE-SLIDE the
			// STORAGE every frame — copy the CURRENT frame's drifted samples into
			// the SAME head-anchored window each iteration — WITHOUT re-acquiring
			// timing. The extraction delay stays PINNED at the head lock
			// (head_delay_interp); no Schmidl-Cox after frame 0. Faithfulness: the
			// whole block was SFO-drifted in place above, so frame f's true preamble
			// sits at frame_off[f]+creep(f) in tx_block; copying the window from the
			// NOMINAL offset frame_off[f] into the buffer leaves the preamble
			// creep(f) samples off the pinned head delay. That growing in-window
			// misalignment is exactly the held-path drift the per-frame pilot
			// estimate + equalizer must absorb (HOLD) or fail (NO-GO). At SFO=0 the
			// nominal offset is exact, so every frame is real signal at the pinned
			// delay -> the cliff is gone (~all decode), proving it was the buffer.
			if(f == 0)
			{
				for(int i = 0; i < buf_interp; i++) data_container.passband_delayed_data[i] = 0.0;
				// Frame 0 window: nominal offset frame_off[0]==0; copy a frame +
				// slack (bounded by the block and the buffer), same span the per-
				// frame path below uses.
				int copy_n = win_frame_max;
				if((long)copy_n > block_len) copy_n = (int)block_len;
				for(int i = 0; i < copy_n && (lead + i) < buf_interp; i++)
				{
					double s = tx_block[(size_t)i];
					if(!clean && sigma > 0.0)
						s += (double)((sigma / sqrtf(2.0f)) * awgn_channel.awgn_value_generator());
					data_container.passband_delayed_data[lead + i] = s;
				}
				// ONE real acquisition on the head preamble (Schmidl-Cox + Moose).
				ofdm_forced_delay = -1;
				mfsk_fixed_delay  = -1;
				st = this->receive_byte(data_container.passband_delayed_data,
				                        data_container.hd_decoded_data_byte);
				head_delay_interp = st.delay;   // acquired head position (interp samples)
			}
			else
			{
				// RE-SLIDE the storage so this frame's window is always in-buffer,
				// then decode at the PINNED head lock (NO re-acquisition). The copy
				// destination is the head-anchored `lead`; the source is the NOMINAL
				// TX offset frame_off[f] (== f*full_frame for the FULL arm). Any
				// SFO creep already in tx_block displaces the preamble within the
				// window relative to the pinned head delay -> the held-path drift.
				for(int i = 0; i < buf_interp; i++) data_container.passband_delayed_data[i] = 0.0;
				long  src = frame_off[f];
				// NEGATIVE CONTROL: inject a cumulative per-frame misalignment that
				// no per-frame pilot estimate can absorb (proves the harness bites).
				// In the re-slide model the storage moves with the copy, so we skew
				// the copy DESTINATION (push the frame's content `f*skew` samples off
				// the pinned head delay) — equivalent to the old extraction-delay
				// skew, but now it survives because the data is actually present.
				int   dest = lead;
				if(notrack_ctrl) dest += (int)((long)f * (long)notrack_skew);
				int copy_n = win_frame_max;
				if(src + copy_n > block_len) copy_n = (int)(block_len - src);
				for(int i = 0; i < copy_n && (dest + i) < buf_interp && (dest + i) >= 0; i++)
				{
					double s = tx_block[(size_t)src + i];
					if(!clean && sigma > 0.0)
						s += (double)((sigma / sqrtf(2.0f)) * awgn_channel.awgn_value_generator());
					data_container.passband_delayed_data[dest + i] = s;
				}
				// PINNED head lock — held timing, no Schmidl-Cox re-acquire.
				ofdm_forced_delay = head_delay_interp;
				mfsk_fixed_delay  = -1;
				st = this->receive_byte(data_container.passband_delayed_data,
				                        data_container.hd_decoded_data_byte);
			}
			ofdm_forced_delay = -1;
			metric = st.coarse_metric;
			det    = st.delay;
			if(frame_pre[f] >= full_pre) { metric_sum_full += metric; n_full++; }
			else                         { metric_sum_mini += metric; n_mini++; }
			goto bigblock_score;
		}

		// Zero the RX window, then copy this frame's drifted samples after `lead`.
		for(int i = 0; i < buf_interp; i++) data_container.passband_delayed_data[i] = 0.0;

		{
		long src = frame_off[f];
		// Pull the frame plus a little of the NEXT frame's lead-in (so the data
		// tail + any SFO over-read is present); bounded by the block end.
		int copy_n = win_frame_max;
		if(src + copy_n > block_len) copy_n = (int)(block_len - src);
		for(int i = 0; i < copy_n && (lead + i) < buf_interp; i++)
		{
			double s = tx_block[(size_t)src + i];
			if(!clean && sigma > 0.0)
				s += (double)((sigma / sqrtf(2.0f)) * awgn_channel.awgn_value_generator());
			data_container.passband_delayed_data[lead + i] = s;
		}
		}

		// REAL acquisition: -1 ⇒ Schmidl-Cox time sync + Moose carrier sync run.
		ofdm_forced_delay = -1;
		mfsk_fixed_delay  = -1;
		// The RX MINI extraction (rx_eff_preamble=1) is gated behind the ARQ
		// batch-predict state machine, which this standalone harness does not run;
		// here every frame is acquired by the FULL-buffer Schmidl-Cox search, whose
		// integration length is set by the preamble actually present in the signal.
		// That is the physically correct discriminator: a frame TX'd with a 1-sym
		// preamble (MINI arm) presents a 4x-shorter half-symbol repeat to the SAME
		// search, so its metric under-integrates under SFO regardless of ARQ state.
		st = this->receive_byte(data_container.passband_delayed_data,
		                        data_container.hd_decoded_data_byte);
		ofdm_forced_delay = -1;
		metric = st.coarse_metric;
		det    = st.delay;
		if(frame_pre[f] >= full_pre) { metric_sum_full += metric; n_full++; }
		else                         { metric_sum_mini += metric; n_mini++; }

	bigblock_score:;
		rx_preamble_nsymb_override = -1;

		// BER for this frame: decode bytes -> bits, compare to TX bits.
		byte_to_bit(data_container.hd_decoded_data_byte, data_container.hd_decoded_data_bit, payload_bytes);
		int errs = 0;
		for(int i = 0; i < payload_bits; i++)
			if(frame_bits[f][i] != data_container.hd_decoded_data_bit[i]) errs++;
		total_bit_errors += errs;
		total_bits       += payload_bits;
		if(errs == 0) frames_zero_ber++;
		if(st.message_decoded == YES) decoded_ok++;

		if(f < 8 || f >= nFrames - 4 || (f % 10) == 0)
		{
			std::cout << "[SFO-BLOCK] frame=" << f
			          << " pre=" << frame_pre[f]
			          << (bigblock ? " pinned_off=" : " expect_off=")
			          << (bigblock ? head_delay_interp : (long)lead)
			          << (bigblock ? "(reslide,no-reacq)" : "(+drift)")
			          << " det=" << det
			          << " metric=" << metric
			          << " decoded=" << (st.message_decoded == YES ? 1 : 0)
			          << " biterr=" << errs << "/" << payload_bits << std::endl;
		}
	}

	double block_ber = (total_bits > 0) ? (double)total_bit_errors / (double)total_bits : 1.0;
	std::cout << "[SFO-BLOCK] ===== RESULT (" << (mini_arm ? "MINI" : "FULL") << " arm) ====="  << std::endl;
	std::cout << "[SFO-BLOCK]   frames_decoded=" << decoded_ok << "/" << nFrames
	          << "  frames_zero_ber=" << frames_zero_ber << "/" << nFrames << std::endl;
	std::cout << "[SFO-BLOCK]   block_BER=" << block_ber
	          << "  total_biterr=" << total_bit_errors << "/" << total_bits << std::endl;
	if(n_full > 0) std::cout << "[SFO-BLOCK]   mean_metric_FULLpre=" << (metric_sum_full / n_full) << std::endl;
	if(n_mini > 0) std::cout << "[SFO-BLOCK]   mean_metric_MINIpre=" << (metric_sum_mini / n_mini) << std::endl;
	std::cout << "[SFO-BLOCK]   tail-frame decode is the make-or-break: a clean control"
	          << " run (SFO OFF) MUST be ~all-decoded; SFO ON must DEGRADE the MINI tail." << std::endl;
}

// ============================================================================
// GENUINE single-grid big-block timing test. The sfo_block_test above tiles K
// independent 9-symbol codeword-frames (per-frame channel estimate); this builds
// ONE Nsymb-symbol grid carried by ONE channel estimate interpolated across the
// whole block (interpolate_bilinear_matrix, 33% pilots), which is the literal
// "one preamble + N data symbols, one estimate" the design asks about.
//
// SFO model (frequency-domain, exact): a sample-timing slip of tau samples at
// OFDM symbol n rotates subcarrier k by exp(-j 2*pi*k*tau_n / Nfft) — a phase RAMP
// in k whose slope GROWS with n (tau_n = ppm*1e-6 * n * (Nfft+Ngi)). This is the
// omega+k*delta per-symbol ramp the design names (Speth/Fechtel/Meyr 1999). The
// pilot lattice samples this ramp every Dy symbols; LS_channel_estimator's bilinear
// interpolation MAY already track it (STEP 1). MERCURY_SFO_GRID_TRACK=1 adds the
// STEP-2 anti-P CPE/PEG corrector: per symbol, LS line-fit pilot phase-error vs k
// (intercept=CPE, slope=PEG), de-rotate every subcarrier BEFORE the equalizer.
// ============================================================================

void cl_telecom_system::sfo_grid_test()
{
	if(M == MOD_MFSK){ std::cout << "[SFO-GRID] MFSK unsupported; use -s 15/16." << std::endl; return; }

	auto env_i = [](const char* k, int def){ const char* e=std::getenv(k); return (e&&*e)?atoi(e):def; };
	auto env_f = [](const char* k, double def){ const char* e=std::getenv(k); return (e&&*e)?atof(e):def; };

	int    Ngrid   = env_i("MERCURY_SFO_GRID_NSYMB", 60);   // data symbols in the ONE grid
	double ppm     = env_f("MERCURY_SIM2_SFO_PPM", 0.0);
	bool   track   = (env_i("MERCURY_SFO_GRID_TRACK", 0) != 0);     // STEP 2: CPE/PEG corrector
	bool   no_interp = (env_i("MERCURY_SFO_GRID_NOINTERP", 0) != 0);// negative control
	// HELD 2026-06-17: cannot be a simple default-flip — sim-harness-scoped, CHANNEL-GATED
	// (REGRESSES ~2 dB on clean), and the production FADE-tier estimator-selector wiring is
	// the unlanded follow-on (HW confirm pending). Keep default-off.
	// TURBO_EQ_VERDICT.md §5 TINTERP-SEED: MERCURY_SFO_GRID_TURBO_SEED=tinterp drives
	// the WHOLE recommended stack — it=0 INIT estimate = TINTERP (the warm faded seed)
	// AND data_aided_channel_estimator keeps that TINTERP H as the low-confidence FLOOR
	// (ofdm.dd_seed_floor) instead of a cold pilots-only interpolation. Default unset =
	// byte-identical (plain LS it=0 + pilots-only floor).
	bool turbo_seed_tinterp = false;
	{ const char* e = std::getenv("MERCURY_SFO_GRID_TURBO_SEED");
	  if(e && (std::string(e)=="tinterp" || std::string(e)=="TINTERP")) turbo_seed_tinterp = true; }
	int    win      = env_i("MERCURY_SFO_GRID_TRACK_WIN", 9);       // CPE/PEG sliding window
	uint64_t seed   = (uint64_t)env_i("MERCURY_SFO_GRID_SEED", 12345);

	// ---- PCS / 64-QAM override (lever #2, fact-documents/data-flow-pas-shaping.md) ----
	// MERCURY_SFO_GRID_M64=1 runs the harness at MOD_64QAM (the PAS-able 8x8 square,
	// psk.cc:159-225) instead of the config's M — additive sim plumbing for the
	// clean-front 64-QAM gear (DEEPDSP_PROGRAM.md §2 L-C "M64 override"). The config
	// table is NOT changed; M + psk are saved here and restored at teardown.
	// MERCURY_SFO_GRID_PCS=1 then turns on Probabilistic Amplitude Shaping: the CODED
	// TX fill draws shaped amplitude bits from the distribution matcher and the RX
	// demaps with the amplitude-bit log-prior (LLR de-shaping). Both default OFF =>
	// the whole harness is byte-identical to the base when unset.
	bool pcs_m64 = (env_i("MERCURY_SFO_GRID_M64", 0) != 0);
	bool pcs     = (env_i("MERCURY_SFO_GRID_PCS", 0) != 0);
	if(pcs) pcs_m64 = true;   // PAS requires the square 64-QAM rails
	double pcs_lambda = env_f("MERCURY_SFO_GRID_PCS_LAMBDA", 0.04);  // Maxwell-Boltzmann steepness
	int    pcs_rail_L = env_i("MERCURY_SFO_GRID_PCS_RAIL_L", 32);    // DM rail blocklength (<=34, __int128-exact)
	int    saved_M_pcs = M;
	if(pcs_m64 && M != MOD_64QAM)
	{
		M = MOD_64QAM;
		psk.set_predefined_constellation(M);   // rebuild the 64-QAM constellation + demod buffers
	}

	// --- Rebuild the OFDM grid at Nsymb=Ngrid (32QAM, 33% pilots Dx=1/Dy=3). ---
	int saved_Nsymb = ofdm.Nsymb;
	int Nc    = ofdm.Nc;
	int Nfft  = ofdm.Nfft;
	float gi  = ofdm.gi;
	int Ngi   = (int)round((double)gi * (double)Nfft);
	int Nofdm = Nfft + Ngi;
	// Preserve the preamble_configurator fields that ofdm.deinit() zeros but the
	// 4-arg ofdm.init() does NOT restore (Nsymb/modulation/nIdentical_sections/
	// boost). Without this, preamble_configurator::init divides nPreamble/Nsymb=0
	// (ofdm.cc:1304) -> crash. We only change the DATA grid (Nsymb), not the
	// preamble, so carry the preamble config across the rebuild verbatim.
	int   sav_pre_Nsymb = ofdm.preamble_configurator.Nsymb;
	int   sav_pre_mod   = ofdm.preamble_configurator.modulation;
	int   sav_pre_nIS   = ofdm.preamble_configurator.nIdentical_sections;
	double sav_pre_boost= ofdm.preamble_configurator.boost;
	int   sav_start_shift = ofdm.start_shift;
	// Pilot configurator: deinit() zeros boost (->1.33) which makes the pilot
	// sequence all-zero -> H=Y/0=NaN -> equalizer erases everything. Preserve the
	// pilot boost/modulation/seed too.
	double sav_pil_boost = ofdm.pilot_configurator.boost;
	int   sav_pil_mod    = ofdm.pilot_configurator.modulation;
	int   sav_pil_seed   = ofdm.pilot_configurator.seed;
	ofdm.deinit();
	ofdm.start_shift = sav_start_shift;
	ofdm.preamble_configurator.Nsymb = sav_pre_Nsymb;
	ofdm.preamble_configurator.modulation = sav_pre_mod;
	ofdm.preamble_configurator.nIdentical_sections = sav_pre_nIS;
	ofdm.preamble_configurator.boost = sav_pre_boost;
	ofdm.pilot_configurator.boost = sav_pil_boost;
	ofdm.pilot_configurator.modulation = sav_pil_mod;
	ofdm.pilot_configurator.seed = sav_pil_seed;
	ofdm.pilot_configurator.Dx = 1; ofdm.pilot_configurator.Dy = 3;   // 33% pilots
	ofdm.pilot_configurator.pilot_density = HIGH_DENSITY;
	ofdm.channel_estimator = LEAST_SQUARE;
	ofdm.channel_estimator_amplitude_restoration = NO;
	ofdm.LS_window_width = 0; ofdm.LS_window_hight = 0;   // re-derive in init
	ofdm.init(Nfft, Nc, Ngrid, gi);   // sets Nfft/Nc/Nsymb/gi/Ngi + allocs + configures lattice

	// ====================================================================
	// TEST 2: PILOT THINNING 33% -> ~6% (CONTINUAL + SCATTERED lattice).
	// ====================================================================
	// The net-PHY-recovery step. The default configure() lays Dy=3 FULL columns
	// (every column gets 1/3 of its rows = 33% pilots). To raise the data-carrying
	// fraction we OVERWRITE the lattice in-harness (NOT the production configure(),
	// which would re-derive nData->nBits and break CFG16's 1600-bit codeword sizing
	// across all 17 configs) with the design's CONTINUAL + SCATTERED layout:
	//
	//   CONTINUAL columns  : a few carriers carrying a pilot on EVERY symbol. These
	//                        anchor the CPE/PEG per-symbol LS phase fit (omega+k*delta
	//                        needs >=2 pilots PER symbol) AND feed the LS-path residual
	//                        nv estimator (every pilot counted, ofdm.cc:1816) so it
	//                        cannot collapse to the 1e-6 floor (the E1/cfg16-nvfix
	//                        sibling bug).
	//   SCATTERED lattice  : interior pilots every SCAT_DX carriers, repeating every
	//                        SCAT_DY symbols, offset by symbol -> the 2D samples the
	//                        channel estimate needs without spending a full column.
	//
	// Because LS_window=(Nc x Nsymb) (full grid), LS_channel_estimator gives EVERY
	// data cell the global LS fit over ALL pilots (ofdm.cc:1684-1745) and the
	// column/bilinear interpolation is a no-op (all cells already MEASURED) -> the
	// sparse layout needs NO interpolation rewrite. After the tracker removes the SFO
	// ramp the channel is flat-unity, so the global LS fit is near-exact.
	bool   thin      = (env_i("MERCURY_SFO_GRID_THIN", 0) != 0);
	int    cont_cols = env_i("MERCURY_SFO_GRID_CONT_COLS", 3);   // continual pilot columns
	int    scat_dx   = env_i("MERCURY_SFO_GRID_SCAT_DX", 12);    // scatter carrier step
	int    scat_dy   = env_i("MERCURY_SFO_GRID_SCAT_DY", 4);     // scatter symbol step
	if(thin)
	{
		// 1) Everything DATA.
		for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
			(ofdm.ofdm_frame+n*Nc+j)->type = DATA;

		// 2) CONTINUAL columns: cont_cols carriers spread across [0,Nc-1] incl. edges.
		//    carrier_c = round(c*(Nc-1)/(cont_cols-1)). Pilot on every symbol.
		std::vector<int> cont(cont_cols);
		for(int c=0;c<cont_cols;c++)
			cont[c] = (cont_cols<=1) ? 0
			        : (int)llround((double)c*(double)(Nc-1)/(double)(cont_cols-1));
		for(int c=0;c<cont_cols;c++)
			for(int n=0;n<Ngrid;n++)
				(ofdm.ofdm_frame+n*Nc+cont[c])->type = PILOT;

		// 3) SCATTERED lattice: at symbols n%scat_dy==0, place interior pilots every
		//    scat_dx carriers, offset by (n/scat_dy) so the diagonal walks across the
		//    band over successive scatter-rows (full 2D coverage for the channel est).
		if(scat_dx > 0 && scat_dy > 0)
		{
			for(int n=0;n<Ngrid;n++)
			{
				if(n % scat_dy != 0) continue;
				int off = (n/scat_dy) * (scat_dx/2 > 0 ? scat_dx/2 : 1);
				for(int j = off % scat_dx; j < Nc; j += scat_dx)
					(ofdm.ofdm_frame+n*Nc+j)->type = PILOT;
			}
		}

		// 4) Recount nPilots/nData. The framer/deframer/LS_channel_estimator and the
		//    CPE/PEG tracker ALL read ofdm.ofdm_frame[].type (NOT virtual_carrier, which
		//    is only consumed inside configure(), bypassed here). pilot_configurator.
		//    carrier IS ofdm_frame (same buffer, ofdm.cc:175), so the lattice we wrote
		//    above is already what every consumer sees. nData = Nc*Nsymb - nPilots,
		//    matching configure()'s convention.
		int np=0;
		for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
			if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT) np++;
		ofdm.pilot_configurator.nPilots = np;
		ofdm.pilot_configurator.nConfig = 0;
		ofdm.pilot_configurator.nData   = Ngrid*Nc - np;

		// 5) Re-allocate + re-seed the pilot DBPSK sequence to the NEW nPilots. The
		//    framer/estimator/tracker all index sequence[] by running pilot count in
		//    raster order, so it MUST have exactly nPilots entries.
		CDELETE(ofdm.pilot_configurator.sequence);
		ofdm.pilot_configurator.sequence =
		    CNEW(std::complex<double>, np, "pilot.sequence.thin");
		__srandom(ofdm.pilot_configurator.seed);
		int last_pilot=0;
		for(int i=0;i<np;i++)
		{
			int pv = (__random()%2) ^ last_pilot;   // DBPSK, same generator as init()
			ofdm.pilot_configurator.sequence[i] =
			    std::complex<double>(2*pv-1,0) * ofdm.pilot_configurator.boost;
			last_pilot = pv;
		}
	}

	int nData = ofdm.pilot_configurator.nData;     // data carriers across the 60-grid
	int log2M = (int)round(log2((double)M));
	int nBits = nData * log2M;

	std::cout << "[SFO-GRID] cfg=" << current_configuration << " M=" << M
	          << " Nsymb=" << Ngrid << " Nc=" << Nc
	          << (thin ? " THIN(cont+scatter)" : " Dx=1 Dy=3")
	          << (thin ? " cont_cols=" : " ") << (thin ? cont_cols : 0)
	          << (thin ? " scat_dx=" : "") << (thin ? scat_dx : 0)
	          << (thin ? " scat_dy=" : "") << (thin ? scat_dy : 0)
	          << " nData=" << nData << " nBits=" << nBits
	          << " pilots=" << ofdm.pilot_configurator.nPilots
	          << " (" << (100.0*ofdm.pilot_configurator.nPilots/(double)(Ngrid*Nc)) << "% of grid)"
	          << " ppm=" << ppm << " track=" << (track?1:0)
	          << " no_interp=" << (no_interp?1:0) << std::endl;

	// --- TX: data symbols + known pilots into the 60-grid. ---
	// Two modes:
	//  - UNCODED (default): nBits random bits (requirement (a), the timing/BER probe).
	//  - CODED (MERCURY_SFO_GRID_CODED=1): fill the FIRST K*N bits with K LDPC
	//    codewords (K = floor(nBits / ldpc.N)), each = [1400 info | 200 parity]
	//    (systematic, ldpc.cc:107-120), so the decode is the REAL rate-0.875 LDPC
	//    block-decode at the achieved pilot fraction. This is the "coded K-codeword
	//    path at 6% pilots + tracker, under SFO" the GO/NO-GO names. AWGN is added in
	//    the channel stage when MERCURY_SFO_GRID_ESN0 < 900 so noise_variance_estimate
	//    is a meaningful quantity to check for the E1/cfg16-nvfix collapse.
	bool coded = (env_i("MERCURY_SFO_GRID_CODED", 0) != 0);
	int  Kcw   = coded ? (nBits / ldpc.N) : 0;     // whole codewords that fit
	std::vector<int> tx_bits(nBits);
	std::vector<std::vector<int>> cw_info(Kcw);    // per-codeword info bits (for BER/CRC)
	ts_srandom((unsigned int)seed);

	// ---- PAS / PCS setup (fact-documents/data-flow-pas-shaping.md §1-§4) -------
	// 64-QAM per-symbol bit order (psk.cc :260/:328-331 enumerated):
	//   pos 0 = sign_re, pos 1,2 = |re| amp (hi,lo), pos 3 = sign_im, pos 4,5 = |im| amp.
	// Amplitude Gray map (hi,lo)->|a|: 00->7, 01->5, 11->3, 10->1; level index 0..3
	// (0=|1| nearest origin .. 3=|7|). pas_amp_pos[] marks the 4 amplitude bit
	// positions; pas_sign_pos the 2 sign positions. The DM emits level indices; we
	// Gray-encode each to (hi,lo) and stream them into the amplitude positions.
	cl_dist_matcher dm;
	static const int LEV_AMP[4]   = {1,3,5,7};            // level index -> magnitude
	// level index -> (hi,lo) amplitude bits (inverse of the Gray map above)
	static const int LEV_HILO[4][2] = { {1,0}, {1,1}, {0,1}, {0,0} }; // idx0=|1|..idx3=|7|
	const int pas_amp_pos[4]  = {1,2,4,5};
	bool pas_on = (pcs && coded && M == MOD_64QAM);
	if(pas_on)
	{
		if(!dm.configure_maxwell_boltzmann(4, LEV_AMP, pcs_lambda, pcs_rail_L))
		{
			std::cout << "[SFO-GRID-PCS] DM configure FAILED (lambda=" << pcs_lambda
			          << " rail_L=" << pcs_rail_L << ") -> falling back to uniform 64-QAM." << std::endl;
			pas_on = false;
		}
		else
		{
			std::cout << "[SFO-GRID-PCS] DM ready: levels{1,3,5,7} counts{"
			          << dm.level_count(0) << "," << dm.level_count(1) << ","
			          << dm.level_count(2) << "," << dm.level_count(3) << "}"
			          << " rail_L=" << dm.block_len() << " k_info=" << dm.info_bits()
			          << " (DM rate " << (double)dm.info_bits()/(double)(2*dm.block_len()) << ")"
			          << " lambda=" << pcs_lambda << std::endl;

			// PAS power re-normalization (fact-documents/data-flow-pas-shaping.md §0):
			// set_constellation normalized to unit UNIFORM power; under the shaped prior
			// the true average power is < 1, which at fixed Es/N0 would LOSE SNR. Build
			// the per-64-QAM-index prior P(idx)=0.25·P(level_re)·P(level_im) (signs uniform)
			// and rescale the constellation so the SHAPED average power = 1 — the freed
			// energy expands the minimum distance, and THAT expansion is the shaping gain.
			double levp[4];
			{
				double tot=(double)(dm.level_count(0)+dm.level_count(1)+dm.level_count(2)+dm.level_count(3));
				if(tot<1.0) tot=1.0;
				for(int l=0;l<4;l++) levp[l]=(double)dm.level_count(l)/tot;
			}
			std::vector<double> sym_prob(64, 0.0);
			for(int idx=0; idx<64; ++idx)
			{
				// per-symbol bit roles (psk.cc enumeration): (b4,b3)=ampRe Gray, (b1,b0)=ampIm.
				int b4=(idx>>4)&1, b3=(idx>>3)&1, b1=(idx>>1)&1, b0=idx&1;
				// Gray (hi,lo)->level index: invert LEV_HILO ({1,0}=idx0..{0,0}=idx3).
				auto hilo2lev = [&](int hi,int lo)->int{
					for(int l=0;l<4;l++) if(LEV_HILO[l][0]==hi && LEV_HILO[l][1]==lo) return l; return 0; };
				int lre=hilo2lev(b4,b3), lim=hilo2lev(b1,b0);
				sym_prob[idx] = 0.25 * levp[lre] * levp[lim];   // 0.5*0.5 for the two uniform signs
			}
			psk.rescale_shaped_power(sym_prob.data());
		}
	}

	if(coded)
	{
		std::vector<int> enc(ldpc.N);
		std::vector<int> info(ldpc.K);
		// PAS amplitude-bit source: a pre-shaped stream of amplitude bits the size of
		// the whole coded grid. Each rail block (block_len levels) is filled from the
		// DM (k_info random payload bits -> block_len levels -> 2*block_len amp bits).
		// We stream Gray (hi,lo) pairs from this buffer into the amplitude positions of
		// the per-symbol info bits below. This makes the TRANSMITTED 64-QAM amplitudes
		// Maxwell-Boltzmann-distributed (the shaping gain) while the LDPC still protects
		// the assembled info bits systematically.
		std::vector<int> amp_bit_stream;   // shaped amplitude bits (hi,lo,hi,lo,...)
		size_t amp_cursor = 0;
		if(pas_on)
		{
			int Lr  = dm.block_len();
			int kdm = dm.info_bits();
			// enough rail blocks to cover ALL amplitude bit positions in the grid:
			// 4 amplitude bits/symbol * nData symbols = 4*nData amp bits = 2*nData levels.
			long need_levels = 2L*(long)nData;
			long n_blocks    = (need_levels + Lr - 1) / Lr;
			std::vector<int> in_bits(kdm), out_levels(Lr);
			for(long blk=0; blk<n_blocks; ++blk)
			{
				for(int i=0;i<kdm;i++) in_bits[i] = (int)(ts_random()%2);
				if(!dm.shape(in_bits.data(), out_levels.data()))
				{ std::cout << "[SFO-GRID-PCS] dm.shape FAILED — abort PAS." << std::endl; pas_on=false; break; }
				for(int p=0;p<Lr;p++)
				{
					int lev = out_levels[p];
					amp_bit_stream.push_back(LEV_HILO[lev][0]);  // hi
					amp_bit_stream.push_back(LEV_HILO[lev][1]);  // lo
				}
			}
		}
		// Track the global symbol-grid bit position so amplitude vs sign positions are
		// identified by (globalbit % 6). The harness maps tx_bits contiguously, so the
		// info bits we assemble here land at the SAME positions psk.mod groups them into.
		long global_bit = 0;
		for(int c=0;c<Kcw;c++)
		{
			for(int i=0;i<ldpc.K;i++)
			{
				int bit;
				if(pas_on)
				{
					int posinsym = (int)((global_bit) % 6);
					bool is_amp = (posinsym==pas_amp_pos[0]||posinsym==pas_amp_pos[1]
					             ||posinsym==pas_amp_pos[2]||posinsym==pas_amp_pos[3]);
					if(is_amp && amp_cursor < amp_bit_stream.size())
						bit = amp_bit_stream[amp_cursor++];     // shaped amplitude bit
					else
						bit = (int)(ts_random()%2);             // uniform sign bit (or amp exhausted)
				}
				else
				{
					bit = (int)(ts_random()%2);
				}
				info[i] = bit;
				global_bit++;
			}
			cw_info[c] = info;
			ldpc.encode(info.data(), enc.data());   // enc = [K info | P parity] = N bits
			for(int i=0;i<ldpc.N;i++) tx_bits[(size_t)c*ldpc.N + i] = enc[i];
			// The P parity bits also advance the global symbol position (they are mapped
			// to symbols too); keep global_bit in sync with the mapped stream.
			global_bit += ldpc.P;
		}
		// Any leftover bits past K*N are random filler (not scored).
		for(int i=Kcw*ldpc.N; i<nBits; i++) tx_bits[i] = (int)(ts_random()%2);
	}
	else
	{
		for(int i=0;i<nBits;i++) tx_bits[i] = (int)(ts_random()%2);
	}

	std::vector<std::complex<double>> tx_syms(nData);
	psk.mod(tx_bits.data(), nBits, tx_syms.data());

	std::vector<std::complex<double>> grid((size_t)Ngrid*Nc);
	ofdm.framer(tx_syms.data(), grid.data());   // data + pilots placed by lattice

	// --- CHANNEL: flat unity H + the SFO per-symbol subcarrier phase ramp. ---
	// tau_n = accumulated timing slip (samples) at symbol n; phase(k) = -2*pi*k*tau_n/Nfft.
	// k index runs over the active subcarriers as the modulator places them
	// (zero_padder maps logical carrier j to FFT bin; the ramp is in the SAME
	// logical-carrier index the estimator/equalizer use, so a per-carrier index j
	// suffices — the bin mapping is a fixed permutation that cancels in est/eq).
	for(int n=0; n<Ngrid; n++)
	{
		double tau_n = ppm*1e-6 * (double)n * (double)Nofdm;   // samples slipped by symbol n
		for(int j=0;j<Nc;j++)
		{
			double ph = -2.0*M_PI*(double)j*tau_n/(double)Nfft;
			grid[(size_t)n*Nc+j] *= std::complex<double>(cos(ph), sin(ph));
		}
	}

	// --- FREQUENCY-SELECTIVE CHANNEL (the §13.5 production gap) -----------------
	// TEST-2 ran a FLAT unity channel: after the tracker removes the SFO ramp the
	// residual is |H|=1 everywhere, so the flat-ML H=mean(Y/X) estimate is exact.
	// A REAL HF/soundcard channel is frequency-SELECTIVE: |H| and phase VARY across
	// the 50 subcarriers even after the SFO timing ramp is removed, so the equalizer
	// needs a real per-subcarrier H interpolated from the sparse 6% pilots. This block
	// injects that selective channel as a per-carrier complex coefficient T(j) (the
	// channel is time-invariant across the block — frequency-selective only — so the
	// same T(j) multiplies every symbol; the SFO ramp above sits ON TOP of it).
	//
	// Two faithful selective models (gated by MERCURY_SFO_GRID_CHAN):
	//   1 = DET-FLOOR (phase-dispersive, |T|=1): the SHIPPED Schroeder all-pass
	//       cl_sim_det_floor (sim_channel.h:750, g=0.50/D=16/N=3). The frequency
	//       response of ONE all-pass section is the closed form
	//         A(e^{jw}) = (-g + e^{-jwD}) / (1 - g·e^{-jwD})   (|A|≡1, dispersive φ),
	//       cascaded ap_n times. |T|=1 (no amplitude null → 32-QAM tolerant) but the
	//       per-subcarrier PHASE varies → the flat-ML estimate (one global H̄) cannot
	//       represent it. This is the EVM-ceiling model the §10 bench shipped.
	//   2 = TWO-RAY (magnitude-selective): the fsel_test model (telecom_system.h:324,
	//       amp 0.6 @ delay 128 passband samples), T(j) = 1 + a·e^{-jw_j·Δ}, unit-power
	//       normalized. |T| FADES across the band (deep nulls) — the HARDER case the
	//       sparse interpolator must handle (it stresses BOTH magnitude and phase).
	// w_j is the TRUE centered FFT-bin angular frequency of logical carrier j (the
	// zero_padder mapping, ofdm.cc:331/697-723) so the ripple period is physical:
	//   k_centered(j) = (j < Nc/2) ? (j - Nc/2) : (j - Nc/2 + start_shift)
	//   w_j = 2*pi*k_centered / Nfft.
	int    chan_sel  = env_i("MERCURY_SFO_GRID_CHAN", 0);          // 0=flat,1=det-floor,2=two-ray
	double ap_g      = env_f("MERCURY_SFO_GRID_AP_G",   0.50);     // all-pass coeff (SHIPPED 0.50)
	int    ap_dly    = env_i("MERCURY_SFO_GRID_AP_DLY", 16);       // all-pass delay D (SHIPPED 16)
	int    ap_n      = env_i("MERCURY_SFO_GRID_AP_N",   3);        // all-pass cascade depth (SHIPPED 3)
	double tr_amp    = env_f("MERCURY_SFO_GRID_FSEL_AMP",   0.6);  // two-ray 2nd-ray amplitude
	int    tr_dly    = env_i("MERCURY_SFO_GRID_FSEL_DLY", 128);    // two-ray delay (passband samp)
	// chan_sel==3: TIME-VARYING WATTERSON (2-path Gaussian-Doppler). The
	// production-finalize question (§15): the freq-focused thin lattice (dy=8,
	// time-SPARSE) was tuned on a STATIC selective channel. A time-varying fade
	// makes H change ACROSS the ~1.3 s / 60-symbol block, so the per-symbol channel
	// estimate needs TIME-axis pilots (smaller dy) to track it. This block injects a
	// faithful 2-path Watterson channel so the layout sweep can find the slow-fade
	// operating envelope (depth × Doppler) the big-block holds vs hands off to the
	// gearshift. Model (Watterson 1970, CCIR 520; the standard HF channel sim):
	//   H(n,j) = g0(n) + g1(n)·e^{-j·w_j·Δ}
	//   g0 is the fixed LOS ray. g1 is a complex-Gaussian scatter tap shaped by
	//           the IONOS firmware's 128-tap adjusted-Gaussian FIR, independently
	//           on its I/Q rails. A new innovation enters the FIR every
	//           UPDATE=round(Fs/(64·fd)) PASSBAND samples and the FIR output is held
	//           between updates (zero-order hold). The grid has one H per OFDM
	//           symbol, so it samples that held process at each symbol boundary and
	//           advances the FIR clock by Nofdm·frequency_interpolation_rate physical
	//           samples per row. Thus f_sym=Fs/(Nofdm·interp), not Fs/Nofdm.
	//   FADE DEPTH D (dB): path 0 is a CONSTANT unit LOS ray (g0≡1, the stable
	//           direct path); path 1 is the RAYLEIGH-FADING scatter ray with mean
	//           amplitude a = 1 - 10^(-D/20) so its destructive combine drives the
	//           |H| null toward ~10^(-D/20) (≈ D dB below the 1.0 LOS reference) and
	//           its constructive combine to ~(1+a). This is the standard Rician /
	//           one-fixed-one-Doppler-spread-tap Watterson used for mild/slow fade
	//           characterization. D=0 ⇒ a=0 ⇒ H≡1 (TRULY FLAT, no fade — the clean
	//           baseline); D=6 ⇒ a≈0.50 (mild-deep, nulls to ~-6 dB); D=10 ⇒ a≈0.68
	//           (the deep fade the HW showed is out-of-scope, 0/8). ONLY g1 varies
	//           (at the Doppler rate); g0 is the constant LOS so depth/Doppler are
	//           independent axes. Es/N0 stays honest: the channel is unit-power-
	//           normalized over the block (mean Σ|H|²/(Ngrid·Nc) == 1).
	double watt_depth = env_f("MERCURY_SFO_GRID_WATT_DEPTH_DB", 0.0); // fade depth (dB)
	double watt_fd    = env_f("MERCURY_SFO_GRID_WATT_FD_HZ",    0.5); // Doppler spread (Hz)
	int    watt_dly   = env_i("MERCURY_SFO_GRID_WATT_DLY",     128);  // 2nd-path delay (passband samp)
	std::vector<std::complex<double>> Tchan(Nc, std::complex<double>(1.0,0.0));
	std::vector<std::complex<double>> Hwatt;   // (Ngrid×Nc) per-symbol time-varying H, chan_sel==3
	if(chan_sel == 3)
	{
		// IONOS 128 Tap Adj Gauss LPF Rev2 (64 Hz design update rate), lifted
		// verbatim from relay_doppler_ref.py / the firmware. DC gain is 1.0.
		static const double GAUS_FIR_COEFFS[128] = {
			1.1755592671332046e-11, 2.0188004956137427e-10, 1.7236333623946176e-09, 9.815423109243151e-09,
			4.219820040519088e-08, 1.4693429486234634e-07, 4.338503956649552e-07, 1.122118806000393e-06,
			2.604091536729611e-06, 5.522713327023963e-06, 1.0857390465642334e-05, 2.0011412649247494e-05,
			3.4893068706162795e-05, 5.7982477259392286e-05, 9.237678934582388e-05, 0.00014180767284007714,
			0.00021062669000635658, 0.0003037561533907518, 0.0004266051229102419, 0.000584952237938201,
			0.0007847989367901134, 0.001032198204838678, 0.001333065243166745, 0.001692977321877409,
			0.002116970561258896, 0.002609341477645015, 0.003173460865247882, 0.00381160700116864,
			0.004524824309342139, 0.005312812558055807, 0.006173850455688009, 0.007104756211217515,
			0.008100886298035966, 0.00915617235512072, 0.010263194925878348, 0.01141329161173599,
			0.012596696236577472, 0.013802704802828978, 0.015019863385625786, 0.016236172665450826,
			0.01743930354214716, 0.01861681819809535, 0.019756391073966928, 0.020846024470695095,
			0.021874253876569306, 0.02283033861665188, 0.023704434009553566, 0.02448774186995001,
			0.02517263689027796, 0.025752767148979578, 0.026223127704178725, 0.026580106921515002,
			0.02682150583616039, 0.026946531447567135, 0.026955765379786157, 0.02685110980161695,
			0.026635712883536795, 0.026313876369100774, 0.025890948056565766, 0.025373202123371387,
			0.024767710285281262, 0.024082206768594926, 0.02332494999439922, 0.022504583735910823,
			0.02163000032189053, 0.020710208229666707, 0.019754206149457228, 0.018770865316331563,
			0.01776882160593159, 0.016756378583127243, 0.01574142238665159, 0.01473134903422507,
			0.013733004447687326, 0.012752637231260112, 0.011795863992392318, 0.010867646776884902,
			0.009972282000446704, 0.009113400098909145, 0.008293974989634013, 0.007516342337046732,
			0.006782225544921226, 0.006092768355660706, 0.005448572920512081, 0.004849742212182516,
			0.004295925680163602, 0.003786367096475506, 0.003319953602663025, 0.002895265044807468,
			0.002510622769190125, 0.002164137144270543, 0.0018537531721837, 0.001577293652557479,
			0.001332499460868328, 0.001117066600796574, 0.0009286797833839573, 0.0007650423737761393,
			0.0006239026277671727, 0.0005030762143350815, 0.0004004650862100223, 0.0003140728178353742,
			0.00024201657867910556, 0.00018253594974316996, 0.00013399882249807025, 9.49046426887381e-05,
			6.388527699708468e-05, 3.970378899074398e-05, 2.1251412801076355e-05, 7.5430092767428655e-06,
			-2.2887192936932196e-06, -8.999992637884094e-06, -1.3243447148502327e-05, -1.557613368708468e-05,
			-1.6467811426850445e-05, -1.63093528492861e-05, -1.5421097939455242e-05, -1.4061018704922785e-05,
			-1.243257788819571e-05, -1.0692187735418308e-05, -8.956195575428226e-06, -7.3073424710351094e-06,
			-5.8006591112396305e-06, -4.468779263172823e-06, -3.3266653970160216e-06, -2.3757534892377295e-06,
			-1.6075344987453848e-06, -1.0065986371058506e-06, -5.531753923550552e-07, -2.252074190014706e-07
		};

		double Fs = (double)sampling_frequency;  // physical passband sample rate (normally 48 kHz)
		long long symbol_samples = (long long)Nofdm * (long long)frequency_interpolation_rate;
		if(symbol_samples < 1) symbol_samples = 1;
		bool gaus_doppler = (watt_fd > 0.0);

		// fd<=0 is a compatibility gate: retain the old pole, initialization, RNG
		// draw count, and per-symbol arithmetic byte-for-byte. Only fd>0 enters the
		// Gaussian-FIR path below.
		double f_sym_legacy = (Nofdm>0) ? (Fs / (double)Nofdm) : 1.0;
		double rho_legacy = std::exp(-2.0*M_PI*watt_fd / f_sym_legacy);
		if(rho_legacy > 0.999999) rho_legacy = 0.999999;
		double inn_legacy = std::sqrt(1.0 - rho_legacy*rho_legacy);
		double a     = 1.0 - std::pow(10.0, -watt_depth/20.0); // 2nd-ray MEAN amplitude
		if(a < 0.0) a = 0.0;
		if(a > 0.99) a = 0.99;
		cl_sim_xoshiro wrng(seed ^ 0xC0FFEEu);
		auto cgauss = [&](){ return std::complex<double>(wrng.gauss(), wrng.gauss())/std::sqrt(2.0); };

		// Gaussian-FIR state for the scatter path: two 128-sample rings, newest
		// innovation at fir_head. Innovation variance per rail is 0.5/sum(h^2),
		// giving var(I)=var(Q)=0.5 and E[|g|^2]=1 at steady state.
		double fir_i[128] = {0.0};
		double fir_q[128] = {0.0};
		int fir_head = 0;
		double fir_sumsq = 0.0;
		for(int k=0;k<128;k++) fir_sumsq += GAUS_FIR_COEFFS[k]*GAUS_FIR_COEFFS[k];
		double fir_inno_std = gaus_doppler ? std::sqrt(0.5/fir_sumsq) : 0.0;
		auto fir_push = [&]() {
			fir_head = (fir_head + 127) & 127;
			fir_i[fir_head] = wrng.gauss() * fir_inno_std;
			fir_q[fir_head] = wrng.gauss() * fir_inno_std;
		};
		auto fir_output = [&]() {
			double gi=0.0, gq=0.0;
			for(int k=0;k<128;k++)
			{
				int idx = (fir_head + k) & 127;
				gi += GAUS_FIR_COEFFS[k] * fir_i[idx];
				gq += GAUS_FIR_COEFFS[k] * fir_q[idx];
			}
			return std::complex<double>(gi,gq);
		};
		long long update_samples = 1;
		long long update_pos = 0;
		std::complex<double> scatter_hold(0.0,0.0);
		if(gaus_doppler)
		{
			// Python round() is nearest-with-ties-to-even; preserve that cadence
			// rule rather than C++ llround()'s half-away-from-zero behavior.
			double update_exact = Fs/(64.0*watt_fd);
			double update_floor = std::floor(update_exact);
			update_samples = (long long)update_floor;
			double update_frac = update_exact - update_floor;
			if(update_frac > 0.5 || (update_frac == 0.5 && (update_samples & 1LL)))
				update_samples++;
			if(update_samples < 1) update_samples = 1;
			// Prime through a full FIR history exactly like DopplerTap.__init__, so
			// the first grid symbol starts at the stationary distribution.
			for(int k=0;k<128;k++) fir_push();
			scatter_hold = fir_output();
		}
		// g0 = CONSTANT unit LOS ray; g1 = Rayleigh-fading scatter ray (mean amp a),
		// initialized at its stationary distribution (start already faded, not 0).
		std::complex<double> g0(1.0, 0.0);              // fixed LOS (D=0 ⇒ H≡1)
		std::complex<double> g1 = gaus_doppler ? (a*scatter_hold) : (a*cgauss());
		int sshift = ofdm.start_shift;
		// precompute per-carrier 2nd-path phase e^{-j w_j Δ}
		std::vector<std::complex<double>> ph2(Nc);
		for(int j=0;j<Nc;j++)
		{
			int k_centered = (j < Nc/2) ? (j - Nc/2) : (j - Nc/2 + sshift);
			double w = 2.0*M_PI*(double)k_centered/(double)Nfft;
			ph2[j] = std::polar(1.0, -w*(double)watt_dly);
		}
		Hwatt.assign((size_t)Ngrid*Nc, std::complex<double>(1.0,0.0));
		double psum=0.0;
		for(int n=0;n<Ngrid;n++)
		{
			// Sample the held scatter gain at this OFDM-symbol boundary. The LOS tap
			// stays fixed so depth and Doppler remain independent axes. fd<=0 keeps
			// the exact legacy advance order for the byte-identical compatibility gate.
			if(!gaus_doppler)
				g1 = rho_legacy*g1 + (a*inn_legacy)*cgauss();
			else
				g1 = a*scatter_hold;
			for(int j=0;j<Nc;j++)
			{
				std::complex<double> H = g0 + g1*ph2[j];
				Hwatt[(size_t)n*Nc+j] = H;
				psum += std::norm(H);
			}
			if(gaus_doppler)
			{
				// DopplerTap.advance(symbol_samples): advance on the physical-sample
				// clock and update the held FIR output at every 64*fd boundary. There
				// is deliberately no interpolation between updates (firmware ZOH).
				long long left = symbol_samples;
				while(left > 0)
				{
					long long span = update_samples - update_pos;
					long long take = std::min(span, left);
					update_pos += take;
					left -= take;
					if(update_pos >= update_samples)
					{
						update_pos = 0;
						fir_push();
						scatter_hold = fir_output();
					}
				}
			}
		}
		// unit-power normalize over the whole block so the AWGN Es/N0 axis is exact
		double gnorm = (psum>0.0) ? std::sqrt((double)(Ngrid*Nc)/psum) : 1.0;
		for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
		{
			Hwatt[(size_t)n*Nc+j] *= gnorm;
			grid[(size_t)n*Nc+j]  *= Hwatt[(size_t)n*Nc+j];
		}
		// expose the time-averaged |T| envelope in Tchan[] for the diagnostics print
		for(int j=0;j<Nc;j++)
		{
			std::complex<double> acc(0,0);
			for(int n=0;n<Ngrid;n++) acc += Hwatt[(size_t)n*Nc+j];
			Tchan[j] = acc / (double)Ngrid;
		}
	}
	else if(chan_sel != 0)
	{
		int sshift = ofdm.start_shift;
		for(int j=0;j<Nc;j++)
		{
			int k_centered = (j < Nc/2) ? (j - Nc/2) : (j - Nc/2 + sshift);
			double w = 2.0*M_PI*(double)k_centered/(double)Nfft;
			std::complex<double> T(1.0,0.0);
			if(chan_sel == 1)
			{
				// Schroeder all-pass cascade A(e^{jw})^ap_n.
				std::complex<double> z_mD = std::polar(1.0, -w*(double)ap_dly);   // e^{-jwD}
				std::complex<double> A = (-ap_g + z_mD) / (1.0 - ap_g*z_mD);
				T = std::pow(A, ap_n);
			}
			else // chan_sel == 2: two-ray magnitude-selective
			{
				T = 1.0 + tr_amp * std::polar(1.0, -w*(double)tr_dly);
			}
			Tchan[j] = T;
		}
		// Two-ray: unit-power normalize so the AWGN Es/N0 axis stays exact
		// (Σ|T|²/Nc == 1). The all-pass is already unit-magnitude → norm == 1.
		if(chan_sel == 2)
		{
			double p=0.0; for(int j=0;j<Nc;j++) p += std::norm(Tchan[j]);
			double g = (p>0.0) ? sqrt((double)Nc/p) : 1.0;
			for(int j=0;j<Nc;j++) Tchan[j] *= g;
		}
		for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
			grid[(size_t)n*Nc+j] *= Tchan[j];
	}
	// Channel diagnostics: |T| min/max (selectivity depth) + phase spread.
	{
		double tmin=1e9, tmax=-1e9; double phmin=1e9, phmax=-1e9;
		for(int j=0;j<Nc;j++){ double m=std::abs(Tchan[j]); if(m<tmin)tmin=m; if(m>tmax)tmax=m;
			double p=std::arg(Tchan[j]); if(p<phmin)phmin=p; if(p>phmax)phmax=p; }
		// chan_sel==3 (Watterson): also report the INSTANTANEOUS |H| span over the
		// whole space-time block (the wandering null's true depth) + the per-carrier
		// TIME variation (how much H drifts symbol-to-symbol = what the time-axis
		// pilots must track). max time-drift = max_j std_n(|H(n,j)|).
		if(chan_sel==3 && !Hwatt.empty())
		{
			double hmn=1e9, hmx=-1e9, maxdrift=0.0;
			for(int j=0;j<Nc;j++)
			{
				double s=0,ss=0;
				for(int n=0;n<Ngrid;n++){ double m=std::abs(Hwatt[(size_t)n*Nc+j]);
					if(m<hmn){hmn=m;} if(m>hmx){hmx=m;} s+=m; ss+=m*m; }
				double mean=s/Ngrid, var=ss/Ngrid-mean*mean; if(var<0)var=0;
				double sd=std::sqrt(var); if(sd>maxdrift)maxdrift=sd;
			}
			std::cout << "[SFO-GRID] watterson depth=" << watt_depth << "dB fd=" << watt_fd
			          << "Hz dly=" << watt_dly << " |H|inst[" << hmn << ".." << hmx << "]"
			          << " max_time_drift(sd|H|)=" << maxdrift << std::endl;
		}
		const char* cname = (chan_sel==1)?"DET-FLOOR(all-pass,|T|=1,phase-disp)"
		                  : (chan_sel==2)?"TWO-RAY(magnitude-selective)"
		                  : (chan_sel==3)?"WATTERSON(time-varying 2-path)" : "FLAT(unity)";
		bool spx = (env_i("MERCURY_SFO_GRID_SPARSE2D",0)!=0);
		std::cout << "[SFO-GRID] channel=" << cname
		          << " |T|[" << tmin << ".." << tmax << "]"
		          << " arg(T)[" << phmin << ".." << phmax << "] rad"
		          << "  estimator=" << (no_interp?"NOINTERP" : (thin?(spx?"SPARSE-2D-INTERP":"FLAT-ML(control)"):"LS+DFT"))
		          << std::endl;
	}

	// AWGN (Es/N0): added per data/pilot subcarrier so noise_variance_estimate is a
	// meaningful quantity for the CODED path. Es/N0 >= 900 => clean (no noise, default
	// for the uncoded timing probe). The 32-QAM symbols carry unit average Es here
	// (psk.mod normalization), so sigma^2 = 1/(2*Es/N0) per real dimension.
	double grid_esn0 = env_f("MERCURY_SFO_GRID_ESN0", 900.0);
	if(grid_esn0 < 900.0)
	{
		double snr_lin = pow(10.0, grid_esn0/10.0);
		double sigma   = 1.0 / sqrt(snr_lin);   // per-complex-sample std (Es=1)
		awgn_channel.set_seed((long)((seed ^ 0xA5A5A5A5u) | 1));
		for(int ci=0; ci<Ngrid*Nc; ci++)
		{
			double nr = (sigma/sqrt(2.0)) * awgn_channel.awgn_value_generator();
			double ni = (sigma/sqrt(2.0)) * awgn_channel.awgn_value_generator();
			grid[ci] += std::complex<double>(nr, ni);
		}
	}

	// --- RX: ONE channel estimate across the whole 60-grid, then equalize. ---
	// (Acquisition is not re-tested here — sfo_block_test proved the head preamble
	//  is found; this isolates the GRID estimate's ability to hold the ramp.)
	// Copy the (drifted) grid into a demod buffer the estimator reads.
	std::vector<std::complex<double>> rx(grid);

	// STEP 2 corrector: per-symbol CPE/PEG LS de-rotation BEFORE the estimator/equalizer.
	if(track)
	{
		// For each symbol n, gather pilot residual phase err vs carrier index k,
		// LS-fit a line (intercept=CPE omega, slope=PEG delta), average over a
		// +/- win/2 sliding window of symbols, de-rotate exp(-j[omega+k*delta]).
		// Pilot value is known (pilot_configurator.sequence); residual phase =
		// arg(rx / pilot) at each pilot carrier (flat H => 0 without SFO).
		std::vector<double> sym_omega(Ngrid,0.0), sym_delta(Ngrid,0.0);
		// first pass: per-symbol raw LS fit from that symbol's pilots
		int pilot_index_base = 0;
		// Need the pilot_index offset per symbol; recompute by walking the lattice.
		std::vector<int> pidx_at_symbol(Ngrid,0);
		{
			int pidx=0;
			for(int n=0;n<Ngrid;n++){ pidx_at_symbol[n]=pidx;
				for(int j=0;j<Nc;j++) if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT) pidx++; }
		}
		(void)pilot_index_base;
		for(int n=0;n<Ngrid;n++)
		{
			int pidx = pidx_at_symbol[n];
			double Sx=0,Sy=0,Sxx=0,Sxy=0; int np=0;
			for(int j=0;j<Nc;j++)
			{
				if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT)
				{
					std::complex<double> X = ofdm.pilot_configurator.sequence[pidx++];
					std::complex<double> r = rx[(size_t)n*Nc+j] / X;   // = H_eff = unity*ramp
					double phi = atan2(r.imag(), r.real());            // wrapped phase err
					double x=(double)j;
					Sx+=x; Sy+=phi; Sxx+=x*x; Sxy+=x*phi; np++;
				}
			}
			if(np>=2){ double den=np*Sxx-Sx*Sx;
				if(fabs(den)>1e-12){ sym_delta[n]=(np*Sxy-Sx*Sy)/den; sym_omega[n]=(Sy-sym_delta[n]*Sx)/np; } }
		}
		// sliding-window average + de-rotate every carrier of every symbol
		for(int n=0;n<Ngrid;n++)
		{
			double om=0,dl=0; int cnt=0;
			for(int w=n-win/2; w<=n+win/2; w++){ if(w>=0&&w<Ngrid){ om+=sym_omega[w]; dl+=sym_delta[w]; cnt++; } }
			if(cnt>0){ om/=cnt; dl/=cnt; }
			for(int j=0;j<Nc;j++)
			{
				double ph = -(om + dl*(double)j);
				rx[(size_t)n*Nc+j] *= std::complex<double>(cos(ph), sin(ph));
			}
		}
	}

	// Channel estimate + equalize over the ONE 60-grid.
	if(no_interp)
	{
		// NEGATIVE CONTROL: estimate ONLY from the head symbols' pilots and FREEZE
		// (no per-block bilinear interpolation tracking the ramp). Implemented by
		// assigning a flat unity estimate everywhere — the ramp is then NOT removed,
		// so the tail MUST fail. Proves the harness is not too lenient.
		for(int ci=0; ci<Ngrid*Nc; ci++){ (ofdm.estimated_channel+ci)->value = std::complex<double>(1.0,0.0);
			(ofdm.estimated_channel+ci)->status = MEASURED; }
	}
	else if(env_i("MERCURY_SFO_GRID_GENIE", 0))
	{
		// Turbo-EQ (RESEARCH_turbo-eq.md §7): the `thin &&` guard is dropped so GENIE is
		// also reachable on the DENSE Dx=1/Dy=3 production lattice — the compass the turbo
		// lever chases (GENIE decodes the POOR/1 Hz Watterson where LS fails the Dy=3
		// Nyquist wall). The genie H is lattice-independent (true H at every cell), so this
		// is additive: the prior thin&&GENIE behavior is unchanged.
		// GENIE estimate: hand the equalizer the EXACT channel (Tchan[j] × the SFO ramp
		// at symbol n, if the tracker is OFF). Isolates the LDPC/SNR decodability limit on
		// the selective channel from the estimator quality — if genie still FAILS to decode
		// at a given EsN0, the channel+SNR is past the waterfall and NO estimator helps.
		for(int n=0;n<Ngrid;n++)
		{
			double tau_n = ppm*1e-6 * (double)n * (double)Nofdm;
			for(int j=0;j<Nc;j++)
			{
				double ph = (track ? 0.0 : -2.0*M_PI*(double)j*tau_n/(double)Nfft);
				// chan_sel==3: the TRUE channel is the per-symbol time-varying Hwatt[n][j];
				// otherwise the static Tchan[j]. The SFO ramp sits on top in both cases.
				std::complex<double> Hsel = (chan_sel==3 && !Hwatt.empty())
				                          ? Hwatt[(size_t)n*Nc+j] : Tchan[j];
				std::complex<double> Hg = Hsel * std::complex<double>(cos(ph), sin(ph));
				(ofdm.estimated_channel+n*Nc+j)->value = Hg;
				(ofdm.estimated_channel+n*Nc+j)->status = MEASURED;
			}
		}
		// genie nv = the true AWGN variance (so CSI/MMSE weighting is correctly scaled).
		double ge = env_f("MERCURY_SFO_GRID_ESN0", 900.0);
		ofdm.noise_variance_estimate = (ge<900.0) ? pow(10.0,-ge/10.0) : 1e-6;
		if(ofdm.noise_variance_estimate < 1e-6) ofdm.noise_variance_estimate = 1e-6;
	}
	else if(thin && env_i("MERCURY_SFO_GRID_SPARSE2D", 0))
	{
		grid_sparse2d_estimator(rx.data(), Ngrid, Nc);
	}
	else if(thin)
	{
		// FLAT-ML CONTROL (the §13.2 flat-channel shortcut). The stock
		// LS_channel_estimator assumes a DENSE REGULAR (Dx=1/Dy=3) lattice: per-cell
		// global-window scalar + a per-symbol DFT smoother (smooth_channel_estimate_dft,
		// ofdm.cc:2127). On a SPARSE IRREGULAR lattice it SMEARS (|H| 0.05..1.37 on a
		// UNITY channel -> BER 0.34 @0ppm). On a FLAT channel (TEST 2) the ML estimate
		// is the pilot-averaged complex gain H̄ = mean(Y_pilot/X_pilot) assigned to every
		// cell — EXACT for flat. On a frequency-SELECTIVE channel this single global
		// scalar CANNOT represent |H| and phase varying across the 50 subcarriers, so it
		// MUST FAIL — this is the negative control that proves the sparse 2D interpolator
		// (MERCURY_SFO_GRID_SPARSE2D=1) is needed and works.
		std::complex<double> Hsum(0,0); int pidx=0; int npil=0;
		for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
			if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT)
			{
				std::complex<double> X = ofdm.pilot_configurator.sequence[pidx++];
				Hsum += rx[(size_t)n*Nc+j] / X;   // Y/X per pilot (post-tracker ~unity on FLAT)
				npil++;
			}
		std::complex<double> Hbar = (npil>0) ? (Hsum / (double)npil) : std::complex<double>(1.0,0.0);
		// Honest residual noise variance against the flat estimate (pilot EVM).
		double nsum=0.0; pidx=0;
		for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
			if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT)
			{
				std::complex<double> X = ofdm.pilot_configurator.sequence[pidx++];
				std::complex<double> resid = rx[(size_t)n*Nc+j] - Hbar*X;
				nsum += resid.real()*resid.real() + resid.imag()*resid.imag();
			}
		for(int ci=0; ci<Ngrid*Nc; ci++){ (ofdm.estimated_channel+ci)->value = Hbar;
			(ofdm.estimated_channel+ci)->status = MEASURED; }
		ofdm.noise_variance_estimate = (npil>0) ? (nsum/(double)npil) : 0.01;
		if(ofdm.noise_variance_estimate < 1e-6) ofdm.noise_variance_estimate = 1e-6;
	}
	else if(env_i("MERCURY_SFO_GRID_TINTERP_PROD", 0) != 0 || turbo_seed_tinterp)
	{
		// feat/fade-tinterp REGRESSION HOOK: drive the PRODUCTION TIME_INTERP
		// estimator (ofdm.cc LS_channel_estimator_tinterp) on the dense Dx=1/Dy=3
		// (turbo_seed_tinterp also lands here so MERCURY_SFO_GRID_TURBO_SEED=tinterp
		//  seeds it=0 with the warm faded TINTERP estimate per the §5 recommended stack)
		// prod lattice — the exact promoted code, including the production cross-
		// pilot AWGN nv-floor (NOT the harness prototype's known-EsN0 floor). This is
		// the failing-first regression: on the GOOD (MPG/0.1 Hz) Watterson cell the
		// final `else { LS_channel_estimator }` decodes 0/K codewords; this branch
		// decodes K/K, proving the promoted production estimator carries the win and
		// the nv-floor (no 1e-6 collapse). A/B vs the prod else on the SAME seed.
		int sm = env_i("MERCURY_SFO_GRID_TINTERP_SMOOTH", 0);
		ofdm.tinterp_smooth_halfwin = (sm > 0) ? sm : 0;
		// Test-only anchor-count arm: after waveform generation, retain exactly the
		// requested number of valid symbols for the production TINTERP estimator.
		// A negative value leaves the normal grid unchanged.
		int valid_pilot_limit = env_i("MERCURY_SFO_GRID_TINTERP_VALID_PILOTS", -1);
		if(valid_pilot_limit >= 0)
		{
			int pidx = 0;
			for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
				if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT)
				{
					if(pidx >= valid_pilot_limit)
						ofdm.pilot_configurator.sequence[pidx] = std::complex<double>(0.0, 0.0);
					pidx++;
				}
		}
		ofdm.LS_channel_estimator_tinterp(rx.data());
		std::cout << "[SFO-GRID-EST] cand=TINTERP_PROD smooth=" << ofdm.tinterp_smooth_halfwin
		          << " nv=" << ofdm.noise_variance_estimate << std::endl;
	}
	else
	{
		ofdm.LS_channel_estimator(rx.data());   // pilots -> interpolate across 60-grid
	}
	sfo_grid_last_noise_variance = ofdm.noise_variance_estimate;

	// Capture per-data-carrier CSI weight |H_k|² (in deframed raster order) BEFORE the
	// equalizer wipes estimated_channel[].status. On a frequency-SELECTIVE channel the
	// deep-magnitude-null carriers are noise-amplified by ZF (1/|H|²); a SCALAR nv makes
	// the LDPC over-confident on those carriers and decode fails. The production path
	// (telecom_system.cc:2874-2927) scales each carrier's LLRs by normalized |H_k|² so the
	// decoder discounts the weak carriers — the SAME CSI weighting is required here for the
	// selective channel (on a flat channel all weights ≈ 1 → no change, so TEST-2 holds).
	std::vector<double> csi_data(nData, 1.0);
	{
		int di=0;
		for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
			if((ofdm.ofdm_frame+n*Nc+j)->type==DATA)
			{
				std::complex<double> H=(ofdm.estimated_channel+n*Nc+j)->value;
				if(di<nData) csi_data[di]=H.real()*H.real()+H.imag()*H.imag();
				di++;
			}
	}

	std::vector<std::complex<double>> eq((size_t)Ngrid*Nc);
	ofdm.channel_equalizer(rx.data(), eq.data());

	std::vector<std::complex<double>> deframed(nData);
	ofdm.deframer(eq.data(), deframed.data());

	if(env_i("MERCURY_SFO_GRID_DIAG",0))
	{
		std::cout << "[SFO-GRID-DIAG] nv=" << ofdm.noise_variance_estimate << std::endl;
		for(int d=0; d<4 && d<nData; d++)
			std::cout << "[SFO-GRID-DIAG] data["<<d<<"] tx=(" << tx_syms[d].real()<<","<<tx_syms[d].imag()
			          << ") eqdeframed=(" << deframed[d].real()<<","<<deframed[d].imag()<<")" << std::endl;
		// also dump a couple of estimated_channel values at DATA positions
		std::cout << "[SFO-GRID-DIAG] H[sym0,c0]=(" << (ofdm.estimated_channel+0)->value.real()
		          << "," << (ofdm.estimated_channel+0)->value.imag() << ")"
		          << " H[sym30,c25]=(" << (ofdm.estimated_channel+30*Nc+25)->value.real()
		          << "," << (ofdm.estimated_channel+30*Nc+25)->value.imag() << ")" << std::endl;
		// Lattice histogram (pilots per symbol) + |H| stats across the grid. The
		// pilots/sym min is the tracker's per-symbol LS-fit support (needs >=2). The
		// |H| min/max/mean is the channel-estimate sanity: on this flat-unity grid a
		// healthy estimate reads ~1.0 everywhere; the per-cell-LS+DFT-smoother path
		// SMEARS it on a sparse lattice (|H| 0.05..1.37, mean 0.38 -> BER 0.34), which
		// is why the thin path uses the flat-channel pilot-averaged estimator instead.
		int pmin=Nc+1, pmax=-1; double psum=0;
		for(int n=0;n<Ngrid;n++){ int pc=0; for(int j=0;j<Nc;j++){ if((ofdm.ofdm_frame+n*Nc+j)->type==PILOT) pc++; }
			if(pc<pmin){ pmin=pc; } if(pc>pmax){ pmax=pc; } psum+=pc; }
		double hmin=1e9,hmax=-1e9,hsum=0;
		for(int ci=0;ci<Ngrid*Nc;ci++){ std::complex<double> H=(ofdm.estimated_channel+ci)->value;
			double m=std::abs(H); if(m<hmin){ hmin=m; } if(m>hmax){ hmax=m; } hsum+=m; }
		std::cout << "[SFO-GRID-DIAG] pilots/sym min="<<pmin<<" max="<<pmax<<" mean="<<(psum/Ngrid)
		          << " |H| min="<<hmin<<" max="<<hmax<<" mean="<<(hsum/(Ngrid*Nc))<<std::endl;
	}

	// --- SCORE: per-symbol uncoded symbol-error-rate, head vs tail. ---
	// Hard-decision: re-mod the known bits per data carrier and compare nearest.
	// Use psk.demod (LLR) -> hard bits and count bit errors, bucketed by symbol.
	std::vector<float> llr(nBits);
	double variance = ofdm.measure_variance(rx.data());
	psk.demod(deframed.data(), nBits, llr.data(), (float)variance);

	// Map each data carrier back to its symbol index to bucket errors head vs tail.
	// data carriers are filled by framer in (symbol,carrier) raster order over DATA
	// positions; reconstruct that order to know which symbol each data index lands in.
	std::vector<int> data_symbol_of_idx(nData,0);
	{
		int di=0;
		for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
			if((ofdm.ofdm_frame+n*Nc+j)->type==DATA){ if(di<nData) data_symbol_of_idx[di]=n; di++; }
	}
	long head_err=0, head_tot=0, tail_err=0, tail_tot=0, all_err=0;
	int tail_start = (Ngrid*3)/4;   // last quarter = "tail"
	for(int b=0;b<nBits;b++)
	{
		int hard = (llr[b] < 0.0f) ? 1 : 0;   // LLR sign -> bit (demod convention)
		int bit_err = (hard != tx_bits[b]) ? 1 : 0;
		all_err += bit_err;
		int data_idx = b / log2M;
		int sym = (data_idx<nData)?data_symbol_of_idx[data_idx]:0;
		if(sym < Ngrid/4){ head_tot++; head_err+=bit_err; }
		else if(sym >= tail_start){ tail_tot++; tail_err+=bit_err; }
	}
	double ber_all  = (double)all_err/(double)nBits;
	double ber_head = head_tot? (double)head_err/(double)head_tot : 0.0;
	double ber_tail = tail_tot? (double)tail_err/(double)tail_tot : 0.0;

	std::cout << "[SFO-GRID] ===== RESULT (one estimate across " << Ngrid << " symbols) =====" << std::endl;
	std::cout << "[SFO-GRID]   uncoded_BER_all=" << ber_all
	          << "  HEAD(sym0.." << (Ngrid/4-1) << ")=" << ber_head
	          << "  TAIL(sym" << tail_start << ".." << (Ngrid-1) << ")=" << ber_tail << std::endl;
	std::cout << "[SFO-GRID]   tot biterr=" << all_err << "/" << nBits
	          << "  mode=" << (no_interp?"NOINTERP(neg-ctrl)":(track?"CPE/PEG-TRACK":"2D-PILOT-EST"))
	          << std::endl;
	std::cout << "[SFO-GRID]   make-or-break: TAIL BER ~0 => grid holds the SFO ramp;"
	          << " TAIL >> HEAD => ramp walks the tail off (needs tracker)." << std::endl;

	// ====================================================================
	// CODED K-CODEWORD DECODE (requirement (b)): real rate-0.875 LDPC block
	// decode at the achieved pilot fraction, under SFO, with the tracker.
	// ====================================================================
	// The first Kcw*N data bits carry Kcw systematic codewords. Demap the SAME
	// deframed data carriers with the production noise_variance_estimate (the
	// quantity the E1/cfg16-nvfix bug collapses), then LDPC-decode each 1600-bit
	// codeword and report decoded/iter. nv is REPORTED so the GO/NO-GO can confirm
	// it did NOT collapse to ~1e-6 (over-confident LLRs -> BP iter caps at 101).
	if(coded)
	{
		// ---- PAS LLR de-shaping prior table (fact-documents/data-flow-pas-shaping.md §4) -
		// Per-output-bit-position log-prior log_prior[pos] = ln P(bit=0) − ln P(bit=1)
		// under the shaped amplitude distribution; pos 0,3 (sign) = 0 (uniform). Built
		// from the DM composition. Passed to demod_pas; demod (uniform) used otherwise.
		double pas_log_prior[6] = {0,0,0,0,0,0};
		if(pas_on)
		{
			// Marginal P(amp bit=0) for the 'hi' and 'lo' positions over the 4 levels.
			double tot = (double)(dm.level_count(0)+dm.level_count(1)+dm.level_count(2)+dm.level_count(3));
			if(tot < 1.0) tot = 1.0;
			double phi0=0.0, plo0=0.0;   // P(hi=0), P(lo=0)
			for(int lev=0; lev<4; ++lev)
			{
				double p = (double)dm.level_count(lev)/tot;
				if(LEV_HILO[lev][0]==0) phi0 += p;
				if(LEV_HILO[lev][1]==0) plo0 += p;
			}
			auto lp = [](double p0){ double a=(p0<1e-12?1e-12:p0), b=(1.0-p0<1e-12?1e-12:1.0-p0);
			                          return std::log(a)-std::log(b); };
			// positions: 1,4 = 'hi'; 2,5 = 'lo' (pas_amp_pos={1,2,4,5}; sign at 0,3=0).
			pas_log_prior[1] = lp(phi0);
			pas_log_prior[4] = lp(phi0);
			pas_log_prior[2] = lp(plo0);
			pas_log_prior[5] = lp(plo0);
		}

		// ---- CFG17 composition: NV_FORCE + ratio-nvfix -----------------------------
		// MERCURY_SFO_GRID_NV_FORCE>0: override the estimator's nv with a tiny forced
		// value, REPRODUCING the HW-only ~1000x post-EQ-EVM nv-collapse (which the
		// in-process sim never reproduces, PCS_VERDICT.md / nvfix a0e22c8) so the
		// ratio-gate is testable in sim. Default unset/<=0 -> nv untouched.
		double nv_force = env_f("MERCURY_SFO_GRID_NV_FORCE", 0.0);
		if(nv_force > 0.0) ofdm.noise_variance_estimate = nv_force;
		// MERCURY_SFO_GRID_NVFIX: apply the production RATIO-GATED demap-variance
		// (telecom_system.cc:2992-2999) -> demap_variance = (nv < measure_var/K) ?
		// measure_var : nv, K=8. `variance` (=measure_variance(rx), :7102) IS the
		// post-EQ measured noise the Euclidean demapper needs. It defaults on only for
		// the thin lattice and remains default-off for the dense shipping configs.
		// Explicit 0/1 values remain available for controlled A/B tests.
		bool   nvfix = (env_i("MERCURY_SFO_GRID_NVFIX", thin ? 1 : 0) != 0);
		const double NV_COLLAPSE_RATIO_K = 8.0;

		// Per-carrier LLR for the whole grid (production demapper + production nv).
		std::vector<float> clr(nBits);
		double cvar = ofdm.noise_variance_estimate;   // the estimator's nv (NOT measure_variance)
		if(nvfix && cvar < variance / NV_COLLAPSE_RATIO_K) cvar = variance;   // ratio-gate (a0e22c8)
		if(cvar < 1e-9) cvar = 1e-9;
		if(pas_on) psk.demod_pas(deframed.data(), nBits, clr.data(), (float)cvar, pas_log_prior);
		else       psk.demod    (deframed.data(), nBits, clr.data(), (float)cvar);

		// CSI-weighted LLR (production telecom_system.cc:2901-2927): scale each data
		// carrier's bit-LLRs by its normalized |H_k|² so the LDPC discounts the deep-null
		// carriers a frequency-selective channel produces. On a flat channel all weights
		// ≈ 1 → no change (TEST-2 byte-identical). Off via MERCURY_SFO_GRID_CSI=0.
		bool csi = (env_i("MERCURY_SFO_GRID_CSI", 1) != 0);
		if(csi)
		{
			double mean_w=0.0; for(int d=0;d<nData;d++) mean_w+=csi_data[d];
			mean_w = (nData>0)?mean_w/(double)nData:1.0; if(mean_w<1e-9) mean_w=1.0;
			for(int d=0;d<nData;d++)
			{
				double w = csi_data[d]/mean_w;
				for(int b=0;b<log2M;b++)
				{
					size_t bi=(size_t)d*log2M+b;
					if(bi>=(size_t)nBits) break;
					float v = clr[bi]*(float)w;
					if(v> 40.0f) v= 40.0f; else if(v<-40.0f) v=-40.0f;
					clr[bi]=v;
				}
			}
		}

		// ---- TURBO-EQ harness loop (RESEARCH_turbo-eq.md §7) -------------------
		// MERCURY_SFO_GRID_TURBO_ITERS (default 1 = OFF, byte-identical to the
		// single-pass coded decode below). When >1: after decoding all Kcw
		// codewords, capture each codeword's a-posteriori LLR (the keystone),
		// re-encode (re-mod), soft_remod to per-data-symbol (x̄,v) virtual pilots,
		// re-estimate the channel data-aided, re-equalize, re-demap, re-decode.
		// Score per iteration; the error-prop guards (§6) keep it monotone-safe.
		int turbo_iters = env_i("MERCURY_SFO_GRID_TURBO_ITERS", 1);
		if(turbo_iters < 1) turbo_iters = 1;
		double turbo_damp = env_f("MERCURY_SFO_GRID_TURBO_DAMP", 0.7);
		if(turbo_damp < 0.0) turbo_damp = 0.0;
		if(turbo_damp > 1.0) turbo_damp = 1.0;
		double turbo_data_conf = env_f("MERCURY_SFO_GRID_TURBO_DATA_CONF", 0.30);
		ofdm.dd_data_conf_thresh = turbo_data_conf;
		// TURBO_EQ_VERDICT.md §5: keep the it=0 (TINTERP) seed H as the low-confidence
		// floor inside data_aided_channel_estimator. Default false (pilots-only floor,
		// byte-identical); MERCURY_SFO_GRID_TURBO_SEED=tinterp turns it on.
		ofdm.dd_seed_floor = turbo_seed_tinterp;

		int    cw_ok = 0, cw_crcfail = 0;
		long   cw_infoerr = 0, cw_infobits = 0;
		long   iter_sum = 0; int iter_min = 1<<30, iter_max = -1;
		// feat/turnaround-eff measurement (turnaround-eff.md §6 TEST-3): the REAL
		// early-term iteration (cl_ldpc::last_early_term_iter) — the harness
		// reports iter_max as the decode_SPA RETURN value (= nIteration_max+1 on a
		// fail/early-term, the FAIL sentinel), so the actual iteration the shared
		// detector quit at is tracked SEPARATELY. Printed on a dedicated line only
		// when at least one codeword tripped => base [SFO-GRID-CODED] lines stay
		// byte-identical when MERCURY_SYND_EARLYTERM is off (default).
		long   et_sum = 0; int et_min = 1<<30, et_max = -1, et_count = 0;
		std::vector<float> cwllr(ldpc.N);
		std::vector<int>   dec(ldpc.N);

		// Per-codeword a-posteriori LLR (all N coded bits), captured for the turbo
		// feedback; and the channel-LLR (clr) the decode consumed, for extrinsic.
		std::vector<double> app(Kcw>0 ? (size_t)Kcw*ldpc.N : 0);
		std::vector<int>    dec_all(Kcw>0 ? (size_t)Kcw*ldpc.K : 0);
		std::vector<double> lext_prev(Kcw>0 ? (size_t)Kcw*ldpc.N : 0, 0.0); // damping state

		// Best-so-far (monotone-or-revert §6.5): keep the iteration with the most
		// decoded codewords; never publish a WORSE result than iteration 0.
		int    best_ok = -1; long best_infoerr = 0; double best_ber = 1.0;
		int    best_it = 0;

		for(int it=0; it<turbo_iters; ++it)
		{
			int    it_ok = 0, it_crcfail = 0;
			long   it_infoerr = 0, it_infobits = 0;
			iter_sum = 0; iter_min = 1<<30; iter_max = -1;
			for(int c=0;c<Kcw;c++)
			{
				for(int i=0;i<ldpc.N;i++) cwllr[i] = clr[(size_t)c*ldpc.N + i];
				// Capture app_llr only when we will use it (it<last iteration).
				double* app_ptr = (turbo_iters>1 && it<turbo_iters-1) ? &app[(size_t)c*ldpc.N] : nullptr;
				int iters = ldpc.decode(cwllr.data(), dec.data(), app_ptr);
				iter_sum += iters;
				if(iters < iter_min) iter_min = iters;
				if(iters > iter_max) iter_max = iters;
				// feat/turnaround-eff: capture the real early-term iteration if the
				// shared detector tripped on this codeword (-1 => no trip).
				if(ldpc.last_early_term_iter >= 0)
				{
					int et = ldpc.last_early_term_iter;
					et_sum += et; et_count++;
					if(et < et_min) et_min = et;
					if(et > et_max) et_max = et;
				}
				int ierr=0;
				for(int i=0;i<ldpc.K;i++){ if(dec[i] != cw_info[c][i]) ierr++; if(Kcw>0) dec_all[(size_t)c*ldpc.K+i]=dec[i]; }
				it_infoerr  += ierr;
				it_infobits += ldpc.K;
				bool capped = (iters > (ldpc.nIteration_max - 1));
				if(ierr == 0 && !capped) it_ok++; else it_crcfail++;
			}
			double it_ber = it_infobits ? (double)it_infoerr/(double)it_infobits : 1.0;

			// iteration 0 result is the single-pass baseline (also the default-OFF path).
			if(it==0){ cw_ok=it_ok; cw_crcfail=it_crcfail; cw_infoerr=it_infoerr; cw_infobits=it_infobits; }

			// Track best (monotone-safe publish): prefer more decoded codewords, then
			// fewer info errors. Guarantees turbo never reports worse than iteration 0.
			if(it_ok > best_ok || (it_ok==best_ok && it_infoerr < best_infoerr))
			{ best_ok=it_ok; best_infoerr=it_infoerr; best_ber=it_ber; best_it=it; }

			if(turbo_iters>1)
				std::cout << "[SFO-GRID-TURBO]   it=" << it << " codewords_decoded=" << it_ok
				          << "/" << Kcw << " post_FEC_info_BER=" << it_ber
				          << " nv=" << ofdm.noise_variance_estimate << std::endl;

			// Early exit: all codewords decoded (the CRC-equivalent success gate).
			if(it_ok==Kcw) break;
			// Last iteration: no feedback to compute.
			if(it>=turbo_iters-1) break;

			// ---- FEEDBACK (§3/§6): re-mod → soft_remod → data-aided re-estimate ----
			// Build per-bit a-priori LLR for the next demod from the decoder extrinsic.
			// L_ext = app − L_ch (the decoder's NEW info), clip ±40, damp λ.
			std::vector<float> fb_bits(nBits, 0.0f);   // a-priori LLRs in clr/deframed order
			for(int c=0;c<Kcw;c++)
			{
				for(int i=0;i<ldpc.N;i++)
				{
					size_t gi = (size_t)c*ldpc.N + i;
					double lext = app[gi] - (double)clr[gi];
					if(lext >  40.0) lext =  40.0; else if(lext < -40.0) lext = -40.0;
					lext = turbo_damp*lext + (1.0-turbo_damp)*lext_prev[gi];
					lext_prev[gi] = lext;
					if(gi < (size_t)nBits) fb_bits[gi] = (float)lext;
				}
			}
			// soft_remod the a-priori LLRs → per-data-symbol (x̄,v). The harness has NO
			// bit-interleaver (tx_bits are codewords mod'd directly, :6398-6410), so the
			// LLR order IS the data-symbol order — no de-interleave needed.
			std::vector<std::complex<double>> xbar(nData);
			std::vector<double> vsoft(nData);
			psk.soft_remod(fb_bits.data(), nBits, xbar.data(), vsoft.data());

			// Data-aided channel re-estimate over the dense lattice, then re-eq/demap.
			ofdm.data_aided_channel_estimator(rx.data(), xbar.data(), vsoft.data());
			// refresh CSI weights from the refined H
			{ int di=0; for(int n=0;n<Ngrid;n++) for(int j=0;j<Nc;j++)
				if((ofdm.ofdm_frame+n*Nc+j)->type==DATA){ std::complex<double> H=(ofdm.estimated_channel+n*Nc+j)->value;
					if(di<nData) csi_data[di]=H.real()*H.real()+H.imag()*H.imag(); di++; } }
			ofdm.channel_equalizer(rx.data(), eq.data());
			ofdm.deframer(eq.data(), deframed.data());
			double cvar2 = ofdm.noise_variance_estimate;
			if(nvfix && cvar2 < variance / NV_COLLAPSE_RATIO_K) cvar2 = variance;   // ratio-gate (a0e22c8)
			if(cvar2 < 1e-9) cvar2 = 1e-9;
			if(pas_on) psk.demod_pas(deframed.data(), nBits, clr.data(), (float)cvar2, pas_log_prior);
			else       psk.demod    (deframed.data(), nBits, clr.data(), (float)cvar2);
			if(csi)
			{
				double mean_w=0.0; for(int d=0;d<nData;d++) mean_w+=csi_data[d];
				mean_w = (nData>0)?mean_w/(double)nData:1.0; if(mean_w<1e-9) mean_w=1.0;
				for(int d=0;d<nData;d++){ double w=csi_data[d]/mean_w;
					for(int b=0;b<log2M;b++){ size_t bi=(size_t)d*log2M+b; if(bi>=(size_t)nBits) break;
						float vv=clr[bi]*(float)w; if(vv>40.0f)vv=40.0f; else if(vv<-40.0f)vv=-40.0f; clr[bi]=vv; } }
			}
		}

		// Publish the BEST iteration (monotone-or-revert §6.5): turbo never reports a
		// worse result than the single-pass iteration 0.
		if(turbo_iters>1 && best_ok >= 0)
		{ cw_ok=best_ok; cw_infoerr=best_infoerr; cw_crcfail=Kcw-best_ok; cw_infobits=(long)Kcw*ldpc.K; }

		double cw_ber = cw_infobits ? (double)cw_infoerr/(double)cw_infobits : 1.0;
		std::cout << "[SFO-GRID-CODED] ===== CODED RESULT (K=" << Kcw
		          << " x " << ldpc.N << "-bit rate-" << ldpc.rate << " codewords) =====" << std::endl;
		std::cout << "[SFO-GRID-CODED]   nv=" << ofdm.noise_variance_estimate
		          << "  EsN0=" << grid_esn0
		          << "  iter_mean=" << (Kcw? (double)iter_sum/(double)Kcw : 0.0)
		          << " iter_min=" << (iter_max<0?0:iter_min) << " iter_max=" << (iter_max<0?0:iter_max)
		          << "  iter_cap=" << ldpc.nIteration_max << std::endl;
		// Turbo summary on a SEPARATE line so the base [SFO-GRID-CODED] lines stay
		// byte-identical when turbo is OFF (default).
		if(turbo_iters>1)
			std::cout << "[SFO-GRID-CODED]   turbo_iters=" << turbo_iters
			          << "  best_it=" << best_it << std::endl;
		std::cout << "[SFO-GRID-CODED]   codewords_decoded=" << cw_ok << "/" << Kcw
		          << "  fail=" << cw_crcfail
		          << "  post_FEC_info_BER=" << cw_ber << std::endl;
		// CFG17 composition self-test snapshot (additive; read by run_cfg17_selftest).
		sfo_grid_last_cw_ok  = cw_ok;
		sfo_grid_last_cw_tot = Kcw;
		std::cout << "[SFO-GRID-CODED]   nv_check: " << (ofdm.noise_variance_estimate > 1e-5 ? "OK (not collapsed)" : "COLLAPSED (<1e-5 -> E1 bug)")
		          << "  iter_cap_check: " << (iter_max < ldpc.nIteration_max ? "OK (no codeword hit cap)" : "CAPPED (BP at 101 -> over-confident LLR)") << std::endl;
		// feat/turnaround-eff (turnaround-eff.md §6 TEST-3): the shared
		// non-convergence detector's REAL early-term iteration, on a DEDICATED tag
		// so the [SFO-GRID-CODED] lines above stay byte-identical when off. Printed
		// only when >=1 codeword tripped (MERCURY_SYND_EARLYTERM set). The decode
		// still RETURNED nIteration_max+1 (FAIL sentinel) for every consumer; this
		// line shows how many wasted iterations were skipped.
		if(et_count > 0)
			std::cout << "[SFO-GRID-EARLYTERM]   tripped=" << et_count << "/" << Kcw
			          << "  et_iter_mean=" << (double)et_sum/(double)et_count
			          << " et_iter_min=" << et_min << " et_iter_max=" << et_max << std::endl;
	}

	// Restore the original grid geometry so the rest of the process is unaffected.
	ofdm.deinit();
	ofdm.start_shift = sav_start_shift;
	ofdm.preamble_configurator.Nsymb = sav_pre_Nsymb;
	ofdm.preamble_configurator.modulation = sav_pre_mod;
	ofdm.preamble_configurator.nIdentical_sections = sav_pre_nIS;
	ofdm.preamble_configurator.boost = sav_pre_boost;
	ofdm.pilot_configurator.boost = sav_pil_boost;
	ofdm.pilot_configurator.modulation = sav_pil_mod;
	ofdm.pilot_configurator.seed = sav_pil_seed;
	ofdm.pilot_configurator.Dx=1; ofdm.pilot_configurator.Dy=3;
	ofdm.pilot_configurator.pilot_density=HIGH_DENSITY;
	ofdm.channel_estimator=LEAST_SQUARE;
	ofdm.channel_estimator_amplitude_restoration=NO;
	ofdm.LS_window_width=0; ofdm.LS_window_hight=0;
	ofdm.init(Nfft, Nc, saved_Nsymb, gi);

	// Restore M + the constellation if the PCS/M64 override changed them, so the
	// rest of the process (and a later -s call) sees the config's modulation.
	if(pcs_m64 && M != saved_M_pcs)
	{
		M = saved_M_pcs;
		psk.set_predefined_constellation(M);
	}
}
void cl_telecom_system::bigblock_livepath_loopback()
{
	auto env_i = [](const char* k, int def){ const char* e=std::getenv(k); return (e&&*e)?atoi(e):def; };
	auto env_d = [](const char* k, double def){ const char* e=std::getenv(k); return (e&&*e)?atof(e):def; };
	if(M == MOD_MFSK){ std::cout << "[BIGBLOCK-LIVE] MFSK unsupported; use -s 15/16." << std::endl; return; }

	std::cout << "[BIGBLOCK-LIVE] ===== LIVE-PATH big-block loopback (production transmit_byte/receive_byte) =====" << std::endl;

	// 1) GATE the production path on. Stock config stays CFG16 (32-QAM/0.875).
	bigblock_framing_enabled = true;

	// 2) size the TX output buffer = preamble + K*frame passband samples, plus lead so
	//    Schmidl-Cox has acquisition room (mirrors the WAV lead silence).
	int interp = frequency_interpolation_rate;
	int lead_n  = (int)(env_d("MERCURY_BIGBLOCK_LIVE_LEAD_MS", 100.0) * sampling_frequency / 1000.0);
	int block_n = bigblock_tx_total_samples();
	if(block_n <= 0){ std::cout << "[BIGBLOCK-LIVE] tx_total_samples=0; abort." << std::endl; bigblock_framing_enabled=false; return; }
	int trail_n = (int)(env_d("MERCURY_BIGBLOCK_LIVE_TRAIL_MS", 50.0) * sampling_frequency / 1000.0);

	std::vector<double> tx_pb(block_n, 0.0);
	// 3) PRODUCTION TX through transmit_byte (which branches to transmit_bigblock).
	//    NO_FILTER_MESSAGE writes raw passband. data/nBytes unused in P1 (known payload).
	//    HEAP-OVERRUN FIX (fact-doc §13): arm the per-call block-emit intent so the CFG16
	//    transmit_byte branch fires; tx_pb is block-sized (block_n).
	int dummy[1] = {0};
	{
		bigblock_emit_scope emit_guard(this, block_n);
		transmit_byte(dummy, 0, tx_pb.data(), NO_FILTER_MESSAGE);
	}
	int K_tx = bigblock_last_tx_K;
	int n_tx = bigblock_last_tx_samples;
	std::cout << "[BIGBLOCK-LIVE] TX via transmit_byte: K=" << K_tx
	          << " block_samples=" << n_tx << " (buf=" << block_n << ")" << std::endl;
	if(K_tx <= 0 || n_tx <= 0){ std::cout << "[BIGBLOCK-LIVE] TX produced no block; abort." << std::endl; bigblock_framing_enabled=false; return; }

	// 4) build the captured RX passband buffer the way the live capture loop would hand
	//    it to receive_byte: [lead silence | block | trail silence], sized to
	//    Nofdm*buffer_Nsymb*interp so receive_bigblock reads the whole window.
	//    Make the capture window at least lead+block+trail.
	int rx_window = lead_n + n_tx + trail_n;
	std::vector<double> rx_pb(rx_window, 0.0);
	for(int i=0;i<n_tx;i++) rx_pb[lead_n + i] = tx_pb[i];

	// optional clean->AWGN to confirm the estimator/LLR path runs (default clean).
	double live_esn0 = env_d("MERCURY_BIGBLOCK_LIVE_ESN0", -999.0);
	if(live_esn0 > -900.0)
	{
		// passband AWGN at the requested Es/N0, using the SAME convention as the
		// production BER path (passband_test_EsN0, telecom_system.cc:480):
		//   sigma = sqrt( 2 * P_sig * f_nyquist / (10^(EsN0/10) * bandwidth) )
		// where P_sig is the mean passband power of the BLOCK (signal region only).
		double sumsq=0.0; for(int i=0;i<n_tx;i++) sumsq += tx_pb[i]*tx_pb[i];
		double P_sig = sumsq / (n_tx>0?n_tx:1);
		double f_nyquist = sampling_frequency / 2.0;
		double sigma = sqrt(2.0 * P_sig * f_nyquist / (pow(10.0, live_esn0/10.0) * bandwidth));
		// AWGN-scale calibration (this-session diag): the shipped validator added
		// `sigma * awgn_value_generator()` per REAL passband sample, but the canonical
		// passband AWGN (apply_with_delay, awgn.cc:68/75; passband_test_EsN0,
		// telecom_system.cc:480/498; sfo_block_test :5618; the -10 path :4484) all add
		// `(sigma/sqrt(2)) * awgn_value_generator()` because awgn_value_generator() is
		// UNIT-variance and the convention is per-real-sample noise variance = sigma^2/2.
		// Bare sigma => 2x noise power => the labeled Es/N0 was 3.01 dB OPTIMISTIC (a TRUE
		// label-3 dB channel). Default is now the canonical convention so the dB label is
		// EXACT; MERCURY_BIGBLOCK_LIVE_AWGN_CONV=1 restores the old bare-sigma for A/B.
		int conv = env_d("MERCURY_BIGBLOCK_LIVE_AWGN_CONV", 0.0) != 0.0 ? 1 : 0;
		double per_sample = (conv==1) ? sigma : (sigma/sqrt(2.0));
		awgn_channel.set_seed(rand());
		double noise_sumsq=0.0;
		for(int i=0;i<rx_window;i++){
			double nz = per_sample * awgn_channel.awgn_value_generator();
			rx_pb[i] += nz; noise_sumsq += nz*nz;
		}
		double P_noise = noise_sumsq/(rx_window>0?rx_window:1);
		// TRUE per-sample Es/N0: passband Es/N0 = P_sig*f_nyquist/(P_noise*bw) under the
		// production convention (P_noise=sigma^2/2). Report what was ACTUALLY injected.
		double true_esn0 = 10.0*log10( P_sig*f_nyquist / (P_noise*bandwidth) );
		std::cout << "[BIGBLOCK-LIVE] AWGN Es/N0(label)=" << live_esn0 << " dB conv=" << conv
		          << " sigma=" << sigma << " per_sample_std=" << per_sample
		          << " P_sig=" << P_sig << " P_noise=" << P_noise
		          << " TRUE_injected_EsN0=" << true_esn0 << " dB bw=" << bandwidth << std::endl;
	}

	// 5) hand the capture buffer to receive_byte through a data_container whose window
	//    matches rx_window. receive_bigblock reads nSamples = Nofdm*buffer_Nsymb*interp;
	//    override buffer_Nsymb so that == rx_window. Save+restore so stock is untouched.
	int Nofdm = data_container.Nofdm;
	int saved_buffer_Nsymb = data_container.buffer_Nsymb;
	if(Nofdm > 0)
	{
		int need_syms = (rx_window + Nofdm*interp - 1) / (Nofdm*interp);
		data_container.buffer_Nsymb = need_syms;
		// re-pad rx_pb to exactly need_syms*Nofdm*interp so the read is in-bounds.
		int exact = need_syms * Nofdm * interp;
		if((int)rx_pb.size() < exact) rx_pb.resize(exact, 0.0);
	}

	// 6) PRODUCTION RX through receive_byte (branches to receive_bigblock).
	std::vector<int> out_bits((size_t)(K_tx+1) * ldpc.K + ldpc.K, 0);
	st_receive_stats rs = receive_byte(rx_pb.data(), out_bits.data());
	data_container.buffer_Nsymb = saved_buffer_Nsymb;

	int K_rx = bigblock_last_rx_K;
	int cw_ok = bigblock_last_rx_cw_ok_count;
	std::cout << "[BIGBLOCK-LIVE] RX via receive_byte: acq_metric=" << rs.coarse_metric
	          << " K=" << K_rx << " codewords_decoded=" << cw_ok << "/" << K_rx
	          << " message_decoded=" << (rs.message_decoded==YES?1:0) << std::endl;

	// 7) explicit byte-correct re-check: every decoded info bit == the known TX info bit.
	// HEAP-OVERRUN FIX (fact-doc §13): receive_bigblock now lands the full K*ldpc.K decode
	// in the dedicated member (out_bits/`out` is only an N_MAX-bounded legacy copy). Compare
	// against the member so the full-block byte-correct gate is unaffected by the bound.
	const std::vector<int>& rx_bits = bigblock_rx_infobits;
	long bit_err=0, bit_tot=0;
	int Kcmp = (K_rx<K_tx?K_rx:K_tx);
	for(int c=0;c<Kcmp && c<(int)bigblock_last_tx_cw_info.size();c++)
		for(int i=0;i<ldpc.K;i++){ bit_tot++;
			size_t bi=(size_t)c*ldpc.K+i;
			int got = (bi < rx_bits.size()) ? rx_bits[bi] : -1;
			if(got != bigblock_last_tx_cw_info[c][i]) bit_err++; }
	(void)out_bits;
	double ber = bit_tot? (double)bit_err/(double)bit_tot : 1.0;

	bool pass = (K_rx==K_tx && K_tx>0 && cw_ok==K_tx && bit_err==0);
	std::cout << "[BIGBLOCK-LIVE] ===== RESULT =====" << std::endl;
	std::cout << "[BIGBLOCK-LIVE]   codewords_decoded=" << cw_ok << "/" << K_tx
	          << "  post_FEC_BER=" << ber << "  infoerr=" << bit_err << "/" << bit_tot << std::endl;
	std::cout << "[BIGBLOCK-LIVE]   VERDICT=" << (pass ? "PASS(live-path 8/8 byte-correct)" : "FAIL") << std::endl;

	bigblock_framing_enabled = false;
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
