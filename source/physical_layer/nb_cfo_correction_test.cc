// Narrowband CONFIG_0 residual carrier-offset correction regression.
//
// The narrowband CONFIG_0 OFDM receive path historically forced
// freq_offset_measured = 0 in its fine-frequency-sync branch, leaving the
// demodulator blind to any residual carrier offset the coarse stage did not
// remove. A residual in the low tens of Hz still locks the preamble (timing
// acquisition survives) but rotates the constellation across the frame, so the
// per-symbol channel estimate cannot track it and LDPC never converges.
//
// This regression drives a genuine CONFIG_0 narrowband frame through the
// in-process simulation channel (cl_sim_awgn) with an exact, fixed
// single-sideband carrier offset placed well inside the narrowband estimator's
// unambiguous window (bandwidth/(2*Nc) at production geometry, ~20.5 Hz), then
// runs the real receive-byte acquisition and asserts the frame decodes
// byte-faithfully. A control arm at zero offset guards the vehicle and proves
// the correction is a no-op on a clean frame (no regression to healthy links).
//
// Contract:
//   FAIL-BEFORE  correction absent (freq_offset_measured stays 0), or applied
//                with the wrong sign so the shared subtracting re-mix doubles
//                the offset instead of cancelling it: the offset arm does not
//                decode -> the test returns non-zero.
//   PASS-AFTER   correct-sign correction cancels the offset: both the control
//                and the offset arm decode byte-faithfully -> returns zero.
//
// Deterministic (fixed payload, fixed channel seed, fixed injector clamp/seed),
// in-process, no audio devices, no sockets; runs in well under a second. The
// injected magnitude and channel SNR are overridable for tuning via
// MERCURY_NBCFO_TEST_HZ / MERCURY_NBCFO_TEST_SNR (test entry only).

#include "physical_layer/telecom_system.h"
#include "physical_layer/physical_defines.h"
#include "common/common_defines.h"
#include "common/sim_channel.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

extern "C" double test_tx_carrier_offset;

namespace {

// Save/restore one environment variable across an arm so the process env is
// left exactly as found (the sim-channel injector reads its knobs once, at
// construction, so they are set immediately before the channel is built).
struct env_saver {
	const char* key;
	bool had;
	std::string prev;
	explicit env_saver(const char* k) : key(k) {
		const char* v = std::getenv(k);
		had = (v != nullptr);
		if (had) prev = v;
	}
	void set(const char* v) {
#if defined(_WIN32)
		_putenv_s(key, v);
#else
		setenv(key, v, 1);
#endif
	}
	~env_saver() {
#if defined(_WIN32)
		if (had) _putenv_s(key, prev.c_str()); else _putenv_s(key, "");
#else
		if (had) setenv(key, prev.c_str(), 1); else unsetenv(key);
#endif
	}
};

// Deterministic payload byte for slot i (shared by TX build and RX compare).
inline int payload_byte(int i) { return (i * 37 + 11) & 0xFF; }

// Run one genuine narrowband CONFIG_0 frame through the sim channel with an
// exact SSB carrier offset of |cfo_mag_hz| (0 disables the injector). The
// magnitude is pinned by clamping a large injector draw to cfo_mag_hz; the sign
// is fixed by the injector seed. Returns true iff the frame decodes and every
// payload byte matches.
bool nb_cfg0_cfo_roundtrip(double cfo_mag_hz, double snr3k_db, const char* arm)
{
	// Pin the injector's single static residual to an exact magnitude: a large
	// draw sigma clamped tightly forces |resid| == the clamp, and a fixed seed
	// makes the one draw deterministic (see sim_channel.h cl_sim_cfo).
	env_saver e_hz("MERCURY_SIM2_CFO_HZ");
	env_saver e_max("MERCURY_SIM2_CFO_MAX_HZ");
	env_saver e_seed("MERCURY_SIM2_CFO_SEED");
	env_saver e_walk("MERCURY_SIM2_CFO_WALK_HZ");
	env_saver e_sfo("MERCURY_SIM2_SFO_PPM");
	e_walk.set("0");
	e_sfo.set("0");
	if (cfo_mag_hz > 0.0) {
		char buf[64];
		e_hz.set("1000");                 // large sigma -> the clamp always binds
		std::snprintf(buf, sizeof(buf), "%.6f", cfo_mag_hz);
		e_max.set(buf);                   // |resid| == cfo_mag_hz exactly
		e_seed.set("20260829");           // fixed sign/draw
	} else {
		e_hz.set("0");                    // injector off -> clean carrier
		e_max.set("25");
		e_seed.set("20260829");
	}

	cl_telecom_system ts;
	ts.operation_mode = ARQ_MODE;
	ts.narrowband_enabled = YES;          // NB CONFIG_0 (Nc=10) — the corrected path
	ts.load_configuration(CONFIG_0);
	if (ts.current_configuration != CONFIG_0) {
		printf("[TEST-NB-CFO] arm=%s FAIL: load_configuration(CONFIG_0) did not take\n", arm);
		fflush(stdout);
		return false;
	}

	const int interp      = ts.frequency_interpolation_rate;
	const int Nofdm       = ts.data_container.Nofdm;
	const int preamble_n  = ts.data_container.preamble_nSymb;
	const int Nsymb       = ts.data_container.Nsymb;
	const int buffer_N    = ts.data_container.buffer_Nsymb;
	const int sym_samples = Nofdm * interp;

	const int nReal_data  = ts.data_container.nBits - ts.ldpc.P;
	const int frame_bytes = (nReal_data - ts.outer_code_reserved_bits) / 8;
	if (frame_bytes <= 0) {
		printf("[TEST-NB-CFO] arm=%s FAIL: non-positive frame_bytes=%d\n", arm, frame_bytes);
		fflush(stdout);
		return false;
	}

	// TX: deterministic payload into a real CONFIG_0 narrowband frame.
	test_tx_carrier_offset = 0.0;         // CFO comes from the channel, not the modulator
	for (int i = 0; i < frame_bytes; i++)
		ts.data_container.data_byte[i] = payload_byte(i);
	ts.transmit_byte(ts.data_container.data_byte, frame_bytes,
		ts.data_container.passband_data, SINGLE_MESSAGE);

	const int frame_samples = Nofdm * (Nsymb + preamble_n) * interp;
	const int rx_samples    = Nofdm * buffer_N * interp;
	std::vector<double> rx((size_t)rx_samples, 0.0);

	// Place the preamble a few symbols in, comfortably inside the coarse-bounds
	// window, keeping the whole frame within the RX buffer.
	int delay = (preamble_n + 4) * sym_samples;
	if (delay + frame_samples > rx_samples) delay = rx_samples - frame_samples;
	if (delay < 0) delay = 0;
	for (int i = 0; i < frame_samples && (delay + i) < rx_samples; i++)
		rx[(size_t)(delay + i)] = ts.data_container.passband_data[i];

	// Channel: exact whole-buffer SSB carrier offset, then AWGN + the always-on
	// deterministic floor. The injector draws its static residual at
	// construction from the env pinned above; disabling the streaming CFO stops
	// process() from re-applying it a second time.
	const uint64_t SEED = ((uint64_t)20260829u << 1) | 1u;
	cl_sim_awgn ch(SEED, snr3k_db);
	ch.apply_ideal_cfo(rx.data(), rx.size());
	ch.disable_streaming_cfo();
	ch.process(rx.data(), rx.size());

	// RX: real acquisition (ofdm_forced_delay < 0) so the narrowband
	// fine-frequency branch — where the correction lives — actually runs.
	ts.ofdm_forced_delay = -1;
	st_receive_stats st = ts.receive_byte(rx.data(), ts.data_container.hd_decoded_data_byte);

	bool decoded  = (st.message_decoded == YES);
	bool bytes_ok = decoded;
	if (decoded) {
		for (int i = 0; i < frame_bytes; i++) {
			if ((ts.data_container.hd_decoded_data_byte[i] & 0xFF) != payload_byte(i)) {
				bytes_ok = false;
				break;
			}
		}
	}

	printf("[TEST-NB-CFO] arm=%-11s cfo=%5.1fHz snr3k=%4.1f decoded=%d bytes_ok=%d crc=%d delay=%d\n",
		arm, cfo_mag_hz, snr3k_db, (int)decoded, (int)bytes_ok, st.crc, (int)st.delay);
	fflush(stdout);
	return bytes_ok;
}

} // namespace

// Entry point: control arm (0 Hz, guards the vehicle + non-regression) and the
// offset arm (in-window residual). Both must decode byte-faithfully for a pass.
int nb_cfo_correction_test()
{
	double snr = 30.0;
	double cfo = 15.0;                     // exact, inside the ~20.5 Hz NB window
	if (const char* e = std::getenv("MERCURY_NBCFO_TEST_SNR")) { double v = atof(e); if (v > 0.0) snr = v; }
	if (const char* e = std::getenv("MERCURY_NBCFO_TEST_HZ"))  { double v = atof(e); if (v >= 0.0) cfo = v; }

	printf("[TEST-NB-CFO] ===== narrowband CONFIG_0 residual carrier-offset correction "
	       "regression (control 0 Hz + offset %.1f Hz, snr3k=%.1f) =====\n", cfo, snr);
	fflush(stdout);

	bool control = nb_cfg0_cfo_roundtrip(0.0, snr, "control-0Hz");
	bool offset  = nb_cfg0_cfo_roundtrip(cfo, snr, "offset");

	bool ok = control && offset;
	printf("[TEST-NB-CFO] %s: control(0Hz)=%d offset(%.1fHz)=%d\n",
		ok ? "PASS" : "FAIL", (int)control, cfo, (int)offset);
	fflush(stdout);
	return ok ? 0 : 1;
}
