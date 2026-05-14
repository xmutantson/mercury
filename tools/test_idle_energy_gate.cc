/*
 * Idle-scan cadence energy-gate test.
 *
 * Captures the bug from IDLE_SCAN_CADENCE_RESEARCH.md §1/§5: cl_arq_controller::
 * process_main()'s IDLE block runs the expensive FIR_rx_time_sync filter (via
 * measure_signal_only -> passband_to_baseband_decimated) UNCONDITIONALLY every
 * 2 ms loop, even when the channel is silent. The fix (Step 3) is a cheap
 * raw-passband RMS energy gate that mirrors the already-shipping ACK-detector
 * gate (arq_common.cc:4099-4124, Opt 1): the FIR runs only when the gate trips.
 *
 * This test exercises the production gate predicate
 *   idle_energy_gate_open(const double* buf, int n, double gate_rms)
 * declared in include/datalink_layer/idle_energy_gate.h.
 *
 *   - true  => gate OPEN  => FIR runs   (signal present)
 *   - false => gate CLOSED => FIR skipped (silent / below threshold)
 *
 * RED state (HEAD 2022256, before Step 3): the predicate / header does not
 * exist, so this test FAILS TO COMPILE -- which IS the failing test: there is
 * no gate at all on HEAD, the FIR runs on every silent IDLE loop.
 * GREEN state (after Step 3): the header exists and the predicate is the same
 * code process_main() uses to gate the FIR; this test compiles, runs, and
 * every case passes.
 *
 * The complementary INTEGRATION proof (the FIR is actually skipped inside the
 * live process_main() loop) is the IDLE_GATE_TRACE fir_runs counter measured
 * on RPi1 -- see IDLE_SCAN_CADENCE_RESEARCH.md §6 Step 0 / Step 3.
 *
 * Build (from mercury/ directory):
 *   g++ -O2 -std=c++14 -I./include -o tools/test_idle_energy_gate.exe \
 *       tools/test_idle_energy_gate.cc
 */

#include "datalink_layer/idle_energy_gate.h"
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <vector>

static int tests_passed = 0;
static int tests_failed = 0;

#define CHECK(cond, msg) do { \
	if (!(cond)) { \
		printf("  FAIL: %s (line %d)\n", msg, __LINE__); \
		tests_failed++; \
		return; \
	} \
} while(0)

#define PASS(msg) do { \
	printf("  PASS: %s\n", msg); \
	tests_passed++; \
} while(0)

// Representative IDLE buffer size: Nofdm * buffer_Nsymb * frequency_interpolation_rate.
// Exact value is config-dependent; the predicate is size-agnostic (it normalises
// by n), so any realistic length exercises the same code path.
static const int BUF_N = 256 * 139 * 4;

// Fill buf with a sine "MFSK-tone-like" signal of the given peak amplitude.
static void fill_tone(std::vector<double>& buf, double amplitude)
{
	for (int i = 0; i < (int)buf.size(); i++)
		buf[i] = amplitude * std::sin(2.0 * M_PI * 1500.0 * i / 48000.0);
}

// Fill buf with low-level white-ish noise of the given RMS (deterministic LCG).
static void fill_noise(std::vector<double>& buf, double rms)
{
	unsigned int s = 0x12345678u;
	for (int i = 0; i < (int)buf.size(); i++)
	{
		s = s * 1664525u + 1013904223u;
		double u = ((double)s / 4294967296.0) * 2.0 - 1.0;   // [-1, 1) uniform
		buf[i] = u * rms * 1.7320508;                        // uniform RMS = range/sqrt(3)
	}
}

// ---- Test 1: silent buffer => gate CLOSED (FIR must NOT run) ----
void test_silent_buffer_closes_gate()
{
	printf("\n--- Test 1: silent IDLE buffer keeps the FIR gate CLOSED ---\n");

	std::vector<double> buf(BUF_N, 0.0);   // exact silence (rx-mute / no input)

	bool open = idle_energy_gate_open(buf.data(), BUF_N, IDLE_ENERGY_GATE_RMS);
	CHECK(!open, "exact-silence buffer must close the gate (FIR skipped)");

	// Low noise floor well below the gate must also stay closed.
	fill_noise(buf, IDLE_ENERGY_GATE_RMS * 0.25);
	open = idle_energy_gate_open(buf.data(), BUF_N, IDLE_ENERGY_GATE_RMS);
	CHECK(!open, "sub-threshold noise floor must close the gate (FIR skipped)");

	PASS("silent buffer -> gate CLOSED -> FIR not invoked");
}

// ---- Test 2: signal present => gate OPEN (FIR must run) ----
void test_signal_buffer_opens_gate()
{
	printf("\n--- Test 2: signal-present IDLE buffer OPENS the FIR gate ---\n");

	std::vector<double> buf(BUF_N, 0.0);

	// A real MFSK tone sits well above the gate (Opt 1: tone ~0.02 RMS).
	fill_tone(buf, 0.05);
	bool open = idle_energy_gate_open(buf.data(), BUF_N, IDLE_ENERGY_GATE_RMS);
	CHECK(open, "a real on-air tone must open the gate (FIR runs)");

	// Even a weak signal a few dB above the gate must still open it.
	fill_tone(buf, IDLE_ENERGY_GATE_RMS * 4.0 * 1.41421356);  // ~4x gate RMS
	open = idle_energy_gate_open(buf.data(), BUF_N, IDLE_ENERGY_GATE_RMS);
	CHECK(open, "a weak signal above the gate must open the gate (FIR runs)");

	PASS("signal present -> gate OPEN -> FIR invoked");
}

// ---- Test 3: threshold boundary is monotonic and uses RMS, not peak ----
void test_threshold_boundary()
{
	printf("\n--- Test 3: gate decision is monotonic in RMS ---\n");

	std::vector<double> buf(BUF_N, 0.0);

	// Just below the gate: closed.  Just above: open.  No hysteresis expected.
	fill_noise(buf, IDLE_ENERGY_GATE_RMS * 0.9);
	CHECK(!idle_energy_gate_open(buf.data(), BUF_N, IDLE_ENERGY_GATE_RMS),
		"RMS 0.9x gate must be CLOSED");

	fill_noise(buf, IDLE_ENERGY_GATE_RMS * 1.1);
	CHECK(idle_energy_gate_open(buf.data(), BUF_N, IDLE_ENERGY_GATE_RMS),
		"RMS 1.1x gate must be OPEN");

	PASS("gate decision monotonic in RMS, boundary at IDLE_ENERGY_GATE_RMS");
}

int main()
{
	printf("=== Idle-scan Energy Gate Test ===\n");
	printf("IDLE_ENERGY_GATE_RMS = %.6f\n", IDLE_ENERGY_GATE_RMS);

	test_silent_buffer_closes_gate();
	test_signal_buffer_opens_gate();
	test_threshold_boundary();

	printf("\n========================================\n");
	printf("Results: %d passed, %d failed\n", tests_passed, tests_failed);
	printf("========================================\n");
	return tests_failed > 0 ? 1 : 0;
}
