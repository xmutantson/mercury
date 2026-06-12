/*
 * Mercury: SE-reclaim grid-materializer regression (Stage 2).
 *
 * Part of the --test-se-reclaim suite (run_se_reclaim_tests, test_se_reclaim.cc).
 * Paired with mercury/fact-documents/data-flow-se-reclaim.md §2/§5.
 *
 * Asserts load_configuration(CONFIG_15) materializes the exact grid params and
 * rbc for each selector (verdict arm_a_sweep_GOOD_e14):
 *   pending_grid=GRID_FULL    -> Ngi=54, Dy=3, Nsymb=12, nData=400, rbc~3348.4
 *                                (stock; byte-identical to HEAD)
 *   pending_grid=GRID_RECLAIM -> Ngi=54, Dy=5, Nsymb=10, nData=400, rbc~3826.7
 *                                (RECLAIM-PILOTS, the shippable 1.082x target)
 *   pending_grid=GRID_RECLAIM + MERCURY_SE_RECLAIM_FULL=1
 *                             -> Ngi=18, Dy=5, Nsymb=10, nData=400, rbc~4329.5
 *                                (RECLAIM-FULL, 1.224x, Stage-4 sync-gated)
 *
 * FAIL-BEFORE / PASS-AFTER: before the materializer is threaded (the Stage-1
 * stub), pending_grid=GRID_RECLAIM does NOTHING -> the grid stays stock 54/3/12
 * and rbc stays ~3348 -> the RECLAIM assertions (Dy==5, Nsymb==10, rbc~3826)
 * FAIL. After Stage 2 -> PASS.
 */

#include "physical_layer/telecom_system.h"
#include "physical_layer/physical_defines.h"
#include "common/common_defines.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

// Portable env set/clear (this toolchain is MinGW-w64 on _WIN32 -> _putenv_s;
// matches the guarded pattern at arq_commander.cc:11051-11066). setenv/unsetenv
// are NOT available on _WIN32 here.
static void se_setenv(const char* k, const char* v) {
#if defined(_WIN32)
	_putenv_s(k, v);
#else
	setenv(k, v, 1);
#endif
}
static void se_unsetenv(const char* k) {
#if defined(_WIN32)
	_putenv_s(k, "");
#else
	unsetenv(k);
#endif
}

static int mat_fail = 0;

static void mat_check(const char* name, bool ok, const char* detail)
{
	if(ok) { printf("  [OK]   %s\n", name); }
	else   { printf("  [FAIL] %s: %s\n", name, detail); mat_fail++; }
}

// Load CONFIG_15 with a given pending_grid and read back the materialized grid.
static void load_grid(int grid, int& ngi, int& dy, int& nsymb, int& ndata, double& rbc)
{
	cl_telecom_system ts;
	ts.operation_mode = ARQ_MODE;
	ts.pending_grid = grid;                 // SE-RECLAIM selector input
	ts.load_configuration(CONFIG_15);
	ngi   = (int)llround((double)ts.ofdm.gi * (double)ts.ofdm.Nfft);
	dy    = ts.ofdm.pilot_configurator.Dy;
	nsymb = ts.ofdm.Nsymb;
	ndata = ts.ofdm.pilot_configurator.nData;
	rbc   = ts.rbc;
}

int run_se_reclaim_grid_materializer_tests()
{
	mat_fail = 0;
	int ngi, dy, nsymb, ndata;
	double rbc;
	char buf[160];

	// --- GRID_FULL: stock grid, byte-identical ---
	load_grid(GRID_FULL, ngi, dy, nsymb, ndata, rbc);
	snprintf(buf, sizeof(buf), "Ngi=%d Dy=%d Nsymb=%d nData=%d rbc=%.1f (want 54/3/12/400/~3348)",
		ngi, dy, nsymb, ndata, rbc);
	mat_check("se_grid_materialize_FULL",
		(ngi == 54 && dy == 3 && nsymb == 12 && ndata == 400 &&
		 fabs(rbc - 3348.4) < 5.0), buf);

	// --- GRID_RECLAIM (default = RECLAIM-PILOTS): Ngi=54, Dy=5, Nsymb=10 ---
	se_unsetenv("MERCURY_SE_RECLAIM_FULL");
	se_unsetenv("MERCURY_SE_NGI"); se_unsetenv("MERCURY_SE_DY"); se_unsetenv("MERCURY_SE_NSYMB");
	load_grid(GRID_RECLAIM, ngi, dy, nsymb, ndata, rbc);
	snprintf(buf, sizeof(buf), "Ngi=%d Dy=%d Nsymb=%d nData=%d rbc=%.1f (want 54/5/10/400/~3826.7)",
		ngi, dy, nsymb, ndata, rbc);
	mat_check("se_grid_materialize_RECLAIM_PILOTS",
		(ngi == 54 && dy == 5 && nsymb == 10 && ndata == 400 &&
		 fabs(rbc - 3826.7) < 5.0), buf);

	// --- INV-1 guard: nData must stay 400 (codeword fits the lattice) ---
	mat_check("se_grid_nData_fixed_400_RECLAIM",
		(ndata == 400), "nData != 400 on RECLAIM => codeword overrun risk (INV-1)");

	// --- RECLAIM-FULL via the Stage-4 A/B flag: Ngi=18 ---
	se_setenv("MERCURY_SE_RECLAIM_FULL", "1");
	load_grid(GRID_RECLAIM, ngi, dy, nsymb, ndata, rbc);
	se_unsetenv("MERCURY_SE_RECLAIM_FULL");
	snprintf(buf, sizeof(buf), "Ngi=%d Dy=%d Nsymb=%d nData=%d rbc=%.1f (want 18/5/10/400/~4329.5)",
		ngi, dy, nsymb, ndata, rbc);
	mat_check("se_grid_materialize_RECLAIM_FULL_ab",
		(ngi == 18 && dy == 5 && nsymb == 10 && ndata == 400 &&
		 fabs(rbc - 4329.5) < 5.0), buf);

	// --- Default-off after the A/B: pending_grid=FULL stays stock (byte-identical) ---
	load_grid(GRID_FULL, ngi, dy, nsymb, ndata, rbc);
	mat_check("se_grid_FULL_unaffected_by_ab_env",
		(ngi == 54 && dy == 3 && nsymb == 12 && fabs(rbc - 3348.4) < 5.0),
		"FULL grid drifted after RECLAIM-FULL env A/B");

	return mat_fail;
}
