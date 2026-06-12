#ifndef INC_SE_RECLAIM_GATE_H_
#define INC_SE_RECLAIM_GATE_H_

#include "common/common_defines.h"   // se_grid_t (GRID_FULL/GRID_RECLAIM)

// =============================================================================
// SE-RECLAIM forward-link gate (data-flow-se-reclaim.md §3, gating_design).
//
// The ENTIRE safety case for the SE-reclaim feature, because the verdict knee is
// SHARP: a moderate fade collapses the reclaim grid to ~0 decode, so a mislabel
// loses traffic on a life-critical modem (CLAUDE.md: robustness over speed).
//
// DESIGN (conservative, default-safe, fail-safe — gating_design):
//   - Default + startup state = GRID_FULL (the safe Ngi=54/Dy=3/Nsymb=12 grid).
//   - Promote FULL->RECLAIM only after CONFIRM_N CONSECUTIVE clean-qualifying
//     FORWARD measurements (slow up).
//   - Demote RECLAIM->FULL on ANY single qualifying drop (instant down).
//   - The demote MUST NOT depend on a successful decode: a no-progress/timeout
//     (silent-link) tick demotes too, so a mislabeled fade that kills decode
//     still falls back to FULL.
//   - Clean-qualifying predicate (deep margin BELOW the 0.15 estimator knee):
//        forward_selectivity in [0, SEL_RECLAIM_MAX]  (default 0.05)
//        AND forward_snr_db   >= SNR_RECLAIM_MIN_DB
//        AND forward_fer      == 0
//     A selectivity SENTINEL (<0, no measurement) is NOT clean (fail-safe).
//
// This is a DEDICATED gate, NOT the rate_optimizer label (which is AWGN-only,
// noisy, 50-batch-lagged, reverse-direction — verified unsuitable, audit §
// optimizer-gate). The optimizer keeps RUNG selection; this gate owns the GRID.
//
// The gate is a PURE state machine over scalar inputs (no DSP, no ARQ coupling)
// so it is fully unit-testable (--test-se-reclaim-gate). Its INPUTS are the
// RSP-measured FORWARD selectivity/SNR/FER (the CMD must feed the RSP-reported
// forward values, NOT its own reverse-link selectivity — audit INV-4). The
// transport of those values on the ACK suffix + the optimizer election are
// HELD behind a default-OFF feature flag until the bench A/B (Stage 6).
// =============================================================================
class cl_se_reclaim_gate
{
public:
	// Tunables (gating_design; bench-calibrated EsN0<->SNR3k before trusting).
	// Deep margin under the 0.15 estimator 'selective' knee (telecom_system.cc:7522).
	double SEL_RECLAIM_MAX   = 0.05;   // forward selectivity must be <= this
	double SNR_RECLAIM_MIN_DB = 12.0;  // forward SNR floor (clean/good front)
	int    CONFIRM_N         = 10;     // consecutive clean ticks to promote (slow up)

	cl_se_reclaim_gate() { reset(); }

	void reset()
	{
		grid_ = GRID_FULL;
		clean_streak_ = 0;
	}

	// Is this forward measurement clean-qualifying (deep-margin predicate)?
	bool is_clean(double fwd_selectivity, double fwd_snr_db, int fwd_fer) const
	{
		if(fwd_selectivity < 0.0) return false;            // sentinel => not clean
		if(fwd_selectivity > SEL_RECLAIM_MAX) return false;
		if(fwd_snr_db < SNR_RECLAIM_MIN_DB) return false;
		if(fwd_fer != 0) return false;
		return true;
	}

	// Feed one FORWARD measurement; returns the (possibly updated) elected grid.
	// Asymmetric: CONFIRM_N clean ticks promote; ONE non-clean tick demotes.
	se_grid_t update(double fwd_selectivity, double fwd_snr_db, int fwd_fer)
	{
		if(is_clean(fwd_selectivity, fwd_snr_db, fwd_fer))
		{
			if(clean_streak_ < CONFIRM_N) clean_streak_++;
			if(clean_streak_ >= CONFIRM_N) grid_ = GRID_RECLAIM;
		}
		else
		{
			// Instant demote — any sign of trouble reverts to the safe grid.
			clean_streak_ = 0;
			grid_ = GRID_FULL;
		}
		return (se_grid_t)grid_;
	}

	// Decode-INDEPENDENT demote: a no-progress / silent-link tick (no forward
	// measurement returned) must still fall back to FULL. A mislabeled fade kills
	// decode, so we cannot wait for a 'clean decode' to demote.
	se_grid_t no_progress_tick()
	{
		clean_streak_ = 0;
		grid_ = GRID_FULL;
		return (se_grid_t)grid_;
	}

	se_grid_t grid() const { return (se_grid_t)grid_; }
	int clean_streak() const { return clean_streak_; }

private:
	int grid_;          // current elected grid (GRID_FULL/GRID_RECLAIM)
	int clean_streak_;  // consecutive clean-qualifying forward measurements
};

#endif // INC_SE_RECLAIM_GATE_H_
