#ifndef INC_ENGAGEMENT_TELEMETRY_H_
#define INC_ENGAGEMENT_TELEMETRY_H_

// Capstone R1 engagement telemetry (Fable R1 silent-kill defense).
//
// Header-only ON PURPOSE: an inline function's function-local static is ONE shared
// object across every translation unit ([basic.def.odr]), so both the physical layer
// (telecom_system.cc: HARQ, TINTERP) and the datalink layer (arq_common/arq_commander:
// SUPER-ACK, compact-confirm, T1 ACK-slot) tick the SAME counters with no new .cc
// file (build.sh uses an explicit CPP_SOURCES list) and no link dependency.
//
// Each merged reverse-path lever tick()s its counter when it FIRES; print_summary()
// emits one greppable [ENGAGE-SUMMARY] line at teardown (registered via atexit in
// main()). A capstone run renders per lever: ENGAGED (count>0), SILENTLY-DEAD (count==0
// where a fire was expected -> the merge misaligned that lever's reverse-burst geometry,
// the Fable R1 signal), and (for HARQ) ENGAGED-BUT-CAPPED (attempts>0 but the iter-cap
// bounded each decode).

#include <cstdio>

namespace mercury_engage {

enum lever_t {
	SUPERACK_LEAP = 0,     // SUPER-ACK direct leap LANDED (config raised from RSP recommend)
	COMPACT_CONFIRM_OK,    // compact reverse confirm DECODED clean in the SACK window
	COMPACT_CONFIRM_FAIL,  // base ACK pattern detected but content CRC missed (decoupled miss)
	HARQ_ATTEMPT,          // one soft chase-combine decode ATTEMPT (per buffered pair)
	HARQ_SUCCESS,          // a failed frame RESCUED by chase-combining (CRC-clean)
	ACK_SLOT_HIT,          // deterministic TDD ACK slot ENGAGED (T1, MERCURY_ACK_SLOT=1)
	ACK_SLOT_CLIP,         // ACK-slot engaged but the block still timed out (missed slot)
	TINTERP_ACTIVATE,      // CRC-fail-gated TIME_INTERP fade-estimator re-decode FIRED
	LEVER_COUNT
};

inline long* counters()
{
	static long c[LEVER_COUNT] = {0};
	return c;
}

inline void tick(lever_t l)
{
	if(l >= 0 && l < LEVER_COUNT) counters()[l]++;
}

inline long count(lever_t l)
{
	return (l >= 0 && l < LEVER_COUNT) ? counters()[l] : 0;
}

// One greppable per-session line. Called at process teardown (atexit) so a capstone
// run can grep "[ENGAGE-SUMMARY]" for the per-lever fire counts.
inline void print_summary()
{
	const long* c = counters();
	printf("[ENGAGE-SUMMARY] superack_leaps_landed=%ld compact_confirm_ok=%ld "
	       "compact_confirm_fail=%ld harq_attempted=%ld harq_succeeded=%ld "
	       "ack_slot_hit=%ld ack_slot_clip=%ld tinterp_activations=%ld\n",
	       c[SUPERACK_LEAP], c[COMPACT_CONFIRM_OK], c[COMPACT_CONFIRM_FAIL],
	       c[HARQ_ATTEMPT], c[HARQ_SUCCESS], c[ACK_SLOT_HIT], c[ACK_SLOT_CLIP],
	       c[TINTERP_ACTIVATE]);
	fflush(stdout);
}

} // namespace mercury_engage

#endif // INC_ENGAGEMENT_TELEMETRY_H_
