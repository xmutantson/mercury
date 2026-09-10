/*
 * Mercury: A configurable open-source software-defined modem.
 *
 * Accepted SNR is a control-plane measurement, not a precision telemetry
 * value. Keep every control consumer on the modem's centi-dB decision grid.
 */

#ifndef INC_SNR_DECISION_GRID_H_
#define INC_SNR_DECISION_GRID_H_

#include <cmath>

static inline double quantize_snr_decision_db(double snr_db)
{
#ifdef SNR_DECISION_GRID_FAILBEFORE
	return snr_db;
#else
	// Keep the established unmeasured range and non-finite fail-closed values.
	// std::round selects halfway-away-from-zero after the binary64 multiply.
	// Mercury's supported process environment is the normal FE_TONEAREST mode;
	// no claim is made for a caller that changes the active rounding mode.
	if(!std::isfinite(snr_db) || snr_db <= -90.0)
		return snr_db;
	return std::round(snr_db * 100.0) / 100.0;
#endif
}

#endif
