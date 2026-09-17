#ifndef GEARSHIFT_QUALITY_REPORT_H_
#define GEARSHIFT_QUALITY_REPORT_H_

#include <cmath>
#include <cstdint>

// Gearshift-v2 advisory forward-channel hint carried in the optional second
// compact-confirm codeword.  It is deliberately small: actual delivered
// application goodput remains authoritative.  SNR is conservatively quantized
// DOWN in the historical 2 dB bins (-5 + 2*q).  Selectivity is quantized UP so
// the reconstructed value never makes a selective channel look flatter than it
// was.  0xF is reserved for unknown selectivity.
static const double GEARSHIFT_QUALITY_SELECTIVITY_STEP = 0.05;
static const int GEARSHIFT_QUALITY_SELECTIVITY_UNKNOWN = 15;
static const int GEARSHIFT_QUALITY_REFRESH_BATCHES_DEFAULT = 4;

inline uint8_t gearshift_quality_pack(double snr_db, double selectivity)
{
    int snr_q = 0;
    if (std::isfinite(snr_db)) {
        snr_q = (int)std::floor((snr_db + 5.0) / 2.0);
        if (snr_q < 0) snr_q = 0;
        if (snr_q > 15) snr_q = 15;
    }

    int sel_q = GEARSHIFT_QUALITY_SELECTIVITY_UNKNOWN;
    if (std::isfinite(selectivity) && selectivity >= 0.0) {
        sel_q = (int)std::ceil(selectivity / GEARSHIFT_QUALITY_SELECTIVITY_STEP - 1e-12);
        if (sel_q < 0) sel_q = 0;
        if (sel_q >= GEARSHIFT_QUALITY_SELECTIVITY_UNKNOWN)
            sel_q = GEARSHIFT_QUALITY_SELECTIVITY_UNKNOWN - 1;
    }
    return (uint8_t)(((unsigned)snr_q << 4) | (unsigned)sel_q);
}

inline double gearshift_quality_unpack_snr(uint8_t report)
{
    return (double)(((report >> 4) & 0x0F) * 2 - 5);
}

inline bool gearshift_quality_unpack_selectivity(uint8_t report, double* out_selectivity)
{
    const int q = report & 0x0F;
    if (q == GEARSHIFT_QUALITY_SELECTIVITY_UNKNOWN) {
        if (out_selectivity) *out_selectivity = -1.0;
        return false;
    }
    if (out_selectivity)
        *out_selectivity = (double)q * GEARSHIFT_QUALITY_SELECTIVITY_STEP;
    return true;
}

inline bool gearshift_quality_report_due(bool have_previous,
                                         uint8_t previous_report,
                                         int previous_bsi,
                                         uint8_t report,
                                         int bsi,
                                         int refresh_batches)
{
    if (!have_previous) return true;
    if (report != previous_report) return true;
    if (refresh_batches < 1) refresh_batches = 1;
    const unsigned age = ((unsigned)(bsi & 0xFF)
                        - (unsigned)(previous_bsi & 0xFF)) & 0xFFu;
    return age >= (unsigned)refresh_batches;
}

#endif // GEARSHIFT_QUALITY_REPORT_H_
