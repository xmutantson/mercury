#include "datalink_layer/gearshift_quality_report.h"
#include <cmath>
#include <cstdio>

static int failures = 0;
static void ok(const char* name, bool value) {
    if (!value) { std::fprintf(stderr, "FAIL %s\n", name); ++failures; }
}

int main()
{
    const uint8_t r = gearshift_quality_pack(20.9, 0.12);
    ok("SNR quantizes down", std::fabs(gearshift_quality_unpack_snr(r) - 19.0) < 1e-9);
    double sel = -1.0;
    ok("selectivity present", gearshift_quality_unpack_selectivity(r, &sel));
    ok("selectivity quantizes conservatively up", std::fabs(sel - 0.15) < 1e-9);

    const uint8_t unknown = gearshift_quality_pack(18.0, NAN);
    sel = 123.0;
    ok("unknown selectivity reserved", !gearshift_quality_unpack_selectivity(unknown, &sel) && sel < 0.0);

    const uint8_t saturated = gearshift_quality_pack(40.0, 2.0);
    ok("SNR saturates safely", std::fabs(gearshift_quality_unpack_snr(saturated) - 25.0) < 1e-9);
    ok("selectivity saturates below unknown code", gearshift_quality_unpack_selectivity(saturated, &sel)
       && std::fabs(sel - 0.70) < 1e-9);

    ok("first report is due", gearshift_quality_report_due(false, 0, -1, r, 10, 4));
    ok("unchanged report suppressed before refresh", !gearshift_quality_report_due(true, r, 10, r, 13, 4));
    ok("unchanged report refreshes at age", gearshift_quality_report_due(true, r, 10, r, 14, 4));
    const uint8_t changed = gearshift_quality_pack(22.0, 0.12);
    ok("quantized change sends immediately", gearshift_quality_report_due(true, r, 10, changed, 11, 4));
    ok("BSI wrap age works", gearshift_quality_report_due(true, r, 254, r, 2, 4));

    if (failures) return 1;
    std::puts("PASS gearshift compact quality codec + rate limiting");
    return 0;
}
