#include <cstdio>
#include "datalink_layer/rate_optimizer.h"

static int failures = 0;
static void check(const char* name, int got, int want)
{
    if (got != want) {
        std::fprintf(stderr, "FAIL %s: got cfg%d want cfg%d\n", name, got, want);
        ++failures;
    }
}
static int stable(cl_rate_optimizer& o, int cfg, double eff, double sack, double partial)
{
    o.reset_session_state();
    int t = cfg;
    for (int i = 0; i < 4; ++i)
        t = o.evaluate(cfg, eff, sack, 20, 16, false, partial);
    return t;
}

int main()
{
    cl_rate_optimizer o;
    if (!o.load("effective_rate_table.json")) {
        std::fprintf(stderr, "FAIL could not load repository effective_rate_table.json\n");
        return 1;
    }

    // Exact calibration-cell observations: the decision should reproduce the
    // goodput ordering represented by the repository's current Q table after
    // switching cost + hysteresis, without a hard-coded adjacent-rung walk.
    check("wgn20 cfg13 -> cfg14", stable(o, 13, 1985.9, 0.0467, 0.0244), 14);
    check("wgn20 cfg14 stays",    stable(o, 14, 2662.0, 0.0520, 0.0333), 14);
    check("wgn20 cfg15 stays",    stable(o, 15, 2489.2, 0.4200, 0.3958), 15);
    check("wgn20 cfg16 -> cfg14", stable(o, 16, 797.0,  0.1667, 0.2333), 14);
    check("wgn16 cfg15 -> cfg14", stable(o, 15, 1925.0, 0.2767, 0.7333), 14);
    check("wgn18 cfg14 -> cfg15", stable(o, 14, 2616.8, 0.0000, 0.0000), 15);
    check("wgn24 cfg16 stays",    stable(o, 16, 3359.4, 0.2191, 0.1677), 16);

    if (failures) return 1;
    std::puts("PASS current Q-table anchors: direct goodput-optimal decisions");
    return 0;
}
