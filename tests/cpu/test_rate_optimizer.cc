#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include "datalink_layer/rate_optimizer.h"

static int failures = 0;
static void expect_eq(const char* name, int got, int want)
{
    if (got != want) {
        std::fprintf(stderr, "FAIL %s: got %d want %d\n", name, got, want);
        ++failures;
    }
}
static void expect_true(const char* name, bool v)
{
    if (!v) {
        std::fprintf(stderr, "FAIL %s\n", name);
        ++failures;
    }
}

static std::string write_table(const char* basename, const char* body)
{
    std::string path = std::string("/tmp/") + basename;
    std::ofstream f(path.c_str(), std::ios::binary);
    f << body;
    f.close();
    return path;
}

static int stable_eval(cl_rate_optimizer& opt, int cfg, double eff,
                       double sack, double partial, int ceiling=16)
{
    int out = cfg;
    for (int i = 0; i < 4; ++i)
        out = opt.evaluate(cfg, eff, sack, 20, ceiling, false, partial);
    return out;
}

int main()
{
    // Same SACK incidence + same current goodput, radically different loss
    // severity. The selector must use frame-loss severity to distinguish them.
    const char* severity_json = R"JSON({
      "table": {
        "13": {
          "cleanish": {"eff_bps_mean":2000,"sack_rate_mean":0.5,"frame_loss_pct":3,"n_runs":5},
          "rough":    {"eff_bps_mean":2000,"sack_rate_mean":0.5,"frame_loss_pct":40,"n_runs":5}
        },
        "14": {
          "cleanish": {"eff_bps_mean":3000,"sack_rate_mean":0.2,"frame_loss_pct":2,"n_runs":5},
          "rough":    {"eff_bps_mean":1500,"sack_rate_mean":0.8,"frame_loss_pct":45,"n_runs":5}
        }
      }
    })JSON";
    std::string severity_path = write_table("mercury_rateopt_severity.json", severity_json);
    cl_rate_optimizer opt;
    expect_true("severity table load", opt.load(severity_path.c_str()));
    expect_true("partial-loss bound loaded", opt.max_calibrated_partial_loss() > 0.44);

    int t = stable_eval(opt, 13, 2000.0, 0.5, 0.03);
    expect_eq("low-severity partials may climb", t, 14);

    // Recommendation alone must not arm cooldown. If dispatch is blocked by a
    // live cap, the next evaluation must remain free to choose again.
    t = opt.evaluate(13, 2000.0, 0.5, 20, 16, false, 0.03);
    expect_eq("recommendation has no phantom cooldown", t, 14);
    opt.force_cooldown(5);
    t = opt.evaluate(13, 2000.0, 0.5, 20, 16, false, 0.03);
    expect_eq("actual switch cooldown suppresses evaluate", t, 13);

    opt.reset_session_state();
    t = stable_eval(opt, 13, 2000.0, 0.5, 0.40);
    expect_eq("high-severity partials do not climb into bad mode", t, 13);

    // Switching economics must be based on one-time wall-time cost over the
    // expected hold horizon. A short/fast batch makes a switch expensive;
    // a long batch makes the same sustained gain pay back.
    const char* cost_json = R"JSON({
      "table": {
        "13": {"clean":{"eff_bps_mean":2000,"sack_rate_mean":0,"frame_loss_pct":0,"n_runs":5}},
        "14": {"clean":{"eff_bps_mean":2400,"sack_rate_mean":0,"frame_loss_pct":0,"n_runs":5}}
      }
    })JSON";
    std::string cost_path = write_table("mercury_rateopt_cost.json", cost_json);
    cl_rate_optimizer short_horizon;
    expect_true("cost table load short", short_horizon.load(cost_path.c_str()));
    short_horizon.set_wire_ms_per_batch(1000.0);
    t = stable_eval(short_horizon, 13, 2000.0, 0.0, 0.0, 14);
    expect_eq("switch not worth short hold-time horizon", t, 13);

    cl_rate_optimizer long_horizon;
    expect_true("cost table load long", long_horizon.load(cost_path.c_str()));
    long_horizon.set_wire_ms_per_batch(5000.0);
    t = stable_eval(long_horizon, 13, 2000.0, 0.0, 0.0, 14);
    expect_eq("switch worth long hold-time horizon", t, 14);

    // Legacy calibration tables lacking frame_loss_pct remain usable and do
    // not impose a zero-loss hard gate.
    const char* legacy_json = R"JSON({
      "table": {
        "13": {"clean":{"eff_bps_mean":2000,"sack_rate_mean":0.1,"n_runs":3}},
        "14": {"clean":{"eff_bps_mean":2800,"sack_rate_mean":0.1,"n_runs":3}}
      }
    })JSON";
    std::string legacy_path = write_table("mercury_rateopt_legacy.json", legacy_json);
    cl_rate_optimizer legacy;
    expect_true("legacy table load", legacy.load(legacy_path.c_str()));
    expect_true("legacy table has no partial-loss hard bound",
                legacy.max_calibrated_partial_loss() < 0.0);
    t = stable_eval(legacy, 13, 2000.0, 0.1, 0.25, 14);
    expect_eq("legacy table still optimizes", t, 14);

    if (failures) return 1;
    std::puts("PASS rate_optimizer: severity, economics, cooldown, legacy-table compatibility");
    return 0;
}
