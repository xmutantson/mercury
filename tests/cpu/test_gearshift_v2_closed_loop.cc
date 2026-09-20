#include <cstdio>
#include <fstream>
#include <map>
#include <string>
#include <vector>
#include "datalink_layer/rate_optimizer.h"

static int failures = 0;
static void ok(const char* n, bool v) {
    if (!v) { std::fprintf(stderr, "FAIL %s\n", n); ++failures; }
}

static std::string write_table() {
    const char* p = "/tmp/gs2_closed_loop.json";
    std::ofstream f(p);
    f << R"JSON({
      "schema_version":2,
      "mercury_head":"unknown",
      "config_signature":"gs2-cfg18-wb16-nb14-sack1-bitmap30",
      "table":{
        "13":{
          "wgn16":{"snr_db":16,"eff_bps_mean":1700,"eff_bps_sigma":120,"sack_rate_mean":0.08,"frame_loss_pct":4,"n_runs":5},
          "wgn20":{"snr_db":20,"eff_bps_mean":1800,"eff_bps_sigma":100,"sack_rate_mean":0.03,"frame_loss_pct":2,"n_runs":5},
          "clean":{"snr_db":40,"eff_bps_mean":2000,"eff_bps_sigma":100,"sack_rate_mean":0,"frame_loss_pct":0,"n_runs":5}},
        "14":{
          "wgn16":{"snr_db":16,"eff_bps_mean":1500,"eff_bps_sigma":150,"sack_rate_mean":0.15,"frame_loss_pct":8,"n_runs":5},
          "wgn20":{"snr_db":20,"eff_bps_mean":2700,"eff_bps_sigma":120,"sack_rate_mean":0.04,"frame_loss_pct":3,"n_runs":5},
          "clean":{"snr_db":40,"eff_bps_mean":3000,"eff_bps_sigma":120,"sack_rate_mean":0,"frame_loss_pct":0,"n_runs":5}},
        "16":{
          "wgn16":{"snr_db":16,"eff_bps_mean":300,"eff_bps_sigma":150,"sack_rate_mean":0.70,"frame_loss_pct":55,"n_runs":5},
          "wgn20":{"snr_db":20,"eff_bps_mean":1200,"eff_bps_sigma":250,"sack_rate_mean":0.35,"frame_loss_pct":28,"n_runs":5},
          "clean":{"snr_db":40,"eff_bps_mean":4500,"eff_bps_sigma":180,"sack_rate_mean":0.02,"frame_loss_pct":1,"n_runs":5}}
      }
    })JSON";
    return p;
}

struct Regime {
    double snr;
    double sel;
    std::map<int,double> bps;
};

static st_rate_observation make_obs(int cfg, const Regime& r, int samples) {
    st_rate_observation o;
    o.current_cfg = cfg;
    o.application_bps = r.bps.at(cfg);
    o.transport_bps = o.application_bps;
    o.application_commits = samples;
    o.rate_samples = samples;
    o.outcome_samples = samples;
    o.failed_batch_rate = (o.application_bps < 500.0) ? 0.7 : 0.0;
    o.frame_success_rate = 1.0 - o.failed_batch_rate;
    o.forward_snr_db = r.snr;
    o.forward_snr_age_batches = 0;
    o.forward_selectivity = r.sel;
    o.batch_size = 30;
    o.queue_bytes = 500000;
    o.feasible_configs = {13,14,16};
    o.nominal_bps[13] = 2200;
    o.nominal_bps[14] = 3300;
    o.nominal_bps[16] = 5000;
    return o;
}

int main() {
    cl_rate_optimizer opt;
    ok("load closed-loop table", opt.load(write_table().c_str()));
    opt.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    st_rate_policy p = opt.get_policy();
    p.min_outcome_samples = 1;
    p.cooldown_batches = 1;
    p.probe_cooldown_batches = 12;
    p.confidence_z = 0.5;
    p.direct_switch_margin = 0.02;
    p.probe_mean_margin = 0.04;
    p.default_switch_cost_ms = 200;
    p.min_horizon_ms = 10000;
    p.default_horizon_ms = 60000;
    p.channel_change_snr_db = 3.0;
    opt.set_policy_for_test(p);
    opt.set_switch_cost_ms(200);

    Regime mid{20.0, 0.03, {{13,1800},{14,2700},{16,1200}}};
    Regime clean{40.0, 0.01, {{13,2000},{14,3000},{16,4500}}};
    Regime rough{16.0, 0.12, {{13,1700},{14,1500},{16,300}}};

    int cfg = 13;
    int since_cfg = 0;
    double delivered_bits = 0.0;
    double oracle_bits = 0.0;
    int mid_reach = -1, clean_reach = -1, rough_reach = -1;

    for (int t=0; t<36; ++t) {
        const Regime& r = (t < 12) ? mid : ((t < 24) ? clean : rough);
        const int oracle_cfg = (t < 12) ? 14 : ((t < 24) ? 16 : 13);
        delivered_bits += r.bps.at(cfg);
        oracle_bits += r.bps.at(oracle_cfg);

        const unsigned app_bytes = (unsigned)(r.bps.at(cfg)/8.0 + 0.5);
        const bool failed = r.bps.at(cfg) < 500.0;
        const unsigned acked = failed ? 0u : 30u;
        opt.observe_transaction(cfg, app_bytes, app_bytes, 1000, acked, 30,
                                false, failed, r.snr, 0, r.sel, 30, false);
        ++since_cfg;
        st_rate_observation o = make_obs(cfg, r, since_cfg);
        st_rate_decision d = opt.evaluate_v2(o, 16);
        if (d.actionable) {
            const int from = cfg;
            opt.notify_switch_dispatched(from, d.target_cfg, d.action,
                                         d.fallback_cfg, (unsigned long long)t*1000ULL, false);
            cfg = d.target_cfg;
            since_cfg = 0;
            opt.notify_switch_confirmed((unsigned long long)t*1000ULL + 200ULL);
        }
        opt.notify_cooldown_tick();

        if (t < 12 && cfg == 14 && mid_reach < 0) mid_reach = t;
        if (t >= 12 && t < 24 && cfg == 16 && clean_reach < 0) clean_reach = t-12;
        if (t >= 24 && cfg == 13 && rough_reach < 0) rough_reach = t-24;
    }

    ok("stationary mid-SNR converges to fixed-mode oracle", mid_reach >= 0 && mid_reach <= 4);
    ok("upward channel step reacquires fast config", clean_reach >= 0 && clean_reach <= 5);
    ok("downward channel step exits fragile config", rough_reach >= 0 && rough_reach <= 4);
    const double regret = oracle_bits > 0.0 ? 1.0 - delivered_bits/oracle_bits : 1.0;
    ok("closed-loop cumulative adaptation loss bounded", regret < 0.18);

    std::printf("closed-loop: mid=%d clean=%d rough=%d regret=%.3f\n",
                mid_reach, clean_reach, rough_reach, regret);

    // no-SNR fallback: an outcome-driven regime change must still be detected
    // and must drive the link out of a suddenly failing high mode.  This is the
    // causal fallback for sessions where no fresh peer-reported quality exists.
    cl_rate_optimizer ns;
    ns.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    st_rate_policy np=ns.get_policy();
    np.min_outcome_samples=1; np.cooldown_batches=0; np.confidence_z=0.0;
    np.direct_switch_margin=0.0; np.probe_mean_margin=0.0;
    np.outcome_change_rel_threshold=0.40;
    ns.set_policy_for_test(np); ns.set_switch_cost_ms(1);
    for(int i=0;i<4;++i)
        ns.observe_transaction(16,562,562,1000,30,30,false,false,-99.9,999,-1.0,30,false);
    ns.observe_transaction(16,0,0,1000,0,30,false,true,-99.9,999,-1.0,30,false);
    ns.observe_transaction(16,0,0,1000,0,30,false,true,-99.9,999,-1.0,30,false);
    ok("no-SNR outcome-driven regime change raises volatility", ns.context_volatility_for_test()>0.0);
    st_rate_observation no_snr;
    no_snr.current_cfg=16; no_snr.application_bps=0; no_snr.transport_bps=0;
    no_snr.failed_batch_rate=1.0; no_snr.frame_success_rate=0.0;
    no_snr.outcome_samples=2; no_snr.batch_size=30; no_snr.queue_bytes=500000;
    no_snr.forward_snr_db=-99.9; no_snr.forward_snr_age_batches=999;
    no_snr.reverse_snr_db=-99.9; no_snr.reverse_snr_age_batches=999;
    no_snr.forward_selectivity=-1.0; no_snr.feasible_configs={13,14,16};
    no_snr.nominal_bps[13]=2000; no_snr.nominal_bps[14]=3000; no_snr.nominal_bps[16]=5000;
    st_rate_decision nd=ns.evaluate_v2(no_snr,16);
    ok("no-SNR full failure causally downshifts", nd.action==GEARSHIFT_ACTION_SWITCH && nd.target_cfg<16);

    // No-calibration + no-channel-telemetry closed loop.  The controller must
    // use analytical capacity only as a bounded discovery prior, learn each
    // tried action independently, detect statistically significant goodput
    // regime changes, and avoid upward experimentation immediately after a
    // negative shock.  This is intentionally the hardest causal fallback: no
    // table and no SNR/selectivity oracle are available.
    cl_rate_optimizer nt;
    nt.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    st_rate_policy tp=nt.get_policy();
    tp.min_outcome_samples=1; tp.min_rate_samples=1; tp.cooldown_batches=1;
    tp.probe_cooldown_batches=12; tp.confidence_z=0.5;
    tp.direct_switch_margin=0.02; tp.probe_mean_margin=0.04;
    tp.default_switch_cost_ms=200; tp.outcome_change_rel_threshold=0.40;
    tp.outcome_change_sigma_threshold=2.25; tp.outcome_change_min_samples=4;
    nt.set_policy_for_test(tp); nt.set_switch_cost_ms(200);

    int ncfg=13, nsince=0, nsw=0;
    int nmid=-1, nclean=-1, nrough=-1;
    double ngot=0.0, noracle=0.0;
    for(int t=0;t<48;++t) {
        const Regime& r=(t<16)?mid:((t<32)?clean:rough);
        const int oracle_cfg=(t<16)?14:((t<32)?16:13);
        ngot += r.bps.at(ncfg); noracle += r.bps.at(oracle_cfg);
        const bool failed=r.bps.at(ncfg)<500.0;
        const unsigned bytes=(unsigned)(r.bps.at(ncfg)/8.0+0.5);
        nt.observe_transaction(ncfg,bytes,bytes,1000,failed?0u:30u,30,
                               false,failed,-99.9,999,-1.0,30,false);
        ++nsince;
        st_rate_observation o=make_obs(ncfg,r,nsince);
        o.forward_snr_db=-99.9; o.forward_snr_age_batches=999;
        o.reverse_snr_db=-99.9; o.reverse_snr_age_batches=999;
        o.forward_selectivity=-1.0;
        st_rate_decision d=nt.evaluate_v2(o,16);
        if(d.actionable) {
            nt.notify_switch_dispatched(ncfg,d.target_cfg,d.action,d.fallback_cfg,
                                        (unsigned long long)t*1000ULL,false);
            ncfg=d.target_cfg; nsince=0; ++nsw;
            nt.notify_switch_confirmed((unsigned long long)t*1000ULL+200ULL);
        }
        nt.notify_cooldown_tick();
        if(t<16 && ncfg==14 && nmid<0) nmid=t;
        if(t>=16 && t<32 && ncfg==16 && nclean<0) nclean=t-16;
        if(t>=32 && ncfg==13 && nrough<0) nrough=t-32;
    }
    const double nregret=noracle>0.0?1.0-ngot/noracle:1.0;
    ok("no-telemetry uncalibrated acquisition finds mid oracle",nmid>=0&&nmid<=4);
    ok("no-telemetry positive regime change reopens faster probe",nclean>=0&&nclean<=4);
    ok("no-telemetry negative regime change moves safer without up-probe thrash",nrough>=0&&nrough<=4);
    ok("no-telemetry exploration remains bounded",nsw<=7);
    ok("no-telemetry cumulative adaptation loss bounded",nregret<0.10);
    std::printf("no-telemetry closed-loop: mid=%d clean=%d rough=%d switches=%d regret=%.3f\n",
                nmid,nclean,nrough,nsw,nregret);

    if (failures) return 1;
    std::puts("PASS Gearshift-v2 closed-loop stationary + step-channel oracle tracking");
    return 0;
}
