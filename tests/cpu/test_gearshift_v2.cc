#include <cstdio>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>
#include "datalink_layer/rate_optimizer.h"
#include "common/common_defines.h"

static int failures = 0;
static void eqi(const char* n, int g, int w) {
    if (g != w) { std::fprintf(stderr, "FAIL %s got %d want %d\n", n, g, w); ++failures; }
}
static void ok(const char* n, bool v) {
    if (!v) { std::fprintf(stderr, "FAIL %s\n", n); ++failures; }
}

static std::string table() {
    const char* p = "/tmp/gs2.json";
    std::ofstream f(p);
    f << R"JSON({"calibration_setup":{"compress":false},"table":{
      "13":{"clean":{"snr_db":40,"eff_bps_mean":2000,"eff_bps_sigma":200,"sack_rate_mean":0,"frame_loss_pct":0,"n_runs":5},
            "wgn20":{"snr_db":20,"eff_bps_mean":1800,"eff_bps_sigma":250,"sack_rate_mean":0.1,"frame_loss_pct":5,"n_runs":5}},
      "14":{"clean":{"snr_db":40,"eff_bps_mean":3000,"eff_bps_sigma":900,"sack_rate_mean":0,"frame_loss_pct":0,"n_runs":5},
            "wgn20":{"snr_db":20,"eff_bps_mean":2500,"eff_bps_sigma":900,"sack_rate_mean":0.1,"frame_loss_pct":5,"n_runs":5}},
      "16":{"clean":{"snr_db":40,"eff_bps_mean":4200,"eff_bps_sigma":300,"sack_rate_mean":0,"frame_loss_pct":0,"n_runs":5},
            "wgn20":{"snr_db":20,"eff_bps_mean":900,"eff_bps_sigma":300,"sack_rate_mean":0.4,"frame_loss_pct":35,"n_runs":5}}
    }})JSON";
    return p;
}

static st_rate_observation obs(int cfg, double app, int commits, double fail=0.0) {
    st_rate_observation o;
    o.current_cfg = cfg;
    o.application_bps = app;
    o.transport_bps = app;
    o.application_commits = commits;
    o.rate_samples = commits;
    o.outcome_samples = 6;
    o.failed_batch_rate = fail;
    o.frame_success_rate = 1.0 - fail;
    o.forward_snr_db = 20;
    o.forward_snr_age_batches = 0;
    o.forward_selectivity = 0.02;
    o.batch_size = 30;
    o.queue_bytes = 200000;
    o.feasible_configs = {13, 14, 16};
    o.nominal_bps[13] = 2200;
    o.nominal_bps[14] = 3200;
    o.nominal_bps[16] = 4800;
    return o;
}

static st_rate_policy permissive(cl_rate_optimizer& r) {
    st_rate_policy p = r.get_policy();
    p.confidence_z = 0.0;
    p.direct_switch_margin = 0.0;
    p.probe_mean_margin = 0.0;
    p.default_switch_cost_ms = 0;
    p.min_outcome_samples = 1;
    p.cooldown_batches = 0;
    return p;
}

int main() {
    cl_rate_optimizer r;
    ok("load", r.load(table().c_str()));
    r.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    st_rate_policy p = permissive(r);
    r.set_policy_for_test(p);
    r.set_switch_cost_ms(1);

    // Current cfg above admission ceiling must be able to leave downward.
    st_rate_decision d = r.evaluate_v2(obs(16, 900, 5, 0.4), 14);
    eqi("ceiling departure target", d.target_cfg, 14);
    ok("ceiling departure action", d.action == GEARSHIFT_ACTION_SWITCH);

    // Strong live evidence on cfg13 must update cfg13 only and overrule stale ordering.
    r.reset_session_state();
    for (int i=0; i<8; ++i)
        r.observe_transaction(13, 440, 440, 1000, 30, 30, false, false, 20, 0, 0.02, 30, false);
    st_rate_observation o = obs(13, 3520, 8, 0);
    d = r.evaluate_v2(o, 16);
    ok("stale prior overridden", d.target_cfg == 13 || d.action == GEARSHIFT_ACTION_HOLD);

    // Full failures are active evidence and can drive a direct downshift.
    r.reset_session_state();
    for (int i=0; i<6; ++i)
        r.observe_transaction(16, 0, 0, 1000, 0, 30, false, true, 20, 0, 0.02, 30, false);
    o = obs(16, 0, 0, 1.0); o.frame_success_rate = 0;
    d = r.evaluate_v2(o, 16);
    ok("full failure downshift", d.target_cfg < 16 && d.action == GEARSHIFT_ACTION_SWITCH);

    // Shadow computes decisions but must never be actionable.
    r.reset_session_state(); r.set_mode_for_test(GEARSHIFT_V2_SHADOW);
    o = obs(13, 1800, 4, 0); d = r.evaluate_v2(o, 16);
    ok("shadow not actionable", !d.actionable);

    // V2 remains useful with no calibration: the modem-defined action set and
    // analytical capacity hints make an uncalibrated mode probeable/switchable.
    cl_rate_optimizer u;
    u.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    p = permissive(u); u.set_policy_for_test(p); u.set_switch_cost_ms(1);
    st_rate_observation no_table = obs(13, 1500, 4, 0);
    no_table.feasible_configs = {13, 14};
    no_table.nominal_bps.clear();
    no_table.nominal_bps[13] = 1800;
    no_table.nominal_bps[14] = 3600;
    d = u.evaluate_v2(no_table, 14);
    ok("uncalibrated controller acts", d.action == GEARSHIFT_ACTION_SWITCH || d.action == GEARSHIFT_ACTION_PROBE);
    eqi("uncalibrated missing-table mode visible", d.target_cfg, 14);

    // Calibration is intentionally collected with modem compression OFF so it
    // describes channel/PHY transport capacity.  Once production has measured
    // a real application/transport ratio, convert untried raw priors into the
    // same application-goodput units before comparing them with live app rate.
    const char* comp_path="/tmp/gs2_compression_units.json";
    { std::ofstream f(comp_path); f << R"JSON({
      "calibration_setup":{"compress":false},
      "table":{
        "13":{"wgn20":{"snr_db":20,"eff_bps_mean":1800,"eff_bps_sigma":100,"n_runs":5}},
        "14":{"wgn20":{"snr_db":20,"eff_bps_mean":2700,"eff_bps_sigma":120,"n_runs":5}}
      }})JSON"; }
    cl_rate_optimizer cu; ok("compression-unit table load",cu.load(comp_path));
    cu.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    st_rate_observation cobs=obs(13,3600,4,0);
    cobs.transport_bps=1800; cobs.compression_enabled=true;
    cobs.application_transport_gain=2.0; cobs.application_transport_gain_valid=true;
    st_rate_prediction c14=cu.predict_for_test(14,cobs);
    ok("raw calibration prior converted to application units",
       c14.valid && c14.mean_bps>5000.0 && c14.source.find("app-unit-converted")!=std::string::npos);

    // Batch-size context changes the authority of old live evidence rather than
    // letting a 30-frame history masquerade as equally relevant to 5-frame traffic.
    cl_rate_optimizer b;
    ok("batch table load", b.load(table().c_str()));
    b.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    for (int i=0; i<8; ++i)
        b.observe_transaction(14, 350, 350, 1000, 30, 30, false, false, 20, 0, 0.02, 30, false);
    st_rate_observation same = obs(13, 1800, 4, 0); same.batch_size = 30;
    st_rate_observation changed = same; changed.batch_size = 5;
    st_rate_prediction ps = b.predict_for_test(14, same);
    st_rate_prediction pc = b.predict_for_test(14, changed);
    ok("batch mismatch downweights live evidence", pc.live_weight < ps.live_weight);
    ok("batch mismatch raises uncertainty", pc.sigma_bps >= ps.sigma_bps);

    // Calibration cells carry the batch geometry they were measured with. A
    // candidate that will start at a different batch size keeps the prior as
    // information but must lower its authority/increase uncertainty.
    const char* batchcal_path="/tmp/gs2_batch_context.json";
    { std::ofstream f(batchcal_path); f << R"JSON({"calibration_setup":{"compress":false},"table":{"14":{
      "wgn20":{"snr_db":20,"batch_size_mean":30,"eff_bps_mean":3000,"eff_bps_sigma":100,"n_runs":8}
    }}})JSON"; }
    cl_rate_optimizer bc; ok("batch-context calibration load",bc.load(batchcal_path));
    st_rate_observation bc_same=obs(13,1800,2,0); bc_same.candidate_batch_size[14]=30;
    st_rate_observation bc_diff=bc_same; bc_diff.candidate_batch_size[14]=5;
    st_rate_prediction bps=bc.predict_for_test(14,bc_same);
    st_rate_prediction bpd=bc.predict_for_test(14,bc_diff);
    ok("calibration batch mismatch lowers prior authority", bpd.prior_weight < bps.prior_weight);
    ok("calibration batch mismatch widens uncertainty", bpd.sigma_bps > bps.sigma_bps);

    // Freshness is measured in channel time, not merely batch count. Three
    // ten-second transactions must age evidence much more than three one-second
    // transactions even though both paths advanced the same number of batches.
    cl_rate_optimizer age_fast, age_slow;
    age_fast.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    age_slow.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    st_rate_policy agep = age_fast.get_policy();
    agep.live_stale_half_life_ms = 5000.0;
    agep.live_stale_half_life_batches = 1000.0; // prove wall time owns freshness
    age_fast.set_policy_for_test(agep); age_slow.set_policy_for_test(agep);
    for(int i=0;i<6;++i) {
        age_fast.observe_transaction(14,350,350,1000,30,30,false,false,-99.9,99,-1.0,30,false);
        age_slow.observe_transaction(14,350,350,1000,30,30,false,false,-99.9,99,-1.0,30,false);
    }
    for(int i=0;i<3;++i) {
        age_fast.observe_transaction(13,0,0,1000,30,30,false,false,-99.9,99,-1.0,30,false);
        age_slow.observe_transaction(13,0,0,10000,30,30,false,false,-99.9,99,-1.0,30,false);
    }
    st_rate_observation ageobs=obs(13,1800,0,0); ageobs.forward_snr_db=-99.9; ageobs.forward_snr_age_batches=99; ageobs.forward_selectivity=-1.0;
    st_rate_prediction af=age_fast.predict_for_test(14,ageobs);
    st_rate_prediction as=age_slow.predict_for_test(14,ageobs);
    ok("live freshness decays by elapsed channel time", as.live_weight < af.live_weight*0.10);
    ok("time age is reported", as.age_ms > af.age_ms*5.0);

    // A detected channel change immediately weakens stale per-config evidence.
    st_rate_prediction before_change = b.predict_for_test(14, same);
    b.observe_transaction(13, 225, 225, 1000, 30, 30, false, false, 28, 0, 0.02, 30, false);
    st_rate_observation changed_channel = same; changed_channel.forward_snr_db = 28;
    st_rate_prediction after_change = b.predict_for_test(14, changed_channel);
    ok("regime change downweights stale live evidence", after_change.live_weight < before_change.live_weight);

    // Cold ACTIVE acquisition is deliberately faster than the ordinary four-
    // outcome steady-state gate. One trustworthy timed observation plus a long
    // queue is enough to try the best faster feasible action directly as a
    // bounded probe; the no-table analytical prior never becomes blind acceptance.
    cl_rate_optimizer cold;
    cold.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    st_rate_policy coldp = cold.get_policy();
    coldp.cooldown_batches = 0;
    cold.set_policy_for_test(coldp);
    cold.set_switch_cost_ms(100);
    st_rate_observation coldobs;
    coldobs.current_cfg=CONFIG_14; coldobs.application_bps=2800; coldobs.transport_bps=2800;
    coldobs.application_commits=1; coldobs.rate_samples=1; coldobs.outcome_samples=1;
    coldobs.failed_batch_rate=0.0; coldobs.frame_success_rate=1.0;
    coldobs.forward_snr_db=20; coldobs.forward_snr_age_batches=0;
    coldobs.forward_selectivity=0.02; coldobs.forward_selectivity_age_batches=0;
    coldobs.batch_size=30; coldobs.queue_bytes=500000; coldobs.remaining_work_known=true;
    coldobs.feasible_configs={CONFIG_14,CONFIG_15,CONFIG_16};
    coldobs.nominal_bps[CONFIG_14]=3000; coldobs.nominal_bps[CONFIG_15]=5000;
    coldobs.nominal_bps[CONFIG_16]=9000;
    coldobs.tx_airtime_ms[CONFIG_15]=3000; coldobs.tx_airtime_ms[CONFIG_16]=5572;
    coldobs.feedback_budget_ms=1000;
    d=cold.evaluate_v2(coldobs,CONFIG_16);
    ok("cold start probes before four outcomes", d.action==GEARSHIFT_ACTION_PROBE &&
       d.target_cfg==CONFIG_16 && d.reason=="cold-start-bounded-probe");
    st_rate_observation coldshort=coldobs; coldshort.queue_bytes=1000;
    d=cold.evaluate_v2(coldshort,CONFIG_16);
    ok("cold start still respects short-transfer economics",
       d.action!=GEARSHIFT_ACTION_PROBE && d.action!=GEARSHIFT_ACTION_SWITCH);

    // SHADOW is observational compatibility mode: the ACTIVE-only acquisition
    // bypass must not silently change its pre-existing evidence gate.
    cl_rate_optimizer coldshadow; coldshadow.set_mode_for_test(GEARSHIFT_V2_SHADOW);
    d=coldshadow.evaluate_v2(coldobs,CONFIG_16);
    ok("shadow cold start keeps steady-state evidence gate",
       d.action==GEARSHIFT_ACTION_ABSTAIN && d.reason=="learning-outcomes");

    // A trustworthy in-support calibration prior is direct empirical evidence.
    // With the same one-outcome cold start it may clear the direct-switch bound,
    // while the no-table controller above still has to discover via a probe.
    const char* fastcal_path="/tmp/gs2_fast_cal.json";
    { std::ofstream f(fastcal_path); f << R"JSON({
      "mercury_head":"unknown",
      "config_signature":"gs2-cfg18-wb16-nb14-sack1-bitmap30",
      "calibration_setup":{"compress":false},
      "table":{
        "14":{"wgn20":{"snr_db":20,"selectivity_mean":0.02,"eff_bps_mean":2800,"eff_bps_sigma":60,"n_runs":10}},
        "16":{"wgn20":{"snr_db":20,"selectivity_mean":0.02,"eff_bps_mean":5200,"eff_bps_sigma":70,"n_runs":10}}
      }})JSON"; }
    cl_rate_optimizer calfast; ok("fast calibration load",calfast.load(fastcal_path));
    calfast.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    coldp=calfast.get_policy(); coldp.cooldown_batches=0; calfast.set_policy_for_test(coldp);
    calfast.set_switch_cost_ms(100);
    d=calfast.evaluate_v2(coldobs,CONFIG_16);
    ok("calibrated fast action can direct-switch on cold start",
       d.action==GEARSHIFT_ACTION_SWITCH && d.target_cfg==CONFIG_16 &&
       d.reason=="cold-start-calibrated-direct");

    // A failed upward transaction is negative evidence, but one isolated failure
    // is not a session-long verdict. Repeated whole-batch failure is the hard
    // condition that earns immediate rollback and strong target-specific backoff.
    cl_rate_optimizer q;
    ok("probe table load", q.load(table().c_str()));
    q.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    p = q.get_policy();
    p.confidence_z = 1.0;
    p.direct_switch_margin = 0.03;
    p.probe_mean_margin = 0.05;
    p.min_outcome_samples = 1;
    p.cooldown_batches = 1;
    p.probe_cooldown_batches = 20;
    q.set_policy_for_test(p);
    q.set_switch_cost_ms(1);
    st_rate_observation base = obs(13, 1800, 5, 0);
    d = q.evaluate_v2(base, 14);
    ok("uncertain faster mode is probed", d.action == GEARSHIFT_ACTION_PROBE && d.target_cfg == 14);
    q.notify_switch_dispatched(13, 14, d.action, 13, 1000, false);
    q.notify_switch_confirmed(1100);
    q.observe_transaction(14, 0, 0, 1000, 0, 30, false, true, 20, 0, 0.02, 30, false);
    st_rate_observation failed_probe = obs(14, 0, 0, 1.0);
    d = q.evaluate_v2(failed_probe, 14);
    ok("single failed transaction stays in probation",
       d.action == GEARSHIFT_ACTION_HOLD && q.probe_is_active());
    ok("single failed transaction is not blacklisted",
       !q.probe_blocked_for_test(14, failed_probe));
    q.observe_transaction(14, 0, 0, 1000, 0, 30, false, true, 20, 0, 0.02, 30, false);
    d = q.evaluate_v2(failed_probe, 14);
    ok("repeated failed probe rolls back", d.action == GEARSHIFT_ACTION_ROLLBACK && d.target_cfg == 13);
    q.notify_switch_dispatched(14, 13, d.action, 13, 1200, false);
    q.notify_switch_confirmed(1300);
    for (int i=0; i<4; ++i) q.notify_cooldown_tick();
    d = q.evaluate_v2(base, 14);
    ok("failed target is not immediately re-probed", !(d.action == GEARSHIFT_ACTION_PROBE && d.target_cfg == 14));
    // A real channel-regime improvement invalidates that negative memory; the
    // old failure must not blacklist the faster mode for the rest of the session.
    q.observe_transaction(13,225,225,1000,30,30,false,false,28,0,0.01,30,false);
    st_rate_observation improved=base; improved.forward_snr_db=28; improved.forward_selectivity=0.01;
    ok("channel change reopens failed probe target", !q.probe_blocked_for_test(14,improved));

    // A technically successful probe that is materially slower than the
    // fallback must roll back too; "decoded something" is not the objective.
    cl_rate_optimizer qs;
    ok("slow-probe table load", qs.load(table().c_str()));
    qs.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    p = qs.get_policy(); p.confidence_z=1.0; p.direct_switch_margin=0.03;
    p.probe_mean_margin=0.05; p.min_outcome_samples=1; p.cooldown_batches=0;
    qs.set_policy_for_test(p); qs.set_switch_cost_ms(1);
    d = qs.evaluate_v2(base, 14);
    ok("slow-success setup probes", d.action==GEARSHIFT_ACTION_PROBE && d.target_cfg==14);
    qs.notify_switch_dispatched(13,14,d.action,13,1000,false);
    qs.notify_switch_confirmed(1001);
    st_rate_observation slow_probe = obs(14,1600,1,0.0);
    for (int i=0; i<2; ++i) {
        qs.observe_transaction(14,200,200,1000,30,30,false,false,20,0,0.02,30,false); // 1600 bps < ~1800 fallback
        d = qs.evaluate_v2(slow_probe,14);
        ok(i==0 ? "first soft underperformance remains probation" :
                  "second soft underperformance remains probation",
           d.action==GEARSHIFT_ACTION_HOLD && qs.probe_is_active());
    }
    qs.observe_transaction(14,200,200,1000,30,30,false,false,20,0,0.02,30,false);
    d = qs.evaluate_v2(slow_probe,14);
    ok("established slower probe rolls back", d.action==GEARSHIFT_ACTION_ROLLBACK && d.target_cfg==13);

    // Regression for the observed CONFIG_16 failure mode: a mediocre first
    // successful application sample must not decide the probe. The local
    // population 2400/4200/4500 bps should be allowed to recover and accept.
    cl_rate_optimizer transient;
    transient.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    p=transient.get_policy(); p.confidence_z=0.0; p.direct_switch_margin=0.50;
    p.probe_mean_margin=0.0; p.min_outcome_samples=1; p.cooldown_batches=0;
    transient.set_policy_for_test(p); transient.set_switch_cost_ms(1);
    st_rate_observation tbase=obs(CONFIG_14,2800,5,0);
    tbase.feasible_configs={CONFIG_14,CONFIG_16};
    tbase.nominal_bps[CONFIG_14]=3000; tbase.nominal_bps[CONFIG_16]=8000;
    tbase.tx_airtime_ms[CONFIG_16]=5572; tbase.feedback_budget_ms=500;
    d=transient.evaluate_v2(tbase,CONFIG_16);
    ok("transient setup probes cfg16",d.action==GEARSHIFT_ACTION_PROBE && d.target_cfg==CONFIG_16);
    transient.notify_switch_dispatched(CONFIG_14,CONFIG_16,d.action,CONFIG_14,1000,false);
    transient.notify_switch_confirmed(1001);
    transient.observe_transaction(CONFIG_16,300,300,1000,30,30,false,false,20,0,0.02,30,false); // 2400
    st_rate_observation tobs=obs(CONFIG_16,2400,1,0.0);
    d=transient.evaluate_v2(tobs,CONFIG_16);
    ok("bad first cfg16 sample does not rollback",d.action==GEARSHIFT_ACTION_HOLD && transient.probe_is_active());
    ok("bad first cfg16 sample does not strong-blacklist",!transient.probe_blocked_for_test(CONFIG_16,tobs));
    transient.observe_transaction(CONFIG_16,525,525,1000,30,30,false,false,20,0,0.02,30,false); // 4200
    d=transient.evaluate_v2(tobs,CONFIG_16);
    ok("second cfg16 sample remains probation",d.action==GEARSHIFT_ACTION_HOLD && transient.probe_is_active());
    transient.observe_transaction(CONFIG_16,563,563,1000,30,30,false,false,20,0,0.02,30,false); // ~4504
    d=transient.evaluate_v2(tobs,CONFIG_16);
    ok("recovered cfg16 probe is accepted",d.action==GEARSHIFT_ACTION_HOLD &&
       !transient.probe_is_active() && d.reason=="probe-accepted");
    ok("accepted cfg16 has no failed-probe backoff",!transient.probe_blocked_for_test(CONFIG_16,tobs));

    // An isolated failed target transaction may recover inside the same bounded
    // experiment. One failure followed by three strong successful application
    // samples is not a session-long CONFIG_16 blacklist.
    cl_rate_optimizer recoverfail;
    recoverfail.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    p=recoverfail.get_policy(); p.confidence_z=0.0; p.direct_switch_margin=0.50;
    p.probe_mean_margin=0.0; p.min_outcome_samples=1; p.cooldown_batches=0;
    recoverfail.set_policy_for_test(p); recoverfail.set_switch_cost_ms(1);
    d=recoverfail.evaluate_v2(tbase,CONFIG_16);
    ok("single-failure recovery setup probes cfg16",d.action==GEARSHIFT_ACTION_PROBE && d.target_cfg==CONFIG_16);
    recoverfail.notify_switch_dispatched(CONFIG_14,CONFIG_16,d.action,CONFIG_14,1000,false);
    recoverfail.notify_switch_confirmed(1001);
    recoverfail.observe_transaction(CONFIG_16,0,0,1000,0,30,false,true,20,0,0.02,30,false);
    d=recoverfail.evaluate_v2(tobs,CONFIG_16);
    ok("isolated cfg16 failure is still probation",d.action==GEARSHIFT_ACTION_HOLD && recoverfail.probe_is_active());
    recoverfail.observe_transaction(CONFIG_16,525,525,1000,30,30,false,false,20,0,0.02,30,false);
    recoverfail.observe_transaction(CONFIG_16,550,550,1000,30,30,false,false,20,0,0.02,30,false);
    recoverfail.observe_transaction(CONFIG_16,563,563,1000,30,30,false,false,20,0,0.02,30,false);
    d=recoverfail.evaluate_v2(tobs,CONFIG_16);
    ok("subsequent good evidence accepts after isolated failure",d.action==GEARSHIFT_ACTION_HOLD &&
       !recoverfail.probe_is_active() && d.reason=="probe-accepted");
    ok("isolated failure never becomes session-long blacklist",
       !recoverfail.probe_blocked_for_test(CONFIG_16,tobs));

    // Patch-3 A/B: once an ordinary Axis-1 switch is dispatched, that
    // transaction exclusively owns the lifecycle. Re-evaluation cannot propose
    // another SWITCH/PROBE/ROLLBACK, and an accidental duplicate dispatch must
    // preserve the original transaction identity.
    cl_rate_optimizer inflight; inflight.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    p=inflight.get_policy(); p.confidence_z=0.0; p.direct_switch_margin=0.50;
    p.probe_mean_margin=0.0; p.min_outcome_samples=1; p.cooldown_batches=0;
    inflight.set_policy_for_test(p); inflight.set_switch_cost_ms(1);
    d=inflight.evaluate_v2(tbase,CONFIG_16);
    ok("in-flight exclusivity setup probes cfg16",
       d.action==GEARSHIFT_ACTION_PROBE && d.target_cfg==CONFIG_16);
    inflight.notify_switch_dispatched(CONFIG_14,CONFIG_16,d.action,CONFIG_14,1000,false);
    ok("original switch transaction is live",
       inflight.switch_inflight_for_test() &&
       inflight.switch_from_cfg_for_test()==CONFIG_14 &&
       inflight.switch_to_cfg_for_test()==CONFIG_16 &&
       inflight.switch_action_for_test()==GEARSHIFT_ACTION_PROBE);
    for(int i=0;i<3;++i) {
        d=inflight.evaluate_v2(tbase,CONFIG_16);
        ok("in-flight evaluation is suppressed",
           d.action==GEARSHIFT_ACTION_HOLD && !d.actionable &&
           d.reason=="switch-inflight");
    }
    inflight.notify_switch_dispatched(CONFIG_14,CONFIG_15,GEARSHIFT_ACTION_SWITCH,
                                      CONFIG_14,1500,false);
    ok("duplicate dispatch preserves original transaction",
       inflight.switch_inflight_for_test() &&
       inflight.switch_from_cfg_for_test()==CONFIG_14 &&
       inflight.switch_to_cfg_for_test()==CONFIG_16 &&
       inflight.switch_action_for_test()==GEARSHIFT_ACTION_PROBE);
    inflight.notify_switch_confirmed(3000);
    ok("confirmation terminates exclusive transition and begins probation",
       !inflight.switch_inflight_for_test() && inflight.probe_is_active());
    inflight.observe_transaction(CONFIG_16,12400,12400,16000,30,30,false,false,20,0,0.02,30,false);
    d=inflight.evaluate_v2(tobs,CONFIG_16);
    ok("confirmed probe resumes normal probation",
       d.action==GEARSHIFT_ACTION_HOLD && d.reason=="probe-probation");

    cl_rate_optimizer switchfail; switchfail.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    p=switchfail.get_policy(); p.confidence_z=0.0; p.direct_switch_margin=0.50;
    p.probe_mean_margin=0.0; p.min_outcome_samples=1; p.cooldown_batches=0;
    switchfail.set_policy_for_test(p); switchfail.set_switch_cost_ms(1);
    d=switchfail.evaluate_v2(tbase,CONFIG_16);
    switchfail.notify_switch_dispatched(CONFIG_14,CONFIG_16,d.action,CONFIG_14,1000,false);
    switchfail.notify_switch_failed();
    ok("switch failure terminates exclusive transition",
       !switchfail.switch_inflight_for_test() && !switchfail.probe_is_active());
    d=switchfail.evaluate_v2(tbase,CONFIG_16);
    ok("policy resumes after switch failure under failure memory",
       d.reason!="switch-inflight" && !(d.action==GEARSHIFT_ACTION_PROBE && d.target_cfg==CONFIG_16));

    // Bench regression: Patch 2 physically showed an occasional unconfirmed
    // 0->16 SET_CONFIG followed by a successful retry about one control window
    // later. Patch 3 made switch_inflight exclusive but forgot a terminal
    // timeout, turning that recoverable loss into an immortal source-config hold.
    cl_rate_optimizer expired; expired.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    p=expired.get_policy(); p.confidence_z=0.0; p.direct_switch_margin=0.50;
    p.probe_mean_margin=0.0; p.min_outcome_samples=1; p.cooldown_batches=0;
    p.switch_inflight_timeout_ms=5000;
    expired.set_policy_for_test(p); expired.set_switch_cost_ms(1);
    d=expired.evaluate_v2(tbase,CONFIG_16);
    ok("expiry setup probes cfg16",
       d.action==GEARSHIFT_ACTION_PROBE && d.target_cfg==CONFIG_16);
    expired.notify_switch_dispatched(CONFIG_14,CONFIG_16,d.action,CONFIG_14,1000,false);
    st_rate_observation expired_obs=tbase;
    expired_obs.monotonic_ms=7001;
    expired_obs.feedback_budget_ms=0;
    d=expired.evaluate_v2(expired_obs,CONFIG_16);
    ok("expired in-flight transition reopens acquisition",
       !expired.switch_inflight_for_test() &&
       d.action==GEARSHIFT_ACTION_PROBE && d.target_cfg==CONFIG_16 &&
       d.reason!="switch-inflight");
    ok("unconfirmed transition expiry does not blacklist target",
       !expired.probe_blocked_for_test(CONFIG_16,expired_obs));

    // Bench regression: production feeds the live receiving_timeout into probe
    // economics. With the Patch-3 population-reachability veto, a realistic
    // ~45 s feedback window made 3 samples + safety mathematically exceed the
    // 120 s probation ceiling, so ACTIVE refused to probe at all. The hard
    // ceiling should bound probation, not prevent cold acquisition.
    cl_rate_optimizer livebudget; livebudget.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    p=livebudget.get_policy(); p.confidence_z=0.0; p.direct_switch_margin=0.50;
    p.probe_mean_margin=0.0; p.min_outcome_samples=4; p.min_rate_samples=1;
    p.cooldown_batches=0; p.probe_min_application_samples=3;
    p.probe_min_outcome_samples=3; p.probe_max_probation_ms=120000;
    livebudget.set_policy_for_test(p); livebudget.set_switch_cost_ms(1800);
    st_rate_observation physical=obs(CONFIG_0,23.1,1,0.0);
    physical.outcome_samples=1; physical.rate_samples=1;
    physical.feasible_configs={CONFIG_0,CONFIG_16};
    physical.nominal_bps[CONFIG_0]=66.4;
    physical.nominal_bps[CONFIG_16]=6743.5;
    physical.tx_airtime_ms[CONFIG_16]=5572;
    physical.feedback_budget_ms=45000;
    physical.queue_bytes=200000;
    d=livebudget.evaluate_v2(physical,CONFIG_16);
    ok("production-sized feedback budget still permits cold cfg16 probe",
       d.action==GEARSHIFT_ACTION_PROBE && d.target_cfg==CONFIG_16 &&
       d.reason=="cold-start-bounded-probe");

    // Patch-3 C: replay the slow physical CONFIG_16 cadence. The old 30-second
    // ceiling would roll back after only two healthy completed target outcomes.
    // Geometry seeds a reachable budget; observed target cadence extends it so
    // three requested samples/outcomes can physically arrive.
    cl_rate_optimizer slowcadence; slowcadence.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    p=slowcadence.get_policy(); p.confidence_z=0.0; p.direct_switch_margin=0.50;
    p.probe_mean_margin=0.0; p.min_outcome_samples=1; p.cooldown_batches=0;
    p.probe_min_application_samples=3; p.probe_min_outcome_samples=3;
    p.probe_max_probation_ms=120000; p.probe_zero_progress_ms=12000;
    slowcadence.set_policy_for_test(p); slowcadence.set_switch_cost_ms(1);
    d=slowcadence.evaluate_v2(tbase,CONFIG_16);
    ok("slow-cadence setup probes cfg16",d.action==GEARSHIFT_ACTION_PROBE);
    slowcadence.notify_switch_dispatched(CONFIG_14,CONFIG_16,d.action,CONFIG_14,1000,false);
    slowcadence.notify_switch_confirmed(3000); // ~2 s physical switch cost
    st_rate_observation fasttarget=obs(CONFIG_16,6200,1,0.0);
    for(int i=0;i<2;++i) {
        slowcadence.observe_transaction(CONFIG_16,12400,12400,16000,30,30,false,false,
                                        20,0,0.02,30,false); // 6200 app bps, healthy
        d=slowcadence.evaluate_v2(fasttarget,CONFIG_16);
        ok(i==0 ? "slow cfg16 first outcome remains probation" :
                  "slow cfg16 survives beyond old 30-second ceiling",
           d.action==GEARSHIFT_ACTION_HOLD && slowcadence.probe_is_active() &&
           d.reason=="probe-probation");
    }
    ok("observed cfg16 cadence extends probation budget",
       slowcadence.probe_channel_ms_for_test()>30000.0 &&
       slowcadence.probe_max_channel_ms_for_test()>slowcadence.probe_channel_ms_for_test());
    slowcadence.observe_transaction(CONFIG_16,12400,12400,16000,30,30,false,false,
                                    20,0,0.02,30,false);
    d=slowcadence.evaluate_v2(fasttarget,CONFIG_16);
    ok("healthy slow cfg16 accepted after requested population",
       d.action==GEARSHIFT_ACTION_HOLD && !slowcadence.probe_is_active() &&
       d.reason=="probe-accepted");

    // Patch-3 D: a transaction that *discovers* a channel generation change
    // straddles an unknown boundary. It may update the general model, but it
    // contributes zero fresh probation population and zero fresh probation time.
    cl_rate_optimizer genprobe; genprobe.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    p=genprobe.get_policy(); p.confidence_z=0.0; p.direct_switch_margin=0.50;
    p.probe_mean_margin=0.0; p.min_outcome_samples=1; p.cooldown_batches=0;
    genprobe.set_policy_for_test(p); genprobe.set_switch_cost_ms(1);
    d=genprobe.evaluate_v2(tbase,CONFIG_16);
    ok("generation-reset setup probes cfg16",d.action==GEARSHIFT_ACTION_PROBE);
    genprobe.notify_switch_dispatched(CONFIG_14,CONFIG_16,d.action,CONFIG_14,1000,false);
    genprobe.notify_switch_confirmed(1001);
    genprobe.observe_transaction(CONFIG_16,300,300,1000,30,30,false,false,20,0,0.02,30,false);
    ok("first probation sample counted",genprobe.probe_application_samples_for_test()==1);
    genprobe.observe_transaction(CONFIG_16,12131,12131,15654,30,30,false,false,
                                 28,0,0.01,30,false);
    ok("generation-trigger transaction is excluded from fresh probation",
       genprobe.probe_application_samples_for_test()==0 &&
       genprobe.probe_outcome_samples_for_test()==0 &&
       std::fabs(genprobe.probe_channel_ms_for_test())<0.1);
    genprobe.observe_transaction(CONFIG_16,525,525,1000,30,30,false,false,28,0,0.01,30,false);
    ok("next causally clean target transaction starts fresh generation",
       genprobe.probe_application_samples_for_test()==1 &&
       genprobe.probe_outcome_samples_for_test()==1 &&
       std::fabs(genprobe.probe_channel_ms_for_test()-1000.0)<0.1 &&
       std::fabs(genprobe.probe_mean_bps_for_test()-4200.0)<1.0);

    // A bounded ultimate ceiling still exists. Continued transport progress with
    // no completed application units is ambiguous, not catastrophic, and may
    // eventually weak-timeout instead of remaining in probation forever.
    cl_rate_optimizer timedprobe; timedprobe.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    p=timedprobe.get_policy(); p.confidence_z=0.0; p.direct_switch_margin=0.50;
    p.probe_mean_margin=0.0; p.min_outcome_samples=1; p.cooldown_batches=0;
    p.probe_min_application_samples=4; p.probe_min_outcome_samples=3;
    p.probe_max_probation_ms=10000; p.probe_zero_progress_ms=10000;
    p.probe_soft_cooldown_batches=2; timedprobe.set_policy_for_test(p);
    timedprobe.set_switch_cost_ms(1);
    st_rate_observation timebase=tbase; timebase.tx_airtime_ms[CONFIG_16]=1000;
    timebase.feedback_budget_ms=0;
    d=timedprobe.evaluate_v2(timebase,CONFIG_16);
    ok("bounded-ceiling setup probes cfg16",d.action==GEARSHIFT_ACTION_PROBE);
    timedprobe.notify_switch_dispatched(CONFIG_14,CONFIG_16,d.action,CONFIG_14,1000,false);
    timedprobe.notify_switch_confirmed(1001);
    for(int i=0;i<10;++i)
        timedprobe.observe_transaction(CONFIG_16,0,300,1000,30,30,false,false,20,0,0.02,30,false);
    d=timedprobe.evaluate_v2(tobs,CONFIG_16);
    ok("ultimate probation ceiling still bounds ambiguous progress",
       d.action==GEARSHIFT_ACTION_ROLLBACK && d.reason=="probe-probation-timeout");
    ok("probation timeout arms only short target memory",
       timedprobe.probe_blocked_for_test(CONFIG_16,tobs));
    timedprobe.notify_switch_dispatched(CONFIG_16,CONFIG_14,d.action,CONFIG_14,12000,false);
    timedprobe.notify_switch_confirmed(12001);
    timedprobe.observe_transaction(CONFIG_14,350,350,1000,30,30,false,false,20,0,0.02,30,false);
    timedprobe.observe_transaction(CONFIG_14,350,350,1000,30,30,false,false,20,0,0.02,30,false);
    ok("weak timeout target memory expires without progressive blacklist",
       !timedprobe.probe_blocked_for_test(CONFIG_16,tbase));

    // Patch-3 E: making healthy probation patient must not make failure timid.
    cl_rate_optimizer hardstill; hardstill.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    p=hardstill.get_policy(); p.confidence_z=0.0; p.direct_switch_margin=0.50;
    p.probe_mean_margin=0.0; p.min_outcome_samples=1; p.cooldown_batches=0;
    p.probe_max_probation_ms=120000; hardstill.set_policy_for_test(p);
    hardstill.set_switch_cost_ms(1);
    d=hardstill.evaluate_v2(tbase,CONFIG_16);
    hardstill.notify_switch_dispatched(CONFIG_14,CONFIG_16,d.action,CONFIG_14,1000,false);
    hardstill.notify_switch_confirmed(1001);
    st_rate_observation hard_failed=obs(CONFIG_16,0,0,1.0); hard_failed.frame_success_rate=0.0;
    hardstill.observe_transaction(CONFIG_16,0,0,16000,0,30,false,true,20,0,0.02,30,false);
    d=hardstill.evaluate_v2(hard_failed,CONFIG_16);
    ok("first hard-failure transaction still gets recovery chance",
       d.action==GEARSHIFT_ACTION_HOLD && hardstill.probe_is_active());
    hardstill.observe_transaction(CONFIG_16,0,0,16000,0,30,false,true,20,0,0.02,30,false);
    d=hardstill.evaluate_v2(hard_failed,CONFIG_16);
    ok("hard failure still rolls back before extended probation budget",
       d.action==GEARSHIFT_ACTION_ROLLBACK && d.reason=="probe-hard-failure");

    // One application unit may cross a configuration boundary.  The final
    // commit must distribute useful-byte reward over every contributing config
    // rather than crediting only whichever config happened to finish it.
    cl_rate_optimizer x;
    x.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    p = permissive(x); x.set_policy_for_test(p);
    x.observe_transaction(13, 0, 300, 1000, 30, 30, false, false, -99.9, 99, -1.0, 30, false);
    x.notify_switch_dispatched(13, 14, GEARSHIFT_ACTION_PROBE, 13, 1000, false);
    x.notify_switch_confirmed(1100);
    x.observe_transaction(14, 0, 700, 1000, 30, 30, false, false, -99.9, 99, -1.0, 30, false);
    x.commit_application_unit(14, 1000, false);
    st_rate_observation xc; xc.current_cfg=14; xc.batch_size=30; xc.feasible_configs={13,14};
    // Deliberately misleading mixed rolling value: per-config attribution must
    // win once it exists.
    xc.application_bps=8000; xc.application_commits=1;
    st_rate_prediction x13=x.predict_for_test(13,xc), x14=x.predict_for_test(14,xc);
    ok("cross-config application unit credits first contributor", x13.live_samples==1 && std::fabs(x13.mean_bps-2400.0)<1.0);
    ok("cross-config application unit credits final contributor", x14.live_samples==1 && std::fabs(x14.mean_bps-5600.0)<1.0);
    ok("cross-config unit does not satisfy probe probation sample bar",
       x.probe_application_samples_for_test()==0 && x.probe_is_active());

    // Multipath calibration must be matched in measured SNR+selectivity space,
    // not by an arbitrary label when WGN and MP cells share the same SNR.
    const char* ctx_path="/tmp/gs2_context.json";
    { std::ofstream f(ctx_path); f << R"JSON({"calibration_setup":{"compress":false},"table":{"14":{
      "wgn20":{"snr_db":20,"selectivity_mean":0.02,"eff_bps_mean":3200,"eff_bps_sigma":100,"n_runs":5},
      "mp20":{"snr_db":20,"selectivity_mean":0.25,"eff_bps_mean":1200,"eff_bps_sigma":100,"n_runs":5}
    }}})JSON"; }
    cl_rate_optimizer ctx; ok("context table load",ctx.load(ctx_path));
    st_rate_observation flat=obs(13,1800,2,0); flat.forward_snr_db=20; flat.forward_selectivity=0.02;
    st_rate_observation selective=flat; selective.forward_selectivity=0.25;
    st_rate_prediction pflat=ctx.predict_for_test(14,flat);
    st_rate_prediction psel=ctx.predict_for_test(14,selective);
    ok("joint context picks flat calibration", pflat.valid && pflat.mean_bps>2600);
    ok("joint context picks selective calibration", psel.valid && psel.mean_bps<1800);
    ok("joint context changes candidate prediction", pflat.mean_bps > psel.mean_bps*1.5);

    // Reverse quality is explicitly tagged and may only act as a weak symmetry
    // prior when a fresh remote/forward report is unavailable.
    cl_rate_optimizer dir; ok("direction table load",dir.load(table().c_str()));
    st_rate_observation fwd=obs(13,1800,2,0);
    st_rate_observation rev=fwd;
    rev.forward_snr_db=-99.9; rev.forward_snr_age_batches=1000;
    rev.reverse_snr_db=20.0; rev.reverse_snr_age_batches=0;
    st_rate_prediction pf=dir.predict_for_test(14,fwd);
    st_rate_prediction pr=dir.predict_for_test(14,rev);
    ok("reverse SNR is weaker than real forward SNR", pr.valid && pf.valid && pr.prior_weight < pf.prior_weight);
    ok("reverse SNR carries larger uncertainty", pr.sigma_bps > pf.sigma_bps);
    ok("reverse SNR source is explicit", pr.source.find("reverse-snr-symmetry-prior") != std::string::npos);

    // A physical-context step produces explicit volatility rather than leaving
    // the controller with a fixed stationary horizon.
    dir.reset_session_state();
    dir.observe_transaction(13, 200, 200, 1000, 30, 30, false, false, 18, 0, 0.02, 30, false);
    dir.observe_transaction(13, 200, 200, 1000, 30, 30, false, false, 28, 0, 0.15, 30, false);
    ok("channel step raises context volatility", dir.context_volatility_for_test() > 0.0);

    // The universal SET_CONFIG chokepoint may see both v2-owned and external
    // safety transitions. Matching v2 transitions must not self-cancel; a later
    // external override must cancel the probe, arm cooldown and age old evidence.
    cl_rate_optimizer ext; ok("external-sync table load", ext.load(table().c_str()));
    ext.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    p = ext.get_policy(); p.confidence_z=1.0; p.direct_switch_margin=0.03;
    p.probe_mean_margin=0.05; p.min_outcome_samples=1; p.cooldown_batches=3;
    p.probe_cooldown_batches=20; ext.set_policy_for_test(p); ext.set_switch_cost_ms(1);
    d = ext.evaluate_v2(base, 14);
    ok("external-sync setup probe", d.action==GEARSHIFT_ACTION_PROBE && d.target_cfg==14);
    ext.notify_switch_dispatched(13,14,d.action,13,1000,false);
    ok("own SET_CONFIG is not external override",
       !ext.notify_external_axis1_transition(13,14,"set-config-chokepoint",false));
    ok("own SET_CONFIG preserves probe", ext.probe_is_active());
    ext.notify_switch_confirmed(1010);
    ext.observe_transaction(14,300,300,1000,30,30,false,false,20,0,0.02,30,false);
    st_rate_observation extobs=obs(14,2400,1,0);
    st_rate_prediction ext_before=ext.predict_for_test(14,extobs);
    ok("external safety move is synchronized",
       ext.notify_external_axis1_transition(14,13,"BREAK-emergency",false));
    ok("external safety move cancels probe", !ext.probe_is_active());
    ok("external safety move arms cooldown", ext.cooldown_active());
    st_rate_prediction ext_after=ext.predict_for_test(14,extobs);
    ok("external safety move ages prior live context", ext_after.live_weight < ext_before.live_weight);

    // A probe whose SET_CONFIG itself fails is negative evidence too. It must
    // not be immediately attempted again just because no DATA probation ran.
    cl_rate_optimizer sf; ok("switch-fail table load", sf.load(table().c_str()));
    sf.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    p = sf.get_policy(); p.confidence_z=1.0; p.direct_switch_margin=0.03;
    p.probe_mean_margin=0.05; p.min_outcome_samples=1; p.cooldown_batches=0;
    p.probe_cooldown_batches=20; sf.set_policy_for_test(p); sf.set_switch_cost_ms(1);
    d=sf.evaluate_v2(base,14);
    ok("switch-fail setup probe", d.action==GEARSHIFT_ACTION_PROBE && d.target_cfg==14);
    sf.notify_switch_dispatched(13,14,d.action,13,1000,false);
    sf.notify_switch_failed();
    d=sf.evaluate_v2(base,14);
    ok("failed probe SET_CONFIG is remembered", !(d.action==GEARSHIFT_ACTION_PROBE && d.target_cfg==14));

    // A configured top-gear action must stay reachable after the legacy topgear
    // election is disabled.  CONFIG_17 is intentionally outside FULL_CONFIG_LADDER,
    // so this guards the v2 action model itself (the static audit guards the builder).
    cl_rate_optimizer tg;
    tg.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    p = permissive(tg); p.confidence_z = 0.0; p.probe_mean_margin = 0.0;
    tg.set_policy_for_test(p); tg.set_switch_cost_ms(1);
    st_rate_observation top;
    top.current_cfg=CONFIG_16; top.application_bps=4200; top.transport_bps=4200;
    top.application_commits=4; top.rate_samples=4; top.outcome_samples=6;
    top.frame_success_rate=1.0; top.batch_size=30; top.queue_bytes=500000;
    top.feasible_configs={CONFIG_16,CONFIG_17};
    top.nominal_bps[CONFIG_16]=4500; top.nominal_bps[CONFIG_17]=7000;
    top.tx_airtime_ms[CONFIG_17]=1500;
    d=tg.evaluate_v2(top,CONFIG_17);
    ok("v2 topgear cfg17 is reachable", d.target_cfg==CONFIG_17 &&
       (d.action==GEARSHIFT_ACTION_SWITCH || d.action==GEARSHIFT_ACTION_PROBE));

    // Probe value-of-information must price candidate DATA airtime.  A giant
    // probation burst on a tiny remaining transfer is not worthwhile even when
    // the candidate's nominal rate is attractive; the same candidate becomes
    // worth probing on a long transfer.
    cl_rate_optimizer econ;
    econ.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    p = econ.get_policy(); p.confidence_z=1.0; p.direct_switch_margin=0.50;
    p.probe_mean_margin=0.02; p.min_outcome_samples=1; p.min_rate_samples=1;
    p.cooldown_batches=0; econ.set_policy_for_test(p); econ.set_switch_cost_ms(100);
    st_rate_observation shortq=obs(13,1800,4,0);
    shortq.feasible_configs={13,14}; shortq.queue_bytes=2500;
    shortq.tx_airtime_ms[14]=7000; // worst-case probe consumes most short horizon
    d=econ.evaluate_v2(shortq,14);
    ok("short transfer suppresses expensive probe", d.action!=GEARSHIFT_ACTION_PROBE);
    st_rate_observation longq=shortq; longq.queue_bytes=500000;
    d=econ.evaluate_v2(longq,14);
    ok("long transfer permits worthwhile probe", d.action==GEARSHIFT_ACTION_PROBE || d.action==GEARSHIFT_ACTION_SWITCH);
    // A known finite queue shorter than the generic minimum horizon must stay
    // short. Otherwise switching economics would amortize cost over bytes that
    // do not exist.
    st_rate_observation tinyq=shortq; tinyq.queue_bytes=100;
    d=econ.evaluate_v2(tinyq,14);
    ok("finite queue horizon is not inflated to policy minimum", d.horizon_ms < p.min_horizon_ms);
    // A long ACK/listen timeout is part of probe exposure too.  The same probe
    // that is worthwhile with a quick robust ACK must be rejected when failure
    // would burn most of the useful horizon waiting for feedback.
    st_rate_observation feedbackq=shortq; feedbackq.queue_bytes=10000;
    feedbackq.tx_airtime_ms[14]=500; feedbackq.feedback_budget_ms=25000;
    d=econ.evaluate_v2(feedbackq,14);
    ok("probe economics include feedback timeout exposure", d.action!=GEARSHIFT_ACTION_PROBE);

    // Outcome-only records must not trigger ordinary rate optimization. They can
    // still drive urgent failure recovery, but ordinary switching waits for one
    // timed denominator as required by min_rate_samples.
    cl_rate_optimizer rg; ok("rate-gate table load", rg.load(table().c_str()));
    rg.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    p=permissive(rg); p.min_rate_samples=1; rg.set_policy_for_test(p); rg.set_switch_cost_ms(1);
    st_rate_observation only_outcome=obs(13,1800,0,0);
    only_outcome.rate_samples=0; only_outcome.outcome_samples=6;
    d=rg.evaluate_v2(only_outcome,14);
    ok("ordinary decision waits for timed rate evidence",
       d.action==GEARSHIFT_ACTION_ABSTAIN && d.reason=="learning-rate");
    only_outcome.failed_batch_rate=1.0; only_outcome.frame_success_rate=0.0;
    only_outcome.feasible_configs={12,13,14}; only_outcome.nominal_bps[12]=1200;
    d=rg.evaluate_v2(only_outcome,14);
    ok("urgent failure may move without timed rate evidence",
       d.action==GEARSHIFT_ACTION_SWITCH && d.target_cfg<13);

    // In a full-ladder (-R) session the robust tier is a normal v2 action, not
    // merely a BREAK destination. Numeric ROBUST_* IDs are 100+, so this also
    // guards against accidentally applying the OFDM config ceiling by raw ID.
    cl_rate_optimizer deep;
    deep.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    p = permissive(deep); deep.set_policy_for_test(p); deep.set_switch_cost_ms(1);
    st_rate_observation cliff;
    cliff.current_cfg = CONFIG_0;
    cliff.application_bps = 0.0; cliff.transport_bps = 0.0;
    cliff.outcome_samples = 6; cliff.rate_samples = 6;
    cliff.failed_batch_rate = 1.0; cliff.frame_success_rate = 0.0;
    cliff.forward_snr_db = -99.9; cliff.forward_snr_age_batches = 1000;
    cliff.batch_size = 1; cliff.queue_bytes = 200000;
    cliff.feasible_configs = {ROBUST_0, ROBUST_1, ROBUST_2, CONFIG_0};
    cliff.nominal_bps[ROBUST_0] = 14.0;
    cliff.nominal_bps[ROBUST_1] = 22.0;
    cliff.nominal_bps[ROBUST_2] = 87.0;
    cliff.nominal_bps[CONFIG_0] = 200.0;
    d = deep.evaluate_v2(cliff, CONFIG_16);
    ok("deep failure selects robust tier", d.action == GEARSHIFT_ACTION_SWITCH && d.target_cfg == ROBUST_2);

    // A robust current mode is below the OFDM ceiling by ladder rank despite its
    // raw numeric ID. It must not be falsely classified as above-admission.
    st_rate_observation robust_live = cliff;
    robust_live.current_cfg = ROBUST_2;
    robust_live.failed_batch_rate = 0.0; robust_live.frame_success_rate = 1.0;
    robust_live.application_bps = 80.0; robust_live.application_commits = 4;
    robust_live.outcome_samples = 6;
    d = deep.evaluate_v2(robust_live, CONFIG_16);
    ok("robust current is not raw-id ceiling departure", d.reason != "current-above-admission-ceiling");

    // Optional calibration context serialized as JSON null means UNKNOWN, not
    // measured 0 dB / perfectly flat / batch-size zero.  The loader must fall
    // back to outcome matching without fabricating physical context.
    const char* null_path="/tmp/gs2_null_context.json";
    { std::ofstream f(null_path); f << R"JSON({"calibration_setup":{"compress":false},"table":{
      "13":{"legacy":{"snr_db":null,"selectivity_mean":null,"batch_size_mean":null,"eff_bps_mean":1800,"sack_rate_mean":0.1,"frame_loss_pct":5,"n_runs":4}},
      "14":{"legacy":{"snr_db":null,"selectivity_mean":null,"batch_size_mean":null,"eff_bps_mean":2500,"sack_rate_mean":0.1,"frame_loss_pct":5,"n_runs":4}}
    }})JSON"; }
    cl_rate_optimizer nul; ok("null-context table load", nul.load(null_path));
    st_rate_observation nulobs=obs(13,1800,2,0);
    nulobs.candidate_batch_size[14]=5;
    st_rate_prediction nulp=nul.predict_for_test(14,nulobs);
    ok("JSON null context remains unknown", nulp.valid && nulp.source.find("outcome-prior")!=std::string::npos);
    ok("JSON null batch does not invent mismatch", nulp.source.find("batch-context")==std::string::npos);

    // A calibration action that repeatedly required emergency BREAK is not as
    // authoritative as an otherwise-identical action that remained stable.
    // Preserve the measured mean (short transactions may still benefit) while
    // widening uncertainty and lowering prior weight.
    const char* break_path="/tmp/gs2_break_risk.json";
    { std::ofstream f(break_path); f << R"JSON({"calibration_setup":{"compress":false},"table":{
      "14":{"wgn20":{"snr_db":20,"eff_bps_mean":2500,"eff_bps_sigma":100,"n_runs":5,"break_run_rate":0.0}},
      "15":{"wgn20":{"snr_db":20,"eff_bps_mean":2500,"eff_bps_sigma":100,"n_runs":5,"break_run_rate":1.0,"break_fired":true}}
    }})JSON"; }
    cl_rate_optimizer br; ok("break-risk table load", br.load(break_path));
    st_rate_observation brobs=obs(13,1800,2,0);
    st_rate_prediction brsafe=br.predict_for_test(14,brobs);
    st_rate_prediction brrisky=br.predict_for_test(15,brobs);
    ok("BREAK calibration lowers prior authority", brrisky.prior_weight < brsafe.prior_weight);
    ok("BREAK calibration widens uncertainty", brrisky.sigma_bps > brsafe.sigma_bps);


    // CP4: selectivity has independent provenance/freshness.  A fresh
    // selectivity step must still create a new context generation when the
    // forward-SNR report is stale or absent.
    cl_rate_optimizer selage;
    selage.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    selage.observe_transaction(13,200,200,1000,30,30,false,false,
                               -99.9,1000,0.01,30,false,0,1000);
    const int sel_gen0 = selage.context_generation_for_test();
    selage.observe_transaction(13,200,200,1000,30,30,false,false,
                               -99.9,1000,0.90,30,false,0,2000);
    ok("fresh selectivity changes context with stale SNR",
       selage.context_generation_for_test() > sel_gen0);

    // CP4: the calibration tool may legitimately emit only table_nb.  Such a
    // file is a usable calibration, not an invalid file merely because WB data
    // is absent.
    const char* nb_only_path="/tmp/gs2_nb_only.json";
    { std::ofstream f(nb_only_path); f << R"JSON({"calibration_setup":{"compress":false},"table_nb":{
      "5":{"nb":{"snr_db":20,"eff_bps_mean":700,"eff_bps_sigma":80,"n_runs":5}}
    }})JSON"; }
    cl_rate_optimizer nbo;
    ok("NB-only calibration loads", nbo.load(nb_only_path) && nbo.has_calibration());
    st_rate_observation nbobs;
    nbobs.current_cfg=5; nbobs.is_nb=true; nbobs.forward_snr_db=20;
    nbobs.forward_snr_age_batches=0; nbobs.batch_size=5; nbobs.candidate_batch_size[5]=5;
    st_rate_prediction nbp=nbo.predict_for_test(5,nbobs);
    ok("NB-only calibration predicts NB action", nbp.valid && nbp.mean_bps>0.0);

    // CP4: nullable numerical fields remain absent.  In particular an unknown
    // eff_bps_mean is not a measured zero-delivery run.
    const char* null_rate_path="/tmp/gs2_null_rate.json";
    { std::ofstream f(null_rate_path); f << R"JSON({"calibration_setup":{"compress":false},"table":{
      "14":{"unknown":{"snr_db":20,"eff_bps_mean":null,"sack_rate_mean":null,"n_runs":5}}
    }})JSON"; }
    cl_rate_optimizer nullrate;
    ok("null-rate file has no valid calibration cell", !nullrate.load(null_rate_path));

    // CP4: explicit configuration-identity mismatch must weaken cold-start
    // action authority, not merely lower an unused mixture weight.
    const char* stale_path="/tmp/gs2_stale_identity.json";
    { std::ofstream f(stale_path); f << R"JSON({"config_signature":"definitely-wrong","calibration_setup":{"compress":false},"table":{
      "14":{"wgn20":{"snr_db":20,"eff_bps_mean":3000,"eff_bps_sigma":100,"n_runs":8}}
    }})JSON"; }
    cl_rate_optimizer stale; ok("stale calibration still parses as prior", stale.load(stale_path));
    st_rate_observation staleobs=obs(13,1800,2,0); staleobs.candidate_batch_size[14]=30;
    st_rate_prediction stalep=stale.predict_for_test(14,staleobs);
    ok("identity-mismatched calibration is probe-only",
       stalep.valid && !stalep.direct_evidence &&
       stalep.source.find("identity-mismatch-probe-only")!=std::string::npos);
    ok("identity-mismatched calibration widens uncertainty", stalep.sigma_bps >= 300.0);

    // CP4: interpolation is empirical only inside measured support.  Far SNR
    // extrapolation keeps the prior as information but loses direct authority
    // and becomes substantially less certain.
    const char* edge_path="/tmp/gs2_snr_edge.json";
    { std::ofstream f(edge_path); f << R"JSON({"calibration_setup":{"compress":false},"table":{"14":{
      "s20":{"snr_db":20,"eff_bps_mean":3000,"eff_bps_sigma":100,"n_runs":8},
      "s30":{"snr_db":30,"eff_bps_mean":4000,"eff_bps_sigma":100,"n_runs":8}
    }}})JSON"; }
    cl_rate_optimizer edge; ok("SNR-edge table load",edge.load(edge_path));
    st_rate_observation inside=obs(13,1800,2,0); inside.forward_snr_db=20;
    st_rate_observation outside=inside; outside.forward_snr_db=-10;
    st_rate_prediction edge_in=edge.predict_for_test(14,inside);
    st_rate_prediction edge_out=edge.predict_for_test(14,outside);
    ok("out-of-support SNR is not direct evidence", edge_out.valid && !edge_out.direct_evidence);
    ok("out-of-support SNR is marked extrapolated", edge_out.source.find("extrapolated")!=std::string::npos);
    ok("out-of-support SNR widens uncertainty", edge_out.sigma_bps > edge_in.sigma_bps*2.0);

    // CP4: live evidence relevance is a property of the candidate geometry,
    // not whatever batch size the currently-running mode happens to use.
    cl_rate_optimizer cbatch;
    cbatch.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    for(int i=0;i<8;++i)
        cbatch.observe_transaction(14,350,350,1000,30,30,false,false,-99.9,99,-1.0,30,false);
    st_rate_observation csmall=obs(13,1800,4,0); csmall.forward_snr_db=-99.9; csmall.forward_selectivity=-1.0;
    csmall.batch_size=5; csmall.candidate_batch_size[14]=30;
    st_rate_observation clarge=csmall; clarge.batch_size=30;
    st_rate_prediction cb_small=cbatch.predict_for_test(14,csmall);
    st_rate_prediction cb_large=cbatch.predict_for_test(14,clarge);
    ok("candidate batch geometry owns live relevance",
       std::fabs(cb_small.live_weight-cb_large.live_weight)<1e-9 &&
       std::fabs(cb_small.sigma_bps-cb_large.sigma_bps)<1e-6);

    // CP4: compression conversion comes from a whole atomic application unit.
    // A SET_CONFIG-reset rolling window seeing only the final transport fragment
    // must not invent a 4x gain for a true 2x-compressed unit.
    cl_rate_optimizer ag;
    ag.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    ag.observe_transaction(13,0,250,1000,30,30,false,false,-99.9,99,-1.0,30,false);
    ag.observe_transaction(14,0,250,1000,30,30,false,false,-99.9,99,-1.0,30,false);
    ag.commit_application_unit(14,1000,false);
    ok("atomic application transport gain is learned",
       ag.atomic_application_transport_gain_valid_for_test() &&
       std::fabs(ag.atomic_application_transport_gain_for_test()-2.0)<0.01);
    st_rate_observation agobs=obs(14,2000,1,0); agobs.compression_enabled=true;
    agobs.application_transport_gain=4.0; agobs.application_transport_gain_valid=true;
    agobs.nominal_bps[16]=1000; agobs.forward_snr_db=-99.9; agobs.forward_selectivity=-1.0;
    st_rate_prediction agp=ag.predict_for_test(16,agobs);
    ok("atomic gain overrides misleading rolling ratio",
       agp.valid && agp.mean_bps>1800 && agp.mean_bps<2200 &&
       agp.source.find("app-unit-converted-atomic")!=std::string::npos);

    // CP4: evidence ages over real monotonic channel time even if no DATA batch
    // completes during a long idle interval.
    cl_rate_optimizer wallage;
    wallage.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    st_rate_policy wallp=wallage.get_policy(); wallp.live_stale_half_life_ms=5000.0;
    wallage.set_policy_for_test(wallp);
    for(int i=0;i<6;++i)
        wallage.observe_transaction(14,350,350,1000,30,30,false,false,-99.9,99,-1.0,30,false,-1,1000+i*1000);
    st_rate_observation wall_now=obs(13,1800,1,0); wall_now.forward_snr_db=-99.9; wall_now.forward_selectivity=-1.0;
    wall_now.monotonic_ms=6000; wall_now.candidate_batch_size[14]=30;
    st_rate_observation wall_late=wall_now; wall_late.monotonic_ms=66000;
    st_rate_prediction wall_fresh=wallage.predict_for_test(14,wall_now);
    st_rate_prediction wall_stale=wallage.predict_for_test(14,wall_late);
    ok("idle wall time ages live evidence", wall_stale.live_weight < wall_fresh.live_weight*0.01);

    // CP4: an unactionable speculative best-mean candidate cannot conceal a
    // lower-mean direct action that pays after switch cost.
    const char* rank_path="/tmp/gs2_action_rank.json";
    { std::ofstream f(rank_path); f << R"JSON({"calibration_setup":{"compress":false},"table":{
      "13":{"wgn20":{"snr_db":20,"eff_bps_mean":1000,"eff_bps_sigma":10,"n_runs":8}},
      "14":{"wgn20":{"snr_db":20,"eff_bps_mean":1500,"eff_bps_sigma":10,"n_runs":8}}
    }})JSON"; }
    cl_rate_optimizer ranker; ok("action-rank table load",ranker.load(rank_path));
    ranker.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    p=permissive(ranker); p.min_rate_samples=1; p.min_outcome_samples=1; ranker.set_policy_for_test(p); ranker.set_switch_cost_ms(100);
    st_rate_observation rankobs=obs(13,1000,2,0); rankobs.feasible_configs={13,14,16};
    rankobs.nominal_bps[16]=100000; rankobs.tx_airtime_ms[16]=50000; rankobs.queue_bytes=5000;
    d=ranker.evaluate_v2(rankobs,16);
    ok("action ranking falls through to profitable evidenced switch",
       d.action==GEARSHIFT_ACTION_SWITCH && d.target_cfg==14);

    // CP4: known empty finite work is not the old 'unknown horizon' sentinel.
    // It must not amortize an ordinary switch/probe over the policy default.
    cl_rate_optimizer emptywork; emptywork.set_mode_for_test(GEARSHIFT_V2_ACTIVE);
    p=permissive(emptywork); p.min_rate_samples=1; p.min_outcome_samples=1; emptywork.set_policy_for_test(p); emptywork.set_switch_cost_ms(100);
    st_rate_observation emptyobs=obs(13,1000,2,0); emptyobs.remaining_work_known=true; emptyobs.queue_bytes=0;
    d=emptywork.evaluate_v2(emptyobs,16);
    ok("known-zero remaining work has finite near-zero horizon", d.horizon_ms<=1.01);
    ok("known-zero remaining work does not speculate",
       d.action==GEARSHIFT_ACTION_HOLD || d.action==GEARSHIFT_ACTION_ABSTAIN);

    if (failures) return 1;
    std::puts("PASS Gearshift-v2 core + uncalibrated/batch/change/probe-memory decisions");
    return 0;
}
