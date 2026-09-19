#!/usr/bin/env python3
"""Gearshift-v2 recursive requirements gate.

Every code-addressable architecture requirement from the adversarial design pass
must be mechanically evidenced here.  A requirement may be EXTERNAL_ONLY only
when it cannot be established by code in this review environment (RF/IONOS or
remote telemetry that is not present on the negotiated wire).  The release gate
fails on every other missing item.
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]

def text(rel):
    return (ROOT / rel).read_text(encoding="utf-8", errors="replace")

arqh = text("include/datalink_layer/arq.h")
cmd = text("source/datalink_layer/arq_commander.cc")
rsp = text("source/datalink_layer/arq_responder.cc")
common = text("source/datalink_layer/arq_common.cc")
telecom = text("source/physical_layer/telecom_system.cc")
defines = text("include/common/common_defines.h")
roh = text("include/datalink_layer/rate_optimizer.h")
ro = text("source/datalink_layer/rate_optimizer.cc")
cal = text("tools/sim/effective_rate_calibrate.py")
run = text("tests/cpu/run_gearshift_tests.sh")
closed = text("tests/cpu/test_gearshift_v2_closed_loop.cc")
metrics_test = text("tests/cpu/test_optimizer_metrics.cc")
core_test = text("tests/cpu/test_gearshift_v2.cc")
quality = text("include/datalink_layer/gearshift_quality_report.h")
quality_test = text("tests/cpu/test_gearshift_quality_report.cc")

checks = []
def req(name, condition, evidence):
    checks.append((name, bool(condition), evidence))

def allin(s, *needles): return all(n in s for n in needles)

# Objective + measurement contract.
req("application-goodput-is-authoritative",
    allin(arqh, "application_bps", "transport_bps", "opt_commit_application_bytes") and
    "batch_uncompressed_size" in cmd,
    "separate application/transport counters + post-ACK original-byte commit")
req("outcome-and-rate-validity-are-separate",
    allin(arqh, "opt_batch_rate_valid", "opt_batch_outcome_valid") and
    allin(metrics_test, "outcome-only rate n", "outcome-only outcome n"),
    "L1 outcome-only evidence cannot fabricate airtime")
req("zero-delivery-is-real-performance",
    "zero app bps" in metrics_test and
    allin(cal, "n_zero_delivery_runs", "v2_all_valid_runs_including_zero"),
    "valid zero delivery stays in denominator/calibration")
req("primitive-calibration-records",
    allin(arqh, "[OPT-BATCH]", "[OPT-APP-COMMIT]") and
    allin(cal, "parse_opt_batch_records", "primitive_records"),
    "non-overlapping transaction records replace rolling-window calibration")
req("primitive-trace-is-not-production-hot-path",
    allin(arqh, "MERCURY_GS2_PRIMITIVE_TRACE", "opt_primitive_trace_enabled") and
    "runtime_env={'MERCURY_GS2_PRIMITIVE_TRACE': '1'}" in cal,
    "per-batch calibration evidence is explicitly enabled by the harness instead of forcing printf+fflush into normal goodput")
req("active-production-window-trace-is-not-hot-path",
    allin(arqh, "rate_opt.get_mode() != GEARSHIFT_V2_ACTIVE", "[OPT-WINDOW]", "opt_primitive_trace_enabled"),
    "ACTIVE normal operation does not pay a printf+fflush every third batch; legacy/shadow/calibration retain diagnostics")
req("receiver-failure-diagnostics-cannot-amplify-a-realtime-stall",
    allin(arqh, "ftr_fail_diag_run", "reports only powers of two") and
    allin(common, "ftr_fail_diag_run = 0", "run_n & (run_n - 1ULL)", "run=%llu") and
    allin(telecom, "if(g_verbose)", "[ENERGY-DIAG]", "[FINE-ENERGY-REL]",
          "[XCORR-RESCUE-FAIL]", "[SUBPEAK-REJECT]") and
    cmd.count("if(g_verbose) print_stats();") >= 2 and
    rsp.count("if(g_verbose) print_stats();") >= 2,
    "a continuous timing-recovery failure remains logarithmically observable while detailed per-attempt PHY and duplicate state-loop statistics are opt-in, so stdout flushing cannot become a competing realtime workload")
req("routine-compact-confirm-clears-peer-turnaround-before-next-data",
    allin(arqh, "arm_routine_turnaround_guard", "MERCURY_TURNAROUND_GUARD_SCOPE_ALL=0") and
    allin(common, "bool scope_all = true", "arm_routine_turnaround_guard()",
          "Arm E (routine, default)", "Arm F (routine, SCOPE_ALL=0)") and
    allin(cmd, "cmd_compact_confirm_crc_valid", "draining its reverse playback",
          "arm_routine_turnaround_guard();"),
    "a CRC-valid routine confirm causally stamps the existing geometry-derived turnaround clearance; the next DATA burst cannot key while the peer is still draining and re-arming")
req("compact-confirm-rearms-rx-after-ptt-release",
    allin(common, "PTT release precedes RX re-arm",
          "start the derived block-span budget", "settle_symbols",
          "ptt_on_delay_ms + ptt_off_delay_ms"),
    "known PTT-off silence cannot consume the next cfg16 block acquisition budget; the responder begins counting only when forward capture is actually re-armed")
req("incomplete-active-block-waits-for-tail-without-lost-tag-probing",
    allin(common, "in_forward_active_batch", "!received_message_stats.frame_data_missing",
          "Expanding that wait to a full block scrolls", "forward_batch=%d") and
    allin(rsp, "!telecom_system->receive_stats.frame_data_missing",
          "not lost-tag evidence"),
    "an active-batch preamble with a not-yet-captured tail is preserved and retried after a short wait; it cannot launch alternate-config decode or a full-block scroll")
req("primitive-ledger-carries-attempt-lineage-and-generation",
    allin(arqh, "lineage_valid=%d", "new_sent=%u", "repair_sent=%u", "cfg_gen=%u", "unit_id=%llu", "dir=forward") and
    allin(cal, "lineage_valid", "new_sent", "repair_sent", "config_generation", "application_unit_id", "direction"),
    "SACK/ARQ remains the truth source while each local primitive identifies new-vs-repair work, config generation and direction")
req("fixed-calibration-cannot-absorb-break-recovery-actions",
    allin(cal, "filter_primitive_records_for_config", "requested fixed configuration") and
    "filter_primitive_records_for_config(r, 14)" in text("tests/cpu/test_calibration_v2.py"),
    "emergency recovery may save the link but its later robust-mode bytes cannot inflate the failed fixed action")
req("break-prone-calibration-is-weaker-prior",
    allin(roh, "break_run_rate", "break_run_rate_valid") and
    allin(ro, "apply_break_risk_to_prior", "fixed action needed emergency recovery") and
    allin(text("tests/cpu/test_gearshift_v2.cc"),
          "BREAK calibration lowers prior authority", "BREAK calibration widens uncertainty"),
    "a mode that repeatedly needed emergency recovery remains measurable but cannot look as certain as a stable mode")
req("calibration-run-join-preserves-v2-context",
    allin(cal, "def build_cell_run", "failed_batch_rate", "frame_success",
          "batch_size", "selectivity", "snr_db", "primitive_records") and
    "return build_cell_run(raw, harvest, run_idx)" in cal and
    "harness-to-run join" in text("tests/cpu/test_calibration_v2.py"),
    "primitive calibration metrics survive the harness join instead of crashing/dropping to neutral defaults")
req("primitive-record-ids-are-monotonic-across-window-resets",
    "++opt_record_sequence" in arqh and "opt_record_sequence = 0" not in arqh and
    "opt_record_sequence = 0" in common,
    "config/window reset cannot alias [OPT-BATCH] ids used by later app-commit joins")
req("application-unit-ids-are-monotonic-across-config-window-resets",
    "opt_application_unit_sequence = 1" in common and
    "++opt_application_unit_sequence" in arqh and
    "opt_application_unit_sequence = 1" not in arqh and
    allin(arqh, "unit_id=%llu", "OPT-APP-COMMIT"),
    "one atomic application unit keeps an unambiguous process-lifetime id even when it spans SET_CONFIG/window resets")
req("application-credit-exact-record",
    allin(arqh, "opt_last_real_slot", "optimizer_commit_application_to_slot") and
    allin(metrics_test, "commit exact real slot", "outcome-only slot cannot steal commit"),
    "application commit cannot drift onto replay/another transaction")
req("cross-config-application-unit-attribution",
    allin(roh, "pending_transport_bytes", "commit_application_unit") and
    allin(ro, "commit_application_unit", "pending_transport_bytes"),
    "per-config live model distributes one atomic app unit over contributing actions")

req("cross-config-attribution-is-tested",
    allin(text("tests/cpu/test_gearshift_v2.cc"),
          "cross-config application unit credits first contributor",
          "cross-config application unit credits final contributor"),
    "application unit crossing SET_CONFIG is regression-tested")
req("mixed-rolling-rate-cannot-overwrite-per-config-attribution",
    allin(text("tests/cpu/test_gearshift_v2.cc"), "Deliberately misleading mixed rolling value",
          "cross-config application unit credits first contributor"),
    "per-config live samples win over a blended rolling application-rate observation")
req("direction-separation-is-tested",
    allin(text("tests/cpu/test_gearshift_v2.cc"),
          "reverse SNR is weaker than real forward SNR",
          "reverse SNR source is explicit"),
    "reverse telemetry cannot silently impersonate forward quality")

# Calibration is a prior, not authority.
req("no-optimizer-remains-a-hard-fixed-mode-kill",
    allin(common, "bool cl_arq_controller::opt_evaluate_batch_end", "if (optimizer_disabled) return false;") and
    allin(common, "void cl_arq_controller::opt_load_rate_table()", "disabled via --no-optimizer"),
    "fixed-mode/calibration runs cannot be moved by active v2 merely because v2 supports table-less operation")
req("calibration-is-optional-prior",
    allin(roh, "calibration_loaded", "has_calibration") and
    allin(ro, "continues uncalibrated", "uncalibrated"),
    "v2 remains operative without a table")
req("calibration-is-versioned",
    allin(ro, "config_signature", "MERCURY_BUILD_ID", "calibration_prior_weight_scale") and
    allin(cal, "get_config_signature", "mercury_head"),
    "stale/mismatched priors are identified and weakened")
req("per-config-online-learning",
    allin(roh, "std::map<int, st_online_rate_model>", "ewma_application_bps", "ewma_variance") and
    "live_scale" not in ro,
    "each action learns independently; no uniform scaling")
req("calibration-and-live-objective-use-same-byte-units",
    allin(roh, "application_transport_gain", "calibration_compressed") and
    allin(ro, "convert_prior_to_application_units", "app-unit-converted") and
    allin(common, "metrics.application_bps / metrics.transport_bps", "compression_enabled") and
    '"compress":     false' in cal and
    "raw calibration prior converted to application units" in text("tests/cpu/test_gearshift_v2.cc"),
    "raw-PHY calibration/analytic priors are converted to live application-goodput units when compression gain is measured")
req("uncertainty-and-age-affect-decisions",
    allin(ro, "sigma_bps", "live_stale_half_life_ms", "current_ucb", "const double lcb"),
    "mean/sigma/freshness enter switching policy")
req("evidence-freshness-is-channel-time-based",
    allin(roh, "live_stale_half_life_ms", "last_observation_ms", "observation_time_ms") and
    allin(ro, "observation_time_ms +=", "age_ms", "MERCURY_GS2_LIVE_HALF_LIFE_MS") and
    allin(text("tests/cpu/test_gearshift_v2.cc"),
          "live freshness decays by elapsed channel time", "time age is reported"),
    "5/10/15-second fading ages evidence by elapsed channel time rather than unequal batch counts")

# Complete feasible action set + economics.
req("candidate-set-comes-from-modem",
    allin(common, "obs.feasible_configs.push_back(cfg)", "predict_keydown_length_ms_for_config") and
    "obs.feasible_configs" in ro,
    "uncalibrated configs are candidates")
req("topgear-opt-in-is-real-not-dead-code",
    allin(common, 'getenv("MERCURY_TOPGEAR_ELECT")', "topgear_elect_enabled == 1",
          "return topgear_elect_enabled == 1;") and
    "topgear_elect_feature_enabled() ? CONFIG_17 : WB_CONFIG_MAX" in cmd,
    "the explicit CONFIG_17 opt-in actually opens the v2 cfg17 action path")
req("calibration-resume-cannot-mix-build-generations",
    allin(cal, "resume_metadata_compatible", "refusing unsafe --resume", "mercury_head mismatch", "config_signature mismatch") and
    "SCHEMA_VERSION = 2" in cal,
    "--resume cannot relabel stale cells as measurements from the current build")
req("calibration-tool-can-cover-top-and-robust-actions",
    "0 <= c <= 17 or c in (100, 101, 102)" in cal,
    "calibration tooling can characterize every ordinary/top/robust action v2 may expose")
req("topgear-cfg17-is-a-v2-action-when-explicitly-enabled",
    allin(common, "topgear_elect_feature_enabled()", "optimizer_ceiling = CONFIG_17",
          "obs.feasible_configs.push_back(CONFIG_17)") and
    "v2 topgear cfg17 is reachable" in text("tests/cpu/test_gearshift_v2.cc"),
    "CONFIG_17 remains reachable after disabling the separate legacy topgear authority")
req("probe-economics-include-candidate-data-airtime",
    allin(ro, "probe_airtime_ms", "probe_exposure_ms", "2.0*switch_cost_ewma_ms") and
    allin(text("tests/cpu/test_gearshift_v2.cc"),
          "short transfer suppresses expensive probe", "long transfer permits worthwhile probe"),
    "bounded exploration prices probe DATA airtime plus both possible transitions")
req("probe-economics-include-feedback-timeout",
    "feedback_budget_ms" in roh and "obs.feedback_budget_ms = receiving_timeout" in common and
    "obs.feedback_budget_ms" in ro and
    "probe economics include feedback timeout exposure" in text("tests/cpu/test_gearshift_v2.cc"),
    "a failed experiment prices the ACK/listen window instead of pretending forward keydown is the whole loss")
req("ordinary-decisions-require-timed-rate-evidence",
    allin(ro, "obs.rate_samples < policy.min_rate_samples", '"learning-rate"') and
    allin(text("tests/cpu/test_gearshift_v2.cc"),
          "ordinary decision waits for timed rate evidence",
          "urgent failure may move without timed rate evidence"),
    "outcome-only replay can drive safety evidence but cannot invent a performance denominator")
req("robust-tier-is-a-normal-v2-action-when-session-enables-it",
    allin(common, "FULL_CONFIG_LADDER_SIZE", "robust_enabled != YES", "obs.feasible_configs.push_back(cfg)") and
    allin(ro, "gearshift_action_rank", "gearshift_destination_within_ofdm_ceiling") and
    "deep failure selects robust tier" in text("tests/cpu/test_gearshift_v2.cc"),
    "ROBUST_0/1/2 are candidates by ladder rank in a full-ladder session; OFDM ceiling is not raw-ID applied")
req("robust-tier-dispatch-does-not-use-raw-id-ofdm-caps",
    allin(cmd, "const bool v2_target_ok", "is_robust_config(target)", "if (is_ofdm_config(target))"),
    "ROBUST_0/1/2 can pass the v2 SET_CONFIG executor without being clamped from raw ID 100+ to cfg16")
req("canonical-analytical-airtime-hint",
    allin(common, "predict_keydown_length_ms_for_config", "preamble_sched_nsymb", "nominal_bps"),
    "counterfactual keydown derives from actual PHY geometry")
req("candidate-batch-geometry-is-derived-per-action",
    allin(common, "predict_initial_batch_size_for_config", "candidate_batch_size",
          "candidate_frames", "if(is_robust_config(config)) return 1") and
    "std::map<int, int> candidate_batch_size" in roh,
    "counterfactual airtime uses the batch geometry the candidate will actually start with")
req("bigblock-candidate-economics-match-bigblock-framing",
    allin(common, "gs2_predict_bigblock_bundle", "bigblock_framing_enabled",
          "predict_payload_bytes_per_batch_for_config") and
    allin(text("include/datalink_layer/optimizer_geometry.h"),
          "optimizer_predict_bigblock_geometry", "BIGBLOCK_HDR_TOTAL_BYTES",
          "BIGBLOCK_BLOCK_CRC_BYTES") and
    "test_optimizer_geometry.cc" in run,
    "CFG16 big-block candidates price the thin-grid keydown/K/payload rather than K stock frames")
req("queue-aware-switch-economics",
    allin(ro, "queue_bytes", "useful_horizon_ms", "switch_cost_ewma_ms") and
    "notify_switch_confirmed" in ro,
    "remaining work + measured SET_CONFIG cost price transitions")
req("finite-queue-horizon-is-not-artificially-inflated",
    allin(ro, "finite_queue_known", "clamp_double(base, 1.0, policy.max_horizon_ms)") and
    "finite queue horizon is not inflated to policy minimum" in text("tests/cpu/test_gearshift_v2.cc"),
    "short transfers cannot justify a switch by amortizing it over nonexistent future bytes")
req("active-v2-is-not-hard-vetoed-by-legacy-snr-floors",
    allin(common, "Legacy SNR/floor gates", "GEARSHIFT_V2_LEGACY", "apply_cfg16_margin_cap", "apply_rung_floor_cap") and
    allin(cmd, "Legacy-only empirical SNR floors", "if (!rate_opt.controls_link())"),
    "old empirical rung/CFG16 SNR thresholds are evidence/fallback policy, not ACTIVE v2 hard ceilings")
req("batch-size-context-is-modeled",
    allin(roh, "batch_size_ewma", "batch_context_half_width") and
    allin(ro, "batch_match", "batch_size_ewma"),
    "Axis-2 changes do not masquerade as channel-only changes")
req("calibration-priors-carry-batch-geometry",
    allin(roh, "batch_size_mean", "batch_size_valid") and
    allin(ro, "apply_calibration_batch_context", "+batch-context") and
    allin(cal, "'batch_size':", "'batch_size_mean':") and
    allin(text("tests/cpu/test_gearshift_v2.cc"),
          "calibration batch mismatch lowers prior authority",
          "calibration batch mismatch widens uncertainty") and
    "test_calibration_v2.py" in run,
    "a table cell measured at one Axis-2 geometry is weakened when the candidate will use another")

# Direction, channel change, probing.
req("multipath-priors-use-joint-measured-context",
    allin(ro, "context_calibration_prediction", "calibration_snr_kernel_db",
          "calibration_selectivity_kernel", "snr-selectivity-context-prior") and
    allin(text("tests/cpu/test_gearshift_v2.cc"),
          "joint context picks flat calibration", "joint context picks selective calibration"),
    "WGN/MP cells at the same SNR are distinguished by measured selectivity instead of label order")
req("forward-and-reverse-quality-are-explicitly-separated",
    allin(arqh, "gearshift_forward_snr_db", "gearshift_reverse_snr_db") and
    allin(common, "gearshift_note_forward_snr", "gearshift_note_reverse_snr") and
    allin(ro, "reverse-snr-symmetry-prior", "reverse_snr_prior_weight_scale", "reverse_snr_sigma_gain"),
    "quality observations carry direction/age; reverse is only a weak explicit symmetry prior")
req("calibration-labels-are-not-fake-receiver-snr",
    'field == "snr_db"' in ro and "snr3k" not in ro and "parse_channel_snr_hint" not in ro,
    "IONOS labels like wgn20 are not silently reinterpreted as measured receiver SNR")
req("forward-selectivity-has-independent-age",
    allin(arqh, "gearshift_forward_selectivity_age_batches", "gearshift_forward_selectivity") and
    "forward_quality_max_age_batches" in common,
    "stale selectivity cannot remain fresh merely because another channel report updated")
req("channel-quality-age-thresholds-are-central-policy",
    allin(roh, "forward_quality_max_age_batches", "reverse_quality_max_age_batches") and
    allin(ro, "MERCURY_GS2_FORWARD_QUALITY_MAX_AGE", "MERCURY_GS2_REVERSE_QUALITY_MAX_AGE") and
    "rate_opt.get_policy().forward_quality_max_age_batches" in common and
    "rate_opt.get_policy().reverse_quality_max_age_batches" in common and
    ro.count("policy.forward_quality_max_age_batches") >= 2 and
    "policy.reverse_quality_max_age_batches" in ro,
    "quality freshness is tunable policy rather than duplicated magic constants")
req("temporal-regime-change-discounts-stale-evidence",
    allin(roh, "context_generation", "context_volatility") and
    allin(ro, "channel-regime-change", "context_generation"),
    "available SNR/selectivity/outcome change shortens stale authority")
req("failed-probe-negative-memory",
    allin(ro, "remember_failed_probe", "probe_target_blocked", "probe_cooldown_remaining") and
    "failed target is not immediately re-probed" in text("tests/cpu/test_gearshift_v2.cc"),
    "failed target cannot immediately burn airtime again")
req("channel-change-reopens-failed-probe-memory",
    allin(roh, "st_probe_memory", "context_generation") and
    allin(ro, "it->second.context_generation != context_generation", "probe_cooldown_remaining = 0") and
    "channel change reopens failed probe target" in text("tests/cpu/test_gearshift_v2.cc"),
    "negative probe evidence is scoped to the channel regime that produced it")
req("bounded-probe-has-explicit-fallback",
    allin(ro, "GEARSHIFT_ACTION_PROBE", "GEARSHIFT_ACTION_ROLLBACK", "probe_fallback_cfg"),
    "one bounded experiment + known rollback")
req("active-cold-start-acquisition-bypasses-four-outcome-gate",
    allin(ro, "cold_start_acquisition", "trustworthy_current_observation",
          "cold-start-ladder-probe", "preferred_probe_rung",
          "config_probe_ladder_up", "compatibility_ladder_acquisition") and
    allin(defines, "config_probe_ladder_up", "config == CONFIG_0", "return CONFIG_7",
          "config == CONFIG_7", "return CONFIG_13", "config == CONFIG_13",
          "return CONFIG_16") and
    "cold start probes next ladder rung before four outcomes" in core_test,
    "ACTIVE GS2 may start a bounded next-rung experiment after one trustworthy timed observation; the staging rung cannot be vetoed by uncertainty that only the experiment can resolve, and uncalibrated discovery cannot jump over the compatibility ladder")
req("cold-start-calibration-can-authorize-faster-direct-acquisition",
    "cold-start-calibrated-direct" in ro and
    "calibrated fast action can direct-switch on cold start" in core_test,
    "in-support empirical calibration remains first-class and can clear direct-switch confidence before no-table discovery")
req("ordinary-coast-is-risk-adjusted",
    allin(ro, "lcb > current_ucb * (1.0 + policy.direct_switch_margin)",
          "context_volatility", "context_volatility_alpha",
          "obs.frame_success_rate <= 0.0",
          "Partial/transient loss stays in the confidence/probation path",
          "lower_probe_ready", "lower-information-probe",
          "current_regime_model->context_generation == context_generation",
          "current_regime_model->application_samples >= policy.probe_min_application_samples",
          "current_regime_model->outcome_samples >= policy.probe_min_outcome_samples") and
    "lower-mode-net-goodput" not in ro and
    allin(core_test,
          "uncertain optimistic lower rung cannot coast against punished current estimate",
          "one failed and one recovered batch is not a full-failure coast",
          "confident lower rung still coasts when its LCB clears current UCB",
          "stable evidence decays transient volatility"),
    "ordinary economic coasts require target LCB to clear current UCB; after a full fresh-regime population, an uncertain safer rung may use the existing bounded probation lifecycle")
req("probe-probation-is-goodput-authoritative-and-channel-time-bounded",
    allin(roh, "probe_min_application_samples", "probe_min_outcome_samples",
          "probe_max_probation_ms", "probe_zero_progress_ms", "probe_ladder_step") and
    allin(ro, "probe_application_samples >= policy.probe_min_application_samples",
          "probe_outcome_samples >= policy.probe_min_outcome_samples",
          "ladder_goodput_confirm", "probe_failed_outcomes == 0",
          "fallback_after_rollback * policy.probe_rollback_ratio",
          "probation_timed_out", "goto ladder_probe_confirmed") and
    allin(cmd, "one-frame goodput/SACK confirmation batch armed",
          "native big-block goodput/SACK confirmation armed",
          "rate_opt.probe_is_ladder_step()", "set_data_batch_size(1)",
          "inband_confirm_coordinated_config(current_configuration)",
          "preserving queued control code=%d across post-block policy") and
    allin(common, "restored normal batch=%d",
          "ladder_probe_accepted_this_evaluation()",
          "normal-batch SET_LINK_PARAMS deferred",
          "add_message_control(SET_LINK_PARAMS)",
          "void cl_arq_controller::inband_confirm_coordinated_config",
          "redundant CONFIG_TAG suppressed") and
    allin(rsp, "inband_confirm_coordinated_config(current_configuration)") and
    allin(core_test, "bad first cfg16 sample does not rollback",
          "recovered cfg16 probe is accepted",
          "one clean goodput and SACK sample confirms a compatibility rung"),
    "a clean measured-goodput/SACK result may promptly confirm an explicit compatibility rung; all other probes retain aggregate evidence and every probe retains hard-failure/channel-time bounds")
req("lost-tag-probe-is-bounded-by-wire-batch",
    allin(rsp, "inband_down_probe_batch_seq_id != rsp_current_expected_batch_seq_id",
          "inband_down_probe_batch_seq_id = rsp_current_expected_batch_seq_id",
          "same-bsi fresh retry is bounded", "next bsi re-arms exactly one",
          "telecom_system->receive_stats.ofdm_preamble_detected",
          "telecom_system->receive_stats.iterations_done >= 0",
          "energy without a PHY-admitted OFDM preamble cannot fire",
          "preamble-only candidate without a payload FEC attempt cannot fire") and
    allin(telecom, "receive_stats.ofdm_preamble_detected=false",
          "receive_stats.ofdm_preamble_detected = true") and
    allin(common, "inband_down_probe_batch_seq_id = -1") and
    "inband_last_announced_config != current_configuration" not in rsp,
    "receiver lost-tag recovery may run one blind decoder-bank probe per active bsi; retransmission and repeated CONFIG_TAG own same-bsi recovery, and TX-only announcement state is never used as an RX predicate")
req("probe-hard-failure-is-distinct-from-soft-underperformance",
    allin(ro, "repeated_whole_failures", "established_failure_fraction",
          "probe-hard-failure", "probe-soft-underperformance") and
    allin(core_test, "single failed transaction stays in probation",
          "repeated failed probe rolls back", "established slower probe rolls back"),
    "repeated/established failure may retreat immediately while ordinary decoded underperformance must accumulate evidence")
req("probe-decision-uses-probe-local-aggregate",
    allin(roh, "probe_application_bps_sum", "probe_application_bps_ewma",
          "probe_channel_ms", "probe_failed_outcomes") and
    allin(ro, "probe_application_bps_sum / (double)probe_application_samples",
          "probe-probation", "probe-generation-reset"),
    "probation verdict uses causal target-local samples/failures/time and resets that population on a new channel generation")
req("uncalibrated-discovery-margin-default-is-three-percent",
    "probe_mean_margin(0.03)" in ro,
    "bounded discovery does not stack an extra 8% default margin on top of switch/probe economics")
req("weak-probe-memory-does-not-earn-progressive-backoff",
    allin(ro, "probe_soft_cooldown_batches", "strength=%s",
          "probation-insufficient-evidence") and
    "if (strong)" in ro,
    "ambiguous/time-limited evidence gets a short cooldown; only hard or established loss increments progressive failure memory")
req("probe-probation-budget-prices-target-geometry",
    allin(ro, "one_trial_ms", "desired_probation_ms", "probation_budget_ms",
          "probe_airtime_ms + feedback_ms", "std::min(policy.probe_max_probation_ms") and
    "CONFIG_16" in core_test,
    "long big-block/thin-grid targets get a channel-time budget scaled from their own airtime rather than raw batch count")
req("axis1-dispatch-is-exclusive-until-terminal-event",
    allin(ro, "if (switch_inflight)", "evaluation-suppressed", 'd.reason = "switch-inflight"',
          "switch-inflight-expired", "acquisition=reopened") and
    allin(core_test, "in-flight evaluation is suppressed",
          "confirmation terminates exclusive transition",
          "switch failure terminates exclusive transition",
          "expired in-flight transition reopens acquisition",
          "unconfirmed transition expiry does not blacklist target"),
    "an ordinary SET_CONFIG is exclusive while live, but a lost confirmation has a bounded terminal timeout that reopens acquisition without target-performance penalty")
req("duplicate-dispatch-cannot-overwrite-live-transaction",
    allin(ro, "duplicate-dispatch-rejected", "live_source=%d", "new_source=%d") and
    allin(core_test, "duplicate dispatch preserves original transaction",
          "original switch transaction is live"),
    "defensive dispatch handling preserves the first transition identity instead of silently replacing it")
req("probe-budget-is-bounded-cadence-adaptive-and-does-not-pre-veto-acquisition",
    allin(roh, "probe_budget_extra_cycles", "probe_budget_safety_factor",
          "probe_cycle_ms_ewma") and
    allin(ro, "required_population", "desired_probation_ms",
          "std::min(policy.probe_max_probation_ms",
          "maybe_extend_probe_budget", "probe-budget-extend",
          "observed-application-cadence") and
    "population_attainable" not in ro and
    allin(core_test, "production-sized feedback budget permits first cold ladder probe",
          "slow cfg16 survives beyond old 30-second ceiling",
          "observed cfg16 cadence extends probation budget",
          "healthy slow cfg16 accepted after requested population"),
    "target geometry/cadence seed a bounded probation budget; an oversized conservative feedback window caps probation instead of deadlocking acquisition before the target is measured")
req("generation-reset-excludes-triggering-transaction",
    allin(ro, "transaction_discovered_generation", "clean_probe_tx",
          "probe_pending_generation_contamination", "probe-generation-unit-excluded",
          "age_ms=0") and
    allin(core_test, "generation-trigger transaction is excluded from fresh probation",
          "next causally clean target transaction starts fresh generation"),
    "the transaction that discovers a new channel generation updates general evidence but contributes zero fresh probation population/time")
req("extended-probation-keeps-hard-failure-fast",
    allin(ro, "repeated_whole_failures", "established_failure_fraction",
          "zero_progress", "probe-hard-failure") and
    "hard failure still rolls back before extended probation budget" in core_test,
    "patient evidence collection does not weaken repeated-failure or meaningful-zero-progress rollback")
req("probe-transition-crossing-application-unit-is-not-a-probation-sample",
    "parts.size() == 1" in ro and
    "cross-config unit does not satisfy probe probation sample bar" in core_test,
    "an application unit spanning SET_CONFIG may update attribution but cannot satisfy the pure-target probation sample count")
req("expired-strong-probe-memory-reopens-as-probe-only-history",
    allin(ro, "post_failed_probe_historical", "+post-probe-historical",
          "m->last_tick < pmi->second.blocked_until_tick") and
    "no-telemetry positive regime change reopens faster probe" in closed,
    "after progressive backoff expires, a rejected episode remains measured history but cannot become a session-long veto or regain blind direct-switch authority")
req("probe-probation-idle-logging-is-bounded",
    allin(ro, "probe_result_ready || policy.trace_idle_decisions",
          "probe_result_ready = false") and "probe-accept" in ro,
    "state changes and evidence updates remain diagnosable without flooding logs on idle probation HOLD polls")

# One normal authority / one normal transition path.
req("active-v2-disables-connect-seed",
    "if(rate_opt.controls_link()) return CONFIG_NONE;" in cmd,
    "legacy connect seed is not a second ACTIVE authority")
req("active-v2-disables-turboshift",
    allin(common, "turboshift_active = !rate_opt.controls_link()", "TURBO_DONE"),
    "legacy climb state machine is inactive in ACTIVE")
req("active-v2-does-not-require-sack-v2",
    "if (!v2_active && !sack_v2_enabled)" in common and
    allin(cmd, "opt_record_batch((unsigned int)opt_clean_bytes_delivered",
          "opt_record_batch(/*bytes_delivered=*/0"),
    "ACTIVE can still learn clean/no-ACK goodput when selective-repeat is disabled; only legacy optimizer retains the SACK-v2 gate")
req("active-v2-disables-topgear-election",
    cmd.count("!rate_opt.controls_link()") >= 2 and "topgear_elect_feature_enabled()" in cmd,
    "both idle and ACK-seam topgear producers cannot independently move ACTIVE")
req("active-v2-uses-config-tag-transport-without-second-selector",
    allin(common, "if(rate_opt.controls_link()) return true",
          "CONFIG_TAG is transport/synchronization",
          "ACTIVE decides *where* to go; CONFIG_TAG decides *how the") and
    allin(cmd, "SET_CONFIG is legacy and is reserved here for two transport exceptions",
          "SPECIAL_CASE=CROSS_TIER", "SPECIAL_CASE=TAG_UNAVAILABLE",
          "Every ordinary intra-tier GS2 probe, switch, rollback, and coast-down",
          "else if(inband_unilateral_config_change(inband_target))",
          "NO control frame on the wire") and
    "coordinated_probe" not in cmd and "coordinated_coastdown" not in cmd and
    allin(common, "inband_unilateral_config_change(int target_cfg)",
          "load_configuration(data_configuration, PHYSICAL_LAYER_ONLY, YES)",
          "inband_retag_armed   = true",
          "LOCAL load_configuration() is not peer-follow evidence") and
    allin(cmd, "bool optimizer_owns_upward_frame = optimizer_is_in_control()",
          "!optimizer_owns_upward_frame"),
    "Gearshift-v2 remains the sole ordinary Axis-1 selector and CONFIG_TAG is the canonical transport for ordinary intra-tier upward probes and downward coasts")
req("active-v2-absorbs-emergency-nack-as-controller-input",
    allin(roh, "consume_failure_signal(int current_cfg", "owns_coastdown_transition") and
    allin(ro, "cl_rate_optimizer::consume_failure_signal(",
          "config_ladder_down(current_cfg, robust_enabled)",
          "failure-coastdown:", "GEARSHIFT-V2-COASTDOWN") and
    allin(cmd, "emergency_nack_threshold", "rate_opt.consume_failure_signal(",
          "owner_target", '"emergency_nack_threshold", true, true') and
    allin(text("tests/cpu/test_rate_optimizer.cc"),
          "GS2 failure signal chooses one lower rung",
          "GS2 owns coast-down transaction",
          "failure during probe returns to owner fallback"),
    "existing block-failure/NACK detection is telemetry input; ACTIVE GS2 chooses and owns the coast-down destination")
req("active-v2-owned-coastdown-uses-canonical-config-tag",
    allin(cmd, "Every ordinary intra-tier GS2 probe, switch, rollback, and coast-down",
          "else if(inband_unilateral_config_change(inband_target))") and
    "transport=CONFIG_TAG_CANONICAL" in ro and
    allin(rsp, "B7.4 ordinary owned coast-down emits no legacy SET_CONFIG control frame",
          "B7.5 ordinary owned coast-down arms canonical CONFIG_TAG transport",
          "B7.6 CONFIG_TAG coast commits locally and repeats until peer confirmation") and
    allin(text("tests/cpu/test_rate_optimizer.cc"),
          "ordinary GS2 downshift is an owned canonical-tag coast-down") and
    allin(ro, "bool cl_rate_optimizer::owns_coastdown_transition",
          "return transition_matches(from_cfg, to_cfg) &&",
          "gearshift_action_rank(to_cfg) < gearshift_action_rank(from_cfg)"),
    "every ordinary GS2-owned downward move, including emergency rollback, uses canonical repeat-until-confirmed CONFIG_TAG")
req("active-v2-upward-probe-uses-canonical-config-tag",
    allin(rsp,
          "A0.1 ACTIVE probe commits locally through canonical CONFIG_TAG",
          "A0.2 ACTIVE probe queues no legacy SET_CONFIG control frame",
          "A0.4 canonical probe arms repeat-until-confirmed CONFIG_TAG state",
          "A0.6 config-discriminating SACK closes tag transition and begins probation"),
    "ordinary upward discovery uses CONFIG_TAG and cannot be locally mistaken for peer confirmation")
req("active-v2-setconfig-special-cases-are-enumerated",
    allin(cmd, "SPECIAL_CASE=CROSS_TIER", "dedicated ACK bridges robust<->OFDM acquisition",
          "SPECIAL_CASE=TAG_UNAVAILABLE", "NB/M<16 exposes no CONFIG_TAG carrier") and
    "coordinated_probe" not in cmd and "coordinated_coastdown" not in cmd,
    "legacy SET_CONFIG is reserved for cross-tier acquisition and configurations without a tag carrier, each with its technical reason")
req("preframe-config-tag-is-not-gated-on-old-phy-geometry",
    allin(common, "Do NOT publish", "pre-frame snapshot", "still-loaded OLD geometry",
          "trailing/capture path continues to publish",
          "POST-FOLLOW target-geometry gate", "CONFIG_TAG processing guard",
          "larger of the emitted tag and one target", "acquisition_samples") and
    "inband_adopt_gate_snapshot     = snapshot" not in common and
    allin(rsp, "exact gapless", "with no synthetic", "A0 RX FOLLOWS the pre-frame tag"),
    "a CRC-valid transition is followed before target-frame decode; guard time is derived from wire/acquisition geometry and the saved snapshot is then judged only at the new geometry")
req("active-v2-config-tag-transition-closes-from-peer-evidence",
    allin(common, "inband_retag_confirm_from_sack(int rx_bsi)",
          "config-discriminating SACK at/after the announce BSI",
          "rate_opt.notify_switch_confirmed_if_matches",
          "No config-discriminating follow evidence arrived after the bounded re-tags",
          "rate_opt.notify_switch_failed_if_matches",
          "inband_handle_nack(uint8_t rx_cfg_index",
          "explicit evidence that this CONFIG_TAG transition failed") and
    allin(cmd, "bool tier_crossing = inband_config_change_is_tier_crossing(inband_target)",
          "if(tier_crossing)", "else if(tag_transport_unavailable)",
          "else if(inband_unilateral_config_change(inband_target))",
          "NO control frame on the wire") and
    allin(text("tests/cpu/test_gearshift_v2.cc"),
          "mismatched peer confirmation is ignored",
          "matching peer confirmation closes the live transition",
          "mismatched peer failure is ignored",
          "matching peer failure closes the live transition"),
    "ACTIVE ordinary intra-tier CONFIG_TAG transitions remain in-flight until matching peer follow proves the selected target or matching failure evidence terminates it; only enumerated transport exceptions close through SET_CONFIG ACK")
req("active-v2-owns-ordinary-ladder-down",
    "GEARSHIFT_V2_ACTIVE" in arqh and "optimizer_owns_normal_downshift" in arqh and
    "return true;  // v2 owns ordinary downshift" in arqh,
    "legacy success ladder cannot move ACTIVE")
req("v2-normal-transition-runs-axis-invalidation",
    allin(cmd, "policy_axis1_supremacy_on_move(from_cfg, target", "notify_switch_dispatched"),
    "normal v2 SET_CONFIG passes transition invariants")
req("universal-setconfig-chokepoint-synchronizes-external-moves",
    allin(cmd, "set-config-chokepoint", "notify_external_axis1_transition",
          "forward_configuration != current_configuration") and
    allin(text("tests/cpu/test_gearshift_v2.cc"),
          "own SET_CONFIG is not external override",
          "external safety move is synchronized",
          "external safety move cancels probe"),
    "every connected on-wire SET_CONFIG synchronizes v2; matching v2 moves are idempotent")
req("every-axis1-transition-enforces-lower-axis-reset-exactly-once",
    allin(arqh, "axis1_supremacy_prepared", "axis1_supremacy_prepared_from",
          "axis1_supremacy_prepared_to") and
    allin(cmd, "Universal Axis-1 transition contract", "set-config-chokepoint",
          "const bool prepared = axis1_supremacy_prepared") and
    allin(common, "inband-chokepoint", "axis1_supremacy_prepared"),
    "normal, safety, BREAK-recovery and in-band Axis-1 moves cannot bypass or double-run Axis-2/3 invalidation")
req("emergency-break-synchronizes-v2",
    allin(common, "void cl_arq_controller::send_break_pattern()",
          "notify_external_axis1_transition", "emergency-break"),
    "BREAK cannot leave a v2 probe/switch context alive across emergency recovery")
req("probe-transport-failure-has-negative-memory",
    allin(ro, "notify_switch_failed", "blocked_until_tick", "probe_cooldown_remaining") and
    "failed probe transport is remembered" in text("tests/cpu/test_gearshift_v2.cc"),
    "an unconfirmed probe transition cannot be re-attempted immediately")
req("break-remains-independent-emergency",
    allin(common, "emergency_break_active", "BREAK") and
    "if (emergency_break_active != 0)" in common,
    "v2 yields to no-progress recovery")
req("known-no-ack-failure-is-evaluated-by-active-v2",
    allin(cmd, "if(rate_opt.controls_link())", "if(opt_evaluate_batch_end(&rec))",
          "ACTIVE v2 owns ordinary performance adaptation"),
    "known-zero forward outcomes can trigger a normal safer action before BREAK is required")
req("cooldown-advances-once-per-logical-transaction",
    "rate_opt.notify_cooldown_tick();" in arqh and
    "rate_opt.notify_cooldown_tick();" not in common and
    "rate_opt.notify_cooldown_tick();" not in cmd,
    "multiple evaluate() calls for one ACK/app commit cannot drain switch/probe cooldown twice")

# Explainability + closed-loop acceptance.
req("hold-and-abstain-are-distinct",
    allin(roh, "GEARSHIFT_ACTION_HOLD", "GEARSHIFT_ACTION_ABSTAIN") and
    allin(ro, '"learning-outcomes"', '"gain-does-not-pay"'),
    "lack of evidence is not disguised as optimal HOLD")
req("structured-decision-trace",
    allin(ro, "[GEARSHIFT-V2]", "target_pred", "horizon_ms", "reason=%s",
          "MERCURY_GS2_TRACE_IDLE") and
    allin(roh, "last_decision", "trace_idle_decisions"),
    "state-changing decisions are always logged; HOLD/ABSTAIN remain structured state and optional verbose trace without production I/O tax")
req("closed-loop-stationary-and-step-tests",
    allin(closed, "fixed-mode oracle", "upward channel step", "downward channel step", "regret") and
    "test_gearshift_v2_closed_loop" in run,
    "controller, model and switching feed one another in deterministic loop")
req("no-snr-outcome-change-test",
    "no-SNR" in closed and "outcome-driven regime change" in closed,
    "fading/change detection is not dependent on SNR telemetry")
req("uncalibrated-upward-actions-are-probe-only-until-directly-evidenced",
    allin(ro, "direct_evidence", "analytic-uncalibrated-prior", "ladder-information-probe") and
    "No-calibration + no-channel-telemetry closed loop" in closed,
    "analytical capacity discovers faster actions without being mistaken for measured channel performance")
req("negative-regime-change-suppresses-immediate-upward-experiment",
    allin(ro, "upward_probe_suppressed_until_tick", "direction=", "worse") and
    "no-telemetry negative regime change moves safer" in closed,
    "a newly degraded channel first considers safer actions instead of immediately probing faster")
req("failed-probe-backoff-is-target-specific",
    allin(ro, "remember_failed_probe", "policy.cooldown_batches") and
    "probe_target_blocked" in ro,
    "one failed experiment does not freeze exploration of every other candidate")

# CP4 semantic-hardening requirements.  Each one pairs production evidence with
# a directed behavioral regression so source-token presence alone is insufficient.
req("selectivity-freshness-is-independent-end-to-end",
    allin(roh, "forward_selectivity_age_batches", "observe_transaction") and
    allin(arqh, "gearshift_forward_selectivity_age_batches, opt_now_ms()") and
    "fresh selectivity changes context with stale SNR" in core_test,
    "fresh selectivity can change channel generation even when forward SNR is stale")
req("nb-only-calibration-is-valid",
    allin(ro, "valid_wb = 0", "valid_nb = 0", "calibration_loaded = valid_wb > 0 || valid_nb > 0") and
    "NB-only calibration loads" in core_test,
    "the calibrator's table_nb-only output is consumable without dummy WB cells")
req("nullable-rate-fields-remain-unknown",
    allin(ro, "json_null_at(body, field_v)", "eff_bps_present") and
    "null-rate file has no valid calibration cell" in core_test,
    "JSON null performance fields cannot become fabricated zero-goodput measurements")
req("mismatched-calibration-loses-direct-authority",
    allin(ro, "calibration_prior_sigma_scale", "calibration_direct_authority", "identity-mismatch-probe-only") and
    "identity-mismatched calibration is probe-only" in core_test,
    "configuration-identity mismatch widens uncertainty and cannot authorize a cold direct jump")
req("calibration-extrapolation-is-not-direct-evidence",
    allin(ro, "snr_extrapolation_db", "+extrapolated", "support_distance") and
    "out-of-support SNR is not direct evidence" in core_test,
    "outside measured SNR/selectivity support priors weaken rather than edge-clamp with full authority")
req("candidate-live-context-uses-candidate-geometry",
    allin(ro, "candidate_batch_size.find(cfg)", "candidate_batch") and
    "candidate batch geometry owns live relevance" in core_test,
    "historical evidence for a candidate is matched against that candidate's batch geometry")
req("compression-gain-is-atomic-unit-derived",
    allin(roh, "atomic_application_transport_gain", "atomic_application_transport_gain_valid") and
    allin(ro, "total_transport", "app-unit-converted-atomic") and
    "atomic gain overrides misleading rolling ratio" in core_test,
    "SET_CONFIG window resets cannot inflate application/transport conversion")
req("live-evidence-ages-during-idle-wall-time",
    allin(roh, "monotonic_ms", "observation_ms") and
    allin(common, "obs.monotonic_ms = opt_now_ms()") and
    "idle wall time ages live evidence" in core_test,
    "freshness follows monotonic channel time rather than completed DATA duration only")
req("action-ranking-cannot-be-masked-by-unactionable-best-mean",
    allin(ro, "st_action_proposal", "utility_bps", "saw_uneconomic_probe") and
    "action ranking falls through to profitable evidenced switch" in core_test,
    "an uneconomic speculative candidate cannot hide a profitable admissible action")
req("known-zero-work-is-distinct-from-unknown-horizon",
    allin(roh, "remaining_work_known") and
    allin(common, "obs.remaining_work_known = true") and
    allin(ro, "obs.remaining_work_known || obs.queue_bytes > 0") and
    "known-zero remaining work has finite near-zero horizon" in core_test,
    "zero remaining bytes no longer means an unknown/streaming default horizon")
req("post-commit-evaluation-excludes-delivered-unit",
    allin(cmd, "committed_application_bytes", "batch_uncompressed_size = 0", "opt_evaluate_batch_end(&rec)") and
    cmd.find("batch_uncompressed_size = 0", cmd.find("const int committed_application_bytes")) <
        cmd.find("opt_evaluate_batch_end(&rec)", cmd.find("const int committed_application_bytes")),
    "the just-ACKed application unit is removed before remaining-work switch economics")
req("newer-active-evaluation-revokes-stale-pending-action",
    allin(common, "An ACTIVE recommendation is a revocable snapshot",
          "opt_pending_switch_cfg = -1", "opt_pending_switch_action = GEARSHIFT_ACTION_HOLD") and
    common.find("opt_pending_switch_cfg = -1", common.find("An ACTIVE recommendation is a revocable snapshot")) <
        common.find("st_rate_observation obs", common.find("An ACTIVE recommendation is a revocable snapshot")),
    "a post-commit HOLD can cancel an ACK-seam recommendation that used now-delivered bytes")
req("active-cfg17-dispatch-is-independent-of-legacy-election-state",
    allin(cmd, "rate_opt.controls_link()", "topgear_elect_feature_enabled() ? CONFIG_17 : WB_CONFIG_MAX",
          "legacy topgear") and
    "v2 topgear cfg17 is reachable" in core_test,
    "ACTIVE cfg17 execution obeys explicit administrative opt-in, not legacy topgear_elect_engaged")

# CP5 minimal forward-quality telemetry: reuse the existing compact-confirm tail;
# do not create a second controller or make telemetry a prerequisite for GS2.
req("generic-forward-quality-report-is-independent-of-topgear",
    allin(common, "gearshift_quality_report_v2_enabled", "MERCURY_GS2_QUALITY_REPORT",
          "gearshift_quality_report_v2_enabled() || topgear_elect_feature_enabled()") and
    allin(common, "this->role == RESPONDER", "gearshift_quality_report_v2_enabled() || topgear_elect_feature_enabled()",
          "last_channel_selectivity"),
    "forward SNR/selectivity transport no longer depends on permission to use CONFIG_17")
req("compact-quality-report-carries-quantized-snr-and-selectivity",
    allin(quality, "gearshift_quality_pack", "gearshift_quality_unpack_snr",
          "gearshift_quality_unpack_selectivity", "GEARSHIFT_QUALITY_SELECTIVITY_UNKNOWN") and
    allin(quality_test, "SNR quantizes down", "selectivity quantizes conservatively up",
          "unknown selectivity reserved"),
    "the one-byte advisory hint encodes conservative forward SNR plus real selectivity with an unknown code")
req("quality-report-is-change-or-refresh-rate-limited",
    allin(quality, "report != previous_report", "refresh_batches", "age >=") and
    allin(common, "MERCURY_GS2_QUALITY_REFRESH_BATCHES", "gearshift_quality_report_due_for_tx") and
    allin(quality_test, "unchanged report suppressed before refresh",
          "unchanged report refreshes at age", "quantized change sends immediately"),
    "unchanged telemetry does not tax every ACK; material quantized changes send immediately and stable evidence refreshes periodically")
req("quality-report-reuses-existing-compact-confirm-tail",
    allin(common, "generate_topgear_confirm_passband", "quality_report", "+gs2-quality") and
    allin(cmd, "gearshift_quality_report_transport_enabled()", "quality_report_valid"),
    "GS2 adds no standalone telemetry packet or new estimator; it reuses the CRC-protected optional compact-confirm codeword")
req("rate-limited-quality-tail-keeps-deferred-consume-path",
    allin(cmd, "want_quality_report", "topgear_pending_report_arm(target)",
          "gearshift_quality_report_transport_enabled()", "topgear_pending_stash_refresh") and
    allin(cmd, "gearshift_apply_quality_report(report, resolved_target)",
          'topgear_pending_report_clear(\"applied\")'),
    "ACK acceptance remains on the compact prefix while a present trailing GS2 report may finish and decode off the critical path")
req("quality-telemetry-remains-advisory-not-required",
    "No-calibration + no-channel-telemetry closed loop" in closed and
    "no-SNR" in closed and "MERCURY_GS2_QUALITY_REPORT" in common,
    "outcomes/goodput remain authoritative and Gearshift still adapts when the advisory quality hint is absent")
req("legacy-topgear-report-format-remains-available",
    allin(common, "legacy_topgear", "topgear_pack_report", "!v2_quality") and
    "Legacy mode is byte-for-byte on its old" in common,
    "LEGACY deployments retain the historical topgear report format/cadence while GS2 uses the generic hint")

req("quality-report-diagnostics-are-opt-in",
    common.count("MERCURY_GS2_QUALITY_TRACE") >= 1 and
    cmd.count("MERCURY_GS2_QUALITY_TRACE") >= 2,
    "rate-limited telemetry and deferred-tail handling do not add unconditional per-batch printf/fflush overhead")

external_only = [
    ("held-out-IONOS/RF-superiority", "requires the occupied physical IONOS/RF bench"),
    ("true-remote-per-subcarrier-temporal-coherence", "requires negotiated remote CSI telemetry not present on the current wire; code uses available SNR/selectivity/outcome dynamics instead"),
    ("successful-full-platform-GUI-build", "requires the normal development environment with glfw/audio dependencies; changed TUs are syntax-checked separately"),
]

failed = False
for name, ok, ev in checks:
    print(("PASS " if ok else "FAIL ") + f"{name}: {ev}")
    failed |= not ok
for name, why in external_only:
    print(f"EXTERNAL_ONLY {name}: {why}")

if failed:
    sys.exit(1)
print(f"PASS Gearshift-v2 recursive spec audit: {len(checks)} code-addressable requirements")
