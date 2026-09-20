/*
 * Mercury: A configurable open-source software-defined modem.
 * Copyright (C) 2022-2026 Fadi Jerji
 * Author: Fadi Jerji
 * Email: fadi.jerji@  <gmail.com, caisresearch.com, ieee.org>
 * ORCID: 0000-0002-2076-5831
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, version 3 of the
 * License.
 *
 * Gearshift v2 / effective-goodput optimizer.
 *
 * The calibration table is a PRIOR.  Live observations update each
 * configuration independently; they never uniformly scale every action.
 * The controller is deliberately self-contained so CPU tests can exercise the
 * real decision policy without constructing cl_arq_controller.
 */

#ifndef RATE_OPTIMIZER_H_
#define RATE_OPTIMIZER_H_

#include <cstdint>
#include <map>
#include <string>
#include <vector>

// One calibrated cell: per-(config, channel) measured useful bps plus its
// uncertainty and failure evidence.  Legacy tables are accepted; missing
// fields receive conservative defaults.
struct st_rate_cell {
    double eff_bps_mean;
    double eff_bps_sigma;
    double sack_rate_mean;
    bool   sack_rate_valid;
    double partial_loss_mean;
    double failed_batch_rate_mean;
    double frame_success_mean;
    double selectivity_mean;
    double batch_size_mean;
    double break_run_rate;
    double snr_hint_db;
    bool   eff_bps_present;
    bool   partial_loss_valid;
    bool   failed_batch_rate_valid;
    bool   frame_success_valid;
    bool   selectivity_valid;
    bool   batch_size_valid;
    bool   break_run_rate_valid;
    int    n_runs;
    int    n_failed_runs;
    int    n_zero_delivery_runs;
    int    n_invalid_runs;
    bool   v2_all_valid_runs;
    bool   failed;
    bool   break_fired;
    bool   valid;

    st_rate_cell();
    int total_runs() const;
    double survival_adjusted_mean() const;
};

enum e_gearshift_v2_mode {
    GEARSHIFT_V2_LEGACY = 0,
    GEARSHIFT_V2_SHADOW = 1,
    GEARSHIFT_V2_ACTIVE = 2
};

enum e_gearshift_v2_action {
    GEARSHIFT_ACTION_HOLD = 0,
    GEARSHIFT_ACTION_SWITCH = 1,
    GEARSHIFT_ACTION_PROBE = 2,
    GEARSHIFT_ACTION_ROLLBACK = 3,
    GEARSHIFT_ACTION_ABSTAIN = 4
};

struct st_rate_observation {
    int current_cfg;
    double application_bps;
    double transport_bps;
    double sack_batch_rate;
    double frame_success_rate;
    double partial_frame_loss_rate;
    double failed_batch_rate;
    int rate_samples;
    int outcome_samples;
    int application_commits;
    double forward_snr_db;
    int forward_snr_age_batches;
    double forward_selectivity;
    int forward_selectivity_age_batches;
    // Separately tagged locally-measured reverse path.  It is never silently
    // substituted for forward quality; at cold start it may be used only as a
    // deliberately weak symmetry prior with inflated uncertainty.
    double reverse_snr_db;
    int reverse_snr_age_batches;
    int batch_size;
    // Remaining original application bytes known to the caller.  queue_bytes=0
    // is meaningful only when remaining_work_known=true; otherwise it means the
    // workload horizon is unknown/streaming.
    long queue_bytes;
    bool remaining_work_known;
    // Monotonic wall-clock timestamp from the modem.  Zero keeps the deterministic
    // transaction-duration clock used by standalone CPU tests.
    unsigned long long monotonic_ms;
    bool is_nb;
    // Calibration is normally collected with modem compression OFF (raw PHY
    // transport goodput), while production may have streaming compression ON.
    // application_transport_gain converts a raw transport-rate prior into the
    // same application-byte units as the live objective once a real completed
    // application unit has established that ratio.
    bool compression_enabled;
    double application_transport_gain;
    bool application_transport_gain_valid;

    // Feasible action set and analytical PHY hints supplied by cl_arq_controller.
    // These make uncalibrated configurations real actions instead of invisible
    // holes in the calibration table. nominal_bps is a no-channel-loss capacity
    // hint; tx_airtime_ms is predicted keydown for one batch at the candidate.
    std::vector<int> feasible_configs;
    std::map<int, double> nominal_bps;
    std::map<int, double> tx_airtime_ms;
    std::map<int, int> candidate_batch_size;
    // Conservative feedback/timeout exposure for a bounded upward probe. Candidate
    // DATA airtime alone is not the full cost when an unworkable mode burns the
    // receive window waiting for an ACK that never arrives.
    double feedback_budget_ms;

    st_rate_observation();
};

struct st_rate_prediction {
    bool valid;
    double mean_bps;
    double sigma_bps;
    double prior_weight;
    double live_weight;
    int live_samples;
    int age_batches;
    double age_ms;
    bool direct_evidence;
    std::string source;

    st_rate_prediction();
};

struct st_rate_decision {
    e_gearshift_v2_action action;
    int current_cfg;
    int target_cfg;
    int fallback_cfg;
    double current_mean_bps;
    double current_sigma_bps;
    double target_mean_bps;
    double target_sigma_bps;
    double net_target_bps;
    double switch_cost_ms;
    double horizon_ms;
    bool actionable;
    std::string reason;

    st_rate_decision();
};

// Centralized policy parameters.  Values may be overridden by MERCURY_GS2_*
// environment variables; they are intentionally not scattered through the
// control path as magic constants.
struct st_rate_policy {
    int min_outcome_samples;
    int min_rate_samples;
    int cooldown_batches;
    int probe_cooldown_batches;
    int probe_soft_cooldown_batches;
    int probe_min_application_samples;
    int probe_min_outcome_samples;
    int probe_hard_failure_streak;
    int probe_budget_extra_cycles;
    int forward_quality_max_age_batches;
    int reverse_quality_max_age_batches;
    int trace_idle_decisions;
    double direct_switch_margin;
    double probe_mean_margin;
    double probe_rollback_ratio;
    double probe_max_probation_ms;
    double probe_zero_progress_ms;
    double probe_budget_safety_factor;
    // An ordinary Axis-1 transition is exclusive, but never immortal. If the
    // SET_CONFIG confirmation/failure callback is lost, ACTIVE must eventually
    // reopen acquisition rather than strand the session at the source config.
    double switch_inflight_timeout_ms;
    double confidence_z;
    double failure_direct_threshold;
    double prior_default_rel_sigma;
    double prior_min_sigma_bps;
    double prior_run_weight_cap;
    double live_weight_cap;
    // Evidence freshness is fundamentally time-based: one cfg16 batch and one
    // robust batch do not represent the same amount of channel time.  The
    // batch half-life is retained only for compatibility/probe bookkeeping.
    double live_stale_half_life_batches;
    double live_stale_half_life_ms;
    double online_ewma_alpha;
    double selectivity_uncertainty_gain;
    double calibration_snr_kernel_db;
    double calibration_selectivity_kernel;
    double calibration_context_distance_sigma_gain;
    double compression_gain_rel_sigma;
    double batch_context_half_width;
    double uncalibrated_prior_rel_sigma;
    double uncalibrated_prior_weight;
    double probe_reopen_snr_db;
    double probe_reopen_selectivity;
    double channel_change_snr_db;
    double channel_change_selectivity;
    double reverse_snr_prior_weight_scale;
    double reverse_snr_sigma_gain;
    double context_volatility_alpha;
    double context_volatility_live_gain;
    double context_volatility_horizon_gain;
    double outcome_change_rel_threshold;
    double outcome_change_sigma_threshold;
    int outcome_change_min_samples;
    double default_horizon_ms;
    double min_horizon_ms;
    double max_horizon_ms;
    double default_switch_cost_ms;

    st_rate_policy();
    void load_env();
};

struct st_online_rate_model {
    bool initialized;
    double ewma_application_bps;
    double ewma_transport_bps;
    double ewma_variance;
    double last_application_sample_bps;
    double pending_cycle_ms;
    double pending_transport_bytes;
    double failure_ewma;
    double sack_ewma;
    double frame_success_ewma;
    double partial_loss_ewma;
    double batch_size_ewma;
    int application_samples;
    int outcome_samples;
    int consecutive_failures;
    int last_tick;
    unsigned long long last_observation_ms;
    int context_generation;

    st_online_rate_model();
};

struct st_probe_memory {
    int blocked_until_tick;
    double failed_snr_db;
    double failed_selectivity;
    int failures;
    int context_generation;

    st_probe_memory()
        : blocked_until_tick(0), failed_snr_db(-99.9),
          failed_selectivity(-1.0), failures(0), context_generation(-1) {}
};

class cl_rate_optimizer {
public:
    cl_rate_optimizer();

    bool load(const char* path);
    // Gearshift-v2 remains available without calibration. Legacy mode still
    // naturally becomes inert when there are no calibrated cells.
    bool is_enabled() const { return enabled; }
    bool has_calibration() const { return calibration_loaded; }
    int context_generation_for_test() const { return context_generation; }
    double atomic_application_transport_gain_for_test() const { return atomic_application_transport_gain; }
    bool atomic_application_transport_gain_valid_for_test() const { return atomic_application_transport_gain_valid; }

    // Mode control.  MERCURY_GEARSHIFT_V2 accepts legacy, shadow, or active.
    void configure_mode_from_env();
    void set_mode_for_test(e_gearshift_v2_mode m) { mode = m; }
    e_gearshift_v2_mode get_mode() const { return mode; }
    bool controls_link() const { return mode == GEARSHIFT_V2_ACTIVE; }
    bool shadow_only() const { return mode == GEARSHIFT_V2_SHADOW; }
    const char* mode_name() const;

    // ACTIVE-v2 ownership firewall.  Legacy ARQ subsystems may detect and report
    // failures, but they do not own Axis-1 or hard-recovery decisions.  A live
    // switch/probe transaction is an exclusive experiment owned by Gearshift-v2.
    bool owns_link_experiment(unsigned long long now_ms = 0) const;
    bool transition_matches(int from_cfg, int to_cfg) const;
    int authorize_external_transition(int from_cfg, int requested_to_cfg,
                                      const char* reason,
                                      unsigned long long now_ms,
                                      bool is_nb = false);
    // Consume a detector's failure signal without accepting its destination.
    // ACTIVE chooses the coast-down rung and opens the authoritative rollback
    // transaction that the ARQ layer transports losslessly.
    int consume_failure_signal(int current_cfg, const char* reason,
                               unsigned long long now_ms, bool is_nb,
                               bool robust_enabled);
    // Every ACTIVE downward transition is a coordinated coast-down, regardless
    // of whether its policy action is SWITCH or ROLLBACK.
    bool owns_coastdown_transition(int from_cfg, int to_cfg) const;
    bool authorize_hard_recovery(int current_cfg, bool at_bottom,
                                 const char* reason) const;

    // Primitive, non-overlapping live evidence.  cycle_ms=0 means the outcome
    // is valid but no independent rate interval is available (e.g. L1 aggregate
    // replay).  application bytes may be credited later by
    // add_application_credit() when an atomic compressed unit commits.
    void observe_transaction(int cfg,
                             unsigned int application_bytes,
                             unsigned int transport_bytes,
                             unsigned int cycle_ms,
                             unsigned int frames_acked,
                             unsigned int frames_sent,
                             bool sack_used,
                             bool failed,
                             double forward_snr_db,
                             int forward_snr_age_batches,
                             double forward_selectivity,
                             int batch_size,
                             bool is_nb,
                             int forward_selectivity_age_batches = -1,
                             unsigned long long observation_ms = 0);
    void add_application_credit(int cfg, unsigned int application_bytes, bool is_nb);
    // Finish one atomic application unit.  Original application bytes are
    // distributed over every config that contributed unique banked transport
    // to the unit; failed zero-progress cycles therefore score zero instead of
    // disappearing or all credit landing on the final config.
    void commit_application_unit(int final_cfg, unsigned int application_bytes, bool is_nb);

    // Main v2 decision interface.
    st_rate_decision evaluate_v2(const st_rate_observation& obs,
                                 int config_ceiling);
    const st_rate_decision& last_decision() const { return last_v2_decision; }

    // Legacy compatibility entry point.  In legacy mode this runs the original
    // table-label controller.  In shadow/active mode it constructs a minimal v2
    // observation and returns the v2 target; the full controller uses
    // evaluate_v2() directly.
    int evaluate(int current_cfg,
                 double current_eff_bps,
                 double current_sack_rate,
                 int window_count,
                 int config_ceiling,
                 bool is_nb = false,
                 double current_partial_loss_rate = -1.0);

    // Switch/probe lifecycle. The controller calls dispatched exactly once when
    // an Axis-1 move is committed to a transport. Legacy SET_CONFIG closes from
    // its ACK; CONFIG_TAG closes only from peer-follow evidence. The target-aware
    // helpers reject stale/mismatched terminal evidence instead of accidentally
    // closing whatever transaction happens to be live.
    void notify_switch_dispatched(int from_cfg,
                                  int to_cfg,
                                  e_gearshift_v2_action action,
                                  int fallback_cfg,
                                  unsigned long long now_ms,
                                  bool is_nb = false);
    void notify_switch_confirmed(unsigned long long now_ms);
    void notify_switch_failed();
    bool notify_switch_confirmed_if_matches(int from_cfg, int to_cfg,
                                            unsigned long long now_ms);
    bool notify_switch_failed_if_matches(int from_cfg, int to_cfg);
    // A config change initiated outside Gearshift-v2 (emergency recovery,
    // implementation safety demote, etc.) must invalidate any in-flight probe
    // and age the model context. Matching v2-dispatched SET_CONFIGs are ignored
    // here so the universal SET_CONFIG chokepoint can call this unconditionally.
    bool notify_external_axis1_transition(int from_cfg, int to_cfg,
                                          const char* reason,
                                          bool is_nb = false);

    void notify_cooldown_tick();
    bool cooldown_active() const { return cooldown_remaining > 0; }
    int min_window_samples() const { return policy.min_outcome_samples; }
    void reset_session_state();

    void set_hysteresis_ratio(double r) { legacy_hysteresis_ratio = r; }
    void set_cooldown_batches(int n) { policy.cooldown_batches = n; }
    void set_switch_cost_ms(int ms);
    void set_wire_ms_per_batch(double ms);
    void set_policy_for_test(const st_rate_policy& p) { policy = p; }
    const st_rate_policy& get_policy() const { return policy; }

    void force_cooldown(int batches);

    int min_calibrated_cfg(bool is_nb = false) const;
    double max_calibrated_sack_rate(bool is_nb = false) const;
    double max_calibrated_partial_loss(bool is_nb = false) const;

    // Test/diagnostic accessors.
    st_rate_prediction predict_for_test(int cfg,
                                        const st_rate_observation& obs) const;
    bool probe_is_active() const { return probe_active; }
    bool probe_is_ladder_step() const { return probe_active && probe_ladder_step; }
    bool ladder_probe_accepted_this_evaluation() const {
        return ladder_probe_accepted_now;
    }
    int probe_application_samples_for_test() const { return probe_application_samples; }
    int probe_outcome_samples_for_test() const { return probe_outcome_samples; }
    double probe_mean_bps_for_test() const {
        return probe_application_samples > 0
            ? probe_application_bps_sum / (double)probe_application_samples : 0.0;
    }
    double probe_channel_ms_for_test() const { return probe_channel_ms; }
    double probe_max_channel_ms_for_test() const { return probe_max_channel_ms; }
    bool switch_inflight_for_test() const { return switch_inflight; }
    int switch_from_cfg_for_test() const { return switch_from_cfg; }
    int switch_to_cfg_for_test() const { return switch_to_cfg; }
    e_gearshift_v2_action switch_action_for_test() const { return switch_action; }
    bool probe_blocked_for_test(int cfg, const st_rate_observation& obs) const {
        return probe_target_blocked(cfg, obs);
    }
    int measured_switch_cost_ms() const;
    double context_volatility_for_test() const { return context_volatility; }

private:
    bool enabled;
    bool calibration_loaded;
    double calibration_prior_weight_scale;
    double calibration_prior_sigma_scale;
    bool calibration_direct_authority;
    std::string calibration_build_id;
    std::string calibration_config_signature;
    bool calibration_compress_known;
    bool calibration_compressed;
    bool nb_enabled;
    int n_configs_loaded;
    int n_channels_loaded;
    int n_configs_loaded_nb;
    int n_channels_loaded_nb;

    std::map<int, std::map<std::string, st_rate_cell> > table;
    std::map<int, std::map<std::string, st_rate_cell> > table_nb;
    std::vector<std::pair<std::string, double> > channel_axis;
    std::vector<std::pair<std::string, double> > channel_axis_nb;

    std::map<int, st_online_rate_model> online_wb;
    std::map<int, st_online_rate_model> online_nb;
    std::map<int, st_probe_memory> probe_memory_wb;
    std::map<int, st_probe_memory> probe_memory_nb;

    e_gearshift_v2_mode mode;
    st_rate_policy policy;
    st_rate_decision last_v2_decision;
    int tick_counter;
    unsigned long long observation_time_ms;
    int cooldown_remaining;
    int probe_cooldown_remaining;

    double switch_cost_ewma_ms;
    bool switch_inflight;
    bool switch_suppression_logged;
    unsigned long long switch_started_ms;
    int switch_from_cfg;
    int switch_to_cfg;
    e_gearshift_v2_action switch_action;
    int switch_fallback_cfg;
    bool switch_is_nb;

    bool probe_active;
    bool probe_confirmed;
    bool probe_result_ready;
    bool probe_ladder_step;
    bool ladder_probe_accepted_now;
    int probe_target_cfg;
    int probe_fallback_cfg;
    double probe_baseline_bps;
    int probe_start_application_samples;
    int probe_start_outcome_samples;
    int probe_context_generation;
    int probe_baseline_generation;
    int probe_application_samples;
    int probe_outcome_samples;
    int probe_failed_outcomes;
    int probe_consecutive_failures;
    int probe_progress_events;
    double probe_application_bps_sum;
    double probe_application_bps_ewma;
    double probe_channel_ms;
    int probe_cycle_samples;
    double probe_cycle_ms_ewma;
    bool probe_pending_generation_contamination;
    double probe_geometry_trial_ms;
    double probe_initial_channel_ms;
    double probe_max_channel_ms;
    double probe_zero_progress_budget_ms;
    double next_probe_trial_ms;
    double next_probe_max_channel_ms;
    double next_probe_zero_progress_budget_ms;
    bool next_probe_ladder_step;

    // Application/transport unit conversion is learned from complete atomic
    // application units, not from a rolling window that can straddle SET_CONFIG.
    double atomic_application_transport_gain;
    bool atomic_application_transport_gain_valid;
    int atomic_application_transport_gain_samples;

    int context_generation;
    double last_context_snr_db;
    double last_context_selectivity;
    double context_volatility;
    int upward_probe_suppressed_until_tick;

    // Original controller state retained for the runtime legacy fallback.
    double legacy_hysteresis_ratio;
    int legacy_switch_cost_ms;
    double legacy_wire_ms_per_batch;
    bool legacy_optclock_measured;
    std::string label_streak_value;
    int label_streak_count;
    static const int LABEL_STREAK_REQUIRED = 4;

    int min_cfg_calibrated;
    double max_sack_calibrated;
    double max_partial_loss_calibrated;
    int min_cfg_calibrated_nb;
    double max_sack_calibrated_nb;
    double max_partial_loss_calibrated_nb;

    int parse_table_section(const std::string& body,
                            size_t section_pos,
                            std::map<int, std::map<std::string, st_rate_cell> >& out_table,
                            std::vector<std::pair<std::string, double> >& out_axis,
                            int& out_min_cfg,
                            double& out_max_sack,
                            double& out_max_partial_loss,
                            int& out_n_configs,
                            int& out_n_channels);

    const st_rate_cell* get_cell(int cfg, const std::string& bucket,
                                 bool is_nb) const;
    std::string identify_channel_label_legacy(int current_cfg,
                                              double current_sack_rate,
                                              double current_eff_bps,
                                              double current_partial_loss_rate,
                                              bool is_nb) const;
    int evaluate_legacy(int current_cfg,
                        double current_eff_bps,
                        double current_sack_rate,
                        int window_count,
                        int config_ceiling,
                        bool is_nb,
                        double current_partial_loss_rate);

    st_online_rate_model& online_model(int cfg, bool is_nb);
    const st_online_rate_model* online_model_const(int cfg, bool is_nb) const;
    st_rate_prediction predict_config(int cfg,
                                      const st_rate_observation& obs) const;
    st_rate_prediction prior_prediction(int cfg,
                                        const st_rate_observation& obs) const;
    const st_rate_cell* nearest_snr_cell(int cfg,
                                         double snr_db,
                                         bool is_nb,
                                         const st_rate_cell** upper,
                                         double* mix) const;
    st_rate_prediction context_calibration_prediction(
                                         int cfg,
                                         const st_rate_observation& obs,
                                         bool use_reverse_snr) const;
    void convert_prior_to_application_units(st_rate_prediction& p,
                                            const st_rate_observation& obs,
                                            bool empirical_calibration) const;
    void apply_calibration_trust(st_rate_prediction& p) const;
    void apply_calibration_batch_context(st_rate_prediction& p,
                                         double calibrated_batch_size,
                                         int cfg,
                                         const st_rate_observation& obs) const;
    std::string outcome_label(const st_rate_observation& obs) const;
    std::vector<int> candidate_configs(const st_rate_observation& obs) const;
    bool probe_target_blocked(int cfg, const st_rate_observation& obs) const;
    void remember_failed_probe(int cfg, const st_rate_observation& obs,
                               bool strong, const char* reason);
    void reset_probe_evidence_for_generation(const char* reason,
                                             bool triggering_transaction = false);
    void maybe_extend_probe_budget(const char* reason);
    void maybe_note_channel_change(double snr_db, double selectivity);
    void maybe_note_outcome_change(int cfg, bool is_nb, bool failed, double sample_bps);
    void apply_application_sample(st_online_rate_model& m, double sample_bps);
    double useful_horizon_ms(const st_rate_observation& obs,
                             double current_bps) const;
    void emit_decision(const st_rate_observation& obs,
                       const st_rate_decision& d) const;
    void clear_probe();
};

#endif // RATE_OPTIMIZER_H_
