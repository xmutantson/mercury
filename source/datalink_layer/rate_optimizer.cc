/*
 * Mercury: A configurable open-source software-defined modem.
 * Copyright (C) 2022-2026 Fadi Jerji
 * Author: Fadi Jerji
 *
 * Effective-Rate Optimizer — Phase 3c implementation.
 *
 * See: include/datalink_layer/rate_optimizer.h
 *      mercury/fact-documents/EFFECTIVE_RATE_OPTIMIZER_DESIGN.md
 *
 * Self-contained — no dependency on cl_arq_controller, common_defines,
 * physical_config etc. Reads only stdlib + cstdio + the rate_optimizer.h
 * declarations.
 *
 * Parsing: the calibration table is well-formed machine output, so we use
 * a small targeted JSON scanner rather than dragging in a full library.
 * Anything we don't recognize is ignored (forward-compatible with extra
 * keys added by tools/effective_rate_calibrate.py).
 */

#include "datalink_layer/rate_optimizer.h"
#include "common/common_defines.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>


// ---------- tiny JSON helpers (targeted, NOT a generic parser) ----------

namespace {

// Strip whitespace + comments. Returns the entire buffer in `out` minus any
// // or /* */ comments. JSON spec doesn't allow comments but the test table
// might have human notes — be permissive.
std::string strip_comments(const std::string& in) {
    std::string out;
    out.reserve(in.size());
    size_t i = 0, n = in.size();
    while (i < n) {
        char c = in[i];
        // // line comment
        if (c == '/' && i + 1 < n && in[i+1] == '/') {
            while (i < n && in[i] != '\n') ++i;
            continue;
        }
        // /* ... */ block comment
        if (c == '/' && i + 1 < n && in[i+1] == '*') {
            i += 2;
            while (i + 1 < n && !(in[i] == '*' && in[i+1] == '/')) ++i;
            i = (i + 1 < n) ? i + 2 : n;
            continue;
        }
        // String literal — copy verbatim (no escape handling needed; the
        // table never contains backslash-quotes in keys or string values).
        if (c == '"') {
            out += c; ++i;
            while (i < n && in[i] != '"') {
                if (in[i] == '\\' && i + 1 < n) { out += in[i++]; }
                out += in[i++];
            }
            if (i < n) { out += in[i++]; }
            continue;
        }
        out += c; ++i;
    }
    return out;
}

// Find the position of the value matched to `key` inside the object whose
// opening '{' is at or after `obj_brace_pos`. Returns std::string::npos
// if `key` is not found at the immediate child level. Nested objects are
// skipped via brace-depth tracking.
//
// If `obj_brace_pos` doesn't point at '{' itself, the function advances
// to the next '{' first. Whitespace before the brace is tolerated.
size_t find_key_value_pos(const std::string& s, size_t obj_brace_pos, const std::string& key) {
    size_t i = obj_brace_pos;
    // Advance to the opening '{' of the object we want to scan inside of.
    while (i < s.size() && std::isspace((unsigned char)s[i])) ++i;
    if (i >= s.size() || s[i] != '{') return std::string::npos;
    ++i;  // step inside the object → depth 0 means "directly inside this obj"

    std::string needle = "\"" + key + "\"";
    int depth = 0;
    bool in_str = false;
    while (i < s.size()) {
        char c = s[i];
        if (in_str) {
            if (c == '\\' && i + 1 < s.size()) { i += 2; continue; }
            if (c == '"') in_str = false;
            ++i; continue;
        }
        if (c == '"') {
            // Match only at depth 0 (immediate children of the scanned object).
            if (depth == 0 && i + needle.size() <= s.size()
                && s.compare(i, needle.size(), needle) == 0) {
                size_t j = i + needle.size();
                while (j < s.size() && std::isspace((unsigned char)s[j])) ++j;
                if (j < s.size() && s[j] == ':') {
                    ++j;
                    while (j < s.size() && std::isspace((unsigned char)s[j])) ++j;
                    return j;
                }
            }
            in_str = true;
            ++i; continue;
        }
        if (c == '{' || c == '[') { ++depth; ++i; continue; }
        if (c == '}' || c == ']') {
            if (depth == 0) return std::string::npos;  // end of scanned object
            --depth; ++i; continue;
        }
        ++i;
    }
    return std::string::npos;
}

// Parse a JSON number / true / false / null at pos. Returns the parsed
// value as double (true=1, false/null=0). Sets *end to the position
// after the value.
bool json_null_at(const std::string& s, size_t pos) {
    while (pos < s.size() && std::isspace((unsigned char)s[pos])) ++pos;
    return pos + 4 <= s.size() && s.compare(pos, 4, "null") == 0;
}

double parse_number_or_bool(const std::string& s, size_t pos, size_t* end) {
    if (pos >= s.size()) { *end = pos; return 0.0; }
    if (s.compare(pos, 4, "true") == 0)  { *end = pos + 4; return 1.0; }
    if (s.compare(pos, 5, "false") == 0) { *end = pos + 5; return 0.0; }
    if (s.compare(pos, 4, "null") == 0)  { *end = pos + 4; return 0.0; }
    char* tail = NULL;
    double v = std::strtod(s.c_str() + pos, &tail);
    if (tail == NULL || tail == s.c_str() + pos) { *end = pos; return 0.0; }
    *end = (size_t)(tail - s.c_str());
    return v;
}

std::string parse_json_string(const std::string& s, size_t pos) {
    if (pos >= s.size() || s[pos] != '"') return std::string();
    std::string out;
    for (size_t i=pos+1; i<s.size(); ++i) {
        char c=s[i];
        if (c == '"') return out;
        if (c == '\\' && i+1 < s.size()) { out += s[++i]; continue; }
        out += c;
    }
    return std::string();
}

std::string gearshift_config_signature() {
    char buf[160];
    std::snprintf(buf, sizeof(buf), "gs2-cfg%d-wb%d-nb%d-sack%d-bitmap%d",
        NUMBER_OF_CONFIGS, WB_CONFIG_MAX, NB_CONFIG_MAX,
        MFSK_ACK_SACK_ENABLED, MFSK_SACK_BITMAP_BITS);
    return std::string(buf);
}

// Skip from `pos` to the position just past the matching close brace/bracket
// for an open brace/bracket at `pos`. If s[pos] is not '{' or '[', returns pos.
size_t skip_container(const std::string& s, size_t pos) {
    if (pos >= s.size() || (s[pos] != '{' && s[pos] != '[')) return pos;
    char open = s[pos];
    char close = (open == '{') ? '}' : ']';
    int depth = 0;
    bool in_str = false;
    size_t i = pos;
    while (i < s.size()) {
        char c = s[i];
        if (in_str) {
            if (c == '\\' && i + 1 < s.size()) { i += 2; continue; }
            if (c == '"') in_str = false;
            ++i; continue;
        }
        if (c == '"') { in_str = true; ++i; continue; }
        if (c == open)  { ++depth; ++i; continue; }
        if (c == close) { --depth; ++i; if (depth == 0) return i; continue; }
        ++i;
    }
    return i;
}

// Iterate object keys at the current brace level. Calls `cb(key, value_pos)`
// for each entry. `obj_pos` must point to the '{' of the object.
template <typename F>
void for_each_object_key(const std::string& s, size_t obj_pos, F cb) {
    if (obj_pos >= s.size() || s[obj_pos] != '{') return;
    size_t i = obj_pos + 1;
    while (i < s.size()) {
        // Skip whitespace + commas
        while (i < s.size() && (std::isspace((unsigned char)s[i]) || s[i] == ',')) ++i;
        if (i >= s.size()) return;
        if (s[i] == '}') return;
        if (s[i] != '"') return;   // malformed
        // Read key string
        ++i;
        std::string key;
        while (i < s.size() && s[i] != '"') {
            if (s[i] == '\\' && i + 1 < s.size()) { ++i; }
            key += s[i++];
        }
        if (i >= s.size()) return;
        ++i;  // past closing quote
        // Skip whitespace then ':'
        while (i < s.size() && std::isspace((unsigned char)s[i])) ++i;
        if (i >= s.size() || s[i] != ':') return;
        ++i;
        while (i < s.size() && std::isspace((unsigned char)s[i])) ++i;
        size_t value_pos = i;
        cb(key, value_pos);
        // Skip the value
        if (i < s.size() && (s[i] == '{' || s[i] == '[')) {
            i = skip_container(s, i);
        } else if (i < s.size() && s[i] == '"') {
            ++i;
            while (i < s.size() && s[i] != '"') {
                if (s[i] == '\\' && i + 1 < s.size()) { ++i; }
                ++i;
            }
            if (i < s.size()) ++i;
        } else {
            size_t end;
            (void)parse_number_or_bool(s, i, &end);
            i = end;
        }
    }
}

} // anonymous namespace


namespace {

double clamp_double(double v, double lo, double hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void apply_break_risk_to_prior(const st_rate_cell* cell,
                               double& sigma_bps,
                               double& weight)
{
    if (!cell || !cell->break_run_rate_valid) return;
    const double p = clamp_double(cell->break_run_rate, 0.0, 1.0);
    // BREAK means a fixed action needed emergency recovery during some
    // calibration runs.  Preserve its measured mean (it may still be useful
    // for a short transaction) but make the counterfactual less authoritative.
    sigma_bps *= 1.0 + 2.0*p;
    weight *= std::max(0.10, 1.0 - 0.80*p);
}

int env_int(const char* name, int fallback, int lo, int hi)
{
    const char* raw = std::getenv(name);
    if (!raw || !*raw) return fallback;
    char* end = NULL;
    long v = std::strtol(raw, &end, 10);
    if (end == raw || *end != '\0') return fallback;
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    return (int)v;
}

double env_double(const char* name, double fallback, double lo, double hi)
{
    const char* raw = std::getenv(name);
    if (!raw || !*raw) return fallback;
    char* end = NULL;
    double v = std::strtod(raw, &end);
    if (end == raw || *end != '\0' || !std::isfinite(v)) return fallback;
    return clamp_double(v, lo, hi);
}

const char* action_name(e_gearshift_v2_action action)
{
    switch (action) {
    case GEARSHIFT_ACTION_SWITCH: return "SWITCH";
    case GEARSHIFT_ACTION_PROBE: return "PROBE";
    case GEARSHIFT_ACTION_ROLLBACK: return "ROLLBACK";
    case GEARSHIFT_ACTION_ABSTAIN: return "ABSTAIN";
    default: return "HOLD";
    }
}

int gearshift_action_rank(int cfg)
{
    const int idx = config_ladder_index(cfg);
    if (idx >= 0) return idx;
    // Off-ladder experimental/pinned configurations are not normal v2 actions.
    // Keep a deterministic ordering only for defensive comparison.
    return FULL_CONFIG_LADDER_SIZE + 1000 + cfg;
}

bool gearshift_destination_within_ofdm_ceiling(int cfg, int config_ceiling)
{
    // The numeric robust IDs live at 100+, so an OFDM ceiling must never be
    // applied to them.  Robust 0/1/2 are ordered by FULL_CONFIG_LADDER instead.
    if (cfg >= CONFIG_0 && cfg <= CONFIG_17) return cfg <= config_ceiling;
    return true;
}

} // namespace

st_rate_cell::st_rate_cell()
    : eff_bps_mean(0.0), eff_bps_sigma(0.0), sack_rate_mean(0.0),
      sack_rate_valid(false), partial_loss_mean(0.0), failed_batch_rate_mean(0.0),
      frame_success_mean(1.0), selectivity_mean(-1.0), batch_size_mean(-1.0),
      break_run_rate(0.0), snr_hint_db(-999.0), eff_bps_present(false), partial_loss_valid(false),
      failed_batch_rate_valid(false), frame_success_valid(false),
      selectivity_valid(false), batch_size_valid(false), break_run_rate_valid(false),
      n_runs(0), n_failed_runs(0),
      n_zero_delivery_runs(0), n_invalid_runs(0), v2_all_valid_runs(false),
      failed(false), break_fired(false), valid(false)
{}

int st_rate_cell::total_runs() const
{
    if (v2_all_valid_runs) return n_runs > 0 ? n_runs : (valid ? 1 : 0);
    const int n = n_runs + n_failed_runs;
    return n > 0 ? n : (valid ? 1 : 0);
}

double st_rate_cell::survival_adjusted_mean() const
{
    if (v2_all_valid_runs) return eff_bps_mean;
    const int total = total_runs();
    if (total <= 0) return eff_bps_mean;
    // Legacy calibration used n_runs for positive-rate survivors and kept
    // zero-delivery attempts only in n_failed_runs.  Treat those as real zero
    // outcomes rather than silently dropping them from the prior.
    if (n_failed_runs > 0 && n_runs > 0)
        return eff_bps_mean * (double)n_runs / (double)total;
    if (n_runs <= 0 && n_failed_runs > 0)
        return 0.0;
    return eff_bps_mean;
}

st_rate_observation::st_rate_observation()
    : current_cfg(-1), application_bps(0.0), transport_bps(0.0),
      sack_batch_rate(0.0), frame_success_rate(1.0),
      partial_frame_loss_rate(0.0), failed_batch_rate(0.0),
      rate_samples(0), outcome_samples(0), application_commits(0),
      forward_snr_db(-99.9), forward_snr_age_batches(1000000000),
      forward_selectivity(-1.0), forward_selectivity_age_batches(1000000000),
      reverse_snr_db(-99.9), reverse_snr_age_batches(1000000000),
      batch_size(0), queue_bytes(0), remaining_work_known(false), monotonic_ms(0),
      is_nb(false), compression_enabled(false), application_transport_gain(1.0),
      application_transport_gain_valid(false), feedback_budget_ms(0.0)
{}

st_rate_prediction::st_rate_prediction()
    : valid(false), mean_bps(0.0), sigma_bps(0.0), prior_weight(0.0),
      live_weight(0.0), live_samples(0), age_batches(1000000000),
      age_ms(1.0e30), direct_evidence(false), source("none")
{}

st_rate_decision::st_rate_decision()
    : action(GEARSHIFT_ACTION_HOLD), current_cfg(-1), target_cfg(-1),
      fallback_cfg(-1), current_mean_bps(0.0), current_sigma_bps(0.0),
      target_mean_bps(0.0), target_sigma_bps(0.0), net_target_bps(0.0),
      switch_cost_ms(0.0), horizon_ms(0.0), actionable(false), reason("hold")
{}

st_rate_policy::st_rate_policy()
    : min_outcome_samples(4), min_rate_samples(1), cooldown_batches(4),
      probe_cooldown_batches(12), probe_soft_cooldown_batches(3),
      probe_min_application_samples(3), probe_min_outcome_samples(3),
      probe_hard_failure_streak(2), probe_budget_extra_cycles(1),
      forward_quality_max_age_batches(4),
      reverse_quality_max_age_batches(2), trace_idle_decisions(0),
      direct_switch_margin(0.03),
      probe_mean_margin(0.03), probe_rollback_ratio(0.97),
      probe_max_probation_ms(120000.0), probe_zero_progress_ms(12000.0),
      probe_budget_safety_factor(1.35), switch_inflight_timeout_ms(300000.0),
      confidence_z(0.75),
      failure_direct_threshold(0.45), prior_default_rel_sigma(0.30),
      prior_min_sigma_bps(150.0), prior_run_weight_cap(5.0),
      live_weight_cap(12.0), live_stale_half_life_batches(24.0),
      live_stale_half_life_ms(30000.0), online_ewma_alpha(0.30),
      selectivity_uncertainty_gain(2.5),
      calibration_snr_kernel_db(3.0), calibration_selectivity_kernel(0.08),
      calibration_context_distance_sigma_gain(0.35), compression_gain_rel_sigma(0.25),
      batch_context_half_width(0.55), uncalibrated_prior_rel_sigma(0.80),
      uncalibrated_prior_weight(0.20), probe_reopen_snr_db(2.0),
      probe_reopen_selectivity(0.06), channel_change_snr_db(4.0),
      channel_change_selectivity(0.10), reverse_snr_prior_weight_scale(0.25),
      reverse_snr_sigma_gain(2.0), context_volatility_alpha(0.25),
      context_volatility_live_gain(1.5), context_volatility_horizon_gain(2.0),
      outcome_change_rel_threshold(0.55), outcome_change_sigma_threshold(2.25),
      outcome_change_min_samples(4),
      default_horizon_ms(30000.0), min_horizon_ms(8000.0),
      max_horizon_ms(120000.0), default_switch_cost_ms(1800.0)
{}

void st_rate_policy::load_env()
{
    min_outcome_samples = env_int("MERCURY_GS2_MIN_OUTCOMES", min_outcome_samples, 1, 100);
    min_rate_samples = env_int("MERCURY_GS2_MIN_RATE", min_rate_samples, 0, 100);
    cooldown_batches = env_int("MERCURY_GS2_COOLDOWN", cooldown_batches, 0, 100);
    probe_cooldown_batches = env_int("MERCURY_GS2_PROBE_COOLDOWN", probe_cooldown_batches, 0, 1000);
    probe_soft_cooldown_batches = env_int("MERCURY_GS2_PROBE_SOFT_COOLDOWN", probe_soft_cooldown_batches, 0, 1000);
    probe_min_application_samples = env_int("MERCURY_GS2_PROBE_MIN_APP", probe_min_application_samples, 1, 100);
    probe_min_outcome_samples = env_int("MERCURY_GS2_PROBE_MIN_OUTCOMES", probe_min_outcome_samples, 1, 100);
    probe_hard_failure_streak = env_int("MERCURY_GS2_PROBE_HARD_FAIL_STREAK", probe_hard_failure_streak, 2, 100);
    probe_budget_extra_cycles = env_int("MERCURY_GS2_PROBE_BUDGET_EXTRA_CYCLES", probe_budget_extra_cycles, 0, 10);
    forward_quality_max_age_batches = env_int("MERCURY_GS2_FORWARD_QUALITY_MAX_AGE",
                                              forward_quality_max_age_batches, 0, 1000);
    reverse_quality_max_age_batches = env_int("MERCURY_GS2_REVERSE_QUALITY_MAX_AGE",
                                              reverse_quality_max_age_batches, 0, 1000);
    trace_idle_decisions = env_int("MERCURY_GS2_TRACE_IDLE", trace_idle_decisions, 0, 1);
    direct_switch_margin = env_double("MERCURY_GS2_SWITCH_MARGIN", direct_switch_margin, 0.0, 1.0);
    probe_mean_margin = env_double("MERCURY_GS2_PROBE_MARGIN", probe_mean_margin, 0.0, 2.0);
    probe_rollback_ratio = env_double("MERCURY_GS2_ROLLBACK_RATIO", probe_rollback_ratio, 0.0, 1.5);
    probe_max_probation_ms = env_double("MERCURY_GS2_PROBE_MAX_MS", probe_max_probation_ms, 1000.0, 300000.0);
    probe_zero_progress_ms = env_double("MERCURY_GS2_PROBE_ZERO_PROGRESS_MS", probe_zero_progress_ms, 1000.0, 300000.0);
    probe_budget_safety_factor = env_double("MERCURY_GS2_PROBE_BUDGET_SAFETY", probe_budget_safety_factor, 1.0, 4.0);
    switch_inflight_timeout_ms = env_double("MERCURY_GS2_SWITCH_INFLIGHT_MAX_MS",
                                            switch_inflight_timeout_ms, 5000.0, 300000.0);
    confidence_z = env_double("MERCURY_GS2_CONFIDENCE_Z", confidence_z, 0.0, 4.0);
    failure_direct_threshold = env_double("MERCURY_GS2_FAILURE_THRESHOLD", failure_direct_threshold, 0.0, 1.0);
    calibration_snr_kernel_db = env_double("MERCURY_GS2_CAL_SNR_KERNEL_DB", calibration_snr_kernel_db, 0.25, 20.0);
    calibration_selectivity_kernel = env_double("MERCURY_GS2_CAL_SEL_KERNEL", calibration_selectivity_kernel, 0.005, 2.0);
    calibration_context_distance_sigma_gain = env_double("MERCURY_GS2_CAL_DISTANCE_SIGMA", calibration_context_distance_sigma_gain, 0.0, 4.0);
    compression_gain_rel_sigma = env_double("MERCURY_GS2_COMPRESSION_GAIN_SIGMA", compression_gain_rel_sigma, 0.0, 2.0);
    batch_context_half_width = env_double("MERCURY_GS2_BATCH_CONTEXT", batch_context_half_width, 0.05, 4.0);
    uncalibrated_prior_rel_sigma = env_double("MERCURY_GS2_UNCAL_SIGMA", uncalibrated_prior_rel_sigma, 0.10, 4.0);
    uncalibrated_prior_weight = env_double("MERCURY_GS2_UNCAL_WEIGHT", uncalibrated_prior_weight, 0.01, 2.0);
    probe_reopen_snr_db = env_double("MERCURY_GS2_PROBE_REOPEN_SNR", probe_reopen_snr_db, 0.0, 20.0);
    probe_reopen_selectivity = env_double("MERCURY_GS2_PROBE_REOPEN_SEL", probe_reopen_selectivity, 0.0, 1.0);
    channel_change_snr_db = env_double("MERCURY_GS2_CHANGE_SNR", channel_change_snr_db, 0.5, 30.0);
    channel_change_selectivity = env_double("MERCURY_GS2_CHANGE_SEL", channel_change_selectivity, 0.01, 1.0);
    reverse_snr_prior_weight_scale = env_double("MERCURY_GS2_REVERSE_SNR_WEIGHT", reverse_snr_prior_weight_scale, 0.0, 1.0);
    reverse_snr_sigma_gain = env_double("MERCURY_GS2_REVERSE_SNR_SIGMA", reverse_snr_sigma_gain, 1.0, 10.0);
    context_volatility_alpha = env_double("MERCURY_GS2_VOLATILITY_ALPHA", context_volatility_alpha, 0.01, 1.0);
    context_volatility_live_gain = env_double("MERCURY_GS2_VOLATILITY_LIVE_GAIN", context_volatility_live_gain, 0.0, 10.0);
    context_volatility_horizon_gain = env_double("MERCURY_GS2_VOLATILITY_HORIZON_GAIN", context_volatility_horizon_gain, 0.0, 10.0);
    outcome_change_rel_threshold = env_double("MERCURY_GS2_OUTCOME_CHANGE", outcome_change_rel_threshold, 0.05, 4.0);
    outcome_change_sigma_threshold = env_double("MERCURY_GS2_OUTCOME_CHANGE_SIGMA", outcome_change_sigma_threshold, 0.5, 10.0);
    outcome_change_min_samples = env_int("MERCURY_GS2_OUTCOME_CHANGE_MIN_SAMPLES", outcome_change_min_samples, 2, 100);
    live_stale_half_life_ms = env_double("MERCURY_GS2_LIVE_HALF_LIFE_MS", live_stale_half_life_ms, 1000.0, 600000.0);
    default_horizon_ms = env_double("MERCURY_GS2_HORIZON_MS", default_horizon_ms, 1000.0, 600000.0);
    min_horizon_ms = env_double("MERCURY_GS2_MIN_HORIZON_MS", min_horizon_ms, 1000.0, 600000.0);
    max_horizon_ms = env_double("MERCURY_GS2_MAX_HORIZON_MS", max_horizon_ms, min_horizon_ms, 3600000.0);
    default_switch_cost_ms = env_double("MERCURY_GS2_SWITCH_COST_MS", default_switch_cost_ms, 0.0, 60000.0);
}

st_online_rate_model::st_online_rate_model()
    : initialized(false), ewma_application_bps(0.0), ewma_transport_bps(0.0),
      ewma_variance(0.0), last_application_sample_bps(0.0),
      pending_cycle_ms(0.0), pending_transport_bytes(0.0),
      failure_ewma(0.0), sack_ewma(0.0), frame_success_ewma(1.0),
      partial_loss_ewma(0.0), batch_size_ewma(0.0), application_samples(0),
      outcome_samples(0), consecutive_failures(0), last_tick(-1000000000),
      last_observation_ms(0), context_generation(0)
{}

cl_rate_optimizer::cl_rate_optimizer()
    : enabled(true), calibration_loaded(false), calibration_prior_weight_scale(1.0),
      calibration_prior_sigma_scale(1.0), calibration_direct_authority(true),
      calibration_build_id(), calibration_config_signature(),
      calibration_compress_known(false), calibration_compressed(false),
      nb_enabled(false), n_configs_loaded(0),
      n_channels_loaded(0), n_configs_loaded_nb(0), n_channels_loaded_nb(0),
      mode(GEARSHIFT_V2_LEGACY), tick_counter(0), observation_time_ms(0),
      cooldown_remaining(0),
      probe_cooldown_remaining(0), switch_cost_ewma_ms(1800.0),
      switch_inflight(false), switch_suppression_logged(false), switch_started_ms(0), switch_from_cfg(-1),
      switch_to_cfg(-1), switch_action(GEARSHIFT_ACTION_HOLD),
      switch_fallback_cfg(-1), switch_is_nb(false), probe_active(false), probe_confirmed(false),
      probe_result_ready(false), probe_ladder_step(false), ladder_probe_accepted_now(false),
      probe_target_cfg(-1), probe_fallback_cfg(-1),
      probe_baseline_bps(0.0), probe_start_application_samples(0),
      probe_start_outcome_samples(0), probe_context_generation(0),
      probe_baseline_generation(0), probe_application_samples(0),
      probe_outcome_samples(0), probe_failed_outcomes(0),
      probe_consecutive_failures(0), probe_progress_events(0),
      probe_application_bps_sum(0.0), probe_application_bps_ewma(0.0),
      probe_channel_ms(0.0), probe_cycle_samples(0), probe_cycle_ms_ewma(0.0),
      probe_pending_generation_contamination(false), probe_geometry_trial_ms(0.0),
      probe_initial_channel_ms(0.0), probe_max_channel_ms(0.0),
      probe_zero_progress_budget_ms(0.0), next_probe_trial_ms(0.0),
      next_probe_max_channel_ms(0.0), next_probe_zero_progress_budget_ms(0.0),
      next_probe_ladder_step(false),
      atomic_application_transport_gain(1.0),
      atomic_application_transport_gain_valid(false), atomic_application_transport_gain_samples(0),
      context_generation(0),
      last_context_snr_db(-99.9), last_context_selectivity(-1.0),
      context_volatility(0.0), upward_probe_suppressed_until_tick(0),
      legacy_hysteresis_ratio(1.05),
      legacy_switch_cost_ms(1800), legacy_wire_ms_per_batch(1800.0),
      legacy_optclock_measured(false), label_streak_count(0),
      min_cfg_calibrated(-1), max_sack_calibrated(-1.0),
      max_partial_loss_calibrated(-1.0), min_cfg_calibrated_nb(-1),
      max_sack_calibrated_nb(-1.0), max_partial_loss_calibrated_nb(-1.0)
{
    policy.load_env();
    switch_cost_ewma_ms = policy.default_switch_cost_ms;
    configure_mode_from_env();
}

void cl_rate_optimizer::configure_mode_from_env()
{
    const char* raw = std::getenv("MERCURY_GEARSHIFT_V2");
    if (!raw || !*raw || std::strcmp(raw, "legacy") == 0 || std::strcmp(raw, "0") == 0) {
        mode = GEARSHIFT_V2_LEGACY;
    } else if (std::strcmp(raw, "shadow") == 0) {
        mode = GEARSHIFT_V2_SHADOW;
    } else if (std::strcmp(raw, "active") == 0 || std::strcmp(raw, "1") == 0) {
        mode = GEARSHIFT_V2_ACTIVE;
    } else {
        mode = GEARSHIFT_V2_LEGACY;
        std::printf("[GEARSHIFT-V2] unknown MERCURY_GEARSHIFT_V2=%s; using legacy\n", raw);
    }
}

const char* cl_rate_optimizer::mode_name() const
{
    if (mode == GEARSHIFT_V2_ACTIVE) return "active";
    if (mode == GEARSHIFT_V2_SHADOW) return "shadow";
    return "legacy";
}

int cl_rate_optimizer::parse_table_section(
        const std::string& body,
        size_t section_pos,
        std::map<int, std::map<std::string, st_rate_cell> >& out_table,
        std::vector<std::pair<std::string, double> >& out_axis,
        int& out_min_cfg,
        double& out_max_sack,
        double& out_max_partial_loss,
        int& out_n_configs,
        int& out_n_channels)
{
    out_table.clear();
    out_axis.clear();
    out_min_cfg = -1;
    out_max_sack = -1.0;
    out_max_partial_loss = -1.0;
    out_n_configs = 0;
    out_n_channels = 0;

    std::map<std::string, std::pair<double, int> > channel_sack_accum;
    int valid_cells = 0;

    for_each_object_key(body, section_pos, [&](const std::string& cfg_key, size_t cfg_v) {
        if (cfg_v >= body.size() || body[cfg_v] != '{') return;
        int cfg_id = std::atoi(cfg_key.c_str());
        if (cfg_id < 0) return;
        std::map<std::string, st_rate_cell>& channels = out_table[cfg_id];

        for_each_object_key(body, cfg_v, [&](const std::string& ch_key, size_t ch_v) {
            if (ch_v >= body.size() || body[ch_v] != '{') return;
            st_rate_cell cell;
            // IMPORTANT: a legacy label such as "wgn20" is an IONOS dial,
            // not necessarily the modem's measured receiver SNR.  Only an
            // explicit measured snr_db field may drive SNR interpolation.
            for_each_object_key(body, ch_v, [&](const std::string& field, size_t field_v) {
                size_t end = field_v;
                if (field == "eff_bps_mean") {
                    if (!json_null_at(body, field_v)) {
                        cell.eff_bps_mean = parse_number_or_bool(body, field_v, &end);
                        cell.eff_bps_present = std::isfinite(cell.eff_bps_mean);
                    }
                } else if (field == "eff_bps_sigma") {
                    if (!json_null_at(body, field_v))
                        cell.eff_bps_sigma = parse_number_or_bool(body, field_v, &end);
                } else if (field == "sack_rate_mean") {
                    if (!json_null_at(body, field_v)) {
                        cell.sack_rate_mean = parse_number_or_bool(body, field_v, &end);
                        cell.sack_rate_valid = std::isfinite(cell.sack_rate_mean);
                    }
                } else if (field == "frame_loss_pct") {
                    if (!json_null_at(body, field_v)) {
                        cell.partial_loss_mean = parse_number_or_bool(body, field_v, &end) / 100.0;
                        cell.partial_loss_valid = true;
                    }
                } else if (field == "failed_batch_rate_mean") {
                    if (!json_null_at(body, field_v)) {
                        cell.failed_batch_rate_mean = parse_number_or_bool(body, field_v, &end);
                        cell.failed_batch_rate_valid = true;
                    }
                } else if (field == "frame_success_mean") {
                    if (!json_null_at(body, field_v)) {
                        cell.frame_success_mean = parse_number_or_bool(body, field_v, &end);
                        cell.frame_success_valid = true;
                    }
                } else if (field == "selectivity_mean") {
                    if (!json_null_at(body, field_v)) {
                        cell.selectivity_mean = parse_number_or_bool(body, field_v, &end);
                        cell.selectivity_valid = true;
                    }
                } else if (field == "batch_size_mean") {
                    if (!json_null_at(body, field_v)) {
                        cell.batch_size_mean = parse_number_or_bool(body, field_v, &end);
                        cell.batch_size_valid = cell.batch_size_mean > 0.0;
                    }
                } else if (field == "break_run_rate") {
                    if (!json_null_at(body, field_v)) {
                        cell.break_run_rate = clamp_double(
                            parse_number_or_bool(body, field_v, &end), 0.0, 1.0);
                        cell.break_run_rate_valid = true;
                    }
                } else if (field == "snr_db") {
                    if (!json_null_at(body, field_v))
                        cell.snr_hint_db = parse_number_or_bool(body, field_v, &end);
                } else if (field == "n_runs") {
                    if (!json_null_at(body, field_v)) cell.n_runs = (int)parse_number_or_bool(body, field_v, &end);
                } else if (field == "n_failed_runs") {
                    if (!json_null_at(body, field_v)) cell.n_failed_runs = (int)parse_number_or_bool(body, field_v, &end);
                } else if (field == "n_zero_delivery_runs") {
                    if (!json_null_at(body, field_v)) cell.n_zero_delivery_runs = (int)parse_number_or_bool(body, field_v, &end);
                } else if (field == "n_invalid_runs") {
                    if (!json_null_at(body, field_v)) cell.n_invalid_runs = (int)parse_number_or_bool(body, field_v, &end);
                } else if (field == "outcome_semantics") {
                    // Presence of the v2 marker means n_runs already includes
                    // valid zero-delivery outcomes; n_failed_runs/n_invalid_runs
                    // are infrastructure failures and must not depress RF goodput.
                    cell.v2_all_valid_runs = true;
                } else if (field == "failed") {
                    if (!json_null_at(body, field_v)) cell.failed = parse_number_or_bool(body, field_v, &end) > 0.5;
                } else if (field == "break_fired") {
                    if (!json_null_at(body, field_v)) cell.break_fired = parse_number_or_bool(body, field_v, &end) > 0.5;
                }
            });

            // A fixed-config run that fired BREAK is still useful evidence, but
            // it proves the action was not stably sustainable for the full run.
            // New tables carry the fraction explicitly; old bool-only tables are
            // conservatively treated as a 100% break-incidence cell.
            if (!cell.break_run_rate_valid && cell.break_fired) {
                cell.break_run_rate = 1.0;
                cell.break_run_rate_valid = true;
            }

            // A calibrated zero-delivery cell is valuable evidence, not missing
            // data.  Overall failed/break cells become known-bad zero priors.
            const int total = cell.total_runs();
            if ((cell.failed || cell.break_fired) && !cell.eff_bps_present) {
                cell.eff_bps_mean = 0.0;
                cell.eff_bps_present = true;
            }
            cell.valid = cell.eff_bps_present && total >= 1;
            channels[ch_key] = cell;
            if (cell.valid) {
                ++valid_cells;
                if (cell.sack_rate_valid) {
                    std::pair<double, int>& acc = channel_sack_accum[ch_key];
                    acc.first += cell.sack_rate_mean;
                    acc.second += 1;
                }
            }
        });
    });

    out_n_configs = (int)out_table.size();
    for (std::map<std::string, std::pair<double, int> >::const_iterator
            it = channel_sack_accum.begin(); it != channel_sack_accum.end(); ++it) {
        if (it->second.second > 0) {
            out_axis.push_back(std::make_pair(
                it->first, it->second.first / (double)it->second.second));
        }
    }
    std::sort(out_axis.begin(), out_axis.end(),
              [](const std::pair<std::string,double>& a,
                 const std::pair<std::string,double>& b){ return a.second < b.second; });
    out_n_channels = (int)out_axis.size();

    out_min_cfg = -1;
    out_max_sack = 0.0;
    out_max_partial_loss = -1.0;
    for (std::map<int, std::map<std::string, st_rate_cell> >::const_iterator
            cit = out_table.begin(); cit != out_table.end(); ++cit) {
        bool any_valid = false;
        for (std::map<std::string, st_rate_cell>::const_iterator
                jt = cit->second.begin(); jt != cit->second.end(); ++jt) {
            if (!jt->second.valid) continue;
            any_valid = true;
            if (jt->second.sack_rate_valid)
                out_max_sack = std::max(out_max_sack, jt->second.sack_rate_mean);
            if (jt->second.partial_loss_valid)
                out_max_partial_loss = std::max(out_max_partial_loss,
                                                jt->second.partial_loss_mean);
        }
        if (any_valid && (out_min_cfg < 0 || cit->first < out_min_cfg))
            out_min_cfg = cit->first;
    }
    return valid_cells;
}

bool cl_rate_optimizer::load(const char* path)
{
    // Loading calibration is optional for Gearshift-v2. A missing/stale table
    // removes only the empirical prior; the live controller remains available.
    calibration_loaded = false;
    calibration_prior_weight_scale = 1.0;
    calibration_prior_sigma_scale = 1.0;
    calibration_direct_authority = true;
    calibration_build_id.clear();
    calibration_config_signature.clear();
    calibration_compress_known = false;
    calibration_compressed = false;
    nb_enabled = false;
    table.clear(); table_nb.clear();
    channel_axis.clear(); channel_axis_nb.clear();
    n_configs_loaded = n_channels_loaded = 0;
    n_configs_loaded_nb = n_channels_loaded_nb = 0;
    min_cfg_calibrated = min_cfg_calibrated_nb = -1;
    max_sack_calibrated = max_sack_calibrated_nb = -1.0;
    max_partial_loss_calibrated = max_partial_loss_calibrated_nb = -1.0;

    if (!path || !*path) {
        std::printf("[OPT] no calibration path; Gearshift-v2 continues uncalibrated\n");
        return false;
    }
    std::ifstream f(path);
    if (!f.is_open()) {
        std::printf("[OPT] table not found (path=%s); Gearshift-v2 continues uncalibrated\n", path);
        return false;
    }
    std::stringstream ss; ss << f.rdbuf();
    const std::string raw = ss.str();
    if (raw.empty()) {
        std::printf("[OPT] table empty (path=%s); Gearshift-v2 continues uncalibrated\n", path);
        return false;
    }
    const std::string body = strip_comments(raw);
    const size_t root = body.find('{');
    if (root != std::string::npos) {
        const size_t hp = find_key_value_pos(body, root, "mercury_head");
        const size_t sp = find_key_value_pos(body, root, "config_signature");
        if (hp != std::string::npos) calibration_build_id = parse_json_string(body, hp);
        if (sp != std::string::npos) calibration_config_signature = parse_json_string(body, sp);
        const size_t setup = find_key_value_pos(body, root, "calibration_setup");
        if (setup != std::string::npos && setup < body.size() && body[setup] == '{') {
            const size_t cp = find_key_value_pos(body, setup, "compress");
            if (cp != std::string::npos) {
                size_t end = cp;
                calibration_compressed = parse_number_or_bool(body, cp, &end) > 0.5;
                calibration_compress_known = true;
            }
        }
    }
    const std::string expected_sig = gearshift_config_signature();
    if (calibration_config_signature.empty()) {
        calibration_prior_weight_scale *= 0.50;
        calibration_prior_sigma_scale *= 1.25;
        std::printf("[OPT] calibration metadata lacks config_signature; prior weight x%.2f sigma x%.2f\n",
                    calibration_prior_weight_scale, calibration_prior_sigma_scale);
    } else if (calibration_config_signature != expected_sig) {
        calibration_prior_weight_scale *= 0.10;
        calibration_prior_sigma_scale *= 3.0;
        calibration_direct_authority = false;
        std::printf("[OPT] WARNING calibration config signature mismatch table=%s current=%s; prior weight x%.2f sigma x%.2f probe-only\n",
                    calibration_config_signature.c_str(), expected_sig.c_str(),
                    calibration_prior_weight_scale, calibration_prior_sigma_scale);
    }
    const std::string build_id = MERCURY_BUILD_ID;
    if (!calibration_build_id.empty() && build_id != "unknown" && build_id != "nogit") {
        std::string clean_build = build_id;
        const size_t dirty = clean_build.find("-dirty");
        if (dirty != std::string::npos) clean_build.erase(dirty);
        if (clean_build.compare(0, calibration_build_id.size(), calibration_build_id) != 0 &&
            calibration_build_id.compare(0, clean_build.size(), clean_build) != 0) {
            calibration_prior_weight_scale *= 0.65;
            calibration_prior_sigma_scale *= 1.5;
            std::printf("[OPT] calibration build differs table=%s current=%s; live evidence will dominate sooner (prior x%.2f sigma x%.2f)\n",
                        calibration_build_id.c_str(), build_id.c_str(),
                        calibration_prior_weight_scale, calibration_prior_sigma_scale);
        }
    }
    int valid_wb = 0;
    const size_t table_v = find_key_value_pos(body, 0, "table");
    if (table_v != std::string::npos && table_v < body.size() && body[table_v] == '{') {
        valid_wb = parse_table_section(body, table_v, table, channel_axis,
                                       min_cfg_calibrated, max_sack_calibrated,
                                       max_partial_loss_calibrated,
                                       n_configs_loaded, n_channels_loaded);
    }

    int valid_nb = 0;
    const size_t table_nb_v = find_key_value_pos(body, 0, "table_nb");
    if (table_nb_v != std::string::npos && table_nb_v < body.size() && body[table_nb_v] == '{') {
        valid_nb = parse_table_section(body, table_nb_v, table_nb, channel_axis_nb,
                                       min_cfg_calibrated_nb, max_sack_calibrated_nb,
                                       max_partial_loss_calibrated_nb,
                                       n_configs_loaded_nb, n_channels_loaded_nb);
    }
    nb_enabled = valid_nb > 0;
    calibration_loaded = valid_wb > 0 || valid_nb > 0;
    if (!calibration_loaded) {
        std::printf("[OPT] calibration contains no valid WB or NB cells; Gearshift-v2 continues uncalibrated\n");
        return false;
    }
    if (valid_wb > 0)
        std::printf("[OPT] WB calibration loaded: %d configs x %d channels (valid_cells=%d, mode=%s, path=%s)\n",
                    n_configs_loaded, n_channels_loaded, valid_wb, mode_name(), path);
    else
        std::printf("[OPT] WB calibration absent — WB v2 will use analytical/live evidence\n");
    if (nb_enabled)
        std::printf("[OPT] NB calibration loaded: %d configs x %d channels (valid_cells=%d, mode=%s, path=%s)\n",
                    n_configs_loaded_nb, n_channels_loaded_nb, valid_nb, mode_name(), path);
    else
        std::printf("[OPT] NB calibration absent — NB v2 will use analytical/live evidence\n");
    std::fflush(stdout);
    return true;
}

const st_rate_cell* cl_rate_optimizer::get_cell(int cfg, const std::string& bucket,
                                                 bool is_nb) const
{
    const std::map<int, std::map<std::string, st_rate_cell> >& active =
        is_nb ? table_nb : table;
    std::map<int, std::map<std::string, st_rate_cell> >::const_iterator it = active.find(cfg);
    if (it == active.end()) return NULL;
    std::map<std::string, st_rate_cell>::const_iterator jt = it->second.find(bucket);
    return jt == it->second.end() ? NULL : &jt->second;
}

std::string cl_rate_optimizer::identify_channel_label_legacy(
        int current_cfg, double current_sack_rate, double current_eff_bps,
        double current_partial_loss_rate, bool is_nb) const
{
    if (!std::isfinite(current_sack_rate) || !std::isfinite(current_eff_bps) ||
        !std::isfinite(current_partial_loss_rate)) return "";
    const std::map<int, std::map<std::string, st_rate_cell> >& active =
        is_nb ? table_nb : table;
    std::map<int, std::map<std::string, st_rate_cell> >::const_iterator cit = active.find(current_cfg);
    if (cit == active.end()) return "";
    const double norm = current_eff_bps > 100.0 ? current_eff_bps : 1000.0;
    double best_d = 1e300;
    std::string best;
    for (std::map<std::string, st_rate_cell>::const_iterator jt = cit->second.begin();
         jt != cit->second.end(); ++jt) {
        if (!jt->second.valid) continue;
        const bool hs = jt->second.sack_rate_valid;
        const double ds = hs ? jt->second.sack_rate_mean - current_sack_rate : 0.0;
        const double de = (jt->second.survival_adjusted_mean() - current_eff_bps) / norm;
        const bool hp = jt->second.partial_loss_valid && current_partial_loss_rate >= 0.0;
        const double dl = hp ? jt->second.partial_loss_mean - current_partial_loss_rate : 0.0;
        const double d = (hp ? 6.0*dl*dl : 0.0) + (hs ? 2.0*ds*ds : 0.0) + de*de;
        if (d < best_d) { best_d = d; best = jt->first; }
    }
    return best;
}

st_online_rate_model& cl_rate_optimizer::online_model(int cfg, bool is_nb)
{
    return (is_nb ? online_nb : online_wb)[cfg];
}

const st_online_rate_model* cl_rate_optimizer::online_model_const(int cfg, bool is_nb) const
{
    const std::map<int, st_online_rate_model>& m = is_nb ? online_nb : online_wb;
    std::map<int, st_online_rate_model>::const_iterator it = m.find(cfg);
    return it == m.end() ? NULL : &it->second;
}

void cl_rate_optimizer::maybe_note_channel_change(double snr_db, double selectivity)
{
    double shock = 0.0;
    bool changed = false;
    bool worsened = false;
    bool improved = false;
    if (std::isfinite(snr_db) && snr_db > -90.0) {
        if (last_context_snr_db > -90.0) {
            const double signed_delta = snr_db - last_context_snr_db;
            const double d = std::fabs(signed_delta) /
                             std::max(0.5, policy.channel_change_snr_db);
            shock = std::max(shock, d);
            if (d >= 1.0) {
                changed = true;
                if (signed_delta < 0.0) worsened = true;
                if (signed_delta > 0.0) improved = true;
            }
        }
        last_context_snr_db = snr_db;
    }
    if (std::isfinite(selectivity) && selectivity >= 0.0) {
        if (last_context_selectivity >= 0.0) {
            const double signed_delta = selectivity - last_context_selectivity;
            const double d = std::fabs(signed_delta) /
                             std::max(0.01, policy.channel_change_selectivity);
            shock = std::max(shock, d);
            if (d >= 1.0) {
                changed = true;
                // Larger selectivity is the adverse direction.
                if (signed_delta > 0.0) worsened = true;
                if (signed_delta < 0.0) improved = true;
            }
        }
        last_context_selectivity = selectivity;
    }
    const double bounded = std::min(4.0, shock);
    context_volatility += policy.context_volatility_alpha *
                          (bounded - context_volatility);
    if (changed) {
        ++context_generation;
        // A materially new channel invalidates old negative probe evidence;
        // allow candidates to be reconsidered, but after a negative shock do
        // not immediately spend airtime probing faster modes before the safer
        // side of the ladder has been re-evaluated.
        probe_cooldown_remaining = 0;
        if (worsened && !improved)
            upward_probe_suppressed_until_tick = std::max(
                upward_probe_suppressed_until_tick,
                tick_counter + policy.probe_cooldown_batches);
        else if (improved && !worsened)
            upward_probe_suppressed_until_tick = 0;
        std::printf("[GEARSHIFT-V2] channel-regime-change generation=%d snr=%.2f sel=%.4f volatility=%.3f direction=%s\n",
                    context_generation, snr_db, selectivity, context_volatility,
                    worsened && !improved ? "worse" : (improved && !worsened ? "better" : "mixed"));
        std::fflush(stdout);
    }
}

void cl_rate_optimizer::maybe_note_outcome_change(
        int cfg, bool is_nb, bool failed, double sample_bps)
{
    const st_online_rate_model* m = online_model_const(cfg, is_nb);
    if (!m || (m->outcome_samples <= 0 && !m->initialized)) return;

    double shock = 0.0;
    bool changed = false;
    bool worsened = false;
    bool improved = false;
    if (failed && m->consecutive_failures >= 1 && m->failure_ewma < 0.75) {
        // Two consecutive full failures after a previously usable state are
        // sufficient evidence of a new regime even when no SNR telemetry exists.
        const double d = std::max(1.0, 1.0 + (double)m->consecutive_failures * 0.5);
        shock = std::max(shock, d);
        changed = true;
        worsened = true;
    }
    if (sample_bps >= 0.0 && m->initialized && m->ewma_application_bps > 1.0) {
        const double signed_delta = sample_bps - m->ewma_application_bps;
        const double rel = std::fabs(signed_delta) /
                           std::max(1.0, m->ewma_application_bps);
        const double d_rel = rel / std::max(0.05, policy.outcome_change_rel_threshold);
        double d_sigma = 0.0;
        if (m->application_samples >= policy.outcome_change_min_samples) {
            const double sigma = std::sqrt(std::max(1.0, m->ewma_variance));
            d_sigma = std::fabs(signed_delta) /
                      std::max(1.0, sigma * policy.outcome_change_sigma_threshold);
        }
        const double d = std::max(d_rel, d_sigma);
        shock = std::max(shock, d);
        if (d >= 1.0) {
            changed = true;
            if (signed_delta < 0.0) worsened = true;
            if (signed_delta > 0.0) improved = true;
        }
    }

    const double bounded = std::min(4.0, shock);
    context_volatility += policy.context_volatility_alpha *
                          (bounded - context_volatility);
    if (changed) {
        ++context_generation;
        probe_cooldown_remaining = 0;
        if (worsened && !improved)
            upward_probe_suppressed_until_tick = std::max(
                upward_probe_suppressed_until_tick,
                tick_counter + policy.probe_cooldown_batches);
        else if (improved && !worsened)
            upward_probe_suppressed_until_tick = 0;
        std::printf("[GEARSHIFT-V2] outcome-driven-regime-change generation=%d cfg=%d failed=%d sample=%.1f volatility=%.3f direction=%s\n",
                    context_generation, cfg, failed ? 1 : 0, sample_bps,
                    context_volatility,
                    worsened && !improved ? "worse" : (improved && !worsened ? "better" : "mixed"));
        std::fflush(stdout);
    }
}

bool cl_rate_optimizer::probe_target_blocked(int cfg, const st_rate_observation& obs) const
{
    const std::map<int, st_probe_memory>& pm = obs.is_nb ? probe_memory_nb : probe_memory_wb;
    std::map<int, st_probe_memory>::const_iterator it = pm.find(cfg);
    if (it == pm.end() || tick_counter >= it->second.blocked_until_tick) return false;
    if (it->second.context_generation != context_generation) return false;
    const bool snr_improved = obs.forward_snr_db > -90.0 && it->second.failed_snr_db > -90.0 &&
        obs.forward_snr_db >= it->second.failed_snr_db + policy.probe_reopen_snr_db;
    const bool sel_improved = obs.forward_selectivity >= 0.0 &&
        it->second.failed_selectivity >= 0.0 &&
        obs.forward_selectivity <= it->second.failed_selectivity - policy.probe_reopen_selectivity;
    return !(snr_improved || sel_improved);
}

void cl_rate_optimizer::remember_failed_probe(
        int cfg, const st_rate_observation& obs, bool strong, const char* reason)
{
    std::map<int, st_probe_memory>& pm = obs.is_nb ? probe_memory_nb : probe_memory_wb;
    st_probe_memory& m = pm[cfg];
    int backoff = policy.probe_soft_cooldown_batches;
    if (strong) {
        ++m.failures;
        // Hard failure and well-established economic underperformance earn
        // progressive target-specific backoff. Weak/ambiguous probation
        // timeouts do not increase the progressive failure count.
        const int mult = std::min(4, m.failures);
        backoff = policy.probe_cooldown_batches * mult;
    }
    m.blocked_until_tick = tick_counter + std::max(0, backoff);
    m.failed_snr_db = obs.forward_snr_db;
    m.failed_selectivity = obs.forward_selectivity;
    m.context_generation = context_generation;
    std::printf("[GEARSHIFT-V2] probe-memory target=%d strength=%s reason=%s "
                "backoff_batches=%d failures=%d generation=%d\n",
                cfg, strong ? "strong" : "weak", reason ? reason : "unknown",
                std::max(0, backoff), m.failures, context_generation);
    std::fflush(stdout);
}

void cl_rate_optimizer::reset_probe_evidence_for_generation(
        const char* reason, bool triggering_transaction)
{
    if (!probe_active) return;
    const st_online_rate_model* to = online_model_const(probe_target_cfg, switch_is_nb);
    const bool pending_prechange_unit = to && to->pending_cycle_ms > 0.0;
    probe_context_generation = context_generation;
    probe_application_samples = 0;
    probe_outcome_samples = 0;
    probe_failed_outcomes = 0;
    probe_consecutive_failures = 0;
    probe_progress_events = 0;
    probe_application_bps_sum = 0.0;
    probe_application_bps_ewma = 0.0;
    probe_channel_ms = 0.0;
    probe_cycle_samples = 0;
    probe_cycle_ms_ewma = 0.0;
    // If the regime boundary was discovered by this transaction (or an
    // application unit was already open when the reset occurred), the next
    // completion is causally mixed across the unknown boundary.  It may update
    // the general online model, but it cannot satisfy fresh probe probation.
    probe_pending_generation_contamination = triggering_transaction || pending_prechange_unit;
    probe_max_channel_ms = probe_initial_channel_ms > 0.0
                         ? probe_initial_channel_ms : policy.probe_max_probation_ms;
    probe_result_ready = false;
    probe_ladder_step = false;
    probe_start_application_samples = to ? to->application_samples : 0;
    probe_start_outcome_samples = to ? to->outcome_samples : 0;
    std::printf("[GEARSHIFT-V2] probe-generation-reset target=%d generation=%d reason=%s "
                "app_n=0 outcome_n=0 fail_n=0 age_ms=0 max_ms=%.0f "
                "exclude_trigger=%d pending_unit=%d\n",
                probe_target_cfg, context_generation, reason ? reason : "channel-change",
                probe_max_channel_ms, triggering_transaction ? 1 : 0,
                probe_pending_generation_contamination ? 1 : 0);
    std::fflush(stdout);
}

void cl_rate_optimizer::maybe_extend_probe_budget(const char* reason)
{
    if (!probe_active || !probe_confirmed) return;
    const double hard_ceiling = std::max(1000.0, policy.probe_max_probation_ms);
    double required_ms = probe_max_channel_ms;
    const int extra = std::max(0, policy.probe_budget_extra_cycles);
    const double safety = std::max(1.0, policy.probe_budget_safety_factor);

    if (probe_cycle_ms_ewma > 0.0 && probe_outcome_samples > 0) {
        const int remaining = std::max(0, policy.probe_min_outcome_samples - probe_outcome_samples);
        required_ms = std::max(required_ms, probe_channel_ms +
            probe_cycle_ms_ewma * (double)(remaining + extra) * safety);
    }
    if (probe_application_samples > 0 && probe_channel_ms > 0.0) {
        // Completed application units can arrive more slowly than transaction
        // outcomes.  Their observed average inter-arrival time therefore gets
        // its own reachability calculation.
        const double app_interval_ms = probe_channel_ms / (double)probe_application_samples;
        const int remaining = std::max(0, policy.probe_min_application_samples - probe_application_samples);
        required_ms = std::max(required_ms, probe_channel_ms +
            app_interval_ms * (double)(remaining + extra) * safety);
    }

    const double extended_ms = std::min(hard_ceiling, required_ms);
    if (extended_ms > probe_max_channel_ms + 0.5) {
        const double old_ms = probe_max_channel_ms;
        probe_max_channel_ms = extended_ms;
        const double app_interval_ms = probe_application_samples > 0
            ? probe_channel_ms / (double)probe_application_samples : 0.0;
        std::printf("[GEARSHIFT-V2] probe-budget-extend target=%d reason=%s "
                    "age_ms=%.0f old_ms=%.0f new_ms=%.0f ceiling_ms=%.0f "
                    "cycle_ms=%.0f app_interval_ms=%.0f app_n=%d/%d outcome_n=%d/%d\n",
                    probe_target_cfg, reason ? reason : "observed-cadence",
                    probe_channel_ms, old_ms, probe_max_channel_ms, hard_ceiling,
                    probe_cycle_ms_ewma, app_interval_ms,
                    probe_application_samples, policy.probe_min_application_samples,
                    probe_outcome_samples, policy.probe_min_outcome_samples);
        std::fflush(stdout);
    }
}

void cl_rate_optimizer::observe_transaction(
        int cfg, unsigned int application_bytes, unsigned int transport_bytes,
        unsigned int cycle_ms, unsigned int frames_acked, unsigned int frames_sent,
        bool sack_used, bool failed, double forward_snr_db,
        int forward_snr_age_batches, double forward_selectivity,
        int batch_size, bool is_nb, int forward_selectivity_age_batches,
        unsigned long long observation_ms)
{
    ++tick_counter;
    if (observation_ms > 0)
        observation_time_ms = std::max(observation_time_ms, observation_ms);
    else if (cycle_ms > 0)
        observation_time_ms += (unsigned long long)cycle_ms;
    if (forward_selectivity_age_batches < 0)
        forward_selectivity_age_batches = forward_snr_age_batches;
    const bool snr_fresh = forward_snr_age_batches <= policy.forward_quality_max_age_batches;
    const bool sel_fresh = forward_selectivity_age_batches <= policy.forward_quality_max_age_batches;
    const int generation_before_quality = context_generation;
    if (snr_fresh || sel_fresh)
        maybe_note_channel_change(snr_fresh ? forward_snr_db : -99.9,
                                  sel_fresh ? forward_selectivity : -1.0);
    const bool transaction_discovered_generation =
        context_generation != generation_before_quality;

    const bool target_probe_tx = probe_active && probe_confirmed &&
                                 cfg == probe_target_cfg && is_nb == switch_is_nb;
    if (target_probe_tx && probe_context_generation != context_generation)
        reset_probe_evidence_for_generation("measured-channel-change",
                                            transaction_discovered_generation);
    const bool clean_probe_tx = target_probe_tx && !transaction_discovered_generation;

    // Outcome dynamics are a second regime sensor. During an active probe the
    // target's probation-local population owns interpretation of its own
    // failures/rate swings; otherwise the very experiment being measured can
    // self-trigger a global regime reset and erase its first sample. Physical
    // SNR/selectivity changes above still reset probation immediately.
    if (!target_probe_tx)
        maybe_note_outcome_change(cfg, is_nb, failed, -1.0);

    st_online_rate_model& m = online_model(cfg, is_nb);
    if ((m.outcome_samples > 0 || m.initialized) &&
        m.context_generation != context_generation) {
        // Prediction history is stale, but exact not-yet-committed application
        // accounting is not. Preserve those causal costs across a regime marker.
        const double pending_cycle = m.pending_cycle_ms;
        const double pending_transport = m.pending_transport_bytes;
        const double batch = m.batch_size_ewma;
        m = st_online_rate_model();
        m.pending_cycle_ms = pending_cycle;
        m.pending_transport_bytes = pending_transport;
        m.batch_size_ewma = batch;
        m.context_generation = context_generation;
    }
    const double a = policy.online_ewma_alpha;
    const double frame_ok = frames_sent > 0
                          ? (double)std::min(frames_acked, frames_sent)/(double)frames_sent
                          : (failed ? 0.0 : 1.0);
    const double partial = (sack_used && frames_sent > 0) ? 1.0 - frame_ok : 0.0;
    if (m.outcome_samples == 0) {
        m.failure_ewma = failed ? 1.0 : 0.0;
        m.sack_ewma = sack_used ? 1.0 : 0.0;
        m.frame_success_ewma = frame_ok;
        m.partial_loss_ewma = partial;
        m.batch_size_ewma = batch_size > 0 ? (double)batch_size : 0.0;
    } else {
        m.failure_ewma += a * ((failed ? 1.0 : 0.0) - m.failure_ewma);
        m.sack_ewma += a * ((sack_used ? 1.0 : 0.0) - m.sack_ewma);
        m.frame_success_ewma += a * (frame_ok - m.frame_success_ewma);
        if (sack_used) m.partial_loss_ewma += a * (partial - m.partial_loss_ewma);
        if (batch_size > 0) {
            if (m.batch_size_ewma <= 0.0) m.batch_size_ewma = (double)batch_size;
            else m.batch_size_ewma += a * ((double)batch_size - m.batch_size_ewma);
        }
    }
    if (failed) ++m.consecutive_failures;
    else m.consecutive_failures = 0;
    ++m.outcome_samples;
    m.last_tick = tick_counter;
    m.last_observation_ms = observation_time_ms;
    m.context_generation = context_generation;

    if (cycle_ms > 0) {
        m.pending_cycle_ms += cycle_ms;
        m.pending_transport_bytes += transport_bytes;
        const double tr = (double)transport_bytes * 8000.0 / (double)cycle_ms;
        if (!m.initialized) m.ewma_transport_bps = tr;
        else m.ewma_transport_bps += a * (tr - m.ewma_transport_bps);
    }
    if (clean_probe_tx) {
        ++probe_outcome_samples;
        if (failed) {
            ++probe_failed_outcomes;
            ++probe_consecutive_failures;
        } else {
            probe_consecutive_failures = 0;
        }
        if (cycle_ms > 0) {
            probe_channel_ms += (double)cycle_ms;
            if (probe_cycle_samples == 0) probe_cycle_ms_ewma = (double)cycle_ms;
            else probe_cycle_ms_ewma += policy.online_ewma_alpha *
                                        ((double)cycle_ms - probe_cycle_ms_ewma);
            ++probe_cycle_samples;
        }
        if (frames_acked > 0 || application_bytes > 0 || transport_bytes > 0)
            ++probe_progress_events;
        probe_result_ready = true; // evidence changed; evaluate_v2 classifies it.
        maybe_extend_probe_budget("observed-target-cycle");
    }
    if (application_bytes > 0)
        commit_application_unit(cfg, application_bytes, is_nb);
}

void cl_rate_optimizer::apply_application_sample(st_online_rate_model& m,
                                                  double sample_bps)
{
    const double sample = std::max(0.0, sample_bps);
    const double a = policy.online_ewma_alpha;
    if (!m.initialized) {
        m.ewma_application_bps = sample;
        const double floor = std::max(policy.prior_min_sigma_bps,
                                      std::max(1.0, sample) * policy.prior_default_rel_sigma);
        m.ewma_variance = floor * floor;
        m.initialized = true;
    } else {
        const double d = sample - m.ewma_application_bps;
        m.ewma_application_bps += a * d;
        m.ewma_variance = (1.0-a) * (m.ewma_variance + a*d*d);
    }
    m.last_application_sample_bps = sample;
    ++m.application_samples;
    m.last_tick = tick_counter;
    m.last_observation_ms = observation_time_ms;
    m.context_generation = context_generation;
}

void cl_rate_optimizer::commit_application_unit(int final_cfg,
                                                 unsigned int application_bytes,
                                                 bool is_nb)
{
    if (application_bytes == 0) return;
    std::map<int, st_online_rate_model>& models = is_nb ? online_nb : online_wb;

    struct contribution { int cfg; double cycle; double transport; };
    std::vector<contribution> parts;
    double total_transport = 0.0;
    double total_cycle = 0.0;
    for (std::map<int, st_online_rate_model>::iterator it=models.begin();
         it!=models.end(); ++it) {
        if (it->second.pending_cycle_ms <= 0.0) continue;
        contribution c;
        c.cfg = it->first;
        c.cycle = it->second.pending_cycle_ms;
        c.transport = std::max(0.0, it->second.pending_transport_bytes);
        parts.push_back(c);
        total_transport += c.transport;
        total_cycle += c.cycle;
    }
    if (parts.empty()) return;

    // Unit conversion must be learned from the complete atomic unit.  A rolling
    // window can reset at SET_CONFIG and would then divide the full original
    // application bytes by only the final config's compressed transport bytes.
    if (total_transport > 0.0) {
        const double gain = clamp_double((double)application_bytes / total_transport, 0.25, 16.0);
        if (!atomic_application_transport_gain_valid)
            atomic_application_transport_gain = gain;
        else
            atomic_application_transport_gain += policy.online_ewma_alpha *
                                                 (gain - atomic_application_transport_gain);
        atomic_application_transport_gain_valid = true;
        ++atomic_application_transport_gain_samples;
    }

    // The application-unit goodput itself is a regime-change sensor that works
    // even with no SNR report.  Detect before updating any per-config EWMA.
    const double unit_bps = total_cycle > 0.0
                          ? (double)application_bytes * 8000.0 / total_cycle : 0.0;
    const int generation_before = context_generation;
    // Compare like with like: a unit spanning several configs has a blended
    // completion rate and must not be compared against the final config's
    // per-action EWMA as though it were a same-action sample.
    if (parts.size() == 1 &&
        !(probe_active && final_cfg == probe_target_cfg && is_nb == switch_is_nb))
        maybe_note_outcome_change(final_cfg, is_nb, false, unit_bps);
    const bool regime_changed = context_generation != generation_before;

    for (size_t i=0; i<parts.size(); ++i) {
        st_online_rate_model& m = online_model(parts[i].cfg, is_nb);
        if (regime_changed && m.context_generation != context_generation) {
            // Preserve only the current unit's exact accounting while removing
            // stale prediction history from the previous regime.
            const double cyc = parts[i].cycle;
            const double tr = parts[i].transport;
            const double batch = m.batch_size_ewma;
            m = st_online_rate_model();
            m.pending_cycle_ms = cyc;
            m.pending_transport_bytes = tr;
            m.batch_size_ewma = batch;
            m.context_generation = context_generation;
        }
        double share = 0.0;
        if (total_transport > 0.0)
            share = parts[i].transport / total_transport;
        else if (parts[i].cfg == final_cfg)
            share = 1.0;
        const double credited = (double)application_bytes * share;
        const double sample = parts[i].cycle > 0.0
                            ? credited * 8000.0 / parts[i].cycle : 0.0;
        apply_application_sample(m, sample);
        m.pending_cycle_ms = 0.0;
        m.pending_transport_bytes = 0.0;

        // Probation counts only pure target application units. A unit that
        // straddles SET_CONFIG still updates the per-action online model above,
        // but is transition-contaminated evidence and must not satisfy the
        // "three completed target samples" acceptance bar.
        const bool target_part = probe_active && probe_confirmed &&
            parts[i].cfg == probe_target_cfg && is_nb == switch_is_nb;
        if (target_part && probe_context_generation != context_generation)
            reset_probe_evidence_for_generation("application-generation-change");

        if (target_part && probe_pending_generation_contamination) {
            // This completed unit contains channel time from the transaction that
            // exposed the generation boundary (or older pending time).  General
            // online attribution above is still useful, but fresh probation must
            // start with the next causally clean target unit.
            probe_pending_generation_contamination = false;
            std::printf("[GEARSHIFT-V2] probe-generation-unit-excluded target=%d "
                        "generation=%d sample=%.1f reason=boundary-straddling-unit\n",
                        probe_target_cfg, context_generation, sample);
            std::fflush(stdout);
        } else if (target_part && parts.size() == 1) {
            ++probe_application_samples;
            probe_application_bps_sum += sample;
            if (probe_application_samples == 1) probe_application_bps_ewma = sample;
            else probe_application_bps_ewma += policy.online_ewma_alpha *
                                               (sample - probe_application_bps_ewma);
            probe_result_ready = true;
            maybe_extend_probe_budget("observed-application-cadence");
        }
    }
}

void cl_rate_optimizer::add_application_credit(int cfg,
                                                unsigned int application_bytes,
                                                bool is_nb)
{
    // Compatibility API: one atomic unit may have crossed configurations, so
    // all callers use the same distributed attribution path.
    commit_application_unit(cfg, application_bytes, is_nb);
}

const st_rate_cell* cl_rate_optimizer::nearest_snr_cell(
        int cfg, double snr_db, bool is_nb, const st_rate_cell** upper,
        double* mix) const
{
    if (upper) *upper = NULL;
    if (mix) *mix = 0.0;
    const std::map<int, std::map<std::string, st_rate_cell> >& active =
        is_nb ? table_nb : table;
    std::map<int, std::map<std::string, st_rate_cell> >::const_iterator cit = active.find(cfg);
    if (cit == active.end()) return NULL;
    std::vector<const st_rate_cell*> cells;
    for (std::map<std::string, st_rate_cell>::const_iterator jt = cit->second.begin();
         jt != cit->second.end(); ++jt)
        if (jt->second.valid && jt->second.snr_hint_db > -900.0)
            cells.push_back(&jt->second);
    if (cells.empty()) return NULL;
    std::sort(cells.begin(), cells.end(), [](const st_rate_cell* a, const st_rate_cell* b){
        return a->snr_hint_db < b->snr_hint_db;
    });
    if (snr_db <= cells.front()->snr_hint_db) return cells.front();
    if (snr_db >= cells.back()->snr_hint_db) return cells.back();
    for (size_t i=1; i<cells.size(); ++i) {
        if (snr_db <= cells[i]->snr_hint_db) {
            const st_rate_cell* lo = cells[i-1];
            const st_rate_cell* hi = cells[i];
            if (upper) *upper = hi;
            if (mix) *mix = (snr_db - lo->snr_hint_db) /
                            (hi->snr_hint_db - lo->snr_hint_db);
            return lo;
        }
    }
    return cells.back();
}

st_rate_prediction cl_rate_optimizer::context_calibration_prediction(
        int cfg, const st_rate_observation& obs, bool use_reverse_snr) const
{
    st_rate_prediction out;
    if (!calibration_loaded || (obs.is_nb && !nb_enabled)) return out;
    const std::map<int, std::map<std::string, st_rate_cell> >& active =
        obs.is_nb ? table_nb : table;
    std::map<int, std::map<std::string, st_rate_cell> >::const_iterator cit = active.find(cfg);
    if (cit == active.end()) return out;

    const double snr = use_reverse_snr ? obs.reverse_snr_db : obs.forward_snr_db;
    const bool have_snr = std::isfinite(snr) && snr > -90.0;
    const bool have_sel = !use_reverse_snr && std::isfinite(obs.forward_selectivity) &&
                          obs.forward_selectivity >= 0.0;
    if (!have_snr && !have_sel) return out;

    bool row_has_snr = false, row_has_sel = false;
    double min_snr=1.0e30, max_snr=-1.0e30, min_sel=1.0e30, max_sel=-1.0e30;
    for (std::map<std::string, st_rate_cell>::const_iterator jt=cit->second.begin();
         jt!=cit->second.end(); ++jt) {
        if (!jt->second.valid) continue;
        row_has_snr |= jt->second.snr_hint_db > -900.0;
        row_has_sel |= jt->second.selectivity_valid;
        if (jt->second.snr_hint_db > -900.0) {
            min_snr=std::min(min_snr,jt->second.snr_hint_db);
            max_snr=std::max(max_snr,jt->second.snr_hint_db);
        }
        if (jt->second.selectivity_valid) {
            min_sel=std::min(min_sel,jt->second.selectivity_mean);
            max_sel=std::max(max_sel,jt->second.selectivity_mean);
        }
    }
    const bool use_snr_dim = have_snr && row_has_snr;
    const bool use_sel_dim = have_sel && row_has_sel;
    if (!use_snr_dim && !use_sel_dim) return out;
    // A one-dimensional SNR calibration should interpolate its two bracketing
    // cells exactly rather than kernel-smooth distant operating points into an
    // exact measured coordinate.  Keep the kernel model for genuine 2-D
    // SNR+selectivity surfaces (and selectivity-only rows).
    if (use_snr_dim && !use_sel_dim) return out;

    double sw=0.0, sm=0.0, second=0.0, sd2=0.0, sbatch=0.0, sbatchw=0.0;
    for (std::map<std::string, st_rate_cell>::const_iterator jt=cit->second.begin();
         jt!=cit->second.end(); ++jt) {
        const st_rate_cell& c=jt->second;
        if (!c.valid) continue;
        // When the row has measurements for a requested dimension, cells that
        // lack that dimension are not silently treated as perfect matches.
        if (use_snr_dim && c.snr_hint_db <= -900.0) continue;
        if (use_sel_dim && !c.selectivity_valid) continue;
        double d2=0.0;
        if (use_snr_dim) {
            const double z=(c.snr_hint_db-snr)/std::max(0.01,policy.calibration_snr_kernel_db);
            d2 += z*z;
        }
        if (use_sel_dim) {
            const double z=(c.selectivity_mean-obs.forward_selectivity)/
                           std::max(0.001,policy.calibration_selectivity_kernel);
            d2 += z*z;
        }
        const double kernel=std::exp(-0.5*d2);
        if (kernel < 1e-8) continue;
        double runs=std::min(policy.prior_run_weight_cap,
                              (double)std::max(1,c.total_runs()));
        const double mean=c.survival_adjusted_mean();
        double sig=c.eff_bps_sigma>0.0 ? c.eff_bps_sigma :
            std::max(policy.prior_min_sigma_bps,
                     std::fabs(mean)*policy.prior_default_rel_sigma);
        apply_break_risk_to_prior(&c, sig, runs);
        const double w=kernel*runs;
        sw += w;
        sm += w*mean;
        second += w*(sig*sig + mean*mean);
        sd2 += w*d2;
        if(c.batch_size_valid) { sbatch += w*c.batch_size_mean; sbatchw += w; }
    }
    if (sw <= 1e-8) return out;
    const double mean=sm/sw;
    double var=std::max(0.0, second/sw - mean*mean);
    double sigma=std::sqrt(var);
    const double rms_distance=std::sqrt(std::max(0.0,sd2/sw));
    sigma *= 1.0 + policy.calibration_context_distance_sigma_gain*rms_distance;
    double support_distance = 0.0;
    if (use_snr_dim && min_snr <= max_snr) {
        if (snr < min_snr) support_distance = std::max(support_distance, (min_snr-snr)/std::max(0.5,policy.calibration_snr_kernel_db));
        if (snr > max_snr) support_distance = std::max(support_distance, (snr-max_snr)/std::max(0.5,policy.calibration_snr_kernel_db));
    }
    if (use_sel_dim && min_sel <= max_sel) {
        if (obs.forward_selectivity < min_sel) support_distance = std::max(support_distance, (min_sel-obs.forward_selectivity)/std::max(0.01,policy.calibration_selectivity_kernel));
        if (obs.forward_selectivity > max_sel) support_distance = std::max(support_distance, (obs.forward_selectivity-max_sel)/std::max(0.01,policy.calibration_selectivity_kernel));
    }
    if (support_distance > 0.0) sigma *= 1.0 + support_distance;
    double prior_weight=std::min(policy.prior_run_weight_cap,sw) * calibration_prior_weight_scale;
    if (support_distance > 0.0) prior_weight /= 1.0 + support_distance*support_distance;
    if (use_reverse_snr) {
        sigma *= policy.reverse_snr_sigma_gain;
        prior_weight *= policy.reverse_snr_prior_weight_scale;
        out.source="reverse-context-prior";
    } else {
        out.source=(use_snr_dim && use_sel_dim) ? "snr-selectivity-context-prior" :
                   (use_snr_dim ? "snr-context-prior" : "selectivity-context-prior");
    }
    out.valid=true;
    out.mean_bps=std::max(0.0,mean);
    out.sigma_bps=std::max(policy.prior_min_sigma_bps,sigma);
    out.prior_weight=std::max(0.02,prior_weight);
    if (support_distance > 0.0) out.source += "+extrapolated";
    if(sbatchw > 0.0) apply_calibration_batch_context(out, sbatch/sbatchw, cfg, obs);
    // A forward-context empirical surface can justify a direct action only
    // inside measured support and when calibration identity is trusted.
    out.direct_evidence = !use_reverse_snr && support_distance <= 0.0;
    apply_calibration_trust(out);
    return out;
}

void cl_rate_optimizer::apply_calibration_batch_context(
        st_rate_prediction& p, double calibrated_batch_size, int cfg,
        const st_rate_observation& obs) const
{
    if(!p.valid || calibrated_batch_size <= 0.0) return;
    int candidate_batch = obs.batch_size;
    std::map<int,int>::const_iterator it = obs.candidate_batch_size.find(cfg);
    if(it != obs.candidate_batch_size.end() && it->second > 0) candidate_batch = it->second;
    if(candidate_batch <= 0) return;
    const double rel = std::fabs((double)candidate_batch - calibrated_batch_size) /
                       std::max(1.0, calibrated_batch_size);
    const double match = std::exp(-rel / std::max(0.05, policy.batch_context_half_width));
    p.prior_weight *= std::max(0.10, match);
    p.sigma_bps /= std::max(0.25, match);
    if(match < 0.98) p.source += "+batch-context";
}

void cl_rate_optimizer::convert_prior_to_application_units(
        st_rate_prediction& p, const st_rate_observation& obs,
        bool empirical_calibration) const
{
    if (!p.valid) return;

    const bool empirical_raw_transport = empirical_calibration &&
        calibration_compress_known && !calibration_compressed;
    const bool analytic_raw_transport = !empirical_calibration;
    double measured_gain = obs.application_transport_gain;
    bool can_convert = obs.compression_enabled &&
        obs.application_transport_gain_valid &&
        std::isfinite(obs.application_transport_gain) &&
        obs.application_transport_gain > 0.05;
    if (obs.compression_enabled && atomic_application_transport_gain_valid) {
        measured_gain = atomic_application_transport_gain;
        can_convert = std::isfinite(measured_gain) && measured_gain > 0.05;
    }

    if ((empirical_raw_transport || analytic_raw_transport) && can_convert) {
        // Calibration with compression OFF and analytical PHY-capacity hints
        // are transport-byte rates. Convert them into the same original
        // application-byte units as the objective using the actually observed
        // compression gain. This is a unit conversion, not the old uniform
        // implementation-wide scaling heuristic: per-config empirical/live corrections
        // remain independent.
        const double gain = clamp_double(measured_gain, 0.25, 16.0);
        const double old_mean = p.mean_bps;
        const double scaled_sigma = p.sigma_bps * gain;
        const double gain_sigma = std::fabs(old_mean * gain) *
                                  policy.compression_gain_rel_sigma;
        p.mean_bps = old_mean * gain;
        p.sigma_bps = std::sqrt(scaled_sigma*scaled_sigma + gain_sigma*gain_sigma);
        p.source += atomic_application_transport_gain_valid
                  ? "+app-unit-converted-atomic" : "+app-unit-converted";
    } else if (empirical_calibration && calibration_compress_known &&
               calibration_compressed != obs.compression_enabled) {
        // A compressed calibration collected on one payload is not directly
        // comparable to an uncompressed production session (or vice versa)
        // unless a live conversion ratio exists. Keep it as weak information
        // rather than allowing a unit/profile mismatch to dominate.
        p.prior_weight *= 0.20;
        p.sigma_bps *= 2.0;
        p.source += "+compression-mismatch";
    } else if (empirical_calibration && !calibration_compress_known) {
        // Old tables without calibration_setup.compress have ambiguous units.
        p.prior_weight *= 0.50;
        p.sigma_bps *= 1.5;
        p.source += "+compression-semantics-unknown";
    }
}

void cl_rate_optimizer::apply_calibration_trust(st_rate_prediction& p) const
{
    if (!p.valid) return;
    p.sigma_bps = std::max(policy.prior_min_sigma_bps,
                           p.sigma_bps * calibration_prior_sigma_scale);
    if (!calibration_direct_authority) {
        p.direct_evidence = false;
        p.source += "+identity-mismatch-probe-only";
    } else if (calibration_prior_sigma_scale > 1.001) {
        p.source += "+identity-weakened";
    }
}

std::string cl_rate_optimizer::outcome_label(const st_rate_observation& obs) const
{
    return identify_channel_label_legacy(obs.current_cfg, obs.sack_batch_rate,
        obs.application_bps > 0.0 ? obs.application_bps : obs.transport_bps,
        obs.partial_frame_loss_rate, obs.is_nb);
}

st_rate_prediction cl_rate_optimizer::prior_prediction(
        int cfg, const st_rate_observation& obs) const
{
    st_rate_prediction out;
    const st_rate_cell* lo = NULL;
    const st_rate_cell* hi = NULL;
    double mix = 0.0;
    bool lo_from_forward_snr = false;
    bool lo_from_reverse_snr = false;
    bool lo_from_outcome = false;
    double snr_extrapolation_db = 0.0;
    const bool fresh_snr = std::isfinite(obs.forward_snr_db) && obs.forward_snr_db > -90.0 &&
                           obs.forward_snr_age_batches <= policy.forward_quality_max_age_batches;
    const bool fresh_reverse_snr = !fresh_snr && std::isfinite(obs.reverse_snr_db) &&
                                   obs.reverse_snr_db > -90.0 &&
                                   obs.reverse_snr_age_batches <= policy.reverse_quality_max_age_batches;
    if (calibration_loaded && (!obs.is_nb || nb_enabled)) {
        // New v2 tables may contain WGN and multipath cells at the same SNR.
        // Match the continuous measured context jointly (SNR + selectivity)
        // instead of collapsing it to a single channel-name label.
        if (fresh_snr || (std::isfinite(obs.forward_selectivity) && obs.forward_selectivity >= 0.0)) {
            st_rate_prediction cp=context_calibration_prediction(cfg,obs,false);
            if (cp.valid) {
                convert_prior_to_application_units(cp, obs, true);
                return cp;
            }
            // SNR-only calibration rows use exact/bracketing linear
            // interpolation.  This reproduces a measured cell at its own SNR
            // and avoids distant cells biasing a known operating point.
            if (fresh_snr) {
                lo = nearest_snr_cell(cfg, obs.forward_snr_db, obs.is_nb, &hi, &mix);
                lo_from_forward_snr = lo != NULL;
            }
        }
        // Forward selectivity may be available even when this calibration row
        // has no selectivity dimension.  That must not suppress a genuinely
        // fresh reverse-SNR symmetry prior: try it whenever forward-SNR did not
        // resolve an empirical cell.
        if (!lo && fresh_reverse_snr) {
            st_rate_prediction cp=context_calibration_prediction(cfg,obs,true);
            if (cp.valid) {
                convert_prior_to_application_units(cp, obs, true);
                return cp;
            }
            lo = nearest_snr_cell(cfg, obs.reverse_snr_db, obs.is_nb, &hi, &mix);
            lo_from_reverse_snr = lo != NULL;
        }
        // Backward-compatible old tables lacking measured context still use
        // outcome matching; this path is deliberately lower fidelity.
        if (!lo) {
            const std::string label = outcome_label(obs);
            if (!label.empty()) {
                lo = get_cell(cfg, label, obs.is_nb);
                lo_from_outcome = lo != NULL;
            }
        }
    }

    if (lo && lo->valid && !hi && (lo_from_forward_snr || lo_from_reverse_snr)) {
        const double query_snr = lo_from_reverse_snr ? obs.reverse_snr_db : obs.forward_snr_db;
        snr_extrapolation_db = std::fabs(query_snr - lo->snr_hint_db);
    }

    if (lo && lo->valid) {
        const double mean_lo = lo->survival_adjusted_mean();
        const double sigma_lo = lo->eff_bps_sigma > 0.0 ? lo->eff_bps_sigma
            : std::max(policy.prior_min_sigma_bps,
                       std::fabs(mean_lo) * policy.prior_default_rel_sigma);
        double mean = mean_lo;
        double sigma = sigma_lo;
        double runs = std::min(policy.prior_run_weight_cap,
                               (double)std::max(1, lo->total_runs()));
        if (hi && hi->valid) {
            const double mean_hi = hi->survival_adjusted_mean();
            const double sigma_hi = hi->eff_bps_sigma > 0.0 ? hi->eff_bps_sigma
                : std::max(policy.prior_min_sigma_bps,
                           std::fabs(mean_hi) * policy.prior_default_rel_sigma);
            mean = mean_lo + mix * (mean_hi - mean_lo);
            sigma = sigma_lo + mix * (sigma_hi - sigma_lo);
            runs = std::min(runs, std::min(policy.prior_run_weight_cap,
                                          (double)std::max(1, hi->total_runs())));
            out.source = "snr-interpolated-prior";
        } else {
            if (lo_from_forward_snr) out.source = "snr-prior";
            else if (lo_from_reverse_snr) out.source = "reverse-snr-symmetry-prior";
            else if (lo_from_outcome) out.source = "outcome-prior";
            else out.source = "calibration-prior";
        }
        // If either bracketing calibration cell required emergency recovery,
        // reduce its authority instead of pretending the mean alone captures
        // long-horizon stability.  Use the worse bracket conservatively.
        if (hi && hi->valid && hi->break_run_rate_valid &&
            (!lo->break_run_rate_valid || hi->break_run_rate > lo->break_run_rate))
            apply_break_risk_to_prior(hi, sigma, runs);
        else
            apply_break_risk_to_prior(lo, sigma, runs);
        if (obs.forward_selectivity >= 0.0) {
            sigma *= 1.0 + policy.selectivity_uncertainty_gain * obs.forward_selectivity;
            runs /= 1.0 + 4.0 * obs.forward_selectivity;
        }
        if (lo_from_reverse_snr) {
            // HF paths are often approximately reciprocal over one exchange, but
            // reverse SNR is never silently relabelled as forward truth.  It is
            // only a weak cold-start symmetry prior with inflated uncertainty.
            sigma *= policy.reverse_snr_sigma_gain;
            runs *= policy.reverse_snr_prior_weight_scale;
            out.source = "reverse-snr-symmetry-prior";
        }
        if (snr_extrapolation_db > 0.0) {
            const double d = snr_extrapolation_db / std::max(0.5, policy.calibration_snr_kernel_db);
            sigma *= 1.0 + d;
            runs /= 1.0 + d*d;
            out.source += "+extrapolated";
        }
        out.valid = true;
        out.mean_bps = std::max(0.0, mean);
        out.sigma_bps = std::max(policy.prior_min_sigma_bps, sigma);
        out.prior_weight = std::max(0.05, runs * calibration_prior_weight_scale);
        out.direct_evidence = !lo_from_reverse_snr && snr_extrapolation_db <= 0.0;
        double calibrated_batch = lo->batch_size_valid ? lo->batch_size_mean : -1.0;
        if(hi && hi->valid && hi->batch_size_valid && lo->batch_size_valid)
            calibrated_batch = lo->batch_size_mean + mix*(hi->batch_size_mean-lo->batch_size_mean);
        apply_calibration_batch_context(out, calibrated_batch, cfg, obs);
        apply_calibration_trust(out);
        convert_prior_to_application_units(out, obs, true);
        return out;
    }

    // No empirical cell: fall back to the analytical PHY-capacity hint supplied
    // by the controller. This is deliberately weak/high-uncertainty evidence; it
    // makes an uncalibrated mode probeable without pretending its channel success
    // probability is known.
    std::map<int,double>::const_iterator ni = obs.nominal_bps.find(cfg);
    if (ni != obs.nominal_bps.end() && ni->second > 0.0) {
        out.valid = true;
        out.mean_bps = ni->second;
        out.sigma_bps = std::max(policy.prior_min_sigma_bps,
                                 ni->second * policy.uncalibrated_prior_rel_sigma);
        out.prior_weight = policy.uncalibrated_prior_weight;
        if (obs.forward_selectivity >= 0.0) {
            // Analytical capacity knows geometry, not frequency-selective channel
            // viability. Selectivity therefore widens uncertainty rather than
            // pretending a no-loss capacity estimate is channel truth.
            out.sigma_bps *= 1.0 + policy.selectivity_uncertainty_gain * obs.forward_selectivity;
            out.prior_weight /= 1.0 + 4.0 * obs.forward_selectivity;
        }
        out.source = "analytic-uncalibrated-prior";
        convert_prior_to_application_units(out, obs, false);
    }
    return out;
}

st_rate_prediction cl_rate_optimizer::predict_config(
        int cfg, const st_rate_observation& obs) const
{
    st_rate_prediction prior = prior_prediction(cfg, obs);
    st_rate_prediction out = prior;
    const st_online_rate_model* m = online_model_const(cfg, obs.is_nb);
    double live_mean = 0.0, live_sigma = 0.0, live_weight = 0.0;
    int live_samples = 0, age = 1000000000;
    bool live_direct = false;
    bool post_failed_probe_historical = false;

    if (m && m->initialized && m->application_samples > 0) {
        age = std::max(0, tick_counter - m->last_tick);
        const unsigned long long now_ms = obs.monotonic_ms > 0
            ? std::max(obs.monotonic_ms, m->last_observation_ms) : observation_time_ms;
        const double age_ms = now_ms >= m->last_observation_ms
            ? (double)(now_ms - m->last_observation_ms) : 0.0;
        const double effective_half_life_ms = policy.live_stale_half_life_ms /
            (1.0 + policy.context_volatility_live_gain * std::max(0.0, context_volatility));
        double freshness = std::exp(-0.6931471805599453 *
            age_ms / std::max(1.0, effective_half_life_ms));
        // Evidence collected before a detected channel-regime change is not
        // discarded, but becomes a weak historical hint immediately.
        if (m->context_generation != context_generation) freshness *= 0.15;

        // Batch size changes alter ACK amortization and SACK incidence. Do not
        // silently treat evidence from a radically different batch geometry as
        // equally relevant to the present action.
        double batch_match = 1.0;
        int candidate_batch = obs.batch_size;
        std::map<int,int>::const_iterator cbi = obs.candidate_batch_size.find(cfg);
        if (cbi != obs.candidate_batch_size.end() && cbi->second > 0) candidate_batch = cbi->second;
        if (candidate_batch > 0 && m->batch_size_ewma > 0.0) {
            const double rel = std::fabs((double)candidate_batch - m->batch_size_ewma) /
                               std::max(1.0, m->batch_size_ewma);
            batch_match = std::exp(-rel / std::max(0.05, policy.batch_context_half_width));
        }
        live_weight = std::min(policy.live_weight_cap,
                               (double)m->application_samples) * freshness * batch_match;
        live_mean = m->ewma_application_bps;
        live_sigma = std::sqrt(std::max(0.0, m->ewma_variance));
        live_sigma = std::max(live_sigma,
            std::max(policy.prior_min_sigma_bps,
                     std::fabs(live_mean) * 0.15) /
            std::sqrt((double)std::max(1, m->application_samples)));
        if (batch_match < 0.95) live_sigma /= std::max(0.20, batch_match);
        live_sigma *= 1.0 + 0.50 * std::max(0.0, context_volatility);
        live_samples = m->application_samples;
        const double direct_age_ms = now_ms >= m->last_observation_ms
            ? (double)(now_ms - m->last_observation_ms) : 0.0;
        live_direct = m->context_generation == context_generation &&
                      direct_age_ms <= std::max(1.0, policy.live_stale_half_life_ms);

        // A strong rejected probe is an episode-level negative result, not a
        // permanent hard veto. While its target-specific backoff is active the
        // action is excluded by probe_target_blocked(). Once that backoff has
        // expired, retain the measured mean as a weak historical hint but cap
        // its confidence so an untelemetried channel can eventually re-measure
        // the target. This does NOT erase the online samples. If the target is
        // subsequently visited and produces fresh evidence (last_tick advances
        // beyond blocked_until_tick), the cap disappears immediately. A retry
        // in the same context is probe-only: the old calibration/live evidence
        // cannot regain blind direct-switch authority until fresh target data or
        // a material context-generation change re-establishes it.
        const std::map<int, st_probe_memory>& pm =
            obs.is_nb ? probe_memory_nb : probe_memory_wb;
        std::map<int, st_probe_memory>::const_iterator pmi = pm.find(cfg);
        if (pmi != pm.end() && pmi->second.failures > 0 &&
            pmi->second.context_generation == context_generation &&
            tick_counter >= pmi->second.blocked_until_tick &&
            m->last_tick < pmi->second.blocked_until_tick) {
            const double historical_cap = std::max(0.05,
                policy.uncalibrated_prior_weight * 0.75);
            live_weight = std::min(live_weight, historical_cap);
            live_sigma = std::max(live_sigma,
                std::max(policy.prior_min_sigma_bps, std::fabs(live_mean)*0.50));
            live_direct = false;
            post_failed_probe_historical = true;
        }
    }

    // The current configuration has the freshest rolling application-goodput
    // observation. Use it directly, never as a scale factor for other modes.
    if (cfg == obs.current_cfg && live_weight <= 0.0 &&
        obs.application_commits > 0 && std::isfinite(obs.application_bps)) {
        // Bootstrap-only fallback for callers that supplied a rolling objective
        // without transaction-level model state.  Once per-config attribution
        // exists, never overwrite it with a mixed/cross-config rolling window.
        live_weight = std::min(policy.live_weight_cap,
                               (double)std::max(1, obs.application_commits));
        live_mean = std::max(0.0, obs.application_bps);
        live_sigma = std::max(policy.prior_min_sigma_bps,
                              std::fabs(live_mean) * 0.12 /
                              std::sqrt((double)std::max(1, obs.application_commits)));
        live_samples = obs.application_commits;
        age = 0;
        live_direct = true;
    } else if (cfg == obs.current_cfg && live_weight == 0.0 && obs.transport_bps > 0.0) {
        live_weight = 0.5;
        live_mean = obs.transport_bps;
        live_sigma = std::max(policy.prior_min_sigma_bps, live_mean * 0.50);
        live_samples = 0;
        age = 0;
    }

    double pw = prior.valid ? prior.prior_weight : 0.0;
    if (live_direct && prior.source.find("analytic-uncalibrated-prior") != std::string::npos) {
        // Analytical no-loss capacity is only a discovery prior. Once this
        // action has been measured directly, do not let geometry overpower
        // even a single causal goodput observation.
        pw *= 0.10;
    }
    if (pw <= 0.0 && live_weight <= 0.0) return st_rate_prediction();
    if (pw <= 0.0) {
        out.valid = true; out.mean_bps = live_mean; out.sigma_bps = live_sigma;
        out.source = "live";
    } else if (live_weight <= 0.0) {
        out = prior;
    } else {
        const double tw = pw + live_weight;
        out.valid = true;
        out.mean_bps = (pw*prior.mean_bps + live_weight*live_mean) / tw;
        const double between = (prior.mean_bps-live_mean)*(prior.mean_bps-live_mean);
        out.sigma_bps = std::sqrt((pw*prior.sigma_bps*prior.sigma_bps +
                                   live_weight*live_sigma*live_sigma)/tw +
                                  (pw*live_weight/(tw*tw))*between);
        out.source = "prior+live";
    }
    out.prior_weight = pw;
    out.live_weight = live_weight;
    out.live_samples = live_samples;
    out.age_batches = age;
    if (m && (m->initialized || m->outcome_samples > 0)) {
        const unsigned long long now_ms = obs.monotonic_ms > 0
            ? std::max(obs.monotonic_ms, m->last_observation_ms) : observation_time_ms;
        out.age_ms = now_ms >= m->last_observation_ms
            ? (double)(now_ms - m->last_observation_ms) : 0.0;
    }
    out.direct_evidence = out.direct_evidence || live_direct;
    if (post_failed_probe_historical) {
        out.direct_evidence = false;
        out.source += "+post-probe-historical";
    }
    return out;
}

st_rate_prediction cl_rate_optimizer::predict_for_test(
        int cfg, const st_rate_observation& obs) const
{
    return predict_config(cfg, obs);
}

std::vector<int> cl_rate_optimizer::candidate_configs(const st_rate_observation& obs) const
{
    if (!obs.feasible_configs.empty()) {
        std::vector<int> out = obs.feasible_configs;
        std::sort(out.begin(), out.end());
        out.erase(std::unique(out.begin(), out.end()), out.end());
        return out;
    }
    // Compatibility fallback for standalone legacy/unit callers. Production v2
    // always supplies the modem-defined feasible action set.
    std::vector<int> out;
    const std::map<int, std::map<std::string, st_rate_cell> >& t = obs.is_nb ? table_nb : table;
    for (std::map<int, std::map<std::string, st_rate_cell> >::const_iterator it=t.begin();
         it!=t.end(); ++it) out.push_back(it->first);
    const std::map<int, st_online_rate_model>& m = obs.is_nb ? online_nb : online_wb;
    for (std::map<int, st_online_rate_model>::const_iterator it=m.begin(); it!=m.end(); ++it)
        if (std::find(out.begin(), out.end(), it->first) == out.end()) out.push_back(it->first);
    std::sort(out.begin(), out.end());
    return out;
}

double cl_rate_optimizer::useful_horizon_ms(const st_rate_observation& obs,
                                             double current_bps) const
{
    const bool finite_queue_known = (obs.remaining_work_known || obs.queue_bytes > 0) &&
                                    current_bps > 1.0;
    double base = policy.default_horizon_ms;
    if (finite_queue_known)
        // queue_bytes is bytes, current_bps is bits/second: bytes*8 / bps is
        // seconds, then *1000 converts to milliseconds.  Do NOT inflate a
        // genuinely short transfer to min_horizon_ms; doing so could make a
        // switch/probe appear repayable after the transfer would already end.
        base = (double)obs.queue_bytes * 8.0 * 1000.0 / current_bps;
    // A volatile channel invalidates a long payback assumption: demand that a
    // switch repay itself over a shorter causal horizon.
    base /= 1.0 + policy.context_volatility_horizon_gain *
                  std::max(0.0, context_volatility);
    return finite_queue_known
        ? clamp_double(base, 1.0, policy.max_horizon_ms)
        : clamp_double(base, policy.min_horizon_ms, policy.max_horizon_ms);
}

void cl_rate_optimizer::emit_decision(const st_rate_observation& obs,
                                      const st_rate_decision& d) const
{
    // State-changing actions are always visible.  HOLD/ABSTAIN can occur every
    // batch, so verbose idle tracing is opt-in to avoid turning diagnostic I/O
    // into measurable application-goodput overhead.
    const bool state_changing_probe_hold =
        d.action == GEARSHIFT_ACTION_HOLD && d.reason == "probe-accepted";
    if ((d.action == GEARSHIFT_ACTION_HOLD || d.action == GEARSHIFT_ACTION_ABSTAIN) &&
        !policy.trace_idle_decisions && !state_changing_probe_hold)
        return;
    std::printf("[GEARSHIFT-V2] mode=%s action=%s cfg=%d target=%d fallback=%d "
                "current=%.1f+/-%.1f target_pred=%.1f+/-%.1f net=%.1f "
                "switch_ms=%.0f horizon_ms=%.0f fail=%.3f frame_ok=%.3f "
                "fsnr=%.1f fsnr_age=%d rsnr=%.1f rsnr_age=%d sel=%.3f "
                "volatility=%.3f reason=%s\n",
                mode_name(), action_name(d.action), obs.current_cfg, d.target_cfg,
                d.fallback_cfg, d.current_mean_bps, d.current_sigma_bps,
                d.target_mean_bps, d.target_sigma_bps, d.net_target_bps,
                d.switch_cost_ms, d.horizon_ms, obs.failed_batch_rate,
                obs.frame_success_rate, obs.forward_snr_db,
                obs.forward_snr_age_batches, obs.reverse_snr_db,
                obs.reverse_snr_age_batches, obs.forward_selectivity,
                context_volatility, d.reason.c_str());
    std::fflush(stdout);
}

void cl_rate_optimizer::clear_probe()
{
    probe_active = false;
    probe_confirmed = false;
    probe_result_ready = false;
    probe_target_cfg = -1;
    probe_fallback_cfg = -1;
    probe_baseline_bps = 0.0;
    probe_start_application_samples = 0;
    probe_start_outcome_samples = 0;
    probe_context_generation = context_generation;
    probe_baseline_generation = context_generation;
    probe_application_samples = 0;
    probe_outcome_samples = 0;
    probe_failed_outcomes = 0;
    probe_consecutive_failures = 0;
    probe_progress_events = 0;
    probe_application_bps_sum = 0.0;
    probe_application_bps_ewma = 0.0;
    probe_channel_ms = 0.0;
    probe_cycle_samples = 0;
    probe_cycle_ms_ewma = 0.0;
    probe_pending_generation_contamination = false;
    probe_geometry_trial_ms = 0.0;
    probe_initial_channel_ms = 0.0;
    probe_max_channel_ms = 0.0;
    probe_zero_progress_budget_ms = 0.0;
    next_probe_trial_ms = 0.0;
    next_probe_max_channel_ms = 0.0;
    next_probe_zero_progress_budget_ms = 0.0;
    next_probe_ladder_step = false;
}

st_rate_decision cl_rate_optimizer::evaluate_v2(const st_rate_observation& obs,
                                                 int config_ceiling)
{
    st_rate_decision d;
    ladder_probe_accepted_now = false;
    d.current_cfg = obs.current_cfg;
    d.target_cfg = obs.current_cfg;
    d.fallback_cfg = obs.current_cfg;
    d.switch_cost_ms = switch_cost_ewma_ms;

    if (!enabled) { d.action=GEARSHIFT_ACTION_ABSTAIN; d.reason="controller-disabled"; last_v2_decision=d; return d; }
    if (obs.current_cfg < 0) { d.action=GEARSHIFT_ACTION_ABSTAIN; d.reason="invalid-current"; last_v2_decision=d; return d; }

    st_rate_prediction current = predict_config(obs.current_cfg, obs);
    if (!current.valid) {
        current.valid = true;
        current.mean_bps = std::max(0.0, obs.application_bps);
        current.sigma_bps = std::max(policy.prior_min_sigma_bps,
                                     current.mean_bps*0.5);
        current.source = "current-fallback";
    }
    d.current_mean_bps = current.mean_bps;
    d.current_sigma_bps = current.sigma_bps;
    d.horizon_ms = useful_horizon_ms(obs, std::max(1.0, current.mean_bps));

    // One ordinary Axis-1 SET_CONFIG owns the transition lifecycle until its
    // confirmation/failure (or an independent external/emergency authority)
    // terminates it. Do not let a later evaluation overwrite that transaction.
    //
    // Physical Patch-2 evidence also showed that a SET_CONFIG can be dispatched
    // without ever reaching the confirmation callback. Exclusivity therefore
    // needs a terminal timeout: otherwise one lost control transition becomes a
    // permanent CONFIG_0 prison. Expiry is NOT target-performance evidence; the
    // target was never confirmed, so clear the unfinished probe without adding
    // failed-probe memory or cooldown and immediately reopen acquisition.
    if (switch_inflight) {
        const unsigned long long now_ms = obs.monotonic_ms;
        const double feedback_guard_ms = std::max(0.0, obs.feedback_budget_ms) +
                                         2.0*switch_cost_ewma_ms + 5000.0;
        // Last-resort corruption/wedge guard only. Normal ACTIVE CONFIG_TAG
        // transitions terminate from peer-follow SACK confirmation or bounded
        // D4 re-tag failure; wall-clock expiry is not the recovery mechanism.
        const double inflight_timeout_ms =
            std::max(policy.switch_inflight_timeout_ms, feedback_guard_ms);
        const bool clock_valid = now_ms > 0 && switch_started_ms > 0 &&
                                 now_ms >= switch_started_ms;
        const double inflight_age_ms = clock_valid
            ? (double)(now_ms - switch_started_ms) : 0.0;
        const bool inflight_expired = clock_valid &&
                                      inflight_age_ms >= inflight_timeout_ms;

        if (inflight_expired) {
            const int expired_from = switch_from_cfg;
            const int expired_to = switch_to_cfg;
            const int expired_fallback = switch_fallback_cfg;
            const e_gearshift_v2_action expired_action = switch_action;
            if (expired_action == GEARSHIFT_ACTION_PROBE)
                clear_probe();
            switch_inflight = false;
            switch_suppression_logged = false;
            switch_started_ms = 0;
            switch_from_cfg = -1;
            switch_to_cfg = -1;
            switch_action = GEARSHIFT_ACTION_HOLD;
            switch_fallback_cfg = -1;
            switch_is_nb = false;
            std::printf("[GEARSHIFT-V2] switch-inflight-expired source=%d target=%d "
                        "action=%s fallback=%d age_ms=%.0f timeout_ms=%.0f "
                        "target_penalty=0 acquisition=reopened\n",
                        expired_from, expired_to, action_name(expired_action),
                        expired_fallback, inflight_age_ms, inflight_timeout_ms);
            std::fflush(stdout);
            // Fall through and make a fresh decision from current observed state.
        } else {
            d.action = GEARSHIFT_ACTION_HOLD;
            d.target_cfg = obs.current_cfg;
            d.fallback_cfg = switch_fallback_cfg >= 0 ? switch_fallback_cfg : obs.current_cfg;
            d.reason = "switch-inflight";
            d.actionable = false;
            if (!switch_suppression_logged) {
                std::printf("[GEARSHIFT-V2] evaluation-suppressed switch_inflight=1 "
                            "source=%d target=%d action=%s fallback=%d started_ms=%llu "
                            "timeout_ms=%.0f\n",
                            switch_from_cfg, switch_to_cfg, action_name(switch_action),
                            switch_fallback_cfg, switch_started_ms, inflight_timeout_ms);
                std::fflush(stdout);
                switch_suppression_logged = true;
            }
            last_v2_decision = d;
            return d;
        }
    }

    // Upward probes have their own causal probation population. One decoded
    // application unit is useful evidence, but it is not a verdict. A target
    // stays under bounded probation until we have enough completed application
    // samples/outcomes to separate a transient from persistent underperformance,
    // or until hard-failure/time bounds terminate the experiment.
    if (probe_active && obs.current_cfg == probe_target_cfg) {
        if (probe_context_generation != context_generation)
            reset_probe_evidence_for_generation("evaluate-generation-change");

        if (probe_baseline_generation != context_generation) {
            st_rate_prediction fb = predict_config(probe_fallback_cfg, obs);
            if (fb.valid && fb.mean_bps > 0.0) probe_baseline_bps = fb.mean_bps;
            probe_baseline_generation = context_generation;
        }

        const double probe_mean = probe_application_samples > 0
            ? probe_application_bps_sum / (double)probe_application_samples : 0.0;
        const double failure_rate = probe_outcome_samples > 0
            ? (double)probe_failed_outcomes / (double)probe_outcome_samples : 0.0;
        const bool repeated_whole_failures =
            probe_consecutive_failures >= policy.probe_hard_failure_streak;
        const bool established_failure_fraction =
            probe_outcome_samples >= policy.probe_min_outcome_samples &&
            failure_rate >= policy.failure_direct_threshold;
        const bool zero_progress =
            probe_outcome_samples >= 2 && probe_application_samples == 0 &&
            probe_progress_events == 0 &&
            probe_channel_ms >= probe_zero_progress_budget_ms;
        const bool hard_probe_failure = repeated_whole_failures ||
                                        established_failure_fraction ||
                                        zero_progress;
        const bool enough_probation =
            probe_application_samples >= policy.probe_min_application_samples &&
            probe_outcome_samples >= policy.probe_min_outcome_samples;
        const bool probation_timed_out =
            probe_max_channel_ms > 0.0 && probe_channel_ms >= probe_max_channel_ms;

        // Evidence-bearing probation updates are useful diagnostics; repeated
        // idle HOLD polls are not. Keep the latter behind trace mode so GS2
        // observability cannot become measurable goodput overhead.
        if (probe_result_ready || policy.trace_idle_decisions) {
            std::printf("[GEARSHIFT-V2] probe-probation source=%d target=%d fallback=%d "
                        "app_n=%d outcome_n=%d fail_n=%d fail_rate=%.3f mean=%.1f "
                        "ewma=%.1f age_ms=%.0f max_ms=%.0f zero_ms=%.0f generation=%d\n",
                        switch_from_cfg, probe_target_cfg, probe_fallback_cfg,
                        probe_application_samples, probe_outcome_samples,
                        probe_failed_outcomes, failure_rate, probe_mean,
                        probe_application_bps_ewma, probe_channel_ms,
                        probe_max_channel_ms, probe_zero_progress_budget_ms,
                        probe_context_generation);
            std::fflush(stdout);
        }
        probe_result_ready = false;

        // The probe already paid one switch. The stay-vs-rollback question now
        // prices only the rollback transition over remaining useful work.
        const double rollback_factor = d.horizon_ms > 0.0
            ? std::max(0.0, 1.0 - switch_cost_ewma_ms/d.horizon_ms) : 0.0;
        const double fallback_after_rollback = probe_baseline_bps * rollback_factor;
        // A compatibility-safe ladder step may continue after the first clean,
        // completed application unit.  This is still a measured-goodput/SACK
        // verdict: zero/failed progress cannot pass, and a slower target stays
        // in the ordinary multi-sample probation path.  The short confirmation
        // keeps a clean 0->7->13->16 climb within the legacy time envelope.
        const bool ladder_goodput_confirm =
            probe_ladder_step && probe_application_samples >= 1 &&
            probe_outcome_samples >= 1 && probe_failed_outcomes == 0 &&
            probe_progress_events >= 1 &&
            probe_mean >= fallback_after_rollback * policy.probe_rollback_ratio;

        if (hard_probe_failure) {
            remember_failed_probe(probe_target_cfg, obs, true,
                                  zero_progress ? "zero-progress" :
                                  (repeated_whole_failures ? "repeated-whole-failure" :
                                   "persistent-failure-fraction"));
            probe_cooldown_remaining = std::max(probe_cooldown_remaining,
                                                policy.cooldown_batches);
            d.action = GEARSHIFT_ACTION_ROLLBACK;
            d.target_cfg = probe_fallback_cfg;
            d.fallback_cfg = probe_fallback_cfg;
            d.target_mean_bps = probe_baseline_bps;
            d.target_sigma_bps = current.sigma_bps;
            d.net_target_bps = fallback_after_rollback;
            d.reason = "probe-hard-failure";
            d.actionable = mode == GEARSHIFT_V2_ACTIVE;
            last_v2_decision = d;
            emit_decision(obs, d);
            return d;
        }

        if (enough_probation || ladder_goodput_confirm) {
            const bool economically_bad =
                probe_mean < fallback_after_rollback * policy.probe_rollback_ratio;
            if (economically_bad) {
                // This is a statistically established soft loss, not a PHY
                // catastrophe, but it is strong enough to earn normal
                // progressive target-specific backoff.
                remember_failed_probe(probe_target_cfg, obs, true,
                                      "established-underperformance");
                d.action = GEARSHIFT_ACTION_ROLLBACK;
                d.target_cfg = probe_fallback_cfg;
                d.fallback_cfg = probe_fallback_cfg;
                d.target_mean_bps = probe_baseline_bps;
                d.target_sigma_bps = current.sigma_bps;
                d.net_target_bps = fallback_after_rollback;
                d.reason = "probe-soft-underperformance";
                d.actionable = mode == GEARSHIFT_V2_ACTIVE;
                last_v2_decision = d;
                emit_decision(obs, d);
                return d;
            }

            std::printf("[GEARSHIFT-V2] probe-accept source=%d target=%d fallback=%d "
                        "app_n=%d outcome_n=%d fail_n=%d mean=%.1f ewma=%.1f "
                        "age_ms=%.0f generation=%d ladder_confirm=%d\n",
                        switch_from_cfg, probe_target_cfg, probe_fallback_cfg,
                        probe_application_samples, probe_outcome_samples,
                        probe_failed_outcomes, probe_mean, probe_application_bps_ewma,
                        probe_channel_ms, probe_context_generation,
                        ladder_goodput_confirm ? 1 : 0);
            std::fflush(stdout);
            const bool accepted_ladder_step = probe_ladder_step;
            clear_probe();
            cooldown_remaining = accepted_ladder_step ? 0 : policy.cooldown_batches;
            if (accepted_ladder_step) {
                ladder_probe_accepted_now = true;
                // Re-evaluate immediately from this confirmed rung. Returning
                // HOLD here would key one extra full batch before the next
                // decision point, defeating the bounded staged-climb cadence.
                const int next_rung = config_probe_ladder_up(obs.current_cfg, obs.is_nb);
                if (next_rung != obs.current_cfg && next_rung <= config_ceiling)
                    goto ladder_probe_confirmed;
                d.reason = "probe-accepted";
                last_v2_decision = d;
                emit_decision(obs, d);
                return d;
            }
            d.reason = "probe-accepted";
            last_v2_decision = d;
            emit_decision(obs, d);
            return d;
        }

        if (probation_timed_out) {
            // The candidate made some progress but did not produce the minimum
            // population in its geometry-aware time budget. Roll back to avoid
            // indefinite probation, but record only weak evidence: this must not
            // become a long/session-level blacklist.
            remember_failed_probe(probe_target_cfg, obs, false,
                                  "probation-insufficient-evidence");
            d.action = GEARSHIFT_ACTION_ROLLBACK;
            d.target_cfg = probe_fallback_cfg;
            d.fallback_cfg = probe_fallback_cfg;
            d.target_mean_bps = probe_baseline_bps;
            d.target_sigma_bps = current.sigma_bps;
            d.net_target_bps = fallback_after_rollback;
            d.reason = "probe-probation-timeout";
            d.actionable = mode == GEARSHIFT_V2_ACTIVE;
            last_v2_decision = d;
            emit_decision(obs, d);
            return d;
        }

        d.action = GEARSHIFT_ACTION_HOLD;
        d.target_cfg = probe_target_cfg;
        d.fallback_cfg = probe_fallback_cfg;
        d.target_mean_bps = probe_mean;
        d.net_target_bps = probe_mean;
        d.reason = "probe-probation";
        d.actionable = false;
        last_v2_decision = d;
        emit_decision(obs, d);
        return d;
    }

ladder_probe_confirmed:
    const bool hard_departure =
        (obs.current_cfg >= CONFIG_0 && obs.current_cfg <= CONFIG_17 &&
         obs.current_cfg > config_ceiling);
    if (!hard_departure && cooldown_remaining > 0) {
        d.reason = "cooldown";
        last_v2_decision = d;
        return d;
    }
    const bool urgent_failure_evidence =
        obs.outcome_samples >= 2 &&
        obs.failed_batch_rate >= policy.failure_direct_threshold;

    // ACTIVE v2 owns its own acquisition path.  A fresh link must not burn
    // four slow-mode outcomes merely to establish that a dramatically faster
    // feasible action is worth measuring.  One trustworthy timed/current-mode
    // observation is sufficient to enter a bounded experiment when useful
    // work remains and at least one faster action has material predicted upside.
    // SHADOW/LEGACY deliberately do not receive this bypass.
    bool cold_start_acquisition = false;
    const bool meaningful_work = !obs.remaining_work_known || obs.queue_bytes > 0;
    const bool trustworthy_current_observation =
        obs.outcome_samples >= 1 && obs.rate_samples >= 1 &&
        (obs.application_commits >= 1 || obs.application_bps > 0.0 ||
         obs.transport_bps > 0.0);
    if (mode == GEARSHIFT_V2_ACTIVE && !hard_departure &&
        !urgent_failure_evidence && meaningful_work &&
        trustworthy_current_observation &&
        obs.outcome_samples < policy.min_outcome_samples) {
        const std::vector<int> acquisition_candidates = candidate_configs(obs);
        for (size_t i=0; i<acquisition_candidates.size(); ++i) {
            const int cfg = acquisition_candidates[i];
            if (cfg < 0 || cfg == obs.current_cfg ||
                !gearshift_destination_within_ofdm_ceiling(cfg, config_ceiling) ||
                gearshift_action_rank(cfg) <= gearshift_action_rank(obs.current_cfg) ||
                probe_target_blocked(cfg, obs))
                continue;
            const st_rate_prediction p = predict_config(cfg, obs);
            if (p.valid && p.mean_bps >
                current.mean_bps * (1.0 + policy.probe_mean_margin)) {
                cold_start_acquisition = true;
                break;
            }
        }
    }

    if (!hard_departure && !urgent_failure_evidence &&
        !cold_start_acquisition &&
        obs.outcome_samples < policy.min_outcome_samples) {
        d.action = GEARSHIFT_ACTION_ABSTAIN;
        d.reason = "learning-outcomes";
        last_v2_decision = d;
        emit_decision(obs, d);
        return d;
    }
    // Outcome-only aggregate ACK replay is useful reliability evidence, but it
    // has no independent elapsed-time denominator.  Do not make an ordinary
    // performance switch/probe until at least min_rate_samples timed exchanges
    // exist.  Hard admission departures and urgent whole-batch failure remain
    // allowed to move for safety even without a rate sample.
    if (!hard_departure && !urgent_failure_evidence &&
        obs.rate_samples < policy.min_rate_samples) {
        d.action = GEARSHIFT_ACTION_ABSTAIN;
        d.reason = "learning-rate";
        last_v2_decision = d;
        emit_decision(obs, d);
        return d;
    }

    const double switch_factor = d.horizon_ms > 0.0
        ? std::max(0.0, 1.0 - switch_cost_ewma_ms/d.horizon_ms) : 0.0;
    // A probe must be economical even in its bounded failure case, which pays
    // a second switch to return to the fallback.  This is a conservative
    // value-of-information gate: direct switches pay one transition; uncertain
    // experiments must clear the two-transition exposure budget.
    const double current_ucb = current.mean_bps + policy.confidence_z*current.sigma_bps;

    // Rank *actions*, not merely predicted candidate means.  An uncertain
    // high-capacity action whose probe cannot repay itself must not hide a
    // lower-mean direct switch that is both evidenced and profitable.
    struct st_action_proposal {
        bool valid;
        int cfg;
        e_gearshift_v2_action action;
        int fallback_cfg;
        st_rate_prediction prediction;
        double net_bps;
        double utility_bps;
        double probe_trial_ms;
        double probation_budget_ms;
        double zero_progress_budget_ms;
        bool ladder_step;
        std::string reason;
        st_action_proposal()
            : valid(false), cfg(-1), action(GEARSHIFT_ACTION_HOLD), fallback_cfg(-1),
              net_bps(0.0), utility_bps(-1.0), probe_trial_ms(0.0),
              probation_budget_ms(0.0), zero_progress_budget_ms(0.0),
              ladder_step(false) {}
    };

    st_action_proposal chosen;
    bool saw_candidate_prediction = false;
    bool saw_uneconomic_probe = false;
    const std::vector<int> candidates = candidate_configs(obs);
    const int preferred_probe_rung = config_probe_ladder_up(obs.current_cfg, obs.is_nb);
    int next_feasible_probe_rung = -1;
    for (size_t i=0; i<candidates.size(); ++i) {
        const int cfg = candidates[i];
        if (gearshift_action_rank(cfg) <= gearshift_action_rank(obs.current_cfg) ||
            !gearshift_destination_within_ofdm_ceiling(cfg, config_ceiling) ||
            probe_target_blocked(cfg, obs))
            continue;
        if (next_feasible_probe_rung < 0 ||
            gearshift_action_rank(cfg) < gearshift_action_rank(next_feasible_probe_rung))
            next_feasible_probe_rung = cfg;
        if (cfg == preferred_probe_rung) {
            next_feasible_probe_rung = cfg;
            break;
        }
    }
    const bool force_lower_for_failure =
        obs.failed_batch_rate >= policy.failure_direct_threshold;
    for (size_t i=0; i<candidates.size(); ++i) {
        const int cfg = candidates[i];
        if (cfg == obs.current_cfg || cfg < 0 ||
            !gearshift_destination_within_ofdm_ceiling(cfg, config_ceiling)) continue;
        if (force_lower_for_failure &&
            gearshift_action_rank(cfg) >= gearshift_action_rank(obs.current_cfg)) continue;
        // A recently failed faster action is temporarily infeasible in this
        // channel context. Exclude it BEFORE ranking so it cannot hide another
        // useful action.
        if (gearshift_action_rank(cfg) > gearshift_action_rank(obs.current_cfg) &&
            probe_target_blocked(cfg, obs)) continue;

        st_rate_prediction p = predict_config(cfg, obs);
        if (!p.valid) continue;
        saw_candidate_prediction = true;
        const bool upward = gearshift_action_rank(cfg) > gearshift_action_rank(obs.current_cfg);
        const bool lower = gearshift_action_rank(cfg) < gearshift_action_rank(obs.current_cfg);
        if (upward && tick_counter < upward_probe_suppressed_until_tick &&
            !p.direct_evidence) continue;

        const double net = p.mean_bps * switch_factor;
        const double lcb = std::max(0.0,
            net - policy.confidence_z*p.sigma_bps*switch_factor);

        st_action_proposal a;
        a.cfg = cfg;
        a.fallback_cfg = obs.current_cfg;
        a.prediction = p;
        a.net_bps = net;

        if (hard_departure) {
            a.valid = net > 0.0;
            a.action = GEARSHIFT_ACTION_SWITCH;
            a.utility_bps = net;
            a.reason = "current-above-admission-ceiling";
        } else if (force_lower_for_failure && lower && net > 0.0) {
            a.valid = true;
            a.action = GEARSHIFT_ACTION_SWITCH;
            a.utility_bps = net;
            a.reason = "full-failure-direct-downshift";
        } else if (lcb > current_ucb * (1.0 + policy.direct_switch_margin) &&
                   (lower || p.direct_evidence)) {
            a.valid = true;
            a.action = GEARSHIFT_ACTION_SWITCH;
            a.utility_bps = net;
            a.reason = (cold_start_acquisition && upward && p.direct_evidence)
                     ? "cold-start-calibrated-direct"
                     : "confidence-clears-switch-cost";
        } else if (upward && probe_cooldown_remaining == 0 &&
                   !probe_target_blocked(cfg, obs)) {
            // Discovery advances only to the next feasible probe rung.  The
            // complete production action set therefore walks the explicit
            // compatibility ladder; sparse synthetic callers use their next
            // feasible action rather than making a hidden target unreachable.
            if (cfg != next_feasible_probe_rung) continue;
            double probe_airtime_ms = 0.0;
            std::map<int,double>::const_iterator ti = obs.tx_airtime_ms.find(cfg);
            if (ti != obs.tx_airtime_ms.end() && ti->second > 0.0)
                probe_airtime_ms = ti->second;
            const double feedback_ms = std::max(0.0, obs.feedback_budget_ms);
            const double probe_exposure_ms = 2.0*switch_cost_ewma_ms +
                                             probe_airtime_ms + feedback_ms;
            // Probation is bounded in channel time and seeded from target
            // transaction geometry, then may extend from measured target cadence.
            // A conservative listen timeout can make the *desired* population
            // budget exceed the hard ceiling. That must cap the experiment, not
            // veto acquisition before the target is ever measured.
            const double one_trial_ms = std::max(1000.0, probe_airtime_ms + feedback_ms);
            const int required_population = std::max(policy.probe_min_application_samples,
                                                     policy.probe_min_outcome_samples);
            const int budget_cycles = required_population +
                                      std::max(0, policy.probe_budget_extra_cycles);
            const double desired_probation_ms = std::max(
                policy.probe_zero_progress_ms,
                one_trial_ms * (double)budget_cycles *
                std::max(1.0, policy.probe_budget_safety_factor));
            a.probe_trial_ms = one_trial_ms;
            a.probation_budget_ms = std::min(policy.probe_max_probation_ms,
                                              desired_probation_ms);
            a.zero_progress_budget_ms = std::min(
                a.probation_budget_ms,
                std::max(policy.probe_zero_progress_ms, one_trial_ms * 2.0));
            const double candidate_probe_factor = d.horizon_ms > 0.0
                ? std::max(0.0, 1.0 - probe_exposure_ms/d.horizon_ms) : 0.0;
            const double probe_net = p.mean_bps * candidate_probe_factor;
            const bool compatibility_ladder_acquisition =
                cold_start_acquisition && cfg == preferred_probe_rung;
            if (probe_net > current.mean_bps * (1.0 + policy.probe_mean_margin) &&
                (compatibility_ladder_acquisition ||
                 (p.mean_bps + policy.confidence_z*p.sigma_bps) *
                     candidate_probe_factor > current_ucb)) {
                a.valid = true;
                a.action = GEARSHIFT_ACTION_PROBE;
                a.utility_bps = probe_net;
                a.net_bps = probe_net;
                a.ladder_step = compatibility_ladder_acquisition;
                a.reason = cold_start_acquisition
                         ? "cold-start-ladder-probe"
                         : "ladder-information-probe";
            } else {
                saw_uneconomic_probe = true;
            }
        } else if (lower &&
                   net > current.mean_bps * (1.0 + policy.direct_switch_margin)) {
            a.valid = true;
            a.action = GEARSHIFT_ACTION_SWITCH;
            a.utility_bps = net;
            a.reason = "lower-mode-net-goodput";
        }

        if (a.valid && (!chosen.valid || a.utility_bps > chosen.utility_bps))
            chosen = a;
    }

    if (!chosen.valid) {
        d.action = hard_departure ? GEARSHIFT_ACTION_ABSTAIN : GEARSHIFT_ACTION_HOLD;
        if (hard_departure) d.reason = "no-admissible-candidate";
        else if (saw_uneconomic_probe) d.reason = "probe-cost-does-not-pay";
        else d.reason = saw_candidate_prediction ? "gain-does-not-pay" : "no-candidate-evidence";
        last_v2_decision = d;
        emit_decision(obs, d);
        return d;
    }

    d.action = chosen.action;
    d.target_cfg = chosen.cfg;
    d.fallback_cfg = chosen.fallback_cfg;
    d.target_mean_bps = chosen.prediction.mean_bps;
    d.target_sigma_bps = chosen.prediction.sigma_bps;
    d.net_target_bps = chosen.net_bps;
    d.reason = chosen.reason;
    if (d.action == GEARSHIFT_ACTION_PROBE) {
        next_probe_trial_ms = chosen.probe_trial_ms;
        next_probe_max_channel_ms = chosen.probation_budget_ms;
        next_probe_zero_progress_budget_ms = chosen.zero_progress_budget_ms;
        next_probe_ladder_step = chosen.ladder_step;
    } else {
        next_probe_trial_ms = 0.0;
        next_probe_max_channel_ms = 0.0;
        next_probe_zero_progress_budget_ms = 0.0;
        next_probe_ladder_step = false;
    }

    if (d.reason == "full-failure-direct-downshift") {
        // A mode abandoned for a full-batch failure is negative evidence even
        // when it was not entered as an explicit probe.
        remember_failed_probe(obs.current_cfg, obs, true,
                              "full-failure-direct-downshift");
    }
    d.actionable = mode == GEARSHIFT_V2_ACTIVE &&
                   d.action != GEARSHIFT_ACTION_HOLD &&
                   d.action != GEARSHIFT_ACTION_ABSTAIN;
    last_v2_decision = d;
    emit_decision(obs, d);
    return d;
}

bool cl_rate_optimizer::owns_link_experiment(unsigned long long now_ms) const
{
    if (mode != GEARSHIFT_V2_ACTIVE) return false;
    if (switch_inflight) return true;
    if (!probe_active) return false;

    // Confirmed probe probation is still exclusive, but not immortal.  The
    // generic liveness sensor may report a failure only after Gearshift's OWN
    // zero-progress wall-clock budget has elapsed.  now_ms==0 keeps diagnostic
    // and synthetic callers conservative (owned until an explicit time exists).
    if (now_ms == 0 || switch_started_ms == 0 || now_ms < switch_started_ms)
        return true;
    const double budget_ms = probe_zero_progress_budget_ms > 0.0
                           ? probe_zero_progress_budget_ms
                           : policy.probe_zero_progress_ms;
    return (double)(now_ms - switch_started_ms) < std::max(1000.0, budget_ms);
}

bool cl_rate_optimizer::transition_matches(int from_cfg, int to_cfg) const
{
    return mode == GEARSHIFT_V2_ACTIVE && switch_inflight &&
           switch_from_cfg == from_cfg && switch_to_cfg == to_cfg;
}

int cl_rate_optimizer::authorize_external_transition(
        int from_cfg, int requested_to_cfg, const char* reason,
        unsigned long long now_ms, bool is_nb)
{
    if (mode != GEARSHIFT_V2_ACTIVE) return requested_to_cfg;
    if (requested_to_cfg == from_cfg) return requested_to_cfg;
    if (transition_matches(from_cfg, requested_to_cfg)) return requested_to_cfg;

    const char* why = (reason && *reason) ? reason : "unspecified";
    if (switch_inflight) {
        std::printf("[GEARSHIFT-V2-AUTHORITY] BLOCK external Axis-1 request %d->%d "
                    "reason=%s: live owner=%d->%d action=%s\n",
                    from_cfg, requested_to_cfg, why,
                    switch_from_cfg, switch_to_cfg, action_name(switch_action));
        std::fflush(stdout);
        return -1;
    }

    // A concrete failure during confirmed probe probation is useful evidence,
    // but the foreign subsystem does NOT choose the destination. Gearshift owns
    // the response and may only roll back to the probe's own fallback.
    if (probe_active) {
        if (probe_target_cfg != from_cfg || probe_fallback_cfg < 0) {
            std::printf("[GEARSHIFT-V2-AUTHORITY] BLOCK external Axis-1 request %d->%d "
                        "reason=%s: probe owner target=%d fallback=%d\n",
                        from_cfg, requested_to_cfg, why,
                        probe_target_cfg, probe_fallback_cfg);
            std::fflush(stdout);
            return -1;
        }
        const int target_cfg = probe_fallback_cfg;
        st_rate_decision d;
        d.action = GEARSHIFT_ACTION_ROLLBACK;
        d.current_cfg = from_cfg;
        d.target_cfg = target_cfg;
        d.fallback_cfg = target_cfg;
        d.actionable = true;
        d.reason = std::string("probe-failure-signal:") + why;
        last_v2_decision = d;
        notify_switch_dispatched(from_cfg, target_cfg, GEARSHIFT_ACTION_ROLLBACK,
                                 target_cfg, now_ms, is_nb);
        std::printf("[GEARSHIFT-V2-AUTHORITY] AUTH probe rollback reason=%s "
                    "foreign_hint=%d->%d owner=%d->%d\n",
                    why, from_cfg, requested_to_cfg, from_cfg, target_cfg);
        std::fflush(stdout);
        return target_cfg;
    }

    // Outside an owned experiment, legacy safety/rate code is TELEMETRY ONLY.
    // It may say "this failed"; it may not nominate CONFIG_N.  The caller must
    // feed that evidence into opt_evaluate_batch_end()/evaluate_v2, whose action
    // set and goodput model select the actual destination.
    std::printf("[GEARSHIFT-V2-AUTHORITY] BLOCK foreign Axis-1 target %d->%d "
                "reason=%s: telemetry-only; Gearshift must select destination\n",
                from_cfg, requested_to_cfg, why);
    std::fflush(stdout);
    return -1;
}

bool cl_rate_optimizer::authorize_hard_recovery(
        int current_cfg, bool at_bottom, const char* reason) const
{
    if (mode != GEARSHIFT_V2_ACTIVE) return true;
    const char* why = (reason && *reason) ? reason : "unspecified";
    if (switch_inflight || probe_active) {
        std::printf("[GEARSHIFT-V2-AUTHORITY] BLOCK hard recovery at cfg=%d reason=%s: "
                    "Gearshift experiment owns link (switch=%d probe=%d)\n",
                    current_cfg, why, switch_inflight ? 1 : 0, probe_active ? 1 : 0);
        std::fflush(stdout);
        return false;
    }
    if (!at_bottom) {
        std::printf("[GEARSHIFT-V2-AUTHORITY] BLOCK hard recovery at cfg=%d reason=%s: "
                    "lower Gearshift action remains available\n",
                    current_cfg, why);
        std::fflush(stdout);
        return false;
    }

    // Floor BREAK is an actuator, not an independent detector.  It is legal only
    // after evaluate_v2 itself has declared a hard departure with no admissible
    // lower action.  A stale/default/HOLD verdict cannot authorize RF recovery.
    const bool owner_declared_dead =
        last_v2_decision.current_cfg == current_cfg &&
        last_v2_decision.action == GEARSHIFT_ACTION_ABSTAIN &&
        last_v2_decision.reason == "no-admissible-candidate";
    if (!owner_declared_dead) {
        std::printf("[GEARSHIFT-V2-AUTHORITY] BLOCK hard recovery at floor cfg=%d "
                    "reason=%s: owner verdict action=%s verdict_reason=%s\n",
                    current_cfg, why, action_name(last_v2_decision.action),
                    last_v2_decision.reason.c_str());
        std::fflush(stdout);
        return false;
    }

    std::printf("[GEARSHIFT-V2-AUTHORITY] AUTH hard recovery at floor cfg=%d reason=%s "
                "owner_verdict=no-admissible-candidate\n",
                current_cfg, why);
    std::fflush(stdout);
    return true;
}

void cl_rate_optimizer::notify_switch_dispatched(
        int from_cfg, int to_cfg, e_gearshift_v2_action action,
        int fallback_cfg, unsigned long long now_ms, bool is_nb)
{
    if (switch_inflight) {
        std::printf("[GEARSHIFT-V2] duplicate-dispatch-rejected live_source=%d live_target=%d "
                    "live_action=%s new_source=%d new_target=%d new_action=%s\n",
                    switch_from_cfg, switch_to_cfg, action_name(switch_action),
                    from_cfg, to_cfg, action_name(action));
        std::fflush(stdout);
        return;
    }
    switch_inflight = true;
    switch_suppression_logged = false;
    switch_started_ms = now_ms;
    switch_from_cfg = from_cfg;
    switch_to_cfg = to_cfg;
    switch_action = action;
    switch_fallback_cfg = fallback_cfg;
    switch_is_nb = is_nb;
    // Do NOT clear pending application-unit evidence here.  An atomic unit can
    // legitimately span configurations after repair/probation; its final commit
    // distributes reward/cost across every contributing action.
    if (action == GEARSHIFT_ACTION_PROBE) {
        probe_active = true;
        probe_confirmed = false;
        probe_result_ready = false;
        probe_ladder_step = next_probe_ladder_step;
        probe_target_cfg = to_cfg;
        probe_fallback_cfg = fallback_cfg;
        probe_context_generation = context_generation;
        probe_baseline_generation = context_generation;
        probe_application_samples = 0;
        probe_outcome_samples = 0;
        probe_failed_outcomes = 0;
        probe_consecutive_failures = 0;
        probe_progress_events = 0;
        probe_application_bps_sum = 0.0;
        probe_application_bps_ewma = 0.0;
        probe_channel_ms = 0.0;
        probe_cycle_samples = 0;
        probe_cycle_ms_ewma = 0.0;
        probe_pending_generation_contamination = false;
        probe_geometry_trial_ms = next_probe_trial_ms;
        probe_initial_channel_ms = next_probe_max_channel_ms > 0.0
                                 ? next_probe_max_channel_ms
                                 : policy.probe_max_probation_ms;
        probe_max_channel_ms = probe_initial_channel_ms;
        probe_zero_progress_budget_ms = next_probe_zero_progress_budget_ms > 0.0
                                      ? next_probe_zero_progress_budget_ms
                                      : policy.probe_zero_progress_ms;
        next_probe_trial_ms = 0.0;
        next_probe_max_channel_ms = 0.0;
        next_probe_zero_progress_budget_ms = 0.0;
        next_probe_ladder_step = false;
        const st_online_rate_model* from = online_model_const(from_cfg, switch_is_nb);
        probe_baseline_bps = from && from->initialized
                           ? from->ewma_application_bps
                           : last_v2_decision.current_mean_bps;
        const st_online_rate_model* to = online_model_const(to_cfg, switch_is_nb);
        probe_start_application_samples = to ? to->application_samples : 0;
        probe_start_outcome_samples = to ? to->outcome_samples : 0;
        std::printf("[GEARSHIFT-V2] probe-start source=%d target=%d fallback=%d "
                    "reason=%s fallback_pred=%.1f target_pred=%.1f+/-%.1f "
                    "net=%.1f trial_ms=%.0f max_ms=%.0f zero_ms=%.0f "
                    "required_app=%d required_outcome=%d safety=%.2f ceiling_ms=%.0f "
                    "generation=%d ladder_step=%d budget_source=target-geometry\n",
                    from_cfg, to_cfg, fallback_cfg, last_v2_decision.reason.c_str(),
                    probe_baseline_bps, last_v2_decision.target_mean_bps,
                    last_v2_decision.target_sigma_bps, last_v2_decision.net_target_bps,
                    probe_geometry_trial_ms, probe_max_channel_ms,
                    probe_zero_progress_budget_ms, policy.probe_min_application_samples,
                    policy.probe_min_outcome_samples, policy.probe_budget_safety_factor,
                    policy.probe_max_probation_ms, context_generation,
                    probe_ladder_step ? 1 : 0);
        std::fflush(stdout);
    }
}

void cl_rate_optimizer::notify_switch_confirmed(unsigned long long now_ms)
{
    if (!switch_inflight) return;
    if (now_ms >= switch_started_ms) {
        const double observed = (double)(now_ms - switch_started_ms);
        if (observed > 0.0 && observed < 60000.0)
            switch_cost_ewma_ms = 0.75*switch_cost_ewma_ms + 0.25*observed;
    }
    if (switch_action == GEARSHIFT_ACTION_PROBE) {
        probe_confirmed = true;
        std::printf("[GEARSHIFT-V2] probe-confirmed source=%d target=%d "
                    "fallback=%d switch_ms=%.1f max_ms=%.0f generation=%d\n",
                    switch_from_cfg, switch_to_cfg, switch_fallback_cfg,
                    switch_cost_ewma_ms, probe_max_channel_ms, context_generation);
        std::fflush(stdout);
    }
    if (switch_action == GEARSHIFT_ACTION_ROLLBACK) clear_probe();
    cooldown_remaining = policy.cooldown_batches;
    switch_inflight = false;
    switch_suppression_logged = false;
}

void cl_rate_optimizer::notify_switch_failed()
{
    if (!switch_inflight) return;
    if (switch_action == GEARSHIFT_ACTION_PROBE) {
        // A transition-transport failure is negative information about the probe itself.
        // We do not have a fresh channel snapshot here, so use a context-free
        // block that expires only by transaction count (not by a guessed SNR).
        std::map<int, st_probe_memory>& pm = switch_is_nb ? probe_memory_nb : probe_memory_wb;
        st_probe_memory& m = pm[switch_to_cfg];
        ++m.failures;
        const int backoff = policy.probe_cooldown_batches * std::min(4, m.failures);
        m.blocked_until_tick = tick_counter + backoff;
        m.failed_snr_db = -99.9;
        m.failed_selectivity = -1.0;
        m.context_generation = context_generation;
        std::printf("[GEARSHIFT-V2] probe-memory target=%d strength=hard "
                    "reason=switch-transport-failed backoff_batches=%d failures=%d generation=%d\n",
                    switch_to_cfg, backoff, m.failures, context_generation);
        std::fflush(stdout);
        probe_cooldown_remaining = std::max(probe_cooldown_remaining, policy.cooldown_batches);
        clear_probe();
    }
    switch_inflight = false;
    switch_suppression_logged = false;
    cooldown_remaining = policy.cooldown_batches;
}

bool cl_rate_optimizer::notify_switch_confirmed_if_matches(
        int from_cfg, int to_cfg, unsigned long long now_ms)
{
    if (!switch_inflight || switch_from_cfg != from_cfg || switch_to_cfg != to_cfg) {
        if (switch_inflight) {
            std::printf("[GEARSHIFT-V2] stale-switch-confirm-ignored live=%d->%d evidence=%d->%d\n",
                        switch_from_cfg, switch_to_cfg, from_cfg, to_cfg);
            std::fflush(stdout);
        }
        return false;
    }
    notify_switch_confirmed(now_ms);
    return true;
}

bool cl_rate_optimizer::notify_switch_failed_if_matches(int from_cfg, int to_cfg)
{
    if (!switch_inflight || switch_from_cfg != from_cfg || switch_to_cfg != to_cfg) {
        if (switch_inflight) {
            std::printf("[GEARSHIFT-V2] stale-switch-failure-ignored live=%d->%d evidence=%d->%d\n",
                        switch_from_cfg, switch_to_cfg, from_cfg, to_cfg);
            std::fflush(stdout);
        }
        return false;
    }
    notify_switch_failed();
    return true;
}

bool cl_rate_optimizer::notify_external_axis1_transition(
        int from_cfg, int to_cfg, const char* reason, bool is_nb)
{
    if (mode != GEARSHIFT_V2_ACTIVE) return false;

    // The SET_CONFIG builder is the universal wire chokepoint.  A v2-owned
    // transition reaches it after notify_switch_dispatched(), so do not treat
    // that same transition as an external override.
    if (switch_inflight && switch_from_cfg == from_cfg && switch_to_cfg == to_cfg)
        return false;

    if (probe_active && probe_target_cfg >= 0) {
        std::map<int, st_probe_memory>& pm = switch_is_nb ? probe_memory_nb : probe_memory_wb;
        st_probe_memory& m = pm[probe_target_cfg];
        ++m.failures;
        const int backoff = policy.probe_cooldown_batches * std::min(4, m.failures);
        m.blocked_until_tick = tick_counter + backoff;
        m.failed_snr_db = -99.9;
        m.failed_selectivity = -1.0;
        m.context_generation = context_generation;
        std::printf("[GEARSHIFT-V2] probe-memory target=%d strength=hard "
                    "reason=%s backoff_batches=%d failures=%d generation=%d\n",
                    probe_target_cfg, reason ? reason : "external-axis1",
                    backoff, m.failures, context_generation);
        std::fflush(stdout);
        probe_cooldown_remaining = std::max(probe_cooldown_remaining, policy.cooldown_batches);
    }
    clear_probe();

    // Any external Axis-1 move means the context changed for reasons the normal
    // selector did not model. Keep historical per-config evidence, but put it in
    // the prior generation so its live weight is sharply discounted until fresh
    // observations arrive.
    switch_inflight = false;
    switch_suppression_logged = false;
    switch_from_cfg = from_cfg;
    switch_to_cfg = to_cfg;
    switch_action = GEARSHIFT_ACTION_HOLD;
    switch_fallback_cfg = -1;
    switch_is_nb = is_nb;
    cooldown_remaining = std::max(cooldown_remaining, policy.cooldown_batches);
    ++context_generation;
    context_volatility = std::max(context_volatility, 1.0);

    std::printf("[GEARSHIFT-V2] external-axis1 from=%d to=%d generation=%d reason=%s\n",
                from_cfg, to_cfg, context_generation, reason ? reason : "external");
    std::fflush(stdout);
    return true;
}

void cl_rate_optimizer::notify_cooldown_tick()
{
    if (cooldown_remaining > 0) --cooldown_remaining;
    if (probe_cooldown_remaining > 0) --probe_cooldown_remaining;
}

void cl_rate_optimizer::reset_session_state()
{
    online_wb.clear(); online_nb.clear();
    probe_memory_wb.clear(); probe_memory_nb.clear();
    tick_counter = 0;
    observation_time_ms = 0;
    context_generation = 0;
    atomic_application_transport_gain = 1.0;
    atomic_application_transport_gain_valid = false;
    atomic_application_transport_gain_samples = 0;
    last_context_snr_db = -99.9;
    last_context_selectivity = -1.0;
    context_volatility = 0.0;
    upward_probe_suppressed_until_tick = 0;
    cooldown_remaining = 0;
    probe_cooldown_remaining = 0;
    switch_inflight = false;
    switch_suppression_logged = false;
    label_streak_value.clear(); label_streak_count = 0;
    clear_probe();
    last_v2_decision = st_rate_decision();
}

void cl_rate_optimizer::set_switch_cost_ms(int ms)
{
    if (ms > 0) {
        legacy_switch_cost_ms = ms;
        switch_cost_ewma_ms = (double)ms;
    }
}

void cl_rate_optimizer::set_wire_ms_per_batch(double ms)
{
    if (ms > 0.0) { legacy_wire_ms_per_batch = ms; legacy_optclock_measured = true; }
}

void cl_rate_optimizer::force_cooldown(int batches)
{
    if (batches > cooldown_remaining) cooldown_remaining = batches;
    label_streak_value.clear(); label_streak_count = 0;
}

int cl_rate_optimizer::min_calibrated_cfg(bool is_nb) const
{
    return is_nb ? min_cfg_calibrated_nb : min_cfg_calibrated;
}

double cl_rate_optimizer::max_calibrated_sack_rate(bool is_nb) const
{
    return is_nb ? max_sack_calibrated_nb : max_sack_calibrated;
}

double cl_rate_optimizer::max_calibrated_partial_loss(bool is_nb) const
{
    return is_nb ? max_partial_loss_calibrated_nb : max_partial_loss_calibrated;
}

int cl_rate_optimizer::measured_switch_cost_ms() const
{
    return (int)(switch_cost_ewma_ms + 0.5);
}

int cl_rate_optimizer::evaluate_legacy(
        int current_cfg, double current_eff_bps, double current_sack_rate,
        int window_count, int config_ceiling, bool is_nb,
        double current_partial_loss_rate)
{
    if (!calibration_loaded || (is_nb && !nb_enabled) || cooldown_remaining > 0 || window_count < 10)
        return current_cfg;
    if (current_cfg < 0) return current_cfg;

    const std::vector<std::pair<std::string, double> >& axis =
        is_nb ? channel_axis_nb : channel_axis;
    std::string label = identify_channel_label_legacy(current_cfg, current_sack_rate,
        current_eff_bps, current_partial_loss_rate, is_nb);
    if (label.empty()) {
        double best_d = 1e300;
        for (size_t i=0; i<axis.size(); ++i) {
            const double d = std::fabs(axis[i].second-current_sack_rate);
            if (d < best_d) { best_d=d; label=axis[i].first; }
        }
    }
    if (label.empty()) return current_cfg;
    if (label == label_streak_value) {
        if (label_streak_count < LABEL_STREAK_REQUIRED) ++label_streak_count;
    } else { label_streak_value=label; label_streak_count=1; }
    if (label_streak_count < LABEL_STREAK_REQUIRED) return current_cfg;

    const st_rate_cell* stay = get_cell(current_cfg, label, is_nb);
    const double stay_score = current_eff_bps > 0.0 ? current_eff_bps
        : (stay && stay->valid ? stay->survival_adjusted_mean() : 0.0);
    const double batch_ms = legacy_optclock_measured ? legacy_wire_ms_per_batch : 1800.0;
    const double horizon = batch_ms*10.0;
    const double factor = horizon>0.0 ? std::max(0.0,1.0-(double)legacy_switch_cost_ms/horizon) : 0.0;
    int best_cfg=current_cfg; double best=stay_score;
    const std::map<int, std::map<std::string, st_rate_cell> >& active = is_nb?table_nb:table;
    for (std::map<int, std::map<std::string, st_rate_cell> >::const_iterator it=active.begin();
         it!=active.end(); ++it) {
        const int cfg=it->first;
        if (cfg==current_cfg || cfg<0 || cfg>config_ceiling) continue;
        const st_rate_cell* cell=get_cell(cfg,label,is_nb);
        if (!cell || !cell->valid) continue;
        const double score=cell->survival_adjusted_mean()*factor;
        if (score>best) { best=score; best_cfg=cfg; }
    }
    if (best_cfg!=current_cfg && (stay_score<=0.0 || best/stay_score>=legacy_hysteresis_ratio))
        return best_cfg;
    return current_cfg;
}

int cl_rate_optimizer::evaluate(int current_cfg, double current_eff_bps,
                                double current_sack_rate, int window_count,
                                int config_ceiling, bool is_nb,
                                double current_partial_loss_rate)
{
    if (mode == GEARSHIFT_V2_LEGACY)
        return evaluate_legacy(current_cfg,current_eff_bps,current_sack_rate,
                               window_count,config_ceiling,is_nb,
                               current_partial_loss_rate);
    st_rate_observation obs;
    obs.current_cfg=current_cfg;
    obs.application_bps=current_eff_bps;
    obs.transport_bps=current_eff_bps;
    obs.sack_batch_rate=current_sack_rate;
    obs.partial_frame_loss_rate=std::max(0.0,current_partial_loss_rate);
    obs.frame_success_rate=1.0-obs.partial_frame_loss_rate;
    obs.rate_samples=window_count;
    obs.outcome_samples=window_count;
    obs.application_commits=window_count;
    obs.is_nb=is_nb;
    st_rate_decision d=evaluate_v2(obs,config_ceiling);
    return d.actionable ? d.target_cfg : current_cfg;
}
