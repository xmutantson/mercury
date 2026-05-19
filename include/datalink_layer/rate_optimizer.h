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
 * Effective-Rate Optimizer — Phase 3c (decision logic).
 *
 * See: mercury/fact-documents/EFFECTIVE_RATE_OPTIMIZER_DESIGN.md
 *
 * Self-contained — no dependency on cl_arq_controller. The controller calls
 * `evaluate()` once per batch-end and acts on the returned target config.
 * Optimizer is inert if `effective_rate_table.json` is missing or unparseable.
 */

#ifndef RATE_OPTIMIZER_H_
#define RATE_OPTIMIZER_H_

#include <map>
#include <string>
#include <vector>

// One calibrated cell: per-(config, channel) measured effective bps.
// Only the fields the optimizer reads are stored; other JSON keys are
// silently ignored by the parser.
struct st_rate_cell {
    double eff_bps_mean;     // Mean effective bps over n_runs
    double sack_rate_mean;   // Mean SACK rate observed at this cell
    bool   valid;            // true iff this cell was successfully parsed
                             // AND not marked failed/break_fired
    st_rate_cell()
        : eff_bps_mean(0.0)
        , sack_rate_mean(0.0)
        , valid(false)
    {}
};

class cl_rate_optimizer {
public:
    cl_rate_optimizer();

    // Load the calibration table. Returns true on success. Prints either
    // `[OPT] table loaded: N configs x M channels` (success) or
    // `[OPT] table not found, optimizer disabled` (any failure). Never
    // throws; never aborts mercury startup.
    bool load(const char* path);

    // True iff load() succeeded AND parsed at least one valid cell.
    // evaluate() always returns "stay at current_cfg" when this is false.
    bool is_enabled() const { return enabled; }

    // Decision tick — called once per batch-end on CMD when sack_v2_enabled.
    //
    // Inputs:
    //   current_cfg       : the cfg the link is using right now
    //   current_eff_bps   : `get_current_effective_rate_bps()` from arq.h
    //   current_sack_rate : `get_current_sack_rate()` from arq.h
    //   window_count      : `get_current_window_count()` (min 10 to fire)
    //   wb_config_max     : ceiling for WB candidates (WB_CONFIG_MAX)
    //
    // Returns:
    //   target_cfg out-param. Caller checks `target_cfg != current_cfg` to
    //   decide whether to queue a SET_CONFIG. Returned value is guaranteed
    //   in [CONFIG_0, wb_config_max] (defensive — we never recommend
    //   ROBUST_X or beyond-ceiling configs).
    //
    // Cooldown:
    //   On a recommended switch the optimizer arms a 5-batch cooldown;
    //   subsequent calls return current_cfg until the cooldown drains.
    //   Caller invokes `notify_cooldown_tick()` once per batch regardless
    //   of switch outcome to drive the counter.
    //
    // Diag: emits one `[OPT-EVAL]` line per fire (including no-op
    // evaluations after the gate passes) for offline trace analysis.
    int evaluate(int current_cfg,
                 double current_eff_bps,
                 double current_sack_rate,
                 int window_count,
                 int wb_config_max);

    // Drain the cooldown counter once per batch-end (regardless of whether
    // evaluate() ran or short-circuited). Cheap; no-op when counter is 0.
    void notify_cooldown_tick();

    // Reset cooldown + last-switch state. Called from cl_arq_controller's
    // reset_session_state() — prior-session state describes a different
    // channel and would skew the first few evaluations.
    void reset_session_state();

    // Direct knobs for tests / future tuning. Defaults from §4.3 + §5.3.
    void set_hysteresis_ratio(double r) { hysteresis_ratio = r; }
    void set_cooldown_batches(int n)    { cooldown_max = n; }
    void set_switch_cost_ms(int ms)     { switch_cost_ms = ms; }

private:
    bool enabled;
    int  n_configs_loaded;
    int  n_channels_loaded;

    // table[cfg][channel_bucket] -> cell.
    // cfg is the integer index (e.g. 15 for CONFIG_15). channel_bucket is
    // the JSON key string ("clean", "wgn30", "wgn28", ...).
    std::map<int, std::map<std::string, st_rate_cell> > table;

    // Ordered list of WGN channel buckets sorted by descending mean SACK
    // rate (i.e. wgn16 first, wgn30 last). Built once at load time so
    // channel_bucket_from_sack_rate() can binary-pick the nearest.
    // Each entry is (bucket_name, representative_sack_rate). Representative
    // SACK rate is the mean across configs at that bucket (with valid > 0
    // cells only).
    std::vector<std::pair<std::string, double> > channel_axis;

    // Decision constants — overridable for tests via set_*().
    double hysteresis_ratio;  // 1.15  (15% gain required to switch)
    int    cooldown_max;      // 5     (batches between optimizer switches)
    int    switch_cost_ms;    // 1800  (PHY-switch round-trip per §3.3)

    // Cooldown remaining (in batches). 0 = optimizer free to switch.
    int    cooldown_remaining;

    // Diag throttle — we emit [OPT-EVAL] at most once per call but tag
    // batches the optimizer was asked about even if no switch was made.
    int    eval_count;

    // Map current SACK rate to the nearest calibrated channel bucket.
    // Returns "" if channel_axis is empty (i.e. table never loaded).
    std::string channel_bucket_from_sack_rate(double sack_rate) const;

    // Look up cell. Returns NULL if missing. Caller checks valid flag.
    const st_rate_cell* get_cell(int cfg, const std::string& bucket) const;

    // Predict the loss-at-target heuristic per §5.1. Naive linear: a
    // 1-step downshift gets ~half the loss; a 2-step downshift gets ~zero.
    // (Conservative — only used to pick the channel bucket for the
    // predicted_rate lookup at the target config.) Returns the SACK rate
    // we'd expect at target_cfg given current channel conditions.
    double predicted_sack_at_target(int current_cfg,
                                    int target_cfg,
                                    double current_sack_rate) const;
};

#endif // RATE_OPTIMIZER_H_
