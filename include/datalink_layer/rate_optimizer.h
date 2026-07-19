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
    //   config_ceiling    : highest cfg the candidate loop may recommend
    //                       (WB_CONFIG_MAX in WB sessions, NB_CONFIG_MAX in NB)
    //   is_nb             : true when the current session is narrowband.
    //                       Routes lookups to the NB calibration table; if
    //                       no NB table was loaded, evaluate returns
    //                       current_cfg (optimizer silent on NB).
    //
    // Returns:
    //   target_cfg. Caller checks `target_cfg != current_cfg` to decide
    //   whether to queue a SET_CONFIG. Returned value is guaranteed in
    //   [CONFIG_0, config_ceiling] (defensive — we never recommend
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
                 int config_ceiling,
                 bool is_nb = false);

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
    // MC-7: supplying live batch airtime also selects the measured-clock
    // penalty path. The controller calls this only under
    // MERCURY_LINKPHASE_OPTCLOCK, leaving the frozen legacy path untouched
    // when the flag is OFF. Ignore non-positive samples rather than allowing
    // a bad geometry observation to create a divide-by-zero decision.
    void set_wire_ms_per_batch(double ms)
    {
        if (ms > 0.0) {
            wire_ms_per_batch = ms;
            optclock_measured = true;
        }
    }

    // BREAK-driven cooldown — called from the BREAK handler in arq_common.cc
    // when BREAK fires. Pins cooldown_remaining to N batches so the optimizer
    // stays silent while gearshift/SUPERSHIFT/BREAK run the descent (and any
    // subsequent reverse-probe up-climb). Without this the optimizer would
    // immediately recommend going back to the config BREAK just bounced us
    // out of, causing thrashing. Idempotent — only raises, never lowers.
    void force_cooldown(int batches)
    {
        if (batches > cooldown_remaining) cooldown_remaining = batches;
        label_streak_value.clear();
        label_streak_count = 0;
    }

    // Below-table-range gate inputs. Populated at load(); used by
    // cl_arq_controller::opt_evaluate_batch_end() to decide whether the
    // optimizer is in its calibrated operating region.
    //   - min_calibrated_cfg: lowest cfg id with a valid cell in the table.
    //     Below this, gearshift/SUPERSHIFT/BREAK own the link entirely.
    //   - max_calibrated_sack_rate: largest sack_rate_mean across all valid
    //     cells. Above this we're observing a channel WORSE than anything
    //     we calibrated for — no business recommending a switch.
    //   - is_nb: select NB calibration data instead of WB.
    // Both return -1 / -1.0 when the table never loaded (signals "ignore").
    int    min_calibrated_cfg(bool is_nb = false) const {
        return is_nb ? min_cfg_calibrated_nb : min_cfg_calibrated;
    }
    double max_calibrated_sack_rate(bool is_nb = false) const {
        return is_nb ? max_sack_calibrated_nb : max_sack_calibrated;
    }

private:
    bool enabled;
    int  n_configs_loaded;
    int  n_channels_loaded;

    // table[cfg][channel_bucket] -> cell.
    // cfg is the integer index (e.g. 15 for CONFIG_15). channel_bucket is
    // the JSON key string ("clean", "wgn30", "wgn28", ...).
    // `table` holds WB cells; `table_nb` holds NB cells. They populate
    // from the JSON keys "table" and "table_nb" respectively. The NB table
    // is optional — when absent, NB sessions get silent-no-op optimizer
    // behavior (correct fallback to gearshift/BREAK).
    std::map<int, std::map<std::string, st_rate_cell> > table;
    std::map<int, std::map<std::string, st_rate_cell> > table_nb;

    // Ordered list of WGN channel buckets sorted by descending mean SACK
    // rate (i.e. wgn16 first, wgn30 last). Built once at load time so
    // channel_bucket_from_sack_rate() can binary-pick the nearest.
    // Each entry is (bucket_name, representative_sack_rate). Representative
    // SACK rate is the mean across configs at that bucket (with valid > 0
    // cells only).
    std::vector<std::pair<std::string, double> > channel_axis;
    std::vector<std::pair<std::string, double> > channel_axis_nb;

    // Decision constants — overridable for tests via set_*().
    double hysteresis_ratio;  // 1.15  (15% gain required to switch)
    int    cooldown_max;      // 5     (batches between optimizer switches)
    int    switch_cost_ms;    // 1800  (PHY-switch round-trip per §3.3)
    double wire_ms_per_batch; // live emitted DATA-keydown airtime (MC-7)
    bool   optclock_measured; // false => exact frozen 1800 ms / 10-batch path

    // Cooldown remaining (in batches). 0 = optimizer free to switch.
    int    cooldown_remaining;

    // Diag throttle — we emit [OPT-EVAL] at most once per call but tag
    // batches the optimizer was asked about even if no switch was made.
    int    eval_count;

    // Label hysteresis. identify_channel_label() runs per batch and uses a
    // single sack_rate sample as primary input (weighted 4× over eff_bps).
    // Normal random partial-batch clustering on a clean channel can produce
    // 1-2 evals at sack_rate≈0.25-0.30, which is enough to flip the label
    // from "clean" to "wgn30" and recommend an unwarranted CFG upshift that
    // destroys the streaming compression context. Require K consecutive
    // evals at the same label before acting on it. ~K batches × 1.8s/batch
    // ≈ K × 1.8s of label stability needed (K=4 → ~7 s).
    std::string label_streak_value;
    int         label_streak_count;
    static const int LABEL_STREAK_REQUIRED = 4;

    // Below-table-range gate inputs — populated by load(). -1 / -1.0 when
    // no table is loaded (signals "no gate active"). Parallel WB / NB.
    int    min_cfg_calibrated;
    double max_sack_calibrated;
    int    min_cfg_calibrated_nb;
    double max_sack_calibrated_nb;

    // True iff load() found a "table_nb" section with at least one valid
    // cell. When false, evaluate() short-circuits to "stay" on NB sessions.
    bool   nb_enabled;

    // Counters per bandwidth mode (purely for [OPT] startup log clarity).
    int    n_configs_loaded_nb;
    int    n_channels_loaded_nb;

    // Identify the channel label that best matches our current observations
    // AT THE CURRENT CONFIG. Returns the channel-label key (e.g. "wgn22")
    // for the cell in the table whose sack_rate_mean is closest to
    // current_sack_rate, with eff_bps_mean as a tiebreaker. Returns "" if
    // the table[current_cfg] row is empty (caller falls back to channel_axis).
    // is_nb selects WB (table) vs NB (table_nb).
    std::string identify_channel_label(int current_cfg,
                                       double current_sack_rate,
                                       double current_eff_bps,
                                       bool is_nb) const;

    // Look up cell. Returns NULL if missing. Caller checks valid flag.
    const st_rate_cell* get_cell(int cfg, const std::string& bucket,
                                 bool is_nb) const;

    // Parse one table section ("table" or "table_nb") into the given target
    // maps. Returns count of valid cells parsed. Shared between WB+NB load.
    int parse_table_section(const std::string& body,
                            size_t section_pos,
                            std::map<int, std::map<std::string, st_rate_cell> >& out_table,
                            std::vector<std::pair<std::string, double> >& out_axis,
                            int& out_min_cfg,
                            double& out_max_sack,
                            int& out_n_configs,
                            int& out_n_channels);
};

#endif // RATE_OPTIMIZER_H_
