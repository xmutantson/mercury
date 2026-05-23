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


// ---------- cl_rate_optimizer ----------

cl_rate_optimizer::cl_rate_optimizer()
    : enabled(false)
    , n_configs_loaded(0)
    , n_channels_loaded(0)
    , hysteresis_ratio(1.15)
    , cooldown_max(5)
    , switch_cost_ms(1800)
    , cooldown_remaining(0)
    , eval_count(0)
    , min_cfg_calibrated(-1)
    , max_sack_calibrated(-1.0)
    , min_cfg_calibrated_nb(-1)
    , max_sack_calibrated_nb(-1.0)
    , nb_enabled(false)
    , n_configs_loaded_nb(0)
    , n_channels_loaded_nb(0)
    , label_streak_count(0)
{}

// Parse a "table" or "table_nb" section. Shared between WB and NB load
// so the per-section logic stays in one place. Returns count of valid
// cells parsed.
int cl_rate_optimizer::parse_table_section(
        const std::string& body,
        size_t section_pos,
        std::map<int, std::map<std::string, st_rate_cell> >& out_table,
        std::vector<std::pair<std::string, double> >& out_axis,
        int& out_min_cfg,
        double& out_max_sack,
        int& out_n_configs,
        int& out_n_channels)
{
    out_table.clear();
    out_axis.clear();
    out_min_cfg = -1;
    out_max_sack = -1.0;
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
            bool failed_flag = false;
            bool break_flag = false;
            int  n_runs = 0;
            for_each_object_key(body, ch_v, [&](const std::string& field, size_t field_v) {
                if (field == "eff_bps_mean") {
                    size_t end;
                    cell.eff_bps_mean = parse_number_or_bool(body, field_v, &end);
                } else if (field == "sack_rate_mean") {
                    size_t end;
                    cell.sack_rate_mean = parse_number_or_bool(body, field_v, &end);
                } else if (field == "n_runs") {
                    size_t end;
                    n_runs = (int)parse_number_or_bool(body, field_v, &end);
                } else if (field == "failed") {
                    size_t end;
                    failed_flag = (parse_number_or_bool(body, field_v, &end) > 0.5);
                } else if (field == "break_fired") {
                    size_t end;
                    break_flag = (parse_number_or_bool(body, field_v, &end) > 0.5);
                }
            });

            cell.valid = (cell.eff_bps_mean > 0.0)
                        && !failed_flag && !break_flag && (n_runs >= 1);
            channels[ch_key] = cell;
            if (cell.valid) {
                ++valid_cells;
                std::pair<double, int>& acc = channel_sack_accum[ch_key];
                acc.first  += cell.sack_rate_mean;
                acc.second += 1;
            }
        });
    });

    out_n_configs = (int)out_table.size();
    for (std::map<std::string, std::pair<double, int> >::const_iterator
            it = channel_sack_accum.begin(); it != channel_sack_accum.end(); ++it) {
        if (it->second.second > 0) {
            double mean_sack = it->second.first / (double)it->second.second;
            out_axis.push_back(std::make_pair(it->first, mean_sack));
        }
    }
    std::sort(out_axis.begin(), out_axis.end(),
              [](const std::pair<std::string,double>& a,
                 const std::pair<std::string,double>& b){
                  return a.second < b.second;
              });
    out_n_channels = (int)out_axis.size();

    // Min cfg + max sack across valid cells.
    out_min_cfg = -1;
    out_max_sack = 0.0;
    for (std::map<int, std::map<std::string, st_rate_cell> >::const_iterator
            cit = out_table.begin(); cit != out_table.end(); ++cit) {
        bool any_valid = false;
        for (std::map<std::string, st_rate_cell>::const_iterator
                jt = cit->second.begin(); jt != cit->second.end(); ++jt) {
            if (!jt->second.valid) continue;
            any_valid = true;
            if (jt->second.sack_rate_mean > out_max_sack)
                out_max_sack = jt->second.sack_rate_mean;
        }
        if (any_valid && (out_min_cfg < 0 || cit->first < out_min_cfg))
            out_min_cfg = cit->first;
    }
    return valid_cells;
}

bool cl_rate_optimizer::load(const char* path) {
    enabled = false;
    nb_enabled = false;
    table.clear();
    table_nb.clear();
    channel_axis.clear();
    channel_axis_nb.clear();
    n_configs_loaded = 0;
    n_channels_loaded = 0;
    n_configs_loaded_nb = 0;
    n_channels_loaded_nb = 0;
    min_cfg_calibrated = -1;
    max_sack_calibrated = -1.0;
    min_cfg_calibrated_nb = -1;
    max_sack_calibrated_nb = -1.0;

    if (path == NULL || path[0] == '\0') {
        printf("[OPT] table not found, optimizer disabled (null path)\n");
        fflush(stdout);
        return false;
    }
    std::ifstream f(path);
    if (!f.is_open()) {
        printf("[OPT] table not found, optimizer disabled (path=%s)\n", path);
        fflush(stdout);
        return false;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    std::string raw = ss.str();
    if (raw.empty()) {
        printf("[OPT] table empty, optimizer disabled (path=%s)\n", path);
        fflush(stdout);
        return false;
    }
    std::string body = strip_comments(raw);

    // Parse WB section ("table").
    size_t table_v = find_key_value_pos(body, 0, "table");
    if (table_v == std::string::npos || table_v >= body.size() || body[table_v] != '{') {
        printf("[OPT] table parse failed (no `table` object), optimizer disabled\n");
        fflush(stdout);
        return false;
    }
    int valid_wb = parse_table_section(body, table_v,
                                       table, channel_axis,
                                       min_cfg_calibrated, max_sack_calibrated,
                                       n_configs_loaded, n_channels_loaded);

    if (valid_wb == 0 || n_configs_loaded == 0 || n_channels_loaded == 0) {
        printf("[OPT] table parsed but no valid cells, optimizer disabled "
               "(WB valid=%d configs=%d channels=%d)\n",
               valid_wb, n_configs_loaded, n_channels_loaded);
        fflush(stdout);
        return false;
    }

    // Parse optional NB section ("table_nb"). Absent is fine — NB sessions
    // get silent-no-op optimizer behavior (correct fallback to gearshift).
    int valid_nb = 0;
    size_t table_nb_v = find_key_value_pos(body, 0, "table_nb");
    if (table_nb_v != std::string::npos && table_nb_v < body.size()
        && body[table_nb_v] == '{')
    {
        valid_nb = parse_table_section(body, table_nb_v,
                                       table_nb, channel_axis_nb,
                                       min_cfg_calibrated_nb, max_sack_calibrated_nb,
                                       n_configs_loaded_nb, n_channels_loaded_nb);
        nb_enabled = (valid_nb > 0 && n_configs_loaded_nb > 0
                      && n_channels_loaded_nb > 0);
    }

    enabled = true;
    printf("[OPT] WB table loaded: %d configs x %d channels (valid_cells=%d, "
           "min_cfg=%d, max_sack=%.3f, path=%s)\n",
           n_configs_loaded, n_channels_loaded, valid_wb,
           min_cfg_calibrated, max_sack_calibrated, path);
    if (nb_enabled) {
        printf("[OPT] NB table loaded: %d configs x %d channels (valid_cells=%d, "
               "min_cfg=%d, max_sack=%.3f)\n",
               n_configs_loaded_nb, n_channels_loaded_nb, valid_nb,
               min_cfg_calibrated_nb, max_sack_calibrated_nb);
    } else {
        printf("[OPT] NB table absent — optimizer will be silent on NB sessions\n");
    }
    fflush(stdout);
    return true;
}

void cl_rate_optimizer::notify_cooldown_tick() {
    if (cooldown_remaining > 0) --cooldown_remaining;
}

void cl_rate_optimizer::reset_session_state() {
    cooldown_remaining = 0;
    eval_count = 0;
    label_streak_value.clear();
    label_streak_count = 0;
}

const st_rate_cell* cl_rate_optimizer::get_cell(int cfg, const std::string& bucket,
                                                 bool is_nb) const {
    const std::map<int, std::map<std::string, st_rate_cell> >& active_table =
        is_nb ? table_nb : table;
    std::map<int, std::map<std::string, st_rate_cell> >::const_iterator
        it = active_table.find(cfg);
    if (it == active_table.end()) return NULL;
    std::map<std::string, st_rate_cell>::const_iterator
        jt = it->second.find(bucket);
    if (jt == it->second.end()) return NULL;
    return &(jt->second);
}

// Identify the channel label that best matches current observations AT THE
// CURRENT CONFIG. Returns "" if table[current_cfg] is empty.
//
// Rationale: the table is a 2D measured surface — for each (config, label)
// we know what (eff_bps, sack_rate) the link produces. To predict what
// other configs would deliver on the SAME physical channel, we first
// identify which calibrated channel-label our current observations match
// AT OUR CURRENT CONFIG. Then evaluate every other config at the same
// label (the table tells us directly — no ladder-distance scaling).
//
// Distance metric: sack_rate is the primary signal (well-defined for the
// channel and the most direct measure of "how rough is this channel").
// eff_bps_mean is a tiebreaker — sack_rate alone can be 0.0 for several
// adjacent buckets (e.g. clean / wgn30 / wgn28 may all measure 0% loss
// on a robust config), so eff_bps disambiguates which we're actually in.
std::string cl_rate_optimizer::identify_channel_label(int current_cfg,
                                                      double current_sack_rate,
                                                      double current_eff_bps,
                                                      bool is_nb) const {
    // H1: defensive — non-finite observations come from upstream bugs.
    // Returning "" forces the channel_axis fallback in evaluate(), which is
    // less precise but safe.
    if (!std::isfinite(current_sack_rate) || !std::isfinite(current_eff_bps))
        return "";

    const std::map<int, std::map<std::string, st_rate_cell> >& active_table =
        is_nb ? table_nb : table;
    std::map<int, std::map<std::string, st_rate_cell> >::const_iterator
        cit = active_table.find(current_cfg);
    if (cit == active_table.end()) return "";
    const std::map<std::string, st_rate_cell>& row = cit->second;
    if (row.empty()) return "";

    // Normalize the two axes so they're comparable. SACK rate is in [0,1].
    // eff_bps spans 0..several thousand. Use a soft normalization: divide
    // eff_bps difference by current_eff_bps (or 1000 if current is 0).
    double eff_norm = (current_eff_bps > 100.0) ? current_eff_bps : 1000.0;

    double best_d = 1e9;
    std::string best;
    for (std::map<std::string, st_rate_cell>::const_iterator
            jt = row.begin(); jt != row.end(); ++jt) {
        if (!jt->second.valid) continue;
        double ds = jt->second.sack_rate_mean - current_sack_rate;
        double de = (jt->second.eff_bps_mean - current_eff_bps) / eff_norm;
        // Weight sack_rate heavily (4x) — more direct channel signal.
        // eff_bps acts as a tiebreaker / sanity check.
        //
        // M1 (known limitation): at saturated configs (e.g. CFG6 where many
        // labels collapse to sack=0/bps=ceiling), several labels produce
        // identical primary distance AND identical sack_rate_mean, so a
        // simple bias term can't break the tie. Iteration here uses
        // alphabetical std::map order, so "clean" wins — the most
        // optimistic label. The projection to higher configs may therefore
        // recommend a config that's actually at its cliff, triggering BREAK
        // which downshifts and rearms the optimizer cooldown — the system
        // self-corrects but burns a BREAK cycle.
        // Belt-and-suspenders mitigations active: max_calibrated_sack_rate
        // gate + force_cooldown(8) on BREAK keep the failure mode bounded.
        // If smoke testing surfaces real thrashing, swap this for a true
        // severity-rank tiebreaker (parse label name → "wgn16 > wgn30 >
        // clean" ordering + 1e-6 * rank term).
        double d = 4.0 * ds * ds + de * de;
        if (d < best_d) { best_d = d; best = jt->first; }
    }
    return best;
}

int cl_rate_optimizer::evaluate(int current_cfg,
                                double current_eff_bps,
                                double current_sack_rate,
                                int window_count,
                                int config_ceiling,
                                bool is_nb) {
    ++eval_count;

    // Hard kill: table missing → caller stays put.
    if (!enabled) return current_cfg;
    // NB session but no NB table → caller stays put (silent fallback to
    // gearshift). WB session always has a valid table since enabled=true.
    if (is_nb && !nb_enabled) return current_cfg;
    // Cooldown active → caller stays put. Don't even log to keep the
    // diagnostic stream sparse.
    if (cooldown_remaining > 0) return current_cfg;

    // §4.3 statistical-confidence floor.
    if (window_count < 10) return current_cfg;

    // Optimizer only operates on OFDM ladder. ROBUST_X (>=100) and any
    // out-of-range value short-circuits — gearshift owns those transitions.
    // (Caller also enforces this gate; double-check for safety.)
    if (current_cfg < 0 || current_cfg > config_ceiling) return current_cfg;

    // Pick the active channel_axis (used for fallback when current_cfg has
    // no table row).
    const std::vector<std::pair<std::string, double> >& active_axis =
        is_nb ? channel_axis_nb : channel_axis;

    // Identify the channel label that matches our current observations at
    // the current config. The table is a 2D surface (config × label);
    // identifying the label collapses it to a 1D scan over candidate configs.
    // If we have no calibration row at current_cfg (e.g. CFG7 in the gap),
    // fall back to nearest channel bucket using config-agnostic axis.
    std::string current_label = identify_channel_label(current_cfg,
                                                       current_sack_rate,
                                                       current_eff_bps,
                                                       is_nb);
    if (current_label.empty()) {
        // Fallback: nearest bucket on the cross-config mean axis. Less
        // accurate but better than refusing to act.
        if (active_axis.empty()) return current_cfg;
        double best_d = 1e9;
        for (size_t i = 0; i < active_axis.size(); ++i) {
            double d = std::fabs(active_axis[i].second - current_sack_rate);
            if (d < best_d) { best_d = d; current_label = active_axis[i].first; }
        }
        if (current_label.empty()) return current_cfg;
    }

    // Label hysteresis: identify_channel_label() runs every batch and uses
    // a single sack_rate sample weighted 4× over eff_bps. Normal random
    // partial-batch clustering on a clean channel can briefly hit
    // sack_rate≈0.25-0.30, which is enough to flip "clean" → "wgn30" and
    // recommend an unwarranted CFG upshift. Require LABEL_STREAK_REQUIRED
    // consecutive evals at the same label before promoting it to the
    // switch-decision logic below.
    //
    // Reproducer that motivated this guard: post-§6g clean-channel r2 of
    // 2026-05-22 phase4 benchmark — sack_rate spiked 0.25-0.29 for one
    // eval, flipped label to wgn30, fired SET_CONFIG 15→16, forced a
    // streaming PPMd+zstd reset, user_bps fell from ~6000 to 2304.
    if (current_label == label_streak_value) {
        if (label_streak_count < LABEL_STREAK_REQUIRED) ++label_streak_count;
    } else {
        label_streak_value = current_label;
        label_streak_count = 1;
    }
    if (label_streak_count < LABEL_STREAK_REQUIRED) {
        const char* bw_tag_early = is_nb ? " nb=1" : "";
        printf("[OPT-EVAL] eff=%.0f sack=%.2f curr_cfg=%d -> stay "
               "(skip=label-unstable streak=%d/%d) label=%s%s\n",
               current_eff_bps, current_sack_rate, current_cfg,
               label_streak_count, (int)LABEL_STREAK_REQUIRED,
               current_label.c_str(), bw_tag_early);
        fflush(stdout);
        return current_cfg;
    }

    // Score "stay". Use the table's measured value at (current_cfg, label)
    // if available; otherwise use the live measurement.
    const st_rate_cell* stay_cell = get_cell(current_cfg, current_label, is_nb);
    double stay_score = (stay_cell && stay_cell->valid)
                       ? stay_cell->eff_bps_mean
                       : current_eff_bps;

    // Switch cost amortized across the expected post-switch holding period.
    // switch_cost_ms is paid ONCE per switch (one SET_CONFIG round trip).
    // If we expect to hold the new config for AMORTIZE_BATCHES batches,
    // the per-batch penalty is (switch_cost_ms / AMORTIZE_BATCHES) of one
    // batch's worth of wire time.
    //
    // Original code amortized across one batch — i.e. assumed every batch
    // would re-pay the cost — which made the penalty equal to stay_score
    // and blocked nearly every upshift. With AMORTIZE_BATCHES=10 the
    // penalty is ~10% of stay_score, and combined with the 15% hysteresis
    // requires the candidate to beat stay by ~25% to trigger. That's a
    // sane bar for "is the switch worth the SET_CONFIG round trip given
    // we'll hold for at least ~10 batches."
    const double WIRE_MS_PER_BATCH = 1800.0;
    const double AMORTIZE_BATCHES  = 10.0;
    double cost_penalty_bps = stay_score *
                              ((double)switch_cost_ms /
                               (WIRE_MS_PER_BATCH * AMORTIZE_BATCHES));

    // SEARCH THE WHOLE TABLE — no ±2 limit. With per-config channel
    // identification we look up tbl[cand][current_label] directly for every
    // candidate; the table tells us what each config does on THIS channel
    // (the one we just identified from our own measurements). No
    // ladder-distance scaling, no half/double heuristic. If the table says
    // a single jump CFG7→CFG16 is worth it, we make that jump.
    int    best_cfg               = current_cfg;
    double best_score             = stay_score;
    int    best_target_seen       = current_cfg;
    double best_target_score_seen = stay_score;

    const std::map<int, std::map<std::string, st_rate_cell> >& active_table =
        is_nb ? table_nb : table;
    for (std::map<int, std::map<std::string, st_rate_cell> >::const_iterator
            cit = active_table.begin(); cit != active_table.end(); ++cit) {
        int cand = cit->first;
        if (cand == current_cfg) continue;
        if (cand < 0 || cand > config_ceiling) continue;

        const st_rate_cell* cand_cell = get_cell(cand, current_label, is_nb);
        if (!cand_cell || !cand_cell->valid) continue;

        double cand_score = cand_cell->eff_bps_mean - cost_penalty_bps;
        if (cand_score > best_target_score_seen) {
            best_target_score_seen = cand_score;
            best_target_seen = cand;
        }
        if (cand_score > best_score) {
            best_score = cand_score;
            best_cfg = cand;
        }
    }

    // Hysteresis: only fire if the best candidate beats stay by 15%.
    int chosen_cfg = current_cfg;
    const char* skip_reason = NULL;
    double gain_pct = 0.0;
    if (best_cfg != current_cfg) {
        if (stay_score > 0.0) {
            double ratio = best_score / stay_score;
            gain_pct = (ratio - 1.0) * 100.0;
            if (ratio >= hysteresis_ratio) {
                chosen_cfg = best_cfg;
                cooldown_remaining = cooldown_max;
                label_streak_value.clear();
                label_streak_count = 0;
            } else {
                skip_reason = "below-hysteresis";
            }
        } else {
            // stay_score is 0 — channel is dead at current_cfg per the
            // table. Switch IS warranted; failure-recovery / BREAK paths
            // run in parallel.
            chosen_cfg = best_cfg;
            cooldown_remaining = cooldown_max;
            label_streak_value.clear();
            label_streak_count = 0;
            gain_pct = 100.0;
        }
    }

    // Diag — emit one line per fire. Include the best-candidate-seen even
    // if we decided to stay, for offline trace analysis. nb flag included
    // so post-hoc log analysis can distinguish WB vs NB decisions.
    const char* bw_tag = is_nb ? " nb=1" : "";
    if (best_target_seen != current_cfg) {
        if (chosen_cfg != current_cfg) {
            printf("[OPT-EVAL] eff=%.0f sack=%.2f curr_cfg=%d -> recommend=%d "
                   "(gain=%.1f%%) label=%s%s\n",
                   current_eff_bps, current_sack_rate, current_cfg, chosen_cfg,
                   gain_pct, current_label.c_str(), bw_tag);
        } else {
            printf("[OPT-EVAL] eff=%.0f sack=%.2f curr_cfg=%d -> stay (best_cand=%d "
                   "gain=%.1f%% skip=%s) label=%s%s\n",
                   current_eff_bps, current_sack_rate, current_cfg, best_target_seen,
                   gain_pct, skip_reason ? skip_reason : "n/a",
                   current_label.c_str(), bw_tag);
        }
        fflush(stdout);
    }

    return chosen_cfg;
}
