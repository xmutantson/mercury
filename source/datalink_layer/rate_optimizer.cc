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
{}

bool cl_rate_optimizer::load(const char* path) {
    enabled = false;
    table.clear();
    channel_axis.clear();
    n_configs_loaded = 0;
    n_channels_loaded = 0;

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

    // Locate `"table"`.
    size_t table_v = find_key_value_pos(body, 0, "table");
    if (table_v == std::string::npos || table_v >= body.size() || body[table_v] != '{') {
        printf("[OPT] table parse failed (no `table` object), optimizer disabled\n");
        fflush(stdout);
        return false;
    }

    // Track per-channel cumulative sack-rate sum + count for channel_axis.
    std::map<std::string, std::pair<double, int> > channel_sack_accum;

    int parsed_cells = 0;
    int valid_cells = 0;

    for_each_object_key(body, table_v, [&](const std::string& cfg_key, size_t cfg_v) {
        if (cfg_v >= body.size() || body[cfg_v] != '{') return;
        int cfg_id = std::atoi(cfg_key.c_str());
        if (cfg_id < 0) return;

        std::map<std::string, st_rate_cell>& channels = table[cfg_id];

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
                // Ignore everything else (eff_bps_min/max/sigma/n_failed_runs/
                // runs[], etc.) — forward-compatible.
            });

            ++parsed_cells;

            // A cell is valid iff:
            //  - eff_bps_mean > 0
            //  - failed != true
            //  - break_fired != true
            // n_runs == 0 is also disqualifying because eff_bps_mean is then
            // necessarily 0; but we don't require n_runs explicitly.
            cell.valid = (cell.eff_bps_mean > 0.0)
                        && !failed_flag && !break_flag && (n_runs >= 0);
            channels[ch_key] = cell;
            if (cell.valid) {
                ++valid_cells;
                std::pair<double, int>& acc = channel_sack_accum[ch_key];
                acc.first  += cell.sack_rate_mean;
                acc.second += 1;
            }
        });
    });

    n_configs_loaded = (int)table.size();
    // Build channel_axis. Each entry = mean of sack_rate_mean across valid
    // cells in that channel bucket.
    for (std::map<std::string, std::pair<double, int> >::const_iterator
            it = channel_sack_accum.begin(); it != channel_sack_accum.end(); ++it) {
        if (it->second.second > 0) {
            double mean_sack = it->second.first / (double)it->second.second;
            channel_axis.push_back(std::make_pair(it->first, mean_sack));
        }
    }
    // Sort by ascending sack_rate so we can pick the nearest bucket easily.
    std::sort(channel_axis.begin(), channel_axis.end(),
              [](const std::pair<std::string,double>& a,
                 const std::pair<std::string,double>& b){
                  return a.second < b.second;
              });
    n_channels_loaded = (int)channel_axis.size();

    if (valid_cells == 0 || n_configs_loaded == 0 || n_channels_loaded == 0) {
        printf("[OPT] table parsed but no valid cells, optimizer disabled "
               "(parsed=%d valid=%d configs=%d channels=%d)\n",
               parsed_cells, valid_cells, n_configs_loaded, n_channels_loaded);
        fflush(stdout);
        return false;
    }

    enabled = true;
    printf("[OPT] table loaded: %d configs x %d channels (valid_cells=%d, path=%s)\n",
           n_configs_loaded, n_channels_loaded, valid_cells, path);
    fflush(stdout);
    return true;
}

void cl_rate_optimizer::notify_cooldown_tick() {
    if (cooldown_remaining > 0) --cooldown_remaining;
}

void cl_rate_optimizer::reset_session_state() {
    cooldown_remaining = 0;
    eval_count = 0;
}

std::string cl_rate_optimizer::channel_bucket_from_sack_rate(double sack_rate) const {
    if (channel_axis.empty()) return "";
    // Pick the calibrated bucket whose representative sack rate is closest
    // to current. channel_axis is sorted by ascending sack_rate (clean
    // smallest, wgn16 largest).
    double best_d = 1e9;
    std::string best;
    for (size_t i = 0; i < channel_axis.size(); ++i) {
        double d = std::fabs(channel_axis[i].second - sack_rate);
        if (d < best_d) { best_d = d; best = channel_axis[i].first; }
    }
    return best;
}

const st_rate_cell* cl_rate_optimizer::get_cell(int cfg, const std::string& bucket) const {
    std::map<int, std::map<std::string, st_rate_cell> >::const_iterator
        it = table.find(cfg);
    if (it == table.end()) return NULL;
    std::map<std::string, st_rate_cell>::const_iterator
        jt = it->second.find(bucket);
    if (jt == it->second.end()) return NULL;
    return &(jt->second);
}

double cl_rate_optimizer::predicted_sack_at_target(int current_cfg,
                                                   int target_cfg,
                                                   double current_sack_rate) const {
    // §5.1 naive linear interpolation: each ladder step down halves the
    // loss; each step up doubles it. Clamp to [0, 1].
    int delta = target_cfg - current_cfg;
    double scale;
    if (delta == 0)       scale = 1.0;
    else if (delta < 0)   scale = std::pow(0.5, (double)(-delta));   // downshift cuts loss
    else                  scale = std::pow(2.0, (double)delta);      // upshift inflates loss
    double s = current_sack_rate * scale;
    if (s < 0.0) s = 0.0;
    if (s > 1.0) s = 1.0;
    return s;
}

int cl_rate_optimizer::evaluate(int current_cfg,
                                double current_eff_bps,
                                double current_sack_rate,
                                int window_count,
                                int wb_config_max) {
    ++eval_count;

    // Hard kill: table missing → caller stays put.
    if (!enabled) return current_cfg;
    // Cooldown active → caller stays put. Don't even log to keep the
    // diagnostic stream sparse.
    if (cooldown_remaining > 0) return current_cfg;

    // §4.3 statistical-confidence floor.
    if (window_count < 10) return current_cfg;

    // Optimizer only operates on WB OFDM ladder. ROBUST_X (>=100) and any
    // out-of-range value short-circuits — gearshift owns those transitions.
    // We deliberately don't try to up-shift OUT of robust here; the
    // existing turboshift path handles ROBUST → CONFIG_X.
    if (current_cfg < 0 || current_cfg > wb_config_max) return current_cfg;

    std::string current_bucket = channel_bucket_from_sack_rate(current_sack_rate);
    if (current_bucket.empty()) return current_cfg;

    // Score "stay". Prefer the table's eff_bps_mean at the channel bucket
    // matching the *observed* sack rate — that's what we're calibrated to
    // see right now. Fall back to current_eff_bps if the lookup misses.
    const st_rate_cell* stay_cell = get_cell(current_cfg, current_bucket);
    double stay_score;
    if (stay_cell && stay_cell->valid) {
        stay_score = stay_cell->eff_bps_mean;
    } else {
        // No table entry for our config at this channel → use the live
        // measurement. Better than discarding the score entirely.
        stay_score = current_eff_bps;
    }

    // Switch cost amortized across one batch cycle. Use stay_score as the
    // rate proxy → switch cost in BYTES = (cost_ms * stay_score / 8 / 1000).
    // We subtract it directly from the candidate score (in bps) by scaling
    // it to a per-batch wire window. A single batch at WB CONFIG_15 ~ 4500
    // bps PHY takes ~1.6-2.0s; we use 1.8s as the typical wire window.
    // The conservative model: amortize the cost across exactly one wire
    // window of the size we're currently spending. That penalty is what
    // separates "marginal" from "clearly better" alternative configs.
    const double WIRE_MS_PER_BATCH = 1800.0;
    double cost_penalty_bps = stay_score * ((double)switch_cost_ms / WIRE_MS_PER_BATCH);

    // Search the ±2 ladder neighborhood. We don't include ROBUST_X (those
    // are <0 or >=100). We DO allow target_cfg == 0 → wb_config_max.
    int best_cfg = current_cfg;
    double best_score = stay_score;
    int   best_target_seen = current_cfg;
    double best_target_score_seen = stay_score;

    for (int delta = -2; delta <= 2; ++delta) {
        if (delta == 0) continue;
        int cand = current_cfg + delta;
        if (cand < 0) continue;
        if (cand > wb_config_max) continue;

        // Predict the channel bucket at the target.
        double pred_sack = predicted_sack_at_target(current_cfg, cand, current_sack_rate);
        std::string cand_bucket = channel_bucket_from_sack_rate(pred_sack);
        if (cand_bucket.empty()) continue;

        const st_rate_cell* cand_cell = get_cell(cand, cand_bucket);
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
            } else {
                skip_reason = "below-hysteresis";
            }
        } else {
            // stay_score is 0 — the channel is dead at current_cfg per the
            // table. Switch IS warranted; the failure-recovery / BREAK
            // path already runs in parallel.
            chosen_cfg = best_cfg;
            cooldown_remaining = cooldown_max;
            gain_pct = 100.0;
        }
    }

    // Diag — emit one line per fire. Include the best-candidate-seen even
    // if we decided to stay, for offline tuning.
    if (best_target_seen != current_cfg) {
        if (chosen_cfg != current_cfg) {
            printf("[OPT-EVAL] eff=%.0f sack=%.2f curr_cfg=%d -> recommend=%d "
                   "(gain=%.1f%%) bucket=%s\n",
                   current_eff_bps, current_sack_rate, current_cfg, chosen_cfg,
                   gain_pct, current_bucket.c_str());
        } else {
            printf("[OPT-EVAL] eff=%.0f sack=%.2f curr_cfg=%d -> stay (best_cand=%d "
                   "gain=%.1f%% skip=%s) bucket=%s\n",
                   current_eff_bps, current_sack_rate, current_cfg, best_target_seen,
                   gain_pct, skip_reason ? skip_reason : "n/a",
                   current_bucket.c_str());
        }
        fflush(stdout);
    }

    return chosen_cfg;
}
