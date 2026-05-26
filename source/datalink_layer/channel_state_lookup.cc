/*
 * Mercury: A configurable open-source software-defined modem.
 * Copyright (C) 2022-2026 Fadi Jerji
 * Author: Fadi Jerji
 *
 * 2D channel-state → optimal-config lookup (Phase 2 Step 3).
 *
 * See: include/datalink_layer/channel_state_lookup.h
 *      mercury/fact-documents/channel-state-2d-lookup.md
 *
 * Self-contained — no dependency on cl_arq_controller, common_defines etc.
 *
 * Parsing strategy mirrors rate_optimizer.cc: the calibration table is
 * machine-written, so a tiny targeted JSON scanner is cheaper than
 * dragging in nlohmann / rapidjson (mercury vendors no JSON library).
 * Anything the scanner doesn't recognize is silently ignored, which is
 * forward-compatible with extra keys added by future calibration tools.
 *
 * Standalone unit test (not built by default):
 *   g++ -std=c++14 -I./include -DCL_CHANNEL_STATE_LOOKUP_TEST \
 *       source/datalink_layer/channel_state_lookup.cc \
 *       -o /tmp/channel_state_lookup_test && /tmp/channel_state_lookup_test
 */

#include "datalink_layer/channel_state_lookup.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>


// ---------- tiny JSON helpers (targeted, NOT a generic parser) ----------
//
// Mirrors the scanner in rate_optimizer.cc — same control flow, same
// limitations (no escape-handling in keys, no unicode normalization).
// Duplicated rather than shared because rate_optimizer.cc tucks these into
// an anonymous namespace and pulling them into a header would expand the
// public surface for what is a 60-line one-shot parser.

namespace {

// Strip // and /* */ comments. JSON spec disallows them; calibration JSON
// might include human notes, so be permissive.
std::string strip_comments(const std::string& in) {
    std::string out;
    out.reserve(in.size());
    size_t i = 0, n = in.size();
    while (i < n) {
        char c = in[i];
        if (c == '/' && i + 1 < n && in[i+1] == '/') {
            while (i < n && in[i] != '\n') ++i;
            continue;
        }
        if (c == '/' && i + 1 < n && in[i+1] == '*') {
            i += 2;
            while (i + 1 < n && !(in[i] == '*' && in[i+1] == '/')) ++i;
            i = (i + 1 < n) ? i + 2 : n;
            continue;
        }
        // String literal — copy verbatim. No backslash-quote support; the
        // calibration JSON never contains escaped quotes in keys / values.
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

// Find the value position for `key` inside the object whose '{' is at or
// just after `obj_brace_pos`. Returns npos if key not found at the
// immediate-child level. Nested objects skipped via brace-depth tracking.
size_t find_key_value_pos(const std::string& s, size_t obj_brace_pos, const std::string& key) {
    size_t i = obj_brace_pos;
    while (i < s.size() && std::isspace((unsigned char)s[i])) ++i;
    if (i >= s.size() || s[i] != '{') return std::string::npos;
    ++i;

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
            if (depth == 0) return std::string::npos;
            --depth; ++i; continue;
        }
        ++i;
    }
    return std::string::npos;
}

// Parse a JSON number / true / false / null at pos.
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

// Read a quoted string starting at pos (s[pos] must be '"'). Returns the
// raw string contents and advances *end past the closing quote.
bool parse_string(const std::string& s, size_t pos, std::string* out, size_t* end) {
    if (pos >= s.size() || s[pos] != '"') return false;
    ++pos;
    out->clear();
    while (pos < s.size() && s[pos] != '"') {
        if (s[pos] == '\\' && pos + 1 < s.size()) { ++pos; }
        *out += s[pos++];
    }
    if (pos >= s.size()) return false;
    *end = pos + 1;
    return true;
}

// Skip past a JSON array starting at s[pos] == '['. Used to advance to
// next sibling key. Returns position after the matching ']'.
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

// Parse a flat JSON array of numbers ([1, 2, 3]) into a vector.
bool parse_number_array(const std::string& s, size_t pos, std::vector<double>* out) {
    out->clear();
    if (pos >= s.size() || s[pos] != '[') return false;
    ++pos;
    while (pos < s.size()) {
        while (pos < s.size() && (std::isspace((unsigned char)s[pos]) || s[pos] == ',')) ++pos;
        if (pos >= s.size()) return false;
        if (s[pos] == ']') return true;
        size_t end;
        double v = parse_number_or_bool(s, pos, &end);
        if (end == pos) return false;   // not a number
        out->push_back(v);
        pos = end;
    }
    return false;
}

// Iterate immediate object keys at the brace pointed to by obj_pos. Calls
// cb(key, value_pos) for each. Mirrors rate_optimizer.cc's helper.
template <typename F>
void for_each_object_key(const std::string& s, size_t obj_pos, F cb) {
    if (obj_pos >= s.size() || s[obj_pos] != '{') return;
    size_t i = obj_pos + 1;
    while (i < s.size()) {
        while (i < s.size() && (std::isspace((unsigned char)s[i]) || s[i] == ',')) ++i;
        if (i >= s.size()) return;
        if (s[i] == '}') return;
        if (s[i] != '"') return;
        ++i;
        std::string key;
        while (i < s.size() && s[i] != '"') {
            if (s[i] == '\\' && i + 1 < s.size()) { ++i; }
            key += s[i++];
        }
        if (i >= s.size()) return;
        ++i;
        while (i < s.size() && std::isspace((unsigned char)s[i])) ++i;
        if (i >= s.size() || s[i] != ':') return;
        ++i;
        while (i < s.size() && std::isspace((unsigned char)s[i])) ++i;
        size_t value_pos = i;
        cb(key, value_pos);
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

}  // anonymous namespace


// ---------- cl_channel_state_lookup ----------

// Pre-C++17 requires an out-of-class definition for constexpr static members
// when they're ODR-used (e.g. passed by reference to printf, compared inside
// the test harness). Mercury builds with -std=c++14, so define them here.
constexpr int cl_channel_state_lookup::SENTINEL_NO_DATA;
constexpr int cl_channel_state_lookup::SENTINEL_DEAD;

cl_channel_state_lookup::cl_channel_state_lookup()
    : loaded_(false)
{}

cl_channel_state_lookup::~cl_channel_state_lookup() {}

bool cl_channel_state_lookup::init_from_json(const char* path) {
    loaded_ = false;
    last_error_.clear();
    snr_bins_.clear();
    sel_bins_.clear();
    cells_.clear();

    if (path == NULL || path[0] == '\0') {
        last_error_ = "null or empty path";
        return false;
    }
    std::ifstream f(path);
    if (!f.is_open()) {
        last_error_ = std::string("cannot open path=") + path;
        return false;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    std::string raw = ss.str();
    if (raw.empty()) {
        last_error_ = "file is empty";
        return false;
    }
    std::string body = strip_comments(raw);

    // Locate top-level "snr_bins", "sel_bins", "cells".
    size_t snr_pos   = find_key_value_pos(body, 0, "snr_bins");
    size_t sel_pos   = find_key_value_pos(body, 0, "sel_bins");
    size_t cells_pos = find_key_value_pos(body, 0, "cells");

    if (snr_pos == std::string::npos)   { last_error_ = "missing snr_bins";   return false; }
    if (sel_pos == std::string::npos)   { last_error_ = "missing sel_bins";   return false; }
    if (cells_pos == std::string::npos) { last_error_ = "missing cells";      return false; }

    if (!parse_number_array(body, snr_pos, &snr_bins_)) {
        last_error_ = "snr_bins not a number array";
        return false;
    }
    if (!parse_number_array(body, sel_pos, &sel_bins_)) {
        last_error_ = "sel_bins not a number array";
        return false;
    }
    if (snr_bins_.empty() || sel_bins_.empty()) {
        last_error_ = "snr_bins or sel_bins empty";
        return false;
    }

    // Sanity-check ordering. snr_bins DESCENDING, sel_bins ASCENDING.
    // Reject (rather than auto-sort) so calibration tool bugs surface
    // loudly — sorting silently would mask a bin/cell key mismatch.
    for (size_t i = 1; i < snr_bins_.size(); ++i) {
        if (snr_bins_[i] >= snr_bins_[i-1]) {
            last_error_ = "snr_bins not strictly descending";
            return false;
        }
    }
    for (size_t i = 1; i < sel_bins_.size(); ++i) {
        if (sel_bins_[i] <= sel_bins_[i-1]) {
            last_error_ = "sel_bins not strictly ascending";
            return false;
        }
    }

    // Initialize the cell grid to SENTINEL_NO_DATA. Cells appearing in the
    // JSON will overwrite their slots; cells absent from JSON stay NO_DATA.
    cells_.assign(snr_bins_.size(),
                  std::vector<int>(sel_bins_.size(), SENTINEL_NO_DATA));

    if (cells_pos >= body.size() || body[cells_pos] != '{') {
        last_error_ = "cells is not an object";
        return false;
    }

    // Iterate the "cells" object. Keys are "snr,sel" (e.g. "30,0.1"); values
    // are either an integer (config id) or the string "dead" (→ SENTINEL_DEAD).
    bool any_cell = false;
    for_each_object_key(body, cells_pos, [&](const std::string& key, size_t v) {
        // Split "snr,sel"
        size_t comma = key.find(',');
        if (comma == std::string::npos) return;
        double k_snr = std::strtod(key.c_str(), NULL);
        double k_sel = std::strtod(key.c_str() + comma + 1, NULL);

        // Bins must exist EXACTLY (don't snap inputs from JSON — those are
        // canonical bin edges, not measurements).
        int si = -1;
        for (size_t i = 0; i < snr_bins_.size(); ++i) {
            if (std::fabs(snr_bins_[i] - k_snr) < 1e-9) { si = (int)i; break; }
        }
        int xi = -1;
        for (size_t i = 0; i < sel_bins_.size(); ++i) {
            if (std::fabs(sel_bins_[i] - k_sel) < 1e-9) { xi = (int)i; break; }
        }
        if (si < 0 || xi < 0) return;  // cell key references unknown bin

        // Value: either "dead" string or a number.
        if (v < body.size() && body[v] == '"') {
            std::string sval;
            size_t end;
            if (parse_string(body, v, &sval, &end)) {
                if (sval == "dead") {
                    cells_[si][xi] = SENTINEL_DEAD;
                    any_cell = true;
                }
                // Any other string ignored — forward compatibility.
            }
        } else {
            size_t end;
            double n = parse_number_or_bool(body, v, &end);
            if (end != v) {
                // Config ids are integers in [0, 102]. Anything else ignored.
                int cfg = (int)std::lround(n);
                if (cfg >= 0 && cfg <= 102) {
                    cells_[si][xi] = cfg;
                    any_cell = true;
                }
            }
        }
    });

    if (!any_cell) {
        last_error_ = "cells object parsed but no valid entries";
        return false;
    }

    loaded_ = true;
    return true;
}

// SNR bins are DESCENDING — find the largest i with snr_db >= snr_bins_[i].
// Above the top edge → bin 0; below the bottom edge → last bin.
int cl_channel_state_lookup::snr_bin_index(double snr_db) const {
    if (snr_bins_.empty()) return -1;
    // Bin 0 covers [snr_bins_[0], +inf). Walk down until snr_db >= edge.
    for (size_t i = 0; i < snr_bins_.size(); ++i) {
        if (snr_db >= snr_bins_[i]) return (int)i;
    }
    return (int)(snr_bins_.size() - 1);
}

// Sel bins are ASCENDING — find the largest i with selectivity >= sel_bins_[i].
int cl_channel_state_lookup::sel_bin_index(double selectivity) const {
    if (sel_bins_.empty()) return -1;
    // Walk up; remember the last bin whose edge we've crossed.
    int best = 0;
    for (size_t i = 0; i < sel_bins_.size(); ++i) {
        if (selectivity >= sel_bins_[i]) best = (int)i;
        else break;
    }
    return best;
}

int cl_channel_state_lookup::lookup(double snr_db, double selectivity) const {
    if (!loaded_) return SENTINEL_NO_DATA;
    int si = snr_bin_index(snr_db);
    int xi = sel_bin_index(selectivity);
    if (si < 0 || xi < 0) return SENTINEL_NO_DATA;
    if ((size_t)si >= cells_.size() || (size_t)xi >= cells_[si].size())
        return SENTINEL_NO_DATA;
    return cells_[si][xi];
}

int cl_channel_state_lookup::n_cells() const {
    int n = 0;
    for (size_t i = 0; i < cells_.size(); ++i) {
        for (size_t j = 0; j < cells_[i].size(); ++j) {
            if (cells_[i][j] != SENTINEL_NO_DATA) ++n;
        }
    }
    return n;
}


// ---------- standalone unit test ----------
//
// Build (one line):
//   g++ -std=c++14 -I./include -DCL_CHANNEL_STATE_LOOKUP_TEST source/datalink_layer/channel_state_lookup.cc -o /tmp/channel_state_lookup_test && /tmp/channel_state_lookup_test
//
// Not compiled into mercury; the binary remains byte-identical without
// the define. Test writes a small JSON to a temp file, loads it, and
// verifies a handful of (snr, sel) -> config lookups.

#ifdef CL_CHANNEL_STATE_LOOKUP_TEST
#include <cstdio>

static int g_failures = 0;
static void check_eq(int got, int want, const char* tag) {
    if (got != want) {
        std::fprintf(stderr, "FAIL %s: got %d want %d\n", tag, got, want);
        ++g_failures;
    } else {
        std::fprintf(stderr, "ok   %s: %d\n", tag, got);
    }
}

int main() {
    // Tiny calibration table: 3 SNR bins x 3 selectivity bins.
    // Cell coverage:
    //   (30, 0.0)→16  (30, 0.1)→14  (30, 0.5)→8
    //   (12, 0.0)→10  (12, 0.1)→8   (12, 0.5)→100  (ROBUST_0)
    //   ( 0, 0.0)→100 ( 0, 0.1)→"dead" ( 0, 0.5) missing
    const char* path = "/tmp/_channel_state_lookup_test.json";
    {
        FILE* fp = std::fopen(path, "w");
        if (!fp) { std::fprintf(stderr, "cannot write %s\n", path); return 2; }
        std::fprintf(fp,
            "{\n"
            "  \"snr_bins\": [30, 12, 0],\n"
            "  \"sel_bins\": [0.0, 0.1, 0.5],\n"
            "  \"cells\": {\n"
            "    \"30,0.0\": 16, \"30,0.1\": 14, \"30,0.5\": 8,\n"
            "    \"12,0.0\": 10, \"12,0.1\":  8, \"12,0.5\": 100,\n"
            "    \"0,0.0\":  100, \"0,0.1\": \"dead\"\n"
            "  }\n"
            "}\n");
        std::fclose(fp);
    }

    cl_channel_state_lookup t;
    if (!t.init_from_json(path)) {
        std::fprintf(stderr, "init_from_json failed: %s\n", t.last_error());
        return 2;
    }
    std::fprintf(stderr, "loaded n_cells=%d\n", t.n_cells());

    // Exact bin edges.
    check_eq(t.lookup(30.0, 0.0), 16, "exact (30,0.0)");
    check_eq(t.lookup(12.0, 0.1),  8, "exact (12,0.1)");
    check_eq(t.lookup( 0.0, 0.0), 100, "exact (0,0.0) ROBUST_0");

    // "Above top" SNR clamps to bin 0.
    check_eq(t.lookup(50.0, 0.0), 16, "above-top SNR clamps to (30,0.0)");

    // "Below bottom" SNR clamps to last bin.
    check_eq(t.lookup(-10.0, 0.0), 100, "below-bottom SNR clamps to (0,0.0)");

    // Mid-range SNR falls into the lower-edge bin. 20 >= 12 but < 30 → bin 1.
    check_eq(t.lookup(20.0, 0.0), 10, "snr=20 → bin (12,*)");

    // Mid-range selectivity: 0.3 >= 0.1 but < 0.5 → bin 1.
    check_eq(t.lookup(30.0, 0.3), 14, "sel=0.3 → bin (*,0.1)");

    // Selectivity above top → last sel bin.
    check_eq(t.lookup(30.0, 0.9), 8, "sel=0.9 clamps to (*,0.5)");

    // Selectivity below bottom (negative) → first sel bin.
    check_eq(t.lookup(30.0, -0.1), 16, "sel=-0.1 clamps to (*,0.0)");

    // Dead cell.
    check_eq(t.lookup(0.0, 0.1), cl_channel_state_lookup::SENTINEL_DEAD, "(0,0.1) dead");

    // Missing cell (0, 0.5) → SENTINEL_NO_DATA.
    check_eq(t.lookup(0.0, 0.5), cl_channel_state_lookup::SENTINEL_NO_DATA, "(0,0.5) missing");

    // Empty / unloaded instance returns NO_DATA on every lookup.
    cl_channel_state_lookup empty;
    check_eq(empty.lookup(30.0, 0.0),
             cl_channel_state_lookup::SENTINEL_NO_DATA,
             "unloaded → NO_DATA");

    std::remove(path);

    if (g_failures) {
        std::fprintf(stderr, "FAIL: %d test(s) failed\n", g_failures);
        return 1;
    }
    std::fprintf(stderr, "PASS: all checks ok\n");
    return 0;
}
#endif  // CL_CHANNEL_STATE_LOOKUP_TEST
