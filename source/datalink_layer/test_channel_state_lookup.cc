/*
 * Mercury: A configurable open-source software-defined modem.
 * Copyright (C) 2022-2026 Fadi Jerji
 * Author: Fadi Jerji
 *
 */

#include "datalink_layer/channel_state_lookup.h"


// ---------- standalone unit test ----------
//
// Build (one line):
//   g++ -std=c++14 -I./include -DCL_CHANNEL_STATE_LOOKUP_TEST source/datalink_layer/channel_state_lookup.cc source/datalink_layer/test_channel_state_lookup.cc -o test_channel_state_lookup && ./test_channel_state_lookup
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
