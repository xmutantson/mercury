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
 * 2D channel-state → optimal-config lookup (Phase 2 Step 3).
 *
 * See: mercury/fact-documents/channel-state-2d-lookup.md
 *
 * Self-contained — no dependency on cl_arq_controller, common_defines,
 * physical_config etc. Reads only stdlib + the channel_state_lookup.h
 * declarations. No call sites yet (those arrive in Step 5); this class
 * is built but inert.
 *
 * The table is a 2D grid keyed by (SNR bucket, selectivity bucket); cells
 * hold the calibrated optimal config id for that channel state. Sentinels
 * distinguish "no data" (lookup() returns NO_DATA → caller stays put) from
 * "dead" (calibration recorded "nothing worked here" → caller knows the
 * channel is gone).
 */

#ifndef CHANNEL_STATE_LOOKUP_H_
#define CHANNEL_STATE_LOOKUP_H_

#include <string>
#include <vector>

class cl_channel_state_lookup {
public:
    cl_channel_state_lookup();
    ~cl_channel_state_lookup();

    // Load the 2D table from a JSON file. Returns true on success, false on
    // any parse / file error (also sets last_error_). On failure the table
    // stays in the "no data" state and lookup() returns SENTINEL_NO_DATA.
    bool init_from_json(const char* path);

    // Lookup. Returns optimal config id (0..16 for OFDM, 100..102 for ROBUST)
    // for the bucket containing (snr_db, selectivity), or:
    //   SENTINEL_NO_DATA   if no table loaded or bucket has no data
    //   SENTINEL_DEAD      if calibration recorded "no config worked" here
    // Caller checks against the sentinels and decides what to do.
    int lookup(double snr_db, double selectivity) const;

    // Inspection / diagnostics
    bool is_loaded() const { return loaded_; }
    int n_cells() const;   // number of populated cells (not SENTINEL_NO_DATA)
    const char* last_error() const { return last_error_.c_str(); }

    // Sentinel values for lookup() return.
    static constexpr int SENTINEL_NO_DATA = -1;
    static constexpr int SENTINEL_DEAD    = -2;

private:
    // Bin layout:
    //   snr_bins_ is DESCENDING (high SNR first), e.g. [30, 20, 12, 6, 0].
    //     Given a measured SNR, the bin index is the largest i such that
    //     snr_db >= snr_bins_[i]. Inputs above the top bin clamp to 0;
    //     inputs below the bottom bin clamp to last index.
    //   sel_bins_ is ASCENDING (low selectivity first), e.g.
    //     [0.0, 0.1, 0.2, 0.3, 0.5, 0.7]. Given a measured selectivity, the
    //     bin index is the largest i such that selectivity >= sel_bins_[i].
    //     Same clamp logic at both ends.
    //   cells_ is row-major [snr_bin_idx][sel_bin_idx] → config id or sentinel.
    //
    // Sentinel-vs-config distinction lives in the cell value itself; no
    // separate "valid" bitmap. This is fine because config ids are in
    // [0, 102] and sentinels are negative.
    bool loaded_;
    std::string last_error_;
    std::vector<double> snr_bins_;
    std::vector<double> sel_bins_;
    std::vector<std::vector<int> > cells_;

    // Pure helpers — exposed only for the in-file unit test.
    int snr_bin_index(double snr_db) const;
    int sel_bin_index(double selectivity) const;
};

#endif  // CHANNEL_STATE_LOOKUP_H_
