/*
 * Quicksilver Gearshift-v2 analytical geometry helpers.
 *
 * Pure functions only: candidate scoring must be able to price a PHY action
 * without loading/mutating the live modem.  Keeping this arithmetic testable
 * prevents the adaptive layer from drifting away from framing geometry.
 */
#ifndef OPTIMIZER_GEOMETRY_H_
#define OPTIMIZER_GEOMETRY_H_

#include <algorithm>
#include <cmath>
#include <vector>
#include "datalink_layer/datalink_defines.h"
#include "physical_layer/physical_defines.h"

struct st_bigblock_candidate_geometry {
    bool valid;
    int codewords;
    int keydown_ms;
    int full_batch_payload_bytes;
    st_bigblock_candidate_geometry()
        : valid(false), codewords(0), keydown_ms(0), full_batch_payload_bytes(0) {}
};

inline st_bigblock_candidate_geometry optimizer_predict_bigblock_geometry(
        int Nc, int Nfft, int Ngi, int preamble_nsymb, int interpolation_rate,
        int modulation_order, int ldpc_N, int ldpc_K, double sampling_frequency,
        int Ngrid, int cont_cols, int scat_dx, int scat_dy, int k_cap)
{
    st_bigblock_candidate_geometry out;
    if (Nc <= 0 || Nfft <= 0 || Ngi < 0 || preamble_nsymb < 0 ||
        interpolation_rate <= 0 || modulation_order <= 1 || ldpc_N <= 0 ||
        ldpc_K <= 0 || sampling_frequency <= 0.0 || Ngrid <= 0)
        return out;

    const int log2M = (int)std::llround(std::log2((double)modulation_order));
    if (log2M <= 0) return out;

    // Mirror telecom_system::bigblock_rebuild_thin_grid(): start all DATA,
    // mark continuous pilot columns, then the time/frequency scatter lattice.
    std::vector<unsigned char> pilot((size_t)Ngrid * (size_t)Nc, 0);
    if (cont_cols > 0) {
        for (int c=0; c<cont_cols; ++c) {
            const int col = (cont_cols <= 1) ? 0
                : (int)std::llround((double)c * (double)(Nc-1) /
                                    (double)(cont_cols-1));
            if (col < 0 || col >= Nc) continue;
            for (int n=0; n<Ngrid; ++n)
                pilot[(size_t)n*(size_t)Nc + (size_t)col] = 1;
        }
    }
    if (scat_dx > 0 && scat_dy > 0) {
        for (int n=0; n<Ngrid; ++n) {
            if (n % scat_dy != 0) continue;
            const int off = (n/scat_dy) * (scat_dx/2 > 0 ? scat_dx/2 : 1);
            for (int j=off % scat_dx; j<Nc; j+=scat_dx)
                pilot[(size_t)n*(size_t)Nc + (size_t)j] = 1;
        }
    }
    int np = 0;
    for (size_t i=0; i<pilot.size(); ++i) np += pilot[i] ? 1 : 0;
    const int ndata = Ngrid * Nc - np;
    if (ndata <= 0) return out;

    const int nbits = ndata * log2M;
    int K = nbits / ldpc_N;
    if (k_cap > 0 && k_cap < K) K = k_cap;
    if (K <= 0) return out;

    int sub_len = ldpc_K / 8;
    if (sub_len <= 0) return out;
    const int alloc_size = N_MAX / 8;
    if (sub_len > alloc_size) sub_len = alloc_size;

    const int hdr = BIGBLOCK_HDR_TOTAL_BYTES(K);
    const int cw0 = sub_len - hdr - BIGBLOCK_CW_CRC_BYTES;
    const int cwmid = sub_len - BIGBLOCK_CW_CRC_BYTES;
    const int cwlast = cwmid - BIGBLOCK_BLOCK_CRC_BYTES;
    if (cw0 < 0 || cwmid < 0 || cwlast < 0) return out;

    int payload = 0;
    if (K == 1) {
        // One codeword is simultaneously first and last.
        payload = sub_len - hdr - BIGBLOCK_CW_CRC_BYTES - BIGBLOCK_BLOCK_CRC_BYTES;
    } else {
        payload = cw0 + cwlast;
        if (K > 2) payload += (K-2) * cwmid;
    }
    if (payload < 0) return out;

    const long long nofdm = (long long)Nfft + (long long)Ngi;
    const long long samples = nofdm * (long long)(preamble_nsymb + Ngrid) *
                              (long long)interpolation_rate;
    const int ms = (int)std::llround(1000.0 * (double)samples / sampling_frequency);
    if (ms <= 0) return out;

    out.valid = true;
    out.codewords = K;
    out.keydown_ms = ms;
    out.full_batch_payload_bytes = payload;
    return out;
}

#endif // OPTIMIZER_GEOMETRY_H_
