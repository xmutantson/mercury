/*
 * Mercury/Quicksilver Gearshift v2 measurement reducer.
 *
 * Outcome evidence and rate evidence have separate validity populations.  An
 * L1 aggregate replay can truthfully say that a batch was clean/partial without
 * inventing a duration; a completed live exchange contributes both outcome and
 * rate evidence.  Application bytes and banked transport bytes are separate so
 * selective repair can provide fast progress evidence without being mistaken
 * for final application delivery.
 */
#ifndef OPTIMIZER_METRICS_H_
#define OPTIMIZER_METRICS_H_

struct st_optimizer_window_metrics {
    double application_bps;
    double transport_bps;
    double sack_batch_rate;
    double frame_success_rate;
    double partial_frame_loss_rate;
    double failed_batch_rate;
    int rate_sample_count;
    int outcome_sample_count;
    int application_commit_count;
    unsigned long long application_bytes;
    unsigned long long transport_bytes;
    unsigned long long cycle_ms;

    st_optimizer_window_metrics()
        : application_bps(0.0), transport_bps(0.0), sack_batch_rate(0.0),
          frame_success_rate(1.0), partial_frame_loss_rate(0.0),
          failed_batch_rate(0.0), rate_sample_count(0),
          outcome_sample_count(0), application_commit_count(0),
          application_bytes(0), transport_bytes(0), cycle_ms(0) {}
};

inline st_optimizer_window_metrics optimizer_reduce_window(
        const unsigned int* application_bytes,
        const unsigned int* transport_bytes,
        const unsigned int* cycle_ms,
        const unsigned char* rate_valid,
        const unsigned char* outcome_valid,
        const unsigned char* application_committed,
        const unsigned char* sack_used,
        const unsigned char* failed,
        const unsigned int* frames_acked,
        const unsigned int* frames_sent,
        int head, int count, int capacity)
{
    st_optimizer_window_metrics out;
    if (!application_bytes || !transport_bytes || !cycle_ms || !rate_valid ||
        !outcome_valid || !application_committed || !sack_used || !failed ||
        !frames_acked || !frames_sent || capacity <= 0 || count <= 0)
        return out;

    unsigned long long acked = 0, sent = 0;
    unsigned long long partial_missing = 0, partial_sent = 0;
    int sacks = 0, failures = 0;
    if (count > capacity) count = capacity;

    for (int i=0; i<count; ++i) {
        const int idx = (head - 1 - i + capacity) % capacity;
        if (outcome_valid[idx]) {
            ++out.outcome_sample_count;
            sacks += sack_used[idx] ? 1 : 0;
            failures += failed[idx] ? 1 : 0;
            if (frames_sent[idx] > 0) {
                unsigned int a = frames_acked[idx];
                if (a > frames_sent[idx]) a = frames_sent[idx];
                acked += a;
                sent += frames_sent[idx];
                if (sack_used[idx]) {
                    partial_missing += frames_sent[idx] - a;
                    partial_sent += frames_sent[idx];
                }
            }
        }
        if (rate_valid[idx] && cycle_ms[idx] > 0) {
            ++out.rate_sample_count;
            out.application_bytes += application_bytes[idx];
            out.transport_bytes += transport_bytes[idx];
            out.cycle_ms += cycle_ms[idx];
            if (application_committed[idx]) ++out.application_commit_count;
        }
    }

    if (out.cycle_ms > 0) {
        out.application_bps = (double)out.application_bytes * 8000.0 /
                              (double)out.cycle_ms;
        out.transport_bps = (double)out.transport_bytes * 8000.0 /
                            (double)out.cycle_ms;
    }
    if (out.outcome_sample_count > 0) {
        out.sack_batch_rate = (double)sacks/(double)out.outcome_sample_count;
        out.failed_batch_rate = (double)failures/(double)out.outcome_sample_count;
    }
    if (sent > 0) out.frame_success_rate = (double)acked/(double)sent;
    if (partial_sent > 0)
        out.partial_frame_loss_rate = (double)partial_missing/(double)partial_sent;
    return out;
}

inline bool optimizer_commit_application_to_slot(
        unsigned int* application_bytes,
        unsigned char* application_committed,
        const unsigned char* rate_valid,
        int capacity, int slot, unsigned int bytes)
{
    if (!application_bytes || !application_committed || !rate_valid ||
        capacity <= 0 || slot < 0 || slot >= capacity || bytes == 0)
        return false;
    if (!rate_valid[slot] || application_committed[slot]) return false;
    application_bytes[slot] += bytes;
    application_committed[slot] = 1;
    return true;
}

#endif // OPTIMIZER_METRICS_H_
