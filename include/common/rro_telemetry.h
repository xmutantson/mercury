/* Optional, loopback-only RRO snapshot producer. No serialization or I/O runs
 * on the modem's audio/decode paths. Disabled unless MERCURY_RRO_TELEMETRY=1. */
#ifndef MERCURY_RRO_TELEMETRY_H_
#define MERCURY_RRO_TELEMETRY_H_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace rro {

// Marks one submitted passband window. Nested scopes restore the caller's lane;
// only ACK and HAIL searches are counted, never the shared detector's other uses.
class CorrelatorWindow final {
public:
    explicit CorrelatorWindow(int lane);
    ~CorrelatorWindow();
    CorrelatorWindow(const CorrelatorWindow&) = delete;
    CorrelatorWindow& operator=(const CorrelatorWindow&) = delete;
private:
    int previous_lane_;
    bool previous_pending_;
};

struct GearshiftObservation {
    int lifecycle;
    int activity;
    int role;
    int current_config;
    int target_config;
    int anchor_config;
    int ceiling_config;
    int last_batch_classification;
    int clean_streak;
    unsigned long long partial_streak;
    bool backoff_active;
    unsigned long long backoff_remaining_ms;
    bool optimizer_enabled;
    int optimizer_target_config;
    bool break_active;
    unsigned long long break_count_total;
};

class Telemetry final {
public:
    static Telemetry& instance();

    // Starts an 8 Hz sender only when explicitly enabled. The destination is
    // always 127.0.0.1; MERCURY_RRO_UDP_PORT may select its port.
    void start_from_environment();
    void stop();
    bool enabled() const { return running_.load(std::memory_order_relaxed); }

    // Source-backed v1 fields. A read before the first source write is
    // unavailable/inactive, never a measured zero.
    void record_processing_load(double processing_time_over_frame_period);
    void record_capture_ring(std::size_t buffered_samples, std::size_t capacity_samples);
    void record_receive_configuration(int fft_size, int ldpc_iteration_limit);
    void record_ofdm_lattice(int active_carriers, int pilot_carriers,
                             int data_carriers, int current_config);
    void clear_ofdm_lattice();
    void record_ofdm_candidate(bool admitted);
    void record_acquisition_timing_residual(int samples);
    void record_acquisition_frequency_offset(double hertz);
    void record_acquisition_coarse_metric(double correlation);
    void record_ldpc_iterations(int iterations);
    void record_crc_result(bool passed);
    void record_gearshift(const GearshiftObservation& observation);
    void record_correlator_invocation(bool memo_enabled);
    void record_correlator_memo_reuses(std::uint64_t reuses);

    // v2 detail observations. Every method is a no-op unless telemetry is on;
    // the v1 packet remains the default wire format.
    void record_ofdm_fft_execution(int symbols, double duration_ms);
    void record_channel_estimate(int model_cells_count, int pilot_observations_count,
        double pilot_coherence, double pilot_selectivity, double noise_variance,
        double mean_h, int estimator_kind, bool model_published);
    void record_demapper_output(int modulation_family, int modulation_order,
        int llr_count, double mean_abs_llr, double weak_fraction,
        bool soft_bits_published);
    void record_ldpc_decode_result(int iterations, int iteration_limit,
        bool converged, double duration_ms);
    void record_correlator_detection(std::uint64_t fft_executions,
        std::uint64_t duration_ns, double best_metric, int best_matched,
        int expected_symbols, bool evaluated);
    void record_arq_state(int role, int decision, int batch_seq_id,
        int retransmit_frames, int batch_frames, bool sack_enabled,
        int sack_window_bits, int sack_ack_bits, int timeout_remaining_ms);
    void record_arq_sack_window(int rx_batch_seq_id, int nbits,
        const unsigned char* bitmap, int nbytes);
    void record_arq_decision(int decision);
    void record_gearshift_decision(double forward_snr_db, bool forward_snr_valid,
        int forward_snr_age_batches, double reverse_snr_db, bool reverse_snr_valid,
        int reverse_snr_age_batches, int action, int recommended_config,
        const char* reason);
    void record_capture_window_handoff(int samples);
    void record_audio_capture_arrived(std::uint64_t samples, std::uint64_t sample_ns);
    void record_audio_capture_written(std::uint64_t samples, std::uint64_t sample_ns);
    void record_audio_capture_read(std::uint64_t samples, std::uint64_t sample_ns);
    void record_audio_capture_reset(std::uint64_t sample_ns);
    void record_audio_input_state(bool available, bool simulated, std::uint64_t sample_ns);
    void record_audio_capture_cursor(std::uint64_t head_samples,
        std::uint64_t tail_samples, std::uint64_t capacity_samples,
        bool full, std::uint64_t sample_ns);
    void record_acquisition_transition(int event, int delay_samples, double metric,
                                       int current_config);
    void record_carrier_bin_handoff(std::uint64_t data_mask, std::uint64_t pilot_mask,
        int carrier_count, int data_cells, int pilot_cells, int current_config);
    void record_ldpc_codeword_handoff(bool converged, int decoded_bits, int iterations);
    void record_crc_frame_outcome(bool accepted, int reject_reason);
    void record_correlator_outcome(int lane, bool accepted, double metric_threshold,
                                   int matched_threshold);
    void record_gearshift_local_config(int from, int to, int role);
    void record_gearshift_engagement(int from, int to, int action, int phase,
        const char* selection_reason, const char* confirmation_reason);
    void record_gearshift_probe_state(bool active);
    void record_gearshift_session_reset();

    // Also used by the cross-repository schema test. It does not send traffic.
    std::string snapshot_json();
    std::string snapshot_json_v2();

private:
    Telemetry();
    ~Telemetry();
    Telemetry(const Telemetry&) = delete;
    Telemetry& operator=(const Telemetry&) = delete;

    void sender_loop(unsigned short port);
    void set_detail_number(std::size_t index, double value, std::uint64_t sample_ns = 0);
    void set_detail_integer(std::size_t index, std::uint64_t value,
                            std::uint64_t sample_ns = 0);
    void increment_detail(std::size_t index, std::uint64_t amount,
                          std::uint64_t sample_ns = 0);
    void clear_detail(std::size_t index);
    std::atomic<bool> running_{false};
    std::atomic<bool> load_available_{false};
    std::atomic<double> processing_load_{0.0};
    std::atomic<std::uint64_t> load_sample_ns_{0};
    std::atomic<bool> ring_available_{false};
    // One atomic pair makes occupancy and its derived fraction one observation.
    std::atomic<std::uint64_t> ring_used_capacity_{0};
    std::atomic<std::uint64_t> ring_sample_ns_{0};
    // Both configured values are observed together by the receive path.
    std::atomic<std::uint64_t> receive_configuration_{0};
    std::atomic<std::uint64_t> configuration_sample_ns_{0};
    // Four 16-bit fields form one configured carrier-geometry observation.
    std::atomic<std::uint64_t> ofdm_lattice_{0};
    std::atomic<std::uint64_t> lattice_sample_ns_{0};
    std::atomic<bool> candidate_admitted_{false};
    std::atomic<std::uint64_t> candidate_sample_ns_{0};
    std::atomic<int> timing_residual_samples_{0};
    std::atomic<std::uint64_t> timing_sample_ns_{0};
    std::atomic<double> frequency_offset_hz_{0.0};
    std::atomic<std::uint64_t> frequency_sample_ns_{0};
    std::atomic<double> coarse_metric_{0.0};
    std::atomic<std::uint64_t> coarse_sample_ns_{0};
    std::atomic<int> ldpc_iterations_{0};
    std::atomic<std::uint64_t> ldpc_sample_ns_{0};
    // One word pairs the verdict with its timestamp even if both receive loops
    // publish concurrently; low bit is pass, upper bits monotonic nanoseconds.
    std::atomic<std::uint64_t> crc_verdict_sample_{0};
    std::atomic<std::uint64_t> crc_frames_total_{0};
    std::atomic<std::uint64_t> crc_frames_failed_{0};
    // One controller writer, one sender reader. A bounded version check keeps
    // the related controller values in one observation without locks.
    std::atomic<std::uint64_t> gear_generation_{0};
    std::atomic<int> gear_lifecycle_{0};
    std::atomic<int> gear_activity_{0};
    std::atomic<int> gear_role_{0};
    std::atomic<int> gear_current_config_{-1};
    std::atomic<int> gear_target_config_{-1};
    std::atomic<int> gear_anchor_config_{-1};
    std::atomic<int> gear_ceiling_config_{-1};
    std::atomic<int> gear_last_batch_classification_{0};
    std::atomic<int> gear_clean_streak_{0};
    std::atomic<unsigned long long> gear_partial_streak_{0};
    std::atomic<bool> gear_backoff_active_{false};
    std::atomic<unsigned long long> gear_backoff_remaining_ms_{0};
    std::atomic<bool> gear_optimizer_enabled_{false};
    std::atomic<int> gear_optimizer_target_config_{-1};
    std::atomic<bool> gear_break_active_{false};
    std::atomic<unsigned long long> gear_break_count_total_{0};
    std::atomic<std::uint64_t> gear_sample_ns_{0};
    std::atomic<std::uint64_t> correlator_invocations_[2]{};
    std::atomic<std::uint64_t> correlator_windows_[2]{};
    std::atomic<std::uint64_t> correlator_reuses_[2]{};
    std::atomic<bool> correlator_memo_enabled_[2]{};
    std::atomic<std::uint64_t> correlator_observed_[2]{};
    std::atomic<std::uint64_t> correlator_generation_[2]{};
    std::atomic<unsigned int> correlator_writers_[2]{};
    std::atomic<std::uint64_t> correlator_snapshot_invocations_[2]{};
    static constexpr std::size_t kDetailCount = 88;
    struct DetailCell {
        std::atomic<double> number{0.0};
        std::atomic<std::uint64_t> integer{0};
        std::atomic<std::uint64_t> sample_ns{0};
    };
    DetailCell detail_[kDetailCount]{};
    std::atomic<std::uint64_t> capture_previous_arrival_ns_{0};
    std::atomic<unsigned char> sack_bytes_[12]{};
    std::atomic<std::uint64_t> sack_generation_{0};
    std::atomic<bool> sack_writer_{false};
    std::atomic<char> gear_reason_[64]{};
    std::atomic<std::uint64_t> gear_decision_generation_{0};
    std::mutex gear_decision_write_mutex_;
    std::atomic<char> engagement_selection_reason_[64]{};
    std::atomic<char> engagement_confirmation_reason_[64]{};
    std::atomic<std::uint64_t> engagement_generation_{0};
    std::mutex engagement_write_mutex_;
    std::atomic<std::uint64_t> ring_cursor_generation_{0};
    std::atomic<bool> ring_cursor_writer_{false};
    std::atomic<std::uint64_t> ring_cursor_callback_ns_{0};
    std::atomic<std::uint64_t> carrier_data_mask_{0};
    std::atomic<std::uint64_t> carrier_pilot_mask_{0};
    std::atomic<std::uint64_t> carrier_generation_{0};
    std::mutex carrier_write_mutex_;
    std::atomic<std::uint64_t> crc_outcome_sequence_{0};
    std::atomic<std::uint64_t> crc_recent_outcomes_[32]{};
    std::atomic<std::uint64_t> crc_recent_sample_ns_[32]{};
    std::atomic<int> packet_version_{1};
    std::atomic<std::uint64_t> sequence_{0};
    std::string session_id_;
    std::thread sender_;
};

} // namespace rro

#endif
