/* Optional, loopback-only RRO snapshot producer. No serialization or I/O runs
 * on the modem's audio/decode paths. Disabled unless MERCURY_RRO_TELEMETRY=1. */
#ifndef MERCURY_RRO_TELEMETRY_H_
#define MERCURY_RRO_TELEMETRY_H_

#include <atomic>
#include <cstddef>
#include <cstdint>
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
    void record_correlator_window(int lane);
    void record_correlator_invocation(bool memo_enabled);
    void record_correlator_memo_reuses(std::uint64_t reuses);

    // Also used by the cross-repository schema test. It does not send traffic.
    std::string snapshot_json();

private:
    Telemetry();
    ~Telemetry();
    Telemetry(const Telemetry&) = delete;
    Telemetry& operator=(const Telemetry&) = delete;

    void sender_loop(unsigned short port);
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
    std::atomic<std::uint64_t> correlator_snapshot_invocations_[2]{};
    std::atomic<std::uint64_t> sequence_{0};
    std::string session_id_;
    std::thread sender_;
};

} // namespace rro

#endif
