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

struct GearshiftObservation {
    int lifecycle;
    int activity;
    int role;
    int current_config;
    int anchor_config;
    int ceiling_config;
    int clean_streak;
    bool backoff_active;
    unsigned long long backoff_remaining_ms;
    bool optimizer_enabled;
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
    void record_ofdm_candidate(bool admitted);
    void record_ldpc_iterations(int iterations);
    void record_crc_result(bool passed);
    void record_gearshift(const GearshiftObservation& observation);

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
    std::atomic<int> fft_size_{0};
    std::atomic<int> ldpc_iteration_limit_{0};
    std::atomic<std::uint64_t> configuration_sample_ns_{0};
    std::atomic<bool> candidate_admitted_{false};
    std::atomic<std::uint64_t> candidate_sample_ns_{0};
    std::atomic<int> ldpc_iterations_{0};
    std::atomic<std::uint64_t> ldpc_sample_ns_{0};
    std::atomic<bool> crc_passed_{false};
    std::atomic<std::uint64_t> crc_sample_ns_{0};
    std::atomic<int> gear_lifecycle_{0};
    std::atomic<int> gear_activity_{0};
    std::atomic<int> gear_role_{0};
    std::atomic<int> gear_current_config_{-1};
    std::atomic<int> gear_anchor_config_{-1};
    std::atomic<int> gear_ceiling_config_{-1};
    std::atomic<int> gear_clean_streak_{0};
    std::atomic<bool> gear_backoff_active_{false};
    std::atomic<unsigned long long> gear_backoff_remaining_ms_{0};
    std::atomic<bool> gear_optimizer_enabled_{false};
    std::atomic<bool> gear_break_active_{false};
    std::atomic<unsigned long long> gear_break_count_total_{0};
    std::atomic<std::uint64_t> gear_sample_ns_{0};
    std::atomic<std::uint64_t> sequence_{0};
    std::string session_id_;
    std::thread sender_;
};

} // namespace rro

#endif
