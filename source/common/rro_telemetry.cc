/* Opt-in RRO telemetry producer. A complete v1 snapshot is emitted from a
 * background thread; unavailable fields are explicit, never fabricated zeroes. */
#include "common/rro_telemetry.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <string>

#if defined(__has_include)
# if __has_include("common/build_id.h")
#  include "common/build_id.h"
# endif
#endif
#ifndef MERCURY_BUILD_ID
# define MERCURY_BUILD_ID "unknown-build"
#endif

#if defined(_WIN32)
# include <winsock2.h>
# include <ws2tcpip.h>
# include <process.h>
#else
# include <arpa/inet.h>
# include <sys/socket.h>
# include <unistd.h>
#endif

namespace rro {
namespace {

thread_local int current_correlator_lane = 0;

struct MetricSpec {
    const char* name;
    const char* type;
    const char* unit;
    const char* quality;
};

// Wire order is frozen by RRO's KNOWN_METRICS registry. Future source-backed
// fields must change from unavailable only after the actual producer hook is
// reviewed. The complete inventory keeps old receivers strict and safe.
const MetricSpec kMetrics[] = {
    {"audio.capture_ring_fill_ratio", "gauge", "1", "derived"},
    {"audio.capture_buffered_samples", "gauge", "sample", "measured"},
    {"audio.processing_load_ratio", "gauge", "1", "derived"},
    {"acquisition.active", "flag", "1", "measured"},
    {"acquisition.timing_offset_samples", "gauge", "sample", "measured"},
    {"acquisition.frequency_offset_hz", "gauge", "Hz", "measured"},
    {"acquisition.coarse_metric", "gauge", "1", "measured"},
    {"ofdm.fft_size", "gauge", "sample", "configured"},
    {"ofdm.active_carriers", "gauge", "carrier", "configured"},
    {"ofdm.pilot_carriers", "gauge", "carrier", "configured"},
    {"ofdm.data_carriers", "gauge", "carrier", "configured"},
    {"ofdm.current_config", "state", "config", "configured"},
    {"decode.ldpc_iterations", "gauge", "iteration", "measured"},
    {"decode.ldpc_max_iterations", "gauge", "iteration", "configured"},
    {"decode.crc_ok", "flag", "1", "measured"},
    {"decode.frames_total", "counter", "frame", "measured"},
    {"decode.frames_failed", "counter", "frame", "measured"},
    {"correlator.ack_invocations_total", "counter", "invocation", "measured"},
    {"correlator.ack_invocations_delta", "counter", "invocation", "derived"},
    {"correlator.ack_distinct_windows_total", "counter", "window", "measured"},
    {"correlator.ack_memo_reuses_total", "counter", "reuse", "measured"},
    {"correlator.ack_memo_enabled", "flag", "1", "configured"},
    {"correlator.hail_invocations_total", "counter", "invocation", "measured"},
    {"correlator.hail_invocations_delta", "counter", "invocation", "derived"},
    {"correlator.hail_distinct_windows_total", "counter", "window", "measured"},
    {"correlator.hail_memo_reuses_total", "counter", "reuse", "measured"},
    {"correlator.hail_memo_enabled", "flag", "1", "configured"},
    {"gearshift.lifecycle", "state", "state", "measured"},
    {"gearshift.activity", "state", "state", "measured"},
    {"gearshift.role", "state", "state", "measured"},
    {"gearshift.current_config", "state", "config", "configured"},
    {"gearshift.target_config", "state", "config", "configured"},
    {"gearshift.anchor_config", "state", "config", "configured"},
    {"gearshift.ceiling_config", "state", "config", "configured"},
    {"gearshift.last_batch_classification", "state", "state", "measured"},
    {"gearshift.clean_streak", "gauge", "batch", "measured"},
    {"gearshift.partial_streak", "gauge", "batch", "measured"},
    {"gearshift.backoff_active", "flag", "1", "measured"},
    {"gearshift.backoff_remaining_ms", "gauge", "ms", "derived"},
    {"gearshift.optimizer_enabled", "flag", "1", "configured"},
    {"gearshift.optimizer_target_config", "state", "config", "configured"},
    {"gearshift.break_active", "flag", "1", "measured"},
    {"gearshift.break_count_total", "counter", "break", "measured"},
};
static_assert(sizeof(kMetrics) / sizeof(kMetrics[0]) == 43,
              "RRO v1 metric inventory must remain complete");
constexpr std::uint64_t kMetricFreshnessNs = 2000000000ULL;

std::uint64_t monotonic_ns() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

std::uint64_t unix_ns() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
}

unsigned int process_id() {
#if defined(_WIN32)
    return static_cast<unsigned int>(_getpid());
#else
    return static_cast<unsigned int>(getpid());
#endif
}

std::string portable_build_id() {
    std::string value(MERCURY_BUILD_ID);
    if (value.empty()) value = "unknown-build";
    if (value.size() > 64) value.resize(64);
    for (char& ch : value) {
        if (!((ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z')
              || (ch >= '0' && ch <= '9') || ch == '.' || ch == '_' || ch == '-'))
            ch = '-';
    }
    return value;
}

bool configured_port(unsigned short& port) {
    const char* raw = std::getenv("MERCURY_RRO_UDP_PORT");
    if (raw == nullptr || *raw == '\0') { port = 38429; return true; }
    unsigned long value = 0;
    for (const char* p = raw; *p; ++p) {
        if (*p < '0' || *p > '9') return false;
        value = value * 10 + static_cast<unsigned long>(*p - '0');
        if (value > 65535) return false;
    }
    if (value == 0) return false;
    port = static_cast<unsigned short>(value);
    return true;
}

std::string config_state(int config) {
    if (config == -1) return "NONE";
    if (config >= 0 && config <= 17) return "CONFIG_" + std::to_string(config);
    if (config >= 100 && config <= 103) return "ROBUST_" + std::to_string(config - 100);
    if (config == 105) return "LOW48_ANCHOR_S20_R6";
    return "UNKNOWN";
}

const char* lifecycle_state(int value) {
    static const char* const states[] = {
        "IDLE", "CONNECTING", "CONNECTED", "DISCONNECTING", "LISTENING",
        "CONNECTION_RECEIVED", "CONNECTION_ACCEPTED", "NEGOTIATING",
    };
    if (value == -1) return "DROPPED";
    return value >= 0 && value < 8 ? states[value] : "UNKNOWN";
}

const char* activity_state(int value) {
    static const char* const states[] = {
        "IDLE", "TRANSMITTING_DATA", "RECEIVING", "RECEIVING_ACKS_DATA",
        "ACKNOWLEDGING_DATA", "TRANSMITTING_CONTROL", "RECEIVING_ACKS_CONTROL",
        "ACKNOWLEDGING_CONTROL",
    };
    return value >= 0 && value < 8 ? states[value] : "UNKNOWN";
}

const char* role_state(int value) {
    return value == 0 ? "COMMANDER" : value == 1 ? "RESPONDER" : "UNKNOWN";
}

const char* batch_classification_state(int value) {
    static const char* const states[] = {
        "NONE", "CLEAN", "PARTIAL", "FAILED", "UNKNOWN",
    };
    return value >= 0 && value < 5 ? states[value] : "UNKNOWN";
}

} // namespace

CorrelatorWindow::CorrelatorWindow(int lane)
    : previous_lane_(current_correlator_lane) {
    current_correlator_lane = lane >= 1 && lane <= 2 ? lane : 0;
    Telemetry::instance().record_correlator_window(current_correlator_lane);
}

CorrelatorWindow::~CorrelatorWindow() {
    current_correlator_lane = previous_lane_;
}

Telemetry& Telemetry::instance() {
    static Telemetry value;
    return value;
}

Telemetry::Telemetry() {
    const auto start_ms = unix_ns() / 1000000;
    session_id_ = "p" + std::to_string(process_id()) + "-" + std::to_string(start_ms);
}

Telemetry::~Telemetry() { stop(); }

void Telemetry::start_from_environment() {
    const char* enabled = std::getenv("MERCURY_RRO_TELEMETRY");
    if (enabled == nullptr || std::string(enabled) != "1") return;
    unsigned short port = 0;
    if (!configured_port(port)) {
        std::fprintf(stderr, "[RRO] invalid MERCURY_RRO_UDP_PORT; telemetry disabled\n");
        return;
    }
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) return;
    sender_ = std::thread(&Telemetry::sender_loop, this, port);
}

void Telemetry::stop() {
    running_.store(false, std::memory_order_release);
    if (sender_.joinable()) sender_.join();
}

void Telemetry::record_processing_load(double ratio) {
    if (!enabled()) return;
    if (!std::isfinite(ratio) || ratio < 0.0) {
        load_available_.store(false, std::memory_order_release);
        return;
    }
    processing_load_.store(ratio, std::memory_order_relaxed);
    load_sample_ns_.store(monotonic_ns(), std::memory_order_relaxed);
    load_available_.store(true, std::memory_order_release);
}

void Telemetry::record_capture_ring(std::size_t used, std::size_t capacity) {
    if (!enabled()) return;
    if (capacity == 0 || used > capacity
        || used > std::numeric_limits<std::uint32_t>::max()
        || capacity > std::numeric_limits<std::uint32_t>::max()) {
        ring_available_.store(false, std::memory_order_release);
        return;
    }
    const std::uint64_t pair =
        (static_cast<std::uint64_t>(capacity) << 32) | static_cast<std::uint64_t>(used);
    ring_used_capacity_.store(pair, std::memory_order_relaxed);
    ring_sample_ns_.store(monotonic_ns(), std::memory_order_relaxed);
    ring_available_.store(true, std::memory_order_release);
}

void Telemetry::record_receive_configuration(int fft_size, int ldpc_iteration_limit) {
    if (!enabled()) return;
    if (fft_size <= 0 || fft_size > 1048576
        || ldpc_iteration_limit <= 0 || ldpc_iteration_limit > 1000000) {
        configuration_sample_ns_.store(0, std::memory_order_release);
        return;
    }
    const std::uint64_t pair =
        (static_cast<std::uint64_t>(static_cast<std::uint32_t>(fft_size)) << 32)
        | static_cast<std::uint32_t>(ldpc_iteration_limit);
    receive_configuration_.store(pair, std::memory_order_relaxed);
    configuration_sample_ns_.store(monotonic_ns(), std::memory_order_release);
}

void Telemetry::record_ofdm_lattice(int active, int pilots, int data, int config) {
    if (!enabled()) return;
    if (active < 0 || active > 65535 || pilots < 0 || pilots > active
        || data < 0 || data > active || active > pilots + data
        || config < -1 || config > 65534) {
        clear_ofdm_lattice();
        return;
    }
    const std::uint64_t packed = (static_cast<std::uint64_t>(active) << 48)
        | (static_cast<std::uint64_t>(pilots) << 32)
        | (static_cast<std::uint64_t>(data) << 16)
        | static_cast<std::uint16_t>(config + 1);
    ofdm_lattice_.store(packed, std::memory_order_relaxed);
    lattice_sample_ns_.store(monotonic_ns(), std::memory_order_release);
}

void Telemetry::clear_ofdm_lattice() {
    if (!enabled()) return;
    lattice_sample_ns_.store(0, std::memory_order_release);
}

void Telemetry::record_ofdm_candidate(bool admitted) {
    if (!enabled()) return;
    candidate_admitted_.store(admitted, std::memory_order_relaxed);
    candidate_sample_ns_.store(monotonic_ns(), std::memory_order_release);
}

void Telemetry::record_acquisition_timing_residual(int samples) {
    if (!enabled()) return;
    timing_residual_samples_.store(samples, std::memory_order_relaxed);
    timing_sample_ns_.store(monotonic_ns(), std::memory_order_release);
}

void Telemetry::record_acquisition_frequency_offset(double hertz) {
    if (!enabled()) return;
    if (!std::isfinite(hertz) || std::fabs(hertz) > 1000000000.0) return;
    frequency_offset_hz_.store(hertz, std::memory_order_relaxed);
    frequency_sample_ns_.store(monotonic_ns(), std::memory_order_release);
}

void Telemetry::record_acquisition_coarse_metric(double correlation) {
    if (!enabled()) return;
    if (!std::isfinite(correlation) || correlation < 0.0 || correlation > 1.0)
        return;
    coarse_metric_.store(correlation, std::memory_order_relaxed);
    coarse_sample_ns_.store(monotonic_ns(), std::memory_order_release);
}

void Telemetry::record_ldpc_iterations(int iterations) {
    if (!enabled()) return;
    if (iterations < 0 || iterations > 1000000) return;
    ldpc_iterations_.store(iterations, std::memory_order_relaxed);
    ldpc_sample_ns_.store(monotonic_ns(), std::memory_order_release);
}

void Telemetry::record_crc_result(bool passed) {
    if (!enabled()) return;
    // The two receive loops can publish concurrently. The cumulative counts
    // are atomic; there is no lock, allocation or serialization on either path.
    crc_frames_total_.fetch_add(1);
    if (!passed) crc_frames_failed_.fetch_add(1);
    crc_verdict_sample_.store((monotonic_ns() << 1) | (passed ? 1u : 0u),
                              std::memory_order_release);
}

void Telemetry::record_gearshift(const GearshiftObservation& state) {
    if (!enabled()) return;
    if (state.clean_streak < 0 || state.partial_streak > 9007199254740991ULL
        || state.backoff_remaining_ms > 86400000ULL
        || state.break_count_total > 9007199254740991ULL) {
        gear_sample_ns_.store(0, std::memory_order_release);
        return;
    }
    // A single controller thread writes these fields. The sequence bracket is
    // intentionally seq_cst: the sender accepts only a fully stable reading.
    gear_generation_.fetch_add(1);
    gear_lifecycle_.store(state.lifecycle);
    gear_activity_.store(state.activity);
    gear_role_.store(state.role);
    gear_current_config_.store(state.current_config);
    gear_target_config_.store(state.target_config);
    gear_anchor_config_.store(state.anchor_config);
    gear_ceiling_config_.store(state.ceiling_config);
    gear_last_batch_classification_.store(state.last_batch_classification);
    gear_clean_streak_.store(state.clean_streak);
    gear_partial_streak_.store(state.partial_streak);
    gear_backoff_active_.store(state.backoff_active);
    gear_backoff_remaining_ms_.store(state.backoff_remaining_ms);
    gear_optimizer_enabled_.store(state.optimizer_enabled);
    gear_optimizer_target_config_.store(state.optimizer_target_config);
    gear_break_active_.store(state.break_active);
    gear_break_count_total_.store(state.break_count_total);
    gear_sample_ns_.store(monotonic_ns());
    gear_generation_.fetch_add(1);
}

void Telemetry::record_correlator_window(int lane) {
    if (!enabled() || lane < 1 || lane > 2) return;
    correlator_windows_[lane - 1].fetch_add(1, std::memory_order_relaxed);
}

void Telemetry::record_correlator_invocation(bool memo_enabled) {
    if (!enabled() || current_correlator_lane == 0) return;
    const int index = current_correlator_lane - 1;
    correlator_memo_enabled_[index].store(memo_enabled, std::memory_order_relaxed);
    correlator_invocations_[index].fetch_add(1, std::memory_order_relaxed);
    correlator_observed_[index].store(1, std::memory_order_release);
}

void Telemetry::record_correlator_memo_reuses(std::uint64_t reuses) {
    if (!enabled() || current_correlator_lane == 0 || reuses == 0) return;
    correlator_reuses_[current_correlator_lane - 1].fetch_add(reuses,
                                                             std::memory_order_relaxed);
}

std::string Telemetry::snapshot_json() {
    const auto seq = sequence_.fetch_add(1, std::memory_order_relaxed) + 1;
    const auto now_ns = monotonic_ns();
    const auto load_sample_ns = load_sample_ns_.load(std::memory_order_relaxed);
    const auto ring_sample_ns = ring_sample_ns_.load(std::memory_order_relaxed);
    const bool load_ready = load_available_.load(std::memory_order_acquire)
        && now_ns >= load_sample_ns && now_ns - load_sample_ns <= kMetricFreshnessNs;
    const bool ring_ready = ring_available_.load(std::memory_order_acquire)
        && now_ns >= ring_sample_ns && now_ns - ring_sample_ns <= kMetricFreshnessNs;
    const double load = processing_load_.load(std::memory_order_relaxed);
    const std::uint64_t ring = ring_used_capacity_.load(std::memory_order_relaxed);
    const std::uint32_t used = static_cast<std::uint32_t>(ring & 0xffffffffu);
    const std::uint32_t capacity = static_cast<std::uint32_t>(ring >> 32);
    const auto configuration_sample_ns = configuration_sample_ns_.load(std::memory_order_acquire);
    const bool configuration_ready = configuration_sample_ns != 0
        && now_ns >= configuration_sample_ns
        && now_ns - configuration_sample_ns <= kMetricFreshnessNs;
    const auto candidate_sample_ns = candidate_sample_ns_.load(std::memory_order_acquire);
    const bool candidate_ready = candidate_sample_ns != 0
        && now_ns >= candidate_sample_ns
        && now_ns - candidate_sample_ns <= kMetricFreshnessNs;
    const auto timing_sample_ns = timing_sample_ns_.load(std::memory_order_acquire);
    const auto frequency_sample_ns = frequency_sample_ns_.load(std::memory_order_acquire);
    const auto coarse_sample_ns = coarse_sample_ns_.load(std::memory_order_acquire);
    const bool timing_ready = timing_sample_ns != 0 && now_ns >= timing_sample_ns
        && now_ns - timing_sample_ns <= kMetricFreshnessNs;
    const bool frequency_ready = frequency_sample_ns != 0 && now_ns >= frequency_sample_ns
        && now_ns - frequency_sample_ns <= kMetricFreshnessNs;
    const bool coarse_ready = coarse_sample_ns != 0 && now_ns >= coarse_sample_ns
        && now_ns - coarse_sample_ns <= kMetricFreshnessNs;
    const int timing_residual = timing_residual_samples_.load(std::memory_order_relaxed);
    const double frequency_offset = frequency_offset_hz_.load(std::memory_order_relaxed);
    const double coarse_metric = coarse_metric_.load(std::memory_order_relaxed);
    const std::uint64_t configuration = receive_configuration_.load(std::memory_order_relaxed);
    const int fft_size = static_cast<int>(configuration >> 32);
    const int ldpc_limit = static_cast<int>(configuration & 0xffffffffu);
    const auto lattice_sample_ns = lattice_sample_ns_.load(std::memory_order_acquire);
    const auto lattice = ofdm_lattice_.load(std::memory_order_relaxed);
    const int active_carriers = static_cast<int>((lattice >> 48) & 0xffffu);
    const int pilot_carriers = static_cast<int>((lattice >> 32) & 0xffffu);
    const int data_carriers = static_cast<int>((lattice >> 16) & 0xffffu);
    const int lattice_config = static_cast<int>(lattice & 0xffffu) - 1;
    const bool lattice_ready = lattice_sample_ns != 0
        && now_ns >= lattice_sample_ns
        && now_ns - lattice_sample_ns <= kMetricFreshnessNs;
    const bool candidate_admitted = candidate_admitted_.load(std::memory_order_relaxed);
    const auto ldpc_sample_ns = ldpc_sample_ns_.load(std::memory_order_acquire);
    const bool ldpc_ready = ldpc_sample_ns != 0 && now_ns >= ldpc_sample_ns
        && now_ns - ldpc_sample_ns <= kMetricFreshnessNs;
    const int ldpc_iterations = ldpc_iterations_.load(std::memory_order_relaxed);
    const auto crc_sample = crc_verdict_sample_.load(std::memory_order_acquire);
    const auto crc_sample_ns = crc_sample >> 1;
    // Read failed before total. Writers advance total first, so the strict
    // receiver never sees more failed checks than total checks.
    const auto crc_failed = crc_frames_failed_.load();
    const auto crc_total = crc_frames_total_.load();
    const bool crc_passed = (crc_sample & 1u) != 0;
    const bool crc_ready = crc_sample_ns != 0
        && now_ns >= crc_sample_ns && now_ns - crc_sample_ns <= kMetricFreshnessNs;
    const bool crc_counts_ready = crc_sample_ns != 0
        && crc_total <= 9007199254740991ULL;
    GearshiftObservation gear{};
    std::uint64_t gear_sample_ns = 0;
    bool gear_coherent = false;
    for (int attempt = 0; attempt < 3; ++attempt) {
        const auto before = gear_generation_.load();
        if (before & 1u) continue;
        gear.lifecycle = gear_lifecycle_.load();
        gear.activity = gear_activity_.load();
        gear.role = gear_role_.load();
        gear.current_config = gear_current_config_.load();
        gear.target_config = gear_target_config_.load();
        gear.anchor_config = gear_anchor_config_.load();
        gear.ceiling_config = gear_ceiling_config_.load();
        gear.last_batch_classification = gear_last_batch_classification_.load();
        gear.clean_streak = gear_clean_streak_.load();
        gear.partial_streak = gear_partial_streak_.load();
        gear.backoff_active = gear_backoff_active_.load();
        gear.backoff_remaining_ms = gear_backoff_remaining_ms_.load();
        gear.optimizer_enabled = gear_optimizer_enabled_.load();
        gear.optimizer_target_config = gear_optimizer_target_config_.load();
        gear.break_active = gear_break_active_.load();
        gear.break_count_total = gear_break_count_total_.load();
        gear_sample_ns = gear_sample_ns_.load();
        if (gear_generation_.load() == before) {
            gear_coherent = true;
            break;
        }
    }
    const bool gear_ready = gear_coherent && gear_sample_ns != 0 && now_ns >= gear_sample_ns
        && now_ns - gear_sample_ns <= kMetricFreshnessNs;

    std::uint64_t correlator_calls[2]{};
    std::uint64_t correlator_delta[2]{};
    std::uint64_t correlator_windows[2]{};
    std::uint64_t correlator_reuses[2]{};
    bool correlator_memo[2]{};
    bool correlator_ready[2]{};
    for (int lane = 0; lane < 2; ++lane) {
        correlator_calls[lane] = correlator_invocations_[lane].load(std::memory_order_relaxed);
        const auto previous = correlator_snapshot_invocations_[lane].exchange(
            correlator_calls[lane], std::memory_order_relaxed);
        correlator_delta[lane] = correlator_calls[lane] >= previous
            ? correlator_calls[lane] - previous : 0;
        correlator_windows[lane] = correlator_windows_[lane].load(std::memory_order_relaxed);
        correlator_reuses[lane] = correlator_reuses_[lane].load(std::memory_order_relaxed);
        correlator_memo[lane] = correlator_memo_enabled_[lane].load(std::memory_order_relaxed);
        correlator_ready[lane] = correlator_observed_[lane].load(std::memory_order_acquire) != 0
            && correlator_calls[lane] <= 9007199254740991ULL
            && correlator_windows[lane] <= correlator_calls[lane]
            && correlator_reuses[lane] <= 9007199254740991ULL;
    }

    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(17);
    out << "{\"schema\":\"mercury.telemetry.snapshot\",\"version\":1,\"source\":{"
        << "\"source_id\":\"mercury\",\"session_id\":\"" << session_id_
        << "\",\"process_id\":" << process_id()
        << ",\"build_id\":\"" << portable_build_id() << "\"},"
        << "\"sequence\":\"" << seq << "\",\"monotonic_ns\":\"" << monotonic_ns()
        << "\",\"sent_unix_ns\":\"" << unix_ns() << "\",\"metrics\":{";
    for (std::size_t i = 0; i < sizeof(kMetrics) / sizeof(kMetrics[0]); ++i) {
        const auto& spec = kMetrics[i];
        if (i != 0) out << ',';
        out << '\"' << spec.name << "\":{\"available\":";
        const bool available = (i <= 1 && ring_ready && capacity > 0)
            || (i == 2 && load_ready)
            || (i == 3 && candidate_ready)
            || (i == 4 && timing_ready)
            || (i == 5 && frequency_ready)
            || (i == 6 && coarse_ready)
            || (i == 7 && configuration_ready)
            || (i >= 8 && i <= 11 && lattice_ready)
            || (i == 12 && ldpc_ready)
            || (i == 13 && configuration_ready)
            || (i == 14 && crc_ready)
            || ((i == 15 || i == 16) && crc_counts_ready)
            || (i >= 17 && i <= 21 && correlator_ready[0])
            || (i >= 22 && i <= 26 && correlator_ready[1])
            || (i >= 27 && gear_ready);
        if (!available) {
            out << "false,\"quality\":\"unavailable\",\"type\":\"" << spec.type
                << "\",\"unit\":\"" << spec.unit << "\",\"reason\":\""
                << "inactive\"}";
            continue;
        }
        out << "true,\"quality\":\"" << spec.quality << "\",\"type\":\""
            << spec.type << "\",\"unit\":\"" << spec.unit << "\",\"value\":";
        if (i == 0) out << static_cast<double>(used) / capacity;
        else if (i == 1) out << used;
        else if (i == 2) out << load;
        else if (i == 3) out << (candidate_admitted ? "true" : "false");
        else if (i == 4) out << timing_residual;
        else if (i == 5) out << frequency_offset;
        else if (i == 6) out << coarse_metric;
        else if (i == 7) out << fft_size;
        else if (i == 8) out << active_carriers;
        else if (i == 9) out << pilot_carriers;
        else if (i == 10) out << data_carriers;
        else if (i == 11) out << '\"' << config_state(lattice_config) << '\"';
        else if (i == 12) out << ldpc_iterations;
        else if (i == 14) out << (crc_passed ? "true" : "false");
        else if (i == 15) out << '\"' << crc_total << '\"';
        else if (i == 16) out << '\"' << crc_failed << '\"';
        else if (i >= 17 && i <= 26) {
            const int lane = i < 22 ? 0 : 1;
            switch ((i - 17) % 5) {
                case 0: out << '\"' << correlator_calls[lane] << '\"'; break;
                case 1: out << '\"' << correlator_delta[lane] << '\"'; break;
                case 2: out << '\"' << correlator_windows[lane] << '\"'; break;
                case 3: out << '\"' << correlator_reuses[lane] << '\"'; break;
                default: out << (correlator_memo[lane] ? "true" : "false"); break;
            }
        }
        else if (i == 27) out << '\"' << lifecycle_state(gear.lifecycle) << '\"';
        else if (i == 28) out << '\"' << activity_state(gear.activity) << '\"';
        else if (i == 29) out << '\"' << role_state(gear.role) << '\"';
        else if (i == 30) out << '\"' << config_state(gear.current_config) << '\"';
        else if (i == 31) out << '\"' << config_state(gear.target_config) << '\"';
        else if (i == 32) out << '\"' << config_state(gear.anchor_config) << '\"';
        else if (i == 33) out << '\"' << config_state(gear.ceiling_config) << '\"';
        else if (i == 34) out << '\"' << batch_classification_state(gear.last_batch_classification) << '\"';
        else if (i == 35) out << gear.clean_streak;
        else if (i == 36) out << gear.partial_streak;
        else if (i == 37) out << (gear.backoff_active ? "true" : "false");
        else if (i == 38) out << gear.backoff_remaining_ms;
        else if (i == 39) out << (gear.optimizer_enabled ? "true" : "false");
        else if (i == 40) out << '\"' << config_state(gear.optimizer_target_config) << '\"';
        else if (i == 41) out << (gear.break_active ? "true" : "false");
        else if (i == 42) out << '\"' << gear.break_count_total << '\"';
        else out << ldpc_limit;
        out << '}';
    }
    out << "}}";
    return out.str();
}

void Telemetry::sender_loop(unsigned short port) {
#if defined(_WIN32)
    WSADATA winsock;
    if (WSAStartup(MAKEWORD(2, 2), &winsock) != 0) {
        running_.store(false, std::memory_order_release);
        return;
    }
    SOCKET socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_fd == INVALID_SOCKET) {
        WSACleanup();
        running_.store(false, std::memory_order_release);
        return;
    }
#else
    int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0) {
        running_.store(false, std::memory_order_release);
        return;
    }
#endif
    sockaddr_in destination{};
    destination.sin_family = AF_INET;
    destination.sin_port = htons(port);
    destination.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    while (running_.load(std::memory_order_acquire)) {
        const auto next = std::chrono::steady_clock::now() + std::chrono::milliseconds(125);
        const std::string payload = snapshot_json();
        if (payload.size() <= 32768) {
            sendto(socket_fd, payload.data(), static_cast<int>(payload.size()), 0,
                   reinterpret_cast<const sockaddr*>(&destination), sizeof(destination));
        }
        std::this_thread::sleep_until(next);
    }
#if defined(_WIN32)
    closesocket(socket_fd);
    WSACleanup();
#else
    close(socket_fd);
#endif
}

} // namespace rro
