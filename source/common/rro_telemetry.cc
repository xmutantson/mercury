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

} // namespace

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
    fft_size_.store(fft_size, std::memory_order_relaxed);
    ldpc_iteration_limit_.store(ldpc_iteration_limit, std::memory_order_relaxed);
    configuration_sample_ns_.store(monotonic_ns(), std::memory_order_release);
}

void Telemetry::record_ofdm_candidate(bool admitted) {
    if (!enabled()) return;
    candidate_admitted_.store(admitted, std::memory_order_relaxed);
    candidate_sample_ns_.store(monotonic_ns(), std::memory_order_release);
}

void Telemetry::record_ldpc_iterations(int iterations) {
    if (!enabled()) return;
    if (iterations < 0 || iterations > 1000000) return;
    ldpc_iterations_.store(iterations, std::memory_order_relaxed);
    ldpc_sample_ns_.store(monotonic_ns(), std::memory_order_release);
}

void Telemetry::record_crc_result(bool passed) {
    if (!enabled()) return;
    crc_passed_.store(passed, std::memory_order_relaxed);
    crc_sample_ns_.store(monotonic_ns(), std::memory_order_release);
}

void Telemetry::record_gearshift(const GearshiftObservation& state) {
    if (!enabled()) return;
    if (state.clean_streak < 0 || state.backoff_remaining_ms > 86400000ULL
        || state.break_count_total > 9007199254740991ULL) {
        gear_sample_ns_.store(0, std::memory_order_release);
        return;
    }
    gear_lifecycle_.store(state.lifecycle, std::memory_order_relaxed);
    gear_activity_.store(state.activity, std::memory_order_relaxed);
    gear_role_.store(state.role, std::memory_order_relaxed);
    gear_current_config_.store(state.current_config, std::memory_order_relaxed);
    gear_anchor_config_.store(state.anchor_config, std::memory_order_relaxed);
    gear_ceiling_config_.store(state.ceiling_config, std::memory_order_relaxed);
    gear_clean_streak_.store(state.clean_streak, std::memory_order_relaxed);
    gear_backoff_active_.store(state.backoff_active, std::memory_order_relaxed);
    gear_backoff_remaining_ms_.store(state.backoff_remaining_ms, std::memory_order_relaxed);
    gear_optimizer_enabled_.store(state.optimizer_enabled, std::memory_order_relaxed);
    gear_break_active_.store(state.break_active, std::memory_order_relaxed);
    gear_break_count_total_.store(state.break_count_total, std::memory_order_relaxed);
    gear_sample_ns_.store(monotonic_ns(), std::memory_order_release);
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
    const int fft_size = fft_size_.load(std::memory_order_relaxed);
    const int ldpc_limit = ldpc_iteration_limit_.load(std::memory_order_relaxed);
    const bool candidate_admitted = candidate_admitted_.load(std::memory_order_relaxed);
    const auto ldpc_sample_ns = ldpc_sample_ns_.load(std::memory_order_acquire);
    const bool ldpc_ready = ldpc_sample_ns != 0 && now_ns >= ldpc_sample_ns
        && now_ns - ldpc_sample_ns <= kMetricFreshnessNs;
    const auto crc_sample_ns = crc_sample_ns_.load(std::memory_order_acquire);
    const bool crc_ready = crc_sample_ns != 0 && now_ns >= crc_sample_ns
        && now_ns - crc_sample_ns <= kMetricFreshnessNs;
    const int ldpc_iterations = ldpc_iterations_.load(std::memory_order_relaxed);
    const bool crc_passed = crc_passed_.load(std::memory_order_relaxed);
    const auto gear_sample_ns = gear_sample_ns_.load(std::memory_order_acquire);
    const bool gear_ready = gear_sample_ns != 0 && now_ns >= gear_sample_ns
        && now_ns - gear_sample_ns <= kMetricFreshnessNs;

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
            || (i == 7 && configuration_ready)
            || (i == 12 && ldpc_ready)
            || (i == 13 && configuration_ready)
            || (i == 14 && crc_ready)
            || (i >= 27 && gear_ready && i != 31 && i != 34 && i != 36 && i != 40);
        if (!available) {
            out << "false,\"quality\":\"unavailable\",\"type\":\"" << spec.type
                << "\",\"unit\":\"" << spec.unit << "\",\"reason\":\""
                << ((i <= 3 || i == 7 || (i >= 12 && i <= 14)
                    || (i >= 27 && i != 31 && i != 34 && i != 36 && i != 40))
                    ? "inactive" : "not_instrumented") << "\"}";
            continue;
        }
        out << "true,\"quality\":\"" << spec.quality << "\",\"type\":\""
            << spec.type << "\",\"unit\":\"" << spec.unit << "\",\"value\":";
        if (i == 0) out << static_cast<double>(used) / capacity;
        else if (i == 1) out << used;
        else if (i == 2) out << load;
        else if (i == 3) out << (candidate_admitted ? "true" : "false");
        else if (i == 7) out << fft_size;
        else if (i == 12) out << ldpc_iterations;
        else if (i == 14) out << (crc_passed ? "true" : "false");
        else if (i == 27) out << '\"' << lifecycle_state(gear_lifecycle_.load()) << '\"';
        else if (i == 28) out << '\"' << activity_state(gear_activity_.load()) << '\"';
        else if (i == 29) out << '\"' << role_state(gear_role_.load()) << '\"';
        else if (i == 30) out << '\"' << config_state(gear_current_config_.load()) << '\"';
        else if (i == 32) out << '\"' << config_state(gear_anchor_config_.load()) << '\"';
        else if (i == 33) out << '\"' << config_state(gear_ceiling_config_.load()) << '\"';
        else if (i == 35) out << gear_clean_streak_.load();
        else if (i == 37) out << (gear_backoff_active_.load() ? "true" : "false");
        else if (i == 38) out << gear_backoff_remaining_ms_.load();
        else if (i == 39) out << (gear_optimizer_enabled_.load() ? "true" : "false");
        else if (i == 41) out << (gear_break_active_.load() ? "true" : "false");
        else if (i == 42) out << '\"' << gear_break_count_total_.load() << '\"';
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
