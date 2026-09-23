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
thread_local bool current_correlator_window_pending = false;

struct MetricSpec {
    const char* name;
    const char* type;
    const char* unit;
    const char* quality;
};

enum DetailIndex {
    AudioInputAvailable, AudioSimulated, AudioArrivals, AudioArrivalSamples,
    AudioInterarrivalMs, AudioRingWrites, AudioRingReads, AudioRingResets,
    OfdmSymbolDemodExecutions, OfdmSymbolDemodDuration,
    ChannelPilotObservations, ChannelPilotCoherence, ChannelPilotSelectivity,
    ChannelNoiseVariance, ChannelModelCells, ChannelMeanMagnitude,
    ChannelEstimatorKind, ChannelModelPublished,
    DemapperModulation, DemapperSoftBits, DemapperMeanAbsLlr,
    DemapperNearZeroFraction, DemapperSoftBitsPublished,
    LdpcConverged, LdpcDuration,
    AckFftExecutions, AckDetectorDuration, AckBestMetric, AckMatchedSymbols,
    AckExpectedSymbols, HailFftExecutions, HailDetectorDuration, HailBestMetric,
    HailMatchedSymbols, HailExpectedSymbols,
    ArqDecision, ArqRetryOccupancy, ArqNextBatchSequence, ArqBatchWidth,
    ArqSackWindowWidth, ArqSackAckCount, ArqSackMask, ArqSackBatchSequence,
    ArqRetryTimeout,
    GearForwardSnr, GearForwardSnrAge, GearReverseSnr, GearReverseSnrAge,
    GearDecisionAction, GearDecisionTarget, GearDecisionReason,
    AudioRingWriteIndex, AudioRingReadIndex, AudioRingCapacity, AudioRingFull,
    AudioWindowHandoffs, AudioWindowSamples,
    AcquisitionTransition, AcquisitionTransitionDelay, AcquisitionTransitionConfig,
    CarrierDataMask, CarrierPilotMask, CarrierWidth, CarrierDataCells,
    CarrierPilotCells,
    LdpcCodewordHandoffs, LdpcCodewordBits, LdpcCodewordIterations,
    CrcFrameOutcome, CrcFrameRejectReason, CrcRecentChecked,
    CrcRecentRejected, CrcRecentWindow,
    AckPatternOutcome, AckMetricThreshold, AckMatchedThreshold,
    HailPatternOutcome, HailMetricThreshold, HailMatchedThreshold,
    GearLocalFrom, GearLocalTo, GearEngagementPhase, GearEngagementFrom,
    GearEngagementTo, GearEngagementAction, GearEngagementSelectionReason,
    GearEngagementConfirmationReason, GearProbeActive,
    DetailIndexCount
};
static_assert(DetailIndexCount == 88, "RRO v2 detail count");

struct DetailSpec {
    const char* name;
    const char* type;
    const char* unit;
    const char* quality;
    bool persistent;
};
const DetailSpec kDetailMetrics[] = {
    {"audio.capture_input_available", "flag", "1", "measured", true},
    {"audio.capture_simulated", "flag", "1", "measured", true},
    {"audio.capture_arrivals_total", "counter", "chunk", "measured", true},
    {"audio.capture_arrival_samples_total", "counter", "sample", "measured", true},
    {"audio.capture_interarrival_ms", "gauge", "ms", "measured", false},
    {"audio.capture_ring_written_samples_total", "counter", "sample", "measured", true},
    {"audio.capture_ring_read_samples_total", "counter", "sample", "measured", true},
    {"audio.capture_ring_resets_total", "counter", "reset", "measured", true},
    {"ofdm.symbol_demod_executions_total", "counter", "execution", "measured", true},
    {"ofdm.symbol_demod_duration_ms", "gauge", "ms", "measured", false},
    {"channel.pilot_observations_count", "gauge", "pilot", "measured", false},
    {"channel.pilot_coherence", "gauge", "1", "measured", false},
    {"channel.pilot_selectivity", "gauge", "1", "measured", false},
    {"channel.noise_variance", "gauge", "power", "measured", false},
    {"channel.model_cells_count", "gauge", "cell", "measured", false},
    {"channel.mean_channel_magnitude", "gauge", "1", "measured", false},
    {"channel.estimator_kind", "state", "method", "configured", false},
    {"channel.model_built", "flag", "1", "measured", false},
    {"demapper.modulation", "state", "mode", "configured", false},
    {"demapper.soft_bits_count", "gauge", "bit", "measured", false},
    {"demapper.llr_mean_abs", "gauge", "LLR", "derived", false},
    {"demapper.llr_near_zero_fraction", "gauge", "1", "derived", false},
    {"demapper.soft_bits_published", "flag", "1", "measured", false},
    {"decode.ldpc_converged", "flag", "1", "measured", false},
    {"decode.ldpc_duration_ms", "gauge", "ms", "measured", false},
    {"correlator.ack_fft_executions_total", "counter", "FFT", "measured", true},
    {"correlator.ack_detector_duration_ms", "gauge", "ms", "measured", false},
    {"correlator.ack_best_metric", "gauge", "score", "measured", false},
    {"correlator.ack_matched_symbols", "gauge", "symbol", "measured", false},
    {"correlator.ack_expected_symbols", "gauge", "symbol", "configured", false},
    {"correlator.hail_fft_executions_total", "counter", "FFT", "measured", true},
    {"correlator.hail_detector_duration_ms", "gauge", "ms", "measured", false},
    {"correlator.hail_best_metric", "gauge", "score", "measured", false},
    {"correlator.hail_matched_symbols", "gauge", "symbol", "measured", false},
    {"correlator.hail_expected_symbols", "gauge", "symbol", "configured", false},
    {"arq.decision", "state", "state", "measured", false},
    {"arq.retry_occupancy", "gauge", "frame", "measured", false},
    {"arq.next_batch_sequence", "gauge", "sequence", "measured", false},
    {"arq.batch_width", "gauge", "frame", "configured", false},
    {"arq.sack_window_width", "gauge", "bit", "measured", false},
    {"arq.sack_ack_count", "gauge", "bit", "measured", false},
    {"arq.sack_mask", "state", "mask", "measured", false},
    {"arq.sack_batch_sequence", "gauge", "sequence", "measured", false},
    {"arq.retry_timeout_remaining_ms", "gauge", "ms", "derived", false},
    {"gearshift.forward_snr_db", "gauge", "dB", "measured", false},
    {"gearshift.forward_snr_age_batches", "gauge", "batch", "derived", false},
    {"gearshift.reverse_snr_db", "gauge", "dB", "measured", false},
    {"gearshift.reverse_snr_age_batches", "gauge", "batch", "derived", false},
    {"gearshift.decision_action", "state", "state", "measured", false},
    {"gearshift.decision_target_config", "state", "config", "derived", false},
    {"gearshift.decision_reason", "state", "reason", "measured", false},
    {"audio.capture_ring_write_index_samples", "gauge", "sample", "measured", false},
    {"audio.capture_ring_read_index_samples", "gauge", "sample", "measured", false},
    {"audio.capture_ring_capacity_samples", "gauge", "sample", "configured", false},
    {"audio.capture_ring_full", "flag", "1", "measured", false},
    {"audio.capture_window_handoffs_total", "counter", "window", "measured", true},
    {"audio.capture_window_samples", "gauge", "sample", "measured", false},
    {"acquisition.transition", "state", "event", "measured", false},
    {"acquisition.transition_delay_samples", "gauge", "sample", "measured", false},
    {"acquisition.transition_config", "state", "config", "configured", false},
    {"ofdm.carrier_data_mask", "state", "mask", "measured", false},
    {"ofdm.carrier_pilot_mask", "state", "mask", "measured", false},
    {"ofdm.carrier_width", "gauge", "carrier", "configured", false},
    {"ofdm.carrier_data_cells", "gauge", "cell", "measured", false},
    {"ofdm.carrier_pilot_cells", "gauge", "cell", "measured", false},
    {"decode.ldpc_codeword_handoffs_total", "counter", "codeword", "measured", true},
    {"decode.ldpc_codeword_bits", "gauge", "bit", "measured", false},
    {"decode.ldpc_codeword_iterations", "gauge", "iteration", "measured", false},
    {"crc.frame_outcome", "state", "outcome", "measured", false},
    {"crc.frame_reject_reason", "state", "reason", "measured", false},
    {"crc.recent_checked_frames", "gauge", "frame", "derived", false},
    {"crc.recent_rejected_frames", "gauge", "frame", "derived", false},
    {"crc.recent_window_frames", "gauge", "frame", "configured", false},
    {"correlator.ack_pattern_outcome", "state", "outcome", "measured", false},
    {"correlator.ack_metric_threshold", "gauge", "score", "configured", false},
    {"correlator.ack_matched_threshold", "gauge", "symbol", "configured", false},
    {"correlator.hail_pattern_outcome", "state", "outcome", "measured", false},
    {"correlator.hail_metric_threshold", "gauge", "score", "configured", false},
    {"correlator.hail_matched_threshold", "gauge", "symbol", "configured", false},
    {"gearshift.local_from_config", "state", "config", "measured", false},
    {"gearshift.local_to_config", "state", "config", "measured", false},
    {"gearshift.engagement_phase", "state", "phase", "measured", false},
    {"gearshift.engagement_from_config", "state", "config", "measured", false},
    {"gearshift.engagement_to_config", "state", "config", "measured", false},
    {"gearshift.engagement_action", "state", "state", "measured", false},
    {"gearshift.engagement_selection_reason", "state", "reason", "measured", false},
    {"gearshift.engagement_confirmation_reason", "state", "reason", "measured", false},
    {"gearshift.probe_active", "flag", "1", "measured", false},
};
static_assert(sizeof(kDetailMetrics) / sizeof(kDetailMetrics[0]) == DetailIndexCount,
              "RRO v2 detail registry must be complete");

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
    : previous_lane_(current_correlator_lane),
      previous_pending_(current_correlator_window_pending) {
    current_correlator_lane = lane >= 1 && lane <= 2 ? lane : 0;
    current_correlator_window_pending = current_correlator_lane != 0;
}

CorrelatorWindow::~CorrelatorWindow() {
    current_correlator_lane = previous_lane_;
    current_correlator_window_pending = previous_pending_;
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
    const char* version = std::getenv("MERCURY_RRO_TELEMETRY_VERSION");
    if (version != nullptr && *version != '\0'
        && std::string(version) != "1" && std::string(version) != "2") {
        std::fprintf(stderr, "[RRO] invalid MERCURY_RRO_TELEMETRY_VERSION; telemetry disabled\n");
        return;
    }
    packet_version_.store(version != nullptr && std::string(version) == "2" ? 2 : 1,
                          std::memory_order_release);
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
    {
        std::lock_guard<std::mutex> guard(carrier_write_mutex_);
        carrier_generation_.fetch_add(1, std::memory_order_acq_rel);
        for (int i = CarrierDataMask; i <= CarrierPilotCells; ++i) clear_detail(i);
        carrier_generation_.fetch_add(1, std::memory_order_release);
    }
    for (int i = ChannelPilotObservations; i <= ChannelModelPublished; ++i) clear_detail(i);
    for (int i = DemapperModulation; i <= DemapperSoftBitsPublished; ++i) clear_detail(i);
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

void Telemetry::record_correlator_invocation(bool memo_enabled) {
    if (!enabled() || current_correlator_lane == 0) return;
    const int index = current_correlator_lane - 1;
    // ACK/HAIL can be submitted from more than one processing path. Bracket
    // the paired counters without a hot-path mutex so the sender can reject a
    // torn interval rather than inventing a calls/window ratio.
    correlator_writers_[index].fetch_add(1, std::memory_order_acq_rel);
    correlator_generation_[index].fetch_add(1, std::memory_order_acq_rel);
    correlator_memo_enabled_[index].store(memo_enabled, std::memory_order_relaxed);
    correlator_invocations_[index].fetch_add(1, std::memory_order_relaxed);
    if (current_correlator_window_pending) {
        correlator_windows_[index].fetch_add(1, std::memory_order_relaxed);
        current_correlator_window_pending = false;
    }
    correlator_observed_[index].store(1, std::memory_order_relaxed);
    correlator_generation_[index].fetch_add(1, std::memory_order_release);
    correlator_writers_[index].fetch_sub(1, std::memory_order_release);
}

void Telemetry::record_correlator_memo_reuses(std::uint64_t reuses) {
    if (!enabled() || current_correlator_lane == 0 || reuses == 0) return;
    correlator_reuses_[current_correlator_lane - 1].fetch_add(reuses,
                                                             std::memory_order_relaxed);
}

void Telemetry::set_detail_number(std::size_t index, double value, std::uint64_t sample_ns) {
    if (index >= kDetailCount || !std::isfinite(value)) return;
    detail_[index].number.store(value, std::memory_order_relaxed);
    detail_[index].sample_ns.store(sample_ns ? sample_ns : monotonic_ns(),
                                   std::memory_order_release);
}

void Telemetry::set_detail_integer(std::size_t index, std::uint64_t value,
                                   std::uint64_t sample_ns) {
    if (index >= kDetailCount) return;
    detail_[index].integer.store(value, std::memory_order_relaxed);
    detail_[index].sample_ns.store(sample_ns ? sample_ns : monotonic_ns(),
                                   std::memory_order_release);
}

void Telemetry::increment_detail(std::size_t index, std::uint64_t amount,
                                 std::uint64_t sample_ns) {
    if (index >= kDetailCount || amount == 0) return;
    detail_[index].integer.fetch_add(amount, std::memory_order_relaxed);
    detail_[index].sample_ns.store(sample_ns ? sample_ns : monotonic_ns(),
                                   std::memory_order_release);
}

void Telemetry::clear_detail(std::size_t index) {
    if (index < kDetailCount)
        detail_[index].sample_ns.store(0, std::memory_order_release);
}

void Telemetry::record_audio_capture_arrived(std::uint64_t samples, std::uint64_t sample_ns) {
    if (!enabled() || samples == 0) return;
    if (sample_ns == 0) sample_ns = monotonic_ns();
    increment_detail(AudioArrivals, 1, sample_ns);
    increment_detail(AudioArrivalSamples, samples, sample_ns);
    const auto previous = capture_previous_arrival_ns_.exchange(sample_ns);
    if (previous != 0 && sample_ns > previous)
        set_detail_number(AudioInterarrivalMs, (sample_ns - previous) / 1000000.0,
                          sample_ns);
}

void Telemetry::record_audio_capture_written(std::uint64_t samples, std::uint64_t sample_ns) {
    if (enabled()) increment_detail(AudioRingWrites, samples, sample_ns);
}

void Telemetry::record_audio_capture_read(std::uint64_t samples, std::uint64_t sample_ns) {
    if (enabled()) increment_detail(AudioRingReads, samples, sample_ns);
}

void Telemetry::record_audio_capture_reset(std::uint64_t sample_ns) {
    if (enabled()) increment_detail(AudioRingResets, 1, sample_ns);
}

void Telemetry::record_audio_input_state(bool available, bool simulated,
                                         std::uint64_t sample_ns) {
    if (!enabled()) return;
    set_detail_integer(AudioInputAvailable, available ? 1 : 0, sample_ns);
    set_detail_integer(AudioSimulated, simulated ? 1 : 0, sample_ns);
    capture_previous_arrival_ns_.store(0, std::memory_order_release);
    clear_detail(AudioInterarrivalMs);
    if (!available) {
        bool expected = false;
        if (ring_cursor_writer_.compare_exchange_strong(expected, true,
                std::memory_order_acquire)) {
            ring_cursor_generation_.fetch_add(1, std::memory_order_acq_rel);
            for (int i = AudioRingWriteIndex; i <= AudioRingFull; ++i) clear_detail(i);
            ring_cursor_generation_.fetch_add(1, std::memory_order_release);
            ring_cursor_writer_.store(false, std::memory_order_release);
        }
    }
}

void Telemetry::record_audio_capture_cursor(std::uint64_t head, std::uint64_t tail,
    std::uint64_t capacity, bool full, std::uint64_t sample_ns) {
    if (!enabled() || capacity == 0 || head >= capacity || tail >= capacity
        || capacity > 9007199254740991ULL) return;
    if (sample_ns == 0) sample_ns = monotonic_ns();
    // Producer/consumer callbacks can race. Drop a collided callback instead
    // of blocking the audio path; the sender accepts only a complete bracket.
    bool expected = false;
    if (!ring_cursor_writer_.compare_exchange_strong(expected, true,
            std::memory_order_acquire)) return;
    const auto previous = ring_cursor_callback_ns_.load(std::memory_order_acquire);
    if (sample_ns >= previous) {
        ring_cursor_generation_.fetch_add(1, std::memory_order_acq_rel);
        set_detail_integer(AudioRingWriteIndex, head, sample_ns);
        set_detail_integer(AudioRingReadIndex, tail, sample_ns);
        set_detail_integer(AudioRingCapacity, capacity, sample_ns);
        set_detail_integer(AudioRingFull, full ? 1 : 0, sample_ns);
        ring_cursor_callback_ns_.store(sample_ns, std::memory_order_release);
        ring_cursor_generation_.fetch_add(1, std::memory_order_release);
    }
    ring_cursor_writer_.store(false, std::memory_order_release);
}

void Telemetry::record_capture_window_handoff(int samples) {
    if (!enabled() || samples <= 0) return;
    const auto now = monotonic_ns();
    increment_detail(AudioWindowHandoffs, 1, now);
    set_detail_integer(AudioWindowSamples, static_cast<std::uint64_t>(samples), now);
}

void Telemetry::record_ofdm_fft_execution(int symbols, double duration_ms) {
    if (!enabled() || symbols <= 0 || !std::isfinite(duration_ms)
        || duration_ms < 0.0) return;
    const auto now = monotonic_ns();
    increment_detail(OfdmSymbolDemodExecutions, static_cast<std::uint64_t>(symbols), now);
    set_detail_number(OfdmSymbolDemodDuration, duration_ms, now);
}

void Telemetry::record_channel_estimate(int model_cells, int pilots,
    double coherence, double selectivity, double noise, double mean_h,
    int estimator_kind, bool model_built) {
    if (!enabled() || model_cells < 0 || pilots < 0) return;
    const auto now = monotonic_ns();
    set_detail_integer(ChannelPilotObservations, static_cast<std::uint64_t>(pilots), now);
    set_detail_integer(ChannelModelCells, static_cast<std::uint64_t>(model_cells), now);
    set_detail_integer(ChannelModelPublished, model_built ? 1 : 0, now);
    if (estimator_kind >= 1 && estimator_kind <= 3)
        set_detail_integer(ChannelEstimatorKind, estimator_kind, now);
    else clear_detail(ChannelEstimatorKind);
    if (pilots > 0 && std::isfinite(coherence) && coherence >= 0.0 && coherence <= 1.0)
        set_detail_number(ChannelPilotCoherence, coherence, now);
    else clear_detail(ChannelPilotCoherence);
    if (pilots > 0 && std::isfinite(selectivity) && selectivity >= 0.0)
        set_detail_number(ChannelPilotSelectivity, selectivity, now);
    else clear_detail(ChannelPilotSelectivity);
    if (pilots > 0 && std::isfinite(noise) && noise >= 0.0)
        set_detail_number(ChannelNoiseVariance, noise, now);
    else clear_detail(ChannelNoiseVariance);
    if (model_cells > 0 && std::isfinite(mean_h) && mean_h >= 0.0)
        set_detail_number(ChannelMeanMagnitude, mean_h, now);
    else clear_detail(ChannelMeanMagnitude);
}

void Telemetry::record_demapper_output(int family, int order, int bits,
    double mean_abs_llr, double weak_fraction, bool published) {
    if (!enabled() || (family != 1 && family != 2) || order <= 0 || order > 4096
        || bits < 0) return;
    const auto now = monotonic_ns();
    set_detail_integer(DemapperModulation,
        static_cast<std::uint64_t>(family * 10000 + order), now);
    set_detail_integer(DemapperSoftBits, static_cast<std::uint64_t>(bits), now);
    set_detail_integer(DemapperSoftBitsPublished, published ? 1 : 0, now);
    if (bits > 0 && std::isfinite(mean_abs_llr) && mean_abs_llr >= 0.0)
        set_detail_number(DemapperMeanAbsLlr, mean_abs_llr, now);
    else clear_detail(DemapperMeanAbsLlr);
    if (bits > 0 && std::isfinite(weak_fraction) && weak_fraction >= 0.0
        && weak_fraction <= 1.0)
        set_detail_number(DemapperNearZeroFraction, weak_fraction, now);
    else clear_detail(DemapperNearZeroFraction);
}

void Telemetry::record_ldpc_decode_result(int iterations, int limit,
    bool converged, double duration_ms) {
    if (!enabled() || iterations < 0 || limit <= 0 || iterations > limit + 1
        || !std::isfinite(duration_ms) || duration_ms < 0.0) return;
    const auto now = monotonic_ns();
    set_detail_integer(LdpcConverged, converged ? 1 : 0, now);
    set_detail_number(LdpcDuration, duration_ms, now);
}

void Telemetry::record_correlator_detection(std::uint64_t ffts,
    std::uint64_t duration_ns, double best_metric, int matched,
    int expected, bool evaluated) {
    if (!enabled() || current_correlator_lane == 0) return;
    const int lane = current_correlator_lane - 1;
    const auto now = monotonic_ns();
    increment_detail(lane ? HailFftExecutions : AckFftExecutions, ffts, now);
    set_detail_number(lane ? HailDetectorDuration : AckDetectorDuration,
        duration_ns / 1000000.0, now);
    if (expected > 0) set_detail_integer(lane ? HailExpectedSymbols : AckExpectedSymbols,
        static_cast<std::uint64_t>(expected), now);
    if (evaluated && std::isfinite(best_metric) && best_metric >= 0.0
        && matched >= 0 && matched <= expected) {
        set_detail_number(lane ? HailBestMetric : AckBestMetric, best_metric, now);
        set_detail_integer(lane ? HailMatchedSymbols : AckMatchedSymbols,
            static_cast<std::uint64_t>(matched), now);
    } else {
        clear_detail(lane ? HailBestMetric : AckBestMetric);
        clear_detail(lane ? HailMatchedSymbols : AckMatchedSymbols);
    }
}

void Telemetry::record_arq_state(int role, int decision, int batch_seq,
    int retry_frames, int batch_frames, bool sack_enabled, int sack_width,
    int sack_acks, int timeout_ms) {
    if (!enabled()) return;
    if (decision >= 1 && decision <= 3) record_arq_decision(decision);
    // These are commander-owned cursors. On responder, suppress them instead
    // of presenting uninitialized or historical commander values as zeros.
    if (role != 0) {
        clear_detail(ArqRetryOccupancy);
        clear_detail(ArqNextBatchSequence);
        clear_detail(ArqBatchWidth);
        clear_detail(ArqRetryTimeout);
        return;
    }
    const auto now = monotonic_ns();
    if (retry_frames >= 0) set_detail_integer(ArqRetryOccupancy,
        static_cast<std::uint64_t>(retry_frames), now);
    else clear_detail(ArqRetryOccupancy);
    if (batch_seq >= 0 && batch_seq <= 255) set_detail_integer(ArqNextBatchSequence,
        static_cast<std::uint64_t>(batch_seq), now);
    else clear_detail(ArqNextBatchSequence);
    if (batch_frames > 0) set_detail_integer(ArqBatchWidth,
        static_cast<std::uint64_t>(batch_frames), now);
    else clear_detail(ArqBatchWidth);
    if (timeout_ms >= 0) set_detail_integer(ArqRetryTimeout,
        static_cast<std::uint64_t>(timeout_ms), now);
    else clear_detail(ArqRetryTimeout);
    // The exact CRC-valid SACK event owns width and bitmap together. A
    // periodic controller summary must not splice a different window into it.
    (void)sack_enabled;
    (void)sack_width;
    (void)sack_acks;
}

void Telemetry::record_arq_sack_window(int batch_seq, int nbits,
    const unsigned char* bitmap, int nbytes) {
    if (!enabled()) return;
    bool expected = false;
    if (!sack_writer_.compare_exchange_strong(expected, true,
            std::memory_order_acquire)) return;
    sack_generation_.fetch_add(1, std::memory_order_acq_rel);
    if (nbits <= 0 || bitmap == nullptr) {
        clear_detail(ArqSackMask);
        clear_detail(ArqSackBatchSequence);
        clear_detail(ArqSackWindowWidth);
        clear_detail(ArqSackAckCount);
    } else if (batch_seq >= 0 && batch_seq <= 255 && nbits <= 96
        && nbytes == (nbits + 7) / 8) {
        const auto now = monotonic_ns();
        int ack_count = 0;
        for (int i = 0; i < 12; ++i) {
            unsigned char value = i < nbytes ? bitmap[i] : 0;
            if (i == nbytes - 1 && (nbits & 7))
                value &= static_cast<unsigned char>((1u << (nbits & 7)) - 1u);
            sack_bytes_[i].store(value, std::memory_order_relaxed);
            for (int bit = 0; bit < 8; ++bit) ack_count += (value >> bit) & 1u;
        }
        set_detail_integer(ArqSackMask, 1, now);
        set_detail_integer(ArqSackBatchSequence, batch_seq, now);
        set_detail_integer(ArqSackWindowWidth, nbits, now);
        set_detail_integer(ArqSackAckCount, ack_count, now);
    }
    sack_generation_.fetch_add(1, std::memory_order_release);
    sack_writer_.store(false, std::memory_order_release);
}

void Telemetry::record_arq_decision(int decision) {
    if (!enabled()) return;
    if (decision == 0) { clear_detail(ArqDecision); return; }
    if (decision >= 1 && decision <= 3)
        set_detail_integer(ArqDecision, decision);
}

void Telemetry::record_gearshift_decision(double forward_snr, bool forward_valid,
    int forward_age, double reverse_snr, bool reverse_valid, int reverse_age,
    int action, int target_config, const char* reason) {
    if (!enabled()) return;
    std::lock_guard<std::mutex> guard(gear_decision_write_mutex_);
    const auto now = monotonic_ns();
    gear_decision_generation_.fetch_add(1, std::memory_order_acq_rel);
    if (forward_valid && std::isfinite(forward_snr)) {
        set_detail_number(GearForwardSnr, forward_snr, now);
        if (forward_age >= 0) set_detail_integer(GearForwardSnrAge, forward_age, now);
        else clear_detail(GearForwardSnrAge);
    } else { clear_detail(GearForwardSnr); clear_detail(GearForwardSnrAge); }
    if (reverse_valid && std::isfinite(reverse_snr)) {
        set_detail_number(GearReverseSnr, reverse_snr, now);
        if (reverse_age >= 0) set_detail_integer(GearReverseSnrAge, reverse_age, now);
        else clear_detail(GearReverseSnrAge);
    } else { clear_detail(GearReverseSnr); clear_detail(GearReverseSnrAge); }
    if (action >= 0 && action <= 4) set_detail_integer(GearDecisionAction, action, now);
    else clear_detail(GearDecisionAction);
    if (target_config >= -1 && target_config <= 105)
        set_detail_integer(GearDecisionTarget,
            static_cast<std::uint64_t>(target_config + 1), now);
    else clear_detail(GearDecisionTarget);
    bool valid_reason = reason != nullptr;
    std::size_t len = 0;
    if (valid_reason) {
        while (len < 63 && reason[len] != '\0') {
            const char ch = reason[len];
            if (!((ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9') || ch == '-'))
                valid_reason = false;
            ++len;
        }
        if (len == 0 || reason[len] != '\0') valid_reason = false;
    }
    if (valid_reason) {
        for (std::size_t i = 0; i < 64; ++i)
            gear_reason_[i].store(i < len ? reason[i] : '\0', std::memory_order_relaxed);
        set_detail_integer(GearDecisionReason, 1, now);
    } else clear_detail(GearDecisionReason);
    gear_decision_generation_.fetch_add(1, std::memory_order_release);
}

void Telemetry::record_acquisition_transition(int event, int delay_samples,
    double metric, int config) {
    if (!enabled() || event < 1 || event > 4) return;
    const auto now = monotonic_ns();
    set_detail_integer(AcquisitionTransition, event, now);
    set_detail_number(AcquisitionTransitionDelay, delay_samples, now);
    if (config >= -1 && config <= 105)
        set_detail_integer(AcquisitionTransitionConfig, config + 1, now);
    else clear_detail(AcquisitionTransitionConfig);
    (void)metric;  // v1 coarse metric retains its separate, qualified source.
}

void Telemetry::record_carrier_bin_handoff(std::uint64_t data_mask,
    std::uint64_t pilot_mask, int width, int data_cells, int pilot_cells, int config) {
    if (!enabled()) return;
    std::lock_guard<std::mutex> guard(carrier_write_mutex_);
    carrier_generation_.fetch_add(1, std::memory_order_acq_rel);
    if (width <= 0 || width > 64 || data_cells < 0 || pilot_cells < 0) {
        for (int i = CarrierDataMask; i <= CarrierPilotCells; ++i) clear_detail(i);
        carrier_generation_.fetch_add(1, std::memory_order_release);
        return;
    }
    const std::uint64_t valid_mask = width == 64 ? ~std::uint64_t{0}
        : ((std::uint64_t{1} << width) - 1u);
    if ((data_mask & ~valid_mask) != 0 || (pilot_mask & ~valid_mask) != 0) {
        for (int i = CarrierDataMask; i <= CarrierPilotCells; ++i) clear_detail(i);
        carrier_generation_.fetch_add(1, std::memory_order_release);
        return;
    }
    const auto now = monotonic_ns();
    carrier_data_mask_.store(data_mask, std::memory_order_relaxed);
    carrier_pilot_mask_.store(pilot_mask, std::memory_order_relaxed);
    set_detail_integer(CarrierDataMask, 1, now);
    set_detail_integer(CarrierPilotMask, 1, now);
    set_detail_integer(CarrierWidth, width, now);
    set_detail_integer(CarrierDataCells, data_cells, now);
    set_detail_integer(CarrierPilotCells, pilot_cells, now);
    (void)config;
    carrier_generation_.fetch_add(1, std::memory_order_release);
}

void Telemetry::record_ldpc_codeword_handoff(bool converged, int bits, int iterations) {
    if (!enabled() || !converged || bits <= 0 || iterations < 0) return;
    const auto now = monotonic_ns();
    increment_detail(LdpcCodewordHandoffs, 1, now);
    set_detail_integer(LdpcCodewordBits, bits, now);
    set_detail_integer(LdpcCodewordIterations, iterations, now);
}

void Telemetry::record_crc_frame_outcome(bool accepted, int reason) {
    if (!enabled() || reason < 0 || reason > 3
        || (accepted && reason != 0) || (!accepted && reason == 0)) return;
    const auto now = monotonic_ns();
    const auto seq = crc_outcome_sequence_.fetch_add(1, std::memory_order_acq_rel) + 1;
    const auto slot = (seq - 1) % 32;
    crc_recent_sample_ns_[slot].store(now, std::memory_order_relaxed);
    crc_recent_outcomes_[slot].store((seq << 3) | (static_cast<std::uint64_t>(reason) << 1)
        | (accepted ? 1u : 0u), std::memory_order_release);
    set_detail_integer(CrcFrameOutcome, accepted ? 1 : 2, now);
    set_detail_integer(CrcFrameRejectReason, accepted ? 0 : reason, now);
    set_detail_integer(CrcRecentWindow, 32, now);
}

void Telemetry::record_correlator_outcome(int lane, bool accepted,
    double metric_threshold, int matched_threshold) {
    if (!enabled() || (lane != 1 && lane != 2)) return;
    const auto now = monotonic_ns();
    set_detail_integer(lane == 1 ? AckPatternOutcome : HailPatternOutcome,
        accepted ? 1 : 2, now);
    if (std::isfinite(metric_threshold) && metric_threshold >= 0.0)
        set_detail_number(lane == 1 ? AckMetricThreshold : HailMetricThreshold,
            metric_threshold, now);
    else clear_detail(lane == 1 ? AckMetricThreshold : HailMetricThreshold);
    if (matched_threshold >= 0)
        set_detail_integer(lane == 1 ? AckMatchedThreshold : HailMatchedThreshold,
            matched_threshold, now);
    else clear_detail(lane == 1 ? AckMatchedThreshold : HailMatchedThreshold);
}

void Telemetry::record_gearshift_local_config(int from, int to, int role) {
    if (!enabled() || from < -1 || from > 105 || to < -1 || to > 105) return;
    const auto now = monotonic_ns();
    set_detail_integer(GearLocalFrom, from + 1, now);
    set_detail_integer(GearLocalTo, to + 1, now);
    (void)role;
}

void Telemetry::record_gearshift_engagement(int from, int to, int action,
    int phase, const char* selection_reason, const char* confirmation_reason) {
    if (!enabled() || from < -1 || from > 105 || to < -1 || to > 105
        || action < -1 || action > 4 || phase < 1 || phase > 3) return;
    std::lock_guard<std::mutex> guard(engagement_write_mutex_);
    const auto now = monotonic_ns();
    engagement_generation_.fetch_add(1, std::memory_order_acq_rel);
    set_detail_integer(GearEngagementFrom, from + 1, now);
    set_detail_integer(GearEngagementTo, to + 1, now);
    set_detail_integer(GearEngagementAction, action + 1, now);
    set_detail_integer(GearEngagementPhase, phase, now);
    auto write_reason = [&](const char* source, std::atomic<char>* target,
                            DetailIndex index) {
        if (source == nullptr) { clear_detail(index); return; }
        std::size_t len = 0;
        bool valid = true;
        while (len < 63 && source[len] != '\0') {
            const char ch = source[len];
            if (!((ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9') || ch == '-'))
                valid = false;
            ++len;
        }
        if (len == 0 || source[len] != '\0' || !valid) {
            clear_detail(index); return;
        }
        for (std::size_t i = 0; i < 64; ++i)
            target[i].store(i < len ? source[i] : '\0', std::memory_order_relaxed);
        set_detail_integer(index, 1, now);
    };
    write_reason(selection_reason, engagement_selection_reason_,
                 GearEngagementSelectionReason);
    write_reason(confirmation_reason, engagement_confirmation_reason_,
                 GearEngagementConfirmationReason);
    engagement_generation_.fetch_add(1, std::memory_order_release);
}

void Telemetry::record_gearshift_probe_state(bool active) {
    if (enabled()) set_detail_integer(GearProbeActive, active ? 1 : 0);
}

void Telemetry::record_gearshift_session_reset() {
    if (!enabled()) return;
    {
        std::lock_guard<std::mutex> guard(gear_decision_write_mutex_);
        gear_decision_generation_.fetch_add(1, std::memory_order_acq_rel);
        for (int i = GearForwardSnr; i <= GearDecisionReason; ++i) clear_detail(i);
        gear_decision_generation_.fetch_add(1, std::memory_order_release);
    }
    {
        std::lock_guard<std::mutex> guard(engagement_write_mutex_);
        engagement_generation_.fetch_add(1, std::memory_order_acq_rel);
        for (int i = GearEngagementPhase; i <= GearProbeActive; ++i) clear_detail(i);
        engagement_generation_.fetch_add(1, std::memory_order_release);
    }
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
        bool coherent = false;
        for (int attempt = 0; attempt < 3; ++attempt) {
            if (correlator_writers_[lane].load(std::memory_order_acquire) != 0) continue;
            const auto before = correlator_generation_[lane].load(std::memory_order_acquire);
            correlator_windows[lane] = correlator_windows_[lane].load(std::memory_order_relaxed);
            correlator_calls[lane] = correlator_invocations_[lane].load(std::memory_order_relaxed);
            correlator_memo[lane] = correlator_memo_enabled_[lane].load(std::memory_order_relaxed);
            const auto after = correlator_generation_[lane].load(std::memory_order_acquire);
            if (before == after
                && correlator_writers_[lane].load(std::memory_order_acquire) == 0) {
                coherent = true;
                break;
            }
        }
        correlator_reuses[lane] = correlator_reuses_[lane].load(std::memory_order_relaxed);
        correlator_ready[lane] = coherent
            && correlator_observed_[lane].load(std::memory_order_acquire) != 0
            && correlator_calls[lane] <= 9007199254740991ULL
            && correlator_windows[lane] <= correlator_calls[lane]
            && correlator_reuses[lane] <= 9007199254740991ULL;
        if (correlator_ready[lane]) {
            const auto previous = correlator_snapshot_invocations_[lane].exchange(
                correlator_calls[lane], std::memory_order_relaxed);
            correlator_delta[lane] = correlator_calls[lane] >= previous
                ? correlator_calls[lane] - previous : 0;
        }
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

std::string Telemetry::snapshot_json_v2() {
    std::string packet = snapshot_json();
    const auto now = monotonic_ns();
    const std::string version_one = "\"version\":1";
    const auto version_at = packet.find(version_one);
    if (version_at != std::string::npos)
        packet.replace(version_at, version_one.size(), "\"version\":2");
    const std::string clock_key = "\"monotonic_ns\":\"";
    const auto clock_at = packet.find(clock_key);
    if (clock_at != std::string::npos) {
        const auto first = clock_at + clock_key.size();
        const auto end = packet.find('"', first);
        if (end != std::string::npos)
            packet.replace(first, end - first, std::to_string(now));
    }

    struct ReadCell { double number; std::uint64_t integer, sample_ns; };
    ReadCell values[kDetailCount]{};
    for (std::size_t i = 0; i < kDetailCount; ++i) {
        values[i].number = detail_[i].number.load(std::memory_order_relaxed);
        values[i].integer = detail_[i].integer.load(std::memory_order_relaxed);
        values[i].sample_ns = detail_[i].sample_ns.load(std::memory_order_acquire);
    }
    // Four cursor values represent one mutex-coherent source snapshot. A
    // collided sender read drops all four, never a partly newer head/tail.
    bool cursor_coherent = false;
    for (int attempt = 0; attempt < 3; ++attempt) {
        const auto before = ring_cursor_generation_.load(std::memory_order_acquire);
        if (before & 1u) continue;
        for (int i = AudioRingWriteIndex; i <= AudioRingFull; ++i) {
            values[i].integer = detail_[i].integer.load(std::memory_order_relaxed);
            values[i].sample_ns = detail_[i].sample_ns.load(std::memory_order_acquire);
        }
        if (ring_cursor_generation_.load(std::memory_order_acquire) == before) {
            cursor_coherent = true;
            break;
        }
    }
    if (!cursor_coherent || (values[AudioInputAvailable].sample_ns != 0
        && values[AudioInputAvailable].integer == 0)
        || values[AudioRingCapacity].integer == 0
        || values[AudioRingWriteIndex].integer >= values[AudioRingCapacity].integer
        || values[AudioRingReadIndex].integer >= values[AudioRingCapacity].integer) {
        for (int i = AudioRingWriteIndex; i <= AudioRingFull; ++i)
            values[i].sample_ns = 0;
    }
    std::uint64_t carrier_data_mask = 0, carrier_pilot_mask = 0;
    bool carrier_coherent = false;
    for (int attempt = 0; attempt < 3; ++attempt) {
        const auto before = carrier_generation_.load(std::memory_order_acquire);
        if (before & 1u) continue;
        carrier_data_mask = carrier_data_mask_.load(std::memory_order_relaxed);
        carrier_pilot_mask = carrier_pilot_mask_.load(std::memory_order_relaxed);
        for (int i = CarrierDataMask; i <= CarrierPilotCells; ++i) {
            values[i].integer = detail_[i].integer.load(std::memory_order_relaxed);
            values[i].sample_ns = detail_[i].sample_ns.load(std::memory_order_acquire);
        }
        if (carrier_generation_.load(std::memory_order_acquire) == before) {
            carrier_coherent = true;
            break;
        }
    }
    if (!carrier_coherent || values[CarrierWidth].integer == 0
        || values[CarrierWidth].integer > 64
        || (values[CarrierWidth].integer < 64
            && ((carrier_data_mask | carrier_pilot_mask)
                >> values[CarrierWidth].integer) != 0)) {
        for (int i = CarrierDataMask; i <= CarrierPilotCells; ++i)
            values[i].sample_ns = 0;
    }
    unsigned char sack[12]{};
    bool sack_coherent = false;
    for (int attempt = 0; attempt < 3; ++attempt) {
        const auto before = sack_generation_.load(std::memory_order_acquire);
        if (before & 1u) continue;
        for (int i = 0; i < 12; ++i)
            sack[i] = sack_bytes_[i].load(std::memory_order_relaxed);
        for (int i = ArqSackWindowWidth; i <= ArqSackBatchSequence; ++i) {
            values[i].integer = detail_[i].integer.load(std::memory_order_relaxed);
            values[i].sample_ns = detail_[i].sample_ns.load(std::memory_order_acquire);
        }
        if (sack_generation_.load(std::memory_order_acquire) == before) {
            sack_coherent = true;
            break;
        }
    }
    if (!sack_coherent) {
        for (int i = ArqSackWindowWidth; i <= ArqSackBatchSequence; ++i)
            values[i].sample_ns = 0;
    }
    char gear_reason[64]{}, selection_reason[64]{}, confirmation_reason[64]{};
    auto copy_reason = [](std::atomic<char>* source, char* target) {
        for (int i = 0; i < 64; ++i) target[i] = source[i].load(std::memory_order_relaxed);
        target[63] = '\0';
    };
    bool decision_coherent = false;
    for (int attempt = 0; attempt < 3; ++attempt) {
        const auto before = gear_decision_generation_.load(std::memory_order_acquire);
        if (before & 1u) continue;
        copy_reason(gear_reason_, gear_reason);
        for (int i = GearForwardSnr; i <= GearDecisionReason; ++i) {
            values[i].number = detail_[i].number.load(std::memory_order_relaxed);
            values[i].integer = detail_[i].integer.load(std::memory_order_relaxed);
            values[i].sample_ns = detail_[i].sample_ns.load(std::memory_order_acquire);
        }
        if (gear_decision_generation_.load(std::memory_order_acquire) == before) {
            decision_coherent = true;
            break;
        }
    }
    if (!decision_coherent)
        for (int i = GearForwardSnr; i <= GearDecisionReason; ++i)
            values[i].sample_ns = 0;
    bool engagement_coherent = false;
    for (int attempt = 0; attempt < 3; ++attempt) {
        const auto before = engagement_generation_.load(std::memory_order_acquire);
        if (before & 1u) continue;
        copy_reason(engagement_selection_reason_, selection_reason);
        copy_reason(engagement_confirmation_reason_, confirmation_reason);
        for (int i = GearEngagementPhase; i <= GearEngagementConfirmationReason; ++i) {
            values[i].integer = detail_[i].integer.load(std::memory_order_relaxed);
            values[i].sample_ns = detail_[i].sample_ns.load(std::memory_order_acquire);
        }
        if (engagement_generation_.load(std::memory_order_acquire) == before) {
            engagement_coherent = true;
            break;
        }
    }
    if (!engagement_coherent)
        for (int i = GearEngagementPhase; i <= GearEngagementConfirmationReason; ++i)
            values[i].sample_ns = 0;

    // A ring slot is a single atomic event (sequence, reason, accepted). The
    // latest display event is a sample, while the last-32 counts preserve
    // outcomes that happen between eight-Hz publisher frames.
    const auto crc_seq = crc_outcome_sequence_.load(std::memory_order_acquire);
    std::uint64_t crc_checked = 0, crc_rejected = 0, crc_latest = 0;
    int crc_latest_reason = 0;
    bool crc_latest_accepted = false;
    std::uint64_t crc_latest_sample_ns = 0;
    for (int slot = 0; slot < 32; ++slot) {
        const auto word = crc_recent_outcomes_[slot].load(std::memory_order_acquire);
        const auto sequence = word >> 3;
        if (sequence == 0 || sequence > crc_seq || crc_seq - sequence >= 32) continue;
        ++crc_checked;
        if ((word & 1u) == 0) ++crc_rejected;
        if (sequence > crc_latest) {
            crc_latest = sequence;
            crc_latest_accepted = (word & 1u) != 0;
            crc_latest_reason = static_cast<int>((word >> 1) & 3u);
            crc_latest_sample_ns = crc_recent_sample_ns_[slot].load(std::memory_order_relaxed);
        }
    }
    if (crc_latest != 0) {
        values[CrcFrameOutcome].integer = crc_latest_accepted ? 1 : 2;
        values[CrcFrameRejectReason].integer = crc_latest_reason;
        values[CrcFrameOutcome].sample_ns = crc_latest_sample_ns;
        values[CrcFrameRejectReason].sample_ns = crc_latest_sample_ns;
        values[CrcRecentChecked].integer = crc_checked;
        values[CrcRecentRejected].integer = crc_rejected;
        values[CrcRecentWindow].integer = 32;
        for (int i = CrcRecentChecked; i <= CrcRecentWindow; ++i)
            values[i].sample_ns = crc_latest_sample_ns;
    }

    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(17);
    for (std::size_t i = 0; i < kDetailCount; ++i) {
        const auto& spec = kDetailMetrics[i];
        const auto& cell = values[i];
        if (i != 0) out << ',';
        out << '"' << spec.name << "\":{\"available\":";
        const bool fresh = cell.sample_ns != 0 && now >= cell.sample_ns
            && (spec.persistent || now - cell.sample_ns <= kMetricFreshnessNs);
        if (!fresh) {
            out << "false,\"quality\":\"unavailable\",\"type\":\""
                << spec.type << "\",\"unit\":\"" << spec.unit
                << "\",\"reason\":\"inactive\"}";
            continue;
        }
        out << "true,\"quality\":\"" << spec.quality << "\",\"type\":\""
            << spec.type << "\",\"unit\":\"" << spec.unit
            << "\",\"age_ns\":\"" << now - cell.sample_ns << "\",\"value\":";
        if (i == ChannelEstimatorKind) {
            const char* names[] = {"UNKNOWN", "ZF", "LS", "LS_TIME_INTERP"};
            out << '"' << names[cell.integer <= 3 ? cell.integer : 0] << '"';
        } else if (i == DemapperModulation) {
            const auto family = cell.integer / 10000;
            const auto order = cell.integer % 10000;
            out << '"' << (family == 2 ? "MFSK_" : "PSK_QAM_") << order << '"';
        } else if (i == ArqDecision) {
            const char* names[] = {"UNKNOWN", "ACCEPT", "RETRY", "HOLD"};
            out << '"' << names[cell.integer <= 3 ? cell.integer : 0] << '"';
        } else if (i == AcquisitionTransition) {
            const char* names[] = {"UNKNOWN", "CANDIDATE_ADMITTED",
                "TIMING_HANDED_TO_OFDM", "FRAME_ACCEPTED", "ATTEMPT_REJECTED"};
            out << '"' << names[cell.integer <= 4 ? cell.integer : 0] << '"';
        } else if (i == AcquisitionTransitionConfig || i == GearDecisionTarget
            || i == GearLocalFrom || i == GearLocalTo || i == GearEngagementFrom
            || i == GearEngagementTo) {
            out << '"' << config_state(static_cast<int>(cell.integer) - 1) << '"';
        } else if (i == GearDecisionAction || i == GearEngagementAction) {
            const char* names[] = {"HOLD", "SWITCH", "PROBE", "ROLLBACK", "ABSTAIN"};
            const int action = i == GearEngagementAction
                ? static_cast<int>(cell.integer) - 1 : static_cast<int>(cell.integer);
            out << '"' << (action >= 0 && action <= 4 ? names[action] : "UNKNOWN") << '"';
        } else if (i == GearEngagementPhase) {
            const char* names[] = {"UNKNOWN", "CONFIRMED_PROBE",
                "CONFIRMED_STABLE", "PROBE_ACCEPTED"};
            out << '"' << names[cell.integer <= 3 ? cell.integer : 0] << '"';
        } else if (i == GearDecisionReason) out << '"' << gear_reason << '"';
        else if (i == GearEngagementSelectionReason) out << '"' << selection_reason << '"';
        else if (i == GearEngagementConfirmationReason) out << '"' << confirmation_reason << '"';
        else if (i == CrcFrameOutcome || i == AckPatternOutcome || i == HailPatternOutcome)
            out << '"' << (cell.integer == 1 ? "ACCEPT" : "REJECT") << '"';
        else if (i == CrcFrameRejectReason) {
            const char* names[] = {"NONE", "CRC_RESIDUAL", "LDPC_NONCONVERGED",
                "OTHER_REJECT"};
            out << '"' << names[cell.integer <= 3 ? cell.integer : 3] << '"';
        } else if (i == CarrierDataMask || i == CarrierPilotMask) {
            const auto mask = i == CarrierDataMask
                ? carrier_data_mask : carrier_pilot_mask;
            out << "\"0x" << std::uppercase << std::hex << std::setw(16)
                << std::setfill('0') << mask << std::dec << std::nouppercase
                << std::setfill(' ') << '"';
        } else if (i == ArqSackMask) {
            out << "\"0x" << std::uppercase << std::hex << std::setfill('0');
            for (int byte = 0; byte < 12; ++byte) out << std::setw(2)
                << static_cast<unsigned int>(sack[byte]);
            out << std::dec << std::nouppercase << std::setfill(' ') << '"';
        } else if (std::string(spec.type) == "flag")
            out << (cell.integer ? "true" : "false");
        else if (std::string(spec.type) == "counter")
            out << '"' << cell.integer << '"';
        else if (i == AudioInterarrivalMs || i == OfdmSymbolDemodDuration
            || i == ChannelPilotCoherence || i == ChannelPilotSelectivity
            || i == ChannelNoiseVariance || i == ChannelMeanMagnitude
            || i == DemapperMeanAbsLlr || i == DemapperNearZeroFraction
            || i == LdpcDuration || i == AckDetectorDuration
            || i == AckBestMetric || i == HailDetectorDuration
            || i == HailBestMetric || i == GearForwardSnr || i == GearReverseSnr
            || i == AcquisitionTransitionDelay || i == AckMetricThreshold
            || i == HailMetricThreshold)
            out << cell.number;
        else out << cell.integer;
        out << '}';
    }
    if (!packet.empty() && packet.back() == '}') packet.pop_back();
    packet += ",\"detail_metrics\":{" + out.str() + "}}";
    return packet;
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
        const std::string payload = packet_version_.load(std::memory_order_acquire) == 2
            ? snapshot_json_v2() : snapshot_json();
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

extern "C" int mercury_rro_telemetry_enabled(void) {
    return rro::Telemetry::instance().enabled() ? 1 : 0;
}

extern "C" void mercury_rro_audio_capture_arrived(std::uint64_t samples,
                                                     std::uint64_t sample_ns) {
    rro::Telemetry::instance().record_audio_capture_arrived(samples, sample_ns);
}

extern "C" void mercury_rro_audio_capture_written(std::uint64_t samples,
                                                     std::uint64_t sample_ns) {
    rro::Telemetry::instance().record_audio_capture_written(samples, sample_ns);
}

extern "C" void mercury_rro_audio_capture_read(std::uint64_t samples,
                                                  std::uint64_t sample_ns) {
    rro::Telemetry::instance().record_audio_capture_read(samples, sample_ns);
}

extern "C" void mercury_rro_audio_capture_reset(std::uint64_t sample_ns) {
    rro::Telemetry::instance().record_audio_capture_reset(sample_ns);
}

extern "C" void mercury_rro_audio_input_state(int available, int simulated,
                                                std::uint64_t sample_ns) {
    rro::Telemetry::instance().record_audio_input_state(available != 0,
        simulated != 0, sample_ns);
}

extern "C" void mercury_rro_audio_capture_cursor(std::uint64_t head_samples,
    std::uint64_t tail_samples, std::uint64_t capacity_samples, int full,
    std::uint64_t sample_ns) {
    rro::Telemetry::instance().record_audio_capture_cursor(head_samples,
        tail_samples, capacity_samples, full != 0, sample_ns);
}
