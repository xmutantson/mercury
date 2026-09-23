#include "common/rro_telemetry.h"

#include <cassert>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#if defined(_WIN32)
# include <fcntl.h>
# include <io.h>
#endif

int main() {
#if defined(_WIN32)
    _setmode(_fileno(stdout), _O_BINARY);
    _putenv_s("MERCURY_RRO_TELEMETRY", "1");
#else
    setenv("MERCURY_RRO_TELEMETRY", "1", 1);
#endif
    rro::Telemetry& telemetry = rro::Telemetry::instance();
    telemetry.start_from_environment();
    assert(telemetry.enabled());
    const std::string before = telemetry.snapshot_json();
    assert(before.find("\"audio.capture_buffered_samples\":{\"available\":false")
           != std::string::npos);
    telemetry.record_processing_load(0.375);
    telemetry.record_capture_ring(128, 512);
    telemetry.record_receive_configuration(1024, 80);
    telemetry.record_ofdm_lattice(30, 10, 20, 105);
    telemetry.record_ofdm_candidate(true);
    telemetry.record_acquisition_timing_residual(-12);
    telemetry.record_acquisition_frequency_offset(3.25);
    telemetry.record_acquisition_coarse_metric(0.75);
    telemetry.record_ldpc_iterations(12);
    telemetry.record_crc_result(false);
    telemetry.record_gearshift({2, 2, 0, 16, 9, 15, 16, 2, 3, 2,
        true, 1400, true, 9, false, 2});
    const std::string after = telemetry.snapshot_json();
    assert(after.find("\"audio.capture_buffered_samples\":{\"available\":true")
           != std::string::npos);
    assert(after.find("\"decode.ldpc_iterations\":{\"available\":true")
           != std::string::npos);
    assert(after.find("\"decode.crc_ok\":{\"available\":true")
           != std::string::npos);
    assert(after.find("\"decode.frames_total\":{\"available\":true,\"quality\":\"measured\",\"type\":\"counter\",\"unit\":\"frame\",\"value\":\"1\"")
           != std::string::npos);
    assert(after.find("\"decode.frames_failed\":{\"available\":true,\"quality\":\"measured\",\"type\":\"counter\",\"unit\":\"frame\",\"value\":\"1\"")
           != std::string::npos);
    assert(after.find("\"gearshift.lifecycle\":{\"available\":true")
           != std::string::npos);
    assert(after.find("\"gearshift.last_batch_classification\":{\"available\":true,\"quality\":\"measured\",\"type\":\"state\",\"unit\":\"state\",\"value\":\"PARTIAL\"")
           != std::string::npos);
    assert(after.find("\"gearshift.partial_streak\":{\"available\":true,\"quality\":\"measured\",\"type\":\"gauge\",\"unit\":\"batch\",\"value\":2")
           != std::string::npos);
    assert(after.find("\"gearshift.target_config\":{\"available\":true,\"quality\":\"configured\",\"type\":\"state\",\"unit\":\"config\",\"value\":\"CONFIG_9\"")
           != std::string::npos);
    assert(after.find("\"gearshift.optimizer_target_config\":{\"available\":true,\"quality\":\"configured\",\"type\":\"state\",\"unit\":\"config\",\"value\":\"CONFIG_9\"")
           != std::string::npos);
    assert(after.find("\"gearshift.break_count_total\":{\"available\":true")
           != std::string::npos);
    assert(after.find("\"acquisition.active\":{\"available\":true")
           != std::string::npos);
    assert(after.find("\"acquisition.timing_offset_samples\":{\"available\":true,\"quality\":\"measured\",\"type\":\"gauge\",\"unit\":\"sample\",\"value\":-12")
           != std::string::npos);
    assert(after.find("\"acquisition.frequency_offset_hz\":{\"available\":true,\"quality\":\"measured\",\"type\":\"gauge\",\"unit\":\"Hz\",\"value\":3.25")
           != std::string::npos);
    assert(after.find("\"acquisition.coarse_metric\":{\"available\":true,\"quality\":\"measured\",\"type\":\"gauge\",\"unit\":\"1\",\"value\":0.75")
           != std::string::npos);
    assert(after.find("\"ofdm.fft_size\":{\"available\":true")
           != std::string::npos);
    assert(after.find("\"ofdm.active_carriers\":{\"available\":true,\"quality\":\"configured\",\"type\":\"gauge\",\"unit\":\"carrier\",\"value\":30")
           != std::string::npos);
    assert(after.find("\"ofdm.current_config\":{\"available\":true,\"quality\":\"configured\",\"type\":\"state\",\"unit\":\"config\",\"value\":\"LOW48_ANCHOR_S20_R6\"")
           != std::string::npos);
    assert(after.find("\"decode.ldpc_max_iterations\":{\"available\":true")
           != std::string::npos);
    std::cout << before << '\n' << after << '\n';
    // The sender must never splice one controller update's configuration onto
    // another update's streak, even while source and snapshot race.
    std::atomic<bool> writing{true};
    std::thread writer([&] {
        for (int i = 0; i < 10000; ++i) {
            const bool first = (i & 1) == 0;
            telemetry.record_gearshift({2, 2, 0, first ? 16 : 5, first ? 16 : 5, 15, 16,
                first ? 1 : 2, first ? 3 : 8, first ? 0ULL : 2ULL,
                true, 1400, true, first ? 16 : 5, false, 2});
            std::this_thread::yield();
        }
        writing.store(false);
    });
    do {
        const std::string sample = telemetry.snapshot_json();
        const auto config_at = sample.find("\"gearshift.current_config\":");
        const auto streak_at = sample.find("\"gearshift.clean_streak\":");
        assert(config_at != std::string::npos && streak_at != std::string::npos);
        const auto config = sample.substr(config_at, sample.find('}', config_at) - config_at);
        const auto streak = sample.substr(streak_at, sample.find('}', streak_at) - streak_at);
        if (config.find("\"available\":true") != std::string::npos) {
            const bool first = config.find("\"value\":\"CONFIG_16\"") != std::string::npos;
            const bool second = config.find("\"value\":\"CONFIG_5\"") != std::string::npos;
            assert(first || second);
            assert(streak.find(first ? "\"value\":3" : "\"value\":8")
                   != std::string::npos);
        }
    } while (writing.load());
    writer.join();
    std::this_thread::sleep_for(std::chrono::milliseconds(2100));
    const std::string stale = telemetry.snapshot_json();
    assert(stale.find("\"audio.capture_buffered_samples\":{\"available\":false")
           != std::string::npos);
    assert(stale.find("\"audio.processing_load_ratio\":{\"available\":false")
           != std::string::npos);
    assert(stale.find("\"ofdm.fft_size\":{\"available\":false")
           != std::string::npos);
    assert(stale.find("\"ofdm.active_carriers\":{\"available\":false")
           != std::string::npos);
    assert(stale.find("\"decode.frames_total\":{\"available\":true")
           != std::string::npos);
    std::cout << stale << '\n';
    telemetry.stop();
}
