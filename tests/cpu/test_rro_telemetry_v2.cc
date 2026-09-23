#include "common/rro_telemetry.h"

#include <cassert>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

int main() {
#if defined(_WIN32)
    _putenv_s("MERCURY_RRO_TELEMETRY", "1");
    _putenv_s("MERCURY_RRO_TELEMETRY_VERSION", "2");
    _putenv_s("MERCURY_RRO_UDP_PORT", "65431");
#else
    setenv("MERCURY_RRO_TELEMETRY", "1", 1);
    setenv("MERCURY_RRO_TELEMETRY_VERSION", "2", 1);
    setenv("MERCURY_RRO_UDP_PORT", "65431", 1);
#endif
    auto& t = rro::Telemetry::instance();
    t.start_from_environment();
    assert(t.enabled());
    t.record_processing_load(0.25);
    t.record_capture_ring(128, 512);
    t.record_receive_configuration(1024, 80);
    t.record_ofdm_lattice(30, 10, 20, 3);
    t.record_ofdm_candidate(true);
    t.record_acquisition_timing_residual(-2);
    t.record_acquisition_frequency_offset(1.5);
    t.record_acquisition_coarse_metric(0.75);
    t.record_ldpc_iterations(5);
    t.record_crc_result(true);
    t.record_gearshift({2, 2, 0, 3, 4, 3, 8, 1, 2, 0, false,
        0, true, 4, false, 0});
    {
        rro::CorrelatorWindow ack(1);
        t.record_correlator_invocation(true);
        t.record_correlator_detection(3, 500000, 2.75, 2, 3, true);
    }
    {
        rro::CorrelatorWindow hail(2);
        t.record_correlator_invocation(true);
        t.record_correlator_detection(2, 400000, 1.5, 1, 2, true);
    }
    t.record_audio_input_state(true, false, 0);
    t.record_audio_capture_arrived(256, 0);
    t.record_audio_capture_arrived(256, 0);
    t.record_audio_capture_written(512, 0);
    t.record_audio_capture_read(256, 0);
    t.record_audio_capture_reset(0);
    t.record_audio_capture_cursor(512, 256, 2048, false, 0);
    t.record_capture_window_handoff(1024);
    t.record_ofdm_fft_execution(4, 0.3);
    t.record_channel_estimate(80, 16, 0.85, 0.2, 0.01, 0.9, 2, true);
    t.record_demapper_output(1, 4, 160, 3.1, 0.15, true);
    t.record_ldpc_decode_result(5, 80, true, 1.3);
    t.record_acquisition_transition(2, -2, 0.75, 3);
    t.record_carrier_bin_handoff(0x3ff, 0x55, 16, 120, 24, 3);
    t.record_ldpc_codeword_handoff(true, 240, 5);
    t.record_correlator_outcome(1, true, 1.0, 2);
    t.record_correlator_outcome(2, false, 0.8, 2);
    t.record_arq_state(0, 0, 7, 2, 8, true, -1, -1, 700);
    const unsigned char sack[2] = {0x55, 0x03};
    t.record_arq_sack_window(6, 10, sack, 2);
    t.record_arq_decision(2);
    t.record_gearshift_decision(12.5, true, 1, 9.0, true, 2, 2, 4,
                                "lower-information-probe");
    t.record_gearshift_local_config(3, 4, 0);
    t.record_gearshift_engagement(3, 4, 2, 1, "lower-information-probe",
                                   "inband-sack-peer-confirmed");
    t.record_gearshift_probe_state(true);
    // Longest legal source slugs and fixed-width masks exercise UDP ceiling.
    const std::string longest_reason(63, 'a');
    t.record_gearshift_decision(12.5, true, 1, 9.0, true, 2, 2, 4,
                                longest_reason.c_str());
    t.record_gearshift_engagement(3, 4, 2, 1, longest_reason.c_str(),
                                   longest_reason.c_str());
    // Two checked frames between 8 Hz packets: latest event must be REJECT,
    // while the bounded window must retain both opposite outcomes.
    t.record_crc_frame_outcome(true, 0);
    t.record_crc_frame_outcome(false, 1);
    const std::string json = t.snapshot_json_v2();
    assert(json.find("\"version\":2") != std::string::npos);
    assert(json.find("\"detail_metrics\":{") != std::string::npos);
    assert(json.find("\"crc.recent_checked_frames\":{\"available\":true")
        != std::string::npos);
    assert(json.find("\"crc.recent_rejected_frames\":{\"available\":true")
        != std::string::npos);
    assert(json.find("\"audio.capture_ring_capacity_samples\":{\"available\":true")
        != std::string::npos);
    assert(json.size() < 32768);
    std::cerr << "v2 packet bytes=" << json.size() << '\n';
    if (const char* packet_path = std::getenv("MERCURY_RRO_TEST_PACKET_PATH")) {
        std::ofstream packet_file(packet_path, std::ios::binary);
        assert(packet_file.good());
        packet_file << json << '\n';
        assert(packet_file.good());
    }
    std::cout << json << '\n';
    t.clear_ofdm_lattice();  // Mode/config change must not preserve old OFDM cues.
    const std::string switched = t.snapshot_json_v2();
    assert(switched.find("\"ofdm.carrier_data_mask\":{\"available\":false")
        != std::string::npos);
    assert(switched.find("\"channel.model_built\":{\"available\":false")
        != std::string::npos);
    std::this_thread::sleep_for(std::chrono::milliseconds(2100));
    const std::string stale = t.snapshot_json_v2();
    assert(stale.find("\"audio.capture_input_available\":{\"available\":true")
        != std::string::npos);
    assert(stale.find("\"audio.capture_simulated\":{\"available\":true")
        != std::string::npos);
    assert(stale.find("\"audio.capture_ring_write_index_samples\":{\"available\":false")
        != std::string::npos);
    if (const char* packet_path = std::getenv("MERCURY_RRO_TEST_PACKET_PATH")) {
        std::ofstream stale_file(std::string(packet_path) + ".stale.json",
                                 std::ios::binary);
        assert(stale_file.good());
        stale_file << stale << '\n';
        assert(stale_file.good());
    }
    t.record_audio_input_state(false, false, 0);
    const std::string stopped = t.snapshot_json_v2();
    const auto input_at = stopped.find("\"audio.capture_input_available\":");
    assert(input_at != std::string::npos);
    assert(stopped.substr(input_at, stopped.find('}', input_at) - input_at)
        .find("\"value\":false") != std::string::npos);
    t.record_gearshift_session_reset();
    const std::string reset = t.snapshot_json_v2();
    assert(reset.find("\"gearshift.engagement_phase\":{\"available\":false")
        != std::string::npos);
    // Focused atomic-hook cost only; this is not an audio-loop or VR benchmark.
    constexpr int hotpath_calls = 100000;
    const auto enabled_begin = std::chrono::steady_clock::now();
    for (int i = 0; i < hotpath_calls; ++i)
        t.record_audio_capture_written(1, 0);
    const auto enabled_end = std::chrono::steady_clock::now();
    const auto carrier_begin = std::chrono::steady_clock::now();
    for (int i = 0; i < hotpath_calls; ++i)
        t.record_carrier_bin_handoff(0x3ff, 0x55, 16, 120, 24, 3);
    const auto carrier_end = std::chrono::steady_clock::now();
    std::thread carrier_writer([&] {
        for (int i = 0; i < 2000; ++i)
            t.record_carrier_bin_handoff(0x3ff, 0x55, 16, 120, 24, 3);
    });
    std::thread mode_switcher([&] {
        for (int i = 0; i < 2000; ++i)
            t.clear_ofdm_lattice();
    });
    for (int i = 0; i < 100; ++i) {
        const std::string raced = t.snapshot_json_v2();
        const bool mask_available = raced.find(
            "\"ofdm.carrier_data_mask\":{\"available\":true") != std::string::npos;
        const bool width_available = raced.find(
            "\"ofdm.carrier_width\":{\"available\":true") != std::string::npos;
        const bool cells_available = raced.find(
            "\"ofdm.carrier_data_cells\":{\"available\":true") != std::string::npos;
        assert(mask_available == width_available && width_available == cells_available);
    }
    carrier_writer.join();
    mode_switcher.join();
    t.stop();
    const auto disabled_begin = std::chrono::steady_clock::now();
    for (int i = 0; i < hotpath_calls; ++i)
        t.record_audio_capture_written(1, 0);
    const auto disabled_end = std::chrono::steady_clock::now();
    std::cerr << "hook enabled ns/call="
        << std::chrono::duration_cast<std::chrono::nanoseconds>(
            enabled_end - enabled_begin).count() / hotpath_calls
        << " disabled ns/call="
        << std::chrono::duration_cast<std::chrono::nanoseconds>(
            disabled_end - disabled_begin).count() / hotpath_calls
        << " carrier ns/handoff="
        << std::chrono::duration_cast<std::chrono::nanoseconds>(
            carrier_end - carrier_begin).count() / hotpath_calls << '\n';
}
