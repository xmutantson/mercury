#include "common/rro_telemetry.h"

#include <cassert>
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
    telemetry.record_ofdm_candidate(true);
    telemetry.record_ldpc_iterations(12);
    telemetry.record_crc_result(false);
    telemetry.record_gearshift({2, 2, 0, 16, 15, 16, 3,
        true, 1400, true, false, 2});
    const std::string after = telemetry.snapshot_json();
    assert(after.find("\"audio.capture_buffered_samples\":{\"available\":true")
           != std::string::npos);
    assert(after.find("\"decode.ldpc_iterations\":{\"available\":true")
           != std::string::npos);
    assert(after.find("\"decode.crc_ok\":{\"available\":true")
           != std::string::npos);
    assert(after.find("\"gearshift.lifecycle\":{\"available\":true")
           != std::string::npos);
    assert(after.find("\"gearshift.break_count_total\":{\"available\":true")
           != std::string::npos);
    assert(after.find("\"acquisition.active\":{\"available\":true")
           != std::string::npos);
    assert(after.find("\"ofdm.fft_size\":{\"available\":true")
           != std::string::npos);
    assert(after.find("\"decode.ldpc_max_iterations\":{\"available\":true")
           != std::string::npos);
    std::cout << before << '\n' << after << '\n';
    std::this_thread::sleep_for(std::chrono::milliseconds(2100));
    const std::string stale = telemetry.snapshot_json();
    assert(stale.find("\"audio.capture_buffered_samples\":{\"available\":false")
           != std::string::npos);
    assert(stale.find("\"audio.processing_load_ratio\":{\"available\":false")
           != std::string::npos);
    assert(stale.find("\"ofdm.fft_size\":{\"available\":false")
           != std::string::npos);
    std::cout << stale << '\n';
    telemetry.stop();
}
