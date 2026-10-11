/*
 * Mercury: A configurable open-source software-defined modem.
 * Copyright (C) 2022-2024 Fadi Jerji
 *               2024 Rhizomatica
 * Authors: Fadi Jerji
 *          Rafael Diniz
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, version 3 of the
 * License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include <iostream>
#include <complex>
#include <vector>
#include <fstream>
#include <chrono>
#include <cstdint>
#include <cerrno>
#include <cstdlib>
#include <ctime>
#include <cstdarg>
#include <atomic>   // R006: std::atomic<bool> shutdown_ (cross-thread termination flag)
#include <thread>   // R006: --test-shutdown-atomic cross-thread smoke
#include <type_traits> // R006: static_assert shutdown_ is atomic
#include <random>   // P1 frame-0 vehicle: deterministic AWGN run-up synthesis
#include <limits>
#include <cstring>  // FIX-C: memset for sigaction struct init (explicit, not transitive)
#include <csignal>  // FIX-C: SIGTERM/SIGINT graceful-shutdown handler (raise/SIGTERM)
#ifndef _WIN32
#include <signal.h> // FIX-C: POSIX sigaction/sigemptyset/struct sigaction
#endif
#include <math.h>
#include <unistd.h>
#include <iostream>
#include <complex>
#include "physical_layer/telecom_system.h"
#include "physical_layer/dist_matcher.h"   // --test-pas PAS/PCS DM bijection self-test
#include "physical_layer/mfsk_ctrl_codec_tests.h"
#include "common/sim_clock_tests.h"
#include "compression/test_winlink_dict.h"
#include "crypto/test_aead_nonce.h"   // AEAD bsi-bound nonce regression suite
#include "crypto/test_mlkem_hybrid.h" // ML-KEM-768 hybrid KEX regression suite
#include "datalink_layer/arq.h"
#include "datalink_layer/fade_core.h"
#include "datalink_layer/lp_transition_harness.h"
#include "audioio/audioio.h"
#include "common/sim_clock.h"
#include "common/build_id.h"
#include "test_main.h"

int run_b2f_bounded_output_test();
int run_fft_scratch_tests();

#ifdef MERCURY_GUI_ENABLED
#include "gui/gui_main.h"
#include "gui/gui_state.h"
#include "gui/ini_parser.h"
#endif

#if defined(_WIN32)
#include <windows.h>
#include <shlobj.h>
// Diagnostics from check_buffer_canaries — set before each canary read
extern volatile const char* g_canary_check_name;
extern volatile int g_canary_check_idx;
extern volatile const char* g_canary_check_ptr;

static LONG WINAPI crash_handler(EXCEPTION_POINTERS *ep) {
    DWORD code = ep->ExceptionRecord->ExceptionCode;
    void *addr = ep->ExceptionRecord->ExceptionAddress;
    DWORD tid = GetCurrentThreadId();
    fprintf(stderr, "\n[CRASH] Exception 0x%08lX at %p in thread %lu\n", code, addr, tid);
    if (code == 0xC0000005) {
        ULONG_PTR rw = ep->ExceptionRecord->ExceptionInformation[0];
        ULONG_PTR target = ep->ExceptionRecord->ExceptionInformation[1];
        fprintf(stderr, "[CRASH] ACCESS_VIOLATION: %s address %p\n",
            rw == 0 ? "reading" : rw == 1 ? "writing" : "executing", (void*)target);
    }
    if (code == 0xC0000374) {
        fprintf(stderr, "[CRASH] HEAP_CORRUPTION detected by heap manager\n");
    }
    fprintf(stderr, "[CRASH] RIP=%p RSP=%p\n",
        (void*)ep->ContextRecord->Rip, (void*)ep->ContextRecord->Rsp);
    // Print which canary buffer was being checked when we crashed
    if (g_canary_check_name != NULL) {
        fprintf(stderr, "[CRASH] Canary check was on: %s[%d] ptr=%p\n",
            (const char*)g_canary_check_name, (int)g_canary_check_idx, (const void*)g_canary_check_ptr);
    }
    fflush(stderr);
    fflush(stdout);
    return EXCEPTION_CONTINUE_SEARCH;
}
#endif

// --- Tee logging: pipe stdout through a thread to both console and log file ---
#ifdef _WIN32
#include <io.h>
#include <fcntl.h>
#define DUP_FD(fd)          _dup(fd)
#define DUP2_FD(src,dst)    _dup2(src,dst)
#define CLOSE_FD(fd)        _close(fd)
#define READ_FD(fd,buf,n)   _read(fd,buf,(unsigned int)(n))
#define WRITE_FD(fd,buf,n)  _write(fd,buf,(unsigned int)(n))
#define PIPE_FD(fds,sz)     _pipe(fds,sz,_O_BINARY)
#else
#include <pthread.h>
#define DUP_FD(fd)          dup(fd)
#define DUP2_FD(src,dst)    dup2(src,dst)
#define CLOSE_FD(fd)        close(fd)
#define READ_FD(fd,buf,n)   read(fd,buf,n)
#define WRITE_FD(fd,buf,n)  write(fd,buf,n)
#define PIPE_FD(fds,sz)     pipe(fds)
#endif

static FILE* g_log_file = NULL;
static int g_saved_stdout_fd = -1;
static int g_pipe_read_fd = -1;
#ifdef _WIN32
static HANDLE g_tee_thread = NULL;
#else
static pthread_t g_tee_thread;
static bool g_tee_thread_created = false;
#endif

static void write_timestamp(FILE* f) {
#ifdef _WIN32
    SYSTEMTIME st;
    GetLocalTime(&st);
    fprintf(f, "[%02d:%02d:%02d.%03d] ", st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
#else
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    struct tm tm;
    localtime_r(&ts.tv_sec, &tm);
    fprintf(f, "[%02d:%02d:%02d.%03ld] ", tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec / 1000000);
#endif
}

#ifdef _WIN32
static DWORD WINAPI tee_thread_func(LPVOID arg) {
#else
static void* tee_thread_func(void* arg) {
#endif
    (void)arg;
    char buf[4096];
    int n;
    bool at_line_start = true;
    while ((n = READ_FD(g_pipe_read_fd, buf, sizeof(buf))) > 0) {
        WRITE_FD(g_saved_stdout_fd, buf, n);
        if (g_log_file) {
            for (int i = 0; i < n; i++) {
                if (at_line_start) {
                    write_timestamp(g_log_file);
                    at_line_start = false;
                }
                fputc(buf[i], g_log_file);
                if (buf[i] == '\n') at_line_start = true;
            }
            fflush(g_log_file);
        }
    }
#ifdef _WIN32
    return 0;
#else
    return NULL;
#endif
}

static void setup_tee_logging(const char* path, int argc, char* argv[]) {
    g_log_file = fopen(path, "w");
    if (!g_log_file) {
        fprintf(stderr, "Cannot open log file: %s\n", path);
        return;
    }
    setvbuf(g_log_file, NULL, _IONBF, 0);
    // Write header
    time_t now = time(NULL);
    struct tm* t = localtime(&now);
    fprintf(g_log_file, "=== Mercury v%s log started %04d-%02d-%02d %02d:%02d:%02d ===\n",
        VERSION__, t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
        t->tm_hour, t->tm_min, t->tm_sec);
    fprintf(g_log_file, "=== Command:");
    for (int i = 0; i < argc; i++) fprintf(g_log_file, " %s", argv[i]);
    fprintf(g_log_file, " ===\n");
    fflush(g_log_file);

    // Save original stdout fd
    g_saved_stdout_fd = DUP_FD(1);
    // Create pipe
    int fds[2];
    if (PIPE_FD(fds, 8192) != 0) {
        fprintf(stderr, "Failed to create logging pipe\n");
        fclose(g_log_file);
        g_log_file = NULL;
        CLOSE_FD(g_saved_stdout_fd);
        g_saved_stdout_fd = -1;
        return;
    }
    g_pipe_read_fd = fds[0];
    // Redirect stdout to pipe write end
    DUP2_FD(fds[1], 1);
    CLOSE_FD(fds[1]);
    setvbuf(stdout, NULL, _IONBF, 0);
    // Start tee thread
#ifdef _WIN32
    g_tee_thread = CreateThread(NULL, 0, tee_thread_func, NULL, 0, NULL);
#else
    pthread_create(&g_tee_thread, NULL, tee_thread_func, NULL);
    g_tee_thread_created = true;
#endif
}

static void shutdown_tee_logging() {
    if (g_saved_stdout_fd < 0) return;
    fflush(stdout);
    // Restore original stdout — closes pipe write end, reader gets EOF
    DUP2_FD(g_saved_stdout_fd, 1);
    CLOSE_FD(g_saved_stdout_fd);
    g_saved_stdout_fd = -1;
    // Wait for reader thread to finish
#ifdef _WIN32
    if (g_tee_thread) { WaitForSingleObject(g_tee_thread, 5000); CloseHandle(g_tee_thread); g_tee_thread = NULL; }
#else
    if (g_tee_thread_created) { pthread_join(g_tee_thread, NULL); g_tee_thread_created = false; }
#endif
    if (g_pipe_read_fd >= 0) { CLOSE_FD(g_pipe_read_fd); g_pipe_read_fd = -1; }
    if (g_log_file) { fclose(g_log_file); g_log_file = NULL; }
}

// some globals TODO: wrap this up into some struct
extern "C" {
    double carrier_frequency_offset; // set 0 to stock HF, or to the radio passband, eg., 15k for sBitx
    double test_tx_carrier_offset;   // Test mode: artificial TX carrier offset in Hz
    int radio_type;
    char *input_dev;
    char *output_dev;
    // R006 fix (race audit 2026-06-06): shutdown_ is read in main-thread spin
    // loops AND written from the audio capture/playback/sim threads
    // (audioio.c). A plain `bool` makes the concurrent unsynchronized
    // read+write a C/C++ data race (UB). Make it a lock-free atomic so the
    // termination flag is well-defined across threads. The C TU (audioio.c)
    // declares the matching `extern _Atomic bool shutdown_;`
    // (std::atomic<bool> and _Atomic bool are representation-compatible for
    // the always-lock-free bool type). Mirrors gui_state.h:185
    // `std::atomic<bool> request_shutdown{false}`. memory_order is the default
    // seq_cst — these are infrequent termination predicates, not a hot path.
    std::atomic<bool> shutdown_{false};
    // Audio channel configuration (0=LEFT, 1=RIGHT, 2=STEREO)
    extern int configured_input_channel;
    extern int configured_output_channel;
    extern int multichannel_mode;
    extern double noise_snr_db;
}

int g_verbose = 0;

// ============================================================================
// FIX-C — graceful SIGTERM / SIGINT shutdown (ALSA capture-substream leak fix)
// ============================================================================
//
// PROBLEM (RPi Fe-Pi / sgtl5000 capture-substream leak, root cause of a recurring
// testbed wedge that required a physical power-cycle): Mercury installed NO
// handler for SIGTERM/SIGINT, so the default disposition (terminate) killed the
// process the instant the bench/butler did `kill mercury` (or even a plain
// SIGTERM). The audio threads' ALSA cleanup — radio_capture_thread / radio_-
// playback_thread fall-through to `audio->free(b)` (audioio.c:1332 / :938) which
// calls ffalsa_free -> snd_pcm_close(b->pcm) (ffaudio/alsa.c:191) — only runs
// when the thread loop observes `shutdown_ == true` and winds down, after which
// audioio_deinit() pthread_joins all three (audioio.c:1840-1842). With no handler
// that orderly wind-down never started: the threads were torn down mid-loop,
// snd_pcm_close never ran, and the Fe-Pi capture substream leaked
// (`arecord -l` shows Subdevices: 0/1 with no holder; every later snd_pcm_open
// returns -16 EBUSY until power-cycle).
//
// FIX: install an async-signal-safe SIGTERM + SIGINT handler whose ONLY action
// is a single atomic store `shutdown_ = true`. That is the exact flip the audio
// threads already poll (`while(!shutdown_)`), so a graceful kill now triggers the
// SAME orderly wind-down a clean exit does: threads stop -> snd_pcm_close (PCM
// released) -> audioio_deinit joins -> process exits with the substream FREED.
//
// async-signal-safety: a std::atomic<bool> store is async-signal-safe (it is a
// lock-free atomic — guaranteed by the --test-shutdown-atomic is_lock_free()
// check above; on this target bool is always-lock-free). The handler does NOT
// printf, does NOT lock, does NOT touch any non-atomic state — the only things a
// handler is permitted to do are the atomic store and `return`. sigaction (not
// signal()) is used for portable, well-defined semantics, and SA_RESTART is
// deliberately OMITTED so a blocked syscall (e.g. snd_pcm_readi waiting on a
// capture period) returns EINTR and the audio loop re-checks `shutdown_` at once
// rather than waiting out the full period.
#ifndef _WIN32
extern "C" void mercury_termination_signal_handler(int /*signum*/)
{
    // ONLY async-signal-safe action: flip the termination flag the audio
    // threads + main loops already poll. No printf, no lock, no allocation.
    shutdown_.store(true, std::memory_order_seq_cst);
}

// Install the SIGTERM/SIGINT handler. Call once, early in main(), BEFORE any
// audio thread is launched. Idempotent and side-effect-free apart from the
// disposition change.
static void install_termination_handlers()
{
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = mercury_termination_signal_handler;
    sigemptyset(&sa.sa_mask);
    // No SA_RESTART: let blocked syscalls (snd_pcm_readi etc.) return EINTR so
    // the audio loops re-check shutdown_ immediately instead of one period late.
    sa.sa_flags = 0;
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT,  &sa, NULL);
}
#else
// Windows build: no POSIX signals on the RPi-leak path; keep a no-op so the
// call site in main() stays unconditional and the bench/Linux path is the only
// place the handler actually arms.
static void install_termination_handlers() {}
#endif


static bool parse_encryption_mode_cli(const char* value, int& mode, std::string& error)
{
    error.clear();
    if (strcmp(value, "strict") == 0 || strcmp(value, "1") == 0)
        mode = ENCRYPT_STRICT;
    else if (strcmp(value, "fast") == 0 || strcmp(value, "2") == 0)
        mode = ENCRYPT_FAST;
    else
    {
        error = "Unknown encryption mode '" + std::string(value)
              + "', use 'strict' or 'fast'";
        return false;
    }
    return true;
}

static bool parse_modulation_cli(const char* value, int& config, std::string& error)
{
    error.clear();
    char* end = NULL;
    long parsed = strtol(value, &end, 10);
    if (end == value || *end != '\0'
        || parsed < std::numeric_limits<int>::min()
        || parsed > std::numeric_limits<int>::max())
    {
        error = "Invalid modulation config '" + std::string(value) + "'";
        return false;
    }

    int candidate = static_cast<int>(parsed);
    if (!is_ofdm_config(candidate) && !is_robust_config(candidate)
        && !is_low48_anchor_config(candidate))
    {
        error = "Unknown modulation config '" + std::string(value) + "'";
        return false;
    }

    config = candidate;
    return true;
}

static bool parse_tx_level_cli(const char* value, double& level, std::string& error)
{
    error.clear();
    errno = 0;
    char* end = NULL;
    double parsed = strtod(value, &end);
    if (end == value || *end != '\0' || errno == ERANGE
        || !std::isfinite(parsed) || parsed < 0.0 || parsed > 1.0)
    {
        error = "Invalid TX level '" + std::string(value)
              + "', expected a number in the range 0.0-1.0";
        return false;
    }

    level = parsed;
    return true;
}

static bool parse_bandpass_cli(const char* value, double& low_hz, double& high_hz,
                               std::string& error)
{
    error.clear();
    const std::string input = value == NULL ? "" : value;
    const std::string::size_type separator = input.find('-');
    if (separator == std::string::npos || separator == 0
        || separator + 1 == input.size()
        || input.find('-', separator + 1) != std::string::npos)
    {
        error = "Invalid bandpass '" + input
              + "', expected LOW-HIGH in the range 0-24000 Hz";
        return false;
    }

    const std::string low_text = input.substr(0, separator);
    const std::string high_text = input.substr(separator + 1);
    errno = 0;
    char* low_end = NULL;
    const double parsed_low = strtod(low_text.c_str(), &low_end);
    const bool low_valid = low_end != low_text.c_str() && *low_end == '\0'
        && errno != ERANGE && std::isfinite(parsed_low);

    errno = 0;
    char* high_end = NULL;
    const double parsed_high = strtod(high_text.c_str(), &high_end);
    const bool high_valid = high_end != high_text.c_str() && *high_end == '\0'
        && errno != ERANGE && std::isfinite(parsed_high);

    if (!low_valid || !high_valid || parsed_low < 0.0
        || parsed_low >= parsed_high || parsed_high > 24000.0)
    {
        error = "Invalid bandpass '" + input
              + "', expected 0 <= LOW < HIGH <= 24000 Hz";
        return false;
    }

    low_hz = parsed_low;
    high_hz = parsed_high;
    return true;
}

static bool parse_audio_subsystem_cli(const char* value, int& audio_system,
                                      std::string& error)
{
    error.clear();
    int parsed = -1;
    if (strcmp(value, "alsa") == 0)
        parsed = AUDIO_SUBSYSTEM_ALSA;
    else if (strcmp(value, "pulse") == 0)
        parsed = AUDIO_SUBSYSTEM_PULSE;
    else if (strcmp(value, "dsound") == 0)
        parsed = AUDIO_SUBSYSTEM_DSOUND;
    else if (strcmp(value, "wasapi") == 0)
        parsed = AUDIO_SUBSYSTEM_WASAPI;
    else if (strcmp(value, "oss") == 0)
        parsed = AUDIO_SUBSYSTEM_OSS;
    else if (strcmp(value, "coreaudio") == 0)
        parsed = AUDIO_SUBSYSTEM_COREAUDIO;
    else if (strcmp(value, "sim") == 0)
        parsed = AUDIO_SUBSYSTEM_SIM;
    else
    {
        error = "Unknown audio subsystem '" + std::string(value)
              + "', use 'alsa', 'pulse', 'dsound', 'wasapi', 'oss', "
                "'coreaudio', or 'sim'";
        return false;
    }

    audio_system = parsed;
    return true;
}

static void copy_audio_device_path(char* destination, const char* source)
{
    strncpy(destination, source, ALSA_MAX_PATH - 1);
    destination[ALSA_MAX_PATH - 1] = '\0';
}

int cl_arq_controller::test_modulation_cli_validation()
{
    int fails = 0;
    auto check = [&](const char* value, bool want_ok, int initial, int want_config)
    {
        int config = initial;
        std::string error;
        bool ok = parse_modulation_cli(value, config, error);
        bool pass = ok == want_ok && config == want_config
                 && (want_ok ? error.empty() : !error.empty());
        if (!pass) fails++;
        printf("[TEST-MOD-CLI] -s %-6s ok=%d config=%d diagnostic=%d -> %s\n",
               value, ok ? 1 : 0, config, error.empty() ? 0 : 1,
               pass ? "PASS" : "FAIL");
    };

    check("0",     true,  CONFIG_1, CONFIG_0);
    check("17",    true,  CONFIG_1, CONFIG_17);
    check("100",   true,  CONFIG_1, ROBUST_0);
    check("105",   true,  CONFIG_1, LOW48_ANCHOR_S20_R6);
    check("INVALID", false, CONFIG_17, CONFIG_17);
    check("BPSKZ", false, CONFIG_17, CONFIG_17);
    check("1junk", false, CONFIG_17, CONFIG_17);
    check("104",   false, CONFIG_17, CONFIG_17);

    printf("[TEST-MOD-CLI] %s (%d failure(s))\n",
           fails == 0 ? "PASS" : "FAIL", fails);
    return fails;
}

int cl_arq_controller::test_data_container_ownership_constraints()
{
    const bool copy_constructible =
        std::is_copy_constructible<cl_data_container>::value;
    const bool copy_assignable =
        std::is_copy_assignable<cl_data_container>::value;
    const bool pass = !copy_constructible && !copy_assignable;

    printf("[TEST-DATA-CONTAINER-OWNERSHIP] copy-constructible=%d "
           "copy-assignable=%d -> %s\n",
           copy_constructible ? 1 : 0, copy_assignable ? 1 : 0,
           pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}

int cl_arq_controller::test_data_container_deinit_resets_pinning()
{
    cl_data_container data;
    data.alloc_shared_buffers(256, 50, 4, 112, 128, 8, 4, 1, 1200);

    const bool pin_precondition = data.precook_ring_pinned
        && data.pinned_capacity_buffer_Nsymb == 1200
        && data.pinned_capacity_samples == 128 * 1200
        && data.pinned_capacity_Nc == 50
        && data.pinned_capacity_nsymb_nc > 0
        && data.pinned_capacity_nData == 256
        && data.pinned_capacity_total_frame_size > 0
        && data.pinned_capacity_preamble == 4
        && data.pinned_capacity_fine_slice > 0;

    data.deinit();
    const bool metadata_reset = !data.precook_ring_pinned
        && data.pinned_capacity_buffer_Nsymb == 0
        && data.pinned_capacity_samples == 0
        && data.pinned_capacity_Nc == 0
        && data.pinned_capacity_nsymb_nc == 0
        && data.pinned_capacity_nData == 0
        && data.pinned_capacity_total_frame_size == 0
        && data.pinned_capacity_preamble == 0
        && data.pinned_capacity_fine_slice == 0;

    // A different, smaller geometry must take set_size()'s allocation path again.
    data.set_size(128, 50, 2, 80, 96, 4, 2, 1);
    const bool reallocated = data.data_bit != nullptr
        && data.modulated_data != nullptr
        && data.passband_delayed_data != nullptr
        && data.ready_to_process_passband_delayed_data != nullptr
        && data.baseband_data != nullptr
        && data.passband_data_tx != nullptr
        && data.bit_energy_dispersal_sequence != nullptr;

    const bool pass = pin_precondition && metadata_reset && reallocated;
    printf("[TEST-DATA-CONTAINER-DEINIT] pin-precondition=%d metadata-reset=%d "
           "reallocated=%d -> %s\n",
           pin_precondition ? 1 : 0, metadata_reset ? 1 : 0,
           reallocated ? 1 : 0, pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}

int cl_arq_controller::test_gbf_decoder_constraints()
{
    const float llr[2] = {1.0f, 1.0f};
    const int checks[2] = {0, 1};
    const int iteration_limit = 4;
    const int untouched = 0x5a5a5a5a;
    int fails = 0;

    int valid_output[1] = {untouched};
    const int valid_result = decode_GBF(
        llr, valid_output, checks, 2, 2, 2, 1, 1, iteration_limit, 0.5f);
    const bool valid_ok = valid_result == 0 && valid_output[0] == 0;
    if (!valid_ok) fails++;

    int inconsistent_output[2] = {untouched, untouched};
    const int inconsistent_result = decode_GBF(
        llr, inconsistent_output, checks, 2, 2, 2, 2, 1,
        iteration_limit, 0.5f);
    const bool inconsistent_ok = inconsistent_result == iteration_limit + 1
        && inconsistent_output[0] == untouched
        && inconsistent_output[1] == untouched;
    if (!inconsistent_ok) fails++;

    int width_output[1] = {untouched};
    const int width_result = decode_GBF(
        llr, width_output, checks, 2, 1, 2, 1, 1,
        iteration_limit, 0.5f);
    const bool width_ok = width_result == iteration_limit + 1
        && width_output[0] == untouched;
    if (!width_ok) fails++;

    const int invalid_checks[2] = {0, 2};
    int index_output[1] = {untouched};
    const int index_result = decode_GBF(
        llr, index_output, invalid_checks, 2, 2, 2, 1, 1,
        iteration_limit, 0.5f);
    const bool index_ok = index_result == iteration_limit + 1
        && index_output[0] == untouched;
    if (!index_ok) fails++;

    printf("[TEST-GBF-CONSTRAINTS] valid=%s inconsistent-NKP=%s width=%s index=%s -> %s\n",
           valid_ok ? "PASS" : "FAIL",
           inconsistent_ok ? "PASS" : "FAIL",
           width_ok ? "PASS" : "FAIL",
           index_ok ? "PASS" : "FAIL",
           fails == 0 ? "PASS" : "FAIL");
    return fails;
}

int cl_arq_controller::test_gbf_final_iteration_sentinel()
{
    // One parity check over two bits. With eta=0.5, bit 0 reaches zero after
    // two flips, so the syndrome first clears at the iteration-3 check.
    const float llr[2] = {-1.0f, 10.0f};
    const int checks[2] = {0, 1};
    const int iteration_limit = 3;
    const int failure = iteration_limit + 1;
    int decoded[1] = {-1};
    int fails = 0;

    const int result = decode_GBF(
        llr, decoded, checks, 2, 2, 2, 1, 1,
        iteration_limit, 0.5f);

    cl_telecom_system telecom;
    telecom.ldpc.nIteration_max = iteration_limit;
    st_receive_stats rs{};
    rs.all_zeros = NO;
    rs.crc = 0;

    // Fail-before: the former boundary result looked converged to the real CRC
    // admission predicate, so a CRC-colliding frame was accepted.
    rs.iterations_done = iteration_limit;
    const bool legacy_rejected = telecom.frame_decode_rejected(
        rs, CRC16_MODBUS_RTU);
    if(legacy_rejected) fails++;

    // Pass-after: the decoder publishes the canonical failure sentinel and the
    // production admission predicate rejects the same CRC-good hard bits.
    rs.iterations_done = result;
    const bool fixed_rejected = telecom.frame_decode_rejected(
        rs, CRC16_MODBUS_RTU);
    if(result != failure || decoded[0] != 0 || !fixed_rejected) fails++;

    // The boundary alone is rejected: with one more allowed iteration, the
    // identical convergence at iteration 3 remains an ordinary success.
    int control_decoded[1] = {-1};
    const int control_result = decode_GBF(
        llr, control_decoded, checks, 2, 2, 2, 1, 1,
        iteration_limit + 1, 0.5f);
    telecom.ldpc.nIteration_max = iteration_limit + 1;
    rs.iterations_done = control_result;
    const bool control_rejected = telecom.frame_decode_rejected(
        rs, CRC16_MODBUS_RTU);
    if(control_result != iteration_limit || control_decoded[0] != 0
       || control_rejected) fails++;

    printf("[TEST-GBF-FINAL-ITERATION] legacy=%s result=%d sentinel=%d rejected=%d "
           "control-result=%d control-rejected=%d -> %s\n",
           legacy_rejected ? "REJECT" : "ACCEPT", result, failure,
           fixed_rejected ? 1 : 0, control_result,
           control_rejected ? 1 : 0, fails == 0 ? "PASS" : "FAIL");
    return fails;
}

int cl_arq_controller::test_tx_level_cli_validation()
{
    int fails = 0;
    auto check = [&](const char* value, bool want_ok, double initial, double want_level)
    {
        double level = initial;
        std::string error;
        bool ok = parse_tx_level_cli(value, level, error);
        bool pass = ok == want_ok && level == want_level
                 && (want_ok ? error.empty() : !error.empty());
        if (!pass) fails++;
        printf("[TEST-TX-LEVEL-CLI] --tx-level %-8s ok=%d level=%.3f diagnostic=%d -> %s\n",
               value, ok ? 1 : 0, level, error.empty() ? 0 : 1,
               pass ? "PASS" : "FAIL");
    };

    check("0.0",  true,  0.25, 0.0);
    check("0.5",  true,  0.25, 0.5);
    check("1.0",  true,  0.25, 1.0);
    check("-0.1", false, 0.25, 0.25);
    check("5.0",  false, 0.25, 0.25);
    check("nan",  false, 0.25, 0.25);
    check("0.5x", false, 0.25, 0.25);

    printf("[TEST-TX-LEVEL-CLI] %s (%d failure(s))\n",
           fails == 0 ? "PASS" : "FAIL", fails);
    return fails;
}

int cl_arq_controller::test_tx_level_concurrent_sync()
{
    const double saved = audioio_get_tx_level();
    int fails = 0;

    // Fail closed: a malformed runtime parameter must not replace the last
    // valid value. These are the same bounds enforced by --tx-level parsing.
    if (!audioio_set_tx_level(0.25))
        fails++;
    const double before_invalid = audioio_get_tx_level();
    const bool rejected = !audioio_set_tx_level(
        std::numeric_limits<double>::quiet_NaN());
    const bool unchanged = audioio_get_tx_level() == before_invalid;
    if (!rejected || !unchanged)
        fails++;

    // Model a parameter callback publishing updates while the playback/main
    // consumer snapshots the active level. The pre-fix global double makes
    // this access a C++ data race; the accessor pair makes every publication
    // atomic, and a consumer can use one coherent snapshot for a whole block.
    std::atomic<bool> start{false};
    std::atomic<bool> done{false};
    std::atomic<int> writer_failures{0};
    std::thread writer([&]() {
        while (!start.load(std::memory_order_acquire))
            std::this_thread::yield();
        for (int i = 0; i < 200000; ++i) {
            const double value = (i & 1) ? 0.75 : 0.25;
            if (!audioio_set_tx_level(value))
                writer_failures.fetch_add(1, std::memory_order_relaxed);
        }
        done.store(true, std::memory_order_release);
    });

    start.store(true, std::memory_order_release);
    int invalid_reads = 0;
    do {
        const double value = audioio_get_tx_level();
        if (value != 0.25 && value != 0.75)
            invalid_reads++;
    } while (!done.load(std::memory_order_acquire));
    writer.join();

    const double final_value = audioio_get_tx_level();
    if (writer_failures.load(std::memory_order_relaxed) != 0 ||
        invalid_reads != 0 || final_value != 0.75)
        fails++;

    // Avoid leaking test state into later --test cases; the saved value came
    // from the validated production store and therefore must restore cleanly.
    const bool restored = audioio_set_tx_level(saved);
    if (!restored)
        fails++;

    printf("[TEST-TX-LEVEL-SYNC] rejected-invalid=%d unchanged=%d "
           "writer-failures=%d invalid-reads=%d final=%.2f restored=%d -> %s\n",
           rejected ? 1 : 0, unchanged ? 1 : 0,
           writer_failures.load(std::memory_order_relaxed), invalid_reads,
           final_value, restored ? 1 : 0, fails == 0 ? "PASS" : "FAIL");
    return fails;
}

int cl_arq_controller::test_bandpass_cli_validation()
{
    int fails = 0;
    auto check = [&](char* value, bool want_ok, double want_low, double want_high)
    {
        const std::string original(value);
        double low = 123.0;
        double high = 456.0;
        std::string error;
        const bool ok = parse_bandpass_cli(value, low, high, error);
        const bool unchanged = original == value;
        const bool pass = ok == want_ok && unchanged
            && low == (want_ok ? want_low : 123.0)
            && high == (want_ok ? want_high : 456.0)
            && (want_ok ? error.empty() : !error.empty());
        if (!pass) fails++;
        printf("[TEST-BANDPASS-CLI] --bandpass %-18s ok=%d low=%.1f high=%.1f "
               "argv-unchanged=%d diagnostic=%d -> %s\n",
               value, ok ? 1 : 0, low, high, unchanged ? 1 : 0,
               error.empty() ? 0 : 1, pass ? "PASS" : "FAIL");
    };

    char valid[] = "300-3000";
    char boundaries[] = "0-24000";
    char invalid_format[] = "invalid-format";
    char missing_high[] = "300-";
    char trailing_junk[] = "300-3000Hz";
    char reversed[] = "3000-300";
    char out_of_range[] = "300-24001";
    char non_finite[] = "300-inf";
    check(valid, true, 300.0, 3000.0);
    check(boundaries, true, 0.0, 24000.0);
    check(invalid_format, false, 0.0, 0.0);
    check(missing_high, false, 0.0, 0.0);
    check(trailing_junk, false, 0.0, 0.0);
    check(reversed, false, 0.0, 0.0);
    check(out_of_range, false, 0.0, 0.0);
    check(non_finite, false, 0.0, 0.0);

    printf("[TEST-BANDPASS-CLI] %s (%d failure(s))\n",
           fails == 0 ? "PASS" : "FAIL", fails);
    return fails;
}

int cl_arq_controller::test_audio_device_path_bounds()
{
    char input[ALSA_MAX_PATH + 1];
    char output[ALSA_MAX_PATH + 1];
    memset(input, 0xa5, sizeof(input));
    memset(output, 0xa5, sizeof(output));
    input[ALSA_MAX_PATH] = 'I';
    output[ALSA_MAX_PATH] = 'O';

    const std::string overlong(ALSA_MAX_PATH + 72, 'i');
    const std::string boundary(ALSA_MAX_PATH - 1, 'o');
    copy_audio_device_path(input, overlong.c_str());
    copy_audio_device_path(output, boundary.c_str());

    const bool input_ok = input[ALSA_MAX_PATH - 1] == '\0'
        && input[ALSA_MAX_PATH] == 'I'
        && memcmp(input, overlong.data(), ALSA_MAX_PATH - 1) == 0;
    const bool output_ok = output[ALSA_MAX_PATH - 1] == '\0'
        && output[ALSA_MAX_PATH] == 'O'
        && memcmp(output, boundary.data(), ALSA_MAX_PATH - 1) == 0;
    const bool pass = input_ok && output_ok;

    printf("[TEST-AUDIO-DEVICE-PATH] overlong-input=%s boundary-output=%s -> %s\n",
           input_ok ? "PASS" : "FAIL", output_ok ? "PASS" : "FAIL",
           pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}

int cl_arq_controller::test_audio_subsystem_cli_validation()
{
    int fails = 0;
    auto check = [&](const char* value, bool want_ok, int initial, int want_system)
    {
        int audio_system = initial;
        std::string error;
        bool ok = parse_audio_subsystem_cli(value, audio_system, error);
        bool pass = ok == want_ok && audio_system == want_system
                 && (want_ok ? error.empty() : !error.empty());
        if (!pass) fails++;
        printf("[TEST-AUDIO-SUBSYSTEM-CLI] -x %-10s ok=%d system=%d diagnostic=%d -> %s\n",
               value, ok ? 1 : 0, audio_system, error.empty() ? 0 : 1,
               pass ? "PASS" : "FAIL");
    };

    check("alsa",      true,  -1, AUDIO_SUBSYSTEM_ALSA);
    check("pulse",     true,  -1, AUDIO_SUBSYSTEM_PULSE);
    check("dsound",    true,  -1, AUDIO_SUBSYSTEM_DSOUND);
    check("wasapi",    true,  -1, AUDIO_SUBSYSTEM_WASAPI);
    check("oss",       true,  -1, AUDIO_SUBSYSTEM_OSS);
    check("coreaudio", true,  -1, AUDIO_SUBSYSTEM_COREAUDIO);
    check("sim",       true,  -1, AUDIO_SUBSYSTEM_SIM);
    check("foo",       false, -1, -1);
    check("foo",       false, AUDIO_SUBSYSTEM_PULSE, AUDIO_SUBSYSTEM_PULSE);

    printf("[TEST-AUDIO-SUBSYSTEM-CLI] %s (%d failure(s))\n",
           fails == 0 ? "PASS" : "FAIL", fails);
    return fails;
}

static int validate_modem_server_startup(int result, FILE* diagnostic)
{
    if (result == SUCCESSFUL)
        return EXIT_SUCCESS;

    fprintf(diagnostic, "ERROR: AGW/TCP server startup failed; modem startup aborted\n");
    return EXIT_FAILURE;
}

int cl_arq_controller::test_agw_server_start_fail_closed()
{
    FILE* diagnostic = tmpfile();
    if (diagnostic == NULL)
    {
        printf("[TEST-AGW-START] FAIL: could not create diagnostic capture\n");
        return 1;
    }
    const int accepted = validate_modem_server_startup(SUCCESSFUL, diagnostic);
    const int rejected = validate_modem_server_startup(ERROR_, diagnostic);
    fflush(diagnostic);
    rewind(diagnostic);
    char message[160] = {0};
    const size_t message_length = fread(message, 1, sizeof(message) - 1, diagnostic);
    fclose(diagnostic);

    const bool passed = accepted == EXIT_SUCCESS
        && rejected == EXIT_FAILURE
        && message_length > 0
        && strstr(message, "server startup failed") != NULL;
    printf("[TEST-AGW-START] success=%d bind-failure=%d diagnostic=%d: %s\n",
           accepted, rejected, message_length > 0, passed ? "PASS" : "FAIL");
    return passed ? 0 : 1;
}

int cl_arq_controller::test_encryption_cli_validation()
{
    int fails = 0;
    auto check = [&](const char* value, bool want_ok, int initial_mode, int want_mode)
    {
        int mode = initial_mode;
        std::string error;
        bool ok = parse_encryption_mode_cli(value, mode, error);
        const int resolved_mode = (mode >= 0) ? mode : ENCRYPT_OFF;
        const bool plaintext_fallback = ok && resolved_mode == ENCRYPT_OFF;
        bool pass = ok == want_ok && mode == want_mode
                 && (want_ok ? error.empty() : !error.empty())
                 && (want_ok || !plaintext_fallback);
        if (!pass) fails++;
        printf("[TEST-ENC-CLI] --encrypt %-7s startup=%s cli-mode=%d resolved-mode=%d "
               "plaintext-fallback=%d "
               "diagnostic=%d -> %s\n",
               value, ok ? "continue" : "abort", mode, resolved_mode,
               plaintext_fallback ? 1 : 0,
               error.empty() ? 0 : 1,
               pass ? "PASS" : "FAIL");
    };

    check("strict",  true,  -1,          ENCRYPT_STRICT);
    check("1",       true,  -1,          ENCRYPT_STRICT);
    check("fast",    true,  -1,          ENCRYPT_FAST);
    check("2",       true,  -1,          ENCRYPT_FAST);
    // Reproduce the dangerous fallback path: the untouched -1 sentinel would
    // resolve to ENCRYPT_OFF, so an unknown explicit request must abort startup.
    check("strictt", false, -1, -1);

    printf("[TEST-ENC-CLI] %s (%d failure(s))\n",
           fails == 0 ? "PASS" : "FAIL", fails);
    return fails;
}

int main(int argc, char *argv[])
{
    if (argc == 2 && strcmp(argv[1], "--print-build-id") == 0) {
        std::printf("%s\n", MERCURY_BUILD_ID);
        return 0;
    }

    int main_exit_status = EXIT_SUCCESS;
    bool gui_thread_abandoned = false;

#if defined(_WIN32)
    SetUnhandledExceptionFilter(crash_handler);
    // Also try vectored handler for heap corruption
    AddVectoredExceptionHandler(1, crash_handler);
#endif
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    // FIX-C: arm the graceful SIGTERM/SIGINT handler BEFORE any audio thread is
    // launched. The handler's only action is `shutdown_ = true`, which drives
    // the audio threads' orderly wind-down (snd_pcm_close on both PCMs) +
    // audioio_deinit join — so a `kill mercury` / Ctrl-C releases the ALSA
    // capture substream instead of leaking it (RPi Fe-Pi EBUSY-until-power-cycle
    // bug). Installed this early so it is also armed for the long-running
    // --test paths and any pre-mode work; it is a pure disposition change with
    // no effect until a signal actually arrives.
    install_termination_handlers();

    // --test : run built-in unit tests and exit. Phase B Wave 1 (this
    // build) wires the MFSK ctrl-suffix codec suite (alphabet, payload
    // pack/unpack, CRC12 corruption, base-pattern cross-correlation,
    // bitmap-30 cap, passband round-trip, HAIL false-trigger). Additional
    // test groups can be added by extending run_mfsk_ctrl_codec_tests's
    // caller below. The flag must be checked before any audio/GUI/threading
    // init so the test process stays minimal.
    for (int i = 1; i < argc; i++) {
        bool test_handled = false;
        int test_status = run_main_test_dispatch(argc, argv, i,
            install_termination_handlers, test_handled);
        if (test_handled) return test_status;
        // --rescore-recovery-ack <iqfile.bin> : LEVER 2 OFFLINE NON-CIRCULAR RESCORE
        // (data-flow-recovery-ack-capture.md §7). Replays a REAL captured recovery-ACK
        // I/Q buffer (the MERCURY_RECOVERY_ACK_DIAG dump: interleaved float64 I,Q,
        // dec_size complex samples = EXACTLY data_container.baseband_data_interpolated
        // [0..size/M) the production recovery poll scored) through the EXACT production
        // scorer ofdm.detect_ack_pattern(iq, n, /*interp=*/1, ...) and reports both the
        // deployed no-fine coarse path AND the LEVER-2 fine+partial-tail path, plus the
        // LEVER-2 two-arm accept decision. The keystone non-circular gate: it isolates
        // the detector relaxation from live-A/B confounds by replaying the real recorded
        // I/Q. Diagnostic-only; no default-path cost; byte-identical guarantee untouched.
        if (strcmp(argv[i], "--rescore-recovery-ack") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "--rescore-recovery-ack needs an I/Q .bin path\n");
                return 2;
            }
            const char* iqpath = argv[i + 1];
            FILE* f = fopen(iqpath, "rb");
            if (!f) { fprintf(stderr, "RESCORE_ERR cannot open %s\n", iqpath); return 2; }
            fseek(f, 0, SEEK_END); long bytes = ftell(f); fseek(f, 0, SEEK_SET);
            long npairs = bytes / (long)(2 * sizeof(double));
            if (npairs <= 0) { fprintf(stderr, "RESCORE_ERR empty/short %s (%ld bytes)\n", iqpath, bytes); fclose(f); return 2; }
            std::vector<std::complex<double> > iq((size_t)npairs);
            for (long k = 0; k < npairs; k++) {
                double dv[2];
                if (fread(dv, sizeof(double), 2, f) != 2) { npairs = k; break; }
                iq[(size_t)k] = std::complex<double>(dv[0], dv[1]);
            }
            fclose(f);
            cl_telecom_system ts;
            ts.operation_mode = ARQ_MODE;
            ts.load_configuration(ROBUST_0);   // config-independent ack_mfsk M=16/16-sym
            int nsymb = ts.ack_mfsk.ack_pattern_nsymb;
            int matched_nf = 0, matched_ff = 0;
            int best_off_nf = -1, best_off_ff = -1;
            uint32_t mask_nf = 0, mask_ff = 0;
            // ARM-OFF (deployed no-fine coarse): always_fine=false, no partial-tail.
            ts.ofdm.ack_allow_partial_tail = false;
            double metric_nf = ts.ofdm.detect_ack_pattern(
                iq.data(), (int)npairs, /*interp=*/1, nsymb,
                ts.ack_mfsk.ack_tones, ts.ack_mfsk.ack_pattern_len,
                ts.ack_mfsk.tone_hop_step, ts.ack_mfsk.M,
                ts.ack_mfsk.nStreams, ts.ack_mfsk.stream_offsets,
                &matched_nf, 0, nullptr, &best_off_nf, 0, &mask_nf,
                /*always_fine=*/false, /*combine_reps=*/ts.ack_mfsk.recovery_ack_reps);
            // ARM-ON (LEVER 2: fine sub-window MAX + partial-tail scoring).
            ts.ofdm.ack_allow_partial_tail = true;
            double metric_ff = ts.ofdm.detect_ack_pattern(
                iq.data(), (int)npairs, /*interp=*/1, nsymb,
                ts.ack_mfsk.ack_tones, ts.ack_mfsk.ack_pattern_len,
                ts.ack_mfsk.tone_hop_step, ts.ack_mfsk.M,
                ts.ack_mfsk.nStreams, ts.ack_mfsk.stream_offsets,
                &matched_ff, 0, nullptr, &best_off_ff, 0, &mask_ff,
                /*always_fine=*/true, /*combine_reps=*/ts.ack_mfsk.recovery_ack_reps);
            ts.ofdm.ack_allow_partial_tail = false;
            // LEVER-2 two-arm accept (the production recovery-fine gate, arq_common.cc):
            // (matched>=7 && metric>=1.5) || (matched>=5 && metric>=3.0). The coarse_metric
            // OFDM-alias class gate is an in-loop value not present in the dumped baseband,
            // so it is NOT applied here (these are PURE ACK-band baseband dumps with no
            // competing OFDM signal — the in-loop gate only adds rejections, never accepts).
            bool arm_a = (matched_ff >= 7) && (metric_ff >= 1.5);
            bool arm_b = (matched_ff >= 5) && (metric_ff >= 3.0);
            bool accept = arm_a || arm_b;
            printf("RESCORE file=%s n=%ld nsymb=%d "
                   "nofine_matched=%d nofine_metric=%.4f nofine_mask=0x%04x nofine_off=%d "
                   "fine_matched=%d fine_metric=%.4f fine_mask=0x%04x fine_off=%d "
                   "armA=%d armB=%d accept=%d\n",
                   iqpath, npairs, nsymb,
                   matched_nf, metric_nf, mask_nf & 0xFFFFu, best_off_nf,
                   matched_ff, metric_ff, mask_ff & 0xFFFFu, best_off_ff,
                   arm_a ? 1 : 0, arm_b ? 1 : 0, accept ? 1 : 0);
            fflush(stdout);
            return 0;
        }
    }

    int cpu_nr = -1;
    bool list_modes = false;
    bool list_sndcards = false;
    bool check_audio = false;
    bool nogui = false;  // GUI enabled by default

    int mod_config = CONFIG_1;
    int operation_mode = ARQ_MODE;
    int gear_shift_mode = NO_GEAR_SHIFT;
    int robust_mode = 0;  // 0=disabled, 1=enabled via -R flag
    int narrowband_mode = -1;  // -1=use INI, 0=force wideband (-W), 1=force narrowband (-N)
    int bandwidth_mode_cli = -1;  // -1=use INI, 0=BW_AUTO, 1=BW_NB_ONLY
    double bandpass_low_cli = -1.0;  // --bandpass LOW-HIGH; negative means unset
    double bandpass_high_cli = -1.0;
    int force_compress_cli = -1;  // -1=use INI, 0=off, 1=on
    int encryption_mode_cli = -1; // -1=use INI, ENCRYPT_OFF/ENCRYPT_STRICT/ENCRYPT_FAST
    char psk_hex_cli[129] = {0};  // Pre-shared key from -K flag (hex string)
    bool explicit_config = false;  // true if user specified -s
    int base_tcp_port = 0;

    int audio_system = -1;

    // ARQ settings (declared here to avoid goto crossing initialization)
    int connection_timeout_ms = 15000;
    int max_connection_attempts = 15;
    int link_timeout_ms = 30000;
    bool link_timeout_cli_given = false;
    int exit_on_disconnect = 0;
    int ldpc_iterations = 0;  // 0 = use default (50 or from INI)
    int puncture_nBits = 0;  // 0 = disabled; >0 = punctured LDPC BER test
    double tx_gain_override = -999.0;  // -999 = not set; otherwise override TX gain in dB
    double rx_gain_override = -999.0;  // -999 = not set; otherwise override RX gain in dB
    double guard_interval_ms_cli = -1.0;  // -1 = use default; > 0 = override GI in ms
    double boost_override = -1.0;     // -1 = use default; >= 0 = override NB MFSK 1S gain (-B flag)
    int nb_probe_max = -1;            // -1 = use default (2); >= 0 = override nb_probe_max
    int audio_channel_override = -1;  // -1 = use INI settings; >= 0 = override both input+output channel index
    int rx_channel_cli = -1;          // -1 = use default; 0=LEFT, 1=RIGHT, 2=STEREO
    int tx_channel_cli = -1;          // -1 = use default; 0=LEFT, 1=RIGHT, 2=STEREO
    bool monitor_stdout = false;      // --stdout: output decoded plaintext to stdout
    bool skip_turbo_reverse = false;  // --skip-turbo-reverse: skip TURBO_REVERSE phase
    bool no_optimizer_cli = false;    // --no-optimizer: disable Phase 3c effective-rate optimizer (calibration runs)
    const char* channel_lookup_path_cli = NULL; // --channel-lookup <path>: 2D channel-state lookup table (Phase 2 Step 5). NULL = OFF (default).
    int max_config_cli = -1;          // --max-config: hard ceiling on turboshift
    int ptt_delay_cli = -1;           // --ptt-delay: override both PTT on/off delays (ms)
    int radio_batch_cli = -1;         // --radio-batch: total frames per radio TX (SACK)
    int retransmit_headroom_cli = -1; // --retransmit-headroom: max retransmit frames per batch
    int skip_var_gate_cli = -1;       // --skip-var-gate=on|off: -1=default(on), 0=off, 1=on
    int wire_stamp_cli = 0;           // --wire-stamp: 0=off(default, bare wire + demod-ADD clock), 1=on (relay-stamped phase-lock; -x sim harness only)
    static char decode_rxctrl_path[1024] = ""; // PROBE X: --decode-rxctrl <file> offline control-frame decode buffer
    int phy_reinit_settle_ms_cli = -1; // --phy-reinit-settle-ms=N: -1=default(300), 0+=override
    int rx_normalize_cli = -1;         // --rx-normalize=on|off: -1=default(on), 0=off, 1=on
    int csi_llr_cli = -1;              // --csi-llr=on|off: -1=default(on), 0=off, 1=on
    int ls_nv_debug_cli = -1;          // --ls-nv-debug=on|off: -1=default(off), 0=off, 1=on (fix/cfg16-nv-restore)
    int ls_crosspilot_cli = -1;        // --ls-crosspilot-nv=on|off: -1=default(off=fix), 1=baseline A.1.4 cross-pilot
    int fsel_test_cli = -1;            // --fsel-test=on|off: -1=default(off), 0=off, 1=on (fix/cfg16-nv-restore BER freq-selective channel)
    float ber_esn0_cli = -999.0f;      // --ber-esn0=<dB>: single-point BER override (<=-900 = full sweep)
    int ber_frames_cli = 0;            // --ber-frames=<N>: frames for single-point BER (0 = default)
    double fsel_amp_cli = -1.0;        // --fsel-amp=<lin>: override 2-ray amplitude (<0 = default 0.6)
    int fsel_delay_cli = -1;           // --fsel-delay=<samples>: override 2-ray delay (<0 = default 128)
    double ack_metric_threshold_cli = -1; // --ack-metric-threshold=F: <0 = default(0.5)
    int emergency_nack_cli = -1;       // --emergency-nack=N: -1=default(3), >=0=override
    int wb_match_bias_cli = 0;         // --wb-match-threshold-bias=N: 0=HEAD, +1=revert 7076a4b 8→7
    double mean_h_gate_cli = -1;       // --mean-h-gate=F: <0=default(0.30), 0..=override
    double psk_var_floor_cli = -1;     // --psk-var-floor=F: <0=default(0.001), 0..=override
    double energy_gate_floor_cli = -1; // --energy-gate-floor=F: <0=default(1e-12), 0..=override
    int ls_window_w_cli = -1, ls_window_h_cli = -1; // --ls-window=WxH (-1 = default 2x8)
    int ofdm_defer_overflow_cli = -1;  // --ofdm-defer-overflow=on|off (-1=default on)
    int sack_timeout_extra_ms_cli = -1; // --sack-timeout-extra-ms=N (-1=default 0 post-SACK_FIX_PLAN §7 step 3)
    bool no_sack_cli = false;          // --no-sack: opt-out (SACK is ON by default since Design A shipped)
    bool enable_sack_cli = false;      // --enable-sack: no-op (kept for harness compat; SACK is ON by default)
    bool enable_sack_v2_cli = false;   // --enable-sack-v2: no-op (default ON since SACK Design A Step 14)
    bool disable_sack_v2_cli = false;  // --disable-sack-v2: SACK Design A Step 14 opt-out (forces CAP_SACK_V2 off)
    MainTestOptions test_opts;
    int  test_rsp_bsi_corrupt_at_cli = 0; // --test-rsp-bsi-corrupt-at=N: SACK Design A Step 4 synthetic discard test
    bool test_rsp_sack_rsp_crc_corrupt_cli = false; // --test-rsp-sack-rsp-crc-corrupt: SACK Design A Step 7 CRC8 fault injection (one-shot)
    int  test_rsp_sack_rsp_crc_corrupt_count_cli = 0; // --test-rsp-sack-rsp-crc-corrupt-count=N: SACK Design A Step 11 N-shot CRC8 fault injection
    int test_force_sack_mode_cli = -1;  // --force-sack-mode={off,on,probe}: SACK Design A Step 11 — set Axis-3 mode at startup
                                        // (after init, before CONNECTED). Used for Gate 4 (SACK_OFF graceful fallback in
                                        // live session) and Gate 7 (SET_LINK_PARAMS sack_mode round-trip). -1 = unset.
    int audio_buffer_ms_cli = 0;       // --alsa-buffer-ms=N (Linux only; 0 = use 30ms default)
    char log_file_path[512] = "";     // --log: tee stdout to file

    input_dev = (char *) malloc(ALSA_MAX_PATH);
    output_dev = (char *) malloc(ALSA_MAX_PATH);
    input_dev[0] = 0;
    output_dev[0] = 0;

#if defined(__linux__)
	printf("\e[0;31mMercury Version %s (build %s)\e[0m\n", VERSION__, MERCURY_BUILD_ID);
#elif defined(_WIN32)
	printf("Mercury Version %s (build %s)\n", VERSION__, MERCURY_BUILD_ID);
#endif


    // If no arguments, default to ARQ mode with GUI (when double-clicked)
    if (argc < 2)
    {
#ifdef MERCURY_GUI_ENABLED
        printf("Starting Mercury in ARQ mode with GUI...\n");
        printf("Use -h for help, -n for headless mode.\n\n");
        // Continue with defaults - ARQ mode, GUI enabled
        goto start_modem;
#else
        // No GUI build - show help
        goto manual;
#endif
    }

    if (0) {
 manual:
        printf("Usage:\n");
        printf("  %s -m [mode] [options]\n", argv[0]);
        printf("  %s -h\n\n", argv[0]);

        printf("Operating modes (-m):\n");
        printf("  ARQ             Automatic Repeat Request (primary mode)\n");
        printf("  MONITOR         Passive monitor — decode without transmitting\n");
        printf("  TX_SHM / RX_SHM Shared memory interface (see examples/)\n");
        printf("  PLOT_BASEBAND   Baseband BER simulation (AWGN)\n");
        printf("  PLOT_PASSBAND   Passband BER simulation (AWGN)\n");
        printf("  TX_TEST / RX_TEST   Test pattern transmission/reception\n");
        printf("  TX_RAND / RX_RAND   Random data transmission/reception\n");
        printf("  SIM_INPROC      In-process self-loopback feasibility prototype (no device/TCP/threads)\n");

        printf("\nDevice and audio:\n");
        printf("  -i [device]       Audio capture device (e.g. \"plughw:0,0\" or device name from -z)\n");
        printf("  -o [device]       Audio playback device\n");
        printf("  -x [api]          Sound system: alsa, pulse, dsound, wasapi, sim (default: alsa/wasapi)\n");
        printf("                    sim = device-free software channel (ARQ loopback via tools/sim/sim_channel_relay.py)\n");
        printf("                    MERCURY_STRICT_AUDIO_SUBSYSTEM=1 rejects unknown -x values\n");
        printf("  -A [channel]      Audio channel index override (enables multichannel mode)\n");
        printf("  --rx-channel [0|1|2]  RX audio channel: 0=LEFT, 1=RIGHT, 2=STEREO (default: 0)\n");
        printf("  --tx-channel [0|1|2]  TX audio channel: 0=LEFT, 1=RIGHT, 2=STEREO (default: 2)\n");
        printf("  -r [radio]        Radio type: stockhf, sbitx\n");
        printf("  -c [cpu_nr]       Pin to CPU core (-1 = auto, default)\n");
        printf("  -C                Check audio config (stereo, sample rate) before starting\n");
        printf("  -z                List available sound devices\n");

        printf("  --skip-turbo-reverse  Skip TURBO_REVERSE phase (benchmark mode)\n");
        printf("  --max-config [N]      Hard ceiling on turboshift (0-15, default: no limit)\n");
        printf("  --no-optimizer        Disable effective-rate optimizer (Phase 3c). For calibration runs only.\n");
        printf("  --channel-lookup <path>  Load 2D channel-state lookup table (observation only, Phase 2 Step 5).\n");
        printf("  --ptt-delay [ms]  Override PTT on/off delay (0 for no-PTT setups)\n");
        printf("\nModulation and bandwidth:\n");
        printf("  -s [config]       Modulation: 0-17 (OFDM), 100-102 (ROBUST MFSK), 105 (LOW48 experiment).\n");
        printf("  -g                Enable adaptive gearshift\n");
        printf("  -R                Enable ROBUST mode (MFSK weak-signal hailing)\n");
        printf("  -M [auto|nb]      Bandwidth: auto (NB hail + WB upgrade) or nb (500 Hz only)\n");
        printf("  --bandpass [LOW-HIGH]  Audio passband in Hz (0 <= LOW < HIGH <= 24000)\n");
        printf("  -N                Force narrowband mode (500 Hz, 10 subcarriers)\n");
        printf("  -W                Force wideband mode (2344 Hz, 50 subcarriers)\n");
        printf("  -I [5-50]         LDPC decoder max iterations (default: 50)\n");
        printf("  -l                List all modulation/coding modes\n");

        printf("\nARQ tuning:\n");
        printf("  -p [port]         TCP base port (control=port, data=port+1). Default: 7002\n");
        printf("  -t [ms]           Connection timeout (default: 15000)\n");
        printf("  -a [attempts]     Max connection attempts (default: 15)\n");
        printf("  -k [ms]           Link timeout (default: 30000)\n");
        printf("  -e                Exit on client disconnect\n");

        printf("\nCompression and encryption:\n");
        printf("  -F [on|off]       Force compression on/off (default: auto-detect B2F)\n");
        printf("  -E [strict|fast]  Encryption: strict (require PQ KX) or fast (classical-first)\n");
        printf("  -K [hex]          Pre-shared key for encryption (hex, up to 64 bytes)\n");

        printf("\nGain and calibration:\n");
        printf("  -T [dB]           TX gain override (e.g. -T -25.6)\n");
        printf("  --tx-level [0.0-1.0]  TX level as a linear amplitude multiplier\n");
        printf("  -G [dB]           RX gain override (e.g. -G 25.6)\n");
        printf("  -B [gain]         NB MFSK boost override\n");
        printf("  -Q [n]            NB probe max (0=disable, default 2)\n");
        printf("  --gi [ms]         Guard interval in ms (1.0-8.0, default 3.0)\n");
        printf("  --radio-batch [n] Total frames per radio TX for SACK (default: 25)\n");
        printf("  --retransmit-headroom [n]  Max retransmit frames per batch (default: 5)\n");
        printf("  --disable-sack-v2 Opt out of CAP_SACK_V2 (legacy v1 ACK behavior)\n");

        printf("\nTesting and debug:\n");
        printf("  -Z [snr_dB]       Inject AWGN noise at specified SNR\n");
        printf("  -f [hz]           TX carrier offset for frequency sync testing\n");
        printf("  -P [nBits]        Punctured LDPC BER test with specified ctrl_nBits\n");
        printf("  -v                Verbose debug output\n");
        printf("  --stdout          Output decoded plaintext to stdout (monitor mode)\n");
        printf("  --log <file>      Log all output to file (timestamped) while keeping console\n");

        printf("\nGeneral:\n");
#ifdef MERCURY_GUI_ENABLED
        printf("  -n                Disable GUI (headless mode)\n");
#endif
        printf("  -h                Print this help\n");
        return EXIT_FAILURE;
    }

    // Scan for long options (--stdout, --log) before getopt
    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--stdout") == 0)
        {
            monitor_stdout = true;
            // Remove from argv so getopt doesn't choke on it
            for (int j = i; j < argc - 1; j++)
                argv[j] = argv[j + 1];
            argc--;
            i--;
        }
        else if (strcmp(argv[i], "--tx-level") == 0)
        {
            if (i + 1 >= argc)
            {
                fprintf(stderr, "ERROR: --tx-level requires a value in the range 0.0-1.0\n");
                return EXIT_FAILURE;
            }
            double tx_level = 1.0;
            std::string error;
            if (!parse_tx_level_cli(argv[i + 1], tx_level, error))
            {
                fprintf(stderr, "ERROR: %s\n", error.c_str());
                return EXIT_FAILURE;
            }
            if (!audioio_set_tx_level(tx_level))
            {
                fprintf(stderr, "ERROR: rejected TX level after validation\n");
                return EXIT_FAILURE;
            }
            for (int j = i; j < argc - 2; j++)
                argv[j] = argv[j + 2];
            argc -= 2;
            i--;
        }
        else if (strcmp(argv[i], "--bandpass") == 0)
        {
            if (i + 1 >= argc)
            {
                fprintf(stderr, "ERROR: --bandpass requires LOW-HIGH in the range 0-24000 Hz\n");
                return EXIT_FAILURE;
            }
            std::string error;
            if (!parse_bandpass_cli(argv[i + 1], bandpass_low_cli,
                                    bandpass_high_cli, error))
            {
                fprintf(stderr, "ERROR: %s\n", error.c_str());
                return EXIT_FAILURE;
            }
            for (int j = i; j < argc - 2; j++)
                argv[j] = argv[j + 2];
            argc -= 2;
            i--;
        }
        else if (strcmp(argv[i], "--log") == 0 && i + 1 < argc)
        {
            strncpy(log_file_path, argv[i + 1], sizeof(log_file_path) - 1);
            // Remove --log and <file> from argv
            for (int j = i; j < argc - 2; j++)
                argv[j] = argv[j + 2];
            argc -= 2;
            i--;
        }
        else if (strcmp(argv[i], "--wire-stamp") == 0)
        {
            // --wire-stamp [0|1]: relay-stamped virtual-clock phase-lock for the
            // 2-process -x sim connect path (sim-arq-channel.md §10.5b/§11.2).
            // Selects the stamped RX wire format + the relay-SET clock instead of
            // the demod-ADD clock, so both peers phase-lock to ONE relay timeline
            // and the Linux FTRT drift no longer desyncs CONNECT. A bare flag
            // means ON; "--wire-stamp 0|1" sets it explicitly (matches the relay
            // / harness arg shape). Inert unless -x sim is also selected.
            int consumed = 1;
            if (i + 1 < argc && (strcmp(argv[i + 1], "0") == 0 ||
                                 strcmp(argv[i + 1], "1") == 0)) {
                wire_stamp_cli = (argv[i + 1][0] == '1') ? 1 : 0;
                consumed = 2;
            } else {
                wire_stamp_cli = 1;
            }
            for (int j = i; j < argc - consumed; j++)
                argv[j] = argv[j + consumed];
            argc -= consumed;
            i--;
        }
        else if (strcmp(argv[i], "--decode-rxctrl") == 0 && i + 1 < argc)
        {
            // PROBE X: offline replay of a dumped control-frame passband buffer
            // through receive_byte() with NO relay/threads/device — the
            // cross-platform decode arbiter. Sets operation_mode=DECODE_RXCTRL
            // (handled before audio init). Use with -s 100 (ROBUST_0 MFSK
            // control config). See sim-arq-channel.md §15.
            strncpy(decode_rxctrl_path, argv[i + 1], sizeof(decode_rxctrl_path) - 1);
            operation_mode = DECODE_RXCTRL;
            for (int j = i; j < argc - 2; j++)
                argv[j] = argv[j + 2];
            argc -= 2;
            i--;
        }
        else if (strncmp(argv[i], "--skip-var-gate=", 16) == 0)
        {
            const char* val = argv[i] + 16;
            if (strcmp(val, "off") == 0 || strcmp(val, "0") == 0) skip_var_gate_cli = 0;
            else if (strcmp(val, "on") == 0 || strcmp(val, "1") == 0) skip_var_gate_cli = 1;
            else { fprintf(stderr, "--skip-var-gate: expected on|off, got %s\n", val); exit(1); }
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--phy-reinit-settle-ms=", 23) == 0)
        {
            phy_reinit_settle_ms_cli = atoi(argv[i] + 23);
            if (phy_reinit_settle_ms_cli < 0) { fprintf(stderr, "--phy-reinit-settle-ms: must be >= 0\n"); exit(1); }
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--rx-normalize=", 15) == 0)
        {
            const char* val = argv[i] + 15;
            if (strcmp(val, "off") == 0 || strcmp(val, "0") == 0) rx_normalize_cli = 0;
            else if (strcmp(val, "on") == 0 || strcmp(val, "1") == 0) rx_normalize_cli = 1;
            else { fprintf(stderr, "--rx-normalize: expected on|off, got %s\n", val); exit(1); }
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--csi-llr=", 10) == 0)
        {
            const char* val = argv[i] + 10;
            if (strcmp(val, "off") == 0 || strcmp(val, "0") == 0) csi_llr_cli = 0;
            else if (strcmp(val, "on") == 0 || strcmp(val, "1") == 0) csi_llr_cli = 1;
            else { fprintf(stderr, "--csi-llr: expected on|off, got %s\n", val); exit(1); }
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--ls-nv-debug=", 14) == 0)
        {
            const char* val = argv[i] + 14;
            if (strcmp(val, "off") == 0 || strcmp(val, "0") == 0) ls_nv_debug_cli = 0;
            else if (strcmp(val, "on") == 0 || strcmp(val, "1") == 0) ls_nv_debug_cli = 1;
            else { fprintf(stderr, "--ls-nv-debug: expected on|off, got %s\n", val); exit(1); }
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--ls-crosspilot-nv=", 19) == 0)
        {
            const char* val = argv[i] + 19;
            if (strcmp(val, "off") == 0 || strcmp(val, "0") == 0) ls_crosspilot_cli = 0;
            else if (strcmp(val, "on") == 0 || strcmp(val, "1") == 0) ls_crosspilot_cli = 1;
            else { fprintf(stderr, "--ls-crosspilot-nv: expected on|off, got %s\n", val); exit(1); }
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--fsel-test=", 12) == 0)
        {
            const char* val = argv[i] + 12;
            if (strcmp(val, "off") == 0 || strcmp(val, "0") == 0) fsel_test_cli = 0;
            else if (strcmp(val, "on") == 0 || strcmp(val, "1") == 0) fsel_test_cli = 1;
            else { fprintf(stderr, "--fsel-test: expected on|off, got %s\n", val); exit(1); }
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--ber-esn0=", 11) == 0)
        {
            ber_esn0_cli = atof(argv[i] + 11);
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--ber-frames=", 13) == 0)
        {
            ber_frames_cli = atoi(argv[i] + 13);
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--fsel-amp=", 11) == 0)
        {
            fsel_amp_cli = atof(argv[i] + 11);
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--fsel-delay=", 13) == 0)
        {
            fsel_delay_cli = atoi(argv[i] + 13);
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--ack-metric-threshold=", 23) == 0)
        {
            ack_metric_threshold_cli = atof(argv[i] + 23);
            if (ack_metric_threshold_cli < 0) { fprintf(stderr, "--ack-metric-threshold: must be >= 0\n"); exit(1); }
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--emergency-nack=", 17) == 0)
        {
            emergency_nack_cli = atoi(argv[i] + 17);
            if (emergency_nack_cli < 1) { fprintf(stderr, "--emergency-nack: must be >= 1\n"); exit(1); }
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--wb-match-threshold-bias=", 26) == 0)
        {
            wb_match_bias_cli = atoi(argv[i] + 26);
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--mean-h-gate=", 14) == 0)
        {
            mean_h_gate_cli = atof(argv[i] + 14);
            if (mean_h_gate_cli < 0) { fprintf(stderr, "--mean-h-gate: must be >= 0\n"); exit(1); }
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--psk-var-floor=", 16) == 0)
        {
            psk_var_floor_cli = atof(argv[i] + 16);
            if (psk_var_floor_cli < 0) { fprintf(stderr, "--psk-var-floor: must be >= 0\n"); exit(1); }
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--energy-gate-floor=", 20) == 0)
        {
            energy_gate_floor_cli = atof(argv[i] + 20);
            if (energy_gate_floor_cli < 0) { fprintf(stderr, "--energy-gate-floor: must be >= 0\n"); exit(1); }
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--ls-window=", 12) == 0)
        {
            const char* val = argv[i] + 12;
            const char* x = strchr(val, 'x');
            if (!x) { fprintf(stderr, "--ls-window: expected WxH (e.g. 20x20), got %s\n", val); exit(1); }
            ls_window_w_cli = atoi(val);
            ls_window_h_cli = atoi(x + 1);
            if (ls_window_w_cli <= 0 || ls_window_h_cli <= 0) {
                fprintf(stderr, "--ls-window: W and H must be > 0\n"); exit(1);
            }
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--ofdm-defer-overflow=", 22) == 0)
        {
            const char* val = argv[i] + 22;
            if (strcmp(val, "off") == 0 || strcmp(val, "0") == 0) ofdm_defer_overflow_cli = 0;
            else if (strcmp(val, "on") == 0 || strcmp(val, "1") == 0) ofdm_defer_overflow_cli = 1;
            else { fprintf(stderr, "--ofdm-defer-overflow: expected on|off, got %s\n", val); exit(1); }
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--sack-timeout-extra-ms=", 24) == 0)
        {
            sack_timeout_extra_ms_cli = atoi(argv[i] + 24);
            if (sack_timeout_extra_ms_cli < 0) { fprintf(stderr, "--sack-timeout-extra-ms: must be >= 0\n"); exit(1); }
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--no-sack") == 0)
        {
            no_sack_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--enable-sack") == 0)
        {
            enable_sack_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--enable-sack-v2") == 0)
        {
            // SACK Design A Step 6 → Step 14: kept for harness compat
            // (sack_lossy_ab.py et al. pass this verbatim). Default is now ON
            // after Step 14, so this flag is effectively a no-op — left in
            // place to avoid breaking scripted invocations.
            enable_sack_v2_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--disable-sack-v2") == 0)
        {
            // SACK Design A Step 14: opt-out of CAP_SACK_V2 advertisement.
            // Forces CAP_SACK_V2 OFF in local_capability — TEST_CONNECTION
            // byte-for-byte matches a pre-Step-14 v1-only build. Use this
            // for v1/v2 interop testing or to roll back to v1 ACK behavior
            // on a per-instance basis without rebuilding.
            disable_sack_v2_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--test-rsp-bsi-corrupt-at=", 26) == 0)
        {
            // SACK Design A Step 4 — synthetic discard test fault injection.
            // On the Nth v2 DATA frame received by RSP, corrupt its parsed
            // batch_seq_id by adding 7 (mod 256). Falls outside both
            // current_expected and prev — must trigger the [RSP-V2-DROP]
            // branch exactly once. Default 0 = off; production builds never
            // pass this flag.
            test_rsp_bsi_corrupt_at_cli = atoi(argv[i] + 26);
            if (test_rsp_bsi_corrupt_at_cli < 0) test_rsp_bsi_corrupt_at_cli = 0;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-rsp-sack-rsp-crc-corrupt") == 0)
        {
            // SACK Design A Step 7 — synthetic CRC8 corruption test (one-shot).
            // On the first SACK_RSP OFDM frame the RSP transmits after this
            // flag is set, the trailing CRC8 byte is XOR'd with 0xFF, causing
            // the CMD-side decoder to fail the CRC check. The CMD then logs
            // [CMD-SACK-V2-CRC-FAIL] and DISCARDS the bitmap (no fabrication
            // per §9.4/A2). Default off; production builds never pass this
            // flag. The corruption clears itself after firing exactly once.
            test_rsp_sack_rsp_crc_corrupt_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--test-rsp-sack-rsp-crc-corrupt-count=", 38) == 0)
        {
            // SACK Design A Step 11 — N-shot CRC8 fault injection. Corrupts
            // the next N SACK_RSP frames the RSP transmits (vs the one-shot
            // version above). Used by Gate 3 (drive 3 misses → ON→PROBE, then
            // 2 more → PROBE→OFF) and Gate 5 (set to small N, then let SACK
            // recover → PROBE→ON).
            test_rsp_sack_rsp_crc_corrupt_count_cli = atoi(argv[i] + 38);
            if (test_rsp_sack_rsp_crc_corrupt_count_cli < 0) test_rsp_sack_rsp_crc_corrupt_count_cli = 0;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (parse_main_test_option(argc, argv, i, test_opts))
        {
        }
        else if (strncmp(argv[i], "--force-sack-mode=", 18) == 0)
        {
            const char* arg = argv[i] + 18;
            if (strcmp(arg, "off") == 0)        test_force_sack_mode_cli = 0;
            else if (strcmp(arg, "on") == 0)    test_force_sack_mode_cli = 1;
            else if (strcmp(arg, "probe") == 0) test_force_sack_mode_cli = 2;
            else { fprintf(stderr, "--force-sack-mode: expected 'off'|'on'|'probe'\n"); exit(1); }
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--alsa-buffer-ms=", 17) == 0)
        {
            audio_buffer_ms_cli = atoi(argv[i] + 17);
            if (audio_buffer_ms_cli < 3) { fprintf(stderr, "--alsa-buffer-ms: must be >= 3\n"); exit(1); }
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--rx-channel") == 0 && i + 1 < argc)
        {
            rx_channel_cli = atoi(argv[i + 1]);
            if (rx_channel_cli < 0 || rx_channel_cli > 2) rx_channel_cli = 0;
            printf("RX channel: %d (%s)\n", rx_channel_cli,
                   rx_channel_cli == 0 ? "LEFT" : rx_channel_cli == 1 ? "RIGHT" : "STEREO");
            for (int j = i; j < argc - 2; j++)
                argv[j] = argv[j + 2];
            argc -= 2;
            i--;
        }
        else if (strcmp(argv[i], "--tx-channel") == 0 && i + 1 < argc)
        {
            tx_channel_cli = atoi(argv[i + 1]);
            if (tx_channel_cli < 0 || tx_channel_cli > 2) tx_channel_cli = 0;
            printf("TX channel: %d (%s)\n", tx_channel_cli,
                   tx_channel_cli == 0 ? "LEFT" : tx_channel_cli == 1 ? "RIGHT" : "STEREO");
            for (int j = i; j < argc - 2; j++)
                argv[j] = argv[j + 2];
            argc -= 2;
            i--;
        }
        else if (strcmp(argv[i], "--gi") == 0 && i + 1 < argc)
        {
            guard_interval_ms_cli = atof(argv[i + 1]);
            if (guard_interval_ms_cli < 1.0) guard_interval_ms_cli = 1.0;
            if (guard_interval_ms_cli > 8.0) guard_interval_ms_cli = 8.0;
            printf("Guard interval override: %.2f ms (Ngi=%d)\n",
                   guard_interval_ms_cli, (int)(guard_interval_ms_cli * 12.0 + 0.5));
            // Remove --gi and <ms> from argv
            for (int j = i; j < argc - 2; j++)
                argv[j] = argv[j + 2];
            argc -= 2;
            i--;
        }
        else if (strcmp(argv[i], "--skip-turbo-reverse") == 0)
        {
            skip_turbo_reverse = true;
            printf("Turboshift: skipping REVERSE phase\n");
            for (int j = i; j < argc - 1; j++)
                argv[j] = argv[j + 1];
            argc -= 1;
            i--;
        }
        else if (strcmp(argv[i], "--no-optimizer") == 0)
        {
            // Phase 3c effective-rate optimizer kill switch. When set,
            // cl_arq_controller::opt_load_rate_table() no-ops and
            // opt_evaluate_batch_end() short-circuits before any state
            // mutation. Used by tools/effective_rate_calibrate.py so
            // calibration runs aren't disturbed by the optimizer trying
            // to switch configs mid-sweep.
            no_optimizer_cli = true;
            printf("Phase 3c optimizer: DISABLED via --no-optimizer\n");
            for (int j = i; j < argc - 1; j++)
                argv[j] = argv[j + 1];
            argc -= 1;
            i--;
        }
        else if (strcmp(argv[i], "--channel-lookup") == 0 && i + 1 < argc)
        {
            // Phase 2 Step 5 — opt-in 2D channel-state lookup. The lookup
            // is wired into the per-batch [CHANNEL-STATE] log site as
            // OBSERVATION ONLY: it emits a [CHANNEL-LOOKUP] proposal line
            // but the existing optimizer / gearshift remain in control.
            // Unlike --no-optimizer the table is user-supplied, so a
            // load failure is an error (typo / missing file) and we exit.
            // See: mercury/fact-documents/channel-state-2d-lookup.md
            channel_lookup_path_cli = argv[i + 1];
            printf("Channel-state lookup table: %s (observation mode)\n",
                   channel_lookup_path_cli);
            for (int j = i; j < argc - 2; j++)
                argv[j] = argv[j + 2];
            argc -= 2;
            i--;
        }
        else if (strcmp(argv[i], "--max-config") == 0 && i + 1 < argc)
        {
            max_config_cli = atoi(argv[i + 1]);
            if (max_config_cli < 0) max_config_cli = 0;
            if (max_config_cli > 15) max_config_cli = 15;
            printf("Turboshift: max config capped at CONFIG_%d\n", max_config_cli);
            for (int j = i; j < argc - 2; j++)
                argv[j] = argv[j + 2];
            argc -= 2;
            i--;
        }
        else if (strcmp(argv[i], "--ptt-delay") == 0 && i + 1 < argc)
        {
            ptt_delay_cli = atoi(argv[i + 1]);
            if (ptt_delay_cli < 0) ptt_delay_cli = 0;
            printf("PTT delay override: on=%d ms, off=%d ms\n", ptt_delay_cli, ptt_delay_cli);
            for (int j = i; j < argc - 2; j++)
                argv[j] = argv[j + 2];
            argc -= 2;
            i--;
        }
        else if (strcmp(argv[i], "--radio-batch") == 0 && i + 1 < argc)
        {
            radio_batch_cli = atoi(argv[i + 1]);
            if (radio_batch_cli < 5) radio_batch_cli = 5;
            if (radio_batch_cli > MAX_SACK_BATCH_SIZE) radio_batch_cli = MAX_SACK_BATCH_SIZE;
            printf("Radio batch size override: %d frames\n", radio_batch_cli);
            for (int j = i; j < argc - 2; j++)
                argv[j] = argv[j + 2];
            argc -= 2;
            i--;
        }
        else if (strcmp(argv[i], "--retransmit-headroom") == 0 && i + 1 < argc)
        {
            retransmit_headroom_cli = atoi(argv[i + 1]);
            if (retransmit_headroom_cli < 1) retransmit_headroom_cli = 1;
            if (retransmit_headroom_cli > MAX_RETRANSMIT_HEADROOM) retransmit_headroom_cli = MAX_RETRANSMIT_HEADROOM;
            printf("Retransmit headroom override: %d frames\n", retransmit_headroom_cli);
            for (int j = i; j < argc - 2; j++)
                argv[j] = argv[j + 2];
            argc -= 2;
            i--;
        }
    }

    // Set up tee logging early if --log specified (captures startup output)
    if (log_file_path[0])
        setup_tee_logging(log_file_path, argc, argv);

    int opt;
    while ((opt = getopt(argc, argv, "hc:m:s:lr:i:o:x:p:zgt:a:k:eCnf:I:RNP:vT:G:WB:Q:A:M:Z:F:E:K:")) != -1)
    {
        switch (opt)
        {
        case 'i':
            if (optarg)
                copy_audio_device_path(input_dev, optarg);
            break;
        case 'o':
            if (optarg)
                copy_audio_device_path(output_dev, optarg);
            break;
        case 'r':
            if (!strcmp(optarg, "stockhf"))
            {
                printf("Stock HF Radio Selected.\n");
                carrier_frequency_offset = 0;
                radio_type = RADIO_STOCKHF;
            }
            if (!strcmp(optarg, "sbitx"))
            {
                printf("sBitx HF Radio Selected.\n");
                carrier_frequency_offset = 15000.0;
                radio_type = RADIO_SBITX;
            }
            if (strcmp(optarg, "sbitx") && strcmp(optarg, "stockhf"))
            {
                printf("Wrong radio.\n");
                goto manual;
            }
            break;
        case 'c':
            if (optarg)
                cpu_nr = atoi(optarg);
            break;
        case 'p':
            if (optarg)
				base_tcp_port = atoi(optarg);
            break;
        case 'm':
            if (!strcmp(optarg, "ARQ"))
                operation_mode = ARQ_MODE;
            if (!strcmp(optarg, "TX_RAND"))
                operation_mode = TX_RAND;
            if (!strcmp(optarg, "RX_RAND"))
                operation_mode = RX_RAND;
            if (!strcmp(optarg, "TX_TEST"))
                operation_mode = TX_TEST;
            if (!strcmp(optarg, "RX_TEST"))
                operation_mode = RX_TEST;
            if (!strcmp(optarg, "TX_SHM"))
                operation_mode = TX_SHM;
            if (!strcmp(optarg, "RX_SHM"))
                operation_mode = RX_SHM;
            if (!strcmp(optarg, "PLOT_BASEBAND"))
                operation_mode = BER_PLOT_baseband;
            if (!strcmp(optarg, "PLOT_PASSBAND"))
                operation_mode = BER_PLOT_passband;
            if (!strcmp(optarg, "MONITOR"))
                operation_mode = MONITOR_MODE;
            if (!strcmp(optarg, "TX_WAV"))
                operation_mode = TX_WAV;
            if (!strcmp(optarg, "SIM_INPROC"))
                operation_mode = SIM_INPROC;
            break;
        case 'x':
            {
                std::string error;
                if (!parse_audio_subsystem_cli(optarg, audio_system, error))
                {
                    // Startup behavior changes are opt-in: without this flag,
                    // retain the legacy platform-default fallback.
                    const char* strict = std::getenv("MERCURY_STRICT_AUDIO_SUBSYSTEM");
                    if (strict != NULL && strict[0] != '\0' && atoi(strict) != 0)
                    {
                        fprintf(stderr, "ERROR: %s\n", error.c_str());
                        return EXIT_FAILURE;
                    }
                }
            }
            break;
        case 'g':
            gear_shift_mode = GEAR_SHIFT_ENABLED;
            break;
        case 'z':
            list_sndcards = true;
            break;
        case 's':
            if (optarg)
            {
                std::string error;
                if (!parse_modulation_cli(optarg, mod_config, error))
                {
                    fprintf(stderr, "ERROR: %s\n", error.c_str());
                    return EXIT_FAILURE;
                }
            }
            explicit_config = true;
            break;
        case 'l':
            list_modes = true;
            break;
        case 't':
            if (optarg)
                connection_timeout_ms = atoi(optarg);
            break;
        case 'a':
            if (optarg)
                max_connection_attempts = atoi(optarg);
            break;
        case 'k':
            if (optarg) {
                link_timeout_ms = atoi(optarg);
                link_timeout_cli_given = true;
            }
            break;
        case 'e':
            exit_on_disconnect = 1;
            break;
        case 'C':
            check_audio = true;
            break;
        case 'n':
            nogui = true;
            break;
        case 'f':
            if (optarg)
            {
                test_tx_carrier_offset = atof(optarg);
                printf("TX carrier offset for testing: %.2f Hz\n", test_tx_carrier_offset);
            }
            break;
        case 'I':
            if (optarg)
            {
                ldpc_iterations = atoi(optarg);
                if (ldpc_iterations < 5) ldpc_iterations = 5;
                if (ldpc_iterations > 50) ldpc_iterations = 50;
                printf("LDPC max iterations: %d\n", ldpc_iterations);
            }
            break;
        case 'P':
            if (optarg)
            {
                puncture_nBits = atoi(optarg);
                printf("Punctured LDPC BER test: ctrl_nBits=%d\n", puncture_nBits);
            }
            break;
        case 'R':
            robust_mode = 1;
            printf("Robust mode (MFSK) enabled.\n");
            break;
        case 'N':
            narrowband_mode = 1;
            printf("Narrowband mode (500 Hz) enabled.\n");
            break;
        case 'W':
            narrowband_mode = 0;
            printf("Wideband mode (2344 Hz) forced.\n");
            break;
        case 'v':
            g_verbose = 1;
            printf("Verbose debug output enabled.\n");
            break;
        case 'T':
            if (optarg)
            {
                tx_gain_override = atof(optarg);
                printf("TX gain override: %.1f dB\n", tx_gain_override);
            }
            break;
        case 'G':
            if (optarg)
            {
                rx_gain_override = atof(optarg);
                printf("RX gain override: %.1f dB\n", rx_gain_override);
            }
            break;
        case 'B':
            if (optarg)
            {
                boost_override = atof(optarg);
                printf("NB MFSK boost override: %.4f\n", boost_override);
            }
            break;
        case 'Q':
            if (optarg)
            {
                nb_probe_max = atoi(optarg);
                printf("NB probe max: %d\n", nb_probe_max);
            }
            break;
        case 'A':
            if (optarg)
            {
                audio_channel_override = atoi(optarg);
                // A multichannel device opens 16 channels, so 0..15 are the
                // only channel indices audioio can address.
                if (audio_channel_override < 0 || audio_channel_override > 15) {
                    fprintf(stderr, "ERROR: -A channel must be 0..15, got %s\n", optarg);
                    return EXIT_FAILURE;
                }
                multichannel_mode = 1;
                printf("Audio channel override: %d (multichannel mode)\n", audio_channel_override);
            }
            break;
        case 'M':
            if (optarg)
            {
                std::string bw_arg(optarg);
                if (bw_arg == "auto" || bw_arg == "0")
                    bandwidth_mode_cli = BW_AUTO;
                else if (bw_arg == "nb" || bw_arg == "1")
                    bandwidth_mode_cli = BW_NB_ONLY;
                else
                    printf("Unknown bandwidth mode '%s', use 'auto' or 'nb'\n", optarg);
                printf("Bandwidth mode: %s\n", bandwidth_mode_cli == BW_AUTO ? "auto" : "nb_only");
            }
            break;
        case 'F':
            if (optarg)
            {
                std::string fc_arg(optarg);
                if (fc_arg == "on" || fc_arg == "1")
                    force_compress_cli = 1;
                else if (fc_arg == "off" || fc_arg == "0")
                    force_compress_cli = 0;
                else
                    printf("Unknown compress mode '%s', use 'on' or 'off'\n", optarg);
                printf("Force compression: %s\n", force_compress_cli == 1 ? "on" : "off");
            }
            break;
        case 'E':
            if (optarg)
            {
                std::string error;
                if (!parse_encryption_mode_cli(optarg, encryption_mode_cli, error))
                {
                    fprintf(stderr, "ERROR: %s\n", error.c_str());
                    return EXIT_FAILURE;
                }
                printf("Encryption: %s\n", encryption_mode_cli == ENCRYPT_STRICT ? "SNDL-safe (strict)" :
                       encryption_mode_cli == ENCRYPT_FAST ? "classical-first (fast)" : "unknown");
            }
            break;
        case 'K':
            if (optarg)
            {
                strncpy(psk_hex_cli, optarg, 128);
                psk_hex_cli[128] = '\0';
                printf("PSK configured (%d hex chars)\n", (int)strlen(psk_hex_cli));
            }
            break;
        case 'Z':
            if (optarg)
            {
                noise_snr_db = atof(optarg);
                printf("AWGN noise injection: SNR=%.1f dB (ref 4kHz BW, cable=-30 dBFS)\n", noise_snr_db);
            }
            break;
        case 'h':

        default:
            goto manual;
        }
    }

start_modem:

    if (bandpass_low_cli >= 0.0)
    {
        // The stock waveform is centered at 1500 Hz. Shift that center to the
        // middle of the explicitly selected radio audio passband.
        carrier_frequency_offset = (bandpass_low_cli + bandpass_high_cli) / 2.0 - 1500.0;
        printf("Audio bandpass: %.2f-%.2f Hz (center %.2f Hz)\n",
               bandpass_low_cli, bandpass_high_cli,
               carrier_frequency_offset + 1500.0);
    }

#ifndef MERCURY_GUI_ENABLED
    nogui = true;  // Force headless if GUI not compiled in
#endif

#ifdef MERCURY_GUI_ENABLED
    // Load settings from INI file early (before audio initialization)
    {
        std::string config_path = getDefaultConfigPath();
        bool load_result = g_settings.load(config_path);
        if (load_result) {
            printf("Loaded settings from: %s\n", config_path.c_str());

            // Apply audio device settings from INI (if not overridden by command line)
            if (input_dev[0] == 0 && !g_settings.input_device.empty()) {
                copy_audio_device_path(input_dev, g_settings.input_device.c_str());
                printf("Using input device from settings: %s\n", input_dev);
            }
            if (output_dev[0] == 0 && !g_settings.output_device.empty()) {
                copy_audio_device_path(output_dev, g_settings.output_device.c_str());
                printf("Using output device from settings: %s\n", output_dev);
            }

            // Apply audio system from INI (if not overridden by command line)
            if (audio_system == -1) {
                if (g_settings.audio_system == "wasapi") {
                    audio_system = AUDIO_SUBSYSTEM_WASAPI;
                } else if (g_settings.audio_system == "dsound") {
                    audio_system = AUDIO_SUBSYSTEM_DSOUND;
                } else if (g_settings.audio_system == "alsa") {
                    audio_system = AUDIO_SUBSYSTEM_ALSA;
                } else if (g_settings.audio_system == "pulse") {
                    audio_system = AUDIO_SUBSYSTEM_PULSE;
                }
            }

            // Apply channel configuration from settings
            configured_input_channel = g_settings.input_channel;
            configured_output_channel = g_settings.output_channel;
            if (g_settings.input_channel < 0 || g_settings.input_channel > 15
                || g_settings.output_channel < 0 || g_settings.output_channel > 15) {
                fprintf(stderr, "ERROR: INI [Audio] InputChannel/OutputChannel must be 0..15, "
                        "got %d/%d\n", g_settings.input_channel, g_settings.output_channel);
                return EXIT_FAILURE;
            }
            // Override with -A flag if specified (sets both to same channel)
            if (audio_channel_override >= 0) {
                configured_input_channel = audio_channel_override;
                configured_output_channel = audio_channel_override;
            }
            // Override with --rx-channel / --tx-channel (takes precedence over -A)
            if (rx_channel_cli >= 0)
                configured_input_channel = rx_channel_cli;
            if (tx_channel_cli >= 0)
                configured_output_channel = tx_channel_cli;
            if (configured_input_channel > 2 || configured_output_channel > 2) {
                printf("Audio channels: input=%d, output=%d\n",
                       configured_input_channel, configured_output_channel);
            } else {
                printf("Audio channels: input=%s, output=%s\n",
                       configured_input_channel == 0 ? "LEFT" : configured_input_channel == 1 ? "RIGHT" : "STEREO",
                       configured_output_channel == 0 ? "LEFT" : configured_output_channel == 1 ? "RIGHT" : "STEREO");
            }

            // Apply TCP port settings from INI (if not overridden by command line)
            if (base_tcp_port == 0) {
                base_tcp_port = g_settings.control_port;
                printf("Using TCP ports from settings: control=%d, data=%d\n",
                       g_settings.control_port, g_settings.data_port);
            }

            // Surface proven, previously env-only modem features from the INI by
            // translating each into its MERCURY_* env var HERE, before the ARQ
            // controller / telecom_system are constructed and first read them
            // (break_fh_gate_enabled / turnaround_rephase_enabled_common cache on
            // first call; opt_load_rate_table reads $MERCURY_RATE_TABLE). A var
            // already present in the environment WINS over the INI so a command
            // line A/B override is never clobbered. Mirrors the _putenv_s/setenv
            // pattern at arq_commander.cc.
            {
                auto set_env = [](const char* k, const char* v) {
#if defined(_WIN32)
                    _putenv_s(k, v);
#else
                    setenv(k, v, 1);
#endif
                };
                // BREAK forward-health gate: env is a DISABLE hatch (presence =
                // disabled). Only set it when the user turned the gate OFF in the
                // GUI and didn't already set the env on the command line.
                if (!g_settings.break_fh_gate_enabled &&
                    std::getenv("MERCURY_BREAK_FH_GATE_DISABLE") == nullptr) {
                    set_env("MERCURY_BREAK_FH_GATE_DISABLE", "1");
                    printf("Feature: BREAK forward-health gate DISABLED (from INI)\n");
                }
                // Turnaround re-phase: default-ON; env "0" disables. Only set
                // when OFF in the GUI and not already overridden on the CLI.
                if (!g_settings.turnaround_rephase_enabled &&
                    std::getenv("MERCURY_TURNAROUND_REPHASE") == nullptr) {
                    set_env("MERCURY_TURNAROUND_REPHASE", "0");
                    printf("Feature: turnaround re-phase DISABLED (from INI)\n");
                }
                // Rate-table path: only when the user supplied one and the env
                // isn't already set on the CLI.
                if (!g_settings.rate_table_path.empty() &&
                    std::getenv("MERCURY_RATE_TABLE") == nullptr) {
                    set_env("MERCURY_RATE_TABLE", g_settings.rate_table_path.c_str());
                    printf("Feature: rate table path = %s (from INI)\n",
                           g_settings.rate_table_path.c_str());
                }
            }
        } else {
            printf("No settings file found, using defaults\n");
        }
    }
    // Apply -A audio channel override (even without INI file)
    if (audio_channel_override >= 0) {
        configured_input_channel = audio_channel_override;
        configured_output_channel = audio_channel_override;
        printf("Audio channel override (-A): %d\n", audio_channel_override);
    }
    // Apply --rx-channel / --tx-channel (even without INI, takes precedence over -A)
    if (rx_channel_cli >= 0) {
        configured_input_channel = rx_channel_cli;
        printf("RX channel override: %d (%s)\n", rx_channel_cli,
               rx_channel_cli == 0 ? "LEFT" : rx_channel_cli == 1 ? "RIGHT" : "STEREO");
    }
    if (tx_channel_cli >= 0) {
        configured_output_channel = tx_channel_cli;
        printf("TX channel override: %d (%s)\n", tx_channel_cli,
               tx_channel_cli == 0 ? "LEFT" : tx_channel_cli == 1 ? "RIGHT" : "STEREO");
    }
    // Set up tee logging from INI if --log wasn't specified on CLI
    if (!log_file_path[0] && g_settings.log_enabled) {
        // Auto-generate log path: %APPDATA%/Mercury/logs/ (Win) or ~/.config/mercury/logs/ (Linux)
        char logs_dir[600] = ".";
#ifdef _WIN32
        char appdata[MAX_PATH];
        if (SUCCEEDED(SHGetFolderPathA(NULL, CSIDL_APPDATA, NULL, 0, appdata))) {
            char mercury_dir[512];
            snprintf(mercury_dir, sizeof(mercury_dir), "%s\\Mercury", appdata);
            CreateDirectoryA(mercury_dir, NULL);
            snprintf(logs_dir, sizeof(logs_dir), "%s\\logs", mercury_dir);
        }
        CreateDirectoryA(logs_dir, NULL);
#else
        const char* home = getenv("HOME");
        if (home) {
            char mercury_dir[512];
            snprintf(mercury_dir, sizeof(mercury_dir), "%s/.config/mercury", home);
            mkdir(mercury_dir, 0755);
            snprintf(logs_dir, sizeof(logs_dir), "%s/logs", mercury_dir);
        }
        mkdir(logs_dir, 0755);
#endif
        // Generate timestamped filename
        time_t now = time(NULL);
        struct tm* t = localtime(&now);
        char auto_log_path[700];
        snprintf(auto_log_path, sizeof(auto_log_path), "%s/%04d%02d%02d_%02d%02d%02d.log",
                 logs_dir, t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
                 t->tm_hour, t->tm_min, t->tm_sec);
        setup_tee_logging(auto_log_path, argc, argv);
    }
    fflush(stdout);  // Ensure output is synchronized
#endif

    if (cpu_nr != -1)
    {
#if defined(__linux__)
        cpu_set_t mask;
        CPU_ZERO(&mask);
        CPU_SET(cpu_nr, &mask);
        sched_setaffinity(0, sizeof(mask), &mask);
        printf("RUNNING ON CPU Nr %d\n", sched_getcpu());
#else
        cpu_nr = -1;
#endif
    }

    // set some defaults... in case the user did not select
    if (audio_system == -1)
    {
#if defined(__linux__)
        audio_system = AUDIO_SUBSYSTEM_ALSA;
#elif defined(_WIN32)
        audio_system = AUDIO_SUBSYSTEM_WASAPI;
#endif
    }

    printf("Audio System: ");
    switch(audio_system)
    {
    case AUDIO_SUBSYSTEM_ALSA:
        if(input_dev[0] == 0)
            strcpy(input_dev, "default");
        if(output_dev[0] == 0)
            strcpy(output_dev, "default");
        printf("Advanced Linux Sound Architecture (ALSA)\n");
        break;
    case AUDIO_SUBSYSTEM_PULSE:
        if (input_dev[0] == 0)
        {
            free(input_dev);
            input_dev = NULL;
        }
        if (output_dev[0] == 0)
        {
            free(output_dev);
            output_dev = NULL;
        }
        printf("PulseAudio\n");
        break;
    case AUDIO_SUBSYSTEM_WASAPI:
        if (input_dev[0] == 0)
        {
            free(input_dev);
            input_dev = NULL;
        }
        if (output_dev[0] == 0)
        {
            free(output_dev);
            output_dev = NULL;
        }
        printf("Windows Audio Session API (WASAPI)\n");
        break;
    case AUDIO_SUBSYSTEM_DSOUND:
        if (input_dev[0] == 0)
        {
            free(input_dev);
            input_dev = NULL;
        }
        if (output_dev[0] == 0)
        {
            free(output_dev);
            output_dev = NULL;
        }
        printf("Microsoft DirectSound (DSOUND)\n");
        break;
    case AUDIO_SUBSYSTEM_SIM:
        // Device-free software channel. No real capture/playback device;
        // input/output device names are ignored (the relay is the channel).
        if (input_dev && input_dev[0] == 0)  { free(input_dev);  input_dev = NULL; }
        if (output_dev && output_dev[0] == 0) { free(output_dev); output_dev = NULL; }
        // Engage the virtual clock: from here on, every cl_timer and the
        // Q-table optimizer's opt_now_ms() measure VIRTUAL channel time
        // (samples through rx_transfer) instead of wall-clock, so the ARQ
        // control loop runs at host-compute speed. Set BEFORE audioio_init
        // starts the bridge threads / the ARQ loop touches any timer. This is
        // the ONLY place g_sim_time_enabled is ever set true; every other -x
        // mode leaves it false -> byte-identical to pre-change.
        sim_clock_set_enabled(1);
        // --wire-stamp phase-lock (sim-arq-channel.md §10.5b/§11.2): adopt the
        // relay's authoritative per-direction END-sample stamp as the shared
        // virtual clock instead of locally counting demod-consumed samples, so
        // both peers phase-lock to ONE relay timeline (drift-immune CONNECT on
        // Linux FTRT boxes). Set HERE, inside the -x sim branch ONLY — every
        // other -x mode (wasapi/alsa) leaves it 0, so HW is byte-identical. The
        // gate is read by the RX bridge (wire format) and rx_transfer (add gate),
        // both started AFTER this point by audioio_init.
        if (wire_stamp_cli)
            sim_clock_set_wire_stamp(1);
        printf("SIM software channel (device-free ARQ loopback via relay)\n");
        printf("[SIM] virtual clock ENABLED — control-loop timers run on "
               "channel-sample time, not wall-clock%s\n",
               wire_stamp_cli ? " (relay-stamped phase-lock: --wire-stamp ON)"
                              : "");
        break;
    default:
        printf("No supported audio system selected. Trying to continue.\n");
    }

    if (list_sndcards)
    {
        list_soundcards(audio_system);
        if (input_dev)
            free(input_dev);
        if (output_dev)
            free(output_dev);
        return EXIT_SUCCESS;
    }

#if defined(_WIN32)
    if (check_audio)
    {
        int result = validate_audio_config(input_dev, output_dev, audio_system);
        if (input_dev)
            free(input_dev);
        if (output_dev)
            free(output_dev);
        return (result == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
    }
#endif


    cl_telecom_system telecom_system;
    telecom_system.operation_mode = operation_mode;
    if (skip_var_gate_cli != -1) {
        telecom_system.skip_var_gate_enabled = (skip_var_gate_cli == 1);
        printf("[FLAG] --skip-var-gate=%s\n",
               telecom_system.skip_var_gate_enabled ? "on" : "off");
    }
    if (rx_normalize_cli != -1) {
        telecom_system.rx_normalize_enabled = (rx_normalize_cli == 1);
        printf("[FLAG] --rx-normalize=%s\n",
               telecom_system.rx_normalize_enabled ? "on" : "off");
    }
    if (csi_llr_cli != -1) {
        telecom_system.csi_llr_enabled = (csi_llr_cli == 1);
        printf("[FLAG] --csi-llr=%s\n",
               telecom_system.csi_llr_enabled ? "on" : "off");
    }
    if (ls_nv_debug_cli != -1) {
        // fix/cfg16-nv-restore: ofdm is a plain member of telecom_system and is
        // never reconstructed on config switch, so this persists across PHY
        // reinit. Prints [LS-NV-DBG] residual_nv vs cross-pilot_nv per LS frame.
        telecom_system.ofdm.ls_nv_debug_enabled = (ls_nv_debug_cli == 1);
        printf("[FLAG] --ls-nv-debug=%s\n",
               telecom_system.ofdm.ls_nv_debug_enabled ? "on" : "off");
    }
    if (ls_crosspilot_cli != -1) {
        telecom_system.ofdm.ls_use_crosspilot_nv = (ls_crosspilot_cli == 1);
        printf("[FLAG] --ls-crosspilot-nv=%s (on = pre-fix A.1.4 baseline arm)\n",
               telecom_system.ofdm.ls_use_crosspilot_nv ? "on" : "off");
    }
    if (fsel_amp_cli >= 0.0) telecom_system.fsel_amp = fsel_amp_cli;
    if (fsel_delay_cli >= 0) telecom_system.fsel_delay = fsel_delay_cli;
    if (fsel_test_cli != -1) {
        telecom_system.fsel_test_enabled = (fsel_test_cli == 1);
        printf("[FLAG] --fsel-test=%s (amp=%.2f delay=%d)\n",
               telecom_system.fsel_test_enabled ? "on" : "off",
               telecom_system.fsel_amp, telecom_system.fsel_delay);
    }
    if (ber_esn0_cli > -900.0f) {
        telecom_system.ber_single_esn0 = ber_esn0_cli;
        telecom_system.ber_frames_override = ber_frames_cli;
        printf("[FLAG] --ber-esn0=%.2f --ber-frames=%d\n", ber_esn0_cli, ber_frames_cli);
    }
    if (wb_match_bias_cli != 0) {
        // Apply to both mfsk instances; cl_mfsk::init() will pick up the bias
        // at the end of each init() call.
        telecom_system.mfsk.wb_match_threshold_bias = wb_match_bias_cli;
        telecom_system.ack_mfsk.wb_match_threshold_bias = wb_match_bias_cli;
        printf("[FLAG] --wb-match-threshold-bias=%d\n", wb_match_bias_cli);
    }
    if (mean_h_gate_cli >= 0) {
        telecom_system.mean_h_gate_threshold = mean_h_gate_cli;
        printf("[FLAG] --mean-h-gate=%.3f\n", telecom_system.mean_h_gate_threshold);
    }
    if (psk_var_floor_cli >= 0) {
        telecom_system.psk.var_floor = (float)psk_var_floor_cli;
        printf("[FLAG] --psk-var-floor=%.5f\n", (double)telecom_system.psk.var_floor);
    }
    if (energy_gate_floor_cli >= 0) {
        telecom_system.energy_gate_floor = energy_gate_floor_cli;
        printf("[FLAG] --energy-gate-floor=%g\n", telecom_system.energy_gate_floor);
    }
    if (ls_window_w_cli > 0 && ls_window_h_cli > 0) {
        // Override BEFORE load_configuration so it propagates to ofdm.LS_window_*.
        telecom_system.default_configurations_telecom_system.ofdm_LS_window_width  = ls_window_w_cli;
        telecom_system.default_configurations_telecom_system.ofdm_LS_window_hight = ls_window_h_cli;
        printf("[FLAG] --ls-window=%dx%d\n", ls_window_w_cli, ls_window_h_cli);
    }
    if (ofdm_defer_overflow_cli != -1) {
        telecom_system.ofdm_defer_overflow_enabled = (ofdm_defer_overflow_cli == 1);
        printf("[FLAG] --ofdm-defer-overflow=%s\n",
               telecom_system.ofdm_defer_overflow_enabled ? "on" : "off");
    }

    // Apply per-signal tx_gain overrides from INI [TxGain] section (plan §7.13.21).
    // Only non-NaN entries override; absent INI keys leave the code defaults from
    // cl_telecom_system::init_tx_gain_defaults() unchanged. Logged per override
    // so the calibration trail lives in the process log alongside [TX-GAIN].
    {
        // tx_signal_type enum is MFSK_1S=0, MFSK_2S=1, OFDM=2, ACK=3, BREAK=4
        // (telecom_system.h:53-60) — matches MercurySettings::tx_gain_override
        // [s][m] index order.
        for (int s = 0; s < MercurySettings::TX_GAIN_NSIG; s++) {
            for (int m = 0; m < MercurySettings::TX_GAIN_NMODE; m++) {
                double v = g_settings.tx_gain_override[s][m];
                if (!std::isnan(v)) {
                    telecom_system.set_tx_gain((tx_signal_type)s, m, v);
                }
            }
        }
    }

    if (list_modes)
    {
        // Print the ladder on the LIVE guard-interval basis (Ngi=36 -> Nofdm=292),
        // matching the guard interval installed at modem startup. Without this,
        // load_configuration below runs on the constructor-default gi (54/256,
        // Nofdm=310) and every printed rbc reads ~6.16% under the on-air rate.
        {
            double gi_ms = 3.0;  // production default
#ifdef MERCURY_GUI_ENABLED
            if (guard_interval_ms_cli > 0)
                gi_ms = guard_interval_ms_cli;
            else if (g_settings.guard_interval_ms != 3.0)
                gi_ms = g_settings.guard_interval_ms;
#else
            if (guard_interval_ms_cli > 0)
                gi_ms = guard_interval_ms_cli;
#endif
            if (!(gi_ms >= 1.0 && gi_ms <= 8.0)) {
                // The INI value is not range-checked by the parser; keep every
                // path inside the range the --gi parser enforces.
                printf("[GI] %.2f ms outside the 1.0-8.0 ms range, clamped\n", gi_ms);
                gi_ms = (gi_ms > 8.0) ? 8.0 : 1.0;
            }
            int ngi = (int)(gi_ms * 12.0 + 0.5);  // 12kHz OFDM rate
            telecom_system.default_configurations_telecom_system.ofdm_gi = (float)ngi / 256.0f;
        }
        for (int i = 0; i < NUMBER_OF_CONFIGS; i++)
        {
            telecom_system.load_configuration(i);
            printf("CONFIG_%d (%f bps), frame_size: %d Bytes / %d bits / %d non-byte-aligned bits\n", i,
                   telecom_system.rbc, telecom_system.get_frame_size_bytes(),
                   telecom_system.get_frame_size_bits(), telecom_system.get_frame_size_bits() - (telecom_system.get_frame_size_bytes() * 8));
        }
        return EXIT_SUCCESS;
    }


    if ((mod_config >= NUMBER_OF_CONFIGS && !is_robust_config(mod_config)
			&& !is_low48_anchor_config(mod_config)) || (mod_config < 0)
			|| (is_low48_anchor_config(mod_config)
				&& (!explicit_config || operation_mode != BER_PLOT_passband)))
    {
        printf("Wrong modulation config %d\n", mod_config);
        exit(EXIT_FAILURE);
    }

    // Set narrowband mode on telecom_system for all modes (ARQ sets it again below)
    telecom_system.narrowband_enabled = (narrowband_mode == 1) ? YES : NO;

    // Apply -B boost override to NB gain table entries
    if (boost_override >= 0.0)
    {
        double ratio_2s = telecom_system.tx_gain[TX_SIG_MFSK_2S][1][1] /
                          (telecom_system.tx_gain[TX_SIG_MFSK_1S][1][1] + 1e-30);
        telecom_system.tx_gain[TX_SIG_MFSK_1S][1][0] = boost_override;
        telecom_system.tx_gain[TX_SIG_MFSK_1S][1][1] = boost_override;
        telecom_system.tx_gain[TX_SIG_MFSK_2S][1][0] = boost_override * ratio_2s;
        telecom_system.tx_gain[TX_SIG_MFSK_2S][1][1] = boost_override * ratio_2s;
        telecom_system.tx_gain[TX_SIG_ACK][1][0] = boost_override;
        telecom_system.tx_gain[TX_SIG_ACK][1][1] = boost_override;
        telecom_system.tx_gain[TX_SIG_BREAK][1][0] = boost_override;
        telecom_system.tx_gain[TX_SIG_BREAK][1][1] = boost_override;
        printf("[TX-GAIN] Override: NB MFSK_1S=%.4f  MFSK_2S=%.4f  ACK/BREAK=%.4f\n",
               boost_override, boost_override * ratio_2s, boost_override);
    }

    // PROBE X — DECODE_RXCTRL: offline cross-platform control-frame decode arbiter.
    // Reads a raw little-endian float64 passband buffer (dumped by the live
    // receive_byte() instrumentation under MERCURY_DUMP_RXCTRL) and replays it
    // through the SAME receive_byte() math with NO relay/threads/device/timing.
    // Reports message_decoded + crc + decoded-byte md5. Run with -s 100 (ROBUST_0
    // MFSK control config). The 2x2 (Windows-buffer x {Win,Linux decoder};
    // Linux-buffer x {Win,Linux decoder}) decisively separates ACQUISITION/POSITION
    // (decode succeeds on the SAME good samples on both OSes) from PLATFORM-DECODE
    // (the SAME bytes decode differently). See sim-arq-channel.md §15.
    if (telecom_system.operation_mode == DECODE_RXCTRL)
    {
        printf("Mode selected: DECODE_RXCTRL (offline control-frame decode arbiter)\n");
        // Read the self-describing RXC2 header (16 int32) carrying the COMPLETE
        // capture geometry so we reproduce the EXACT decoder state with no guessing.
        FILE* df = fopen(decode_rxctrl_path, "rb");
        if(!df) { fprintf(stderr, "[DRX] FATAL: cannot open %s\n", decode_rxctrl_path); return 2; }
        int32_t hdr[16] = {0};
        size_t hgot = fread(hdr, sizeof(int32_t), 16, df);
        if(hgot != 16 || hdr[0] != (int32_t)0x32435852) {
            fprintf(stderr, "[DRX] FATAL: bad/absent RXC2 header (magic=0x%08x)\n", (unsigned)hdr[0]);
            fclose(df); return 2;
        }
        int file_Nofdm=hdr[1], file_Nsymb=hdr[2], file_interp=hdr[3];
        int file_nb=hdr[4], file_cfg=hdr[5], file_dlen=hdr[6];
        int file_Nfft=hdr[7], file_Ngi=hdr[8], file_Nc=hdr[9];
        int file_pre=hdr[10], file_frmNsymb=hdr[11];
        int file_mM=hdr[12], file_mStreams=hdr[13];
        printf("[DRX] header: Nofdm=%d buffer_Nsymb=%d interp=%d nb=%d cfg=%d dlen=%d "
               "Nfft=%d Ngi=%d Nc=%d pre=%d Nsymb=%d mfsk_M=%d nStreams=%d\n",
               file_Nofdm, file_Nsymb, file_interp, file_nb, file_cfg, file_dlen,
               file_Nfft, file_Ngi, file_Nc, file_pre, file_frmNsymb, file_mM, file_mStreams);
        if (file_Nfft <= 0 || file_Nfft > 256 || (file_Nfft % 4) != 0
            || file_Ngi < 1 || file_Ngi > file_Nfft) {
            // The OFDM layer needs 1 <= Ngi <= Nfft, Nfft a multiple of 4, <= 256.
            fprintf(stderr, "[DRX] FATAL: capture geometry out of range (Nfft=%d Ngi=%d)\n",
                    file_Nfft, file_Ngi);
            fclose(df); return 2;
        }
        // Reproduce the EXACT capture geometry. The live RSP's ROBUST_0 MFSK uses
        // gi=Ngi/Nfft (e.g. 36/256) which differs from the standalone default
        // (54/256). Force the default ofdm geometry to the captured values BEFORE
        // load_configuration, match narrowband, and enlarge the allocation, so
        // set_size() produces a byte-identical decoder layout.
        telecom_system.narrowband_enabled = file_nb ? YES : NO;
        telecom_system.default_configurations_telecom_system.ofdm_Nfft = file_Nfft;
        telecom_system.default_configurations_telecom_system.ofdm_gi   = (float)file_Ngi/(float)file_Nfft;
        telecom_system.data_container.buffer_Nsymb_min = file_Nsymb + 8;
        // Force a real (non-no-op) reload: load_configuration early-returns when the
        // requested config == current_configuration. current starts at CONFIG_NONE.
        telecom_system.load_configuration(mod_config);
        int dc_buffer_Nsymb = telecom_system.data_container.buffer_Nsymb.load();
        // Pin the EXACT captured symbol count so receive_byte's scan span matches.
        telecom_system.data_container.buffer_Nsymb.store(file_Nsymb);
        printf("[DRX] config=%d M=%.0f Nofdm=%d buffer_Nsymb=%d(was %d) interp=%d Nfft=%d Ngi=%d Nc=%d narrowband=%d\n",
               mod_config, telecom_system.M,
               telecom_system.data_container.Nofdm,
               file_Nsymb, dc_buffer_Nsymb,
               telecom_system.frequency_interpolation_rate,
               telecom_system.data_container.Nfft, telecom_system.data_container.Ngi,
               telecom_system.data_container.Nc,
               (int)telecom_system.narrowband_enabled);
        if(telecom_system.data_container.Nofdm != file_Nofdm) {
            fprintf(stderr, "[DRX] WARNING: live Nofdm %d != captured %d — geometry mismatch; "
                    "results may be invalid.\n", telecom_system.data_container.Nofdm, file_Nofdm);
        }
        int use_len = file_dlen;
        printf("[DRX] file=%s decode_len=%d\n", decode_rxctrl_path, use_len);
        double* buf = (double*)calloc((size_t)(use_len > 0 ? use_len : 1), sizeof(double));
        size_t got = fread(buf, sizeof(double), (size_t)(use_len > 0 ? use_len : 0), df);
        fclose(df);
        // input checksum (independent of platform — same file => same numbers)
        double in_e = 0, in_pk = 0;
        for(int i=0;i<use_len;i++){ double v=buf[i]; in_e+=v*v; if(fabs(v)>in_pk)in_pk=fabs(v); }
        printf("[DRX] read_doubles=%zu in_rms=%.12f in_peak=%.12f\n",
               got, sqrt(in_e/(use_len>0?use_len:1)), in_pk);
        // The decode oracle — ONE receive_byte() call, identical math to the live path.
        int* out_bits = (int*)calloc(N_MAX, sizeof(int));
        st_receive_stats rs = telecom_system.receive_byte(buf, out_bits);
        // checksum of decoded info bytes (so a same-bytes/same-result check is byte-exact)
        unsigned int byte_sum = 0; int nbytes = 0;
        for(int i=0;i<N_MAX/8;i++){ int b = telecom_system.data_container.hd_decoded_data_byte[i];
            byte_sum = byte_sum*131 + (unsigned)(b & 0xFF); }
        (void)nbytes;
        printf("[DRX-RESULT] message_decoded=%d crc=%d SNR=%.3f coarse_metric=%.6f "
               "iterations=%d delay=%d sync_trials=%d byte_hash=%u\n",
               (rs.message_decoded==YES)?1:0, rs.crc, rs.SNR, rs.coarse_metric,
               rs.iterations_done, rs.delay, rs.sync_trials, byte_sum);
        fflush(stdout);
        free(buf); free(out_bits);
        return (rs.message_decoded==YES) ? 0 : 1;
    }

    // SIM_INPROC: single-process in-process self-loopback feasibility prototype.
    // Additive one-shot — runs ONE telecom_system + arq_controller in-process
    // with a single-thread step-pumped stepper. NO audio device, NO bridge/prep
    // threads, NO TCP server, NO relay. Firewall-safe (no sockets) and
    // audio-safe (no device). Proves the TX-path spin-loops become
    // step-pumpable without a concurrent drainer while preserving spin-exit
    // timing. See fact-documents/single-process-sim-refactor.md. Exits rc.
    if (telecom_system.operation_mode == SIM_INPROC)
    {
        // The normal ARQ startup applies --gi below, immediately before the
        // production PHY is initialized.  SIM_INPROC returns before reaching
        // that block and its two private telecom_system instances therefore
        // used the constructor GI (Ngi=54) even when --gi was explicit.  Pass
        // the already-validated CLI value through the simulator's existing
        // environment parameter seam.  With --gi absent no variable is set and
        // the historical SIM_INPROC path is byte-identical.
        if (guard_interval_ms_cli > 0)
        {
            char sim2_gi_ms[32];
            snprintf(sim2_gi_ms, sizeof(sim2_gi_ms), "%.9g",
                     guard_interval_ms_cli);
#if defined(_WIN32)
            _putenv_s("MERCURY_SIM2_GI_MS", sim2_gi_ms);
#else
            setenv("MERCURY_SIM2_GI_MS", sim2_gi_ms, 1);
#endif
            printf("[SIM_INPROC] honoring --gi %.2f ms in private modem instances\n",
                   guard_interval_ms_cli);
            fflush(stdout);
        }
        // §10.5: with MERCURY_SIM_2INST=1 (or --sim-2inst, parsed earlier into
        // the same env-style toggle) run the 2-INSTANCE lockstep stepper; default
        // runs the single-instance Stage-2 GO/NO-GO prototype (preserved as the
        // regression). Both are additive, in-process, no device/TCP/threads.
        const char* two = getenv("MERCURY_SIM_2INST");
        bool run_2inst = (two && *two && *two != '0');
        if (run_2inst)
        {
            printf("Mode selected: SIM_INPROC (2-instance lockstep stepper)\n");
            fflush(stdout);
            int rc = cl_arq_controller::test_sim_inproc_2();
            printf("[FLAG] SIM_INPROC 2-instance complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            return rc;
        }
        printf("Mode selected: SIM_INPROC (in-process self-loopback prototype)\n");
        fflush(stdout);
        cl_arq_controller ARQ;
        ARQ.telecom_system = &telecom_system;
        int rc = ARQ.test_sim_inproc();
        printf("[FLAG] SIM_INPROC prototype complete (rc=%d) — exiting.\n", rc);
        fflush(stdout);
        return rc;
    }

    // initializing audio system
    pthread_t radio_capture, radio_playback, radio_capture_prep;
    auto start_audio_or_fail = [&]() {
        if(audioio_init_internal(input_dev, output_dev, audio_system, &radio_capture,
                                 &radio_playback, &radio_capture_prep, &telecom_system) != 0)
        {
            fprintf(stderr, "ERROR: audio device startup failed; modem startup aborted\n");
            main_exit_status = EXIT_FAILURE;
            // Not every init failure path stops the audio threads itself (the
            // payload-ring failures return without it). Every mode loop tests
            // shutdown_ before its first pass, so no loop body runs after this.
            shutdown_.store(true);
        }
    };

    if (telecom_system.operation_mode == MONITOR_MODE)
        telecom_system.operation_mode = ARQ_MODE;  // Reuse ARQ infrastructure

    if (telecom_system.operation_mode == ARQ_MODE)
    {
        bool is_monitor_mode = (operation_mode == MONITOR_MODE);
        if (is_monitor_mode)
            printf("Mode selected: MONITOR (passive third-party decode)\n");
        else
            printf("Mode selected: ARQ\n");
        cl_arq_controller ARQ;
        ARQ.telecom_system = &telecom_system;
        // CONTINUOUS-KEYDOWN production setters (real -m ARQ path). Both peers set these.
        telecom_system.preamble_amortization_enabled = true;   // DEFAULT-ON: continuous-keydown amortized preamble
        if(const char* kd_e=std::getenv("MERCURY_PREAMBLE_AMORT")) { if(atoi(kd_e)==0) { telecom_system.preamble_amortization_enabled = false; printf("[FLAG] MERCURY_PREAMBLE_AMORT=0: preamble amortization DISABLED\n"); fflush(stdout); } }
        telecom_system.keydown_track_timing_enabled = true;   // DEFAULT-ON: continuous-keydown carried timing
        if(const char* kt_e=std::getenv("MERCURY_KEYDOWN_TRACK")) { if(atoi(kt_e)==0) { telecom_system.keydown_track_timing_enabled = false; printf("[FLAG] MERCURY_KEYDOWN_TRACK=0: continuous-keydown carried-timing DISABLED\n"); fflush(stdout); } }
        printf("[FLAG] preamble_amortization=%s keydown_track=%s (link-wide: both peers must match)\n",
               telecom_system.preamble_amortization_enabled ? "on" : "off",
               telecom_system.keydown_track_timing_enabled ? "on" : "off");
        ARQ.passive_monitor = is_monitor_mode;
        ARQ.monitor_stdout = is_monitor_mode && monitor_stdout;
        if (phy_reinit_settle_ms_cli != -1) {
            ARQ.phy_reinit_settle_us = phy_reinit_settle_ms_cli * 1000;
            printf("[FLAG] --phy-reinit-settle-ms=%d (us=%d)\n",
                   phy_reinit_settle_ms_cli, ARQ.phy_reinit_settle_us);
        }
        if (ack_metric_threshold_cli >= 0) {
            ARQ.ack_metric_threshold = ack_metric_threshold_cli;
            printf("[FLAG] --ack-metric-threshold=%.3f\n", ARQ.ack_metric_threshold);
        }
        if (emergency_nack_cli >= 1) {
            ARQ.emergency_nack_threshold = emergency_nack_cli;
            printf("[FLAG] --emergency-nack=%d\n", ARQ.emergency_nack_threshold);
        }
        if (sack_timeout_extra_ms_cli >= 0) {
            ARQ.sack_timeout_extra_ms = sack_timeout_extra_ms_cli;
            printf("[FLAG] --sack-timeout-extra-ms=%d\n", ARQ.sack_timeout_extra_ms);
        }
        if (audio_buffer_ms_cli > 0) {
            g_audio_buffer_ms_override = audio_buffer_ms_cli;
            printf("[FLAG] --alsa-buffer-ms=%d (Linux only)\n", audio_buffer_ms_cli);
        }
        if (no_sack_cli) {
            // CAP_SACK + CAP_SACK_V2 removed from wire — the flag now just
            // sets the local disable_sack which the negotiation block in
            // arq_responder.cc / arq_commander.cc reads to take SACK offline.
            ARQ.disable_sack = true;
            ARQ.enable_sack_v2 = false;
            printf("[FLAG] --no-sack: SACK disabled locally\n");
        }
        if (enable_sack_cli) {
            ARQ.disable_sack = false;
            printf("[FLAG] --enable-sack: no-op (SACK is ON by default since Design A)\n");
        }
        if (no_sack_cli && enable_sack_cli) {
            fprintf(stderr, "ERROR: cannot pass both --no-sack and --enable-sack\n");
            exit(1);
        }
        if (enable_sack_v2_cli) {
            printf("[FLAG] --enable-sack-v2: no-op (CAP_SACK_V2 removed; v2 always on)\n");
        }
        if (disable_sack_v2_cli) {
            // Wire CAP bit is gone — disabling v2 now means disabling SACK entirely.
            ARQ.enable_sack_v2 = false;
            ARQ.disable_sack = true;
            printf("[FLAG] --disable-sack-v2: SACK disabled locally "
                   "(CAP_SACK_V2 removed; behaves like --no-sack)\n");
        }
        if (test_rsp_bsi_corrupt_at_cli > 0) {
            ARQ.test_rsp_bsi_corrupt_at = test_rsp_bsi_corrupt_at_cli;
            printf("[FLAG] --test-rsp-bsi-corrupt-at=%d: will corrupt the %dth v2 "
                   "DATA frame's batch_seq_id by +7 mod 256 (SACK Design A Step 4 "
                   "synthetic discard test — one-shot)\n",
                   test_rsp_bsi_corrupt_at_cli, test_rsp_bsi_corrupt_at_cli);
        }
        if (test_rsp_sack_rsp_crc_corrupt_cli) {
            ARQ.test_rsp_sack_rsp_crc_corrupt = true;
            ARQ.test_rsp_sack_rsp_crc_corrupt_armed = true;
            printf("[FLAG] --test-rsp-sack-rsp-crc-corrupt: next SACK_RSP TX will have "
                   "CRC8 XOR'd with 0xFF (SACK Design A Step 7 synthetic CRC8 fault "
                   "injection — one-shot)\n");
        }
        if (test_force_sack_mode_cli >= 0) {
            // SACK Design A Step 11 — pre-CONNECTED override of axis3_sack_mode.
            // Used by Gates 4 (SACK_OFF graceful fallback) and 7 (SET_LINK_PARAMS
            // sack_mode round-trip). When CMD's Axis-2 controller later fires
            // a SET_LINK_PARAMS for any reason, the carried sack_mode will be
            // this value (per the pending_link_params_sack_mode = axis3_sack_mode
            // assignment in policy_evaluate_axis2). RSP applies it through
            // the existing SET_LINK_PARAMS handler.
            ARQ.axis3_sack_mode = test_force_sack_mode_cli;
            printf("[FLAG] --force-sack-mode=%s: pre-CONNECTED Axis-3 mode override "
                   "(SACK Design A Step 11 testing)\n",
                   test_force_sack_mode_cli == 0 ? "OFF"
                   : test_force_sack_mode_cli == 1 ? "ON"
                   : "PROBE");
        }
        if (test_rsp_sack_rsp_crc_corrupt_count_cli > 0) {
            ARQ.test_rsp_sack_rsp_crc_corrupt_count = test_rsp_sack_rsp_crc_corrupt_count_cli;
            printf("[FLAG] --test-rsp-sack-rsp-crc-corrupt-count=%d: next %d SACK_RSP TXs "
                   "will have CRC8 XOR'd with 0xFF (SACK Design A Step 11 N-shot CRC8 "
                   "fault injection)\n",
                   test_rsp_sack_rsp_crc_corrupt_count_cli,
                   test_rsp_sack_rsp_crc_corrupt_count_cli);
        }
        run_main_post_tests(ARQ, telecom_system, test_opts);

        // Monitor mode: force monitor on, disable TX
        if (is_monitor_mode) {
#ifdef MERCURY_GUI_ENABLED
            g_gui_state.monitor_enabled.store(true);
#endif
        }

#ifdef MERCURY_GUI_ENABLED
        // Apply PTT timing settings from INI before init
        ARQ.default_configuration_ARQ.ptt_on_delay_ms = g_settings.ptt_on_delay_ms;
        ARQ.default_configuration_ARQ.ptt_off_delay_ms = g_settings.ptt_off_delay_ms;
        ARQ.default_configuration_ARQ.pilot_tone_ms = g_settings.pilot_tone_ms;
        ARQ.default_configuration_ARQ.pilot_tone_hz = g_settings.pilot_tone_hz;
        ARQ.default_configuration_ARQ.link_timeout = g_settings.link_timeout_ms;
        printf("PTT timing: on_delay=%dms, off_delay=%dms, pilot=%dms@%dHz\n",
               g_settings.ptt_on_delay_ms, g_settings.ptt_off_delay_ms,
               g_settings.pilot_tone_ms, g_settings.pilot_tone_hz);
#endif

        // CLI --ptt-delay overrides INI PTT timing
        // -k takes the same pre-init channel as the INI value, so init and the
        // per-configuration floor in load_configuration both see it.
        if (link_timeout_cli_given)
            ARQ.default_configuration_ARQ.link_timeout = link_timeout_ms;

        if (ptt_delay_cli >= 0)
        {
            ARQ.default_configuration_ARQ.ptt_on_delay_ms = ptt_delay_cli;
            ARQ.default_configuration_ARQ.ptt_off_delay_ms = ptt_delay_cli;
            printf("PTT delay CLI override: on=%d ms, off=%d ms\n", ptt_delay_cli, ptt_delay_cli);
        }

        // CLI --radio-batch and --retransmit-headroom override SACK defaults
        if (radio_batch_cli >= 0)
        {
            ARQ.radio_batch_size = radio_batch_cli;
            if (retransmit_headroom_cli >= 0)
                ARQ.retransmit_headroom = retransmit_headroom_cli;
            ARQ.crypto_batch_size = ARQ.radio_batch_size - ARQ.retransmit_headroom;
            if (ARQ.crypto_batch_size < 1) ARQ.crypto_batch_size = 1;
            printf("SACK CLI: radio_batch=%d crypto_batch=%d headroom=%d\n",
                ARQ.radio_batch_size, ARQ.crypto_batch_size, ARQ.retransmit_headroom);
        }
        else if (retransmit_headroom_cli >= 0)
        {
            ARQ.retransmit_headroom = retransmit_headroom_cli;
            ARQ.crypto_batch_size = ARQ.radio_batch_size - ARQ.retransmit_headroom;
            if (ARQ.crypto_batch_size < 1) ARQ.crypto_batch_size = 1;
            printf("SACK CLI: radio_batch=%d crypto_batch=%d headroom=%d\n",
                ARQ.radio_batch_size, ARQ.crypto_batch_size, ARQ.retransmit_headroom);
        }

        // Apply LDPC iterations: CLI overrides INI, INI overrides default
#ifdef MERCURY_GUI_ENABLED
        if (ldpc_iterations > 0)
            telecom_system.default_configurations_telecom_system.ldpc_nIteration_max = ldpc_iterations;
        else if (g_settings.ldpc_iterations_max != 50)
            telecom_system.default_configurations_telecom_system.ldpc_nIteration_max = g_settings.ldpc_iterations_max;
        g_gui_state.ldpc_iterations_max.store(telecom_system.default_configurations_telecom_system.ldpc_nIteration_max);
        telecom_system.coarse_freq_sync_enabled = g_settings.coarse_freq_sync_enabled;
        g_gui_state.coarse_freq_sync_enabled.store(g_settings.coarse_freq_sync_enabled);
        // Robust mode: CLI -R overrides INI setting
        // CLI values are run-only: they go to g_gui_state and ARQ, never into
        // g_settings, which holds only what the user saved.
        g_gui_state.robust_mode_enabled.store(g_settings.robust_mode_enabled || robust_mode);
        // All stations always start NB — bandwidth_mode controls WB upgrade.
        // CLI -N/-W still available for BER testing but ignored for ARQ.
        g_gui_state.narrowband_enabled.store(true);
        // Bandwidth mode: CLI -M overrides INI setting
        const int bandwidth_setting = (bandwidth_mode_cli >= 0) ? bandwidth_mode_cli
                                                                : g_settings.bandwidth_mode;
        g_gui_state.bandwidth_mode.store(bandwidth_setting);
        // Initialize GUI gain state from INI (needed even with -n nogui,
        // since gui_apply_tx_gain/rx_gain read from g_gui_state always)
        g_gui_state.tx_gain_db.store(g_settings.tx_gain_db);
        g_gui_state.rx_gain_db.store(g_settings.rx_gain_db);
        g_gui_state.gains_locked.store(g_settings.gains_locked);
        printf("[TX-GAIN] INI: %.1f dB  [RX-GAIN] INI: %.1f dB\n",
               g_settings.tx_gain_db, g_settings.rx_gain_db);
        // TX gain override from -T flag (temporary, not saved to INI)
        if (tx_gain_override > -900.0) {
            g_gui_state.tx_gain_db.store(tx_gain_override);
            g_gui_state.gains_locked.store(true);
            printf("TX gain set to %.1f dB (signal at ~%.1f dBFS)\n",
                   tx_gain_override, -4.4 + tx_gain_override);
        }
        // RX gain override from -G flag (temporary, not saved to INI)
        if (rx_gain_override > -900.0) {
            g_gui_state.rx_gain_db.store(rx_gain_override);
            g_gui_state.gains_locked.store(true);
            printf("RX gain set to %.1f dB\n", rx_gain_override);
        }
#else
        if (ldpc_iterations > 0)
            telecom_system.default_configurations_telecom_system.ldpc_nIteration_max = ldpc_iterations;
#endif

        // RX digital gain (works in both GUI and headless builds)
        {
            extern double rx_gain_linear;  // defined in audioio.c
            double rx_gain_db = 0.0;
#ifdef MERCURY_GUI_ENABLED
            rx_gain_db = g_gui_state.rx_gain_db.load();
#endif
            if (rx_gain_override > -900.0)
                rx_gain_db = rx_gain_override;
            if (rx_gain_db != 0.0) {
                rx_gain_linear = pow(10.0, rx_gain_db / 20.0);
                printf("[RX-GAIN] %.1f dB (linear=%.4f)\n", rx_gain_db, rx_gain_linear);
            }
        }

        // Apply guard interval: CLI --gi overrides INI, INI overrides default (3.0ms)
        {
            double gi_ms = 3.0;  // default
#ifdef MERCURY_GUI_ENABLED
            if (guard_interval_ms_cli > 0)
                gi_ms = guard_interval_ms_cli;
            else if (g_settings.guard_interval_ms != 3.0)
                gi_ms = g_settings.guard_interval_ms;
#else
            if (guard_interval_ms_cli > 0)
                gi_ms = guard_interval_ms_cli;
#endif
            if (!(gi_ms >= 1.0 && gi_ms <= 8.0)) {
                // The INI value is not range-checked by the parser; keep every
                // path inside the range the --gi parser enforces.
                printf("[GI] %.2f ms outside the 1.0-8.0 ms range, clamped\n", gi_ms);
                gi_ms = (gi_ms > 8.0) ? 8.0 : 1.0;
            }
            int ngi = (int)(gi_ms * 12.0 + 0.5);  // 12kHz OFDM rate
            telecom_system.default_configurations_telecom_system.ofdm_gi = (float)ngi / 256.0f;
            printf("Guard interval: %.2f ms (Ngi=%d, gi=%.4f)\n", gi_ms, ngi,
                   telecom_system.default_configurations_telecom_system.ofdm_gi);
        }

        // Apply GUI settings: gearshift and initial config from INI
#ifdef MERCURY_GUI_ENABLED
        if (!explicit_config) {
            if (g_settings.gear_shift_enabled)
                gear_shift_mode = GEAR_SHIFT_ENABLED;
            mod_config = g_settings.initial_config;
            if (is_robust_config(mod_config))
                robust_mode = 1;
        }
#endif
        // CLI/headless gearshift with no explicit -s: default to ROBUST_0 and enable
        // robust mode. GUI builds intentionally do NOT force ROBUST_0 here — the GUI
        // block above sets mod_config from g_settings.initial_config so GUI users can
        // configure their own start config. That INI default is ROBUST_0
        // (ini_parser.cc) so a GUI user who hasn't changed it still starts at the
        // floor. Do NOT remove the #ifndef — it is the GUI-configurability seam.
        // See fact-documents/gearshift-start-and-recovery.md §2 Bug 1.
        if(gear_shift_mode != NO_GEAR_SHIFT && !explicit_config)
        {
#ifndef MERCURY_GUI_ENABLED
            mod_config = ROBUST_0;
#endif
            robust_mode = 1;
        }

        // Robust mode: CLI -R or INI setting enables MFSK hailing
#ifdef MERCURY_GUI_ENABLED
        ARQ.robust_enabled = (g_settings.robust_mode_enabled || robust_mode) ? YES : NO;
        ARQ.bandwidth_mode = bandwidth_setting;
        // -Q 0 with auto mode: skip NB start, go directly to WB.
        // Both sides are controlled (benchmark/test), no NB probe needed.
        if (nb_probe_max == 0 && ARQ.bandwidth_mode == BW_AUTO)
            ARQ.narrowband_enabled = NO;
        else
            ARQ.narrowband_enabled = YES;  // Normal: start NB, negotiate WB via probe
        // Pre-dates SACK / streaming-compression default-on. Include the full
        // current default cap set so main.cc's rewrite doesn't silently strip
        // bits the constructor (arq_common.cc:299-302) just set. SACK / v2
        // can still be opt-out via --no-sack / --disable-sack-v2 (those flags
        // run later and mask the bits at main.cc:1505-1534).
        ARQ.local_capability = ((ARQ.bandwidth_mode == BW_AUTO) ? CAP_WB_CAPABLE : 0)
                             | CAP_RETX_TURN_TAIL;
        ARQ.force_compress = (force_compress_cli >= 0) ? (force_compress_cli == 1) : g_settings.force_compress;
        ARQ.skip_turbo_reverse = skip_turbo_reverse;
        ARQ.max_config_override = max_config_cli;
        ARQ.set_optimizer_disabled(no_optimizer_cli);
        // Phase 2 Step 5 — load 2D channel-state lookup table if --channel-lookup
        // path was supplied. User-supplied path => load failure is fatal (typo
        // / missing file). When the flag is unset the lookup stays in the
        // "not loaded" state and the commander emits no [CHANNEL-LOOKUP] line.
        if (channel_lookup_path_cli != NULL) {
            if (!ARQ.channel_lookup.init_from_json(channel_lookup_path_cli)) {
                fprintf(stderr, "ERROR: --channel-lookup load failed: %s (path=%s)\n",
                        ARQ.channel_lookup.last_error(), channel_lookup_path_cli);
                exit(1);
            }
            printf("[CHANNEL-LOOKUP] loaded %d cells from %s\n",
                   ARQ.channel_lookup.n_cells(), channel_lookup_path_cli);
            fflush(stdout);
        }
        // Encryption: CLI -E overrides INI setting
        ARQ.encryption_mode = (encryption_mode_cli >= 0) ? encryption_mode_cli : g_settings.encryption_mode;
        if (ARQ.encryption_mode != ENCRYPT_OFF)
            ARQ.local_capability |= CAP_ENCRYPTION;
        g_gui_state.encryption_mode.store(ARQ.encryption_mode);
        if (!g_settings.psk_hex.empty() && psk_hex_cli[0] == '\0')
            strncpy(ARQ.psk_hex, g_settings.psk_hex.c_str(), 128);
#else
        ARQ.robust_enabled = robust_mode ? YES : NO;
        ARQ.bandwidth_mode = (bandwidth_mode_cli >= 0) ? bandwidth_mode_cli : BW_AUTO;
        // -Q 0 with auto mode: skip NB start, go directly to WB.
        // Both sides are controlled (benchmark/test), no NB probe needed.
        if (nb_probe_max == 0 && ARQ.bandwidth_mode == BW_AUTO)
            ARQ.narrowband_enabled = NO;
        else
            ARQ.narrowband_enabled = YES;  // Normal: start NB, negotiate WB via probe
        // Pre-dates SACK / streaming-compression default-on. Include the full
        // current default cap set so main.cc's rewrite doesn't silently strip
        // bits the constructor (arq_common.cc:299-302) just set. SACK / v2
        // can still be opt-out via --no-sack / --disable-sack-v2 (those flags
        // run later and mask the bits at main.cc:1505-1534).
        ARQ.local_capability = ((ARQ.bandwidth_mode == BW_AUTO) ? CAP_WB_CAPABLE : 0)
                             | CAP_RETX_TURN_TAIL;
        ARQ.force_compress = (force_compress_cli == 1);
        ARQ.skip_turbo_reverse = skip_turbo_reverse;
        ARQ.max_config_override = max_config_cli;
        ARQ.set_optimizer_disabled(no_optimizer_cli);
        // Phase 2 Step 5 — load 2D channel-state lookup table if --channel-lookup
        // path was supplied. See GUI branch above for rationale.
        if (channel_lookup_path_cli != NULL) {
            if (!ARQ.channel_lookup.init_from_json(channel_lookup_path_cli)) {
                fprintf(stderr, "ERROR: --channel-lookup load failed: %s (path=%s)\n",
                        ARQ.channel_lookup.last_error(), channel_lookup_path_cli);
                exit(1);
            }
            printf("[CHANNEL-LOOKUP] loaded %d cells from %s\n",
                   ARQ.channel_lookup.n_cells(), channel_lookup_path_cli);
            fflush(stdout);
        }
        // Encryption: CLI -E flag
        ARQ.encryption_mode = (encryption_mode_cli >= 0) ? encryption_mode_cli : ENCRYPT_OFF;
        if (ARQ.encryption_mode != ENCRYPT_OFF)
            ARQ.local_capability |= CAP_ENCRYPTION;
#endif
        telecom_system.narrowband_enabled = ARQ.narrowband_enabled;
        if (narrowband_mode != -1)
            printf("[FLAG] -%c ignored in ARQ mode: entry bandwidth follows the BW host "
                   "command and -Q (nb_probe_max)\n", narrowband_mode == 1 ? 'N' : 'W');
        // ROBUST_3 (cfg103) is a pinned-only, off-ladder low-band rung. When it is
        // explicitly pinned in ARQ mode, force the gearshift OFF so no climb / optimizer /
        // SNR producer can negotiate the session off the pinned rung; it is held for the
        // whole session (a BREAK still degrades to the robust floor as the sanctioned exit).
        // Byte-identical for every non-cfg103 config. Mirrors the cfg105 anchor staging.
        if (is_robust3_config(mod_config) && explicit_config)
            gear_shift_mode = NO_GEAR_SHIFT;
        const int server_start_result = ARQ.init(
            base_tcp_port, (gear_shift_mode == NO_GEAR_SHIFT)? NO : YES, mod_config);
        if (validate_modem_server_startup(server_start_result, stderr) != EXIT_SUCCESS)
        {
            if (input_dev) free(input_dev);
            if (output_dev) free(output_dev);
            shutdown_tee_logging();
            return EXIT_FAILURE;
        }

        // Monitor mode: auto-start in LISTENING state (no TCP LISTEN ON needed)
        if (is_monitor_mode) {
            ARQ.original_role = RESPONDER;
            ARQ.set_role(RESPONDER);
            ARQ.link_status = LISTENING;
            ARQ.connection_status = RECEIVING;

            // Monitor must always be BW_AUTO to follow WB upgrades
            ARQ.bandwidth_mode = BW_AUTO;
            ARQ.narrowband_enabled = YES;  // Start NB, follow upgrade
            ARQ.local_capability |= CAP_WB_CAPABLE;
#ifdef MERCURY_GUI_ENABLED
            g_gui_state.bandwidth_mode.store(BW_AUTO);
#endif
            printf("[MONITOR] Auto-started in LISTENING mode (BW_AUTO, scanning for HAIL)\n");
            fflush(stdout);
        }

        // Apply command-line arguments
        ARQ.connection_timeout = connection_timeout_ms;
        ARQ.max_connection_attempts = max_connection_attempts;
        ARQ.exit_on_disconnect = exit_on_disconnect;
        if (nb_probe_max >= 0)
            ARQ.nb_probe_max = nb_probe_max;
        if (psk_hex_cli[0] != '\0') {
            strncpy(ARQ.psk_hex, psk_hex_cli, sizeof(ARQ.psk_hex) - 1);
            ARQ.psk_hex[sizeof(ARQ.psk_hex) - 1] = '\0';
        }

        const int link_timeout_after_init = ARQ.link_timeout;
        // Ensure timeouts are adequate for MFSK frame durations
        {
            int min_ct = 2 * (ARQ.control_batch_size + ARQ.ack_batch_size)
                * ARQ.message_transmission_time_ms + 5000;
            if (ARQ.connection_timeout < min_ct) {
                printf("Adjusting connection_timeout from %d to %d ms for frame duration\n",
                       ARQ.connection_timeout, min_ct);
                ARQ.connection_timeout = min_ct;
            }
            // Link timeout must survive multiple consecutive NAck cycles.
            // Each cycle: data/ctrl TX + ACK wait ≈ 2 × message_time.
            // Allow 5 consecutive NAck cycles before disconnect.
            int min_lt = 5 * 2 * ARQ.message_transmission_time_ms + 5000;
            if (min_lt < 90000) min_lt = 90000;  // minimum 90s for very slow modes
            if (ARQ.link_timeout < min_lt) {
                printf("Adjusting link_timeout from %d to %d ms for frame duration\n",
                       ARQ.link_timeout, min_lt);
                ARQ.link_timeout = min_lt;
            }
        }
        printf("[ARQ-TIMEOUTS] init_config=%d connection_timeout=%d ms link_timeout=%d ms "
               "(after init %d ms)\n", mod_config, ARQ.connection_timeout, ARQ.link_timeout,
               link_timeout_after_init);

        if (connection_timeout_ms != 15000 || max_connection_attempts != 15 || link_timeout_ms != 30000 || exit_on_disconnect) {
            printf("ARQ config: connection_timeout=%dms, link_timeout=%dms, max_attempts=%d, exit_on_disconnect=%s\n",
                   connection_timeout_ms, link_timeout_ms, max_connection_attempts, exit_on_disconnect ? "yes" : "no");
        }

        ARQ.print_stats();

		// PRECOOK (Stage 1): pin the persistent shared capture ring at the MAX geometry across
		// the config ladder BEFORE the capture thread spawns (audioio_init_internal below).
		// After this, a gearshift/adopt config switch is a sub-µs scalar publish under a leaf
		// lock instead of a ~100-200 ms deinit→init held under capture_prep_mutex — which was
		// freezing the audio capture thread (demod deaf). No-op under MERCURY_PRECOOK_DEFEAT.
		telecom_system.precook_pin_shared_ring();

		// PRECOOK STEP 3: build the per-config geometry BUNDLES (one deep-copied cl_ofdm/ldpc/
		// psk/mfsk + pre_eq + descrambler per FULL_CONFIG_LADDER config) for the active bandwidth,
		// so every subsequent gearshift/adopt load_configuration is an M3 copy_from swap instead
		// of a deinit→init rebuild. Only when the ring actually pinned (skipped under
		// MERCURY_PRECOOK_DEFEAT, and when no config was loadable at pin time) — otherwise the
		// legacy path stays active and the bundles would never be consulted (load_configuration's
		// swap fast-path is gated on precook_ring_pinned). Built AFTER the pin's ladder walk (which
		// leaves config_bundles empty, so those sizing loads correctly take the legacy path) and
		// BEFORE audioio spawns the capture thread (S0 — no C1 race).
		if(telecom_system.data_container.precook_ring_pinned)
		{
			telecom_system.precook_config_bundles();
			printf("[PRECOOK] config bundles built (DUAL-BAND, live=%s): WB=%d slots, NB=%d slots "
				"(gearshift + NB↔WB adopt switches are now M3 swaps)\n",
				(telecom_system.narrowband_enabled == YES) ? "NB" : "WB",
				(int)telecom_system.config_bundles_wb.size(),
				(int)telecom_system.config_bundles_nb.size());
			fflush(stdout);
		}

		start_audio_or_fail();

        // Initialize parallel OFDM decoders for monitor mode (after audio init
        // so primary telecom_system has its final narrowband/config state)
        if (main_exit_status == EXIT_SUCCESS && is_monitor_mode)
            ARQ.init_monitor_decoders();

#ifdef MERCURY_GUI_ENABLED
        pthread_t gui_thread;
        bool gui_thread_created = false;
        bool gui_startup_failed = false;
        gui_thread_context gui_context;
        if (main_exit_status == EXIT_SUCCESS && !nogui && !shutdown_.load()) {
            printf("Starting GUI...\n");
            int thread_result = pthread_create(&gui_thread, NULL, gui_thread_func,
                                               &gui_context);
            if (thread_result != 0) {
                fprintf(stderr, "ERROR: could not start GUI thread (%d); modem startup aborted\n",
                        thread_result);
                gui_startup_failed = true;
            } else {
                gui_thread_created = true;
                int startup_status;
                // No numeric deadline: gui_init has no bounded call to derive one
                // from, so the bound is the operator's or supervisor's SIGTERM /
                // SIGINT (which store shutdown_). The 10 ms poll period is far
                // below the 10 s termination grace harnesses give.
                do {
                    startup_status = gui_context.status.load(std::memory_order_acquire);
                    if (startup_status == GUI_STARTUP_PENDING) {
                        if (shutdown_.load())
                            break;
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }
                } while (startup_status == GUI_STARTUP_PENDING);

                if (startup_status == GUI_STARTUP_PENDING) {
                    // gui_init is still blocked inside a display call; joining it
                    // would hang teardown, so the thread is left to process exit.
                    fprintf(stderr, "[GUI-START] abandoned: GUI init still pending at shutdown\n");
                    gui_startup_failed = true;
                    pthread_detach(gui_thread);
                    gui_thread_created = false;
                    gui_thread_abandoned = true;
                } else if (gui_validate_startup(startup_status, stderr) != 0) {
                    gui_startup_failed = true;
                    pthread_join(gui_thread, NULL);
                    gui_thread_created = false;
                }
            }
            if (gui_startup_failed) {
                main_exit_status = EXIT_FAILURE;
                // Audio is already running here: tell its threads to stop now so
                // audioio_deinit joins them instead of waiting out its grace.
                shutdown_.store(true);
            }
        }
#endif

        if (main_exit_status == EXIT_SUCCESS)
            audioio_apply_receive_decode_priority();

        while (
#ifdef MERCURY_GUI_ENABLED
               !gui_startup_failed
#else
               true
#endif
        )
        {
            if (shutdown_)
                break;

            ARQ.process_main();

            if (shutdown_)
                break;

#ifdef MERCURY_GUI_ENABLED
            if (!nogui) {
                // Update GUI state from ARQ
                gui_update_connection_status(ARQ.link_status, ARQ.connection_status, ARQ.role);
                g_gui_state.current_configuration.store(ARQ.current_configuration);
                g_gui_state.current_bitrate.store(telecom_system.rbc);
                g_gui_state.is_transmitting.store(ARQ.connection_status == TRANSMITTING_DATA ||
                                                   ARQ.connection_status == TRANSMITTING_CONTROL);
                g_gui_state.is_receiving.store(ARQ.connection_status == RECEIVING);
                g_gui_state.data_activity.store(ARQ.block_under_tx == YES ||
                                                 ARQ.connection_status == ACKNOWLEDGING_DATA);
                g_gui_state.ack_activity.store(ARQ.connection_status == RECEIVING_ACKS_DATA ||
                                                ARQ.connection_status == RECEIVING_ACKS_CONTROL ||
                                                ARQ.connection_status == ACKNOWLEDGING_DATA ||
                                                ARQ.connection_status == ACKNOWLEDGING_CONTROL);
                g_gui_state.constellation_is_mfsk.store(ARQ.current_configuration >= ROBUST_0
                                                        && ARQ.link_status == CONNECTED);

                // Update SNR measurements (uplink = what we receive, downlink = what remote receives from us)
                gui_update_arq_measurements(ARQ.get_snr_uplink(), ARQ.get_snr_downlink());

                // Sync coarse freq sync from GUI to telecom_system
                telecom_system.coarse_freq_sync_enabled = g_gui_state.coarse_freq_sync_enabled.load();
                // Sync robust mode and bandwidth mode from GUI to ARQ
                ARQ.robust_enabled = g_gui_state.robust_mode_enabled.load() ? YES : NO;
                ARQ.bandwidth_mode = g_gui_state.bandwidth_mode.load();
                ARQ.local_capability = ((ARQ.bandwidth_mode == BW_AUTO) ? CAP_WB_CAPABLE : 0)
                                    | ((ARQ.encryption_mode != ENCRYPT_OFF) ? CAP_ENCRYPTION : 0)
                                    | CAP_RETX_TURN_TAIL;
                // narrowband_enabled is set at startup (line ~728) based on -Q and -M flags.
                // Do NOT override here — forcing NB on telecom_system while the actual
                // config is WB causes get_tx_gain() to return NB gains (+7 dB overboosted).
                g_gui_state.session_is_wideband.store(ARQ.narrowband_enabled == NO && ARQ.link_status == CONNECTED);
                g_gui_state.peer_wb_capable.store((ARQ.peer_capability & CAP_WB_CAPABLE) != 0);

                // Compression status
                g_gui_state.compression_active.store(ARQ.compression_enabled);
                g_gui_state.compression_ratio.store(ARQ.compress_ratio_estimate);

                // Rolling throughput (10-second window, updated every 1s)
                {
                    static long long last_bytes = 0;
                    static uint32_t last_time = 0;
                    static double throughput_samples[10] = {0};
                    static int throughput_idx = 0;
                    static uint32_t last_bucket_time = 0;

                    auto tp = std::chrono::steady_clock::now();
                    uint32_t now = (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp.time_since_epoch()).count();
                    if (last_time == 0) { last_time = now; last_bucket_time = now; }

                    if (now - last_bucket_time >= 1000) {
                        long long current_bytes = g_gui_state.bytes_acked_total.load()
                                                + g_gui_state.bytes_received_total.load();
                        long long delta_bytes = current_bytes - last_bytes;
                        double delta_sec = (now - last_time) / 1000.0;
                        throughput_samples[throughput_idx % 10] =
                            (delta_sec > 0.01) ? (delta_bytes * 8.0 / delta_sec) : 0.0;
                        throughput_idx++;
                        last_bytes = current_bytes;
                        last_time = now;
                        last_bucket_time = now;

                        // Average over filled buckets
                        int n = (throughput_idx < 10) ? throughput_idx : 10;
                        double sum = 0;
                        for (int i = 0; i < n; i++) sum += throughput_samples[i];
                        g_gui_state.throughput_bps.store(n > 0 ? sum / n : 0.0);
                    }
                }

                // Check if GUI requested shutdown
                if (g_gui_state.request_shutdown.load()) {
                    shutdown_ = true;
                }
            }
#endif
        }

#ifdef MERCURY_GUI_ENABLED
        if (gui_thread_created) {
            g_gui_state.request_shutdown.store(true);
            pthread_join(gui_thread, NULL);
        }
#endif
    }

    if (telecom_system.operation_mode == TX_WAV)
    {
        // ULTRA audio render (SIM instrument, ultra-audio worktree only).
        // Build ONE frame via transmit_byte() -> data_container.passband_data
        // (double, 48 kHz mono) and write it straight to an S16LE WAV. No audio
        // device, no playback thread, no IONOS. With MERCURY_BAUD_MULT set on a
        // robust/MFSK config (-s 100), the captured tones are baud-scaled.
        printf("Mode selected: TX_WAV (sim render, no audio device)\n");
        telecom_system.load_configuration(mod_config);
        printf("Modulation: %d  Bitrate: %.2f bps  Shannon_limit: %.2f db\n",
               mod_config, telecom_system.rbc, telecom_system.Shannon_limit);

        int nReal_data = telecom_system.data_container.nBits - telecom_system.ldpc.P;
        int frame_size = (nReal_data - telecom_system.outer_code_reserved_bits) / 8;

        // Deterministic, recognizable payload (repeating 0x00..0xFF ramp).
        for (int i = 0; i < frame_size; i++)
            telecom_system.data_container.data_byte[i] = i & 0xFF;

        telecom_system.transmit_byte(telecom_system.data_container.data_byte,
                                     frame_size,
                                     telecom_system.data_container.passband_data,
                                     SINGLE_MESSAGE);

        long n_samples = (long)telecom_system.data_container.Nofdm
                       * telecom_system.data_container.interpolation_rate
                       * (telecom_system.ofdm.Nsymb
                          + telecom_system.ofdm.preamble_configurator.Nsymb);

        // Peak-normalize to 0.9 full-scale so the render is audible and clean
        // regardless of per-config passband amplitude.
        double peak = 0.0;
        for (long i = 0; i < n_samples; i++)
        {
            double a = telecom_system.data_container.passband_data[i];
            if (a < 0) a = -a;
            if (a > peak) peak = a;
        }
        double scale = (peak > 1e-12) ? (0.9 * 32767.0 / peak) : 1.0;

        const char* out_path = std::getenv("MERCURY_WAV_OUT");
        if (out_path == NULL) out_path = "tx_render.wav";

        double sym_ms = 1000.0 * telecom_system.data_container.Nofdm
                      * telecom_system.data_container.interpolation_rate / 48000.0;
        double dur_s = (double)n_samples / 48000.0;
        printf("[TX_WAV] Nfft=%d Nofdm=%d Nsymb=%d preamble=%d interp=%d -> %ld samples, "
               "%.3f s, %.2f ms/sym, peak=%.4f scale=%.1f -> %s\n",
               telecom_system.ofdm.Nfft, telecom_system.data_container.Nofdm,
               telecom_system.ofdm.Nsymb, telecom_system.ofdm.preamble_configurator.Nsymb,
               telecom_system.data_container.interpolation_rate,
               n_samples, dur_s, sym_ms, peak, scale, out_path);
        fflush(stdout);

        // Write a canonical 44-byte RIFF/WAVE header (PCM, mono, 48 kHz, 16-bit).
        uint32_t sample_rate = 48000;
        uint16_t n_chan = 1, bits = 16;
        uint32_t byte_rate = sample_rate * n_chan * (bits / 8);
        uint16_t block_align = n_chan * (bits / 8);
        uint32_t data_bytes = (uint32_t)(n_samples * (bits / 8));
        uint32_t riff_size = 36 + data_bytes;

        std::ofstream wf(out_path, std::ios::binary);
        if (!wf)
        {
            printf("[TX_WAV] ERROR: cannot open %s for writing\n", out_path);
        }
        else
        {
            auto w32 = [&](uint32_t v){ wf.write((const char*)&v, 4); };
            auto w16 = [&](uint16_t v){ wf.write((const char*)&v, 2); };
            wf.write("RIFF", 4); w32(riff_size); wf.write("WAVE", 4);
            wf.write("fmt ", 4); w32(16); w16(1); w16(n_chan);
            w32(sample_rate); w32(byte_rate); w16(block_align); w16(bits);
            wf.write("data", 4); w32(data_bytes);
            for (long i = 0; i < n_samples; i++)
            {
                double v = telecom_system.data_container.passband_data[i] * scale;
                if (v > 32767.0) v = 32767.0;
                if (v < -32768.0) v = -32768.0;
                int16_t s = (int16_t)(v >= 0 ? v + 0.5 : v - 0.5);
                wf.write((const char*)&s, 2);
            }
            wf.close();
            printf("[TX_WAV] wrote %u PCM bytes (%.3f s) to %s\n",
                   data_bytes, dur_s, out_path);
        }
        fflush(stdout);
    }

    if (telecom_system.operation_mode == RX_RAND)
    {
        printf("Mode selected: RX_RAND\n");
        telecom_system.load_configuration(mod_config);
        printf("Modulation: %d  Bitrate: %.2f bps  Shannon_limit: %.2f db\n",  mod_config, telecom_system.rbc, telecom_system.Shannon_limit);

        telecom_system.constellation_plot.open("PLOT");
        telecom_system.constellation_plot.reset("PLOT");

		start_audio_or_fail();

        while (!shutdown_)
        {
            telecom_system.RX_RAND_process_main();
        }
        telecom_system.constellation_plot.close();
    }

    if (telecom_system.operation_mode == TX_RAND)
    {
        printf("Mode selected: TX_RAND\n");
        telecom_system.load_configuration(mod_config);
        printf("Modulation: %d  Bitrate: %.2f bps  Shannon_limit: %.2f db\n",  mod_config, telecom_system.rbc, telecom_system.Shannon_limit);

		start_audio_or_fail();

        while (!shutdown_)
        {
            telecom_system.TX_RAND_process_main();
        }
    }

    if (telecom_system.operation_mode == BER_PLOT_baseband)
    {
        printf("Mode selected: PLOT_BASEBAND\n");
        telecom_system.load_configuration(mod_config);
        printf("Modulation: %d  Bitrate: %.2f bps  Shannon_limit: %.2f db\n",  mod_config, telecom_system.rbc, telecom_system.Shannon_limit);

        telecom_system.constellation_plot.open("PLOT");
        telecom_system.constellation_plot.reset("PLOT");

        telecom_system.BER_PLOT_baseband_process_main();

        telecom_system.constellation_plot.close();
    }

    if (telecom_system.operation_mode == BER_PLOT_passband)
    {
        printf("Mode selected: PLOT_PASSBAND\n");
        telecom_system.load_configuration(mod_config);
		// Measurement-only deterministic seed battery for exact-decoder A/Bs.
		// Unset preserves the historical process-default RNG streams verbatim.
		if(const char* e=std::getenv("MERCURY_BER_SEED"))
		{
			unsigned int seed=(unsigned int)strtoul(e, nullptr, 0);
			srand(seed);
			telecom_system.ts_srandom(seed);
		}
        telecom_system.test_puncture_nBits = puncture_nBits;
        if(puncture_nBits > 0)
            printf("Punctured LDPC: transmitting %d of %d bits\n", puncture_nBits, telecom_system.data_container.nBits);
        printf("Modulation: %d  Bitrate: %.2f bps  Shannon_limit: %.2f db\n",  mod_config, telecom_system.rbc, telecom_system.Shannon_limit);

        telecom_system.constellation_plot.open("PLOT");
        telecom_system.constellation_plot.reset("PLOT");

        telecom_system.BER_PLOT_passband_process_main();
        telecom_system.constellation_plot.close();
    }

    if (telecom_system.operation_mode == RX_TEST)
    {
        printf("Mode selected: RX_TEST\n");
        telecom_system.load_configuration(mod_config);
        printf("Modulation: %d  Bitrate: %.2f bps  Shannon_limit: %.2f db\n",  mod_config, telecom_system.rbc, telecom_system.Shannon_limit);

		start_audio_or_fail();

        while (!shutdown_)
        {
            telecom_system.RX_TEST_process_main();
        }

    }

    if (telecom_system.operation_mode == TX_TEST)
    {
        printf("Mode selected: TX_TEST\n");
        telecom_system.load_configuration(mod_config);
        printf("Modulation: %d  Bitrate: %.2f bps  Shannon_limit: %.2f db\n",  mod_config, telecom_system.rbc, telecom_system.Shannon_limit);

		start_audio_or_fail();

        while (!shutdown_)
        {
            telecom_system.TX_TEST_process_main();
        }

    }


    if (telecom_system.operation_mode == RX_SHM)
    {
        printf("Mode selected: RX_SHM\n");
        telecom_system.load_configuration(mod_config);
        printf("Modulation: %d  Bitrate: %.2f bps  Shannon_limit: %.2f db\n",  mod_config, telecom_system.rbc, telecom_system.Shannon_limit);

        cbuf_handle_t buffer;

        buffer = circular_buf_init_shm(SHM_PAYLOAD_BUFFER_SIZE, (char *) SHM_PAYLOAD_NAME);

        start_audio_or_fail();

        while (!shutdown_)
        {
            telecom_system.RX_SHM_process_main(buffer);
        }

        circular_buf_destroy_shm(buffer, SHM_PAYLOAD_BUFFER_SIZE, (char *) SHM_PAYLOAD_NAME);
        circular_buf_free_shm(buffer);
    }

    if (telecom_system.operation_mode == TX_SHM)
    {
        printf("Mode: TX_SHM  Modulation config: %d\n", mod_config);
        telecom_system.load_configuration(mod_config);
        printf("Bitrate: %.2f bps  Shannon lim.: %.2f db  TX: ",  mod_config, telecom_system.rbc, telecom_system.Shannon_limit);

        cbuf_handle_t buffer;

        buffer = circular_buf_init_shm(SHM_PAYLOAD_BUFFER_SIZE, (char *) SHM_PAYLOAD_NAME);

		start_audio_or_fail();

        while (!shutdown_)
        {
            telecom_system.TX_SHM_process_main(buffer);
        }

        circular_buf_destroy_shm(buffer, SHM_PAYLOAD_BUFFER_SIZE, (char *) SHM_PAYLOAD_NAME);
        circular_buf_free_shm(buffer);
    }

    if (input_dev)
        free(input_dev);
    if (output_dev)
        free(output_dev);

    // Contract with audioio: shutdown_ is set before audioio_deinit on every
    // path, so the joins below never wait on threads that were not told to stop.
    shutdown_.store(true);
    const bool audio_thread_detached =
        audioio_deinit(&radio_capture, &radio_playback, &radio_capture_prep) > 0;

    shutdown_tee_logging();

    if (gui_thread_abandoned || audio_thread_detached) {
        // A GUI thread left inside gui_init, or an audio thread audioio_deinit
        // had to detach (it may still use telecom_system), must not race the
        // unwinding of main's locals or the static destructors of a normal
        // return, so the process ends here.
        fflush(stdout);
        fflush(stderr);
        _exit(main_exit_status);
    }

    return main_exit_status;
}
