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
#include <fstream>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <cstdarg>
#include <math.h>
#include <unistd.h>
#include <iostream>
#include <complex>
#include "physical_layer/telecom_system.h"
#include "physical_layer/mfsk_ctrl_codec_tests.h"
#include "physical_layer/ldpc_bp_osd_tests.h"
#include "datalink_layer/arq.h"
#include "audioio/audioio.h"

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
    bool shutdown_;
    // Audio channel configuration (0=LEFT, 1=RIGHT, 2=STEREO)
    extern int configured_input_channel;
    extern int configured_output_channel;
    extern int multichannel_mode;
    extern double noise_snr_db;
}

int g_verbose = 0;

int main(int argc, char *argv[])
{
#if defined(_WIN32)
    SetUnhandledExceptionFilter(crash_handler);
    // Also try vectored handler for heap corruption
    AddVectoredExceptionHandler(1, crash_handler);
#endif
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    // --test : run built-in unit tests and exit. Runs BOTH the MFSK ctrl-suffix
    // codec suite and the BP+OSD LDPC decoder pair (Phase A.2). Exit 0 only if
    // both pass. Checked before any audio/GUI/threading init so the test
    // process stays minimal.
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--test") == 0) {
            int failed = run_mfsk_ctrl_codec_tests();
            failed += run_ldpc_bp_osd_tests();
            return (failed == 0) ? 0 : 1;
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
    int ldpc_osd_norder_cli = -999;    // --ldpc-osd-norder=N: -999=unset, -1=BP-only, 0..3=OSD order
    int ldpc_osd_maxosd_cli = -999;    // --ldpc-osd-maxosd=N: -999=unset, -1..2=OSD call budget
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
    int  test_rsp_bsi_corrupt_at_cli = 0; // --test-rsp-bsi-corrupt-at=N: SACK Design A Step 4 synthetic discard test
    bool test_rsp_sack_rsp_crc_corrupt_cli = false; // --test-rsp-sack-rsp-crc-corrupt: SACK Design A Step 7 CRC8 fault injection (one-shot)
    int  test_rsp_sack_rsp_crc_corrupt_count_cli = 0; // --test-rsp-sack-rsp-crc-corrupt-count=N: SACK Design A Step 11 N-shot CRC8 fault injection
    int test_policy_axis1_fire_cli = 0; // --test-policy-axis1-fire=up|down: SACK Design A Step 9 — synthetic Axis-1 trigger.
                                        // 0=off, 1=up (success=100%), 2=down (success=0%). One-shot at startup, then exit.
                                        // Forces sack_v2_enabled=true so policy_evaluate_axis1() is taken.
    int test_policy_axis2_fire_cli = 0; // --test-policy-axis2-fire=up|down: SACK Design A Step 10 — synthetic Axis-2 trigger.
                                        // 0=off, 1=up (clean ring), 2=down (lossy ring). One-shot at startup, then exit.
                                        // Forces sack_v2_enabled=true so policy_evaluate_axis2() is taken.
    const char* test_partial_bsi_advance_cli = NULL; // --test-partial-bsi-advance=mfsk|ofdm: SACK partial-path BSI non-advance reproducer.
                                        // 'mfsk' should FAIL on HEAD (used_mfsk_path=true bypasses send_sack_v2_frame's Step 8a bump).
                                        // 'ofdm' should PASS on HEAD (regression guard for the existing OFDM SACK_RSP path).
                                        // One-shot at startup, then exit. See fact-documents/sack_partial_bsi_advance.md §5.
    bool test_data_anchored_promote_cli = false; // --test-data-anchored-promote: Option B (data-anchored gearshift
                                        // promotion) regression. Drives break_target_with_anchor() + policy_evaluate_axis1()
                                        // with last_data_viable_config primed; asserts BREAK floors at the anchor and the
                                        // up-shifter promotes only one rung past it. One-shot, exits rc. See
                                        // fact-documents/gearshift-start-and-recovery.md §6.4.
    bool test_phantom_ack_gate_cli = false; // --test-phantom-ack-gate: phantom-ACK content-gate regression.
                                        // Drives data_ack_bare_pattern_acceptable() across WB/NB x CRC-valid/CRC-absent
                                        // (the WB-no-CRC phantom cell must be REJECTED) + asserts a rejected phantom leaves
                                        // data_ack_received NO, does not raise last_data_viable_config / reset the BREAK
                                        // panic counter, and BREAK still reaches ROBUST_0. One-shot, exits rc. See
                                        // fact-documents/gearshift-start-and-recovery.md §8.
    bool test_clean_batch_viability_cli = false; // --test-clean-batch-viability: CLEAN-BATCH VIABILITY regression (§9).
    bool test_climb_engine_cli = false; // --test-climb-engine: integrated 3-bug climb regression (gearshift-climb-engine.md §7).
                                        // Asserts a PARTIAL SACK does NOT raise last_data_viable_config, reset the BREAK
                                        // panic counter / break_drop_step, advance the FRAME-UP counter, or clear the 85%
                                        // up-promotion gate; a CLEAN all-ones batch does all of those; and that BREAK can
                                        // still reach ROBUST_0 after a partial-only run. One-shot, exits rc. See §9.
    int test_policy_axis1_then_axis2_cli = 0; // --test-policy-axis1-then-axis2=up|down: SACK Design A Step 10 — fire Axis-1
                                        // (engages axis2_cooldown_batches=3) then attempt Axis-2 fire (should be SUPPRESSED).
                                        // 1=axis1=up then axis2=up; 2=axis1=down then axis2=down. One-shot at startup, then exit.
    int test_policy_axis3_fire_cli = 0; // --test-policy-axis3-fire={ok,miss,walk}: SACK Design A Step 11 — synthetic Axis-3 trigger.
                                        // 0=off, 1=single ok event, 2=single miss event, 3=composite walk demo
                                        // (3 misses → ON→PROBE, 2 more → PROBE→OFF, 20 ticks → OFF→PROBE, ok → PROBE→ON).
                                        // One-shot at startup, then exit. Forces sack_v2_enabled=true.
    int test_policy_axis1_then_axis3_cli = 0; // --test-policy-axis1-then-axis3=miss: SACK Design A Step 11 — fire Axis-1
                                        // supremacy then attempt Axis-3 fires (should be SUPPRESSED during the 3-batch cooldown).
                                        // 1=axis1 supremacy + 3 attempted Axis-3 misses. One-shot at startup, then exit.
    int test_policy_axis3_miss_burst_cli = 0; // --test-policy-axis3-miss-burst=N: SACK Design A Step 11 — fire N miss events
                                        // directly into policy_evaluate_axis3 at startup, then exit. Used by Gate 3
                                        // (drives ON→PROBE at N=3, PROBE→OFF at N=5) and Gate 6 (large N then tick to 20
                                        // batches to drive OFF→PROBE).
    int test_policy_axis3_recover_cli = 0; // --test-policy-axis3-recover=N: SACK Design A Step 11 — fire N misses then a single
                                        // ok (Gate 5: PROBE→ON recovery demo).
    int test_policy_axis3_offperiodic_cli = 0; // --test-policy-axis3-offperiodic=N: drive 5+ misses to OFF, then tick N batches
                                        // (Gate 6: OFF→PROBE 20-batch periodic re-probe).
    int test_force_sack_mode_cli = -1;  // --force-sack-mode={off,on,probe}: SACK Design A Step 11 — set Axis-3 mode at startup
                                        // (after init, before CONNECTED). Used for Gate 4 (SACK_OFF graceful fallback in
                                        // live session) and Gate 7 (SET_LINK_PARAMS sack_mode round-trip). -1 = unset.
    int test_policy_axis2_ceiling_fire_cli = 0; // --test-policy-axis2-ceiling-fire=1: SACK Design A Step 12 — drive a synthetic
                                        // Axis-2 down-move (sets proven_ceiling) then attempt an up-move (must be VETOED
                                        // by the ceiling). One-shot at startup, then exit.
    int test_policy_break_supremacy_cli = 0; // --test-policy-break-supremacy=1: SACK Design A Step 12 — invoke the supremacy
                                        // hook with a synthetic BREAK reason; assert Axis-2 + Axis-3 reset + 3 evaluations
                                        // suppressed. One-shot at startup, then exit.
    int audio_buffer_ms_cli = 0;       // --alsa-buffer-ms=N (Linux only; 0 = use 30ms default)
    char log_file_path[512] = "";     // --log: tee stdout to file

    input_dev = (char *) malloc(ALSA_MAX_PATH);
    output_dev = (char *) malloc(ALSA_MAX_PATH);
    input_dev[0] = 0;
    output_dev[0] = 0;

#if defined(__linux__)
	printf("\e[0;31mMercury Version %s\e[0m\n", VERSION__);
#elif defined(_WIN32)
	printf("Mercury Version %s\n", VERSION__);
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

        printf("\nDevice and audio:\n");
        printf("  -i [device]       Audio capture device (e.g. \"plughw:0,0\" or device name from -z)\n");
        printf("  -o [device]       Audio playback device\n");
        printf("  -x [api]          Sound system: alsa, pulse, dsound, wasapi (default: alsa/wasapi)\n");
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
        printf("  -s [config]       Modulation: 0-16 (OFDM), 100-102 (ROBUST MFSK). Use -l to list.\n");
        printf("  -g                Enable adaptive gearshift\n");
        printf("  -R                Enable ROBUST mode (MFSK weak-signal hailing)\n");
        printf("  -M [auto|nb]      Bandwidth: auto (NB hail + WB upgrade) or nb (500 Hz only)\n");
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
        else if (strcmp(argv[i], "--log") == 0 && i + 1 < argc)
        {
            strncpy(log_file_path, argv[i + 1], sizeof(log_file_path) - 1);
            // Remove --log and <file> from argv
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
        else if (strncmp(argv[i], "--ldpc-osd-norder=", 18) == 0)
        {
            // Phase A.2 §7.5 item 6. Range -1..3:
            //   -1 = disable OSD entirely (BP-only fallback)
            //    0 = OSD order-0 (just MRB encode of hard decisions)
            //    1 = OSD-1 (default; single-bit MRB flips, ~100 TEPs)
            //    2 = OSD-2 (double flips, ~5k TEPs — ~25 ms on Pi 5)
            //    3 = OSD-3 (triple flips, ~162k TEPs — ~800 ms; offline analysis only)
            ldpc_osd_norder_cli = atoi(argv[i] + 18);
            if (ldpc_osd_norder_cli < -1) ldpc_osd_norder_cli = -1;
            if (ldpc_osd_norder_cli > 3)  ldpc_osd_norder_cli = 3;
            printf("[FLAG] --ldpc-osd-norder=%d\n", ldpc_osd_norder_cli);
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--ldpc-osd-maxosd=", 18) == 0)
        {
            // Phase A.2 §7.5 item 6. Range -1..2 per research §2.4 (`ndepth`
            // mapping). 0 = single OSD call (current implementation); negative
            // = ignored / unset.
            ldpc_osd_maxosd_cli = atoi(argv[i] + 18);
            if (ldpc_osd_maxosd_cli < -1) ldpc_osd_maxosd_cli = -1;
            if (ldpc_osd_maxosd_cli > 2)  ldpc_osd_maxosd_cli = 2;
            printf("[FLAG] --ldpc-osd-maxosd=%d\n", ldpc_osd_maxosd_cli);
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
        else if (strncmp(argv[i], "--test-policy-axis1-fire=", 25) == 0)
        {
            // SACK Design A Step 9 — synthetic Axis-1 trigger (one-shot at startup).
            //
            // Primes the LADDER state and calls policy_evaluate_axis1() once,
            // then exits. Demonstrates that the multi-axis-policy entry point
            // is wired correctly and the [POLICY-MOVE] / [POLICY-SUPREMACY]
            // log surface (§4.3.4 invariants 5/6) fires on a real Axis-1 move.
            //
            // Values:
            //   up   — success_rate=100%, block-counter at threshold → LADDER UP
            //   down — success_rate=0%, consecutive_fails forced to threshold → LADDER DOWN
            //
            // Default off; production builds never pass this flag. The mercury
            // process exits with code 0 after the single synthetic evaluation.
            const char* arg = argv[i] + 25;
            if (strcmp(arg, "up") == 0)        test_policy_axis1_fire_cli = 1;
            else if (strcmp(arg, "down") == 0) test_policy_axis1_fire_cli = 2;
            else { fprintf(stderr, "--test-policy-axis1-fire: expected 'up' or 'down'\n"); exit(1); }
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--test-policy-axis2-fire=", 25) == 0)
        {
            // SACK Design A Step 10 — synthetic Axis-2 trigger (one-shot at startup).
            //
            // Primes the partial-rate ring + hysteresis counters and calls
            // policy_evaluate_axis2() once, then exits. Demonstrates that the
            // Axis-2 entry point is wired and the [POLICY-MOVE] axis=2 log
            // surface (§4.3.4 invariant 5) plus SET_LINK_PARAMS TX wire path
            // fire on a real Axis-2 move.
            //
            // Values:
            //   up   — ring primed clean (mean=0), good_run at threshold → UP
            //   down — ring primed lossy (mean=0.4), bad_run at threshold → DOWN
            //
            // Default off; production builds never pass this flag. The mercury
            // process exits with code 0 after the single synthetic evaluation.
            const char* arg = argv[i] + 25;
            if (strcmp(arg, "up") == 0)        test_policy_axis2_fire_cli = 1;
            else if (strcmp(arg, "down") == 0) test_policy_axis2_fire_cli = 2;
            else { fprintf(stderr, "--test-policy-axis2-fire: expected 'up' or 'down'\n"); exit(1); }
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--test-partial-bsi-advance=", 27) == 0)
        {
            // SACK partial-path BSI non-advance reproducer (one-shot at startup, then exit).
            // See fact-documents/sack_partial_bsi_advance.md §5.3 / §5.4. The 'mfsk'
            // variant exercises the bug (MFSK suffix bypass of send_sack_v2_frame's
            // Step 8a bump); the 'ofdm' variant exercises the working OFDM path
            // (regression guard).
            const char* arg = argv[i] + 27;
            if (strcmp(arg, "mfsk") == 0 || strcmp(arg, "ofdm") == 0)
                test_partial_bsi_advance_cli = arg;
            else { fprintf(stderr, "--test-partial-bsi-advance: expected 'mfsk' or 'ofdm'\n"); exit(1); }
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-data-anchored-promote") == 0)
        {
            // Option B (data-anchored gearshift promotion) regression — one-shot
            // at startup, then exit with the test's rc. See
            // fact-documents/gearshift-start-and-recovery.md §6.4.
            test_data_anchored_promote_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-phantom-ack-gate") == 0)
        {
            // Phantom-ACK content-gate regression — one-shot at startup, then
            // exit with the test's rc. See
            // fact-documents/gearshift-start-and-recovery.md §8.
            test_phantom_ack_gate_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-clean-batch-viability") == 0)
        {
            // CLEAN-BATCH VIABILITY regression (§9) — one-shot at startup, then
            // exit with the test's rc. See
            // fact-documents/gearshift-start-and-recovery.md §9.8.
            test_clean_batch_viability_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-climb-engine") == 0)
        {
            // Integrated 3-bug climb regression — one-shot at startup, then exit
            // with the test's rc. See fact-documents/gearshift-climb-engine.md §7.
            test_climb_engine_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--test-policy-axis2-ceiling-fire=", 33) == 0)
        {
            // SACK Design A Step 12 — synthetic Axis-2 proven-ceiling fire
            // (§4.3.4 invariant #7). Drives a synthetic Axis-2 down-move (which
            // sets batch_size_proven_ceiling) then attempts an up-move that
            // MUST be VETOED by the ceiling check. One-shot at startup, then
            // exit. Default off; production builds never pass this flag.
            const char* arg = argv[i] + 33;
            if (strcmp(arg, "1") == 0) test_policy_axis2_ceiling_fire_cli = 1;
            else { fprintf(stderr, "--test-policy-axis2-ceiling-fire: expected '1'\n"); exit(1); }
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--test-policy-break-supremacy=", 30) == 0)
        {
            // SACK Design A Step 12 — synthetic BREAK supremacy fire
            // (§4.3.4 invariant #6, BREAK integration). Invokes the supremacy
            // hook with a synthetic BREAK reason tag and confirms Axis-2 +
            // Axis-3 reset + 3 evaluations suppressed by the cooldown.
            // One-shot at startup, then exit. Default off.
            const char* arg = argv[i] + 30;
            if (strcmp(arg, "1") == 0) test_policy_break_supremacy_cli = 1;
            else { fprintf(stderr, "--test-policy-break-supremacy: expected '1'\n"); exit(1); }
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--test-policy-axis1-then-axis2=", 31) == 0)
        {
            // SACK Design A Step 10 — composite synthetic fire demonstrating
            // §4.3.4 invariant #6 (Axis-1 supremacy → Axis-2 3-batch cooldown).
            // Fires Axis-1 first (sets axis2_cooldown_batches=3 via the supremacy
            // hook), then immediately attempts an Axis-2 fire that SHOULD BE
            // SUPPRESSED because the cooldown is engaged. The fire then evaluates
            // 3 more synthetic batches to drain the cooldown and demonstrate
            // Axis-2 fires once cooldown reaches 0.
            const char* arg = argv[i] + 31;
            if (strcmp(arg, "up") == 0)        test_policy_axis1_then_axis2_cli = 1;
            else if (strcmp(arg, "down") == 0) test_policy_axis1_then_axis2_cli = 2;
            else { fprintf(stderr, "--test-policy-axis1-then-axis2: expected 'up' or 'down'\n"); exit(1); }
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--test-policy-axis3-fire=", 25) == 0)
        {
            // SACK Design A Step 11 — synthetic Axis-3 fire (single event or walk).
            const char* arg = argv[i] + 25;
            if (strcmp(arg, "ok") == 0)        test_policy_axis3_fire_cli = 1;
            else if (strcmp(arg, "miss") == 0) test_policy_axis3_fire_cli = 2;
            else if (strcmp(arg, "walk") == 0) test_policy_axis3_fire_cli = 3;
            else { fprintf(stderr, "--test-policy-axis3-fire: expected 'ok' or 'miss' or 'walk'\n"); exit(1); }
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--test-policy-axis1-then-axis3=", 31) == 0)
        {
            // SACK Design A Step 11 — composite synthetic fire demonstrating
            // §4.3.4 invariant #6 (Axis-1 supremacy → Axis-3 cooldown).
            const char* arg = argv[i] + 31;
            if (strcmp(arg, "miss") == 0)      test_policy_axis1_then_axis3_cli = 1;
            else { fprintf(stderr, "--test-policy-axis1-then-axis3: expected 'miss'\n"); exit(1); }
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--test-policy-axis3-miss-burst=", 31) == 0)
        {
            test_policy_axis3_miss_burst_cli = atoi(argv[i] + 31);
            if (test_policy_axis3_miss_burst_cli < 1) test_policy_axis3_miss_burst_cli = 1;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--test-policy-axis3-recover=", 28) == 0)
        {
            test_policy_axis3_recover_cli = atoi(argv[i] + 28);
            if (test_policy_axis3_recover_cli < 1) test_policy_axis3_recover_cli = 1;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strncmp(argv[i], "--test-policy-axis3-offperiodic=", 32) == 0)
        {
            test_policy_axis3_offperiodic_cli = atoi(argv[i] + 32);
            if (test_policy_axis3_offperiodic_cli < 1) test_policy_axis3_offperiodic_cli = 1;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
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
                strncpy(input_dev, optarg, ALSA_MAX_PATH-1);
            break;
        case 'o':
            if (optarg)
                strncpy(output_dev, optarg, ALSA_MAX_PATH-1);
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
            break;
        case 'x':
            if (!strcmp(optarg, "alsa"))
                audio_system = AUDIO_SUBSYSTEM_ALSA;
            if (!strcmp(optarg, "pulse"))
                audio_system = AUDIO_SUBSYSTEM_PULSE;
            if (!strcmp(optarg, "dsound"))
                audio_system = AUDIO_SUBSYSTEM_DSOUND;
            if (!strcmp(optarg, "wasapi"))
                audio_system = AUDIO_SUBSYSTEM_WASAPI;
            if (!strcmp(optarg, "oss"))
                audio_system = AUDIO_SUBSYSTEM_OSS;
            if (!strcmp(optarg, "coreaudio"))
                audio_system = AUDIO_SUBSYSTEM_COREAUDIO;
            break;
        case 'g':
            gear_shift_mode = GEAR_SHIFT_ENABLED;
            break;
        case 'z':
            list_sndcards = true;
            break;
        case 's':
            if (optarg)
                mod_config = atoi(optarg);
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
            if (optarg)
                link_timeout_ms = atoi(optarg);
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
                std::string enc_arg(optarg);
                if (enc_arg == "strict" || enc_arg == "1")
                    encryption_mode_cli = ENCRYPT_STRICT;
                else if (enc_arg == "fast" || enc_arg == "2")
                    encryption_mode_cli = ENCRYPT_FAST;
                else
                    printf("Unknown encryption mode '%s', use 'strict' or 'fast'\n", optarg);
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
                strncpy(input_dev, g_settings.input_device.c_str(), ALSA_MAX_PATH - 1);
                printf("Using input device from settings: %s\n", input_dev);
            }
            if (output_dev[0] == 0 && !g_settings.output_device.empty()) {
                strncpy(output_dev, g_settings.output_device.c_str(), ALSA_MAX_PATH - 1);
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
    if (ldpc_osd_norder_cli != -999) {
        // Apply BEFORE load_configuration so the value propagates through
        // default_configurations_telecom_system into cl_ldpc::osd_norder at
        // init time. (See telecom_system.cc ~4694 where the field is plumbed.)
        telecom_system.default_configurations_telecom_system.ldpc_osd_norder = ldpc_osd_norder_cli;
        printf("[FLAG] --ldpc-osd-norder=%d (applied to defaults)\n", ldpc_osd_norder_cli);
    }
    if (ldpc_osd_maxosd_cli != -999) {
        telecom_system.default_configurations_telecom_system.ldpc_osd_maxosd = ldpc_osd_maxosd_cli;
        printf("[FLAG] --ldpc-osd-maxosd=%d (applied to defaults)\n", ldpc_osd_maxosd_cli);
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
        for (int i = 0; i < NUMBER_OF_CONFIGS; i++)
        {
            telecom_system.load_configuration(i);
            printf("CONFIG_%d (%f bps), frame_size: %d Bytes / %d bits / %d non-byte-aligned bits\n", i,
                   telecom_system.rbc, telecom_system.get_frame_size_bytes(),
                   telecom_system.get_frame_size_bits(), telecom_system.get_frame_size_bits() - (telecom_system.get_frame_size_bytes() * 8));
        }
        return EXIT_SUCCESS;
    }


    if ((mod_config >= NUMBER_OF_CONFIGS && !is_robust_config(mod_config)) || (mod_config < 0))
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

    // initializing audio system
    pthread_t radio_capture, radio_playback, radio_capture_prep;

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
        if (test_policy_axis1_fire_cli != 0) {
            // SACK Design A Step 9 — synthetic Axis-1 fire (one-shot, then exit).
            // Calls the public test helper on cl_arq_controller which primes
            // the LADDER state with a synthetic observable + counters, then
            // calls policy_evaluate_axis1() once. The [POLICY-MOVE] and
            // [POLICY-SUPREMACY] log lines should appear on stdout.
            printf("[FLAG] --test-policy-axis1-fire=%s: invoking synthetic Axis-1 fire\n",
                   test_policy_axis1_fire_cli == 1 ? "up" : "down");
            fflush(stdout);
            ARQ.test_fire_policy_axis1(test_policy_axis1_fire_cli);
            printf("[FLAG] Synthetic fire complete — exiting.\n");
            fflush(stdout);
            exit(0);
        }
        if (test_policy_axis2_fire_cli != 0) {
            // SACK Design A Step 10 — synthetic Axis-2 fire (one-shot, then exit).
            printf("[FLAG] --test-policy-axis2-fire=%s: invoking synthetic Axis-2 fire\n",
                   test_policy_axis2_fire_cli == 1 ? "up" : "down");
            fflush(stdout);
            ARQ.test_fire_policy_axis2(test_policy_axis2_fire_cli);
            printf("[FLAG] Synthetic fire complete — exiting.\n");
            fflush(stdout);
            exit(0);
        }
        if (test_policy_axis2_ceiling_fire_cli != 0) {
            // SACK Design A Step 12 — Axis-2 proven-ceiling fire (one-shot, then exit).
            printf("[FLAG] --test-policy-axis2-ceiling-fire=1: invoking synthetic "
                   "Axis-2 ceiling enforcement demo (§4.3.4 invariant #7)\n");
            fflush(stdout);
            ARQ.test_fire_policy_axis2_ceiling();
            printf("[FLAG] Ceiling fire complete — exiting.\n");
            fflush(stdout);
            exit(0);
        }
        if (test_partial_bsi_advance_cli != NULL) {
            // SACK partial-path BSI non-advance reproducer (one-shot, then exit).
            // Returns 0 on PASS, 1 on FAIL. Exit code propagates so test
            // harnesses can assert via shell.
            printf("[FLAG] --test-partial-bsi-advance=%s: invoking synthetic "
                   "partial-path BSI advance reproducer\n", test_partial_bsi_advance_cli);
            fflush(stdout);
            int rc = ARQ.test_partial_bsi_advance(test_partial_bsi_advance_cli);
            printf("[FLAG] Partial-bsi-advance test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_data_anchored_promote_cli) {
            // Option B (data-anchored gearshift promotion) regression (one-shot,
            // then exit rc). Drives break_target_with_anchor() + the real
            // policy_evaluate_axis1() up-shifter with last_data_viable_config
            // primed. See fact-documents/gearshift-start-and-recovery.md §6.4.
            printf("[FLAG] --test-data-anchored-promote: invoking Option B "
                   "promotion/recovery regression\n");
            fflush(stdout);
            int rc = ARQ.test_data_anchored_promote();
            printf("[FLAG] Data-anchored-promote test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_phantom_ack_gate_cli) {
            // Phantom-ACK content-gate regression (one-shot, then exit rc).
            // Drives the pure acceptance policy + the cross-layer anchor/panic/
            // BREAK invariant. See fact-documents/gearshift-start-and-recovery.md §8.
            printf("[FLAG] --test-phantom-ack-gate: invoking phantom-ACK "
                   "content-gate regression\n");
            fflush(stdout);
            int rc = ARQ.test_phantom_ack_gate();
            printf("[FLAG] Phantom-ack-gate test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_clean_batch_viability_cli) {
            // CLEAN-BATCH VIABILITY regression (one-shot, then exit rc). Drives the
            // pure promotion predicate + replays the four gated gearshift consumers
            // for a partial vs a clean batch. See
            // fact-documents/gearshift-start-and-recovery.md §9.8.
            printf("[FLAG] --test-clean-batch-viability: invoking CLEAN-BATCH "
                   "VIABILITY regression\n");
            fflush(stdout);
            int rc = ARQ.test_clean_batch_viability();
            printf("[FLAG] Clean-batch-viability test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_climb_engine_cli) {
            // Integrated climb regression (one-shot, then exit rc). Drives the
            // split SACK dedupe (Bug 1, Part A), the REAL Axis-2 robust guard
            // (Bug 2/3, Part B), the end-to-end multi-rung climb (Bug 3, Part C —
            // the assertion the C1/C2/C3 singles lacked), and the connect-path
            // CMD/RSP batch symmetry (the 4th wire failure, Part D — the
            // negotiated_configuration-vs-current_configuration default-init).
            // See fact-documents/gearshift-climb-engine.md §7 +
            // data-flow-batch-size.md §6.
            printf("[FLAG] --test-climb-engine: invoking integrated climb "
                   "regression (Parts A-D)\n");
            fflush(stdout);
            int rc = ARQ.test_climb_engine();
            printf("[FLAG] Climb-engine test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_policy_break_supremacy_cli != 0) {
            // SACK Design A Step 12 — synthetic BREAK supremacy fire (one-shot, then exit).
            printf("[FLAG] --test-policy-break-supremacy=1: invoking synthetic "
                   "BREAK supremacy demo (§4.3.4 invariant #6 — BREAK integration)\n");
            fflush(stdout);
            ARQ.test_fire_policy_break_supremacy();
            printf("[FLAG] BREAK supremacy fire complete — exiting.\n");
            fflush(stdout);
            exit(0);
        }
        if (test_policy_axis1_then_axis2_cli != 0) {
            // SACK Design A Step 10 — Axis-1-then-Axis-2 supremacy demo
            // (one-shot, then exit). Demonstrates §4.3.4 invariant #6 + §4.3.3.
            //
            // We CANNOT call add_message_control() in this synthetic test
            // (messages_control.data is null in pre-init mode) — so we avoid
            // the test_fire_policy_axis1 path (which calls cleanup() +
            // add_message_control on a real move). Instead we directly call
            // policy_axis1_supremacy_on_move() to engage the cooldown, then
            // attempt Axis-2 fires which DO route to add_message_control
            // (SET_LINK_PARAMS) if the cooldown is drained — but only the
            // suppression-path eval calls run here, so messages_control is
            // never touched.
            const char* dir_str = (test_policy_axis1_then_axis2_cli == 1 ? "up" : "down");
            int dir = test_policy_axis1_then_axis2_cli;
            printf("[FLAG] --test-policy-axis1-then-axis2=%s: invoking composite "
                   "Axis-1-then-Axis-2 supremacy demo\n", dir_str);
            fflush(stdout);

            // Mark v2 enabled (gate for all Axis-2 paths).
            ARQ.sack_v2_enabled = true;

            // Step A: directly engage the supremacy hook (sets cooldown=3 + resets ring/counters).
            printf("[DEMO-STEP-A] firing supremacy hook directly to engage Axis-2 "
                   "cooldown (skipping full Axis-1 fire to avoid messages_control "
                   "init dependency)...\n");
            fflush(stdout);
            ARQ.policy_axis1_supremacy_on_move(4, dir == 1 ? 5 : 3,
                                               dir == 1 ? "ladder_up_synthetic"
                                                        : "ladder_down_synthetic");

            // Set a starting batch (bypass clamp since max_*_length=0).
            int batch_for_demo = 25;
            if (batch_for_demo < ARQ.AXIS2_BATCH_FLOOR) batch_for_demo = ARQ.AXIS2_BATCH_FLOOR;
            if (batch_for_demo > ARQ.AXIS2_BATCH_CEIL)  batch_for_demo = ARQ.AXIS2_BATCH_CEIL;
            ARQ.data_batch_size = batch_for_demo;

            int synth_rx;
            if (dir == 1) {
                // UP attempt: would normally fire if not for cooldown.
                ARQ.axis2_consecutive_good_batches = ARQ.AXIS2_UP_GOOD_RUN - 1;
                for (int i = 0; i < ARQ.AXIS2_RING_DEPTH; i++)
                    ARQ.axis2_partial_rate_ring[i] = 0.0f;
                ARQ.axis2_partial_rate_count = ARQ.AXIS2_RING_DEPTH;
                synth_rx = batch_for_demo;  // partial_rate = 0
            } else {
                ARQ.axis2_consecutive_bad_batches = ARQ.AXIS2_DOWN_BAD_RUN - 1;
                for (int i = 0; i < ARQ.AXIS2_RING_DEPTH; i++)
                    ARQ.axis2_partial_rate_ring[i] = 0.4f;
                ARQ.axis2_partial_rate_count = ARQ.AXIS2_RING_DEPTH;
                synth_rx = (int)(batch_for_demo * 0.6f);  // partial_rate = 0.4
            }

            // Step B: 3 attempted fires while cooldown active (MUST be suppressed).
            // The controller's cooldown-gate path emits [POLICY-AXIS2] eval ...
            // COOLDOWN_REMAINING=N — never [POLICY-MOVE] axis=2. We do NOT
            // expect any add_message_control(SET_LINK_PARAMS) call in this branch.
            for (int b = 1; b <= 3; b++) {
                printf("[DEMO-STEP-B%d] attempt Axis-2 fire while cooldown active "
                       "(expect SUPPRESSED, cooldown_remaining=%d after)...\n",
                       b, 3 - b);
                fflush(stdout);
                ARQ.policy_evaluate_axis2(synth_rx, batch_for_demo);
            }

            // The cooldown is now 0. To demonstrate that a 4th eval WOULD now
            // fire, we'd call policy_evaluate_axis2 once more — but that path
            // calls add_message_control(SET_LINK_PARAMS) which crashes on
            // uninitialized messages_control.data. Instead we just print the
            // post-state for visibility — the [POLICY-AXIS2] log line on the
            // 3rd suppressed call already showed COOLDOWN_REMAINING=0 → next
            // eval will fire.
            printf("[DEMO-STEP-C] cooldown drained. The 4th evaluation WOULD fire "
                   "[POLICY-MOVE] axis=2 direction=%s (skipped here to avoid the "
                   "messages_control.data == NULL crash in pre-init synthetic "
                   "mode; the suppression behavior is the load-bearing demo and "
                   "is fully demonstrated by Steps B1..B3 above).\n", dir_str);
            fflush(stdout);

            printf("[FLAG] Composite fire complete — exiting.\n");
            fflush(stdout);
            exit(0);
        }
        if (test_policy_axis3_fire_cli != 0) {
            // SACK Design A Step 11 — synthetic Axis-3 fire (one-shot, then exit).
            const char* kind_str =
                (test_policy_axis3_fire_cli == 1) ? "ok"
              : (test_policy_axis3_fire_cli == 2) ? "miss"
              : (test_policy_axis3_fire_cli == 3) ? "walk" : "?";
            printf("[FLAG] --test-policy-axis3-fire=%s: invoking synthetic Axis-3 fire\n",
                   kind_str);
            fflush(stdout);
            ARQ.test_fire_policy_axis3(test_policy_axis3_fire_cli);
            printf("[FLAG] Synthetic fire complete — exiting.\n");
            fflush(stdout);
            exit(0);
        }
        if (test_policy_axis1_then_axis3_cli != 0) {
            // SACK Design A Step 11 — Axis-1-then-Axis-3 supremacy demo (one-shot, exit).
            // Fires Axis-1 supremacy hook directly (engages axis3_cooldown_batches=3)
            // then attempts 3 Axis-3 fires that MUST be suppressed (cooldown 3→2→1→0).
            printf("[FLAG] --test-policy-axis1-then-axis3=miss: invoking composite "
                   "Axis-1-then-Axis-3 supremacy demo\n");
            fflush(stdout);
            ARQ.sack_v2_enabled = true;
            // Start in ON to make the supremacy hook transition observable.
            ARQ.axis3_sack_mode = ARQ.SACK_MODE_ON;
            printf("[DEMO-AXIS3-STEP-A] firing supremacy hook directly to engage "
                   "Axis-3 cooldown ...\n");
            fflush(stdout);
            ARQ.policy_axis1_supremacy_on_move(4, 3, "ladder_down_synthetic");
            // Three attempted Axis-3 misses while cooldown active.
            for (int b = 1; b <= 3; b++) {
                printf("[DEMO-AXIS3-STEP-B%d] attempt Axis-3 miss while cooldown "
                       "active (expect SUPPRESSED, cooldown drains by 1 in "
                       "batch_tick afterwards)...\n", b);
                fflush(stdout);
                ARQ.policy_evaluate_axis3(false);
                // The Axis-3 cooldown is decremented by axis3_batch_tick()
                // (not by policy_evaluate_axis3 itself), so we tick once per
                // attempt to mirror the per-batch drain pattern used in
                // production.
                ARQ.axis3_batch_tick();
            }
            printf("[DEMO-AXIS3-STEP-C] cooldown drained. Subsequent Axis-3 "
                   "evaluations are free to move (skipped here to avoid the "
                   "messages_control.data == NULL crash in pre-init synthetic "
                   "mode; the suppression behavior is the load-bearing demo).\n");
            fflush(stdout);
            printf("[FLAG] Composite fire complete — exiting.\n");
            fflush(stdout);
            exit(0);
        }
        if (test_policy_axis3_miss_burst_cli > 0) {
            // SACK Design A Step 11 — drive N consecutive synthetic miss events
            // into policy_evaluate_axis3() to demonstrate Gate 3 ON→PROBE→OFF.
            printf("[FLAG] --test-policy-axis3-miss-burst=%d: driving %d consecutive miss events into Axis-3 controller\n",
                   test_policy_axis3_miss_burst_cli, test_policy_axis3_miss_burst_cli);
            fflush(stdout);
            ARQ.sack_v2_enabled = true;
            ARQ.axis3_sack_mode = ARQ.SACK_MODE_ON;
            for (int i = 1; i <= test_policy_axis3_miss_burst_cli; i++) {
                printf("[TEST-AXIS3-BURST] miss #%d/%d (mode before=%s consec=%d)\n",
                       i, test_policy_axis3_miss_burst_cli,
                       (ARQ.axis3_sack_mode == ARQ.SACK_MODE_OFF) ? "OFF"
                       : (ARQ.axis3_sack_mode == ARQ.SACK_MODE_PROBE) ? "PROBE" : "ON",
                       ARQ.axis3_consecutive_sack_misses);
                fflush(stdout);
                ARQ.policy_evaluate_axis3(false);
            }
            printf("[FLAG] miss-burst complete. final mode=%s consec=%d "
                   "on_to_probe=%lld probe_to_off=%lld\n",
                   (ARQ.axis3_sack_mode == ARQ.SACK_MODE_OFF) ? "OFF"
                   : (ARQ.axis3_sack_mode == ARQ.SACK_MODE_PROBE) ? "PROBE" : "ON",
                   ARQ.axis3_consecutive_sack_misses,
                   ARQ.axis3_move_on_to_probe_count,
                   ARQ.axis3_move_probe_to_off_count);
            fflush(stdout);
            exit(0);
        }
        if (test_policy_axis3_recover_cli > 0) {
            // SACK Design A Step 11 — Gate 5: N misses then a single ok → PROBE→ON.
            int n = test_policy_axis3_recover_cli;
            printf("[FLAG] --test-policy-axis3-recover=%d: driving %d miss events then a single ok event (expect PROBE→ON on the ok)\n", n, n);
            fflush(stdout);
            ARQ.sack_v2_enabled = true;
            ARQ.axis3_sack_mode = ARQ.SACK_MODE_ON;
            for (int i = 1; i <= n; i++) {
                printf("[TEST-AXIS3-RECOVER] miss #%d/%d (mode before=%s consec=%d)\n",
                       i, n,
                       (ARQ.axis3_sack_mode == ARQ.SACK_MODE_OFF) ? "OFF"
                       : (ARQ.axis3_sack_mode == ARQ.SACK_MODE_PROBE) ? "PROBE" : "ON",
                       ARQ.axis3_consecutive_sack_misses);
                fflush(stdout);
                ARQ.policy_evaluate_axis3(false);
            }
            printf("[TEST-AXIS3-RECOVER] feeding single ok event (mode before=%s consec=%d) — expect PROBE→ON if currently PROBE\n",
                   (ARQ.axis3_sack_mode == ARQ.SACK_MODE_OFF) ? "OFF"
                   : (ARQ.axis3_sack_mode == ARQ.SACK_MODE_PROBE) ? "PROBE" : "ON",
                   ARQ.axis3_consecutive_sack_misses);
            fflush(stdout);
            ARQ.policy_evaluate_axis3(true);
            printf("[FLAG] recover demo complete. final mode=%s probe_to_on=%lld\n",
                   (ARQ.axis3_sack_mode == ARQ.SACK_MODE_OFF) ? "OFF"
                   : (ARQ.axis3_sack_mode == ARQ.SACK_MODE_PROBE) ? "PROBE" : "ON",
                   ARQ.axis3_move_probe_to_on_count);
            fflush(stdout);
            exit(0);
        }
        if (test_policy_axis3_offperiodic_cli > 0) {
            // SACK Design A Step 11 — Gate 6: drive to OFF then tick N batches
            // (expect OFF→PROBE on the 20th tick).
            int n = test_policy_axis3_offperiodic_cli;
            printf("[FLAG] --test-policy-axis3-offperiodic=%d: driving 5 misses (to OFF) then ticking %d batches "
                   "(OFF→PROBE expected on the 20th)\n", n, n);
            fflush(stdout);
            ARQ.sack_v2_enabled = true;
            ARQ.axis3_sack_mode = ARQ.SACK_MODE_ON;
            for (int i = 1; i <= 5; i++) {
                printf("[TEST-AXIS3-OFFP] miss #%d/5\n", i);
                fflush(stdout);
                ARQ.policy_evaluate_axis3(false);
            }
            printf("[TEST-AXIS3-OFFP] now in mode=%s; ticking %d batches\n",
                   (ARQ.axis3_sack_mode == ARQ.SACK_MODE_OFF) ? "OFF"
                   : (ARQ.axis3_sack_mode == ARQ.SACK_MODE_PROBE) ? "PROBE" : "ON",
                   n);
            fflush(stdout);
            for (int b = 1; b <= n; b++) {
                printf("[TEST-AXIS3-OFFP] tick %d/%d (mode=%s batches_since_off=%d)\n",
                       b, n,
                       (ARQ.axis3_sack_mode == ARQ.SACK_MODE_OFF) ? "OFF"
                       : (ARQ.axis3_sack_mode == ARQ.SACK_MODE_PROBE) ? "PROBE" : "ON",
                       ARQ.axis3_batches_since_off);
                fflush(stdout);
                ARQ.axis3_batch_tick();
            }
            printf("[FLAG] offperiodic demo complete. final mode=%s off_to_probe=%lld\n",
                   (ARQ.axis3_sack_mode == ARQ.SACK_MODE_OFF) ? "OFF"
                   : (ARQ.axis3_sack_mode == ARQ.SACK_MODE_PROBE) ? "PROBE" : "ON",
                   ARQ.axis3_move_off_to_probe_count);
            fflush(stdout);
            exit(0);
        }

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

        // Phase A.2 §7.5 BP+OSD: CLI overrides INI, INI overrides default.
        // (CLI was already applied earlier — see the ldpc_osd_norder_cli /
        // ldpc_osd_maxosd_cli block above. Here we apply INI only if the user
        // didn't set the CLI flag.)
        if (ldpc_osd_norder_cli == -999 && g_settings.ldpc_osd_norder != -1)
        {
            telecom_system.default_configurations_telecom_system.ldpc_osd_norder = g_settings.ldpc_osd_norder;
            printf("[INI] LDPC.OSDNorder=%d\n", g_settings.ldpc_osd_norder);
        }
        if (ldpc_osd_maxosd_cli == -999 && g_settings.ldpc_osd_maxosd != -1)
        {
            telecom_system.default_configurations_telecom_system.ldpc_osd_maxosd = g_settings.ldpc_osd_maxosd;
            printf("[INI] LDPC.OSDMaxOsd=%d\n", g_settings.ldpc_osd_maxosd);
        }
        telecom_system.coarse_freq_sync_enabled = g_settings.coarse_freq_sync_enabled;
        g_gui_state.coarse_freq_sync_enabled.store(g_settings.coarse_freq_sync_enabled);
        // Robust mode: CLI -R overrides INI setting
        if (robust_mode)
            g_settings.robust_mode_enabled = true;
        g_gui_state.robust_mode_enabled.store(g_settings.robust_mode_enabled);
        // All stations always start NB — bandwidth_mode controls WB upgrade.
        // CLI -N/-W still available for BER testing but ignored for ARQ.
        g_settings.narrowband_enabled = true;
        g_gui_state.narrowband_enabled.store(true);
        // Bandwidth mode: CLI -M overrides INI setting
        if (bandwidth_mode_cli >= 0)
            g_settings.bandwidth_mode = bandwidth_mode_cli;
        g_gui_state.bandwidth_mode.store(g_settings.bandwidth_mode);
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
        ARQ.bandwidth_mode = g_settings.bandwidth_mode;
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
                             ;
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
                             ;
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
        ARQ.init(base_tcp_port, (gear_shift_mode == NO_GEAR_SHIFT)? NO : YES, mod_config);

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
        ARQ.link_timeout = link_timeout_ms;
        ARQ.max_connection_attempts = max_connection_attempts;
        ARQ.exit_on_disconnect = exit_on_disconnect;
        if (nb_probe_max >= 0)
            ARQ.nb_probe_max = nb_probe_max;
        if (psk_hex_cli[0] != '\0') {
            strncpy(ARQ.psk_hex, psk_hex_cli, sizeof(ARQ.psk_hex) - 1);
            ARQ.psk_hex[sizeof(ARQ.psk_hex) - 1] = '\0';
        }

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

        if (connection_timeout_ms != 15000 || max_connection_attempts != 15 || link_timeout_ms != 30000 || exit_on_disconnect) {
            printf("ARQ config: connection_timeout=%dms, link_timeout=%dms, max_attempts=%d, exit_on_disconnect=%s\n",
                   connection_timeout_ms, link_timeout_ms, max_connection_attempts, exit_on_disconnect ? "yes" : "no");
        }

        ARQ.print_stats();

		audioio_init_internal(input_dev, output_dev, audio_system, &radio_capture,
							  &radio_playback, &radio_capture_prep, &telecom_system);

        // Initialize parallel OFDM decoders for monitor mode (after audio init
        // so primary telecom_system has its final narrowband/config state)
        if (is_monitor_mode)
            ARQ.init_monitor_decoders();

#ifdef MERCURY_GUI_ENABLED
        pthread_t gui_thread;
        if (!nogui) {
            printf("Starting GUI...\n");
            pthread_create(&gui_thread, NULL, gui_thread_func, NULL);
        }
#endif

        while (!shutdown_)
        {
            ARQ.process_main();

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
                                    | ((ARQ.encryption_mode != ENCRYPT_OFF) ? CAP_ENCRYPTION : 0);
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
        if (!nogui) {
            g_gui_state.request_shutdown.store(true);
            pthread_join(gui_thread, NULL);
        }
#endif
    }

    if (telecom_system.operation_mode == RX_RAND)
    {
        printf("Mode selected: RX_RAND\n");
        telecom_system.load_configuration(mod_config);
        printf("Modulation: %d  Bitrate: %.2f bps  Shannon_limit: %.2f db\n",  mod_config, telecom_system.rbc, telecom_system.Shannon_limit);

        telecom_system.constellation_plot.open("PLOT");
        telecom_system.constellation_plot.reset("PLOT");

		audioio_init_internal(input_dev, output_dev, audio_system, &radio_capture,
							  &radio_playback, &radio_capture_prep, &telecom_system);

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

		audioio_init_internal(input_dev, output_dev, audio_system, &radio_capture,
							  &radio_playback, &radio_capture_prep, &telecom_system);

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

		audioio_init_internal(input_dev, output_dev, audio_system, &radio_capture,
							  &radio_playback, &radio_capture_prep, &telecom_system);

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

		audioio_init_internal(input_dev, output_dev, audio_system, &radio_capture,
							  &radio_playback, &radio_capture_prep, &telecom_system);

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

        audioio_init_internal(input_dev, output_dev, audio_system, &radio_capture, &radio_playback, &radio_capture_prep, &telecom_system);

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

		audioio_init_internal(input_dev, output_dev, audio_system, &radio_capture,
							  &radio_playback, &radio_capture_prep, &telecom_system);

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

    audioio_deinit(&radio_capture, &radio_playback, &radio_capture_prep);

    shutdown_tee_logging();

    return EXIT_SUCCESS;
}
