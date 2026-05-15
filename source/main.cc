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
    int max_config_cli = -1;          // --max-config: hard ceiling on turboshift
    int ptt_delay_cli = -1;           // --ptt-delay: override both PTT on/off delays (ms)
    int radio_batch_cli = -1;         // --radio-batch: total frames per radio TX (SACK)
    int retransmit_headroom_cli = -1; // --retransmit-headroom: max retransmit frames per batch
    int skip_var_gate_cli = -1;       // --skip-var-gate=on|off: -1=default(on), 0=off, 1=on
    int phy_reinit_settle_ms_cli = -1; // --phy-reinit-settle-ms=N: -1=default(300), 0+=override
    int rx_normalize_cli = -1;         // --rx-normalize=on|off: -1=default(on), 0=off, 1=on
    int csi_llr_cli = -1;              // --csi-llr=on|off: -1=default(on), 0=off, 1=on
    double ack_metric_threshold_cli = -1; // --ack-metric-threshold=F: <0 = default(0.5)
    int emergency_nack_cli = -1;       // --emergency-nack=N: -1=default(3), >=0=override
    int wb_match_bias_cli = 0;         // --wb-match-threshold-bias=N: 0=HEAD, +1=revert 7076a4b 8→7
    double mean_h_gate_cli = -1;       // --mean-h-gate=F: <0=default(0.30), 0..=override
    double psk_var_floor_cli = -1;     // --psk-var-floor=F: <0=default(0.001), 0..=override
    double energy_gate_floor_cli = -1; // --energy-gate-floor=F: <0=default(1e-12), 0..=override
    int ls_window_w_cli = -1, ls_window_h_cli = -1; // --ls-window=WxH (-1 = default 2x8)
    int ofdm_defer_overflow_cli = -1;  // --ofdm-defer-overflow=on|off (-1=default on)
    int sack_timeout_extra_ms_cli = -1; // --sack-timeout-extra-ms=N (-1=default 0 post-SACK_FIX_PLAN §7 step 3)
    bool no_sack_cli = false;          // --no-sack: force disable (no-op after B2 fix)
    bool enable_sack_cli = false;      // --enable-sack: opt-in to SACK (B2 fix: now off by default)
    bool enable_sack_v2_cli = false;   // --enable-sack-v2: SACK Design A Step 6 (negotiate-only opt-in)
    bool test_sack_ldpc_fail_cli = false; // --test-sack-ldpc-fail: fault-inject ldpc=NO (repro test)
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
            // SACK Design A Step 6: opt-in to CAP_SACK_V2 advertisement.
            // Negotiate-only at this step — sack_v2_enabled gates nothing yet.
            enable_sack_v2_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-sack-ldpc-fail") == 0)
        {
            // Fault-injection repro toggle: forces ldpc_ok=false for every
            // SACK reception (see SACK_LDPC_FALLBACK_INVESTIGATION.md §7 Step 1).
            test_sack_ldpc_fail_cli = true;
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
            ARQ.disable_sack = true;
            ARQ.local_capability &= ~CAP_SACK;  // strip from current value too
            printf("[FLAG] --no-sack: CAP_SACK masked from local_capability "
                   "(no-op after B2 fix; default is now disabled)\n");
        }
        if (enable_sack_cli) {
            ARQ.disable_sack = false;
            ARQ.local_capability |= CAP_SACK;
            printf("[FLAG] --enable-sack: opt-in SACK negotiation (default after B2 is OFF)\n");
        }
        if (no_sack_cli && enable_sack_cli) {
            fprintf(stderr, "ERROR: cannot pass both --no-sack and --enable-sack\n");
            exit(1);
        }
        if (enable_sack_v2_cli) {
            // SACK Design A Step 6 scaffolding: opt-in to CAP_SACK_V2
            // advertisement. Wire effect: byte 5 of TEST_CONNECTION carries
            // the CAP_SACK_V2 bit ORed into local_capability. No other wire
            // change — sack_v2_enabled is computed by both peers but gates
            // nothing functional at this step. Default builds do NOT set
            // this bit, preserving byte-for-byte TEST_CONNECTION shape on
            // v1-only sessions.
            ARQ.enable_sack_v2 = true;
            ARQ.local_capability |= CAP_SACK_V2;
            printf("[FLAG] --enable-sack-v2: CAP_SACK_V2 added to local_capability "
                   "(negotiate-only; gates nothing yet — SACK Design A Step 6)\n");
        }
        if (test_sack_ldpc_fail_cli) {
            ARQ.force_sack_ldpc_fail = true;
            printf("[FLAG] --test-sack-ldpc-fail: forcing ldpc_ok=false on every "
                   "SACK reception (repro test — SACK_LDPC_FALLBACK_INVESTIGATION.md)\n");
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
        // CLI gearshift with no explicit -s: default to ROBUST_0 and enable robust mode
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
        ARQ.local_capability = ((ARQ.bandwidth_mode == BW_AUTO) ? CAP_WB_CAPABLE : 0) | CAP_COMPRESSION | CAP_B2F_UNROLL;
        ARQ.force_compress = (force_compress_cli >= 0) ? (force_compress_cli == 1) : g_settings.force_compress;
        ARQ.skip_turbo_reverse = skip_turbo_reverse;
        ARQ.max_config_override = max_config_cli;
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
        ARQ.local_capability = ((ARQ.bandwidth_mode == BW_AUTO) ? CAP_WB_CAPABLE : 0) | CAP_COMPRESSION | CAP_B2F_UNROLL;
        ARQ.force_compress = (force_compress_cli == 1);
        ARQ.skip_turbo_reverse = skip_turbo_reverse;
        ARQ.max_config_override = max_config_cli;
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
                ARQ.local_capability = ((ARQ.bandwidth_mode == BW_AUTO) ? CAP_WB_CAPABLE : 0) | CAP_COMPRESSION | CAP_B2F_UNROLL
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
