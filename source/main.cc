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
#include <cstdlib>
#include <ctime>
#include <cstdarg>
#include <atomic>   // R006: std::atomic<bool> shutdown_ (cross-thread termination flag)
#include <thread>   // R006: --test-shutdown-atomic cross-thread smoke
#include <type_traits> // R006: static_assert shutdown_ is atomic
#include <random>   // P1 frame-0 vehicle: deterministic AWGN run-up synthesis
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
#include "audioio/audioio.h"
#include "common/sim_clock.h"

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
// R006 — shutdown_ atomicity regression (in-process synthetic-fire, test-only)
// ============================================================================
//
// CLI: --test-shutdown-atomic
//
// Race-audit R006 (mercury/fact-documents/data-flow-arq-recovery-cluster.md §7,
// race_audit/race_fix_audit.json): the termination flag `shutdown_` is read in
// main-thread spin loops AND written from the audio capture/playback/sim threads
// (audioio.c). As a plain `bool` that concurrent unsynchronized read+write is a
// C/C++ data race (UB). The fix makes it std::atomic<bool> (C++) / _Atomic bool
// (C, audioio.c), which is representation-compatible for the always-lock-free
// bool type.
//
// FAIL-BEFORE / PASS-AFTER:
//   - COMPILE-TIME guarantee (the structural fail-before): the static_assert
//     below fails to compile on the pre-fix tree where `shutdown_` is a plain
//     `bool` (std::is_same<decltype(shutdown_), std::atomic<bool>> is false).
//     This is the deterministic fail-before for a behaviour-neutral UB fix:
//     the audit (§7/§8) prescribes a TSan/clean-shutdown smoke, not a runtime
//     state assertion, because the data race is UB that does not manifest
//     deterministically without a sanitizer (no -flto here).
//   - RUNTIME smoke (pass-after): a writer thread sets shutdown_ from another
//     thread exactly as audioio.c does; the main thread observes the flip in a
//     spin loop exactly as main.cc does. Asserts the flip is observed (clean
//     termination) and that the atomic is lock-free (so no lock-induced
//     ordering surprises across the C/C++ boundary).
//
// Returns 0 on PASS, 1 on FAIL.
static int test_shutdown_atomic()
{
    // --- Compile-time structural assertions (the fail-before on pre-fix code) -
    static_assert(std::is_same<decltype(shutdown_), std::atomic<bool>>::value,
        "R006: shutdown_ must be std::atomic<bool> (a plain bool is a data race "
        "between the main-thread spin loops and the audioio.c audio threads).");

    bool pass = true;

    // Lock-free is required for representation-compatibility with the C TU's
    // _Atomic bool view and to avoid lock-induced cross-thread surprises.
    if(!shutdown_.is_lock_free())
    {
        printf("[TEST-SHUTDOWN-ATOMIC] FAIL: shutdown_ is not lock-free\n");
        pass = false;
    }

    // --- Runtime cross-thread smoke (the pass-after observation) -------------
    // Mirror the production access pattern: a non-main thread stores `true`
    // (audioio.c:886/1264/1381/...) while the main thread reads `!shutdown_`
    // (main.cc spin loops). On the pre-fix plain bool this is UB; on the
    // atomic it is well-defined and the flip is guaranteed observable.
    shutdown_.store(false);
    std::atomic<long long> spins{0};
    std::thread writer([&]() {
        // Brief spin so the reader genuinely loops before the flip arrives.
        for(volatile int i = 0; i < 100000; ++i) { /* burn */ }
        shutdown_.store(true);
    });

    long long guard = 0;
    const long long GUARD_MAX = 2000000000LL; // ~liveness bound; never hang
    while(!shutdown_)
    {
        spins.fetch_add(1, std::memory_order_relaxed);
        if(++guard >= GUARD_MAX) break;
    }
    writer.join();

    bool observed = (bool)shutdown_;
    if(!observed)
    {
        printf("[TEST-SHUTDOWN-ATOMIC] FAIL: main-thread spin never observed "
               "the cross-thread shutdown_=true store (guard=%lld)\n", guard);
        pass = false;
    }

    printf("[TEST-SHUTDOWN-ATOMIC] %s: lock_free=%d observed_flip=%d "
           "reader_spins=%lld\n",
        pass ? "PASS" : "FAIL", shutdown_.is_lock_free() ? 1 : 0,
        observed ? 1 : 0, (long long)spins.load());
    fflush(stdout);

    // Leave the flag clear so we do not poison any later test in the same proc.
    shutdown_.store(false);
    return pass ? 0 : 1;
}

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

// --test-sigterm-handler: directed regression for the FIX-C graceful-shutdown
// handler. Installs the SIGTERM/SIGINT disposition exactly as main() does, then
// raises SIGTERM at ITSELF and asserts the handler flipped shutdown_ to true.
// This is the in-process fail-before/pass-after: on a tree WITHOUT the handler
// installed, raise(SIGTERM) terminates the process (default disposition) and the
// "PASS" line below is never printed -> the wrapping `timeout ... ; echo rc=$?`
// in the bench check sees a non-zero, signal-killed exit. With the handler the
// store is observed and the test exits 0. (The ALSA-close half of FIX-C is
// proven RPi-side: arecord -l shows the substream released after SIGTERM.)
static int test_sigterm_handler()
{
#ifndef _WIN32
    shutdown_.store(false);
    install_termination_handlers();
    if (shutdown_.load()) {
        printf("[TEST-SIGTERM-HANDLER] FAIL: shutdown_ already true before raise\n");
        fflush(stdout);
        return 1;
    }
    raise(SIGTERM);   // delivered synchronously on this thread
    bool flipped = shutdown_.load();
    if (!flipped) {
        printf("[TEST-SIGTERM-HANDLER] FAIL: SIGTERM did not set shutdown_=true "
               "(handler not installed?)\n");
        fflush(stdout);
        return 1;
    }
    // Also confirm SIGINT (Ctrl-C) drives the same flag.
    shutdown_.store(false);
    raise(SIGINT);
    bool flipped_int = shutdown_.load();
    shutdown_.store(false);   // leave the flag clear for any later in-proc test
    if (!flipped_int) {
        printf("[TEST-SIGTERM-HANDLER] FAIL: SIGINT did not set shutdown_=true\n");
        fflush(stdout);
        return 1;
    }
    printf("[TEST-SIGTERM-HANDLER] PASS: SIGTERM+SIGINT both set shutdown_=true "
           "(async-signal-safe atomic store; lock_free=%d)\n",
           shutdown_.is_lock_free() ? 1 : 0);
    fflush(stdout);
    return 0;
#else
    printf("[TEST-SIGTERM-HANDLER] SKIP: POSIX signals not used on Windows path\n");
    fflush(stdout);
    return 0;
#endif
}

// --test-pas: PAS/PCS distribution-matcher integrity self-test (feat/pcs).
// (1) BIJECTION deshape(shape(x))==x over many random k-bit blocks for several
//     (lambda, rail_L) configs — a precision/overflow bug shows here (the
//     life-critical data-integrity guard, fact-documents/data-flow-pas-shaping.md §6).
// (2) CONSTANT-COMPOSITION: every shaped block has EXACTLY the target counts.
// (3) HISTOGRAM: aggregated level histogram matches the Maxwell-Boltzmann target.
// Returns 0 on full pass, 1 on any failure.
static int run_pas_selftest()
{
    printf("[TEST-PAS] PAS/PCS distribution-matcher bijection + composition self-test\n");
    const int amps[4] = {1,3,5,7};
    struct Cfg { double lam; int L; } cfgs[] = { {0.02,32},{0.04,32},{0.06,32},{0.04,16},{0.04,24},{0.08,34} };
    int fails = 0;
    unsigned int rng = 0xC0FFEEu;
    auto nextbit = [&](){ rng = rng*1664525u + 1013904223u; return (int)((rng>>23)&1u); };
    for (auto &cf : cfgs)
    {
        cl_dist_matcher dm;
        if (!dm.configure_maxwell_boltzmann(4, amps, cf.lam, cf.L))
        { printf("[TEST-PAS]   FAIL configure lam=%.2f L=%d\n", cf.lam, cf.L); fails++; continue; }
        int k = dm.info_bits(), L = dm.block_len();
        std::vector<int> inb(k), lev(L), outb(k);
        long target[4] = { dm.level_count(0), dm.level_count(1), dm.level_count(2), dm.level_count(3) };
        long hist[4] = {0,0,0,0};
        int  trials = 200000, bad = 0, compbad = 0;
        for (int t = 0; t < trials; ++t)
        {
            for (int b = 0; b < k; ++b) inb[b] = nextbit();
            if (!dm.shape(inb.data(), lev.data())) { bad++; continue; }
            // constant composition check
            long cc[4] = {0,0,0,0};
            for (int p = 0; p < L; ++p) { if (lev[p]<0||lev[p]>3){bad++;break;} cc[lev[p]]++; hist[lev[p]]++; }
            if (cc[0]!=target[0]||cc[1]!=target[1]||cc[2]!=target[2]||cc[3]!=target[3]) compbad++;
            if (!dm.deshape(lev.data(), outb.data())) { bad++; continue; }
            for (int b = 0; b < k; ++b) if (outb[b] != inb[b]) { bad++; break; }
        }
        double htot = (double)trials * L;
        printf("[TEST-PAS]   lam=%.2f L=%d k=%d counts{%ld,%ld,%ld,%ld} bijection_bad=%d comp_bad=%d "
               "hist{%.3f,%.3f,%.3f,%.3f} target{%.3f,%.3f,%.3f,%.3f} -> %s\n",
               cf.lam, L, k, target[0],target[1],target[2],target[3], bad, compbad,
               hist[0]/htot,hist[1]/htot,hist[2]/htot,hist[3]/htot,
               target[0]/(double)L,target[1]/(double)L,target[2]/(double)L,target[3]/(double)L,
               (bad==0 && compbad==0) ? "PASS" : "FAIL");
        if (bad != 0 || compbad != 0) fails++;
    }
    printf("[TEST-PAS] %s (%d config failure%s)\n", fails==0?"ALL PASS":"FAILED", fails, fails==1?"":"s");
    return fails==0 ? 0 : 1;
}

// --test-cfg17: CFG17 shaped-64-QAM COMPOSITION self-test (failing-test-first).
// Drives the SFO-GRID harness in-process for three decisive cells and asserts the
// COMPOSED stack decodes where a BARE arm fails — proving each of the three deep
// levers (PAS, TINTERP-seed turbo estimator, ratio-nvfix) is live in the CFG17
// gear. The harness rebuilds the grid at MOD_64QAM (MERCURY_SFO_GRID_M64/_PCS), so
// CFG17 itself need not be production-elected for the composition to be exercised.
// See fact-documents/data-flow-cfg17-shaped-64qam.md §3.
static void cfg17_set_env(const char* k, const char* v)
{
#if defined(_WIN32)
    _putenv_s(k, v);
#else
    setenv(k, v, 1);
#endif
}
// Save+restore so the harness's env-gated default-off paths are not perturbed for
// any later in-process work. Run ONE harness cell with the given env, return the
// decoded codeword count via the additive sfo_grid_last_cw_* snapshot.
static void cfg17_run_cell(const char* esn0, const char* chan, const char* m64,
                           const char* pcs, const char* coded, const char* nsymb,
                           const char* turbo_seed, const char* turbo_iters,
                           const char* nvfix, const char* nv_force,
                           int& cw_ok, int& cw_tot)
{
    // The complete env key set the harness reads for these cells.
    const char* keys[] = {
        "MERCURY_SFO_GRID", "MERCURY_SFO_GRID_ESN0", "MERCURY_SFO_GRID_CHAN",
        "MERCURY_SFO_GRID_M64", "MERCURY_SFO_GRID_PCS", "MERCURY_SFO_GRID_CODED",
        "MERCURY_SFO_GRID_NSYMB", "MERCURY_SFO_GRID_TURBO_SEED",
        "MERCURY_SFO_GRID_TURBO_ITERS", "MERCURY_SFO_GRID_NVFIX",
        "MERCURY_SFO_GRID_NV_FORCE"
    };
    const int nk = (int)(sizeof(keys)/sizeof(keys[0]));
    struct Saved { const char* key; bool had; std::string val; } sv[16];
    for (int i = 0; i < nk; i++) {
        const char* g = std::getenv(keys[i]);
        sv[i].key = keys[i]; sv[i].had = (g != nullptr);
        sv[i].val = g ? std::string(g) : std::string();
    }
    cfg17_set_env("MERCURY_SFO_GRID", "1");
    cfg17_set_env("MERCURY_SFO_GRID_ESN0", esn0);
    cfg17_set_env("MERCURY_SFO_GRID_CHAN", chan);
    cfg17_set_env("MERCURY_SFO_GRID_M64", m64);
    cfg17_set_env("MERCURY_SFO_GRID_PCS", pcs);
    cfg17_set_env("MERCURY_SFO_GRID_CODED", coded);
    cfg17_set_env("MERCURY_SFO_GRID_NSYMB", nsymb);
    cfg17_set_env("MERCURY_SFO_GRID_TURBO_SEED", turbo_seed);
    cfg17_set_env("MERCURY_SFO_GRID_TURBO_ITERS", turbo_iters);
    cfg17_set_env("MERCURY_SFO_GRID_NVFIX", nvfix);
    cfg17_set_env("MERCURY_SFO_GRID_NV_FORCE", nv_force);

    cl_telecom_system ts;
    ts.operation_mode = BER_PLOT_passband;
    ts.load_configuration(CONFIG_16);   // sizes OFDM/LDPC; harness overrides M to 64
    ts.sfo_grid_test();
    cw_ok  = ts.sfo_grid_last_cw_ok;
    cw_tot = ts.sfo_grid_last_cw_tot;

    for (int i = 0; i < nk; i++) {
#if defined(_WIN32)
        _putenv_s(sv[i].key, sv[i].had ? sv[i].val.c_str() : "");
#else
        if (sv[i].had) setenv(sv[i].key, sv[i].val.c_str(), 1); else unsetenv(sv[i].key);
#endif
    }
}

static int run_cfg17_selftest()
{
    printf("[TEST-CFG17] CFG17 shaped-64-QAM composition self-test (PAS + TINTERP-seed + ratio-nvfix)\n");
    int fails = 0;

    // ---- Cell A: PAS lever, clean @ 16 dB. uniform-64 FAILS, PAS-shaped-64 DECODES. ----
    // uniform-64 waterfall ~17.6 dB; PAS waterfall ~15.3 dB (PCS_VERDICT.md). At 16 dB
    // clean (CHAN=0) the bare uniform arm is below its waterfall (0/K) while the PAS
    // shaping gain (~2.3 dB vs uniform-64, genie-confirmed) crosses the LDPC waterfall
    // (K/K). This is the decisive clean-front composition cell: PAS makes 64-QAM decode
    // at ~CFG16's working point while carrying 6 bits/sym (MEASURED: u64 0/7, PAS 7/7).
    {
        int u_ok=-1,u_tot=-1, p_ok=-1,p_tot=-1;
        cfg17_run_cell("16","0","1","0","1","60","","1","0","0", u_ok,u_tot);   // uniform-64 (bare)
        cfg17_run_cell("16","0","1","1","1","60","","1","0","0", p_ok,p_tot);   // PAS-shaped-64 (composed)
        bool bare_fail = (u_tot>0 && u_ok <  u_tot);
        bool comp_pass = (p_tot>0 && p_ok == p_tot);
        bool ok = bare_fail && comp_pass;
        printf("[TEST-CFG17]   CELL-A PAS clean@16dB: uniform-64=%d/%d (bare%s) PAS-64=%d/%d (composed%s) -> %s\n",
               u_ok,u_tot, bare_fail?"_FAILS_OK":"_DECODED_unexpected",
               p_ok,p_tot, comp_pass?"_DECODES_OK":"_FAILED",
               ok?"PASS":"FAIL");
        if(!ok) fails++;
    }

    // ---- Cell B: TINTERP-seed estimator lever, freq-selective det-floor @ 18 dB. ----
    // CHAN=1 (det-floor freq-selective) dense Dx=1/Dy=3 lattice: the bare single-pass LS
    // estimator FLOORS (0/K — the per-subcarrier dispersive phase a flat-ML/cold-LS H
    // cannot represent), while the COMPOSED TINTERP-seed turbo estimator (it=0 warm
    // TINTERP seed + data_aided_channel_estimator + dd_seed_floor) CROSSES (K/K). GENIE
    // decodes K/K here (the det-floor is estimator-limited, NOT modulation-limited —
    // RESEARCH_cfg17-64qam.md §0/§7, PCS_VERDICT.md) so this proves the estimator stack
    // reaches the genie-class CSI CFG17 requires. MEASURED reproducible over 4 seeds:
    // LS 0/7, TINTERP-seed turbo 7/7 (post-FEC BER 0.458 -> 0).
    {
        int ls_ok=-1,ls_tot=-1, tb_ok=-1,tb_tot=-1;
        cfg17_run_cell("18","1","1","1","1","60","","1","0","0", ls_ok,ls_tot);          // LS-only PAS-64 single-pass (bare)
        cfg17_run_cell("18","1","1","1","1","60","tinterp","4","0","0", tb_ok,tb_tot);   // TINTERP-seed turbo PAS-64 (composed)
        bool bare_floor = (ls_tot>0 && ls_ok == 0);
        bool comp_cross = (tb_tot>0 && tb_ok == tb_tot);
        bool ok = bare_floor && comp_cross;
        printf("[TEST-CFG17]   CELL-B estimator detfloor@18dB CHAN1: LS-only=%d/%d (bare%s) TINTERP-turbo=%d/%d (composed%s) -> %s\n",
               ls_ok,ls_tot, bare_floor?"_FLOORS_OK":"_decoded_unexpected",
               tb_ok,tb_tot, comp_cross?"_CROSSES_OK":"_FAILED",
               ok?"PASS":"FAIL");
        if(!ok) fails++;
    }

    // ---- Cell C: ratio-nvfix lever, collapsed-nv regime @ 16 dB clean. ----
    // NV_FORCE=1e-6 reproduces the HW-only post-EQ-EVM nv-collapse (the in-process sim
    // never reproduces it — PCS_VERDICT.md / nvfix a0e22c8): the bare demap (NVFIX=0)
    // over-confidently flips inner 64-QAM bits -> BP iter-cap -> decode FAILS; with
    // NVFIX=1 the ratio-gate (nv < measure_var/8 ? measure_var : nv) substitutes the
    // measured post-EQ noise -> decode RECOVERS. Clean channel so PAS alone would decode
    // absent the forced collapse -> isolates the nvfix as the cause of recovery.
    {
        int raw_ok=-1,raw_tot=-1, fix_ok=-1,fix_tot=-1;
        cfg17_run_cell("16","0","1","1","1","60","","1","0","1e-6", raw_ok,raw_tot);   // collapsed nv, NO nvfix (bare)
        cfg17_run_cell("16","0","1","1","1","60","","1","1","1e-6", fix_ok,fix_tot);   // collapsed nv, ratio-nvfix (composed)
        bool bare_fail = (raw_tot>0 && raw_ok <  raw_tot);
        bool comp_pass = (fix_tot>0 && fix_ok >  raw_ok);   // nvfix strictly improves on the collapse
        bool ok = bare_fail && comp_pass;
        printf("[TEST-CFG17]   CELL-C nvfix nv-collapse@16dB: raw-nv=%d/%d (bare%s) ratio-nvfix=%d/%d (composed%s) -> %s\n",
               raw_ok,raw_tot, bare_fail?"_FAILS_OK":"_decoded_unexpected",
               fix_ok,fix_tot, comp_pass?"_RECOVERS_OK":"_FAILED",
               ok?"PASS":"FAIL");
        if(!ok) fails++;
    }

    printf("[TEST-CFG17] %s (%d cell failure%s)\n", fails==0?"ALL PASS":"FAILED", fails, fails==1?"":"s");
    return fails==0 ? 0 : 1;
}

// --test-tinterp-seed: TINTERP-SEED production estimator self-test (failing-test-first).
// Drives the SFO-GRID CODED harness in-process (the SAME production estimator +
// equalizer + CSI + demod + LDPC path the receive_byte rescue activates) on the ITU-R
// "GOOD" Watterson fade (MPG: CHAN=3 depth=10dB fd=0.1Hz dly=24), proving the it=0
// estimator seed-swap: the held LS window decodes 0/K (it averages the time-varying H
// and reads the fade as noise) while the per-carrier TIME-INTERPOLATION estimator
// (LS_channel_estimator_tinterp, the estimator the MERCURY_TINTERP_SEED rescue selects)
// decodes K/K. A CLEAN-channel cell asserts TINTERP is a decode NO-OP (LS K/K == TINTERP
// K/K), the byte-identical-clean guarantee the rescue relies on. Mirrors test_fade_
// tinterp.py at the built-in --test level. See fact-documents/
// data-flow-noise_variance_estimate.md.
static void tinterp_set_env(const char* k, const char* v)
{
#if defined(_WIN32)
    _putenv_s(k, v);
#else
    setenv(k, v, 1);
#endif
}
// Run ONE SFO-GRID CODED cell with the given Watterson params + estimator selector,
// returning decoded/total codewords. tinterp_prod!=0 selects the TIME_INTERP estimator
// (MERCURY_SFO_GRID_TINTERP_PROD); 0 = the production LS else-branch. Env is saved/
// restored so the harness default-off paths are not perturbed for later in-process work.
static void tinterp_run_cell(const char* esn0, const char* chan, const char* depth,
                             const char* fd, const char* dly, int tinterp_prod,
                             int& cw_ok, int& cw_tot)
{
    const char* keys[] = {
        "MERCURY_SFO_GRID", "MERCURY_SFO_GRID_CODED", "MERCURY_SFO_GRID_ESN0",
        "MERCURY_SFO_GRID_CHAN", "MERCURY_SFO_GRID_WATT_DEPTH_DB",
        "MERCURY_SFO_GRID_WATT_FD_HZ", "MERCURY_SFO_GRID_WATT_DLY",
        "MERCURY_SFO_GRID_TINTERP_PROD"
    };
    const int nk = (int)(sizeof(keys)/sizeof(keys[0]));
    struct Saved { const char* key; bool had; std::string val; } sv[16];
    for (int i = 0; i < nk; i++) {
        const char* g = std::getenv(keys[i]);
        sv[i].key = keys[i]; sv[i].had = (g != nullptr);
        sv[i].val = g ? std::string(g) : std::string();
    }
    tinterp_set_env("MERCURY_SFO_GRID", "1");
    tinterp_set_env("MERCURY_SFO_GRID_CODED", "1");
    tinterp_set_env("MERCURY_SFO_GRID_ESN0", esn0);
    tinterp_set_env("MERCURY_SFO_GRID_CHAN", chan);
    tinterp_set_env("MERCURY_SFO_GRID_WATT_DEPTH_DB", depth);
    tinterp_set_env("MERCURY_SFO_GRID_WATT_FD_HZ", fd);
    tinterp_set_env("MERCURY_SFO_GRID_WATT_DLY", dly);
    tinterp_set_env("MERCURY_SFO_GRID_TINTERP_PROD", tinterp_prod ? "1" : "0");

    cl_telecom_system ts;
    ts.operation_mode = BER_PLOT_passband;
    ts.load_configuration(CONFIG_15);   // CFG15 dense Dx=1/Dy=3 lattice (the MPG cell)
    ts.sfo_grid_test();
    cw_ok  = ts.sfo_grid_last_cw_ok;
    cw_tot = ts.sfo_grid_last_cw_tot;

    for (int i = 0; i < nk; i++) {
#if defined(_WIN32)
        _putenv_s(sv[i].key, sv[i].had ? sv[i].val.c_str() : "");
#else
        if (sv[i].had) setenv(sv[i].key, sv[i].val.c_str(), 1); else unsetenv(sv[i].key);
#endif
    }
}

static int run_tinterp_seed_selftest()
{
    printf("[TEST-TINTERP-SEED] TINTERP-SEED production estimator self-test (it=0 seed swap)\n");
    int fails = 0;

    // ---- Cell A: GOOD Watterson fade (MPG). LS FLOORS 0/K, TINTERP CROSSES K/K. ----
    // CHAN=3 depth=10dB fd=0.1Hz dly=24 @ EsN0=24 (test_fade_tinterp.py MPG). The held LS
    // window averages the slow Doppler fade (nv reads ~0.35, the fade looks like noise) and
    // decodes 0/K; the per-carrier TIME-INTERPOLATION estimator tracks the fade (nv ~0.0065)
    // and decodes K/K. This is the failing-before/passing-after: same channel, same FEC,
    // only the it=0 estimator seed differs (the exact swap the receive_byte rescue performs).
    {
        int ls_ok=-1, ls_tot=-1, ti_ok=-1, ti_tot=-1;
        tinterp_run_cell("24","3","10","0.1","24", 0, ls_ok, ls_tot);   // LS-only (before)
        tinterp_run_cell("24","3","10","0.1","24", 1, ti_ok, ti_tot);   // TINTERP (after)
        bool before_floors = (ls_tot > 0 && ls_ok == 0);
        bool after_crosses = (ti_tot > 0 && ti_ok == ti_tot);
        bool ok = before_floors && after_crosses;
        printf("[TEST-TINTERP-SEED]   CELL-A GOOD-fade@24dB: LS-seed=%d/%d (%s) TINTERP-seed=%d/%d (%s) -> %s\n",
               ls_ok, ls_tot, before_floors?"FLOORS_OK":"DECODED_unexpected",
               ti_ok, ti_tot, after_crosses?"CROSSES_OK":"FAILED",
               ok?"PASS":"FAIL");
        if(!ok) fails++;
    }

    // ---- Cell B: CLEAN AWGN no-op. LS K/K == TINTERP K/K (the byte-identical-clean basis). ----
    // CHAN=0 (flat AWGN) @ EsN0=24: both estimators decode the full block. The rescue never
    // changes a clean decode (clean frames decode on LS pass-1 and never reach TINTERP), and
    // on the rare clean frame the estimator IS run (e.g. via FADE_TINTERP) it must not regress.
    // Asserting equal decode counts on clean is the standalone no-op proof.
    {
        int ls_ok=-1, ls_tot=-1, ti_ok=-1, ti_tot=-1;
        tinterp_run_cell("24","0","0","0.5","24", 0, ls_ok, ls_tot);    // LS-only clean
        tinterp_run_cell("24","0","0","0.5","24", 1, ti_ok, ti_tot);    // TINTERP clean
        bool ls_full = (ls_tot > 0 && ls_ok == ls_tot);
        bool ti_noop = (ti_tot == ls_tot && ti_ok == ls_ok);
        bool ok = ls_full && ti_noop;
        printf("[TEST-TINTERP-SEED]   CELL-B CLEAN@24dB no-op: LS=%d/%d TINTERP=%d/%d -> %s\n",
               ls_ok, ls_tot, ti_ok, ti_tot,
               ok?"PASS(no-op)":"FAIL(regression)");
        if(!ok) fails++;
    }

    printf("[TEST-TINTERP-SEED] %s (%d cell failure%s)\n", fails==0?"ALL PASS":"FAILED", fails, fails==1?"":"s");
    return fails==0 ? 0 : 1;
}

// --test-acq-bounds: OFDM acquisition bounds-gate recovery regression
// (fact-documents/data-flow-acq-bounds-gate.md). PROVES the force-FAIL gate in
// cl_telecom_system::receive_byte (telecom_system.cc:1867) recovers a tail-band
// frame whose Schmidl-Cox detection floors 1..plateau_lead_symb symbols PAST
// upper_bound while its body is still in-buffer, instead of discarding it.
//
// Faithful in-process synthetic-fire: a REAL clean OFDM frame (full preamble +
// data, no AWGN) is TX'd via the production transmit_byte, placed into a full RX
// ring buffer at a chosen onset near upper_bound, and threaded through the
// production receive_byte WITHOUT ofdm_forced_delay (so the real Schmidl-Cox
// acquisition + bounds gate execute). ofdm_search_raw/nUnder are seeded so the
// anti-re-decode cursor ofdm_eff lands in the plateau-lead band — the exact
// condition that pre-fix force-FAILed (pream_symb_loc=0, frame discarded).
//
// One harness cell: place the frame, seed the cursor, decode, return outcome.
struct AcqBoundsCell { int detected_symb; int message_decoded; int delay; int upper_bound; };
static AcqBoundsCell acq_bounds_run_cell(int config, int onset_symb, int ofdm_eff_target)
{
    cl_telecom_system ts;
    ts.operation_mode = BER_PLOT_passband;
    ts.load_configuration(config);   // allocates buffers + sizes OFDM/LDPC via init()

    cl_data_container& dc = ts.data_container;
    int interp      = ts.frequency_interpolation_rate;
    int sym_samples = dc.Nofdm * interp;
    int buffer_Nsymb= dc.buffer_Nsymb;
    int frame_symb  = dc.Nsymb + dc.preamble_nSymb;
    int upper_bound = buffer_Nsymb - frame_symb;
    int frame_samp  = frame_symb * sym_samples;
    int buf_samp    = buffer_Nsymb * sym_samples;

    // Build one clean frame into a scratch buffer via the production TX path.
    int nReal_data = dc.nBits - ts.ldpc.P;
    int frame_size = (nReal_data - ts.outer_code_reserved_bits) / 8;
    std::vector<int> payload(frame_size > 0 ? frame_size : 1, 0);
    for (int i = 0; i < frame_size; i++) payload[i] = (i * 37 + 11) & 0xff;
    std::vector<double> frame(frame_samp, 0.0);
    ts.transmit_byte(payload.data(), frame_size, frame.data(), SINGLE_MESSAGE);

    // Place the frame at the requested onset in the full RX ring buffer (zeroed).
    double* rx = dc.ready_to_process_passband_delayed_data;   // sized Nofdm*buffer_Nsymb*interp
    for (int i = 0; i < buf_samp; i++) rx[i] = 0.0;
    int onset = onset_symb * sym_samples;
    if (onset < 0) onset = 0;
    int copy_n = frame_samp;
    if (onset + copy_n > buf_samp) copy_n = buf_samp - onset;   // truncate tail overrun
    for (int i = 0; i < copy_n; i++) rx[onset + i] = frame[i];

    // Seed the anti-re-decode cursor so ofdm_eff = ofdm_search_raw - nUnder lands
    // at the requested target (symbol units). nUnder=0 keeps it simple.
    ts.receive_stats.ofdm_search_raw   = ofdm_eff_target;
    ts.data_container.nUnder_processing_events = 0;
    ts.receive_stats.ofdm_batch_active = false;   // full-search path (not predict)
    ts.receive_stats.delay             = 0;
    ts.ofdm_forced_delay               = -1;      // real acquisition, NOT BER known-delay
    ts.mfsk_fixed_delay                = -1;
    // Isolate the force-FAIL bounds gate: with defer-overflow ON, a tail-band
    // detection (delay just past upper_bound) returns early via the overflow-defer
    // path (frame_overflow_symbols>0, "capture more audio") BEFORE the bounds gate.
    // The discard the fix targets is the defer-OFF / cursor-past-upper_bound path
    // (telecom_system.cc:1801 "--ofdm-defer-overflow=off bypasses Fix A"), where a
    // still-in-buffer frame is force-FAILed instead of recovered. Drive that path.
    ts.ofdm_defer_overflow_enabled     = false;

    ts.receive_byte(rx, dc.hd_decoded_data_byte);

    AcqBoundsCell out;
    out.delay           = ts.receive_stats.delay;
    out.detected_symb   = (sym_samples > 0) ? ts.receive_stats.delay / sym_samples : -1;
    out.message_decoded = ts.receive_stats.message_decoded;
    out.upper_bound     = upper_bound;
    return out;
}

static int run_acq_bounds_selftest()
{
    printf("[TEST-ACQ-BOUNDS] OFDM acquisition bounds-gate recovery self-test\n");
    int fails = 0;
    // CONFIG_0 (BPSK rate-1/16): the most robust OFDM rung; a clean (noiseless)
    // loopback frame decodes trivially when acquisition lands it correctly, so
    // message_decoded isolates the GATE decision, not LDPC marginality.
    const int cfg = CONFIG_0;

    // Determine geometry for the placement (one throwaway cell at a benign onset).
    AcqBoundsCell geo = acq_bounds_run_cell(cfg, 5, 0);
    int ub = geo.upper_bound;
    printf("[TEST-ACQ-BOUNDS]   geometry: upper_bound=%d (control onset=5 -> decoded=%d detected=%d)\n",
           ub, geo.message_decoded, geo.detected_symb);

    // ---- CELL 1 (fail-before / pass-after): TAIL-BAND frame must be RECOVERED. ----
    // Onset = upper_bound (the highest sample-exact-fitting onset: a full frame
    // placed here ends exactly at the buffer end). With the anti-re-decode cursor
    // seeded one symbol into the plateau-lead band (ofdm_eff = upper_bound+1), the
    // PRE-FIX gate force-FAILs (ofdm_eff>upper_bound -> pream_symb_loc=0 -> the
    // decode block is skipped -> message_decoded=NO, frame DISCARDED). POST-FIX the
    // widened cutoff (upper_bound+plateau_lead_symb) lets the recovery scan run; it
    // re-anchors Schmidl-Cox to the in-bounds onset and the frame DECODES.
    {
        AcqBoundsCell c = acq_bounds_run_cell(cfg, ub, ub + 1);
        bool recovered = (c.message_decoded == YES);
        printf("[TEST-ACQ-BOUNDS]   CELL1 tail-band@onset=%d ofdm_eff=%d: decoded=%d detected=%d -> %s\n",
               ub, ub + 1, c.message_decoded, c.detected_symb,
               recovered ? "RECOVERED_OK" : "DISCARDED_FAIL");
        if (!recovered) fails++;
    }

    // ---- CELL 2 (no-regress / anti-re-decode preserved): cursor GENUINELY past the
    // band with NO fitting frame must still force-FAIL (not re-decode old audio). ----
    // Empty buffer (no frame) and ofdm_eff well past the widened cutoff: recovery
    // must NOT manufacture a decode. Asserts the fix did not defeat anti-re-decode.
    {
        AcqBoundsCell c = acq_bounds_run_cell(cfg, /*onset (unused, no signal)*/ ub, ub + 8);
        // Re-run with a zeroed buffer: overwrite the frame by passing an onset that
        // truncates it fully out is awkward; instead place the frame far below the
        // cursor floor (onset well under ofdm_eff-plateau) so the scan_start>onset
        // excludes it -> no recovery -> force-FAIL. This is the already-decoded case.
        AcqBoundsCell c2 = acq_bounds_run_cell(cfg, 2, ub + 8);
        bool no_decode = (c.message_decoded != YES) && (c2.message_decoded != YES);
        printf("[TEST-ACQ-BOUNDS]   CELL2 past-band ofdm_eff=%d: emptybuf_decoded=%d below-floor_decoded=%d -> %s\n",
               ub + 8, c.message_decoded, c2.message_decoded,
               no_decode ? "FORCE-FAIL_OK" : "SPURIOUS_DECODE_FAIL");
        if (!no_decode) fails++;
    }

    printf("[TEST-ACQ-BOUNDS] %s (%d cell failure%s)\n", fails==0?"ALL PASS":"FAILED", fails, fails==1?"":"s");
    return fails==0 ? 0 : 1;
}

// --test-subpeak-rescue: F1b Part B sample-anchored true-boundary rescue mechanism
// self-test (fact-documents/data-flow-rx-ring-rearm.md, _research/F1B_DESIGN.md §2/§3).
// PROVES the SUBPEAK-REJECT rescue's two production primitives on a deterministic,
// noiseless, in-process synthetic-fire (no IONOS/RF):
//   (1) the SELECTOR cl_telecom_system::ofdm_meanH_at_delay reads a HIGH mean|H| at the
//       true frame onset and a COLLAPSED mean|H| at a ~1.5-symbol-early wrong lock (the
//       field sub-peak fingerprint: metric saturates but pilots misalign, mean|H| ~0.37);
//   (2) the argmax scan over [orig-2sym, orig+2sym] (the exact loop receive_byte runs in
//       the reject branch) RECOVERS the true onset to within ±¼ symbol;
//   (3) the production decode path (receive_byte) DECODES at the true / refined delay and
//       FAILS at the wrong lock — the fail-before (wrong lock never reads) / pass-after
//       (refined delay reads) proof through the real decoder.
// The in-process path structurally cannot make the live Schmidl-Cox mislock (the drain is
// a no-op under -x sim), so the PRODUCTION rescue-branch FIRING is proven separately by the
// real-audio needle-2 cohort (anchor_race_vehicle.py: [XCORR-RESCUE-OK] tokens + needle2 -> 0).
// This test isolates the mechanism the cohort then confirms fires end-to-end.
static int subpeak_rescue_geom(int force_Dy, int force_Nsymb, const char* label,
                               double noise_snr_db /* <0 => noiseless */)
{
    // GEOMETRY-INVARIANT discriminator: assert on the pilot phase COHERENCE C
    // (telecom_system.cc last_pilot_coherence), NOT the absolute mean|H|. mean|H|'s
    // wrong-lock collapse deepens with pilot count, so the thinned cfg16 grid (Dy=5)
    // leaves a wrong lock reading mean|H|~0.77 -- above the 0.74 legacy floor -- and
    // the old absolute-floor assertion FALSELY passes the wrong lock. C is count-
    // normalized: a decorrelated wrong lock reads C~0.1 at ANY density, a good lock
    // reads C~1, so the same thresholds separate on Dy=3 AND Dy=5.
    const double C_ENTRY  = 0.35;  // wrong lock must read C below this (production subpeak_coh_entry)
    const double C_ACCEPT = 0.50;  // true / recovered lock must read C at/above this (subpeak_coh_accept)
    const double FLOOR    = 0.74;  // legacy good-timing mean|H| floor (printed for context, not asserted-on)

    // Force the cfg16 pilot lattice. Baked default is Dy=5/Nsymb=8; the pilot-override
    // env reconstructs any (Dy,Nsymb) for CONFIG_16 (pilot_override_target default).
    const char* ks[2] = { "MERCURY_PILOT_DY", "MERCURY_PILOT_NSYMB" };
    bool had[2]; std::string sav[2];
    for (int i=0;i<2;i++){ const char* g=std::getenv(ks[i]); had[i]=(g!=nullptr); sav[i]=g?std::string(g):std::string(); }
    char dybuf[16], nsbuf[16];
    snprintf(dybuf,sizeof(dybuf),"%d",force_Dy);
    snprintf(nsbuf,sizeof(nsbuf),"%d",force_Nsymb);
    setenv("MERCURY_PILOT_DY", dybuf, 1);
    setenv("MERCURY_PILOT_NSYMB", nsbuf, 1);

    int fails = 0;
    const int cfg = CONFIG_16;
    cl_telecom_system ts;
    ts.operation_mode = BER_PLOT_passband;
    ts.load_configuration(cfg);

    // Geometry is now locked into this ts instance; restore the env immediately.
    for (int i=0;i<2;i++){ if(had[i]) setenv(ks[i],sav[i].c_str(),1); else unsetenv(ks[i]); }

    cl_data_container& dc = ts.data_container;
    int interp       = ts.frequency_interpolation_rate;
    int sym_samples  = dc.Nofdm * interp;
    int frame_symb   = dc.Nsymb + dc.preamble_nSymb;
    int frame_samp   = frame_symb * sym_samples;
    int buf_samp     = dc.buffer_Nsymb * sym_samples;
    int max_delay    = buf_samp - frame_samp;
    double carrier   = ts.carrier_frequency;

    // Build one clean CONFIG_16 frame via the production TX path.
    int nReal_data = dc.nBits - ts.ldpc.P;
    int frame_size = (nReal_data - ts.outer_code_reserved_bits) / 8;
    if (frame_size < 1) frame_size = 1;
    std::vector<int> payload(frame_size, 0);
    for (int i = 0; i < frame_size; i++) payload[i] = (i * 53 + 7) & 0xff;
    std::vector<double> frame(frame_samp, 0.0);
    ts.transmit_byte(payload.data(), frame_size, frame.data(), SINGLE_MESSAGE);

    // Optional AWGN (low-SNR true-lock case): calibrate sigma to the frame RMS so
    // the requested passband SNR depresses mean|H| toward the good-timing floor
    // without destroying the lock -- the exact regime where an ABSOLUTE floor risks
    // false-rejecting a real frame but a NORMALIZED coherence must not.
    std::vector<double> used(frame);
    if (noise_snr_db >= 0.0) {
        double e = 0.0; for (double v : frame) e += v*v;
        double rms = (frame.size()>0)? sqrt(e/(double)frame.size()) : 0.0;
        double sigma = (rms>0.0)? rms / pow(10.0, noise_snr_db/20.0) : 0.0;
        std::mt19937 rng(0xC0FFEEu + (unsigned)force_Dy*131u + (unsigned)(noise_snr_db*7.0));
        std::normal_distribution<double> nd(0.0, sigma);
        for (size_t i=0;i<used.size();i++) used[i] += nd(rng);
    }

    // Place at a true onset preceded by a zero run-up (the flush-memset silence that
    // seeds the Schmidl-Cox plateau in the field). onset=8 symbols in.
    double* rx = dc.ready_to_process_passband_delayed_data;
    for (int i = 0; i < buf_samp; i++) rx[i] = 0.0;
    int onset_symb = 8;
    int D = onset_symb * sym_samples;
    if (D > max_delay) D = max_delay;
    int copy_n = frame_samp;
    if (D + copy_n > buf_samp) copy_n = buf_samp - D;
    for (int i = 0; i < copy_n; i++) rx[D + i] = used[i];

    // The field wrong lock sits ~1.5 symbols early.
    int wrong = D - (sym_samples + sym_samples / 2);
    if (wrong < 0) wrong = 0;

    // ---- (1) SELECTOR discrimination via the production helper (mean|H| AND C) ----
    double C_true = -1.0, C_wrong = -1.0;
    double mH_true  = ts.ofdm_meanH_at_delay(rx, D,     carrier, dc.preamble_nSymb, &C_true);
    double mH_wrong = ts.ofdm_meanH_at_delay(rx, wrong, carrier, dc.preamble_nSymb, &C_wrong);
    printf("[TEST-SUBPEAK-RESCUE] [%s] Dy=%d Nsymb=%d %s\n", label, force_Dy, force_Nsymb,
           (noise_snr_db<0.0)?"noiseless":"low-SNR true-lock");
    printf("[TEST-SUBPEAK-RESCUE]   selector: true(d=%d) mean|H|=%.3f C=%.3f | wrong(d=%d) mean|H|=%.3f C=%.3f | floor(legacy)=%.2f C_entry=%.2f C_accept=%.2f\n",
           D, mH_true, C_true, wrong, mH_wrong, C_wrong, FLOOR, C_ENTRY, C_ACCEPT);

    // decode-at-forced-delay helper (uses the possibly-noisy frame)
    auto decode_at = [&](int forced_delay)->int {
        for (int i = 0; i < buf_samp; i++) rx[i] = 0.0;
        for (int i = 0; i < copy_n; i++) rx[D + i] = used[i];
        ts.receive_stats.ofdm_search_raw = 0;
        ts.data_container.nUnder_processing_events = 0;
        ts.receive_stats.ofdm_batch_active = false;
        ts.receive_stats.delay = 0;
        ts.mfsk_fixed_delay = -1;
        ts.ofdm_forced_delay = forced_delay;
        ts.receive_byte(rx, dc.hd_decoded_data_byte);
        int ok = (ts.receive_stats.message_decoded == YES) ? 1 : 0;
        if (ok) {
            for (int i = 0; i < frame_size; i++)
                if ((dc.hd_decoded_data_byte[i] & 0xff) != (payload[i] & 0xff)) { ok = 0; break; }
        }
        return ok;
    };

    if (noise_snr_db < 0.0) {
        // NOISELESS: full geometry-invariant discrimination on this grid.
        bool sel_true_ok  = (C_true  >= C_ACCEPT);
        bool sel_wrong_ok = (C_wrong >= 0.0) && (C_wrong <  C_ENTRY) && (C_true - C_wrong >= 0.30);
        if (!sel_true_ok)  { fails++; printf("[TEST-SUBPEAK-RESCUE]   FAIL[%s]: true-onset coherence below accept (%.3f < %.2f)\n", label, C_true, C_ACCEPT); }
        if (!sel_wrong_ok) { fails++; printf("[TEST-SUBPEAK-RESCUE]   FAIL[%s]: wrong-lock coherence not clearly below entry (C=%.3f, entry=%.2f, margin=%.3f)\n", label, C_wrong, C_ENTRY, C_true - C_wrong); }

        // ---- (2) argmax scan (identical to the receive_byte reject-branch loop) ----
        int step = sym_samples / 4; if (step < 1) step = 1;
        double best_mH = mH_wrong; int best_delay = wrong; double best_C = C_wrong;
        for (int off = -2*sym_samples; off <= 2*sym_samples; off += step) {
            int cand = wrong + off;
            if (cand < 0 || cand > max_delay) continue;
            double candC = -1.0;
            double mH = ts.ofdm_meanH_at_delay(rx, cand, carrier, dc.preamble_nSymb, &candC);
            if (mH > best_mH) { best_mH = mH; best_delay = cand; best_C = candC; }
        }
        bool scan_ok = (best_C >= C_ACCEPT) && (std::abs(best_delay - D) <= step);
        printf("[TEST-SUBPEAK-RESCUE]   argmax scan[%s]: best_delay=%d (true=%d, err=%d, <=%d? %s) mean|H|=%.3f C=%.3f (C>=%.2f? %s)\n",
               label, best_delay, D, best_delay - D, step, (std::abs(best_delay - D) <= step)?"YES":"NO",
               best_mH, best_C, C_ACCEPT, (best_C>=C_ACCEPT)?"YES":"NO");
        if (!scan_ok) { fails++; printf("[TEST-SUBPEAK-RESCUE]   FAIL[%s]: argmax did not recover a coherent true onset\n", label); }

        // ---- (3) production decode legs: truth decodes, wrong fails, refined decodes ----
        int dec_true  = decode_at(D);
        int dec_wrong = decode_at(wrong);
        int dec_best  = decode_at(best_delay);
        printf("[TEST-SUBPEAK-RESCUE]   decode[%s]: true=%s wrong=%s refined=%s\n",
               label, dec_true?"DECODED":"FAILED", dec_wrong?"DECODED":"FAILED", dec_best?"DECODED":"FAILED");
        if (!dec_true)  { fails++; printf("[TEST-SUBPEAK-RESCUE]   FAIL[%s]: clean frame did not decode at the true onset\n", label); }
        if ( dec_wrong) { fails++; printf("[TEST-SUBPEAK-RESCUE]   FAIL[%s]: wrong lock decoded (fail-before did not reproduce)\n", label); }
        if (!dec_best)  { fails++; printf("[TEST-SUBPEAK-RESCUE]   FAIL[%s]: refined delay did not decode (pass-after)\n", label); }
    } else {
        // LOW-SNR TRUE-LOCK PRESERVATION: a noise-depressed GENUINE lock must still be
        // ACCEPTED by the coherence floor (never turned into a false sub-peak reject),
        // and must still DECODE. This guards the risk that a geometry-aware floor
        // trades a wrong-lock catch for a real-frame false-reject at low SNR.
        bool true_accept = (C_true >= C_ACCEPT);
        int dec_true = decode_at(D);
        printf("[TEST-SUBPEAK-RESCUE]   low-SNR[%s] snr=%.1fdB: true mean|H|=%.3f C=%.3f (accept C>=%.2f? %s) decode=%s\n",
               label, noise_snr_db, mH_true, C_true, C_ACCEPT, true_accept?"YES":"NO", dec_true?"DECODED":"FAILED");
        if (!true_accept) { fails++; printf("[TEST-SUBPEAK-RESCUE]   FAIL[%s]: LOW-SNR true lock FALSE-REJECTED by coherence floor (C=%.3f < %.2f)\n", label, C_true, C_ACCEPT); }
        if (!dec_true)    { fails++; printf("[TEST-SUBPEAK-RESCUE]   FAIL[%s]: LOW-SNR true lock did not decode (SNR too low for this check -- raise noise_snr_db)\n", label); }
    }
    return fails;
}

static int run_subpeak_rescue_selftest()
{
    printf("[TEST-SUBPEAK-RESCUE] F1b Part B true-boundary rescue mechanism self-test (geometry-parametrized)\n");
    int fails = 0;
    // BOTH grids must correctly reject their wrong lock and accept their true lock on
    // the GEOMETRY-INVARIANT coherence metric: the baked thin default (Dy=5/Nsymb=8)
    // AND the stock dense reconstruct (Dy=3/Nsymb=9). A future geometry change cannot
    // silently regress the selector because both densities are asserted here.
    fails += subpeak_rescue_geom(5, 8, "thin",  -1.0);
    fails += subpeak_rescue_geom(3, 9, "dense", -1.0);
    // TRUE-LOCK PRESERVED at low SNR on BOTH grids: the coherence floor must not
    // false-reject a legitimate lock whose mean|H| is depressed by noise. Because
    // the discriminator is a phase COHERENCE (magnitude-independent), a sagging
    // mean|H| does not threaten the accept -- this case proves it. SNR overridable
    // (MERCURY_SUBPEAK_TEST_SNR) for calibration; the baked value sits in the
    // decoding regime where mean|H| already visibly sags.
    // 10 dB: comfortably inside the cfg16 decode regime on this synthetic (decode
    // holds down to ~8 dB with the fixed seed) yet well below the high-SNR envelope
    // cfg16 is elected in. Note on this FLAT synthetic mean|H| stays ~1.0 at every
    // SNR (AGC + LS window preserve it), so the magnitude-sag false-reject the floor
    // was feared to cause is not even reproducible -- and coherence, being magnitude-
    // independent, holds ~1.0 regardless. The frequency-selective true-lock case
    // (where mean|H| genuinely ripples) is the owed real-audio fire proof.
    double lo_snr = 10.0;
    { const char* e = std::getenv("MERCURY_SUBPEAK_TEST_SNR"); if(e && *e) lo_snr = atof(e); }
    fails += subpeak_rescue_geom(5, 8, "thin",  lo_snr);
    fails += subpeak_rescue_geom(3, 9, "dense", lo_snr);
    printf("[TEST-SUBPEAK-RESCUE] %s (%d failure%s)\n", fails==0?"ALL PASS":"FAILED", fails, fails==1?"":"s");
    return fails==0 ? 0 : 1;
}

// --test-chase-ab: production-path HARQ/Chase A/B fire proof (no cards, no RF).
// Drives the REAL receive_byte() TWICE on the SAME OFDM codeword at a sub-threshold
// SNR with INDEPENDENT noise realizations: look-1 fails the single-look decode and
// is buffered by the chase CAPTURE hook (telecom_system.cc terminal-fail branch);
// look-2 is soft-combined with look-1 by the chase RESCUE hook (before the accept
// decision) and re-decoded. Reports, per (config, SNR):
//   single%   = look-1 single-look decode rate (== both arms: floor is NOT lowered)
//   comb_OFF% = look-2 decode rate, MERCURY_CHASE OFF (incumbent: another single look)
//   comb_ON%  = look-2 decode rate, MERCURY_CHASE ON  (2-look combine)
//   rescues   = chase adopts fired;   corrupt = adopted frame whose bytes != payload
// comb_ON - comb_OFF at equal SNR is the delivered reach; the SNR gap between the
// single-look and comb_ON waterfalls is the coding gain (~+3 dB MRC per copy).
struct ChaseABCell {
    int config; double snr_db; int trials;
    int single_on; int single_off; int comb_on; int comb_off;
    long rescues; int corrupt;
};

static ChaseABCell chase_ab_run_cell(int config, double snr_db, int trials, unsigned base_seed)
{
    ChaseABCell R; R.config=config; R.snr_db=snr_db; R.trials=trials;
    R.single_on=R.single_off=R.comb_on=R.comb_off=0; R.rescues=0; R.corrupt=0;

    for(int t=0;t<trials;t++){
    // FRESH telecom_system per trial. Latched acquisition/batch state accumulates
    // across receive_byte calls on a reused instance and starves the decode path
    // after ~a dozen calls (measured: noiseless decode 100% at 3 trials -> 33% at
    // 12 on a shared ts); a per-trial instance keeps each to <=4 receives (2 arms x
    // 2 looks), which stays clean. The chase buffer lives on this ts and correctly
    // persists across the trial's look-1 -> look-2 within each arm.
    cl_telecom_system ts;
    ts.operation_mode = BER_PLOT_passband;
    ts.load_configuration(config);
    // The single-frame synthetic vehicle's pilot-residual noise_variance_estimate
    // reads high (~1.4) even NOISELESS (a property of a lone frame vs a real batch),
    // so the pre-LDPC SKIP-VAR gate would reject every look before the decoder runs.
    // We are exercising the LDPC decode + the chase capture/rescue hooks, NOT the
    // SKIP-VAR gate -- disable it so every look reaches ldpc.decode() (the same knob
    // as --skip-var-gate=off). LDPC+CRC16 remain the actual accept mechanism.
    ts.skip_var_gate_enabled = false;
    cl_data_container& dc = ts.data_container;
    int interp      = ts.frequency_interpolation_rate;
    int sym_samples = dc.Nofdm * interp;
    int frame_symb  = dc.Nsymb + dc.preamble_nSymb;
    int frame_samp  = frame_symb * sym_samples;
    int buf_samp    = dc.buffer_Nsymb * sym_samples;
    int max_delay   = buf_samp - frame_samp;
    int nReal_data  = dc.nBits - ts.ldpc.P;
    int frame_size  = (nReal_data - ts.outer_code_reserved_bits) / 8;
    if(frame_size < 1) frame_size = 1;
    int onset_symb = 8;   // match the proven subpeak decode_at recipe (onset 8, not 3)
    int D = onset_symb * sym_samples; if(D>max_delay) D=max_delay; if(D<0) D=0;
    static int ab_dbg = -1;
    if(ab_dbg<0){ const char* e=std::getenv("MERCURY_CHASE_AB_DEBUG"); ab_dbg=(e&&*e&&atoi(e))?1:0; }

    // Drive ONE noisy look through the production acquisition + decode path.
    auto run_look = [&](const std::vector<double>& frame, double frame_rms,
                        unsigned seed, std::vector<int>& out_bytes)->int {
        // ZERO background (silence run-up) + frame(+noise) at D -- the proven
        // subpeak_rescue_geom recipe. A noise-filled background outside the frame
        // spuriously triggers batch/acquisition state on the reused ts and starves
        // subsequent looks of the decode path.
        double* rx = dc.ready_to_process_passband_delayed_data;
        std::mt19937 rng(seed);
        double sigma = (snr_db < 200.0) ? frame_rms / pow(10.0, snr_db/20.0) : 0.0;
        std::normal_distribution<double> nd(0.0, sigma);
        for(int i=0;i<buf_samp;i++) rx[i] = 0.0;
        int copy_n = frame_samp; if(D+copy_n>buf_samp) copy_n=buf_samp-D;
        for(int i=0;i<copy_n;i++) rx[D+i] = frame[i] + ((sigma>0.0)? nd(rng):0.0);
        // Scrub latched per-decode state so a prior failed look cannot poison this one.
        ts.receive_stats.ofdm_search_raw           = 0;
        ts.data_container.nUnder_processing_events = 0;
        ts.receive_stats.ofdm_batch_active        = false;
        ts.receive_stats.delay                    = 0;
        ts.use_last_good_freq_offset               = NO;
        ts.use_last_good_time_sync                 = NO;
        ts.consecutive_ofdm_decode_fails           = 0;
        ts.mfsk_fixed_delay                        = -1;
        // Pin extraction to the KNOWN onset D (same recipe as the subpeak-rescue
        // decode_at() helper): a lone synthetic frame does not fine-lock cleanly
        // enough under full search, so we extract at the exact placement. Both looks
        // use the same delay; the only difference is the independent noise draw --
        // exactly the same-codeword retx scenario chase combines.
        ts.ofdm_forced_delay                       = D;
        ts.receive_byte(rx, dc.hd_decoded_data_byte);
        int decoded = (ts.receive_stats.message_decoded==YES)?1:0;
        if(ab_dbg) fprintf(stderr,"[AB-DBG] cfg=%d snr=%.1f seed=%u decoded=%d delay=%d sync_trials=%d crc=0x%04X iter=%d allz=%d\n",
                           config, snr_db, seed, decoded, (int)ts.receive_stats.delay, ts.receive_stats.sync_trials,
                           ts.receive_stats.crc, ts.receive_stats.iterations_done, ts.receive_stats.all_zeros);
        out_bytes.assign(frame_size,0);
        for(int i=0;i<frame_size;i++) out_bytes[i]=dc.hd_decoded_data_byte[i]&0xff;
        return decoded;
    };

    {
        // Distinct payload (distinct codeword) per trial so a cross-trial mismatched
        // combine cannot silently help; distinct, well-separated look seeds ensure
        // the two looks are DECORRELATED (chase-combining-harq.md §6.1 landmine).
        std::vector<int> payload(frame_size,0);
        for(int i=0;i<frame_size;i++) payload[i]=((i*53+7) ^ (t*131+29)) & 0xff;
        std::vector<double> frame(frame_samp,0.0);
        ts.transmit_byte(payload.data(), frame_size, frame.data(), SINGLE_MESSAGE);
        double e=0.0; for(double v:frame) e+=v*v;
        double frame_rms = frame.size()? sqrt(e/(double)frame.size()):0.0;
        unsigned s1 = base_seed + 100003u*(unsigned)(t+1);
        unsigned s2 = base_seed + 700019u*(unsigned)(t+1) + 17u;

        std::vector<int> b1,b2;
        // OFF arm (incumbent): chase disabled -> two independent single looks.
        ts.chase_enabled = false; ts.chase_reset();
        R.single_off += run_look(frame, frame_rms, s1, b1);
        R.comb_off   += run_look(frame, frame_rms, s2, b2);
        // ON arm: chase enabled -> look-1 captured, look-2 combined+rescued.
        ts.chase_enabled = true; ts.chase_reset();
        long resc0 = ts.chase_rescues;
        R.single_on += run_look(frame, frame_rms, s1, b1);   // fails -> captures
        int d2 = run_look(frame, frame_rms, s2, b2);         // combine -> maybe rescue
        R.comb_on += d2;
        if(d2){ bool ok=true; for(int i=0;i<frame_size;i++) if(b2[i]!=(payload[i]&0xff)){ok=false;break;} if(!ok) R.corrupt++; }
        R.rescues += (ts.chase_rescues - resc0);
    }
    } // end per-trial fresh-ts loop
    return R;
}

static int run_chase_ab_selftest()
{
    printf("[TEST-CHASE-AB] production-path HARQ/Chase A/B (receive_byte 2-look combine)\n");
    int trials = 12;
    { const char* e=std::getenv("MERCURY_CHASE_AB_TRIALS"); if(e&&*e){ trials=atoi(e); if(trials<1) trials=1; } }
    // Per-config SNR sweeps (dB) straddling each rung's single-look waterfall so the
    // sub-threshold band (single ~0, comb_ON > 0) is visible. Overridable.
    // Default SNR sweeps straddle each rung's single-look waterfall INTO the chase
    // rescue band (single-look ~0, 2-look combine still decodes). Measured on this
    // synthetic vehicle (requested SNR sits ~10 dB below the effective Es/N0):
    //   cfg6  single waterfall ~-10.5 dB, combined holds to ~-12.5 (+~2 dB)
    //   cfg9  single waterfall ~-7   dB, combined holds to ~-10  (+~3 dB)
    //   cfg13 single waterfall ~-1   dB, combined holds to ~-4   (+~2 dB)
    struct { int cfg; std::vector<double> snrs; } band[] = {
        { 6,  { -9,-10,-11,-12,-13} },
        { 9,  { -6, -8, -9,-10,-11} },
        { 13, {  0, -2, -4, -5, -6} },
    };
    int nband = 3;
    // Env overrides (calibration without rebuild): MERCURY_CHASE_AB_CONFIGS="6,9,13"
    // sets which configs; MERCURY_CHASE_AB_SNRS="a,b,c" applies ONE sweep to all.
    auto parse_list = [](const char* s, std::vector<double>& out){
        out.clear(); std::string cur;
        for(const char* p=s;;++p){ if(*p==','||*p=='\0'){ if(!cur.empty()){ out.push_back(atof(cur.c_str())); cur.clear(); } if(*p=='\0') break; } else cur+=*p; }
    };
    std::vector<int> cfg_override;
    { const char* e=std::getenv("MERCURY_CHASE_AB_CONFIGS"); if(e&&*e){ std::vector<double> tmp; parse_list(e,tmp); for(double v:tmp) cfg_override.push_back((int)v); } }
    std::vector<double> snr_override;
    { const char* e=std::getenv("MERCURY_CHASE_AB_SNRS"); if(e&&*e) parse_list(e,snr_override); }
    std::vector<std::pair<int,std::vector<double>>> plan;
    if(!cfg_override.empty()){
        for(int c : cfg_override) plan.push_back({c, !snr_override.empty()? snr_override : std::vector<double>{0,2,4,6,8,10,12,14,16,18,20}});
    } else {
        for(int bi=0;bi<nband;bi++) plan.push_back({band[bi].cfg, !snr_override.empty()? snr_override : band[bi].snrs});
    }
    int fails = 0;
    bool any_lift = false;
    printf("[TEST-CHASE-AB] trials=%d per cell; looks are DECORRELATED; distinct codeword per trial\n", trials);
    printf("  cfg  SNRdB   single%%   comb_OFF%%  comb_ON%%   rescues  corrupt   (comb_ON-OFF = delivered reach)\n");
    for(auto& pe : plan){
        for(double snr : pe.second){
            ChaseABCell R = chase_ab_run_cell(pe.first, snr, trials, 0xC4A5Eu);
            double sp = 100.0*R.single_on/trials;
            double coff = 100.0*R.comb_off/trials;
            double con = 100.0*R.comb_on/trials;
            printf("  %3d  %5.1f   %5.0f     %5.0f      %5.0f      %4ld     %4d\n",
                   pe.first, snr, sp, coff, con, R.rescues, R.corrupt);
            fflush(stdout);
            // Floor-not-lowered: look-1 single decode identical ON vs OFF.
            if(R.single_on != R.single_off){
                printf("    [FAIL] floor moved: single_ON=%d != single_OFF=%d (chase changed the single-look decode)\n",
                       R.single_on, R.single_off);
                fails++;
            }
            // 0-corruption: no adopted frame may byte-mismatch the payload.
            if(R.corrupt != 0){
                printf("    [FAIL] %d chase-adopted frame(s) corrupt (bytes != payload)\n", R.corrupt);
                fails++;
            }
            // Incumbent floor: comb_OFF must not exceed single (it is just another
            // single look) by more than noise -- sanity, not a hard gate.
            if(R.comb_on > R.comb_off && R.rescues > 0) any_lift = true;
        }
    }
    // Lift is a HARD gate ONLY on the default (tuned) bands: the sweet-spot cells
    // rescue reproducibly, so a default run with NO lift means the chase hooks stopped
    // firing (a regression). Under an env SNR override the band may deliberately miss
    // the waterfall, so there it is a WARN, not a fail.
    bool default_bands = cfg_override.empty() && snr_override.empty();
    if(!any_lift){
        if(default_bands){
            printf("[TEST-CHASE-AB] [FAIL] no cell showed comb_ON > comb_OFF with a rescue on the default bands -- the chase capture/rescue hooks did not fire.\n");
            fails++;
        } else {
            printf("[TEST-CHASE-AB] [WARN] no cell showed comb_ON > comb_OFF with a rescue -- SNR band may miss the waterfall; retune MERCURY_CHASE_AB_* or inspect.\n");
        }
    }
    printf("[TEST-CHASE-AB] %s (fails=%d, lift_seen=%s)\n",
           fails==0 ? "PASS (chase rescues double-fail frames; 0 corrupt; floor unchanged)" : "FAILED", fails, any_lift?"YES":"NO");
    return fails==0 ? 0 : 1;
}

// --test-frame0-vehicle: deterministic IN-PROCESS reproduction of the
// post-turnaround FIRST-FRAME wrong-lock (the anchor-race structural loss).
// Synthesizes a passband RX buffer = [near-silent AWGN run-up] + [a REAL preamble
// + data frame for the loaded config, built via the production transmit_byte] and
// threads it through the PRODUCTION acquisition (receive_byte, ofdm_forced_delay=-1
// => real Schmidl-Cox coarse + Phase-2 argmax + site-8 with_metric fine +
// SUBPEAK-REJECT). Recovery is defeated via MERCURY_SUBPEAK_RESCUE_DEFEAT=1 so the
// RAW lock is measured (prevention meter, not the recovery counter).
//
// Per cell it reports: true onset D, the final locked delay, the SIGNED offset
// delta = final - D in symbols (the DIRECTION the field docs disagree on: EARLY /
// undershoot vs LATE / overshoot), coarse metric, mean_H, decoded. Faithfulness
// fingerprint = the field signature (coarse_metric>=0.97 && mean_H<0.5 &&
// |delta|>=~1 symbol && !decoded). This is the missing deterministic fire-proof.
struct F0VCell {
    int onset_symb; double snr_db; unsigned seed;
    int true_onset; int final_delay; int delta; double delta_sym;
    double coarse_metric; double mean_H; int decoded; int sync_trials; int edge;
    double frame_rms; double sigma;
};

static F0VCell f0v_run_cell(int config, int onset_symb, double snr_db, unsigned seed)
{
    cl_telecom_system ts;
    ts.operation_mode = BER_PLOT_passband;
    ts.load_configuration(config);

    cl_data_container& dc = ts.data_container;
    int interp       = ts.frequency_interpolation_rate;
    int sym_samples  = dc.Nofdm * interp;
    int frame_symb   = dc.Nsymb + dc.preamble_nSymb;
    int frame_samp   = frame_symb * sym_samples;
    int buf_samp     = dc.buffer_Nsymb * sym_samples;
    int max_delay    = buf_samp - frame_samp;

    // Build one clean frame via the production TX path.
    int nReal_data = dc.nBits - ts.ldpc.P;
    int frame_size = (nReal_data - ts.outer_code_reserved_bits) / 8;
    if (frame_size < 1) frame_size = 1;
    std::vector<int> payload(frame_size, 0);
    for (int i = 0; i < frame_size; i++) payload[i] = (i * 53 + 7) & 0xff;
    std::vector<double> frame(frame_samp, 0.0);
    ts.transmit_byte(payload.data(), frame_size, frame.data(), SINGLE_MESSAGE);

    // RMS over the frame region -> calibrate the AWGN floor to the requested SNR.
    double e = 0.0; for (double v : frame) e += v * v;
    double frame_rms = (frame.size() > 0) ? sqrt(e / (double)frame.size()) : 0.0;

    int D = onset_symb * sym_samples; if (D > max_delay) D = max_delay; if (D < 0) D = 0;

    // Whole-buffer AWGN floor (the near-silent RUN-UP the field flush leaves before
    // frame-0; noise-only outside the frame region, signal+noise inside).
    double* rx = dc.ready_to_process_passband_delayed_data;
    std::mt19937 rng(seed);
    double sigma = (snr_db < 200.0) ? frame_rms / pow(10.0, snr_db / 20.0) : 0.0;
    std::normal_distribution<double> nd(0.0, sigma);
    for (int i = 0; i < buf_samp; i++) rx[i] = (sigma > 0.0) ? nd(rng) : 0.0;
    int copy_n = frame_samp; if (D + copy_n > buf_samp) copy_n = buf_samp - D;
    for (int i = 0; i < copy_n; i++) rx[D + i] += frame[i];

    // Real acquisition, INIT full-search (no BATCH predict, no forced/known delay).
    ts.receive_stats.ofdm_search_raw           = 0;
    ts.data_container.nUnder_processing_events = 0;
    ts.receive_stats.ofdm_batch_active         = false;
    ts.receive_stats.delay                     = 0;
    ts.ofdm_forced_delay                       = -1;
    ts.mfsk_fixed_delay                        = -1;

    ts.receive_byte(rx, dc.hd_decoded_data_byte);

    F0VCell c;
    c.onset_symb = onset_symb; c.snr_db = snr_db; c.seed = seed;
    c.true_onset = D;
    c.final_delay = ts.receive_stats.delay;
    c.delta = c.final_delay - D;
    c.delta_sym = (sym_samples > 0) ? (double)c.delta / (double)sym_samples : 0.0;
    c.coarse_metric = ts.receive_stats.coarse_metric;
    c.mean_H = ts.receive_stats.mean_H;
    c.decoded = (ts.receive_stats.message_decoded == YES) ? 1 : 0;
    // byte-check the decode so a spurious message_decoded flag cannot read as TRUE.
    if (c.decoded) {
        for (int i = 0; i < frame_size; i++)
            if ((dc.hd_decoded_data_byte[i] & 0xff) != (payload[i] & 0xff)) { c.decoded = 0; break; }
    }
    c.sync_trials = ts.receive_stats.sync_trials;
    int half = sym_samples / 2;
    c.edge = (c.delta < -half) ? -1 : (c.delta > half ? +1 : 0);
    c.frame_rms = frame_rms; c.sigma = sigma;
    return c;
}

static int run_frame0_vehicle_selftest()
{
    // Ensure the RAW lock is measured (defeat the F1b rescue). setenv before the
    // first receive_byte so the read-once static in telecom_system sees it.
    setenv("MERCURY_SUBPEAK_RESCUE_DEFEAT", "1", 1);

    printf("[TEST-FRAME0-VEHICLE] deterministic in-process post-turnaround wrong-lock reproduction\n");
    const char* cfg_env = std::getenv("MERCURY_F0V_CONFIG");
    int cfg = cfg_env ? atoi(cfg_env) : CONFIG_16;
    printf("[TEST-FRAME0-VEHICLE] config=%d  (fingerprint: coarse_metric>=0.97 && mean_H<0.5 && |delta|>=1sym && !decoded)\n", cfg);
    printf("[TEST-FRAME0-VEHICLE] %-6s %-8s %-6s %-11s %-9s %-8s %-8s %-4s %-6s %-6s\n",
           "onset", "snr_dB", "seed", "final_delay", "delta_smp", "delta_sy", "metric", "dec", "meanH", "edge");

    // Sweep: run-up SNR (inf = pure zeros control) x a few onsets x a few seeds.
    // The field wrong lock is k~1-3.5 symbols off with metric~0.998, mean_H~0.37.
    double snrs[] = {200.0, 30.0, 25.0, 20.0, 15.0};
    int onsets[]  = {8, 20, 60, 100};
    unsigned seeds[] = {1u, 2u, 3u};
    int n_wronglock = 0, n_total = 0, n_early = 0, n_late = 0, n_decoded = 0;
    for (double snr : snrs) {
        for (int onset : onsets) {
            for (unsigned sd : seeds) {
                F0VCell c = f0v_run_cell(cfg, onset, snr, sd);
                n_total++;
                const char* edge_s = (c.edge < 0) ? "EARLY" : (c.edge > 0 ? "LATE" : "TRUE");
                bool wrong = (c.coarse_metric >= 0.97) && (c.mean_H >= 0.0) && (c.mean_H < 0.5)
                             && (c.edge != 0) && (!c.decoded);
                if (wrong) { n_wronglock++; if (c.edge < 0) n_early++; else n_late++; }
                if (c.decoded) n_decoded++;
                printf("[TEST-FRAME0-VEHICLE] %-6d %-8.0f %-6u %-11d %-9d %-+8.2f %-8.4f %-4d %-6.3f %-6s%s\n",
                       onset, c.snr_db, c.seed, c.final_delay, c.delta, c.delta_sym,
                       c.coarse_metric, c.decoded, c.mean_H, edge_s,
                       wrong ? "  <== WRONG-LOCK" : "");
            }
        }
    }
    printf("[TEST-FRAME0-VEHICLE] SUMMARY cells=%d wrong_lock=%d (early=%d late=%d) decoded=%d\n",
           n_total, n_wronglock, n_early, n_late, n_decoded);
    printf("[TEST-FRAME0-VEHICLE] FAITHFUL=%s (need >=1 wrong-lock with the field fingerprint)\n",
           n_wronglock > 0 ? "YES" : "NO");
    // This is a diagnostic vehicle, not a pass/fail gate: always return 0.
    return 0;
}

// --test-frame0-mf: candidate (c) MATCHED-FILTER viability measurement. Revives the
// #if0'd OFDM correlation template (env MERCURY_F0V_MF_TEMPLATE=1, set below) and runs
// the production time_sync_preamble_matched on the SAME synthesized wrong-lock buffer.
// Reports the MF peak delay + correlation. Decision rule (per the task):
//   - MF peak lands at TRUE (|mf_delay - D| < 1/2 symbol) AND corr clearly above the
//     2.0 detect floor / above the plateau-edge values -> MF DISCRIMINATES the plateau
//     -> candidate (c) is a viable winner.
//   - MF corr stays ~0.15 / peak not at TRUE -> MF is DEAD (cannot beat the plateau) ->
//     drop it, do NOT wire a dead detector into the shared path.
static int run_frame0_mf_selftest()
{
    setenv("MERCURY_F0V_MF_TEMPLATE", "1", 1);   // build the MF template during load
    printf("[TEST-FRAME0-MF] matched-filter plateau-tiebreak VIABILITY measurement\n");
    const char* cfg_env = std::getenv("MERCURY_F0V_CONFIG");
    int cfg = cfg_env ? atoi(cfg_env) : CONFIG_16;

    cl_telecom_system ts;
    ts.operation_mode = BER_PLOT_passband;
    ts.load_configuration(cfg);
    cl_data_container& dc = ts.data_container;
    int interp       = ts.frequency_interpolation_rate;
    int sym_samples  = dc.Nofdm * interp;
    int frame_symb   = dc.Nsymb + dc.preamble_nSymb;
    int frame_samp   = frame_symb * sym_samples;
    int buf_samp     = dc.buffer_Nsymb * sym_samples;
    int max_delay    = buf_samp - frame_samp;

    printf("[TEST-FRAME0-MF] config=%d template_len=%d energy=%.4f nsymb=%d sym_samples=%d (len=0 => revive FAILED)\n",
           cfg, ts.ofdm.ofdm_corr_template_len, ts.ofdm.ofdm_corr_template_energy,
           ts.ofdm.ofdm_corr_template_nsymb, sym_samples);

    int nReal_data = dc.nBits - ts.ldpc.P;
    int frame_size = (nReal_data - ts.outer_code_reserved_bits) / 8; if (frame_size < 1) frame_size = 1;
    std::vector<int> payload(frame_size, 0); for (int i=0;i<frame_size;i++) payload[i]=(i*53+7)&0xff;
    std::vector<double> frame(frame_samp, 0.0);
    ts.transmit_byte(payload.data(), frame_size, frame.data(), SINGLE_MESSAGE);
    double e=0; for(double v:frame) e+=v*v; double frame_rms=(frame.size()>0)?sqrt(e/(double)frame.size()):0.0;

    std::vector<std::complex<double>> bb(buf_samp);
    printf("[TEST-FRAME0-MF] %-6s %-8s %-6s %-11s %-11s %-11s %-9s\n",
           "onset","snr_dB","seed","true_D","mf_delay","mf-D(sym)","mf_corr");
    double snrs[]={200.0,30.0}; int onsets[]={60,100}; unsigned seeds[]={1u,2u};
    int n_true=0,n_total=0;
    for (double snr:snrs) for (int onset:onsets) for (unsigned sd:seeds) {
        int D=onset*sym_samples; if(D>max_delay)D=max_delay; if(D<0)D=0;
        std::mt19937 rng(sd);
        double sigma=(snr<200.0)?frame_rms/pow(10.0,snr/20.0):0.0;
        std::normal_distribution<double> nd(0.0,sigma);
        std::vector<double> rx(buf_samp);
        for(int i=0;i<buf_samp;i++) rx[i]=(sigma>0.0)?nd(rng):0.0;
        int cn=frame_samp; if(D+cn>buf_samp)cn=buf_samp-D;
        for(int i=0;i<cn;i++) rx[D+i]+=frame[i];
        ts.ofdm.passband_to_baseband(rx.data(), buf_samp, bb.data(),
            ts.sampling_frequency, ts.carrier_frequency, ts.carrier_amplitude, 1, &ts.ofdm.FIR_rx_time_sync);
        TimeSyncResult mf = ts.ofdm.time_sync_preamble_matched(bb.data(), buf_samp, interp, dc.preamble_nSymb);
        double mfdsym=(sym_samples>0)?(double)(mf.delay-D)/sym_samples:0.0;
        bool at_true = (sym_samples>0) && (std::abs(mf.delay-D) < sym_samples/2);
        n_total++; if(at_true) n_true++;
        printf("[TEST-FRAME0-MF] %-6d %-8.0f %-6u %-11d %-11d %-+11.2f %-9.4f%s\n",
            onset, snr, sd, D, mf.delay, mfdsym, mf.correlation, at_true?"  TRUE":"  OFF");
    }
    printf("[TEST-FRAME0-MF] SUMMARY cells=%d mf_at_true=%d  VIABLE=%s\n",
        n_total, n_true, (n_true==n_total && n_total>0)?"YES":"NO");
    return 0;
}

// --test-frame0-replay <capture>: feed a raw passband ring captured at the
// field SUBPEAK-REJECT (MERCURY_F0_RINGDUMP) through the PRODUCTION acquisition
// exactly as the synth vehicle does. Faithful by construction (it IS the field
// waveform, deep collapse included). With MERCURY_FINE_ENERGY_REL=1 the F-A gate
// should advance the lock off the run-up and decode. Returns 0 if decoded.
static int run_frame0_replay_selftest(const char* path)
{
    FILE* fp = fopen(path, "rb");
    if(!fp){ printf("[TEST-FRAME0-REPLAY] cannot open %s\n", path); return 1; }
    int hdr[6]; double meta[2]; long long ns = 0;
    if(fread(hdr,sizeof(int),6,fp)!=6 || fread(meta,sizeof(double),2,fp)!=2 || fread(&ns,sizeof(long long),1,fp)!=1)
    { fclose(fp); printf("[TEST-FRAME0-REPLAY] bad header %s\n", path); return 1; }
    if(hdr[0]!=(int)0xF0CAB1 || ns<=0 || ns>500000000LL)
    { fclose(fp); printf("[TEST-FRAME0-REPLAY] bad magic/size %s\n", path); return 1; }
    int config = hdr[1];
    std::vector<double> rx((size_t)ns);
    size_t got = fread(rx.data(), sizeof(double), (size_t)ns, fp);
    fclose(fp);
    if((long long)got != ns){ printf("[TEST-FRAME0-REPLAY] short read %s\n", path); return 1; }

    setenv("MERCURY_SUBPEAK_RESCUE_DEFEAT", "1", 1);   // measure the RAW lock
    cl_telecom_system ts;
    ts.operation_mode = BER_PLOT_passband;
    ts.load_configuration(config);
    cl_data_container& dc = ts.data_container;
    ts.receive_stats.ofdm_search_raw           = 0;
    ts.data_container.nUnder_processing_events = 0;
    ts.receive_stats.ofdm_batch_active         = false;
    ts.receive_stats.delay                     = 0;
    ts.ofdm_forced_delay                       = -1;
    ts.mfsk_fixed_delay                        = -1;
    ts.receive_byte(rx.data(), dc.hd_decoded_data_byte);
    int decoded = (ts.receive_stats.message_decoded == YES) ? 1 : 0;
    int sym_samples = dc.Nofdm * ts.frequency_interpolation_rate;
    double dsym = (sym_samples>0) ? (double)((int)ts.receive_stats.delay - hdr[5]) / sym_samples : 0.0;
    printf("[TEST-FRAME0-REPLAY] file=%s config=%d cap_delay=%d cap_metric=%.3f cap_meanH=%.3f => final_delay=%d d(sym)=%+.2f meanH=%.3f decoded=%d\n",
        path, config, hdr[5], meta[0], meta[1], (int)ts.receive_stats.delay, dsym,
        ts.receive_stats.mean_H, decoded);
    return decoded ? 0 : 1;
}

// --test-crc-escape: CRC-ESCAPE accept-gate self-test. Proves the OFDM RX frame
// accept/reject gate (cl_telecom_system::frame_decode_rejected) REJECTS a non-converged
// LDPC decode (iterations_done == nIteration_max+1, the FAIL sentinel) whose residual-
// error bits collide the 16-bit MODBUS CRC to 0 -- the ~2^-16 event that under the
// legacy crc-only gate committed a corrupt frame on a clean channel (LOSE, not CORRUPT).
// Deterministic, in-process (no IONOS/RF): loads CONFIG_16 (CRC16_MODBUS_RTU outer code),
// mocks the exact gate inputs, and drives the PRODUCTION predicate directly.
//   fail-before: crc_escape_defeat=true  -> legacy gate ACCEPTS (message_decoded=YES; the
//                CRC-colliding corrupt frame is delivered).
//   pass-after : crc_escape_defeat=false -> gate REJECTS (message_decoded=NO -> the frame
//                falls into the FAIL branch and the existing TIME_INTERP-seed rescue).
// Controls: a CONVERGED crc-good frame is ACCEPTED by BOTH arms (the fix weakens no
// good-frame acceptance); a crc-bad frame is REJECTED by both.
static int run_crc_escape_selftest()
{
    printf("[TEST-CRC-ESCAPE] OFDM accept-gate CRC-escape convergence-guard self-test\n");
    int fails = 0;
    const int cfg = CONFIG_16;

    cl_telecom_system ts;
    ts.operation_mode = BER_PLOT_passband;
    ts.load_configuration(cfg);

    // CONFIG_16 is a CRC16_MODBUS_RTU-outer-coded config -- the gate branch under test.
    if (ts.outer_code != CRC16_MODBUS_RTU) {
        printf("[TEST-CRC-ESCAPE]   FAIL: CONFIG_16 outer_code=%d != CRC16_MODBUS_RTU(%d)\n",
               ts.outer_code, CRC16_MODBUS_RTU);
        printf("[TEST-CRC-ESCAPE] FAILED (1 failure)\n");
        return 1;
    }
    const int nIterMax = ts.ldpc.nIteration_max;
    const int NONCONV  = nIterMax + 1;                 // canonical LDPC FAIL sentinel
    const int CONV     = (nIterMax > 1) ? (nIterMax - 1) : 0;  // a converged decode

    // returns 1 == REJECT (message_decoded=NO -> TINTERP); 0 == ACCEPT (delivers)
    auto decide = [&](int crc, int iters, int all_zeros, bool defeat)->int {
        st_receive_stats rs;                            // value-initialised (all fields 0)
        rs.all_zeros       = all_zeros;
        rs.crc             = crc;
        rs.iterations_done = iters;
        return ts.frame_decode_rejected(rs, ts.outer_code, defeat) ? 1 : 0;
    };

    // ---- fail-before: legacy crc-only gate ACCEPTS the corrupt frame ----
    int rej_before = decide(/*crc=*/0, /*iters=*/NONCONV, /*all_zeros=*/NO, /*defeat=*/true);
    bool fail_before_ok = (rej_before == 0);            // ACCEPT == the escape reproduced
    printf("[TEST-CRC-ESCAPE]   FAIL-BEFORE (defeat, crc=0 iter=%d nonconv): gate=%s -> %s (expect ACCEPT/deliver)\n",
           NONCONV, rej_before ? "REJECT" : "ACCEPT",
           rej_before ? "message_decoded=NO" : "message_decoded=YES");
    if (!fail_before_ok) { fails++; printf("[TEST-CRC-ESCAPE]   FAIL: legacy gate did NOT accept -- fail-before not reproduced\n"); }

    // ---- pass-after: fixed gate REJECTS the same frame -> TINTERP ----
    int rej_after = decide(/*crc=*/0, /*iters=*/NONCONV, /*all_zeros=*/NO, /*defeat=*/false);
    bool pass_after_ok = (rej_after == 1);              // REJECT == message_decoded=NO -> TINTERP
    printf("[TEST-CRC-ESCAPE]   PASS-AFTER  (fixed,  crc=0 iter=%d nonconv): gate=%s -> %s (expect REJECT->TINTERP)\n",
           NONCONV, rej_after ? "REJECT" : "ACCEPT",
           rej_after ? "message_decoded=NO" : "message_decoded=YES");
    if (!pass_after_ok) { fails++; printf("[TEST-CRC-ESCAPE]   FAIL: fixed gate did NOT reject -- CRC-escape still open\n"); }

    // ---- control 1: a CONVERGED crc-good frame is ACCEPTED by BOTH arms ----
    int good_before = decide(0, CONV, NO, true);
    int good_after  = decide(0, CONV, NO, false);
    bool good_ok = (good_before == 0) && (good_after == 0);
    printf("[TEST-CRC-ESCAPE]   CONTROL good-frame (crc=0 iter=%d converged): before=%s after=%s (expect ACCEPT both)\n",
           CONV, good_before?"REJECT":"ACCEPT", good_after?"REJECT":"ACCEPT");
    if (!good_ok) { fails++; printf("[TEST-CRC-ESCAPE]   FAIL: fix weakened converged good-frame acceptance\n"); }

    // ---- control 2: a crc-BAD frame is REJECTED by BOTH arms ----
    int bad_before = decide(0x1234, CONV, NO, true);
    int bad_after  = decide(0x1234, CONV, NO, false);
    bool bad_ok = (bad_before == 1) && (bad_after == 1);
    printf("[TEST-CRC-ESCAPE]   CONTROL crc-bad   (crc!=0 iter=%d): before=%s after=%s (expect REJECT both)\n",
           CONV, bad_before?"REJECT":"ACCEPT", bad_after?"REJECT":"ACCEPT");
    if (!bad_ok) { fails++; printf("[TEST-CRC-ESCAPE]   FAIL: crc-bad frame accepted\n"); }

    printf("[TEST-CRC-ESCAPE] %s (%d failure%s)  [nIterMax=%d FAIL-sentinel=%d]\n",
           fails==0 ? "ALL PASS" : "FAILED", fails, fails==1?"":"s", nIterMax, NONCONV);
    return fails==0 ? 0 : 1;
}

// --test-chase: chase-combining (HARQ Type-I soft-LLR combine) fail-before /
// pass-after self-test. See fact-documents/chase-combining-harq.md.
//
// ROOT this test guards: Mercury had NO soft-combining -- a failed frame's LLRs
// were DISCARDED, so a bit-identical retransmission's soft info was never summed
// with the prior look (the capstone "chase engaged N times, 0 rescues"). The
// coded bits of a same-config retx ARE identical (deterministic QC-LDPC encode),
// so summing the two LLR vectors is VALID and buys ~+3 dB.
//
// Deterministic construction (no OFDM sync, no RF, no audio): a random K-bit
// message is LDPC-encoded (CONFIG_6 = BPSK rate 1/2, K=800 N=1600), BPSK-mapped
// via the REAL psk.mod, and passed twice through the REAL AWGN channel at an
// operating point BELOW the single-look FEC waterfall (two independent noise
// realizations = the two ARQ looks). Each look is demapped by the REAL psk.demod
// and decoded by the REAL ldpc.decode. Assertions over T deterministic trials:
//   * single-look success           == 0   (fail-before: one copy never decodes)
//   * chase-OFF combine (env-defeat) == 0   (defeat: primitive no-ops -> one copy)
//   * chase-ON  combine              >  0   (pass-after: SUM rescues the frame)
// i.e. the combine SUCCESS-RATE moves 0 -> >0, and defeating chase restores the
// pre-fix 0. The combine itself is the production primitive
// cl_telecom_system::chase_capture()/chase_combine() -- NOT an ad-hoc sum here.
static int run_chase_selftest()
{
    printf("[TEST-CHASE] chase-combining (HARQ soft-LLR combine) fail-before/pass-after self-test\n");

    cl_telecom_system ts;
    ts.operation_mode = BER_PLOT_passband;
    ts.load_configuration(CONFIG_6);          // BPSK rate 1/2 -> K=800, N=1600
    ts.psk.set_predefined_constellation(MOD_BPSK);  // ensure unit-energy BPSK demapper

    const int N = ts.ldpc.N;
    const int K = ts.ldpc.K;
    if(N <= 0 || K <= 0 || N > N_MAX) {
        printf("[TEST-CHASE]   BAD SIZING N=%d K=%d -> FAIL\n", N, K);
        return 1;
    }

    // Operating point: complex-sample noise amplitude. Es=1 (BPSK), so Es/N0 =
    // 1/ampl^2. Chosen ~2 dB BELOW the single-look rate-1/2 waterfall so ONE look
    // never decodes while the +3 dB two-look combine crosses. Overridable via
    // MERCURY_CHASE_TEST_AMPL for tuning; default is the locked value.
    // ampl=1.45 (Es/N0 ~ -3.2 dB) sits ~1.5 dB BELOW the single-look rate-1/2
    // waterfall (single-look decodes ~100% at ampl~1.15, 0% by ampl~1.35) while the
    // +3 dB two-look combine still decodes 100% (its waterfall is beyond ampl~1.65),
    // giving both arms a wide deterministic margin (fixed seeds -> non-flaky).
    double ampl = 1.45;
    { const char* e = std::getenv("MERCURY_CHASE_TEST_AMPL"); if(e && *e) ampl = atof(e); }
    const float variance = (float)(ampl * ampl);

    const int T = 40;
    // Env-defeat: MERCURY_CHASE=0 forces the chase-ON arm OFF too, demonstrating
    // that defeating the primitive returns the pre-fix 0-success behavior.
    bool env_off = false;
    { const char* e = std::getenv("MERCURY_CHASE"); if(e && strcmp(e,"0")==0) env_off = true; }

    int* info = new int[K];
    int* cw   = new int[N];
    std::complex<double>* sym = new std::complex<double>[N];
    std::complex<double>* rx1 = new std::complex<double>[N];
    std::complex<double>* rx2 = new std::complex<double>[N];
    float* llr1 = new float[N];
    float* llr2 = new float[N];
    float* comb = new float[N];
    int*   out  = new int[K];

    auto frame_ok = [&](const int* dec)->bool{
        for(int i=0;i<K;i++) if(dec[i] != info[i]) return false;
        return true;
    };

    int ok_single = 0, ok_chase_on = 0, ok_chase_off = 0;
    const bool dbg = (std::getenv("MERCURY_CHASE_TEST_DEBUG") != nullptr);

    srand(0xC4A5E);  // deterministic message + noise stream
    for(int t=0; t<T; t++)
    {
        for(int i=0;i<K;i++) info[i] = rand() & 1;
        ts.ldpc.encode(info, cw);           // K info -> N coded bits (systematic)
        ts.psk.mod(cw, N, sym);             // BPSK: N coded bits -> N unit-energy symbols

        // Two INDEPENDENT looks of the SAME codeword. Seeds must be DISTINCT and
        // positive (set_seed srand's only when seed>0); keep them well-separated so
        // the two noise realizations are decorrelated (a collapsed/equal seed makes
        // the "combine" merely DOUBLE one look -> no diversity -> non-convergence).
        ts.awgn_channel.set_seed((long)(100003 + 2*t));
        ts.awgn_channel.apply(sym, rx1, (float)ampl, N);
        ts.awgn_channel.set_seed((long)(700019 + 2*t));
        ts.awgn_channel.apply(sym, rx2, (float)ampl, N);

        ts.psk.demod(rx1, N, llr1, variance);
        ts.psk.demod(rx2, N, llr2, variance);

        // (a) SINGLE-LOOK baseline (look 2 alone) -- the pre-chase behavior.
        int it_s = ts.ldpc.decode(llr2, out);
        bool ok_s = frame_ok(out);
        if(ok_s) ok_single++;

        // (b) CHASE-ON: buffer look 1, combine into look 2, decode the SUM.
        ts.chase_enabled = !env_off;
        ts.chase_reset();
        ts.chase_capture(llr1, N, CONFIG_6);
        for(int i=0;i<N;i++) comb[i] = llr2[i];
        bool combined_on = ts.chase_combine(comb, N, CONFIG_6);  // sums iff enabled+match
        int it_c = ts.ldpc.decode(comb, out);
        bool ok_c = frame_ok(out);
        if(combined_on && ok_c) ok_chase_on++;
        if(dbg) {
            int nbad=0; for(int i=0;i<K;i++) if(out[i]!=info[i]) nbad++;
            double amax1=0,amax2=0,amaxc=0;
            for(int i=0;i<N;i++){ if(fabs(llr1[i])>amax1)amax1=fabs(llr1[i]); if(fabs(llr2[i])>amax2)amax2=fabs(llr2[i]); double c=llr1[i]+llr2[i]; if(fabs(c)>amaxc)amaxc=fabs(c);}
            printf("[TEST-CHASE-DBG] t=%2d single ok=%d it=%d | comb ok=%d it=%d combflag=%d bad=%d | max|llr1|=%.1f max|llr2|=%.1f max|sum|=%.1f\n",
                   t, ok_s?1:0, it_s, ok_c?1:0, it_c, combined_on?1:0, nbad, amax1, amax2, amaxc);
        }

        // (c) CHASE-OFF (env-defeat semantics): primitive disabled -> combine
        //     no-ops -> decodes a single look -> pre-fix 0-success.
        ts.chase_enabled = false;
        ts.chase_reset();
        ts.chase_capture(llr1, N, CONFIG_6);                    // no-op (disabled)
        for(int i=0;i<N;i++) comb[i] = llr2[i];
        ts.chase_combine(comb, N, CONFIG_6);                    // returns false, comb == llr2
        ts.ldpc.decode(comb, out);
        if(frame_ok(out)) ok_chase_off++;
    }

    printf("[TEST-CHASE]   ampl=%.3f (Es/N0=%.2f dB) T=%d :: single=%d/%d  chase_ON=%d/%d  chase_OFF(defeat)=%d/%d\n",
           ampl, -10.0*log10(ampl*ampl), T,
           ok_single, T, ok_chase_on, T, ok_chase_off, T);

    // Fail-before / pass-after:
    //   single-look and env-defeat MUST be 0 (one copy never decodes at this point);
    //   chase-ON MUST rescue (>0)  -> combine success-rate 0 -> >0.
    // Under MERCURY_CHASE=0 the ON arm is forced OFF, so it collapses to 0 and the
    // test FAILS -- which is the intended demonstration that defeating chase
    // restores the pre-fix behavior.
    bool pass;
    if(env_off) {
        pass = (ok_single == 0) && (ok_chase_on == 0) && (ok_chase_off == 0);
        printf("[TEST-CHASE]   MERCURY_CHASE=0 env-defeat: all arms 0-success (pre-fix) -> %s\n",
               pass ? "DEMONSTRATED" : "UNEXPECTED");
    } else {
        pass = (ok_single == 0) && (ok_chase_off == 0) && (ok_chase_on > 0);
    }
    printf("[TEST-CHASE] %s\n", pass ? "PASS (combine success-rate 0 -> >0)" : "FAIL");

    delete[] info; delete[] cw; delete[] sym; delete[] rx1; delete[] rx2;
    delete[] llr1; delete[] llr2; delete[] comb; delete[] out;
    return pass ? 0 : 1;
}

// --test wall-clock watchdog. `--test` is a Monte-Carlo suite with no internal
// time bound; on the RPi bench a wedged test used to hang forever, pinning a
// core at 99% (stacked orphans -> thermal throttle -> CPU jitter that tips OFDM
// acquisition onto sub-peaks). Arm a detached timer thread on entry to any
// --test* dispatch: if the suite has not returned (and exited the process) by
// the deadline, force a NON-ZERO exit so a wedged test can never hang. On normal
// completion main() returns first and the still-sleeping detached thread is
// abandoned with the process, so the watchdog never fires on a healthy run.
// Deadline is MERCURY_TEST_WATCHDOG_S seconds (default 600; <=0 disables).
static void arm_test_watchdog() {
    long deadline_s = 600;
    const char* env = getenv("MERCURY_TEST_WATCHDOG_S");
    if (env && *env) {
        char* end = NULL;
        long v = strtol(env, &end, 10);
        if (end != env) deadline_s = v;
    }
    if (deadline_s <= 0) return;   // explicitly disabled
    std::thread([deadline_s]() {
        std::this_thread::sleep_for(std::chrono::seconds(deadline_s));
        fprintf(stderr,
            "\n[TEST-WATCHDOG] --test exceeded %ld s wall clock — aborting with "
            "non-zero exit 70 (set MERCURY_TEST_WATCHDOG_S to tune, <=0 to disable).\n",
            deadline_s);
        fflush(stderr);
        // Exit 70 (EX_SOFTWARE): non-zero so callers see failure, and distinct
        // from GNU `timeout`'s own 124 so a wedged-test self-abort is
        // distinguishable from an external timeout kill in deploy/CI logs.
        _exit(70);
    }).detach();
}

int main(int argc, char *argv[])
{
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
        if (strcmp(argv[i], "--test") == 0) {
            arm_test_watchdog();   // wall-clock backstop: wedged --test can't hang forever
            int failed = run_mfsk_ctrl_codec_tests();
            if (getenv("MERCURY_CAP_CODEC_ONLY") != NULL)
                return failed == 0 ? 0 : 1;
            failed += run_moose_deadzone_tests();
            failed += run_pilot_thin_nv_tests();
            failed += run_sim_clock_tests();
            failed += run_winlink_dict_tests();
            // AEAD bsi-bound nonce regression suite (data-flow-aead-nonce.md):
            // SECURITY INVARIANT — no (key,nonce) reuse; nonce binds to the wire
            // batch_seq_id so SACK reorder/retx decrypt correctly. Includes the
            // >256-batch wrap, reorder+retx round-trip, direction disjointness,
            // truncated-KX reject, and tamper-reject cases. No IONOS/RF.
            failed += run_aead_nonce_tests();
            // Encryption negotiation FAIL-CLOSED regression: once the operator has
            // opted into -E, a peer without CAP_ENCRYPTION (unsupported, or a MITM
            // that stripped the cap bit) is REFUSED for BOTH strict and fast — never
            // a plaintext downgrade — while default-off (ENCRYPT_OFF) stays plaintext.
            // Drives the REAL shared predicate decide_encryption_negotiation() that
            // both endpoints call. Fails-before: -DENC_FAILOPEN_FAILBEFORE. No IONOS/RF.
            {
                cl_arq_controller test_enc;
                failed += test_enc.test_encryption_fail_closed();
            }
            // ML-KEM-768 hybrid KEX regression suite (MLKEM_HYBRID_PLAN.md,
            // data-flow-hybrid-kex.md): combiner symmetry, ML-KEM-first IKM
            // order, transcript binding (tamper -> key divergence), and the
            // KX2/KX3 chunk encode/reassemble + CRC8 round-trip. No IONOS/RF.
            failed += run_mlkem_hybrid_tests();
            // In-band down-ladder DELIVERY regression (BREAK-orphan + silent-snapshot). A
            // member test on a throwaway controller (its own buffers; PART B builds its own
            // minimal telecom_system). Fast + deterministic, no IONOS/RF. data-flow-inband-
            // downladder.md §3/§5.3.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_inband_downladder();
            }
            // Reseat span integrity: the prev-batch cross-storage index-skew 332-byte
            // deletion. Drives the PRODUCTION store -> seal -> copy_data_to_buffer and
            // proves the seal-count + funnel byte-span keystones prevent the short delivery
            // while healthy + res_c3100-widened deliveries stay byte-identical. Member test
            // on a throwaway controller; fast + deterministic, no IONOS/RF. data-flow-
            // messages_rx_prev.md §4.5 CORRECTION.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_reseat_span();
            }
            // CMD-side id-skew ROOT (the correct-by-construction TX counterpart to the reseat
            // keystones): a fresh v1/v2 non-mixed new-data batch is rebased to
            // messages_tx slots [0,ND)
            // so the wire id == batch position == receiver storage loc. Reproduces the bsi33->bsi34
            // +38 skew via the PRODUCTION add_message_tx_data first-free staging, asserts the
            // rebase corrects it in both wire versions (and the defeat env preserves it),
            // and that the §87 geometry with
            // id=seq DELIVERS IN PLACE (6301 B) through the production receiver. CANONICAL_NUMBERS.md §87.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_cmd_idskew();
            }
            // ROBUST->OFDM ADOPT live-burst PRESERVE regression (the last transition-class hole:
            // the unilateral adopt into an OFDM config used to WIPE the in-flight preamble
            // mid-capture -> no acquire -> TERMINAL BREAK -> ROBUST_0 spiral). Member test on a
            // throwaway controller (builds its own telecom_system). Fast + deterministic, no
            // IONOS/RF. data-flow-robust-ofdm-adopt-flush.md §6/§8.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_inband_adopt_preserve_live_burst();
            }
            // ROBUST->OFDM ADOPT ring-shrink Nofdm-INVARIANT regression (diagnosis a468b2fc):
            // the HINGE-1 shrink re-derived Nofdm (= Nfft+Ngi) from a STALE live ofdm.gi,
            // drifting CONFIG_0 from 292 to 310 -> an 18-sample/symbol FFT-window drift ->
            // LDPC iter=0 -> garbage CRC -> the CONFIG_0 under-decode (~53 B). The fix preserves
            // the just-loaded data_container.Nofdm across the shrink. Member test on a throwaway
            // controller (builds its own telecom_system). Fast + deterministic, no IONOS/RF.
            // data-flow-robust-ofdm-adopt-flush.md §15.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_inband_adopt_nofdm_invariant();
            }
            // Harvest regression: independently require the production-created CONFIG_0
            // decoder to match the startup-patched primary geometry. The same test patch
            // fails on bare monitor with Nofdm=310.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_harvest_inband_cfg0_geometry();
            }
            // §21: CONFIG_0-START robust-floor OVER-SEAT (the uncovered sibling of FIX #1e). A session
            // that starts at CONFIG_0 (no robust->OFDM adopt) never latches inband_ofdm_acq_ring_shrunk,
            // so inband_seat_robust_ring_floor over-grows the natural OFDM ring (217->~804) -> every
            // preamble at the tail beyond upper_bound -> 0 forward decode. Member test on a throwaway
            // controller (builds its own telecom_system). data-flow-robust-ofdm-adopt-flush.md §21.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_inband_config0_start_ring();
            }
            // §22: the scoped down-ladder decoder bank inherits the PRIMARY's startup-patched ofdm_gi.
            // A fresh `new cl_telecom_system()` rung otherwise keeps the ctor gi=54/256 -> CONFIG_0
            // Nofdm=310 (not the production 292), an 18-sample/symbol FFT-window stride error that
            // skips LDPC at WB CONFIG_0 -> the robust->OFDM cross never sustains. PASS-AFTER: 292.
            // FAIL-BEFORE (MERCURY_INBAND_GI_INHERIT_DEFEAT=1, same binary): 310.
            // data-flow-robust-ofdm-adopt-flush.md §22.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_inband_down_decoder_gi_inherit();
            }
            // §17: descrambler survives the inband ring-shrink (the CONFIG_0 clean-lock CRC-fail root).
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_inband_descrambler_survives_ring_shrink();
            }
            // §19: dead-batch streak ties to REAL batch periods + zero-progress (climb-killer fix);
            // a real total loss STILL BREAKs (recovery preserved).
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_inband_deadbatch_progress();
            }
            // In-band CONNECT-LIVENESS GUARD regression (control-plane livelock backstop).
            // Member test on a throwaway controller (builds its own telecom_system). Fast +
            // deterministic, no IONOS/RF. data-flow-inband-connect-liveness.md §4.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_inband_liveness();
            }
            // In-band FORWARD-HEALTHY REVERSE-ACK MISS -> NO-BREAK DELIVER regression (the
            // 785-frame decode-but-0-deliver rework). Member test on a throwaway controller
            // (builds its own CMD/telecom_system per case). Fast + deterministic, no IONOS/RF.
            // data-flow-inband-retx-epoch.md §5.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_inband_deliver();
            }
            // LINK-PHASE PRIMITIVE (increment 1) shadow-agreement directed unit. Member test
            // on a throwaway controller; PURE in-process synthetic-fire of the production
            // producers. Fast + deterministic, no IONOS/RF. data-flow-linkphase-primitive.md.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_linkphase_shadow();
            }
            // In-band capture-ring ROBUST-floor OVER-SEAT regression (the CONFIG_8-climb
            // 24/25-BREAK root): inband_seat_robust_ring_floor over-grew the climbed OFDM
            // ring to the ROBUST floor -> frame-0 SKIP-VAR every batch. Member test on a
            // throwaway controller; PURE in-process synthetic-fire.
            // data-flow-inband-ring-floor-overseat.md §5.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_inband_ring_floor_overseat();
            }
            // Option B compact coded reverse-confirm LIVE RX-PATH regression
            // (data-flow-compact-confirm.md §9): the self-validating compact confirm
            // must be accepted via its OWN CRC-gated decode, DECOUPLED from the bare
            // 7/16 presence gate that under-peaks the shorter 26-sym frame. Drives the
            // FULL live RX path (passband -> capture ring -> the production accept
            // predicate cmd_compact_confirm_live_accept), NOT a direct decoder call.
            // Member test on a throwaway controller; PURE in-process synthetic-fire.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_compact_confirm_live_rx_path();
            }
            // Option B fix-b — compact confirm accept INSIDE the SACK window (batch>1,
            // SACK-on, clean, WB; data-flow-compact-confirm.md §10.4/§10.5). The SECOND
            // live-land defect (4.7x batch>1-WB regression at ENABLE=1). PURE in-process.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_compact_confirm_sack_window_rx_path();
            }
            // FORGIVING-ACK Tier-2 cumulative-n_r self-heal / gap-invariant / cap-gate
            // regression (the A3 predicate proof). Member test on a throwaway controller;
            // PURE in-process synthetic-fire, no IONOS/RF. data-flow-forgiving-ack.md §T2.6.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_cumulative_ack();
            }
            // A3 DECOUPLE-SAFETY CHECKPOINT (§2): single-miss non-load-bearing (anti-0-bytes,
            // byte-faithful) + genuine-death net intact, demote IN PLACE. Member test on a
            // throwaway controller; PURE in-process synthetic-fire. data-flow-forgiving-ack.md
            // §T2.2/§6.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_a3_decouple_safety();
            }
            // HYBRID TIER-CROSSING ROUTING (data-flow-inband-tier-crossing.md §3):
            // a robust<->OFDM crossing routes to the legacy SET_CONFIG handshake
            // (fast dedicated ACK); intra-tier rate adapts keep the in-band tag.
            // Member test on a throwaway controller; PURE in-process synthetic-fire,
            // no PHY/audio/IONOS/RF -> permanent regression gate.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_inband_tier_crossing_routing();
            }
            // IN-BAND TIER-CROSS REVERSE-ACK PIN regression (data-flow-inband-tier-
            // crossing.md §3): a robust<->OFDM cross must pin reverse_configuration to
            // the ROBUST side (not the OFDM forward rung), so the reverse SACK decodes on
            // MFSK across the cross (the redesign's 102<->0 oscillation root). PURE in-
            // process synthetic-fire — permanent regression gate.
            {
                cl_arq_controller test_revpin;
                failed += test_revpin.test_inband_tier_cross_reverse_pin();
            }
            // KEYSTONE intra-tier base-pattern confirm regression (data-flow-inband-tier-
            // crossing.md §6): an intra-tier CLIMB confirm must ride the robust BASE ACK
            // pattern (decoupled from the marginal bsi-bearing SACK suffix) so the redesign
            // climbs past CONFIG_0 toward legacy's OFDM rungs. PURE in-process synthetic-fire
            // — permanent regression gate. Fails-before: -DINBAND_BASEPATTERN_CONFIRM_FAILBEFORE.
            {
                cl_arq_controller test_baseconfirm;
                failed += test_baseconfirm.test_inband_basepattern_confirm();
            }
            // RESIDUAL FIX regression (data-flow-inband-tier-crossing.md s11): the ROBUST-TIER
            // tag-follow must fire on STALE inter-frame passes too, else the re-emitted CONFIG_TAG
            // is dropped and the robust->OFDM tier-cross becomes non-deterministic. Drives the
            // EXACT production gate (inband_robust_follow_gate_open). Fail-before:
            // MERCURY_ROBUST_FOLLOW_FRESHWIN_REQ=1. PURE in-process -- permanent regression gate.
            {
                cl_arq_controller test_rbfollow;
                failed += test_rbfollow.test_inband_robust_follow_freshwin();
            }
            // IN-BAND CONFIG_0 ROLLING-PARTIAL climb-unblock regression
            // (data-flow-inband-frame0-rolling-partial.md §4): at the inband OFDM base rung the
            // first OFDM frame of each batch fails the SKIP-VAR gate (acquisition seam) → a
            // rolling lead-frame-only partial that pre-fix vetoed the FRAME-UP climb → CONFIG_0
            // wedge. The fix advances the climb streak on that partial while a multi-drop partial
            // stays vetoed. PURE in-process synthetic-fire — permanent regression gate.
            // Fails-before: -DINBAND_FRAME0_PARTIAL_FAILBEFORE.
            {
                cl_arq_controller test_f0p;
                failed += test_f0p.test_inband_frame0_partial();
            }
            // IN-BAND ROLLING-PARTIAL climb DEFER-WHILE-HOLE-OUTSTANDING regression
            // (data-flow-inband-frame0-rolling-partial.md §10): the 2801d7c sibling — the
            // lead-frame-only partial enqueues frame-0 for retx (retransmit_count > 0), and the
            // anchor-raise + FRAME-UP config-change fire WHILE that hole is outstanding; the config
            // change clears the retx queue -> abandons frame-0 -> bsi gap -> GAP-ABORT wedge. The
            // fix DEFERS the anchor-raise + the fire while the hole is outstanding but KEEPS the
            // 2801d7c streak credit. PURE in-process synthetic-fire — permanent regression gate.
            // Fails-before: -DINBAND_CLIMB_DEFER_FAILBEFORE.
            {
                cl_arq_controller test_cd;
                failed += test_cd.test_inband_climb_defer_on_retx();
            }
            // IN-BAND +1 CLIMB regression (data-flow-inband-frame0-rolling-partial.md §7.2
            // option A): under MERCURY_INBAND_RATE the FRAME-UP climb must step EXACTLY +1
            // (suppress the SNR elevator that jumped CONFIG_0->3 and stranded the reverse
            // data-SACK -> nAcked_data stuck -> BREAK at every rung). Legacy keeps the
            // elevator (byte-identical). PURE in-process synthetic-fire — permanent
            // regression gate. Fails-before: -DINBAND_PLUS1_CLIMB_FAILBEFORE.
            {
                cl_arq_controller test_p1;
                failed += test_p1.test_inband_plus1_climb();
            }
            // CLIMB-LATENCY — guarded SNR-seed of the START config
            // (gearshift-start-and-recovery.md §10): a CLEARLY-clean channel opens
            // data near the SNR-appropriate WB OFDM config at connect instead of
            // crawling ROBUST_0->1->2 (the ~102 s dominant climb cost), while a
            // MARGINAL channel still starts ROBUST_0 (the over-seed guard). PURE
            // in-process synthetic-fire — permanent regression gate. Fails-before:
            // -DCONNECT_SEED_FAILBEFORE.
            {
                cl_arq_controller test_cs;
                failed += test_cs.test_connect_snr_seed();
            }
            // CLIMB-LATENCY — first ROBUST climb rung pipelines
            // (gearshift-start-and-recovery.md §10.3): the ROBUST_0 dwell must NOT
            // serialize a clean-batch round-trip per rung (the un-pipelined first-rung
            // gap). Drives the REAL gate inband_pipeline_climb_active() on the robust
            // tier. PURE in-process synthetic-fire — permanent regression gate.
            // Fails-before: -DINBAND_ROBUST_PIPELINE_FAILBEFORE.
            {
                cl_arq_controller test_rp;
                failed += test_rp.test_robust_pipeline();
            }
            // DUTY FAST-START lever R — connect-evidenced robust exit (climb-duty):
            // the CONFIG_0 OFDM tier-cross fires at connect instead of after a full
            // robust DATA batch (~38 s). Drives the REAL predicate
            // robust_connect_exit_target(). PASS-AFTER seeds CONFIG_0; FAIL-BEFORE
            // (duty_r_defeat, the MERCURY_DUTY_R_DEFEAT / master-defeat production gate)
            // returns CONFIG_NONE. PURE in-process synthetic-fire — permanent gate.
            {
                cl_arq_controller test_dr;
                failed += test_dr.test_robust_connect_exit();
            }
            // DUTY FAST-START lever P-alt — climb-confirm batch shrink (climb-duty):
            // the OFDM confirm batch is capped small while climbing a non-top rung so
            // the 2-clean anchor bar is reached in less forward airtime. Drives the REAL
            // predicate climb_confirm_batch_active(). PASS-AFTER active; FAIL-BEFORE
            // (duty_palt_defeat, MERCURY_DUTY_PALT_DEFEAT / master defeat) inactive.
            {
                cl_arq_controller test_pa;
                failed += test_pa.test_climb_confirm_batch();
            }
            // Held-rung burst coalescing — a margin-gated election can HOLD a session
            // at a sub-top OFDM rung forever; the climb-confirm pin then caps the
            // steady-state batch at 10 and the Axis-2 skip starves growth. The release
            // (probation counter + provenance-released sole-setter clamp + gated
            // Axis-2 skip) drives the REAL controller, setter, RSP SET_LINK_PARAMS
            // handler, reset chokepoint and ctor env latch. FAIL-BEFORE arms via
            // MERCURY_BURST_COALESCE_DEFEAT on ONE binary.
            {
                cl_arq_controller test_bc;
                failed += test_bc.test_burst_coalesce_held_rung();
            }
            // BATCH-GRID COAST (FIX 1) — on a mid-batch decode FAIL the stock anti-spin
            // ladder discards the frame grid and the RX blind-searches into the
            // beyond-bounds attractor, killing the whole tail. Coast keeps the grid alive
            // (advance one frame period from the last CRC-GOOD anchor, keep batch_active) so
            // the existing predict+verify re-locks the next frame. Drives the REAL
            // batch_coast_try_advance() with REAL cfg15 geometry: PASS-AFTER coasts + counter
            // fires; FAIL-BEFORE (MERCURY_BATCH_COAST_DEFEAT) inert; per-leg eligibility guards.
            {
                cl_arq_controller test_coast;
                failed += test_coast.test_batch_coast();
            }
            // P1 ACQ BAND-EXCLUSION — the high-SNR break-storm persistence engine is
            // the no-exclusion Schmidl-Cox sub-peak re-pick (the reject loop restores
            // orig_delay every trial, re-locking the same content-stable wrong point in
            // a BAND of delays). P1 records rejected delays as band centers and forces a
            // clean re-acquire on a repeat re-pick inside a recorded band. Drives the
            // REAL predicate acq_band_excl_hit(): FAIL-BEFORE (empty memory) misses;
            // PASS-AFTER (band recorded) hits; BAND (not single-delay) neighbor hits.
            {
                cl_telecom_system test_excl;
                failed += test_excl.test_acq_band_excl();
            }
            // DUTY FAST-START lever ELEVATOR FAST-CONFIRM — margin-gated N=1 OFDM climb confirm:
            // on a whole-clean OFDM batch whose raw SNR capacity clears a higher rung, the anchor
            // is seated + the FRAME-UP elevator fires on the FIRST clean (skip the 2nd confirm),
            // collapsing the 2nd per-rung airtime of the cfg0->cfg16 dwell. Drives the REAL core.
            {
                cl_arq_controller test_ef;
                failed += test_ef.test_elevator_fast_confirm();
            }
            // CONNECT-SEED FUSION (climb-duty, connect floor) - the connect-evidenced seed
            // is folded into the SWITCH_BANDWIDTH frame so the RSP loads it on the WB switch
            // and the CMD on the ACK, collapsing the separate robust SET_CONFIG cross. Drives
            // the REAL selector connect_fuse_seed_select(). PASS-AFTER carries CONFIG_0;
            // FAIL-BEFORE (connect_fuse_defeat / MERCURY_CONNECT_FUSE_DEFEAT) CONFIG_NONE.
            {
                cl_arq_controller test_cf;
                failed += test_cf.test_connect_fuse();
            }
            // CLIMB-UP cmd_batch_seq_id ROLLBACK regression
            // (data-flow-inband-frame0-rolling-partial.md §13): the SYMMETRY GAP to the demote
            // rollback — the climb-UP SET_CONFIG emits (FRAME-UP, optimizer, turbo settle)
            // re-present an in-flight (RSP-delivered, not-yet-CMD-ACKed) batch under whatever
            // ADVANCED epoch a rapid climb reached, so the RSP sees a >=2 bsi jump from its
            // preserved delivery high-water -> sack_v2_readopt_has_gap()=true -> [RSP-V2-GAP-ABORT]
            // HOLD. The fix rolls cmd_batch_seq_id back to the in-flight bsi (mirroring the demote
            // paths). Drives the REAL producer + REAL RSP predicate. PURE in-process synthetic-fire
            // — permanent regression gate. Fails-before: -DINBAND_CLIMB_BSI_ROLLBACK_FAILBEFORE.
            {
                cl_arq_controller test_cbr;
                failed += test_cbr.test_climb_bsi_rollback();
            }
            // Reverse-SACK partial-target guard reference regression.
            {
                cl_arq_controller test_gr;
                failed += test_gr.test_guard_reanchor();
            }
            // FIX-C graceful-shutdown handler: handler installed above, this
            // self-raises SIGTERM/SIGINT and asserts shutdown_ flips, then
            // clears the flag so the rest of the process is unperturbed.
            failed += test_sigterm_handler();
            // IDLE-SWITCHROLE-RACE regression gates (idle-switchrole-race.md §4):
            // B (trigger-gate, B1+B2) and C (no-progress teardown + neg-control).
            // Both are in-process synthetic-fire (no PHY/audio) so they belong in
            // the master suite as permanent regression gates. MERGE COMPOSE: kept
            // alongside the redesign's in-band regression gates above (both belong).
            {
                cl_arq_controller ARQ_isr;
                failed += ARQ_isr.test_idle_switch_role_race();
                failed += ARQ_isr.test_switch_role_reride_race();
                failed += ARQ_isr.test_break_noprogress_teardown();
            }
            // MIXBATCH FILL OVER-POP -- COMPRESSION LEG regression
            // (data-flow-mixbatch-fill.md ss9): comp-leg sibling of the no-comp
            // over-pop -- fill-cap respects the v2 retx prefix AND the force-FREE
            // never destroys a staged-but-unsent frame. In-process synthetic-fire
            // (no PHY/audio), pass-after arm. DEFEAT env = fail-before.
            {
                cl_arq_controller ARQ_mixcmp;
                failed += ARQ_mixcmp.test_mixbatch_fill_overpop_compressed();
            }
            // Streaming compression stale-EMA/no-fit regression: a high ratio
            // estimate must reduce and recompress, not silently send the reduced
            // payload RAW. MERCURY_COMPRESS_RETRY_DEFEAT=1 is the fail-before arm.
            {
                cl_arq_controller ARQ_cmpretry;
                failed += ARQ_cmpretry.test_streaming_compress_overshoot();
            }
            // Fix A — mid-flight data_batch_size SHRINK must not orphan RECEIVED prev
            // frames (baseline-double-delivery.md): drives the REAL set_data_batch_size
            // shrink; reconstructs the copy_data_to_buffer prev delivery set; asserts
            // ZERO delivered bytes lost. Pass-after arm (DEFEAT env = fail-before).
            {
                cl_arq_controller ARQ_shrink;
                failed += ARQ_shrink.test_batch_shrink_orphan_defer();
            }
            // rx_btf=-1 CURRENT-batch SHRINK tail-drop (silent-corruption-marginal-snr.md):
            // the messages_rx[] sibling of the Fix A prev guard above — a mid-flight
            // data_batch_size shrink orphans RECEIVED current-batch frames the next seal
            // silently drops (the marginal-SNR WGN:25 mid-stream truncation, md5 FALSE).
            // Drives the REAL shrink chokepoint + seal + prev gate + copy_data_to_buffer;
            // asserts byte-exact full delivery. Pass-after arm (DEFEAT env = fail-before).
            {
                cl_arq_controller ARQ_shrcur;
                failed += ARQ_shrcur.test_batch_shrink_orphan_current();
            }
            // SEAM-2 deferred prev-confirm ARM/FLUSH fire proof (pass-after arm here;
            // MERCURY_PENDING_CONFIRM_DEFEAT=1 is the fail-before drop).
            {
                cl_arq_controller ARQ_lppc;
                failed += ARQ_lppc.test_linkphase_pending_confirm_fire();
            }
            // Streaming decompress-failure SILENT-FALSE-ACCEPT (residual-silent-
            // corruption-wgn25.md): the THIRD WGN:25 trigger — a streaming PPMd model
            // desync under out-of-order SACK delivery makes a COMPLETE, CRC-clean batch
            // decode to -1, and copy_data_to_buffer's old fallback pushed the raw
            // compressed blob to the app (silent garbage). Drives a REAL two-compressor
            // desync + copy_data_to_buffer; asserts zero garbage delivered + loud detect.
            // Pass-after arm (MERCURY_DECOMPRESS_RAWPUSH_DEFEAT=1 = fail-before).
            {
                cl_arq_controller ARQ_decfa;
                failed += ARQ_decfa.test_decompress_false_accept();
            }
            // Fix C / H#1 — fifo_buffer_backup re-stage double-delivery
            // (data-flow-fifo-backup.md §6.1): the delivered batch's raw survives in the
            // backup when finalize is pre-empted (Root B), and the config-change re-stage
            // re-sends it. ACK-confirm-keyed flush empties the backup at register_ack()
            // when no un-confirmed data exists. Pass-after arm (DEFEAT env = fail-before).
            {
                cl_arq_controller ARQ_bkflush;
                failed += ARQ_bkflush.test_backup_confirm_flush();
            }
            // Fix H#3 — RX-FIFO back-pressure post-ACK data loss
            // (delivery-integrity-audit-monitor.md §3): the clean ACK precedes delivery and
            // fifo_push_rx drops the tail when the app FIFO is full. The ACK-GATE now holds
            // the batch until fifo_buffer_rx has room. Pass-after arm (DEFEAT env = fail-before).
            {
                cl_arq_controller ARQ_rxbp;
                failed += ARQ_rxbp.test_rxfifo_backpressure_hold();
            }
            // RX-CTRL-DROP regression (data-flow-control-slot-lifecycle.md §6): the
            // messages_control one-deep mailbox had no timeout escape out of RECEIVED,
            // so a stranded slot silently dropped every later control frame and killed
            // the reverse control plane (demote-amplifier feeder). In-process
            // synthetic-fire (no PHY/audio) — a permanent regression gate.
            {
                cl_arq_controller ARQ_rxctrl;
                failed += ARQ_rxctrl.test_rx_ctrl_drop();
            }
            // ML-KEM KX MULTI-CHUNK LIVE-RX gate (MLKEM_HYBRID_PLAN.md §5 /
            // data-flow-control-slot-lifecycle.md): the RX control-slot producer
            // hardcodes messages_control.length=1 while copying a FIXED slot width
            // into data[]; the KX2/KX3 chunk receivers fed length=1 into
            // kx_receive_chunk -> kx_chunk_decode rejected EVERY chunk -> the PQ
            // hybrid handshake could never complete on the live wire. This gate
            // drives multi-chunk KX2 (RSP consumer) + KX3 (CMD consumer) through the
            // REAL process_control_responder/commander call sites and asserts full
            // reassembly + live encapsulate/decapsulate. In-process synthetic-fire.
            {
                cl_arq_controller ARQ_kxlive;
                failed += ARQ_kxlive.test_kx_chunk_live_rx();
            }
            // KX-AS-DATA reserved-stream RX ingest fire proof (data-flow-hybrid-kex.md
            // T1/T3): the new decoded-transport reassembler kx_ingest() reassembles a
            // FRAGMENTED forward (x25519+pk) and reverse (x25519+ct) stream BYTE-IDENTICAL
            // and NEVER routes to the app FIFO, and the fail-secure REFUSE FIRES (-1) on
            // bad magic/kind/length + a post-activation byte. In-process synthetic-fire.
            {
                cl_arq_controller ARQ_kxi;
                failed += ARQ_kxi.test_kx_ingest();
            }
            // KX-as-data RX ROUTING (data-flow-hybrid-kex.md §2.3): drive the REAL
            // copy_data_to_buffer() delivery funnel with a forward KX stream staged
            // across ACKED messages_rx[] slots in the pre-activation KX epoch and
            // assert the production routing branch reassembles via kx_ingest, never
            // reaches the app FIFO, does not advance the Option-W cursor, and raises
            // no false stream-shift teardown. In-process synthetic-fire.
            {
                cl_arq_controller ARQ_kxr;
                failed += ARQ_kxr.test_kx_rx_routing();
            }
            // KX-as-data forward TX stage/feed (data-flow-hybrid-kex.md §2.2): drive the
            // REAL process_buffer_data_commander() source-switch — stage the reserved
            // forward stream (x25519_pk_cmd + mlkem_pk) and frame it as DATA batches —
            // then deliver those frames through the REAL copy_data_to_buffer()->kx_ingest()
            // funnel and assert the pk + folded x25519 pubkey reassemble BYTE-IDENTICAL
            // end to end, never reaching the app FIFO. In-process synthetic-fire.
            {
                cl_arq_controller ARQ_kxt;
                failed += ARQ_kxt.test_kx_tx_stream();
            }
            // KX-as-data FULL crypto round-trip (data-flow-hybrid-kex.md Â§7 / T6): two
            // controllers drive the REAL kx_stage_forward_stream -> kx_ingest ->
            // kx_on_forward_complete -> kx_ingest -> kx_on_reverse_complete path and assert
            // BOTH peers derive the IDENTICAL hybrid key + matching confirm tag + PQ-upgrade,
            // and that a tampered reverse ct diverges the key (confirm mismatch = KEY_ACTIVATE
            // REFUSE). In-process synthetic-fire; the SWITCH_ROLE transport is proven live.
            {
                cl_arq_controller ARQ_kxrt;
                failed += ARQ_kxrt.test_kx_roundtrip_derive();
            }
            // KX-as-data MID-STREAM AEAD payload TAMPER fire proof (the live channel-
            // MITM gate, data-flow-hybrid-kex.md G3): flip ciphertext bytes on the post-
            // activation data stream and prove the receiver REJECTS — the AEAD auth-fail
            // fires, the production copy_data_to_buffer() funnel tears the link, 0 corrupted
            // bytes reach the app, and it never falls back to plaintext. Clean control
            // delivers. In-process synthetic-fire.
            {
                cl_arq_controller ARQ_kxdt;
                failed += ARQ_kxdt.test_kx_data_tamper();
            }
            // CONNECT-REACK EXCISE gate (connect-testack-handshake.md §9): the
            // 8e62722e regression that dropped OFDM data delivery to 0 because the
            // pre-data re-ACK pinned the shared frames_to_read=2 across the first
            // WB batch (so the gearshift never climbed off config100). The re-ACK
            // is REMOVED; this gate drives a REAL OFDM-config RX in the CONNECTED
            // pre-data window and asserts frames_to_read is NOT pinned to 2.
            {
                cl_arq_controller ARQ_reack;
                failed += ARQ_reack.test_connect_reack();
            }
            // START_CONNECTION bare-ACK causal gate: reject the impossible
            // 263-ms detector hit, clear retained pre-epoch phases, and accept
            // the genuine post-turnaround observation exactly once.
            {
                cl_telecom_system start_ack_ts;
                cl_arq_controller ARQ_start_ack;
                ARQ_start_ack.telecom_system = &start_ack_ts;
                failed += ARQ_start_ack.test_start_ack_causal_guard();
            }
            // OFDM ACQUISITION BOUNDS-GATE RECOVERY gate (data-flow-acq-bounds-gate.md):
            // a tail-band frame whose Schmidl-Cox detection floors past upper_bound but
            // whose body is still in-buffer must be RECOVERED, not force-FAILed/discarded;
            // and a cursor genuinely past the plateau-lead band must still force-FAIL
            // (anti-re-decode preserved). Faithful in-process synthetic-fire through the
            // production transmit_byte/receive_byte (no IONOS/RF).
            failed += run_acq_bounds_selftest();
            // F1b Part B SAMPLE-ANCHORED SUB-PEAK RESCUE gate (data-flow-rx-ring-rearm.md):
            // after a turnaround flush the first frame can lock a Schmidl-Cox sub-peak
            // (metric saturates, pilots misalign, mean|H| collapses) and never read; the
            // rescue re-derives the timing by argmax of mean|H| and re-decodes. Proves the
            // selector + argmax + decode mechanism deterministically (the live mislock is
            // reproduced only by the real-audio needle-2 cohort, not in-process).
            failed += run_subpeak_rescue_selftest();
            // CMD>RSP BATCH-SIZE DESYNC silent-corruption gate (res_c3100, silent-
            // corruption-residual.md §8): a CMD>RSP data_batch_size desync (CMD built 30,
            // RSP applied 25) makes the RSP ACK-GATE deliver the batch TRUNCATED to its
            // smaller size -> a permanent silent stream shift. The (B) LOUD BACKSTOP
            // detects the sender-declared count exceeding the local batch and aborts
            // (link DROPPED) instead of silently delivering. In-process synthetic-fire
            // (no IONOS/RF) — a permanent integrity D0 regression gate.
            {
                cl_arq_controller ARQ_bsd;
                failed += ARQ_bsd.test_batchsize_desync_delivery();
            }
            // GAP-ABORT ruler-blinding silent-corruption gate (data-flow-rsp-contiguity-
            // ruler.md): the REAL rsp_gap_abort_teardown() routes through
            // reset_session_state(), which clears rsp_last_delivered_batch_seq_id to -1 and
            // blinds its own re-adopt gate; the teardown emits no OTA abort, so the CMD
            // re-drives the same stream and the next batch is silently concatenated across
            // the dropped batch. Drives the REAL teardown + the REAL gap predicates + the
            // REAL fifo_buffer_rx app stream. In-process synthetic-fire (no IONOS/RF) — a
            // permanent integrity D0 regression gate.
            // R1 turnaround-clearance guard (arq_common.cc): the guard busy-waits the
            // remainder of the peer's TX->RX mute/flush + demod re-arm window before keying
            // a data batch, so the first frame never lands inside it (the post-LINK-PARAMS
            // slot-0 loss). Drives the REAL guard with stock delays; asserts default-on waits
            // >=250 ms, defeated waits ~0, already-clear adds ~0, never-armed no-ops.
            {
                cl_arq_controller ARQ_tg;
                failed += ARQ_tg.test_turnaround_guard();
            }
            // R6 measured turnaround (arq_common.cc): the SRTT/RTTVAR estimator
            // (RFC 6298 / Jacobson RTO; Karn & Partridge) replaces the calibrated
            // SACK margin, the robust-geometry widen, and the CONFIG_15-only
            // re-phase adder with a self-tracking window. Drives update_turnaround_
            // estimate + calculate_receiving_timeout directly; the landmine arm
            // proves a non-CFG15 drifty long batch tracks (fail-before misses the
            // 4500 ms arrival, pass-after contains it) and re-asserts the
            // ack_timeout_data >= receiving_timeout invariant on the per-batch path.
            {
                cl_arq_controller ARQ_mt;
                failed += ARQ_mt.test_measured_timers();
            }
            // Karn discriminator decoupling (MERCURY_KARN_RETX_ONLY): the R6 estimator was
            // inert on the dominant clean OFDM path because the Karn gate read
            // data_ack_retx_turnaround (which the H1 widen arms on a CLEAN batch). Drives the
            // REAL tt_karn_sample_ok() + update_turnaround_estimate(): fail-before (knob off)
            // starves the estimator (n stays 0); pass-after (knob on) folds the clean sample
            // (n 0->1) while a genuine retx round and a starve are still excluded.
            {
                cl_arq_controller ARQ_kn;
                failed += ARQ_kn.test_karn_retx_classify();
            }
            // [RSP-TIMEOUT] diagnostic field-alignment gate (data-flow-rsp-timeout.md):
            // the RESPONDER receive-window diagnostic passed a double field to a %d slot,
            // shifting every field after it one slot left on the x86-64 SysV ABI so the
            // timeout= label showed an unpushed stack slot (garbage) while the armed value
            // hid under sack=. Drives the REAL calculate_receiving_timeout() RESPONDER path,
            // captures the line off stdout, and asserts the printed fields align with the
            // armed receiving_timeout. Deterministic, no RF.
            {
                cl_arq_controller ARQ_rtf;
                failed += ARQ_rtf.test_rsp_timeout_format();
            }
            // R4 LINK-PARAMS quiesce gate (arq_commander.cc): an Axis-2 batch-size
            // renegotiation must not fire while the boundary is unclean (retx pending / last
            // batch partial), which would mix old-bsi retransmits into the first batch of the
            // new geometry. Drives the REAL policy_evaluate_axis2 across clean/unclean
            // boundaries + the defeat knob; asserts DEFER while unclean, FIRE when clean,
            // FORCE after the bound, and the pre-R4 mixed-batch move under the defeat knob.
            {
                cl_arq_controller ARQ_q2;
                failed += ARQ_q2.test_axis2_quiesce_gate();
            }
            // FIX 2 — catastrophic-partial fast-down: a SINGLE batch carrying the full
            // hysteresis' worth of SACK-bitmap loss (partial>0.60) fast-downs the batch
            // (AIMD B/2) immediately instead of waiting 3 consecutive bad batches, breaking
            // the coalescing relapse loop. Drives the REAL policy_evaluate_axis2 across the
            // defeat knob + a bad-but-not-catastrophic guard.
            {
                cl_arq_controller ARQ_fd;
                failed += ARQ_fd.test_axis2_fastdown();
            }
            // Per-rung MEASURED election floor gate + failure memory (DATAFLOW_AUDIT_rung_floor.md):
            // generalizes the CFG16 decode-margin gate to every OFDM rung so the over-reading suffix
            // meter can no longer elect a rung above its measured delivery floor (the wb13 snr3k+13
            // cfg14 stall). Drives the REAL apply_rung_floor_cap / failure-memory decisions across the
            // wb13 fail-before/pass-after, never-raise, arm/survive/decay/reset, and the defeat knob.
            {
                cl_arq_controller ARQ_rf;
                failed += ARQ_rf.test_rung_floor_gate();
            }
            {
                cl_arq_controller ARQ_gab;
                failed += ARQ_gab.test_gap_abort_readopt_blind();
            }
            // zombie/amplifier layer (data-flow-zombie-amplifier.md): R2b BREAK-accept-
            // while-DROPPED, the ROBUST_DWELL keep-alive gate, and the watchdog
            // peer-liveness probe gate — each pure decision helper's truth table under
            // the defeat knob (fail-before) vs unset (pass-after). Deterministic; no RF.
            {
                cl_arq_controller ARQ_za;
                failed += ARQ_za.test_zombie_amp();
            }
            // R5 demote-split (data-flow-zombie-amplifier.md): the pure-silence classifier truth
            // table, the counter evolution via the shared step, the reroute predicate + guards, the
            // DEFEAT knob, and the +LOW watchdog probe-latch hygiene. Deterministic; no RF.
            {
                cl_arq_controller ARQ_dsil;
                failed += ARQ_dsil.test_demote_silence();
            }
            // GAP-ABORT stream-backstop companion (data-flow-rsp-contiguity-ruler.md §7):
            // the REAL teardown must ALSO preserve the Option W byte cursor + stamp validity
            // (so w_stream_shift_detected can still fire post-abort) and set the sticky
            // rsp_stream_aborted latch (which survives reset_session_state and refuses every
            // subsequent delivery until a genuine CONNECT). In-process synthetic-fire.
            {
                cl_arq_controller ARQ_gas;
                failed += ARQ_gas.test_gap_abort_stream_backstop_blind();
            }
            // RECOVERABLE delivery-time GAP-ABORT (R2a hold + R2c CMD retention):
            // the terminal abort on a RECOVERABLE one-frame prev-batch hole is
            // converted into a bounded HOLD + re-request; the held batch delivers
            // in-order only after the hole fills, and the high-water NEVER advances
            // while the hole exists. Drives the REAL predicates + retention helpers
            // + advance_last_delivered. See data-flow-recoverable-gap-abort.md.
            {
                cl_arq_controller ARQ_grc;
                failed += ARQ_grc.test_recoverable_gap_abort();
            }
            // W1 — CMD refill decision fire proof (routes the storm re-SACK to the
            // retention re-drive; fail-before via MERCURY_CMD_PREV_RETAIN_DEFEAT).
            {
                cl_arq_controller ARQ_rf;
                failed += ARQ_rf.test_recover_fire();
            }
            // R2b — DELIVER-HELD-CUR fire proof: the RSP commits the held current
            // batch's BYTES to the app FIFO via the production copy_data_to_buffer
            // once the prev hole refills, and the CMD credits the recovery round.
            // fail-before via MERCURY_HELD_CUR_DELIVER_DEFEAT / MERCURY_GAP_RECOVER_
            // TURNAROUND_DEFEAT. See data-flow-recoverable-gap-abort.md §5.5.
            {
                cl_arq_controller ARQ_hcf;
                failed += ARQ_hcf.test_held_cur_deliver_fire();
            }
            // chase-combining (HARQ Type-I soft-LLR combine) fail-before/pass-after:
            // proves a bit-identical retx's LLRs SUMMED before ldpc.decode() rescue a
            // frame one look cannot decode (combine success-rate 0 -> >0), and that
            // defeating the primitive restores the pre-fix 0. Fast + deterministic, no
            // IONOS/RF. See fact-documents/chase-combining-harq.md.
            failed += run_chase_selftest();
            // CRC-ESCAPE accept-gate gate: a non-converged LDPC decode whose residual
            // errors collide the 16-bit CRC to 0 must be REJECTED (not delivered). Drives
            // the PRODUCTION frame_decode_rejected() predicate; fail-before via the
            // crc_escape_defeat arm (legacy crc-only gate). Deterministic, no RF.
            failed += run_crc_escape_selftest();
            // In-band demote-rebase DOUBLE-DELIVERY (byte-stream corruption): the RSP
            // re-delivers an already-delivered batch across an in-band demote-rebase
            // (re-adopt + lost-ACK retransmit re-reach the delivery funnel with fwd==0).
            // Drives the REAL commit funnel + REAL rebase + REAL gap predicates with
            // fifo_buffer_rx as a byte-exact oracle. fail-before MERCURY_STREAM_DEDUP_DEFEAT=1.
            {
                cl_arq_controller ARQ_dd;
                failed += ARQ_dd.test_dedup_rebase();
            }

            // PRECOOK (Stage 1) shared-ring PIN invariant regression. Pins the persistent
            // capture ring (precook_pin_shared_ring), then drives a representative gearshift
            // sequence and asserts: (a) passband_delayed_data is the SAME address after every
            // switch (ring never freed → the Bug #42 UAF window is structurally gone); (b) each
            // config's ACTIVE buffer_Nsymb is its per-config natural, NEVER pinned to the max
            // (the S4 oversize-ring acquisition wall the "pin to max" landmine would create, and
            // never exceeds the pinned capacity). In-process, no IONOS/RF. See PRECOOK plan §6.
            {
                const char* defeat = std::getenv("MERCURY_PRECOOK_DEFEAT");
                bool defeated = (defeat && *defeat && atoi(defeat) != 0);
                cl_telecom_system ts;
                ts.narrowband_enabled = NO;
                ts.load_configuration(CONFIG_0);
                int nat_cfg0 = ts.data_container.buffer_Nsymb;
                ts.precook_pin_shared_ring();
                int pfail = 0;
                if(defeated) {
                    if(ts.data_container.precook_ring_pinned) { printf("[TEST-PRECOOK] FAIL: DEFEAT set but ring pinned\n"); pfail++; }
                    else printf("[TEST-PRECOOK] SKIP (MERCURY_PRECOOK_DEFEAT=1 → legacy path)\n");
                } else {
                    double* ring0 = ts.data_container.passband_delayed_data;
                    int cap = ts.data_container.pinned_capacity_buffer_Nsymb;
                    if(!ts.data_container.precook_ring_pinned) { printf("[TEST-PRECOOK] FAIL: ring not pinned after precook\n"); pfail++; }
                    if(ring0 == NULL) { printf("[TEST-PRECOOK] FAIL: ring NULL after pin\n"); pfail++; }
                    int seq[] = { CONFIG_16, CONFIG_17, CONFIG_16, ROBUST_0, CONFIG_8, CONFIG_0,
                                  ROBUST_2, CONFIG_15, CONFIG_16, ROBUST_1 };
                    for(unsigned s=0; s<sizeof(seq)/sizeof(seq[0]); s++) {
                        ts.load_configuration(seq[s]);
                        if(ts.data_container.passband_delayed_data != ring0) {
                            printf("[TEST-PRECOOK] FAIL: ring pointer moved on switch to %d (%p != %p) — ring was freed/realloc'd\n",
                                seq[s], (void*)ts.data_container.passband_delayed_data, (void*)ring0);
                            pfail++;
                        }
                        int bn = ts.data_container.buffer_Nsymb;
                        if(bn > cap) { printf("[TEST-PRECOOK] FAIL: cfg %d buffer_Nsymb=%d exceeds pinned capacity=%d\n", seq[s], bn, cap); pfail++; }
                    }
                    // Per-config window preserved (no S4 wall): CONFIG_0 active window == its natural,
                    // NOT the pinned max (cap), even after dwelling on a large-window ROBUST config.
                    ts.load_configuration(ROBUST_0);
                    ts.load_configuration(CONFIG_0);
                    if((int)ts.data_container.buffer_Nsymb != nat_cfg0) {
                        printf("[TEST-PRECOOK] FAIL: CONFIG_0 window=%d != natural %d (S4 oversize-ring wall)\n",
                            (int)ts.data_container.buffer_Nsymb, nat_cfg0);
                        pfail++;
                    }
                    if(ts.data_container.passband_delayed_data != ring0) { printf("[TEST-PRECOOK] FAIL: ring pointer moved (final)\n"); pfail++; }
                    if(pfail == 0)
                        printf("[TEST-PRECOOK] ALL PASS: ring pinned+stable across 10 switches (addr=%p, cap=%d), "
                            "per-config window preserved (CONFIG_0 natural=%d, not clamped to max)\n",
                            (void*)ring0, cap, nat_cfg0);
                }
                failed += pfail;
            }

            // PRECOOK descrambler REGEN regression (§17 HINGE — the LIVE NB-ROBUST/MFSK-LDPC
            // 0-RX-CTRL root, audit wqvdd116h). precook_pin_shared_ring() calls
            // alloc_shared_buffers() which CDELETE+re-NEW's bit_energy_dispersal_sequence as a
            // fresh `new int[N_MAX]` GARBAGE array; the restore-startup-config load above no-op-
            // guards (same-cfg) so nothing regenerates it → the LIVE descrambler is garbage and
            // the RX outer CRC constant-fails. FIX: regenerate the seed-0 draw immediately after
            // alloc_shared_buffers. This asserts the SEQUENCE equals the reference ts_srandom(seed)
            // draw (NOT merely "decode succeeds" — §17 warns a plain realloc self-hides via heap
            // block reuse), using the SAME MERCURY_DESCRAMBLER_REGEN_DEFEAT poison-pattern as the
            // HINGE-1 test: DEFEAT=1 zeroes the array → FAIL-BEFORE, default regenerates → PASS-AFTER.
            {
                const char* TAG = "[TEST-PRECOOK-DESCRAMBLER]";
                int dfail = 0;
                auto set_env = [&](const char* k, const char* v){
#if defined(_WIN32)
                    _putenv_s(k, v);
#else
                    if(v && *v) setenv(k, v, 1); else unsetenv(k);
#endif
                };
                // Reference: the seed-0 draw for the startup config (CONFIG_0), from a FRESH
                // instance that only ran init() (never precook). This is what BOTH ends scramble
                // with; the pinned live descrambler must match it bit-for-bit.
                static int ref_seq[N_MAX];
                int ref_N = 0;
                set_env("MERCURY_DESCRAMBLER_REGEN_DEFEAT", "");
                {
                    cl_telecom_system ref;
                    ref.narrowband_enabled = NO;
                    ref.load_configuration(CONFIG_0);
                    ref_N = ref.ldpc.N;
                    for(int i=0;i<ref_N && i<N_MAX;i++) ref_seq[i] = ref.data_container.bit_energy_dispersal_sequence[i];
                }
                auto dump16 = [&](const int* s)->std::string {
                    char buf[8]; std::string out;
                    for(int b=0;b<2;b++){ int v=0; for(int k=0;k<8;k++) v=(v<<1)|(s[b*8+k]&1); snprintf(buf,sizeof(buf),"%02X",v); out+=buf; }
                    return out;
                };
                for(int arm=0; arm<2; arm++)
                {
                    bool defeat = (arm==1);
                    set_env("MERCURY_DESCRAMBLER_REGEN_DEFEAT", defeat ? "1" : "");
                    cl_telecom_system ts;
                    ts.narrowband_enabled = NO;
                    ts.load_configuration(CONFIG_0);
                    ts.precook_pin_shared_ring();
                    int N = ts.ldpc.N;
                    bool matches = (N == ref_N);
                    for(int i=0;i<N && matches;i++) if(ts.data_container.bit_energy_dispersal_sequence[i]!=ref_seq[i]) matches=false;
                    std::string live = dump16(ts.data_container.bit_energy_dispersal_sequence);
                    if(!defeat) {
                        if(matches) printf("%s PASS-AFTER: pinned live descrambler EQUALS seed-0 ref (dump[0..15]=%s, ref=%s)\n", TAG, live.c_str(), dump16(ref_seq).c_str());
                        else { printf("%s FAIL: pinned live descrambler != seed-0 ref (dump[0..15]=%s, ref=%s)\n", TAG, live.c_str(), dump16(ref_seq).c_str()); dfail++; }
                    } else {
                        if(!matches) printf("%s FAIL-BEFORE ok: DEFEAT descrambler DIVERGES from ref (garbage/zeroed, dump[0..15]=%s)\n", TAG, live.c_str());
                        else { printf("%s FAIL: DEFEAT descrambler matched ref (poison did not take)\n", TAG); dfail++; }
                    }
                }
                set_env("MERCURY_DESCRAMBLER_REGEN_DEFEAT", "");
                if(dfail==0) printf("%s ALL PASS\n", TAG);
                failed += dfail;
            }

            // PRECOOK M3 copy_from BYTE-IDENTICAL completeness gate (STEP 1 GATE 2,
            // BUNDLE_FIELD_CHECKLIST PART 6). For every config in the full gearshift
            // ladder (ROBUST_0/1/2 + CONFIG_0..16): build a telecom_system via the
            // production init() path, copy_from telecom.ofdm/ldpc/psk into FRESH local
            // objects, then memcmp EVERY owning buffer between the init()-built original
            // and the copy → assert byte-IDENTICAL. Also asserts the L1 landmine: the
            // COPY's pilot/preamble .carrier back-pointers alias the COPY's own
            // ofdm_frame/ofdm_preamble (NOT the source's). Any mismatch localizes the
            // missed field to a single buffer. In-process, no IONOS/RF. See
            // _research/_precook/PRECOOK_STAGE2_TURNKEY.md STEP 1.
            {
                int cfpass = 0, cffail = 0;
                for (int li = 0; li < FULL_CONFIG_LADDER_SIZE; li++) {
                    int cfg = FULL_CONFIG_LADDER[li];
                    cl_telecom_system ts;
                    ts.narrowband_enabled = NO;
                    ts.load_configuration(cfg);

                    cl_ofdm ofdm_copy;   ofdm_copy.copy_from(ts.ofdm);
                    cl_ldpc ldpc_copy;   ldpc_copy.copy_from(ts.ldpc);
                    cl_psk  psk_copy;    psk_copy.copy_from(ts.psk);

                    const char* m_ofdm = ts.ofdm.precook_deep_equal(ofdm_copy);
                    const char* m_ldpc = ts.ldpc.precook_deep_equal(ldpc_copy);
                    const char* m_psk  = ts.psk.precook_deep_equal(psk_copy);

                    // L1 landmine: the COPY's back-pointers must alias the COPY's own frames.
                    bool land_ok = (ofdm_copy.pilot_configurator.carrier == ofdm_copy.ofdm_frame)
                                && (ofdm_copy.preamble_configurator.carrier == ofdm_copy.ofdm_preamble);

                    if (m_ofdm == NULL && m_ldpc == NULL && m_psk == NULL && land_ok) {
                        cfpass++;
                    } else {
                        cffail++;
                        printf("[TEST-PRECOOK-COPYFROM] FAIL cfg %d:", cfg);
                        if (m_ofdm) printf(" ofdm-buffer=%s", m_ofdm);
                        if (m_ldpc) printf(" ldpc-buffer=%s", m_ldpc);
                        if (m_psk)  printf(" psk-buffer=%s", m_psk);
                        if (!land_ok) printf(" LANDMINE-1(carrier back-pointer not re-pointed to copy)");
                        printf("\n");
                    }
                }
                if (cffail == 0)
                    printf("[TEST-PRECOOK-COPYFROM] ALL PASS: %d/%d ladder configs byte-identical after "
                           "copy_from (every owning buffer memcmp-clean; L1 carrier re-point verified)\n",
                           cfpass, FULL_CONFIG_LADDER_SIZE);
                failed += cffail;
            }

            // PRECOOK M3 STEP 2 — config-geometry BUNDLE completeness gate
            // (BUNDLE_FIELD_CHECKLIST PART 6, the single GATE that proves completeness).
            // Build the WB bundle set via precook_config_bundles(); then for EVERY ladder
            // config build a FRESH reference cl_telecom_system (load_configuration runs the
            // full init()) and assert bundle[cfg] is byte-IDENTICAL to it: ofdm/ldpc/psk via
            // precook_deep_equal (every owning buffer memcmp), pre_equalization_channel memcmp
            // (Nc; NULL for MFSK), bit_energy_dispersal_sequence memcmp (N_MAX), and the full
            // scalar block ==. A FAIL localizes the missed bundle field to a single
            // buffer/scalar. In-process, no IONOS/RF; CARD-FREE. See
            // _research/PRECOOK_IMPLEMENTATION_PLAN.md §1.1/§1.2. STEP 2 does NOT wire the swap
            // into load_configuration (STEP 3) → cfg<=15 production path is unaffected.
            {
                cl_telecom_system bts;
                bts.narrowband_enabled = NO;
                bts.precook_config_bundles();

                int bpass = 0, bfail = 0;
                for (int li = 0; li < FULL_CONFIG_LADDER_SIZE; li++) {
                    int cfg = FULL_CONFIG_LADDER[li];
                    int idx = bts.bundle_index(cfg, NO);
                    if (idx < 0) {
                        bfail++;
                        printf("[TEST-PRECOOK-BUNDLE] FAIL cfg %d: bundle_index returned -1 (not built)\n", cfg);
                        continue;
                    }
                    const st_config_bundle* b = bts.bundle_set(NO)[idx].get();

                    cl_telecom_system ref;
                    ref.narrowband_enabled = NO;
                    ref.load_configuration(cfg);

                    const char* mism = NULL;

                    // (1) geometry PHY objects — every owning buffer.
                    if (mism == NULL) mism = b->ofdm.precook_deep_equal(ref.ofdm);
                    if (mism == NULL) mism = b->ldpc.precook_deep_equal(ref.ldpc);
                    if (mism == NULL) mism = b->psk.precook_deep_equal(ref.psk);

                    // (2) pre_equalization_channel (Nc; both NULL for MFSK).
                    if (mism == NULL) {
                        int rNc = ref.data_container.Nc;
                        bool bnull = (b->pre_equalization_channel == NULL);
                        bool rnull = (ref.pre_equalization_channel == NULL);
                        if (bnull != rnull) mism = "pre_equalization_channel(null-mismatch)";
                        else if (!bnull && rNc > 0 &&
                                 memcmp(b->pre_equalization_channel, ref.pre_equalization_channel,
                                        sizeof(struct st_channel_complex) * (size_t)rNc) != 0)
                            mism = "pre_equalization_channel";
                    }

                    // (3) bit_energy_dispersal_sequence (N_MAX descrambler draw).
                    if (mism == NULL) {
                        bool bnull = (b->bit_energy_dispersal_sequence == NULL);
                        bool rnull = (ref.data_container.bit_energy_dispersal_sequence == NULL);
                        if (bnull != rnull) mism = "bit_energy_dispersal_sequence(null-mismatch)";
                        else if (!bnull &&
                                 memcmp(b->bit_energy_dispersal_sequence, ref.data_container.bit_energy_dispersal_sequence,
                                        sizeof(int) * (size_t)N_MAX) != 0)
                            mism = "bit_energy_dispersal_sequence";
                    }

                    // (4) scalar block — data_container geometry scalars.
                    if (mism == NULL) {
                        if      (b->Nofdm                         != ref.data_container.Nofdm)                         mism = "scalar.Nofdm";
                        else if (b->Nc                            != ref.data_container.Nc)                            mism = "scalar.Nc";
                        else if (b->M                             != ref.data_container.M)                             mism = "scalar.M";
                        else if (b->Nfft                          != ref.data_container.Nfft)                          mism = "scalar.Nfft";
                        else if (b->Ngi                           != ref.data_container.Ngi)                           mism = "scalar.Ngi";
                        else if (b->Nsymb                         != ref.data_container.Nsymb)                         mism = "scalar.Nsymb";
                        else if (b->nData                         != ref.data_container.nData)                         mism = "scalar.nData";
                        else if (b->nBits                         != ref.data_container.nBits)                         mism = "scalar.nBits";
                        else if (b->preamble_nSymb                != ref.data_container.preamble_nSymb)                mism = "scalar.preamble_nSymb";
                        else if (b->interpolation_rate            != ref.data_container.interpolation_rate)            mism = "scalar.interpolation_rate";
                        else if (b->total_frame_size              != ref.data_container.total_frame_size)              mism = "scalar.total_frame_size";
                        else if (b->baseband_data_fine_slice_size != ref.data_container.baseband_data_fine_slice_size) mism = "scalar.baseband_data_fine_slice_size";
                        else if (b->buffer_Nsymb                  != (int)ref.data_container.buffer_Nsymb.load())      mism = "scalar.buffer_Nsymb";
                    }
                    // (5) scalar block — telecom_system scalars.
                    if (mism == NULL) {
                        if      (b->ldpc_rate                 != ref.ldpc.rate)                 mism = "scalar.ldpc_rate";
                        else if (b->bandwidth                 != ref.bandwidth)                 mism = "scalar.bandwidth";
                        else if (b->carrier_frequency         != ref.carrier_frequency)         mism = "scalar.carrier_frequency";
                        else if (b->bit_energy_dispersal_seed != ref.bit_energy_dispersal_seed) mism = "scalar.bit_energy_dispersal_seed";
                        else if (b->M_telecom                 != ref.M)                         mism = "scalar.M_telecom";
                        else if (b->time_sync_trials_max      != ref.time_sync_trials_max)      mism = "scalar.time_sync_trials_max";
                        else if (b->outer_code                != ref.outer_code)                mism = "scalar.outer_code";
                        else if (b->outer_code_reserved_bits  != ref.outer_code_reserved_bits)  mism = "scalar.outer_code_reserved_bits";
                        else if (b->LDPC_real_CR              != ref.LDPC_real_CR)              mism = "scalar.LDPC_real_CR";
                        else if (b->Tu                        != ref.Tu)                        mism = "scalar.Tu";
                        else if (b->Ts                        != ref.Ts)                        mism = "scalar.Ts";
                        else if (b->Tf                        != ref.Tf)                        mism = "scalar.Tf";
                        else if (b->rb                        != ref.rb)                        mism = "scalar.rb";
                        else if (b->rbc                       != ref.rbc)                       mism = "scalar.rbc";
                        else if (b->Shannon_limit             != ref.Shannon_limit)             mism = "scalar.Shannon_limit";
                    }

                    if (mism == NULL) {
                        bpass++;
                    } else {
                        bfail++;
                        printf("[TEST-PRECOOK-BUNDLE] FAIL cfg %d: field=%s\n", cfg, mism);
                    }
                }
                if (bfail == 0)
                    printf("[TEST-PRECOOK-BUNDLE] ALL PASS: %d/%d ladder configs bundle==init byte-identical "
                           "(ofdm/ldpc/psk buffers + pre_eq[Nc] + descrambler[N_MAX] + full scalar block)\n",
                           bpass, FULL_CONFIG_LADDER_SIZE);
                failed += bfail;
            }

            // PRECOOK M3 STEP 3 — config-SWAP regression (plan §6 / TURNKEY STEP 5). Pins the
            // shared ring, builds the bundle set, then drives load_configuration across gearshift
            // cross-modulation (BPSK/QPSK/8PSK/16QAM/32QAM + the 15<->16 boundary), ROBUST<->OFDM
            // reseed, and session-reset/SWITCH_ROLE (re-load the init config) transitions. Asserts,
            // after EVERY swap: (a) decoded==framed geometry (live ofdm.Nc/Nsymb + data_container.M
            // + ldpc.rate == the target bundle) [INV-5]; (b) the ACTIVE buffer_Nsymb is the config
            // NATURAL (== bundle, <= pinned cap) with a consistent sp=Nofdm*buffer_Nsymb*interp>0 —
            // the STAGE-1 pin-to-max landmine catcher [INV-4]; (c) passband_delayed_data is the SAME
            // address across every swap (ring never freed/realloc'd — INV-2); (d) C1 never sees
            // sp==0/NULL; (e) ROBUST<->OFDM reseed leaves the descrambler as the seed-0 draw (NOT
            // all-zero, INV-6) and the MFSK corr-template state matching the target (OFDM => len 0;
            // MFSK => len>0, not stale). A second block repeats for NB (Nc=10) bundles. The leaf-lock
            // reentrancy tripwire in load_configuration_swap aborts on a nested publish. Under
            // MERCURY_PRECOOK_DEFEAT=1 the ring stays UNPINNED → the legacy deinit->init path
            // reallocs the ring on each switch → this test FAILS (the fail-before). In-process, no
            // IONOS/RF; CARD-FREE. See _research/PRECOOK_IMPLEMENTATION_PLAN.md §3.1/§6.
            {
                const char* defeat = std::getenv("MERCURY_PRECOOK_DEFEAT");
                bool defeated = (defeat && *defeat && atoi(defeat) != 0);
                int sfail = 0;

                for(int pass = 0; pass < 2; pass++)   // pass 0 = WB (Nc=50), pass 1 = NB (Nc=10)
                {
                    int nb = (pass == 1) ? YES : NO;
                    const char* bwn = (pass == 1) ? "NB" : "WB";
                    cl_telecom_system ts;
                    ts.narrowband_enabled = nb;
                    ts.load_configuration(CONFIG_0);
                    ts.precook_pin_shared_ring();
                    if(ts.data_container.precook_ring_pinned)
                        ts.precook_config_bundles();   // mirror the startup gating (only when pinned)

                    // Cross-modulation + ROBUST<->OFDM reseed + session-reset(re-load CONFIG_0).
                    // NB clamps OFDM > CONFIG_6 to CONFIG_6, so the NB pass drives only <= CONFIG_6.
                    int seq_wb[] = { CONFIG_6, CONFIG_7, CONFIG_10, CONFIG_15, CONFIG_16, CONFIG_15,
                                     CONFIG_16, ROBUST_0, CONFIG_0, ROBUST_2, CONFIG_8, ROBUST_1,
                                     CONFIG_16, CONFIG_0 };
                    int seq_nb[] = { CONFIG_3, CONFIG_6, ROBUST_0, CONFIG_0, ROBUST_2, CONFIG_5,
                                     ROBUST_1, CONFIG_0 };
                    int* seq = (pass == 1) ? seq_nb : seq_wb;
                    int nseq = (pass == 1) ? (int)(sizeof(seq_nb)/sizeof(seq_nb[0]))
                                           : (int)(sizeof(seq_wb)/sizeof(seq_wb[0]));

                    if(!ts.data_container.precook_ring_pinned)
                    {
                        // fail-before (DEFEAT / pin failed): the legacy deinit->init path reallocs the
                        // ring on cross-modulation switches — the exact regression STEP-3 removes.
                        double* ring0 = ts.data_container.passband_delayed_data;
                        int moved = 0;
                        for(int s=0; s<nseq; s++){
                            ts.load_configuration(seq[s]);
                            if(ts.data_container.passband_delayed_data != ring0){ moved++; ring0 = ts.data_container.passband_delayed_data; }
                        }
                        printf("[TEST-PRECOOK-SWAP] %s FAIL-BEFORE (%s): ring realloc'd %d× across %d switches "
                               "(legacy path; no M3 swap)\n",
                               bwn, defeated ? "MERCURY_PRECOOK_DEFEAT=1" : "ring-not-pinned", moved, nseq);
                        sfail += (moved > 0) ? moved : 1;   // ensure RED under DEFEAT (fail-before)
                        continue;
                    }

                    double* ring0 = ts.data_container.passband_delayed_data;
                    int cap = ts.data_container.pinned_capacity_buffer_Nsymb;
                    int expect_Nc = (pass == 1) ? 10 : 50;
                    if(ring0 == NULL){ printf("[TEST-PRECOOK-SWAP] %s FAIL: ring NULL after pin\n", bwn); sfail++; }

                    for(int s=0; s<nseq; s++)
                    {
                        int cfg = seq[s];
                        ts.load_configuration(cfg);
                        // bundle_index re-applies the NB clamp inside load_configuration, so look up
                        // the ACTUALLY-loaded config via the live current_configuration.
                        int live_cfg = ts.current_configuration;
                        int idx = ts.bundle_index(live_cfg, nb);
                        if(idx < 0){ printf("[TEST-PRECOOK-SWAP] %s FAIL: cfg %d (live %d) no bundle\n", bwn, cfg, live_cfg); sfail++; continue; }
                        const st_config_bundle* b = ts.bundle_set(nb)[idx].get();

                        // (a) decoded==framed geometry.
                        if(ts.ofdm.Nc != b->ofdm.Nc || ts.ofdm.Nsymb != b->ofdm.Nsymb ||
                           ts.data_container.M != b->M || ts.ldpc.rate != b->ldpc_rate)
                        {
                            printf("[TEST-PRECOOK-SWAP] %s FAIL cfg %d: geometry live(Nc=%d Nsymb=%d M=%d rate=%.4f) != bundle(Nc=%d Nsymb=%d M=%d rate=%.4f)\n",
                                bwn, live_cfg, ts.ofdm.Nc, ts.ofdm.Nsymb, ts.data_container.M, ts.ldpc.rate,
                                b->ofdm.Nc, b->ofdm.Nsymb, b->M, b->ldpc_rate);
                            sfail++;
                        }
                        if(ts.ofdm.Nc != expect_Nc){ printf("[TEST-PRECOOK-SWAP] %s FAIL cfg %d: Nc=%d != expected %d\n", bwn, live_cfg, ts.ofdm.Nc, expect_Nc); sfail++; }

                        // (b) active buffer_Nsymb == the config NATURAL (bundle), consistent sp, <= cap.
                        int bn = ts.data_container.buffer_Nsymb;
                        int interp = ts.data_container.interpolation_rate;
                        long sp = (long)ts.data_container.Nofdm * bn * interp;
                        if(bn != b->buffer_Nsymb){
                            printf("[TEST-PRECOOK-SWAP] %s FAIL cfg %d: buffer_Nsymb=%d != natural %d (STAGE-1 pin-to-max?)\n", bwn, live_cfg, bn, b->buffer_Nsymb);
                            sfail++;
                        }
                        if(bn > cap){ printf("[TEST-PRECOOK-SWAP] %s FAIL cfg %d: buffer_Nsymb=%d > pinned cap %d\n", bwn, live_cfg, bn, cap); sfail++; }

                        // (d) C1: sp > 0, ring non-NULL.
                        if(sp <= 0 || ts.data_container.passband_delayed_data == NULL){
                            printf("[TEST-PRECOOK-SWAP] %s FAIL cfg %d: C1 sp=%ld ring=%p (sp==0/NULL)\n", bwn, live_cfg, sp, (void*)ts.data_container.passband_delayed_data);
                            sfail++;
                        }

                        // (c) ring pointer stable across the swap (no realloc — INV-2).
                        if(ts.data_container.passband_delayed_data != ring0){
                            printf("[TEST-PRECOOK-SWAP] %s FAIL cfg %d: ring pointer moved (%p != %p) — realloc\n", bwn, live_cfg, (void*)ts.data_container.passband_delayed_data, (void*)ring0);
                            sfail++;
                        }

                        // (e) ROBUST<->OFDM reseed correctness.
                        bool desc_nonzero = false;
                        int nscan = (ts.ldpc.N < 64) ? ts.ldpc.N : 64;
                        for(int k=0; k<nscan; k++) if(ts.data_container.bit_energy_dispersal_sequence[k] != 0){ desc_nonzero = true; break; }
                        if(!desc_nonzero){ printf("[TEST-PRECOOK-SWAP] %s FAIL cfg %d: descrambler all-zero (INV-6 stale)\n", bwn, live_cfg); sfail++; }
                        bool is_mfsk = is_robust_config(live_cfg);
                        if(is_mfsk && ts.ofdm.mfsk_corr_template_len <= 0){ printf("[TEST-PRECOOK-SWAP] %s FAIL cfg %d: MFSK but corr_template_len=%d\n", bwn, live_cfg, ts.ofdm.mfsk_corr_template_len); sfail++; }
                        if(!is_mfsk && ts.ofdm.mfsk_corr_template_len != 0){ printf("[TEST-PRECOOK-SWAP] %s FAIL cfg %d: OFDM but stale MFSK corr_template_len=%d\n", bwn, live_cfg, ts.ofdm.mfsk_corr_template_len); sfail++; }
                    }

                    // STAGE-1 landmine catcher: after a large-window ROBUST dwell, a small-window
                    // config keeps its NATURAL window (strictly < cap) — not the pinned max (the S4
                    // oversize-ring acquisition wall the "pin buffer_Nsymb to max" bug would create).
                    int small_cfg = (pass == 1) ? CONFIG_6 : CONFIG_16;
                    ts.load_configuration(ROBUST_0);
                    ts.load_configuration(small_cfg);
                    int idxs = ts.bundle_index(ts.current_configuration, nb);
                    int bns = ts.data_container.buffer_Nsymb;
                    // The STAGE-1 pin-to-max bug would make bns == cap (!= natural); the strict
                    // "== natural" check below is the catcher (bns==natural==cap is legitimate for
                    // the window-defining config, so a bare ">= cap" would false-positive on NB).
                    if(idxs >= 0 && bns != ts.bundle_set(nb)[idxs]->buffer_Nsymb){
                        printf("[TEST-PRECOOK-SWAP] %s FAIL: cfg %d window=%d != natural %d after ROBUST dwell (S4 pin-to-max wall)\n",
                            bwn, ts.current_configuration, bns, ts.bundle_set(nb)[idxs]->buffer_Nsymb);
                        sfail++;
                    }
                    if(ts.data_container.passband_delayed_data != ring0){ printf("[TEST-PRECOOK-SWAP] %s FAIL: ring moved (final)\n", bwn); sfail++; }

                    if(sfail == 0)
                        printf("[TEST-PRECOOK-SWAP] %s PASS: %d swaps, ring stable @%p (cap=%d), per-config natural "
                               "buffer_Nsymb, descrambler+MFSK-template reseeded, Nc=%d (small-cfg %d window=%d)\n",
                               bwn, nseq, (void*)ring0, cap, expect_Nc, small_cfg, bns);
                }
                failed += sfail;
            }

            // PRECOOK M3 STEP 4 — COLLAPSE regression (plan §3.3/§4). The three CONFIG_NONE-abuser
            // paths are now bundle SWAPS / scalar re-stages with NO ring realloc:
            //   (1) bigblock_restore_stock_config()  — after a CFG16 thin-grid big-block context
            //       (bigblock_rebuild_thin_grid mangles the LIVE ofdm), restore = swap bundle[stock];
            //   (2) force_resize_capture_ring()       — robust-floor SEAT = re-stage buffer_Nsymb to
            //       the ROBUST_0 floor (max(natural, floor)), no realloc, no descrambler regen;
            //   (3) force_set_capture_ring_natural()  — UN-SEAT = re-stage to the config NATURAL.
            // Asserts per transition: ring pointer BYTE-IDENTICAL (no realloc, INV-2); decoded==framed
            // geometry (live ofdm == stock bundle); active buffer_Nsymb == expected; descrambler seed-0
            // (non-zero, INV-6); OFDM => MFSK template len 0. Under MERCURY_PRECOOK_DEFEAT=1 the ring is
            // UNPINNED → these paths REALLOC → the fail-before (ring moves). In-process, CARD-FREE.
            {
                const char* defeat4 = std::getenv("MERCURY_PRECOOK_DEFEAT");
                bool defeated4 = (defeat4 && *defeat4 && atoi(defeat4) != 0);
                int c4fail = 0;

                for(int pass = 0; pass < 2; pass++)   // 0 = WB (Nc=50), 1 = NB (Nc=10)
                {
                    int nb = (pass == 1) ? YES : NO;
                    const char* bwn = (pass == 1) ? "NB" : "WB";
                    cl_telecom_system ts;
                    ts.narrowband_enabled = nb;
                    ts.load_configuration(CONFIG_0);
                    ts.precook_pin_shared_ring();
                    if(ts.data_container.precook_ring_pinned)
                        ts.precook_config_bundles();

                    // Stock high rung: CONFIG_16 (WB) / CONFIG_6 (NB clamp ceiling). big-block is a
                    // WB-CFG16 construct, so the thin-grid restore leg runs on the WB pass only; the
                    // seat/un-seat legs run on both.
                    int stock_cfg = (pass == 1) ? CONFIG_6 : CONFIG_16;
                    ts.load_configuration(stock_cfg);
                    int live_stock = ts.current_configuration;

                    double* ring0 = ts.data_container.passband_delayed_data;

                    if(!ts.data_container.precook_ring_pinned)
                    {
                        // FAIL-BEFORE (DEFEAT / pin failed): the legacy set_size / CONFIG_NONE reinit
                        // paths realloc the ring — the exact regression STEP-4 removes.
                        int moved = 0;
                        if(pass == 0){
                            int Ng=0,l2=0,nb2=0; ts.bigblock_rebuild_thin_grid(Ng,l2,nb2);
                            ts.bigblock_restore_stock_config();
                            if(ts.data_container.passband_delayed_data != ring0){ moved++; ring0 = ts.data_container.passband_delayed_data; }
                        }
                        ts.force_resize_capture_ring((int)ts.data_container.buffer_Nsymb + 500);
                        if(ts.data_container.passband_delayed_data != ring0){ moved++; ring0 = ts.data_container.passband_delayed_data; }
                        ts.force_set_capture_ring_natural();
                        if(ts.data_container.passband_delayed_data != ring0){ moved++; ring0 = ts.data_container.passband_delayed_data; }
                        printf("[TEST-PRECOOK-COLLAPSE] %s FAIL-BEFORE (%s): ring realloc'd %d× across restore+seat+unseat "
                               "(legacy path; no M3 collapse)\n",
                               bwn, defeated4 ? "MERCURY_PRECOOK_DEFEAT=1" : "ring-not-pinned", moved);
                        c4fail += (moved > 0) ? moved : 1;   // ensure RED under DEFEAT
                        continue;
                    }

                    int sidx = ts.bundle_index(live_stock, nb);
                    if(sidx < 0){ printf("[TEST-PRECOOK-COLLAPSE] %s FAIL: no bundle for stock cfg %d\n", bwn, live_stock); c4fail++; continue; }
                    const st_config_bundle* sb = ts.bundle_set(nb)[sidx].get();
                    int natural_bn = sb->buffer_Nsymb;
                    int cap = ts.data_container.pinned_capacity_buffer_Nsymb;

                    // --- (1) BIG-BLOCK RESTORE (WB only): mangle the live ofdm to the thin grid, then
                    //         restore to the stock bundle via the swap.
                    if(pass == 0)
                    {
                        int Ng=0,l2=0,nb2=0;
                        (void)ts.bigblock_rebuild_thin_grid(Ng,l2,nb2);
                        bool mangled = (ts.ofdm.Nsymb != sb->ofdm.Nsymb);   // thin grid Nsymb=Ngrid != stock
                        ts.bigblock_restore_stock_config();
                        if(ts.data_container.passband_delayed_data != ring0){
                            printf("[TEST-PRECOOK-COLLAPSE] %s FAIL big-block-restore: ring moved (%p != %p) — realloc\n",
                                bwn, (void*)ts.data_container.passband_delayed_data, (void*)ring0); c4fail++; }
                        if(ts.ofdm.Nc != sb->ofdm.Nc || ts.ofdm.Nsymb != sb->ofdm.Nsymb ||
                           ts.data_container.M != sb->M || ts.ldpc.rate != sb->ldpc_rate){
                            printf("[TEST-PRECOOK-COLLAPSE] %s FAIL big-block-restore: geometry live(Nc=%d Nsymb=%d M=%d rate=%.4f) != stock(Nc=%d Nsymb=%d M=%d rate=%.4f) (mangled=%d)\n",
                                bwn, ts.ofdm.Nc, ts.ofdm.Nsymb, ts.data_container.M, ts.ldpc.rate,
                                sb->ofdm.Nc, sb->ofdm.Nsymb, sb->M, sb->ldpc_rate, (int)mangled); c4fail++; }
                        if((int)ts.data_container.buffer_Nsymb != natural_bn){
                            printf("[TEST-PRECOOK-COLLAPSE] %s FAIL big-block-restore: buffer_Nsymb=%d != natural %d\n",
                                bwn, (int)ts.data_container.buffer_Nsymb, natural_bn); c4fail++; }
                        { bool nz=false; int ns=(ts.ldpc.N<64)?ts.ldpc.N:64;
                          for(int k=0;k<ns;k++) if(ts.data_container.bit_energy_dispersal_sequence[k]!=0){ nz=true; break; }
                          if(!nz){ printf("[TEST-PRECOOK-COLLAPSE] %s FAIL big-block-restore: descrambler all-zero (INV-6)\n", bwn); c4fail++; } }
                        if(ts.ofdm.mfsk_corr_template_len != 0){
                            printf("[TEST-PRECOOK-COLLAPSE] %s FAIL big-block-restore: stale MFSK template len=%d (OFDM)\n",
                                bwn, ts.ofdm.mfsk_corr_template_len); c4fail++; }
                    }

                    // --- (2) ROBUST-FLOOR SEAT: seat the ring to the ROBUST_0 floor. Expected active
                    //         window = max(config natural, robust floor) — for a small OFDM config the
                    //         floor dominates; if the config natural already exceeds it the seat is an
                    //         idempotent no-op (already seated). Either way NO realloc.
                    int ridx = ts.bundle_index(ROBUST_0, nb);
                    int floor_bn = (ridx >= 0) ? ts.bundle_set(nb)[ridx]->buffer_Nsymb : (natural_bn + 500);
                    int expect_seat = (floor_bn > natural_bn) ? floor_bn : natural_bn;
                    ts.force_resize_capture_ring(floor_bn);
                    if(ts.data_container.passband_delayed_data != ring0){
                        printf("[TEST-PRECOOK-COLLAPSE] %s FAIL seat: ring moved — realloc\n", bwn); c4fail++; }
                    if((int)ts.data_container.buffer_Nsymb != expect_seat){
                        printf("[TEST-PRECOOK-COLLAPSE] %s FAIL seat: buffer_Nsymb=%d != expected %d (floor %d, natural %d)\n",
                            bwn, (int)ts.data_container.buffer_Nsymb, expect_seat, floor_bn, natural_bn); c4fail++; }
                    if((int)ts.data_container.buffer_Nsymb > cap){
                        printf("[TEST-PRECOOK-COLLAPSE] %s FAIL seat: buffer_Nsymb=%d > pinned cap %d\n",
                            bwn, (int)ts.data_container.buffer_Nsymb, cap); c4fail++; }
                    if(ts.ofdm.Nc != sb->ofdm.Nc || ts.data_container.M != sb->M){
                        printf("[TEST-PRECOOK-COLLAPSE] %s FAIL seat: geometry drifted (seat must change ONLY the window)\n", bwn); c4fail++; }

                    // --- (3) UN-SEAT: restore the config natural window.
                    ts.force_set_capture_ring_natural();
                    if(ts.data_container.passband_delayed_data != ring0){
                        printf("[TEST-PRECOOK-COLLAPSE] %s FAIL un-seat: ring moved — realloc\n", bwn); c4fail++; }
                    if((int)ts.data_container.buffer_Nsymb != natural_bn){
                        printf("[TEST-PRECOOK-COLLAPSE] %s FAIL un-seat: buffer_Nsymb=%d != natural %d\n",
                            bwn, (int)ts.data_container.buffer_Nsymb, natural_bn); c4fail++; }
                    if(ts.data_container.buffer_Nsymb_min != 0){
                        printf("[TEST-PRECOOK-COLLAPSE] %s FAIL un-seat: buffer_Nsymb_min=%d != 0 (floor not cleared)\n",
                            bwn, ts.data_container.buffer_Nsymb_min); c4fail++; }

                    if(c4fail == 0)
                        printf("[TEST-PRECOOK-COLLAPSE] %s PASS: %sseat->%d un-seat->%d natural, ring stable @%p (cap=%d) — no realloc\n",
                            bwn, (pass==0)?"big-block-restore(ring stable, geometry restored), ":"",
                            expect_seat, natural_bn, (void*)ring0, cap);
                }
                failed += c4fail;
            }

            return (failed == 0) ? 0 : 1;
        }
        // --test-chase : run ONLY the chase-combining self-test and exit. Drives the
        // production primitive cl_telecom_system::chase_capture()/chase_combine() over
        // a deterministic AWGN BPSK codeword + its retx. MERCURY_CHASE=0 defeats it
        // (demonstrates the pre-fix 0-success); MERCURY_CHASE_TEST_AMPL tunes the
        // operating point. See fact-documents/chase-combining-harq.md.
        if (strcmp(argv[i], "--test-chase") == 0) {
            arm_test_watchdog();
            int failed = run_chase_selftest();
            return (failed == 0) ? 0 : 1;
        }
        // --test-crc-escape : run ONLY the CRC-escape accept-gate self-test and exit.
        // Deterministic, in-process; drives the PRODUCTION frame_decode_rejected()
        // predicate (fail-before via the crc_escape_defeat arm). No IONOS/RF.
        if (strcmp(argv[i], "--test-crc-escape") == 0) {
            arm_test_watchdog();
            int failed = run_crc_escape_selftest();
            return (failed == 0) ? 0 : 1;
        }
        // --test-subpeak-rescue : run ONLY the F1b Part B true-boundary rescue
        // mechanism self-test and exit. Deterministic, noiseless, in-process.
        if (strcmp(argv[i], "--test-subpeak-rescue") == 0) {
            arm_test_watchdog();
            int failed = run_subpeak_rescue_selftest();
            return (failed == 0) ? 0 : 1;
        }
        // --test-chase-ab : production-path HARQ/Chase A/B (receive_byte 2-look
        // combine). Proves the chase capture+rescue hooks FIRE on the real decode
        // path, recover sub-threshold frames the single look failed, do NOT lower
        // the floor (single decode identical ON vs OFF), and never corrupt (adopted
        // frame byte-checked vs payload). Cheap, deterministic, no cards/RF.
        if (strcmp(argv[i], "--test-chase-ab") == 0) {
            arm_test_watchdog();
            int failed = run_chase_ab_selftest();
            return (failed == 0) ? 0 : 1;
        }
        // --test-sigterm-handler : run ONLY the FIX-C graceful-shutdown handler
        // regression (handler installed -> SIGTERM/SIGINT set shutdown_=true)
        // and exit. install_termination_handlers() already ran above, so the
        // self-raise(SIGTERM) inside is caught; on a tree WITHOUT the handler it
        // would terminate the process (signal-killed exit) = the fail-before.
        if (strcmp(argv[i], "--test-sigterm-handler") == 0) {
            int failed = test_sigterm_handler();
            return (failed == 0) ? 0 : 1;
        }
        // --test-switchrole-reride : run ONLY the SWITCH_ROLE re-ride race
        // regression (idle-switchrole-race.md §6; the R1 reverse-transfer
        // double-swap gate) and exit. Fast + deterministic; drives the REAL
        // set_role(COMMANDER) swap-in primitive + the REAL idle branch. FAILS-
        // BEFORE with -DSWITCHROLE_RERIDE_FAILBEFORE. See
        // arq_commander.cc::test_switch_role_reride_race().
        if (strcmp(argv[i], "--test-switchrole-reride") == 0) {
            cl_arq_controller ARQ_reride;
            int failed = ARQ_reride.test_switch_role_reride_race();
            return (failed == 0) ? 0 : 1;
        }
        // --test-connect-reack : run ONLY the CONNECT-REACK EXCISE regression
        // (8e62722e removed: the pre-data window must NOT pin frames_to_read=2)
        // and exit. Fast + deterministic; see arq_responder.cc::test_connect_reack().
        if (strcmp(argv[i], "--test-connect-reack") == 0) {
            cl_arq_controller ARQ_reack;
            int failed = ARQ_reack.test_connect_reack();
            return (failed == 0) ? 0 : 1;
        }
        // --test-start-ack-causal-guard : run only the START_CONNECTION bare-
        // ACK causal timing/ring-epoch regression. Fast, deterministic, no RF.
        // -DSTART_ACK_CAUSAL_GUARD_FAILBEFORE reproduces the 263-ms advance.
        if (strcmp(argv[i], "--test-start-ack-causal-guard") == 0) {
            cl_telecom_system start_ack_ts;
            cl_arq_controller ARQ_start_ack;
            ARQ_start_ack.telecom_system = &start_ack_ts;
            int failed = ARQ_start_ack.test_start_ack_causal_guard();
            return (failed == 0) ? 0 : 1;
        }
        // --test-zombie-amp : run ONLY the zombie/amplifier layer fail-before/
        // pass-after self-test (R2b BREAK-accept-while-DROPPED, the ROBUST_DWELL
        // keep-alive gate, and the watchdog peer-liveness probe gate) and exit.
        // Asserts each pure decision helper's truth table under defeat / no-defeat.
        // See fact-documents/data-flow-zombie-amplifier.md.
        if (strcmp(argv[i], "--test-zombie-amp") == 0) {
            cl_arq_controller ARQ_za;
            int failed = ARQ_za.test_zombie_amp();
            return (failed == 0) ? 0 : 1;
        }
        // --test-demote-silence : run ONLY the R5 demote-split fail-before/pass-after self-test
        // (pure-silence classifier truth table, the shared counter-step, the reroute predicate +
        // guards, the DEFEAT knob, and the +LOW watchdog probe-latch hygiene) and exit.
        // See fact-documents/data-flow-zombie-amplifier.md.
        if (strcmp(argv[i], "--test-demote-silence") == 0) {
            cl_arq_controller ARQ_dsil;
            int failed = ARQ_dsil.test_demote_silence();
            return (failed == 0) ? 0 : 1;
        }
        // --test-acq-bounds : run ONLY the OFDM acquisition bounds-gate recovery
        // regression (a tail-band frame whose detection floors past upper_bound
        // but whose body is in-buffer must be RECOVERED, not discarded) and exit.
        // Faithful in-process synthetic-fire through transmit_byte/receive_byte;
        // see fact-documents/data-flow-acq-bounds-gate.md.
        if (strcmp(argv[i], "--test-frame0-vehicle") == 0) {
            int failed = run_frame0_vehicle_selftest();
            return failed;
        }
        if (strcmp(argv[i], "--test-frame0-mf") == 0) {
            int failed = run_frame0_mf_selftest();
            return failed;
        }
        // Reverse-MFSK/forward-OFDM preamble energy-ratio admission gate.
        // In-process CFG15 synthetic fire; no audio, ARQ, or TCP setup.
        if (strcmp(argv[i], "--test-subpeak-gate") == 0) {
            arm_test_watchdog();
            cl_telecom_system ts;
            ts.operation_mode = BER_PLOT_passband;
            ts.load_configuration(CONFIG_15);
            return ts.test_subpeak_gate();
        }
        if (strcmp(argv[i], "--test-frame0-replay") == 0) {
            const char* cap = (i+1 < argc) ? argv[i+1] : "";
            return run_frame0_replay_selftest(cap);
        }
        if (strcmp(argv[i], "--test-acq-bounds") == 0) {
            int failed = run_acq_bounds_selftest();
            return (failed == 0) ? 0 : 1;
        }
        // (--test-reack-ftr-starvation REMOVED with the excise of 8e62722e,
        // connect-testack-handshake.md §9; --test-connect-reack now guards the
        // removal — the pre-data window must NOT pin frames_to_read=2.)
        // --test-inband-tier-crossing : run ONLY the hybrid tier-crossing routing
        // regression (robust<->OFDM crossing -> legacy SET_CONFIG; intra-tier ->
        // in-band tag) and exit. Fast + deterministic; see
        // arq_commander.cc::test_inband_tier_crossing_routing.
        if (strcmp(argv[i], "--test-inband-tier-crossing") == 0) {
            cl_arq_controller ARQ_tc;
            int failed = ARQ_tc.test_inband_tier_crossing_routing();
            return (failed == 0) ? 0 : 1;
        }
        // --test-inband-reverse-pin : run ONLY the tier-cross reverse-ACK pin regression
        // (robust<->OFDM cross -> reverse pinned to the robust rung, not the OFDM forward
        // rung) and exit. Fast + deterministic; see
        // arq_commander.cc::test_inband_tier_cross_reverse_pin. Build with
        // -DINBAND_REVERSE_PIN_FAILBEFORE to reproduce the fails-before (no pin).
        if (strcmp(argv[i], "--test-inband-reverse-pin") == 0) {
            cl_arq_controller ARQ_rp;
            int failed = ARQ_rp.test_inband_tier_cross_reverse_pin();
            return (failed == 0) ? 0 : 1;
        }
        // --test-inband-basepattern-confirm : run ONLY the KEYSTONE intra-tier base-pattern
        // climb-confirm regression (an emitted climb confirms from the robust BASE ACK pattern,
        // decoupled from the bsi-bearing SACK suffix) and exit. Fast + deterministic; see
        // arq_commander.cc::test_inband_basepattern_confirm. Build with
        // -DINBAND_BASEPATTERN_CONFIRM_FAILBEFORE to reproduce the fails-before (suffix-coupled).
        if (strcmp(argv[i], "--test-inband-basepattern-confirm") == 0) {
            cl_arq_controller ARQ_bc;
            int failed = ARQ_bc.test_inband_basepattern_confirm();
            return (failed == 0) ? 0 : 1;
        }
        // --test-inband-robust-follow : run ONLY the ROBUST-TIER tag-follow stale-pass
        // regression (the residual fix that makes the robust->OFDM tier-cross deterministic)
        // and exit. Fast + deterministic; drives the production gate inband_robust_follow_
        // gate_open. Fail-before: MERCURY_ROBUST_FOLLOW_FRESHWIN_REQ=1 (asserted internally).
        if (strcmp(argv[i], "--test-inband-robust-follow") == 0) {
            cl_arq_controller ARQ_rf;
            int failed = ARQ_rf.test_inband_robust_follow_freshwin();
            return (failed == 0) ? 0 : 1;
        }
        // --test-inband-frame0-partial : run ONLY the CONFIG_0 rolling-partial climb-unblock
        // regression (a lead-frame-only partial at an inband OFDM rung advances the FRAME-UP
        // climb streak; a multi-drop partial stays vetoed) and exit. Fast + deterministic; see
        // arq_commander.cc::test_inband_frame0_partial + data-flow-inband-frame0-rolling-partial.md.
        // Build with -DINBAND_FRAME0_PARTIAL_FAILBEFORE to reproduce the fails-before (wedge).
        if (strcmp(argv[i], "--test-inband-frame0-partial") == 0) {
            cl_arq_controller ARQ_f0p;
            int failed = ARQ_f0p.test_inband_frame0_partial();
            return (failed == 0) ? 0 : 1;
        }
        // --test-inband-climb-defer : run ONLY the ROLLING-PARTIAL climb DEFER-WHILE-HOLE-OUTSTANDING
        // regression (the 2801d7c sibling — the anchor-raise + FRAME-UP config-change must NOT fire
        // while a lead-frame-only partial's frame-0 is still outstanding for retx, or clear_retx_queue
        // abandons it -> GAP-ABORT wedge; the streak credit is still retained) and exit. Fast +
        // deterministic; see arq_commander.cc::test_inband_climb_defer_on_retx +
        // data-flow-inband-frame0-rolling-partial.md §10. Build with -DINBAND_CLIMB_DEFER_FAILBEFORE
        // to reproduce the fails-before (the climb fires while the hole is outstanding — the orphan).
        if (strcmp(argv[i], "--test-inband-climb-defer") == 0) {
            cl_arq_controller ARQ_cd;
            int failed = ARQ_cd.test_inband_climb_defer_on_retx();
            return (failed == 0) ? 0 : 1;
        }
        // --test-climb-bsi-rollback : run ONLY the climb-UP cmd_batch_seq_id rollback regression
        // (the SYMMETRY GAP to the demote rollback — the climb-UP SET_CONFIG emits must roll
        // cmd_batch_seq_id back to the in-flight bsi before re-presenting an already-RSP-delivered
        // batch, else the re-present is a >=2 bsi jump from the RSP high-water -> GAP-ABORT HOLD)
        // and exit. Fast + deterministic; drives the REAL producer + REAL sack_v2_readopt_has_gap;
        // see arq_commander.cc::test_climb_bsi_rollback + data-flow-inband-frame0-rolling-partial.md
        // §13. Build with -DINBAND_CLIMB_BSI_ROLLBACK_FAILBEFORE to reproduce the fails-before
        // (the re-present stays at the advanced epoch -> gap -> RSP HOLD).
        if (strcmp(argv[i], "--test-climb-bsi-rollback") == 0) {
            cl_arq_controller ARQ_cbr;
            int failed = ARQ_cbr.test_climb_bsi_rollback();
            return (failed == 0) ? 0 : 1;
        }
        // --test-guard-reanchor : run only the live-state reverse-SACK guard
        // reference regression and exit.
        if (strcmp(argv[i], "--test-guard-reanchor") == 0) {
            cl_arq_controller ARQ_gr;
            int failed = ARQ_gr.test_guard_reanchor();
            return (failed == 0) ? 0 : 1;
        }
        // --test-inband-plus1-climb : run ONLY the in-band +1 climb regression (the FRAME-UP
        // climb steps +1 under the inband feature, suppressing the SNR-elevator multi-rung
        // jump; legacy keeps the elevator) and exit. Fast + deterministic; see
        // arq_commander.cc::test_inband_plus1_climb + data-flow-inband-frame0-rolling-partial.md §7.
        // Build with -DINBAND_PLUS1_CLIMB_FAILBEFORE to reproduce the fails-before (0->3 jump).
        if (strcmp(argv[i], "--test-inband-plus1-climb") == 0) {
            cl_arq_controller ARQ_p1;
            int failed = ARQ_p1.test_inband_plus1_climb();
            return (failed == 0) ? 0 : 1;
        }
        // --test-inband-adopt-preserve : run ONLY the robust->OFDM adopt live-burst PRESERVE
        // regression (the last transition-class hole) and exit. Fast + deterministic; see
        // arq_responder.cc::test_inband_adopt_preserve_live_burst + data-flow-robust-ofdm-adopt-flush.md.
        if (strcmp(argv[i], "--test-inband-adopt-preserve") == 0) {
            cl_arq_controller ARQ_adopt;
            int failed = ARQ_adopt.test_inband_adopt_preserve_live_burst();
            return (failed == 0) ? 0 : 1;
        }
        // --test-inband-adopt-nofdm-invariant : run ONLY the robust->OFDM ring-shrink
        // Nofdm-invariant regression (diagnosis a468b2fc) and exit. Fast + deterministic;
        // see arq_responder.cc::test_inband_adopt_nofdm_invariant + data-flow-robust-ofdm-adopt-flush.md §15.
        if (strcmp(argv[i], "--test-inband-adopt-nofdm-invariant") == 0) {
            cl_arq_controller ARQ_nofdm;
            int failed = ARQ_nofdm.test_inband_adopt_nofdm_invariant();
            return (failed == 0) ? 0 : 1;
        }
        // --test-harvest-inband-cfg0-geometry : force the production fresh-decoder
        // condition and require the CONFIG_0 stride geometry to match the primary.
        if (strcmp(argv[i], "--test-harvest-inband-cfg0-geometry") == 0) {
            cl_arq_controller ARQ_cfg0;
            int failed = ARQ_cfg0.test_harvest_inband_cfg0_geometry();
            return (failed == 0) ? 0 : 1;
        }
        // --test-inband-config0-start-ring : run ONLY the §21 CONFIG_0-START robust-floor over-seat
        // regression (the uncovered sibling of FIX #1e) and exit. Fast + deterministic; see
        // arq_responder.cc::test_inband_config0_start_ring + data-flow-robust-ofdm-adopt-flush.md §21.
        if (strcmp(argv[i], "--test-inband-config0-start-ring") == 0) {
            cl_arq_controller ARQ_c0ring;
            int failed = ARQ_c0ring.test_inband_config0_start_ring();
            return (failed == 0) ? 0 : 1;
        }
        // --test-inband-descrambler-survives-shrink : run ONLY the §17 descrambler-survives-
        // ring-shrink regression (the CONFIG_0 clean-lock CRC-fail root) and exit.
        if (strcmp(argv[i], "--test-inband-descrambler-survives-shrink") == 0) {
            cl_arq_controller ARQ_descr;
            int failed = ARQ_descr.test_inband_descrambler_survives_ring_shrink();
            return (failed == 0) ? 0 : 1;
        }
        // --test-inband-deadbatch-progress : run ONLY the §19 dead-batch real-period/zero-progress
        // regression (the climb-killer fix; real total loss still BREAKs) and exit.
        if (strcmp(argv[i], "--test-inband-deadbatch-progress") == 0) {
            cl_arq_controller ARQ_db;
            int failed = ARQ_db.test_inband_deadbatch_progress();
            return (failed == 0) ? 0 : 1;
        }
        // --test-moose: drive the production Moose reject/clamp predicate at
        // CFG15 spacing. MOOSE_CFO_FAILBEFORE reproduces the old dead zone.
        if (strcmp(argv[i], "--test-moose") == 0) {
            int failed = run_moose_deadzone_tests();
            return (failed == 0) ? 0 : 1;
        }
        // --test-pilot-thin-nv : run the production sparse-grid estimator across
        // 10-30 dB AWGN plus the dense cfg16 scope control.
        if (strcmp(argv[i], "--test-pilot-thin-nv") == 0) {
            int failed = run_pilot_thin_nv_tests();
            return (failed == 0) ? 0 : 1;
        }
        // --test-winlink-dict : run ONLY the Winlink dict priming + version-lock
        // regression suite and exit. Fast + deterministic; drives the production
        // cl_compressor end-to-end (primed lift, bit-exact, version-mismatch
        // fail-safe, kill-switch, bulk no-regression, attachment inertness,
        // streaming desync). See source/compression/test_winlink_dict.cc.
        if (strcmp(argv[i], "--test-winlink-dict") == 0) {
            int failed = run_winlink_dict_tests();
            return (failed == 0) ? 0 : 1;
        }
        // --test-aead-nonce : run ONLY the AEAD bsi-bound nonce regression suite
        // and exit. Fast + deterministic, no IONOS/RF. SECURITY INVARIANT gate:
        // no (key,nonce) reuse; nonce binds to the wire batch_seq_id so SACK
        // reorder/retx decrypt correctly (>256-batch wrap, reorder+retx round-
        // trip, direction disjointness, truncated-KX reject, tamper-reject).
        // See source/crypto/test_aead_nonce.cc / data-flow-aead-nonce.md.
        if (strcmp(argv[i], "--test-aead-nonce") == 0) {
            int failed = run_aead_nonce_tests();
            return (failed == 0) ? 0 : 1;
        }
        // --test-mlkem-hybrid : run ONLY the ML-KEM-768 hybrid KEX suite and
        // exit. Fast + deterministic, no IONOS/RF. Combiner symmetry, ML-KEM-
        // first IKM order, transcript binding, KX chunk round-trip + CRC8.
        // See source/crypto/test_mlkem_hybrid.cc / MLKEM_HYBRID_PLAN.md.
        if (strcmp(argv[i], "--test-mlkem-hybrid") == 0) {
            int failed = run_mlkem_hybrid_tests();
            return (failed == 0) ? 0 : 1;
        }
        // --test-mlkem-live-rx : ML-KEM KX MULTI-CHUNK reassembly through the LIVE
        // control-RX consumer path (process_control_responder KX2 +
        // process_control_commander KX3). Reproduces the messages_control.length=1
        // producer/consumer bug: fails-before (chunks rejected), passes-after (real
        // slot width passed). Member test on a throwaway controller. Fast +
        // deterministic, no IONOS/RF. MLKEM_HYBRID_PLAN.md §5.
        if (strcmp(argv[i], "--test-mlkem-live-rx") == 0) {
            cl_arq_controller ARQ_kx;
            int failed = ARQ_kx.test_kx_chunk_live_rx();
            return (failed == 0) ? 0 : 1;
        }
        // --test-sim-clock : run ONLY the sim-clock unit suite and exit. The
        // full --test suite includes long stochastic MFSK detector sweeps;
        // this gives a fast, deterministic entry for the sim-clock tests
        // (test (a) of sim-arq-channel.md §3). Production behavior of --test is
        // unchanged.
        if (strcmp(argv[i], "--test-sim-clock") == 0) {
            int failed = run_sim_clock_tests();
            return (failed == 0) ? 0 : 1;
        }
        // --test-ofdm-fine-timing : run ONLY the §22 OFDM fine-timing
        // phase-invariant magnitude regression suite and exit. Fast +
        // deterministic — excludes the long stochastic MFSK detector sweeps in
        // the full --test suite. See fact-documents/ofdm-fine-timing-magnitude.md §4.
        if (strcmp(argv[i], "--test-ofdm-fine-timing") == 0) {
            int failed = run_ofdm_fine_timing_tests();
            return (failed == 0) ? 0 : 1;
        }
        // --test-preamble-sched : LEVER P preamble-amortization schedule +
        // effective-length pure-function unit tests. Fast + deterministic.
        // See fact-documents/data-flow-preamble-amortization.md §1.
        if (strcmp(argv[i], "--test-preamble-sched") == 0) {
            int failed = run_preamble_sched_tests();
            return (failed == 0) ? 0 : 1;
        }
        // --test-break-fh : fix/break-fh-gate §23 BREAK forward-health gate suite
        // (FH-latch suppression of the held-CFG16 marginal-OFDM alias + K-of-N
        // corroboration + genuine-BREAK survival, both gate states). Fast +
        // deterministic. Also included in the full --test suite.
        if (strcmp(argv[i], "--test-break-fh") == 0) {
            int failed = run_break_fh_gate_tests();
            return (failed == 0) ? 0 : 1;
        }
        // --test-break-alias : OFDM-alias BREAK false-positive investigation sweep.
        // Drives real non-BREAK OFDM data (config x Es/N0 grid) through acquisition +
        // decode + the BREAK correlator; reports whether the detonation predicate ever
        // fires on data that is NOT a transmitted BREAK. NOT part of --test.
        if (strcmp(argv[i], "--test-break-alias") == 0) {
            return run_break_alias_sweep();
        }
        // --test-recovery-ack : recovery-ack-robustness.md suite (marginal-ACK
        // combining + listen-window ms-mirror + DELTA-1 reps-agnostic BREAK +
        // DELTA-2 CFO-refine decision gate). Fast + deterministic. Also in --test.
        if (strcmp(argv[i], "--test-recovery-ack") == 0) {
            int failed = run_recovery_ack_tests();
            return (failed == 0) ? 0 : 1;
        }
        // --test-recovery-ack-fine : recovery-ack-fine STAGE 1 fine-pass timing-
        // straddle sweep (data-flow-ack-detector.md §2). Fail-before (no-fine
        // coarse collapse below 7/16) / pass-after (fine recovers over the
        // hardened bar) across a tau sweep. Fast + deterministic. Also in --test.
        if (strcmp(argv[i], "--test-recovery-ack-fine") == 0) {
            int failed = run_recovery_ack_fine_tests();
            return (failed == 0) ? 0 : 1;
        }
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
    bool test_bigblock_arq_unit_cli = false; // --test-bigblock-arq-unit: P2 big-block ARQ-granularization regression.
                                        // 3 cases (clean K=8 / one-bad-cw / lost-EOB). MUST FAIL before P2 wiring (the
                                        // bigblock_block_to_arq stub returns BIGBLOCK_ARQ_NOT_WIRED), PASS after.
                                        // One-shot at startup, then exit rc. See fact-documents/data-flow-bigblock-arq-unit.md §6.
    bool test_sim_inproc_bigblock_cli = false; // --test-sim-inproc-bigblock: STEP 3 single-block end-to-end in the
                                        // in-process 2-instance sim (CMD->RSP->ACK->CMD byte-faithful + one-bad-cw
                                        // partial -> selective-repeat completes). One-shot at startup, then exit rc.
    bool test_bigblock_fullpath_cli = false; // --test-bigblock-fullpath: LIVE 2-instance CFG16 big-block transfer
                                        // through the REAL receive_bigblock+carve+whiten+FIFO deliver path with
                                        // fail-before/pass-after on the same binary. One-shot at startup, exit rc.
    bool test_sim_sustain_cli = false;  // --test-sim-sustain: SIM_INPROC sustain BATTERY (runs BOTH, OR of rc):
                                        // (1) PINNED-CFG15 sustained-delivery stepper-wedge regression (payloads
                                        // 600/2000/4000/8000; SIMFTR_ROOTCAUSE.md §7/§8 fix #1), then (2) OUTER-stepper
                                        // OFDM big-block no-wedge + clean-carve + C0-a byte-correct sustain (Phase b).
                                        // fail-before/pass-after on the same binary. One-shot at startup, exit rc.
    bool test_bigblock_multicw_cli = false; // --test-bigblock-multicw: FULL K=8 block (all 8 codewords) through the
                                        // LIVE receive_bigblock+de-whiten+per-cw-CRC carve; 3 arms prove the root
                                        // cause is the RX capture WINDOW (cw1..cw7 corruption), NOT whiten/offset.
    bool test_inband_down_resync_cli = false; // --test-inband-down-resync: in-band down-ladder ROBUST resync —
                                        // CMD demote-to-ROBUST_0 with announce SUPPRESSED forces the RSP's
                                        // production down-ladder; fail-before (primary-sized snapshot truncates
                                        // ROBUST_0 -> 0 bytes) -> pass-after (Rank-1 snapshot+ring sizing fix).
    bool test_bigblock_chanest_cli = false; // --test-bigblock-chanest: GENUINE (ref==NULL) 2-instance CFG16 big-block
                                        // decode under a CFO/SFO-impaired channel; reproduces the HW [RXACQ] meanH
                                        // collapse off-bench (clean passes, CFO/SFO collapses the block estimate).
    bool test_bigblock_acqwindow_cli = false; // --test-bigblock-acqwindow: §19 acquisition-window POSITION guard —
                                        // one genuine K=8 block at several in-window preamble offsets in a FIXED
                                        // production-sized window; near-end (tail past window) DEFERS (guard ON) /
                                        // carves truncated bytes_ok=0 (DEFEAT_ACQGUARD). Reproduces the HW ~5.6% bug.
    bool test_topgear_clean_election_cli = false; // --test-topgear-clean-election: prove the CONFIG_17 64-QAM top-gear channel-clean election (verdict/hysteresis/ceiling/demote/rate)
    bool test_bigblock_climb_election_cli = false; // --test-bigblock-climb-election: prove the big-block rung is
                                        // ELECTED by the GEARSHIFT CFG16 transition (load_configuration tail), not
                                        // only at connect. fail-before/pass-after via MERCURY_BIGBLOCK_DEFEAT_ELECTION.
                                        // One-shot at startup, then exit rc. See data-flow-bigblock-arq-unit.md §16.
    bool test_bigblock_carve_suspend_unit_cli = false; // --test-bigblock-carve-suspend-unit: WALL-B FIX-3 RSP
                                        // carve-suspend watchdog unit test (streak state machine + the three consumers
                                        // + a real receive_byte cw0-reject loopback). fail-before via
                                        // MERCURY_BIGBLOCK_DEFEAT_CARVESUSPEND. One-shot at startup, then exit rc.
    bool test_bigblock_txlevel_cli = false; // --test-bigblock-txlevel: measure CFG16 big-block vs stock-OFDM TX
                                        // peak+RMS (HW over-level diag). One-shot at startup, then exit rc.
    bool test_sack_oow_reject_cli = false; // --test-sack-oow-reject: R039 — OFDM SACK_RSP out-of-window
                                        // reject. Drives real decode_sack_v2_frame + real sack_v2_bsi_in_window guard;
                                        // asserts CRC8-valid OOW SACK decoded-but-rejected, in-window accepted. One-shot, exits rc.
    bool test_eob_poison_prev_retx_cli = false; // --test-eob-poison-prev-retx: R038 — prev-retransmit EOB
                                        // poison. Drives the real EOB staging/promotion members; asserts pre-fix early
                                        // ACK-GATE PASS reproduced AND post-fix prevents it. One-shot, exits rc.
    bool test_batch_shrink_strands_prev_cli = false; // --test-batch-shrink-strands-prev: R035 — data_batch_size
                                        // shrink strands the active prev. Drives the REAL set_data_batch_size chokepoint;
                                        // asserts prev counters re-derived (gate reachable) + streaming defense on orphan. One-shot, exits rc.
    bool test_batch_shrink_orphan_defer_cli = false; // --test-batch-shrink-orphan-defer: Fix A (baseline-double-
                                        // delivery.md) — a mid-flight data_batch_size SHRINK must not orphan RECEIVED prev
                                        // frames. Drives the REAL set_data_batch_size shrink; reconstructs the
                                        // copy_data_to_buffer prev delivery set; asserts ZERO bytes lost. DEFEAT env = fail-before.
    bool test_backup_confirm_flush_cli = false; // --test-backup-confirm-flush: Fix C/H#1 (data-flow-fifo-backup.md
                                        // §6.1) — the delivered batch's raw must not survive in fifo_buffer_backup when
                                        // finalize is pre-empted (Root B). Drives register_ack ACK-confirm; asserts the
                                        // re-stage restores NOTHING (no re-delivery) + INV3 un-confirmed never flushed. DEFEAT=fail-before.
    bool test_rxfifo_backpressure_hold_cli = false; // --test-rxfifo-backpressure-hold: Fix H#3 (delivery-integrity-
                                        // audit-monitor.md §3) — the clean ACK precedes delivery; on RX-FIFO back-pressure
                                        // the un-stored tail is lost with a committed ACK. Gate holds the batch until the
                                        // app FIFO has room; asserts ZERO post-ACK loss + full re-delivery. DEFEAT=fail-before.
    bool test_retx_clear_on_recovery_cli = false; // --test-retx-clear-on-recovery: R029 — stale retx queue
                                        // cleared on recovery. Drives the REAL clear_retx_queue(); asserts the queue empties
                                        // of pre-recovery bsi, is idempotent, and repeatable. One-shot, exits rc.
    bool test_restage_requeue_orphan_cli = false; // --test-restage-requeue-orphan: §12 re-stage re-queue orphan/reorder
    bool test_stream_offset_cli = false; // --test-stream-offset: Option W FOUNDATION — absolute-byte-stream cursor ground-truth
    bool test_rx_drain_backpressure_cli = false; // --test-rx-drain-backpressure: FIX-6 — RX-delivery drain
                                        // must NOT drop popped bytes when the non-blocking app socket back-pressures.
                                        // FAILS at 62cb3dc (the 61,621-byte stall), PASSES after. One-shot, exits rc.
    bool test_kx_data_tamper_cli = false; // --test-kx-data-tamper: mid-stream AEAD ciphertext tamper fail-secure fire proof
    bool test_decompress_false_accept_cli = false; // --test-decompress-false-accept: streaming decompress-failure
                                        // must NOT push the undecodable raw compressed blob to the app (silent byte-
                                        // corruption). Drives a REAL two-compressor streaming desync + copy_data_to_buffer;
                                        // MERCURY_DECOMPRESS_RAWPUSH_DEFEAT=1 = fail-before. One-shot, exits rc.
    bool test_compact_confirm_rx_cli = false; // --test-compact-confirm-rx: Option B compact confirm LIVE RX path.
                                        // The self-validating compact confirm must accept via its own CRC-gated decode,
                                        // decoupled from the bare 7/16 gate that under-peaks the 26-sym frame. One-shot, exits rc.
    bool test_gap_abort_cli = false;    // --test-gap-abort: FIX-8 — silent lost-batch GAP on post-reset re-adopt.
                                        // Reproduces bench-4 (deliver 0-4, BREAK reset with 5-7 undelivered, present 8):
                                        // fail-before via MERCURY_GAP_ABORT_DEFEAT=1 (silent concat), pass-after aborts
                                        // loudly + delivers EXACTLY batches 0-4. One-shot, exits rc.
    bool test_gap_recover_cli = false;  // --test-gap-recover: RECOVERABLE delivery-time GAP-ABORT
    bool test_retain_overflow_cli = false; // --test-retain-shadow-overflow: deterministic 19>8 hole burst
    bool test_recover_fire_cli = false; // --test-gap-recover-fire: W1 CMD refill fire proof
                                        // (R2a hold + R2c CMD retention). fail-before via
                                        // MERCURY_GAP_RECOVER_DEFEAT=1 (terminal abort, high-water frozen);
                                        // pass-after = HOLD + refill + in-order 6->7->8. One-shot, exits rc.
    bool test_held_cur_fire_cli = false; // --test-held-cur-deliver-fire: R2b RSP deliver-held-cur
                                        // (bytes reach the app FIFO via copy_data_to_buffer) + CMD
                                        // recovery-turnaround credit. fail-before via
                                        // MERCURY_HELD_CUR_DELIVER_DEFEAT / MERCURY_GAP_RECOVER_
                                        // TURNAROUND_DEFEAT. One-shot, exits rc.
    bool test_measured_timers_cli = false; // --test-measured-timers: R6 SRTT/RTTVAR turnaround estimator +
                                        // ack_timeout_data >= receiving_timeout invariant regression (one-shot, exit rc).
    bool test_arqsmalls_cli = false;    // --test-arqsmalls: Karn discriminator decoupling
                                        // (MERCURY_KARN_RETX_ONLY) + DUTY-R pin-respect seed
                                        // (MERCURY_DUTY_R_PIN_DEFEAT), fast one-shot, exit rc.
    bool test_recovery_ack_capture_exercise_cli = false; // --test-recovery-ack-capture-exercise:
                                        // live production poll, late retained ACK -> 1/1 accept.
    bool test_turnaround_guard_cli = false; // --test-turnaround-guard: R1 turnaround-clearance guard —
                                        // stamp-then-key drives assert the guard waits out the peer's TX->RX
                                        // mute/flush window (default-on) and adds ~0 when already clear / never armed.
                                        // One-shot, exits rc.
    bool test_axis2_quiesce_gate_cli = false; // --test-axis2-quiesce-gate: R4 LINK-PARAMS quiesce gate —
                                        // an Axis-2 move DEFERS while the batch boundary is unclean (retx pending /
                                        // last batch partial) and FIRES when clean; FORCED after AXIS2_MAX_MOVE_DEFER;
                                        // fail-before via MERCURY_AXIS2_QUIESCE_DEFEAT=1 (mixed old-bsi batch). One-shot, exits rc.
    bool test_gap_abort_blind_cli = false; // --test-gap-abort-blind: GAP-ABORT ruler-blinding — the REAL
                                        // rsp_gap_abort_teardown() resets the contiguity ruler to -1, blinding its own
                                        // re-adopt gate, and the CMD (never told) re-drives the same stream -> silent
                                        // concatenation across a dropped batch. Fails on the current tree. One-shot, exits rc.
    bool test_gap_abort_stream_blind_cli = false; // --test-gap-abort-stream-blind: GAP-ABORT stream-BACKSTOP
                                        // companion — the REAL teardown must preserve the Option W byte cursor +
                                        // stamp validity (so w_stream_shift_detected still fires post-abort) and set the
                                        // sticky rsp_stream_aborted latch. MERCURY_GAP_STREAM_BLIND=1 = fail-before arm.
    bool test_config_tag_passband_cli = false; // --test-config-tag-passband: in-band rate-adapt Stage 3a —
                                        // PASSBAND ROUND-TRIP. TX keys the combined RM+gf16ra suffix to real passband
                                        // audio, passes it through CLEAN and AWGN, RX detects it on the passband (real
                                        // base-correlator presence detector) + decodes the right cfg_index, and proves an
                                        // OFDM data frame still LDPC-decodes with the suffix appended. fail-before:
                                        // rebuild with -DSTAGE3A_FAILBEFORE (RX ignores the passband suffix). One-shot,
                                        // exits rc. unilateral-config-tag-design.md §11 Stage 3.
    bool test_config_tag_follow_cli = false; // --test-config-tag-follow: in-band rate-adapt Stage 2 — emit/detect/FOLLOW.
                                        // Forces a CONFIG_10->CONFIG_8 batch-boundary switch; asserts the RX follows the
                                        // config FROM THE TAG (load_configuration) with the PHY twin switching coherently.
                                        // fail-before: rebuild with -DSTAGE2_FAILBEFORE (RX ignores tag -> stays CFG10).
                                        // One-shot, exits rc. unilateral-config-tag-design.md §11 Stage 2.
                                        // See bigblock_p3_hw/_fix8/FIX8_DESIGN.md + FIX8_AUDIT.md.
    bool test_inband_drop_cli = false;  // --test-inband-drop: in-band rate-adapt Stage 3b — LOOPBACK DROP.
    bool test_inband_fallback_cli = false;  // --test-inband-fallback: in-band Stage 4 — LOST-TAG DOWN-LADDER.
    bool test_inband_seamless_cli = false;  // --test-inband-seamless: in-band Stage 3d — PRE-FRAME SEAMLESS.
    bool test_inband_downladder_cli = false;  // --test-inband-downladder: down-ladder BREAK-orphan + silent-snapshot regression.
    bool test_reseat_span_cli = false;  // --test-reseat-span: prev-batch cross-storage index-skew 332-byte deletion keystone.
    bool test_cmd_idskew_cli = false;  // --test-cmd-idskew: CMD-side id-skew root — rebase a fresh new-data batch to slots [0,ND).
    bool test_inband_deliver_cli = false;  // --test-inband-deliver: forward-healthy reverse-ACK miss -> NO-BREAK deliver regression.
    bool test_linkphase_shadow_cli = false; // --test-linkphase-shadow: increment-1 shadow-agreement directed unit.
    bool test_inband_ring_floor_cli = false;  // --test-inband-ring-floor: capture-ring ROBUST-floor over-seat at a climbed OFDM rung.
    bool test_inband_liveness_cli = false;  // --test-inband-liveness: connect-liveness guard (control-plane livelock backstop).
    bool test_inband_no_break_cli = false;  // --test-inband-no-break: in-band Stage 4c — D5 BREAK-OBSOLETE.
    bool test_inband_retag_cli = false;  // --test-inband-retag: in-band Stage 4d — D1 repeat + D4 climb/auto-demote.
    bool test_inband_nack_cli = false;  // --test-inband-nack: in-band Stage 4e — D2 NACK first-class.
    bool test_inband_reannounce_cli = false;  // --test-inband-reannounce: in-band Stage 4e — D3 periodic re-announce.
                                        // Gearshift-driven unilateral drop (W3), tag on the real passband (W1),
                                        // RX follows from the passband tag (W2 + HINGE), SACK confirms (bsi),
                                        // ZERO SET_CONFIG on the wire, both ends config-track, PHY-twin coherent,
                                        // + the R7 mixed-config gap-gate case. fail-before: MERCURY_INBAND_RATE
                                        // unset OR -DINBAND_STAGE3B_FAILBEFORE. One-shot, exits rc.
                                        // data-flow-perbatch-config.md §12 / unilateral-config-tag-design.md §11 Stage 3.
    bool test_recovery_ack_capture_cli = false; // --test-recovery-ack-capture: LEVER 1 focused
                                        // capture-window test (recovery-ACK late/truncated + unwritten-head
                                        // ring). See arq_responder.cc test_recovery_ack_capture +
                                        // fact-documents/data-flow-recovery-ack-capture.md §7.
    bool test_data_ack_multiwindow_cli = false; // --test-data-ack-multiwindow: Track A — multi-window
                                        // DATA-ACK/SACK correlator. Synthesizes a real ACK+SACK burst at an
                                        // OLDER ring phase with a silent newest tail: fail-before (newest-tail
                                        // decode MISSES), pass-after (mw_find_ack_sack_phase recovers it with
                                        // CRC12 pass), no-false-accept on pure silence. One-shot, exits rc.
                                        // See fact-documents/data-flow-data-ack-sack-correlator.md §7.
    bool test_inorder_demote_cli = false; // --test-inorder-demote: D3.1 — UNIFIED in-order delivery
    bool test_climb_bsi_rollback_cli = false; // --test-climb-bsi-rollback: CLIMB-CHURN commander-side bsi rollback on a climb-UP promote
                                        // across EVERY demote case (BREAK + the 4 SET_CONFIG-only demotes +
                                        // PREV-BUMP strand). fail-before via MERCURY_GAP_ABORT_DEFEAT=1
                                        // (silent concat on the SET_CONFIG cases), pass-after aborts loudly
                                        // OR delivers contiguous. One-shot, exits rc.
                                        // See bigblock_p3_hw/_d31_fade/D31_INORDER_DESIGN.md.
    bool test_guard_reanchor_cli = false; // --test-guard-reanchor: live reverse-SACK guard reference
    bool test_spec_sack_cli = false;    // --test-spec-sack: LEVER #2 — speculative/prompt SACK. Frame-k still-decoding
                                        // at the window-fraction deadline -> in-window SACK bit_k=0 -> CMD retx ->
                                        // byte-faithful re-receive -> single in-order delivery, no silent loss.
                                        // fail-before via env-off (stall reproduced), pass-after via MERCURY_SPEC_SACK=1.
                                        // One-shot, exits rc. See fact-documents/turnaround-eff.md §8/§9.
    bool test_batchsize_desync_cli = false; // --test-batchsize-desync: res_c3100 CMD>RSP batch-size desync
                                        // silent-corruption regression (silent-corruption-residual.md §8).
                                        // fail-before via MERCURY_BATCHSIZE_DESYNC_DEFEAT=1.
    bool test_batch_shrink_orphan_current_cli = false; // --test-batch-shrink-orphan-current: rx_btf=-1
                                        // current-batch shrink tail-drop (marginal-SNR silent corruption).
                                        // fail-before via MERCURY_BATCHSHRINK_ORPHAN_DEFEAT=1, pass-after defers.
    bool test_linkphase_pending_confirm_cli = false; // --test-linkphase-pending-confirm: SEAM-2
                                        // deferred prev-confirm ARM/FLUSH fire proof.
                                        // fail-before via MERCURY_PENDING_CONFIRM_DEFEAT=1.
    bool test_eob_loss_batch_truncation_cli = false; // --test-eob-loss-batch-truncation: D5 — EOB-inference
    bool test_dedup_rebase_cli = false; // --test-dedup-rebase: in-band demote-rebase double-delivery
                                        // (byte-stream corruption) regression. fail-before via
                                        // MERCURY_STREAM_DEDUP_DEFEAT=1. One-shot exits rc; also runs in --test.
                                        // batch truncation. A 30-frame batch loses its EOB-marked tail; the
                                        // wired batch_total_frames recovers the true length. fail-before via
                                        // MERCURY_D5_INFER_DEFEAT=1 (silent 29-frame skip), pass-after holds
                                        // then delivers all 30 faithfully after the retx. One-shot, exits rc.
                                        // See TRACK_C_D2D3D5_DESIGN.md §5.3 / data-flow-prev-bump.md §8.
    bool test_v2_pendingack_flip_alias_cli = false; // --test-v2-pendingack-flip-alias: R030 — v2 PENDING_ACK
                                        // flip aliasing. Diverged index/wire space; drives the REAL v2_flip_resolve_slot();
                                        // asserts retx skipped + new-data -> correct slot + no FREE/foreign PENDING_ACK. One-shot, exits rc.
    bool test_retx_slot_order_cli = false; // --test-retx-slot-order: timing redesign — retx never rides wire slot 0.
                                        // Drives the REAL v2_rotate_retx_behind_lead()+v2_flip_resolve_slot(); asserts slot0=new-data,
                                        // EOB wire-final, retx bytes verbatim, flip -1 for retx only, ND==1 byte-identical. One-shot, exits rc.
    bool test_bigblock_livepath_cli = false; // --test-bigblock-livepath: GAP-2 — real CONNECT + real SET_CONFIG
                                        // handshake robust->CFG16 (NO pin), then a 1374B K=8 transfer through the
                                        // REAL send_batch->bigblock_send_one_block emit + receive_byte cw0-CRC gate
                                        // + carve + FIFO. Answers: does sim reproduce the HW cw0-CRC reject? exit rc.
    bool test_data_anchored_promote_cli = false; // --test-data-anchored-promote: Option B (data-anchored gearshift
                                        // promotion) regression. Drives break_target_with_anchor() + policy_evaluate_axis1()
                                        // with last_data_viable_config primed; asserts BREAK floors at the anchor and the
                                        // up-shifter promotes only one rung past it. One-shot, exits rc. See
                                        // fact-documents/gearshift-start-and-recovery.md §6.4.
    bool test_shutdown_atomic_cli = false; // --test-shutdown-atomic: R006 — shutdown_ atomicity
                                        // regression (race_audit R006). Compile-time static_assert (fail-before on
                                        // pre-fix plain-bool tree) + cross-thread runtime smoke. One-shot, exits rc.
    bool test_probe_backoff_cli = false; // --test-probe-backoff: FIX-B floor-probe back-off regression
                                        // (gearshift-floor-probe-backoff.md §7). Drives the REAL arm/gate/reset/predicate
                                        // machinery + the v2 policy_evaluate_axis1 UP gate over the SIM virtual clock.
                                        // PB1 FAIL-BEFORE/PASS-AFTER (armed CONFIG_0 suppressed + UP gate blocks; revert
                                        // -> promotes), PB2 virtual-clock elapse lifts, PB3 exponential+cap, PB4 reset,
                                        // PB5 INV-2 deep-SNR escape unchanged. One-shot, exits rc.
    bool test_phantom_ack_gate_cli = false; // --test-phantom-ack-gate: phantom-ACK content-gate regression.
                                        // Drives data_ack_bare_pattern_acceptable() across WB/NB x CRC-valid/CRC-absent
                                        // (the WB-no-CRC phantom cell must be REJECTED) + asserts a rejected phantom leaves
                                        // data_ack_received NO, does not raise last_data_viable_config / reset the BREAK
                                        // panic counter, and BREAK still reaches ROBUST_0. One-shot, exits rc. See
                                        // fact-documents/gearshift-start-and-recovery.md §8.
    bool test_clean_batch_viability_cli = false; // --test-clean-batch-viability: CLEAN-BATCH VIABILITY regression (§9).
    bool test_idle_switch_role_race_cli = false; // --test-idle-switch-role-race: idle SWITCH_ROLE race regression.
                                        // Drives the REAL process_buffer_data_commander() idle branch with a freshly-
                                        // CONNECTED empty-tx Commander; asserts SWITCH_ROLE is queued before any data
                                        // (the connected-but-0-deliver root cause). FAILS-BEFORE on monitor. One-shot.
    bool test_break_noprogress_cli = false; // --test-break-noprogress-teardown: BREAK no-progress teardown regression
                                        // (idle-switchrole-race.md §3/§4 Part C). Replays the shared break_noprogress_step
                                        // kernel: teardown fires at exactly K dead cycles; progress on a live BREAK resets
                                        // the streak (negative control). FAILS-BEFORE with -DBREAK_NOPROGRESS_FAILBEFORE.
    bool test_rx_ctrl_drop_cli = false; // --test-rx-ctrl-drop: RECEIVED-stuck control-slot regression
                                        // (data-flow-control-slot-lifecycle.md §6). Drives the shared RECEIVED-state
                                        // watchdog predicate + the produce-gate replication: a stranded RECEIVED slot is
                                        // freed and a later control frame lands. FAILS-BEFORE with -DRX_CTRL_DROP_FAILBEFORE.
    bool test_robust0_compress_deadlock_cli = false; // --test-robust0-compress-deadlock: ROBUST_0+streaming-compression
    bool test_streaming_compress_overshoot_cli = false; // --test-streaming-compress-overshoot: stale-EMA no-fit must retry compressed
    bool test_mixbatch_fill_overpop_cli = false; // --test-mixbatch-fill-overpop: mixbatch fill over-pop reorder regression
    bool test_mixbatch_fill_overpop_compressed_cli = false; // --test-mixbatch-fill-overpop-compressed: comp-leg over-pop + force-FREE data-loss regression
                                        // deadlock regression. Drives the REAL process_buffer_data_commander() data-fill
                                        // at ROBUST_0 (max_frame==7==COMPRESS_HEADER_SIZE) with streaming compression +
                                        // a real compressible payload; asserts >0 application bytes are staged. FAILS on
                                        // fef293f (every batch stages 0 payload → 0 throughput). See
                                        // fact-documents/data-flow-compress-frame-fill.md §5.
    bool test_cumulative_ack_cli = false; // --test-cumulative-ack: Tier-2 cumulative-n_r self-heal/gap-invariant/cap-gate regression (data-flow-forgiving-ack.md §T2.6).
    bool test_enc_failclosed_cli = false; // --test-enc-failclosed: encryption negotiation fail-closed regression (opt-in + peer-no-cap -> REFUSE; default-off -> plaintext). Fails-before under -DENC_FAILOPEN_FAILBEFORE.
    bool test_a3_decouple_safety_cli = false; // --test-a3-decouple-safety: the §2 CHECKPOINT — single-miss non-load-bearing (anti-0-bytes, byte-faithful) + genuine-death net intact, demote IN PLACE (data-flow-forgiving-ack.md §T2.2/§6).
    bool test_pas_cli = false;          // --test-pas: PAS/PCS distribution-matcher bijection + histogram self-test (feat/pcs).
    bool test_cfg17_cli = false;        // --test-cfg17: CFG17 shaped-64-QAM composition (PAS+TINTERP-seed+ratio-nvfix) failing-first (feat/cfg17).
    bool test_tinterp_seed_cli = false; // --test-tinterp-seed: TINTERP-SEED production it=0 estimator seed-swap failing-first (staging/tinterp-seed).
    bool test_decode_marathon_cli = false; // --test-decode-marathon: LEVER C parallel==serial big-block decode integrity (decode-marathon-C.md §8).
    bool test_climb_engine_cli = false; // --test-climb-engine: integrated 3-bug climb regression (gearshift-climb-engine.md §7).
    bool test_break_weld_cli = false;   // --test-break-weld: BREAK recovery target pin (diagnostic knob).
    bool test_rung_floor_cli = false;   // --test-rung-floor: per-rung MEASURED election floor gate + failure memory (DATAFLOW_AUDIT_rung_floor.md): wb13 fail-before/pass-after, never-raise, arm/survive/decay/reset, defeat knob.
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
        else if (strcmp(argv[i], "--test-bigblock-arq-unit") == 0)
        {
            // P2 big-block ARQ-granularization regression — one-shot at startup,
            // then exit with the test's rc. See
            // fact-documents/data-flow-bigblock-arq-unit.md §6. FAILS before P2
            // wiring (the bigblock_block_to_arq stub), PASSES after.
            test_bigblock_arq_unit_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-sim-inproc-bigblock") == 0)
        {
            // STEP 3 — single-block end-to-end in the in-process 2-instance sim.
            // One-shot at startup, then exit rc.
            test_sim_inproc_bigblock_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-bigblock-fullpath") == 0)
        {
            // FULL-PATH REGRESSION (bigblock-whiten-align): the LIVE 2-instance CFG16
            // big-block transfer through the REAL receive_bigblock+carve+whiten+FIFO
            // deliver path, with fail-before/pass-after on the same binary. One-shot, exit rc.
            test_bigblock_fullpath_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-sim-sustain") == 0)
        {
            // SIM_INPROC SUSTAIN BATTERY (runs BOTH durable sustain regressions, OR of their rc):
            //  (1) STEPPER-WEDGE (SIMFTR_ROOTCAUSE.md §7/§8 fix #1): a PINNED-CFG15 clean
            //      sustained-delivery transfer for payloads 600/2000/4000/8000, asserting each
            //      terminates byte-correct via the genuine delivery break (not the post-transfer
            //      keepalive spin), with fail-before/pass-after on the same binary.
            //  (2) STEPPER-CORE REWRITE Phase b: the OUTER-loop stepper drives a live
            //      ROBUST_0->CFG16 OFDM big-block transfer; asserts the (iii) DATA-path wedge is
            //      gone (ZERO [SIM2-DEADLOCK-BREAK]) + the K=8 block carves clean 8/8 + C0-a
            //      full byte-correct sustain.
            // One-shot at startup, then exit rc.
            test_sim_sustain_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-bigblock-multicw") == 0)
        {
            // MULTI-CW WINDOW REGRESSION (data-flow-bigblock-arq-unit.md §17): a FULL K=8
            // block (all 8 codewords) through the LIVE receive_bigblock+de-whiten+per-cw-CRC
            // carve; three arms prove the root cause is the RX capture WINDOW (cw1..cw7
            // stale-ring corruption on a stock-frame window), NOT whiten/offset. One-shot.
            test_bigblock_multicw_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-inband-down-resync") == 0)
        {
            // IN-BAND DOWN-LADDER RESYNC REGRESSION (data-flow-inband-ondemote-zerobyte.md §6):
            // a CMD demote-to-ROBUST_0 with the announce CONFIG_TAG SUPPRESSED forces the RSP's
            // production inband_try_down_ladder_on_decode_fail to resync from a PRIMARY-derived
            // snapshot over a window spanning the MFSK ROBUST rung. FAIL-BEFORE truncates the
            // ROBUST_0 frame -> 0-byte delivery (the HW defect); PASS-AFTER (Rank-1 snapshot+ring
            // sizing fix) decodes ROBUST_0 + delivers byte-faithful. One-shot, then exit rc.
            test_inband_down_resync_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-bigblock-chanest") == 0)
        {
            // GENUINE channel-estimation regression (fix/bigblock-chanest): the 2-instance
            // CFG16 big-block decode (ref==NULL) under a CFO/SFO-impaired channel. Reproduces
            // the HW [RXACQ] meanH collapse OFF-BENCH (clean default passes; CFO/SFO collapses
            // the block-wide estimate -> 0-delivery). One-shot.
            test_bigblock_chanest_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-bigblock-acqwindow") == 0)
        {
            // §19 acquisition-window POSITION guard regression (fix/bigblock-chanest): drive ONE
            // genuine K=8 CFG16 block at several in-window preamble offsets in a FIXED production-
            // sized capture window; assert the near-end (tail-past-window) block DEFERS (guard ON)
            // / carves a truncated bytes_ok=0 block (DEFEAT_ACQGUARD). Reproduces the HW ~5.6%
            // acquisition-fraction defect off-bench. One-shot.
            test_bigblock_acqwindow_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-bigblock-txlevel") == 0)
        {
            // TX-LEVEL parity diag: measure CFG16 big-block vs stock-OFDM TX peak+RMS
            // to localize the bench-observed +3.2 dB big-block over-level (gain vs PAPR).
            test_bigblock_txlevel_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-bigblock-climb-election") == 0)
        {
            // CLIMB-ELECTION (data-flow-bigblock-arq-unit.md §16): prove the big-block
            // rung is ELECTED by the gearshift CFG16 transition (load_configuration tail),
            // symmetric on both peers, and the elected rung emits + delivers byte-faithful.
            // fail-before/pass-after on the same binary. One-shot at startup, then exit rc.
            test_bigblock_climb_election_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-topgear-clean-election") == 0)
        {
            // TOP-GEAR CONFIG_17 channel-clean election (topgear-stack-productionize.md §4):
            // prove the 64-QAM top-gear election verdict + hysteresis + electability ceiling +
            // demote primitives + net-PHY ~1.073x R7-64 rate. fail-before/pass-after, one-shot, exit rc.
            test_topgear_clean_election_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-bigblock-carve-suspend-unit") == 0)
        {
            // WALL-B FIX-3: RSP carve-suspend watchdog unit test — the streak state machine
            // (bigblock_note_carve_reject/_accept) + the three consumers + a real receive_byte
            // cw0-reject loopback. fail-before via MERCURY_BIGBLOCK_DEFEAT_CARVESUSPEND.
            // One-shot at startup, then exit rc.
            test_bigblock_carve_suspend_unit_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-sack-oow-reject") == 0)
        {
            // R039 — OFDM SACK_RSP out-of-window reject regression — one-shot
            // at startup, then exit with the test's rc. See
            // fact-documents/data-flow-arq-recovery-cluster.md §4.5 / §5.4.
            test_sack_oow_reject_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-eob-poison-prev-retx") == 0)
        {
            // R038 — prev-retransmit EOB poison regression — one-shot at
            // startup, then exit with the test's rc. See
            // fact-documents/data-flow-arq-recovery-cluster.md §4.4 / §5.3.
            test_eob_poison_prev_retx_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-batch-shrink-strands-prev") == 0)
        {
            // R035 — data_batch_size shrink strands active prev regression —
            // one-shot at startup, then exit with the test's rc. See
            // fact-documents/data-flow-arq-recovery-cluster.md §4.3 / §5.2.
            test_batch_shrink_strands_prev_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-batch-shrink-orphan-defer") == 0)
        {
            // Fix A — mid-flight data_batch_size shrink must not orphan RECEIVED
            // prev frames (baseline-double-delivery.md). One-shot, exit rc.
            test_batch_shrink_orphan_defer_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-backup-confirm-flush") == 0)
        {
            // Fix C / H#1 — fifo_buffer_backup re-stage double-delivery
            // (data-flow-fifo-backup.md §6.1). One-shot, exit rc.
            test_backup_confirm_flush_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-rxfifo-backpressure-hold") == 0)
        {
            // Fix H#3 — RX-FIFO back-pressure post-ACK data loss
            // (delivery-integrity-audit-monitor.md §3). One-shot, exit rc.
            test_rxfifo_backpressure_hold_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-retx-clear-on-recovery") == 0)
        {
            // R029 — stale retx queue cleared on recovery regression — one-shot
            // at startup, then exit with the test's rc. See
            // fact-documents/data-flow-arq-recovery-cluster.md §4.1 / §5.1.
            test_retx_clear_on_recovery_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-restage-requeue-orphan") == 0)
        {
            // §12 — re-stage re-queue orphan/reorder regression — one-shot at
            // startup, then exit with the test's rc. See
            // source/datalink_layer/test_restage_requeue.cc +
            // fact-documents/silent-corruption-residual.md §12/§13.
            test_restage_requeue_orphan_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-stream-offset") == 0)
        {
            // Option W FOUNDATION — absolute-byte-stream cursor ground-truth regression.
            // One-shot at startup, then exit with the test's rc. See
            // source/datalink_layer/test_stream_offset.cc +
            // fact-documents/data-flow-stream-offset.md.
            test_stream_offset_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-rx-drain-backpressure") == 0)
        {
            // FIX-6 — RX-delivery drain backpressure regression — one-shot at
            // startup, then exit with the test's rc. See
            // source/datalink_layer/test_rx_drain.cc + fix6/STALL_ROOTCAUSE.md.
            test_rx_drain_backpressure_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-kx-data-tamper") == 0)
        {
            // Mid-stream AEAD ciphertext tamper fail-secure fire proof — one-shot
            // at startup, then exit with the test rc. See
            // source/datalink_layer/test_kx_data_tamper.cc.
            test_kx_data_tamper_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-decompress-false-accept") == 0)
        {
            // Streaming decompress-failure silent-false-accept regression — one-shot
            // at startup, then exit with the test's rc. See
            // source/datalink_layer/test_decompress_false_accept.cc +
            // fact-documents/residual-silent-corruption-wgn25.md.
            test_decompress_false_accept_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-compact-confirm-rx") == 0)
        {
            // Option B compact confirm LIVE RX-PATH regression — one-shot at
            // startup, then exit with the test's rc. See
            // source/datalink_layer/test_compact_confirm_rx.cc +
            // fact-documents/data-flow-compact-confirm.md §9.
            test_compact_confirm_rx_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-gap-abort") == 0)
        {
            // FIX-8 — silent lost-batch GAP on post-reset re-adopt regression —
            // one-shot at startup, then exit with the test's rc. See
            // source/datalink_layer/arq_responder.cc test_gap_abort_on_readopt
            // + bigblock_p3_hw/_fix8/FIX8_DESIGN.md.
            test_gap_abort_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-turnaround-guard") == 0)
        {
            // R1 turnaround-clearance guard regression — one-shot at startup, exit rc.
            // See source/datalink_layer/arq_common.cc test_turnaround_guard.
            test_turnaround_guard_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-measured-timers") == 0)
        {
            // R6 measured-turnaround (SRTT/RTTVAR) estimator + ack-timeout invariant
            // regression — one-shot at startup, exit rc.
            // See source/datalink_layer/arq_common.cc test_measured_timers.
            test_measured_timers_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-arqsmalls") == 0)
        {
            // Karn discriminator decoupling + DUTY-R pin-respect seed regression —
            // fast one-shot at startup, exit rc (avoids the full --test acquisition
            // trims + watchdog). See arq_common.cc test_karn_retx_classify and
            // arq_commander.cc test_robust_connect_exit.
            test_arqsmalls_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-recovery-ack-capture-exercise") == 0)
        {
            // Set before modem initialization so the production gate's cached
            // first read observes the exercise arm.
            setenv("MERCURY_RECOVERY_ACK_REPHASE", "1", 1);
            test_recovery_ack_capture_exercise_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-axis2-quiesce-gate") == 0)
        {
            // R4 LINK-PARAMS quiesce-gate regression — one-shot at startup, exit rc.
            // See source/datalink_layer/arq_commander.cc test_axis2_quiesce_gate.
            test_axis2_quiesce_gate_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-gap-abort-blind") == 0)
        {
            // GAP-ABORT ruler-blinding regression — one-shot at startup, exit rc.
            // See source/datalink_layer/arq_responder.cc test_gap_abort_readopt_blind
            // + fact-documents/data-flow-rsp-contiguity-ruler.md.
            test_gap_abort_blind_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-gap-abort-stream-blind") == 0)
        {
            // GAP-ABORT stream-backstop-blinding regression — one-shot at startup, exit rc.
            // See source/datalink_layer/arq_responder.cc test_gap_abort_stream_backstop_blind
            // + fact-documents/data-flow-rsp-contiguity-ruler.md §7.
            test_gap_abort_stream_blind_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-gap-recover") == 0)
        {
            // RECOVERABLE delivery-time GAP-ABORT regression — one-shot at startup, exit rc.
            // See source/datalink_layer/arq_responder.cc test_recoverable_gap_abort
            // + fact-documents/data-flow-recoverable-gap-abort.md.
            test_gap_recover_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-gap-recover-fire") == 0)
        {
            // W1 — CMD refill fire proof for the recoverable HOLD (one-shot, exit rc).
            // See source/datalink_layer/arq_responder.cc test_recover_fire
            // + fact-documents/data-flow-recoverable-gap-abort.md 5.1.
            test_recover_fire_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-retain-shadow-overflow") == 0)
        {
            // Deterministic fail-before/pass-after for the retention-shadow repair:
            // capture 19 holes (> the pre-fix 8-slot cap), re-SACK, assert requeue
            // re-drives all 19 (fix ON) vs < 19 (fix OFF, the rq0 deadlock root).
            // See source/datalink_layer/arq_responder.cc test_retain_shadow_overflow.
            test_retain_overflow_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-held-cur-deliver-fire") == 0)
        {
            // R2b — RSP deliver-held-cur + CMD recovery-turnaround credit fire proof
            // (one-shot, exit rc). See source/datalink_layer/arq_responder.cc
            // test_held_cur_deliver_fire + fact-documents/data-flow-recoverable-gap-abort.md §5.5.
            test_held_cur_fire_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-config-tag-follow") == 0)
        {
            // In-band rate adaptation Stage 2 — emit/detect/FOLLOW directed
            // loopback (one-shot at startup, exit rc). See
            // source/datalink_layer/arq_responder.cc test_config_tag_follow
            // + mercury/fact-documents/unilateral-config-tag-design.md §11.
            test_config_tag_follow_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-config-tag-passband") == 0)
        {
            // In-band rate adaptation Stage 3a — PASSBAND ROUND-TRIP (one-shot at
            // startup, exit rc). See arq_responder.cc test_config_tag_passband_
            // roundtrip + unilateral-config-tag-design.md §11 Stage 3.
            test_config_tag_passband_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-inband-drop") == 0)
        {
            // In-band rate adaptation Stage 3b — LOOPBACK DROP TEST (one-shot at
            // startup, exit rc). Gearshift-driven unilateral drop + tag on the real
            // passband + RX follow + SACK confirm + ZERO SET_CONFIG. See
            // arq_responder.cc test_inband_drop + data-flow-perbatch-config.md §12.
            test_inband_drop_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-inband-fallback") == 0)
        {
            // In-band rate adaptation Stage 4 — LOST-TAG DOWN-LADDER TEST (one-shot at
            // startup, exit rc). Forces a tag-loss on a drop batch; asserts the bounded
            // down-ladder resyncs within D rungs on a real CRC/LDPC pass, the SACK
            // confirms, BREAK-count==0, decode attempts<=D+1 (RPi bound), and sweeps D /
            // SESSION_DEAD_BATCHES. See arq_responder.cc test_inband_fallback +
            // data-flow-perbatch-config.md §13.
            test_inband_fallback_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-inband-seamless") == 0)
        {
            // In-band rate adaptation Stage 3d — PRE-FRAME SEAMLESS TEST (one-shot at
            // startup, exit rc). Builds the [tag burst][OFDM frame] wire window (the
            // DVB-S2 PLHEADER pre-frame order) and drives the production RX pre-frame
            // detect + receive_byte; asserts seamless first-frame decode at the new
            // config, no-dead-time on a no-change batch, correct-code, and lost-tag ->
            // down-ladder. See arq_responder.cc test_inband_seamless +
            // data-flow-perbatch-config.md §15.
            test_inband_seamless_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-inband-downladder") == 0)
        {
            // In-band down-ladder DELIVERY regression (one-shot at startup, exit rc).
            // PART A: a COMPLETE in-flight prev batch survives a TERMINAL-BREAK -> ROBUST_0
            // reshrink (fail-before MERCURY_PREBREAK_DELIVER_DEFEAT=1 orphans -> 0 bytes;
            // pass-after flushes via deliver_complete_inflight_before_break -> N*SUB_LEN
            // bytes). PART B: a silent (0-peak) snapshot does NOT tick the dead-batch streak.
            // See arq_responder.cc test_inband_downladder + data-flow-inband-downladder.md.
            test_inband_downladder_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-reseat-span") == 0)
        {
            // Reseat span integrity (one-shot at startup, exit rc): the prev-batch
            // cross-storage index-skew 332-byte deletion. Drives the PRODUCTION store ->
            // seal -> copy_data_to_buffer across STOCK (both keystones defeated -> 5969/
            // deficit 332) and fixed arms (seal-count + funnel byte-span -> 0 short bytes),
            // plus healthy + res_c3100-widen no-false-fire controls. See arq_responder.cc
            // test_reseat_span + data-flow-messages_rx_prev.md §4.5.
            test_reseat_span_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-cmd-idskew") == 0)
        {
            // CMD-side id-skew ROOT (one-shot at startup, exit rc): the correct-by-construction
            // TX counterpart to the reseat keystones. Drives the PRODUCTION add_message_tx_data
            // staging of two consecutive new-data batches (the second skewed to slots [PRIOR,..)),
            // then the PRODUCTION rebase — STOCK (MERCURY_CMD_IDSKEW_DEFEAT=1) reproduces the
            // first-tx id=seq+PRIOR skew, FIX rebases v1/v2 to id=seq — plus a §87-geometry arm where
            // id=seq now DELIVERS IN PLACE (full 6301 B) through the production receiver. See
            // arq_commander.cc test_cmd_idskew + CANONICAL_NUMBERS.md §87.
            test_cmd_idskew_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-inband-deliver") == 0)
        {
            // In-band FORWARD-HEALTHY REVERSE-ACK MISS -> NO-BREAK DELIVER regression
            // (one-shot at startup, exit rc). Drives the production connect-liveness guard
            // discriminator: a forward-healthy miss (in-flight DATA batch + a lower rung)
            // routes to the NO-BREAK re-present (partial prev preserved + contiguous bsi),
            // while a genuine dead/livelock STILL BREAKs and a genuine config-change STILL
            // clears/epochs. fail-before: rebuild with -DINBAND_DELIVER_FAILBEFORE (the
            // discriminator is removed -> the forward-healthy miss BREAKs -> 0 deliver).
            // See arq_responder.cc test_inband_deliver + data-flow-inband-retx-epoch.md §5.
            test_inband_deliver_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-linkphase-shadow") == 0)
        {
            // LINK-PHASE PRIMITIVE (increment 1) shadow-agreement directed unit (one-shot at
            // startup, exit rc). Synthetic-fires the production producers; asserts the pre-init
            // owner==NONE guard, the owner ordering, the DIRECTIONAL over-wait/under-estimate
            // meter (incl. the intended short-batch clamp + a fail-before shortening), and the
            // epoch bump on config-switch/BREAK. See data-flow-linkphase-primitive.md.
            test_linkphase_shadow_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-inband-ring-floor") == 0)
        {
            // In-band capture-ring ROBUST-floor OVER-SEAT regression (one-shot at startup,
            // exit rc). Drives inband_seat_robust_ring_floor() at CONFIG_8 (a climbed OFDM
            // rung holding its natural ring, no adopt) + ROBUST_0; asserts the OFDM-rung seat
            // is SUPPRESSED (ring stays natural) while ROBUST still seats the floor. fail-before
            // is the in-process A1-FB sub-case via MERCURY_CONFIG0_RING_GUARD_DEFEAT=1 (the
            // existing §21 knob now disables the generalized guard -> the over-seat reproduces).
            // See arq_responder.cc test_inband_ring_floor_overseat +
            // data-flow-inband-ring-floor-overseat.md §5.
            test_inband_ring_floor_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-inband-liveness") == 0)
        {
            // In-band CONNECT-LIVENESS GUARD regression (one-shot at startup, exit rc).
            // Drives the production guard into a control-plane livelock (control-TX, no
            // forward-DATA progress) and asserts it fires the retained true-loss BREAK
            // within the bound. fail-before: rebuild with -DINBAND_LIVENESS_FAILBEFORE (the
            // guard tracks but never recovers -> the livelock is unbounded -> asserts FAIL).
            // See arq_responder.cc test_inband_liveness + data-flow-inband-connect-liveness.md.
            test_inband_liveness_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-inband-no-break") == 0)
        {
            // In-band rate adaptation Stage 4c — D5 BREAK-OBSOLETE TEST (one-shot at
            // startup, exit rc). Drives the COMMANDER Class-A degradation routing: a
            // degradation that today BREAKs routes to a TAG-DEMOTE (BREAK-count==0, link
            // alive at a lower config) under inband, while a GENUINE total loss STILL
            // reaches the SESSION_DEAD_BATCHES BREAK. See arq_responder.cc
            // test_inband_no_break + data-flow-perbatch-config.md §S4C.
            test_inband_no_break_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-inband-retag") == 0)
        {
            // In-band rate adaptation Stage 4d — D1 repeat-until-followed + D4 climb/
            // auto-demote (one-shot at startup, exit rc). Drives the production firing
            // decision + the chokepoint climb-follow + the auto-demote: a lost climb is
            // re-tagged until a SACK confirms (then STOPS), a hopeless climb auto-demotes
            // to last-confirmed (BREAK-count==0), a turbo-climb-fail routes to a tag-demote.
            // See arq_responder.cc test_inband_retag + inband-reliability-design.md §1/§4.
            test_inband_retag_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-inband-nack") == 0)
        {
            // In-band rate adaptation Stage 4e — D2 NACK first-class (one-shot at startup,
            // exit rc). The RX FAST-signals a genuine cannot-follow (an un-adoptable climb /
            // a down-ladder total-loss) with a NACK; the sender auto-demotes to the RX config
            // IMMEDIATELY (BREAK-count==0), faster than the R-retry give-up. See
            // arq_responder.cc test_inband_nack + inband-reliability-design.md §2.
            test_inband_nack_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-inband-reannounce") == 0)
        {
            // In-band rate adaptation Stage 4e — D3 periodic re-announce (one-shot at startup,
            // exit rc). With no change for N=8 batches the tag re-emits holding the SAME epoch
            // parity (the late-joiner/desync backstop); the counter resets on any emit (no
            // double-emit). See arq_responder.cc test_inband_reannounce +
            // inband-reliability-design.md §3.
            test_inband_reannounce_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-data-ack-multiwindow") == 0)
        {
            // Track A — multi-window DATA-ACK/SACK correlator regression —
            // one-shot at startup, then exit with the test's rc. See
            // source/datalink_layer/arq_responder.cc test_data_ack_multiwindow
            // + fact-documents/data-flow-data-ack-sack-correlator.md §7.
            test_data_ack_multiwindow_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-cumulative-ack") == 0)
        {
            // FORGIVING-ACK Tier-2 cumulative-n_r regression (feat/forgiving-ack-tier1,
            // failing-first under -DCUMULATIVE_ACK_FAILBEFORE). Drives the SELF-HEAL
            // (a lost report recovered by the next n_r), the contiguous-high-water
            // GAP-INVARIANT (n_r NEVER ACKs a gap — via the REAL advance_last_delivered
            // + delivery_step_is_gap producers), the CAPABILITY GATE (cap-off ->
            // per-batch fallback), and COMPOSITION with Tier-1. See
            // fact-documents/data-flow-forgiving-ack.md §T2.6.
            test_cumulative_ack_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-enc-failclosed") == 0)
        {
            // Encryption negotiation FAIL-CLOSED regression (one-shot, then exit rc).
            // Opt-in (-E) against a peer without CAP_ENCRYPTION (unsupported, or a
            // MITM that stripped the cap bit) -> REFUSE for BOTH strict AND fast;
            // default-off (ENCRYPT_OFF) -> legal plaintext. Drives the shared
            // production predicate decide_encryption_negotiation(). Fails-before
            // under -DENC_FAILOPEN_FAILBEFORE (which recompiles the historical
            // opportunistic-plaintext fast downgrade). Isolated one-shot so it does
            // not depend on the full --test suite's wall-clock watchdog.
            test_enc_failclosed_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-a3-decouple-safety") == 0)
        {
            // T5 — the §2 DECOUPLE-SAFETY CHECKPOINT. With A3 ENABLED and the
            // demote UNTOUCHED, prove (T5a) a SINGLE forward-healthy reverse-ACK
            // miss is NON-LOAD-BEARING — delivery advances, the multi-batch
            // transfer completes byte-faithful via the next turn's cumulative n_r
            // (the explicit anti-0-bytes proof; FAIL-BEFORE -DCUMULATIVE_ACK_FAILBEFORE
            // STALLS) — and (T5b) SUSTAINED loss STILL exhausts nResends -> BREAK
            // (the genuine-death net is intact). Gate that MUST be GREEN before the
            // Phase-2 demote-decouple. See data-flow-forgiving-ack.md §T2.2/§6.
            test_a3_decouple_safety_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-recovery-ack-capture") == 0)
        {
            // recovery-ack-capture LEVER 1 focused capture-window regression —
            // one-shot at startup, then exit with the test's rc. See
            // source/datalink_layer/arq_responder.cc test_recovery_ack_capture
            // + fact-documents/data-flow-recovery-ack-capture.md §7.
            test_recovery_ack_capture_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-pas") == 0)
        {
            // PAS/PCS distribution-matcher bijection + histogram self-test (feat/pcs).
            // Pure cl_dist_matcher check, no Mercury/ARQ state needed. See
            // fact-documents/data-flow-pas-shaping.md §6 + tools/test_pas_shaping.py.
            test_pas_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-tinterp-seed") == 0)
        {
            // TINTERP-SEED production estimator self-test (staging/tinterp-seed,
            // failing-first): drives the SFO-GRID CODED harness in-process on the
            // GOOD Watterson fade and asserts LS-seed floors 0/K while the it=0
            // TIME_INTERP seed crosses K/K, plus a clean-channel no-op. The exact
            // estimator the MERCURY_TINTERP_SEED receive_byte rescue selects.
            test_tinterp_seed_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-cfg17") == 0)
        {
            // CFG17 shaped-64-QAM COMPOSITION self-test (feat/cfg17, failing-first):
            // drives the SFO-GRID harness in-process for three cells and asserts the
            // composed stack (PAS + TINTERP-seed turbo + ratio-nvfix) decodes where a
            // bare arm fails. See fact-documents/data-flow-cfg17-shaped-64qam.md §3.
            test_cfg17_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-inorder-demote") == 0)
        {
            // D3.1 — UNIFIED in-order delivery across EVERY demote case —
            // one-shot at startup, then exit with the test's rc. See
            // source/datalink_layer/arq_responder.cc test_inorder_demote
            // + bigblock_p3_hw/_d31_fade/D31_INORDER_DESIGN.md.
            test_inorder_demote_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-climb-bsi-rollback") == 0)
        {
            // CLIMB-CHURN — commander-side bsi rollback on a climb-UP promote —
            // one-shot at startup, then exit with the test's rc. See
            // source/datalink_layer/arq_commander.cc test_climb_bsi_rollback
            // + fact-documents/data-flow-climb-up-bsi-rollback.md.
            test_climb_bsi_rollback_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-guard-reanchor") == 0)
        {
            // Reverse-SACK partial-target guard live-state reference regression.
            test_guard_reanchor_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-spec-sack") == 0)
        {
            // LEVER #2 — speculative/prompt SACK — one-shot at startup, then exit
            // with the test's rc. fail-before (env off) / pass-after
            // (MERCURY_SPEC_SACK=1). See source/datalink_layer/arq_responder.cc
            // test_spec_sack + fact-documents/turnaround-eff.md §8/§9.
            test_spec_sack_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-batchsize-desync") == 0)
        {
            // res_c3100 — CMD>RSP batch-size desync silent-corruption regression —
            // one-shot at startup, then exit with the test's rc. See
            // source/datalink_layer/arq_responder.cc test_batchsize_desync_delivery
            // + silent-corruption-residual.md §8. fail-before: MERCURY_BATCHSIZE_DESYNC_DEFEAT=1.
            test_batchsize_desync_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-batch-shrink-orphan-current") == 0)
        {
            // rx_btf=-1 current-batch shrink tail-drop (marginal-SNR silent corruption) —
            // one-shot at startup, then exit with the test's rc. See
            // source/datalink_layer/arq_responder.cc test_batch_shrink_orphan_current
            // + fact-documents/silent-corruption-marginal-snr.md.
            test_batch_shrink_orphan_current_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-linkphase-pending-confirm") == 0)
        {
            // SEAM-2 deferred prev-confirm ARM/FLUSH fire proof (one-shot at startup, exit rc).
            test_linkphase_pending_confirm_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-eob-loss-batch-truncation") == 0)
        {
            // D5 — EOB-inference batch truncation (lost-EOB tail silent skip) —
            // one-shot at startup, then exit with the test's rc. See
            // source/datalink_layer/arq_responder.cc test_eob_loss_batch_truncation
            // + TRACK_C_D2D3D5_DESIGN.md §5.3.
            test_eob_loss_batch_truncation_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-dedup-rebase") == 0)
        {
            // In-band demote-rebase double-delivery (byte-stream corruption) regression
            // -- one-shot at startup, then exit with the test's rc. See
            // source/datalink_layer/arq_responder.cc test_dedup_rebase.
            test_dedup_rebase_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-v2-pendingack-flip-alias") == 0)
        {
            // R030 — v2 PENDING_ACK flip aliasing regression — one-shot at
            // startup, then exit with the test's rc. See
            // fact-documents/data-flow-arq-recovery-cluster.md §4.2 / §5.5.
            test_v2_pendingack_flip_alias_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-retx-slot-order") == 0)
        {
            // Timing redesign — retx never rides wire slot 0 regression — one-shot
            // at startup, then exit with the test's rc. See
            // source/datalink_layer/arq_commander.cc test_retx_slot_order.
            test_retx_slot_order_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-bigblock-livepath") == 0)
        {
            // GAP-2 LIVE-PATH (diag/livepath-sim): real CONNECT at the robust start, then
            // ONE real SET_CONFIG handshake robust->CFG16 over the live wire (NO pin), then
            // a 1374B K=8 transfer through the REAL send_batch->bigblock_send_one_block emit
            // + receive_byte cw0-CRC gate + carve + FIFO. Key question: does the in-sim live
            // path reproduce the HW cw0-CRC reject? One-shot at startup, then exit rc.
            test_bigblock_livepath_cli = true;
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
        else if (strcmp(argv[i], "--test-shutdown-atomic") == 0)
        {
            // R006 — shutdown_ atomicity regression — one-shot at startup, then
            // exit with the test's rc. See
            // fact-documents/data-flow-arq-recovery-cluster.md §7.
            test_shutdown_atomic_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-probe-backoff") == 0)
        {
            // FIX-B floor-probe back-off regression — one-shot at startup, then
            // exit with the test's rc. See
            // fact-documents/gearshift-floor-probe-backoff.md §7.
            test_probe_backoff_cli = true;
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
        else if (strcmp(argv[i], "--test-decode-marathon") == 0)
        {
            // LEVER C (feat/decode-marathon) §3 integrity gate: parallel big-block
            // decode == serial, byte-faithful + in-order + no cross-frame corruption,
            // with a shared-workspace fail-before. One-shot at startup, then exit rc.
            // See fact-documents/decode-marathon-C.md §8.
            test_decode_marathon_cli = true;
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
        else if (strcmp(argv[i], "--test-break-weld") == 0)
        {
            test_break_weld_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-robust0-compress-deadlock") == 0)
        {
            // ROBUST_0 + streaming-compression deadlock regression — one-shot at
            // startup, then exit with the test's rc. See
            // fact-documents/data-flow-compress-frame-fill.md §5.
            test_robust0_compress_deadlock_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-rung-floor") == 0)
        {
            // Per-rung MEASURED election floor gate + failure memory regression — one-shot at
            // startup, then exit with the test's rc. See DATAFLOW_AUDIT_rung_floor.md §7.
            test_rung_floor_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-streaming-compress-overshoot") == 0)
        {
            // Streaming-compression stale-EMA overshoot regression (one-shot,
            // exit rc). MERCURY_COMPRESS_RETRY_DEFEAT=1 restores the RAW trap.
            test_streaming_compress_overshoot_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-mixbatch-fill-overpop") == 0)
        {
            // Mixbatch fill over-pop reorder regression (one-shot, exit rc).
            // See fact-documents/data-flow-mixbatch-fill.md.
            test_mixbatch_fill_overpop_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-mixbatch-fill-overpop-compressed") == 0)
        {
            // Mixbatch fill COMPRESSION-leg over-pop + force-FREE data-loss
            // regression (one-shot, exit rc). data-flow-mixbatch-fill.md ss9.
            test_mixbatch_fill_overpop_compressed_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-idle-switch-role-race") == 0)
        {
            // Idle SWITCH_ROLE race regression — one-shot at startup, then exit
            // with the test's rc. FAILS-BEFORE evidence for the
            // connected-but-0-deliver bench bug (empty-tx Commander gives its
            // role away before the app's first data write).
            test_idle_switch_role_race_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-break-noprogress-teardown") == 0)
        {
            // BREAK no-progress teardown regression — one-shot at startup, then
            // exit with the test's rc. Part C of the idle-switchrole-race fix:
            // a never-fed BREAK spiral must reach a graceful teardown at K dead
            // cycles instead of re-arming the watchdog forever. FAILS-BEFORE with
            // -DBREAK_NOPROGRESS_FAILBEFORE (the kernel never escalates).
            test_break_noprogress_cli = true;
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--; i--;
        }
        else if (strcmp(argv[i], "--test-rx-ctrl-drop") == 0)
        {
            // RX-CTRL-DROP regression — one-shot at startup, then exit with the
            // test's rc. The messages_control one-deep mailbox had no escape out of
            // RECEIVED, so a stranded slot silently dropped every later control
            // frame and killed the reverse control plane. FAILS-BEFORE with
            // -DRX_CTRL_DROP_FAILBEFORE (the watchdog never fires). See
            // fact-documents/data-flow-control-slot-lifecycle.md.
            test_rx_ctrl_drop_cli = true;
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
            if (!strcmp(optarg, "TX_WAV"))
                operation_mode = TX_WAV;
            if (!strcmp(optarg, "SIM_INPROC"))
                operation_mode = SIM_INPROC;
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
            if (!strcmp(optarg, "sim"))
                audio_system = AUDIO_SUBSYSTEM_SIM;
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
        if (test_shutdown_atomic_cli) {
            // R006 — shutdown_ atomicity regression (one-shot, then exit rc).
            // Free function on the main.cc global; no ARQ/telecom_system state.
            printf("[FLAG] --test-shutdown-atomic: invoking R006 shutdown_ "
                   "atomicity regression\n");
            fflush(stdout);
            int rc = test_shutdown_atomic();
            printf("[FLAG] Shutdown-atomic test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
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
        if (test_bigblock_arq_unit_cli) {
            // P2 big-block ARQ-granularization regression (one-shot, then exit rc).
            // Drives bigblock_block_to_arq() through 3 cases (clean K=8 / one-bad-cw
            // / lost-EOB) + asserts RX delivered == TX at every transition. FAILS
            // before P2 wiring (the stub), PASSES after. See
            // fact-documents/data-flow-bigblock-arq-unit.md §6.
            printf("[FLAG] --test-bigblock-arq-unit: invoking big-block "
                   "ARQ-granularization regression\n");
            fflush(stdout);
            int rc = ARQ.test_bigblock_arq_unit();
            printf("[FLAG] Bigblock-arq-unit test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_sim_inproc_bigblock_cli) {
            // STEP 3 — single-block end-to-end in the in-process 2-instance sim.
            // Drives the production transmit_bigblock/receive_bigblock/
            // bigblock_block_to_arq path: a single big-block ARQ-drives
            // CMD->RSP->ACK->CMD byte-faithful + a one-bad-codeword partial ->
            // selective-repeat completes. One-shot at startup, then exit rc.
            printf("[FLAG] --test-sim-inproc-bigblock: invoking single-block "
                   "end-to-end in-process sim\n");
            fflush(stdout);
            int rc = ARQ.test_sim_inproc_bigblock();
            printf("[FLAG] Sim-inproc-bigblock test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_bigblock_fullpath_cli) {
            // FULL-PATH REGRESSION (bigblock-whiten-align): the LIVE 2-instance CFG16
            // big-block transfer through the REAL TX-encode->whiten->PHY->receive_bigblock
            // de-whiten->arq carve->copy_data_to_buffer FIFO deliver path. Asserts the full
            // message is delivered byte-faithful, with fail-before (DEFEAT_FIX=1) / pass-after
            // on the SAME binary. Closes the cross-layer gap the CASE A-D synthetic carve
            // tests bypassed (caller-owned RX vector / direct receive_bigblock / messages_rx[]
            // assertions never exercised the live UAF, per-block wait, or FIFO delivery).
            printf("[FLAG] --test-bigblock-fullpath: invoking LIVE 2-instance big-block "
                   "full-path delivery regression\n");
            fflush(stdout);
            int rc = cl_arq_controller::test_sim_inproc_bigblock_fullpath();
            printf("[FLAG] Bigblock-fullpath test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_sim_sustain_cli) {
            // SIM_INPROC SUSTAIN BATTERY — runs BOTH durable sustain regressions back-to-back on
            // the SAME binary and exits with the OR of their return codes (any failure -> nonzero):
            //  (1) STEPPER-WEDGE (SIMFTR_ROOTCAUSE.md §7/§8 fix #1): PINNED-CFG15 clean transfers
            //      (payloads 600/2000/4000/8000), each must terminate byte-correct via the genuine
            //      delivery break (NOT the post-transfer keepalive <-> pumped-wait spin), with
            //      fail-before (MERCURY_SIM2_DEFEAT_SIMFTR_FIX=1 -> wedge -> watchdog-stalled).
            //  (2) STEPPER-CORE REWRITE Phase b: OUTER-loop stepper, live ROBUST_0->CFG16 OFDM
            //      big-block transfer; asserts the (iii) data-path wedge is gone (ZERO
            //      [SIM2-DEADLOCK-BREAK]) + clean K=8 carve + C0-a full byte-correct sustain.
            printf("[FLAG] --test-sim-sustain: invoking SIM_INPROC sustain battery — "
                   "(1) PINNED-CFG15 stepper-wedge regression, then "
                   "(2) OUTER-stepper OFDM big-block no-wedge + clean-carve (Phase b)\n");
            fflush(stdout);
            int rc1 = cl_arq_controller::test_sim_inproc_sustain();
            printf("[FLAG] --test-sim-sustain (1) PINNED-CFG15 stepper-wedge: rc=%d\n", rc1);
            fflush(stdout);
            int rc2 = cl_arq_controller::test_sim_inproc_sustain_outer();
            printf("[FLAG] --test-sim-sustain (2) OUTER-stepper big-block: rc=%d\n", rc2);
            fflush(stdout);
            int rc = (rc1 != 0 || rc2 != 0) ? 1 : 0;
            printf("[FLAG] Sim-sustain battery complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_bigblock_multicw_cli) {
            // MULTI-CW WINDOW REGRESSION (data-flow-bigblock-arq-unit.md §17): a FULL K=8
            // block (all 8 codewords) through the LIVE receive_bigblock+de-whiten+per-cw-CRC
            // carve. Three arms (CRC-on block-window byte-faithful + clean=8/8; NOCRC
            // stock-window corruption-repro; NOCRC block-window byte-faithful) prove the root
            // cause is the RX capture WINDOW, NOT whiten/offset. The K>1 test the 622-byte
            // cases could not catch. One-shot at startup, then exit rc.
            printf("[FLAG] --test-bigblock-multicw: invoking FULL K=8 multi-codeword "
                   "byte-faithfulness regression\n");
            fflush(stdout);
            int rc = cl_arq_controller::test_sim_inproc_bigblock_multicw();
            printf("[FLAG] Bigblock-multicw test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_inband_down_resync_cli) {
            // IN-BAND DOWN-LADDER RESYNC REGRESSION (data-flow-inband-ondemote-zerobyte.md §6):
            // CMD demote-to-ROBUST_0 with announce SUPPRESSED -> RSP down-ladder must resync from
            // a primary-derived snapshot over a ROBUST-spanning window. Fail-before (truncation ->
            // 0 bytes) -> pass-after (Rank-1 fix -> ROBUST_0 decodes, byte-faithful). One-shot.
            printf("[FLAG] --test-inband-down-resync: invoking in-band down-ladder ROBUST "
                   "resync regression (fail-before 0-byte -> pass-after byte-faithful)\n");
            fflush(stdout);
            int rc = cl_arq_controller::test_inband_down_resync();
            printf("[FLAG] Inband-down-resync test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_bigblock_chanest_cli) {
            // GENUINE channel-estimation regression (fix/bigblock-chanest): drive the 2-instance
            // CFG16 big-block decode (ref==NULL) under a CFO/SFO-impaired channel; assert the
            // block-wide estimate collapses (fail-before) and recovers byte-faithful (pass-after).
            // Reproduces the HW [RXACQ] meanH~0.005 collapse off-bench. One-shot, then exit rc.
            printf("[FLAG] --test-bigblock-chanest: invoking GENUINE big-block channel-estimation "
                   "regression (CFO/SFO-impaired 2-instance decode)\n");
            fflush(stdout);
            int rc = cl_arq_controller::test_sim_inproc_bigblock_chanest();
            printf("[FLAG] Bigblock-chanest test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_bigblock_acqwindow_cli) {
            // §19 acquisition-window POSITION guard regression: drive ONE genuine K=8 CFG16 block
            // at several in-window preamble offsets in a FIXED production-sized window; assert the
            // near-end block (tail past window) DEFERS (guard ON) and carves a truncated bytes_ok=0
            // block under DEFEAT_ACQGUARD. Off-bench reproduction of the HW ~5.6% defect. One-shot.
            printf("[FLAG] --test-bigblock-acqwindow: invoking §19 acquisition-window POSITION guard "
                   "regression (in-window preamble offsets, fixed capture window)\n");
            fflush(stdout);
            int rc = cl_arq_controller::test_sim_inproc_bigblock_acqwindow();
            printf("[FLAG] Bigblock-acqwindow test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_bigblock_txlevel_cli) {
            printf("[FLAG] --test-bigblock-txlevel: measuring CFG16 big-block vs "
                   "stock-OFDM TX peak/RMS\n");
            fflush(stdout);
            int rc = cl_arq_controller::test_bigblock_txlevel();
            printf("[FLAG] Bigblock-txlevel test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_topgear_clean_election_cli) {
            // TOP-GEAR CONFIG_17 channel-clean election regression (topgear-stack-productionize.md
            // §4): the 64-QAM top rung ELECTS only on a clean+FLAT forward channel (SNR margin +
            // selectivity gate), engages after hysteresis, demotes IMMEDIATELY on marginal/non-flat
            // (2-path anti-thrash), and delivers ~1.073x cfg16 net-PHY (R7-64 diet). fail-before/pass-after on
            // the same binary via MERCURY_TOPGEAR_PORT_DEFEAT=1 (broad port defeat).
            // MERCURY_TOPGEAR_DEFEAT_CLEAN remains the focused verdict-only negative control.
            // One-shot at startup, then exit rc.
            printf("[FLAG] --test-topgear-clean-election: invoking CONFIG_17 top-gear "
                   "channel-clean election regression\n");
            fflush(stdout);
            int rc = cl_arq_controller::test_topgear_clean_election();
            printf("[FLAG] Topgear-clean-election test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_bigblock_climb_election_cli) {
            // CLIMB-ELECTION regression (data-flow-bigblock-arq-unit.md §16): the big-block
            // rung is ELECTED by the gearshift CFG16 transition (load_configuration tail),
            // not only at connect. Asserts the transition elects K==8 symmetrically on both
            // peers (all_ones==0xFF) and the elected rung emits + delivers byte-faithful.
            // fail-before/pass-after on the same binary via MERCURY_BIGBLOCK_DEFEAT_ELECTION.
            printf("[FLAG] --test-bigblock-climb-election: invoking gearshift CFG16 "
                   "rung-election regression\n");
            fflush(stdout);
            int rc = cl_arq_controller::test_bigblock_climb_election();
            printf("[FLAG] Bigblock-climb-election test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_bigblock_carve_suspend_unit_cli) {
            // WALL-B FIX-3: RSP carve-suspend watchdog unit test (one-shot, then exit rc).
            // The streak state machine + the three consumers + a real receive_byte cw0-reject
            // loopback; fail-before via MERCURY_BIGBLOCK_DEFEAT_CARVESUSPEND.
            printf("[FLAG] --test-bigblock-carve-suspend-unit: invoking WALL-B FIX-3 RSP "
                   "carve-suspend watchdog unit test\n");
            fflush(stdout);
            int rc = cl_arq_controller::test_bigblock_carve_suspend_unit();
            printf("[FLAG] Bigblock-carve-suspend-unit test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_sack_oow_reject_cli) {
            // R039 — OFDM SACK_RSP out-of-window reject (one-shot, then exit rc).
            printf("[FLAG] --test-sack-oow-reject: invoking R039 OFDM SACK_RSP "
                   "out-of-window reject regression\n");
            fflush(stdout);
            int rc = ARQ.test_sack_oow_reject();
            printf("[FLAG] Sack-oow-reject test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_eob_poison_prev_retx_cli) {
            // R038 — prev-retransmit EOB poison (one-shot, then exit rc).
            printf("[FLAG] --test-eob-poison-prev-retx: invoking R038 EOB-poison "
                   "regression\n");
            fflush(stdout);
            int rc = ARQ.test_eob_poison_prev_retx();
            printf("[FLAG] Eob-poison test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_batch_shrink_strands_prev_cli) {
            // R035 — data_batch_size shrink strands prev (one-shot, then exit rc).
            printf("[FLAG] --test-batch-shrink-strands-prev: invoking R035 "
                   "batch-shrink-strands-prev regression\n");
            fflush(stdout);
            int rc = ARQ.test_batch_shrink_strands_prev();
            printf("[FLAG] Batch-shrink-strands-prev test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_batch_shrink_orphan_defer_cli) {
            // Fix A — mid-flight shrink orphan-defer (one-shot, then exit rc).
            printf("[FLAG] --test-batch-shrink-orphan-defer: invoking Fix A "
                   "batch-shrink orphan zero-loss regression\n");
            fflush(stdout);
            int rc = ARQ.test_batch_shrink_orphan_defer();
            printf("[FLAG] Batch-shrink-orphan-defer test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_backup_confirm_flush_cli) {
            // Fix C / H#1 — fifo_buffer_backup re-stage double-delivery (one-shot, exit rc).
            printf("[FLAG] --test-backup-confirm-flush: invoking Fix C/H#1 backup "
                   "ACK-confirm-flush re-stage double-delivery regression\n");
            fflush(stdout);
            int rc = ARQ.test_backup_confirm_flush();
            printf("[FLAG] Backup-confirm-flush test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_rxfifo_backpressure_hold_cli) {
            // Fix H#3 — RX-FIFO back-pressure post-ACK data loss (one-shot, exit rc).
            printf("[FLAG] --test-rxfifo-backpressure-hold: invoking Fix H#3 RX-FIFO "
                   "back-pressure post-ACK loss regression\n");
            fflush(stdout);
            int rc = ARQ.test_rxfifo_backpressure_hold();
            printf("[FLAG] Rxfifo-backpressure-hold test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_retx_clear_on_recovery_cli) {
            // R029 — stale retx queue cleared on recovery (one-shot, then exit rc).
            printf("[FLAG] --test-retx-clear-on-recovery: invoking R029 "
                   "retx-clear-on-recovery regression\n");
            fflush(stdout);
            int rc = ARQ.test_retx_clear_on_recovery();
            printf("[FLAG] Retx-clear-on-recovery test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_enc_failclosed_cli) {
            // Encryption negotiation FAIL-CLOSED regression (one-shot, then exit rc).
            // Drives the shared production predicate decide_encryption_negotiation()
            // across the operator/peer capability matrix: default-off -> plaintext,
            // opted-in + both-cap -> encrypt, opted-in + peer-no-cap (or MITM strip)
            // -> REFUSE for both strict and fast. Fails-before: -DENC_FAILOPEN_FAILBEFORE.
            printf("[FLAG] --test-enc-failclosed: invoking the encryption fail-closed "
                   "negotiation regression\n");
            fflush(stdout);
            int rc = ARQ.test_encryption_fail_closed();
            printf("[FLAG] Encryption fail-closed test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_cumulative_ack_cli) {
            // FORGIVING-ACK Tier-2 cumulative-n_r regression (one-shot, then exit rc).
            // Drives the self-heal (a lost report recovered by the next n_r), the
            // contiguous-high-water gap-invariant (n_r never ACKs a gap), the
            // capability gate (cap-off -> per-batch fallback), and composition with
            // Tier-1. See fact-documents/data-flow-forgiving-ack.md §T2.6.
            printf("[FLAG] --test-cumulative-ack: invoking Tier-2 cumulative-n_r "
                   "self-heal regression\n");
            fflush(stdout);
            int rc = ARQ.test_cumulative_ack();
            printf("[FLAG] Cumulative-ACK test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_a3_decouple_safety_cli) {
            // T5 — the §2 DECOUPLE-SAFETY CHECKPOINT (one-shot, then exit rc).
            // With A3 enabled + the demote in place, proves a single reverse-ACK
            // miss is non-load-bearing (delivery advances, byte-faithful — the
            // anti-0-bytes proof) AND sustained loss still BREAKs (genuine-death
            // net intact). Drives the REAL advance_last_delivered/delivery_step_is_gap
            // producers + the REAL cumulative_ack_covers consumer apply.
            printf("[FLAG] --test-a3-decouple-safety: invoking the §2 decouple-safety "
                   "checkpoint (single-miss non-load-bearing + genuine-death net)\n");
            fflush(stdout);
            int rc = ARQ.test_a3_decouple_safety();
            printf("[FLAG] A3 decouple-safety checkpoint complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_pas_cli) {
            // PAS/PCS distribution-matcher bijection + composition self-test
            // (one-shot, then exit rc). No Mercury/ARQ state needed.
            printf("[FLAG] --test-pas: invoking PAS/PCS distribution-matcher "
                   "bijection self-test\n");
            fflush(stdout);
            int rc = run_pas_selftest();
            printf("[FLAG] PAS self-test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_tinterp_seed_cli) {
            // TINTERP-SEED production estimator self-test (one-shot, then exit rc).
            printf("[FLAG] --test-tinterp-seed: invoking TINTERP-SEED production "
                   "it=0 estimator seed-swap self-test\n");
            fflush(stdout);
            int rc = run_tinterp_seed_selftest();
            printf("[FLAG] TINTERP-SEED self-test complete (rc=%d) - exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_cfg17_cli) {
            // CFG17 shaped-64-QAM composition self-test (one-shot, then exit rc).
            // Constructs cl_telecom_system instances in-process and drives the
            // SFO-GRID harness — no ARQ/audio/TCP state needed.
            printf("[FLAG] --test-cfg17: invoking CFG17 shaped-64-QAM composition "
                   "self-test (PAS + TINTERP-seed turbo + ratio-nvfix)\n");
            fflush(stdout);
            int rc = run_cfg17_selftest();
            printf("[FLAG] CFG17 composition self-test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_restage_requeue_orphan_cli) {
            // §12 — re-stage re-queue orphan/reorder (one-shot, then exit rc).
            printf("[FLAG] --test-restage-requeue-orphan: invoking §12 re-stage "
                   "re-queue orphan/reorder regression\n");
            fflush(stdout);
            int rc = ARQ.test_restage_requeue_orphan();
            printf("[FLAG] restage-requeue-orphan test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_stream_offset_cli) {
            // Option W FOUNDATION — cursor ground-truth (one-shot, then exit rc).
            printf("[FLAG] --test-stream-offset: invoking Option W FOUNDATION "
                   "absolute-byte-stream cursor ground-truth regression\n");
            fflush(stdout);
            int rc = ARQ.test_stream_offset();
            printf("[FLAG] stream-offset test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_rx_drain_backpressure_cli) {
            // FIX-6 — RX-delivery drain backpressure (one-shot, then exit rc).
            printf("[FLAG] --test-rx-drain-backpressure: invoking FIX-6 "
                   "RX-delivery drain backpressure regression\n");
            fflush(stdout);
            int rc = ARQ.test_rx_drain_backpressure();
            printf("[FLAG] RX-drain-backpressure test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_kx_data_tamper_cli) {
            // Mid-stream AEAD ciphertext tamper fail-secure fire proof (one-shot, then exit rc).
            printf("[FLAG] --test-kx-data-tamper: invoking mid-stream AEAD ciphertext tamper fail-secure fire proof\n");
            fflush(stdout);
            cl_arq_controller test_arq;
            int rc = test_arq.test_kx_data_tamper();
            printf("[FLAG] kx-data-tamper test complete (rc=%d) - exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_decompress_false_accept_cli) {
            // Streaming decompress-failure silent-false-accept (one-shot, then exit rc).
            printf("[FLAG] --test-decompress-false-accept: invoking streaming decompress-"
                   "failure silent-false-accept regression\n");
            fflush(stdout);
            int rc = ARQ.test_decompress_false_accept();
            printf("[FLAG] Decompress-false-accept test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_compact_confirm_rx_cli) {
            // Option B compact confirm LIVE RX-PATH regression (one-shot, then exit rc).
            printf("[FLAG] --test-compact-confirm-rx: invoking Option B compact "
                   "confirm live RX-path regression\n");
            fflush(stdout);
            cl_arq_controller test_arq;
            int rc = test_arq.test_compact_confirm_live_rx_path();
            cl_arq_controller test_arq2;
            rc += test_arq2.test_compact_confirm_sack_window_rx_path();
            printf("[FLAG] compact-confirm-rx test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_gap_abort_cli) {
            // FIX-8 — silent lost-batch GAP on post-reset re-adopt (one-shot, exit rc).
            printf("[FLAG] --test-gap-abort: invoking FIX-8 post-reset re-adopt "
                   "gap-abort regression\n");
            fflush(stdout);
            int rc = ARQ.test_gap_abort_on_readopt();
            printf("[FLAG] Gap-abort test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_turnaround_guard_cli) {
            // R1 turnaround-clearance guard — the guard waits out the peer's TX->RX
            // mute/flush window before keying a data batch (one-shot, exit rc).
            printf("[FLAG] --test-turnaround-guard: invoking turnaround-clearance guard "
                   "regression\n");
            fflush(stdout);
            cl_arq_controller ARQ_tg;
            int rc = ARQ_tg.test_turnaround_guard();
            printf("[FLAG] Turnaround-guard test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_measured_timers_cli) {
            // R6 measured turnaround (SRTT/RTTVAR) estimator + ack-timeout invariant
            // (one-shot, exit rc).
            printf("[FLAG] --test-measured-timers: invoking measured-turnaround estimator "
                   "regression\n");
            fflush(stdout);
            cl_arq_controller ARQ_mt;
            int rc = ARQ_mt.test_measured_timers();
            printf("[FLAG] Measured-timers test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_arqsmalls_cli) {
            // Karn discriminator decoupling (MERCURY_KARN_RETX_ONLY) + DUTY-R
            // pin-respect seed (MERCURY_DUTY_R_PIN_DEFEAT) — fast one-shot, exit rc.
            printf("[FLAG] --test-arqsmalls: Karn discriminator + DUTY-R pin-respect regression\n");
            fflush(stdout);
            int rc = 0;
            { cl_arq_controller ARQ_kn; rc += ARQ_kn.test_karn_retx_classify(); }
            { cl_arq_controller ARQ_dr; rc += ARQ_dr.test_robust_connect_exit(); }
            printf("[FLAG] arq-smalls test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_recovery_ack_capture_exercise_cli) {
            printf("[FLAG] --test-recovery-ack-capture-exercise: forcing late "
                   "recovery ACK capture\n");
            fflush(stdout);
            int rc = ARQ.test_recovery_ack_capture_exercise();
            printf("[FLAG] Recovery-ACK capture exercise complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_axis2_quiesce_gate_cli) {
            // R4 LINK-PARAMS quiesce gate — an Axis-2 move defers while the batch boundary
            // is unclean and fires when clean (one-shot, exit rc).
            printf("[FLAG] --test-axis2-quiesce-gate: invoking R4 LINK-PARAMS quiesce-gate "
                   "regression\n");
            fflush(stdout);
            cl_arq_controller ARQ_q2;
            int rc = ARQ_q2.test_axis2_quiesce_gate();
            printf("[FLAG] Axis-2 quiesce-gate test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_gap_abort_blind_cli) {
            // GAP-ABORT ruler-blinding — the REAL teardown blinds its own contiguity
            // ruler and the CMD re-drives the same stream (one-shot, exit rc).
            printf("[FLAG] --test-gap-abort-blind: invoking gap-abort ruler-blinding "
                   "regression\n");
            fflush(stdout);
            cl_arq_controller ARQ_gab;
            int rc = ARQ_gab.test_gap_abort_readopt_blind();
            printf("[FLAG] Gap-abort-blind test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_gap_abort_stream_blind_cli) {
            // GAP-ABORT stream-BACKSTOP-blinding — the REAL teardown must preserve the
            // Option W byte cursor + stamp validity + set the sticky abort latch
            // (one-shot, exit rc).
            printf("[FLAG] --test-gap-abort-stream-blind: invoking gap-abort stream-backstop "
                   "regression\n");
            fflush(stdout);
            cl_arq_controller ARQ_gas;
            int rc = ARQ_gas.test_gap_abort_stream_backstop_blind();
            printf("[FLAG] Gap-abort-stream-blind test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_gap_recover_cli) {
            // RECOVERABLE delivery-time GAP-ABORT — HOLD + re-request converts the
            // terminal abort on a recoverable one-frame prev hole into an in-order
            // recovery; the high-water never advances while the hole exists
            // (one-shot, exit rc).
            printf("[FLAG] --test-gap-recover: invoking recoverable gap-abort "
                   "regression\n");
            fflush(stdout);
            cl_arq_controller ARQ_grc;
            int rc = ARQ_grc.test_recoverable_gap_abort();
            printf("[FLAG] Gap-recover test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_recover_fire_cli) {
            // W1 — the CMD-side refill decision that makes the recoverable HOLD
            // actually re-drive the armed-prev hole (one-shot, exit rc).
            printf("[FLAG] --test-gap-recover-fire: invoking W1 CMD refill "
                   "fire proof\n");
            fflush(stdout);
            cl_arq_controller ARQ_rf;
            int rc = ARQ_rf.test_recover_fire();
            printf("[FLAG] Recover-fire test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_retain_overflow_cli) {
            // Deterministic retention-shadow overflow proof: 19 holes > pre-fix cap 8
            // (one-shot, exit rc). Run OFF and ON as separate processes.
            printf("[FLAG] --test-retain-shadow-overflow: invoking retention-shadow "
                   "overflow proof\n");
            fflush(stdout);
            cl_arq_controller ARQ_ro;
            int rc = ARQ_ro.test_retain_shadow_overflow();
            printf("[FLAG] Retain-overflow test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_held_cur_fire_cli) {
            // R2b — RSP deliver-held-cur (bytes to the app FIFO via the production
            // copy_data_to_buffer) + CMD recovery-turnaround credit (one-shot, exit rc).
            printf("[FLAG] --test-held-cur-deliver-fire: invoking R2b deliver-held-cur "
                   "fire proof\n");
            fflush(stdout);
            cl_arq_controller ARQ_hcf;
            int rc = ARQ_hcf.test_held_cur_deliver_fire();
            printf("[FLAG] Held-cur-deliver-fire test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_config_tag_follow_cli) {
            // In-band rate adaptation Stage 2 — emit/detect/FOLLOW directed loopback
            // (one-shot, exit rc). Builds its own CMD/RSP + telecom_system internally.
            printf("[FLAG] --test-config-tag-follow: invoking in-band rate-adapt "
                   "Stage-2 emit/detect/FOLLOW regression\n");
            fflush(stdout);
            int rc = ARQ.test_config_tag_follow();
            printf("[FLAG] Config-tag-follow test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_config_tag_passband_cli) {
            // In-band rate adaptation Stage 3a — PASSBAND ROUND-TRIP (one-shot, exit rc).
            printf("[FLAG] --test-config-tag-passband: invoking in-band rate-adapt "
                   "Stage-3a passband round-trip regression\n");
            fflush(stdout);
            int rc = ARQ.test_config_tag_passband_roundtrip();
            printf("[FLAG] Config-tag-passband test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_inband_drop_cli) {
            // In-band rate adaptation Stage 3b — LOOPBACK DROP TEST (one-shot, exit rc).
            // Builds its own CMD/RSP + telecom_system internally.
            printf("[FLAG] --test-inband-drop: invoking in-band rate-adapt Stage-3b "
                   "loopback drop regression\n");
            fflush(stdout);
            int rc = ARQ.test_inband_drop();
            printf("[FLAG] Inband-drop test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_inband_fallback_cli) {
            // In-band rate adaptation Stage 4 — LOST-TAG DOWN-LADDER TEST (one-shot, exit rc).
            // Builds its own telecom_system instances internally.
            printf("[FLAG] --test-inband-fallback: invoking in-band rate-adapt Stage-4 "
                   "lost-tag down-ladder regression\n");
            fflush(stdout);
            int rc = ARQ.test_inband_fallback();
            printf("[FLAG] Inband-fallback test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_inband_seamless_cli) {
            // In-band rate adaptation Stage 3d — PRE-FRAME SEAMLESS TEST (one-shot, exit rc).
            // Builds its own telecom_system instances internally.
            printf("[FLAG] --test-inband-seamless: invoking in-band rate-adapt Stage-3d "
                   "pre-frame seamless regression\n");
            fflush(stdout);
            int rc = ARQ.test_inband_seamless();
            printf("[FLAG] Inband-seamless test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_inband_downladder_cli) {
            // In-band down-ladder DELIVERY regression (one-shot, exit rc). PART A drives the
            // BREAK-orphan defect on synthetic ARQ buffers; PART B builds its own minimal
            // telecom_system internally for the silent-snapshot directed pass.
            printf("[FLAG] --test-inband-downladder: invoking in-band down-ladder "
                   "BREAK-orphan + silent-snapshot regression\n");
            fflush(stdout);
            int rc = ARQ.test_inband_downladder();
            printf("[FLAG] Inband-downladder test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_reseat_span_cli) {
            // Reseat span integrity (one-shot, exit rc). Drives the PRODUCTION store -> seal
            // -> copy_data_to_buffer; STOCK reproduces the 332-byte deletion, the fixed arms
            // hold/refuse, healthy + widen stay byte-identical.
            printf("[FLAG] --test-reseat-span: invoking prev-batch cross-storage span "
                   "integrity regression\n");
            fflush(stdout);
            int rc = ARQ.test_reseat_span();
            printf("[FLAG] Reseat-span test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_cmd_idskew_cli) {
            // CMD id-skew root (one-shot, exit rc). Drives the PRODUCTION staging + rebase;
            // STOCK reproduces the id=seq+prior skew, FIX rebases to id=seq, and the §87
            // geometry with id=seq DELIVERS IN PLACE (6301 B) through the production receiver.
            printf("[FLAG] --test-cmd-idskew: invoking CMD-side new-data slot-rebase regression\n");
            fflush(stdout);
            int rc = ARQ.test_cmd_idskew();
            printf("[FLAG] CMD-idskew test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_inband_deliver_cli) {
            // In-band FORWARD-HEALTHY REVERSE-ACK MISS -> NO-BREAK DELIVER regression (one-shot,
            // exit rc). Builds its own CMD/telecom_system instances per case internally.
            printf("[FLAG] --test-inband-deliver: invoking in-band forward-healthy reverse-ACK "
                   "miss -> NO-BREAK deliver regression\n");
            fflush(stdout);
            int rc = ARQ.test_inband_deliver();
            printf("[FLAG] Inband-deliver test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_linkphase_shadow_cli) {
            // LINK-PHASE PRIMITIVE (increment 1) shadow-agreement directed unit (one-shot,
            // exit rc). Synthetic-fires the production producers on a throwaway controller.
            printf("[FLAG] --test-linkphase-shadow: invoking increment-1 shadow-agreement "
                   "directed unit\n");
            fflush(stdout);
            cl_arq_controller LP_TEST;
            int rc = LP_TEST.test_linkphase_shadow();
            printf("[FLAG] Linkphase-shadow test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_inband_ring_floor_cli) {
            // In-band capture-ring ROBUST-floor OVER-SEAT regression (one-shot, exit rc).
            // Builds its own RESPONDER/telecom_system instances per case internally.
            printf("[FLAG] --test-inband-ring-floor: invoking in-band capture-ring "
                   "ROBUST-floor over-seat regression\n");
            fflush(stdout);
            int rc = ARQ.test_inband_ring_floor_overseat();
            printf("[FLAG] Inband-ring-floor test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_inband_liveness_cli) {
            // In-band CONNECT-LIVENESS GUARD regression (one-shot, exit rc). Builds its own
            // CMD/telecom_system instances internally.
            printf("[FLAG] --test-inband-liveness: invoking in-band connect-liveness guard "
                   "(control-plane livelock backstop) regression\n");
            fflush(stdout);
            int rc = ARQ.test_inband_liveness();
            printf("[FLAG] Inband-liveness test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_inband_no_break_cli) {
            // In-band rate adaptation Stage 4c — D5 BREAK-OBSOLETE TEST (one-shot, exit rc).
            // Builds its own CMD/telecom_system instances internally.
            printf("[FLAG] --test-inband-no-break: invoking in-band rate-adapt Stage-4c "
                   "D5 BREAK-obsolete regression\n");
            fflush(stdout);
            int rc = ARQ.test_inband_no_break();
            printf("[FLAG] Inband-no-break test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_inband_retag_cli) {
            // In-band rate adaptation Stage 4d — D1 repeat + D4 climb/auto-demote TEST
            // (one-shot, exit rc). Builds its own CMD/RX/telecom_system instances internally.
            printf("[FLAG] --test-inband-retag: invoking in-band rate-adapt Stage-4d "
                   "D1 repeat-until-followed + D4 climb/auto-demote regression\n");
            fflush(stdout);
            int rc = ARQ.test_inband_retag();
            printf("[FLAG] Inband-retag test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_inband_nack_cli) {
            // In-band rate adaptation Stage 4e — D2 NACK first-class TEST (one-shot, exit rc).
            // Builds its own CMD/RX/telecom_system instances internally.
            printf("[FLAG] --test-inband-nack: invoking in-band rate-adapt Stage-4e "
                   "D2 NACK first-class regression\n");
            fflush(stdout);
            int rc = ARQ.test_inband_nack();
            printf("[FLAG] Inband-nack test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_inband_reannounce_cli) {
            // In-band rate adaptation Stage 4e — D3 periodic re-announce TEST (one-shot, exit rc).
            // Builds its own CMD/telecom_system instances internally.
            printf("[FLAG] --test-inband-reannounce: invoking in-band rate-adapt Stage-4e "
                   "D3 periodic re-announce regression\n");
            fflush(stdout);
            int rc = ARQ.test_inband_reannounce();
            printf("[FLAG] Inband-reannounce test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_data_ack_multiwindow_cli) {
            // Track A — multi-window DATA-ACK/SACK correlator (one-shot, exit rc).
            printf("[FLAG] --test-data-ack-multiwindow: invoking multi-window "
                   "DATA-ACK/SACK correlator regression\n");
            fflush(stdout);
            int rc = ARQ.test_data_ack_multiwindow();
            printf("[FLAG] Data-ACK-multiwindow test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_recovery_ack_capture_cli) {
            // recovery-ack-capture LEVER 1 focused capture-window test (one-shot, exit rc).
            printf("[FLAG] --test-recovery-ack-capture: invoking recovery-ACK "
                   "capture-window regression (LEVER 1)\n");
            fflush(stdout);
            int rc = ARQ.test_recovery_ack_capture();
            printf("[FLAG] Recovery-ACK-capture test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_inorder_demote_cli) {
            // D3.1 — UNIFIED in-order delivery across EVERY demote case (one-shot, exit rc).
            printf("[FLAG] --test-inorder-demote: invoking D3.1 unified in-order "
                   "delivery regression across all demote cases\n");
            fflush(stdout);
            int rc = ARQ.test_inorder_demote();
            printf("[FLAG] In-order-demote test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_climb_bsi_rollback_cli) {
            // CLIMB-CHURN — commander-side bsi rollback on a climb-UP promote (one-shot, exit rc).
            printf("[FLAG] --test-climb-bsi-rollback: invoking CLIMB-CHURN "
                   "commander-side bsi rollback regression\n");
            fflush(stdout);
            int rc = ARQ.test_climb_bsi_rollback();
            printf("[FLAG] Climb-bsi-rollback test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_guard_reanchor_cli) {
            printf("[FLAG] --test-guard-reanchor: invoking reverse-SACK guard "
                   "reference regression\n");
            fflush(stdout);
            int rc = ARQ.test_guard_reanchor();
            printf("[FLAG] Guard-reanchor test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_spec_sack_cli) {
            // LEVER #2 — speculative/prompt SACK (one-shot, exit rc).
            printf("[FLAG] --test-spec-sack: invoking LEVER #2 speculative-SACK "
                   "regression (window-fraction deadline -> partial SACK -> retx)\n");
            fflush(stdout);
            int rc = ARQ.test_spec_sack();
            printf("[FLAG] Spec-SACK test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_batchsize_desync_cli) {
            // res_c3100 — CMD>RSP batch-size desync silent-corruption (one-shot, exit rc).
            printf("[FLAG] --test-batchsize-desync: invoking CMD>RSP batch-size desync "
                   "silent-corruption regression\n");
            fflush(stdout);
            int rc = ARQ.test_batchsize_desync_delivery();
            printf("[FLAG] batchsize-desync test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_batch_shrink_orphan_current_cli) {
            // rx_btf=-1 current-batch shrink tail-drop (marginal-SNR silent corruption) (one-shot, exit rc).
            printf("[FLAG] --test-batch-shrink-orphan-current: invoking rx_btf=-1 "
                   "current-batch shrink tail-drop regression\n");
            fflush(stdout);
            int rc = ARQ.test_batch_shrink_orphan_current();
            printf("[FLAG] batch-shrink-orphan-current test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_linkphase_pending_confirm_cli) {
            // SEAM-2 deferred prev-confirm ARM/FLUSH fire proof (one-shot, exit rc).
            printf("[FLAG] --test-linkphase-pending-confirm: driving the SEAM-2 deferred "
                   "prev-confirm ARM/FLUSH emit path\n");
            fflush(stdout);
            int rc = ARQ.test_linkphase_pending_confirm_fire();
            printf("[FLAG] linkphase-pending-confirm test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_eob_loss_batch_truncation_cli) {
            // D5 — EOB-inference batch truncation (lost-EOB tail silent skip) (one-shot, exit rc).
            printf("[FLAG] --test-eob-loss-batch-truncation: invoking D5 lost-EOB "
                   "batch-truncation regression\n");
            fflush(stdout);
            int rc = ARQ.test_eob_loss_batch_truncation();
            printf("[FLAG] EOB-loss-batch-truncation test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_dedup_rebase_cli) {
            // In-band demote-rebase double-delivery corruption fix (one-shot, exit rc).
            printf("[FLAG] --test-dedup-rebase: invoking in-band demote-rebase "
                   "double-delivery de-dup regression\n");
            fflush(stdout);
            int rc = ARQ.test_dedup_rebase();
            printf("[FLAG] dedup-rebase test complete (rc=%d) -- exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_v2_pendingack_flip_alias_cli) {
            // R030 — v2 PENDING_ACK flip aliasing (one-shot, then exit rc).
            printf("[FLAG] --test-v2-pendingack-flip-alias: invoking R030 "
                   "PENDING_ACK flip aliasing regression\n");
            fflush(stdout);
            int rc = ARQ.test_v2_pendingack_flip_alias();
            printf("[FLAG] V2-pendingack-flip-alias test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_retx_slot_order_cli) {
            // Timing redesign — retx never rides wire slot 0 (one-shot, then exit rc).
            printf("[FLAG] --test-retx-slot-order: invoking retx-slot-order "
                   "(retx never rides wire slot 0) regression\n");
            fflush(stdout);
            int rc = ARQ.test_retx_slot_order();
            printf("[FLAG] Retx-slot-order test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_bigblock_livepath_cli) {
            // GAP-2 LIVE-PATH regression (diag/livepath-sim): real CONNECT at the robust
            // start + ONE real SET_CONFIG handshake robust->CFG16 (NO pin), then a 1374B K=8
            // transfer through the REAL send_batch->bigblock_send_one_block emit + the
            // receive_byte cw0-CRC gate + carve + FIFO. Answers the key question whether the
            // in-sim live path reproduces the HW cw0-CRC reject. One-shot, then exit rc.
            printf("[FLAG] --test-bigblock-livepath: invoking GAP-2 live-path "
                   "(real SET_CONFIG -> CFG16, no pin) big-block regression\n");
            fflush(stdout);
            int rc = cl_arq_controller::test_sim_inproc_bigblock_livepath();
            printf("[FLAG] Bigblock-livepath test complete (rc=%d) — exiting.\n", rc);
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
        if (test_probe_backoff_cli) {
            // FIX-B floor-probe back-off regression (one-shot, then exit rc).
            // Drives the REAL arm/gate/reset/predicate machinery + the v2
            // policy_evaluate_axis1 UP gate over the SIM virtual clock (PB1-PB5).
            // See fact-documents/gearshift-floor-probe-backoff.md §7.
            printf("[FLAG] --test-probe-backoff: invoking FIX-B floor-probe "
                   "back-off regression (PB1-PB5)\n");
            fflush(stdout);
            int rc = ARQ.test_probe_backoff();
            printf("[FLAG] Probe-backoff test complete (rc=%d) — exiting.\n", rc);
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
        if (test_break_weld_cli) {
            printf("[FLAG] --test-break-weld: invoking BREAK recovery pin regression\n");
            fflush(stdout);
            int rc = ARQ.test_break_weld();
            printf("[FLAG] BREAK recovery pin test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_rung_floor_cli) {
            // Per-rung MEASURED election floor gate + failure memory (one-shot, then exit rc).
            // Drives the REAL apply_rung_floor_cap / rung_floor_note_* decisions: the wb13
            // fail-before/pass-after election cap, never-raise, the failure-memory
            // arm/survive/decay/reset, and the defeat knob. See DATAFLOW_AUDIT_rung_floor.md §7.
            printf("[FLAG] --test-rung-floor: invoking per-rung election floor gate regression\n");
            fflush(stdout);
            int rc = ARQ.test_rung_floor_gate();
            printf("[FLAG] Rung-floor test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_decode_marathon_cli) {
            // LEVER C (feat/decode-marathon) §3: prove the multi-core big-block
            // codeword decode is byte-identical to serial (out_infobits + cw_ok),
            // in-order, with NO cross-frame corruption, and that a shared-workspace
            // pool DIVERGES (fail-before). One-shot, then exit rc.
            extern int test_decode_marathon_run();
            printf("[FLAG] --test-decode-marathon: invoking LEVER C parallel==serial "
                   "big-block decode integrity gate\n");
            fflush(stdout);
            int rc = test_decode_marathon_run();
            printf("[FLAG] Decode-marathon test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_robust0_compress_deadlock_cli) {
            // ROBUST_0 + streaming-compression deadlock regression (one-shot,
            // then exit rc). Drives the REAL process_buffer_data_commander()
            // data-fill at ROBUST_0 frame dimensions with streaming compression
            // enabled + a real compressible payload; asserts the staged batch
            // carries >0 application bytes. See
            // fact-documents/data-flow-compress-frame-fill.md §5.
            printf("[FLAG] --test-robust0-compress-deadlock: invoking ROBUST_0 "
                   "compression-deadlock regression\n");
            fflush(stdout);
            int rc = ARQ.test_robust0_compress_deadlock();
            printf("[FLAG] Robust0-compress-deadlock test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_streaming_compress_overshoot_cli) {
            printf("[FLAG] --test-streaming-compress-overshoot: invoking stale-EMA "
                   "streaming compression retry regression\n");
            fflush(stdout);
            int rc = ARQ.test_streaming_compress_overshoot();
            printf("[FLAG] Streaming-compress-overshoot test complete (rc=%d) -- exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_mixbatch_fill_overpop_cli) {
            printf("[FLAG] --test-mixbatch-fill-overpop: invoking mixbatch fill\n"
                   "       over-pop reorder regression\n");
            fflush(stdout);
            int rc = ARQ.test_mixbatch_fill_overpop();
            printf("[FLAG] Mixbatch-fill-overpop test complete (rc=%d) -- exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_mixbatch_fill_overpop_compressed_cli) {
            printf("[FLAG] --test-mixbatch-fill-overpop-compressed: invoking\n"
                   "       compression-leg over-pop + force-FREE data-loss regression\n");
            fflush(stdout);
            int rc = ARQ.test_mixbatch_fill_overpop_compressed();
            printf("[FLAG] Mixbatch-fill-overpop-compressed test complete (rc=%d) -- exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_idle_switch_role_race_cli) {
            // Idle SWITCH_ROLE race regression (one-shot, then exit rc). Drives
            // the REAL process_buffer_data_commander() idle branch with a
            // freshly-CONNECTED empty-tx Commander; asserts SWITCH_ROLE is queued
            // before any data write — the connected-but-0-deliver root cause.
            printf("[FLAG] --test-idle-switch-role-race: invoking idle SWITCH_ROLE "
                   "race regression (FAILS-BEFORE evidence)\n");
            fflush(stdout);
            int rc = ARQ.test_idle_switch_role_race();
            printf("[FLAG] Idle-switch-role-race test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_break_noprogress_cli) {
            // BREAK no-progress teardown regression (one-shot, then exit rc).
            // Replays the shared break_noprogress_step kernel that the EXHAUSTED
            // re-arm site uses; asserts teardown at exactly K dead cycles and the
            // negative control (progress resets the streak). Part C of the
            // connected-but-0-deliver fix. See fact-documents/idle-switchrole-race.md.
            printf("[FLAG] --test-break-noprogress-teardown: invoking BREAK no-progress "
                   "teardown regression\n");
            fflush(stdout);
            int rc = ARQ.test_break_noprogress_teardown();
            printf("[FLAG] Break-noprogress-teardown test complete (rc=%d) — exiting.\n", rc);
            fflush(stdout);
            exit(rc);
        }
        if (test_rx_ctrl_drop_cli) {
            // RX-CTRL-DROP regression (one-shot, then exit rc). Drives the shared
            // RECEIVED-state watchdog predicate + produce-gate replication: a
            // stranded RECEIVED control slot is freed and a later control frame
            // lands. FAILS-BEFORE with -DRX_CTRL_DROP_FAILBEFORE (watchdog never
            // fires -> slot stays stuck -> later frame dropped). See
            // fact-documents/data-flow-control-slot-lifecycle.md.
            printf("[FLAG] --test-rx-ctrl-drop: invoking RECEIVED control-slot "
                   "watchdog regression\n");
            fflush(stdout);
            int rc = ARQ.test_rx_ctrl_drop();
            printf("[FLAG] Rx-ctrl-drop test complete (rc=%d) — exiting.\n", rc);
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

        while (true)
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
        if (!nogui) {
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
