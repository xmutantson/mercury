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
            failed += run_sim_clock_tests();
            failed += run_winlink_dict_tests();
            // In-band down-ladder DELIVERY regression (BREAK-orphan + silent-snapshot). A
            // member test on a throwaway controller (its own buffers; PART B builds its own
            // minimal telecom_system). Fast + deterministic, no IONOS/RF. data-flow-inband-
            // downladder.md §3/§5.3.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_inband_downladder();
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
            // §21: CONFIG_0-START robust-floor OVER-SEAT (the uncovered sibling of FIX #1e). A session
            // that starts at CONFIG_0 (no robust->OFDM adopt) never latches inband_ofdm_acq_ring_shrunk,
            // so inband_seat_robust_ring_floor over-grows the natural OFDM ring (217->~804) -> every
            // preamble at the tail beyond upper_bound -> 0 forward decode. Member test on a throwaway
            // controller (builds its own telecom_system). data-flow-robust-ofdm-adopt-flush.md §21.
            {
                cl_arq_controller test_arq;
                failed += test_arq.test_inband_config0_start_ring();
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
                failed += ARQ_isr.test_break_noprogress_teardown();
            }
            // CONNECT-REACK T4 (connect-testack-handshake.md §5): in-process
            // synthetic-fire unit for the duplicate-TEST_CONNECTION re-ACK
            // pre-data window. No PHY/audio -> permanent regression gate.
            {
                cl_arq_controller ARQ_reack;
                failed += ARQ_reack.test_connect_reack();
            }
            // CONNECT-REACK FTR-STARVATION gate (connect-testack-handshake.md
            // §3.3): the 8e62722e regression that dropped OFDM data delivery to 0
            // (the re-ACK pinned frames_to_read=2 across the data phase). Drives
            // the REAL ftr-arbiter; asserts a connect-heal never starves the OFDM
            // data-acquisition path. Permanent regression gate — this class
            // slipped past --test before because the old unit only checked the
            // predicate boolean, not the shared ftr the data path consumes.
            {
                cl_arq_controller ARQ_reack_ftr;
                failed += ARQ_reack_ftr.test_connect_reack_ftr_starvation();
            }
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
        // --test-connect-reack : run ONLY the CONNECT-REACK T4 in-process unit
        // (duplicate-TEST_CONNECTION pre-data re-ACK) and exit. Fast +
        // deterministic; see arq_responder.cc::test_connect_reack().
        if (strcmp(argv[i], "--test-connect-reack") == 0) {
            cl_arq_controller ARQ_reack;
            int failed = ARQ_reack.test_connect_reack();
            return (failed == 0) ? 0 : 1;
        }
        // --test-reack-ftr-starvation : run ONLY the CONNECT-REACK FTR-STARVATION
        // regression (8e62722e: OFDM data delivery dropped to 0 because the
        // re-ACK pinned frames_to_read=2 across the data phase) and exit. Fast +
        // deterministic; see arq_responder.cc::test_connect_reack_ftr_starvation().
        if (strcmp(argv[i], "--test-reack-ftr-starvation") == 0) {
            cl_arq_controller ARQ_reack_ftr;
            int failed = ARQ_reack_ftr.test_connect_reack_ftr_starvation();
            return (failed == 0) ? 0 : 1;
        }
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
        // --test-winlink-dict : run ONLY the Winlink dict priming + version-lock
        // regression suite and exit. Fast + deterministic; drives the production
        // cl_compressor end-to-end (primed lift, bit-exact, version-mismatch
        // fail-safe, kill-switch, bulk no-regression, attachment inertness,
        // streaming desync). See source/compression/test_winlink_dict.cc.
        if (strcmp(argv[i], "--test-winlink-dict") == 0) {
            int failed = run_winlink_dict_tests();
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
    bool test_retx_clear_on_recovery_cli = false; // --test-retx-clear-on-recovery: R029 — stale retx queue
                                        // cleared on recovery. Drives the REAL clear_retx_queue(); asserts the queue empties
                                        // of pre-recovery bsi, is idempotent, and repeatable. One-shot, exits rc.
    bool test_rx_drain_backpressure_cli = false; // --test-rx-drain-backpressure: FIX-6 — RX-delivery drain
                                        // must NOT drop popped bytes when the non-blocking app socket back-pressures.
                                        // FAILS at 62cb3dc (the 61,621-byte stall), PASSES after. One-shot, exits rc.
    bool test_gap_abort_cli = false;    // --test-gap-abort: FIX-8 — silent lost-batch GAP on post-reset re-adopt.
                                        // Reproduces bench-4 (deliver 0-4, BREAK reset with 5-7 undelivered, present 8):
                                        // fail-before via MERCURY_GAP_ABORT_DEFEAT=1 (silent concat), pass-after aborts
                                        // loudly + delivers EXACTLY batches 0-4. One-shot, exits rc.
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
    bool test_inband_deliver_cli = false;  // --test-inband-deliver: forward-healthy reverse-ACK miss -> NO-BREAK deliver regression.
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
    bool test_data_ack_multiwindow_cli = false; // --test-data-ack-multiwindow: Track A — multi-window
                                        // DATA-ACK/SACK correlator. Synthesizes a real ACK+SACK burst at an
                                        // OLDER ring phase with a silent newest tail: fail-before (newest-tail
                                        // decode MISSES), pass-after (mw_find_ack_sack_phase recovers it with
                                        // CRC12 pass), no-false-accept on pure silence. One-shot, exits rc.
                                        // See fact-documents/data-flow-data-ack-sack-correlator.md §7.
    bool test_inorder_demote_cli = false; // --test-inorder-demote: D3.1 — UNIFIED in-order delivery
                                        // across EVERY demote case (BREAK + the 4 SET_CONFIG-only demotes +
                                        // PREV-BUMP strand). fail-before via MERCURY_GAP_ABORT_DEFEAT=1
                                        // (silent concat on the SET_CONFIG cases), pass-after aborts loudly
                                        // OR delivers contiguous. One-shot, exits rc.
                                        // See bigblock_p3_hw/_d31_fade/D31_INORDER_DESIGN.md.
    bool test_spec_sack_cli = false;    // --test-spec-sack: LEVER #2 — speculative/prompt SACK. Frame-k still-decoding
                                        // at the window-fraction deadline -> in-window SACK bit_k=0 -> CMD retx ->
                                        // byte-faithful re-receive -> single in-order delivery, no silent loss.
                                        // fail-before via env-off (stall reproduced), pass-after via MERCURY_SPEC_SACK=1.
                                        // One-shot, exits rc. See fact-documents/turnaround-eff.md §8/§9.
    bool test_eob_loss_batch_truncation_cli = false; // --test-eob-loss-batch-truncation: D5 — EOB-inference
                                        // batch truncation. A 30-frame batch loses its EOB-marked tail; the
                                        // wired batch_total_frames recovers the true length. fail-before via
                                        // MERCURY_D5_INFER_DEFEAT=1 (silent 29-frame skip), pass-after holds
                                        // then delivers all 30 faithfully after the retx. One-shot, exits rc.
                                        // See TRACK_C_D2D3D5_DESIGN.md §5.3 / data-flow-prev-bump.md §8.
    bool test_v2_pendingack_flip_alias_cli = false; // --test-v2-pendingack-flip-alias: R030 — v2 PENDING_ACK
                                        // flip aliasing. Diverged index/wire space; drives the REAL v2_flip_resolve_slot();
                                        // asserts retx skipped + new-data -> correct slot + no FREE/foreign PENDING_ACK. One-shot, exits rc.
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
    bool test_robust0_compress_deadlock_cli = false; // --test-robust0-compress-deadlock: ROBUST_0+streaming-compression
                                        // deadlock regression. Drives the REAL process_buffer_data_commander() data-fill
                                        // at ROBUST_0 (max_frame==7==COMPRESS_HEADER_SIZE) with streaming compression +
                                        // a real compressible payload; asserts >0 application bytes are staged. FAILS on
                                        // fef293f (every batch stages 0 payload → 0 throughput). See
                                        // fact-documents/data-flow-compress-frame-fill.md §5.
    bool test_cumulative_ack_cli = false; // --test-cumulative-ack: Tier-2 cumulative-n_r self-heal/gap-invariant/cap-gate regression (data-flow-forgiving-ack.md §T2.6).
    bool test_a3_decouple_safety_cli = false; // --test-a3-decouple-safety: the §2 CHECKPOINT — single-miss non-load-bearing (anti-0-bytes, byte-faithful) + genuine-death net intact, demote IN PLACE (data-flow-forgiving-ack.md §T2.2/§6).
    bool test_pas_cli = false;          // --test-pas: PAS/PCS distribution-matcher bijection + histogram self-test (feat/pcs).
    bool test_cfg17_cli = false;        // --test-cfg17: CFG17 shaped-64-QAM composition (PAS+TINTERP-seed+ratio-nvfix) failing-first (feat/cfg17).
    bool test_decode_marathon_cli = false; // --test-decode-marathon: LEVER C parallel==serial big-block decode integrity (decode-marathon-C.md §8).
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
        else if (strcmp(argv[i], "--test-retx-clear-on-recovery") == 0)
        {
            // R029 — stale retx queue cleared on recovery regression — one-shot
            // at startup, then exit with the test's rc. See
            // fact-documents/data-flow-arq-recovery-cluster.md §4.1 / §5.1.
            test_retx_clear_on_recovery_cli = true;
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
        else if (strcmp(argv[i], "--test-pas") == 0)
        {
            // PAS/PCS distribution-matcher bijection + histogram self-test (feat/pcs).
            // Pure cl_dist_matcher check, no Mercury/ARQ state needed. See
            // fact-documents/data-flow-pas-shaping.md §6 + tools/test_pas_shaping.py.
            test_pas_cli = true;
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
        else if (strcmp(argv[i], "--test-v2-pendingack-flip-alias") == 0)
        {
            // R030 — v2 PENDING_ACK flip aliasing regression — one-shot at
            // startup, then exit with the test's rc. See
            // fact-documents/data-flow-arq-recovery-cluster.md §4.2 / §5.5.
            test_v2_pendingack_flip_alias_cli = true;
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
        else if (strcmp(argv[i], "--test-robust0-compress-deadlock") == 0)
        {
            // ROBUST_0 + streaming-compression deadlock regression — one-shot at
            // startup, then exit with the test's rc. See
            // fact-documents/data-flow-compress-frame-fill.md §5.
            test_robust0_compress_deadlock_cli = true;
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
        printf("SIM software channel (device-free ARQ loopback via relay)\n");
        printf("[SIM] virtual clock ENABLED — control-loop timers run on "
               "channel-sample time, not wall-clock\n");
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
