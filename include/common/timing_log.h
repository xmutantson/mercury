/*
 * timing_log.h — Phase-D instrumentation for batch-cycle profiling.
 *
 * Provides monotonic absolute-timestamp helpers so the same call site on
 * CMD and RSP can be cross-correlated by a parser. Always-on (no flag).
 * Per-event cost is one printf + steady_clock::now() ≈ <10 us.
 *
 * Emits lines of the form:
 *   [T] <event_name> abs_ms=<N> [k1=v1 k2=v2 ...]
 *
 * See mercury/fact-documents/PI_VS_HOST_DEEP_ANALYSIS_PLAN.md.
 */
#ifndef INC_TIMING_LOG_H_
#define INC_TIMING_LOG_H_

#include <chrono>
#include <cstdio>
#include <cstdarg>
#include <cstdlib>

namespace mtl {  // mercury timing log

inline std::chrono::steady_clock::time_point& epoch_ref() {
    static auto e = std::chrono::steady_clock::now();
    return e;
}

inline long long now_ms() {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        now - epoch_ref()).count();
}

// SIM-SWEEP quiet gate (gearshift settling-time WGN sweep). mtl::log_event* is
// "always-on" and does a printf + fflush PER EVENT (many per batch). In the
// faster-than-real-time 2-instance in-process sim that per-event fflush syscall
// is the dominant WALL-clock cost (it dwarfs the actual DSP). MERCURY_SIM2_QUIET=1
// suppresses these timing events so a settling-time sweep finishes in seconds.
// Cached on first call (no getenv() in the hot path). Additive + env-gated:
// default (unset) is byte-identical to prior always-on behaviour; the gate only
// fires in the SIM_INPROC sweep, which sets the env.
inline bool quiet_enabled() {
    static int q = -1;
    if (q < 0) q = (std::getenv("MERCURY_SIM2_QUIET") != nullptr) ? 1 : 0;
    return q != 0;
}

// Emit:  [T] <event> abs_ms=<N>\n
inline void log_event(const char* event) {
    if (quiet_enabled()) return;
    printf("[T] %s abs_ms=%lld\n", event, now_ms());
    fflush(stdout);
}

// Emit:  [T] <event> abs_ms=<N> <fmt-formatted suffix>\n
inline void log_event_kv(const char* event, const char* fmt, ...) {
    if (quiet_enabled()) return;
    printf("[T] %s abs_ms=%lld ", event, now_ms());
    va_list ap; va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("\n");
    fflush(stdout);
}

}  // namespace mtl

#endif  // INC_TIMING_LOG_H_
