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

// Emit:  [T] <event> abs_ms=<N>\n
inline void log_event(const char* event) {
    printf("[T] %s abs_ms=%lld\n", event, now_ms());
    fflush(stdout);
}

// Emit:  [T] <event> abs_ms=<N> <fmt-formatted suffix>\n
inline void log_event_kv(const char* event, const char* fmt, ...) {
    printf("[T] %s abs_ms=%lld ", event, now_ms());
    va_list ap; va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("\n");
    fflush(stdout);
}

}  // namespace mtl

#endif  // INC_TIMING_LOG_H_
