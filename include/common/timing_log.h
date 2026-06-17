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

// ===========================================================================
// Recovery-turnaround timing facility (env-gated MERCURY_TURNAROUND_TIMING).
//
// PURPOSE: pin the recovery-ACK turnaround timing mismatch EMPIRICALLY. Emits a
// HIGH-RES MONOTONIC microsecond timestamp (steady_clock) PLUS a UTC wall-clock
// microsecond anchor so the CMD and RSP processes' SEPARATE stdout logs can be
// aligned post-hoc. A role tag (CMD/RSP) distinguishes the two processes.
//
// DEFAULT-OFF / BYTE-IDENTICAL: every emitter below short-circuits on
// turnaround_timing_enabled(); when MERCURY_TURNAROUND_TIMING is unset the
// functions do nothing (no printf, no clock read) — the SFO-grid / climb-engine
// render output is byte-identical to base. This is purely ADDITIVE to the
// always-on Phase-D [T] events above (those are unchanged).
//
// LINE FORMATS (only when enabled):
//   [TT] role=<CMD|RSP> <event> mono_us=<N> utc_us=<N>\n
//   [TT] role=<CMD|RSP> <event> mono_us=<N> utc_us=<N> <kv-suffix>\n
//   [TT-ANCHOR] role=<CMD|RSP> mono_us=<N> utc_us=<N>\n   (cross-proc align pair)
//
// CROSS-PROCESS ALIGNMENT: each process has its OWN steady_clock epoch, so
// mono_us is NOT comparable across processes directly. utc_us (CLOCK-REALTIME
// microseconds since the Unix epoch) IS a shared wall clock. The anchor line
// emits BOTH on a cadence so the parser can fit mono_us<->utc_us per process and
// then place every event of both processes on ONE common UTC timeline. A shared
// wire event (BREAK/ACK round-trip) provides a second alignment cross-check.
// ===========================================================================

inline bool turnaround_timing_enabled() {
    // Cached once: getenv on every event would dominate the <1us emit cost and
    // (more importantly) keep the OFF path from being a single predictable
    // branch. Read once at first use.
    static const bool en = (getenv("MERCURY_TURNAROUND_TIMING") != nullptr);
    return en;
}

// Monotonic microseconds since this process's steady_clock epoch.
inline long long now_us() {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(
        now - epoch_ref()).count();
}

// UTC microseconds since the Unix epoch (shared wall clock for cross-process
// alignment). system_clock is the UTC clock on every platform we target.
inline long long utc_us() {
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(
        now.time_since_epoch()).count();
}

inline const char* role_tag(int role) {
    // COMMANDER==0, RESPONDER==1 (datalink_defines.h). Anything else -> "UNK".
    return (role == 0) ? "CMD" : (role == 1) ? "RSP" : "UNK";
}

// Periodic UTC<->monotonic anchor so the two processes' logs can be aligned.
inline void log_turn_anchor(int role) {
    if(!turnaround_timing_enabled()) return;
    printf("[TT-ANCHOR] role=%s mono_us=%lld utc_us=%lld\n",
        role_tag(role), now_us(), utc_us());
    fflush(stdout);
}

// Emit one recovery-turnaround event (no key-values).
inline void log_turn(int role, const char* event) {
    if(!turnaround_timing_enabled()) return;
    printf("[TT] role=%s %s mono_us=%lld utc_us=%lld\n",
        role_tag(role), event, now_us(), utc_us());
    fflush(stdout);
}

// Emit one recovery-turnaround event with a printf-style key-value suffix.
inline void log_turn_kv(int role, const char* event, const char* fmt, ...) {
    if(!turnaround_timing_enabled()) return;
    printf("[TT] role=%s %s mono_us=%lld utc_us=%lld ",
        role_tag(role), event, now_us(), utc_us());
    va_list ap; va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("\n");
    fflush(stdout);
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
