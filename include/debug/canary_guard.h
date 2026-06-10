#ifndef CANARY_GUARD_H_
#define CANARY_GUARD_H_

// Canary guard system — RE-ARMED (Phase 1.5 of the O3 intermittent-crash hunt).
//
// The Feb-2026 O3 crash class that UBSan + _GLIBCXX_ASSERTIONS CANNOT see is an
// out-of-bounds / use-after-free on the RAW heap buffers Mercury allocates with
// CNEW (new T[]): passband_delayed_data, baseband_data_interpolated, the
// ldpc/ofdm scratch arrays, etc. (data_container.cc, ldpc.cc, ofdm.cc). ASan is
// unavailable on this MinGW box (no libasan), so this guard is the detector for
// that class.
//
// HOW: every CNEW over-allocates CANARY_PAD extra elements at the TAIL of the
// block and stamps a known byte pattern (0x5A...) into that slack. The base
// pointer (offset 0) is returned UNCHANGED, so the [0..count) payload view and
// `delete[]` semantics are identical to the passthrough macro — call sites need
// no changes and the freed pointer is still the original new[] pointer. A
// registry records {base, total_bytes, payload_bytes, name}. canary_check_all()
// re-reads every block's tail slack: if the modem wrote past the end of a
// buffer (the predicted Feb crash), the canary bytes are clobbered and we print
// [CANARY] OOB <name> ... loudly with the buffer name. CDELETE verifies the
// canary one last time, then POISONS the freed payload with 0xDD (a freed-memory
// pattern) so a use-after-free that reads through the dangling pointer before
// the allocator reuses the page sees an obvious sentinel (NaN-ish doubles /
// 0xdddddddd ints) instead of plausible stale data — and the next
// canary_check_all over a still-registered-then-unregistered block can't false
// match. The registry is mutex-guarded because load_configuration (producer:
// CNEW/CDELETE) can interleave with process_main (consumer: canary_check_all)
// across threads in the live -x sim / -m ARQ build.
//
// Set CANARY_ABORT=1 in the environment to abort() at the first corrupted
// canary (for a pinpoint backtrace under gdb). Default = log-and-continue so one
// run surfaces every corrupted buffer.
//
// Defaults to ARMED. Define CANARY_GUARD_DISABLE to fall back to passthrough
// (zero overhead) for production builds.

#include <cstdlib>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <mutex>

#ifdef CANARY_GUARD_DISABLE
// -------- passthrough (production) --------
#define CNEW(type, count, name) (new type[(size_t)(count)])
#define CDELETE(ptr) do { if(ptr) { delete[] (ptr); (ptr) = NULL; } } while(0)
inline int canary_check_all() { return 0; }
inline void canary_clear() {}
#else
// -------- armed (instrumented) --------

// Extra TAIL elements stamped with the canary pattern. 16 elements is large
// enough that a small forward over-write (the predicted mirror-write past the
// 2*sp boundary in audioio.c, or an off-by-N in an extraction slice) always
// lands in the guarded region for the worst-case element type (>= 16 bytes for
// std::complex<double>) -> at least 256 guarded tail bytes.
#define CANARY_PAD 16
#define CANARY_BYTE 0x5A          // 'Z' — distinctive in a hex dump
#define CANARY_FREE_BYTE 0xDD     // freed-payload poison

#define CANARY_MAX 4096

struct canary_entry {
    const void* base;             // the new[] base pointer (what we return)
    size_t total_bytes;           // payload + CANARY_PAD*elem_size
    size_t payload_bytes;         // user-visible [0..count) bytes
    const char* name;
    int live;                     // 1 = registered, 0 = free slot
};

inline std::mutex& canary_mutex() { static std::mutex m; return m; }
inline canary_entry* canary_table() { static canary_entry t[CANARY_MAX]; return t; }
inline int& canary_count() { static int n = 0; return n; }
inline int& canary_oob_total() { static int n = 0; return n; }

inline int canary_should_abort() {
    static int v = -1;
    if (v < 0) { const char* e = getenv("CANARY_ABORT"); v = (e && *e && *e != '0') ? 1 : 0; }
    return v;
}

// Verify the tail slack of one entry. Returns the count of clobbered tail bytes.
inline size_t canary_verify_entry(const canary_entry& e) {
    const unsigned char* tail =
        (const unsigned char*)e.base + e.payload_bytes;
    size_t guard = e.total_bytes - e.payload_bytes;
    size_t bad = 0;
    for (size_t i = 0; i < guard; ++i)
        if (tail[i] != (unsigned char)CANARY_BYTE) ++bad;
    return bad;
}

// Register an allocation and stamp its tail canary. elem_size and count are the
// requested element size and element count; we already over-allocated PAD more.
inline void canary_register(void* base, size_t elem_size, size_t count) {
    // (name is stamped by the macro after this call via canary_set_name)
    unsigned char* tail = (unsigned char*)base + elem_size * count;
    memset(tail, CANARY_BYTE, elem_size * CANARY_PAD);
    std::lock_guard<std::mutex> lk(canary_mutex());
    canary_entry* t = canary_table();
    int n = canary_count();
    int slot = -1;
    for (int i = 0; i < n; ++i) if (!t[i].live) { slot = i; break; }
    if (slot < 0) { if (n >= CANARY_MAX) return; slot = n; canary_count() = n + 1; }
    t[slot].base = base;
    t[slot].payload_bytes = elem_size * count;
    t[slot].total_bytes = elem_size * (count + CANARY_PAD);
    t[slot].name = "(unnamed)";
    t[slot].live = 1;
}

inline void canary_set_name(void* base, const char* name) {
    std::lock_guard<std::mutex> lk(canary_mutex());
    canary_entry* t = canary_table();
    int n = canary_count();
    for (int i = 0; i < n; ++i)
        if (t[i].live && t[i].base == base) { t[i].name = name; return; }
}

// CNEW: over-allocate PAD tail elements, stamp the canary, register, return base.
// Wrapped in a helper template so the macro can stay a single expression and the
// returned type is exactly type* (so delete[] and indexing are unchanged).
template <typename T>
inline T* canary_new(size_t count, const char* name) {
    T* p = new T[count + CANARY_PAD];
    canary_register((void*)p, sizeof(T), count);
    canary_set_name((void*)p, name);
    return p;
}

#define CNEW(type, count, name) (canary_new<type>((size_t)(count), (name)))

// Verify + unregister + poison + free one pointer.
inline void canary_free(void* base) {
    if (!base) return;
    std::lock_guard<std::mutex> lk(canary_mutex());
    canary_entry* t = canary_table();
    int n = canary_count();
    for (int i = 0; i < n; ++i) {
        if (t[i].live && t[i].base == base) {
            size_t bad = canary_verify_entry(t[i]);
            if (bad) {
                ++canary_oob_total();
                fprintf(stderr,
                    "[CANARY] OOB on free: buffer '%s' base=%p payload=%zu "
                    "tail-bytes-clobbered=%zu\n",
                    t[i].name, base, t[i].payload_bytes, bad);
                fflush(stderr);
                if (canary_should_abort()) {
                    fprintf(stderr, "[CANARY] CANARY_ABORT=1 -> abort at OOB\n");
                    fflush(stderr); abort();
                }
            }
            // Poison the payload so a UAF read through the dangling pointer
            // sees an obvious sentinel before the allocator reuses the page.
            memset(base, CANARY_FREE_BYTE, t[i].payload_bytes);
            t[i].live = 0;
            t[i].base = NULL;
            return;
        }
    }
    // base not registered (shouldn't happen for CNEW'd pointers) — nothing to do.
}

// CDELETE: verify+poison via the registry, then delete[] the ORIGINAL pointer
// (which is base, unchanged) and null the caller's pointer.
#define CDELETE(ptr) do { \
    if (ptr) { canary_free((void*)(ptr)); delete[] (ptr); (ptr) = NULL; } \
} while(0)

// Walk every live block and verify its tail canary. Returns number of corrupt
// buffers found. Called from process_main (after every receive_byte) and deinit.
inline int canary_check_all() {
    std::lock_guard<std::mutex> lk(canary_mutex());
    canary_entry* t = canary_table();
    int n = canary_count();
    int corrupt = 0;
    for (int i = 0; i < n; ++i) {
        if (!t[i].live) continue;
        size_t bad = canary_verify_entry(t[i]);
        if (bad) {
            ++corrupt;
            ++canary_oob_total();
            fprintf(stderr,
                "[CANARY] OOB: buffer '%s' base=%p payload=%zu "
                "tail-bytes-clobbered=%zu\n",
                t[i].name, t[i].base, t[i].payload_bytes, bad);
            fflush(stderr);
            if (canary_should_abort()) {
                fprintf(stderr, "[CANARY] CANARY_ABORT=1 -> abort at OOB\n");
                fflush(stderr); abort();
            }
        }
    }
    return corrupt;
}

// Reset the registry (deinit path clears it after freeing all blocks).
inline void canary_clear() {
    std::lock_guard<std::mutex> lk(canary_mutex());
    // Do NOT zero live entries blindly — deinit frees via CDELETE which already
    // unregisters. Anything still 'live' here leaked or was double-tracked; drop
    // it so the table doesn't grow unbounded across config switches.
    canary_entry* t = canary_table();
    int n = canary_count();
    for (int i = 0; i < n; ++i) t[i].live = 0;
}

#endif // CANARY_GUARD_DISABLE
#endif // CANARY_GUARD_H_
