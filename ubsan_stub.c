/* ubsan_stub.c — minimal libubsan replacement for MinGW/GCC (no libubsan ships
 * with Strawberry/MSYS2 GCC on this box). Implements the __ubsan_handle_* ABI so
 * a -fsanitize=undefined build links AND prints the source file:line:col of every
 * UB hit, instead of an opaque trap. Handlers LOG-AND-CONTINUE (recover) so one
 * run surfaces ALL distinct UB sites, not just the first.
 *
 * SourceLocation is the first member of every *Data handler struct (stable ABI
 * since clang 3.x / gcc 4.9): { const char* filename; uint32_t line; uint32_t col }.
 * We read only that; the rest of each Data struct is ignored (safe — we never
 * dereference past it).
 *
 * Set UBSAN_ABORT=1 in the environment to make the FIRST hit abort (for
 * pinpoint backtraces under gdb). Default = log-and-continue.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct { const char* filename; uint32_t line; uint32_t column; } UbSrcLoc;

static int g_ubsan_count = 0;
static int g_ubsan_abort = -1;            /* -1 = uninit, read env lazily */
/* De-dup table: cap at 4096 distinct (kind,file,line) tuples to bound output. */
#define UB_DEDUP_MAX 8192
static struct { const char* file; uint32_t line; const char* kind; int hits; }
    g_seen[UB_DEDUP_MAX];
static int g_seen_n = 0;

static int ub_should_abort(void){
    if (g_ubsan_abort < 0){
        const char* e = getenv("UBSAN_ABORT");
        g_ubsan_abort = (e && *e && *e != '0') ? 1 : 0;
    }
    return g_ubsan_abort;
}

static void ub_report(const char* kind, UbSrcLoc* loc){
    g_ubsan_count++;
    const char* file = (loc && loc->filename) ? loc->filename : "<no-file>";
    uint32_t line = loc ? loc->line : 0;
    uint32_t col  = loc ? loc->column : 0;
    /* de-dup so a hot UB site doesn't flood the log; tally hit count */
    int i;
    for (i = 0; i < g_seen_n; i++){
        if (g_seen[i].line == line && g_seen[i].kind == kind &&
            g_seen[i].file == file){
            g_seen[i].hits++;
            /* still print first 3 of each site so it shows in the per-run log */
            if (g_seen[i].hits <= 3)
                fprintf(stderr, "[UBSAN] %s at %s:%u:%u (hit %d)\n",
                        kind, file, line, col, g_seen[i].hits);
            return;
        }
    }
    if (g_seen_n < UB_DEDUP_MAX){
        g_seen[g_seen_n].file = file;
        g_seen[g_seen_n].line = line;
        g_seen[g_seen_n].kind = kind;
        g_seen[g_seen_n].hits = 1;
        g_seen_n++;
    }
    fprintf(stderr, "[UBSAN] %s at %s:%u:%u\n", kind, file, line, col);
    fflush(stderr);
    if (ub_should_abort()){
        fprintf(stderr, "[UBSAN] UBSAN_ABORT=1 -> aborting at first hit\n");
        fflush(stderr);
        abort();
    }
}

/* Print a summary tally at process exit (atexit). */
__attribute__((destructor))
static void ub_summary(void){
    if (g_ubsan_count == 0) return;
    fprintf(stderr, "\n[UBSAN-SUMMARY] %d total hits across %d distinct sites:\n",
            g_ubsan_count, g_seen_n);
    int i;
    for (i = 0; i < g_seen_n; i++)
        fprintf(stderr, "  %5dx  %-28s %s:%u\n",
                g_seen[i].hits, g_seen[i].kind, g_seen[i].file, g_seen[i].line);
    fflush(stderr);
}

/* Recoverable handlers (log + continue). GCC calls these when -fsanitize-recover
 * is in effect (the default for most checks). */
#define H1(name)  void __ubsan_handle_##name(void* d){ ub_report(#name,(UbSrcLoc*)d); }
#define H2(name)  void __ubsan_handle_##name(void* d, void* a){ (void)a; ub_report(#name,(UbSrcLoc*)d); }
#define H3(name)  void __ubsan_handle_##name(void* d, void* a, void* b){ (void)a;(void)b; ub_report(#name,(UbSrcLoc*)d); }
/* Non-recoverable (_abort) variants: GCC emits these for checks compiled with
 * -fno-sanitize-recover. We log then abort (so the trap is at the UB site). */
#define H1A(name) void __ubsan_handle_##name##_abort(void* d){ ub_report(#name "!",(UbSrcLoc*)d); abort(); }
#define H2A(name) void __ubsan_handle_##name##_abort(void* d, void* a){ (void)a; ub_report(#name "!",(UbSrcLoc*)d); abort(); }
#define H3A(name) void __ubsan_handle_##name##_abort(void* d, void* a, void* b){ (void)a;(void)b; ub_report(#name "!",(UbSrcLoc*)d); abort(); }

H3(add_overflow)        H3A(add_overflow)
H3(sub_overflow)        H3A(sub_overflow)
H3(mul_overflow)        H3A(mul_overflow)
H2(negate_overflow)     H2A(negate_overflow)
H3(divrem_overflow)     H3A(divrem_overflow)
H3(shift_out_of_bounds) H3A(shift_out_of_bounds)
H2(out_of_bounds)       H2A(out_of_bounds)
H1(builtin_unreachable)
H1(missing_return)
H2(vla_bound_not_positive) H2A(vla_bound_not_positive)
H2(load_invalid_value)  H2A(load_invalid_value)
H2(nonnull_arg)
H3(pointer_overflow)    H3A(pointer_overflow)
H2(alignment_assumption) H2A(alignment_assumption)

/* type_mismatch v1: (Data*, void* ptr). The Data layout is
 * { SrcLoc; TypeDescriptor* type; unsigned char log_alignment; unsigned char kind }.
 * We only read SrcLoc (first member). The pointer arg tells null vs misaligned
 * vs insufficient-size; we surface the raw pointer to disambiguate. */
void __ubsan_handle_type_mismatch_v1(void* d, void* ptr){
    UbSrcLoc* loc = (UbSrcLoc*)d;
    /* Distinguish null-deref (ptr==0) from misaligned in the kind string. */
    ub_report(ptr ? "type_mismatch(misalign/size)" : "type_mismatch(null-deref)", loc);
}
void __ubsan_handle_type_mismatch_v1_abort(void* d, void* ptr){
    UbSrcLoc* loc = (UbSrcLoc*)d;
    ub_report(ptr ? "type_mismatch(misalign/size)!" : "type_mismatch(null-deref)!", loc);
    abort();
}

void __ubsan_handle_invalid_builtin(void* d, unsigned char k){ (void)k; ub_report("invalid_builtin",(UbSrcLoc*)d); }
void __ubsan_handle_invalid_builtin_abort(void* d, unsigned char k){ (void)k; ub_report("invalid_builtin!",(UbSrcLoc*)d); abort(); }

/* function_type_mismatch (indirect call through wrong function-pointer type). */
void __ubsan_handle_function_type_mismatch(void* d, void* fn){ (void)fn; ub_report("function_type_mismatch",(UbSrcLoc*)d); }

/* vptr / dynamic-type (-fsanitize=vptr). We disable vptr at compile time, but
 * provide the symbols too in case any TU still references them. The global
 * type-cache the instrumentation hashes into; size is arbitrary (256 slots). */
void* __ubsan_vptr_type_cache[256];
void __ubsan_handle_dynamic_type_cache_miss(void* d, void* ptr, void* hash){
    (void)ptr; (void)hash; ub_report("dynamic_type_cache_miss",(UbSrcLoc*)d);
}

int ubsan_total_hits(void){ return g_ubsan_count; }
