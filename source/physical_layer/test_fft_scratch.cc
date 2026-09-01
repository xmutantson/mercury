// Persistent FFT execution-scratch allocation regression.
//
// The OFDM receive and idle-detection hot path runs many same-size complex FFTs
// through cached pocketfft plans (see fft_plan_cache in source/physical_layer/
// ofdm.cc, which builds one pocketfft_c<double> plan per size and re-executes it).
// Each pocketfft butterfly pass historically allocated a fresh execution buffer
// from the heap, so a receiver doing nothing but listening churned on the order
// of 1e5 heap allocations per second. Reusing one thread-local execution buffer
// per pass makes the steady-state execute path allocation free after warmup.
//
// This test drives the real pocketfft_c<double>::exec entry point -- the same
// class and call the OFDM layer caches -- installs pocketfft's optional heap
// allocation observer, and asserts that no heap allocation occurs on the execute
// path once the plan and scratch are warm. It FAILS on the per-pass-allocation
// path (one allocation per pass) and PASSES with the persistent scratch. It is
// deterministic, single-threaded, and hardware free.

#include <atomic>
#include <cstddef>
#include <cstdio>
#include <vector>

#include "physical_layer/pocketfft_hdronly.h"

namespace {

std::atomic<std::size_t> g_exec_allocs{0};

void observe_alloc(std::size_t /*nbytes*/)
{
    g_exec_allocs.fetch_add(1, std::memory_order_relaxed);
}

// Count heap allocations issued on the pocketfft execute path over `iters`
// forward+inverse passes at a fixed size, after `warmup` passes. Returns the
// count observed strictly between observer install and removal.
std::size_t exec_path_allocs(std::size_t n, int warmup, int iters)
{
    using pocketfft::detail::pocketfft_c;
    using pocketfft::detail::cmplx;

    pocketfft_c<double> plan(n);              // built once, as ofdm.cc caches it
    std::vector<cmplx<double>> buf(n);
    for (std::size_t i = 0; i < n; ++i) {
        buf[i].r = double(i % 17) - 8.0;
        buf[i].i = double((i * 3) % 13) - 6.0;
    }

    for (int w = 0; w < warmup; ++w) {        // settle plan + persistent scratch
        plan.exec(buf.data(), 1.0, true);
        plan.exec(buf.data(), 1.0, false);
    }

    pocketfft::detail::heap_alloc_observer() = &observe_alloc;
    g_exec_allocs.store(0, std::memory_order_relaxed);
    for (int i = 0; i < iters; ++i) {
        plan.exec(buf.data(), 1.0, true);
        plan.exec(buf.data(), 1.0, false);
    }
    pocketfft::detail::heap_alloc_observer() = nullptr;

    return g_exec_allocs.load(std::memory_order_relaxed);
}

} // namespace

int run_fft_scratch_tests()
{
    const int warmup = 4;
    const int iters  = 1000;
    // Power-of-two lengths route cfftp::pass_all through the specialized radix
    // passes, whose only per-call heap buffer is the execution scratch this change
    // makes persistent -- so this isolates exactly the buffer under test. (Lengths
    // with a prime factor > 11 additionally allocate a per-call roots table inside
    // passg, which is a separate buffer outside this change's scope; the OFDM hot
    // path uses power-of-two transforms.)
    const std::size_t sizes[] = {256, 512, 1024};

    std::size_t total = 0;
    for (std::size_t n : sizes) {
        std::size_t a = exec_path_allocs(n, warmup, iters);
        total += a;
        std::printf("[TEST-FFT-SCRATCH-PERSISTENT] size=%zu per_call_allocs=%zu passes=%d\n",
                    n, a, iters * 2);
    }

    bool pass = (total == 0);
    std::printf("[TEST-FFT-SCRATCH-PERSISTENT] %s total_exec_allocs=%zu\n",
                pass ? "PASS" : "FAIL", total);
    return pass ? 0 : 1;
}
