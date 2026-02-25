/*
 * Mercury block compression — adaptive dual-algorithm (PPMd + zstd).
 *
 * Each compressed block carries a 5-byte header:
 *   [algo:1][comp_size:2 LE][orig_size:2 LE][payload...]
 *
 * algo = 0x00 raw, 0x01 PPMd, 0x02 zstd.
 * A quick Shannon entropy test steers algorithm selection:
 *   entropy > 7.5  → skip compression (send raw)
 *   entropy > 6.0  → try zstd only
 *   entropy <= 6.0 → try both PPMd and zstd, pick smaller
 * Raw is always the fallback if compression doesn't shrink the data.
 */

#ifndef MERCURY_COMPRESS_H
#define MERCURY_COMPRESS_H

#include <cstdint>
#include <cstring>

#define COMPRESS_HEADER_SIZE    5
#define COMPRESS_ALGO_RAW       0x00
#define COMPRESS_ALGO_PPMD      0x01
#define COMPRESS_ALGO_ZSTD      0x02

// Entropy thresholds (bits per byte, 0.0 = constant, 8.0 = random)
#define ENTROPY_SKIP_ALL        7.5f   // Incompressible — send raw
#define ENTROPY_ZSTD_ONLY       6.0f   // Mixed — try zstd only
                                       // Below 6.0: try both PPMd and zstd

class cl_compressor {
public:
    cl_compressor();
    ~cl_compressor();

    void init();       // Allocate PPMd model and zstd contexts
    void deinit();     // Free all contexts

    // TX: compress input block, write header+payload to output.
    // Tries PPMd and/or zstd based on entropy, picks smallest.
    // Falls back to raw if compression doesn't help.
    // Returns total bytes written (header + payload), or 0 on error.
    int compress_block(const char* in, int in_len, char* out, int out_capacity);

    // RX: decompress a single block (header + payload already assembled).
    // Returns bytes of decompressed data written to out, or -1 on error.
    int decompress_block(const char* in, int in_len, char* out, int out_capacity);

private:
    float quick_entropy(const unsigned char* data, int len);
    int ppmd_compress(const unsigned char* in, int in_len, unsigned char* out, int out_cap);
    int ppmd_decompress(const unsigned char* in, int in_len, int orig_len, unsigned char* out, int out_cap);
    int zstd_compress_buf(const unsigned char* in, int in_len, unsigned char* out, int out_cap);
    int zstd_decompress_buf(const unsigned char* in, int in_len, unsigned char* out, int out_cap);

    void* ppmd_ctx;    // CPpmd8*
    void* ppmd_mem;    // PPMd allocator memory
    void* zstd_cctx;   // ZSTD_CCtx*
    void* zstd_dctx;   // ZSTD_DCtx*
    bool initialized;
};

#endif
