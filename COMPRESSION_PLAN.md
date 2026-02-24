# Plan: Adaptive Block Compression for Mercury

**Status**: Planned — not yet implemented

## Context

Mercury's PHY layer is competitive with VARA HF, but VARA reports ~2x higher
net throughput on typical Winlink traffic. The gap is largely explained by VARA's
Huffman compression at the data layer — Mercury sends all data raw. Adding
block-level compression with adaptive algorithm selection (PPMd for text,
zstd for binary) will close this gap on text-heavy traffic and match VARA on
binary payloads where neither compressor helps.

Design goals:
- **Block-level** compression — compress each block as it's popped from the FIFO
- **Dual-algorithm**: PPMd (best on text, ~4.5-5x) + zstd (best on binary, ~2-3x)
- **Per-block adaptive**: try both on each block, send whichever is smaller
- **Backwards-compatible**: negotiate via existing capability flags; old Mercury = raw mode

---

## Data Flow (Current)

```
TX: TCP recv → fifo_buffer_tx.push() → ARQ frames → PHY
RX: PHY → ARQ frames → fifo_buffer_rx → pop() → TCP send
```

**Key facts:**
- TCP buffer: 8192 bytes max per recv (`MAX_BUFFER_SIZE`, tcp_socket.h:54)
- FIFO buffers: 128KB each (datalink_config.cc:28-29)
- Frame payloads: 10 bytes (CONFIG_0) to 173 bytes (CONFIG_16)
- ARQ layer handles fragmentation/reassembly — treats FIFO contents as opaque bytes

---

## Architecture

Compress each data block before it enters the ARQ framing, decompress after
reassembly on the receive side:

```
TX: TCP recv → fifo_buffer_tx → pop block → compress(best_of PPMd,zstd) → ARQ frame
RX: ARQ frame → decompress → fifo_buffer_rx → TCP send
```

### Block Format

Each compressed block has a 5-byte header:

```
[algo:1][comp_size:2 LE][orig_size:2 LE][compressed_payload...]
```

- `algo`: 0x00=raw (no compression), 0x01=PPMd, 0x02=zstd
- `comp_size`: bytes of compressed payload following header
- `orig_size`: original uncompressed size
- Raw blocks (algo=0x00): comp_size == orig_size, payload is uncompressed

### Compression Strategy

1. When ARQ commander pops a block from `fifo_buffer_tx` for framing
2. Compress with **both** PPMd and zstd
3. Pick whichever produces smaller output
4. If neither compresses (output + 5-byte header >= input + 5-byte header), send raw
5. Frame the compressed block (header + payload) as the ARQ data payload
6. Every block carries its own algorithm tag — no synchronization needed

At Mercury's max data rate (717 bytes/sec), CPU is never the bottleneck.
PPMd on small blocks: <10ms. Zstd on small blocks: <1ms.

### Decompression

1. Receive ARQ data frame
2. Parse 5-byte header → know algo and comp_size and orig_size
3. Decompress with indicated algorithm
4. Push decompressed data to `fifo_buffer_rx`

### Capability Negotiation

Existing `TEST_CONNECTION` handshake carries capability byte (byte 5):
- Current: `CAP_WB_CAPABLE = 0x01`
- Add: `CAP_COMPRESSION = 0x02`

Both sides advertise in TEST_CONNECTION. If **both** sides have the flag,
compression is enabled. Otherwise, raw mode. Fully backwards-compatible:
old Mercury sends 0x00 or 0x01 → no compression bit → raw.

When compression is enabled on a connection, **both directions** use it
(after SWITCH_ROLE, the former responder becomes commander and vice versa).

---

## Libraries

### PPMd8 (from LZMA SDK)

7 files, ~3000 lines, **public domain**:
```
Ppmd.h, Ppmd8.h, Ppmd8.c, Ppmd8Enc.c, Ppmd8Dec.c, CpuArch.h, 7zTypes.h
```

- Byte-at-a-time callback API (`IByteIn`/`IByteOut`)
- Per-block: `Ppmd8_Init()` resets model, `Ppmd8_Flush_RangeEnc()` closes block
- Memory: 2MB (order 6) — both compress and decompress sides
- Pure C89, works with MinGW
- Source: LZMA SDK by Igor Pavlov, also available via pps83/libppmd on GitHub

### Zstd (amalgamated)

2 files, **BSD-3-Clause** (compatible with AGPL):
```
zstd.h, zstd.c (~1.2MB source)
```

- Buffer API: `ZSTD_compressStream2()` with `ZSTD_e_end` per block
- Each block is an independent frame — no inter-block state
- Memory: ~100KB per context at level 3 for small blocks
- Pure C99, works with MinGW
- Source: facebook/zstd on GitHub, `build/single_file_libs/` for amalgamated

---

## Implementation

### New files

| File | Purpose |
|------|---------|
| `include/compression/mercury_compress.h` | Compression layer API |
| `source/compression/mercury_compress.cc` | Dual-algorithm compress/decompress |
| `source/compression/ppmd/` | PPMd8 files from LZMA SDK (7 files) |
| `source/compression/zstd/` | zstd.h + zstd.c (amalgamated) |

### `mercury_compress.h` — API

```cpp
#ifndef MERCURY_COMPRESS_H
#define MERCURY_COMPRESS_H

#include <cstdint>

// Block header: [algo:1][comp_size:2][orig_size:2]
#define COMPRESS_HEADER_SIZE    5
#define COMPRESS_ALGO_RAW       0x00
#define COMPRESS_ALGO_PPMD      0x01
#define COMPRESS_ALGO_ZSTD      0x02

class cl_compressor {
public:
    cl_compressor();
    ~cl_compressor();

    void init();       // Allocate PPMd and zstd contexts
    void deinit();     // Free contexts

    // TX: compress input block, write header+payload to output
    // Tries both PPMd and zstd, picks smaller. Falls back to raw if no gain.
    // Returns total bytes written (header + payload), or 0 on error
    int compress_block(const char* in, int in_len, char* out, int out_capacity);

    // RX: decompress a single block (header + payload already assembled)
    // Returns bytes of decompressed data written to out, or -1 on error
    int decompress_block(const char* in, int in_len, char* out, int out_capacity);

private:
    void* ppmd_ctx;    // CPpmd8*
    void* zstd_cctx;   // ZSTD_CCtx*
    void* zstd_dctx;   // ZSTD_DCtx*
};

#endif
```

### Modified files

**`include/datalink_layer/datalink_defines.h`**
```cpp
#define CAP_COMPRESSION  0x02   // Supports block compression
```

**`include/datalink_layer/arq.h`** — add members to `cl_arq_controller`:
```cpp
#include "compression/mercury_compress.h"

cl_compressor compressor;
bool compression_enabled;          // Negotiated with peer
```

**`source/datalink_layer/arq_commander.cc`** — TX side
(`process_buffer_data_commander()`, line ~1893):

Currently pops raw data from FIFO and creates ARQ frames. With compression:
after popping from `fifo_buffer_tx`, compress the block, then pass the
compressed data (header+payload) to `add_message_tx_data()`.

**`source/datalink_layer/arq_responder.cc`** — RX side
(`add_message_rx_data()` or `copy_data_to_buffer()`):

After ARQ reassembly, before pushing to `fifo_buffer_rx`: parse the
compression header, decompress, push decompressed data to FIFO.

**`source/datalink_layer/arq_common.cc`** — Capability setup:

Where `local_capability` is set (lines 1811, 1907, 1942, 1960, 1979),
OR in `CAP_COMPRESSION`:
```cpp
local_capability |= CAP_COMPRESSION;
```

Where `peer_capability` is read in TEST_CONNECTION handlers (commander line 1425,
responder line 727), check for compression:
```cpp
compression_enabled = (local_capability & CAP_COMPRESSION) &&
                      (peer_capability & CAP_COMPRESSION);
```

**Build system** (`build.sh`):
- Add `source/compression/mercury_compress.cc` to compile list
- Add `source/compression/ppmd/*.c` and `source/compression/zstd/zstd.c`
- PPMd: no special flags needed (C89)
- Zstd: add `-DZSTD_DISABLE_ASM` if needed for MinGW compatibility

---

## Expected Performance

| Data type | Ratio | Effective throughput boost |
|-----------|-------|--------------------------|
| English text email | 3-5x (PPMd wins) | 3-5x |
| HTML/forms | 4-6x (PPMd wins) | 4-6x |
| JPEG/PNG/ZIP | 1.0x (raw) | none (no penalty) |
| Mixed text+attachments | 1.5-2x | 1.5-2x |
| Random binary | 1.0x (raw) | none (no penalty) |

Note: compression ratios on very small blocks (10-48 bytes for low configs)
will be reduced compared to larger blocks. PPMd needs ~1-2KB of data to build
useful context models. Best ratios will be seen on CONFIG_8+ where frame
payloads are larger. For very small frames, the 5-byte header overhead may
cause raw mode to be selected most of the time, which is fine — no penalty.

CONFIG_12 WB example: 1555 bps raw → ~5000-7000 bps on text email (exceeds VARA).

---

## Verification

1. **Build**: `./build.sh o3` — verify compiles clean with PPMd + zstd
2. **Unit test**: Add `-m COMPRESS_TEST` mode that compresses/decompresses test
   strings and verifies round-trip integrity
3. **Loopback test**: ARQ loopback on VB-Cable, send known text file,
   verify received == sent, check stdout for compression ratio logging
4. **Incompressible test**: Send random binary, verify raw mode selected,
   no expansion
5. **Backwards-compat**: Run new Mercury against old Mercury (no compression flag),
   verify raw mode negotiated and data transfers normally
6. **Benchmark**: Re-run parallel sweep with compression enabled using text
   test data to measure actual throughput improvement

---

## Key Code References

| Location | Purpose |
|----------|---------|
| `arq_common.cc:1713-1758` | TX: TCP recv → fifo_buffer_tx |
| `arq_commander.cc:1882-1914` | TX: fifo_buffer_tx → ARQ frames |
| `arq_commander.cc:565-606` | add_message_tx_data() |
| `arq_responder.cc:49-101` | RX: add_message_rx_data() |
| `arq_common.cc:3185-3204` | RX: copy_data_to_buffer() |
| `arq_responder.cc:898-913` | RX: fifo_buffer_rx → TCP send |
| `arq_responder.cc:715-758` | TEST_CONNECTION handshake (responder) |
| `arq_commander.cc:1420-1430` | TEST_CONNECTION handshake (commander) |
| `datalink_defines.h:87` | CAP_WB_CAPABLE flag |
| `arq.h:364-365` | local_capability / peer_capability |

---

## Files Summary

| File | Action |
|------|--------|
| `include/compression/mercury_compress.h` | NEW — compression API |
| `source/compression/mercury_compress.cc` | NEW — dual-algorithm impl |
| `source/compression/ppmd/` (7 files) | NEW — PPMd8 from LZMA SDK |
| `source/compression/zstd/zstd.{h,c}` | NEW — zstd amalgamated |
| `include/datalink_layer/datalink_defines.h` | ADD `CAP_COMPRESSION` |
| `include/datalink_layer/arq.h` | ADD compression members |
| `source/datalink_layer/arq_commander.cc` | MODIFY TX: compress before framing |
| `source/datalink_layer/arq_responder.cc` | MODIFY RX: decompress after reassembly |
| `source/datalink_layer/arq_common.cc` | MODIFY capability + teardown |
| `build.sh` | ADD compression source files |
