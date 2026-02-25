/*
 * Mercury block compression — adaptive dual-algorithm (PPMd8 + zstd).
 *
 * TX: entropy test → try PPMd/zstd → pick smallest (including raw).
 * RX: parse 5-byte header → decompress with indicated algorithm.
 *
 * PPMd8 from LZMA SDK (Igor Pavlov, public domain).
 * zstd from Facebook (BSD-3-Clause).
 */

#include "compression/mercury_compress.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>

// ---------- PPMd8 ----------
extern "C" {
#include "ppmd/Ppmd8.h"
}

// ---------- zstd ----------
#include "zstd/zstd.h"

// PPMd model order (2-16). Order 6 = 2 MB memory, good for small blocks.
#define PPMD_ORDER   6
#define PPMD_MEM_SIZE (1 << 21)  // 2 MB

// ---------- PPMd allocator (uses malloc/free) ----------

static void* SzAlloc(ISzAllocPtr p, size_t size)
{
	(void)p;
	return malloc(size);
}

static void SzFree(ISzAllocPtr p, void* address)
{
	(void)p;
	free(address);
}

static const ISzAlloc g_Alloc = { SzAlloc, SzFree };

// ---------- PPMd byte-stream adapters (memory buffers) ----------

struct CByteOutBuf
{
	IByteOut vt;
	unsigned char* buf;
	int capacity;
	int pos;
	int overflow;
};

static void ByteOutBuf_Write(const IByteOut* pp, Byte b)
{
	// The IByteOut interface uses const pointer; cast to access our mutable struct.
	CByteOutBuf* p = (CByteOutBuf*)(void*)pp;
	if (p->pos < p->capacity)
		p->buf[p->pos++] = b;
	else
		p->overflow = 1;
}

struct CByteInBuf
{
	IByteIn vt;
	const unsigned char* buf;
	int size;
	int pos;
};

static Byte ByteInBuf_Read(const IByteIn* pp)
{
	CByteInBuf* p = (CByteInBuf*)(void*)pp;
	if (p->pos < p->size)
		return p->buf[p->pos++];
	return 0;
}

// ---------- cl_compressor ----------

cl_compressor::cl_compressor()
{
	ppmd_ctx = nullptr;
	ppmd_mem = nullptr;
	zstd_cctx = nullptr;
	zstd_dctx = nullptr;
	initialized = false;
}

cl_compressor::~cl_compressor()
{
	deinit();
}

void cl_compressor::init()
{
	if (initialized) return;

	// PPMd8
	CPpmd8* p = (CPpmd8*)malloc(sizeof(CPpmd8));
	if (!p) return;
	Ppmd8_Construct(p);
	if (!Ppmd8_Alloc(p, PPMD_MEM_SIZE, &g_Alloc))
	{
		free(p);
		return;
	}
	ppmd_ctx = p;

	// zstd
	zstd_cctx = ZSTD_createCCtx();
	zstd_dctx = ZSTD_createDCtx();

	if (!zstd_cctx || !zstd_dctx)
	{
		deinit();
		return;
	}

	// zstd level 3: good ratio, fast on small blocks
	ZSTD_CCtx_setParameter((ZSTD_CCtx*)zstd_cctx, ZSTD_c_compressionLevel, 3);

	initialized = true;
	printf("[COMPRESS] Initialized: PPMd8 (order %d, %d KB) + zstd (level 3)\n",
		PPMD_ORDER, PPMD_MEM_SIZE / 1024);
	fflush(stdout);
}

void cl_compressor::deinit()
{
	if (ppmd_ctx)
	{
		Ppmd8_Free((CPpmd8*)ppmd_ctx, &g_Alloc);
		free(ppmd_ctx);
		ppmd_ctx = nullptr;
	}
	if (zstd_cctx)
	{
		ZSTD_freeCCtx((ZSTD_CCtx*)zstd_cctx);
		zstd_cctx = nullptr;
	}
	if (zstd_dctx)
	{
		ZSTD_freeDCtx((ZSTD_DCtx*)zstd_dctx);
		zstd_dctx = nullptr;
	}
	initialized = false;
}

// ---------- Shannon entropy (bits per byte) ----------

float cl_compressor::quick_entropy(const unsigned char* data, int len)
{
	if (len <= 0) return 8.0f;
	int freq[256] = {0};
	for (int i = 0; i < len; i++)
		freq[data[i]]++;

	float entropy = 0.0f;
	float inv_len = 1.0f / (float)len;
	for (int i = 0; i < 256; i++)
	{
		if (freq[i] == 0) continue;
		float p = (float)freq[i] * inv_len;
		entropy -= p * log2f(p);
	}
	return entropy;
}

// ---------- PPMd compress/decompress ----------

int cl_compressor::ppmd_compress(const unsigned char* in, int in_len,
	unsigned char* out, int out_cap)
{
	if (!ppmd_ctx || in_len <= 0) return -1;

	CPpmd8* p = (CPpmd8*)ppmd_ctx;

	CByteOutBuf outStream;
	outStream.vt.Write = ByteOutBuf_Write;
	outStream.buf = out;
	outStream.capacity = out_cap;
	outStream.pos = 0;
	outStream.overflow = 0;

	p->Stream.Out = &outStream.vt;
	Ppmd8_Init(p, PPMD_ORDER, PPMD8_RESTORE_METHOD_RESTART);
	Ppmd8_Init_RangeEnc(p);

	for (int i = 0; i < in_len; i++)
		Ppmd8_EncodeSymbol(p, in[i]);

	Ppmd8_Flush_RangeEnc(p);

	if (outStream.overflow)
		return -1;
	return outStream.pos;
}

int cl_compressor::ppmd_decompress(const unsigned char* in, int in_len,
	int orig_len, unsigned char* out, int out_cap)
{
	if (!ppmd_ctx || in_len <= 0 || orig_len <= 0 || orig_len > out_cap)
		return -1;

	CPpmd8* p = (CPpmd8*)ppmd_ctx;

	CByteInBuf inStream;
	inStream.vt.Read = ByteInBuf_Read;
	inStream.buf = in;
	inStream.size = in_len;
	inStream.pos = 0;

	p->Stream.In = &inStream.vt;
	Ppmd8_Init(p, PPMD_ORDER, PPMD8_RESTORE_METHOD_RESTART);
	if (!Ppmd8_Init_RangeDec(p))
		return -1;

	for (int i = 0; i < orig_len; i++)
	{
		int sym = Ppmd8_DecodeSymbol(p);
		if (sym < 0)
			return -1;
		out[i] = (unsigned char)sym;
	}

	return orig_len;
}

// ---------- zstd compress/decompress ----------

int cl_compressor::zstd_compress_buf(const unsigned char* in, int in_len,
	unsigned char* out, int out_cap)
{
	if (!zstd_cctx || in_len <= 0) return -1;

	size_t result = ZSTD_compress2((ZSTD_CCtx*)zstd_cctx, out, out_cap, in, in_len);
	if (ZSTD_isError(result))
		return -1;
	return (int)result;
}

int cl_compressor::zstd_decompress_buf(const unsigned char* in, int in_len,
	unsigned char* out, int out_cap)
{
	if (!zstd_dctx || in_len <= 0) return -1;

	size_t result = ZSTD_decompressDCtx((ZSTD_DCtx*)zstd_dctx, out, out_cap, in, in_len);
	if (ZSTD_isError(result))
		return -1;
	return (int)result;
}

// ---------- Block compress (TX) ----------

int cl_compressor::compress_block(const char* in, int in_len, char* out, int out_capacity)
{
	if (!initialized || in_len <= 0)
		return 0;

	int raw_total = COMPRESS_HEADER_SIZE + in_len;
	if (raw_total > out_capacity)
		return 0;

	const unsigned char* uin = (const unsigned char*)in;
	unsigned char* uout = (unsigned char*)out;

	// Quick entropy test
	float entropy = quick_entropy(uin, in_len);

	int best_algo = COMPRESS_ALGO_RAW;
	int best_comp_size = in_len;
	const unsigned char* best_payload = uin;

	// Temp buffers for compression output (max frame payload is ~173 bytes)
	unsigned char zstd_buf[512];
	unsigned char ppmd_buf[512];

	// Try zstd if entropy suggests compressibility
	if (entropy <= ENTROPY_SKIP_ALL)
	{
		int zs = zstd_compress_buf(uin, in_len, zstd_buf, (int)sizeof(zstd_buf));
		if (zs > 0 && zs < best_comp_size)
		{
			best_algo = COMPRESS_ALGO_ZSTD;
			best_comp_size = zs;
			best_payload = zstd_buf;
		}
	}

	// Try PPMd if entropy suggests text-like data
	if (entropy < ENTROPY_ZSTD_ONLY)
	{
		int ps = ppmd_compress(uin, in_len, ppmd_buf, (int)sizeof(ppmd_buf));
		if (ps > 0 && ps < best_comp_size)
		{
			best_algo = COMPRESS_ALGO_PPMD;
			best_comp_size = ps;
			best_payload = ppmd_buf;
		}
	}

	// If compressed + header >= raw + header, send raw
	int compressed_total = COMPRESS_HEADER_SIZE + best_comp_size;
	if (compressed_total >= raw_total)
	{
		best_algo = COMPRESS_ALGO_RAW;
		best_comp_size = in_len;
		best_payload = uin;
	}

	// Write 5-byte header: [algo:1][comp_size:2 LE][orig_size:2 LE]
	uout[0] = (unsigned char)best_algo;
	uout[1] = (unsigned char)(best_comp_size & 0xFF);
	uout[2] = (unsigned char)((best_comp_size >> 8) & 0xFF);
	uout[3] = (unsigned char)(in_len & 0xFF);
	uout[4] = (unsigned char)((in_len >> 8) & 0xFF);

	// Write payload
	memcpy(uout + COMPRESS_HEADER_SIZE, best_payload, best_comp_size);

	int total = COMPRESS_HEADER_SIZE + best_comp_size;

	if (best_algo != COMPRESS_ALGO_RAW)
	{
		printf("[COMPRESS] %d -> %d bytes (%s, entropy=%.1f, ratio=%.1fx)\n",
			in_len, total,
			best_algo == COMPRESS_ALGO_PPMD ? "PPMd" : "zstd",
			entropy,
			(float)in_len / (float)best_comp_size);
		fflush(stdout);
	}

	return total;
}

// ---------- Block decompress (RX) ----------

int cl_compressor::decompress_block(const char* in, int in_len, char* out, int out_capacity)
{
	if (!initialized || in_len < COMPRESS_HEADER_SIZE)
		return -1;

	const unsigned char* uin = (const unsigned char*)in;
	unsigned char* uout = (unsigned char*)out;

	// Parse header
	int algo = uin[0];
	int comp_size = uin[1] | (uin[2] << 8);
	int orig_size = uin[3] | (uin[4] << 8);

	// Sanity checks
	if (comp_size < 0 || orig_size < 0 || orig_size > out_capacity)
		return -1;
	if (COMPRESS_HEADER_SIZE + comp_size > in_len)
		return -1;

	const unsigned char* payload = uin + COMPRESS_HEADER_SIZE;

	if (algo == COMPRESS_ALGO_RAW)
	{
		if (comp_size != orig_size)
			return -1;
		memcpy(uout, payload, orig_size);
		return orig_size;
	}
	else if (algo == COMPRESS_ALGO_PPMD)
	{
		int result = ppmd_decompress(payload, comp_size, orig_size, uout, out_capacity);
		if (result != orig_size)
		{
			printf("[DECOMPRESS] PPMd error: expected %d, got %d\n", orig_size, result);
			fflush(stdout);
			return -1;
		}
		return result;
	}
	else if (algo == COMPRESS_ALGO_ZSTD)
	{
		int result = zstd_decompress_buf(payload, comp_size, uout, out_capacity);
		if (result != orig_size)
		{
			printf("[DECOMPRESS] zstd error: expected %d, got %d\n", orig_size, result);
			fflush(stdout);
			return -1;
		}
		return result;
	}

	printf("[DECOMPRESS] Unknown algo 0x%02X\n", algo);
	fflush(stdout);
	return -1;
}
