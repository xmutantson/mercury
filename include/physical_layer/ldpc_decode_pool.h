/*
 * Mercury: A configurable open-source software-defined modem.
 *
 * LEVER C (feat/decode-marathon): multi-core big-block codeword decode pool.
 *
 * The held-CFG16 big block carries Kcw (nominal 8; held-CFG16 up to ~30)
 * independent LDPC codewords whose LLRs all sit in one read-only buffer after
 * acquisition/equalization (telecom_system.cc bigblock_rx_passband). Each
 * codeword is an EMBARRASSINGLY-PARALLEL decode: read-only LLR slice in,
 * DISJOINT info-bit + cw_ok slot out, no cross-codeword state. The serial loop
 * runs them on ONE core while a Pi 5 leaves 2-3 idle; this pool decodes them
 * across a small persistent thread pool so the main RX thread reaches the
 * reverse-ACK turnaround sooner.
 *
 * ISOLATION (the landmine): cl_ldpc holds the only per-decode mutable HEAP
 * workspace (R/Q/V_pos, ldpc.h:43-45). decode_SPA's Cout/LLRbin/LLRtmp/L/fb_*
 * are stack-local => already thread-private. So EACH worker owns a PRIVATE
 * cl_ldpc (its own R/Q/V_pos), config-cloned once for the active config; the
 * QCmatrix* tables it points at are read-only process globals (mercury_normal_*),
 * safe to share. This is the monitor_decoders[] isolation model
 * (arq_common.cc:1764) applied at codeword granularity within ONE config.
 *
 * The pool NEVER touches ARQ state (messages_rx[], SACK, copy_data_to_buffer,
 * streaming PPMd/zstd). It fills ONLY disjoint out_infobits[c*K..] and
 * cw_ok_out[c] slices, then JOINS before returning. The caller's post-join carve
 * sees the EXACT fully-populated serial layout in slot order => byte-identical to
 * the serial decode (fact-documents/decode-marathon-C.md §4 INV-BITEXACT /
 * INV-SLOT-ORDER).
 *
 * DEFAULT-OFF: this pool is constructed/used ONLY when MERCURY_LDPC_MULTICORE is
 * set (>=2). Unset => the serial loop runs bit-for-bit (telecom_system.cc) and
 * this class is never instantiated => zero overhead, byte-identical render.
 */

#ifndef INC_LDPC_DECODE_POOL_H_
#define INC_LDPC_DECODE_POOL_H_

#include "ldpc.h"
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

class cl_ldpc_decode_pool
{
public:
	cl_ldpc_decode_pool();
	~cl_ldpc_decode_pool();

	// Spin up `n_workers` threads, each owning a PRIVATE cl_ldpc cloned from
	// `cfg_template` (its public config scalars copied + init() called, so each
	// worker allocates its OWN R/Q/V_pos and points QCmatrix* at the shared
	// read-only globals). Idempotent IF the same n_workers + same config signature
	// (N,K,decoding_algorithm,nIteration_max) is requested; otherwise it is torn
	// down and rebuilt. Returns true on success.
	bool ensure(const cl_ldpc& cfg_template, int n_workers);

	// Decode all Kcw codewords across the pool. clr is the read-only LLR buffer
	// (length >= Kcw*N). out_infobits receives the K info bits of codeword c at
	// [c*K .. c*K+K) (disjoint per c); cw_ok_out[c] is set 1 when the decode
	// matches cw_info_ref[c] (loopback byte-correct gate) or always 1 when
	// cw_info_ref==nullptr (production: the carve's wire-CRC is the real gate).
	// Returns the count of cw_ok==1 codewords. BLOCKS until every codeword is
	// decoded (barrier) so the caller reads a fully-populated serial layout.
	int decode_batch(const float* clr, int Kcw, int N, int K,
	                 int* out_infobits, std::vector<int>& cw_ok_out,
	                 const std::vector<std::vector<int>>* cw_info_ref);

	void shutdown();

	int worker_count() const { return (int)workers.size(); }

private:
	struct st_worker_ctx
	{
		cl_ldpc           ldpc;        // PRIVATE R/Q/V_pos (the replicated heap)
		std::vector<float> cwllr;      // PRIVATE input scratch
		std::vector<int>   dec;        // PRIVATE output scratch
	};

	std::vector<std::thread>      workers;
	std::vector<st_worker_ctx*>   ctx;          // one per worker (heap to avoid relocation)

	// --- job state for the current batch (set by decode_batch, read by workers) ---
	const float*                              job_clr      = nullptr;
	int*                                      job_out      = nullptr;
	std::vector<int>*                         job_cw_ok    = nullptr;
	const std::vector<std::vector<int>>*      job_cw_ref   = nullptr;
	int                                       job_Kcw      = 0;
	int                                       job_N        = 0;
	int                                       job_K        = 0;
	std::atomic<int>                          job_next{0}; // next codeword index to claim
	std::atomic<int>                          job_remaining{0}; // codewords still outstanding

	// --- pool lifecycle / synchronization ---
	std::mutex                 mtx;
	std::condition_variable    cv_work;   // wake workers when a batch is posted / shutdown
	std::condition_variable    cv_done;   // wake the caller when the batch completes
	unsigned long long         job_generation = 0;  // bumped per batch (workers compare)
	bool                       stop_flag = false;
	bool                       defeat_share = false; // test-only FAIL-BEFORE: share ctx[0] workspace

	// Config signature the pool was built for (including config identity because
	// cfg14->cfg15 keeps N/K/algorithm/iterations but changes scoped policy).
	int sig_N = -1, sig_K = -1, sig_alg = -1, sig_iter = -1, sig_config = -1;

	void worker_loop(int widx);
	void clone_config_into(cl_ldpc& dst, const cl_ldpc& src);
};

#endif // INC_LDPC_DECODE_POOL_H_
