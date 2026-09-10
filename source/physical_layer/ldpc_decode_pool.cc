/*
 * Mercury: A configurable open-source software-defined modem.
 *
 * LEVER C (feat/decode-marathon): multi-core big-block codeword decode pool.
 * See include/physical_layer/ldpc_decode_pool.h and
 * fact-documents/decode-marathon-C.md for the cross-layer audit + design.
 */

#include "physical_layer/ldpc_decode_pool.h"
#include <cstdlib> // std::getenv / atoi for the test-only defeat-share hook

cl_ldpc_decode_pool::cl_ldpc_decode_pool() {}

cl_ldpc_decode_pool::~cl_ldpc_decode_pool()
{
	shutdown();
}

// Clone the PUBLIC config scalars of `src` into `dst` and init(). After this
// `dst` owns its OWN R/Q/V_pos heap (allocated by init()->update_code_parameters)
// and points its QCmatrix* at the SAME read-only process globals (mercury_normal_*)
// as `src`. No shared mutable state. (fact-documents/decode-marathon-C.md §5)
void cl_ldpc_decode_pool::clone_config_into(cl_ldpc& dst, const cl_ldpc& src)
{
	dst.deinit();   // free any prior R/Q/V_pos (no-op on a fresh instance)
	dst.standard            = src.standard;
	dst.framesize           = src.framesize;
	dst.rate                = src.rate;
	dst.decoding_algorithm  = src.decoding_algorithm;
	dst.GBF_eta             = src.GBF_eta;
	dst.nIteration_max      = src.nIteration_max;
	dst.print_nIteration    = src.print_nIteration;
	dst.configuration      = src.configuration;
	dst.init();             // allocates private R/Q/V_pos; freezes N/P/K/Cwidth...
}

bool cl_ldpc_decode_pool::ensure(const cl_ldpc& cfg_template, int n_workers)
{
	if(n_workers < 1) n_workers = 1;

	// Config signature of the requested template. init() computes K from framesize
	// & rate, so use the post-init N/K of a transient clone-free probe: the
	// template is the PRIMARY ldpc which has already had init() run by
	// load_configuration, so cfg_template.N/K are valid here.
	const int want_N    = cfg_template.N;
	const int want_K    = cfg_template.K;
	const int want_alg  = cfg_template.decoding_algorithm;
	const int want_iter = cfg_template.nIteration_max;
	const int want_config = cfg_template.configuration;

	// test-only FAIL-BEFORE hook (read fresh; a toggle forces a rebuild below).
	bool want_defeat = false;
	{ const char* e = std::getenv("MERCURY_DECODE_POOL_DEFEAT_SHARE"); if(e && *e && atoi(e)!=0) want_defeat = true; }

	// Already built for this exact config + worker count + defeat state? Reuse.
	if(!workers.empty()
	   && (int)workers.size() == n_workers
	   && sig_N == want_N && sig_K == want_K
	   && sig_alg == want_alg && sig_iter == want_iter && sig_config == want_config
	   && defeat_share == want_defeat)
	{
		return true;
	}

	// Otherwise tear down and rebuild.
	shutdown();

	{
		std::lock_guard<std::mutex> lk(mtx);
		stop_flag = false;
		job_generation = 0;
		job_next.store(0);
		job_remaining.store(0);
		defeat_share = want_defeat;
	}

	ctx.reserve(n_workers);
	for(int w=0; w<n_workers; ++w)
	{
		st_worker_ctx* c = new st_worker_ctx();
		clone_config_into(c->ldpc, cfg_template);
		c->cwllr.assign((size_t)want_N, 0.0f);
		c->dec.assign((size_t)want_N, 0);
		ctx.push_back(c);
	}

	workers.reserve(n_workers);
	for(int w=0; w<n_workers; ++w)
		workers.emplace_back(&cl_ldpc_decode_pool::worker_loop, this, w);

	sig_N = want_N; sig_K = want_K; sig_alg = want_alg; sig_iter = want_iter;
	sig_config = want_config;
	return true;
}

void cl_ldpc_decode_pool::shutdown()
{
	if(workers.empty() && ctx.empty()) return;

	{
		std::lock_guard<std::mutex> lk(mtx);
		stop_flag = true;
		++job_generation;           // wake any waiting workers
	}
	cv_work.notify_all();

	for(auto& t : workers)
		if(t.joinable()) t.join();
	workers.clear();

	for(auto* c : ctx) delete c;    // ~cl_ldpc frees R/Q/V_pos
	ctx.clear();

	sig_N = sig_K = sig_alg = sig_iter = sig_config = -1;
}

// One persistent worker. Sleeps on cv_work until a batch is posted (job_generation
// bumped) or shutdown. For a batch it claims codeword indices via the atomic
// job_next counter (work-stealing), decodes each into its PRIVATE cl_ldpc/scratch,
// writes the DISJOINT out_infobits[c*K..] and cw_ok_out[c] slices, and signals
// cv_done when the last outstanding codeword completes.
void cl_ldpc_decode_pool::worker_loop(int widx)
{
	st_worker_ctx* self = ctx[widx];
	unsigned long long seen_gen = 0;

	for(;;)
	{
		{
			std::unique_lock<std::mutex> lk(mtx);
			cv_work.wait(lk, [&]{ return stop_flag || job_generation != seen_gen; });
			if(stop_flag) return;
			seen_gen = job_generation;
		}

		// Pull job parameters (set under mtx before generation bump; safe to read).
		const float* clr   = job_clr;
		int*         out   = job_out;
		std::vector<int>* cw_ok = job_cw_ok;
		const std::vector<std::vector<int>>* cw_ref = job_cw_ref;
		const int Kcw = job_Kcw;
		const int N   = job_N;
		const int K   = job_K;

		// FAIL-BEFORE HOOK (test-only, fact-documents/decode-marathon-C.md §8): when
		// defeat_share is set, EVERY worker decodes through ctx[0]'s SINGLE shared
		// cl_ldpc + shared cwllr/dec instead of its own — the exact bug the lever
		// guards against. Two threads then write the same R/Q/V_pos workspace
		// concurrently => silent cross-frame corruption (>=1 codeword wrong / cw_ok
		// dropped). Production never sets MERCURY_DECODE_POOL_DEFEAT_SHARE.
		st_worker_ctx* dwork = defeat_share ? ctx[0] : self;

		for(;;)
		{
			int c = job_next.fetch_add(1, std::memory_order_relaxed);
			if(c >= Kcw) break;

			// slice the read-only LLRs for codeword c
			for(int i=0;i<N;i++) dwork->cwllr[(size_t)i] = clr[(size_t)c*N + i];
			dwork->ldpc.decode(dwork->cwllr.data(), dwork->dec.data());

			// disjoint info-bit slice out
			for(int i=0;i<K;i++)
				out[(size_t)c*K + i] = dwork->dec[(size_t)i];

			// per-codeword clean gate (mirrors the serial loop exactly)
			int ierr=0;
			if(cw_ref && (int)cw_ref->size() > c)
				for(int i=0;i<K;i++) if(dwork->dec[(size_t)i] != (*cw_ref)[c][(size_t)i]) { ierr++; }
			(*cw_ok)[(size_t)c] = (ierr==0) ? 1 : 0;

			// signal completion of this codeword
			if(job_remaining.fetch_sub(1, std::memory_order_acq_rel) == 1)
			{
				std::lock_guard<std::mutex> lk(mtx);
				cv_done.notify_one();
			}
		}
	}
}

int cl_ldpc_decode_pool::decode_batch(
		const float* clr, int Kcw, int N, int K,
		int* out_infobits, std::vector<int>& cw_ok_out,
		const std::vector<std::vector<int>>* cw_info_ref)
{
	cw_ok_out.assign((size_t)Kcw, 0);
	if(Kcw <= 0) return 0;

	// Post the batch and wake the workers.
	{
		std::lock_guard<std::mutex> lk(mtx);
		job_clr    = clr;
		job_out    = out_infobits;
		job_cw_ok  = &cw_ok_out;
		job_cw_ref = cw_info_ref;
		job_Kcw    = Kcw;
		job_N      = N;
		job_K      = K;
		job_next.store(0, std::memory_order_relaxed);
		job_remaining.store(Kcw, std::memory_order_relaxed);
		++job_generation;
	}
	cv_work.notify_all();

	// Barrier: block until every codeword has been decoded.
	{
		std::unique_lock<std::mutex> lk(mtx);
		cv_done.wait(lk, [&]{ return job_remaining.load(std::memory_order_acquire) == 0; });
	}

	// Reduce cw_ok = sum(cw_ok_out) (order-independent count, post-join).
	int cw_ok = 0;
	for(int c=0;c<Kcw;c++) cw_ok += cw_ok_out[(size_t)c];
	return cw_ok;
}
