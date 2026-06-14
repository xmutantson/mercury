/*
 * Mercury: A configurable open-source software-defined modem.
 * Copyright (C) 2022-2024 Fadi Jerji
 * Author: Fadi Jerji
 * Email: fadi.jerji@  <gmail.com, caisresearch.com, ieee.org>
 * ORCID: 0000-0002-2076-5831
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, version 3 of the
 * License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "physical_layer/ldpc_decoder_SPA.h"
#include <cstdlib>   // std::getenv / atoi for the MERCURY_LDPC_FWDBACK gate

// SOLUTION A (fix/ldpc-decode-accel): forward-backward check-node update.
// The original SPA check-node update (below, default path) recomputes the full
// leave-one-out tanh product SEPARATELY for every outgoing edge of a check node
// => O(dc^2) tanh per check (dc=46 for CFG16 rate-14/16 => ~46*46 ~= 2116 tanh
// per check, P=200 checks => ~414k tanh per BP iteration; ldpc_decoder_SPA.cc
// FACTS). The forward-backward (prefix/suffix product) form computes the SAME
// leave-one-out product in O(dc): one pass of forward partials F[i]=prod(t[0..i-1])
// and one pass of backward partials B[i]=prod(t[i+1..dc-1]), out[i]=F[i]*B[i].
// Output is mathematically identical to the original; the only possible delta is
// floating-point product RE-ASSOCIATION in the last ULP (proven via the coded
// SFO-GRID BER diff — see FACTS / the branch verdict).
//
// Gate: MERCURY_LDPC_FWDBACK unset/0 => the ORIGINAL O(dc^2) loop runs
// bit-for-bit (default-off byte-identical). The env is read once per process
// (static cache) so the gate adds nothing to the hot path.
static inline bool ldpc_fwdback_enabled()
{
	static const int v = []{
		const char* e = std::getenv("MERCURY_LDPC_FWDBACK");
		return (e && *e) ? atoi(e) : 0;
	}();
	return v != 0;
}

// LEVER D (feat/decode-marathon): LAYERED / row-layered / horizontal-shuffled BP.
// The default path above is FLOODING: within one iteration every check node reads
// the SAME old var->check messages (Q snapshot), and the var nodes are updated only
// AFTER all checks have run. Layered BP instead processes the check rows ONE AT A
// TIME and immediately propagates each updated check->var message into a running
// a-posteriori sum L[v] = LLRi[v] + sum_all R[v][slot], so a LATER row in the SAME
// iteration reads var->check messages already refreshed by the EARLIER rows of that
// iteration ("the most recent information is disseminated"). On the QC/IRA matrices
// of mercury_normal_*.cc this is the natural row-layered schedule. Standard result:
// ~2x faster convergence (about HALF the iterations) at bit-comparable BER, often a
// slightly LOWER non-converger floor.
//   D. Hocevar, "A reduced complexity decoder architecture via layered decoding of
//   LDPC codes," IEEE Workshop on Signal Processing Systems (SIPS), 2004, pp. 107-112
//   (doi:10.1109/SIPS.2004.1363033) — the ~2x-iteration / 50%-logic result.
//   M. M. Mansour and N. R. Shanbhag, "High-throughput LDPC decoders," IEEE Trans.
//   VLSI 11(6):976-996, 2003 — the turbo-decoding-message-passing (layered) schedule.
//   E. Sharon, S. Litsyn, J. Goldberger, "Efficient serial message-passing schedules
//   for LDPC decoding," IEEE Trans. Inf. Theory 53(11):4076-4091, 2007 — serial-C
//   schedule, same ~2x convergence acceleration, BER comparable-or-better.
//
// The per-edge check-node math is UNCHANGED: each outgoing R is 2*atanh of the same
// leave-one-out product of tanh(0.5*Q) over the OTHER edges of the row, with the
// same +-1 saturation clamp. Only WHEN the var->check extrinsic Q is formed (from
// the LATEST L instead of a frozen snapshot) differs. The leave-one-out product
// reuses the EXACT O(dc^2) or (with MERCURY_LDPC_FWDBACK) O(dc) forward-backward
// kernel — lever D composes cleanly with A. The syndrome / #3 early-term detector
// run on the layered L[v] hard decision unchanged.
//
// Gate: MERCURY_LDPC_LAYERED unset/0 => the FLOODING loop above runs bit-for-bit
// (default-off byte-identical render). Read once per process (static cache).
static inline bool ldpc_layered_enabled()
{
	static const int v = []{
		const char* e = std::getenv("MERCURY_LDPC_LAYERED");
		return (e && *e) ? atoi(e) : 0;
	}();
	return v != 0;
}

// ============================================================================
// SHARED NON-CONVERGENCE DETECTOR (feat/turnaround-eff, fact-documents/
// turnaround-eff.md §2). ONE implementation; #3 (env MERCURY_SYND_EARLYTERM,
// mode 1) and #1(c) (cl_ldpc::early_term_speculative, mode 2) both call it.
//
// Check-Sum Variation criterion (D. Li, X. Huang et al., "A Unified Early
// Stopping Criterion for Binary and Nonbinary LDPC Codes Based on Check-Sum
// Variation Patterns," IEEE Comm. Letters 14(11):1053-1055, 2010): for a
// DECODABLE block the syndrome weight eventually DESCENDS toward 0 (the
// existing nOnes==0 break catches success); for an UNDECODABLE block it
// fluctuates/plateaus in a range above 0.
//
// EMPIRICAL CAVEAT (measured on this code/channel, turnaround-eff.md §6): a
// real-but-slow CFG16 converger is NOT monotone per-iteration — it can find a
// LOW early minimum, then RISE and oscillate in the 30s-50s for 30+ iterations
// before the final descent (trace conv@45/conv@62). So "no new minimum for N
// iters" alone FALSELY kills slow convergers. The robust, LOSSLESS gate is the
// SYNDROME FLOOR: a real converger ALWAYS dips its running minimum BELOW a
// meaningful fraction of the check count P on its way down; a truly-stuck frame
// (the clean-WGN non-convergence this branch targets, §0) never gets close —
// its running minimum stays HIGH (often pinned at P). We therefore trip ONLY
// when the running minimum has STALLED *and is still above the floor*.
//
// floor = P/2: swept on the 27-cell grid (1674 codewords) — at floor>=P/2 the
// detector loses ZERO of 416 convergers at EVERY (warmup,confirm) while catching
// ~89% of 1258 failers at mean iter ~12 (turnaround-eff.md §6). Lowering the
// floor toward P/8 starts costing convergers => P/2 is the lossless setting.
//
// Caller carries `min_nones` (lowest syndrome weight so far, init INT_MAX) and
// `best_synd_iter` (iteration that set it). Returns true => declare
// non-convergence, quit now (frame stays classified FAIL via the
// nIteration_max+1 sentinel the caller returns).
static inline bool spa_nonconverge_detect(
		int iteration, int nOnes, int& min_nones, int& best_synd_iter,
		int warmup, int confirm, int floor)
{
	// Track the running minimum (a NEW strict minimum == still descending).
	if(nOnes < min_nones) { min_nones = nOnes; best_synd_iter = iteration; }

	// Guard 1: syndrome-depth >= warmup (>=5). Protects the early plateau of a
	// slow converger before its descent kicks in.
	if(iteration < warmup) return false;

	// Guard 2: FLOOR. Only a frame whose running minimum NEVER dropped below the
	// floor is a candidate. A real converger has already dipped below P/2 by
	// here, so min_nones<=floor => never trip (LOSSLESS). min_nones<=0 also fails
	// this gate (defensive: a 0 min would have hit the success break anyway).
	if(min_nones <= floor) return false;

	// Guard 3: confirm window. The high-floor minimum has not improved for
	// `confirm` consecutive iterations => stuck above the floor => undecodable.
	return (iteration - best_synd_iter) >= confirm;
}

// Per-call override of the mode-1 (MERCURY_SYND_EARLYTERM) warmup/confirm for
// the campaign sweep. Read once (static). Mode 2 (#1 speculative) uses fixed
// eager constants. Mode-1 defaults warmup=12, confirm=8 (swept LOSSLESS on the
// 27-cell grid at floor=P/2, mean catch iter ~12; turnaround-eff.md §6). Both
// satisfy depth>=5.
static inline void spa_earlyterm_params(int mode, int& warmup, int& confirm)
{
	if(mode == 2) { warmup = 8; confirm = 6; return; }   // #1(c) eager wrong-pos
	// mode 1 (#3): defaults + optional env overrides.
	static const int w = []{
		const char* e = std::getenv("MERCURY_SYND_WARMUP");
		int v = (e && *e) ? atoi(e) : 12;
		if(v < 5) v = 5;   // depth>=5 invariant (turnaround-eff.md §2 Guard 1)
		return v;
	}();
	static const int c = []{
		const char* e = std::getenv("MERCURY_SYND_CONFIRM");
		int v = (e && *e) ? atoi(e) : 8;
		if(v < 2) v = 2;
		return v;
	}();
	warmup = w; confirm = c;
}

int decode_SPA(
		const float LLRi[],
		int LLRo[],
		int* C,
		int CWidth,
		int CWidthMax,
		int* V,
		int VWidth,
		int VWidthMax,
		int d[],
		int dWidth,
		double* R,
		double* Q,
		int* V_pos,
		int N,
		int K,
		int P,
		int nIteration_max,
		std::atomic<bool>* abort_flag,
		double* app_llr,
		int early_term_mode,
		int* out_early_term_iter
)
{
	int Cout[N_MAX];
	int LLRbin[N_MAX];
	int iteration=0;
	int i,j,nOnes;
	double LLRtmp[N_MAX];

	// feat/turnaround-eff: shared non-convergence detector state + params.
	// early_term_mode==0 (default) => the detector is NEVER called => the loop
	// runs to the cap exactly as before (byte-identical).
	if(out_early_term_iter) *out_early_term_iter = -1;
	int et_min_nones = 0x7FFFFFFF;   // INT_MAX: lowest syndrome weight so far
	int et_best_iter = 0;            // iteration that set et_min_nones
	int et_warmup = 0, et_confirm = 0, et_floor = 0;
	if(early_term_mode != 0)
	{
		spa_earlyterm_params(early_term_mode, et_warmup, et_confirm);
		// FLOOR = P/2 (swept lossless, turnaround-eff.md §6). MERCURY_SYND_FLOOR
		// overrides as a PERCENT of P for the campaign sweep (e.g. 50 => P/2).
		et_floor = P / 2;
		static const int floor_pct = []{
			const char* e = std::getenv("MERCURY_SYND_FLOOR");
			int v = (e && *e) ? atoi(e) : -1;   // -1 => use P/2 default
			return v;
		}();
		if(floor_pct >= 0) et_floor = (P * floor_pct) / 100;
	}

	for( i=0;i<N;i++)
	{
		for(j=0;j<VWidth;j++)
		{
			*(R+i*VWidthMax+j)=0;
			*(Q+i*VWidthMax+j)=0;
		}
		LLRbin[i]= (LLRi[i]<0);
		LLRtmp[i]=LLRi[i];
	}

	nOnes=0;
	for (i=0;i<P;i++)
	{
		Cout[i]=LLRbin[*(C+i*CWidthMax+0)];
		for( j=1;j<CWidth;j++)
		{
			if(*(C+i*CWidthMax+j)!=-1)
			{
				Cout[i]^= LLRbin[*(C+i*CWidthMax+j)];
			}
		}

		nOnes+=Cout[i];
	}
	if(nOnes!=0)
	{
		// Precompute V matrix positions to eliminate linear search in inner loop
		// V_pos[check * CWidthMax + col] = position of check in V[C[check][col]][]
		// V_pos buffer is pre-allocated by cl_ldpc::load() and passed in
		for(int ci=0;ci<P;ci++)
		{
			for(int cj=0;cj<CWidth;cj++)
			{
				int v=*(C+ci*CWidthMax+cj);
				if(v!=-1)
				{
					int pos=-1;
					for(int vk=0;vk<VWidth;vk++)
					{
						if(*(V+v*VWidthMax+vk)==ci)
						{
							pos=vk;
							break;
						}
					}
					V_pos[ci*CWidthMax+cj]=pos;
				}
				else
				{
					V_pos[ci*CWidthMax+cj]=-1;
				}
			}
		}

		int start=0;
		int end=0;
		int width=0;
		for (int section=0;section<dWidth;section+=2)
		{
			end+=d[section];
			width=d[section+1];

			for( i=start;i<end;i++)
			{
				for( j=0;j<width;j++)
				{
					*(Q+i*VWidthMax+j)=LLRi[i];
				}
			}
			start+=d[section];
		}

		int iindex,Cindex,i1index,i1;
		double temp;

		// SOLUTION A scratch (forward-backward path only). Sized to the max
		// check-node degree across all Mercury LDPC matrices (CFG16 rate-14/16
		// dc=46, mercury_normal_14_16.cc:28); 64 leaves margin. Stack-resident
		// => no heap churn, reused every iteration.
		const bool fwdback = ldpc_fwdback_enabled();
		const int  CW_SCRATCH = 64;            // >= max Cwidth (46)
		double  fb_t   [CW_SCRATCH];           // tanh(0.5*Q) per valid edge slot
		double  fb_pref[CW_SCRATCH];           // forward partial product (exclusive)
		int     fb_slot[CW_SCRATCH];           // original Cindex of each valid edge

		// LEVER D scratch: V_pos of each valid edge of the row currently being
		// processed (so the in-row APP write-back addresses R[v][slot] directly).
		int     fb_vslot[CW_SCRATCH];          // V_pos[iindex][Cindex] per valid edge

		const bool layered = ldpc_layered_enabled();
		if(layered)
		{
		// ====================================================================
		// LEVER D: LAYERED (row-by-row) BP. Runs ONLY when MERCURY_LDPC_LAYERED
		// is set; otherwise the FLOODING loop in the else-branch runs bit-for-bit
		// (default-off byte-identical). See the header comment for the citations.
		// ====================================================================
		// Running a-posteriori LLR: L[v] = LLRi[v] + sum_all R[v][slot].
		// R is all-zero here (init at :179-188), so L == LLRi at entry. As each
		// row's outgoing messages are recomputed below, L is updated INCREMENTALLY
		// (L += R_new - R_old) so the next row reads the freshest extrinsic.
		double L[N_MAX];
		for(i=0;i<N;i++) L[i]=LLRi[i];

		for(iteration=1;iteration<=nIteration_max;iteration++)
		{
			// Early exit: another parallel decoder already succeeded.
			if(abort_flag && abort_flag->load(std::memory_order_relaxed))
				return -iteration;

			// --- One sweep over the P check rows (the layers) -------------
			for( iindex=0;iindex<P;iindex++)
			{
				// Gather this row's valid edges: tanh(0.5*Q) where the var->check
				// extrinsic Q = L[v] - R[v][slot] uses the LATEST L (already
				// refreshed by the earlier rows of THIS iteration = the layered
				// win). Record each edge's V-slot so the write-back is O(1).
				int nv=0;
				for( Cindex=0;Cindex<CWidth;Cindex++)
				{
					int vj=*(C+iindex*CWidthMax+Cindex);
					if(vj!=-1)
					{
						int vi=V_pos[iindex*CWidthMax+Cindex];
						double q = L[vj] - *(R+vj*VWidthMax+vi);   // extrinsic Q
						fb_t[nv]    = tanh(0.5*q);
						fb_slot[nv] = vj;                          // var-node index
						fb_vslot[nv]= vi;                          // its V-slot
						nv++;
					}
				}
				if(nv==0) continue;

				if(!fwdback)
				{
					// O(dc^2) leave-one-out product (composes with A: same kernel
					// as the flooding default path, just fed layered-fresh Q).
					for(int k=0;k<nv;k++)
					{
						temp=1;
						for(int m=0;m<nv;m++) if(m!=k) temp*=fb_t[m];
						if(temp==1)  temp= 0.9999999;   // SAME clamp as flooding
						if(temp==-1) temp=-0.9999999;
						double Rnew = 2*atanh(temp);
						int vj=fb_slot[k], vi=fb_vslot[k];
						double* Rcell = R+vj*VWidthMax+vi;
						L[vj] += Rnew - *Rcell;          // incremental APP update
						*Rcell = Rnew;
					}
				}
				else
				{
					// O(dc) forward-backward leave-one-out product (lever A kernel).
					fb_pref[0]=1.0;
					for(int k=1;k<nv;k++) fb_pref[k]=fb_pref[k-1]*fb_t[k-1];
					double suffix=1.0;
					for(int k=nv-1;k>=0;k--)
					{
						temp = fb_pref[k]*suffix;        // leave-one-out product
						if(temp==1)  temp= 0.9999999;
						if(temp==-1) temp=-0.9999999;
						double Rnew = 2*atanh(temp);
						int vj=fb_slot[k], vi=fb_vslot[k];
						double* Rcell = R+vj*VWidthMax+vi;
						L[vj] += Rnew - *Rcell;          // incremental APP update
						*Rcell = Rnew;
						suffix*=fb_t[k];
					}
				}
			}

			// Hard decision + syndrome from the layered APP L[v].
			for( i=0;i<N;i++) LLRbin[i]=(L[i]<0);
			nOnes=0;
			for( i=0;i<P;i++)
			{
				Cout[i]=LLRbin[*(C+i*CWidthMax+0)];
				for( j=1;j<CWidth;j++)
					if(*(C+i*CWidthMax+j)!=-1) Cout[i]^=LLRbin[*(C+i*CWidthMax+j)];
				nOnes+=Cout[i];
			}

			// Keep LLRtmp in sync so the function's shared epilogue (hard-decision
			// LLRo[0..K) at :426 and the optional app_llr publish at :437) operate
			// on the layered APP exactly as the flooding path's LLRtmp would.
			for( i=0;i<N;i++) LLRtmp[i]=L[i];

			if(nOnes==0) break;

			// #3 shared non-convergence early-term — operates on the layered
			// syndrome unchanged (composes with #3). On a trip return the same
			// canonical FAIL sentinel.
			if(early_term_mode != 0 &&
			   spa_nonconverge_detect(iteration, nOnes, et_min_nones,
			                          et_best_iter, et_warmup, et_confirm, et_floor))
			{
				if(out_early_term_iter) *out_early_term_iter = iteration;
				if(app_llr)
					for(int ai=0; ai<N; ai++) app_llr[ai]=LLRtmp[ai];
				for(int oi=0; oi<K; oi++) LLRo[oi]=(LLRtmp[oi]<0);
				return nIteration_max + 1;
			}
		}
		}
		else
		{
		for(iteration=1;iteration<=nIteration_max;iteration++)
		{
			// Early exit: another parallel decoder already succeeded
			if(abort_flag && abort_flag->load(std::memory_order_relaxed))
				return -iteration;

			if(!fwdback)
			{
			// --- ORIGINAL O(dc^2) check-node update (default, byte-identical) ---
			for ( iindex=0;iindex<P;iindex++)
			{
				for ( Cindex=0;Cindex<CWidth;Cindex++)
				{
					j=*(C+iindex*CWidthMax+Cindex);
					if(j!=-1)
					{
						temp=1;
						for ( i1index=0;i1index<CWidth;i1index++)
						{
							i1=*(C+iindex*CWidthMax+i1index);
							if(i1!=j && i1!=-1)
							{
								i=V_pos[iindex*CWidthMax+i1index];
								temp*=tanh(0.5* (double)*(Q+i1*VWidthMax+i));
							}

						}
						if(temp==1) // to avoid the limitation of the float/double
						{
							temp=0.9999999;
						}
						if(temp==-1)
						{
							temp=-0.9999999;
						}
						*(R+j*VWidthMax+V_pos[iindex*CWidthMax+Cindex])=2*atanh(temp);

					}

				}
			}
			}
			else
			{
			// --- SOLUTION A: O(dc) forward-backward leave-one-out product -------
			// For each check node, gather the valid edges' tanh(0.5*Q) into fb_t[]
			// (preserving the original Cindex slot order in fb_slot[]), then form
			// the leave-one-out product out[k] = prod(fb_t[m], m!=k) via a forward
			// prefix pass + a single backward sweep. The product set, ordering and
			// per-edge tanh argument are IDENTICAL to the O(dc^2) loop; only the
			// multiply ordering changes (FP last-ULP reassociation, proven benign
			// by the coded BER diff). The temp==±1 saturation clamp is applied to
			// the SAME leave-one-out product value before 2*atanh, exactly as above.
			for ( iindex=0;iindex<P;iindex++)
			{
				int nv=0;   // number of valid (non -1) edges in this check row
				for ( Cindex=0;Cindex<CWidth;Cindex++)
				{
					int vj=*(C+iindex*CWidthMax+Cindex);
					if(vj!=-1)
					{
						int vi=V_pos[iindex*CWidthMax+Cindex];
						fb_t[nv]   = tanh(0.5* (double)*(Q+vj*VWidthMax+vi));
						fb_slot[nv]= Cindex;
						nv++;
					}
				}
				if(nv==0) continue;

				// Forward exclusive-prefix products: fb_pref[k] = prod(fb_t[0..k-1]).
				fb_pref[0]=1.0;
				for(int k=1;k<nv;k++)
					fb_pref[k]=fb_pref[k-1]*fb_t[k-1];

				// Backward sweep carries suffix product; out[k]=fb_pref[k]*suffix.
				double suffix=1.0;
				for(int k=nv-1;k>=0;k--)
				{
					temp = fb_pref[k]*suffix;     // leave-one-out product, edge k
					if(temp==1)  temp= 0.9999999; // SAME clamp as the original loop
					if(temp==-1) temp=-0.9999999;
					int Cidx=fb_slot[k];
					j=*(C+iindex*CWidthMax+Cidx);
					*(R+j*VWidthMax+V_pos[iindex*CWidthMax+Cidx])=2*atanh(temp);
					suffix*=fb_t[k];
				}
			}
			}

			for( i=0;i<N;i++)
			{
				LLRtmp[i]=LLRi[i];
				for ( j=0;j<VWidth;j++)
				{
					LLRtmp[i]+=*(R+i*VWidthMax+j);
				}
				LLRbin[i]= (LLRtmp[i]<0);
			}


			nOnes=0;
			for( i=0;i<P;i++)
			{
				Cout[i]=LLRbin[*(C+i*CWidthMax+0)];
				for( j=1;j<CWidth;j++)
				{
					if(*(C+i*CWidthMax+j)!=-1)
					{
						Cout[i]^=LLRbin[*(C+i*CWidthMax+j)];
					}
				}
				nOnes+=Cout[i];
			}

			if(nOnes==0)
			{
				break;
			}

			// feat/turnaround-eff: shared non-convergence early-term (§2/§3).
			// Runs ONLY when early_term_mode != 0 (env MERCURY_SYND_EARLYTERM
			// mode 1, or the #1(c) speculative flag mode 2). On a trip the frame
			// has NOT converged (nOnes != 0 here), so we return the canonical
			// FAIL sentinel nIteration_max+1 — every consumer's
			// `iterations_done > nIteration_max-1` FAIL predicate fires exactly
			// as for a natural cap-out. The real early-term iter is exposed for
			// measurement only.
			if(early_term_mode != 0 &&
			   spa_nonconverge_detect(iteration, nOnes, et_min_nones,
			                          et_best_iter, et_warmup, et_confirm, et_floor))
			{
				if(out_early_term_iter) *out_early_term_iter = iteration;
				// Publish app_llr exactly as the fall-through path would (the
				// caller may consume it); LLRtmp already holds the a-posteriori
				// LLR for this iteration.
				if(app_llr)
					for(int ai=0; ai<N; ai++) app_llr[ai]=LLRtmp[ai];
				for(int oi=0; oi<K; oi++) LLRo[oi]=(LLRtmp[oi]<0);
				return nIteration_max + 1;
			}


			int start=0;
			int end=0;
			int width=0;
			for (int section=0;section<dWidth;section+=2)
			{
				end+=d[section];
				width=d[section+1];

				for( i=start;i<end;i++)
				{
					for( j=0;j<width;j++)
					{
						*(Q+i*VWidthMax+j)=LLRtmp[i]-*(R+i*VWidthMax+j);
					}
				}
				start+=d[section];
			}
		}
		}   // end else (flooding loop; layered branch above)
	}
	for( i=0;i<K;i++)
	{
		LLRo[i]=(LLRtmp[i]<0);
	}
	// Turbo-EQ keystone (RESEARCH_turbo-eq.md §4.2): expose the full a-posteriori
	// LLR vector for ALL N coded bits. LLRtmp[i] = LLRi[i] + Σ_j R[i][j] is computed
	// every BP iteration at :167-173 (and equals the input LLRi[i] when the frame
	// is already valid before iterating, :60). Copied here ONLY when app_llr is
	// non-null → zero cost + byte-identical for every existing caller.
	// The extrinsic for soft re-modulation is app_llr[i] − LLRi[i] (= Σ_j R[i][j],
	// the pure decoder contribution); the caller forms it.
	if(app_llr)
	{
		for(i=0;i<N;i++)
		{
			app_llr[i]=LLRtmp[i];
		}
	}
	return iteration;

}

/* F. R. Kschischang, B. J. Frey, and H. . Loeliger, "Factor graphs and the sum-product algorithm," IEEE Transactions on Information Theory, vol. 47, no. 2, pp. 498–519, Feb 2001.
 * https://ieeexplore.ieee.org/document/910572
 */
