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
		double* app_llr
)
{
	int Cout[N_MAX];
	int LLRbin[N_MAX];
	int iteration=0;
	int i,j,nOnes;
	double LLRtmp[N_MAX];

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
