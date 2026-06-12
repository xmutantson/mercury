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
#include <cstdlib>   // getenv (DX-1 env-gated diagnostic; default-off, byte-identical when off)
#include <cstdio>    // fprintf for the trajectory dump

// DX-1 diagnostic (default OFF, byte-identical when off): when MERCURY_LDPC_DX is set
// in the environment, decode_SPA emits, for each decoded codeword, the per-iteration
// unsatisfied-parity-check count trajectory nOnes[1..iter] to stderr. This is the
// oscillation / trapping-set classifier from the LDPC-decoder DX investigation
// (_research/LDPC_DECODER_DX_VERDICT.json). It reads ONLY the nOnes value the decoder
// already computes (this file, ~line 189) and writes ONLY when the env var is present;
// it never alters R/Q/LLR state or the return value, so the decode is bit-for-bit
// unchanged when the flag is unset. One static read of the env on first call.
static int dx_ldpc_log_enabled()
{
	static int s = -1;
	if (s == -1) { const char* e = std::getenv("MERCURY_LDPC_DX"); s = (e && *e && atoi(e) != 0) ? 1 : 0; }
	return s;
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
		std::atomic<bool>* abort_flag
)
{
	int Cout[N_MAX];
	int LLRbin[N_MAX];
	int iteration=0;
	int i,j,nOnes;
	double LLRtmp[N_MAX];

	// DX-1 (default-off): per-iteration unsatisfied-check trajectory buffer. Only touched
	// when the env flag is set; otherwise this is a single cheap branch that records nothing.
	const int dx_log = dx_ldpc_log_enabled();
	int dx_traj[256];
	int dx_traj_len = 0;

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

		for(iteration=1;iteration<=nIteration_max;iteration++)
		{
			// Early exit: another parallel decoder already succeeded
			if(abort_flag && abort_flag->load(std::memory_order_relaxed))
				return -iteration;

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

			// DX-1 (default-off): record this iteration's unsatisfied-check count.
			if(dx_log && dx_traj_len < 256)
			{
				dx_traj[dx_traj_len++] = nOnes;
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

	// DX-1 (default-off): emit the per-iteration nOnes trajectory for this codeword.
	// Format: "[LDPC-DX] iters=<I> final_nOnes=<W> conv=<0|1> traj=w1|w2|...". A converged
	// frame ends with final_nOnes 0; a non-converging (iter=max) frame shows the limit-cycle
	// or high-plateau the DX investigation classifies. Writing only — decode state untouched.
	if(dx_log && dx_traj_len>0)
	{
		int final_w = dx_traj[dx_traj_len-1];
		fprintf(stderr,"[LDPC-DX] N=%d K=%d iters=%d final_nOnes=%d conv=%d traj=",
				N,K,iteration,final_w,(final_w==0)?1:0);
		for(int t=0;t<dx_traj_len;t++)
			fprintf(stderr,"%d%s",dx_traj[t],(t+1<dx_traj_len)?"|":"");
		fprintf(stderr,"\n");
	}

	return iteration;

}

/* F. R. Kschischang, B. J. Frey, and H. . Loeliger, "Factor graphs and the sum-product algorithm," IEEE Transactions on Information Theory, vol. 47, no. 2, pp. 498–519, Feb 2001.
 * https://ieeexplore.ieee.org/document/910572
 */
