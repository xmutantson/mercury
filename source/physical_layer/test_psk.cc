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


#include "physical_layer/psk.h"

void cl_psk::rescale_shaped_power(const double* sym_prob)
{
	if(constellation==NULL || nSymbols<=0 || sym_prob==NULL) return;
	// shaped average power under the given per-symbol prior (constellation is already
	// unit-uniform-power). E_shaped = sum_i p_i |x_i|². The freed energy (E_shaped<1)
	// is reclaimed by scaling all points by 1/sqrt(E_shaped) so the shaped power = 1.
	double e_shaped=0.0, psum=0.0;
	for(int i=0;i<nSymbols;i++)
	{
		double p = sym_prob[i];
		if(p<0.0) p=0.0;
		e_shaped += p*(constellation[i].real()*constellation[i].real()
		             + constellation[i].imag()*constellation[i].imag());
		psum += p;
	}
	if(psum<=0.0 || e_shaped<=0.0) return;
	double scale = 1.0/sqrt(e_shaped/psum);   // normalize by mean shaped power
	for(int i=0;i<nSymbols;i++) constellation[i]*=scale;
}


// PAS / PCS LLR de-shaping demapper (fact-documents/data-flow-pas-shaping.md §4).
// A literal copy of demod() with ONE addition: after the max-log LLR is written
// in per-symbol output order (out[i+j] = LLR of index-bit nBits-1-j), the
// a-priori log-prior log_prior[j] is added to that bit's LLR. For amplitude-bit
// positions log_prior[j] = ln P(bit=0) − ln P(bit=1) under the shaped (Maxwell-
// Boltzmann) distribution; for the (uniform) sign-bit positions log_prior[j]=0.
// Sign convention matches demod (LLR>0 ⇒ bit 0), so a positive log-prior biases
// toward bit 0. demod() above is UNCHANGED -> production is byte-identical.
void cl_psk::demod_pas(const std::complex <double> *in,int nItems,float *out,float variance,
                       const double* log_prior)
{
	float* D=D_buf;
	float* LLR=LLR_buf;
	float Dmin0,Dmin1;
	unsigned int mask;

	float eff_var = (variance > var_floor) ? variance : var_floor;

	for(int i=0;i<nItems;i+=nBits)
	{
		for(int j=0;j<nSymbols;j++)
		{
			D[j]=pow((real(*(in+i/nBits))-real(constellation[j])),2)+pow((imag(*(in+i/nBits))-imag(constellation[j])),2);
		}

		mask=1;
		for(int k=0;k<nBits;k++)
		{
			Dmin0=D[0];
			Dmin1=D[mask];

			for(int j=0;j<nSymbols;j++)
			{
				if((j & mask)==0)
				{
					if(D[j]<Dmin0)
					{
						Dmin0=D[j];
					}
				}
				if((j & mask)==mask)
				{
					if(D[j]<Dmin1)
					{
						Dmin1=D[j];
					}
				}
			}
			LLR[k]=(Dmin1-Dmin0)/eff_var;
			mask<<=1;
		}
		for(int j=0;j<nBits;j++)
		{
			// out position j corresponds to index-bit (nBits-1-j); add its log-prior.
			*(out+i+j)=LLR[nBits-j-1] + (float)(log_prior!=nullptr ? log_prior[j] : 0.0);
		}
	}
}

// Turbo-EQ soft re-modulation (RESEARCH_turbo-eq.md §3 + §4.3).
// Given per-bit a-priori LLRs from the decoder (SAME order/convention as demod()
// produces — see below), compute the soft symbol MEAN x̄_n = E[x_n] and VARIANCE
// v_n = E|x_n|² − |x̄_n|² for each data symbol, by an exact sum over the ≤32
// constellation points. x̄ is the VIRTUAL-PILOT value; v is its reliability (a
// converged symbol → |x̄|→1, v→0; an uncertain symbol → x̄→0, v→1, auto-ignored).
