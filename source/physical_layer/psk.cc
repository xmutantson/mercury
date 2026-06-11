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



cl_psk::cl_psk()
{
	constellation=NULL;
	D_buf=NULL;
	LLR_buf=NULL;
	nBits=0;
	nSymbols=0;
	var_floor=0.001f;  // Phase-2 flag default = HEAD (b806b76); pre-IONOS was 0.05
}

cl_psk::~cl_psk()
{
	deinit();
	constellation=NULL;
	D_buf=NULL;
	LLR_buf=NULL;
	nBits=0;
	nSymbols=0;
}

void cl_psk::deinit()
{
	if(constellation!=NULL)
	{
		delete[] constellation;
		constellation=NULL;
	}
	if(D_buf!=NULL)
	{
		delete[] D_buf;
		D_buf=NULL;
	}
	if(LLR_buf!=NULL)
	{
		delete[] LLR_buf;
		LLR_buf=NULL;
	}
}

void cl_psk::set_predefined_constellation(int M)
{
	std::complex <double>* _constellation =new std::complex <double>[M];
	if(M==MOD_BPSK)
	{
		_constellation[0]=std::complex <double> ( 1 , 0 );
		_constellation[1]=std::complex <double> ( -1 , 0 );
	}
	else if(M==MOD_QPSK)
	{
		_constellation[0]=std::complex <double> ( -1,1);
		_constellation[1]=std::complex <double> ( -1,-1);
		_constellation[2]=std::complex <double> ( 1,1);
		_constellation[3]=std::complex <double> ( 1,-1);

	}
	else if(M==MOD_8PSK)
	{
		_constellation[0]=std::complex <double> ( -1,-1) * sqrt(2.0)/2.0;
		_constellation[1]=std::complex <double> ( -1,0);
		_constellation[2]=std::complex <double> ( 0,1);
		_constellation[3]=std::complex <double> ( -1,1) * sqrt(2.0)/2.0;
		_constellation[4]=std::complex <double> ( 0,-1);
		_constellation[5]=std::complex <double> ( 1,-1) * sqrt(2.0)/2.0;
		_constellation[6]=std::complex <double> ( 1,1) * sqrt(2.0)/2.0;
		_constellation[7]=std::complex <double> ( 1,0);
	}
//	else if(M==MOD_8QAM)
//	{
//		_constellation[0]=std::complex <double> ( -3,1);
//		_constellation[1]=std::complex <double> ( -3,-1);
//		_constellation[2]=std::complex <double> ( -1,1);
//		_constellation[3]=std::complex <double> ( -1,-1);
//		_constellation[4]=std::complex <double> ( 3,1);
//		_constellation[5]=std::complex <double> ( 3,-1);
//		_constellation[6]=std::complex <double> ( 1,1);
//		_constellation[7]=std::complex <double> ( 1,-1);
//	}
	else if(M==MOD_16QAM)
	{
		_constellation[0]=std::complex <double> ( -3,3);
		_constellation[1]=std::complex <double> ( -3,1);
		_constellation[2]=std::complex <double> ( -3,-3);
		_constellation[3]=std::complex <double> ( -3,-1);
		_constellation[4]=std::complex <double> ( -1,3);
		_constellation[5]=std::complex <double> ( -1,1);
		_constellation[6]=std::complex <double> ( -1,-3);
		_constellation[7]=std::complex <double> ( -1,-1);
		_constellation[8]=std::complex <double> ( 3,3);
		_constellation[9]=std::complex <double> ( 3,1);
		_constellation[10]=std::complex <double> ( 3,-3);
		_constellation[11]=std::complex <double> ( 3,-1);
		_constellation[12]=std::complex <double> ( 1,3);
		_constellation[13]=std::complex <double> ( 1,1);
		_constellation[14]=std::complex <double> ( 1,-3);
		_constellation[15]=std::complex <double> ( 1,-1);
	}
	else if(M==MOD_32QAM)
	{
		_constellation[0]=std::complex <double> ( -3,5);
		_constellation[1]=std::complex <double> ( -1,5);
		_constellation[2]=std::complex <double> ( -3,-5);
		_constellation[3]=std::complex <double> ( -1,-5);
		_constellation[4]=std::complex <double> ( -5,3);
		_constellation[5]=std::complex <double> ( -5,1);
		_constellation[6]=std::complex <double> ( -5,-3);
		_constellation[7]=std::complex <double> ( -5,-1);
		_constellation[8]=std::complex <double> ( -1,3);
		_constellation[9]=std::complex <double> ( -1,1);
		_constellation[10]=std::complex <double> ( -1,-3);
		_constellation[11]=std::complex <double> ( -1,-1);
		_constellation[12]=std::complex <double> ( -3,3);
		_constellation[13]=std::complex <double> ( -3,1);
		_constellation[14]=std::complex <double> ( -3,-3);
		_constellation[15]=std::complex <double> ( -3,-1);
		_constellation[16]=std::complex <double> ( 3,5);
		_constellation[17]=std::complex <double> ( 1,5);
		_constellation[18]=std::complex <double> ( 3,-5);
		_constellation[19]=std::complex <double> ( 1,-5);
		_constellation[20]=std::complex <double> ( 5,3);
		_constellation[21]=std::complex <double> ( 5,1);
		_constellation[22]=std::complex <double> ( 5,-3);
		_constellation[23]=std::complex <double> ( 5,-1);
		_constellation[24]=std::complex <double> ( 1,3);
		_constellation[25]=std::complex <double> ( 1,1);
		_constellation[26]=std::complex <double> ( 1,-3);
		_constellation[27]=std::complex <double> ( 1,-1);
		_constellation[28]=std::complex <double> ( 3,3);
		_constellation[29]=std::complex <double> ( 3,1);
		_constellation[30]=std::complex <double> ( 3,-3);
		_constellation[31]=std::complex <double> ( 3,-1);

	}
	else if(M==MOD_64QAM)
	{
		_constellation[0]=std::complex <double> ( -7,7);
		_constellation[1]=std::complex <double> ( -7,5);
		_constellation[2]=std::complex <double> ( -7,1);
		_constellation[3]=std::complex <double> ( -7,3);
		_constellation[4]=std::complex <double> ( -7,-7);
		_constellation[5]=std::complex <double> ( -7,-5);
		_constellation[6]=std::complex <double> ( -7,-1);
		_constellation[7]=std::complex <double> ( -7,-3);
		_constellation[8]=std::complex <double> ( -5,7);
		_constellation[9]=std::complex <double> ( -5,5);
		_constellation[10]=std::complex <double> ( -5,1);
		_constellation[11]=std::complex <double> ( -5,3);
		_constellation[12]=std::complex <double> ( -5,-7);
		_constellation[13]=std::complex <double> ( -5,-5);
		_constellation[14]=std::complex <double> ( -5,-1);
		_constellation[15]=std::complex <double> ( -5,-3);
		_constellation[16]=std::complex <double> ( -1,7);
		_constellation[17]=std::complex <double> ( -1,5);
		_constellation[18]=std::complex <double> ( -1,1);
		_constellation[19]=std::complex <double> ( -1,3);
		_constellation[20]=std::complex <double> ( -1,-7);
		_constellation[21]=std::complex <double> ( -1,-5);
		_constellation[22]=std::complex <double> ( -1,-1);
		_constellation[23]=std::complex <double> ( -1,-3);
		_constellation[24]=std::complex <double> ( -3,7);
		_constellation[25]=std::complex <double> ( -3,5);
		_constellation[26]=std::complex <double> ( -3,1);
		_constellation[27]=std::complex <double> ( -3,3);
		_constellation[28]=std::complex <double> ( -3,-7);
		_constellation[29]=std::complex <double> ( -3,-5);
		_constellation[30]=std::complex <double> ( -3,-1);
		_constellation[31]=std::complex <double> ( -3,-3);
		_constellation[32]=std::complex <double> ( 7,7);
		_constellation[33]=std::complex <double> ( 7,5);
		_constellation[34]=std::complex <double> ( 7,1);
		_constellation[35]=std::complex <double> ( 7,3);
		_constellation[36]=std::complex <double> ( 7,-7);
		_constellation[37]=std::complex <double> ( 7,-5);
		_constellation[38]=std::complex <double> ( 7,-1);
		_constellation[39]=std::complex <double> ( 7,-3);
		_constellation[40]=std::complex <double> ( 5,7);
		_constellation[41]=std::complex <double> ( 5,5);
		_constellation[42]=std::complex <double> ( 5,1);
		_constellation[43]=std::complex <double> ( 5,3);
		_constellation[44]=std::complex <double> ( 5,-7);
		_constellation[45]=std::complex <double> ( 5,-5);
		_constellation[46]=std::complex <double> ( 5,-1);
		_constellation[47]=std::complex <double> ( 5,-3);
		_constellation[48]=std::complex <double> ( 1,7);
		_constellation[49]=std::complex <double> ( 1,5);
		_constellation[50]=std::complex <double> ( 1,1);
		_constellation[51]=std::complex <double> ( 1,3);
		_constellation[52]=std::complex <double> ( 1,-7);
		_constellation[53]=std::complex <double> ( 1,-5);
		_constellation[54]=std::complex <double> ( 1,-1);
		_constellation[55]=std::complex <double> ( 1,-3);
		_constellation[56]=std::complex <double> ( 3,7);
		_constellation[57]=std::complex <double> ( 3,5);
		_constellation[58]=std::complex <double> ( 3,1);
		_constellation[59]=std::complex <double> ( 3,3);
		_constellation[60]=std::complex <double> ( 3,-7);
		_constellation[61]=std::complex <double> ( 3,-5);
		_constellation[62]=std::complex <double> ( 3,-1);
		_constellation[63]=std::complex <double> ( 3,-3);
	}
	set_constellation(_constellation,M);
	delete[] _constellation;
}

void cl_psk::set_constellation(std::complex <double> *_constellation, int size)
{
	float power_normalization_value=0;

	constellation=new std::complex <double>[size];
	nSymbols=size;
	nBits=(int)log2(nSymbols);

	// Pre-allocate demod workspace buffers (eliminates per-frame heap churn)
	D_buf=new float[nSymbols];
	LLR_buf=new float[nBits];

	for(int i=0;i<size;i++)
	{
		constellation[i]=*(_constellation+i);
	}

	for(int i=0;i<nSymbols;i++)
	{
		power_normalization_value+=pow(constellation[i].real(),2)+pow(constellation[i].imag(),2);
	}
	power_normalization_value=1/(sqrt(power_normalization_value/nSymbols));

	for(int i=0;i<nSymbols;i++)
	{
		constellation[i]*=power_normalization_value;
	}
}


void cl_psk::mod(const int *in,int nItems,std::complex <double> *out)
{
	for(int i=0;i<nItems;i+=nBits)
	{
		unsigned int const_loc=0;
		for(int j=0;j<nBits;j++)
		{
			const_loc+=*(in+i+j);
			const_loc<<=1;
		}
		const_loc>>=1;
		*(out+i/nBits)=constellation[const_loc];
	}
}





void cl_psk::demod(const std::complex <double> *in,int nItems,float *out,float variance)
{

	float* D=D_buf;
	float* LLR=LLR_buf;
	float Dmin0,Dmin1;
	unsigned int mask;

	// Floor variance to prevent LLR overflow (±inf breaks LDPC SPA convergence).
	// With LS channel estimation, noise_variance_estimate is always > 0, so
	// the floor only needs to prevent numerical issues. Value 0.001 allows
	// effective SNR up to 30 dB while keeping max |LLR| bounded.
	// Phase-2: --psk-var-floor=F overrides.
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
			*(out+i+j)=LLR[nBits-j-1];
		}
	}

}

// Turbo-EQ soft re-modulation (RESEARCH_turbo-eq.md §3 + §4.3).
// Given per-bit a-priori LLRs from the decoder (SAME order/convention as demod()
// produces — see below), compute the soft symbol MEAN x̄_n = E[x_n] and VARIANCE
// v_n = E|x_n|² − |x̄_n|² for each data symbol, by an exact sum over the ≤32
// constellation points. x̄ is the VIRTUAL-PILOT value; v is its reliability (a
// converged symbol → |x̄|→1, v→0; an uncertain symbol → x̄→0, v→1, auto-ignored).
//
// Bit order / sign convention (must match mod()/demod() exactly):
//   - mod() (psk.cc:260): in[i+0] is the MSB of the constellation index, in[i+nBits-1]
//     the LSB:  const_loc = in[i+0]<<(nBits-1) | … | in[i+nBits-1].
//   - demod() (psk.cc:328-331): out[i+j] = LLR for constellation index bit
//     (nBits-1-j) [LLR[k] uses mask=1<<k]. So input llr[i+j] here ↔ index-bit
//     k = nBits-1-j — the inverse of demod's reorder, so a demod→soft_remod
//     round-trip at high |LLR| reproduces the transmitted symbol (R6 unit test).
//   - Sign: LLR>0 ⇒ bit 0 (ldpc_decoder_SPA.cc:59, demod LLR=(Dmin1−Dmin0)/var).
//     P(bit=0)=1/(1+e^{−L}), P(bit=1)=1/(1+e^{+L}).
// Tüchler/Singer 2002 (soft-symbol mean/variance from a-priori LLRs); Kim/Park 2011.
void cl_psk::soft_remod(const float* llr, int nItems, std::complex<double>* xbar_out, double* v_out)
{
	for(int i=0;i<nItems;i+=nBits)
	{
		int sym = i/nBits;
		// Per index-bit-position k (0=LSB .. nBits-1=MSB), the a-priori P(bit_k=1).
		// llr[i + (nBits-1-k)] is the LLR of index bit k (the demod() reorder inverse).
		double p1[32];   // nBits <= log2(64)=6 here; MOD_64QAM is the max
		for(int k=0;k<nBits;k++)
		{
			double L = (double)llr[i + (nBits-1-k)];
			// Clip to keep exp() bounded (matches the ±40 LLR clamp telecom_system.cc).
			if(L >  40.0) L =  40.0;
			else if(L < -40.0) L = -40.0;
			// P(bit=1) = 1/(1+e^{+L})  (LLR>0 ⇒ bit 0).
			p1[k] = 1.0 / (1.0 + std::exp(L));
		}
		std::complex<double> mean(0.0,0.0);
		double e_abs2 = 0.0;
		for(int a=0;a<nSymbols;a++)
		{
			// P(symbol a) = Π_k P(bit_k = bit_k(a)). a's bit k is ((a>>k)&1).
			double pa = 1.0;
			for(int k=0;k<nBits;k++)
			{
				int bit_k = (a>>k)&1;
				pa *= bit_k ? p1[k] : (1.0 - p1[k]);
			}
			mean   += pa * constellation[a];
			e_abs2 += pa * (constellation[a].real()*constellation[a].real()
			              + constellation[a].imag()*constellation[a].imag());
		}
		double v = e_abs2 - (mean.real()*mean.real() + mean.imag()*mean.imag());
		if(v < 0.0) v = 0.0;   // numerical guard (variance is non-negative)
		xbar_out[sym] = mean;
		v_out[sym]    = v;
	}
}

std::complex <double> cl_psk::slice_nearest(std::complex <double> y) const
{
	if(constellation==NULL || nSymbols<=0) return std::complex<double>(0,0);
	int best=0; double bestd=1e300;
	for(int j=0;j<nSymbols;j++)
	{
		double dr=real(y)-real(constellation[j]);
		double di=imag(y)-imag(constellation[j]);
		double d=dr*dr+di*di;
		if(d<bestd){ bestd=d; best=j; }
	}
	return constellation[best];
}


