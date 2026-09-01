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

#include "physical_layer/ldpc.h"
#include "debug/canary_guard.h"
#include <cstring>  // memset: zero decode workspace at alloc (PRECOOK memcmp determinism)

cl_ldpc::cl_ldpc()
{
	standard_val=0;
	N=0;
	K=0;
	P=0;
	decoding_algorithm_val=0;
	eta_val=0;
	nIteration_max_val=0;
	print_nIteration_val=0;
	Cwidth=0;
	r=0;
	decoding_algorithm=0;
	rate=0;
	framesize=0;
	standard=0;
	nIteration_max=0;
	GBF_eta=0;
	print_nIteration=NO;

	Cwidth=0;
	QCmatrixC=NULL;
	QCmatrixEnc=NULL;
	QCmatrixV=NULL;
	QCmatrixd=NULL;
	Q=NULL;
	Vwidth=0;
	R=NULL;
	V_pos=NULL;
	dwidth=0;
}

cl_ldpc::~cl_ldpc()
{
	deinit();
}

int cl_ldpc::init()
{
	deinit();
	standard_val=standard;
	N=framesize;
	decoding_algorithm_val=decoding_algorithm;
	eta_val=GBF_eta;
	nIteration_max_val=nIteration_max;
	print_nIteration_val= print_nIteration;
	if(N<=0 || N>N_MAX || !(rate>0.0f && rate<1.0f))
		return -1;
	K=(int)((float)N*rate);
	P=N-K;
	if(decoding_algorithm_val!=GBF && decoding_algorithm_val!=SPA)
		return -1;
	return update_code_parameters();
}

void cl_ldpc::deinit()
{
	standard_val=0;
	N=0;
	K=0;
	P=0;
	decoding_algorithm_val=0;
	eta_val=0;
	nIteration_max_val=0;
	print_nIteration_val=0;
	Cwidth=0;
	Vwidth=0;
	dwidth=0;
	QCmatrixC=NULL;
	QCmatrixEnc=NULL;
	QCmatrixV=NULL;
	QCmatrixd=NULL;

	CDELETE(R);
	CDELETE(Q);
	CDELETE(V_pos);

}


void cl_ldpc::encode(const int* data, int*  encoded_data)
 {
	if(encoded_data==NULL)
		return;

	if(data==NULL || QCmatrixEnc==NULL || N<=0 || N>N_MAX
		|| K<=0 || K>N || P<=0 || P!=N-K || Cwidth<2)
	{
		if(N>0 && N<=N_MAX)
			memset(encoded_data, 0, sizeof(int) * (size_t)N);
		return;
	}

 	int CwidthMax=Cwidth-1;
 	int* QCmatrixEnc_;
 	QCmatrixEnc_=QCmatrixEnc;
 	for(int i=0;i<K;i++)
 	{
 		encoded_data[i]=data[i];
 	}

 	for(int i=0;i<P;i++)
 	{
 		encoded_data[i+K]=0;
 		for(int j=0;j<Cwidth-1;j++)
 		{
 			if(*(QCmatrixEnc_+i*CwidthMax+j)!=-1)
 			{
 				encoded_data[i+K]^=encoded_data[*(QCmatrixEnc_+i*CwidthMax+j)];
 			}
 		}
 	}
 }


 int cl_ldpc::update_code_parameters()
  {
  	int success=-1;
  	if(standard_val==MERCURY)
  	{
  		if(N==MERCURY_NORMAL)
  		{
  			success=0;
  			if(K==100)//rate == 1/16
  			{
  				Cwidth=mercury_normal_Cwidth_1_16;
  				Vwidth=mercury_normal_Vwidth_1_16;
  				dwidth=mercury_normal_dwidth_1_16;
  				QCmatrixC=&mercury_normal_QCmatrixC_1_16[0][0];
  				QCmatrixEnc=&mercury_normal_QCmatrixEnc_1_16[0][0];
  				QCmatrixV=&mercury_normal_QCmatrixV_1_16[0][0];
  				QCmatrixd=&mercury_normal_QCmatrixd_1_16[0];
  				R=CNEW(double, N*Vwidth, "ldpc.R");
  				Q=CNEW(double, N*Vwidth, "ldpc.Q");
  			}
  			else if(K==200)//rate == 2/16
  			{
//  				Cwidth=mercury_met_Cwidth_2_16;
//  				Vwidth=mercury_met_Vwidth_2_16;
//  				dwidth=mercury_met_dwidth_2_16;
//  				QCmatrixC=&mercury_met_QCmatrixC_2_16[0][0];
//  				QCmatrixEnc=&mercury_met_QCmatrixEnc_2_16[0][0];
//  				QCmatrixV=&mercury_met_QCmatrixV_2_16[0][0];
//  				QCmatrixd=&mercury_met_QCmatrixd_2_16[0];

  				Cwidth=mercury_normal_Cwidth_2_16;
  				Vwidth=mercury_normal_Vwidth_2_16;
  				dwidth=mercury_normal_dwidth_2_16;
  				QCmatrixC=&mercury_normal_QCmatrixC_2_16[0][0];
  				QCmatrixEnc=&mercury_normal_QCmatrixEnc_2_16[0][0];
  				QCmatrixV=&mercury_normal_QCmatrixV_2_16[0][0];
  				QCmatrixd=&mercury_normal_QCmatrixd_2_16[0];
  				R=CNEW(double, N*Vwidth, "ldpc.R");
  				Q=CNEW(double, N*Vwidth, "ldpc.Q");
  			}
  			else if(K==300)//rate == 3/16
  			{
  				Cwidth=mercury_normal_Cwidth_3_16;
  				Vwidth=mercury_normal_Vwidth_3_16;
  				dwidth=mercury_normal_dwidth_3_16;
  				QCmatrixC=&mercury_normal_QCmatrixC_3_16[0][0];
  				QCmatrixEnc=&mercury_normal_QCmatrixEnc_3_16[0][0];
  				QCmatrixV=&mercury_normal_QCmatrixV_3_16[0][0];
  				QCmatrixd=&mercury_normal_QCmatrixd_3_16[0];
  				R=CNEW(double, N*Vwidth, "ldpc.R");
  				Q=CNEW(double, N*Vwidth, "ldpc.Q");
  			}
  			else if(K==400)//rate == 4/16
  			{
  				Cwidth=mercury_normal_Cwidth_4_16;
  				Vwidth=mercury_normal_Vwidth_4_16;
  				dwidth=mercury_normal_dwidth_4_16;
  				QCmatrixC=&mercury_normal_QCmatrixC_4_16[0][0];
  				QCmatrixEnc=&mercury_normal_QCmatrixEnc_4_16[0][0];
  				QCmatrixV=&mercury_normal_QCmatrixV_4_16[0][0];
  				QCmatrixd=&mercury_normal_QCmatrixd_4_16[0];
  				R=CNEW(double, N*Vwidth, "ldpc.R");
  				Q=CNEW(double, N*Vwidth, "ldpc.Q");
  			}
  			else if(K==500)//rate == 5/16
  			{
  				Cwidth=mercury_normal_Cwidth_5_16;
  				Vwidth=mercury_normal_Vwidth_5_16;
  				dwidth=mercury_normal_dwidth_5_16;
  				QCmatrixC=&mercury_normal_QCmatrixC_5_16[0][0];
  				QCmatrixEnc=&mercury_normal_QCmatrixEnc_5_16[0][0];
  				QCmatrixV=&mercury_normal_QCmatrixV_5_16[0][0];
  				QCmatrixd=&mercury_normal_QCmatrixd_5_16[0];
  				R=CNEW(double, N*Vwidth, "ldpc.R");
  				Q=CNEW(double, N*Vwidth, "ldpc.Q");
  			}
  			else if(K==600)//rate == 6/16
  			{
  				Cwidth=mercury_normal_Cwidth_6_16;
  				Vwidth=mercury_normal_Vwidth_6_16;
  				dwidth=mercury_normal_dwidth_6_16;
  				QCmatrixC=&mercury_normal_QCmatrixC_6_16[0][0];
  				QCmatrixEnc=&mercury_normal_QCmatrixEnc_6_16[0][0];
  				QCmatrixV=&mercury_normal_QCmatrixV_6_16[0][0];
  				QCmatrixd=&mercury_normal_QCmatrixd_6_16[0];
  				R=CNEW(double, N*Vwidth, "ldpc.R");
  				Q=CNEW(double, N*Vwidth, "ldpc.Q");
  			}
  			else if(K==800)//rate == 8/16
  			{
  				Cwidth=mercury_normal_Cwidth_8_16;
  				Vwidth=mercury_normal_Vwidth_8_16;
  				dwidth=mercury_normal_dwidth_8_16;
  				QCmatrixC=&mercury_normal_QCmatrixC_8_16[0][0];
  				QCmatrixEnc=&mercury_normal_QCmatrixEnc_8_16[0][0];
  				QCmatrixV=&mercury_normal_QCmatrixV_8_16[0][0];
  				QCmatrixd=&mercury_normal_QCmatrixd_8_16[0];
  				R=CNEW(double, N*Vwidth, "ldpc.R");
  				Q=CNEW(double, N*Vwidth, "ldpc.Q");
  			}
  			else if(K==1000)//rate == 10/16
  			{
  				Cwidth=mercury_normal_Cwidth_10_16;
  				Vwidth=mercury_normal_Vwidth_10_16;
  				dwidth=mercury_normal_dwidth_10_16;
  				QCmatrixC=&mercury_normal_QCmatrixC_10_16[0][0];
  				QCmatrixEnc=&mercury_normal_QCmatrixEnc_10_16[0][0];
  				QCmatrixV=&mercury_normal_QCmatrixV_10_16[0][0];
  				QCmatrixd=&mercury_normal_QCmatrixd_10_16[0];
  				R=CNEW(double, N*Vwidth, "ldpc.R");
  				Q=CNEW(double, N*Vwidth, "ldpc.Q");
  			}
  			else if(K==1200)//rate == 12/16
  			{
  				Cwidth=mercury_normal_Cwidth_12_16;
  				Vwidth=mercury_normal_Vwidth_12_16;
  				dwidth=mercury_normal_dwidth_12_16;
  				QCmatrixC=&mercury_normal_QCmatrixC_12_16[0][0];
  				QCmatrixEnc=&mercury_normal_QCmatrixEnc_12_16[0][0];
  				QCmatrixV=&mercury_normal_QCmatrixV_12_16[0][0];
  				QCmatrixd=&mercury_normal_QCmatrixd_12_16[0];
  				R=CNEW(double, N*Vwidth, "ldpc.R");
  				Q=CNEW(double, N*Vwidth, "ldpc.Q");
  			}
  			else if(K==1400)//rate == 14/16
  			{
  				Cwidth=mercury_normal_Cwidth_14_16;
  				Vwidth=mercury_normal_Vwidth_14_16;
  				dwidth=mercury_normal_dwidth_14_16;
  				QCmatrixC=&mercury_normal_QCmatrixC_14_16[0][0];
  				QCmatrixEnc=&mercury_normal_QCmatrixEnc_14_16[0][0];
  				QCmatrixV=&mercury_normal_QCmatrixV_14_16[0][0];
  				QCmatrixd=&mercury_normal_QCmatrixd_14_16[0];
  				R=CNEW(double, N*Vwidth, "ldpc.R");
  				Q=CNEW(double, N*Vwidth, "ldpc.Q");
  			}
  			else
  			{
  				std::cout<<"K="<<K<<" Wrong Code Rate"<<std::endl;
  				return -1;
  			}
  			if(R==NULL || Q==NULL)
  			{
  				std::cout<<"Memory allocation error"<<std::endl;
  				exit(2);
  			}
  			// Pre-allocate V_pos workspace for SPA decoder (eliminates per-frame heap churn)
  			V_pos=CNEW(int, P*Cwidth, "ldpc.V_pos");
  			// PRECOOK determinism: R/Q/V_pos are decode SCRATCH — their content is reset at the
  			// start of every decode (SPA message accumulators / node-position workspace), so `new[]`
  			// leaves indeterminate heap junk that the modem never reads. That junk made a raw memcmp
  			// of two INDEPENDENTLY-built ldpc objects diverge, breaking the precook bundle==init
  			// byte-identical gate (copy_from faithfully propagated the scratch's junk, but the
  			// reference build had its own). Zeroing here makes the workspace deterministic; decode
  			// output is unchanged (the first decode step overwrites these before reading them).
  			memset(R, 0, sizeof(double) * (size_t)N * (size_t)Vwidth);
  			memset(Q, 0, sizeof(double) * (size_t)N * (size_t)Vwidth);
  			if(V_pos!=NULL) memset(V_pos, 0, sizeof(int) * (size_t)P * (size_t)Cwidth);
  		}
  		// Step 15: MERCURY_SACK / MERCURY_SACK_LONG branches removed alongside
  		// the legacy MFSK SACK bitmap path (mercury_sack_*_16.cc tables deleted).

  	}
  	return success;
  }


 int cl_ldpc::decode(const float* data,  int*  decoded_data, double* app_llr)
 {
	 auto fail_decode = [&]() {
		 last_early_term_iter=-1;
		 if(decoded_data!=NULL && K>0 && K<=N && K<=N_MAX)
			 memset(decoded_data, 0, sizeof(int) * (size_t)K);
		 if(app_llr!=NULL && N>0 && N<=N_MAX)
			 memset(app_llr, 0, sizeof(double) * (size_t)N);
		 return nIteration_max_val + 1;
	 };

	 if(data==NULL || decoded_data==NULL || N<=0 || N>N_MAX
		 || K<=0 || K>N || P<=0 || P!=N-K
		 || Cwidth<2 || QCmatrixC==NULL)
		 return fail_decode();
	 if(decoding_algorithm_val==SPA
		 && (Vwidth<=0 || dwidth<=0 || QCmatrixV==NULL
			 || QCmatrixd==NULL || R==NULL || Q==NULL || V_pos==NULL))
		 return fail_decode();

	 int iterations_done=0;
 	if(decoding_algorithm_val==GBF)
 	{
 		// GBF has no soft output. The turbo loop (RESEARCH_turbo-eq.md §4.2 R2)
 		// forces SPA when MERCURY_TURBO_ITERS>1; if a config is GBF-only the loop
 		// no-ops to iteration 0 (app_llr left untouched → caller treats as no soft).
 		iterations_done=decode_GBF(data,decoded_data,QCmatrixC,Cwidth,Cwidth,N,K,P,nIteration_max_val,eta_val);
 	}
 	else if(decoding_algorithm_val==SPA)
 	{
 		// feat/turnaround-eff (fact-documents/turnaround-eff.md §2/§3/§4):
 		// select the shared non-convergence detector mode.
 		//   mode 2 (#1(c) eager)  when this is a SPECULATIVE wrong-position
 		//                         decode (early_term_speculative set by the
 		//                         sub-peak / extra-trial caller);
 		//   mode 1 (#3 standard)  DEFAULT-ON: MERCURY_SYND_EARLYTERM defaults to 1
 		//                         (env-absent => 1; see the getenv below), so a
 		//                         non-speculative decode runs mode 1 UNLESS the env
 		//                         is explicitly set to 0;
 		//   mode 0 (OFF)          only when MERCURY_SYND_EARLYTERM=0 is set explicitly
 		//                         => byte-identical (loop to the cap).
 		// Speculative TAKES PRECEDENCE (a wrong-position decode is doomed; bail
 		// eagerly even if #3's env is unset).
 		static const int synd_earlyterm_env = []{
 			const char* e = std::getenv("MERCURY_SYND_EARLYTERM");
 			return (e && *e) ? atoi(e) : 1;
 		}();
 		int et_mode = early_term_speculative ? 2 : (synd_earlyterm_env != 0 ? 1 : 0);
 		last_early_term_iter = -1;
 		iterations_done=decode_SPA(data,decoded_data,QCmatrixC,Cwidth,Cwidth, QCmatrixV,Vwidth,Vwidth,QCmatrixd,dwidth,R,Q,V_pos,N,K,P,nIteration_max_val,decode_abort,app_llr,et_mode,&last_early_term_iter);
 	}
	else
	{
		return fail_decode();
	}
 	return iterations_done;
 }
