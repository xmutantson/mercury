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
#include "physical_layer/ldpc_generator_1_16.h"
#include "debug/canary_guard.h"

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
	osd_norder_val=1;
	osd_maxosd_val=0;
	dense_G_1_16=nullptr;
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

void cl_ldpc::init()
{
	standard_val=standard;
	N=framesize;
	K=(int)((float)N*rate);
	P=N-K;
	decoding_algorithm_val=decoding_algorithm;
	eta_val=GBF_eta;
	nIteration_max_val=nIteration_max;
	osd_norder_val=osd_norder;
	osd_maxosd_val=osd_maxosd;
	print_nIteration_val= print_nIteration;
	Cwidth=0;
	update_code_parameters();

	// Phase A.2 §7.5 item 5: amortize dense_G build outside the decode hot path.
	// Only build for rate 1/16 — that's the only code BP_OSD currently targets
	// (ROBUST_0/1). Idempotent via std::call_once inside the helper, so the
	// per-rate cost is paid once per process regardless of how many cl_ldpc
	// instances exist (e.g. parallel monitor decoders).
	if (decoding_algorithm_val == BP_OSD)
	{
		if (K == K_RATE_1_16 && N == N_RATE_1_16)
		{
			dense_G_1_16 = ldpc_get_dense_G_1_16();
		}
		else
		{
			// [robust3-feas] BP_OSD on a non-1/16 rate (e.g. ROBUST_3 rate 8/16,
			// K=800). The a26 branch only had a rate-1/16 dense generator; the
			// generic builder constructs G[K*N] for any Mercury matrix family by
			// encoding K one-hot vectors at the matching rate. dense_G_1_16 just
			// holds "the dense G for this instance's rate" (name kept for the
			// decode() call site). NULL if the rate has no matrix family.
			dense_G_1_16 = ldpc_get_dense_G_generic(K, N, rate);
			if (dense_G_1_16 == nullptr)
				std::cout << "[BP-OSD] WARN dense_G not built for K=" << K
				          << " N=" << N << " rate=" << rate
				          << "; decode() will return LDPC_BP_OSD_FAIL." << std::endl;
		}
	}
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
	osd_norder_val=1;
	osd_maxosd_val=0;
	print_nIteration_val=0;
	// dense_G_1_16 points into a process-static buffer owned by
	// ldpc_generator_1_16.cc — do NOT delete; just drop our handle.
	dense_G_1_16=nullptr;
	Cwidth=0;

	Cwidth=0;
	QCmatrixC=NULL;
	QCmatrixEnc=NULL;

	CDELETE(R);
	CDELETE(Q);
	CDELETE(V_pos);

}


void cl_ldpc::encode(const int* data, int*  encoded_data)
 {
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
  	int success=0;
  	if(standard_val==MERCURY)
  	{
  		if(N==MERCURY_NORMAL)
  		{
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
  				std::cout<<"K="<<K<<" Wrong Code Rate"<<std::endl<<"Exiting.."<<std::endl;
  				success=-1;
  				exit(1);
  			}
  			if(R==NULL || Q==NULL)
  			{
  				std::cout<<"Memory allocation error"<<std::endl;
  				exit(2);
  			}
  			// Pre-allocate V_pos workspace for SPA decoder (eliminates per-frame heap churn)
  			V_pos=CNEW(int, P*Cwidth, "ldpc.V_pos");
  		}
  		// Step 15: MERCURY_SACK / MERCURY_SACK_LONG branches removed alongside
  		// the legacy MFSK SACK bitmap path (mercury_sack_*_16.cc tables deleted).

  	}
  	return success;
  }


 int cl_ldpc::decode(const float* data,  int*  decoded_data)
 {
	 int iterations_done=0;
 	if(decoding_algorithm_val==GBF)
 	{
 		iterations_done=decode_GBF(data,decoded_data,QCmatrixC,Cwidth,Cwidth,N,K,P,nIteration_max_val,eta_val);
 	}
 	else if(decoding_algorithm_val==SPA)
 	{
 		iterations_done=decode_SPA(data,decoded_data,QCmatrixC,Cwidth,Cwidth, QCmatrixV,Vwidth,Vwidth,QCmatrixd,dwidth,R,Q,V_pos,N,K,P,nIteration_max_val,decode_abort);
 	}
 	else if(decoding_algorithm_val==BP_OSD)
 	{
 		// Phase A.2 §7.5 item 2: BP+OSD cascade. nIteration_max_val is reused
 		// as the BP iter cap (per integration brief). OSD knobs come from the
 		// per-config osd_norder/osd_maxosd captured at init().
 		//
 		// dense_G_1_16 is built at init() above; if it's still NULL we lack
 		// a generator matrix for this rate — return LDPC_BP_OSD_FAIL which
 		// the helper ldpc_decode_failed() will classify as failure. The
 		// is_robust_config gate in telecom_system.cc keeps BP_OSD off of
 		// non-1/16 codes in production, so this path is purely defensive.
 		if (dense_G_1_16 == nullptr)
 		{
 			iterations_done = LDPC_BP_OSD_FAIL;
 		}
 		else
 		{
 			iterations_done = decode_BP_OSD(
 				data, decoded_data,
 				QCmatrixC, Cwidth, Cwidth,
 				QCmatrixV, Vwidth, Vwidth,
 				dense_G_1_16,
 				/*apmask=*/nullptr,
 				N, K, P,
 				nIteration_max_val,
 				osd_norder_val,
 				osd_maxosd_val,
 				decode_abort
 			);
 		}
 	}
 	return iterations_done;
 }
