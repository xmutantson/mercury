#include <cmath>
#include <cstdio>
#include "datalink_layer/optimizer_metrics.h"
static int failures=0;
static void nearv(const char* n,double g,double w,double t){if(std::fabs(g-w)>t){std::fprintf(stderr,"FAIL %s got %.9f want %.9f\n",n,g,w);++failures;}}
static void eqi(const char* n,int g,int w){if(g!=w){std::fprintf(stderr,"FAIL %s got %d want %d\n",n,g,w);++failures;}}
int main(){
 const int N=4;
 unsigned int app[N]={1000,0,9999,0};
 unsigned int tr[N]={1000,900,9999,0};
 unsigned int ms[N]={1000,1000,0,0};
 unsigned char rv[N]={1,1,0,0};
 unsigned char ov[N]={1,1,1,0};
 unsigned char ac[N]={1,0,0,0};
 unsigned char sack[N]={0,1,1,0};
 unsigned char fail[N]={0,0,0,0};
 unsigned int ack[N]={30,27,1,0};
 unsigned int sent[N]={30,30,30,0};
 st_optimizer_window_metrics m=optimizer_reduce_window(app,tr,ms,rv,ov,ac,sack,fail,ack,sent,3,3,N);
 eqi("rate n",m.rate_sample_count,2); eqi("outcome n",m.outcome_sample_count,3); eqi("app commits",m.application_commit_count,1);
 nearv("app bps",m.application_bps,4000.0,1e-9); nearv("transport bps",m.transport_bps,7600.0,1e-9);
 nearv("sack incidence",m.sack_batch_rate,2.0/3.0,1e-12); nearv("frame success",m.frame_success_rate,58.0/90.0,1e-12);
 nearv("partial loss",m.partial_frame_loss_rate,(3.0+29.0)/60.0,1e-12); nearv("failure",m.failed_batch_rate,0.0,1e-12);
 // A valid zero-delivery timed failure must remain in both populations and rate denominator.
 app[2]=0; tr[2]=0; ms[2]=1000; rv[2]=1; fail[2]=1; sack[2]=0; ack[2]=0; sent[2]=30;
 m=optimizer_reduce_window(app,tr,ms,rv,ov,ac,sack,fail,ack,sent,3,3,N);
 eqi("zero rate n",m.rate_sample_count,3); eqi("zero outcome n",m.outcome_sample_count,3);
 nearv("zero app bps",m.application_bps,1000.0*8000.0/3000.0,1e-9); nearv("zero fail",m.failed_batch_rate,1.0/3.0,1e-12);
 // Outcome-only replay contributes delivery evidence without inventing airtime.
 app[3]=0; tr[3]=0; ms[3]=0; rv[3]=0; ov[3]=1; ac[3]=0; sack[3]=0; fail[3]=0; ack[3]=10; sent[3]=10;
 m=optimizer_reduce_window(app,tr,ms,rv,ov,ac,sack,fail,ack,sent,0,4,N);
 eqi("outcome-only rate n",m.rate_sample_count,3); eqi("outcome-only outcome n",m.outcome_sample_count,4);
 // Application credit attaches to one explicit real record and is idempotent.
 unsigned int capp[N]={0,0,0,0}; unsigned char cac[N]={0,0,0,0};
 unsigned char crv[N]={1,0,1,0};
 eqi("commit exact real slot", optimizer_commit_application_to_slot(capp,cac,crv,N,2,777)?1:0,1);
 eqi("commit bytes exact", (int)capp[2],777);
 eqi("duplicate commit refused", optimizer_commit_application_to_slot(capp,cac,crv,N,2,777)?1:0,0);
 eqi("outcome-only slot cannot steal commit", optimizer_commit_application_to_slot(capp,cac,crv,N,1,55)?1:0,0);
 if(failures) return 1;
 std::puts("PASS optimizer_metrics v2 populations + zero outcomes");
 return 0;
}
