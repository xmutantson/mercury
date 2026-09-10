#define _POSIX_C_SOURCE 200809L
/*
 * Native real-audio IONOS bridge.  This is a C port of the staged
 * realaudio_bridge_s32.py + sim_channel_relay.py Channel path.
 *
 * Build from the repository root with `make realaudio-bridge`, or directly
 * with `make -C tools/sim/realaudio`.  Mercury's build.sh does not build this
 * helper; fleet deployments build it explicitly.
 *
 * Wire contract: raw ALSA S32_LE, 48000 Hz, stereo; capture channel zero is
 * divided by INT_MAX, impaired as float64, multiplied by INT_MAX, clamped to
 * [-INT_MAX, INT_MAX], truncated toward zero, and duplicated to both channels.
 */
#include <alsa/asoundlib.h>
#include <alloca.h>
#include <complex.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "realaudio_bridge_s32_c currently requires a little-endian target for S32_LE"
#endif

#define RATE 48000
#define PERIOD 1024
#define F_NYQUIST 24000.0
#define BW_NOISE 3000.0
#define INT_MAX_D 2147483647.0
#define FIR_NH 128
#define HILBERT_N 129
#define PI 3.141592653589793238462643383279502884

static volatile sig_atomic_t stop_requested;

typedef enum { PROF_WGN, PROF_FLAT, PROF_MPG, PROF_MPM, PROF_MPP, PROF_MPD } profile_t;
typedef enum { PSIG_STEADY, PSIG_FIX, PSIG_PEAK } psig_mode_t;

typedef struct {
    char fwd_cap[256], fwd_play[256], rev_cap[256], rev_play[256];
    char profile_name[16], cell[64], statsfile[1024];
    char axis[64], input_coordinate[32], binary_sha256[80], recipe_sha256[80];
    char s32_mode[32];
    profile_t profile;
    double snr, commanded_snr, snr_offset, cfo_hz, phase_noise_deg;
    double fade_depth_db, loss, sig_ref, bp_lo, bp_hi;
    double configured_bandwidth_hz, cn_config_db;
    int seed, cap_periods, play_periods, prime_periods, bp_taps;
    bool passthrough, burst, dry_run, self_test, format_only;
    char vector_in[1024], vector_out[1024], double_in[1024], double_out[1024];
    char tap_out[1024];
    size_t tap_count,tap_stride;
} options_t;

/* IONOS firmware 128-tap adjusted-Gaussian Doppler FIR, verbatim. */
static const double gaus_fir[FIR_NH] = {
  1.1755592671332046e-11,2.0188004956137427e-10,1.7236333623946176e-09,9.815423109243151e-09,
  4.219820040519088e-08,1.4693429486234634e-07,4.338503956649552e-07,1.122118806000393e-06,
  2.604091536729611e-06,5.522713327023963e-06,1.0857390465642334e-05,2.0011412649247494e-05,
  3.4893068706162795e-05,5.7982477259392286e-05,9.237678934582388e-05,0.00014180767284007714,
  0.00021062669000635658,0.0003037561533907518,0.0004266051229102419,0.000584952237938201,
  0.0007847989367901134,0.001032198204838678,0.001333065243166745,0.001692977321877409,
  0.002116970561258896,0.002609341477645015,0.003173460865247882,0.00381160700116864,
  0.004524824309342139,0.005312812558055807,0.006173850455688009,0.007104756211217515,
  0.008100886298035966,0.00915617235512072,0.010263194925878348,0.01141329161173599,
  0.012596696236577472,0.013802704802828978,0.015019863385625786,0.016236172665450826,
  0.01743930354214716,0.01861681819809535,0.019756391073966928,0.020846024470695095,
  0.021874253876569306,0.02283033861665188,0.023704434009553566,0.02448774186995001,
  0.02517263689027796,0.025752767148979578,0.026223127704178725,0.026580106921515002,
  0.02682150583616039,0.026946531447567135,0.026955765379786157,0.02685110980161695,
  0.026635712883536795,0.026313876369100774,0.025890948056565766,0.025373202123371387,
  0.024767710285281262,0.024082206768594926,0.02332494999439922,0.022504583735910823,
  0.02163000032189053,0.020710208229666707,0.019754206149457228,0.018770865316331563,
  0.01776882160593159,0.016756378583127243,0.01574142238665159,0.01473134903422507,
  0.013733004447687326,0.012752637231260112,0.011795863992392318,0.010867646776884902,
  0.009972282000446704,0.009113400098909145,0.008293974989634013,0.007516342337046732,
  0.006782225544921226,0.006092768355660706,0.005448572920512081,0.004849742212182516,
  0.004295925680163602,0.003786367096475506,0.003319953602663025,0.002895265044807468,
  0.002510622769190125,0.002164137144270543,0.0018537531721837,0.001577293652557459,
  0.001332499460868328,0.001117066600796574,0.0009286797833839573,0.0007650423737761393,
  0.0006239026277671727,0.0005030762143350815,0.0004004650862100223,0.0003140728178353742,
  0.00024201657867910556,0.00018253594974316996,0.00013399882249807025,9.49046426887381e-05,
  6.388527699708468e-05,3.970378899074398e-05,2.1251412801076355e-05,7.543009276742865e-06,
  -2.2887192936932196e-06,-8.999992637884094e-06,-1.3243447148502327e-05,-1.557613368708468e-05,
  -1.6467811426850445e-05,-1.63093528492861e-05,-1.5421097939455242e-05,-1.4061018704922785e-05,
  -1.243257788819571e-05,-1.0692187735418308e-05,-8.956195575428226e-06,-7.3073424710351094e-06,
  -5.8006591112396305e-06,-4.468779263172823e-06,-3.326665397016021e-06,-2.3757534892377295e-06,
  -1.6075344987453848e-06,-1.0065986371058506e-06,-5.531753923550552e-07,-2.252074190014706e-07
};

typedef struct { uint64_t s; bool spare_valid; double spare; } rng_t;

static uint64_t splitmix_next(rng_t *r) {
    uint64_t z;
    r->s += UINT64_C(0x9E3779B97F4A7C15);
    z = r->s;
    z = (z ^ (z >> 30)) * UINT64_C(0xBF58476D1CE4E5B9);
    z = (z ^ (z >> 27)) * UINT64_C(0x94D049BB133111EB);
    return z ^ (z >> 31);
}
static void rng_init(rng_t *r, uint64_t seed) {
    memset(r, 0, sizeof(*r));
    r->s = seed * UINT64_C(0x9E3779B97F4A7C15);
}
static double rng_uniform(rng_t *r) {
    return (double)(splitmix_next(r) >> 11) * (1.0 / 9007199254740992.0);
}
static double rng_gauss(rng_t *r) {
    if (r->spare_valid) { r->spare_valid = false; return r->spare; }
    double u1 = rng_uniform(r), u2 = rng_uniform(r);
    if (u1 < 1e-15) u1 = 1e-15;
    double mag = sqrt(-2.0 * log(u1));
    r->spare = mag * sin(2.0 * PI * u2);
    r->spare_valid = true;
    return mag * cos(2.0 * PI * u2);
}

typedef struct {
    double fd, inno_std, fi[FIR_NH], fq[FIR_NH];
    double complex hold;
    size_t pos, update;
    rng_t rng;
} doppler_t;

static double fir_sumsq(void) {
    double s = 0; for (int i=0;i<FIR_NH;i++) s += gaus_fir[i]*gaus_fir[i]; return s;
}
static double complex doppler_output(const doppler_t *d) {
    double i=0,q=0; for (int k=0;k<FIR_NH;k++) { i+=gaus_fir[k]*d->fi[k]; q+=gaus_fir[k]*d->fq[k]; }
    return i + I*q;
}
static void doppler_update(doppler_t *d) {
    memmove(d->fi+1,d->fi,(FIR_NH-1)*sizeof(double));
    memmove(d->fq+1,d->fq,(FIR_NH-1)*sizeof(double));
    d->fi[0]=rng_gauss(&d->rng)*d->inno_std;
    d->fq[0]=rng_gauss(&d->rng)*d->inno_std;
}
static void doppler_init(doppler_t *d,double fd,uint64_t seed) {
    memset(d,0,sizeof(*d)); d->fd=fd; rng_init(&d->rng,seed);
    d->update = fd>0 ? (size_t)llround(RATE/(fd*64.0)) : PERIOD;
    if (!d->update) d->update=1;
    if (fd>0) {
        d->inno_std=sqrt(0.5/fir_sumsq());
        for(int i=0;i<FIR_NH;i++) doppler_update(d);
        d->hold=doppler_output(d);
    } else d->hold=(rng_gauss(&d->rng)+I*rng_gauss(&d->rng))/sqrt(2.0);
}
static void doppler_advance(doppler_t *d,size_t n,double complex *out) {
    size_t k=0;
    while(k<n) {
        size_t take=n-k;
        if(d->fd>0) { size_t span=d->update-d->pos; if(take>span) take=span; }
        for(size_t j=0;j<take;j++) out[k+j]=d->hold;
        k+=take; d->pos=(d->pos+take)%d->update;
        if(d->fd>0 && d->pos==0) { doppler_update(d); d->hold=doppler_output(d); }
    }
}
static void doppler_skip(doppler_t *d,size_t n) {
    while(n) {
        size_t take=n;
        if(d->fd>0){size_t span=d->update-d->pos;if(take>span)take=span;}
        n-=take;d->pos=(d->pos+take)%d->update;
        if(d->fd>0&&d->pos==0){doppler_update(d);d->hold=doppler_output(d);}
    }
}

typedef struct {
    options_t opt;
    rng_t noise_rng, pn_rng;
    doppler_t tap0,tap1;
    bool fading;
    size_t delay, delay_pos;
    double complex delay_buf[192];
    double hilbert_h[HILBERT_N], hist[HILBERT_N-1];
    size_t hist_pos;
    double snr_lin,p_sig,noise_std,peak_ms,phase;
    psig_mode_t psig_mode;
    double psig_fix,*heap_lo,*heap_hi;
    size_t heap_lo_n,heap_hi_n,heap_cap,active_n,active_since;
    int ge_state;
    double *bp_h,*bp_hist;
    int bp_n,bp_pos;
    uint64_t sample_clock;
} channel_t;

static double ionos_wgn_to_snr3k(double label) {
    double requested=label+4.8, endpoint=49.7;
    return -10.0*log10(pow(10.0,-requested/10.0)+pow(10.0,-endpoint/10.0));
}
static void profile_params(profile_t p,double *dtau,double *fd) {
    *dtau=0;*fd=0;
    if(p==PROF_MPG){*dtau=.0005;*fd=.1;}
    else if(p==PROF_MPM){*dtau=.001;*fd=.5;}
    else if(p==PROF_MPP){*dtau=.002;*fd=1.;}
    else if(p==PROF_MPD){*dtau=.004;*fd=2.;}
}
static void hilbert_init(channel_t *c) {
    int m=(HILBERT_N-1)/2;
    for(int k=0;k<HILBERT_N;k++) {
        int n=k-m; double h=0;
        if(n%2) h=2.0/(PI*n);
        h*=0.5-0.5*cos(2.0*PI*k/(HILBERT_N-1));
        c->hilbert_h[k]=h;
    }
}
/* Streaming convolution aligned exactly like AnalyticFilter.process(). */
static void analytic_process(channel_t *c,const double *x,size_t n,double complex *z) {
    const int delay=(HILBERT_N-1)/2;
    for(size_t j=0;j<n;j++) {
        /* hist_pos is the next slot; before insertion it addresses x[j-128]. */
        double q=c->hilbert_h[HILBERT_N-1]*x[j];
        for(int back=1;back<HILBERT_N;back++) {
            int idx=(int)c->hist_pos-back;
            while(idx<0) idx+=HILBERT_N-1;
            q+=c->hilbert_h[HILBERT_N-1-back]*c->hist[idx];
        }
        int ii=(int)c->hist_pos-delay;
        while(ii<0) ii+=HILBERT_N-1;
        double iv=c->hist[ii];
        z[j]=iv+I*q;
        c->hist[c->hist_pos]=x[j];
        c->hist_pos=(c->hist_pos+1)%(HILBERT_N-1);
    }
}

static void max_push(double*h,size_t*n,double v){size_t i=(*n)++;while(i){size_t p=(i-1)/2;if(h[p]>=v)break;h[i]=h[p];i=p;}h[i]=v;}
static void min_push(double*h,size_t*n,double v){size_t i=(*n)++;while(i){size_t p=(i-1)/2;if(h[p]<=v)break;h[i]=h[p];i=p;}h[i]=v;}
static double max_pop(double*h,size_t*n){double root=h[0],v=h[--*n];size_t i=0;while(2*i+1<*n){size_t ch=2*i+1;if(ch+1<*n&&h[ch+1]>h[ch])ch++;if(h[ch]<=v)break;h[i]=h[ch];i=ch;}if(*n)h[i]=v;return root;}
static double min_pop(double*h,size_t*n){double root=h[0],v=h[--*n];size_t i=0;while(2*i+1<*n){size_t ch=2*i+1;if(ch+1<*n&&h[ch+1]<h[ch])ch++;if(h[ch]>=v)break;h[i]=h[ch];i=ch;}if(*n)h[i]=v;return root;}
static double median_active(channel_t*c){return c->heap_lo_n==c->heap_hi_n?.5*(c->heap_lo[0]+c->heap_hi[0]):c->heap_lo[0];}
static double noise_std(channel_t*c,double p){return sqrt(fmax(p*F_NYQUIST/(c->snr_lin*BW_NOISE),0.0));}
static double sinc_np(double x){return x==0.0?1.0:sin(PI*x)/(PI*x);}
static int bandpass_init(channel_t*c) {
    if(!(c->opt.bp_hi>c->opt.bp_lo && c->opt.bp_lo>0)) return 0;
    int nt=c->opt.bp_taps; if(!(nt&1)) nt++; c->bp_n=nt;
    c->bp_h=calloc((size_t)nt,sizeof(double));c->bp_hist=calloc((size_t)nt,sizeof(double));
    if(!c->bp_h||!c->bp_hist)return -1;
    int m=(nt-1)/2;double fc=.5*(c->opt.bp_lo+c->opt.bp_hi),gain=0;
    for(int k=0;k<nt;k++){
        int n=k-m;double wh=2*c->opt.bp_hi/RATE,wl=2*c->opt.bp_lo/RATE;
        double black=.42-.5*cos(2*PI*k/(nt-1))+.08*cos(4*PI*k/(nt-1));
        c->bp_h[k]=(wh*sinc_np(wh*n)-wl*sinc_np(wl*n))*black;
        gain+=c->bp_h[k]*cos(2*PI*fc/RATE*n);
    }
    if(fabs(gain)>1e-12)for(int k=0;k<nt;k++)c->bp_h[k]/=gain;
    return 0;
}
static double bandpass_one(channel_t*c,double x){
    c->bp_hist[c->bp_pos]=x;double y=0;
    for(int k=0;k<c->bp_n;k++){int idx=c->bp_pos-(c->bp_n-1-k);while(idx<0)idx+=c->bp_n;y+=c->bp_h[k]*c->bp_hist[idx];}
    c->bp_pos=(c->bp_pos+1)%c->bp_n;return y;
}
static int channel_init(channel_t*c,const options_t*o,uint32_t seed){
    memset(c,0,sizeof(*c));c->opt=*o;
    rng_init(&c->noise_rng,seed);
    uint64_t tap_seed=splitmix_next(&c->noise_rng); /* Python seed_np consumes one u64. */
    rng_init(&c->pn_rng,tap_seed^UINT64_C(0xD1B54A32D192ED03));
    c->snr_lin=pow(10.0,o->snr/10.0);c->p_sig=pow(fmax(o->sig_ref,1e-6),2);
    c->noise_std=noise_std(c,c->p_sig);
    const char*mode=getenv("MERCURY_SIM_PSIG_MODE");
    c->psig_mode=mode&&!strcasecmp(mode,"fix")?PSIG_FIX:mode&&!strcasecmp(mode,"peak")?PSIG_PEAK:PSIG_STEADY;
    const char*fix=getenv("MERCURY_SIM_PSIG_FIX");c->psig_fix=fix?strtod(fix,NULL):0;
    if(c->psig_mode==PSIG_FIX&&c->psig_fix>0){c->p_sig=c->psig_fix;c->noise_std=noise_std(c,c->p_sig);}
    if(c->psig_mode==PSIG_STEADY){c->heap_cap=16384;c->heap_lo=malloc(c->heap_cap*sizeof(double));c->heap_hi=malloc(c->heap_cap*sizeof(double));if(!c->heap_lo||!c->heap_hi)return -1;}
    double dtau,fd;profile_params(o->profile,&dtau,&fd);c->fading=dtau>0;
    if(c->fading){
        c->delay=(size_t)llround(dtau*RATE);if(c->delay<1)c->delay=1;
        hilbert_init(c);doppler_init(&c->tap0,fd,tap_seed^UINT64_C(0x9E3779B97F4A7C15));
        doppler_init(&c->tap1,fd,tap_seed^UINT64_C(0xBF58476D1CE4E5B9));
    }
    return bandpass_init(c);
}
static void channel_free(channel_t*c){free(c->heap_lo);free(c->heap_hi);free(c->bp_h);free(c->bp_hist);}

static int append_active(channel_t*c,double ms){
    if(c->heap_lo_n+c->heap_hi_n>=c->heap_cap){size_t cap=c->heap_cap*2;double*lo=realloc(c->heap_lo,cap*sizeof(double));if(!lo)return-1;c->heap_lo=lo;double*hi=realloc(c->heap_hi,cap*sizeof(double));if(!hi)return-1;c->heap_hi=hi;c->heap_cap=cap;}
    if(!c->heap_lo_n||ms<=c->heap_lo[0])max_push(c->heap_lo,&c->heap_lo_n,ms);else min_push(c->heap_hi,&c->heap_hi_n,ms);
    if(c->heap_lo_n>c->heap_hi_n+1){double v=max_pop(c->heap_lo,&c->heap_lo_n);min_push(c->heap_hi,&c->heap_hi_n,v);}
    else if(c->heap_hi_n>c->heap_lo_n){double v=min_pop(c->heap_hi,&c->heap_hi_n);max_push(c->heap_lo,&c->heap_lo_n,v);}
    c->active_n++;return 0;
}
static void channel_process(channel_t*c,const double*x,double*out,size_t n){
    double ms=0;for(size_t i=0;i<n;i++)ms+=x[i]*x[i];if(n)ms/=n;
    bool peak=ms>c->peak_ms;if(peak)c->peak_ms=ms;
    if(c->psig_mode==PSIG_STEADY&&ms>1e-7){
        if(!append_active(c,ms)&&++c->active_since>=8){c->active_since=0;c->p_sig=median_active(c);c->noise_std=noise_std(c,c->p_sig);}
    }else if(c->psig_mode==PSIG_PEAK&&peak){c->p_sig=c->peak_ms;c->noise_std=noise_std(c,c->p_sig);}

    double complex z[PERIOD],g0[PERIOD],g1[PERIOD];
    if(c->fading){analytic_process(c,x,n,z);doppler_advance(&c->tap0,n,g0);doppler_advance(&c->tap1,n,g1);}
    for(size_t i=0;i<n;i++){
        double complex y;
        if(c->fading){
            double complex zd=c->delay_buf[c->delay_pos];c->delay_buf[c->delay_pos]=z[i];c->delay_pos=(c->delay_pos+1)%c->delay;
            y=(g0[i]*z[i]+g1[i]*zd)/sqrt(2.0);
        }else y=x[i]+I*0.0;
        if(c->opt.cfo_hz!=0||c->opt.phase_noise_deg>0){
            c->phase+=2*PI*c->opt.cfo_hz/RATE;double ph=c->phase;
            if(c->opt.phase_noise_deg>0)ph+=rng_gauss(&c->pn_rng)*(c->opt.phase_noise_deg*PI/180.0);
            y*=cexp(I*ph);if(c->phase>=2*PI||c->phase<=-2*PI)c->phase=fmod(c->phase,2*PI);
        }
        double v=creal(y);if(c->bp_n)v=bandpass_one(c,v);
        if(c->noise_std>0)v+=c->noise_std*rng_gauss(&c->noise_rng);
        if(c->opt.burst&&c->opt.loss>0){
            if(!c->ge_state){if(rng_uniform(&c->noise_rng)<c->opt.loss*.05)c->ge_state=1;}
            else{v=0;if(rng_uniform(&c->noise_rng)<.03)c->ge_state=0;}
        }else if(c->opt.loss>0&&rng_uniform(&c->noise_rng)<c->opt.loss)v=0;
        out[i]=v;
    }
    c->sample_clock+=n;
}

typedef struct {
    uint64_t frames,sig_frames,underruns,hard_clips,s32_saturations;
    uint64_t clip_denominator;
    double sig_sumsq,pre_scale_peak,reference_power,noise_variance;
    pthread_mutex_t lock;
} pump_stats_t;
typedef struct {
    char name[8],cap_dev[256],play_dev[256];
    channel_t channel;const options_t*opt;pump_stats_t stats;
    snd_pcm_t *cap_shared,*play_shared;
    pthread_mutex_t pcm_lock;
} pump_t;

static int pcm_open_exact(snd_pcm_t**pcm,const char*dev,snd_pcm_stream_t stream,int periods){
    int e=snd_pcm_open(pcm,dev,stream,0);if(e<0)return e;
    snd_pcm_hw_params_t*p;snd_pcm_hw_params_alloca(&p);unsigned rate=RATE,per=(unsigned)periods;
    snd_pcm_uframes_t psz=PERIOD;
    if((e=snd_pcm_hw_params_any(*pcm,p))<0||
       (e=snd_pcm_hw_params_set_access(*pcm,p,SND_PCM_ACCESS_RW_INTERLEAVED))<0||
       (e=snd_pcm_hw_params_set_format(*pcm,p,SND_PCM_FORMAT_S32_LE))<0||
       (e=snd_pcm_hw_params_set_channels(*pcm,p,2))<0||
       (e=snd_pcm_hw_params_set_rate(*pcm,p,rate,0))<0||
       (e=snd_pcm_hw_params_set_period_size(*pcm,p,psz,0))<0||
       (e=snd_pcm_hw_params_set_periods(*pcm,p,per,0))<0||
       (e=snd_pcm_hw_params(*pcm,p))<0){snd_pcm_close(*pcm);*pcm=NULL;return e;}
    return snd_pcm_prepare(*pcm);
}
static int write_frames(snd_pcm_t*p,const int32_t*buf,size_t frames,int*xruns){
    size_t off=0;while(off<frames&&!stop_requested){snd_pcm_sframes_t n=snd_pcm_writei(p,buf+2*off,frames-off);if(n<0){(*xruns)++;int e=snd_pcm_recover(p,(int)n,1);if(e<0)return e;continue;}off+=(size_t)n;}return 0;
}
static void stats_add(pump_t*p,size_t n,bool signal,double ss,int underrun,
                      double peak,uint64_t hard,uint64_t saturation,size_t clip_n){
    pthread_mutex_lock(&p->stats.lock);p->stats.frames+=n;if(signal){p->stats.sig_frames+=n;p->stats.sig_sumsq+=ss;}p->stats.underruns+=underrun;
    if(peak>p->stats.pre_scale_peak)p->stats.pre_scale_peak=peak;
    p->stats.hard_clips+=hard;p->stats.s32_saturations+=saturation;
    p->stats.clip_denominator+=clip_n;p->stats.reference_power=p->channel.p_sig;
    p->stats.noise_variance=p->channel.noise_std*p->channel.noise_std;
    pthread_mutex_unlock(&p->stats.lock);
}
static void *pump_main(void*arg){
    pump_t*p=arg;snd_pcm_t *cap=NULL,*play=NULL;
    int e=pcm_open_exact(&cap,p->cap_dev,SND_PCM_STREAM_CAPTURE,p->opt->cap_periods);
    if(e<0){fprintf(stderr,"[bridge_s32_c] %s capture %s: %s\n",p->name,p->cap_dev,snd_strerror(e));stop_requested=1;return NULL;}
    pthread_mutex_lock(&p->pcm_lock);p->cap_shared=cap;pthread_mutex_unlock(&p->pcm_lock);
    e=pcm_open_exact(&play,p->play_dev,SND_PCM_STREAM_PLAYBACK,p->opt->play_periods);
    if(e<0){fprintf(stderr,"[bridge_s32_c] %s playback %s: %s\n",p->name,p->play_dev,snd_strerror(e));pthread_mutex_lock(&p->pcm_lock);p->cap_shared=NULL;snd_pcm_close(cap);pthread_mutex_unlock(&p->pcm_lock);stop_requested=1;return NULL;}
    pthread_mutex_lock(&p->pcm_lock);p->play_shared=play;pthread_mutex_unlock(&p->pcm_lock);
    int32_t in[PERIOD*2],ob[PERIOD*2]={0};double x[PERIOD],out[PERIOD];
    for(int i=0;i<p->opt->prime_periods;i++){int xruns=0;if(write_frames(play,ob,PERIOD,&xruns)<0)xruns++;if(xruns)stats_add(p,0,false,0,xruns,0,0,0,0);}
    while(!stop_requested){
        snd_pcm_sframes_t nr=snd_pcm_readi(cap,in,PERIOD);
        if(nr<0){stats_add(p,0,false,0,1,0,0,0,0);if(stop_requested||snd_pcm_recover(cap,(int)nr,1)<0)break;continue;}
        size_t n=(size_t)nr;bool signal=false;double ss=0;
        for(size_t i=0;i<n;i++){if(in[2*i])signal=true;x[i]=(double)in[2*i]/INT_MAX_D;ss+=x[i]*x[i];}
        double peak=0;uint64_t hard=0,saturation=0;
        if(p->opt->passthrough){for(size_t i=0;i<n;i++)ob[2*i]=ob[2*i+1]=in[2*i];}
        else{
            channel_process(&p->channel,x,out,n);
            for(size_t i=0;i<n;i++){double a=fabs(out[i]);if(a>peak)peak=a;if(a>1.0){hard++;saturation++;}double v=out[i]*INT_MAX_D;if(v>INT_MAX_D)v=INT_MAX_D;if(v< -INT_MAX_D)v=-INT_MAX_D;int32_t q=(int32_t)v;ob[2*i]=ob[2*i+1]=q;}
        }
        int xruns=0;int wr=write_frames(play,ob,n,&xruns);if(wr<0)xruns++;
        stats_add(p,n,signal,ss,xruns,peak,hard,saturation,p->opt->passthrough?0:n);if(wr<0&&!stop_requested)snd_pcm_prepare(play);
    }
    pthread_mutex_lock(&p->pcm_lock);p->cap_shared=p->play_shared=NULL;
    snd_pcm_close(cap);snd_pcm_close(play);pthread_mutex_unlock(&p->pcm_lock);
    return NULL;
}

static void json_string(FILE*f,const char*s){fputc('"',f);for(;*s;s++){unsigned char c=*s;if(c=='"'||c=='\\'){fputc('\\',f);fputc(c,f);}else if(c<32)fprintf(f,"\\u%04x",c);else fputc(c,f);}fputc('"',f);}
static void stat_snapshot(pump_t*p,uint64_t*fr,uint64_t*sf,double*ss,uint64_t*ur,
                          double*peak,uint64_t*hard,uint64_t*sat,uint64_t*den,
                          double*reference_power,double*noise_variance){
    pthread_mutex_lock(&p->stats.lock);*fr=p->stats.frames;*sf=p->stats.sig_frames;*ss=p->stats.sig_sumsq;*ur=p->stats.underruns;*peak=p->stats.pre_scale_peak;*hard=p->stats.hard_clips;*sat=p->stats.s32_saturations;*den=p->stats.clip_denominator;*reference_power=p->stats.reference_power;*noise_variance=p->stats.noise_variance;pthread_mutex_unlock(&p->stats.lock);
}
static int flush_stats(const options_t*o,pump_t*fwd,pump_t*rev){
    if(!o->statsfile[0]) return 0;
    char tmp[1200];snprintf(tmp,sizeof(tmp),"%s.tmp.%ld",o->statsfile,(long)getpid());FILE*f=fopen(tmp,"w");if(!f)return-1;
    uint64_t ff,fs,fu,fh,fx,fd,rf,rs,ru,rh,rx,rd;double fss,rss,fp,rp,fref,fnoise,rref,rnoise;
    stat_snapshot(fwd,&ff,&fs,&fss,&fu,&fp,&fh,&fx,&fd,&fref,&fnoise);stat_snapshot(rev,&rf,&rs,&rss,&ru,&rp,&rh,&rx,&rd,&rref,&rnoise);(void)rref;(void)rnoise;
    fprintf(f,"{\n  \"fwd\": {\"frames\": %"PRIu64", \"sig_frames\": %"PRIu64", \"sig_sumsq\": %.17g, \"underruns\": %"PRIu64"},\n",ff,fs,fss,fu);
    fprintf(f,"  \"rev\": {\"frames\": %"PRIu64", \"sig_frames\": %"PRIu64", \"sig_sumsq\": %.17g, \"underruns\": %"PRIu64"},\n",rf,rs,rss,ru);
    fprintf(f,"  \"channel_attestation\": {\"cell\": ");json_string(f,o->cell);fprintf(f,", \"profile\": ");
    char up[16];size_t i;for(i=0;i<sizeof(up)-1&&o->profile_name[i];i++)up[i]=(char)toupper((unsigned char)o->profile_name[i]);up[i]=0;json_string(f,up);
    fprintf(f,", \"commanded_snr\": %.17g, \"realized_snr3k\": %.17g, \"realized_snr_offset_db\": %.17g, \"seed\": %d, \"passthrough\": %s, \"realized_p_sig\": %.17g},\n",o->commanded_snr,o->snr,o->snr_offset,o->seed,o->passthrough?"true":"false",fs?fss/fs:0.0);
    fprintf(f,"  \"axis_attestation\": {\"axis_version\": ");json_string(f,o->axis);fprintf(f,", \"input_coordinate\": ");json_string(f,o->input_coordinate);
    fprintf(f,", \"reference_mode\": \"steady-active-chunk-median\", \"reference_id\": \"traffic-steady-v1\", \"reference_power\": %.17g, \"reference_n_samples\": 0",fref);
    fprintf(f,", \"configured_bandwidth_hz\": %.17g, \"snr3k_db\": %.17g, \"cn_config_db\": %.17g, \"noise_variance\": %.17g",o->configured_bandwidth_hz,o->snr,o->cn_config_db,fnoise);
    fprintf(f,", \"seed\": %d, \"composite_scale\": 1.0, \"pre_scale_peak\": %.17g, \"hard_clip_count\": %"PRIu64", \"s32_saturation_count\": %"PRIu64", \"clip_event_denominator\": %"PRIu64", \"s32_mode\": ",o->seed,fmax(fp,rp),fh+rh,fx+rx,fd+rd);json_string(f,!strcmp(o->s32_mode,"auto")?"s32-hardclip":o->s32_mode);
    fprintf(f,", \"binary_sha256\": ");json_string(f,o->binary_sha256);fprintf(f,", \"recipe_sha256\": ");json_string(f,o->recipe_sha256);fprintf(f,"}\n}\n");
    fflush(f);fsync(fileno(f));if(fclose(f)||rename(tmp,o->statsfile)){unlink(tmp);return-1;}return 0;
}

static void signal_handler(int sig){(void)sig;stop_requested=1;}
static void defaults(options_t*o){
    memset(o,0,sizeof(*o));strcpy(o->fwd_cap,"hw:Loopback,1,0");strcpy(o->fwd_play,"hw:Loopback,0,1");strcpy(o->rev_cap,"hw:Loopback,1,2");strcpy(o->rev_play,"hw:Loopback,0,3");strcpy(o->profile_name,"wgn");o->profile=PROF_WGN;o->snr=30;o->sig_ref=.15;o->seed=1;o->cap_periods=4;o->play_periods=5;o->prime_periods=2;o->bp_taps=511;
    strcpy(o->axis,"v1-steady-snr3k");strcpy(o->input_coordinate,"default_snr3k_db");strcpy(o->s32_mode,"auto");memset(o->binary_sha256,'0',64);o->binary_sha256[64]=0;memset(o->recipe_sha256,'0',64);o->recipe_sha256[64]=0;o->configured_bandwidth_hz=2343.75;
    const char *bp=getenv("IRIS_AUDIO_BANDPASS");
    if(bp&&!strcmp(bp,"narrow")){o->bp_lo=300;o->bp_hi=2900;}
    else if(bp&&!strcmp(bp,"wide")){o->bp_lo=300;o->bp_hi=6300;}
}
static void usage(FILE*f){fprintf(f,"usage: realaudio_bridge_s32_c [Python-compatible bridge options]\n       test modes: --vector-in F --vector-out F [--format-only|--passthrough]\n                   --double-in F --double-out F | --tap-out F --tap-count N\n");}
static int val(int argc,char**argv,int*i,const char**v){if(*i+1>=argc)return-1;*v=argv[++*i];return 0;}
static int parse_profile(options_t*o,const char*s){
    snprintf(o->profile_name,sizeof(o->profile_name),"%s",s);for(char*p=o->profile_name;*p;p++)*p=(char)tolower((unsigned char)*p);
    if(!strcmp(o->profile_name,"wgn"))o->profile=PROF_WGN;else if(!strcmp(o->profile_name,"flat"))o->profile=PROF_FLAT;else if(!strcmp(o->profile_name,"mpg"))o->profile=PROF_MPG;else if(!strcmp(o->profile_name,"mpm"))o->profile=PROF_MPM;else if(!strcmp(o->profile_name,"mpp"))o->profile=PROF_MPP;else if(!strcmp(o->profile_name,"mpd"))o->profile=PROF_MPD;else return-1;return 0;
}
static int parse_args(int argc,char**argv,options_t*o){
    defaults(o);for(int i=1;i<argc;i++){const char*a=argv[i],*v=NULL;
        if(!strcmp(a,"--help")||!strcmp(a,"-h")){usage(stdout);exit(0);}else if(!strcmp(a,"--passthrough"))o->passthrough=true;else if(!strcmp(a,"--burst"))o->burst=true;else if(!strcmp(a,"--dry-run"))o->dry_run=true;else if(!strcmp(a,"--self-test"))o->self_test=true;else if(!strcmp(a,"--format-only"))o->format_only=true;
        else if(val(argc,argv,&i,&v)<0)return-1;
        else if(!strcmp(a,"--fwd-cap"))snprintf(o->fwd_cap,sizeof(o->fwd_cap),"%s",v);else if(!strcmp(a,"--fwd-play"))snprintf(o->fwd_play,sizeof(o->fwd_play),"%s",v);else if(!strcmp(a,"--rev-cap"))snprintf(o->rev_cap,sizeof(o->rev_cap),"%s",v);else if(!strcmp(a,"--rev-play"))snprintf(o->rev_play,sizeof(o->rev_play),"%s",v);
        else if(!strcmp(a,"--profile")){if(parse_profile(o,v))return-1;}else if(!strcmp(a,"--snr")){o->snr=strtod(v,NULL);strcpy(o->input_coordinate,"snr");}else if(!strcmp(a,"--snr3k")){o->snr=strtod(v,NULL);strcpy(o->input_coordinate,"snr3k");}else if(!strcmp(a,"--snr3k-db")){o->snr=strtod(v,NULL);strcpy(o->input_coordinate,"snr3k_db");}else if(!strcmp(a,"--cn-config-db")){o->cn_config_db=strtod(v,NULL);strcpy(o->input_coordinate,"cn_config_db");}else if(!strcmp(a,"--cell")){snprintf(o->cell,sizeof(o->cell),"%s",v);strcpy(o->input_coordinate,"cell");}else if(!strcmp(a,"--axis"))snprintf(o->axis,sizeof(o->axis),"%s",v);else if(!strcmp(a,"--configured-bandwidth-hz"))o->configured_bandwidth_hz=strtod(v,NULL);else if(!strcmp(a,"--binary-sha256"))snprintf(o->binary_sha256,sizeof(o->binary_sha256),"%s",v);else if(!strcmp(a,"--recipe-sha256"))snprintf(o->recipe_sha256,sizeof(o->recipe_sha256),"%s",v);else if(!strcmp(a,"--s32-mode"))snprintf(o->s32_mode,sizeof(o->s32_mode),"%s",v);else if(!strcmp(a,"--cfo-hz"))o->cfo_hz=strtod(v,NULL);else if(!strcmp(a,"--phase-noise-deg"))o->phase_noise_deg=strtod(v,NULL);else if(!strcmp(a,"--fade-depth-db"))o->fade_depth_db=strtod(v,NULL);else if(!strcmp(a,"--loss"))o->loss=strtod(v,NULL);else if(!strcmp(a,"--sig-ref"))o->sig_ref=strtod(v,NULL);else if(!strcmp(a,"--seed"))o->seed=(int)strtol(v,NULL,10);else if(!strcmp(a,"--cap-periods"))o->cap_periods=(int)strtol(v,NULL,10);else if(!strcmp(a,"--play-periods"))o->play_periods=(int)strtol(v,NULL,10);else if(!strcmp(a,"--prime-periods"))o->prime_periods=(int)strtol(v,NULL,10);else if(!strcmp(a,"--statsfile"))snprintf(o->statsfile,sizeof(o->statsfile),"%s",v);
        else if(!strcmp(a,"--audio-bandpass")){if(!strcmp(v,"narrow")){o->bp_lo=300;o->bp_hi=2900;}else if(!strcmp(v,"wide")){o->bp_lo=300;o->bp_hi=6300;}else if(strcmp(v,"off"))return-1;}else if(!strcmp(a,"--bandpass-lo-hz"))o->bp_lo=strtod(v,NULL);else if(!strcmp(a,"--bandpass-hi-hz"))o->bp_hi=strtod(v,NULL);else if(!strcmp(a,"--bandpass-taps"))o->bp_taps=(int)strtol(v,NULL,10);
        else if(!strcmp(a,"--vector-in"))snprintf(o->vector_in,sizeof(o->vector_in),"%s",v);else if(!strcmp(a,"--vector-out"))snprintf(o->vector_out,sizeof(o->vector_out),"%s",v);else if(!strcmp(a,"--double-in"))snprintf(o->double_in,sizeof(o->double_in),"%s",v);else if(!strcmp(a,"--double-out"))snprintf(o->double_out,sizeof(o->double_out),"%s",v);else if(!strcmp(a,"--tap-out"))snprintf(o->tap_out,sizeof(o->tap_out),"%s",v);else if(!strcmp(a,"--tap-count"))o->tap_count=(size_t)strtoull(v,NULL,10);else if(!strcmp(a,"--tap-stride"))o->tap_stride=(size_t)strtoull(v,NULL,10);else return-1;
    }
    if(strcmp(o->axis,"v1-steady-snr3k")||!strcmp(o->s32_mode,"prescaled")){
        fprintf(stderr,"native bridge currently requires --axis v1-steady-snr3k and a hard-clip S32 mode\n");return-1;
    }
    if(o->configured_bandwidth_hz<=0)return-1;
    if(!strcmp(o->input_coordinate,"cn_config_db"))o->snr=o->cn_config_db-10.0*log10(3000.0/o->configured_bandwidth_hz);
    o->commanded_snr=o->snr;o->snr_offset=0;
    if(o->cell[0]){char*colon=strchr(o->cell,':');if(!colon||strncasecmp(o->cell,"WGN:",4))return-1;o->commanded_snr=strtod(colon+1,NULL);o->snr=ionos_wgn_to_snr3k(o->commanded_snr);o->snr_offset=o->snr-o->commanded_snr;}
    else {
        char up[16]; size_t k;
        for(k=0;k<sizeof(up)-1&&o->profile_name[k];k++) up[k]=(char)toupper((unsigned char)o->profile_name[k]);
        up[k]=0; snprintf(o->cell,sizeof(o->cell),"%s:%g",up,o->snr);
    }
    o->cn_config_db=o->snr+10.0*log10(3000.0/o->configured_bandwidth_hz);
    return 0;
}

static int vector_mode(const options_t*o){
    FILE*fi=fopen(o->vector_in,"rb"),*fo=fopen(o->vector_out,"wb");if(!fi||!fo){perror("vector file");return 2;}channel_t c;if(channel_init(&c,o,(uint32_t)(o->seed*UINT32_C(2654435761))))return 2;
    int32_t ib[PERIOD*2],ob[PERIOD*2];double x[PERIOD],y[PERIOD];size_t words;
    while((words=fread(ib,sizeof(int32_t),PERIOD*2,fi))){size_t n=words/2;for(size_t i=0;i<n;i++)x[i]=(double)ib[2*i]/INT_MAX_D;
        if(o->passthrough){for(size_t i=0;i<n;i++)ob[2*i]=ob[2*i+1]=ib[2*i];}
        else {if(o->format_only)memcpy(y,x,n*sizeof(double));else channel_process(&c,x,y,n);for(size_t i=0;i<n;i++){double v=y[i]*INT_MAX_D;if(v>INT_MAX_D)v=INT_MAX_D;if(v< -INT_MAX_D)v=-INT_MAX_D;ob[2*i]=ob[2*i+1]=(int32_t)v;}}
        if(fwrite(ob,sizeof(int32_t),n*2,fo)!=n*2){perror("write");return 2;}if(words%2)break;
    }channel_free(&c);fclose(fi);fclose(fo);return 0;
}
static int double_mode(const options_t*o){
    FILE*fi=fopen(o->double_in,"rb"),*fo=fopen(o->double_out,"wb");if(!fi||!fo){perror("double file");return 2;}channel_t c;if(channel_init(&c,o,(uint32_t)(o->seed*UINT32_C(2654435761))))return 2;double x[PERIOD],y[PERIOD];size_t n;while((n=fread(x,sizeof(double),PERIOD,fi))){channel_process(&c,x,y,n);if(fwrite(y,sizeof(double),n,fo)!=n)return 2;}channel_free(&c);fclose(fi);fclose(fo);return 0;
}
static int tap_mode(const options_t*o){
    double dt,fd;profile_params(o->profile,&dt,&fd);if(fd<=0){fprintf(stderr,"tap mode needs fading profile\n");return 2;}FILE*f=fopen(o->tap_out,"wb");if(!f){perror("tap-out");return 2;}doppler_t d;doppler_init(&d,fd,(uint64_t)o->seed);size_t stride=o->tap_stride?o->tap_stride:d.update;for(size_t i=0;i<o->tap_count;i++){double pair[2]={creal(d.hold),cimag(d.hold)};if(fwrite(pair,sizeof(double),2,f)!=2){fclose(f);return 2;}doppler_skip(&d,stride);}fclose(f);return 0;
}
static void dry_stats(pump_t*p,const options_t*o,uint32_t seed){
    channel_init(&p->channel,o,seed);double x[PERIOD],y[PERIOD],ss=0;for(int i=0;i<PERIOD;i++)x[i]=o->sig_ref*sqrt(2.0)*sin(2*PI*1500*i/RATE);for(int k=0;k<8;k++){if(!o->passthrough)channel_process(&p->channel,x,y,PERIOD);for(int i=0;i<PERIOD;i++)ss+=x[i]*x[i];p->stats.frames+=PERIOD;p->stats.sig_frames+=PERIOD;}p->stats.sig_sumsq=ss;p->stats.reference_power=p->channel.p_sig;p->stats.noise_variance=p->channel.noise_std*p->channel.noise_std;
}
int main(int argc,char**argv){
    options_t o;if(parse_args(argc,argv,&o)){usage(stderr);return 2;}
    if(o.vector_in[0]||o.vector_out[0])return(o.vector_in[0]&&o.vector_out[0])?vector_mode(&o):2;
    if(o.double_in[0]||o.double_out[0])return(o.double_in[0]&&o.double_out[0])?double_mode(&o):2;
    if(o.tap_out[0])return tap_mode(&o);
    pump_t fwd={.opt=&o},rev={.opt=&o};strcpy(fwd.name,"fwd");strcpy(rev.name,"rev");strcpy(fwd.cap_dev,o.fwd_cap);strcpy(fwd.play_dev,o.fwd_play);strcpy(rev.cap_dev,o.rev_cap);strcpy(rev.play_dev,o.rev_play);pthread_mutex_init(&fwd.stats.lock,NULL);pthread_mutex_init(&rev.stats.lock,NULL);pthread_mutex_init(&fwd.pcm_lock,NULL);pthread_mutex_init(&rev.pcm_lock,NULL);
    if(o.dry_run||o.self_test){dry_stats(&fwd,&o,(uint32_t)(o.seed*UINT32_C(2654435761)));dry_stats(&rev,&o,(uint32_t)(o.seed*UINT32_C(40503)+7));if(!o.statsfile[0])snprintf(o.statsfile,sizeof(o.statsfile),"/tmp/bridge_c_dry_%ld.json",(long)getpid());int e=flush_stats(&o,&fwd,&rev);fprintf(stderr,"[bridge_s32_c] %s wrote %s\n",e?"DRY-RUN FAIL":"DRY-RUN PASS",o.statsfile);channel_free(&fwd.channel);channel_free(&rev.channel);return e?1:0;}
    if(channel_init(&fwd.channel,&o,(uint32_t)(o.seed*UINT32_C(2654435761)))||channel_init(&rev.channel,&o,(uint32_t)(o.seed*UINT32_C(40503)+7))){fprintf(stderr,"channel init failed\n");return 2;}
    fprintf(stderr,"[bridge_s32_c] %s SNR3k=%.3f profile=%s seed=%d rings cap=%d play=%d prime=%d cables fwd[%s->%s] rev[%s->%s]\n",o.passthrough?"PASSTHROUGH":"CHANNEL",o.snr,o.profile_name,o.seed,o.cap_periods,o.play_periods,o.prime_periods,o.fwd_cap,o.fwd_play,o.rev_cap,o.rev_play);
    struct sigaction sa={0};sa.sa_handler=signal_handler;sigaction(SIGINT,&sa,NULL);sigaction(SIGTERM,&sa,NULL);
    pthread_t tf,tr;if(pthread_create(&tf,NULL,pump_main,&fwd)||pthread_create(&tr,NULL,pump_main,&rev)){fprintf(stderr,"pthread_create failed\n");return 2;}
    while(!stop_requested){struct timespec ts={.tv_sec=0,.tv_nsec=500000000};nanosleep(&ts,NULL);flush_stats(&o,&fwd,&rev);}
    pthread_mutex_lock(&fwd.pcm_lock);if(fwd.cap_shared)snd_pcm_drop(fwd.cap_shared);if(fwd.play_shared)snd_pcm_drop(fwd.play_shared);pthread_mutex_unlock(&fwd.pcm_lock);
    pthread_mutex_lock(&rev.pcm_lock);if(rev.cap_shared)snd_pcm_drop(rev.cap_shared);if(rev.play_shared)snd_pcm_drop(rev.play_shared);pthread_mutex_unlock(&rev.pcm_lock);
    pthread_join(tf,NULL);pthread_join(tr,NULL);flush_stats(&o,&fwd,&rev);channel_free(&fwd.channel);channel_free(&rev.channel);return 0;
}
