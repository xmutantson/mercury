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
typedef enum { PSIG_STEADY, PSIG_FIX, PSIG_PEAK, PSIG_GEOMETRY, PSIG_BENCH } psig_mode_t;
/* Waveform kinds recognised in the per-transmission record (see geo_kind()). */
enum { KIND_OFDM_WB, KIND_OFDM_NB, KIND_OFDM_PREAMBLE, KIND_MFSK_CTL, KIND_MFSK_DATA, KIND_SHORT, KIND_N };
static const char*const kind_name[KIND_N]={"ofdm-wb","ofdm-nb","ofdm-preamble","mfsk-ctl","mfsk-data","short"};
/* Mercury OFDM WB data power on this bridge (mean square of the S32 input,
 * full scale = 1), the reference the bench-mirror noise level is set from.
 * Measured on this bridge: airtime-weighted forward OFDM WB data power of
 * pinned configs 0/7/8/13/14/15/16 at SNR3k 30 was 0.02030 / 0.02038 /
 * 0.02038 / 0.02048 / 0.02044 / 0.02053 / 0.01864 (config 16, thinned
 * pilots, sits 0.39 dB lower); the reference is their median, 0.0204
 * (-16.90 dBFS rms). */
#define BENCH_OFDM_REF_DEFAULT 0.0204
#define BENCH_OFDM_REF_DERIVATION "median forward OFDM WB data power of pinned cfgs 0/7/8/13/14/15/16 at snr3k 30 (0.02030/0.02038/0.02038/0.02048/0.02044/0.02053/0.01864), bridge calibration 2026-09-22"
#define GEO_MAX_CLASSES 16
#define GEO_SEG_MAX 16
/* A transmission ends after this many consecutive samples below GEO_ACTIVE. */
#define GEO_QUIET_RUN 256
#define GEO_ACTIVE 1e-6
#define GEO_SHIFT_CHUNKS 3
#define GEO_SHIFT_MIN_SAMPLES 512
/* Periodogram frame for the waveform-kind record: 93.75 Hz bins, so the NB
 * OFDM band (468.75 Hz) spans 5 bins and the WB band 25. */
#define SPEC_N 512

typedef struct {
    char fwd_cap[256], fwd_play[256], rev_cap[256], rev_play[256];
    char profile_name[16], cell[64], statsfile[1024];
    char axis[64], input_coordinate[32], binary_sha256[80], recipe_sha256[80];
    char bridge_sha256[80], harness_lineage[32], harness_sha256[80];
    char s32_mode[32];
    profile_t profile;
    double snr, commanded_snr, snr_offset, cfo_hz, phase_noise_deg;
    double fade_depth_db, loss, sig_ref, bp_lo, bp_hi;
    double configured_bandwidth_hz, cn_config_db;
    int seed, cap_periods, play_periods, prime_periods, bp_taps;
    int erase_a2b_burst, erase_b2a_burst;
    /* Receiver-passband noise model (see noise_lpf_init / composite_scale). */
    double noise_lpf_hz, headroom_rms;
    int noise_lpf_taps;
    bool noise_lpf_set, headroom_set;
    /* Noise reference (see "Noise reference modes" below). */
    char reference_mode[24], reference_mode_source[16], burst_log[1100], clip_log[1100];
    double psig_fix, floor_ref, class_tol_db, shift_db;
    double ofdm_ref, kind_kurt_thr, kind_bw_thr_hz;
    char ofdm_ref_source[16];
    bool psig_fix_set;
    bool passthrough, burst, dry_run, self_test, format_only;
    char vector_in[1024], vector_out[1024], double_in[1024], double_out[1024];
    char tap_out[1024];
    char snr_schedule[1024];
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
    double onset_pow;      /* running geometric mean of onset estimates */
    double ref_pow;        /* airtime-weighted mean power of completed transmissions */
    double ss,n;           /* accumulated sum of squares / samples */
    uint64_t bursts;
    double snr_sum,snr_min,snr_max;  /* realized SNR3k over completed transmissions */
} geo_class_t;
/* Per-kind totals of the per-transmission record (all accounting modes). */
typedef struct {
    uint64_t tx;
    double ss,n;                     /* signal sum of squares / samples (airtime) */
    double nv;                       /* applied noise variance, summed per sample */
    double snr_sum,snr_min,snr_max;  /* realized SNR3k per transmission */
} geo_kind_t;

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
    /* Noise low-pass: unit-DC-gain FIR applied to the unit Gaussian draw, so the
     * noise PSD inside the passband is exactly the white-noise PSD that the
     * SNR3k axis defines; only the out-of-passband noise is removed. */
    double *nf_h,*nf_hist,nf_gain2;
    int nf_n,nf_pos;
    /* Fixed per-session gain applied to (signal+noise) before S32 quantization. */
    double composite_scale,scale_min;
    uint64_t sample_clock;
    double sched_last_snr;
    bool sched_valid,sched_offline;
    char sched_tag[8];
    /* Geometry mode: one reference per transmission class, see geo_*(). */
    char name[8];
    geo_class_t cls[GEO_MAX_CLASSES];
    int ncls,cur_cls,rec_cls,shift_run;
    bool in_burst,cur_first,rec_first;
    size_t quiet_run,pend_quiet;
    double b_ss_d,b_n_d;                     /* decision-side running power of the current transmission */
    double b_ss,b_nv,b_nm,pend_nv,pend_nm;   /* record-side accumulators of the current transmission */
    double b_onset,cur_ref,floor_ref,noise_rms_max;
    uint64_t b_n,b_start,bursts_done,class_switches;
    double worst_snr_err_db;
    /* Waveform features of the current transmission (clean input): active
     * samples, sum x^4, and the averaged periodogram of the active samples. */
    double b_na,b_x4;
    double spec_buf[SPEC_N],spec_acc[SPEC_N/2+1];
    int spec_fill;uint64_t spec_frames;
    geo_kind_t kind[KIND_N];
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

/* Optional time-indexed SNR schedule.  Rows are "t_offset_s,snr3k_db" pairs on
 * the same axis as --snr; the applied SNR is a linear interpolation of the rows
 * against stream time, clamped to the first/last row outside the range.  The
 * schedule is process-wide so the forward and reverse channels fade together
 * off one shared time origin.  When no schedule is loaded none of this code
 * runs: the per-sample path and its RNG draws are untouched, so output is
 * byte-identical to the static --snr path. */
typedef struct { double t, snr; } sched_point_t;
typedef struct { double wall, t, applied, noise_std, realized; char chan[8]; } sched_event_t;
typedef struct {
    bool loaded;
    sched_point_t *pts; size_t n;                 /* parsed intent, sorted by t */
    sched_event_t *events; size_t nevents, cap_events;
    pthread_mutex_t lock;
    bool origin_set;
    struct timespec origin_mono;                  /* CLOCK_MONOTONIC at stream start */
    double origin_wall;                           /* CLOCK_REALTIME epoch at stream start */
    char time_base[16];
    bool logged; double log_snr, log_t;           /* material-change gate state */
    bool applied_any; double last_t, last_snr, last_noise_std, last_realized;
} snr_schedule_t;
static snr_schedule_t g_schedule = { .lock = PTHREAD_MUTEX_INITIALIZER };

static int sched_cmp(const void *a, const void *b) {
    double ta = ((const sched_point_t *)a)->t, tb = ((const sched_point_t *)b)->t;
    return ta < tb ? -1 : ta > tb ? 1 : 0;
}
static int schedule_load(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) { fprintf(stderr, "snr-schedule: cannot open %s: %s\n", path, strerror(errno)); return -1; }
    char line[512]; size_t cap = 0, n = 0; sched_point_t *pts = NULL; long lineno = 0;
    while (fgets(line, sizeof(line), f)) {
        lineno++;
        char *p = line; while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
        if (*p == '\0' || *p == '#') continue;
        char *end = NULL; double t = strtod(p, &end);
        char *q = end; while (*q == ' ' || *q == '\t') q++; if (*q == ',') q++;
        char *end2 = NULL; double s = strtod(q, &end2);
        char *r = end2; while (*r == ' ' || *r == '\t' || *r == '\r' || *r == '\n') r++;
        if (end == p || end2 == q || (*r != '\0' && *r != '#')) {
            fprintf(stderr, "snr-schedule: %s:%ld: malformed row (expected 't_offset_s,snr_db')\n", path, lineno);
            free(pts); fclose(f); return -1;
        }
        if (n == cap) {
            size_t nc = cap ? cap * 2 : 8; sched_point_t *np = realloc(pts, nc * sizeof(*np));
            if (!np) { free(pts); fclose(f); fprintf(stderr, "snr-schedule: out of memory\n"); return -1; }
            pts = np; cap = nc;
        }
        pts[n].t = t; pts[n].snr = s; n++;
    }
    fclose(f);
    if (n == 0) { free(pts); fprintf(stderr, "snr-schedule: %s: no data rows\n", path); return -1; }
    qsort(pts, n, sizeof(*pts), sched_cmp);
    g_schedule.pts = pts; g_schedule.n = n; g_schedule.loaded = true;
    return 0;
}
static double schedule_interp(double t) {
    const sched_point_t *p = g_schedule.pts; size_t n = g_schedule.n;
    if (t <= p[0].t) return p[0].snr;
    if (t >= p[n - 1].t) return p[n - 1].snr;
    for (size_t i = 0; i + 1 < n; i++) {
        if (t >= p[i].t && t <= p[i + 1].t) {
            double dt = p[i + 1].t - p[i].t;
            if (dt <= 0.0) return p[i + 1].snr;           /* coincident times -> step */
            return p[i].snr + (t - p[i].t) / dt * (p[i + 1].snr - p[i].snr);
        }
    }
    return p[n - 1].snr;
}
static void schedule_record(channel_t *c, double t, double applied) {
    double nv = c->noise_std;
    double realized = nv > 0.0 ? 10.0 * log10(c->p_sig * F_NYQUIST / (nv * nv * BW_NOISE)) : applied;
    pthread_mutex_lock(&g_schedule.lock);
    g_schedule.applied_any = true;
    g_schedule.last_t = t; g_schedule.last_snr = applied;
    g_schedule.last_noise_std = nv; g_schedule.last_realized = realized;
    bool material = !g_schedule.logged || fabs(applied - g_schedule.log_snr) >= 0.1 ||
                    (t - g_schedule.log_t) >= 0.25;
    if (material) {
        if (g_schedule.nevents == g_schedule.cap_events) {
            size_t nc = g_schedule.cap_events ? g_schedule.cap_events * 2 : 256;
            sched_event_t *ne = realloc(g_schedule.events, nc * sizeof(*ne));
            if (ne) { g_schedule.events = ne; g_schedule.cap_events = nc; }
        }
        if (g_schedule.nevents < g_schedule.cap_events) {
            sched_event_t *e = &g_schedule.events[g_schedule.nevents++];
            e->wall = g_schedule.origin_wall + t; e->t = t; e->applied = applied;
            e->noise_std = nv; e->realized = realized;
            snprintf(e->chan, sizeof(e->chan), "%s", c->sched_tag[0] ? c->sched_tag : "");
        }
        g_schedule.logged = true; g_schedule.log_snr = applied; g_schedule.log_t = t;
    }
    pthread_mutex_unlock(&g_schedule.lock);
}
static void schedule_apply(channel_t *c) {
    double t;
    pthread_mutex_lock(&g_schedule.lock);
    if (!g_schedule.origin_set) {
        clock_gettime(CLOCK_MONOTONIC, &g_schedule.origin_mono);
        struct timespec rt; clock_gettime(CLOCK_REALTIME, &rt);
        g_schedule.origin_wall = (double)rt.tv_sec + rt.tv_nsec * 1e-9;
        snprintf(g_schedule.time_base, sizeof(g_schedule.time_base), "%s",
                 c->sched_offline ? "sample_clock" : "monotonic");
        g_schedule.origin_set = true;
    }
    if (c->sched_offline) {
        t = (double)c->sample_clock / RATE;
    } else {
        struct timespec now; clock_gettime(CLOCK_MONOTONIC, &now);
        t = (double)(now.tv_sec - g_schedule.origin_mono.tv_sec) +
            (now.tv_nsec - g_schedule.origin_mono.tv_nsec) * 1e-9;
        if (t < 0.0) t = 0.0;
    }
    pthread_mutex_unlock(&g_schedule.lock);
    double snr = schedule_interp(t);
    if (!c->sched_valid || snr != c->sched_last_snr) {
        c->snr_lin = pow(10.0, snr / 10.0);
        c->noise_std = noise_std(c, c->p_sig);
        c->sched_last_snr = snr; c->sched_valid = true;
    }
    schedule_record(c, t, snr);
}
static void schedule_finalize(void) {
    pthread_mutex_lock(&g_schedule.lock);
    if (g_schedule.applied_any &&
        (g_schedule.nevents == 0 || g_schedule.events[g_schedule.nevents - 1].t < g_schedule.last_t)) {
        if (g_schedule.nevents == g_schedule.cap_events) {
            size_t nc = g_schedule.cap_events ? g_schedule.cap_events * 2 : 256;
            sched_event_t *ne = realloc(g_schedule.events, nc * sizeof(*ne));
            if (ne) { g_schedule.events = ne; g_schedule.cap_events = nc; }
        }
        if (g_schedule.nevents < g_schedule.cap_events) {
            sched_event_t *e = &g_schedule.events[g_schedule.nevents++];
            e->wall = g_schedule.origin_wall + g_schedule.last_t; e->t = g_schedule.last_t;
            e->applied = g_schedule.last_snr; e->noise_std = g_schedule.last_noise_std;
            e->realized = g_schedule.last_realized; strcpy(e->chan, "end");
        }
    }
    pthread_mutex_unlock(&g_schedule.lock);
}
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
/*
 * The SNR3k axis defines a white noise PSD N0 = noise_std^2 / F_NYQUIST.  The
 * legacy path injected that PSD over the whole 0-24 kHz Nyquist band, so the
 * total noise power was 8x the 3 kHz reference power and, at low SNR3k, the
 * sum ran past S32 full scale (about half of all samples clip at -10 dB).  A
 * receiver never delivers noise outside its audio passband, so the noise is
 * low-passed here.  The FIR is a Blackman-windowed sinc normalised to unit DC
 * gain: its passband ripple is far below 0.001 dB, so the in-band PSD (and the
 * realized SNR3k) is unchanged.
 */
static int noise_lpf_init(channel_t*c,uint64_t prime_seed){
    if(!(c->opt.noise_lpf_hz>0)) return 0;
    int nt=c->opt.noise_lpf_taps; if(nt<3) nt=3; if(!(nt&1)) nt++; c->nf_n=nt;
    c->nf_h=calloc((size_t)nt,sizeof(double));c->nf_hist=calloc((size_t)nt,sizeof(double));
    if(!c->nf_h||!c->nf_hist)return -1;
    int m=(nt-1)/2;double wc=2*c->opt.noise_lpf_hz/RATE,sum=0;
    for(int k=0;k<nt;k++){
        double black=.42-.5*cos(2*PI*k/(nt-1))+.08*cos(4*PI*k/(nt-1));
        c->nf_h[k]=wc*sinc_np(wc*(k-m))*black;sum+=c->nf_h[k];
    }
    c->nf_gain2=0;for(int k=0;k<nt;k++){c->nf_h[k]/=sum;c->nf_gain2+=c->nf_h[k]*c->nf_h[k];}
    /* Pre-fill the history so the first output samples already carry the
     * steady-state noise power.  A separate generator keeps the main noise
     * sequence identical to the legacy draw order. */
    rng_t r;rng_init(&r,prime_seed);for(int k=0;k<nt;k++)c->nf_hist[k]=rng_gauss(&r);
    return 0;
}
static double noise_lpf_one(channel_t*c,double x){
    c->nf_hist[c->nf_pos]=x;double y=0;
    for(int k=0;k<c->nf_n;k++){int idx=c->nf_pos-(c->nf_n-1-k);while(idx<0)idx+=c->nf_n;y+=c->nf_h[k]*c->nf_hist[idx];}
    c->nf_pos=(c->nf_pos+1)%c->nf_n;return y;
}
/* Delivered noise power relative to the 3 kHz reference noise power. */
static double noise_power_ratio(const channel_t*c){
    return (F_NYQUIST/BW_NOISE)*(c->nf_n?c->nf_gain2:1.0);
}
/*
 * Receiver gain ahead of the S32 stage.  When the delivered noise RMS exceeds
 * headroom_rms the composite (signal and noise together, so every SNR is
 * unchanged) is scaled so the delivered noise RMS equals headroom_rms, the way
 * a receiver's AGC is noise-driven at low SNR.  The gain follows the noise
 * level (in geometry mode, the loudest noise level the direction has carried),
 * so it only moves when the noise level itself moves.  It is exactly 1
 * whenever the delivered noise RMS is at or below headroom_rms, so those
 * outputs are unchanged.  Sizing of the 0.12 default: the hottest Mercury
 * waveform is the MFSK robust/connect class at about 5-6x the OFDM WB power of
 * 0.0206, i.e. up to about 0.11-0.12.  Referenced to its own power at +10 dB
 * SNR3k its delivered noise RMS is sqrt(0.12 * 1.30 / 10) = 0.125, so +10 dB
 * keeps a gain of 0.96 or above for that class and exactly 1 for OFDM
 * (sqrt(0.0206 * 1.30 / 10) = 0.052).  The input signal is bounded by full
 * scale, so with the noise held at headroom_rms the composite essentially
 * never reaches full scale.
 */
/* Fading profiles: the two-path sum (g0*z + g1*zd)/sqrt(2) lifts a transmission
 * above its unfaded level whenever the taps add up, and the noise law above only
 * scales for the noise. On a fading channel the receiver gain is therefore capped
 * at 1/2 (6 dB of fade headroom): a transmission whose peaks sit near full scale
 * then clips only where the instantaneous fade gain exceeds 2, i.e. the tap power
 * exceeds 4x its mean (Rayleigh: exp(-4) of the time) AND the waveform is at a
 * peak. The scale multiplies signal and noise together, so SNR is unchanged. */
#define FADE_HEADROOM_SCALE 0.5
static double fade_capped(const channel_t*c,double s){
    return (c->fading&&c->opt.headroom_rms>0&&!c->opt.passthrough&&s>FADE_HEADROOM_SCALE)?FADE_HEADROOM_SCALE:s;
}
static double composite_scale_for(const channel_t*c){
    if(!(c->opt.headroom_rms>0)||c->opt.passthrough)return 1.0;
    double sn=c->noise_std*(c->nf_n?sqrt(c->nf_gain2):1.0);
    return fade_capped(c,sn>c->opt.headroom_rms?c->opt.headroom_rms/sn:1.0);
}
static int channel_init(channel_t*c,const options_t*o,uint32_t seed){
    memset(c,0,sizeof(*c));c->opt=*o;
    rng_init(&c->noise_rng,seed);
    uint64_t tap_seed=splitmix_next(&c->noise_rng); /* Python seed_np consumes one u64. */
    rng_init(&c->pn_rng,tap_seed^UINT64_C(0xD1B54A32D192ED03));
    c->snr_lin=pow(10.0,o->snr/10.0);c->p_sig=pow(fmax(o->sig_ref,1e-6),2);
    c->noise_std=noise_std(c,c->p_sig);
    const char*mode=o->reference_mode;
    c->psig_mode=!strcmp(mode,"bench")?PSIG_BENCH:!strcmp(mode,"fix")?PSIG_FIX:!strcmp(mode,"peak")?PSIG_PEAK:!strcmp(mode,"geometry")?PSIG_GEOMETRY:PSIG_STEADY;
    c->psig_fix=o->psig_fix;
    if(c->psig_mode==PSIG_FIX&&c->psig_fix>0){c->p_sig=c->psig_fix;c->noise_std=noise_std(c,c->p_sig);}
    if(c->psig_mode==PSIG_GEOMETRY){c->floor_ref=c->cur_ref=c->p_sig=o->floor_ref;c->noise_std=noise_std(c,c->p_sig);c->cur_cls=c->rec_cls=-1;c->worst_snr_err_db=0;}
    /* Bench mirror: one noise level, fixed from the first sample, set from the
     * OFDM WB data power.  The transmission classes are kept for the record
     * only; they never change the noise. */
    if(c->psig_mode==PSIG_BENCH){c->floor_ref=c->cur_ref=c->p_sig=o->ofdm_ref;c->noise_std=noise_std(c,c->p_sig);c->cur_cls=c->rec_cls=-1;c->worst_snr_err_db=0;}
    for(int k=0;k<KIND_N;k++){c->kind[k].snr_min=1e9;c->kind[k].snr_max=-1e9;}
    if(c->psig_mode==PSIG_STEADY){c->heap_cap=16384;c->heap_lo=malloc(c->heap_cap*sizeof(double));c->heap_hi=malloc(c->heap_cap*sizeof(double));if(!c->heap_lo||!c->heap_hi)return -1;}
    double dtau,fd;profile_params(o->profile,&dtau,&fd);c->fading=dtau>0;
    if(c->fading){
        c->delay=(size_t)llround(dtau*RATE);if(c->delay<1)c->delay=1;
        hilbert_init(c);doppler_init(&c->tap0,fd,tap_seed^UINT64_C(0x9E3779B97F4A7C15));
        doppler_init(&c->tap1,fd,tap_seed^UINT64_C(0xBF58476D1CE4E5B9));
    }
    if(bandpass_init(c))return -1;
    if(noise_lpf_init(c,tap_seed^UINT64_C(0x632BE59BD9B4E019)))return -1;
    c->composite_scale=c->scale_min=composite_scale_for(c);
    c->noise_rms_max=c->noise_std;
    return 0;
}
static void channel_free(channel_t*c){free(c->heap_lo);free(c->heap_hi);free(c->bp_h);free(c->bp_hist);free(c->nf_h);free(c->nf_hist);}
/* Scale by the session gain, clamp to S32 full scale, truncate toward zero.
 * pre_peak/over_unscaled describe the composite before the gain (what the
 * legacy path would have clipped); hard counts samples that actually clip. */
static void emit_s32(const channel_t*c,const double*out,int32_t*ob,size_t n,
                     double*pre_peak,uint64_t*hard,uint64_t*over_unscaled){
    const double g=c->composite_scale>0?c->composite_scale:1.0;
    for(size_t i=0;i<n;i++){
        double a=fabs(out[i]);if(a>*pre_peak)*pre_peak=a;if(a>1.0)(*over_unscaled)++;
        double s=out[i]*g;if(fabs(s)>1.0)(*hard)++;
        double v=s*INT_MAX_D;if(v>INT_MAX_D)v=INT_MAX_D;if(v< -INT_MAX_D)v=-INT_MAX_D;
        int32_t q=(int32_t)v;ob[2*i]=ob[2*i+1]=q;
    }
}

static int append_active(channel_t*c,double ms){
    if(c->heap_lo_n+c->heap_hi_n>=c->heap_cap){size_t cap=c->heap_cap*2;double*lo=realloc(c->heap_lo,cap*sizeof(double));if(!lo)return-1;c->heap_lo=lo;double*hi=realloc(c->heap_hi,cap*sizeof(double));if(!hi)return-1;c->heap_hi=hi;c->heap_cap=cap;}
    if(!c->heap_lo_n||ms<=c->heap_lo[0])max_push(c->heap_lo,&c->heap_lo_n,ms);else min_push(c->heap_hi,&c->heap_hi_n,ms);
    if(c->heap_lo_n>c->heap_hi_n+1){double v=max_pop(c->heap_lo,&c->heap_lo_n);min_push(c->heap_hi,&c->heap_hi_n,v);}
    else if(c->heap_hi_n>c->heap_lo_n){double v=min_pop(c->heap_hi,&c->heap_hi_n);max_push(c->heap_lo,&c->heap_lo_n,v);}
    c->active_n++;return 0;
}
/*
 * Noise reference modes.
 *
 * The SNR3k axis is a statement about the signal a receiver is trying to
 * decode: signal power over the noise power in 3 kHz.  Mercury transmits
 * several waveforms at different power (OFDM data, MFSK robust data, the MFSK
 * connect and acknowledgement bursts run about 5-6x the OFDM mean power).
 *
 *   bench (default): mirrors the IONOS channel simulator, which adds one fixed
 *     noise level per S:N setting and never measures its input.  The noise
 *     power is set once, from the first sample, from the OFDM WB data power
 *     (ofdm_ref), and is the same on both directions.  "--snr3k N" is then the
 *     SNR3k that Mercury's OFDM WB data realizes; any other waveform realizes
 *     N + 10*log10(P_waveform / ofdm_ref), as it does on the bench.  Every
 *     transmission is recorded with its measured power, the applied noise, the
 *     realized SNR3k, its level offset from ofdm_ref and its waveform kind.
 *   per-class (alias geometry): every transmission is referenced to its own waveform
 *     class.  A transmission starts at the first active sample after silence
 *     and ends after GEO_QUIET_RUN quiet samples.  Its onset power (the rest of
 *     the chunk it starts in) picks a class within class_tol_db of the class
 *     onset power, or opens a new class.  The noise inside the transmission is
 *     set from the class reference (the mean power of its completed
 *     transmissions); for the first transmission of a class, from its own
 *     running mean.  A sustained power step inside one transmission
 *     (shift_db for GEO_SHIFT_CHUNKS chunks) starts a new class segment.
 *     Silence carries the floor noise, referenced to floor_ref (default: the
 *     measured Mercury OFDM WB transmit power), so the noise a receiver sees
 *     ahead of an OFDM data block already matches the commanded level.  Each
 *     direction keeps its own classes.
 *   steady (legacy): running median of all active chunk powers since start.
 *   fix: fixed reference power (--psig-fix).
 *   peak: highest chunk power seen.
 */
static void geo_open(channel_t*c,double est,bool*first){
    int best=-1;double bd=1e9;
    for(int k=0;k<c->ncls;k++){double d=fabs(10.0*log10(est/c->cls[k].onset_pow));if(d<bd){bd=d;best=k;}}
    if(best<0||(bd>c->opt.class_tol_db&&c->ncls<GEO_MAX_CLASSES)){
        best=c->ncls++;memset(&c->cls[best],0,sizeof(c->cls[best]));
        c->cls[best].onset_pow=c->cls[best].ref_pow=est;c->cls[best].snr_min=1e9;c->cls[best].snr_max=-1e9;
    }else if(c->cls[best].bursts>0){
        c->cls[best].onset_pow=exp(0.8*log(c->cls[best].onset_pow)+0.2*log(est));
    }
    c->cur_cls=best;*first=c->cls[best].bursts==0;
}
typedef struct { size_t a,b; bool burst,onset,offset,split,first; int cls; double ns; } geo_seg_t;
static FILE*g_burst_log;static pthread_mutex_t g_burst_log_lock=PTHREAD_MUTEX_INITIALIZER;
/* Clip log: one JSON line per processed chunk that carried a clipped sample, with the
 * chunk's position on the bridge sample clock and the CLOCK_MONOTONIC time it was
 * emitted, so a clip can be placed against the harness clock (same clock on Linux).
 * lat_s bounds how long before mono_s the chunk's samples entered the capture ring. */
static FILE*g_clip_log;static pthread_mutex_t g_clip_log_lock=PTHREAD_MUTEX_INITIALIZER;
static void clip_event(const char*dir,uint64_t chunk_start,size_t n,double lat_s,
                       uint64_t hard,uint64_t over,uint64_t infs,double scale){
    if(!g_clip_log||!(hard||over||infs))return;
    struct timespec now;clock_gettime(CLOCK_MONOTONIC,&now);
    pthread_mutex_lock(&g_clip_log_lock);
    fprintf(g_clip_log,"{\"dir\":\"%s\",\"event\":\"clip\",\"t_s\":%.6f,\"dur_s\":%.6f,\"mono_s\":%.6f,\"lat_s\":%.6f,\"hard_clips\":%"PRIu64",\"over_fs_unscaled\":%"PRIu64",\"input_at_fs\":%"PRIu64",\"composite_scale\":%.9g}\n",
            dir,(double)chunk_start/RATE,(double)n/RATE,(double)now.tv_sec+now.tv_nsec*1e-9,lat_s,hard,over,infs,scale);
    fflush(g_clip_log);
    pthread_mutex_unlock(&g_clip_log_lock);
}
/*
 * Waveform features of one transmission, from the clean input:
 *   kurtosis E[x^4]/E[x^2]^2: a constant-envelope tone is 1.5, two equal tones
 *     2.25 (Mercury's MFSK sends at most two tones at once), a band-limited
 *     Gaussian-like multicarrier signal 3 (OFDM, slightly lower after its
 *     10 dB PAPR clip);
 *   occupied band from the averaged periodogram (Hann-windowed SPEC_N-point
 *     frames of the active samples): the equivalent width
 *     (sum P)^2 / sum P^2 * bin width, which is the width of a flat band, and
 *     the power-weighted centre.
 * Kind (thresholds are the midpoints between the feature ranges measured on
 * this bridge for every Mercury waveform, pinned configs 0/7/8/13-16 and
 * 100-103):
 *   kurtosis >= kind_kurt_thr (2.6): OFDM data (measured 2.87-3.03), wide band
 *     (ofdm-wb, measured 2342-2397 Hz) when the width is above kind_bw_thr_hz,
 *     else ofdm-nb;
 *   otherwise width >= 1800 Hz: the OFDM block preamble segment (kurtosis
 *     2.29-2.35, width 2000-2024 Hz, about 0.05 s at -4 dB);
 *   otherwise MFSK: width below 1200 Hz is the connect / acknowledgement /
 *     control MFSK (kurtosis 1.51, width 729-854 Hz), wider is robust data
 *     (config 100: kurtosis 1.52, width 1549-1586 Hz; configs 101-103 key two
 *     tones: kurtosis 2.26-2.27, width 1519-1595 Hz).
 * Transmissions shorter than 20 ms are "short" (too few samples).
 */
#define KIND_PREAMBLE_BW_HZ 1800.0
#define KIND_MFSK_DATA_BW_HZ 1200.0
static void fft_inplace(double complex*a,int n){
    for(int i=1,j=0;i<n;i++){int bit=n>>1;for(;j&bit;bit>>=1)j^=bit;j^=bit;if(i<j){double complex t=a[i];a[i]=a[j];a[j]=t;}}
    for(int len=2;len<=n;len<<=1){double complex w=cexp(-2.0*PI*I/len);
        for(int i=0;i<n;i+=len){double complex wk=1;for(int k=0;k<len/2;k++){double complex u=a[i+k],v=a[i+k+len/2]*wk;a[i+k]=u+v;a[i+k+len/2]=u-v;wk*=w;}}}
}
static void spec_frame(channel_t*c){
    double complex a[SPEC_N];
    for(int i=0;i<SPEC_N;i++)a[i]=c->spec_buf[i]*(0.5-0.5*cos(2.0*PI*i/SPEC_N));
    fft_inplace(a,SPEC_N);
    for(int k=0;k<=SPEC_N/2;k++){double p=creal(a[k])*creal(a[k])+cimag(a[k])*cimag(a[k]);c->spec_acc[k]+=p;}
    c->spec_frames++;c->spec_fill=0;
}
static int geo_kind(const channel_t*c,double dur_s,double*kurt,double*bw_hz,double*fc_hz){
    *kurt=*bw_hz=*fc_hz=0;
    if(c->b_na<1||!(c->b_ss>0))return KIND_SHORT;
    double m2x=c->b_ss/c->b_na;*kurt=(c->b_x4/c->b_na)/(m2x*m2x);
    if(c->spec_frames>0){
        double s1=0,s2=0,sf=0;const double df=(double)RATE/SPEC_N;
        for(int k=0;k<=SPEC_N/2;k++){double p=c->spec_acc[k];s1+=p;s2+=p*p;sf+=p*k*df;}
        if(s2>0){*bw_hz=s1*s1/s2*df;*fc_hz=sf/s1;}
    }
    if(dur_s<0.02)return KIND_SHORT;
    if(*kurt>=c->opt.kind_kurt_thr)return *bw_hz>c->opt.kind_bw_thr_hz?KIND_OFDM_WB:KIND_OFDM_NB;
    if(*bw_hz>=KIND_PREAMBLE_BW_HZ)return KIND_OFDM_PREAMBLE;
    return *bw_hz<KIND_MFSK_DATA_BW_HZ?KIND_MFSK_CTL:KIND_MFSK_DATA;
}
static void geo_finalize(channel_t*c,bool split){
    if(!c->b_n){c->b_ss=c->b_nv=c->b_nm=0;c->b_na=c->b_x4=0;c->spec_fill=0;c->spec_frames=0;memset(c->spec_acc,0,sizeof(c->spec_acc));return;}
    double n=(double)c->b_n,p=c->b_ss/n,nv=c->b_nv/n;
    double g2=c->nf_n?c->nf_gain2:1.0,inband=BW_NOISE/F_NYQUIST;
    double snr_nom=nv>0?10.0*log10(p/(nv*inband)):INFINITY;
    double nm=c->b_nm/n/g2;double snr_meas=nm>0?10.0*log10(p/(nm*inband)):INFINITY;
    int k=c->rec_cls;
    if(k>=0){
        geo_class_t*q=&c->cls[k];q->ss+=c->b_ss;q->n+=n;q->ref_pow=q->ss/q->n;q->bursts++;
        if(isfinite(snr_nom)){q->snr_sum+=snr_nom;if(snr_nom<q->snr_min)q->snr_min=snr_nom;if(snr_nom>q->snr_max)q->snr_max=snr_nom;
            if(c->psig_mode==PSIG_GEOMETRY){double e=fabs(snr_nom-c->opt.snr);if(e>c->worst_snr_err_db)c->worst_snr_err_db=e;}}
    }
    double kurt,bw,fc;int kd=geo_kind(c,n/RATE,&kurt,&bw,&fc);
    geo_kind_t*gk=&c->kind[kd];gk->tx++;gk->ss+=c->b_ss;gk->n+=n;gk->nv+=c->b_nv;
    if(isfinite(snr_nom)){gk->snr_sum+=snr_nom;if(snr_nom<gk->snr_min)gk->snr_min=snr_nom;if(snr_nom>gk->snr_max)gk->snr_max=snr_nom;}
    /* Bench mode: the level offset from the OFDM WB reference is the realized
     * SNR3k minus the label, by construction of the fixed noise. */
    double level_db=10.0*log10(p/c->opt.ofdm_ref);
    if(g_burst_log){
        pthread_mutex_lock(&g_burst_log_lock);
        fprintf(g_burst_log,"{\"dir\":\"%s\",\"i\":%"PRIu64",\"t0_s\":%.6f,\"dur_s\":%.6f,\"cls\":%d,\"cls_first\":%s,\"split\":%s,\"p_sig\":%.9g,\"noise_var\":%.9g,\"snr3k_nominal_db\":%.4f,\"snr3k_measured_db\":%.4f,\"commanded_snr3k_db\":%.4f,\"composite_scale\":%.9g,\"kind\":\"%s\",\"kurtosis\":%.4f,\"band_hz\":%.1f,\"centre_hz\":%.1f,\"level_vs_ofdm_ref_db\":%.4f,\"reference_mode\":\"%s\"}\n",
                c->name,c->bursts_done,(double)c->b_start/RATE,n/RATE,k,c->rec_first?"true":"false",split?"true":"false",p,nv,snr_nom,snr_meas,c->opt.snr,c->composite_scale,
                kind_name[kd],kurt,bw,fc,level_db,c->opt.reference_mode);
        fflush(g_burst_log);
        pthread_mutex_unlock(&g_burst_log_lock);
    }
    c->bursts_done++;c->b_ss=c->b_nv=c->b_nm=0;c->b_n=0;c->b_na=c->b_x4=0;c->spec_fill=0;c->spec_frames=0;memset(c->spec_acc,0,sizeof(c->spec_acc));
}
/* Split one chunk into silence / transmission segments and choose the noise
 * level of each.  Returns the segment count. */
static int geo_plan(channel_t*c,const double*x,size_t n,geo_seg_t*seg){
    int ns=0;size_t i=0;
    double seg_ss=0;size_t seg_act=0,pend=0;
    geo_seg_t cur={.a=0,.burst=c->in_burst,.cls=c->cur_cls};
    for(i=0;i<n;i++){
        bool a=fabs(x[i])>GEO_ACTIVE;
        if(!c->in_burst){
            if(a){
                if(i>cur.a){cur.b=i;seg[ns++]=cur;}
                cur=(geo_seg_t){.a=i,.burst=true,.onset=true,.cls=-1};
                c->in_burst=true;c->quiet_run=0;seg_ss=x[i]*x[i];seg_act=1;pend=0;
            }
        }else if(a){c->quiet_run=0;seg_act+=pend+1;pend=0;seg_ss+=x[i]*x[i];}
        else if(++c->quiet_run>=GEO_QUIET_RUN){
            cur.offset=true;cur.b=i+1;cur.ns=seg_act?seg_ss/(double)seg_act:0;
            seg[ns++]=cur;
            cur=(geo_seg_t){.a=i+1,.cls=-1};c->in_burst=false;seg_ss=0;seg_act=0;pend=0;
        }else pend++;
    }
    cur.b=n;if(cur.burst)cur.ns=seg_act?seg_ss/(double)seg_act:0;
    if(cur.b>cur.a||ns==0)seg[ns++]=cur;
    /* Decide the reference of each segment (ns field: power in, noise std out). */
    for(int s=0;s<ns;s++){
        geo_seg_t*g=&seg[s];
        if(!g->burst){g->ns=noise_std(c,c->floor_ref);continue;}
        double pw=g->ns;size_t len=g->b-g->a;
        if(g->onset){
            geo_open(c,pw>0?pw:c->floor_ref,&c->cur_first);c->shift_run=0;
            c->b_onset=pw;c->b_ss_d=0;c->b_n_d=0;
        }else if(!c->cur_first&&len>=GEO_SHIFT_MIN_SAMPLES&&pw>0&&fabs(10.0*log10(pw/c->cur_ref))>c->opt.shift_db){
            if(++c->shift_run>=GEO_SHIFT_CHUNKS){
                g->split=true;geo_open(c,pw,&c->cur_first);c->shift_run=0;c->b_ss_d=0;c->b_n_d=0;c->class_switches++;
            }
        }else c->shift_run=0;
        c->b_ss_d+=pw*(double)len;c->b_n_d+=(double)len;
        geo_class_t*q=&c->cls[c->cur_cls];
        c->cur_ref=c->cur_first?(c->b_n_d>0?c->b_ss_d/c->b_n_d:pw):q->ref_pow;
        if(!(c->cur_ref>0))c->cur_ref=c->floor_ref;
        g->cls=c->cur_cls;g->first=c->cur_first;g->ns=noise_std(c,c->cur_ref);
    }
    return ns;
}
/* Account one chunk's segments after impairment: per-transmission signal and
 * noise power (nominal from the applied level, measured from the injected
 * noise), and the per-transmission record. */
static void geo_account(channel_t*c,const double*x,const double*ninj,const geo_seg_t*seg,int ns,const double*nstd,size_t n){
    (void)n;
    for(int s=0;s<ns;s++){
        const geo_seg_t*g=&seg[s];
        if(!g->burst)continue;
        if(g->onset||g->split){
            if(g->split)geo_finalize(c,true);
            c->rec_cls=g->cls;c->rec_first=g->first;
            c->b_start=c->sample_clock+g->a;c->b_ss=c->b_nv=c->b_nm=0;c->b_n=0;c->pend_quiet=0;c->pend_nv=c->pend_nm=0;
            c->b_na=c->b_x4=0;c->spec_fill=0;c->spec_frames=0;memset(c->spec_acc,0,sizeof(c->spec_acc));
        }
        for(size_t i=g->a;i<g->b;i++){
            double nv=nstd[i]*nstd[i],nm=ninj[i]*ninj[i];
            if(fabs(x[i])>GEO_ACTIVE){c->b_n+=c->pend_quiet+1;c->b_nv+=c->pend_nv+nv;c->b_nm+=c->pend_nm+nm;c->b_ss+=x[i]*x[i];c->pend_quiet=0;c->pend_nv=c->pend_nm=0;
                double x2=x[i]*x[i];c->b_na+=1;c->b_x4+=x2*x2;
                c->spec_buf[c->spec_fill++]=x[i];if(c->spec_fill==SPEC_N)spec_frame(c);}
            else{c->pend_quiet++;c->pend_nv+=nv;c->pend_nm+=nm;}
        }
        if(g->offset)geo_finalize(c,false);
    }
}
static void channel_process(channel_t*c,const double*x,double*out,size_t n){
    if(g_schedule.loaded)schedule_apply(c);
    double nstd[PERIOD],ninj[PERIOD];geo_seg_t seg[GEO_SEG_MAX];int nseg=0;
    const bool geo=c->psig_mode==PSIG_GEOMETRY,bench=c->psig_mode==PSIG_BENCH;
    double ms=0;for(size_t i=0;i<n;i++)ms+=x[i]*x[i];if(n)ms/=n;
    bool peak=ms>c->peak_ms;if(peak)c->peak_ms=ms;
    if(c->psig_mode==PSIG_STEADY&&ms>1e-7){
        if(!append_active(c,ms)&&++c->active_since>=8){c->active_since=0;c->p_sig=median_active(c);c->noise_std=noise_std(c,c->p_sig);}
    }else if(c->psig_mode==PSIG_PEAK&&peak){c->p_sig=c->peak_ms;c->noise_std=noise_std(c,c->p_sig);}
    if(bench){
        /* Record-only segmentation; the noise stays at the one fixed level. */
        nseg=geo_plan(c,x,n,seg);
        for(size_t i=0;i<n;i++)nstd[i]=c->noise_std;
        c->composite_scale=composite_scale_for(c);
    }else if(geo){
        nseg=geo_plan(c,x,n,seg);
        for(int s=0;s<nseg;s++){for(size_t i=seg[s].a;i<seg[s].b;i++)nstd[i]=seg[s].ns;if(seg[s].ns>c->noise_rms_max)c->noise_rms_max=seg[s].ns;}
        /* The receiver gain follows the loudest noise level this direction has
         * carried, so it is constant inside a transmission and only ever steps
         * down, when a louder class first appears. */
        double sn=c->noise_rms_max*(c->nf_n?sqrt(c->nf_gain2):1.0);
        c->composite_scale=fade_capped(c,(!(c->opt.headroom_rms>0)||c->opt.passthrough||sn<=c->opt.headroom_rms)?1.0:c->opt.headroom_rms/sn);
    }else c->composite_scale=composite_scale_for(c);
    if(c->composite_scale<c->scale_min)c->scale_min=c->composite_scale;

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
        const double nsd=(geo||bench)?nstd[i]:c->noise_std;
        if(nsd>0){double w=rng_gauss(&c->noise_rng);if(c->nf_n)w=noise_lpf_one(c,w);v+=nsd*w;if(geo||bench)ninj[i]=nsd*w;}
        else if(geo||bench)ninj[i]=0;
        if(c->opt.burst&&c->opt.loss>0){
            if(!c->ge_state){if(rng_uniform(&c->noise_rng)<c->opt.loss*.05)c->ge_state=1;}
            else{v=0;if(rng_uniform(&c->noise_rng)<.03)c->ge_state=0;}
        }else if(c->opt.loss>0&&rng_uniform(&c->noise_rng)<c->opt.loss)v=0;
        out[i]=v;
    }
    if(geo||bench)geo_account(c,x,ninj,seg,nseg,nstd,n);
    c->sample_clock+=n;
}

typedef struct {
    uint64_t frames,sig_frames,underruns,hard_clips,s32_saturations;
    uint64_t clip_denominator,over_unscaled,input_fs;
    double sig_sumsq,pre_scale_peak,reference_power,noise_variance,scale,scale_min;
    pthread_mutex_t lock;
} pump_stats_t;
typedef struct {
    char name[8],cap_dev[256],play_dev[256];
    channel_t channel;const options_t*opt;pump_stats_t stats;
    int erase_target_burst;
    bool erase_prev_silent;
    uint64_t signal_bursts_seen,erased_chunks;
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
                      double peak,uint64_t hard,uint64_t saturation,size_t clip_n,
                      uint64_t over_unscaled,uint64_t input_fs){
    pthread_mutex_lock(&p->stats.lock);p->stats.frames+=n;if(signal){p->stats.sig_frames+=n;p->stats.sig_sumsq+=ss;}p->stats.underruns+=underrun;
    if(peak>p->stats.pre_scale_peak)p->stats.pre_scale_peak=peak;
    p->stats.hard_clips+=hard;p->stats.s32_saturations+=saturation;p->stats.over_unscaled+=over_unscaled;p->stats.input_fs+=input_fs;
    p->stats.clip_denominator+=clip_n;p->stats.reference_power=p->channel.p_sig;
    p->stats.noise_variance=p->channel.noise_std*p->channel.noise_std;
    p->stats.scale=p->channel.composite_scale;p->stats.scale_min=p->channel.scale_min;
    pthread_mutex_unlock(&p->stats.lock);
}
static void maybe_erase_burst(pump_t*p,double*x,size_t n,bool signal){
    bool erase=false;
    pthread_mutex_lock(&p->stats.lock);
    if(p->erase_target_burst>0){
        if(signal&&p->erase_prev_silent)p->signal_bursts_seen++;
        p->erase_prev_silent=!signal;
        erase=signal&&p->signal_bursts_seen==(uint64_t)p->erase_target_burst;
        if(erase)p->erased_chunks++;
    }
    pthread_mutex_unlock(&p->stats.lock);
    if(erase)memset(x,0,n*sizeof(*x));
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
    for(int i=0;i<p->opt->prime_periods;i++){int xruns=0;if(write_frames(play,ob,PERIOD,&xruns)<0)xruns++;if(xruns)stats_add(p,0,false,0,xruns,0,0,0,0,0,0);}
    while(!stop_requested){
        snd_pcm_sframes_t nr=snd_pcm_readi(cap,in,PERIOD);
        if(nr<0){stats_add(p,0,false,0,1,0,0,0,0,0,0);if(stop_requested||snd_pcm_recover(cap,(int)nr,1)<0)break;continue;}
        size_t n=(size_t)nr;bool signal=false;double ss=0;uint64_t infs=0;
        for(size_t i=0;i<n;i++){if(in[2*i])signal=true;if(in[2*i]>=INT32_MAX||in[2*i]<=-INT32_MAX)infs++;x[i]=(double)in[2*i]/INT_MAX_D;ss+=x[i]*x[i];}
        maybe_erase_burst(p,x,n,signal);
        double peak=0;uint64_t hard=0,over=0;const uint64_t chunk_start=p->channel.sample_clock;
        if(p->opt->passthrough){for(size_t i=0;i<n;i++)ob[2*i]=ob[2*i+1]=in[2*i];}
        else{
            channel_process(&p->channel,x,out,n);
            emit_s32(&p->channel,out,ob,n,&peak,&hard,&over);
        }
        clip_event(p->name,chunk_start,n,(double)(n+(size_t)p->opt->cap_periods*PERIOD)/RATE,hard,over,infs,p->channel.composite_scale);
        int xruns=0;int wr=write_frames(play,ob,n,&xruns);if(wr<0)xruns++;
        stats_add(p,n,signal,ss,xruns,peak,hard,hard,p->opt->passthrough?0:n,over,infs);if(wr<0&&!stop_requested)snd_pcm_prepare(play);
    }
    pthread_mutex_lock(&p->pcm_lock);p->cap_shared=p->play_shared=NULL;
    snd_pcm_close(cap);snd_pcm_close(play);pthread_mutex_unlock(&p->pcm_lock);
    return NULL;
}

static void json_string(FILE*f,const char*s){fputc('"',f);for(;*s;s++){unsigned char c=*s;if(c=='"'||c=='\\'){fputc('\\',f);fputc(c,f);}else if(c<32)fprintf(f,"\\u%04x",c);else fputc(c,f);}fputc('"',f);}
/* Emits the schedule origin/intent/applied keys (no leading/trailing comma) so
 * the caller can splice them into an existing object.  Locked because the pump
 * threads append events while the main thread periodically flushes. */
static void schedule_emit_json(FILE*f){
    pthread_mutex_lock(&g_schedule.lock);
    fprintf(f,"\n  \"snr_schedule_origin\": {\"wall_clock_epoch\": %.6f, \"monotonic_origin_s\": %.6f, \"time_base\": ",
            g_schedule.origin_wall,
            (double)g_schedule.origin_mono.tv_sec+g_schedule.origin_mono.tv_nsec*1e-9);
    json_string(f,g_schedule.time_base[0]?g_schedule.time_base:"unset");
    fprintf(f,"},\n  \"snr_schedule_intent\": [");
    for(size_t i=0;i<g_schedule.n;i++)
        fprintf(f,"%s\n    {\"t_offset_s\": %.6f, \"snr_db\": %.6f}",i?",":"",g_schedule.pts[i].t,g_schedule.pts[i].snr);
    fprintf(f,"\n  ],\n  \"snr_schedule_applied\": [");
    for(size_t i=0;i<g_schedule.nevents;i++){
        sched_event_t*e=&g_schedule.events[i];
        fprintf(f,"%s\n    {\"wall_clock\": %.6f, \"t_offset_s\": %.6f, \"applied_snr3k_db\": %.6f, \"noise_std\": %.9g, \"realized_snr3k\": %.6f, \"chan\": ",
                i?",":"",e->wall,e->t,e->applied,e->noise_std,e->realized);
        json_string(f,e->chan);
        fputc('}',f);
    }
    fprintf(f,"\n  ]");
    pthread_mutex_unlock(&g_schedule.lock);
}
static int write_schedule_statsfile(const options_t*o){
    if(!o->statsfile[0]||!g_schedule.loaded)return 0;
    char tmp[1200];snprintf(tmp,sizeof(tmp),"%s.tmp.%ld",o->statsfile,(long)getpid());
    FILE*f=fopen(tmp,"w");if(!f)return-1;
    fputc('{',f);schedule_emit_json(f);fprintf(f,"\n}\n");
    fflush(f);fsync(fileno(f));
    if(fclose(f)||rename(tmp,o->statsfile)){unlink(tmp);return-1;}
    return 0;
}
static void stat_snapshot(pump_t*p,uint64_t*fr,uint64_t*sf,double*ss,uint64_t*ur,
                          double*peak,uint64_t*hard,uint64_t*sat,uint64_t*den,
                          double*reference_power,double*noise_variance,
                          uint64_t*bursts,uint64_t*erased,uint64_t*over,uint64_t*infs,double*scale,double*scale_min){
    pthread_mutex_lock(&p->stats.lock);*over=p->stats.over_unscaled;*infs=p->stats.input_fs;*scale=p->stats.scale;*scale_min=p->stats.scale_min;*fr=p->stats.frames;*sf=p->stats.sig_frames;*ss=p->stats.sig_sumsq;*ur=p->stats.underruns;*peak=p->stats.pre_scale_peak;*hard=p->stats.hard_clips;*sat=p->stats.s32_saturations;*den=p->stats.clip_denominator;*reference_power=p->stats.reference_power;*noise_variance=p->stats.noise_variance;*bursts=p->signal_bursts_seen;*erased=p->erased_chunks;pthread_mutex_unlock(&p->stats.lock);
}
/* Per-direction class table (geometry mode).  Read without the pump lock:
 * each field is an aligned double/integer written by one pump thread. */
static void geo_json(FILE*f,const channel_t*c){
    fprintf(f,"{\"transmissions\": %"PRIu64", \"class_switches\": %"PRIu64", \"worst_snr3k_error_db\": %.6g, \"classes\": [",c->bursts_done,c->class_switches,c->worst_snr_err_db);
    int n=c->ncls;if(n>GEO_MAX_CLASSES)n=GEO_MAX_CLASSES;
    for(int k=0;k<n;k++){const geo_class_t*q=&c->cls[k];
        fprintf(f,"%s{\"id\": %d, \"onset_power\": %.9g, \"reference_power\": %.9g, \"transmissions\": %"PRIu64", \"airtime_s\": %.6f, \"snr3k_mean_db\": %.4f, \"snr3k_min_db\": %.4f, \"snr3k_max_db\": %.4f}",
                k?", ":"",k,q->onset_pow,q->ref_pow,q->bursts,q->n/RATE,q->bursts?q->snr_sum/(double)q->bursts:0.0,q->bursts?q->snr_min:0.0,q->bursts?q->snr_max:0.0);}
    fprintf(f,"]}");
}
/* Per-direction, per-waveform-kind attestation: transmissions, airtime, mean
 * signal power, its offset from the OFDM WB reference, the applied noise and
 * the realized SNR3k. */
static void kinds_json(FILE*f,const channel_t*c){
    const double inband=BW_NOISE/F_NYQUIST;fprintf(f,"{");
    for(int k=0;k<KIND_N;k++){const geo_kind_t*q=&c->kind[k];
        double p=q->n>0?q->ss/q->n:0,nv=q->n>0?q->nv/q->n:0;
        fprintf(f,"%s\"%s\": {\"transmissions\": %"PRIu64", \"airtime_s\": %.6f, \"signal_power\": %.9g, \"level_vs_ofdm_ref_db\": %.4f, \"noise_var\": %.9g, \"snr3k_airtime_db\": %.4f, \"snr3k_mean_db\": %.4f, \"snr3k_min_db\": %.4f, \"snr3k_max_db\": %.4f}",
                k?", ":"",kind_name[k],q->tx,q->n/RATE,p,p>0?10.0*log10(p/c->opt.ofdm_ref):0.0,nv,(p>0&&nv>0)?10.0*log10(p/(nv*inband)):0.0,
                q->tx?q->snr_sum/(double)q->tx:0.0,q->tx?q->snr_min:0.0,q->tx?q->snr_max:0.0);}
    fprintf(f,"}");
}
static const char*reference_mode_label(const char*m){
    return !strcmp(m,"bench")?"bench-fixed-ofdm-wb":!strcmp(m,"geometry")?"geometry-class":!strcmp(m,"fix")?"fixed":!strcmp(m,"peak")?"peak":"steady-active-chunk-median";
}
static const char*reference_id_label(const char*m){
    return !strcmp(m,"bench")?"bench-ofdm-wb-v1":!strcmp(m,"geometry")?"geometry-class-v1":!strcmp(m,"fix")?"fixed-cli":!strcmp(m,"peak")?"peak":"traffic-steady-v1";
}
static int flush_stats(const options_t*o,pump_t*fwd,pump_t*rev){
    if(!o->statsfile[0]) return 0;
    char tmp[1200];snprintf(tmp,sizeof(tmp),"%s.tmp.%ld",o->statsfile,(long)getpid());FILE*f=fopen(tmp,"w");if(!f)return-1;
    uint64_t ff,fs,fu,fh,fx,fd,fb,fe,fo,fi,rf,rs,ru,rh,rx,rd,rb,re,ro,ri;double fss,rss,fp,rp,fref,fnoise,rref,rnoise,fg,fgm,rg,rgm;
    stat_snapshot(fwd,&ff,&fs,&fss,&fu,&fp,&fh,&fx,&fd,&fref,&fnoise,&fb,&fe,&fo,&fi,&fg,&fgm);stat_snapshot(rev,&rf,&rs,&rss,&ru,&rp,&rh,&rx,&rd,&rref,&rnoise,&rb,&re,&ro,&ri,&rg,&rgm);
    fprintf(f,"{\n  \"fwd\": {\"frames\": %"PRIu64", \"sig_frames\": %"PRIu64", \"sig_sumsq\": %.17g, \"underruns\": %"PRIu64", \"erase_target_burst\": %d, \"signal_bursts_seen\": %"PRIu64", \"erased_chunks\": %"PRIu64", \"hard_clips\": %"PRIu64", \"clip_den\": %"PRIu64", \"over_fs_unscaled\": %"PRIu64", \"input_at_fs\": %"PRIu64", \"composite_scale\": %.17g, \"composite_scale_min\": %.17g},\n",ff,fs,fss,fu,fwd->erase_target_burst,fb,fe,fh,fd,fo,fi,fg,fgm);
    fprintf(f,"  \"rev\": {\"frames\": %"PRIu64", \"sig_frames\": %"PRIu64", \"sig_sumsq\": %.17g, \"underruns\": %"PRIu64", \"erase_target_burst\": %d, \"signal_bursts_seen\": %"PRIu64", \"erased_chunks\": %"PRIu64", \"hard_clips\": %"PRIu64", \"clip_den\": %"PRIu64", \"over_fs_unscaled\": %"PRIu64", \"input_at_fs\": %"PRIu64", \"composite_scale\": %.17g, \"composite_scale_min\": %.17g},\n",rf,rs,rss,ru,rev->erase_target_burst,rb,re,rh,rd,ro,ri,rg,rgm);
    fprintf(f,"  \"psig_mode\": ");json_string(f,o->reference_mode);fprintf(f,", \"psig_mode_source\": ");json_string(f,o->reference_mode_source);fprintf(f,",\n");
    if(fwd->channel.psig_mode==PSIG_GEOMETRY||fwd->channel.psig_mode==PSIG_BENCH){fprintf(f,"  \"geometry\": {\"fwd\": ");geo_json(f,&fwd->channel);fprintf(f,", \"rev\": ");geo_json(f,&rev->channel);fprintf(f,"},\n");
        fprintf(f,"  \"waveform_kinds\": {\"fwd\": ");kinds_json(f,&fwd->channel);fprintf(f,", \"rev\": ");kinds_json(f,&rev->channel);fprintf(f,"},\n");}
    fprintf(f,"  \"bench\": {\"ofdm_reference_power\": %.17g, \"ofdm_reference_source\": ",o->ofdm_ref);json_string(f,o->ofdm_ref_source);
    fprintf(f,", \"ofdm_reference_derivation\": ");json_string(f,BENCH_OFDM_REF_DERIVATION);
    fprintf(f,", \"fwd_noise_variance\": %.17g, \"rev_noise_variance\": %.17g, \"kind_kurtosis_threshold\": %.17g, \"kind_band_threshold_hz\": %.17g},\n",fnoise,rnoise,o->kind_kurt_thr,o->kind_bw_thr_hz);
    fprintf(f,"  \"channel_attestation\": {\"cell\": ");json_string(f,o->cell);fprintf(f,", \"profile\": ");
    char up[16];size_t i;for(i=0;i<sizeof(up)-1&&o->profile_name[i];i++)up[i]=(char)toupper((unsigned char)o->profile_name[i]);up[i]=0;json_string(f,up);
    fprintf(f,", \"commanded_snr\": %.17g, \"realized_snr3k\": %.17g, \"realized_snr_offset_db\": %.17g, \"seed\": %d, \"passthrough\": %s, \"realized_p_sig\": %.17g},\n",o->commanded_snr,o->snr,o->snr_offset,o->seed,o->passthrough?"true":"false",fs?fss/fs:0.0);
    fprintf(f,"  \"axis_attestation\": {\"axis_version\": ");json_string(f,o->axis);fprintf(f,", \"input_coordinate\": ");json_string(f,o->input_coordinate);
    fprintf(f,", \"reference_mode\": ");json_string(f,reference_mode_label(o->reference_mode));fprintf(f,", \"reference_id\": ");json_string(f,reference_id_label(o->reference_mode));
    fprintf(f,", \"reference_power\": %.17g, \"reference_n_samples\": 0",fref);
    fprintf(f,", \"psig_mode\": ");json_string(f,o->reference_mode);fprintf(f,", \"psig_mode_source\": ");json_string(f,o->reference_mode_source);
    fprintf(f,", \"psig_fix\": %.17g, \"floor_reference_power\": %.17g, \"class_tol_db\": %.17g, \"shift_db\": %.17g, \"reverse_reference_power\": %.17g, \"reverse_noise_variance\": %.17g",o->psig_fix,o->floor_ref,o->class_tol_db,o->shift_db,rref,rnoise);
    fprintf(f,", \"burst_log\": ");json_string(f,o->burst_log);
    fprintf(f,", \"configured_bandwidth_hz\": %.17g, \"snr3k_db\": %.17g, \"cn_config_db\": %.17g, \"noise_variance\": %.17g",o->configured_bandwidth_hz,o->snr,o->cn_config_db,fnoise);
    fprintf(f,", \"seed\": %d, \"composite_scale\": %.17g, \"pre_scale_peak\": %.17g, \"hard_clip_count\": %"PRIu64", \"s32_saturation_count\": %"PRIu64", \"clip_event_denominator\": %"PRIu64", \"over_fs_unscaled_count\": %"PRIu64", \"noise_lpf_hz\": %.17g, \"noise_lpf_taps\": %d, \"noise_power_ratio_vs_3k\": %.17g, \"headroom_rms\": %.17g, \"s32_mode\": ",o->seed,fmin(fgm,rgm),fmax(fp,rp),fh+rh,fx+rx,fd+rd,fo+ro,fwd->channel.nf_n?o->noise_lpf_hz:0.0,fwd->channel.nf_n,noise_power_ratio(&fwd->channel),o->headroom_rms);json_string(f,!strcmp(o->s32_mode,"auto")?"s32-hardclip":o->s32_mode);
    fprintf(f,", \"binary_sha256\": ");json_string(f,o->binary_sha256);fprintf(f,", \"recipe_sha256\": ");json_string(f,o->recipe_sha256);fprintf(f,", \"bridge_sha256\": ");json_string(f,o->bridge_sha256);fprintf(f,", \"harness_lineage\": ");json_string(f,o->harness_lineage);fprintf(f,", \"harness_sha256\": ");json_string(f,o->harness_sha256);fprintf(f,"}");
    if(g_schedule.loaded){fputc(',',f);schedule_emit_json(f);}
    fprintf(f,"\n}\n");
    fflush(f);fsync(fileno(f));if(fclose(f)||rename(tmp,o->statsfile)){unlink(tmp);return-1;}return 0;
}

static void signal_handler(int sig){(void)sig;stop_requested=1;}
static void defaults(options_t*o){
    memset(o,0,sizeof(*o));strcpy(o->fwd_cap,"hw:Loopback,1,0");strcpy(o->fwd_play,"hw:Loopback,0,1");strcpy(o->rev_cap,"hw:Loopback,1,2");strcpy(o->rev_play,"hw:Loopback,0,3");strcpy(o->profile_name,"wgn");o->profile=PROF_WGN;o->snr=30;o->sig_ref=.15;o->seed=1;o->cap_periods=4;o->play_periods=5;o->prime_periods=2;o->bp_taps=511;
    strcpy(o->axis,"v1-steady-snr3k");strcpy(o->input_coordinate,"default_snr3k_db");strcpy(o->s32_mode,"auto");memset(o->binary_sha256,'0',64);o->binary_sha256[64]=0;memset(o->recipe_sha256,'0',64);o->recipe_sha256[64]=0;memset(o->bridge_sha256,'0',64);o->bridge_sha256[64]=0;o->harness_lineage[0]=0;memset(o->harness_sha256,'0',64);o->harness_sha256[64]=0;o->configured_bandwidth_hz=2343.75;
    o->noise_lpf_hz=4000;o->noise_lpf_taps=255;o->headroom_rms=0.12;
    /* Mercury OFDM WB transmit power on this bridge (a fixed-reference cfg13
     * run held 0.0206 and the settled legacy median read 0.0207-0.0227). */
    o->floor_ref=0.0206;o->class_tol_db=2.0;o->shift_db=3.0;
    o->ofdm_ref=BENCH_OFDM_REF_DEFAULT;strcpy(o->ofdm_ref_source,"default");
    /* Kind thresholds (record only, see geo_kind()): kurtosis midway between
     * the highest non-OFDM-data waveform (2.35) and the lowest OFDM data
     * (2.87); band threshold at the geometric mean of the WB (2343.75 Hz) and
     * NB (one fifth of the WB carriers, 468.75 Hz) OFDM widths. */
    o->kind_kurt_thr=2.6;o->kind_bw_thr_hz=sqrt(2343.75*468.75);
    strcpy(o->reference_mode,"bench");strcpy(o->reference_mode_source,"default");
    /* Environment fallback only; the harness passes --reference-mode, because
     * a privileged launcher (sudo) does not carry the environment through. */
    const char*em=getenv("MERCURY_SIM_PSIG_MODE"),*ef=getenv("MERCURY_SIM_PSIG_FIX");
    if(em&&*em){snprintf(o->reference_mode,sizeof(o->reference_mode),"%s",em);strcpy(o->reference_mode_source,"env");}
    if(ef&&*ef){o->psig_fix=strtod(ef,NULL);o->psig_fix_set=true;}
    const char *bp=getenv("IRIS_AUDIO_BANDPASS");
    if(bp&&!strcmp(bp,"narrow")){o->bp_lo=300;o->bp_hi=2900;}
    else if(bp&&!strcmp(bp,"wide")){o->bp_lo=300;o->bp_hi=6300;}
}
static void usage(FILE*f){fprintf(f,"usage: realaudio_bridge_s32_c [Python-compatible bridge options]\n       channel: --noise-lpf-hz HZ (default 4000, 0=full band) --headroom-rms R (default 0.12, 0=off) --legacy-fullband\n       reference: --reference-mode bench|per-class|legacy-median|fix|peak (default bench)\n                  --ofdm-ref P (bench: OFDM WB data power the noise is set from) --psig-fix P (fix)\n                  --floor-ref P (per-class silence, default 0.0206) --kind-kurtosis-thr K --kind-band-thr-hz B\n                  --class-tol-db D --shift-db D --burst-log F (default <statsfile>.bursts.jsonl)\n                  --clip-log F (default <statsfile>.clips.jsonl; one line per chunk that clipped)\n       test modes: --vector-in F --vector-out F [--format-only|--passthrough]\n                   --double-in F --double-out F | --tap-out F --tap-count N\n");}
static int val(int argc,char**argv,int*i,const char**v){if(*i+1>=argc)return-1;*v=argv[++*i];return 0;}
static int parse_profile(options_t*o,const char*s){
    snprintf(o->profile_name,sizeof(o->profile_name),"%s",s);for(char*p=o->profile_name;*p;p++)*p=(char)tolower((unsigned char)*p);
    if(!strcmp(o->profile_name,"wgn"))o->profile=PROF_WGN;else if(!strcmp(o->profile_name,"flat"))o->profile=PROF_FLAT;else if(!strcmp(o->profile_name,"mpg"))o->profile=PROF_MPG;else if(!strcmp(o->profile_name,"mpm"))o->profile=PROF_MPM;else if(!strcmp(o->profile_name,"mpp"))o->profile=PROF_MPP;else if(!strcmp(o->profile_name,"mpd"))o->profile=PROF_MPD;else return-1;return 0;
}
static int parse_args(int argc,char**argv,options_t*o){
    defaults(o);for(int i=1;i<argc;i++){const char*a=argv[i],*v=NULL;
        if(!strcmp(a,"--help")||!strcmp(a,"-h")){usage(stdout);exit(0);}else if(!strcmp(a,"--passthrough"))o->passthrough=true;else if(!strcmp(a,"--burst"))o->burst=true;else if(!strcmp(a,"--dry-run"))o->dry_run=true;else if(!strcmp(a,"--self-test"))o->self_test=true;else if(!strcmp(a,"--format-only"))o->format_only=true;
        else if(!strcmp(a,"--legacy-fullband")){o->noise_lpf_hz=0;o->headroom_rms=0;o->noise_lpf_set=o->headroom_set=true;}
        else if(val(argc,argv,&i,&v)<0)return-1;
        else if(!strcmp(a,"--fwd-cap"))snprintf(o->fwd_cap,sizeof(o->fwd_cap),"%s",v);else if(!strcmp(a,"--fwd-play"))snprintf(o->fwd_play,sizeof(o->fwd_play),"%s",v);else if(!strcmp(a,"--rev-cap"))snprintf(o->rev_cap,sizeof(o->rev_cap),"%s",v);else if(!strcmp(a,"--rev-play"))snprintf(o->rev_play,sizeof(o->rev_play),"%s",v);
        else if(!strcmp(a,"--profile")){if(parse_profile(o,v))return-1;}else if(!strcmp(a,"--snr")){o->snr=strtod(v,NULL);strcpy(o->input_coordinate,"snr");}else if(!strcmp(a,"--snr3k")){o->snr=strtod(v,NULL);strcpy(o->input_coordinate,"snr3k");}else if(!strcmp(a,"--snr3k-db")){o->snr=strtod(v,NULL);strcpy(o->input_coordinate,"snr3k_db");}else if(!strcmp(a,"--cn-config-db")){o->cn_config_db=strtod(v,NULL);strcpy(o->input_coordinate,"cn_config_db");}else if(!strcmp(a,"--cell")){snprintf(o->cell,sizeof(o->cell),"%s",v);strcpy(o->input_coordinate,"cell");}else if(!strcmp(a,"--axis"))snprintf(o->axis,sizeof(o->axis),"%s",v);else if(!strcmp(a,"--configured-bandwidth-hz"))o->configured_bandwidth_hz=strtod(v,NULL);else if(!strcmp(a,"--binary-sha256"))snprintf(o->binary_sha256,sizeof(o->binary_sha256),"%s",v);else if(!strcmp(a,"--recipe-sha256"))snprintf(o->recipe_sha256,sizeof(o->recipe_sha256),"%s",v);else if(!strcmp(a,"--bridge-sha256"))snprintf(o->bridge_sha256,sizeof(o->bridge_sha256),"%s",v);else if(!strcmp(a,"--harness-lineage"))snprintf(o->harness_lineage,sizeof(o->harness_lineage),"%s",v);else if(!strcmp(a,"--harness-sha256"))snprintf(o->harness_sha256,sizeof(o->harness_sha256),"%s",v);else if(!strcmp(a,"--s32-mode"))snprintf(o->s32_mode,sizeof(o->s32_mode),"%s",v);else if(!strcmp(a,"--cfo-hz"))o->cfo_hz=strtod(v,NULL);else if(!strcmp(a,"--phase-noise-deg"))o->phase_noise_deg=strtod(v,NULL);else if(!strcmp(a,"--fade-depth-db"))o->fade_depth_db=strtod(v,NULL);else if(!strcmp(a,"--erase-a2b-burst"))o->erase_a2b_burst=(int)strtol(v,NULL,10);else if(!strcmp(a,"--erase-b2a-burst"))o->erase_b2a_burst=(int)strtol(v,NULL,10);else if(!strcmp(a,"--loss"))o->loss=strtod(v,NULL);else if(!strcmp(a,"--sig-ref"))o->sig_ref=strtod(v,NULL);else if(!strcmp(a,"--seed"))o->seed=(int)strtol(v,NULL,10);else if(!strcmp(a,"--cap-periods"))o->cap_periods=(int)strtol(v,NULL,10);else if(!strcmp(a,"--play-periods"))o->play_periods=(int)strtol(v,NULL,10);else if(!strcmp(a,"--prime-periods"))o->prime_periods=(int)strtol(v,NULL,10);else if(!strcmp(a,"--statsfile"))snprintf(o->statsfile,sizeof(o->statsfile),"%s",v);else if(!strcmp(a,"--snr-schedule"))snprintf(o->snr_schedule,sizeof(o->snr_schedule),"%s",v);
        else if(!strcmp(a,"--audio-bandpass")){if(!strcmp(v,"narrow")){o->bp_lo=300;o->bp_hi=2900;}else if(!strcmp(v,"wide")){o->bp_lo=300;o->bp_hi=6300;}else if(strcmp(v,"off"))return-1;}else if(!strcmp(a,"--bandpass-lo-hz"))o->bp_lo=strtod(v,NULL);else if(!strcmp(a,"--bandpass-hi-hz"))o->bp_hi=strtod(v,NULL);else if(!strcmp(a,"--bandpass-taps"))o->bp_taps=(int)strtol(v,NULL,10);
        else if(!strcmp(a,"--reference-mode")){snprintf(o->reference_mode,sizeof(o->reference_mode),"%s",v);strcpy(o->reference_mode_source,"cli");}
        else if(!strcmp(a,"--psig-fix")){o->psig_fix=strtod(v,NULL);o->psig_fix_set=true;}
        else if(!strcmp(a,"--ofdm-ref")){o->ofdm_ref=strtod(v,NULL);strcpy(o->ofdm_ref_source,"cli");}
        else if(!strcmp(a,"--kind-kurtosis-thr"))o->kind_kurt_thr=strtod(v,NULL);else if(!strcmp(a,"--kind-band-thr-hz"))o->kind_bw_thr_hz=strtod(v,NULL);
        else if(!strcmp(a,"--floor-ref"))o->floor_ref=strtod(v,NULL);else if(!strcmp(a,"--class-tol-db"))o->class_tol_db=strtod(v,NULL);else if(!strcmp(a,"--shift-db"))o->shift_db=strtod(v,NULL);
        else if(!strcmp(a,"--burst-log"))snprintf(o->burst_log,sizeof(o->burst_log),"%s",v);
        else if(!strcmp(a,"--clip-log"))snprintf(o->clip_log,sizeof(o->clip_log),"%s",v);
        else if(!strcmp(a,"--noise-lpf-hz")){o->noise_lpf_hz=strtod(v,NULL);o->noise_lpf_set=true;}else if(!strcmp(a,"--noise-lpf-taps"))o->noise_lpf_taps=(int)strtol(v,NULL,10);else if(!strcmp(a,"--headroom-rms")){o->headroom_rms=strtod(v,NULL);o->headroom_set=true;}
        else if(!strcmp(a,"--vector-in"))snprintf(o->vector_in,sizeof(o->vector_in),"%s",v);else if(!strcmp(a,"--vector-out"))snprintf(o->vector_out,sizeof(o->vector_out),"%s",v);else if(!strcmp(a,"--double-in"))snprintf(o->double_in,sizeof(o->double_in),"%s",v);else if(!strcmp(a,"--double-out"))snprintf(o->double_out,sizeof(o->double_out),"%s",v);else if(!strcmp(a,"--tap-out"))snprintf(o->tap_out,sizeof(o->tap_out),"%s",v);else if(!strcmp(a,"--tap-count"))o->tap_count=(size_t)strtoull(v,NULL,10);else if(!strcmp(a,"--tap-stride"))o->tap_stride=(size_t)strtoull(v,NULL,10);else return-1;
    }
    if(strcmp(o->axis,"v1-steady-snr3k")||!strcmp(o->s32_mode,"prescaled")){
        fprintf(stderr,"native bridge currently requires --axis v1-steady-snr3k and a hard-clip S32 mode\n");return-1;
    }
    if(o->configured_bandwidth_hz<=0)return-1;
    /* Reference mode names.  "steady" (the name every campaign environment
     * used for "the default reference") selects the default, bench.  The
     * per-waveform-class reference is per-class (old name geometry); the old
     * running median of all active chunks is kept, byte-for-byte, as
     * legacy-median. */
    for(char*p=o->reference_mode;*p;p++)*p=(char)tolower((unsigned char)*p);
    if(!strcmp(o->reference_mode,"steady")||!strcmp(o->reference_mode,"bench"))strcpy(o->reference_mode,"bench");
    else if(!strcmp(o->reference_mode,"per-class")||!strcmp(o->reference_mode,"geometry"))strcpy(o->reference_mode,"geometry");
    else if(!strcmp(o->reference_mode,"legacy-median")||!strcmp(o->reference_mode,"legacy"))strcpy(o->reference_mode,"legacy-median");
    else if(strcmp(o->reference_mode,"fix")&&strcmp(o->reference_mode,"peak")){fprintf(stderr,"unknown --reference-mode %s\n",o->reference_mode);return-1;}
    if(!strcmp(o->reference_mode,"fix")&&!(o->psig_fix>0)){fprintf(stderr,"--reference-mode fix needs --psig-fix > 0\n");return-1;}
    if(!(o->floor_ref>0)||!(o->class_tol_db>0)||!(o->shift_db>0)||!(o->ofdm_ref>0)||!(o->kind_kurt_thr>0)||!(o->kind_bw_thr_hz>0))return-1;
    /* A configured audio bandpass (the FM-modem path) keeps the legacy noise
     * model unless the receiver-passband options are given explicitly. */
    if(o->bp_hi>o->bp_lo&&o->bp_lo>0){if(!o->noise_lpf_set)o->noise_lpf_hz=0;if(!o->headroom_set)o->headroom_rms=0;}
    if(o->noise_lpf_hz<0||o->noise_lpf_hz>=RATE/2.0||o->headroom_rms<0)return-1;
    if(!strcmp(o->input_coordinate,"cn_config_db"))o->snr=o->cn_config_db-10.0*log10(3000.0/o->configured_bandwidth_hz);
    o->commanded_snr=o->snr;o->snr_offset=0;
    if(o->cell[0]){char*colon=strchr(o->cell,':');if(!colon||strncasecmp(o->cell,"WGN:",4))return-1;o->commanded_snr=strtod(colon+1,NULL);o->snr=ionos_wgn_to_snr3k(o->commanded_snr);o->snr_offset=o->snr-o->commanded_snr;}
    else {
        char up[16]; size_t k;
        for(k=0;k<sizeof(up)-1&&o->profile_name[k];k++) up[k]=(char)toupper((unsigned char)o->profile_name[k]);
        up[k]=0; snprintf(o->cell,sizeof(o->cell),"%s:%g",up,o->snr);
    }
    o->cn_config_db=o->snr+10.0*log10(3000.0/o->configured_bandwidth_hz);
    if(o->snr_schedule[0]&&schedule_load(o->snr_schedule))return-1;
    return 0;
}

static int vector_mode(const options_t*o){
    FILE*fi=fopen(o->vector_in,"rb"),*fo=fopen(o->vector_out,"wb");if(!fi||!fo){perror("vector file");return 2;}channel_t c;if(channel_init(&c,o,(uint32_t)(o->seed*UINT32_C(2654435761))))return 2;strcpy(c.name,"vec");
    c.sched_offline=true;strcpy(c.sched_tag,"vec");
    if((c.psig_mode==PSIG_GEOMETRY||c.psig_mode==PSIG_BENCH)&&o->burst_log[0]&&strcmp(o->burst_log,"-")){g_burst_log=fopen(o->burst_log,"w");if(!g_burst_log){perror("burst log");return 2;}}
    if(o->clip_log[0]&&strcmp(o->clip_log,"-")){g_clip_log=fopen(o->clip_log,"w");if(!g_clip_log){perror("clip log");return 2;}}
    int32_t ib[PERIOD*2],ob[PERIOD*2];double x[PERIOD],y[PERIOD],vpk=0;size_t words;uint64_t vhard=0,vover=0,vden=0;
    while((words=fread(ib,sizeof(int32_t),PERIOD*2,fi))){size_t n=words/2;for(size_t i=0;i<n;i++)x[i]=(double)ib[2*i]/INT_MAX_D;
        if(o->passthrough){for(size_t i=0;i<n;i++)ob[2*i]=ob[2*i+1]=ib[2*i];}
        else {const uint64_t h0=vhard,o0=vover,cs=c.sample_clock;if(o->format_only)memcpy(y,x,n*sizeof(double));else channel_process(&c,x,y,n);emit_s32(&c,y,ob,n,&vpk,&vhard,&vover);vden+=n;
            clip_event("vec",cs,n,0.0,vhard-h0,vover-o0,0,c.composite_scale);}
        if(fwrite(ob,sizeof(int32_t),n*2,fo)!=n*2){perror("write");return 2;}if(words%2)break;
    }
    fprintf(stderr,"[bridge_s32_c] vector clip hard=%"PRIu64" over_fs_unscaled=%"PRIu64" den=%"PRIu64" composite_scale=%.9g noise_lpf_hz=%g pre_scale_peak=%.6g\n",vhard,vover,vden,c.composite_scale,c.nf_n?o->noise_lpf_hz:0.0,vpk);
    if(c.psig_mode==PSIG_GEOMETRY||c.psig_mode==PSIG_BENCH){if(c.in_burst)geo_finalize(&c,false);fprintf(stderr,"[bridge_s32_c] vector geometry reference_mode=%s transmissions=%"PRIu64" classes=%d worst_snr3k_error_db=%.4f\n",o->reference_mode,c.bursts_done,c.ncls,c.worst_snr_err_db);}
    else fprintf(stderr,"[bridge_s32_c] vector reference_mode=%s reference_power=%.9g\n",o->reference_mode,c.p_sig);
    if(g_burst_log){fclose(g_burst_log);g_burst_log=NULL;}
    if(g_clip_log){fclose(g_clip_log);g_clip_log=NULL;}
    schedule_finalize();write_schedule_statsfile(o);channel_free(&c);fclose(fi);fclose(fo);return 0;
}
static int double_mode(const options_t*o){
    FILE*fi=fopen(o->double_in,"rb"),*fo=fopen(o->double_out,"wb");if(!fi||!fo){perror("double file");return 2;}channel_t c;if(channel_init(&c,o,(uint32_t)(o->seed*UINT32_C(2654435761))))return 2;c.sched_offline=true;strcpy(c.sched_tag,"dbl");double x[PERIOD],y[PERIOD];size_t n;while((n=fread(x,sizeof(double),PERIOD,fi))){channel_process(&c,x,y,n);if(fwrite(y,sizeof(double),n,fo)!=n)return 2;}schedule_finalize();write_schedule_statsfile(o);channel_free(&c);fclose(fi);fclose(fo);return 0;
}
static int tap_mode(const options_t*o){
    double dt,fd;profile_params(o->profile,&dt,&fd);if(fd<=0){fprintf(stderr,"tap mode needs fading profile\n");return 2;}FILE*f=fopen(o->tap_out,"wb");if(!f){perror("tap-out");return 2;}doppler_t d;doppler_init(&d,fd,(uint64_t)o->seed);size_t stride=o->tap_stride?o->tap_stride:d.update;for(size_t i=0;i<o->tap_count;i++){double pair[2]={creal(d.hold),cimag(d.hold)};if(fwrite(pair,sizeof(double),2,f)!=2){fclose(f);return 2;}doppler_skip(&d,stride);}fclose(f);return 0;
}
static void dry_stats(pump_t*p,const options_t*o,uint32_t seed){
    channel_init(&p->channel,o,seed);double x[PERIOD],y[PERIOD],ss=0;for(int k=0;k<8;k++){for(int i=0;i<PERIOD;i++){x[i]=o->sig_ref*sqrt(2.0)*sin(2*PI*1500*i/RATE);ss+=x[i]*x[i];}maybe_erase_burst(p,x,PERIOD,true);if(!o->passthrough)channel_process(&p->channel,x,y,PERIOD);p->stats.frames+=PERIOD;p->stats.sig_frames+=PERIOD;}p->stats.sig_sumsq=ss;p->stats.reference_power=p->channel.p_sig;p->stats.noise_variance=p->channel.noise_std*p->channel.noise_std;p->stats.scale=p->channel.composite_scale;p->stats.scale_min=p->channel.scale_min;
}
int main(int argc,char**argv){
    options_t o;if(parse_args(argc,argv,&o)){usage(stderr);return 2;}
    if(o.vector_in[0]||o.vector_out[0])return(o.vector_in[0]&&o.vector_out[0])?vector_mode(&o):2;
    if(o.double_in[0]||o.double_out[0])return(o.double_in[0]&&o.double_out[0])?double_mode(&o):2;
    if(o.tap_out[0])return tap_mode(&o);
    pump_t fwd={.opt=&o,.erase_target_burst=o.erase_a2b_burst,.erase_prev_silent=true},rev={.opt=&o,.erase_target_burst=o.erase_b2a_burst,.erase_prev_silent=true};strcpy(fwd.name,"fwd");strcpy(rev.name,"rev");strcpy(fwd.cap_dev,o.fwd_cap);strcpy(fwd.play_dev,o.fwd_play);strcpy(rev.cap_dev,o.rev_cap);strcpy(rev.play_dev,o.rev_play);pthread_mutex_init(&fwd.stats.lock,NULL);pthread_mutex_init(&rev.stats.lock,NULL);pthread_mutex_init(&fwd.pcm_lock,NULL);pthread_mutex_init(&rev.pcm_lock,NULL);
    if(o.dry_run||o.self_test){dry_stats(&fwd,&o,(uint32_t)(o.seed*UINT32_C(2654435761)));dry_stats(&rev,&o,(uint32_t)(o.seed*UINT32_C(40503)+7));if(!o.statsfile[0])snprintf(o.statsfile,sizeof(o.statsfile),"/tmp/bridge_c_dry_%ld.json",(long)getpid());int e=flush_stats(&o,&fwd,&rev);fprintf(stderr,"[bridge_s32_c] %s wrote %s\n",e?"DRY-RUN FAIL":"DRY-RUN PASS",o.statsfile);channel_free(&fwd.channel);channel_free(&rev.channel);return e?1:0;}
    if(channel_init(&fwd.channel,&o,(uint32_t)(o.seed*UINT32_C(2654435761)))||channel_init(&rev.channel,&o,(uint32_t)(o.seed*UINT32_C(40503)+7))){fprintf(stderr,"channel init failed\n");return 2;}
    strcpy(fwd.channel.sched_tag,"fwd");strcpy(rev.channel.sched_tag,"rev");
    strcpy(fwd.channel.name,"fwd");strcpy(rev.channel.name,"rev");
    if(!o.burst_log[0]&&o.statsfile[0])snprintf(o.burst_log,sizeof(o.burst_log),"%s.bursts.jsonl",o.statsfile);
    if(!o.clip_log[0]&&o.statsfile[0])snprintf(o.clip_log,sizeof(o.clip_log),"%s.clips.jsonl",o.statsfile);
    if(o.clip_log[0]&&strcmp(o.clip_log,"-")){g_clip_log=fopen(o.clip_log,"w");if(!g_clip_log){fprintf(stderr,"[bridge_s32_c] clip log %s: %s\n",o.clip_log,strerror(errno));return 2;}}
    if((fwd.channel.psig_mode==PSIG_GEOMETRY||fwd.channel.psig_mode==PSIG_BENCH)&&o.burst_log[0]&&strcmp(o.burst_log,"-")){g_burst_log=fopen(o.burst_log,"w");if(!g_burst_log){fprintf(stderr,"[bridge_s32_c] burst log %s: %s\n",o.burst_log,strerror(errno));return 2;}}
    fwd.stats.scale=fwd.stats.scale_min=fwd.channel.composite_scale;rev.stats.scale=rev.stats.scale_min=rev.channel.composite_scale;
    fprintf(stderr,"[bridge_s32_c] reference_mode=%s source=%s psig_fix=%.9g floor_ref=%.9g ofdm_ref=%.9g ofdm_ref_source=%s fwd_noise_var=%.9g rev_noise_var=%.9g\n",o.reference_mode,o.reference_mode_source,o.psig_fix,o.floor_ref,o.ofdm_ref,o.ofdm_ref_source,fwd.channel.noise_std*fwd.channel.noise_std,rev.channel.noise_std*rev.channel.noise_std);
    fprintf(stderr,"[bridge_s32_c] %s SNR3k=%.3f noise_lpf_hz=%g composite_scale=%.6f profile=%s seed=%d rings cap=%d play=%d prime=%d cables fwd[%s->%s] rev[%s->%s]\n",o.passthrough?"PASSTHROUGH":"CHANNEL",o.snr,fwd.channel.nf_n?o.noise_lpf_hz:0.0,fwd.channel.composite_scale,o.profile_name,o.seed,o.cap_periods,o.play_periods,o.prime_periods,o.fwd_cap,o.fwd_play,o.rev_cap,o.rev_play);
    struct sigaction sa={0};sa.sa_handler=signal_handler;sigaction(SIGINT,&sa,NULL);sigaction(SIGTERM,&sa,NULL);
    pthread_t tf,tr;if(pthread_create(&tf,NULL,pump_main,&fwd)||pthread_create(&tr,NULL,pump_main,&rev)){fprintf(stderr,"pthread_create failed\n");return 2;}
    while(!stop_requested){struct timespec ts={.tv_sec=0,.tv_nsec=500000000};nanosleep(&ts,NULL);flush_stats(&o,&fwd,&rev);}
    pthread_mutex_lock(&fwd.pcm_lock);if(fwd.cap_shared)snd_pcm_drop(fwd.cap_shared);if(fwd.play_shared)snd_pcm_drop(fwd.play_shared);pthread_mutex_unlock(&fwd.pcm_lock);
    pthread_mutex_lock(&rev.pcm_lock);if(rev.cap_shared)snd_pcm_drop(rev.cap_shared);if(rev.play_shared)snd_pcm_drop(rev.play_shared);pthread_mutex_unlock(&rev.pcm_lock);
    pthread_join(tf,NULL);pthread_join(tr,NULL);schedule_finalize();flush_stats(&o,&fwd,&rev);if(g_burst_log){fclose(g_burst_log);g_burst_log=NULL;}if(g_clip_log){fclose(g_clip_log);g_clip_log=NULL;}channel_free(&fwd.channel);channel_free(&rev.channel);return 0;
}
