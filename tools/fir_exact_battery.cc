#include "physical_layer/fir_filter.h"
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

static double bits(std::uint64_t u) { double x; std::memcpy(&x,&u,sizeof(x)); return x; }
static void emit(const void* p,std::size_t n) { if(std::fwrite(p,1,n,stdout)!=n) std::abort(); }
static std::uint64_t rng_state=UINT64_C(0x7a7c5e3d91b2468f);
static std::uint64_t rng() { rng_state^=rng_state<<13; rng_state^=rng_state>>7; rng_state^=rng_state<<17; return rng_state; }

int main()
{
    const double transitions[]={6000.0,3000.0,1000.0}; // 17, 33, 97 taps
    const int lengths[]={1,2,3,4,5,7,15,31,32,33,47,63,64,65,96,97,98,257,4096};
    for(double transition:transitions) {
        cl_FIR fir;
        fir.sampling_frequency=48000.0; fir.filter_transition_bandwidth=transition;
        fir.lpf_filter_cut_frequency=1800.0; fir.type=LPF; fir.filter_window=HAMMING; fir.design();
        for(int n:lengths) for(int align=0;align<4;align++) {
            std::vector<double> ib((std::size_t)n+16),ob((std::size_t)n+16,bits(UINT64_C(0x7ff8000000001234)));
            double* in=ib.data()+align; double* out=ob.data()+((align+1)&3);
            for(int i=0;i<n;i++) in[i]=std::sin(.017*i)+std::cos(.0031*i)*.125+((i%19)-9)*.001;
            if(n>8) { in[1]=0.0; in[2]=-0.0; in[3]=bits(1); in[4]=-bits(1); in[5]=INFINITY; in[6]=-INFINITY; in[7]=bits(UINT64_C(0x7ff8000000004567)); }
            fir.apply(in,out,n); emit(out,(std::size_t)n*sizeof(double));

            std::vector<double> same(in,in+n); fir.apply(same.data(),same.data(),n); emit(same.data(),same.size()*sizeof(double));
            std::vector<double> partial((std::size_t)n+2); std::memcpy(partial.data(),in,(std::size_t)n*sizeof(double));
            fir.apply(partial.data(),partial.data()+1,n); emit(partial.data()+1,(std::size_t)n*sizeof(double));

            std::vector<std::complex<double>> ci((std::size_t)n),co((std::size_t)n);
            for(int i=0;i<n;i++) ci[i]={in[i],i&1?-in[i]:in[i]*.5};
            if(n>=fir.filter_nTaps) {
                fir.apply(ci.data(),co.data(),n); emit(co.data(),co.size()*sizeof(co[0]));
            }
            for(int M:{1,2,3,8}) if(n>=fir.filter_nTaps && n/M>0) {
                std::vector<std::complex<double>> dec((std::size_t)n/M);
                fir.apply_decimate(ci.data(),dec.data(),n,M); emit(dec.data(),dec.size()*sizeof(dec[0]));
            }
        }
        // Reproducibly randomized tails, alignments, values, specials, and both
        // overlap directions. This supplements (and contains) the canonical
        // 18-shape battery above; the seed is part of the exact probe identity.
        for(int trial=0;trial<128;trial++) {
            int n;
            switch(trial%8) {
            case 0: n=fir.filter_nTaps-1; break; case 1: n=fir.filter_nTaps; break;
            case 2: n=fir.filter_nTaps+1; break; case 3: n=4*fir.filter_nTaps-1; break;
            default: n=1+(int)(rng()%2048); break;
            }
            const int align=(int)(rng()%8);
            std::vector<double> storage((std::size_t)n+32),outbuf((std::size_t)n+32);
            double* in=storage.data()+8+align; double* out=outbuf.data()+((align+3)&7);
            for(int i=0;i<n;i++) in[i]=((double)(rng()>>11)*(1.0/9007199254740992.0))*2.0-1.0;
            if(n>8 && trial%5==0) {
                in[1]=0.0; in[2]=-0.0; in[3]=bits(1); in[4]=-bits(1);
                in[5]=INFINITY; in[6]=-INFINITY; in[7]=bits(UINT64_C(0x7ff8000000004567));
            }
            fir.apply(in,out,n); emit(out,(std::size_t)n*sizeof(double));
            std::vector<double> forward(in,in+n); forward.resize((std::size_t)n+1);
            fir.apply(forward.data(),forward.data()+1,n); emit(forward.data()+1,(std::size_t)n*sizeof(double));
            std::vector<double> reverse((std::size_t)n+1); std::memcpy(reverse.data()+1,in,(std::size_t)n*sizeof(double));
            fir.apply(reverse.data()+1,reverse.data(),n); emit(reverse.data(),(std::size_t)n*sizeof(double));
        }
    }
    return 0;
}
