#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <vector>

// Test-only access permits randomized coefficient corpora without changing the
// production cl_FIR interface.
#define private public
#include "physical_layer/fir_filter.h"
#undef private

#if defined(__GNUC__) || defined(__clang__)
extern "C" void mercury_fir_test_reset_dispatch_counts() __attribute__((weak));
extern "C" std::uint64_t mercury_fir_test_complex_avx2_dispatches() __attribute__((weak));
extern "C" std::uint64_t mercury_fir_test_decimate_complex_avx2_dispatches() __attribute__((weak));
#endif

static std::uint64_t rng_state=UINT64_C(0x80a62be1d43fc759);

static std::uint64_t rng()
{
	rng_state^=rng_state<<13;
	rng_state^=rng_state>>7;
	rng_state^=rng_state<<17;
	return rng_state;
}

static double from_bits(std::uint64_t bits)
{
	double value;
	std::memcpy(&value,&bits,sizeof(value));
	return value;
}

static void emit(const void* data, std::size_t size)
{
	if(std::fwrite(data,1,size,stdout)!=size) std::abort();
}

static void fill_input(std::complex<double>* in, int n, std::uint64_t salt)
{
	for(int i=0;i<n;i++)
	{
		const std::uint64_t a=rng()^salt^(std::uint64_t)i;
		const std::uint64_t b=rng()+(salt<<1)+(std::uint64_t)i;
		const double real=std::ldexp((double)(std::int64_t)a,-63);
		const double imag=std::ldexp((double)(std::int64_t)b,-63);
		in[i]=std::complex<double>(real,imag);
	}
	if(n>8)
	{
		in[0]=std::complex<double>(0.0,-0.0);
		in[1]=std::complex<double>(from_bits(1),-from_bits(1));
		in[2]=std::complex<double>(std::ldexp(1.9999999999999998,500),
			-std::ldexp(1.0,-500));
		in[3]=std::complex<double>(-std::ldexp(1.0,-900),std::ldexp(1.0,300));
	}
}

static void set_random_taps(cl_FIR& fir, int n_taps, std::uint64_t salt)
{
	fir.deinit();
	fir.filter_nTaps=n_taps;
	fir.filter_coefficients=new double[(std::size_t)n_taps];
	for(int j=0;j<n_taps;j++)
	{
		const std::uint64_t r=rng()^salt^(std::uint64_t)j;
		const double sign=(r&1)?-1:1;
		const double mantissa=1.0+
			std::ldexp((double)((r>>12)&UINT64_C(0xfffff)),-20);
		const int exponent=(int)((r>>40)%31)-20;
		fir.filter_coefficients[j]=sign*std::ldexp(mantissa,exponent);
	}
}

struct case_header
{
	std::uint32_t tag;
	std::uint32_t n;
	std::uint32_t n_taps;
	std::uint32_t decimation;
	std::uint32_t phase;
	std::uint32_t alignment;
};

static void emit_case(cl_FIR& fir, int n, int decimation, int phase,
	int alignment, std::uint64_t salt)
{
	const int out_size=n/decimation;
	std::vector<std::complex<double> > input_storage((std::size_t)n+8);
	std::vector<std::complex<double> > full_storage((std::size_t)n+8);
	std::vector<std::complex<double> > decimated_storage((std::size_t)out_size+8);
	std::complex<double>* input=input_storage.data()+alignment;
	std::complex<double>* full=full_storage.data()+((alignment+1)&3);
	std::complex<double>* decimated=decimated_storage.data()+((alignment+2)&3);
	fill_input(input,n,salt);
	fir.apply(input,full,n);
	fir.apply_decimate(input,decimated,n,decimation);

	for(int m=0;m<out_size;m++)
	{
		if(std::memcmp(decimated+m,full+(std::size_t)m*decimation,
			sizeof(*decimated))!=0)
		{
			std::fprintf(stderr,
				"decimation authority mismatch n=%d taps=%d M=%d phase=%d align=%d m=%d\n",
				n,fir.filter_nTaps,decimation,phase,alignment,m);
			std::abort();
		}
	}

	const case_header header={UINT32_C(0x46495243),(std::uint32_t)n,
		(std::uint32_t)fir.filter_nTaps,(std::uint32_t)decimation,
		(std::uint32_t)phase,(std::uint32_t)alignment};
	emit(&header,sizeof(header));
	emit(full,(std::size_t)n*sizeof(*full));
	emit(decimated,(std::size_t)out_size*sizeof(*decimated));
}

int main(int argc, char** argv)
{
	const bool require_dispatch=argc==2 && std::strcmp(argv[1],"--require-avx2")==0;
#if defined(__GNUC__) || defined(__clang__)
	if(mercury_fir_test_reset_dispatch_counts)
		mercury_fir_test_reset_dispatch_counts();
#endif

	const int tap_counts[]={1,3,5,7,9,15,17,31,33,63,65,97,127};
	const int lengths[]={1,2,3,4,5,7,15,16,17,31,32,33,47,63,64,65,
		96,97,98,127,128,129,257,1024,4096};
	const int decimations[]={1,2,3,4,5,7,8};
	std::uint64_t case_id=0;

	// Deterministic randomized taps, short/boundary lengths, all vector tails,
	// and four complex-address alignment residues.
	for(int n_taps:tap_counts)
	{
		cl_FIR fir;
		set_random_taps(fir,n_taps,case_id++);
		for(int n:lengths)
		{
			if(n<n_taps) continue;
			for(int alignment=0;alignment<4;alignment++)
				for(int decimation:decimations)
					emit_case(fir,n,decimation,
						((n_taps-1)/2)%decimation,alignment,case_id++);
		}
	}

	// For each decimator, explicitly cover every FIR-window phase half % M.
	for(int decimation=2;decimation<=8;decimation++)
	{
		for(int phase=0;phase<decimation;phase++)
		{
			cl_FIR fir;
			const int half=decimation*2+phase;
			set_random_taps(fir,2*half+1,case_id++);
			emit_case(fir,521+phase,decimation,phase,phase&3,case_id++);
		}
	}

	// A second seeded corpus varies taps, sizes, decimation and alignment jointly.
	for(int corpus_case=0;corpus_case<128;corpus_case++)
	{
		cl_FIR fir;
		const int n_taps=1+2*(int)(rng()%65);
		const int n=n_taps+32+(int)(rng()%2017);
		const int decimation=1+(int)(rng()%8);
		const int phase=((n_taps-1)/2)%decimation;
		const int alignment=(int)(rng()&3);
		set_random_taps(fir,n_taps,case_id++);
		emit_case(fir,n,decimation,phase,alignment,case_id++);
	}

	if(require_dispatch)
	{
#if defined(__GNUC__) || defined(__clang__)
		if(!mercury_fir_test_complex_avx2_dispatches ||
			!mercury_fir_test_decimate_complex_avx2_dispatches)
		{
			std::fprintf(stderr,"complex AVX2 dispatch instrumentation is absent\n");
			return 86;
		}
		const std::uint64_t full=mercury_fir_test_complex_avx2_dispatches();
		const std::uint64_t decimated=mercury_fir_test_decimate_complex_avx2_dispatches();
		if(full==0 || decimated==0)
		{
			std::fprintf(stderr,"complex AVX2 dispatch was not exercised: full=%llu decimated=%llu\n",
				(unsigned long long)full,(unsigned long long)decimated);
			return 87;
		}
#else
		return 86;
#endif
	}
	return 0;
}
