#include "physical_layer/fir_filter.h"
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

int main(int argc, char** argv)
{
	if(argc!=5 || (std::strcmp(argv[1],"apply")!=0 &&
		std::strcmp(argv[1],"decimate")!=0))
	{
		std::fprintf(stderr,"usage: %s apply|decimate input-size rounds M\n",argv[0]);
		return 2;
	}
	const bool decimate=std::strcmp(argv[1],"decimate")==0;
	const int n=std::atoi(argv[2]);
	const int rounds=std::atoi(argv[3]);
	const int M=std::atoi(argv[4]);
	if(n<256 || rounds<1 || M<1) return 2;

	cl_FIR fir;
	fir.sampling_frequency=48000.0;
	fir.filter_transition_bandwidth=1000.0;
	fir.lpf_filter_cut_frequency=1800.0;
	fir.type=LPF;
	fir.filter_window=HAMMING;
	fir.design();
	const int out_size=decimate?n/M:n;
	std::vector<std::complex<double> > input((std::size_t)n);
	std::vector<std::complex<double> > output((std::size_t)out_size);
	for(int i=0;i<n;i++)
		input[(std::size_t)i]=std::complex<double>(
			std::sin(0.017*i)+0.125*std::cos(0.0031*i),
			std::cos(0.011*i)-0.0625*std::sin(0.0019*i));

	if(decimate) fir.apply_decimate(input.data(),output.data(),n,M);
	else fir.apply(input.data(),output.data(),n);
	const auto start=std::chrono::steady_clock::now();
	for(int round=0;round<rounds;round++)
	{
		if(decimate) fir.apply_decimate(input.data(),output.data(),n,M);
		else fir.apply(input.data(),output.data(),n);
	}
	const auto stop=std::chrono::steady_clock::now();
	double checksum=0.0;
	for(const std::complex<double>& value:output)
		checksum+=value.real()*0.75+value.imag()*0.25;
	const double seconds=std::chrono::duration<double>(stop-start).count();
	const double ns_per_output=seconds*1e9/((double)out_size*rounds);
	std::printf("mode=%s n=%d taps=%d M=%d rounds=%d seconds=%.9f ns_per_output=%.3f checksum=%.17g\n",
		decimate?"decimate":"apply",n,fir.filter_nTaps,M,rounds,seconds,
		ns_per_output,checksum);
	return 0;
}
