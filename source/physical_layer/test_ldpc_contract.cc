#include "physical_layer/ldpc.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <type_traits>
#include <vector>

namespace {

void configure_valid(cl_ldpc* codec) {
	codec->standard = MERCURY;
	codec->framesize = MERCURY_NORMAL;
	codec->rate = 1.0f / 16.0f;
	codec->decoding_algorithm = SPA;
	codec->GBF_eta = 0.5f;
	codec->nIteration_max = 20;
	codec->print_nIteration = NO;
}

template <typename Codec>
int init_status(Codec* codec, std::true_type) {
	return codec->init();
}

template <typename Codec>
int init_status(Codec* codec, std::false_type) {
	codec->init();
	return 0;
}

int init_status(cl_ldpc* codec) {
	return init_status(codec,
		std::is_same<decltype(codec->init()), int>());
}

bool all_zero(const std::vector<int>& values) {
	for(std::size_t i = 0; i < values.size(); ++i)
		if(values[i] != 0) return false;
	return true;
}

bool all_zero(const std::vector<double>& values) {
	for(std::size_t i = 0; i < values.size(); ++i)
		if(values[i] != 0.0) return false;
	return true;
}

uint64_t hash_words(const std::vector<int>& values, uint64_t hash) {
	for(std::size_t i = 0; i < values.size(); ++i) {
		uint32_t word = (uint32_t)values[i];
		for(int byte = 0; byte < 4; ++byte) {
			hash ^= (uint8_t)(word >> (8 * byte));
			hash *= UINT64_C(1099511628211);
		}
	}
	return hash;
}

// Run the exact valid-path witness used on both sides of the hardening commit.
// init_status() is deliberately compatible with both the former void API and
// the hardened int API. On the former API an invalid init never returns because
// production calls exit(1); that is the runtime fail-before this test records.
bool valid_path(std::vector<int>* encoded_out, std::vector<int>* decoded_out,
	int* iterations_out) {
	cl_ldpc codec;
	configure_valid(&codec);
	if(init_status(&codec) != 0 || codec.N != MERCURY_NORMAL
		|| codec.K != 100 || codec.P != 1500)
		return false;

	std::vector<int> input((std::size_t)codec.K, 0);
	uint32_t state = UINT32_C(0x6d657263);
	for(int i = 0; i < codec.K; ++i) {
		state ^= state << 13;
		state ^= state >> 17;
		state ^= state << 5;
		input[(std::size_t)i] = (int)(state & 1u);
	}

	std::vector<int> encoded((std::size_t)codec.N, -1);
	codec.encode(input.data(), encoded.data());
	std::vector<float> llr((std::size_t)codec.N, 0.0f);
	for(int i = 0; i < codec.N; ++i)
		llr[(std::size_t)i] = encoded[(std::size_t)i] ? -20.0f : 20.0f;
	std::vector<int> decoded((std::size_t)codec.K, -1);
	const int iterations = codec.decode(llr.data(), decoded.data());
	if(decoded != input || iterations < 0 || iterations > codec.nIteration_max)
		return false;

	*encoded_out = encoded;
	*decoded_out = decoded;
	*iterations_out = iterations;
	return true;
}

#ifndef LDPC_CONTRACT_VALID_ONLY

int failures = 0;

#define CHECK(expr, label) do {                                                \
	if(!(expr)) { std::printf("FAIL: %s\n", label); ++failures; }               \
	else std::printf("PASS: %s\n", label);                                     \
} while(0)

void invalid_init_case(const char* label, int standard, int framesize,
	float rate) {
	cl_ldpc codec;
	configure_valid(&codec);
	codec.standard = standard;
	codec.framesize = framesize;
	codec.rate = rate;
	codec.nIteration_max = 9;
	const int status = init_status(&codec);
	CHECK(status == -1, label);

	const int safe_k = codec.K > 0 ? codec.K : 1;
	const int safe_n = codec.N > 0 ? codec.N : 1;
	std::vector<float> llr((std::size_t)safe_n, 1.0f);
	std::vector<int> decoded((std::size_t)safe_k, 0x5a5a5a5a);
	std::vector<double> app_llr((std::size_t)safe_n, 3.25);
	const int iterations = codec.decode(llr.data(), decoded.data(), app_llr.data());
	CHECK(iterations == codec.nIteration_max + 1 && all_zero(decoded)
		&& all_zero(app_llr), "invalid/unset LDPC state decodes fail closed");

	std::vector<int> encoded((std::size_t)safe_n, 0x5a5a5a5a);
	codec.encode(NULL, encoded.data());
	CHECK(all_zero(encoded), "invalid/unset LDPC state encodes zero output");
}

void null_decode_case() {
	cl_ldpc codec;
	configure_valid(&codec);
	CHECK(init_status(&codec) == 0, "valid LDPC init succeeds");
	std::vector<int> decoded((std::size_t)codec.K, 0x5a5a5a5a);
	std::vector<double> app_llr((std::size_t)codec.N, 3.25);
	const int iterations = codec.decode(NULL, decoded.data(), app_llr.data());
	CHECK(iterations == codec.nIteration_max + 1 && all_zero(decoded)
		&& all_zero(app_llr), "NULL LDPC decode input returns sentinel and zeroes outputs");

	std::vector<int> encoded((std::size_t)codec.N, 0x5a5a5a5a);
	codec.encode(NULL, encoded.data());
	CHECK(all_zero(encoded), "NULL LDPC encode input zeroes output");
}

#endif

}  // namespace

int main() {
	std::vector<int> encoded;
	std::vector<int> decoded;
	int iterations = -1;
	if(!valid_path(&encoded, &decoded, &iterations)) return 1;

#ifdef LDPC_CONTRACT_VALID_ONLY
	// Binary, complete witness for byte-exact pre/post cmp. No digest-only claim.
	if(std::fwrite(encoded.data(), sizeof(encoded[0]), encoded.size(), stdout)
		!= encoded.size()) return 2;
	if(std::fwrite(decoded.data(), sizeof(decoded[0]), decoded.size(), stdout)
		!= decoded.size()) return 2;
	if(std::fwrite(&iterations, sizeof(iterations), 1, stdout) != 1) return 2;
	return 0;
#else
	uint64_t hash = hash_words(encoded, UINT64_C(1469598103934665603));
	hash = hash_words(decoded, hash);
	CHECK(true, "valid LDPC init+encode+decode round trip");
	std::printf("VALID-LDPC-WITNESS encoded=%zu decoded=%zu iterations=%d fnv64=%016llx\n",
		encoded.size(), decoded.size(), iterations, (unsigned long long)hash);

	invalid_init_case("wrong LDPC code rate returns -1", MERCURY,
		MERCURY_NORMAL, 7.0f / 16.0f);
	invalid_init_case("non-MERCURY LDPC standard returns -1", MERCURY + 1,
		MERCURY_NORMAL, 1.0f / 16.0f);
	invalid_init_case("non-NORMAL LDPC frame size returns -1", MERCURY,
		MERCURY_NORMAL / 2, 1.0f / 16.0f);
	null_decode_case();

	std::printf("LDPC contract: %s (%d failures)\n",
		failures ? "FAIL" : "PASS", failures);
	return failures ? 1 : 0;
#endif
}
