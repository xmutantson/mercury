// Standalone round-trip test for pack/unpack_ack_sack_payload
// Build: g++ -std=c++17 -O2 source/physical_layer/test_pack_ack_sack.cc \
//        source/physical_layer/mfsk.cc -I include -o test_pack_ack_sack
// Run:   ./test_pack_ack_sack
#include "physical_layer/mfsk.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>

int main() {
	cl_mfsk m;
	m.init(16, 50, 1);  // WB-style M=16
	int n = m.ack_sack_suffix_len();
	if (n != 13) { fprintf(stderr, "expected 13 tones at M=16, got %d\n", n); return 1; }

	std::mt19937 rng(42);
	int n_ok = 0, n_fail = 0;
	for (int trial = 0; trial < 10000; trial++) {
		uint8_t  bsi    = (uint8_t)(rng() & 0xFF);
		uint32_t bitmap = (uint32_t)rng();
		uint16_t crc12  = (uint16_t)(rng() & 0x0FFF);
		int tones[16] = {0};
		int wrote = m.pack_ack_sack_payload(bsi, bitmap, crc12, tones);
		if (wrote != n) { fprintf(stderr, "pack returned %d\n", wrote); n_fail++; continue; }
		uint8_t  rx_bsi    = 0;
		uint32_t rx_bitmap = 0;
		uint16_t rx_crc12  = 0;
		bool ok = m.unpack_ack_sack_payload(tones, &rx_bsi, &rx_bitmap, &rx_crc12);
		if (!ok || rx_bsi != bsi || rx_bitmap != bitmap || rx_crc12 != crc12) {
			fprintf(stderr, "FAIL trial=%d bsi=%02x bitmap=%08x crc12=%03x -> rx_bsi=%02x rx_bitmap=%08x rx_crc12=%03x\n",
			        trial, bsi, bitmap, crc12, rx_bsi, rx_bitmap, rx_crc12);
			n_fail++;
		} else {
			n_ok++;
		}
	}
	printf("OK=%d FAIL=%d\n", n_ok, n_fail);
	return n_fail == 0 ? 0 : 1;
}
