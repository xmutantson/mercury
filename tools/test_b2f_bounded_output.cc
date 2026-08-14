/*
 * Production-cap B2F transform-output ownership regression.
 *
 * A completed transform may exceed the caller buffer. Every byte must remain
 * handler-owned and drain in order; consuming the record and returning zero is
 * silent deletion.
 */

#include "datalink_layer/b2f_handler.h"
#include "compression/lzhuf_buffer.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

static std::string cr(const char* s)
{
	return std::string(s) + "\r";
}

static void sid_exchange(cl_b2f_handler& handler, char* out, int out_cap)
{
	std::string s = cr("[PAT-12.18.0-B2FHQEHX$]");
	handler.filter_tx(s.data(), (int)s.size(), out, out_cap);
	s = cr("[CMS-5.0.0-B2FHQEHX$]");
	handler.filter_rx(s.data(), (int)s.size(), out, out_cap);
}

static bool tx_large_body()
{
	std::string plain;
	const char* line =
		"Mid: LARGE_TX Body: deterministic emergency traffic message. "
		"Shelter status, logistics, resource requests, and operator notes.\n";
	while (plain.size() < 40000)
		plain += line;
	plain.resize(40000);

	std::vector<uint8_t> encoded(plain.size() + 65536);
	size_t encoded_len = 0;
	if (lzhuf_encode_buffer((const uint8_t*)plain.data(), plain.size(),
		encoded.data(), encoded.size(), &encoded_len) != 0)
		return false;

	cl_b2f_handler handler;
	handler.init();
	char out[32768];
	sid_exchange(handler, out, sizeof(out));
	char fc[256];
	std::snprintf(fc, sizeof(fc), "FC EM LARGE_TX %zu %zu\rF>\r",
		plain.size(), encoded_len);
	handler.filter_tx(fc, (int)std::strlen(fc), out, sizeof(out));
	std::string s = cr("FS +");
	handler.filter_rx(s.data(), (int)s.size(), out, sizeof(out));

	std::vector<char> delivered;
	int n = handler.filter_tx((const char*)encoded.data(), (int)encoded_len,
		out, sizeof(out));
	if (n > 0) delivered.insert(delivered.end(), out, out + n);
	for (int i = 0; handler.has_pending_tx_work() && i < 32; i++)
	{
		n = handler.filter_tx(nullptr, 0, out, sizeof(out));
		if (n < 0) return false;
		delivered.insert(delivered.end(), out, out + n);
	}

	bool exact = delivered.size() == plain.size() &&
		std::memcmp(delivered.data(), plain.data(), plain.size()) == 0;
	std::printf("TX raw=%zu encoded=%zu delivered=%zu exact=%d\n",
		plain.size(), encoded_len, delivered.size(), exact ? 1 : 0);
	if (!exact || handler.has_pending_tx_work()) return false;

	// Re-run with a same-receive control tail and restore the first returned
	// prefix as if the downstream all-or-nothing FIFO rejected it.
	handler.reset();
	sid_exchange(handler, out, sizeof(out));
	std::snprintf(fc, sizeof(fc), "FC EM RETRY_TX %zu %zu\rF>\r",
		plain.size(), encoded_len);
	handler.filter_tx(fc, (int)std::strlen(fc), out, sizeof(out));
	s = cr("FS +");
	handler.filter_rx(s.data(), (int)s.size(), out, sizeof(out));
	std::vector<char> input(encoded.begin(), encoded.begin() + encoded_len);
	input.push_back('F'); input.push_back('F'); input.push_back('\r');
	n = handler.filter_tx(input.data(), (int)input.size(), out, sizeof(out));
	if (n <= 0 || !handler.requeue_tx_output(out, n)) return false;
	delivered.clear();
	for (int i = 0; handler.has_pending_tx_work() && i < 32; i++)
	{
		n = handler.filter_tx(nullptr, 0, out, sizeof(out));
		if (n < 0) return false;
		delivered.insert(delivered.end(), out, out + n);
	}
	std::string expected = plain + "FF\r";
	bool retry_exact = delivered.size() == expected.size() &&
		std::memcmp(delivered.data(), expected.data(), expected.size()) == 0;
	std::printf("TX retry+tail delivered=%zu exact=%d\n",
		delivered.size(), retry_exact ? 1 : 0);
	return retry_exact && !handler.has_pending_tx_work();
}

static bool rx_large_body()
{
	std::vector<char> plain(40000);
	uint32_t x = 0x2468ace1u;
	for (size_t i = 0; i < plain.size(); i++)
	{
		x ^= x << 13; x ^= x >> 17; x ^= x << 5;
		plain[i] = (char)(x & 0xff);
	}

	std::vector<uint8_t> expected(plain.size() + 65536);
	size_t expected_len = 0;
	if (lzhuf_encode_buffer((const uint8_t*)plain.data(), plain.size(),
		expected.data(), expected.size(), &expected_len) != 0 || expected_len <= 32768)
		return false;

	cl_b2f_handler handler;
	handler.init();
	char out[8192];
	sid_exchange(handler, out, sizeof(out));
	char fc[256];
	std::snprintf(fc, sizeof(fc), "FC EM LARGE_RX %zu %zu\rF>\r",
		plain.size(), expected_len);
	handler.filter_rx(fc, (int)std::strlen(fc), out, sizeof(out));
	std::string s = cr("FS +");
	handler.filter_tx(s.data(), (int)s.size(), out, sizeof(out));

	std::vector<char> delivered;
	for (size_t off = 0; off < plain.size();)
	{
		int chunk = (int)(plain.size() - off);
		if (chunk > 173) chunk = 173;
		int n = handler.filter_rx(plain.data() + off, chunk, out, sizeof(out));
		if (n < 0) return false;
		delivered.insert(delivered.end(), out, out + n);
		off += chunk;
	}
	for (int i = 0; handler.has_pending_rx_work() && i < 64; i++)
	{
		int n = handler.filter_rx(nullptr, 0, out, sizeof(out));
		if (n < 0) return false;
		delivered.insert(delivered.end(), out, out + n);
	}

	bool exact = delivered.size() == expected_len &&
		std::memcmp(delivered.data(), expected.data(), expected_len) == 0;
	std::printf("RX raw=%zu rerolled=%zu delivered=%zu exact=%d\n",
		plain.size(), expected_len, delivered.size(), exact ? 1 : 0);
	return exact && !handler.has_pending_rx_work();
}

int main()
{
	bool tx_ok = tx_large_body();
	bool rx_ok = rx_large_body();
	std::printf("B2F_BOUNDED_OUTPUT tx=%d rx=%d\n", tx_ok ? 1 : 0, rx_ok ? 1 : 0);
	return tx_ok && rx_ok ? 0 : 1;
}
