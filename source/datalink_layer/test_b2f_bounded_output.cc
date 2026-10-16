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

// Feed bounded receives and drain retained output before accepting more input.
static bool feed(cl_b2f_handler& handler, bool tx, const std::string& input,
	std::string& output, int split)
{
	char out[8192];
	for (size_t off = 0; off < input.size();)
	{
		int count = (int)(input.size() - off);
		if (count > split) count = split;
		int n = tx ? handler.filter_tx(input.data() + off, count, out, sizeof(out)) :
			handler.filter_rx(input.data() + off, count, out, sizeof(out));
		if (n < 0) return false;
		output.append(out, n);
		off += count;
		for (int i = 0; tx ? handler.has_pending_tx_work() : handler.has_pending_rx_work(); ++i)
		{
			if (i == 1024) return false;
			n = tx ? handler.filter_tx(nullptr, 0, out, sizeof(out)) :
				handler.filter_rx(nullptr, 0, out, sizeof(out));
			if (n < 0) return false;
			output.append(out, n);
		}
	}
	return true;
}

static bool forward(cl_b2f_handler& sender, cl_b2f_handler& receiver,
	bool forward_direction, const std::string& input, std::string& delivered, int split)
{
	std::string wire;
	return feed(sender, forward_direction, input, wire, split) &&
		feed(receiver, !forward_direction, wire, delivered, split);
}

static bool payload_round(cl_b2f_handler& sender, cl_b2f_handler& receiver,
	size_t size, bool random, bool enabled, unsigned resume, bool rejected, int split,
	bool check_size)
{
	std::string plain(size, ' ');
	uint32_t x = 0x2468ace1u;
	const char text[] = "Emergency shelter status and resource requests.\n";
	for (size_t i = 0; i < size; ++i)
	{
		x ^= x << 13; x ^= x >> 17; x ^= x << 5;
		plain[i] = random ? (char)(x & 255) : text[i % (sizeof(text) - 1)];
	}
	std::vector<uint8_t> encoded(size * 2 + 65536);
	size_t count = 0;
	if (lzhuf_encode_buffer((const uint8_t*)plain.data(), size,
		encoded.data(), encoded.size(), &count) != 0 || count <= resume) return false;
	bool fixture = !check_size || (random ? count > B2F_PLAIN_BUF_SIZE :
		count < B2F_PLAIN_BUF_SIZE);
	if (!fixture) return false;

	char fc[256];
	std::snprintf(fc, sizeof(fc), "FC EM BODY %zu %zu\rF>\r", size, count);
	std::string proposal = rejected ? "FC EM REJECT 10 10\r" : "";
	proposal += fc;
	std::string delivered;
	if (!forward(sender, receiver, true, proposal, delivered, split) || delivered != proposal)
		return false;
	char fs[64];
	if (resume) std::snprintf(fs, sizeof(fs), "FS %s!%u\r", rejected ? "-" : "", resume);
	else std::snprintf(fs, sizeof(fs), "FS %s+\r", rejected ? "-" : "");
	delivered.clear();
	if (!forward(receiver, sender, true, fs, delivered, split) || delivered != fs) return false;

	std::string original((const char*)encoded.data() + resume, count - resume);
	std::string wire;
	// Keep the control receive independent of transform drain capacity.
	if (!feed(sender, true, original, wire, split) ||
		!feed(sender, true, "FF\r", wire, split)) return false;
	const bool transform = enabled && !resume && size <= B2F_PLAIN_BUF_SIZE &&
		count <= B2F_PLAIN_BUF_SIZE;
	const std::string expected_wire = (transform ? plain : original) + "FF\r";
	delivered.clear();
	bool ok = wire == expected_wire && feed(receiver, false, wire, delivered, split) &&
		delivered == original + "FF\r" && !receiver.is_reroll_poisoned();
	std::printf("B2F_FIRST_ROUND size=%zu comp=%zu enabled=%d resume=%u rejected=%d split=%d transform=%d wire=%zu delivered=%zu exact=%d\n",
		size, count, enabled, resume, rejected, split, transform,
		wire.size(), delivered.size(), ok);
	return ok;
}

static bool first_payload_sizes()
{
	struct Case { size_t size; bool random; bool enabled; unsigned resume; bool rejected; int split; };
	const Case cases[] = {
		{B2F_PLAIN_BUF_SIZE - 1, false, true, 0, false, 173},
		{B2F_PLAIN_BUF_SIZE, false, true, 0, false, 8192},
		{B2F_PLAIN_BUF_SIZE + 1, false, true, 0, false, 173},
		{B2F_PLAIN_BUF_SIZE - 1, true, true, 0, false, 8192},
		{B2F_PLAIN_BUF_SIZE + 1, false, false, 0, false, 8192},
		{B2F_PLAIN_BUF_SIZE + 1, false, true, 7, false, 173},
		{B2F_PLAIN_BUF_SIZE + 1, false, true, 0, true, 8192},
		{B2F_PLAIN_BUF_SIZE + 1, false, true, 0, false, 8192}
	};
	bool all = true;
	for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i)
	{
		cl_b2f_handler sender, receiver;
		sender.init(); receiver.init();
		const Case& c = cases[i];
		sender.unroll_enabled = receiver.unroll_enabled = c.enabled;
		for (int repeat = 0; repeat < 2; ++repeat)
		{
			sender.reset(); receiver.reset();
			std::string delivered;
			const std::string sid = cr("[PAT-12.18.0-B2FHQEHX$]");
			bool ok = forward(sender, receiver, true, sid, delivered, c.split) && delivered == sid;
			delivered.clear();
			ok = ok && forward(receiver, sender, true, sid, delivered, c.split) && delivered == sid;
			const bool first = ok && payload_round(sender, receiver, c.size, c.random,
				c.enabled, c.resume, c.rejected, c.split, true);
			// A byte-exact first body alone can hide RX still being in payload state.
			const bool next = first && payload_round(sender, receiver, 5000, false,
				c.enabled, 0, false, c.split, false);
			std::printf("B2F_FIRST_PAYLOAD case=%zu repeat=%d first=%d next=%d\n", i, repeat, first, next);
			all = all && first && next;
		}
	}
	return all;
}

int run_b2f_bounded_output_test()
{
	bool tx_ok = tx_large_body();
	bool rx_ok = rx_large_body();
	bool first_ok = first_payload_sizes();
	std::printf("B2F_FIRST_PAYLOAD_RESULT exact=%d\n", first_ok);
	std::printf("B2F_BOUNDED_OUTPUT tx=%d rx=%d\n", tx_ok ? 1 : 0, rx_ok ? 1 : 0);
	return tx_ok && rx_ok && first_ok ? 0 : 1;
}
