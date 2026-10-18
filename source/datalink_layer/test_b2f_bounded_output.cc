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
	bool check_size, bool same_receive = false)
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
	if (same_receive)
	{
		if (!feed(sender, true, original + "FF\r", wire, split)) return false;
	}
	else if (!feed(sender, true, original, wire, split) ||
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
		for (bool same_receive : {false, true})
		for (int repeat = 0; repeat < 2; ++repeat)
		{
			sender.reset(); receiver.reset();
			std::string delivered;
			const std::string sid = cr("[PAT-12.18.0-B2FHQEHX$]");
			bool ok = forward(sender, receiver, true, sid, delivered, c.split) && delivered == sid;
			delivered.clear();
			ok = ok && forward(receiver, sender, true, sid, delivered, c.split) && delivered == sid;
			const bool first = ok && payload_round(sender, receiver, c.size, c.random,
				c.enabled, c.resume, c.rejected, c.split, true, same_receive);
			// A byte-exact first body alone can hide RX still being in payload state.
			const bool next = first && payload_round(sender, receiver, 5000, false,
				c.enabled, 0, false, c.split, false);
			std::printf("B2F_FIRST_PAYLOAD case=%zu same_receive=%d repeat=%d first=%d next=%d\n", i, same_receive, repeat, first, next);
			all = all && first && next;
		}
	}
	return all;
}


// A batch changes representation per accepted proposal, including after a
// resumed or oversized passthrough. Check each body and the next protocol round
// independently so correct prefix bytes cannot hide a retained payload state.
static bool post_passthrough_payloads()
{
	struct Case { size_t size; bool random; unsigned resume; bool enabled; };
	const Case cases[] = {
		{5000, false, 7, true},
		{B2F_PLAIN_BUF_SIZE + 1, false, 0, true},
		{B2F_PLAIN_BUF_SIZE - 1, true, 0, true},
		{5000, false, 0, true}, // two-unrolled control
		{5000, false, 7, false} // disabled control
	};
	bool all = true;
	for (size_t id = 0; id < sizeof(cases) / sizeof(cases[0]); ++id)
	for (int split : {173, 8192})
	{
		const Case& c = cases[id];
		std::string plain[2] = {std::string(c.size, ' '), std::string(5000, ' ')};
		std::string encoded[2];
		uint32_t x = 0x2468ace1u;
		for (int body = 0; body < 2; ++body)
		{
			const char text[] = "Keyed shelter status and resource requests.\n";
			for (size_t i = 0; i < plain[body].size(); ++i)
			{
				x ^= x << 13; x ^= x >> 17; x ^= x << 5;
				plain[body][i] = c.random && body == 0 ? (char)(x & 255) :
					text[(i + id + body * 7) % (sizeof(text) - 1)];
			}
			std::vector<uint8_t> buf(plain[body].size() * 2 + 65536);
			size_t count = 0;
			if (lzhuf_encode_buffer((const uint8_t*)plain[body].data(), plain[body].size(),
				buf.data(), buf.size(), &count) != 0 || count <= c.resume) return false;
			encoded[body].assign((const char*)buf.data(), count);
		}
		if ((id == 1 && encoded[0].size() > B2F_PLAIN_BUF_SIZE) ||
			(id == 2 && encoded[0].size() <= B2F_PLAIN_BUF_SIZE) ||
			encoded[1].size() >= plain[1].size()) return false;

		cl_b2f_handler sender, receiver;
		sender.init(); receiver.init();
		sender.unroll_enabled = receiver.unroll_enabled = c.enabled;
		for (int repeat = 0; repeat < 2; ++repeat)
		{
			sender.reset(); receiver.reset();
			std::string delivered;
			const std::string sid = cr("[PAT-12.18.0-B2FHQEHX$]");
			bool setup = forward(sender, receiver, true, sid, delivered, split) && delivered == sid;
			delivered.clear();
			setup = setup && forward(receiver, sender, true, sid, delivered, split) && delivered == sid;
			char fc[256];
			std::snprintf(fc, sizeof(fc),
				"FC EM FIRST %zu %zu\rFC EM SKIP 10 10\rFC EM SECOND %zu %zu\rF>\r",
				plain[0].size(), encoded[0].size(), plain[1].size(), encoded[1].size());
			delivered.clear();
			setup = setup && forward(sender, receiver, true, fc, delivered, split) && delivered == fc;
			char fs[64];
			if (c.resume) std::snprintf(fs, sizeof(fs), "FS !%u-+\r", c.resume);
			else std::snprintf(fs, sizeof(fs), "FS +-+\r");
			delivered.clear();
			setup = setup && forward(receiver, sender, true, fs, delivered, split) && delivered == fs;
			bool bodies[2] = {false, false};
			for (int body = 0; body < 2; ++body)
			{
				unsigned resume = body == 0 ? c.resume : 0;
				const bool transform = c.enabled && !resume &&
					plain[body].size() <= B2F_PLAIN_BUF_SIZE && encoded[body].size() <= B2F_PLAIN_BUF_SIZE;
				const std::string original = encoded[body].substr(resume);
				std::string wire;
				delivered.clear();
				bool sent = setup && feed(sender, true, original, wire, split);
				bool received = sent && feed(receiver, false, wire, delivered, split);
				bodies[body] = received && wire == (transform ? plain[body] : original) &&
					delivered == original && !receiver.is_reroll_poisoned();
				std::printf("B2F_POST_BODY case=%zu split=%d repeat=%d body=%d transform=%d plain=%zu comp=%zu wire=%zu delivered=%zu exact=%d poisoned=%d\n",
					id, split, repeat, body, transform, plain[body].size(), encoded[body].size(),
					wire.size(), delivered.size(), bodies[body], receiver.is_reroll_poisoned());
			}
			delivered.clear();
			bool tail = forward(sender, receiver, true, "FF\r", delivered, split) && delivered == "FF\r";
			bool next = tail && payload_round(sender, receiver, 5000, false, c.enabled, 0, false, split, false);
			bool ok = setup && bodies[0] && bodies[1] && tail && next &&
				!sender.has_fatal_error() && !receiver.has_fatal_error();
			std::printf("B2F_POST_PAYLOAD case=%zu split=%d repeat=%d setup=%d first=%d second=%d tail=%d next=%d exact=%d\n",
				id, split, repeat, setup, bodies[0], bodies[1], tail, next, ok);
			all = all && ok;
		}
	}
	return all;
}

// The receive ending a body also contains control framing. Choose the final
// drain's capacity from the known remaining byte count, so every output seam
// (exact fill, one byte free, two bytes free) is exercised in both directions.
static bool control_tail_case(bool tx, bool retained_body, int gap, bool requeue)
{
	std::string plain(5000, ' ');
	const char key[] = "TAIL-327 keyed application body\n";
	for (size_t i = 0; i < plain.size(); ++i)
		plain[i] = key[i % (sizeof(key) - 1)];
	std::vector<uint8_t> encoded(plain.size() * 2 + 65536);
	size_t count = 0;
	if (lzhuf_encode_buffer((const uint8_t*)plain.data(), plain.size(),
		encoded.data(), encoded.size(), &count) != 0) return false;
	cl_b2f_handler handler;
	handler.init();
	char out[8192];
	sid_exchange(handler, out, sizeof(out));
	auto filter = [&](const char* data, int len, int cap) {
		return tx ? handler.filter_tx(data, len, out, cap) :
			handler.filter_rx(data, len, out, cap);
	};
	auto pending = [&]() {
		return tx ? handler.has_pending_tx_work() : handler.has_pending_rx_work();
	};
	char fc[128];
	std::snprintf(fc, sizeof(fc), "FC EM TAIL327 %zu %zu\rF>\r", plain.size(), count);
	if (filter(fc, std::strlen(fc), sizeof(out)) != (int)std::strlen(fc)) return false;
	const std::string fs = "FS +\r";
	int n = tx ? handler.filter_rx(fs.data(), fs.size(), out, sizeof(out)) :
		handler.filter_tx(fs.data(), fs.size(), out, sizeof(out));
	if (n != (int)fs.size()) return false;
	const std::string body = tx ? plain : std::string((const char*)encoded.data(), count);
	const std::string input = (tx ? std::string((const char*)encoded.data(), count) : plain) + "FF\r";
	std::string delivered;
	int cap = retained_body ? 1 : (int)body.size() + gap;
	n = filter(input.data(), input.size(), cap);
	bool ok = n >= 0;
	if (n > 0)
	{
		if (requeue) ok = handler.requeue_tx_output(out, n);
		else delivered.append(out, n);
	}
	if (retained_body && ok)
	{
		// Restored bytes also remain before the retained body and control line.
		cap = (int)body.size() - (int)delivered.size() + gap;
		n = filter(nullptr, 0, cap);
		ok = n >= 0;
		if (n > 0) delivered.append(out, n);
	}
	// Capacity 0 preserves custody. Capacity 1 must make bounded progress even
	// when the remaining control record is longer than the caller's buffer.
	if (ok && pending()) ok = filter(nullptr, 0, 0) == 0 && pending();
	for (int i = 0; ok && pending() && i < 8192; ++i)
	{
		n = filter(nullptr, 0, 1);
		ok = n > 0;
		if (n > 0) delivered.append(out, n);
	}
	ok = ok && !pending() && delivered == body + "FF\r" && !handler.has_fatal_error();
	// A second proposal/response/body proves the control transition completed.
	if (ok)
	{
		if (filter(fc, std::strlen(fc), sizeof(out)) != (int)std::strlen(fc)) ok = false;
		n = tx ? handler.filter_rx(fs.data(), fs.size(), out, sizeof(out)) :
			handler.filter_tx(fs.data(), fs.size(), out, sizeof(out));
		if (n != (int)fs.size()) ok = false;
		n = filter(input.data(), input.size(), sizeof(out));
		ok = ok && n == (int)body.size() + 3 && std::string(out, n) == body + "FF\r";
	}
	std::printf("B2F_CONTROL_TAIL tx=%d retained=%d gap=%d requeue=%d exact=%d\n",
		tx, retained_body, gap, requeue, ok);
	return ok;
}

static bool small_control_lines(bool tx)
{
	cl_b2f_handler handler;
	handler.init();
	char out[8192];
	sid_exchange(handler, out, sizeof(out));
	auto filter = [&](const char* data, int len, int cap) {
		return tx ? handler.filter_tx(data, len, out, cap) :
			handler.filter_rx(data, len, out, cap);
	};
	auto pending = [&]() {
		return tx ? handler.has_pending_tx_work() : handler.has_pending_rx_work();
	};
	// A split accumulator, multiple complete lines and an empty line exercise
	// state transitions exactly once without using payload capture as scratch.
	bool ok = filter("F", 1, 1) == 0;
	std::string rest = "F\rFQ\r\r";
	int n = filter(rest.data(), rest.size(), 1);
	ok = ok && n == 1;
	std::string got;
	if (n > 0) got.append(out, n);
	for (int i = 0; ok && pending() && i < 32; ++i)
	{
		n = filter(nullptr, 0, 1);
		ok = n > 0;
		if (n > 0) got.append(out, n);
	}
	ok = ok && got == "FF\rFQ\r\r" && !pending();
	// Reset discards retained framing and its same-receive tail. A fresh
	// session must deliver only its new keyed data, in either direction.
	n = filter("FF\rFQ\r", 6, 1);
	ok = ok && n == 1 && pending();
	if (pending())
	{
		int opposite = tx ? handler.filter_rx("X", 1, out, sizeof(out)) :
			handler.filter_tx("X", 1, out, sizeof(out));
		ok = ok && opposite < 0 && !handler.has_fatal_error();
	}
	handler.reset();
	ok = ok && !pending() && filter(nullptr, 0, 1) == 0;
	sid_exchange(handler, out, sizeof(out));
	n = filter("NEW327\r", 7, sizeof(out));
	ok = ok && n == 7 && std::string(out, n) == "NEW327\r";
	std::printf("B2F_SMALL_CONTROL tx=%d exact=%d\n", tx, ok);
	return ok;
}

static bool control_tail_boundaries()
{
	bool all = true;
	for (bool tx : {false, true})
	{
		bool small = small_control_lines(tx);
		all = small && all;
		for (bool retained : {false, true})
		for (int gap : {0, 1, 2})
		{
			bool ok = control_tail_case(tx, retained, gap, false);
			all = ok && all;
			if (tx)
			{
				ok = control_tail_case(tx, retained, gap, true);
				all = ok && all;
			}
		}
	}
	return all;
}

int run_b2f_bounded_output_test()
{
	bool tail_ok = control_tail_boundaries();
	bool tx_ok = tx_large_body();
	bool rx_ok = rx_large_body();
	bool first_ok = first_payload_sizes();
	bool post_ok = post_passthrough_payloads();
	std::printf("B2F_POST_PAYLOAD_RESULT exact=%d\n", post_ok);
	std::printf("B2F_FIRST_PAYLOAD_RESULT exact=%d\n", first_ok);
	std::printf("B2F_BOUNDED_OUTPUT tx=%d rx=%d\n", tx_ok ? 1 : 0, rx_ok ? 1 : 0);
	std::printf("B2F_CONTROL_TAIL_RESULT exact=%d\n", tail_ok);
	return tail_ok && tx_ok && rx_ok && first_ok && post_ok ? 0 : 1;
}
