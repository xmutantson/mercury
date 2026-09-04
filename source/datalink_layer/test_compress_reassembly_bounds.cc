// Compression reassembly bounds regression.
//
// A maximum-width SACK batch can carry more than 16 KiB of compressed-envelope
// bytes. copy_data_to_buffer() used to account every ACKED frame in Option W but
// silently omit any frame that no longer fit its 16384-byte `assembled` array.
// This deterministic test stages a valid 96-frame, stamp-carrying compression
// batch and uses the real RX delivery funnel plus fifo_buffer_rx as byte oracle.

#include "datalink_layer/arq.h"
#include "datalink_layer/datalink_defines.h"
#include "common/common_defines.h"
#include "compression/mercury_compress.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

int cl_arq_controller::test_compress_reassembly_bounds()
{
	const char* defeat_env = std::getenv("MERCURY_COMPRESS_REASSEMBLY_BOUNDS_DEFEAT");
	const bool defeat = defeat_env && *defeat_env && std::atoi(defeat_env) != 0;
	int failed = 0;
	auto check = [&](bool condition, const char* name) {
		if(condition)
			std::printf("[TEST-COMP-REASSEMBLY-BOUNDS] PASS: %s\n", name);
		else
		{
			std::printf("[TEST-COMP-REASSEMBLY-BOUNDS] FAIL: %s\n", name);
			failed++;
		}
	};

	// 96 * ~178 bytes reproduces the large-batch geometry while remaining within
	// the real 200-byte per-message allocation (N_MAX / 8).
	const int frame_count = 96;
	const int plain_size = 17000;
	const int bsi = 37;

	nMessages = 120;
	max_data_length = 192;
	max_message_length = N_MAX / 8;
	max_header_length = 8;
	if(init_messages_buffers() != SUCCESSFUL)
	{
		std::printf("[TEST-COMP-REASSEMBLY-BOUNDS] FAIL: message-buffer allocation\n");
		return 1;
	}

	compression_enabled = true;
	encryption_enabled = false;
	sack_enabled = true;
	sack_v2_enabled = true;
	header_carries_d5 = true;
	current_configuration = CONFIG_13;
	original_role = COMMANDER; // keep fifo_push_rx() socket-free in this unit test
	role = COMMANDER;
	data_batch_size = frame_count;
	rx_copy_window = -1;
	decrypt_delivered_bsi = bsi;
	rx_stream_delivered = 0;
	rx_stream_emitted_bsi_hw = -1;
	rsp_cross_session_seam_armed = false;
	rsp_rebase_seam_armed = false;

	compressor.init();
	compressor.streaming_disable();
	cl_compressor tx_compressor;
	tx_compressor.init();
	tx_compressor.streaming_disable();

	std::vector<char> plain(plain_size);
	unsigned int state = 0x6d2b79f5u;
	for(int i = 0; i < plain_size; i++)
	{
		state ^= state << 13;
		state ^= state >> 17;
		state ^= state << 5;
		plain[i] = (char)(unsigned char)(state >> 24);
	}

	std::vector<char> wire(COMPRESS_WORKSPACE_SIZE);
	const int wire_size = tx_compressor.compress_block(
		plain.data(), (int)plain.size(), wire.data(), (int)wire.size());
	check(wire_size > 16384, "compressed envelope exceeds the former 16 KiB bound");
	check(wire_size <= COMPRESS_WORKSPACE_SIZE, "compressed envelope fits the protocol workspace");
	check(wire_size > 0 && ((unsigned char)wire[0] & COMPRESS_ALGO_MASK) == COMPRESS_ALGO_RAW,
		"deterministic high-entropy payload uses the lossless RAW compression envelope");

	int offset = 0;
	for(int i = 0; i < frame_count; i++)
	{
		const int slots_left = frame_count - i;
		const int length = (wire_size - offset + slots_left - 1) / slots_left;
		if(length <= 0 || length > N_MAX / 8)
		{
			std::printf("[TEST-COMP-REASSEMBLY-BOUNDS] FAIL: slot %d length=%d\n", i, length);
			return 1;
		}
		std::memcpy(messages_rx[i].data, wire.data() + offset, length);
		messages_rx[i].length = length;
		messages_rx[i].status = ACKED;
		offset += length;
	}
	check(offset == wire_size, "all transported bytes are staged across exactly 96 ACKED frames");

	rx_stream_stamp[bsi].start = rx_stream_delivered;
	rx_stream_stamp[bsi].length = (uint32_t)wire_size;
	rx_stream_stamp[bsi].valid = true;
	const uint64_t stamp_start = rx_stream_stamp[bsi].start;
	check(w_stamp_rides(), "test geometry carries an Option-W stream stamp");
	check(!w_stream_shift_detected(bsi), "stamp.start equals rx_stream_delivered before delivery");

	fifo_buffer_rx.set_size(COMPRESS_WORKSPACE_SIZE);
	fifo_buffer_rx.flush();
	copy_data_to_buffer();

	const int delivered = fifo_buffer_rx.get_size() - fifo_buffer_rx.get_free_size();
	std::vector<char> received(delivered > 0 ? delivered : 1);
	const int popped = delivered > 0 ? fifo_buffer_rx.pop(received.data(), delivered) : 0;
	const bool bytes_exact = popped == plain_size
		&& std::memcmp(received.data(), plain.data(), plain.size()) == 0;
	const bool cursor_exact = rx_stream_delivered == stamp_start + (uint64_t)wire_size;

	if(defeat)
	{
		check(!bytes_exact, "fail-before arm reproduces incomplete app delivery");
		check(cursor_exact, "fail-before arm still counts the omitted bytes as delivered");
	}
	else
	{
		check(bytes_exact, "every plaintext byte reaches fifo_buffer_rx in order");
		check(cursor_exact, "transported cursor advances by the complete stamped batch");
		check(!rx_stream_stamp[bsi].valid, "delivered stream stamp is consumed");
		bool all_free = true;
		for(int i = 0; i < frame_count; i++)
			if(messages_rx[i].status != FREE) all_free = false;
		check(all_free, "all delivered frame slots are released");
	}

	std::printf("[TEST-COMP-REASSEMBLY-BOUNDS] %s (defeat=%d wire=%d plain=%d frames=%d delivered=%d cursor=%llu)\n",
		failed == 0 ? "ALL PASS" : "FAILURES", defeat ? 1 : 0, wire_size,
		plain_size, frame_count, delivered, (unsigned long long)rx_stream_delivered);
	std::fflush(stdout);
	return failed == 0 ? 0 : 1;
}
