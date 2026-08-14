/*
 * B2F protocol handler implementation.
 *
 * Parses the B2F (Bin2Forwarding) protocol used by Winlink for message
 * transfer.  Identifies LZHUF-compressed payloads within the stream and
 * (when unrolling is enabled) decompresses them on TX / recompresses on RX.
 *
 * B2F is bidirectional: FC proposals may arrive via RX while FS responses
 * go out via TX (or vice versa).  The state machine is shared between both
 * directions, so line parsers check b2f_detected broadly rather than
 * restricting to specific states.
 *
 * B2F session structure (one round):
 *   [SID exchange]
 *   FC EM <mid> <uncomp> <comp>   (proposals, CR-delimited)
 *   F>                             (end of proposals)
 *   FS +-=...                      (accept/reject per proposal)
 *   <comp_size bytes>              (LZHUF payload for each accepted)
 *   FQ / FF                        (checksum confirmation)
 */

#include "datalink_layer/b2f_handler.h"
#include "compression/lzhuf_buffer.h"
#include <cstdio>
#include <cstdlib>

// ---- Constructor / Destructor ----

cl_b2f_handler::cl_b2f_handler()
{
	payload_buf = nullptr;
	plain_buf = nullptr;
	rx_pending_buf = nullptr;
	initialized = false;
	unroll_enabled = true;
	tx_pending_output_pos = 0;
	tx_pending_output_len = 0;
	tx_deferred_input_len = 0;
	rx_pending_output_pos = 0;
	rx_pending_output_len = 0;
	rx_deferred_input_len = 0;
	fatal_error = false;
	reset();
}

cl_b2f_handler::~cl_b2f_handler()
{
	deinit();
}

void cl_b2f_handler::init()
{
	if (initialized) return;

	payload_buf = (uint8_t*)malloc(B2F_PAYLOAD_BUF_SIZE);
	plain_buf = (uint8_t*)malloc(B2F_PLAIN_BUF_SIZE);
	rx_pending_buf = (uint8_t*)malloc(B2F_PLAIN_BUF_SIZE);

	if (!payload_buf || !plain_buf || !rx_pending_buf)
	{
		deinit();
		return;
	}

	initialized = true;
	printf("[B2F] Handler initialized (payload buf %d KB, plain buf %d KB)\n",
		B2F_PAYLOAD_BUF_SIZE / 1024, B2F_PLAIN_BUF_SIZE / 1024);
	fflush(stdout);
}

void cl_b2f_handler::deinit()
{
	if (payload_buf) { free(payload_buf); payload_buf = nullptr; }
	if (plain_buf) { free(plain_buf); plain_buf = nullptr; }
	if (rx_pending_buf) { free(rx_pending_buf); rx_pending_buf = nullptr; }
	tx_pending_output_pos = 0;
	tx_pending_output_len = 0;
	tx_deferred_input_len = 0;
	rx_pending_output_pos = 0;
	rx_pending_output_len = 0;
	rx_deferred_input_len = 0;
	fatal_error = false;
	initialized = false;
}

void cl_b2f_handler::reset()
{
	tx_pending_output_pos = 0;
	tx_pending_output_len = 0;
	tx_deferred_input_len = 0;
	rx_pending_output_pos = 0;
	rx_pending_output_len = 0;
	rx_deferred_input_len = 0;
	fatal_error = false;
	state = B2F_IDLE;
	current_proposer = PROPOSER_NONE;
	b2f_detected = false;
	num_proposals = 0;
	current_payload_idx = -1;
	payload_bytes_remaining = 0;
	tx_line_pos = 0;
	rx_line_pos = 0;
	payload_buf_pos = 0;
	reroll_divergence = 0;
	reroll_poisoned = false;
}

bool cl_b2f_handler::has_pending_tx_work() const
{
	return tx_pending_output_pos < tx_pending_output_len ||
		tx_deferred_input_len > 0;
}

bool cl_b2f_handler::has_pending_rx_work() const
{
	return rx_pending_output_pos < rx_pending_output_len ||
		rx_deferred_input_len > 0;
}

bool cl_b2f_handler::requeue_tx_output(const char* data, int len)
{
	if (len <= 0)
		return true;
	if (!data || !payload_buf)
	{
		fatal_error = true;
		return false;
	}

	int remaining = tx_pending_output_len - tx_pending_output_pos;
	if (remaining < 0 || len > B2F_PAYLOAD_BUF_SIZE - remaining)
	{
		fatal_error = true;
		return false;
	}
	if (remaining > 0)
		memmove(payload_buf + len, payload_buf + tx_pending_output_pos, remaining);
	memcpy(payload_buf, data, len);
	tx_pending_output_pos = 0;
	tx_pending_output_len = len + remaining;
	return true;
}

int cl_b2f_handler::drain_tx_pending(char* out, int out_cap)
{
	if (out_cap <= 0 || tx_pending_output_pos >= tx_pending_output_len)
		return 0;
	int remaining = tx_pending_output_len - tx_pending_output_pos;
	int copy = remaining < out_cap ? remaining : out_cap;
	memcpy(out, payload_buf + tx_pending_output_pos, copy);
	tx_pending_output_pos += copy;
	if (tx_pending_output_pos == tx_pending_output_len)
	{
		tx_pending_output_pos = 0;
		tx_pending_output_len = 0;
		payload_buf_pos = 0;
	}
	return copy;
}

int cl_b2f_handler::drain_rx_pending(char* out, int out_cap)
{
	if (out_cap <= 0 || rx_pending_output_pos >= rx_pending_output_len)
		return 0;
	int remaining = rx_pending_output_len - rx_pending_output_pos;
	int copy = remaining < out_cap ? remaining : out_cap;
	memcpy(out, rx_pending_buf + rx_pending_output_pos, copy);
	rx_pending_output_pos += copy;
	if (rx_pending_output_pos == rx_pending_output_len)
	{
		rx_pending_output_pos = 0;
		rx_pending_output_len = 0;
		payload_buf_pos = 0;
	}
	return copy;
}

// ---- B2F Line Parsers ----

bool cl_b2f_handler::parse_sid_line(const char* line, int len)
{
	if (len < 5 || line[0] != '[' || line[len-1] != ']')
		return false;
	for (int i = 0; i < len - 2; i++)
	{
		if (line[i] == 'B' && line[i+1] == '2' && line[i+2] == 'F')
			return true;
	}
	return false;
}

bool cl_b2f_handler::parse_fc_line(const char* line, int len, st_b2f_proposal* prop)
{
	if (len < 10 || line[0] != 'F' || line[1] != 'C' || line[2] != ' ')
		return false;

	if (line[3] == 'E' && line[4] == 'M')
		prop->type = 'E';
	else if (line[3] == 'C' && line[4] == 'M')
		prop->type = 'C';
	else
		return false;

	int pos = 6;

	int mid_start = pos;
	while (pos < len && line[pos] != ' ') pos++;
	int mid_len = pos - mid_start;
	if (mid_len <= 0 || mid_len > 12) return false;
	memcpy(prop->mid, line + mid_start, mid_len);
	prop->mid[mid_len] = '\0';
	pos++;

	prop->uncomp_size = 0;
	while (pos < len && line[pos] >= '0' && line[pos] <= '9')
	{
		prop->uncomp_size = prop->uncomp_size * 10 + (line[pos] - '0');
		pos++;
	}
	pos++;

	prop->comp_size = 0;
	while (pos < len && line[pos] >= '0' && line[pos] <= '9')
	{
		prop->comp_size = prop->comp_size * 10 + (line[pos] - '0');
		pos++;
	}

	prop->accepted = -1;
	prop->resume_offset = 0;
	return true;
}

bool cl_b2f_handler::parse_fs_line(const char* line, int len)
{
	if (len < 3 || line[0] != 'F' || line[1] != 'S' || line[2] != ' ')
		return false;

	int pos = 3;
	for (int i = 0; i < num_proposals && pos < len; i++, pos++)
	{
		// FBB protocol FS response codes:
		//   +/Y = accepted, -/N/R/E = rejected, =/L = deferred, H = hold (accepted)
		//   !offset = accepted with resume from byte offset
		switch (line[pos])
		{
			case '+': case 'Y':
				proposals[i].accepted = 1; break;
			case '-': case 'N': case 'R': case 'E':
				proposals[i].accepted = 0; break;
			case '=': case 'L':
				proposals[i].accepted = -1; break;
			case 'H':
				proposals[i].accepted = 1; break;
			case '!': {
				proposals[i].accepted = 1;
				// Parse trailing offset digits
				uint32_t offset = 0;
				pos++;
				while (pos < len && line[pos] >= '0' && line[pos] <= '9')
					offset = offset * 10 + (line[pos++] - '0');
				proposals[i].resume_offset = offset;
				pos--;  // loop will increment
				printf("[B2F] FS: proposal %d accepted with resume offset %u\n",
					i, offset);
				fflush(stdout);
				break;
			}
			default: break;
		}
	}
	return true;
}

int cl_b2f_handler::find_next_accepted(int from)
{
	for (int i = from; i < num_proposals; i++)
	{
		if (proposals[i].accepted == 1)
			return i;
	}
	return -1;
}

// Helper: start payload transfer after FS acceptance
static void start_payload_for_proposer(cl_b2f_handler* h, const char* tag);

// ---- Shared line processing (called from both TX and RX) ----
//
// B2F lines can arrive from either direction:
//   - SID: both sides send one
//   - FC + F>: sent by the proposer (could be local or remote)
//   - FS: sent by the responder (opposite direction of FC)
//   - FQ/FF: either side
//
// We process all recognized B2F lines regardless of direction, using
// b2f_detected as the gate.

int cl_b2f_handler::process_tx_line(const char* line, int len, char* out, int out_cap)
{
	if (len == 0)
		goto passthrough;

	// --- SID detection ---
	if (!b2f_detected || state == B2F_SID_EXCHANGE)
	{
		if (parse_sid_line(line, len))
		{
			b2f_detected = true;
			state = B2F_SID_EXCHANGE;
			printf("[B2F-TX] SID: %.*s\n", len, line);
			fflush(stdout);
			goto passthrough;
		}
	}

	if (!b2f_detected)
		goto passthrough;

	// After SID exchange, any non-SID line advances to WAIT_PROPOSALS
	if (state == B2F_SID_EXCHANGE)
		state = B2F_WAIT_PROPOSALS;

	// --- FC proposal (local client proposing outward) ---
	{
		st_b2f_proposal prop;
		if (parse_fc_line(line, len, &prop))
		{
			if (current_proposer != PROPOSER_LOCAL)
			{
				num_proposals = 0;
				current_proposer = PROPOSER_LOCAL;
			}
			state = B2F_PARSING_FC;
			if (num_proposals < B2F_MAX_PROPOSALS)
			{
				proposals[num_proposals++] = prop;
				printf("[B2F-TX] FC: %s %cM uncomp=%u comp=%u\n",
					prop.mid, prop.type, prop.uncomp_size, prop.comp_size);
				fflush(stdout);
			}
			goto passthrough;
		}
	}

	// --- F> (end of proposals from local) ---
	if (len >= 2 && line[0] == 'F' && line[1] == '>')
	{
		state = B2F_WAIT_FS;
		printf("[B2F-TX] F> — %d proposals from local, awaiting FS\n", num_proposals);
		fflush(stdout);
		goto passthrough;
	}

	// --- FS response (local responding to remote proposals) ---
	if (line[0] == 'F' && line[1] == 'S' && len >= 3)
	{
		if (parse_fs_line(line, len))
		{
			printf("[B2F-TX] FS sent:");
			for (int i = 0; i < num_proposals; i++)
				printf(" %c", proposals[i].accepted == 1 ? '+' :
					proposals[i].accepted == 0 ? '-' : '=');
			printf("\n");
			fflush(stdout);

			// Remote proposed, we responded — remote payloads come via RX
			if (current_proposer == PROPOSER_REMOTE)
			{
				current_payload_idx = find_next_accepted(0);
				if (current_payload_idx >= 0)
				{
					// RX side will receive plaintext (if unroll) or LZHUF (if not)
					// Resume transfers send partial LZHUF — can't unroll
					auto& prop = proposals[current_payload_idx];
					bool can_unroll = unroll_enabled && prop.resume_offset == 0;
					payload_bytes_remaining = can_unroll ?
						prop.uncomp_size :
						(prop.comp_size - prop.resume_offset);
					state = B2F_PAYLOAD_TRANSFER;
					payload_buf_pos = 0;
					printf("[B2F] Remote payloads expected via RX (%u bytes %s)\n",
						payload_bytes_remaining,
						can_unroll ? "plaintext" : "LZHUF");
					fflush(stdout);
				}
				else
					state = B2F_CHECKSUM;
			}
			goto passthrough;
		}
	}

	// --- FF / FQ ---
	if (len >= 2 && line[0] == 'F' && (line[1] == 'F' || line[1] == 'Q'))
	{
		printf("[B2F-TX] %c%c\n", line[0], line[1]);
		fflush(stdout);
		state = B2F_WAIT_PROPOSALS;
		current_proposer = PROPOSER_NONE;
		goto passthrough;
	}

passthrough:
	if (len + 1 > out_cap)
		return -1;
	memcpy(out, line, len);
	out[len] = '\r';
	return len + 1;
}

int cl_b2f_handler::process_rx_line(const char* line, int len, char* out, int out_cap)
{
	if (len == 0)
		goto passthrough;

	// --- SID detection ---
	if (!b2f_detected || state == B2F_SID_EXCHANGE)
	{
		if (parse_sid_line(line, len))
		{
			b2f_detected = true;
			state = B2F_SID_EXCHANGE;
			printf("[B2F-RX] SID: %.*s\n", len, line);
			fflush(stdout);
			goto passthrough;
		}
	}

	if (!b2f_detected)
		goto passthrough;

	if (state == B2F_SID_EXCHANGE)
		state = B2F_WAIT_PROPOSALS;

	// --- FC proposal (remote proposing to us) ---
	{
		st_b2f_proposal prop;
		if (parse_fc_line(line, len, &prop))
		{
			if (current_proposer != PROPOSER_REMOTE)
			{
				num_proposals = 0;
				current_proposer = PROPOSER_REMOTE;
			}
			state = B2F_PARSING_FC;
			if (num_proposals < B2F_MAX_PROPOSALS)
			{
				proposals[num_proposals++] = prop;
				printf("[B2F-RX] FC: %s %cM uncomp=%u comp=%u\n",
					prop.mid, prop.type, prop.uncomp_size, prop.comp_size);
				fflush(stdout);
			}
			goto passthrough;
		}
	}

	// --- F> (end of proposals from remote) ---
	if (len >= 2 && line[0] == 'F' && line[1] == '>')
	{
		state = B2F_WAIT_FS;
		printf("[B2F-RX] F> — %d proposals from remote, awaiting FS\n", num_proposals);
		fflush(stdout);
		goto passthrough;
	}

	// --- FS response from remote (to our local proposals) ---
	if (line[0] == 'F' && line[1] == 'S' && len >= 3)
	{
		if (parse_fs_line(line, len))
		{
			printf("[B2F-RX] FS received:");
			for (int i = 0; i < num_proposals; i++)
				printf(" %c", proposals[i].accepted == 1 ? '+' :
					proposals[i].accepted == 0 ? '-' : '=');
			printf("\n");
			fflush(stdout);

			// We proposed, remote responded — our payloads flow via TX
			if (current_proposer == PROPOSER_LOCAL)
			{
				current_payload_idx = find_next_accepted(0);
				if (current_payload_idx >= 0)
				{
					auto& prop = proposals[current_payload_idx];
					payload_bytes_remaining = prop.comp_size - prop.resume_offset;
					state = B2F_PAYLOAD_TRANSFER;
					payload_buf_pos = 0;
					printf("[B2F] Local payloads will flow via TX (%u bytes LZHUF%s)\n",
						payload_bytes_remaining,
						prop.resume_offset > 0 ? " resume" : "");
					fflush(stdout);
				}
				else
					state = B2F_CHECKSUM;
			}
			goto passthrough;
		}
	}

	// --- FF / FQ ---
	if (len >= 2 && line[0] == 'F' && (line[1] == 'F' || line[1] == 'Q'))
	{
		printf("[B2F-RX] %c%c\n", line[0], line[1]);
		fflush(stdout);
		state = B2F_WAIT_PROPOSALS;
		current_proposer = PROPOSER_NONE;
		goto passthrough;
	}

passthrough:
	if (len + 1 > out_cap)
		return -1;
	memcpy(out, line, len);
	out[len] = '\r';
	return len + 1;
}

// ---- TX payload handling ----

int cl_b2f_handler::process_tx_payload(const char* in, int in_len, char* out, int out_cap, int* in_consumed)
{
	if (current_proposer != PROPOSER_LOCAL || current_payload_idx < 0)
	{
		int copy = in_len < out_cap ? in_len : out_cap;
		memcpy(out, in, copy);
		*in_consumed = copy;
		return copy;
	}

	int out_pos = 0;
	int in_pos = 0;

	while (in_pos < in_len && payload_bytes_remaining > 0)
	{
		int chunk = in_len - in_pos;
		if (chunk > payload_bytes_remaining)
			chunk = payload_bytes_remaining;

		// Can only unroll full transfers — resume sends partial LZHUF
		bool can_unroll = unroll_enabled && initialized &&
		                  proposals[current_payload_idx].resume_offset == 0 &&
		                  proposals[current_payload_idx].comp_size <= B2F_PLAIN_BUF_SIZE &&
		                  proposals[current_payload_idx].uncomp_size <= B2F_PLAIN_BUF_SIZE;
		if (can_unroll)
		{
			if (payload_buf_pos + chunk <= B2F_PAYLOAD_BUF_SIZE)
			{
				memcpy(payload_buf + payload_buf_pos, in + in_pos, chunk);
				payload_buf_pos += chunk;
			}
			in_pos += chunk;
			payload_bytes_remaining -= chunk;

			if (payload_bytes_remaining == 0)
			{
				size_t plain_len = 0;
				int rc = lzhuf_decode_buffer(payload_buf, payload_buf_pos,
					plain_buf, B2F_PLAIN_BUF_SIZE, &plain_len);

				if (rc == 0 && plain_len > 0)
				{
					printf("[B2F-TX] Unrolled %s: %d LZHUF -> %zu plaintext (%.1fx)\n",
						proposals[current_payload_idx].mid,
						payload_buf_pos, plain_len,
						(float)payload_buf_pos / (float)plain_len);
					fflush(stdout);

					// Caller capacity is backpressure, not permission to discard a
					// completed record. Retain it and release bounded prefixes.
					memcpy(payload_buf, plain_buf, plain_len);
					tx_pending_output_pos = 0;
					tx_pending_output_len = (int)plain_len;
				}
				else
				{
					printf("[B2F-TX] LZHUF decode FAILED for %s (rc=%d), passthrough\n",
						proposals[current_payload_idx].mid, rc);
					fflush(stdout);
					tx_pending_output_pos = 0;
					tx_pending_output_len = payload_buf_pos;
				}

				current_payload_idx = find_next_accepted(current_payload_idx + 1);
				if (current_payload_idx >= 0)
				{
					auto& np = proposals[current_payload_idx];
					payload_bytes_remaining = np.comp_size - np.resume_offset;
					printf("[B2F-TX] Next payload: %s (%u bytes%s)\n",
						np.mid, payload_bytes_remaining,
						np.resume_offset > 0 ? " resume" : "");
					fflush(stdout);
				}
				else
				{
					state = B2F_CHECKSUM;
					printf("[B2F-TX] All payloads unrolled\n");
					fflush(stdout);
				}

				out_pos += drain_tx_pending(out + out_pos, out_cap - out_pos);
				if (tx_pending_output_pos < tx_pending_output_len)
					break;
			}
		}
		else
		{
			int room = out_cap - out_pos;
			if (chunk > room) chunk = room;
			if (chunk <= 0) break;
			memcpy(out + out_pos, in + in_pos, chunk);
			out_pos += chunk;
			in_pos += chunk;
			payload_bytes_remaining -= chunk;

			if (payload_bytes_remaining == 0)
			{
				current_payload_idx = find_next_accepted(current_payload_idx + 1);
				if (current_payload_idx >= 0)
				{
					auto& np = proposals[current_payload_idx];
					payload_bytes_remaining = np.comp_size - np.resume_offset;
				}
				else
					state = B2F_CHECKSUM;
			}
		}
	}

	// Return only the payload bytes consumed; remaining bytes go back to
	// the filter's line parser for proper state-machine processing.
	*in_consumed = in_pos;
	return out_pos;
}

// ---- RX payload handling ----

int cl_b2f_handler::process_rx_payload(const char* in, int in_len, char* out, int out_cap, int* in_consumed)
{
	if (current_proposer != PROPOSER_REMOTE || current_payload_idx < 0)
	{
		int copy = in_len < out_cap ? in_len : out_cap;
		memcpy(out, in, copy);
		*in_consumed = copy;
		return copy;
	}

	int out_pos = 0;
	int in_pos = 0;

	while (in_pos < in_len && payload_bytes_remaining > 0)
	{
		int chunk = in_len - in_pos;
		if (chunk > payload_bytes_remaining)
			chunk = payload_bytes_remaining;

		// Can only reroll full transfers — resume sends partial LZHUF
		bool can_unroll = unroll_enabled && initialized &&
		                  proposals[current_payload_idx].resume_offset == 0 &&
		                  proposals[current_payload_idx].comp_size <= B2F_PLAIN_BUF_SIZE &&
		                  proposals[current_payload_idx].uncomp_size <= B2F_PLAIN_BUF_SIZE;
		if (can_unroll)
		{
			if (payload_buf_pos + chunk <= B2F_PAYLOAD_BUF_SIZE)
			{
				memcpy(payload_buf + payload_buf_pos, in + in_pos, chunk);
				payload_buf_pos += chunk;
			}
			in_pos += chunk;
			payload_bytes_remaining -= chunk;

			if (payload_bytes_remaining == 0)
			{
				size_t lzhuf_len = 0;
				int rc = lzhuf_encode_buffer(payload_buf, payload_buf_pos,
					plain_buf, B2F_PLAIN_BUF_SIZE, &lzhuf_len);

				if (rc == 0 && lzhuf_len > 0)
				{
					if ((uint32_t)lzhuf_len == proposals[current_payload_idx].comp_size)
					{
						printf("[B2F-RX] Rerolled %s: %d plaintext -> %zu LZHUF (match)\n",
							proposals[current_payload_idx].mid,
							payload_buf_pos, lzhuf_len);
						fflush(stdout);

						memcpy(rx_pending_buf, plain_buf, lzhuf_len);
						rx_pending_output_pos = 0;
						rx_pending_output_len = (int)lzhuf_len;
					}
					else
					{
						// FATAL reroll DIVERGENCE: the re-encoded LZHUF length does not
						// match the sender's declared comp_size, which PROVES this encoder's
						// output is not byte-identical to the sending client's. A resumed
						// splice (the client-saved reroll prefix ++ the sender-resent
						// original suffix) is correct ONLY if the two encoders are byte-
						// identical, so a divergent reroll must NEVER go on the wire. On the
						// RX side we hold only the unrolled plaintext, not the sender's
						// original LZHUF, so there is no safe substitute stream to ship.
						// Fail closed: drop the divergent body and POISON the B2F transform
						// for the rest of this session (filter_rx then ships nothing) so
						// neither divergent nor framing-desynced bytes reach the client. The
						// client re-requests from offset 0 on the fresh reconnect. A test-only
						// defeat knob (MERCURY_B2F_REROLL_MISMATCH_SHIP=1) restores the old
						// warn-and-ship for the fail-before arm of the reroll-mismatch test.
						reroll_divergence++;
						bool ship_divergent = false;
						{ const char* e = std::getenv("MERCURY_B2F_REROLL_MISMATCH_SHIP");
						  if (e && *e && atoi(e) != 0) ship_divergent = true; }
						if (ship_divergent)
						{
							printf("[B2F-RX] WARNING(defeat): Rerolled %zu != declared %u for %s "
								"— shipping divergent LZHUF (MERCURY_B2F_REROLL_MISMATCH_SHIP)\n",
								lzhuf_len, proposals[current_payload_idx].comp_size,
								proposals[current_payload_idx].mid);
							fflush(stdout);
							memcpy(rx_pending_buf, plain_buf, lzhuf_len);
							rx_pending_output_pos = 0;
							rx_pending_output_len = (int)lzhuf_len;
						}
						else
						{
							reroll_poisoned = true;
							printf("[B2F-RX] FATAL: reroll size mismatch for %s (rerolled %zu != "
								"declared %u) — refusing to ship divergent LZHUF; poisoning B2F "
								"transform (fail-closed; client re-requests from offset 0) "
								"(divergence count=%lld)\n",
								proposals[current_payload_idx].mid, lzhuf_len,
								proposals[current_payload_idx].comp_size, reroll_divergence);
							fflush(stdout);
						}
					}
				}
				else
				{
					printf("[B2F-RX] LZHUF encode FAILED for %s (rc=%d), passthrough\n",
						proposals[current_payload_idx].mid, rc);
					fflush(stdout);
					memcpy(rx_pending_buf, payload_buf, payload_buf_pos);
					rx_pending_output_pos = 0;
					rx_pending_output_len = payload_buf_pos;
				}

				current_payload_idx = find_next_accepted(current_payload_idx + 1);
				if (current_payload_idx >= 0)
				{
					auto& np = proposals[current_payload_idx];
					bool next_can_unroll = unroll_enabled && initialized &&
						np.resume_offset == 0 &&
						np.comp_size <= B2F_PLAIN_BUF_SIZE &&
						np.uncomp_size <= B2F_PLAIN_BUF_SIZE;
					payload_bytes_remaining = next_can_unroll ?
						np.uncomp_size :
						(np.comp_size - np.resume_offset);
					printf("[B2F-RX] Next payload: %s (%u bytes%s)\n",
						np.mid, payload_bytes_remaining,
						np.resume_offset > 0 ? " resume" : "");
					fflush(stdout);
				}
				else
				{
					state = B2F_CHECKSUM;
					printf("[B2F-RX] All payloads rerolled\n");
					fflush(stdout);
				}

				out_pos += drain_rx_pending(out + out_pos, out_cap - out_pos);
				if (rx_pending_output_pos < rx_pending_output_len)
					break;
			}
		}
		else
		{
			int room = out_cap - out_pos;
			if (chunk > room) chunk = room;
			if (chunk <= 0) break;
			memcpy(out + out_pos, in + in_pos, chunk);
			out_pos += chunk;
			in_pos += chunk;
			payload_bytes_remaining -= chunk;

			if (payload_bytes_remaining == 0)
			{
				current_payload_idx = find_next_accepted(current_payload_idx + 1);
				if (current_payload_idx >= 0)
				{
					auto& np = proposals[current_payload_idx];
					payload_bytes_remaining = np.comp_size - np.resume_offset;
				}
				else
					state = B2F_CHECKSUM;
			}
		}
	}

	// Return only the payload bytes consumed; remaining bytes go back to
	// the filter's line parser for proper state-machine processing.
	*in_consumed = in_pos;
	return out_pos;
}

// ---- Top-level filters ----

int cl_b2f_handler::filter_tx_input(const char* in, int in_len, char* out, int out_cap)
{
	if (!initialized)
	{
		if (in_len > out_cap) return -1;
		memcpy(out, in, in_len);
		return in_len;
	}

	int out_pos = 0;
	int in_pos = 0;

	// Pre-detection: pass bytes through one at a time while shadow-scanning
	// for B2F SID.  When SID is detected, stop passthrough at the \r boundary
	// and fall through to the line parser for remaining bytes in this chunk.
	// (Bulk-copy passthrough would duplicate post-SID bytes: once from
	// passthrough, again from the line parser on the next call.)
	if (!b2f_detected)
	{
		for (; in_pos < in_len && !b2f_detected && out_pos < out_cap; in_pos++)
		{
			char c = in[in_pos];

			if (out_pos < out_cap)
				out[out_pos++] = c;

			if (c == '\r')
			{
				if (tx_line_pos > 0)
				{
					tx_line_buf[tx_line_pos] = '\0';
					if (parse_sid_line(tx_line_buf, tx_line_pos))
					{
						b2f_detected = true;
						state = B2F_SID_EXCHANGE;
						printf("[B2F-TX] SID detected: %.*s\n", tx_line_pos, tx_line_buf);
						fflush(stdout);
					}
					tx_line_pos = 0;
				}
			}
			else if (c != '\n')
			{
				if (tx_line_pos < B2F_LINE_BUF_SIZE - 1)
					tx_line_buf[tx_line_pos++] = c;
				else
					tx_line_pos = 0;
			}
		}

		if (!b2f_detected && in_pos < in_len)
		{
			int remaining = in_len - in_pos;
			if (remaining > B2F_DEFERRED_INPUT_SIZE || tx_deferred_input_len != 0)
			{
				fatal_error = true;
				return -1;
			}
			memcpy(tx_deferred_input, in + in_pos, remaining);
			tx_deferred_input_len = remaining;
			return out_pos;
		}

		if (!b2f_detected)
			return out_pos;

		// SID found mid-chunk: remaining bytes fall through to line parser
	}

	// B2F active: full line parser with state machine

	while (in_pos < in_len)
	{
		if (out_pos >= out_cap)
		{
			int remaining = in_len - in_pos;
			if (remaining > B2F_DEFERRED_INPUT_SIZE || tx_deferred_input_len != 0)
			{
				fatal_error = true;
				return -1;
			}
			memcpy(tx_deferred_input, in + in_pos, remaining);
			tx_deferred_input_len = remaining;
			break;
		}

		if (state == B2F_PAYLOAD_TRANSFER && current_proposer == PROPOSER_LOCAL)
		{
			int avail = in_len - in_pos;
			int consumed = 0;
			int written = process_tx_payload(in + in_pos, avail, out + out_pos, out_cap - out_pos, &consumed);
			if (written < 0) return -1;
			out_pos += written;
			in_pos += consumed;
			if (tx_pending_output_pos < tx_pending_output_len)
			{
				int remaining = in_len - in_pos;
				if (remaining > B2F_DEFERRED_INPUT_SIZE || tx_deferred_input_len != 0)
				{
					fatal_error = true;
					return -1;
				}
				if (remaining > 0)
				{
					memcpy(tx_deferred_input, in + in_pos, remaining);
					tx_deferred_input_len = remaining;
				}
				break;
			}
			if (consumed == 0 && written == 0)
			{
				fatal_error = true;
				return -1;
			}
		}
		else
		{
			char c = in[in_pos++];

			if (c == '\r')
			{
				tx_line_buf[tx_line_pos] = '\0';
				int written = process_tx_line(tx_line_buf, tx_line_pos, out + out_pos, out_cap - out_pos);
				if (written < 0) return -1;
				out_pos += written;
				tx_line_pos = 0;
			}
			else if (c != '\n')
			{
				if (tx_line_pos < B2F_LINE_BUF_SIZE - 1)
					tx_line_buf[tx_line_pos++] = c;
			}
		}
	}

	return out_pos;
}

int cl_b2f_handler::filter_tx(const char* in, int in_len, char* out, int out_cap)
{
	if (in_len < 0 || out_cap < 0 || (in_len > 0 && !in) || !out || fatal_error)
		return -1;
	if (in_len > 0 && has_pending_rx_work())
		return -1;

	int out_pos = drain_tx_pending(out, out_cap);
	if (tx_pending_output_pos < tx_pending_output_len)
		return in_len == 0 ? out_pos : -1;

	if (tx_deferred_input_len > 0)
	{
		if (in_len > 0)
			return -1;
		char deferred[B2F_DEFERRED_INPUT_SIZE];
		int deferred_len = tx_deferred_input_len;
		memcpy(deferred, tx_deferred_input, deferred_len);
		tx_deferred_input_len = 0;
		int written = filter_tx_input(deferred, deferred_len,
			out + out_pos, out_cap - out_pos);
		if (written < 0) return -1;
		out_pos += written;
	}

	if (in_len > 0)
	{
		int written = filter_tx_input(in, in_len,
			out + out_pos, out_cap - out_pos);
		if (written < 0) return -1;
		out_pos += written;
	}
	return out_pos;
}

int cl_b2f_handler::filter_rx_input(const char* in, int in_len, char* out, int out_cap)
{
	if (!initialized)
	{
		if (in_len > out_cap) return -1;
		memcpy(out, in, in_len);
		return in_len;
	}

	// Fail-closed (C3): a prior reroll DIVERGED from the sender's LZHUF encoding.
	// Ship NOTHING further this session so no divergent or framing-desynced bytes
	// reach the client; -1 is treated by the RX drain as "send 0 bytes this tick".
	if (reroll_poisoned)
		return -1;

	int out_pos = 0;
	int in_pos = 0;

	// Pre-detection: pass bytes through one at a time while shadow-scanning
	// for B2F SID.  When SID is detected, stop passthrough at the \r boundary
	// and fall through to the line parser for remaining bytes in this chunk.
	if (!b2f_detected)
	{
		for (; in_pos < in_len && !b2f_detected && out_pos < out_cap; in_pos++)
		{
			char c = in[in_pos];

			if (out_pos < out_cap)
				out[out_pos++] = c;

			if (c == '\r')
			{
				if (rx_line_pos > 0)
				{
					rx_line_buf[rx_line_pos] = '\0';
					if (parse_sid_line(rx_line_buf, rx_line_pos))
					{
						b2f_detected = true;
						state = B2F_SID_EXCHANGE;
						printf("[B2F-RX] SID detected: %.*s\n", rx_line_pos, rx_line_buf);
						fflush(stdout);
					}
					rx_line_pos = 0;
				}
			}
			else if (c != '\n')
			{
				if (rx_line_pos < B2F_LINE_BUF_SIZE - 1)
					rx_line_buf[rx_line_pos++] = c;
				else
					rx_line_pos = 0;
			}
		}

		if (!b2f_detected && in_pos < in_len)
		{
			int remaining = in_len - in_pos;
			if (remaining > B2F_DEFERRED_INPUT_SIZE || rx_deferred_input_len != 0)
			{
				fatal_error = true;
				return -1;
			}
			memcpy(rx_deferred_input, in + in_pos, remaining);
			rx_deferred_input_len = remaining;
			return out_pos;
		}

		if (!b2f_detected)
			return out_pos;

		// SID found mid-chunk: remaining bytes fall through to line parser
	}

	// B2F active: full line parser with state machine

	while (in_pos < in_len)
	{
		if (out_pos >= out_cap)
		{
			int remaining = in_len - in_pos;
			if (remaining > B2F_DEFERRED_INPUT_SIZE || rx_deferred_input_len != 0)
			{
				fatal_error = true;
				return -1;
			}
			memcpy(rx_deferred_input, in + in_pos, remaining);
			rx_deferred_input_len = remaining;
			break;
		}

		if (state == B2F_PAYLOAD_TRANSFER && current_proposer == PROPOSER_REMOTE)
		{
			int avail = in_len - in_pos;
			int consumed = 0;
			int written = process_rx_payload(in + in_pos, avail, out + out_pos, out_cap - out_pos, &consumed);
			if (written < 0) return -1;
			out_pos += written;
			in_pos += consumed;
			if (rx_pending_output_pos < rx_pending_output_len)
			{
				int remaining = in_len - in_pos;
				if (remaining > B2F_DEFERRED_INPUT_SIZE || rx_deferred_input_len != 0)
				{
					fatal_error = true;
					return -1;
				}
				if (remaining > 0)
				{
					memcpy(rx_deferred_input, in + in_pos, remaining);
					rx_deferred_input_len = remaining;
				}
				break;
			}
			if (consumed == 0 && written == 0)
			{
				fatal_error = true;
				return -1;
			}
		}
		else
		{
			char c = in[in_pos++];

			if (c == '\r')
			{
				rx_line_buf[rx_line_pos] = '\0';
				int written = process_rx_line(rx_line_buf, rx_line_pos, out + out_pos, out_cap - out_pos);
				if (written < 0) return -1;
				out_pos += written;
				rx_line_pos = 0;
			}
			else if (c != '\n')
			{
				if (rx_line_pos < B2F_LINE_BUF_SIZE - 1)
					rx_line_buf[rx_line_pos++] = c;
			}
		}
	}

	return out_pos;
}

int cl_b2f_handler::filter_rx(const char* in, int in_len, char* out, int out_cap)
{
	if (in_len < 0 || out_cap < 0 || (in_len > 0 && !in) || !out || fatal_error)
		return -1;
	if (in_len > 0 && has_pending_tx_work())
		return -1;

	int out_pos = drain_rx_pending(out, out_cap);
	if (rx_pending_output_pos < rx_pending_output_len)
		return in_len == 0 ? out_pos : -1;

	if (rx_deferred_input_len > 0)
	{
		if (in_len > 0)
			return -1;
		char deferred[B2F_DEFERRED_INPUT_SIZE];
		int deferred_len = rx_deferred_input_len;
		memcpy(deferred, rx_deferred_input, deferred_len);
		rx_deferred_input_len = 0;
		int written = filter_rx_input(deferred, deferred_len,
			out + out_pos, out_cap - out_pos);
		if (written < 0) return -1;
		out_pos += written;
	}

	if (in_len > 0)
	{
		int written = filter_rx_input(in, in_len,
			out + out_pos, out_cap - out_pos);
		if (written < 0) return -1;
		out_pos += written;
	}
	return out_pos;
}
