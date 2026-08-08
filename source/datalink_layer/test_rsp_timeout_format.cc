// ============================================================================
// [RSP-TIMEOUT] diagnostic field-alignment regression (in-process, test-only)
// ============================================================================
//
// Pairs with mercury/fact-documents/data-flow-rsp-timeout.md. The RESPONDER
// branch of cl_arq_controller::calculate_receiving_timeout() computes the
// receive-window scalar
//     rsp_timeout = data_batch_size*message_transmission_time_ms
//                   + time_left_to_send_last_frame + ptt_on_delay_ms
// and reports it on the [RSP-TIMEOUT] diagnostic line. time_left_to_send_last_
// frame is declared double (a dead field, always 0.0), yet it was passed to a
// %d conversion. On the x86-64 SysV ABI a floating-point vararg travels in an
// XMM register, not an integer register, so the integer %d conversions consume
// one integer-register argument too few: every field printed after that arg is
// shifted one slot to the left and the final timeout=%d reads an unpushed stack
// slot (stale garbage, sometimes negative). The value actually armed into
// receiving_timeout is well-formed; only the diagnostic is wrong, and it mis-
// labels the armed value under the sack= field while timeout= shows garbage.
//
// THIS TEST drives the PRODUCTION calculate_receiving_timeout() on the RESPONDER
// path with distinctive inputs, captures the [RSP-TIMEOUT] line off real stdout,
// and asserts the reported fields align with the value that was armed:
//   - the printed ptt= field equals the real ptt_on_delay_ms, and
//   - the printed timeout= field equals the armed receiving_timeout, and
//   - the armed receiving_timeout equals the hand-computed sum.
//
// Inputs are chosen distinct and non-aliasing so a one-slot shift cannot pass by
// coincidence:
//   batch=10  msg_time=517  ptt=100  sack=false  time_left=0
//   armed rsp_timeout = 10*517 + 0 + 100 = 5270
//
// FAIL-BEFORE / PASS-AFTER CONTRACT (same test, production code toggled):
//   PRE-FIX  (double passed to %d): the fields shift left one slot -> the
//            printed ptt= reads sack_enabled?1:0 = 0 (not 100) and timeout=
//            reads an uninitialised stack slot (not 5270) -> FAIL.
//   POST-FIX ((int) cast at the call site): all six args are INTEGER-class, the
//            fields align -> printed ptt=100 and printed timeout=5270 = armed
//            receiving_timeout -> PASS.
//
// Deterministic, no RF, no sockets, runs in well under 1 s. 0 = PASS, 1 = FAIL.
// ============================================================================

#include "datalink_layer/arq.h"
#include "datalink_layer/datalink_defines.h"
#include "common/common_defines.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

// Portable file-descriptor primitives for the stdout capture (Linux/macOS +
// MSYS2/MinGW). Local to this TU so it stays self-contained.
#ifdef _WIN32
#include <io.h>
#define RTF_DUP(fd)     _dup(fd)
#define RTF_DUP2(a,b)   _dup2(a,b)
#define RTF_CLOSE(fd)   _close(fd)
#define RTF_FILENO(f)   _fileno(f)
#else
#include <unistd.h>
#define RTF_DUP(fd)     dup(fd)
#define RTF_DUP2(a,b)   dup2(a,b)
#define RTF_CLOSE(fd)   close(fd)
#define RTF_FILENO(f)   fileno(f)
#endif

int cl_arq_controller::test_rsp_timeout_format()
{
	// --- distinctive, non-aliasing inputs -----------------------------------
	const int   in_batch    = 10;
	const int   in_msg_time  = 517;
	const int   in_ptt       = 100;
	const bool  in_sack      = false;
	const int   expected_timeout = in_batch * in_msg_time + 0 + in_ptt;  // 5270

	// Prime the RESPONDER path directly (bypass load_configuration()/telecom_
	// system, which the RSP branch of calculate_receiving_timeout() does not
	// read). Assigning the private members inline mirrors the in-process
	// synthetic-fire pattern used by the sibling ARQ unit tests.
	this->role                          = RESPONDER;
	this->data_batch_size               = in_batch;
	this->message_transmission_time_ms  = in_msg_time;
	this->ptt_on_delay_ms               = in_ptt;
	this->sack_enabled                  = in_sack;
	this->time_left_to_send_last_frame  = 0.0;   // the dead double field
	this->receiving_timeout             = 0;     // make the store observable

	// --- capture stdout around the PRODUCTION call --------------------------
	char cap_buf[4096];
	cap_buf[0] = '\0';
	bool capture_ok = false;

	fflush(stdout);
	int stdout_fd = RTF_FILENO(stdout);
	int saved_fd  = RTF_DUP(stdout_fd);
	FILE* cap = tmpfile();
	if(saved_fd >= 0 && cap != NULL)
	{
		RTF_DUP2(RTF_FILENO(cap), stdout_fd);

		// THE production path under test.
		calculate_receiving_timeout();

		fflush(stdout);
		RTF_DUP2(saved_fd, stdout_fd);   // restore the real stdout
		RTF_CLOSE(saved_fd);

		fflush(cap);
		rewind(cap);
		size_t n = fread(cap_buf, 1, sizeof(cap_buf) - 1, cap);
		cap_buf[n] = '\0';
		fclose(cap);
		capture_ok = true;
	}
	else
	{
		if(saved_fd >= 0) { RTF_DUP2(saved_fd, stdout_fd); RTF_CLOSE(saved_fd); }
		if(cap != NULL) fclose(cap);
		// Fall back: run the call so the member is still armed (output visible).
		calculate_receiving_timeout();
	}

	// --- parse the [RSP-TIMEOUT] line ---------------------------------------
	// Assert the parser matched the expected line + all six fields before
	// trusting any statistic derived from it.
	int printed_batch = -1, printed_msg = -1, printed_time_left = -1;
	int printed_ptt = -1, printed_sack = -1, printed_timeout = -1;
	int matched = 0;
	const char* line = strstr(cap_buf, "[RSP-TIMEOUT]");
	if(line != NULL)
	{
		matched = sscanf(line,
			"[RSP-TIMEOUT] batch=%d msg_time=%d time_left=%d ptt=%d sack=%d -> timeout=%d",
			&printed_batch, &printed_msg, &printed_time_left,
			&printed_ptt, &printed_sack, &printed_timeout);
	}

	// --- assertions ---------------------------------------------------------
	// The armed member is well-formed regardless of the printf defect; the
	// defect lives only in the diagnostic. So we gate on BOTH: the member is
	// the expected sum, AND the diagnostic reports it faithfully.
	bool parse_ok      = capture_ok && (line != NULL) && (matched == 6);
	bool armed_ok      = (this->receiving_timeout == expected_timeout);
	bool ptt_field_ok  = parse_ok && (printed_ptt == in_ptt);
	bool timeout_field_ok = parse_ok && (printed_timeout == this->receiving_timeout);
	bool pass = parse_ok && armed_ok && ptt_field_ok && timeout_field_ok;

	printf("[TEST-RSP-TIMEOUT-FMT] %s: parsed_fields=%d/6 "
		"printed_batch=%d printed_msg=%d printed_time_left=%d printed_ptt=%d "
		"printed_sack=%d printed_timeout=%d | armed(receiving_timeout)=%d expected=%d "
		"(ptt_field_ok=%d timeout_field_ok=%d armed_ok=%d)\n",
		pass ? "PASS" : "FAIL", matched,
		printed_batch, printed_msg, printed_time_left, printed_ptt,
		printed_sack, printed_timeout,
		this->receiving_timeout, expected_timeout,
		ptt_field_ok ? 1 : 0, timeout_field_ok ? 1 : 0, armed_ok ? 1 : 0);
	fflush(stdout);

	return pass ? 0 : 1;
}
