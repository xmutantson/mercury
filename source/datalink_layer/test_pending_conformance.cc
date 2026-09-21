// ============================================================================
// [PENDING-CONF] scanner-control PENDING conformance regression (in-process)
// ============================================================================
//
// Pairs with mercury/fact-documents/session-connect-handshake.md. Verifies the
// responder's scanner-control TCP status emissions against the observed peer
// (VARA HF) cadence:
//   1. PENDING is a BARE token (no callsign argument) and is emitted only once
//      the inbound START_CONNECTION CRC resolves to MYCALL. The callsign rides
//      on CONNECTED, never on PENDING.
//   2. A START_CONNECTION addressed to a callsign that is NOT MYCALL emits
//      NOTHING (no PENDING) -- the address-filtered silence.
//   3. A for-us PENDING that never reaches CONNECTED releases the scanning host
//      exactly once with CANCELPENDING + DISCONNECTED (single-owner, latched).
//   4. A session with no outstanding PENDING (already CONNECTED / never pending)
//      emits no release.
//
// The tests drive the PRODUCTION emit path -- process_control_responder() for
// the crc-match branch and rsp_emit_release() for the teardown release -- and
// capture the exact bytes written to tcp_socket_control via the existing test
// transmit seam (cl_tcp_socket::g_test_transmit_hook). The release state used in
// T3 is the latch SET by the real process_control_responder() in T1, so the
// release is exercised from a production-derived state, not a hand poke.
//
// FAIL-BEFORE / PASS-AFTER: with the pre-fix parameterized "PENDING <caller>\r"
// at the crc-match branch, T1 captures "PENDING TESTB\r" -> bare-token and
// single-PENDING assertions FAIL. With no release wiring, T3 captures nothing.
// Deterministic, no RF, no sockets. 0 = PASS, 1 = FAIL.
// ============================================================================

#include "datalink_layer/arq.h"
#include "datalink_layer/datalink_defines.h"
#include "common/common_defines.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

// --- captured control-socket byte stream (the "host" side of the wire) -------
static char  g_pconf_cap[4096];
static int   g_pconf_len = 0;

static void pconf_reset()
{
	g_pconf_len = 0;
	g_pconf_cap[0] = '\0';
}

static int pconf_capture_hook(const char* buf, int length)
{
	for(int i = 0; i < length && g_pconf_len < (int)sizeof(g_pconf_cap) - 1; i++)
		g_pconf_cap[g_pconf_len++] = buf[i];
	g_pconf_cap[g_pconf_len] = '\0';
	return length;   // model an app socket that accepts every byte
}

static int pconf_count_token(const char* tok)
{
	int n = 0;
	const char* p = g_pconf_cap;
	while((p = strstr(p, tok)) != NULL) { n++; p += 1; }
	return n;
}

int cl_arq_controller::test_pending_conformance()
{
	int failed = 0;
	auto check = [&](bool cond, const char* name) {
		printf("[TEST-PENDING-CONF] %s: %s | captured=%d bytes {%s}\n",
			cond ? "PASS" : "FAIL", name, g_pconf_len, g_pconf_cap);
		if(!cond) failed++;
	};

	// --- prime a RESPONDER instance (mirrors test_stream_offset setup) --------
	cl_telecom_system test_telecom;
	test_telecom.operation_mode = ARQ_MODE;
	test_telecom.load_configuration(CONFIG_0);

	// save the members we touch
	cl_telecom_system* saved_telecom = telecom_system;
	int         saved_link      = link_status;
	int         saved_conn      = connection_status;
	int         saved_role      = role;
	bool        saved_pm        = passive_monitor;
	int         saved_nb        = narrowband_enabled;
	std::string saved_call      = my_call_sign;
	int         saved_tcp_stat  = tcp_socket_control.status;
	bool        saved_pending   = pending_emitted;
	int         saved_nMessages = nMessages;
	int         saved_mdl       = max_data_length;
	int         saved_mml       = max_message_length;
	int         saved_mhl       = max_header_length;
	int (*saved_hook)(const char*, int) = cl_tcp_socket::g_test_transmit_hook;

	telecom_system      = &test_telecom;
	role                = RESPONDER;
	passive_monitor     = false;
	narrowband_enabled  = NO;
	my_call_sign        = "TESTA";
	nMessages           = 255;
	max_data_length     = 170;
	max_message_length  = 200;
	max_header_length   = 6;

	bool bufs_ok = (init_messages_buffers() == SUCCESSFUL);
	check(bufs_ok, "init_messages_buffers");
	if(!bufs_ok)
	{
		telecom_system = saved_telecom;
		return 1;
	}

	tcp_socket_control.status = TCP_STATUS_ACCEPTED;   // a host is attached
	cl_tcp_socket::g_test_transmit_hook = pconf_capture_hook;

	const unsigned char my_crc =
		(unsigned char)CRC8_calc((char*)my_call_sign.c_str(), my_call_sign.length());

	// ===================================================================== T1 =
	// for-MYCALL START_CONNECTION -> exactly one BARE PENDING, no callsign arg.
	pconf_reset();
	link_status        = LISTENING;
	connection_status  = RECEIVING;
	pending_emitted    = false;
	messages_control.data[0] = (char)START_CONNECTION;
	messages_control.data[1] = (char)my_crc;                       // addressed to us
	callsign_pack("TESTB", 5, &messages_control.data[2], 0);       // caller
	messages_control.length  = 7;
	messages_control.status  = RECEIVED;
	process_control_responder();
	check(strcmp(g_pconf_cap, "PENDING\r") == 0,
		"for-MYCALL START_CONNECTION emits exactly one BARE PENDING\\r");
	check(pconf_count_token("PENDING") == 1, "exactly one PENDING token");
	check(strstr(g_pconf_cap, "PENDING TESTB") == NULL,
		"no parameterized PENDING <caller>");
	check(pending_emitted == true, "pending_emitted latch set by production path");
	check(link_status == CONNECTION_RECEIVED, "advanced to CONNECTION_RECEIVED");

	// ===================================================================== T1b =
	// a duplicate/retry START_CONNECTION must NOT re-emit PENDING (single-shot).
	pconf_reset();
	messages_control.data[0] = (char)START_CONNECTION;
	messages_control.data[1] = (char)my_crc;
	callsign_pack("TESTB", 5, &messages_control.data[2], 0);
	messages_control.length  = 7;
	messages_control.status  = RECEIVED;
	process_control_responder();
	check(g_pconf_len == 0, "duplicate START_CONNECTION emits no second PENDING");

	// ===================================================================== T2 =
	// non-MYCALL START_CONNECTION -> ZERO emissions (address-filtered silence).
	pconf_reset();
	link_status        = LISTENING;
	connection_status  = RECEIVING;
	pending_emitted    = false;
	messages_control.data[0] = (char)START_CONNECTION;
	messages_control.data[1] = (char)(my_crc ^ 0xFF);             // NOT us
	callsign_pack("TESTC", 5, &messages_control.data[2], 0);
	messages_control.length  = 7;
	messages_control.status  = RECEIVED;
	process_control_responder();
	check(g_pconf_len == 0, "non-MYCALL START_CONNECTION emits NOTHING");
	check(pending_emitted == false, "no latch on non-MYCALL");
	check(link_status == LISTENING, "stays LISTENING on non-MYCALL");

	// ===================================================================== T3 =
	// a for-us PENDING that never reaches CONNECTED releases the scanner once.
	// Re-establish the real pending state via the production crc-match path,
	// then fire the production release owner (the fn E1/E3/E4 call).
	pconf_reset();
	link_status        = LISTENING;
	connection_status  = RECEIVING;
	pending_emitted    = false;
	messages_control.data[0] = (char)START_CONNECTION;
	messages_control.data[1] = (char)my_crc;
	callsign_pack("TESTB", 5, &messages_control.data[2], 0);
	messages_control.length  = 7;
	messages_control.status  = RECEIVED;
	process_control_responder();          // real PENDING -> pending_emitted=true
	pconf_reset();
	bool released = rsp_emit_release();    // production release owner
	check(released, "release fires for an outstanding for-us PENDING");
	check(strcmp(g_pconf_cap, "CANCELPENDING\rDISCONNECTED\r") == 0,
		"release = CANCELPENDING then DISCONNECTED");
	check(pending_emitted == false, "latch cleared by release");
	check(pconf_count_token("CANCELPENDING") == 1 && pconf_count_token("DISCONNECTED") == 1,
		"exactly one CANCELPENDING and one DISCONNECTED");

	// idempotency: a second teardown emits nothing more.
	pconf_reset();
	bool released2 = rsp_emit_release();
	check(!released2 && g_pconf_len == 0, "release is idempotent (no double-send)");

	// ===================================================================== T4 =
	// negative control: a session with no outstanding PENDING emits no release.
	pconf_reset();
	pending_emitted = false;
	bool released3 = rsp_emit_release();
	check(!released3 && strstr(g_pconf_cap, "CANCELPENDING") == NULL,
		"no CANCELPENDING when nothing is pending (connected/idle)");

	// --- restore --------------------------------------------------------------
	cl_tcp_socket::g_test_transmit_hook = saved_hook;
	tcp_socket_control.status = saved_tcp_stat;
	telecom_system      = saved_telecom;
	link_status         = saved_link;
	connection_status   = saved_conn;
	role                = saved_role;
	passive_monitor     = saved_pm;
	narrowband_enabled  = saved_nb;
	my_call_sign        = saved_call;
	pending_emitted     = saved_pending;
	nMessages           = saved_nMessages;
	max_data_length     = saved_mdl;
	max_message_length  = saved_mml;
	max_header_length   = saved_mhl;

	printf("[TEST-PENDING-CONF] %s (%d checks failed)\n",
		failed ? "FAIL" : "PASS", failed);
	fflush(stdout);
	return failed ? 1 : 0;
}
